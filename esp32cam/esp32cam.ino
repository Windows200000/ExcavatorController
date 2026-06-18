/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 * AprilTag detection: raspiduino/apriltag-esp32 (ZIP install), family tag16h5.
 *
 * Pixel format: PIXFORMAT_GRAYSCALE — no JPEG decode needed for detection.
 * Stream: free-running snap_task captures frames as fast as sensor allows.
 * Detection runs every DETECTION_INTERVAL_MS on the latest grayscale frame.
 *
 * Camera fb ownership: snap_task is the SOLE caller of esp_camera_fb_get/return.
 *   Runs free-running. On each frame:
 *     - Try-locks streamGrayMux (no wait): copies gray into grayStreamBuf for stream_task.
 *     - Every DETECTION_INTERVAL_MS, try-locks detGrayMux (no wait): copies gray into
 *       grayDetBuf for det_task.
 *   snap_task NEVER blocks — both locks are try-only. Skips consumer if busy.
 *   det_task and stream_task read independent buffers — zero cross-consumer contention.
 *
 * OV2640 grayscale quirk: sensor outputs 477 active lines at VGA instead of 480
 *   (305280 bytes instead of 307200). Frame is accepted if
 *   fb->len >= fb->width * (fb->height - 4). Actual fb->height passed into
 *   grayDetH / grayStreamH so consumers always see correct dimensions.
 *
 * Shared buffer locking: streamGrayMux and detGrayMux are FreeRTOS mutexes.
 *   snap_task uses try-lock (timeout=0) — never blocks, skips frame for that
 *   consumer if busy. Consumers (stream_task, det_task) use 20ms timeout to
 *   copy out their local working buffer, then release immediately.
 *
 * Gray bufs (grayStreamBuf, grayDetBuf) allocated in PSRAM via ps_malloc.
 *   305KB each — won't fit internal SRAM. ESP32-CAM AI-Thinker has 4MB PSRAM;
 *   camera driver already uses it for DMA framebuffer automatically.
 *
 * Core layout:
 *   Core 0: snap_task (prio 2, highest) — free-running capture, gray memcpy only
 *           tiT (prio 18, lwIP/WiFi — preempts briefly but yields quickly)
 *   Core 1: det_task (prio 2, highest) — AprilTag detect + POST
 *           stream_task (prio 1) — JPEG encode + MJPEG write per connected client
 *           loopTask (prio 1) — Arduino loop()/handleClient()
 *
 * CPU profiling: printTopCpuTasks() runs every CPU_REPORT_INTERVAL_MS from loop().
 *   Uses uxTaskGetSystemState(). Per-core totals computed by summing ulRunTimeCounter
 *   across ALL tasks on each core — percentages reflect % of that core's capacity.
 *   tskNO_AFFINITY tasks are split half/half across both core totals.
 *   Percentages reflect last 10s window (delta between snapshots). First report
 *   prints a warm-up notice instead of percentages.
 *
 * Stream: max 1 concurrent client enforced via streamClientActive flag.
 *   A second /stream request is rejected with 503 while a client is connected.
 *   stream_task is notify-driven: snap_task calls xTaskNotifyGive(streamTaskHandle)
 *   after each successful grayStreamBuf write. stream_task wakes, copies gray buf,
 *   JPEG-encodes locally via fmt2jpg(), writes TCP.
 *   streamTaskHandle set on task start, cleared on disconnect.
 *
 * fb_count=2: sensor DMA double-buffered — sensor fills buf[1] while CPU processes buf[0].
 *   Previously forced to 1 due to suspected OV2640 partial-frame bug with grayscale+fb_count=2.
 *   Re-enabled: partial frames are already tolerated (fb->len >= w*(h-4) check).
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <apriltag.h>
#include <tag16h5.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>

struct Detection {
  String label;
  float  x, y, w, h;
  float  confidence;
};

const char*    WIFI_SSID            = "ExcavatorAP";
const char*    WIFI_PASSWORD        = "exc@vator123";
const char*    CONTROLLER_IP        = "192.168.4.1";
const uint16_t CONTROLLER_PORT      = 80;
const int      REG_MAX_RETRIES      = 5;
const uint32_t REG_RETRY_DELAY_MS   = 1000;
const float    MARKER_CONFIDENCE_CUTOFF = 50.0f;

#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

const int      PIN_PUMP             = 13;
const int      PIN_LED              = 4;
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;

const framesize_t FRAME_SIZE            = FRAMESIZE_VGA;  // 640x480 (sensor outputs 640x477)
const uint8_t     JPEG_QUALITY          = 12;
const uint32_t    DETECTION_INTERVAL_MS = 500;
// snap_task is free-running, not timer-driven.
// At ~8fps the sensor cycle self-paces to ~125ms per frame naturally.

const uint32_t    CPU_REPORT_INTERVAL_MS = 10000;

WebServer server(80);

apriltag_detector_t* atDetector = nullptr;
apriltag_family_t*   atFamily   = nullptr;

uint32_t lastDetectionMs  = 0;
uint32_t lastCpuReportMs  = 0;

enum CamMode { MODE_SAFE, MODE_ARMED };
CamMode camMode = MODE_SAFE;

bool     pumpActive   = false;
uint32_t pumpLastSeen = 0;

// ── Gray buf for stream_task — written by snap_task (try-lock), read by stream_task ──
// Allocated in PSRAM (ps_malloc) — 305KB won't fit internal SRAM.
// stream_task copies out under streamGrayMux, then fmt2jpg encodes outside lock.
static uint8_t*          grayStreamBuf     = nullptr;
static size_t            grayStreamLen     = 0;
static int               grayStreamW       = 0;
static int               grayStreamH       = 0;
static SemaphoreHandle_t streamGrayMux     = nullptr;

// ── Gray buf for det_task — written by snap_task (try-lock) every DETECTION_INTERVAL_MS ──
// Allocated in PSRAM (ps_malloc). det_task copies out under detGrayMux, works on local copy.
static uint8_t*          grayDetBuf        = nullptr;
static size_t            grayDetLen        = 0;
static int               grayDetW          = 0;
static int               grayDetH          = 0;
static SemaphoreHandle_t detGrayMux        = nullptr;

// ── Stream client guard — only 1 concurrent stream client allowed ──
static volatile bool     streamClientActive = false;

// ── Task handles ──
TaskHandle_t detectionTaskHandle = NULL;
TaskHandle_t snapshotTaskHandle  = NULL;
TaskHandle_t streamTaskHandle    = NULL;  // set by stream_task on start, cleared on disconnect

// ── Tracks when snap_task last wrote grayDetBuf ──
static uint32_t lastDetGrayWriteMs = 0;

// ── Rolling-window CPU state — delta between snapshots every 10s ──
static uint64_t lastSnapshotCoreTicks[2] = { 0, 0 };
static uint32_t lastSnapshotTaskTicks[5] = { 0, 0, 0, 0, 0 };
static bool     cpuSnapshotValid         = false;

// ════════════════════════════════════════════════════════════
//  CPU profiling — top-5 hardcoded tasks, per-core percentages
//
//  Algorithm:
//    Pass 1: iterate ALL tasks, accumulate per-core total ticks.
//            tskNO_AFFINITY tasks: add half their ticks to each core.
//    Pass 2: for each watched task, find it and compute delta ticks
//            vs previous snapshot, then:
//            pct = deltaTaskTicks * 1000 / deltaCoreTotal  (tenths of %)
//  This yields true % of that core's capacity over the last 10s window.
// ════════════════════════════════════════════════════════════
struct CpuWatchTask { const char* exactName; };

static const CpuWatchTask CPU_WATCH_TOP5[5] = {
  { "det_task"    },
  { "snap_task"   },
  { "stream_task" },
  { "loopTask"    },
  { "tiT"         }
};

void printTopCpuTasks() {
  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  if (taskCount == 0) { Serial.println("[CPU] No tasks"); return; }

  TaskStatus_t* taskStatus = (TaskStatus_t*)malloc(taskCount * sizeof(TaskStatus_t));
  if (!taskStatus) { Serial.println("[CPU] malloc failed"); return; }

  uint32_t totalRunTime = 0;
  UBaseType_t actualCount = uxTaskGetSystemState(taskStatus, taskCount, &totalRunTime);
  if (actualCount == 0 || totalRunTime == 0) {
    Serial.println("[CPU] Stats unavailable (totalRunTime=0 — configGENERATE_RUN_TIME_STATS may be off)");
    free(taskStatus);
    return;
  }

  // Pass 1: sum ticks per core
  uint64_t coreTicks[2] = { 0, 0 };
  for (UBaseType_t j = 0; j < actualCount; j++) {
    BaseType_t core = taskStatus[j].xCoreID;
    uint32_t   t    = taskStatus[j].ulRunTimeCounter;
    if (core == 0)               coreTicks[0] += t;
    else if (core == 1)          coreTicks[1] += t;
    else {                        // tskNO_AFFINITY — split half/half
      coreTicks[0] += t / 2;
      coreTicks[1] += t / 2;
    }
  }

  Serial.println("[CPU] ══ Per-Core CPU Report ══");
  Serial.printf("[CPU]  Core 0 total ticks: %llu\n", coreTicks[0]);
  Serial.printf("[CPU]  Core 1 total ticks: %llu\n", coreTicks[1]);

  if (!cpuSnapshotValid) {
    Serial.println("[CPU]  (warming up — no delta yet)");
    lastSnapshotCoreTicks[0] = coreTicks[0];
    lastSnapshotCoreTicks[1] = coreTicks[1];
    for (int i = 0; i < 5; i++) {
      for (UBaseType_t j = 0; j < actualCount; j++) {
        if (strcmp(taskStatus[j].pcTaskName, CPU_WATCH_TOP5[i].exactName) == 0) {
          lastSnapshotTaskTicks[i] = taskStatus[j].ulRunTimeCounter;
          break;
        }
      }
    }
    cpuSnapshotValid = true;
    free(taskStatus);
    return;
  }

  uint64_t deltaCoreTicks[2] = {
    coreTicks[0] - lastSnapshotCoreTicks[0],
    coreTicks[1] - lastSnapshotCoreTicks[1]
  };

  // Pass 2: report watched tasks using delta ticks
  for (int i = 0; i < 5; i++) {
    bool found = false;
    for (UBaseType_t j = 0; j < actualCount; j++) {
      if (strcmp(taskStatus[j].pcTaskName, CPU_WATCH_TOP5[i].exactName) == 0) {
        BaseType_t core       = taskStatus[j].xCoreID;
        uint32_t   deltaTicks = taskStatus[j].ulRunTimeCounter - lastSnapshotTaskTicks[i];
        uint64_t   coreTotal  = (core == 0) ? deltaCoreTicks[0]
                              : (core == 1) ? deltaCoreTicks[1]
                              : (deltaCoreTicks[0] + deltaCoreTicks[1]);
        uint32_t pct10 = (coreTotal > 0)
                       ? (uint32_t)(((uint64_t)deltaTicks * 1000ULL) / coreTotal)
                       : 0;
        const char* coreStr = (core == 0) ? "core=0" : (core == 1) ? "core=1" : "core=any";
        Serial.printf("[CPU]  %-12s  %s  %3lu.%lu%%/core   ticks=%lu\n",
          CPU_WATCH_TOP5[i].exactName,
          coreStr,
          (unsigned long)(pct10 / 10), (unsigned long)(pct10 % 10),
          (unsigned long)deltaTicks);
        found = true;
        break;
      }
    }
    if (!found) Serial.printf("[CPU]  %-12s  not found\n", CPU_WATCH_TOP5[i].exactName);
  }
  Serial.println("[CPU] ════════════════════════════");

  // Save snapshot for next window
  lastSnapshotCoreTicks[0] = coreTicks[0];
  lastSnapshotCoreTicks[1] = coreTicks[1];
  for (int i = 0; i < 5; i++) {
    for (UBaseType_t j = 0; j < actualCount; j++) {
      if (strcmp(taskStatus[j].pcTaskName, CPU_WATCH_TOP5[i].exactName) == 0) {
        lastSnapshotTaskTicks[i] = taskStatus[j].ulRunTimeCounter;
        break;
      }
    }
  }
  free(taskStatus);
}

// ════════════════════════════════════════════════════════════
//  Pump helpers
// ════════════════════════════════════════════════════════════
void pumpOn() {
  if (camMode == MODE_SAFE) { Serial.println("[PUMP] Blocked — SAFE mode"); return; }
  if (!pumpActive) {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    digitalWrite(PIN_PUMP, HIGH);
    pumpActive = true;
    Serial.println("[PUMP] ON");
  }
  pumpLastSeen = millis();
}

void pumpOff() {
  if (pumpActive) {
    pinMode(PIN_LED, INPUT);
    digitalWrite(PIN_PUMP, LOW);   // LOW → gate off → pump stops
    pumpActive = false;
    Serial.println("[PUMP] OFF");
  }
}

// ════════════════════════════════════════════════════════════
//  Camera init
// ════════════════════════════════════════════════════════════
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // raw gray pixels — no decode needed
  config.frame_size   = FRAME_SIZE;
  config.fb_count     = 2;  // DMA double-buffer: sensor fills buf[1] while CPU processes buf[0]
                            // Partial frames already tolerated via fb->len >= w*(h-4) check

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("[CAM] Init failed: 0x%x\n", err); return false; }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAME_SIZE);
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);

  Serial.printf("[CAM] Camera ready — 640x480 GRAYSCALE (sensor outputs 477 lines), fb_count=%d\n", config.fb_count);
  return true;
}

// ════════════════════════════════════════════════════════════
//  AprilTag detector init
// ════════════════════════════════════════════════════════════
void initAprilTag() {
  atFamily   = tag16h5_create();
  atDetector = apriltag_detector_create();
  apriltag_detector_add_family(atDetector, atFamily);
  atDetector->quad_decimate = 4.0f;
  atDetector->quad_sigma    = 0.0f;
  atDetector->nthreads      = 1;
  atDetector->debug         = 0;
  atDetector->refine_edges  = 1;
  Serial.println("[AT] AprilTag detector ready — tag16h5, quad_decimate=4.0");
}

// ════════════════════════════════════════════════════════════
//  Snapshot task — Core 0, priority 2 (highest on Core 0) — FREE-RUNNING
//  SOLE owner of esp_camera_fb_get / esp_camera_fb_return.
//  Runs as fast as sensor allows (~8fps at VGA grayscale).
//  No xTaskNotifyGive needed — loop() does NOT notify this task.
//
//  Frame acceptance: fb->len >= fb->width * (fb->height - 4)
//  Tolerates OV2640 grayscale quirk of 477 active lines (305280 bytes).
//  Dimensions stored in grayStreamW/H and grayDetW/H for consumers.
//
//  On each valid frame:
//    1. Try-lock streamGrayMux (no wait): copy gray → grayStreamBuf, notify stream_task.
//       Skip silently if stream_task still reading (lock busy).
//    2. Every DETECTION_INTERVAL_MS: try-lock detGrayMux (no wait): copy gray → grayDetBuf.
//       Skip silently if det_task still reading. det_task triggered by loop() separately.
//
//  snap_task NEVER blocks. fb_return always happens immediately.
// ════════════════════════════════════════════════════════════
void snapshotTask(void*) {
  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[SNAP] fb_get failed");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Accept frames within 4 lines of expected — tolerates OV2640 477-line output
    if (fb->format != PIXFORMAT_GRAYSCALE ||
        fb->len < (size_t)(fb->width * (fb->height - 4))) {
      Serial.printf("[SNAP] Bad frame: format=%d len=%u w=%d h=%d\n",
                    fb->format, fb->len, fb->width, fb->height);
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // ── 1. Try-lock streamGrayMux — skip frame for stream if busy ──
    if (xSemaphoreTake(streamGrayMux, 0) == pdTRUE) {
      if (grayStreamLen != fb->len) {
        free(grayStreamBuf);
        grayStreamBuf = (uint8_t*)ps_malloc(fb->len);
      }
      if (grayStreamBuf) {
        memcpy(grayStreamBuf, fb->buf, fb->len);
        grayStreamLen = fb->len;
        grayStreamW   = fb->width;
        grayStreamH   = fb->height;
      }
      xSemaphoreGive(streamGrayMux);
      // Notify stream_task — new frame ready
      TaskHandle_t sh = streamTaskHandle;
      if (sh) xTaskNotifyGive(sh);
    }

    // ── 2. Every DETECTION_INTERVAL_MS: try-lock detGrayMux — skip if det_task busy ──
    uint32_t now = millis();
    if ((now - lastDetGrayWriteMs) >= DETECTION_INTERVAL_MS) {
      if (xSemaphoreTake(detGrayMux, 0) == pdTRUE) {
        if (grayDetLen != fb->len) {
          free(grayDetBuf);
          grayDetBuf = (uint8_t*)ps_malloc(fb->len);
        }
        if (grayDetBuf) {
          memcpy(grayDetBuf, fb->buf, fb->len);
          grayDetLen = fb->len;
          grayDetW   = fb->width;
          grayDetH   = fb->height;
        }
        xSemaphoreGive(detGrayMux);
        lastDetGrayWriteMs = now;
      }
      // If lock busy: det_task still running — skip this detection frame silently.
      // loop() will still call xTaskNotifyGive(detectionTaskHandle) on schedule;
      // det_task will use the previously written grayDetBuf on next wake.
    }

    esp_camera_fb_return(fb);
  }
}

// ════════════════════════════════════════════════════════════
//  HTTP handlers
// ════════════════════════════════════════════════════════════

void handleStream() {
  if (streamClientActive) {
    server.send(503, "text/plain", "Stream busy — only 1 client allowed");
    Serial.println("[CAM] Stream rejected — client already connected");
    return;
  }

  WiFiClient* clientPtr = new WiFiClient(server.client());

  String header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n";
  clientPtr->print(header);

  streamClientActive = true;

  xTaskCreatePinnedToCore(
    [](void* arg) {
      WiFiClient* c = (WiFiClient*)arg;
      // Register handle so snap_task can notify us
      streamTaskHandle = xTaskGetCurrentTaskHandle();
      while (c->connected()) {
        // Block until snap_task signals a new frame (200ms watchdog for disconnect detection)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

        // Copy gray buf locally under lock, then encode outside lock
        uint8_t* localGray = nullptr;
        size_t   localLen  = 0;
        int      localW    = 0, localH = 0;
        if (xSemaphoreTake(streamGrayMux, pdMS_TO_TICKS(20)) == pdTRUE) {
          if (grayStreamBuf && grayStreamLen > 0) {
            localGray = (uint8_t*)malloc(grayStreamLen);
            if (localGray) {
              memcpy(localGray, grayStreamBuf, grayStreamLen);
              localLen = grayStreamLen;
              localW   = grayStreamW;
              localH   = grayStreamH;
            }
          }
          xSemaphoreGive(streamGrayMux);
        }

        // JPEG encode outside lock — fmt2jpg on raw gray buf, can take 30-50ms
        if (localGray && localLen > 0 && localW > 0 && localH > 0) {
          uint8_t* jBuf = nullptr;
          size_t   jLen = 0;
          if (fmt2jpg(localGray, localLen, localW, localH,
                      PIXFORMAT_GRAYSCALE, JPEG_QUALITY, &jBuf, &jLen) && jLen > 0) {
            String partHeader =
              "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
              + String(jLen) + "\r\n\r\n";
            c->print(partHeader);
            c->write(jBuf, jLen);
            c->print("\r\n");
            free(jBuf);
          }
          free(localGray);
        }
      }
      Serial.println("[CAM] Stream client disconnected");
      streamTaskHandle = NULL;
      delete c;
      streamClientActive = false;
      vTaskDelete(NULL);
    },
    "stream_task", 8192, clientPtr,  // 8192: fmt2jpg needs stack headroom
    1, NULL, 1
  );
}

void handlePump() {
  String action = server.hasArg("action") ? server.arg("action") : "press";
  if (action == "press")         pumpOn();
  else if (action == "hold")   { if (pumpActive) pumpLastSeen = millis(); }  // ignore if not active — prevents stale queued hold re-firing pump after release
  else if (action == "release")  pumpOff();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", pumpActive ? "on" : "off");
}

void handleMode() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "Missing set"); return; }
  String m = server.arg("set");
  if (m == "armed")     { camMode = MODE_ARMED; Serial.println("[MODE] ARMED"); }
  else if (m == "safe") { camMode = MODE_SAFE; pumpOff(); Serial.println("[MODE] SAFE"); }
  else                  { server.send(400, "text/plain", "Unknown mode"); return; }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", camMode == MODE_ARMED ? "armed" : "safe");
}

void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["ip"]         = WiFi.localIP().toString();
  doc["uptime"]     = millis() / 1000;
  doc["heap"]       = ESP.getFreeHeap();
  doc["psram"]      = ESP.getFreePsram();
  doc["mode"]       = (camMode == MODE_ARMED) ? "armed" : "safe";
  doc["pumpActive"] = pumpActive;
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// ════════════════════════════════════════════════════════════
//  Detection + overlay push
//
//  Runs on Core 1 via det_task at priority 2 (highest on Core 1).
//  Triggered via xTaskNotifyGive() from loop() every DETECTION_INTERVAL_MS.
//  Reads grayDetBuf (written by snap_task on Core 0) — no fb_get calls.
//  quad_decimate=4.0: AprilTag processes 160x120 internally.
//
//  StaticJsonDocument<1024> used — stack-allocated, no heap churn per cycle.
// ════════════════════════════════════════════════════════════
void runDetectionAndPush() {
  uint8_t* grayBuf = nullptr;
  size_t   grayLen = 0;
  int      W = 0, H = 0;

  if (xSemaphoreTake(detGrayMux, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (grayDetBuf && grayDetLen > 0) {
      grayBuf = (uint8_t*)malloc(grayDetLen);
      if (grayBuf) {
        memcpy(grayBuf, grayDetBuf, grayDetLen);
        grayLen = grayDetLen;
        W       = grayDetW;
        H       = grayDetH;
      }
    }
    xSemaphoreGive(detGrayMux);
  } else {
    Serial.println("[DET] detGrayMux timeout — skipping detection cycle");
    return;
  }

  if (!grayBuf || grayLen == 0 || W == 0 || H == 0) {
    Serial.println("[DET] No frame available yet");
    free(grayBuf);
    return;
  }

  image_u8_t aprilImg = { .width = W, .height = H, .stride = W, .buf = grayBuf };
  zarray_t* results = apriltag_detector_detect(atDetector, &aprilImg);

  std::vector<Detection> detections;
  for (int i = 0; i < zarray_size(results); i++) {
    apriltag_detection_t* det;
    zarray_get(results, i, &det);
    float x0 = det->p[0][0], x1 = det->p[0][0];
    float y0 = det->p[0][1], y1 = det->p[0][1];
    for (int k = 1; k < 4; k++) {
      if (det->p[k][0] < x0) x0 = det->p[k][0];
      if (det->p[k][0] > x1) x1 = det->p[k][0];
      if (det->p[k][1] < y0) y0 = det->p[k][1];
      if (det->p[k][1] > y1) y1 = det->p[k][1];
    }
    float confidence = det->decision_margin;
    if (confidence < MARKER_CONFIDENCE_CUTOFF) {
      if (confidence > 5.0f) {
        Serial.printf("[AT] SKIPPING ID %d confidence=%.1f%% CUTOFF=%.1f\n",
                      det->id, confidence, MARKER_CONFIDENCE_CUTOFF);
      }
    } else {
      Detection d;
      d.label      = "marker_" + String(det->id);
      d.x          = x0 / W;  d.y = y0 / H;
      d.w          = (x1 - x0) / W;  d.h = (y1 - y0) / H;
      d.confidence = confidence / 100.0f;
      detections.push_back(d);
      Serial.printf("[AT] ID %d @ (%.2f,%.2f) size %.2fx%.2f confidence=%.1f%%\n",
                    det->id, d.x, d.y, d.w, d.h, confidence);
    }
  }
  apriltag_detections_destroy(results);
  free(grayBuf);

  // StaticJsonDocument — stack-allocated, no heap alloc per detection cycle
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.createNestedArray("detections");
  for (auto& d : detections) {
    JsonObject o = arr.createNestedObject();
    o["label"]=d.label; o["x"]=d.x; o["y"]=d.y; o["w"]=d.w; o["h"]=d.h; o["confidence"]=d.confidence;
  }
  String body; serializeJson(doc, body);

  HTTPClient http;
  http.begin("http://" + String(CONTROLLER_IP) + ":" + String(CONTROLLER_PORT) + "/overlay");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code < 0) Serial.printf("[DET] POST failed: %s\n", http.errorToString(code).c_str());
  http.end();
}

// ════════════════════════════════════════════════════════════
//  WiFi helpers
// ════════════════════════════════════════════════════════════
void connectWiFi() {
  Serial.printf("[WIFI] Connecting to '%s'", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t > 15000) { Serial.println("\n[WIFI] Timeout — restarting"); ESP.restart(); }
    delay(500); Serial.print(".");
  }
  Serial.printf("\n[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

void registerWithController() {
  String url  = "http://" + String(CONTROLLER_IP) + ":" + String(CONTROLLER_PORT) + "/register";
  String body = "{\"ip\":\"" + WiFi.localIP().toString() + "\"}";
  for (int attempt = 1; attempt <= REG_MAX_RETRIES; attempt++) {
    Serial.printf("[REG] Registering (attempt %d/%d)...\n", attempt, REG_MAX_RETRIES);
    HTTPClient http; http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body); http.end();
    if (code == 200) { Serial.printf("[REG] Registered at %s\n", WiFi.localIP().toString().c_str()); return; }
    Serial.printf("[REG] Failed (HTTP %d) — retrying in %dms\n", code, REG_RETRY_DELAY_MS);
    delay(REG_RETRY_DELAY_MS);
  }
  Serial.println("[REG] Registration failed after all retries.");
}

// ════════════════════════════════════════════════════════════
//  Setup & loop
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32-CAM starting");
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);

  // Create mutexes before any task that uses them
  streamGrayMux = xSemaphoreCreateMutex();
  detGrayMux    = xSemaphoreCreateMutex();
  if (!streamGrayMux || !detGrayMux) {
    Serial.println("[BOOT] Mutex creation failed — halting");
    while (true) delay(1000);
  }

  if (!initCamera()) {
    Serial.println("[BOOT] Camera init failed — halting");
    while (true) delay(1000);
  }
  initAprilTag();
  connectWiFi();
  registerWithController();

  // det_task — Core 1, priority 2 (highest on Core 1).
  // Triggered via xTaskNotifyGive() from loop() every DETECTION_INTERVAL_MS.
  // Reads from grayDetBuf written by snap_task on Core 0 — no fb_get calls.
  xTaskCreatePinnedToCore(
    [](void* arg) {
      for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        runDetectionAndPush();
      }
    },
    "det_task", 8192, NULL,
    2, &detectionTaskHandle, 1
  );

  // snap_task — Core 0, priority 2 (highest on Core 0) — FREE-RUNNING.
  // SOLE owner of esp_camera_fb_get/return. Never blocks — try-lock only.
  // Writes grayStreamBuf (every frame) and grayDetBuf (every 500ms).
  // Notifies streamTaskHandle after each grayStreamBuf write.
  // Stack 4096: gray memcpy only, no frame2jpg.
  xTaskCreatePinnedToCore(
    snapshotTask,
    "snap_task", 4096, NULL,
    2, &snapshotTaskHandle, 0
  );

  server.on("/stream", HTTP_GET, handleStream);
  server.on("/pump",   HTTP_GET, handlePump);
  server.on("/mode",   HTTP_GET, handleMode);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Stream  : http://%s/stream\n",  WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Mode    : SAFE (boots safe, pump blocked)\n");
  Serial.printf("[BOOT] Detection interval: %dms\n", DETECTION_INTERVAL_MS);
  Serial.printf("[BOOT] Free heap: %u  Free PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());
  Serial.println("[BOOT] Core layout: Core0=snap_task(p2) | Core1=det_task(p2)+stream_task(p1)+loopTask(p1)");
}

void loop() {
  server.handleClient();

  if (pumpActive && (millis() - pumpLastSeen) > PUMP_HOLD_TIMEOUT_MS) {
    Serial.println("[PUMP] Heartbeat timeout — auto-release");
    pumpOff();
  }

  uint32_t now = millis();

  if ((now - lastCpuReportMs) >= CPU_REPORT_INTERVAL_MS) {
    lastCpuReportMs = now;
    printTopCpuTasks();
  }

  if ((now - lastDetectionMs) >= DETECTION_INTERVAL_MS) {
    lastDetectionMs = now;
    if (detectionTaskHandle) xTaskNotifyGive(detectionTaskHandle);
  }
}
