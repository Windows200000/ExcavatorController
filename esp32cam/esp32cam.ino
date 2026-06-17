/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 * AprilTag detection: raspiduino/apriltag-esp32 (ZIP install), family tag16h5.
 *
 * Pixel format: PIXFORMAT_GRAYSCALE — no JPEG decode needed for detection.
 * Stream: free-running snap_task captures frames as fast as sensor+encode allows.
 * Detection runs every DETECTION_INTERVAL_MS on the latest grayscale frame.
 *
 * Camera fb ownership: snap_task is the SOLE caller of esp_camera_fb_get/return.
 *   Runs free-running (no notify needed). Copies raw grayscale pixels into
 *   sharedGrayBuf (protected by grayMux), then JPEG-encodes for streaming.
 *   det_task reads sharedGrayBuf — no fb_get calls ever.
 *
 * OV2640 grayscale quirk: sensor outputs 477 active lines at VGA instead of 480
 *   (305280 bytes instead of 307200). Frame is accepted if
 *   fb->len >= fb->width * (fb->height - 4). Actual fb->height passed into
 *   sharedGrayH so AprilTag always sees correct dimensions.
 *
 * CPU profiling: printTopCpuTasks() runs every CPU_REPORT_INTERVAL_MS from loop().
 *   Uses uxTaskGetSystemState() — reports runtime % and core for 5 hardcoded tasks:
 *   det_task, snap_task, stream_task, loopTask, tiT.
 *   Percentages are cumulative since boot (not rolling window).
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

const int      PIN_PUMP             = 4;
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;

const framesize_t FRAME_SIZE            = FRAMESIZE_VGA;  // 640x480 (sensor outputs 640x477)
const uint8_t     JPEG_QUALITY          = 12;
const uint32_t    DETECTION_INTERVAL_MS = 500;
// SNAPSHOT_FACTOR kept for reference: snap_task is free-running, not timer-driven.
// At ~8fps the sensor+encode cycle self-paces to ~125ms per frame naturally.
// const uint32_t SNAPSHOT_FACTOR      = 4;
// const uint32_t SNAPSHOT_INTERVAL_MS = DETECTION_INTERVAL_MS / SNAPSHOT_FACTOR;

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

// ── Stashed JPEG for /stream — written by snap_task, read by stream_task ──
static uint8_t*     streamFrameBuf = nullptr;
static size_t       streamFrameLen = 0;
static portMUX_TYPE streamMux      = portMUX_INITIALIZER_UNLOCKED;

// ── Shared grayscale frame — written by snap_task, read by det_task ──
// snap_task is the SOLE owner of esp_camera_fb_get/return.
// det_task copies sharedGrayBuf under grayMux, then works on local copy.
static uint8_t*     sharedGrayBuf = nullptr;
static size_t       sharedGrayLen = 0;
static int          sharedGrayW   = 0;
static int          sharedGrayH   = 0;
static portMUX_TYPE grayMux       = portMUX_INITIALIZER_UNLOCKED;

// ── Task handles ──
TaskHandle_t detectionTaskHandle = NULL;
TaskHandle_t snapshotTaskHandle  = NULL;

// ════════════════════════════════════════════════════════════
//  CPU profiling — top-5 hardcoded tasks
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

  Serial.println("[CPU] ══ Top-5 Task CPU Report ══");
  for (int i = 0; i < 5; i++) {
    bool found = false;
    for (UBaseType_t j = 0; j < actualCount; j++) {
      if (strcmp(taskStatus[j].pcTaskName, CPU_WATCH_TOP5[i].exactName) == 0) {
        uint32_t pct10 = (uint32_t)(((uint64_t)taskStatus[j].ulRunTimeCounter * 1000ULL) / totalRunTime);
        BaseType_t core = taskStatus[j].xCoreID;
        if (core == tskNO_AFFINITY) {
          Serial.printf("[CPU]  %-12s  core=any  %2lu.%lu%%  ticks=%lu\n",
            CPU_WATCH_TOP5[i].exactName,
            (unsigned long)(pct10 / 10), (unsigned long)(pct10 % 10),
            (unsigned long)taskStatus[j].ulRunTimeCounter);
        } else {
          Serial.printf("[CPU]  %-12s  core=%ld    %2lu.%lu%%  ticks=%lu\n",
            CPU_WATCH_TOP5[i].exactName,
            (long)core,
            (unsigned long)(pct10 / 10), (unsigned long)(pct10 % 10),
            (unsigned long)taskStatus[j].ulRunTimeCounter);
        }
        found = true;
        break;
      }
    }
    if (!found) Serial.printf("[CPU]  %-12s  not found\n", CPU_WATCH_TOP5[i].exactName);
  }
  Serial.println("[CPU] ════════════════════════════");
  free(taskStatus);
}

// ════════════════════════════════════════════════════════════
//  Pump helpers
// ════════════════════════════════════════════════════════════
void pumpOn() {
  if (camMode == MODE_SAFE) { Serial.println("[PUMP] Blocked — SAFE mode"); return; }
  if (!pumpActive) {
    pinMode(PIN_PUMP, OUTPUT);
    digitalWrite(PIN_PUMP, HIGH);
    pumpActive = true;
    Serial.println("[PUMP] ON");
  }
  pumpLastSeen = millis();
}

void pumpOff() {
  if (pumpActive) {
    pinMode(PIN_PUMP, INPUT);
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
  config.fb_count     = 1;  // grayscale: fb_count=2 causes partial DMA frames on OV2640

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
//  Snapshot task — Core 0, priority 1 — FREE-RUNNING
//  SOLE owner of esp_camera_fb_get / esp_camera_fb_return.
//  Runs as fast as sensor + frame2jpg allows (~8fps at VGA grayscale).
//  No xTaskNotifyGive needed — loop() does NOT notify this task.
//
//  Frame acceptance: fb->len >= fb->width * (fb->height - 4)
//  Tolerates OV2640 grayscale quirk of 477 active lines (305280 bytes).
//  Actual fb->width/height stored in sharedGrayW/H for correct AprilTag dims.
//
//  On each valid frame:
//    1. Copies raw grayscale into sharedGrayBuf (grayMux) for det_task.
//    2. JPEG-encodes and stashes in streamFrameBuf (streamMux) for /stream.
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

    // ── 1. Stash raw grayscale for det_task ──
    portENTER_CRITICAL(&grayMux);
    if (sharedGrayLen != fb->len) {
      free(sharedGrayBuf);
      sharedGrayBuf = (uint8_t*)malloc(fb->len);
    }
    if (sharedGrayBuf) {
      memcpy(sharedGrayBuf, fb->buf, fb->len);
      sharedGrayLen = fb->len;
      sharedGrayW   = fb->width;
      sharedGrayH   = fb->height;
    }
    portEXIT_CRITICAL(&grayMux);

    // ── 2. JPEG encode for /stream ──
    uint8_t* jBuf = nullptr;
    size_t   jLen = 0;
    if (frame2jpg(fb, JPEG_QUALITY, &jBuf, &jLen) && jLen > 0) {
      portENTER_CRITICAL(&streamMux);
      free(streamFrameBuf);
      streamFrameBuf = jBuf;
      streamFrameLen = jLen;
      portEXIT_CRITICAL(&streamMux);
    }

    esp_camera_fb_return(fb);
  }
}

// ════════════════════════════════════════════════════════════
//  HTTP handlers
// ════════════════════════════════════════════════════════════

void handleStream() {
  WiFiClient client = server.client();
  String header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n";
  client.print(header);

  WiFiClient* clientPtr = new WiFiClient(client);
  xTaskCreatePinnedToCore(
    [](void* arg) {
      WiFiClient* c = (WiFiClient*)arg;
      while (c->connected()) {
        portENTER_CRITICAL(&streamMux);
        size_t   len = streamFrameLen;
        uint8_t* buf = (len > 0) ? (uint8_t*)malloc(len) : nullptr;
        if (buf) memcpy(buf, streamFrameBuf, len);
        portEXIT_CRITICAL(&streamMux);

        if (buf && len > 0) {
          String partHeader =
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + String(len) + "\r\n\r\n";
          c->print(partHeader);
          c->write(buf, len);
          c->print("\r\n");
          free(buf);
        } else {
          delay(10);
        }
      }
      Serial.println("[CAM] Stream client disconnected");
      delete c;
      vTaskDelete(NULL);
    },
    "stream_task", 4096, clientPtr,
    1, NULL, 1
  );
}

void handlePump() {
  String action = server.hasArg("action") ? server.arg("action") : "press";
  if (action == "press" || action == "hold") pumpOn();
  else if (action == "release") pumpOff();
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
  StaticJsonDocument<192> doc;
  doc["ip"]         = WiFi.localIP().toString();
  doc["uptime"]     = millis() / 1000;
  doc["heap"]       = ESP.getFreeHeap();
  doc["mode"]       = (camMode == MODE_ARMED) ? "armed" : "safe";
  doc["pumpActive"] = pumpActive;
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// ════════════════════════════════════════════════════════════
//  Detection + overlay push
//
//  Runs on Core 0 via det_task at priority 2 (highest).
//  Triggered via xTaskNotifyGive() from loop() every DETECTION_INTERVAL_MS.
//  Reads sharedGrayBuf (written by snap_task) — no fb_get calls.
//  quad_decimate=4.0: AprilTag processes 160x120 internally.
// ════════════════════════════════════════════════════════════
void runDetectionAndPush() {
  uint8_t* grayBuf = nullptr;
  size_t   grayLen = 0;
  int      W = 0, H = 0;

  portENTER_CRITICAL(&grayMux);
  if (sharedGrayBuf && sharedGrayLen > 0) {
    grayBuf = (uint8_t*)malloc(sharedGrayLen);
    if (grayBuf) {
      memcpy(grayBuf, sharedGrayBuf, sharedGrayLen);
      grayLen = sharedGrayLen;
      W       = sharedGrayW;
      H       = sharedGrayH;
    }
  }
  portEXIT_CRITICAL(&grayMux);

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

  DynamicJsonDocument doc(1024);
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
  pinMode(PIN_PUMP, INPUT);
  if (!initCamera()) {
    Serial.println("[BOOT] Camera init failed — halting");
    while (true) delay(1000);
  }
  initAprilTag();
  connectWiFi();
  registerWithController();

  // det_task — Core 0, priority 2 (highest).
  // Triggered via xTaskNotifyGive() from loop() every DETECTION_INTERVAL_MS.
  // Reads from sharedGrayBuf — no esp_camera_fb_get() calls.
  xTaskCreatePinnedToCore(
    [](void* arg) {
      for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        runDetectionAndPush();
      }
    },
    "det_task", 8192, NULL,
    2, &detectionTaskHandle, 0
  );

  // snap_task — Core 0, priority 1 — FREE-RUNNING.
  // SOLE owner of esp_camera_fb_get/return. No notify from loop().
  // Writes sharedGrayBuf for det_task and streamFrameBuf for /stream.
  // Stack 6144: free-running frame2jpg allocs need headroom.
  xTaskCreatePinnedToCore(
    snapshotTask,
    "snap_task", 6144, NULL,
    1, &snapshotTaskHandle, 0
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
