/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 * AprilTag detection: raspiduino/apriltag-esp32 (ZIP install), family tag16h5.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <apriltag.h>
#include <tag16h5.h>

struct Detection {
  String label;
  float  x, y, w, h;
  float  confidence;
};

const char* WIFI_SSID     = "ExcavatorAP";
const char* WIFI_PASSWORD = "exc@vator123";
const char*    CONTROLLER_IP   = "192.168.4.1";
const uint16_t CONTROLLER_PORT = 80;
const int      REG_MAX_RETRIES    = 5;
const uint32_t REG_RETRY_DELAY_MS = 1000;

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

const int PIN_PUMP = 4;
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;

const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;
const uint8_t     JPEG_QUALITY      = 12;
const int         STREAM_DELAY_MS   = 0;

static void getFrameDims(framesize_t fs, int& w, int& h) {
  switch (fs) {
    case FRAMESIZE_QQVGA: w = 160;  h = 120;  break;
    case FRAMESIZE_QVGA:  w = 320;  h = 240;  break;
    case FRAMESIZE_VGA:   w = 640;  h = 480;  break;
    case FRAMESIZE_SVGA:  w = 800;  h = 600;  break;
    case FRAMESIZE_XGA:   w = 1024; h = 768;  break;
    case FRAMESIZE_SXGA:  w = 1280; h = 1024; break;
    case FRAMESIZE_UXGA:  w = 1600; h = 1200; break;
    default:              w = 640;  h = 480;  break;
  }
}

const uint32_t DETECTION_INTERVAL_MS = 500;
const uint32_t MARKER_CLEAR_MS       = DETECTION_INTERVAL_MS * 2;
const uint8_t  RED_R_MIN             = 160;
const uint8_t  RED_G_MAX             = 80;
const uint8_t  RED_B_MAX             = 80;
const int      RED_BLOB_MIN          = 30;

// Set true  = achromatic pre-filter (colored pixels → 255 before AprilTag).
// Set false = plain luminance only — recommended: AprilTag's internal adaptive
//             thresholding handles color noise better than pre-filtering did for ArUco.
const bool    GRAY_ACHROMATIC_FILTER = false;
const uint8_t GRAY_SAT_MAX           = 30;   // used only when GRAY_ACHROMATIC_FILTER=true

WebServer server(80);

apriltag_detector_t* atDetector = nullptr;
apriltag_family_t*   atFamily   = nullptr;

uint32_t lastDetectionMs = 0;
enum CamMode { MODE_SAFE, MODE_ARMED };
CamMode camMode = MODE_SAFE;
bool     pumpActive   = false;
uint32_t pumpLastSeen = 0;
int               lastDetectedMarkerId = -1;
volatile uint32_t markerDetectedAt     = 0;

// ── Debug gray frame (stashed by detection, served via /stream/gray) ──
static uint8_t*     dbgFrameBuf = nullptr;
static size_t       dbgFrameLen = 0;
static portMUX_TYPE dbgMux      = portMUX_INITIALIZER_UNLOCKED;

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
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = STREAM_FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
  } else {
    config.frame_size   = STREAM_FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY + 4;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("[CAM] Init failed: 0x%x\n", err); return false; }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, STREAM_FRAME_SIZE);
  s->set_quality(s, JPEG_QUALITY);
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);

  int w, h;
  getFrameDims(STREAM_FRAME_SIZE, w, h);
  Serial.printf("[CAM] Camera ready — %dx%d JPEG, fb_count=%d\n", w, h, psramFound() ? 2 : 1);
  return true;
}

// ════════════════════════════════════════════════════════════
//  AprilTag detector init
// ════════════════════════════════════════════════════════════
void initAprilTag() {
  atFamily   = tag16h5_create();
  atDetector = apriltag_detector_create();
  apriltag_detector_add_family(atDetector, atFamily);
  atDetector->quad_decimate = 2.0f;  // downsample 2x before quad detection — faster, fine at VGA
  atDetector->quad_sigma    = 0.0f;  // no blur — camera already soft enough
  atDetector->nthreads      = 1;     // ESP32 single-core detection task
  atDetector->debug         = 0;
  atDetector->refine_edges  = 1;     // sub-pixel edge refinement — worth it for bad image quality
  Serial.println("[AT] AprilTag detector ready — tag16h5");
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
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;
        if (fb->format == PIXFORMAT_JPEG) {
          String partHeader =
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + String(fb->len) + "\r\n\r\n";
          c->print(partHeader);
          c->write(fb->buf, fb->len);
          c->print("\r\n");
        }
        esp_camera_fb_return(fb);
        if (STREAM_DELAY_MS > 0) delay(STREAM_DELAY_MS);
      }
      Serial.println("[CAM] Stream client disconnected");
      delete c;
      vTaskDelete(NULL);
    },
    "stream_task", 8192, clientPtr, 1, NULL, 1
  );
}

// ── Gray debug stream — sources frames from dbgFrameBuf at ~2fps.
//  Zero impact on /stream — does not call esp_camera_fb_get().
//  Frame updated every DETECTION_INTERVAL_MS by runDetectionAndPush().
void handleGrayStream() {
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
        portENTER_CRITICAL(&dbgMux);
        size_t   len = dbgFrameLen;
        uint8_t* buf = (len > 0) ? (uint8_t*)malloc(len) : nullptr;
        if (buf) memcpy(buf, dbgFrameBuf, len);
        portEXIT_CRITICAL(&dbgMux);

        if (buf && len > 0) {
          String partHeader =
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + String(len) + "\r\n\r\n";
          c->print(partHeader);
          c->write(buf, len);
          c->print("\r\n");
          free(buf);
        }
        delay(500);
      }
      Serial.println("[CAM] Gray stream client disconnected");
      delete c;
      vTaskDelete(NULL);
    },
    "gray_stream", 4096, clientPtr, 1, NULL, 1
  );
}

void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "Capture failed"); return; }
  if (fb->format == PIXFORMAT_JPEG) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  } else {
    uint8_t* jpgBuf = nullptr; size_t jpgLen = 0;
    bool ok = frame2jpg(fb, JPEG_QUALITY, &jpgBuf, &jpgLen);
    esp_camera_fb_return(fb);
    if (!ok || jpgLen == 0) { server.send(500, "text/plain", "JPEG conversion failed"); return; }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send_P(200, "image/jpeg", (const char*)jpgBuf, jpgLen);
    free(jpgBuf);
  }
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

void handleDetectionStatus() {
  bool active = (lastDetectedMarkerId >= 0) &&
                ((millis() - markerDetectedAt) < MARKER_CLEAR_MS);
  StaticJsonDocument<128> doc;
  doc["marker_detected"] = active;
  doc["marker_id"]       = active ? lastDetectedMarkerId : -1;
  doc["ts"]              = millis();
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// ════════════════════════════════════════════════════════════
//  Gray helpers
// ════════════════════════════════════════════════════════════
inline uint8_t toGray(uint8_t r, uint8_t g, uint8_t b) {
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

inline bool isAchromatic(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t hi = max(r, max(g, b));
  uint8_t lo = min(r, min(g, b));
  return (hi - lo) < GRAY_SAT_MAX;
}

// ════════════════════════════════════════════════════════════
//  Combined detection + overlay push
//
//  Single fb_get() per cycle — no set_framesize() calls.
//  W/H read from fb directly (works in JPEG mode on this board);
//  getFrameDims() used as fallback if either is 0.
//
//  Red dot thresholds: r>160, g<80, b<80, blob>=30 (from bc9eced).
//  AprilTag (tag16h5) runs on grayscale derived from the same RGB888 buffer.
//  GRAY_ACHROMATIC_FILTER selects plain luma (recommended, false) vs
//  achromatic-filtered gray. AprilTag handles its own adaptive binarization.
//
//  After detection, gray[] is JPEG-encoded and stashed in dbgFrameBuf for
//  /stream/gray. Uses a fake camera_fb_t with PIXFORMAT_GRAYSCALE.
//  Critical section (dbgMux) guards the swap — safe against concurrent
//  handleGrayStream() reads on the same core.
// ════════════════════════════════════════════════════════════
void runDetectionAndPush() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[DET] fb_get failed"); return; }
  if (fb->format != PIXFORMAT_JPEG || fb->len == 0) {
    Serial.printf("[DET] Bad frame: format=%d len=%d\n", fb->format, fb->len);
    esp_camera_fb_return(fb);
    return;
  }

  int W = fb->width, H = fb->height;
  if (W == 0 || H == 0) getFrameDims(STREAM_FRAME_SIZE, W, H);
  Serial.printf("[DET] Frame %dx%d len=%d\n", W, H, fb->len);

  const size_t rgbLen = (size_t)W * H * 3;
  uint8_t* rgb = (uint8_t*)heap_caps_malloc(rgbLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb) rgb = (uint8_t*)malloc(rgbLen);
  if (!rgb) {
    Serial.println("[DET] RGB alloc failed");
    esp_camera_fb_return(fb);
    return;
  }

  bool decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  Serial.printf("[DET] fmt2rgb888: %s\n", decoded ? "OK" : "FAILED");
  if (!decoded) { free(rgb); return; }

  std::vector<Detection> detections;

  // ── Red dot detection — thresholds from bc9eced (confirmed working) ──
  {
    int bx1 = W, by1 = H, bx2 = 0, by2 = 0, blobCount = 0;
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++) {
        uint8_t r = rgb[(y*W+x)*3+0];
        uint8_t g = rgb[(y*W+x)*3+1];
        uint8_t b = rgb[(y*W+x)*3+2];
        if (r > RED_R_MIN && g < RED_G_MAX && b < RED_B_MAX) {
          if (x < bx1) bx1 = x; if (x > bx2) bx2 = x;
          if (y < by1) by1 = y; if (y > by2) by2 = y;
          blobCount++;
        }
      }
    Serial.printf("[RED] blob count=%d (min=%d)\n", blobCount, RED_BLOB_MIN);
    if (blobCount >= RED_BLOB_MIN) {
      Detection d;
      d.label = "red_dot";
      float cx = (bx1+bx2)/2.0f, cy = (by1+by2)/2.0f;
      float hw = (bx2-bx1)/2.0f*0.8f, hh = (by2-by1)/2.0f*0.8f;
      d.x=(cx-hw)/W; d.y=(cy-hh)/H; d.w=hw*2/W; d.h=hh*2/H;
      d.confidence = min(1.0f, (float)blobCount/500.0f);
      detections.push_back(d);
      Serial.printf("[RED] Detected @ (%.2f,%.2f) size %.2fx%.2f conf=%.2f\n",
                    d.x, d.y, d.w, d.h, d.confidence);
    }
  }

  // ── Build grayscale buffer ──
  uint8_t* gray = (uint8_t*)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!gray) gray = (uint8_t*)malloc((size_t)W * H);
  if (gray) {
    for (int i = 0; i < W * H; i++) {
      uint8_t r = rgb[i*3+0], g = rgb[i*3+1], b = rgb[i*3+2];
      gray[i] = (GRAY_ACHROMATIC_FILTER && !isAchromatic(r, g, b)) ? 255 : toGray(r, g, b);
    }

    // ── AprilTag detection ──
    image_u8_t aprilImg = { .width = W, .height = H, .stride = W, .buf = gray };
    zarray_t* results = apriltag_detector_detect(atDetector, &aprilImg);
    Serial.printf("[AT] Detections: %d\n", zarray_size(results));
    for (int i = 0; i < zarray_size(results); i++) {
      apriltag_detection_t* det;
      zarray_get(results, i, &det);
      // Bounding box from corner points
      float x0 = det->p[0][0], x1 = det->p[0][0];
      float y0 = det->p[0][1], y1 = det->p[0][1];
      for (int k = 1; k < 4; k++) {
        if (det->p[k][0] < x0) x0 = det->p[k][0];
        if (det->p[k][0] > x1) x1 = det->p[k][0];
        if (det->p[k][1] < y0) y0 = det->p[k][1];
        if (det->p[k][1] > y1) y1 = det->p[k][1];
      }
      lastDetectedMarkerId = det->id;
      markerDetectedAt     = millis();
      Detection d;
      d.label      = "marker_" + String(det->id);
      d.x          = x0 / W;  d.y = y0 / H;
      d.w          = (x1 - x0) / W;  d.h = (y1 - y0) / H;
      d.confidence = det->decision_margin / 100.0f;  // library returns 0–100 float
      detections.push_back(d);
      Serial.printf("[AT] ID %d @ (%.2f,%.2f) size %.2fx%.2f margin=%.1f\n",
                    det->id, d.x, d.y, d.w, d.h, det->decision_margin);
    }
    apriltag_detections_destroy(results);

    // ── Stash gray frame for /stream/gray debug view ──────────────────
    // Encodes gray[] buffer as JPEG using a fake camera_fb_t with
    // PIXFORMAT_GRAYSCALE. Swapped into dbgFrameBuf under dbgMux so
    // handleGrayStream() always sees a consistent pointer+length.
    {
      camera_fb_t fake;
      memset(&fake, 0, sizeof(fake));
      fake.buf    = gray;
      fake.len    = (size_t)W * H;
      fake.width  = W;
      fake.height = H;
      fake.format = PIXFORMAT_GRAYSCALE;
      uint8_t* jBuf = nullptr; size_t jLen = 0;
      if (frame2jpg(&fake, JPEG_QUALITY, &jBuf, &jLen) && jLen > 0) {
        portENTER_CRITICAL(&dbgMux);
        free(dbgFrameBuf);
        dbgFrameBuf = jBuf;
        dbgFrameLen = jLen;
        portEXIT_CRITICAL(&dbgMux);
        Serial.printf("[DBG] Gray frame stashed: %d bytes\n", (int)jLen);
      }
    }

    free(gray);
  }
  free(rgb);

  // ── POST to controller /overlay ──
  Serial.printf("[DET] Posting %d detection(s)\n", (int)detections.size());
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.createNestedArray("detections");
  for (auto& d : detections) {
    JsonObject o = arr.createNestedObject();
    o["label"]=d.label; o["x"]=d.x; o["y"]=d.y; o["w"]=d.w; o["h"]=d.h; o["confidence"]=d.confidence;
  }
  doc["ts"] = millis();
  String body; serializeJson(doc, body);

  HTTPClient http;
  http.begin("http://" + String(CONTROLLER_IP) + ":" + String(CONTROLLER_PORT) + "/overlay");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code < 0) Serial.printf("[DET] POST failed: %s\n", http.errorToString(code).c_str());
  else Serial.printf("[DET] POST /overlay -> HTTP %d\n", code);
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
  server.on("/stream",           HTTP_GET, handleStream);
  server.on("/stream/gray",      HTTP_GET, handleGrayStream);
  server.on("/snapshot",         HTTP_GET, handleSnapshot);
  server.on("/pump",             HTTP_GET, handlePump);
  server.on("/mode",             HTTP_GET, handleMode);
  server.on("/status",           HTTP_GET, handleStatus);
  server.on("/detection_status", HTTP_GET, handleDetectionStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Stream          : http://%s/stream\n",           WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Gray stream     : http://%s/stream/gray\n",      WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Snapshot        : http://%s/snapshot\n",         WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Detection status: http://%s/detection_status\n", WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Mode            : SAFE (boots safe, pump blocked)\n");
}

void loop() {
  server.handleClient();

  if (pumpActive && (millis() - pumpLastSeen) > PUMP_HOLD_TIMEOUT_MS) {
    Serial.println("[PUMP] Heartbeat timeout — auto-release");
    pumpOff();
  }

  if (!pumpActive && (millis() - lastDetectionMs) >= DETECTION_INTERVAL_MS) {
    lastDetectionMs = millis();
    runDetectionAndPush();
  }
}
