/*
 * ============================================================
 *  EXCAVATOR CAM  —  ESP32-CAM sketch
 *  Role   : WiFi client → connects to controller AP
 *           Streams MJPEG  → GET  /stream
 *           Single frame   → GET  /snapshot
 *           Image proc.    → runs on-device, POSTs overlay
 *                            JSON to controller  /overlay
 * ============================================================
 *
 *  CHANGES vs. v1:
 *   - Added registerWithController() — cam now POSTs its IP to
 *     /register after WiFi connect (was missing in v1 setup!)
 *   - Registration retries up to 5 times on failure
 * ============================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
//  WiFi  (controller is the AP)
// ─────────────────────────────────────────────
const char* WIFI_SSID       = "ExcavatorAP";
const char* WIFI_PASSWORD   = "exc@vator123";

// ─────────────────────────────────────────────
//  Controller address (AP default gateway)
// ─────────────────────────────────────────────
const char*    CONTROLLER_IP   = "192.168.4.1";
const uint16_t CONTROLLER_PORT = 80;

// ─────────────────────────────────────────────
//  Registration retry settings
// ─────────────────────────────────────────────
const int     REG_MAX_RETRIES   = 5;
const uint32_t REG_RETRY_DELAY_MS = 1000;

// ─────────────────────────────────────────────
//  Camera  (AI-Thinker / Joy-IT ESP32-CAM pinout)
// ─────────────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1   // tied to EN
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

// ─────────────────────────────────────────────
//  Stream settings
// ─────────────────────────────────────────────
// Resolution for the MJPEG stream sent to the browser.
// Keep low for smooth remote viewing; snapshot uses full res.
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_QVGA;   // 320×240
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;    // 640×480
const uint8_t     JPEG_QUALITY      = 12;   // 0 best … 63 worst; 10-15 sweet spot
const int         STREAM_DELAY_MS   = 0;    // extra inter-frame delay (ms)

// ─────────────────────────────────────────────
//  Image processing
// ─────────────────────────────────────────────
// How often to run detection and push overlay to controller (ms).
const uint32_t DETECTION_INTERVAL_MS = 500;

// ─────────────────────────────────────────────
//  Web server on the cam ESP32
// ─────────────────────────────────────────────
WebServer server(80);

// ─────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────
uint32_t lastDetectionMs = 0;

// ════════════════════════════════════════════════════════════
//  Camera init
// ════════════════════════════════════════════════════════════
bool initCamera() {
  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = CAM_PIN_D0;
  config.pin_d1        = CAM_PIN_D1;
  config.pin_d2        = CAM_PIN_D2;
  config.pin_d3        = CAM_PIN_D3;
  config.pin_d4        = CAM_PIN_D4;
  config.pin_d5        = CAM_PIN_D5;
  config.pin_d6        = CAM_PIN_D6;
  config.pin_d7        = CAM_PIN_D7;
  config.pin_xclk      = CAM_PIN_XCLK;
  config.pin_pclk      = CAM_PIN_PCLK;
  config.pin_vsync     = CAM_PIN_VSYNC;
  config.pin_href      = CAM_PIN_HREF;
  config.pin_sscb_sda  = CAM_PIN_SIOD;
  config.pin_sscb_scl  = CAM_PIN_SIOC;
  config.pin_pwdn      = CAM_PIN_PWDN;
  config.pin_reset     = CAM_PIN_RESET;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;

  // Use PSRAM if available for larger frame buffers
  if (psramFound()) {
    config.frame_size   = SNAP_FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
  } else {
    config.frame_size   = STREAM_FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY + 4;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  // Apply stream resolution after init
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, STREAM_FRAME_SIZE);
  s->set_quality(s, JPEG_QUALITY);
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);

  Serial.println("[CAM] Camera ready");
  return true;
}

// ════════════════════════════════════════════════════════════
//  HTTP handlers
// ════════════════════════════════════════════════════════════

// -- /stream  (MJPEG)
void handleStream() {
  WiFiClient client = server.client();

  String header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n";
  client.print(header);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[CAM] Frame capture failed");
      break;
    }

    String partHeader =
      "--frame\r\n"
      "Content-Type: image/jpeg\r\n"
      "Content-Length: " + String(fb->len) + "\r\n\r\n";

    client.print(partHeader);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    if (STREAM_DELAY_MS > 0) delay(STREAM_DELAY_MS);
  }
  Serial.println("[CAM] Stream client disconnected");
}

// -- /snapshot  (single JPEG, higher res)
void handleSnapshot() {
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, SNAP_FRAME_SIZE);

  camera_fb_t* fb = esp_camera_fb_get();
  s->set_framesize(s, STREAM_FRAME_SIZE);   // restore stream resolution

  if (!fb) { server.send(500, "text/plain", "Capture failed"); return; }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// -- /status  (JSON: IP, uptime, heap)
void handleStatus() {
  StaticJsonDocument<128> doc;
  doc["ip"]      = WiFi.localIP().toString();
  doc["uptime"]  = millis() / 1000;
  doc["heap"]    = ESP.getFreeHeap();
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// ════════════════════════════════════════════════════════════
//  Image processing  (stub — extend here)
//
//  This runs on the cam ESP32. Capture a frame, analyse it,
//  build a list of detections (label, bounding box in
//  0..1 normalised coords), then POST to the controller.
//  The browser canvas overlay reads those boxes from /overlay.
//
//  To add detection logic:
//   1. Capture a frame (fb->buf, fb->len, fb->width, fb->height)
//   2. Run your algorithm (edge detection, blob tracking,
//      TFLite model inference, ArUco markers, colour thresholding…)
//   3. Populate the detections vector with Detection structs
//   4. The rest is handled automatically (JSON + POST)
// ════════════════════════════════════════════════════════════
struct Detection {
  String label;
  float  x, y, w, h;   // normalised 0..1 (top-left origin)
  float  confidence;
};

void runDetectionAndPush() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  // ── TODO: insert your image processing here ──────────────
  // Example stub (uncomment and adapt):
  //
  //   Detection d;
  //   d.label      = "rock";
  //   d.x          = 0.30f;  // left edge, fraction of frame width
  //   d.y          = 0.20f;  // top  edge, fraction of frame height
  //   d.w          = 0.10f;
  //   d.h          = 0.15f;
  //   d.confidence = 0.87f;
  //   detections.push_back(d);
  // ─────────────────────────────────────────────────────────

  std::vector<Detection> detections;   // empty until you fill it

  esp_camera_fb_return(fb);

  // Build JSON payload
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.createNestedArray("detections");
  for (auto& d : detections) {
    JsonObject o = arr.createNestedObject();
    o["label"]      = d.label;
    o["x"]          = d.x;
    o["y"]          = d.y;
    o["w"]          = d.w;
    o["h"]          = d.h;
    o["confidence"] = d.confidence;
  }
  doc["ts"] = millis();

  String body; serializeJson(doc, body);

  // POST to controller /overlay
  HTTPClient http;
  String url = "http://" + String(CONTROLLER_IP) + ":" +
               String(CONTROLLER_PORT) + "/overlay";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code < 0) {
    Serial.printf("[PROC] POST failed: %s\n", http.errorToString(code).c_str());
  }
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
    if (millis() - t > 15000) {
      Serial.println("\n[WIFI] Timeout — restarting");
      ESP.restart();
    }
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

// ════════════════════════════════════════════════════════════
//  Controller registration
//  POSTs {"ip":"<our IP>"} to the controller so the browser
//  can discover us and start the camera stream.
//  Retries up to REG_MAX_RETRIES times before giving up.
// ════════════════════════════════════════════════════════════
void registerWithController() {
  String url  = "http://" + String(CONTROLLER_IP) + ":" +
                String(CONTROLLER_PORT) + "/register";
  String body = "{\"ip\":\"" + WiFi.localIP().toString() + "\"}";

  for (int attempt = 1; attempt <= REG_MAX_RETRIES; attempt++) {
    Serial.printf("[REG] Registering (attempt %d/%d)…\n", attempt, REG_MAX_RETRIES);
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    http.end();

    if (code == 200) {
      Serial.printf("[REG] Registered successfully at %s\n",
                    WiFi.localIP().toString().c_str());
      return;
    }
    Serial.printf("[REG] Failed (HTTP %d) — retrying in %dms\n",
                  code, REG_RETRY_DELAY_MS);
    delay(REG_RETRY_DELAY_MS);
  }
  Serial.println("[REG] Registration failed after all retries. "
                 "Browser will not receive cam IP until reboot or reconnect.");
}

// ════════════════════════════════════════════════════════════
//  Setup & loop
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32-CAM starting");

  if (!initCamera()) {
    Serial.println("[BOOT] Camera init failed — halting");
    while (true) delay(1000);
  }

  connectWiFi();

  // Register our IP with the controller so the browser can find us
  registerWithController();

  // Register HTTP routes
  server.on("/stream",   HTTP_GET, handleStream);
  server.on("/snapshot", HTTP_GET, handleSnapshot);
  server.on("/status",   HTTP_GET, handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Stream   : http://%s/stream\n",    WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Snapshot : http://%s/snapshot\n",  WiFi.localIP().toString().c_str());
}

void loop() {
  server.handleClient();

  // Periodic detection + overlay push
  if (millis() - lastDetectionMs >= DETECTION_INTERVAL_MS) {
    lastDetectionMs = millis();
    runDetectionAndPush();
  }
}
