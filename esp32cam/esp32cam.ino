/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
//  WiFi  (controller is the AP)
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "ExcavatorAP";
const char* WIFI_PASSWORD = "exc@vator123";

// ─────────────────────────────────────────────
//  Controller address (AP default gateway)
// ─────────────────────────────────────────────
const char*    CONTROLLER_IP   = "192.168.4.1";
const uint16_t CONTROLLER_PORT = 80;

// ─────────────────────────────────────────────
//  Registration retry settings
// ─────────────────────────────────────────────
const int      REG_MAX_RETRIES    = 5;
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
//  Pump pin
//  AI-Thinker ESP32-CAM, no SD card:
//  GPIO 13 is free and safe for output.
// ─────────────────────────────────────────────
const int PIN_PUMP = 4; //Temp for LED, change fire action back to LOW when attaching pump

// ─────────────────────────────────────────────
//  Pump heartbeat timeout
//  If controller stops sending hold heartbeats,
//  auto-release the pump after this many ms.
// ─────────────────────────────────────────────
const uint32_t PUMP_HOLD_TIMEOUT_MS = 300;  // Fix B: was 300 — widened to 800 for WiFi round-trip headroom

// ─────────────────────────────────────────────
//  Stream settings
// ─────────────────────────────────────────────
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;    // 640×480 — sharper image
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;    // 640×480
const uint8_t     JPEG_QUALITY      = 12;
const int         STREAM_DELAY_MS   = 0;

// ─────────────────────────────────────────────
//  Image processing interval
// ─────────────────────────────────────────────
const uint32_t DETECTION_INTERVAL_MS = 500;

// ─────────────────────────────────────────────
//  Web server
// ─────────────────────────────────────────────
WebServer server(80);

// ─────────────────────────────────────────────
//  Runtime state
// ─────────────────────────────────────────────
uint32_t lastDetectionMs = 0;

// Safe/Armed mode — boots in SAFE
enum CamMode { MODE_SAFE, MODE_ARMED };
CamMode camMode = MODE_SAFE;

// Pump hold state
bool     pumpActive    = false;
uint32_t pumpLastSeen  = 0;   // millis() of last hold heartbeat

// ════════════════════════════════════════════════════════════
//  Pump helpers
// ════════════════════════════════════════════════════════════
void pumpOn() {
  if (camMode == MODE_SAFE) {
    Serial.println("[PUMP] Blocked — in SAFE mode");
    return;
  }
  if (!pumpActive) {
    pinMode(PIN_PUMP, OUTPUT);
    digitalWrite(PIN_PUMP, HIGH);   // active-low relay — adjust if needed
    pumpActive = true;
    Serial.println("[PUMP] ON");
  }
  pumpLastSeen = millis();
}

void pumpOff() {
  if (pumpActive) {
    pinMode(PIN_PUMP, INPUT);   // high-Z
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
  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
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

  // RGB565 at VGA requires 614400 bytes — exceeds reliable PSRAM allocation.
  // Cap at QVGA (320x240 = 153600 bytes) for both paths.
  if (psramFound()) {
    config.frame_size   = STREAM_FRAME_SIZE;  // QVGA — safe for RGB565
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

  Serial.println("[CAM] Camera ready");
  return true;
}

// ════════════════════════════════════════════════════════════
//  HTTP handlers
// ════════════════════════════════════════════════════════════

// GET /stream  — MJPEG
// Camera captures RGB565; frames are converted to JPEG via frame2jpg() before sending.
void handleStream() {
  // Grab the client before handing off
  WiFiClient client = server.client();

  // Send the initial HTTP header directly
  String header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n"
    "\r\n";
  client.print(header);

  // Spin off a FreeRTOS task so handleClient() is no longer blocked
  WiFiClient* clientPtr = new WiFiClient(client);
  xTaskCreatePinnedToCore(
    [](void* arg) {
      WiFiClient* c = (WiFiClient*)arg;
      while (c->connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;

        // Camera is in RGB565 mode — convert to JPEG for MJPEG stream
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
    "stream_task",
    8192,        // stack size — needs to be generous for camera ops
    clientPtr,
    1,           // priority
    NULL,
    1            // pin to core 1 (core 0 runs WiFi)
  );
}

// GET /snapshot  — single JPEG at full res
void handleSnapshot() {
  // RGB565 mode: stay at STREAM_FRAME_SIZE (QVGA) — VGA would overflow the frame buffer
  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb) { server.send(500, "text/plain", "Capture failed"); return; }

  // Convert RGB565 to JPEG for snapshot delivery
  uint8_t* jpgBuf = nullptr;
  size_t   jpgLen = 0;
  bool converted = frame2jpg(fb, 12, &jpgBuf, &jpgLen);
  esp_camera_fb_return(fb);

  if (!converted || jpgLen == 0) { server.send(500, "text/plain", "JPEG conversion failed"); return; }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)jpgBuf, jpgLen);
  free(jpgBuf);
}

// GET /pump?action=press|hold|release
void handlePump() {
  String action = server.hasArg("action") ? server.arg("action") : "press";
  if (action == "press" || action == "hold") {
    pumpOn();
  } else if (action == "release") {
    pumpOff();
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", pumpActive ? "on" : "off");
}

// GET /mode?set=safe|armed
void handleMode() {
  if (!server.hasArg("set")) { server.send(400, "text/plain", "Missing set"); return; }
  String m = server.arg("set");
  if (m == "armed") {
    camMode = MODE_ARMED;
    Serial.println("[MODE] ARMED");
  } else if (m == "safe") {
    camMode = MODE_SAFE;
    pumpOff();   // safety: immediately cut pump when entering safe mode
    Serial.println("[MODE] SAFE");
  } else {
    server.send(400, "text/plain", "Unknown mode");
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", camMode == MODE_ARMED ? "armed" : "safe");
}

// GET /status  — JSON: ip, uptime, heap, mode, pumpActive
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
//  Detection struct
// ════════════════════════════════════════════════════════════
struct Detection {
  String label;
  float  x, y, w, h;   // normalised 0..1 (top-left origin)
  float  confidence;
};

// ════════════════════════════════════════════════════════════
//  Image processing — Red dot detection + overlay push
// ════════════════════════════════════════════════════════════
void runDetectionAndPush() {
  sensor_t* s = esp_camera_sensor_get();
  s->set_pixformat(s, PIXFORMAT_RGB565);
  delay(100); // let sensor settle

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  std::vector<Detection> detections;

  // ── Red dot detection via RGB565 colour thresholding ──────
  // RGB565: each pixel = 2 bytes, big-endian
  // R[4:0] = bits[15:11], G[5:0] = bits[10:5], B[4:0] = bits[4:0]
  const int W = fb->width;
  const int H = fb->height;
  const uint16_t* pixels = (const uint16_t*)fb->buf;

  // Thresholds tuned for a bright red dot
  const uint8_t R_MIN = 15, R_MAX = 31;  // 5-bit red   (>=15 = saturated red)
  const uint8_t G_MAX = 12;              // 6-bit green  (low green)
  const uint8_t B_MAX = 12;              // 5-bit blue   (low blue)
  const int MIN_BLOB  = 30;              // min pixels to count as a dot

  // Simple bounding-box blob finder
  int bx1 = W, by1 = H, bx2 = 0, by2 = 0;
  int blobCount = 0;

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint16_t px = pixels[y * W + x];
      // RGB565 byte-swap (ESP32 stores little-endian)
      px = (px >> 8) | (px << 8);
      uint8_t r = (px >> 11) & 0x1F;
      uint8_t g = (px >>  5) & 0x3F;
      uint8_t b =  px        & 0x1F;

      if (r >= R_MIN && r <= R_MAX && g <= G_MAX && b <= B_MAX) {
        if (x < bx1) bx1 = x;
        if (x > bx2) bx2 = x;
        if (y < by1) by1 = y;
        if (y > by2) by2 = y;
        blobCount++;
      }
    }
  }

  if (blobCount >= MIN_BLOB) {
    Detection d;
    d.label      = "red_dot";
    float cx = (float)(bx1 + bx2) / 2.0f;
    float cy = (float)(by1 + by2) / 2.0f;
    float hw = (float)(bx2 - bx1) / 2.0f;
    float hh = (float)(by2 - by1) / 2.0f;
    // Shrink box by 20% to cut edge noise
    hw *= 0.8f;
    hh *= 0.8f;
    d.x = (cx - hw) / W;
    d.y = (cy - hh) / H;
    d.w = (hw * 2.0f) / W;
    d.h = (hh * 2.0f) / H;
    d.confidence = min(1.0f, (float)blobCount / 500.0f);  // scale to 0..1
    detections.push_back(d);
    Serial.printf("[PROC] Red dot @ (%.2f,%.2f) size %.2fx%.2f conf=%.2f\n",
                  d.x, d.y, d.w, d.h, d.confidence);
  }
  // ── end detection ─────────────────────────────────────────

  esp_camera_fb_return(fb);

  s->set_pixformat(s, PIXFORMAT_JPEG);

  // Serialize + POST to /overlay
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.createNestedArray("detections");
  for (auto& d : detections) {
    JsonObject o = arr.createNestedObject();
    o["label"] = d.label;
    o["x"] = d.x; o["y"] = d.y;
    o["w"] = d.w; o["h"] = d.h;
    o["confidence"] = d.confidence;
  }
  doc["ts"] = millis();
  String body; serializeJson(doc, body);

  HTTPClient http;
  String url = "http://" + String(CONTROLLER_IP) + ":" +
               String(CONTROLLER_PORT) + "/overlay";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code < 0)
    Serial.printf("[PROC] POST failed: %s\n", http.errorToString(code).c_str());
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
    Serial.printf("[REG] Registering (attempt %d/%d)…\n", attempt, REG_MAX_RETRIES);
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    http.end();
    if (code == 200) {
      Serial.printf("[REG] Registered at %s\n", WiFi.localIP().toString().c_str());
      return;
    }
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

  // Pump pin — start as input (high-Z / off)
  pinMode(PIN_PUMP, INPUT);

  if (!initCamera()) {
    Serial.println("[BOOT] Camera init failed — halting");
    while (true) delay(1000);
  }

  connectWiFi();
  registerWithController();

  server.on("/stream",   HTTP_GET, handleStream);
  server.on("/snapshot", HTTP_GET, handleSnapshot);
  server.on("/pump",     HTTP_GET, handlePump);
  server.on("/mode",     HTTP_GET, handleMode);
  server.on("/status",   HTTP_GET, handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Stream   : http://%s/stream\n",   WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Snapshot : http://%s/snapshot\n", WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Mode     : SAFE (boots safe, pump blocked)\n");
}

void loop() {
  server.handleClient();

  // Auto-release pump if hold heartbeat has timed out
  if (pumpActive && (millis() - pumpLastSeen) > PUMP_HOLD_TIMEOUT_MS) {
    Serial.println("[PUMP] Heartbeat timeout — auto-release");
    pumpOff();
  }

  // Periodic detection + overlay push — skip while pump is firing (Fix D)
  if (!pumpActive && (millis() - lastDetectionMs >= DETECTION_INTERVAL_MS)) {
    lastDetectionMs = millis();
    runDetectionAndPush();
  }
}