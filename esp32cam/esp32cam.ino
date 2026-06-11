/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Detection struct — defined here with guard so the Arduino preprocessor
// cannot duplicate it when it auto-includes all .h files in the sketch folder.
#ifndef DETECTION_STRUCT_DEFINED
#define DETECTION_STRUCT_DEFINED
struct Detection {
  String label;
  float  x, y, w, h;   // normalised 0..1 (top-left origin)
  float  confidence;
};
#endif

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

// ─────────────────────────────────────────────
//  Pump pin
// ─────────────────────────────────────────────
const int PIN_PUMP = 4;

// ─────────────────────────────────────────────
//  Pump heartbeat timeout
// ─────────────────────────────────────────────
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;

// ─────────────────────────────────────────────
//  Stream settings
// ─────────────────────────────────────────────
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;
const uint8_t     JPEG_QUALITY      = 12;
const int         STREAM_DELAY_MS   = 0;

// ─────────────────────────────────────────────
//  Detection settings
// ─────────────────────────────────────────────
const uint32_t DETECTION_INTERVAL_MS = 500;   // Detection runs every 500 ms, independent of stream state
const uint32_t ARUCO_CLEAR_MS        = DETECTION_INTERVAL_MS * 2;  // auto-clear after 1s
const framesize_t ARUCO_FRAME_SIZE   = FRAMESIZE_QQVGA;  // 160×120 — ArUco detection frame
const int      ARUCO_W               = 160;
const int      ARUCO_H               = 120;
const uint8_t  ARUCO_BINARIZE_THRESH = 100;
const int      ARUCO_MIN_CELL_PX     = 2;
const float    ARUCO_MIN_CONF        = 0.60f;

// ─────────────────────────────────────────────
//  Web server
// ─────────────────────────────────────────────
WebServer server(80);

// ─────────────────────────────────────────────
//  Runtime state
// ─────────────────────────────────────────────
uint32_t lastDetectionMs = 0;

enum CamMode { MODE_SAFE, MODE_ARMED };
CamMode camMode = MODE_SAFE;

bool     pumpActive   = false;
uint32_t pumpLastSeen = 0;

// ArUco detection state — written by loop(), read by HTTP handler
int      lastDetectedArucoId = -1;
volatile uint32_t arucoDetectedAt     = 0;

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

  Serial.println("[CAM] Camera ready");
  return true;
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
      streamClientActive = true;
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
      streamClientActive = false;
      Serial.println("[CAM] Stream client disconnected");
      delete c;
      vTaskDelete(NULL);
    },
    "stream_task", 8192, clientPtr, 1, NULL, 1
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

// Returns current ArUco detection state for UI polling.
// aruco_detected: true if a marker was seen within the last ARUCO_CLEAR_MS.
// aruco_id: the last detected marker ID, or -1 if none active.
void handleDetectionStatus() {
  bool active = (lastDetectedArucoId >= 0) &&
                ((millis() - arucoDetectedAt) < ARUCO_CLEAR_MS);
  StaticJsonDocument<128> doc;
  doc["aruco_detected"] = active;
  doc["aruco_id"]       = active ? lastDetectedArucoId : -1;
  doc["ts"]             = millis();
  String out; serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

// ════════════════════════════════════════════════════════════
//  ArUco dictionary + helpers
// ════════════════════════════════════════════════════════════

static const uint16_t ARUCO_4X4_50[50] = {
  0b0111001001101101, // 0
  0b0111011000001101, // 1
  0b0111101101000010, // 2
  0b0111111100100010, // 3
  0b0101011010011101, // 4
  0b0101001011111101, // 5
  0b0101111110100000, // 6
  0b0101101111000000, // 7
  0b0011001110001011, // 8
  0b0011011111101011, // 9
  0b0011101010110110, // 10
  0b0011111011010110, // 11
  0b0001011001111011, // 12
  0b0001001000011011, // 13
  0b0001111101000110, // 14
  0b0001101100100110, // 15
  0b1110001001010010, // 16
  0b1110011000110010, // 17
  0b1110101101101111, // 18
  0b1110111100001111, // 19
  0b1100011010100000, // 20
  0b1100001011000000, // 21
  0b1100111110011101, // 22
  0b1100101111111101, // 23
  0b1010001110010001, // 24
  0b1010011111110001, // 25
  0b1010101010101100, // 26
  0b1010111011001100, // 27
  0b1000011001100001, // 28
  0b1000001000000001, // 29
  0b1000111101011100, // 30
  0b1000101100111100, // 31
  0b0110110010001110, // 32
  0b0110100011101110, // 33
  0b0110010110110011, // 34
  0b0110000111010011, // 35
  0b0100100001111110, // 36
  0b0100110000011110, // 37
  0b0100000101000011, // 38
  0b0100010100100011, // 39
  0b0010110010011010, // 40
  0b0010100011111010, // 41
  0b0010010110100111, // 42
  0b0010000111000111, // 43
  0b0000100001101010, // 44
  0b0000110000001010, // 45
  0b0000000101010111, // 46
  0b0000010100110111, // 47
  0b1111000001001000, // 48
  0b1111010000101000, // 49
};

inline uint8_t toGray(uint8_t r, uint8_t g, uint8_t b) {
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

bool findDarkBlob(const uint8_t* gray, int W, int H,
                  int& bx1, int& by1, int& bx2, int& by2) {
  bx1 = W; by1 = H; bx2 = 0; by2 = 0;
  int count = 0;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      if (gray[y * W + x] < ARUCO_BINARIZE_THRESH) {
        if (x < bx1) bx1 = x; if (x > bx2) bx2 = x;
        if (y < by1) by1 = y; if (y > by2) by2 = y;
        count++;
      }
  return ((bx2 - bx1) * (by2 - by1) >= 36 && count >= 36);
}

bool sampleArucoGrid(const uint8_t* gray, int W, int H,
                     int rx, int ry, int rw, int rh, uint16_t& outBits) {
  float cellW = (float)rw / 6.0f;
  float cellH = (float)rh / 6.0f;
  // Remove min cell size check — QQVGA is small enough
  const int MIN_CELL_PX = 1;  // was ARUCO_MIN_CELL_PX = 2
  bool cells[6][6];
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      int x0 = max(0, min(W-1, rx + (int)(col * cellW + cellW * 0.25f)));
      int y0 = max(0, min(H-1, ry + (int)(row * cellH + cellH * 0.25f)));
      int x1 = max(0, min(W-1, rx + (int)(col * cellW + cellW * 0.75f)));
      int y1 = max(0, min(H-1, ry + (int)(row * cellH + cellH * 0.75f)));
      long sum = 0; int cnt = 0;
      for (int py = y0; py <= y1; py++)
        for (int px = x0; px <= x1; px++) { sum += gray[py * W + px]; cnt++; }
      cells[row][col] = cnt > 0 && (sum / cnt) < ARUCO_BINARIZE_THRESH;
    }
  }
  for (int i = 0; i < 6; i++)
    if (!cells[0][i] || !cells[5][i] || !cells[i][0] || !cells[i][5]) return false;
  outBits = 0;
  for (int r = 1; r <= 4; r++)
    for (int c = 1; c <= 4; c++) { outBits <<= 1; if (!cells[r][c]) outBits |= 1; }
  return true;
}

int lookupAruco(uint16_t bits) {
  for (int rot = 0; rot < 4; rot++) {
    for (int id = 0; id < 50; id++) if (ARUCO_4X4_50[id] == bits) return id;
    uint16_t rotated = 0;
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 4; c++) {
        if (bits & (1 << (15 - ((3 - c) * 4 + r))))
          rotated |= (1 << (15 - (r * 4 + c)));
      }
    bits = rotated;
  }
  return -1;
}

void runArucoDetection(std::vector<Detection>& detections) {
  // Temporarily switch to QQVGA for smaller detection frame
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, ARUCO_FRAME_SIZE);
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0) {
    esp_camera_fb_return(fb);
    s->set_framesize(s, STREAM_FRAME_SIZE);  // restore
    return;
  }
  
  const int srcW = ARUCO_W, srcH = ARUCO_H;  // use fixed QQVGA size
  s->set_framesize(s, STREAM_FRAME_SIZE);  // restore immediately
  const int W = srcW / ARUCO_DSAMPLE, H = srcH / ARUCO_DSAMPLE;

  
  size_t rgbLen = (size_t)srcW * srcH * 3;
  uint8_t* rgb = (uint8_t*)heap_caps_malloc(rgbLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb) rgb = (uint8_t*)malloc(rgbLen);
  bool decoded = rgb && fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  if (!decoded) { if (rgb) free(rgb); return; }

  uint8_t* gray = (uint8_t*)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!gray) gray = (uint8_t*)malloc((size_t)W * H);
  if (!gray) { free(rgb); return; }

  for (int dy = 0; dy < H; dy++)
    for (int dx = 0; dx < W; dx++) {
      long sum = 0;
      for (int by = 0; by < ARUCO_DSAMPLE; by++)
        for (int bx = 0; bx < ARUCO_DSAMPLE; bx++) {
          int idx = ((dy * ARUCO_DSAMPLE + by) * srcW + (dx * ARUCO_DSAMPLE + bx)) * 3;
          sum += toGray(rgb[idx], rgb[idx+1], rgb[idx+2]);
        }
      gray[dy * W + dx] = (uint8_t)(sum / (ARUCO_DSAMPLE * ARUCO_DSAMPLE));
    }
  free(rgb);

  int bx1, by1, bx2, by2;
    if (findDarkBlob(gray, W, H, bx1, by1, bx2, by2)) {
    int rw = bx2 - bx1, rh = by2 - by1;
    if (rw >= 6 * MIN_CELL_PX && rh >= 6 * MIN_CELL_PX) {
      uint16_t bits = 0;
      if (sampleArucoGrid(gray, W, H, bx1, by1, rw, rh, bits)) {
        int id = lookupAruco(bits);
        if (id >= 0) {
          // Update shared state for /detection_status endpoint
          lastDetectedArucoId = id;
          arucoDetectedAt     = millis();

          Detection d;
          d.label = "aruco_" + String(id);
          d.x = (float)bx1/srcW; d.y = (float)by1/srcH;
          d.w = (float)rw/srcW;  d.h = (float)rh/srcH;
          d.confidence = ARUCO_MIN_CONF + 0.4f * min(1.0f, (float)(rw*rh)/(float)(W*H/4));
          detections.push_back(d);
          Serial.printf("[ARUCO] ID %d @ (%.2f,%.2f) size %.2fx%.2f conf=%.2f\n",
                        id, d.x, d.y, d.w, d.h, d.confidence);
        }
      }size_t rgbLen = (size_t)srcW * srcH * 3;
    }
  }
  free(gray);
}

// ════════════════════════════════════════════════════════════
//  Red dot detection
//  Tightened thresholds to prevent false positives on ArUco ink:
//  r > 180, g < 60, b < 60, red must dominate by 3x, min 200px blob
// ════════════════════════════════════════════════════════════
void runRedDotDetection(std::vector<Detection>& detections) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0) {
    if (fb) esp_camera_fb_return(fb);
    return;
  }
  const int W = fb->width, H = fb->height;
  size_t rgbLen = (size_t)W * H * 3;
  uint8_t* rgb = (uint8_t*)heap_caps_malloc(rgbLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb) rgb = (uint8_t*)malloc(rgbLen);
  bool decoded = rgb && fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);
  esp_camera_fb_return(fb);
  if (decoded) {
    int bx1 = W, by1 = H, bx2 = 0, by2 = 0, blobCount = 0;
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++) {
        uint8_t r = rgb[(y*W+x)*3], g = rgb[(y*W+x)*3+1], b = rgb[(y*W+x)*3+2];
        if (r > 180 && g < 60 && b < 60 && r > (uint8_t)(g * 3) && r > (uint8_t)(b * 3)) {
          if (x < bx1) bx1 = x; if (x > bx2) bx2 = x;
          if (y < by1) by1 = y; if (y > by2) by2 = y;
          blobCount++;
        }
      }
    if (blobCount >= 200) {
      Detection d;
      d.label = "red_dot";
      float cx = (bx1+bx2)/2.0f, cy = (by1+by2)/2.0f;
      float hw = (bx2-bx1)/2.0f*0.8f, hh = (by2-by1)/2.0f*0.8f;
      d.x=(cx-hw)/W; d.y=(cy-hh)/H; d.w=hw*2/W; d.h=hh*2/H;
      d.confidence = min(1.0f, (float)blobCount/500.0f);
      detections.push_back(d);
      Serial.printf("[PROC] Red dot @ (%.2f,%.2f) size %.2fx%.2f conf=%.2f\n",
                    d.x, d.y, d.w, d.h, d.confidence);
    }
  }
  if (rgb) free(rgb);
}

// ════════════════════════════════════════════════════════════
//  Combined detection + overlay push
// ════════════════════════════════════════════════════════════
void runDetectionAndPush() {
  std::vector<Detection> detections;
  runRedDotDetection(detections);
  runArucoDetection(detections);

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
  if (code < 0) Serial.printf("[PROC] POST failed: %s\n", http.errorToString(code).c_str());
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
  connectWiFi();
  registerWithController();
  server.on("/stream",           HTTP_GET, handleStream);
  server.on("/snapshot",         HTTP_GET, handleSnapshot);
  server.on("/pump",             HTTP_GET, handlePump);
  server.on("/mode",             HTTP_GET, handleMode);
  server.on("/status",           HTTP_GET, handleStatus);
  server.on("/detection_status", HTTP_GET, handleDetectionStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  // Spawn detection on Core 0; stream task uses Core 1
  server.begin();

  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Stream          : http://%s/stream\n",           WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Snapshot        : http://%s/snapshot\n",         WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Detection status: http://%s/detection_status\n", WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Mode            : SAFE (boots safe, pump blocked)\n");
}

void loop() {
  server.handleClient();
  
  // Pump heartbeat watchdog
  if (pumpActive && (millis() - pumpLastSeen) > PUMP_HOLD_TIMEOUT_MS) {
    Serial.println("[PUMP] Heartbeat timeout — auto-release");
    pumpOff();
  }
  
  // Detection — runs independently of stream; suppressed only while pump is active
  if (!pumpActive && (millis() - lastDetectionMs) >= DETECTION_INTERVAL_MS) {
    lastDetectionMs = millis();
    runDetectionAndPush();
  }
}