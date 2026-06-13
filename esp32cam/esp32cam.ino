/*
 * EXCAVATOR CAM — ESP32-CAM sketch
 * See design.md for full architecture and endpoint reference.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>

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
const uint32_t ARUCO_CLEAR_MS        = DETECTION_INTERVAL_MS * 2;
const uint8_t  RED_R_MIN       = 160;
const uint8_t  RED_G_MAX       = 80;
const uint8_t  RED_B_MAX       = 80;
const int      RED_BLOB_MIN    = 30;
const uint8_t  ARUCO_SAT_MAX   = 30;
const uint8_t  ARUCO_OTSU_MIN  = 40;
const uint8_t  ARUCO_OTSU_MAX  = 140;
const float    ARUCO_MIN_CONF  = 0.60f;

WebServer server(80);

uint32_t lastDetectionMs = 0;
enum CamMode { MODE_SAFE, MODE_ARMED };
CamMode camMode = MODE_SAFE;
bool     pumpActive   = false;
uint32_t pumpLastSeen = 0;
int               lastDetectedArucoId = -1;
volatile uint32_t arucoDetectedAt     = 0;

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

inline bool isAchromatic(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t hi = max(r, max(g, b));
  uint8_t lo = min(r, min(g, b));
  return (hi - lo) < ARUCO_SAT_MAX;
}

static uint8_t otsuThreshold(const uint8_t* gray, int N, bool& bimodal) {
  uint32_t hist[256] = {};
  for (int i = 0; i < N; i++) hist[gray[i]]++;
  hist[255] = 0;

  uint64_t totalSum = 0;
  for (int i = 0; i < 256; i++) totalSum += (uint64_t)i * hist[i];
  uint32_t totalCount = 0;
  for (int i = 0; i < 256; i++) totalCount += hist[i];

  if (totalCount == 0) { bimodal = false; return 0; }

  uint32_t darkCount = 0;
  for (int i = 0; i < 80; i++) darkCount += hist[i];
  if (darkCount < totalCount / 200) {
    Serial.printf("[ARUCO] No dark population (darkCount=%u / %u) — skip\n", darkCount, totalCount);
    bimodal = false;
    return 0;
  }
  bimodal = true;

  uint64_t sumBg   = 0;
  uint32_t wBg     = 0;
  float    bestVar = 0.0f;
  uint8_t  thresh  = 128;

  for (int t = 0; t < 256; t++) {
    wBg   += hist[t];
    if (wBg == 0) continue;
    uint32_t wFg = totalCount - wBg;
    if (wFg == 0) break;
    sumBg += (uint64_t)t * hist[t];
    float muBg = (float)sumBg  / (float)wBg;
    float muFg = (float)(totalSum - sumBg) / (float)wFg;
    float var  = (float)wBg * (float)wFg * (muBg - muFg) * (muBg - muFg);
    if (var > bestVar) { bestVar = var; thresh = (uint8_t)t; }
  }

  if (thresh < ARUCO_OTSU_MIN) thresh = ARUCO_OTSU_MIN;
  if (thresh > ARUCO_OTSU_MAX) thresh = ARUCO_OTSU_MAX;
  Serial.printf("[ARUCO] Otsu threshold: %d (darkCount=%u achromatic=%u)\n",
                thresh, darkCount, totalCount);
  return thresh;
}

bool findDarkBlob(const uint8_t* gray, int W, int H,
                  int& bx1, int& by1, int& bx2, int& by2,
                  uint8_t thresh) {
  bx1 = W; by1 = H; bx2 = 0; by2 = 0;
  int count = 0;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      if (gray[y * W + x] < thresh) {
        if (x < bx1) bx1 = x; if (x > bx2) bx2 = x;
        if (y < by1) by1 = y; if (y > by2) by2 = y;
        count++;
      }
  int rw = bx2 - bx1, rh = by2 - by1;
  Serial.printf("[ARUCO] Dark pixel count=%d bbox=(%d,%d)-(%d,%d)\n", count, bx1, by1, bx2, by2);
  if (rw > W * 6 / 10 || rh > H * 6 / 10) {
    Serial.println("[ARUCO] Blob too large — background noise, skip");
    return false;
  }
  if (rw == 0 || rh == 0) return false;
  float asp = (float)rw / (float)rh;
  if (asp < 0.4f || asp > 2.5f) {
    Serial.println("[ARUCO] Blob aspect bad — skip");
    return false;
  }
  return (rw * rh >= 36 && count >= 36);
}

bool sampleArucoGrid(const uint8_t* gray, int W, int H,
                     int rx, int ry, int rw, int rh, uint16_t& outBits,
                     uint8_t thresh) {
  float cellW = (float)rw / 6.0f;
  float cellH = (float)rh / 6.0f;
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
      cells[row][col] = cnt > 0 && (sum / cnt) < thresh;
    }
  }
  for (int i = 0; i < 6; i++) {
    if (!cells[0][i] || !cells[5][i] || !cells[i][0] || !cells[i][5]) {
      Serial.printf("[ARUCO] Border check failed at i=%d\n", i);
      return false;
    }
  }
  outBits = 0;
  for (int r = 1; r <= 4; r++)
    for (int c = 1; c <= 4; c++) { outBits <<= 1; if (!cells[r][c]) outBits |= 1; }
  Serial.printf("[ARUCO] Grid sampled — bits=0x%04X\n", outBits);
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
  Serial.println("[ARUCO] No dictionary match");
  return -1;
}

// ════════════════════════════════════════════════════════════
//  Combined detection + overlay push
//
//  Single fb_get() per cycle — no set_framesize() calls.
//  W/H read from fb directly (works in JPEG mode on this board);
//  getFrameDims() used as fallback if either is 0.
//
//  Red dot thresholds: r>160, g<80, b<80, blob>=30 (from bc9eced).
//  ArUco runs on grayscale derived from the same RGB888 buffer.
//  Binarization threshold computed per-frame via Otsu's method on the
//  achromatic-filtered grayscale image.
//
//  After ArUco, gray[] is JPEG-encoded and stashed in dbgFrameBuf for
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

  // ── Build achromatic-filtered grayscale for ArUco ──
  uint8_t* gray = (uint8_t*)heap_caps_malloc((size_t)W * H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!gray) gray = (uint8_t*)malloc((size_t)W * H);
  if (gray) {
    for (int i = 0; i < W * H; i++) {
      uint8_t r = rgb[i*3+0], g = rgb[i*3+1], b = rgb[i*3+2];
      gray[i] = isAchromatic(r, g, b) ? toGray(r, g, b) : 255;
    }

    bool bimodal = false;
    uint8_t arucoThresh = otsuThreshold(gray, W * H, bimodal);
    if (bimodal) {
      int bx1, by1, bx2, by2;
      bool blobFound = findDarkBlob(gray, W, H, bx1, by1, bx2, by2, arucoThresh);
      Serial.printf("[ARUCO] Dark blob: %s\n", blobFound ? "YES" : "NO");
      if (blobFound) {
        int rw = bx2 - bx1, rh = by2 - by1;
        if (rw >= 6 && rh >= 6) {
          uint16_t bits = 0;
          bool gridOk = sampleArucoGrid(gray, W, H, bx1, by1, rw, rh, bits, arucoThresh);
          if (gridOk) {
            int id = lookupAruco(bits);
            Serial.printf("[ARUCO] Lookup: %d\n", id);
            if (id >= 0) {
              lastDetectedArucoId = id;
              arucoDetectedAt     = millis();
              Detection d;
              d.label = "aruco_" + String(id);
              d.x = (float)bx1/W; d.y = (float)by1/H;
              d.w = (float)rw/W;  d.h = (float)rh/H;
              d.confidence = ARUCO_MIN_CONF + 0.4f * min(1.0f, (float)(rw*rh)/(float)(W*H/4));
              detections.push_back(d);
              Serial.printf("[ARUCO] ID %d @ (%.2f,%.2f) size %.2fx%.2f conf=%.2f\n",
                            id, d.x, d.y, d.w, d.h, d.confidence);
            }
          }
        }
      }
    } else {
      Serial.println("[ARUCO] No marker candidate — skipping");
    }

    // ── Stash gray frame for /stream/gray debug view ──────────────────
    // Encodes the achromatic-filtered gray[] buffer as JPEG using a fake
    // camera_fb_t with PIXFORMAT_GRAYSCALE. Swapped into dbgFrameBuf under
    // dbgMux so handleGrayStream() always sees a consistent pointer+length.
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
