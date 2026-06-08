# Excavator Robot â€” System Design Document

---

## Overview

This document describes the full firmware architecture for a two-module wireless excavator robot. One module is the **ESP32-CAM** (Joy-IT SBC-ESP32-Cam), which streams video and performs on-device image processing. The other is the **NodeMCU ESP32** (Berrybase NMCU-ESP32), which acts as the WiFi Access Point, hosts the web control console, and drives the physical remote-control buttons of the excavator via GPIO. The controller runs on the Berrybase NodeMCU-ESP32 and the camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM), which connects to the controller's AP as a client, streams MJPEG video, runs on-device image processing, and pushes detection overlays to the controller. The camera also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

---

## Module Roles

| Module | Hardware | Role |
|--------|----------|------|
| Controller | Berrybase NodeMCU-ESP32 | WiFi AP Â· Web console Â· GPIO button driver Â· Overlay store |
| Camera | Joy-IT SBC-ESP32-Cam | WiFi client Â· MJPEG streamer Â· Image processing Â· Overlay pusher |

---

## Network Topology

```
[ Browser ]
    |  HTTP (port 80)
    |
[ NodeMCU-ESP32 â€“ Controller ]   192.168.4.1
    WiFi AP: ExcavatorAP
    |
    |  WiFi (STA <- AP)
    |
[ ESP32-CAM â€“ Camera ]           192.168.4.x (DHCP)
```

All three parties communicate over the single `ExcavatorAP` network. The browser connects to the AP and addresses both devices via HTTP. The cam never receives commands directly from the browser â€” all relayed commands go through the controller's `/camcmd` endpoint.

---

## Data Flow

### Camera Feed

```
ESP32-CAM  ->  /stream (MJPEG)  ->  Browser <img> tag
```

### Detection Overlay

```
ESP32-CAM  ->  POST /overlay (JSON)  ->  Controller
Browser    ->  GET  /overlay (JSON)  ->  Controller  ->  Canvas draw
```

### Button Commands

```
Browser (hold/release events)
    ->  GET /cmd?btn=X&action=Y
    ->  Controller: pressPin / releasePin
    ->  Physical GPIO pulls remote-control button line LOW
```

### Cam Command Relay

```
Browser (pump / mode actions)
    ->  GET /camcmd?cmd=X
    ->  Controller
    ->  GET http://<camIP>/pump?action=X  or  /mode?set=X
    ->  ESP32-CAM executes
```

### Status Badge

```
Browser  ->  GET http://<camIP>/status  ->  { mode, pumpActive }  ->  SAFE/ARMED badge
```

---

## Required Libraries

| Library | Used by |
|---------|---------|
| `WiFi.h` | Both |
| `WebServer.h` | Both |
| `ArduinoJson` | Both |
| `esp_camera.h` | Camera only |
| `HTTPClient.h` | Camera only |

ESP32 board package URL: `https://dl.espressif.com/dl/package_esp32_index.json`

---

## Controller â€” NodeMCU ESP32

The controller runs on the Berrybase NodeMCU-ESP32. It hosts the WiFi Access Point, serves the web console, stores detection overlays pushed by the camera, and drives the physical remote-control buttons of the excavator via GPIO. It also relays pump and mode commands to the camera module via the `/camcmd` endpoint.

### WiFi / Network Constants

```cpp
const char* AP_SSID       = "ExcavatorAP";
const char* AP_PASSWORD   = "exc@vator123";
const IPAddress AP_IP     (192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET (255, 255, 255, 0);
```

### GPIO Pin Constants

> **Important:** GPIO 34, 35, 36, 39 are **INPUT-ONLY** on the ESP32 â€” they have no output driver and will silently fail. Safe output-capable GPIOs on the NodeMCU-ESP32: 2, 4, 5, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33.

```cpp
const int PIN_LEFT_FWD    = 13;   // Left track  forward
const int PIN_LEFT_BACK   = 12;   // Left track  back
const int PIN_RIGHT_FWD   = 14;   // Right track forward
const int PIN_RIGHT_BACK  = 27;   // Right track back
const int PIN_TURN_LEFT   = 26;   // Turn left
const int PIN_TURN_RIGHT  = 25;   // Turn right
const int PIN_UP          = 33;   // Arm / bucket up
const int PIN_DOWN        = 32;   // Arm / bucket down
const int PIN_LIGHT       = 15;   // Light (momentary pulse)
const int PIN_TEST        = 4;    // Test button 1
```

`PIN_LIGHT` is split into two pins:

```cpp
// PIN_LIGHT_ON  â€“ pulse to turn light on
// PIN_LIGHT_OFF â€“ pulse to turn light off
// (hidden behind a single toggle in the web UI)
```

All output pins are tracked in an array for safe initialisation and `releaseAll()`:

```cpp
const int ALL_PINS[] = {
  PIN_LEFT_FWD, PIN_LEFT_BACK,
  PIN_RIGHT_FWD, PIN_RIGHT_BACK,
  PIN_TURN_LEFT, PIN_TURN_RIGHT,
  PIN_UP, PIN_DOWN,
  PIN_LIGHT, PIN_TEST
};
const int PIN_COUNT = sizeof(ALL_PINS) / sizeof(ALL_PINS[0]);
```

### Timing Constants

```cpp
const uint32_t PULSE_DURATION_MS = 150;   // Duration of light / test momentary pulse
const uint32_t HOLD_TIMEOUT_MS   = 300;   // Auto-release timeout when heartbeat stops
```

### Button Behaviour

| Button | Type | Behaviour |
|--------|------|-----------|
| `left_fwd` | Hold | Active while held; released on pointer-up / key-up |
| `left_back` | Hold | Active while held |
| `right_fwd` | Hold | Active while held |
| `right_back` | Hold | Active while held |
| `turn_left` | Hold | Active while held |
| `turn_right` | Hold | Active while held |
| `up` | Hold | Active while held |
| `down` | Hold | Active while held |
| `light` | Pulse | Toggle backed by `PIN_LIGHT_ON` / `PIN_LIGHT_OFF` pulses |
| `test` | Hold | Active while held |

Multi-button simultaneous hold is supported, allowing multi-pin actions. WASD simultaneously drives both tracks (forward/back) or opposite-spin turn.

### Pin-Control Pattern

The button emulation works by pulling the remote-control line LOW (simulating a button press) and releasing it by returning the GPIO to high-impedance input mode:

```cpp
// Press the button (pull the line down)
pinMode(pin, OUTPUT);
digitalWrite(pin, LOW);
// some delay

// Release the button
pinMode(pin, INPUT);        // back to high-Z
```

Helper functions wrap this pattern:

```cpp
void pressPin(int pin)   { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
void releasePin(int pin) { pinMode(pin, INPUT); }   // back to high-Z
void releaseAll()        { /* iterates ALL_PINS, calls releasePin */ }
void pulsePin(int pin, uint32_t durationMs) { pressPin(pin); delay(durationMs); releasePin(pin); }
```

### Heartbeat Watchdog

The browser sends a heartbeat `action=hold` every `HOLD_HEARTBEAT_INTERVAL_MS` while a key or button is held. If no heartbeat arrives within `HOLD_TIMEOUT_MS` (300 ms), the controller auto-releases the pin. This is a safety feature preventing the excavator from running away if the browser tab closes or network drops.

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Serves the web console HTML |
| `POST` | `/register` | Cam POSTs `{"ip":"<addr>"}` to register its IP |
| `GET` | `/camip` | Browser polls to learn the cam's IP |
| `POST` | `/overlay` | Cam pushes detection JSON |
| `GET` | `/overlay` | Browser polls detection overlay JSON |
| `GET` | `/cmd?btn=X&action=Y` | Browser sends button press / hold / release |
| `GET` | `/camcmd?cmd=X` | Controller relays pump and mode commands to cam |
| `GET` | `/status` | Returns `{camIP, uptime, heap, heldPin}` |

#### `/cmd` Action Values

- `press` â€” start holding the pin LOW; begins heartbeat window
- `hold` â€” heartbeat renewal; resets the `HOLD_TIMEOUT_MS` watchdog
- `release` â€” release the pin immediately

### Web Console

The HTML console is stored in flash as `INDEX_HTML[]` (`PROGMEM`). It includes:

- An **MJPEG feed** `<img>` whose `src` is set dynamically after `/camip` returns the cam's address
- A `<canvas>` overlay rendered on top of the feed for detection bounding boxes
- Five control groups: **Left Track**, **Right Track**, **Turn**, **Arm / Bucket**, **Lights & Aux**
- Each button shows an icon, a label, and a keyboard shortcut badge
- Simplified controls with a top-down excavator model SVG
- Model glows live for tracks / turntable / arm / pump / test / light activation states
- Live **SAFE / ARMED** badge polled from cam `/status` endpoint

#### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` | Left track forward |
| `S` | Left track back |
| `â†‘` Arrow Up | Right track forward |
| `â†“` Arrow Down | Right track back |
| `â†` Arrow Left | Turn left |
| `â†’` Arrow Right | Turn right |
| `Q` | Arm / bucket up |
| `E` | Arm / bucket down |
| `L` | Light toggle (pulse) |
| `T` | Test hold |
| `Space` | Pump hold (relayed to cam via `/camcmd`) |

#### Overlay Canvas

The browser polls `/overlay` every 500 ms. Detection boxes are drawn in amber (`#f0a500`) with label and confidence percentage. The canvas is sized to match the feed container on every `resize` event.

#### Camera Feed Polling

The browser polls `/camip` every 2000 ms. When the cam's IP is received, `feedImg.src` is set to `http://<camIP>/stream`. On stream error the feed retries after 3 s.

### Main Loop

```
setup():
  releaseAll()          <- ensure all pins start high-Z
  WiFi.softAP(...)
  register HTTP routes
  server.begin()

loop():
  server.handleClient()
  // Heartbeat watchdog
  if (heldPin.active && (millis() - heldPin.lastSeen) > HOLD_TIMEOUT_MS) {
    auto-release pin
  }
```

---

## Camera Module â€” Joy-IT SBC-ESP32-Cam

The camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM). It connects to the controller's WiFi AP as a client, streams MJPEG video, runs on-device image processing, and pushes detection overlays to the controller. It also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

> Select `AI Thinker ESP32-CAM` in Arduino IDE under Tools -> Board -> ESP32 Arduino.

### WiFi / Network Constants

```cpp
const char*    WIFI_SSID         = "ExcavatorAP";
const char*    WIFI_PASSWORD     = "exc@vator123";
const char*    CONTROLLER_IP     = "192.168.4.1";
const uint16_t CONTROLLER_PORT   = 80;
```

### Registration

After WiFi connect, the cam POSTs its IP to the controller's `/register` endpoint, retrying up to `REG_MAX_RETRIES` times with `REG_RETRY_DELAY_MS` between attempts:

```
POST http://192.168.4.1/register
Content-Type: application/json
{"ip":"<cam LAN IP>"}
```

If registration fails after all retries, the browser will not receive the cam IP until a reboot or reconnect.

```cpp
const int      REG_MAX_RETRIES    = 5;
const uint32_t REG_RETRY_DELAY_MS = 1000;
```

### Camera Hardware Pinout

```cpp
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
```

#### SD Card / LED Pin Conflicts

The following pins are internally connected to the SD card slot on the SBC-ESP32-Cam and should not be used for other purposes:

| Pin | SD Function |
|-----|------------|
| IO14 | CLK |
| IO15 | CMD |
| IO2 | Data 0 |
| IO4 | Data 1 (also on-board LED) |
| IO12 | Data 2 |
| IO13 | Data 3 |

#### Flash / Upload Mode

To put the device into flash mode: connect **IO0 to GND** before power-on. Remove the connection after upload. A USB-to-TTL converter (3.3 V) is required (no built-in USB). Serial baud rate: **115200**.

| Camera Module Pin | USB-TTL Converter |
|-------------------|--------------------|
| 5V | 5V |
| GND | GND |
| U0T (IO1) | RX |
| U0R (IO3) | TX |

### Stream / Quality Constants

```cpp
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_QVGA;   // 320x240
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;    // 640x480
const uint8_t     JPEG_QUALITY      = 12;   // 0 = best ... 63 = worst; 10-15 sweet spot
const int         STREAM_DELAY_MS   = 0;    // extra inter-frame delay (ms)
```

If PSRAM is present, `SNAP_FRAME_SIZE` is used for the initial frame buffer with `fb_count = 2`; otherwise `STREAM_FRAME_SIZE` with `fb_count = 1` and `JPEG_QUALITY + 4`.

### MJPEG Stream Format

```
Content-Type: multipart/x-mixed-replace; boundary=frame
Access-Control-Allow-Origin: *

--frame
Content-Type: image/jpeg
Content-Length: <N>

<JPEG bytes>
```

### Image Processing

Detection runs on-device at `DETECTION_INTERVAL_MS` intervals. Results are POSTed as JSON to the controller's `/overlay` endpoint:

```cpp
const uint32_t DETECTION_INTERVAL_MS = 500;   // How often to run detection and push overlay
```

```cpp
struct Detection {
  String label;
  float  x, y, w, h;   // normalised 0..1, top-left origin
  float  confidence;
};
```

Detection result JSON schema:

```json
{
  "detections": [
    { "label": "rock", "x": 0.30, "y": 0.20, "w": 0.10, "h": 0.15, "confidence": 0.87 }
  ],
  "ts": 12345
}
```

#### Extension Points

1. Capture a frame (`fb->buf`, `fb->len`, `fb->width`, `fb->height`)
2. Run algorithm: edge detection, blob tracking, TFLite model inference, ArUco markers, colour thresholding, etc.
3. Populate the `detections` vector with `Detection` structs
4. The rest is handled automatically (JSON serialisation + POST to `/overlay`)

### Pump Control

```cpp
// Pump relay pin
#define GPIO_PUMP  13   // hold-while-pressed

// Pump heartbeat timeout (ms) - auto-releases if no hold heartbeat arrives
const uint32_t PUMP_HOLD_TIMEOUT_MS = 300;
```

### Safe / Armed Mode

- **Boots in SAFE mode** by default.
- In **SAFE mode**: pump is blocked; entering SAFE mode immediately turns the pump off.
- In **ARMED mode**: pump operation is permitted.
- The controller web UI reads the mode from `/status` and displays a live **SAFE / ARMED** badge.

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/stream` | MJPEG multipart stream |
| `GET` | `/snapshot` | Single JPEG at `SNAP_FRAME_SIZE`, then restores stream resolution |
| `GET` | `/status` | JSON: `{ ip, uptime, heap, mode, pumpActive }` |
| `GET` | `/pump?action=press\|hold\|release` | Controls pump relay on GPIO 13 |
| `GET` | `/mode?set=safe\|armed` | Switches operating mode |

### Main Loop

```
setup():
  initCamera()
  connectWiFi()         <- 15 s timeout then ESP.restart()
  registerWithController()
  register HTTP routes
  server.begin()

loop():
  server.handleClient()
  if (millis() - lastDetectionMs >= DETECTION_INTERVAL_MS) {
    runDetectionAndPush()
  }
  // pump heartbeat watchdog (same pattern as controller held-pin)
```

---

## Future: Automatic Action

The detection overlay pipeline is designed to support future autonomous operation:

- Detections from on-device image processing are already available on the controller via `/overlay`
- The controller can read these detections in `loop()` and autonomously issue `pressPin` / `releasePin` calls without browser involvement
- The **SAFE / ARMED** mode gate on the cam ensures the pump (and by extension any autonomous actuator) cannot operate until explicitly armed
- Suggested detection algorithms: colour thresholding (simple), ArUco marker tracking (positioning), TFLite model inference (object recognition)
