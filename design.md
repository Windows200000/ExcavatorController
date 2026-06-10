# Excavator Robot – System Design Document

---

## Overview

This document describes the full firmware architecture for a two-module wireless excavator robot. One module is the **ESP32-CAM** (Joy-IT SBC-ESP32-Cam), which streams video and performs on-device image processing. The other is the **NodeMCU ESP32** (Berrybase NMCU-ESP32), which acts as the WiFi Access Point, hosts the web control console, and drives the physical remote-control buttons of the excavator via GPIO. The controller runs on the Berrybase NodeMCU-ESP32 and the camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM), which connects to the controller's AP as a client, streams MJPEG video, runs on-device image processing, and pushes detection overlays to the controller. The camera also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

---

## Module Roles

| Module | Hardware | Role |
|--------|----------|------|
| Controller | Berrybase NodeMCU-ESP32 | WiFi AP · Web console · GPIO button driver · Overlay store |
| Camera | Joy-IT SBC-ESP32-Cam | WiFi client · MJPEG streamer · Image processing · Overlay pusher |

---

## Network Topology

```
[ Browser ]
    |  HTTP (port 80)
    |
[ NodeMCU-ESP32 – Controller ]   192.168.4.1
    WiFi AP: ExcavatorAP
    |
    |  WiFi (STA <- AP)
    |
[ ESP32-CAM – Camera ]           192.168.4.x (DHCP)
```

All three parties communicate over the single `ExcavatorAP` network. The browser connects to the AP and addresses both devices via HTTP. The cam never receives commands directly from the browser – all relayed commands go through the controller's `/camcmd` endpoint.

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
    ->  Controller: setPinActive / setPinIdle
    ->  Physical GPIO drives remote-control button line
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

### Autonomous Action (future)

```
ESP32-CAM  ->  POST /overlay (detections)  ->  Controller
Controller ->  reads /overlay in loop()    ->  setPinActive / setPinIdle autonomously
```

The controller is the single decision-maker. The cam detects and reports; the controller acts. This keeps all actuation logic in one place and means the cam never needs to know about motors or buttons.

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

## Controller – NodeMCU ESP32

The controller runs on the Berrybase NodeMCU-ESP32. It hosts the WiFi Access Point, serves the web console, stores detection overlays pushed by the camera, and drives the physical remote-control buttons of the excavator via GPIO. It also relays pump and mode commands to the camera module via the `/camcmd` endpoint.

### WiFi / Network Constants

```cpp
const char* AP_SSID       = "ExcavatorAP";
const char* AP_PASSWORD   = "exc@vator123";
const IPAddress AP_IP     (192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET (255, 255, 255, 0);
```

### GPIO Pin Architecture

> **Important:** GPIO 34, 35, 36, 39 are **INPUT-ONLY** on the ESP32 — they have no output driver and will silently fail. Safe output-capable GPIOs on the NodeMCU-ESP32: 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33.

Pin definitions use a unified `PinDef` struct with a `PinMode_t` enum that selects between simple push-button emulation and MOSFET-driven track control:

```cpp
enum PinMode_t { PIN_SIMPLE, PIN_MOSFET };

struct PinDef {
  int         gpio;
  PinMode_t   mode;
  // PIN_SIMPLE fields
  const char* action;       // logical button name, e.g. "up", "turn_left"
  int         activeLevel;  // HIGH or LOW
  // PIN_MOSFET fields
  const char* actionHigh;   // OUTPUT HIGH state (track off); nullptr = no binding
  const char* actionHiZ;    // INPUT (Hi-Z) state  → track forward
  const char* actionLow;    // OUTPUT LOW  state  → track back
};
```

#### PIN_SIMPLE

Used for all buttons except the track MOSFET gates. `setPinActive()` drives OUTPUT + `activeLevel`; `setPinIdle()` returns the pin to INPUT (Hi-Z), letting the external pull-up/pull-down restore the idle state.

#### PIN_MOSFET (N-channel MOSFET per track)

The excavator tracks are driven through a single N-channel MOSFET per track. One ESP32 GPIO controls three logical states:

| ESP32 GPIO state | MOSFET effect | Track action |
|-----------------|---------------|--------------|
| OUTPUT HIGH | Gate fully on → Drain–Source closed to GND | Track **off** (no button) |
| INPUT (Hi-Z) | Gate at intermediate voltage via internal paths | Track **forward** |
| OUTPUT LOW | Gate off → Drain–Source open | Track **back** |

**Wiring (per track):**
- Gate → ESP32 GPIO (series resistor ~1 kΩ from remote S node)
- Drain → excavator remote P node
- Source → excavator remote G (GND), tied to ESP32 GND

`setPinIdle()` on a MOSFET pin drives OUTPUT HIGH (track off). `setPinActive()` uses the `useHiZ` flag: `true` → INPUT (forward), `false` → OUTPUT LOW (back).

### Pin Table

All physical GPIOs are declared in a single `PIN_TABLE[]` array. There is exactly one entry per GPIO:

| GPIO | Mode | Action(s) | Notes |
|------|------|-----------|-------|
| 13 | MOSFET | HiZ=`left_fwd`, LOW=`left_back` | Left track gate |
| 14 | MOSFET | HiZ=`right_fwd`, LOW=`right_back` | Right track gate |
| 26 | SIMPLE | `turn_left` (HIGH) | Turntable left |
| 25 | SIMPLE | `turn_right` (LOW) | Turntable right |
| 33 | SIMPLE | `up` (LOW) | Arm / bucket up |
| 32 | SIMPLE | `down` (HIGH) | Arm / bucket down |
| 15 | SIMPLE | `light_on` (LOW) | Light ON pulse |
| 2 | SIMPLE | `light_off` (HIGH) | Light OFF pulse |
| 4 | SIMPLE | `test` (HIGH) | Test hold |

### Timing Constants

```cpp
const uint32_t PULSE_DURATION_MS = 150;   // Duration of light on/off pulse
const uint32_t HOLD_TIMEOUT_MS   = 300;   // Auto-release timeout when heartbeat stops
```

### Button Behaviour

| Button | Type | Behaviour |
|--------|------|-----------|
| `left_fwd` | Hold | Left track forward (GPIO13 Hi-Z) |
| `left_back` | Hold | Left track back (GPIO13 LOW) |
| `right_fwd` | Hold | Right track forward (GPIO14 Hi-Z) |
| `right_back` | Hold | Right track back (GPIO14 LOW) |
| `fwd` | Hold | Both tracks forward simultaneously |
| `back` | Hold | Both tracks back simultaneously |
| `spin_left` | Hold | Left back + Right forward (counter-clockwise) |
| `spin_right` | Hold | Left forward + Right back (clockwise) |
| `turn_left` | Hold | Turntable rotate left |
| `turn_right` | Hold | Turntable rotate right |
| `up` | Hold | Arm / bucket up |
| `down` | Hold | Arm / bucket down |
| `light` | Pulse | Toggle: pulses `light_on` or `light_off` pin |
| `test` | Hold | Active while held |

Up to `MAX_HELD = 6` simultaneous held actions are supported.

#### Composite Button Actions

Composite buttons fan out to multiple `HeldPin` entries via recursive calls to `buttonToHeldPins()`:

| Button | HeldPins driven | Effect |
|--------|-----------------|--------|
| `fwd` | `left_fwd` + `right_fwd` | Both tracks forward |
| `back` | `left_back` + `right_back` | Both tracks back |
| `spin_left` | `left_back` + `right_fwd` | Counter-clockwise spin |
| `spin_right` | `left_fwd` + `right_back` | Clockwise spin |

### Multi-Button Hold State

The controller supports up to `MAX_HELD = 6` simultaneous held actions. Each slot is a `HeldAction` struct. `HeldPin` stores a pointer into `PIN_TABLE` plus a `useHiZ` flag (relevant for MOSFET pins):

```cpp
struct HeldPin {
  const PinDef* pin;
  bool          useHiZ;  // MOSFET: true=INPUT(fwd), false=LOW(back)
};

struct HeldAction {
  String   name;
  HeldPin  pins[4];
  int      pinCount = 0;
  uint32_t lastSeen = 0;
  bool     active   = false;
};
HeldAction heldActions[MAX_HELD];
```

### Pin-Control Pattern

```cpp
// Put a pin in its idle / safe state
void setPinIdle(const PinDef& p);
  // SIMPLE  → pinMode(INPUT)
  // MOSFET  → pinMode(OUTPUT); digitalWrite(HIGH)  [track off]

// Activate a pin
void setPinActive(const PinDef& p, bool hiZForMosfet);
  // SIMPLE         → pinMode(OUTPUT); digitalWrite(activeLevel)
  // MOSFET + hiZ   → pinMode(INPUT)                [track forward]
  // MOSFET + !hiZ  → pinMode(OUTPUT); digitalWrite(LOW) [track back]

void releaseAllPins();  // iterates PIN_TABLE, calls setPinIdle; clears all heldActions
void pulseSimplePin(const PinDef& p, uint32_t durationMs);  // SIMPLE pins only
```

### Heartbeat Watchdog

The browser sends a heartbeat `action=hold` every 150 ms while a key or button is held. If no heartbeat arrives within `HOLD_TIMEOUT_MS` (300 ms), the controller auto-releases all pins for that action and clears the slot. This prevents the excavator from running away if the browser tab closes or network drops.

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
| `GET` | `/status` | Returns `{camIP, uptime, heap, light}` |

#### `/cmd` Action Values

- `press` – start holding the pin; begins heartbeat window
- `hold` – heartbeat renewal; resets the `HOLD_TIMEOUT_MS` watchdog
- `release` – release the pin immediately (back to idle state)

### Web Console

The HTML console is stored in flash as `INDEX_HTML[]` (`PROGMEM`). It includes:

- An **MJPEG feed** `<img>` whose `src` is set dynamically after `/camip` returns the cam's address
- A `<canvas>` overlay rendered on top of the feed for detection bounding boxes
- Four control groups: **Drive**, **Turret**, **Arm**, **Aux**
- Each button shows an icon, a label, and a keyboard shortcut badge
- A top-down excavator model SVG that glows live for tracks / turntable / arm / pump / test / light
- Live **SAFE / ARMED** badge polled from cam `/status` endpoint

#### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` | Both tracks forward |
| `S` | Both tracks back |
| `A` | Spin left (counter-clockwise) |
| `D` | Spin right (clockwise) |
| `←` Arrow Left | Turntable rotate left |
| `→` Arrow Right | Turntable rotate right |
| `↑` Arrow Up | Arm / bucket up |
| `↓` Arrow Down | Arm / bucket down |
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
  releaseAllPins()      <- ensure all pins start in idle state
                           (SIMPLE → Hi-Z, MOSFET → OUTPUT HIGH / track off)
  WiFi.softAP(...)
  register HTTP routes
  server.begin()

loop():
  server.handleClient()
  // Heartbeat watchdog — iterate all heldActions slots
  for each heldActions[i] where active:
    if (millis() - lastSeen) > HOLD_TIMEOUT_MS:
      auto-release all pins for that action via setPinIdle()
      heldActions[i].active = false
```

---

## Camera Module – Joy-IT SBC-ESP32-Cam

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
|-------------------|-------------------|
| 5V | 5V |
| GND | GND |
| U0T (IO1) | RX |
| U0R (IO3) | TX |

### Stream / Quality Constants

```cpp
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;    // 640×480
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;    // 640×480
const uint8_t     JPEG_QUALITY      = 12;   // 0 = best ... 63 = worst; 10-15 sweet spot
const int         STREAM_DELAY_MS   = 0;    // extra inter-frame delay (ms)
```

### Pixel Format

The camera is initialised with `PIXFORMAT_JPEG`. The MJPEG stream sends frames directly. For detection, `fmt2rgb888()` decodes each JPEG frame into a raw RGB888 buffer in PSRAM before colour thresholding. This avoids keeping a second raw frame buffer resident at all times.

### Concurrent Stream and Detection

The MJPEG stream runs in a dedicated FreeRTOS task (pinned to core 1). Detection runs in `loop()` on core 1 as well, every `DETECTION_INTERVAL_MS`. Both call `esp_camera_fb_get()` independently. With `fb_count = 2` (PSRAM path), the camera driver maintains two frame buffer slots and hands them out to concurrent callers without conflict. Detection is only suppressed while the pump is actively firing (`!pumpActive`) to avoid contention during a critical actuation window.

```cpp
const uint32_t DETECTION_INTERVAL_MS = 500;   // Detection runs every 500 ms regardless of stream state
```

### Image Processing

Detection runs on-device at `DETECTION_INTERVAL_MS` intervals. Results are POSTed as JSON to the controller's `/overlay` endpoint:

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
    { "label": "red_dot", "x": 0.30, "y": 0.20, "w": 0.10, "h": 0.15, "confidence": 0.87 }
  ],
  "ts": 12345
}
```

#### Red Dot Detection (RGB888 Colour Thresholding)

The active detection algorithm finds a bright red dot using per-pixel RGB888 colour thresholding followed by a simple bounding-box blob finder. The JPEG frame is decoded to RGB888 via `fmt2rgb888()` before thresholding.

**Thresholds** (tuned for a bright red dot, 8-bit per channel):

| Channel | Threshold | Rationale |
|---------|-----|-----------|
| R | > 160 | Saturated red |
| G | < 80 | Low green |
| B | < 80 | Low blue |

**Blob finder**: Iterates all pixels, accumulates a bounding box (bx1, by1, bx2, by2) and pixel count for matching pixels. If `blobCount >= MIN_BLOB` (30), a `Detection` is emitted with label `"red_dot"`, normalised bounding box coordinates, and confidence scaled as `min(1.0, blobCount / 500.0)`.

**Serial output** on detection:
```
[PROC] Red dot @ (x,y) size WxH conf=C
```

#### Extension Points

1. Capture a frame (`fb->buf`, `fb->len`, `fb->width`, `fb->height`)
2. Run algorithm: edge detection, blob tracking, TFLite model inference, ArUco markers, colour thresholding, etc.
3. Populate the `detections` vector with `Detection` structs
4. The rest is handled automatically (JSON serialisation + POST to `/overlay`)

### Pump Control

```cpp
const int PIN_PUMP = 4;  // Temp for LED; change digitalWrite to LOW for real pump relay
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;
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
| `GET` | `/snapshot` | Single JPEG at `SNAP_FRAME_SIZE` |
| `GET` | `/status` | JSON: `{ ip, uptime, heap, mode, pumpActive }` |
| `GET` | `/pump?action=press\|hold\|release` | Controls pump relay |
| `GET` | `/mode?set=safe\|armed` | Switches operating mode |

### Main Loop

```
setup():
  initCamera()          <- PIXFORMAT_JPEG, fb_count=2 (PSRAM)
  connectWiFi()         <- 15 s timeout then ESP.restart()
  registerWithController()
  register HTTP routes
  server.begin()

loop():
  server.handleClient()
  // pump heartbeat watchdog
  if pumpActive and timeout: pumpOff()
  // detection — always runs, independent of stream
  if (!pumpActive && millis() - lastDetectionMs >= DETECTION_INTERVAL_MS):
    runDetectionAndPush()
```

---

## Future: Automatic Action

The detection overlay pipeline is designed to support future autonomous operation:

- Detections from on-device image processing are already available on the controller via `/overlay`
- The controller can read these detections in `loop()` and autonomously issue `setPinActive` / `setPinIdle` calls without browser involvement
- The **SAFE / ARMED** mode gate on the cam ensures the pump (and by extension any autonomous actuator) cannot operate until explicitly armed
- The controller is the single decision-maker: the cam detects and reports, the controller acts. This keeps all actuation logic in one place.
- Suggested detection algorithms: colour thresholding (simple, implemented), ArUco marker tracking (positioning), TFLite model inference (object recognition)

---

## Tester – ESP32 ADC/DAC Oscilloscope

A separate `tester/tester.ino` sketch turns the NodeMCU-ESP32 into a high-speed
single-channel oscilloscope and DAC playground for debugging the excavator's
hydraulic and electrical signals.

- WiFi: runs its own AP `ExcavatorAP` on 192.168.4.1, matching the controller.
- ADC: GPIO34 in single-shot mode at ~50 kSa/s (`SAMPLE_INTERVAL_US = 20`).
- Delta compression: samples are only stored when the raw ADC code changes by
  at least 16 LSBs (≈12.9 mV at 3.3 V / 12‑bit). The ESP32 ring buffer holds
  4,000 stored samples, which is more than 4 seconds of activity at this
  threshold even at maximum sampling rate.
- Transport: `/samples` returns JSON of compressed samples as
  `[seq, time_us, raw]` plus `sampleIntervalUs`.
- DAC: GPIO25 is driven via `/dac?value=N` and rendered as a horizontal
  reference line in the browser.

The browser decompresses the delta-encoded stream back into a staircase
waveform before drawing and CSV export. For every compressed sample after
the first it injects an extra "hold" point one sampling interval before the
new sample at the previous voltage level, so each plateau is represented
by exactly two points: a start and an end. The in-browser buffers retain
the full decompressed timeline since page load or last Clear and the
Download CSV button exports this full history.
