# Excavator Robot – System Design Document

---

## Overview

This document describes the full firmware architecture for a two-module wireless excavator robot. One module is the **ESP32-CAM** (Joy-IT SBC-ESP32-Cam), which streams video and performs on-device image processing. The other is the **NodeMCU ESP32** (Berrybase NMCU-ESP32), which acts as the WiFi Access Point, hosts the web control console, and drives the physical remote-control buttons of the excavator via GPIO by trying to emulate presses. The controller runs on the Berrybase NodeMCU-ESP32 and the camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM), which connects to the controller's AP as a client, streams MJPEG video, runs on-device image processing, and pushes detection overlays to the controller. The camera also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

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
| `apriltag-esp32` | Camera only — install from the repo ZIP: [apriltag-esp32 master.zip](https://github.com/raspiduino/apriltag-esp32/archive/refs/heads/master.zip) or from the project repo copy if bundled locally |

ESP32 board package URL: `https://dl.espressif.com/dl/package_esp32_index.json`

### AprilTag Library Install Notes

Install the AprilTag dependency through Arduino IDE:

1. Download the ZIP from [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32) or use the ZIP bundled in this repo if present.
2. In Arduino IDE, open **Sketch → Include Library → Add .ZIP Library...**
3. Select the downloaded `apriltag-esp32` ZIP.
4. Restart Arduino IDE if the library does not appear immediately.

### AprilTag Compile Fix

Some Arduino / ESP32 toolchain combinations fail while compiling the installed AprilTag library with an error similar to:

```text
error: expected '=', ',', ';', 'asm' or '__attribute__' before 'svd22'
```

This happens because `IRAM_ATTR` is not recognized in `svd22.c`. Fix it by adding the ESP attribute header at the top of the library source file:

File:
```text
<Arduino sketchbook>/libraries/Apriltag_library_for_Arduino_ESP32/src/common/svd22.c
```

Add near the top:
```c
#include <esp_attr.h>
```

After adding that include, rebuild the ESP32-CAM sketch. This resolves the `IRAM_ATTR` compile error.

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

Pin definitions use a unified `PinDef` struct. Every GPIO is described by an explicit idle state and up to three action bindings (one per driven state). There is no separate mode enum — the `idle` field fully determines what `setPinIdle()` does, and each `actionXxx` field is either a button name string or `nullptr` (unbound).

`PinState_t` values:
- `HIGH` → `pinMode(OUTPUT); digitalWrite(HIGH)`
- `LOW`  → `pinMode(OUTPUT); digitalWrite(LOW)`
- `HiZ`  → `pinMode(INPUT)` — floating / high-impedance

```cpp
enum PinState_t { HIGH_STATE, LOW_STATE, HIZ_STATE };

struct PinDef {
  int         gpio;
  PinState_t  idle;        // state applied by setPinIdle()
  const char* actionHigh;  // button name that drives this pin HIGH   (nullptr = unbound)
  const char* actionHiZ;   // button name that drives this pin to HiZ (nullptr = unbound)
  const char* actionLow;   // button name that drives this pin LOW    (nullptr = unbound)
};
```

#### Typical Wiring Patterns

**MOSFET gate (track / turntable / arm / light):**
- `idle = HIGH` — MOSFET fully on, drain clamped to GND (track off / no button)
- `actionHiZ` drives the "forward / on" button via the internal analog path (Hi-Z gate)
- `actionLow` drives the "back / pulse" button by pulling the gate low

**Simple button (active-LOW):**
- `idle = HiZ` — pin floating, external pull-up holds line idle
- `actionLow` drives the button line to GND

#### MOSFET (N-channel, one per track/axis)

The excavator tracks are driven through a single N-channel MOSFET per track. One ESP32 GPIO controls three logical states:

| ESP32 GPIO state | MOSFET effect | Track action |
|-----------------|---------------|--------------|
| OUTPUT HIGH | Gate fully on → Drain–Source closed to GND | Track **off** (no button) |
| INPUT (Hi-Z) | Gate at intermediate voltage via internal paths | Track **forward** |
| OUTPUT LOW | Gate off → Drain–Source open | Track **back** |

**Wiring (per button pair):**
- Gate → Excavator remote "S" for shared. It's a mistery trace with a connection to the remote mistery chip, shared between half the buttons. (The other buttons connect to ground) It's mostly at ground, but has frequent spikes to HIGH that seem to be random and don't follow a pattern.
- Drain → ESP32 GPIO  
- Source → Excavator remote "P" for pair. It goes to the remote mistery chip and 2 buttons - one connected to ground and the other to S. It has a pullup. If directly driven to 0, it triggers the Button connected to ground.

### Pin Table

All physical GPIOs are declared in a single `PIN_TABLE[]` array. There is exactly one entry per GPIO:

| GPIO | idle | actionHigh | actionHiZ | actionLow | Notes |
|------|------|------------|-----------|-----------|-------|
| 13 | HIGH | — | `left_fwd` | `left_back` | Left track gate |
| 14 | HIGH | — | `right_fwd` | `right_back` | Right track gate |
| 26 | HIGH | — | `turn_left` | `turn_right` | Turntable |
| 33 | HIGH | — | `arm_fwd` | `arm_back` | Arm / bucket |
| 15 | HIGH | — | `light_on` | `light_off` | Light (Pulse) |
| 4 | HiZ | — | — | `test` | Test hold (active LOW) |

### Timing Constants

```cpp
const uint32_t PULSE_DURATION_MS = 150;   // Duration of light on/off pulse
const uint32_t HOLD_TIMEOUT_MS   = 300;   // Auto-release timeout when heartbeat stops
```

### Control Characteristics

- All controls are **binary** — fully on or fully off; no analog or PWM speed control.
- All controls have a mechanical response delay of approximately **0.5–2 s** after the GPIO signal changes.
- With camera-in-the-loop (detection → POST → decision → actuation), the total effective delay is even longer and must be accounted for in all autonomous timing logic.

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
| `arm_fwd` | Hold | Arm / bucket forward (GPIO33 Hi-Z) |
| `arm_back` | Hold | Arm / bucket backward (GPIO33 LOW) |
| `light` | Pulse | Toggle: pulses `light_on` or `light_off` pin |
| `test` | Hold | Active while held (GPIO4 LOW) |

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

The controller supports up to `MAX_HELD = 6` simultaneous held actions. Each slot is a `HeldAction` struct. `HeldPin` stores a pointer into `PIN_TABLE` plus a `targetState` (the `PinState_t` to drive when active):

```cpp
struct HeldPin {
  const PinDef* pin;
  PinState_t    targetState;  // which state to drive this pin to when active
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
// Put a pin in its idle / safe state (drives p.idle: HIGH, LOW, or HiZ)
void setPinIdle(const PinDef& p);

// Activate a pin to a specific state
void setPinActive(const PinDef& p, PinState_t targetState);

void releaseAllPins();  // iterates PIN_TABLE, calls setPinIdle; clears all heldActions

// Pulse a pin to activeState for durationMs, then return to idle
void pulsePin(const PinDef& p, PinState_t activeState, uint32_t durationMs);
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
| `GET` | `/auto_status` | Returns `{autoEnabled, status}` — status is `"waiting_for_detection"` or `"performing_action"` |

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
- Live **SAFE / ARMED** badge polled directly from the cam's `/status` endpoint
- Live **AUTO** status badge polled from `/auto_status` every 500 ms — shows `OFF`, `WAITING`, or `ACTING`
- All held actions are released on both `pointerup` and `window blur` (tab loses focus), as a second safety path

#### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` | Both tracks forward |
| `S` | Both tracks back |
| `A` | Spin left (counter-clockwise) |
| `D` | Spin right (clockwise) |
| `←` Arrow Left | Turntable rotate left |
| `→` Arrow Right | Turntable rotate right |
| `↑` Arrow Up | Arm forward (`arm_fwd`) |
| `↓` Arrow Down | Arm backward (`arm_back`) |
| `L` | Light toggle (pulse) |
| `T` | Test hold |
| `Space` | Pump hold (relayed to cam via `/camcmd`) |

#### Overlay Canvas

The browser polls `/overlay` every 500 ms. Detection boxes are drawn with label and confidence percentage:
- `red_dot` → red (`#ff3030`)
- `qr_*` (AprilTag) → cyan (`#00e5ff`)
- Everything else → amber (`#f0a500`)

The canvas is sized to match the feed container on every resize event.

#### Camera Feed Polling

The browser polls `/camip` every 2000 ms. When the cam's IP is received, `feedImg.src` is set to `http://<camIP>/stream`. On stream error the feed retries after 3 s.

### Main Loop

```
setup():
  releaseAllPins()      <- ensure all pins start in idle state
                           (MOSFET gates → OUTPUT HIGH / track off; simple buttons → Hi-Z)
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
const framesize_t STREAM_FRAME_SIZE = FRAMESIZE_VGA;      // 640×480
const framesize_t SNAP_FRAME_SIZE   = FRAMESIZE_VGA;      // 640×480
const uint8_t     JPEG_QUALITY      = 12;                 // 0 = best ... 63 = worst; 10-15 sweet spot
const int         STREAM_DELAY_MS   = 0;                  // extra inter-frame delay (ms)
```

### Pixel Format

The camera is initialised with `PIXFORMAT_JPEG`. The MJPEG stream sends frames directly. For detection, `fmt2rgb888()` decodes each JPEG frame into a raw RGB888 buffer in PSRAM before colour thresholding or AprilTag preprocessing. This avoids keeping a second raw frame buffer resident at all times.

### `/snapshot` Endpoint

`handleSnapshot()` checks `fb->format` before encoding:
- If `PIXFORMAT_JPEG`: `fb->buf` sent directly via `send_P`; `fb` returned before the send to free the buffer slot for the stream task sooner.
- Otherwise: `frame2jpg()` called with `JPEG_QUALITY` to convert raw pixel data to JPEG, then buffer is freed after send.

`frame2jpg()` expects raw pixel data (RGB/YUV). Calling it on an already-JPEG frame produces corrupted output and must be avoided.

### Concurrent Stream and Detection

The MJPEG stream runs in a dedicated FreeRTOS task (pinned to core 1). Detection runs in `loop()` on core 1 as well, every `DETECTION_INTERVAL_MS`. Both call `esp_camera_fb_get()` independently. With `fb_count = 2` (PSRAM path), the camera driver maintains two frame buffer slots and hands them out to concurrent callers without conflict. Detection is only suppressed while the pump is actively firing (`!pumpActive`) to avoid contention during a critical actuation window.

```cpp
const uint32_t DETECTION_INTERVAL_MS = 500;   // Detection runs every 500 ms, independent of stream state
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
    { "label": "red_dot", "x": 0.30, "y": 0.20, "w": 0.10, "h": 0.15, "confidence": 0.87 },
    { "label": "qr_7",    "x": 0.42, "y": 0.18, "w": 0.22, "h": 0.29, "confidence": 0.81 }
  ],
  "ts": 12345
}
```

#### Red Dot Detection (RGB888 Colour Thresholding)

The active colour detector finds a bright red dot using per-pixel RGB888 thresholding followed by a simple bounding-box blob finder. The JPEG frame is decoded to RGB888 via `fmt2rgb888()` before thresholding.

**Thresholds** (tuned for a bright red dot, 8-bit per channel):

| Channel | Threshold | Rationale |
|---------|-----------|-----------|
| R | > 160 | Saturated red |
| G | < 80 | Low green |
| B | < 80 | Low blue |

**Blob finder**: Iterates all pixels, accumulates a bounding box (`bx1`, `by1`, `bx2`, `by2`) and pixel count for matching pixels. If `blobCount >= 30`, a `Detection` is emitted with label `"red_dot"`, normalised bounding box coordinates, and confidence scaled as `min(1.0, blobCount / 500.0)`.

**Serial output** on detection:
```
[RED] Detected @ (x,y) size WxH conf=C
```

#### AprilTag Detection (tag16h5, raspiduino/apriltag-esp32)

AprilTag detection (family `tag16h5`) is used for marker-based target recognition and positioning. The library is [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32), installed from ZIP in Arduino IDE.

**Detector configuration** (set once in `initAprilTag()`):

| Parameter | Value | Notes |
|-----------|-------|-------|
| `quad_decimate` | 2.0 | Downsample 2× before quad detection — faster, fine at VGA |
| `quad_sigma` | 0.0 | No blur — camera is already soft enough |
| `nthreads` | 1 | ESP32 single-core detection task |
| `refine_edges` | 1 | Sub-pixel edge refinement — worth it for low-res images |
| `debug` | 0 | No debug image output |

**Processing steps:**
1. Capture JPEG frame and decode to RGB888 via `fmt2rgb888()`
2. Build grayscale buffer: plain BT.601 luma (`(r*77 + g*150 + b*29) >> 8`) by default. Set `GRAY_ACHROMATIC_FILTER = true` to instead map coloured pixels to 255 before detection (legacy ArUco behaviour; generally not needed — AprilTag's adaptive binarization handles colour noise internally)
3. Wrap in `image_u8_t` and call `apriltag_detector_detect()`
4. For each result: extract bounding box from the four corner points (`det->p[0..3]`), populate a `Detection` with label `"qr_<id>"`, normalised coords, and `decision_margin / 100.0` as confidence
5. Call `apriltag_detections_destroy()` to free library-allocated results

**Grayscale filter constants:**

```cpp
const bool    GRAY_ACHROMATIC_FILTER = false;  // false = plain luma (recommended)
const uint8_t GRAY_SAT_MAX           = 30;     // saturation threshold (used only when filter = true)
```

**Serial output** on detection:
```
[AT] Detections: N
[AT] ID N @ (x,y) size WxH margin=M
```

**Detection labels:** `qr_<id>` where `<id>` is the raw integer from the `tag16h5` dictionary (IDs 0–29, 30 valid tags).

**Performance notes:**
- `quad_decimate = 2.0` halves the resolution before quad detection — the primary speed lever. Increase to `4.0` if heap/CPU is tight.
- `refine_edges = 1` recovers sub-pixel accuracy after decimation; recommended at VGA.
- AprilTag's internal adaptive thresholding handles most lighting conditions without a pre-filter.

**Known limitations:**
- Designed for one or a few markers in frame at moderate distance
- Very small markers (< ~40 px per side after decimation) may not be detected reliably
- RAM usage: the library allocates intermediate buffers from heap; PSRAM allocation path in `runDetectionAndPush()` is used for the gray buffer

#### Extension Points

1. Capture frame suited to algorithm (`STREAM_FRAME_SIZE` for all current detectors)
2. Run detection algorithm
3. Populate the `detections` vector with `Detection` structs
4. JSON serialisation and POST to `/overlay` are reused unchanged

Current algorithms:
- Colour thresholding for `red_dot` (implemented)
- AprilTag `tag16h5` marker tracking for `qr_N` IDs (implemented, raspiduino/apriltag-esp32)

Future candidates:
- Multi-colour blob tracking
- TFLite Micro model inference
- Optical flow / motion tracking
- Multi-family AprilTag support (e.g. `tag36h11`)

### Pump Control

```cpp
const int PIN_PUMP = 4;  // Temp for LED; change digitalWrite to LOW for real pump relay
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800;
```

### Safe / Armed Mode

- **Boots in SAFE mode** by default.
- In **SAFE mode**: pump is blocked; entering SAFE mode immediately turns the pump off.
- In **ARMED mode**: pump operation is permitted.
- The controller web UI reads the mode from the cam's `/status` endpoint and displays a live **SAFE / ARMED** badge.

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/stream` | MJPEG multipart stream (live, full FPS) |
| `GET` | `/snapshot` | Single JPEG at `SNAP_FRAME_SIZE` |
| `GET` | `/status` | JSON: `{ ip, uptime, heap, mode, pumpActive }` |
| `GET` | `/pump?action=press\|hold\|release` | Controls pump relay |
| `GET` | `/mode?set=safe\|armed` | Switches operating mode |
| `GET` | `/detection_status` | JSON: `{ marker_detected, marker_id, ts }` — latched for `MARKER_CLEAR_MS` |

### Main Loop

```
setup():
  initCamera()          <- PIXFORMAT_JPEG, fb_count=2 (PSRAM)
  initAprilTag()        <- tag16h5 family, quad_decimate=2.0, refine_edges=1
  connectWiFi()         <- 15 s timeout then ESP.restart()
  registerWithController()
  register HTTP routes
  server.begin()

loop():
  server.handleClient()
  // pump heartbeat watchdog
  if pumpActive and timeout: pumpOff()
  // detection — runs independently of stream; suppressed only while pump is active
  if (!pumpActive && millis() - lastDetectionMs >= DETECTION_INTERVAL_MS):
    runDetectionAndPush()   // posts detection JSON to controller /overlay
```

---

## Future: Automatic Action

The detection overlay pipeline is designed to support future autonomous operation:

- Detections from on-device image processing are already available on the controller via `/overlay`
- The controller can read these detections in `loop()` and autonomously issue `setPinActive` / `setPinIdle` calls without browser involvement
- The **SAFE / ARMED** mode gate on the cam ensures the pump (and by extension any autonomous actuator) cannot operate until explicitly armed
- The controller is the single decision-maker: the cam detects and reports, the controller acts. This keeps all actuation logic in one place.

### Auto-Drive Status

The controller exposes `/auto_status` — polled by the browser every 500 ms. When `autoEnabled` is false the status reads `"waiting_for_detection"` as a neutral idle state. When a detection triggers autonomous actuation, status switches to `"performing_action"` for the duration of the blocking action sequence, then returns to `"waiting_for_detection"`. The dashboard displays this live so the operator always knows whether the controller is idle or actively executing a move.

A practical first autonomous mode for this project is **marker-guided seek-and-shoot**:

1. Camera detects an AprilTag marker and reports `qr_N` with normalised bounding box
2. Controller reads detection centre X position from `/overlay`
3. Controller steers left or right until marker is centred in frame
4. When marker is centred and confidence is high enough, controller can stop drive motion and activate the pump
5. All actuation still remains gated by SAFE / ARMED mode and controlled only by the controller

Suggested detection algorithms now implemented:
- Colour thresholding (`red_dot`) — simple, implemented
- AprilTag `tag16h5` marker tracking (`qr_N`) — positioning and target ID, implemented (raspiduino/apriltag-esp32)
- TFLite Micro model inference — future object recognition

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
