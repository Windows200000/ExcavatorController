# Excavator Robot – System Design Document

---

## Overview

This document describes the full firmware architecture for a two-module wireless excavator robot. One module is the **ESP32-CAM** (Joy-IT SBC-ESP32-Cam), which streams video and performs on-device image processing. The other is the **NodeMCU ESP32** (Berrybase NMCU-ESP32), which acts as the WiFi Access Point, hosts the web control console, and drives the physical remote-control buttons of the excavator via GPIO by trying to emulate presses. The controller runs on the Berrybase NodeMCU-ESP32 and the camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM), which connects to the controller's AP as a client, streams MJPEG video, runs on-device image processing, and pushes detection overlays to the controller. The camera also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

---

## Module Roles

| Module | Hardware | Role |
|--------|----------|------|
| Controller | Berrybase NodeMCU-ESP32 | WiFi AP · Web console · GPIO button driver · Overlay store |
| Camera | Joy-IT SBC-ESP32-Cam | WiFi client · Snapshot-based MJPEG streamer · Image processing · Overlay pusher |

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
ESP32-CAM  ->  /stream (snapshot-based MJPEG)  ->  Browser <img> tag
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

### Autonomous Action

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
enum PinState_t { _HIGH, _LOW, _HiZ };

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

### Pin Table ("Weirdo Mode" — current wiring)

The current wiring uses **separate GPIOs per direction** rather than one GPIO per axis in the classic MOSFET pattern. Each direction has its own dedicated pin. Idle states are chosen so the pin is electrically passive when no action is active.

All physical GPIOs are declared in a single `PIN_TABLE[]` array. There is exactly one entry per GPIO:

| GPIO | idle | actionHigh | actionHiZ | actionLow | Notes |
|------|------|------------|-----------|-----------|-------|
| 12 | LOW | `left_fwd` | — | — | Left track forward |
| 26 | LOW | `right_fwd` | — | — | Right track forward |
| 33 | LOW | `arm_dwn` | — | — | Arm forward (up) |
| 14 | HiZ | — | — | `left_back` | Left track back |
| 25 | HiZ | — | — | `right_back` | Right track back |
| 32 | HiZ | — | — | `arm_up` | Arm back (down) |
| 16 | LOW | `turn_left` | — | — | Turntable left |
| 17 | HiZ | — | — | `turn_right` | Turntable right |
| 18 | LOW | `light_on` | — | — | Light on pulse |
| 19 | HiZ | — | — | `light_of` | Light off pulse |
| 4 | HiZ | — | — | `test` | Test pin (active LOW pulse) |

> **Note:** The classic single-MOSFET-per-axis wiring (GPIOs 13, 14, 26, 33, 15) is preserved as commented-out entries in the source for reference.

### Timing Constants

```cpp
const uint32_t PULSE_DURATION_MS    = 300;   // Duration of light/test pulse
const uint32_t HOLD_TIMEOUT_MS      = 300;   // Auto-release timeout when heartbeat stops
const uint32_t PUMP_HB_INTERVAL_MS  = 200;   // Pump hold heartbeat sent from loop() while firing
```

`PUMP_HB_INTERVAL_MS` is independent of the detection cadence. `loop()` sends `/pump?action=hold` to the cam every 200 ms while `autoState.is_firing` is true, preventing the cam's `PUMP_HOLD_TIMEOUT_MS = 800 ms` watchdog from releasing the pump between detection frames.

### Control Characteristics

- All controls are **binary** — fully on or fully off; no analog or PWM speed control.
- All controls have a mechanical response delay of approximately **0.5–2 s** after the GPIO signal changes.
- With camera-in-the-loop (detection → POST → decision → actuation), the total effective delay is even longer and must be accounted for in all autonomous timing logic.

### Toggle State Variables

Two boolean variables track the software state of pulse-based toggles. These are updated by `handleCmd` on every `press` action and reported via `/status`:

```cpp
bool lightOn = false;  // true when light is currently on
bool testOn  = false;  // true when test was last toggled on
```

Because both controls are **pulse-based** (the hardware toggle is a momentary GPIO pulse, not a sustained level), the ESP32 tracks state internally in software.

### Button Behaviour

| Button | Type | Behaviour |
|--------|------|-----------|
| `left_fwd` | Hold | Left track forward (GPIO 12 HIGH) |
| `left_back` | Hold | Left track back (GPIO 14 LOW) |
| `right_fwd` | Hold | Right track forward (GPIO 26 HIGH) |
| `right_back` | Hold | Right track back (GPIO 25 LOW) |
| `fwd` | Hold | Both tracks forward simultaneously |
| `back` | Hold | Both tracks back simultaneously |
| `spin_left` | Hold | Left back + Right forward (counter-clockwise) |
| `spin_right` | Hold | Left forward + Right back (clockwise) |
| `turn_left` | Hold | Turntable rotate left (GPIO 16 HIGH) |
| `turn_right` | Hold | Turntable rotate right (GPIO 17 LOW) |
| `arm_dwn` | Hold | Arm forward / up (GPIO 33 HIGH) |
| `arm_up` | Hold | Arm back / down (GPIO 32 LOW) |
| `light` | Toggle/Pulse | Pulses `light_on` (GPIO 18 HIGH) or `light_of` (GPIO 19 LOW) depending on tracked `lightOn` state |
| `test` | Toggle/Pulse | Pulses GPIO 4 LOW for `PULSE_DURATION_MS`; flips tracked `testOn` state each press |

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
| `GET` | `/status` | Returns `{camIP, uptime, heap, light, test}` |
| `GET` | `/auto_status` | Returns `{autoEnabled, status}` — status is `"waiting_for_detection"` or `"performing_action"` |
| `GET` | `/auto?set=on\|off` | Enable / disable autonomous mode |

#### `/cmd` Action Values

- `press` – for hold buttons: starts holding the pin and begins the heartbeat window. For toggle buttons (`light`, `test`): pulses the pin and flips the internal state.
- `hold` – heartbeat renewal; resets the `HOLD_TIMEOUT_MS` watchdog (hold buttons only)
- `release` – release the pin immediately (back to idle state; hold buttons only)

#### `/status` Response

```json
{
  "camIP":  "192.168.4.x",
  "uptime": 123,
  "heap":   182432,
  "light":  false,
  "test":   false
}
```

`light` and `test` reflect the current software-tracked toggle state. The browser re-reads this endpoint 500 ms after any toggle press to reconcile UI state with the server.

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

| Key | Action | Type |
|-----|--------|------|
| `W` | Both tracks forward | Hold |
| `S` | Both tracks back | Hold |
| `A` | Spin left (counter-clockwise) | Hold |
| `D` | Spin right (clockwise) | Hold |
| `←` Arrow Left | Turntable rotate left | Hold |
| `→` Arrow Right | Turntable rotate right | Hold |
| `↑` Arrow Up | Arm up (`arm_dwn`) | Hold |
| `↓` Arrow Down | Arm down (`arm_up`) | Hold |
| `L` | Light toggle (pulse) | Toggle |
| `T` | Test toggle (pulse) | Toggle |
| `Space` | Pump hold (relayed to cam via `/camcmd`) | Hold |

#### Toggle Button Behaviour (Light & Test)

Both `light` and `test` are **toggle buttons** — a single press pulses the hardware pin and flips the tracked state. The UI does **not** use an optimistic flip; instead, after sending `cmd(action, 'press')` the browser waits `TOGGLE_SYNC_MS = 500 ms` then fetches `/status` and reconciles both toggle button visuals and SVG indicator to match the server-reported state. This ensures the UI stays correct even if a request is lost.

#### Overlay Canvas

The browser polls `/overlay` every 500 ms. Detection boxes are drawn with label and confidence percentage:
- `marker_*` (AprilTag) → cyan (`#00e5ff`)
- `red_dot` → red (`#ff3030`)
- Everything else → amber (`#f0a500`)

The canvas is sized to match the feed container on every resize event.

#### Camera Feed Polling

The browser polls `/camip` every 2000 ms. When the cam's IP is received, `feedImg.src` is set to `http://<camIP>/stream`. On stream error the feed retries after 3 s.

### Main Loop

```
setup():
  releaseAllPins()      <- ensure all pins start in idle state
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
  autoDefaultPosition()
  // Pump hold heartbeat — keeps cam pump relay alive during blocking auto actions
  if autoState.is_firing and (millis() - lastPumpHbMs) >= PUMP_HB_INTERVAL_MS:
    relayCamCmd("/pump?action=hold")
    lastPumpHbMs = millis()
```

---

## Camera Module – Joy-IT SBC-ESP32-Cam

The camera module runs on the Joy-IT SBC-ESP32-Cam (AI-Thinker ESP32-CAM). It connects to the controller's WiFi AP as a client, streams MJPEG video via a snapshot-based architecture, runs on-device AprilTag detection, and pushes detection overlays to the controller. It also exposes pump control and safe/armed mode endpoints, which the controller relays to from the browser.

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

### Timing Constants

```cpp
const framesize_t FRAME_SIZE             = FRAMESIZE_VGA;   // 640×480
const uint8_t     JPEG_QUALITY           = 12;              // 0=best … 63=worst
const uint32_t    DETECTION_INTERVAL_MS  = 500;             // AprilTag detection cycle
const uint32_t    CPU_REPORT_INTERVAL_MS = 10000;           // CPU task stats printed every 10 s
```

### Pixel Format

The camera is initialised with `PIXFORMAT_GRAYSCALE`. The camera driver outputs raw single-byte-per-pixel grayscale data directly in `fb->buf`. No JPEG decode step (`fmt2rgb888`) is needed — the buffer is passed directly to AprilTag's `image_u8_t`. JPEG encoding happens in `stream_task` via `fmt2jpg()` (raw buffer variant) only when a stream client is connected — never in `snap_task`.

### Frame Drop Prevention — Architectural Decisions

Frame drops were caused by several compounding issues identified by analysis of the original code:

#### Root causes (original design)

1. **`snap_task` JPEG encode on every frame** — `frame2jpg` took ~30–50ms on Core 0, blocking `fb_return` and idling the sensor even with no stream client connected.
2. **`streamMux` contention** — `snap_task` held `streamMux` while writing the JPEG stash; `stream_task` held it during `c->write()` TCP send (unbounded latency). If TCP stalled 50ms on congested WiFi, `snap_task` hit its timeout and dropped the frame.
3. **`grayMux` contention** — every 500ms `det_task` held `grayMux` for a full 305KB memcpy. If this overlapped with `snap_task`'s gray write, `snap_task` blocked and sensor DMA stalled.
4. **`fb_count=1`** — sensor DMA had only one buffer. Any `snap_task` block (mutex wait or encode time) idled the sensor entirely.

#### Decision: JPEG encode moved to `stream_task`

`snap_task` no longer calls `frame2jpg` or `fmt2jpg` at all. `stream_task` calls `fmt2jpg()` on a local gray copy after releasing `streamGrayMux`. This means:
- JPEG encode only happens when a client is connected
- `snap_task` loop time drops to just DMA grab + memcpy + `fb_return`
- `stream_task` stack increased 4096 → 8192 to accommodate `fmt2jpg` stack usage

`fmt2jpg()` (raw buffer variant) is used instead of `frame2jpg()` (which requires `camera_fb_t*`) since `stream_task` works from a gray buffer copy, not the DMA framebuffer.

#### Decision: separate gray buffers per consumer

`det_task` and `stream_task` have independent gray buffers:
- `grayStreamBuf` — written by `snap_task` every frame for `stream_task`
- `grayDetBuf` — written by `snap_task` every `DETECTION_INTERVAL_MS` for `det_task`

This eliminates all cross-consumer contention. `det_task` holding `detGrayMux` during a 200ms detection cycle has zero effect on `stream_task`, and vice versa.

#### Decision: `snap_task` waits up to 150ms per consumer

Both `streamGrayMux` and `detGrayMux` are taken with `xSemaphoreTake(..., pdMS_TO_TICKS(150))` in `snap_task`. If a consumer is still holding the mutex, `snap_task` blocks for up to 150ms before giving up. This ensures frames are not silently dropped under normal load — consumers finish their memcpy well within 150ms. `fb_return` still happens after both consumers are served (or timed out).

Previously, zero-timeout try-locks were used and frames were silently skipped if a consumer was busy. This caused silent frame drops under load.

Consumers (`stream_task`, `det_task`) also use 150ms timeout when copying out their local working buffer, up from the previous 20ms. This accommodates the ~60–100ms memcpy time for a 305KB PSRAM buffer at ~3–5 MB/s PSRAM bandwidth.

#### Decision: `det_task` notified directly by `snap_task`

`det_task` is triggered via `xTaskNotifyGive()` from `snap_task` every `DETECTION_INTERVAL_MS`, **not** from `loop()`. This ensures det_task is always triggered at the exact moment a fresh frame is available in `grayDetBuf`, rather than on a separate timer that could drift.

A `detBusy` flag prevents queuing multiple notifies while `det_task` is mid-cycle (running AprilTag detect + HTTP POST). `snap_task` checks `detBusy` before calling `xTaskNotifyGive`; `det_task` clears `detBusy` at the end of `runDetectionAndPush()`. `loop()` no longer notifies `det_task`.

#### Decision: detection has priority over stream in `snap_task`

On each frame, `snap_task` checks the det interval **first** — if elapsed, it copies into `grayDetBuf` and notifies `det_task` before touching `grayStreamBuf`. This ensures `det_task` always gets the freshest possible frame with minimal latency.

#### Decision: `fb_count=2`

Re-enabled after being forced to 1 due to a suspected OV2640 partial-frame bug with grayscale + `fb_count=2`. The partial-frame bug is already tolerated by the `fb->len >= w*(h-4)` frame acceptance check, so `fb_count=2` is safe. This is DMA-level double-buffering — the sensor fills buf[1] while the CPU processes buf[0] — completely independent of the application-level gray buffers above.

#### Decision: gray buffers in PSRAM

`grayStreamBuf` and `grayDetBuf` are allocated with `ps_malloc()` (PSRAM). At 305KB each, they will not fit in internal SRAM (ESP32 has ~320KB usable internal RAM after FreeRTOS and WiFi stack). The ESP32-CAM AI-Thinker has 4MB PSRAM; the camera driver already uses it automatically for the DMA framebuffer.

### Snapshot-Based MJPEG Stream

`snap_task` (Core 0, priority 2) runs **free-running** — no notify from `loop()`. It captures frames as fast as the sensor allows (~5 fps at VGA grayscale, self-paced by `esp_camera_fb_get` blocking until the sensor delivers a frame).

On each frame, `snap_task` follows this sequence:

1. `esp_camera_fb_get()` — blocks until sensor delivers a grayscale frame. A stall watchdog logs if this takes >50ms: `[SNP] fb_get stalled Xms`. OV2640 occasionally drops a VSYNC causing stalls of 500–1200ms; these are surfaced by the watchdog but are a sensor-level quirk.
2. Validate frame (size check, OV2640 477-line quirk tolerance: `fb->len >= fb->width * (fb->height - 4)`)
3. **Detection first (highest priority) — every `DETECTION_INTERVAL_MS`:** wait up to 150ms for `detGrayMux`, copy raw gray into `grayDetBuf`, release `detGrayMux`, notify `det_task` (if `!detBusy`).
4. **Stream — every frame:** wait up to 150ms for `streamGrayMux`, copy raw gray into `grayStreamBuf`, release `streamGrayMux`, notify `stream_task` via `xTaskNotifyGive(streamTaskHandle)`.
5. `esp_camera_fb_return(fb)` — always called after both consumers are served or timed out.

`snap_task` is the **sole** caller of `esp_camera_fb_get` / `esp_camera_fb_return`. Neither `det_task` nor `stream_task` ever calls these.

A **stream serve task** (`stream_task`, Core **0**, priority 1) is spawned per client connection on `/stream`. It blocks on `ulTaskNotifyTake` (200ms watchdog) waiting for `snap_task` to signal a new frame, then:
1. Copies `grayStreamBuf` into a local buffer under `streamGrayMux` (150ms timeout)
2. Releases `streamGrayMux` immediately
3. Calls `fmt2jpg(localGray, ...)` outside the lock — encode takes 30–50ms but holds no mutex
4. Writes the resulting JPEG as an MJPEG multipart frame via `c->write()`

> **Note:** `stream_task` runs on Core **0** (same as `snap_task`). This is intentional — `snap_task` is priority 2 and `stream_task` is priority 1, so `snap_task` preempts `stream_task` freely whenever a new frame is ready. Keeping both on Core 0 leaves Core 1 entirely free for `det_task` (heavy AprilTag compute).

The MJPEG multipart format (`multipart/x-mixed-replace; boundary=frame`) is unchanged — no changes required in the controller or browser.

### fb_count

```cpp
config.fb_count = 2;  // DMA double-buffer: sensor fills buf[1] while CPU processes buf[0]
                      // Partial frames already tolerated via fb->len >= w*(h-4) check
```

Previously `fb_count=1` due to a suspected OV2640 partial-frame bug with grayscale. Re-enabled because the partial-frame acceptance check already handles this case.

### Shared Buffer Architecture

```
Sensor DMA (fb_count=2)
    └── snap_task (Core 0, p2) — sole fb_get/return owner
            │  fb_get stall watchdog: logs if >50ms
            │
            ├── [DET FIRST] wait detGrayMux (150ms) every DETECTION_INTERVAL_MS
            │       └── grayDetBuf [PSRAM, 305KB]
            │               └── notify det_task (if !detBusy → set detBusy=true)
            │                       └── det_task: copy under lock (150ms) → AprilTag → POST
            │                               └── detBusy = false on completion
            │
            └── [STREAM] wait streamGrayMux (150ms) every frame
                    └── grayStreamBuf [PSRAM, 305KB]
                            └── notify stream_task
                                    └── stream_task: copy under lock (150ms) → fmt2jpg → TCP write
```

### Task Priorities

| Task | Core | Priority | Role |
|------|------|----------|------|
| `snap_task` | 0 | 2 (highest on Core 0) | Free-running grayscale capture + gray memcpy; sole fb_get/return owner; waits up to 150ms per consumer |
| `tiT` | 0 | 18 | lwIP / WiFi TCP stack worker (preempts briefly, yields quickly) |
| `stream_task` | 0 | 1 | JPEG encode (fmt2jpg) + MJPEG frame serve per connected client — preempted by snap_task |
| `det_task` | 1 | 2 (highest on Core 1) | AprilTag detection + POST to controller; notified by snap_task |
| `loopTask` | 1 | 1 | Arduino `loop()` / `handleClient()` |

> `tiT` has nominal priority 18 but yields to the lwIP stack quickly; effective CPU usage is ~0.5–2% of Core 0.

### Diagnostics — Watchdogs

Two watchdogs are active for pipeline visibility:

**fb_get stall watchdog** — in `snap_task`, logs if `esp_camera_fb_get()` takes >50ms:
```
[SNP] fb_get stalled 283ms
```
Typical frame period at VGA grayscale is ~200–290ms (~4–5 fps). Spikes of 500–1200ms indicate OV2640 VSYNC drops — a known sensor quirk, not a firmware bug.

**POST duration watchdog** — in `runDetectionAndPush()`, logs if HTTP POST to `/overlay` takes >100ms:
```
[DET] POST took 247ms
```
Typical POST over the local WiFi AP is 150–350ms. No hung POSTs observed; the HTTPClient default timeout (5 s) acts as the hard ceiling.

### CPU Run-Time Stats

`printTopCpuTasks()` runs every `CPU_REPORT_INTERVAL_MS` (10 s) from `loop()`. Uses `uxTaskGetSystemState()` — no `vTaskGetRunTimeStats()` string parsing.

**Algorithm (rolling window):**
- Pass 1: iterate ALL tasks, accumulate per-core total ticks. `tskNO_AFFINITY` tasks add half their ticks to each core total.
- Compute delta against previous snapshot (`lastSnapshotCoreTicks`, `lastSnapshotTaskTicks`).
- Pass 2: for each watched task, `pct = deltaTaskTicks * 1000 / deltaCoreTicks` — true % of that core's capacity over the last 10 s window.
- Save current snapshot for next report.
- First report after boot prints `(warming up — no delta yet)` instead of percentages.

Hardcoded watched tasks:

| Task name | Core | Priority | Why watched |
|-----------|------|----------|-------------|
| `det_task` | 1 | 2 | AprilTag detect + HTTP POST — heaviest compute |
| `snap_task` | 0 | 2 | Free-running gray memcpy; no encode |
| `stream_task` | 0 | 1 | fmt2jpg encode + MJPEG write per connected client |
| `loopTask` | 1 | 1 | Arduino `loop()` / `handleClient()` |
| `tiT` | 0 | 18 | lwIP / WiFi TCP stack worker |

Serial output format (one line per task, printed every 10 s):

```
[CPU] ══ Per-Core CPU Report ══
[CPU]  Core 0 total ticks: NNNN
[CPU]  Core 1 total ticks: NNNN
[CPU]  det_task      core=1     X.X%/core   ticks=NNNN
[CPU]  snap_task     core=0     X.X%/core   ticks=NNNN
[CPU]  stream_task   core=0     X.X%/core   ticks=NNNN
[CPU]  loopTask      core=1     X.X%/core   ticks=NNNN
[CPU]  tiT           core=0     X.X%/core   ticks=NNNN
[CPU] ════════════════════════════
```

If a task is not running (e.g. no stream client connected), the line prints `not found`.

> **Note:** `stream_task` is spawned per client — if no stream client is connected, it prints `not found`. Percentages are per-core: each task's ticks are measured against its own pinned core's total delta ticks. `tskNO_AFFINITY` tasks in Pass 2 use combined delta as denominator — none of the 5 watched tasks use `tskNO_AFFINITY`.

### AprilTag Detection

Detection runs on-device every `DETECTION_INTERVAL_MS`. Results are POSTed as JSON to the controller's `/overlay` endpoint. `det_task` is triggered via `xTaskNotifyGive()` from `snap_task` (not `loop()`) immediately after a fresh frame is written to `grayDetBuf`.

**Detector configuration** (set once in `initAprilTag()`):

| Parameter | Value | Notes |
|-----------|-------|-------|
| `quad_decimate` | 4.0 | Downsample 4× before quad detection — processes 160×120 internally, significantly faster than 2.0 |
| `quad_sigma` | 0.0 | No blur |
| `nthreads` | 1 | ESP32 single-core detection task |
| `refine_edges` | 1 | Sub-pixel edge refinement |
| `debug` | 0 | No debug image output |

**Processing steps:**
1. Copy `grayDetBuf` under `detGrayMux` mutex (150ms timeout) into local buffer — `det_task` never calls `esp_camera_fb_get`
2. Wrap in `image_u8_t { .width=W, .height=H, .stride=W, .buf=localBuf }` — no decode needed
3. `apriltag_detector_detect()` — AprilTag runs its own adaptive binarization internally
4. For each result: extract bounding box from four corner points, populate `Detection` with label `"marker_<id>"`, normalised coords, `decision_margin / 100.0` as confidence
5. `apriltag_detections_destroy()`, `free(localBuf)`
6. POST `detections` JSON to `/overlay`; POST duration watchdog logs if >100ms
7. `detBusy = false` — allows `snap_task` to queue the next detection notify

Detection result JSON schema:

```json
{
  "detections": [
    { "label": "marker_0", "x": 0.42, "y": 0.18, "w": 0.22, "h": 0.29, "confidence": 0.81 }
  ],
  "ts": 12345
}
```

**Detection labels:** `marker_<id>` where `<id>` is the raw integer from the `tag16h5` dictionary (IDs 0–29).

**Confidence / skip log behaviour:**
- Detections with `confidence >= MARKER_CONFIDENCE_CUTOFF` (50%) are accepted and pushed.
- Detections with `5% < confidence < 50%` log a `[AT] SKIPPING` line — tag is visible but weak.
- Detections with `confidence <= 5%` are silently discarded — treated as noise, not logged, to avoid serial spam.

**Performance notes:**
- `quad_decimate = 4.0` reduces the quad search to 160×120 — the primary speed lever. Revert to 2.0 if detection range needs to increase.
- `refine_edges = 1` recovers sub-pixel accuracy after decimation; recommended at VGA.
- With `PIXFORMAT_GRAYSCALE`, the ~80–200 ms JPEG decode step (`fmt2rgb888`) is eliminated entirely.

### Pump Control

```cpp
const int      PIN_PUMP             = 13;  // Real pump relay GPIO
const int      PIN_LED              = 4;   // On-board LED (driven alongside pump)
const uint32_t PUMP_HOLD_TIMEOUT_MS = 800; // Watchdog: auto-release if no hold heartbeat
const uint32_t PUMP_MIN_ON_MS       = 500; // Minimum pump-on duration — pump fires for at
                                           // least this long even if release arrives sooner
```

**Minimum-on enforcement:** `pumpOn()` records `pumpOnMs = millis()` and clears `pumpReleaseRequested`. `pumpOff()` checks `millis() - pumpOnMs < PUMP_MIN_ON_MS`; if too early it sets `pumpReleaseRequested = true` and returns without cutting power. `loop()` polls `pumpReleaseRequested` and calls `pumpOff()` again once the minimum has elapsed. This guarantees the pump (and LED) fires for at least 500 ms regardless of how quickly release is received.

### Safe / Armed Mode

- **Boots in SAFE mode** by default.
- In **SAFE mode**: pump is blocked; entering SAFE mode immediately turns the pump off.
- In **ARMED mode**: pump operation is permitted.
- The controller web UI reads the mode from the cam's `/status` endpoint and displays a live **SAFE / ARMED** badge.

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/stream` | Snapshot-based MJPEG multipart stream (~5 fps) |
| `GET` | `/status` | JSON: `{ ip, uptime, heap, psram, mode, pumpActive }` |
| `GET` | `/pump?action=press\|hold\|release` | Controls pump relay |
| `GET` | `/mode?set=safe\|armed` | Switches operating mode |

### Main Loop

```
setup():
  initCamera()          <- PIXFORMAT_GRAYSCALE, fb_count=2
  initAprilTag()        <- tag16h5 family, quad_decimate=4.0, refine_edges=1
  connectWiFi()         <- 15 s timeout then ESP.restart()
  registerWithController()
  start det_task        <- Core 1, priority 2; notified by snap_task every DETECTION_INTERVAL_MS
  start snap_task       <- Core 0, priority 2; free-running, waits up to 150ms per consumer
  register HTTP routes
  server.begin()

loop() [Core 1, priority 1]:
  server.handleClient()
  if pumpActive and (millis() - pumpLastSeen) > PUMP_HOLD_TIMEOUT_MS: pumpOff()
  if pumpReleaseRequested and pumpActive and (millis() - pumpOnMs) >= PUMP_MIN_ON_MS: pumpOff()
  if millis() - lastCpuReportMs >= CPU_REPORT_INTERVAL_MS: printTopCpuTasks()
  // det_task is notified directly by snap_task — loop() does NOT notify det_task
```

---

## Future: Automatic Action

The detection overlay pipeline is designed to support autonomous operation:

- Detections from on-device image processing are already available on the controller via `/overlay`
- The controller can read these detections in `loop()` and autonomously issue `setPinActive` / `setPinIdle` calls without browser involvement
- The **SAFE / ARMED** mode gate on the cam ensures the pump cannot operate until explicitly armed
- The controller is the single decision-maker: the cam detects and reports, the controller acts.

### Auto-Drive Status

The controller exposes `/auto_status` — polled by the browser every 500 ms. When `autoEnabled` is false the status reads `"waiting_for_detection"`. When a detection triggers autonomous actuation, status switches to `"performing_action"` for the duration of the blocking action sequence, then returns to `"waiting_for_detection"`.

A practical first autonomous mode for this project is **marker-guided seek-and-shoot**:

1. Camera detects an AprilTag marker and reports `marker_N` with normalised bounding box
2. Controller reads detection centre X position from `/overlay`
3. Controller steers left or right until marker is centred in frame
4. When marker is centred and confidence is high enough, controller can stop drive motion and activate the pump
5. All actuation remains gated by SAFE / ARMED mode and controlled only by the controller

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
