/*
 * EXCAVATOR CONTROLLER — NodeMCU ESP32 sketch
 * See design.md for full architecture and endpoint reference.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
//  WiFi AP credentials
// ─────────────────────────────────────────────
const char* AP_SSID       = "ExcavatorAP";
const char* AP_PASSWORD   = "exc@vator123";
const IPAddress AP_IP     (192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET (255, 255, 255, 0);

// ─────────────────────────────────────────────
//  Unified PinDef — every GPIO is described by an idle state and up to
//  three action bindings (one per driven state).  No separate modes exist;
//  the idle field fully determines what setPinIdle() does, and each
//  actionXxx field is either a button name string or nullptr (unbound).
//
//  PinState_t values:
//    HIGH  →  pinMode(OUTPUT); digitalWrite(HIGH)
//    LOW   →  pinMode(OUTPUT); digitalWrite(LOW)
//    HiZ   →  pinMode(INPUT)   ← floating / high-impedance
//
//  Typical wiring patterns:
//    MOSFET gate (track / turntable / arm / light):
//      idle = HIGH  (track off — MOSFET fully on, drain clamped to GND)
//      actionHiZ drives the "forward / on" button via the internal analog path
//      actionLow drives the "back / pulse" button by pulling gate low
//
//    Simple button (active-LOW):
//      idle = HiZ   (pin floating, external pull-up holds line idle)
//      actionLow drives the button line to GND
//
//  !! GPIO 34, 35, 36, 39 are INPUT-ONLY on the ESP32 — do NOT use here !!
//
//  Available output-capable GPIOs on the NodeMCU-ESP32 and their modes:
//    GPIO  2  — output only (on-board LED, boot-sensitive)     HIGH / LOW / HiZ
//    GPIO  4  — output / input                                 HIGH / LOW / HiZ  ← test pin
//    GPIO  5  — output / input                                 HIGH / LOW / HiZ
//    GPIO 12  — output / input (boot-sensitive, use carefully) HIGH / LOW / HiZ
//    GPIO 13  — output / input                                 HIGH / LOW / HiZ  ← left track
//    GPIO 14  — output / input                                 HIGH / LOW / HiZ  ← right track
//    GPIO 15  — output / input (boot-sensitive)                HIGH / LOW / HiZ  ← light
//    GPIO 16  — output / input                                 HIGH / LOW / HiZ
//    GPIO 17  — output / input                                 HIGH / LOW / HiZ
//    GPIO 18  — output / input                                 HIGH / LOW / HiZ
//    GPIO 19  — output / input                                 HIGH / LOW / HiZ
//    GPIO 21  — output / input (I2C SDA by default)            HIGH / LOW / HiZ
//    GPIO 22  — output / input (I2C SCL by default)            HIGH / LOW / HiZ
//    GPIO 23  — output / input                                 HIGH / LOW / HiZ
//    GPIO 25  — output / input / DAC                           HIGH / LOW / HiZ
//    GPIO 26  — output / input / DAC                           HIGH / LOW / HiZ  ← turntable
//    GPIO 27  — output / input                                 HIGH / LOW / HiZ
//    GPIO 32  — output / input                                 HIGH / LOW / HiZ
//    GPIO 33  — output / input                                 HIGH / LOW / HiZ  ← arm
// ─────────────────────────────────────────────
enum PinState_t { _HIGH, _LOW, _HiZ };

struct PinDef {
  int         gpio;
  PinState_t  idle;        // state applied by setPinIdle()
  const char* actionHigh;  // button name that drives this pin HIGH   (nullptr = unbound)
  const char* actionHiZ;   // button name that drives this pin to HiZ (nullptr = unbound)
  const char* actionLow;   // button name that drives this pin LOW    (nullptr = unbound)
};

// ─────────────────────────────────────────────
//  Pin table — one entry per physical GPIO used.
// ─────────────────────────────────────────────
const PinDef PIN_TABLE[] = {
//Weirdo mode:                  Green            Cyan
// gpio   idle   actionHigh     actionHiZ        actionLow
  { 12 , _LOW  , "left_fwd"   ,  nullptr     ,  nullptr      },
  { 26 , _LOW  , "right_fwd"  ,  nullptr     ,  nullptr      },
  { 33 , _LOW  , "arm_fwd"    ,  nullptr     ,  nullptr      },
  { 14 , _HiZ  ,  nullptr     ,  nullptr     , "left_back"   },
  { 25 , _HiZ  ,  nullptr     ,  nullptr     , "right_back"  },
  { 32 , _HiZ  ,  nullptr     ,  nullptr     , "arm_back"    },
  { 16 , _LOW  , "turn_left"  ,  nullptr     ,  nullptr      },
  { 17 , _HiZ  ,  nullptr     ,  nullptr     , "turn_right"  },
  { 18 , _LOW  , "light_on"   ,  nullptr     ,  nullptr      },
  { 19 , _HiZ  ,  nullptr     ,  nullptr     , "light_off"   },
//{ 13 , _HIGH ,  nullptr     , "left_fwd"   , "left_back"   },
//{ 14 , _HIGH ,  nullptr     , "right_fwd"  , "right_back"  },
//{ 26 , _HIGH ,  nullptr     , "turn_left"  , "turn_right"  },
//{ 33 , _HIGH ,  nullptr     , "arm_fwd"    , "arm_back"    },
//{ 15 , _HIGH ,  nullptr     , "light_on"   , "light_off"   },
  {  4 , _HiZ  ,  nullptr     , nullptr      , "test"        },
};
const int PIN_COUNT = sizeof(PIN_TABLE) / sizeof(PIN_TABLE[0]);

// ─────────────────────────────────────────────
//  Timing
// ─────────────────────────────────────────────
const uint32_t PULSE_DURATION_MS = 150;   // light on/off pulse
const uint32_t HOLD_TIMEOUT_MS   = 300;   // auto-release if heartbeat stops

// ─────────────────────────────────────────────
//  Multi-button hold state
// ─────────────────────────────────────────────
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

const int MAX_HELD = 6;
HeldAction heldActions[MAX_HELD];

WebServer server(80);

// ─────────────────────────────────────────────
//  Cam IP (filled on registration)
// ─────────────────────────────────────────────
String camIP = "";
String overlayJson = "{\"detections\":[]}";

// ─────────────────────────────────────────────
//  Light toggle state
// ─────────────────────────────────────────────
bool lightOn = false;

// ════════════════════════════════════════════════════════════
//  Pin helpers
// ════════════════════════════════════════════════════════════
static void applyPinState(int gpio, PinState_t state) {
  switch (state) {
    case _HIGH: pinMode(gpio, OUTPUT); digitalWrite(gpio, HIGH); break;
    case _LOW:  pinMode(gpio, OUTPUT); digitalWrite(gpio, LOW);  break;
    case _HiZ:  pinMode(gpio, INPUT);                            break;
  }
}

static void setPinIdle(const PinDef& p) {
  applyPinState(p.gpio, p.idle);
}

static void setPinActive(const PinDef& p, PinState_t targetState) {
  applyPinState(p.gpio, targetState);
}

void releaseAllPins() {
  for (int i = 0; i < PIN_COUNT; i++) setPinIdle(PIN_TABLE[i]);
  for (int i = 0; i < MAX_HELD;  i++) heldActions[i].active = false;
}

static void pulsePin(const PinDef& p, PinState_t activeState, uint32_t durationMs) {
  setPinActive(p, activeState);
  delay(durationMs);
  setPinIdle(p);
}

// ─────────────────────────────────────────────
//  Held-action slot management
// ─────────────────────────────────────────────
HeldAction* findHeld(const String& name) {
  for (int i = 0; i < MAX_HELD; i++)
    if (heldActions[i].active && heldActions[i].name == name)
      return &heldActions[i];
  return nullptr;
}

HeldAction* allocHeld(const String& name) {
  HeldAction* h = findHeld(name);
  if (h) return h;
  for (int i = 0; i < MAX_HELD; i++)
    if (!heldActions[i].active) return &heldActions[i];
  return nullptr;
}

// ─────────────────────────────────────────────
//  Map button name → HeldPin list
//  Returns count; 0 = unknown button.
//  Composite actions (fwd/back/spin_*) fan out recursively.
// ─────────────────────────────────────────────
int buttonToHeldPins(const String& btn, HeldPin* out) {
  // Composites
  if (btn == "fwd") {
    HeldPin tmp[4];
    int c1 = buttonToHeldPins("left_fwd",  tmp);
    int c2 = buttonToHeldPins("right_fwd", tmp + c1);
    for (int i = 0; i < c1 + c2; i++) out[i] = tmp[i];
    return c1 + c2;
  }
  if (btn == "back") {
    HeldPin tmp[4];
    int c1 = buttonToHeldPins("left_back",  tmp);
    int c2 = buttonToHeldPins("right_back", tmp + c1);
    for (int i = 0; i < c1 + c2; i++) out[i] = tmp[i];
    return c1 + c2;
  }
  if (btn == "spin_left") {
    HeldPin tmp[4];
    int c1 = buttonToHeldPins("left_back", tmp);
    int c2 = buttonToHeldPins("right_fwd", tmp + c1);
    for (int i = 0; i < c1 + c2; i++) out[i] = tmp[i];
    return c1 + c2;
  }
  if (btn == "spin_right") {
    HeldPin tmp[4];
    int c1 = buttonToHeldPins("left_fwd",  tmp);
    int c2 = buttonToHeldPins("right_back", tmp + c1);
    for (int i = 0; i < c1 + c2; i++) out[i] = tmp[i];
    return c1 + c2;
  }

  // Primitives — walk PIN_TABLE, check all three action slots
  for (int i = 0; i < PIN_COUNT; i++) {
    const PinDef& p = PIN_TABLE[i];
    if (p.actionHigh && btn == p.actionHigh) { out[0] = { &p, _HIGH }; return 1; }
    if (p.actionHiZ  && btn == p.actionHiZ)  { out[0] = { &p, _HiZ  }; return 1; }
    if (p.actionLow  && btn == p.actionLow)  { out[0] = { &p, _LOW  }; return 1; }
  }
  return 0;
}

void startAction(const String& name, HeldPin* pins, int count) {
  HeldAction* h = allocHeld(name);
  if (!h) return;
  if (!h->active) {
    h->name     = name;
    h->pinCount = count;
    for (int i = 0; i < count; i++) {
      h->pins[i] = pins[i];
      setPinActive(*pins[i].pin, pins[i].targetState);
    }
    h->active = true;
    Serial.printf("[CMD] Hold  '%s'  (%d pins)\n", name.c_str(), count);
  }
  h->lastSeen = millis();
}

void stopAction(const String& name) {
  HeldAction* h = findHeld(name);
  if (!h) return;
  for (int i = 0; i < h->pinCount; i++) setPinIdle(*h->pins[i].pin);
  h->active = false;
  Serial.printf("[CMD] Release '%s'\n", name.c_str());
}

// ════════════════════════════════════════════════════════════
//  Cam relay helper
// ════════════════════════════════════════════════════════════
void relayCamCmd(const String& path) {
  if (camIP.length() == 0) return;
  HTTPClient http;
  http.begin("http://" + camIP + path);
  http.setTimeout(200);
  http.GET();
  http.end();
}

// ════════════════════════════════════════════════════════════
//  HTTP handlers
// ════════════════════════════════════════════════════════════
void handleRoot() {
  extern const char INDEX_HTML[] PROGMEM;
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleRegister() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "No body"); return; }
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }
  camIP = doc["ip"].as<String>();
  Serial.printf("[REG] Cam registered at %s\n", camIP.c_str());
  server.send(200, "text/plain", "OK");
}

void handleCamIP() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", camIP);
}

void handleOverlayPost() {
  if (server.hasArg("plain")) overlayJson = server.arg("plain");
  server.send(200, "text/plain", "OK");
}

void handleOverlayGet() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", overlayJson);
}

// /cmd?btn=X&action=press|hold|release
void handleCmd() {
  if (!server.hasArg("btn")) { server.send(400, "text/plain", "Missing btn"); return; }
  String btn    = server.arg("btn");
  String action = server.hasArg("action") ? server.arg("action") : "press";

  if (btn == "light") {
    if (action == "press") {
      lightOn = !lightOn;
      const char* desired = lightOn ? "light_on" : "light_off";
      for (int i = 0; i < PIN_COUNT; i++) {
        const PinDef& p = PIN_TABLE[i];
        if ((p.actionHiZ  && String(p.actionHiZ)  == desired) ||
            (p.actionLow  && String(p.actionLow)   == desired) ||
            (p.actionHigh && String(p.actionHigh)  == desired)) {
          PinState_t s = (p.actionHiZ  && String(p.actionHiZ)  == desired) ? _HiZ
                       : (p.actionLow  && String(p.actionLow)   == desired) ? _LOW
                       : _HIGH;
          pulsePin(p, s, PULSE_DURATION_MS);
          break;
        }
      }
      Serial.printf("[CMD] Light %s\n", lightOn ? "ON" : "OFF");
    }
    server.send(200, "text/plain", lightOn ? "on" : "off");
    return;
  }

  HeldPin pins[4];
  int count = buttonToHeldPins(btn, pins);
  if (count == 0) { server.send(400, "text/plain", "Unknown button"); return; }

  if (action == "press" || action == "hold") {
    startAction(btn, pins, count);
  } else if (action == "release") {
    stopAction(btn);
  }
  server.send(200, "text/plain", "OK");
}

// /camcmd?cmd=pump_press|pump_hold|pump_release|mode_arm|mode_safe
void handleCamCmd() {
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "Missing cmd"); return; }
  String cmd = server.arg("cmd");

  if      (cmd == "pump_press")   relayCamCmd("/pump?action=press");
  else if (cmd == "pump_hold")    relayCamCmd("/pump?action=hold");
  else if (cmd == "pump_release") relayCamCmd("/pump?action=release");
  else if (cmd == "mode_arm")     relayCamCmd("/mode?set=armed");
  else if (cmd == "mode_safe")    relayCamCmd("/mode?set=safe");
  else { server.send(400, "text/plain", "Unknown cmd"); return; }

  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["camIP"]  = camIP;
  doc["uptime"] = millis() / 1000;
  doc["heap"]   = ESP.getFreeHeap();
  doc["light"]  = lightOn;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

extern const char INDEX_HTML[] PROGMEM;

// ════════════════════════════════════════════════════════════
//  Setup & loop
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Controller starting");
  releaseAllPins();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[WIFI] AP started  SSID: %s  IP: %s\n", AP_SSID, AP_IP.toString().c_str());

  server.on("/",         HTTP_GET,  handleRoot);
  server.on("/register", HTTP_POST, handleRegister);
  server.on("/camip",    HTTP_GET,  handleCamIP);
  server.on("/overlay",  HTTP_GET,  handleOverlayGet);
  server.on("/overlay",  HTTP_POST, handleOverlayPost);
  server.on("/cmd",      HTTP_GET,  handleCmd);
  server.on("/camcmd",   HTTP_GET,  handleCamCmd);
  server.on("/status",   HTTP_GET,  handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Console: http://%s/\n", AP_IP.toString().c_str());
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  for (int i = 0; i < MAX_HELD; i++) {
    if (heldActions[i].active && (now - heldActions[i].lastSeen) > HOLD_TIMEOUT_MS) {
      Serial.printf("[SAFE] Timeout-release '%s'\n", heldActions[i].name.c_str());
      for (int j = 0; j < heldActions[i].pinCount; j++)
        setPinIdle(*heldActions[i].pins[j].pin);
      heldActions[i].active = false;
    }
  }
}


// ════════════════════════════════════════════════════════════
//  Web console HTML  (stored in flash)
// ════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Excavator Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap');
  :root {
    --bg:#0a0e14; --panel:#111720; --border:#1e2d3d;
    --accent:#f0a500; --accent2:#e05c00; --danger:#e03030;
    --green:#00c97a; --blue:#3ab8ff; --text:#c8d8e8; --dim:#4a5a6a;
    --btn-h:60px; --radius:6px;
  }
  *,*::before,*::after{box-sizing:border-box;margin:0;padding:0;}
  body{
    background:var(--bg);color:var(--text);
    font-family:'Exo 2',sans-serif;
    min-height:100vh;display:flex;flex-direction:column;
    align-items:center;padding:12px;gap:10px;
  }
  header{
    width:100%;max-width:1100px;
    display:flex;align-items:center;justify-content:space-between;
    border-bottom:1px solid var(--border);padding-bottom:8px;
    flex-wrap:wrap;gap:6px;
  }
  header h1{font-size:1rem;font-weight:800;letter-spacing:.15em;text-transform:uppercase;color:var(--accent);}
  #header-right{display:flex;align-items:center;gap:10px;flex-wrap:wrap;}
  #cam-status{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);}
  #cam-status.online{color:var(--green);}
  #kbd-hint{
    font-family:'Share Tech Mono',monospace;font-size:.62rem;color:var(--dim);
    background:#0d1520;border:1px solid var(--border);border-radius:20px;
    padding:2px 8px;white-space:nowrap;
  }

  /* ── Main layout: feed left, model right ── */
  #main-row{
    width:100%;max-width:1100px;
    display:grid;grid-template-columns:1fr 420px;gap:10px;
  }
  @media(max-width:800px){#main-row{grid-template-columns:1fr;}}

  /* ── Camera feed ── */
  #feed-wrap{
    position:relative;background:#000;border:1px solid var(--border);
    border-radius:var(--radius);overflow:hidden;aspect-ratio:4/3;
  }
  #feed-img{width:100%;height:100%;object-fit:contain;display:block;}
  #overlay-canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;}
  #feed-placeholder{
    position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
    font-family:'Share Tech Mono',monospace;font-size:.8rem;color:var(--dim);background:#000;
  }

  /* ── Excavator model panel ── */
  #model-panel{
    background:var(--panel);border:1px solid var(--border);border-radius:var(--radius);
    padding:10px;display:flex;flex-direction:column;gap:8px;position:relative;
  }
  #model-title{font-size:.62rem;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);font-weight:600;}
  #model-svg-wrap{flex:1;display:flex;align-items:center;justify-content:center;}
  #model-svg-wrap svg{width:100%;max-width:400px;height:auto;}

  /* Safety badge */
  #safety-badge{
    position:absolute;top:10px;right:10px;
    font-family:'Share Tech Mono',monospace;font-size:.65rem;font-weight:700;
    padding:3px 10px;border-radius:20px;border:1px solid;cursor:pointer;
    transition:all .2s;user-select:none;letter-spacing:.08em;
  }
  #safety-badge.safe{background:#1a0a0a;border-color:var(--danger);color:var(--danger);}
  #safety-badge.armed{background:#0a1a0a;border-color:var(--green);color:var(--green);}

  /* ── Controls grid ── */
  #controls{width:100%;max-width:1100px;display:grid;grid-template-columns:repeat(4,1fr);gap:8px;}
  @media(max-width:800px){#controls{grid-template-columns:repeat(2,1fr);}}
  @media(max-width:450px){#controls{grid-template-columns:1fr;}}

  .control-group{
    background:var(--panel);border:1px solid var(--border);border-radius:var(--radius);
    padding:8px;display:flex;flex-direction:column;gap:6px;
  }
  .group-label{
    font-size:.6rem;letter-spacing:.12em;text-transform:uppercase;
    color:var(--dim);font-weight:600;border-bottom:1px solid var(--border);padding-bottom:4px;
  }
  .btn-row{display:flex;gap:5px;}
  button.ctrl{
    flex:1;height:var(--btn-h);background:#161e2a;border:1px solid var(--border);
    border-radius:var(--radius);color:var(--text);font-family:'Exo 2',sans-serif;
    font-weight:600;font-size:.72rem;letter-spacing:.04em;cursor:pointer;
    user-select:none;-webkit-user-select:none;touch-action:none;
    transition:background .08s,border-color .08s,color .08s;
    display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;
  }
  button.ctrl .icon{font-size:1.2rem;line-height:1;}
  button.ctrl .label{font-size:.6rem;opacity:.7;}
  button.ctrl .key{
    font-family:'Share Tech Mono',monospace;font-size:.55rem;color:var(--dim);
    background:#0a0e14;border:1px solid var(--border);border-radius:3px;
    padding:1px 4px;margin-top:1px;line-height:1.4;
  }
  button.ctrl:active,button.ctrl.held{background:var(--accent2);border-color:var(--accent);color:#fff;}
  button.ctrl.held .key{color:var(--accent);border-color:var(--accent);}
  button.ctrl.green-btn:active,button.ctrl.green-btn.held{background:#0a1f0a;border-color:var(--green);color:var(--green);}
  button.ctrl.blue-btn:active,button.ctrl.blue-btn.held{background:#0a1520;border-color:var(--blue);color:var(--blue);}
  button.ctrl.danger-btn:active,button.ctrl.danger-btn.held{background:#2a1010;border-color:var(--danger);color:var(--danger);}
  button.ctrl.toggle-on{background:#0a1a0a;border-color:var(--green);color:var(--green);}

  /* SVG part glow animations */
  .part{transition:filter .15s,opacity .15s;}
  .part.active{filter:drop-shadow(0 0 6px currentColor);opacity:1;}
  .track-fill{fill:#1e2d3d;transition:fill .15s;}
  .track-fill.active{fill:#e05c00;}
  .arrow-indicator{opacity:0;transition:opacity .15s;}
  .arrow-indicator.active{opacity:1;}
</style>
</head>
<body>

<header>
  <h1>&#9881; Excavator Control</h1>
  <div id="header-right">
    <span id="kbd-hint">WASD=drive &nbsp; &#8592;&#8594;=turret &nbsp; &#8593;&#8595;=arm &nbsp; L=light &nbsp; T=test &nbsp; Space=pump</span>
    <span id="cam-status">CAM: searching&hellip;</span>
  </div>
</header>

<div id="main-row">
  <!-- Camera feed -->
  <div id="feed-wrap">
    <div id="feed-placeholder">Waiting for camera&hellip;</div>
    <img id="feed-img" src="" alt="camera feed" style="display:none">
    <canvas id="overlay-canvas"></canvas>
  </div>

  <!-- Excavator model -->
  <div id="model-panel">
    <div id="model-title">Live Status</div>

    <!-- Safety badge — driven live from cam module -->
    <div id="safety-badge" class="safe" title="Click to toggle arm/safe on cam module">&#128274; SAFE</div>

    <div id="model-svg-wrap">
      <!-- Isometric-ish top-down excavator SVG -->
      <svg viewBox="0 0 400 340" xmlns="http://www.w3.org/2000/svg" style="font-family:'Share Tech Mono',monospace;">
        <defs>
          <filter id="glow-orange">
            <feGaussianBlur stdDeviation="3" result="blur"/>
            <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
          </filter>
          <filter id="glow-green">
            <feGaussianBlur stdDeviation="4" result="blur"/>
            <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
          </filter>
          <filter id="glow-blue">
            <feGaussianBlur stdDeviation="3" result="blur"/>
            <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
          </filter>
        </defs>

        <!-- ── Left track ── -->
        <g id="svg-left-track">
          <rect class="track-fill" id="lt-fill" x="20" y="100" width="60" height="140" rx="18"/>
          <rect fill="none" stroke="#2a3f55" stroke-width="2" x="20" y="100" width="60" height="140" rx="18"/>
          <!-- tread lines -->
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="125" x2="80" y2="125"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="145" x2="80" y2="145"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="165" x2="80" y2="165"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="185" x2="80" y2="185"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="205" x2="80" y2="205"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="20" y1="225" x2="80" y2="225"/>
          <!-- fwd arrow -->
          <text id="lt-arrow-fwd" class="arrow-indicator" x="50" y="108" text-anchor="middle" font-size="16" fill="#f0a500">&#9650;</text>
          <!-- back arrow -->
          <text id="lt-arrow-back" class="arrow-indicator" x="50" y="252" text-anchor="middle" font-size="16" fill="#f0a500">&#9660;</text>
          <text x="50" y="175" text-anchor="middle" font-size="9" fill="#4a5a6a" letter-spacing="1">L</text>
        </g>

        <!-- ── Right track ── -->
        <g id="svg-right-track">
          <rect class="track-fill" id="rt-fill" x="320" y="100" width="60" height="140" rx="18"/>
          <rect fill="none" stroke="#2a3f55" stroke-width="2" x="320" y="100" width="60" height="140" rx="18"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="125" x2="380" y2="125"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="145" x2="380" y2="145"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="165" x2="380" y2="165"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="185" x2="380" y2="185"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="205" x2="380" y2="205"/>
          <line stroke="#2a3f55" stroke-width="1.5" x1="320" y1="225" x2="380" y2="225"/>
          <text id="rt-arrow-fwd" class="arrow-indicator" x="350" y="108" text-anchor="middle" font-size="16" fill="#f0a500">&#9650;</text>
          <text id="rt-arrow-back" class="arrow-indicator" x="350" y="252" text-anchor="middle" font-size="16" fill="#f0a500">&#9660;</text>
          <text x="350" y="175" text-anchor="middle" font-size="9" fill="#4a5a6a" letter-spacing="1">R</text>
        </g>

        <!-- ── Chassis body ── -->
        <rect fill="#111720" stroke="#1e2d3d" stroke-width="2" x="80" y="115" width="240" height="110" rx="6"/>

        <!-- ── Turntable ring ── -->
        <circle id="svg-turntable" fill="none" stroke="#2a3f55" stroke-width="3" cx="200" cy="170" r="55"/>
        <circle fill="#0d141e" stroke="#1e2d3d" stroke-width="1" cx="200" cy="170" r="50"/>

        <!-- Turntable rotation arrows -->
        <text id="tt-arrow-left" class="arrow-indicator" x="138" y="176" text-anchor="middle" font-size="18" fill="#3ab8ff">&#8592;</text>
        <text id="tt-arrow-right" class="arrow-indicator" x="262" y="176" text-anchor="middle" font-size="18" fill="#3ab8ff">&#8594;</text>

        <!-- Cabin on turntable -->
        <rect id="svg-cabin" fill="#111720" stroke="#1e2d3d" stroke-width="2" x="175" y="148" width="50" height="44" rx="4"/>
        <text x="200" y="175" text-anchor="middle" font-size="8" fill="#2a3f55" letter-spacing="1">CAB</text>

        <!-- ── Arm assembly (top-down silhouette extending forward/up) ── -->
        <!-- Boom -->
        <line id="svg-boom" x1="200" y1="148" x2="200" y2="60" stroke="#2a3f55" stroke-width="10" stroke-linecap="round"/>
        <!-- Stick -->
        <line id="svg-stick" x1="200" y1="60" x2="200" y2="20" stroke="#2a3f55" stroke-width="7" stroke-linecap="round"/>
        <!-- Bucket -->
        <path id="svg-bucket" d="M188 10 Q200 0 212 10 L210 22 L190 22 Z" fill="#1e2d3d" stroke="#2a3f55" stroke-width="1.5"/>

        <!-- Arm up/down arrows -->
        <text id="arm-arrow-up" class="arrow-indicator" x="215" y="38" font-size="14" fill="#f0a500">&#11014;</text>
        <text id="arm-arrow-down" class="arrow-indicator" x="215" y="90" font-size="14" fill="#f0a500">&#11015;</text>

        <!-- ── Pump / sprayer (on arm tip) ── -->
        <!-- nozzle body -->
        <rect id="svg-pump-body" fill="#1a1a2a" stroke="#2a3f55" stroke-width="1.5" x="193" y="3" width="14" height="8" rx="2"/>
        <!-- spray lines (hidden unless active) -->
        <g id="svg-pump-spray">
          <line class="arrow-indicator" id="pump-spray-1" x1="196" y1="3" x2="192" y2="-4" stroke="#00c97a" stroke-width="1.5" stroke-linecap="round"/>
          <line class="arrow-indicator" id="pump-spray-2" x1="200" y1="3" x2="200" y2="-5" stroke="#00c97a" stroke-width="1.5" stroke-linecap="round"/>
          <line class="arrow-indicator" id="pump-spray-3" x1="204" y1="3" x2="208" y2="-4" stroke="#00c97a" stroke-width="1.5" stroke-linecap="round"/>
        </g>

        <!-- ── Light indicator (front of cabin) ── -->
        <circle id="svg-light" cx="200" cy="143" r="5" fill="#1e2d3d" stroke="#2a3f55" stroke-width="1.5"/>

        <!-- ── Test pin indicator ── -->
        <rect id="svg-test" fill="#1e2d3d" stroke="#2a3f55" stroke-width="1.5" x="218" y="157" width="12" height="12" rx="2"/>
        <text x="224" y="167" text-anchor="middle" font-size="7" fill="#2a3f55">T</text>

        <!-- Labels -->
        <text x="200" y="330" text-anchor="middle" font-size="8" fill="#2a3f55" letter-spacing="1">EXCAVATOR — TOP VIEW</text>
      </svg>
    </div>
  </div>
</div>

<!-- Controls -->
<div id="controls">
  <!-- Drive -->
  <div class="control-group">
    <div class="group-label">Drive</div>
    <div class="btn-row">
      <button class="ctrl" data-action="fwd"><span class="icon">&#9650;</span><span class="label">FORWARD</span><span class="key">W</span></button>
      <button class="ctrl" data-action="back"><span class="icon">&#9660;</span><span class="label">BACK</span><span class="key">S</span></button>
    </div>
    <div class="btn-row">
      <button class="ctrl" data-action="spin_left"><span class="icon">&#8634;</span><span class="label">SPIN L</span><span class="key">A</span></button>
      <button class="ctrl" data-action="spin_right"><span class="icon">&#8635;</span><span class="label">SPIN R</span><span class="key">D</span></button>
    </div>
  </div>

  <!-- Turret -->
  <div class="control-group">
    <div class="group-label">Turret</div>
    <div class="btn-row">
      <button class="ctrl blue-btn" data-action="turn_left"><span class="icon">&#8592;</span><span class="label">ROTATE L</span><span class="key">&#8592;</span></button>
      <button class="ctrl blue-btn" data-action="turn_right"><span class="icon">&#8594;</span><span class="label">ROTATE R</span><span class="key">&#8594;</span></button>
    </div>
  </div>

  <!-- Arm -->
  <div class="control-group">
    <div class="group-label">Arm</div>
    <div class="btn-row">
      <button class="ctrl" data-action="arm_fwd"><span class="icon">&#11014;</span><span class="label">FORWARD</span><span class="key">&#8593;</span></button>
      <button class="ctrl" data-action="arm_back"><span class="icon">&#11015;</span><span class="label">BACKWARD</span><span class="key">&#8595;</span></button>
    </div>
  </div>

  <!-- Aux -->
  <div class="control-group">
    <div class="group-label">Aux</div>
    <div class="btn-row">
      <button class="ctrl green-btn" id="btn-pump" data-camaction="pump"><span class="icon">&#128167;</span><span class="label">PUMP</span><span class="key">Space</span></button>
      <button class="ctrl" id="btn-light" data-action="light" data-toggle="1"><span class="icon">&#128161;</span><span class="label">LIGHT</span><span class="key">L</span></button>
    </div>
    <div class="btn-row">
      <button class="ctrl danger-btn" data-action="test"><span class="icon">&#9312;</span><span class="label">TEST</span><span class="key">T</span></button>
    </div>
  </div>
</div>

<script>
(function(){
'use strict';

// ── Constants ──────────────────────────────────────────────
const HEARTBEAT_MS   = 150;
const OVERLAY_POLL   = 500;
const CAM_POLL       = 2000;
const MODE_POLL      = 300;  // poll cam /status for armed/safe

// ── State ──────────────────────────────────────────────────
let camIP = null;
let lightToggleOn = false;
const activeHolds = {};

// ── Visual model parts map ──────────────────────────────────
// Keys must match the action names sent to /cmd and used in PIN_TABLE.
const ACTION_SVG = {
  fwd:        ['lt-fill','rt-fill','lt-arrow-fwd','rt-arrow-fwd'],
  back:       ['lt-fill','rt-fill','lt-arrow-back','rt-arrow-back'],
  spin_left:  ['lt-fill','rt-fill','lt-arrow-back','rt-arrow-fwd'],
  spin_right: ['lt-fill','rt-fill','lt-arrow-fwd','rt-arrow-back'],
  left_fwd:   ['lt-fill','lt-arrow-fwd'],
  left_back:  ['lt-fill','lt-arrow-back'],
  right_fwd:  ['rt-fill','rt-arrow-fwd'],
  right_back: ['rt-fill','rt-arrow-back'],
  turn_left:  ['svg-turntable','tt-arrow-left'],
  turn_right: ['svg-turntable','tt-arrow-right'],
  arm_fwd:    ['svg-boom','svg-stick','svg-bucket','arm-arrow-up'],
  arm_back:   ['svg-boom','svg-stick','svg-bucket','arm-arrow-down'],
  test:       ['svg-test'],
  light:      ['svg-light'],
  pump:       ['svg-pump-body','pump-spray-1','pump-spray-2','pump-spray-3'],
};

const SVG_COLORS = {
  'svg-turntable': '#3ab8ff',
  'tt-arrow-left': '#3ab8ff',
  'tt-arrow-right': '#3ab8ff',
  'svg-boom': '#f0a500',
  'svg-stick': '#f0a500',
  'svg-bucket': '#f0a500',
  'arm-arrow-up': '#f0a500',
  'arm-arrow-down': '#f0a500',
  'svg-light': '#f0e040',
  'svg-test': '#e03030',
  'svg-pump-body': '#00c97a',
  'pump-spray-1': '#00c97a',
  'pump-spray-2': '#00c97a',
  'pump-spray-3': '#00c97a',
};

function svgActivate(action) {
  const ids = ACTION_SVG[action] || [];
  ids.forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    el.classList.add('active');
    if (SVG_COLORS[id]) el.style.stroke = SVG_COLORS[id];
    if (el.tagName === 'text') el.style.fill = SVG_COLORS[id] || '#f0a500';
    if (el.tagName === 'rect' && id.endsWith('-fill')) el.style.fill = '#e05c00';
  });
}

function svgDeactivate(action) {
  const ids = ACTION_SVG[action] || [];
  ids.forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    const stillActive = Object.keys(activeHolds).some(a =>
      a !== action && (ACTION_SVG[a]||[]).includes(id)
    );
    if (!stillActive) {
      el.classList.remove('active');
      el.style.stroke = '';
      el.style.fill = '';
    }
  });
}

// ── Command helpers ─────────────────────────────────────────
function cmd(btn, action) {
  fetch('/cmd?btn=' + btn + '&action=' + action).catch(() => {});
}
function camCmd(c) {
  fetch('/camcmd?cmd=' + c).catch(() => {});
}

// ── Hold logic ──────────────────────────────────────────────
function startHold(action, isCamAction) {
  if (activeHolds[action]) return;
  if (isCamAction) camCmd(action + '_press');
  else cmd(action, 'press');
  activeHolds[action] = setInterval(() => {
    if (isCamAction) camCmd(action + '_hold');
    else cmd(action, 'hold');
  }, HEARTBEAT_MS);
  svgActivate(action);
  document.querySelector('[data-action="'+action+'"]')?.classList.add('held');
  document.querySelector('[data-camaction="'+action+'"]')?.classList.add('held');
}

function stopHold(action, isCamAction) {
  if (!activeHolds[action]) return;
  clearInterval(activeHolds[action]);
  delete activeHolds[action];
  if (isCamAction) camCmd(action + '_release');
  else cmd(action, 'release');
  svgDeactivate(action);
  document.querySelector('[data-action="'+action+'"]')?.classList.remove('held');
  document.querySelector('[data-camaction="'+action+'"]')?.classList.remove('held');
}

function stopAllHolds() {
  Object.keys(activeHolds).forEach(a => {
    const el = document.querySelector('[data-camaction="'+a+'"]');
    stopHold(a, !!el);
  });
}

// ── Button wiring ───────────────────────────────────────────
document.querySelectorAll('button.ctrl').forEach(btn => {
  const action    = btn.dataset.action;
  const camAction = btn.dataset.camaction;
  const isToggle  = !!btn.dataset.toggle;
  if (!action && !camAction) return;

  function onDown(e) {
    e.preventDefault();
    if (isToggle) {
      lightToggleOn = !lightToggleOn;
      cmd('light', 'press');
      btn.classList.toggle('toggle-on', lightToggleOn);
      svgActivate('light');
      if (!lightToggleOn) svgDeactivate('light');
      return;
    }
    if (camAction) startHold(camAction, true);
    else startHold(action, false);
  }
  function onUp(e) {
    e.preventDefault();
    if (isToggle) return;
    if (camAction) stopHold(camAction, true);
    else stopHold(action, false);
  }

  btn.addEventListener('pointerdown', onDown);
  btn.addEventListener('pointerup', onUp);
  btn.addEventListener('pointerleave', onUp);
  btn.addEventListener('pointercancel', onUp);
});

document.addEventListener('pointerup', stopAllHolds);
window.addEventListener('blur', stopAllHolds);

// ── Safety badge ────────────────────────────────────────────
const safetyBadge = document.getElementById('safety-badge');
let currentMode = 'safe';

const updateSafetyBadge = (mode) => {
  currentMode = mode;
  if (mode === 'armed') {
    safetyBadge.className = 'armed';
    safetyBadge.innerHTML = '(&#x1F4A2; -_&bull;)&#x256C;&#x336;&#x336;&#x33F;&#x2564;&#x2500;&#x2500; ARMED';
  } else {
    safetyBadge.className = 'safe';
    safetyBadge.textContent = '\uD83D\uDD12 SAFE';
  }
};

safetyBadge.addEventListener('click', () => {
  camCmd(currentMode === 'safe' ? 'mode_arm' : 'mode_safe');
});

// ── Keyboard shortcuts ──────────────────────────────────────
// ArrowUp/Down map to arm_fwd/arm_back to match PIN_TABLE action names.
const KEY_MAP = {
  'w':'fwd','s':'back','a':'spin_left','d':'spin_right',
  'ArrowLeft':'turn_left','ArrowRight':'turn_right',
  'ArrowUp':'arm_fwd','ArrowDown':'arm_back','t':'test',
};
const TOGGLE_KEYS = new Set(['l']);
const CAM_KEYS    = new Set([' ']);
const activeKeys  = new Set();

document.addEventListener('keydown', ev => {
  if (ev.target.tagName === 'INPUT' || ev.target.tagName === 'TEXTAREA') return;
  ev.preventDefault();
  if (TOGGLE_KEYS.has(ev.key)) {
    if (ev.repeat) return;
    lightToggleOn = !lightToggleOn;
    cmd('light', 'press');
    document.getElementById('btn-light')?.classList.toggle('toggle-on', lightToggleOn);
    if (lightToggleOn) svgActivate('light'); else svgDeactivate('light');
    return;
  }
  if (CAM_KEYS.has(ev.key)) { if (!ev.repeat) startHold('pump', true); return; }
  const action = KEY_MAP[ev.key];
  if (!action || activeKeys.has(ev.key)) return;
  activeKeys.add(ev.key);
  startHold(action, false);
});

document.addEventListener('keyup', ev => {
  if (CAM_KEYS.has(ev.key)) { stopHold('pump', true); return; }
  const action = KEY_MAP[ev.key];
  if (!action) return;
  activeKeys.delete(ev.key);
  stopHold(action, false);
});

// ── Camera feed ─────────────────────────────────────────────
const feedWrap    = document.getElementById('feed-wrap');
const feedImg     = document.getElementById('feed-img');
const placeholder = document.getElementById('feed-placeholder');
const canvas      = document.getElementById('overlay-canvas');
const ctx         = canvas.getContext('2d');
const camStatus   = document.getElementById('cam-status');

// Always measure the parent container, not the canvas itself.
// canvas.offsetWidth is circular once a width attribute has been set.
function resizeCanvas() {
  canvas.width  = feedWrap.clientWidth;
  canvas.height = feedWrap.clientHeight;
}
new ResizeObserver(resizeCanvas).observe(feedWrap);
resizeCanvas();

function startFeed(ip) {
  feedImg.src = 'http://' + ip + '/stream';
  feedImg.style.display = 'block';
  placeholder.style.display = 'none';
  camStatus.textContent = 'CAM: ' + ip;
  camStatus.classList.add('online');
  feedImg.onerror = () => {
    feedImg.style.display = 'none';
    placeholder.style.display = 'flex';
    placeholder.textContent = 'Stream error \u2014 retrying\u2026';
    camStatus.classList.remove('online');
    setTimeout(() => { if (camIP) startFeed(camIP); }, 3000);
  };
}

function pollCamIP() {
  fetch('/camip').then(r => r.text()).then(ip => {
    ip = ip.trim();
    if (ip && ip !== camIP) { camIP = ip; startFeed(ip); }
    else if (!ip) { camStatus.textContent = 'CAM: searching\u2026'; camStatus.classList.remove('online'); }
  }).catch(() => {});
}
setInterval(pollCamIP, CAM_POLL); pollCamIP();

// ── Live cam status poll ─────────────────────────────────────
function pollCamStatus() {
  if (!camIP) return;
  fetch('http://' + camIP + '/status').then(r => r.json()).then(d => {
    if (d.mode) updateSafetyBadge(d.mode);
  }).catch(() => {});
}
setInterval(pollCamStatus, MODE_POLL);

// ── Canvas overlay — detection bounding boxes ────────────────
// red_dot detections are drawn with a red box; other labels use amber.
// Green dashed border is always drawn as a canvas-alignment indicator.
function drawOverlay(detections) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  // Green dashed border — confirms canvas covers full feed area
  ctx.save();
  ctx.strokeStyle = '#00ff00';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.strokeRect(1, 1, canvas.width - 2, canvas.height - 2);
  ctx.restore();
  if (!detections || !detections.length) return;
  const W = canvas.width, H = canvas.height;
  detections.forEach(d => {
    const x = d.x * W, y = d.y * H, w = d.w * W, h = d.h * H;
    // red_dot: red, aruco_*: cyan, everything else: amber
    const isRedDot = d.label === 'red_dot';
    const isAruco = d.label.startsWith('aruco_');
    const color = isRedDot ? '#ff3030' : (isAruco ? '#00e5ff' : '#f0a500');
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.strokeRect(x, y, w, h);
    const label = d.label + ' ' + Math.round(d.confidence * 100) + '%';
    ctx.font = 'bold 12px "Share Tech Mono",monospace';
    const tw = ctx.measureText(label).width;
    const bgAlpha = isRedDot ? 0.85 : (isAruco ? 0.85 : 0.85);
    const bgColor = isRedDot ? 'rgba(255,48,48,' : (isAruco ? 'rgba(0,229,255,' : 'rgba(240,165,0,');
    ctx.fillStyle = bgColor + bgAlpha + ')';
    ctx.fillRect(x, y - 18, tw + 8, 18);
    ctx.fillStyle = '#fff';
    ctx.fillText(label, x + 4, y - 4);
  });
}

function pollOverlay() {
  fetch('/overlay').then(r => r.json()).then(d => {
    drawOverlay(d.detections);
  }).catch(() => {});
}
setInterval(pollOverlay, OVERLAY_POLL);

})();
</script>
</body>
</html>
)HTMLEOF";
