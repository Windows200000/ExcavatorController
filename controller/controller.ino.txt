/*
 * ============================================================
 *  EXCAVATOR CONTROLLER  —  NodeMCU ESP32 sketch
 *  Role   : WiFi Access Point
 *           Hosts web console  → GET  /
 *           Cam registration   → POST /register   (cam tells us its IP)
 *           Overlay store      → POST /overlay    (cam pushes detections)
 *           Overlay serve      → GET  /overlay    (browser polls)
 *           Cam IP serve       → GET  /camip      (browser learns cam IP)
 *           Button commands    → GET  /cmd?btn=X  (browser sends presses)
 * ============================================================
 *
 *  CHANGES vs. v1:
 *   - PIN_LIGHT / PIN_TEST moved off GPIO 34/35 (input-only on ESP32!)
 *   - Added keyboard shortcut support in web UI (WASD / arrows / Q E / L T)
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
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
//  Output pins — fill in your actual GPIO numbers
// ─────────────────────────────────────────────
//  !! IMPORTANT: GPIO 34, 35, 36, 39 are INPUT-ONLY on the ESP32.
//  !! They have no output driver and will silently fail.
//  !! Safe output-capable GPIOs on the NodeMCU-ESP32:
//  !!   2, 4, 5, 13, 14, 15, 16, 17, 18, 19,
//  !!   21, 22, 23, 25, 26, 27, 32, 33
const int PIN_LEFT_FWD    = 13;   // Left track  forward
const int PIN_LEFT_BACK   = 12;   // Left track  back
const int PIN_RIGHT_FWD   = 14;   // Right track forward
const int PIN_RIGHT_BACK  = 27;   // Right track back
const int PIN_TURN_LEFT   = 26;   // Turn left
const int PIN_TURN_RIGHT  = 25;   // Turn right
const int PIN_UP          = 33;   // Arm / bucket up
const int PIN_DOWN        = 32;   // Arm / bucket down
const int PIN_LIGHT       = 15;   // Light (momentary pulse)  ← was 35 (INPUT-ONLY!)
const int PIN_TEST        = 4;    // Test button 1            ← was 34 (INPUT-ONLY!)

// List of ALL output pins (used for safe initialisation)
const int ALL_PINS[] = {
  PIN_LEFT_FWD, PIN_LEFT_BACK,
  PIN_RIGHT_FWD, PIN_RIGHT_BACK,
  PIN_TURN_LEFT, PIN_TURN_RIGHT,
  PIN_UP, PIN_DOWN,
  PIN_LIGHT, PIN_TEST
};
const int PIN_COUNT = sizeof(ALL_PINS) / sizeof(ALL_PINS[0]);

// ─────────────────────────────────────────────
//  Button timing
// ─────────────────────────────────────────────
// Duration of the light / test momentary pulse (ms)
const uint32_t PULSE_DURATION_MS = 150;

// How long after the last "hold" heartbeat before we
// auto-release a held button (browser sends heartbeats
// every HOLD_HEARTBEAT_INTERVAL_MS while key is held)
const uint32_t HOLD_TIMEOUT_MS   = 300;

// ─────────────────────────────────────────────
//  Cam registration
// ─────────────────────────────────────────────
String camIP = "";   // filled when cam POSTs /register

// ─────────────────────────────────────────────
//  Overlay storage  (last payload from cam)
// ─────────────────────────────────────────────
String overlayJson = "{\"detections\":[]}";

// ─────────────────────────────────────────────
//  Held-button state
// ─────────────────────────────────────────────
struct HeldPin {
  int      pin       = -1;
  uint32_t lastSeen  = 0;   // millis() of last heartbeat
  bool     active    = false;
};

HeldPin heldPin;   // only one button held at a time (expand if needed)

WebServer server(80);

// ════════════════════════════════════════════════════════════
//  Pin helpers
// ════════════════════════════════════════════════════════════

void pressPin(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void releasePin(int pin) {
  pinMode(pin, INPUT);   // back to high-Z
}

void releaseAll() {
  for (int i = 0; i < PIN_COUNT; i++) releasePin(ALL_PINS[i]);
  if (heldPin.active) { heldPin.active = false; heldPin.pin = -1; }
}

void pulsePin(int pin, uint32_t durationMs) {
  pressPin(pin);
  delay(durationMs);
  releasePin(pin);
}

int buttonToPin(const String& btn) {
  if (btn == "left_fwd")    return PIN_LEFT_FWD;
  if (btn == "left_back")   return PIN_LEFT_BACK;
  if (btn == "right_fwd")   return PIN_RIGHT_FWD;
  if (btn == "right_back")  return PIN_RIGHT_BACK;
  if (btn == "turn_left")   return PIN_TURN_LEFT;
  if (btn == "turn_right")  return PIN_TURN_RIGHT;
  if (btn == "up")          return PIN_UP;
  if (btn == "down")        return PIN_DOWN;
  if (btn == "light")       return PIN_LIGHT;
  if (btn == "test")        return PIN_TEST;
  return -1;
}

bool isPulseButton(const String& btn) {
  return (btn == "light" || btn == "test");
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

void handleCmd() {
  if (!server.hasArg("btn")) { server.send(400, "text/plain", "Missing btn"); return; }
  String btn    = server.arg("btn");
  String action = server.hasArg("action") ? server.arg("action") : "press";
  int    pin    = buttonToPin(btn);

  if (pin < 0) { server.send(400, "text/plain", "Unknown button"); return; }

  if (isPulseButton(btn)) {
    pulsePin(pin, PULSE_DURATION_MS);
    Serial.printf("[CMD] Pulse  %s  pin=%d\n", btn.c_str(), pin);
  } else if (action == "press" || action == "hold") {
    if (!heldPin.active || heldPin.pin != pin) {
      if (heldPin.active) releasePin(heldPin.pin);
      pressPin(pin);
      heldPin.pin    = pin;
      heldPin.active = true;
      Serial.printf("[CMD] Hold   %s  pin=%d\n", btn.c_str(), pin);
    }
    heldPin.lastSeen = millis();
  } else if (action == "release") {
    if (heldPin.active && heldPin.pin == pin) {
      releasePin(pin);
      heldPin.active = false;
      heldPin.pin    = -1;
      Serial.printf("[CMD] Release %s  pin=%d\n", btn.c_str(), pin);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["camIP"]   = camIP;
  doc["uptime"]  = millis() / 1000;
  doc["heap"]    = ESP.getFreeHeap();
  doc["heldPin"] = heldPin.active ? heldPin.pin : -1;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
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
    --bg:      #0a0e14; --panel:  #111720; --border: #1e2d3d;
    --accent:  #f0a500; --accent2:#e05c00; --danger: #e03030;
    --green:   #00c97a; --text:   #c8d8e8; --dim:    #4a5a6a;
    --btn-h:   64px;    --radius: 6px;
  }
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Exo 2', sans-serif;
    min-height: 100vh; display: flex; flex-direction: column;
    align-items: center; padding: 16px; gap: 14px;
  }
  header {
    width: 100%; max-width: 900px;
    display: flex; align-items: center; justify-content: space-between;
    border-bottom: 1px solid var(--border); padding-bottom: 10px;
    flex-wrap: wrap; gap: 8px;
  }
  header h1 { font-size:1.1rem; font-weight:800; letter-spacing:.15em; text-transform:uppercase; color:var(--accent); }
  #header-right { display:flex; align-items:center; gap:14px; }
  #cam-status { font-family:'Share Tech Mono',monospace; font-size:.72rem; color:var(--dim); }
  #cam-status.online { color: var(--green); }
  #kbd-hint {
    font-family:'Share Tech Mono',monospace; font-size:.65rem; color:var(--dim);
    background:#0d1520; border:1px solid var(--border); border-radius:20px;
    padding:3px 10px; white-space:nowrap;
  }
  #feed-wrap {
    position:relative; width:100%; max-width:900px;
    background:#000; border:1px solid var(--border); border-radius:var(--radius);
    overflow:hidden; aspect-ratio:4/3;
  }
  #feed-img { width:100%; height:100%; object-fit:contain; display:block; }
  #overlay-canvas { position:absolute; inset:0; pointer-events:none; }
  #feed-placeholder {
    position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
    font-family:'Share Tech Mono',monospace; font-size:.85rem; color:var(--dim); background:#000;
  }
  #controls {
    width:100%; max-width:900px;
    display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px;
  }
  .control-group {
    background:var(--panel); border:1px solid var(--border); border-radius:var(--radius);
    padding:10px; display:flex; flex-direction:column; gap:8px;
  }
  .group-label {
    font-size:.65rem; letter-spacing:.12em; text-transform:uppercase;
    color:var(--dim); font-weight:600; border-bottom:1px solid var(--border); padding-bottom:6px;
  }
  .btn-row { display:flex; gap:6px; }
  button.ctrl {
    flex:1; height:var(--btn-h); background:#161e2a; border:1px solid var(--border);
    border-radius:var(--radius); color:var(--text); font-family:'Exo 2',sans-serif;
    font-weight:600; font-size:.78rem; letter-spacing:.05em; cursor:pointer;
    user-select:none; -webkit-user-select:none; touch-action:none;
    transition:background .08s, border-color .08s, color .08s;
    display:flex; flex-direction:column; align-items:center; justify-content:center; gap:4px;
  }
  button.ctrl .icon  { font-size:1.3rem; line-height:1; }
  button.ctrl .label { font-size:.65rem; opacity:.7; }
  button.ctrl .key   {
    font-family:'Share Tech Mono',monospace; font-size:.58rem; color:var(--dim);
    background:#0a0e14; border:1px solid var(--border); border-radius:3px;
    padding:1px 5px; margin-top:2px; line-height:1.4;
  }
  button.ctrl:active, button.ctrl.held { background:var(--accent2); border-color:var(--accent); color:#fff; }
  button.ctrl.held .key { color:var(--accent); border-color:var(--accent); }
  button.ctrl.pulse-btn:active, button.ctrl.pulse-btn.held { background:#1a2a1a; border-color:var(--green); color:var(--green); }
  button.ctrl.danger:active, button.ctrl.danger.held { background:#2a1010; border-color:var(--danger); color:var(--danger); }
  @media (max-width:600px) { #controls { grid-template-columns:1fr; } }
</style>
</head>
<body>

<header>
  <h1>&#9881; Excavator Control</h1>
  <div id="header-right">
    <span id="kbd-hint">&#9000; WASD / &#8593;&#8595;&#8592;&#8594; / Q E / L T</span>
    <span id="cam-status">CAM: searching&hellip;</span>
  </div>
</header>

<div id="feed-wrap">
  <div id="feed-placeholder">Waiting for camera&hellip;</div>
  <img id="feed-img" src="" alt="camera feed" style="display:none">
  <canvas id="overlay-canvas"></canvas>
</div>

<div id="controls">
  <div class="control-group">
    <div class="group-label">Left Track</div>
    <div class="btn-row">
      <button class="ctrl" data-btn="left_fwd"><span class="icon">&#9650;</span><span class="label">FWD</span><span class="key">W</span></button>
      <button class="ctrl" data-btn="left_back"><span class="icon">&#9660;</span><span class="label">BACK</span><span class="key">S</span></button>
    </div>
  </div>
  <div class="control-group">
    <div class="group-label">Right Track</div>
    <div class="btn-row">
      <button class="ctrl" data-btn="right_fwd"><span class="icon">&#9650;</span><span class="label">FWD</span><span class="key">&#8593;</span></button>
      <button class="ctrl" data-btn="right_back"><span class="icon">&#9660;</span><span class="label">BACK</span><span class="key">&#8595;</span></button>
    </div>
  </div>
  <div class="control-group">
    <div class="group-label">Turn</div>
    <div class="btn-row">
      <button class="ctrl" data-btn="turn_left"><span class="icon">&#8634;</span><span class="label">LEFT</span><span class="key">&#8592;</span></button>
      <button class="ctrl" data-btn="turn_right"><span class="icon">&#8635;</span><span class="label">RIGHT</span><span class="key">&#8594;</span></button>
    </div>
  </div>
  <div class="control-group">
    <div class="group-label">Arm / Bucket</div>
    <div class="btn-row">
      <button class="ctrl" data-btn="up"><span class="icon">&#11014;</span><span class="label">UP</span><span class="key">Q</span></button>
      <button class="ctrl" data-btn="down"><span class="icon">&#11015;</span><span class="label">DOWN</span><span class="key">E</span></button>
    </div>
  </div>
  <div class="control-group">
    <div class="group-label">Lights &amp; Aux</div>
    <div class="btn-row">
      <button class="ctrl pulse-btn" data-btn="light" data-pulse="1"><span class="icon">&#128161;</span><span class="label">LIGHT</span><span class="key">L</span></button>
      <button class="ctrl pulse-btn danger" data-btn="test" data-pulse="1"><span class="icon">&#9312;</span><span class="label">TEST</span><span class="key">T</span></button>
    </div>
  </div>
</div>

<script>
(function(){
  'use strict';
  const HEARTBEAT_MS=150, OVERLAY_POLL_MS=500, CAM_POLL_MS=2000;
  let camIP=null, heldBtn=null, heartbeatTimer=null;
  const feedImg=document.getElementById('feed-img');
  const placeholder=document.getElementById('feed-placeholder');
  const canvas=document.getElementById('overlay-canvas');
  const ctx=canvas.getContext('2d');
  const camStatus=document.getElementById('cam-status');

  function cmd(btn,action){ fetch('/cmd?btn='+btn+'&action='+action).catch(()=>{}); }

  function startHold(btn){
    if(heldBtn===btn)return;
    stopHold();
    heldBtn=btn;
    cmd(btn,'press');
    heartbeatTimer=setInterval(()=>cmd(btn,'hold'),HEARTBEAT_MS);
    document.querySelector('[data-btn="'+btn+'"]')?.classList.add('held');
  }
  function stopHold(){
    if(!heldBtn)return;
    clearInterval(heartbeatTimer);
    cmd(heldBtn,'release');
    document.querySelector('[data-btn="'+heldBtn+'"]')?.classList.remove('held');
    heldBtn=null;
  }

  document.querySelectorAll('button.ctrl').forEach(btn=>{
    const name=btn.dataset.btn, isPulse=!!btn.dataset.pulse;
    function onDown(e){
      e.preventDefault();
      if(isPulse){ cmd(name,'press'); btn.classList.add('held'); setTimeout(()=>btn.classList.remove('held'),200); }
      else startHold(name);
    }
    function onUp(e){ e.preventDefault(); if(!isPulse)stopHold(); }
    btn.addEventListener('pointerdown',onDown);
    btn.addEventListener('pointerup',onUp);
    btn.addEventListener('pointerleave',onUp);
    btn.addEventListener('pointercancel',onUp);
  });
  document.addEventListener('pointerup',()=>stopHold());
  window.addEventListener('blur',()=>stopHold());

  // ── Keyboard shortcuts ──────────────────────────────────────
  const KEY_MAP={
    'w':'left_fwd','s':'left_back',
    'ArrowUp':'right_fwd','ArrowDown':'right_back',
    'ArrowLeft':'turn_left','ArrowRight':'turn_right',
    'q':'up','e':'down',
    'l':'light','t':'test'
  };
  const PULSE_KEYS=new Set(['l','t']);
  const activeKeys=new Set();

  document.addEventListener('keydown',ev=>{
    if(ev.target.tagName==='INPUT'||ev.target.tagName==='TEXTAREA')return;
    const btn=KEY_MAP[ev.key]; if(!btn)return;
    ev.preventDefault();
    if(PULSE_KEYS.has(ev.key)){
      if(ev.repeat)return;
      cmd(btn,'press');
      const el=document.querySelector('[data-btn="'+btn+'"]');
      if(el){el.classList.add('held');setTimeout(()=>el.classList.remove('held'),200);}
      return;
    }
    if(activeKeys.has(ev.key))return;
    activeKeys.add(ev.key);
    startHold(btn);
  });
  document.addEventListener('keyup',ev=>{
    const btn=KEY_MAP[ev.key]; if(!btn||PULSE_KEYS.has(ev.key))return;
    activeKeys.delete(ev.key);
    if(heldBtn===btn)stopHold();
  });

  // ── Camera feed ─────────────────────────────────────────────
  function startFeed(ip){
    feedImg.src='http://'+ip+'/stream';
    feedImg.style.display='block';
    placeholder.style.display='none';
    camStatus.textContent='CAM: '+ip;
    camStatus.classList.add('online');
    feedImg.onerror=()=>{
      feedImg.style.display='none';
      placeholder.style.display='flex';
      placeholder.textContent='Stream error \u2014 retrying\u2026';
      camStatus.classList.remove('online');
      setTimeout(()=>{if(camIP)startFeed(camIP);},3000);
    };
  }
  function pollCamIP(){
    fetch('/camip').then(r=>r.text()).then(ip=>{
      ip=ip.trim();
      if(ip&&ip!==camIP){camIP=ip;startFeed(ip);}
      else if(!ip){camStatus.textContent='CAM: searching\u2026';camStatus.classList.remove('online');}
    }).catch(()=>{});
  }
  setInterval(pollCamIP,CAM_POLL_MS); pollCamIP();

  // ── Canvas overlay ───────────────────────────────────────────
  function resizeCanvas(){canvas.width=canvas.offsetWidth;canvas.height=canvas.offsetHeight;}
  window.addEventListener('resize',resizeCanvas); resizeCanvas();

  function drawOverlay(detections){
    ctx.clearRect(0,0,canvas.width,canvas.height);
    if(!detections||!detections.length)return;
    const W=canvas.width,H=canvas.height;
    detections.forEach(d=>{
      const x=d.x*W,y=d.y*H,w=d.w*W,h=d.h*H;
      ctx.strokeStyle='#f0a500'; ctx.lineWidth=2; ctx.strokeRect(x,y,w,h);
      const label=d.label+' '+Math.round(d.confidence*100)+'%';
      ctx.font='bold 12px "Share Tech Mono",monospace';
      const tw=ctx.measureText(label).width;
      ctx.fillStyle='rgba(240,165,0,0.85)'; ctx.fillRect(x,y-18,tw+8,18);
      ctx.fillStyle='#0a0e14'; ctx.fillText(label,x+4,y-4);
    });
  }
  function pollOverlay(){
    fetch('/overlay').then(r=>r.json()).then(d=>drawOverlay(d.detections)).catch(()=>{});
  }
  setInterval(pollOverlay,OVERLAY_POLL_MS);
})();
</script>
</body>
</html>
)HTMLEOF";

// ════════════════════════════════════════════════════════════
//  Setup & loop
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Controller starting");
  releaseAll();

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
  server.on("/status",   HTTP_GET,  handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("[BOOT] HTTP server started");
  Serial.printf("[BOOT] Console: http://%s/\n", AP_IP.toString().c_str());
}

void loop() {
  server.handleClient();
  if (heldPin.active && (millis() - heldPin.lastSeen) > HOLD_TIMEOUT_MS) {
    Serial.printf("[SAFE] Auto-release pin %d (heartbeat timeout)\n", heldPin.pin);
    releasePin(heldPin.pin);
    heldPin.active = false;
    heldPin.pin    = -1;
  }
}
