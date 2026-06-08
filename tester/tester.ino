#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>

const char* AP_SSID     = "ExcavatorAP";
const char* AP_PASSWORD = "exc@vator123";

const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

const int MONITOR_PIN         = 34;
const int DAC_PIN             = 25;

// High-speed single-shot sampling with delta compression
const uint32_t SAMPLE_INTERVAL_US = 20;    // ~50 kSa/s target (single-shot analogRead)
const uint32_t MAX_JSON_SAMPLES   = 500;   // hard cap per response
const size_t   BUFFER_SIZE        = 4000;  // 4 s safety net in terms of stored (compressed) samples

WebServer server(80);

volatile uint16_t sampleRaw[BUFFER_SIZE];
volatile uint32_t sampleTimeUs[BUFFER_SIZE];
volatile uint32_t totalSamples = 0;

// Delta compression parameters: store only when raw changes by >= 16 LSBs (~12.9 mV)
const uint16_t DELTA_THRESH_RAW = 16;
volatile uint16_t lastStoredRaw = 0;
volatile bool     hasLastStored = false;

uint8_t dacValue = 0;

void IRAM_ATTR onSampleTimer(void*) {
  // Take one ADC sample in single-shot mode
  uint16_t raw = (uint16_t)analogRead(MONITOR_PIN);

  // First stored sample is always kept
  if (!hasLastStored) {
    uint32_t seq = totalSamples + 1;
    size_t   idx = (seq - 1) % BUFFER_SIZE;

    sampleRaw[idx]    = raw;
    sampleTimeUs[idx] = (uint32_t)micros();
    lastStoredRaw     = raw;
    totalSamples      = seq;
    hasLastStored     = true;
    return;
  }

  // Delta compression in raw domain: only keep changes >= DELTA_THRESH_RAW
  int diff = (int)raw - (int)lastStoredRaw;
  if (diff >= - (int)DELTA_THRESH_RAW && diff <= (int)DELTA_THRESH_RAW) {
    // Below threshold -> skip storing, no seq increment
    return;
  }

  uint32_t seq = totalSamples + 1;
  size_t   idx = (seq - 1) % BUFFER_SIZE;

  sampleRaw[idx]    = raw;
  sampleTimeUs[idx] = (uint32_t)micros();
  lastStoredRaw     = raw;
  totalSamples      = seq;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 Scope</title>
  <style>
    :root{
      --bg:#08110a; --panel:#0e1a10; --grid:#16351a;
      --trace:#61ff79; --text:#d8ffe0; --muted:#8eb596;
      --accent:#40c85e; --dac:#69c8ff; --gap:#ff4444;
    }
    *{box-sizing:border-box}
    body{ margin:0; font-family:system-ui,Segoe UI,Roboto,Arial,sans-serif;
          background:var(--bg); color:var(--text); }
    .wrap{ max-width:1200px; margin:0 auto; padding:14px; }
    .top{ display:flex; gap:12px; flex-wrap:wrap; align-items:center; margin-bottom:12px; }
    .card{ background:var(--panel); border:1px solid #1a321d; border-radius:12px; padding:10px 12px; }
    .stat{ min-width:140px; }
    .label{ font-size:12px; color:var(--muted); margin-bottom:4px; }
    .value{ font-size:20px; font-weight:700; }
    .controls{ display:flex; flex-wrap:wrap; gap:12px; align-items:center;
               margin-bottom:12px; background:var(--panel); border:1px solid #1a321d;
               border-radius:12px; padding:12px; }
    .control{ min-width:220px; flex:1 1 220px; }
    .control label{ display:flex; justify-content:space-between; gap:12px;
                    font-size:14px; margin-bottom:6px; color:var(--muted); }
    input[type="range"]{ width:100%; }
    canvas{ width:100%; height:60vh; min-height:320px; display:block;
            background:#050805; border:1px solid #1a321d; border-radius:12px; }
    .dac-panel{ background:var(--panel); border:1px solid #1a3a4a;
                border-radius:12px; padding:14px 16px; margin-top:12px; }
    .dac-panel h3{ margin:0 0 10px 0; font-size:14px; color:var(--dac); letter-spacing:1px; }
    .dac-row{ display:flex; gap:16px; flex-wrap:wrap; align-items:center; }
    .dac-row .control{ min-width:260px; }
    .dac-row .control label span{ color:var(--dac); }
    #dacSlider{ accent-color:var(--dac); }
    .dac-presets{ display:flex; gap:8px; flex-wrap:wrap; }
    .preset{ background:#0d2030; border:1px solid #1a5070; color:var(--dac);
             border-radius:8px; padding:6px 14px; font-size:13px; cursor:pointer; }
    .preset:hover{ background:#163550; }
    .dl-btn{ background:#0e1a10; border:1px solid var(--accent); color:var(--accent);
             border-radius:8px; padding:8px 18px; font-size:14px; cursor:pointer;
             font-family:inherit; }
    .dl-btn:hover{ background:#1a3a1f; }
    .clr-btn{ background:#0e1a10; border:1px solid var(--gap); color:var(--gap);
              border-radius:8px; padding:8px 18px; font-size:14px; cursor:pointer;
              font-family:inherit; }
    .clr-btn:hover{ background:#2a1010; }
    .legend{ display:flex; gap:16px; margin-top:8px; font-size:12px; color:var(--muted); }
    .legend span{ display:flex; align-items:center; gap:5px; }
    .dot{ width:12px; height:3px; border-radius:2px; }
    .foot{ margin-top:10px; color:var(--muted); font-size:13px; }
    .ok{color:#6dff86} .warn{color:#ffd76d}
  </style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div class="card stat">
      <div class="label">Live voltage (GPIO34)</div>
      <div class="value" id="liveV">--.- V</div>
    </div>
    <div class="card stat">
      <div class="label">Sample rate</div>
      <div class="value" id="rate">-- Sa/s</div>
    </div>
    <div class="card stat">
      <div class="label">Stored samples</div>
      <div class="value" id="stored">0</div>
    </div>
    <div class="card stat">
      <div class="label">Dropped</div>
      <div class="value" id="dropped" style="color:var(--gap)">0</div>
    </div>
    <div class="card stat">
      <div class="label">DAC out (GPIO25)</div>
      <div class="value" id="dacDisplay" style="color:var(--dac)">0.000 V</div>
    </div>
    <div class="card stat">
      <div class="label">Status</div>
      <div class="value" id="status">Connecting</div>
    </div>
    <div class="card" style="display:flex;align-items:center;gap:8px;">
      <button class="dl-btn" onclick="downloadCSV()">⬇ CSV</button>
      <button class="clr-btn" onclick="clearData()">✕ Clear</button>
    </div>
  </div>

  <div class="controls">
    <div class="control">
      <label><span>View window</span><span id="windowText">10 s</span></label>
      <input id="windowRange" type="range" min="1" max="300" value="10" step="1">
    </div>
    <div class="control">
      <label><span>Vertical full scale</span><span id="scaleText">3.3 V</span></label>
      <input id="scaleRange" type="range" min="0.5" max="3.3" value="3.3" step="0.1">
    </div>
  </div>

  <canvas id="scope"></canvas>
  <div class="legend">
    <span><span class="dot" style="background:var(--trace)"></span>Signal (GPIO34)</span>
    <span><span class="dot" style="background:var(--dac)"></span>DAC level (GPIO25)</span>
    <span><span class="dot" style="background:var(--gap)"></span>Missing data</span>
  </div>

  <div class="dac-panel">
    <h3>⚡ DAC OUTPUT — GPIO25</h3>
    <div class="dac-row">
      <div class="control">
        <label><span>Output voltage</span>
               <span id="dacSliderText" style="color:var(--dac)">0.000 V</span></label>
        <input id="dacSlider" type="range" min="0" max="255" value="0" step="1">
      </div>
      <div class="dac-presets">
        <button class="preset" onclick="setDacVolt(0)">0 V</button>
        <button class="preset" onclick="setDacVolt(0.5)">0.5 V</button>
        <button class="preset" onclick="setDacVolt(1.0)">1.0 V</button>
        <button class="preset" onclick="setDacVolt(1.65)">1.65 V</button>
        <button class="preset" onclick="setDacVolt(2.0)">2.0 V</button>
        <button class="preset" onclick="setDacVolt(3.3)">3.3 V</button>
      </div>
    </div>
  </div>

  <div class="foot">
    Input: GPIO34, 0-3.3 V max. | Output: GPIO25, 0-3.3 V (~13 mV steps).
    Browser holds full dataset - CSV exports everything collected since page load or last clear.
  </div>
</div>

<script>
const canvas     = document.getElementById('scope');
const ctx        = canvas.getContext('2d');
const liveVEl    = document.getElementById('liveV');
const rateEl     = document.getElementById('rate');
const statusEl   = document.getElementById('status');
const droppedEl  = document.getElementById('dropped');
const storedEl   = document.getElementById('stored');
const dacDisplay = document.getElementById('dacDisplay');

const windowRange   = document.getElementById('windowRange');
const scaleRange    = document.getElementById('scaleRange');
const dacSlider     = document.getElementById('dacSlider');
const windowText    = document.getElementById('windowText');
const scaleText     = document.getElementById('scaleText');
const dacSliderText = document.getElementById('dacSliderText');

let timeWindowS  = Number(windowRange.value);
let fullScaleV   = Number(scaleRange.value);
let dacVolt      = 0;
let totalDropped = 0;

const MAX_STORE = 2_000_000;
let storeTUs  = new Uint32Array(MAX_STORE);
let storeV    = new Float32Array(MAX_STORE);
let storeGap  = new Uint8Array(MAX_STORE);
let storeLen  = 0;

let lastSeq = 0;
let sampleIntervalUs = 1000;

const POLL_MS = 100;

function dacRaw255ToVolt(r) { return (r / 255) * 3.3; }
function voltToDacRaw255(v) { return Math.round(Math.min(3.3, Math.max(0, v)) / 3.3 * 255); }

function updateDacDisplays(raw) {
  dacVolt = dacRaw255ToVolt(raw);
  dacDisplay.textContent    = `${dacVolt.toFixed(3)} V`;
  dacSliderText.textContent = `${dacVolt.toFixed(3)} V`;
  dacSlider.value           = raw;
}
async function sendDac(raw) {
  updateDacDisplays(raw);
  try { await fetch(`/dac?value=${raw}`, { cache:'no-store' }); } catch(e) {}
}
function setDacVolt(v) { sendDac(voltToDacRaw255(v)); }
dacSlider.oninput = () => sendDac(Number(dacSlider.value));

windowRange.oninput = () => {
  timeWindowS = Number(windowRange.value);
  windowText.textContent = `${timeWindowS} s`;
};
scaleRange.oninput = () => {
  fullScaleV = Number(scaleRange.value);
  scaleText.textContent = `${fullScaleV.toFixed(1)} V`;
};

windowText.textContent = `${timeWindowS} s`;
scaleText.textContent  = `${fullScaleV.toFixed(1)} V`;

function resizeCanvas() {
  const dpr  = Math.max(1, window.devicePixelRatio || 1);
  const rect = canvas.getBoundingClientRect();
  canvas.width  = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
addEventListener('resize', resizeCanvas);
resizeCanvas();

function rawToVolt(r) { return (r / 4095) * 3.3; }

async function poll() {
  try {
    const res = await fetch(`/samples?since=${lastSeq}`, { cache:'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const data = await res.json();
    sampleIntervalUs = data.sampleIntervalUs || sampleIntervalUs;

    if (Array.isArray(data.samples) && data.samples.length) {
      if (storeLen >= MAX_STORE) {
        statusEl.textContent = 'Buffer full!';
        statusEl.className   = 'warn';
      } else {
        for (const s of data.samples) {
          const seq = s[0];
          const tUs = s[1];
          const raw = s[2];
          const v   = rawToVolt(raw);

          // Real packet loss detection: delta compression never creates seq gaps
          const gap = (lastSeq !== 0 && seq !== lastSeq + 1) ? 1 : 0;
          if (gap) {
            totalDropped += seq - lastSeq - 1;
            droppedEl.textContent = totalDropped;
          }

          if (storeLen === 0) {
            // First point: just store as-is
            storeTUs[0] = tUs;
            storeV[0]   = v;
            storeGap[0] = gap;
            storeLen    = 1;
          } else {
            const prevT = storeTUs[storeLen - 1];
            const prevV = storeV[storeLen - 1];

            // Synthetic hold point for previous level, just before new sample
            let tHold = tUs - sampleIntervalUs;
            if (tHold <= prevT) tHold = prevT; // clamp to avoid time going backwards

            if (storeLen + 2 > MAX_STORE) break;

            // 1) Hold point (no gap)
            storeTUs[storeLen] = tHold;
            storeV[storeLen]   = prevV;
            storeGap[storeLen] = 0;
            storeLen++;

            // 2) New sample (with gap marker if there was loss)
            storeTUs[storeLen] = tUs;
            storeV[storeLen]   = v;
            storeGap[storeLen] = gap;
            storeLen++;
          }

          lastSeq = seq;
        }
        storedEl.textContent = storeLen.toLocaleString();
        liveVEl.textContent  = `${storeV[storeLen - 1].toFixed(3)} V`;
      }
    }

    const rate = 1000000 / sampleIntervalUs;
    rateEl.textContent   = rate >= 1000
      ? `${(rate/1000).toFixed(1)} kSa/s`
      : `${rate.toFixed(0)} Sa/s`;
    if (storeLen < MAX_STORE) {
      statusEl.textContent = 'Online';
      statusEl.className   = 'ok';
    }
  } catch(e) {
    statusEl.textContent = 'Reconnecting';
    statusEl.className   = 'warn';
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

function clearData() {
  storeLen     = 0;
  lastSeq      = 0;
  totalDropped = 0;
  droppedEl.textContent = '0';
  storedEl.textContent  = '0';
  liveVEl.textContent   = '--.- V';
}

function downloadCSV() {
  if (storeLen === 0) { alert('No data collected yet.'); return; }
  const rows  = ['seq,time_us,voltage_v\r\n'];
  const CHUNK = 50000;
  let blobs   = [new Blob(rows, { type: 'text/csv' })];
  rows.length = 0;

  for (let i = 0; i < storeLen; i++) {
    rows.push(`${i + 1},${storeTUs[i]},${storeV[i].toFixed(4)}\r\n`);
    if (rows.length >= CHUNK) {
      blobs.push(new Blob(rows, { type: 'text/csv' }));
      rows.length = 0;
    }
  }
  if (rows.length) blobs.push(new Blob(rows, { type: 'text/csv' }));

  const blob = new Blob(blobs, { type: 'text/csv' });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href     = url;
  a.download = `scope_${Date.now()}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
}

function drawGrid(w, h) {
  ctx.fillStyle = '#050805';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#16351a';
  ctx.lineWidth = 1;
  const vDivs = 8, hDivs = 10;
  for (let i = 0; i <= vDivs; i++) {
    const y = (h / vDivs) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  for (let i = 0; i <= hDivs; i++) {
    const x = (w / hDivs) * i;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  ctx.fillStyle = '#8eb596';
  ctx.font = '12px system-ui,sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  for (let i = 0; i <= vDivs; i++) {
    const y = (h / vDivs) * i;
    const v = fullScaleV * (1 - i / vDivs);
    ctx.fillText(v.toFixed(2) + 'V', 6, Math.max(0, y - 14));
  }
}

function drawDacLine(w, h) {
  const y = h - (dacVolt / fullScaleV) * h;
  if (y < 0 || y > h) return;
  ctx.save();
  ctx.strokeStyle = '#69c8ff88';
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 4]);
  ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#69c8ffcc';
  ctx.font = '11px system-ui,sans-serif';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'bottom';
  ctx.fillText(`DAC ${dacVolt.toFixed(2)}V`, w - 6, y - 2);
  ctx.restore();
}

function drawTrace(w, h) {
  if (storeLen < 2) return;

  const newestUs = storeTUs[storeLen - 1];
  const cutoffUs = newestUs - timeWindowS * 1_000_000;

  let lo = 0, hi = storeLen - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (storeTUs[mid] < cutoffUs) lo = mid + 1;
    else hi = mid;
  }
  const startIdx = lo;
  const windowUs = timeWindowS * 1_000_000;

  // Gap markers
  for (let i = startIdx; i < storeLen; i++) {
    if (!storeGap[i]) continue;
    const x = ((storeTUs[i] - cutoffUs) / windowUs) * w;
    ctx.save();
    ctx.fillStyle   = '#ff444422';
    ctx.strokeStyle = '#ff4444cc';
    ctx.lineWidth   = 1;
    ctx.fillRect(x - 1, 0, 3, h);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.restore();
  }

  ctx.strokeStyle = '#61ff79';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  let started = false;
  for (let i = startIdx; i < storeLen; i++) {
    const x = ((storeTUs[i] - cutoffUs) / windowUs) * w;
    const y = Math.min(h, Math.max(0, h - (storeV[i] / fullScaleV) * h));
    if (!started || storeGap[i]) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function animate() {
  requestAnimationFrame(animate);
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  drawGrid(w, h);
  drawDacLine(w, h);
  drawTrace(w, h);
}
animate();
poll();
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleSamples() {
  uint32_t latest = totalSamples;
  uint32_t since  = 0;
  if (server.hasArg("since")) since = strtoul(server.arg("since").c_str(), nullptr, 10);

  uint32_t oldest   = (latest > BUFFER_SIZE) ? (latest - BUFFER_SIZE + 1) : 1;
  uint32_t startSeq = since + 1;
  if (startSeq < oldest) startSeq = oldest;
  if (latest >= startSeq && (latest - startSeq + 1) > MAX_JSON_SAMPLES)
    startSeq = latest - MAX_JSON_SAMPLES + 1;

  String json;
  json.reserve(10000);
  json += "{\"latest\":";           json += latest;
  json += ",\"oldest\":";           json += oldest;
  json += ",\"sampleIntervalUs\":"; json += SAMPLE_INTERVAL_US;
  json += ",\"samples\":[";
  bool first = true;
  if (latest >= startSeq) {
    for (uint32_t seq = startSeq; seq <= latest; ++seq) {
      size_t idx = (seq - 1) % BUFFER_SIZE;
      if (!first) json += ",";
      first = false;
      json += "["; json += seq;      json += ",";
      json += sampleTimeUs[idx];     json += ",";
      json += sampleRaw[idx];        json += "]";
    }
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleDac() {
  if (server.hasArg("value")) {
    int v = constrain(server.arg("value").toInt(), 0, 255);
    dacValue = (uint8_t)v;
    dacWrite(DAC_PIN, dacValue);
  }
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(MONITOR_PIN, INPUT);
  dacWrite(DAC_PIN, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/",        handleRoot);
  server.on("/samples", handleSamples);
  server.on("/dac",     handleDac);
  server.onNotFound(handleNotFound);
  server.begin();

  esp_timer_handle_t timer;
  esp_timer_create_args_t args = {
    .callback        = onSampleTimer,
    .arg             = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name            = "scope_sample"
  };
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, SAMPLE_INTERVAL_US);

  Serial.println("\nTemporary oscilloscope + DAC firmware");
  Serial.print("AP: ");          Serial.println(AP_SSID);
  Serial.print("Open: http://"); Serial.println(WiFi.softAPIP());
  Serial.printf("ADC GPIO%d @ %lu Sa/s  |  DAC GPIO%d\n",
                MONITOR_PIN, 1000000UL / SAMPLE_INTERVAL_US, DAC_PIN);
}

void loop() {
  server.handleClient();
}
