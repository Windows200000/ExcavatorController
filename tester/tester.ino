#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <driver/adc.h>

const char* AP_SSID     = "ExcavatorAP";
const char* AP_PASSWORD = "exc@vator123";

const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

const int MONITOR_PIN         = 34;   // ADC1_CHANNEL_6
const int DAC_PIN             = 25;

// High-speed single-shot sampling with delta compression
// timerBegin() takes frequency in Hz; 1000000/20 = 50000 Hz = 50 kSa/s target
const uint32_t SAMPLE_INTERVAL_US = 20;
const uint32_t SAMPLE_FREQ_HZ     = 1000000UL / SAMPLE_INTERVAL_US;  // 50000
const uint32_t MAX_JSON_SAMPLES   = 500;   // hard cap per HTTP response
const size_t   BUFFER_SIZE        = 4000;  // stored (compressed) sample ring buffer

// Delta compression: store only when raw changes by >= 16 LSBs (~12.9 mV at 3.3V/12-bit)
const uint16_t DELTA_THRESH_RAW = 16;

// Force a sample at least every 100 ms so a flat/quiet signal still streams
const uint32_t MAX_GAP_US = 100000;

WebServer server(80);

// Ring buffer placed in DRAM (no flash-cache miss latency from ISR)
static uint16_t DRAM_ATTR sampleRaw[BUFFER_SIZE];
static uint32_t DRAM_ATTR sampleTimeUs[BUFFER_SIZE];

volatile uint32_t totalSamples  = 0;  // count of stored (compressed) samples
volatile uint32_t writeIdx      = 0;  // rolling write index — avoids % in hot path
volatile uint16_t lastStoredRaw = 0;  // seeded in setup() before timer starts

// Debug counters — updated only in ISR, read only in loop()
volatile uint32_t DRAM_ATTR dbgIsrFired  = 0;  // total ISR invocations
volatile uint32_t DRAM_ATTR dbgIsrStored = 0;  // ISR invocations that stored a sample

uint8_t dacValue = 0;

// Hardware timer ISR — called directly by the hardware timer peripheral.
// Arduino-ESP32 v3 hardware timer callbacks take no arguments.
// IRAM_ATTR ensures the function is resident in IRAM so it runs even
// when the flash cache is busy serving WiFi / WebServer code.
void IRAM_ATTR onSampleTimer() {
  dbgIsrFired++;

  uint16_t raw = (uint16_t)adc1_get_raw(ADC1_CHANNEL_6);
  uint32_t now = (uint32_t)esp_timer_get_time();

  // Delta compression: skip if change is below threshold
  int diff = (int)raw - (int)lastStoredRaw;
  bool changed = diff >= (int)DELTA_THRESH_RAW || diff <= -(int)DELTA_THRESH_RAW;

  // Force a store if no sample has been written yet, or if too much time has passed
  uint32_t lastT = (writeIdx == 0) ? sampleTimeUs[BUFFER_SIZE - 1] : sampleTimeUs[writeIdx - 1];
  bool timeout = (totalSamples == 0) || ((now - lastT) > MAX_GAP_US);

  if (!changed && !timeout) return;

  // Store sample
  sampleRaw[writeIdx]    = raw;
  sampleTimeUs[writeIdx] = now;
  lastStoredRaw          = raw;

  // Advance rolling index without division
  if (++writeIdx >= BUFFER_SIZE) writeIdx = 0;
  totalSamples++;
  dbgIsrStored++;
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

          const gap = (lastSeq !== 0 && seq !== lastSeq + 1) ? 1 : 0;
          if (gap) {
            totalDropped += seq - lastSeq - 1;
            droppedEl.textContent = totalDropped;
          }

          if (storeLen === 0) {
            storeTUs[0] = tUs;
            storeV[0]   = v;
            storeGap[0] = gap;
            storeLen    = 1;
          } else {
            const prevT = storeTUs[storeLen - 1];
            const prevV = storeV[storeLen - 1];

            let tHold = tUs - sampleIntervalUs;
            if (tHold <= prevT) tHold = prevT;

            if (storeLen + 2 > MAX_STORE) break;

            storeTUs[storeLen] = tHold;
            storeV[storeLen]   = prevV;
            storeGap[storeLen] = 0;
            storeLen++;

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
    rateEl.textContent = rate >= 1000
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

// Serialise uint32 into buf[], returns number of chars written (no null terminator).
static int u32toa(uint32_t v, char* buf) {
  if (v == 0) { buf[0] = '0'; return 1; }
  char tmp[10]; int n = 0;
  while (v) { tmp[n++] = '0' + (v % 10); v /= 10; }
  for (int i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
  return n;
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

  uint32_t sendCount = (latest >= startSeq) ? (latest - startSeq + 1) : 0;
  Serial.printf("[SAMPLES] since=%lu latest=%lu oldest=%lu startSeq=%lu sending=%lu\n",
                (unsigned long)since, (unsigned long)latest,
                (unsigned long)oldest, (unsigned long)startSeq,
                (unsigned long)sendCount);

  // Fixed stack buffer avoids heap fragmentation during JSON build.
  // Max: header ~60 + 500 samples * ~28 bytes = ~14 060 bytes; 16 KB is safe.
  static char jsonBuf[16384];
  int pos = 0;

  pos += snprintf(jsonBuf + pos, sizeof(jsonBuf) - pos,
                  "{\"latest\":%lu,\"oldest\":%lu,\"sampleIntervalUs\":%lu,\"samples\":[",
                  (unsigned long)latest, (unsigned long)oldest,
                  (unsigned long)SAMPLE_INTERVAL_US);

  bool first = true;
  if (latest >= startSeq) {
    for (uint32_t seq = startSeq; seq <= latest && pos < (int)sizeof(jsonBuf) - 40; ++seq) {
      size_t idx = (seq - 1) % BUFFER_SIZE;
      if (!first) jsonBuf[pos++] = ',';
      first = false;
      jsonBuf[pos++] = '[';
      pos += u32toa(seq,               jsonBuf + pos);
      jsonBuf[pos++] = ',';
      pos += u32toa(sampleTimeUs[idx], jsonBuf + pos);
      jsonBuf[pos++] = ',';
      pos += u32toa(sampleRaw[idx],    jsonBuf + pos);
      jsonBuf[pos++] = ']';
    }
  }
  jsonBuf[pos++] = ']';
  jsonBuf[pos++] = '}';
  jsonBuf[pos]   = '\0';

  server.send(200, "application/json", jsonBuf);
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

// ── Loop debug: print ISR stats every 2 s ──────────────────────────────────
static uint32_t lastDbgMs       = 0;
static uint32_t lastDbgFired    = 0;
static uint32_t lastDbgStored   = 0;
static uint32_t lastDbgSamples  = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Configure ADC1 via IDF for fastest direct single-shot reads
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO34 = ADC1_CH6

  dacWrite(DAC_PIN, 0);

  // Prime the delta baseline so the first ISR tick has a valid lastStoredRaw
  // to compare against. Do NOT seed a fake stored sample — totalSamples and
  // writeIdx stay at 0 so the browser's since-cursor starts correctly.
  lastStoredRaw = (uint16_t)adc1_get_raw(ADC1_CHANNEL_6);
  totalSamples  = 0;
  writeIdx      = 0;
  dbgIsrFired   = 0;
  dbgIsrStored  = 0;

  Serial.printf("[BOOT] lastStoredRaw seed = %u  (%.3f V)\n",
                (unsigned)lastStoredRaw, lastStoredRaw * 3.3f / 4095.0f);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/",        handleRoot);
  server.on("/samples", handleSamples);
  server.on("/dac",     handleDac);
  server.onNotFound(handleNotFound);
  server.begin();

  // Arduino-ESP32 v3 hardware timer API:
  //   timerBegin(freq_hz) — configures a hardware timer peripheral at the
  //   given frequency. Returns a hw_timer_t* handle.
  //   timerAttachInterrupt attaches the IRAM_ATTR ISR; timerStart arms it.
  // This fires a true hardware interrupt, bypassing FreeRTOS scheduling
  // and WiFi task preemption, without requiring ESP_TIMER_ISR.
  hw_timer_t* hwTimer = timerBegin(SAMPLE_FREQ_HZ);
  timerAttachInterrupt(hwTimer, &onSampleTimer);
  timerStart(hwTimer);

  lastDbgMs = millis();
  Serial.println("\nTemporary oscilloscope + DAC firmware");
  Serial.print("AP: ");          Serial.println(AP_SSID);
  Serial.print("Open: http://"); Serial.println(WiFi.softAPIP());
  Serial.printf("ADC GPIO%d @ %lu Sa/s  |  DAC GPIO%d  |  delta=%u raw  |  max_gap=%lu us\n",
                MONITOR_PIN, (unsigned long)SAMPLE_FREQ_HZ, DAC_PIN, DELTA_THRESH_RAW,
                (unsigned long)MAX_GAP_US);
  Serial.println("[DBG] ISR debug prints every 2 s: fired/stored/totalSamples/lastRaw");
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastDbgMs >= 2000) {
    lastDbgMs = now;

    // Snapshot volatile counters once
    uint32_t fired   = dbgIsrFired;
    uint32_t stored  = dbgIsrStored;
    uint32_t total   = totalSamples;
    uint16_t lastRaw = lastStoredRaw;

    Serial.printf("[DBG] ISR fired=%lu (+%lu)  stored=%lu (+%lu)  totalSamples=%lu  lastRaw=%u (%.3fV)\n",
                  (unsigned long)fired,   (unsigned long)(fired  - lastDbgFired),
                  (unsigned long)stored,  (unsigned long)(stored - lastDbgStored),
                  (unsigned long)total,
                  (unsigned)lastRaw, lastRaw * 3.3f / 4095.0f);

    lastDbgFired   = fired;
    lastDbgStored  = stored;
    lastDbgSamples = total;
  }
}
