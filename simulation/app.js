'use strict';

/* ============================================================
 * State
 * ============================================================ */
const NUM_DEVICES = 4;
const HISTORY_LEN = 60;

const state = {
  connected: false,
  activeDevice: 0,         // which device the fixed panel is simulating (0-3)
  displayDevice: 0,        // which device the wearable is showing
  cycleTimer: 0,
  packetsSent: 0,
  packetsRecv: 0,
  rssi: 75,                 // 0-100
  ota: { running: false, progress: 0, intervalId: null },
  sampleInterval: 3000,     // ms
  sleeping: false,
  thresholds: { aqi: 100, temp: 40, humidity: 80 },

  devices: Array.from({ length: NUM_DEVICES }, (_, i) => ({
    id: i + 1,
    active: i === 0,
    batt: 80 - i * 10,
    data: { temp: 22 + i, hum: 50 + i * 3, pres: 1013, gas: 95000, aqi: 45 + i * 10 },
  })),

  history: { temp: [], hum: [], aqi: [] },
  log: [],
};

/* ============================================================
 * OLED Renderer  (128×64, white pixels on black)
 * ============================================================ */
const oled = document.getElementById('oled');
const ctx  = oled.getContext('2d');
oled.width  = 128;
oled.height = 64;
const SCALE = 3;
oled.style.width  = `${128 * SCALE}px`;
oled.style.height = `${64  * SCALE}px`;

function oledClear() {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, 128, 64);
}

function oledText(text, x, y, small = false) {
  ctx.font = small ? '6px monospace' : '8px monospace';
  ctx.fillStyle = '#fff';
  ctx.fillText(text, x, y);
}

function oledRect(x, y, w, h, fill = false) {
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1;
  if (fill) {
    ctx.fillStyle = '#fff';
    ctx.fillRect(x, y, w, h);
  } else {
    ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
  }
}

function renderOLED(d, deviceId, warnAqi, warnTemp, warnHum) {
  oledClear();
  const alertActive = d.aqi >= warnAqi || d.temp >= warnTemp || d.hum >= warnHum;
  if (alertActive && Math.floor(Date.now() / 300) % 2 === 0) {
    oledClear(); // flash blank
    return;
  }
  // Title bar
  oledText(`AQM Device ${deviceId}`, 2, 9);
  oledRect(0, 0, 128, 11, false);
  // Metrics
  oledText(`T: ${d.temp.toFixed(1)} C`, 2, 22);
  oledText(`H: ${d.hum.toFixed(0)} %`, 2, 32);
  oledText(`P: ${d.pres.toFixed(0)} hPa`, 2, 42);
  oledText(`AQI: ${d.aqi.toFixed(0)}`, 2, 52);

  // AQI bar
  const barW = Math.min(60, Math.round((d.aqi / 200) * 60));
  oledRect(65, 44, 60, 8, false);
  if (barW > 0) oledRect(65, 44, barW, 8, true);

  // Battery indicator (top right)
  const battW = Math.round((state.devices[deviceId - 1].batt / 100) * 16);
  oledRect(108, 1, 18, 8, false);
  oledRect(126, 3, 2, 4, true);
  if (battW > 0) oledRect(109, 2, battW, 6, true);
}

/* ============================================================
 * Chart.js
 * ============================================================ */
const chartCtx = document.getElementById('sensor-chart').getContext('2d');
const chart = new Chart(chartCtx, {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      { label: 'Temp (°C)',   data: [], borderColor: '#4f8ef7', tension: .3, pointRadius: 0, borderWidth: 2 },
      { label: 'Humidity (%)',data: [], borderColor: '#34d399', tension: .3, pointRadius: 0, borderWidth: 2 },
      { label: 'AQI',         data: [], borderColor: '#f59e0b', tension: .3, pointRadius: 0, borderWidth: 2 },
    ],
  },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: true,
    plugins: { legend: { labels: { color: '#94a3b8', boxWidth: 12, font: { size: 11 } } } },
    scales: {
      x: { ticks: { color: '#64748b', maxTicksLimit: 8, font: { size: 10 } }, grid: { color: '#1e2230' } },
      y: { ticks: { color: '#64748b', font: { size: 10 } }, grid: { color: '#1e2230' } },
    },
  },
});

function pushHistory(d) {
  const now = new Date().toLocaleTimeString('en', { hour12: false });
  if (state.history.temp.length >= HISTORY_LEN) {
    state.history.temp.shift();
    state.history.hum.shift();
    state.history.aqi.shift();
    chart.data.labels.shift();
  }
  state.history.temp.push(+d.temp.toFixed(1));
  state.history.hum.push(+d.hum.toFixed(0));
  state.history.aqi.push(+d.aqi.toFixed(0));
  chart.data.labels.push(now);
  chart.data.datasets[0].data = [...state.history.temp];
  chart.data.datasets[1].data = [...state.history.hum];
  chart.data.datasets[2].data = [...state.history.aqi];
  chart.update();
}

/* ============================================================
 * Random walk simulation
 * ============================================================ */
function walk(val, min, max, step) {
  return Math.max(min, Math.min(max, val + (Math.random() - 0.5) * step * 2));
}

function tickDevice(dev) {
  const d = dev.data;
  d.temp = walk(d.temp, 15, 45, 0.4);
  d.hum  = walk(d.hum, 20, 95, 1);
  d.pres = walk(d.pres, 990, 1030, 0.5);
  d.gas  = walk(d.gas, 20000, 200000, 2000);
  d.aqi  = Math.max(0, Math.min(300, d.aqi + (Math.random() - 0.48) * 6));
  dev.batt = Math.max(0, dev.batt - 0.02);
}

/* ============================================================
 * DOM helpers
 * ============================================================ */
function $(id) { return document.getElementById(id); }
function aqiClass(v) { return v < 50 ? 'aqi-good' : v < 150 ? 'aqi-moderate' : 'aqi-bad'; }

function renderFixed() {
  const dev = state.devices[state.activeDevice];
  const d = dev.data;

  $('f-temp').textContent  = d.temp.toFixed(1) + ' °C';
  $('f-hum').textContent   = d.hum.toFixed(0)  + ' %';
  $('f-pres').textContent  = d.pres.toFixed(0) + ' hPa';
  $('f-gas').textContent   = (d.gas / 1000).toFixed(1) + ' kΩ';

  const aqiEl = $('f-aqi');
  aqiEl.textContent = d.aqi.toFixed(0);
  aqiEl.className = 'metric-value ' + aqiClass(d.aqi);

  // battery
  const bp = Math.round(dev.batt);
  $('f-batt-pct').textContent = bp + '%';
  const fill = $('f-batt-fill');
  fill.style.width = bp + '%';
  fill.className = 'batt-bar-fill' + (bp < 20 ? ' low' : '');

  // sleep badge
  $('sleep-badge').className = 'sleep-badge' + (state.sleeping ? ' sleeping' : '');
  $('sleep-badge').textContent = state.sleeping ? 'SLEEP' : 'AWAKE';

  // alert check
  const alert = d.aqi >= state.thresholds.aqi || d.temp >= state.thresholds.temp || d.hum >= state.thresholds.humidity;
  $('alert-indicator').style.display = alert ? 'inline' : 'none';
}

function renderBLE() {
  $('pkt-sent').textContent  = state.packetsSent;
  $('pkt-recv').textContent  = state.packetsRecv;
  $('rssi-fill').style.width = state.rssi + '%';
  $('ble-line').className    = state.connected ? '' : 'disconnected';
  const btn = $('connect-btn');
  btn.textContent = state.connected ? 'Disconnect' : 'Connect';
  btn.className   = state.connected ? 'connected' : '';
}

function renderWearable() {
  const dev = state.devices[state.displayDevice];
  renderOLED(dev.data, dev.id, state.thresholds.aqi, state.thresholds.temp, state.thresholds.humidity);
  $('w-device-label').textContent = `Device ${dev.id} / ${state.devices.filter(d => d.active).length} active`;

  const wp = Math.round(dev.batt);
  $('w-batt-pct').textContent = wp + '%';
  const wf = $('w-batt-fill');
  wf.style.width = wp + '%';
  wf.className = 'batt-bar-fill' + (wp < 20 ? ' low' : '');
}

function addLogEntry(d, devId) {
  const entry = {
    t: new Date().toLocaleTimeString('en', { hour12: false }),
    devId,
    temp: d.temp.toFixed(1),
    hum:  d.hum.toFixed(0),
    aqi:  d.aqi.toFixed(0),
  };
  state.log.unshift(entry);
  if (state.log.length > 20) state.log.pop();

  const tbody = $('log-body');
  tbody.innerHTML = state.log.map(e =>
    `<tr><td>${e.t}</td><td>${e.devId}</td><td>${e.temp}</td><td>${e.hum}</td><td class="${aqiClass(+e.aqi)}">${e.aqi}</td></tr>`
  ).join('');
}

/* ============================================================
 * Main tick
 * ============================================================ */
let lastSample = 0;
let wearableCycle = 0;

function tick() {
  const now = Date.now();

  // Tick all active devices
  state.devices.filter(d => d.active).forEach(tickDevice);

  // Sample + BLE send on interval
  if (now - lastSample >= state.sampleInterval) {
    lastSample = now;
    state.sleeping = false;

    const dev = state.devices[state.activeDevice];
    addLogEntry(dev.data, dev.id);
    pushHistory(dev.data);

    if (state.connected) {
      state.packetsSent++;
      state.packetsRecv++;
      state.rssi = Math.max(10, Math.min(100, state.rssi + (Math.random() - 0.5) * 10));
    }
  } else {
    // Show sleeping between samples
    const timeToNext = state.sampleInterval - (now - lastSample);
    state.sleeping = (timeToNext > 50 && !state.connected);
  }

  // Wearable: cycle device every 5 s
  const activeDev = state.devices.filter(d => d.active);
  if (activeDev.length > 1 && now - wearableCycle >= 5000) {
    wearableCycle = now;
    const idx = activeDev.findIndex(d => d.id === state.devices[state.displayDevice].id);
    const next = activeDev[(idx + 1) % activeDev.length];
    state.displayDevice = state.devices.indexOf(next);
  }

  renderFixed();
  renderBLE();
  renderWearable();
}

setInterval(tick, 250);

/* ============================================================
 * Event wiring
 * ============================================================ */

// Device selector buttons (fixed panel)
document.querySelectorAll('.device-btn[data-dev]').forEach(btn => {
  btn.addEventListener('click', () => {
    state.activeDevice = +btn.dataset.dev;
    document.querySelectorAll('.device-btn[data-dev]').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
  });
});

// Device active toggles (mesh)
document.querySelectorAll('.mesh-toggle').forEach(cb => {
  cb.addEventListener('change', () => {
    const id = +cb.dataset.dev;
    state.devices[id].active = cb.checked;
  });
});

// Sample interval slider
$('interval-slider').addEventListener('input', e => {
  state.sampleInterval = +e.target.value * 1000;
  $('interval-val').textContent = e.target.value + 's';
});

// Connect / disconnect
$('connect-btn').addEventListener('click', () => {
  state.connected = !state.connected;
});

// Threshold sliders
$('thr-aqi').addEventListener('input', e => {
  state.thresholds.aqi = +e.target.value;
  $('thr-aqi-val').textContent = e.target.value;
});
$('thr-temp').addEventListener('input', e => {
  state.thresholds.temp = +e.target.value;
  $('thr-temp-val').textContent = e.target.value;
});
$('thr-hum').addEventListener('input', e => {
  state.thresholds.humidity = +e.target.value;
  $('thr-hum-val').textContent = e.target.value;
});

// OTA button
$('ota-btn').addEventListener('click', () => {
  if (state.ota.running) return;
  if (!state.connected) { alert('Connect BLE first'); return; }
  state.ota.running = true;
  state.ota.progress = 0;
  $('ota-bar-fill').style.width = '0%';
  $('ota-status').textContent = 'Uploading…';
  state.ota.intervalId = setInterval(() => {
    state.ota.progress = Math.min(100, state.ota.progress + Math.random() * 4 + 1);
    $('ota-bar-fill').style.width = state.ota.progress.toFixed(0) + '%';
    if (state.ota.progress >= 100) {
      clearInterval(state.ota.intervalId);
      state.ota.running = false;
      $('ota-status').textContent = 'Complete ✓ (CRC OK)';
    }
  }, 120);
});

// Reset all
$('reset-btn').addEventListener('click', () => {
  state.devices.forEach((dev, i) => {
    dev.batt = 80 - i * 10;
    dev.data = { temp: 22 + i, hum: 50 + i * 3, pres: 1013, gas: 95000, aqi: 45 + i * 10 };
  });
  state.history.temp = [];
  state.history.hum  = [];
  state.history.aqi  = [];
  state.log = [];
  chart.data.labels = [];
  chart.data.datasets.forEach(ds => { ds.data = []; });
  chart.update();
  $('log-body').innerHTML = '';
  $('ota-status').textContent = 'Idle';
  $('ota-bar-fill').style.width = '0%';
});

// Theme toggle
$('theme-btn').addEventListener('click', () => {
  document.body.classList.toggle('light');
  $('theme-btn').textContent = document.body.classList.contains('light') ? '🌙 Dark' : '☀️ Light';
});

// Kick off
tick();
