#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <nvs.h>

// ===== WiFi Config =====
const char* SSID = "NIRINIUM_LEVEL";
const char* PASSWORD = "leveling123";
const char* NVS_NAMESPACE = "scopelevel";

// ===== Web Server =====
AsyncWebServer server(80);

// ===== Hardware =====
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// === RTC-RETAINED STATE ===
RTC_DATA_ATTR static int bootCount = 0;
RTC_DATA_ATTR static float rtcCalibrationOffset = 0.0f;
RTC_DATA_ATTR static bool rtcCalibrated = false;

// === PIN ASSIGNMENTS ===
const int GREEN_LED    = 21; // D6
const int YELLOW_LEFT  = 20; // D7
const int RED_LEFT     = 3;  // D1
const int YELLOW_RIGHT = 4;  // D2
const int RED_RIGHT    = 5;  // D3
const int BATT_PIN     = 2;  // A0/D0
const int POWER_SW_PIN = 10; // D10

// === I2C PINS ===
const int I2C_SDA = 6; // D4
const int I2C_SCL = 7; // D5

// === PWM CHANNELS ===
const int PWM_CH_GREEN = 0;
const int PWM_CH_YL    = 1;
const int PWM_CH_RL    = 2;
const int PWM_CH_YR    = 3;
const int PWM_CH_RR    = 4;
const int PWM_FREQ     = 5000;
const int PWM_RES      = 8;

// === ZONE THRESHOLDS (configurable via web) ===
float THRESHOLD  = 0.5;
float NEAR_ZONE  = 1.5;
float MAX_CANT   = 5.0;
float HYSTERESIS = 0.15;
float EMA_ALPHA  = 0.4;

// === STATE ===
unsigned long cycleStart = 0;
int currentCycle = 1500;
float filteredRoll = 0.0;
float calibrationOffset = 0.0;
int currentZone = 0;
float currentRoll = 0.0;
unsigned long lastBattCheck = 0;
float currentBattVoltage = 0.0f;
bool batteryLow = false;
bool firstReading = true;
bool calibrating = false;

void ledPWM(int channel, uint8_t brightness) {
  ledcWrite(channel, brightness);
}

void allLedsOff() {
  ledPWM(PWM_CH_GREEN, 0);
  ledPWM(PWM_CH_YL, 0);
  ledPWM(PWM_CH_RL, 0);
  ledPWM(PWM_CH_YR, 0);
  ledPWM(PWM_CH_RR, 0);
}

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  return (raw / 4095.0f) * 3.3f * 2.0f; // 100k/100k divider
}

void calibrate() {
  calibrating = true;
  float sum = 0.0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
    
    bool ledOn = (i % 10) < 5;
    uint8_t brightness = ledOn ? 150 : 0;
    ledPWM(PWM_CH_GREEN, brightness);
    ledPWM(PWM_CH_YL, brightness);
    ledPWM(PWM_CH_RL, brightness);
    ledPWM(PWM_CH_YR, brightness);
    ledPWM(PWM_CH_RR, brightness);
    delay(20);
  }
  calibrationOffset = sum / samples;
  rtcCalibrationOffset = calibrationOffset;
  rtcCalibrated = true;
  allLedsOff();
  calibrating = false;
  Serial.printf("Calibration offset: %.2f\n", calibrationOffset);
}

int getZoneWithHysteresis(float absRoll, int prevZone) {
  switch (prevZone) {
    case 0:
      if (absRoll >= THRESHOLD + HYSTERESIS) {
        return (absRoll >= NEAR_ZONE + HYSTERESIS) ? 2 : 1;
      }
      return 0;
    case 1:
      if (absRoll < THRESHOLD - HYSTERESIS) return 0;
      if (absRoll >= NEAR_ZONE + HYSTERESIS) return 2;
      return 1;
    case 2:
      if (absRoll < NEAR_ZONE - HYSTERESIS) {
        return (absRoll < THRESHOLD - HYSTERESIS) ? 0 : 1;
      }
      return 2;
  }
  return 0;
}

void saveThresholdsToNVS() {
  nvs_handle_t handle;
  nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  nvs_set_i32(handle, "threshold_x100", (int32_t)(THRESHOLD * 100));
  nvs_set_i32(handle, "nearzone_x100", (int32_t)(NEAR_ZONE * 100));
  nvs_set_i32(handle, "maxcant_x100", (int32_t)(MAX_CANT * 100));
  nvs_set_i32(handle, "hysteresis_x100", (int32_t)(HYSTERESIS * 100));
  nvs_set_i32(handle, "ema_alpha_x1000", (int32_t)(EMA_ALPHA * 1000));
  nvs_commit(handle);
  nvs_close(handle);
}

void loadThresholdsFromNVS() {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
    int32_t val;
    if (nvs_get_i32(handle, "threshold_x100", &val) == ESP_OK) THRESHOLD = val / 100.0f;
    if (nvs_get_i32(handle, "nearzone_x100", &val) == ESP_OK) NEAR_ZONE = val / 100.0f;
    if (nvs_get_i32(handle, "maxcant_x100", &val) == ESP_OK) MAX_CANT = val / 100.0f;
    if (nvs_get_i32(handle, "hysteresis_x100", &val) == ESP_OK) HYSTERESIS = val / 100.0f;
    if (nvs_get_i32(handle, "ema_alpha_x1000", &val) == ESP_OK) EMA_ALPHA = val / 1000.0f;
    nvs_close(handle);
  }
}

// ===== EMBEDDED HTML/CSS/JS =====
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NIRLABS // TACTICAL LEVEL SYS</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    :root {
      --green: #00ff41;
      --amber: #ffb000;
      --red:   #ff2828;
      --bg:    #050a05;
      --dim:   #003b00;
      --border:#1a4a1a;
    }
    body {
      font-family: 'Courier New', Courier, monospace;
      background: var(--bg);
      color: var(--green);
      padding: 14px;
      min-height: 100vh;
      text-transform: uppercase;
      background-image: repeating-linear-gradient(
        0deg, transparent, transparent 2px,
        rgba(0,255,65,0.025) 2px, rgba(0,255,65,0.025) 4px
      );
    }
    .scanline {
      position: fixed; top: 0; left: 0; width: 100%; height: 100%;
      pointer-events: none; z-index: 9999;
      background: repeating-linear-gradient(
        0deg, rgba(0,0,0,0.12) 0px, rgba(0,0,0,0.12) 1px,
        transparent 1px, transparent 2px
      );
    }
    .container { max-width: 800px; margin: 0 auto; }
    .header {
      border: 1px solid var(--green); padding: 10px 16px;
      margin-bottom: 12px; position: relative;
      display: flex; justify-content: space-between; align-items: center;
    }
    .header::before {
      content: '///'; position: absolute; top: -9px; left: 10px;
      background: var(--bg); padding: 0 6px;
      font-size: 0.65em; color: var(--green);
    }
    .header-title { font-size: 1.3em; font-weight: bold; letter-spacing: 4px; }
    .header-sub { font-size: 0.65em; color: #006a20; letter-spacing: 2px; margin-top: 2px; }
    .status-led {
      width: 9px; height: 9px; border-radius: 50%;
      background: var(--green); box-shadow: 0 0 7px var(--green);
      display: inline-block; animation: blink 2s step-end infinite;
    }
    @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }
    .grid-2 {
      display: grid; grid-template-columns: 1fr 1fr;
      gap: 10px; margin-bottom: 10px;
    }
    .panel {
      border: 1px solid var(--green); padding: 14px;
      position: relative; background: rgba(0,18,0,0.6);
    }
    .panel-label {
      position: absolute; top: -9px; left: 10px;
      background: var(--bg); padding: 0 6px;
      font-size: 0.6em; letter-spacing: 3px; color: var(--green);
    }
    .roll-primary {
      font-size: 3.8em; font-weight: bold; text-align: center;
      letter-spacing: 2px; line-height: 1; margin: 8px 0;
      text-shadow: 0 0 18px var(--green);
      transition: color 0.2s, text-shadow 0.2s;
    }
    .roll-primary.cant { color: var(--red); text-shadow: 0 0 18px var(--red); }
    .roll-primary.near { color: var(--amber); text-shadow: 0 0 18px var(--amber); }
    .roll-bar-track {
      height: 22px; background: #001200;
      border: 1px solid var(--green); position: relative; overflow: visible;
      margin: 14px 0 4px;
    }
    .roll-bar-fill {
      position: absolute; top: 0; bottom: 0;
      background: var(--green); opacity: 0.5;
      transition: left 0.15s, width 0.15s, background 0.15s;
    }
    .roll-bar-center {
      position: absolute; top: 0; bottom: 0; left: 50%;
      width: 1px; background: var(--green); opacity: 0.35;
    }
    .roll-bar-needle {
      position: absolute; top: -5px; bottom: -5px; width: 3px;
      background: var(--green); box-shadow: 0 0 8px var(--green);
      transform: translateX(-50%);
      transition: left 0.15s, background 0.15s, box-shadow 0.15s;
    }
    .roll-bar-needle.cant { background: var(--red); box-shadow: 0 0 8px var(--red); }
    .roll-bar-needle.near { background: var(--amber); box-shadow: 0 0 8px var(--amber); }
    .roll-bar-labels {
      display: flex; justify-content: space-between;
      font-size: 0.55em; color: #005010; margin-top: 3px;
    }
    .zone-box {
      text-align: center; padding: 6px;
      border: 1px solid var(--green); margin-top: 8px;
      font-size: 0.8em; letter-spacing: 5px;
      transition: border-color 0.2s, color 0.2s;
    }
    .zone-box.level { border-color: var(--green); color: var(--green); }
    .zone-box.near  { border-color: var(--amber); color: var(--amber); }
    .zone-box.cant  { border-color: var(--red); color: var(--red); animation: blink 0.4s step-end infinite; }
    .data-row {
      display: flex; justify-content: space-between; align-items: center;
      padding: 5px 0; border-bottom: 1px solid rgba(0,255,65,0.08);
      font-size: 0.78em;
    }
    .data-row:last-child { border-bottom: none; }
    .data-label { color: #006a20; letter-spacing: 1px; }
    .data-value { color: var(--green); font-weight: bold; }
    .batt-bar {
      display: inline-block; width: 52px; height: 9px;
      border: 1px solid var(--green); vertical-align: middle; position: relative; margin-left: 5px;
    }
    .batt-bar-fill {
      position: absolute; top: 1px; left: 1px; bottom: 1px;
      background: var(--green); transition: width 0.5s, background 0.5s;
    }
    .ctrl-row {
      display: flex; align-items: center; justify-content: space-between;
      padding: 7px 0; border-bottom: 1px solid rgba(0,255,65,0.08); gap: 10px;
    }
    .ctrl-row:last-child { border-bottom: none; }
    .ctrl-label { font-size: 0.72em; letter-spacing: 2px; color: #006a20; width: 140px; flex-shrink: 0; }
    .ctrl-val { font-size: 0.82em; color: var(--green); width: 46px; text-align: right; flex-shrink: 0; }
    input[type="range"] {
      -webkit-appearance: none; flex: 1; height: 3px;
      background: #002200; border: none; outline: none; cursor: pointer;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none; width: 13px; height: 13px;
      background: var(--green); box-shadow: 0 0 5px var(--green); cursor: pointer;
    }
    input[type="range"]::-moz-range-thumb {
      width: 13px; height: 13px; background: var(--green); border: none; cursor: pointer;
    }
    .btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 12px; }
    button {
      padding: 9px 14px; background: transparent;
      border: 1px solid var(--green); color: var(--green);
      font-family: 'Courier New', monospace; font-size: 0.75em;
      letter-spacing: 3px; text-transform: uppercase; cursor: pointer;
      transition: background 0.1s, box-shadow 0.1s;
    }
    button:hover {
      background: rgba(0,255,65,0.08);
      box-shadow: 0 0 10px rgba(0,255,65,0.25), inset 0 0 10px rgba(0,255,65,0.08);
    }
    button:active { background: rgba(0,255,65,0.2); }
    button.btn-alert { border-color: var(--amber); color: var(--amber); }
    button.btn-alert:hover {
      background: rgba(255,176,0,0.08);
      box-shadow: 0 0 10px rgba(255,176,0,0.25);
    }
    button:disabled { opacity: 0.3; cursor: not-allowed; box-shadow: none; }
    .footer {
      text-align: center; font-size: 0.58em; color: #004a10;
      letter-spacing: 3px; margin-top: 12px; padding: 8px;
      border-top: 1px solid var(--border);
    }
  </style>
</head>
<body>
<div class="scanline"></div>
<div class="container">
  <div class="header">
    <div>
      <div class="header-title">NIRLABS // ANTI-CANT SYS</div>
      <div class="header-sub">TACTICAL LEVEL CONTROL // v2.0</div>
    </div>
    <div style="text-align:right;font-size:0.68em;letter-spacing:2px;">
      <span class="status-led"></span>&nbsp;ONLINE<br>
      <span id="clockDisplay" style="color:#006a20">--:--:--</span>
    </div>
  </div>

  <div class="grid-2">
    <div class="panel">
      <div class="panel-label">[ ROLL ANGLE ]</div>
      <div class="roll-primary" id="rollValue">+0.0&deg;</div>
      <div class="roll-bar-track">
        <div class="roll-bar-fill" id="rollFill"></div>
        <div class="roll-bar-center"></div>
        <div class="roll-bar-needle" id="rollNeedle"></div>
      </div>
      <div class="roll-bar-labels">
        <span>-5&deg;</span><span>-2.5&deg;</span><span>0&deg;</span><span>+2.5&deg;</span><span>+5&deg;</span>
      </div>
      <div class="zone-box level" id="zoneBox">--- LEVEL ---</div>
    </div>

    <div class="panel">
      <div class="panel-label">[ SYSTEM STATUS ]</div>
      <div class="data-row">
        <span class="data-label">RAW ROLL</span>
        <span class="data-value" id="rollRaw">+0.0&deg;</span>
      </div>
      <div class="data-row">
        <span class="data-label">CAL OFFSET</span>
        <span class="data-value" id="calOffset">+0.00&deg;</span>
      </div>
      <div class="data-row">
        <span class="data-label">CALIBRATED</span>
        <span class="data-value" id="calibStatus">NO</span>
      </div>
      <div class="data-row">
        <span class="data-label">BOOT COUNT</span>
        <span class="data-value" id="bootCount">0</span>
      </div>
      <div class="data-row">
        <span class="data-label">BATTERY</span>
        <span class="data-value">
          <span id="batteryVoltage">0.00V</span>
          <span class="batt-bar"><span class="batt-bar-fill" id="battFill" style="width:80%"></span></span>
        </span>
      </div>
      <div class="data-row">
        <span class="data-label">LINK STATUS</span>
        <span class="data-value" id="status">ACQUIRING...</span>
      </div>
    </div>
  </div>

  <div class="panel" style="margin-bottom:10px;">
    <div class="panel-label">[ PARAMETERS ]</div>
    <div class="ctrl-row">
      <span class="ctrl-label">THRESHOLD</span>
      <input type="range" id="threshold" min="0.1" max="1.0" step="0.1" value="0.5">
      <span class="ctrl-val" id="thresholdVal">0.5&deg;</span>
    </div>
    <div class="ctrl-row">
      <span class="ctrl-label">NEAR ZONE</span>
      <input type="range" id="nearzone" min="0.5" max="3.0" step="0.1" value="1.5">
      <span class="ctrl-val" id="nearzoneVal">1.5&deg;</span>
    </div>
    <div class="ctrl-row">
      <span class="ctrl-label">MAX CANT</span>
      <input type="range" id="maxcant" min="2.0" max="10.0" step="0.5" value="5.0">
      <span class="ctrl-val" id="maxcantVal">5.0&deg;</span>
    </div>
    <div class="ctrl-row">
      <span class="ctrl-label">HYSTERESIS</span>
      <input type="range" id="hysteresis" min="0.05" max="0.5" step="0.05" value="0.15">
      <span class="ctrl-val" id="hysteresisVal">0.15&deg;</span>
    </div>
    <div class="ctrl-row">
      <span class="ctrl-label">EMA ALPHA</span>
      <input type="range" id="ema" min="0.1" max="0.9" step="0.1" value="0.4">
      <span class="ctrl-val" id="emaVal">0.4</span>
    </div>
    <div class="btn-row">
      <button id="saveButton" onclick="saveConfig()">[ COMMIT PARAMS ]</button>
      <button id="calibrateButton" class="btn-alert" onclick="triggerCalibrate()">[ CALIBRATE ]</button>
    </div>
  </div>

  <div class="footer">
    NIRINIUM|LABS &bull; ANTI-CANT LEVEL SYS &bull; RF: NIRINIUM_LEVEL &bull; IP: 192.168.4.1
  </div>
</div>

<script>
  setInterval(() => {
    const n = new Date();
    document.getElementById('clockDisplay').textContent =
      String(n.getHours()).padStart(2,'0') + ':' +
      String(n.getMinutes()).padStart(2,'0') + ':' +
      String(n.getSeconds()).padStart(2,'0');
  }, 1000);

  window.addEventListener('load', () => {
    loadStoredValues();
    updateDashboard();
    setInterval(updateDashboard, 500);
  });

  function loadStoredValues() {
    const defs = {threshold:'0.5',nearzone:'1.5',maxcant:'5.0',hysteresis:'0.15',ema:'0.4'};
    Object.keys(defs).forEach(k => {
      document.getElementById(k).value = localStorage.getItem(k) || defs[k];
    });
    updateLabels();
  }

  function updateLabels() {
    document.getElementById('thresholdVal').textContent  = document.getElementById('threshold').value  + '\xb0';
    document.getElementById('nearzoneVal').textContent   = document.getElementById('nearzone').value   + '\xb0';
    document.getElementById('maxcantVal').textContent    = document.getElementById('maxcant').value    + '\xb0';
    document.getElementById('hysteresisVal').textContent = document.getElementById('hysteresis').value + '\xb0';
    document.getElementById('emaVal').textContent        = document.getElementById('ema').value;
  }
  ['threshold','nearzone','maxcant','hysteresis','ema'].forEach(id => {
    document.getElementById(id).oninput = updateLabels;
  });

  const ZONE_NAMES   = ['--- LEVEL ---', '-- NEAR CANT --', '-- CANT DETECTED --'];
  const ZONE_CLASSES = ['level', 'near', 'cant'];
  const ZONE_COLORS  = ['#00ff41', '#ffb000', '#ff2828'];

  async function updateDashboard() {
    try {
      const data = await fetch('/api/status').then(r => r.json());
      const roll = data.roll;
      const sign = roll >= 0 ? '+' : '';
      const zone = data.zone || 0;

      const rollEl   = document.getElementById('rollValue');
      const zoneBox  = document.getElementById('zoneBox');
      const needle   = document.getElementById('rollNeedle');
      const fill     = document.getElementById('rollFill');

      rollEl.textContent = sign + roll.toFixed(1) + '\xb0';
      rollEl.className = 'roll-primary' + (zone === 2 ? ' cant' : zone === 1 ? ' near' : '');

      zoneBox.className   = 'zone-box ' + ZONE_CLASSES[zone];
      zoneBox.textContent = ZONE_NAMES[zone];
      needle.className    = 'roll-bar-needle' + (zone > 0 ? ' ' + ZONE_CLASSES[zone] : '');
      fill.style.background = ZONE_COLORS[zone];

      const maxRange = 5.0;
      const pct    = Math.max(0, Math.min(100, ((roll + maxRange) / (maxRange * 2)) * 100));
      needle.style.left = pct + '%';
      if (pct >= 50) {
        fill.style.left  = '50%';
        fill.style.width = (pct - 50) + '%';
      } else {
        fill.style.left  = pct + '%';
        fill.style.width = (50 - pct) + '%';
      }

      const s = v => (v >= 0 ? '+' : '') + v.toFixed(1) + '\xb0';
      document.getElementById('rollRaw').textContent    = s(data.rawRoll);
      document.getElementById('calOffset').textContent  = (data.calOffset >= 0 ? '+' : '') + data.calOffset.toFixed(2) + '\xb0';
      document.getElementById('calibStatus').textContent = data.calibrated ? 'YES' : 'NO';
      document.getElementById('bootCount').textContent  = data.bootCount;
      document.getElementById('batteryVoltage').textContent = data.battery.toFixed(2) + 'V';
      document.getElementById('status').textContent     = data.calibrating ? 'CALIBRATING...' : 'NOMINAL';

      const battPct   = Math.max(0, Math.min(100, (data.battery / 4.2) * 100));
      const battFill  = document.getElementById('battFill');
      battFill.style.width      = battPct + '%';
      battFill.style.background = battPct > 50 ? '#00ff41' : battPct > 20 ? '#ffb000' : '#ff2828';
    } catch (e) {
      document.getElementById('status').textContent = 'LINK LOST';
    }
  }

  async function saveConfig() {
    const config = {
      threshold: parseFloat(document.getElementById('threshold').value),
      nearzone:  parseFloat(document.getElementById('nearzone').value),
      maxcant:   parseFloat(document.getElementById('maxcant').value),
      hysteresis:parseFloat(document.getElementById('hysteresis').value),
      ema:       parseFloat(document.getElementById('ema').value)
    };
    Object.keys(config).forEach(k => localStorage.setItem(k, config[k]));
    try {
      await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config)
      });
      const btn = document.getElementById('saveButton');
      btn.textContent = '[ COMMITTED ]';
      setTimeout(() => { btn.textContent = '[ COMMIT PARAMS ]'; }, 1500);
    } catch (e) { alert('TRANSMIT FAILURE'); }
  }

  async function triggerCalibrate() {
    const btn = document.getElementById('calibrateButton');
    btn.disabled = true;
    btn.textContent = '[ CALIBRATING... ]';
    try { await fetch('/api/calibrate', { method: 'POST' }); } catch (e) {}
    setTimeout(() => { btn.disabled = false; btn.textContent = '[ CALIBRATE ]'; }, 3000);
  }
</script>
</body>
</html>
)HTML";

void setupWiFi() {
  Serial.println("Setting up WiFi AP...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setHostname("NIRLABS_SCOPE");

  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure AP IP.");
  }

  if (!WiFi.softAP(SSID, PASSWORD, 1, false, 4)) {
    Serial.println("WiFi.softAP failed!");
  } else {
    Serial.println("WiFi.softAP started successfully");
  }

  delay(1000);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("AP SSID: ");
  Serial.println(SSID);
  Serial.print("AP channel: ");
  Serial.println(WiFi.channel());
  Serial.print("AP stations: ");
  Serial.println(WiFi.softAPgetStationNum());
}

void setupWebServer() {
  // Serve main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // API: Get status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<256> doc;
    doc["roll"] = currentRoll;
    doc["rawRoll"] = currentRoll + calibrationOffset;
    doc["zone"] = currentZone;
    doc["zoneName"] = (currentZone == 0) ? "LEVEL" : (currentZone == 1) ? "NEAR" : "CANT";
    doc["battery"] = currentBattVoltage;
    doc["bootCount"] = bootCount;
    doc["calibrated"] = rtcCalibrated;
    doc["calOffset"] = calibrationOffset;
    doc["calibrating"] = calibrating;
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // API: Set config
  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, data, len);
      
      if (doc.containsKey("threshold")) THRESHOLD = doc["threshold"];
      if (doc.containsKey("nearzone")) NEAR_ZONE = doc["nearzone"];
      if (doc.containsKey("maxcant")) MAX_CANT = doc["maxcant"];
      if (doc.containsKey("hysteresis")) HYSTERESIS = doc["hysteresis"];
      if (doc.containsKey("ema")) EMA_ALPHA = doc["ema"];
      
      saveThresholdsToNVS();
      
      StaticJsonDocument<128> response;
      response["message"] = "Configuration updated and saved";
      
      String responseStr;
      serializeJson(response, responseStr);
      request->send(200, "application/json", responseStr);
    });

  // API: Calibrate
  server.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest *request) {
    calibrate();
    
    StaticJsonDocument<128> response;
    response["message"] = "Calibration complete";
    response["offset"] = calibrationOffset;
    
    String responseStr;
    serializeJson(response, responseStr);
    request->send(200, "application/json", responseStr);
  });

  server.begin();
  Serial.println("Web server started");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ++bootCount;
  Serial.println("\n\nNIRINIUM|LABS - Scope Level WebUI");
  Serial.printf("Boot #%d\n", bootCount);

  // Initialize SPIFFS and NVS
  SPIFFS.begin();
  nvs_flash_init();
  loadThresholdsFromNVS();

  // LED setup
  ledcSetup(PWM_CH_GREEN, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_YL, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_RL, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_YR, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_RR, PWM_FREQ, PWM_RES);

  ledcAttachPin(GREEN_LED, PWM_CH_GREEN);
  ledcAttachPin(YELLOW_LEFT, PWM_CH_YL);
  ledcAttachPin(RED_LEFT, PWM_CH_RL);
  ledcAttachPin(YELLOW_RIGHT, PWM_CH_YR);
  ledcAttachPin(RED_RIGHT, PWM_CH_RR);

  // Sensor setup
  analogReadResolution(12);
  pinMode(BATT_PIN, INPUT);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!accel.begin()) {
    Serial.println("ADXL345 not found!");
    while (1) {
      allLedsOff();
      ledPWM(PWM_CH_YL, 220);
      ledPWM(PWM_CH_YR, 220);
      delay(500);
      allLedsOff();
      delay(500);
    }
  }

  accel.setRange(ADXL345_RANGE_2_G);
  
  // Calibration on boot if not previously calibrated
  if (!rtcCalibrated) {
    calibrate();
  } else {
    calibrationOffset = rtcCalibrationOffset;
    Serial.printf("Restored cal offset: %.2f\n", calibrationOffset);
  }

  // WiFi & Web Server
  setupWiFi();
  setupWebServer();

  Serial.println("Scope Level WebUI ready!");
  lastBattCheck = millis();
}

void loop() {
  unsigned long now = millis();

  // Battery check
  if (now - lastBattCheck >= 10000UL) {
    lastBattCheck = now;
    currentBattVoltage = readBatteryVoltage();
    batteryLow = (currentBattVoltage < 3.3f && currentBattVoltage > 0.5f);
  }

  // Read accelerometer
  if (!calibrating) {
    sensors_event_t event;
    accel.getEvent(&event);

    float rawRoll = atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
    rawRoll -= calibrationOffset;

    if (firstReading) {
      filteredRoll = rawRoll;
      firstReading = false;
    } else {
      filteredRoll = EMA_ALPHA * rawRoll + (1.0f - EMA_ALPHA) * filteredRoll;
    }

    currentRoll = filteredRoll;
    float absRoll = fabs(currentRoll);

    // Update zone
    currentZone = getZoneWithHysteresis(absRoll, currentZone);

    // LED feedback
    unsigned long elapsed = now - cycleStart;
    if (currentZone == 0) {
      uint8_t greenBright = batteryLow ? 60 : 200;
      ledPWM(PWM_CH_GREEN, greenBright);
      allLedsOff();
      ledPWM(PWM_CH_GREEN, greenBright);
      cycleStart = now;
    } else if (currentZone == 1) {
      float t = (absRoll - THRESHOLD) / (NEAR_ZONE - THRESHOLD);
      t = constrain(t, 0.0f, 1.0f);
      currentCycle = 1500 - (int)(t * t * 1000 * 0.3f);
      if (elapsed >= (unsigned long)currentCycle) cycleStart = now;
      
      uint8_t yellowBright = elapsed < 80UL ? (uint8_t)(120 + 80 * t) : 0;
      ledPWM(PWM_CH_YL, (currentRoll < 0) ? yellowBright : 0);
      ledPWM(PWM_CH_YR, (currentRoll > 0) ? yellowBright : 0);
      ledPWM(PWM_CH_RL, 0);
      ledPWM(PWM_CH_RR, 0);
    } else {
      float t = constrain((absRoll - NEAR_ZONE) / (MAX_CANT - NEAR_ZONE), 0.0f, 1.0f);
      currentCycle = 1500 - (int)(t * t * 1000);
      if (elapsed >= (unsigned long)currentCycle) cycleStart = now;
      
      bool pulse1 = elapsed < 80UL;
      bool pulse2 = elapsed >= 180UL && elapsed < 260UL;
      bool redOn = pulse1 || pulse2;
      
      uint8_t redBright = redOn ? (uint8_t)(150 + 105 * t) : 0;
      uint8_t yellowBright = (uint8_t)(25 + 80 * t);
      ledPWM(PWM_CH_YL, (currentRoll < 0) ? yellowBright : 0);
      ledPWM(PWM_CH_RL, (currentRoll < 0) ? redBright : 0);
      ledPWM(PWM_CH_YR, (currentRoll > 0) ? yellowBright : 0);
      ledPWM(PWM_CH_RR, (currentRoll > 0) ? redBright : 0);
    }

    if (batteryLow && (now % 5000UL) < 150UL) {
      ledPWM(PWM_CH_YL, 90);
      ledPWM(PWM_CH_RL, 90);
      ledPWM(PWM_CH_YR, 90);
      ledPWM(PWM_CH_RR, 90);
    }
  }

  delay(25);
}
