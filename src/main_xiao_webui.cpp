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
  <title>Scope Level Controller</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
      color: #fff;
      padding: 20px;
      min-height: 100vh;
    }
    .container {
      max-width: 900px;
      margin: 0 auto;
      background: rgba(0, 0, 0, 0.3);
      border-radius: 15px;
      padding: 30px;
      backdrop-filter: blur(10px);
      box-shadow: 0 8px 32px rgba(31, 38, 135, 0.37);
    }
    h1 {
      text-align: center;
      margin-bottom: 30px;
      font-size: 2.5em;
      text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
    }
    .dashboard {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 20px;
      margin-bottom: 30px;
    }
    .card {
      background: rgba(255, 255, 255, 0.1);
      border-radius: 10px;
      padding: 20px;
      border: 1px solid rgba(255, 255, 255, 0.2);
    }
    .gauge-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 15px;
    }
    .gauge {
      width: 200px;
      height: 200px;
      border-radius: 50%;
      background: conic-gradient(
        from 180deg,
        #ff4444 0deg 90deg,
        #ffaa00 90deg 180deg,
        #44ff44 180deg 270deg,
        #ffaa00 270deg 360deg,
        #ff4444 360deg
      );
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.8em;
      font-weight: bold;
      position: relative;
      box-shadow: 0 0 20px rgba(0, 0, 0, 0.5);
    }
    .gauge::after {
      content: '';
      position: absolute;
      width: 10px;
      height: 10px;
      background: #fff;
      border-radius: 50%;
      z-index: 10;
    }
    .zone-indicator {
      font-size: 1.3em;
      font-weight: bold;
      text-align: center;
    }
    .status-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      font-size: 0.95em;
    }
    .status-row {
      display: flex;
      justify-content: space-between;
      padding: 8px;
      background: rgba(255, 255, 255, 0.05);
      border-radius: 5px;
    }
    .controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-bottom: 20px;
    }
    .control-group {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    label {
      font-size: 0.9em;
      opacity: 0.8;
      font-weight: 600;
    }
    input[type="range"], input[type="number"] {
      padding: 8px;
      border-radius: 5px;
      border: none;
      background: rgba(255, 255, 255, 0.15);
      color: #fff;
      font-size: 0.9em;
    }
    input[type="range"] {
      cursor: pointer;
      height: 6px;
      -webkit-appearance: none;
      width: 100%;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: #4CAF50;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(76, 175, 80, 0.5);
    }
    input[type="range"]::-moz-range-thumb {
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: #4CAF50;
      cursor: pointer;
      border: none;
      box-shadow: 0 0 10px rgba(76, 175, 80, 0.5);
    }
    button {
      padding: 12px 20px;
      border: none;
      border-radius: 8px;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: #fff;
      font-weight: bold;
      cursor: pointer;
      font-size: 1em;
      transition: transform 0.2s, box-shadow 0.2s;
      box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
    }
    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(102, 126, 234, 0.4);
    }
    button:active {
      transform: translateY(0);
    }
    button:disabled {
      opacity: 0.5;
      cursor: not-allowed;
    }
    .full-width {
      grid-column: 1 / -1;
    }
    .battery-indicator {
      display: inline-block;
      width: 40px;
      height: 20px;
      border: 2px solid #fff;
      border-radius: 3px;
      background: linear-gradient(90deg, #4CAF50 0%, #4CAF50 var(--battery-pct), #333 var(--battery-pct), #333 100%);
      position: relative;
    }
    .battery-indicator::after {
      content: '';
      position: absolute;
      right: -6px;
      top: 50%;
      transform: translateY(-50%);
      width: 4px;
      height: 8px;
      background: #fff;
      border-radius: 1px;
    }
    .info-section {
      margin-top: 20px;
      font-size: 0.85em;
      opacity: 0.8;
      text-align: center;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🎯 Scope Level Controller</h1>
    
    <div class="dashboard">
      <div class="card">
        <div class="gauge-container">
          <div class="gauge" id="gauge">
            <span id="rollValue">0.0°</span>
          </div>
          <div class="zone-indicator">
            Zone: <span id="zoneDisplay">LEVEL</span>
          </div>
        </div>
      </div>
      
      <div class="card">
        <div class="status-grid">
          <div class="status-row">
            <span>Roll (raw):</span>
            <span id="rollRaw">0.0°</span>
          </div>
          <div class="status-row">
            <span>Battery:</span>
            <span id="batteryVoltage">0.0V</span>
          </div>
          <div class="status-row">
            <span>Boot Count:</span>
            <span id="bootCount">0</span>
          </div>
          <div class="status-row">
            <span>Calibrated:</span>
            <span id="calibStatus">No</span>
          </div>
          <div class="status-row">
            <span>Cal Offset:</span>
            <span id="calOffset">0.0°</span>
          </div>
          <div class="status-row">
            <span>Status:</span>
            <span id="status">Connecting...</span>
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2 style="margin-bottom: 15px; font-size: 1.2em;">⚙️ Configuration</h2>
      <div class="controls">
        <div class="control-group">
          <label>Threshold: <span id="thresholdVal">0.5</span>°</label>
          <input type="range" id="threshold" min="0.1" max="1.0" step="0.1" value="0.5">
        </div>
        <div class="control-group">
          <label>Near Zone: <span id="nearzoneVal">1.5</span>°</label>
          <input type="range" id="nearzone" min="0.5" max="3.0" step="0.1" value="1.5">
        </div>
        <div class="control-group">
          <label>Max Cant: <span id="maxcantVal">5.0</span>°</label>
          <input type="range" id="maxcant" min="2.0" max="10.0" step="0.5" value="5.0">
        </div>
        <div class="control-group">
          <label>Hysteresis: <span id="hysteresisVal">0.15</span>°</label>
          <input type="range" id="hysteresis" min="0.05" max="0.5" step="0.05" value="0.15">
        </div>
        <div class="control-group">
          <label>EMA Alpha: <span id="emaVal">0.4</span></label>
          <input type="range" id="ema" min="0.1" max="0.9" step="0.1" value="0.4">
        </div>
        <div class="control-group">
          <button id="saveButton" onclick="saveConfig()">💾 Save Settings</button>
        </div>
        <div class="control-group full-width">
          <button id="calibrateButton" onclick="triggerCalibrate()" style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);">
            📊 Calibrate Now
          </button>
        </div>
      </div>
    </div>

    <div class="info-section">
      <p>NIRINIUM|LABS Anti-Cant Level Control Panel</p>
      <p>Auto-updates every 500ms | WiFi: NIRINIUM_LEVEL</p>
    </div>
  </div>

  <script>
    let lastUpdate = 0;
    let updateInterval = 500; // ms

    // Load stored values on page load
    window.addEventListener('load', () => {
      loadStoredValues();
      updateDashboard();
      setInterval(updateDashboard, updateInterval);
    });

    function loadStoredValues() {
      const threshold = localStorage.getItem('threshold') || '0.5';
      const nearzone = localStorage.getItem('nearzone') || '1.5';
      const maxcant = localStorage.getItem('maxcant') || '5.0';
      const hysteresis = localStorage.getItem('hysteresis') || '0.15';
      const ema = localStorage.getItem('ema') || '0.4';
      
      document.getElementById('threshold').value = threshold;
      document.getElementById('nearzone').value = nearzone;
      document.getElementById('maxcant').value = maxcant;
      document.getElementById('hysteresis').value = hysteresis;
      document.getElementById('ema').value = ema;
      
      updateLabels();
    }

    function updateLabels() {
      document.getElementById('thresholdVal').textContent = document.getElementById('threshold').value;
      document.getElementById('nearzoneVal').textContent = document.getElementById('nearzone').value;
      document.getElementById('maxcantVal').textContent = document.getElementById('maxcant').value;
      document.getElementById('hysteresisVal').textContent = document.getElementById('hysteresis').value;
      document.getElementById('emaVal').textContent = document.getElementById('ema').value;
    }

    document.getElementById('threshold').oninput = updateLabels;
    document.getElementById('nearzone').oninput = updateLabels;
    document.getElementById('maxcant').oninput = updateLabels;
    document.getElementById('hysteresis').oninput = updateLabels;
    document.getElementById('ema').oninput = updateLabels;

    async function updateDashboard() {
      try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        // Update gauge and values
        document.getElementById('rollValue').textContent = data.roll.toFixed(1) + '°';
        document.getElementById('rollRaw').textContent = data.rawRoll.toFixed(1) + '°';
        document.getElementById('zoneDisplay').textContent = data.zoneName;
        document.getElementById('bootCount').textContent = data.bootCount;
        document.getElementById('calibStatus').textContent = data.calibrated ? 'Yes' : 'No';
        document.getElementById('calOffset').textContent = data.calOffset.toFixed(2) + '°';
        document.getElementById('batteryVoltage').textContent = data.battery.toFixed(2) + 'V';
        document.getElementById('status').textContent = data.calibrating ? '⏳ Calibrating...' : '✓ Ready';
        
        // Update gauge needle rotation
        const rollAngle = Math.max(-30, Math.min(30, data.roll * 6));
        document.getElementById('gauge').style.setProperty('--needle-angle', rollAngle + 'deg');
        
        // Update battery color
        const batteryPct = Math.max(0, Math.min(100, (data.battery / 4.2) * 100));
        const batteryColor = batteryPct > 50 ? '#4CAF50' : batteryPct > 20 ? '#ffaa00' : '#ff4444';
        const gauge = document.getElementById('gauge');
        gauge.style.color = data.zone === 0 ? '#44ff44' : data.zone === 1 ? '#ffaa00' : '#ff4444';
      } catch (e) {
        document.getElementById('status').textContent = '⚠ No connection';
        console.error('Update failed:', e);
      }
    }

    async function saveConfig() {
      const config = {
        threshold: parseFloat(document.getElementById('threshold').value),
        nearzone: parseFloat(document.getElementById('nearzone').value),
        maxcant: parseFloat(document.getElementById('maxcant').value),
        hysteresis: parseFloat(document.getElementById('hysteresis').value),
        ema: parseFloat(document.getElementById('ema').value)
      };
      
      localStorage.setItem('threshold', config.threshold);
      localStorage.setItem('nearzone', config.nearzone);
      localStorage.setItem('maxcant', config.maxcant);
      localStorage.setItem('hysteresis', config.hysteresis);
      localStorage.setItem('ema', config.ema);
      
      try {
        const response = await fetch('/api/config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(config)
        });
        const result = await response.json();
        alert(result.message || 'Settings saved!');
      } catch (e) {
        alert('Failed to save settings');
      }
    }

    async function triggerCalibrate() {
      const btn = document.getElementById('calibrateButton');
      btn.disabled = true;
      try {
        const response = await fetch('/api/calibrate', { method: 'POST' });
        const result = await response.json();
        alert(result.message || 'Calibration started');
      } catch (e) {
        alert('Failed to start calibration');
      }
      setTimeout(() => { btn.disabled = false; }, 2000);
    }
  </script>
</body>
</html>
)HTML";

void setupWiFi() {
  Serial.println("Setting up WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
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
