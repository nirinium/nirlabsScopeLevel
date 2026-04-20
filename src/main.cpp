#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <esp_sleep.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// === PIN ASSIGNMENTS ===
const int GREEN_LED = 25;
const int RED_LEFT  = 26;
const int RED_RIGHT = 27;
const int BATT_PIN  = 34;   // ADC pin for battery voltage divider

// === PWM CHANNELS (ESP32 LEDC) ===
const int PWM_CH_GREEN = 0;
const int PWM_CH_LEFT  = 1;
const int PWM_CH_RIGHT = 2;
const int PWM_FREQ     = 5000;  // 5kHz — no visible flicker
const int PWM_RES      = 8;     // 8-bit (0-255)

// === ZONE THRESHOLDS ===
const float THRESHOLD     = 0.5;  // degrees — dead-level zone
const float NEAR_ZONE     = 1.5;  // degrees — "almost level" transition
const float MAX_CANT      = 5.0;  // degrees — maximum urgency
const float HYSTERESIS    = 0.15; // degrees — prevents zone boundary flicker

// === HEARTBEAT TIMING ===
const int FLASH_ON    = 80;    // ms — short sharp flash
const int FLASH_GAP   = 100;   // ms — gap between double-pulse beats
const int CYCLE_MIN   = 500;   // ms — fastest full cycle (high cant)
const int CYCLE_MAX   = 1500;  // ms — slowest full cycle (near level)

// === FILTER ===
const float EMA_ALPHA = 0.4;   // Exponential moving average weight (0.0-1.0, lower=smoother)

// === BATTERY MONITORING ===
const float BATT_LOW      = 3.3;  // volts — low battery warning
const float BATT_DIVIDER  = 2.0;  // voltage divider ratio (adjust to your circuit)
const unsigned long BATT_CHECK_INTERVAL = 10000; // check every 10s

// === POWER MANAGEMENT ===
const unsigned long SLEEP_TIMEOUT = 60000;  // ms idle before light sleep (1 min level = sleep)
const unsigned long SLEEP_DURATION = 500000; // µs light sleep (500ms between wake checks)

// === DEBUG (set to false for production) ===
const bool DEBUG_MODE = true;

// === STATE ===
unsigned long cycleStart = 0;
int currentCycle = CYCLE_MAX;
float filteredRoll = 0.0;
float calibrationOffset = 0.0;
int currentZone = 0;  // 0=level, 1=near, 2=cant
unsigned long lastActivity = 0;
unsigned long lastBattCheck = 0;
bool batteryLow = false;
bool firstReading = true;

// === FUNCTIONS ===

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  float voltage = (raw / 4095.0) * 3.3 * BATT_DIVIDER;
  return voltage;
}

void calibrate() {
  Serial.println("Calibrating... hold level!");
  float sum = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += atan2(event.acceleration.y, event.acceleration.z) * 180.0 / PI;
    delay(20);
  }
  calibrationOffset = sum / samples;
  Serial.printf("Calibration offset: %.2f°\n", calibrationOffset);
}

int getZoneWithHysteresis(float absRoll, int prevZone) {
  // Enter zone at threshold, exit with hysteresis buffer
  switch (prevZone) {
    case 0: // currently LEVEL
      if (absRoll >= THRESHOLD + HYSTERESIS) return (absRoll >= NEAR_ZONE + HYSTERESIS) ? 2 : 1;
      return 0;
    case 1: // currently NEAR
      if (absRoll < THRESHOLD - HYSTERESIS) return 0;
      if (absRoll >= NEAR_ZONE + HYSTERESIS) return 2;
      return 1;
    case 2: // currently CANT
      if (absRoll < NEAR_ZONE - HYSTERESIS) return (absRoll < THRESHOLD - HYSTERESIS) ? 0 : 1;
      return 2;
  }
  return 0;
}

void ledPWM(int channel, uint8_t brightness) {
  ledcWrite(channel, brightness);
}

void allLedsOff() {
  ledPWM(PWM_CH_GREEN, 0);
  ledPWM(PWM_CH_LEFT, 0);
  ledPWM(PWM_CH_RIGHT, 0);
}

void enterLightSleep() {
  allLedsOff();
  Serial.println("Entering light sleep...");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION);
  esp_light_sleep_start();
  // Wakes up here
  lastActivity = millis();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("NIRINIUM|LABS");
  Serial.println("Anti-Cant Level v2.0");
  Serial.println("Booting...");

  // Setup PWM channels instead of plain digital
  ledcSetup(PWM_CH_GREEN, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(GREEN_LED, PWM_CH_GREEN);
  ledcAttachPin(RED_LEFT, PWM_CH_LEFT);
  ledcAttachPin(RED_RIGHT, PWM_CH_RIGHT);

  // Battery ADC
  analogReadResolution(12);
  pinMode(BATT_PIN, INPUT);

  Wire.begin(21, 22);

  // I2C scan only in debug mode
  if (DEBUG_MODE) {
    Serial.println("Scanning I2C...");
    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  Found device at 0x%02X\n", addr);
        count++;
      }
    }
    Serial.printf("I2C scan done, %d device(s) found.\n", count);
  }

  if (!accel.begin()) {
    Serial.println("ADXL345 not found! Check wiring.");
    while (1) {
      // Flash all LEDs as error indicator
      ledPWM(PWM_CH_LEFT, 255);
      ledPWM(PWM_CH_RIGHT, 255);
      delay(200);
      allLedsOff();
      delay(200);
    }
  }
  accel.setRange(ADXL345_RANGE_2_G);

  // Calibrate on boot — assumes device is held level
  calibrate();

  Serial.println("Anti-Cant Level ready!");
  lastActivity = millis();
}

void loop() {
  unsigned long now = millis();

  // === BATTERY CHECK ===
  if (now - lastBattCheck >= BATT_CHECK_INTERVAL) {
    lastBattCheck = now;
    float voltage = readBatteryVoltage();
    batteryLow = (voltage < BATT_LOW && voltage > 0.5); // >0.5 to ignore if no divider connected
    if (DEBUG_MODE && batteryLow) {
      Serial.printf("LOW BATTERY: %.2fV\n", voltage);
    }
  }

  // === READ SENSOR ===
  sensors_event_t event;
  accel.getEvent(&event);

  float rawRoll = atan2(event.acceleration.y, event.acceleration.z) * 180.0 / PI;
  rawRoll -= calibrationOffset; // apply calibration

  // === EMA FILTER ===
  if (firstReading) {
    filteredRoll = rawRoll;
    firstReading = false;
  } else {
    filteredRoll = EMA_ALPHA * rawRoll + (1.0 - EMA_ALPHA) * filteredRoll;
  }

  float roll = filteredRoll;
  float absRoll = fabs(roll);

  // === ZONE DETECTION WITH HYSTERESIS ===
  currentZone = getZoneWithHysteresis(absRoll, currentZone);

  // === SLEEP CHECK: if level for too long, save power ===
  if (currentZone != 0) {
    lastActivity = now;
  } else if (now - lastActivity >= SLEEP_TIMEOUT) {
    enterLightSleep();
    firstReading = true; // re-init filter after wake
    return;
  }

  unsigned long elapsed = now - cycleStart;

  // === LED LOGIC ===
  if (currentZone == 0) {
    // LEVEL: solid green (PWM dimmed for battery life), reds off
    uint8_t greenBright = batteryLow ? 60 : 200; // dimmer if low batt
    ledPWM(PWM_CH_GREEN, greenBright);
    ledPWM(PWM_CH_LEFT, 0);
    ledPWM(PWM_CH_RIGHT, 0);
    cycleStart = now;

  } else if (currentZone == 1) {
    // NEAR LEVEL: green fades in/out, single red flash per cycle
    float t = (absRoll - THRESHOLD) / (NEAR_ZONE - THRESHOLD);
    currentCycle = CYCLE_MAX - (int)(t * t * (CYCLE_MAX - CYCLE_MIN) * 0.3);

    if (elapsed >= (unsigned long)currentCycle) cycleStart = now;

    // Green: smooth sine fade (PWM)
    float phase = (float)elapsed / (float)currentCycle; // 0.0 to 1.0
    uint8_t greenBright = (uint8_t)(128.0 + 127.0 * cos(phase * 2.0 * PI));
    ledPWM(PWM_CH_GREEN, greenBright);

    // Red: single short flash per cycle (full brightness)
    bool redOn = elapsed < (unsigned long)FLASH_ON;
    uint8_t redBright = redOn ? 255 : 0;
    ledPWM(PWM_CH_LEFT,  (roll < 0) ? redBright : 0);
    ledPWM(PWM_CH_RIGHT, (roll > 0) ? redBright : 0);

  } else {
    // CANT: heartbeat double-pulse, green off
    float t = constrain((absRoll - NEAR_ZONE) / (MAX_CANT - NEAR_ZONE), 0.0, 1.0);
    currentCycle = CYCLE_MAX - (int)(t * t * (CYCLE_MAX - CYCLE_MIN));

    if (elapsed >= (unsigned long)currentCycle) cycleStart = now;

    ledPWM(PWM_CH_GREEN, 0);

    // Double-pulse heartbeat pattern
    bool pulse1 = elapsed < (unsigned long)FLASH_ON;
    bool pulse2 = elapsed >= (unsigned long)(FLASH_ON + FLASH_GAP)
               && elapsed <  (unsigned long)(FLASH_ON + FLASH_GAP + FLASH_ON);
    bool redOn = pulse1 || pulse2;

    // Brightness scales with urgency
    uint8_t redBright = redOn ? (uint8_t)(150 + 105 * t) : 0; // 150-255
    ledPWM(PWM_CH_LEFT,  (roll < 0) ? redBright : 0);
    ledPWM(PWM_CH_RIGHT, (roll > 0) ? redBright : 0);
  }

  // === LOW BATTERY WARNING: brief red double-blink every 5s ===
  if (batteryLow && (now % 5000) < 150) {
    ledPWM(PWM_CH_LEFT, 100);
    ledPWM(PWM_CH_RIGHT, 100);
  }

  // === DEBUG OUTPUT ===
  if (DEBUG_MODE) {
    const char* zoneName[] = {"LEVEL", "NEAR", "CANT"};
    Serial.printf("Roll: %.2f° (raw: %.2f°) [%s]\n", roll, rawRoll, zoneName[currentZone]);
  }

  delay(25); // 40 Hz
}