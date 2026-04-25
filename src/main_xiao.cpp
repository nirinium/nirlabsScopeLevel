#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <esp_sleep.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// === RTC-RETAINED STATE (survives deep sleep) ===
RTC_DATA_ATTR static int     bootCount           = 0;
RTC_DATA_ATTR static float   rtcCalibrationOffset = 0.0f;
RTC_DATA_ATTR static bool    rtcCalibrated        = false;

// === PIN ASSIGNMENTS (Seeed Studio XIAO ESP32C3) ===
const int GREEN_LED    = 21; // D6
const int YELLOW_LEFT  = 20; // D7
const int RED_LEFT     = 3;  // D1
const int YELLOW_RIGHT = 4;  // D2
const int RED_RIGHT    = 5;  // D3
const int BATT_PIN     = 2;  // A0/D0
const int POWER_SW_PIN = 10; // D10, optional SPST to GND for standby

// === I2C PINS (XIAO ESP32C3) ===
const int I2C_SDA = 6; // D4
const int I2C_SCL = 7; // D5

// === PWM CHANNELS (ESP32 LEDC) ===
const int PWM_CH_GREEN = 0;
const int PWM_CH_YL    = 1;
const int PWM_CH_RL    = 2;
const int PWM_CH_YR    = 3;
const int PWM_CH_RR    = 4;
const int PWM_FREQ     = 5000; // 5kHz
const int PWM_RES      = 8;    // 8-bit (0-255)

// === ZONE THRESHOLDS ===
const float THRESHOLD  = 0.5;  // degrees: dead-level zone
const float NEAR_ZONE  = 1.5;  // degrees: almost-level transition
const float MAX_CANT   = 5.0;  // degrees: maximum urgency
const float HYSTERESIS = 0.15; // degrees: prevents boundary flicker

// === HEARTBEAT TIMING ===
const int FLASH_ON  = 80;   // ms
const int FLASH_GAP = 100;  // ms
const int CYCLE_MIN = 500;  // ms
const int CYCLE_MAX = 1500; // ms

// === FILTER ===
const float EMA_ALPHA = 0.4; // lower = smoother

// === BATTERY MONITORING ===
const float BATT_LOW = 3.3;                         // volts
const float BATT_DIVIDER = 2.0;                     // 100k/100k divider
const unsigned long BATT_CHECK_INTERVAL = 10000UL;  // ms

// === POWER MANAGEMENT ===
const unsigned long SLEEP_TIMEOUT  = 60000UL;  // ms of level-idle before deep sleep
const int ACCEL_INT_PIN            = 9;        // D9 / GPIO9 — wire ADXL345 INT1 here
const bool ENABLE_DEEP_SLEEP       = false;    // set falserely on physical battery switch for power-off

// === ADXL345 ACTIVITY INTERRUPT (shake-awake) ===
// Each LSB of THRESH_ACT = 62.5 mg. 4 = 250 mg — adjust for sensitivity.
const uint8_t ACCEL_ACTIVITY_THRESHOLD = 4;
static constexpr uint8_t ADXL_ADDR_CUSTOM = 0x53;
static constexpr uint8_t ADXL_REG_THRESH_ACT_CUSTOM = 0x24;
static constexpr uint8_t ADXL_REG_ACT_INACT_CTL_CUSTOM = 0x27;
static constexpr uint8_t ADXL_REG_INT_ENABLE_CUSTOM = 0x2E;
static constexpr uint8_t ADXL_REG_INT_MAP_CUSTOM = 0x2F;
static constexpr uint8_t ADXL_REG_INT_SOURCE_CUSTOM = 0x30;

// === DEBUG ===
const bool DEBUG_MODE = true;
const bool USE_SOFT_POWER_SWITCH = false;

// === STATE ===
unsigned long cycleStart = 0;
int currentCycle = CYCLE_MAX;
float filteredRoll = 0.0;
float calibrationOffset = 0.0;
int currentZone = 0; // 0=level, 1=near, 2=cant
unsigned long lastActivity = 0;
unsigned long lastBattCheck = 0;
bool batteryLow = false;
bool firstReading = true;
unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 200UL;

void ledPWM(int channel, uint8_t brightness);
void allLedsOff();

// --- ADXL345 low-level register access (bypasses library to reach INT regs) ---
static void adxlWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL_ADDR_CUSTOM);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t adxlReadReg(uint8_t reg) {
  Wire.beginTransmission(ADXL_ADDR_CUSTOM);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  const uint8_t requested = Wire.requestFrom(static_cast<uint8_t>(ADXL_ADDR_CUSTOM), static_cast<uint8_t>(1));
  if (requested != 1 || !Wire.available()) {
    return 0;
  }
  return Wire.read();
}

// Arm ADXL345 activity interrupt so INT1 goes HIGH on shake.
// Call this just before entering deep sleep.
void setupAccelActivityInterrupt() {
  adxlWriteReg(ADXL_REG_THRESH_ACT_CUSTOM,    ACCEL_ACTIVITY_THRESHOLD);
  adxlWriteReg(ADXL_REG_ACT_INACT_CTL_CUSTOM, 0xFF); // AC-coupled, all axes
  adxlWriteReg(ADXL_REG_INT_MAP_CUSTOM,       0x00); // all interrupts -> INT1
  adxlWriteReg(ADXL_REG_INT_ENABLE_CUSTOM,    0x10); // Activity interrupt enable
  adxlReadReg(ADXL_REG_INT_SOURCE_CUSTOM);           // clear any stale interrupt
}

bool handleSoftPowerSwitch() {
  if (!USE_SOFT_POWER_SWITCH) return false;

  // Switch wiring: D10 to GND when OFF, internal pull-up keeps it HIGH when ON.
  if (digitalRead(POWER_SW_PIN) == LOW) {
    allLedsOff();
    if (DEBUG_MODE) {
      Serial.println("Power switch OFF -> standby sleep");
      Serial.flush();
    }
    esp_sleep_enable_timer_wakeup(2000000UL);
    esp_light_sleep_start();
    firstReading = true;
    return true;
  }
  return false;
}

float readBatteryVoltage() {
  int raw = analogRead(BATT_PIN);
  return (raw / 4095.0f) * 3.3f * BATT_DIVIDER;
}

void calibrate() {
  Serial.println("Calibrating... hold level!");
  float sum = 0.0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
    
    // Flash all LEDs during calibration to show progress
    bool ledOn = (i % 10) < 5; // alternate every ~100ms
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
  Serial.printf("Calibration offset: %.2f\n", calibrationOffset);
}

int getZoneWithHysteresis(float absRoll, int prevZone) {
  switch (prevZone) {
    case 0: // LEVEL
      if (absRoll >= THRESHOLD + HYSTERESIS) {
        return (absRoll >= NEAR_ZONE + HYSTERESIS) ? 2 : 1;
      }
      return 0;
    case 1: // NEAR
      if (absRoll < THRESHOLD - HYSTERESIS) return 0;
      if (absRoll >= NEAR_ZONE + HYSTERESIS) return 2;
      return 1;
    case 2: // CANT
      if (absRoll < NEAR_ZONE - HYSTERESIS) {
        return (absRoll < THRESHOLD - HYSTERESIS) ? 0 : 1;
      }
      return 2;
  }
  return 0;
}

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

void enterDeepSleep() {
  allLedsOff();
  setupAccelActivityInterrupt(); // arm ADXL shake-wake before cutting power
  if (DEBUG_MODE) {
    Serial.println("Entering deep sleep. Move scope to wake (ADXL INT1 on D8).");
    Serial.flush();
  }
  // INT1 is active-HIGH; wake when it goes high on movement
  esp_deep_sleep_enable_gpio_wakeup(BIT(ACCEL_INT_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ++bootCount;
  Serial.println("NIRINIUM|LABS");
  Serial.println("Anti-Cant Level XIAO ESP32C3");
  Serial.printf("Boot #%d\n", bootCount);

  if (ENABLE_DEEP_SLEEP) {
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    if (wakeReason == ESP_SLEEP_WAKEUP_GPIO) {
      Serial.println("Woke from deep sleep via movement (ADXL shake-awake).");
    } else {
      Serial.println("Cold boot.");
    }
  } else {
    Serial.println("Deep sleep disabled. Use battery switch for power-off.");
  }

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

  analogReadResolution(12);
  pinMode(BATT_PIN, INPUT);
  pinMode(POWER_SW_PIN, INPUT_PULLUP);
  if (ENABLE_DEEP_SLEEP) {
    pinMode(ACCEL_INT_PIN, INPUT); // ADXL INT1 input (no pull — driven by ADXL)
  }

  Wire.begin(I2C_SDA, I2C_SCL);

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
      ledPWM(PWM_CH_YL, 220);
      ledPWM(PWM_CH_RL, 220);
      ledPWM(PWM_CH_YR, 220);
      ledPWM(PWM_CH_RR, 220);
      delay(200);
      allLedsOff();
      delay(200);
    }
  }

  accel.setRange(ADXL345_RANGE_2_G);
  if (ENABLE_DEEP_SLEEP) {
    // Clear any pending ADXL interrupt so INT1 goes low before we check the pin again
    adxlReadReg(ADXL_REG_INT_SOURCE_CUSTOM);

    esp_sleep_wakeup_cause_t wakeReasonForCal = esp_sleep_get_wakeup_cause();
    if (wakeReasonForCal == ESP_SLEEP_WAKEUP_GPIO && rtcCalibrated) {
      calibrationOffset = rtcCalibrationOffset;
      Serial.printf("Restored calibration offset: %.2f (skipping recal)\n", calibrationOffset);
    } else {
      calibrate();
    }
  } else {
    calibrate();
  }

  Serial.println("Anti-Cant Level ready!");
  lastActivity = millis();
}

void loop() {
  unsigned long now = millis();

  if (handleSoftPowerSwitch()) {
    return;
  }

  if (now - lastBattCheck >= BATT_CHECK_INTERVAL) {
    lastBattCheck = now;
    float voltage = readBatteryVoltage();
    batteryLow = (voltage < BATT_LOW && voltage > 0.5f);
    if (DEBUG_MODE && batteryLow) {
      Serial.printf("LOW BATTERY: %.2fV\n", voltage);
    }
  }

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

  float roll = filteredRoll;
  float absRoll = fabs(roll);

  currentZone = getZoneWithHysteresis(absRoll, currentZone);

  if (currentZone != 0) {
    lastActivity = now;
  } else if (ENABLE_DEEP_SLEEP && (now - lastActivity >= SLEEP_TIMEOUT)) {
    enterDeepSleep(); // never returns — setup() runs on wake
  }

  unsigned long elapsed = now - cycleStart;

  if (currentZone == 0) {
    uint8_t greenBright = batteryLow ? 60 : 200;
    ledPWM(PWM_CH_GREEN, greenBright);
    ledPWM(PWM_CH_YL, 0);
    ledPWM(PWM_CH_RL, 0);
    ledPWM(PWM_CH_YR, 0);
    ledPWM(PWM_CH_RR, 0);
    cycleStart = now;

  } else if (currentZone == 1) {
    float t = (absRoll - THRESHOLD) / (NEAR_ZONE - THRESHOLD);
    t = constrain(t, 0.0f, 1.0f);
    currentCycle = CYCLE_MAX - (int)(t * t * (CYCLE_MAX - CYCLE_MIN) * 0.3f);

    if (elapsed >= (unsigned long)currentCycle) cycleStart = now;

    float phase = (float)elapsed / (float)currentCycle;
    uint8_t greenBright = (uint8_t)(100.0f + 100.0f * cos(phase * 2.0f * PI));
    ledPWM(PWM_CH_GREEN, greenBright);

    bool yellowOn = elapsed < (unsigned long)FLASH_ON;
    uint8_t yellowBright = yellowOn ? (uint8_t)(120 + 80 * t) : 0;

    ledPWM(PWM_CH_YL, (roll < 0) ? yellowBright : 0);
    ledPWM(PWM_CH_YR, (roll > 0) ? yellowBright : 0);
    ledPWM(PWM_CH_RL, 0);
    ledPWM(PWM_CH_RR, 0);

  } else {
    float t = constrain((absRoll - NEAR_ZONE) / (MAX_CANT - NEAR_ZONE), 0.0f, 1.0f);
    currentCycle = CYCLE_MAX - (int)(t * t * (CYCLE_MAX - CYCLE_MIN));

    if (elapsed >= (unsigned long)currentCycle) cycleStart = now;

    ledPWM(PWM_CH_GREEN, 0);

    bool pulse1 = elapsed < (unsigned long)FLASH_ON;
    bool pulse2 = elapsed >= (unsigned long)(FLASH_ON + FLASH_GAP)
               && elapsed <  (unsigned long)(FLASH_ON + FLASH_GAP + FLASH_ON);
    bool redOn = pulse1 || pulse2;

    uint8_t redBright = redOn ? (uint8_t)(150 + 105 * t) : 0;
    uint8_t yellowBright = (uint8_t)(25 + 80 * t);

    ledPWM(PWM_CH_YL, (roll < 0) ? yellowBright : 0);
    ledPWM(PWM_CH_RL, (roll < 0) ? redBright : 0);
    ledPWM(PWM_CH_YR, (roll > 0) ? yellowBright : 0);
    ledPWM(PWM_CH_RR, (roll > 0) ? redBright : 0);
  }

  if (batteryLow && (now % 5000UL) < 150UL) {
    ledPWM(PWM_CH_YL, 90);
    ledPWM(PWM_CH_RL, 90);
    ledPWM(PWM_CH_YR, 90);
    ledPWM(PWM_CH_RR, 90);
  }

  if (DEBUG_MODE && (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL)) {
    const char* zoneName[] = {"LEVEL", "NEAR", "CANT"};
    Serial.printf("Roll: %.2f (raw: %.2f) [%s]\n", roll, rawRoll, zoneName[currentZone]);
    lastDebugPrint = now;
  }

  delay(25);
}
