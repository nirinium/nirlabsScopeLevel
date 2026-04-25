#include <Arduino.h>

// XIAO ESP32C3 LED test pins (same mapping as main_xiao.cpp).
const int RED_LEFT_PIN     = 3;   // D1
const int YELLOW_LEFT_PIN  = 20;  // D7
const int GREEN_PIN        = 21;  // D6
const int YELLOW_RIGHT_PIN = 4;   // D2
const int RED_RIGHT_PIN    = 5;   // D3

const int LED_PINS[] = {
  RED_LEFT_PIN,
  YELLOW_LEFT_PIN,
  GREEN_PIN,
  YELLOW_RIGHT_PIN,
  RED_RIGHT_PIN
};

const int LED_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

void allOff() {
  for (int i = 0; i < LED_COUNT; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
}

void allOn() {
  for (int i = 0; i < LED_COUNT; i++) {
    digitalWrite(LED_PINS[i], HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("LED pin diagnostic test starting...");

  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  allOff();
}

void loop() {
  // Step through each LED one-by-one and test both output states.
  for (int i = 0; i < LED_COUNT; i++) {
    allOff();
    Serial.printf("Pin %d HIGH (active-high test)\n", LED_PINS[i]);
    digitalWrite(LED_PINS[i], HIGH);
    delay(350);

    Serial.printf("Pin %d LOW (active-low test)\n", LED_PINS[i]);
    digitalWrite(LED_PINS[i], LOW);
    delay(350);
  }

  // Flash all LEDs together to verify rail/power behavior.
  Serial.println("All pins HIGH");
  allOn();
  delay(400);
  Serial.println("All pins LOW");
  allOff();
  delay(400);
}
