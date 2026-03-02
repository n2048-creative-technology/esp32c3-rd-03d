#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"

// Uncomment to enable serial logs.
//#define USE_SERIAL_LOGS

// Uncomment to enable the OLED display.
//#define USE_DISPLAY
#ifdef USE_DISPLAY
#include <U8g2lib.h>
#endif
#include "RadarSensor.h"

#ifdef USE_DISPLAY
// --- OLED: fixed on your board ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, /*SCL=*/6, /*SDA=*/5);
#endif

#define DISPLAY_SCL_PIN 6
#define DISPLAY_SDA_PIN 5
// Optional: define a GPIO that switches OLED power (e.g., a MOSFET or enable pin).
// #define DISPLAY_POWER_PIN 7

#ifndef LED_PIN
#define LED_PIN 2 // Default LED pin for ESP32-C3 is 8
#endif

#define MIN_DISTANCE_MM 500.0f
#define MAX_DISTANCE_MM 2000.0f
#define LED_HOLD_MS 10000UL

#ifdef USE_DISPLAY
int xOffset = 28;
int yOffset = 24;
unsigned long pm = 0;
#endif
unsigned long last_detected_ms = 0;

RadarSensor radar(GPIO_NUM_20,GPIO_NUM_21); // RX, TX pins on the ESP to sensor TX, RX pins

void setup() {
  
#ifdef USE_SERIAL_LOGS
  Serial.begin(115200);
#endif

  delay(10);
  // Disable radios to reduce power (no impact on radar readings).
  WiFi.mode(WIFI_OFF);
  btStop();
  radar.begin(256000);

#ifdef USE_SERIAL_LOGS
  Serial.println("Radar Sensor Started");
#endif

  delay(10);
#ifdef USE_DISPLAY
  // OLED init (as in your working sketch)
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setBusClock(400000);
  u8g2.setFont(u8g2_font_10x20_tr);
#else
  // Ensure OLED is not powered or back-powered when display is disabled.
#ifdef DISPLAY_POWER_PIN
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, LOW);
#endif
  pinMode(DISPLAY_SCL_PIN, INPUT);
  pinMode(DISPLAY_SDA_PIN, INPUT);
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

float distance, p_distance = 0.0f;
bool led_state = LOW;

void loop() {

  unsigned long m = millis();
  if (radar.update()) {
    RadarTarget tgt = radar.getTarget();
    distance = tgt.distance;

#ifdef USE_SERIAL_LOGS
    // Serial.print("X (mm): "); Serial.println(tgt.x);
    // Serial.print("Y (mm): "); Serial.println(tgt.y);
    Serial.print("Distance (mm): "); Serial.println(tgt.distance);
    // Serial.print("Angle (degrees): "); Serial.println(tgt.angle);
    // Serial.print("Speed (cm/s): "); Serial.println(tgt.speed);
#endif

    bool in_range = (distance > MIN_DISTANCE_MM && distance < MAX_DISTANCE_MM);
    if (in_range) {
      last_detected_ms = m;
    }

    // keep LED on while in range, and for 10s after last detection
    bool led_on = in_range || (m - last_detected_ms < LED_HOLD_MS);
    digitalWrite(LED_PIN, led_on ? HIGH : LOW);
    if (led_on != led_state) {
      led_state = led_on;
    }

#ifdef USE_SERIAL_LOGS
    Serial.print("LED state: ");
    Serial.println(led_state ? "HIGH" : "LOW");
    Serial.println("-------------------------");
#endif

#ifdef USE_DISPLAY
    if (m - pm >= 250 && p_distance != distance) {
      u8g2.clearBuffer();
      u8g2.setCursor(xOffset + 15, yOffset + 30);
      u8g2.printf("%.2f", distance/1000.0f); // convert mm to meters
      u8g2.sendBuffer();
      pm = m;
      p_distance = distance;
    }
#endif
  }

  // Small cooperative delay to keep system responsive
  delay(1);
}
