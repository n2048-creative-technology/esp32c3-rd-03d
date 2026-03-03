#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"

// Uncomment to enable serial logs.
#define USE_SERIAL_LOGS

#include "RadarSensor.h"

#define RADAR_RX_PIN 4
#define RADAR_TX_PIN 5

#define MIN_DISTANCE_MM 0.0f
#define MAX_DISTANCE_MM 2000.0f
#define LED_HOLD_MS 10000UL

unsigned long last_detected_ms = 0;
unsigned long last_no_target_log_ms = 0;

RadarSensor radar(RADAR_RX_PIN, RADAR_TX_PIN); // ESP RX/TX pins connected to sensor TX/RX

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

}

float distance = 0.0f;

void loop() {

  unsigned long m = millis();
  if (radar.update()) {
    RadarTarget tgt = radar.getTarget();
    distance = tgt.distance;

#ifdef USE_SERIAL_LOGS
    Serial.print("X (mm): "); Serial.println(tgt.x);
    Serial.print("Y (mm): "); Serial.println(tgt.y);
    Serial.print("Distance (mm): "); Serial.println(tgt.distance);
    Serial.print("Angle (degrees): "); Serial.println(tgt.angle);
    Serial.print("Speed (cm/s): "); Serial.println(tgt.speed);
    Serial.println("---");
#endif

    bool in_range = (distance > MIN_DISTANCE_MM && distance < MAX_DISTANCE_MM);
    if (in_range) {
      last_detected_ms = m;
    }

  }
  else{
    if (m - last_no_target_log_ms >= 1000) {
      Serial.println("No target detected");
      last_no_target_log_ms = m;
    }
  }
  // Small cooperative delay to keep system responsive
  delay(1);
}
