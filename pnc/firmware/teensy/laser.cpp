#include "laser.h"

#include "config.h"
#include "util.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal state
// --------------------------------------------------------------
static bool ldOn = false;

// --------------------------------------------------------------
// Internal helpers
// --------------------------------------------------------------
//
// Long-duration averaging of the photodiode analog channel.
// This is a diagnostic tool, not a real-time signal.
//
static float averageImonSeconds(uint32_t seconds) {
  const uint32_t duration_ms = seconds * 1000UL;
  uint32_t start = millis();
  uint64_t sum = 0;
  uint32_t count = 0;

  // Enforce deterministic resolution
  analogReadResolution(12);

  while (millis() - start < duration_ms) {
    sum += (uint16_t)analogRead(PHOTODIODE_ANALOG_PIN);
    count++;
    delay(1); // ~1 kHz cadence
  }

  if (count == 0) return 0.0f;

  float adc_avg = (float)sum / (float)count;
  return (adc_avg / 4095.0f) * 3.3f;
}

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void laser_init() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(LD_ON_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(LD_ON_PIN, LOW);

  ldOn = false;
}

void laser_on() {
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(LD_ON_PIN, HIGH);
  ldOn = true;
}

void laser_off() {
  digitalWrite(LD_ON_PIN, LOW);
  ldOn = false;
}

bool laser_is_enabled() {
  return ldOn;
}

String laser_measure_voltages() {
  // (1) Laser OFF
  laser_off();
  delay(50);
  float off_v = averageImonSeconds(10);

  // (2) Laser ON
  laser_on();
  delay(50);
  float on_v = averageImonSeconds(10);

  // (3) Laser OFF again (leave EN untouched)
  laser_off();

  String b;
  appendFloatKV(b, "off_v", off_v, 5);
  b += ",";
  appendFloatKV(b, "on_v", on_v, 5);

  return b;
}
