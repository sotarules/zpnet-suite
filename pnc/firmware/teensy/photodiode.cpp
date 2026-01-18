#include "photodiode.h"

#include "config.h"

#include <Arduino.h>

// --------------------------------------------------------------
// ISR-driven edge visibility
// --------------------------------------------------------------
static volatile bool     photodiode_edge_seen = false;
static volatile bool     photodiode_episode_latched = false;
static volatile uint32_t photodiode_episode_count = 0;

// --------------------------------------------------------------
// ISR
// --------------------------------------------------------------
static void photodiodeISR() {
  photodiode_edge_seen = true;
}

// --------------------------------------------------------------
// Initialization
// --------------------------------------------------------------
void photodiode_init() {
  pinMode(PHOTODIODE_EDGE_PIN, INPUT);
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PHOTODIODE_EDGE_PIN),
    photodiodeISR,
    CHANGE
  );
}

// --------------------------------------------------------------
// Episode latch maintenance
// --------------------------------------------------------------
void photodiode_update() {
  // Sample analog channel (authoritative)
  analogReadResolution(12);
  int analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);
  float analog_v = (analog_raw / 4095.0f) * 3.3f;

  static bool light_present = false;
  static uint32_t dark_since_ms = 0;

  uint32_t now = millis();

  // --- Hysteresis-based light presence ---
  if (!light_present) {
    if (analog_v >= PHOTODIODE_ON_THRESHOLD_V) {
      light_present = true;
    }
  } else {
    if (analog_v <= PHOTODIODE_OFF_THRESHOLD_V) {
      light_present = false;
    }
  }

  noInterrupts();

  // --- Dark stability / latch reset ---
  if (!light_present) {
    if (dark_since_ms == 0) {
      dark_since_ms = now;
    }

    if (now - dark_since_ms >= PHOTODIODE_OFF_STABLE_MS) {
      photodiode_episode_latched = false;
    }
  } else {
    dark_since_ms = 0;

    // --- Episode detection ---
    if (photodiode_edge_seen && !photodiode_episode_latched) {
      photodiode_episode_latched = true;
      photodiode_episode_count++;
    }
  }

  photodiode_edge_seen = false;
  interrupts();
}

// --------------------------------------------------------------
// Lifecycle control
// --------------------------------------------------------------
void photodiode_clear() {
  noInterrupts();
  photodiode_episode_count = 0;
  photodiode_episode_latched = false;
  photodiode_edge_seen = false;
  interrupts();
}

// --------------------------------------------------------------
// Telemetry builders
// --------------------------------------------------------------
String buildPhotodiodeStatusBody() {
  int edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  uint32_t count;
  noInterrupts();
  count = photodiode_episode_count;
  interrupts();

  analogReadResolution(12);
  int analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);

  float analog_v = (analog_raw / 4095.0f) * 3.3f;

  String out;
  out += "{";

  out += "\"edge_level\":";
  out += edge_level;
  out += ",\"edge_pulse_count\":";
  out += count;
  out += ",\"analog_raw\":";
  out += analog_raw;
  out += ",\"analog_v\":";

  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.5f", analog_v);
    out += buf;
  }

  out += "}";
  return out;
}

String buildPhotodiodeCountBody() {
  uint32_t count;
  noInterrupts();
  count = photodiode_episode_count;
  interrupts();

  String out;
  out += "{";

  out += "\"count\":";
  out += count;

  out += "}";
  return out;
}
