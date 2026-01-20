#include "process_photodiode.h"

#include "config.h"
#include "event_bus.h"
#include "process.h"

#include <Arduino.h>

// ================================================================
// ISR-driven edge visibility
// ================================================================

static volatile bool     pd_edge_seen = false;
static volatile bool     pd_episode_latched = false;
static volatile uint32_t pd_episode_count = 0;

static volatile uint32_t pd_isr_count = 0;
static int last_edge_level = -1;
static bool edge_level_changed = false;

// ================================================================
// ISR
// ================================================================

static void photodiodeISR() {
  pd_isr_count++;
  pd_edge_seen = true;
}

// ================================================================
// Photodiode State (authoritative snapshot)
// ================================================================

struct photodiode_state_t {
  uint32_t edge_pulse_count = 0;
  int      edge_level = 0;

  uint16_t analog_raw = 0;
  float    analog_v = NAN;
};

static photodiode_state_t PD;

// ================================================================
// Snapshot / Update Logic
// ================================================================

static void photodiode_snapshot(void) {
  // --- Sample analog channel ---
  analogReadResolution(12);
  PD.analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);
  PD.analog_v   = (PD.analog_raw / 4095.0f) * 3.3f;

  static bool     light_present = false;
  static uint32_t dark_since_ms = 0;

  uint32_t now = millis();

  // --- Hysteresis-based light presence ---
  if (!light_present) {
    if (PD.analog_v >= PHOTODIODE_ON_THRESHOLD_V) {
      light_present = true;
    }
  } else {
    if (PD.analog_v <= PHOTODIODE_OFF_THRESHOLD_V) {
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
      pd_episode_latched = false;
    }
  } else {
    dark_since_ms = 0;

    // --- Episode detection ---
    if (pd_edge_seen && !pd_episode_latched) {
      pd_episode_latched = true;
      pd_episode_count++;
    }
  }

  pd_edge_seen = false;

  PD.edge_pulse_count = pd_episode_count;
  PD.edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  int level = digitalRead(PHOTODIODE_EDGE_PIN);
  if (last_edge_level != -1 && level != last_edge_level) {
    edge_level_changed = true;
  }
  last_edge_level = level;

  PD.edge_level = level;

  interrupts();
}

// ================================================================
// Initialization Event
// ================================================================

static void photodiode_emit_init_event(void) {
  String body = "\"payload\": {";

  body += "\"edge_pin\":"; body += PHOTODIODE_EDGE_PIN;
  body += ",\"analog_pin\":"; body += PHOTODIODE_ANALOG_PIN;

  body += "}";

  enqueueEvent("PHOTODIODE_INITIALIZATION", body);
}

// ================================================================
// Lifecycle
// ================================================================

static bool photodiode_start(void) {
  enqueueEvent("PHOTODIODE_INIT_ENTER", "\"stage\":\"process_start\"");

  pinMode(PHOTODIODE_EDGE_PIN, INPUT);
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PHOTODIODE_EDGE_PIN),
    photodiodeISR,
    CHANGE
  );

  noInterrupts();
  pd_edge_seen = false;
  pd_episode_latched = false;
  pd_episode_count = 0;
  interrupts();

  photodiode_emit_init_event();
  return true;
}

static void photodiode_stop(void) {
  detachInterrupt(digitalPinToInterrupt(PHOTODIODE_EDGE_PIN));

  enqueueEvent(
    "PHOTODIODE_STOP",
    "\"action\":\"interrupt_detached\""
  );
}

// ================================================================
// Commands
// ================================================================

static String cmd_report(const char*) {
  photodiode_snapshot();

  String p = "{";

  p += "\"edge_level\":";
  p += PD.edge_level;

  p += ",\"edge_pulse_count\":";
  p += PD.edge_pulse_count;

  p += ",\"analog_raw\":";
  p += PD.analog_raw;

  p += ",\"analog_v\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.5f", PD.analog_v);
    p += buf;
  }

  p += ",\"isr_count\":";
  {
    uint32_t count;
    noInterrupts();
    count = pd_isr_count;
    interrupts();
    p += count;
  }

  p += ",\"edge_level_changed\":";
  p += (edge_level_changed ? "true" : "false");

  p += "}";

  return p;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PHOTODIODE_COMMANDS[] = {
  { "REPORT", cmd_report },
};

static const process_vtable_t PHOTODIODE_PROCESS = {
  .name = "PHOTODIODE",
  .start = photodiode_start,
  .stop = photodiode_stop,
  .query = nullptr,
  .commands = PHOTODIODE_COMMANDS,
  .command_count = 1,
};

void process_photodiode_register(void) {
  process_register(PROCESS_TYPE_PHOTODIODE, &PHOTODIODE_PROCESS);
}
