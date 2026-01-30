#include "process_photodiode.h"

#include "config.h"
#include "events.h"
#include "payload.h"
#include "process.h"

#include <Arduino.h>

// ================================================================
// ISR-driven edge visibility
// ================================================================

static volatile bool     pd_edge_seen        = false;
static volatile bool     pd_episode_latched  = false;
static volatile uint32_t pd_episode_count    = 0;

static volatile uint32_t pd_isr_count        = 0;
static int  last_edge_level                  = -1;
static bool edge_level_changed               = false;

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
  int      edge_level       = 0;

  uint16_t analog_raw = 0;
  float    analog_v   = NAN;
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

  int level = digitalRead(PHOTODIODE_EDGE_PIN);
  if (last_edge_level != -1 && level != last_edge_level) {
    edge_level_changed = true;
  }
  last_edge_level = level;

  PD.edge_level = level;

  interrupts();
}

// ================================================================
// Explicit initialization (authoritative, idempotent)
// ================================================================

void process_photodiode_init(void) {

  pinMode(PHOTODIODE_EDGE_PIN, INPUT);
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PHOTODIODE_EDGE_PIN),
    photodiodeISR,
    CHANGE
  );

  noInterrupts();
  pd_edge_seen       = false;
  pd_episode_latched = false;
  pd_episode_count   = 0;
  interrupts();

  Payload ev;
  ev.add("edge_pin", PHOTODIODE_EDGE_PIN);
  ev.add("analog_pin", PHOTODIODE_ANALOG_PIN);
  enqueueEvent("PHOTODIODE_INITIALIZATION", ev);
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// INIT — explicit hardware initialization wrapper
// ------------------------------------------------------------
static Payload cmd_init(const Payload& /*args*/) {
  process_photodiode_init();
  return ok_payload();
}

// ------------------------------------------------------------
// REPORT — return current photodiode state snapshot
// ------------------------------------------------------------
static Payload cmd_report(const Payload& /*args*/) {

  photodiode_snapshot();

  Payload p;

  p.add("edge_level", PD.edge_level);
  p.add("edge_pulse_count", PD.edge_pulse_count);
  p.add("analog_raw", PD.analog_raw);
  p.add("analog_v", PD.analog_v);

  {
    uint32_t count;
    noInterrupts();
    count = pd_isr_count;
    interrupts();
    p.add("isr_count", count);
  }

  p.add("edge_level_changed", edge_level_changed);

  return p;
}

// ------------------------------------------------------------
// COUNT — episode counter snapshot
// ------------------------------------------------------------
static Payload cmd_count(const Payload& /*args*/) {

  uint32_t count;
  noInterrupts();
  count = pd_episode_count;
  interrupts();

  Payload p;
  p.add("count", count);

  return p;
}

// ------------------------------------------------------------
// CLEAR — reset episode counter
// ------------------------------------------------------------
static Payload cmd_clear(const Payload& /*args*/) {

  noInterrupts();
  pd_episode_count   = 0;
  pd_episode_latched = false;
  pd_edge_seen       = false;
  interrupts();

  Payload ev;
  ev.add("action", "counter_reset");
  enqueueEvent("PHOTODIODE_CLEAR", ev);

  return ok_payload();
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PHOTODIODE_COMMANDS[] = {
  { "INIT",   cmd_init   },
  { "REPORT", cmd_report },
  { "COUNT",  cmd_count  },
  { "CLEAR",  cmd_clear  },
};

static const process_vtable_t PHOTODIODE_PROCESS = {
  .name = "PHOTODIODE",
  .query = nullptr,
  .commands = PHOTODIODE_COMMANDS,
  .command_count = 4,
};

// ------------------------------------------------------------
// Registration
// ------------------------------------------------------------

void process_photodiode_register(void) {
  process_register("PHOTODIODE", &PHOTODIODE_PROCESS);
}
