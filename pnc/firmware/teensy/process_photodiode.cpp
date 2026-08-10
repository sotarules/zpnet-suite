#include "process_photodiode.h"

#include "config.h"
#include "events.h"
#include "payload.h"
#include "process.h"
#include "process_interrupt.h"
#include "util.h"

#include <Arduino.h>

// ================================================================
// PHOTODIODE compatibility shim
// ================================================================
//
// This legacy process remains temporarily available while PHOTONS takes over
// optical timing custody.
//
// Important boundary:
//   • process_interrupt owns PHOTODIODE_EDGE_PIN and all PD200T edge timing.
//   • process_photodiode NEVER attaches an ISR and NEVER resets interrupt state.
//   • PHOTODIODE_MON_PIN is sampled only as slow PD200T comparator-threshold
//     telemetry.
//
// The command surface is retained so existing tools continue to work during
// migration.  COUNT reflects process_interrupt's observed PHOTODIODE IRQ count.
// CLEAR is intentionally a no-op because this compatibility layer has no
// authority to erase process_interrupt evidence.
// ================================================================

struct photodiode_compat_state_t {
  int      edge_level = 0;
  uint16_t mon_raw = 0;
  float    mon_v = NAN;
};

static photodiode_compat_state_t PD;

// ================================================================
// Snapshot
// ================================================================

static void photodiode_snapshot(void) {
  analogReadResolution(12);

  PD.mon_raw = analogRead(PHOTODIODE_MON_PIN);
  PD.mon_v = (PD.mon_raw / ADC_FS_COUNTS) * ADC_FS_VOLTS;

  // Ambient GPIO level is compatibility telemetry only.  It is not timing
  // evidence and must never substitute for process_interrupt's edge capture.
  PD.edge_level = digitalRead(PHOTODIODE_EDGE_PIN);
}

// ================================================================
// Explicit initialization — compatibility only
// ================================================================

void process_photodiode_init(void) {
  pinMode(PHOTODIODE_EDGE_PIN, INPUT);
  pinMode(PHOTODIODE_MON_PIN, INPUT);
  analogReadResolution(12);

  // DO NOT attachInterrupt() here.
  // process_interrupt is the sole owner of PD200T comparator edge custody.

  Payload ev;
  ev.add("compatibility_shim", true);
  ev.add("edge_pin", PHOTODIODE_EDGE_PIN);
  ev.add("mon_pin", PHOTODIODE_MON_PIN);
  ev.add("interrupt_owner", "PROCESS_INTERRUPT");
  enqueueEvent("PHOTODIODE_INITIALIZATION", ev);
}

// ================================================================
// Commands
// ================================================================

static Payload cmd_init(const Payload& /*args*/) {
  process_photodiode_init();
  return ok_payload();
}

static Payload cmd_report(const Payload& /*args*/) {
  photodiode_snapshot();

  interrupt_photodiode_diag_t diag{};
  const bool diag_ok = interrupt_photodiode_snapshot(&diag);

  Payload p;
  p.add("compatibility_shim", true);

  p.add("edge_pin", PHOTODIODE_EDGE_PIN);
  p.add("edge_level", PD.edge_level);

  p.add("mon_pin", PHOTODIODE_MON_PIN);
  p.add("mon_raw", PD.mon_raw);
  p.add("mon_v", toFixedDecimal(PD.mon_v, 6));

  p.add("interrupt_diag_ok", diag_ok);
  p.add("interrupt_subscribed", diag.subscribed);
  p.add("interrupt_active", diag.active);
  p.add("interrupt_irq_count", diag.irq_count);
  p.add("interrupt_callback_count", diag.callback_count);
  p.add("interrupt_callback_missing_count", diag.callback_missing_count);
  p.add("interrupt_inactive_edge_count", diag.inactive_edge_count);
  p.add("interrupt_last_isr_entry_dwt_raw", diag.last_isr_entry_dwt_raw);
  p.add("interrupt_last_dwt_at_edge", diag.last_dwt_at_edge);

  return p;
}

static Payload cmd_count(const Payload& /*args*/) {
  interrupt_photodiode_diag_t diag{};
  const bool diag_ok = interrupt_photodiode_snapshot(&diag);

  Payload p;
  p.add("compatibility_shim", true);
  p.add("snapshot_ok", diag_ok);
  p.add("count", diag.irq_count);
  return p;
}

static Payload cmd_clear(const Payload& /*args*/) {
  // Compatibility no-op.  This process no longer owns any edge counter or ISR
  // state, and therefore has nothing lawful to clear.
  Payload p = ok_payload();
  p.add("compatibility_shim", true);
  p.add("cleared", false);
  p.add("reason", "interrupt custody owned by PROCESS_INTERRUPT");
  return p;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PHOTODIODE_COMMANDS[] = {
  { "INIT",   cmd_init   },
  { "REPORT", cmd_report },
  { "COUNT",  cmd_count  },
  { "CLEAR",  cmd_clear  },
  { nullptr, nullptr }
};

static const process_vtable_t PHOTODIODE_PROCESS = {
  .process_id = "PHOTODIODE",
  .commands = PHOTODIODE_COMMANDS,
  .subscriptions = nullptr,
};

void process_photodiode_register(void) {
  process_register("PHOTODIODE", &PHOTODIODE_PROCESS);
}
