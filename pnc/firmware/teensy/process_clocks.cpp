#include "process_clocks.h"

#include "clock.h"
#include "events.h"
#include "payload.h"
#include "process.h"

#include <Arduino.h>

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — dump raw ledgers + synthetic nanosecond clocks
// ------------------------------------------------------------
static const Payload* cmd_report(const char* /*args_json*/) {

  // Persistent payload storage (safe to return pointer)
  static Payload p;
  p.clear();

  // Raw authoritative ledgers
  const uint64_t dwt_cycles = clock_dwt_cycles_now();
  const uint64_t gnss_10khz = clock_gnss_10khz_ticks();
  const uint64_t ocxo_10khz = clock_ocxo_10khz_ticks();

  // Synthetic nanosecond clocks
  const uint64_t dwt_ns  = clock_dwt_ns_now();
  const uint64_t gnss_ns = clock_gnss_ns_now();
  const uint64_t ocxo_ns = clock_ocxo_ns_now();

  // ----------------------------------------------------------
  // Raw ledgers
  // ----------------------------------------------------------
  p.add("dwt_cycles", dwt_cycles);
  p.add("gnss_10khz_ticks", gnss_10khz);
  p.add("ocxo_10khz_ticks", ocxo_10khz);

  // ----------------------------------------------------------
  // Synthetic nanosecond clocks
  // ----------------------------------------------------------
  p.add("dwt_ns", dwt_ns);
  p.add("gnss_ns", gnss_ns);
  p.add("ocxo_ns", ocxo_ns);

  return &p;
}

// ------------------------------------------------------------
// CLEAR — zero all synthetic clocks
// ------------------------------------------------------------
static const Payload* cmd_clear(const char* /*args_json*/) {

  clock_zero_all();

  Payload ev;
  ev.add("action", "all_zeroed");
  enqueueEvent("CLOCKS_CLEAR", ev);

  // Side-effect only, no payload
  return nullptr;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "REPORT", cmd_report },
  { "CLEAR",  cmd_clear  },
};

static const process_vtable_t CLOCKS_PROCESS = {
  .name = "CLOCKS",
  .query = nullptr,
  .commands = CLOCKS_COMMANDS,
  .command_count = 2,
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}
