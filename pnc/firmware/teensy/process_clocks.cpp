#include "process_clocks.h"

#include "clock.h"
#include "events.h"
#include "process.h"

#include <Arduino.h>

// ================================================================
// Lifecycle
// ================================================================

static bool clocks_start(void) {
  enqueueEvent("CLOCKS_INIT_ENTER", "\"stage\":\"process_start\"");
  return true;
}

static void clocks_stop(void) {
  enqueueEvent("CLOCKS_STOP", "\"stage\":\"process_stop\"");
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — dump raw ledgers + synthetic nanosecond clocks
// ------------------------------------------------------------
static const String* cmd_report(const char* /*args_json*/) {

  // Persistent payload storage (safe to return pointer)
  static String payload;
  payload = "{";

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
  payload += "\"dwt_cycles\":";
  payload += dwt_cycles;

  payload += ",\"gnss_10khz_ticks\":";
  payload += gnss_10khz;

  payload += ",\"ocxo_10khz_ticks\":";
  payload += ocxo_10khz;

  // ----------------------------------------------------------
  // Synthetic nanosecond clocks
  // ----------------------------------------------------------
  payload += ",\"dwt_ns\":";
  payload += dwt_ns;

  payload += ",\"gnss_ns\":";
  payload += gnss_ns;

  payload += ",\"ocxo_ns\":";
  payload += ocxo_ns;

  payload += "}";

  return &payload;
}

// ------------------------------------------------------------
// CLEAR — zero all synthetic clocks
// ------------------------------------------------------------
static const String* cmd_clear(const char*) {
  clock_zero_all();

  enqueueEvent("CLOCKS_CLEAR", "\"action\":\"all_zeroed\"");

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
  .start = clocks_start,
  .stop  = clocks_stop,
  .query = nullptr,
  .commands = CLOCKS_COMMANDS,
  .command_count = 2,
};

void process_clocks_register(void) {
  process_register(PROCESS_TYPE_CLOCKS, &CLOCKS_PROCESS);
}
