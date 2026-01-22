#include "process_clocks.h"

#include "clock.h"
#include "event_bus.h"
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
// REPORT — dump raw synthetic clocks
// ------------------------------------------------------------
static String cmd_report(const char*) {
  uint64_t dwt  = dwt_now();
  uint64_t gnss = gnss_now();
  uint64_t ocxo = ocxo_now();

  String r = "{";

  r += "\"dwt_cycles\":";
  r += dwt;

  r += ",\"gnss_ticks\":";
  r += gnss;

  r += ",\"ocxo_ticks\":";
  r += ocxo;

  r += "}";

  return r;
}

// ------------------------------------------------------------
// CLEAR — zero all synthetic clocks
// ------------------------------------------------------------
static String cmd_clear(const char*) {
  dwt_zero();
  gnss_zero();
  ocxo_zero();

  enqueueEvent("CLOCKS_CLEAR", "\"action\":\"all_zeroed\"");

  return "{\"status\":\"ok\"}";
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
