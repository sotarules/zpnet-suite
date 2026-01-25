// =============================================================
// FILE: process_system.cpp
// =============================================================
//
// SYSTEM Process — Teensy-side system status clearinghouse
//
// Phase 1 implementation:
//   • Implements SYSTEM.REPORT
//   • Payload is identical to legacy TEENSY.STATUS
//   • No side effects
//
// =============================================================

#include "process_system.h"

#include "config.h"
#include "process.h"
#include "event_bus.h"
#include "cpu_usage.h"
#include "util.h"

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

static bool system_start(void) {
  // SYSTEM has no hardware ownership.
  // Existence implies availability.
  enqueueEvent("SYSTEM_INIT_ENTER", "\"stage\":\"process_start\"");
  return true;
}

static void system_stop(void) {
  // No teardown required
  enqueueEvent("SYSTEM_STOP", "\"stage\":\"process_stop\"");
}



// ------------------------------------------------------------
// System status functions
// ------------------------------------------------------------

String buildTeensyStatusBody() {

  String out;
  out += "{";

  // ------------------------------------------------------------
  // Firmware identity
  // ------------------------------------------------------------
  out += "\"fw_version\":\"";
  out += FW_VERSION;
  out += "\"";

  // ------------------------------------------------------------
  // CPU temperature (best-effort)
  // ------------------------------------------------------------
  out += ",\"cpu_temp_c\":";
  out += cpuTempC();

  // ------------------------------------------------------------
  // Internal reference voltage (best-effort)
  // ------------------------------------------------------------
  out += ",\"vref_v\":";
  out += readVrefVolts();

  // ------------------------------------------------------------
  // Heap availability
  // ------------------------------------------------------------
  out += ",\"free_heap_bytes\":";
  out += freeHeapBytes();

  // ------------------------------------------------------------
  // CPU usage (authoritative, idle-cycle accounting)
  // ------------------------------------------------------------
  out += ",\"cpu_usage_pct\":";
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.4f", cpu_usage_get_percent());
    out += buf;
  }

  // ------------------------------------------------------------
  // CPU usage raw counters (audit + diagnostics)
  // ------------------------------------------------------------
  out += ",\"cpu_busy_cycles\":";
  out += cpu_usage_get_busy_cycles();

  out += ",\"cpu_total_cycles\":";
  out += cpu_usage_get_total_cycles();

  out += ",\"cpu_sample_window_ms\":";
  out += cpu_usage_get_sample_window_ms();

  out += ",\"cpu_freq_mhz\":";
  out += cpu_usage_get_cpu_freq_mhz();

  out += "}";
  return out;
}

// ------------------------------------------------------------
// Commands
// ------------------------------------------------------------

// REPORT — authoritative system snapshot
//
// Phase 1 semantics:
//   • EXACTLY the same payload as legacy TEENSY.STATUS
//   • No aggregation
//   • No interpretation
//
static String cmd_report(const char*) {
  return buildTeensyStatusBody();
}

// ------------------------------------------------------------
// Registration
// ------------------------------------------------------------

static const process_command_entry_t SYSTEM_COMMANDS[] = {
  { "REPORT", cmd_report },
};

static const process_vtable_t SYSTEM_PROCESS = {
  .name          = "SYSTEM",
  .start         = system_start,
  .stop          = system_stop,
  .query         = nullptr,
  .commands      = SYSTEM_COMMANDS,
  .command_count = 1,
};

void process_system_register(void) {
  process_register(PROCESS_TYPE_SYSTEM, &SYSTEM_PROCESS);
}