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
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "util.h"
#include "timepop.h"
#include "debug.h"

// --------------------------------------------------------------
// Forward declarations (internal terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (already implemented by you)
extern "C" void enter_bootloader_cleanly(void);

// --------------------------------------------------------------
// Internal system state
// --------------------------------------------------------------
static bool system_shutdown = false;
static bool system_bootloader = false;

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

static bool system_start(void) {
  system_shutdown   = false;
  system_bootloader = false;

  Payload ev;
  ev.add("stage", "process_start");
  enqueueEvent("SYSTEM_INIT_ENTER", ev);

  return true;
}

static void system_stop(void) {
  Payload ev;
  ev.add("stage", "process_stop");
  enqueueEvent("SYSTEM_STOP", ev);
}

// ================================================================
// Internal helpers
// ================================================================

static void enter_bootloader_cb(timepop_ctx_t*, void*) {
  // Idempotent + higher priority than shutdown
  if (system_bootloader) {
    system_enter_quiescence();
  }

  system_bootloader = true;

  // Best-effort observability
  {
    Payload ev;
    ev.add("status", "ENTERING");
    enqueueEvent("SYSTEM_BOOTLOADER", ev);
  }

  // Allow event bus to drain
  delay(10);

  // Visible debug pattern
  debug_blink("911");

  // Terminal transition — never returns
  enter_bootloader_cleanly();

  // Absolute fallback
  system_enter_quiescence();
}

bool system_is_shutdown() {
  return system_shutdown;
}

// --------------------------------------------------------------
// Terminal actions
// --------------------------------------------------------------

void system_request_shutdown() {
  // Idempotent
  if (system_shutdown || system_bootloader) {
    system_enter_quiescence();
  }

  system_shutdown = true;

  {
    Payload ev;
    ev.add("status", "REQUESTED");
    enqueueEvent("SYSTEM_SHUTDOWN", ev);
  }

  system_enter_quiescence();
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------

void system_enter_quiescence() {
  while (true) {
    delay(1000);
  }
}

// ------------------------------------------------------------
// Commands
// ------------------------------------------------------------

// ------------------------------------------------------------
// REPORT — authoritative system snapshot
//
// Phase 1 semantics:
//   • EXACTLY the same payload as legacy TEENSY.STATUS
//   • No aggregation
//   • No interpretation
// ------------------------------------------------------------
static const Payload* cmd_report(const char* /*args_json*/) {

  static Payload p;
  p.clear();

  // ------------------------------------------------------------
  // Firmware identity
  // ------------------------------------------------------------
  p.add("fw_version", FW_VERSION);

  // ------------------------------------------------------------
  // CPU temperature (best-effort)
  // ------------------------------------------------------------
  p.add("cpu_temp_c", cpuTempC());

  // ------------------------------------------------------------
  // Internal reference voltage (best-effort)
  // ------------------------------------------------------------
  p.add("vref_v", readVrefVolts());

  // ------------------------------------------------------------
  // Heap availability
  // ------------------------------------------------------------
  p.add("free_heap_bytes", freeHeapBytes());

  // ------------------------------------------------------------
  // CPU usage (authoritative, idle-cycle accounting)
  // ------------------------------------------------------------
  p.add("cpu_usage_pct", cpu_usage_get_percent());

  // ------------------------------------------------------------
  // CPU usage raw counters (audit + diagnostics)
  // ------------------------------------------------------------
  p.add("cpu_busy_cycles", cpu_usage_get_busy_cycles());
  p.add("cpu_total_cycles", cpu_usage_get_total_cycles());
  p.add("cpu_sample_window_ms", cpu_usage_get_sample_window_ms());
  p.add("cpu_freq_mhz", cpu_usage_get_cpu_freq_mhz());

  return &p;
}

// ------------------------------------------------------------
// ENTER_BOOTLOADER — terminal, irreversible
// ------------------------------------------------------------
static const Payload* cmd_enter_bootloader(const char* /*args_json*/) {

  // Schedule bootloader entry asynchronously so the command path
  // can return cleanly before USB disappears.
  timepop_arm(
    TIMEPOP_CLASS_FLASH,
    false,                       // one-shot
    enter_bootloader_cb,
    nullptr,
    "bootloader-flash"
  );

  {
    Payload ev;
    ev.add("action", "scheduled");
    enqueueEvent("SYSTEM_ENTER_BOOTLOADER", ev);
  }

  // Side-effect only
  return nullptr;
}

// ------------------------------------------------------------
// SHUTDOWN — cooperative system shutdown
// ------------------------------------------------------------
static const Payload* cmd_shutdown(const char* /*args_json*/) {

  {
    Payload ev;
    ev.add("action", "requested");
    enqueueEvent("SYSTEM_SHUTDOWN", ev);
  }

  system_request_shutdown();

  // Side-effect only
  return nullptr;
}

// ------------------------------------------------------------
// PROCESS LIST — registry introspection
// ------------------------------------------------------------
static const Payload* cmd_process_list(const char* /*args_json*/) {

  // process_list_json already returns a serialized object;
  // wrap it as a payload explicitly.
  static Payload p;
  p.clear();

  // Controlled escape hatch: inject trusted JSON object
  p.add("processes", process_list_json().c_str());

  return &p;
}

// ------------------------------------------------------------
// PROCESS START
// ------------------------------------------------------------
static const Payload* cmd_process_start(const char* args_json) {

  process_type_t type;
  if (!process_type_from_name(args_json, type)) {
    // Programmer error — no payload
    return nullptr;
  }

  process_start(type);

  // Side-effect only
  return nullptr;
}

// ------------------------------------------------------------
// PROCESS STOP
// ------------------------------------------------------------
static const Payload* cmd_process_stop(const char* args_json) {

  process_type_t type;
  if (!process_type_from_name(args_json, type)) {
    return nullptr;
  }

  process_stop(type);

  // Side-effect only
  return nullptr;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t SYSTEM_COMMANDS[] = {
  { "REPORT",            cmd_report            },
  { "ENTER_BOOTLOADER",  cmd_enter_bootloader  },
  { "SHUTDOWN",          cmd_shutdown          },
  { "PROCESS_LIST",      cmd_process_list      },
  { "PROCESS_START",     cmd_process_start     },
  { "PROCESS_STOP",      cmd_process_stop      },
};

static const process_vtable_t SYSTEM_PROCESS = {
  .name          = "SYSTEM",
  .start         = system_start,
  .stop          = system_stop,
  .query         = nullptr,            // deprecated
  .commands      = SYSTEM_COMMANDS,
  .command_count = sizeof(SYSTEM_COMMANDS) / sizeof(SYSTEM_COMMANDS[0]),
};

void process_system_register(void) {
  process_register(PROCESS_TYPE_SYSTEM, &SYSTEM_PROCESS);
}
