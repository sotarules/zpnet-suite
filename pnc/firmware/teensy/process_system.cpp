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
  enqueueEvent("SYSTEM_INIT_ENTER", "\"stage\":\"process_start\"");
  return true;
}

static void system_stop(void) {
  enqueueEvent("SYSTEM_STOP", "\"stage\":\"process_stop\"");
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
    enqueueEvent(
      "SYSTEM_BOOTLOADER",
      "\"status\":\"ENTERING\""
    );

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

  enqueueEvent(
    "SYSTEM_SHUTDOWN",
    "\"status\":\"REQUESTED\""
  );

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
static const String* cmd_report(const char* /*args_json*/) {

  // Persistent payload storage
  static String payload;
  payload = "{";

  // ------------------------------------------------------------
  // Firmware identity
  // ------------------------------------------------------------
  payload += "\"fw_version\":\"";
  payload += FW_VERSION;
  payload += "\"";

  // ------------------------------------------------------------
  // CPU temperature (best-effort)
  // ------------------------------------------------------------
  payload += ",\"cpu_temp_c\":";
  payload += cpuTempC();

  // ------------------------------------------------------------
  // Internal reference voltage (best-effort)
  // ------------------------------------------------------------
  payload += ",\"vref_v\":";
  payload += readVrefVolts();

  // ------------------------------------------------------------
  // Heap availability
  // ------------------------------------------------------------
  payload += ",\"free_heap_bytes\":";
  payload += freeHeapBytes();

  // ------------------------------------------------------------
  // CPU usage (authoritative, idle-cycle accounting)
  // ------------------------------------------------------------
  payload += ",\"cpu_usage_pct\":";
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.4f", cpu_usage_get_percent());
    payload += buf;
  }

  // ------------------------------------------------------------
  // CPU usage raw counters (audit + diagnostics)
  // ------------------------------------------------------------
  payload += ",\"cpu_busy_cycles\":";
  payload += cpu_usage_get_busy_cycles();

  payload += ",\"cpu_total_cycles\":";
  payload += cpu_usage_get_total_cycles();

  payload += ",\"cpu_sample_window_ms\":";
  payload += cpu_usage_get_sample_window_ms();

  payload += ",\"cpu_freq_mhz\":";
  payload += cpu_usage_get_cpu_freq_mhz();

  payload += "}";

  return &payload;
}

// ------------------------------------------------------------
// ENTER_BOOTLOADER — terminal, irreversible
// ------------------------------------------------------------
static const String* cmd_enter_bootloader(const char* /*args_json*/) {

  // Schedule bootloader entry asynchronously so the command path
  // can return cleanly before USB disappears.
  timepop_arm(
    TIMEPOP_CLASS_FLASH,
    false,                       // one-shot
    enter_bootloader_cb,
    nullptr,
    "bootloader-flash"
  );

  enqueueEvent(
    "SYSTEM_ENTER_BOOTLOADER",
    "\"action\":\"scheduled\""
  );

  // Side-effect only
  return nullptr;
}

// ------------------------------------------------------------
// SHUTDOWN — cooperative system shutdown
// ------------------------------------------------------------
static const String* cmd_shutdown(const char* /*args_json*/) {

  enqueueEvent(
    "SYSTEM_SHUTDOWN",
    "\"action\":\"requested\""
  );

  system_request_shutdown();

  // Side-effect only
  return nullptr;
}

// ------------------------------------------------------------
// PROCESS LIST — registry introspection
// ------------------------------------------------------------
static const String* cmd_process_list(const char* /*args_json*/) {

  // Persistent payload storage
  static String payload;
  payload = process_list_json();

  return &payload;
}

// ------------------------------------------------------------
// PROCESS START
// ------------------------------------------------------------
static const String* cmd_process_start(const char* args_json) {

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
static const String* cmd_process_stop(const char* args_json) {

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