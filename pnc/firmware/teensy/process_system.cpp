// =============================================================
// FILE: process_system.cpp
// =============================================================
//
// SYSTEM Process — Teensy-side system truth surface
//
// SYSTEM is not a lifecycle-managed process.
// It cannot start or stop from within itself.
//
// It exposes authoritative, read-only system facts and
// provides explicit terminal transitions (shutdown, bootloader)
// that represent irreversible boundary crossings.
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
// Forward declarations (terminal paths)
// --------------------------------------------------------------
void system_enter_quiescence(void);

// Bootloader entry symbol (ROM-provided, never returns)
extern "C" void enter_bootloader_cleanly(void);

// --------------------------------------------------------------
// Internal terminal state
// --------------------------------------------------------------
static bool system_shutdown   = false;
static bool system_bootloader = false;

// ================================================================
// Terminal helpers
// ================================================================

static void enter_bootloader_cb(timepop_ctx_t*, void*) {

  // Idempotent, higher priority than shutdown
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

  // Irreversible transition
  enter_bootloader_cleanly();

  // Absolute fallback (should never return)
  system_enter_quiescence();
}

bool system_is_shutdown(void) {
  return system_shutdown;
}

void system_request_shutdown(void) {

  // Idempotent terminal action
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
void system_enter_quiescence(void) {
  while (true) {
    delay(1000);
  }
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — authoritative system snapshot
//
// Semantics:
//   • Stateless
//   • Read-only
//   • No aggregation
//   • No inference
// ------------------------------------------------------------
static const Payload* cmd_report(const Payload& /*args*/) {

  static Payload p;
  p.clear();

  // Firmware identity
  p.add("fw_version", FW_VERSION);

  // CPU temperature (best-effort)
  p.add("cpu_temp_c", cpuTempC());

  // Internal reference voltage (best-effort)
  p.add("vref_v", readVrefVolts());

  // Heap availability
  p.add("free_heap_bytes", freeHeapBytes());

  // CPU usage (authoritative, idle-cycle accounting)
  p.add("cpu_usage_pct", cpu_usage_get_percent());

  // CPU usage diagnostics
  p.add("cpu_busy_cycles", cpu_usage_get_busy_cycles());
  p.add("cpu_total_cycles", cpu_usage_get_total_cycles());
  p.add("cpu_sample_window_ms", cpu_usage_get_sample_window_ms());
  p.add("cpu_freq_mhz", cpu_usage_get_cpu_freq_mhz());

  return &p;
}

// ------------------------------------------------------------
// ENTER_BOOTLOADER — terminal, irreversible
// ------------------------------------------------------------
static const Payload* cmd_enter_bootloader(const Payload& /*args*/) {

  // Schedule transition so command path can return cleanly
  timepop_arm(
    TIMEPOP_CLASS_FLASH,
    false,                 // one-shot
    enter_bootloader_cb,
    nullptr,
    "bootloader-flash"
  );

  {
    Payload ev;
    ev.add("action", "scheduled");
    enqueueEvent("SYSTEM_ENTER_BOOTLOADER", ev);
  }

  return nullptr;
}

// ------------------------------------------------------------
// SHUTDOWN — terminal, irreversible
// ------------------------------------------------------------
static const Payload* cmd_shutdown(const Payload& /*args*/) {

  {
    Payload ev;
    ev.add("action", "requested");
    enqueueEvent("SYSTEM_SHUTDOWN", ev);
  }

  system_request_shutdown();
  return nullptr;
}

// ------------------------------------------------------------
// PROCESS_LIST — registry introspection (diagnostic only)
// ------------------------------------------------------------
static const Payload* cmd_process_list(const Payload&) {

  static Payload p;
  p.clear();

  PayloadArray arr;

  for (size_t i = 0; i < process_get_count(); i++) {
    Payload entry;
    entry.add("name", process_get_name(i));
    arr.add(entry);
  }

  p.add_array("processes", arr);
  return &p;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t SYSTEM_COMMANDS[] = {
  { "REPORT",           cmd_report           },
  { "ENTER_BOOTLOADER", cmd_enter_bootloader },
  { "SHUTDOWN",         cmd_shutdown         },
  { "PROCESS_LIST",     cmd_process_list     },
};

static const process_vtable_t SYSTEM_PROCESS = {
  .name          = "SYSTEM",
  .query         = nullptr,
  .commands      = SYSTEM_COMMANDS,
  .command_count = sizeof(SYSTEM_COMMANDS) / sizeof(SYSTEM_COMMANDS[0]),
};

void process_system_register(void) {
  process_register("SYSTEM", &SYSTEM_PROCESS);
}
