#include "config.h"
#include "debug.h"

#include "clock.h"
#include "timepop.h"
#include "events.h"
#include "cpu_usage.h"
#include "process.h"
#include "transport.h"

#include "payload.h"

#include "process_clocks.h"
#include "process_events.h"
#include "process_timepop.h"
#include "process_laser.h"
#include "process_photodiode.h"
#include "process_tempest.h"
#include "process_system.h"

// ============================================================================
// HARD FAULT BARRIER (TOP-LEVEL, NON-NEGOTIABLE)
// ============================================================================
//
// On Teensy (ARM Cortex-M), there are NO language-level exceptions.
// Any serious failure results in a CPU fault.
//
// These handlers ARE the outermost fault barrier.
// They must log aggressively and NEVER return.
//
// ============================================================================

extern "C" {

// --------------------------------------------------------------------------
// Shared fault barrier
// --------------------------------------------------------------------------

[[noreturn]]
void fault_barrier(const char* fault_name) {

  // Announce failure loudly and immediately
  debug_log("FAULT", "==================================================");
  debug_log("FAULT", "ENTERING FAULT BARRIER");
  debug_log("FAULT", fault_name);
  debug_log("FAULT", "SYSTEM STATE IS NO LONGER TRUSTWORTHY");
  debug_log("FAULT", "EXECUTION HALTED");
  debug_log("FAULT", "==================================================");

  // Visual indicator: unmistakable fault pattern
  while (true) {
    debug_blink("FAULT");
    delay(500);
  }
}

// --------------------------------------------------------------------------
// Cortex-M fault handlers
// --------------------------------------------------------------------------

void HardFault_Handler(void) {
  fault_barrier("HardFault");
}

void MemManage_Handler(void) {
  fault_barrier("MemManageFault");
}

void BusFault_Handler(void) {
  fault_barrier("BusFault");
}

void UsageFault_Handler(void) {
  fault_barrier("UsageFault");
}

} // extern "C"

// ============================================================================
// ZPNet Runtime Initialization
// ============================================================================
//
// This function performs all construction-time wiring of the system.
// No execution-time assumptions are made here.
//
// Order is logical, not temporal.
// No data flows until loop() begins dispatching TimePop.
// ============================================================================

void setup() {

  delay(100);   // Allow USB enumeration to fully settle

  // ----------------------------------------------------------
  // Visible liveness + debug channel
  // ----------------------------------------------------------

  debug_blink("1155");
  debug_init();
  debug_log("setup", "*fire*");

  // ----------------------------------------------------------
  // Core instrumentation / diagnostics
  // ----------------------------------------------------------

  debug_log("setup", "cpu_usage *init*");
  cpu_usage_init();

  // ----------------------------------------------------------
  // Bring time into existence (control plane)
  // ----------------------------------------------------------

  debug_log("setup", "timepop *init*");
  timepop_init();

  // ----------------------------------------------------------
  // Transport subsystem (byte ↔ meaning boundary)
  // ----------------------------------------------------------

  debug_log("setup", "transport *init*");
  transport_init();

  // ----------------------------------------------------------
  // Register receive callbacks (semantic entry points)
  // ----------------------------------------------------------
  //
  // Transport owns decoding and routing.
  // Application intent is bound here, once.
  //

  debug_log("setup", "transport_register_receive_callback(REQUEST_RESPONSE)");
  transport_register_receive_callback(
    TRAFFIC_REQUEST_RESPONSE,
    process_command
  );

  // ----------------------------------------------------------
  // Core subsystems that depend on time
  // ----------------------------------------------------------

  debug_log("setup", "clock *init*");
  clock_init();

  // ----------------------------------------------------------
  // Process framework (semantic command surface)
  // ----------------------------------------------------------

  debug_log("setup", "process *init*");
  process_init();

  // ----------------------------------------------------------
  // Register processes (authoritative command surfaces)
  // ----------------------------------------------------------

  debug_log("setup", "process_events *init*");
  process_events_init();
  process_events_register();

  // Explicit boot fact (durable truth)
  debug_log("setup", "enqueueEvent(TEENSY_BOOT)");
  Payload ev;
  ev.add("status", "READY");
  enqueueEvent("TEENSY_BOOT", ev);

  debug_log("setup", "process_timepop_register");
  process_timepop_register();

  debug_log("setup", "process_clocks_register");
  process_clocks_register();

  debug_log("setup", "process_laser *init*");
  process_laser_init();
  process_laser_register();

  debug_log("setup", "process_photodiode *init*");
  process_photodiode_init();
  process_photodiode_register();

  debug_log("setup", "process_tempest_register");
  process_tempest_register();

  debug_log("setup", "process_system_register");
  process_system_register();

  // ----------------------------------------------------------
  // Periodic CPU usage sampling (TimePop-managed)
  // ----------------------------------------------------------

  debug_log("setup", "cpu_usage_init_timer");
  cpu_usage_init_timer();

  // ----------------------------------------------------------
  // Keep debug channel alive during development
  // ----------------------------------------------------------

  debug_beacon();
}

// ============================================================================
// ZPNet Runtime Loop
// ============================================================================
//
// All execution happens here.
// TimePop owns dispatch.
// No work occurs outside scheduled context.
// ============================================================================

void loop() {
  timepop_dispatch();
}
