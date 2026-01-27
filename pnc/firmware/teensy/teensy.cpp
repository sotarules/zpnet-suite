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

// ------------------------------------------------------------
// CPU usage sampler (TimePop recurring task)
// ------------------------------------------------------------
//
// This callback is invoked by TimePop at the cadence defined
// for TIMEPOP_CLASS_CPU_SAMPLE. No self-rescheduling.
//
static void cpu_usage_tick(timepop_ctx_t* timer, void* /*user*/) {
  cpu_usage_sample();
}

// -----------------------------------------------------------------------------
// ZPNet Runtime Initialization
// -----------------------------------------------------------------------------

void setup() {
  delay(100);   // allow USB enumeration to fully settle
  debug_blink("1155");
  debug_init();
  debug_log("setup", "*fire*");

  // ----------------------------------------------------------
  // Core instrumentation / diagnostics
  // ----------------------------------------------------------
  debug_log("setup", "cpu_usage_init");
  cpu_usage_init();

  // ----------------------------------------------------------
  // Bring time into existence (control plane)
  // ----------------------------------------------------------
  debug_log("setup", "timepop_init");
  timepop_init();

  // ----------------------------------------------------------
  // Transport subsystem
  // ----------------------------------------------------------
  debug_log("setup", "transport_init");
  transport_init();

  // ----------------------------------------------------------
  // Core subsystems that depend on time
  // ----------------------------------------------------------
  debug_log("setup", "clock_init");
  clock_init();

  // ----------------------------------------------------------
  // Process framework
  // ----------------------------------------------------------
  debug_log("setup", "process_init");
  process_init();

  // ----------------------------------------------------------
  // Register processes
  // ----------------------------------------------------------

  debug_log("setup", "process_events_register");
  process_events_register();

  debug_log("setup", "process_timepop_register");
  process_timepop_register();

  debug_log("setup", "process_clocks_register");
  process_clocks_register();

  debug_log("setup", "process_laser_register");
  process_laser_register();
  debug_log("setup", "process_start(PROCESS_TYPE_LASER)");
  process_start(PROCESS_TYPE_LASER);

  debug_log("setup", "process_photodiode_register");
  process_photodiode_register();
  debug_log("setup", "process_start(PROCESS_TYPE_PHOTODIODE)");
  process_start(PROCESS_TYPE_PHOTODIODE);

  debug_log("setup", "process_tempest_register");
  process_tempest_register();

  debug_log("setup", "process_system_register");
  process_system_register();

  // ----------------------------------------------------------
  // Arm recurring CPU usage sampler
  // ----------------------------------------------------------
  //
  // One call.
  // No durations.
  // No self-reschedule.
  //
  debug_log("setup", "timepop_arm(TIMEPOP_CLASS_CPU_SAMPLE)");
  timepop_arm(
    TIMEPOP_CLASS_CPU_SAMPLE,
    true,                    // recurring
    cpu_usage_tick,
    nullptr,
    "cpu-usage"
  );

  // ----------------------------------------------------------
  // Signal readiness
  // ----------------------------------------------------------
  debug_log("setup", "enqueueEvent(BOOT)");
  {
    Payload ev;
    ev.add("status", "READY");
    enqueueEvent("BOOT", ev);
  }

  // ----------------------------------------------------------
  // Keep stuff flowing in the debug channel during development
  // ----------------------------------------------------------
  debug_beacon();
}

// -----------------------------------------------------------------------------
// ZPNet Runtime Loop
// -----------------------------------------------------------------------------

void loop() {
  timepop_dispatch();
}
