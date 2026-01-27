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
  debug_log("setup", "cpu_usage *init*");
  cpu_usage_init();

  // ----------------------------------------------------------
  // Bring time into existence (control plane)
  // ----------------------------------------------------------
  debug_log("setup", "timepop *init*");
  timepop_init();

  // ----------------------------------------------------------
  // Transport subsystem
  // ----------------------------------------------------------
  debug_log("setup", "transport *init*");
  transport_init();

  // ----------------------------------------------------------
  // Core subsystems that depend on time
  // ----------------------------------------------------------
  debug_log("setup", "clock *init*");
  clock_init();

  // ----------------------------------------------------------
  // Process framework
  // ----------------------------------------------------------
  debug_log("setup", "process *init*");
  process_init();

  // ----------------------------------------------------------
  // Register processes
  // ----------------------------------------------------------

  debug_log("setup", "process_events *init*");
  process_events_init();
  process_events_register();

  debug_log("setup", "enqueueEvent(BOOT)");
  Payload ev;
  ev.add("status", "READY");
  enqueueEvent("TEENSY_BOOT", ev);

  debug_log("setup", "process_timepop_register *init*");
  process_timepop_register();

  debug_log("setup", "process_clocks_register *init*");
  process_clocks_register();

  debug_log("setup", "process_laser *int*");
  process_laser_init();
  process_laser_register();

  debug_log("setup", "process_photodiode *init*");
  process_photodiode_init();
  process_photodiode_register();

  debug_log("setup", "process_tempest_register");
  process_tempest_register();

  debug_log("setup", "process_system_register");
  process_system_register();

  debug_log("setup", "timepop_arm(TIMEPOP_CLASS_CPU_SAMPLE)");
  cpu_usage_init_timer();

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
