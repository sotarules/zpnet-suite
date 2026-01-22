#include "debug.h"
#include "system.h"
#include "clock.h"
#include "timepop.h"
#include "event_bus.h"
#include "serial.h"
#include "laser.h"
#include "cpu_usage.h"
#include "process.h"
#include "process_clocks.h"
#include "process_laser.h"
#include "process_photodiode.h"
#include "process_tempest.h"

// ------------------------------------------------------------
// CPU usage sampler (TimePop task)
// ------------------------------------------------------------
static void cpu_usage_tick(void*) {
    cpu_usage_sample();

    // Self-reschedule (1 Hz)
    timepop_schedule(
        1000,
        TIMEPOP_UNITS_MILLISECONDS,
        cpu_usage_tick,
        nullptr,
        "cpu-usage"
    );
}

// -----------------------------------------------------------------------------
// ZPNet Runtime Initialization
// -----------------------------------------------------------------------------

void setup() {

  cpu_usage_init();
  debug_init();
  timepop_init();
  serial_init();
  clock_init();
  event_bus_init();
  system_init();
  process_init();

  process_clocks_register();
  process_laser_register();
  process_start(PROCESS_TYPE_LASER);
  process_photodiode_register();
  process_start(PROCESS_TYPE_PHOTODIODE);
  process_tempest_register();

  // Arm CPU usage sampler AFTER timepop_init()
  timepop_schedule(
      1000,
      TIMEPOP_UNITS_MILLISECONDS,
      cpu_usage_tick,
      nullptr,
      "cpu-usage"
  );

  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

// -----------------------------------------------------------------------------
// ZPNet Runtime Loop
// -----------------------------------------------------------------------------

void loop() {

  // Dispatch interrupt-authorized callbacks
  timepop_dispatch();
}
