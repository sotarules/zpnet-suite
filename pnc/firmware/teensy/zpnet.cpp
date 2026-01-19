#include "zpnet.h"
#include "timepop.h"
#include "event_bus.h"
#include "serial.h"
#include "process.h"
#include "process_gnss.h"
#include "debug.h"
#include "cpu_usage.h"

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

void zpnet_setup() {

  cpu_usage_init();
  debug_init();
  timepop_init();
  serial_init();
  event_bus_init();
  process_init();
  process_gnss_register();
  //process_start(PROCESS_TYPE_GNSS);

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

void zpnet_loop() {

  // Dispatch interrupt-authorized callbacks
  timepop_dispatch();
}
