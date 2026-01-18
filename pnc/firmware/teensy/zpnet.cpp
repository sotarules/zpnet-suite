#include "zpnet.h"
#include "timepop.h"
#include "event_bus.h"
#include "serial.h"
#include "process.h"
#include "process_gnss.h"
#include "debug.h"

// -----------------------------------------------------------------------------
// ZPNet Runtime Initialization
// -----------------------------------------------------------------------------

void zpnet_setup() {

  // Debug will always be enabled in this build
  debug_init();

  // Initialize TimePop kernel
  timepop_init();

  // Initialize serial subsystem (this primes RX ingestion)
  serial_init();

  // Event bus initialization
  event_bus_init();

  // Initialize process subsystem and register processes
  process_init();
  process_gnss_register();
  process_start(PROCESS_TYPE_GNSS);

  // ------------------------------------------------------------
  // Durable boot signal
  // ------------------------------------------------------------
  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

// -----------------------------------------------------------------------------
// ZPNet Runtime Loop
// -----------------------------------------------------------------------------

void zpnet_loop() {
  // Dispatch interrupt-authorized callbacks
  timepop_dispatch();
}
