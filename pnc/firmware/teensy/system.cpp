#include "debug.h"
#include "system.h"
#include "event_bus.h"
#include "config.h"

#include <Arduino.h>
#include <NativeEthernet.h>

// --------------------------------------------------------------
// Internal system state
// --------------------------------------------------------------
static bool system_shutdown = false;

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void system_init() {
  // Reset shutdown state
  system_shutdown = false;
}

bool system_is_shutdown() {
  return system_shutdown;
}

void system_request_shutdown() {
  // Idempotent: multiple calls collapse to one fate
  if (system_shutdown) {
    system_enter_quiescence();
  }

  system_shutdown = true;

  // Durable signal before silence
  enqueueEvent("SYSTEM_SHUTDOWN", "\"status\":\"REQUESTED\"");

  system_enter_quiescence();
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------
void system_enter_quiescence() {

  // Inert forever
  while (true) {
    delay(1000);
  }
}
