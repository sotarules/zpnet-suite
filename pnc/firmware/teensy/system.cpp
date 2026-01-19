#include "system.h"

#include "event_bus.h"
#include "config.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal system state
// --------------------------------------------------------------
static bool system_shutdown = false;

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void system_init() {
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

  // Safety: disable laser outputs immediately
  digitalWrite(LD_ON_PIN, LOW);

  // Durable signal before silence
  enqueueEvent("SYSTEM_SHUTDOWN", "\"status\":\"REQUESTED\"");

  system_enter_quiescence();
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------
void system_enter_quiescence() {
  // Optional: detach interrupts to reduce noise
  detachInterrupt(digitalPinToInterrupt(PHOTODIODE_EDGE_PIN));

  // Inert forever
  while (true) {
    delay(1000);
  }
}
