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
// Ethernet identity
// --------------------------------------------------------------
// NOTE: MAC address must be unique on your LAN.
// You may later want to derive this from hardware ID.
static byte SYSTEM_MAC[6] = {
  0x04, 0xE9, 0xE5, 0x00, 0x00, 0x01
};

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
