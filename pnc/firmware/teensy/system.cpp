#include "debug.h"
#include "system.h"
#include "event_bus.h"
#include "config.h"

#include <Arduino.h>
#include <NativeEthernet.h>

// Bootloader entry symbol (already implemented by you)
extern "C" void enter_bootloader_cleanly(void);

// --------------------------------------------------------------
// Internal system state
// --------------------------------------------------------------
static bool system_shutdown = false;
static bool system_bootloader = false;

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void system_init() {
  system_shutdown   = false;
  system_bootloader = false;
}

bool system_is_shutdown() {
  return system_shutdown;
}

// --------------------------------------------------------------
// Terminal actions
// --------------------------------------------------------------
void system_request_shutdown() {
  // Idempotent
  if (system_shutdown || system_bootloader) {
    system_enter_quiescence();
  }

  system_shutdown = true;

  enqueueEvent(
    "SYSTEM_SHUTDOWN",
    "\"status\":\"REQUESTED\""
  );

  system_enter_quiescence();
}

void system_enter_bootloader() {

  // Idempotent + higher priority than shutdown
  if (system_bootloader) {
    system_enter_quiescence();
  }

  system_bootloader = true;

  // Best-effort observability
  enqueueEvent(
    "SYSTEM_BOOTLOADER",
    "\"status\":\"ENTERING\""
  );

  // Allow event bus to drain
  delay(10);

  // Visible debug pattern
  debug_blink("911");

  // Terminal transition — never returns
  enter_bootloader_cleanly();

  // Absolute fallback
  system_enter_quiescence();
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------
void system_enter_quiescence() {
  while (true) {
    delay(1000);
  }
}
