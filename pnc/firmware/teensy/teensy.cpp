/*
  ZPNet Teensy Telemetry Firmware — Integration Shell

  Responsibilities:
    • setup() / loop()
    • Priority ordering
    • Delegation to subsystems
    • No domain logic

  Invariants:
    • No unsolicited serial output
    • Durable truth exits only via event bus
    • QUERY replies are immediate and non-destructive
    • Exactly one serial consumer
*/

#include <Arduino.h>

// --------------------------------------------------------------
// Core configuration and shared infrastructure
// --------------------------------------------------------------
#include "config.h"
#include "util.h"
#include "event_bus.h"
#include "command.h"
#include "system.h"

// --------------------------------------------------------------
// Subsystems
// --------------------------------------------------------------
#include "gnss.h"
#include "laser.h"
#include "photodiode.h"

// --------------------------------------------------------------
// Setup
// --------------------------------------------------------------
void setup() {
  // USB CDC (commands + telemetry)
  Serial.begin(USB_SERIAL_BAUD);
  while (!Serial) {}

  // Core system state
  system_init();

  // Subsystems (order matters)
  laser_init();
  photodiode_init();
  gnss_init();

  // Durable boot signal
  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

// --------------------------------------------------------------
// Main loop
// --------------------------------------------------------------
void loop() {
  // Terminal condition
  if (system_is_shutdown()) {
    system_enter_quiescence();
    // never returns
  }

  // ------------------------------------------------------------
  // Priority 1: photodiode episode state machine
  // ------------------------------------------------------------
  photodiode_update();

  // ------------------------------------------------------------
  // Priority 2: GNSS PPS handling (internally rate-limited)
  // ------------------------------------------------------------
  gnss_handle_pps();

  // ------------------------------------------------------------
  // Priority 3: USB CDC command ingestion
  // ------------------------------------------------------------
  static char cmd_buf[256];
  static size_t cmd_len = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len > 0) {
        command_exec(cmd_buf);
      }
      cmd_len = 0;
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }

  // ------------------------------------------------------------
  // Priority 4: GNSS serial ingestion (bounded, opportunistic)
  // ------------------------------------------------------------
  gnss_poll();
}
