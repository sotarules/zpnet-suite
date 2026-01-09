/*
  ZPNet Teensy Telemetry Firmware — Integration Shell

  Responsibilities:
    • setup() / loop()
    • Priority ordering
    • Delegation to subsystems
    • Transport ingress only (no semantics)

  Invariants:
    • No unsolicited serial output
    • Durable truth exits only via event bus
    • QUERY replies are immediate and non-destructive
    • Exactly one serial consumer
    • All inbound messages are framed and validated
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
#include "zpnet_serial.h"
#include "transport.h"

// --------------------------------------------------------------
// Subsystems
// --------------------------------------------------------------
#include "gnss.h"
#include "laser.h"
#include "photodiode.h"
#include "dwt_clock.h"

// --------------------------------------------------------------
// Transport RX callback
// --------------------------------------------------------------
//
// Called ONLY after a fully validated frame is received.
//
static void on_transport_frame(
    const char* payload,
    size_t      length
) {
  // Payload is NOT null-terminated; copy safely
  static char cmd_buf[256];

  if (length == 0 || length >= sizeof(cmd_buf)) {
    return;  // silently discard
  }

  memcpy(cmd_buf, payload, length);
  cmd_buf[length] = '\0';

  command_exec(cmd_buf);
}

// --------------------------------------------------------------
// Setup
// --------------------------------------------------------------
void setup() {
  // ------------------------------------------------------------
  // Enable DWT cycle counter (USB-safe)
  // ------------------------------------------------------------
  dwt_clock_init();

  // ------------------------------------------------------------
  // Initialize UART transport
  // ------------------------------------------------------------
  ZPNET_SERIAL.begin(115200);
  transport_init(on_transport_frame);

  // ------------------------------------------------------------
  // Core system state
  // ------------------------------------------------------------
  system_init();

  // ------------------------------------------------------------
  // Subsystems (order matters)
  // ------------------------------------------------------------
  laser_init();
  photodiode_init();
  gnss_init();

  // ------------------------------------------------------------
  // Durable boot signal
  // ------------------------------------------------------------
  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

// --------------------------------------------------------------
// Main loop
// --------------------------------------------------------------
void loop() {
  // ------------------------------------------------------------
  // Terminal condition
  // ------------------------------------------------------------
  if (system_is_shutdown()) {
    system_enter_quiescence();
    // never returns
  }

  // ------------------------------------------------------------
  // Priority 1: photodiode episode state machine
  // ------------------------------------------------------------
  photodiode_update();

  // ------------------------------------------------------------
  // Priority 2: GNSS serial ingestion
  // ------------------------------------------------------------
  gnss_poll();

  // ------------------------------------------------------------
  // Priority 3: UART framed command ingestion
  // ------------------------------------------------------------
  while (ZPNET_SERIAL.available()) {
    char c = ZPNET_SERIAL.read();
    transport_ingest_byte(c);
  }
}
