/*
  ZPNet Teensy Telemetry Firmware — Core Shell

  Responsibilities:
    • setup() / loop()
    • Priority ordering
    • Delegation only (no domain logic)

  Invariants:
    • Single-threaded, single-loop execution
    • No multitasking, no scheduler
    • No unsolicited serial output
    • Durable truth exits ONLY via event bus
    • QUERY replies are immediate and non-destructive
*/

#include <Arduino.h>

// --------------------------------------------------------------
// Core configuration and infrastructure
// --------------------------------------------------------------
#include "config/config.h"
#include "util/util.h"
#include "event/event_bus.h"
#include "command/command.h"
#include "core/system.h"
#include "transport/transport.h"
#include "transport/zpnet_serial.h"

// --------------------------------------------------------------
// Subsystems (passive, cooperative)
// --------------------------------------------------------------
#include "subsystems/gnss/gnss.h"
#include "subsystems/laser/laser.h"
#include "subsystems/photodiode/photodiode.h"

// --------------------------------------------------------------
// Clock substrate (shared, side-effect conscious)
// --------------------------------------------------------------
#include "clock/dwt_clock.h"

// --------------------------------------------------------------
// Transport RX callback
// --------------------------------------------------------------
//
// Invoked ONLY after a fully validated framed payload is received.
// Payload is NOT null-terminated.
//
static void on_transport_frame(
    const char* payload,
    size_t      length
) {
  static char cmd_buf[256];

  if (length == 0 || length >= sizeof(cmd_buf)) {
    return;  // silently discard
  }

  memcpy(cmd_buf, payload, length);
  cmd_buf[length] = '\0';

  command_exec(cmd_buf);
}

// --------------------------------------------------------------
// setup()
// --------------------------------------------------------------
void setup() {
  // ------------------------------------------------------------
  // Enable core-local cycle counter (USB-safe)
  // ------------------------------------------------------------
  dwt_clock_init();

  // ------------------------------------------------------------
  // Initialize UART transport (runtime channel)
  // ------------------------------------------------------------
  ZPNET_SERIAL.begin(115200);
  transport_init(on_transport_frame);

  // ------------------------------------------------------------
  // System lifecycle state
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
// loop()
// --------------------------------------------------------------
void loop() {
  // ------------------------------------------------------------
  // Terminal condition (irreversible)
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
  // Priority 2: GNSS serial ingestion (budgeted)
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
