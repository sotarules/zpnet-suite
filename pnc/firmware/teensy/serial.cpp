#include "config.h"
#include "serial.h"
#include "timepop.h"
#include "command.h"
#include "debug.h"
#include "event_bus.h"

#include <Arduino.h>

/**
 * -----------------------------------------------------------------------------
 *  Serial Transport Adapter (serial.cpp)
 * -----------------------------------------------------------------------------
 *
 *  Owns the ZPNet transport serial interface.
 *
 *  IMPORTANT TRANSPORT SEMANTICS:
 *  -------------------------------
 *  ZPNET_SERIAL is USB CDC (Serial), NOT a hardware UART.
 *
 *    • Boolean test   (if (ZPNET_SERIAL))  → USB host attached
 *    • available()    → bytes received from host
 *
 *  RX ingestion is performed via a self-rescheduling TimePop callback.
 *  No polling occurs in zpnet_loop().
 *
 *  This module is the ONLY place where ZPNET_SERIAL is referenced.
 * -----------------------------------------------------------------------------
 */

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// RX drain interval (microseconds)
static constexpr uint32_t SERIAL_RX_INTERVAL_MS = 5;

// Optional bounded wait for USB host attach during bring-up
static constexpr uint32_t USB_ATTACH_TIMEOUT_MS = 2000;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

static void serial_rx_tick(void*);

// -----------------------------------------------------------------------------
// Transport RX callback
// -----------------------------------------------------------------------------
//
// Called ONLY after a fully validated frame is received.
//
static void on_transport_frame(
    const char* payload,
    size_t      length
) {
    static char cmd_buf[TRANSPORT_MAX_PAYLOAD + 1];

    if (length >= sizeof(cmd_buf)) {
        enqueueErrEvent("COMMAND_ERROR", "command too long");
        return;
    }

    static char dbg[256];
    size_t n = (length < 256) ? length : 256;
    memcpy(dbg, payload, n);
    dbg[n] = '\0';

    memcpy(cmd_buf, payload, length);
    cmd_buf[length] = '\0';

    command_exec(cmd_buf);
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void serial_init() {
    // -------------------------------------------------------------------------
    // Initialize USB CDC transport (ZPNET_SERIAL)
    // -------------------------------------------------------------------------
    ZPNET_SERIAL.begin(115200);

    // Optional, bounded wait for USB host enumeration.
    // This tests *attachment only*, NOT data presence.
    unsigned long start = millis();
    while (!ZPNET_SERIAL && (millis() - start < USB_ATTACH_TIMEOUT_MS)) {
        // wait briefly for host (safe to skip in headless mode)
    }

    // -------------------------------------------------------------------------
    // Initialize transport layer (pure parser + framing)
    // -------------------------------------------------------------------------
    transport_init(on_transport_frame);

    // -------------------------------------------------------------------------
    // Arm RX ingestion loop (explicit causality via TimePop)
    // -------------------------------------------------------------------------
    timepop_schedule(
        SERIAL_RX_INTERVAL_MS,
        TIMEPOP_UNITS_MILLISECONDS,
        serial_rx_tick,
        nullptr,
        "serial-rx"
    );
}

// -----------------------------------------------------------------------------
// RX ingestion (TimePop-authorized, scheduled context)
// -----------------------------------------------------------------------------

static void serial_rx_tick(void*) {
    // Drain RX buffer.
    // NOTE:
    //   available() is the ONLY correct test for incoming data.
    //   Do NOT use boolean semantics here.
    while (ZPNET_SERIAL.available()) {
        char c = (char)ZPNET_SERIAL.read();
        transport_ingest_byte(c);
    }

    // Self-reschedule next ingestion window
    timepop_schedule(
        SERIAL_RX_INTERVAL_MS,
        TIMEPOP_UNITS_MILLISECONDS,
        serial_rx_tick,
        nullptr,
        "serial-rx"
    );
}

// -----------------------------------------------------------------------------
// TX (framed transport egress)
// -----------------------------------------------------------------------------

void serial_send(const char* payload, size_t length) {
    transport_send_frame(payload, length);
}

void serial_send(const char* payload) {
    transport_send_frame(payload);
}
