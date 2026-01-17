#pragma once

#include <stddef.h>
#include "transport.h"

/**
 * -----------------------------------------------------------------------------
 *  Serial Transport Adapter (serial.h)
 * -----------------------------------------------------------------------------
 *
 *  Owns the physical serial interface (ZPNET_SERIAL) and bridges it to the
 *  transport layer.
 *
 *  Architectural rules:
 *   - This is the ONLY module allowed to touch ZPNET_SERIAL
 *   - RX ingestion is TimePop-authorized and scheduled
 *   - No ISR executes protocol logic
 *   - No polling from zpnet_loop()
 *
 *  This module is intentionally thin and stateful.
 * -----------------------------------------------------------------------------
 */

/**
 * Initialize the serial subsystem and underlying transport.
 *
 * This function:
 *  - Initializes the physical UART
 *  - Initializes the transport layer
 *  - Arms the TimePop-based RX ingestion loop
 */
void serial_init();

/**
 * Send a framed payload over serial.
 *
 * Convenience wrapper around transport_send_frame().
 */
void serial_send(const char* payload, size_t length);
void serial_send(const char* payload);
