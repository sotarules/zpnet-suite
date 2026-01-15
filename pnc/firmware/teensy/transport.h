#pragma once

#include <Arduino.h>
#include <stddef.h>

// --------------------------------------------------------------
// ZPNet Framed UART Transport
//
// Frame format:
//
//   <STX=N>
//   <payload bytes...>
//   <ETX>
//
// Validation rules:
//   1. Must start with <STX=N>
//   2. Must read exactly N payload bytes
//   3. Must end with <ETX>
//
// Invalid frames are discarded silently.
//
// --------------------------------------------------------------

// Maximum payload size accepted (defensive bound)
static const size_t TRANSPORT_MAX_PAYLOAD = 512;

// Callback invoked when a valid payload is received.
// Payload is NOT null-terminated.
typedef void (*transport_rx_callback_t)(
    const char* payload,
    size_t      length
);

// Initialize transport with a callback for valid frames
void transport_init(transport_rx_callback_t cb);

// Feed one byte into the transport parser
void transport_ingest_byte(char c);

// Send a framed payload (payload does NOT include framing)
void transport_send_frame(const char* payload, size_t length);

// Convenience overload for null-terminated strings
void transport_send_frame(const char* payload);
