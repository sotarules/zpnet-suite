#pragma once

#include <stddef.h>
#include <stdint.h>

// =============================================================
// ZPNet Transport (Unified, Final)
// =============================================================
//
// This module owns ALL transport responsibilities:
//
//   • Runtime selection of physical transport (HID or Serial)
//   • Framed TX (<STX=N> ... <ETX>)
//   • Bytewise RX ingestion
//   • RX scheduling via TimePop
//   • Transport → command bridge (command_exec)
//
// There are NO other transport-related files in the system.
//
// =============================================================

// Defensive payload bound
static constexpr size_t TRANSPORT_MAX_PAYLOAD = 512;


// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------

// Initialize the transport subsystem.
// Must be called once during setup().
void transport_init(void);

// -------------------------------------------------------------
// TX egress (framed)
// -------------------------------------------------------------

// Send framed payload (payload only, no framing bytes)
void transport_send_frame(const char* payload, size_t length);

// Convenience overload
void transport_send_frame(const char* payload);
