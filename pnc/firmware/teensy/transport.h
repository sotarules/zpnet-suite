#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// ============================================================================
// ZPNet Transport — Public Interface (Teensy)
// ============================================================================
//
// This header defines the ONLY public contract for the transport layer.
//
// TRUTHFUL DESIGN:
//
//   • Transport owns all physical I/O.
//   • Receive is asynchronous and callback-driven.
//   • Send is semantic and message-oriented.
//   • HID and SERIAL diverge internally at the physical layer.
//   • Message framing and dispatch are transport responsibilities.
//
// No public API exposes byte- or block-level I/O.
// That boundary is intentionally sealed.
//
// ============================================================================


// =============================================================
// Compile-time transport selection (authoritative)
// =============================================================
//
// Transport backend selection is determined by ONE of:
//
//   1. Explicit ZPNET override (preferred for non-Arduino builds):
//        - ZPNET_TRANSPORT_HID
//        - ZPNET_TRANSPORT_SERIAL
//
//   2. Teensy USB build configuration (authoritative on Teensy):
//        - USB_SERIAL
//        - USB_RAWHID
//        - USB_SERIAL_HID
//        - USB_RAWHID_SERIAL
//
// Exactly ONE backend must be selected.
//

#if defined(ZPNET_TRANSPORT_HID)
  #define ZPNET_TRANSPORT_SELECTED_HID

#elif defined(ZPNET_TRANSPORT_SERIAL)
  #define ZPNET_TRANSPORT_SELECTED_SERIAL

#elif defined(USB_RAWHID) || defined(USB_RAWHID_SERIAL)
  #define ZPNET_TRANSPORT_SELECTED_HID

#elif defined(USB_SERIAL) || defined(USB_SERIAL_HID)
  #define ZPNET_TRANSPORT_SELECTED_SERIAL

#else
  #error "No transport backend defined (ZPNET_TRANSPORT_* or USB_*)"
#endif


// =============================================================
// Transport limits (authoritative)
// =============================================================

static constexpr size_t TRANSPORT_MAX_MESSAGE = 10 * 1024;


// =============================================================
// Traffic types (authoritative)
// =============================================================
//
// Traffic byte is the first byte of every message and determines
// semantic routing after message reassembly.
//

static constexpr uint8_t TRAFFIC_DEBUG             = 0xD0;
static constexpr uint8_t TRAFFIC_REQUEST_RESPONSE  = 0xD1;
static constexpr uint8_t TRAFFIC_PUBLISH_SUBSCRIBE = 0xD2;


// =============================================================
// Application receive callback
// =============================================================
//
// Transport delivers fully reassembled, message-atomic Payloads
// via registered callbacks. No receive polling API exists.
//

using transport_receive_cb_t =
  void (*)(const Payload&);


// =============================================================
// Receive registration
// =============================================================
//
// Register a callback for a given traffic class.
// Only one callback per traffic class is supported.
//

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
);


// =============================================================
// Semantic send API (public entry point)
// =============================================================
//
// Send ONE complete semantic message.
// Framing, fragmentation, and physical I/O are handled internally.
//

void transport_send(
  uint8_t traffic,
  const Payload& payload
);


// =============================================================
// Lifecycle
// =============================================================
//
// Initialize the transport subsystem.
// Owns physical I/O, RX polling, framing, and dispatch.
//

void transport_init(void);