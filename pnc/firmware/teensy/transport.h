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
// NOTE:
//   • This API is non-reentrant by construction.
//   • Physical I/O is scheduled internally; callers never transmit inline.
//

void transport_send(
  uint8_t traffic,
  const Payload& payload
);


// =============================================================
// Transport RX diagnostics (authoritative snapshot)
// =============================================================
//
// This structure represents a coherent snapshot of transport-layer
// receive behavior and framing outcomes.
//
// Semantics:
//   • All fields are monotonic counters.
//   • Values are best-effort snapshots.
//   • No locking or synchronization is performed.
//   • This data is read-only from the caller’s perspective.
//
// Intended use:
//   • SYSTEM.REPORT
//   • PERFORMANCE diagnostics
//   • Forensic debugging without perturbation
//

typedef struct {

  // Raw ingress
  uint32_t rx_blocks_total;        // RawHID blocks or serial RX chunks
  uint32_t rx_bytes_total;         // Bytes appended to RX buffer

  // Framing outcomes
  uint32_t rx_frames_complete;     // Frames passing STX + length + ETX
  uint32_t rx_frames_dispatched;   // Frames delivered to recv_cb

  // RX state resets
  uint32_t rx_reset_hard;          // Hard RX state resets

  // Framing failures
  uint32_t rx_bad_stx;             // Buffer did not start with "<STX="
  uint32_t rx_bad_etx;             // Missing or misplaced "<ETX>"
  uint32_t rx_len_overflow;        // Declared length exceeded limits

  // Concurrency / overlap signal
  uint32_t rx_overlap;             // New traffic observed while RX active

  // Invalid traffic bit
  uint32_t rx_expected_traffic_missing;

} transport_info_t;


// =============================================================
// Transport diagnostics access
// =============================================================
//
// Populate a snapshot of current transport RX diagnostics.
//
// Contract:
//   • Safe to call from scheduled (non-ISR) context.
//   • Does NOT allocate.
//   • Does NOT schedule.
//   • Does NOT emit transport traffic.
//   • Transport remains the sole owner of the underlying state.
//

void transport_get_info(
  transport_info_t* out
);


// =============================================================
// Lifecycle
// =============================================================
//
// Initialize the transport subsystem.
// Owns physical I/O, RX polling, framing, and dispatch.
//

void transport_init(void);
