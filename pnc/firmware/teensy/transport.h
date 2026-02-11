#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// ============================================================================
// ZPNet Transport — Public Interface (Teensy)
// ============================================================================
//
// TRUTHFUL DESIGN:
//
//   • Transport owns all physical I/O.
//   • Receive is asynchronous and callback-driven.
//   • Send is semantic and message-oriented.
//   • Callers enqueue; transport despools.
//   • Only the transport subsystem may touch HID / Serial.
//   • Message framing and dispatch are transport responsibilities.
//
// No public API exposes byte- or block-level I/O.
// That boundary is intentionally sealed.
//
// TX Architecture (Revised):
//
//   • transport_send() enqueues a cloned payload.
//   • A TIMEPOP-driven pump performs physical transmission.
//   • Single-writer guarantee eliminates block interleave.
//   • Memory is bounded via FIFO arena allocator.
//
// ============================================================================


// =============================================================
// Compile-time transport selection (authoritative)
// =============================================================
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

using transport_receive_cb_t =
  void (*)(const Payload&);


// =============================================================
// Receive registration
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
);


// =============================================================
// Semantic send API (public entry point)
// =============================================================
//
// Send ONE complete semantic message.
//
// IMPORTANT:
//
//   • Callers do NOT transmit inline.
//   • transport_send() enqueues.
//   • Physical transmission occurs later in scheduled context.
//   • This eliminates multi-writer interleave by construction.
//

void transport_send(
  uint8_t traffic,
  const Payload& payload
);


// =============================================================
// Transport diagnostics snapshot
// =============================================================
//
// This structure exposes monotonic counters and bounded
// memory usage metrics for both RX and TX subsystems.
//
// Semantics:
//   • All fields are monotonic counters or bounded snapshots.
//   • Best-effort values (no locking).
//   • No allocation.
//   • No side effects.
//
// Intended use:
//   • SYSTEM.REPORT
//   • PERFORMANCE diagnostics
//   • Forensic debugging
//

typedef struct {

  // ===========================================================
  // TX — Arena / Queue Health
  // ===========================================================

  uint32_t tx_arena_size;         // Total arena bytes
  uint32_t tx_arena_used;         // Current bytes allocated
  uint32_t tx_arena_high_water;   // Peak arena usage
  uint32_t tx_arena_alloc_fail;   // Allocation failures

  uint32_t tx_job_count;          // Current queued messages
  uint32_t tx_job_high_water;     // Peak queue depth

  uint32_t tx_jobs_enqueued;      // Total jobs enqueued
  uint32_t tx_jobs_sent;          // Total jobs fully sent

  uint32_t tx_bytes_enqueued;     // Total bytes enqueued
  uint32_t tx_bytes_sent;         // Total bytes sent

  // ===========================================================
  // RX — Raw ingress
  // ===========================================================

  uint32_t rx_blocks_total;       // RawHID blocks or serial RX chunks
  uint32_t rx_bytes_total;        // Bytes appended to RX buffer

  // ===========================================================
  // RX — Framing outcomes
  // ===========================================================

  uint32_t rx_frames_complete;     // Frames passing STX + length + ETX
  uint32_t rx_frames_dispatched;   // Frames delivered to recv_cb

  // ===========================================================
  // RX — State resets
  // ===========================================================

  uint32_t rx_reset_hard;          // Hard RX state resets

  // ===========================================================
  // RX — Framing failures
  // ===========================================================

  uint32_t rx_bad_stx;             // Buffer did not start with "<STX="
  uint32_t rx_bad_etx;             // Missing or misplaced "<ETX>"
  uint32_t rx_len_overflow;        // Declared length exceeded limits

  // ===========================================================
  // RX — Concurrency / anomaly signals
  // ===========================================================

  uint32_t rx_overlap;             // New traffic observed while RX active
  uint32_t rx_expected_traffic_missing;

} transport_info_t;


// =============================================================
// Transport diagnostics access
// =============================================================
//
// Populate a snapshot of current transport diagnostics.
//
// Contract:
//   • Safe to call from scheduled (non-ISR) context.
//   • Does NOT allocate.
//   • Does NOT schedule.
//   • Does NOT emit transport traffic.
//   • Transport remains sole owner of state.
//

void transport_get_info(
  transport_info_t* out
);


// =============================================================
// Lifecycle
// =============================================================
//
// Initialize the transport subsystem.
// Owns physical I/O, RX polling, TX pumping, framing, dispatch.
//

void transport_init(void);
