// transport.h — ZPNet USB CDC Serial Transport Public Interface (Teensy)
// ----------------------------------------------------------------------
//
// TRUTHFUL DESIGN:
//
//   • Transport owns all physical USB CDC Serial I/O.
//   • HID / RawHID support is intentionally retired.
//   • Receive is asynchronous and callback-driven.
//   • Send is semantic and message-oriented.
//   • Callers enqueue; transport despools.
//   • Message framing and dispatch are transport responsibilities.
//
// TX Architecture:
//
//   • transport_send() serializes directly into one queued heap allocation.
//   • A TIMEPOP-driven pump performs physical Serial writes.
//   • Each job owns its heap allocation — freed on completion.
//   • Memory is bounded via soft budget cap (TX_BUDGET_MAX).
//
// Wire format:
//
//   traffic-byte <STX=n> JSON <ETX>
//
// ============================================================

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// =============================================================
// Transport limits (authoritative)
// =============================================================
//
// Payload::to_json() is bounded by Payload::ARENA_MAX, currently below 16 KiB.
// Keep transport's receive buffer above that envelope while avoiding the old
// 32 KiB static RX allocation.

static constexpr size_t TRANSPORT_MAX_MESSAGE = 16 * 1024;

// =============================================================
// Traffic types (authoritative)
// =============================================================

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
//   • transport_send() serializes and enqueues.
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
// Semantics:
//   • All fields are monotonic counters or bounded snapshots.
//   • Best-effort values (no locking).
//   • No allocation.
//   • No side effects.
//

typedef struct {

  // TX — Budget / Queue Health
  size_t   tx_budget_max;
  size_t   tx_budget_used;
  size_t   tx_budget_high_water;

  size_t   tx_job_count;
  size_t   tx_job_high_water;

  uint32_t tx_jobs_enqueued;
  uint32_t tx_jobs_sent;

  uint32_t tx_bytes_enqueued;
  uint32_t tx_bytes_sent;

  // TX — Failure counters
  uint32_t tx_alloc_fail;
  uint32_t tx_budget_fail;
  uint32_t tx_queue_full;
  uint32_t tx_rr_drop_count;

  // RX — Raw ingress
  uint32_t rx_blocks_total;       // retained for schema compatibility; serial-only reports 0
  uint32_t rx_bytes_total;

  // RX — Framing outcomes
  uint32_t rx_frames_complete;
  uint32_t rx_frames_dispatched;

  // RX — State resets
  uint32_t rx_reset_hard;

  // RX — Framing failures
  uint32_t rx_bad_stx;
  uint32_t rx_bad_etx;
  uint32_t rx_len_overflow;

  // RX — Concurrency / anomaly signals
  uint32_t rx_overlap;
  uint32_t rx_expected_traffic_missing;

} transport_info_t;

// =============================================================
// Transport diagnostics access
// =============================================================

void transport_get_info(
  transport_info_t* out
);

// =============================================================
// Lifecycle
// =============================================================

void transport_init(void);
