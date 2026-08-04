// transport.h — ZPNet Transport Public Interface (Teensy)
// --------------------------------------------------------
//
// TRUTHFUL DESIGN:
//
//   • Transport owns all USB CDC serial I/O.
//   • Receive is asynchronous and callback-driven.
//   • Send is semantic and message-oriented.
//   • Callers enqueue; transport despools.
//   • Only the transport subsystem may touch Serial.
//   • Message framing and dispatch are transport responsibilities.
//
// WIRE FORMAT:
//
//   [traffic byte] <STX=json_len> JSON <ETX>
//
//   The length is authoritative. <ETX> is retained as a redundant safety
//   delimiter so malformed or misaligned frames are rejected cleanly.
//
// TX Architecture:
//
//   • transport_send() serializes one payload into one complete wire image.
//   • The wire image includes the traffic byte.
//   • Recurring TimePop callbacks perform bounded physical I/O.
//   • The pump advances only by bytes actually accepted by Serial.
//   • Each job owns its heap allocation — freed on completion.
//   • Memory is bounded via soft budget cap (TX_BUDGET_MAX).
//   • TimePop is the sole physical RX/TX service path.
//
// Serial-only doctrine:
//
//   HID is retired. There is no compile-time or runtime transport
//   selection here; USB CDC serial is the only physical transcript path.
//
// ============================================================

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"


// =============================================================
// USB mode guard
// =============================================================

#ifndef USB_SERIAL
  #error "ZPNet transport is serial-only; build Teensy with usb=serial / USB_SERIAL"
#endif


// =============================================================
// Transport limits (authoritative)
// =============================================================

static constexpr size_t TRANSPORT_MAX_MESSAGE = 40 * 1024;


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
//   • true means the complete wire image entered transport custody.
//   • false means serialization, sizing, budget, queue, or allocation failed.
//   • Physical transmission occurs later in scheduled context.
//   • This eliminates multi-writer interleave by construction.
//

bool transport_send(
  uint8_t traffic,
  const Payload& payload
);

// Enqueue one publish/subscribe envelope without constructing a second full
// Payload object. Transport writes {"topic":...,"payload":...} directly into
// the final TRAFFIC_PUBLISH_SUBSCRIBE wire allocation.
bool transport_send_publish(
  const char* topic,
  const Payload& payload
);


// =============================================================
// Runtime-loop witness
// =============================================================
//
// Physical RX/TX service is owned exclusively by recurring TimePop callbacks
// armed by transport_init().  This witness remains independent so reports can
// distinguish scheduler/transport progress from ordinary loop progress.

void transport_note_runtime_loop(void);


// =============================================================
// Retained RX semantic-dispatch breadcrumb
// =============================================================

static constexpr uint32_t TRANSPORT_RX_DISPATCH_SCHEMA_VERSION = 1U;

enum transport_rx_dispatch_stage_t : uint32_t {
  TRANSPORT_RX_DISPATCH_NONE = 0U,
  TRANSPORT_RX_DISPATCH_ENTER = 1U,
  TRANSPORT_RX_DISPATCH_CALLBACK_RETURN = 2U,
  TRANSPORT_RX_DISPATCH_COMPLETE = 3U,
  TRANSPORT_RX_DISPATCH_INVALID_CALLBACK = 4U,
  TRANSPORT_RX_DISPATCH_STACK_MISMATCH = 5U,
};

struct transport_rx_dispatch_breadcrumb_t {
  uint32_t sequence, sequence_inv, schema_version, stage;
  uint32_t dwt, msp_before, msp_after, min_msp;
  uint32_t callback, traffic, payload, payload_count;
  uint32_t ipsr, lr, reserved0, reserved1;
};

static_assert(sizeof(transport_rx_dispatch_breadcrumb_t) == 64U,
              "RX dispatch breadcrumb must stay two cache lines");

struct transport_rx_dispatch_snapshot_t {
  bool live_valid;
  bool retained_valid;
  transport_rx_dispatch_breadcrumb_t live;
  transport_rx_dispatch_breadcrumb_t retained;
};

void transport_rx_dispatch_snapshot(transport_rx_dispatch_snapshot_t* out);
const char* transport_rx_dispatch_stage_name(uint32_t stage);

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

  // ===========================================================
  // Scheduled transport service / runtime witness
  // ===========================================================

  uint32_t poll_count;              // Total scheduled RX + TX callback passes
  uint32_t poll_skipped_too_soon;   // Retained ABI field; always zero
  uint32_t rx_poll_count;           // Scheduled RX callback passes
  uint32_t tx_poll_count;           // Scheduled TX callback passes
  uint32_t runtime_loop_count;      // Runtime loop iterations observed
  uint32_t poll_interval_us;        // Scheduled callback period in microseconds

  // ===========================================================
  // TX — Budget / Queue Health
  // ===========================================================

  size_t   tx_budget_max;         // Soft cap (bytes)
  size_t   tx_budget_used;        // Current outstanding bytes
  size_t   tx_budget_high_water;  // Peak outstanding bytes

  size_t   tx_job_count;          // Current queued messages
  size_t   tx_job_high_water;     // Peak queue depth

  uint32_t tx_jobs_enqueued;      // Total jobs enqueued
  uint32_t tx_jobs_sent;          // Total jobs fully sent

  uint32_t tx_bytes_enqueued;     // Total complete wire bytes enqueued
  uint32_t tx_bytes_sent;         // Total complete wire bytes accepted by Serial

  // ===========================================================
  // TX — Failure counters
  // ===========================================================

  uint32_t tx_alloc_fail;         // malloc returned null
  uint32_t tx_budget_fail;        // Budget cap would be exceeded
  uint32_t tx_queue_full;         // Job ring was full
  uint32_t tx_rr_drop_count;      // RR messages dropped (any reason)

  // ===========================================================
  // TX — Per-message custody transcript
  // ===========================================================

  uint32_t tx_send_attempt_count;
  uint32_t tx_send_serialized_count;
  uint32_t tx_send_reject_count;
  uint32_t tx_send_enqueued_count;
  uint32_t tx_send_completed_count;

  uint32_t tx_message_too_large_count;
  uint32_t tx_header_format_fail_count;
  uint32_t tx_outstanding_budget_fail_count;
  uint32_t tx_empty_serialization_count;
  uint32_t tx_semantic_empty_count;

  uint32_t tx_last_sequence;
  uint32_t tx_last_traffic;
  uint32_t tx_last_json_bytes;
  uint32_t tx_last_wire_bytes;
  uint32_t tx_last_outcome_id;
  uint32_t tx_last_reason_id;
  char     tx_last_topic[48];

  uint32_t tx_largest_json_bytes;
  uint32_t tx_largest_wire_bytes;
  char     tx_largest_topic[48];

  uint32_t tx_last_reject_sequence;
  uint32_t tx_last_reject_traffic;
  uint32_t tx_last_reject_json_bytes;
  uint32_t tx_last_reject_wire_bytes;
  uint32_t tx_last_reject_reason_id;
  char     tx_last_reject_topic[48];

  uint32_t tx_last_completed_sequence;
  uint32_t tx_last_completed_traffic;
  uint32_t tx_last_completed_json_bytes;
  uint32_t tx_last_completed_wire_bytes;
  char     tx_last_completed_topic[48];

  uint32_t tx_timebase_attempt_count;
  uint32_t tx_timebase_serialized_count;
  uint32_t tx_timebase_reject_count;
  uint32_t tx_timebase_enqueued_count;
  uint32_t tx_timebase_completed_count;
  uint32_t tx_timebase_last_sequence;
  uint32_t tx_timebase_last_json_bytes;
  uint32_t tx_timebase_last_wire_bytes;
  uint32_t tx_timebase_last_outcome_id;
  uint32_t tx_timebase_last_reason_id;
  uint32_t tx_timebase_largest_json_bytes;
  uint32_t tx_timebase_largest_wire_bytes;

  // ===========================================================
  // RX — Raw ingress
  // ===========================================================

  uint32_t rx_blocks_total;       // Serial receive polls that appended bytes
  uint32_t rx_bytes_total;        // Bytes appended to RX buffer

  // ===========================================================
  // RX — Framing outcomes
  // ===========================================================

  uint32_t rx_frames_complete;    // Frames passing STX + length + ETX
  uint32_t rx_frames_dispatched;  // Frames delivered to recv_cb

  // ===========================================================
  // RX — State resets
  // ===========================================================

  uint32_t rx_reset_hard;         // Hard RX state resets

  // ===========================================================
  // RX — Framing failures
  // ===========================================================

  uint32_t rx_bad_stx;            // Buffer did not start with "<STX="
  uint32_t rx_bad_etx;            // Missing or misplaced "<ETX>"
  uint32_t rx_len_overflow;       // Declared length exceeded limits

  // ===========================================================
  // RX — Concurrency / anomaly signals
  // ===========================================================

  uint32_t rx_overlap;                  // New traffic while RX active / resync
  uint32_t rx_expected_traffic_missing; // Expected traffic byte absent
  uint32_t rx_dispatch_invalid_callback;
  uint32_t rx_dispatch_stack_mismatch;

  // ===========================================================
  // RX — Guarded RAM2 placement experiment
  // ===========================================================

  uint32_t rx_buffer_in_dmamem;
  uint32_t rx_buffer_address;
  size_t   rx_buffer_size;
  size_t   rx_buffer_alignment;
  uint32_t rx_buffer_alignment_ok;
  uint32_t rx_poison_byte;
  uint32_t rx_storage_init_count;
  uint32_t rx_json_terminator_count;
  uint32_t rx_guard_check_count;
  uint32_t rx_guard_failure_count;
  uint32_t rx_guard_before_failure_count;
  uint32_t rx_guard_after_failure_count;
  uint32_t rx_guard_last_stage;     // 1=before parse, 2=after parse, 3=after dispatch
  uint32_t rx_guard_last_side;      // 1=before buffer, 2=after buffer
  uint32_t rx_guard_last_index;
  uint32_t rx_guard_last_expected;
  uint32_t rx_guard_last_observed;

  // ===========================================================
  // RX — Startup attach quarantine / first-corruption witness
  // ===========================================================

  uint32_t rx_first_frame_seen;
  uint32_t rx_startup_grace_active;
  uint32_t rx_startup_grace_ms;
  uint32_t rx_ms_since_init;
  uint32_t rx_startup_expected_traffic_missing_suppressed;
  uint32_t rx_startup_bad_stx_suppressed;
  uint32_t rx_startup_bad_etx_suppressed;
  uint32_t rx_startup_len_overflow_suppressed;
  uint32_t rx_startup_first_missing_byte;
  uint32_t rx_startup_last_missing_byte;

} transport_info_t;


// =============================================================
// Transport diagnostics access
// =============================================================

void transport_get_info(
  transport_info_t* out
);

const char* transport_tx_outcome_name(uint32_t outcome_id);
const char* transport_tx_reason_name(uint32_t reason_id);


// =============================================================
// Lifecycle
// =============================================================

void transport_init(void);
