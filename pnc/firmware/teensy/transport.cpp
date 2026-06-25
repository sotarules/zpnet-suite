// transport.cpp — ZPNet USB CDC Serial Transport (Teensy)
// -------------------------------------------------------
//
// Serial-only doctrine:
//   • HID / RawHID support has been retired from Teensy firmware.
//   • There is one physical transport: USB CDC Serial.
//   • The first byte of each serial frame is the traffic class.
//   • The payload frame remains length-authoritative:
//       <STX=n> JSON <ETX>
//
// TX Architecture:
//   • transport_send() serializes directly into the queued wire buffer.
//   • A single TIMEPOP-driven pump performs physical Serial writes.
//   • Callers enqueue complete semantic messages; only transport touches Serial.
//   • Each job owns one heap allocation and frees it on completion.
//
// Memory model:
//   • No HID 64-byte block/deblock path exists.
//   • No Arduino String is constructed in transport_send().
//   • TX memory is bounded by TX_BUDGET_MAX.
//   • Failure mode is explicit: serialization, budget, queue, or malloc failure.
//

#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "process_performance.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =============================================================
// Constants
// =============================================================

static constexpr uint64_t TRANSPORT_RX_POLL_NS = 2000000ULL;  // 2 ms
static constexpr uint64_t TRANSPORT_TX_POLL_NS = 2000000ULL;  // 2 ms

static constexpr size_t RX_BUF_MAX = TRANSPORT_MAX_MESSAGE + 64;

// TX job queue depth.  Keep this unchanged: recent campaign telemetry has
// legitimately reached the low 50s without queue failure.
static constexpr size_t TX_JOB_MAX = 64;

// TX memory budget (soft cap, bytes).  Counts outstanding heap allocations,
// not just bytes remaining to send.
static constexpr size_t TX_BUDGET_MAX = 48 * 1024;

static constexpr uint8_t TX_TRAFFIC_BYTES = 1;
static constexpr size_t  TX_HEADER_RESERVE = 32;

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

static const char* BUILD_FINGERPRINT = "__FP__ZPNET_SERIAL_ONLY__";

// =============================================================
// TX Job Queue
// =============================================================
//
// data[] contains the complete wire image:
//
//   traffic-byte <STX=n> JSON <ETX>
//
// length is the number of wire bytes to send.  alloc_length is the actual heap
// allocation size and is used for budget accounting because transport_send()
// intentionally allocates a conservative serialization bound and then sends the
// actual serialized length.

struct tx_job_t {
  uint8_t* data = nullptr;
  size_t   length = 0;
  size_t   alloc_length = 0;
  size_t   sent = 0;
};

static tx_job_t tx_jobs[TX_JOB_MAX];

static size_t tx_job_head  = 0;
static size_t tx_job_tail  = 0;
static size_t tx_job_count = 0;

// =============================================================
// TX Counters & Budget Tracking
// =============================================================

static size_t tx_budget_used = 0;

static size_t tx_budget_high_water = 0;
static size_t tx_job_high_water    = 0;

static volatile uint32_t tx_jobs_enqueued  = 0;
static volatile uint32_t tx_jobs_sent      = 0;
static volatile uint32_t tx_bytes_enqueued = 0;
static volatile uint32_t tx_bytes_sent     = 0;

static volatile uint32_t tx_alloc_fail     = 0;
static volatile uint32_t tx_budget_fail    = 0;
static volatile uint32_t tx_queue_full     = 0;
static volatile uint32_t tx_rr_drop_count  = 0;

// =============================================================
// RX State
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic      = 0;

static volatile uint32_t rx_blocks_total             = 0;  // retained for schema compatibility; serial-only leaves it at 0
static volatile uint32_t rx_bytes_total              = 0;
static volatile uint32_t rx_frames_complete          = 0;
static volatile uint32_t rx_frames_dispatched        = 0;
static volatile uint32_t rx_reset_hard_count         = 0;
static volatile uint32_t rx_bad_stx_count            = 0;
static volatile uint32_t rx_bad_etx_count            = 0;
static volatile uint32_t rx_len_overflow_count       = 0;
static volatile uint32_t rx_overlap_count            = 0;
static volatile uint32_t rx_expected_traffic_missing = 0;

// =============================================================
// Helpers
// =============================================================

static inline bool is_valid_traffic(uint8_t b) {
  return
    b == TRAFFIC_DEBUG ||
    b == TRAFFIC_REQUEST_RESPONSE ||
    b == TRAFFIC_PUBLISH_SUBSCRIBE;
}

static inline void note_rr_drop(uint8_t traffic) {
  if (traffic == TRAFFIC_REQUEST_RESPONSE) {
    tx_rr_drop_count++;
  }
}

static size_t payload_serialization_bound(const Payload& payload) {
  size_t bound = (payload.arena_used() * 2) + (payload.count() * 32) + 64;
  if (bound < 256) bound = 256;
  if (bound > Payload::ARENA_MAX) bound = Payload::ARENA_MAX;
  return bound;
}

static size_t serial_write_bytes(const uint8_t* buf, size_t len) {
  if (!buf || len == 0) return 0;
  const size_t n = Serial.write(buf, len);
  Serial.flush();
  return n;
}

// =============================================================
// Registration
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
) {
  recv_cb[traffic] = cb;
}

// =============================================================
// TX Pump (TIMEPOP-driven, single writer)
// =============================================================

static void transport_tx_pump(timepop_ctx_t*, timepop_diag_t*, void*) {

  if (tx_job_count == 0) return;

  tx_job_t& job = tx_jobs[tx_job_tail];
  const size_t remaining = job.length - job.sent;
  const size_t written = serial_write_bytes(job.data + job.sent, remaining);

  if (written == 0) return;

  const size_t accepted = (written > remaining) ? remaining : written;
  job.sent += accepted;
  tx_bytes_sent += accepted;

  if (job.sent < job.length) return;

  const size_t alloc_length = job.alloc_length;
  free(job.data);
  job = tx_job_t{};

  tx_budget_used -= alloc_length;

  tx_job_tail = (tx_job_tail + 1) % TX_JOB_MAX;
  tx_job_count--;
  tx_jobs_sent++;
}

// =============================================================
// transport_send() — serialize directly into queued wire buffer
// =============================================================

void transport_send(uint8_t traffic, const Payload& payload) {

  if (!is_valid_traffic(traffic)) return;

  if (tx_job_count >= TX_JOB_MAX) {
    tx_queue_full++;
    note_rr_drop(traffic);
    return;
  }

  const size_t json_buf_size = payload_serialization_bound(payload);
  const size_t alloc_len =
      TX_TRAFFIC_BYTES + TX_HEADER_RESERVE + json_buf_size + ETX_LEN;

  if (tx_budget_used + alloc_len > TX_BUDGET_MAX) {
    tx_budget_fail++;
    note_rr_drop(traffic);
    return;
  }

  uint8_t* data = (uint8_t*)malloc(alloc_len);
  if (!data) {
    tx_alloc_fail++;
    note_rr_drop(traffic);
    return;
  }

  char* json_buf = reinterpret_cast<char*>(data + TX_TRAFFIC_BYTES + TX_HEADER_RESERVE);
  const size_t json_len = payload.write_json(json_buf, json_buf_size);

  if (json_len == 0 || json_len > TRANSPORT_MAX_MESSAGE) {
    free(data);
    tx_budget_fail++;
    note_rr_drop(traffic);
    return;
  }

  if (!payload.empty() && json_len == 2 && json_buf[0] == '{' && json_buf[1] == '}') {
    free(data);
    tx_budget_fail++;
    note_rr_drop(traffic);
    return;
  }

  char header[TX_HEADER_RESERVE];
  const int header_len = snprintf(header, sizeof(header),
                                  "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0 || (size_t)header_len >= sizeof(header)) {
    free(data);
    tx_budget_fail++;
    note_rr_drop(traffic);
    return;
  }

  const size_t wire_len =
      TX_TRAFFIC_BYTES + (size_t)header_len + json_len + ETX_LEN;
  if (wire_len > alloc_len) {
    free(data);
    tx_budget_fail++;
    note_rr_drop(traffic);
    return;
  }

  data[0] = traffic;
  memcpy(data + TX_TRAFFIC_BYTES, header, (size_t)header_len);
  memmove(data + TX_TRAFFIC_BYTES + (size_t)header_len, json_buf, json_len);
  memcpy(data + TX_TRAFFIC_BYTES + (size_t)header_len + json_len, ETX_SEQ, ETX_LEN);

  tx_jobs[tx_job_head] = {
    data,
    wire_len,
    alloc_len,
    0
  };

  tx_job_head = (tx_job_head + 1) % TX_JOB_MAX;
  tx_job_count++;
  tx_jobs_enqueued++;
  tx_bytes_enqueued += wire_len;

  tx_budget_used += alloc_len;
  if (tx_budget_used > tx_budget_high_water) tx_budget_high_water = tx_budget_used;
  if (tx_job_count > tx_job_high_water) tx_job_high_water = tx_job_count;
}

// =============================================================
// RX — Serial stream framing
// =============================================================

static void rx_reset_hard() {
  rx_reset_hard_count++;
  rx_len = 0;
  rx_have_traffic = false;
}

static void dispatch_if_complete() {

  if (!rx_have_traffic) {
    rx_len = 0;
    return;
  }

  if (rx_len < (STX_LEN + 2)) return;

  if (memcmp(rx_buf, STX_SEQ, STX_LEN) != 0) {
    rx_bad_stx_count++;
    rx_reset_hard();
    return;
  }

  size_t i = STX_LEN;
  size_t declared_len = 0;
  bool saw_digit = false;

  while (i < rx_len && rx_buf[i] != '>') {
    uint8_t c = rx_buf[i];
    if (c < '0' || c > '9') {
      rx_len_overflow_count++;
      rx_reset_hard();
      return;
    }
    saw_digit = true;
    declared_len = declared_len * 10 + (size_t)(c - '0');
    ++i;
  }

  if (!saw_digit || i >= rx_len) return;

  const size_t header_end = i;
  const size_t json_start = header_end + 1;
  const size_t required_total = json_start + declared_len + ETX_LEN;

  if (required_total > RX_BUF_MAX) {
    rx_len_overflow_count++;
    rx_reset_hard();
    return;
  }

  if (rx_len < required_total) return;

  const size_t etx_pos = json_start + declared_len;
  if (memcmp(rx_buf + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    rx_bad_etx_count++;
    rx_reset_hard();
    return;
  }

  rx_frames_complete++;

  Payload p;
  p.parseJSON(rx_buf + json_start, declared_len);

  if (recv_cb[rx_traffic]) {
    rx_frames_dispatched++;
    recv_cb[rx_traffic](p);
  }

  rx_reset_hard();
}

static void rx_serial_tick() {

  transport_rx_entered();

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (!rx_have_traffic) {
      if (!is_valid_traffic(b)) {
        rx_expected_traffic_missing++;
        continue;
      }
      rx_traffic = b;
      rx_have_traffic = true;
      rx_len = 0;
      continue;
    }

    if (is_valid_traffic(b)) {
      rx_overlap_count++;
    }

    if (rx_len >= RX_BUF_MAX) {
      rx_len_overflow_count++;
      rx_reset_hard();
      return;
    }

    rx_buf[rx_len++] = b;
    rx_bytes_total++;

    dispatch_if_complete();
  }
}

static void transport_rx_tick(timepop_ctx_t*, timepop_diag_t*, void*) {
  rx_serial_tick();
}

// =============================================================
// transport_get_info()
// =============================================================

void transport_get_info(transport_info_t* out) {

  if (!out) return;

  out->tx_budget_max        = TX_BUDGET_MAX;
  out->tx_budget_used       = tx_budget_used;
  out->tx_budget_high_water = tx_budget_high_water;

  out->tx_job_count         = tx_job_count;
  out->tx_job_high_water    = tx_job_high_water;

  out->tx_jobs_enqueued     = tx_jobs_enqueued;
  out->tx_jobs_sent         = tx_jobs_sent;

  out->tx_bytes_enqueued    = tx_bytes_enqueued;
  out->tx_bytes_sent        = tx_bytes_sent;

  out->tx_alloc_fail        = tx_alloc_fail;
  out->tx_budget_fail       = tx_budget_fail;
  out->tx_queue_full        = tx_queue_full;
  out->tx_rr_drop_count     = tx_rr_drop_count;

  out->rx_blocks_total              = rx_blocks_total;
  out->rx_bytes_total               = rx_bytes_total;
  out->rx_frames_complete           = rx_frames_complete;
  out->rx_frames_dispatched         = rx_frames_dispatched;
  out->rx_reset_hard                = rx_reset_hard_count;
  out->rx_bad_stx                   = rx_bad_stx_count;
  out->rx_bad_etx                   = rx_bad_etx_count;
  out->rx_len_overflow              = rx_len_overflow_count;
  out->rx_overlap                   = rx_overlap_count;
  out->rx_expected_traffic_missing  = rx_expected_traffic_missing;
}

// =============================================================
// Init
// =============================================================

void transport_init(void) {

  Serial.begin(USB_SERIAL_BAUD);

  debug_log("transport", BUILD_FINGERPRINT);

  timepop_arm(
    TRANSPORT_RX_POLL_NS,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );

  timepop_arm(
    TRANSPORT_TX_POLL_NS,
    true,
    transport_tx_pump,
    nullptr,
    "transport-tx"
  );
}
