// transport.cpp — ZPNet Transport (Teensy)
// -----------------------------------------
//
// TX Architecture:
//   • transport_send() serializes, heap-allocates, and enqueues.
//   • A single TIMEPOP-driven pump performs physical transmission.
//   • Only the pump ever touches RawHID / Serial.
//   • Interleave is structurally impossible.
//   • Each job owns its own heap allocation — no ring, no gap.
//
// RX Architecture (UNCHANGED):
//   • Length-authoritative framing
//   • <STX=n> JSON <ETX>
//   • Hard reset on corruption
//
// Memory model:
//   • TX memory is bounded by TX_BUDGET_MAX (soft cap).
//   • Each message is exactly the size it needs to be.
//   • No geometric fragmentation. No dead space. No wrap logic.
//   • Failure mode is simple: malloc succeeds or it doesn't.
//
// This file is intentionally verbose and explicit.
// Determinism > cleverness.
//

#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "process_performance.h"

#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// =============================================================
// Constants
// =============================================================

static constexpr uint64_t TRANSPORT_RX_POLL_NS = 1000000ULL;  // 1 millisecond
static constexpr uint64_t TRANSPORT_TX_POLL_NS = 1000000ULL;  // 1 millisecond

static constexpr size_t TRANSPORT_BLOCK_SIZE = 64;
static constexpr size_t RX_BUF_MAX           = 2048;

// TX job queue depth
static constexpr size_t TX_JOB_MAX = 64;

// TX memory budget (soft cap, bytes)
// Replaces the fixed arena. Enforces bounded memory usage
// without imposing geometric constraints.
static constexpr size_t TX_BUDGET_MAX = 48 * 1024;

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

// =============================================================
// TX Job Queue
// =============================================================
//
// Each job owns a heap-allocated buffer containing a fully
// framed message (STX + JSON + ETX). The pump sends from
// this buffer and frees it on completion.
//
// The job descriptors form a simple ring for FIFO ordering.
// The memory management is trivial: malloc on enqueue,
// free on completion.
//

struct tx_job_t {
  uint8_t  traffic;
  uint8_t* data;       // heap-allocated, owned by this job
  size_t   length;     // total bytes in data[]
  size_t   sent;       // bytes already transmitted
};

static tx_job_t tx_jobs[TX_JOB_MAX];

static size_t tx_job_head  = 0;
static size_t tx_job_tail  = 0;
static size_t tx_job_count = 0;

// =============================================================
// TX Counters & Budget Tracking
// =============================================================

// Memory budget (current outstanding bytes across all jobs)
static size_t tx_budget_used = 0;

// High-water marks
static size_t tx_budget_high_water = 0;
static size_t tx_job_high_water    = 0;

// Lifetime counters (monotonic)
static volatile uint32_t tx_jobs_enqueued  = 0;
static volatile uint32_t tx_jobs_sent      = 0;
static volatile uint32_t tx_bytes_enqueued = 0;
static volatile uint32_t tx_bytes_sent     = 0;

// Failure counters
static volatile uint32_t tx_alloc_fail     = 0;  // malloc returned null
static volatile uint32_t tx_budget_fail    = 0;  // budget cap exceeded
static volatile uint32_t tx_queue_full     = 0;  // job ring full
static volatile uint32_t tx_rr_drop_count  = 0;  // RR messages dropped (any reason)

// =============================================================
// RX State
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic      = 0;

// RX counters
static volatile uint32_t rx_blocks_total             = 0;
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
// Traffic Validation
// =============================================================

static inline bool is_valid_traffic(uint8_t b) {
  return
    b == TRAFFIC_DEBUG ||
    b == TRAFFIC_REQUEST_RESPONSE ||
    b == TRAFFIC_PUBLISH_SUBSCRIBE;
}

// =============================================================
// Transport fingerprint
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
static const char* BUILD_FINGERPRINT = "__FP__ZPNET_HID__";
#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
static const char* BUILD_FINGERPRINT = "__FP__ZPNET_SERIAL__";
#endif

// =============================================================
// Physical Send
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

static inline bool hid_send_block(const uint8_t block[TRANSPORT_BLOCK_SIZE]) {
  return RawHID.send(block, 50) > 0;
}

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

static inline bool serial_send_bytes(const uint8_t* buf, size_t len) {
  size_t n = Serial.write(buf, len);
  Serial.flush();
  return n == len;
}

#endif

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

static void transport_tx_pump(timepop_ctx_t*, void*) {

  if (tx_job_count == 0)
    return;

  tx_job_t& job = tx_jobs[tx_job_tail];

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

  uint8_t block[TRANSPORT_BLOCK_SIZE];
  size_t remaining = job.length - job.sent;

  if (job.sent == 0) {
    // First block carries traffic byte
    block[0] = job.traffic;
    size_t chunk = min(remaining, TRANSPORT_BLOCK_SIZE - 1);
    memcpy(block + 1, job.data, chunk);
    memset(block + 1 + chunk, 0, TRANSPORT_BLOCK_SIZE - (1 + chunk));
    hid_send_block(block);
    job.sent += chunk;
    tx_bytes_sent += chunk;
  } else {
    size_t chunk = min(remaining, TRANSPORT_BLOCK_SIZE);
    memcpy(block, job.data + job.sent, chunk);
    memset(block + chunk, 0, TRANSPORT_BLOCK_SIZE - chunk);
    hid_send_block(block);
    job.sent += chunk;
    tx_bytes_sent += chunk;
  }

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

  if (job.sent == 0) {
    uint8_t header = job.traffic;
    serial_send_bytes(&header, 1);
  }

  size_t remaining = job.length - job.sent;
  size_t chunk = remaining;
  serial_send_bytes(job.data + job.sent, chunk);
  job.sent += chunk;
  tx_bytes_sent += chunk;

#endif

  // Job complete?
  if (job.sent >= job.length) {

    // Release heap allocation
    free(job.data);
    job.data = nullptr;

    // Release budget
    tx_budget_used -= job.length;

    // Advance queue
    tx_job_tail = (tx_job_tail + 1) % TX_JOB_MAX;
    tx_job_count--;
    tx_jobs_sent++;
  }
}

// =============================================================
// transport_send() — serialize, allocate, enqueue
// =============================================================

void transport_send(uint8_t traffic, const Payload& payload) {

  // --------------------------------------------------------
  // 1. Serialize to stack buffer
  // --------------------------------------------------------

  char local_buf[2048];
  size_t json_len = payload.write_json(local_buf, sizeof(local_buf));

  if (json_len == 0)
    return;

  // --------------------------------------------------------
  // 2. Compute framed message size
  // --------------------------------------------------------

  char header[32];
  int header_len = snprintf(header, sizeof(header),
                            "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0)
    return;

  size_t total_len = (size_t)header_len + json_len + ETX_LEN;

  // --------------------------------------------------------
  // 3. Budget check (soft cap)
  // --------------------------------------------------------

  if (tx_budget_used + total_len > TX_BUDGET_MAX) {
    tx_budget_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  // --------------------------------------------------------
  // 4. Job queue check
  // --------------------------------------------------------

  if (tx_job_count >= TX_JOB_MAX) {
    tx_queue_full++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  // --------------------------------------------------------
  // 5. Heap allocate
  // --------------------------------------------------------

  uint8_t* data = (uint8_t*)malloc(total_len);

  if (!data) {
    tx_alloc_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  // --------------------------------------------------------
  // 6. Assemble framed message
  // --------------------------------------------------------

  memcpy(data, header, header_len);
  memcpy(data + header_len, local_buf, json_len);
  memcpy(data + header_len + json_len, ETX_SEQ, ETX_LEN);

  // --------------------------------------------------------
  // 7. Enqueue
  // --------------------------------------------------------

  tx_jobs[tx_job_head] = {
    traffic,
    data,
    total_len,
    0
  };

  tx_job_head = (tx_job_head + 1) % TX_JOB_MAX;
  tx_job_count++;
  tx_jobs_enqueued++;
  tx_bytes_enqueued += total_len;

  // --------------------------------------------------------
  // 8. Budget + high-water tracking
  // --------------------------------------------------------

  tx_budget_used += total_len;

  if (tx_budget_used > tx_budget_high_water)
    tx_budget_high_water = tx_budget_used;

  if (tx_job_count > tx_job_high_water)
    tx_job_high_water = tx_job_count;
}

// =============================================================
// RX — unchanged
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

  if (rx_len < (STX_LEN + 2))
    return;

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

  if (!saw_digit || i >= rx_len)
    return;

  size_t header_end = i;
  size_t json_start = header_end + 1;
  size_t required_total = json_start + declared_len + ETX_LEN;

  if (required_total > RX_BUF_MAX) {
    rx_len_overflow_count++;
    rx_reset_hard();
    return;
  }

  if (rx_len < required_total)
    return;

  size_t etx_pos = json_start + declared_len;
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

// =============================================================
// RECEIVE: HID RX (record-based)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
static void rx_hid_tick() {

  transport_rx_entered();

  uint8_t block[TRANSPORT_BLOCK_SIZE];
  if (RawHID.recv(block, 0) <= 0)
    return;

  rx_blocks_total++;

  size_t offset = 0;

  if (!rx_have_traffic) {
    if (!is_valid_traffic(block[0])) {
      rx_expected_traffic_missing++;
      rx_reset_hard();
      return;
    }
    rx_traffic = block[0];
    rx_have_traffic = true;
    offset = 1;
  }

  size_t copy = TRANSPORT_BLOCK_SIZE - offset;
  rx_bytes_total += copy;

  if (rx_len + copy > RX_BUF_MAX) {
    rx_reset_hard();
    return;
  }

  memcpy(rx_buf + rx_len, block + offset, copy);
  rx_len += copy;

  dispatch_if_complete();
}
#endif

// =============================================================
// RECEIVE: SERIAL RX (stream-based)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

static void rx_serial_tick() {

  transport_rx_entered();

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    // Await traffic byte
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

    // Overlap signal
    if (is_valid_traffic(b)) {
      rx_overlap_count++;
    }

    // Accumulate payload bytes
    if (rx_len >= RX_BUF_MAX) {
      rx_reset_hard();
      return;
    }

    rx_buf[rx_len++] = b;
    rx_bytes_total++;

    dispatch_if_complete();
  }
}

#endif

// =============================================================
// RX Poll (timepop)
// =============================================================

static void transport_rx_tick(timepop_ctx_t*, void*) {

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
  rx_hid_tick();
#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  rx_serial_tick();
#endif

}

// =============================================================
// transport_get_info()
// =============================================================

void transport_get_info(transport_info_t* out) {

  if (!out) return;

  // TX — Budget / Queue health
  out->tx_budget_max        = TX_BUDGET_MAX;
  out->tx_budget_used       = tx_budget_used;
  out->tx_budget_high_water = tx_budget_high_water;

  out->tx_job_count         = tx_job_count;
  out->tx_job_high_water    = tx_job_high_water;

  out->tx_jobs_enqueued     = tx_jobs_enqueued;
  out->tx_jobs_sent         = tx_jobs_sent;

  out->tx_bytes_enqueued    = tx_bytes_enqueued;
  out->tx_bytes_sent        = tx_bytes_sent;

  // TX — Failure counters
  out->tx_alloc_fail        = tx_alloc_fail;
  out->tx_budget_fail       = tx_budget_fail;
  out->tx_queue_full        = tx_queue_full;
  out->tx_rr_drop_count     = tx_rr_drop_count;

  // RX
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

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  Serial.begin(115200);
#endif

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