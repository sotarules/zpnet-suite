// transport.cpp — ZPNet Transport (Teensy)
// ---------------------------------------
//
// TX Architecture (REVISED):
//   • transport_send() no longer writes to HID/Serial.
//   • It clones the payload and enqueues it into a TX arena.
//   • A single TIMEPOP-driven pump function despools jobs.
//   • Only the pump ever touches RawHID / Serial.
//   • Interleave is now structurally impossible.
//
// RX Architecture (UNCHANGED):
//   • Length-authoritative framing
//   • <STX=n> JSON <ETX>
//   • Hard reset on corruption
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

static constexpr size_t TRANSPORT_BLOCK_SIZE = 64;
static constexpr size_t RX_BUF_MAX           = 2048;

// TX arena size (bounded, deterministic memory)
// Adjust if needed. 12 KB is safe on Teensy 4.x.
static constexpr size_t TX_ARENA_SIZE = 48 * 1024;
static constexpr size_t TX_JOB_MAX    = 64;   // max queued messages

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

// =============================================================
// TX Arena (FIFO, zero-fragmentation)
// =============================================================
//
// The arena behaves like a circular buffer.
// Allocation and free must occur in FIFO order.
//
// Because TX jobs are strictly FIFO,
// fragmentation is impossible.
//

static uint8_t tx_arena[TX_ARENA_SIZE];

static size_t  tx_head = 0;  // next write position
static size_t  tx_tail = 0;  // next free position
static size_t  tx_used = 0;

static size_t  tx_arena_high_water = 0;

// Allocation failure counter
static volatile uint32_t tx_arena_alloc_fail = 0;

// =============================================================
// TX Job Queue (small descriptor ring)
// =============================================================

struct tx_job_t {
  uint8_t  traffic;
  size_t   start;     // offset into arena
  size_t   length;    // total bytes
  size_t   sent;      // bytes already sent
};

static tx_job_t tx_jobs[TX_JOB_MAX];

static size_t tx_job_head = 0;
static size_t tx_job_tail = 0;
static size_t tx_job_count = 0;


static size_t tx_job_high_water = 0;

static volatile uint32_t tx_jobs_enqueued = 0;
static volatile uint32_t tx_jobs_sent     = 0;
static volatile uint32_t tx_bytes_enqueued = 0;
static volatile uint32_t tx_bytes_sent     = 0;

static volatile uint32_t tx_rr_drop_count  = 0;

// =============================================================
// Arena state (additions)
// =============================================================

static volatile uint32_t tx_arena_gap_skips     = 0;  // gap-skip events
static volatile uint32_t tx_arena_gap_bytes_tot = 0;  // cumulative wasted bytes
static volatile uint32_t tx_arena_gap_bytes_max = 0;  // largest single gap

static size_t tx_gap_start = TX_ARENA_SIZE;           // "no gap" sentinel

static volatile uint32_t tx_debug_last_tail_at_gap_check = 0; // debug
static volatile size_t tx_debug_tail_before_free = 0; // debug

// =============================================================
// RX State (unchanged)
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic      = 0;

// RX counters (unchanged)
static volatile uint32_t rx_blocks_total        = 0;
static volatile uint32_t rx_bytes_total         = 0;
static volatile uint32_t rx_frames_complete     = 0;
static volatile uint32_t rx_frames_dispatched   = 0;
static volatile uint32_t rx_reset_hard_count    = 0;
static volatile uint32_t rx_bad_stx_count       = 0;
static volatile uint32_t rx_bad_etx_count       = 0;
static volatile uint32_t rx_len_overflow_count  = 0;
static volatile uint32_t rx_overlap_count       = 0;
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
// Arena Allocation (FIFO ring, with tail-gap skip)
// =============================================================
//
// When a message won't fit in the contiguous tail space but
// WOULD fit if we wrapped to offset 0, we skip the unusable
// tail bytes ("dead space") and allocate from the beginning.
//
// The dead space is accounted for in tx_used so that the
// arena's free-space bookkeeping remains correct. The dead
// bytes are reclaimed when tx_arena_free() advances tx_tail
// past them.
//
// Invariant: at most one outstanding gap exists at any time.
// This is guaranteed because gap creation requires head >= tail,
// and after wrapping head to 0 we enter head < tail until the
// tail catches up.
//

static bool tx_arena_alloc(size_t len, size_t* out_start) {

  // Reject obviously impossible requests
  if (len == 0 || len > TX_ARENA_SIZE) {
    tx_arena_alloc_fail++;
    return false;
  }

  // Not enough total free space regardless of layout
  if (len > TX_ARENA_SIZE - tx_used) {
    tx_arena_alloc_fail++;
    return false;
  }

  // Case 1: head >= tail (free space is at end + beginning)
  if (tx_head >= tx_tail) {

    size_t space_end = TX_ARENA_SIZE - tx_head;

    if (len <= space_end) {
      // Fits at the end — simple case
      *out_start = tx_head;
      tx_head = (tx_head + len) % TX_ARENA_SIZE;
    } else {
      // Won't fit at end. Skip the tail gap and wrap to 0.

      size_t dead = space_end;

      // After wasting the gap, is there still room at the front?
      if (len > TX_ARENA_SIZE - tx_used - dead) {
        tx_arena_alloc_fail++;
        return false;
      }

      // Record where the gap begins
      tx_gap_start = tx_head;

      // Account for dead space
      tx_used += dead;
      if (tx_used > tx_arena_high_water)
        tx_arena_high_water = tx_used;

      // Gap-skip telemetry
      tx_arena_gap_skips++;
      tx_arena_gap_bytes_tot += dead;
      if (dead > tx_arena_gap_bytes_max)
        tx_arena_gap_bytes_max = dead;

      // Allocate from offset 0
      *out_start = 0;
      tx_head = len;
    }

  } else {
    // Case 2: head < tail (free space is contiguous between them)
    size_t space_mid = tx_tail - tx_head;

    if (len <= space_mid) {
      *out_start = tx_head;
      tx_head += len;
    } else {
      tx_arena_alloc_fail++;
      return false;
    }
  }

  tx_used += len;
  if (tx_used > tx_arena_high_water)
    tx_arena_high_water = tx_used;

  return true;
}
// =============================================================
// Arena Free (FIFO, with gap reclamation)
// =============================================================
//
// Jobs are freed in strict FIFO order. When the tail reaches
// the dead gap at the end of the arena, the gap bytes are
// reclaimed and the tail wraps to 0.
//

static void tx_arena_free(size_t len) {

  if (tx_gap_start != TX_ARENA_SIZE) {
    tx_debug_tail_before_free = tx_tail;
  }

  tx_tail += len;
  tx_used -= len;

  // If the tail has entered the dead gap region, skip to 0
  if (tx_gap_start != TX_ARENA_SIZE && tx_tail >= tx_gap_start) {
    size_t dead = TX_ARENA_SIZE - tx_gap_start;
    tx_used -= dead;
    tx_tail = 0;
    tx_gap_start = TX_ARENA_SIZE;
  }

  // Normal wrap (no gap case)
  if (tx_tail >= TX_ARENA_SIZE) {
    tx_tail -= TX_ARENA_SIZE;
  }
}

// =============================================================
// Physical Send (unchanged semantics)
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
// Registration (shared)
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
    memcpy(block + 1, tx_arena + job.start, chunk);
    memset(block + 1 + chunk, 0, TRANSPORT_BLOCK_SIZE - (1 + chunk));
    hid_send_block(block);
    job.sent += chunk;
    tx_bytes_sent += chunk;
  } else {
    size_t chunk = min(remaining, TRANSPORT_BLOCK_SIZE);
    memcpy(block, tx_arena + job.start + job.sent, chunk);
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
  size_t chunk = remaining;  // serial sends whole remaining
  serial_send_bytes(tx_arena + job.start + job.sent, chunk);
  job.sent += chunk;
  tx_bytes_sent += chunk;

#endif

  // Job complete?
  if (job.sent >= job.length) {
    tx_arena_free(job.length);
    tx_job_tail = (tx_job_tail + 1) % TX_JOB_MAX;
    tx_job_count--;
    tx_jobs_sent++;
  }
}

// =============================================================
// transport_send() — now enqueue only
// =============================================================

void transport_send(uint8_t traffic, const Payload& payload) {

    char local_buf[2048];
    size_t json_len = payload.write_json(local_buf, sizeof(local_buf));

    if (json_len == 0)
        return;

    char header[32];
    int header_len = snprintf(header, sizeof(header),
                              "<STX=%u>", (unsigned)json_len);
    if (header_len <= 0)
        return;

    size_t total_len = header_len + json_len + ETX_LEN;

    size_t start;
    if (!tx_arena_alloc(total_len, &start)) {
        if (traffic == TRAFFIC_REQUEST_RESPONSE) {
            tx_rr_drop_count++;
        }
        return;
    }

    memcpy(tx_arena + start, header, header_len);
    memcpy(tx_arena + start + header_len, local_buf, json_len);
    memcpy(tx_arena + start + header_len + json_len, ETX_SEQ, ETX_LEN);

    if (tx_job_count >= TX_JOB_MAX) {
        tx_arena_free(total_len);
        if (traffic == TRAFFIC_REQUEST_RESPONSE) {
            tx_rr_drop_count++;
        }
        return;
    }

    tx_jobs[tx_job_head] = {
        traffic,
        start,
        total_len,
        0
    };

    tx_job_head = (tx_job_head + 1) % TX_JOB_MAX;
    tx_job_count++;
    tx_jobs_enqueued++;
    tx_bytes_enqueued += total_len;

    if (tx_job_count > tx_job_high_water)
        tx_job_high_water = tx_job_count;
}

// =============================================================
// RX (unchanged from your version)
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
      rx_len = 0;  // explicit reset: beginning of a new message
      continue;
    }

    // Optional overlap signal:
    // JSON is ASCII; a traffic byte (0xD0..0xD2) here strongly suggests
    // a new frame arrived before the previous completed.
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
// transport_get_info() — expanded
// =============================================================

void transport_get_info(transport_info_t* out) {

  if (!out) return;

  out->tx_arena_size       = TX_ARENA_SIZE;
  out->tx_arena_used       = tx_used;
  out->tx_arena_high_water = tx_arena_high_water;
  out->tx_arena_alloc_fail = tx_arena_alloc_fail;

  out->tx_job_count        = tx_job_count;
  out->tx_job_high_water   = tx_job_high_water;
  out->tx_jobs_enqueued    = tx_jobs_enqueued;
  out->tx_jobs_sent        = tx_jobs_sent;
  out->tx_bytes_enqueued   = tx_bytes_enqueued;
  out->tx_bytes_sent       = tx_bytes_sent;
  out->tx_rr_drop_count    = tx_rr_drop_count;

  out->tx_gap_start         = tx_gap_start;
  out->tx_arena_gap_skips  = tx_arena_gap_skips;
  out->tx_arena_gap_bytes_tot = tx_arena_gap_bytes_tot;
  out->tx_arena_gap_bytes_max = tx_arena_gap_bytes_max;
  out->tx_debug_last_tail_at_gap_check = tx_debug_last_tail_at_gap_check;
  out->tx_debug_tail_before_free  = tx_debug_tail_before_free ;

  out->rx_blocks_total     = rx_blocks_total;
  out->rx_bytes_total      = rx_bytes_total;
  out->rx_frames_complete  = rx_frames_complete;
  out->rx_frames_dispatched = rx_frames_dispatched;
  out->rx_reset_hard       = rx_reset_hard_count;
  out->rx_bad_stx          = rx_bad_stx_count;
  out->rx_bad_etx          = rx_bad_etx_count;
  out->rx_len_overflow     = rx_len_overflow_count;
  out->rx_overlap          = rx_overlap_count;
  out->rx_expected_traffic_missing = rx_expected_traffic_missing;
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
    TIMEPOP_CLASS_RX_POLL,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );

  timepop_arm(
    TIMEPOP_CLASS_TX_PUMP,
    true,
    transport_tx_pump,
    nullptr,
    "transport-tx"
  );
}
