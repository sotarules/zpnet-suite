// transport.cpp — ZPNet Transport (Teensy, USB CDC serial only)
// ----------------------------------------------------------------
//
// Wire protocol:
//   [traffic byte] <STX=json_len> JSON <ETX>
//
// TX Architecture:
//   • transport_send() serializes one semantic payload into one complete
//     wire image, including the traffic byte.
//   • A recurring TimePop callback performs bounded physical transmission.
//   • The pump advances by the number of bytes actually accepted by Serial.
//   • Only transport ever touches Serial for TX.
//   • Interleave is structurally impossible.
//   • TimePop is the sole runtime custody path for physical TX and RX service.
//
// RX Architecture:
//   • Stream parser, length-authoritative framing.
//   • Traffic byte selects semantic channel.
//   • <STX=n> supplies the exact JSON byte count.
//   • <ETX> is retained as a redundant delimiter/sanity check.
//   • If a new traffic byte appears before a valid STX prefix, RX resyncs
//     to the new traffic byte instead of poisoning the next frame.
//
// Serial-only doctrine:
//   • HID is retired.
//   • There is no compile-time or runtime backend selection.
//   • No 64-byte HID blocking/deblocking layer remains.
//
// This file is intentionally boring.
// Determinism > cleverness.
//

#include "transport.h"
#include "config.h"
#include "debug.h"
#include "process_performance.h"
#include "timepop.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// Constants
// =============================================================

static constexpr uint64_t TRANSPORT_SERVICE_PERIOD_NS = 2000000ULL;  // 2 ms
static constexpr size_t TRANSPORT_RX_QUANTUM_MAX = 512U;
static constexpr size_t TRANSPORT_TX_QUANTUM_MAX = 1024U;

// USB CDC attach/reboot can deliver a tail of a Pi frame after the traffic byte
// was lost at the host/device boundary.  Until the first lawful incoming frame
// proves alignment, quarantine startup artifacts into segregated counters.
static constexpr uint32_t TRANSPORT_RX_STARTUP_GRACE_MS = 15000UL;

static constexpr size_t FRAME_SLACK = 64;
static constexpr size_t RX_BUF_MAX = TRANSPORT_MAX_MESSAGE + FRAME_SLACK;

// RAM2 placement experiment.  The RX body is CPU-written/CPU-read, but the
// earlier bare-DMAMEM experiment produced message corruption.  Fence the
// payload with dedicated cache lines, poison it at initialization, and retain
// first-failure evidence so this trial can distinguish placement from an
// overrun/overread or neighboring cache-maintenance problem.
static constexpr size_t   RX_CACHE_LINE_BYTES = 32U;
static constexpr size_t   RX_GUARD_WORDS =
    RX_CACHE_LINE_BYTES / sizeof(uint32_t);
static constexpr uint8_t  RX_POISON_BYTE = 0xA5U;
static constexpr uint32_t RX_GUARD_BEFORE_WORD = 0x52584742UL;  // 'RXGB'
static constexpr uint32_t RX_GUARD_AFTER_WORD  = 0x52584741UL;  // 'RXGA'

static_assert((RX_BUF_MAX % RX_CACHE_LINE_BYTES) == 0U,
              "RX buffer must occupy complete cache lines");

static constexpr size_t TX_JOB_MAX = 64;
static constexpr size_t TX_BUDGET_MAX = 48 * 1024;

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

static const char* BUILD_FINGERPRINT =
    "__FP__ZPNET_SERIAL_CANONICAL_WIRE_RX_DMAMEM_GUARDED__";
static const char* TRANSPORT_LIFELINE_FINGERPRINT = "__FP__ZPNET_TRANSPORT_TIMEPOP_RX_ALAP__";

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
// TX Job Queue
// =============================================================
//
// Each job owns one complete wire image:
//
//   data[0]      = traffic byte
//   data[1..N]   = <STX=json_len> JSON <ETX>
//
// sent is a byte offset into that same wire image. Partial USB CDC writes are
// therefore harmless: the next pump resumes at exactly the byte that did not
// cross the physical boundary yet.
//

struct tx_job_t {
  uint8_t* data;       // heap-allocated complete wire image
  size_t   length;     // total bytes in data[]
  size_t   sent;       // bytes actually accepted by Serial
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
static volatile uint32_t tx_bytes_enqueued = 0;  // complete wire bytes
static volatile uint32_t tx_bytes_sent     = 0;  // complete wire bytes accepted by Serial

static volatile uint32_t tx_alloc_fail     = 0;
static volatile uint32_t tx_budget_fail    = 0;
static volatile uint32_t tx_queue_full     = 0;
static volatile uint32_t tx_rr_drop_count  = 0;

// =============================================================
// Scheduled service counters
// =============================================================

static volatile uint32_t transport_poll_count = 0;
static volatile uint32_t transport_poll_skipped_too_soon = 0;
static volatile uint32_t transport_rx_poll_count = 0;
static volatile uint32_t transport_tx_poll_count = 0;
static volatile uint32_t transport_runtime_loop_count = 0;


// =============================================================
// RX State
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

struct alignas(RX_CACHE_LINE_BYTES) rx_storage_t {
  uint32_t guard_before[RX_GUARD_WORDS];
  uint8_t  buffer[RX_BUF_MAX];
  uint32_t guard_after[RX_GUARD_WORDS];
};

static_assert(offsetof(rx_storage_t, buffer) == RX_CACHE_LINE_BYTES,
              "RX buffer must begin after one guard cache line");
static_assert((offsetof(rx_storage_t, guard_after) % RX_CACHE_LINE_BYTES) == 0U,
              "RX trailing guard must begin on a cache-line boundary");
static_assert((sizeof(rx_storage_t) % RX_CACHE_LINE_BYTES) == 0U,
              "RX storage must occupy complete cache lines");

// Deliberately uninitialized by startup: transport_init() establishes the
// guard and poison patterns before the first receive poll.
static rx_storage_t g_rx_storage DMAMEM;

static inline uint8_t* rx_buffer(void) {
  return g_rx_storage.buffer;
}

static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic      = 0;

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

// One complete parsed frame may wait here between the bounded timed RX service
// and ordinary ALAP semantic dispatch.  Payload move assignment transfers owned
// storage; the mailbox never borrows from the reusable RX byte buffer.
static bool    rx_dispatch_pending = false;
static uint8_t rx_dispatch_traffic = 0;
static Payload rx_dispatch_payload;
static volatile uint32_t rx_dispatch_alap_arm_fail = 0;

enum class rx_guard_stage_t : uint32_t {
  NONE          = 0U,
  BEFORE_PARSE  = 1U,
  AFTER_PARSE   = 2U,
  AFTER_DISPATCH = 3U,
};

static volatile uint32_t rx_storage_init_count = 0;
static volatile uint32_t rx_json_terminator_count = 0;
static volatile uint32_t rx_guard_check_count = 0;
static volatile uint32_t rx_guard_failure_count = 0;
static volatile uint32_t rx_guard_before_failure_count = 0;
static volatile uint32_t rx_guard_after_failure_count = 0;
static volatile uint32_t rx_guard_last_stage = 0;
static volatile uint32_t rx_guard_last_side = 0;  // 1=before, 2=after
static volatile uint32_t rx_guard_last_index = 0xFFFFFFFFUL;
static volatile uint32_t rx_guard_last_expected = 0;
static volatile uint32_t rx_guard_last_observed = 0;

static uint32_t rx_init_ms = 0;
static bool     rx_first_frame_seen = false;
static volatile uint32_t rx_startup_expected_traffic_missing_suppressed = 0;
static volatile uint32_t rx_startup_bad_stx_suppressed = 0;
static volatile uint32_t rx_startup_bad_etx_suppressed = 0;
static volatile uint32_t rx_startup_len_overflow_suppressed = 0;
static volatile uint32_t rx_startup_first_missing_byte = 0xFFFFFFFFUL;
static volatile uint32_t rx_startup_last_missing_byte = 0;

static bool rx_startup_grace_active() {
  if (rx_first_frame_seen) return false;
  return (uint32_t)(millis() - rx_init_ms) < TRANSPORT_RX_STARTUP_GRACE_MS;
}

static void rx_startup_note_missing_byte(uint8_t b) {
  rx_startup_expected_traffic_missing_suppressed++;
  if (rx_startup_first_missing_byte == 0xFFFFFFFFUL) {
    rx_startup_first_missing_byte = b;
  }
  rx_startup_last_missing_byte = b;
}

static void rx_storage_initialize(void) {
  for (size_t i = 0; i < RX_GUARD_WORDS; ++i) {
    g_rx_storage.guard_before[i] = RX_GUARD_BEFORE_WORD;
    g_rx_storage.guard_after[i] = RX_GUARD_AFTER_WORD;
  }
  memset(rx_buffer(), RX_POISON_BYTE, RX_BUF_MAX);
  rx_storage_init_count++;
}

static void rx_guard_note_failure(rx_guard_stage_t stage,
                                  uint32_t side,
                                  uint32_t index,
                                  uint32_t expected,
                                  uint32_t observed) {
  rx_guard_last_stage = (uint32_t)stage;
  rx_guard_last_side = side;
  rx_guard_last_index = index;
  rx_guard_last_expected = expected;
  rx_guard_last_observed = observed;

  char line[192];
  snprintf(line, sizeof(line),
           "RX_GUARD stage=%lu side=%lu index=%lu expected=0x%08lX "
           "observed=0x%08lX rx_len=%lu buffer=0x%08lX",
           (unsigned long)stage,
           (unsigned long)side,
           (unsigned long)index,
           (unsigned long)expected,
           (unsigned long)observed,
           (unsigned long)rx_len,
           (unsigned long)(uintptr_t)rx_buffer());
  debug_log("transport.rx_guard", line);
}

static bool rx_guard_check(rx_guard_stage_t stage) {
  rx_guard_check_count++;

  bool ok = true;
  bool recorded = false;
  uint32_t before_bad = 0U;
  uint32_t after_bad = 0U;

  for (size_t i = 0; i < RX_GUARD_WORDS; ++i) {
    const uint32_t observed = g_rx_storage.guard_before[i];
    if (observed == RX_GUARD_BEFORE_WORD) continue;
    ok = false;
    before_bad++;
    if (!recorded) {
      rx_guard_note_failure(stage, 1U, (uint32_t)i,
                            RX_GUARD_BEFORE_WORD, observed);
      recorded = true;
    }
  }

  for (size_t i = 0; i < RX_GUARD_WORDS; ++i) {
    const uint32_t observed = g_rx_storage.guard_after[i];
    if (observed == RX_GUARD_AFTER_WORD) continue;
    ok = false;
    after_bad++;
    if (!recorded) {
      rx_guard_note_failure(stage, 2U, (uint32_t)i,
                            RX_GUARD_AFTER_WORD, observed);
      recorded = true;
    }
  }

  if (!ok) {
    rx_guard_failure_count++;
    rx_guard_before_failure_count += before_bad;
    rx_guard_after_failure_count += after_bad;
  }
  return ok;
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
// Physical Send
// =============================================================

static inline size_t serial_write_some(const uint8_t* buf, size_t len) {
  if (len == 0)
    return 0;

  size_t n = ZPNET_SERIAL.write(buf, len);

  if (n > len)
    return len;

  return n;
}

// =============================================================
// TX Pump (TimePop scheduled, single writer)
// =============================================================

static void tx_release_current_job() {
  tx_job_t& job = tx_jobs[tx_job_tail];

  free(job.data);
  job.data = nullptr;

  tx_budget_used -= job.length;

  tx_job_tail = (tx_job_tail + 1) % TX_JOB_MAX;
  tx_job_count--;
  tx_jobs_sent++;
}

static void tx_pump_once() {

  if (tx_job_count == 0)
    return;

  tx_job_t& job = tx_jobs[tx_job_tail];

  const size_t remaining = job.length - job.sent;
  const int available = ZPNET_SERIAL.availableForWrite();

  if (available <= 0)
    return;

  size_t quantum = remaining;
  if (quantum > (size_t)available)
    quantum = (size_t)available;
  if (quantum > TRANSPORT_TX_QUANTUM_MAX)
    quantum = TRANSPORT_TX_QUANTUM_MAX;

  const size_t n = serial_write_some(job.data + job.sent, quantum);

  if (n == 0)
    return;

  job.sent += n;
  tx_bytes_sent += n;

  if (job.sent >= job.length) {
    tx_release_current_job();
  }

}

// =============================================================
// transport_send() — serialize, allocate complete wire image, enqueue
// =============================================================

void transport_send(uint8_t traffic, const Payload& payload) {

  String json = payload.to_json();
  if (json.length() == 0) {
    return;
  }

  if (!payload.empty() && json == "{}") {
    return;
  }

  const size_t json_len = json.length();
  if (json_len > TRANSPORT_MAX_MESSAGE) {
    tx_budget_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  char header[32];
  int header_len = snprintf(header, sizeof(header),
                            "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0 || (size_t)header_len >= sizeof(header)) {
    tx_budget_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  const size_t wire_len =
    1 +                       // traffic byte
    (size_t)header_len +
    json_len +
    ETX_LEN;

  if (tx_budget_used + wire_len > TX_BUDGET_MAX) {
    tx_budget_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  if (tx_job_count >= TX_JOB_MAX) {
    tx_queue_full++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  uint8_t* data = (uint8_t*)malloc(wire_len);

  if (!data) {
    tx_alloc_fail++;
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return;
  }

  size_t pos = 0;
  data[pos++] = traffic;
  memcpy(data + pos, header, header_len);
  pos += (size_t)header_len;
  memcpy(data + pos, json.c_str(), json_len);
  pos += json_len;
  memcpy(data + pos, ETX_SEQ, ETX_LEN);

  tx_jobs[tx_job_head] = {
    data,
    wire_len,
    0
  };

  tx_job_head = (tx_job_head + 1) % TX_JOB_MAX;
  tx_job_count++;
  tx_jobs_enqueued++;
  tx_bytes_enqueued += wire_len;

  tx_budget_used += wire_len;

  if (tx_budget_used > tx_budget_high_water)
    tx_budget_high_water = tx_budget_used;

  if (tx_job_count > tx_job_high_water)
    tx_job_high_water = tx_job_count;
}

// =============================================================
// RX Helpers
// =============================================================

static void rx_begin(uint8_t traffic) {
  rx_traffic = traffic;
  rx_have_traffic = true;
  rx_len = 0;
}

static void rx_reset_hard() {
  rx_reset_hard_count++;
  rx_len = 0;
  rx_have_traffic = false;
  rx_traffic = 0;
}

static inline bool rx_stx_accepts(uint8_t b) {
  if (rx_len >= STX_LEN)
    return true;

  for (size_t i = 0; i < rx_len; ++i) {
    if (rx_buffer()[i] != (uint8_t)STX_SEQ[i])
      return false;
  }

  return b == (uint8_t)STX_SEQ[rx_len];
}

static inline bool rx_header_complete() {
  if (rx_len <= STX_LEN)
    return false;

  for (size_t i = STX_LEN; i < rx_len; ++i) {
    if (rx_buffer()[i] == (uint8_t)'>')
      return true;
  }

  return false;
}

static inline bool rx_should_resync(uint8_t b) {
  if (!is_valid_traffic(b))
    return false;

  if (rx_len == 0)
    return true;

  if (rx_header_complete())
    return false;

  if (rx_len < STX_LEN)
    return !rx_stx_accepts(b);

  // We have <STX= but not the closing '>' yet. A traffic byte cannot be a
  // legal decimal length byte, so it is a fresh frame boundary.
  return true;
}

static void rx_dispatch_alap(
  timepop_ctx_t*,
  timepop_diag_t*,
  void*
) {
  const uint8_t traffic = rx_dispatch_traffic;
  rx_dispatch_pending = false;

  if (recv_cb[traffic]) {
    rx_frames_dispatched++;
    recv_cb[traffic](rx_dispatch_payload);
  }

  rx_dispatch_payload.clear();

  if (!rx_guard_check(rx_guard_stage_t::AFTER_DISPATCH)) {
    rx_reset_hard();
  }
}

static bool dispatch_if_complete() {

  if (!rx_have_traffic) {
    rx_len = 0;
    return false;
  }

  if (rx_len < STX_LEN)
    return false;

  if (memcmp(rx_buffer(), STX_SEQ, STX_LEN) != 0) {
    if (rx_startup_grace_active()) {
      rx_startup_bad_stx_suppressed++;
    } else {
      rx_bad_stx_count++;
    }
    rx_reset_hard();
    return false;
  }

  size_t i = STX_LEN;
  size_t declared_len = 0;
  bool saw_digit = false;

  while (i < rx_len && rx_buffer()[i] != '>') {
    uint8_t c = rx_buffer()[i];
    if (c < '0' || c > '9') {
      if (rx_startup_grace_active()) {
        rx_startup_len_overflow_suppressed++;
      } else {
        rx_len_overflow_count++;
      }
      rx_reset_hard();
      return false;
    }
    saw_digit = true;
    declared_len = declared_len * 10 + (size_t)(c - '0');
    ++i;
  }

  if (!saw_digit || i >= rx_len)
    return false;

  size_t header_end = i;
  size_t json_start = header_end + 1;
  size_t required_total = json_start + declared_len + ETX_LEN;

  if (required_total > RX_BUF_MAX) {
    if (rx_startup_grace_active()) {
      rx_startup_len_overflow_suppressed++;
    } else {
      rx_len_overflow_count++;
    }
    rx_reset_hard();
    return false;
  }

  if (rx_len < required_total)
    return false;

  size_t etx_pos = json_start + declared_len;
  if (memcmp(rx_buffer() + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    if (rx_startup_grace_active()) {
      rx_startup_bad_etx_suppressed++;
    } else {
      rx_bad_etx_count++;
    }
    rx_reset_hard();
    return false;
  }

  if (!rx_guard_check(rx_guard_stage_t::BEFORE_PARSE)) {
    rx_reset_hard();
    return false;
  }

  // ETX is already validated.  Replace its first byte with an explicit NUL so
  // any hidden C-string dependency or one-byte parser overread sees a lawful
  // terminator while parseJSON() still receives the authoritative JSON length.
  rx_buffer()[etx_pos] = '\0';
  rx_json_terminator_count++;

  rx_first_frame_seen = true;
  rx_frames_complete++;

  Payload parsed;
  parsed.parseJSON(rx_buffer() + json_start, declared_len);

  if (!rx_guard_check(rx_guard_stage_t::AFTER_PARSE)) {
    rx_reset_hard();
    return false;
  }

  rx_dispatch_traffic = rx_traffic;
  rx_dispatch_payload = static_cast<Payload&&>(parsed);
  rx_dispatch_pending = true;
  rx_reset_hard();

  if (timepop_arm_alap(
        rx_dispatch_alap,
        nullptr,
        "TRANSPORT_RX_DISPATCH") == TIMEPOP_INVALID_HANDLE) {
    rx_dispatch_pending = false;
    rx_dispatch_payload.clear();
    rx_dispatch_alap_arm_fail++;
  }

  return true;
}

// =============================================================
// RECEIVE: SERIAL RX (stream-based)
// =============================================================

static void rx_serial_tick() {

  transport_rx_entered();

  bool appended = false;

  size_t consumed = 0;

  while (consumed < TRANSPORT_RX_QUANTUM_MAX && ZPNET_SERIAL.available()) {
    uint8_t b = (uint8_t)ZPNET_SERIAL.read();
    consumed++;

    if (!rx_have_traffic) {
      if (!is_valid_traffic(b)) {
        if (rx_startup_grace_active()) {
          rx_startup_note_missing_byte(b);
        } else {
          rx_expected_traffic_missing++;
        }
        continue;
      }
      rx_begin(b);
      continue;
    }

    // If a new traffic byte appears before the STX prefix has been established,
    // treat it as a fresh frame boundary. This makes orphaned traffic bytes and
    // partial prior frames recoverable without sacrificing length-authoritative
    // parsing once a real frame is underway.
    if (rx_should_resync(b)) {
      rx_overlap_count++;
      rx_begin(b);
      continue;
    }

    if (rx_len >= RX_BUF_MAX) {
      rx_reset_hard();
      return;
    }

    rx_buffer()[rx_len++] = b;
    rx_bytes_total++;
    appended = true;

    if (dispatch_if_complete())
      break;
  }

  if (appended) {
    rx_blocks_total++;
  }
}

static void transport_rx_timepop(
  timepop_ctx_t*,
  timepop_diag_t*,
  void*
) {
  transport_poll_count++;
  transport_rx_poll_count++;
  rx_serial_tick();
}

static void transport_tx_timepop(
  timepop_ctx_t*,
  timepop_diag_t*,
  void*
) {
  transport_poll_count++;
  transport_tx_poll_count++;
  tx_pump_once();
}

void transport_note_runtime_loop(void) {
  transport_runtime_loop_count++;
}

// =============================================================
// transport_get_info()
// =============================================================

FLASHMEM void transport_get_info(transport_info_t* out) {

  if (!out) return;

  out->poll_count              = transport_poll_count;
  out->poll_skipped_too_soon   = transport_poll_skipped_too_soon;
  out->rx_poll_count           = transport_rx_poll_count;
  out->tx_poll_count           = transport_tx_poll_count;
  out->runtime_loop_count      = transport_runtime_loop_count;
  out->poll_interval_us        = (uint32_t)(TRANSPORT_SERVICE_PERIOD_NS / 1000ULL);

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

  out->rx_buffer_in_dmamem = 1U;
  out->rx_buffer_address = (uint32_t)(uintptr_t)rx_buffer();
  out->rx_buffer_size = RX_BUF_MAX;
  out->rx_buffer_alignment = RX_CACHE_LINE_BYTES;
  out->rx_buffer_alignment_ok =
      (((uintptr_t)rx_buffer() & (RX_CACHE_LINE_BYTES - 1U)) == 0U) ? 1U : 0U;
  out->rx_poison_byte = RX_POISON_BYTE;
  out->rx_storage_init_count = rx_storage_init_count;
  out->rx_json_terminator_count = rx_json_terminator_count;
  out->rx_guard_check_count = rx_guard_check_count;
  out->rx_guard_failure_count = rx_guard_failure_count;
  out->rx_guard_before_failure_count = rx_guard_before_failure_count;
  out->rx_guard_after_failure_count = rx_guard_after_failure_count;
  out->rx_guard_last_stage = rx_guard_last_stage;
  out->rx_guard_last_side = rx_guard_last_side;
  out->rx_guard_last_index = rx_guard_last_index;
  out->rx_guard_last_expected = rx_guard_last_expected;
  out->rx_guard_last_observed = rx_guard_last_observed;

  out->rx_first_frame_seen = rx_first_frame_seen ? 1U : 0U;
  out->rx_startup_grace_active = rx_startup_grace_active() ? 1U : 0U;
  out->rx_startup_grace_ms = TRANSPORT_RX_STARTUP_GRACE_MS;
  out->rx_ms_since_init = (uint32_t)(millis() - rx_init_ms);
  out->rx_startup_expected_traffic_missing_suppressed =
      rx_startup_expected_traffic_missing_suppressed;
  out->rx_startup_bad_stx_suppressed = rx_startup_bad_stx_suppressed;
  out->rx_startup_bad_etx_suppressed = rx_startup_bad_etx_suppressed;
  out->rx_startup_len_overflow_suppressed =
      rx_startup_len_overflow_suppressed;
  out->rx_startup_first_missing_byte =
      (rx_startup_first_missing_byte == 0xFFFFFFFFUL)
          ? 0xFFFFFFFFUL
          : rx_startup_first_missing_byte;
  out->rx_startup_last_missing_byte = rx_startup_last_missing_byte;
}

// =============================================================
// Init
// =============================================================

void transport_init(void) {

  rx_storage_initialize();

  ZPNET_SERIAL.begin(USB_SERIAL_BAUD);
  rx_init_ms = millis();
  rx_first_frame_seen = false;
  rx_len = 0;
  rx_have_traffic = false;
  rx_traffic = 0;
  rx_dispatch_pending = false;
  rx_dispatch_traffic = 0;
  rx_dispatch_payload.clear();

  timepop_arm(
    TRANSPORT_SERVICE_PERIOD_NS,
    true,
    transport_rx_timepop,
    nullptr,
    "TRANSPORT_RX"
  );

  timepop_arm(
    TRANSPORT_SERVICE_PERIOD_NS,
    true,
    transport_tx_timepop,
    nullptr,
    "TRANSPORT_TX"
  );

  debug_log("transport", BUILD_FINGERPRINT);
  debug_log("transport", TRANSPORT_LIFELINE_FINGERPRINT);
}
