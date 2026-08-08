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
#include "crash_forensics.h"

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
static constexpr size_t TX_CONTROL_RESERVE = 16 * 1024;
static constexpr size_t TX_BUDGET_MAX = 64 * 1024;

static_assert(TX_BUDGET_MAX >=
                  TRANSPORT_MAX_MESSAGE + FRAME_SLACK +
                      TX_CONTROL_RESERVE,
              "TX budget must hold one maximum message plus control reserve");

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
  uint32_t sequence;
  uint32_t json_length;
  uint8_t  traffic;
  bool     timebase_fragment;
  char     topic[48];
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

enum : uint32_t {
  TX_OUTCOME_NONE = 0U,
  TX_OUTCOME_REJECTED = 1U,
  TX_OUTCOME_ENQUEUED = 2U,
  TX_OUTCOME_COMPLETED = 3U,
};

enum : uint32_t {
  TX_REASON_NONE = 0U,
  TX_REASON_EMPTY_SERIALIZATION = 1U,
  TX_REASON_SEMANTIC_EMPTY = 2U,
  TX_REASON_MESSAGE_TOO_LARGE = 3U,
  TX_REASON_HEADER_FORMAT = 4U,
  TX_REASON_OUTSTANDING_BUDGET = 5U,
  TX_REASON_QUEUE_FULL = 6U,
  TX_REASON_ALLOC_FAIL = 7U,
};

static volatile uint32_t tx_send_attempt_count = 0U;
static volatile uint32_t tx_send_serialized_count = 0U;
static volatile uint32_t tx_send_reject_count = 0U;
static volatile uint32_t tx_send_enqueued_count = 0U;
static volatile uint32_t tx_send_completed_count = 0U;
static volatile uint32_t tx_message_too_large_count = 0U;
static volatile uint32_t tx_header_format_fail_count = 0U;
static volatile uint32_t tx_outstanding_budget_fail_count = 0U;
static volatile uint32_t tx_empty_serialization_count = 0U;
static volatile uint32_t tx_semantic_empty_count = 0U;

static uint32_t tx_last_sequence = 0U;
static uint32_t tx_last_traffic = 0U;
static uint32_t tx_last_json_bytes = 0U;
static uint32_t tx_last_wire_bytes = 0U;
static uint32_t tx_last_outcome_id = TX_OUTCOME_NONE;
static uint32_t tx_last_reason_id = TX_REASON_NONE;
static char tx_last_topic[48] = {0};

static uint32_t tx_largest_json_bytes = 0U;
static uint32_t tx_largest_wire_bytes = 0U;
static char tx_largest_topic[48] = {0};

static uint32_t tx_last_reject_sequence = 0U;
static uint32_t tx_last_reject_traffic = 0U;
static uint32_t tx_last_reject_json_bytes = 0U;
static uint32_t tx_last_reject_wire_bytes = 0U;
static uint32_t tx_last_reject_reason_id = TX_REASON_NONE;
static char tx_last_reject_topic[48] = {0};

static uint32_t tx_last_completed_sequence = 0U;
static uint32_t tx_last_completed_traffic = 0U;
static uint32_t tx_last_completed_json_bytes = 0U;
static uint32_t tx_last_completed_wire_bytes = 0U;
static char tx_last_completed_topic[48] = {0};

static volatile uint32_t tx_timebase_attempt_count = 0U;
static volatile uint32_t tx_timebase_serialized_count = 0U;
static volatile uint32_t tx_timebase_reject_count = 0U;
static volatile uint32_t tx_timebase_enqueued_count = 0U;
static volatile uint32_t tx_timebase_completed_count = 0U;
static uint32_t tx_timebase_last_sequence = 0U;
static uint32_t tx_timebase_last_json_bytes = 0U;
static uint32_t tx_timebase_last_wire_bytes = 0U;
static uint32_t tx_timebase_last_outcome_id = TX_OUTCOME_NONE;
static uint32_t tx_timebase_last_reason_id = TX_REASON_NONE;
static uint32_t tx_timebase_largest_json_bytes = 0U;
static uint32_t tx_timebase_largest_wire_bytes = 0U;

static void tx_copy_topic(char* dst, size_t capacity, const char* topic) {
  if (!dst || capacity == 0U) return;
  const char* src = (topic && *topic) ? topic : "<none>";
  size_t i = 0U;
  while (i + 1U < capacity && src[i] != '\0') {
    dst[i] = src[i];
    ++i;
  }
  dst[i] = '\0';
}

static bool tx_topic_is_timebase(const char* topic) {
  return topic && strcmp(topic, "TIMEBASE_FRAGMENT") == 0;
}

const char* transport_tx_outcome_name(uint32_t outcome_id) {
  switch (outcome_id) {
    case TX_OUTCOME_REJECTED: return "REJECTED";
    case TX_OUTCOME_ENQUEUED: return "ENQUEUED";
    case TX_OUTCOME_COMPLETED: return "COMPLETED";
    default: return "NONE";
  }
}

const char* transport_tx_reason_name(uint32_t reason_id) {
  switch (reason_id) {
    case TX_REASON_EMPTY_SERIALIZATION: return "EMPTY_SERIALIZATION";
    case TX_REASON_SEMANTIC_EMPTY: return "SEMANTIC_EMPTY";
    case TX_REASON_MESSAGE_TOO_LARGE: return "MESSAGE_TOO_LARGE";
    case TX_REASON_HEADER_FORMAT: return "HEADER_FORMAT";
    case TX_REASON_OUTSTANDING_BUDGET: return "OUTSTANDING_BUDGET";
    case TX_REASON_QUEUE_FULL: return "QUEUE_FULL";
    case TX_REASON_ALLOC_FAIL: return "ALLOC_FAIL";
    default: return "NONE";
  }
}

static void tx_note_last(uint32_t sequence,
                         uint8_t traffic,
                         const char* topic,
                         size_t json_len,
                         size_t wire_len,
                         uint32_t outcome,
                         uint32_t reason) {
  tx_last_sequence = sequence;
  tx_last_traffic = traffic;
  tx_last_json_bytes = (uint32_t)json_len;
  tx_last_wire_bytes = (uint32_t)wire_len;
  tx_last_outcome_id = outcome;
  tx_last_reason_id = reason;
  tx_copy_topic(tx_last_topic, sizeof(tx_last_topic), topic);

  if (json_len > tx_largest_json_bytes) {
    tx_largest_json_bytes = (uint32_t)json_len;
    tx_largest_wire_bytes = (uint32_t)wire_len;
    tx_copy_topic(tx_largest_topic, sizeof(tx_largest_topic), topic);
  }
}

static void tx_note_reject(uint32_t sequence,
                           uint8_t traffic,
                           const char* topic,
                           size_t json_len,
                           size_t wire_len,
                           uint32_t reason,
                           bool timebase) {
  tx_send_reject_count++;
  tx_last_reject_sequence = sequence;
  tx_last_reject_traffic = traffic;
  tx_last_reject_json_bytes = (uint32_t)json_len;
  tx_last_reject_wire_bytes = (uint32_t)wire_len;
  tx_last_reject_reason_id = reason;
  tx_copy_topic(tx_last_reject_topic, sizeof(tx_last_reject_topic), topic);
  tx_note_last(sequence, traffic, topic, json_len, wire_len,
               TX_OUTCOME_REJECTED, reason);
  if (timebase) {
    tx_timebase_reject_count++;
    tx_timebase_last_sequence = sequence;
    tx_timebase_last_json_bytes = (uint32_t)json_len;
    tx_timebase_last_wire_bytes = (uint32_t)wire_len;
    tx_timebase_last_outcome_id = TX_OUTCOME_REJECTED;
    tx_timebase_last_reason_id = reason;
  }
}

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
static volatile uint32_t rx_dispatch_invalid_callback = 0;
static volatile uint32_t rx_dispatch_stack_mismatch = 0;

// A complete frame must not overwrite the one-slot semantic-dispatch mailbox
// before ALAP has taken custody.  For now this is forensic only: record the
// exact collision and leave behavior unchanged so the instrument can tell us
// whether this boundary participates in the sparse RPC anomaly.
static volatile uint32_t rx_dispatch_pending_collision_count = 0;
static volatile uint32_t rx_dispatch_pending_collision_dwt = 0;
static volatile uint32_t rx_dispatch_pending_collision_pending_traffic = 0;
static volatile uint32_t rx_dispatch_pending_collision_incoming_traffic = 0;
static volatile uint32_t rx_dispatch_pending_collision_pending_req_id = 0;
static volatile uint32_t rx_dispatch_pending_collision_incoming_req_id = 0;
static volatile uint32_t rx_dispatch_pending_collision_pending_req_id_valid = 0;
static volatile uint32_t rx_dispatch_pending_collision_incoming_req_id_valid = 0;

static constexpr uint32_t RX_DISPATCH_BREADCRUMB_MAGIC = 0x5A505258UL;
alignas(32) static transport_rx_dispatch_breadcrumb_t
    g_rx_dispatch_breadcrumb_live[2] DMAMEM;
alignas(32) static transport_rx_dispatch_breadcrumb_t
    g_rx_dispatch_breadcrumb_retained DMAMEM;
static bool g_rx_dispatch_breadcrumb_boot_latched = false;
static uint32_t g_rx_dispatch_min_msp = 0xFFFFFFFFUL;

static inline uint32_t transport_read_msp() {
  uint32_t v; __asm__ volatile("mrs %0, msp" : "=r"(v) :: "memory"); return v;
}
static inline uint32_t transport_read_ipsr() {
  uint32_t v; __asm__ volatile("mrs %0, ipsr" : "=r"(v) :: "memory"); return v & 0x1FFU;
}
static inline uint32_t transport_read_lr() {
  uint32_t v; __asm__ volatile("mov %0, lr" : "=r"(v) :: "memory"); return v;
}
static bool transport_code_address(uintptr_t bits) {
  if (bits == 0U || (bits & 1U) == 0U) return false;
  const uintptr_t a = bits & ~uintptr_t(1U);
  return (a >= 0x00000400UL && a < 0x00100000UL) ||
         (a >= 0x60001000UL && a < 0x61000000UL);
}
static bool rx_dispatch_breadcrumb_valid(const transport_rx_dispatch_breadcrumb_t& b) {
  return b.sequence != 0U && (b.sequence ^ b.sequence_inv) == 0xFFFFFFFFUL &&
         b.schema_version == TRANSPORT_RX_DISPATCH_SCHEMA_VERSION &&
         b.reserved0 == RX_DISPATCH_BREADCRUMB_MAGIC &&
         b.reserved1 == ~RX_DISPATCH_BREADCRUMB_MAGIC;
}
static void rx_dispatch_breadcrumb_note(uint32_t stage, uint8_t traffic,
                                        transport_receive_cb_t callback,
                                        uint32_t before, uint32_t after) {
  uint32_t sequence = g_rx_dispatch_breadcrumb_live[0].sequence;
  if ((int32_t)(g_rx_dispatch_breadcrumb_live[1].sequence - sequence) > 0)
    sequence = g_rx_dispatch_breadcrumb_live[1].sequence;
  if (++sequence == 0U) sequence = 1U;
  transport_rx_dispatch_breadcrumb_t& b = g_rx_dispatch_breadcrumb_live[sequence & 1U];
  b.sequence = 0U; b.sequence_inv = 0U;
  b.schema_version = TRANSPORT_RX_DISPATCH_SCHEMA_VERSION; b.stage = stage;
  b.dwt = ARM_DWT_CYCCNT; b.msp_before = before; b.msp_after = after;
  b.min_msp = g_rx_dispatch_min_msp; b.callback = (uint32_t)(uintptr_t)callback;
  b.traffic = traffic; b.payload = (uint32_t)(uintptr_t)&rx_dispatch_payload;
  b.payload_count = (uint32_t)rx_dispatch_payload.count();
  b.ipsr = transport_read_ipsr(); b.lr = transport_read_lr();
  b.reserved0 = RX_DISPATCH_BREADCRUMB_MAGIC; b.reserved1 = ~RX_DISPATCH_BREADCRUMB_MAGIC;
  b.sequence_inv = ~sequence; __asm__ volatile("dmb" ::: "memory");
  b.sequence = sequence; __asm__ volatile("dmb" ::: "memory");
  arm_dcache_flush(&b, sizeof(b));
}
static void rx_dispatch_breadcrumb_boot_latch() {
  if (g_rx_dispatch_breadcrumb_boot_latched) return;
  g_rx_dispatch_breadcrumb_boot_latched = true;
  const bool v0 = rx_dispatch_breadcrumb_valid(g_rx_dispatch_breadcrumb_live[0]);
  const bool v1 = rx_dispatch_breadcrumb_valid(g_rx_dispatch_breadcrumb_live[1]);
  const transport_rx_dispatch_breadcrumb_t* newest = nullptr;
  if (v0 && v1) newest = (int32_t)(g_rx_dispatch_breadcrumb_live[1].sequence -
                                   g_rx_dispatch_breadcrumb_live[0].sequence) > 0
      ? &g_rx_dispatch_breadcrumb_live[1] : &g_rx_dispatch_breadcrumb_live[0];
  else if (v0) newest = &g_rx_dispatch_breadcrumb_live[0];
  else if (v1) newest = &g_rx_dispatch_breadcrumb_live[1];
  if (newest) { g_rx_dispatch_breadcrumb_retained = *newest;
    arm_dcache_flush(&g_rx_dispatch_breadcrumb_retained, sizeof(g_rx_dispatch_breadcrumb_retained)); }
  memset(g_rx_dispatch_breadcrumb_live, 0, sizeof(g_rx_dispatch_breadcrumb_live));
  arm_dcache_flush(g_rx_dispatch_breadcrumb_live, sizeof(g_rx_dispatch_breadcrumb_live));
}

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
  const size_t released_length = job.length;

  tx_send_completed_count++;
  tx_last_completed_sequence = job.sequence;
  tx_last_completed_traffic = job.traffic;
  tx_last_completed_json_bytes = job.json_length;
  tx_last_completed_wire_bytes = (uint32_t)job.length;
  tx_copy_topic(tx_last_completed_topic, sizeof(tx_last_completed_topic),
                job.topic);
  tx_note_last(job.sequence, job.traffic, job.topic, job.json_length,
               job.length, TX_OUTCOME_COMPLETED, TX_REASON_NONE);
  if (job.timebase_fragment) {
    tx_timebase_completed_count++;
    tx_timebase_last_sequence = job.sequence;
    tx_timebase_last_json_bytes = job.json_length;
    tx_timebase_last_wire_bytes = (uint32_t)job.length;
    tx_timebase_last_outcome_id = TX_OUTCOME_COMPLETED;
    tx_timebase_last_reason_id = TX_REASON_NONE;
  }

  free(job.data);
  job = tx_job_t{};

  tx_budget_used -= released_length;

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
// TX wire allocation / queue helpers
// =============================================================

static bool tx_allocate_wire(uint32_t sequence,
                             uint8_t traffic,
                             const char* topic,
                             size_t json_len,
                             bool timebase,
                             uint8_t** out_data,
                             size_t* out_wire_len,
                             size_t* out_json_offset) {
  if (!out_data || !out_wire_len || !out_json_offset) return false;
  *out_data = nullptr;
  *out_wire_len = 0U;
  *out_json_offset = 0U;

  if (json_len > TRANSPORT_MAX_MESSAGE) {
    tx_budget_fail++;
    tx_message_too_large_count++;
    tx_note_reject(sequence, traffic, topic, json_len, 0U,
                   TX_REASON_MESSAGE_TOO_LARGE, timebase);
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return false;
  }

  char header[32];
  const int header_len = snprintf(header, sizeof(header),
                                  "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0 || (size_t)header_len >= sizeof(header)) {
    tx_budget_fail++;
    tx_header_format_fail_count++;
    tx_note_reject(sequence, traffic, topic, json_len, 0U,
                   TX_REASON_HEADER_FORMAT, timebase);
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return false;
  }

  const size_t wire_len =
      1U +                     // traffic byte
      (size_t)header_len +
      json_len +
      ETX_LEN;

  if (timebase) {
    tx_timebase_last_wire_bytes = (uint32_t)wire_len;
    if (wire_len > tx_timebase_largest_wire_bytes) {
      tx_timebase_largest_wire_bytes = (uint32_t)wire_len;
    }
  }

  if (tx_budget_used + wire_len > TX_BUDGET_MAX) {
    tx_budget_fail++;
    tx_outstanding_budget_fail_count++;
    tx_note_reject(sequence, traffic, topic, json_len, wire_len,
                   TX_REASON_OUTSTANDING_BUDGET, timebase);
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return false;
  }

  if (tx_job_count >= TX_JOB_MAX) {
    tx_queue_full++;
    tx_note_reject(sequence, traffic, topic, json_len, wire_len,
                   TX_REASON_QUEUE_FULL, timebase);
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return false;
  }

  // One allocation owns the complete frame for its entire queued lifetime.
  uint8_t* data = (uint8_t*)malloc(wire_len);
  if (!data) {
    tx_alloc_fail++;
    tx_note_reject(sequence, traffic, topic, json_len, wire_len,
                   TX_REASON_ALLOC_FAIL, timebase);
    if (traffic == TRAFFIC_REQUEST_RESPONSE) {
      tx_rr_drop_count++;
    }
    return false;
  }

  size_t pos = 0U;
  data[pos++] = traffic;
  memcpy(data + pos, header, (size_t)header_len);
  pos += (size_t)header_len;

  *out_data = data;
  *out_wire_len = wire_len;
  *out_json_offset = pos;
  return true;
}

static void tx_commit_wire(uint32_t sequence,
                           uint8_t traffic,
                           const char* topic,
                           size_t json_len,
                           size_t wire_len,
                           uint8_t* data,
                           bool timebase) {
  tx_send_serialized_count++;
  if (timebase) {
    tx_timebase_serialized_count++;
    tx_timebase_last_json_bytes = (uint32_t)json_len;
    if (json_len > tx_timebase_largest_json_bytes) {
      tx_timebase_largest_json_bytes = (uint32_t)json_len;
    }
  }

  tx_job_t& job = tx_jobs[tx_job_head];
  job = tx_job_t{};
  job.data = data;
  job.length = wire_len;
  job.sent = 0U;
  job.sequence = sequence;
  job.json_length = (uint32_t)json_len;
  job.traffic = traffic;
  job.timebase_fragment = timebase;
  tx_copy_topic(job.topic, sizeof(job.topic), topic);

  tx_job_head = (tx_job_head + 1U) % TX_JOB_MAX;
  tx_job_count++;
  tx_jobs_enqueued++;
  tx_send_enqueued_count++;
  tx_bytes_enqueued += wire_len;

  tx_note_last(sequence, traffic, topic, json_len, wire_len,
               TX_OUTCOME_ENQUEUED, TX_REASON_NONE);
  if (timebase) {
    tx_timebase_enqueued_count++;
    tx_timebase_last_outcome_id = TX_OUTCOME_ENQUEUED;
    tx_timebase_last_reason_id = TX_REASON_NONE;
  }

  tx_budget_used += wire_len;

  if (tx_budget_used > tx_budget_high_water) {
    tx_budget_high_water = tx_budget_used;
  }

  if (tx_job_count > tx_job_high_water) {
    tx_job_high_water = tx_job_count;
  }
}

// =============================================================
// transport_send() — measure, allocate final wire image, serialize, enqueue
// =============================================================

bool transport_send(uint8_t traffic, const Payload& payload) {

  const uint32_t sequence = ++tx_send_attempt_count;
  const char* topic = payload.getString("topic");
  const bool timebase = tx_topic_is_timebase(topic);
  if (timebase) {
    tx_timebase_attempt_count++;
    tx_timebase_last_sequence = sequence;
  }

  // Measure without allocating. Payload performs the same structural and
  // semantic validation used by write_json(). A zero result is therefore an
  // explicit serialization failure, not an empty JSON document (which is "{}").
  const size_t json_len = payload.json_size();
  if (json_len == 0U) {
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, 0U, 0U,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  if (!payload.empty() && json_len == 2U) {
    tx_semantic_empty_count++;
    tx_note_reject(sequence, traffic, topic, json_len, 0U,
                   TX_REASON_SEMANTIC_EMPTY, timebase);
    return false;
  }

  uint8_t* data = nullptr;
  size_t wire_len = 0U;
  size_t json_offset = 0U;
  if (!tx_allocate_wire(sequence, traffic, topic, json_len, timebase,
                        &data, &wire_len, &json_offset)) {
    return false;
  }

  // write_json() writes its trailing NUL into the first ETX byte; ETX is copied
  // immediately afterward, so no extra byte or temporary String is required.
  const size_t written = payload.write_json(
      reinterpret_cast<char*>(data + json_offset), json_len + 1U);
  if (written != json_len) {
    free(data);
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, json_len, wire_len,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  size_t pos = json_offset + written;
  memcpy(data + pos, ETX_SEQ, ETX_LEN);
  pos += ETX_LEN;

  if (pos != wire_len) {
    free(data);
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, json_len, wire_len,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  tx_commit_wire(sequence, traffic, topic, json_len, wire_len, data, timebase);
  return true;
}

// =============================================================
// transport_send_publish() — write pub/sub envelope directly into final wire
// =============================================================

bool transport_send_publish(const char* topic, const Payload& payload) {

  const uint8_t traffic = TRAFFIC_PUBLISH_SUBSCRIBE;
  const uint32_t sequence = ++tx_send_attempt_count;
  const bool timebase = tx_topic_is_timebase(topic);
  if (timebase) {
    tx_timebase_attempt_count++;
    tx_timebase_last_sequence = sequence;
  }

  if (!topic || !*topic) {
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, 0U, 0U,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  // Use a tiny Payload only for the topic field so string validation and JSON
  // escaping remain identical to the canonical Payload serializer. The large
  // publication payload is never copied into another Payload.
  Payload topic_field;
  if (!topic_field.add("topic", topic)) {
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, 0U, 0U,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  const size_t topic_json_len = topic_field.json_size();
  const size_t payload_json_len = payload.json_size();
  static constexpr char PAYLOAD_KEY[] = "\"payload\":";
  static constexpr size_t PAYLOAD_KEY_LEN = sizeof(PAYLOAD_KEY) - 1U;

  if (topic_json_len < 2U || payload_json_len == 0U) {
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, 0U, 0U,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  // topic_field is {"topic":...}. Replace its closing brace with a comma, append
  // "payload": plus the original payload JSON, then close the envelope. Payload
  // arena limits keep this addition far below size_t overflow; tx_allocate_wire()
  // remains the authoritative message-size court for the completed envelope.
  const size_t envelope_len =
      topic_json_len + PAYLOAD_KEY_LEN + payload_json_len + 1U;

  uint8_t* data = nullptr;
  size_t wire_len = 0U;
  size_t json_offset = 0U;
  if (!tx_allocate_wire(sequence, traffic, topic, envelope_len, timebase,
                        &data, &wire_len, &json_offset)) {
    return false;
  }

  const size_t topic_written = topic_field.write_json(
      reinterpret_cast<char*>(data + json_offset), topic_json_len + 1U);
  if (topic_written != topic_json_len ||
      data[json_offset + topic_json_len - 1U] != (uint8_t)'}') {
    free(data);
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, envelope_len, wire_len,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  size_t pos = json_offset + topic_json_len;
  data[pos - 1U] = (uint8_t)',';
  memcpy(data + pos, PAYLOAD_KEY, PAYLOAD_KEY_LEN);
  pos += PAYLOAD_KEY_LEN;

  const size_t payload_written = payload.write_json(
      reinterpret_cast<char*>(data + pos), payload_json_len + 1U);
  if (payload_written != payload_json_len) {
    free(data);
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, envelope_len, wire_len,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }
  pos += payload_written;
  data[pos++] = (uint8_t)'}';

  memcpy(data + pos, ETX_SEQ, ETX_LEN);
  pos += ETX_LEN;

  if (pos != wire_len) {
    free(data);
    tx_empty_serialization_count++;
    tx_note_reject(sequence, traffic, topic, envelope_len, wire_len,
                   TX_REASON_EMPTY_SERIALIZATION, timebase);
    return false;
  }

  tx_commit_wire(sequence, traffic, topic, envelope_len, wire_len, data, timebase);
  return true;
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
  const transport_receive_cb_t callback = recv_cb[traffic];
  const uint32_t msp_before = transport_read_msp();
  g_rx_dispatch_min_msp = msp_before;
  rx_dispatch_pending = false;
  rx_dispatch_breadcrumb_note(TRANSPORT_RX_DISPATCH_ENTER,
                              traffic, callback, msp_before, 0U);

  if (callback && transport_code_address((uintptr_t)callback)) {
    rx_frames_dispatched++;
    callback(rx_dispatch_payload);
  } else if (callback) {
    rx_dispatch_invalid_callback++;
    rx_dispatch_breadcrumb_note(TRANSPORT_RX_DISPATCH_INVALID_CALLBACK,
                                traffic, callback, msp_before,
                                transport_read_msp());
  }

  const uint32_t msp_after = transport_read_msp();
  if (msp_after < g_rx_dispatch_min_msp) g_rx_dispatch_min_msp = msp_after;
  if (msp_after != msp_before) {
    rx_dispatch_stack_mismatch++;
    rx_dispatch_breadcrumb_note(TRANSPORT_RX_DISPATCH_STACK_MISMATCH,
                                traffic, callback, msp_before, msp_after);
    __asm__ volatile("udf #0");
  }
  rx_dispatch_breadcrumb_note(TRANSPORT_RX_DISPATCH_CALLBACK_RETURN,
                              traffic, callback, msp_before, msp_after);
  rx_dispatch_payload.clear();
  if (!rx_guard_check(rx_guard_stage_t::AFTER_DISPATCH)) rx_reset_hard();
  rx_dispatch_breadcrumb_note(TRANSPORT_RX_DISPATCH_COMPLETE,
                              traffic, callback, msp_before,
                              transport_read_msp());
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

  if (rx_dispatch_pending) {
    uint32_t pending_req_id = 0U;
    uint32_t incoming_req_id = 0U;
    const bool pending_req_id_valid =
        rx_dispatch_payload.tryGetUInt("req_id", pending_req_id) &&
        pending_req_id != 0U;
    const bool incoming_req_id_valid =
        parsed.tryGetUInt("req_id", incoming_req_id) &&
        incoming_req_id != 0U;

    rx_dispatch_pending_collision_count++;
    rx_dispatch_pending_collision_dwt = ARM_DWT_CYCCNT;
    rx_dispatch_pending_collision_pending_traffic = rx_dispatch_traffic;
    rx_dispatch_pending_collision_incoming_traffic = rx_traffic;
    rx_dispatch_pending_collision_pending_req_id =
        pending_req_id_valid ? pending_req_id : 0U;
    rx_dispatch_pending_collision_incoming_req_id =
        incoming_req_id_valid ? incoming_req_id : 0U;
    rx_dispatch_pending_collision_pending_req_id_valid =
        pending_req_id_valid ? 1U : 0U;
    rx_dispatch_pending_collision_incoming_req_id_valid =
        incoming_req_id_valid ? 1U : 0U;
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

const char* transport_rx_dispatch_stage_name(uint32_t stage) {
  switch (stage) {
    case TRANSPORT_RX_DISPATCH_ENTER: return "ENTER";
    case TRANSPORT_RX_DISPATCH_CALLBACK_RETURN: return "CALLBACK_RETURN";
    case TRANSPORT_RX_DISPATCH_COMPLETE: return "COMPLETE";
    case TRANSPORT_RX_DISPATCH_INVALID_CALLBACK: return "INVALID_CALLBACK";
    case TRANSPORT_RX_DISPATCH_STACK_MISMATCH: return "STACK_MISMATCH";
    default: return "NONE";
  }
}
void transport_rx_dispatch_snapshot(transport_rx_dispatch_snapshot_t* out) {
  if (!out) return; rx_dispatch_breadcrumb_boot_latch();
  *out = transport_rx_dispatch_snapshot_t{};
  const bool v0 = rx_dispatch_breadcrumb_valid(g_rx_dispatch_breadcrumb_live[0]);
  const bool v1 = rx_dispatch_breadcrumb_valid(g_rx_dispatch_breadcrumb_live[1]);
  if (v0 || v1) {
    const transport_rx_dispatch_breadcrumb_t* n = v0 && v1
      ? ((int32_t)(g_rx_dispatch_breadcrumb_live[1].sequence - g_rx_dispatch_breadcrumb_live[0].sequence) > 0
         ? &g_rx_dispatch_breadcrumb_live[1] : &g_rx_dispatch_breadcrumb_live[0])
      : (v0 ? &g_rx_dispatch_breadcrumb_live[0] : &g_rx_dispatch_breadcrumb_live[1]);
    out->live_valid = true; out->live = *n;
  }
  if (rx_dispatch_breadcrumb_valid(g_rx_dispatch_breadcrumb_retained)) {
    out->retained_valid = true; out->retained = g_rx_dispatch_breadcrumb_retained;
  }
}

// =============================================================
// RX deferred-dispatch mailbox forensic snapshot
// =============================================================

void transport_get_rx_dispatch_mailbox_info(
    uint32_t* pending_now,
    uint32_t* alap_arm_fail,
    uint32_t* collision_count,
    uint32_t* collision_dwt,
    uint32_t* pending_traffic,
    uint32_t* incoming_traffic,
    uint32_t* pending_req_id_valid,
    uint32_t* pending_req_id,
    uint32_t* incoming_req_id_valid,
    uint32_t* incoming_req_id) {
  if (pending_now) *pending_now = rx_dispatch_pending ? 1U : 0U;
  if (alap_arm_fail) *alap_arm_fail = rx_dispatch_alap_arm_fail;
  if (collision_count) {
    *collision_count = rx_dispatch_pending_collision_count;
  }
  if (collision_dwt) *collision_dwt = rx_dispatch_pending_collision_dwt;
  if (pending_traffic) {
    *pending_traffic = rx_dispatch_pending_collision_pending_traffic;
  }
  if (incoming_traffic) {
    *incoming_traffic = rx_dispatch_pending_collision_incoming_traffic;
  }
  if (pending_req_id_valid) {
    *pending_req_id_valid =
        rx_dispatch_pending_collision_pending_req_id_valid;
  }
  if (pending_req_id) {
    *pending_req_id = rx_dispatch_pending_collision_pending_req_id;
  }
  if (incoming_req_id_valid) {
    *incoming_req_id_valid =
        rx_dispatch_pending_collision_incoming_req_id_valid;
  }
  if (incoming_req_id) {
    *incoming_req_id = rx_dispatch_pending_collision_incoming_req_id;
  }
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

  out->tx_send_attempt_count = tx_send_attempt_count;
  out->tx_send_serialized_count = tx_send_serialized_count;
  out->tx_send_reject_count = tx_send_reject_count;
  out->tx_send_enqueued_count = tx_send_enqueued_count;
  out->tx_send_completed_count = tx_send_completed_count;
  out->tx_message_too_large_count = tx_message_too_large_count;
  out->tx_header_format_fail_count = tx_header_format_fail_count;
  out->tx_outstanding_budget_fail_count = tx_outstanding_budget_fail_count;
  out->tx_empty_serialization_count = tx_empty_serialization_count;
  out->tx_semantic_empty_count = tx_semantic_empty_count;

  out->tx_last_sequence = tx_last_sequence;
  out->tx_last_traffic = tx_last_traffic;
  out->tx_last_json_bytes = tx_last_json_bytes;
  out->tx_last_wire_bytes = tx_last_wire_bytes;
  out->tx_last_outcome_id = tx_last_outcome_id;
  out->tx_last_reason_id = tx_last_reason_id;
  tx_copy_topic(out->tx_last_topic, sizeof(out->tx_last_topic), tx_last_topic);
  out->tx_largest_json_bytes = tx_largest_json_bytes;
  out->tx_largest_wire_bytes = tx_largest_wire_bytes;
  tx_copy_topic(out->tx_largest_topic, sizeof(out->tx_largest_topic),
                tx_largest_topic);

  out->tx_last_reject_sequence = tx_last_reject_sequence;
  out->tx_last_reject_traffic = tx_last_reject_traffic;
  out->tx_last_reject_json_bytes = tx_last_reject_json_bytes;
  out->tx_last_reject_wire_bytes = tx_last_reject_wire_bytes;
  out->tx_last_reject_reason_id = tx_last_reject_reason_id;
  tx_copy_topic(out->tx_last_reject_topic,
                sizeof(out->tx_last_reject_topic), tx_last_reject_topic);

  out->tx_last_completed_sequence = tx_last_completed_sequence;
  out->tx_last_completed_traffic = tx_last_completed_traffic;
  out->tx_last_completed_json_bytes = tx_last_completed_json_bytes;
  out->tx_last_completed_wire_bytes = tx_last_completed_wire_bytes;
  tx_copy_topic(out->tx_last_completed_topic,
                sizeof(out->tx_last_completed_topic), tx_last_completed_topic);

  out->tx_timebase_attempt_count = tx_timebase_attempt_count;
  out->tx_timebase_serialized_count = tx_timebase_serialized_count;
  out->tx_timebase_reject_count = tx_timebase_reject_count;
  out->tx_timebase_enqueued_count = tx_timebase_enqueued_count;
  out->tx_timebase_completed_count = tx_timebase_completed_count;
  out->tx_timebase_last_sequence = tx_timebase_last_sequence;
  out->tx_timebase_last_json_bytes = tx_timebase_last_json_bytes;
  out->tx_timebase_last_wire_bytes = tx_timebase_last_wire_bytes;
  out->tx_timebase_last_outcome_id = tx_timebase_last_outcome_id;
  out->tx_timebase_last_reason_id = tx_timebase_last_reason_id;
  out->tx_timebase_largest_json_bytes = tx_timebase_largest_json_bytes;
  out->tx_timebase_largest_wire_bytes = tx_timebase_largest_wire_bytes;

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
  out->rx_dispatch_invalid_callback = rx_dispatch_invalid_callback;
  out->rx_dispatch_stack_mismatch = rx_dispatch_stack_mismatch;

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

  rx_dispatch_breadcrumb_boot_latch();
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
