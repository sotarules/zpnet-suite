// transport.cpp — ZPNet Transport (Teensy, USB CDC serial only)
// ----------------------------------------------------------------
//
// Wire protocol:
//   [traffic byte] <STX=json_len> JSON <ETX>
//
// TX Architecture:
//   • transport_send() serializes one semantic payload into one complete
//     wire image, including the traffic byte.
//   • An imperative runtime-loop pump performs physical transmission.
//   • The pump advances by the number of bytes actually accepted by Serial.
//   • Only transport ever touches Serial for TX.
//   • Interleave is structurally impossible.
//   • TimePop is not required for command/debug transport liveness.
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
#include "timepop.h"
#include "memory_info.h"
#include "process_interrupt.h"
#include "process_performance.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// Local scalar-only ABI exported by process_timepop.cpp for the PPS heartbeat.
// Kept here instead of widening public TimePop headers in this patch so the
// transport lifeline remains a narrow diagnostic seam.
struct timepop_health_snapshot_t {
  uint32_t active_count = 0;
  bool     pending = false;
  uint32_t dispatch_depth = 0;
  uint32_t dispatch_phase = 0;

  uint32_t isr_count = 0;
  uint32_t isr_callbacks = 0;
  uint32_t expired_count = 0;
  uint32_t dispatch_calls = 0;
  uint32_t dispatch_callbacks = 0;

  uint32_t arm_failures = 0;
  uint32_t schedule_next_calls_total = 0;
  uint32_t schedule_next_too_close_count = 0;
  uint32_t schedule_next_passed_count = 0;
  uint32_t missed_deadline_slots = 0;

  uint32_t dispatch_mutation_count = 0;
  uint32_t dispatch_mutation_overflow = 0;

  bool     idle_witness_running = false;
  uint32_t idle_witness_shadow_dwt = 0;
  uint32_t idle_witness_enter_count = 0;
  uint32_t idle_witness_exit_count = 0;
  uint32_t idle_witness_pending_exit_count = 0;
  uint32_t idle_witness_yield_count = 0;
  uint32_t idle_witness_last_residency_cycles = 0;
};

void timepop_health_snapshot(timepop_health_snapshot_t* out);

// =============================================================
// Constants
// =============================================================

static constexpr uint32_t TRANSPORT_IMPERATIVE_POLL_US = 1000UL;  // 1 ms

// USB CDC attach/reboot can deliver a tail of a Pi frame after the traffic byte
// was lost at the host/device boundary.  Until the first lawful incoming frame
// proves alignment, quarantine startup artifacts into segregated counters.
static constexpr uint32_t TRANSPORT_RX_STARTUP_GRACE_MS = 15000UL;

static constexpr size_t FRAME_SLACK = 64;
static constexpr size_t RX_BUF_MAX = TRANSPORT_MAX_MESSAGE + FRAME_SLACK;

static constexpr size_t TX_JOB_MAX = 64;
static constexpr size_t TX_BUDGET_MAX = 48 * 1024;

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

static const char* BUILD_FINGERPRINT = "__FP__ZPNET_SERIAL_CANONICAL_WIRE__";
static const char* TRANSPORT_LIFELINE_FINGERPRINT = "__FP__ZPNET_TRANSPORT_LOOP_LIFELINE__";

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
// Imperative service + D0 heartbeat counters
// =============================================================

static volatile uint32_t transport_poll_count = 0;
static volatile uint32_t transport_poll_skipped_too_soon = 0;
static volatile uint32_t transport_rx_poll_count = 0;
static volatile uint32_t transport_tx_poll_count = 0;
static volatile uint32_t transport_runtime_loop_count = 0;

static volatile uint32_t heartbeat_build_count = 0;
static volatile uint32_t heartbeat_send_count = 0;
static volatile uint32_t heartbeat_drop_pending = 0;
static volatile uint32_t heartbeat_build_fail = 0;
static volatile uint32_t heartbeat_last_edge_count = 0;
static volatile uint32_t heartbeat_bytes_sent = 0;

static uint32_t transport_last_poll_us = 0;

static char heartbeat_wire[2048] DMAMEM;
static char heartbeat_json[1800] DMAMEM;
static size_t heartbeat_wire_len = 0;
static size_t heartbeat_wire_sent = 0;
static bool heartbeat_wire_pending = false;

// =============================================================
// RX State
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
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

static FLASHMEM bool build_heartbeat_wire(const char* json, size_t json_len) {
  char header[32];
  const int header_len = snprintf(header, sizeof(header),
                                  "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0 || (size_t)header_len >= sizeof(header)) {
    heartbeat_build_fail++;
    return false;
  }

  const size_t wire_len = 1 + (size_t)header_len + json_len + ETX_LEN;
  if (wire_len > sizeof(heartbeat_wire)) {
    heartbeat_build_fail++;
    return false;
  }

  size_t pos = 0;
  heartbeat_wire[pos++] = (char)TRAFFIC_DEBUG;
  memcpy(heartbeat_wire + pos, header, (size_t)header_len);
  pos += (size_t)header_len;
  memcpy(heartbeat_wire + pos, json, json_len);
  pos += json_len;
  memcpy(heartbeat_wire + pos, ETX_SEQ, ETX_LEN);

  heartbeat_wire_len = wire_len;
  heartbeat_wire_sent = 0;
  heartbeat_wire_pending = true;
  heartbeat_build_count++;
  return true;
}

static FLASHMEM void heartbeat_build_if_new_pps(void) {
  const interrupt_pps_edge_heartbeat_t pps = interrupt_pps_edge_heartbeat();
  if (pps.edge_count == 0) return;
  if (pps.edge_count == heartbeat_last_edge_count) return;

  if (heartbeat_wire_pending) {
    heartbeat_drop_pending++;
    heartbeat_last_edge_count = pps.edge_count;
    return;
  }

  memory_info_t mem{};
  memory_info_get(&mem);

  timepop_health_snapshot_t tp{};
  timepop_health_snapshot(&tp);

  transport_info_t ti{};
  transport_get_info(&ti);

  char* json = heartbeat_json;
  const size_t json_cap = sizeof(heartbeat_json);
  const int json_len = snprintf(
      json,
      json_cap,
      "{\"name\":\"pps_heartbeat\",\"value\":{"
      "\"schema\":\"PPS_DEBUG_HEARTBEAT_V1\","
      "\"pps_edge_count\":%lu,"
      "\"pps_last_dwt\":%lu,"
      "\"pps_last_gnss_ns\":%lld,"
      "\"pps_gpio_irq_count\":%lu,"
      "\"pps_gpio_miss_count\":%lu,"
      "\"runtime_loop_count\":%lu,"
      "\"transport_poll_count\":%lu,"
      "\"transport_rx_poll_count\":%lu,"
      "\"transport_tx_poll_count\":%lu,"
      "\"tx_job_count\":%u,"
      "\"tx_jobs_enqueued\":%lu,"
      "\"tx_jobs_sent\":%lu,"
      "\"tx_bytes_enqueued\":%lu,"
      "\"tx_bytes_sent\":%lu,"
      "\"tx_budget_used\":%u,"
      "\"tx_budget_fail\":%lu,"
      "\"tx_alloc_fail\":%lu,"
      "\"tx_queue_full\":%lu,"
      "\"rx_frames_complete\":%lu,"
      "\"rx_frames_dispatched\":%lu,"
      "\"rx_reset_hard\":%lu,"
      "\"timepop_pending\":%s,"
      "\"timepop_active_count\":%lu,"
      "\"timepop_isr_count\":%lu,"
      "\"timepop_dispatch_calls\":%lu,"
      "\"timepop_dispatch_callbacks\":%lu,"
      "\"timepop_schedule_next_calls\":%lu,"
      "\"timepop_missed_deadline_slots\":%lu,"
      "\"timepop_idle_running\":%s,"
      "\"timepop_idle_enter_count\":%lu,"
      "\"timepop_idle_exit_count\":%lu,"
      "\"timepop_idle_yield_count\":%lu,"
      "\"timepop_idle_shadow_dwt\":%lu,"
      "\"heap_free_total\":%lu,"
      "\"heap_used\":%lu,"
      "\"heap_fragmentation_pct\":%lu,"
      "\"stack_free_high_water\":%lu,"
      "\"stack_collision_risk\":%s"
      "}}",
      (unsigned long)pps.edge_count,
      (unsigned long)pps.last_dwt,
      (long long)pps.last_gnss_ns,
      (unsigned long)pps.gpio_irq_count,
      (unsigned long)pps.gpio_miss_count,
      (unsigned long)transport_runtime_loop_count,
      (unsigned long)transport_poll_count,
      (unsigned long)transport_rx_poll_count,
      (unsigned long)transport_tx_poll_count,
      (unsigned)ti.tx_job_count,
      (unsigned long)ti.tx_jobs_enqueued,
      (unsigned long)ti.tx_jobs_sent,
      (unsigned long)ti.tx_bytes_enqueued,
      (unsigned long)ti.tx_bytes_sent,
      (unsigned)ti.tx_budget_used,
      (unsigned long)ti.tx_budget_fail,
      (unsigned long)ti.tx_alloc_fail,
      (unsigned long)ti.tx_queue_full,
      (unsigned long)ti.rx_frames_complete,
      (unsigned long)ti.rx_frames_dispatched,
      (unsigned long)ti.rx_reset_hard,
      tp.pending ? "true" : "false",
      (unsigned long)tp.active_count,
      (unsigned long)tp.isr_count,
      (unsigned long)tp.dispatch_calls,
      (unsigned long)tp.dispatch_callbacks,
      (unsigned long)tp.schedule_next_calls_total,
      (unsigned long)tp.missed_deadline_slots,
      tp.idle_witness_running ? "true" : "false",
      (unsigned long)tp.idle_witness_enter_count,
      (unsigned long)tp.idle_witness_exit_count,
      (unsigned long)tp.idle_witness_yield_count,
      (unsigned long)tp.idle_witness_shadow_dwt,
      (unsigned long)mem.heap_free_total,
      (unsigned long)mem.heap_used,
      (unsigned long)mem.heap_fragmentation_pct,
      (unsigned long)mem.stack_free_high_water,
      mem.stack_collision_risk ? "true" : "false");

  heartbeat_last_edge_count = pps.edge_count;
  if (json_len <= 0 || (size_t)json_len >= json_cap) {
    heartbeat_build_fail++;
    return;
  }
  (void)build_heartbeat_wire(json, (size_t)json_len);
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
  ZPNET_SERIAL.flush();

  if (n > len)
    return len;

  return n;
}

// =============================================================
// TX Pump (imperative runtime-loop, single writer)
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

static bool heartbeat_tx_in_progress() {
  return heartbeat_wire_pending && heartbeat_wire_sent < heartbeat_wire_len;
}

static void heartbeat_tx_pump_once() {
  if (!heartbeat_tx_in_progress()) return;

  const size_t remaining = heartbeat_wire_len - heartbeat_wire_sent;
  const size_t n = serial_write_some(
      (const uint8_t*)heartbeat_wire + heartbeat_wire_sent,
      remaining);
  if (n == 0) return;

  heartbeat_wire_sent += n;
  heartbeat_bytes_sent += n;
  if (heartbeat_wire_sent >= heartbeat_wire_len) {
    heartbeat_wire_pending = false;
    heartbeat_wire_len = 0;
    heartbeat_wire_sent = 0;
    heartbeat_send_count++;
  }
}

static void tx_pump_once() {

  if (tx_job_count == 0) {
    heartbeat_tx_pump_once();
    return;
  }

  tx_job_t& job = tx_jobs[tx_job_tail];

  size_t remaining = job.length - job.sent;
  size_t n = serial_write_some(job.data + job.sent, remaining);

  if (n == 0)
    return;

  job.sent += n;
  tx_bytes_sent += n;

  if (job.sent >= job.length) {
    tx_release_current_job();
  }

  // The ordinary job was never interrupted.  If the queue is now empty, a
  // pending heartbeat may use the same loop pass while preserving single-writer
  // and no-interleave semantics.
  if (tx_job_count == 0) {
    heartbeat_tx_pump_once();
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
    if (rx_buf[i] != (uint8_t)STX_SEQ[i])
      return false;
  }

  return b == (uint8_t)STX_SEQ[rx_len];
}

static inline bool rx_header_complete() {
  if (rx_len <= STX_LEN)
    return false;

  for (size_t i = STX_LEN; i < rx_len; ++i) {
    if (rx_buf[i] == (uint8_t)'>')
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

static void dispatch_if_complete() {

  if (!rx_have_traffic) {
    rx_len = 0;
    return;
  }

  if (rx_len < STX_LEN)
    return;

  if (memcmp(rx_buf, STX_SEQ, STX_LEN) != 0) {
    if (rx_startup_grace_active()) {
      rx_startup_bad_stx_suppressed++;
    } else {
      rx_bad_stx_count++;
    }
    rx_reset_hard();
    return;
  }

  size_t i = STX_LEN;
  size_t declared_len = 0;
  bool saw_digit = false;

  while (i < rx_len && rx_buf[i] != '>') {
    uint8_t c = rx_buf[i];
    if (c < '0' || c > '9') {
      if (rx_startup_grace_active()) {
        rx_startup_len_overflow_suppressed++;
      } else {
        rx_len_overflow_count++;
      }
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
    if (rx_startup_grace_active()) {
      rx_startup_len_overflow_suppressed++;
    } else {
      rx_len_overflow_count++;
    }
    rx_reset_hard();
    return;
  }

  if (rx_len < required_total)
    return;

  size_t etx_pos = json_start + declared_len;
  if (memcmp(rx_buf + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    if (rx_startup_grace_active()) {
      rx_startup_bad_etx_suppressed++;
    } else {
      rx_bad_etx_count++;
    }
    rx_reset_hard();
    return;
  }

  rx_first_frame_seen = true;
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
// RECEIVE: SERIAL RX (stream-based)
// =============================================================

static void rx_serial_tick() {

  transport_rx_entered();

  bool appended = false;

  while (ZPNET_SERIAL.available()) {
    uint8_t b = (uint8_t)ZPNET_SERIAL.read();

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

    rx_buf[rx_len++] = b;
    rx_bytes_total++;
    appended = true;

    dispatch_if_complete();
  }

  if (appended) {
    rx_blocks_total++;
  }
}

void transport_note_runtime_loop(void) {
  transport_runtime_loop_count++;
}

void transport_poll(void) {
  const uint32_t now_us = micros();
  if (transport_last_poll_us != 0 &&
      (uint32_t)(now_us - transport_last_poll_us) < TRANSPORT_IMPERATIVE_POLL_US) {
    transport_poll_skipped_too_soon++;
    return;
  }
  transport_last_poll_us = now_us;
  transport_poll_count++;

  heartbeat_build_if_new_pps();

  transport_rx_poll_count++;
  rx_serial_tick();

  transport_tx_poll_count++;
  tx_pump_once();
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
  out->poll_interval_us        = TRANSPORT_IMPERATIVE_POLL_US;
  out->heartbeat_build_count   = heartbeat_build_count;
  out->heartbeat_send_count    = heartbeat_send_count;
  out->heartbeat_drop_pending  = heartbeat_drop_pending;
  out->heartbeat_build_fail    = heartbeat_build_fail;
  out->heartbeat_last_edge_count = heartbeat_last_edge_count;
  out->heartbeat_bytes_sent    = heartbeat_bytes_sent;

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

  ZPNET_SERIAL.begin(USB_SERIAL_BAUD);
  rx_init_ms = millis();
  rx_first_frame_seen = false;
  rx_len = 0;
  rx_have_traffic = false;
  rx_traffic = 0;

  debug_log("transport", BUILD_FINGERPRINT);
  debug_log("transport", TRANSPORT_LIFELINE_FINGERPRINT);
}
