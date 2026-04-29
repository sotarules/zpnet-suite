// ============================================================================
// process_time.cpp — VCLOCK-authored GNSS Nanosecond Interface + Process
// ============================================================================
//
// PPS is not the timebase here.  PPS selects a VCLOCK edge; the selected
// PPS/VCLOCK edge becomes the canonical anchor.  DWT is then used only as a
// high-resolution ruler to interpolate from that selected VCLOCK edge.
//
// This module performs NO latency adjustment.  All DWT inputs are already
// event-coordinate values.
//
// process_time owns:
//   • PPS/VCLOCK anchor state for DWT↔GNSS interpolation
//   • DWT next-second prediction state
//   • recent prediction history and TIME.REPORT command surface
// ============================================================================

#include "process_time.h"
#include "time.h"
#include "config.h"
#include "process.h"
#include "payload.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

static constexpr uint64_t NS_PER_SEC = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;
static constexpr uint32_t PREDICTION_HISTORY_CAPACITY = 32;

// ============================================================================
// Anchor state (written by clocks alpha, read by time readers)
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq;
  volatile uint32_t dwt_at_pps_vclock;
  volatile uint32_t dwt_cycles_per_pps_vclock_s;
  volatile uint32_t counter32_at_pps_vclock;
  volatile uint32_t pps_vclock_count;
  volatile bool     valid;
};

static time_anchor_t anchor = {};

static inline void dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

// ============================================================================
// Prediction state
// ============================================================================

struct prediction_state_t {
  volatile uint32_t seq;

  bool     valid = false;
  uint32_t pps_vclock_count = 0;
  uint32_t predicted_cycles_last = 0;
  uint32_t actual_cycles_last = 0;
  int32_t  residual_cycles_last = 0;
  uint32_t predicted_cycles_next = 0;

  uint32_t history_head = 0;   // next write slot
  uint32_t history_count = 0;
  time_dwt_prediction_record_t history[PREDICTION_HISTORY_CAPACITY] = {};
};

static prediction_state_t prediction = {};

static void prediction_reset(void) {
  prediction.seq++;
  dmb();

  prediction.valid = false;
  prediction.pps_vclock_count = 0;
  prediction.predicted_cycles_last = 0;
  prediction.actual_cycles_last = 0;
  prediction.residual_cycles_last = 0;
  prediction.predicted_cycles_next = 0;
  prediction.history_head = 0;
  prediction.history_count = 0;
  for (uint32_t i = 0; i < PREDICTION_HISTORY_CAPACITY; i++) {
    prediction.history[i] = time_dwt_prediction_record_t{};
  }

  dmb();
  prediction.seq++;
}

static void prediction_push_record(const time_dwt_prediction_record_t& rec) {
  prediction.history[prediction.history_head] = rec;
  prediction.history_head = (prediction.history_head + 1U) % PREDICTION_HISTORY_CAPACITY;
  if (prediction.history_count < PREDICTION_HISTORY_CAPACITY) {
    prediction.history_count++;
  }
}

static void prediction_observe_actual(uint32_t pps_vclock_count,
                                      uint32_t dwt_at_pps_vclock,
                                      uint32_t counter32_at_pps_vclock,
                                      uint32_t actual_cycles) {
  if (actual_cycles == 0) return;

  prediction.seq++;
  dmb();

  const bool had_prediction = prediction.predicted_cycles_next != 0;
  const uint32_t predicted_for_this_second = prediction.predicted_cycles_next;
  const int32_t residual = had_prediction
      ? (int32_t)((int64_t)actual_cycles - (int64_t)predicted_for_this_second)
      : 0;

  prediction.pps_vclock_count = pps_vclock_count;
  prediction.predicted_cycles_last = predicted_for_this_second;
  prediction.actual_cycles_last = actual_cycles;
  prediction.residual_cycles_last = residual;
  prediction.predicted_cycles_next = actual_cycles;  // random-walk predictor
  prediction.valid = had_prediction;

  time_dwt_prediction_record_t rec{};
  rec.pps_vclock_count = pps_vclock_count;
  rec.dwt_at_pps_vclock = dwt_at_pps_vclock;
  rec.counter32_at_pps_vclock = counter32_at_pps_vclock;
  rec.predicted_cycles = predicted_for_this_second;
  rec.actual_cycles = actual_cycles;
  rec.residual_cycles = residual;
  rec.valid = had_prediction;
  prediction_push_record(rec);

  dmb();
  prediction.seq++;
}

// ============================================================================
// Writer helpers
// ============================================================================

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock             = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s   = 0;
  anchor.counter32_at_pps_vclock       = counter32_at_pps_vclock;
  anchor.pps_vclock_count              = 1;
  anchor.valid                         = false;

  dmb();
  anchor.seq++;

  prediction_reset();
}

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  uint32_t new_pps_vclock_count = 1;

  anchor.seq++;
  dmb();

  if (anchor.pps_vclock_count == 0) {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = 0;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count            = 1;
    anchor.valid                       = false;
    new_pps_vclock_count               = 1;
  } else {
    anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
    anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
    anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
    anchor.pps_vclock_count++;
    anchor.valid = (dwt_cycles_per_pps_vclock_s > 0);
    new_pps_vclock_count = anchor.pps_vclock_count;
  }

  dmb();
  anchor.seq++;

  if (dwt_cycles_per_pps_vclock_s > 0) {
    prediction_observe_actual(new_pps_vclock_count,
                              dwt_at_pps_vclock,
                              counter32_at_pps_vclock,
                              dwt_cycles_per_pps_vclock_s);
  }
}

void time_pps_epoch_reset(uint32_t dwt_at_pps,
                          uint32_t qtimer_at_pps) {
  time_pps_vclock_epoch_reset(dwt_at_pps, qtimer_at_pps);
}

void time_pps_update(uint32_t dwt_at_pps,
                     uint32_t dwt_cycles_per_s,
                     uint32_t qtimer_at_pps) {
  time_pps_vclock_update(dwt_at_pps, dwt_cycles_per_s, qtimer_at_pps);
}

// ============================================================================
// Internal reader — stack-local snapshot with torn-read protection
// ============================================================================

struct time_snapshot_t {
  uint32_t dwt_at_pps_vclock;
  uint32_t dwt_cycles_per_pps_vclock_s;
  uint32_t counter32_at_pps_vclock;
  uint32_t pps_vclock_count;
  bool     valid;
  bool     ok;
};

static time_snapshot_t read_anchor(void) {
  time_snapshot_t s = {};
  s.ok = false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = anchor.seq;
    dmb();

    s.dwt_at_pps_vclock           = anchor.dwt_at_pps_vclock;
    s.dwt_cycles_per_pps_vclock_s = anchor.dwt_cycles_per_pps_vclock_s;
    s.counter32_at_pps_vclock     = anchor.counter32_at_pps_vclock;
    s.pps_vclock_count            = anchor.pps_vclock_count;
    s.valid                       = anchor.valid;

    dmb();
    const uint32_t s2 = anchor.seq;

    if (s1 == s2 && (s1 & 1u) == 0u) {
      s.ok = true;
      return s;
    }
  }

  return s;
}

// ============================================================================
// Public reader — seqlock-safe anchor snapshot
// ============================================================================

time_anchor_snapshot_t time_anchor_snapshot(void) {
  const time_snapshot_t s = read_anchor();
  time_anchor_snapshot_t pub = {};

  pub.dwt_at_pps_vclock           = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = s.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock     = s.counter32_at_pps_vclock;
  pub.pps_vclock_count            = s.pps_vclock_count;
  pub.valid                       = s.valid;
  pub.ok                          = s.ok;

  // Legacy aliases.
  pub.dwt_at_pps       = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps    = s.counter32_at_pps_vclock;
  pub.pps_count        = s.pps_vclock_count;

  return pub;
}

// ============================================================================
// Internal helper — DWT elapsed to GNSS nanoseconds
// ============================================================================

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s,
                                          uint32_t dwt_elapsed) {
  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * NS_PER_SEC +
       (uint64_t)s.dwt_cycles_per_pps_vclock_s / 2ULL) /
      (uint64_t)s.dwt_cycles_per_pps_vclock_s;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) * NS_PER_SEC +
                   ns_into_second);
}

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return -1;

  const uint32_t dwt_now = ARM_DWT_CYCCNT;
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count) {
  if (anchor_dwt_cycles_per_pps_vclock_s == 0) return -1;
  if (anchor_pps_vclock_count == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - anchor_dwt_at_pps_vclock;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * NS_PER_SEC +
       (uint64_t)anchor_dwt_cycles_per_pps_vclock_s / 2ULL) /
      (uint64_t)anchor_dwt_cycles_per_pps_vclock_s;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * NS_PER_SEC +
                   ns_into_second);
}

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  if (s.dwt_cycles_per_pps_vclock_s == 0) return 0;
  if (gnss_ns < 0) return 0;

  const int64_t anchor_ns =
      (int64_t)(s.pps_vclock_count - 1) * (int64_t)NS_PER_SEC;
  const int64_t ns_into_second = gnss_ns - anchor_ns;
  if (ns_into_second < 0) return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second *
                  (uint64_t)s.dwt_cycles_per_pps_vclock_s + NS_PER_SEC / 2ULL) /
                 NS_PER_SEC);

  return s.dwt_at_pps_vclock + dwt_elapsed;
}

// ============================================================================
// DWT prediction accessors
// ============================================================================

time_dwt_prediction_snapshot_t time_dwt_prediction_snapshot(void) {
  time_dwt_prediction_snapshot_t out{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = prediction.seq;
    dmb();

    out.valid = prediction.valid;
    out.pps_vclock_count = prediction.pps_vclock_count;
    out.predicted_cycles_last = prediction.predicted_cycles_last;
    out.actual_cycles_last = prediction.actual_cycles_last;
    out.residual_cycles_last = prediction.residual_cycles_last;
    out.predicted_cycles_next = prediction.predicted_cycles_next;
    out.history_count = prediction.history_count;
    out.history_capacity = PREDICTION_HISTORY_CAPACITY;

    dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }

  return time_dwt_prediction_snapshot_t{};
}

uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t* out_records,
                                     uint32_t max_records) {
  if (!out_records || max_records == 0) return 0;

  time_dwt_prediction_record_t local[PREDICTION_HISTORY_CAPACITY];
  uint32_t count = 0;
  uint32_t head = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = prediction.seq;
    dmb();

    count = prediction.history_count;
    head = prediction.history_head;
    if (count > PREDICTION_HISTORY_CAPACITY) count = PREDICTION_HISTORY_CAPACITY;
    for (uint32_t i = 0; i < count; i++) local[i] = prediction.history[i];

    dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) break;
    if (attempt == 3) return 0;
  }

  const uint32_t n = (count < max_records) ? count : max_records;
  const uint32_t start = (count < PREDICTION_HISTORY_CAPACITY)
      ? 0U
      : head;

  // If the caller asks for fewer than all records, return the newest n while
  // preserving oldest-to-newest order.
  const uint32_t skip = count - n;
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t logical = skip + i;
    const uint32_t idx = (start + logical) % PREDICTION_HISTORY_CAPACITY;
    out_records[i] = local[idx];
  }

  return n;
}

bool time_dwt_prediction_valid(void) {
  return time_dwt_prediction_snapshot().valid;
}

uint32_t time_dwt_actual_cycles_last_second(void) {
  return time_dwt_prediction_snapshot().actual_cycles_last;
}

uint32_t time_dwt_predicted_cycles_last_second(void) {
  return time_dwt_prediction_snapshot().predicted_cycles_last;
}

uint32_t time_dwt_next_prediction_cycles(void) {
  return time_dwt_prediction_snapshot().predicted_cycles_next;
}

int32_t time_dwt_prediction_residual_cycles(void) {
  return time_dwt_prediction_snapshot().residual_cycles_last;
}

// ============================================================================
// Status
// ============================================================================

uint32_t time_pps_vclock_count(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok) return 0;
  return s.pps_vclock_count;
}

uint32_t time_pps_count(void) {
  return time_pps_vclock_count();
}

bool time_valid(void) {
  const time_snapshot_t s = read_anchor();
  return s.ok && s.valid && s.dwt_cycles_per_pps_vclock_s > 0;
}

void time_init(void) {
  anchor = {};
  prediction_reset();
}

void process_time_init(void) {
  time_init();
}

// ============================================================================
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;

  const time_anchor_snapshot_t a = time_anchor_snapshot();
  const time_dwt_prediction_snapshot_t p = time_dwt_prediction_snapshot();

  out.add("time_valid", time_valid());
  out.add("anchor_ok", a.ok);
  out.add("anchor_valid", a.valid);
  out.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  out.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  out.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  out.add("pps_vclock_count", a.pps_vclock_count);

  out.add("prediction_valid", p.valid);
  out.add("prediction_pps_vclock_count", p.pps_vclock_count);
  out.add("predicted_cycles_last", p.predicted_cycles_last);
  out.add("actual_cycles_last", p.actual_cycles_last);
  out.add("residual_cycles_last", p.residual_cycles_last);
  out.add("predicted_cycles_next", p.predicted_cycles_next);
  out.add("history_count", p.history_count);
  out.add("history_capacity", p.history_capacity);

  time_dwt_prediction_record_t hist[PREDICTION_HISTORY_CAPACITY];
  const uint32_t n = time_dwt_prediction_history(hist, PREDICTION_HISTORY_CAPACITY);

  PayloadArray arr;
  for (uint32_t i = 0; i < n; i++) {
    Payload e;
    e.add("pps_vclock_count", hist[i].pps_vclock_count);
    e.add("dwt_at_pps_vclock", hist[i].dwt_at_pps_vclock);
    e.add("counter32_at_pps_vclock", hist[i].counter32_at_pps_vclock);
    e.add("predicted_cycles", hist[i].predicted_cycles);
    e.add("actual_cycles", hist[i].actual_cycles);
    e.add("residual_cycles", hist[i].residual_cycles);
    e.add("valid", hist[i].valid);
    arr.add(e);
  }
  out.add_array("prediction_history", arr);

  return out;
}

static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    }
};

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}
