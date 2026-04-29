// ============================================================================
// process_time.cpp — VCLOCK-authored GNSS Nanosecond Interface + Process
// ============================================================================
//
// PPS is not the timebase here. PPS selects a VCLOCK edge; the selected
// PPS/VCLOCK edge becomes the canonical anchor. DWT is then used only as a
// high-resolution ruler to interpolate from that selected VCLOCK edge.
//
// This module performs NO latency adjustment. All DWT inputs are already
// event-coordinate values.
//
// process_time owns:
//   • PPS/VCLOCK anchor state for DWT↔GNSS interpolation
//   • DWT next-second prediction state
//   • dynamic CPS refinement state
//   • command/report surfaces for compact state and bounded history reports
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
#include <stdlib.h>

static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;
static constexpr uint32_t PREDICTION_HISTORY_CAPACITY = 32;
static constexpr uint32_t DYNAMIC_CPS_HISTORY_CAPACITY = 32;
static constexpr uint32_t HISTORY_DEFAULT_LIMIT = 16;

static constexpr uint64_t DYNAMIC_CPS_MIN_REFINE_OFFSET_NS = 10000000ULL;
static constexpr uint64_t DYNAMIC_CPS_MAX_REFINE_OFFSET_NS = 990000000ULL;
static constexpr int32_t  DYNAMIC_CPS_MAX_STEP_CYCLES      = 128;
static constexpr uint32_t DYNAMIC_CPS_BLEND_SHIFT          = 4;

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
// Dynamic CPS state
// ============================================================================

struct dynamic_cps_state_t {
  volatile uint32_t seq;

  bool     valid = false;
  uint32_t pvc_sequence = 0;
  uint32_t current_pvc_dwt_at_edge = 0;
  uint32_t base_cycles = 0;
  uint32_t current_cycles = 0;
  uint32_t last_reseed_value = 0;
  bool     last_reseed_was_computed = false;

  uint32_t last_inferred_cycles = 0;
  int32_t  last_cps_error = 0;
  int32_t  last_cps_step = 0;
  uint64_t last_gnss_offset_ns = 0;

  uint32_t refine_ticks_this_second = 0;
  uint32_t adjustments_this_second = 0;
  uint32_t total_refine_ticks = 0;
  uint32_t total_adjustments = 0;
  uint32_t skipped_no_anchor = 0;
  uint32_t skipped_no_cps = 0;
  uint32_t skipped_offset_low = 0;
  uint32_t skipped_offset_high = 0;

  uint32_t history_head = 0;
  uint32_t history_count = 0;
  time_dynamic_cps_record_t history[DYNAMIC_CPS_HISTORY_CAPACITY] = {};
};

static dynamic_cps_state_t dynamic_cps = {};

static void dynamic_cps_push_record_locked(void) {
  if (dynamic_cps.pvc_sequence == 0) return;

  time_dynamic_cps_record_t rec{};
  rec.pvc_sequence = dynamic_cps.pvc_sequence;
  rec.pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
  rec.base_cycles = dynamic_cps.base_cycles;
  rec.final_cycles = dynamic_cps.current_cycles;
  rec.net_adjustment_cycles =
      (int32_t)((int64_t)dynamic_cps.current_cycles - (int64_t)dynamic_cps.base_cycles);
  rec.refine_ticks = dynamic_cps.refine_ticks_this_second;
  rec.adjustments = dynamic_cps.adjustments_this_second;
  rec.last_inferred_cycles = dynamic_cps.last_inferred_cycles;
  rec.last_cps_error = dynamic_cps.last_cps_error;
  rec.last_cps_step = dynamic_cps.last_cps_step;
  rec.last_gnss_offset_ns = dynamic_cps.last_gnss_offset_ns;
  rec.valid = dynamic_cps.valid;

  dynamic_cps.history[dynamic_cps.history_head] = rec;
  dynamic_cps.history_head = (dynamic_cps.history_head + 1U) % DYNAMIC_CPS_HISTORY_CAPACITY;
  if (dynamic_cps.history_count < DYNAMIC_CPS_HISTORY_CAPACITY) {
    dynamic_cps.history_count++;
  }
}

void time_dynamic_cps_reset(void) {
  dynamic_cps.seq++;
  dmb();

  dynamic_cps.valid = false;
  dynamic_cps.pvc_sequence = 0;
  dynamic_cps.current_pvc_dwt_at_edge = 0;
  dynamic_cps.base_cycles = 0;
  dynamic_cps.current_cycles = 0;
  dynamic_cps.last_reseed_value = 0;
  dynamic_cps.last_reseed_was_computed = false;
  dynamic_cps.last_inferred_cycles = 0;
  dynamic_cps.last_cps_error = 0;
  dynamic_cps.last_cps_step = 0;
  dynamic_cps.last_gnss_offset_ns = 0;
  dynamic_cps.refine_ticks_this_second = 0;
  dynamic_cps.adjustments_this_second = 0;
  dynamic_cps.total_refine_ticks = 0;
  dynamic_cps.total_adjustments = 0;
  dynamic_cps.skipped_no_anchor = 0;
  dynamic_cps.skipped_no_cps = 0;
  dynamic_cps.skipped_offset_low = 0;
  dynamic_cps.skipped_offset_high = 0;
  dynamic_cps.history_head = 0;
  dynamic_cps.history_count = 0;
  for (uint32_t i = 0; i < DYNAMIC_CPS_HISTORY_CAPACITY; i++) {
    dynamic_cps.history[i] = time_dynamic_cps_record_t{};
  }

  dmb();
  dynamic_cps.seq++;
}

void time_dynamic_cps_pps_vclock_edge(uint32_t pvc_sequence,
                                      uint32_t pvc_dwt_at_edge) {
  dynamic_cps.seq++;
  dmb();

  dynamic_cps_push_record_locked();

  const bool have_prev = (dynamic_cps.pvc_sequence != 0);
  const uint32_t base = have_prev
      ? (uint32_t)(pvc_dwt_at_edge - dynamic_cps.current_pvc_dwt_at_edge)
      : 0;

  dynamic_cps.pvc_sequence = pvc_sequence;
  dynamic_cps.current_pvc_dwt_at_edge = pvc_dwt_at_edge;
  dynamic_cps.base_cycles = base;
  dynamic_cps.current_cycles = base;
  dynamic_cps.last_reseed_value = base;
  dynamic_cps.last_reseed_was_computed = have_prev;
  dynamic_cps.valid = have_prev && base != 0;

  dynamic_cps.last_inferred_cycles = 0;
  dynamic_cps.last_cps_error = 0;
  dynamic_cps.last_cps_step = 0;
  dynamic_cps.last_gnss_offset_ns = 0;
  dynamic_cps.refine_ticks_this_second = 0;
  dynamic_cps.adjustments_this_second = 0;

  dmb();
  dynamic_cps.seq++;
}

void time_dynamic_cps_cadence_update(uint32_t qtimer_event_dwt,
                                     uint32_t cadence_tick_mod_1000) {
  dynamic_cps.seq++;
  dmb();

  if (dynamic_cps.pvc_sequence == 0) {
    dynamic_cps.skipped_no_anchor++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  if (!dynamic_cps.valid || dynamic_cps.current_cycles == 0) {
    dynamic_cps.skipped_no_cps++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint64_t gnss_offset_ns = (uint64_t)cadence_tick_mod_1000 * NS_PER_MILLISECOND;

  if (gnss_offset_ns < DYNAMIC_CPS_MIN_REFINE_OFFSET_NS) {
    dynamic_cps.skipped_offset_low++;
    dmb();
    dynamic_cps.seq++;
    return;
  }
  if (gnss_offset_ns >= NS_PER_SECOND ||
      gnss_offset_ns > DYNAMIC_CPS_MAX_REFINE_OFFSET_NS) {
    dynamic_cps.skipped_offset_high++;
    dmb();
    dynamic_cps.seq++;
    return;
  }

  const uint32_t observed_cycles = qtimer_event_dwt - dynamic_cps.current_pvc_dwt_at_edge;
  const uint32_t inferred_cps =
      (uint32_t)(((uint64_t)observed_cycles * (uint64_t)NS_PER_SECOND +
                  gnss_offset_ns / 2ULL) /
                 gnss_offset_ns);

  int32_t cps_error = (int32_t)((int64_t)inferred_cps - (int64_t)dynamic_cps.current_cycles);
  int32_t cps_step  = cps_error >> DYNAMIC_CPS_BLEND_SHIFT;
  if (cps_step == 0 && cps_error != 0) {
    cps_step = (cps_error > 0) ? 1 : -1;
  }
  if (cps_step >  DYNAMIC_CPS_MAX_STEP_CYCLES) cps_step =  DYNAMIC_CPS_MAX_STEP_CYCLES;
  if (cps_step < -DYNAMIC_CPS_MAX_STEP_CYCLES) cps_step = -DYNAMIC_CPS_MAX_STEP_CYCLES;

  int64_t next_cycles = (int64_t)dynamic_cps.current_cycles + (int64_t)cps_step;
  if (next_cycles < 1) next_cycles = 1;
  dynamic_cps.current_cycles = (uint32_t)next_cycles;

  dynamic_cps.last_inferred_cycles = inferred_cps;
  dynamic_cps.last_cps_error = cps_error;
  dynamic_cps.last_cps_step = cps_step;
  dynamic_cps.last_gnss_offset_ns = gnss_offset_ns;
  dynamic_cps.refine_ticks_this_second++;
  dynamic_cps.total_refine_ticks++;
  if (cps_step != 0) {
    dynamic_cps.adjustments_this_second++;
    dynamic_cps.total_adjustments++;
  }

  dmb();
  dynamic_cps.seq++;
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
  time_dynamic_cps_reset();
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

static uint32_t effective_cycles_for_anchor(uint32_t anchor_dwt_at_pps_vclock,
                                            uint32_t raw_cycles) {
  const time_dynamic_cps_snapshot_t cps = time_dynamic_cps_snapshot();
  if (cps.valid && cps.current_cycles != 0 &&
      cps.current_pvc_dwt_at_edge == anchor_dwt_at_pps_vclock) {
    return cps.current_cycles;
  }
  return raw_cycles;
}

static inline uint32_t effective_cycles_per_snapshot(const time_snapshot_t& s) {
  return effective_cycles_for_anchor(s.dwt_at_pps_vclock,
                                     s.dwt_cycles_per_pps_vclock_s);
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
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return -1;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * (uint64_t)NS_PER_SECOND +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) * (uint64_t)NS_PER_SECOND +
                   ns_into_second);
}

// ============================================================================
// Core API — forward (DWT → GNSS nanoseconds)
// ============================================================================

int64_t time_gnss_ns_now(void) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (effective_cycles_per_snapshot(s) == 0) return -1;

  const uint32_t dwt_now = ARM_DWT_CYCCNT;
  const uint32_t dwt_elapsed = dwt_now - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return -1;
  if (effective_cycles_per_snapshot(s) == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - s.dwt_at_pps_vclock;
  return interpolate_gnss_ns(s, dwt_elapsed);
}

int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count) {
  const uint32_t cycles = effective_cycles_for_anchor(anchor_dwt_at_pps_vclock,
                                                      anchor_dwt_cycles_per_pps_vclock_s);
  if (cycles == 0) return -1;
  if (anchor_pps_vclock_count == 0) return -1;

  const uint32_t dwt_elapsed = dwt_cyccnt - anchor_dwt_at_pps_vclock;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * (uint64_t)NS_PER_SECOND +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * (uint64_t)NS_PER_SECOND +
                   ns_into_second);
}

// ============================================================================
// Core API — reverse (GNSS nanoseconds → DWT)
// ============================================================================

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return 0;
  if (gnss_ns < 0) return 0;

  const int64_t anchor_ns =
      (int64_t)(s.pps_vclock_count - 1) * (int64_t)NS_PER_SECOND;
  const int64_t ns_into_second = gnss_ns - anchor_ns;
  if (ns_into_second < 0) return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second *
                  (uint64_t)cycles + (uint64_t)NS_PER_SECOND / 2ULL) /
                 (uint64_t)NS_PER_SECOND);

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
// Dynamic CPS accessors
// ============================================================================

time_dynamic_cps_snapshot_t time_dynamic_cps_snapshot(void) {
  time_dynamic_cps_snapshot_t out{};

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = dynamic_cps.seq;
    dmb();

    out.valid = dynamic_cps.valid;
    out.pvc_sequence = dynamic_cps.pvc_sequence;
    out.current_pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
    out.pvc_dwt_at_edge = dynamic_cps.current_pvc_dwt_at_edge;
    out.base_cycles = dynamic_cps.base_cycles;
    out.current_cycles = dynamic_cps.current_cycles;
    out.net_adjustment_cycles =
        (int32_t)((int64_t)dynamic_cps.current_cycles - (int64_t)dynamic_cps.base_cycles);
    out.last_reseed_value = dynamic_cps.last_reseed_value;
    out.last_reseed_was_computed = dynamic_cps.last_reseed_was_computed;
    out.last_inferred_cycles = dynamic_cps.last_inferred_cycles;
    out.last_cps_error = dynamic_cps.last_cps_error;
    out.last_cps_step = dynamic_cps.last_cps_step;
    out.last_gnss_offset_ns = dynamic_cps.last_gnss_offset_ns;
    out.refine_ticks_this_second = dynamic_cps.refine_ticks_this_second;
    out.adjustments_this_second = dynamic_cps.adjustments_this_second;
    out.total_refine_ticks = dynamic_cps.total_refine_ticks;
    out.total_adjustments = dynamic_cps.total_adjustments;
    out.skipped_no_anchor = dynamic_cps.skipped_no_anchor;
    out.skipped_no_cps = dynamic_cps.skipped_no_cps;
    out.skipped_offset_low = dynamic_cps.skipped_offset_low;
    out.skipped_offset_high = dynamic_cps.skipped_offset_high;
    out.history_count = dynamic_cps.history_count;
    out.history_capacity = DYNAMIC_CPS_HISTORY_CAPACITY;

    dmb();
    const uint32_t s2 = dynamic_cps.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }

  return time_dynamic_cps_snapshot_t{};
}

uint32_t time_dynamic_cps_history(time_dynamic_cps_record_t* out_records,
                                  uint32_t max_records) {
  if (!out_records || max_records == 0) return 0;

  time_dynamic_cps_record_t local[DYNAMIC_CPS_HISTORY_CAPACITY];
  uint32_t count = 0;
  uint32_t head = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = dynamic_cps.seq;
    dmb();

    count = dynamic_cps.history_count;
    head = dynamic_cps.history_head;
    if (count > DYNAMIC_CPS_HISTORY_CAPACITY) count = DYNAMIC_CPS_HISTORY_CAPACITY;
    for (uint32_t i = 0; i < count; i++) local[i] = dynamic_cps.history[i];

    dmb();
    const uint32_t s2 = dynamic_cps.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) break;
    if (attempt == 3) return 0;
  }

  const uint32_t n = (count < max_records) ? count : max_records;
  const uint32_t start = (count < DYNAMIC_CPS_HISTORY_CAPACITY)
      ? 0U
      : head;

  const uint32_t skip = count - n;
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t logical = skip + i;
    const uint32_t idx = (start + logical) % DYNAMIC_CPS_HISTORY_CAPACITY;
    out_records[i] = local[idx];
  }

  return n;
}

bool time_dynamic_cps_valid(void) {
  return time_dynamic_cps_snapshot().valid;
}

uint32_t time_dynamic_cps_current_cycles(void) {
  return time_dynamic_cps_snapshot().current_cycles;
}

uint32_t time_dynamic_cps_current(void) {
  return time_dynamic_cps_current_cycles();
}

bool time_dynamic_cps_cycles_for_anchor(uint32_t pvc_sequence,
                                        uint32_t* out_cycles) {
  if (!out_cycles) return false;

  const time_dynamic_cps_snapshot_t snap = time_dynamic_cps_snapshot();
  if (snap.valid && snap.pvc_sequence == pvc_sequence && snap.current_cycles != 0) {
    *out_cycles = snap.current_cycles;
    return true;
  }

  time_dynamic_cps_record_t hist[DYNAMIC_CPS_HISTORY_CAPACITY];
  const uint32_t n = time_dynamic_cps_history(hist, DYNAMIC_CPS_HISTORY_CAPACITY);
  for (uint32_t i = 0; i < n; i++) {
    if (hist[i].valid && hist[i].pvc_sequence == pvc_sequence && hist[i].final_cycles != 0) {
      *out_cycles = hist[i].final_cycles;
      return true;
    }
  }

  return false;
}

bool time_dynamic_cps_for_pvc_sequence(uint32_t pvc_sequence,
                                       uint32_t* out_cycles) {
  return time_dynamic_cps_cycles_for_anchor(pvc_sequence, out_cycles);
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
  return s.ok && s.valid && effective_cycles_per_snapshot(s) > 0;
}

void time_init(void) {
  anchor = {};
  prediction_reset();
  time_dynamic_cps_reset();
}

void process_time_init(void) {
  time_init();
}

// ============================================================================
// REPORT helpers
// ============================================================================

static uint32_t report_limit_arg(const Payload& args,
                                 uint32_t default_limit,
                                 uint32_t max_limit) {
  const char* s = args.getString("limit");
  if (!s || !*s) return default_limit;
  const uint32_t v = (uint32_t)strtoul(s, nullptr, 10);
  if (v == 0) return default_limit;
  return (v > max_limit) ? max_limit : v;
}

static void add_anchor_scalars(Payload& out) {
  const time_anchor_snapshot_t a = time_anchor_snapshot();
  out.add("time_valid", time_valid());
  out.add("anchor_ok", a.ok);
  out.add("anchor_valid", a.valid);
  out.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  out.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  out.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  out.add("pps_vclock_count", a.pps_vclock_count);
}

static void add_prediction_scalars(Payload& out) {
  const time_dwt_prediction_snapshot_t p = time_dwt_prediction_snapshot();
  out.add("prediction_valid", p.valid);
  out.add("prediction_pps_vclock_count", p.pps_vclock_count);
  out.add("predicted_cycles_last", p.predicted_cycles_last);
  out.add("actual_cycles_last", p.actual_cycles_last);
  out.add("residual_cycles_last", p.residual_cycles_last);
  out.add("predicted_cycles_next", p.predicted_cycles_next);
  out.add("prediction_history_count", p.history_count);
  out.add("prediction_history_capacity", p.history_capacity);
}

static void add_dynamic_cps_scalars(Payload& out) {
  const time_dynamic_cps_snapshot_t c = time_dynamic_cps_snapshot();
  out.add("dynamic_cps_valid", c.valid);
  out.add("dynamic_cps_pvc_sequence", c.pvc_sequence);
  out.add("dynamic_cps_current_pvc_dwt_at_edge", c.current_pvc_dwt_at_edge);
  out.add("dynamic_cps_base_cycles", c.base_cycles);
  out.add("dynamic_cps_current_cycles", c.current_cycles);
  out.add("dynamic_cps_net_adjustment_cycles", c.net_adjustment_cycles);
  out.add("dynamic_cps_last_reseed_value", c.last_reseed_value);
  out.add("dynamic_cps_last_reseed_was_computed", c.last_reseed_was_computed);
  out.add("dynamic_cps_last_inferred_cycles", c.last_inferred_cycles);
  out.add("dynamic_cps_last_cps_error", c.last_cps_error);
  out.add("dynamic_cps_last_cps_step", c.last_cps_step);
  out.add("dynamic_cps_last_gnss_offset_ns", c.last_gnss_offset_ns);
  out.add("dynamic_cps_refine_ticks_this_second", c.refine_ticks_this_second);
  out.add("dynamic_cps_adjustments_this_second", c.adjustments_this_second);
  out.add("dynamic_cps_total_refine_ticks", c.total_refine_ticks);
  out.add("dynamic_cps_total_adjustments", c.total_adjustments);
  out.add("dynamic_cps_skipped_no_anchor", c.skipped_no_anchor);
  out.add("dynamic_cps_skipped_no_cps", c.skipped_no_cps);
  out.add("dynamic_cps_skipped_offset_low", c.skipped_offset_low);
  out.add("dynamic_cps_skipped_offset_high", c.skipped_offset_high);
  out.add("dynamic_cps_history_count", c.history_count);
  out.add("dynamic_cps_history_capacity", c.history_capacity);
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_COMPACT_REPORT");
  add_anchor_scalars(out);
  add_prediction_scalars(out);
  add_dynamic_cps_scalars(out);
  return out;
}

static Payload cmd_anchor_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_ANCHOR_REPORT");
  add_anchor_scalars(out);
  return out;
}

static Payload cmd_prediction_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_DWT_PREDICTION_REPORT");
  add_prediction_scalars(out);
  return out;
}

static Payload cmd_dynamic_cps_report(const Payload&) {
  Payload out;
  out.add("model", "TIME_DYNAMIC_CPS_REPORT");
  add_dynamic_cps_scalars(out);
  return out;
}

static Payload cmd_prediction_history(const Payload& args) {
  const uint32_t limit = report_limit_arg(args, HISTORY_DEFAULT_LIMIT,
                                          PREDICTION_HISTORY_CAPACITY);
  time_dwt_prediction_record_t hist[PREDICTION_HISTORY_CAPACITY];
  const uint32_t n = time_dwt_prediction_history(hist, limit);

  Payload out;
  out.add("model", "TIME_DWT_PREDICTION_HISTORY");
  out.add("limit", limit);
  out.add("count", n);
  out.add("capacity", (uint32_t)PREDICTION_HISTORY_CAPACITY);

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

static Payload cmd_dynamic_cps_history(const Payload& args) {
  const uint32_t limit = report_limit_arg(args, HISTORY_DEFAULT_LIMIT,
                                          DYNAMIC_CPS_HISTORY_CAPACITY);
  time_dynamic_cps_record_t hist[DYNAMIC_CPS_HISTORY_CAPACITY];
  const uint32_t n = time_dynamic_cps_history(hist, limit);

  Payload out;
  out.add("model", "TIME_DYNAMIC_CPS_HISTORY");
  out.add("limit", limit);
  out.add("count", n);
  out.add("capacity", (uint32_t)DYNAMIC_CPS_HISTORY_CAPACITY);

  PayloadArray arr;
  for (uint32_t i = 0; i < n; i++) {
    Payload e;
    e.add("pvc_sequence", hist[i].pvc_sequence);
    e.add("pvc_dwt_at_edge", hist[i].pvc_dwt_at_edge);
    e.add("base_cycles", hist[i].base_cycles);
    e.add("final_cycles", hist[i].final_cycles);
    e.add("net_adjustment_cycles", hist[i].net_adjustment_cycles);
    e.add("refine_ticks", hist[i].refine_ticks);
    e.add("adjustments", hist[i].adjustments);
    e.add("last_inferred_cycles", hist[i].last_inferred_cycles);
    e.add("last_cps_error", hist[i].last_cps_error);
    e.add("last_cps_step", hist[i].last_cps_step);
    e.add("last_gnss_offset_ns", hist[i].last_gnss_offset_ns);
    e.add("valid", hist[i].valid);
    arr.add(e);
  }
  out.add_array("dynamic_cps_history", arr);
  return out;
}

static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT",              cmd_report              },
  { "ANCHOR_REPORT",       cmd_anchor_report       },
  { "PREDICTION_REPORT",   cmd_prediction_report   },
  { "PREDICTION_HISTORY",  cmd_prediction_history  },
  { "DYNAMIC_CPS_REPORT",  cmd_dynamic_cps_report  },
  { "DYNAMIC_CPS_HISTORY", cmd_dynamic_cps_history },
  { nullptr,               nullptr                 }
};

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}
