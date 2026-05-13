// ============================================================================
// process_time.cpp — legacy TIME process, static PPS/VCLOCK interpolation only
// ============================================================================
//
// Step 2 of process_time retirement.
//
// This file deliberately remains the owner of the legacy TIME anchor and
// time_clock_snapshot() state for now because that transition is operationally
// sensitive.  This iteration winds down only one more safe layer: TIME no
// longer keeps its own static DWT prediction history or exposes a prediction
// history command.  CLOCKS/Alpha owns the real static prediction audit, and
// CLOCKS/Beta publishes that audit in TIMEBASE_FRAGMENT.
//
// Retired here:
//   • dynamic CPS servo / rebasing / command surface
//   • 100 Hz / 1 kHz refinement client
//   • local TIME DWT prediction history ring
//   • prediction-history command
//
// Preserved here:
//   • time_anchor_snapshot() ownership
//   • time_valid(), time_pps_count(), time_gnss_ns_now()
//   • time_dwt_to_gnss_ns() / time_gnss_ns_to_dwt()
//   • time_clock_snapshot(), time_clock_update(), time_clock_ns_at_dwt()
//   • static/no-op compatibility symbols required by time.h callers
//
// CLOCKS/Alpha feeds this module the PPS/VCLOCK anchor and per-lane update
// facts.  TIME interpolates linearly from those facts using DWT_CYCCNT.
//
// All DWT inputs are already event-coordinate values.  This module performs no
// latency adjustment.
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

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;

// ============================================================================
// Anchor state (written by CLOCKS/Alpha, read by time readers)
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
// Generalized static clock projection state
// ============================================================================

static constexpr uint32_t TIME_CLOCK_SLOT_COUNT = 4;  // index by time_clock_id_t value

struct time_clock_state_t {
  volatile uint32_t seq;
  bool     valid = false;
  bool     prediction_valid = false;
  uint32_t dwt_at_update = 0;
  uint64_t ns_at_update = 0;
  uint32_t predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  uint32_t update_count = 0;
  uint32_t last_observed_dwt_cycles = 0;
  uint64_t last_observed_ns = 0;
  int32_t  last_prediction_residual_cycles = 0;
};

static time_clock_state_t time_clocks[TIME_CLOCK_SLOT_COUNT] = {};

static int time_clock_index(time_clock_id_t clock) {
  const uint8_t v = (uint8_t)clock;
  if (v == (uint8_t)time_clock_id_t::VCLOCK) return 1;
  if (v == (uint8_t)time_clock_id_t::OCXO1) return 2;
  if (v == (uint8_t)time_clock_id_t::OCXO2) return 3;
  return -1;
}

static bool time_clock_load(time_clock_id_t clock, time_clock_snapshot_t& out) {
  out = time_clock_snapshot_t{};
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;

  const time_clock_state_t& c = time_clocks[idx];
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = c.seq;
    dmb();

    out.valid = c.valid;
    out.prediction_valid = c.prediction_valid;
    out.dwt_at_update = c.dwt_at_update;
    out.ns_at_update = c.ns_at_update;
    out.predicted_dwt_cycles_per_second = c.predicted_dwt_cycles_per_second;
    out.update_count = c.update_count;
    out.last_observed_dwt_cycles = c.last_observed_dwt_cycles;
    out.last_observed_ns = c.last_observed_ns;
    out.last_prediction_residual_cycles = c.last_prediction_residual_cycles;

    dmb();
    const uint32_t s2 = c.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }

  out = time_clock_snapshot_t{};
  return false;
}

void time_clock_reset_all(void) {
  for (uint32_t i = 0; i < TIME_CLOCK_SLOT_COUNT; i++) {
    time_clock_state_t& c = time_clocks[i];
    c.seq++;
    dmb();

    c.valid = false;
    c.prediction_valid = false;
    c.dwt_at_update = 0;
    c.ns_at_update = 0;
    c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.update_count = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
    c.last_prediction_residual_cycles = 0;

    dmb();
    c.seq++;
  }
}

bool time_clock_epoch_reset(time_clock_id_t clock,
                            uint32_t dwt_at_update,
                            uint64_t ns_at_update) {
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;
  time_clock_state_t& c = time_clocks[idx];

  c.seq++;
  dmb();

  c.valid = true;
  c.prediction_valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  c.update_count = 0;
  c.last_observed_dwt_cycles = 0;
  c.last_observed_ns = 0;
  c.last_prediction_residual_cycles = 0;

  dmb();
  c.seq++;
  return true;
}

bool time_clock_update(time_clock_id_t clock,
                       uint32_t dwt_at_update,
                       uint64_t ns_at_update) {
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;
  time_clock_state_t& c = time_clocks[idx];

  c.seq++;
  dmb();

  if (c.valid) {
    const uint32_t observed_dwt = dwt_at_update - c.dwt_at_update;
    const uint64_t observed_ns = (ns_at_update >= c.ns_at_update)
        ? (ns_at_update - c.ns_at_update)
        : 0ULL;

    if (observed_dwt != 0 && observed_ns != 0) {
      const uint32_t observed_cps =
          (uint32_t)(((uint64_t)observed_dwt * TIME_NS_PER_SECOND_U64 +
                      observed_ns / 2ULL) /
                     observed_ns);
      c.last_prediction_residual_cycles = c.prediction_valid
          ? (int32_t)((int64_t)observed_cps -
                      (int64_t)c.predicted_dwt_cycles_per_second)
          : 0;
      c.predicted_dwt_cycles_per_second = observed_cps ? observed_cps : DWT_EXPECTED_PER_PPS;
      c.prediction_valid = true;
      c.last_observed_dwt_cycles = observed_dwt;
      c.last_observed_ns = observed_ns;
    }
  } else {
    c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.prediction_valid = true;
    c.last_prediction_residual_cycles = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
  }

  c.valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.update_count++;

  dmb();
  c.seq++;
  return true;
}

bool time_clock_ns_at_dwt_deprecated(time_clock_id_t clock,
                          uint32_t dwt_cyccnt,
                          uint64_t* out_ns) {
  if (!out_ns) return false;
  time_clock_snapshot_t s{};
  if (!time_clock_load(clock, s)) return false;
  if (!s.valid || !s.prediction_valid || s.predicted_dwt_cycles_per_second == 0) {
    return false;
  }

  const uint32_t elapsed_dwt = dwt_cyccnt - s.dwt_at_update;
  const uint64_t elapsed_ns =
      ((uint64_t)elapsed_dwt * TIME_NS_PER_SECOND_U64 +
       (uint64_t)s.predicted_dwt_cycles_per_second / 2ULL) /
      (uint64_t)s.predicted_dwt_cycles_per_second;

  *out_ns = s.ns_at_update + elapsed_ns;
  return true;
}

bool time_clock_snapshot(time_clock_id_t clock,
                         time_clock_snapshot_t* out) {
  if (!out) return false;
  return time_clock_load(clock, *out);
}

// ============================================================================
// Static DWT prediction compatibility snapshot
// ============================================================================
//
// TIME no longer owns a prediction history surface.  CLOCKS/Alpha owns the real
// per-lane static prediction audit, and CLOCKS/Beta publishes that audit in
// TIMEBASE_FRAGMENT.  Keep this tiny single-value compatibility snapshot so
// remaining time.h callers can still compile while process_time is wound down.
//
// This snapshot is intentionally local to the TIME anchor and is not a campaign
// statistical authority.  The prior completed PPS/VCLOCK interval remains the
// next prediction, but no ring/history is retained here.
// ============================================================================

struct prediction_state_t {
  volatile uint32_t seq;

  bool     valid = false;
  uint32_t pps_vclock_count = 0;
  uint32_t predicted_cycles_last = 0;
  uint32_t actual_cycles_last = 0;
  int32_t  residual_cycles_last = 0;
  uint32_t predicted_cycles_next = 0;
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

  dmb();
  prediction.seq++;
}

static void prediction_observe_actual(uint32_t pps_vclock_count,
                                      uint32_t,
                                      uint32_t,
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
  prediction.predicted_cycles_next = actual_cycles;  // static random-walk compatibility
  prediction.valid = had_prediction;

  dmb();
  prediction.seq++;
}

// ============================================================================
// Legacy dynamic-CPS compatibility symbols — retired/static implementation
// ============================================================================
//
// These symbols remain only because older interrupt/report code may still link
// against them.  They do not operate a servo, collect history, or affect
// projection.  The static TIME anchor is the only denominator used here.
// ============================================================================

void time_dynamic_cps_reset(void) {}

void time_dynamic_cps_phase_probe_update(uint32_t,
                                         uint32_t,
                                         uint32_t,
                                         uint32_t,
                                         uint32_t,
                                         uint32_t,
                                         uint32_t,
                                         uint32_t) {}

void time_dynamic_cps_pps_vclock_edge(uint32_t,
                                      uint32_t) {}

void time_dynamic_cps_cadence_update(uint32_t,
                                     uint32_t) {}

// ============================================================================
// Writer helpers
// ============================================================================

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = 0;
  anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
  anchor.pps_vclock_count            = 1;
  anchor.valid                       = false;

  dmb();
  anchor.seq++;

  prediction_reset();
}

static void time_pps_vclock_update_explicit(uint32_t dwt_at_pps_vclock,
                                             uint32_t dwt_cycles_per_pps_vclock_s,
                                             uint32_t counter32_at_pps_vclock,
                                             uint32_t pps_vclock_count) {
  const uint32_t new_pps_vclock_count = pps_vclock_count ? pps_vclock_count : 1U;

  anchor.seq++;
  dmb();

  anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
  anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
  anchor.pps_vclock_count            = new_pps_vclock_count;
  anchor.valid                       = dwt_cycles_per_pps_vclock_s > 0;

  dmb();
  anchor.seq++;

  if (dwt_cycles_per_pps_vclock_s > 0) {
    prediction_observe_actual(new_pps_vclock_count,
                              dwt_at_pps_vclock,
                              counter32_at_pps_vclock,
                              dwt_cycles_per_pps_vclock_s);
  }
}

void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  const uint32_t next_count = anchor.pps_vclock_count ?
      (anchor.pps_vclock_count + 1U) : 1U;
  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  next_count);
}

// Compatibility overload for header/call sites that carry an explicit
// PPS/VCLOCK count.  Parameter order follows the transitional time.h form:
//   dwt, counter32, count, cycles_per_second.
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t counter32_at_pps_vclock,
                            uint32_t pps_vclock_count,
                            uint32_t dwt_cycles_per_pps_vclock_s) {
  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  pps_vclock_count);
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
// Anchor readers and interpolation
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

static inline uint32_t effective_cycles_for_anchor(uint32_t,
                                                   uint32_t raw_cycles) {
  return raw_cycles;
}

static inline uint32_t effective_cycles_per_snapshot(const time_snapshot_t& s) {
  return effective_cycles_for_anchor(s.dwt_at_pps_vclock,
                                     s.dwt_cycles_per_pps_vclock_s);
}

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

static inline int64_t interpolate_gnss_ns(const time_snapshot_t& s,
                                          uint32_t dwt_elapsed) {
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return -1;

  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * TIME_NS_PER_SECOND_U64 +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(s.pps_vclock_count - 1) * TIME_NS_PER_SECOND_U64 +
                   ns_into_second);
}

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
      ((uint64_t)dwt_elapsed * TIME_NS_PER_SECOND_U64 +
       (uint64_t)cycles / 2ULL) /
      (uint64_t)cycles;

  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * TIME_NS_PER_SECOND_U64 +
                   ns_into_second);
}

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid) return 0;
  const uint32_t cycles = effective_cycles_per_snapshot(s);
  if (cycles == 0) return 0;
  if (gnss_ns < 0) return 0;

  const int64_t anchor_ns =
      (int64_t)(s.pps_vclock_count - 1) * (int64_t)TIME_NS_PER_SECOND_U64;
  const int64_t ns_into_second = gnss_ns - anchor_ns;
  if (ns_into_second < 0) return 0;

  const uint32_t dwt_elapsed =
      (uint32_t)(((uint64_t)ns_into_second *
                  (uint64_t)cycles + TIME_NS_PER_SECOND_U64 / 2ULL) /
                 TIME_NS_PER_SECOND_U64);

  return s.dwt_at_pps_vclock + dwt_elapsed;
}

// ============================================================================
// Static prediction accessors
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
    out.history_count = 0;
    out.history_capacity = 0;

    dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }

  return time_dwt_prediction_snapshot_t{};
}

uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t*,
                                     uint32_t) {
  return 0;
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
// Dynamic-CPS compatibility accessors — static aliases
// ============================================================================
//
// Expose the current static anchor through the old snapshot shape so remaining
// callers compile during the retirement window.  The reported object is
// witness-only and projection-disabled; net adjustment is always zero.
// ============================================================================

time_dynamic_cps_snapshot_t time_dynamic_cps_snapshot(void) {
  const time_snapshot_t s = read_anchor();
  time_dynamic_cps_snapshot_t out{};

  out.valid = s.ok && s.valid && s.dwt_cycles_per_pps_vclock_s != 0;
  out.witness_only = true;
  out.projection_enabled = false;
  out.pvc_sequence = s.pps_vclock_count;
  out.current_pvc_dwt_at_edge = s.dwt_at_pps_vclock;
  out.pvc_dwt_at_edge = s.dwt_at_pps_vclock;
  out.base_cycles = s.dwt_cycles_per_pps_vclock_s;
  out.current_cycles = s.dwt_cycles_per_pps_vclock_s;
  out.net_adjustment_cycles = 0;
  out.last_reseed_value = s.dwt_cycles_per_pps_vclock_s;
  out.last_reseed_was_computed = out.valid;
  out.history_count = 0;
  out.history_capacity = 0;
  return out;
}

uint32_t time_dynamic_cps_history(time_dynamic_cps_record_t*,
                                  uint32_t) {
  return 0;
}

bool time_prediction_detail_snapshot(time_prediction_detail_snapshot_t* out) {
  if (out) *out = time_prediction_detail_snapshot_t{};
  return false;
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
  const time_snapshot_t s = read_anchor();
  if (!s.ok || !s.valid || s.dwt_cycles_per_pps_vclock_s == 0) return false;
  if (s.pps_vclock_count != pvc_sequence) return false;
  *out_cycles = s.dwt_cycles_per_pps_vclock_s;
  return true;
}

bool time_dynamic_cps_for_pvc_sequence(uint32_t pvc_sequence,
                                       uint32_t* out_cycles) {
  return time_dynamic_cps_cycles_for_anchor(pvc_sequence, out_cycles);
}

// ============================================================================
// Status / init
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
  time_clock_reset_all();
}

void process_time_init(void) {
  time_init();
}

// ============================================================================
// Reduced command surface
// ============================================================================

static Payload cmd_report(const Payload&) {
  const time_anchor_snapshot_t a = time_anchor_snapshot();
  const time_dwt_prediction_snapshot_t p = time_dwt_prediction_snapshot();

  Payload out;
  out.add("model", "TIME_STEP2_STATIC_PROJECTION_REPORT");
  out.add("note", "process_time still owns anchor/projection; local prediction history retired");
  out.add("time_valid", time_valid());
  out.add("anchor_ok", a.ok);
  out.add("anchor_valid", a.valid);
  out.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  out.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  out.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  out.add("pps_vclock_count", a.pps_vclock_count);
  out.add("prediction_valid", p.valid);
  out.add("predicted_cycles_next", p.predicted_cycles_next);
  out.add("actual_cycles_last", p.actual_cycles_last);
  out.add("residual_cycles_last", p.residual_cycles_last);
  out.add("prediction_history_retired", true);
  out.add("dynamic_cps_retired", true);
  return out;
}

static Payload cmd_anchor_report(const Payload&) {
  const time_anchor_snapshot_t a = time_anchor_snapshot();
  Payload out;
  out.add("model", "TIME_STATIC_ANCHOR_REPORT");
  out.add("time_valid", time_valid());
  out.add("anchor_ok", a.ok);
  out.add("anchor_valid", a.valid);
  out.add("dwt_at_pps_vclock", a.dwt_at_pps_vclock);
  out.add("dwt_cycles_per_pps_vclock_s", a.dwt_cycles_per_pps_vclock_s);
  out.add("counter32_at_pps_vclock", a.counter32_at_pps_vclock);
  out.add("pps_vclock_count", a.pps_vclock_count);
  return out;
}

static Payload cmd_prediction_report(const Payload&) {
  const time_dwt_prediction_snapshot_t p = time_dwt_prediction_snapshot();
  Payload out;
  out.add("model", "TIME_STATIC_DWT_PREDICTION_COMPAT_REPORT");
  out.add("prediction_valid", p.valid);
  out.add("prediction_pps_vclock_count", p.pps_vclock_count);
  out.add("predicted_cycles_last", p.predicted_cycles_last);
  out.add("actual_cycles_last", p.actual_cycles_last);
  out.add("residual_cycles_last", p.residual_cycles_last);
  out.add("predicted_cycles_next", p.predicted_cycles_next);
  out.add("history_retired", true);
  return out;
}


static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT",                cmd_report               },
  { "ANCHOR_REPORT",         cmd_anchor_report        },
  { "PREDICTION_REPORT",     cmd_prediction_report    },
  { nullptr,                 nullptr                  }
};

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}
