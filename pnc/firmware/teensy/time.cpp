#include "time.h"
#include "process_clocks.h"

#include <Arduino.h>
#include "imxrt.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.cpp -- canonical TIME conversion and projection owner
// ============================================================================
//
// process_time.cpp/process_time.h have been retired.  This file owns the TIME
// clock projection state, PPS/VCLOCK compatibility anchor, and DWT<->clock
// nanosecond conversion helpers.
//
// Canonical conversion doctrine:
//   * TIME does not sample ambient DWT for canonical conversions.
//   * Callers supply authored DWT coordinates captured by interrupt custody.
//   * Legacy "now" helpers remain for diagnostics/compatibility only.
//
// CLOCKS/Alpha feeds this module event facts through time_clock_epoch_reset(),
// time_clock_update(), time_pps_vclock_epoch_reset(), and
// time_pps_vclock_update().  TIME performs projection only.
// ============================================================================

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t MAX_AGE_NS = 3000000000ULL;
static constexpr uint32_t TIME_CLOCK_SLOT_COUNT = 4;  // index by time_clock_id_t value

static inline void time_dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

// ============================================================================
// Generalized clock projection state
// ============================================================================

struct time_clock_state_t {
  volatile uint32_t seq = 0;
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
  if (v == (uint8_t)time_clock_id_t::OCXO1)  return 2;
  if (v == (uint8_t)time_clock_id_t::OCXO2)  return 3;
  return -1;
}

static bool time_clock_load(time_clock_id_t clock, time_clock_snapshot_t& out) {
  out = time_clock_snapshot_t{};
  const int idx = time_clock_index(clock);
  if (idx < 0) return false;

  const time_clock_state_t& c = time_clocks[idx];
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = c.seq;
    time_dmb();

    out.valid = c.valid;
    out.prediction_valid = c.prediction_valid;
    out.dwt_at_update = c.dwt_at_update;
    out.ns_at_update = c.ns_at_update;
    out.predicted_dwt_cycles_per_second = c.predicted_dwt_cycles_per_second;
    out.update_count = c.update_count;
    out.last_observed_dwt_cycles = c.last_observed_dwt_cycles;
    out.last_observed_ns = c.last_observed_ns;
    out.last_prediction_residual_cycles = c.last_prediction_residual_cycles;

    time_dmb();
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
    time_dmb();

    c.valid = false;
    c.prediction_valid = false;
    c.dwt_at_update = 0;
    c.ns_at_update = 0;
    c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
    c.update_count = 0;
    c.last_observed_dwt_cycles = 0;
    c.last_observed_ns = 0;
    c.last_prediction_residual_cycles = 0;

    time_dmb();
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
  time_dmb();

  c.valid = true;
  c.prediction_valid = true;
  c.dwt_at_update = dwt_at_update;
  c.ns_at_update = ns_at_update;
  c.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  c.update_count = 0;
  c.last_observed_dwt_cycles = 0;
  c.last_observed_ns = 0;
  c.last_prediction_residual_cycles = 0;

  time_dmb();
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
  time_dmb();

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

  time_dmb();
  c.seq++;
  return true;
}

bool time_clock_snapshot(time_clock_id_t clock,
                         time_clock_snapshot_t* out) {
  if (!out) return false;
  return time_clock_load(clock, *out);
}

// ============================================================================
// PPS/VCLOCK compatibility anchor
// ============================================================================

struct time_anchor_t {
  volatile uint32_t seq = 0;
  volatile uint32_t dwt_at_pps_vclock = 0;
  volatile uint32_t dwt_cycles_per_pps_vclock_s = 0;
  volatile uint32_t counter32_at_pps_vclock = 0;
  volatile uint32_t pps_vclock_count = 0;
  volatile bool     valid = false;
};

static time_anchor_t anchor = {};

struct prediction_state_t {
  volatile uint32_t seq = 0;
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
  time_dmb();

  prediction.valid = false;
  prediction.pps_vclock_count = 0;
  prediction.predicted_cycles_last = 0;
  prediction.actual_cycles_last = 0;
  prediction.residual_cycles_last = 0;
  prediction.predicted_cycles_next = 0;

  time_dmb();
  prediction.seq++;
}

static void prediction_observe_actual(uint32_t pps_vclock_count,
                                      uint32_t actual_cycles) {
  if (actual_cycles == 0) return;

  prediction.seq++;
  time_dmb();

  const bool had_prediction = prediction.predicted_cycles_next != 0;
  const uint32_t predicted_for_this_second = prediction.predicted_cycles_next;
  const int32_t residual = had_prediction
      ? (int32_t)((int64_t)actual_cycles - (int64_t)predicted_for_this_second)
      : 0;

  prediction.pps_vclock_count = pps_vclock_count;
  prediction.predicted_cycles_last = predicted_for_this_second;
  prediction.actual_cycles_last = actual_cycles;
  prediction.residual_cycles_last = residual;
  prediction.predicted_cycles_next = actual_cycles;
  prediction.valid = had_prediction;

  time_dmb();
  prediction.seq++;
}

void time_pps_vclock_epoch_reset(uint32_t dwt_at_pps_vclock,
                                 uint32_t counter32_at_pps_vclock) {
  anchor.seq++;
  time_dmb();

  anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = 0;
  anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
  anchor.pps_vclock_count            = 1;
  anchor.valid                       = false;

  time_dmb();
  anchor.seq++;

  prediction_reset();
}

static void time_pps_vclock_update_explicit(uint32_t dwt_at_pps_vclock,
                                            uint32_t dwt_cycles_per_pps_vclock_s,
                                            uint32_t counter32_at_pps_vclock,
                                            uint32_t pps_vclock_count) {
  const uint32_t new_pps_vclock_count = pps_vclock_count ? pps_vclock_count : 1U;

  anchor.seq++;
  time_dmb();

  anchor.dwt_at_pps_vclock           = dwt_at_pps_vclock;
  anchor.dwt_cycles_per_pps_vclock_s = dwt_cycles_per_pps_vclock_s;
  anchor.counter32_at_pps_vclock     = counter32_at_pps_vclock;
  anchor.pps_vclock_count            = new_pps_vclock_count;
  anchor.valid                       = dwt_cycles_per_pps_vclock_s > 0;

  time_dmb();
  anchor.seq++;

  if (dwt_cycles_per_pps_vclock_s > 0) {
    prediction_observe_actual(new_pps_vclock_count,
                              dwt_cycles_per_pps_vclock_s);
  }
}

// Historical 3-arg order used by Alpha:
//   dwt, cycles_per_second, counter32
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t dwt_cycles_per_pps_vclock_s,
                            uint32_t counter32_at_pps_vclock) {
  const uint32_t next_count = anchor.pps_vclock_count
      ? (anchor.pps_vclock_count + 1U)
      : 1U;
  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  next_count);
}

// Explicit-count overload:
//   dwt, counter32, count, cycles_per_second
void time_pps_vclock_update(uint32_t dwt_at_pps_vclock,
                            uint32_t counter32_at_pps_vclock,
                            uint32_t pps_vclock_count,
                            uint32_t dwt_cycles_per_pps_vclock_s) {
  time_pps_vclock_update_explicit(dwt_at_pps_vclock,
                                  dwt_cycles_per_pps_vclock_s,
                                  counter32_at_pps_vclock,
                                  pps_vclock_count);
}

struct time_anchor_read_t {
  uint32_t dwt_at_pps_vclock = 0;
  uint32_t dwt_cycles_per_pps_vclock_s = 0;
  uint32_t counter32_at_pps_vclock = 0;
  uint32_t pps_vclock_count = 0;
  bool     valid = false;
  bool     ok = false;
};

static time_anchor_read_t read_anchor(void) {
  time_anchor_read_t s{};
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = anchor.seq;
    time_dmb();

    s.dwt_at_pps_vclock           = anchor.dwt_at_pps_vclock;
    s.dwt_cycles_per_pps_vclock_s = anchor.dwt_cycles_per_pps_vclock_s;
    s.counter32_at_pps_vclock     = anchor.counter32_at_pps_vclock;
    s.pps_vclock_count            = anchor.pps_vclock_count;
    s.valid                       = anchor.valid;

    time_dmb();
    const uint32_t s2 = anchor.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) {
      s.ok = true;
      return s;
    }
  }
  return time_anchor_read_t{};
}

time_anchor_snapshot_t time_anchor_snapshot(void) {
  const time_anchor_read_t s = read_anchor();
  time_anchor_snapshot_t pub{};
  pub.dwt_at_pps_vclock = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_pps_vclock_s = s.dwt_cycles_per_pps_vclock_s;
  pub.counter32_at_pps_vclock = s.counter32_at_pps_vclock;
  pub.pps_vclock_count = s.pps_vclock_count;
  pub.valid = s.valid;
  pub.ok = s.ok;

  pub.dwt_at_pps = s.dwt_at_pps_vclock;
  pub.dwt_cycles_per_s = s.dwt_cycles_per_pps_vclock_s;
  pub.qtimer_at_pps = s.counter32_at_pps_vclock;
  pub.pps_count = s.pps_vclock_count;
  return pub;
}

// ============================================================================
// Projection helpers
// ============================================================================

static uint64_t time_lane_fallback_ns(time_clock_id_t clock) {
  if (clock == time_clock_id_t::OCXO1) return clocks_ocxo1_measured_gnss_ns_now();
  if (clock == time_clock_id_t::OCXO2) return clocks_ocxo2_measured_gnss_ns_now();
  return clocks_gnss_ns_now();
}

static uint32_t time_lane_dwt_cycles_per_second(time_clock_id_t clock,
                                                const time_clock_snapshot_t& basis) {
  if (basis.prediction_valid && basis.predicted_dwt_cycles_per_second != 0) {
    return basis.predicted_dwt_cycles_per_second;
  }

  clocks_static_prediction_snapshot_t pred{};
  if (clocks_static_prediction_snapshot(clock, &pred)) {
    if (pred.actual_cycles != 0) return pred.actual_cycles;
    if (pred.static_prediction_cycles != 0) return pred.static_prediction_cycles;
  }

  const uint32_t dwt_cps = clocks_dwt_cycles_per_gnss_second();
  return dwt_cps ? dwt_cps : (uint32_t)DWT_EXPECTED_PER_PPS;
}

static bool time_build_projection(time_clock_id_t clock,
                                  time_clock_projection_t& out) {
  out = time_clock_projection_t{};
  out.clock = clock;

  time_clock_snapshot_t basis{};
  const bool live = time_clock_snapshot(clock, &basis) && basis.valid;

  if (live) {
    out.valid = true;
    out.basis_is_live = true;
    out.dwt_at_update = basis.dwt_at_update;
    out.ns_at_update = basis.ns_at_update;
    out.dwt_cycles_per_second = time_lane_dwt_cycles_per_second(clock, basis);
    out.update_count = basis.update_count;
    out.last_observed_dwt_cycles = basis.last_observed_dwt_cycles;
    out.last_observed_ns = basis.last_observed_ns;
    out.last_prediction_residual_cycles = basis.last_prediction_residual_cycles;
    return out.dwt_cycles_per_second != 0;
  }

  out.valid = false;
  out.basis_is_live = false;
  out.dwt_at_update = 0;
  out.ns_at_update = time_lane_fallback_ns(clock);
  out.dwt_cycles_per_second = time_lane_dwt_cycles_per_second(clock, basis);
  return false;
}

bool time_clock_projection(time_clock_id_t clock,
                           time_clock_projection_t* out) {
  if (!out) return false;
  return time_build_projection(clock, *out);
}

static int64_t time_mul_div_round_i64(int64_t value,
                                      int64_t multiplier,
                                      int64_t divisor) {
  const int64_t product = value * multiplier;
  if (product >= 0) return (product + divisor / 2) / divisor;
  return -(((-product) + divisor / 2) / divisor);
}

static int64_t time_cycles_to_ns_delta(int32_t cycles,
                                       uint32_t dwt_cycles_per_second) {
  return time_mul_div_round_i64((int64_t)cycles,
                                (int64_t)TIME_NS_PER_SECOND_U64,
                                (int64_t)dwt_cycles_per_second);
}

static int64_t time_ns_to_cycles_delta(int64_t ns,
                                       uint32_t dwt_cycles_per_second) {
  const int64_t whole_seconds = ns / (int64_t)TIME_NS_PER_SECOND_U64;
  const int64_t remainder_ns = ns % (int64_t)TIME_NS_PER_SECOND_U64;

  return whole_seconds * (int64_t)dwt_cycles_per_second +
         time_mul_div_round_i64(remainder_ns,
                                (int64_t)dwt_cycles_per_second,
                                (int64_t)TIME_NS_PER_SECOND_U64);
}

bool time_clock_ns_at_dwt(time_clock_id_t clock,
                          uint32_t authored_dwt_cycle_count,
                          uint64_t* out_ns) {
  if (!out_ns) return false;

  time_clock_projection_t p{};
  if (!time_build_projection(clock, p) || !p.valid || p.dwt_cycles_per_second == 0) {
    *out_ns = p.ns_at_update;
    return false;
  }

  const int32_t dwt_delta = (int32_t)(authored_dwt_cycle_count - p.dwt_at_update);
  const int64_t ns_delta = time_cycles_to_ns_delta(dwt_delta,
                                                   p.dwt_cycles_per_second);
  *out_ns = (uint64_t)((int64_t)p.ns_at_update + ns_delta);
  return true;
}

bool time_clock_dwt_at_ns(time_clock_id_t clock,
                          uint64_t clock_ns,
                          uint32_t* out_dwt_cycle_count) {
  if (!out_dwt_cycle_count) return false;

  time_clock_projection_t p{};
  if (!time_build_projection(clock, p) || !p.valid || p.dwt_cycles_per_second == 0) {
    *out_dwt_cycle_count = 0;
    return false;
  }

  const int64_t ns_delta = (clock_ns >= p.ns_at_update)
      ? (int64_t)(clock_ns - p.ns_at_update)
      : -(int64_t)(p.ns_at_update - clock_ns);

  const int64_t cycle_delta = time_ns_to_cycles_delta(ns_delta,
                                                      p.dwt_cycles_per_second);
  *out_dwt_cycle_count = p.dwt_at_update + (uint32_t)cycle_delta;
  return true;
}

uint64_t time_dwt_to_clock_ns(time_clock_id_t clock,
                              uint32_t authored_dwt_cycle_count) {
  uint64_t ns = 0;
  (void)time_clock_ns_at_dwt(clock, authored_dwt_cycle_count, &ns);
  return ns;
}

uint32_t time_clock_ns_to_dwt(time_clock_id_t clock,
                              uint64_t clock_ns) {
  uint32_t dwt = 0;
  (void)time_clock_dwt_at_ns(clock, clock_ns, &dwt);
  return dwt;
}

int64_t time_dwt_to_gnss_ns(uint32_t authored_dwt_cycle_count) {
  return (int64_t)time_dwt_to_clock_ns(time_clock_id_t::GNSS,
                                       authored_dwt_cycle_count);
}

uint64_t time_dwt_to_vclock_ns(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::VCLOCK,
                              authored_dwt_cycle_count);
}

uint64_t time_dwt_to_ocxo1_ns(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::OCXO1,
                              authored_dwt_cycle_count);
}

uint64_t time_dwt_to_ocxo2_ns(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::OCXO2,
                              authored_dwt_cycle_count);
}

uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  if (gnss_ns < 0) return 0;
  return time_clock_ns_to_dwt(time_clock_id_t::GNSS, (uint64_t)gnss_ns);
}

uint32_t time_vclock_ns_to_dwt(uint64_t vclock_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::VCLOCK, vclock_ns);
}

uint32_t time_ocxo1_ns_to_dwt(uint64_t ocxo1_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::OCXO1, ocxo1_ns);
}

uint32_t time_ocxo2_ns_to_dwt(uint64_t ocxo2_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::OCXO2, ocxo2_ns);
}

// Explicit-anchor compatibility overload used by older bridge code.
int64_t time_dwt_to_gnss_ns(uint32_t dwt_cyccnt,
                            uint32_t anchor_dwt_at_pps_vclock,
                            uint32_t anchor_dwt_cycles_per_pps_vclock_s,
                            uint32_t anchor_pps_vclock_count) {
  if (anchor_dwt_cycles_per_pps_vclock_s == 0 || anchor_pps_vclock_count == 0) {
    return -1;
  }

  const uint32_t dwt_elapsed = dwt_cyccnt - anchor_dwt_at_pps_vclock;
  const uint64_t ns_into_second =
      ((uint64_t)dwt_elapsed * TIME_NS_PER_SECOND_U64 +
       (uint64_t)anchor_dwt_cycles_per_pps_vclock_s / 2ULL) /
      (uint64_t)anchor_dwt_cycles_per_pps_vclock_s;
  if (ns_into_second > MAX_AGE_NS) return -1;

  return (int64_t)((uint64_t)(anchor_pps_vclock_count - 1) * TIME_NS_PER_SECOND_U64 +
                   ns_into_second);
}

// Legacy diagnostic-only "now" surface.  Canonical users must pass an authored
// DWT coordinate to time_clock_ns_at_dwt()/time_dwt_to_gnss_ns().
int64_t time_gnss_ns_now(void) {
  return time_dwt_to_gnss_ns(ARM_DWT_CYCCNT);
}

bool time_valid(void) {
  time_clock_snapshot_t s{};
  return time_clock_snapshot(time_clock_id_t::VCLOCK, &s) &&
         s.valid && s.prediction_valid && s.predicted_dwt_cycles_per_second != 0;
}

uint32_t time_pps_count(void) {
  const time_anchor_read_t s = read_anchor();
  return s.ok ? s.pps_vclock_count : 0;
}

// ============================================================================
// Retired prediction/dynamic-CPS compatibility surfaces
// ============================================================================

time_dwt_prediction_snapshot_t time_dwt_prediction_snapshot(void) {
  time_dwt_prediction_snapshot_t out{};
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = prediction.seq;
    time_dmb();

    out.valid = prediction.valid;
    out.pps_vclock_count = prediction.pps_vclock_count;
    out.predicted_cycles_last = prediction.predicted_cycles_last;
    out.actual_cycles_last = prediction.actual_cycles_last;
    out.residual_cycles_last = prediction.residual_cycles_last;
    out.predicted_cycles_next = prediction.predicted_cycles_next;
    out.history_count = 0;
    out.history_capacity = 0;

    time_dmb();
    const uint32_t s2 = prediction.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }
  return time_dwt_prediction_snapshot_t{};
}

uint32_t time_dwt_prediction_history(time_dwt_prediction_record_t*, uint32_t) { return 0; }
bool     time_dwt_prediction_valid(void) { return time_dwt_prediction_snapshot().valid; }
uint32_t time_dwt_actual_cycles_last_second(void) { return time_dwt_prediction_snapshot().actual_cycles_last; }
uint32_t time_dwt_predicted_cycles_last_second(void) { return time_dwt_prediction_snapshot().predicted_cycles_last; }
uint32_t time_dwt_next_prediction_cycles(void) { return time_dwt_prediction_snapshot().predicted_cycles_next; }
int32_t  time_dwt_prediction_residual_cycles(void) { return time_dwt_prediction_snapshot().residual_cycles_last; }

void time_dynamic_cps_reset(void) {}
void time_dynamic_cps_phase_probe_update(uint32_t, uint32_t, uint32_t, uint32_t,
                                         uint32_t, uint32_t, uint32_t, uint32_t) {}
void time_dynamic_cps_pps_vclock_edge(uint32_t, uint32_t) {}
void time_dynamic_cps_cadence_update(uint32_t, uint32_t) {}

time_dynamic_cps_snapshot_t time_dynamic_cps_snapshot(void) {
  const time_anchor_read_t s = read_anchor();
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

uint32_t time_dynamic_cps_history(time_dynamic_cps_record_t*, uint32_t) { return 0; }
bool time_prediction_detail_snapshot(time_prediction_detail_snapshot_t* out) {
  if (out) *out = time_prediction_detail_snapshot_t{};
  return false;
}
uint32_t time_dynamic_cps_current(void) { return time_dynamic_cps_snapshot().current_cycles; }
bool time_dynamic_cps_for_pvc_sequence(uint32_t pvc_sequence, uint32_t* out_cycles) {
  if (!out_cycles) return false;
  const time_anchor_read_t s = read_anchor();
  if (!s.ok || !s.valid || s.dwt_cycles_per_pps_vclock_s == 0) return false;
  if (s.pps_vclock_count != pvc_sequence) return false;
  *out_cycles = s.dwt_cycles_per_pps_vclock_s;
  return true;
}

void time_init(void) {
  anchor = time_anchor_t{};
  prediction_reset();
  time_clock_reset_all();
}
