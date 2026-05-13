#include "time.h"
#include "process_clocks.h"

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// time.cpp -- stateless CLOCKS/static-prediction facade
// ============================================================================
//
// TIME owns no state here.
// TIME does not read DWT.
// TIME does not install anchors.
//
// The caller supplies an authored DWT coordinate.  The transitional
// process_time clock snapshot supplies the most recent per-lane basis;
// CLOCKS static prediction is used only as a fallback denominator.
// Gamma/dynamic prediction is retired.
// ============================================================================

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;

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

// Bridge-period definition. process_time.cpp currently defines this symbol too;
// this weak version becomes active once the legacy owner is removed.
__attribute__((weak)) int64_t time_dwt_to_gnss_ns(uint32_t authored_dwt_cycle_count) {
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

// Bridge-period definition. process_time.cpp currently defines the signed form;
// this weak version becomes active once the legacy owner is removed.
__attribute__((weak)) uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
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
