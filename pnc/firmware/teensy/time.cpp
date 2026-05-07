#include "time.h"
#include "process_clocks.h"

#include <stdint.h>

// ============================================================================
// time.cpp -- stateless CLOCKS/Gamma facade
// ============================================================================
//
// TIME owns no state here.
// TIME does not read DWT.
// TIME does not install anchors.
//
// The caller supplies a DWT coordinate. CLOCKS supplies the clock-domain
// nanosecond ledger. Gamma supplies the current lane-local DWT-cycle
// prediction. This file only performs projection math.
// ============================================================================

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;

static uint64_t time_lane_ns(time_clock_id_t clock) {
  if (clock == time_clock_id_t::OCXO1) {
    return clocks_ocxo1_measured_gnss_ns_now();
  }
  if (clock == time_clock_id_t::OCXO2) {
    return clocks_ocxo2_measured_gnss_ns_now();
  }
  return clocks_gnss_ns_now();
}

static clocks_gamma_prediction_snapshot_t time_lane_gamma(time_clock_id_t clock) {
  clocks_gamma_prediction_snapshot_t snap = {};
  (void)clocks_gamma_snapshot(clock, &snap);
  return snap;
}

static uint32_t time_lane_basis_dwt(time_clock_id_t clock) {
  const clocks_gamma_prediction_snapshot_t snap = time_lane_gamma(clock);
  return snap.second_start_dwt;
}

static uint32_t time_lane_dwt_cycles_per_second(time_clock_id_t clock) {
  const clocks_gamma_prediction_snapshot_t snap = time_lane_gamma(clock);
  return snap.current_dynamic_prediction_cycles;
}

static int64_t time_mul_div_round_i64(int64_t value,
                                      int64_t multiplier,
                                      int64_t divisor) {
  const int64_t product = value * multiplier;
  if (product >= 0) {
    return (product + divisor / 2) / divisor;
  }
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

static uint64_t time_project_dwt_to_lane_ns(time_clock_id_t clock,
                                            uint32_t dwt_cycle_count) {
  const uint64_t basis_ns = time_lane_ns(clock);
  const uint32_t basis_dwt = time_lane_basis_dwt(clock);
  const uint32_t dwt_cycles_per_second = time_lane_dwt_cycles_per_second(clock);

  const int32_t dwt_delta = (int32_t)(dwt_cycle_count - basis_dwt);
  const int64_t ns_delta = time_cycles_to_ns_delta(dwt_delta,
                                                   dwt_cycles_per_second);
  return (uint64_t)((int64_t)basis_ns + ns_delta);
}

static uint32_t time_project_lane_ns_to_dwt(time_clock_id_t clock,
                                            uint64_t target_ns) {
  const uint64_t basis_ns = time_lane_ns(clock);
  const uint32_t basis_dwt = time_lane_basis_dwt(clock);
  const uint32_t dwt_cycles_per_second = time_lane_dwt_cycles_per_second(clock);

  const int64_t ns_delta = (target_ns >= basis_ns)
      ? (int64_t)(target_ns - basis_ns)
      : -(int64_t)(basis_ns - target_ns);

  const int64_t cycle_delta = time_ns_to_cycles_delta(ns_delta,
                                                      dwt_cycles_per_second);
  return basis_dwt + (uint32_t)cycle_delta;
}

// Bridge-period definition. process_time.cpp currently defines this symbol too;
// this weak version becomes active once the legacy owner is removed.
__attribute__((weak)) int64_t time_dwt_to_gnss_ns(uint32_t dwt_cycle_count) {
  return (int64_t)time_project_dwt_to_lane_ns(time_clock_id_t::GNSS,
                                             dwt_cycle_count);
}

uint64_t time_dwt_to_ocxo1_ns(uint32_t dwt_cycle_count) {
  return time_project_dwt_to_lane_ns(time_clock_id_t::OCXO1, dwt_cycle_count);
}

uint64_t time_dwt_to_ocxo2_ns(uint32_t dwt_cycle_count) {
  return time_project_dwt_to_lane_ns(time_clock_id_t::OCXO2, dwt_cycle_count);
}

// Bridge-period definition. process_time.cpp currently defines the signed form;
// this weak version becomes active once the legacy owner is removed.
__attribute__((weak)) uint32_t time_gnss_ns_to_dwt(int64_t gnss_ns) {
  return time_project_lane_ns_to_dwt(time_clock_id_t::GNSS,
                                    (uint64_t)gnss_ns);
}

uint32_t time_ocxo1_ns_to_dwt(uint64_t ocxo1_ns) {
  return time_project_lane_ns_to_dwt(time_clock_id_t::OCXO1, ocxo1_ns);
}

uint32_t time_ocxo2_ns_to_dwt(uint64_t ocxo2_ns) {
  return time_project_lane_ns_to_dwt(time_clock_id_t::OCXO2, ocxo2_ns);
}
