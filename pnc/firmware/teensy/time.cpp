#include "time.h"
#include "process_clocks.h"

#include <stdint.h>

// ============================================================================
// time.cpp -- stateless CLOCKS/static-prediction facade
// ============================================================================
//
// TIME owns no state here.
// TIME does not read DWT.
// TIME does not install anchors.
//
// The caller supplies a DWT coordinate.  The transitional process_time clock
// snapshot supplies the most recent per-lane basis; CLOCKS static prediction is
// used only as a fallback denominator.  Gamma/dynamic prediction is retired.
// ============================================================================

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;

static uint64_t time_lane_fallback_ns(time_clock_id_t clock) {
  if (clock == time_clock_id_t::OCXO1) {
    return clocks_ocxo1_measured_gnss_ns_now();
  }
  if (clock == time_clock_id_t::OCXO2) {
    return clocks_ocxo2_measured_gnss_ns_now();
  }
  return clocks_gnss_ns_now();
}

static bool time_lane_snapshot(time_clock_id_t clock,
                               time_clock_snapshot_t& out) {
  if (time_clock_snapshot(clock, &out) && out.valid) {
    return true;
  }

  out = time_clock_snapshot_t{};
  out.valid = false;
  out.ns_at_update = time_lane_fallback_ns(clock);
  out.predicted_dwt_cycles_per_second = DWT_EXPECTED_PER_PPS;
  return false;
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
  time_clock_snapshot_t basis{};
  if (!time_lane_snapshot(clock, basis)) {
    return basis.ns_at_update;
  }

  const uint32_t dwt_cycles_per_second =
      time_lane_dwt_cycles_per_second(clock, basis);
  if (dwt_cycles_per_second == 0) return basis.ns_at_update;

  const int32_t dwt_delta = (int32_t)(dwt_cycle_count - basis.dwt_at_update);
  const int64_t ns_delta = time_cycles_to_ns_delta(dwt_delta,
                                                   dwt_cycles_per_second);
  return (uint64_t)((int64_t)basis.ns_at_update + ns_delta);
}

static uint32_t time_project_lane_ns_to_dwt(time_clock_id_t clock,
                                            uint64_t target_ns) {
  time_clock_snapshot_t basis{};
  if (!time_lane_snapshot(clock, basis)) {
    return 0;
  }

  const uint32_t dwt_cycles_per_second =
      time_lane_dwt_cycles_per_second(clock, basis);
  if (dwt_cycles_per_second == 0) return 0;

  const int64_t ns_delta = (target_ns >= basis.ns_at_update)
      ? (int64_t)(target_ns - basis.ns_at_update)
      : -(int64_t)(basis.ns_at_update - target_ns);

  const int64_t cycle_delta = time_ns_to_cycles_delta(ns_delta,
                                                      dwt_cycles_per_second);
  return basis.dwt_at_update + (uint32_t)cycle_delta;
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
