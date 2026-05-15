#include "time.h"
#include "process_clocks.h"

#include <stdint.h>

// ============================================================================
// time.cpp -- caller-facing TIME conversion facade
// ============================================================================
//
// process_time remains alive and owns the legacy anchor/projection state that
// TimePop depends on.  This file owns only the new, self-documenting conversion
// facade names and non-conflicting compatibility aliases.
//
// Rules:
//   • Callers provide authored DWT coordinates; this file never captures DWT.
//   • Conversion math is DWT <-> clock-domain nanoseconds.
//   • process_time supplies the current per-clock projection basis through
//     time_clock_snapshot() / time_clock_ns_at_dwt().
//   • CLOCKS static prediction is a fallback denominator only.
//   • Legacy symbols already owned by process_time are NOT defined here.
// ============================================================================

static constexpr uint64_t TIME_NS_PER_SECOND_U64 = 1000000000ULL;

static bool time_is_gnss_like(time_clock_id_t clock) {
  return clock == time_clock_id_t::VCLOCK || clock == time_clock_id_t::GNSS;
}

static bool time_is_known_clock(time_clock_id_t clock) {
  return time_is_gnss_like(clock) ||
         clock == time_clock_id_t::OCXO1 ||
         clock == time_clock_id_t::OCXO2;
}

static bool time_lane_snapshot(time_clock_id_t clock,
                               time_clock_snapshot_t& out) {
  out = time_clock_snapshot_t{};
  if (!time_is_known_clock(clock)) return false;
  return time_clock_snapshot(clock, &out) && out.valid;
}

static uint32_t time_lane_dwt_cycles_per_second(time_clock_id_t clock,
                                                const time_clock_snapshot_t& basis) {
  if (basis.prediction_valid && basis.predicted_dwt_cycles_per_second != 0) {
    return basis.predicted_dwt_cycles_per_second;
  }

  clocks_static_prediction_snapshot_t pred{};
  if (time_is_gnss_like(clock)) {
    if (clocks_static_prediction_pps_snapshot(&pred)) {
      if (pred.actual_cycles != 0) return pred.actual_cycles;
      if (pred.static_prediction_cycles != 0) return pred.static_prediction_cycles;
    }
  } else if (clocks_static_prediction_snapshot(clock, &pred)) {
    if (pred.actual_cycles != 0) return pred.actual_cycles;
    if (pred.static_prediction_cycles != 0) return pred.static_prediction_cycles;
  }

  const uint32_t dwt_cps = clocks_dwt_cycles_per_gnss_second();
  return dwt_cps ? dwt_cps : (uint32_t)DWT_EXPECTED_PER_PPS;
}

static int64_t time_mul_div_round_i64(int64_t value,
                                      int64_t multiplier,
                                      int64_t divisor) {
  if (divisor == 0) return 0;

  const int64_t product = value * multiplier;
  if (product >= 0) {
    return (product + divisor / 2) / divisor;
  }
  return -(((-product) + divisor / 2) / divisor);
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

bool time_clock_projection(time_clock_id_t clock,
                           time_clock_projection_t* out) {
  if (!out) return false;
  *out = time_clock_projection_t{};

  time_clock_snapshot_t basis{};
  if (!time_lane_snapshot(clock, basis)) return false;

  const uint32_t dwt_cycles_per_second =
      time_lane_dwt_cycles_per_second(clock, basis);
  if (dwt_cycles_per_second == 0) return false;

  out->clock = time_is_gnss_like(clock) ? time_clock_id_t::VCLOCK : clock;
  out->valid = true;
  out->basis_is_live = true;
  out->dwt_at_update = basis.dwt_at_update;
  out->ns_at_update = basis.ns_at_update;
  out->dwt_cycles_per_second = dwt_cycles_per_second;
  out->update_count = basis.update_count;
  out->last_observed_dwt_cycles = basis.last_observed_dwt_cycles;
  out->last_observed_ns = basis.last_observed_ns;
  out->last_prediction_residual_cycles = basis.last_prediction_residual_cycles;
  return true;
}

bool time_dwt_at_clock_ns(time_clock_id_t clock,
                          uint64_t clock_ns,
                          uint32_t* out_dwt_cycle_count) {
  if (!out_dwt_cycle_count) return false;

  time_clock_projection_t p{};
  if (!time_clock_projection(clock, &p)) return false;

  const int64_t ns_delta = (clock_ns >= p.ns_at_update)
      ? (int64_t)(clock_ns - p.ns_at_update)
      : -(int64_t)(p.ns_at_update - clock_ns);

  const int64_t cycle_delta =
      time_ns_to_cycles_delta(ns_delta, p.dwt_cycles_per_second);
  *out_dwt_cycle_count = p.dwt_at_update + (uint32_t)cycle_delta;
  return true;
}

uint64_t time_dwt_to_clock_ns(time_clock_id_t clock,
                              uint32_t authored_dwt_cycle_count) {
  uint64_t out = 0;
  (void)time_clock_ns_at_dwt(clock, authored_dwt_cycle_count, &out);
  return out;
}

uint32_t time_clock_ns_to_dwt(time_clock_id_t clock,
                              uint64_t clock_ns) {
  uint32_t out = 0;
  (void)time_dwt_at_clock_ns(clock, clock_ns, &out);
  return out;
}

// ============================================================================
// Preferred convenience wrappers: <clock> ns at DWT, DWT at <clock> ns
// ============================================================================

uint64_t time_vclock_ns_at_dwt(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::VCLOCK,
                              authored_dwt_cycle_count);
}

uint32_t time_dwt_at_vclock_ns(uint64_t vclock_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::VCLOCK, vclock_ns);
}

uint64_t time_gnss_ns_at_dwt(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::GNSS,
                              authored_dwt_cycle_count);
}

uint32_t time_dwt_at_gnss_ns(uint64_t gnss_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::GNSS, gnss_ns);
}

uint64_t time_ocxo1_ns_at_dwt(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::OCXO1,
                              authored_dwt_cycle_count);
}

uint32_t time_dwt_at_ocxo1_ns(uint64_t ocxo1_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::OCXO1, ocxo1_ns);
}

uint64_t time_ocxo2_ns_at_dwt(uint32_t authored_dwt_cycle_count) {
  return time_dwt_to_clock_ns(time_clock_id_t::OCXO2,
                              authored_dwt_cycle_count);
}

uint32_t time_dwt_at_ocxo2_ns(uint64_t ocxo2_ns) {
  return time_clock_ns_to_dwt(time_clock_id_t::OCXO2, ocxo2_ns);
}

// ============================================================================
// Non-conflicting legacy aliases retained for migration
// ============================================================================

uint64_t time_dwt_to_vclock_ns(uint32_t authored_dwt_cycle_count) {
  return time_vclock_ns_at_dwt(authored_dwt_cycle_count);
}

uint32_t time_vclock_ns_to_dwt(uint64_t vclock_ns) {
  return time_dwt_at_vclock_ns(vclock_ns);
}

uint64_t time_dwt_to_ocxo1_ns(uint32_t dwt_cycle_count) {
  return time_ocxo1_ns_at_dwt(dwt_cycle_count);
}

uint64_t time_dwt_to_ocxo2_ns(uint32_t dwt_cycle_count) {
  return time_ocxo2_ns_at_dwt(dwt_cycle_count);
}

uint32_t time_ocxo1_ns_to_dwt(uint64_t ocxo1_ns) {
  return time_dwt_at_ocxo1_ns(ocxo1_ns);
}

uint32_t time_ocxo2_ns_to_dwt(uint64_t ocxo2_ns) {
  return time_dwt_at_ocxo2_ns(ocxo2_ns);
}
