// ============================================================================
// process_clocks_gamma.cpp — CLOCKS Gamma next-second prediction layer
// ============================================================================
//
// Gamma owns generalized next-second DWT prediction for the three measured
// 10 MHz clock lanes: VCLOCK, OCXO1, and OCXO2.
//
// The unit being predicted is deliberately lane-local:
//   • VCLOCK: DWT cycles between consecutive 10,000,000-tick VCLOCK edges
//   • OCXO1:  DWT cycles between consecutive 10,000,000-tick OCXO1 edges
//   • OCXO2:  DWT cycles between consecutive 10,000,000-tick OCXO2 edges
//
// Static prediction is the last completed clock-second's actual DWT cycle
// count. Dynamic prediction starts from static, then a 100 Hz courtroom tests
// in-flight ISR event-coordinate DWT facts against the current prediction.
//
// Courtroom rule:
//   error == 0      -> prediction stands
//   error == +1/-1  -> dynamic prediction is adjusted by that one cycle
//   |error| > 1     -> witness is ignored as ISR/preemption contamination
//
// No smoothing. No line fitting. No validity fog. The lane either has prior
// edges and emits facts, or its surfaces remain zero until the first completed
// second exists.
// ============================================================================

#include "process_clocks.h"
#include "process_clocks_internal.h"

#include <Arduino.h>
#include <stdint.h>

static constexpr uint32_t GAMMA_CADENCE_HZ = 100U;
static constexpr uint32_t GAMMA_ACCEPT_ERROR_CYCLES = 1U;

struct gamma_lane_t {
  volatile uint32_t seq = 0;

  uint32_t edge_count = 0;
  uint32_t second_start_dwt = 0;

  uint32_t current_static_prediction_cycles = DWT_EXPECTED_PER_PPS;
  uint32_t current_dynamic_prediction_cycles = DWT_EXPECTED_PER_PPS;
  uint32_t current_sample_count = 0;
  uint32_t current_match_count = 0;
  uint32_t current_adjust_count = 0;
  uint32_t current_ignored_count = 0;
  int32_t  current_ignored_min_error_cycles = 0;
  int32_t  current_ignored_max_error_cycles = 0;
  int32_t  current_last_error_cycles = 0;
  uint32_t current_last_expected_dwt = 0;
  uint32_t current_last_sample_dwt = 0;

  uint32_t completed_edge_count = 0;
  uint32_t completed_static_prediction_cycles = 0;
  uint32_t completed_dynamic_prediction_cycles = 0;
  uint32_t completed_actual_dwt_cycles_between_edges = 0;
  uint32_t completed_match_count = 0;
  uint32_t completed_adjust_count = 0;
  uint32_t completed_ignored_count = 0;
  int32_t  completed_ignored_min_error_cycles = 0;
  int32_t  completed_ignored_max_error_cycles = 0;
  int32_t  completed_last_error_cycles = 0;
};

static gamma_lane_t g_gamma_vclock;
static gamma_lane_t g_gamma_ocxo1;
static gamma_lane_t g_gamma_ocxo2;

static inline void gamma_dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static gamma_lane_t* gamma_lane(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return &g_gamma_vclock;
    case time_clock_id_t::OCXO1:  return &g_gamma_ocxo1;
    case time_clock_id_t::OCXO2:  return &g_gamma_ocxo2;
    default:                     return nullptr;
  }
}

static uint32_t gamma_clock_id(time_clock_id_t clock) {
  return (uint32_t)((uint8_t)clock);
}

static void gamma_reset_current_second(gamma_lane_t& s) {
  s.current_sample_count = 0;
  s.current_match_count = 0;
  s.current_adjust_count = 0;
  s.current_ignored_count = 0;
  s.current_ignored_min_error_cycles = 0;
  s.current_ignored_max_error_cycles = 0;
  s.current_last_error_cycles = 0;
  s.current_last_expected_dwt = 0;
  s.current_last_sample_dwt = 0;
}

static void gamma_reset_lane(gamma_lane_t& s) {
  s.seq++;
  gamma_dmb();

  s.edge_count = 0;
  s.second_start_dwt = 0;
  s.current_static_prediction_cycles = DWT_EXPECTED_PER_PPS;
  s.current_dynamic_prediction_cycles = DWT_EXPECTED_PER_PPS;
  gamma_reset_current_second(s);

  s.completed_edge_count = 0;
  s.completed_static_prediction_cycles = 0;
  s.completed_dynamic_prediction_cycles = 0;
  s.completed_actual_dwt_cycles_between_edges = 0;
  s.completed_match_count = 0;
  s.completed_adjust_count = 0;
  s.completed_ignored_count = 0;
  s.completed_ignored_min_error_cycles = 0;
  s.completed_ignored_max_error_cycles = 0;
  s.completed_last_error_cycles = 0;

  gamma_dmb();
  s.seq++;
}

void clocks_gamma_reset_all(void) {
  gamma_reset_lane(g_gamma_vclock);
  gamma_reset_lane(g_gamma_ocxo1);
  gamma_reset_lane(g_gamma_ocxo2);
}

void clocks_gamma_second_edge(time_clock_id_t clock, uint32_t dwt_at_edge) {
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return;

  s->seq++;
  gamma_dmb();

  if (s->edge_count == 0) {
    s->edge_count = 1;
    s->second_start_dwt = dwt_at_edge;
    s->current_static_prediction_cycles = DWT_EXPECTED_PER_PPS;
    s->current_dynamic_prediction_cycles = DWT_EXPECTED_PER_PPS;
    gamma_reset_current_second(*s);
    gamma_dmb();
    s->seq++;
    return;
  }

  const uint32_t actual_cycles = dwt_at_edge - s->second_start_dwt;

  s->completed_edge_count = s->edge_count;
  s->completed_static_prediction_cycles = s->current_static_prediction_cycles;
  s->completed_dynamic_prediction_cycles = s->current_dynamic_prediction_cycles;
  s->completed_actual_dwt_cycles_between_edges = actual_cycles;
  s->completed_match_count = s->current_match_count;
  s->completed_adjust_count = s->current_adjust_count;
  s->completed_ignored_count = s->current_ignored_count;
  s->completed_ignored_min_error_cycles = s->current_ignored_min_error_cycles;
  s->completed_ignored_max_error_cycles = s->current_ignored_max_error_cycles;
  s->completed_last_error_cycles = s->current_last_error_cycles;

  s->edge_count++;
  s->second_start_dwt = dwt_at_edge;
  s->current_static_prediction_cycles = actual_cycles;
  s->current_dynamic_prediction_cycles = actual_cycles;
  gamma_reset_current_second(*s);

  gamma_dmb();
  s->seq++;
}

static inline uint32_t gamma_expected_offset(uint32_t prediction_cycles,
                                             uint32_t sample_index) {
  return (uint32_t)(((uint64_t)prediction_cycles * (uint64_t)sample_index +
                     (uint64_t)GAMMA_CADENCE_HZ / 2ULL) /
                    (uint64_t)GAMMA_CADENCE_HZ);
}

static inline uint32_t gamma_abs_error(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

void clocks_gamma_100hz_sample(time_clock_id_t clock, uint32_t dwt_at_sample) {
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return;

  s->seq++;
  gamma_dmb();

  if (s->edge_count == 0) {
    gamma_dmb();
    s->seq++;
    return;
  }

  const uint32_t sample_index = s->current_sample_count + 1U;
  if (sample_index >= GAMMA_CADENCE_HZ) {
    gamma_dmb();
    s->seq++;
    return;
  }

  const uint32_t expected_dwt =
      s->second_start_dwt +
      gamma_expected_offset(s->current_dynamic_prediction_cycles, sample_index);
  const int32_t error_cycles = (int32_t)(dwt_at_sample - expected_dwt);
  const uint32_t abs_error = gamma_abs_error(error_cycles);

  s->current_sample_count = sample_index;
  s->current_last_error_cycles = error_cycles;
  s->current_last_expected_dwt = expected_dwt;
  s->current_last_sample_dwt = dwt_at_sample;

  if (error_cycles == 0) {
    s->current_match_count++;
  } else if (abs_error <= GAMMA_ACCEPT_ERROR_CYCLES) {
    const int64_t corrected =
        (int64_t)s->current_dynamic_prediction_cycles + (int64_t)error_cycles;
    s->current_dynamic_prediction_cycles = (corrected <= 0)
        ? 1U
        : ((corrected > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)corrected);
    s->current_adjust_count++;
  } else {
    if (s->current_ignored_count == 0) {
      s->current_ignored_min_error_cycles = error_cycles;
      s->current_ignored_max_error_cycles = error_cycles;
    } else {
      if (error_cycles < s->current_ignored_min_error_cycles) {
        s->current_ignored_min_error_cycles = error_cycles;
      }
      if (error_cycles > s->current_ignored_max_error_cycles) {
        s->current_ignored_max_error_cycles = error_cycles;
      }
    }
    s->current_ignored_count++;
  }

  gamma_dmb();
  s->seq++;
}

static void gamma_copy_snapshot(time_clock_id_t clock,
                                const gamma_lane_t& s,
                                clocks_gamma_prediction_snapshot_t& out) {
  out.clock_id = gamma_clock_id(clock);
  out.completed_edge_count = s.completed_edge_count;
  out.completed_static_prediction_cycles = s.completed_static_prediction_cycles;
  out.completed_dynamic_prediction_cycles = s.completed_dynamic_prediction_cycles;
  out.completed_actual_dwt_cycles_between_edges = s.completed_actual_dwt_cycles_between_edges;
  out.completed_match_count = s.completed_match_count;
  out.completed_adjust_count = s.completed_adjust_count;
  out.completed_ignored_count = s.completed_ignored_count;
  out.completed_ignored_min_error_cycles = s.completed_ignored_min_error_cycles;
  out.completed_ignored_max_error_cycles = s.completed_ignored_max_error_cycles;
  out.completed_last_error_cycles = s.completed_last_error_cycles;
  out.edge_count = s.edge_count;
  out.second_start_dwt = s.second_start_dwt;
  out.current_static_prediction_cycles = s.current_static_prediction_cycles;
  out.current_dynamic_prediction_cycles = s.current_dynamic_prediction_cycles;
  out.current_sample_count = s.current_sample_count;
  out.current_match_count = s.current_match_count;
  out.current_adjust_count = s.current_adjust_count;
  out.current_ignored_count = s.current_ignored_count;
  out.current_ignored_min_error_cycles = s.current_ignored_min_error_cycles;
  out.current_ignored_max_error_cycles = s.current_ignored_max_error_cycles;
  out.current_last_error_cycles = s.current_last_error_cycles;
  out.current_last_expected_dwt = s.current_last_expected_dwt;
  out.current_last_sample_dwt = s.current_last_sample_dwt;
}

bool clocks_gamma_snapshot(time_clock_id_t clock,
                           clocks_gamma_prediction_snapshot_t* out) {
  if (!out) return false;
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    gamma_dmb();

    clocks_gamma_prediction_snapshot_t local{};
    gamma_copy_snapshot(clock, *s, local);

    gamma_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return true;
    }
  }

  *out = clocks_gamma_prediction_snapshot_t{};
  return false;
}
