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
// No smoothing. No line fitting. No validity fog. A lane has an acquisition
// second first; prediction begins only after a lane-local 10,000,000-tick span
// has actually completed.
// ============================================================================

#include "process_clocks.h"
#include "process_clocks_internal.h"

#include <Arduino.h>
#include <stdint.h>

static constexpr uint32_t GAMMA_CADENCE_HZ = CLOCKS_GAMMA_CADENCE_HZ;
static constexpr uint32_t GAMMA_ACCEPT_ERROR_CYCLES = 1U;
static constexpr uint32_t GAMMA_DETAIL_SAMPLE_COUNT = 10U;
static constexpr uint32_t GAMMA_DETAIL_SAMPLE_STRIDE =
    GAMMA_CADENCE_HZ / GAMMA_DETAIL_SAMPLE_COUNT;

static_assert(CLOCKS_GAMMA_CADENCE_HZ == 100U,
              "Gamma prediction detail assumes 100 Hz courtroom cadence");

struct gamma_detail_sample_t {
  bool     populated = false;
  bool     endpoint = false;
  uint32_t sample_index = 0;
  uint32_t sample_percent = 0;

  uint32_t static_prediction_cycles = 0;
  uint32_t dynamic_prediction_cycles = 0;
  uint32_t dynamic_prediction_after_sample_cycles = 0;

  uint32_t static_prediction_thus_far_cycles = 0;
  uint32_t dynamic_prediction_thus_far_cycles = 0;
  int32_t  dynamic_minus_static_thus_far_cycles = 0;
  uint32_t actual_cycles_thus_far = 0;

  int32_t  residual_cycles = 0;
  uint32_t abs_residual_cycles = 0;
  bool     accepted = false;
  bool     ignored = false;
  int32_t  correction_cycles = 0;
  uint32_t gate_threshold_cycles = GAMMA_ACCEPT_ERROR_CYCLES;
};

struct gamma_detail_snapshot_t {
  bool     valid = false;
  uint32_t completed_edge_count = 0;
  uint32_t anchor_dwt = 0;
  uint32_t static_prediction_cycles = 0;
  uint32_t dynamic_final_prediction_cycles = 0;
  uint32_t actual_cycles = 0;
  int32_t  static_residual_cycles = 0;
  int32_t  dynamic_residual_cycles = 0;
  uint32_t sample_count = 0;
  gamma_detail_sample_t samples[GAMMA_DETAIL_SAMPLE_COUNT] = {};
};

struct gamma_lane_t {
  volatile uint32_t seq = 0;

  uint32_t edge_count = 0;
  uint32_t second_start_dwt = 0;

  uint32_t current_static_prediction_cycles = 0;
  uint32_t current_dynamic_prediction_cycles = 0;
  uint32_t current_sample_count = 0;
  uint32_t current_match_count = 0;
  uint32_t current_adjust_count = 0;
  uint32_t current_ignored_count = 0;
  int32_t  current_ignored_min_error_cycles = 0;
  int32_t  current_ignored_max_error_cycles = 0;
  int32_t  current_last_error_cycles = 0;
  uint32_t current_last_expected_dwt = 0;
  uint32_t current_last_sample_dwt = 0;
  uint32_t current_last_sample_index = 0;

  uint32_t completed_edge_count = 0;
  uint32_t completed_static_prediction_cycles = 0;
  uint32_t completed_dynamic_prediction_cycles = 0;
  uint32_t completed_best_dwt_cycles = 0;
  uint32_t completed_actual_dwt_cycles_between_edges = 0;
  uint32_t completed_match_count = 0;
  uint32_t completed_adjust_count = 0;
  uint32_t completed_ignored_count = 0;
  int32_t  completed_ignored_min_error_cycles = 0;
  int32_t  completed_ignored_max_error_cycles = 0;
  int32_t  completed_last_error_cycles = 0;
  uint32_t completed_last_sample_index = 0;

  gamma_detail_sample_t current_detail[GAMMA_DETAIL_SAMPLE_COUNT] = {};
  gamma_detail_snapshot_t completed_detail = {};
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

static inline uint32_t gamma_abs_error(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

static uint32_t gamma_clock_id(time_clock_id_t clock) {
  return (uint32_t)((uint8_t)clock);
}

static void gamma_reset_current_detail(gamma_lane_t& s) {
  for (uint32_t i = 0; i < GAMMA_DETAIL_SAMPLE_COUNT; i++) {
    s.current_detail[i] = gamma_detail_sample_t{};
  }
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
  s.current_last_sample_index = 0;
  gamma_reset_current_detail(s);
}

static void gamma_reset_lane(gamma_lane_t& s) {
  s.seq++;
  gamma_dmb();

  s.edge_count = 0;
  s.second_start_dwt = 0;
  s.current_static_prediction_cycles = 0;
  s.current_dynamic_prediction_cycles = 0;
  gamma_reset_current_second(s);

  s.completed_edge_count = 0;
  s.completed_static_prediction_cycles = 0;
  s.completed_dynamic_prediction_cycles = 0;
  s.completed_best_dwt_cycles = 0;
  s.completed_actual_dwt_cycles_between_edges = 0;
  s.completed_match_count = 0;
  s.completed_adjust_count = 0;
  s.completed_ignored_count = 0;
  s.completed_ignored_min_error_cycles = 0;
  s.completed_ignored_max_error_cycles = 0;
  s.completed_last_error_cycles = 0;
  s.completed_last_sample_index = 0;
  s.completed_detail = gamma_detail_snapshot_t{};

  gamma_dmb();
  s.seq++;
}

void clocks_gamma_reset_all(void) {
  gamma_reset_lane(g_gamma_vclock);
  gamma_reset_lane(g_gamma_ocxo1);
  gamma_reset_lane(g_gamma_ocxo2);
}

static inline uint32_t gamma_detail_index_for_sample(uint32_t sample_index) {
  return (sample_index / GAMMA_DETAIL_SAMPLE_STRIDE) - 1U;
}

static inline uint32_t gamma_round_for_percent(uint32_t cycles,
                                               uint32_t sample_percent) {
  return (uint32_t)(((uint64_t)cycles * (uint64_t)sample_percent + 50ULL) / 100ULL);
}

static void gamma_capture_detail_sample(gamma_lane_t& s,
                                        uint32_t sample_index,
                                        uint32_t dwt_at_sample,
                                        uint32_t dynamic_before,
                                        uint32_t dynamic_after,
                                        int32_t error_cycles,
                                        bool accepted,
                                        bool ignored,
                                        int32_t correction_cycles,
                                        bool endpoint) {
  if (sample_index == 0) return;
  if (!endpoint && (sample_index % GAMMA_DETAIL_SAMPLE_STRIDE) != 0U) return;
  if (sample_index > GAMMA_CADENCE_HZ) return;

  const uint32_t idx = endpoint
      ? (GAMMA_DETAIL_SAMPLE_COUNT - 1U)
      : gamma_detail_index_for_sample(sample_index);
  if (idx >= GAMMA_DETAIL_SAMPLE_COUNT) return;

  const uint32_t sample_percent = endpoint ? 100U : sample_index;
  gamma_detail_sample_t d{};
  d.populated = true;
  d.endpoint = endpoint;
  d.sample_index = sample_index;
  d.sample_percent = sample_percent;
  d.static_prediction_cycles = s.current_static_prediction_cycles;
  d.dynamic_prediction_cycles = dynamic_before;
  d.dynamic_prediction_after_sample_cycles = dynamic_after;
  d.static_prediction_thus_far_cycles =
      gamma_round_for_percent(s.current_static_prediction_cycles, sample_percent);
  d.dynamic_prediction_thus_far_cycles =
      gamma_round_for_percent(dynamic_before, sample_percent);
  d.dynamic_minus_static_thus_far_cycles =
      (int32_t)((int64_t)d.dynamic_prediction_thus_far_cycles -
                (int64_t)d.static_prediction_thus_far_cycles);
  d.actual_cycles_thus_far = dwt_at_sample - s.second_start_dwt;
  d.residual_cycles = error_cycles;
  d.abs_residual_cycles = gamma_abs_error(error_cycles);
  d.accepted = accepted;
  d.ignored = ignored;
  d.correction_cycles = correction_cycles;
  d.gate_threshold_cycles = endpoint
      ? GAMMA_ACCEPT_ERROR_CYCLES
      : GAMMA_ACCEPT_ERROR_CYCLES;

  s.current_detail[idx] = d;
}

static void gamma_publish_completed_detail(gamma_lane_t& s,
                                           uint32_t dwt_at_edge,
                                           uint32_t actual_cycles) {
  gamma_detail_snapshot_t d{};
  d.valid = s.edge_count >= 2;
  d.completed_edge_count = s.edge_count;
  d.anchor_dwt = s.second_start_dwt;
  d.static_prediction_cycles = s.current_static_prediction_cycles;
  d.dynamic_final_prediction_cycles = s.current_dynamic_prediction_cycles;
  d.actual_cycles = actual_cycles;
  d.static_residual_cycles =
      (int32_t)((int64_t)actual_cycles - (int64_t)s.current_static_prediction_cycles);
  d.dynamic_residual_cycles =
      (int32_t)((int64_t)actual_cycles - (int64_t)s.current_dynamic_prediction_cycles);
  d.sample_count = GAMMA_DETAIL_SAMPLE_COUNT;

  for (uint32_t i = 0; i < GAMMA_DETAIL_SAMPLE_COUNT - 1U; i++) {
    d.samples[i] = s.current_detail[i];
  }

  const int32_t endpoint_error = d.dynamic_residual_cycles;
  gamma_capture_detail_sample(s,
                              GAMMA_CADENCE_HZ,
                              dwt_at_edge,
                              s.current_dynamic_prediction_cycles,
                              s.current_dynamic_prediction_cycles,
                              endpoint_error,
                              gamma_abs_error(endpoint_error) <= GAMMA_ACCEPT_ERROR_CYCLES,
                              gamma_abs_error(endpoint_error) > GAMMA_ACCEPT_ERROR_CYCLES,
                              0,
                              true);
  d.samples[GAMMA_DETAIL_SAMPLE_COUNT - 1U] =
      s.current_detail[GAMMA_DETAIL_SAMPLE_COUNT - 1U];

  s.completed_detail = d;
}

void clocks_gamma_second_edge(time_clock_id_t clock, uint32_t dwt_at_edge) {
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return;

  s->seq++;
  gamma_dmb();

  if (s->edge_count == 0) {
    s->edge_count = 1;
    s->second_start_dwt = dwt_at_edge;
    s->current_static_prediction_cycles = 0;
    s->current_dynamic_prediction_cycles = 0;
    gamma_reset_current_second(*s);
    gamma_dmb();
    s->seq++;
    return;
  }

  const uint32_t actual_cycles = dwt_at_edge - s->second_start_dwt;

  gamma_publish_completed_detail(*s, dwt_at_edge, actual_cycles);

  const uint32_t best_cycles = (s->current_dynamic_prediction_cycles != 0)
      ? s->current_dynamic_prediction_cycles
      : actual_cycles;

  s->completed_edge_count = s->edge_count;
  s->completed_static_prediction_cycles = s->current_static_prediction_cycles;
  s->completed_dynamic_prediction_cycles = s->current_dynamic_prediction_cycles;
  s->completed_best_dwt_cycles = best_cycles;
  s->completed_actual_dwt_cycles_between_edges = actual_cycles;
  s->completed_match_count = s->current_match_count;
  s->completed_adjust_count = s->current_adjust_count;
  s->completed_ignored_count = s->current_ignored_count;
  s->completed_ignored_min_error_cycles = s->current_ignored_min_error_cycles;
  s->completed_ignored_max_error_cycles = s->current_ignored_max_error_cycles;
  s->completed_last_error_cycles = s->current_last_error_cycles;
  s->completed_last_sample_index = s->current_last_sample_index;

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

static void gamma_100hz_sample_indexed(gamma_lane_t& s,
                                       uint32_t sample_index,
                                       uint32_t dwt_at_sample) {
  // Edge 1 opens the acquisition second.  Edge 2 creates the first actual
  // lane-local 10,000,000-tick DWT span.  Courtroom samples before that first
  // completed span have no honest static prediction to test.
  if (s.edge_count < 2) return;

  if (sample_index == 0 || sample_index >= GAMMA_CADENCE_HZ) return;

  const uint32_t expected_dwt =
      s.second_start_dwt +
      gamma_expected_offset(s.current_dynamic_prediction_cycles, sample_index);
  const int32_t error_cycles = (int32_t)(dwt_at_sample - expected_dwt);
  const uint32_t abs_error = gamma_abs_error(error_cycles);

  s.current_sample_count++;
  s.current_last_sample_index = sample_index;
  s.current_last_error_cycles = error_cycles;
  s.current_last_expected_dwt = expected_dwt;
  s.current_last_sample_dwt = dwt_at_sample;

  const uint32_t dynamic_before = s.current_dynamic_prediction_cycles;
  bool accepted = false;
  bool ignored = false;
  int32_t correction_cycles = 0;

  if (error_cycles == 0) {
    s.current_match_count++;
    accepted = true;
  } else if (abs_error <= GAMMA_ACCEPT_ERROR_CYCLES) {
    const int64_t corrected =
        (int64_t)s.current_dynamic_prediction_cycles + (int64_t)error_cycles;
    s.current_dynamic_prediction_cycles = (corrected <= 0)
        ? 1U
        : ((corrected > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)corrected);
    s.current_adjust_count++;
    accepted = true;
    correction_cycles = error_cycles;
  } else {
    ignored = true;
    if (s.current_ignored_count == 0) {
      s.current_ignored_min_error_cycles = error_cycles;
      s.current_ignored_max_error_cycles = error_cycles;
    } else {
      if (error_cycles < s.current_ignored_min_error_cycles) {
        s.current_ignored_min_error_cycles = error_cycles;
      }
      if (error_cycles > s.current_ignored_max_error_cycles) {
        s.current_ignored_max_error_cycles = error_cycles;
      }
    }
    s.current_ignored_count++;
  }

  gamma_capture_detail_sample(s,
                              sample_index,
                              dwt_at_sample,
                              dynamic_before,
                              s.current_dynamic_prediction_cycles,
                              error_cycles,
                              accepted,
                              ignored,
                              correction_cycles,
                              false);
}

void clocks_gamma_100hz_sample_at_index(time_clock_id_t clock,
                                        uint32_t sample_index,
                                        uint32_t dwt_at_sample) {
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return;

  s->seq++;
  gamma_dmb();

  gamma_100hz_sample_indexed(*s, sample_index, dwt_at_sample);

  gamma_dmb();
  s->seq++;
}

void clocks_gamma_100hz_sample(time_clock_id_t clock, uint32_t dwt_at_sample) {
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return;

  s->seq++;
  gamma_dmb();

  const uint32_t sample_index = s->current_sample_count + 1U;
  gamma_100hz_sample_indexed(*s, sample_index, dwt_at_sample);

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
  out.completed_best_dwt_cycles = s.completed_best_dwt_cycles;
  out.completed_actual_dwt_cycles_between_edges = s.completed_actual_dwt_cycles_between_edges;
  out.completed_match_count = s.completed_match_count;
  out.completed_adjust_count = s.completed_adjust_count;
  out.completed_ignored_count = s.completed_ignored_count;
  out.completed_ignored_min_error_cycles = s.completed_ignored_min_error_cycles;
  out.completed_ignored_max_error_cycles = s.completed_ignored_max_error_cycles;
  out.completed_last_error_cycles = s.completed_last_error_cycles;
  out.completed_last_sample_index = s.completed_last_sample_index;
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
  out.current_last_sample_index = s.current_last_sample_index;
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


static void gamma_copy_detail_sample(const gamma_detail_sample_t& in,
                                     clocks_gamma_prediction_detail_sample_t& out) {
  out.populated = in.populated;
  out.endpoint = in.endpoint;
  out.sample_index = in.sample_index;
  out.sample_percent = in.sample_percent;
  out.static_prediction_cycles = in.static_prediction_cycles;
  out.dynamic_prediction_cycles = in.dynamic_prediction_cycles;
  out.dynamic_prediction_after_sample_cycles = in.dynamic_prediction_after_sample_cycles;
  out.static_prediction_thus_far_cycles = in.static_prediction_thus_far_cycles;
  out.dynamic_prediction_thus_far_cycles = in.dynamic_prediction_thus_far_cycles;
  out.dynamic_minus_static_thus_far_cycles = in.dynamic_minus_static_thus_far_cycles;
  out.actual_cycles_thus_far = in.actual_cycles_thus_far;
  out.residual_cycles = in.residual_cycles;
  out.abs_residual_cycles = in.abs_residual_cycles;
  out.accepted = in.accepted;
  out.ignored = in.ignored;
  out.correction_cycles = in.correction_cycles;
  out.gate_threshold_cycles = in.gate_threshold_cycles;
}

bool clocks_gamma_prediction_detail_snapshot(
    time_clock_id_t clock,
    clocks_gamma_prediction_detail_snapshot_t* out) {
  if (!out) return false;
  gamma_lane_t* s = gamma_lane(clock);
  if (!s) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    gamma_dmb();

    clocks_gamma_prediction_detail_snapshot_t local{};
    local.valid = s->completed_detail.valid;
    local.clock_id = gamma_clock_id(clock);
    local.completed_edge_count = s->completed_detail.completed_edge_count;
    local.anchor_dwt = s->completed_detail.anchor_dwt;
    local.static_prediction_cycles = s->completed_detail.static_prediction_cycles;
    local.dynamic_final_prediction_cycles = s->completed_detail.dynamic_final_prediction_cycles;
    local.actual_cycles = s->completed_detail.actual_cycles;
    local.static_residual_cycles = s->completed_detail.static_residual_cycles;
    local.dynamic_residual_cycles = s->completed_detail.dynamic_residual_cycles;
    local.sample_count = s->completed_detail.sample_count;
    local.sample_capacity = GAMMA_DETAIL_SAMPLE_COUNT;
    local.sample_step_percent = 10U;
    local.gate_threshold_cycles = GAMMA_ACCEPT_ERROR_CYCLES;
    for (uint32_t i = 0; i < GAMMA_DETAIL_SAMPLE_COUNT; i++) {
      gamma_copy_detail_sample(s->completed_detail.samples[i], local.samples[i]);
    }

    gamma_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return true;
    }
  }

  *out = clocks_gamma_prediction_detail_snapshot_t{};
  return false;
}
