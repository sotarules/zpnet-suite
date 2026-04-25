// ============================================================================
// process_clocks_beta.cpp — Campaign Layer
// ============================================================================
//
// Statistical surface doctrine:
//
//   Teensy owns every statistical quantity published in TIMEBASE_FRAGMENT.
//   The Pi transcribes what the Teensy says — it does not recompute.
//
//   Every Welford accumulator is published with the identical suffix set:
//
//     <prefix>_welford_n
//     <prefix>_welford_mean
//     <prefix>_welford_stddev
//     <prefix>_welford_stderr
//     <prefix>_welford_min
//     <prefix>_welford_max
//
//   Seven published Welford prefixes:
//
//     dwt_welford         — Teensy CPU XTAL offset (ppb, positive = fast)
//     vclock_welford      — bridge interpolation residual (ns)
//     ocxo1_welford       — OCXO1 per-second residual (ns, positive = fast)
//     ocxo2_welford       — OCXO2 per-second residual (ns, positive = fast)
//     pps_witness_welford — GPIO PPS witness offset (ns)
//     ocxo1_dac_welford   — OCXO1 DAC fractional code (LSB)
//     ocxo2_dac_welford   — OCXO2 DAC fractional code (LSB)
//
//   Four published tau/ppb pairs (one per frequency-bearing clock):
//
//     dwt_tau,    dwt_ppb
//     vclock_tau, vclock_ppb
//     ocxo1_tau,  ocxo1_ppb
//     ocxo2_tau,  ocxo2_ppb
//
//   Sign convention is uniform:  positive ppb → clock RUNNING FAST.
//
// Unified Welford:
//
//   welford_t replaces the old pps_residual_t and dac_welford_t.  One
//   struct, one API, double-valued samples (supports ppb + ns + LSB
//   with the same type).  Global instances named welford_<what>:
//   welford_dwt, welford_vclock, welford_ocxo1, welford_ocxo2,
//   welford_pps_witness, welford_ocxo1_dac, welford_ocxo2_dac.
//
// Everything else (campaign lifecycle, servo, DAC pacing, watchdog,
// command surface) is unchanged in behavior.
//
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_interrupt.h"

#include "debug.h"
#include "timebase.h"
#include "time.h"

#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"
#include "util.h"
#include "ad5693r.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <stdlib.h>
#include <climits>

// ============================================================================
// Campaign state — definitions
// ============================================================================

volatile clocks_campaign_state_t campaign_state =
    clocks_campaign_state_t::STOPPED;

char     campaign_name[64] = {0};
uint64_t campaign_seconds  = 0;

volatile bool request_start   = false;
volatile bool request_stop    = false;
volatile bool request_recover = false;
volatile bool request_zero    = false;

uint64_t recover_dwt_ns   = 0;
uint64_t recover_gnss_ns  = 0;
uint64_t recover_ocxo1_ns = 0;
uint64_t recover_ocxo2_ns = 0;

// Most recently published TIMEBASE_FRAGMENT, retained for cmd_report.
static Payload g_last_fragment;

// ============================================================================
// Campaign warmup suppression
// ============================================================================
//
// Alpha remains fully alive during warmup: PPS/VCLOCK/OCXO measurements,
// bridge anchors, and predictor state continue to settle normally.  Beta,
// however, suppresses the first CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS
// candidate TIMEBASE_FRAGMENT records so the public campaign begins only
// after the estimators have stopped stretching their legs.
//
// START semantics:
//   Suppressed records are private warmup only.  The first emitted fragment
//   is teensy_pps_count=1, gnss_ns=1e9, and public totals start from that
//   first canonical record.
//
// RECOVER semantics:
//   Suppressed records are real elapsed campaign seconds.  The first emitted
//   fragment therefore appears after a deliberate canonical gap, preserving
//   the recovered absolute PPS identity.

enum class campaign_warmup_mode_t : uint8_t {
  NONE    = 0,
  START   = 1,
  RECOVER = 2,
};

static volatile campaign_warmup_mode_t g_campaign_warmup_mode =
    campaign_warmup_mode_t::NONE;
static volatile uint32_t g_campaign_warmup_remaining = 0;
static volatile uint32_t g_campaign_warmup_suppressed_total = 0;

static uint64_t g_campaign_public_dwt_base = 0;
static uint64_t g_campaign_public_gnss_base = 0;
static uint64_t g_campaign_public_ocxo1_base = 0;
static uint64_t g_campaign_public_ocxo2_base = 0;

static void campaign_public_bases_reset_to_current(void) {
  g_campaign_public_dwt_base   = g_dwt_cycle_count_total;
  g_campaign_public_gnss_base  = g_gnss_ns_count_at_pps;
  g_campaign_public_ocxo1_base = g_ocxo1_clock.ns_count_at_pps;
  g_campaign_public_ocxo2_base = g_ocxo2_clock.ns_count_at_pps;
}

static uint64_t saturated_base_for_recovered_value(uint64_t current,
                                                   uint64_t recovered) {
  return (current >= recovered) ? (current - recovered) : 0;
}

static void campaign_public_bases_reset_for_recover(void) {
  g_campaign_public_dwt_base =
      saturated_base_for_recovered_value(g_dwt_cycle_count_total,
                                          dwt_ns_to_cycles(recover_dwt_ns));
  g_campaign_public_gnss_base =
      saturated_base_for_recovered_value(g_gnss_ns_count_at_pps,
                                          recover_gnss_ns);
  g_campaign_public_ocxo1_base =
      saturated_base_for_recovered_value(g_ocxo1_clock.ns_count_at_pps,
                                          recover_ocxo1_ns);
  g_campaign_public_ocxo2_base =
      saturated_base_for_recovered_value(g_ocxo2_clock.ns_count_at_pps,
                                          recover_ocxo2_ns);
}

static void campaign_warmup_begin(campaign_warmup_mode_t mode) {
  g_campaign_warmup_mode = mode;
  g_campaign_warmup_remaining = CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS;
  g_campaign_warmup_suppressed_total = 0;

  if (mode == campaign_warmup_mode_t::RECOVER) {
    campaign_public_bases_reset_for_recover();
  } else {
    campaign_public_bases_reset_to_current();
  }
}

static bool campaign_warmup_active(void) {
  return g_campaign_warmup_mode != campaign_warmup_mode_t::NONE &&
         g_campaign_warmup_remaining > 0;
}

static void campaign_warmup_finish_after_this_suppressed_record(void) {
  const campaign_warmup_mode_t finishing_mode = g_campaign_warmup_mode;

  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    campaign_public_bases_reset_to_current();
    return;
  }

  if (finishing_mode == campaign_warmup_mode_t::START) {
    // START: warmup records are private.  Public identity begins on the
    // next PPS after this one, so use the current alpha totals as the base.
    // The next published record will add exactly one public second.
    campaign_public_bases_reset_to_current();
  }

  // RECOVER: leave the recovery bases intact.  Because campaign_seconds
  // advanced during suppression, the next emitted fragment will preserve
  // the canonical gap created by the buried records.
}

static bool campaign_warmup_consume_one_candidate_record(void) {
  if (!campaign_warmup_active()) return false;

  g_campaign_warmup_suppressed_total++;

  if (g_campaign_warmup_mode == campaign_warmup_mode_t::RECOVER) {
    // Recovery gaps are canonical.  The suppressed records are real
    // elapsed campaign seconds, so advance the public PPS identity even
    // though we do not publish the fragments.
    campaign_seconds++;
  }

  if (g_campaign_warmup_remaining > 0) {
    g_campaign_warmup_remaining--;
  }

  if (g_campaign_warmup_remaining == 0) {
    campaign_warmup_finish_after_this_suppressed_record();
  }

  return true;
}

static void campaign_warmup_reset(void) {
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  g_campaign_warmup_suppressed_total = 0;
  campaign_public_bases_reset_to_current();
}

static uint64_t campaign_public_dwt_total(void) {
  return (g_dwt_cycle_count_total >= g_campaign_public_dwt_base)
           ? (g_dwt_cycle_count_total - g_campaign_public_dwt_base)
           : 0;
}

static uint64_t campaign_public_gnss_ns(void) {
  return (g_gnss_ns_count_at_pps >= g_campaign_public_gnss_base)
           ? (g_gnss_ns_count_at_pps - g_campaign_public_gnss_base)
           : 0;
}

static uint64_t campaign_public_ocxo1_ns(void) {
  const uint64_t now = g_ocxo1_clock.ns_count_at_pps;
  return (now >= g_campaign_public_ocxo1_base)
           ? (now - g_campaign_public_ocxo1_base)
           : 0;
}

static uint64_t campaign_public_ocxo2_ns(void) {
  const uint64_t now = g_ocxo2_clock.ns_count_at_pps;
  return (now >= g_campaign_public_ocxo2_base)
           ? (now - g_campaign_public_ocxo2_base)
           : 0;
}

// ============================================================================
// Watchdog state
// ============================================================================

volatile bool     watchdog_anomaly_active          = false;
volatile bool     watchdog_anomaly_publish_pending = false;
volatile uint32_t watchdog_anomaly_sequence        = 0;
char              watchdog_anomaly_reason[64]      = {0};
volatile uint32_t watchdog_anomaly_detail0         = 0;
volatile uint32_t watchdog_anomaly_detail1         = 0;
volatile uint32_t watchdog_anomaly_detail2         = 0;
volatile uint32_t watchdog_anomaly_detail3         = 0;
volatile uint32_t watchdog_anomaly_trigger_dwt     = 0;

// ============================================================================
// Welford — unified accumulator
// ============================================================================

welford_t welford_dwt          = {};
welford_t welford_vclock       = {};
welford_t welford_ocxo1        = {};
welford_t welford_ocxo2        = {};
welford_t welford_pps_witness  = {};
welford_t welford_ocxo1_dac    = {};
welford_t welford_ocxo2_dac    = {};

void welford_reset(welford_t& w) {
  w.n       = 0;
  w.mean    = 0.0;
  w.m2      = 0.0;
  w.min_val = 1e30;
  w.max_val = -1e30;
}

void welford_update(welford_t& w, double sample) {
  w.n++;
  const double d1 = sample - w.mean;
  w.mean += d1 / (double)w.n;
  const double d2 = sample - w.mean;
  w.m2 += d1 * d2;
  if (sample < w.min_val) w.min_val = sample;
  if (sample > w.max_val) w.max_val = sample;
}

double welford_stddev(const welford_t& w) {
  return (w.n >= 2) ? sqrt(w.m2 / (double)(w.n - 1)) : 0.0;
}

double welford_stderr(const welford_t& w) {
  if (w.n < 2) return 0.0;
  const double sd = sqrt(w.m2 / (double)(w.n - 1));
  return sd / sqrt((double)w.n);
}

// ============================================================================
// Payload helpers — standardized publication
// ============================================================================

// Emits the complete six-field Welford block under <prefix>_welford_*.
// All six fields are always emitted, regardless of n.  Downstream
// consumers can rely on the complete set being present in every
// fragment after campaign start.
static void publish_welford(Payload& p, const char* prefix, const welford_t& w) {
  char key[80];

  snprintf(key, sizeof(key), "%s_welford_n", prefix);
  p.add(key, w.n);

  snprintf(key, sizeof(key), "%s_welford_mean", prefix);
  p.add(key, w.mean, 6);

  snprintf(key, sizeof(key), "%s_welford_stddev", prefix);
  p.add(key, welford_stddev(w), 6);

  snprintf(key, sizeof(key), "%s_welford_stderr", prefix);
  p.add(key, welford_stderr(w), 6);

  snprintf(key, sizeof(key), "%s_welford_min", prefix);
  p.add(key, (w.n > 0) ? w.min_val : 0.0, 6);

  snprintf(key, sizeof(key), "%s_welford_max", prefix);
  p.add(key, (w.n > 0) ? w.max_val : 0.0, 6);
}

// Emits <clock>_tau and <clock>_ppb.  ppb_value is in parts-per-billion
// under the uniform sign convention (positive = clock running fast).
// tau = 1.0 + ppb / 1e9.
static void publish_freq(Payload& p, const char* clock_name, double ppb_value) {
  char key[80];
  const double tau_value = 1.0 + ppb_value / 1e9;

  snprintf(key, sizeof(key), "%s_tau", clock_name);
  p.add(key, tau_value, 12);

  snprintf(key, sizeof(key), "%s_ppb", clock_name);
  p.add(key, ppb_value, 3);
}

// ============================================================================
// Event publishers — slim, post-scorched-earth
// ============================================================================
//
// Each subscription event carries {gnss_ns_at_edge, dwt_at_edge, sequence}.
// Beta publishes those three fields verbatim per lane.  No diag struct,
// no counter32, no coincidence, no anomaly count.

static void clocks_payload_add_event(Payload& p,
                                     const char* prefix,
                                     const interrupt_event_t& event) {
  char key[96];
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_i64 = [&](const char* suffix, int64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_u32("dwt_at_edge",     event.dwt_at_edge);
  add_i64("gnss_ns_at_edge", event.gnss_ns_at_edge);
  add_u32("sequence",        event.sequence);
}

// ============================================================================
// Predictive servo tuning
// ============================================================================

static constexpr uint32_t NOW_WINDOW_SIZE = 5;

static constexpr double SERVO_NS_PER_DAC_LSB = 100.0;
static constexpr double SERVO_MIN_RESIDUAL_NS = 5.0;
static constexpr double SERVO_MIN_PREDICTED_NS = 2.0;

static constexpr double SERVO_TOTAL_RESIDUAL_ALPHA = 0.08;
static constexpr double SERVO_TOTAL_SLOPE_ALPHA    = 0.12;
static constexpr double SERVO_TOTAL_HORIZON_S      = 8.0;

static constexpr double SERVO_NOW_RESIDUAL_ALPHA = 0.25;
static constexpr double SERVO_NOW_SLOPE_ALPHA    = 0.35;
static constexpr double SERVO_NOW_HORIZON_S      = 3.0;

struct now_window_t {
  int64_t  samples[NOW_WINDOW_SIZE];
  uint32_t count;
  uint32_t head;
};

static now_window_t g_now_window_ocxo1 = {};
static now_window_t g_now_window_ocxo2 = {};

static void now_window_reset(now_window_t& w) {
  for (uint32_t i = 0; i < NOW_WINDOW_SIZE; i++) w.samples[i] = 0;
  w.count = 0;
  w.head = 0;
}

static double now_window_mean(const now_window_t& w) {
  if (w.count == 0) return 0.0;
  double sum = 0.0;
  for (uint32_t i = 0; i < w.count; i++) sum += (double)w.samples[i];
  return sum / (double)w.count;
}

static void now_window_push(now_window_t& w, int64_t sample) {
  w.samples[w.head] = sample;
  w.head = (w.head + 1) % NOW_WINDOW_SIZE;
  if (w.count < NOW_WINDOW_SIZE) w.count++;
}

// ============================================================================
// DAC pacing / deferred commit
// ============================================================================

static const char* OCXO_DAC_COMMIT_NAME = "ocxo-dac-commit";

static volatile bool   g_ocxo_dac_commit_scheduled = false;
static ocxo_dac_state_t* g_ocxo_dac_commit_selected = nullptr;
static double          g_ocxo_dac_commit_target = 0.0;
static uint16_t        g_ocxo_dac_commit_target_hw_code = 0;
static uint64_t        g_ocxo_dac_last_schedule_second = 0;
static uint64_t        g_ocxo_dac_last_commit_second = 0;
static uint8_t         g_ocxo_dac_last_winner = 0;
static uint32_t        g_ocxo_dac_arbitration_passes = 0;
static uint32_t        g_ocxo_dac_no_candidate_passes = 0;
static uint32_t        g_ocxo_dac_deferred_candidates = 0;
static uint32_t        g_ocxo_dac_schedule_failures = 0;

static uint8_t ocxo_dac_id(const ocxo_dac_state_t* d) {
  if (d == &ocxo1_dac) return 1;
  if (d == &ocxo2_dac) return 2;
  return 0;
}

static void ocxo_dac_clear_pending(ocxo_dac_state_t& d) {
  d.pacing_pending = false;
  d.pacing_pending_target = 0.0;
  d.pacing_pending_step = 0.0;
  d.pacing_pending_hw_code = 0;
  d.pacing_pending_since_second = 0;
}

static void ocxo_dac_pacing_reset(void) {
  ocxo_dac_clear_pending(ocxo1_dac);
  ocxo_dac_clear_pending(ocxo2_dac);
  g_ocxo_dac_commit_scheduled = false;
  g_ocxo_dac_commit_selected = nullptr;
  g_ocxo_dac_commit_target = 0.0;
  g_ocxo_dac_commit_target_hw_code = 0;
  g_ocxo_dac_last_winner = 0;
}

static void ocxo_dac_pacing_abort_all(void) {
  ocxo_dac_pacing_reset();
}

static bool ocxo_dac_pending_eligible(const ocxo_dac_state_t& d) {
  return d.pacing_pending;
}

static ocxo_dac_state_t* ocxo_dac_pick_commit_candidate(void) {
  const bool e1 = ocxo_dac_pending_eligible(ocxo1_dac);
  const bool e2 = ocxo_dac_pending_eligible(ocxo2_dac);
  if (!e1 && !e2) return nullptr;
  if (e1 && !e2) return &ocxo1_dac;
  if (!e1 && e2) return &ocxo2_dac;
  return (g_ocxo_dac_last_winner == 1) ? &ocxo2_dac : &ocxo1_dac;
}

static void ocxo_dac_queue_intent(ocxo_dac_state_t& dac, double step) {
  const double target = dac.dac_fractional + step;
  uint16_t hw_code = (uint16_t)(target + 0.5);
  if (hw_code == dac.dac_hw_code) {
    dac.pacing_skip_small_delta_count++;
    ocxo_dac_clear_pending(dac);
    return;
  }
  dac.pacing_pending = true;
  dac.pacing_pending_target = target;
  dac.pacing_pending_step = step;
  dac.pacing_pending_hw_code = hw_code;
  dac.pacing_pending_since_second = campaign_seconds;
  dac.pacing_last_request_second = campaign_seconds;
  dac.pacing_intents++;
}

static void ocxo_dac_commit_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  g_ocxo_dac_commit_scheduled = false;

  ocxo_dac_state_t* dac = g_ocxo_dac_commit_selected;
  g_ocxo_dac_commit_selected = nullptr;
  if (!dac) return;

  const double target = g_ocxo_dac_commit_target;
  const double step   = target - dac->dac_fractional;
  const bool ok = ocxo_dac_set(*dac, target);
  if (!ok) {
    dac->io_fault_latched = true;
    calibrate_ocxo_mode = servo_mode_t::OFF;
    ocxo_dac_pacing_abort_all();
    return;
  }
  dac->servo_last_step = step;
  dac->servo_adjustments++;
  dac->pacing_commit_count++;
  dac->pacing_last_commit_second = campaign_seconds;
  g_ocxo_dac_last_commit_second = campaign_seconds;
  ocxo_dac_clear_pending(*dac);
}

static void ocxo_dac_schedule_paced_commit(void) {
  g_ocxo_dac_arbitration_passes++;

  if (g_ocxo_dac_commit_scheduled) return;

  ocxo_dac_state_t* winner = ocxo_dac_pick_commit_candidate();
  if (!winner) {
    g_ocxo_dac_no_candidate_passes++;
    return;
  }

  ocxo_dac_state_t* loser = nullptr;
  if (winner == &ocxo1_dac && ocxo_dac_pending_eligible(ocxo2_dac)) loser = &ocxo2_dac;
  if (winner == &ocxo2_dac && ocxo_dac_pending_eligible(ocxo1_dac)) loser = &ocxo1_dac;
  if (loser) {
    loser->pacing_deferred_count++;
    g_ocxo_dac_deferred_candidates++;
  }

  g_ocxo_dac_commit_selected = winner;
  g_ocxo_dac_commit_target = winner->pacing_pending_target;
  g_ocxo_dac_commit_target_hw_code = winner->pacing_pending_hw_code;
  g_ocxo_dac_last_schedule_second = campaign_seconds;
  g_ocxo_dac_last_winner = ocxo_dac_id(winner);

  const timepop_handle_t h =
      timepop_arm_alap(ocxo_dac_commit_callback, nullptr, OCXO_DAC_COMMIT_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_ocxo_dac_schedule_failures++;
    g_ocxo_dac_commit_selected = nullptr;
    g_ocxo_dac_commit_target = 0.0;
    g_ocxo_dac_commit_target_hw_code = 0;
    winner->pacing_deferred_count++;
    g_ocxo_dac_deferred_candidates++;
    return;
  }

  g_ocxo_dac_commit_scheduled = true;
}

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void) {
  timebase_invalidate();

  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;

  campaign_public_bases_reset_to_current();

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
  welford_reset(welford_pps_witness);
  welford_reset(welford_ocxo1_dac);
  welford_reset(welford_ocxo2_dac);

  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);
  ocxo_dac_pacing_reset();

  now_window_reset(g_now_window_ocxo1);
  now_window_reset(g_now_window_ocxo2);
}

// ============================================================================
// Servo logic
// ============================================================================

static void ocxo_servo_predictive(ocxo_dac_state_t& dac,
                                  double residual_signal_ns,
                                  double residual_alpha,
                                  double slope_alpha,
                                  double horizon_s) {
  dac.servo_last_residual = residual_signal_ns;

  if (!dac.servo_predictor_initialized) {
    dac.servo_predictor_initialized = true;
    dac.servo_last_raw_residual = residual_signal_ns;
    dac.servo_filtered_residual = residual_signal_ns;
    dac.servo_filtered_slope = 0.0;
    dac.servo_predicted_residual = residual_signal_ns;
    dac.servo_predictor_updates = 1;
    return;
  }

  const double raw_slope = residual_signal_ns - dac.servo_last_raw_residual;
  dac.servo_last_raw_residual = residual_signal_ns;

  dac.servo_filtered_residual =
      (1.0 - residual_alpha) * dac.servo_filtered_residual +
      residual_alpha * residual_signal_ns;

  dac.servo_filtered_slope =
      (1.0 - slope_alpha) * dac.servo_filtered_slope +
      slope_alpha * raw_slope;

  dac.servo_predicted_residual =
      dac.servo_filtered_residual + dac.servo_filtered_slope * horizon_s;

  dac.servo_predictor_updates++;

  if (fabs(dac.servo_filtered_residual) < SERVO_MIN_RESIDUAL_NS &&
      fabs(dac.servo_predicted_residual) < SERVO_MIN_PREDICTED_NS) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  double step = -dac.servo_predicted_residual / SERVO_NS_PER_DAC_LSB;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  if (fabs(step) < 1.0) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  dac.servo_last_step = step;
  ocxo_dac_queue_intent(dac, step);
}

static void ocxo_servo_total(ocxo_dac_state_t& dac, const welford_t& w) {
  if (w.n < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  ocxo_servo_predictive(dac,
                        w.mean,
                        SERVO_TOTAL_RESIDUAL_ALPHA,
                        SERVO_TOTAL_SLOPE_ALPHA,
                        SERVO_TOTAL_HORIZON_S);

  dac.servo_settle_count = 0;
}

static void ocxo_servo_now(ocxo_dac_state_t& dac, const now_window_t& window) {
  if (window.count < NOW_WINDOW_SIZE) return;
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;

  ocxo_servo_predictive(dac,
                        now_window_mean(window),
                        SERVO_NOW_RESIDUAL_ALPHA,
                        SERVO_NOW_SLOPE_ALPHA,
                        SERVO_NOW_HORIZON_S);
}

static void ocxo_calibration_servo(void) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    ocxo_servo_now(ocxo1_dac, g_now_window_ocxo1);
    ocxo_servo_now(ocxo2_dac, g_now_window_ocxo2);
  } else {
    ocxo_servo_total(ocxo1_dac, welford_ocxo1);
    ocxo_servo_total(ocxo2_dac, welford_ocxo2);
  }

  ocxo_dac_schedule_paced_commit();
}

// ============================================================================
// Watchdog
// ============================================================================

static void clocks_force_stop_campaign(void) {
  campaign_state = clocks_campaign_state_t::STOPPED;
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  ocxo_dac_pacing_abort_all();
  campaign_warmup_reset();
  timebase_invalidate();
}

static void clocks_watchdog_anomaly_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!watchdog_anomaly_publish_pending) {
    clocks_force_stop_campaign();
    return;
  }

  Payload p;
  p.add("sequence",         watchdog_anomaly_sequence);
  p.add("reason",           watchdog_anomaly_reason);
  p.add("campaign",         campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("trigger_dwt",      watchdog_anomaly_trigger_dwt);
  p.add("detail0",          watchdog_anomaly_detail0);
  p.add("detail1",          watchdog_anomaly_detail1);
  p.add("detail2",          watchdog_anomaly_detail2);
  p.add("detail3",          watchdog_anomaly_detail3);

  publish("WATCHDOG_ANOMALY", p);

  watchdog_anomaly_publish_pending = false;
  clocks_force_stop_campaign();
}

void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3) {
  if (!watchdog_anomaly_active) {
    watchdog_anomaly_sequence++;
    safeCopy(watchdog_anomaly_reason, sizeof(watchdog_anomaly_reason),
             (reason && *reason) ? reason : "watchdog_anomaly");
    watchdog_anomaly_detail0 = detail0;
    watchdog_anomaly_detail1 = detail1;
    watchdog_anomaly_detail2 = detail2;
    watchdog_anomaly_detail3 = detail3;
    watchdog_anomaly_trigger_dwt = DWT_CYCCNT;
    watchdog_anomaly_publish_pending = true;
  }

  watchdog_anomaly_active = true;
  ocxo_dac_pacing_abort_all();

  const timepop_handle_t h =
      timepop_arm_asap(clocks_watchdog_anomaly_callback, nullptr, "clocks-anomaly");

  if (h == TIMEPOP_INVALID_HANDLE) {
    watchdog_anomaly_publish_pending = false;
    clocks_force_stop_campaign();
  }
}

// ============================================================================
// clocks_beta_pps — invoked from alpha's pps_edge_callback
// ============================================================================

void clocks_beta_pps(void) {
  // ── Zero / start handshake ──
  //
  // Under PPS-anchored epoch discipline, all of the interrupt-side
  // synchronization lives in alpha:
  //
  //   • alpha's pps_edge_callback, on seeing request_zero/start and
  //     !g_epoch_pending, calls alpha_request_epoch_zero() which sets
  //     g_epoch_pending AND arms a GPIO-ISR-side rebootstrap.
  //   • The NEXT physical PPS edge consumes the rebootstrap (re-phasing
  //     the VCLOCK cadence) and alpha installs the epoch from that
  //     snapshot, clearing g_epoch_pending.
  //
  // Beta's job here is simply to wait for alpha to complete the
  // PPS-anchored install.  When g_epoch_pending clears, the campaign
  // time base is aligned to a physical PPS moment and we can finalize
  // the zero/start transition.
  if (request_zero || request_start) {
    if (clocks_epoch_pending()) return;

    clocks_zero_all();

    request_zero = false;

    if (request_start) {
      watchdog_anomaly_active = false;
      campaign_state = clocks_campaign_state_t::STARTED;
      request_start = false;
      campaign_warmup_begin(campaign_warmup_mode_t::START);
    }
    return;
  }

  if (request_stop) {
    watchdog_anomaly_active = false;
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    calibrate_ocxo_mode = servo_mode_t::OFF;
    ocxo_dac_pacing_abort_all();
    campaign_warmup_reset();
    timebase_invalidate();
    return;
  }

  if (request_recover) {
    watchdog_anomaly_active = false;
    request_zero = false;
    ocxo_dac_pacing_abort_all();

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_ticks_64        = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    request_recover = false;
    campaign_state = clocks_campaign_state_t::STARTED;
    campaign_warmup_begin(campaign_warmup_mode_t::RECOVER);
    return;
  }

  if (watchdog_anomaly_active) return;
  if (campaign_state != clocks_campaign_state_t::STARTED) return;

  // ── Warmup suppression ──
  //
  // Suppressed records still allow alpha's measurement/predictor state to
  // settle, but they are not allowed to become canonical TIMEBASE_FRAGMENT
  // rows.  START hides these seconds completely; RECOVER counts them as
  // deliberate canonical gaps.
  if (campaign_warmup_consume_one_candidate_record()) {
    return;
  }

  // ── Per-second campaign work ──
  campaign_seconds++;

  dwt_cycle_count_total = campaign_public_dwt_total();
  gnss_raw_64           = campaign_public_gnss_ns() / 100ull;
  ocxo1_ticks_64        = campaign_public_ocxo1_ns() / 100ull;
  ocxo2_ticks_64        = campaign_public_ocxo2_ns() / 100ull;

  // ── Welford updates ──
  //
  // DWT sample is fed directly as ppb using the uniform sign convention:
  // positive ppb → Teensy running FAST (measured cycles > expected).
  {
    const double cycles = (double)g_dwt_cycle_count_between_pps;
    const double expected = (double)DWT_EXPECTED_PER_PPS;
    const double dwt_ppb_sample = (cycles - expected) / expected * 1e9;
    welford_update(welford_dwt, dwt_ppb_sample);
  }

  // VCLOCK measurement Welford — bridge-interpolation self-test.
  // Sample in ns; mean == ppb under the "ns/sec == ppb" identity.
  //
  // Guard: gnss_ns_between_edges is zero only on the first-edge bootstrap
  // (clocks_apply_edge writes 0 when there's no previous edge).  Feeding
  // that bootstrap 0 would pin welford_min to 0 and bias the mean.
  if (g_vclock_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_vclock, (double)g_vclock_measurement.second_residual_ns);
  }

  // OCXO measurement Welfords — same bootstrap guard.
  if (g_ocxo1_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_ocxo1, (double)g_ocxo1_measurement.second_residual_ns);
    now_window_push(g_now_window_ocxo1, g_ocxo1_measurement.second_residual_ns);
  }

  if (g_ocxo2_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_ocxo2, (double)g_ocxo2_measurement.second_residual_ns);
    now_window_push(g_now_window_ocxo2, g_ocxo2_measurement.second_residual_ns);
  }

  // welford_pps_witness is fed by alpha's vclock_callback from the GPIO
  // witness snapshot.  We do NOT update it here.

  // Servo runs AFTER welford updates so it sees this second's data.
  ocxo_calibration_servo();

  // DAC Welfords — track servo effort (the TEMPEST signal).
  welford_update(welford_ocxo1_dac, ocxo1_dac.dac_fractional);
  welford_update(welford_ocxo2_dac, ocxo2_dac.dac_fractional);

  // ── Build TIMEBASE_FRAGMENT ──
  Payload p;
  p.add("campaign", campaign_name);
  p.add("teensy_pps_count", campaign_seconds);
  p.add("gnss_ns", campaign_public_gnss_ns());
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  // DWT canonical surface.
  p.add("dwt_cycle_count_total", dwt_cycle_count_total);
  p.add("dwt_cycle_count_at_pps", g_dwt_cycle_count_at_pps);
  p.add("dwt_cycle_count_between_pps", g_dwt_cycle_count_between_pps);
  p.add("dwt_expected_per_pps", (uint32_t)DWT_EXPECTED_PER_PPS);
  // Instantaneous cycles residual: measured minus expected.
  // Positive → Teensy CPU fast; negative → Teensy CPU slow.
  // Same sign convention as dwt_ppb, same units as dwt_cycle_count_between_pps.
  p.add("dwt_second_residual_cycles",
        (int32_t)((int64_t)g_dwt_cycle_count_between_pps -
                  (int64_t)DWT_EXPECTED_PER_PPS));

  // Prediction residual (diagnostic): measured[N] - measured[N-1].
  // The bridge's implicit one-step predictor carries rate[N-1] into
  // second N, so this IS the predictor's per-second error.  Near
  // zero in steady state; non-zero mean during thermal settling;
  // spikes indicate per-second anomalies.
  p.add("dwt_prediction_residual_cycles",
        g_dwt_prediction_residual_cycles);

  // QTimer counter32 is no longer published.  The post-scorched-earth
  // event payload is {gnss_ns_at_edge, dwt_at_edge, sequence} only.

  // VCLOCK surface — authored facts + per-edge measurement + window accounting.
  p.add("vclock_ns_count_at_pps", campaign_public_gnss_ns());
  p.add("vclock_gnss_ns_between_edges", g_vclock_measurement.gnss_ns_between_edges);
  p.add("vclock_dwt_cycles_between_edges", g_vclock_measurement.dwt_cycles_between_edges);
  p.add("vclock_second_residual_ns", g_vclock_measurement.second_residual_ns);
  p.add("vclock_phase_offset_ns", g_vclock_clock.phase_offset_ns);
  p.add("vclock_zero_established", g_vclock_clock.zero_established);
  p.add("vclock_window_checks", g_vclock_clock.window_checks);
  p.add("vclock_window_mismatches", g_vclock_clock.window_mismatches);
  p.add("vclock_window_error_ns", g_vclock_clock.window_error_ns);

  // OCXO1 surface.
  p.add("ocxo1_ns_count_at_pps", campaign_public_ocxo1_ns());
  p.add("ocxo1_gnss_ns_between_edges", g_ocxo1_measurement.gnss_ns_between_edges);
  p.add("ocxo1_dwt_cycles_between_edges", g_ocxo1_measurement.dwt_cycles_between_edges);
  p.add("ocxo1_second_residual_ns", g_ocxo1_measurement.second_residual_ns);
  p.add("ocxo1_phase_offset_ns", g_ocxo1_clock.phase_offset_ns);
  p.add("ocxo1_zero_established", g_ocxo1_clock.zero_established);
  p.add("ocxo1_window_checks", g_ocxo1_clock.window_checks);
  p.add("ocxo1_window_mismatches", g_ocxo1_clock.window_mismatches);
  p.add("ocxo1_window_error_ns", g_ocxo1_clock.window_error_ns);

  // OCXO2 surface.
  p.add("ocxo2_ns_count_at_pps", campaign_public_ocxo2_ns());
  p.add("ocxo2_gnss_ns_between_edges", g_ocxo2_measurement.gnss_ns_between_edges);
  p.add("ocxo2_dwt_cycles_between_edges", g_ocxo2_measurement.dwt_cycles_between_edges);
  p.add("ocxo2_second_residual_ns", g_ocxo2_measurement.second_residual_ns);
  p.add("ocxo2_phase_offset_ns", g_ocxo2_clock.phase_offset_ns);
  p.add("ocxo2_zero_established", g_ocxo2_clock.zero_established);
  p.add("ocxo2_window_checks", g_ocxo2_clock.window_checks);
  p.add("ocxo2_window_mismatches", g_ocxo2_clock.window_mismatches);
  p.add("ocxo2_window_error_ns", g_ocxo2_clock.window_error_ns);

  // DAC + servo state (live values).
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo1_dac_fault_latched", ocxo1_dac.io_fault_latched);
  p.add("ocxo1_servo_predicted_residual_ns", ocxo1_dac.servo_predicted_residual, 6);
  p.add("ocxo1_servo_last_step", ocxo1_dac.servo_last_step, 6);
  p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);

  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("ocxo2_dac_fault_latched", ocxo2_dac.io_fault_latched);
  p.add("ocxo2_servo_predicted_residual_ns", ocxo2_dac.servo_predicted_residual, 6);
  p.add("ocxo2_servo_last_step", ocxo2_dac.servo_last_step, 6);
  p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);

  // ── Standardized statistics publication ──
  //
  // Every Welford publishes the same six fields.  Every frequency-bearing
  // clock publishes the same tau/ppb pair.  Uniform sign convention:
  // positive ppb = clock running FAST vs GNSS reference.

  // Welford blocks (seven).
  publish_welford(p, "dwt",          welford_dwt);
  publish_welford(p, "vclock",       welford_vclock);
  publish_welford(p, "ocxo1",        welford_ocxo1);
  publish_welford(p, "ocxo2",        welford_ocxo2);
  publish_welford(p, "pps_witness",  welford_pps_witness);
  publish_welford(p, "ocxo1_dac",    welford_ocxo1_dac);
  publish_welford(p, "ocxo2_dac",    welford_ocxo2_dac);

  // Frequency pairs (four).  For all four clocks the Welford samples are
  // already in ppb-compatible units (ppb for DWT; ns/sec == ppb numerically
  // for VCLOCK and OCXOs).
  publish_freq(p, "dwt",    welford_dwt.mean);
  publish_freq(p, "vclock", welford_vclock.mean);
  publish_freq(p, "ocxo1",  welford_ocxo1.mean);
  publish_freq(p, "ocxo2",  welford_ocxo2.mean);

  // Per-lane event fact (gnss_ns_at_edge, dwt_at_edge, sequence).  The
  // PPS_VCLOCK lane's gnss_ns is VCLOCK-authored (sequence × 1e9, ends
  // in "00000000"); VCLOCK/OCXO lanes' gnss_ns is bridge-projected.
  clocks_payload_add_event(p, "pps_vclock", g_last_pps_vclock_event);
  clocks_payload_add_event(p, "vclock",     g_last_vclock_event);
  clocks_payload_add_event(p, "ocxo1",      g_last_ocxo1_event);
  clocks_payload_add_event(p, "ocxo2",      g_last_ocxo2_event);

  g_last_fragment = p;
  publish("TIMEBASE_FRAGMENT", p);
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_start(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);

  ocxo_dac_pacing_abort_all();
  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (args.tryGetDouble("set_dac1", dac_val)) dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_mode = servo_mode_parse(args.getString("calibrate_ocxo"));
  if (!dac1_ok || !dac2_ok) calibrate_ocxo_mode = servo_mode_t::OFF;

  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);
  ocxo_dac_pacing_reset();

  request_start = true;
  request_stop = false;
  request_recover = false;

  Payload p;
  p.add("status", (!dac1_ok || !dac2_ok) ?
                      "start_requested_dac_fault" : "start_requested");
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  return p;
}

static Payload cmd_stop(const Payload&) {
  request_stop = true;
  request_start = false;
  request_zero = false;
  Payload p;
  p.add("status", "stop_requested");
  return p;
}

static Payload cmd_zero(const Payload&) {
  request_zero = true;
  request_start = false;
  request_stop = false;
  request_recover = false;
  Payload p;
  p.add("status", "zero_requested");
  return p;
}

static Payload cmd_recover(const Payload& args) {
  const char* s_dwt_ns = args.getString("dwt_ns");
  const char* s_gnss   = args.getString("gnss_ns");
  const char* s_ocxo1  = args.getString("ocxo1_ns");
  const char* s_ocxo2  = args.getString("ocxo2_ns");

  if (!s_dwt_ns || !s_gnss || !s_ocxo1 || !s_ocxo2) {
    Payload err;
    err.add("error", "missing recovery parameters (dwt_ns, gnss_ns, ocxo1_ns, ocxo2_ns)");
    return err;
  }

  recover_dwt_ns   = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns  = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo1_ns = strtoull(s_ocxo1,  nullptr, 10);
  recover_ocxo2_ns = strtoull(s_ocxo2,  nullptr, 10);

  request_recover = true;
  request_start   = false;
  request_stop    = false;
  request_zero    = false;

  Payload p;
  p.add("status", "recover_requested");
  return p;
}

static Payload cmd_watchdog_test(const Payload&) {
  clocks_watchdog_anomaly("watchdog_test");
  Payload p;
  p.add("status", "watchdog_anomaly_requested");
  return p;
}

static Payload cmd_report(const Payload&) {
  Payload p = g_last_fragment.clone();
  p.add("campaign_state",
        campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  p.add("request_start", request_start);
  p.add("request_stop", request_stop);
  p.add("request_recover", request_recover);
  p.add("request_zero", request_zero);
  p.add("epoch_pending", clocks_epoch_pending());
  p.add("interrupt_pps_rebootstrap_pending",
        interrupt_pps_rebootstrap_pending());
  p.add("campaign_warmup_active", campaign_warmup_active());
  p.add("campaign_warmup_required", CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS);
  p.add("campaign_warmup_remaining", (uint32_t)g_campaign_warmup_remaining);
  p.add("campaign_warmup_suppressed_total",
        (uint32_t)g_campaign_warmup_suppressed_total);
  return p;
}

static Payload cmd_set_dac(const Payload& args) {
  ocxo_dac_pacing_abort_all();

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (args.tryGetDouble("set_dac1", dac_val)) {
    dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
    if (dac1_ok) ocxo_dac_retry_reset(ocxo1_dac);
  }
  if (args.tryGetDouble("set_dac2", dac_val)) {
    dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);
    if (dac2_ok) ocxo_dac_retry_reset(ocxo2_dac);
  }

  Payload p;
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("status", (dac1_ok && dac2_ok) ? "ok" : "dac_write_fault");
  return p;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",         cmd_start         },
  { "STOP",          cmd_stop          },
  { "ZERO",          cmd_zero          },
  { "RECOVER",       cmd_recover       },
  { "REPORT",        cmd_report        },
  { "WATCHDOG_TEST", cmd_watchdog_test },
  { "SET_DAC",       cmd_set_dac       },
  { nullptr,          nullptr           }
};

static const process_subscription_entry_t CLOCKS_SUBSCRIPTIONS[] = {
  { "TIMEBASE_FRAGMENT", on_timebase_fragment },
  { nullptr, nullptr },
};

const process_subscription_entry_t* timebase_subscriptions(void) {
  return CLOCKS_SUBSCRIPTIONS;
}

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = CLOCKS_SUBSCRIPTIONS
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}