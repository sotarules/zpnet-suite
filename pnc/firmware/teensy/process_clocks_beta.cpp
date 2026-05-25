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
//     ocxo1_welford       — OCXO1 PPS-interval residual (ns, positive = fast)
//     ocxo2_welford       — OCXO2 PPS-interval residual (ns, positive = fast)
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
#include <strings.h>

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

// Alpha-authored physical PPS witness DWT audit surface.  These are published
// into TIMEBASE_FRAGMENT so Pi-side reports can compare physical PPS-to-PPS
// DWT intervals against the canonical PPS/VCLOCK DWT rail.
extern volatile uint32_t g_pps_dwt_at_edge;
extern volatile uint32_t g_pps_dwt_cycles_between_edges;
extern volatile bool     g_pps_dwt_cycles_between_edges_valid;

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
//   is teensy_pps_vclock_count=1, gnss_ns=1e9, and public totals start from that
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

// Step C: OCXO public/canonical bases now track the PPS-edge projected
// OCXO clock values. Keep separate bases for the legacy measured-GNSS
// ledgers so measured_gnss_ns remains useful as a diagnostic side surface.
static uint64_t g_campaign_public_ocxo1_measured_base = 0;
static uint64_t g_campaign_public_ocxo2_measured_base = 0;

// Step D: canonical OCXO residuals/Welfords are now PPS-founded.  The
// previous public TIMEBASE clock tuple is retained so Beta can compute the
// row-to-row PPS-interval residual directly from the same public values that
// downstream panels see:
//
//   residual_ns = (current_ocxo_ns - previous_ocxo_ns)
//               - (current_gnss_ns - previous_gnss_ns)
//
// Positive residual means the OCXO clock ledger advanced too far during the
// PPS interval: clock running fast.
struct pps_interval_residuals_t {
  bool     ocxo1_valid = false;
  bool     ocxo2_valid = false;
  uint32_t public_count = 0;
  uint64_t gnss_interval_ns = 0;
  uint64_t ocxo1_interval_ns = 0;
  uint64_t ocxo2_interval_ns = 0;
  int64_t  ocxo1_fast_residual_ns = 0;
  int64_t  ocxo2_fast_residual_ns = 0;
};

static uint32_t g_pps_residual_prev_public_count = 0;
static uint64_t g_pps_residual_prev_gnss_ns = 0;
static uint64_t g_pps_residual_prev_ocxo1_ns = 0;
static uint64_t g_pps_residual_prev_ocxo2_ns = 0;
static bool     g_pps_residual_prev_ocxo1_pps_projected = false;
static bool     g_pps_residual_prev_ocxo2_pps_projected = false;
static pps_interval_residuals_t g_last_pps_interval_residuals = {};

static void pps_interval_residuals_reset(void) {
  g_pps_residual_prev_public_count = 0;
  g_pps_residual_prev_gnss_ns = 0;
  g_pps_residual_prev_ocxo1_ns = 0;
  g_pps_residual_prev_ocxo2_ns = 0;
  g_pps_residual_prev_ocxo1_pps_projected = false;
  g_pps_residual_prev_ocxo2_pps_projected = false;
  g_last_pps_interval_residuals = pps_interval_residuals_t{};
}

static pps_interval_residuals_t pps_interval_residuals_update(
    uint32_t public_count,
    uint64_t public_gnss_ns,
    uint64_t public_ocxo1_ns,
    uint64_t public_ocxo2_ns,
    bool ocxo1_pps_projected,
    bool ocxo2_pps_projected) {
  pps_interval_residuals_t r{};
  r.public_count = public_count;

  const bool have_previous = (g_pps_residual_prev_public_count != 0);
  const bool consecutive = have_previous &&
      (public_count == (g_pps_residual_prev_public_count + 1U));
  const bool gnss_monotonic = public_gnss_ns >= g_pps_residual_prev_gnss_ns;
  const bool ocxo1_monotonic = public_ocxo1_ns >= g_pps_residual_prev_ocxo1_ns;
  const bool ocxo2_monotonic = public_ocxo2_ns >= g_pps_residual_prev_ocxo2_ns;

  if (consecutive && gnss_monotonic) {
    r.gnss_interval_ns = public_gnss_ns - g_pps_residual_prev_gnss_ns;

    if (r.gnss_interval_ns != 0 &&
        ocxo1_monotonic &&
        g_pps_residual_prev_ocxo1_pps_projected &&
        ocxo1_pps_projected) {
      r.ocxo1_interval_ns = public_ocxo1_ns - g_pps_residual_prev_ocxo1_ns;
      r.ocxo1_fast_residual_ns =
          (int64_t)r.ocxo1_interval_ns - (int64_t)r.gnss_interval_ns;
      r.ocxo1_valid = true;
    }

    if (r.gnss_interval_ns != 0 &&
        ocxo2_monotonic &&
        g_pps_residual_prev_ocxo2_pps_projected &&
        ocxo2_pps_projected) {
      r.ocxo2_interval_ns = public_ocxo2_ns - g_pps_residual_prev_ocxo2_ns;
      r.ocxo2_fast_residual_ns =
          (int64_t)r.ocxo2_interval_ns - (int64_t)r.gnss_interval_ns;
      r.ocxo2_valid = true;
    }
  }

  g_pps_residual_prev_public_count = public_count;
  g_pps_residual_prev_gnss_ns = public_gnss_ns;
  g_pps_residual_prev_ocxo1_ns = public_ocxo1_ns;
  g_pps_residual_prev_ocxo2_ns = public_ocxo2_ns;
  g_pps_residual_prev_ocxo1_pps_projected = ocxo1_pps_projected;
  g_pps_residual_prev_ocxo2_pps_projected = ocxo2_pps_projected;
  g_last_pps_interval_residuals = r;
  return r;
}

static bool clock_projection_valid(time_clock_id_t clock) {
  time_clock_projection_t projection{};
  return time_clock_projection(clock, &projection);
}

static uint64_t current_raw_gnss_ns(void) {
  const uint32_t fragment_dwt = g_dwt_at_pps_vclock;
  if (fragment_dwt != 0 && clock_projection_valid(time_clock_id_t::VCLOCK)) {
    return time_gnss_ns_at_dwt(fragment_dwt);
  }
  return g_gnss_ns_at_pps_vclock;
}

static bool current_raw_ocxo_pps_projected_ns(time_clock_id_t clock,
                                              uint64_t* out_raw_ns) {
  if (!out_raw_ns) return false;
  *out_raw_ns = 0;

  clocks_alpha_ocxo_pps_projection_snapshot_t s{};
  if (!clocks_alpha_ocxo_pps_projection_snapshot(clock, &s)) return false;
  if (!s.valid || s.projected_ocxo_ns_at_pps == 0) return false;

  *out_raw_ns = s.projected_ocxo_ns_at_pps;
  return true;
}

static uint64_t current_raw_ocxo1_measured_ns(void) {
  // Legacy measured GNSS-elapsed ledger retained as a diagnostic surface.
  return clocks_ocxo1_measured_gnss_ns_now();
}

static uint64_t current_raw_ocxo2_measured_ns(void) {
  // Legacy measured GNSS-elapsed ledger retained as a diagnostic surface.
  return clocks_ocxo2_measured_gnss_ns_now();
}

static uint64_t current_raw_ocxo1_ns(void) {
  // Step C authority: prefer Alpha's PPS/VCLOCK-edge OCXO clock projection.
  // Fall back to the legacy measured ledger only until the projection surface
  // has enough edge history to become valid.
  uint64_t projected = 0;
  if (current_raw_ocxo_pps_projected_ns(time_clock_id_t::OCXO1, &projected)) {
    return projected;
  }
  return current_raw_ocxo1_measured_ns();
}

static uint64_t current_raw_ocxo2_ns(void) {
  // Step C authority: prefer Alpha's PPS/VCLOCK-edge OCXO clock projection.
  // Fall back to the legacy measured ledger only until the projection surface
  // has enough edge history to become valid.
  uint64_t projected = 0;
  if (current_raw_ocxo_pps_projected_ns(time_clock_id_t::OCXO2, &projected)) {
    return projected;
  }
  return current_raw_ocxo2_measured_ns();
}

static uint64_t campaign_public_from_base(uint64_t raw_ns,
                                          uint64_t base_ns) {
  return (raw_ns >= base_ns) ? (raw_ns - base_ns) : 0;
}

static uint64_t campaign_public_ocxo1_measured_ns(void) {
  return campaign_public_from_base(current_raw_ocxo1_measured_ns(),
                                   g_campaign_public_ocxo1_measured_base);
}

static uint64_t campaign_public_ocxo2_measured_ns(void) {
  return campaign_public_from_base(current_raw_ocxo2_measured_ns(),
                                   g_campaign_public_ocxo2_measured_base);
}

static void campaign_public_bases_reset_to_current(void) {
  g_campaign_public_dwt_base   = g_dwt_cycle_count_total;
  g_campaign_public_gnss_base  = current_raw_gnss_ns();
  g_campaign_public_ocxo1_base = current_raw_ocxo1_ns();
  g_campaign_public_ocxo2_base = current_raw_ocxo2_ns();
  g_campaign_public_ocxo1_measured_base = current_raw_ocxo1_measured_ns();
  g_campaign_public_ocxo2_measured_base = current_raw_ocxo2_measured_ns();
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
      saturated_base_for_recovered_value(current_raw_gnss_ns(),
                                          recover_gnss_ns);
  g_campaign_public_ocxo1_base =
      saturated_base_for_recovered_value(current_raw_ocxo1_ns(),
                                          recover_ocxo1_ns);
  g_campaign_public_ocxo2_base =
      saturated_base_for_recovered_value(current_raw_ocxo2_ns(),
                                          recover_ocxo2_ns);
  g_campaign_public_ocxo1_measured_base =
      saturated_base_for_recovered_value(current_raw_ocxo1_measured_ns(),
                                          recover_ocxo1_ns);
  g_campaign_public_ocxo2_measured_base =
      saturated_base_for_recovered_value(current_raw_ocxo2_measured_ns(),
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
    // next selected PPS/VCLOCK edge after this one, so use the current alpha totals as the base.
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
  const uint64_t now = current_raw_gnss_ns();
  return (now >= g_campaign_public_gnss_base)
           ? (now - g_campaign_public_gnss_base)
           : 0;
}

static uint64_t campaign_public_ocxo1_ns(void) {
  const uint64_t now = current_raw_ocxo1_ns();
  return (now >= g_campaign_public_ocxo1_base)
           ? (now - g_campaign_public_ocxo1_base)
           : 0;
}

static uint64_t campaign_public_ocxo2_ns(void) {
  const uint64_t now = current_raw_ocxo2_ns();
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

static const char* smartzero_lane_state_name_beta(interrupt_smartzero_lane_state_t s) {
  switch (s) {
    case interrupt_smartzero_lane_state_t::ACQUIRING: return "ACQUIRING";
    case interrupt_smartzero_lane_state_t::LOCKED:    return "LOCKED";
    default:                                         return "IDLE";
  }
}

static const char* smartzero_phase_name_beta(interrupt_smartzero_phase_t p) {
  switch (p) {
    case interrupt_smartzero_phase_t::RUNNING:  return "RUNNING";
    case interrupt_smartzero_phase_t::COMPLETE: return "COMPLETE";
    case interrupt_smartzero_phase_t::ABORTED:  return "ABORTED";
    default:                                    return "IDLE";
  }
}

static const char* smartzero_decision_name_beta(interrupt_smartzero_decision_t d) {
  switch (d) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:  return "WAITING_FOR_CPS";
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:     return "FIRST_SAMPLE";
    case interrupt_smartzero_decision_t::ACCEPTED:         return "ACCEPTED";
    case interrupt_smartzero_decision_t::REJECTED_DWT:     return "REJECTED_DWT";
    case interrupt_smartzero_decision_t::REJECTED_COUNTER: return "REJECTED_COUNTER";
    default:                                               return "NONE";
  }
}

static void payload_add_smartzero_lane(Payload& parent,
                                       const char* key,
                                       const interrupt_smartzero_lane_snapshot_t& z) {
  Payload lane;
  lane.add("kind", interrupt_subscriber_kind_str(z.kind));
  lane.add("state", smartzero_lane_state_name_beta(z.state));
  lane.add("last_decision", smartzero_decision_name_beta(z.last_decision));
  lane.add("sample_count", z.sample_count);
  lane.add("interval_attempt_count", z.interval_attempt_count);
  lane.add("accepted_count", z.accepted_count);
  lane.add("rejected_count", z.rejected_count);
  lane.add("waiting_for_cps_count", z.waiting_for_cps_count);
  lane.add("cps_used", z.cps_used);
  lane.add("expected_interval_cycles", z.expected_interval_cycles);
  lane.add("tolerance_cycles", z.tolerance_cycles);
  lane.add("required_counter_delta_ticks", z.required_counter_delta_ticks);
  lane.add("last_interval_cycles", z.last_interval_cycles);
  lane.add("last_interval_error_cycles", z.last_interval_error_cycles);
  lane.add("max_abs_interval_error_cycles", z.max_abs_interval_error_cycles);
  lane.add("last_counter_delta_ticks", z.last_counter_delta_ticks);
  lane.add("anchor_dwt", z.anchor_dwt);
  lane.add("anchor_counter32", z.anchor_counter32);
  lane.add("anchor_hardware16", (uint32_t)z.anchor_hardware16);
  lane.add("anchor_pair_previous_dwt", z.anchor_pair_previous_dwt);
  lane.add("anchor_pair_previous_counter32", z.anchor_pair_previous_counter32);
  lane.add("arm_count", z.arm_count);
  lane.add("fire_count", z.fire_count);
  parent.add_object(key, lane);
}

static void payload_add_smartzero_snapshot_object(
  Payload& parent,
  const char* key,
  const interrupt_smartzero_snapshot_t& z,
  bool valid,
  bool include_lanes
) {
  Payload obj;
  obj.add("valid", valid);
  obj.add("phase", smartzero_phase_name_beta(z.phase));
  obj.add("running", z.running);
  obj.add("complete", z.complete);
  obj.add("aborted", z.aborted);
  obj.add("sequence", z.sequence);
  obj.add("begin_count", z.begin_count);
  obj.add("complete_count", z.complete_count);
  obj.add("abort_count", z.abort_count);
  obj.add("current_lane", interrupt_subscriber_kind_str(z.current_lane));
  obj.add("current_lane_index", z.current_lane_index);
  obj.add("sample_rate_hz", z.sample_rate_hz);
  obj.add("counter_delta_ticks", z.counter_delta_ticks);
  obj.add("tolerance_cycles", z.tolerance_cycles);

  if (include_lanes) {
    Payload lanes;
    payload_add_smartzero_lane(lanes, "vclock", z.lanes[0]);
    payload_add_smartzero_lane(lanes, "ocxo1", z.lanes[1]);
    payload_add_smartzero_lane(lanes, "ocxo2", z.lanes[2]);
    obj.add_object("lanes", lanes);
  }

  parent.add_object(key, obj);
}

static void payload_add_prefixed_smartzero_compact(
  Payload& p,
  const char* prefix,
  const interrupt_smartzero_snapshot_t& z,
  bool valid
) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value ? value : "");
  };

  add_bool("valid", valid);
  add_str("phase", smartzero_phase_name_beta(z.phase));
  add_bool("running", z.running);
  add_bool("complete", z.complete);
  add_bool("aborted", z.aborted);
  add_u32("sequence", z.sequence);
  add_u32("begin_count", z.begin_count);
  add_u32("complete_count", z.complete_count);
  add_u32("abort_count", z.abort_count);
  add_str("current_lane", interrupt_subscriber_kind_str(z.current_lane));
  add_u32("current_lane_index", z.current_lane_index);
}

static void payload_add_smartzero_install_transaction(Payload& p) {
  p.add("smartzero_install_in_progress", clocks_alpha_epoch_install_in_progress());
  p.add("smartzero_install_attempt_count",
        clocks_alpha_smartzero_install_attempt_count());
  p.add("smartzero_install_commit_count",
        clocks_alpha_smartzero_install_commit_count());
  p.add("smartzero_install_failure_count",
        clocks_alpha_smartzero_install_failure_count());
  p.add("smartzero_install_last_stage",
        clocks_alpha_smartzero_install_last_stage());
  p.add("smartzero_install_last_stage_name",
        clocks_alpha_smartzero_install_last_stage_name());
  p.add("smartzero_install_last_failure_stage",
        clocks_alpha_smartzero_install_last_failure_stage());
  p.add("smartzero_install_last_failure_stage_name",
        clocks_alpha_smartzero_install_last_failure_stage_name());
  p.add("smartzero_install_last_failure_code",
        clocks_alpha_smartzero_install_last_failure_code());
  p.add("smartzero_install_last_live_sequence",
        clocks_alpha_smartzero_install_last_live_sequence());
  p.add("smartzero_install_last_prior_epoch_sequence",
        clocks_alpha_smartzero_install_last_prior_epoch_sequence());
  p.add("smartzero_install_last_committed_epoch_sequence",
        clocks_alpha_smartzero_install_last_committed_epoch_sequence());
  p.add("smartzero_install_last_committed_smartzero_sequence",
        clocks_alpha_smartzero_install_last_committed_smartzero_sequence());
  p.add("smartzero_install_last_success",
        clocks_alpha_smartzero_install_last_success());
  p.add("smartzero_install_last_atomic",
        clocks_alpha_smartzero_install_last_atomic());
  p.add("smartzero_install_last_reason",
        clocks_alpha_smartzero_install_last_reason());
}

static void payload_add_smartzero_summary(Payload& p) {
  interrupt_smartzero_snapshot_t live{};
  (void)interrupt_smartzero_live_snapshot(&live);

  interrupt_smartzero_snapshot_t installed{};
  const bool installed_valid = clocks_alpha_epoch_last_smartzero(&installed);
  const bool installed_backing_epoch = clocks_alpha_installed_smartzero_backing_epoch();

  payload_add_prefixed_smartzero_compact(p, "installed_smartzero", installed, installed_valid);
  p.add("installed_smartzero_backing_epoch", installed_backing_epoch);
  p.add("installed_smartzero_epoch_sequence", clocks_alpha_epoch_sequence());
  if (installed_valid) {
    p.add("installed_smartzero_vclock_anchor_dwt", installed.lanes[0].anchor_dwt);
    p.add("installed_smartzero_ocxo1_anchor_dwt", installed.lanes[1].anchor_dwt);
    p.add("installed_smartzero_ocxo2_anchor_dwt", installed.lanes[2].anchor_dwt);
    p.add("installed_smartzero_vclock_anchor_counter32", installed.lanes[0].anchor_counter32);
    p.add("installed_smartzero_ocxo1_anchor_counter32", installed.lanes[1].anchor_counter32);
    p.add("installed_smartzero_ocxo2_anchor_counter32", installed.lanes[2].anchor_counter32);
  } else {
    p.add("installed_smartzero_vclock_anchor_dwt", 0U);
    p.add("installed_smartzero_ocxo1_anchor_dwt", 0U);
    p.add("installed_smartzero_ocxo2_anchor_dwt", 0U);
    p.add("installed_smartzero_vclock_anchor_counter32", 0U);
    p.add("installed_smartzero_ocxo1_anchor_counter32", 0U);
    p.add("installed_smartzero_ocxo2_anchor_counter32", 0U);
  }

  payload_add_prefixed_smartzero_compact(p, "live_smartzero", live, true);
  p.add("smartzero_pending_active", clocks_alpha_smartzero_pending_active());
  p.add("smartzero_pending_reason", clocks_alpha_smartzero_pending_reason());
  payload_add_smartzero_install_transaction(p);
  p.add("smartzero_begin_service_epoch_preserved",
        clocks_alpha_smartzero_last_begin_preserved_epoch());
  p.add("smartzero_begin_preserved_epoch_sequence",
        clocks_alpha_smartzero_last_begin_preserved_epoch_sequence());
  p.add("smartzero_begin_reason", clocks_alpha_smartzero_last_begin_reason());
  p.add("smartzero_begin_preserved_epoch_count",
        clocks_alpha_smartzero_begin_preserved_epoch_count());
  p.add("smartzero_begin_cold_count",
        clocks_alpha_smartzero_begin_cold_count());

  // Legacy aliases retained for existing tools.  These refer ONLY to the live
  // acquisition attempt.  The installed proof is reported with the explicit
  // installed_smartzero_* prefix and object below.
  p.add("smartzero_running", live.running);
  p.add("smartzero_complete", live.complete);
  p.add("smartzero_aborted", live.aborted);
  p.add("smartzero_sequence", live.sequence);
  p.add("smartzero_begin_count", live.begin_count);
  p.add("smartzero_complete_count", live.complete_count);
  p.add("smartzero_abort_count", live.abort_count);
  p.add("smartzero_current_lane", interrupt_subscriber_kind_str(live.current_lane));
  p.add("smartzero_current_lane_index", live.current_lane_index);
  p.add("smartzero_sample_rate_hz", live.sample_rate_hz);
  p.add("smartzero_counter_delta_ticks", live.counter_delta_ticks);
  p.add("smartzero_tolerance_cycles", live.tolerance_cycles);

  payload_add_smartzero_snapshot_object(p, "installed_smartzero", installed, installed_valid, true);
  payload_add_smartzero_snapshot_object(p, "live_smartzero", live, true, true);

  Payload live_lanes;
  payload_add_smartzero_lane(live_lanes, "vclock", live.lanes[0]);
  payload_add_smartzero_lane(live_lanes, "ocxo1", live.lanes[1]);
  payload_add_smartzero_lane(live_lanes, "ocxo2", live.lanes[2]);
  p.add_object("smartzero", live_lanes);
}

static void publish_freq(Payload& p, const char* clock_name, double ppb_value) {
  char key[80];
  const double tau_value = 1.0 + ppb_value / 1e9;

  snprintf(key, sizeof(key), "%s_tau", clock_name);
  p.add(key, tau_value, 12);

  snprintf(key, sizeof(key), "%s_ppb", clock_name);
  p.add(key, ppb_value, 3);
}



static clocks_static_prediction_snapshot_t prediction_snapshot_for_pps(void) {
  clocks_static_prediction_snapshot_t s{};
  (void)clocks_static_prediction_pps_snapshot(&s);
  return s;
}

static clocks_static_prediction_snapshot_t prediction_snapshot_for_clock(time_clock_id_t clock) {
  clocks_static_prediction_snapshot_t s{};
  (void)clocks_static_prediction_snapshot(clock, &s);
  return s;
}

static void prediction_add_legacy_lane_summary(Payload& prediction,
                                               const char* prefix,
                                               const clocks_static_prediction_snapshot_t& s) {
  char key[96];
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    prediction.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    prediction.add(key, value);
  };
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    prediction.add(key, value);
  };

  add_bool("static_prediction_valid", s.valid);
  add_u32("completed_interval_count", s.completed_interval_count);
  add_u32("static_prediction_cycles", s.static_prediction_cycles);
  add_u32("actual_cycles", s.actual_cycles);
  add_i32("static_residual_cycles", s.static_residual_cycles);
}

static void payload_add_flat_prediction_lane(Payload& p,
                                             const char* prefix,
                                             const clocks_static_prediction_snapshot_t& s) {
  char key[96];
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_bool("prediction_valid", s.valid);
  add_u32("completed_interval_count", s.completed_interval_count);
  add_u32("prediction_cycles", s.static_prediction_cycles);
  add_u32("actual_cycles", s.actual_cycles);
  add_i32("residual_cycles", s.static_residual_cycles);
}

static void payload_add_prediction_summary(Payload& p) {
  const clocks_static_prediction_snapshot_t pps = prediction_snapshot_for_pps();
  const clocks_static_prediction_snapshot_t vclock = prediction_snapshot_for_clock(time_clock_id_t::VCLOCK);
  const clocks_static_prediction_snapshot_t ocxo1 = prediction_snapshot_for_clock(time_clock_id_t::OCXO1);
  const clocks_static_prediction_snapshot_t ocxo2 = prediction_snapshot_for_clock(time_clock_id_t::OCXO2);

  // Canonical flat four-rail prediction surface.  Each rail uses the prior
  // completed one-second DWT interval as its next-second prediction.
  payload_add_flat_prediction_lane(p, "pps",    pps);
  payload_add_flat_prediction_lane(p, "vclock", vclock);
  payload_add_flat_prediction_lane(p, "ocxo1",  ocxo1);
  payload_add_flat_prediction_lane(p, "ocxo2",  ocxo2);

  // Legacy nested prediction object retained during Pi-side report migration.
  Payload prediction;
  prediction_add_legacy_lane_summary(prediction, "pps",    pps);
  prediction_add_legacy_lane_summary(prediction, "vclock", vclock);
  prediction_add_legacy_lane_summary(prediction, "ocxo1",  ocxo1);
  prediction_add_legacy_lane_summary(prediction, "ocxo2",  ocxo2);

  p.add_object("prediction", prediction);
}


static void payload_add_lane_forensics_flat(Payload& p,
                                            const char* prefix,
                                            bool valid,
                                            const clocks_alpha_lane_forensics_t& f) {
  char key[96];

  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  // Compact flat Alpha-forensics fields for TIMEBASE_FRAGMENT.
  // raw_cycles.py needs the event DWT and the previous-event counter delta;
  // the new OCXO service fields make compare-service latency visible in the
  // same row as the static residual without carrying the full lane-report
  // debugging surface in every fragment.
  add_bool("forensics_valid", valid);
  add_u32("forensics_update_count", valid ? f.update_count : 0U);
  add_u32("forensics_last_event_dwt", valid ? f.last_event_dwt : 0U);
  add_u32("alpha_event_last_event_dwt", valid ? f.last_event_dwt : 0U);  // raw_cycles alias
  add_u32("forensics_last_event_counter32", valid ? f.last_event_counter32 : 0U);
  add_u32("forensics_dwt_cycles_between_edges", valid ? f.dwt_cycles_between_edges : 0U);
  add_u32("forensics_counter32_delta_since_previous_event",
          valid ? f.counter32_delta_since_previous_event : 0U);
  add_u32("forensics_zero_offset_counter32", valid ? f.zero_offset_counter32 : 0U);
  add_u32("forensics_counter32_delta_since_zero_offset",
          valid ? f.counter32_delta_since_zero_offset : 0U);

  // OCXO compare-service diagnostics propagated from process_interrupt through
  // Alpha.  These are zero for missing diag.  The signed service offset is the
  // primary correlation surface for OCXO residual excursions:
  //   negative -> pre-target/early service
  //   zero     -> exact target service
  //   positive -> late service after target
  add_u32("forensics_service_class", valid ? f.diag_service_class : 0U);
  add_i32("forensics_service_offset_ticks",
          valid ? f.diag_service_offset_signed_ticks : 0);
  add_u32("forensics_service_offset_abs_ticks",
          valid ? f.diag_service_offset_abs_ticks : 0U);
  add_u32("forensics_interpreted_late_ticks",
          valid ? f.diag_interpreted_late_ticks : 0U);
  add_u32("forensics_early_ticks", valid ? f.diag_early_ticks : 0U);
  add_u32("forensics_target_delta_mod65536_ticks",
          valid ? f.diag_target_delta_mod65536_ticks : 0U);
  add_u32("forensics_arm_remaining_ticks",
          valid ? f.diag_arm_remaining_ticks : 0U);
  add_u32("forensics_arm_to_isr_ticks",
          valid ? f.diag_arm_to_isr_ticks : 0U);
  add_u32("forensics_arm_to_isr_dwt_cycles",
          valid ? f.diag_arm_to_isr_dwt_cycles : 0U);

  // Extreme ISR hygiene diagnostics.  Corrected DWT is diagnostic-only in
  // this pass; canonical OCXO event DWT remains forensics_last_event_dwt.
  add_u32("forensics_perishable_fact_sequence",
          valid ? f.diag_perishable_fact_sequence : 0U);
  add_i32("forensics_service_correction_cycles",
          valid ? f.diag_service_correction_cycles : 0);
  add_u32("forensics_service_corrected_dwt_at_event",
          valid ? f.diag_service_corrected_dwt_at_event : 0U);
  add_u32("forensics_fact_ring_overflow_count",
          valid ? f.diag_fact_ring_overflow_count : 0U);
  add_u32("forensics_counter_delta_violation_count",
          valid ? f.diag_counter_delta_violation_count : 0U);
  add_u32("forensics_last_bad_counter_delta",
          valid ? f.diag_last_bad_counter_delta : 0U);
  add_u32("forensics_last_counter_delta_ticks",
          valid ? f.diag_last_counter_delta_ticks : 0U);

  // Quiet-zone OCXO sample phase / Alpha boundary projection.  raw_cycles can
  // continue to use forensics_last_event_dwt as the Alpha-applied boundary DWT
  // while these fields expose the observed sample and the subtracted phase.
  add_bool("forensics_sample_phase_valid",
           valid && f.diag_sample_phase_valid);
  add_u32("forensics_sample_phase_ticks",
          valid ? f.diag_sample_phase_ticks : 0U);
  add_u32("forensics_sample_phase_us",
          valid ? f.diag_sample_phase_us : 0U);
  add_u32("forensics_sample_phase_ns",
          valid ? f.diag_sample_phase_ns : 0U);
  add_u32("forensics_sample_period_ticks",
          valid ? f.diag_sample_period_ticks : 0U);
  add_u32("forensics_sample_dwt_at_event",
          valid ? f.diag_sample_dwt_at_event : 0U);
  add_u32("forensics_sample_counter32_at_event",
          valid ? f.diag_sample_counter32_at_event : 0U);
  add_u32("forensics_boundary_dwt_at_event",
          valid ? f.diag_boundary_dwt_at_event : 0U);
  add_u32("forensics_boundary_counter32_at_event",
          valid ? f.diag_boundary_counter32_at_event : 0U);
  add_i32("forensics_boundary_correction_cycles",
          valid ? f.diag_boundary_correction_cycles : 0);

  // Linear-regression diagnostics are disabled in the quiet-phase checkpoint.
  // The zero-valued/invalid surface is kept temporarily so existing report
  // readers do not fail while the OCXO custody experiment is evaluated.
  // The subscriber-facing event DWT remains the traditional edge DWT; these
  // fields expose what the regression engine inferred from the 1 kHz window.
  add_bool("forensics_regression_valid",
           valid && f.regression_valid);
  add_u32("forensics_regression_sequence",
          valid ? f.regression_sequence : 0U);
  add_u32("forensics_regression_sample_count",
          valid ? f.regression_sample_count : 0U);
  add_u32("forensics_regression_observed_dwt_at_event",
          valid ? f.regression_observed_dwt_at_event : 0U);
  add_u32("forensics_regression_inferred_dwt_at_event",
          valid ? f.regression_inferred_dwt_at_event : 0U);
  add_i32("forensics_regression_inferred_minus_observed_cycles",
          valid ? f.regression_inferred_minus_observed_cycles : 0);
  add_u32("forensics_regression_target_counter32_at_event",
          valid ? f.regression_target_counter32_at_event : 0U);
  add_u32("forensics_regression_target_hardware16_at_event",
          valid ? (uint32_t)f.regression_target_hardware16_at_event : 0U);
  add_u32("forensics_regression_observed_hardware16_at_event",
          valid ? (uint32_t)f.regression_observed_hardware16_at_event : 0U);
  snprintf(key, sizeof(key), "%s_%s", prefix,
           "forensics_regression_slope_q16_cycles_per_sample");
  p.add(key, valid ? f.regression_slope_q16_cycles_per_sample : 0ULL);
  snprintf(key, sizeof(key), "%s_%s", prefix,
           "forensics_regression_slope_delta_q16_cycles_per_sample");
  p.add(key, valid ? f.regression_slope_delta_q16_cycles_per_sample : 0LL);
  add_i32("forensics_regression_fit_error_mean_q16_cycles",
          valid ? f.regression_fit_error_mean_q16_cycles : 0);
  add_u32("forensics_regression_fit_error_stddev_q16_cycles",
          valid ? f.regression_fit_error_stddev_q16_cycles : 0U);
  add_i32("forensics_regression_fit_error_min_cycles",
          valid ? f.regression_fit_error_min_cycles : 0);
  add_i32("forensics_regression_fit_error_max_cycles",
          valid ? f.regression_fit_error_max_cycles : 0);
  add_u32("forensics_regression_fit_error_gt_plus4_count",
          valid ? f.regression_fit_error_gt_plus4_count : 0U);
  add_u32("forensics_regression_fit_error_lt_minus4_count",
          valid ? f.regression_fit_error_lt_minus4_count : 0U);
  add_u32("forensics_regression_fit_error_abs_gt4_count",
          valid ? f.regression_fit_error_abs_gt4_count : 0U);
}


// ============================================================================
// TIMEBASE_FRAGMENT v2 — hierarchical publication helpers
// ============================================================================
//
// TIMEBASE_FRAGMENT used to publish almost everything as flat key/value pairs.
// That was convenient for ad-hoc scripts, but it burned Payload entry slots and
// arena bytes through repeated prefixes such as
// ocxo1_forensics_regression_fit_error_abs_gt4_count.  The fragment is now
// lane-centric: PPS/DWT/GNSS remain shared sections, prediction stays separate,
// and each clock owns its own measurement/forensics/service/regression objects.
//
// Keep the legacy flat helpers above for focused reports and transition tools;
// TIMEBASE_FRAGMENT itself uses the helpers below.

static void payload_add_welford_object(Payload& parent,
                                       const char* key,
                                       const welford_t& w) {
  Payload obj;
  obj.add("n", w.n);
  obj.add("mean", w.mean, 6);
  obj.add("stddev", welford_stddev(w), 6);
  obj.add("stderr", welford_stderr(w), 6);
  obj.add("min", (w.n > 0) ? w.min_val : 0.0, 6);
  obj.add("max", (w.n > 0) ? w.max_val : 0.0, 6);
  parent.add_object(key, obj);
}

static void payload_add_frequency_fields(Payload& obj, double ppb_value) {
  const double tau_value = 1.0 + ppb_value / 1e9;
  obj.add("tau", tau_value, 12);
  obj.add("ppb", ppb_value, 3);
}

static void payload_add_stats_clock(Payload& parent,
                                    const char* key,
                                    const welford_t& w,
                                    bool include_frequency) {
  Payload obj;
  payload_add_welford_object(obj, "welford", w);
  if (include_frequency) {
    payload_add_frequency_fields(obj, w.mean);
  }
  parent.add_object(key, obj);
}

static void payload_add_stats_summary_hierarchical(Payload& p) {
  Payload stats;
  payload_add_stats_clock(stats, "dwt", welford_dwt, true);
  payload_add_stats_clock(stats, "vclock", welford_vclock, true);
  payload_add_stats_clock(stats, "ocxo1", welford_ocxo1, true);
  payload_add_stats_clock(stats, "ocxo2", welford_ocxo2, true);
  payload_add_stats_clock(stats, "pps_witness", welford_pps_witness, false);

  Payload dac;
  payload_add_welford_object(dac, "ocxo1", welford_ocxo1_dac);
  payload_add_welford_object(dac, "ocxo2", welford_ocxo2_dac);
  stats.add_object("dac", dac);

  p.add_object("stats", stats);
}

static void payload_add_prediction_lane_object(Payload& parent,
                                               const char* key,
                                               const clocks_static_prediction_snapshot_t& s) {
  Payload lane;
  lane.add("valid", s.valid);
  lane.add("completed_interval_count", s.completed_interval_count);
  lane.add("prediction_cycles", s.static_prediction_cycles);
  lane.add("actual_cycles", s.actual_cycles);
  lane.add("residual_cycles", s.static_residual_cycles);
  parent.add_object(key, lane);
}

static void payload_add_prediction_summary_hierarchical(Payload& p) {
  Payload prediction;
  payload_add_prediction_lane_object(prediction, "pps", prediction_snapshot_for_pps());
  payload_add_prediction_lane_object(prediction, "vclock", prediction_snapshot_for_clock(time_clock_id_t::VCLOCK));
  payload_add_prediction_lane_object(prediction, "ocxo1", prediction_snapshot_for_clock(time_clock_id_t::OCXO1));
  payload_add_prediction_lane_object(prediction, "ocxo2", prediction_snapshot_for_clock(time_clock_id_t::OCXO2));
  p.add_object("prediction", prediction);
}

static void payload_add_clock_measurement_object(Payload& parent,
                                                 const clock_state_t& clock,
                                                 const clock_measurement_t& meas) {
  Payload measurement;
  measurement.add("gnss_ns_between_edges", (uint64_t)meas.gnss_ns_between_edges);
  measurement.add("second_residual_ns", (int64_t)meas.second_residual_ns);
  measurement.add("phase_offset_ns", (int64_t)clock.phase_offset_ns);
  measurement.add("zero_established", (bool)clock.zero_established);
  measurement.add("window_checks", (uint32_t)clock.window_checks);
  measurement.add("window_mismatches", (uint32_t)clock.window_mismatches);
  measurement.add("window_error_ns", (int64_t)clock.window_error_ns);
  parent.add_object("measurement", measurement);
}

static void payload_add_ocxo_interval_object(Payload& parent,
                                             bool valid,
                                             const clocks_alpha_lane_forensics_t& f) {
  Payload interval;
  interval.add("counter_nominal_ns_between_edges",
               valid ? f.counter_nominal_ns_between_edges : 0ULL);
  interval.add("bridge_gnss_ns_between_edges",
               valid ? f.bridge_gnss_ns_between_edges : 0ULL);
  interval.add("bridge_residual_ns",
               valid ? f.bridge_residual_ns : 0LL);
  interval.add("bridge_interval_valid",
               valid && f.bridge_interval_valid);
  parent.add_object("interval", interval);
}

static void payload_add_lane_forensics_object(Payload& parent,
                                              bool valid,
                                              const clocks_alpha_lane_forensics_t& f) {
  Payload forensics;
  forensics.add("valid", valid);
  forensics.add("update_count", valid ? f.update_count : 0U);
  forensics.add("last_event_dwt", valid ? f.last_event_dwt : 0U);
  forensics.add("last_event_counter32", valid ? f.last_event_counter32 : 0U);
  forensics.add("dwt_cycles_between_edges", valid ? f.dwt_cycles_between_edges : 0U);
  forensics.add("counter32_delta_since_previous_event",
                valid ? f.counter32_delta_since_previous_event : 0U);
  forensics.add("zero_offset_counter32", valid ? f.zero_offset_counter32 : 0U);
  forensics.add("counter32_delta_since_zero_offset",
                valid ? f.counter32_delta_since_zero_offset : 0U);
  parent.add_object("forensics", forensics);
}

static void payload_add_ocxo_service_object(Payload& parent,
                                            bool valid,
                                            const clocks_alpha_lane_forensics_t& f) {
  Payload service;
  service.add("class", valid ? f.diag_service_class : 0U);
  service.add("offset_ticks", valid ? f.diag_service_offset_signed_ticks : 0);
  service.add("offset_abs_ticks", valid ? f.diag_service_offset_abs_ticks : 0U);
  service.add("late_ticks", valid ? f.diag_interpreted_late_ticks : 0U);
  service.add("early_ticks", valid ? f.diag_early_ticks : 0U);
  service.add("target_delta_mod65536_ticks",
              valid ? f.diag_target_delta_mod65536_ticks : 0U);
  service.add("arm_remaining_ticks", valid ? f.diag_arm_remaining_ticks : 0U);
  service.add("arm_to_isr_ticks", valid ? f.diag_arm_to_isr_ticks : 0U);
  service.add("arm_to_isr_dwt_cycles", valid ? f.diag_arm_to_isr_dwt_cycles : 0U);
  service.add("perishable_fact_sequence",
              valid ? f.diag_perishable_fact_sequence : 0U);
  service.add("correction_cycles", valid ? f.diag_service_correction_cycles : 0);
  service.add("corrected_dwt_at_event",
              valid ? f.diag_service_corrected_dwt_at_event : 0U);
  service.add("fact_ring_overflow_count",
              valid ? f.diag_fact_ring_overflow_count : 0U);
  service.add("counter_delta_violation_count",
              valid ? f.diag_counter_delta_violation_count : 0U);
  service.add("last_bad_counter_delta", valid ? f.diag_last_bad_counter_delta : 0U);
  service.add("last_counter_delta_ticks", valid ? f.diag_last_counter_delta_ticks : 0U);
  service.add("sample_phase_valid", valid && f.diag_sample_phase_valid);
  service.add("sample_phase_ticks", valid ? f.diag_sample_phase_ticks : 0U);
  service.add("sample_phase_us", valid ? f.diag_sample_phase_us : 0U);
  service.add("sample_phase_ns", valid ? f.diag_sample_phase_ns : 0U);
  service.add("sample_period_ticks", valid ? f.diag_sample_period_ticks : 0U);
  service.add("sample_dwt_at_event", valid ? f.diag_sample_dwt_at_event : 0U);
  service.add("sample_counter32_at_event",
              valid ? f.diag_sample_counter32_at_event : 0U);
  service.add("boundary_dwt_at_event", valid ? f.diag_boundary_dwt_at_event : 0U);
  service.add("boundary_counter32_at_event",
              valid ? f.diag_boundary_counter32_at_event : 0U);
  service.add("boundary_correction_cycles",
              valid ? f.diag_boundary_correction_cycles : 0);
  parent.add_object("service", service);
}

static void payload_add_lane_regression_object(Payload& parent,
                                               bool valid,
                                               const clocks_alpha_lane_forensics_t& f) {
  Payload regression;
  regression.add("enabled", false);
  regression.add("valid", false);
  regression.add("sequence", valid ? f.regression_sequence : 0U);
  regression.add("sample_count", valid ? f.regression_sample_count : 0U);
  regression.add("observed_dwt_at_event",
                 valid ? f.regression_observed_dwt_at_event : 0U);
  regression.add("inferred_dwt_at_event",
                 valid ? f.regression_inferred_dwt_at_event : 0U);
  regression.add("inferred_minus_observed_cycles",
                 valid ? f.regression_inferred_minus_observed_cycles : 0);
  regression.add("target_counter32_at_event",
                 valid ? f.regression_target_counter32_at_event : 0U);
  regression.add("target_hardware16_at_event",
                 valid ? (uint32_t)f.regression_target_hardware16_at_event : 0U);
  regression.add("observed_hardware16_at_event",
                 valid ? (uint32_t)f.regression_observed_hardware16_at_event : 0U);
  regression.add("slope_q16_cycles_per_sample",
                 valid ? f.regression_slope_q16_cycles_per_sample : 0ULL);
  regression.add("slope_delta_q16_cycles_per_sample",
                 valid ? f.regression_slope_delta_q16_cycles_per_sample : 0LL);
  regression.add("fit_error_mean_q16_cycles",
                 valid ? f.regression_fit_error_mean_q16_cycles : 0);
  regression.add("fit_error_stddev_q16_cycles",
                 valid ? f.regression_fit_error_stddev_q16_cycles : 0U);
  regression.add("fit_error_min_cycles",
                 valid ? f.regression_fit_error_min_cycles : 0);
  regression.add("fit_error_max_cycles",
                 valid ? f.regression_fit_error_max_cycles : 0);
  regression.add("fit_error_gt_plus4_count",
                 valid ? f.regression_fit_error_gt_plus4_count : 0U);
  regression.add("fit_error_lt_minus4_count",
                 valid ? f.regression_fit_error_lt_minus4_count : 0U);
  regression.add("fit_error_abs_gt4_count",
                 valid ? f.regression_fit_error_abs_gt4_count : 0U);
  parent.add_object("regression", regression);
}

static void payload_add_ocxo_pps_residual_object(Payload& parent,
                                                bool valid,
                                                uint64_t gnss_interval_ns,
                                                uint64_t clock_interval_ns,
                                                int64_t fast_residual_ns) {
  Payload residual;
  residual.add("valid", valid);
  residual.add("gnss_interval_ns", valid ? gnss_interval_ns : 0ULL);
  residual.add("clock_interval_ns", valid ? clock_interval_ns : 0ULL);
  residual.add("fast_residual_ns", valid ? fast_residual_ns : 0LL);
  residual.add("positive_means", "clock_fast");
  parent.add_object("pps_residual", residual);
}

static void payload_add_vclock_fragment(Payload& p,
                                        uint64_t public_gnss_ns,
                                        bool forensics_valid,
                                        const clocks_alpha_lane_forensics_t& f) {
  Payload lane;
  lane.add("ns", public_gnss_ns);
  lane.add("gnss_ns_at_pps_vclock", public_gnss_ns);
  lane.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);
  payload_add_clock_measurement_object(lane, g_vclock_clock, g_vclock_measurement);
  payload_add_lane_forensics_object(lane, forensics_valid, f);
  payload_add_lane_regression_object(lane, forensics_valid, f);
  p.add_object("vclock", lane);
}

static void payload_add_ocxo_fragment(Payload& p,
                                      const char* key,
                                      uint64_t public_ns,
                                      uint64_t public_measured_ns,
                                      const clock_state_t& clock,
                                      const clock_measurement_t& meas,
                                      bool forensics_valid,
                                      const clocks_alpha_lane_forensics_t& f,
                                      bool pps_projection_valid,
                                      const clocks_alpha_ocxo_pps_projection_snapshot_t& pps_projection,
                                      uint64_t public_pps_projected_ns,
                                      bool pps_residual_valid,
                                      uint64_t pps_gnss_interval_ns,
                                      uint64_t pps_clock_interval_ns,
                                      int64_t pps_fast_residual_ns) {
  Payload lane;

  // Step C authority: ocxoN.ns is now the campaign-public OCXO clock value
  // projected to this TIMEBASE row's canonical PPS/VCLOCK edge.  The legacy
  // Alpha measured-GNSS ledger remains visible as a diagnostic side surface.
  lane.add("ns", public_ns);
  lane.add("ns_at_pps_vclock", public_ns);
  lane.add("ns_source", pps_projection_valid ? "PPS_PROJECTED"
                                              : "MEASURED_FALLBACK");
  lane.add("ns_source_id", pps_projection_valid ? 2U : 1U);

  lane.add("measured_gnss_ns", public_measured_ns);
  lane.add("measured_gnss_ns_at_pps_vclock", public_measured_ns);
  lane.add("legacy_measured_gnss_ns", public_measured_ns);
  const int64_t measured_minus_ns =
      (public_measured_ns >= public_ns)
          ? (int64_t)(public_measured_ns - public_ns)
          : -(int64_t)(public_ns - public_measured_ns);
  lane.add("measured_minus_ns", measured_minus_ns);

  // Keep the Step B sidecar during promotion.  When the projection is valid,
  // pps_projected_ns is the same canonical value as ns; if the projection is
  // not yet valid during early startup, ns falls back to the measured ledger
  // and this sidecar records the unavailability.
  const int64_t projected_minus_ns =
      (!pps_projection_valid)
          ? 0LL
          : ((public_pps_projected_ns >= public_ns)
                 ? (int64_t)(public_pps_projected_ns - public_ns)
                 : -(int64_t)(public_ns - public_pps_projected_ns));
  const int64_t projected_minus_measured_ns =
      (!pps_projection_valid)
          ? 0LL
          : ((public_pps_projected_ns >= public_measured_ns)
                 ? (int64_t)(public_pps_projected_ns - public_measured_ns)
                 : -(int64_t)(public_measured_ns - public_pps_projected_ns));
  lane.add("pps_projected_valid", pps_projection_valid);
  lane.add("pps_projected_ns",
           pps_projection_valid ? public_pps_projected_ns : 0ULL);
  lane.add("pps_projected_raw_ns",
           pps_projection_valid
               ? pps_projection.projected_ocxo_ns_at_pps
               : 0ULL);
  lane.add("pps_projected_minus_ns", projected_minus_ns);
  lane.add("pps_projected_minus_measured_ns", projected_minus_measured_ns);
  lane.add("pps_projection_source",
           pps_projection_valid ? pps_projection.source : 0U);

  // Step D canonical residual surface: this is the same PPS-founded
  // row-to-row residual used to feed the OCXO Welfords.  It is deliberately
  // computed from public TIMEBASE clock values, not OCXO event-edge witnesses
  // and not DWT cycle residuals.
  payload_add_ocxo_pps_residual_object(lane,
                                       pps_residual_valid,
                                       pps_gnss_interval_ns,
                                       pps_clock_interval_ns,
                                       pps_fast_residual_ns);

  // Direct interrupt-authored GNSS-at-edge witness. This is the normalized
  // GNSS timestamp of the observed OCXO quiet-zone sample, not Alpha's
  // measured ledger and not a boundary-normalized value. These fields are
  // passive TIMEBASE witnesses only; no residual is computed here.
  const bool sample_available =
      forensics_valid && f.sample_gnss_ns_at_event_available;
  const bool previous_sample_available =
      forensics_valid && f.previous_sample_gnss_ns_at_event_available;
  lane.add("sample_gnss_ns_at_event_available", sample_available);
  lane.add("sample_gnss_ns_at_event",
           sample_available ? f.sample_gnss_ns_at_event : 0ULL);
  lane.add("previous_sample_gnss_ns_at_event_available",
           previous_sample_available);
  lane.add("previous_sample_gnss_ns_at_event",
           previous_sample_available
               ? f.previous_sample_gnss_ns_at_event
               : 0ULL);

  payload_add_clock_measurement_object(lane, clock, meas);
  payload_add_ocxo_interval_object(lane, forensics_valid, f);
  payload_add_lane_forensics_object(lane, forensics_valid, f);
  payload_add_ocxo_service_object(lane, forensics_valid, f);
  payload_add_lane_regression_object(lane, forensics_valid, f);
  p.add_object(key, lane);
}

static void payload_add_dac_summary_hierarchical(Payload& p) {
  Payload dac;
  dac.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  dac.add("ad5693r_init_ok", g_ad5693r_init_ok);

  Payload o1;
  o1.add("value", ocxo1_dac.dac_fractional);
  o1.add("last_write_ok", ocxo1_dac.io_last_write_ok);
  o1.add("fault_latched", ocxo1_dac.io_fault_latched);
  o1.add("servo_predicted_residual_ns", ocxo1_dac.servo_predicted_residual, 6);
  o1.add("servo_last_step", ocxo1_dac.servo_last_step, 6);
  o1.add("servo_adjustments", ocxo1_dac.servo_adjustments);
  dac.add_object("ocxo1", o1);

  Payload o2;
  o2.add("value", ocxo2_dac.dac_fractional);
  o2.add("last_write_ok", ocxo2_dac.io_last_write_ok);
  o2.add("fault_latched", ocxo2_dac.io_fault_latched);
  o2.add("servo_predicted_residual_ns", ocxo2_dac.servo_predicted_residual, 6);
  o2.add("servo_last_step", ocxo2_dac.servo_last_step, 6);
  o2.add("servo_adjustments", ocxo2_dac.servo_adjustments);
  dac.add_object("ocxo2", o2);

  p.add_object("dac", dac);
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
  // Beta-local accounting reset only.  Alpha owns the active time/epoch
  // projection. Do not invalidate it here: after SmartZero install the new
  // epoch has just been authored; during acquisition the old service epoch
  // must remain alive.
  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_measured_gnss_ticks_64        = 0;
  ocxo2_measured_gnss_ticks_64        = 0;

  campaign_public_bases_reset_to_current();

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
  welford_reset(welford_pps_witness);
  welford_reset(welford_ocxo1_dac);
  welford_reset(welford_ocxo2_dac);

  pps_interval_residuals_reset();

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
// Local ZERO / START completion helpers
// ============================================================================

static void clocks_finish_zero_accounting(void) {
  clocks_zero_all();
  request_zero = false;
}

static void clocks_finish_start_accounting(void) {
  clocks_zero_all();
  request_zero = false;
  request_start = false;
  watchdog_anomaly_active = false;
  campaign_state = clocks_campaign_state_t::STARTED;
  campaign_warmup_begin(campaign_warmup_mode_t::START);
}


static bool clocks_try_finish_pending_smartzero(void) {
  if (!request_start && !request_zero) return false;
  if (!interrupt_smartzero_complete()) return false;

  const bool finishing_start = request_start;
  const bool zero_ok = clocks_alpha_zero_from_smartzero(finishing_start ? "start" : "zero");
  if (!zero_ok) {
    request_start = false;
    request_zero = false;
    return false;
  }

  if (finishing_start) {
    clocks_finish_start_accounting();
  } else {
    campaign_state = clocks_campaign_state_t::STOPPED;
    clocks_finish_zero_accounting();
  }
  return true;
}

// ============================================================================
// clocks_beta_pps — invoked from alpha's pps_selector_callback
// ============================================================================

void clocks_beta_pps(void) {
  if (request_stop) {
    const bool was_started = (campaign_state == clocks_campaign_state_t::STARTED);
    watchdog_anomaly_active = false;
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    calibrate_ocxo_mode = servo_mode_t::OFF;
    ocxo_dac_pacing_abort_all();
    campaign_warmup_reset();
    if (was_started) {
      timebase_invalidate();
    }
    return;
  }

  if (request_start || request_zero) {
    (void)clocks_try_finish_pending_smartzero();
    return;
  }

  if (request_recover) {
    watchdog_anomaly_active = false;
    request_zero = false;
    ocxo_dac_pacing_abort_all();

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_measured_gnss_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_measured_gnss_ticks_64        = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;
    pps_interval_residuals_reset();

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
  ocxo1_measured_gnss_ticks_64        = campaign_public_ocxo1_ns() / 100ull;
  ocxo2_measured_gnss_ticks_64        = campaign_public_ocxo2_ns() / 100ull;

  // ── Welford updates ──
  //
  // DWT sample is fed directly as ppb using the uniform sign convention:
  // positive ppb → Teensy running FAST (measured cycles > expected).
  {
    const double cycles = (double)g_dwt_cycles_between_pps_vclock;
    const double expected = (double)DWT_EXPECTED_PER_PPS;
    const double dwt_ppb_sample = (cycles - expected) / expected * 1e9;
    welford_update(welford_dwt, dwt_ppb_sample);
  }

  clocks_alpha_lane_forensics_t vclock_forensics{};
  clocks_alpha_lane_forensics_t ocxo1_forensics{};
  clocks_alpha_lane_forensics_t ocxo2_forensics{};

  const bool vclock_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK, &vclock_forensics);
  const bool ocxo1_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &ocxo1_forensics);
  const bool ocxo2_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &ocxo2_forensics);

  clocks_alpha_ocxo_pps_projection_snapshot_t ocxo1_pps_projection{};
  clocks_alpha_ocxo_pps_projection_snapshot_t ocxo2_pps_projection{};
  const bool ocxo1_pps_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO1,
                                                &ocxo1_pps_projection);
  const bool ocxo2_pps_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO2,
                                                &ocxo2_pps_projection);

  // ── Public PPS-edge clock tuple ──
  //
  // Step C made public_ocxoN_ns the canonical PPS-projected OCXO clock value.
  // Step D computes OCXO residuals/Welfords from consecutive values in this
  // same public tuple.  No normalization happens here; if these values are
  // not already PPS-founded, that is an upstream architecture problem.
  const uint64_t public_gnss_ns   = campaign_public_gnss_ns();
  const uint64_t public_dwt_total = campaign_public_dwt_total();
  const uint64_t public_dwt_ns    = dwt_cycles_to_ns(public_dwt_total);
  const uint64_t public_ocxo1_measured_ns = campaign_public_ocxo1_measured_ns();
  const uint64_t public_ocxo2_measured_ns = campaign_public_ocxo2_measured_ns();

  const bool ocxo1_pps_projected_raw_valid =
      ocxo1_pps_projection_ok && ocxo1_pps_projection.valid &&
      ocxo1_pps_projection.projected_ocxo_ns_at_pps != 0;
  const bool ocxo2_pps_projected_raw_valid =
      ocxo2_pps_projection_ok && ocxo2_pps_projection.valid &&
      ocxo2_pps_projection.projected_ocxo_ns_at_pps != 0;
  const bool ocxo1_pps_projected_valid =
      ocxo1_pps_projected_raw_valid &&
      ocxo1_pps_projection.projected_ocxo_ns_at_pps >=
          g_campaign_public_ocxo1_base;
  const bool ocxo2_pps_projected_valid =
      ocxo2_pps_projected_raw_valid &&
      ocxo2_pps_projection.projected_ocxo_ns_at_pps >=
          g_campaign_public_ocxo2_base;
  const uint64_t public_ocxo1_pps_projected_ns =
      ocxo1_pps_projected_valid
          ? campaign_public_from_base(
                ocxo1_pps_projection.projected_ocxo_ns_at_pps,
                g_campaign_public_ocxo1_base)
          : 0ULL;
  const uint64_t public_ocxo2_pps_projected_ns =
      ocxo2_pps_projected_valid
          ? campaign_public_from_base(
                ocxo2_pps_projection.projected_ocxo_ns_at_pps,
                g_campaign_public_ocxo2_base)
          : 0ULL;

  const uint64_t public_ocxo1_ns = ocxo1_pps_projected_valid
      ? public_ocxo1_pps_projected_ns
      : public_ocxo1_measured_ns;
  const uint64_t public_ocxo2_ns = ocxo2_pps_projected_valid
      ? public_ocxo2_pps_projected_ns
      : public_ocxo2_measured_ns;

  const uint32_t public_count = (uint32_t)campaign_seconds;
  const pps_interval_residuals_t pps_residuals =
      pps_interval_residuals_update(public_count,
                                    public_gnss_ns,
                                    public_ocxo1_ns,
                                    public_ocxo2_ns,
                                    ocxo1_pps_projected_valid,
                                    ocxo2_pps_projected_valid);

  // VCLOCK measurement Welford — bridge-interpolation self-test.
  // Sample in ns; mean == ppb under the "ns/sec == ppb" identity.
  if (g_vclock_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_vclock, (double)g_vclock_measurement.second_residual_ns);
  }

  // Step D OCXO Welfords use PPS-founded public clock residuals.
  // The residual is computed from consecutive TIMEBASE clock values:
  //
  //   (current_ocxo_ns - previous_ocxo_ns) -
  //   (current_gnss_ns - previous_gnss_ns)
  //
  // Positive residual means the OCXO clock ledger advanced too far during
  // the PPS interval: OCXO running fast.  This intentionally ignores the
  // old edge-domain measured residuals; those remain published as diagnostics.
  if (pps_residuals.ocxo1_valid) {
    welford_update(welford_ocxo1,
                   (double)pps_residuals.ocxo1_fast_residual_ns);
    now_window_push(g_now_window_ocxo1,
                    pps_residuals.ocxo1_fast_residual_ns);
  }
  if (pps_residuals.ocxo2_valid) {
    welford_update(welford_ocxo2,
                   (double)pps_residuals.ocxo2_fast_residual_ns);
    now_window_push(g_now_window_ocxo2,
                    pps_residuals.ocxo2_fast_residual_ns);
  }

  // PPS witness statistics are owned by the witness/PPS_PHASE path.
  // Beta publishes the accumulator but does not update it here.

  // Servo runs AFTER welford updates so it sees this second's data.
  ocxo_calibration_servo();

  // DAC Welfords — track servo effort (the TEMPEST signal).
  welford_update(welford_ocxo1_dac, ocxo1_dac.dac_fractional);
  welford_update(welford_ocxo2_dac, ocxo2_dac.dac_fractional);

  // ── Build TIMEBASE_FRAGMENT ──
  Payload p;
  p.add("schema", "TIMEBASE_FRAGMENT_V2");
  p.add("fragment_version", 2U);

  // Minimal flat compatibility spine.  TIMEBASE stores this fragment as an
  // opaque object, but Pi-side indexing and older tools still need these few
  // top-level identities while reports migrate to the hierarchical schema.
  p.add("campaign", campaign_name);
  p.add("teensy_pps_vclock_count", public_count);
  p.add("teensy_pps_count",        public_count);  // legacy alias
  p.add("pps_count",               public_count);  // legacy alias / DB order key
  p.add("gnss_ns",                 public_gnss_ns);
  p.add("dwt_at_pps_vclock",       (uint32_t)g_dwt_at_pps_vclock);
  p.add("dwt_cycles_between_pps_vclock", (uint32_t)g_dwt_cycles_between_pps_vclock);
  p.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);
  p.add("pps_dwt_at_edge",         (uint32_t)g_pps_dwt_at_edge);
  p.add("pps_vclock_phase_cycles", (int32_t)g_pps_vclock_phase_cycles);

  {
    Payload gnss;
    gnss.add("ns", public_gnss_ns);
    gnss.add("pps_ns", public_gnss_ns);
    gnss.add("pps_vclock_ns", public_gnss_ns);
    gnss.add("vclock_ns", public_gnss_ns);
    gnss.add("ocxo1_ns", public_ocxo1_ns);
    gnss.add("ocxo2_ns", public_ocxo2_ns);
    gnss.add("ocxo1_measured_ns", public_ocxo1_measured_ns);
    gnss.add("ocxo2_measured_ns", public_ocxo2_measured_ns);
    p.add_object("gnss", gnss);
  }

  {
    Payload environmental;
    environmental.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
    environmental.add("ad5693r_init_ok", g_ad5693r_init_ok);
    p.add_object("environmental", environmental);
  }

  {
    Payload dwt;
    dwt.add("cycle_count_total", public_dwt_total);
    dwt.add("ns", public_dwt_ns);
    dwt.add("at_pps_vclock", (uint32_t)g_dwt_at_pps_vclock);
    dwt.add("cycles_between_pps_vclock", (uint32_t)g_dwt_cycles_between_pps_vclock);
    dwt.add("expected_per_pps_vclock", (uint32_t)DWT_EXPECTED_PER_PPS);
    dwt.add("second_residual_cycles",
            (int32_t)((int64_t)g_dwt_cycles_between_pps_vclock -
                      (int64_t)DWT_EXPECTED_PER_PPS));
    p.add_object("dwt", dwt);
  }

  {
    Payload pps;
    pps.add("count", public_count);
    pps.add("dwt_at_edge", (uint32_t)g_pps_dwt_at_edge);
    pps.add("dwt_cycles_between_edges",
            g_pps_dwt_cycles_between_edges_valid
                ? (uint32_t)g_pps_dwt_cycles_between_edges
                : 0U);
    pps.add("dwt_cycles_between_edges_valid",
            (bool)g_pps_dwt_cycles_between_edges_valid);
    pps.add("vclock_phase_cycles", (int32_t)g_pps_vclock_phase_cycles);
    p.add_object("pps", pps);
  }

  // Prediction remains an intentionally separate four-rail courtroom surface.
  // Clock objects below carry live measurement and diagnostics, but not their
  // static-prediction fields.
  payload_add_prediction_summary_hierarchical(p);

  payload_add_vclock_fragment(p,
                              public_gnss_ns,
                              vclock_forensics_valid,
                              vclock_forensics);

  payload_add_ocxo_fragment(p,
                            "ocxo1",
                            public_ocxo1_ns,
                            public_ocxo1_measured_ns,
                            g_ocxo1_clock,
                            g_ocxo1_measurement,
                            ocxo1_forensics_valid,
                            ocxo1_forensics,
                            ocxo1_pps_projected_valid,
                            ocxo1_pps_projection,
                            public_ocxo1_pps_projected_ns,
                            pps_residuals.ocxo1_valid,
                            pps_residuals.gnss_interval_ns,
                            pps_residuals.ocxo1_interval_ns,
                            pps_residuals.ocxo1_fast_residual_ns);

  payload_add_ocxo_fragment(p,
                            "ocxo2",
                            public_ocxo2_ns,
                            public_ocxo2_measured_ns,
                            g_ocxo2_clock,
                            g_ocxo2_measurement,
                            ocxo2_forensics_valid,
                            ocxo2_forensics,
                            ocxo2_pps_projected_valid,
                            ocxo2_pps_projection,
                            public_ocxo2_pps_projected_ns,
                            pps_residuals.ocxo2_valid,
                            pps_residuals.gnss_interval_ns,
                            pps_residuals.ocxo2_interval_ns,
                            pps_residuals.ocxo2_fast_residual_ns);

  payload_add_dac_summary_hierarchical(p);
  payload_add_stats_summary_hierarchical(p);

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
  request_zero = false;
  request_stop = false;
  request_recover = false;
  watchdog_anomaly_active = false;
  campaign_state = clocks_campaign_state_t::STOPPED;
  campaign_warmup_reset();

  // Non-destructive SmartZero begin: preserve the currently installed service
  // epoch while the replacement proof is acquired. The destructive science
  // rebase happens only after clocks_alpha_zero_from_smartzero() succeeds.
  const bool smartzero_started = clocks_alpha_begin_smartzero_epoch("start");
  if (!smartzero_started) {
    request_start = false;
  }

  Payload p;
  p.add("status", !smartzero_started
                      ? "start_rejected_smartzero_start_failed"
                      : ((!dac1_ok || !dac2_ok)
                            ? "start_pending_smartzero_dac_fault"
                            : "start_pending_smartzero"));
  p.add("zero_installed", false);
  p.add("smartzero_required", true);
  p.add("smartzero_started", smartzero_started);
  p.add("service_epoch_preserved",
        clocks_alpha_smartzero_last_begin_preserved_epoch());
  p.add("smartzero_begin_destructive", false);
  p.add("smartzero_begin_reason", clocks_alpha_smartzero_last_begin_reason());
  p.add("smartzero_pending_active", clocks_alpha_smartzero_pending_active());
  p.add("smartzero_pending_reason", clocks_alpha_smartzero_pending_reason());
  p.add("epoch_owner", "CLOCKS_SMARTZERO");
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  payload_add_smartzero_summary(p);
  return p;
}

static Payload cmd_stop(const Payload&) {
  const bool had_live_smartzero = interrupt_smartzero_running();
  const bool had_pending_start = request_start;
  const bool had_pending_zero = request_zero;

  interrupt_smartzero_abort();
  if (had_live_smartzero || had_pending_start || had_pending_zero) {
    clocks_alpha_smartzero_pending_clear();
  }

  request_start = false;
  request_zero = false;
  request_recover = false;

  Payload p;

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    request_stop = true;
    p.add("status", "stop_requested");
    p.add("service_epoch_preserved", false);
    return p;
  }

  // STOP while no campaign is running is a control-plane abort. It must not
  // invalidate the installed epoch or defer a destructive stop branch to PPS.
  request_stop = false;
  watchdog_anomaly_active = false;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  ocxo_dac_pacing_abort_all();

  p.add("status", (had_live_smartzero || had_pending_start || had_pending_zero)
                      ? "smartzero_abort_requested"
                      : "stopped_idle");
  p.add("service_epoch_preserved", true);
  p.add("had_live_smartzero", had_live_smartzero);
  p.add("had_pending_start", had_pending_start);
  p.add("had_pending_zero", had_pending_zero);
  payload_add_smartzero_summary(p);
  return p;
}


static Payload cmd_zero(const Payload&) {
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = true;
  campaign_state = clocks_campaign_state_t::STOPPED;
  campaign_warmup_reset();

  // ZERO acquisition is also non-destructive until a completed SmartZero proof
  // is installed. Service time remains alive while the new proof is sought.
  const bool smartzero_started = clocks_alpha_begin_smartzero_epoch("zero");
  if (!smartzero_started) {
    request_zero = false;
  }

  Payload p;
  p.add("status", smartzero_started
                      ? "zero_pending_smartzero"
                      : "zero_rejected_smartzero_start_failed");
  p.add("epoch_owner", "CLOCKS_SMARTZERO");
  p.add("zero_installed", false);
  p.add("smartzero_required", true);
  p.add("smartzero_started", smartzero_started);
  p.add("service_epoch_preserved",
        clocks_alpha_smartzero_last_begin_preserved_epoch());
  p.add("smartzero_begin_destructive", false);
  p.add("smartzero_begin_reason", clocks_alpha_smartzero_last_begin_reason());
  p.add("smartzero_pending_active", clocks_alpha_smartzero_pending_active());
  p.add("smartzero_pending_reason", clocks_alpha_smartzero_pending_reason());
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_install_count", clocks_alpha_epoch_install_count());
  p.add("epoch_install_failures", clocks_alpha_epoch_install_failures());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  payload_add_smartzero_summary(p);
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

  interrupt_smartzero_abort();
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


// ============================================================================
// CLOCKS report forensics
// ============================================================================

static int64_t signed_delta_u64(uint64_t a, uint64_t b) {
  return (a >= b) ? (int64_t)(a - b) : -(int64_t)(b - a);
}

static uint32_t apparent_cps_from_projection(uint32_t elapsed_cycles,
                                             uint64_t projected_ns) {
  if (projected_ns == 0) return 0;
  const uint64_t cps =
      ((uint64_t)elapsed_cycles * 1000000000ULL + projected_ns / 2ULL) /
      projected_ns;
  return (cps > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)cps;
}

static double ppb_fast_vs_vclock(uint32_t vclock_cycles,
                                 uint32_t lane_cycles) {
  if (vclock_cycles == 0 || lane_cycles == 0) return 0.0;
  return (((double)vclock_cycles - (double)lane_cycles) /
          (double)vclock_cycles) * 1.0e9;
}

static void add_alpha_event_payload(Payload& p,
                                    const clocks_alpha_lane_forensics_t& f,
                                    const clocks_alpha_lane_forensics_t* vclock_ref) {
  p.add("valid", f.valid);
  p.add("update_count", f.update_count);
  p.add("last_event_dwt", f.last_event_dwt);
  p.add("last_event_counter32", f.last_event_counter32);
  p.add("zero_offset_valid", f.zero_offset_valid);
  p.add("zero_offset_counter32", f.zero_offset_counter32);
  p.add("counter32_delta_since_zero_offset", f.counter32_delta_since_zero_offset);
  p.add("counter32_delta_since_previous_event", f.counter32_delta_since_previous_event);
  p.add("logical_ticks64_since_zero", f.logical_ticks64_since_zero);
  p.add("nominal_ns64_since_zero", f.nominal_ns64_since_zero);

  // Legacy aliases retained for consumers that still use the old epoch labels.
  p.add("epoch_counter32", f.epoch_counter32);
  p.add("counter32_delta_since_epoch", f.counter32_delta_since_epoch);
  p.add("nominal_ns_from_counter32_epoch", f.nominal_ns_from_counter32_epoch);
  p.add("event_gnss_ns", f.event_gnss_ns);
  p.add("previous_event_gnss_ns", f.previous_event_gnss_ns);
  p.add("sample_gnss_ns_at_event", f.sample_gnss_ns_at_event);
  p.add("sample_gnss_ns_at_event_available",
        f.sample_gnss_ns_at_event_available);
  p.add("previous_sample_gnss_ns_at_event",
        f.previous_sample_gnss_ns_at_event);
  p.add("previous_sample_gnss_ns_at_event_available",
        f.previous_sample_gnss_ns_at_event_available);
  p.add("phase_offset_ns", f.phase_offset_ns);
  p.add("counter_nominal_ns_between_edges", f.counter_nominal_ns_between_edges);
  p.add("bridge_interval_valid", f.bridge_interval_valid);
  p.add("bridge_gnss_ns_between_edges", f.bridge_gnss_ns_between_edges);
  p.add("bridge_residual_ns", f.bridge_residual_ns);

  if (vclock_ref && vclock_ref->valid && f.valid) {
    p.add("event_delta_vs_vclock_ns",
          signed_delta_u64(f.nominal_ns_from_counter32_epoch,
                           vclock_ref->nominal_ns_from_counter32_epoch));
    p.add("event_dwt_delta_vs_vclock_cycles",
          (int32_t)(f.last_event_dwt - vclock_ref->last_event_dwt));
    p.add("phase_offset_delta_vs_vclock_ns",
          f.phase_offset_ns - vclock_ref->phase_offset_ns);
    p.add("dwt_cycles_between_edges_delta_vs_vclock",
          (int32_t)((int64_t)f.dwt_cycles_between_edges -
                    (int64_t)vclock_ref->dwt_cycles_between_edges));
    p.add("dwt_interval_ppb_fast_vs_vclock",
          ppb_fast_vs_vclock(vclock_ref->dwt_cycles_between_edges,
                             f.dwt_cycles_between_edges),
          3);
    if (f.bridge_interval_valid && vclock_ref->bridge_interval_valid) {
      p.add("bridge_residual_delta_vs_vclock_ns",
            f.bridge_residual_ns - vclock_ref->bridge_residual_ns);
    }
  }

  p.add("ns_between_edges", f.ns_between_edges);
  p.add("dwt_cycles_between_edges", f.dwt_cycles_between_edges);
  p.add("second_residual_ns", f.second_residual_ns);
  p.add("window_error_ns", f.window_error_ns);
  p.add("window_checks", f.window_checks);
  p.add("window_mismatches", f.window_mismatches);

  Payload anchor;
  anchor.add("sequence_used", f.diag_anchor_sequence_used);
  anchor.add("age_slots", f.diag_anchor_age_slots);
  anchor.add("selection_kind", f.diag_anchor_selection_kind);
  anchor.add("dwt_at_edge", f.diag_anchor_dwt_at_edge);
  anchor.add("gnss_ns_at_edge", f.diag_anchor_gnss_ns_at_edge);
  anchor.add("cps", f.diag_anchor_cps);
  anchor.add("ns_delta", f.diag_anchor_ns_delta);
  anchor.add("failure_mask", f.diag_anchor_failure_mask);
  p.add_object("bridge_anchor", anchor);

  Payload service;
  service.add("class", f.diag_service_class);
  service.add("offset_signed_ticks", f.diag_service_offset_signed_ticks);
  service.add("offset_abs_ticks", f.diag_service_offset_abs_ticks);
  service.add("interpreted_late_ticks", f.diag_interpreted_late_ticks);
  service.add("early_ticks", f.diag_early_ticks);
  service.add("target_delta_mod65536_ticks",
              f.diag_target_delta_mod65536_ticks);
  service.add("arm_remaining_ticks", f.diag_arm_remaining_ticks);
  service.add("arm_to_isr_ticks", f.diag_arm_to_isr_ticks);
  service.add("arm_to_isr_dwt_cycles", f.diag_arm_to_isr_dwt_cycles);
  service.add("perishable_fact_sequence",
              f.diag_perishable_fact_sequence);
  service.add("correction_cycles", f.diag_service_correction_cycles);
  service.add("corrected_dwt_at_event",
              f.diag_service_corrected_dwt_at_event);
  service.add("fact_ring_overflow_count",
              f.diag_fact_ring_overflow_count);
  service.add("counter_delta_violation_count",
              f.diag_counter_delta_violation_count);
  service.add("last_bad_counter_delta", f.diag_last_bad_counter_delta);
  service.add("last_counter_delta_ticks", f.diag_last_counter_delta_ticks);
  service.add("sample_phase_valid", f.diag_sample_phase_valid);
  service.add("sample_phase_ticks", f.diag_sample_phase_ticks);
  service.add("sample_phase_us", f.diag_sample_phase_us);
  service.add("sample_phase_ns", f.diag_sample_phase_ns);
  service.add("sample_period_ticks", f.diag_sample_period_ticks);
  service.add("sample_dwt_at_event", f.diag_sample_dwt_at_event);
  service.add("sample_counter32_at_event", f.diag_sample_counter32_at_event);
  service.add("boundary_dwt_at_event", f.diag_boundary_dwt_at_event);
  service.add("boundary_counter32_at_event", f.diag_boundary_counter32_at_event);
  service.add("boundary_correction_cycles", f.diag_boundary_correction_cycles);
  p.add_object("ocxo_service", service);

  Payload regression;
  regression.add("enabled", false);
  regression.add("valid", false);
  regression.add("sequence", f.regression_sequence);
  regression.add("sample_count", f.regression_sample_count);
  regression.add("observed_dwt_at_event",
                 f.regression_observed_dwt_at_event);
  regression.add("inferred_dwt_at_event",
                 f.regression_inferred_dwt_at_event);
  regression.add("inferred_minus_observed_cycles",
                 f.regression_inferred_minus_observed_cycles);
  regression.add("target_counter32_at_event",
                 f.regression_target_counter32_at_event);
  regression.add("target_hardware16_at_event",
                 (uint32_t)f.regression_target_hardware16_at_event);
  regression.add("observed_hardware16_at_event",
                 (uint32_t)f.regression_observed_hardware16_at_event);
  regression.add("slope_q16_cycles_per_sample",
                 f.regression_slope_q16_cycles_per_sample);
  regression.add("slope_delta_q16_cycles_per_sample",
                 f.regression_slope_delta_q16_cycles_per_sample);
  regression.add("fit_error_mean_q16_cycles",
                 f.regression_fit_error_mean_q16_cycles);
  regression.add("fit_error_stddev_q16_cycles",
                 f.regression_fit_error_stddev_q16_cycles);
  regression.add("fit_error_min_cycles",
                 f.regression_fit_error_min_cycles);
  regression.add("fit_error_max_cycles",
                 f.regression_fit_error_max_cycles);
  regression.add("fit_error_gt_plus4_count",
                 f.regression_fit_error_gt_plus4_count);
  regression.add("fit_error_lt_minus4_count",
                 f.regression_fit_error_lt_minus4_count);
  regression.add("fit_error_abs_gt4_count",
                 f.regression_fit_error_abs_gt4_count);
  p.add_object("regression", regression);
}

static void add_projection_payload(Payload& p,
                                   time_clock_id_t clock_id,
                                   uint32_t report_dwt,
                                   uint64_t report_ns,
                                   bool report_valid,
                                   uint64_t vclock_report_ns,
                                   bool vclock_report_valid,
                                   const clocks_alpha_lane_forensics_t& f,
                                   const clocks_alpha_lane_forensics_t* vclock_ref) {
  p.add("report_valid", report_valid);
  p.add("report_ns", report_valid ? report_ns : 0ULL);

  if (report_valid && vclock_report_valid) {
    p.add("report_delta_vs_vclock_ns", signed_delta_u64(report_ns, vclock_report_ns));
  }

  if (f.valid) {
    const uint32_t elapsed_cycles = report_dwt - f.last_event_dwt;
    uint64_t projected_delta_ns = 0;
    if (report_valid && report_ns >= f.nominal_ns_from_counter32_epoch) {
      projected_delta_ns = report_ns - f.nominal_ns_from_counter32_epoch;
    }

    p.add("elapsed_cycles_from_last_event_to_report", elapsed_cycles);
    p.add("projected_delta_ns_from_last_event_to_report", projected_delta_ns);
    p.add("apparent_projection_cps", apparent_cps_from_projection(elapsed_cycles,
                                                                  projected_delta_ns));
  }

  Payload alpha;
  add_alpha_event_payload(alpha, f, vclock_ref);
  p.add_object("alpha_event", alpha);

  (void)clock_id;
}

static void add_clock_forensics_payload(Payload& p,
                                        uint32_t report_dwt,
                                        uint64_t vclock_ns,
                                        bool vclock_ok,
                                        uint64_t ocxo1_ns,
                                        bool ocxo1_ok,
                                        uint64_t ocxo2_ns,
                                        bool ocxo2_ok) {
  Payload forensic;
  forensic.add("report_dwt", report_dwt);

  clocks_alpha_lane_forensics_t vf{};
  clocks_alpha_lane_forensics_t o1f{};
  clocks_alpha_lane_forensics_t o2f{};
  const bool vf_ok = clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK, &vf);
  const bool o1f_ok = clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &o1f);
  const bool o2f_ok = clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &o2f);

  forensic.add("alpha_vclock_valid", vf_ok);
  forensic.add("alpha_ocxo1_valid", o1f_ok);
  forensic.add("alpha_ocxo2_valid", o2f_ok);

  Payload vclock;
  add_projection_payload(vclock, time_clock_id_t::VCLOCK, report_dwt,
                         vclock_ns, vclock_ok, vclock_ns, vclock_ok,
                         vf, nullptr);
  forensic.add_object("vclock", vclock);

  Payload ocxo1;
  add_projection_payload(ocxo1, time_clock_id_t::OCXO1, report_dwt,
                         ocxo1_ns, ocxo1_ok, vclock_ns, vclock_ok,
                         o1f, &vf);
  forensic.add_object("ocxo1", ocxo1);

  Payload ocxo2;
  add_projection_payload(ocxo2, time_clock_id_t::OCXO2, report_dwt,
                         ocxo2_ns, ocxo2_ok, vclock_ns, vclock_ok,
                         o2f, &vf);
  forensic.add_object("ocxo2", ocxo2);

  if (vclock_ok && ocxo1_ok && ocxo2_ok) {
    Payload deltas;
    deltas.add("ocxo1_minus_vclock_ns", signed_delta_u64(ocxo1_ns, vclock_ns));
    deltas.add("ocxo2_minus_vclock_ns", signed_delta_u64(ocxo2_ns, vclock_ns));
    deltas.add("ocxo2_minus_ocxo1_ns", signed_delta_u64(ocxo2_ns, ocxo1_ns));
    if (vf_ok && o1f_ok) {
      deltas.add("ocxo1_phase_offset_delta_vs_vclock_ns",
                 o1f.phase_offset_ns - vf.phase_offset_ns);
      deltas.add("ocxo1_dwt_interval_ppb_fast_vs_vclock",
                 ppb_fast_vs_vclock(vf.dwt_cycles_between_edges,
                                    o1f.dwt_cycles_between_edges),
                 3);
    }
    if (vf_ok && o2f_ok) {
      deltas.add("ocxo2_phase_offset_delta_vs_vclock_ns",
                 o2f.phase_offset_ns - vf.phase_offset_ns);
      deltas.add("ocxo2_dwt_interval_ppb_fast_vs_vclock",
                 ppb_fast_vs_vclock(vf.dwt_cycles_between_edges,
                                    o2f.dwt_cycles_between_edges),
                 3);
    }
    forensic.add_object("report_deltas", deltas);
  }

  p.add_object("clock_forensics", forensic);
}


static void add_single_clock_forensics_payload(Payload& p,
                                               const char* key,
                                               time_clock_id_t clock_id,
                                               uint32_t report_dwt,
                                               uint64_t vclock_ns,
                                               bool vclock_ok,
                                               uint64_t report_ns,
                                               bool report_ok) {
  Payload lane;
  lane.add("report_dwt", report_dwt);
  lane.add("clock", key ? key : "");

  clocks_alpha_lane_forensics_t vf{};
  clocks_alpha_lane_forensics_t lf{};
  const bool vf_ok = clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK, &vf);
  const bool lf_ok = clocks_alpha_lane_forensics(clock_id, &lf);
  lane.add("alpha_vclock_valid", vf_ok);
  lane.add("alpha_lane_valid", lf_ok);

  add_projection_payload(lane,
                         clock_id,
                         report_dwt,
                         report_ok ? report_ns : 0ULL,
                         report_ok,
                         vclock_ok ? vclock_ns : 0ULL,
                         vclock_ok,
                         lf,
                         (clock_id == time_clock_id_t::VCLOCK) ? nullptr : &vf);

  p.add_object(key ? key : "lane", lane);
}

// ─────────────────────────────────────────────────────────────────────────────
// CLOCKS report command split
// ─────────────────────────────────────────────────────────────────────────────
//
// The former CLOCKS.REPORT tried to carry summary, epoch, SmartZero, full Alpha
// forensics, campaign state, warmup state, and report-time projections in one
// large nested payload.  That report was diagnostically useful but too expensive
// for the Teensy transport/heap budget.  Keep CLOCKS.REPORT small and make each
// heavy surface explicitly opt-in through a focused command.

static void add_summary_payload(Payload& p) {
  Payload summary;
  const uint32_t report_dwt = DWT_CYCCNT;
  const uint64_t dwt64_cycles_at_report = clocks_dwt_cycles_at_dwt(report_dwt);
  const bool vclock_ok = clock_projection_valid(time_clock_id_t::VCLOCK);
  const bool ocxo1_ok  = clock_projection_valid(time_clock_id_t::OCXO1);
  const bool ocxo2_ok  = clock_projection_valid(time_clock_id_t::OCXO2);
  const uint64_t vclock_ns = vclock_ok ? time_vclock_ns_at_dwt(report_dwt) : 0ULL;
  const uint64_t ocxo1_ns  = ocxo1_ok  ? time_ocxo1_ns_at_dwt(report_dwt)  : 0ULL;
  const uint64_t ocxo2_ns  = ocxo2_ok  ? time_ocxo2_ns_at_dwt(report_dwt)  : 0ULL;

  summary.add("report_dwt", report_dwt);
  summary.add("dwt64_cycles", dwt64_cycles_at_report);
  summary.add("dwt64_ns", dwt_cycles_to_ns(dwt64_cycles_at_report));
  summary.add("gnss_ns", vclock_ok ? vclock_ns : 0ULL);
  summary.add("vclock_ns", vclock_ok ? vclock_ns : 0ULL);
  summary.add("ocxo1_measured_gnss_ns", ocxo1_ok ? ocxo1_ns : 0ULL);
  summary.add("ocxo2_measured_gnss_ns", ocxo2_ok ? ocxo2_ns : 0ULL);
  summary.add("ocxo1_ns", ocxo1_ok ? ocxo1_ns : 0ULL);  // legacy alias
  summary.add("ocxo2_ns", ocxo2_ok ? ocxo2_ns : 0ULL);  // legacy alias
  summary.add("vclock_valid", vclock_ok);
  summary.add("ocxo1_valid", ocxo1_ok);
  summary.add("ocxo2_valid", ocxo2_ok);
  p.add_object("summary", summary);
}

static void add_campaign_payload(Payload& p) {
  p.add("campaign_state",
        campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("request_start", request_start);
  p.add("request_stop", request_stop);
  p.add("request_recover", request_recover);
  p.add("request_zero", request_zero);
  p.add("watchdog_anomaly_active", watchdog_anomaly_active);
  p.add("watchdog_anomaly_publish_pending", watchdog_anomaly_publish_pending);
  p.add("watchdog_anomaly_sequence", watchdog_anomaly_sequence);
  p.add("watchdog_anomaly_reason", watchdog_anomaly_reason);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  p.add("campaign_warmup_active", campaign_warmup_active());
  p.add("campaign_warmup_required", CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS);
  p.add("campaign_warmup_remaining", (uint32_t)g_campaign_warmup_remaining);
  p.add("campaign_warmup_suppressed_total",
        (uint32_t)g_campaign_warmup_suppressed_total);
}

static void add_epoch_payload(Payload& p) {
  p.add("epoch_pending", clocks_epoch_pending());
  p.add("epoch_owner", "CLOCKS_SMARTZERO");
  p.add("epoch_source", "SMARTZERO");
  p.add("epoch_initialized", clocks_alpha_epoch_initialized());
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_install_count", clocks_alpha_epoch_install_count());
  p.add("epoch_install_failures", clocks_alpha_epoch_install_failures());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  p.add("epoch_dwt_at_edge", clocks_alpha_epoch_last_dwt_at_edge());
  p.add("epoch_vclock_counter32", clocks_alpha_epoch_last_vclock_counter32());
  p.add("epoch_ocxo1_counter32", clocks_alpha_epoch_last_ocxo1_counter32());
  p.add("epoch_ocxo2_counter32", clocks_alpha_epoch_last_ocxo2_counter32());
  p.add("zero_offset_vclock_valid", clocks_alpha_epoch_last_vclock_zero_valid());
  p.add("zero_offset_ocxo1_valid", clocks_alpha_epoch_last_ocxo1_zero_valid());
  p.add("zero_offset_ocxo2_valid", clocks_alpha_epoch_last_ocxo2_zero_valid());
  p.add("zero_offset_vclock_counter32", clocks_alpha_epoch_last_vclock_counter32());
  p.add("zero_offset_ocxo1_counter32", clocks_alpha_epoch_last_ocxo1_counter32());
  p.add("zero_offset_ocxo2_counter32", clocks_alpha_epoch_last_ocxo2_counter32());
  p.add("zero_offset_vclock_hardware16_observed",
        (uint32_t)clocks_alpha_epoch_last_vclock_hardware16_observed());
  p.add("zero_offset_vclock_hardware16_selected",
        (uint32_t)clocks_alpha_epoch_last_vclock_hardware16_selected());
  p.add("zero_offset_ocxo1_hardware16",
        (uint32_t)clocks_alpha_epoch_last_ocxo1_hardware16());
  p.add("zero_offset_ocxo2_hardware16",
        (uint32_t)clocks_alpha_epoch_last_ocxo2_hardware16());
  p.add("epoch_capture_sequence", clocks_alpha_epoch_last_capture_sequence());
  p.add("epoch_capture_window_cycles", clocks_alpha_epoch_last_capture_window_cycles());
  p.add("epoch_capture_vclock_valid", clocks_alpha_epoch_last_vclock_capture_valid());
  p.add("epoch_capture_all_lanes_valid", clocks_alpha_epoch_last_all_lanes_valid());
  p.add("alpha_smartzero_begin_count", clocks_alpha_smartzero_begin_count());
  p.add("alpha_smartzero_begin_failures", clocks_alpha_smartzero_begin_failures());
  p.add("alpha_smartzero_begin_preserved_epoch_count",
        clocks_alpha_smartzero_begin_preserved_epoch_count());
  p.add("alpha_smartzero_begin_cold_count",
        clocks_alpha_smartzero_begin_cold_count());
  p.add("alpha_smartzero_last_begin_preserved_epoch",
        clocks_alpha_smartzero_last_begin_preserved_epoch());
  p.add("alpha_smartzero_last_begin_preserved_epoch_sequence",
        clocks_alpha_smartzero_last_begin_preserved_epoch_sequence());
  p.add("alpha_smartzero_last_begin_reason",
        clocks_alpha_smartzero_last_begin_reason());
  p.add("smartzero_pending_active", clocks_alpha_smartzero_pending_active());
  p.add("smartzero_pending_reason", clocks_alpha_smartzero_pending_reason());
  payload_add_smartzero_install_transaction(p);

  interrupt_smartzero_snapshot_t installed{};
  const bool installed_valid = clocks_alpha_epoch_last_smartzero(&installed);
  p.add("installed_smartzero_valid", installed_valid);
  p.add("installed_smartzero_backing_epoch",
        clocks_alpha_installed_smartzero_backing_epoch());
  p.add("installed_smartzero_sequence", installed.sequence);
  p.add("installed_smartzero_vclock_anchor_dwt",
        installed_valid ? installed.lanes[0].anchor_dwt : 0U);
  p.add("installed_smartzero_ocxo1_anchor_dwt",
        installed_valid ? installed.lanes[1].anchor_dwt : 0U);
  p.add("installed_smartzero_ocxo2_anchor_dwt",
        installed_valid ? installed.lanes[2].anchor_dwt : 0U);

  p.add("interrupt_pps_rebootstrap_pending", interrupt_pps_rebootstrap_pending());
}

static void add_compact_smartzero_status(Payload& p) {
  interrupt_smartzero_snapshot_t installed{};
  const bool installed_valid = clocks_alpha_epoch_last_smartzero(&installed);
  const bool installed_backing_epoch = clocks_alpha_installed_smartzero_backing_epoch();

  interrupt_smartzero_snapshot_t live{};
  (void)interrupt_smartzero_live_snapshot(&live);

  p.add("installed_smartzero_valid", installed_valid);
  p.add("installed_smartzero_backing_epoch", installed_backing_epoch);
  p.add("installed_smartzero_sequence", installed.sequence);
  p.add("installed_smartzero_vclock_anchor_dwt",
        installed_valid ? installed.lanes[0].anchor_dwt : 0U);
  p.add("installed_smartzero_ocxo1_anchor_dwt",
        installed_valid ? installed.lanes[1].anchor_dwt : 0U);
  p.add("installed_smartzero_ocxo2_anchor_dwt",
        installed_valid ? installed.lanes[2].anchor_dwt : 0U);

  p.add("live_smartzero_phase", smartzero_phase_name_beta(live.phase));
  p.add("live_smartzero_running", live.running);
  p.add("live_smartzero_complete", live.complete);
  p.add("live_smartzero_aborted", live.aborted);
  p.add("live_smartzero_sequence", live.sequence);
  p.add("live_smartzero_current_lane",
        interrupt_subscriber_kind_str(live.current_lane));
  p.add("live_smartzero_begin_count", live.begin_count);
  p.add("live_smartzero_complete_count", live.complete_count);
  p.add("live_smartzero_abort_count", live.abort_count);

  p.add("smartzero_pending_active", clocks_alpha_smartzero_pending_active());
  p.add("smartzero_pending_reason", clocks_alpha_smartzero_pending_reason());
  payload_add_smartzero_install_transaction(p);
  p.add("smartzero_begin_service_epoch_preserved",
        clocks_alpha_smartzero_last_begin_preserved_epoch());
  p.add("smartzero_begin_preserved_epoch_sequence",
        clocks_alpha_smartzero_last_begin_preserved_epoch_sequence());
  p.add("smartzero_begin_reason", clocks_alpha_smartzero_last_begin_reason());
  p.add("smartzero_begin_preserved_epoch_count",
        clocks_alpha_smartzero_begin_preserved_epoch_count());
  p.add("smartzero_begin_cold_count",
        clocks_alpha_smartzero_begin_cold_count());

  // Legacy aliases: live acquisition only.
  p.add("smartzero_running", live.running);
  p.add("smartzero_complete", live.complete);
  p.add("smartzero_aborted", live.aborted);
  p.add("smartzero_sequence", live.sequence);
  p.add("smartzero_current_lane", interrupt_subscriber_kind_str(live.current_lane));
  p.add("smartzero_begin_count", live.begin_count);
  p.add("smartzero_complete_count", live.complete_count);
  p.add("smartzero_abort_count", live.abort_count);
}

static void add_dac_payload(Payload& p) {
  Payload o1;
  o1.add("dac", ocxo1_dac.dac_fractional);
  o1.add("dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  o1.add("dac_min", ocxo1_dac.dac_min);
  o1.add("dac_max", ocxo1_dac.dac_max);
  o1.add("servo_last_step", ocxo1_dac.servo_last_step, 6);
  o1.add("servo_last_residual", ocxo1_dac.servo_last_residual, 6);
  o1.add("servo_settle_count", ocxo1_dac.servo_settle_count);
  o1.add("servo_adjustments", ocxo1_dac.servo_adjustments);
  o1.add("servo_predictor_initialized", ocxo1_dac.servo_predictor_initialized);
  o1.add("servo_filtered_residual", ocxo1_dac.servo_filtered_residual, 6);
  o1.add("servo_filtered_slope", ocxo1_dac.servo_filtered_slope, 6);
  o1.add("servo_predicted_residual", ocxo1_dac.servo_predicted_residual, 6);
  o1.add("servo_predictor_updates", ocxo1_dac.servo_predictor_updates);
  o1.add("pacing_pending", ocxo1_dac.pacing_pending);
  o1.add("pacing_pending_target", ocxo1_dac.pacing_pending_target, 6);
  o1.add("pacing_pending_step", ocxo1_dac.pacing_pending_step, 6);
  o1.add("pacing_intents", ocxo1_dac.pacing_intents);
  o1.add("pacing_deferred_count", ocxo1_dac.pacing_deferred_count);
  o1.add("pacing_commit_count", ocxo1_dac.pacing_commit_count);
  o1.add("pacing_skip_small_delta_count", ocxo1_dac.pacing_skip_small_delta_count);
  o1.add("io_last_write_ok", ocxo1_dac.io_last_write_ok);
  o1.add("io_fault_latched", ocxo1_dac.io_fault_latched);
  o1.add("io_write_attempts", ocxo1_dac.io_write_attempts);
  o1.add("io_write_successes", ocxo1_dac.io_write_successes);
  o1.add("io_write_failures", ocxo1_dac.io_write_failures);
  o1.add("io_last_attempted_hw_code", (uint32_t)ocxo1_dac.io_last_attempted_hw_code);
  o1.add("io_last_good_hw_code", (uint32_t)ocxo1_dac.io_last_good_hw_code);
  o1.add("io_last_failure_stage", (uint32_t)ocxo1_dac.io_last_failure_stage);
  p.add_object("ocxo1", o1);

  Payload o2;
  o2.add("dac", ocxo2_dac.dac_fractional);
  o2.add("dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  o2.add("dac_min", ocxo2_dac.dac_min);
  o2.add("dac_max", ocxo2_dac.dac_max);
  o2.add("servo_last_step", ocxo2_dac.servo_last_step, 6);
  o2.add("servo_last_residual", ocxo2_dac.servo_last_residual, 6);
  o2.add("servo_settle_count", ocxo2_dac.servo_settle_count);
  o2.add("servo_adjustments", ocxo2_dac.servo_adjustments);
  o2.add("servo_predictor_initialized", ocxo2_dac.servo_predictor_initialized);
  o2.add("servo_filtered_residual", ocxo2_dac.servo_filtered_residual, 6);
  o2.add("servo_filtered_slope", ocxo2_dac.servo_filtered_slope, 6);
  o2.add("servo_predicted_residual", ocxo2_dac.servo_predicted_residual, 6);
  o2.add("servo_predictor_updates", ocxo2_dac.servo_predictor_updates);
  o2.add("pacing_pending", ocxo2_dac.pacing_pending);
  o2.add("pacing_pending_target", ocxo2_dac.pacing_pending_target, 6);
  o2.add("pacing_pending_step", ocxo2_dac.pacing_pending_step, 6);
  o2.add("pacing_intents", ocxo2_dac.pacing_intents);
  o2.add("pacing_deferred_count", ocxo2_dac.pacing_deferred_count);
  o2.add("pacing_commit_count", ocxo2_dac.pacing_commit_count);
  o2.add("pacing_skip_small_delta_count", ocxo2_dac.pacing_skip_small_delta_count);
  o2.add("io_last_write_ok", ocxo2_dac.io_last_write_ok);
  o2.add("io_fault_latched", ocxo2_dac.io_fault_latched);
  o2.add("io_write_attempts", ocxo2_dac.io_write_attempts);
  o2.add("io_write_successes", ocxo2_dac.io_write_successes);
  o2.add("io_write_failures", ocxo2_dac.io_write_failures);
  o2.add("io_last_attempted_hw_code", (uint32_t)ocxo2_dac.io_last_attempted_hw_code);
  o2.add("io_last_good_hw_code", (uint32_t)ocxo2_dac.io_last_good_hw_code);
  o2.add("io_last_failure_stage", (uint32_t)ocxo2_dac.io_last_failure_stage);
  p.add_object("ocxo2", o2);

  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("ad5693r_init_ok", g_ad5693r_init_ok);
  p.add("commit_scheduled", g_ocxo_dac_commit_scheduled);
  p.add("last_schedule_second", g_ocxo_dac_last_schedule_second);
  p.add("last_commit_second", g_ocxo_dac_last_commit_second);
  p.add("last_winner", (uint32_t)g_ocxo_dac_last_winner);
  p.add("arbitration_passes", g_ocxo_dac_arbitration_passes);
  p.add("no_candidate_passes", g_ocxo_dac_no_candidate_passes);
  p.add("deferred_candidates", g_ocxo_dac_deferred_candidates);
  p.add("schedule_failures", g_ocxo_dac_schedule_failures);
}


static const char* alpha_flow_stage_name_beta(uint32_t stage) {
  switch (stage) {
    case 1:  return "CALLBACK_ENTRY";
    case 2:  return "REJECT_EPOCH";
    case 3:  return "CALLBACK_ACCEPTED";
    case 4:  return "APPLY_ENTRY";
    case 5:  return "PHASE_PROJECTED";
    case 6:  return "TICKS64_OK";
    case 7:  return "TICKS64_FAIL";
    case 8:  return "MEASURED_SECOND";
    case 9:  return "TIME_UPDATE";
    case 10: return "STATIC_PREDICTION";
    case 11: return "FORENSICS_PUBLISH";
    case 12: return "COMPLETE";
    case 13: return "FORENSICS_RESET";
    case 14: return "FORENSICS_SNAPSHOT";
    case 15: return "MEASURED_STORE_FAIL";
    default: return "NONE";
  }
}

static void payload_add_alpha_flow_lane(Payload& parent,
                                        const char* key,
                                        time_clock_id_t clock,
                                        bool detailed) {
  clocks_alpha_event_flow_snapshot_t f{};
  const bool ok = clocks_alpha_event_flow_snapshot(clock, &f);

  clocks_alpha_lane_forensics_t lane{};
  const bool lane_ok = clocks_alpha_lane_forensics(clock, &lane);

  Payload p;
  p.add("snapshot_ok", ok);
  p.add("clock_id", ok ? f.clock_id : 0U);

  p.add("forensics_reset_count", ok ? f.forensics_reset_count : 0U);
  p.add("callback_entry_count", ok ? f.callback_entry_count : 0U);
  p.add("callback_diag_present_count", ok ? f.callback_diag_present_count : 0U);
  p.add("callback_diag_missing_count", ok ? f.callback_diag_missing_count : 0U);
  p.add("callback_accepted_count", ok ? f.callback_accepted_count : 0U);
  p.add("callback_rejected_epoch_not_ready_count",
        ok ? f.callback_rejected_epoch_not_ready_count : 0U);

  p.add("apply_entry_count", ok ? f.apply_entry_count : 0U);
  p.add("apply_phase_projected_count", ok ? f.apply_phase_projected_count : 0U);
  p.add("apply_ticks64_success_count", ok ? f.apply_ticks64_success_count : 0U);
  p.add("apply_ticks64_failure_count", ok ? f.apply_ticks64_failure_count : 0U);
  p.add("apply_measured_second_count", ok ? f.apply_measured_second_count : 0U);
  p.add("apply_measured_store_missing_count",
        ok ? f.apply_measured_store_missing_count : 0U);
  p.add("apply_time_update_count", ok ? f.apply_time_update_count : 0U);
  p.add("apply_static_prediction_count", ok ? f.apply_static_prediction_count : 0U);
  p.add("apply_complete_count", ok ? f.apply_complete_count : 0U);

  p.add("forensics_publish_count", ok ? f.forensics_publish_count : 0U);
  p.add("forensics_publish_missing_store_count",
        ok ? f.forensics_publish_missing_store_count : 0U);
  p.add("forensics_snapshot_request_count",
        ok ? f.forensics_snapshot_request_count : 0U);
  p.add("forensics_snapshot_consistent_count",
        ok ? f.forensics_snapshot_consistent_count : 0U);
  p.add("forensics_snapshot_valid_true_count",
        ok ? f.forensics_snapshot_valid_true_count : 0U);
  p.add("forensics_snapshot_valid_false_count",
        ok ? f.forensics_snapshot_valid_false_count : 0U);
  p.add("forensics_snapshot_retry_fail_count",
        ok ? f.forensics_snapshot_retry_fail_count : 0U);
  p.add("forensics_snapshot_missing_store_count",
        ok ? f.forensics_snapshot_missing_store_count : 0U);

  p.add("last_stage", ok ? f.last_stage : 0U);
  p.add("last_stage_name", ok ? alpha_flow_stage_name_beta(f.last_stage) : "NONE");
  p.add("last_failure_stage", ok ? f.last_failure_stage : 0U);
  p.add("last_failure_stage_name",
        ok ? alpha_flow_stage_name_beta(f.last_failure_stage) : "NONE");

  if (!detailed) {
    parent.add_object(key, p);
    return;
  }

  Payload callback;
  callback.add("dwt_at_event", ok ? f.last_callback_dwt_at_event : 0U);
  callback.add("counter32_at_event", ok ? f.last_callback_counter32_at_event : 0U);
  callback.add("gnss_ns_at_event", ok ? f.last_callback_gnss_ns_at_event : 0ULL);
  callback.add("gnss_ns_available", ok && f.last_callback_gnss_ns_available);
  callback.add("diag_present", ok && f.last_callback_diag_present);
  callback.add("diag_anchor_selection_kind",
               ok ? f.last_callback_diag_anchor_selection_kind : 0U);
  callback.add("diag_anchor_failure_mask",
               ok ? f.last_callback_diag_anchor_failure_mask : 0U);
  callback.add("diag_service_class", ok ? f.last_callback_diag_service_class : 0U);
  callback.add("diag_service_offset_ticks",
               ok ? f.last_callback_diag_service_offset_ticks : 0);
  callback.add("diag_perishable_fact_sequence",
               ok ? f.last_callback_diag_perishable_fact_sequence : 0U);
  callback.add("sample_phase_valid", ok && f.last_callback_sample_phase_valid);
  callback.add("sample_phase_ticks", ok ? f.last_callback_sample_phase_ticks : 0U);
  p.add_object("last_callback", callback);

  Payload rejected;
  rejected.add("dwt_at_event", ok ? f.last_rejected_dwt_at_event : 0U);
  rejected.add("counter32_at_event", ok ? f.last_rejected_counter32_at_event : 0U);
  rejected.add("gnss_ns_at_event", ok ? f.last_rejected_gnss_ns_at_event : 0ULL);
  p.add_object("last_rejected", rejected);

  Payload applied;
  applied.add("dwt_at_event", ok ? f.last_applied_dwt_at_event : 0U);
  applied.add("counter32_at_event", ok ? f.last_applied_counter32_at_event : 0U);
  applied.add("phase_ticks", ok ? f.last_applied_phase_ticks : 0U);
  applied.add("phase_cycles", ok ? f.last_applied_phase_cycles : 0U);
  applied.add("dwt_cycles_between_edges",
              ok ? f.last_applied_dwt_cycles_between_edges : 0U);
  applied.add("gnss_ns_between_edges",
              ok ? f.last_applied_gnss_ns_between_edges : 0ULL);
  applied.add("second_residual_ns", ok ? f.last_applied_second_residual_ns : 0LL);
  applied.add("ns_now", ok ? f.last_applied_ns_now : 0ULL);
  applied.add("counter32_delta_since_previous_event",
              ok ? f.last_applied_counter32_delta_since_previous_event : 0U);
  p.add_object("last_applied", applied);

  Payload published;
  published.add("store_valid", ok && f.last_forensics_store_valid);
  published.add("update_count", ok ? f.last_forensics_update_count : 0U);
  published.add("seq", ok ? f.last_forensics_seq : 0U);
  published.add("last_event_dwt", ok ? f.last_forensics_last_event_dwt : 0U);
  published.add("last_event_counter32", ok ? f.last_forensics_last_event_counter32 : 0U);
  published.add("sample_gnss_available", ok && f.last_forensics_sample_gnss_available);
  published.add("sample_gnss_ns_at_event",
                ok ? f.last_forensics_sample_gnss_ns_at_event : 0ULL);
  p.add_object("last_forensics_publish", published);

  Payload snapshot;
  snapshot.add("lane_forensics_return", lane_ok);
  snapshot.add("lane_forensics_valid", lane.valid);
  snapshot.add("lane_forensics_update_count", lane_ok ? lane.update_count : 0U);
  snapshot.add("lane_forensics_sample_available",
               lane_ok && lane.sample_gnss_ns_at_event_available);
  snapshot.add("lane_forensics_sample_gnss_ns_at_event",
               lane_ok ? lane.sample_gnss_ns_at_event : 0ULL);
  snapshot.add("last_return_value", ok && f.last_snapshot_return_value);
  snapshot.add("last_store_valid", ok && f.last_snapshot_store_valid);
  snapshot.add("last_update_count", ok ? f.last_snapshot_update_count : 0U);
  snapshot.add("last_seq", ok ? f.last_snapshot_seq : 0U);
  p.add_object("current_beta_probe", snapshot);

  parent.add_object(key, p);
}

static const char* ocxo_pps_projection_source_name(uint32_t source) {
  switch (source) {
    case 1: return "ACTUAL_BRACKET";
    case 2: return "STATIC_NEXT_EDGE";
    default: return "NONE";
  }
}

static const char* ocxo_pps_projection_invalid_reason_name(uint32_t reason) {
  switch (reason) {
    case 1: return "NO_EDGE";
    case 2: return "NO_INTERVAL";
    case 3: return "TARGET_OUT_OF_WINDOW";
    default: return "NONE";
  }
}

static void payload_add_ocxo_pps_projection_lane(Payload& parent,
                                                 const char* key,
                                                 time_clock_id_t clock) {
  clocks_alpha_ocxo_pps_projection_snapshot_t s{};
  const bool ok = clocks_alpha_ocxo_pps_projection_snapshot(clock, &s);

  Payload lane;
  lane.add("snapshot_ok", ok);
  lane.add("valid", s.valid);
  lane.add("clock_id", s.clock_id);
  lane.add("source", s.source);
  lane.add("source_name", ocxo_pps_projection_source_name(s.source));
  lane.add("last_invalid_reason", s.last_invalid_reason);
  lane.add("last_invalid_reason_name",
           ocxo_pps_projection_invalid_reason_name(s.last_invalid_reason));

  Payload counters;
  counters.add("update_count", s.update_count);
  counters.add("compute_count", s.compute_count);
  counters.add("invalid_no_edge_count", s.invalid_no_edge_count);
  counters.add("invalid_no_interval_count", s.invalid_no_interval_count);
  counters.add("invalid_target_out_of_window_count",
               s.invalid_target_out_of_window_count);
  lane.add_object("counters", counters);

  Payload pps;
  pps.add("sequence", s.pps_sequence);
  pps.add("dwt_at_edge", s.pps_dwt_at_edge);
  pps.add("vclock_ns", s.pps_vclock_ns);
  lane.add_object("pps", pps);

  Payload edge0;
  edge0.add("dwt_at_edge", s.edge0_dwt_at_edge);
  edge0.add("counter32_at_edge", s.edge0_counter32_at_edge);
  edge0.add("ocxo_ns_at_edge", s.edge0_ocxo_ns_at_edge);
  edge0.add("measured_ns_at_edge", s.edge0_measured_ns_at_edge);
  edge0.add("sample_gnss_available", s.edge0_sample_gnss_available);
  edge0.add("sample_gnss_ns_at_event", s.edge0_sample_gnss_ns_at_event);
  edge0.add("boundary_gnss_available", s.edge0_boundary_gnss_available);
  edge0.add("boundary_gnss_ns_at_edge", s.edge0_boundary_gnss_ns_at_edge);
  lane.add_object("edge0", edge0);

  Payload edge1;
  edge1.add("dwt_at_edge", s.edge1_dwt_at_edge);
  edge1.add("counter32_at_edge", s.edge1_counter32_at_edge);
  edge1.add("ocxo_ns_at_edge", s.edge1_ocxo_ns_at_edge);
  edge1.add("measured_ns_at_edge", s.edge1_measured_ns_at_edge);
  edge1.add("sample_gnss_available", s.edge1_sample_gnss_available);
  edge1.add("sample_gnss_ns_at_event", s.edge1_sample_gnss_ns_at_event);
  edge1.add("boundary_gnss_available", s.edge1_boundary_gnss_available);
  edge1.add("boundary_gnss_ns_at_edge", s.edge1_boundary_gnss_ns_at_edge);
  lane.add_object("edge1", edge1);

  Payload projection;
  projection.add("interval_dwt_cycles", s.interval_dwt_cycles);
  projection.add("interval_ocxo_ns", s.interval_ocxo_ns);
  projection.add("target_delta_cycles", s.target_delta_cycles);
  projection.add("target_remaining_cycles", s.target_remaining_cycles);
  projection.add("projected_ocxo_ns_at_pps", s.projected_ocxo_ns_at_pps);
  projection.add("projected_minus_existing_pps_ns",
                 s.projected_minus_existing_pps_ns);
  projection.add("projected_minus_vclock_ns", s.projected_minus_vclock_ns);
  lane.add_object("projection", projection);

  Payload static_prediction;
  static_prediction.add("latest_actual_interval_cycles",
                        s.latest_actual_interval_cycles);
  static_prediction.add("completed_interval_count",
                        s.static_prediction_completed_interval_count);
  static_prediction.add("valid", s.static_prediction_valid);
  lane.add_object("static_prediction", static_prediction);

  parent.add_object(key, lane);
}

static Payload cmd_report_ocxo_pps_projection(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_OCXO_PPS_PROJECTION");
  p.add("description",
        "Report-only Alpha projection of OCXO clock ns to the PPS/VCLOCK DWT edge; not TIMEBASE authority yet");
  payload_add_ocxo_pps_projection_lane(p, "ocxo1", time_clock_id_t::OCXO1);
  payload_add_ocxo_pps_projection_lane(p, "ocxo2", time_clock_id_t::OCXO2);
  return p;
}

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_COMPACT");
  p.add("subreports", "REPORT_STATUS REPORT_SUMMARY REPORT_EPOCH REPORT_SMARTZERO REPORT_INSTALLED_SMARTZERO REPORT_LIVE_SMARTZERO REPORT_FORENSICS REPORT_FORENSICS_VCLOCK REPORT_FORENSICS_OCXO1 REPORT_FORENSICS_OCXO2 REPORT_OCXO_PPS_PROJECTION REPORT_ALPHA_FLOW REPORT_ALPHA_FLOW_VCLOCK REPORT_ALPHA_FLOW_OCXO1 REPORT_ALPHA_FLOW_OCXO2 REPORT_PREDICTION REPORT_STATS REPORT_DAC");
  add_summary_payload(p);
  add_campaign_payload(p);

  // Compact epoch status only.  The full zero-offset / SmartZero proof surface
  // lives in REPORT_EPOCH and REPORT_SMARTZERO.
  p.add("epoch_pending", clocks_epoch_pending());
  p.add("epoch_initialized", clocks_alpha_epoch_initialized());
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_install_count", clocks_alpha_epoch_install_count());
  p.add("epoch_install_failures", clocks_alpha_epoch_install_failures());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  p.add("epoch_source", "SMARTZERO");

  add_compact_smartzero_status(p);
  return p;
}

static Payload cmd_report_status(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_STATUS");
  add_campaign_payload(p);
  add_epoch_payload(p);
  add_compact_smartzero_status(p);
  return p;
}

static Payload cmd_report_summary(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_SUMMARY");
  add_summary_payload(p);
  return p;
}

static Payload cmd_report_epoch(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_EPOCH");
  add_epoch_payload(p);
  return p;
}

static Payload cmd_report_smartzero(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_SMARTZERO");
  payload_add_smartzero_summary(p);
  return p;
}

static Payload cmd_report_installed_smartzero(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_INSTALLED_SMARTZERO");
  interrupt_smartzero_snapshot_t installed{};
  const bool installed_valid = clocks_alpha_epoch_last_smartzero(&installed);
  payload_add_smartzero_snapshot_object(
      p,
      "installed_smartzero",
      installed,
      installed_valid,
      true);
  p.add("installed_smartzero_backing_epoch",
        clocks_alpha_installed_smartzero_backing_epoch());
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  return p;
}

static Payload cmd_report_live_smartzero(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_LIVE_SMARTZERO");
  interrupt_smartzero_snapshot_t live{};
  (void)interrupt_smartzero_live_snapshot(&live);
  payload_add_smartzero_snapshot_object(p, "live_smartzero", live, true, true);
  return p;
}

static Payload cmd_report_forensics(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_FORENSICS");
  const uint32_t report_dwt = DWT_CYCCNT;
  const bool vclock_ok = clock_projection_valid(time_clock_id_t::VCLOCK);
  const bool ocxo1_ok  = clock_projection_valid(time_clock_id_t::OCXO1);
  const bool ocxo2_ok  = clock_projection_valid(time_clock_id_t::OCXO2);
  const uint64_t vclock_ns = vclock_ok ? time_vclock_ns_at_dwt(report_dwt) : 0ULL;
  const uint64_t ocxo1_ns  = ocxo1_ok  ? time_ocxo1_ns_at_dwt(report_dwt)  : 0ULL;
  const uint64_t ocxo2_ns  = ocxo2_ok  ? time_ocxo2_ns_at_dwt(report_dwt)  : 0ULL;
  add_clock_forensics_payload(p, report_dwt,
                              vclock_ok ? vclock_ns : 0ULL, vclock_ok,
                              ocxo1_ok ? ocxo1_ns : 0ULL, ocxo1_ok,
                              ocxo2_ok ? ocxo2_ns : 0ULL, ocxo2_ok);
  return p;
}

static Payload cmd_report_forensics_vclock(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_VCLOCK");
  const uint32_t report_dwt = DWT_CYCCNT;
  const bool vclock_ok = clock_projection_valid(time_clock_id_t::VCLOCK);
  const uint64_t vclock_ns = vclock_ok ? time_vclock_ns_at_dwt(report_dwt) : 0ULL;
  add_single_clock_forensics_payload(p, "vclock", time_clock_id_t::VCLOCK,
                                     report_dwt, vclock_ns, vclock_ok,
                                     vclock_ns, vclock_ok);
  return p;
}

static Payload cmd_report_forensics_ocxo1(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_OCXO1");
  const uint32_t report_dwt = DWT_CYCCNT;
  const bool vclock_ok = clock_projection_valid(time_clock_id_t::VCLOCK);
  const bool ocxo1_ok  = clock_projection_valid(time_clock_id_t::OCXO1);
  const uint64_t vclock_ns = vclock_ok ? time_vclock_ns_at_dwt(report_dwt) : 0ULL;
  const uint64_t ocxo1_ns  = ocxo1_ok  ? time_ocxo1_ns_at_dwt(report_dwt)  : 0ULL;
  add_single_clock_forensics_payload(p, "ocxo1", time_clock_id_t::OCXO1,
                                     report_dwt, vclock_ns, vclock_ok,
                                     ocxo1_ns, ocxo1_ok);
  return p;
}

static Payload cmd_report_forensics_ocxo2(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_OCXO2");
  const uint32_t report_dwt = DWT_CYCCNT;
  const bool vclock_ok = clock_projection_valid(time_clock_id_t::VCLOCK);
  const bool ocxo2_ok  = clock_projection_valid(time_clock_id_t::OCXO2);
  const uint64_t vclock_ns = vclock_ok ? time_vclock_ns_at_dwt(report_dwt) : 0ULL;
  const uint64_t ocxo2_ns  = ocxo2_ok  ? time_ocxo2_ns_at_dwt(report_dwt)  : 0ULL;
  add_single_clock_forensics_payload(p, "ocxo2", time_clock_id_t::OCXO2,
                                     report_dwt, vclock_ns, vclock_ok,
                                     ocxo2_ns, ocxo2_ok);
  return p;
}


static Payload cmd_report_alpha_flow(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW");
  p.add("description", "Alpha subscriber-event flow counters; report-only, not TIMEBASE");
  p.add("detail", "compact");
  p.add("detail_commands", "REPORT_ALPHA_FLOW_VCLOCK REPORT_ALPHA_FLOW_OCXO1 REPORT_ALPHA_FLOW_OCXO2");
  payload_add_alpha_flow_lane(p, "vclock", time_clock_id_t::VCLOCK, false);
  payload_add_alpha_flow_lane(p, "ocxo1", time_clock_id_t::OCXO1, false);
  payload_add_alpha_flow_lane(p, "ocxo2", time_clock_id_t::OCXO2, false);
  return p;
}

static Payload cmd_report_alpha_flow_vclock(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_VCLOCK");
  payload_add_alpha_flow_lane(p, "vclock", time_clock_id_t::VCLOCK, true);
  return p;
}

static Payload cmd_report_alpha_flow_ocxo1(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_OCXO1");
  payload_add_alpha_flow_lane(p, "ocxo1", time_clock_id_t::OCXO1, true);
  return p;
}

static Payload cmd_report_alpha_flow_ocxo2(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_OCXO2");
  payload_add_alpha_flow_lane(p, "ocxo2", time_clock_id_t::OCXO2, true);
  return p;
}

static Payload cmd_report_prediction(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_PREDICTION");
  payload_add_prediction_summary(p);
  return p;
}

static Payload cmd_report_stats(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_STATS");
  publish_welford(p, "dwt",          welford_dwt);
  publish_welford(p, "vclock",       welford_vclock);
  publish_welford(p, "ocxo1",        welford_ocxo1);
  publish_welford(p, "ocxo2",        welford_ocxo2);
  publish_welford(p, "pps_witness",  welford_pps_witness);
  publish_welford(p, "ocxo1_dac",    welford_ocxo1_dac);
  publish_welford(p, "ocxo2_dac",    welford_ocxo2_dac);
  publish_freq(p, "dwt",    welford_dwt.mean);
  publish_freq(p, "vclock", welford_vclock.mean);
  publish_freq(p, "ocxo1",  welford_ocxo1.mean);
  publish_freq(p, "ocxo2",  welford_ocxo2.mean);
  return p;
}

static Payload cmd_report_dac(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_DAC");
  add_dac_payload(p);
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
  { "START",             cmd_start             },
  { "STOP",              cmd_stop              },
  { "ZERO",              cmd_zero              },
  { "RECOVER",           cmd_recover           },
  { "REPORT",            cmd_report            },
  { "REPORT_STATUS",     cmd_report_status     },
  { "REPORT_SUMMARY",    cmd_report_summary    },
  { "REPORT_EPOCH",      cmd_report_epoch      },
  { "REPORT_SMARTZERO",  cmd_report_smartzero  },
  { "REPORT_INSTALLED_SMARTZERO", cmd_report_installed_smartzero },
  { "REPORT_LIVE_SMARTZERO",      cmd_report_live_smartzero      },
  { "REPORT_FORENSICS",        cmd_report_forensics        },
  { "REPORT_FORENSICS_VCLOCK", cmd_report_forensics_vclock },
  { "REPORT_FORENSICS_OCXO1",  cmd_report_forensics_ocxo1  },
  { "REPORT_FORENSICS_OCXO2",  cmd_report_forensics_ocxo2  },
  { "REPORT_OCXO_PPS_PROJECTION", cmd_report_ocxo_pps_projection },
  { "REPORT_ALPHA_FLOW",       cmd_report_alpha_flow       },
  { "REPORT_ALPHA_FLOW_VCLOCK", cmd_report_alpha_flow_vclock },
  { "REPORT_ALPHA_FLOW_OCXO1",  cmd_report_alpha_flow_ocxo1  },
  { "REPORT_ALPHA_FLOW_OCXO2",  cmd_report_alpha_flow_ocxo2  },
  { "REPORT_PREDICTION",       cmd_report_prediction       },
  { "REPORT_STATS",      cmd_report_stats      },
  { "REPORT_DAC",        cmd_report_dac        },
  { "WATCHDOG_TEST",     cmd_watchdog_test     },
  { "SET_DAC",           cmd_set_dac           },
  { nullptr,              nullptr               }
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
