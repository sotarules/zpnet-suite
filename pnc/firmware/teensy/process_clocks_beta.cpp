// ============================================================================
// process_clocks_beta.cpp — Campaign Layer
// ============================================================================
//
// Statistical surface doctrine:
//
//   Teensy owns every statistical quantity published in the TIMEBASE
//   publication pair. The Pi transcribes what the Teensy says — it does not
//   recompute.
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
// Campaign lifecycle, DAC pacing, and watchdog behavior remain unchanged.
// Servo inputs now consume the same PPS-founded OCXO residual surface that
// feeds the OCXO Welfords, and DAC/TIMEBASE reports expose that provenance.
//
// TIMEBASE publication is intentionally split:
//   TIMEBASE_FRAGMENT   — compact canonical campaign row / science spine.
//   TIMEBASE_FORENSICS  — companion diagnostic row for the same PPS identity.
// The Pi pairs the two by pps_count and stores them as { fragment, forensics }.
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

// Most recently published compact TIMEBASE_FRAGMENT, retained for cmd_report.
// TIMEBASE_FORENSICS is deliberately not retained; it is the larger companion
// payload that this split keeps out of steady heap.
static Payload g_last_fragment;

// Alpha-authored physical PPS witness DWT audit surface.  These are published
// into the TIMEBASE publication pair so Pi-side reports can compare physical PPS-to-PPS
// DWT intervals against the canonical PPS/VCLOCK DWT rail.
extern volatile uint32_t g_pps_dwt_at_edge;
extern volatile uint32_t g_pps_dwt_cycles_between_edges;
extern volatile bool     g_pps_dwt_cycles_between_edges_valid;

// ============================================================================
// TIMEBASE publication-tail diagnostics
// ============================================================================
//
// Report-only flight recorder for the PPS -> Alpha -> Beta -> publish tail.
// These counters deliberately do not change TIMEBASE_FRAGMENT shape or publish
// behavior. They make it possible to distinguish whether Beta returned through
// an early gate, reached per-second campaign work, stopped during fragment
// construction, or called publish("TIMEBASE_FRAGMENT", fragment) /
// publish("TIMEBASE_FORENSICS", forensics) and returned.
//
// Focused report: CLOCKS.REPORT_TIMEBASE_PUBLISH

static constexpr uint32_t TIMEBASE_BUILD_STAGE_NONE = 0;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_ENTRY = 1;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_STOP_GATE = 2;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_START_ZERO_GATE = 3;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_RECOVER_GATE = 4;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_WATCHDOG_GATE = 5;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_NOT_STARTED_GATE = 6;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_WARMUP_GATE = 7;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_CANDIDATE = 8;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_PER_SECOND = 9;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_WELFORD = 10;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_SERVO = 11;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_DAC_WELFORD = 12;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_BUILD_BEGIN = 20;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_SPINE = 21;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_GNSS = 22;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_ENVIRONMENTAL = 23;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_DWT = 24;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_PPS = 25;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_PREDICTION = 26;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_VCLOCK = 27;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_OCXO1 = 28;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_OCXO2 = 29;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_DAC = 30;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_STATS = 31;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_BUILD_COMPLETE = 32;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_ASSIGN_LAST_FRAGMENT = 33;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_PUBLISH_ATTEMPT = 34;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_PUBLISH_RETURN = 35;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_BEGIN = 36;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_COMPLETE = 37;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_ATTEMPT = 38;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_RETURN = 39;

static uint32_t g_timebase_pps_entry_count = 0;
static uint32_t g_timebase_stop_gate_count = 0;
static uint32_t g_timebase_start_zero_gate_count = 0;
static uint32_t g_timebase_recover_gate_count = 0;
static uint32_t g_timebase_watchdog_gate_count = 0;
static uint32_t g_timebase_not_started_gate_count = 0;
static uint32_t g_timebase_warmup_suppressed_count = 0;
static uint32_t g_timebase_candidate_count = 0;
static uint32_t g_timebase_per_second_count = 0;
static uint32_t g_timebase_build_begin_count = 0;
static uint32_t g_timebase_build_complete_count = 0;
static uint32_t g_timebase_assign_last_fragment_count = 0;
static uint32_t g_timebase_publish_attempt_count = 0;
static uint32_t g_timebase_publish_return_count = 0;
static uint32_t g_timebase_forensics_build_begin_count = 0;
static uint32_t g_timebase_forensics_build_complete_count = 0;
static uint32_t g_timebase_forensics_publish_attempt_count = 0;
static uint32_t g_timebase_forensics_publish_return_count = 0;

static uint32_t g_timebase_last_stage = TIMEBASE_BUILD_STAGE_NONE;
static uint64_t g_timebase_last_entry_campaign_seconds = 0;
static uint64_t g_timebase_last_candidate_campaign_seconds = 0;
static uint64_t g_timebase_last_per_second_campaign_seconds = 0;
static uint64_t g_timebase_last_build_begin_campaign_seconds = 0;
static uint64_t g_timebase_last_build_complete_campaign_seconds = 0;
static uint64_t g_timebase_last_assign_campaign_seconds = 0;
static uint64_t g_timebase_last_publish_attempt_campaign_seconds = 0;
static uint64_t g_timebase_last_publish_return_campaign_seconds = 0;
static uint64_t g_timebase_last_forensics_build_begin_campaign_seconds = 0;
static uint64_t g_timebase_last_forensics_build_complete_campaign_seconds = 0;
static uint64_t g_timebase_last_forensics_publish_attempt_campaign_seconds = 0;
static uint64_t g_timebase_last_forensics_publish_return_campaign_seconds = 0;
static uint32_t g_timebase_last_public_count = 0;
static uint64_t g_timebase_last_public_gnss_ns = 0;
static uint64_t g_timebase_last_public_dwt_total = 0;
static uint64_t g_timebase_last_public_ocxo1_ns = 0;
static uint64_t g_timebase_last_public_ocxo2_ns = 0;
static bool     g_timebase_last_ocxo1_pps_projected = false;
static bool     g_timebase_last_ocxo2_pps_projected = false;
static bool     g_timebase_last_ocxo1_pps_residual_valid = false;
static bool     g_timebase_last_ocxo2_pps_residual_valid = false;

static FLASHMEM const char* timebase_build_stage_name(uint32_t stage) {
  switch (stage) {
    case TIMEBASE_BUILD_STAGE_ENTRY: return "ENTRY";
    case TIMEBASE_BUILD_STAGE_STOP_GATE: return "STOP_GATE";
    case TIMEBASE_BUILD_STAGE_START_ZERO_GATE: return "START_ZERO_GATE";
    case TIMEBASE_BUILD_STAGE_RECOVER_GATE: return "RECOVER_GATE";
    case TIMEBASE_BUILD_STAGE_WATCHDOG_GATE: return "WATCHDOG_GATE";
    case TIMEBASE_BUILD_STAGE_NOT_STARTED_GATE: return "NOT_STARTED_GATE";
    case TIMEBASE_BUILD_STAGE_WARMUP_GATE: return "WARMUP_GATE";
    case TIMEBASE_BUILD_STAGE_CANDIDATE: return "CANDIDATE";
    case TIMEBASE_BUILD_STAGE_PER_SECOND: return "PER_SECOND";
    case TIMEBASE_BUILD_STAGE_WELFORD: return "WELFORD";
    case TIMEBASE_BUILD_STAGE_SERVO: return "SERVO";
    case TIMEBASE_BUILD_STAGE_DAC_WELFORD: return "DAC_WELFORD";
    case TIMEBASE_BUILD_STAGE_BUILD_BEGIN: return "BUILD_BEGIN";
    case TIMEBASE_BUILD_STAGE_SPINE: return "SPINE";
    case TIMEBASE_BUILD_STAGE_GNSS: return "GNSS";
    case TIMEBASE_BUILD_STAGE_ENVIRONMENTAL: return "ENVIRONMENTAL";
    case TIMEBASE_BUILD_STAGE_DWT: return "DWT";
    case TIMEBASE_BUILD_STAGE_PPS: return "PPS";
    case TIMEBASE_BUILD_STAGE_PREDICTION: return "PREDICTION";
    case TIMEBASE_BUILD_STAGE_VCLOCK: return "VCLOCK";
    case TIMEBASE_BUILD_STAGE_OCXO1: return "OCXO1";
    case TIMEBASE_BUILD_STAGE_OCXO2: return "OCXO2";
    case TIMEBASE_BUILD_STAGE_DAC: return "DAC";
    case TIMEBASE_BUILD_STAGE_STATS: return "STATS";
    case TIMEBASE_BUILD_STAGE_BUILD_COMPLETE: return "BUILD_COMPLETE";
    case TIMEBASE_BUILD_STAGE_ASSIGN_LAST_FRAGMENT: return "ASSIGN_LAST_FRAGMENT";
    case TIMEBASE_BUILD_STAGE_PUBLISH_ATTEMPT: return "PUBLISH_ATTEMPT";
    case TIMEBASE_BUILD_STAGE_PUBLISH_RETURN: return "PUBLISH_RETURN";
    case TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_BEGIN: return "FORENSICS_BUILD_BEGIN";
    case TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_COMPLETE: return "FORENSICS_BUILD_COMPLETE";
    case TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_ATTEMPT: return "FORENSICS_PUBLISH_ATTEMPT";
    case TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_RETURN: return "FORENSICS_PUBLISH_RETURN";
    default: return "NONE";
  }
}

static void timebase_build_stage(uint32_t stage) {
  g_timebase_last_stage = stage;
}

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
static FLASHMEM void publish_welford(Payload& p, const char* prefix, const welford_t& w) {
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

static FLASHMEM const char* smartzero_lane_state_name_beta(interrupt_smartzero_lane_state_t s) {
  switch (s) {
    case interrupt_smartzero_lane_state_t::ACQUIRING: return "ACQUIRING";
    case interrupt_smartzero_lane_state_t::LOCKED:    return "LOCKED";
    default:                                         return "IDLE";
  }
}

static FLASHMEM const char* smartzero_phase_name_beta(interrupt_smartzero_phase_t p) {
  switch (p) {
    case interrupt_smartzero_phase_t::RUNNING:  return "RUNNING";
    case interrupt_smartzero_phase_t::COMPLETE: return "COMPLETE";
    case interrupt_smartzero_phase_t::ABORTED:  return "ABORTED";
    default:                                    return "IDLE";
  }
}

static FLASHMEM const char* smartzero_decision_name_beta(interrupt_smartzero_decision_t d) {
  switch (d) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:  return "WAITING_FOR_CPS";
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:     return "FIRST_SAMPLE";
    case interrupt_smartzero_decision_t::ACCEPTED:         return "ACCEPTED";
    case interrupt_smartzero_decision_t::REJECTED_DWT:     return "REJECTED_DWT";
    case interrupt_smartzero_decision_t::REJECTED_COUNTER: return "REJECTED_COUNTER";
    default:                                               return "NONE";
  }
}

static FLASHMEM void payload_add_smartzero_lane(Payload& parent,
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

static FLASHMEM void payload_add_smartzero_snapshot_object(
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

static FLASHMEM void payload_add_prefixed_smartzero_compact(
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

static FLASHMEM void payload_add_smartzero_install_transaction(Payload& p) {
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

static FLASHMEM void payload_add_smartzero_summary(Payload& p) {
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

static FLASHMEM void publish_freq(Payload& p, const char* clock_name, double ppb_value) {
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

static FLASHMEM void prediction_add_legacy_lane_summary(Payload& prediction,
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

static FLASHMEM void payload_add_flat_prediction_lane(Payload& p,
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

static FLASHMEM void payload_add_prediction_summary(Payload& p) {
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


static FLASHMEM void payload_add_lane_forensics_flat(Payload& p,
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
  add_bool("forensics_dwt_synthetic", valid && f.dwt_synthetic);
  add_bool("forensics_dwt_repair_candidate", valid && f.dwt_repair_candidate);
  add_u32("forensics_dwt_original_at_event", valid ? f.dwt_original_at_event : 0U);
  add_u32("forensics_dwt_predicted_at_event", valid ? f.dwt_predicted_at_event : 0U);
  add_u32("forensics_dwt_used_at_event", valid ? f.dwt_used_at_event : 0U);
  add_u32("forensics_dwt_isr_entry_raw", valid ? f.dwt_isr_entry_raw : 0U);
  add_i32("forensics_dwt_synthetic_error_cycles",
          valid ? f.dwt_synthetic_error_cycles : 0);
  add_u32("forensics_dwt_synthetic_threshold_cycles",
          valid ? f.dwt_synthetic_threshold_cycles : 0U);
  add_bool("forensics_dwt_interval_gate_valid",
           valid && f.dwt_interval_gate_valid);
  add_bool("forensics_dwt_interval_sample_accepted",
           valid && f.dwt_interval_sample_accepted);
  add_bool("forensics_dwt_interval_sample_rejected",
           valid && f.dwt_interval_sample_rejected);
  add_bool("forensics_dwt_interval_ema_updated",
           valid && f.dwt_interval_ema_updated);
  add_u32("forensics_dwt_interval_observed_cycles",
          valid ? f.dwt_interval_observed_cycles : 0U);
  add_u32("forensics_dwt_interval_prediction_cycles",
          valid ? f.dwt_interval_prediction_cycles : 0U);
  add_u32("forensics_dwt_interval_effective_cycles",
          valid ? f.dwt_interval_effective_cycles : 0U);
  add_i32("forensics_dwt_interval_residual_cycles",
          valid ? f.dwt_interval_residual_cycles : 0);
  add_u32("forensics_dwt_interval_gate_threshold_cycles",
          valid ? f.dwt_interval_gate_threshold_cycles : 0U);
  add_u32("forensics_dwt_interval_accept_count",
          valid ? f.dwt_interval_accept_count : 0U);
  add_u32("forensics_dwt_interval_reject_count",
          valid ? f.dwt_interval_reject_count : 0U);
  add_bool("forensics_dwt_interval_resync_applied",
           valid && f.dwt_interval_resync_applied);
  add_u32("forensics_dwt_interval_resync_count",
          valid ? f.dwt_interval_resync_count : 0U);
  add_u32("forensics_dwt_interval_reject_streak",
          valid ? f.dwt_interval_reject_streak : 0U);

  // Counter-adjacency trap propagated from process_interrupt.  This is the
  // durable proof raw_cycles needs to decide whether a synthetic DWT endpoint
  // came from a normal EMA interval gate or from a broken one-second lineage.
  add_bool("forensics_dwt_interval_adjacency_gate_valid",
           valid && f.dwt_interval_adjacency_gate_valid);
  add_bool("forensics_dwt_interval_adjacency_ok",
           valid && f.dwt_interval_adjacency_ok);
  add_bool("forensics_dwt_interval_adjacency_rejected",
           valid && f.dwt_interval_adjacency_rejected);
  add_u32("forensics_dwt_interval_counter_delta_ticks",
          valid ? f.dwt_interval_counter_delta_ticks : 0U);
  add_u32("forensics_dwt_interval_expected_counter_delta_ticks",
          valid ? f.dwt_interval_expected_counter_delta_ticks : 0U);
  add_u32("forensics_dwt_interval_adjacency_reject_count",
          valid ? f.dwt_interval_adjacency_reject_count : 0U);

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

  // SpinIdle / SpinCatch ISR-entry witness diagnostics.  These are
  // subscriber-carried process_interrupt facts: the idle shadow DWT sampled
  // at ISR entry, the cycle distance from shadow to ISR-entry DWT, and the
  // validity decision under the configured threshold.
  add_bool("forensics_spincatch_shadow_valid",
           valid && f.spinidle_shadow_valid);
  add_u32("forensics_spincatch_shadow_dwt",
          valid ? f.spinidle_shadow_dwt : 0U);
  add_u32("forensics_spincatch_shadow_to_isr_entry_cycles",
          valid ? f.spinidle_shadow_to_isr_entry_cycles : 0U);
  add_u32("forensics_spincatch_shadow_valid_threshold_cycles",
          valid ? f.spinidle_shadow_valid_threshold_cycles : 0U);

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
// TIMEBASE publication pair — hierarchical helpers
// ============================================================================
//
// TIMEBASE_FRAGMENT is now the compact canonical campaign row / science spine.
// TIMEBASE_FORENSICS is the larger diagnostic companion for the same pps_count.
// Pi-side clocks pairs the two opaque payloads by identity and persists one
// TIMEBASE record with { fragment, forensics }.
//
// Keep the legacy flat helpers above for focused reports and transition tools;
// the publication pair itself uses the helpers below.

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

static FLASHMEM void payload_add_clock_measurement_object(Payload& parent,
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

static FLASHMEM void payload_add_ocxo_interval_object(Payload& parent,
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

static FLASHMEM void payload_add_spincatch_object(Payload& parent,
                                         bool valid,
                                         const clocks_alpha_lane_forensics_t& f) {
  Payload spincatch;
  spincatch.add("valid", valid && f.spinidle_shadow_valid);
  spincatch.add("shadow_dwt", valid ? f.spinidle_shadow_dwt : 0U);
  spincatch.add("shadow_to_isr_entry_cycles",
                 valid ? f.spinidle_shadow_to_isr_entry_cycles : 0U);
  spincatch.add("shadow_valid_threshold_cycles",
                 valid ? f.spinidle_shadow_valid_threshold_cycles : 0U);
  spincatch.add("source", "SPINIDLE_ISR_WITNESS");
  parent.add_object("spincatch", spincatch);
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

  // Raw/effective DWT authority proof.  This is the durable per-row evidence
  // needed by raw_cycles-style reports after the 500-cycle catastrophic gate.
  forensics.add("dwt_synthetic", valid && f.dwt_synthetic);
  forensics.add("dwt_original_at_event", valid ? f.dwt_original_at_event : 0U);
  forensics.add("dwt_predicted_at_event", valid ? f.dwt_predicted_at_event : 0U);
  forensics.add("dwt_used_at_event", valid ? f.dwt_used_at_event : 0U);
  forensics.add("dwt_isr_entry_raw", valid ? f.dwt_isr_entry_raw : 0U);
  forensics.add("dwt_synthetic_error_cycles",
                valid ? f.dwt_synthetic_error_cycles : 0);
  forensics.add("dwt_synthetic_threshold_cycles",
                valid ? f.dwt_synthetic_threshold_cycles : 0U);

  Payload gate;
  gate.add("valid", valid && f.dwt_interval_gate_valid);
  gate.add("accepted", valid && f.dwt_interval_sample_accepted);
  gate.add("rejected", valid && f.dwt_interval_sample_rejected);
  gate.add("observed_cycles", valid ? f.dwt_interval_observed_cycles : 0U);
  gate.add("prediction_cycles", valid ? f.dwt_interval_prediction_cycles : 0U);
  gate.add("effective_cycles", valid ? f.dwt_interval_effective_cycles : 0U);
  gate.add("residual_cycles", valid ? f.dwt_interval_residual_cycles : 0);
  gate.add("threshold_cycles", valid ? f.dwt_interval_gate_threshold_cycles : 0U);
  gate.add("accept_count", valid ? f.dwt_interval_accept_count : 0U);
  gate.add("reject_count", valid ? f.dwt_interval_reject_count : 0U);
  gate.add("resync_applied", valid && f.dwt_interval_resync_applied);
  gate.add("resync_count", valid ? f.dwt_interval_resync_count : 0U);
  gate.add("reject_streak", valid ? f.dwt_interval_reject_streak : 0U);
  forensics.add_object("dwt_interval_gate", gate);

  Payload adjacency;
  adjacency.add("valid", valid && f.dwt_interval_adjacency_gate_valid);
  adjacency.add("ok", valid && f.dwt_interval_adjacency_ok);
  adjacency.add("rejected", valid && f.dwt_interval_adjacency_rejected);
  adjacency.add("counter_delta_ticks",
                valid ? f.dwt_interval_counter_delta_ticks : 0U);
  adjacency.add("expected_counter_delta_ticks",
                valid ? f.dwt_interval_expected_counter_delta_ticks : 0U);
  adjacency.add("reject_count",
                valid ? f.dwt_interval_adjacency_reject_count : 0U);
  forensics.add_object("dwt_interval_adjacency", adjacency);

  parent.add_object("forensics", forensics);
}

static void payload_add_ocxo_service_object(Payload& parent,
                                            bool valid,
                                            const clocks_alpha_lane_forensics_t& f) {
  Payload service;

  // Tiny service/counter ladder proof only.  Full OCXO compare-service timing
  // remains available through focused interrupt/CLOCKS reports.
  service.add("class", valid ? f.diag_service_class : 0U);
  service.add("offset_ticks", valid ? f.diag_service_offset_signed_ticks : 0);
  service.add("arm_to_isr_ticks", valid ? f.diag_arm_to_isr_ticks : 0U);
  service.add("last_counter_delta_ticks", valid ? f.diag_last_counter_delta_ticks : 0U);
  parent.add_object("service", service);
}

static void payload_add_lane_regression_object(Payload& parent,
                                               bool valid,
                                               const clocks_alpha_lane_forensics_t& f) {
  Payload regression;

  // Compact report-only LR edge proof for TIMEBASE_FORENSICS.  The LR engine
  // remains diagnostic-only in this pass: subscribers/TIMEBASE authority still
  // use the existing DWT path, while this object carries the independent
  // current-window inferred DWT-at-edge beside observed and EMA/authority DWT
  // facts for raw_cycles-style review.
  const bool lr_valid = valid && f.regression_valid;
  const uint32_t authority_dwt = valid ? f.dwt_used_at_event : 0U;
  const int32_t inferred_minus_authority =
      (lr_valid && authority_dwt != 0U)
          ? (int32_t)(f.regression_inferred_dwt_at_event - authority_dwt)
          : 0;

  regression.add("enabled", true);
  regression.add("report_only", true);
  regression.add("authority_enabled", false);
  regression.add("valid", lr_valid);
  regression.add("sequence", lr_valid ? f.regression_sequence : 0U);
  regression.add("sample_count", lr_valid ? f.regression_sample_count : 0U);

  // The three DWT endpoints raw_cycles wants side-by-side:
  //   observed  — service-offset-corrected target-edge DWT from process_interrupt
  //   inferred  — LR current-window estimate at the one-second target
  //   authority — current subscriber/EMA DWT endpoint carried in the diag
  regression.add("observed_dwt_at_event",
                 lr_valid ? f.regression_observed_dwt_at_event : 0U);
  regression.add("inferred_dwt_at_event",
                 lr_valid ? f.regression_inferred_dwt_at_event : 0U);
  regression.add("authority_dwt_at_event",
                 lr_valid ? authority_dwt : 0U);
  regression.add("authority_source", "EMA_DIAG_DWT_USED_AT_EVENT");
  regression.add("inferred_minus_observed_cycles",
                 lr_valid ? f.regression_inferred_minus_observed_cycles : 0);
  regression.add("inferred_minus_authority_cycles",
                 lr_valid ? inferred_minus_authority : 0);

  regression.add("target_counter32_at_event",
                 lr_valid ? f.regression_target_counter32_at_event : 0U);
  regression.add("target_hardware16_at_event",
                 lr_valid ? (uint32_t)f.regression_target_hardware16_at_event : 0U);
  regression.add("observed_hardware16_at_event",
                 lr_valid ? (uint32_t)f.regression_observed_hardware16_at_event : 0U);

  // Keep fit quality compact.  Detailed lifecycle/segment fields stay in the
  // focused interrupt lane reports to protect the 1 Hz forensics payload.
  regression.add("slope_delta_q16_cycles_per_sample",
                 lr_valid ? f.regression_slope_delta_q16_cycles_per_sample : 0LL);
  regression.add("fit_error_mean_q16_cycles",
                 lr_valid ? f.regression_fit_error_mean_q16_cycles : 0);
  regression.add("fit_error_stddev_q16_cycles",
                 lr_valid ? f.regression_fit_error_stddev_q16_cycles : 0U);
  regression.add("fit_error_abs_gt4_count",
                 lr_valid ? f.regression_fit_error_abs_gt4_count : 0U);

  parent.add_object("regression", regression);
}

// ============================================================================
// OCXO cycle-domain residual diagnostic
// ============================================================================
//
// Courtroom-only diagnostic for the current OCXO normalization investigation.
// The authoritative OCXO residual remains the PPS-founded public nanosecond
// residual.  Welfords, tau, and servo input continue to consume only that
// traditional surface.
//
// This object compares the GNSS/PPS one-second DWT interval and each OCXO
// one-second DWT interval in the same Teensy DWT coordinate species.  If the
// nanosecond-domain residual and this same-yardstick cycle residual diverge in
// common mode, the residual path is leaking DWT normalization error.
//
// Sign convention is aligned with pps_residual.fast_residual_ns:
//   positive diagnostic_fast_residual_cycles => OCXO running fast.
//
// Since a fast OCXO produces a shorter OCXO one-second period in DWT cycles,
// the sign-aligned diagnostic is:
//
//   diagnostic_fast_residual_cycles = gnss_actual_cycles - ocxo_actual_cycles
//
// The raw Dave-subtraction is also published as clock_minus_gnss_actual_cycles.

struct ocxo_cycle_residual_diag_t {
  bool     valid = false;
  bool     traditional_valid = false;

  uint32_t gnss_prediction_cycles = 0;
  uint32_t clock_prediction_cycles = 0;
  int64_t  prediction_fast_residual_cycles = 0;
  int64_t  clock_minus_gnss_prediction_cycles = 0;

  uint32_t gnss_actual_cycles = 0;
  uint32_t clock_actual_cycles = 0;
  int64_t  diagnostic_fast_residual_cycles = 0;
  int64_t  clock_minus_gnss_actual_cycles = 0;

  int64_t  traditional_fast_residual_ns = 0;
  int64_t  diagnostic_minus_traditional = 0;
};

static ocxo_cycle_residual_diag_t ocxo_cycle_residual_diag_build(
    const clocks_static_prediction_snapshot_t& gnss,
    const clocks_static_prediction_snapshot_t& clock,
    bool traditional_valid,
    int64_t traditional_fast_residual_ns) {
  ocxo_cycle_residual_diag_t d{};

  d.valid = gnss.valid && clock.valid &&
      gnss.actual_cycles != 0U && clock.actual_cycles != 0U;
  d.traditional_valid = traditional_valid;

  d.gnss_prediction_cycles = gnss.static_prediction_cycles;
  d.clock_prediction_cycles = clock.static_prediction_cycles;
  d.prediction_fast_residual_cycles =
      (int64_t)gnss.static_prediction_cycles -
      (int64_t)clock.static_prediction_cycles;
  d.clock_minus_gnss_prediction_cycles =
      (int64_t)clock.static_prediction_cycles -
      (int64_t)gnss.static_prediction_cycles;

  d.gnss_actual_cycles = gnss.actual_cycles;
  d.clock_actual_cycles = clock.actual_cycles;
  d.diagnostic_fast_residual_cycles =
      (int64_t)gnss.actual_cycles - (int64_t)clock.actual_cycles;
  d.clock_minus_gnss_actual_cycles =
      (int64_t)clock.actual_cycles - (int64_t)gnss.actual_cycles;

  d.traditional_fast_residual_ns =
      traditional_valid ? traditional_fast_residual_ns : 0LL;
  d.diagnostic_minus_traditional =
      traditional_valid ? (d.diagnostic_fast_residual_cycles -
                           traditional_fast_residual_ns) : 0LL;
  return d;
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

static void payload_add_ocxo_cycle_residual_diag_object(
    Payload& parent,
    const ocxo_cycle_residual_diag_t& d) {
  Payload diag;

  diag.add("valid", d.valid);
  diag.add("diagnostic_only", true);
  diag.add("doctrine", "DWT_SAME_YARDSTICK_RESIDUAL_CHECK");
  diag.add("positive_means", "clock_fast");
  diag.add("welford_source", false);
  diag.add("servo_source", false);

  diag.add("gnss_prediction_cycles", d.gnss_prediction_cycles);
  diag.add("clock_prediction_cycles", d.clock_prediction_cycles);
  diag.add("prediction_fast_residual_cycles",
           d.prediction_fast_residual_cycles);
  diag.add("clock_minus_gnss_prediction_cycles",
           d.clock_minus_gnss_prediction_cycles);

  diag.add("gnss_actual_cycles", d.gnss_actual_cycles);
  diag.add("clock_actual_cycles", d.clock_actual_cycles);
  diag.add("diagnostic_fast_residual_cycles",
           d.diagnostic_fast_residual_cycles);
  diag.add("clock_minus_gnss_actual_cycles",
           d.clock_minus_gnss_actual_cycles);

  diag.add("traditional_residual_valid", d.traditional_valid);
  diag.add("traditional_fast_residual_ns",
           d.traditional_fast_residual_ns);
  diag.add("diagnostic_minus_traditional",
           d.diagnostic_minus_traditional);

  parent.add_object("cycle_residual_diagnostic", diag);
}

static void payload_add_timebase_pair_identity(Payload& p,
                                               const char* schema,
                                               const char* version_key,
                                               uint32_t version,
                                               const char* pair_role,
                                               uint32_t public_count,
                                               uint64_t public_gnss_ns) {
  p.add("schema", schema);
  p.add(version_key, version);
  p.add("timebase_pair_version", 1U);
  p.add("pair_role", pair_role ? pair_role : "");
  p.add("campaign", campaign_name);
  p.add("teensy_pps_vclock_count", public_count);
  p.add("teensy_pps_count",        public_count);
  p.add("pps_count",               public_count);
  p.add("gnss_ns",                 public_gnss_ns);
  p.add("dwt_at_pps_vclock",       (uint32_t)g_dwt_at_pps_vclock);
  p.add("dwt_cycles_between_pps_vclock", (uint32_t)g_dwt_cycles_between_pps_vclock);
  p.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);
  p.add("pps_dwt_at_edge",         (uint32_t)g_pps_dwt_at_edge);
  p.add("pps_vclock_phase_cycles", (int32_t)g_pps_vclock_phase_cycles);

  // Durable servo mode identity.  TIMEBASE consumers should not infer active
  // calibration merely from the presence of DAC persistence values.  Publish
  // the explicit CLOCKS-owned servo mode in both the fragment and forensics
  // halves of the paired row.
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("servo_active", calibrate_ocxo_mode != servo_mode_t::OFF);
}

static void payload_add_vclock_fragment(Payload& p, uint64_t public_gnss_ns) {
  Payload lane;
  lane.add("ns", public_gnss_ns);
  lane.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);
  p.add_object("vclock", lane);
}

static void payload_add_vclock_forensics(Payload& p,
                                         uint64_t public_gnss_ns,
                                         bool forensics_valid,
                                         const clocks_alpha_lane_forensics_t& f) {
  Payload lane;
  lane.add("ns", public_gnss_ns);
  lane.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);

  // TIMEBASE_FORENSICS is now a compact per-row courtroom audit.  Deep
  // measurement and SpinCatch/SpinIdle remain available through focused
  // reports; the paired 1 Hz stream carries DWT authority proof plus the
  // compact LR inferred-edge witness needed by raw_cycles.
  payload_add_lane_forensics_object(lane, forensics_valid, f);
  payload_add_lane_regression_object(lane, forensics_valid, f);
  p.add_object("vclock", lane);
}

static void payload_add_ocxo_fragment(Payload& p,
                                      const char* key,
                                      uint64_t public_ns,
                                      uint64_t public_measured_ns,
                                      bool pps_projection_valid,
                                      uint32_t pps_projection_source,
                                      bool pps_residual_valid,
                                      uint64_t pps_gnss_interval_ns,
                                      uint64_t pps_clock_interval_ns,
                                      int64_t pps_fast_residual_ns) {
  Payload lane;

  // Compact science surface.  The canonical OCXO value is the PPS-projected
  // clock when available, with measured_gnss_ns retained as the single legacy
  // side-channel for dashboards/report migration.  Duplicate string aliases
  // and repeated *_at_pps_vclock names were removed from the durable row.
  lane.add("ns", public_ns);
  lane.add("ns_source_id", pps_projection_valid ? 2U : 1U);
  lane.add("pps_projected_valid", pps_projection_valid);
  lane.add("pps_projection_source", pps_projection_valid ? pps_projection_source : 0U);
  lane.add("measured_gnss_ns", public_measured_ns);
  const int64_t measured_minus_ns =
      (public_measured_ns >= public_ns)
          ? (int64_t)(public_measured_ns - public_ns)
          : -(int64_t)(public_ns - public_measured_ns);
  lane.add("measured_minus_ns", measured_minus_ns);

  payload_add_ocxo_pps_residual_object(lane,
                                       pps_residual_valid,
                                       pps_gnss_interval_ns,
                                       pps_clock_interval_ns,
                                       pps_fast_residual_ns);

  p.add_object(key, lane);
}

static void payload_add_ocxo_forensics(Payload& p,
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
                                       int64_t pps_fast_residual_ns,
                                       const ocxo_cycle_residual_diag_t& cycle_diag) {
  (void)clock;
  (void)meas;
  (void)public_pps_projected_ns;

  Payload lane;

  // Compact forensic surface for the paired TIMEBASE row.  Keep the public
  // clock value, source identity, the raw PPS-projection value, the PPS-founded
  // residual, the DWT gate courtroom, the tiny OCXO service/counter ladder
  // proof, and the compact LR inferred-edge witness.  Retired quiet-phase
  // sample fields, measurement/window counters, SpinCatch/SpinIdle, and verbose
  // projection deltas remain report-only.
  lane.add("ns", public_ns);
  lane.add("ns_source_id", pps_projection_valid ? 2U : 1U);
  lane.add("pps_projected_valid", pps_projection_valid);
  lane.add("pps_projected_raw_ns",
           pps_projection_valid ? pps_projection.projected_ocxo_ns_at_pps : 0ULL);
  lane.add("pps_projection_source", pps_projection_valid ? pps_projection.source : 0U);
  lane.add("measured_gnss_ns", public_measured_ns);

  payload_add_ocxo_pps_residual_object(lane,
                                       pps_residual_valid,
                                       pps_gnss_interval_ns,
                                       pps_clock_interval_ns,
                                       pps_fast_residual_ns);

  payload_add_ocxo_cycle_residual_diag_object(lane, cycle_diag);

  payload_add_lane_forensics_object(lane, forensics_valid, f);
  payload_add_ocxo_service_object(lane, forensics_valid, f);
  payload_add_lane_regression_object(lane, forensics_valid, f);
  p.add_object(key, lane);
}

// Step E servo source doctrine. Servo input is explicitly the PPS-founded
// OCXO slope surface introduced in Step D. MEAN consumes the Welford mean of
// public PPS-edge residuals (ns/sec == ppb). TOTAL consumes the total public
// OCXO/GNSS ratio over the campaign (tau). NOW consumes the most recent
// PPS-founded one-second residual directly. NOW is intentionally aggressive
// and exists for live plant-response testing and fast visible convergence.
static constexpr uint32_t SERVO_INPUT_SOURCE_NONE = 0;
static constexpr uint32_t SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD = 1;
static constexpr uint32_t SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU = 2;
static constexpr uint32_t SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL = 3;

static const char* servo_input_source_name(uint32_t source) {
  switch (source) {
    case SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD:
      return "PPS_RESIDUAL_WELFORD_MEAN_PPB";
    case SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU:
      return "PUBLIC_OCXO_TOTAL_TAU";
    case SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL:
      return "PPS_RESIDUAL_NOW_PPB";
    default:
      return "NONE";
  }
}

struct servo_input_diag_t {
  bool     pps_residual_valid = false;
  uint64_t pps_gnss_interval_ns = 0;
  uint64_t pps_clock_interval_ns = 0;
  int64_t  pps_fast_residual_ns = 0;

  uint64_t mean_welford_n = 0;
  double   mean_welford_ppb = 0.0;
  bool     mean_input_valid = false;

  double   total_tau = 1.0;
  double   total_ppb = 0.0;
  bool     total_input_valid = false;

  double   total_catchup_target_now_ppb = 0.0;
  double   total_catchup_control_error_ppb = 0.0;
  double   total_catchup_elapsed_seconds = 0.0;
  double   total_catchup_horizon_seconds = 0.0;
  double   total_catchup_max_target_ppb = 0.0;
  bool     total_catchup_active = false;

  double   now_ppb = 0.0;
  bool     now_input_valid = false;

  uint32_t selected_source = SERVO_INPUT_SOURCE_NONE;
  bool     selected_input_valid = false;
  double   selected_input_ppb = 0.0;
  int64_t  selected_residual_ns = 0;
};

static servo_input_diag_t g_servo_input_ocxo1 = {};
static servo_input_diag_t g_servo_input_ocxo2 = {};

static void servo_input_diag_reset(servo_input_diag_t& d) {
  d = servo_input_diag_t{};
}

static FLASHMEM void payload_add_servo_input_diag(Payload& lane,
                                         const servo_input_diag_t& d) {
  Payload input;
  input.add("control_doctrine", "SLOPE_NEUTRALIZATION");
  input.add("positive_means", "clock_fast_lower_dac");
  input.add("dac_transfer", "higher_dac_higher_voltage_faster_ocxo");
  input.add("selected_source", servo_input_source_name(d.selected_source));
  input.add("selected_source_id", d.selected_source);
  input.add("selected_valid", d.selected_input_valid);
  input.add("selected_ppb", d.selected_input_ppb, 6);
  input.add("selected_residual_ns", d.selected_residual_ns);
  input.add("pps_residual_valid", d.pps_residual_valid);
  input.add("pps_fast_residual_ns", d.pps_fast_residual_ns);
  input.add("pps_gnss_interval_ns", d.pps_gnss_interval_ns);
  input.add("pps_clock_interval_ns", d.pps_clock_interval_ns);
  input.add("mean_welford_n", d.mean_welford_n);
  input.add("mean_welford_ppb", d.mean_welford_ppb, 6);
  input.add("mean_input_valid", d.mean_input_valid);
  input.add("total_tau", d.total_tau, 12);
  input.add("total_ppb", d.total_ppb, 6);
  input.add("total_input_valid", d.total_input_valid);
  input.add("total_catchup_active", d.total_catchup_active);
  input.add("total_catchup_target_now_ppb", d.total_catchup_target_now_ppb, 6);
  input.add("total_catchup_control_error_ppb", d.total_catchup_control_error_ppb, 6);
  input.add("total_catchup_elapsed_seconds", d.total_catchup_elapsed_seconds, 3);
  input.add("total_catchup_horizon_seconds", d.total_catchup_horizon_seconds, 3);
  input.add("total_catchup_max_target_ppb", d.total_catchup_max_target_ppb, 3);
  input.add("now_ppb", d.now_ppb, 6);
  input.add("now_input_valid", d.now_input_valid);
  lane.add_object("servo_input", input);
}

static FLASHMEM void payload_add_dac_summary_hierarchical(Payload& p) {
  Payload dac;
  dac.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  dac.add("ad5693r_init_ok", g_ad5693r_init_ok);

  Payload o1;
  o1.add("value", ocxo1_dac.dac_fractional);
  o1.add("last_write_ok", ocxo1_dac.io_last_write_ok);
  o1.add("fault_latched", ocxo1_dac.io_fault_latched);
  o1.add("servo_control_ppb", ocxo1_dac.servo_predicted_residual, 6);
  o1.add("servo_last_step", ocxo1_dac.servo_last_step, 6);
  o1.add("servo_adjustments", ocxo1_dac.servo_adjustments);
  payload_add_servo_input_diag(o1, g_servo_input_ocxo1);
  dac.add_object("ocxo1", o1);

  Payload o2;
  o2.add("value", ocxo2_dac.dac_fractional);
  o2.add("last_write_ok", ocxo2_dac.io_last_write_ok);
  o2.add("fault_latched", ocxo2_dac.io_fault_latched);
  o2.add("servo_control_ppb", ocxo2_dac.servo_predicted_residual, 6);
  o2.add("servo_last_step", ocxo2_dac.servo_last_step, 6);
  o2.add("servo_adjustments", ocxo2_dac.servo_adjustments);
  payload_add_servo_input_diag(o2, g_servo_input_ocxo2);
  dac.add_object("ocxo2", o2);

  p.add_object("dac", dac);
}

// ============================================================================
// Always-on OCXO DAC SyncDAC realization
// ============================================================================
//
// SyncDAC runs a fixed 1 kHz virtual timing grid but does not write the DAC at
// 1 kHz.  Each one-second frame is divided into ten 100 ms cells.  The desired
// fractional DAC code selects the total HIGH dwell time in milliseconds for
// that frame; the dwell time is then distributed across the ten cells with a
// small Bresenham-style remainder spread.  Within each cell, the output holds
// one adjacent code for part of the cell and the other adjacent code for the
// remainder.  I2C writes occur only at LOW/HIGH state boundaries and are capped
// to keep the shared SMBus topology safe.
//
// Example: desired 59745.763 -> LOW=59745, HIGH=59746.
//   HIGH dwell = 763 ms/sec, LOW dwell = 237 ms/sec.
//   Cells receive either 76 or 77 ms of HIGH dwell so the full second realizes
//   59745.763 while using sparse DAC state changes.

static constexpr uint32_t OCXO_DAC_SYNCDAC_EXECUTION_HZ = 1000U;
static constexpr uint32_t OCXO_DAC_SYNCDAC_PERIOD_NS = 1000000U;
static constexpr uint32_t OCXO_DAC_SYNCDAC_FRAME_TICKS = 1000U;
static constexpr uint32_t OCXO_DAC_SYNCDAC_CELL_COUNT = 10U;
static constexpr uint32_t OCXO_DAC_SYNCDAC_CELL_TICKS =
    OCXO_DAC_SYNCDAC_FRAME_TICKS / OCXO_DAC_SYNCDAC_CELL_COUNT;
static constexpr uint32_t OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME = 20U;
static constexpr const char* OCXO_DAC_SYNCDAC_NAME = "OCXO_DAC_SYNCDAC_1KHZ";
static constexpr const char* OCXO_DAC_REALIZATION_MODE = "SYNCDAC";

static_assert(OCXO_DAC_SYNCDAC_EXECUTION_HZ == 1000U,
              "SyncDAC execution grid is fixed at 1 kHz");
static_assert(OCXO_DAC_SYNCDAC_CELL_COUNT * OCXO_DAC_SYNCDAC_CELL_TICKS ==
              OCXO_DAC_SYNCDAC_FRAME_TICKS,
              "SyncDAC cells must fill exactly one second");

static timepop_handle_t g_ocxo_dac_dither_handle = TIMEPOP_INVALID_HANDLE;
static uint32_t g_ocxo_dac_dither_arm_count = 0;
static uint32_t g_ocxo_dac_dither_arm_fail_count = 0;

static uint32_t ocxo_dac_syncdac_high_ms_from_fraction(uint32_t frac_q16) {
  if (frac_q16 == 0U) return 0U;
  const uint32_t high_ms =
      (uint32_t)(((uint64_t)frac_q16 * (uint64_t)OCXO_DAC_SYNCDAC_FRAME_TICKS +
                  32768ULL) >> 16);
  return (high_ms > OCXO_DAC_SYNCDAC_FRAME_TICKS)
      ? OCXO_DAC_SYNCDAC_FRAME_TICKS
      : high_ms;
}

static uint32_t ocxo_dac_syncdac_cell_high_ms(uint32_t total_high_ms,
                                              uint32_t cell_index) {
  const uint32_t base = total_high_ms / OCXO_DAC_SYNCDAC_CELL_COUNT;
  const uint32_t rem  = total_high_ms % OCXO_DAC_SYNCDAC_CELL_COUNT;
  const uint32_t extra_before = (cell_index * rem) / OCXO_DAC_SYNCDAC_CELL_COUNT;
  const uint32_t extra_after  = ((cell_index + 1U) * rem) / OCXO_DAC_SYNCDAC_CELL_COUNT;
  const uint32_t extra = extra_after - extra_before;
  const uint32_t high = base + extra;
  return (high > OCXO_DAC_SYNCDAC_CELL_TICKS) ? OCXO_DAC_SYNCDAC_CELL_TICKS : high;
}

static double ocxo_dac_last_window_average(const ocxo_dac_state_t& s) {
  const uint32_t ticks = s.dither_last_window_tick_count;
  if (ticks == 0) return s.dac_fractional;
  const uint32_t high = s.dither_last_window_high_count;
  return (double)s.dither_last_window_low_code + ((double)high / (double)ticks);
}

static FLASHMEM void payload_add_dac_dither_object(Payload& parent,
                                          const char* key,
                                          const ocxo_dac_state_t& s) {
  Payload d;
  d.add("enabled", s.dither_enabled);
  d.add("mode", OCXO_DAC_REALIZATION_MODE);
  d.add("execution_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  d.add("rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);  // compatibility alias
  d.add("period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  d.add("grid_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  d.add("frame_ticks", OCXO_DAC_SYNCDAC_FRAME_TICKS);
  d.add("cell_count", OCXO_DAC_SYNCDAC_CELL_COUNT);
  d.add("cell_ticks", OCXO_DAC_SYNCDAC_CELL_TICKS);
  d.add("max_writes_per_frame", OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME);
  d.add("desired_q16", s.dac_desired_q16);
  d.add("desired", s.dac_fractional, 6);
  d.add("hw_code", (uint32_t)s.dac_hw_code);
  d.add("low_code", (uint32_t)s.dither_low_code);
  d.add("high_code", (uint32_t)s.dither_high_code);
  d.add("fraction_q16", s.dither_fraction_q16);
  d.add("last_selected_hw_code", (uint32_t)s.dither_last_selected_hw_code);

  d.add("frame_tick", s.syncdac_frame_tick);
  d.add("cell_index", s.syncdac_cell_index);
  d.add("cell_tick", s.syncdac_cell_tick);
  d.add("high_ms_per_second", s.syncdac_high_ms_per_second);
  d.add("low_ms_per_second", s.syncdac_low_ms_per_second);
  d.add("high_ms_this_cell", s.syncdac_high_ms_this_cell);
  d.add("low_ms_this_cell", s.syncdac_low_ms_this_cell);

  d.add("window_sequence", s.dither_window_sequence);
  d.add("window_tick_count", s.dither_window_tick_count);
  d.add("window_low_ms", s.dither_window_low_count);
  d.add("window_high_ms", s.dither_window_high_count);
  d.add("last_window_tick_count", s.dither_last_window_tick_count);
  d.add("last_window_low_ms", s.dither_last_window_low_count);
  d.add("last_window_high_ms", s.dither_last_window_high_count);
  d.add("last_window_low_count", s.dither_last_window_low_count);    // compatibility alias
  d.add("last_window_high_count", s.dither_last_window_high_count);  // compatibility alias
  d.add("last_window_low_code", (uint32_t)s.dither_last_window_low_code);
  d.add("last_window_high_code", (uint32_t)s.dither_last_window_high_code);
  d.add("last_window_average", ocxo_dac_last_window_average(s), 6);

  d.add("tick_count_total", s.dither_tick_count_total);
  d.add("write_attempts", s.dither_write_attempts);
  d.add("write_successes", s.dither_write_successes);
  d.add("write_failures", s.dither_write_failures);
  d.add("write_skip_same_code_count", s.dither_write_skip_same_code_count);
  d.add("not_ready_count", s.dither_not_ready_count);
  d.add("writes_this_frame", s.syncdac_write_attempts_this_frame);
  d.add("write_successes_this_frame", s.syncdac_write_successes_this_frame);
  d.add("write_failures_this_frame", s.syncdac_write_failures_this_frame);
  d.add("write_suppressed_this_frame", s.syncdac_write_suppressed_this_frame);
  d.add("last_window_write_attempts", s.syncdac_last_window_write_attempts);
  d.add("last_window_write_successes", s.syncdac_last_window_write_successes);
  d.add("last_window_write_failures", s.syncdac_last_window_write_failures);
  d.add("last_window_write_suppressed", s.syncdac_last_window_write_suppressed);
  d.add("write_budget_suppressed_total", s.syncdac_write_budget_suppressed_total);
  parent.add_object(key, d);
}

static void ocxo_dac_dither_roll_window(ocxo_dac_state_t& s) {
  s.dither_last_window_tick_count = s.dither_window_tick_count;
  s.dither_last_window_low_count = s.dither_window_low_count;
  s.dither_last_window_high_count = s.dither_window_high_count;
  s.dither_last_window_low_code = s.dither_low_code;
  s.dither_last_window_high_code = s.dither_high_code;
  s.syncdac_last_window_write_attempts = s.syncdac_write_attempts_this_frame;
  s.syncdac_last_window_write_successes = s.syncdac_write_successes_this_frame;
  s.syncdac_last_window_write_failures = s.syncdac_write_failures_this_frame;
  s.syncdac_last_window_write_suppressed = s.syncdac_write_suppressed_this_frame;

  s.dither_window_sequence++;
  s.dither_window_tick_count = 0;
  s.dither_window_low_count = 0;
  s.dither_window_high_count = 0;
  s.syncdac_write_attempts_this_frame = 0;
  s.syncdac_write_successes_this_frame = 0;
  s.syncdac_write_failures_this_frame = 0;
  s.syncdac_write_suppressed_this_frame = 0;
}

static void ocxo_dac_syncdac_refresh_codes(ocxo_dac_state_t& s) {
  const uint32_t desired_q16 = s.dac_desired_q16;
  const uint16_t low = (uint16_t)(desired_q16 >> 16);
  const uint32_t frac = desired_q16 & 0xFFFFUL;
  const uint16_t high = (low >= 65535U || frac == 0U)
      ? low
      : (uint16_t)(low + 1U);

  s.dither_fraction_q16 = frac;
  s.dither_low_code = low;
  s.dither_high_code = high;
  s.dither_accumulator_q16 = 0;
  s.syncdac_high_ms_per_second = ocxo_dac_syncdac_high_ms_from_fraction(frac);
  s.syncdac_low_ms_per_second =
      OCXO_DAC_SYNCDAC_FRAME_TICKS - s.syncdac_high_ms_per_second;
}

static uint16_t ocxo_dac_syncdac_selected_code(ocxo_dac_state_t& s) {
  const uint32_t cell = s.syncdac_frame_tick / OCXO_DAC_SYNCDAC_CELL_TICKS;
  const uint32_t pos  = s.syncdac_frame_tick % OCXO_DAC_SYNCDAC_CELL_TICKS;
  const uint32_t high_ms = ocxo_dac_syncdac_cell_high_ms(
      s.syncdac_high_ms_per_second, cell);
  const uint32_t low_ms = OCXO_DAC_SYNCDAC_CELL_TICKS - high_ms;

  s.syncdac_cell_index = cell;
  s.syncdac_cell_tick = pos;
  s.syncdac_high_ms_this_cell = high_ms;
  s.syncdac_low_ms_this_cell = low_ms;

  if (s.dither_high_code == s.dither_low_code || high_ms == 0U) {
    return s.dither_low_code;
  }
  if (high_ms >= OCXO_DAC_SYNCDAC_CELL_TICKS) {
    return s.dither_high_code;
  }

  // Alternate the order so cell boundaries usually preserve the previous code.
  // This avoids a large phase-oriented one-second block and also reduces bus
  // traffic while preserving the exact per-cell dwell ratio.
  const bool high_first = ((cell & 1U) != 0U);
  if (high_first) {
    return (pos < high_ms) ? s.dither_high_code : s.dither_low_code;
  }
  return (pos < low_ms) ? s.dither_low_code : s.dither_high_code;
}

static void ocxo_dac_syncdac_write_selected(ocxo_dac_state_t& s,
                                            uint16_t selected) {
  s.dither_last_selected_hw_code = selected;

  if (!g_ad5693r_init_ok) {
    s.dither_not_ready_count++;
    return;
  }

  if (selected == s.dac_hw_code) {
    s.dither_write_skip_same_code_count++;
    return;
  }

  if (s.syncdac_write_attempts_this_frame >=
      OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME) {
    s.syncdac_write_suppressed_this_frame++;
    s.syncdac_write_budget_suppressed_total++;
    return;
  }

  s.syncdac_write_attempts_this_frame++;
  s.dither_write_attempts++;
  if (ocxo_dac_write_hw_code(s, selected, false)) {
    s.syncdac_write_successes_this_frame++;
    s.dither_write_successes++;
  } else {
    // SyncDAC writes are opportunistic.  Occasional I2C failures are evidence,
    // not campaign-fatal faults; the next scheduled edge will try again if
    // still needed and within the frame write budget.
    s.syncdac_write_failures_this_frame++;
    s.dither_write_failures++;
  }
}

static void ocxo_dac_dither_apply(ocxo_dac_state_t& s) {
  if (!s.dither_enabled) return;

  s.dither_rate_hz = OCXO_DAC_SYNCDAC_EXECUTION_HZ;
  s.dither_period_ns = OCXO_DAC_SYNCDAC_PERIOD_NS;
  s.dither_effective_window_ticks = OCXO_DAC_SYNCDAC_FRAME_TICKS;

  ocxo_dac_syncdac_refresh_codes(s);
  const uint16_t selected = ocxo_dac_syncdac_selected_code(s);

  s.dither_tick_count_total++;
  s.dither_window_tick_count++;
  if (selected == s.dither_high_code && s.dither_high_code != s.dither_low_code) {
    s.dither_window_high_count++;
  } else {
    s.dither_window_low_count++;
  }

  ocxo_dac_syncdac_write_selected(s, selected);

  s.syncdac_frame_tick++;
  if (s.syncdac_frame_tick >= OCXO_DAC_SYNCDAC_FRAME_TICKS) {
    s.syncdac_frame_tick = 0;
    ocxo_dac_dither_roll_window(s);
  }
}

static void ocxo_dac_dither_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  ocxo_dac_dither_apply(ocxo1_dac);
  ocxo_dac_dither_apply(ocxo2_dac);
}

void clocks_dac_dither_begin(void) {
  if (g_ocxo_dac_dither_handle != TIMEPOP_INVALID_HANDLE) return;
  ocxo1_dac.dither_rate_hz = OCXO_DAC_SYNCDAC_EXECUTION_HZ;
  ocxo1_dac.dither_period_ns = OCXO_DAC_SYNCDAC_PERIOD_NS;
  ocxo1_dac.dither_effective_window_ticks = OCXO_DAC_SYNCDAC_FRAME_TICKS;
  ocxo2_dac.dither_rate_hz = OCXO_DAC_SYNCDAC_EXECUTION_HZ;
  ocxo2_dac.dither_period_ns = OCXO_DAC_SYNCDAC_PERIOD_NS;
  ocxo2_dac.dither_effective_window_ticks = OCXO_DAC_SYNCDAC_FRAME_TICKS;

  const timepop_handle_t h = timepop_arm((uint64_t)OCXO_DAC_SYNCDAC_PERIOD_NS,
                                         true,
                                         ocxo_dac_dither_callback,
                                         nullptr,
                                         OCXO_DAC_SYNCDAC_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_ocxo_dac_dither_arm_fail_count++;
    return;
  }
  g_ocxo_dac_dither_handle = h;
  g_ocxo_dac_dither_arm_count++;
}

// ============================================================================
// Slope servo tuning
// ============================================================================

// Conservative plant estimate: ppb change per DAC LSB.  The sign is positive:
// increasing DAC voltage increases OCXO ppb/tau.  The servo therefore applies
// a negative DAC step for positive ppb and a positive DAC step for negative ppb.
static constexpr double SERVO_PPB_PER_DAC_LSB_ESTIMATE = 100.0;
static constexpr double SERVO_CONTROL_DEADBAND_PPB = 3.0;
static constexpr double SERVO_SOFT_LANDING_PPB = 300.0;
static constexpr double SERVO_MIN_STEP_LSB = 1.0;

static constexpr double SERVO_MEAN_FILTER_ALPHA = 0.35;
static constexpr double SERVO_MEAN_GAIN = 1.00;

static constexpr double SERVO_TOTAL_FILTER_ALPHA = 0.45;
static constexpr double SERVO_TOTAL_GAIN = 0.75;
static constexpr double SERVO_TOTAL_DEADBAND_PPB = 1.0;

// TOTAL mode is the long-haul science servo, but pure campaign-total tau is
// slow to correct because early accumulated error remains in the denominator.
// Shape TOTAL into a catch-up controller: compute the instantaneous ppb we
// want the OCXO to run at so the campaign-total error burns down over a fixed
// horizon, then servo the latest PPS residual toward that moving target.
//
// Example:
//   total_ppb = -80 at t=900s, horizon=120s
//   desired_now_ppb = +600 ppb (clamped)
//   control_error = now_ppb - desired_now_ppb
//
// Negative control_error raises DAC and speeds the OCXO; as total_ppb approaches
// zero the desired_now target collapses naturally toward zero.
static constexpr double SERVO_TOTAL_CATCHUP_HORIZON_SECONDS = 120.0;
static constexpr double SERVO_TOTAL_CATCHUP_MAX_TARGET_PPB = 700.0;

// NOW mode is deliberately immediate: no settle gate, no slope averaging, and
// a small deadband.  It is a live plant-response/test mode that chases the
// most recent PPS-founded one-second residual directly.
static constexpr double SERVO_NOW_FILTER_ALPHA = 1.00;
static constexpr double SERVO_NOW_GAIN = 1.00;
static constexpr double SERVO_NOW_DEADBAND_PPB = 0.5;

static double servo_total_tau_from_public(uint64_t public_gnss_ns,
                                          uint64_t public_clock_ns) {
  if (public_gnss_ns == 0) return 1.0;
  return (double)public_clock_ns / (double)public_gnss_ns;
}

static double servo_total_ppb_from_tau(double tau) {
  return (tau - 1.0) * 1.0e9;
}

static double servo_clamp(double value, double limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static double servo_total_catchup_target_now_ppb(double total_ppb,
                                                 uint64_t elapsed_seconds) {
  if (elapsed_seconds == 0) return 0.0;

  const double elapsed = (double)elapsed_seconds;
  const double target =
      -total_ppb * (elapsed / SERVO_TOTAL_CATCHUP_HORIZON_SECONDS);
  return servo_clamp(target, SERVO_TOTAL_CATCHUP_MAX_TARGET_PPB);
}

static void servo_input_diag_update(servo_input_diag_t& d,
                                    bool pps_residual_valid,
                                    uint64_t pps_gnss_interval_ns,
                                    uint64_t pps_clock_interval_ns,
                                    int64_t pps_fast_residual_ns,
                                    const welford_t& w,
                                    bool total_input_valid,
                                    double total_tau) {
  d.pps_residual_valid = pps_residual_valid;
  d.pps_gnss_interval_ns = pps_residual_valid ? pps_gnss_interval_ns : 0ULL;
  d.pps_clock_interval_ns = pps_residual_valid ? pps_clock_interval_ns : 0ULL;
  d.pps_fast_residual_ns = pps_residual_valid ? pps_fast_residual_ns : 0LL;

  d.mean_welford_n = w.n;
  d.mean_welford_ppb = (w.n > 0) ? w.mean : 0.0;
  d.mean_input_valid = pps_residual_valid && (w.n >= SERVO_MIN_SAMPLES);

  d.total_tau = total_input_valid ? total_tau : 1.0;
  d.total_ppb = total_input_valid ? servo_total_ppb_from_tau(total_tau) : 0.0;
  d.total_input_valid = total_input_valid && pps_residual_valid &&
                        (campaign_seconds >= SERVO_MIN_SAMPLES);

  // A one-second residual in ns is numerically ppb over a one-second gate.
  // NOW intentionally consumes this value directly rather than averaging it
  // into MEAN or folding it into campaign-total tau.
  d.now_ppb = pps_residual_valid ? (double)pps_fast_residual_ns : 0.0;
  d.now_input_valid = pps_residual_valid;

  d.total_catchup_elapsed_seconds = (double)campaign_seconds;
  d.total_catchup_horizon_seconds = SERVO_TOTAL_CATCHUP_HORIZON_SECONDS;
  d.total_catchup_max_target_ppb = SERVO_TOTAL_CATCHUP_MAX_TARGET_PPB;
  d.total_catchup_target_now_ppb = 0.0;
  d.total_catchup_control_error_ppb = 0.0;
  d.total_catchup_active = false;

  if (d.total_input_valid && d.now_input_valid) {
    d.total_catchup_target_now_ppb =
        servo_total_catchup_target_now_ppb(d.total_ppb, campaign_seconds);
    d.total_catchup_control_error_ppb =
        d.now_ppb - d.total_catchup_target_now_ppb;
    d.total_catchup_active = true;
  } else {
    d.total_catchup_control_error_ppb = d.total_ppb;
  }

  d.selected_residual_ns = 0;
  if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
    d.selected_source = SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD;
    d.selected_input_valid = d.mean_input_valid;
    d.selected_input_ppb = d.mean_input_valid ? d.mean_welford_ppb : 0.0;
    d.selected_residual_ns = (int64_t)d.selected_input_ppb;
  } else if (calibrate_ocxo_mode == servo_mode_t::TOTAL) {
    d.selected_source = SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU;
    d.selected_input_valid = d.total_input_valid;
    d.selected_input_ppb = d.total_input_valid
        ? d.total_catchup_control_error_ppb
        : 0.0;
    d.selected_residual_ns = (int64_t)d.selected_input_ppb;
  } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    d.selected_source = SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL;
    d.selected_input_valid = d.now_input_valid;
    d.selected_input_ppb = d.now_input_valid ? d.now_ppb : 0.0;
    d.selected_residual_ns = pps_residual_valid ? pps_fast_residual_ns : 0;
  } else {
    d.selected_source = SERVO_INPUT_SOURCE_NONE;
    d.selected_input_valid = false;
    d.selected_input_ppb = 0.0;
    d.selected_residual_ns = 0;
  }
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

static void ocxo_dac_apply_synthetic_servo_step(ocxo_dac_state_t& dac,
                                                double step) {
  const double before = dac.dac_fractional;
  (void)ocxo_dac_set_desired(dac, before + step);
  const double applied = dac.dac_fractional - before;
  dac.servo_last_step = applied;
  dac.servo_adjustments++;
  dac.pacing_intents++;
  dac.pacing_last_request_second = campaign_seconds;
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

  servo_input_diag_reset(g_servo_input_ocxo1);
  servo_input_diag_reset(g_servo_input_ocxo2);
}

// ============================================================================
// Servo logic
// ============================================================================

static double servo_soft_landing_scale(double abs_ppb) {
  if (abs_ppb >= SERVO_SOFT_LANDING_PPB) return 1.0;
  // Retain a little authority near zero but taper hard enough to avoid
  // marching through the target.  The <1 LSB gate below performs the final
  // deadband.
  return 0.25 + 0.75 * (abs_ppb / SERVO_SOFT_LANDING_PPB);
}

static void ocxo_servo_slope(ocxo_dac_state_t& dac,
                             const servo_input_diag_t& input,
                             double filter_alpha,
                             double gain,
                             double deadband_ppb = SERVO_CONTROL_DEADBAND_PPB,
                             bool use_soft_landing = true) {
  if (!input.selected_input_valid) return;

  const double ppb = input.selected_input_ppb;
  dac.servo_last_residual = ppb;

  if (!dac.servo_predictor_initialized) {
    dac.servo_predictor_initialized = true;
    dac.servo_last_raw_residual = ppb;
    dac.servo_filtered_residual = ppb;
    dac.servo_filtered_slope = 0.0;
    dac.servo_predicted_residual = ppb;
    dac.servo_predictor_updates = 1;
  } else {
    const double raw_delta_ppb = ppb - dac.servo_last_raw_residual;
    dac.servo_last_raw_residual = ppb;

    dac.servo_filtered_residual =
        (1.0 - filter_alpha) * dac.servo_filtered_residual +
        filter_alpha * ppb;

    // Diagnostic-only slope of the slope estimate; this is not used as a
    // phase-prediction term.  The control variable is the filtered ppb itself.
    dac.servo_filtered_slope =
        (1.0 - filter_alpha) * dac.servo_filtered_slope +
        filter_alpha * raw_delta_ppb;

    dac.servo_predicted_residual = dac.servo_filtered_residual;
    dac.servo_predictor_updates++;
  }

  const double control_ppb = dac.servo_predicted_residual;
  const double abs_ppb = fabs(control_ppb);
  if (abs_ppb < deadband_ppb) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  const double landing_scale = use_soft_landing ? servo_soft_landing_scale(abs_ppb) : 1.0;

  // Sign law:
  //   positive ppb / tau > 1  -> OCXO fast -> lower DAC -> negative step
  //   negative ppb / tau < 1  -> OCXO slow -> raise DAC -> positive step
  double step = -control_ppb / SERVO_PPB_PER_DAC_LSB_ESTIMATE;
  step *= gain * landing_scale;

  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  if (fabs(step) < 0.000001) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  ocxo_dac_apply_synthetic_servo_step(dac, step);
}

static void ocxo_servo_mean(ocxo_dac_state_t& dac,
                            const servo_input_diag_t& input) {
  if (!input.mean_input_valid) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  ocxo_servo_slope(dac, input, SERVO_MEAN_FILTER_ALPHA, SERVO_MEAN_GAIN);
  dac.servo_settle_count = 0;
}

static void ocxo_servo_total(ocxo_dac_state_t& dac,
                             const servo_input_diag_t& input) {
  if (!input.total_input_valid) return;

  // TOTAL now runs every second.  The selected ppb has already been shaped into
  // a catch-up control error:
  //
  //   selected_ppb = now_ppb - desired_now_ppb
  //
  // where desired_now_ppb is the instantaneous rate needed to burn down the
  // campaign-total error over SERVO_TOTAL_CATCHUP_HORIZON_SECONDS.  This makes
  // TOTAL aggressive when the accumulated error is large, while naturally
  // flattening as the total error approaches zero.
  dac.servo_settle_count = 0;
  ocxo_servo_slope(dac,
                   input,
                   SERVO_TOTAL_FILTER_ALPHA,
                   SERVO_TOTAL_GAIN,
                   SERVO_TOTAL_DEADBAND_PPB,
                   true);
}

static void ocxo_servo_now(ocxo_dac_state_t& dac,
                           const servo_input_diag_t& input) {
  if (!input.now_input_valid) return;

  // NOW chases the current one-second residual directly.  There is no settle
  // divider and no soft landing: SyncDAC can realize sub-LSB corrections, so
  // even small residuals are useful as live plant-response evidence.
  dac.servo_settle_count = 0;
  ocxo_servo_slope(dac, input,
                   SERVO_NOW_FILTER_ALPHA,
                   SERVO_NOW_GAIN,
                   SERVO_NOW_DEADBAND_PPB,
                   false);
}

static void ocxo_calibration_servo(void) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
    ocxo_servo_mean(ocxo1_dac, g_servo_input_ocxo1);
    ocxo_servo_mean(ocxo2_dac, g_servo_input_ocxo2);
  } else if (calibrate_ocxo_mode == servo_mode_t::TOTAL) {
    ocxo_servo_total(ocxo1_dac, g_servo_input_ocxo1);
    ocxo_servo_total(ocxo2_dac, g_servo_input_ocxo2);
  } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    ocxo_servo_now(ocxo1_dac, g_servo_input_ocxo1);
    ocxo_servo_now(ocxo2_dac, g_servo_input_ocxo2);
  }
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

static bool clocks_servo_active(void) {
  return calibrate_ocxo_mode != servo_mode_t::OFF;
}

static void payload_add_dac_dither_compact(Payload& parent,
                                           const char* key,
                                           const ocxo_dac_state_t& s) {
  Payload d;
  d.add("enabled", s.dither_enabled);
  d.add("mode", OCXO_DAC_REALIZATION_MODE);
  d.add("rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  d.add("period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  d.add("desired", s.dac_fractional, 6);
  d.add("hw_code", (uint32_t)s.dac_hw_code);
  d.add("low_code", (uint32_t)s.dither_last_window_low_code);
  d.add("high_code", (uint32_t)s.dither_last_window_high_code);
  d.add("low_count_1s", s.dither_last_window_low_count);
  d.add("high_count_1s", s.dither_last_window_high_count);
  d.add("low_ms_1s", s.dither_last_window_low_count);
  d.add("high_ms_1s", s.dither_last_window_high_count);
  d.add("tick_count_1s", s.dither_last_window_tick_count);
  d.add("average_1s", ocxo_dac_last_window_average(s), 6);
  d.add("writes_1s", s.syncdac_last_window_write_attempts);
  d.add("write_suppressed_1s", s.syncdac_last_window_write_suppressed);
  d.add("write_failures", s.dither_write_failures);
  parent.add_object(key, d);
}

static void payload_add_servo_dac_values(Payload& parent) {
  // Minimal durable DAC payload.  This is intentionally only the values the Pi
  // should persist as the current system DAC configuration, plus the explicit
  // servo mode that explains why this object is present in the TIMEBASE row.
  Payload dac;
  dac.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  dac.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  dac.add("servo_active", clocks_servo_active());
  dac.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  dac.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  dac.add("ocxo1_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  dac.add("ocxo2_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  dac.add("dither_mode", OCXO_DAC_REALIZATION_MODE);
  dac.add("dither_rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  dac.add("dither_period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  dac.add("dither_grid_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  dac.add("dither_cell_count", OCXO_DAC_SYNCDAC_CELL_COUNT);
  dac.add("dither_cell_ticks", OCXO_DAC_SYNCDAC_CELL_TICKS);
  dac.add("dither_max_writes_per_frame", OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME);

  Payload dither;
  payload_add_dac_dither_compact(dither, "ocxo1", ocxo1_dac);
  payload_add_dac_dither_compact(dither, "ocxo2", ocxo2_dac);
  dac.add_object("dither", dither);
  parent.add_object("dac", dac);
}

// ============================================================================
// clocks_beta_pps — invoked from alpha's pps_selector_callback
// ============================================================================

void clocks_beta_pps(void) {
  g_timebase_pps_entry_count++;
  g_timebase_last_entry_campaign_seconds = campaign_seconds;
  timebase_build_stage(TIMEBASE_BUILD_STAGE_ENTRY);

  if (request_stop) {
    g_timebase_stop_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_STOP_GATE);
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
    g_timebase_start_zero_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_START_ZERO_GATE);
    (void)clocks_try_finish_pending_smartzero();
    return;
  }

  if (request_recover) {
    g_timebase_recover_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_RECOVER_GATE);
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

  if (watchdog_anomaly_active) {
    g_timebase_watchdog_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_WATCHDOG_GATE);
    return;
  }
  if (campaign_state != clocks_campaign_state_t::STARTED) {
    g_timebase_not_started_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_NOT_STARTED_GATE);
    return;
  }

  // ── Warmup suppression ──
  //
  // Suppressed records still allow alpha's measurement/predictor state to
  // settle, but they are not allowed to become canonical TIMEBASE_FRAGMENT
  // rows.  START hides these seconds completely; RECOVER counts them as
  // deliberate canonical gaps.
  if (campaign_warmup_consume_one_candidate_record()) {
    g_timebase_warmup_suppressed_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_WARMUP_GATE);
    return;
  }

  g_timebase_candidate_count++;
  g_timebase_last_candidate_campaign_seconds = campaign_seconds;
  timebase_build_stage(TIMEBASE_BUILD_STAGE_CANDIDATE);

  // ── Per-second campaign work ──
  campaign_seconds++;
  g_timebase_per_second_count++;
  g_timebase_last_per_second_campaign_seconds = campaign_seconds;
  timebase_build_stage(TIMEBASE_BUILD_STAGE_PER_SECOND);

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

  // ── Cycle-domain residual diagnostics ──
  //
  // These are forensic-only.  They deliberately do not feed Welford, tau, or
  // servo control.  They compare each OCXO one-second DWT interval against the
  // GNSS/PPS one-second DWT interval using the same moving Teensy yardstick.
  const clocks_static_prediction_snapshot_t pps_cycle_prediction =
      prediction_snapshot_for_pps();
  const clocks_static_prediction_snapshot_t ocxo1_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO1);
  const clocks_static_prediction_snapshot_t ocxo2_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO2);
  const ocxo_cycle_residual_diag_t ocxo1_cycle_residual_diag =
      ocxo_cycle_residual_diag_build(pps_cycle_prediction, ocxo1_cycle_prediction,
                                     pps_residuals.ocxo1_valid,
                                     pps_residuals.ocxo1_fast_residual_ns);
  const ocxo_cycle_residual_diag_t ocxo2_cycle_residual_diag =
      ocxo_cycle_residual_diag_build(pps_cycle_prediction, ocxo2_cycle_prediction,
                                     pps_residuals.ocxo2_valid,
                                     pps_residuals.ocxo2_fast_residual_ns);

  // VCLOCK measurement Welford — bridge-interpolation self-test.
  // Sample in ns; mean == ppb under the "ns/sec == ppb" identity.
  if (g_vclock_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_vclock, (double)g_vclock_measurement.second_residual_ns);
  }

  // Step D/E OCXO Welfords and servo inputs use
  // PPS-founded public clock residuals.  The residual is computed from
  // consecutive TIMEBASE clock values:
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
  }
  if (pps_residuals.ocxo2_valid) {
    welford_update(welford_ocxo2,
                   (double)pps_residuals.ocxo2_fast_residual_ns);
  }

  // PPS witness statistics are owned by the witness/PPS_PHASE path.
  // Beta publishes the accumulator but does not update it here.

  const bool ocxo1_total_slope_valid = ocxo1_pps_projected_valid &&
      public_gnss_ns != 0 && public_ocxo1_ns != 0;
  const bool ocxo2_total_slope_valid = ocxo2_pps_projected_valid &&
      public_gnss_ns != 0 && public_ocxo2_ns != 0;
  const double ocxo1_total_tau = servo_total_tau_from_public(
      public_gnss_ns, public_ocxo1_ns);
  const double ocxo2_total_tau = servo_total_tau_from_public(
      public_gnss_ns, public_ocxo2_ns);

  servo_input_diag_update(g_servo_input_ocxo1,
                          pps_residuals.ocxo1_valid,
                          pps_residuals.gnss_interval_ns,
                          pps_residuals.ocxo1_interval_ns,
                          pps_residuals.ocxo1_fast_residual_ns,
                          welford_ocxo1,
                          ocxo1_total_slope_valid,
                          ocxo1_total_tau);
  servo_input_diag_update(g_servo_input_ocxo2,
                          pps_residuals.ocxo2_valid,
                          pps_residuals.gnss_interval_ns,
                          pps_residuals.ocxo2_interval_ns,
                          pps_residuals.ocxo2_fast_residual_ns,
                          welford_ocxo2,
                          ocxo2_total_slope_valid,
                          ocxo2_total_tau);

  timebase_build_stage(TIMEBASE_BUILD_STAGE_WELFORD);

  // Servo runs AFTER PPS-founded residual/Welford updates so it sees this
  // second's public TIMEBASE residual sample.
  ocxo_calibration_servo();
  timebase_build_stage(TIMEBASE_BUILD_STAGE_SERVO);

  // DAC Welfords — track servo effort (the TEMPEST signal).
  welford_update(welford_ocxo1_dac, ocxo1_dac.dac_fractional);
  welford_update(welford_ocxo2_dac, ocxo2_dac.dac_fractional);
  timebase_build_stage(TIMEBASE_BUILD_STAGE_DAC_WELFORD);

  // ── Build TIMEBASE_FRAGMENT ──
  g_timebase_build_begin_count++;
  g_timebase_last_build_begin_campaign_seconds = campaign_seconds;
  g_timebase_last_public_count = public_count;
  g_timebase_last_public_gnss_ns = public_gnss_ns;
  g_timebase_last_public_dwt_total = public_dwt_total;
  g_timebase_last_public_ocxo1_ns = public_ocxo1_ns;
  g_timebase_last_public_ocxo2_ns = public_ocxo2_ns;
  g_timebase_last_ocxo1_pps_projected = ocxo1_pps_projected_valid;
  g_timebase_last_ocxo2_pps_projected = ocxo2_pps_projected_valid;
  g_timebase_last_ocxo1_pps_residual_valid = pps_residuals.ocxo1_valid;
  g_timebase_last_ocxo2_pps_residual_valid = pps_residuals.ocxo2_valid;
  timebase_build_stage(TIMEBASE_BUILD_STAGE_BUILD_BEGIN);

  {
    Payload p;
    payload_add_timebase_pair_identity(p,
                                       "TIMEBASE_FRAGMENT_V3",
                                       "fragment_version",
                                       3U,
                                       "fragment",
                                       public_count,
                                       public_gnss_ns);
    p.add("paired_forensics_topic", "TIMEBASE_FORENSICS");
    p.add("paired_forensics_schema", "TIMEBASE_FORENSICS_V1");

    // Minimal flat compatibility spine.  TIMEBASE stores this fragment as an
    // opaque object, but Pi-side indexing and older tools still need these few
    // top-level identities while reports migrate to the paired schema.
    timebase_build_stage(TIMEBASE_BUILD_STAGE_SPINE);

    {
      timebase_build_stage(TIMEBASE_BUILD_STAGE_GNSS);
      Payload gnss;
      gnss.add("ns", public_gnss_ns);
      gnss.add("ocxo1_ns", public_ocxo1_ns);
      gnss.add("ocxo2_ns", public_ocxo2_ns);
      p.add_object("gnss", gnss);
    }

    {
      timebase_build_stage(TIMEBASE_BUILD_STAGE_DWT);
      Payload dwt;
      // DWT is a first-class clock, but its native ledger unit is CPU cycles,
      // not nanoseconds.  Keep a generic value/unit pair for dashboards while
      // retaining the explicit cycle_count_total name for durable tools.
      dwt.add("value", public_dwt_total);
      dwt.add("unit", "cycles");
      dwt.add("cycles", public_dwt_total);
      dwt.add("cycle_count_total", public_dwt_total);
      dwt.add("cycles_between_pps_vclock", (uint32_t)g_dwt_cycles_between_pps_vclock);
      dwt.add("second_residual_cycles",
              (int32_t)((int64_t)g_dwt_cycles_between_pps_vclock -
                        (int64_t)DWT_EXPECTED_PER_PPS));
      p.add_object("dwt", dwt);
    }

    {
      timebase_build_stage(TIMEBASE_BUILD_STAGE_PPS);
      Payload pps;
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
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PREDICTION);
    payload_add_prediction_summary_hierarchical(p);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_VCLOCK);
    payload_add_vclock_fragment(p, public_gnss_ns);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO1);
    payload_add_ocxo_fragment(p,
                              "ocxo1",
                              public_ocxo1_ns,
                              public_ocxo1_measured_ns,
                              ocxo1_pps_projected_valid,
                              ocxo1_pps_projection.source,
                              pps_residuals.ocxo1_valid,
                              pps_residuals.gnss_interval_ns,
                              pps_residuals.ocxo1_interval_ns,
                              pps_residuals.ocxo1_fast_residual_ns);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO2);
    payload_add_ocxo_fragment(p,
                              "ocxo2",
                              public_ocxo2_ns,
                              public_ocxo2_measured_ns,
                              ocxo2_pps_projected_valid,
                              ocxo2_pps_projection.source,
                              pps_residuals.ocxo2_valid,
                              pps_residuals.gnss_interval_ns,
                              pps_residuals.ocxo2_interval_ns,
                              pps_residuals.ocxo2_fast_residual_ns);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_STATS);
    payload_add_stats_summary_hierarchical(p);

    // System DAC persistence feed.  The Pi should update the simplified
    // system config only while the servo is actively tuning; manual/static DAC
    // values are loaded at START/SET_DAC but are not re-persisted every second.
    if (clocks_servo_active()) {
      payload_add_servo_dac_values(p);
    }

    g_timebase_build_complete_count++;
    g_timebase_last_build_complete_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_BUILD_COMPLETE);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_ASSIGN_LAST_FRAGMENT);
    g_last_fragment = p;
    g_timebase_assign_last_fragment_count++;
    g_timebase_last_assign_campaign_seconds = campaign_seconds;

    g_timebase_publish_attempt_count++;
    g_timebase_last_publish_attempt_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PUBLISH_ATTEMPT);
    publish("TIMEBASE_FRAGMENT", p);

    g_timebase_publish_return_count++;
    g_timebase_last_publish_return_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PUBLISH_RETURN);
  }

  {
    g_timebase_forensics_build_begin_count++;
    g_timebase_last_forensics_build_begin_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_BEGIN);

    Payload f;
    payload_add_timebase_pair_identity(f,
                                       "TIMEBASE_FORENSICS_V1",
                                       "forensics_version",
                                       1U,
                                       "forensics",
                                       public_count,
                                       public_gnss_ns);
    f.add("paired_fragment_topic", "TIMEBASE_FRAGMENT");
    f.add("paired_fragment_schema", "TIMEBASE_FRAGMENT_V3");
    f.add("paired_fragment_version", 3U);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_VCLOCK);
    payload_add_vclock_forensics(f,
                                 public_gnss_ns,
                                 vclock_forensics_valid,
                                 vclock_forensics);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO1);
    payload_add_ocxo_forensics(f,
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
                               pps_residuals.ocxo1_fast_residual_ns,
                               ocxo1_cycle_residual_diag);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO2);
    payload_add_ocxo_forensics(f,
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
                               pps_residuals.ocxo2_fast_residual_ns,
                               ocxo2_cycle_residual_diag);

    g_timebase_forensics_build_complete_count++;
    g_timebase_last_forensics_build_complete_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_COMPLETE);

    g_timebase_forensics_publish_attempt_count++;
    g_timebase_last_forensics_publish_attempt_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_ATTEMPT);
    publish("TIMEBASE_FORENSICS", f);

    g_timebase_forensics_publish_return_count++;
    g_timebase_last_forensics_publish_return_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_RETURN);
  }
}

// ============================================================================
// Commands
// ============================================================================


static bool payload_try_get_double_alias(const Payload& args,
                                         double& out,
                                         const char* k1,
                                         const char* k2,
                                         const char* k3) {
  return (k1 && args.tryGetDouble(k1, out)) ||
         (k2 && args.tryGetDouble(k2, out)) ||
         (k3 && args.tryGetDouble(k3, out));
}

static bool payload_try_get_ocxo1_dac(const Payload& args, double& out) {
  // New system-config contract: { "ocxo1_dac": <code>, "ocxo2_dac": <code> }.
  // Retain the old command aliases so existing Pi-side callers do not break.
  return payload_try_get_double_alias(args, out,
                                      "ocxo1_dac",
                                      "dac1",
                                      "set_dac1");
}

static bool payload_try_get_ocxo2_dac(const Payload& args, double& out) {
  return payload_try_get_double_alias(args, out,
                                      "ocxo2_dac",
                                      "dac2",
                                      "set_dac2");
}

static FLASHMEM Payload cmd_start(const Payload& args) {
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
  if (payload_try_get_ocxo1_dac(args, dac_val)) {
    dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
  }
  if (payload_try_get_ocxo2_dac(args, dac_val)) {
    dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);
  }

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

static FLASHMEM Payload cmd_stop(const Payload&) {
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


static FLASHMEM Payload cmd_zero(const Payload&) {
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

static FLASHMEM Payload cmd_recover(const Payload& args) {
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

static FLASHMEM Payload cmd_watchdog_test(const Payload&) {
  clocks_watchdog_anomaly("watchdog_test");
  Payload p;
  p.add("status", "watchdog_anomaly_requested");
  return p;
}


// ============================================================================
// CLOCKS report forensics
// ============================================================================

static FLASHMEM int64_t signed_delta_u64(uint64_t a, uint64_t b) {
  return (a >= b) ? (int64_t)(a - b) : -(int64_t)(b - a);
}

static FLASHMEM uint32_t apparent_cps_from_projection(uint32_t elapsed_cycles,
                                             uint64_t projected_ns) {
  if (projected_ns == 0) return 0;
  const uint64_t cps =
      ((uint64_t)elapsed_cycles * 1000000000ULL + projected_ns / 2ULL) /
      projected_ns;
  return (cps > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)cps;
}

static FLASHMEM double ppb_fast_vs_vclock(uint32_t vclock_cycles,
                                 uint32_t lane_cycles) {
  if (vclock_cycles == 0 || lane_cycles == 0) return 0.0;
  return (((double)vclock_cycles - (double)lane_cycles) /
          (double)vclock_cycles) * 1.0e9;
}

static FLASHMEM void add_alpha_event_payload(Payload& p,
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

  Payload dwt_interval_adjacency;
  dwt_interval_adjacency.add("valid", f.dwt_interval_adjacency_gate_valid);
  dwt_interval_adjacency.add("ok", f.dwt_interval_adjacency_ok);
  dwt_interval_adjacency.add("rejected", f.dwt_interval_adjacency_rejected);
  dwt_interval_adjacency.add("counter_delta_ticks",
                             f.dwt_interval_counter_delta_ticks);
  dwt_interval_adjacency.add("expected_counter_delta_ticks",
                             f.dwt_interval_expected_counter_delta_ticks);
  dwt_interval_adjacency.add("reject_count",
                             f.dwt_interval_adjacency_reject_count);
  p.add_object("dwt_interval_adjacency", dwt_interval_adjacency);

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

  Payload spincatch;
  spincatch.add("valid", f.valid && f.spinidle_shadow_valid);
  spincatch.add("shadow_dwt", f.valid ? f.spinidle_shadow_dwt : 0U);
  spincatch.add("shadow_to_isr_entry_cycles",
                 f.valid ? f.spinidle_shadow_to_isr_entry_cycles : 0U);
  spincatch.add("shadow_valid_threshold_cycles",
                 f.valid ? f.spinidle_shadow_valid_threshold_cycles : 0U);
  spincatch.add("source", "SPINIDLE_ISR_WITNESS");
  p.add_object("spincatch", spincatch);

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

static FLASHMEM void add_projection_payload(Payload& p,
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

static FLASHMEM void add_clock_forensics_payload(Payload& p,
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


static FLASHMEM void add_single_clock_forensics_payload(Payload& p,
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

static FLASHMEM void add_summary_payload(Payload& p) {
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

static FLASHMEM void add_campaign_payload(Payload& p) {
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

static FLASHMEM void add_epoch_payload(Payload& p) {
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

static FLASHMEM void add_compact_smartzero_status(Payload& p) {
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

static FLASHMEM void add_dac_payload(Payload& p) {
  Payload o1;
  o1.add("dac", ocxo1_dac.dac_fractional);
  o1.add("dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  o1.add("dac_min", ocxo1_dac.dac_min);
  o1.add("dac_max", ocxo1_dac.dac_max);
  o1.add("servo_last_step", ocxo1_dac.servo_last_step, 6);
  o1.add("servo_last_residual", ocxo1_dac.servo_last_residual, 6);
  o1.add("servo_last_ppb", ocxo1_dac.servo_last_residual, 6);
  o1.add("servo_settle_count", ocxo1_dac.servo_settle_count);
  o1.add("servo_adjustments", ocxo1_dac.servo_adjustments);
  o1.add("servo_predictor_initialized", ocxo1_dac.servo_predictor_initialized);
  o1.add("servo_filtered_residual", ocxo1_dac.servo_filtered_residual, 6);
  o1.add("servo_filtered_ppb", ocxo1_dac.servo_filtered_residual, 6);
  o1.add("servo_filtered_slope", ocxo1_dac.servo_filtered_slope, 6);
  o1.add("servo_filtered_ppb_delta", ocxo1_dac.servo_filtered_slope, 6);
  o1.add("servo_predicted_residual", ocxo1_dac.servo_predicted_residual, 6);
  o1.add("servo_control_ppb", ocxo1_dac.servo_predicted_residual, 6);
  o1.add("servo_predictor_updates", ocxo1_dac.servo_predictor_updates);
  payload_add_servo_input_diag(o1, g_servo_input_ocxo1);
  payload_add_dac_dither_object(o1, "dither", ocxo1_dac);
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
  o2.add("servo_last_ppb", ocxo2_dac.servo_last_residual, 6);
  o2.add("servo_settle_count", ocxo2_dac.servo_settle_count);
  o2.add("servo_adjustments", ocxo2_dac.servo_adjustments);
  o2.add("servo_predictor_initialized", ocxo2_dac.servo_predictor_initialized);
  o2.add("servo_filtered_residual", ocxo2_dac.servo_filtered_residual, 6);
  o2.add("servo_filtered_ppb", ocxo2_dac.servo_filtered_residual, 6);
  o2.add("servo_filtered_slope", ocxo2_dac.servo_filtered_slope, 6);
  o2.add("servo_filtered_ppb_delta", ocxo2_dac.servo_filtered_slope, 6);
  o2.add("servo_predicted_residual", ocxo2_dac.servo_predicted_residual, 6);
  o2.add("servo_control_ppb", ocxo2_dac.servo_predicted_residual, 6);
  o2.add("servo_predictor_updates", ocxo2_dac.servo_predictor_updates);
  payload_add_servo_input_diag(o2, g_servo_input_ocxo2);
  payload_add_dac_dither_object(o2, "dither", ocxo2_dac);
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
  p.add("dither_handle", g_ocxo_dac_dither_handle);
  p.add("dither_mode", OCXO_DAC_REALIZATION_MODE);
  p.add("dither_rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("dither_period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  p.add("dither_grid_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("dither_cell_count", OCXO_DAC_SYNCDAC_CELL_COUNT);
  p.add("dither_cell_ticks", OCXO_DAC_SYNCDAC_CELL_TICKS);
  p.add("dither_max_writes_per_frame", OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME);
  p.add("dither_arm_count", g_ocxo_dac_dither_arm_count);
  p.add("dither_arm_fail_count", g_ocxo_dac_dither_arm_fail_count);
  p.add("commit_scheduled", g_ocxo_dac_commit_scheduled);
  p.add("last_schedule_second", g_ocxo_dac_last_schedule_second);
  p.add("last_commit_second", g_ocxo_dac_last_commit_second);
  p.add("last_winner", (uint32_t)g_ocxo_dac_last_winner);
  p.add("arbitration_passes", g_ocxo_dac_arbitration_passes);
  p.add("no_candidate_passes", g_ocxo_dac_no_candidate_passes);
  p.add("deferred_candidates", g_ocxo_dac_deferred_candidates);
  p.add("schedule_failures", g_ocxo_dac_schedule_failures);
}


static FLASHMEM const char* alpha_flow_stage_name_beta(uint32_t stage) {
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

static FLASHMEM void payload_add_alpha_flow_lane(Payload& parent,
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

static FLASHMEM const char* ocxo_pps_projection_source_name(uint32_t source) {
  switch (source) {
    case 1: return "ACTUAL_BRACKET";
    case 2: return "STATIC_NEXT_EDGE";
    default: return "NONE";
  }
}

static FLASHMEM const char* ocxo_pps_projection_invalid_reason_name(uint32_t reason) {
  switch (reason) {
    case 1: return "NO_EDGE";
    case 2: return "NO_INTERVAL";
    case 3: return "TARGET_OUT_OF_WINDOW";
    default: return "NONE";
  }
}

static FLASHMEM void payload_add_ocxo_pps_projection_lane(Payload& parent,
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
  counters.add("static_projection_advance_count",
               s.static_projection_advance_count);
  counters.add("last_static_projection_advance_count",
               s.last_static_projection_advance_count);
  counters.add("max_static_projection_advance_count",
               s.max_static_projection_advance_count);
  counters.add("max_target_overrun_cycles", s.max_target_overrun_cycles);
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
  projection.add("target_delta_raw_cycles", s.target_delta_raw_cycles);
  projection.add("target_overrun_cycles", s.target_overrun_cycles);
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
  static_prediction.add("advance_limit", s.static_projection_advance_limit);
  static_prediction.add("last_advance_count",
                        s.last_static_projection_advance_count);
  lane.add_object("static_prediction", static_prediction);

  parent.add_object(key, lane);
}

static FLASHMEM Payload cmd_report_ocxo_pps_projection(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_OCXO_PPS_PROJECTION");
  p.add("description",
        "Report-only Alpha projection of OCXO clock ns to the PPS/VCLOCK DWT edge; not TIMEBASE authority yet");
  payload_add_ocxo_pps_projection_lane(p, "ocxo1", time_clock_id_t::OCXO1);
  payload_add_ocxo_pps_projection_lane(p, "ocxo2", time_clock_id_t::OCXO2);
  return p;
}

static FLASHMEM Payload cmd_report_timebase_publish(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_TIMEBASE_PUBLISH");
  p.add("description", "Report-only TIMEBASE_FRAGMENT / TIMEBASE_FORENSICS build/publish flight recorder");

  Payload counters;
  counters.add("pps_entry_count", g_timebase_pps_entry_count);
  counters.add("candidate_count", g_timebase_candidate_count);
  counters.add("per_second_count", g_timebase_per_second_count);
  counters.add("build_begin_count", g_timebase_build_begin_count);
  counters.add("build_complete_count", g_timebase_build_complete_count);
  counters.add("assign_last_fragment_count", g_timebase_assign_last_fragment_count);
  counters.add("publish_attempt_count", g_timebase_publish_attempt_count);
  counters.add("publish_return_count", g_timebase_publish_return_count);
  counters.add("forensics_build_begin_count", g_timebase_forensics_build_begin_count);
  counters.add("forensics_build_complete_count", g_timebase_forensics_build_complete_count);
  counters.add("forensics_publish_attempt_count", g_timebase_forensics_publish_attempt_count);
  counters.add("forensics_publish_return_count", g_timebase_forensics_publish_return_count);
  p.add_object("counters", counters);

  Payload gates;
  gates.add("stop_gate_count", g_timebase_stop_gate_count);
  gates.add("start_zero_gate_count", g_timebase_start_zero_gate_count);
  gates.add("recover_gate_count", g_timebase_recover_gate_count);
  gates.add("watchdog_gate_count", g_timebase_watchdog_gate_count);
  gates.add("not_started_gate_count", g_timebase_not_started_gate_count);
  gates.add("warmup_suppressed_count", g_timebase_warmup_suppressed_count);
  gates.add("campaign_state", campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  gates.add("warmup_active", campaign_warmup_active());
  gates.add("watchdog_anomaly_active", watchdog_anomaly_active);
  gates.add("request_start", request_start);
  gates.add("request_stop", request_stop);
  gates.add("request_recover", request_recover);
  gates.add("request_zero", request_zero);
  p.add_object("gates", gates);

  Payload last;
  last.add("stage", g_timebase_last_stage);
  last.add("stage_name", timebase_build_stage_name(g_timebase_last_stage));
  last.add("entry_campaign_seconds", g_timebase_last_entry_campaign_seconds);
  last.add("candidate_campaign_seconds", g_timebase_last_candidate_campaign_seconds);
  last.add("per_second_campaign_seconds", g_timebase_last_per_second_campaign_seconds);
  last.add("build_begin_campaign_seconds", g_timebase_last_build_begin_campaign_seconds);
  last.add("build_complete_campaign_seconds", g_timebase_last_build_complete_campaign_seconds);
  last.add("assign_campaign_seconds", g_timebase_last_assign_campaign_seconds);
  last.add("publish_attempt_campaign_seconds", g_timebase_last_publish_attempt_campaign_seconds);
  last.add("publish_return_campaign_seconds", g_timebase_last_publish_return_campaign_seconds);
  last.add("forensics_build_begin_campaign_seconds", g_timebase_last_forensics_build_begin_campaign_seconds);
  last.add("forensics_build_complete_campaign_seconds", g_timebase_last_forensics_build_complete_campaign_seconds);
  last.add("forensics_publish_attempt_campaign_seconds", g_timebase_last_forensics_publish_attempt_campaign_seconds);
  last.add("forensics_publish_return_campaign_seconds", g_timebase_last_forensics_publish_return_campaign_seconds);
  last.add("public_count", g_timebase_last_public_count);
  last.add("public_gnss_ns", g_timebase_last_public_gnss_ns);
  last.add("public_dwt_total", g_timebase_last_public_dwt_total);
  last.add("public_ocxo1_ns", g_timebase_last_public_ocxo1_ns);
  last.add("public_ocxo2_ns", g_timebase_last_public_ocxo2_ns);
  last.add("ocxo1_pps_projected", g_timebase_last_ocxo1_pps_projected);
  last.add("ocxo2_pps_projected", g_timebase_last_ocxo2_pps_projected);
  last.add("ocxo1_pps_residual_valid", g_timebase_last_ocxo1_pps_residual_valid);
  last.add("ocxo2_pps_residual_valid", g_timebase_last_ocxo2_pps_residual_valid);
  p.add_object("last", last);

  Payload gaps;
  gaps.add("candidate_minus_build_begin", (int64_t)g_timebase_candidate_count - (int64_t)g_timebase_build_begin_count);
  gaps.add("build_begin_minus_build_complete", (int64_t)g_timebase_build_begin_count - (int64_t)g_timebase_build_complete_count);
  gaps.add("build_complete_minus_assign", (int64_t)g_timebase_build_complete_count - (int64_t)g_timebase_assign_last_fragment_count);
  gaps.add("assign_minus_publish_attempt", (int64_t)g_timebase_assign_last_fragment_count - (int64_t)g_timebase_publish_attempt_count);
  gaps.add("publish_attempt_minus_return", (int64_t)g_timebase_publish_attempt_count - (int64_t)g_timebase_publish_return_count);
  gaps.add("publish_return_lag_vs_campaign_seconds", (int64_t)campaign_seconds - (int64_t)g_timebase_last_publish_return_campaign_seconds);
  gaps.add("publish_tail_returning", g_timebase_publish_attempt_count == g_timebase_publish_return_count);
  gaps.add("forensics_build_complete_minus_publish_attempt",
           (int64_t)g_timebase_forensics_build_complete_count -
           (int64_t)g_timebase_forensics_publish_attempt_count);
  gaps.add("forensics_publish_attempt_minus_return",
           (int64_t)g_timebase_forensics_publish_attempt_count -
           (int64_t)g_timebase_forensics_publish_return_count);
  gaps.add("forensics_publish_tail_returning",
           g_timebase_forensics_publish_attempt_count ==
           g_timebase_forensics_publish_return_count);
  p.add_object("gaps", gaps);

  return p;
}

static FLASHMEM Payload cmd_report(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_COMPACT");
  p.add("subreports", "REPORT_STATUS REPORT_SUMMARY REPORT_EPOCH REPORT_SMARTZERO REPORT_INSTALLED_SMARTZERO REPORT_LIVE_SMARTZERO REPORT_FORENSICS REPORT_FORENSICS_VCLOCK REPORT_FORENSICS_OCXO1 REPORT_FORENSICS_OCXO2 REPORT_OCXO_PPS_PROJECTION REPORT_TIMEBASE_PUBLISH REPORT_ALPHA_FLOW REPORT_ALPHA_FLOW_VCLOCK REPORT_ALPHA_FLOW_OCXO1 REPORT_ALPHA_FLOW_OCXO2 REPORT_PREDICTION REPORT_STATS REPORT_DAC");
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

static FLASHMEM Payload cmd_report_status(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_STATUS");
  add_campaign_payload(p);
  add_epoch_payload(p);
  add_compact_smartzero_status(p);
  return p;
}

static FLASHMEM Payload cmd_report_summary(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_SUMMARY");
  add_summary_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_epoch(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_EPOCH");
  add_epoch_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_smartzero(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_SMARTZERO");
  payload_add_smartzero_summary(p);
  return p;
}

static FLASHMEM Payload cmd_report_installed_smartzero(const Payload&) {
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

static FLASHMEM Payload cmd_report_live_smartzero(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_LIVE_SMARTZERO");
  interrupt_smartzero_snapshot_t live{};
  (void)interrupt_smartzero_live_snapshot(&live);
  payload_add_smartzero_snapshot_object(p, "live_smartzero", live, true, true);
  return p;
}

static FLASHMEM Payload cmd_report_forensics(const Payload&) {
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

static FLASHMEM Payload cmd_report_forensics_vclock(const Payload&) {
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

static FLASHMEM Payload cmd_report_forensics_ocxo1(const Payload&) {
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

static FLASHMEM Payload cmd_report_forensics_ocxo2(const Payload&) {
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


static FLASHMEM Payload cmd_report_alpha_flow(const Payload&) {
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

static FLASHMEM Payload cmd_report_alpha_flow_vclock(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_VCLOCK");
  payload_add_alpha_flow_lane(p, "vclock", time_clock_id_t::VCLOCK, true);
  return p;
}

static FLASHMEM Payload cmd_report_alpha_flow_ocxo1(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_OCXO1");
  payload_add_alpha_flow_lane(p, "ocxo1", time_clock_id_t::OCXO1, true);
  return p;
}

static FLASHMEM Payload cmd_report_alpha_flow_ocxo2(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_ALPHA_FLOW_OCXO2");
  payload_add_alpha_flow_lane(p, "ocxo2", time_clock_id_t::OCXO2, true);
  return p;
}

static FLASHMEM Payload cmd_report_prediction(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_PREDICTION");
  payload_add_prediction_summary(p);
  return p;
}

static FLASHMEM Payload cmd_report_stats(const Payload&) {
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

static FLASHMEM Payload cmd_report_dac(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_DAC");
  add_dac_payload(p);
  return p;
}

static FLASHMEM Payload cmd_dither(const Payload& args) {
  bool enabled = true;
  (void)args.tryGetBool("dither", enabled);

  ocxo1_dac.dither_enabled = enabled;
  ocxo2_dac.dither_enabled = enabled;

  if (g_ocxo_dac_dither_handle == TIMEPOP_INVALID_HANDLE) {
    clocks_dac_dither_begin();
  }

  Payload p;
  p.add("status", "ok");
  p.add("dither", enabled);
  p.add("mode", OCXO_DAC_REALIZATION_MODE);
  p.add("rate_fixed", true);
  p.add("rate_args_ignored", true);
  p.add("execution_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  p.add("grid_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("frame_ticks", OCXO_DAC_SYNCDAC_FRAME_TICKS);
  p.add("cell_count", OCXO_DAC_SYNCDAC_CELL_COUNT);
  p.add("cell_ticks", OCXO_DAC_SYNCDAC_CELL_TICKS);
  p.add("max_writes_per_frame", OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME);
  p.add("handle", g_ocxo_dac_dither_handle);
  p.add("arm_count", g_ocxo_dac_dither_arm_count);
  p.add("arm_fail_count", g_ocxo_dac_dither_arm_fail_count);
  return p;
}

static FLASHMEM Payload cmd_set_dac(const Payload& args) {
  ocxo_dac_pacing_abort_all();

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (payload_try_get_ocxo1_dac(args, dac_val)) {
    dac1_ok = ocxo_dac_set_desired(ocxo1_dac, dac_val);
    ocxo_dac_dither_reset(ocxo1_dac);
    if (dac1_ok) ocxo_dac_retry_reset(ocxo1_dac);
  }
  if (payload_try_get_ocxo2_dac(args, dac_val)) {
    dac2_ok = ocxo_dac_set_desired(ocxo2_dac, dac_val);
    ocxo_dac_dither_reset(ocxo2_dac);
    if (dac2_ok) ocxo_dac_retry_reset(ocxo2_dac);
  }

  Payload p;
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("ocxo1_dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  p.add("dither_realizes_desired", true);
  p.add("dither_mode", OCXO_DAC_REALIZATION_MODE);
  p.add("dither_rate_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("dither_period_ns", OCXO_DAC_SYNCDAC_PERIOD_NS);
  p.add("dither_grid_hz", OCXO_DAC_SYNCDAC_EXECUTION_HZ);
  p.add("dither_cell_count", OCXO_DAC_SYNCDAC_CELL_COUNT);
  p.add("dither_cell_ticks", OCXO_DAC_SYNCDAC_CELL_TICKS);
  p.add("dither_max_writes_per_frame", OCXO_DAC_SYNCDAC_MAX_WRITES_PER_FRAME);
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
  { "REPORT_TIMEBASE_PUBLISH", cmd_report_timebase_publish },
  { "REPORT_ALPHA_FLOW",       cmd_report_alpha_flow       },
  { "REPORT_ALPHA_FLOW_VCLOCK", cmd_report_alpha_flow_vclock },
  { "REPORT_ALPHA_FLOW_OCXO1",  cmd_report_alpha_flow_ocxo1  },
  { "REPORT_ALPHA_FLOW_OCXO2",  cmd_report_alpha_flow_ocxo2  },
  { "REPORT_PREDICTION",       cmd_report_prediction       },
  { "REPORT_STATS",      cmd_report_stats      },
  { "REPORT_DAC",        cmd_report_dac        },
  { "DITHER",            cmd_dither            },
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