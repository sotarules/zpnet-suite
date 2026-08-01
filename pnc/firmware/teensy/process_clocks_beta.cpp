// ============================================================================
// process_clocks_beta.cpp — Campaign Layer
// ============================================================================
//
// Statistical surface doctrine:
//
//   Teensy owns every statistical quantity exposed in the completed campaign
//   observation. The Pi transcribes what the Teensy says — it does not recompute.
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
//   Eight published Welford prefixes:
//
//     gnss_welford        — exact GNSS reference residual (ns, always 0)
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
// Campaign lifecycle is now recording-only with respect to clock statistics.
// Alpha owns boot-lifetime PPB/TAU/Welford state; Beta snapshots it for the
// completed campaign observation and command reports. Servo DAC updates remain
// planned from the 1 Hz science path but physically committed by a low-priority
// actuator sandbox so I2C failures cannot poison campaign science. Servo inputs
// consume the same PPS-founded OCXO residual surface that feeds the OCXO Welfords.
//
// Beta stages one typed completed observation per public campaign second. It
// carries canonical clocks, cycle evidence, residuals, Welfords, and conditional
// rejection/recovery testimony. START may privately
// acquire a PPS0 bookend before public campaign time begins; after public PPS1
// is released, every campaign identity is emitted continuously.
//
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_interrupt.h"
#include "process_system.h"

#include "debug.h"
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
#include <errno.h>
#include <stdio.h>
#include <climits>
#include <string.h>
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

// FLASH_CUT is a hot campaign boundary: preserve the installed Alpha/service
// epoch and all learned instrument statistics, change the campaign identity,
// and rebase only Beta's campaign-public clock presentation at the next
// PPS/VCLOCK edge.
static volatile bool request_flash_cut = false;
static char flash_cut_campaign_name[64] = {0};

uint64_t recover_dwt_ns   = 0;
uint64_t recover_gnss_ns  = 0;
uint64_t recover_ocxo1_ns = 0;
uint64_t recover_ocxo2_ns = 0;

// Beta owns campaign/science adjudication and freezes at most one typed completed
// observation for SYSTEM. It does not construct a transport payload or know the
// MONITOR_FRAGMENT wire schema. SYSTEM consumes this handoff and owns serialization.

// Alpha-authored physical PPS witness DWT audit surface. These facts enter the
// typed handoff so SYSTEM can expose physical PPS-to-PPS DWT intervals beside the
// canonical PPS/VCLOCK DWT rail.
extern volatile uint32_t g_pps_dwt_at_edge;
extern volatile uint32_t g_pps_dwt_cycles_between_edges;
extern volatile bool     g_pps_dwt_cycles_between_edges_valid;

// Species-pure observed PPS/VCLOCK edge-to-edge interval. Delta Cycles uses
// this rail, not the smoother PPS/GPIO DWT-GNSS calibration, so the reference
// interval is formed by the same observed DWT-at-edge subtraction doctrine as
// OCXO.
extern volatile uint32_t g_pps_vclock_dwt_cycles_between_edges;
extern volatile bool     g_pps_vclock_dwt_cycles_between_edges_valid;


// ============================================================================
// Completed campaign-observation handoff liveness
// ============================================================================
//
// Keep only the minimal stage, candidate, and last-public identity required by
// REPORT_RECOVERY. Detailed publication flight-recorder reports are retired.

// The typed campaign observation is the compact canonical campaign row. It carries
// clocks, observed cycle intervals, residuals, Welfords, and only the integrated
// ISR-delay verdict needed by raw_cycles.  Deep Interrupt/Alpha transcripts,
// serialized CounterLedger state, duplicate predictions, and compatibility
// aliases are deliberately not transported at 1 Hz.
static constexpr uint32_t CAMPAIGN_RECORD_ISR_DELAY_EXPLANATION_GATE_CYCLES = 16U;

static constexpr uint32_t CAMPAIGN_RECORD_INVALID_OCXO1_CUSTODY =
    CLOCKS_ROW_LANE_OCXO1;
static constexpr uint32_t CAMPAIGN_RECORD_INVALID_OCXO2_CUSTODY =
    CLOCKS_ROW_LANE_OCXO2;

// Row objections are the only survivable-science disposition.  They are
// authored as soon as a layer has enough evidence, consulted by Alpha before
// Welford/TAU mutation, consumed by Beta at the completed-row boundary, and
// always serialized for TIMEBASE.  There is no runtime gate mode.

// Defined with the watchdog/recovery state later in this translation unit.
bool clocks_watchdog_campaign_armed(void);
static uint32_t clocks_row_lifecycle_science_hold_flags(void);

// Cross-layer row-objection latch. State is an atomic three-state word:
//   0 = idle, 1 = writer owns the record, 2 = complete record awaits Beta.
// The first objection is the primary human explanation. Later objections add
// to the count and lane mask without replacing the original testimony.
struct clocks_row_objection_record_t {
  clocks_row_objection_source_t source = clocks_row_objection_source_t::NONE;
  clocks_row_objection_reason_t reason = clocks_row_objection_reason_t::NONE;
  uint32_t lane = 0U;
  uint32_t lane_mask = 0U;
  uint32_t objection_count = 0U;
  uint32_t detail0 = 0U;
  uint32_t detail1 = 0U;
  uint32_t detail2 = 0U;
  uint32_t detail3 = 0U;
};

static volatile uint32_t g_row_objection_latch_state = 0U;
static clocks_row_objection_record_t g_row_objection_latch DMAMEM = {};
static volatile uint32_t g_row_objection_pending_count = 0U;
static volatile uint32_t g_row_objection_pending_lane_mask = 0U;
static volatile uint32_t g_row_objection_raise_count = 0U;
static volatile uint32_t g_row_objection_coalesced_count = 0U;
static uint32_t g_row_objection_consumed_count = 0U;
static uint32_t g_row_objection_last_public_count = 0U;
static clocks_row_objection_record_t g_row_objection_last DMAMEM = {};

// A rejected endpoint participates in two adjacent intervals.  When Alpha
// excludes PPS X for a cycle/interval excursion, this one-shot holds PPS
// X+1 out of science so no accumulator or servo consumes an interval whose
// opening endpoint was already rejected.  TIMEBASE still persists the row.
static volatile uint32_t g_row_antecedent_hold_target_pps = 0U;
static volatile uint32_t g_row_antecedent_hold_source_pps = 0U;
static volatile uint32_t g_row_antecedent_hold_lane_mask = 0U;

// One-shot diagnostic injection. The command surface is intentionally open-ended
// (type=...), but only type=excursion is implemented initially. Alpha consumes
// an armed injection through the ordinary pre-statistics eligibility query, so
// the real SCIENCE_EXCLUDE path is exercised without falsifying counters,
// clockfaces, or interrupt custody.
// State: 0=idle, 1=command writer owns parameters, 2=armed for one completed row.
static volatile uint32_t g_problem_injection_armed = 0U;
static volatile uint32_t g_problem_injection_lane = CLOCKS_ROW_LANE_OCXO1;
static volatile uint32_t g_problem_injection_cycles = 1000U;
static volatile uint32_t g_problem_injection_arm_count = 0U;
static volatile uint32_t g_problem_injection_fire_count = 0U;

static bool clocks_row_objection_requires_antecedent_hold(
    clocks_row_objection_reason_t reason) {
  return reason == clocks_row_objection_reason_t::ALPHA_CYCLE_EXCURSION ||
      reason ==
          clocks_row_objection_reason_t::ALPHA_CYCLE_INTERVAL_IMPLAUSIBLE;
}

static void clocks_row_antecedent_hold_arm(
    clocks_row_objection_reason_t reason,
    uint32_t lane_mask,
    uint32_t source_pps) {
  if (!clocks_row_objection_requires_antecedent_hold(reason) ||
      source_pps == 0U) {
    return;
  }

  const uint32_t target_pps = source_pps + 1U;
  const uint32_t existing_target =
      __atomic_load_n(&g_row_antecedent_hold_target_pps,
                      __ATOMIC_ACQUIRE);
  if (existing_target == target_pps) {
    (void)__atomic_or_fetch(&g_row_antecedent_hold_lane_mask,
                            lane_mask,
                            __ATOMIC_RELAXED);
    return;
  }

  __atomic_store_n(&g_row_antecedent_hold_source_pps,
                   source_pps,
                   __ATOMIC_RELAXED);
  __atomic_store_n(&g_row_antecedent_hold_lane_mask,
                   lane_mask,
                   __ATOMIC_RELAXED);
  __atomic_store_n(&g_row_antecedent_hold_target_pps,
                   target_pps,
                   __ATOMIC_RELEASE);
}

static void clocks_row_antecedent_hold_apply(uint32_t pps_sequence) {
  if (pps_sequence == 0U) return;

  const uint32_t target_pps =
      __atomic_load_n(&g_row_antecedent_hold_target_pps,
                      __ATOMIC_ACQUIRE);
  if (target_pps == 0U) return;

  if (pps_sequence == target_pps) {
    uint32_t expected = target_pps;
    if (!__atomic_compare_exchange_n(&g_row_antecedent_hold_target_pps,
                                     &expected,
                                     0U,
                                     false,
                                     __ATOMIC_ACQ_REL,
                                     __ATOMIC_ACQUIRE)) {
      return;
    }

    const uint32_t source_pps =
        __atomic_load_n(&g_row_antecedent_hold_source_pps,
                        __ATOMIC_RELAXED);
    uint32_t lane_mask =
        __atomic_load_n(&g_row_antecedent_hold_lane_mask,
                        __ATOMIC_RELAXED);
    if (lane_mask == 0U) {
      lane_mask = CLOCKS_ROW_LANE_PPS | CLOCKS_ROW_LANE_VCLOCK |
          CLOCKS_ROW_LANE_OCXO1 | CLOCKS_ROW_LANE_OCXO2;
    }
    clocks_row_exclude(
        clocks_row_objection_source_t::BETA,
        clocks_row_objection_reason_t::BETA_ANTECEDENT_SCIENCE_HOLD,
        lane_mask,
        pps_sequence,
        source_pps,
        0U,
        0U);
    return;
  }

  // If publication jumped over the one adjacent identity, the suspect
  // interval was never presented and the one-shot no longer applies.
  if ((int32_t)(pps_sequence - target_pps) > 0) {
    uint32_t expected = target_pps;
    (void)__atomic_compare_exchange_n(&g_row_antecedent_hold_target_pps,
                                      &expected,
                                      0U,
                                      false,
                                      __ATOMIC_ACQ_REL,
                                      __ATOMIC_ACQUIRE);
  }
}

static const char* clocks_row_objection_source_name(
    clocks_row_objection_source_t source) {
  switch (source) {
    case clocks_row_objection_source_t::BETA: return "BETA";
    case clocks_row_objection_source_t::ALPHA: return "ALPHA";
    case clocks_row_objection_source_t::INTERRUPT: return "INTERRUPT";
    default: return "NONE";
  }
}

static const char* clocks_row_objection_reason_name(
    clocks_row_objection_reason_t reason) {
  switch (reason) {
    case clocks_row_objection_reason_t::BETA_OCXO_SCIENCE_CUSTODY:
      return "beta_ocxo_science_custody";
    case clocks_row_objection_reason_t::BETA_RECOVERY_SCIENCE_HOLD:
      return "beta_recovery_science_hold";
    case clocks_row_objection_reason_t::BETA_ANTECEDENT_SCIENCE_HOLD:
      return "beta_antecedent_science_hold";
    case clocks_row_objection_reason_t::ALPHA_COUNTERLEDGER_INTERVAL:
      return "alpha_counterledger_interval";
    case clocks_row_objection_reason_t::ALPHA_BRIDGE_NONMONOTONIC:
      return "alpha_bridge_nonmonotonic";
    case clocks_row_objection_reason_t::ALPHA_OCXO_PROJECTION_WINDOW:
      return "alpha_ocxo_projection_window";
    case clocks_row_objection_reason_t::ALPHA_OCXO_CLOCK_APPLY:
      return "alpha_ocxo_clock_apply";
    case clocks_row_objection_reason_t::ALPHA_COUNTERLEDGER_CAPTURE:
      return "alpha_counterledger_capture";
    case clocks_row_objection_reason_t::ALPHA_CYCLE_EXCURSION:
      return "alpha_cycle_excursion";
    case clocks_row_objection_reason_t::ALPHA_CYCLE_INTERVAL_IMPLAUSIBLE:
      return "alpha_cycle_interval_implausible";
    default: return "none";
  }
}

static void clocks_row_objection_format_reason(
    char* out,
    size_t out_size,
    const clocks_row_objection_record_t& objection) {
  if (!out || out_size == 0U) return;
  if (objection.reason == clocks_row_objection_reason_t::ALPHA_CYCLE_EXCURSION) {
    const bool injected =
        objection.source == clocks_row_objection_source_t::BETA;
    if (injected) {
      snprintf(out, out_size,
               "PPS %lu injected cycle excursion: lanes=0x%02lX "
               "residual span=%lu cycles exceeds gate=%lu; "
               "excluded from science/control",
               (unsigned long)objection.detail0,
               (unsigned long)objection.lane_mask,
               (unsigned long)objection.detail1,
               (unsigned long)objection.detail2);
    } else {
      snprintf(out, out_size,
               "PPS %lu cycle excursion: lanes=0x%02lX cycle span=%lu cycles "
               "exceeds gate=%lu (median=%ld); excluded from science/control",
               (unsigned long)objection.detail0,
               (unsigned long)objection.lane_mask,
               (unsigned long)objection.detail1,
               (unsigned long)objection.detail2,
               (long)(int32_t)objection.detail3);
    }
    return;
  }

  if (objection.reason ==
      clocks_row_objection_reason_t::ALPHA_CYCLE_INTERVAL_IMPLAUSIBLE) {
    snprintf(out, out_size,
             "PPS %lu implausible one-second interval: lanes=0x%02lX "
             "min=%lu max=%lu cycles; excluded from science/control",
             (unsigned long)objection.detail0,
             (unsigned long)objection.lane_mask,
             (unsigned long)objection.detail1,
             (unsigned long)objection.detail2);
    return;
  }

  if (objection.reason ==
      clocks_row_objection_reason_t::BETA_RECOVERY_SCIENCE_HOLD) {
    snprintf(out, out_size,
             "PPS %lu recovery science hold: lanes=0x%02lX flags=0x%02lX; "
             "persisted for audit and excluded from science/control",
             (unsigned long)objection.detail0,
             (unsigned long)objection.lane_mask,
             (unsigned long)objection.detail1);
    return;
  }

  if (objection.reason ==
      clocks_row_objection_reason_t::BETA_ANTECEDENT_SCIENCE_HOLD) {
    snprintf(out, out_size,
             "PPS %lu antecedent science hold: prior PPS %lu had a rejected "
             "endpoint in lanes=0x%02lX; persisted for audit and excluded "
             "from science/control",
             (unsigned long)objection.detail0,
             (unsigned long)objection.detail1,
             (unsigned long)objection.lane_mask);
    return;
  }

  snprintf(out, out_size,
           "%s: lane=0x%08lX detail0=%lu detail1=%lu detail2=%lu detail3=%lu; "
           "excluded from Welford/TAU/PPB/servo",
           clocks_row_objection_reason_name(objection.reason),
           (unsigned long)objection.lane,
           (unsigned long)objection.detail0,
           (unsigned long)objection.detail1,
           (unsigned long)objection.detail2,
           (unsigned long)objection.detail3);
}

void clocks_row_exclude(clocks_row_objection_source_t source,
                        clocks_row_objection_reason_t reason,
                        uint32_t lane,
                        uint32_t detail0,
                        uint32_t detail1,
                        uint32_t detail2,
                        uint32_t detail3) {
  if (reason == clocks_row_objection_reason_t::NONE) return;

  clocks_row_antecedent_hold_arm(reason, lane, detail0);

  uint32_t expected = 0U;
  if (!__atomic_compare_exchange_n(&g_row_objection_latch_state,
                                   &expected,
                                   1U,
                                   false,
                                   __ATOMIC_ACQ_REL,
                                   __ATOMIC_ACQUIRE)) {
    (void)__atomic_add_fetch(&g_row_objection_pending_count,
                             1U,
                             __ATOMIC_RELAXED);
    (void)__atomic_or_fetch(&g_row_objection_pending_lane_mask,
                            lane,
                            __ATOMIC_RELAXED);
    (void)__atomic_add_fetch(&g_row_objection_coalesced_count,
                             1U,
                             __ATOMIC_RELAXED);
    return;
  }

  g_row_objection_latch.source = source;
  g_row_objection_latch.reason = reason;
  g_row_objection_latch.lane = lane;
  g_row_objection_latch.lane_mask = lane;
  g_row_objection_latch.objection_count = 1U;
  g_row_objection_latch.detail0 = detail0;
  g_row_objection_latch.detail1 = detail1;
  g_row_objection_latch.detail2 = detail2;
  g_row_objection_latch.detail3 = detail3;
  (void)__atomic_add_fetch(&g_row_objection_pending_count,
                           1U,
                           __ATOMIC_RELAXED);
  (void)__atomic_or_fetch(&g_row_objection_pending_lane_mask,
                          lane,
                          __ATOMIC_RELAXED);
  (void)__atomic_add_fetch(&g_row_objection_raise_count,
                           1U,
                           __ATOMIC_RELAXED);
  __atomic_store_n(&g_row_objection_latch_state, 2U, __ATOMIC_RELEASE);
}

bool clocks_row_science_eligible(uint32_t pps_sequence) {
  clocks_row_antecedent_hold_apply(pps_sequence);

  uint32_t armed = 2U;
  if (__atomic_compare_exchange_n(&g_problem_injection_armed,
                                  &armed,
                                  0U,
                                  false,
                                  __ATOMIC_ACQ_REL,
                                  __ATOMIC_ACQUIRE)) {
    const uint32_t lane =
        __atomic_load_n(&g_problem_injection_lane, __ATOMIC_RELAXED);
    const uint32_t cycles =
        __atomic_load_n(&g_problem_injection_cycles, __ATOMIC_RELAXED);
    (void)__atomic_add_fetch(&g_problem_injection_fire_count,
                             1U,
                             __ATOMIC_RELAXED);
    clocks_row_exclude(
        clocks_row_objection_source_t::BETA,
        clocks_row_objection_reason_t::ALPHA_CYCLE_EXCURSION,
        lane,
        pps_sequence,
        cycles,
        256U,
        0U);
  }

  const uint32_t hold_flags = clocks_row_lifecycle_science_hold_flags();
  if (hold_flags != 0U) {
    clocks_row_exclude(
        clocks_row_objection_source_t::BETA,
        clocks_row_objection_reason_t::BETA_RECOVERY_SCIENCE_HOLD,
        CLOCKS_ROW_LANE_OCXO1 | CLOCKS_ROW_LANE_OCXO2,
        pps_sequence,
        hold_flags,
        0U,
        0U);
  }

  // Read after any lifecycle objection is raised. This also closes the small
  // window in which a future Interrupt-owned objection could arrive between
  // the first read and Alpha's irreversible Welford/TAU update.
  const bool objection_pending =
      __atomic_load_n(&g_row_objection_latch_state,
                      __ATOMIC_ACQUIRE) != 0U;
  return !objection_pending && hold_flags == 0U;
}

static bool clocks_row_objection_consume(
    clocks_row_objection_record_t& out) {
  if (__atomic_load_n(&g_row_objection_latch_state,
                      __ATOMIC_ACQUIRE) != 2U) {
    out = clocks_row_objection_record_t{};
    return false;
  }

  out = g_row_objection_latch;
  out.objection_count = __atomic_load_n(&g_row_objection_pending_count,
                                        __ATOMIC_RELAXED);
  out.lane_mask = __atomic_load_n(&g_row_objection_pending_lane_mask,
                                  __ATOMIC_RELAXED);
  g_row_objection_consumed_count++;
  __atomic_store_n(&g_row_objection_latch_state, 0U, __ATOMIC_RELEASE);
  __atomic_store_n(&g_row_objection_pending_count, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&g_row_objection_pending_lane_mask, 0U, __ATOMIC_RELAXED);
  return out.reason != clocks_row_objection_reason_t::NONE;
}

static void clocks_row_objection_clear(void) {
  __atomic_store_n(&g_problem_injection_armed, 0U, __ATOMIC_RELEASE);
  __atomic_store_n(&g_row_objection_latch_state, 0U, __ATOMIC_RELEASE);
  __atomic_store_n(&g_row_objection_pending_count, 0U, __ATOMIC_RELAXED);
  __atomic_store_n(&g_row_objection_pending_lane_mask, 0U, __ATOMIC_RELAXED);
}


static constexpr uint32_t CAMPAIGN_RECORD_STAGE_NONE = 0;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_ENTRY = 1;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_STOP_GATE = 2;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_START_ZERO_GATE = 3;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_RECOVER_GATE = 4;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_WATCHDOG_GATE = 5;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_NOT_STARTED_GATE = 6;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_WARMUP_GATE = 7;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_CANDIDATE = 8;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_PER_SECOND = 9;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_WELFORD = 10;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_SERVO = 11;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_DAC_WELFORD = 12;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE = 13;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_RECOVER_REATTACH_GATE = 14;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE = 15;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_HANDOFF_BEGIN = 20;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_HANDOFF_READY = 21;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_HANDOFF_BACKLOG = 22;

static uint32_t g_campaign_record_candidate_count = 0;

static uint32_t g_campaign_record_last_stage = CAMPAIGN_RECORD_STAGE_NONE;
static uint32_t g_campaign_record_last_public_count = 0;
static uint64_t g_campaign_record_last_public_gnss_ns = 0;
static uint64_t g_campaign_record_last_public_dwt_total = 0;

// START handoff diagnostics.  These counters prove whether Beta captured the
// campaign-public zero offset from a row whose Alpha OCXO PPS projection was
// valid for the current PPS/VCLOCK identity.  A public-origin flag alone is
// not enough: the current row must also be authored from the PPS-row OCXO
// projection species, not from a fallback measured-edge surface.
static uint32_t g_start_handoff_launch_wait_count = 0;
static uint32_t g_start_handoff_timeout_count = 0;
static bool     g_start_handoff_last_ready = false;
static bool     g_start_handoff_last_origin_ready = false;
static bool     g_start_handoff_last_ocxo1_projection_ready = false;
static bool     g_start_handoff_last_ocxo2_projection_ready = false;
static uint64_t g_start_handoff_last_raw_gnss_ns = 0;
static uint64_t g_start_handoff_last_raw_ocxo1_ns = 0;
static uint64_t g_start_handoff_last_raw_ocxo2_ns = 0;
static uint64_t g_start_handoff_last_ocxo1_projected_ns = 0;
static uint64_t g_start_handoff_last_ocxo2_projected_ns = 0;
static uint64_t g_start_handoff_last_ocxo1_projection_vclock_ns = 0;
static uint64_t g_start_handoff_last_ocxo2_projection_vclock_ns = 0;

// CounterLedger/PhaseLedger START maturity witness.  The old
// CounterLedger START branch proved only that a refined value existed, then
// released public PPS1 after a single private PPS0 sample.  That was too weak:
// public PPS1 is an interval statement, so both OCXO lanes must already have
// contiguous integer CounterLedger intervals and contiguous refined
// PhaseLedger intervals before the private PPS0 candidate is trusted.
//
// The readiness booleans and reason string participate in the bounded START
// handoff. Historical report-only counters have been removed.
static constexpr uint32_t CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS = 0U;
// CounterLedger/PhaseLedger remains the public OCXO clockface authority and
// START therefore still requires a mature refined interval.  Per-second science,
// Welford, and servo authority are Delta Cycles; a missing PhaseLedger interval
// is counted as sidecar evidence and must not overwrite or invalidate Delta.
static constexpr bool CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL = true;
static uint32_t g_phaseledger_science_last_missing_public_count = 0;
static bool     g_start_phaseledger_last_ready = false;
static bool     g_start_phaseledger_last_ocxo1_ready = false;
static bool     g_start_phaseledger_last_ocxo2_ready = false;
static bool     g_start_phaseledger_last_sequence_aligned = false;
static char     g_start_phaseledger_last_reason[64] = "reset";
static char     g_start_phaseledger_last_first_problem[32] = "";
static clocks_alpha_ocxo_counterledger_snapshot_t
    g_start_phaseledger_last_ocxo1 DMAMEM = {};
static clocks_alpha_ocxo_counterledger_snapshot_t
    g_start_phaseledger_last_ocxo2 DMAMEM = {};

// Large CounterLedger snapshots are intentionally cached at file scope rather
// than constructed as automatic locals in report/hot paths.  This keeps stack
// use predictable after the read_anchor() stack-waterline crash.
static clocks_alpha_ocxo_counterledger_snapshot_t
    g_beta_counterledger_raw_scratch DMAMEM = {};
static clocks_alpha_ocxo_counterledger_snapshot_t
    g_beta_ocxo1_counterledger_row DMAMEM = {};
static clocks_alpha_ocxo_counterledger_snapshot_t
    g_beta_ocxo2_counterledger_row DMAMEM = {};

// One immutable completed campaign record may await SYSTEM.  The typed snapshot
// lives in RAM2 so the completed-row path never places the handoff object on the
// scarce DTCM stack.  SYSTEM consumes it only when the requested PPS sequence
// matches exactly; an unconsumed record is never overwritten.
static clocks_monitor_campaign_snapshot_t
    g_monitor_campaign_record DMAMEM = {};
static bool     g_monitor_campaign_record_pending = false;
static uint32_t g_monitor_campaign_record_sequence = 0U;
static uint32_t g_monitor_campaign_record_stage_count = 0U;
static uint32_t g_monitor_campaign_record_take_count = 0U;
static uint32_t g_monitor_campaign_record_backlog_count = 0U;

// Command-report construction is a serialized foreground service. Priority-0
// capture remains live, but the priority-16 TimePop/handoff tier may not enter a
// second Payload-building path while a report owns the allocator/formatting
// surface. The typed MONITOR handoff uses separate RAM2 state and never enters
// the command-report Payload arena.
static constexpr uint32_t CLOCKS_REPORT_BASEPRI_GUARD = 16U;
static volatile bool g_clocks_report_build_active = false;
static uint32_t g_clocks_report_build_count = 0U;
static uint32_t g_clocks_report_busy_reject_count = 0U;
static uint32_t g_clocks_report_max_duration_cycles = 0U;

static clocks_instrument_stats_snapshot_t
    g_beta_report_instrument_stats DMAMEM = {};
static Payload g_report_clocks_payload DMAMEM;
static Payload g_report_stats_payload DMAMEM;
static Payload g_report_child_clocks DMAMEM;
static Payload g_report_child_stats DMAMEM;
static Payload g_report_child_clock DMAMEM;
static Payload g_report_child_welford DMAMEM;
static Payload g_report_child_maturity DMAMEM;
static Payload g_report_child_admission DMAMEM;
static Payload g_report_child_dac DMAMEM;

// Dedicated MONITOR snapshot scratch. The 1 Hz side rail never borrows command
// report Payload headers and never constructs transport objects inside CLOCKS.
static clocks_instrument_stats_snapshot_t
    g_beta_monitor_instrument_stats DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_monitor_pps_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_monitor_vclock_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_monitor_ocxo1_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_monitor_ocxo2_prediction DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_monitor_vclock_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_monitor_ocxo1_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_monitor_ocxo2_forensics DMAMEM = {};


static FLASHMEM void clocks_monitor_stats_snapshot_from_instrument(
    clocks_monitor_stats_snapshot_t& out,
    const clocks_instrument_stats_snapshot_t& instrument);
static FLASHMEM void clocks_monitor_dac_snapshot(
    clocks_monitor_dac_snapshot_t& out);

// ============================================================================
// Durable structured recovery state
// ============================================================================
//
// MONITOR and TIMEBASE carry ordinary typed state.  Floating values cross the
// Payload boundary as fixed-decimal JSON numbers; firmware never constructs,
// stores, or accepts opaque binary recovery state.

static constexpr uint32_t CLOCKS_STRUCTURED_RESTORE_VERSION = 2U;

struct clocks_recovery_restore_state_t {
  bool valid = false;

  uint64_t instrument_gnss_ns = 0ULL;
  uint64_t instrument_dwt_cycles = 0ULL;
  uint64_t instrument_ocxo1_ns = 0ULL;
  uint64_t instrument_ocxo2_ns = 0ULL;

  clocks_instrument_stats_snapshot_t stats{};
  bool control_present = false;
  clocks_monitor_dither_lane_t ocxo1_dac{};
  clocks_monitor_dither_lane_t ocxo2_dac{};
  servo_mode_t servo_mode = servo_mode_t::OFF;
  bool dither_operator_enabled = false;
};

static clocks_recovery_restore_state_t
    g_monitor_restore_state DMAMEM = {};
static clocks_recovery_restore_state_t
    g_campaign_restore_state DMAMEM = {};
static volatile bool g_monitor_restore_requested = false;
static uint32_t g_monitor_restore_request_count = 0U;
static uint32_t g_monitor_restore_commit_count = 0U;
static uint32_t g_monitor_restore_failure_count = 0U;
static uint32_t g_campaign_restore_count = 0U;
static uint32_t g_campaign_restore_failure_count = 0U;
static uint32_t g_campaign_restore_ignored_live_count = 0U;

// Campaign-neutral presentation transform.  The fresh SmartZero epoch remains
// Alpha's physical coordinate; this transform makes the durable instrument
// clockfaces continue across a reboot without fabricating cross-reboot edge
// intervals.
static bool    g_instrument_continuity_active = false;
static int64_t g_instrument_gnss_offset = 0;
static int64_t g_instrument_dwt_offset = 0;
static int64_t g_instrument_ocxo1_offset = 0;
static int64_t g_instrument_ocxo2_offset = 0;
static uint32_t g_instrument_continuity_install_count = 0U;
static uint32_t g_instrument_continuity_reset_count = 0U;

static inline uint32_t clocks_report_irq_save(void) {
  uint32_t prior_basepri = 0U;
  __asm__ volatile ("mrs %0, basepri" : "=r" (prior_basepri) :: "memory");
  if (prior_basepri == 0U || prior_basepri > CLOCKS_REPORT_BASEPRI_GUARD) {
    __asm__ volatile ("msr basepri, %0"
                      :: "r" (CLOCKS_REPORT_BASEPRI_GUARD) : "memory");
  }
  __asm__ volatile ("dmb" ::: "memory");
  return prior_basepri;
}

static inline void clocks_report_irq_restore(uint32_t prior_basepri) {
  __asm__ volatile ("dmb" ::: "memory");
  __asm__ volatile ("msr basepri, %0" :: "r" (prior_basepri) : "memory");
}

static bool clocks_recovery_dac_lane_valid(
    const clocks_monitor_dither_lane_t& state) {
  return isfinite(state.value) &&
         state.value >= 0.0 &&
         state.value <= (double)OCXO_DAC_SAFE_MAX_HW_CODE &&
         isfinite(state.servo_last_step) &&
         isfinite(state.servo_last_residual) &&
         isfinite(state.servo_last_raw_residual) &&
         isfinite(state.servo_filtered_residual) &&
         isfinite(state.servo_filtered_slope) &&
         isfinite(state.servo_predicted_residual);
}

static bool clocks_recovery_restore_statistics_valid(
    const clocks_recovery_restore_state_t& state) {
  return state.valid &&
         state.instrument_gnss_ns != 0ULL &&
         state.instrument_dwt_cycles != 0ULL &&
         state.instrument_ocxo1_ns != 0ULL &&
         state.instrument_ocxo2_ns != 0ULL;
}

static bool clocks_recovery_restore_control_valid(
    const clocks_recovery_restore_state_t& state) {
  return clocks_recovery_restore_statistics_valid(state) &&
         state.control_present &&
         (uint8_t)state.servo_mode <= (uint8_t)servo_mode_t::NOW &&
         clocks_recovery_dac_lane_valid(state.ocxo1_dac) &&
         clocks_recovery_dac_lane_valid(state.ocxo2_dac);
}

static bool restore_get_u64(const Payload& args, const char* key,
                            uint64_t& out, bool required = true) {
  if (!args.has(key)) return !required;
  return args.tryGetUInt64(key, out);
}

static bool restore_get_u32(const Payload& args, const char* key,
                            uint32_t& out, bool required = true) {
  if (!args.has(key)) return !required;
  return args.tryGetUInt(key, out);
}

static bool restore_get_double(const Payload& args, const char* key,
                               double& out, bool required = true) {
  if (!args.has(key)) return !required;
  const char* token = args.getString(key);
  if (!token || !*token) return false;
  errno = 0;
  char* end = nullptr;
  const double parsed = strtod(token, &end);
  if (errno == ERANGE || !end || *end != '\0' || !isfinite(parsed)) {
    return false;
  }
  out = parsed;
  return true;
}

static bool restore_get_bool(const Payload& args, const char* key,
                             bool& out, bool required = true) {
  if (!args.has(key)) return !required;
  return args.tryGetBool(key, out);
}

static bool restore_parse_welford(const Payload& args,
                                  const char* prefix,
                                  welford_t& out) {
  char key[72];
  snprintf(key, sizeof(key), "%s_n", prefix);
  if (!restore_get_u64(args, key, out.n)) return false;
  snprintf(key, sizeof(key), "%s_mean", prefix);
  if (!restore_get_double(args, key, out.mean)) return false;
  snprintf(key, sizeof(key), "%s_m2", prefix);
  if (!restore_get_double(args, key, out.m2)) return false;
  snprintf(key, sizeof(key), "%s_min", prefix);
  if (!restore_get_double(args, key, out.min_val)) return false;
  snprintf(key, sizeof(key), "%s_max", prefix);
  if (!restore_get_double(args, key, out.max_val)) return false;
  return out.n == 0ULL ||
         (isfinite(out.mean) && isfinite(out.m2) && out.m2 >= 0.0 &&
          isfinite(out.min_val) && isfinite(out.max_val) &&
          out.min_val <= out.max_val);
}

static bool restore_parse_tau(const Payload& args,
                              const char* prefix,
                              time_clock_id_t clock_id,
                              clocks_alpha_tau_snapshot_t& out) {
  char key[80];
  bool valid = false;
  snprintf(key, sizeof(key), "%s_valid", prefix);
  if (!restore_get_bool(args, key, valid)) return false;
  out = clocks_alpha_tau_snapshot_t{};
  out.clock_id = (uint32_t)((uint8_t)clock_id);
  out.valid = valid;
#define RESTORE_TAU_U32(field) \
  do { snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_u32(args, key, out.field)) return false; } while (0)
#define RESTORE_TAU_U64(field) \
  do { snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_u64(args, key, out.field)) return false; } while (0)
#define RESTORE_TAU_DBL(field) \
  do { snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_double(args, key, out.field)) return false; } while (0)
  RESTORE_TAU_U32(reset_count);
  RESTORE_TAU_U32(sample_count);
  RESTORE_TAU_U32(interval_count);
  RESTORE_TAU_U32(reject_count);
  RESTORE_TAU_U32(gap_reset_count);
  RESTORE_TAU_U32(last_pps_sequence);
  RESTORE_TAU_U32(last_interval_pps_sequence);
  RESTORE_TAU_U64(first_refined_ns);
  RESTORE_TAU_U64(last_refined_ns);
  {
    uint64_t raw = 0ULL;
    snprintf(key, sizeof(key), "%s_last_fast_residual_ns", prefix);
    const char* text = args.getString(key);
    if (!text || !*text) return false;
    char* end = nullptr;
    const long long value = strtoll(text, &end, 10);
    if (!end || *end != '\0') return false;
    out.last_fast_residual_ns = (int64_t)value;
    (void)raw;
  }
  RESTORE_TAU_U64(cumulative_reference_ns);
  RESTORE_TAU_U64(cumulative_clock_ns);
  RESTORE_TAU_DBL(cumulative_clock_ns_exact);
  RESTORE_TAU_DBL(mean_x);
  RESTORE_TAU_DBL(mean_y);
  RESTORE_TAU_DBL(sxx);
  RESTORE_TAU_DBL(sxy);
  RESTORE_TAU_DBL(syy);
  RESTORE_TAU_DBL(interval_mean_ppb);
  RESTORE_TAU_DBL(interval_m2_ppb);
#undef RESTORE_TAU_U32
#undef RESTORE_TAU_U64
#undef RESTORE_TAU_DBL
  return isfinite(out.cumulative_clock_ns_exact) &&
         isfinite(out.mean_x) && isfinite(out.mean_y) &&
         isfinite(out.sxx) && isfinite(out.sxy) && isfinite(out.syy) &&
         isfinite(out.interval_mean_ppb) &&
         isfinite(out.interval_m2_ppb) && out.interval_m2_ppb >= 0.0;
}

static bool restore_parse_dac_lane(const Payload& args,
                                   const char* prefix,
                                   clocks_monitor_dither_lane_t& out) {
  char key[80];
#define RESTORE_DAC_DBL(field) \
  do { snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_double(args, key, out.field)) return false; } while (0)
#define RESTORE_DAC_U32(field) \
  do { uint32_t value = 0U; snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_u32(args, key, value)) return false; out.field = value; } while (0)
#define RESTORE_DAC_BOOL(field) \
  do { snprintf(key, sizeof(key), "%s_" #field, prefix); \
       if (!restore_get_bool(args, key, out.field)) return false; } while (0)
  RESTORE_DAC_DBL(value);
  RESTORE_DAC_DBL(servo_last_step);
  RESTORE_DAC_DBL(servo_last_residual);
  RESTORE_DAC_U32(servo_settle_count);
  RESTORE_DAC_U32(servo_adjustments);
  RESTORE_DAC_BOOL(servo_predictor_initialized);
  RESTORE_DAC_DBL(servo_last_raw_residual);
  RESTORE_DAC_DBL(servo_filtered_residual);
  RESTORE_DAC_DBL(servo_filtered_slope);
  RESTORE_DAC_DBL(servo_predicted_residual);
  RESTORE_DAC_U32(servo_predictor_updates);
#undef RESTORE_DAC_DBL
#undef RESTORE_DAC_U32
#undef RESTORE_DAC_BOOL
  return clocks_recovery_dac_lane_valid(out);
}

static bool clocks_recovery_state_from_args(
    const Payload& args,
    clocks_recovery_restore_state_t& out,
    bool require_control) {
  out = clocks_recovery_restore_state_t{};
  uint32_t version = 0U;
  if (!restore_get_u32(args, "restore_schema_version", version) ||
      version != CLOCKS_STRUCTURED_RESTORE_VERSION) return false;
  if (!restore_get_u64(args, "restore_instrument_gnss_ns",
                       out.instrument_gnss_ns) ||
      !restore_get_u64(args, "restore_instrument_dwt_cycles",
                       out.instrument_dwt_cycles) ||
      !restore_get_u64(args, "restore_instrument_ocxo1_ns",
                       out.instrument_ocxo1_ns) ||
      !restore_get_u64(args, "restore_instrument_ocxo2_ns",
                       out.instrument_ocxo2_ns)) return false;

  if (!restore_get_bool(args, "restore_stats_valid", out.stats.valid) ||
      !restore_get_bool(args, "restore_stats_completed_row_coherent",
                        out.stats.completed_row_coherent) ||
      !restore_get_u32(args, "restore_stats_reset_count",
                       out.stats.reset_count) ||
      !restore_get_u32(args, "restore_stats_update_count",
                       out.stats.update_count) ||
      !restore_get_u32(args, "restore_stats_vclock_interval_reject_count",
                       out.stats.vclock_interval_reject_count) ||
      !restore_get_u32(args, "restore_stats_ocxo1_interval_reject_count",
                       out.stats.ocxo1_interval_reject_count) ||
      !restore_get_u32(args, "restore_stats_ocxo2_interval_reject_count",
                       out.stats.ocxo2_interval_reject_count)) return false;

  if (!restore_parse_welford(args, "restore_gnss_welford",
                             out.stats.gnss_welford) ||
      !restore_parse_welford(args, "restore_dwt_welford",
                             out.stats.dwt_welford) ||
      !restore_parse_welford(args, "restore_vclock_welford",
                             out.stats.vclock_welford) ||
      !restore_parse_welford(args, "restore_ocxo1_welford",
                             out.stats.ocxo1_welford) ||
      !restore_parse_welford(args, "restore_ocxo2_welford",
                             out.stats.ocxo2_welford) ||
      !restore_parse_welford(args, "restore_pps_witness_welford",
                             out.stats.pps_witness_welford) ||
      !restore_parse_welford(args, "restore_ocxo1_dac_welford",
                             out.stats.ocxo1_dac_welford) ||
      !restore_parse_welford(args, "restore_ocxo2_dac_welford",
                             out.stats.ocxo2_dac_welford) ||
      !restore_parse_tau(args, "restore_ocxo1_tau",
                         time_clock_id_t::OCXO1,
                         out.stats.ocxo1_tau_state) ||
      !restore_parse_tau(args, "restore_ocxo2_tau",
                         time_clock_id_t::OCXO2,
                         out.stats.ocxo2_tau_state)) {
    return false;
  }

  if (require_control) {
    if (!restore_parse_dac_lane(args, "restore_ocxo1_dac",
                                out.ocxo1_dac) ||
        !restore_parse_dac_lane(args, "restore_ocxo2_dac",
                                out.ocxo2_dac)) {
      return false;
    }

    const char* mode = args.getString("restore_servo_mode");
    if (!mode || !*mode) return false;
    out.servo_mode = servo_mode_parse(mode);
    if (strcmp(mode, "OFF") != 0 && strcmp(mode, "MEAN") != 0 &&
        strcmp(mode, "TOTAL") != 0 && strcmp(mode, "NOW") != 0) {
      return false;
    }
    if (!restore_get_bool(args, "restore_dither_operator_enabled",
                          out.dither_operator_enabled)) {
      return false;
    }
    out.control_present = true;
  }

  out.stats.last_pps_sequence = 0U;
  out.stats.gnss_ns = out.instrument_gnss_ns;
  out.stats.dwt_cycles = out.instrument_dwt_cycles;
  out.stats.ocxo1_ns = out.instrument_ocxo1_ns;
  out.stats.ocxo2_ns = out.instrument_ocxo2_ns;
  out.valid = true;
  return require_control
      ? clocks_recovery_restore_control_valid(out)
      : clocks_recovery_restore_statistics_valid(out);
}


static void clocks_recovery_state_snapshot(
    clocks_monitor_restore_state_t& out,
    const clocks_instrument_stats_snapshot_t& instrument,
    uint64_t logical_gnss_ns,
    uint64_t logical_dwt_cycles,
    uint64_t logical_ocxo1_ns,
    uint64_t logical_ocxo2_ns) {
  out = clocks_monitor_restore_state_t{};
  out.present = instrument.valid && logical_gnss_ns != 0ULL &&
                logical_dwt_cycles != 0ULL && logical_ocxo1_ns != 0ULL &&
                logical_ocxo2_ns != 0ULL;
  out.schema_version = CLOCKS_STRUCTURED_RESTORE_VERSION;
  out.instrument_gnss_ns = logical_gnss_ns;
  out.instrument_dwt_cycles = logical_dwt_cycles;
  out.instrument_ocxo1_ns = logical_ocxo1_ns;
  out.instrument_ocxo2_ns = logical_ocxo2_ns;
  clocks_monitor_stats_snapshot_from_instrument(out.stats, instrument);
  clocks_monitor_dac_snapshot(out.dac);
}


struct clocks_report_build_guard_t {
  uint32_t prior_basepri = 0U;
  uint32_t begin_dwt = 0U;
  bool acquired = false;

  clocks_report_build_guard_t() {
    prior_basepri = clocks_report_irq_save();
    if (!g_clocks_report_build_active) {
      g_clocks_report_build_active = true;
      g_clocks_report_build_count++;
      begin_dwt = DWT_CYCCNT;
      acquired = true;
    } else {
      g_clocks_report_busy_reject_count++;
      clocks_report_irq_restore(prior_basepri);
    }
  }

  ~clocks_report_build_guard_t() {
    if (!acquired) return;
    const uint32_t elapsed = (uint32_t)(DWT_CYCCNT - begin_dwt);
    if (elapsed > g_clocks_report_max_duration_cycles) {
      g_clocks_report_max_duration_cycles = elapsed;
    }
    g_clocks_report_build_active = false;
    clocks_report_irq_restore(prior_basepri);
  }
};


// FLASH_CUT flight recorder.  These counters are deliberately Beta-local:
// Flash Cut does not install a new Alpha epoch and must not perturb the
// timing/custody surfaces it is trying to preserve.
static uint32_t g_flash_cut_request_count = 0;
static uint32_t g_flash_cut_commit_count = 0;
static uint32_t g_flash_cut_reject_count = 0;
static uint32_t g_flash_cut_busy_reject_count = 0;
static char     g_flash_cut_last_requested_campaign[64] = {0};
static char     g_flash_cut_last_from_campaign[64] = {0};
static char     g_flash_cut_last_to_campaign[64] = {0};
static uint64_t g_flash_cut_last_raw_gnss_ns = 0;
static uint64_t g_flash_cut_last_raw_dwt_cycles = 0;
static uint64_t g_flash_cut_last_raw_ocxo1_ns = 0;
static uint64_t g_flash_cut_last_raw_ocxo2_ns = 0;
static uint32_t g_flash_cut_last_boundary_pps_count = 0;
static char     g_flash_cut_last_status[48] = {0};

// ============================================================================
// SYSTEM feature status — CLOCKS/Beta-owned readiness surfaces
// ============================================================================
//
// These are publication/science-pipeline facts.  They remain observational;
// START/ZERO/RECOVER do not use them as command-admission policy.

static system_feature_status_t g_clocks_feature_science_residuals =
    system_feature_status_t::ANOMALY;
static system_feature_status_t g_clocks_feature_campaign_record =
    system_feature_status_t::ANOMALY;

static void clocks_beta_feature_set_cached(const char* feature,
                                           system_feature_status_t& cached,
                                           system_feature_status_t status,
                                           bool force = false) {
  if (force || cached != status || !system_feature_has("CLOCKS", feature)) {
    (void)system_feature_set("CLOCKS", feature, status, nullptr);
    cached = status;
  }
}

static FLASHMEM void clocks_beta_cold_diagnostics_init(void);
static bool g_clocks_beta_dmamem_initialized = false;

static void clocks_beta_features_mark_initializing(void) {
  clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                 g_clocks_feature_science_residuals,
                                 system_feature_status_t::INITIALIZING,
                                 true);
  clocks_beta_feature_set_cached("CAMPAIGN_RECORD_HANDOFF",
                                 g_clocks_feature_campaign_record,
                                 system_feature_status_t::INITIALIZING,
                                 true);
}

void clocks_beta_features_init(void) {
  clocks_beta_cold_diagnostics_init();
  memset(&g_monitor_campaign_record, 0,
         sizeof(g_monitor_campaign_record));
  g_monitor_campaign_record_pending = false;
  g_monitor_campaign_record_sequence = 0U;
  g_report_clocks_payload.clear();
  g_report_stats_payload.clear();
  g_report_child_clocks.clear();
  g_report_child_stats.clear();
  g_report_child_clock.clear();
  g_report_child_welford.clear();
  g_report_child_maturity.clear();
  g_report_child_admission.clear();
  g_report_child_dac.clear();
  clocks_beta_features_mark_initializing();
}

// ============================================================================
// Campaign admission authority
// ============================================================================
//
// Global campaign-readiness policy is evaluated once by Pi CLOCKS from a fresh
// unified MONITOR.features heartbeat before it sends CLOCKS.START.  Teensy
// CLOCKS must not subscribe to MONITOR (which would echo its own state back
// through the Pi) or to a feature-only side feed.  Receipt of CLOCKS.START is
// therefore the global-policy authorization boundary.
//
// Firmware still owns every local imperative and physical transition court:
// command/state validation, numeric integrity, DAC realization, SmartZero,
// private PPS0 acquisition, CounterLedger/PhaseLedger maturity, watchdog
// custody, and the release of public PPS1.  Those courts remain authoritative
// and cannot be bypassed by MONITOR.
//

static FLASHMEM const char* campaign_record_stage_name(uint32_t stage) {
  switch (stage) {
    case CAMPAIGN_RECORD_STAGE_ENTRY: return "ENTRY";
    case CAMPAIGN_RECORD_STAGE_STOP_GATE: return "STOP_GATE";
    case CAMPAIGN_RECORD_STAGE_START_ZERO_GATE: return "START_ZERO_GATE";
    case CAMPAIGN_RECORD_STAGE_RECOVER_GATE: return "RECOVER_GATE";
    case CAMPAIGN_RECORD_STAGE_WATCHDOG_GATE: return "WATCHDOG_GATE";
    case CAMPAIGN_RECORD_STAGE_NOT_STARTED_GATE: return "NOT_STARTED_GATE";
    case CAMPAIGN_RECORD_STAGE_WARMUP_GATE: return "WARMUP_GATE";
    case CAMPAIGN_RECORD_STAGE_CANDIDATE: return "CANDIDATE";
    case CAMPAIGN_RECORD_STAGE_PER_SECOND: return "PER_SECOND";
    case CAMPAIGN_RECORD_STAGE_WELFORD: return "WELFORD";
    case CAMPAIGN_RECORD_STAGE_SERVO: return "SERVO";
    case CAMPAIGN_RECORD_STAGE_DAC_WELFORD: return "DAC_WELFORD";
    case CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE: return "FLASH_CUT_GATE";
    case CAMPAIGN_RECORD_STAGE_RECOVER_REATTACH_GATE: return "RECOVER_REATTACH_GATE";
    case CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE: return "RECOVERING_NO_REQUEST_GATE";
    case CAMPAIGN_RECORD_STAGE_HANDOFF_BEGIN: return "HANDOFF_BEGIN";
    case CAMPAIGN_RECORD_STAGE_HANDOFF_READY: return "HANDOFF_READY";
    case CAMPAIGN_RECORD_STAGE_HANDOFF_BACKLOG: return "HANDOFF_BACKLOG";
    default: return "NONE";
  }
}

static void campaign_record_stage(uint32_t stage) {
  g_campaign_record_last_stage = stage;
}

static const char* clocks_campaign_state_name(clocks_campaign_state_t state) {
  switch (state) {
    case clocks_campaign_state_t::STARTED:    return "STARTED";
    case clocks_campaign_state_t::RECOVERING: return "RECOVERING";
    default:                                  return "STOPPED";
  }
}

static bool clocks_campaign_recovery_lifecycle_active(void) {
  return campaign_state == clocks_campaign_state_t::RECOVERING;
}

// ============================================================================
// Stack witness — crash autopsy breadcrumbs
// ============================================================================
//
// A DACCVIOL at process_time::read_anchor() while zeroing a tiny local object
// means the caller arrived with the stack already compromised. Keep this
// witness tiny and projection-free.
//
// Important correction: this witness must NOT live in .noinit. On Teensy 4.x
// the linker may place .noinit near the upper DTCM/stack-guard region; merely
// reading such an address can fault before the witness records anything. Keep
// it in ordinary safe storage and make every touch pass a pointer-address
// safety court first. This build puts the witness in RAM2/DMAMEM so it cannot
// consume the tiny DTCM stack/local-variable budget.

static constexpr uint32_t CLOCKS_STACK_WITNESS_MAGIC = 0x5A505357UL; // ZPSW
static constexpr uint32_t CLOCKS_STACK_WITNESS_DTCM_BASE = 0x20000000UL;
// Teensy RAM1 is split at build/link time between ITCM code and DTCM
// data/stack. The DTCM top is therefore not a fixed 0x20040000 boundary;
// derive the active top from the observed stack pointer rounded to the RAM1
// allocation granule. The full T4.1 RAM1 DTCM address ceiling is retained only
// as a sanity clamp for that derivation.
static constexpr uint32_t CLOCKS_STACK_WITNESS_RAM1_MAX_TOP = 0x20080000UL;
static constexpr uint32_t CLOCKS_STACK_WITNESS_DTCM_GRANULE_BYTES = 32768UL;
static constexpr uint32_t CLOCKS_STACK_WITNESS_DTCM_STACK_GUARD_BYTES = 16384UL;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_NONE = 0U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_RECOVERY = 3U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_STACK = 4U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_RECOVER_REFRESH_READY = 10U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_RECOVER_SHOULD_HOLD = 11U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_RECOVER_DEGRADED_HOLD = 12U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_BETA_PPS_ENTRY = 20U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_BETA_PPS_BUILD = 21U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_BETA_PPS_PUBLISH = 22U;
static constexpr bool CLOCKS_STACK_WITNESS_RECORD_HOTPATHS = false;
static constexpr bool CLOCKS_STACK_WITNESS_RECORD_COMMANDS = true;


struct clocks_stack_witness_t {
  uint32_t magic;
  uint32_t record_count;
  uint32_t reset_count;
  uint32_t last_sp;
  uint32_t min_sp;
  uint32_t last_context;
  uint32_t min_context;
  uint32_t last_campaign_seconds;
  uint32_t min_campaign_seconds;
};

static volatile clocks_stack_witness_t g_clocks_stack_witness DMAMEM = {};

static inline uint32_t clocks_stack_witness_sp(void) {
  uint32_t sp = 0;
  __asm__ volatile ("mov %0, sp" : "=r" (sp) :: "memory");
  return sp;
}

static inline bool clocks_stack_witness_addr_in_dtcm(uintptr_t addr) {
  return addr >= (uintptr_t)CLOCKS_STACK_WITNESS_DTCM_BASE &&
         addr < (uintptr_t)CLOCKS_STACK_WITNESS_RAM1_MAX_TOP;
}

static FLASHMEM uint32_t clocks_stack_witness_dtcm_top_from_sp(uint32_t sp) {
  if (!clocks_stack_witness_addr_in_dtcm((uintptr_t)sp)) return 0U;

  const uint32_t granule = CLOCKS_STACK_WITNESS_DTCM_GRANULE_BYTES;
  const uint32_t top = (sp + granule - 1U) & ~(granule - 1U);
  if (top <= sp || top > CLOCKS_STACK_WITNESS_RAM1_MAX_TOP) return 0U;
  return top;
}

static FLASHMEM uint32_t clocks_stack_witness_observed_dtcm_top(
    uint32_t sp0,
    uint32_t sp1,
    uint32_t sp2) {
  uint32_t top = 0U;
  const uint32_t t0 = clocks_stack_witness_dtcm_top_from_sp(sp0);
  const uint32_t t1 = clocks_stack_witness_dtcm_top_from_sp(sp1);
  const uint32_t t2 = clocks_stack_witness_dtcm_top_from_sp(sp2);
  if (t0 > top) top = t0;
  if (t1 > top) top = t1;
  if (t2 > top) top = t2;
  return top;
}

static FLASHMEM uint32_t clocks_stack_witness_current_dtcm_top(void) {
  return clocks_stack_witness_observed_dtcm_top(
      clocks_stack_witness_sp(), 0U, 0U);
}

static FLASHMEM bool clocks_stack_witness_storage_safe(void) {
  const uintptr_t addr = (uintptr_t)&g_clocks_stack_witness;
  const uintptr_t end = addr + sizeof(g_clocks_stack_witness);

  // Ordinary DTCM static storage is safe only if it stays well below the
  // descending stack/guard neighborhood. If the linker ever places this object
  // in DTCM, use the observed build-specific DTCM top rather than the obsolete
  // fixed 0x20040000 boundary.
  if (clocks_stack_witness_addr_in_dtcm(addr)) {
    const uint32_t dtcm_top = clocks_stack_witness_current_dtcm_top();
    if (dtcm_top == 0U || end > (uintptr_t)dtcm_top) return false;

    const uintptr_t safe_top =
        (uintptr_t)dtcm_top -
        (uintptr_t)CLOCKS_STACK_WITNESS_DTCM_STACK_GUARD_BYTES;
    return end <= safe_top;
  }

  // Non-DTCM placements, especially DMAMEM/RAM2, are not in the DTCM stack
  // guard region this witness is trying to avoid.
  return addr != 0U && end > addr;
}

static FLASHMEM void clocks_stack_witness_reset(void) {
  if (!clocks_stack_witness_storage_safe()) return;

  clocks_stack_witness_t* w =
      (clocks_stack_witness_t*)&g_clocks_stack_witness;
  const uint32_t prior_reset_count =
      (w->magic == CLOCKS_STACK_WITNESS_MAGIC) ? w->reset_count : 0U;
  memset(w, 0, sizeof(*w));
  w->magic = CLOCKS_STACK_WITNESS_MAGIC;
  w->reset_count = prior_reset_count + 1U;
  w->min_sp = 0xFFFFFFFFUL;
}

static FLASHMEM bool clocks_stack_witness_ready(void) {
  if (!clocks_stack_witness_storage_safe()) return false;
  if (g_clocks_stack_witness.magic == CLOCKS_STACK_WITNESS_MAGIC &&
      g_clocks_stack_witness.min_sp != 0U) {
    return true;
  }
  clocks_stack_witness_reset();
  return g_clocks_stack_witness.magic == CLOCKS_STACK_WITNESS_MAGIC &&
         g_clocks_stack_witness.min_sp != 0U;
}

static FLASHMEM const char* clocks_stack_witness_context_name(uint32_t context) {
  switch (context) {
    case CLOCKS_STACK_CONTEXT_REPORT_RECOVERY: return "REPORT_RECOVERY";
    case CLOCKS_STACK_CONTEXT_REPORT_STACK: return "REPORT_STACK";
    case CLOCKS_STACK_CONTEXT_RECOVER_REFRESH_READY: return "RECOVER_REFRESH_READY";
    case CLOCKS_STACK_CONTEXT_RECOVER_SHOULD_HOLD: return "RECOVER_SHOULD_HOLD";
    case CLOCKS_STACK_CONTEXT_RECOVER_DEGRADED_HOLD: return "RECOVER_DEGRADED_HOLD";
    case CLOCKS_STACK_CONTEXT_BETA_PPS_ENTRY: return "BETA_PPS_ENTRY";
    case CLOCKS_STACK_CONTEXT_BETA_PPS_BUILD: return "BETA_PPS_BUILD";
    case CLOCKS_STACK_CONTEXT_BETA_PPS_PUBLISH: return "BETA_PPS_PUBLISH";
    default: return "NONE";
  }
}

static FLASHMEM uint32_t clocks_stack_witness_bytes_below_dtcm_top(
    uint32_t sp,
    uint32_t dtcm_top) {
  if (sp == 0U || dtcm_top == 0U || sp > dtcm_top) return 0U;
  return dtcm_top - sp;
}

static FLASHMEM void clocks_stack_witness_note(uint32_t context) {
  if (!clocks_stack_witness_ready()) return;

  const uint32_t sp = clocks_stack_witness_sp();
  clocks_stack_witness_t* w =
      (clocks_stack_witness_t*)&g_clocks_stack_witness;

  w->record_count++;
  w->last_sp = sp;
  w->last_context = context;
  w->last_campaign_seconds = (uint32_t)campaign_seconds;
  if (sp != 0U && sp < w->min_sp) {
    w->min_sp = sp;
    w->min_context = context;
    w->min_campaign_seconds = (uint32_t)campaign_seconds;
  }
}
static inline void clocks_stack_witness_note_hot(uint32_t context) {
  if (CLOCKS_STACK_WITNESS_RECORD_HOTPATHS) {
    clocks_stack_witness_note(context);
  }
}

static inline void clocks_stack_witness_note_command(uint32_t context) {
  if (CLOCKS_STACK_WITNESS_RECORD_COMMANDS) {
    clocks_stack_witness_note(context);
  }
}


static FLASHMEM void payload_add_stack_witness(Payload& p) {
  const uintptr_t storage_addr = (uintptr_t)&g_clocks_stack_witness;
  const bool storage_safe = clocks_stack_witness_storage_safe();
  const bool ready = clocks_stack_witness_ready();

  uint32_t last_sp = 0U;
  uint32_t min_sp = 0U;
  uint32_t min_sp_report = 0U;
  const uint32_t current_sp = clocks_stack_witness_sp();
  uint32_t observed_dtcm_top = clocks_stack_witness_observed_dtcm_top(
      current_sp, 0U, 0U);

  p.add("stack_witness_schema", "CLOCKS_STACK_WITNESS_V4");
  p.add("stack_witness_enabled", storage_safe && ready);
  p.add("stack_witness_persistent", false);
  p.add("stack_witness_storage_addr", (uint32_t)storage_addr);
  p.add("stack_witness_storage_size", (uint32_t)sizeof(g_clocks_stack_witness));
  p.add("stack_witness_storage_safe", storage_safe);
  p.add("stack_witness_dtcm_base", CLOCKS_STACK_WITNESS_DTCM_BASE);
  p.add("stack_witness_dtcm_top", observed_dtcm_top);
  p.add("stack_witness_dtcm_top_source", "SP_ROUND_UP_GRANULE");
  p.add("stack_witness_dtcm_granule_bytes",
        CLOCKS_STACK_WITNESS_DTCM_GRANULE_BYTES);
  p.add("stack_witness_ram1_max_top", CLOCKS_STACK_WITNESS_RAM1_MAX_TOP);
  p.add("stack_witness_dtcm_guard_bytes",
        CLOCKS_STACK_WITNESS_DTCM_STACK_GUARD_BYTES);
  p.add("stack_witness_current_sp", current_sp);
  p.add("stack_witness_current_bytes_from_top",
        clocks_stack_witness_bytes_below_dtcm_top(current_sp,
                                                  observed_dtcm_top));

  if (!ready) {
    p.add("stack_witness_magic", 0U);
    p.add("stack_witness_count", 0U);
    p.add("stack_witness_reset_count", 0U);
    p.add("stack_witness_last_sp", 0U);
    p.add("stack_witness_min_sp", 0U);
    p.add("stack_witness_last_context_id", 0U);
    p.add("stack_witness_last_context", "NONE");
    p.add("stack_witness_min_context_id", 0U);
    p.add("stack_witness_min_context", "NONE");
    p.add("stack_witness_last_campaign_seconds", 0U);
    p.add("stack_witness_min_campaign_seconds", 0U);
    p.add("stack_witness_last_bytes_from_top", 0U);
    p.add("stack_witness_min_bytes_from_top", 0U);
    return;
  }

  last_sp = g_clocks_stack_witness.last_sp;
  min_sp = g_clocks_stack_witness.min_sp;
  min_sp_report = (min_sp == 0xFFFFFFFFUL) ? 0U : min_sp;
  observed_dtcm_top = clocks_stack_witness_observed_dtcm_top(
      current_sp, last_sp, min_sp_report);

  // Compatibility alias retained, but it now reports the derived build-specific
  // DTCM top instead of the obsolete fixed 0x20040000 boundary.
  p.add("stack_witness_dtcm_top_effective", observed_dtcm_top);

  p.add("stack_witness_magic", (uint32_t)g_clocks_stack_witness.magic);
  p.add("stack_witness_count", (uint32_t)g_clocks_stack_witness.record_count);
  p.add("stack_witness_reset_count", (uint32_t)g_clocks_stack_witness.reset_count);
  p.add("stack_witness_last_sp", last_sp);
  p.add("stack_witness_min_sp", min_sp_report);
  p.add("stack_witness_last_context_id", (uint32_t)g_clocks_stack_witness.last_context);
  p.add("stack_witness_last_context",
        clocks_stack_witness_context_name((uint32_t)g_clocks_stack_witness.last_context));
  p.add("stack_witness_min_context_id", (uint32_t)g_clocks_stack_witness.min_context);
  p.add("stack_witness_min_context",
        clocks_stack_witness_context_name((uint32_t)g_clocks_stack_witness.min_context));
  p.add("stack_witness_last_campaign_seconds",
        (uint32_t)g_clocks_stack_witness.last_campaign_seconds);
  p.add("stack_witness_min_campaign_seconds",
        (uint32_t)g_clocks_stack_witness.min_campaign_seconds);
  p.add("stack_witness_last_bytes_from_top",
        clocks_stack_witness_bytes_below_dtcm_top(last_sp, observed_dtcm_top));
  p.add("stack_witness_min_bytes_from_top",
        clocks_stack_witness_bytes_below_dtcm_top(min_sp_report, observed_dtcm_top));
  p.add("stack_witness_last_bytes_below_dtcm_top",
        clocks_stack_witness_bytes_below_dtcm_top(last_sp, observed_dtcm_top));
  p.add("stack_witness_min_bytes_below_dtcm_top",
        clocks_stack_witness_bytes_below_dtcm_top(min_sp_report, observed_dtcm_top));
}

// ============================================================================
// Campaign publication handoff
// ============================================================================
//
// Fixed campaign record row burial is retired.  START instead owns a private,
// pre-publication prologue: Alpha and Interrupt continue advancing while Beta
// keeps campaign_seconds at zero, captures a lawful PPS0 bookend, and releases
// the next mature candidate as public PPS1.  No public campaign identity is
// screened or skipped; public campaign time simply has not begun yet.
//
// RECOVER remains different: it resumes an existing public timeline and
// therefore continues emitting candidate identities while reattachment and
// science custody recover.
//
// RECOVER publishes the next PPS/VCLOCK row after the recovered base count.
// Recovery science residuals may be quarantined for population hygiene, but
// the campaign record row itself is no longer suppressed.

enum class campaign_warmup_mode_t : uint8_t {
  NONE    = 0,
  START   = 1,
  RECOVER = 2,
};

static volatile campaign_warmup_mode_t g_campaign_warmup_mode =
    campaign_warmup_mode_t::NONE;
static volatile uint32_t g_campaign_warmup_remaining = 0;
static volatile uint32_t g_campaign_warmup_suppressed_total = 0;

// START science prologue.  These are internal PPS candidates after SmartZero
// has installed the epoch but before public campaign time begins.  They are
// not hidden campaign record rows: campaign_seconds remains zero.  The prologue
// establishes a valid conceptual PPS0/bookend, then probes the next candidate
// so public PPS1 can carry a fully qualified observed Delta sample
// and enter Welford as n=1.
static constexpr uint32_t CLOCKS_START_PROLOGUE_INTERVAL_CONTINUITY_GATE_CYCLES = 16U;
static constexpr uint32_t CLOCKS_START_PROLOGUE_MAX_PRIVATE_CANDIDATES = 32U;
// Bound the purely-private START handoff.  This is not science-row burial; it
// is a launch-acquisition watchdog so a failed OCXO public-origin/projection
// proof becomes a local aborted START instead of 90 seconds of campaign record silence.
static constexpr uint32_t CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES = 32U;
static volatile bool     g_start_prologue_seeded = false;
static volatile bool     g_start_prologue_reference_ready = false;
static volatile uint32_t g_start_prologue_private_candidate_count = 0;
static volatile uint32_t g_start_prologue_release_count = 0;
static volatile uint32_t g_start_prologue_last_private_count = 0;
static volatile uint32_t g_start_prologue_last_release_public_count = 0;
static volatile uint32_t g_start_prologue_private_limit_count = 0;
static char              g_start_prologue_last_reason[64] = "reset";
static volatile bool     g_start_prologue_pps0_interval_valid = false;
static volatile uint32_t g_start_prologue_pps0_pps_obs = 0;
static volatile uint32_t g_start_prologue_pps0_v_obs = 0;
static volatile uint32_t g_start_prologue_pps0_o1_obs = 0;
static volatile uint32_t g_start_prologue_pps0_o2_obs = 0;

// START private-PPS0 continuity gate. The observed-DWT rail decides whether a
// private PPS0 bookend is mature enough to let the next candidate become public
// PPS1. In PhaseLedger mode it remains only a launch witness; it does not author
// the public OCXO nanosecond clock.

// START candidate-readiness court. START interval
// authority is Alpha's ordinary dwt_cycles_between_edges measurement.  The
// process_interrupt-authored dwt_interval_observed_cycles member remains visible
// beside it as audit evidence only; it no longer gates or authors START.
// Missing bits 3..5 retain their original numeric identities for report
// compatibility, but now mean that the corresponding Alpha measurement interval
// is absent or outside the broad lawful one-second band.
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_VCLOCK_SNAPSHOT =
    1U << 0;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_OCXO1_SNAPSHOT =
    1U << 1;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_OCXO2_SNAPSHOT =
    1U << 2;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_VCLOCK_OBSERVED =
    1U << 3;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_OCXO1_OBSERVED =
    1U << 4;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_OCXO2_OBSERVED =
    1U << 5;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_VALID =
    1U << 6;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_INTERVAL =
    1U << 7;
static constexpr uint32_t CLOCKS_START_CANDIDATE_MISSING_EFFECTIVE_CPS =
    1U << 8;

static bool     g_start_candidate_last_ready = false;
static bool     g_start_candidate_last_vclock_snapshot_valid = false;
static bool     g_start_candidate_last_ocxo1_snapshot_valid = false;
static bool     g_start_candidate_last_ocxo2_snapshot_valid = false;
static uint32_t g_start_candidate_last_vclock_observed_interval = 0;
static uint32_t g_start_candidate_last_ocxo1_observed_interval = 0;
static uint32_t g_start_candidate_last_ocxo2_observed_interval = 0;
static uint32_t g_start_candidate_last_vclock_measurement_interval = 0;
static uint32_t g_start_candidate_last_ocxo1_measurement_interval = 0;
static uint32_t g_start_candidate_last_ocxo2_measurement_interval = 0;
static uint32_t g_start_candidate_last_vclock_update_count = 0;
static uint32_t g_start_candidate_last_ocxo1_update_count = 0;
static uint32_t g_start_candidate_last_ocxo2_update_count = 0;
static bool     g_start_candidate_last_selected_reference_valid = false;
static uint32_t g_start_candidate_last_selected_reference_interval = 0;
static uint32_t g_start_candidate_last_effective_cps = 0;
static uint32_t g_start_candidate_last_missing_mask = 0;
static char     g_start_candidate_last_first_problem[48] = "not_checked";

// Compatibility names retained: these now count unusable Alpha measurement
// intervals, not missing process_interrupt audit intervals.

// Campaign-public continuity transform.
//
// Alpha owns the live service/epoch ledgers.  Beta owns campaign presentation:
//   public = raw_alpha_service_value + signed_campaign_offset
//
// START/FLASH_CUT use a negative offset to make the first public campaign
// row begin at zero.  RECOVER uses a positive/negative offset to make the
// current Alpha service ledger appear as the Pi-projected campaign ledger at
// the recovery PPS.  This is deliberately signed; the old unsigned base model
// could not represent current_raw < recovered_public after a system
// restart/SmartZero epoch.
static int64_t g_campaign_public_dwt_offset = 0;
static int64_t g_campaign_public_gnss_offset = 0;
static int64_t g_campaign_public_ocxo1_offset = 0;
static int64_t g_campaign_public_ocxo2_offset = 0;

// Step C: OCXO public/canonical offsets now track the PPS-edge projected
// OCXO clock values. Keep separate offsets for the legacy measured-GNSS
// ledgers so measured_gnss_ns remains useful as a diagnostic side surface.
static int64_t g_campaign_public_ocxo1_measured_offset = 0;
static int64_t g_campaign_public_ocxo2_measured_offset = 0;

// Report-only CounterLedger public transform.  These offsets never author the
// canonical OCXO public clock unless CLOCKS_OCXO_PUBLIC_NS_AUTHORITY is later
// promoted to PPS_COUNTERLEDGER.  In traditional mode they let campaign record carry
// a campaign-aligned CounterLedger candidate beside the existing projection/
// Delta path so long runs can prove whether the integer PPS-sampled ledger is
// stable.
static int64_t g_campaign_public_counterledger_ocxo1_offset = 0;
static int64_t g_campaign_public_counterledger_ocxo2_offset = 0;

// Canonical OCXO residual rendering.  Delta Cycles is now the authority:
// each OCXO observed one-second DWT interval is compared against the exact
// same-row PPS/VCLOCK one-second DWT interval.  The
// public pps_residual object remains a compatibility rendering of the
// canonical Delta result:
//
//   delta:         fast = ref_gnss_cycles - ocxo_observed_cycles
//   pps_residual:  gnss_interval = 1e9, clock_interval = 1e9 - fast_ns
//
// Projected-GNSS residuals remain preserved under science.traditional_*
// for courtroom/report comparison only.
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

// Delta science totals.  Canonical one-second residuals remain physical
// interval sums: for a fast clock the one-second period is shorter, so
//   clock_interval = reference_interval - fast_residual.
//
// Campaign TAU/PPB publication, however, is now a clockface ledger ratio:
//   total_tau = public_clock_ns / public_gnss_ns
//   total_ppb = (total_tau - 1) * 1e9
//
// That ratio is applied after the public OCXO nanosecond value is rendered.
// The interval totals below are still accumulated so residual/Welford/servo
// diagnostics keep their existing Delta Cycles evidence surface.
// Traditional projected-GNSS totals are kept beside this for audit.
static constexpr uint64_t CLOCKS_BETA_NS_PER_SECOND = 1000000000ULL;

// Delta Raw sanity gate.  Delta Raw is only valid when both the same-row
// PPS/VCLOCK reference interval and the OCXO observed interval are plausible
// one-second DWT spans.  Sentinel / uninitialized values such as 0xFFFFFFFF
// must remain evidence, but they must never publish with delta_raw_valid=true.
// Keep the band deliberately broad: this is corruption/sentinel gating, not a
// tight clock-quality gate.
static constexpr uint32_t CLOCKS_DELTA_RAW_INTERVAL_MIN_CYCLES = 900000000UL;
static constexpr uint32_t CLOCKS_DELTA_RAW_INTERVAL_MAX_CYCLES = 1100000000UL;
static constexpr uint32_t CLOCKS_DELTA_RAW_INTERVAL_SENTINEL_U32 = 0xFFFFFFFFUL;

static uint32_t g_delta_raw_interval_reject_count = 0;
static uint32_t g_delta_raw_interval_reject_reference_count = 0;
static uint32_t g_delta_raw_interval_reject_clock_count = 0;
static uint32_t g_delta_raw_interval_sentinel_reject_count = 0;
static uint32_t g_delta_raw_interval_plausibility_reject_count = 0;
static uint32_t g_delta_raw_interval_last_public_count = 0;
static uint32_t g_delta_raw_interval_last_clock_id = 0;
static uint32_t g_delta_raw_interval_last_reference_cycles = 0;
static uint32_t g_delta_raw_interval_last_clock_cycles = 0;
static char     g_delta_raw_interval_last_reject_field[16] = "";
static char     g_delta_raw_interval_last_reject_reason[64] = "OK";

struct ocxo_science_totals_t {
  // Delta Cycles residual accumulation.  These interval sums are used to
  // render the public OCXO nanosecond ledger and to preserve the residual
  // evidence, but published total_tau/total_ppb are overwritten later with the
  // campaign clockface ratio public_ocxo_ns / public_gnss_ns.
  uint32_t sample_count = 0;
  uint64_t clock_interval_total_ns = 0;
  uint64_t gnss_interval_total_ns = 0;
  double   clock_interval_total_ns_exact = 0.0;
  double   gnss_interval_total_ns_exact = 0.0;

  // Preserved traditional projected-GNSS residual totals.  These remain
  // courtroom/comparison evidence only after Delta Cycles becomes canonical.
  uint32_t traditional_sample_count = 0;
  uint64_t traditional_clock_interval_total_ns = 0;
  uint64_t traditional_gnss_interval_total_ns = 0;
  double   traditional_clock_interval_total_ns_exact = 0.0;
  double   traditional_gnss_interval_total_ns_exact = 0.0;
};

static ocxo_science_totals_t g_ocxo_science_totals_ocxo1 DMAMEM = {};
static ocxo_science_totals_t g_ocxo_science_totals_ocxo2 DMAMEM = {};

struct delta_residual_reference_t {
  bool     captured = false;
  bool     gnss_available = false;
  bool     gnss_valid = false;
  bool     raw_valid = false;
  uint32_t public_count = 0;
  uint32_t gnss_interval_cycles = 0;
  uint32_t raw_interval_cycles = 0;
};


static delta_residual_reference_t g_delta_previous_vclock_reference DMAMEM = {};

// RAM1 relief scratch.  These are per-Beta-cycle temporary witnesses that used
// to be automatic locals in START/prologue and 1 Hz publication paths.  Keeping
// them in RAM2 avoids compiler-generated stack memset in the same upper-DTCM
// neighborhood reported by CrashReport.
static clocks_alpha_lane_forensics_t g_beta_pps_vclock_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t g_beta_pps_ocxo1_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t g_beta_pps_ocxo2_forensics DMAMEM = {};

static clocks_alpha_ocxo_pps_projection_snapshot_t
    g_beta_start_ocxo1_projection DMAMEM = {};
static clocks_alpha_ocxo_pps_projection_snapshot_t
    g_beta_start_ocxo2_projection DMAMEM = {};
static clocks_alpha_ocxo_pps_projection_snapshot_t
    g_beta_pps_ocxo1_projection DMAMEM = {};
static clocks_alpha_ocxo_pps_projection_snapshot_t
    g_beta_pps_ocxo2_projection DMAMEM = {};

static clocks_pps_vclock_edge_forensics_t
    g_beta_pps_vclock_edge_forensics DMAMEM = {};
static clocks_alpha_tau_snapshot_t g_beta_pps_ocxo1_alpha_tau DMAMEM = {};
static clocks_alpha_tau_snapshot_t g_beta_pps_ocxo2_alpha_tau DMAMEM = {};
static clocks_instrument_stats_snapshot_t
    g_beta_instrument_stats DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_pps_cycle_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_vclock_cycle_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_ocxo1_cycle_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_ocxo2_cycle_prediction DMAMEM = {};

// Report/recovery scratch objects.  These command-path snapshots used to be
// automatic locals in large report builders.  Keep them in RAM2 so asking for
// diagnostics cannot consume the scarce DTCM stack we are trying to measure.
static interrupt_smartzero_snapshot_t
    g_beta_report_live_smartzero_scratch DMAMEM = {};
static interrupt_smartzero_snapshot_t
    g_beta_report_installed_smartzero_scratch DMAMEM = {};
static clocks_alpha_smartzero_delay_snapshot_t
    g_beta_report_smartzero_delay_scratch DMAMEM = {};
static clocks_alpha_ocxo_visible_origin_snapshot_t
    g_beta_report_visible_origin_scratch DMAMEM = {};
struct clock_science_row_t {
  // valid means the row contains numeric material usable by the currently
  // unified row-admission contract. science_worthy is the numeric court verdict.
  bool     valid = false;
  bool     science_worthy = false;
  bool     antecedents_complete = false;
  uint32_t clock_id = 0;
  uint32_t public_count = 0;

  uint32_t pps_vclock_dwt_at_edge = 0;
  uint64_t pps_vclock_gnss_ns_at_edge = 0;
  uint32_t projection_cps_cycles = 0;

  uint32_t clock_published_dwt_at_edge = 0;
  uint32_t clock_raw_dwt_at_edge = 0;
  uint32_t clock_observed_interval_cycles = 0;
  uint32_t clock_effective_interval_cycles = 0;

  // Delta residuals: OCXO one-second DWT interval minus the exact same-row
  // PPS/VCLOCK one-second DWT interval.
  // The *_residual_cycles fields are Dave-subtraction signed
  // (clock_interval - reference_interval).  The *_fast_residual_* mirrors
  // the project-wide sign convention: positive means the clock is fast.
  bool     delta_raw_valid = false;
  bool     delta_raw_reference_plausible = false;
  bool     delta_raw_clock_plausible = false;
  uint32_t delta_reference_public_count = 0;
  uint32_t delta_publication_public_count = 0;
  uint32_t delta_raw_reference_interval_cycles = 0;
  uint32_t delta_raw_clock_interval_cycles = 0;
  int64_t  delta_raw_residual_cycles = 0;
  int64_t  delta_raw_fast_residual_cycles = 0;
  int64_t  delta_raw_residual_ns = 0;
  int64_t  delta_raw_fast_residual_ns = 0;
  double   delta_raw_residual_ns_exact = 0.0;
  double   delta_raw_fast_residual_ns_exact = 0.0;
  int64_t  delta_raw_fast_minus_traditional_ns = 0;

  // Traditional projected-GNSS residual surface preserved for reports.
  // After the Delta Cycles promotion, the unprefixed residual fields below
  // are canonical Delta renderings; these fields retain the old projection.
  bool     traditional_valid = false;
  uint64_t traditional_gnss_interval_ns = 0;
  uint64_t traditional_clock_interval_ns = 0;
  int64_t  traditional_fast_residual_ns = 0;
  double   traditional_gnss_interval_ns_exact = 0.0;
  double   traditional_clock_interval_ns_exact = 0.0;
  double   traditional_fast_residual_ns_exact = 0.0;
  double   traditional_tau_1s = 1.0;
  double   traditional_ppb_1s = 0.0;

  bool     traditional_total_valid = false;
  uint32_t traditional_total_sample_count = 0;
  uint64_t traditional_total_clock_interval_ns = 0;
  uint64_t traditional_total_gnss_interval_ns = 0;
  int64_t  traditional_total_fast_residual_ns = 0;
  double   traditional_total_clock_interval_ns_exact = 0.0;
  double   traditional_total_gnss_interval_ns_exact = 0.0;
  double   traditional_total_fast_residual_ns_exact = 0.0;
  double   traditional_total_tau = 1.0;
  double   traditional_total_ppb = 0.0;

  int64_t  prior_edge_gnss_ns = 0;
  int64_t  current_edge_gnss_ns = 0;
  double   prior_edge_gnss_ns_exact = 0.0;
  double   current_edge_gnss_ns_exact = 0.0;
  uint64_t gnss_interval_ns = 0;
  uint64_t clock_interval_ns = 0;
  int64_t  fast_residual_ns = 0;
  double   gnss_interval_ns_exact = 0.0;
  double   clock_interval_ns_exact = 0.0;
  double   fast_residual_ns_exact = 0.0;
  double   tau_1s = 1.0;
  double   ppb_1s = 0.0;

  bool     total_valid = false;
  uint32_t total_sample_count = 0;
  uint64_t total_clock_interval_ns = 0;
  uint64_t total_gnss_interval_ns = 0;
  int64_t  total_fast_residual_ns = 0;
  double   total_clock_interval_ns_exact = 0.0;
  double   total_gnss_interval_ns_exact = 0.0;
  double   total_fast_residual_ns_exact = 0.0;
  double   total_tau = 1.0;
  double   total_ppb = 0.0;

  // Alpha-owned always-on PhaseLedger TAU snapshot. These fields remain
  // side-channel frequency evidence.  Panel-facing campaign TAU/PPB is the
  // continuity-aligned public clockface ratio so RECOVER does not appear to
  // restart the OCXO totals from a fresh reattachment intercept.
  bool     alpha_tau_valid = false;
  uint32_t alpha_tau_sample_count = 0;
  uint32_t alpha_tau_interval_count = 0;
  uint32_t alpha_tau_last_pps_sequence = 0;
  uint32_t alpha_tau_last_interval_pps_sequence = 0;
  double   alpha_tau = 1.0;
  double   alpha_tau_ppb = 0.0;
  double   alpha_tau_stderr_ppb = 0.0;
  double   alpha_tau_interval_mean_ppb = 0.0;
  double   alpha_tau_interval_stderr_ppb = 0.0;
  int64_t  alpha_tau_intercept_ns = 0;
  int64_t  alpha_tau_detrended_fast_residual_ns = 0;
};

static clock_science_row_t g_beta_vclock_science_row DMAMEM = {};
static clock_science_row_t g_beta_ocxo1_science_row DMAMEM = {};
static clock_science_row_t g_beta_ocxo2_science_row DMAMEM = {};
static clock_science_row_t g_beta_probe_ocxo1_science_row DMAMEM = {};
static clock_science_row_t g_beta_probe_ocxo2_science_row DMAMEM = {};
static ocxo_science_totals_t g_beta_probe_science_totals_o1 DMAMEM = {};
static ocxo_science_totals_t g_beta_probe_science_totals_o2 DMAMEM = {};
static ocxo_science_totals_t g_beta_row_science_totals_o1_before DMAMEM = {};
static ocxo_science_totals_t g_beta_row_science_totals_o2_before DMAMEM = {};

static const char* delta_raw_interval_reject_reason(uint32_t cycles) {
  if (cycles == 0U) return "interval_zero";
  if (cycles == CLOCKS_DELTA_RAW_INTERVAL_SENTINEL_U32) {
    return "interval_sentinel_0xffffffff";
  }
  if (cycles < CLOCKS_DELTA_RAW_INTERVAL_MIN_CYCLES) {
    return "interval_below_plausible_min";
  }
  if (cycles > CLOCKS_DELTA_RAW_INTERVAL_MAX_CYCLES) {
    return "interval_above_plausible_max";
  }
  return "OK";
}

static bool delta_raw_interval_cycles_plausible(uint32_t cycles) {
  return strcmp(delta_raw_interval_reject_reason(cycles), "OK") == 0;
}

static void delta_raw_interval_note_reject(const clock_science_row_t& row,
                                           const char* field,
                                           uint32_t reference_cycles,
                                           uint32_t clock_cycles,
                                           const char* reason) {
  g_delta_raw_interval_reject_count++;
  if (field && strcmp(field, "reference") == 0) {
    g_delta_raw_interval_reject_reference_count++;
  } else {
    g_delta_raw_interval_reject_clock_count++;
  }

  if (reason && strstr(reason, "sentinel") != nullptr) {
    g_delta_raw_interval_sentinel_reject_count++;
  } else {
    g_delta_raw_interval_plausibility_reject_count++;
  }

  g_delta_raw_interval_last_public_count = row.public_count;
  g_delta_raw_interval_last_clock_id = row.clock_id;
  g_delta_raw_interval_last_reference_cycles = reference_cycles;
  g_delta_raw_interval_last_clock_cycles = clock_cycles;
  safeCopy(g_delta_raw_interval_last_reject_field,
           sizeof(g_delta_raw_interval_last_reject_field),
           field ? field : "clock");
  safeCopy(g_delta_raw_interval_last_reject_reason,
           sizeof(g_delta_raw_interval_last_reject_reason),
           reason ? reason : "interval_rejected");
}

static void delta_residual_state_reset(void) {
  g_delta_previous_vclock_reference = delta_residual_reference_t{};
}

static void ocxo_science_totals_reset(void) {
  g_ocxo_science_totals_ocxo1 = ocxo_science_totals_t{};
  g_ocxo_science_totals_ocxo2 = ocxo_science_totals_t{};
  delta_residual_state_reset();
}

// RECOVER deliberately cuts the campaign publication stream while preserving
// the installed service epoch.  Alpha re-primes OCXO edge state at the RECOVER
// gate; Beta additionally quarantines the first published science residual rows
// so no Welford sample can be formed from a pre-recovery previous edge or a
// one-edge bridge warm-up artifact.  campaign record rows themselves are not hidden.
static constexpr uint32_t CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS = 2U;
static uint32_t g_science_residual_quarantine_remaining = 0;
static uint32_t g_science_residual_quarantine_begin_count = 0;
static uint32_t g_science_residual_quarantine_consumed_count = 0;
static uint32_t g_science_residual_quarantine_last_public_count = 0;

// Alpha-owned instrument statistics continue across RECOVER.  Beta still
// quarantines campaign science/servo inputs formed across a recovery custody
// boundary; the raw interval remains visible in campaign record forensics.

// RECOVER OCXO reattachment gate.  Alpha deliberately cuts OCXO measurement
// custody during warm recovery.  Beta initially treats recovered candidates as
// private elapsed seconds while waiting for both OCXO lanes to prove fresh
// reattachment evidence.  This gate must be finite: after timeout, campaign
// publication resumes in degraded mode and OCXO science remains quarantined/
// invalid until PhaseLedger/reattach evidence catches up.
static constexpr uint32_t CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES = 32U;
// Recovery publication is layered:
//   * Timeline readiness (PPS/VCLOCK, GNSS, DWT) permits a public row.
//   * OCXO clockface readiness proves a fresh post-RECOVER integer ledger.
//   * OCXO science readiness additionally proves the refined PhaseLedger
//     interval required by Welford, PPB, and servo.
//
// The Pi is expected to persist explicitly degraded timeline rows instead of
// restarting RECOVER.  A later science-ready row naturally clears degradation.
static constexpr bool     CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED = true;
static volatile bool     g_recover_reattach_active = false;
static volatile bool     g_recover_reattach_degraded_active = false;
static volatile bool     g_recover_reattach_clockface_ready = false;
static volatile bool     g_recover_reattach_science_ready = false;
static uint32_t          g_recover_reattach_begin_count = 0;
static uint32_t          g_recover_reattach_hold_count = 0;
static uint32_t          g_recover_reattach_release_count = 0;
static uint32_t          g_recover_reattach_timeout_count = 0;
static uint32_t          g_recover_reattach_degraded_release_count = 0;
static uint32_t          g_recover_reattach_degraded_clear_count = 0;
static uint32_t          g_recover_reattach_degraded_public_row_count = 0;
static uint32_t          g_recover_reattach_degraded_science_suppressed_count = 0;
// Stall means no movement toward the currently missing OCXO proof, not merely
// a fixed number of degraded rows.  Sixty 1 Hz rows gives the resolver a full
// minute before a dedicated CLOCKS_RECOVERY_STALLED event is emitted.
static constexpr uint32_t CLOCKS_RECOVER_REATTACH_DEGRADED_STALL_CANDIDATES = 60U;
static volatile bool     g_recover_reattach_stalled = false;
static uint32_t          g_recover_reattach_stall_count = 0;
static uint32_t          g_recover_reattach_stall_publish_count = 0;
static uint32_t          g_recover_reattach_progress_count = 0;
static uint32_t          g_recover_reattach_progress_resume_count = 0;
static uint32_t          g_recover_reattach_no_progress_row_count = 0;
static uint32_t          g_recover_reattach_degraded_window_row_count = 0;
static uint32_t          g_recover_reattach_last_progress_public_count = 0;
static uint32_t          g_recover_reattach_last_stall_public_count = 0;
static char              g_recover_reattach_stall_reason[64] = "idle";
static uint32_t          g_recover_reattach_hidden_candidate_count = 0;
static uint32_t          g_recover_reattach_last_hidden_public_count = 0;
static uint32_t          g_recover_reattach_last_release_public_count = 0;
static uint32_t          g_recover_reattach_last_degraded_release_public_count = 0;
static uint32_t          g_recover_reattach_last_degraded_public_count = 0;
static char              g_recover_reattach_last_reason[64] = "idle";
static clocks_alpha_recover_reattach_snapshot_t
    g_recover_reattach_last_ocxo1 DMAMEM = {};
static clocks_alpha_recover_reattach_snapshot_t
    g_recover_reattach_last_ocxo2 DMAMEM = {};

// Recovery stall detection tracks one-way readiness milestones, not ordinary
// per-second activity.  Capture/sample/forensics counters advance on a healthy
// degraded stream even when the missing PhaseLedger proof never gets closer;
// counting them as progress makes a true science stall impossible to detect.
// The seen mask is monotonic for one RECOVER generation: a bit can prove new
// progress only once.
struct recover_reattach_progress_marker_t {
  uint32_t seen_readiness_mask = 0;
};

static bool g_recover_reattach_progress_marker_valid = false;
static recover_reattach_progress_marker_t g_recover_reattach_progress_ocxo1 = {};
static recover_reattach_progress_marker_t g_recover_reattach_progress_ocxo2 = {};

// Recovery request flight recorder for Pi-side polling.  These values make
// REPORT_RECOVERY useful while core.py waits for the first public pair.
static uint32_t          g_recover_request_count = 0;
static char              g_recover_last_campaign[64] = {0};
static bool              g_recover_last_campaign_supplied = false;
static uint64_t          g_recover_last_base_count = 0;
static uint64_t          g_recover_last_expected_first_public_count = 0;
static uint64_t          g_recover_last_base_gnss_ns = 0;
static uint64_t          g_recover_last_base_dwt_ns = 0;
static uint64_t          g_recover_last_base_ocxo1_ns = 0;
static uint64_t          g_recover_last_base_ocxo2_ns = 0;

// RECOVER lifecycle flight recorder.  RECOVER is intentionally promoted to a
// visible campaign state at command time so watchdog recovery cannot leave the
// firmware in STOPPED + request_recover limbo while Pi waits for campaign record.
static uint32_t          g_recover_lifecycle_begin_count = 0;
static uint32_t          g_recover_lifecycle_pps_gate_count = 0;
static uint32_t          g_recover_lifecycle_command_custody_reset_count = 0;
static uint32_t          g_recover_lifecycle_gate_custody_reset_count = 0;
static uint32_t          g_recover_lifecycle_interrupt_service_rearm_count = 0;
static uint32_t          g_recover_lifecycle_interrupt_service_rearm_failure_count = 0;
static bool              g_recover_lifecycle_last_interrupt_service_rearm_ok = false;

enum class recover_lifecycle_mode_t : uint8_t {
  NONE           = 0,
  LIVE_REATTACH  = 1,
  COLD_BOOTSTRAP = 2,
};

static volatile recover_lifecycle_mode_t g_recover_lifecycle_mode =
    recover_lifecycle_mode_t::NONE;
static bool     g_recover_lifecycle_cold_bootstrap_epoch_ready = false;
static uint32_t g_recover_lifecycle_cold_bootstrap_begin_count = 0;
static uint32_t g_recover_lifecycle_cold_bootstrap_wait_count = 0;
static uint32_t g_recover_lifecycle_cold_bootstrap_ready_count = 0;
static uint32_t g_recover_lifecycle_cold_bootstrap_commit_count = 0;
static uint32_t g_recover_lifecycle_cold_bootstrap_start_failure_count = 0;
static uint32_t          g_recover_lifecycle_complete_count = 0;
static uint32_t          g_recover_lifecycle_abort_count = 0;
static uint32_t          g_recover_lifecycle_stale_gate_count = 0;
static uint32_t          g_recover_lifecycle_last_begin_campaign_seconds = 0;
static uint32_t          g_recover_lifecycle_last_gate_campaign_seconds = 0;
static uint32_t          g_recover_lifecycle_last_abort_campaign_seconds = 0;
static char              g_recover_lifecycle_reason[64] = "idle";
static char              g_recover_lifecycle_abort_reason[64] = "none";

// RECOVER presentation continuity.  A cold SmartZero epoch creates a fresh
// local CounterLedger/PhaseLedger intercept, while the Pi-projected campaign
// clockface must continue across the outage.  Arm one signed presentation
// transform at the recovery splice and apply it on the first public row whose
// CounterLedger clockfaces exist, including an explicitly degraded row.
// One-second science, Welfords, and servo NOW/MEAN inputs remain independently
// gated until fresh post-recovery interval custody is complete.
static volatile bool g_recover_continuity_align_pending = false;
static uint32_t g_recover_continuity_align_count = 0;
static uint32_t g_recover_continuity_align_failure_count = 0;
static uint32_t g_recover_continuity_align_last_public_count = 0;
static uint32_t g_recover_continuity_align_requested_public_count = 0;
static uint64_t g_recover_continuity_ocxo1_target_ns = 0;
static uint64_t g_recover_continuity_ocxo2_target_ns = 0;
static uint64_t g_recover_continuity_ocxo1_before_ns = 0;
static uint64_t g_recover_continuity_ocxo2_before_ns = 0;
static uint64_t g_recover_continuity_ocxo1_after_ns = 0;
static uint64_t g_recover_continuity_ocxo2_after_ns = 0;
static int64_t  g_recover_continuity_ocxo1_correction_ns = 0;
static int64_t  g_recover_continuity_ocxo2_correction_ns = 0;
static char     g_recover_continuity_last_reason[64] = "idle";

static FLASHMEM void recover_reattach_reset(const char* reason);
static FLASHMEM void recover_reattach_begin(void);
static FLASHMEM bool recover_reattach_should_hold(void);
static FLASHMEM bool recover_reattach_degraded_science_hold_active(void);
static FLASHMEM void recover_reattach_apply_degraded_science_hold(clock_science_row_t& row);

static uint32_t clocks_row_lifecycle_science_hold_flags(void) {
  uint32_t flags = 0U;
  if (g_recover_reattach_active) flags |= 1U << 0;
  if (g_recover_reattach_degraded_active) flags |= 1U << 1;
  if (g_science_residual_quarantine_remaining != 0U) flags |= 1U << 2;
  if (g_recover_continuity_align_pending) flags |= 1U << 3;
  return flags;
}

static void pps_interval_residuals_reset(void) {
  ocxo_science_totals_reset();
  g_science_residual_quarantine_remaining = 0;
  clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                 g_clocks_feature_science_residuals,
                                 system_feature_status_t::INITIALIZING,
                                 true);
}

static void pps_interval_residuals_begin_recover_quarantine(uint32_t rows) {
  pps_interval_residuals_reset();
  g_science_residual_quarantine_remaining = rows;
  g_science_residual_quarantine_begin_count++;
}

static uint64_t science_render_legacy_clock_interval_ns(
    const clock_science_row_t& row) {
  if (!row.valid) return 0ULL;

  if (clocks_ocxo_counterledger_mode_enabled()) {
    // CounterLedger mode changes public clockface authority, not this row's
    // canonical one-second residual.  The row is still the Delta rendering.
    return row.clock_interval_ns;
  }

  // Traditional Delta rendering: positive fast means the physical clock period
  // was shorter than the GNSS reference second.
  const int64_t rendered =
      (int64_t)CLOCKS_BETA_NS_PER_SECOND - row.fast_residual_ns;
  return (rendered > 0) ? (uint64_t)rendered : 0ULL;
}

static uint64_t science_render_public_clock_ns(
    uint64_t public_gnss_ns,
    const clock_science_row_t& row,
    uint64_t fallback_public_ns) {
  if (clocks_ocxo_counterledger_mode_enabled()) {
    // CounterLedger public ns is already authored by Alpha from the
    // PPS-captured OCXO counter.  Do not re-render it from Delta totals.
    (void)public_gnss_ns;
    (void)row;
    return fallback_public_ns;
  }

  if (!row.total_valid) return fallback_public_ns;

  if (row.total_fast_residual_ns >= 0) {
    const uint64_t add = (uint64_t)row.total_fast_residual_ns;
    return (UINT64_MAX - public_gnss_ns < add)
        ? UINT64_MAX
        : public_gnss_ns + add;
  }

  const uint64_t sub = (uint64_t)(-row.total_fast_residual_ns);
  return (public_gnss_ns >= sub) ? (public_gnss_ns - sub) : 0ULL;
}

static pps_interval_residuals_t pps_interval_residuals_update(
    uint32_t public_count,
    const clock_science_row_t& ocxo1_science,
    const clock_science_row_t& ocxo2_science) {
  pps_interval_residuals_t r{};
  r.public_count = public_count;

  r.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;

  if (ocxo1_science.valid) {
    r.ocxo1_valid = true;
    r.ocxo1_interval_ns =
        science_render_legacy_clock_interval_ns(ocxo1_science);
    r.ocxo1_fast_residual_ns = ocxo1_science.fast_residual_ns;
  }

  if (ocxo2_science.valid) {
    r.ocxo2_valid = true;
    r.ocxo2_interval_ns =
        science_render_legacy_clock_interval_ns(ocxo2_science);
    r.ocxo2_fast_residual_ns = ocxo2_science.fast_residual_ns;
  }

  if (ocxo1_science.science_worthy &&
      ocxo2_science.science_worthy) {
    clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                   g_clocks_feature_science_residuals,
                                   system_feature_status_t::NOMINAL);
  }
  return r;
}

static uint64_t current_raw_gnss_ns(void) {
  // At the selected PPS/VCLOCK edge, GNSS time is identity, not discovery.
  // The DWT projection path remains an Alpha edge self-check; it must not
  // author the public GNSS ledger because doing so turns bridge rounding/
  // projection error into a common-mode OCXO residual.
  return g_gnss_ns_at_pps_vclock;
}

static uint64_t current_raw_ocxo1_measured_ns(void) {
  // Legacy measured GNSS-elapsed ledger retained as a diagnostic surface.
  return clocks_ocxo1_measured_gnss_ns_now();
}

static uint64_t current_raw_ocxo2_measured_ns(void) {
  // Legacy measured GNSS-elapsed ledger retained as a diagnostic surface.
  return clocks_ocxo2_measured_gnss_ns_now();
}

static uint64_t current_raw_counterledger_ns(time_clock_id_t clock) {
  g_beta_counterledger_raw_scratch = clocks_alpha_ocxo_counterledger_snapshot_t{};
  if (!clocks_alpha_ocxo_counterledger_snapshot(clock,
                                                &g_beta_counterledger_raw_scratch) ||
      !g_beta_counterledger_raw_scratch.valid) {
    return 0ULL;
  }
  return g_beta_counterledger_raw_scratch.refined_valid
      ? g_beta_counterledger_raw_scratch.refined_ns
      : g_beta_counterledger_raw_scratch.ns;
}

static uint64_t current_raw_ocxo1_ns(void) {
  if (clocks_ocxo_counterledger_mode_enabled()) {
    const uint64_t ns = current_raw_counterledger_ns(time_clock_id_t::OCXO1);
    if (ns != 0ULL) return ns;
  }

  // Traditional authority uses Alpha's PPS-row measured/projection surface.
  // CounterLedger mode falls back here only before the PPS-sampled ledger has
  // produced its first valid row, which START gates prevent from becoming
  // public campaign output.
  return current_raw_ocxo1_measured_ns();
}

static uint64_t current_raw_ocxo2_ns(void) {
  if (clocks_ocxo_counterledger_mode_enabled()) {
    const uint64_t ns = current_raw_counterledger_ns(time_clock_id_t::OCXO2);
    if (ns != 0ULL) return ns;
  }

  // Traditional authority uses Alpha's PPS-row measured/projection surface.
  // CounterLedger mode falls back here only before the PPS-sampled ledger has
  // produced its first valid row, which START gates prevent from becoming
  // public campaign output.
  return current_raw_ocxo2_measured_ns();
}

static int64_t campaign_public_offset_for_recovered_value(uint64_t current,
                                                           uint64_t recovered) {
  if (recovered >= current) {
    const uint64_t delta = recovered - current;
    return (delta > (uint64_t)INT64_MAX) ? INT64_MAX : (int64_t)delta;
  }

  const uint64_t delta = current - recovered;
  return (delta > (uint64_t)INT64_MAX) ? -INT64_MAX : -(int64_t)delta;
}

static int64_t campaign_public_offset_to_zero(uint64_t current) {
  return (current > (uint64_t)INT64_MAX) ? -INT64_MAX : -(int64_t)current;
}

static uint64_t campaign_public_from_offset(uint64_t raw, int64_t offset) {
  if (offset >= 0) {
    const uint64_t add = (uint64_t)offset;
    return (UINT64_MAX - raw < add) ? UINT64_MAX : raw + add;
  }

  const uint64_t sub = (uint64_t)(-offset);
  return (raw >= sub) ? (raw - sub) : 0ULL;
}


static void instrument_continuity_reset(void) {
  g_instrument_continuity_active = false;
  g_instrument_gnss_offset = 0;
  g_instrument_dwt_offset = 0;
  g_instrument_ocxo1_offset = 0;
  g_instrument_ocxo2_offset = 0;
  g_instrument_continuity_reset_count++;
}

static void instrument_continuity_install(
    const clocks_recovery_restore_state_t& state) {
  g_instrument_gnss_offset = campaign_public_offset_for_recovered_value(
      current_raw_gnss_ns(), state.instrument_gnss_ns);
  g_instrument_dwt_offset = campaign_public_offset_for_recovered_value(
      g_dwt_cycle_count_total, state.instrument_dwt_cycles);
  g_instrument_ocxo1_offset = campaign_public_offset_for_recovered_value(
      current_raw_ocxo1_measured_ns(), state.instrument_ocxo1_ns);
  g_instrument_ocxo2_offset = campaign_public_offset_for_recovered_value(
      current_raw_ocxo2_measured_ns(), state.instrument_ocxo2_ns);
  g_instrument_continuity_active = true;
  g_instrument_continuity_install_count++;
}

static uint64_t instrument_continuity_gnss_ns(uint64_t raw) {
  return g_instrument_continuity_active
      ? campaign_public_from_offset(raw, g_instrument_gnss_offset)
      : raw;
}

static uint64_t instrument_continuity_dwt_cycles(uint64_t raw) {
  return g_instrument_continuity_active
      ? campaign_public_from_offset(raw, g_instrument_dwt_offset)
      : raw;
}

static uint64_t instrument_continuity_ocxo1_ns(uint64_t raw) {
  return g_instrument_continuity_active
      ? campaign_public_from_offset(raw, g_instrument_ocxo1_offset)
      : raw;
}

static uint64_t instrument_continuity_ocxo2_ns(uint64_t raw) {
  return g_instrument_continuity_active
      ? campaign_public_from_offset(raw, g_instrument_ocxo2_offset)
      : raw;
}

static int64_t campaign_recover_signed_delta_u64(uint64_t lhs, uint64_t rhs) {
  return (lhs >= rhs)
      ? ((lhs - rhs) > (uint64_t)INT64_MAX ? INT64_MAX : (int64_t)(lhs - rhs))
      : ((rhs - lhs) > (uint64_t)INT64_MAX ? -INT64_MAX : -(int64_t)(rhs - lhs));
}

static int64_t campaign_recover_round_double_to_i64(double value) {
  return (value >= 0.0)
      ? (int64_t)(value + 0.5)
      : (int64_t)(value - 0.5);
}

static uint64_t campaign_recover_project_ocxo_to_public_gnss(
    uint64_t public_gnss_ns,
    uint64_t recovered_ocxo_ns) {
  if (public_gnss_ns == 0ULL || recover_gnss_ns == 0ULL ||
      recovered_ocxo_ns == 0ULL) {
    return 0ULL;
  }

  // Preserve the pre-recovery campaign clockface ratio without multiplying two
  // large nanosecond ledgers.  The signed offset is small compared to GNSS ns,
  // so scaling the offset avoids uint64 overflow and keeps ns precision.
  const int64_t recovered_offset_ns =
      campaign_recover_signed_delta_u64(recovered_ocxo_ns, recover_gnss_ns);
  const double scale = (double)public_gnss_ns / (double)recover_gnss_ns;
  const int64_t projected_offset_ns =
      campaign_recover_round_double_to_i64((double)recovered_offset_ns * scale);
  return campaign_public_from_offset(public_gnss_ns, projected_offset_ns);
}

static void recover_continuity_set_reason(const char* reason) {
  safeCopy(g_recover_continuity_last_reason,
           sizeof(g_recover_continuity_last_reason),
           reason ? reason : "recover_continuity");
}

static void recover_continuity_align_arm(void) {
  g_recover_continuity_align_pending = true;
  g_recover_continuity_align_requested_public_count =
      (uint32_t)(campaign_seconds + 1ULL);
  recover_continuity_set_reason("armed_for_first_recovered_public_row");
}

static void recover_continuity_align_reset(const char* reason) {
  g_recover_continuity_align_pending = false;
  g_recover_continuity_align_requested_public_count = 0;
  recover_continuity_set_reason(reason ? reason : "reset");
}

static void recover_continuity_align_if_pending(uint32_t public_count,
                                                uint64_t public_gnss_ns) {
  if (!g_recover_continuity_align_pending) return;

  const uint64_t raw_c1 =
      current_raw_counterledger_ns(time_clock_id_t::OCXO1);
  const uint64_t raw_c2 =
      current_raw_counterledger_ns(time_clock_id_t::OCXO2);
  const bool counterledger_authority =
      clocks_ocxo_counterledger_mode_enabled();
  const uint64_t raw_o1 = counterledger_authority
      ? raw_c1
      : current_raw_ocxo1_ns();
  const uint64_t raw_o2 = counterledger_authority
      ? raw_c2
      : current_raw_ocxo2_ns();
  const uint64_t target_o1 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo1_ns);
  const uint64_t target_o2 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo2_ns);

  if (target_o1 == 0ULL || target_o2 == 0ULL) {
    g_recover_continuity_align_pending = false;
    g_recover_continuity_align_failure_count++;
    g_recover_continuity_ocxo1_target_ns = target_o1;
    g_recover_continuity_ocxo2_target_ns = target_o2;
    recover_continuity_set_reason("missing_recovered_target");
    return;
  }

  if (raw_o1 == 0ULL || raw_o2 == 0ULL) {
    // CounterLedger authority may not exist at the RECOVER command boundary on
    // a freshly flashed Teensy.  Keep the one-shot armed until the first exact
    // post-SmartZero row supplies both local clockfaces; publishing them with a
    // zero/stale offset would leak the fresh local epoch into campaign time.
    g_recover_continuity_ocxo1_target_ns = target_o1;
    g_recover_continuity_ocxo2_target_ns = target_o2;
    recover_continuity_set_reason(counterledger_authority
        ? "waiting_for_counterledger_clockface"
        : "waiting_for_raw_ocxo_clockface");
    return;
  }

  const uint64_t before_o1 =
      campaign_public_from_offset(raw_o1, g_campaign_public_ocxo1_offset);
  const uint64_t before_o2 =
      campaign_public_from_offset(raw_o2, g_campaign_public_ocxo2_offset);

  g_campaign_public_ocxo1_offset =
      campaign_public_offset_for_recovered_value(raw_o1, target_o1);
  g_campaign_public_ocxo2_offset =
      campaign_public_offset_for_recovered_value(raw_o2, target_o2);

  const uint64_t raw_m1 = current_raw_ocxo1_measured_ns();
  const uint64_t raw_m2 = current_raw_ocxo2_measured_ns();
  if (raw_m1 != 0ULL) {
    g_campaign_public_ocxo1_measured_offset =
        campaign_public_offset_for_recovered_value(raw_m1, target_o1);
  }
  if (raw_m2 != 0ULL) {
    g_campaign_public_ocxo2_measured_offset =
        campaign_public_offset_for_recovered_value(raw_m2, target_o2);
  }

  if (raw_c1 != 0ULL) {
    g_campaign_public_counterledger_ocxo1_offset =
        campaign_public_offset_for_recovered_value(raw_c1, target_o1);
  }
  if (raw_c2 != 0ULL) {
    g_campaign_public_counterledger_ocxo2_offset =
        campaign_public_offset_for_recovered_value(raw_c2, target_o2);
  }

  g_recover_continuity_align_pending = false;
  g_recover_continuity_align_count++;
  g_recover_continuity_align_last_public_count = public_count;
  g_recover_continuity_ocxo1_target_ns = target_o1;
  g_recover_continuity_ocxo2_target_ns = target_o2;
  g_recover_continuity_ocxo1_before_ns = before_o1;
  g_recover_continuity_ocxo2_before_ns = before_o2;
  g_recover_continuity_ocxo1_after_ns =
      campaign_public_from_offset(raw_o1, g_campaign_public_ocxo1_offset);
  g_recover_continuity_ocxo2_after_ns =
      campaign_public_from_offset(raw_o2, g_campaign_public_ocxo2_offset);
  g_recover_continuity_ocxo1_correction_ns =
      campaign_recover_signed_delta_u64(g_recover_continuity_ocxo1_after_ns,
                                        before_o1);
  g_recover_continuity_ocxo2_correction_ns =
      campaign_recover_signed_delta_u64(g_recover_continuity_ocxo2_after_ns,
                                        before_o2);
  recover_continuity_set_reason("aligned_public_ocxo_ratio");
}

static uint64_t campaign_public_counterledger_ns(time_clock_id_t clock,
                                                 int64_t offset) {
  const uint64_t raw = current_raw_counterledger_ns(clock);
  return raw ? campaign_public_from_offset(raw, offset) : 0ULL;
}

static uint64_t campaign_public_counterledger_ocxo1_ns(void) {
  return campaign_public_counterledger_ns(
      time_clock_id_t::OCXO1,
      g_campaign_public_counterledger_ocxo1_offset);
}

static uint64_t campaign_public_counterledger_ocxo2_ns(void) {
  return campaign_public_counterledger_ns(
      time_clock_id_t::OCXO2,
      g_campaign_public_counterledger_ocxo2_offset);
}

static void campaign_public_counterledger_offsets_reset_to_current(void) {
  const uint64_t o1 = current_raw_counterledger_ns(time_clock_id_t::OCXO1);
  const uint64_t o2 = current_raw_counterledger_ns(time_clock_id_t::OCXO2);
  g_campaign_public_counterledger_ocxo1_offset = o1
      ? campaign_public_offset_to_zero(o1)
      : 0;
  g_campaign_public_counterledger_ocxo2_offset = o2
      ? campaign_public_offset_to_zero(o2)
      : 0;
}

static void campaign_public_counterledger_offsets_reset_for_recover(void) {
  const uint64_t o1 = current_raw_counterledger_ns(time_clock_id_t::OCXO1);
  const uint64_t o2 = current_raw_counterledger_ns(time_clock_id_t::OCXO2);
  g_campaign_public_counterledger_ocxo1_offset = o1
      ? campaign_public_offset_for_recovered_value(o1, recover_ocxo1_ns)
      : 0;
  g_campaign_public_counterledger_ocxo2_offset = o2
      ? campaign_public_offset_for_recovered_value(o2, recover_ocxo2_ns)
      : 0;
}

static uint64_t campaign_public_ocxo1_measured_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo1_measured_ns(),
                                     g_campaign_public_ocxo1_measured_offset);
}

static uint64_t campaign_public_ocxo2_measured_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo2_measured_ns(),
                                     g_campaign_public_ocxo2_measured_offset);
}

static void campaign_public_offsets_reset_to_current(void) {
  recover_continuity_align_reset("offsets_reset_to_current");
  g_campaign_public_dwt_offset =
      campaign_public_offset_to_zero(g_dwt_cycle_count_total);
  g_campaign_public_gnss_offset =
      campaign_public_offset_to_zero(current_raw_gnss_ns());
  g_campaign_public_ocxo1_offset =
      campaign_public_offset_to_zero(current_raw_ocxo1_ns());
  g_campaign_public_ocxo2_offset =
      campaign_public_offset_to_zero(current_raw_ocxo2_ns());
  g_campaign_public_ocxo1_measured_offset =
      campaign_public_offset_to_zero(current_raw_ocxo1_measured_ns());
  g_campaign_public_ocxo2_measured_offset =
      campaign_public_offset_to_zero(current_raw_ocxo2_measured_ns());
  campaign_public_counterledger_offsets_reset_to_current();
}

static void campaign_public_offsets_reset_for_recover(void) {
  recover_continuity_align_reset("recover_offsets_seeded");
  g_campaign_public_dwt_offset =
      campaign_public_offset_for_recovered_value(g_dwt_cycle_count_total,
                                                 dwt_ns_to_cycles(recover_dwt_ns));
  g_campaign_public_gnss_offset =
      campaign_public_offset_for_recovered_value(current_raw_gnss_ns(),
                                                 recover_gnss_ns);
  g_campaign_public_ocxo1_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo1_ns(),
                                                 recover_ocxo1_ns);
  g_campaign_public_ocxo2_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo2_ns(),
                                                 recover_ocxo2_ns);
  g_campaign_public_ocxo1_measured_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo1_measured_ns(),
                                                 recover_ocxo1_ns);
  g_campaign_public_ocxo2_measured_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo2_measured_ns(),
                                                 recover_ocxo2_ns);
  campaign_public_counterledger_offsets_reset_for_recover();
}

static void campaign_start_phaseledger_set_reason(const char* reason,
                                                   const char* first_problem = nullptr) {
  safeCopy(g_start_phaseledger_last_reason,
           sizeof(g_start_phaseledger_last_reason),
           reason ? reason : "phaseledger_start");
  safeCopy(g_start_phaseledger_last_first_problem,
           sizeof(g_start_phaseledger_last_first_problem),
           first_problem ? first_problem : "");
}


static bool campaign_start_phaseledger_capture_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return s.last_capture_available &&
         s.last_capture_valid &&
         s.last_capture_lane_valid &&
         s.last_capture_all_lanes_valid &&
         s.last_capture_sequence_match &&
         s.last_capture_sequence == s.pps_sequence;
}


static bool campaign_start_phaseledger_integer_interval_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return s.interval_valid && s.interval_ns != 0ULL;
}


static bool campaign_start_phaseledger_phase_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return s.phase_valid &&
         !s.phase_pending &&
         s.phase_pps_sequence != 0U &&
         s.phase_pps_sequence == s.pps_sequence;
}


static bool campaign_start_phaseledger_lag_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return campaign_start_phaseledger_phase_ready(s) &&
         s.phase_lag_pps <= CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS;
}


static bool campaign_start_phaseledger_refined_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  // Exact phase identity is proven separately; Alpha only marks refined_valid
  // after deriving refined_ns from that currently resolved phase record.
  return s.refined_valid && s.refined_ns != 0ULL;
}


static bool campaign_start_phaseledger_refined_interval_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return s.refined_interval_valid && s.refined_interval_ns != 0ULL;
}


static bool campaign_start_phaseledger_lane_ready(
    bool snapshot_ok,
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return snapshot_ok &&
         s.valid &&
         s.initialized &&
         campaign_start_phaseledger_capture_ready(s) &&
         campaign_start_phaseledger_integer_interval_ready(s) &&
         campaign_start_phaseledger_phase_ready(s) &&
         campaign_start_phaseledger_lag_ready(s) &&
         campaign_start_phaseledger_refined_ready(s) &&
         campaign_start_phaseledger_refined_interval_ready(s);
}


static const char* campaign_start_phaseledger_lane_problem(
    bool snapshot_ok,
    const clocks_alpha_ocxo_counterledger_snapshot_t& s,
    const char* lane) {
  static char reason[32];

  if (!snapshot_ok || !s.valid || !s.initialized) {
    snprintf(reason, sizeof(reason), "%s_snapshot", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_capture_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_capture", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_integer_interval_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_integer_interval", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_phase_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_phase", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_lag_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_phase_lag", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_refined_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_refined", lane);
    return reason;
  }
  if (!campaign_start_phaseledger_refined_interval_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_refined_interval", lane);
    return reason;
  }

  return nullptr;
}


static bool campaign_start_counterledger_maturity_ready(void) {

  g_start_phaseledger_last_ocxo1 = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_start_phaseledger_last_ocxo2 = clocks_alpha_ocxo_counterledger_snapshot_t{};
  const bool ocxo1_snapshot_ok = clocks_alpha_ocxo_counterledger_snapshot(
      time_clock_id_t::OCXO1, &g_start_phaseledger_last_ocxo1);
  const bool ocxo2_snapshot_ok = clocks_alpha_ocxo_counterledger_snapshot(
      time_clock_id_t::OCXO2, &g_start_phaseledger_last_ocxo2);

  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo1 =
      g_start_phaseledger_last_ocxo1;
  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo2 =
      g_start_phaseledger_last_ocxo2;

  const bool ocxo1_ready =
      campaign_start_phaseledger_lane_ready(ocxo1_snapshot_ok, ocxo1);
  const bool ocxo2_ready =
      campaign_start_phaseledger_lane_ready(ocxo2_snapshot_ok, ocxo2);
  const bool sequence_aligned =
      ocxo1_ready && ocxo2_ready &&
      ocxo1.pps_sequence == ocxo2.pps_sequence &&
      ocxo1.phase_pps_sequence == ocxo2.phase_pps_sequence;

  g_start_phaseledger_last_ocxo1_ready = ocxo1_ready;
  g_start_phaseledger_last_ocxo2_ready = ocxo2_ready;
  g_start_phaseledger_last_sequence_aligned = sequence_aligned;

  const bool ready = ocxo1_ready && ocxo2_ready && sequence_aligned;
  g_start_phaseledger_last_ready = ready;

  if (ready) {
    campaign_start_phaseledger_set_reason("phaseledger_mature");
    return true;
  }


  const char* problem =
      campaign_start_phaseledger_lane_problem(ocxo1_snapshot_ok,
                                              ocxo1,
                                              "o1");
  if (!problem) {
    problem = campaign_start_phaseledger_lane_problem(ocxo2_snapshot_ok,
                                                      ocxo2,
                                                      "o2");
  }
  if (!problem && !sequence_aligned) {
    problem = "sequence_alignment";
  }
  campaign_start_phaseledger_set_reason("waiting_for_phaseledger_maturity",
                                        problem ? problem : "unknown");
  return false;
}


static bool campaign_start_handoff_ready(void) {

  clocks_alpha_ocxo_pps_projection_snapshot_t& ocxo1_projection =
      g_beta_start_ocxo1_projection;
  clocks_alpha_ocxo_pps_projection_snapshot_t& ocxo2_projection =
      g_beta_start_ocxo2_projection;
  ocxo1_projection = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  ocxo2_projection = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  const bool ocxo1_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO1,
                                                &ocxo1_projection);
  const bool ocxo2_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO2,
                                                &ocxo2_projection);

  const uint64_t raw_gnss_ns = current_raw_gnss_ns();
  const bool origin_ready = clocks_alpha_ocxo_public_origin_ready();
  const bool ocxo1_projection_ready =
      ocxo1_projection_ok && ocxo1_projection.valid &&
      ocxo1_projection.projected_ocxo_ns_at_pps != 0ULL &&
      ocxo1_projection.pps_vclock_ns == raw_gnss_ns;
  const bool ocxo2_projection_ready =
      ocxo2_projection_ok && ocxo2_projection.valid &&
      ocxo2_projection.projected_ocxo_ns_at_pps != 0ULL &&
      ocxo2_projection.pps_vclock_ns == raw_gnss_ns;

  g_start_handoff_last_origin_ready = origin_ready;
  g_start_handoff_last_ocxo1_projection_ready = ocxo1_projection_ready;
  g_start_handoff_last_ocxo2_projection_ready = ocxo2_projection_ready;
  g_start_handoff_last_raw_gnss_ns = raw_gnss_ns;
  g_start_handoff_last_raw_ocxo1_ns = current_raw_ocxo1_ns();
  g_start_handoff_last_raw_ocxo2_ns = current_raw_ocxo2_ns();
  g_start_handoff_last_ocxo1_projected_ns =
      (ocxo1_projection_ok && ocxo1_projection.valid)
          ? ocxo1_projection.projected_ocxo_ns_at_pps
          : 0ULL;
  g_start_handoff_last_ocxo2_projected_ns =
      (ocxo2_projection_ok && ocxo2_projection.valid)
          ? ocxo2_projection.projected_ocxo_ns_at_pps
          : 0ULL;
  g_start_handoff_last_ocxo1_projection_vclock_ns =
      ocxo1_projection_ok ? ocxo1_projection.pps_vclock_ns : 0ULL;
  g_start_handoff_last_ocxo2_projection_vclock_ns =
      ocxo2_projection_ok ? ocxo2_projection.pps_vclock_ns : 0ULL;

  if (clocks_ocxo_counterledger_mode_enabled()) {
    const bool counterledger_ready =
        campaign_start_counterledger_maturity_ready();
    g_start_handoff_last_ready = origin_ready && counterledger_ready;

    // Preserve the historical report field names, but in
    // PPS_COUNTERLEDGER mode these booleans now mean lane-level
    // PhaseLedger launch maturity, not PPS-projection readiness.
    g_start_handoff_last_ocxo1_projection_ready =
        g_start_phaseledger_last_ocxo1_ready;
    g_start_handoff_last_ocxo2_projection_ready =
        g_start_phaseledger_last_ocxo2_ready;

    if (!origin_ready) {
    }
    if (!counterledger_ready) {
      // Historical counter name retained for report compatibility: in
      // CounterLedger/PhaseLedger mode this means the PPS-sampled hardware
      // ledger, capture custody, or refined PhaseLedger interval is not mature.
    }
    return g_start_handoff_last_ready;
  }

  g_start_handoff_last_ready =
      origin_ready && ocxo1_projection_ready && ocxo2_projection_ready;

  if (!origin_ready) {
  }
  if (!ocxo1_projection_ready || !ocxo2_projection_ready) {
  }

  return g_start_handoff_last_ready;
}



static void campaign_start_prologue_reset(const char* reason) {
  g_start_prologue_seeded = false;
  g_start_prologue_reference_ready = false;
  g_start_prologue_private_candidate_count = 0;
  g_start_prologue_last_private_count = 0;
  g_start_prologue_last_release_public_count = 0;
  g_start_prologue_release_count = 0;
  g_start_prologue_private_limit_count = 0;
  g_start_handoff_launch_wait_count = 0;
  g_start_prologue_pps0_interval_valid = false;
  g_start_prologue_pps0_pps_obs = 0;
  g_start_prologue_pps0_v_obs = 0;
  g_start_prologue_pps0_o1_obs = 0;
  g_start_prologue_pps0_o2_obs = 0;

  g_start_candidate_last_ready = false;
  g_start_candidate_last_vclock_snapshot_valid = false;
  g_start_candidate_last_ocxo1_snapshot_valid = false;
  g_start_candidate_last_ocxo2_snapshot_valid = false;
  g_start_candidate_last_vclock_observed_interval = 0;
  g_start_candidate_last_ocxo1_observed_interval = 0;
  g_start_candidate_last_ocxo2_observed_interval = 0;
  g_start_candidate_last_vclock_measurement_interval = 0;
  g_start_candidate_last_ocxo1_measurement_interval = 0;
  g_start_candidate_last_ocxo2_measurement_interval = 0;
  g_start_candidate_last_vclock_update_count = 0;
  g_start_candidate_last_ocxo1_update_count = 0;
  g_start_candidate_last_ocxo2_update_count = 0;
  g_start_candidate_last_selected_reference_valid = false;
  g_start_candidate_last_selected_reference_interval = 0;
  g_start_candidate_last_effective_cps = 0;
  g_start_candidate_last_missing_mask = 0;
  safeCopy(g_start_candidate_last_first_problem,
           sizeof(g_start_candidate_last_first_problem),
           "not_checked");

  g_start_phaseledger_last_ready = false;
  g_start_phaseledger_last_ocxo1_ready = false;
  g_start_phaseledger_last_ocxo2_ready = false;
  g_start_phaseledger_last_sequence_aligned = false;
  g_start_phaseledger_last_ocxo1 =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_start_phaseledger_last_ocxo2 =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  campaign_start_phaseledger_set_reason(reason ? reason : "reset");
  safeCopy(g_start_prologue_last_reason,
           sizeof(g_start_prologue_last_reason),
           reason ? reason : "reset");
}

static bool campaign_start_prologue_should_hold(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f);
static void flash_cut_clear_pending(void);
static void ocxo_dac_pacing_abort_all(void);
static FLASHMEM void recover_reattach_begin(void);
static FLASHMEM void recover_reattach_reset(const char* reason);

static void campaign_warmup_begin(campaign_warmup_mode_t mode) {
  g_campaign_warmup_suppressed_total = 0;

  if (mode == campaign_warmup_mode_t::RECOVER) {
    // RECOVER no longer buries fixed rows, but it must prove OCXO
    // reattachment before public campaign record resumes.  Install recovered offsets
    // now; the reattach gate advances hidden candidate identity while waiting
    // for fresh OCXO evidence.
    interrupt_dwt_publication_launch_acquisition_end();
    g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
    g_campaign_warmup_remaining = 0;
    campaign_start_prologue_reset("recover_no_prologue");
    // RECOVER offsets are installed in the RECOVER gate before Alpha re-primes
    // OCXO/CounterLedger custody.  Do not recompute them here after the raw
    // CounterLedger lanes have intentionally been converted back into seed
    // state.
    recover_reattach_begin();
    return;
  }

  if (mode == campaign_warmup_mode_t::START) {
    recover_reattach_reset("not_recovering");

    // START owns a private acquisition bookend, not a public skipped row.
    // Interrupt and Alpha continue to run while campaign_seconds remains zero.
    // The launch-acquisition window relaxes only the non-poisonous publication
    // courts needed to let OCXO origin, CounterLedger, and PhaseLedger mature.
    interrupt_dwt_publication_launch_acquisition_begin();
    g_campaign_warmup_mode = campaign_warmup_mode_t::START;
    g_campaign_warmup_remaining = 1;
    campaign_start_prologue_reset("waiting_for_private_pps0");
    return;
  }

  interrupt_dwt_publication_launch_acquisition_end();
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  campaign_start_prologue_reset("none");
  recover_reattach_reset("none_lifecycle");
  campaign_public_offsets_reset_to_current();
}

static bool campaign_warmup_active(void) {
  // Treat the entire RECOVER transition as not-yet-campaign-continuous for
  // watchdog arming.  The first degraded/quarantined rows exist only to prove
  // liveness and reattach custody to the Pi; a DWT publication court verdict
  // during that window must not fire a second campaign-surrender watchdog and
  // stop the only rows the Pi can use to observe recovery progress.
  return g_campaign_warmup_mode != campaign_warmup_mode_t::NONE ||
         g_recover_reattach_active ||
         g_recover_reattach_degraded_active ||
         g_science_residual_quarantine_remaining != 0U ||
         clocks_campaign_recovery_lifecycle_active();
}

static void recover_reattach_set_reason(const char* reason) {
  safeCopy(g_recover_reattach_last_reason,
           sizeof(g_recover_reattach_last_reason),
           reason ? reason : "recover_reattach");
}

static void recover_reattach_set_stall_reason(const char* reason) {
  safeCopy(g_recover_reattach_stall_reason,
           sizeof(g_recover_reattach_stall_reason),
           reason ? reason : "recover_reattach_stall");
}

static constexpr uint32_t RECOVER_READY_SNAPSHOT       = 1U << 0;
static constexpr uint32_t RECOVER_READY_VALID          = 1U << 1;
static constexpr uint32_t RECOVER_READY_CAPTURE        = 1U << 2;
static constexpr uint32_t RECOVER_READY_INTERVAL       = 1U << 3;
static constexpr uint32_t RECOVER_READY_CLOCKFACE      = 1U << 4;
static constexpr uint32_t RECOVER_READY_PHASE          = 1U << 5;
static constexpr uint32_t RECOVER_READY_PHASE_LAG      = 1U << 6;
static constexpr uint32_t RECOVER_READY_REFINED        = 1U << 7;
static constexpr uint32_t RECOVER_READY_REFINED_INT    = 1U << 8;
static constexpr uint32_t RECOVER_READY_SCIENCE        = 1U << 9;

static uint32_t recover_reattach_readiness_mask(
    const clocks_alpha_recover_reattach_snapshot_t& s) {
  uint32_t mask = 0U;
  if (s.counterledger_mode) {
    if (s.counterledger_snapshot_ok) mask |= RECOVER_READY_SNAPSHOT;
    if (s.counterledger_valid) mask |= RECOVER_READY_VALID;
    if (s.counterledger_capture_ready) mask |= RECOVER_READY_CAPTURE;
    if (s.counterledger_interval_valid) mask |= RECOVER_READY_INTERVAL;
    if (s.clockface_ready) mask |= RECOVER_READY_CLOCKFACE;
    if (s.counterledger_phase_valid) mask |= RECOVER_READY_PHASE;
    if (s.counterledger_phase_lag_ok) mask |= RECOVER_READY_PHASE_LAG;
    if (s.counterledger_refined_valid) mask |= RECOVER_READY_REFINED;
    if (s.counterledger_refined_interval_valid) {
      mask |= RECOVER_READY_REFINED_INT;
    }
    if (s.science_ready) mask |= RECOVER_READY_SCIENCE;
    return mask;
  }

  if (s.forensics_ready) mask |= RECOVER_READY_SNAPSHOT;
  if (s.edge_history_ready) mask |= RECOVER_READY_VALID;
  if (s.projection_ready) mask |= RECOVER_READY_INTERVAL;
  if (s.public_ns_nonzero) mask |= RECOVER_READY_CAPTURE;
  if (s.clockface_ready) mask |= RECOVER_READY_CLOCKFACE;
  if (s.science_ready) mask |= RECOVER_READY_SCIENCE;
  return mask;
}

static uint32_t recover_reattach_readiness_score(
    const clocks_alpha_recover_reattach_snapshot_t& s) {
  uint32_t mask = recover_reattach_readiness_mask(s);
  uint32_t score = 0U;
  while (mask != 0U) {
    score += mask & 1U;
    mask >>= 1U;
  }
  return score;
}

static bool recover_reattach_note_lane_progress(
    recover_reattach_progress_marker_t& marker,
    const clocks_alpha_recover_reattach_snapshot_t& s) {
  const uint32_t current = recover_reattach_readiness_mask(s);
  const uint32_t newly_proven = current & ~marker.seen_readiness_mask;
  marker.seen_readiness_mask |= current;
  return newly_proven != 0U;
}

static bool recover_reattach_note_progress(uint32_t public_count) {
  const bool first = !g_recover_reattach_progress_marker_valid;
  const bool ocxo1_progress = recover_reattach_note_lane_progress(
      g_recover_reattach_progress_ocxo1,
      g_recover_reattach_last_ocxo1);
  const bool ocxo2_progress = recover_reattach_note_lane_progress(
      g_recover_reattach_progress_ocxo2,
      g_recover_reattach_last_ocxo2);

  g_recover_reattach_progress_marker_valid = true;
  const bool progressed = first || ocxo1_progress || ocxo2_progress;
  if (progressed) {
    g_recover_reattach_progress_count++;
    g_recover_reattach_last_progress_public_count = public_count;
  }
  return progressed;
}

static void recover_reattach_add_stall_lane(
    Payload& parent,
    const char* key,
    const clocks_alpha_recover_reattach_snapshot_t& s) {
  Payload lane;
  lane.add("clockface_ready", s.clockface_ready);
  lane.add("science_ready", s.science_ready);
  lane.add("readiness_score", recover_reattach_readiness_score(s));
  lane.add("readiness_mask", recover_reattach_readiness_mask(s));
  lane.add("counterledger_pps_sequence", s.counterledger_pps_sequence);
  lane.add("counterledger_phase_pps_sequence",
           s.counterledger_phase_pps_sequence);
  lane.add("counterledger_phase_lag_pps", s.counterledger_phase_lag_pps);
  lane.add("counterledger_recover_capture_ready_count",
           s.counterledger_recover_capture_ready_count);
  lane.add("counterledger_recover_sample_interval_accept_count",
           s.counterledger_recover_sample_interval_accept_count);
  lane.add("counterledger_recover_phase_resolve_success_count",
           s.counterledger_recover_phase_resolve_success_count);
  lane.add("counterledger_recover_refined_interval_accept_count",
           s.counterledger_recover_refined_interval_accept_count);
  lane.add("counterledger_last_phase_resolve_reason_id",
           s.counterledger_last_phase_resolve_reason_id);
  lane.add("counterledger_last_phase_resolve_reason",
           clocks_phaseledger_resolve_reason_name(
               s.counterledger_last_phase_resolve_reason_id));
  parent.add_object(key, lane);
}

static void recover_reattach_publish_stalled_event(void) {
  Payload p;
  p.add("schema", "CLOCKS_RECOVERY_STALLED_V1");
  p.add("reason", "ocxo_science_reattach_no_progress");
  p.add("source", "TEENSY_CLOCKS_RECOVERY_LIVENESS");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("recovery_generation", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("clockface_ready", (bool)g_recover_reattach_clockface_ready);
  p.add("science_ready", (bool)g_recover_reattach_science_ready);
  p.add("degraded_publication_active",
        (bool)g_recover_reattach_degraded_active);
  p.add("no_progress_rows", g_recover_reattach_no_progress_row_count);
  p.add("stall_threshold_rows",
        (uint32_t)CLOCKS_RECOVER_REATTACH_DEGRADED_STALL_CANDIDATES);
  p.add("last_progress_public_count",
        g_recover_reattach_last_progress_public_count);
  p.add("stall_count", g_recover_reattach_stall_count);
  p.add("progress_count", g_recover_reattach_progress_count);
  p.add("progress_resume_count",
        g_recover_reattach_progress_resume_count);

  Payload lanes;
  recover_reattach_add_stall_lane(
      lanes, "ocxo1", g_recover_reattach_last_ocxo1);
  recover_reattach_add_stall_lane(
      lanes, "ocxo2", g_recover_reattach_last_ocxo2);
  p.add_object("lanes", lanes);

  publish("CLOCKS_RECOVERY_STALLED", p);
  g_recover_reattach_stall_publish_count++;
}

static FLASHMEM bool recover_reattach_refresh_ready(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_REFRESH_READY);

  // Write directly into the cached recovery flight-recorder snapshots. This
  // keeps the recovery gate off the large-local-object path; REPORT_RECOVERY
  // later publishes these cached facts without re-entering Alpha or process_time.
  const bool ocxo1_snapshot_ok =
      clocks_alpha_ocxo_recover_reattach_snapshot(
          time_clock_id_t::OCXO1, &g_recover_reattach_last_ocxo1);
  const bool ocxo2_snapshot_ok =
      clocks_alpha_ocxo_recover_reattach_snapshot(
          time_clock_id_t::OCXO2, &g_recover_reattach_last_ocxo2);

  g_recover_reattach_clockface_ready =
      ocxo1_snapshot_ok && ocxo2_snapshot_ok &&
      g_recover_reattach_last_ocxo1.clockface_ready &&
      g_recover_reattach_last_ocxo2.clockface_ready;
  g_recover_reattach_science_ready =
      ocxo1_snapshot_ok && ocxo2_snapshot_ok &&
      g_recover_reattach_last_ocxo1.science_ready &&
      g_recover_reattach_last_ocxo2.science_ready;

  return g_recover_reattach_science_ready;
}
static FLASHMEM void recover_reattach_reset(const char* reason) {
  recover_continuity_align_reset(reason ? reason : "reattach_reset");
  g_recover_reattach_active = false;
  g_recover_reattach_degraded_active = false;
  g_recover_reattach_clockface_ready = false;
  g_recover_reattach_science_ready = false;
  g_recover_reattach_stalled = false;
  g_recover_reattach_no_progress_row_count = 0;
  g_recover_reattach_degraded_window_row_count = 0;
  g_recover_reattach_last_progress_public_count = 0;
  g_recover_reattach_progress_marker_valid = false;
  g_recover_reattach_progress_ocxo1 = recover_reattach_progress_marker_t{};
  g_recover_reattach_progress_ocxo2 = recover_reattach_progress_marker_t{};
  recover_reattach_set_stall_reason(reason ? reason : "reset");
  g_recover_reattach_hidden_candidate_count = 0;
  g_recover_reattach_last_hidden_public_count = 0;
  g_recover_reattach_last_degraded_public_count = 0;
  g_recover_reattach_last_ocxo1 = clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_reattach_last_ocxo2 = clocks_alpha_recover_reattach_snapshot_t{};
  recover_reattach_set_reason(reason ? reason : "reset");
}

static FLASHMEM void recover_reattach_begin(void) {
  recover_continuity_align_reset("recover_clockface_alignment_reset");
  recover_continuity_align_arm();
  g_recover_reattach_active = true;
  g_recover_reattach_degraded_active = false;
  g_recover_reattach_clockface_ready = false;
  g_recover_reattach_science_ready = false;
  g_recover_reattach_stalled = false;
  g_recover_reattach_no_progress_row_count = 0;
  g_recover_reattach_degraded_window_row_count = 0;
  g_recover_reattach_last_progress_public_count = 0;
  g_recover_reattach_progress_marker_valid = false;
  g_recover_reattach_progress_ocxo1 = recover_reattach_progress_marker_t{};
  g_recover_reattach_progress_ocxo2 = recover_reattach_progress_marker_t{};
  recover_reattach_set_stall_reason("not_stalled");
  g_recover_reattach_hidden_candidate_count = 0;
  g_recover_reattach_last_hidden_public_count = 0;
  g_recover_reattach_last_degraded_public_count = 0;
  g_recover_reattach_begin_count++;
  g_recover_reattach_last_ocxo1 = clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_reattach_last_ocxo2 = clocks_alpha_recover_reattach_snapshot_t{};
  recover_reattach_set_reason("waiting_for_ocxo_reattach");
}

static FLASHMEM void recover_reattach_release(const char* reason, bool degraded) {
  g_recover_reattach_active = false;
  g_recover_reattach_release_count++;
  g_recover_reattach_last_release_public_count =
      (uint32_t)(campaign_seconds + 1ULL);

  if (degraded) {
    g_recover_reattach_degraded_active = true;
    g_recover_reattach_stalled = false;
    g_recover_reattach_degraded_window_row_count = 0;
    recover_reattach_set_stall_reason("not_stalled");
    g_recover_reattach_degraded_release_count++;
    g_recover_reattach_last_degraded_release_public_count =
        g_recover_reattach_last_release_public_count;
    g_recover_reattach_last_degraded_public_count =
        g_recover_reattach_last_release_public_count;
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
  } else {
    // Campaign clockface continuity was armed when RECOVER reattachment began
    // and therefore precedes every public row, including degraded testimony.
    // Science readiness must not author a second presentation intercept.
    if (g_recover_reattach_degraded_active) {
      g_recover_reattach_degraded_active = false;
      g_recover_reattach_degraded_clear_count++;
    }
  }

  recover_reattach_set_reason(reason ? reason : "ocxo_reattach_release");
}

static bool campaign_warmup_consume_one_candidate_record(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {
  if (g_campaign_warmup_mode != campaign_warmup_mode_t::START) {
    return false;
  }

  if (campaign_start_prologue_should_hold(
          vclock_valid, vclock_f,
          ocxo1_valid, ocxo1_f,
          ocxo2_valid, ocxo2_f)) {
    // This is private pre-publication evidence.  Alpha/Interrupt have advanced,
    // but campaign_seconds has not and no public campaign record identity exists yet.
    if (g_campaign_warmup_mode == campaign_warmup_mode_t::START) {
      g_campaign_warmup_remaining = 1;
    }
    g_campaign_warmup_suppressed_total++;
    return true;
  }

  // The current PPS candidate has passed the prologue court.  End the relaxed
  // Interrupt window before publishing it as the first strict public identity.
  interrupt_dwt_publication_launch_acquisition_end();
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  g_start_prologue_release_count++;
  g_start_prologue_last_release_public_count = 1U;
  safeCopy(g_start_prologue_last_reason,
           sizeof(g_start_prologue_last_reason),
           clocks_ocxo_counterledger_mode_enabled()
               ? "phaseledger_release_public_pps1"
               : "release_public_pps1");
  return false;
}


static void campaign_warmup_reset(void) {
  interrupt_dwt_publication_launch_acquisition_end();
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  g_campaign_warmup_suppressed_total = 0;
  campaign_start_prologue_reset("reset");
  recover_reattach_reset("warmup_reset");
  campaign_public_offsets_reset_to_current();
}

static void clocks_watchdog_clear_surrender_for_new_lifecycle(void);
static void clocks_watchdog_disarm_campaign_publication(void);

static void recover_lifecycle_set_reason(const char* reason) {
  safeCopy(g_recover_lifecycle_reason,
           sizeof(g_recover_lifecycle_reason),
           reason ? reason : "recover_lifecycle");
}

static void recover_lifecycle_set_abort_reason(const char* reason) {
  safeCopy(g_recover_lifecycle_abort_reason,
           sizeof(g_recover_lifecycle_abort_reason),
           reason ? reason : "recover_abort");
}

static const char* recover_lifecycle_mode_name(
    recover_lifecycle_mode_t mode) {
  switch (mode) {
    case recover_lifecycle_mode_t::LIVE_REATTACH:  return "LIVE_REATTACH";
    case recover_lifecycle_mode_t::COLD_BOOTSTRAP: return "COLD_BOOTSTRAP";
    default:                                       return "NONE";
  }
}

static bool recover_lifecycle_prepare_cold_bootstrap(void) {
  g_recover_lifecycle_mode = recover_lifecycle_mode_t::COLD_BOOTSTRAP;
  g_recover_lifecycle_cold_bootstrap_epoch_ready = false;
  g_recover_lifecycle_cold_bootstrap_begin_count++;

  if (clocks_alpha_installed_smartzero_backing_epoch()) {
    g_recover_lifecycle_cold_bootstrap_epoch_ready = true;
    g_recover_lifecycle_cold_bootstrap_ready_count++;
    return true;
  }

  // process_clocks_init() normally has startup SmartZero already running.
  // Preserve that acquisition (or its completed proof) across RECOVER.  Only
  // start a replacement acquisition when neither proof nor install transaction
  // is currently alive.
  if (interrupt_smartzero_running() ||
      interrupt_smartzero_complete() ||
      clocks_alpha_epoch_install_in_progress()) {
    return true;
  }

  if (!clocks_alpha_begin_smartzero_epoch("recover_cold_bootstrap")) {
    g_recover_lifecycle_cold_bootstrap_start_failure_count++;
    return false;
  }

  return true;
}

static bool recover_lifecycle_enter_from_command(const char* reason) {
  g_recover_lifecycle_begin_count++;
  g_recover_lifecycle_last_begin_campaign_seconds = (uint32_t)campaign_seconds;
  recover_lifecycle_set_reason(reason ? reason : "recover_command_armed");
  recover_lifecycle_set_abort_reason("none");

  // A watchdog may have stopped the campaign and left process_interrupt's
  // publication court in a surrendered state. Clear that custody immediately
  // at command acceptance so the next PPS/VCLOCK can reach Beta and consume
  // request_recover. The PPS gate repeats the reset as an idempotent boundary
  // proof before publication resumes.
  interrupt_recover_reset_publication_custody();
  g_recover_lifecycle_command_custody_reset_count++;

  // A flashed/rebooted Teensy has durable campaign state on the Pi but no
  // installed local service epoch.  That is not a failed live rearm.  Preserve
  // or start startup SmartZero, enter RECOVERING, and let sovereign PPS drive
  // the epoch install before the ordinary RECOVER grid-rephase transaction.
  if (!clocks_alpha_installed_smartzero_backing_epoch()) {
    if (!recover_lifecycle_prepare_cold_bootstrap()) {
      recover_lifecycle_set_reason("recover_cold_bootstrap_start_failed");
      return false;
    }

    g_recover_lifecycle_last_interrupt_service_rearm_ok = false;
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    clocks_watchdog_disarm_campaign_publication();
    campaign_state = clocks_campaign_state_t::RECOVERING;
    recover_lifecycle_set_reason("recover_cold_bootstrap_wait_smartzero");
    return true;
  }

  g_recover_lifecycle_mode = recover_lifecycle_mode_t::LIVE_REATTACH;
  g_recover_lifecycle_cold_bootstrap_epoch_ready = true;

  // Live recovery preserves the installed service epoch.  Verify sovereign
  // VCLOCK service, then launch the shared TimePop-staged OCXO physical-grid
  // rephase before the reattachment timeout starts.
  g_recover_lifecycle_interrupt_service_rearm_count++;
  g_recover_lifecycle_last_interrupt_service_rearm_ok =
      clocks_alpha_recover_rearm_interrupt_service();
  if (!g_recover_lifecycle_last_interrupt_service_rearm_ok) {
    g_recover_lifecycle_interrupt_service_rearm_failure_count++;
    recover_lifecycle_set_reason("recover_interrupt_service_rearm_failed");
    return false;
  }

  clocks_watchdog_clear_surrender_for_new_lifecycle();
  clocks_watchdog_disarm_campaign_publication();
  campaign_state = clocks_campaign_state_t::RECOVERING;
  return true;
}

static void recover_lifecycle_complete_at_pps(void) {
  g_recover_lifecycle_complete_count++;
  recover_lifecycle_set_reason("recover_pps_gate_consumed");
  campaign_state = clocks_campaign_state_t::STARTED;
}

static void recover_lifecycle_abort(const char* reason) {
  g_recover_lifecycle_abort_count++;
  g_recover_lifecycle_last_abort_campaign_seconds = (uint32_t)campaign_seconds;
  recover_lifecycle_set_abort_reason(reason ? reason : "recover_aborted");
  recover_lifecycle_set_reason("idle");

  if (g_recover_lifecycle_mode ==
          recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
      !clocks_alpha_installed_smartzero_backing_epoch()) {
    interrupt_smartzero_abort();
    clocks_alpha_smartzero_pending_clear();
  }

  clocks_alpha_ocxo_grid_rephase_acknowledge(
      clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
  request_recover = false;
  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  request_start = false;
  request_stop = false;
  request_zero = false;
  flash_cut_clear_pending();
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  campaign_state = clocks_campaign_state_t::STOPPED;
  campaign_warmup_reset();
}

static uint64_t campaign_public_dwt_total(void) {
  return campaign_public_from_offset(g_dwt_cycle_count_total,
                                     g_campaign_public_dwt_offset);
}

static uint64_t campaign_public_gnss_ns(uint32_t public_count) {
  // Campaign GNSS is an identity surface, not a recovered projection.
  // Rendering it directly from the public PPS identity prevents a warm
  // RECOVER splice from inheriting an ambient raw-ledger phase of +1 second.
  return (uint64_t)public_count * CLOCKS_BETA_NS_PER_SECOND;
}

static uint64_t campaign_public_ocxo1_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo1_ns(),
                                     g_campaign_public_ocxo1_offset);
}

static uint64_t campaign_public_ocxo2_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo2_ns(),
                                     g_campaign_public_ocxo2_offset);
}

static FLASHMEM bool recover_reattach_should_hold(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_SHOULD_HOLD);
  if (!g_recover_reattach_active) return false;

  const bool science_ready = recover_reattach_refresh_ready();
  (void)recover_reattach_note_progress((uint32_t)campaign_seconds);

  if (science_ready) {
    recover_reattach_release("ocxo_science_reattach_ready", false);
    return false;
  }

  if (g_recover_reattach_clockface_ready) {
    // Timeline and both OCXO integer clockfaces are now fresh.  Publish the row
    // immediately, but keep refined science/Welford/servo gated until the
    // PhaseLedger interval proves itself.
    recover_reattach_release(
        "ocxo_clockface_ready_science_initializing", true);
    return false;
  }

  // Legacy "hidden candidate" counters now count emitted recovery candidates.
  // The identity advances through the normal per-second path below; this gate
  // only observes how long reattachment remains incomplete.
  g_recover_reattach_hold_count++;
  g_recover_reattach_hidden_candidate_count++;
  g_recover_reattach_last_hidden_public_count =
      (uint32_t)(campaign_seconds + 1ULL);

  if (g_recover_reattach_hidden_candidate_count >=
      CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES) {
    g_recover_reattach_timeout_count++;
    if (CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED) {
      recover_reattach_release("ocxo_reattach_timeout_degraded_release", true);
      return false;
    }
    recover_reattach_set_reason("ocxo_reattach_timeout_candidates_emitted");
  } else if (!g_recover_reattach_last_ocxo1.clockface_ready &&
             !g_recover_reattach_last_ocxo2.clockface_ready) {
    recover_reattach_set_reason("emitting_without_both_ocxo_clockfaces");
  } else if (!g_recover_reattach_last_ocxo1.clockface_ready) {
    recover_reattach_set_reason("emitting_without_ocxo1_clockface");
  } else if (!g_recover_reattach_last_ocxo2.clockface_ready) {
    recover_reattach_set_reason("emitting_without_ocxo2_clockface");
  } else {
    recover_reattach_set_reason("emitting_while_ocxo_science_initializes");
  }

  // Reattachment state remains active, but it no longer gates campaign record.
  return false;
}

static FLASHMEM bool recover_reattach_degraded_science_hold_active(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_DEGRADED_HOLD);
  if (!g_recover_reattach_degraded_active) return false;

  const bool science_ready = recover_reattach_refresh_ready();
  const bool progressed =
      recover_reattach_note_progress((uint32_t)campaign_seconds);

  if (science_ready) {
    const bool was_stalled = g_recover_reattach_stalled;
    g_recover_reattach_degraded_active = false;
    g_recover_reattach_stalled = false;
    if (was_stalled) g_recover_reattach_progress_resume_count++;
    g_recover_reattach_no_progress_row_count = 0;
    g_recover_reattach_degraded_window_row_count = 0;
    recover_reattach_set_stall_reason("cleared_by_science_ready");
    g_recover_reattach_degraded_clear_count++;

    // Campaign presentation was aligned before the first recovered public
    // row.  This transition releases science custody only; it must not create a
    // second OCXO clockface intercept.

    // The row that proves reattachment is still a boundary row: it may carry
    // PhaseLedger/CounterLedger state formed across the degraded window.
    // Start a real science quarantine here so Welford/PPB/servo do not consume
    // the first reattachment transient.
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
    recover_reattach_set_reason("ocxo_science_ready_after_degraded_release");
    return false;
  }

  g_recover_reattach_degraded_public_row_count++;
  g_recover_reattach_degraded_window_row_count++;
  g_recover_reattach_last_degraded_public_count =
      (uint32_t)campaign_seconds;

  if (progressed) {
    g_recover_reattach_no_progress_row_count = 0;
    if (g_recover_reattach_stalled) {
      g_recover_reattach_stalled = false;
      g_recover_reattach_progress_resume_count++;
      recover_reattach_set_stall_reason("progress_resumed");
    }
  } else {
    g_recover_reattach_no_progress_row_count++;
  }

  if (!g_recover_reattach_stalled &&
      g_recover_reattach_no_progress_row_count >=
          CLOCKS_RECOVER_REATTACH_DEGRADED_STALL_CANDIDATES) {
    g_recover_reattach_stalled = true;
    g_recover_reattach_stall_count++;
    g_recover_reattach_last_stall_public_count =
        (uint32_t)campaign_seconds;
    recover_reattach_set_stall_reason(
        "ocxo_science_reattach_no_progress");
    recover_reattach_publish_stalled_event();
  }

  recover_reattach_set_reason(
      g_recover_reattach_stalled
          ? "degraded_publication_science_stalled"
          : "degraded_publication_science_initializing");
  clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                 g_clocks_feature_science_residuals,
                                 system_feature_status_t::INITIALIZING,
                                 true);
  return true;
}

static FLASHMEM void ocxo_science_row_suppress_for_recover_hold(clock_science_row_t& row) {
  row.valid = false;
  row.antecedents_complete = false;

  row.delta_raw_valid = false;
  row.traditional_valid = false;
  row.traditional_total_valid = false;

  row.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
  row.clock_interval_ns = 0ULL;
  row.fast_residual_ns = 0LL;
  row.gnss_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
  row.clock_interval_ns_exact = 0.0;
  row.fast_residual_ns_exact = 0.0;
  row.tau_1s = 1.0;
  row.ppb_1s = 0.0;

  row.total_valid = false;
  row.total_tau = 1.0;
  row.total_ppb = 0.0;
  row.total_fast_residual_ns = 0LL;
  row.total_fast_residual_ns_exact = 0.0;
}

static FLASHMEM void recover_reattach_apply_degraded_science_hold(clock_science_row_t& row) {
  ocxo_science_row_suppress_for_recover_hold(row);
  g_recover_reattach_degraded_science_suppressed_count++;
}

static FLASHMEM bool science_residual_quarantine_apply(
    uint32_t public_count,
    clock_science_row_t& ocxo1_science,
    clock_science_row_t& ocxo2_science) {
  if (g_science_residual_quarantine_remaining == 0U) {
    return false;
  }

  g_science_residual_quarantine_remaining--;
  g_science_residual_quarantine_consumed_count++;
  g_science_residual_quarantine_last_public_count = public_count;

  // This quarantine is supposed to protect the actual OCXO science row, not
  // merely the legacy pps_residual compatibility object.  Keep Welford, PPB,
  // and servo input inert while recovery/reattach bookends settle.
  ocxo_science_row_suppress_for_recover_hold(ocxo1_science);
  ocxo_science_row_suppress_for_recover_hold(ocxo2_science);

  clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                 g_clocks_feature_science_residuals,
                                 system_feature_status_t::INITIALIZING,
                                 true);
  return true;
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

// WATCHDOG_ANOMALY is a campaign-continuity surrender, not an always-on
// VCLOCK/PPS housekeeping alarm.  START/RECOVER briefly put Beta into
// STARTED/name-owned state before the first public campaign record row exists;
// those pre-row tribunal verdicts must remain local diagnostics.
static volatile bool watchdog_campaign_publication_armed = false;

// Sticky local circuit breaker.  WATCHDOG_ANOMALY is not merely a message to
// the Pi; it is a Teensy-side campaign-continuity surrender.  Once latched,
// Beta refuses to publish any more campaign record rows from the current campaign
// until an explicit lifecycle boundary clears the latch.
static volatile bool     watchdog_campaign_surrendered = false;
static volatile uint32_t watchdog_campaign_surrender_count = 0;
static volatile uint32_t watchdog_anomaly_suppressed_unarmed_count = 0;
static volatile uint32_t watchdog_anomaly_verbose_publish_count = 0;
static volatile uint32_t watchdog_anomaly_legacy_publish_count = 0;

// Live servo-mode command handoff.
//
// SERVOS is the sole operator control surface. Campaign START, STOP, FLASH_CUT,
// and RECOVER neither author nor clear servo mode. Foreground command context
// records the desired mode; Beta applies it at a safe completed-row boundary
// while campaign publication is live, or immediately while the instrument is
// otherwise idle.
static volatile bool          request_servo_mode_change = false;
static volatile servo_mode_t  requested_servo_mode = servo_mode_t::OFF;
static uint32_t               g_servo_mode_request_count = 0;
static uint32_t               g_servo_mode_commit_count = 0;
static servo_mode_t           g_servo_mode_last_requested = servo_mode_t::OFF;
static servo_mode_t           g_servo_mode_last_committed = servo_mode_t::OFF;

static void clocks_apply_servo_mode_now(servo_mode_t mode);

static void clocks_watchdog_disarm_campaign_publication(void) {
  watchdog_campaign_publication_armed = false;
}

static void clocks_watchdog_clear_surrender_for_new_lifecycle(void) {
  watchdog_campaign_surrendered = false;
  watchdog_anomaly_active = false;
  watchdog_anomaly_publish_pending = false;
}

static void clocks_watchdog_arm_campaign_publication(void) {
  interrupt_dwt_publication_launch_acquisition_end();
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  watchdog_campaign_publication_armed = true;
}

static bool clocks_watchdog_publication_blocked(void) {
  return watchdog_campaign_surrendered || watchdog_anomaly_active;
}

bool clocks_watchdog_campaign_armed(void) {
  // Campaign name is operator/persistence metadata, not timing custody.
  // Warm RECOVER can resume a known public clock ledger before the Pi has
  // supplied or restored a human campaign label; do not leave the campaign
  // continuity watchdog disarmed merely because the label is empty.
  return watchdog_campaign_publication_armed &&
         !clocks_watchdog_publication_blocked() &&
         campaign_state == clocks_campaign_state_t::STARTED &&
         campaign_seconds != 0ULL &&
         !request_start &&
         !request_stop &&
         !request_recover &&
         !request_zero &&
         !request_flash_cut &&
         !campaign_warmup_active();
}

// ============================================================================
// CLOCKS-local Payload numeric integrity court
// ============================================================================
//
// Payload owns generic storage integrity.  Beta owns the recovery/control-plane
// semantics: a present-but-bad recovery numeric is not an optional-missing
// field; it is a campaign-continuity violation.  Keep the court local to
// CLOCKS so Payload remains a generic carrier while the watchdog anomaly can
// carry CLOCKS path/lane/field evidence.

void clocks_watchdog_anomaly_payload(const char* reason,
                                     const Payload& payload,
                                     uint32_t detail0,
                                     uint32_t detail1,
                                     uint32_t detail2,
                                     uint32_t detail3);

static volatile bool g_clocks_payload_numeric_integrity_failed DMAMEM = false;
static volatile uint32_t g_clocks_payload_numeric_integrity_fail_count DMAMEM = 0;
static volatile uint32_t g_clocks_payload_numeric_last_reject_reason DMAMEM = 0;
static char g_clocks_payload_numeric_last_path[64] DMAMEM = {0};
static char g_clocks_payload_numeric_last_key[64] DMAMEM = {0};
static char g_clocks_payload_numeric_last_token_preview[48] DMAMEM = {0};

static constexpr size_t CLOCKS_PAYLOAD_NUMERIC_TOKEN_SCAN_LIMIT = 96U;
static constexpr size_t CLOCKS_PAYLOAD_NUMERIC_TOKEN_PREVIEW_LIMIT = 40U;


enum clocks_payload_numeric_reject_reason_t : uint32_t {
  CLOCKS_PAYLOAD_NUMERIC_OK = 0,
  CLOCKS_PAYLOAD_NUMERIC_NON_PRIMITIVE_OR_INVALID = 1,
  CLOCKS_PAYLOAD_NUMERIC_EMPTY = 2,
  CLOCKS_PAYLOAD_NUMERIC_TOO_LONG = 3,
  CLOCKS_PAYLOAD_NUMERIC_NO_DIGITS = 4,
  CLOCKS_PAYLOAD_NUMERIC_BAD_CHARACTER = 5,
  CLOCKS_PAYLOAD_NUMERIC_BAD_DECIMAL_POINT = 6,
  CLOCKS_PAYLOAD_NUMERIC_BAD_EXPONENT = 7,
  CLOCKS_PAYLOAD_NUMERIC_EXPONENT_TOO_LARGE = 8,
  CLOCKS_PAYLOAD_NUMERIC_DOUBLE_OVERFLOW_RISK = 9,
  CLOCKS_PAYLOAD_NUMERIC_DOUBLE_UNDERFLOW_RISK = 10,
  CLOCKS_PAYLOAD_NUMERIC_UINT64_OVERFLOW = 11,
  CLOCKS_PAYLOAD_NUMERIC_LIBC_PARSE_FAILED = 12,
  CLOCKS_PAYLOAD_NUMERIC_LIBC_NONFINITE = 13,
};

enum clocks_payload_checked_status_t : uint8_t {
  CLOCKS_PAYLOAD_FIELD_MISSING = 0,
  CLOCKS_PAYLOAD_FIELD_OK = 1,
  CLOCKS_PAYLOAD_FIELD_INVALID = 2,
};

struct clocks_payload_numeric_court_t {
  uint32_t reason = CLOCKS_PAYLOAD_NUMERIC_OK;
  uint32_t token_len = 0;
  bool negative = false;
  bool exponent_present = false;
  bool exponent_negative = false;
  uint32_t exponent_abs = 0;
  int32_t magnitude10 = 0;
  uint32_t digit_count = 0;
  uint32_t integer_digit_count = 0;
  char token_preview[CLOCKS_PAYLOAD_NUMERIC_TOKEN_PREVIEW_LIMIT] = {0};
};

// Numeric court state used to live as automatic stack objects.  That is exactly
// the wrong place during this crash hunt: aggregate zero-initialization becomes
// a memset into the upper DTCM stack neighborhood and can fault before it tells
// us anything useful.  Keep the scratch court in RAM2/DMAMEM and manually clear
// only the fields we consume.
static clocks_payload_numeric_court_t
    g_clocks_payload_numeric_court_scratch DMAMEM = {};

static FLASHMEM clocks_payload_numeric_court_t&
clocks_payload_numeric_court_scratch(void) {
  clocks_payload_numeric_court_t& c = g_clocks_payload_numeric_court_scratch;
  c.reason = CLOCKS_PAYLOAD_NUMERIC_OK;
  c.token_len = 0U;
  c.negative = false;
  c.exponent_present = false;
  c.exponent_negative = false;
  c.exponent_abs = 0U;
  c.magnitude10 = 0;
  c.digit_count = 0U;
  c.integer_digit_count = 0U;
  c.token_preview[0] = '\0';
  return c;
}

static inline void clocks_payload_numeric_integrity_reset(void) {
  g_clocks_payload_numeric_integrity_failed = false;
}

static FLASHMEM const char* clocks_payload_numeric_reject_reason_name(
    uint32_t reason) {
  switch (reason) {
    case CLOCKS_PAYLOAD_NUMERIC_OK: return "OK";
    case CLOCKS_PAYLOAD_NUMERIC_NON_PRIMITIVE_OR_INVALID:
      return "NON_PRIMITIVE_OR_INVALID";
    case CLOCKS_PAYLOAD_NUMERIC_EMPTY: return "EMPTY";
    case CLOCKS_PAYLOAD_NUMERIC_TOO_LONG: return "TOO_LONG";
    case CLOCKS_PAYLOAD_NUMERIC_NO_DIGITS: return "NO_DIGITS";
    case CLOCKS_PAYLOAD_NUMERIC_BAD_CHARACTER: return "BAD_CHARACTER";
    case CLOCKS_PAYLOAD_NUMERIC_BAD_DECIMAL_POINT: return "BAD_DECIMAL_POINT";
    case CLOCKS_PAYLOAD_NUMERIC_BAD_EXPONENT: return "BAD_EXPONENT";
    case CLOCKS_PAYLOAD_NUMERIC_EXPONENT_TOO_LARGE: return "EXPONENT_TOO_LARGE";
    case CLOCKS_PAYLOAD_NUMERIC_DOUBLE_OVERFLOW_RISK:
      return "DOUBLE_OVERFLOW_RISK";
    case CLOCKS_PAYLOAD_NUMERIC_DOUBLE_UNDERFLOW_RISK:
      return "DOUBLE_UNDERFLOW_RISK";
    case CLOCKS_PAYLOAD_NUMERIC_UINT64_OVERFLOW: return "UINT64_OVERFLOW";
    case CLOCKS_PAYLOAD_NUMERIC_LIBC_PARSE_FAILED: return "LIBC_PARSE_FAILED";
    case CLOCKS_PAYLOAD_NUMERIC_LIBC_NONFINITE: return "LIBC_NONFINITE";
    default: return "UNKNOWN";
  }
}

static FLASHMEM size_t clocks_payload_bounded_strlen(const char* s,
                                                     size_t cap) {
  if (!s) return 0;
  size_t n = 0;
  while (n < cap && s[n] != '\0') n++;
  return n;
}

static FLASHMEM void clocks_payload_copy_token_preview(
    char* dst,
    size_t dst_cap,
    const char* token,
    size_t token_len) {
  if (!dst || dst_cap == 0U) return;
  size_t n = token_len;
  if (n > dst_cap - 1U) n = dst_cap - 1U;
  if (token && n != 0U) memcpy(dst, token, n);
  dst[n] = '\0';
}

static FLASHMEM void clocks_payload_numeric_note_failure(
    const char* path,
    const char* key,
    const clocks_payload_numeric_court_t& court) {
  g_clocks_payload_numeric_integrity_failed = true;
  g_clocks_payload_numeric_integrity_fail_count++;
  g_clocks_payload_numeric_last_reject_reason = court.reason;
  safeCopy(g_clocks_payload_numeric_last_path,
           sizeof(g_clocks_payload_numeric_last_path),
           path ? path : "");
  safeCopy(g_clocks_payload_numeric_last_key,
           sizeof(g_clocks_payload_numeric_last_key),
           key ? key : "");
  safeCopy(g_clocks_payload_numeric_last_token_preview,
           sizeof(g_clocks_payload_numeric_last_token_preview),
           court.token_preview);
}

static FLASHMEM void clocks_payload_numeric_emit_watchdog(
    const Payload& payload,
    const char* context,
    const char* path,
    const char* lane,
    const char* field,
    const char* key,
    const clocks_payload_numeric_court_t& court) {
  clocks_payload_numeric_note_failure(path, key, court);

  if (!clocks_watchdog_campaign_armed() && !watchdog_campaign_surrendered) {
    return;
  }

  Payload p;
  p.add("schema", "CLOCKS_PAYLOAD_NUMERIC_INTEGRITY_V1");
  p.add("reason", "payload_numeric_integrity_failed");
  p.add("context", context ? context : "payload_numeric");
  p.add("path", path ? path : "");
  p.add("lane", lane ? lane : "");
  p.add("field", field ? field : "");
  p.add("key", key ? key : "");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("reject_reason_id", court.reason);
  p.add("reject_reason", clocks_payload_numeric_reject_reason_name(court.reason));
  p.add("token_len", court.token_len);
  p.add("token_preview", court.token_preview);
  p.add("negative", court.negative);
  p.add("exponent_present", court.exponent_present);
  p.add("exponent_negative", court.exponent_negative);
  p.add("exponent_abs", court.exponent_abs);
  p.add("magnitude10", court.magnitude10);
  p.add("digit_count", court.digit_count);
  p.add("integer_digit_count", court.integer_digit_count);
  p.add("payload_count", (uint32_t)payload.count());
  p.add("payload_arena_used", (uint32_t)payload.arena_used());
  p.add("payload_arena_cap", (uint32_t)payload.arena_capacity());
  p.add("payload_entry_cap", (uint32_t)payload.entry_capacity());
  p.add("integrity_fail_count", (uint32_t)g_clocks_payload_numeric_integrity_fail_count);
  p.add("watchdog_campaign_armed", clocks_watchdog_campaign_armed());

  clocks_watchdog_anomaly_payload("payload_numeric_integrity_failed",
                                  p,
                                  court.reason,
                                  court.token_len,
                                  (uint32_t)payload.count(),
                                  (uint32_t)payload.arena_used());
}

static FLASHMEM clocks_payload_checked_status_t clocks_payload_fetch_token(
    const Payload& payload,
    const char* key,
    const char* context,
    const char* path,
    const char* lane,
    const char* field,
    const char*& token,
    size_t& token_len) {
  token = nullptr;
  token_len = 0;
  if (!key || !payload.has(key)) {
    return CLOCKS_PAYLOAD_FIELD_MISSING;
  }

  token = payload.getString(key);
  if (!token) {
    clocks_payload_numeric_court_t& court = clocks_payload_numeric_court_scratch();
    court.reason = CLOCKS_PAYLOAD_NUMERIC_NON_PRIMITIVE_OR_INVALID;
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }

  token_len = clocks_payload_bounded_strlen(
      token, CLOCKS_PAYLOAD_NUMERIC_TOKEN_SCAN_LIMIT);
  if (token_len >= CLOCKS_PAYLOAD_NUMERIC_TOKEN_SCAN_LIMIT) {
    clocks_payload_numeric_court_t& court = clocks_payload_numeric_court_scratch();
    court.reason = CLOCKS_PAYLOAD_NUMERIC_TOO_LONG;
    court.token_len = (uint32_t)token_len;
    clocks_payload_copy_token_preview(court.token_preview,
                                      sizeof(court.token_preview),
                                      token,
                                      token_len);
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }

  return CLOCKS_PAYLOAD_FIELD_OK;
}

static FLASHMEM bool clocks_payload_numeric_char_is_digit(char c) {
  return c >= '0' && c <= '9';
}

static FLASHMEM bool clocks_payload_validate_uint64_token(
    const char* token,
    size_t token_len,
    clocks_payload_numeric_court_t& court,
    uint64_t& out) {
  out = 0ULL;
  court.token_len = (uint32_t)token_len;
  clocks_payload_copy_token_preview(court.token_preview,
                                    sizeof(court.token_preview),
                                    token,
                                    token_len);

  if (!token || token_len == 0U) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_EMPTY;
    return false;
  }
  if (token_len > 20U) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_TOO_LONG;
    return false;
  }

  uint64_t v = 0ULL;
  for (size_t i = 0; i < token_len; i++) {
    const char c = token[i];
    if (!clocks_payload_numeric_char_is_digit(c)) {
      court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_CHARACTER;
      return false;
    }
    const uint32_t d = (uint32_t)(c - '0');
    if (v > (UINT64_MAX - (uint64_t)d) / 10ULL) {
      court.reason = CLOCKS_PAYLOAD_NUMERIC_UINT64_OVERFLOW;
      return false;
    }
    v = v * 10ULL + (uint64_t)d;
    court.digit_count++;
    court.integer_digit_count++;
  }

  out = v;
  court.reason = CLOCKS_PAYLOAD_NUMERIC_OK;
  return true;
}

static FLASHMEM bool clocks_payload_validate_double_token(
    const char* token,
    size_t token_len,
    clocks_payload_numeric_court_t& court) {
  court.token_len = (uint32_t)token_len;
  clocks_payload_copy_token_preview(court.token_preview,
                                    sizeof(court.token_preview),
                                    token,
                                    token_len);

  if (!token || token_len == 0U) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_EMPTY;
    return false;
  }
  if (token_len >= CLOCKS_PAYLOAD_NUMERIC_TOKEN_SCAN_LIMIT) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_TOO_LONG;
    return false;
  }

  size_t i = 0;
  if (token[i] == '+' || token[i] == '-') {
    court.negative = token[i] == '-';
    i++;
    if (i >= token_len) {
      court.reason = CLOCKS_PAYLOAD_NUMERIC_NO_DIGITS;
      return false;
    }
  }

  bool seen_dot = false;
  bool seen_digit = false;
  bool seen_nonzero = false;
  uint32_t integer_digit_count = 0;
  uint32_t leading_integer_zero_count = 0;
  uint32_t fractional_digit_index = 0;
  uint32_t first_nonzero_fraction_index = 0;
  bool first_nonzero_in_fraction = false;

  while (i < token_len) {
    const char c = token[i];
    if (clocks_payload_numeric_char_is_digit(c)) {
      seen_digit = true;
      court.digit_count++;
      if (!seen_dot) {
        integer_digit_count++;
        if (!seen_nonzero && c == '0') {
          leading_integer_zero_count++;
        }
        if (c != '0' && !seen_nonzero) {
          seen_nonzero = true;
        }
      } else {
        if (c != '0' && !seen_nonzero) {
          seen_nonzero = true;
          first_nonzero_in_fraction = true;
          first_nonzero_fraction_index = fractional_digit_index;
        }
        fractional_digit_index++;
      }
      i++;
      continue;
    }

    if (c == '.') {
      if (seen_dot) {
        court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_DECIMAL_POINT;
        return false;
      }
      seen_dot = true;
      i++;
      continue;
    }

    if (c == 'e' || c == 'E') {
      court.exponent_present = true;
      i++;
      if (i >= token_len) {
        court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_EXPONENT;
        return false;
      }
      if (token[i] == '+' || token[i] == '-') {
        court.exponent_negative = token[i] == '-';
        i++;
        if (i >= token_len) {
          court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_EXPONENT;
          return false;
        }
      }
      bool exp_digit = false;
      uint32_t exp_abs = 0;
      while (i < token_len) {
        const char e = token[i];
        if (!clocks_payload_numeric_char_is_digit(e)) {
          court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_CHARACTER;
          return false;
        }
        exp_digit = true;
        const uint32_t d = (uint32_t)(e - '0');
        if (exp_abs > 1000000U) {
          court.reason = CLOCKS_PAYLOAD_NUMERIC_EXPONENT_TOO_LARGE;
          return false;
        }
        exp_abs = exp_abs * 10U + d;
        i++;
      }
      if (!exp_digit) {
        court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_EXPONENT;
        return false;
      }
      court.exponent_abs = exp_abs;
      break;
    }

    court.reason = CLOCKS_PAYLOAD_NUMERIC_BAD_CHARACTER;
    return false;
  }

  if (!seen_digit || court.digit_count == 0U) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_NO_DIGITS;
    return false;
  }

  court.integer_digit_count = integer_digit_count;

  // Even an all-zero mantissa with a wildly large exponent is not useful
  // recovery evidence, and keeping it away from libc avoids implementation
  // range/error side paths during this investigation.
  if (court.exponent_present && court.exponent_abs > 400U) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_EXPONENT_TOO_LARGE;
    return false;
  }

  if (seen_nonzero) {
    int32_t magnitude10 = 0;
    if (first_nonzero_in_fraction) {
      magnitude10 = -(int32_t)(first_nonzero_fraction_index + 1U);
    } else {
      const uint32_t significant_integer_digits =
          integer_digit_count - leading_integer_zero_count;
      magnitude10 = significant_integer_digits
          ? (int32_t)(significant_integer_digits - 1U)
          : 0;
    }

    if (court.exponent_present) {
      if (court.exponent_abs > 10000U) {
        court.reason = CLOCKS_PAYLOAD_NUMERIC_EXPONENT_TOO_LARGE;
        return false;
      }
      const int32_t signed_exp = court.exponent_negative
          ? -(int32_t)court.exponent_abs
          : (int32_t)court.exponent_abs;
      magnitude10 += signed_exp;
    }

    court.magnitude10 = magnitude10;

    // Keep libc away from range-error paths.  These limits are deliberately
    // conservative for recovery science fields: legitimate Welford/DAC values
    // are nowhere near IEEE double overflow or subnormal-underflow territory.
    if (magnitude10 > 307) {
      court.reason = CLOCKS_PAYLOAD_NUMERIC_DOUBLE_OVERFLOW_RISK;
      return false;
    }
    if (magnitude10 < -300) {
      court.reason = CLOCKS_PAYLOAD_NUMERIC_DOUBLE_UNDERFLOW_RISK;
      return false;
    }
  }

  court.reason = CLOCKS_PAYLOAD_NUMERIC_OK;
  return true;
}

static FLASHMEM clocks_payload_checked_status_t clocks_payload_try_get_u64_checked(
    const Payload& payload,
    const char* key,
    const char* context,
    const char* path,
    const char* lane,
    const char* field,
    uint64_t& out) {
  const char* token = nullptr;
  size_t token_len = 0;
  const clocks_payload_checked_status_t fetched =
      clocks_payload_fetch_token(payload, key, context, path, lane, field,
                                 token, token_len);
  if (fetched != CLOCKS_PAYLOAD_FIELD_OK) return fetched;

  clocks_payload_numeric_court_t& court = clocks_payload_numeric_court_scratch();
  if (!clocks_payload_validate_uint64_token(token, token_len, court, out)) {
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }
  return CLOCKS_PAYLOAD_FIELD_OK;
}

static FLASHMEM clocks_payload_checked_status_t clocks_payload_try_get_double_checked(
    const Payload& payload,
    const char* key,
    const char* context,
    const char* path,
    const char* lane,
    const char* field,
    double& out) {
  const char* token = nullptr;
  size_t token_len = 0;
  const clocks_payload_checked_status_t fetched =
      clocks_payload_fetch_token(payload, key, context, path, lane, field,
                                 token, token_len);
  if (fetched != CLOCKS_PAYLOAD_FIELD_OK) return fetched;

  clocks_payload_numeric_court_t& court = clocks_payload_numeric_court_scratch();
  if (!clocks_payload_validate_double_token(token, token_len, court)) {
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }

  errno = 0;
  char* end = nullptr;
  out = strtod(token, &end);
  if (errno == ERANGE ||
      !end || end != token + token_len || *end != '\0') {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_LIBC_PARSE_FAILED;
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }
  if (!isfinite(out)) {
    court.reason = CLOCKS_PAYLOAD_NUMERIC_LIBC_NONFINITE;
    clocks_payload_numeric_emit_watchdog(payload, context, path, lane,
                                         field, key, court);
    return CLOCKS_PAYLOAD_FIELD_INVALID;
  }
  return CLOCKS_PAYLOAD_FIELD_OK;
}

static FLASHMEM Payload clocks_payload_numeric_reject_response(
    const char* status) {
  Payload err;
  err.add("error", "payload numeric integrity failure");
  err.add("status", status ? status : "rejected_numeric_integrity");
  err.add("numeric_integrity_fail_count",
          (uint32_t)g_clocks_payload_numeric_integrity_fail_count);
  err.add("numeric_last_reject_reason_id",
          (uint32_t)g_clocks_payload_numeric_last_reject_reason);
  err.add("numeric_last_reject_reason",
          clocks_payload_numeric_reject_reason_name(
              g_clocks_payload_numeric_last_reject_reason));
  err.add("numeric_last_path", g_clocks_payload_numeric_last_path);
  err.add("numeric_last_key", g_clocks_payload_numeric_last_key);
  err.add("numeric_last_token_preview",
          g_clocks_payload_numeric_last_token_preview);
  return err;
}

// ============================================================================
// Payload helpers — standardized publication
// ============================================================================


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

static FLASHMEM void payload_add_smartzero_delay_transaction(Payload& p) {
  clocks_alpha_smartzero_delay_snapshot_t& s =
      g_beta_report_smartzero_delay_scratch;
  s = clocks_alpha_smartzero_delay_snapshot_t{};
  const bool available = clocks_alpha_smartzero_delay_snapshot(&s);

  p.add("smartzero_delay_available", available);
  p.add("smartzero_delay_valid", available && s.valid);
  p.add("smartzero_delay_reference_from_pps_vclock",
        available && s.reference_from_pps_vclock);
  p.add("smartzero_delay_all_minimums_satisfied",
        available && s.all_minimums_satisfied);
  p.add("smartzero_delay_epoch_sequence",
        available ? s.epoch_sequence : 0U);
  p.add("smartzero_delay_smartzero_sequence",
        available ? s.smartzero_sequence : 0U);
  p.add("smartzero_delay_install_count",
        available ? s.install_count : 0U);
  p.add("smartzero_delay_minimum_us",
        available ? s.minimum_separation_us
                  : CLOCKS_SMARTZERO_MIN_EDGE_SEPARATION_US);
  p.add("smartzero_delay_minimum_cycles",
        available ? s.minimum_separation_cycles : 0U);
  p.add("smartzero_delay_reference_dwt", available ? s.reference_dwt : 0U);

  p.add("smartzero_delay_ocxo1_zero_ok", available && s.ocxo1_zero_ok);
  p.add("smartzero_delay_ocxo1_minimum_satisfied",
        available && s.ocxo1_minimum_satisfied);
  p.add("smartzero_delay_ocxo1_earliest_dwt",
        available ? s.ocxo1_earliest_dwt : 0U);
  p.add("smartzero_delay_ocxo1_install_begin_dwt",
        available ? s.ocxo1_install_begin_dwt : 0U);
  p.add("smartzero_delay_ocxo1_install_complete_dwt",
        available ? s.ocxo1_install_complete_dwt : 0U);
  p.add("smartzero_delay_ocxo1_reference_gap_cycles",
        available ? s.ocxo1_reference_gap_cycles : 0U);
  p.add("smartzero_delay_ocxo1_lateness_cycles",
        available ? s.ocxo1_lateness_cycles : 0U);
  p.add("smartzero_delay_ocxo1_zero_counter32",
        available ? s.ocxo1_zero_counter32 : 0U);

  p.add("smartzero_delay_ocxo2_zero_ok", available && s.ocxo2_zero_ok);
  p.add("smartzero_delay_ocxo2_minimum_satisfied",
        available && s.ocxo2_minimum_satisfied);
  p.add("smartzero_delay_ocxo2_earliest_dwt",
        available ? s.ocxo2_earliest_dwt : 0U);
  p.add("smartzero_delay_ocxo2_install_begin_dwt",
        available ? s.ocxo2_install_begin_dwt : 0U);
  p.add("smartzero_delay_ocxo2_install_complete_dwt",
        available ? s.ocxo2_install_complete_dwt : 0U);
  p.add("smartzero_delay_ocxo2_from_ocxo1_gap_cycles",
        available ? s.ocxo2_from_ocxo1_gap_cycles : 0U);
  p.add("smartzero_delay_ocxo2_lateness_cycles",
        available ? s.ocxo2_lateness_cycles : 0U);
  p.add("smartzero_delay_ocxo2_zero_counter32",
        available ? s.ocxo2_zero_counter32 : 0U);
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
  payload_add_smartzero_delay_transaction(p);
}

static FLASHMEM void payload_add_visible_origin_snapshot(Payload& parent,
                                                const char* key,
                                                time_clock_id_t clock) {
  clocks_alpha_ocxo_visible_origin_snapshot_t& s = g_beta_report_visible_origin_scratch;
  s = clocks_alpha_ocxo_visible_origin_snapshot_t{};
  const bool available =
      clocks_alpha_ocxo_visible_origin_snapshot(clock, &s);

  Payload obj;
  obj.add("available", available);
  obj.add("valid", available && s.valid);
  obj.add("pending", available && s.pending);
  obj.add("phase_offset_in_range",
          available && s.phase_offset_in_range);
  obj.add("clock_id", available ? s.clock_id : 0U);
  obj.add("epoch_sequence", available ? s.epoch_sequence : 0U);
  obj.add("smartzero_sequence", available ? s.smartzero_sequence : 0U);
  obj.add("capture_count", available ? s.capture_count : 0U);
  obj.add("pps_vclock_dwt", available ? s.pps_vclock_dwt : 0U);
  obj.add("ocxo_anchor_dwt",
          available ? s.ocxo_anchor_dwt : 0U);
  obj.add("dwt_cycles_per_second",
          available ? s.dwt_cycles_per_second : 0U);
  obj.add("elapsed_cycles_since_pps_vclock",
          available ? s.elapsed_cycles_since_pps_vclock : 0U);
  obj.add("elapsed_ns_since_pps_vclock",
          available ? s.elapsed_ns_since_pps_vclock : 0ULL);
  obj.add("phase_offset_ns", available ? s.phase_offset_ns : 0U);
  obj.add("public_origin_valid",
          available && s.public_origin_valid);
  obj.add("public_origin_capture_count",
          available ? s.public_origin_capture_count : 0U);
  obj.add("public_origin_pps_sequence",
          available ? s.public_origin_pps_sequence : 0U);
  obj.add("public_origin_vclock_ns",
          available ? s.public_origin_vclock_ns : 0ULL);
  obj.add("public_origin_ocxo_ns_before_offset",
          available ? s.public_origin_ocxo_ns_before_offset : 0ULL);
  obj.add("public_origin_offset_ns",
          available ? s.public_origin_offset_ns : 0LL);
  obj.add("public_origin_ocxo_ns_after_offset",
          available ? s.public_origin_ocxo_ns_after_offset : 0ULL);

  parent.add_object(key, obj);
}

static FLASHMEM void payload_add_visible_origin_summary(Payload& p) {
  Payload visible_origin;
  payload_add_visible_origin_snapshot(visible_origin, "ocxo1",
                                      time_clock_id_t::OCXO1);
  payload_add_visible_origin_snapshot(visible_origin, "ocxo2",
                                      time_clock_id_t::OCXO2);
  p.add_object("visible_origin", visible_origin);
}

static FLASHMEM void payload_add_smartzero_summary(Payload& p) {
  interrupt_smartzero_snapshot_t& live = g_beta_report_live_smartzero_scratch;
  live = interrupt_smartzero_snapshot_t{};
  (void)interrupt_smartzero_live_snapshot(&live);

  interrupt_smartzero_snapshot_t& installed = g_beta_report_installed_smartzero_scratch;
  installed = interrupt_smartzero_snapshot_t{};
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
  payload_add_visible_origin_summary(p);
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


static FLASHMEM clocks_static_prediction_snapshot_t prediction_snapshot_for_pps(void) {
  clocks_static_prediction_snapshot_t s{};
  (void)clocks_static_prediction_pps_snapshot(&s);
  return s;
}

static FLASHMEM clocks_static_prediction_snapshot_t prediction_snapshot_for_clock(time_clock_id_t clock) {
  clocks_static_prediction_snapshot_t s{};
  (void)clocks_static_prediction_snapshot(clock, &s);
  return s;
}


// ============================================================================
// Command-report statistics serializers and typed MONITOR snapshots
// ============================================================================
//
// SYSTEM serializes the compact campaign row from the typed handoff.
// Deep forensic transcripts are no longer transported at 1 Hz. Pi-side CLOCKS
// persists the fragment and may retain an empty compatibility forensics object.
//
// The publication path uses the compact helpers below.

static FLASHMEM void payload_add_frequency_fields(Payload& obj, double ppb_value) {
  const double tau_value = 1.0 + ppb_value / 1e9;
  obj.add("tau", toFixedDecimal(tau_value, 12));
  obj.add("ppb", toFixedDecimal(ppb_value, 3));
}

static double campaign_total_tau_from_ratio(uint64_t reference_value,
                                             uint64_t clock_value) {
  if (reference_value == 0ULL) return 1.0;
  return (double)clock_value / (double)reference_value;
}

static double campaign_total_ppb_from_tau(double tau) {
  return (tau - 1.0) * 1.0e9;
}

// Report-only serializer: every child Payload header lives in RAM2 and is reused.
// It is called only while clocks_report_build_guard_t excludes priority-16
// TimePop/handoff entry. Typed MONITOR snapshots use independent RAM2 state.
static FLASHMEM void report_add_welford_object(Payload& parent,
                                               const char* key,
                                               const welford_t& w) {
  Payload& obj = g_report_child_welford;
  obj.clear();
  obj.add("n", w.n);
  obj.add("mean", toFixedDecimal(w.mean, 6));
  obj.add("stddev", toFixedDecimal(welford_stddev(w), 6));
  obj.add("stderr", toFixedDecimal(welford_stderr(w), 6));
  obj.add("min", toFixedDecimal((w.n > 0) ? w.min_val : 0.0, 6));
  obj.add("max", toFixedDecimal((w.n > 0) ? w.max_val : 0.0, 6));
  parent.add_object(key, obj);
  obj.clear();
}

static FLASHMEM void report_add_stats_clock(Payload& parent,
                                             const char* key,
                                             const welford_t& w,
                                             bool include_frequency,
                                             double ppb_value = 0.0) {
  Payload& obj = g_report_child_clock;
  obj.clear();
  report_add_welford_object(obj, "welford", w);
  if (include_frequency) payload_add_frequency_fields(obj, ppb_value);
  parent.add_object(key, obj);
  obj.clear();
}

static FLASHMEM void report_add_stats_summary_from_snapshot(
    Payload& p,
    const clocks_instrument_stats_snapshot_t& instrument) {
  Payload& stats = g_report_child_stats;
  stats.clear();
  stats.add("schema", "CLOCKS_INSTRUMENT_STATS_V2");
  stats.add("always_on", true);
  stats.add("owner", "ALPHA");
  stats.add("lifetime", "BOOT_TO_REBOOT_OR_STATS_RESET");
  stats.add("valid", instrument.valid);
  stats.add("reset_count", instrument.reset_count);
  stats.add("update_count", instrument.update_count);
  stats.add("last_pps_sequence", instrument.last_pps_sequence);
  stats.add("completed_row_coherent", instrument.completed_row_coherent);

  report_add_stats_clock(stats, "gnss", instrument.gnss_welford, true, 0.0);
  report_add_stats_clock(stats, "dwt", instrument.dwt_welford, true,
                         instrument.dwt_frequency.ppb);
  report_add_stats_clock(stats, "vclock", instrument.vclock_welford, true,
                         instrument.vclock_frequency.ppb);
  report_add_stats_clock(stats, "ocxo1", instrument.ocxo1_welford, true,
                         instrument.ocxo1_frequency.ppb);
  report_add_stats_clock(stats, "ocxo2", instrument.ocxo2_welford, true,
                         instrument.ocxo2_frequency.ppb);
  report_add_stats_clock(stats, "pps_witness",
                         instrument.pps_witness_welford, false);

  Payload& maturity = g_report_child_maturity;
  maturity.clear();
  maturity.add("gnss_samples", instrument.gnss_welford.n);
  maturity.add("dwt_samples", instrument.dwt_frequency.sample_count);
  maturity.add("vclock_samples", instrument.vclock_frequency.sample_count);
  maturity.add("vclock_intervals", instrument.vclock_frequency.interval_count);
  maturity.add("ocxo1_samples", instrument.ocxo1_frequency.sample_count);
  maturity.add("ocxo1_intervals", instrument.ocxo1_frequency.interval_count);
  maturity.add("ocxo1_stderr_ppb",
               toFixedDecimal(instrument.ocxo1_frequency.stderr_ppb, 6));
  maturity.add("ocxo2_samples", instrument.ocxo2_frequency.sample_count);
  maturity.add("ocxo2_intervals", instrument.ocxo2_frequency.interval_count);
  maturity.add("ocxo2_stderr_ppb",
               toFixedDecimal(instrument.ocxo2_frequency.stderr_ppb, 6));
  stats.add_object("maturity", maturity);
  maturity.clear();

  Payload& admission = g_report_child_admission;
  admission.clear();
  admission.add("interval_min_cycles", 900000000UL);
  admission.add("interval_max_cycles", 1100000000UL);
  admission.add("vclock_reject_count", instrument.vclock_interval_reject_count);
  admission.add("ocxo1_reject_count", instrument.ocxo1_interval_reject_count);
  admission.add("ocxo2_reject_count", instrument.ocxo2_interval_reject_count);
  stats.add_object("interval_admission", admission);
  admission.clear();

  Payload& dac = g_report_child_dac;
  dac.clear();
  report_add_welford_object(dac, "ocxo1", instrument.ocxo1_dac_welford);
  report_add_welford_object(dac, "ocxo2", instrument.ocxo2_dac_welford);
  stats.add_object("dac", dac);
  dac.clear();

  p.add_object("stats", stats);
  stats.clear();
}

static FLASHMEM clocks_monitor_welford_snapshot_t clocks_monitor_welford_snapshot(
    const welford_t& w) {
  clocks_monitor_welford_snapshot_t out{};
  out.n = w.n;
  out.mean = w.mean;
  out.m2 = w.m2;
  out.stddev = welford_stddev(w);
  out.stderr_value = welford_stderr(w);
  out.min = (w.n > 0U) ? w.min_val : 0.0;
  out.max = (w.n > 0U) ? w.max_val : 0.0;
  return out;
}

static FLASHMEM clocks_monitor_stats_clock_snapshot_t clocks_monitor_stats_clock(
    const welford_t& w,
    bool frequency_present,
    double ppb) {
  clocks_monitor_stats_clock_snapshot_t out{};
  out.welford = clocks_monitor_welford_snapshot(w);
  out.frequency_present = frequency_present;
  out.ppb = frequency_present ? ppb : 0.0;
  out.tau = frequency_present ? (1.0 + ppb / 1.0e9) : 1.0;
  return out;
}

static FLASHMEM clocks_monitor_tau_recovery_snapshot_t
clocks_monitor_tau_recovery_snapshot(
    const clocks_alpha_tau_snapshot_t& state) {
  clocks_monitor_tau_recovery_snapshot_t out{};
  out.valid = state.valid;
  out.reset_count = state.reset_count;
  out.sample_count = state.sample_count;
  out.interval_count = state.interval_count;
  out.reject_count = state.reject_count;
  out.gap_reset_count = state.gap_reset_count;
  out.last_pps_sequence = state.last_pps_sequence;
  out.last_interval_pps_sequence = state.last_interval_pps_sequence;
  out.first_refined_ns = state.first_refined_ns;
  out.last_refined_ns = state.last_refined_ns;
  out.last_fast_residual_ns = state.last_fast_residual_ns;
  out.cumulative_reference_ns = state.cumulative_reference_ns;
  out.cumulative_clock_ns = state.cumulative_clock_ns;
  out.cumulative_clock_ns_exact = state.cumulative_clock_ns_exact;
  out.mean_x = state.mean_x;
  out.mean_y = state.mean_y;
  out.sxx = state.sxx;
  out.sxy = state.sxy;
  out.syy = state.syy;
  out.interval_mean_ppb = state.interval_mean_ppb;
  out.interval_m2_ppb = state.interval_m2_ppb;
  return out;
}

static FLASHMEM void clocks_monitor_stats_snapshot_from_instrument(
    clocks_monitor_stats_snapshot_t& out,
    const clocks_instrument_stats_snapshot_t& instrument) {
  memset(&out, 0, sizeof(out));
  out.valid = instrument.valid;
  out.reset_count = instrument.reset_count;
  out.update_count = instrument.update_count;
  out.last_pps_sequence = instrument.last_pps_sequence;
  out.completed_row_coherent = instrument.completed_row_coherent;

  out.gnss = clocks_monitor_stats_clock(instrument.gnss_welford, true, 0.0);
  out.dwt = clocks_monitor_stats_clock(
      instrument.dwt_welford, true, instrument.dwt_frequency.ppb);
  out.vclock = clocks_monitor_stats_clock(
      instrument.vclock_welford, true, instrument.vclock_frequency.ppb);
  out.ocxo1 = clocks_monitor_stats_clock(
      instrument.ocxo1_welford, true, instrument.ocxo1_frequency.ppb);
  out.ocxo2 = clocks_monitor_stats_clock(
      instrument.ocxo2_welford, true, instrument.ocxo2_frequency.ppb);
  out.pps_witness = clocks_monitor_stats_clock(
      instrument.pps_witness_welford, false, 0.0);
  out.ocxo1_tau_state = clocks_monitor_tau_recovery_snapshot(
      instrument.ocxo1_tau_state);
  out.ocxo2_tau_state = clocks_monitor_tau_recovery_snapshot(
      instrument.ocxo2_tau_state);

  out.maturity_gnss_samples = instrument.gnss_welford.n;
  out.maturity_dwt_samples = instrument.dwt_frequency.sample_count;
  out.maturity_vclock_samples = instrument.vclock_frequency.sample_count;
  out.maturity_vclock_intervals = instrument.vclock_frequency.interval_count;
  out.maturity_ocxo1_samples = instrument.ocxo1_frequency.sample_count;
  out.maturity_ocxo1_intervals = instrument.ocxo1_frequency.interval_count;
  out.maturity_ocxo1_stderr_ppb = instrument.ocxo1_frequency.stderr_ppb;
  out.maturity_ocxo2_samples = instrument.ocxo2_frequency.sample_count;
  out.maturity_ocxo2_intervals = instrument.ocxo2_frequency.interval_count;
  out.maturity_ocxo2_stderr_ppb = instrument.ocxo2_frequency.stderr_ppb;

  out.interval_min_cycles = CLOCKS_DELTA_RAW_INTERVAL_MIN_CYCLES;
  out.interval_max_cycles = CLOCKS_DELTA_RAW_INTERVAL_MAX_CYCLES;
  out.vclock_reject_count = instrument.vclock_interval_reject_count;
  out.ocxo1_reject_count = instrument.ocxo1_interval_reject_count;
  out.ocxo2_reject_count = instrument.ocxo2_interval_reject_count;
  out.ocxo1_dac = clocks_monitor_welford_snapshot(instrument.ocxo1_dac_welford);
  out.ocxo2_dac = clocks_monitor_welford_snapshot(instrument.ocxo2_dac_welford);
}


static FLASHMEM void clocks_beta_cold_diagnostics_init(void) {
  if (g_clocks_beta_dmamem_initialized) return;

  // Teensy places DMAMEM in the NOLOAD .bss.dma section.  Startup clears
  // ordinary BSS only, so explicitly initialize every Beta RAM2 object before
  // campaign control, reporting, recovery, or the 1 Hz science path can run.
  g_start_phaseledger_last_ocxo1 =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_start_phaseledger_last_ocxo2 =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_counterledger_raw_scratch =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo1_counterledger_row =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo2_counterledger_row =
      clocks_alpha_ocxo_counterledger_snapshot_t{};

  clocks_stack_witness_t* stack_witness =
      (clocks_stack_witness_t*)&g_clocks_stack_witness;
  memset(stack_witness, 0, sizeof(*stack_witness));
  stack_witness->magic = CLOCKS_STACK_WITNESS_MAGIC;
  stack_witness->reset_count = 1U;
  stack_witness->min_sp = UINT32_MAX;

  g_ocxo_science_totals_ocxo1 = ocxo_science_totals_t{};
  g_ocxo_science_totals_ocxo2 = ocxo_science_totals_t{};
  g_delta_previous_vclock_reference = delta_residual_reference_t{};

  g_beta_pps_vclock_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_pps_ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_pps_ocxo2_forensics = clocks_alpha_lane_forensics_t{};

  g_beta_start_ocxo1_projection =
      clocks_alpha_ocxo_pps_projection_snapshot_t{};
  g_beta_start_ocxo2_projection =
      clocks_alpha_ocxo_pps_projection_snapshot_t{};
  g_beta_pps_ocxo1_projection =
      clocks_alpha_ocxo_pps_projection_snapshot_t{};
  g_beta_pps_ocxo2_projection =
      clocks_alpha_ocxo_pps_projection_snapshot_t{};
  g_beta_pps_vclock_edge_forensics =
      clocks_pps_vclock_edge_forensics_t{};
  g_beta_pps_ocxo1_alpha_tau = clocks_alpha_tau_snapshot_t{};
  g_beta_pps_ocxo2_alpha_tau = clocks_alpha_tau_snapshot_t{};
  g_beta_pps_cycle_prediction = clocks_static_prediction_snapshot_t{};
  g_beta_vclock_cycle_prediction = clocks_static_prediction_snapshot_t{};
  g_beta_ocxo1_cycle_prediction = clocks_static_prediction_snapshot_t{};
  g_beta_ocxo2_cycle_prediction = clocks_static_prediction_snapshot_t{};

  g_beta_report_live_smartzero_scratch = interrupt_smartzero_snapshot_t{};
  g_beta_report_installed_smartzero_scratch =
      interrupt_smartzero_snapshot_t{};
      interrupt_integrity_snapshot_t{};
      clocks_alpha_integrity_snapshot_t{};
      clocks_alpha_lane_forensics_t{};
      clocks_alpha_lane_forensics_t{};
      clocks_alpha_lane_forensics_t{};
      clocks_alpha_lane_forensics_t{};
  g_beta_report_visible_origin_scratch =
      clocks_alpha_ocxo_visible_origin_snapshot_t{};
      clocks_alpha_ocxo_pps_projection_snapshot_t{};

  g_beta_vclock_science_row = clock_science_row_t{};
  g_beta_ocxo1_science_row = clock_science_row_t{};
  g_beta_ocxo2_science_row = clock_science_row_t{};
  g_beta_probe_ocxo1_science_row = clock_science_row_t{};
  g_beta_probe_ocxo2_science_row = clock_science_row_t{};
  g_beta_probe_science_totals_o1 = ocxo_science_totals_t{};
  g_beta_probe_science_totals_o2 = ocxo_science_totals_t{};
  g_beta_instrument_stats = clocks_instrument_stats_snapshot_t{};
  g_beta_report_instrument_stats = clocks_instrument_stats_snapshot_t{};
  g_beta_monitor_instrument_stats = clocks_instrument_stats_snapshot_t{};
  g_monitor_restore_state = clocks_recovery_restore_state_t{};
  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  g_clocks_report_build_active = false;
  g_clocks_report_build_count = 0U;
  g_clocks_report_busy_reject_count = 0U;
  g_clocks_report_max_duration_cycles = 0U;

  g_recover_reattach_last_ocxo1 =
      clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_reattach_last_ocxo2 =
      clocks_alpha_recover_reattach_snapshot_t{};

  g_clocks_payload_numeric_integrity_failed = false;
  g_clocks_payload_numeric_integrity_fail_count = 0U;
  g_clocks_payload_numeric_last_reject_reason =
      CLOCKS_PAYLOAD_NUMERIC_OK;
  memset(g_clocks_payload_numeric_last_path,
         0,
         sizeof(g_clocks_payload_numeric_last_path));
  memset(g_clocks_payload_numeric_last_key,
         0,
         sizeof(g_clocks_payload_numeric_last_key));
  memset(g_clocks_payload_numeric_last_token_preview,
         0,
         sizeof(g_clocks_payload_numeric_last_token_preview));
  g_clocks_payload_numeric_court_scratch =
      clocks_payload_numeric_court_t{};

  g_clocks_beta_dmamem_initialized = true;
}







static int32_t beta_signed_delta_u32(uint32_t observed, uint32_t expected) {
  return (observed >= expected)
      ? (int32_t)(observed - expected)
      : -(int32_t)(expected - observed);
}







static int32_t beta_dwt32_signed_delta_near(uint32_t from_dwt32,
                                            uint32_t to_dwt32) {
  const uint32_t u = (uint32_t)(to_dwt32 - from_dwt32);
  if (u <= 0x7FFFFFFFUL) {
    return (int32_t)u;
  }
  const uint32_t magnitude = (uint32_t)((~u) + 1U);
  if (magnitude == 0x80000000UL) {
    return (int32_t)(-2147483647 - 1);
  }
  return -(int32_t)magnitude;
}

static int64_t beta_i64_from_u64_saturating(uint64_t value) {
  return (value > (uint64_t)INT64_MAX) ? INT64_MAX : (int64_t)value;
}

static uint64_t beta_abs_i64_to_u64(int64_t value) {
  return (value >= 0) ? (uint64_t)value : (uint64_t)(-value);
}

static uint64_t beta_dwt_cycles_to_gnss_ns_rounded(uint64_t cycles,
                                                   uint32_t cps) {
  if (cps == 0U) return 0ULL;
  return (cycles * CLOCKS_BETA_NS_PER_SECOND + (uint64_t)cps / 2ULL) /
         (uint64_t)cps;
}

static int64_t beta_dwt_cycles_to_gnss_ns_signed(int64_t cycles,
                                                 uint32_t cps) {
  const uint64_t ns = beta_dwt_cycles_to_gnss_ns_rounded(
      beta_abs_i64_to_u64(cycles), cps);
  const int64_t signed_ns = beta_i64_from_u64_saturating(ns);
  return (cycles >= 0) ? signed_ns : -signed_ns;
}

static int64_t beta_signed_delta_u64(uint64_t lhs, uint64_t rhs) {
  return (lhs >= rhs)
      ? beta_i64_from_u64_saturating(lhs - rhs)
      : -beta_i64_from_u64_saturating(rhs - lhs);
}

static int64_t beta_round_double_to_i64(double value) {
  return (value >= 0.0)
      ? (int64_t)(value + 0.5)
      : (int64_t)(value - 0.5);
}

static double beta_dwt_cycles_to_gnss_ns_exact_signed(int64_t cycles,
                                                       uint32_t cps) {
  if (cps == 0U) return 0.0;
  return ((double)cycles * (double)CLOCKS_BETA_NS_PER_SECOND) /
         (double)cps;
}

static delta_residual_reference_t delta_residual_capture_vclock_reference(
    uint32_t public_count,
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f) {
  delta_residual_reference_t r{};
  r.captured = true;
  r.public_count = public_count;

  const uint32_t selected_pps_vclock_interval =
      g_pps_vclock_dwt_cycles_between_edges_valid
          ? (uint32_t)g_pps_vclock_dwt_cycles_between_edges
          : 0U;
  r.gnss_available = g_pps_vclock_dwt_cycles_between_edges_valid;
  r.gnss_interval_cycles = r.gnss_available
      ? selected_pps_vclock_interval
      : 0U;
  r.gnss_valid = r.gnss_available &&
      delta_raw_interval_cycles_plausible(selected_pps_vclock_interval);

  // Alpha's ordinary observed edge-to-edge measurement is the canonical live
  // interval consumed by Beta.  The process_interrupt gate-audit field may be
  // zero during launch acquisition and remains report-only.
  if (vclock_valid &&
      delta_raw_interval_cycles_plausible(vclock_f.dwt_cycles_between_edges)) {
    r.raw_valid = true;
    r.raw_interval_cycles = vclock_f.dwt_cycles_between_edges;
  }
  return r;
}


static void delta_residual_apply_one(clock_science_row_t& row,
                                     const delta_residual_reference_t& ref,
                                     bool clock_valid,
                                     const clocks_alpha_lane_forensics_t& clock_f) {
  row.delta_reference_public_count = ref.public_count;
  row.delta_publication_public_count = row.public_count;

  const bool ref_count_aligned = ref.captured &&
      (ref.public_count == row.public_count ||
       (ref.public_count == 0U && row.public_count == 1U));

  if (ref_count_aligned && ref.gnss_available && clock_valid) {
    row.delta_raw_reference_interval_cycles = ref.gnss_interval_cycles;
    row.delta_raw_clock_interval_cycles = clock_f.dwt_cycles_between_edges;

    row.delta_raw_reference_plausible = delta_raw_interval_cycles_plausible(
        row.delta_raw_reference_interval_cycles);
    row.delta_raw_clock_plausible = delta_raw_interval_cycles_plausible(
        row.delta_raw_clock_interval_cycles);
    row.science_worthy = row.delta_raw_reference_plausible &&
                         row.delta_raw_clock_plausible;

    if (!row.delta_raw_reference_plausible) {
      delta_raw_interval_note_reject(
          row,
          "reference",
          row.delta_raw_reference_interval_cycles,
          row.delta_raw_clock_interval_cycles,
          delta_raw_interval_reject_reason(row.delta_raw_reference_interval_cycles));
    }
    if (!row.delta_raw_clock_plausible) {
      delta_raw_interval_note_reject(
          row,
          "clock",
          row.delta_raw_reference_interval_cycles,
          row.delta_raw_clock_interval_cycles,
          delta_raw_interval_reject_reason(row.delta_raw_clock_interval_cycles));
    }

    row.delta_raw_valid = row.science_worthy;

    if (row.delta_raw_valid) {
      row.delta_raw_residual_cycles =
          (int64_t)row.delta_raw_clock_interval_cycles -
          (int64_t)row.delta_raw_reference_interval_cycles;
      row.delta_raw_fast_residual_cycles = -row.delta_raw_residual_cycles;
      const uint32_t raw_delta_cps = row.delta_raw_reference_interval_cycles
          ? row.delta_raw_reference_interval_cycles
          : row.projection_cps_cycles;
      row.delta_raw_residual_ns_exact = beta_dwt_cycles_to_gnss_ns_exact_signed(
          row.delta_raw_residual_cycles, raw_delta_cps);
      row.delta_raw_fast_residual_ns_exact = -row.delta_raw_residual_ns_exact;
      row.delta_raw_residual_ns =
          beta_round_double_to_i64(row.delta_raw_residual_ns_exact);
      row.delta_raw_fast_residual_ns =
          beta_round_double_to_i64(row.delta_raw_fast_residual_ns_exact);
    }
  }
}


static void delta_residual_attach_comparisons(clock_science_row_t& row) {
  if (row.delta_raw_valid && row.traditional_valid) {
    row.delta_raw_fast_minus_traditional_ns =
        row.delta_raw_fast_residual_ns - row.traditional_fast_residual_ns;
  }
}


static void ocxo_science_attach_totals(
    clock_science_row_t& row,
    const ocxo_science_totals_t& totals) {
  row.total_sample_count = totals.sample_count;
  row.total_clock_interval_ns = totals.clock_interval_total_ns;
  row.total_gnss_interval_ns = totals.gnss_interval_total_ns;
  row.total_clock_interval_ns_exact = totals.clock_interval_total_ns_exact;
  row.total_gnss_interval_ns_exact = totals.gnss_interval_total_ns_exact;
  row.total_valid = totals.gnss_interval_total_ns_exact != 0.0 &&
                    totals.clock_interval_total_ns_exact != 0.0;
  row.total_tau = row.total_valid
      ? (totals.gnss_interval_total_ns_exact /
         totals.clock_interval_total_ns_exact)
      : 1.0;
  row.total_ppb = row.total_valid
      ? campaign_total_ppb_from_tau(row.total_tau)
      : 0.0;
  row.total_fast_residual_ns_exact = row.total_valid
      ? (totals.gnss_interval_total_ns_exact -
         totals.clock_interval_total_ns_exact)
      : 0.0;
  row.total_fast_residual_ns = row.total_valid
      ? beta_round_double_to_i64(row.total_fast_residual_ns_exact)
      : 0LL;

  row.traditional_total_sample_count = totals.traditional_sample_count;
  row.traditional_total_clock_interval_ns =
      totals.traditional_clock_interval_total_ns;
  row.traditional_total_gnss_interval_ns =
      totals.traditional_gnss_interval_total_ns;
  row.traditional_total_clock_interval_ns_exact =
      totals.traditional_clock_interval_total_ns_exact;
  row.traditional_total_gnss_interval_ns_exact =
      totals.traditional_gnss_interval_total_ns_exact;
  row.traditional_total_valid =
      totals.traditional_gnss_interval_total_ns_exact != 0.0 &&
      totals.traditional_clock_interval_total_ns_exact != 0.0;
  row.traditional_total_tau = row.traditional_total_valid
      ? (totals.traditional_clock_interval_total_ns_exact /
         totals.traditional_gnss_interval_total_ns_exact)
      : 1.0;
  row.traditional_total_ppb = row.traditional_total_valid
      ? campaign_total_ppb_from_tau(row.traditional_total_tau)
      : 0.0;
  row.traditional_total_fast_residual_ns_exact = row.traditional_total_valid
      ? (totals.traditional_clock_interval_total_ns_exact -
         totals.traditional_gnss_interval_total_ns_exact)
      : 0.0;
  row.traditional_total_fast_residual_ns = row.traditional_total_valid
      ? beta_round_double_to_i64(row.traditional_total_fast_residual_ns_exact)
      : 0LL;
}

static void clock_science_apply_campaign_public_ratio(clock_science_row_t& row,
                                                      uint64_t public_gnss_ns,
                                                      uint64_t public_clock_ns,
                                                      uint32_t public_count) {
  row.total_sample_count = public_count;
  row.total_clock_interval_ns = public_clock_ns;
  row.total_gnss_interval_ns = public_gnss_ns;
  row.total_clock_interval_ns_exact = (double)public_clock_ns;
  row.total_gnss_interval_ns_exact = (double)public_gnss_ns;
  row.total_valid = public_gnss_ns != 0ULL && public_clock_ns != 0ULL;

  row.total_tau = row.total_valid
      ? campaign_total_tau_from_ratio(public_gnss_ns, public_clock_ns)
      : 1.0;
  row.total_ppb = row.total_valid
      ? campaign_total_ppb_from_tau(row.total_tau)
      : 0.0;
  row.total_fast_residual_ns_exact = row.total_valid
      ? ((double)public_clock_ns - (double)public_gnss_ns)
      : 0.0;
  row.total_fast_residual_ns = row.total_valid
      ? beta_signed_delta_u64(public_clock_ns, public_gnss_ns)
      : 0LL;
}


static void clock_science_apply_alpha_tau(clock_science_row_t& row,
                                          bool tau_ok,
                                          const clocks_alpha_tau_snapshot_t& tau) {
  row.alpha_tau_valid = tau_ok && tau.valid;
  row.alpha_tau_sample_count = tau.sample_count;
  row.alpha_tau_interval_count = tau.interval_count;
  row.alpha_tau_last_pps_sequence = tau.last_pps_sequence;
  row.alpha_tau_last_interval_pps_sequence = tau.last_interval_pps_sequence;
  row.alpha_tau = row.alpha_tau_valid ? tau.tau : 1.0;
  row.alpha_tau_ppb = row.alpha_tau_valid ? tau.ppb : 0.0;
  row.alpha_tau_stderr_ppb = row.alpha_tau_valid ? tau.stderr_ppb : 0.0;
  row.alpha_tau_interval_mean_ppb = row.alpha_tau_valid
      ? tau.interval_mean_ppb
      : 0.0;
  row.alpha_tau_interval_stderr_ppb = row.alpha_tau_valid
      ? tau.interval_stderr_ppb
      : 0.0;
  row.alpha_tau_intercept_ns = row.alpha_tau_valid ? tau.intercept_ns : 0;

  if (!row.alpha_tau_valid) return;

  // AlphaTau is preserved as side evidence, but it no longer overwrites the
  // panel-facing campaign total.  The operator experience should be continuous
  // across RECOVER; therefore total_tau/total_ppb remain the public clockface
  // ratio after Beta's recovery presentation transform.
  if (row.valid) {
    row.alpha_tau_detrended_fast_residual_ns =
        row.fast_residual_ns - beta_round_double_to_i64(tau.ppb);
  }
}

static void clock_science_apply_counterledger_row(
    clock_science_row_t& row,
    const clocks_alpha_ocxo_counterledger_snapshot_t& ledger,
    uint64_t public_gnss_ns) {
  (void)public_gnss_ns;
  if (!clocks_ocxo_counterledger_mode_enabled()) return;

  const bool refined_interval_available =
      ledger.valid && ledger.refined_interval_valid &&
      ledger.refined_interval_ns != 0ULL;

  if (CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL &&
      !refined_interval_available) {
    g_phaseledger_science_last_missing_public_count = row.public_count;
    if (ledger.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO1)) {
    } else if (ledger.clock_id ==
               (uint32_t)((uint8_t)time_clock_id_t::OCXO2)) {
    }
  }

  // Deliberately do not rewrite row.{valid,clock_interval,fast_residual,tau,ppb}.
  // clock_science_build_ocxo() already authored those fields from same-row
  // observed Delta Cycles.  The full CounterLedger/PhaseLedger candidate remains
  // published beside it under ocxoN.counterledger, including refined interval,
  // residual, phase suffix, wrap transcript, and long-block statistics.
}

static bool clocks_beta_counterledger_completed_row_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& ledger,
    uint32_t completed_pps_sequence) {
  return completed_pps_sequence != 0U &&
         ledger.valid &&
         ledger.initialized &&
         ledger.pps_sequence == completed_pps_sequence &&
         ledger.last_capture_available &&
         ledger.last_capture_valid &&
         ledger.last_capture_lane_valid &&
         ledger.last_capture_all_lanes_valid &&
         ledger.last_capture_sequence_match &&
         ledger.last_capture_sequence == completed_pps_sequence &&
         ledger.phase_valid &&
         !ledger.phase_pending &&
         ledger.phase_pps_sequence == completed_pps_sequence &&
         ledger.phase_lag_pps == 0U &&
         ledger.refined_valid &&
         ledger.refined_ns != 0ULL;
}

static bool clocks_beta_counterledger_clockface_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& ledger,
    uint32_t completed_pps_sequence,
    uint64_t public_ocxo_ns) {
  return public_ocxo_ns != 0ULL &&
         clocks_beta_counterledger_completed_row_ready(
             ledger, completed_pps_sequence);
}

static bool clocks_beta_completed_ocxo_forensics_ready(
    bool forensics_valid,
    const clocks_alpha_lane_forensics_t& forensics,
    const clocks_alpha_ocxo_counterledger_snapshot_t& ledger,
    uint32_t completed_pps_sequence) {
  return forensics_valid &&
         clocks_beta_counterledger_completed_row_ready(
             ledger, completed_pps_sequence) &&
         forensics.last_event_dwt == ledger.phase_next_ocxo_dwt_at_edge &&
         forensics.dwt_cycles_between_edges ==
             ledger.phase_ocxo_interval_cycles &&
         forensics.counter32_delta_since_previous_event ==
             (uint32_t)VCLOCK_COUNTS_PER_SECOND;
}

static bool clocks_beta_ocxo_science_custody_ok(
    const clock_science_row_t& row,
    uint64_t public_ocxo_ns) {
  if (public_ocxo_ns == 0ULL) return true;
  return row.science_worthy && row.antecedents_complete;
}

static uint32_t clocks_beta_public_ocxo_science_invalid_mask(
    uint32_t public_count,
    uint64_t public_ocxo1_ns,
    uint64_t public_ocxo2_ns,
    const clock_science_row_t& ocxo1_science,
    const clock_science_row_t& ocxo2_science) {
  if (public_count <= 2U) return 0U;
  if (g_recover_reattach_degraded_active) return 0U;
  if (g_science_residual_quarantine_last_public_count == public_count) return 0U;

  uint32_t mask = 0U;
  if (!clocks_beta_ocxo_science_custody_ok(
          ocxo1_science, public_ocxo1_ns)) {
    mask |= CAMPAIGN_RECORD_INVALID_OCXO1_CUSTODY;
  }
  if (!clocks_beta_ocxo_science_custody_ok(
          ocxo2_science, public_ocxo2_ns)) {
    mask |= CAMPAIGN_RECORD_INVALID_OCXO2_CUSTODY;
  }
  return mask;
}

static void clock_science_build_ocxo(
    clock_science_row_t& row,
    time_clock_id_t clock,
    uint32_t public_count,
    uint64_t public_gnss_ns,
    const delta_residual_reference_t& same_row_reference,
    bool clock_valid,
    const clocks_alpha_lane_forensics_t& clock_f,
    ocxo_science_totals_t& totals) {
  row = clock_science_row_t{};
  row.clock_id = (uint32_t)((uint8_t)clock);
  row.public_count = public_count;
  row.pps_vclock_dwt_at_edge = (uint32_t)g_dwt_at_pps_vclock;
  row.pps_vclock_gnss_ns_at_edge = public_gnss_ns;
  row.projection_cps_cycles = (uint32_t)g_dwt_cycles_between_pps_vclock;

  row.clock_published_dwt_at_edge = clock_valid ? clock_f.dwt_used_at_event : 0U;
  row.clock_raw_dwt_at_edge = clock_valid ? clock_f.dwt_original_at_event : 0U;
  row.clock_observed_interval_cycles =
      clock_valid ? clock_f.dwt_cycles_between_edges : 0U;
  row.clock_effective_interval_cycles =
      clock_valid ? clock_f.dwt_interval_effective_cycles : 0U;

  delta_residual_apply_one(row, same_row_reference, clock_valid, clock_f);

  const bool traditional_antecedents_valid =
      clock_valid &&
      row.clock_raw_dwt_at_edge != 0U &&
      row.clock_observed_interval_cycles != 0U &&
      row.projection_cps_cycles != 0U;

  if (traditional_antecedents_valid) {
    const int32_t current_delta_cycles = beta_dwt32_signed_delta_near(
        row.pps_vclock_dwt_at_edge, row.clock_raw_dwt_at_edge);
    const int64_t current_delta_ns = beta_dwt_cycles_to_gnss_ns_signed(
        (int64_t)current_delta_cycles, row.projection_cps_cycles);

    row.current_edge_gnss_ns =
        beta_i64_from_u64_saturating(public_gnss_ns) + current_delta_ns;
    row.current_edge_gnss_ns_exact =
        (double)public_gnss_ns +
        ((double)current_delta_cycles * (double)CLOCKS_BETA_NS_PER_SECOND) /
            (double)row.projection_cps_cycles;

    row.traditional_valid = true;
    row.traditional_gnss_interval_ns_exact =
        ((double)row.clock_observed_interval_cycles *
         (double)CLOCKS_BETA_NS_PER_SECOND) /
        (double)row.projection_cps_cycles;
    row.traditional_gnss_interval_ns = beta_dwt_cycles_to_gnss_ns_rounded(
        (uint64_t)row.clock_observed_interval_cycles,
        row.projection_cps_cycles);
    row.prior_edge_gnss_ns =
        row.current_edge_gnss_ns -
        beta_i64_from_u64_saturating(row.traditional_gnss_interval_ns);
    row.prior_edge_gnss_ns_exact =
        row.current_edge_gnss_ns_exact -
        row.traditional_gnss_interval_ns_exact;

    row.traditional_clock_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
    row.traditional_clock_interval_ns_exact =
        (double)CLOCKS_BETA_NS_PER_SECOND;
    row.traditional_fast_residual_ns_exact =
        row.traditional_clock_interval_ns_exact -
        row.traditional_gnss_interval_ns_exact;
    row.traditional_fast_residual_ns =
        beta_round_double_to_i64(row.traditional_fast_residual_ns_exact);
    row.traditional_tau_1s = (double)row.projection_cps_cycles /
                             (double)row.clock_observed_interval_cycles;
    row.traditional_ppb_1s =
        campaign_total_ppb_from_tau(row.traditional_tau_1s);

    totals.traditional_sample_count++;
    totals.traditional_clock_interval_total_ns +=
        row.traditional_clock_interval_ns;
    totals.traditional_gnss_interval_total_ns +=
        row.traditional_gnss_interval_ns;
    totals.traditional_clock_interval_total_ns_exact +=
        row.traditional_clock_interval_ns_exact;
    totals.traditional_gnss_interval_total_ns_exact +=
        row.traditional_gnss_interval_ns_exact;
  }

  // Delta Cycles is canonical: compare the OCXO observed interval against the
  // exact same-row selected PPS/VCLOCK observed interval.
  row.valid = row.delta_raw_valid;
  // Science eligibility and numeric validity are one contract. Rejected
  // intervals remain visible in raw_cycles but cannot author science totals.
  row.antecedents_complete = row.science_worthy &&
                             row.delta_raw_reference_interval_cycles != 0U &&
                             row.delta_raw_clock_interval_cycles != 0U;

  delta_residual_attach_comparisons(row);

  if (!row.valid) {
    ocxo_science_attach_totals(row, totals);
    return;
  }

  row.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
  row.gnss_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
  row.fast_residual_ns_exact = row.delta_raw_fast_residual_ns_exact;
  row.fast_residual_ns = row.delta_raw_fast_residual_ns;
  row.clock_interval_ns_exact = row.gnss_interval_ns_exact -
                                row.fast_residual_ns_exact;
  row.clock_interval_ns = row.clock_interval_ns_exact > 0.0
      ? (uint64_t)beta_round_double_to_i64(row.clock_interval_ns_exact)
      : 0ULL;
  row.tau_1s = (row.clock_interval_ns_exact > 0.0)
      ? (row.gnss_interval_ns_exact / row.clock_interval_ns_exact)
      : 1.0;
  row.ppb_1s = campaign_total_ppb_from_tau(row.tau_1s);

  totals.sample_count++;
  totals.clock_interval_total_ns += row.clock_interval_ns;
  totals.gnss_interval_total_ns += row.gnss_interval_ns;
  totals.clock_interval_total_ns_exact += row.clock_interval_ns_exact;
  totals.gnss_interval_total_ns_exact += row.gnss_interval_ns_exact;
  ocxo_science_attach_totals(row, totals);
}


static uint32_t beta_abs_i32_to_u32(int32_t value) {
  return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}


static bool beta_abs_i32_within_gate(int32_t value, uint32_t gate) {
  return beta_abs_i32_to_u32(value) <= gate;
}


static void campaign_start_prologue_set_reason(const char* reason) {
  safeCopy(g_start_prologue_last_reason,
           sizeof(g_start_prologue_last_reason),
           reason ? reason : "prologue");
}


static const char* campaign_start_candidate_first_problem(uint32_t mask) {
  if (mask & CLOCKS_START_CANDIDATE_MISSING_VCLOCK_SNAPSHOT) {
    return "vclock_snapshot";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_OCXO1_SNAPSHOT) {
    return "ocxo1_snapshot";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_OCXO2_SNAPSHOT) {
    return "ocxo2_snapshot";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_VCLOCK_OBSERVED) {
    return g_start_candidate_last_vclock_measurement_interval == 0U
        ? "vclock_start_interval_missing"
        : "vclock_start_interval_implausible";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_OCXO1_OBSERVED) {
    return g_start_candidate_last_ocxo1_measurement_interval == 0U
        ? "ocxo1_start_interval_missing"
        : "ocxo1_start_interval_implausible";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_OCXO2_OBSERVED) {
    return g_start_candidate_last_ocxo2_measurement_interval == 0U
        ? "ocxo2_start_interval_missing"
        : "ocxo2_start_interval_implausible";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_VALID) {
    return "selected_reference_invalid";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_INTERVAL) {
    return g_start_candidate_last_selected_reference_interval == 0U
        ? "selected_reference_interval_missing"
        : "selected_reference_interval_implausible";
  }
  if (mask & CLOCKS_START_CANDIDATE_MISSING_EFFECTIVE_CPS) {
    return g_start_candidate_last_effective_cps == 0U
        ? "effective_cps_missing"
        : "effective_cps_implausible";
  }
  return "ready";
}


static bool campaign_start_prologue_candidate_ready(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {
  // The caller has already formed the current Beta candidate tuple and proved
  // that VCLOCK matches the selected PPS/VCLOCK identity. Validate that exact
  // tuple here; do not re-read Alpha stores from the earlier warmup gate and
  // accidentally mix snapshots from different candidate moments.

  g_start_candidate_last_vclock_snapshot_valid = vclock_valid;
  g_start_candidate_last_ocxo1_snapshot_valid = ocxo1_valid;
  g_start_candidate_last_ocxo2_snapshot_valid = ocxo2_valid;
  g_start_candidate_last_vclock_observed_interval =
      vclock_valid ? vclock_f.dwt_interval_observed_cycles : 0U;
  g_start_candidate_last_ocxo1_observed_interval =
      ocxo1_valid ? ocxo1_f.dwt_interval_observed_cycles : 0U;
  g_start_candidate_last_ocxo2_observed_interval =
      ocxo2_valid ? ocxo2_f.dwt_interval_observed_cycles : 0U;
  g_start_candidate_last_vclock_measurement_interval =
      vclock_valid ? vclock_f.dwt_cycles_between_edges : 0U;
  g_start_candidate_last_ocxo1_measurement_interval =
      ocxo1_valid ? ocxo1_f.dwt_cycles_between_edges : 0U;
  g_start_candidate_last_ocxo2_measurement_interval =
      ocxo2_valid ? ocxo2_f.dwt_cycles_between_edges : 0U;
  g_start_candidate_last_vclock_update_count =
      vclock_valid ? vclock_f.update_count : 0U;
  g_start_candidate_last_ocxo1_update_count =
      ocxo1_valid ? ocxo1_f.update_count : 0U;
  g_start_candidate_last_ocxo2_update_count =
      ocxo2_valid ? ocxo2_f.update_count : 0U;
  g_start_candidate_last_selected_reference_valid =
      g_pps_vclock_dwt_cycles_between_edges_valid;
  g_start_candidate_last_selected_reference_interval =
      (uint32_t)g_pps_vclock_dwt_cycles_between_edges;
  g_start_candidate_last_effective_cps =
      (uint32_t)g_dwt_cycles_between_pps_vclock;

  uint32_t missing = 0U;
  if (!vclock_valid) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_VCLOCK_SNAPSHOT;
  }
  if (!ocxo1_valid) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_OCXO1_SNAPSHOT;
  }
  if (!ocxo2_valid) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_OCXO2_SNAPSHOT;
  }
  if (!delta_raw_interval_cycles_plausible(
          g_start_candidate_last_vclock_measurement_interval)) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_VCLOCK_OBSERVED;
  }
  if (!delta_raw_interval_cycles_plausible(
          g_start_candidate_last_ocxo1_measurement_interval)) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_OCXO1_OBSERVED;
  }
  if (!delta_raw_interval_cycles_plausible(
          g_start_candidate_last_ocxo2_measurement_interval)) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_OCXO2_OBSERVED;
  }
  if (!g_start_candidate_last_selected_reference_valid) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_VALID;
  }
  if (!delta_raw_interval_cycles_plausible(
          g_start_candidate_last_selected_reference_interval)) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_SELECTED_REFERENCE_INTERVAL;
  }
  if (!delta_raw_interval_cycles_plausible(
          g_start_candidate_last_effective_cps)) {
    missing |= CLOCKS_START_CANDIDATE_MISSING_EFFECTIVE_CPS;
  }

  g_start_candidate_last_missing_mask = missing;
  g_start_candidate_last_ready = (missing == 0U);
  safeCopy(g_start_candidate_last_first_problem,
           sizeof(g_start_candidate_last_first_problem),
           campaign_start_candidate_first_problem(missing));

  if (missing != 0U) {
    campaign_start_prologue_set_reason("waiting_for_lawful_alpha_intervals");
    return false;
  }

  return true;
}


static bool campaign_start_prologue_consume_private_candidate(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f,
    const char* reason) {
  if (!vclock_valid || !ocxo1_valid || !ocxo2_valid ||
      !delta_raw_interval_cycles_plausible(
          vclock_f.dwt_cycles_between_edges) ||
      !delta_raw_interval_cycles_plausible(
          ocxo1_f.dwt_cycles_between_edges) ||
      !delta_raw_interval_cycles_plausible(
          ocxo2_f.dwt_cycles_between_edges) ||
      !g_pps_vclock_dwt_cycles_between_edges_valid ||
      !delta_raw_interval_cycles_plausible(
          (uint32_t)g_pps_vclock_dwt_cycles_between_edges) ||
      !delta_raw_interval_cycles_plausible(
          (uint32_t)g_dwt_cycles_between_pps_vclock)) {
    campaign_start_prologue_set_reason(
        "private_candidate_missing_or_implausible_alpha_interval");
    return false;
  }

  // This candidate becomes the private PPS0 bookend.  Reset public presentation
  // offsets to this edge; the next qualified candidate becomes public PPS1.
  campaign_public_offsets_reset_to_current();

  const delta_residual_reference_t ref =
      delta_residual_capture_vclock_reference(0U, vclock_valid, vclock_f);

  g_start_prologue_pps0_pps_obs =
      ref.gnss_valid ? ref.gnss_interval_cycles : 0U;
  g_start_prologue_pps0_v_obs =
      vclock_valid ? vclock_f.dwt_cycles_between_edges : 0U;
  g_start_prologue_pps0_o1_obs =
      ocxo1_valid ? ocxo1_f.dwt_cycles_between_edges : 0U;
  g_start_prologue_pps0_o2_obs =
      ocxo2_valid ? ocxo2_f.dwt_cycles_between_edges : 0U;
  g_start_prologue_pps0_interval_valid =
      ref.gnss_valid &&
      delta_raw_interval_cycles_plausible(g_start_prologue_pps0_pps_obs) &&
      delta_raw_interval_cycles_plausible(g_start_prologue_pps0_v_obs) &&
      delta_raw_interval_cycles_plausible(g_start_prologue_pps0_o1_obs) &&
      delta_raw_interval_cycles_plausible(g_start_prologue_pps0_o2_obs);

  g_delta_previous_vclock_reference = ref;
  g_start_prologue_seeded = true;
  g_start_prologue_reference_ready = ref.gnss_valid;
  g_start_prologue_private_candidate_count++;
  g_start_prologue_last_private_count = g_start_prologue_private_candidate_count;
  campaign_start_prologue_set_reason(reason ? reason : "private_pps0_consumed");
  return true;
}



static bool campaign_start_prologue_science_row_ready(
    const clock_science_row_t& row) {
  return row.valid &&
         row.delta_raw_reference_interval_cycles != 0U &&
         row.delta_raw_clock_interval_cycles != 0U;
}


static bool campaign_start_prologue_reference_ready(
    const delta_residual_reference_t& ref) {
  return ref.captured && ref.gnss_valid && ref.gnss_interval_cycles != 0U;
}


static bool campaign_start_prologue_interval_continuity_ok(uint32_t current,
                                                     uint32_t previous) {
  if (!delta_raw_interval_cycles_plausible(current) ||
      !delta_raw_interval_cycles_plausible(previous)) {
    return false;
  }
  return beta_abs_i32_within_gate(
      beta_signed_delta_u32(current, previous),
      CLOCKS_START_PROLOGUE_INTERVAL_CONTINUITY_GATE_CYCLES);
}


static bool campaign_start_prologue_private_pps0_continuity_ok(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {

  const uint32_t current_reference_interval =
      g_pps_vclock_dwt_cycles_between_edges_valid
          ? (uint32_t)g_pps_vclock_dwt_cycles_between_edges
          : 0U;
  const uint32_t current_vclock_interval =
      vclock_valid ? vclock_f.dwt_cycles_between_edges : 0U;
  const uint32_t current_ocxo1_interval =
      ocxo1_valid ? ocxo1_f.dwt_cycles_between_edges : 0U;
  const uint32_t current_ocxo2_interval =
      ocxo2_valid ? ocxo2_f.dwt_cycles_between_edges : 0U;

  const auto reject = [](const char* reason) -> bool {
    campaign_start_prologue_set_reason(reason);
    return false;
  };

  if (!g_start_prologue_pps0_interval_valid) {
    return reject("private_pps0_interval_missing");
  }

  if (!campaign_start_prologue_interval_continuity_ok(
          current_reference_interval,
          g_start_prologue_pps0_pps_obs)) {
    return reject("private_pps0_reference_interval_unsettled");
  }

  if (!vclock_valid ||
      !campaign_start_prologue_interval_continuity_ok(
          current_vclock_interval,
          g_start_prologue_pps0_v_obs)) {
    return reject("private_pps0_vclock_interval_unsettled");
  }

  if (!ocxo1_valid ||
      !campaign_start_prologue_interval_continuity_ok(
          current_ocxo1_interval,
          g_start_prologue_pps0_o1_obs)) {
    return reject("private_pps0_ocxo1_interval_unsettled");
  }

  if (!ocxo2_valid ||
      !campaign_start_prologue_interval_continuity_ok(
          current_ocxo2_interval,
          g_start_prologue_pps0_o2_obs)) {
    return reject("private_pps0_ocxo2_interval_unsettled");
  }

  return true;
}

static bool campaign_start_prologue_probe_public_pps1(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {
  if (!g_start_prologue_seeded || !g_start_prologue_reference_ready) {
    campaign_start_prologue_set_reason("waiting_for_private_pps0_reference");
    return false;
  }

  if (g_dwt_cycles_between_pps_vclock == 0U) {
    campaign_start_prologue_set_reason("waiting_for_gnss_interval");
    return false;
  }

  if (!campaign_start_prologue_reference_ready(
          g_delta_previous_vclock_reference)) {
    campaign_start_prologue_set_reason("private_gnss_reference_missing");
    return false;
  }

  // The private PPS0 is a continuity witness.  Candidate PPS1 uses the current
  // same-row selected PPS/VCLOCK interval, matching the public science path.
  if (!campaign_start_prologue_private_pps0_continuity_ok(
          vclock_valid, vclock_f, ocxo1_valid, ocxo1_f,
          ocxo2_valid, ocxo2_f)) {
    return false;
  }

  const delta_residual_reference_t candidate_reference =
      delta_residual_capture_vclock_reference(
          1U, vclock_valid, vclock_f);

  g_beta_probe_science_totals_o1 = ocxo_science_totals_t{};
  g_beta_probe_science_totals_o2 = ocxo_science_totals_t{};

  clock_science_row_t& o1 = g_beta_probe_ocxo1_science_row;
  clock_science_row_t& o2 = g_beta_probe_ocxo2_science_row;
  clock_science_build_ocxo(
      o1,
      time_clock_id_t::OCXO1,
      1U,
      CLOCKS_BETA_NS_PER_SECOND,
      candidate_reference,
      ocxo1_valid,
      ocxo1_f,
      g_beta_probe_science_totals_o1);
  clock_science_build_ocxo(
      o2,
      time_clock_id_t::OCXO2,
      1U,
      CLOCKS_BETA_NS_PER_SECOND,
      candidate_reference,
      ocxo2_valid,
      ocxo2_f,
      g_beta_probe_science_totals_o2);

  if (!campaign_start_prologue_reference_ready(candidate_reference)) {
    campaign_start_prologue_set_reason("candidate_gnss_reference_missing");
    return false;
  }
  if (!o1.valid || !o2.valid) {
    campaign_start_prologue_set_reason("waiting_for_delta_observed_valid");
    return false;
  }
  if (!campaign_start_prologue_science_row_ready(o1)) {
    campaign_start_prologue_set_reason("ocxo1_observed_interval_missing");
    return false;
  }
  if (!campaign_start_prologue_science_row_ready(o2)) {
    campaign_start_prologue_set_reason("ocxo2_observed_interval_missing");
    return false;
  }

  return true;
}



static void campaign_start_prologue_abort_launch(const char* reason) {
  campaign_start_prologue_set_reason(reason ? reason : "start_handoff_timeout");
  g_start_handoff_timeout_count++;
  interrupt_dwt_publication_launch_acquisition_end();
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  campaign_state = clocks_campaign_state_t::STOPPED;
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  flash_cut_clear_pending();

  // A failed START closes only the requested recording namespace. Servo mode,
  // controller memory, and queued actuator intent belong to the always-on
  // instrument and must survive the campaign admission failure.
  clocks_watchdog_clear_surrender_for_new_lifecycle();
}

static bool campaign_start_prologue_should_hold(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {
  if (!campaign_start_handoff_ready()) {
    if (g_start_handoff_launch_wait_count != UINT32_MAX) {
      g_start_handoff_launch_wait_count++;
    }
    if (g_start_handoff_launch_wait_count >=
        CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES) {
      campaign_start_prologue_abort_launch(
          clocks_ocxo_counterledger_mode_enabled()
              ? "phaseledger_start_handoff_timeout"
              : "start_handoff_timeout");
      return true;
    }
    campaign_start_prologue_set_reason(
        clocks_ocxo_counterledger_mode_enabled()
            ? g_start_phaseledger_last_reason
            : "waiting_for_start_handoff");
    return true;
  }

  // A ready handoff is not enough if the current observed DWT bookends have
  // not arrived yet.  Keep the same bounded launch wait active through this
  // final evidence-acquisition step so START cannot become silently infinite.
  if (!campaign_start_prologue_candidate_ready(
          vclock_valid, vclock_f,
          ocxo1_valid, ocxo1_f,
          ocxo2_valid, ocxo2_f)) {
    if (g_start_handoff_launch_wait_count != UINT32_MAX) {
      g_start_handoff_launch_wait_count++;
    }
    if (g_start_handoff_launch_wait_count >=
        CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES) {
      campaign_start_prologue_abort_launch(
          "start_observed_bookend_timeout");
    }
    return true;
  }
  g_start_handoff_launch_wait_count = 0;

  if (campaign_start_prologue_probe_public_pps1(
          vclock_valid, vclock_f,
          ocxo1_valid, ocxo1_f,
          ocxo2_valid, ocxo2_f)) {
    campaign_start_prologue_set_reason(
        clocks_ocxo_counterledger_mode_enabled()
            ? "phaseledger_release_public_pps1"
            : "release_public_pps1");
    return false;
  }

  if (g_start_prologue_private_candidate_count >=
      CLOCKS_START_PROLOGUE_MAX_PRIVATE_CANDIDATES) {
    g_start_prologue_private_limit_count++;
    campaign_start_prologue_abort_launch(
        "start_private_bookend_timeout");
    return true;
  }

  (void)campaign_start_prologue_consume_private_candidate(
      vclock_valid,
      vclock_f,
      ocxo1_valid,
      ocxo1_f,
      ocxo2_valid,
      ocxo2_f,
      g_start_prologue_reference_ready
          ? (clocks_ocxo_counterledger_mode_enabled()
                 ? "phaseledger_private_pps0_refreshed_after_probe_reject"
                 : "private_pps0_refreshed_after_probe_reject")
          : (clocks_ocxo_counterledger_mode_enabled()
                 ? "phaseledger_private_pps0_seeded"
                 : "private_pps0_seeded"));
  return true;
}


static FLASHMEM void clocks_monitor_copy_text(char* dst,
                                     size_t capacity,
                                     const char* src) {
  if (!dst || capacity == 0U) return;
  safeCopy(dst, capacity, src ? src : "");
}

static FLASHMEM void clocks_monitor_science_snapshot_from_row(
    clocks_monitor_science_snapshot_t& out,
    const clock_science_row_t& row) {
  out = clocks_monitor_science_snapshot_t{};
  out.valid = row.valid;
  out.science_worthy = row.science_worthy;
  out.antecedents_complete = row.antecedents_complete;
  out.gnss_interval_ns = row.gnss_interval_ns;
  out.clock_interval_ns = row.clock_interval_ns;
  out.fast_residual_ns = row.fast_residual_ns;
  out.fast_residual_ns_exact = row.fast_residual_ns_exact;
  out.delta_raw_valid = row.delta_raw_valid;
  out.delta_raw_reference_interval_cycles =
      row.delta_raw_reference_interval_cycles;
  out.delta_raw_clock_interval_cycles = row.delta_raw_clock_interval_cycles;
  out.delta_raw_fast_residual_cycles = row.delta_raw_fast_residual_cycles;
}

static FLASHMEM void clocks_monitor_raw_cycles_lane_snapshot(
    clocks_monitor_raw_cycles_lane_t& out,
    const clocks_static_prediction_snapshot_t& sample,
    const interrupt_delay_forensics_t& delay) {
  out = clocks_monitor_raw_cycles_lane_t{};
  out.valid = sample.valid;
  out.completed_interval_count = sample.completed_interval_count;
  out.observed_cycles = sample.actual_cycles;
  out.previous_observed_cycles = sample.static_prediction_cycles;
  out.residual_cycles = sample.static_residual_cycles;

  const char* delay_status = interrupt_delay_verdict_str(delay.verdict);
  clocks_monitor_copy_text(out.delay_status,
                           sizeof(out.delay_status),
                           delay_status);
  out.delay_detail_present = true;

  out.residual_delay_valid = delay.residual_delay_valid && sample.valid;
  const int64_t residual_after_delay_64 = out.residual_delay_valid
      ? (int64_t)sample.static_residual_cycles -
            (int64_t)delay.residual_delay_cycles
      : 0LL;
  const int32_t residual_after_delay =
      residual_after_delay_64 > INT32_MAX
          ? INT32_MAX
          : (residual_after_delay_64 < INT32_MIN
                 ? INT32_MIN
                 : (int32_t)residual_after_delay_64);
  const uint32_t residual_after_delay_abs = residual_after_delay < 0
      ? (uint32_t)(-(int64_t)residual_after_delay)
      : (uint32_t)residual_after_delay;

  clocks_monitor_copy_text(
      out.delay_by,
      sizeof(out.delay_by),
      (delay.valid && delay.delayed)
          ? interrupt_delay_cause_str(delay.delayed_by)
          : "NONE");
  out.residual_delay_cycles = out.residual_delay_valid
      ? delay.residual_delay_cycles
      : 0;
  clocks_monitor_copy_text(
      out.residual_delay_by,
      sizeof(out.residual_delay_by),
      out.residual_delay_valid
          ? interrupt_delay_cause_str(delay.residual_delayed_by)
          : "UNKNOWN");
  out.delay_explains_residual = out.residual_delay_valid &&
      delay.residual_delay_cycles != 0 &&
      residual_after_delay_abs <=
          CAMPAIGN_RECORD_ISR_DELAY_EXPLANATION_GATE_CYCLES;
}

static FLASHMEM void clocks_monitor_raw_cycles_snapshot(
    clocks_monitor_raw_cycles_snapshot_t& out,
    const clocks_static_prediction_snapshot_t& pps,
    const clocks_static_prediction_snapshot_t& vclock,
    const clocks_static_prediction_snapshot_t& ocxo1,
    const clocks_static_prediction_snapshot_t& ocxo2,
    const interrupt_delay_forensics_t& pps_delay,
    const interrupt_delay_forensics_t& vclock_delay,
    const interrupt_delay_forensics_t& ocxo1_delay,
    const interrupt_delay_forensics_t& ocxo2_delay) {
  memset(&out, 0, sizeof(out));
  clocks_monitor_raw_cycles_lane_snapshot(out.pps, pps, pps_delay);
  clocks_monitor_raw_cycles_lane_snapshot(out.vclock, vclock, vclock_delay);
  clocks_monitor_raw_cycles_lane_snapshot(out.ocxo1, ocxo1, ocxo1_delay);
  clocks_monitor_raw_cycles_lane_snapshot(out.ocxo2, ocxo2, ocxo2_delay);
}


// Servo source doctrine. MEAN and NOW always consume the same-row observed
// Delta Cycles residual that feeds the OCXO Welfords and public pps_residual.
// CounterLedger/PhaseLedger remains a parallel clockface/forensic surface.
// TOTAL owns a separate cascaded controller: the long-baseline public TAU/PPB
// is the position objective, while a short admitted-residual window is the live
// rate witness used to accelerate, coast, or brake before the total crosses zero.
static constexpr uint32_t SERVO_INPUT_SOURCE_NONE = 0;
static constexpr uint32_t SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD = 1;
static constexpr uint32_t SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU = 2;
static constexpr uint32_t SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL = 3;

static constexpr uint32_t SERVO_TOTAL_BASIS_NONE = 0;
static constexpr uint32_t SERVO_TOTAL_BASIS_INSTRUMENT = 2;


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
  uint32_t total_basis = SERVO_TOTAL_BASIS_NONE;
  uint64_t total_population_seconds = 0;

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


// ============================================================================
// Operator-gated OCXO DAC realization report
// ============================================================================
//
// Enabled/boot default: one-second fractional dither, with TimePop timed
// callbacks only latching desired phase/code and foreground ALAP service doing
// any hardware writes.  Static rounded authority remains available by command.

static constexpr const char* OCXO_DAC_REFERENCE_MODE = "INTERNAL_VREF_2X";

static const char* ocxo_dac_realization_mode_runtime(void) {
  return clocks_ocxo_dac_dither_operator_enabled()
      ? "ONE_SECOND_FRACTIONAL_DITHER"
      : "STATIC_ROUNDED";
}

static bool ocxo_dac_fractional_stream_possible_runtime(void) {
  return clocks_ocxo_dac_dither_operator_enabled();
}

static bool ocxo_dac_static_rounded_only_runtime(void) {
  return !clocks_ocxo_dac_dither_operator_enabled();
}

static const char* servo_hold_reason_name(uint32_t reason);


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

// TOTAL is a cascaded position/rate controller.
//
//   position: long-baseline public TOTAL PPB (the quantity we want at zero)
//   rate:     mean of a short contiguous window of admitted one-second Delta
//             residuals (the oscillator's present direction and momentum)
//
// The position population determines the rate required to retire the remaining
// total error over a bounded horizon.  The short-window rate controller then
// moves the DAC only far enough to make the oscillator approach that requested
// rate.  A projected zero crossing forces the requested rate to zero so braking
// begins before the long-memory TOTAL display reaches zero.
static constexpr uint32_t SERVO_TOTAL_RECENT_WINDOW_SAMPLES = 30U;
static constexpr uint32_t SERVO_TOTAL_COARSE_MIN_SAMPLES = 10U;
static constexpr uint32_t SERVO_TOTAL_FINE_MIN_SAMPLES = 30U;
static constexpr double SERVO_TOTAL_COARSE_THRESHOLD_PPB = 50.0;
static constexpr double SERVO_TOTAL_COARSE_EXIT_PPB = 5.0;
static constexpr double SERVO_TOTAL_COARSE_RATE_ENTRY_PPB = 100.0;
static constexpr double SERVO_TOTAL_COARSE_RATE_EXIT_PPB = 20.0;
static constexpr double SERVO_TOTAL_COARSE_HORIZON_SECONDS = 300.0;
static constexpr double SERVO_TOTAL_FINE_HORIZON_SECONDS = 450.0;
static constexpr double SERVO_TOTAL_COARSE_MAX_RATE_PPB = 400.0;
static constexpr double SERVO_TOTAL_FINE_MAX_RATE_PPB = 20.0;
static constexpr double SERVO_TOTAL_COARSE_MAX_STEP_LSB = 4.0;
static constexpr double SERVO_TOTAL_FINE_MAX_STEP_LSB = 0.25;
static constexpr double SERVO_TOTAL_COARSE_ACTUATOR_GAIN = 1.0;
static constexpr double SERVO_TOTAL_FINE_ACTUATOR_GAIN = 0.75;
static constexpr double SERVO_TOTAL_COARSE_RATE_DEADBAND_PPB = 8.0;
static constexpr double SERVO_TOTAL_FINE_RATE_DEADBAND_PPB = 0.5;
static constexpr double SERVO_TOTAL_COARSE_STDERR_MULTIPLIER = 3.0;
static constexpr double SERVO_TOTAL_FINE_STDERR_MULTIPLIER = 1.5;
static constexpr double SERVO_TOTAL_POSITION_HOLD_PPB = 0.005;
static constexpr double SERVO_TOTAL_NEAR_ZERO_RATE_HOLD_PPB = 2.0;
static constexpr double SERVO_TOTAL_ZERO_CROSSING_LOOKAHEAD_SECONDS = 60.0;
static constexpr double SERVO_TOTAL_DAC_CHANGE_EPSILON_LSB = 0.000001;

struct servo_total_state_t {
  bool     initialized = false;
  uint32_t basis = SERVO_TOTAL_BASIS_NONE;
  uint32_t last_commit_count = 0U;
  double   last_dac_value = 0.0;
  uint64_t last_population_seconds = 0ULL;
  uint64_t last_sample_control_second = 0ULL;

  double   recent_samples[SERVO_TOTAL_RECENT_WINDOW_SAMPLES] = {};
  uint32_t recent_count = 0U;
  uint32_t recent_next = 0U;
  double   recent_sum = 0.0;
  double   recent_sum_sq = 0.0;
  uint32_t samples_since_reset = 0U;

  bool     coarse_latched = false;
};

// This is deliberately transient control memory, not durable instrument truth.
// A cold restore reacquires a fresh post-DAC residual window; long-baseline TAU
// and physical DAC custody remain durable through structured recovery state.
static servo_total_state_t g_servo_total_ocxo1 DMAMEM = {};
static servo_total_state_t g_servo_total_ocxo2 DMAMEM = {};

static servo_total_state_t& servo_total_state_for(ocxo_dac_state_t& dac) {
  return (&dac == &ocxo1_dac) ? g_servo_total_ocxo1 : g_servo_total_ocxo2;
}

static void servo_total_window_clear(servo_total_state_t& state) {
  memset(state.recent_samples, 0, sizeof(state.recent_samples));
  state.recent_count = 0U;
  state.recent_next = 0U;
  state.recent_sum = 0.0;
  state.recent_sum_sq = 0.0;
  state.samples_since_reset = 0U;
  state.last_sample_control_second = 0ULL;
}

static void servo_total_state_reset(servo_total_state_t& state,
                                    const ocxo_dac_state_t& dac,
                                    uint32_t basis,
                                    uint64_t population_seconds) {
  state = servo_total_state_t{};
  state.initialized = true;
  state.basis = basis;
  state.last_commit_count = dac.pacing_commit_count;
  state.last_dac_value = ocxo_dac_fractional_snapshot(dac);
  state.last_population_seconds = population_seconds;
}

static void servo_total_reset_all(void) {
  g_servo_total_ocxo1 = servo_total_state_t{};
  g_servo_total_ocxo2 = servo_total_state_t{};
}

static bool servo_total_sync_operating_point(
    servo_total_state_t& state,
    const ocxo_dac_state_t& dac,
    const servo_input_diag_t& input) {
  const double dac_value = ocxo_dac_fractional_snapshot(dac);
  if (!state.initialized) {
    servo_total_state_reset(
        state, dac, input.total_basis, input.total_population_seconds);
    return false;
  }

  const bool basis_changed = state.basis != input.total_basis;
  const bool population_regressed =
      input.total_population_seconds < state.last_population_seconds;
  const bool commit_changed =
      state.last_commit_count != dac.pacing_commit_count;
  const bool dac_changed =
      fabs(dac_value - state.last_dac_value) >
          SERVO_TOTAL_DAC_CHANGE_EPSILON_LSB;

  if (basis_changed || population_regressed || commit_changed || dac_changed) {
    servo_total_state_reset(
        state, dac, input.total_basis, input.total_population_seconds);
    return true;
  }

  state.last_population_seconds = input.total_population_seconds;
  return false;
}

static void servo_total_push_recent(servo_total_state_t& state,
                                    double sample_ppb,
                                    uint64_t control_second) {
  if (state.last_sample_control_second != 0ULL &&
      control_second != state.last_sample_control_second + 1ULL) {
    servo_total_window_clear(state);
  }
  state.last_sample_control_second = control_second;

  if (state.recent_count < SERVO_TOTAL_RECENT_WINDOW_SAMPLES) {
    state.recent_samples[state.recent_next] = sample_ppb;
    state.recent_sum += sample_ppb;
    state.recent_sum_sq += sample_ppb * sample_ppb;
    state.recent_count++;
  } else {
    const double old = state.recent_samples[state.recent_next];
    state.recent_sum -= old;
    state.recent_sum_sq -= old * old;
    state.recent_samples[state.recent_next] = sample_ppb;
    state.recent_sum += sample_ppb;
    state.recent_sum_sq += sample_ppb * sample_ppb;
  }

  state.recent_next =
      (state.recent_next + 1U) % SERVO_TOTAL_RECENT_WINDOW_SAMPLES;
  state.samples_since_reset++;
}

static double servo_total_recent_mean(const servo_total_state_t& state) {
  return state.recent_count
      ? state.recent_sum / (double)state.recent_count
      : 0.0;
}

static double servo_total_recent_stderr(const servo_total_state_t& state) {
  if (state.recent_count < 2U) return 0.0;
  const double n = (double)state.recent_count;
  double variance =
      (state.recent_sum_sq - (state.recent_sum * state.recent_sum) / n) /
      (n - 1.0);
  if (variance < 0.0) variance = 0.0;
  return sqrt(variance / n);
}

// NOW mode is deliberately immediate: no settle gate, no slope averaging, and
// a small deadband.  It is a live plant-response/test mode that chases the
// most recent PPS-founded one-second residual directly.
static constexpr double SERVO_NOW_FILTER_ALPHA = 1.00;
static constexpr double SERVO_NOW_GAIN = 1.00;
static constexpr double SERVO_NOW_DEADBAND_PPB = 0.5;

static double servo_total_ppb_from_tau(double tau) {
  return (tau - 1.0) * 1.0e9;
}

static double servo_clamp(double value, double limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static double servo_total_requested_rate_ppb(double total_ppb,
                                             uint64_t population_seconds,
                                             bool coarse) {
  if (population_seconds == 0ULL) return 0.0;
  const double horizon = coarse
      ? SERVO_TOTAL_COARSE_HORIZON_SECONDS
      : SERVO_TOTAL_FINE_HORIZON_SECONDS;
  const double max_rate = coarse
      ? SERVO_TOTAL_COARSE_MAX_RATE_PPB
      : SERVO_TOTAL_FINE_MAX_RATE_PPB;
  const double requested =
      -total_ppb * ((double)population_seconds / horizon);
  return servo_clamp(requested, max_rate);
}

static bool servo_total_crosses_zero_within_lookahead(
    double total_ppb,
    uint64_t population_seconds,
    double recent_mean_ppb) {
  if (population_seconds == 0ULL || total_ppb == 0.0 ||
      recent_mean_ppb == 0.0 || total_ppb * recent_mean_ppb >= 0.0) {
    return false;
  }

  const double seconds_to_zero =
      -total_ppb * (double)population_seconds / recent_mean_ppb;
  return seconds_to_zero >= 0.0 &&
      seconds_to_zero <= SERVO_TOTAL_ZERO_CROSSING_LOOKAHEAD_SECONDS;
}

// Servo-input diagnostics are populated directly in clocks_beta_pps().
// Keep this hot path free of a large reference-heavy helper call boundary.

// ============================================================================
// Servo DAC requests — dither-owned realization
// ============================================================================
//
// Beta/servo is now a pure intent producer.  It computes the real-valued DAC
// target from the science residual and queues that request into the shared DAC
// state.  Alpha's dither owner consumes the request at the one-second frame
// boundary before programming the low/high waveform.  Beta never performs
// Wire/AD5693R I/O and never schedules a competing hardware commit service.

static ocxo_dac_state_t* g_ocxo_dac_commit_selected = nullptr;
static double          g_ocxo_dac_commit_target = 0.0;
static uint16_t        g_ocxo_dac_commit_target_hw_code = 0;
static uint64_t        g_ocxo_dac_last_schedule_second = 0;
static uint8_t         g_ocxo_dac_last_winner = 0;
static uint32_t        g_ocxo_dac_arbitration_passes = 0;
static uint32_t        g_ocxo_dac_deferred_candidates = 0;
static uint64_t        g_servo_control_second = 0;

static uint8_t ocxo_dac_lane_id(const ocxo_dac_state_t& dac) {
  return (&dac == &ocxo1_dac) ? 1U : 2U;
}

static void ocxo_dac_clear_pending(ocxo_dac_state_t& d) {
  ocxo_dac_clear_servo_request(d);
}

static void ocxo_dac_pacing_reset(void) {
  ocxo_dac_clear_pending(ocxo1_dac);
  ocxo_dac_clear_pending(ocxo2_dac);
  g_ocxo_dac_commit_selected = nullptr;
  g_ocxo_dac_commit_target = 0.0;
  g_ocxo_dac_commit_target_hw_code = 0;
  g_ocxo_dac_last_winner = 0;
}

static void ocxo_dac_pacing_abort_all(void) {
  ocxo_dac_pacing_reset();
}

static const char* servo_hold_reason_name(uint32_t reason) {
  switch (reason) {
    case SERVO_HOLD_PENDING_COMMIT:
      return "PENDING_DITHER_OWNER_INSTALL";
    case SERVO_HOLD_SETTLE_QUARANTINE:
      return "SETTLE_QUARANTINE";
    case SERVO_HOLD_COMMIT_FAULT_BACKOFF:
      return "COMMIT_FAULT_BACKOFF";
    case SERVO_HOLD_SMALL_STATIC_DELTA:
      return "SMALL_STATIC_DELTA";
    default:
      return "NONE";
  }
}

static void ocxo_dac_note_servo_hold(ocxo_dac_state_t& dac, uint8_t reason) {
  dac.servo_hold_reason = reason;
  dac.servo_hold_count++;
  if (reason == SERVO_HOLD_COMMIT_FAULT_BACKOFF) {
    dac.servo_commit_fault_hold_count++;
  }
}

static bool ocxo_dac_servo_hold_active(ocxo_dac_state_t& dac) {
  if (dac.servo_quarantine_remaining != 0U) {
    const uint8_t reason = dac.servo_quarantine_reason
        ? dac.servo_quarantine_reason
        : SERVO_HOLD_SETTLE_QUARANTINE;
    dac.servo_quarantine_remaining--;
    dac.servo_quarantine_consumed_count++;
    ocxo_dac_note_servo_hold(dac, reason);
    return true;
  }

  dac.servo_hold_reason = SERVO_HOLD_NONE;
  dac.servo_quarantine_reason = SERVO_HOLD_NONE;
  return false;
}

static void clocks_apply_servo_mode_now(servo_mode_t mode) {
  const servo_mode_t previous = calibrate_ocxo_mode;
  calibrate_ocxo_mode = mode;

  // A mode boundary changes the meaning/filtering of selected_input_ppb.
  // Reset predictor/settle state so a new live SERVOS mode never inherits stale
  // slope memory from MEAN/TOTAL/NOW or from an OFF interval.
  if (previous != mode) {
    ocxo_dac_pacing_abort_all();
    servo_total_reset_all();
    ocxo_dac_predictor_reset(ocxo1_dac);
    ocxo_dac_predictor_reset(ocxo2_dac);
    ocxo1_dac.servo_settle_count = 0;
    ocxo2_dac.servo_settle_count = 0;
    ocxo1_dac.servo_hold_reason = SERVO_HOLD_NONE;
    ocxo2_dac.servo_hold_reason = SERVO_HOLD_NONE;
    ocxo1_dac.servo_quarantine_reason = SERVO_HOLD_NONE;
    ocxo2_dac.servo_quarantine_reason = SERVO_HOLD_NONE;
    ocxo1_dac.servo_quarantine_remaining = 0;
    ocxo2_dac.servo_quarantine_remaining = 0;
  }

  if (mode == servo_mode_t::OFF) {
    ocxo_dac_pacing_abort_all();
    clocks_ocxo_dac_cancel_all_motion();
    ocxo1_dac.servo_last_step = 0.0;
    ocxo2_dac.servo_last_step = 0.0;
  }
}

static void clocks_commit_pending_servo_mode_change(void) {
  if (!request_servo_mode_change) return;

  const servo_mode_t mode = requested_servo_mode;
  request_servo_mode_change = false;
  clocks_apply_servo_mode_now(mode);
  g_servo_mode_last_committed = mode;
  g_servo_mode_commit_count++;
}


static void clocks_recovery_restore_dac_lane_runtime(
    ocxo_dac_state_t& dst,
    const clocks_monitor_dither_lane_t& src) {
  dst.servo_last_step = src.servo_last_step;
  dst.servo_last_residual = src.servo_last_residual;
  dst.servo_settle_count = src.servo_settle_count;
  dst.servo_adjustments = src.servo_adjustments;
  dst.servo_predictor_initialized = src.servo_predictor_initialized;
  dst.servo_last_raw_residual = src.servo_last_raw_residual;
  dst.servo_filtered_residual = src.servo_filtered_residual;
  dst.servo_filtered_slope = src.servo_filtered_slope;
  dst.servo_predicted_residual = src.servo_predicted_residual;
  dst.servo_predictor_updates = src.servo_predictor_updates;

  // Reboot cannot preserve an in-flight actuator transaction or a row-local
  // quarantine.  Resume only the learned plant model.
  dst.servo_hold_reason = SERVO_HOLD_NONE;
  dst.servo_quarantine_reason = SERVO_HOLD_NONE;
  dst.servo_quarantine_remaining = 0U;
}

static bool clocks_recovery_install_dac_control(
    const clocks_recovery_restore_state_t& state) {
  if (!clocks_recovery_restore_control_valid(state)) return false;

  ocxo_dac_pacing_abort_all();
  clocks_ocxo_dac_cancel_all_motion();
  (void)clocks_ocxo_dac_dither_disable();

  // Restore the physical operating point while firmware is in a foreground
  // command context.  The PPS commit later restores statistics and logical
  // clock presentation without performing I2C in the timing path.
  calibrate_ocxo_mode = servo_mode_t::OFF;
  const bool dac1_ok = ocxo_dac_set(ocxo1_dac, state.ocxo1_dac.value);
  const bool dac2_ok = ocxo_dac_set(ocxo2_dac, state.ocxo2_dac.value);
  if (!dac1_ok || !dac2_ok) {
    calibrate_ocxo_mode = servo_mode_t::OFF;
    requested_servo_mode = servo_mode_t::OFF;
    request_servo_mode_change = false;
    return false;
  }

  clocks_recovery_restore_dac_lane_runtime(ocxo1_dac, state.ocxo1_dac);
  clocks_recovery_restore_dac_lane_runtime(ocxo2_dac, state.ocxo2_dac);

  // Do not call clocks_apply_servo_mode_now(): a mode transition there
  // deliberately erases the predictor state we just restored.
  calibrate_ocxo_mode = state.servo_mode;
  requested_servo_mode = state.servo_mode;
  request_servo_mode_change = false;
  g_servo_mode_last_requested = state.servo_mode;
  g_servo_mode_last_committed = state.servo_mode;
  servo_total_reset_all();
  if (state.servo_mode == servo_mode_t::TOTAL) {
    ocxo_dac_predictor_reset(ocxo1_dac);
    ocxo_dac_predictor_reset(ocxo2_dac);
  }

  if (state.dither_operator_enabled) {
    if (!clocks_ocxo_dac_dither_enable()) {
      (void)clocks_ocxo_dac_dither_disable();
      clocks_ocxo_dac_cancel_all_motion();
      calibrate_ocxo_mode = servo_mode_t::OFF;
      requested_servo_mode = servo_mode_t::OFF;
      request_servo_mode_change = false;
      return false;
    }
  }
  return true;
}

static bool clocks_recovery_commit_statistics_and_clockfaces(
    const clocks_recovery_restore_state_t& state) {
  if (!clocks_recovery_restore_statistics_valid(state)) return false;
  if (!clocks_alpha_instrument_stats_restore(&state.stats)) return false;
  instrument_continuity_install(state);
  return true;
}

static void ocxo_dac_apply_synthetic_servo_step(ocxo_dac_state_t& dac,
                                                double step) {
  const double before = ocxo_dac_fractional_snapshot(dac);
  const double target = ocxo_dac_clamp_real_value(before + step);
  const double planned_step = target - before;
  const uint16_t target_hw_code =
      ocxo_dac_rounded_hw_code_from_value(target);

  if (fabs(planned_step) < 0.000001) {
    dac.pacing_skip_small_delta_count++;
    dac.servo_last_step = 0.0;
    dac.servo_hold_reason = SERVO_HOLD_SMALL_STATIC_DELTA;
    return;
  }

  if (!clocks_ocxo_dac_dither_operator_enabled() &&
      target_hw_code == dac.dac_hw_code) {
    dac.pacing_skip_small_delta_count++;
    dac.servo_last_step = 0.0;
    dac.servo_hold_reason = SERVO_HOLD_SMALL_STATIC_DELTA;
    return;
  }

  if (dac.pacing_pending) {
    g_ocxo_dac_deferred_candidates++;
  }

  g_ocxo_dac_arbitration_passes++;
  g_ocxo_dac_commit_selected = &dac;
  g_ocxo_dac_commit_target = target;
  g_ocxo_dac_commit_target_hw_code = target_hw_code;
  g_ocxo_dac_last_schedule_second = g_servo_control_second;
  g_ocxo_dac_last_winner = ocxo_dac_lane_id(dac);

  ocxo_dac_request_servo_target(
      dac, target, planned_step, g_servo_control_second);
}

// ============================================================================
// Zeroing
// ============================================================================

static void campaign_accounting_reset_common() {
  clocks_watchdog_disarm_campaign_publication();
  clocks_row_objection_clear();

  // Beta-local recording/accounting reset only. Instrument statistics, DAC
  // targets, dither ownership, servo mode, controller memory, and pending
  // actuator intent survive every campaign boundary.
  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_measured_gnss_ticks_64 = 0;
  ocxo2_measured_gnss_ticks_64 = 0;

  campaign_public_offsets_reset_to_current();
  pps_interval_residuals_reset();
  clocks_beta_feature_set_cached("CAMPAIGN_RECORD_HANDOFF",
                                 g_clocks_feature_campaign_record,
                                 system_feature_status_t::INITIALIZING,
                                 true);

  // These are row-presentation diagnostics, not controller memory.
  servo_input_diag_reset(g_servo_input_ocxo1);
  servo_input_diag_reset(g_servo_input_ocxo2);
}


void clocks_zero_all(void) {
  campaign_accounting_reset_common();
}

static void campaign_flash_cut_accounting_reset(void) {
  campaign_accounting_reset_common();
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
  if (ocxo_dac_servo_hold_active(dac)) return;

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
  if (!input.total_input_valid || !input.now_input_valid ||
      input.total_population_seconds == 0ULL) {
    return;
  }

  servo_total_state_t& state = servo_total_state_for(dac);
  const bool operating_point_changed =
      servo_total_sync_operating_point(state, dac, input);

  // A queued actuator request is immutable custody.  TOTAL never overwrites it
  // with another one-second opinion.  After installation the commit counter / DAC
  // target change above resets the residual population, and the ordinary owner
  // quarantine removes the first two post-commit rows from control math.
  if (dac.pacing_pending) {
    ocxo_dac_note_servo_hold(dac, SERVO_HOLD_PENDING_COMMIT);
    return;
  }
  if (ocxo_dac_servo_hold_active(dac)) return;
  if (operating_point_changed) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  servo_total_push_recent(state, input.now_ppb, g_servo_control_second);

  const double total_ppb = input.total_ppb;
  const double recent_mean_ppb = servo_total_recent_mean(state);
  const double recent_stderr_ppb = servo_total_recent_stderr(state);

  // Coarse authority is hysteretic and population-aware.  Even a modest PPB
  // error represents substantial phase debt after a long population.  Enter
  // coarse mode whenever the rate required to retire that debt within the fine
  // horizon would exceed fine authority.  Remain coarse long enough to brake the
  // rate it created, then hand off only when position, rate, and required fine
  // authority are all inside the fine-control neighborhood.
  const double fine_rate_required_abs_ppb =
      fabs(total_ppb) *
      ((double)input.total_population_seconds /
       SERVO_TOTAL_FINE_HORIZON_SECONDS);
  if (state.coarse_latched) {
    if (fabs(total_ppb) <= SERVO_TOTAL_COARSE_EXIT_PPB &&
        fabs(recent_mean_ppb) <= SERVO_TOTAL_COARSE_RATE_EXIT_PPB &&
        fine_rate_required_abs_ppb <= SERVO_TOTAL_FINE_MAX_RATE_PPB) {
      state.coarse_latched = false;
    }
  } else if (fabs(total_ppb) >= SERVO_TOTAL_COARSE_THRESHOLD_PPB ||
             fabs(recent_mean_ppb) >= SERVO_TOTAL_COARSE_RATE_ENTRY_PPB ||
             fine_rate_required_abs_ppb > SERVO_TOTAL_FINE_MAX_RATE_PPB) {
    state.coarse_latched = true;
  }
  const bool coarse = state.coarse_latched;
  const uint32_t required_samples = coarse
      ? SERVO_TOTAL_COARSE_MIN_SAMPLES
      : SERVO_TOTAL_FINE_MIN_SAMPLES;

  const bool population_ready = state.recent_count >= required_samples;

  // Retain the legacy monitor fields as a compact TOTAL flight recorder:
  //   last_residual       = long-baseline TOTAL position
  //   last_raw_residual   = newest admitted one-second residual
  //   filtered_residual   = short-window residual mean (live rate)
  //   filtered_slope      = requested live rate
  //   predicted_residual  = live-rate error presented to the DAC controller
  dac.servo_last_residual = total_ppb;
  dac.servo_last_raw_residual = input.now_ppb;
  dac.servo_filtered_residual = recent_mean_ppb;
  dac.servo_predictor_initialized = true;
  dac.servo_predictor_updates = state.samples_since_reset;
  dac.servo_settle_count = state.recent_count;

  if (!population_ready) {
    dac.servo_filtered_slope = 0.0;
    dac.servo_predicted_residual = 0.0;
    ocxo_dac_clear_pending(dac);
    return;
  }

  double requested_rate_ppb = servo_total_requested_rate_ppb(
      total_ppb, input.total_population_seconds, coarse);
  const bool crossing_brake = servo_total_crosses_zero_within_lookahead(
      total_ppb, input.total_population_seconds, recent_mean_ppb);
  if (crossing_brake) {
    // The current residual trajectory would carry the long-memory total through
    // zero within the look-ahead window.  Command zero live rate now; do not wait
    // for TOTAL itself to cross before reversing DAC direction.
    requested_rate_ppb = 0.0;
  }

  const double rate_error_ppb = recent_mean_ppb - requested_rate_ppb;
  const double stderr_multiplier = coarse
      ? SERVO_TOTAL_COARSE_STDERR_MULTIPLIER
      : SERVO_TOTAL_FINE_STDERR_MULTIPLIER;
  const double floor_deadband = coarse
      ? SERVO_TOTAL_COARSE_RATE_DEADBAND_PPB
      : SERVO_TOTAL_FINE_RATE_DEADBAND_PPB;
  const double rate_deadband_ppb = fmax(
      floor_deadband, stderr_multiplier * recent_stderr_ppb);

  dac.servo_filtered_slope = requested_rate_ppb;
  dac.servo_predicted_residual = rate_error_ppb;

  const double near_zero_rate_hold_ppb = fmax(
      rate_deadband_ppb, SERVO_TOTAL_NEAR_ZERO_RATE_HOLD_PPB);
  if ((fabs(total_ppb) <= SERVO_TOTAL_POSITION_HOLD_PPB &&
       fabs(recent_mean_ppb) <= near_zero_rate_hold_ppb) ||
      fabs(rate_error_ppb) <= rate_deadband_ppb) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  const double actuator_gain = coarse
      ? SERVO_TOTAL_COARSE_ACTUATOR_GAIN
      : SERVO_TOTAL_FINE_ACTUATOR_GAIN;
  const double max_step_lsb = coarse
      ? SERVO_TOTAL_COARSE_MAX_STEP_LSB
      : SERVO_TOTAL_FINE_MAX_STEP_LSB;

  // Inner rate-loop sign law:
  //   rate_error > 0 -> actual OCXO rate is too fast -> lower DAC
  //   rate_error < 0 -> actual OCXO rate is too slow -> raise DAC
  double step =
      -rate_error_ppb / SERVO_PPB_PER_DAC_LSB_ESTIMATE * actuator_gain;
  step = servo_clamp(step, max_step_lsb);

  if (fabs(step) < 0.000001) {
    ocxo_dac_clear_pending(dac);
    return;
  }

  ocxo_dac_apply_synthetic_servo_step(dac, step);
}

static void ocxo_servo_now(ocxo_dac_state_t& dac,
                           const servo_input_diag_t& input) {
  if (!input.now_input_valid) return;

  // NOW chases the current one-second residual directly.  There is no settle
  // divider and no soft landing.  Decimal DAC targets are retained as control
  // intent, while the hardware receives the nearest static integer code.
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
// Always-on instrument servo
// ============================================================================
//
// Campaigns are recording namespaces, not owners of the physical instrument.
// When no campaign is active, drive the selected servo from Alpha's coherent
// boot-lifetime instrument snapshot instead of campaign_seconds/public totals.
// This keeps MEAN/TOTAL/NOW behavior available whenever the instrument has a
// lawful completed row.
//
// TOTAL uses Alpha's boot-lifetime frequency estimate as its position basis
// while idle and the same-row Delta residual as its live-rate witness.  A basis
// change between campaign and instrument state resets the short control window;
// the long-lived Alpha statistics themselves remain untouched.
//
// NOW derives the latest one-second fast residual directly from the coherent
// selected-reference and OCXO DWT intervals.  MEAN uses Alpha's always-on
// Welford population.
static clocks_instrument_stats_snapshot_t
    g_beta_idle_servo_instrument_stats DMAMEM = {};

static double idle_servo_fast_ppb_from_cycles(uint32_t reference_cycles,
                                               uint32_t clock_cycles) {
  if (reference_cycles == 0U || clock_cycles == 0U) return 0.0;
  const int64_t fast_cycles =
      (int64_t)reference_cycles - (int64_t)clock_cycles;
  return ((double)fast_cycles * 1.0e9) / (double)reference_cycles;
}

static void idle_servo_populate_lane(
    servo_input_diag_t& input,
    const clocks_instrument_frequency_snapshot_t& frequency,
    const welford_t& residual_welford,
    uint32_t reference_cycles,
    uint32_t clock_cycles,
    uint64_t elapsed_seconds) {
  input = servo_input_diag_t{};

  const bool interval_valid =
      reference_cycles != 0U && clock_cycles != 0U;
  const double now_ppb = interval_valid
      ? idle_servo_fast_ppb_from_cycles(reference_cycles, clock_cycles)
      : 0.0;

  input.pps_residual_valid = interval_valid;
  input.pps_gnss_interval_ns = interval_valid
      ? CLOCKS_BETA_NS_PER_SECOND
      : 0ULL;
  input.pps_clock_interval_ns = interval_valid
      ? (uint64_t)((double)CLOCKS_BETA_NS_PER_SECOND -
                   now_ppb)
      : 0ULL;
  input.pps_fast_residual_ns = interval_valid
      ? (int64_t)llround(now_ppb)
      : 0LL;

  input.mean_welford_n = residual_welford.n;
  input.mean_welford_ppb =
      residual_welford.n ? residual_welford.mean : 0.0;
  input.mean_input_valid =
      interval_valid && residual_welford.n >= SERVO_MIN_SAMPLES;

  input.total_tau = frequency.valid ? frequency.tau : 1.0;
  input.total_ppb = frequency.valid ? frequency.ppb : 0.0;
  input.total_basis = SERVO_TOTAL_BASIS_INSTRUMENT;
  input.total_population_seconds =
      (frequency.valid && frequency.sample_count != 0ULL)
          ? frequency.sample_count
          : elapsed_seconds;
  input.total_input_valid =
      interval_valid &&
      frequency.valid &&
      frequency.sample_count >= SERVO_MIN_SAMPLES;

  input.now_ppb = now_ppb;
  input.now_input_valid = interval_valid;

  input.selected_residual_ns = 0;
  if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
    input.selected_source =
        SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD;
    input.selected_input_valid = input.mean_input_valid;
    input.selected_input_ppb =
        input.mean_input_valid ? input.mean_welford_ppb : 0.0;
    input.selected_residual_ns =
        (int64_t)llround(input.selected_input_ppb);
  } else if (calibrate_ocxo_mode == servo_mode_t::TOTAL) {
    input.selected_source = SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU;
    input.selected_input_valid =
        input.total_input_valid && input.now_input_valid;
    input.selected_input_ppb = input.total_input_valid
        ? input.total_ppb
        : 0.0;
    input.selected_residual_ns =
        (int64_t)llround(input.selected_input_ppb);
  } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    input.selected_source = SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL;
    input.selected_input_valid = input.now_input_valid;
    input.selected_input_ppb =
        input.now_input_valid ? input.now_ppb : 0.0;
    input.selected_residual_ns =
        input.now_input_valid ? input.pps_fast_residual_ns : 0LL;
  } else {
    input.selected_source = SERVO_INPUT_SOURCE_NONE;
    input.selected_input_valid = false;
    input.selected_input_ppb = 0.0;
  }
}

static void clocks_idle_instrument_servo(uint32_t completed_pps_sequence) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  g_beta_idle_servo_instrument_stats =
      clocks_instrument_stats_snapshot_t{};
  if (!clocks_alpha_instrument_stats_snapshot(
          &g_beta_idle_servo_instrument_stats) ||
      !g_beta_idle_servo_instrument_stats.valid ||
      !g_beta_idle_servo_instrument_stats.completed_row_coherent ||
      g_beta_idle_servo_instrument_stats.last_pps_sequence !=
          completed_pps_sequence) {
    return;
  }

  const uint64_t elapsed_seconds =
      (uint64_t)g_beta_idle_servo_instrument_stats.update_count;

  idle_servo_populate_lane(
      g_servo_input_ocxo1,
      g_beta_idle_servo_instrument_stats.ocxo1_frequency,
      g_beta_idle_servo_instrument_stats.ocxo1_welford,
      g_beta_idle_servo_instrument_stats.selected_reference_interval_cycles,
      g_beta_idle_servo_instrument_stats.ocxo1_interval_cycles,
      elapsed_seconds);
  idle_servo_populate_lane(
      g_servo_input_ocxo2,
      g_beta_idle_servo_instrument_stats.ocxo2_frequency,
      g_beta_idle_servo_instrument_stats.ocxo2_welford,
      g_beta_idle_servo_instrument_stats.selected_reference_interval_cycles,
      g_beta_idle_servo_instrument_stats.ocxo2_interval_cycles,
      elapsed_seconds);

  g_servo_control_second = elapsed_seconds;
  ocxo_calibration_servo();
}

// ============================================================================
// Flash Cut
// ============================================================================

static void flash_cut_clear_pending(void) {
  request_flash_cut = false;
  flash_cut_campaign_name[0] = '\0';
}

static bool flash_cut_busy(void) {
  return request_flash_cut || request_start || request_stop ||
         request_recover || request_zero || g_monitor_restore_requested ||
         interrupt_smartzero_running() ||
         clocks_alpha_epoch_install_in_progress();
}

static void campaign_flash_cut_commit_at_pps(void) {
  safeCopy(g_flash_cut_last_from_campaign,
           sizeof(g_flash_cut_last_from_campaign),
           campaign_name);

  g_flash_cut_last_raw_gnss_ns = current_raw_gnss_ns();
  g_flash_cut_last_raw_dwt_cycles = g_dwt_cycle_count_total;
  g_flash_cut_last_raw_ocxo1_ns = current_raw_ocxo1_ns();
  g_flash_cut_last_raw_ocxo2_ns = current_raw_ocxo2_ns();
  g_flash_cut_last_boundary_pps_count = (uint32_t)campaign_seconds;

  safeCopy(campaign_name, sizeof(campaign_name), flash_cut_campaign_name);
  safeCopy(g_flash_cut_last_to_campaign,
           sizeof(g_flash_cut_last_to_campaign),
           campaign_name);

  // This is the cut itself: Beta rebases campaign-public clock presentation at
  // the already-authored PPS/VCLOCK edge while Alpha statistics continue, then
  // returns without emitting a
  // campaign record pair for the boundary row.  The next PPS publishes count=1 and
  // public GNSS/DWT/OCXO ledgers approximately +1 second from this boundary.
  campaign_flash_cut_accounting_reset();
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
  g_campaign_warmup_suppressed_total = 0;
  campaign_state = clocks_campaign_state_t::STARTED;

  flash_cut_clear_pending();
  g_flash_cut_commit_count++;
  safeCopy(g_flash_cut_last_status, sizeof(g_flash_cut_last_status),
           "flash_cut_committed");
}

// ============================================================================
// Watchdog
// ============================================================================

static void clocks_force_stop_campaign(void) {
  clocks_watchdog_disarm_campaign_publication();
  campaign_state = clocks_campaign_state_t::STOPPED;
  safeCopy(g_recover_lifecycle_abort_reason,
           sizeof(g_recover_lifecycle_abort_reason),
           "force_stop_campaign");
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  flash_cut_clear_pending();

  // Continuity surrender stops campaign publication, not the independent servo
  // control plane. The watchdog gate already prevents suspect rows from feeding
  // servo math until recovery clears custody.
  campaign_warmup_reset();
}

static bool clocks_watchdog_surrender_now(const char* reason,
                                        uint32_t detail0,
                                        uint32_t detail1,
                                        uint32_t detail2,
                                        uint32_t detail3,
                                        bool publish_pending) {
  if (!clocks_watchdog_campaign_armed() && !watchdog_campaign_surrendered) {
    watchdog_anomaly_suppressed_unarmed_count++;
    return false;
  }

  const bool first = !watchdog_anomaly_active && !watchdog_campaign_surrendered;
  if (first) {
    watchdog_anomaly_sequence++;
    safeCopy(watchdog_anomaly_reason, sizeof(watchdog_anomaly_reason),
             (reason && *reason) ? reason : "watchdog_anomaly");
    watchdog_anomaly_detail0 = detail0;
    watchdog_anomaly_detail1 = detail1;
    watchdog_anomaly_detail2 = detail2;
    watchdog_anomaly_detail3 = detail3;
    watchdog_anomaly_trigger_dwt = DWT_CYCCNT;
    watchdog_campaign_surrender_count++;
  }

  watchdog_campaign_surrendered = true;
  watchdog_campaign_publication_armed = false;
  watchdog_anomaly_active = true;
  watchdog_anomaly_publish_pending = publish_pending && first;

  // Campaign continuity failure must not cancel an already-lawful actuator
  // request. The publication/watchdog gate holds further servo input until the
  // lifecycle is recovered.
  return first;
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

  watchdog_anomaly_legacy_publish_count++;
  publish("WATCHDOG_ANOMALY", p);

  watchdog_anomaly_publish_pending = false;
  clocks_force_stop_campaign();
}

void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3) {
  if (!clocks_watchdog_surrender_now(reason, detail0, detail1, detail2, detail3, true)) {
    return;
  }

  const timepop_handle_t h =
      timepop_arm_asap(clocks_watchdog_anomaly_callback, nullptr, "clocks-anomaly");

  if (h == TIMEPOP_INVALID_HANDLE) {
    watchdog_anomaly_publish_pending = false;
    clocks_force_stop_campaign();
  }
}

void clocks_watchdog_anomaly_payload(const char* reason,
                                     const Payload& payload,
                                     uint32_t detail0,
                                     uint32_t detail1,
                                     uint32_t detail2,
                                     uint32_t detail3) {
  if (!clocks_watchdog_surrender_now(reason, detail0, detail1, detail2, detail3, false)) {
    return;
  }

  watchdog_anomaly_verbose_publish_count++;
  publish("WATCHDOG_ANOMALY", payload);
  clocks_force_stop_campaign();
}

// ============================================================================
// Local ZERO / START completion helpers
// ============================================================================

static void clocks_finish_zero_accounting(void) {
  clocks_zero_all();
  instrument_continuity_reset();
  request_zero = false;
}

static void clocks_finish_start_accounting(void) {
  campaign_accounting_reset_common();
  request_zero = false;
  request_start = false;
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  campaign_state = clocks_campaign_state_t::STARTED;
  campaign_warmup_begin(campaign_warmup_mode_t::START);
}


static bool clocks_try_finish_pending_smartzero(void) {
  if (!request_zero) return false;
  if (!interrupt_smartzero_complete()) return false;

  const bool zero_ok = clocks_alpha_zero_from_smartzero("zero");
  if (!zero_ok) {
    const clocks_alpha_ocxo_grid_rephase_status_t status =
        clocks_alpha_ocxo_grid_rephase_status(
            clocks_alpha_ocxo_grid_rephase_owner_t::SMARTZERO_EPOCH);
    if (status == clocks_alpha_ocxo_grid_rephase_status_t::PENDING) {
      return false;
    }
    if (status == clocks_alpha_ocxo_grid_rephase_status_t::FAILED) {
      clocks_alpha_ocxo_grid_rephase_acknowledge(
          clocks_alpha_ocxo_grid_rephase_owner_t::SMARTZERO_EPOCH);
    }
    request_zero = false;
    return false;
  }

  campaign_state = clocks_campaign_state_t::STOPPED;
  clocks_finish_zero_accounting();
  return true;
}


static bool clocks_servo_active(void) {
  return calibrate_ocxo_mode != servo_mode_t::OFF;
}


static FLASHMEM void clocks_monitor_dac_lane_snapshot(
    clocks_monitor_dither_lane_t& out,
    const ocxo_dac_state_t& dac) {
  out = clocks_monitor_dither_lane_t{};
  out.value = ocxo_dac_fractional_snapshot(dac);
  out.hw_code = dac.dac_hw_code;
  out.readback_valid = dac.hw_readback_valid;
  out.readback_code = dac.hw_readback_code;
  out.active = dac.dither_active_this_frame;
  out.low_code = dac.dither_low_code;
  out.high_code = dac.dither_high_code;
  out.high_ms = dac.dither_high_ms;
  out.phase_high = dac.dither_current_phase_high;
  out.servo_last_step = dac.servo_last_step;
  out.servo_last_residual = dac.servo_last_residual;
  out.servo_settle_count = dac.servo_settle_count;
  out.servo_adjustments = dac.servo_adjustments;
  out.servo_predictor_initialized = dac.servo_predictor_initialized;
  out.servo_last_raw_residual = dac.servo_last_raw_residual;
  out.servo_filtered_residual = dac.servo_filtered_residual;
  out.servo_filtered_slope = dac.servo_filtered_slope;
  out.servo_predicted_residual = dac.servo_predicted_residual;
  out.servo_predictor_updates = dac.servo_predictor_updates;
}

static FLASHMEM void clocks_monitor_dac_snapshot(
    clocks_monitor_dac_snapshot_t& out) {
  memset(&out, 0, sizeof(out));
  clocks_monitor_copy_text(out.servo_mode,
                           sizeof(out.servo_mode),
                           servo_mode_str(calibrate_ocxo_mode));
  out.servo_active = clocks_servo_active();
  clocks_monitor_copy_text(out.realization_mode,
                           sizeof(out.realization_mode),
                           ocxo_dac_realization_mode_runtime());
  out.dither_operator_enabled = clocks_ocxo_dac_dither_operator_enabled();
  out.servo_request_pending =
      ocxo1_dac.pacing_pending || ocxo2_dac.pacing_pending;
  out.actuator_service_pending = clocks_ocxo_dac_actuator_service_pending();
  clocks_monitor_dac_lane_snapshot(out.ocxo1, ocxo1_dac);
  clocks_monitor_dac_lane_snapshot(out.ocxo2, ocxo2_dac);
}

static FLASHMEM void clocks_monitor_refresh_prediction_snapshots(void) {
  g_beta_monitor_pps_prediction = prediction_snapshot_for_pps();
  g_beta_monitor_vclock_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::VCLOCK);
  g_beta_monitor_ocxo1_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO1);
  g_beta_monitor_ocxo2_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO2);

  g_beta_monitor_vclock_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_monitor_ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_monitor_ocxo2_forensics = clocks_alpha_lane_forensics_t{};
  (void)clocks_alpha_lane_forensics(
      time_clock_id_t::VCLOCK, &g_beta_monitor_vclock_forensics);
  (void)clocks_alpha_lane_forensics(
      time_clock_id_t::OCXO1, &g_beta_monitor_ocxo1_forensics);
  (void)clocks_alpha_lane_forensics(
      time_clock_id_t::OCXO2, &g_beta_monitor_ocxo2_forensics);
}

static FLASHMEM void clocks_monitor_live_snapshot_fill(
    clocks_monitor_live_snapshot_t& out) {
  memset(&out, 0, sizeof(out));
  g_beta_monitor_instrument_stats = clocks_instrument_stats_snapshot_t{};
  out.snapshot_ok = clocks_alpha_instrument_stats_snapshot(
      &g_beta_monitor_instrument_stats);
  const clocks_instrument_stats_snapshot_t& instrument =
      g_beta_monitor_instrument_stats;

  const bool campaign_bound =
      campaign_state != clocks_campaign_state_t::STOPPED &&
      campaign_name[0] != '\0';
  const bool campaign_presentation_ready =
      campaign_bound && campaign_seconds > 0ULL;

  const uint64_t logical_instrument_gnss_ns =
      instrument_continuity_gnss_ns(instrument.gnss_ns);
  const uint64_t logical_instrument_dwt_cycles =
      instrument_continuity_dwt_cycles(instrument.dwt_cycles);
  const uint64_t logical_instrument_ocxo1_ns =
      instrument_continuity_ocxo1_ns(instrument.ocxo1_ns);
  const uint64_t logical_instrument_ocxo2_ns =
      instrument_continuity_ocxo2_ns(instrument.ocxo2_ns);

  out.valid = instrument.valid;
  out.completed_row_coherent = instrument.completed_row_coherent;
  out.completed_pps_sequence = instrument.last_pps_sequence;
  out.instrument_age_seconds =
      (uint32_t)(logical_instrument_gnss_ns / CLOCKS_BETA_NS_PER_SECOND);

  clocks_monitor_copy_text(out.campaign_state,
                           sizeof(out.campaign_state),
                           clocks_campaign_state_name(campaign_state));
  out.recording = campaign_state == clocks_campaign_state_t::STARTED;
  out.campaign_present = campaign_bound;
  clocks_monitor_copy_text(out.campaign,
                           sizeof(out.campaign),
                           campaign_bound ? campaign_name : "");
  clocks_monitor_copy_text(out.last_campaign,
                           sizeof(out.last_campaign),
                           campaign_name);
  out.campaign_seconds = campaign_seconds;
  out.campaign_presentation_ready = campaign_presentation_ready;
  clocks_monitor_copy_text(
      out.presentation_mode,
      sizeof(out.presentation_mode),
      campaign_presentation_ready
          ? "CAMPAIGN"
          : (campaign_bound ? "CAMPAIGN_ARMING" : "INSTRUMENT"));
  clocks_monitor_copy_text(
      out.presentation_basis,
      sizeof(out.presentation_basis),
      campaign_presentation_ready
          ? "CAMPAIGN_RELATIVE"
          : (g_instrument_continuity_active
                 ? "RESTORED_INSTRUMENT_CONTINUITY"
                 : "ALPHA_SERVICE_EPOCH"));
  out.presentation_clockfaces_zeroed = campaign_presentation_ready;

  out.presentation_gnss_ns = campaign_presentation_ready
      ? campaign_public_from_offset(
            instrument.gnss_ns, g_campaign_public_gnss_offset)
      : logical_instrument_gnss_ns;
  out.presentation_dwt_cycles = campaign_presentation_ready
      ? campaign_public_from_offset(
            instrument.dwt_cycles, g_campaign_public_dwt_offset)
      : logical_instrument_dwt_cycles;

  // Alpha's coherent snapshot carries public-origin-normalized OCXO values.
  // Use the matching measured/public offsets for the live presentation species;
  // the raw CounterLedger/PhaseLedger offsets remain campaign-record authority.
  out.presentation_ocxo1_ns = campaign_presentation_ready
      ? campaign_public_from_offset(
            instrument.ocxo1_ns, g_campaign_public_ocxo1_measured_offset)
      : logical_instrument_ocxo1_ns;
  out.presentation_ocxo2_ns = campaign_presentation_ready
      ? campaign_public_from_offset(
            instrument.ocxo2_ns, g_campaign_public_ocxo2_measured_offset)
      : logical_instrument_ocxo2_ns;
  out.presentation_count = (uint32_t)(
      out.presentation_gnss_ns / CLOCKS_BETA_NS_PER_SECOND);
  out.timeline_valid = instrument.valid &&
      out.presentation_gnss_ns != 0ULL &&
      out.presentation_dwt_cycles != 0ULL;
  out.ocxo_clockface_valid = instrument.valid &&
      out.presentation_ocxo1_ns != 0ULL &&
      out.presentation_ocxo2_ns != 0ULL;

  out.instrument_gnss_ns = logical_instrument_gnss_ns;
  out.instrument_dwt_cycles = logical_instrument_dwt_cycles;
  out.instrument_ocxo1_ns = logical_instrument_ocxo1_ns;
  out.instrument_ocxo2_ns = logical_instrument_ocxo2_ns;
  out.instrument_pps_sequence = instrument.last_pps_sequence;

  out.dwt_cycles_per_second = instrument.dwt_cycles_per_second;
  out.dwt_at_pps_vclock = instrument.dwt_at_pps_vclock;
  out.counter32_at_pps_vclock = instrument.counter32_at_pps_vclock;
  out.selected_reference_interval_cycles =
      instrument.selected_reference_interval_cycles;
  out.vclock_interval_cycles = instrument.vclock_interval_cycles;
  out.ocxo1_interval_cycles = instrument.ocxo1_interval_cycles;
  out.ocxo2_interval_cycles = instrument.ocxo2_interval_cycles;

  clocks_monitor_refresh_prediction_snapshots();
  clocks_monitor_raw_cycles_snapshot(
      out.raw_cycles,
      g_beta_monitor_pps_prediction,
      g_beta_monitor_vclock_prediction,
      g_beta_monitor_ocxo1_prediction,
      g_beta_monitor_ocxo2_prediction,
      g_pps_witness_diag.interrupt_delay,
      g_beta_monitor_vclock_forensics.interrupt_delay,
      g_beta_monitor_ocxo1_forensics.interrupt_delay,
      g_beta_monitor_ocxo2_forensics.interrupt_delay);
  clocks_monitor_stats_snapshot_from_instrument(out.stats, instrument);
  clocks_monitor_dac_snapshot(out.dac);
  clocks_recovery_state_snapshot(
      out.restore_state,
      instrument,
      logical_instrument_gnss_ns,
      logical_instrument_dwt_cycles,
      logical_instrument_ocxo1_ns,
      logical_instrument_ocxo2_ns);
}

FLASHMEM bool clocks_monitor_snapshot_take(
    uint32_t completed_second_sequence,
    clocks_monitor_snapshot_t* out) {
  if (!out) return false;
  memset(out, 0, sizeof(*out));
  clocks_monitor_live_snapshot_fill(out->live);

  if (g_monitor_campaign_record_pending &&
      g_monitor_campaign_record_sequence == completed_second_sequence) {
    out->campaign = g_monitor_campaign_record;
    memset(&g_monitor_campaign_record, 0,
           sizeof(g_monitor_campaign_record));
    g_monitor_campaign_record_pending = false;
    g_monitor_campaign_record_sequence = 0U;
    g_monitor_campaign_record_take_count++;
  }

  return true;
}

// ----------------------------------------------------------------------------
// CLOCKS_DAC_TICK -- retired automatic DAC/servo telemetry
// ----------------------------------------------------------------------------
//
// Servo/DAC facts ride in the typed CLOCKS handoff. CLOCKS no longer emits a
// separate DAC report or independent 1 Hz DAC_TICK publication.
//
// Keep this no-op helper so existing gate sites remain visually explicit:
// those sites are acknowledging DAC/servo transition points, not publishing
// a separate diagnostic stream.

static FLASHMEM void publish_dac_tick(const char*) {
  // Intentionally silent.
}

// ============================================================================
// clocks_beta_pps — PPS control probe or exact completed-row publication
// ============================================================================
//
// completed_pps_sequence == 0 is a control-only physical-PPS probe used for
// recording START boundaries, explicit ZERO, and RECOVER. A non-zero sequence is one
// immutable Alpha row released only after both post-PPS OCXO edges resolve the
// exact CounterLedger + PhaseLedger clockfaces for that same PPS.

void clocks_beta_pps(uint32_t completed_pps_sequence) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_ENTRY);
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_ENTRY);

  if (request_stop) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_STOP_GATE);
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    clocks_row_objection_clear();
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    flash_cut_clear_pending();
    // STOP closes the recording namespace only.  Servo mode and any lawful
    // always-on instrument control continue independently of campaigns.
    campaign_warmup_reset();
    publish_dac_tick("STOP_GATE");
    return;
  }

  if (request_start) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_START_ZERO_GATE);
    clocks_finish_start_accounting();
    publish_dac_tick("START_RECORDING_GATE");
    return;
  }

  if (request_zero) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_START_ZERO_GATE);
    (void)clocks_try_finish_pending_smartzero();
    publish_dac_tick("ZERO_GATE");
    return;
  }

  if (request_flash_cut) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE);
    campaign_flash_cut_commit_at_pps();
    publish_dac_tick("FLASH_CUT_GATE");
    return;
  }

  if (request_recover) {
    clocks_watchdog_disarm_campaign_publication();
    g_recover_lifecycle_pps_gate_count++;
    g_recover_lifecycle_last_gate_campaign_seconds = (uint32_t)campaign_seconds;
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_RECOVER_GATE);
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    request_zero = false;

    if (g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        !clocks_alpha_installed_smartzero_backing_epoch()) {
      g_recover_lifecycle_cold_bootstrap_wait_count++;
      recover_lifecycle_set_reason("recover_cold_bootstrap_wait_smartzero");
      publish_dac_tick("RECOVER_COLD_BOOTSTRAP_PENDING");
      return;
    }

    if (g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        !g_recover_lifecycle_cold_bootstrap_epoch_ready) {
      g_recover_lifecycle_cold_bootstrap_epoch_ready = true;
      g_recover_lifecycle_cold_bootstrap_ready_count++;
      recover_lifecycle_set_reason("recover_cold_bootstrap_epoch_ready");
    }

    const bool cold_bootstrap_commit =
        g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::COLD_BOOTSTRAP;

    if (cold_bootstrap_commit) {
      // SmartZero has already created and physically staggered the fresh OCXO
      // service grids for this firmware generation.  Starting a second
      // RECOVER-owned rephase here would stop those newly installed lanes while
      // request_recover suppresses ordinary Alpha rows, creating a circular
      // wait in which OCXO_PUBLIC_ORIGIN and STATIC_PREDICTION can never mature.
      // Treat the installed SmartZero-backed epoch as the completed cold-service
      // bootstrap and splice the durable campaign onto it directly below.
      g_recover_lifecycle_last_interrupt_service_rearm_ok = true;
      recover_lifecycle_set_reason("recover_cold_bootstrap_commit_ready");
    } else {
      clocks_alpha_ocxo_grid_rephase_status_t rephase_status =
          clocks_alpha_ocxo_grid_rephase_status(
              clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
      if (rephase_status == clocks_alpha_ocxo_grid_rephase_status_t::IDLE) {
        g_recover_lifecycle_interrupt_service_rearm_count++;
        g_recover_lifecycle_last_interrupt_service_rearm_ok =
            clocks_alpha_recover_rearm_interrupt_service();
        rephase_status = clocks_alpha_ocxo_grid_rephase_status(
            clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
      }
      if (rephase_status == clocks_alpha_ocxo_grid_rephase_status_t::PENDING) {
        recover_lifecycle_set_reason("recover_ocxo_grid_rephase_pending");
        publish_dac_tick("RECOVER_GRID_REPHASE_PENDING");
        return;
      }
      if (rephase_status != clocks_alpha_ocxo_grid_rephase_status_t::COMPLETE) {
        g_recover_lifecycle_last_interrupt_service_rearm_ok = false;
        g_recover_lifecycle_interrupt_service_rearm_failure_count++;
        clocks_alpha_ocxo_grid_rephase_acknowledge(
            clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
        recover_lifecycle_abort("recover_ocxo_grid_rephase_failed");
        publish_dac_tick("RECOVER_GRID_REPHASE_FAILED");
        return;
      }
    }

    if (cold_bootstrap_commit && g_campaign_restore_state.valid) {
      if (!clocks_recovery_commit_statistics_and_clockfaces(
              g_campaign_restore_state)) {
        g_campaign_restore_failure_count++;
        recover_lifecycle_abort("recover_state_commit_failed");
        publish_dac_tick("RECOVER_STATE_COMMIT_FAILED");
        return;
      }
      g_campaign_restore_count++;
    }

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_measured_gnss_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_measured_gnss_ticks_64        = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    // Capture recovered public offsets before Alpha cuts OCXO/CounterLedger
    // measurement custody.  After re-prime, the first PPS-sampled OCXO
    // CounterLedger rows are deliberately bookend seeds rather than valid
    // intervals, so offset recovery must use the last pre-cut raw clock
    // coordinate.
    campaign_public_offsets_reset_for_recover();

    // Live reattach must cut pre-recovery edge history so no interval bridges
    // the outage.  Cold bootstrap is already a fresh SmartZero epoch: its Alpha
    // ledgers, CounterLedger seeds, PhaseLedger state, and visible-origin basis
    // were created from scratch during the just-completed epoch commit.  Do not
    // erase that new state a second time.
    if (!cold_bootstrap_commit) {
      clocks_alpha_recover_reprime_ocxo_state();
    }
    interrupt_recover_reset_publication_custody();
    g_recover_lifecycle_gate_custody_reset_count++;
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
    // Alpha-owned PPB/TAU/Welfords remain live across warm recovery.

    request_recover = false;
    g_campaign_restore_state = clocks_recovery_restore_state_t{};
    flash_cut_clear_pending();
    if (cold_bootstrap_commit) {
      g_recover_lifecycle_cold_bootstrap_commit_count++;
    } else {
      clocks_alpha_ocxo_grid_rephase_acknowledge(
          clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
    }
    recover_lifecycle_complete_at_pps();
    campaign_warmup_begin(campaign_warmup_mode_t::RECOVER);
    publish_dac_tick("RECOVER_GATE");
    return;
  }

  if (g_monitor_restore_requested) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_RECOVER_GATE);
    if (!clocks_alpha_installed_smartzero_backing_epoch() ||
        clocks_alpha_epoch_install_in_progress()) {
      publish_dac_tick("MONITOR_RESTORE_WAIT_EPOCH");
      return;
    }

    if (!clocks_recovery_commit_statistics_and_clockfaces(
            g_monitor_restore_state)) {
      g_monitor_restore_requested = false;
      g_monitor_restore_failure_count++;
      clocks_apply_servo_mode_now(servo_mode_t::OFF);
      publish_dac_tick("MONITOR_RESTORE_COMMIT_FAILED");
      return;
    }

    g_monitor_restore_requested = false;
    g_monitor_restore_commit_count++;
    g_monitor_restore_state = clocks_recovery_restore_state_t{};
    clocks_row_objection_clear();
    publish_dac_tick("MONITOR_RESTORE_COMMITTED");
    return;
  }

  if (clocks_campaign_recovery_lifecycle_active()) {
    g_recover_lifecycle_stale_gate_count++;
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE);
    publish_dac_tick("RECOVERING_NO_REQUEST_GATE");
    return;
  }

  if (clocks_watchdog_publication_blocked()) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_WATCHDOG_GATE);
    publish_dac_tick("WATCHDOG_GATE");
    return;
  }
  // Servo mode is instrument control state, not campaign state.  Commit a live
  // mode change before the campaign publication gate so OFF/MEAN/TOTAL/NOW take
  // effect while the instrument is idle.
  clocks_commit_pending_servo_mode_change();

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    // Idle rows share the same exclusion contract. Alpha already declined any
    // Welford/TAU mutation; Beta must also prevent NOW/servo control from using
    // the excluded second. There is no campaign row to serialize while idle.
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_NOT_STARTED_GATE);
    if (clocks_row_science_eligible(completed_pps_sequence)) {
      clocks_idle_instrument_servo(completed_pps_sequence);
    } else {
      servo_input_diag_reset(g_servo_input_ocxo1);
      servo_input_diag_reset(g_servo_input_ocxo2);
    }
    clocks_row_objection_clear();
    publish_dac_tick("NOT_STARTED_GATE");
    return;
  }

  // Sequence zero is never a scientific row.  START/ZERO consumed it above;
  // any other zero-sequence call is intentionally silent.
  if (completed_pps_sequence == 0U) return;

  // Final Beta custody gate: acquire the current Alpha VCLOCK forensic row
  // before campaign identity advances.  The forensic event and the canonical
  // PPS/VCLOCK globals are two views of the same authored bookend; accepting a
  // mismatch would publish the previous diagnostic endpoint beside the current
  // campaign record interval.
  clocks_alpha_lane_forensics_t& vclock_forensics =
      g_beta_pps_vclock_forensics;
  vclock_forensics = clocks_alpha_lane_forensics_t{};
  const bool vclock_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK,
                                  &vclock_forensics);
  // Form the complete current-row forensic tuple before the START prologue
  // decides whether this candidate remains private PPS0 or becomes public PPS1.
  // The old ordering ran the warmup court first, so it could suppress the very
  // candidate needed to seed its observed bookends.
  clocks_alpha_lane_forensics_t& ocxo1_forensics = g_beta_pps_ocxo1_forensics;
  clocks_alpha_lane_forensics_t& ocxo2_forensics = g_beta_pps_ocxo2_forensics;
  ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  ocxo2_forensics = clocks_alpha_lane_forensics_t{};

  const bool ocxo1_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &ocxo1_forensics);
  const bool ocxo2_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &ocxo2_forensics);

  // The completed row is one immutable scientific identity.  Snapshot every
  // Alpha authority once, then prove that the PPS/VCLOCK bookend, both
  // PPS-synchronous CounterLedgers, both PhaseLedger suffixes, and both direct
  // adjacent OCXO edge intervals all belong to completed_pps_sequence.
  clocks_pps_vclock_edge_forensics_t& pps_vclock_edge_forensics =
      g_beta_pps_vclock_edge_forensics;
  pps_vclock_edge_forensics = clocks_pps_vclock_edge_forensics_t{};
  const bool pps_vclock_edge_forensics_valid =
      clocks_alpha_pps_vclock_edge_forensics(&pps_vclock_edge_forensics);

  g_beta_ocxo1_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo2_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  const bool ocxo1_counterledger_ok =
      clocks_alpha_ocxo_counterledger_snapshot(
          time_clock_id_t::OCXO1, &g_beta_ocxo1_counterledger_row);
  const bool ocxo2_counterledger_ok =
      clocks_alpha_ocxo_counterledger_snapshot(
          time_clock_id_t::OCXO2, &g_beta_ocxo2_counterledger_row);
  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo1_counterledger =
      g_beta_ocxo1_counterledger_row;
  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo2_counterledger =
      g_beta_ocxo2_counterledger_row;

  const bool vclock_sequence_exact =
      pps_vclock_edge_forensics_valid &&
      pps_vclock_edge_forensics.sequence == completed_pps_sequence &&
      pps_vclock_edge_forensics.authority_dwt_at_edge ==
          g_dwt_at_pps_vclock &&
      pps_vclock_edge_forensics.counter32_at_edge ==
          g_counter32_at_pps_vclock &&
      vclock_forensics_valid &&
      vclock_forensics.last_event_counter32 == g_counter32_at_pps_vclock &&
      vclock_forensics.last_event_dwt == g_dwt_at_pps_vclock;
  if (!vclock_sequence_exact) {
    clocks_watchdog_anomaly(
        "beta_completed_vclock_custody",
        completed_pps_sequence,
        pps_vclock_edge_forensics_valid
            ? pps_vclock_edge_forensics.sequence
            : 0U,
        vclock_forensics_valid ? vclock_forensics.last_event_counter32 : 0U,
        vclock_forensics_valid ? vclock_forensics.last_event_dwt : 0U);
    return;
  }

  const bool ocxo1_sequence_exact =
      ocxo1_counterledger_ok &&
      clocks_beta_completed_ocxo_forensics_ready(
          ocxo1_forensics_valid, ocxo1_forensics,
          ocxo1_counterledger, completed_pps_sequence);
  const bool ocxo2_sequence_exact =
      ocxo2_counterledger_ok &&
      clocks_beta_completed_ocxo_forensics_ready(
          ocxo2_forensics_valid, ocxo2_forensics,
          ocxo2_counterledger, completed_pps_sequence);
  if (!ocxo1_sequence_exact || !ocxo2_sequence_exact) {
    const uint32_t mismatch_mask =
        (ocxo1_sequence_exact ? 0U : 1U) |
        (ocxo2_sequence_exact ? 0U : 2U);
    clocks_watchdog_anomaly(
        "beta_completed_pps_custody",
        completed_pps_sequence,
        ocxo1_counterledger.pps_sequence,
        ocxo2_counterledger.pps_sequence,
        mismatch_mask);
    return;
  }

  // RECOVER resumes an existing public timeline, so its reattachment state is
  // observational here and candidate identities continue to be emitted.  Run
  // that court only after exact completed-row custody has been proven.
  (void)recover_reattach_should_hold();

  // START consumes this exact candidate tuple privately while campaign_seconds
  // remains zero. The helper seeds PPS0 from the normal observed-edge inputs and
  // returns true until a following mature tuple can be released as public PPS1.
  // Welford, servo, watchdog, payload construction, and publication remain below
  // this boundary and therefore cannot consume private startup evidence.
  if (campaign_warmup_consume_one_candidate_record(
          vclock_forensics_valid, vclock_forensics,
          ocxo1_forensics_valid, ocxo1_forensics,
          ocxo2_forensics_valid, ocxo2_forensics)) {
    clocks_row_objection_clear();
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_WARMUP_GATE);
    publish_dac_tick("WARMUP_GATE");
    return;
  }

  g_campaign_record_candidate_count++;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_CANDIDATE);

  // ── Per-second campaign work ──
  campaign_seconds++;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_PER_SECOND);

  const uint32_t public_count = (uint32_t)campaign_seconds;
  const uint64_t public_gnss_ns = campaign_public_gnss_ns(public_count);
  const uint64_t public_dwt_total = campaign_public_dwt_total();

  // RECOVER public presentation must be aligned before any shadow ledger,
  // Welford gate, science row, or payload field observes the new local epoch.
  recover_continuity_align_if_pending(public_count, public_gnss_ns);

  dwt_cycle_count_total = public_dwt_total;
  gnss_raw_64           = public_gnss_ns / 100ull;
  ocxo1_measured_gnss_ticks_64        = campaign_public_ocxo1_ns() / 100ull;
  ocxo2_measured_gnss_ticks_64        = campaign_public_ocxo2_ns() / 100ull;

  // Alpha already advanced the boot-lifetime instrument statistics for this
  // completed row before entering Beta.  The remaining work is campaign
  // presentation, candidate disposition, and servo selection.

  clocks_alpha_ocxo_pps_projection_snapshot_t& ocxo1_pps_projection =
      g_beta_pps_ocxo1_projection;
  clocks_alpha_ocxo_pps_projection_snapshot_t& ocxo2_pps_projection =
      g_beta_pps_ocxo2_projection;
  ocxo1_pps_projection = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  ocxo2_pps_projection = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  const bool ocxo1_pps_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO1,
                                                &ocxo1_pps_projection);
  const bool ocxo2_pps_projection_ok =
      clocks_alpha_ocxo_pps_projection_snapshot(time_clock_id_t::OCXO2,
                                                &ocxo2_pps_projection);

  // ── Public PPS-edge clock tuple ──
  //
  // Alpha still exposes the legacy measured-GNSS OCXO row values as a
  // diagnostic side-channel.  Beta now renders the published OCXO ns tuple
  // from the same canonical Delta Cycles totals that feed pps_residual,
  // Welford, servo, and stats, so authoritative fields tell one story.
  // The Alpha PPS-projection snapshots remain courtroom collateral.
  // public_gnss_ns/public_dwt_total and any RECOVER presentation transform
  // were captured before per-second consumers above.

  const uint64_t public_ocxo1_measured_ns = campaign_public_ocxo1_measured_ns();
  const uint64_t public_ocxo2_measured_ns = campaign_public_ocxo2_measured_ns();

  const bool ocxo1_pps_projected_valid =
      ocxo1_pps_projection_ok && ocxo1_pps_projection.valid &&
      ocxo1_pps_projection.projected_ocxo_ns_at_pps != 0;
  const bool ocxo2_pps_projected_valid =
      ocxo2_pps_projection_ok && ocxo2_pps_projection.valid &&
      ocxo2_pps_projection.projected_ocxo_ns_at_pps != 0;

  // Delta residuals are same-row aligned. Exact-row PhaseLedger completion
  // has already been proven above, so Beta compares the current PPS/VCLOCK
  // interval to the current adjacent OCXO intervals. Capture the reference
  // for this public row and use that same identity for OCXO science.
  const delta_residual_reference_t delta_reference_this_row =
      delta_residual_capture_vclock_reference(public_count,
                                              vclock_forensics_valid,
                                              vclock_forensics);
  const delta_residual_reference_t delta_reference_for_ocxo =
      delta_reference_this_row;

  // ── Delta Cycles canonical science ──
  //
  // Observed DWT-at-edge owns the canonical residual: each OCXO observed
  // interval is subtracted from the same-row observed PPS/VCLOCK DWT-edge
  // interval. Projected-GNSS comparison remains under science.traditional_*; it does
  // not feed Welford.  Published OCXO TAU/PPB are re-authored below as simple
  // campaign public-ledger ratios.
  clock_science_row_t& ocxo1_science = g_beta_ocxo1_science_row;
  clock_science_row_t& ocxo2_science = g_beta_ocxo2_science_row;
  g_beta_row_science_totals_o1_before = g_ocxo_science_totals_ocxo1;
  g_beta_row_science_totals_o2_before = g_ocxo_science_totals_ocxo2;
  clock_science_build_ocxo(ocxo1_science,
                           time_clock_id_t::OCXO1,
                           public_count,
                           public_gnss_ns,
                           delta_reference_for_ocxo,
                           ocxo1_forensics_valid,
                           ocxo1_forensics,
                           g_ocxo_science_totals_ocxo1);
  clock_science_build_ocxo(ocxo2_science,
                           time_clock_id_t::OCXO2,
                           public_count,
                           public_gnss_ns,
                           delta_reference_for_ocxo,
                           ocxo2_forensics_valid,
                           ocxo2_forensics,
                           g_ocxo_science_totals_ocxo2);

  clocks_alpha_tau_snapshot_t& ocxo1_alpha_tau = g_beta_pps_ocxo1_alpha_tau;
  clocks_alpha_tau_snapshot_t& ocxo2_alpha_tau = g_beta_pps_ocxo2_alpha_tau;
  ocxo1_alpha_tau = clocks_alpha_tau_snapshot_t{};
  ocxo2_alpha_tau = clocks_alpha_tau_snapshot_t{};
  const bool ocxo1_alpha_tau_ok = clocks_alpha_ocxo_tau_snapshot(
      time_clock_id_t::OCXO1, &ocxo1_alpha_tau);
  const bool ocxo2_alpha_tau_ok = clocks_alpha_ocxo_tau_snapshot(
      time_clock_id_t::OCXO2, &ocxo2_alpha_tau);

  // RECOVER degraded release keeps public GNSS/DWT/VCLOCK time moving after
  // the finite OCXO reattachment timeout.  OCXO science/Welford/servo input
  // stays inert until Alpha proves fresh post-recovery OCXO custody.
  const bool recover_degraded_science_hold =
      recover_reattach_degraded_science_hold_active();

  const uint64_t public_ocxo1_counterledger_ns =
      campaign_public_counterledger_ocxo1_ns();
  const uint64_t public_ocxo2_counterledger_ns =
      campaign_public_counterledger_ocxo2_ns();
  const uint64_t public_ocxo1_ns = clocks_ocxo_counterledger_mode_enabled()
      ? public_ocxo1_counterledger_ns
      : science_render_public_clock_ns(
            public_gnss_ns, ocxo1_science, public_ocxo1_measured_ns);
  const uint64_t public_ocxo2_ns = clocks_ocxo_counterledger_mode_enabled()
      ? public_ocxo2_counterledger_ns
      : science_render_public_clock_ns(
            public_gnss_ns, ocxo2_science, public_ocxo2_measured_ns);

  const bool ocxo1_clockface_valid = clocks_ocxo_counterledger_mode_enabled()
      ? clocks_beta_counterledger_clockface_ready(ocxo1_counterledger,
                                                   completed_pps_sequence,
                                                   public_ocxo1_ns)
      : (public_ocxo1_ns != 0ULL && ocxo1_pps_projected_valid);
  const bool ocxo2_clockface_valid = clocks_ocxo_counterledger_mode_enabled()
      ? clocks_beta_counterledger_clockface_ready(ocxo2_counterledger,
                                                   completed_pps_sequence,
                                                   public_ocxo2_ns)
      : (public_ocxo2_ns != 0ULL && ocxo2_pps_projected_valid);

  clock_science_apply_counterledger_row(ocxo1_science,
                                        ocxo1_counterledger,
                                        public_gnss_ns);
  clock_science_apply_counterledger_row(ocxo2_science,
                                        ocxo2_counterledger,
                                        public_gnss_ns);

  // Published OCXO TAU/PPB are campaign clockface ratios, not physical-period
  // Delta ratios: public_ocxo_ns / public_gnss_ns.  RECOVER may have just
  // applied a one-time presentation transform so this ratio continues from the
  // Pi-projected campaign ledger instead of a fresh reattachment intercept.
  // The one-second residuals and Welfords above remain Delta/PhaseLedger
  // evidence surfaces.
  clock_science_apply_campaign_public_ratio(ocxo1_science,
                                            public_gnss_ns,
                                            public_ocxo1_ns,
                                            public_count);
  clock_science_apply_campaign_public_ratio(ocxo2_science,
                                            public_gnss_ns,
                                            public_ocxo2_ns,
                                            public_count);

  clock_science_apply_alpha_tau(ocxo1_science,
                                ocxo1_alpha_tau_ok,
                                ocxo1_alpha_tau);
  clock_science_apply_alpha_tau(ocxo2_science,
                                ocxo2_alpha_tau_ok,
                                ocxo2_alpha_tau);

  if (recover_degraded_science_hold) {
    recover_reattach_apply_degraded_science_hold(ocxo1_science);
    recover_reattach_apply_degraded_science_hold(ocxo2_science);
  }

  const bool recover_science_quarantine_applied =
      science_residual_quarantine_apply(public_count,
                                        ocxo1_science,
                                        ocxo2_science);

  const bool recover_timeline_ready =
      public_gnss_ns != 0ULL && public_dwt_total != 0ULL;
  const bool recover_clockface_ready =
      ocxo1_clockface_valid && ocxo2_clockface_valid;
  const bool recover_science_ready =
      ocxo1_science.science_worthy &&
      ocxo1_science.antecedents_complete &&
      ocxo2_science.science_worthy &&
      ocxo2_science.antecedents_complete;
  const bool recover_transition_active =
      g_recover_reattach_active ||
      g_recover_reattach_degraded_active ||
      recover_science_quarantine_applied ||
      g_science_residual_quarantine_remaining != 0U ||
      g_recover_continuity_align_pending;

  ocxo1_measured_gnss_ticks_64 = public_ocxo1_ns / 100ULL;
  ocxo2_measured_gnss_ticks_64 = public_ocxo2_ns / 100ULL;

  g_delta_previous_vclock_reference = delta_reference_this_row;

  const pps_interval_residuals_t pps_residuals =
      pps_interval_residuals_update(public_count,
                                    ocxo1_science,
                                    ocxo2_science);

  // Preserve the pre-publication court as compact evidence. Every structurally
  // coherent candidate becomes a durable TIMEBASE row; this court decides only
  // whether science/control consumers may use it.
  const uint32_t ocxo_science_invalid_mask =
      clocks_beta_public_ocxo_science_invalid_mask(
          public_count,
          public_ocxo1_ns,
          public_ocxo2_ns,
          ocxo1_science,
          ocxo2_science);

  clocks_row_objection_record_t candidate_objection{};
  const bool external_row_objection =
      clocks_row_objection_consume(candidate_objection);
  if (!external_row_objection && ocxo_science_invalid_mask != 0U) {
    candidate_objection.source = clocks_row_objection_source_t::BETA;
    candidate_objection.reason =
        clocks_row_objection_reason_t::BETA_OCXO_SCIENCE_CUSTODY;
    candidate_objection.lane = ocxo_science_invalid_mask;
    candidate_objection.lane_mask = ocxo_science_invalid_mask;
    candidate_objection.objection_count = 1U;
    candidate_objection.detail0 = ocxo_science_invalid_mask;
  } else if (external_row_objection && ocxo_science_invalid_mask != 0U) {
    candidate_objection.lane_mask |= ocxo_science_invalid_mask;
    candidate_objection.objection_count++;
  }
  const bool candidate_science_excluded =
      candidate_objection.reason != clocks_row_objection_reason_t::NONE ||
      ocxo_science_invalid_mask != 0U;

  const bool candidate_math_permitted = !candidate_science_excluded;

  if (candidate_science_excluded) {
    // Beta science totals were tentatively formed to preserve complete row
    // testimony. Restore them unconditionally; no mode may authorize
    // contamination. Alpha Welford/TAU mutation was already prevented upstream.
    g_ocxo_science_totals_ocxo1 = g_beta_row_science_totals_o1_before;
    g_ocxo_science_totals_ocxo2 = g_beta_row_science_totals_o2_before;
    ocxo1_science.science_worthy = false;
    ocxo2_science.science_worthy = false;
    g_row_objection_last_public_count = public_count;
    g_row_objection_last = candidate_objection;
    clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                   g_clocks_feature_science_residuals,
                                   system_feature_status_t::ANOMALY,
                                   true);
  }

  // Four local observed-interval snapshots feed raw_cycles.  The prior
  // completed interval is the prediction field; no duplicate prediction or
  // cycle-residual diagnostic object is serialized.
  g_beta_pps_cycle_prediction = prediction_snapshot_for_pps();
  g_beta_vclock_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::VCLOCK);
  g_beta_ocxo1_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO1);
  g_beta_ocxo2_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO2);

  g_beta_instrument_stats = clocks_instrument_stats_snapshot_t{};
  bool instrument_stats_ready =
      clocks_alpha_instrument_stats_snapshot(&g_beta_instrument_stats) &&
      g_beta_instrument_stats.valid &&
      g_beta_instrument_stats.completed_row_coherent &&
      g_beta_instrument_stats.last_pps_sequence == completed_pps_sequence;

  if (candidate_math_permitted) {
    // Welford/TAU/PPB are Alpha-owned and were updated before this call.
    // Beta reads the mature populations below for servo compatibility only.

    const clocks_instrument_frequency_snapshot_t& ocxo1_frequency =
        g_beta_instrument_stats.ocxo1_frequency;
    const clocks_instrument_frequency_snapshot_t& ocxo2_frequency =
        g_beta_instrument_stats.ocxo2_frequency;
    const bool ocxo1_total_slope_valid =
        instrument_stats_ready &&
        pps_residuals.ocxo1_valid &&
        ocxo1_frequency.valid;
    const bool ocxo2_total_slope_valid =
        instrument_stats_ready &&
        pps_residuals.ocxo2_valid &&
        ocxo2_frequency.valid;

    // Populate OCXO1 servo diagnostics directly.  This deliberately avoids a
    // reference-heavy helper boundary on the completed-row hot path.
    g_servo_input_ocxo1.pps_residual_valid = pps_residuals.ocxo1_valid;
    g_servo_input_ocxo1.pps_gnss_interval_ns = pps_residuals.ocxo1_valid
        ? pps_residuals.gnss_interval_ns
        : 0ULL;
    g_servo_input_ocxo1.pps_clock_interval_ns = pps_residuals.ocxo1_valid
        ? pps_residuals.ocxo1_interval_ns
        : 0ULL;
    g_servo_input_ocxo1.pps_fast_residual_ns = pps_residuals.ocxo1_valid
        ? pps_residuals.ocxo1_fast_residual_ns
        : 0LL;

    g_servo_input_ocxo1.mean_welford_n = welford_ocxo1.n;
    g_servo_input_ocxo1.mean_welford_ppb =
        (welford_ocxo1.n > 0) ? welford_ocxo1.mean : 0.0;
    g_servo_input_ocxo1.mean_input_valid =
        pps_residuals.ocxo1_valid &&
        (welford_ocxo1.n >= SERVO_MIN_SAMPLES);

    g_servo_input_ocxo1.total_tau =
        ocxo1_total_slope_valid ? ocxo1_frequency.tau : 1.0;
    g_servo_input_ocxo1.total_ppb =
        ocxo1_total_slope_valid ? ocxo1_frequency.ppb : 0.0;
    g_servo_input_ocxo1.total_basis = SERVO_TOTAL_BASIS_INSTRUMENT;
    g_servo_input_ocxo1.total_population_seconds =
        ocxo1_total_slope_valid
            ? ocxo1_frequency.sample_count
            : 0ULL;
    g_servo_input_ocxo1.total_input_valid =
        ocxo1_total_slope_valid &&
        ocxo1_frequency.sample_count >= SERVO_MIN_SAMPLES;

    // A one-second residual in ns is numerically ppb over a one-second gate.
    g_servo_input_ocxo1.now_ppb = pps_residuals.ocxo1_valid
        ? ocxo1_science.fast_residual_ns_exact
        : 0.0;
    g_servo_input_ocxo1.now_input_valid = pps_residuals.ocxo1_valid;

    g_servo_input_ocxo1.selected_residual_ns = 0;
    if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
      g_servo_input_ocxo1.selected_source =
          SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD;
      g_servo_input_ocxo1.selected_input_valid =
          g_servo_input_ocxo1.mean_input_valid;
      g_servo_input_ocxo1.selected_input_ppb =
          g_servo_input_ocxo1.mean_input_valid
              ? g_servo_input_ocxo1.mean_welford_ppb
              : 0.0;
      g_servo_input_ocxo1.selected_residual_ns =
          (int64_t)g_servo_input_ocxo1.selected_input_ppb;
    } else if (calibrate_ocxo_mode == servo_mode_t::TOTAL) {
      g_servo_input_ocxo1.selected_source =
          SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU;
      g_servo_input_ocxo1.selected_input_valid =
          g_servo_input_ocxo1.total_input_valid &&
          g_servo_input_ocxo1.now_input_valid;
      g_servo_input_ocxo1.selected_input_ppb =
          g_servo_input_ocxo1.total_input_valid
              ? g_servo_input_ocxo1.total_ppb
              : 0.0;
      g_servo_input_ocxo1.selected_residual_ns =
          (int64_t)g_servo_input_ocxo1.selected_input_ppb;
    } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
      g_servo_input_ocxo1.selected_source =
          SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL;
      g_servo_input_ocxo1.selected_input_valid =
          g_servo_input_ocxo1.now_input_valid;
      g_servo_input_ocxo1.selected_input_ppb =
          g_servo_input_ocxo1.now_input_valid
              ? g_servo_input_ocxo1.now_ppb
              : 0.0;
      g_servo_input_ocxo1.selected_residual_ns =
          pps_residuals.ocxo1_valid
              ? pps_residuals.ocxo1_fast_residual_ns
              : 0LL;
    } else {
      g_servo_input_ocxo1.selected_source = SERVO_INPUT_SOURCE_NONE;
      g_servo_input_ocxo1.selected_input_valid = false;
      g_servo_input_ocxo1.selected_input_ppb = 0.0;
      g_servo_input_ocxo1.selected_residual_ns = 0;
    }

    // Populate OCXO2 servo diagnostics directly for the same reason.
    g_servo_input_ocxo2.pps_residual_valid = pps_residuals.ocxo2_valid;
    g_servo_input_ocxo2.pps_gnss_interval_ns = pps_residuals.ocxo2_valid
        ? pps_residuals.gnss_interval_ns
        : 0ULL;
    g_servo_input_ocxo2.pps_clock_interval_ns = pps_residuals.ocxo2_valid
        ? pps_residuals.ocxo2_interval_ns
        : 0ULL;
    g_servo_input_ocxo2.pps_fast_residual_ns = pps_residuals.ocxo2_valid
        ? pps_residuals.ocxo2_fast_residual_ns
        : 0LL;

    g_servo_input_ocxo2.mean_welford_n = welford_ocxo2.n;
    g_servo_input_ocxo2.mean_welford_ppb =
        (welford_ocxo2.n > 0) ? welford_ocxo2.mean : 0.0;
    g_servo_input_ocxo2.mean_input_valid =
        pps_residuals.ocxo2_valid &&
        (welford_ocxo2.n >= SERVO_MIN_SAMPLES);

    g_servo_input_ocxo2.total_tau =
        ocxo2_total_slope_valid ? ocxo2_frequency.tau : 1.0;
    g_servo_input_ocxo2.total_ppb =
        ocxo2_total_slope_valid ? ocxo2_frequency.ppb : 0.0;
    g_servo_input_ocxo2.total_basis = SERVO_TOTAL_BASIS_INSTRUMENT;
    g_servo_input_ocxo2.total_population_seconds =
        ocxo2_total_slope_valid
            ? ocxo2_frequency.sample_count
            : 0ULL;
    g_servo_input_ocxo2.total_input_valid =
        ocxo2_total_slope_valid &&
        ocxo2_frequency.sample_count >= SERVO_MIN_SAMPLES;

    g_servo_input_ocxo2.now_ppb = pps_residuals.ocxo2_valid
        ? ocxo2_science.fast_residual_ns_exact
        : 0.0;
    g_servo_input_ocxo2.now_input_valid = pps_residuals.ocxo2_valid;

    g_servo_input_ocxo2.selected_residual_ns = 0;
    if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
      g_servo_input_ocxo2.selected_source =
          SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD;
      g_servo_input_ocxo2.selected_input_valid =
          g_servo_input_ocxo2.mean_input_valid;
      g_servo_input_ocxo2.selected_input_ppb =
          g_servo_input_ocxo2.mean_input_valid
              ? g_servo_input_ocxo2.mean_welford_ppb
              : 0.0;
      g_servo_input_ocxo2.selected_residual_ns =
          (int64_t)g_servo_input_ocxo2.selected_input_ppb;
    } else if (calibrate_ocxo_mode == servo_mode_t::TOTAL) {
      g_servo_input_ocxo2.selected_source =
          SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU;
      g_servo_input_ocxo2.selected_input_valid =
          g_servo_input_ocxo2.total_input_valid &&
          g_servo_input_ocxo2.now_input_valid;
      g_servo_input_ocxo2.selected_input_ppb =
          g_servo_input_ocxo2.total_input_valid
              ? g_servo_input_ocxo2.total_ppb
              : 0.0;
      g_servo_input_ocxo2.selected_residual_ns =
          (int64_t)g_servo_input_ocxo2.selected_input_ppb;
    } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
      g_servo_input_ocxo2.selected_source =
          SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL;
      g_servo_input_ocxo2.selected_input_valid =
          g_servo_input_ocxo2.now_input_valid;
      g_servo_input_ocxo2.selected_input_ppb =
          g_servo_input_ocxo2.now_input_valid
              ? g_servo_input_ocxo2.now_ppb
              : 0.0;
      g_servo_input_ocxo2.selected_residual_ns =
          pps_residuals.ocxo2_valid
              ? pps_residuals.ocxo2_fast_residual_ns
              : 0LL;
    } else {
      g_servo_input_ocxo2.selected_source = SERVO_INPUT_SOURCE_NONE;
      g_servo_input_ocxo2.selected_input_valid = false;
      g_servo_input_ocxo2.selected_input_ppb = 0.0;
      g_servo_input_ocxo2.selected_residual_ns = 0;
    }

    campaign_record_stage(CAMPAIGN_RECORD_STAGE_WELFORD);
    g_servo_control_second = instrument_stats_ready
        ? (uint64_t)g_beta_instrument_stats.update_count
        : (uint64_t)completed_pps_sequence;
    ocxo_calibration_servo();
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_SERVO);

    // DAC populations are sampled by Alpha with the same completed row.
  } else {
    servo_input_diag_reset(g_servo_input_ocxo1);
    servo_input_diag_reset(g_servo_input_ocxo2);
  }
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_DAC_WELFORD);

  publish_dac_tick("STARTED");

  // Freeze one immutable completed campaign record for SYSTEM.  Beta owns the
  // lifecycle/science verdicts and values; SYSTEM alone decides how those facts
  // are named and nested in MONITOR_FRAGMENT.
  g_campaign_record_last_public_count = public_count;
  g_campaign_record_last_public_gnss_ns = public_gnss_ns;
  g_campaign_record_last_public_dwt_total = public_dwt_total;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_BEGIN);
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_BUILD);

  // Never overwrite an unconsumed record. Transport pressure must become an
  // explicit structural failure rather than silent campaign data loss.
  if (g_monitor_campaign_record_pending) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_BACKLOG);
    g_monitor_campaign_record_backlog_count++;
    clocks_watchdog_anomaly("monitor_campaign_record_backlog",
                            g_monitor_campaign_record_sequence,
                            completed_pps_sequence,
                            g_monitor_campaign_record_backlog_count,
                            public_count);
    return;
  }

  clocks_monitor_campaign_snapshot_t& record = g_monitor_campaign_record;
  memset(&record, 0, sizeof(record));
  record.present = true;
  record.completed_second_sequence = completed_pps_sequence;
  clocks_monitor_copy_text(record.campaign,
                           sizeof(record.campaign),
                           campaign_name);
  clocks_monitor_copy_text(record.campaign_state,
                           sizeof(record.campaign_state),
                           clocks_campaign_state_name(campaign_state));
  record.public_count = public_count;
  record.gnss_ns = public_gnss_ns;
  clocks_monitor_copy_text(record.disposition,
                           sizeof(record.disposition),
                           candidate_science_excluded
                               ? "SCIENCE_EXCLUDE"
                               : "ACCEPT");
  clocks_monitor_copy_text(record.servo_mode,
                           sizeof(record.servo_mode),
                           servo_mode_str(calibrate_ocxo_mode));
  record.timeline_valid = recover_timeline_ready;
  record.ocxo_clockface_valid = recover_clockface_ready;
  record.ocxo_science_valid = recover_science_ready &&
                              !candidate_science_excluded;
  record.science_eligible = !candidate_science_excluded;
  record.control_eligible = !candidate_science_excluded;
  record.persist = true;
  record.science_excluded = candidate_science_excluded;

  if (candidate_science_excluded) {
    record.rejection.present = true;
    record.rejection.reason_code = (uint32_t)candidate_objection.reason;
    clocks_monitor_copy_text(
        record.rejection.reason_name,
        sizeof(record.rejection.reason_name),
        clocks_row_objection_reason_name(candidate_objection.reason));
    clocks_row_objection_format_reason(
        record.rejection.reason,
        sizeof(record.rejection.reason),
        candidate_objection);
    clocks_monitor_copy_text(
        record.rejection.source,
        sizeof(record.rejection.source),
        clocks_row_objection_source_name(candidate_objection.source));
    record.rejection.lane = candidate_objection.lane;
    record.rejection.detail0 = candidate_objection.detail0;
    record.rejection.detail1 = candidate_objection.detail1;
    record.rejection.detail2 = candidate_objection.detail2;
    record.rejection.detail3 = candidate_objection.detail3;
    record.rejection.reject_mask =
        ocxo_science_invalid_mask | candidate_objection.lane_mask;
    record.rejection.objection_count =
        candidate_objection.objection_count != 0U
            ? candidate_objection.objection_count
            : 1U;
  }

  // Recovery testimony remains conditional so ordinary steady-state records do
  // not carry an idle recovery transcript forever.
  const bool recovery_row =
      recover_transition_active ||
      g_recover_reattach_active ||
      g_recover_reattach_degraded_active ||
      recover_degraded_science_hold ||
      recover_science_quarantine_applied ||
      g_science_residual_quarantine_remaining != 0U ||
      g_recover_reattach_stalled ||
      g_recover_continuity_align_last_public_count == public_count ||
      g_recover_reattach_last_release_public_count == public_count;
  if (recovery_row) {
    record.recovery.present = true;
    record.recovery.generation = g_recover_request_count;
    record.recovery.transition_active = recover_transition_active;
    record.recovery.timeline_ready = recover_timeline_ready;
    record.recovery.clockface_ready = recover_clockface_ready;
    record.recovery.science_ready = recover_science_ready;
    record.recovery.ocxo1_clockface_ready = ocxo1_clockface_valid;
    record.recovery.ocxo2_clockface_ready = ocxo2_clockface_valid;
    record.recovery.ocxo1_science_ready =
        ocxo1_science.valid && ocxo1_science.antecedents_complete;
    record.recovery.ocxo2_science_ready =
        ocxo2_science.valid && ocxo2_science.antecedents_complete;
    record.recovery.science_quarantine_active =
        recover_science_quarantine_applied ||
        g_science_residual_quarantine_remaining != 0U;
    record.recovery.science_quarantine_remaining =
        g_science_residual_quarantine_remaining;
    record.recovery.no_progress_rows =
        g_recover_reattach_no_progress_row_count;
    record.recovery.last_progress_public_count =
        g_recover_reattach_last_progress_public_count;
    record.recovery.reattach_active = g_recover_reattach_active;
    record.recovery.degraded_active = g_recover_reattach_degraded_active;
    record.recovery.degraded_science_hold = recover_degraded_science_hold;
    record.recovery.reattach_stalled = g_recover_reattach_stalled;
    clocks_monitor_copy_text(
        record.recovery.reattach_reason,
        sizeof(record.recovery.reattach_reason),
        g_recover_reattach_last_reason);
  }

  record.dwt_cycle_count_total = public_dwt_total;
  record.dwt_cycles_between_pps_vclock =
      (uint32_t)g_dwt_cycles_between_pps_vclock;
  record.dwt_at_pps_vclock = (uint32_t)g_dwt_at_pps_vclock;
  record.counter32_at_pps_vclock = (uint32_t)g_counter32_at_pps_vclock;
  clocks_monitor_raw_cycles_snapshot(
      record.raw_cycles,
      g_beta_pps_cycle_prediction,
      g_beta_vclock_cycle_prediction,
      g_beta_ocxo1_cycle_prediction,
      g_beta_ocxo2_cycle_prediction,
      g_pps_witness_diag.interrupt_delay,
      g_beta_pps_vclock_forensics.interrupt_delay,
      g_beta_pps_ocxo1_forensics.interrupt_delay,
      g_beta_pps_ocxo2_forensics.interrupt_delay);

  record.ocxo1_ns = public_ocxo1_ns;
  record.ocxo1_clockface_valid = ocxo1_clockface_valid;
  clocks_monitor_science_snapshot_from_row(
      record.ocxo1_science, ocxo1_science);
  record.ocxo2_ns = public_ocxo2_ns;
  record.ocxo2_clockface_valid = ocxo2_clockface_valid;
  clocks_monitor_science_snapshot_from_row(
      record.ocxo2_science, ocxo2_science);

  if (!instrument_stats_ready) {
    g_beta_instrument_stats = clocks_instrument_stats_snapshot_t{};
    instrument_stats_ready =
        clocks_alpha_instrument_stats_snapshot(&g_beta_instrument_stats);
  }
  clocks_monitor_stats_snapshot_from_instrument(
      record.stats, g_beta_instrument_stats);

  // DAC/servo state is scientific context regardless of whether a servo is
  // active. Campaigns observe it; they never own it.
  record.dac_present = true;
  clocks_monitor_dac_snapshot(record.dac);

  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_PUBLISH);
  g_monitor_campaign_record_sequence = completed_pps_sequence;
  g_monitor_campaign_record_pending = true;
  g_monitor_campaign_record_stage_count++;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_READY);

  // The interrupt-owned tick normally already has this sequence pending. This
  // call coalesces the completed record into that publication, or schedules a
  // same-sequence copy if the observation-only fragment already escaped.
  system_monitor_campaign_row_ready(completed_pps_sequence);

  clocks_beta_feature_set_cached("CAMPAIGN_RECORD_HANDOFF",
                                 g_clocks_feature_campaign_record,
                                 system_feature_status_t::NOMINAL,
                                 true);
  clocks_watchdog_arm_campaign_publication();
}

// ============================================================================
// Commands
// ============================================================================


static bool payload_try_get_double_alias(const Payload& args,
                                         const char* path,
                                         const char* lane,
                                         double& out,
                                         const char* k1,
                                         const char* k2,
                                         const char* k3) {
  if (k1 && args.has(k1)) {
    return clocks_payload_try_get_double_checked(args, k1, "command_double",
                                                 path, lane, k1, out) ==
           CLOCKS_PAYLOAD_FIELD_OK;
  }
  if (k2 && args.has(k2)) {
    return clocks_payload_try_get_double_checked(args, k2, "command_double",
                                                 path, lane, k2, out) ==
           CLOCKS_PAYLOAD_FIELD_OK;
  }
  if (k3 && args.has(k3)) {
    return clocks_payload_try_get_double_checked(args, k3, "command_double",
                                                 path, lane, k3, out) ==
           CLOCKS_PAYLOAD_FIELD_OK;
  }
  return false;
}

static bool payload_try_get_ocxo1_dac(const Payload& args,
                                      const char* path,
                                      double& out) {
  // Dedicated SET_DAC accepts these historical aliases. Campaign commands
  // reject every DAC alias through payload_campaign_control_argument().
  return payload_try_get_double_alias(args, path, "ocxo1_dac", out,
                                      "ocxo1_dac",
                                      "dac1",
                                      "set_dac1");
}

static bool payload_try_get_ocxo2_dac(const Payload& args,
                                      const char* path,
                                      double& out) {
  return payload_try_get_double_alias(args, path, "ocxo2_dac", out,
                                      "ocxo2_dac",
                                      "dac2",
                                      "set_dac2");
}

static bool payload_try_get_nonempty_string(const Payload& args,
                                            const char* key,
                                            const char*& out) {
  const char* s = args.getString(key);
  if (!s || !*s) return false;
  out = s;
  return true;
}

static bool payload_try_get_string_alias(const Payload& args,
                                         const char*& out,
                                         const char* k1,
                                         const char* k2,
                                         const char* k3,
                                         const char* k4,
                                         const char* k5,
                                         const char* k6,
                                         const char* k7) {
  return (k1 && payload_try_get_nonempty_string(args, k1, out)) ||
         (k2 && payload_try_get_nonempty_string(args, k2, out)) ||
         (k3 && payload_try_get_nonempty_string(args, k3, out)) ||
         (k4 && payload_try_get_nonempty_string(args, k4, out)) ||
         (k5 && payload_try_get_nonempty_string(args, k5, out)) ||
         (k6 && payload_try_get_nonempty_string(args, k6, out)) ||
         (k7 && payload_try_get_nonempty_string(args, k7, out));
}

static bool servo_mode_parse_strict(const char* s, servo_mode_t& out) {
  if (!s || !*s) return false;
  if (!strcasecmp(s, "OFF")) {
    out = servo_mode_t::OFF;
    return true;
  }
  if (!strcasecmp(s, "MEAN")) {
    out = servo_mode_t::MEAN;
    return true;
  }
  if (!strcasecmp(s, "TOTAL")) {
    out = servo_mode_t::TOTAL;
    return true;
  }
  if (!strcasecmp(s, "NOW")) {
    out = servo_mode_t::NOW;
    return true;
  }
  return false;
}

static bool payload_try_get_servo_mode(const Payload& args,
                                       servo_mode_t& out,
                                       const char*& raw) {
  raw = nullptr;
  if (!payload_try_get_string_alias(args, raw,
                                    "servos",
                                    "SERVOS",
                                    "servo",
                                    "SERVO",
                                    "mode",
                                    "MODE",
                                    "calibrate_ocxo")) {
    return false;
  }
  return servo_mode_parse_strict(raw, out);
}


static bool payload_campaign_control_argument(
    const Payload& args,
    const char*& out_key) {
  static const char* const keys[] = {
    "ocxo1_dac", "dac1", "set_dac1",
    "ocxo2_dac", "dac2", "set_dac2",
    "servos", "SERVOS", "servo", "SERVO",
    "mode", "MODE", "calibrate_ocxo",
  };
  for (const char* key : keys) {
    if (args.has(key)) {
      out_key = key;
      return true;
    }
  }
  out_key = nullptr;
  return false;
}

static Payload campaign_control_argument_rejected(
    const char* command,
    const char* key) {
  Payload err;
  err.add("error", "campaign command does not accept DAC/servo control");
  err.add("status", "campaign_control_argument_rejected");
  err.add("command", command ? command : "");
  err.add("argument", key ? key : "");
  err.add("control_surface", "CLOCKS.SET_DAC / CLOCKS.SERVOS");
  return err;
}

static FLASHMEM Payload cmd_servos(const Payload& args) {
  const servo_mode_t previous = calibrate_ocxo_mode;
  const bool campaign_live =
      campaign_state == clocks_campaign_state_t::STARTED &&
      !campaign_warmup_active() &&
      !request_start &&
      !request_stop &&
      !request_recover &&
      !request_zero &&
      !request_flash_cut;

  const char* raw_mode = nullptr;
  servo_mode_t requested = servo_mode_t::OFF;
  if (!payload_try_get_servo_mode(args, requested, raw_mode)) {
    Payload err;
    err.add("error", raw_mode ? "invalid SERVOS mode" : "missing SERVOS mode");
    err.add("status", "servos_rejected");
    err.add("expected", "SERVOS=OFF|NOW|MEAN|TOTAL");
    err.add("supplied", raw_mode ? raw_mode : "");
    err.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
    err.add("servo_active", calibrate_ocxo_mode != servo_mode_t::OFF);
    return err;
  }

  g_servo_mode_request_count++;
  g_servo_mode_last_requested = requested;

  if (campaign_live) {
    requested_servo_mode = requested;
    request_servo_mode_change = true;
  } else {
    request_servo_mode_change = false;
    requested_servo_mode = requested;
    clocks_apply_servo_mode_now(requested);
    g_servo_mode_last_committed = requested;
    g_servo_mode_commit_count++;
  }

  Payload p;
  p.add("status", campaign_live ? "servos_update_requested" : "servos_updated");
  p.add("previous_mode", servo_mode_str(previous));
  p.add("requested_mode", servo_mode_str(requested));
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("effective_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("pending_mode", request_servo_mode_change
                          ? servo_mode_str(requested_servo_mode)
                          : servo_mode_str(calibrate_ocxo_mode));
  p.add("request_pending", (bool)request_servo_mode_change);
  p.add("active_campaign", campaign_live);
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("request_count", g_servo_mode_request_count);
  p.add("commit_count", g_servo_mode_commit_count);
  p.add("last_requested", servo_mode_str(g_servo_mode_last_requested));
  p.add("last_committed", servo_mode_str(g_servo_mode_last_committed));
  return p;
}

static FLASHMEM Payload cmd_flash_cut(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    g_flash_cut_reject_count++;
    Payload err;
    err.add("error", "missing campaign");
    err.add("status", "flash_cut_rejected_missing_campaign");
    return err;
  }

  const char* retired_argument = nullptr;
  if (payload_campaign_control_argument(args, retired_argument)) {
    g_flash_cut_reject_count++;
    return campaign_control_argument_rejected("FLASH_CUT", retired_argument);
  }

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    g_flash_cut_reject_count++;
    Payload err;
    err.add("error", "flash cut requires an active campaign");
    err.add("status", "flash_cut_rejected_not_started");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    err.add("campaign", campaign_name);
    return err;
  }

  if (watchdog_anomaly_active || flash_cut_busy()) {
    g_flash_cut_reject_count++;
    g_flash_cut_busy_reject_count++;
    Payload err;
    err.add("error", "campaign control busy");
    err.add("status", "flash_cut_rejected_busy");
    err.add("request_flash_cut", request_flash_cut);
    err.add("request_start", request_start);
    err.add("request_stop", request_stop);
    err.add("request_recover", request_recover);
    err.add("request_zero", request_zero);
    err.add("smartzero_running", interrupt_smartzero_running());
    err.add("epoch_install_in_progress", clocks_alpha_epoch_install_in_progress());
    err.add("watchdog_anomaly_active", watchdog_anomaly_active);
    return err;
  }

  safeCopy(g_flash_cut_last_requested_campaign,
           sizeof(g_flash_cut_last_requested_campaign),
           name);
  safeCopy(flash_cut_campaign_name, sizeof(flash_cut_campaign_name), name);
  g_flash_cut_request_count++;
  request_flash_cut = true;
  safeCopy(g_flash_cut_last_status, sizeof(g_flash_cut_last_status),
           "flash_cut_requested");

  Payload p;
  p.add("status", g_flash_cut_last_status);
  p.add("campaign", flash_cut_campaign_name);
  p.add("current_campaign", campaign_name);
  p.add("boundary", "next_pps_vclock_edge");
  p.add("zero_installed", false);
  p.add("smartzero_required", false);
  p.add("service_epoch_preserved", true);
  p.add("alpha_reprime", false);
  p.add("warmup_suppression", false);
  p.add("request_count", g_flash_cut_request_count);
  p.add("commit_count", g_flash_cut_commit_count);
  p.add("ocxo1_dac", toFixedDecimal(ocxo1_dac.dac_fractional, 6));
  p.add("ocxo2_dac", toFixedDecimal(ocxo2_dac.dac_fractional, 6));
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("instrument_control_preserved", true);
  return p;
}


static FLASHMEM Payload cmd_start(const Payload& args) {
  if (g_monitor_restore_requested) {
    Payload err;
    err.add("error", "MONITOR restore is pending");
    err.add("status", "start_rejected_monitor_restore_pending");
    return err;
  }

  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  const char* retired_argument = nullptr;
  if (payload_campaign_control_argument(args, retired_argument)) {
    return campaign_control_argument_rejected("START", retired_argument);
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    return cmd_flash_cut(args);
  }

  if (clocks_campaign_recovery_lifecycle_active() || request_recover) {
    Payload err;
    err.add("error", "campaign recovery is active");
    err.add("status", "start_rejected_recovering");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    err.add("request_recover", request_recover);
    err.add("recover_reason", g_recover_lifecycle_reason);
    err.add("recover_abort_command", "RECOVER_ABORT");
    return err;
  }

  if (!clocks_alpha_installed_smartzero_backing_epoch() ||
      clocks_alpha_epoch_install_in_progress()) {
    Payload err;
    err.add("error", "instrument clock epoch is not ready");
    err.add("status", "start_rejected_instrument_initializing");
    err.add("epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
    err.add("epoch_install_in_progress",
            clocks_alpha_epoch_install_in_progress());
    err.add("retry", "START after CLOCKS.REPORT_CLOCKS shows valid=true");
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);

  request_start = true;
  request_zero = false;
  request_stop = false;
  request_recover = false;
  flash_cut_clear_pending();
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  campaign_state = clocks_campaign_state_t::STOPPED;
  campaign_warmup_reset();

  Payload p;
  p.add("status", "start_requested");
  p.add("campaign_admission_source", "PI_MONITOR_PREFLIGHT");
  p.add("recording_boundary", "NEXT_COMPLETED_PPS");
  p.add("instrument_always_on", true);
  p.add("service_epoch_preserved", true);
  p.add("statistics_preserved", true);
  p.add("smartzero_required", false);
  p.add("epoch_owner", "CLOCKS_ALPHA");
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("epoch_reason", clocks_alpha_epoch_last_reason());
  p.add("ocxo1_dac", toFixedDecimal(ocxo1_dac.dac_fractional, 6));
  p.add("ocxo2_dac", toFixedDecimal(ocxo2_dac.dac_fractional, 6));
  p.add("ocxo1_dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("instrument_control_preserved", true);
  return p;
}


static FLASHMEM Payload cmd_stop(const Payload&) {
  g_monitor_restore_requested = false;
  g_monitor_restore_state = clocks_recovery_restore_state_t{};
  const bool had_live_smartzero = interrupt_smartzero_running();
  const bool had_pending_start = request_start;
  const bool had_pending_zero = request_zero;
  const bool had_pending_recover = request_recover;
  const bool had_recovering = clocks_campaign_recovery_lifecycle_active();

  interrupt_smartzero_abort();
  if (had_live_smartzero || had_pending_start || had_pending_zero) {
    clocks_alpha_smartzero_pending_clear();
  }

  request_start = false;
  request_zero = false;
  request_recover = false;
  flash_cut_clear_pending();

  Payload p;

  if (campaign_state == clocks_campaign_state_t::STARTED &&
      !had_pending_recover && !had_recovering) {
    request_stop = true;
    p.add("status", "stop_requested");
    p.add("service_epoch_preserved", true);
    p.add("statistics_preserved", true);
    return p;
  }

  if (had_recovering || had_pending_recover) {
    recover_lifecycle_abort("stop_command_abort_recover");
    p.add("status", "recover_aborted_by_stop");
    p.add("service_epoch_preserved", true);
    p.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return p;
  }

  // STOP while no campaign is running is a campaign/control-plane abort. It
  // must not invalidate the installed epoch or alter the independent servo
  // control plane.
  request_stop = false;
  clocks_watchdog_clear_surrender_for_new_lifecycle();

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
  g_monitor_restore_requested = false;
  g_monitor_restore_state = clocks_recovery_restore_state_t{};
  if (clocks_campaign_recovery_lifecycle_active() || request_recover) {
    recover_lifecycle_abort("zero_command_abort_recover");
  }
  request_start = false;
  request_stop = false;
  request_recover = false;
  flash_cut_clear_pending();
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

static FLASHMEM Payload cmd_restore_monitor(const Payload& args) {
  if (campaign_state != clocks_campaign_state_t::STOPPED ||
      request_start || request_stop || request_recover || request_zero ||
      request_flash_cut || g_monitor_restore_requested ||
      clocks_campaign_recovery_lifecycle_active()) {
    Payload err;
    err.add("error", "instrument lifecycle busy");
    err.add("status", "monitor_restore_rejected_busy");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return err;
  }

  g_monitor_restore_state = clocks_recovery_restore_state_t{};
  if (!clocks_recovery_state_from_args(
          args, g_monitor_restore_state, true)) {
    g_monitor_restore_failure_count++;
    Payload err;
    err.add("error", "invalid structured recovery state");
    err.add("status", "monitor_restore_rejected_state");
    err.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
    return err;
  }

  if (!clocks_recovery_install_dac_control(g_monitor_restore_state)) {
    g_monitor_restore_state = clocks_recovery_restore_state_t{};
    g_monitor_restore_failure_count++;
    Payload err;
    err.add("error", "failed to restore DAC/control state");
    err.add("status", "monitor_restore_rejected_dac");
    err.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
    return err;
  }

  if (!clocks_alpha_installed_smartzero_backing_epoch() &&
      !interrupt_smartzero_running() &&
      !interrupt_smartzero_complete() &&
      !clocks_alpha_epoch_install_in_progress()) {
    if (!clocks_alpha_begin_smartzero_epoch("monitor_restore")) {
      g_monitor_restore_state = clocks_recovery_restore_state_t{};
      g_monitor_restore_failure_count++;
      clocks_apply_servo_mode_now(servo_mode_t::OFF);
      Payload err;
      err.add("error", "failed to start SmartZero for monitor restore");
      err.add("status", "monitor_restore_rejected_smartzero");
      return err;
    }
  }

  g_monitor_restore_requested = true;
  g_monitor_restore_request_count++;

  Payload p;
  p.add("status", "monitor_restore_requested");
  p.add("boundary", "next_completed_pps_after_epoch_ready");
  p.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
  p.add("request_count", g_monitor_restore_request_count);
  p.add("epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  p.add("smartzero_running", interrupt_smartzero_running());
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("dither_operator_enabled",
        clocks_ocxo_dac_dither_operator_enabled());
  p.add("ocxo1_dac", toFixedDecimal(
      ocxo_dac_fractional_snapshot(ocxo1_dac), 6));
  p.add("ocxo2_dac", toFixedDecimal(
      ocxo_dac_fractional_snapshot(ocxo2_dac), 6));
  return p;
}

static FLASHMEM Payload cmd_recover(const Payload& args) {
  clocks_payload_numeric_integrity_reset();

  if (g_monitor_restore_requested) {
    Payload err;
    err.add("error", "MONITOR restore is pending");
    err.add("status", "recover_rejected_monitor_restore_pending");
    return err;
  }

  const char* retired_argument = nullptr;
  if (payload_campaign_control_argument(args, retired_argument)) {
    return campaign_control_argument_rejected("RECOVER", retired_argument);
  }

  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  const bool recovery_state_supplied = args.has("restore_schema_version");
  if (recovery_state_supplied &&
      !clocks_recovery_state_from_args(
          args, g_campaign_restore_state, false)) {
    Payload err;
    err.add("error", "invalid structured recovery state");
    err.add("status", "recover_rejected_state");
    err.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
    return err;
  }

  uint64_t dwt_ns = 0ULL;
  uint64_t gnss_ns = 0ULL;
  uint64_t ocxo1_ns = 0ULL;
  uint64_t ocxo2_ns = 0ULL;

  const clocks_payload_checked_status_t dwt_status =
      clocks_payload_try_get_u64_checked(args, "dwt_ns",
                                         "command_recover_base",
                                         "args",
                                         "base",
                                         "dwt_ns",
                                         dwt_ns);
  const clocks_payload_checked_status_t gnss_status =
      clocks_payload_try_get_u64_checked(args, "gnss_ns",
                                         "command_recover_base",
                                         "args",
                                         "base",
                                         "gnss_ns",
                                         gnss_ns);
  const clocks_payload_checked_status_t ocxo1_status =
      clocks_payload_try_get_u64_checked(args, "ocxo1_ns",
                                         "command_recover_base",
                                         "args",
                                         "base",
                                         "ocxo1_ns",
                                         ocxo1_ns);
  const clocks_payload_checked_status_t ocxo2_status =
      clocks_payload_try_get_u64_checked(args, "ocxo2_ns",
                                         "command_recover_base",
                                         "args",
                                         "base",
                                         "ocxo2_ns",
                                         ocxo2_ns);

  if (g_clocks_payload_numeric_integrity_failed) {
    return clocks_payload_numeric_reject_response("recover_rejected_numeric_integrity");
  }

  if (dwt_status != CLOCKS_PAYLOAD_FIELD_OK ||
      gnss_status != CLOCKS_PAYLOAD_FIELD_OK ||
      ocxo1_status != CLOCKS_PAYLOAD_FIELD_OK ||
      ocxo2_status != CLOCKS_PAYLOAD_FIELD_OK) {
    Payload err;
    err.add("error", "missing recovery parameters (dwt_ns, gnss_ns, ocxo1_ns, ocxo2_ns)");
    err.add("status", "recover_rejected_missing_parameters");
    return err;
  }

  const char* recover_campaign_arg = args.getString("campaign");
  if (!recover_campaign_arg || !*recover_campaign_arg) {
    recover_campaign_arg = args.getString("name");
  }

  char requested_campaign[64] = {0};
  const bool campaign_supplied =
      recover_campaign_arg && *recover_campaign_arg;
  safeCopy(requested_campaign,
           sizeof(requested_campaign),
           campaign_supplied ? recover_campaign_arg : campaign_name);

  const uint64_t requested_base_count =
      gnss_ns / CLOCKS_BETA_NS_PER_SECOND;
  const bool same_identity =
      g_recover_request_count != 0U &&
      requested_base_count == g_recover_last_base_count &&
      gnss_ns == g_recover_last_base_gnss_ns &&
      dwt_ns == g_recover_last_base_dwt_ns &&
      ocxo1_ns == g_recover_last_base_ocxo1_ns &&
      ocxo2_ns == g_recover_last_base_ocxo2_ns &&
      strcmp(requested_campaign, g_recover_last_campaign) == 0;

  const bool recovery_control_active =
      request_recover ||
      clocks_campaign_recovery_lifecycle_active() ||
      g_recover_reattach_active ||
      g_recover_reattach_degraded_active ||
      g_science_residual_quarantine_remaining != 0U ||
      g_recover_continuity_align_pending;

  if (recovery_control_active) {
    Payload p;
    p.add("status", same_identity
                        ? "recover_already_active"
                        : "recover_rejected_busy");
    if (!same_identity) {
      p.add("error", "different recovery already active");
    }
    p.add("idempotent", same_identity);
    p.add("recovery_generation", g_recover_request_count);
    p.add("campaign", campaign_name);
    p.add("requested_campaign", requested_campaign);
    p.add("base_count", g_recover_last_base_count);
    p.add("requested_base_count", requested_base_count);
    p.add("expected_first_public_count",
          g_recover_last_expected_first_public_count);
    p.add("recover_reattach_active", (bool)g_recover_reattach_active);
    p.add("recover_degraded_active",
          (bool)g_recover_reattach_degraded_active);
    p.add("recover_clockface_ready",
          (bool)g_recover_reattach_clockface_ready);
    p.add("recover_science_ready",
          (bool)g_recover_reattach_science_ready);
    p.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return p;
  }

  // A transport retry may arrive after the exact recovery already completed.
  // Re-running the same identity would cut OCXO custody a second time, so
  // acknowledge it without touching Alpha/Beta state.
  if (same_identity &&
      campaign_state == clocks_campaign_state_t::STARTED &&
      g_campaign_record_last_public_count >=
          g_recover_last_expected_first_public_count) {
    Payload p;
    p.add("status", "recover_already_completed");
    p.add("idempotent", true);
    p.add("recovery_generation", g_recover_request_count);
    p.add("campaign", campaign_name);
    p.add("base_count", g_recover_last_base_count);
    p.add("expected_first_public_count",
          g_recover_last_expected_first_public_count);
    p.add("last_public_count", g_campaign_record_last_public_count);
    p.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return p;
  }

  g_recover_last_campaign_supplied = campaign_supplied;
  if (campaign_supplied) {
    safeCopy(campaign_name, sizeof(campaign_name), requested_campaign);
  }
  safeCopy(g_recover_last_campaign,
           sizeof(g_recover_last_campaign),
           requested_campaign);

  recover_dwt_ns   = dwt_ns;
  recover_gnss_ns  = gnss_ns;
  recover_ocxo1_ns = ocxo1_ns;
  recover_ocxo2_ns = ocxo2_ns;

  g_recover_request_count++;
  g_recover_last_base_count = requested_base_count;
  g_recover_last_expected_first_public_count =
      g_recover_last_base_count + 1ULL;
  g_recover_last_base_gnss_ns = recover_gnss_ns;
  g_recover_last_base_dwt_ns = recover_dwt_ns;
  g_recover_last_base_ocxo1_ns = recover_ocxo1_ns;
  g_recover_last_base_ocxo2_ns = recover_ocxo2_ns;

  clocks_watchdog_disarm_campaign_publication();
  // A live epoch may have a stray replacement SmartZero acquisition; abort it
  // before live reattach.  After a flash there is no epoch yet, and startup
  // SmartZero is the lawful bootstrap proof RECOVER must preserve.
  if (clocks_alpha_installed_smartzero_backing_epoch()) {
    interrupt_smartzero_abort();
  }
  request_recover = true;
  request_start   = false;
  request_stop    = false;
  request_zero    = false;
  flash_cut_clear_pending();

  if (!recover_lifecycle_enter_from_command("recover_command_armed")) {
    recover_lifecycle_abort("recover_interrupt_service_rearm_failed");

    Payload err;
    err.add("error", "failed to rearm VCLOCK/OCXO interrupt service");
    err.add("status", "recover_rejected_interrupt_service_rearm");
    err.add("recovery_generation", g_recover_request_count);
    err.add("recover_interrupt_service_rearm_count",
            g_recover_lifecycle_interrupt_service_rearm_count);
    err.add("recover_interrupt_service_rearm_failure_count",
            g_recover_lifecycle_interrupt_service_rearm_failure_count);
    err.add("recover_interrupt_service_rearm_ok",
            g_recover_lifecycle_last_interrupt_service_rearm_ok);
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return err;
  }

  if (g_recover_lifecycle_mode ==
          recover_lifecycle_mode_t::LIVE_REATTACH &&
      g_campaign_restore_state.valid) {
    // The running Teensy is newer than the last durable TIMEBASE row. Preserve
    // its live statistics and all instrument control state.
    g_campaign_restore_ignored_live_count++;
    g_campaign_restore_state = clocks_recovery_restore_state_t{};
  }

  Payload p;
  p.add("status", "recover_requested");
  p.add("idempotent", false);
  p.add("recovery_generation", g_recover_request_count);
  p.add("instrument_statistics_preserved",
        clocks_alpha_installed_smartzero_backing_epoch());
  p.add("instrument_statistics_restore_staged",
        g_recover_lifecycle_mode == recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        g_campaign_restore_state.valid);
  p.add("instrument_statistics_restored", false);
  p.add("instrument_statistics_recovery_source",
        g_recover_lifecycle_mode == recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        g_campaign_restore_state.valid
            ? "TIMEBASE_STRUCTURED_RESTORE"
            : "LIVE_ALPHA_OR_LEGACY_COLD_REFILL");
  p.add("recovery_state_supplied", recovery_state_supplied);
  p.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count",
        g_recover_last_expected_first_public_count);
  p.add("campaign", campaign_name);
  p.add("campaign_supplied", g_recover_last_campaign_supplied);
  p.add("recover_last_campaign", g_recover_last_campaign);
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("instrument_control_preserved", true);
  p.add("recover_status_report", "REPORT_RECOVERY");
  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("recover_lifecycle", clocks_campaign_recovery_lifecycle_active());
  p.add("recover_lifecycle_reason", g_recover_lifecycle_reason);
  p.add("recover_mode",
        recover_lifecycle_mode_name(g_recover_lifecycle_mode));
  p.add("recover_cold_bootstrap_active",
        g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        clocks_campaign_recovery_lifecycle_active());
  p.add("recover_cold_bootstrap_epoch_ready",
        g_recover_lifecycle_cold_bootstrap_epoch_ready);
  p.add("recover_smartzero_running", interrupt_smartzero_running());
  p.add("recover_smartzero_complete", interrupt_smartzero_complete());
  p.add("recover_epoch_ready",
        clocks_alpha_installed_smartzero_backing_epoch());
  p.add("recover_command_custody_reset_count",
        g_recover_lifecycle_command_custody_reset_count);
  p.add("recover_interrupt_service_rearm_count",
        g_recover_lifecycle_interrupt_service_rearm_count);
  p.add("recover_interrupt_service_rearm_failure_count",
        g_recover_lifecycle_interrupt_service_rearm_failure_count);
  p.add("recover_interrupt_service_rearm_ok",
        g_recover_lifecycle_last_interrupt_service_rearm_ok);
  p.add("interrupt_recover_publication_reset_count",
        interrupt_recover_publication_custody_reset_count());
  return p;
}

static FLASHMEM Payload cmd_recover_abort(const Payload& args) {
  const char* reason = args.getString("reason");
  recover_lifecycle_abort(reason && *reason
                              ? reason
                              : "recover_abort_command");
  Payload p;
  p.add("status", "recover_aborted");
  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("abort_count", g_recover_lifecycle_abort_count);
  p.add("abort_reason", g_recover_lifecycle_abort_reason);
  p.add("request_recover", request_recover);
  p.add("watchdog_publication_blocked", clocks_watchdog_publication_blocked());
  return p;
}

static FLASHMEM Payload cmd_watchdog_test(const Payload&) {
  clocks_watchdog_anomaly("watchdog_test");
  Payload p;
  p.add("status", "watchdog_anomaly_requested");
  return p;
}

static uint32_t clocks_problem_injection_lane_parse(const char* lane) {
  if (!lane || !*lane || !strcasecmp(lane, "OCXO1")) {
    return CLOCKS_ROW_LANE_OCXO1;
  }
  if (!strcasecmp(lane, "PPS")) return CLOCKS_ROW_LANE_PPS;
  if (!strcasecmp(lane, "VCLOCK")) return CLOCKS_ROW_LANE_VCLOCK;
  if (!strcasecmp(lane, "OCXO2")) return CLOCKS_ROW_LANE_OCXO2;
  return 0U;
}

static FLASHMEM Payload cmd_inject_problem(const Payload& args) {
  const char* type = args.getString("type");
  if (!type || !*type || strcasecmp(type, "excursion") != 0) {
    Payload err;
    err.add("error", type && *type
                         ? "unsupported problem type"
                         : "missing problem type");
    err.add("status", "inject_problem_rejected_type");
    err.add("supplied", type ? type : "");
    err.add("supported", "excursion");
    return err;
  }

  const char* lane_text = args.getString("lane");
  const uint32_t lane = clocks_problem_injection_lane_parse(lane_text);
  if (lane == 0U) {
    Payload err;
    err.add("error", "invalid excursion lane");
    err.add("status", "inject_problem_rejected_lane");
    err.add("supplied", lane_text ? lane_text : "");
    err.add("expected", "PPS|VCLOCK|OCXO1|OCXO2");
    return err;
  }

  double cycles_value = 1000.0;
  if (args.has("cycles") || args.has("CYCLES") || args.has("magnitude")) {
    clocks_payload_numeric_integrity_reset();
    if (!payload_try_get_double_alias(args,
                                      "command.inject_problem",
                                      "cycles",
                                      cycles_value,
                                      "cycles",
                                      "CYCLES",
                                      "magnitude") ||
        g_clocks_payload_numeric_integrity_failed) {
      return clocks_payload_numeric_reject_response(
          "inject_problem_rejected_numeric_integrity");
    }
  }

  if (!isfinite(cycles_value) || cycles_value < 257.0 ||
      cycles_value > 1000000.0) {
    Payload err;
    err.add("error", "excursion cycles outside supported range");
    err.add("status", "inject_problem_rejected_cycles");
    err.add("minimum", 257U);
    err.add("maximum", 1000000U);
    err.add("supplied", toFixedDecimal(cycles_value, 3));
    return err;
  }
  const uint32_t cycles = (uint32_t)llround(cycles_value);

  const bool stable_campaign =
      campaign_state == clocks_campaign_state_t::STARTED &&
      !campaign_warmup_active() &&
      !clocks_watchdog_publication_blocked() &&
      !request_start && !request_stop && !request_recover &&
      !request_zero && !request_flash_cut;
  if (!stable_campaign) {
    Payload err;
    err.add("error", "problem injection requires a stable active campaign");
    err.add("status", "inject_problem_rejected_campaign_state");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    err.add("campaign", campaign_name);
    err.add("campaign_seconds", campaign_seconds);
    err.add("warmup_active", campaign_warmup_active());
    err.add("watchdog_blocked", clocks_watchdog_publication_blocked());
    return err;
  }

  uint32_t expected = 0U;
  if (!__atomic_compare_exchange_n(&g_problem_injection_armed,
                                   &expected,
                                   1U,
                                   false,
                                   __ATOMIC_ACQ_REL,
                                   __ATOMIC_ACQUIRE)) {
    Payload err;
    err.add("error", "a problem injection is already armed");
    err.add("status", "inject_problem_rejected_already_armed");
    err.add("armed_state", expected);
    err.add("arm_count", (uint32_t)g_problem_injection_arm_count);
    err.add("fire_count", (uint32_t)g_problem_injection_fire_count);
    return err;
  }

  __atomic_store_n(&g_problem_injection_lane, lane, __ATOMIC_RELAXED);
  __atomic_store_n(&g_problem_injection_cycles, cycles, __ATOMIC_RELAXED);
  (void)__atomic_add_fetch(&g_problem_injection_arm_count,
                           1U,
                           __ATOMIC_RELAXED);
  __atomic_store_n(&g_problem_injection_armed, 2U, __ATOMIC_RELEASE);

  Payload p;
  p.add("status", "inject_problem_armed");
  p.add("type", "excursion");
  p.add("lane", lane_text && *lane_text ? lane_text : "OCXO1");
  p.add("lane_mask", lane);
  p.add("cycles", cycles);
  p.add("one_shot", true);
  p.add("boundary", "next_completed_pps_row");
  p.add("raw_counters_modified", false);
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("arm_count", (uint32_t)g_problem_injection_arm_count);
  p.add("fire_count", (uint32_t)g_problem_injection_fire_count);
  return p;
}


// ============================================================================
// Recovery polling report
// ============================================================================

static FLASHMEM Payload cmd_report_recovery(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_RECOVERY);

  Payload p;
  p.add("report", "CLOCKS_RECOVERY");
  p.add("schema", "CLOCKS_RECOVERY_COMPACT_V2");

  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);

  p.add("recover_lifecycle_active", clocks_campaign_recovery_lifecycle_active());
  p.add("recover_lifecycle_reason", g_recover_lifecycle_reason);
  p.add("recover_mode", recover_lifecycle_mode_name(g_recover_lifecycle_mode));
  p.add("recover_cold_bootstrap_active",
        g_recover_lifecycle_mode == recover_lifecycle_mode_t::COLD_BOOTSTRAP &&
        clocks_campaign_recovery_lifecycle_active());
  p.add("recover_cold_bootstrap_epoch_ready",
        g_recover_lifecycle_cold_bootstrap_epoch_ready);
  p.add("recover_cold_bootstrap_begin_count",
        g_recover_lifecycle_cold_bootstrap_begin_count);
  p.add("recover_cold_bootstrap_wait_count",
        g_recover_lifecycle_cold_bootstrap_wait_count);
  p.add("recover_cold_bootstrap_ready_count",
        g_recover_lifecycle_cold_bootstrap_ready_count);
  p.add("recover_cold_bootstrap_commit_count",
        g_recover_lifecycle_cold_bootstrap_commit_count);
  p.add("recover_smartzero_running", interrupt_smartzero_running());
  p.add("recover_smartzero_complete", interrupt_smartzero_complete());
  p.add("recover_epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  p.add("recover_interrupt_service_rearm_ok",
        g_recover_lifecycle_last_interrupt_service_rearm_ok);
  p.add("recover_interrupt_service_rearm_count",
        g_recover_lifecycle_interrupt_service_rearm_count);
  p.add("recover_interrupt_service_rearm_failure_count",
        g_recover_lifecycle_interrupt_service_rearm_failure_count);

  p.add("recovery_generation", g_recover_request_count);
  p.add("request_count", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count",
        g_recover_last_expected_first_public_count);
  p.add("last_public_count", g_campaign_record_last_public_count);
  p.add("candidate_count", g_campaign_record_candidate_count);
  p.add("hidden_candidate_count", g_recover_reattach_hidden_candidate_count);
  p.add("campaign_record_last_stage", g_campaign_record_last_stage);
  p.add("campaign_record_last_stage_name",
        campaign_record_stage_name(g_campaign_record_last_stage));

  p.add("recover_reattach_active", (bool)g_recover_reattach_active);
  p.add("recover_reattach_degraded_active",
        (bool)g_recover_reattach_degraded_active);
  p.add("recover_reattach_reason", g_recover_reattach_last_reason);
  p.add("recover_clockface_ready", (bool)g_recover_reattach_clockface_ready);
  p.add("recover_science_ready", (bool)g_recover_reattach_science_ready);
  p.add("recover_reattach_stalled", (bool)g_recover_reattach_stalled);
  p.add("degraded_no_progress_row_count",
        g_recover_reattach_no_progress_row_count);
  p.add("degraded_last_progress_public_count",
        g_recover_reattach_last_progress_public_count);
  p.add("science_quarantine_remaining",
        g_science_residual_quarantine_remaining);
  p.add("instrument_statistics_owner", "ALPHA");
  p.add("instrument_statistics_preserved", true);
  p.add("instrument_statistics_restored", false);
  p.add("warmup_active", campaign_warmup_active());

  const bool recover_timeline_ready =
      g_campaign_record_last_public_count != 0U &&
      g_campaign_record_last_public_gnss_ns != 0ULL &&
      g_campaign_record_last_public_dwt_total != 0ULL &&
      !watchdog_anomaly_active &&
      !clocks_watchdog_publication_blocked();
  p.add("recover_timeline_ready", recover_timeline_ready);

  p.add("watchdog_anomaly_active", watchdog_anomaly_active);
  p.add("watchdog_publication_blocked",
        clocks_watchdog_publication_blocked());

  return p;
}


static FLASHMEM void report_add_instrument_clock_values(
    Payload& parent,
    const clocks_instrument_stats_snapshot_t& instrument) {
  Payload& clocks = g_report_child_clocks;
  clocks.clear();
  clocks.add("gnss_ns", instrument.gnss_ns);
  clocks.add("dwt_cycles", instrument.dwt_cycles);
  clocks.add("ocxo1_ns", instrument.ocxo1_ns);
  clocks.add("ocxo2_ns", instrument.ocxo2_ns);
  clocks.add("completed_pps_sequence", instrument.last_pps_sequence);
  clocks.add("completed_row_coherent", instrument.completed_row_coherent);
  clocks.add("dwt_cycles_per_second", instrument.dwt_cycles_per_second);
  clocks.add("selected_reference_interval_cycles",
             instrument.selected_reference_interval_cycles);
  clocks.add("vclock_interval_cycles", instrument.vclock_interval_cycles);
  clocks.add("ocxo1_interval_cycles", instrument.ocxo1_interval_cycles);
  clocks.add("ocxo2_interval_cycles", instrument.ocxo2_interval_cycles);
  parent.add_object("clocks", clocks);
  clocks.clear();
}

static FLASHMEM Payload clocks_report_busy_response(const char* report) {
  Payload p;
  p.add("error", "clock report builder busy");
  p.add("status", "report_deferred_busy");
  p.add("report", report ? report : "CLOCKS_REPORT");
  p.add("retry", true);
  p.add("report_busy_reject_count", g_clocks_report_busy_reject_count);
  return p;
}

static FLASHMEM void report_add_common_metadata(
    Payload& p,
    const char* report,
    const char* schema,
    bool snapshot_ok) {
  p.add("report", report);
  p.add("schema", schema);
  p.add("instrument_always_on", true);
  p.add("instrument_owner", "ALPHA");
  p.add("snapshot_ok", snapshot_ok);
  p.add("valid", g_beta_report_instrument_stats.valid);
  p.add("completed_row_coherent",
        g_beta_report_instrument_stats.completed_row_coherent);
  p.add("completed_pps_sequence",
        g_beta_report_instrument_stats.last_pps_sequence);
  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("recording", campaign_state == clocks_campaign_state_t::STARTED);
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  p.add("epoch_sequence", clocks_alpha_epoch_sequence());
  p.add("report_priority0_capture_live", true);
  p.add("report_priority16_excluded", true);
  p.add("report_build_count", g_clocks_report_build_count);
  p.add("report_busy_reject_count", g_clocks_report_busy_reject_count);
  p.add("report_max_duration_cycles", g_clocks_report_max_duration_cycles);
}

static FLASHMEM Payload cmd_report_clocks(const Payload&) {
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_INSTRUMENT");

  g_beta_report_instrument_stats = clocks_instrument_stats_snapshot_t{};
  const bool snapshot_ok = clocks_alpha_instrument_stats_snapshot(
      &g_beta_report_instrument_stats);

  Payload& built = g_report_clocks_payload;
  built.clear();
  report_add_common_metadata(built,
                             "CLOCKS_INSTRUMENT",
                             "CLOCKS_INSTRUMENT_REPORT_V2",
                             snapshot_ok);
  report_add_instrument_clock_values(built, g_beta_report_instrument_stats);

  // Deliberately compact: full distributions live under REPORT_STATS.
  built.add("stats_reset_count", g_beta_report_instrument_stats.reset_count);
  built.add("stats_update_count", g_beta_report_instrument_stats.update_count);
  built.add("dwt_ppb",
            toFixedDecimal(g_beta_report_instrument_stats.dwt_frequency.ppb, 3));
  built.add("vclock_ppb",
            toFixedDecimal(g_beta_report_instrument_stats.vclock_frequency.ppb, 3));
  built.add("ocxo1_ppb",
            toFixedDecimal(g_beta_report_instrument_stats.ocxo1_frequency.ppb, 3));
  built.add("ocxo2_ppb",
            toFixedDecimal(g_beta_report_instrument_stats.ocxo2_frequency.ppb, 3));
  built.add("dwt_samples",
            g_beta_report_instrument_stats.dwt_frequency.sample_count);
  built.add("vclock_samples",
            g_beta_report_instrument_stats.vclock_frequency.sample_count);
  built.add("ocxo1_samples",
            g_beta_report_instrument_stats.ocxo1_frequency.sample_count);
  built.add("ocxo2_samples",
            g_beta_report_instrument_stats.ocxo2_frequency.sample_count);

  Payload response = built;
  built.clear();
  return response;
}

static FLASHMEM Payload cmd_report_stats(const Payload&) {
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_STATS");

  g_beta_report_instrument_stats = clocks_instrument_stats_snapshot_t{};
  const bool snapshot_ok = clocks_alpha_instrument_stats_snapshot(
      &g_beta_report_instrument_stats);

  Payload& built = g_report_stats_payload;
  built.clear();
  report_add_common_metadata(built,
                             "CLOCKS_STATS",
                             "CLOCKS_INSTRUMENT_STATS_REPORT_V2",
                             snapshot_ok);
  report_add_stats_summary_from_snapshot(built, g_beta_report_instrument_stats);

  Payload response = built;
  built.clear();
  return response;
}

static FLASHMEM Payload cmd_stats_reset(const Payload&) {
  clocks_alpha_instrument_stats_reset();

  // Keep reset acknowledgment tiny. The operator may request REPORT_CLOCKS or
  // REPORT_STATS after the next completed row without recursively entering the
  // heavy report builder from this command.
  Payload p;
  p.add("status", "instrument_statistics_reset");
  p.add("reset", true);
  p.add("clocks_reset", false);
  p.add("campaign_changed", false);
  p.add("instrument_owner", "ALPHA");
  p.add("lifetime", "BOOT_TO_REBOOT_OR_STATS_RESET");
  p.add("next_report", "REPORT_CLOCKS");
  return p;
}

static FLASHMEM Payload cmd_stack_witness_reset(const Payload&) {
  clocks_stack_witness_reset();
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_STACK);
  Payload p;
  p.add("report", "CLOCKS_STACK_WITNESS");
  p.add("schema", "CLOCKS_STACK_WITNESS_REPORT_V1");
  p.add("reset", true);
  payload_add_stack_witness(p);
  return p;
}


static FLASHMEM void payload_add_dither_status_lane_compact(
    Payload& parent,
    const char* key,
    const ocxo_dac_state_t& dac) {
  Payload lane;
  lane.add("dac", toFixedDecimal(dac.dac_fractional, 6));
  lane.add("hw_code", (uint32_t)dac.dac_hw_code);
  lane.add("voltage", toFixedDecimal(ocxo_dac_voltage_from_code((double)dac.dac_hw_code), 9));

  lane.add("active", dac.dither_active_this_frame);
  lane.add("low_code", (uint32_t)dac.dither_low_code);
  lane.add("high_code", (uint32_t)dac.dither_high_code);
  lane.add("high_ms", (uint32_t)dac.dither_high_ms);
  lane.add("phase_high", dac.dither_current_phase_high);
  lane.add("pending_hw_write", dac.dither_pending_hw_write);
  lane.add("pending_hw_code", (uint32_t)dac.dither_pending_hw_code);

  lane.add("frame_count", dac.dither_frame_count);
  lane.add("transition_count", dac.dither_transition_count);
  lane.add("write_count", dac.dither_write_count);
  lane.add("write_failures", dac.dither_write_failure_count);
  lane.add("skip_same_code_count", dac.dither_skip_same_code_count);
  lane.add("service_count", dac.dither_service_count);
  lane.add("service_write_count", dac.dither_service_write_count);
  lane.add("service_defer_count", dac.dither_service_defer_count);

  lane.add("io_ok", dac.io_last_write_ok &&
                    !dac.io_fault_latched &&
                    dac.io_last_failure_stage == 0);
  lane.add("io_write_attempts", dac.io_write_attempts);
  lane.add("io_write_failures", dac.io_write_failures);
  lane.add("servo_adjustments", dac.servo_adjustments);
  lane.add("servo_hold_reason", servo_hold_reason_name(dac.servo_hold_reason));
  lane.add("servo_hold_reason_id", (uint32_t)dac.servo_hold_reason);
  lane.add("servo_hold_count", dac.servo_hold_count);
  lane.add("servo_quarantine_remaining", dac.servo_quarantine_remaining);
  lane.add("servo_commit_fault_hold_count", dac.servo_commit_fault_hold_count);
  lane.add("servo_request_install_count", dac.servo_request_install_count);
  lane.add("servo_request_overwrite_count", dac.servo_request_overwrite_count);
  lane.add("servo_request_dither_frame_install_count",
           dac.servo_request_dither_frame_install_count);
  lane.add("servo_request_static_install_count", dac.servo_request_static_install_count);
  lane.add("pacing_intents", dac.pacing_intents);
  lane.add("pacing_pending", dac.pacing_pending);
  lane.add("pacing_pending_hw_code", (uint32_t)dac.pacing_pending_hw_code);
  lane.add("pacing_commit_count", dac.pacing_commit_count);

  parent.add_object(key, lane);
}


static FLASHMEM Payload cmd_dither_enable(const Payload&) {
  const bool ok = clocks_ocxo_dac_dither_enable();

  Payload p;
  p.add("status", ok ? "dither_enabled" : "dither_enable_blocked_or_failed");
  p.add("enabled", clocks_ocxo_dac_dither_operator_enabled());
  p.add("started", clocks_ocxo_dac_dither_started());
  p.add("service_pending", clocks_ocxo_dac_dither_service_pending());
  p.add("write_context", clocks_ocxo_dac_dither_context());
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  payload_add_dither_status_lane_compact(p, "ocxo1", ocxo1_dac);
  payload_add_dither_status_lane_compact(p, "ocxo2", ocxo2_dac);
  return p;
}


static FLASHMEM Payload cmd_dither_disable(const Payload&) {
  const bool ok = clocks_ocxo_dac_dither_disable();

  Payload p;
  p.add("status", ok ? "dither_disabled_no_dac_write" : "dither_disable_failed");
  p.add("enabled", clocks_ocxo_dac_dither_operator_enabled());
  p.add("started", clocks_ocxo_dac_dither_started());
  p.add("service_pending", clocks_ocxo_dac_dither_service_pending());
  p.add("write_context", clocks_ocxo_dac_dither_context());
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  payload_add_dither_status_lane_compact(p, "ocxo1", ocxo1_dac);
  payload_add_dither_status_lane_compact(p, "ocxo2", ocxo2_dac);
  return p;
}


static FLASHMEM void payload_add_dac_lane_custody(
    Payload& parent,
    const char* key,
    const ocxo_dac_state_t& dac) {
  Payload lane;
  lane.add("readback_valid", dac.hw_readback_valid);
  lane.add("readback_code", (uint32_t)dac.hw_readback_code);
  lane.add("reset_signature", dac.hw_reset_signature);
  lane.add("desired", toFixedDecimal(dac.dac_fractional, 6));
  lane.add("hw_code", (uint32_t)dac.dac_hw_code);
  lane.add("safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);
  lane.add("request_pending", dac.pacing_pending);
  lane.add("pending_target", toFixedDecimal(dac.pacing_pending_target, 6));
  lane.add("last_author_source", (uint32_t)dac.io_last_author_source);
  lane.add("readback_count", dac.hw_readback_count);
  lane.add("readback_failures", dac.hw_readback_failures);
  lane.add("adopt_count", dac.hw_adopt_count);
  lane.add("same_code_skip_count", dac.io_skip_same_code_count);
  lane.add("write_attempts", dac.io_write_attempts);
  lane.add("write_successes", dac.io_write_successes);
  lane.add("write_failures", dac.io_write_failures);
  parent.add_object(key, lane);
}

static FLASHMEM Payload cmd_dac_info(const Payload&) {
  Payload p;
  p.add("schema", "CLOCKS_DAC_CUSTODY_V1");
  p.add("status", "ok");
  p.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  p.add("servo_active", clocks_servo_active());
  p.add("servo_request_pending",
        ocxo1_dac.pacing_pending || ocxo2_dac.pacing_pending);
  p.add("actuator_service_pending",
        clocks_ocxo_dac_actuator_service_pending());
  p.add("initialization_authored_output", false);
  payload_add_dac_lane_custody(p, "ocxo1", ocxo1_dac);
  payload_add_dac_lane_custody(p, "ocxo2", ocxo2_dac);
  return p;
}

static FLASHMEM Payload cmd_set_dac(const Payload& args) {
  clocks_payload_numeric_integrity_reset();
  ocxo_dac_pacing_abort_all();

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (payload_try_get_ocxo1_dac(args, "command.set_dac", dac_val)) {
    dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
    if (dac1_ok) ocxo_dac_retry_reset(ocxo1_dac);
  }
  if (payload_try_get_ocxo2_dac(args, "command.set_dac", dac_val)) {
    dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);
    if (dac2_ok) ocxo_dac_retry_reset(ocxo2_dac);
  }

  if (g_clocks_payload_numeric_integrity_failed) {
    return clocks_payload_numeric_reject_response("set_dac_rejected_numeric_integrity");
  }

  Payload p;
  p.add("ocxo1_dac", toFixedDecimal(ocxo1_dac.dac_fractional, 6));
  p.add("ocxo2_dac", toFixedDecimal(ocxo2_dac.dac_fractional, 6));
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("ocxo1_dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  p.add("ocxo1_dac_voltage",
        toFixedDecimal(ocxo_dac_voltage_from_code((double)ocxo1_dac.dac_hw_code), 9));
  p.add("ocxo2_dac_voltage",
        toFixedDecimal(ocxo_dac_voltage_from_code((double)ocxo2_dac.dac_hw_code), 9));
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  p.add("reference_mode", OCXO_DAC_REFERENCE_MODE);
  p.add("external_vref_used", false);
  p.add("internal_ref_voltage", toFixedDecimal(OCXO_DAC_INTERNAL_REF_VOLTAGE, 9));
  p.add("output_gain", toFixedDecimal(OCXO_DAC_OUTPUT_GAIN, 3));
  p.add("output_full_scale_voltage", toFixedDecimal(OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE, 9));
  p.add("dac_code_scale", toFixedDecimal(OCXO_DAC_CODE_SCALE, 1));
  p.add("safe_max_output_voltage", toFixedDecimal(OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE, 9));
  p.add("safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);
  p.add("static_rounded_only", ocxo_dac_static_rounded_only_runtime());
  p.add("fractional_stream_possible", ocxo_dac_fractional_stream_possible_runtime());
  p.add("recurring_timer_possible", clocks_ocxo_dac_dither_started());
  p.add("dither_operator_enabled", clocks_ocxo_dac_dither_operator_enabled());
  p.add("dither_service_pending", clocks_ocxo_dac_dither_service_pending());
  p.add("status", (dac1_ok && dac2_ok) ? "ok" : "dac_write_fault");
  return p;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",               cmd_start               },
  { "FLASH_CUT",           cmd_flash_cut           },
  { "STOP",                cmd_stop                },
  { "ZERO",                cmd_zero                },
  { "RECOVER",             cmd_recover             },
  { "RESTORE_MONITOR",     cmd_restore_monitor     },
  { "RECOVER_ABORT",       cmd_recover_abort       },
  { "SERVOS",              cmd_servos              },
  { "REPORT_CLOCKS",       cmd_report_clocks       },
  { "REPORT_STATS",        cmd_report_stats        },
  { "STATS_RESET",         cmd_stats_reset         },
  { "REPORT_RECOVERY",     cmd_report_recovery     },
  { "STACK_WITNESS_RESET", cmd_stack_witness_reset },
  { "DITHER_ENABLE",       cmd_dither_enable       },
  { "DITHER_DISABLE",      cmd_dither_disable      },
  { "WATCHDOG_TEST",       cmd_watchdog_test       },
  { "INJECT_PROBLEM",      cmd_inject_problem      },
  { "DAC_INFO",            cmd_dac_info            },
  { "SET_DAC",             cmd_set_dac             },
  { nullptr,                 nullptr                 }
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = nullptr
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}
