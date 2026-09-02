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
//   welford_t standardizes every published statistical accumulator.  One
//   struct, one API, double-valued samples (supports ppb + ns + LSB
//   with the same type).  Global instances named welford_<what>:
//   welford_dwt, welford_vclock, welford_ocxo1, welford_ocxo2,
//   welford_pps_witness.
//
// Campaign lifecycle is recording-only with respect to clock statistics.
// Alpha owns boot-lifetime PPB/TAU/Welford state; Beta snapshots it for the
// completed campaign observation and command reports.  DAC actuation, dithering,
// and servo control are Pi-owned and absent from Teensy CLOCKS.
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
// CLOCKS Payload / foreground ownership court
// ============================================================================
//
// CLOCKS intentionally permits Alpha/Beta acquisition to continue while an older
// immutable CLOCKS_FRAGMENT queue entry is being serialized.  What may not overlap
// is Payload construction itself.  Command handlers, the canonical fragment
// serializer, and exceptional Beta/watchdog event builders therefore share one
// explicit fail-hard Payload owner.
//
// Payload custody is an operation owner, not an interrupt mask.  QTimer1 at
// Priority 16 and process_interrupt continuation at Priority 32 must remain live
// even while a foreground command is constructing a Payload: continuation is
// what extends the 16-bit 10 MHz timer domains across low-word revolutions.
// TimePop application callbacks, including CLOCKS_FRAGMENT serialization, are
// foreground-owned after immutable interrupt facts have escaped handler context.
// If any execution context nevertheless attempts a competing Payload mutation,
// the atomic owner below traps rather than hiding the architecture violation.
static constexpr size_t CLOCKS_RAM2_CACHE_LINE_BYTES = 32U;
static_assert((CLOCKS_RAM2_CACHE_LINE_BYTES &
               (CLOCKS_RAM2_CACHE_LINE_BYTES - 1U)) == 0U,
              "CLOCKS RAM2 cache-line alignment must be a power of two");
static_assert((CLOCKS_RAM2_CACHE_LINE_BYTES %
               Payload::FIXED_STORAGE_ALIGNMENT) == 0U,
              "CLOCKS RAM2 alignment must satisfy Payload fixed storage");

enum class clocks_payload_owner_t : uint32_t {
  NONE = 0U,
  FRAGMENT = 1U,
  COMMAND = 2U,
  EVENT = 3U,
};

static volatile uint32_t g_clocks_payload_owner = 0U;

static inline void clocks_payload_owner_assert(clocks_payload_owner_t owner) {
  if (owner == clocks_payload_owner_t::NONE ||
      __atomic_load_n(&g_clocks_payload_owner, __ATOMIC_ACQUIRE) !=
          (uint32_t)owner) {
    __builtin_trap();
  }
}

class clocks_payload_custody_t {
 public:
  explicit clocks_payload_custody_t(clocks_payload_owner_t owner)
      : owner_(owner) {
    if (owner_ == clocks_payload_owner_t::NONE) __builtin_trap();
    uint32_t expected = 0U;
    if (!__atomic_compare_exchange_n(&g_clocks_payload_owner,
                                     &expected,
                                     (uint32_t)owner_,
                                     false,
                                     __ATOMIC_ACQ_REL,
                                     __ATOMIC_ACQUIRE)) {
      __builtin_trap();
    }
  }

  ~clocks_payload_custody_t() {
    if (__atomic_load_n(&g_clocks_payload_owner, __ATOMIC_ACQUIRE) !=
        (uint32_t)owner_) {
      __builtin_trap();
    }
    __atomic_store_n(&g_clocks_payload_owner, 0U, __ATOMIC_RELEASE);
  }

  clocks_payload_custody_t(const clocks_payload_custody_t&) = delete;
  clocks_payload_custody_t& operator=(const clocks_payload_custody_t&) = delete;

 private:
  clocks_payload_owner_t owner_;
};

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
volatile bool request_rearm   = false;
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
// CLOCKS_FRAGMENT wire schema. The CLOCKS publisher consumes this handoff and owns serialization.

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
// always serialized with the canonical campaign row.  There is no runtime gate mode.

// Defined with the watchdog/recovery state later in this translation unit.
bool clocks_watchdog_campaign_armed(void);
static void recover_lifecycle_abort(const char* reason);
static uint32_t clocks_row_lifecycle_science_hold_flags(void);
static FLASHMEM bool recover_proof_driven_release_try(uint32_t pps_sequence);

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
// X+1 out of science so no scientific accumulator consumes an interval whose
// opening endpoint was already rejected.  The canonical campaign row still persists.
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

static void clocks_row_antecedent_hold_arm_exact(
    uint32_t source_pps,
    uint32_t lane_mask) {
  if (source_pps == 0U) return;
  if (lane_mask == 0U) {
    lane_mask = CLOCKS_ROW_LANE_OCXO1 | CLOCKS_ROW_LANE_OCXO2;
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

void clocks_row_hold_successor(uint32_t source_pps, uint32_t lane_mask) {
  clocks_row_antecedent_hold_arm_exact(source_pps, lane_mask);
}

static void clocks_row_antecedent_hold_arm(
    clocks_row_objection_reason_t reason,
    uint32_t lane_mask,
    uint32_t source_pps) {
  if (!clocks_row_objection_requires_antecedent_hold(reason)) return;
  clocks_row_antecedent_hold_arm_exact(source_pps, lane_mask);
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

  // RECOVER science release is evidence-driven, not row-count-driven.  This
  // court runs before lifecycle hold_flags are authored so a row whose two OCXO
  // lanes prove fresh post-RECOVER ancestry may enter Alpha Welford/TAU/PPB and
  // science custody on this exact completed PPS.
  (void)recover_proof_driven_release_try(pps_sequence);

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
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE = 13;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_RECOVER_PROOF_GATE = 14;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE = 15;
static constexpr uint32_t CAMPAIGN_RECORD_STAGE_REARM_GATE = 16;
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
// Welford authority is Delta Cycles; a missing PhaseLedger interval
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

// At most two immutable completed campaign records may await the CLOCKS publisher.
// The typed snapshots live in RAM2 so the completed-row path never places the
// handoff object on the scarce DTCM stack. The CLOCKS publisher consumes a record
// only when the requested PPS sequence matches exactly; an unconsumed committed
// record is never overwritten.
//
// A long observer outage is different from a producer/science failure.  If this
// bounded handoff fills while a stable STARTED campaign continues, Beta retires
// the new Pi observation at the producer boundary instead of surrendering the
// campaign.  The producer never advances the consumer index.  A monotonically
// advancing observer-loss watermark tells the CLOCKS publisher which older
// undelivered observations it may explicitly retire once publication runs again.
static constexpr uint32_t CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY = 2U;
alignas(CLOCKS_RAM2_CACHE_LINE_BYTES)
static clocks_fragment_campaign_snapshot_t
    g_clocks_fragment_campaign_queue[CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY]
        DMAMEM = {};
static volatile uint32_t g_clocks_fragment_campaign_queue_read = 0U;
static volatile uint32_t g_clocks_fragment_campaign_queue_write = 0U;
static uint32_t g_clocks_fragment_campaign_record_stage_count = 0U;
static uint32_t g_clocks_fragment_campaign_record_take_count = 0U;
static uint32_t g_clocks_fragment_campaign_record_backlog_count = 0U;

// Observer-loss testimony is intentionally scalar and bounded.  It does not
// replay or reconstruct missing rows; it proves only that a stable Beta producer
// continued after the Pi-facing handoff saturated.  The watermark is physical
// completed-row identity, not campaign public_count.
static bool     g_clocks_fragment_campaign_observer_loss_seen = false;
static uint32_t g_clocks_fragment_campaign_observer_retired_through_sequence = 0U;
static uint32_t g_clocks_fragment_campaign_observer_drop_count = 0U;
static uint32_t g_clocks_fragment_campaign_observer_drop_first_sequence = 0U;
static uint32_t g_clocks_fragment_campaign_observer_drop_last_sequence = 0U;
static uint32_t g_clocks_fragment_campaign_observer_drop_last_public_count = 0U;
static uint32_t g_clocks_fragment_campaign_observer_queue_retire_count = 0U;
static uint32_t g_clocks_fragment_campaign_observer_queue_retire_last_sequence = 0U;

static inline void clocks_fragment_campaign_queue_barrier(void) {
  __asm__ volatile("dmb" ::: "memory");
}

static uint32_t clocks_fragment_campaign_queue_depth(void) {
  const uint32_t write = g_clocks_fragment_campaign_queue_write;
  clocks_fragment_campaign_queue_barrier();
  const uint32_t read = g_clocks_fragment_campaign_queue_read;
  clocks_fragment_campaign_queue_barrier();
  return write - read;
}

static clocks_fragment_campaign_snapshot_t*
clocks_fragment_campaign_queue_acquire_write(void) {
  const uint32_t write = g_clocks_fragment_campaign_queue_write;
  clocks_fragment_campaign_queue_barrier();
  const uint32_t read = g_clocks_fragment_campaign_queue_read;
  if ((write - read) >= CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY) {
    return nullptr;
  }
  return &g_clocks_fragment_campaign_queue[
      write % CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY];
}

static void clocks_fragment_campaign_queue_commit_write(void) {
  const uint32_t write = g_clocks_fragment_campaign_queue_write;
  clocks_fragment_campaign_queue_barrier();
  const uint32_t read = g_clocks_fragment_campaign_queue_read;
  if ((write - read) >= CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY) {
    __builtin_trap();
  }
  clocks_fragment_campaign_queue_barrier();
  g_clocks_fragment_campaign_queue_write = write + 1U;
}

static clocks_fragment_campaign_snapshot_t*
clocks_fragment_campaign_queue_front(void) {
  const uint32_t read = g_clocks_fragment_campaign_queue_read;
  clocks_fragment_campaign_queue_barrier();
  const uint32_t write = g_clocks_fragment_campaign_queue_write;
  if (read == write) return nullptr;
  return &g_clocks_fragment_campaign_queue[
      read % CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY];
}

static uint32_t clocks_fragment_campaign_queue_front_sequence(void) {
  clocks_fragment_campaign_snapshot_t* front =
      clocks_fragment_campaign_queue_front();
  return front ? front->completed_second_sequence : 0U;
}

static void clocks_fragment_campaign_queue_release(void) {
  const uint32_t read = g_clocks_fragment_campaign_queue_read;
  clocks_fragment_campaign_queue_barrier();
  if (read == g_clocks_fragment_campaign_queue_write) __builtin_trap();
  g_clocks_fragment_campaign_queue[
      read % CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY] =
      clocks_fragment_campaign_snapshot_t{};
  clocks_fragment_campaign_queue_barrier();
  g_clocks_fragment_campaign_queue_read = read + 1U;
}

static void clocks_fragment_campaign_queue_reset(void) {
  for (uint32_t i = 0U; i < CLOCKS_FRAGMENT_CAMPAIGN_QUEUE_CAPACITY; ++i) {
    g_clocks_fragment_campaign_queue[i] = clocks_fragment_campaign_snapshot_t{};
  }
  g_clocks_fragment_campaign_queue_read = 0U;
  g_clocks_fragment_campaign_queue_write = 0U;
  clocks_fragment_campaign_queue_barrier();
}

static void clocks_fragment_campaign_observer_loss_reset_epoch(void) {
  g_clocks_fragment_campaign_observer_loss_seen = false;
  g_clocks_fragment_campaign_observer_retired_through_sequence = 0U;
}

static bool clocks_fragment_campaign_sequence_observer_retired(
    uint32_t sequence) {
  if (!g_clocks_fragment_campaign_observer_loss_seen || sequence == 0U) {
    return false;
  }
  return (int32_t)(
      g_clocks_fragment_campaign_observer_retired_through_sequence -
      sequence) >= 0;
}

static void clocks_fragment_campaign_note_observer_drop(
    uint32_t completed_second_sequence,
    uint32_t public_count) {
  if (completed_second_sequence == 0U) __builtin_trap();
  if (!g_clocks_fragment_campaign_observer_loss_seen) {
    g_clocks_fragment_campaign_observer_loss_seen = true;
    g_clocks_fragment_campaign_observer_drop_first_sequence =
        completed_second_sequence;
  }
  g_clocks_fragment_campaign_observer_retired_through_sequence =
      completed_second_sequence;
  g_clocks_fragment_campaign_observer_drop_last_sequence =
      completed_second_sequence;
  g_clocks_fragment_campaign_observer_drop_last_public_count = public_count;
  g_clocks_fragment_campaign_observer_drop_count++;
}

static void clocks_fragment_campaign_retire_observer_queue_through(
    uint32_t sequence) {
  while (true) {
    clocks_fragment_campaign_snapshot_t* front =
        clocks_fragment_campaign_queue_front();
    if (!front) return;
    if ((int32_t)(sequence - front->completed_second_sequence) < 0) return;
    if (!clocks_fragment_campaign_sequence_observer_retired(
            front->completed_second_sequence)) {
      __builtin_trap();
    }
    g_clocks_fragment_campaign_observer_queue_retire_last_sequence =
        front->completed_second_sequence;
    g_clocks_fragment_campaign_observer_queue_retire_count++;
    clocks_fragment_campaign_queue_release();
  }
}

static volatile uint32_t g_clocks_beta_pps_owner = 0U;

struct clocks_beta_pps_owner_guard_t {
  clocks_beta_pps_owner_guard_t() {
    uint32_t expected = 0U;
    if (!__atomic_compare_exchange_n(&g_clocks_beta_pps_owner,
                                     &expected,
                                     1U,
                                     false,
                                     __ATOMIC_ACQ_REL,
                                     __ATOMIC_ACQUIRE)) {
      __builtin_trap();
    }
  }
  ~clocks_beta_pps_owner_guard_t() {
    __atomic_store_n(&g_clocks_beta_pps_owner, 0U, __ATOMIC_RELEASE);
  }
};

// ============================================================================
// CLOCKS_FRAGMENT publication ownership
// ============================================================================
//
// CLOCKS owns its canonical publication symmetrically with PHOTONS. Alpha and
// Beta author the immutable instrument/campaign facts; this foreground publisher
// owns exact-sequence serialization, transport retry custody, and final publish().
// SYSTEM remains only the feature-status registry queried while building the
// curated readiness tree.

static constexpr uint64_t CLOCKS_FRAGMENT_RETRY_DELAY_NS = 25000000ULL;  // 25 ms
// Exact publication retry is deliberately bounded in every stable lifecycle.
// D1 request/response transport capacity is reserved below the D0/D2 ceiling;
// a CLOCKS_FRAGMENT can therefore be rejected while the command plane still has
// lawful transport custody.  Never let that ordinary backpressure turn into a
// permanent 40 Hz foreground retry loop that prevents D1 from getting a turn.
//
// A public STARTED campaign gets a substantially wider court than ambient
// publication. After the typed Alpha/Beta row passes the exact-row court, Payload
// construction is a hard invariant: a failed add/add_object never returns to this
// client. The only recoverable publication failure here is transport refusing
// custody of the completed wire observation. Retire that undeliverable Pi
// observation explicitly and let Beta continue authoring subsequent campaign
// seconds. Ambiguous/legacy retry state still fails through WATCHDOG_ANOMALY.
// RECOVERING also remains transactional: retry exhaustion aborts that recovery so
// Pi may adjudicate/retry while Alpha lives.
static constexpr uint32_t CLOCKS_FRAGMENT_IDLE_RETRY_MAX_ATTEMPTS = 4U;
static constexpr uint32_t CLOCKS_FRAGMENT_ACTIVE_RETRY_MAX_ATTEMPTS = 16U;

static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_NONE = 0U;
// Payload-build reason IDs remain only so existing report/log decoders retain
// their historical vocabulary. Defenseless Payload clients can no longer author
// these reasons: construction either returns complete or does not return at all.
static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_CAMPAIGN_EMBED = 1U;
static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_CLOCKS_PAYLOAD = 2U;
static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_TRANSPORT_ENQUEUE = 3U;
static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_CLOCKS_OBJECT = 4U;
static constexpr uint32_t CLOCKS_FRAGMENT_RETRY_REASON_FEATURES_OBJECT = 5U;

static bool g_clocks_fragment_publication_initialized = false;

// --------------------------------------------------------------
// PPS-aligned CLOCKS_FRAGMENT publication custody
// --------------------------------------------------------------
// process_interrupt contributes only the completed PPS/VCLOCK sequence. CLOCKS
// later takes one typed CLOCKS snapshot in foreground ALAP service and authors the
// complete CLOCKS_FRAGMENT, including optional campaign facts. The former campaign
// readiness notification remains a same-owner wake after Alpha freezes the exact row.
static volatile uint32_t g_clocks_fragment_publication_pending_sequence = 0U;
static volatile bool g_clocks_fragment_publication_pending = false;
static volatile bool g_clocks_fragment_publication_service_armed = false;
// Foreground publication has one owner from snapshot acquisition through queue
// release. A wake observed while that owner is active remains represented by
// scalar pending/retry state and is armed only after ownership returns.
static volatile uint32_t g_clocks_fragment_publication_service_active = 0U;
static volatile uint32_t g_clocks_fragment_publication_service_rearm_deferred = 0U;
static timepop_handle_t g_clocks_fragment_publication_service_handle =
    TIMEPOP_INVALID_HANDLE;
static uint32_t g_clocks_fragment_publication_service_generation = 0U;
static uint32_t g_clocks_fragment_publication_service_armed_generation = 0U;
static uint32_t g_clocks_fragment_publication_service_forced_rearm_count = 0U;
static uint32_t g_clocks_fragment_publication_service_cancel_count = 0U;
static uint32_t g_clocks_fragment_publication_service_cancel_failure_count = 0U;
static uint32_t g_clocks_fragment_publication_service_stale_callback_count = 0U;
static bool g_clocks_fragment_publication_campaign_ready_last_valid = false;
static uint32_t g_clocks_fragment_publication_campaign_ready_last_sequence = 0U;
static uint32_t g_clocks_fragment_publication_campaign_ready_signal_count = 0U;
static uint32_t g_clocks_fragment_publication_campaign_ready_repeat_count = 0U;
static uint32_t g_clocks_fragment_publication_publish_count = 0U;
static uint32_t g_clocks_fragment_publication_publish_reject_count = 0U;
static uint32_t g_clocks_fragment_publication_coalesce_count = 0U;
static uint32_t g_clocks_fragment_publication_service_arm_failures = 0U;
static uint32_t g_clocks_fragment_publication_campaign_rows_embedded = 0U;
static uint32_t g_clocks_fragment_publication_campaign_ready_enrichment_retry_count = 0U;
static uint32_t g_clocks_fragment_publication_campaign_retry_count = 0U;
static uint32_t g_clocks_fragment_publication_campaign_row_embed_fail_count = 0U;
static bool g_clocks_fragment_publication_retry_snapshot_valid = false;
static bool g_clocks_fragment_publication_retry_pi_only = false;
static uint32_t g_clocks_fragment_publication_retry_sequence = 0U;
static uint32_t g_clocks_fragment_publication_retry_attempt_count = 0U;
static uint32_t g_clocks_fragment_publication_retry_reason_id =
    CLOCKS_FRAGMENT_RETRY_REASON_NONE;
static uint32_t g_clocks_fragment_publication_retry_tick_hold_count = 0U;
static uint32_t g_clocks_fragment_publication_retry_schedule_count = 0U;
static uint32_t g_clocks_fragment_publication_retry_success_count = 0U;
static uint32_t g_clocks_fragment_publication_active_retry_exhaustion_count = 0U;
static uint32_t g_clocks_fragment_publication_recovery_retry_abort_count = 0U;
static uint32_t g_clocks_fragment_publication_last_exhausted_sequence = 0U;
static uint32_t g_clocks_fragment_publication_last_exhausted_attempt_count = 0U;
static uint32_t g_clocks_fragment_publication_last_exhausted_reason_id =
    CLOCKS_FRAGMENT_RETRY_REASON_NONE;
static uint32_t g_clocks_fragment_publication_last_exhausted_pending_sequence = 0U;
static bool g_clocks_fragment_publication_last_tick_valid = false;
static uint32_t g_clocks_fragment_publication_last_tick_sequence = 0U;
static uint32_t g_clocks_fragment_publication_recovery_reset_count = 0U;
static uint32_t g_clocks_fragment_publication_recovery_retired_retry_sequence = 0U;
static uint32_t g_clocks_fragment_publication_recovery_retired_pending_sequence = 0U;
static uint32_t g_clocks_fragment_publication_recovery_retired_campaign_sequence = 0U;
static uint32_t g_clocks_fragment_publication_watchdog_reset_count = 0U;
static uint32_t g_clocks_fragment_publication_watchdog_retired_retry_sequence = 0U;
static uint32_t g_clocks_fragment_publication_watchdog_retired_pending_sequence = 0U;
static uint32_t g_clocks_fragment_publication_watchdog_retired_campaign_sequence = 0U;
static uint32_t g_clocks_fragment_publication_idle_retry_abandon_count = 0U;
static uint32_t g_clocks_fragment_publication_idle_retry_last_sequence = 0U;
static uint32_t g_clocks_fragment_publication_idle_retry_last_attempt_count = 0U;
static uint32_t g_clocks_fragment_publication_observer_retire_count = 0U;
static uint32_t g_clocks_fragment_publication_observer_retire_last_sequence = 0U;

// The CLOCKS publication handoff is intentionally large and must never become a
// foreground stack local.  Publication now crosses one explicit SPSC ownership
// boundary: the capture side owns the write slot until commit; the serializer owns
// the read slot until publish succeeds or the observation is explicitly retired.
// No Payload builder may reach back into live CLOCKS/SYSTEM state.
static constexpr uint32_t CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY = 2U;

struct clocks_fragment_feature_value_t {
  bool present = false;
  system_feature_status_t status = system_feature_status_t::INITIALIZING;
};

struct clocks_fragment_features_snapshot_t {
  clocks_fragment_feature_value_t alpha_epoch{};
  clocks_fragment_feature_value_t dwt_calibration{};
  clocks_fragment_feature_value_t ocxo_public_origin{};
  clocks_fragment_feature_value_t static_prediction{};
  clocks_fragment_feature_value_t counter32_lineage{};
  clocks_fragment_feature_value_t observed_edge_authority{};
  clocks_fragment_feature_value_t pps_vclock_authority{};
  clocks_fragment_feature_value_t qtimer_counter_custody{};
};

struct clocks_fragment_publication_item_t {
  uint32_t sequence = 0U;
  clocks_fragment_snapshot_t clocks{};
  clocks_fragment_features_snapshot_t features{};
};

alignas(CLOCKS_RAM2_CACHE_LINE_BYTES)
static clocks_fragment_publication_item_t
    g_clocks_fragment_publication_queue[CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY]
        DMAMEM = {};
static volatile uint32_t g_clocks_fragment_publication_queue_read = 0U;
static volatile uint32_t g_clocks_fragment_publication_queue_write = 0U;

static inline void clocks_fragment_publication_queue_barrier(void) {
  __asm__ volatile("dmb" ::: "memory");
}

static uint32_t clocks_fragment_publication_queue_depth(void) {
  const uint32_t write = g_clocks_fragment_publication_queue_write;
  clocks_fragment_publication_queue_barrier();
  const uint32_t read = g_clocks_fragment_publication_queue_read;
  clocks_fragment_publication_queue_barrier();
  return write - read;
}

static clocks_fragment_publication_item_t*
clocks_fragment_publication_queue_acquire_write(void) {
  const uint32_t write = g_clocks_fragment_publication_queue_write;
  clocks_fragment_publication_queue_barrier();
  const uint32_t read = g_clocks_fragment_publication_queue_read;
  if ((write - read) >= CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY) {
    __builtin_trap();
  }
  return &g_clocks_fragment_publication_queue[
      write % CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY];
}

static void clocks_fragment_publication_queue_commit_write(void) {
  const uint32_t write = g_clocks_fragment_publication_queue_write;
  clocks_fragment_publication_queue_barrier();
  const uint32_t read = g_clocks_fragment_publication_queue_read;
  if ((write - read) >= CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY) {
    __builtin_trap();
  }
  clocks_fragment_publication_queue_barrier();
  g_clocks_fragment_publication_queue_write = write + 1U;
}

static clocks_fragment_publication_item_t*
clocks_fragment_publication_queue_front(void) {
  const uint32_t read = g_clocks_fragment_publication_queue_read;
  clocks_fragment_publication_queue_barrier();
  const uint32_t write = g_clocks_fragment_publication_queue_write;
  if (read == write) return nullptr;
  return &g_clocks_fragment_publication_queue[
      read % CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY];
}

static void clocks_fragment_publication_queue_release(void) {
  const uint32_t read = g_clocks_fragment_publication_queue_read;
  clocks_fragment_publication_queue_barrier();
  if (read == g_clocks_fragment_publication_queue_write) __builtin_trap();
  clocks_fragment_publication_item_t& item =
      g_clocks_fragment_publication_queue[
          read % CLOCKS_FRAGMENT_PUBLICATION_QUEUE_CAPACITY];
  memset(&item, 0, sizeof(item));
  clocks_fragment_publication_queue_barrier();
  g_clocks_fragment_publication_queue_read = read + 1U;
}

static void clocks_fragment_publication_queue_reset(void) {
  memset(g_clocks_fragment_publication_queue, 0,
         sizeof(g_clocks_fragment_publication_queue));
  g_clocks_fragment_publication_queue_read = 0U;
  g_clocks_fragment_publication_queue_write = 0U;
  clocks_fragment_publication_queue_barrier();
}

// CLOCKS_FRAGMENT serialization is a bounded, single-owner foreground workload.
// Large canonical byte stores live in RAM2 so they never enter the general heap,
// but every Payload control block (guards, capacity, generation, fingerprint)
// lives in ordinary RAM1. Nested fixed scratch is entirely RAM1. Inline-only
// locals remain ordinary Payloads.
//
// These capacities are part of the canonical schema contract, not allocator
// tuning. Growth beyond one of them is a fail-hard FIXED_CAPACITY event and must
// be accompanied by an explicit schema/storage review.
#define CLOCKS_FRAGMENT_FIXED_RAM2(name, capacity)                         \
  static_assert(((capacity) % CLOCKS_RAM2_CACHE_LINE_BYTES) == 0U,        \
                "CLOCKS RAM2 fixed store must fill cache lines");         \
  alignas(CLOCKS_RAM2_CACHE_LINE_BYTES)                                   \
  static uint8_t name##_storage[capacity] DMAMEM;                         \
  static Payload name(                                                    \
      Payload::StorageMode::FIXED,                                        \
      name##_storage,                                                     \
      sizeof(name##_storage))

#define CLOCKS_FRAGMENT_FIXED_RAM1(name, capacity)                         \
  alignas(Payload::FIXED_STORAGE_ALIGNMENT)                               \
  static uint8_t name##_storage[capacity];                                \
  static Payload name(                                                    \
      Payload::StorageMode::FIXED,                                        \
      name##_storage,                                                     \
      sizeof(name##_storage))

// Canonical parents. Conservative max-width schema sizing leaves explicit
// headroom over the currently observed mature rows.
CLOCKS_FRAGMENT_FIXED_RAM2(g_clocks_fragment_root_payload, 16384U);
CLOCKS_FRAGMENT_FIXED_RAM2(g_clocks_fragment_campaign_payload, 6144U);
CLOCKS_FRAGMENT_FIXED_RAM2(g_clocks_fragment_clocks_payload, 12288U);
CLOCKS_FRAGMENT_FIXED_RAM2(g_clocks_fragment_stats_payload, 8192U);

// Nested fragment scratch that can exceed Payload's inline store.
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_features_teensy_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_features_root_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_welford_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_ppb_endpoint_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_ppb_window_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_ppb_checkpoint_payload, 3072U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_ppb_second_append_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_ppb_minute_append_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_stats_clock_payload, 1024U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_tau_state_payload, 1280U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_aux_welford_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_raw_lane_payload, 768U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_raw_payload, 3072U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_science_payload, 768U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_candidate_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_candidates_payload, 1536U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_campaign_ocxo_payload, 2048U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_campaign_disposition_payload, 512U);
CLOCKS_FRAGMENT_FIXED_RAM1(g_clocks_fragment_campaign_recovery_payload, 512U);

#undef CLOCKS_FRAGMENT_FIXED_RAM1
#undef CLOCKS_FRAGMENT_FIXED_RAM2

static void clocks_fragment_schedule_publish(void);

static inline uint32_t clocks_fragment_current_ipsr(void) {
#if defined(__arm__)
  uint32_t value = 0U;
  __asm__ volatile("mrs %0, ipsr" : "=r"(value) :: "memory");
  return value;
#else
  return 0U;
#endif
}

static void clocks_fragment_feature_snapshot_take(
    clocks_fragment_feature_value_t& out,
    const char* subsystem,
    const char* name) {
  out = clocks_fragment_feature_value_t{};
  if (!system_feature_has(subsystem, name)) return;
  const char* status = system_feature_get_status(subsystem, name);
  if (!status || !system_feature_status_parse(status, &out.status)) {
    __builtin_trap();
  }
  out.present = true;
}

static void clocks_fragment_features_snapshot_take(
    clocks_fragment_features_snapshot_t& out) {
  out = clocks_fragment_features_snapshot_t{};
  clocks_fragment_feature_snapshot_take(
      out.alpha_epoch, "CLOCKS", "ALPHA_EPOCH");
  clocks_fragment_feature_snapshot_take(
      out.dwt_calibration, "CLOCKS", "DWT_CALIBRATION");
  clocks_fragment_feature_snapshot_take(
      out.ocxo_public_origin, "CLOCKS", "OCXO_PUBLIC_ORIGIN");
  clocks_fragment_feature_snapshot_take(
      out.static_prediction, "CLOCKS", "STATIC_PREDICTION");
  clocks_fragment_feature_snapshot_take(
      out.counter32_lineage, "INTERRUPT", "COUNTER32_LINEAGE");
  clocks_fragment_feature_snapshot_take(
      out.observed_edge_authority, "INTERRUPT", "OBSERVED_EDGE_AUTHORITY");
  clocks_fragment_feature_snapshot_take(
      out.pps_vclock_authority, "INTERRUPT", "PPS_VCLOCK_AUTHORITY");
  clocks_fragment_feature_snapshot_take(
      out.qtimer_counter_custody, "INTERRUPT", "QTIMER_COUNTER_CUSTODY");
}

static FLASHMEM Payload& clocks_fragment_features_payload(
    const clocks_fragment_features_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload clocks;
  if (snapshot.alpha_epoch.present)
    clocks.add("ALPHA_EPOCH", system_feature_status_str(snapshot.alpha_epoch.status));
  if (snapshot.dwt_calibration.present)
    clocks.add("DWT_CALIBRATION", system_feature_status_str(snapshot.dwt_calibration.status));
  if (snapshot.ocxo_public_origin.present)
    clocks.add("OCXO_PUBLIC_ORIGIN", system_feature_status_str(snapshot.ocxo_public_origin.status));
  if (snapshot.static_prediction.present)
    clocks.add("STATIC_PREDICTION", system_feature_status_str(snapshot.static_prediction.status));

  Payload interrupt;
  if (snapshot.counter32_lineage.present)
    interrupt.add("COUNTER32_LINEAGE", system_feature_status_str(snapshot.counter32_lineage.status));
  if (snapshot.observed_edge_authority.present)
    interrupt.add("OBSERVED_EDGE_AUTHORITY",
                  system_feature_status_str(snapshot.observed_edge_authority.status));
  if (snapshot.pps_vclock_authority.present)
    interrupt.add("PPS_VCLOCK_AUTHORITY", system_feature_status_str(snapshot.pps_vclock_authority.status));
  if (snapshot.qtimer_counter_custody.present)
    interrupt.add("QTIMER_COUNTER_CUSTODY",
                  system_feature_status_str(snapshot.qtimer_counter_custody.status));

  Payload& teensy = g_clocks_fragment_features_teensy_payload;
  teensy.clear();
  if (!clocks.empty()) teensy.add_object("CLOCKS", clocks);
  if (!interrupt.empty()) teensy.add_object("INTERRUPT", interrupt);

  Payload& root = g_clocks_fragment_features_root_payload;
  root.clear();
  root.add_object("TEENSY", teensy);
  return root;
}

static void clocks_fragment_publication_ensure_initialized(void) {
  if (g_clocks_fragment_publication_initialized) return;

  clocks_fragment_publication_queue_reset();
  g_clocks_fragment_publication_retry_snapshot_valid = false;
  g_clocks_fragment_publication_retry_pi_only = false;
  g_clocks_fragment_publication_retry_sequence = 0U;
  g_clocks_fragment_publication_retry_attempt_count = 0U;
  g_clocks_fragment_publication_retry_reason_id =
      CLOCKS_FRAGMENT_RETRY_REASON_NONE;
  g_clocks_fragment_publication_retry_tick_hold_count = 0U;
  g_clocks_fragment_publication_retry_schedule_count = 0U;
  g_clocks_fragment_publication_retry_success_count = 0U;

  g_clocks_fragment_publication_initialized = true;
}

// ============================================================================
// CLOCKS_FRAGMENT — normalized canonical Teensy observation
// ============================================================================
//
// CLOCKS owns serialization and publication custody for its own canonical feed.
// Alpha supplies one always-on instrument snapshot plus, when available, Beta supplies one campaign-relative
// TEMPEST delta.  The wire format deliberately contains no full Better-Buckets
// ring mirror and no periodic transport/process/Payload flight recorder.
// Compact Alpha-authored checkpoint/proof testimony is carried at 1 Hz so Pi
// may maintain a literal synthetic recovery checkpoint without re-authoring
// the instrument math.  raw_cycles is retained as
// the compact four-rail sanity-check surface; deeper forensics remain on focused
// reports.

// ============================================================================
// CLOCKS-owned canonical serialization
// ============================================================================

static void clocks_fragment_add_welford(
    Payload& parent,
    const char* key,
    const clocks_fragment_welford_snapshot_t& sample) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_welford_payload;
  value.clear();
  value.add("n", sample.n);
  value.add("mean", toFixedDecimal(sample.mean, 12));
  value.add("m2", toFixedDecimal(sample.m2, 12));
  value.add("stddev", toFixedDecimal(sample.stddev, 6));
  value.add("stderr", toFixedDecimal(sample.stderr_value, 6));
  value.add("min", toFixedDecimal(sample.min, 12));
  value.add("max", toFixedDecimal(sample.max, 12));
  parent.add_object(key, value);
}

static void clocks_fragment_add_ppb_bucket(
    Payload& buckets,
    const char* key,
    const clocks_fragment_ppb_value_snapshot_t& sample) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  if (sample.sample_count == 0ULL) return;
  buckets.add(key, toFixedDecimal(sample.ppb, 6));
}

static void clocks_fragment_add_ppb_buckets(
    Payload& clock,
    const clocks_fragment_ppb_buckets_snapshot_t& buckets) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload values;
  clocks_fragment_add_ppb_bucket(values, "10_min", buckets.minute_10);
  clocks_fragment_add_ppb_bucket(values, "60_min", buckets.minute_60);
  clocks_fragment_add_ppb_bucket(values, "8_hour", buckets.hour_8);
  clocks_fragment_add_ppb_bucket(values, "24_hour", buckets.hour_24);
  clocks_fragment_add_ppb_bucket(values, "total", buckets.total);
  clock.add_object("ppb_buckets", values);
}

static void clocks_fragment_add_ppb_endpoint(
    Payload& parent,
    const char* key,
    const clocks_fragment_ppb_endpoint_snapshot_t& endpoint) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_ppb_endpoint_payload;
  value.clear();
  value.add("reference_ns", endpoint.reference_ns);
  value.add("dwt_error_cycles", toFixedDecimal(endpoint.dwt_error_cycles, 12));
  value.add("ocxo1_error_ns", endpoint.ocxo1_error_ns);
  value.add("ocxo2_error_ns", endpoint.ocxo2_error_ns);
  value.add("rolling_sequence", endpoint.rolling_sequence);
  value.add("interval_count", endpoint.interval_count);
  parent.add_object(key, value);
}

static void clocks_fragment_add_ppb_window_proof(
    Payload& parent,
    const char* key,
    const clocks_fragment_ppb_window_proof_snapshot_t& proof) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_ppb_window_payload;
  value.clear();
  value.add("valid", proof.valid);
  value.add("sample_count", proof.sample_count);
  if (proof.valid) {
    clocks_fragment_add_ppb_endpoint(value, "anchor", proof.anchor);
  }
  parent.add_object(key, value);
}

static void clocks_fragment_add_ppb_checkpoint(
    Payload& parent,
    const clocks_fragment_ppb_checkpoint_delta_snapshot_t& checkpoint) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_ppb_checkpoint_payload;
  value.clear();
  value.add("schema", "CLOCKS_PPB_CHECKPOINT_DELTA_V1");
  value.add("valid", checkpoint.valid);
  value.add("rolling_sequence", checkpoint.rolling_sequence);
  value.add("second_count", checkpoint.second_count);
  value.add("minute_count", checkpoint.minute_count);
  value.add("last_minute_key", checkpoint.last_minute_key);
  value.add("origin_valid", checkpoint.origin_valid);

  if (checkpoint.valid) {
    clocks_fragment_add_ppb_endpoint(
        value, "current", checkpoint.current);
  }
  if (checkpoint.origin_valid) {
    clocks_fragment_add_ppb_endpoint(
        value, "origin", checkpoint.origin);
  }

  clocks_fragment_add_ppb_window_proof(
      value, "10_min", checkpoint.minute_10);
  clocks_fragment_add_ppb_window_proof(
      value, "60_min", checkpoint.minute_60);
  clocks_fragment_add_ppb_window_proof(
      value, "8_hour", checkpoint.hour_8);
  clocks_fragment_add_ppb_window_proof(
      value, "24_hour", checkpoint.hour_24);

  Payload& second_append = g_clocks_fragment_ppb_second_append_payload;
  second_append.clear();
  second_append.add("valid", checkpoint.second_append_valid);
  if (checkpoint.second_append_valid) {
    clocks_fragment_add_ppb_endpoint(
        second_append, "endpoint", checkpoint.second_append);
  }
  value.add_object("second_append", second_append);

  Payload& minute_append = g_clocks_fragment_ppb_minute_append_payload;
  minute_append.clear();
  minute_append.add("valid", checkpoint.minute_append_valid);
  if (checkpoint.minute_append_valid) {
    clocks_fragment_add_ppb_endpoint(
        minute_append, "endpoint", checkpoint.minute_append);
  }
  value.add_object("minute_append", minute_append);

  parent.add_object("rolling_ppb_checkpoint", value);
}

static void clocks_fragment_add_stats_clock(
    Payload& parent,
    const char* key,
    const clocks_fragment_stats_clock_snapshot_t& clock) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_stats_clock_payload;
  value.clear();
  clocks_fragment_add_welford(value, "welford", clock.welford);
  if (clock.frequency_present) {
    value.add("tau", toFixedDecimal(clock.tau, 12));
    value.add("ppb", toFixedDecimal(clock.ppb, 3));
  }
  clocks_fragment_add_ppb_buckets(value, clock.ppb_buckets);
  parent.add_object(key, value);
}

static void clocks_fragment_add_tau_state(
    Payload& parent,
    const char* key,
    const clocks_fragment_tau_recovery_snapshot_t& state) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_tau_state_payload;
  value.clear();
  value.add("valid", state.valid);
  value.add("reset_count", state.reset_count);
  value.add("sample_count", state.sample_count);
  value.add("interval_count", state.interval_count);
  value.add("reject_count", state.reject_count);
  value.add("gap_reset_count", state.gap_reset_count);
  value.add("last_pps_sequence", state.last_pps_sequence);
  value.add("last_interval_pps_sequence", state.last_interval_pps_sequence);
  value.add("first_refined_ns", state.first_refined_ns);
  value.add("last_refined_ns", state.last_refined_ns);
  value.add("last_fast_residual_ns", state.last_fast_residual_ns);
  value.add("cumulative_reference_ns", state.cumulative_reference_ns);
  value.add("cumulative_clock_ns", state.cumulative_clock_ns);
  value.add("cumulative_clock_ns_exact",
            toFixedDecimal(state.cumulative_clock_ns_exact, 12));
  value.add("mean_x", toFixedDecimal(state.mean_x, 12));
  value.add("mean_y", toFixedDecimal(state.mean_y, 12));
  value.add("sxx", toFixedDecimal(state.sxx, 12));
  value.add("sxy", toFixedDecimal(state.sxy, 12));
  value.add("syy", toFixedDecimal(state.syy, 12));
  value.add("interval_mean_ppb", toFixedDecimal(state.interval_mean_ppb, 12));
  value.add("interval_m2_ppb", toFixedDecimal(state.interval_m2_ppb, 12));
  parent.add_object(key, value);
}

static void clocks_fragment_add_stats(
    Payload& parent,
    const clocks_fragment_stats_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& stats = g_clocks_fragment_stats_payload;
  stats.clear();

  stats.add("schema", "CLOCKS_INSTRUMENT_STATS_V4");
  stats.add("snapshot_ok", snapshot.snapshot_ok);
  stats.add("valid", snapshot.valid);
  stats.add("reset_count", snapshot.reset_count);
  stats.add("update_count", snapshot.update_count);
  stats.add("last_pps_sequence", snapshot.last_pps_sequence);
  stats.add("rolling_ppb_current_sequence",
            snapshot.rolling_ppb_current_sequence);
  stats.add("rolling_ppb_endpoint_admitted",
            snapshot.rolling_ppb_endpoint_admitted);
  stats.add("rolling_ppb_interval_advanced",
            snapshot.rolling_ppb_interval_advanced);
  clocks_fragment_add_ppb_checkpoint(
      stats, snapshot.rolling_ppb_checkpoint);
  stats.add("completed_row_coherent", snapshot.completed_row_coherent);

  clocks_fragment_add_stats_clock(stats, "gnss", snapshot.gnss);
  clocks_fragment_add_stats_clock(stats, "dwt", snapshot.dwt);
  clocks_fragment_add_stats_clock(stats, "vclock", snapshot.vclock);
  clocks_fragment_add_stats_clock(stats, "ocxo1", snapshot.ocxo1);
  clocks_fragment_add_stats_clock(stats, "ocxo2", snapshot.ocxo2);

  // These sufficient-state objects are retained because Holistic Restore must
  // continue the exact Alpha populations rather than manufacture new ones.
  clocks_fragment_add_tau_state(
      stats, "ocxo1_tau_state", snapshot.ocxo1_tau_state);
  clocks_fragment_add_tau_state(
      stats, "ocxo2_tau_state", snapshot.ocxo2_tau_state);

  Payload& auxiliary_welford = g_clocks_fragment_aux_welford_payload;
  auxiliary_welford.clear();
  clocks_fragment_add_welford(
      auxiliary_welford, "pps_witness", snapshot.pps_witness.welford);
  stats.add_object("auxiliary_welford", auxiliary_welford);

  parent.add_object("stats", stats);
  stats.clear();
}

static void clocks_fragment_add_raw_cycles_lane(
    Payload& parent,
    const char* key,
    const clocks_fragment_raw_cycles_lane_t& sample) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& lane = g_clocks_fragment_raw_lane_payload;
  lane.clear();
  lane.add("snapshot_ok", sample.snapshot_ok);
  lane.add("forensics_snapshot_ok", sample.forensics_snapshot_ok);
  lane.add("valid", sample.valid);
  lane.add("completed_interval_count", sample.completed_interval_count);
  lane.add("observed_cycles", sample.observed_cycles);
  lane.add("previous_observed_cycles", sample.previous_observed_cycles);
  lane.add("residual_cycles", sample.residual_cycles);
  lane.add("delay_status", sample.delay_status);
  lane.add("delay_detail_present", sample.delay_detail_present);
  lane.add("delay_by", sample.delay_by);
  lane.add("residual_delay_valid", sample.residual_delay_valid);
  lane.add("residual_delay_cycles", sample.residual_delay_cycles);
  lane.add("residual_delay_by", sample.residual_delay_by);
  lane.add("delay_explains_residual", sample.delay_explains_residual);
  parent.add_object(key, lane);
}

static void clocks_fragment_add_raw_cycles(
    Payload& parent,
    const clocks_fragment_raw_cycles_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& raw = g_clocks_fragment_raw_payload;
  raw.clear();
  clocks_fragment_add_raw_cycles_lane(raw, "pps", snapshot.pps);
  clocks_fragment_add_raw_cycles_lane(raw, "vclock", snapshot.vclock);
  clocks_fragment_add_raw_cycles_lane(raw, "ocxo1", snapshot.ocxo1);
  clocks_fragment_add_raw_cycles_lane(raw, "ocxo2", snapshot.ocxo2);
  parent.add_object("raw_cycles", raw);
}

static void clocks_fragment_add_science(
    Payload& parent,
    const clocks_fragment_science_snapshot_t& science) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& value = g_clocks_fragment_science_payload;
  value.clear();
  value.add("valid", science.valid);
  value.add("science_worthy", science.science_worthy);
  value.add("antecedents_complete", science.antecedents_complete);
  value.add("gnss_interval_ns", science.gnss_interval_ns);
  value.add("clock_interval_ns", science.clock_interval_ns);
  value.add("fast_residual_ns", science.fast_residual_ns);
  value.add("fast_residual_ns_exact",
            toFixedDecimal(science.fast_residual_ns_exact, 6));
  value.add("delta_raw_valid", science.delta_raw_valid);
  value.add("delta_raw_reference_interval_cycles",
            science.delta_raw_reference_interval_cycles);
  value.add("delta_raw_clock_interval_cycles",
            science.delta_raw_clock_interval_cycles);
  value.add("delta_raw_fast_residual_cycles",
            science.delta_raw_fast_residual_cycles);
  parent.add_object("science", value);
}

static Payload& clocks_fragment_clocks_payload(
    const clocks_fragment_live_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& clocks = g_clocks_fragment_clocks_payload;
  clocks.clear();
  clocks.add("schema", "CLOCKS_INSTRUMENT_V1");
  clocks.add("snapshot_ok", snapshot.snapshot_ok);
  clocks.add("valid", snapshot.valid);
  clocks.add("completed_row_coherent", snapshot.completed_row_coherent);
  clocks.add("completed_pps_sequence", snapshot.completed_pps_sequence);
  clocks.add("instrument_age_seconds", snapshot.instrument_age_seconds);

  Payload clockfaces;
  clockfaces.add("gnss_ns", snapshot.instrument_gnss_ns);
  clockfaces.add("dwt_cycles", snapshot.instrument_dwt_cycles);
  clockfaces.add("ocxo1_ns", snapshot.instrument_ocxo1_ns);
  clockfaces.add("ocxo2_ns", snapshot.instrument_ocxo2_ns);
  clockfaces.add("pps_sequence", snapshot.instrument_pps_sequence);
  clocks.add_object("clockfaces", clockfaces);

  Payload anchor;
  anchor.add("dwt_cycles_per_second", snapshot.dwt_cycles_per_second);
  anchor.add("dwt_at_pps_vclock", snapshot.dwt_at_pps_vclock);
  anchor.add("counter32_at_pps_vclock", snapshot.counter32_at_pps_vclock);
  clocks.add_object("anchor", anchor);

  clocks_fragment_add_raw_cycles(clocks, snapshot.raw_cycles);
  clocks_fragment_add_stats(clocks, snapshot.stats);
  return clocks;
}

static const char* clocks_fragment_clock_candidate_status_name(
    clocks_fragment_clock_candidate_status_t status) {
  switch (status) {
    case clocks_fragment_clock_candidate_status_t::SEEDED:
      return "SEEDED";
    case clocks_fragment_clock_candidate_status_t::ADVANCED:
      return "ADVANCED";
    case clocks_fragment_clock_candidate_status_t::DELTA_INVALID:
      return "DELTA_INVALID";
    case clocks_fragment_clock_candidate_status_t::PUBLIC_COUNT_GAP:
      return "PUBLIC_COUNT_GAP";
    case clocks_fragment_clock_candidate_status_t::ARITHMETIC_FAILURE:
      return "ARITHMETIC_FAILURE";
    case clocks_fragment_clock_candidate_status_t::UNAVAILABLE:
    default:
      return "UNAVAILABLE";
  }
}

static void clocks_fragment_add_clock_candidate(
    Payload& parent,
    const char* key,
    const clocks_fragment_clock_candidate_t& candidate) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& out = g_clocks_fragment_candidate_payload;
  out.clear();
  out.add("available", candidate.available);
  out.add("continuity_valid", candidate.continuity_valid);
  out.add("status_id", (uint32_t)candidate.status);
  out.add("status",
          clocks_fragment_clock_candidate_status_name(candidate.status));
  out.add("start_public_count", candidate.start_public_count);
  out.add("last_public_count", candidate.last_public_count);
  out.add("interval_count", candidate.interval_count);
  out.add("ns", candidate.ns);
  out.add("fractional_ns", toFixedDecimal(candidate.fractional_ns, 12));
  out.add("residual_available", candidate.residual_available);
  out.add("residual_ns", candidate.residual_ns);
  out.add("residual_ns_exact",
          toFixedDecimal(candidate.residual_ns_exact, 12));
  parent.add_object(key, out);
}

static void clocks_fragment_add_clock_candidates(
    Payload& parent,
    const clocks_fragment_clock_candidates_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& candidates = g_clocks_fragment_candidates_payload;
  candidates.clear();
  candidates.add("schema", "OCXO_CLOCK_CANDIDATES_V1");
  candidates.add("published_source", snapshot.published_source);
  clocks_fragment_add_clock_candidate(
      candidates, "phaseledger", snapshot.phaseledger);
  clocks_fragment_add_clock_candidate(
      candidates, "delta_cycles", snapshot.delta_cycles);
  candidates.add("comparable", snapshot.comparable);
  candidates.add("delta_cycles_minus_phaseledger_ns",
                 snapshot.delta_cycles_minus_phaseledger_ns);
  candidates.add("residuals_comparable", snapshot.residuals_comparable);
  candidates.add(
      "delta_cycles_minus_phaseledger_residual_ns_exact",
      toFixedDecimal(
          snapshot.delta_cycles_minus_phaseledger_residual_ns_exact, 12));
  parent.add_object("clock_candidates", candidates);
}

static void clocks_fragment_add_campaign_ocxo(
    Payload& parent,
    const char* key,
    const clocks_fragment_clock_candidates_snapshot_t& clock_candidates,
    const clocks_fragment_science_snapshot_t& science) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& lane = g_clocks_fragment_campaign_ocxo_payload;
  lane.clear();
  clocks_fragment_add_clock_candidates(lane, clock_candidates);
  clocks_fragment_add_science(lane, science);
  parent.add_object(key, lane);
}

static void clocks_fragment_add_campaign_ppb(
    Payload& ppb,
    const char* key,
    const clocks_fragment_ppb_value_snapshot_t& sample) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  if (sample.sample_count == 0ULL) return;
  ppb.add(key, toFixedDecimal(sample.ppb, 6));
}

static void clocks_fragment_add_campaign_stats(
    Payload& parent,
    const clocks_fragment_campaign_stats_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload stats;
  Payload ppb;
  clocks_fragment_add_campaign_ppb(ppb, "gnss", snapshot.gnss);
  clocks_fragment_add_campaign_ppb(ppb, "dwt", snapshot.dwt);
  clocks_fragment_add_campaign_ppb(ppb, "vclock", snapshot.vclock);
  clocks_fragment_add_campaign_ppb(ppb, "ocxo1", snapshot.ocxo1);
  clocks_fragment_add_campaign_ppb(ppb, "ocxo2", snapshot.ocxo2);
  stats.add_object("ppb", ppb);
  parent.add_object("stats", stats);
}

static Payload& clocks_fragment_campaign_payload(
    const clocks_fragment_campaign_snapshot_t& snapshot) {
  clocks_payload_owner_assert(clocks_payload_owner_t::FRAGMENT);
  Payload& campaign = g_clocks_fragment_campaign_payload;
  campaign.clear();
  campaign.add("schema", "TEMPEST_FRAGMENT_V1");
  campaign.add("name", snapshot.campaign);
  campaign.add("state", snapshot.campaign_state);
  campaign.add("public_count", snapshot.public_count);

  Payload clockfaces;
  clockfaces.add("gnss_ns", snapshot.gnss_ns);
  clockfaces.add("dwt_cycles", snapshot.dwt_cycles);
  clockfaces.add("ocxo1_ns", snapshot.ocxo1_ns);
  clockfaces.add("ocxo2_ns", snapshot.ocxo2_ns);
  campaign.add_object("clockfaces", clockfaces);

  Payload status;
  status.add("timeline_valid", snapshot.timeline_valid);
  status.add("ocxo_clockface_valid", snapshot.ocxo_clockface_valid);
  status.add("ocxo_science_valid", snapshot.ocxo_science_valid);
  campaign.add_object("status", status);

  Payload& disposition = g_clocks_fragment_campaign_disposition_payload;
  disposition.clear();
  disposition.add("status", snapshot.disposition);
  disposition.add("use", snapshot.science_eligible && snapshot.control_eligible
      ? "SCIENCE_AND_CONTROL"
      : "AUDIT_ONLY");
  disposition.add("science_eligible", snapshot.science_eligible);
  disposition.add("control_eligible", snapshot.control_eligible);
  if (snapshot.rejection.present) {
    disposition.add("reason_code", snapshot.rejection.reason_code);
    disposition.add("reason_name", snapshot.rejection.reason_name);
    disposition.add("source", snapshot.rejection.source);
    disposition.add("lane_mask", snapshot.rejection.lane_mask);
  }
  campaign.add_object("disposition", disposition);

  if (snapshot.recovery.present) {
    Payload& recovery = g_clocks_fragment_campaign_recovery_payload;
    recovery.clear();
    recovery.add("generation", snapshot.recovery.generation);
    recovery.add("transition_active", snapshot.recovery.transition_active);
    recovery.add("timeline_ready", snapshot.recovery.timeline_ready);
    recovery.add("clockface_ready", snapshot.recovery.clockface_ready);
    recovery.add("science_ready", snapshot.recovery.science_ready);
    recovery.add("degraded_active", snapshot.recovery.degraded_active);
    recovery.add("science_quarantine_active",
                 snapshot.recovery.science_quarantine_active);
    recovery.add("science_quarantine_remaining",
                 snapshot.recovery.science_quarantine_remaining);
    recovery.add("proof_stalled", snapshot.recovery.proof_stalled);
    campaign.add_object("recovery", recovery);
  }

  clocks_fragment_add_campaign_ocxo(
      campaign, "ocxo1", snapshot.ocxo1_clock_candidates, snapshot.ocxo1_science);
  clocks_fragment_add_campaign_ocxo(
      campaign, "ocxo2", snapshot.ocxo2_clock_candidates, snapshot.ocxo2_science);
  clocks_fragment_add_campaign_stats(campaign, snapshot.stats);
  return campaign;
}

static FLASHMEM const char* clocks_fragment_retry_reason_name(uint32_t reason_id) {
  switch (reason_id) {
    case CLOCKS_FRAGMENT_RETRY_REASON_CAMPAIGN_EMBED:
      return "CAMPAIGN_EMBED";
    case CLOCKS_FRAGMENT_RETRY_REASON_CLOCKS_PAYLOAD:
      return "CLOCKS_PAYLOAD_LEGACY";
    case CLOCKS_FRAGMENT_RETRY_REASON_TRANSPORT_ENQUEUE:
      return "TRANSPORT_ENQUEUE";
    case CLOCKS_FRAGMENT_RETRY_REASON_CLOCKS_OBJECT:
      return "CLOCKS_OBJECT";
    case CLOCKS_FRAGMENT_RETRY_REASON_FEATURES_OBJECT:
      return "FEATURES_OBJECT";
    default:
      return "NONE";
  }
}

static bool clocks_fragment_retry_reason_is_observer_plane(uint32_t reason_id) {
  // Payload construction failures are no longer retryable observer-plane events.
  // The only current author of retry state is a completed observation that
  // transport declined to enqueue. Any legacy Payload-build reason found active
  // is therefore contradictory state and must fail loud.
  return reason_id == CLOCKS_FRAGMENT_RETRY_REASON_TRANSPORT_ENQUEUE;
}

static void clocks_fragment_retry_note_failure(uint32_t sequence,
                                               uint32_t reason_id,
                                               bool retrying_snapshot,
                                               bool pi_only) {
  if (!retrying_snapshot) {
    g_clocks_fragment_publication_retry_snapshot_valid = true;
    g_clocks_fragment_publication_retry_sequence = sequence;
    g_clocks_fragment_publication_retry_attempt_count = 1U;
    g_clocks_fragment_publication_retry_reason_id = reason_id;
    g_clocks_fragment_publication_retry_pi_only = pi_only;
  } else {
    g_clocks_fragment_publication_retry_attempt_count++;
    // Once transport rejects a broadcast publication, retain the existing
    // direct-to-Pi retry policy across any later Payload-build failure.
    if (pi_only) g_clocks_fragment_publication_retry_pi_only = true;
  }
}

static void clocks_fragment_retry_note_exhaustion(uint32_t sequence) {
  g_clocks_fragment_publication_last_exhausted_sequence = sequence;
  g_clocks_fragment_publication_last_exhausted_attempt_count =
      g_clocks_fragment_publication_retry_attempt_count;
  g_clocks_fragment_publication_last_exhausted_reason_id =
      g_clocks_fragment_publication_retry_reason_id;
  g_clocks_fragment_publication_last_exhausted_pending_sequence =
      g_clocks_fragment_publication_pending
          ? g_clocks_fragment_publication_pending_sequence
          : 0U;
}

static void clocks_fragment_retry_retire_observation(uint32_t sequence,
                                                     bool count_idle_abandon) {
  if (count_idle_abandon) {
    g_clocks_fragment_publication_idle_retry_abandon_count++;
    g_clocks_fragment_publication_idle_retry_last_sequence = sequence;
    g_clocks_fragment_publication_idle_retry_last_attempt_count =
        g_clocks_fragment_publication_retry_attempt_count;
  }
  g_clocks_fragment_publication_retry_snapshot_valid = false;
  g_clocks_fragment_publication_retry_pi_only = false;
  g_clocks_fragment_publication_retry_sequence = 0U;
  g_clocks_fragment_publication_retry_attempt_count = 0U;
  g_clocks_fragment_publication_retry_reason_id =
      CLOCKS_FRAGMENT_RETRY_REASON_NONE;
  clocks_fragment_publication_item_t* item =
      clocks_fragment_publication_queue_front();
  if (!item || item->sequence != sequence) __builtin_trap();
  clocks_fragment_publication_queue_release();
  if (g_clocks_fragment_publication_pending) {
    clocks_fragment_schedule_publish();
  }
}

// Return true when the current failed publication has crossed a lifecycle
// boundary and the caller must stop retrying it. No path here fabricates delivery:
// ambient testimony is retired; STARTED observer-plane failures retire only the
// undeliverable Pi observation while Beta continues; ambiguous/legacy active state
// surrenders through the watchdog; an in-flight recovery is explicitly aborted.
static bool clocks_fragment_retry_finish_if_exhausted(uint32_t sequence) {
  const uint32_t attempts = g_clocks_fragment_publication_retry_attempt_count;

  if (campaign_state == clocks_campaign_state_t::STOPPED &&
      attempts >= CLOCKS_FRAGMENT_IDLE_RETRY_MAX_ATTEMPTS) {
    clocks_fragment_retry_note_exhaustion(sequence);
    clocks_fragment_retry_retire_observation(sequence, true);
    return true;
  }

  if (attempts < CLOCKS_FRAGMENT_ACTIVE_RETRY_MAX_ATTEMPTS) {
    return false;
  }

  if (campaign_state == clocks_campaign_state_t::RECOVERING) {
    clocks_fragment_retry_note_exhaustion(sequence);
    g_clocks_fragment_publication_recovery_retry_abort_count++;
    recover_lifecycle_abort("recover_publication_retry_exhausted");
    return true;
  }

  if (clocks_watchdog_campaign_armed()) {
    clocks_fragment_retry_note_exhaustion(sequence);
    g_clocks_fragment_publication_active_retry_exhaustion_count++;

    // The retained retry snapshot already owns this exact campaign observation.
    // Once the typed Alpha/Beta row has passed the exact-row court, failures to
    // serialize that immutable testimony into the outbound Payload are observer-
    // plane failures just like transport enqueue rejection. They can lose a Pi
    // observation, but they do not invalidate Beta's timeline/counter continuity.
    // Retire the undeliverable observation and let any reserved successor run.
    // Missing Pi observations remain missing; Beta itself stays STARTED.
    if (clocks_fragment_retry_reason_is_observer_plane(
            g_clocks_fragment_publication_retry_reason_id)) {
      clocks_fragment_retry_retire_observation(sequence, false);
      return true;
    }

    clocks_watchdog_anomaly(
        "clocks_fragment_publication_retry_exhausted",
        sequence,
        attempts,
        g_clocks_fragment_publication_retry_reason_id,
        g_clocks_fragment_publication_last_exhausted_pending_sequence);
    return true;
  }

  // START/STOP/FLASH_CUT command boundaries temporarily disarm the public
  // watchdog.  Those transactions are already finite; retain exact custody
  // until their existing lifecycle court resolves, then this same retry court
  // will either become STOPPED or armed STARTED on a subsequent attempt.
  return false;
}

static void clocks_fragment_publish_service_release_owner(void) {
  __atomic_store_n(&g_clocks_fragment_publication_service_active,
                   0U,
                   __ATOMIC_RELEASE);
  const bool deferred =
      __atomic_exchange_n(&g_clocks_fragment_publication_service_rearm_deferred,
                          0U,
                          __ATOMIC_ACQ_REL) != 0U;
  if (deferred) clocks_fragment_schedule_publish();
}

static void clocks_fragment_publish_service(timepop_ctx_t*,
                                                   timepop_diag_t*,
                                                   void* user_data) {
  const uint32_t callback_generation = (uint32_t)(uintptr_t)user_data;
  if (!g_clocks_fragment_publication_service_armed ||
      callback_generation == 0U ||
      callback_generation != g_clocks_fragment_publication_service_armed_generation) {
    g_clocks_fragment_publication_service_stale_callback_count++;
    return;
  }

  g_clocks_fragment_publication_service_armed = false;
  g_clocks_fragment_publication_service_handle = TIMEPOP_INVALID_HANDLE;

  const bool retrying_snapshot = g_clocks_fragment_publication_retry_snapshot_valid;
  if (!retrying_snapshot && !g_clocks_fragment_publication_pending) return;

  // From snapshot acquisition through queue release this service is the sole
  // owner of CLOCKS publication storage. Scheduling requests raised while it is
  // active become one deferred wake; they cannot arm a nested serializer.
  uint32_t expected_service_owner = 0U;
  if (!__atomic_compare_exchange_n(
          &g_clocks_fragment_publication_service_active,
          &expected_service_owner,
          1U,
          false,
          __ATOMIC_ACQ_REL,
          __ATOMIC_ACQUIRE)) {
    __builtin_trap();
  }
  __atomic_store_n(&g_clocks_fragment_publication_service_rearm_deferred,
                   0U,
                   __ATOMIC_RELEASE);
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::FRAGMENT);

  clocks_fragment_publication_ensure_initialized();

  // Beta can establish observer loss only after later completed rows fail to
  // enter the bounded campaign handoff.  Once that positive watermark covers
  // an older retained transport retry, continuing to retry that immutable Pi
  // observation is a head-of-line liveness bug: the producer has advanced and
  // the missing observer interval cannot be replayed.  The publication consumer
  // therefore retires its own stale retry and every committed campaign record
  // covered by the same watermark.  Beta never advances either consumer index.
  if (retrying_snapshot &&
      clocks_fragment_retry_reason_is_observer_plane(
          g_clocks_fragment_publication_retry_reason_id) &&
      clocks_fragment_campaign_sequence_observer_retired(
          g_clocks_fragment_publication_retry_sequence)) {
    const uint32_t retired_retry_sequence =
        g_clocks_fragment_publication_retry_sequence;
    clocks_fragment_campaign_retire_observer_queue_through(
        g_clocks_fragment_campaign_observer_retired_through_sequence);
    g_clocks_fragment_publication_observer_retire_count++;
    g_clocks_fragment_publication_observer_retire_last_sequence =
        retired_retry_sequence;
    clocks_fragment_retry_retire_observation(
        retired_retry_sequence, false);
    clocks_fragment_publish_service_release_owner();
    return;
  }

  uint32_t sequence = 0U;
  bool clocks_snapshot_ok = false;
  clocks_fragment_publication_item_t* publication_item = nullptr;
  if (retrying_snapshot) {
    sequence = g_clocks_fragment_publication_retry_sequence;
    publication_item = clocks_fragment_publication_queue_front();
    if (!publication_item || publication_item->sequence != sequence) {
      __builtin_trap();
    }
    clocks_snapshot_ok = true;
  } else {
    if (clocks_fragment_publication_queue_depth() != 0U) __builtin_trap();
    sequence = g_clocks_fragment_publication_pending_sequence;
    g_clocks_fragment_publication_pending = false;

    // A stable STARTED campaign may outlive its Pi observer.  Once Beta has
    // positively recorded handoff saturation, exact physical observations at or
    // before that watermark are no longer deliverable: Alpha's latest snapshot
    // cannot replay them, and the bounded campaign queue must not be grown to
    // simulate an arbitrarily long host outage.  The publisher is the campaign
    // queue consumer, so it explicitly retires only those committed records and
    // drops this Pi observation.  The next fresh Beta row remains eligible for
    // ordinary exact publication with campaign testimony.
    if (clocks_fragment_campaign_sequence_observer_retired(sequence)) {
      clocks_fragment_campaign_retire_observer_queue_through(
          g_clocks_fragment_campaign_observer_retired_through_sequence);
      g_clocks_fragment_publication_observer_retire_count++;
      g_clocks_fragment_publication_observer_retire_last_sequence = sequence;
      clocks_fragment_publish_service_release_owner();
      return;
    }

    publication_item = clocks_fragment_publication_queue_acquire_write();
    memset(publication_item, 0, sizeof(*publication_item));
    publication_item->sequence = sequence;
    clocks_snapshot_ok = clocks_fragment_snapshot_take(
        sequence, &publication_item->clocks);
  }

  clocks_fragment_snapshot_t& clocks_snapshot = publication_item->clocks;
  const bool instrument_row_exact = clocks_snapshot_ok &&
      clocks_snapshot.live.snapshot_ok &&
      clocks_snapshot.live.completed_row_coherent &&
      clocks_snapshot.live.completed_pps_sequence == sequence;

  if (!instrument_row_exact) {
    // The interrupt-side PPS notification arrives before both post-PPS OCXO edges
    // have necessarily completed Alpha's immutable row. Never label the latest
    // completed instrument state with the newer trigger identity. Hold this exact
    // requested sequence until Beta's completed-row notification rearms service.
    if (retrying_snapshot) {
      g_clocks_fragment_publication_publish_reject_count++;
      g_clocks_fragment_publication_retry_snapshot_valid = false;
      g_clocks_fragment_publication_retry_pi_only = false;
      g_clocks_fragment_publication_retry_sequence = 0U;
      g_clocks_fragment_publication_retry_attempt_count = 0U;
    }
    g_clocks_fragment_publication_pending_sequence = sequence;
    g_clocks_fragment_publication_pending = true;
    memset(publication_item, 0, sizeof(*publication_item));
    clocks_fragment_publish_service_release_owner();
    return;
  }

  const bool campaign_available = clocks_snapshot_ok &&
      clocks_snapshot.campaign.present &&
      clocks_snapshot.campaign.completed_second_sequence == sequence;

  if (clocks_snapshot_ok &&
      clocks_snapshot.campaign_row_expected &&
      !campaign_available) {
    // Public campaign time is already advancing, so this exact physical second
    // must wait for Beta's matching campaign delta.  This custody bit is never
    // serialized and does not contaminate the canonical instrument model.
    if (retrying_snapshot) {
      g_clocks_fragment_publication_publish_reject_count++;
      g_clocks_fragment_publication_retry_snapshot_valid = false;
      g_clocks_fragment_publication_retry_pi_only = false;
      g_clocks_fragment_publication_retry_sequence = 0U;
      g_clocks_fragment_publication_retry_attempt_count = 0U;
    }
    g_clocks_fragment_publication_pending_sequence = sequence;
    g_clocks_fragment_publication_pending = true;
    memset(publication_item, 0, sizeof(*publication_item));
    clocks_fragment_publish_service_release_owner();
    return;
  }

  if (!retrying_snapshot) {
    clocks_fragment_features_snapshot_take(publication_item->features);
    clocks_fragment_publication_queue_commit_write();
    publication_item = clocks_fragment_publication_queue_front();
    if (!publication_item || publication_item->sequence != sequence) {
      __builtin_trap();
    }
  }

  // From this point through publish(), every runtime Payload input belongs to
  // this consumer-owned immutable queue entry.
  const clocks_fragment_snapshot_t& owned_clocks = publication_item->clocks;
  const clocks_fragment_features_snapshot_t& owned_features =
      publication_item->features;

  Payload& fragment = g_clocks_fragment_root_payload;
  fragment.clear();
  fragment.add("schema", "CLOCKS_FRAGMENT_V4");
  fragment.add("sequence",
               owned_clocks.live.completed_pps_sequence);

  // Campaign is optional enrichment. If Beta has not frozen it yet, publish the
  // instrument row now; campaign_row_ready() will schedule a same-sequence
  // strengthening copy. This avoids carrying campaign lifecycle mirrors inside
  // the always-on instrument snapshot merely to coordinate serialization.
  const bool campaign_embedded = campaign_available;
  if (campaign_available) {
    Payload& campaign = clocks_fragment_campaign_payload(
        owned_clocks.campaign);
    fragment.add_object("campaign", campaign);
    campaign.clear();
  }

  // instrument_row_exact above has already proved the typed Alpha snapshot and
  // its completed-row identity. Payload construction is now a hard invariant:
  // these attachments either return complete or Payload escalates through its
  // retained fatal WATCHDOG court and does not return to CLOCKS.
  Payload& clocks = clocks_fragment_clocks_payload(
      owned_clocks.live);
  fragment.add_object("clocks", clocks);
  clocks.clear();

  fragment.add_object("features",
                      clocks_fragment_features_payload(owned_features));

  const uint32_t next_publish_count = g_clocks_fragment_publication_publish_count + 1U;
  const uint32_t next_campaign_rows_embedded =
      g_clocks_fragment_publication_campaign_rows_embedded +
      (campaign_embedded ? 1U : 0U);

  const bool publication_enqueued =
      retrying_snapshot && g_clocks_fragment_publication_retry_pi_only
          ? publish_to_pi("CLOCKS_FRAGMENT", fragment)
          : publish("CLOCKS_FRAGMENT", fragment);
  fragment.clear();

  if (!publication_enqueued) {
    g_clocks_fragment_publication_publish_reject_count++;
    clocks_fragment_retry_note_failure(
        sequence,
        CLOCKS_FRAGMENT_RETRY_REASON_TRANSPORT_ENQUEUE,
        retrying_snapshot,
        true);
    if (!retrying_snapshot && campaign_embedded) {
      g_clocks_fragment_publication_campaign_retry_count++;
    }
    if (clocks_fragment_retry_finish_if_exhausted(sequence)) {
      clocks_fragment_publish_service_release_owner();
      return;
    }

    g_clocks_fragment_publication_retry_schedule_count++;
    clocks_fragment_schedule_publish();
    clocks_fragment_publish_service_release_owner();
    return;
  }

  g_clocks_fragment_publication_publish_count = next_publish_count;
  g_clocks_fragment_publication_campaign_rows_embedded = next_campaign_rows_embedded;
  if (retrying_snapshot) {
    g_clocks_fragment_publication_retry_success_count++;
  }
  g_clocks_fragment_publication_retry_snapshot_valid = false;
  g_clocks_fragment_publication_retry_pi_only = false;
  g_clocks_fragment_publication_retry_sequence = 0U;
  g_clocks_fragment_publication_retry_attempt_count = 0U;
  g_clocks_fragment_publication_retry_reason_id =
      CLOCKS_FRAGMENT_RETRY_REASON_NONE;

  clocks_fragment_publication_item_t* completed_item =
      clocks_fragment_publication_queue_front();
  if (!completed_item || completed_item->sequence != sequence) __builtin_trap();
  clocks_fragment_publication_queue_release();
  if (g_clocks_fragment_publication_pending) clocks_fragment_schedule_publish();
  clocks_fragment_publish_service_release_owner();
}

static void clocks_fragment_schedule_publish(void) {
  if (!g_clocks_fragment_publication_pending &&
      !g_clocks_fragment_publication_retry_snapshot_valid) {
    return;
  }
  if (__atomic_load_n(&g_clocks_fragment_publication_service_active,
                      __ATOMIC_ACQUIRE) != 0U) {
    __atomic_store_n(&g_clocks_fragment_publication_service_rearm_deferred,
                     1U,
                     __ATOMIC_RELEASE);
    return;
  }
  if (g_clocks_fragment_publication_service_armed) {
    return;
  }

  uint32_t generation = g_clocks_fragment_publication_service_generation + 1U;
  if (generation == 0U) generation = 1U;

  const bool delayed_retry =
      g_clocks_fragment_publication_retry_snapshot_valid &&
      g_clocks_fragment_publication_retry_attempt_count > 0U;
  const timepop_handle_t handle = delayed_retry
      ? timepop_arm(CLOCKS_FRAGMENT_RETRY_DELAY_NS,
                    false,
                    clocks_fragment_publish_service,
                    (void*)(uintptr_t)generation,
                    "CLOCKS_FRAGMENT_RETRY")
      : timepop_arm_alap(clocks_fragment_publish_service,
                         (void*)(uintptr_t)generation,
                         "CLOCKS_FRAGMENT_PUBLISH");
  if (handle == TIMEPOP_INVALID_HANDLE) {
    g_clocks_fragment_publication_service_arm_failures++;
    return;
  }

  g_clocks_fragment_publication_service_handle = handle;
  g_clocks_fragment_publication_service_generation = generation;
  g_clocks_fragment_publication_service_armed_generation = generation;
  g_clocks_fragment_publication_service_armed = true;
}

static void clocks_fragment_force_rearm(void) {
  if (!g_clocks_fragment_publication_service_armed) {
    clocks_fragment_schedule_publish();
    return;
  }

  bool cancelled = false;
  if (g_clocks_fragment_publication_service_handle != TIMEPOP_INVALID_HANDLE) {
    cancelled = timepop_cancel(g_clocks_fragment_publication_service_handle);
  }
  if (cancelled) {
    g_clocks_fragment_publication_service_cancel_count++;
  } else {
    g_clocks_fragment_publication_service_cancel_failure_count++;
  }

  // Generation-tagged callbacks make this safe even if the old handle was
  // already selected for dispatch and could not be cancelled. The replacement
  // arm advances the generation; any superseded callback becomes a no-op.
  g_clocks_fragment_publication_service_handle = TIMEPOP_INVALID_HANDLE;
  g_clocks_fragment_publication_service_armed = false;
  g_clocks_fragment_publication_service_forced_rearm_count++;
  clocks_fragment_schedule_publish();
}

static void clocks_fragment_accept_tick(uint32_t requested_sequence) {
  if (g_clocks_fragment_publication_last_tick_valid &&
      requested_sequence == g_clocks_fragment_publication_last_tick_sequence) {
    return;
  }
  g_clocks_fragment_publication_last_tick_sequence = requested_sequence;
  g_clocks_fragment_publication_last_tick_valid = true;

  // A retained campaign retry already owns the SPSC queue front.
  // While that exact row remains in transport custody, a newer physical tick
  // must not replace the pending sequence selected by Beta's next immutable
  // campaign row. Doing so creates a permanent head-of-line mismatch: Beta
  // keeps the older row while the CLOCKS publisher repeatedly asks for the newest tick.
  //
  // The physical tick still drives retry progress. Once the retained row is
  // accepted, any campaign_row_ready() received during the retry remains in
  // g_clocks_fragment_publication_pending_sequence and is published next.
  if (g_clocks_fragment_publication_retry_snapshot_valid) {
    g_clocks_fragment_publication_retry_tick_hold_count++;
    if (!g_clocks_fragment_publication_pending) {
      g_clocks_fragment_publication_pending_sequence = requested_sequence;
      g_clocks_fragment_publication_pending = true;
    } else if (g_clocks_fragment_publication_pending_sequence != requested_sequence) {
      // One exact snapshot is in retry custody and one following identity is
      // reserved. A further arrival is visible as coalescing rather than
      // silently replacing the reserved successor. Under normal USB budget
      // pressure the 25 ms retry must clear long before this boundary.
      g_clocks_fragment_publication_coalesce_count++;
    }
    clocks_fragment_schedule_publish();
    return;
  }

  if (g_clocks_fragment_publication_pending) g_clocks_fragment_publication_coalesce_count++;
  g_clocks_fragment_publication_pending_sequence = requested_sequence;
  g_clocks_fragment_publication_pending = true;
  clocks_fragment_schedule_publish();
}

void clocks_fragment_pps_tick_from_interrupt(
    uint32_t pps_sequence) {
  if (clocks_fragment_current_ipsr() != 0U) return;
  clocks_fragment_accept_tick(pps_sequence);
}

void clocks_fragment_completed_row_ready(uint32_t completed_second_sequence) {
  if (clocks_fragment_current_ipsr() != 0U || completed_second_sequence == 0U) return;

  // Beta calls this only after Alpha has frozen the exact completed row. If the
  // interrupt-side notification already reserved this identity, wake/rearm the
  // held serializer now that the row is actually available.
  if (g_clocks_fragment_publication_pending &&
      g_clocks_fragment_publication_pending_sequence == completed_second_sequence) {
    if (g_clocks_fragment_publication_service_armed) {
      clocks_fragment_force_rearm();
    } else {
      clocks_fragment_schedule_publish();
    }
    return;
  }

  clocks_fragment_accept_tick(completed_second_sequence);
}

void clocks_fragment_campaign_row_ready(uint32_t completed_second_sequence) {
  if (clocks_fragment_current_ipsr() != 0U || completed_second_sequence == 0U) {
    return;
  }

  g_clocks_fragment_publication_campaign_ready_signal_count++;
  const bool repeated_signal =
      g_clocks_fragment_publication_campaign_ready_last_valid &&
      g_clocks_fragment_publication_campaign_ready_last_sequence ==
          completed_second_sequence;
  const bool service_was_armed = g_clocks_fragment_publication_service_armed;
  if (repeated_signal) {
    g_clocks_fragment_publication_campaign_ready_repeat_count++;
  }
  g_clocks_fragment_publication_campaign_ready_last_valid = true;
  g_clocks_fragment_publication_campaign_ready_last_sequence = completed_second_sequence;

  // If the canonical interrupt tick is still pending, keep one publication and
  // let the completed campaign observation ride along. If that sequence already
  // published, arm a deliberate same-sequence enrichment copy rather than losing
  // durable truth.
  if (g_clocks_fragment_publication_pending &&
      g_clocks_fragment_publication_pending_sequence == completed_second_sequence) {
    if (repeated_signal && service_was_armed) {
      clocks_fragment_force_rearm();
    } else {
      clocks_fragment_schedule_publish();
    }
    return;
  }

  if (g_clocks_fragment_publication_last_tick_valid &&
      g_clocks_fragment_publication_last_tick_sequence == completed_second_sequence) {
    g_clocks_fragment_publication_campaign_ready_enrichment_retry_count++;
    g_clocks_fragment_publication_pending_sequence = completed_second_sequence;
    g_clocks_fragment_publication_pending = true;
    if (repeated_signal && service_was_armed) {
      clocks_fragment_force_rearm();
    } else {
      clocks_fragment_schedule_publish();
    }
    return;
  }

  clocks_fragment_accept_tick(completed_second_sequence);
  if (repeated_signal && service_was_armed) {
    clocks_fragment_force_rearm();
  }
}



// CLOCKS publication is normally armed by the first readiness notification.
// Keep one bounded second-chance notification inside the same public second so
// an orphaned/stale ALAP arm cannot strand Beta's immutable row until the next
// candidate reaches the structural backlog court.
static constexpr uint64_t CAMPAIGN_RECORD_READY_RETRY_NS = 100000000ULL;
static constexpr const char* CAMPAIGN_RECORD_READY_RETRY_TIMER =
    "clocks-monitor-campaign-ready-retry";
static timepop_handle_t g_clocks_fragment_campaign_record_ready_retry_handle =
    TIMEPOP_INVALID_HANDLE;
static uint32_t g_clocks_fragment_campaign_record_ready_retry_arm_count = 0U;
static uint32_t g_clocks_fragment_campaign_record_ready_retry_arm_failure_count = 0U;
static uint32_t g_clocks_fragment_campaign_record_ready_retry_fire_count = 0U;
static uint32_t g_clocks_fragment_campaign_record_ready_retry_cancel_count = 0U;

static void clocks_fragment_campaign_record_ready_retry_cancel(void) {
  if (g_clocks_fragment_campaign_record_ready_retry_handle == TIMEPOP_INVALID_HANDLE) {
    return;
  }
  (void)timepop_cancel(g_clocks_fragment_campaign_record_ready_retry_handle);
  g_clocks_fragment_campaign_record_ready_retry_handle = TIMEPOP_INVALID_HANDLE;
  g_clocks_fragment_campaign_record_ready_retry_cancel_count++;
}

static void clocks_fragment_campaign_record_ready_retry_callback(
    timepop_ctx_t*, timepop_diag_t*, void*) {
  g_clocks_fragment_campaign_record_ready_retry_handle = TIMEPOP_INVALID_HANDLE;
  const uint32_t pending_sequence =
      clocks_fragment_campaign_queue_front_sequence();
  if (pending_sequence == 0U) return;

  g_clocks_fragment_campaign_record_ready_retry_fire_count++;
  clocks_fragment_campaign_row_ready(pending_sequence);
}

static void clocks_fragment_campaign_record_ready_retry_arm(void) {
  clocks_fragment_campaign_record_ready_retry_cancel();
  g_clocks_fragment_campaign_record_ready_retry_handle =
      timepop_arm(CAMPAIGN_RECORD_READY_RETRY_NS,
                  false,
                  clocks_fragment_campaign_record_ready_retry_callback,
                  nullptr,
                  CAMPAIGN_RECORD_READY_RETRY_TIMER);
  if (g_clocks_fragment_campaign_record_ready_retry_handle ==
      TIMEPOP_INVALID_HANDLE) {
    g_clocks_fragment_campaign_record_ready_retry_arm_failure_count++;
    return;
  }
  g_clocks_fragment_campaign_record_ready_retry_arm_count++;
}

// RECOVER is a new CLOCKS_FRAGMENT transport-custody epoch. A Pi/PUBSUB
// outage can strand one exact retry snapshot plus one Beta campaign handoff;
// once continuity has already been surrendered those pre-recovery obligations
// must not hold the newly proved recovery boundary hostage. Retire them
// explicitly while preserving Alpha's physical/statistical state.
static void clocks_fragment_recover_reset_publication_custody(bool count_recovery = true) {
  clocks_fragment_publication_ensure_initialized();

  g_clocks_fragment_publication_recovery_retired_retry_sequence =
      g_clocks_fragment_publication_retry_snapshot_valid
          ? g_clocks_fragment_publication_retry_sequence
          : 0U;
  g_clocks_fragment_publication_recovery_retired_pending_sequence =
      g_clocks_fragment_publication_pending
          ? g_clocks_fragment_publication_pending_sequence
          : 0U;
  g_clocks_fragment_publication_recovery_retired_campaign_sequence =
      clocks_fragment_campaign_queue_front_sequence();

  if (g_clocks_fragment_publication_service_handle != TIMEPOP_INVALID_HANDLE) {
    const bool cancelled =
        timepop_cancel(g_clocks_fragment_publication_service_handle);
    if (cancelled) {
      g_clocks_fragment_publication_service_cancel_count++;
    } else {
      g_clocks_fragment_publication_service_cancel_failure_count++;
    }
  }
  g_clocks_fragment_publication_service_handle = TIMEPOP_INVALID_HANDLE;
  g_clocks_fragment_publication_service_armed = false;
  g_clocks_fragment_publication_service_armed_generation = 0U;

  g_clocks_fragment_publication_pending_sequence = 0U;
  g_clocks_fragment_publication_pending = false;
  g_clocks_fragment_publication_retry_snapshot_valid = false;
  g_clocks_fragment_publication_retry_pi_only = false;
  g_clocks_fragment_publication_retry_sequence = 0U;
  g_clocks_fragment_publication_retry_attempt_count = 0U;
  g_clocks_fragment_publication_retry_reason_id =
      CLOCKS_FRAGMENT_RETRY_REASON_NONE;
  g_clocks_fragment_publication_last_tick_valid = false;
  g_clocks_fragment_publication_last_tick_sequence = 0U;
  g_clocks_fragment_publication_campaign_ready_last_valid = false;
  g_clocks_fragment_publication_campaign_ready_last_sequence = 0U;
  clocks_fragment_publication_queue_reset();

  clocks_fragment_campaign_record_ready_retry_cancel();
  clocks_fragment_campaign_queue_reset();
  clocks_fragment_campaign_observer_loss_reset_epoch();
  if (count_recovery) {
    g_clocks_fragment_publication_recovery_reset_count++;
  }
}

// A watchdog surrender must cut stale CLOCKS_FRAGMENT transport custody at
// the surrender boundary itself.  Waiting for Pi RECOVER is too late: a Pi
// reboot can leave the retained 25 ms retry loop hot enough that REPORT/RECOVER
// commands never get a turn.  Preserve separate testimony because the later
// RECOVER reset is intentionally idempotent and may find nothing left to retire.
static void clocks_fragment_watchdog_reset_publication_custody(void) {
  g_clocks_fragment_publication_watchdog_retired_retry_sequence =
      g_clocks_fragment_publication_retry_snapshot_valid
          ? g_clocks_fragment_publication_retry_sequence
          : 0U;
  g_clocks_fragment_publication_watchdog_retired_pending_sequence =
      g_clocks_fragment_publication_pending
          ? g_clocks_fragment_publication_pending_sequence
          : 0U;
  g_clocks_fragment_publication_watchdog_retired_campaign_sequence =
      clocks_fragment_campaign_queue_front_sequence();
  clocks_fragment_recover_reset_publication_custody(false);
  g_clocks_fragment_publication_watchdog_reset_count++;
}

// Command-report construction is a serialized foreground service.  The outer
// COMMAND Payload custody keeps Priority-0 capture live while excluding the
// Priority-16 TimePop/handoff tier for the complete command transaction.  This
// report-local owner then proves that the reusable report scratch itself has one
// writer.  The typed CLOCKS handoff uses separate immutable queue state.
static volatile uint32_t g_clocks_report_build_active = 0U;
static uint32_t g_clocks_report_build_count = 0U;
static uint32_t g_clocks_report_busy_reject_count = 0U;
static uint32_t g_clocks_report_max_duration_cycles = 0U;

static clocks_instrument_stats_snapshot_t
    g_beta_report_instrument_stats DMAMEM = {};
static Payload g_report_clocks_payload;
static Payload g_report_stats_payload;
static Payload g_report_smartzero_payload;
static Payload g_report_child_clocks;
static Payload g_report_child_stats;
static Payload g_report_child_clock;
static Payload g_report_child_welford;
static Payload g_report_child_maturity;
static Payload g_report_child_admission;

// Dedicated CLOCKS snapshot scratch. The 1 Hz side rail never borrows command
// report Payload headers and never constructs transport objects inside CLOCKS.
static clocks_instrument_stats_snapshot_t
    g_beta_clocks_fragment_instrument_stats DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_clocks_fragment_pps_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_clocks_fragment_vclock_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_clocks_fragment_ocxo1_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t
    g_beta_clocks_fragment_ocxo2_prediction DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_clocks_fragment_vclock_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_clocks_fragment_ocxo1_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t
    g_beta_clocks_fragment_ocxo2_forensics DMAMEM = {};


static FLASHMEM void clocks_fragment_stats_snapshot_from_instrument(
    clocks_fragment_stats_snapshot_t& out,
    const clocks_instrument_stats_snapshot_t& instrument);

// ============================================================================
// Durable structured recovery state
// ============================================================================
//
// CLOCKS recovery crosses the control boundary as ordinary typed state.
// Floating values cross the Payload boundary as fixed-decimal JSON numbers;
// firmware never constructs, stores, or accepts opaque binary recovery state.

static constexpr uint32_t CLOCKS_STRUCTURED_RESTORE_VERSION = 4U;
static constexpr uint32_t CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS = 4U;

struct clocks_recovery_restore_state_t {
  bool valid = false;

  uint64_t instrument_gnss_ns = 0ULL;
  uint64_t instrument_dwt_cycles = 0ULL;
  uint64_t instrument_ocxo1_ns = 0ULL;
  uint64_t instrument_ocxo2_ns = 0ULL;

  clocks_instrument_stats_snapshot_t stats{};
};

static clocks_recovery_restore_state_t
    g_clocks_restore_state DMAMEM = {};
static clocks_recovery_restore_state_t
    g_campaign_restore_state DMAMEM = {};
static volatile bool g_clocks_restore_requested = false;
static uint32_t g_clocks_restore_request_count = 0U;
static uint32_t g_clocks_restore_commit_count = 0U;
static uint32_t g_clocks_restore_failure_count = 0U;
static uint32_t g_campaign_restore_count = 0U;
static uint32_t g_campaign_restore_failure_count = 0U;
static uint32_t g_campaign_restore_ignored_live_count = 0U;

// Better-Buckets restore is a recovery-only staging transaction. Pi CLOCKS
// supplies Alpha-authored endpoints from its durable synthetic checkpoint in
// small idempotent chunks before structured restore may consume them.
static bool g_ppb_restore_protocol_active = false;
static bool g_ppb_restore_protocol_committed = false;
static uint32_t g_ppb_restore_protocol_sequence = 0U;
static uint32_t g_ppb_restore_second_expected = 0U;
static uint32_t g_ppb_restore_second_accepted = 0U;
static uint32_t g_ppb_restore_minute_expected = 0U;
static uint32_t g_ppb_restore_minute_accepted = 0U;

static void clocks_ppb_restore_protocol_clear(bool abort_alpha) {
  if (abort_alpha && g_ppb_restore_protocol_active) {
    clocks_alpha_ppb_restore_abort();
  }
  g_ppb_restore_protocol_active = false;
  g_ppb_restore_protocol_committed = false;
  g_ppb_restore_protocol_sequence = 0U;
  g_ppb_restore_second_expected = 0U;
  g_ppb_restore_second_accepted = 0U;
  g_ppb_restore_minute_expected = 0U;
  g_ppb_restore_minute_accepted = 0U;
}

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

static bool clocks_recovery_restore_statistics_valid(
    const clocks_recovery_restore_state_t& state) {
  return state.valid &&
         state.instrument_gnss_ns != 0ULL &&
         state.instrument_dwt_cycles != 0ULL &&
         state.instrument_ocxo1_ns != 0ULL &&
         state.instrument_ocxo2_ns != 0ULL;
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

static bool restore_get_i64(const Payload& args, const char* key,
                            int64_t& out, bool required = true) {
  if (!args.has(key)) return !required;
  const char* token = args.getString(key);
  if (!token || !*token) return false;
  errno = 0;
  char* end = nullptr;
  const long long parsed = strtoll(token, &end, 10);
  if (errno == ERANGE || !end || *end != '\0') return false;
  out = (int64_t)parsed;
  return true;
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

static bool clocks_recovery_state_from_args(
    const Payload& args,
    clocks_recovery_restore_state_t& out) {
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
      !restore_parse_tau(args, "restore_ocxo1_tau",
                         time_clock_id_t::OCXO1,
                         out.stats.ocxo1_tau_state) ||
      !restore_parse_tau(args, "restore_ocxo2_tau",
                         time_clock_id_t::OCXO2,
                         out.stats.ocxo2_tau_state)) {
    return false;
  }

  out.stats.last_pps_sequence = 0U;
  out.stats.rolling_ppb_current_sequence = 0U;
  out.stats.rolling_ppb_endpoint_admitted = false;
  out.stats.rolling_ppb_interval_advanced = false;
  out.stats.gnss_ns = out.instrument_gnss_ns;
  out.stats.dwt_cycles = out.instrument_dwt_cycles;
  out.stats.ocxo1_ns = out.instrument_ocxo1_ns;
  out.stats.ocxo2_ns = out.instrument_ocxo2_ns;
  out.valid = true;
  return clocks_recovery_restore_statistics_valid(out);
}


static FLASHMEM Payload clocks_report_busy_response(const char* report);

struct clocks_report_build_guard_t {
  uint32_t begin_dwt = 0U;
  bool acquired = false;

  clocks_report_build_guard_t() {
    clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
    uint32_t expected = 0U;
    if (!__atomic_compare_exchange_n(&g_clocks_report_build_active,
                                     &expected,
                                     1U,
                                     false,
                                     __ATOMIC_ACQ_REL,
                                     __ATOMIC_ACQUIRE)) {
      g_clocks_report_busy_reject_count++;
      __builtin_trap();
    }
    g_clocks_report_build_count++;
    begin_dwt = DWT_CYCCNT;
    acquired = true;
  }

  ~clocks_report_build_guard_t() {
    if (!acquired) return;
    const uint32_t elapsed = (uint32_t)(DWT_CYCCNT - begin_dwt);
    if (elapsed > g_clocks_report_max_duration_cycles) {
      g_clocks_report_max_duration_cycles = elapsed;
    }
    __atomic_store_n(&g_clocks_report_build_active, 0U, __ATOMIC_RELEASE);
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
// Beta diagnostics initialization
// ============================================================================
//
// Campaign publication and science-residual readiness are lifecycle facts, not
// ambient instrument prerequisites. They are intentionally absent from the
// canonical CLOCKS feature tree; focused CLOCKS reports retain detailed state.

static FLASHMEM void clocks_beta_cold_diagnostics_init(void);
static bool g_clocks_beta_dmamem_initialized = false;

FLASHMEM void clocks_beta_features_init(void) {
  clocks_beta_cold_diagnostics_init();
  // Publication custody is CLOCKS state too.  Initialize it synchronously with
  // the rest of Beta so the first completed Alpha row never depends on a lazy
  // callback-side initialization transaction.
  clocks_fragment_publication_ensure_initialized();
  clocks_fragment_campaign_queue_reset();
  g_report_clocks_payload.clear();
  g_report_stats_payload.clear();
  g_report_smartzero_payload.clear();
  g_report_child_clocks.clear();
  g_report_child_stats.clear();
  g_report_child_clock.clear();
  g_report_child_welford.clear();
  g_report_child_maturity.clear();
  g_report_child_admission.clear();
}

// ============================================================================
// Campaign admission authority
// ============================================================================
//
// Global campaign-readiness policy is evaluated once by Pi CLOCKS from a fresh
// unified CLOCKS.features heartbeat before it sends CLOCKS.START.  Teensy
// CLOCKS must not subscribe to CLOCKS (which would echo its own state back
// through the Pi) or to a feature-only side feed.  Receipt of CLOCKS.START is
// therefore the global-policy authorization boundary.
//
// Firmware still owns every local imperative and physical transition court:
// command/state validation, numeric integrity, SmartZero,
// private PPS0 acquisition, CounterLedger/PhaseLedger maturity, watchdog
// custody, and the release of public PPS1.  Those courts remain authoritative
// and cannot be bypassed by CLOCKS.
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
    case CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE: return "FLASH_CUT_GATE";
    case CAMPAIGN_RECORD_STAGE_RECOVER_PROOF_GATE: return "RECOVER_PROOF_GATE";
    case CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE: return "RECOVERING_NO_REQUEST_GATE";
    case CAMPAIGN_RECORD_STAGE_REARM_GATE: return "REARM_GATE";
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
// RECOVER is the dead-producer restore transaction. Surviving producers never
// receive RECOVER: the Pi proves, reacquires, and adopts them read-only. RECOVER
// therefore always creates a fresh Alpha custody boundary and resumes the durable
// campaign at the projected public identity.
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
// row begin at zero.  DEAD_PRODUCER_RESTORE and surviving-Alpha REARM use a
// positive/negative offset to make the current Alpha service ledger appear as
// the Pi-projected campaign ledger at the resume boundary.  This is deliberately
// signed; the old unsigned base model
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
// The interval totals below are still accumulated so residual/Welford
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

// Defined below the candidate implementation.
static bool delta_raw_interval_cycles_plausible(uint32_t cycles);
static int64_t beta_round_double_to_i64(double value);

// Campaign-scoped Delta-Cycles shadow clocks.  The canonical public OCXO
// clockface remains CounterLedger + PhaseLedger.  The independent Delta clock
// closes on the same selected VCLOCK endpoints, but it derives elapsed OCXO
// nanoseconds directly from the VCLOCK and OCXO DWT-at-edge transcripts.
//
// A selected VCLOCK second generally straddles two OCXO one-second intervals:
// the tail of the prior interval and the head of the current interval.  Applying
// only the current interval's rate to the whole VCLOCK second creates a boundary
// term whenever adjacent OCXO intervals differ and makes D-P drift until the
// next campaign/recovery seed.  Preserve the prior bracket so each increment is
// the exact two-piece, window-aligned Delta construction.
struct delta_clock_candidate_state_t {
  bool seeded = false;
  bool continuity_valid = false;
  clocks_fragment_clock_candidate_status_t status =
      clocks_fragment_clock_candidate_status_t::UNAVAILABLE;
  uint32_t start_public_count = 0;
  uint32_t last_public_count = 0;
  uint32_t interval_count = 0;
  uint64_t ns = 0;
  double fractional_ns = 0.0;

  bool geometry_valid = false;
  uint32_t geometry_target_dwt = 0;
  uint32_t geometry_next_ocxo_dwt = 0;
  uint32_t geometry_interval_cycles = 0;

  bool last_residual_available = false;
  int64_t last_residual_ns = 0;
  double last_residual_ns_exact = 0.0;
};

static delta_clock_candidate_state_t
    g_delta_clock_candidate_ocxo1 DMAMEM = {};
static delta_clock_candidate_state_t
    g_delta_clock_candidate_ocxo2 DMAMEM = {};

static void delta_clock_candidate_seed(delta_clock_candidate_state_t& state,
                                       uint64_t ns,
                                       uint32_t public_count) {
  state = delta_clock_candidate_state_t{};
  state.seeded = true;
  state.continuity_valid = true;
  state.status = clocks_fragment_clock_candidate_status_t::SEEDED;
  state.start_public_count = public_count;
  state.last_public_count = public_count;
  state.ns = ns;
}

static void delta_clock_candidates_seed_zero(void) {
  delta_clock_candidate_seed(g_delta_clock_candidate_ocxo1, 0ULL, 0U);
  delta_clock_candidate_seed(g_delta_clock_candidate_ocxo2, 0ULL, 0U);
}

static void delta_clock_candidates_seed_projected_resume(void) {
  const uint32_t recovered_public_count =
      (uint32_t)(recover_gnss_ns / CLOCKS_BETA_NS_PER_SECOND);
  delta_clock_candidate_seed(g_delta_clock_candidate_ocxo1,
                             recover_ocxo1_ns,
                             recovered_public_count);
  delta_clock_candidate_seed(g_delta_clock_candidate_ocxo2,
                             recover_ocxo2_ns,
                             recovered_public_count);
}

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
// Beta extends the canonical fragment science value with private calculation
// and comparison state.  Common published fields have one typed home.
struct clock_science_row_t : clocks_fragment_science_snapshot_t {
  // valid/science_worthy/antecedents_complete are inherited canonical fields.
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
  bool     delta_raw_reference_plausible = false;
  bool     delta_raw_clock_plausible = false;
  uint32_t delta_reference_public_count = 0;
  uint32_t delta_publication_public_count = 0;
  int64_t  delta_raw_residual_cycles = 0;
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
  double   gnss_interval_ns_exact = 0.0;
  double   clock_interval_ns_exact = 0.0;
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
  // restart the OCXO totals from a fresh fresh-ancestry intercept.
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

static bool delta_clock_candidate_geometry_valid(
    uint32_t target_dwt,
    uint32_t next_ocxo_dwt,
    uint32_t interval_cycles) {
  if (!delta_raw_interval_cycles_plausible(interval_cycles)) return false;
  const uint32_t previous_ocxo_dwt = next_ocxo_dwt - interval_cycles;
  const uint32_t target_delta_cycles = target_dwt - previous_ocxo_dwt;
  return target_delta_cycles <= interval_cycles;
}

static bool delta_clock_candidate_capture_geometry(
    delta_clock_candidate_state_t& state,
    uint32_t target_dwt,
    uint32_t next_ocxo_dwt,
    uint32_t interval_cycles) {
  if (!delta_clock_candidate_geometry_valid(
          target_dwt, next_ocxo_dwt, interval_cycles)) {
    return false;
  }

  state.geometry_valid = true;
  state.geometry_target_dwt = target_dwt;
  state.geometry_next_ocxo_dwt = next_ocxo_dwt;
  state.geometry_interval_cycles = interval_cycles;
  return true;
}

static void delta_clock_candidate_fail(
    delta_clock_candidate_state_t& state,
    clocks_fragment_clock_candidate_status_t status,
    uint32_t public_count) {
  state.continuity_valid = false;
  state.geometry_valid = false;
  state.last_residual_available = false;
  state.last_residual_ns = 0;
  state.last_residual_ns_exact = 0.0;
  state.status = status;
  state.last_public_count = public_count;
}

static bool delta_clock_candidate_reanchor(
    delta_clock_candidate_state_t& state,
    uint32_t public_count,
    uint64_t phaseledger_ns,
    uint32_t target_dwt,
    uint32_t next_ocxo_dwt,
    uint32_t interval_cycles) {
  if (phaseledger_ns == 0ULL ||
      !delta_clock_candidate_capture_geometry(
          state, target_dwt, next_ocxo_dwt, interval_cycles)) {
    return false;
  }

  // START supplies private PPS0 geometry, so public PPS1 normally advances
  // independently.  RECOVER and FLASH_CUT may have no lawful pre-boundary
  // bracket; their first public row becomes an explicit splice anchor.
  state.ns = phaseledger_ns;
  state.fractional_ns = 0.0;
  state.last_public_count = public_count;
  state.interval_count = public_count >= state.start_public_count
      ? public_count - state.start_public_count
      : 0U;
  state.last_residual_available = false;
  state.last_residual_ns = 0;
  state.last_residual_ns_exact = 0.0;
  state.status = clocks_fragment_clock_candidate_status_t::SEEDED;
  return true;
}

static void delta_clock_candidate_advance(
    delta_clock_candidate_state_t& state,
    uint32_t public_count,
    const clock_science_row_t& row,
    const clocks_alpha_lane_forensics_t& clock_forensics,
    uint64_t phaseledger_ns,
    bool phaseledger_clockface_valid) {
  if (!state.seeded) {
    state.status = clocks_fragment_clock_candidate_status_t::UNAVAILABLE;
    state.last_public_count = public_count;
    return;
  }
  if (!state.continuity_valid) {
    // Preserve the first specific failure verdict.  Later rows may show that
    // the candidate remained unavailable, but they must not erase why the
    // independent clock stopped advancing.
    state.last_public_count = public_count;
    return;
  }

  if (public_count != state.last_public_count + 1U) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::PUBLIC_COUNT_GAP,
        public_count);
    return;
  }

  const uint32_t current_target_dwt = row.pps_vclock_dwt_at_edge;
  const uint32_t current_next_ocxo_dwt = clock_forensics.last_event_dwt;
  const uint32_t current_interval_cycles =
      clock_forensics.dwt_cycles_between_edges;
  const bool current_geometry_valid =
      clock_forensics.valid &&
      delta_clock_candidate_geometry_valid(
          current_target_dwt,
          current_next_ocxo_dwt,
          current_interval_cycles);

  if (!phaseledger_clockface_valid || phaseledger_ns == 0ULL ||
      !current_geometry_valid) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::DELTA_INVALID,
        public_count);
    return;
  }

  if (!state.geometry_valid) {
    if (!delta_clock_candidate_reanchor(
            state,
            public_count,
            phaseledger_ns,
            current_target_dwt,
            current_next_ocxo_dwt,
            current_interval_cycles)) {
      delta_clock_candidate_fail(
          state,
          clocks_fragment_clock_candidate_status_t::DELTA_INVALID,
          public_count);
    }
    return;
  }

  const uint32_t current_previous_ocxo_dwt =
      current_next_ocxo_dwt - current_interval_cycles;
  const uint32_t current_head_cycles =
      current_target_dwt - current_previous_ocxo_dwt;
  const uint32_t previous_tail_cycles =
      state.geometry_next_ocxo_dwt - state.geometry_target_dwt;
  const uint32_t reference_interval_cycles =
      current_target_dwt - state.geometry_target_dwt;

  const bool geometry_contiguous =
      current_previous_ocxo_dwt == state.geometry_next_ocxo_dwt;
  const bool pieces_valid =
      previous_tail_cycles <= state.geometry_interval_cycles &&
      current_head_cycles <= current_interval_cycles &&
      (uint64_t)previous_tail_cycles + (uint64_t)current_head_cycles ==
          (uint64_t)reference_interval_cycles;
  const bool row_matches_geometry =
      row.delta_raw_valid &&
      isfinite(row.delta_raw_fast_residual_ns_exact) &&
      row.delta_raw_reference_interval_cycles == reference_interval_cycles &&
      row.delta_raw_clock_interval_cycles == current_interval_cycles &&
      row.clock_observed_interval_cycles == current_interval_cycles;

  if (!geometry_contiguous || !pieces_valid || !row_matches_geometry ||
      !delta_raw_interval_cycles_plausible(reference_interval_cycles) ||
      !delta_raw_interval_cycles_plausible(state.geometry_interval_cycles) ||
      !delta_raw_interval_cycles_plausible(current_interval_cycles)) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::DELTA_INVALID,
        public_count);
    return;
  }

  // The VCLOCK window straddles the common OCXO boundary.  Convert each DWT
  // piece with the OCXO interval that actually contains it, then add the two
  // elapsed OCXO-nanosecond pieces.  This is the endpoint-aligned Delta clock.
  const double previous_tail_ns =
      ((double)CLOCKS_BETA_NS_PER_SECOND *
       (double)previous_tail_cycles) /
      (double)state.geometry_interval_cycles;
  const double current_head_ns =
      ((double)CLOCKS_BETA_NS_PER_SECOND *
       (double)current_head_cycles) /
      (double)current_interval_cycles;
  const double exact_increment_ns = previous_tail_ns + current_head_ns;
  const double increment_with_carry =
      state.fractional_ns + exact_increment_ns;
  if (!isfinite(exact_increment_ns) || !isfinite(increment_with_carry) ||
      increment_with_carry > (double)INT64_MAX ||
      increment_with_carry < 1.0) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::ARITHMETIC_FAILURE,
        public_count);
    return;
  }

  const int64_t increment_ns = beta_round_double_to_i64(increment_with_carry);
  if (increment_ns <= 0 ||
      UINT64_MAX - state.ns < (uint64_t)increment_ns) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::ARITHMETIC_FAILURE,
        public_count);
    return;
  }

  state.ns += (uint64_t)increment_ns;
  state.fractional_ns =
      increment_with_carry - (double)increment_ns;
  state.last_public_count = public_count;
  state.interval_count++;
  state.last_residual_available = true;
  state.last_residual_ns_exact =
      exact_increment_ns - (double)CLOCKS_BETA_NS_PER_SECOND;
  state.last_residual_ns =
      beta_round_double_to_i64(state.last_residual_ns_exact);
  state.status = clocks_fragment_clock_candidate_status_t::ADVANCED;

  if (!delta_clock_candidate_capture_geometry(
          state,
          current_target_dwt,
          current_next_ocxo_dwt,
          current_interval_cycles)) {
    delta_clock_candidate_fail(
        state,
        clocks_fragment_clock_candidate_status_t::DELTA_INVALID,
        public_count);
  }
}

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

// RECOVER deliberately cuts OCXO interval custody while preserving the installed
// service epoch.  Alpha's first post-recovery edge is therefore only a bookend;
// later science is released only when both lanes prove that the current interval,
// PhaseLedger suffix, and refined interval all descend from that fresh bookend.
// There is no fixed-N recovery quarantine.  The legacy quarantine counters remain
// as a dormant report/compatibility surface and should stay zero in proof-driven
// recovery.
static uint32_t g_science_residual_quarantine_remaining = 0;
static uint32_t g_science_residual_quarantine_begin_count = 0;
static uint32_t g_science_residual_quarantine_consumed_count = 0;
static uint32_t g_science_residual_quarantine_last_public_count = 0;

// A healthy proof normally appears almost immediately after the mandatory fresh
// bookend.  Keep ordinary proof search silent; the sixth failed current-row proof
// emits one observational CLOCKS_RECOVERY_STALLED event because that duration is
// more suggestive of a programming/design error than normal proof convergence latency.
static constexpr uint32_t CLOCKS_RECOVER_PROOF_WARN_AFTER_ATTEMPTS = 5U;
static uint32_t g_recover_proof_attempt_count = 0U;
static uint32_t g_recover_proof_last_attempt_pps_sequence = 0U;
static uint32_t g_recover_science_proof_release_count = 0U;
static uint32_t g_recover_proof_last_release_pps_sequence = 0U;
static volatile bool g_recover_proof_warning_pending = false;
static bool g_recover_proof_warning_published = false;

// Alpha-owned instrument statistics continue across RECOVER.  Beta gates the
// current row until proof is complete; raw interval testimony remains visible.

// RECOVER OCXO proof gate for the dead-producer restore transaction. Surviving
// producers never enter firmware RECOVER, so every RECOVER generation owns a fresh
// Alpha epoch whose first complete OCXO rows must prove new ancestry. The gate may
// first complete OCXO rows.  The gate remains finite: after timeout, campaign
// publication resumes in degraded mode and OCXO science remains quarantined/
// invalid until PhaseLedger/reattach evidence catches up.
static constexpr uint32_t CLOCKS_RECOVER_PROOF_TIMEOUT_CANDIDATES = 32U;
// Recovery publication is layered:
//   * Timeline readiness (PPS/VCLOCK, GNSS, DWT) permits a public row.
//   * OCXO clockface readiness proves a fresh post-RECOVER integer ledger.
//   * OCXO science readiness additionally proves the refined PhaseLedger
//     interval required by Welford and PPB.
//
// The Pi is expected to persist explicitly degraded timeline rows instead of
// restarting RECOVER.  A later science-ready row naturally clears degradation.
static constexpr bool     CLOCKS_RECOVER_PROOF_TIMEOUT_RELEASE_DEGRADED = true;
static volatile bool     g_recover_proof_active = false;
static volatile bool     g_recover_proof_degraded_active = false;
static volatile bool     g_recover_proof_clockface_ready = false;
static volatile bool     g_recover_proof_science_ready = false;
static uint32_t          g_recover_proof_begin_count = 0;
static uint32_t          g_recover_proof_hold_count = 0;
static uint32_t          g_recover_proof_release_count = 0;
static uint32_t          g_recover_proof_timeout_count = 0;
static uint32_t          g_recover_proof_degraded_release_count = 0;
static uint32_t          g_recover_proof_degraded_clear_count = 0;
static uint32_t          g_recover_proof_degraded_public_row_count = 0;
static uint32_t          g_recover_proof_degraded_science_suppressed_count = 0;
// Stall means no movement toward the currently missing OCXO proof, not merely
// a fixed number of degraded rows.  Sixty 1 Hz rows gives the resolver a full
// minute before a dedicated CLOCKS_RECOVERY_STALLED event is emitted.
static constexpr uint32_t CLOCKS_RECOVER_PROOF_DEGRADED_STALL_CANDIDATES = 60U;
static volatile bool     g_recover_proof_stalled = false;
static uint32_t          g_recover_proof_stall_count = 0;
static uint32_t          g_recover_proof_stall_publish_count = 0;
static uint32_t          g_recover_proof_progress_count = 0;
static uint32_t          g_recover_proof_progress_resume_count = 0;
static uint32_t          g_recover_proof_no_progress_row_count = 0;
static uint32_t          g_recover_proof_degraded_window_row_count = 0;
static uint32_t          g_recover_proof_last_progress_public_count = 0;
static uint32_t          g_recover_proof_last_stall_public_count = 0;
static char              g_recover_proof_stall_reason[64] = "idle";
static uint32_t          g_recover_proof_hidden_candidate_count = 0;
static uint32_t          g_recover_proof_last_hidden_public_count = 0;
static uint32_t          g_recover_proof_last_release_public_count = 0;
static uint32_t          g_recover_proof_last_degraded_release_public_count = 0;
static uint32_t          g_recover_proof_last_degraded_public_count = 0;
static char              g_recover_proof_last_reason[64] = "idle";
static clocks_alpha_recover_proof_snapshot_t
    g_recover_proof_last_ocxo1 DMAMEM = {};
static clocks_alpha_recover_proof_snapshot_t
    g_recover_proof_last_ocxo2 DMAMEM = {};

// Recovery stall detection tracks one-way readiness milestones, not ordinary
// per-second activity.  Capture/sample/forensics counters advance on a healthy
// degraded stream even when the missing PhaseLedger proof never gets closer;
// counting them as progress makes a true science stall impossible to detect.
// The seen mask is monotonic for one RECOVER generation: a bit can prove new
// progress only once.
struct recover_proof_progress_marker_t {
  uint32_t seen_readiness_mask = 0;
};

static bool g_recover_proof_progress_marker_valid = false;
static recover_proof_progress_marker_t g_recover_proof_progress_ocxo1 = {};
static recover_proof_progress_marker_t g_recover_proof_progress_ocxo2 = {};

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

// Surviving-Alpha campaign rearm flight recorder.  REARM is deliberately
// Beta-only: it restores the durable recording coordinate after a Pi outage or
// watchdog surrender without touching Alpha statistics, clockfaces, or epoch.
static uint32_t          g_rearm_request_count = 0;
static uint32_t          g_rearm_commit_count = 0;
static uint32_t          g_rearm_reject_count = 0;
static uint64_t          g_rearm_last_base_count = 0;
static uint64_t          g_rearm_last_expected_first_public_count = 0;
static uint64_t          g_rearm_last_base_gnss_ns = 0;
static uint64_t          g_rearm_last_base_dwt_ns = 0;
static uint64_t          g_rearm_last_base_ocxo1_ns = 0;
static uint64_t          g_rearm_last_base_ocxo2_ns = 0;
static char              g_rearm_last_campaign[64] = {0};
static char              g_rearm_last_status[48] = "idle";

// RECOVER lifecycle flight recorder.  RECOVER is intentionally promoted to a
// visible campaign state at command time so watchdog recovery cannot leave the
// firmware in STOPPED + request_recover limbo while Pi waits for campaign record.
static uint32_t          g_recover_lifecycle_begin_count = 0;
static uint32_t          g_recover_lifecycle_pps_gate_count = 0;
static uint32_t          g_recover_lifecycle_command_custody_reset_count = 0;
static uint32_t          g_recover_lifecycle_gate_custody_reset_count = 0;

enum class recover_lifecycle_mode_t : uint8_t {
  NONE                  = 0,
  DEAD_PRODUCER_RESTORE = 1,
};

static volatile recover_lifecycle_mode_t g_recover_lifecycle_mode =
    recover_lifecycle_mode_t::NONE;
static bool     g_recover_lifecycle_dead_producer_restore_epoch_ready = false;
static uint32_t g_recover_lifecycle_dead_producer_restore_begin_count = 0;
static uint32_t g_recover_lifecycle_dead_producer_restore_wait_count = 0;
static uint32_t g_recover_lifecycle_dead_producer_restore_ready_count = 0;
static uint32_t g_recover_lifecycle_dead_producer_restore_commit_count = 0;
static uint32_t g_recover_lifecycle_dead_producer_restore_start_failure_count = 0;
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
// One-second science and Welfords remain independently
// gated until fresh post-recovery interval custody is complete.
static volatile bool g_recover_continuity_align_pending = false;
// Presentation alignment participates in the recovery science hold until the
// fresh dead-producer Alpha ancestry proves the current row.
static bool g_recover_continuity_align_science_hold = false;
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

static FLASHMEM void recover_proof_reset(const char* reason);
static FLASHMEM void recover_proof_begin(void);
static FLASHMEM bool recover_proof_should_hold(void);
static FLASHMEM bool recover_proof_degraded_science_hold_active(void);
static FLASHMEM void recover_proof_apply_degraded_science_hold(clock_science_row_t& row);

static uint32_t clocks_row_lifecycle_science_hold_flags(void) {
  uint32_t flags = 0U;
  if (g_recover_proof_active) flags |= 1U << 0;
  if (g_recover_proof_degraded_active) flags |= 1U << 1;
  if (g_science_residual_quarantine_remaining != 0U) flags |= 1U << 2;
  if (g_recover_continuity_align_pending &&
      g_recover_continuity_align_science_hold) {
    flags |= 1U << 3;
  }
  return flags;
}

static void pps_interval_residuals_reset(void) {
  ocxo_science_totals_reset();
  g_science_residual_quarantine_remaining = 0;
}

static void pps_interval_residuals_begin_recover_quarantine(uint32_t rows) {
  pps_interval_residuals_reset();
  g_science_residual_quarantine_remaining = rows;
  g_science_residual_quarantine_begin_count++;
}

static uint64_t science_render_legacy_clock_interval_ns(
    const clock_science_row_t& row) {
  if (!row.valid) return 0ULL;

  if (clocks_ocxo_counterledger_mode()) {
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
  if (clocks_ocxo_counterledger_mode()) {
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

  return r;
}

static uint64_t current_raw_gnss_ns(void) {
  // At the selected PPS/VCLOCK edge, GNSS time is identity, not discovery.
  // The DWT projection path remains an Alpha edge self-check; it must not
  // author the public GNSS ledger because doing so turns bridge rounding/
  // projection error into a common-mode OCXO residual.
  return g_gnss_ns_at_pps_vclock;
}

static uint64_t current_raw_ocxo_measured_ns(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return clocks_ocxo1_measured_gnss_ns_now();
    case time_clock_id_t::OCXO2: return clocks_ocxo2_measured_gnss_ns_now();
    default:                      return 0ULL;
  }
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

static uint64_t current_raw_ocxo_ns(time_clock_id_t clock) {
  if (clocks_ocxo_counterledger_mode()) {
    const uint64_t ns = current_raw_counterledger_ns(clock);
    if (ns != 0ULL) return ns;
  }

  // Traditional authority uses Alpha's PPS-row measured/projection surface.
  // CounterLedger mode falls back here only before that ledger has a valid row.
  return current_raw_ocxo_measured_ns(clock);
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
      current_raw_ocxo_measured_ns(time_clock_id_t::OCXO1), state.instrument_ocxo1_ns);
  g_instrument_ocxo2_offset = campaign_public_offset_for_recovered_value(
      current_raw_ocxo_measured_ns(time_clock_id_t::OCXO2), state.instrument_ocxo2_ns);
  g_instrument_continuity_active = true;
  g_instrument_continuity_install_count++;
}

static bool clocks_recovery_commit_statistics_and_clockfaces(
    const clocks_recovery_restore_state_t& state) {
  if (!clocks_recovery_restore_statistics_valid(state)) return false;
  if (!clocks_alpha_instrument_stats_restore(&state.stats)) return false;
  instrument_continuity_install(state);
  return true;
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

static void recover_continuity_align_arm(bool science_hold = true) {
  g_recover_continuity_align_pending = true;
  g_recover_continuity_align_science_hold = science_hold;
  g_recover_continuity_align_requested_public_count =
      (uint32_t)(campaign_seconds + 1ULL);
  recover_continuity_set_reason(science_hold
      ? "armed_for_first_recovered_public_row"
      : "armed_campaign_bootstrap_public_row");
}

static void recover_continuity_align_reset(const char* reason) {
  g_recover_continuity_align_pending = false;
  g_recover_continuity_align_science_hold = false;
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
      clocks_ocxo_counterledger_mode();
  const uint64_t raw_o1 = counterledger_authority
      ? raw_c1
      : current_raw_ocxo_ns(time_clock_id_t::OCXO1);
  const uint64_t raw_o2 = counterledger_authority
      ? raw_c2
      : current_raw_ocxo_ns(time_clock_id_t::OCXO2);
  const uint64_t target_o1 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo1_ns);
  const uint64_t target_o2 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo2_ns);

  if (target_o1 == 0ULL || target_o2 == 0ULL) {
    g_recover_continuity_align_pending = false;
    g_recover_continuity_align_science_hold = false;
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

  const uint64_t raw_m1 = current_raw_ocxo_measured_ns(time_clock_id_t::OCXO1);
  const uint64_t raw_m2 = current_raw_ocxo_measured_ns(time_clock_id_t::OCXO2);
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
  g_recover_continuity_align_science_hold = false;
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

static void campaign_public_counterledger_offsets_reset_for_projected_resume(void) {
  const uint64_t o1 = current_raw_counterledger_ns(time_clock_id_t::OCXO1);
  const uint64_t o2 = current_raw_counterledger_ns(time_clock_id_t::OCXO2);
  g_campaign_public_counterledger_ocxo1_offset = o1
      ? campaign_public_offset_for_recovered_value(o1, recover_ocxo1_ns)
      : 0;
  g_campaign_public_counterledger_ocxo2_offset = o2
      ? campaign_public_offset_for_recovered_value(o2, recover_ocxo2_ns)
      : 0;
}

static void campaign_public_offsets_reset_to_current(void) {
  recover_continuity_align_reset("offsets_reset_to_current");
  g_campaign_public_dwt_offset =
      campaign_public_offset_to_zero(g_dwt_cycle_count_total);
  g_campaign_public_gnss_offset =
      campaign_public_offset_to_zero(current_raw_gnss_ns());
  g_campaign_public_ocxo1_offset =
      campaign_public_offset_to_zero(current_raw_ocxo_ns(time_clock_id_t::OCXO1));
  g_campaign_public_ocxo2_offset =
      campaign_public_offset_to_zero(current_raw_ocxo_ns(time_clock_id_t::OCXO2));
  g_campaign_public_ocxo1_measured_offset =
      campaign_public_offset_to_zero(current_raw_ocxo_measured_ns(time_clock_id_t::OCXO1));
  g_campaign_public_ocxo2_measured_offset =
      campaign_public_offset_to_zero(current_raw_ocxo_measured_ns(time_clock_id_t::OCXO2));
  campaign_public_counterledger_offsets_reset_to_current();
  delta_clock_candidates_seed_zero();
}

static void campaign_public_offsets_reset_for_projected_resume(void) {
  recover_continuity_align_reset("projected_resume_offsets_seeded");
  g_campaign_public_dwt_offset =
      campaign_public_offset_for_recovered_value(g_dwt_cycle_count_total,
                                                 dwt_ns_to_cycles(recover_dwt_ns));
  g_campaign_public_gnss_offset =
      campaign_public_offset_for_recovered_value(current_raw_gnss_ns(),
                                                 recover_gnss_ns);
  g_campaign_public_ocxo1_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo_ns(time_clock_id_t::OCXO1),
                                                 recover_ocxo1_ns);
  g_campaign_public_ocxo2_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo_ns(time_clock_id_t::OCXO2),
                                                 recover_ocxo2_ns);
  g_campaign_public_ocxo1_measured_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo_measured_ns(time_clock_id_t::OCXO1),
                                                 recover_ocxo1_ns);
  g_campaign_public_ocxo2_measured_offset =
      campaign_public_offset_for_recovered_value(current_raw_ocxo_measured_ns(time_clock_id_t::OCXO2),
                                                 recover_ocxo2_ns);
  campaign_public_counterledger_offsets_reset_for_projected_resume();
  delta_clock_candidates_seed_projected_resume();
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


static bool campaign_start_phaseledger_counter_identity_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  // Alpha commits only the whole-cell identity derived from programmed OCXO
  // compare targets plus PPS DWT-at-edge.  The ambient PPS CNTR capture remains
  // report-only and must not participate in START admission.
  return s.phase_valid &&
         s.phase_pps_sequence == s.pps_sequence &&
         s.phase_implied_counter32_at_pps == s.last_counter32;
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
         campaign_start_phaseledger_counter_identity_ready(s) &&
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
  if (!campaign_start_phaseledger_counter_identity_ready(s)) {
    snprintf(reason, sizeof(reason), "%s_counter_identity", lane);
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
  g_start_handoff_last_raw_ocxo1_ns = current_raw_ocxo_ns(time_clock_id_t::OCXO1);
  g_start_handoff_last_raw_ocxo2_ns = current_raw_ocxo_ns(time_clock_id_t::OCXO2);
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

  if (clocks_ocxo_counterledger_mode()) {
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
static FLASHMEM void recover_proof_begin(void);
static FLASHMEM void recover_proof_reset(const char* reason);

static void campaign_warmup_begin(campaign_warmup_mode_t mode) {
  g_campaign_warmup_suppressed_total = 0;

  if (mode == campaign_warmup_mode_t::RECOVER) {
    // RECOVER no longer buries fixed rows, but the fresh dead-producer Alpha
    // ancestry must prove fresh OCXO ancestry before unrestricted science resumes.
    interrupt_dwt_publication_launch_acquisition_end();
    g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
    g_campaign_warmup_remaining = 0;
    campaign_start_prologue_reset("recover_no_prologue");
    // RECOVER offsets are installed in the RECOVER gate before this fresh-Alpha
    // proof court begins.  Do not recompute them here from a partially matured
    // post-bootstrap presentation.
    recover_proof_begin();
    return;
  }

  if (mode == campaign_warmup_mode_t::START) {
    recover_proof_reset("not_recovering");

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
  recover_proof_reset("none_lifecycle");
  campaign_public_offsets_reset_to_current();
}

static bool campaign_warmup_active(void) {
  // Treat the entire RECOVER transition as not-yet-campaign-continuous for
  // watchdog arming.  The first degraded/quarantined rows exist only to prove
  // liveness and prove fresh custody to the Pi; a DWT publication court verdict
  // during that window must not fire a second campaign-surrender watchdog and
  // stop the only rows the Pi can use to observe recovery progress.
  return g_campaign_warmup_mode != campaign_warmup_mode_t::NONE ||
         g_recover_proof_active ||
         g_recover_proof_degraded_active ||
         g_science_residual_quarantine_remaining != 0U ||
         clocks_campaign_recovery_lifecycle_active();
}

static void recover_proof_set_reason(const char* reason) {
  safeCopy(g_recover_proof_last_reason,
           sizeof(g_recover_proof_last_reason),
           reason ? reason : "recover_proof");
}

static void recover_proof_set_stall_reason(const char* reason) {
  safeCopy(g_recover_proof_stall_reason,
           sizeof(g_recover_proof_stall_reason),
           reason ? reason : "recover_proof_stall");
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

static uint32_t recover_proof_readiness_mask(
    const clocks_alpha_recover_proof_snapshot_t& s) {
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

static uint32_t recover_proof_readiness_score(
    const clocks_alpha_recover_proof_snapshot_t& s) {
  uint32_t mask = recover_proof_readiness_mask(s);
  uint32_t score = 0U;
  while (mask != 0U) {
    score += mask & 1U;
    mask >>= 1U;
  }
  return score;
}

static bool recover_proof_note_lane_progress(
    recover_proof_progress_marker_t& marker,
    const clocks_alpha_recover_proof_snapshot_t& s) {
  const uint32_t current = recover_proof_readiness_mask(s);
  const uint32_t newly_proven = current & ~marker.seen_readiness_mask;
  marker.seen_readiness_mask |= current;
  return newly_proven != 0U;
}

static bool recover_proof_note_progress(uint32_t public_count) {
  const bool first = !g_recover_proof_progress_marker_valid;
  const bool ocxo1_progress = recover_proof_note_lane_progress(
      g_recover_proof_progress_ocxo1,
      g_recover_proof_last_ocxo1);
  const bool ocxo2_progress = recover_proof_note_lane_progress(
      g_recover_proof_progress_ocxo2,
      g_recover_proof_last_ocxo2);

  g_recover_proof_progress_marker_valid = true;
  const bool progressed = first || ocxo1_progress || ocxo2_progress;
  if (progressed) {
    g_recover_proof_progress_count++;
    g_recover_proof_last_progress_public_count = public_count;
  }
  return progressed;
}

static void recover_proof_add_stall_lane(
    Payload& parent,
    const char* key,
    const clocks_alpha_recover_proof_snapshot_t& s) {
  clocks_payload_owner_assert(clocks_payload_owner_t::EVENT);
  Payload lane;
  lane.add("clockface_ready", s.clockface_ready);
  lane.add("science_ready", s.science_ready);
  lane.add("readiness_score", recover_proof_readiness_score(s));
  lane.add("readiness_mask", recover_proof_readiness_mask(s));
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

static void recover_proof_publish_stalled_event(void) {
  const clocks_payload_custody_t payload_custody(clocks_payload_owner_t::EVENT);
  Payload p;
  p.add("schema", "CLOCKS_RECOVERY_STALLED_V1");
  p.add("reason", "ocxo_science_reattach_no_progress");
  p.add("source", "TEENSY_CLOCKS_RECOVERY_LIVENESS");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("recovery_generation", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("clockface_ready", (bool)g_recover_proof_clockface_ready);
  p.add("science_ready", (bool)g_recover_proof_science_ready);
  p.add("degraded_publication_active",
        (bool)g_recover_proof_degraded_active);
  p.add("no_progress_rows", g_recover_proof_no_progress_row_count);
  p.add("stall_threshold_rows",
        (uint32_t)CLOCKS_RECOVER_PROOF_DEGRADED_STALL_CANDIDATES);
  p.add("last_progress_public_count",
        g_recover_proof_last_progress_public_count);
  p.add("stall_count", g_recover_proof_stall_count);
  p.add("progress_count", g_recover_proof_progress_count);
  p.add("progress_resume_count",
        g_recover_proof_progress_resume_count);

  Payload lanes;
  recover_proof_add_stall_lane(
      lanes, "ocxo1", g_recover_proof_last_ocxo1);
  recover_proof_add_stall_lane(
      lanes, "ocxo2", g_recover_proof_last_ocxo2);
  p.add_object("lanes", lanes);

  publish("CLOCKS_RECOVERY_STALLED", p);
  g_recover_proof_stall_publish_count++;
}

static FLASHMEM bool recover_proof_refresh_ready(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_REFRESH_READY);

  // Write directly into the cached recovery flight-recorder snapshots. This
  // keeps the recovery gate off the large-local-object path; REPORT_RECOVERY
  // later publishes these cached facts without re-entering Alpha or process_time.
  const bool ocxo1_snapshot_ok =
      clocks_alpha_ocxo_recover_proof_snapshot(
          time_clock_id_t::OCXO1, &g_recover_proof_last_ocxo1);
  const bool ocxo2_snapshot_ok =
      clocks_alpha_ocxo_recover_proof_snapshot(
          time_clock_id_t::OCXO2, &g_recover_proof_last_ocxo2);

  g_recover_proof_clockface_ready =
      ocxo1_snapshot_ok && ocxo2_snapshot_ok &&
      g_recover_proof_last_ocxo1.clockface_ready &&
      g_recover_proof_last_ocxo2.clockface_ready;
  g_recover_proof_science_ready =
      ocxo1_snapshot_ok && ocxo2_snapshot_ok &&
      g_recover_proof_last_ocxo1.science_ready &&
      g_recover_proof_last_ocxo2.science_ready;

  return g_recover_proof_science_ready;
}
static FLASHMEM void recover_proof_reset(const char* reason) {
  recover_continuity_align_reset(reason ? reason : "reattach_reset");
  g_recover_proof_active = false;
  g_recover_proof_degraded_active = false;
  g_recover_proof_clockface_ready = false;
  g_recover_proof_science_ready = false;
  g_recover_proof_stalled = false;
  g_recover_proof_no_progress_row_count = 0;
  g_recover_proof_degraded_window_row_count = 0;
  g_recover_proof_last_progress_public_count = 0;
  g_recover_proof_progress_marker_valid = false;
  g_recover_proof_progress_ocxo1 = recover_proof_progress_marker_t{};
  g_recover_proof_progress_ocxo2 = recover_proof_progress_marker_t{};
  recover_proof_set_stall_reason(reason ? reason : "reset");
  g_recover_proof_hidden_candidate_count = 0;
  g_recover_proof_last_hidden_public_count = 0;
  g_recover_proof_last_degraded_public_count = 0;
  g_recover_proof_last_ocxo1 = clocks_alpha_recover_proof_snapshot_t{};
  g_recover_proof_last_ocxo2 = clocks_alpha_recover_proof_snapshot_t{};
  g_recover_proof_attempt_count = 0U;
  g_recover_proof_last_attempt_pps_sequence = 0U;
  g_recover_proof_last_release_pps_sequence = 0U;
  g_recover_proof_warning_pending = false;
  g_recover_proof_warning_published = false;
  recover_proof_set_reason(reason ? reason : "reset");
}

static FLASHMEM void recover_proof_begin(void) {
  recover_continuity_align_reset("recover_clockface_alignment_reset");
  recover_continuity_align_arm();
  g_recover_proof_active = true;
  g_recover_proof_degraded_active = false;
  g_recover_proof_clockface_ready = false;
  g_recover_proof_science_ready = false;
  g_recover_proof_stalled = false;
  g_recover_proof_no_progress_row_count = 0;
  g_recover_proof_degraded_window_row_count = 0;
  g_recover_proof_last_progress_public_count = 0;
  g_recover_proof_progress_marker_valid = false;
  g_recover_proof_progress_ocxo1 = recover_proof_progress_marker_t{};
  g_recover_proof_progress_ocxo2 = recover_proof_progress_marker_t{};
  recover_proof_set_stall_reason("not_stalled");
  g_recover_proof_hidden_candidate_count = 0;
  g_recover_proof_last_hidden_public_count = 0;
  g_recover_proof_last_degraded_public_count = 0;
  g_recover_proof_begin_count++;
  g_recover_proof_last_ocxo1 = clocks_alpha_recover_proof_snapshot_t{};
  g_recover_proof_last_ocxo2 = clocks_alpha_recover_proof_snapshot_t{};
  g_recover_proof_attempt_count = 0U;
  g_recover_proof_last_attempt_pps_sequence = 0U;
  g_recover_proof_last_release_pps_sequence = 0U;
  g_recover_proof_warning_pending = false;
  g_recover_proof_warning_published = false;
  recover_proof_set_reason("waiting_for_ocxo_reattach");
}

static FLASHMEM void recover_proof_release(const char* reason, bool degraded) {
  g_recover_proof_active = false;
  g_recover_proof_release_count++;
  g_recover_proof_last_release_public_count =
      (uint32_t)(campaign_seconds + 1ULL);

  if (degraded) {
    g_recover_proof_degraded_active = true;
    g_recover_proof_stalled = false;
    g_recover_proof_degraded_window_row_count = 0;
    recover_proof_set_stall_reason("not_stalled");
    g_recover_proof_degraded_release_count++;
    g_recover_proof_last_degraded_release_public_count =
        g_recover_proof_last_release_public_count;
    g_recover_proof_last_degraded_public_count =
        g_recover_proof_last_release_public_count;
  } else {
    // Campaign clockface continuity was armed when RECOVER proof began
    // and therefore precedes every public row, including degraded testimony.
    // Science readiness must not author a second presentation intercept.
    if (g_recover_proof_degraded_active) {
      g_recover_proof_degraded_active = false;
      g_recover_proof_degraded_clear_count++;
    }
  }

  recover_proof_set_reason(reason ? reason : "ocxo_reattach_release");
}

static bool recover_proof_lane_current_row_ready(
    const clocks_alpha_recover_proof_snapshot_t& lane,
    uint32_t pps_sequence) {
  if (!lane.science_ready || pps_sequence == 0U) return false;

  if (!lane.counterledger_mode) {
    // Legacy projection mode has no RECOVER-specific ancestry counters.  Its
    // existing science_ready surface already requires fresh forensics + edge
    // history; additionally bind the proof to this exact PPS identity.
    return lane.edge_history_current_valid &&
           lane.edge_history_previous_valid &&
           lane.projection_pps_sequence == pps_sequence;
  }

  const bool exact_current_identity =
      lane.counterledger_snapshot_ok &&
      lane.counterledger_valid &&
      lane.counterledger_capture_ready &&
      lane.counterledger_interval_valid &&
      lane.counterledger_phase_valid &&
      lane.counterledger_phase_lag_ok &&
      lane.counterledger_refined_valid &&
      lane.counterledger_refined_interval_valid &&
      lane.counterledger_pps_sequence == pps_sequence &&
      lane.counterledger_phase_pps_sequence == pps_sequence &&
      lane.counterledger_last_sample_pps_sequence == pps_sequence &&
      lane.counterledger_last_sample_previous_pps_sequence + 1U == pps_sequence &&
      lane.counterledger_last_phase_resolve_pps_sequence == pps_sequence;
  if (!exact_current_identity) return false;

  // RECOVER owns a brand-new SmartZero epoch, so no pre-recovery edge can
  // survive into this interval. Exact current-row readiness is sufficient proof.
  return true;
}

static FLASHMEM bool recover_proof_driven_release_try(uint32_t pps_sequence) {
  if (pps_sequence == 0U ||
      (!g_recover_proof_active && !g_recover_proof_degraded_active)) {
    return false;
  }

  (void)recover_proof_refresh_ready();
  const bool ocxo1_ready = recover_proof_lane_current_row_ready(
      g_recover_proof_last_ocxo1, pps_sequence);
  const bool ocxo2_ready = recover_proof_lane_current_row_ready(
      g_recover_proof_last_ocxo2, pps_sequence);

  if (ocxo1_ready && ocxo2_ready) {
    // Release before Alpha reads lifecycle hold_flags.  No value is repaired or
    // synthesized: this merely removes the recovery hold because both current
    // lanes have independently proved clean post-boundary ancestry.  Do not reset
    // Beta residual state here: the previous recovered row is the lawful left
    // bookend required by this proving row's one-second Delta interval.
    const bool was_stalled = g_recover_proof_stalled;
    if (g_recover_proof_active) {
      recover_proof_release("ocxo_science_proof_driven_release", false);
    } else if (g_recover_proof_degraded_active) {
      // A prior clockface-only release already incremented release_count.  Clear
      // only the degraded science layer here so one RECOVER generation still has
      // one presentation-release accounting event.
      g_recover_proof_degraded_active = false;
      g_recover_proof_degraded_clear_count++;
      recover_proof_set_reason("ocxo_science_proof_driven_release");
    }
    g_recover_proof_stalled = false;
    if (was_stalled) g_recover_proof_progress_resume_count++;
    g_recover_proof_no_progress_row_count = 0U;
    g_recover_proof_degraded_window_row_count = 0U;
    recover_proof_set_stall_reason("cleared_by_proof_driven_release");
    g_recover_science_proof_release_count++;
    g_recover_proof_last_release_pps_sequence = pps_sequence;
    g_recover_proof_warning_pending = false;
    return true;
  }

  // Count only real science-proof attempts after both integer OCXO clockfaces
  // exist.  Earlier rows are clockface acquisition, not failed science proofs.
  if (g_recover_proof_clockface_ready &&
      g_recover_proof_last_attempt_pps_sequence != pps_sequence) {
    g_recover_proof_last_attempt_pps_sequence = pps_sequence;
    g_recover_proof_attempt_count++;
    if (g_recover_proof_attempt_count > CLOCKS_RECOVER_PROOF_WARN_AFTER_ATTEMPTS &&
        !g_recover_proof_warning_published) {
      g_recover_proof_warning_pending = true;
    }
  }
  return false;
}

static void recover_proof_warning_publish_if_pending(void) {
  const clocks_payload_custody_t payload_custody(clocks_payload_owner_t::EVENT);
  if (!g_recover_proof_warning_pending || g_recover_proof_warning_published) {
    return;
  }

  Payload p;
  p.add("schema", "CLOCKS_RECOVERY_STALLED_V1");
  p.add("reason", "ocxo_science_proof_not_established");
  p.add("source", "TEENSY_CLOCKS_RECOVERY_PROOF");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("recovery_generation", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("clockface_ready", (bool)g_recover_proof_clockface_ready);
  p.add("science_ready", (bool)g_recover_proof_science_ready);
  p.add("proof_attempts", g_recover_proof_attempt_count);
  p.add("proof_warn_after_attempts",
        (uint32_t)CLOCKS_RECOVER_PROOF_WARN_AFTER_ATTEMPTS);
  p.add("last_attempt_pps_sequence",
        g_recover_proof_last_attempt_pps_sequence);

  Payload lanes;
  recover_proof_add_stall_lane(
      lanes, "ocxo1", g_recover_proof_last_ocxo1);
  recover_proof_add_stall_lane(
      lanes, "ocxo2", g_recover_proof_last_ocxo2);
  p.add_object("lanes", lanes);

  publish("CLOCKS_RECOVERY_STALLED", p);
  g_recover_proof_stall_publish_count++;
  g_recover_proof_warning_published = true;
  g_recover_proof_warning_pending = false;
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
           clocks_ocxo_counterledger_mode()
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
  recover_proof_reset("warmup_reset");
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
  return mode == recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE
      ? "DEAD_PRODUCER_RESTORE"
      : "NONE";
}


static bool recover_restore_court_ready_now(void) {
  // RESTORE_MONITOR may be accepted before startup SmartZero completes.  Its
  // command handler already preserves an in-flight SmartZero acquisition and
  // commits the structured restore only on the next completed PPS after the
  // Alpha epoch becomes ready.  Report that control-plane admission fact
  // separately from recover_epoch_ready, which continues to mean that an
  // installed SmartZero-backed Alpha epoch exists right now.
  return campaign_state == clocks_campaign_state_t::STOPPED &&
         !request_start &&
         !request_stop &&
         !request_recover &&
         !request_rearm &&
         !request_zero &&
         !request_flash_cut &&
         !g_clocks_restore_requested &&
         !clocks_campaign_recovery_lifecycle_active();
}

static bool recover_lifecycle_prepare_dead_producer_restore(void) {
  g_recover_lifecycle_mode = recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE;
  g_recover_lifecycle_dead_producer_restore_epoch_ready = false;
  g_recover_lifecycle_dead_producer_restore_begin_count++;

  if (clocks_alpha_installed_smartzero_backing_epoch()) {
    g_recover_lifecycle_dead_producer_restore_epoch_ready = true;
    g_recover_lifecycle_dead_producer_restore_ready_count++;
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

  if (!clocks_alpha_begin_smartzero_epoch("recover_dead_producer_restore")) {
    g_recover_lifecycle_dead_producer_restore_start_failure_count++;
    return false;
  }

  return true;
}

static bool recover_lifecycle_enter_from_command(const char* reason) {
  g_recover_lifecycle_begin_count++;
  g_recover_lifecycle_last_begin_campaign_seconds = (uint32_t)campaign_seconds;
  recover_lifecycle_set_reason(reason ? reason : "recover_command_armed");
  recover_lifecycle_set_abort_reason("none");

  // RECOVER has one meaning: install a Pi-authored dead-producer desired state.
  // A surviving producer is never sent here; Pi proves and adopts it read-only.
  interrupt_recover_reset_publication_custody();
  clocks_fragment_recover_reset_publication_custody();
  g_recover_lifecycle_command_custody_reset_count++;

  if (!recover_lifecycle_prepare_dead_producer_restore()) {
    recover_lifecycle_set_reason("recover_dead_producer_restore_start_failed");
    return false;
  }

  clocks_watchdog_clear_surrender_for_new_lifecycle();
  clocks_watchdog_disarm_campaign_publication();
  campaign_state = clocks_campaign_state_t::RECOVERING;
  recover_lifecycle_set_reason("recover_dead_producer_restore_armed");
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
          recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE &&
      !clocks_alpha_installed_smartzero_backing_epoch()) {
    interrupt_smartzero_abort();
    clocks_alpha_smartzero_pending_clear();
  }

  clocks_alpha_ocxo_grid_rephase_acknowledge(
      clocks_alpha_ocxo_grid_rephase_owner_t::RECOVER);
  request_recover = false;
  request_rearm = false;
  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  request_start = false;
  request_stop = false;
  request_zero = false;
  flash_cut_clear_pending();
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  // RECOVER_ABORT is also a publication boundary. If the failed attempt
  // accumulated a new undeliverable retry, retire it so always-on Alpha rows
  // can resume publication instead of inheriting the failed transaction.
  clocks_fragment_recover_reset_publication_custody();
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

static FLASHMEM bool recover_proof_should_hold(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_SHOULD_HOLD);
  if (!g_recover_proof_active) return false;

  (void)recover_proof_refresh_ready();
  (void)recover_proof_note_progress((uint32_t)campaign_seconds);

  // Generic science_ready is useful progress testimony, but only the pre-Alpha
  // proof-driven court may release science custody for a row.  Clockface-ready
  // candidates may still resume the truthful timeline in degraded mode.
  if (g_recover_proof_clockface_ready) {
    // Timeline and both OCXO integer clockfaces are now fresh.  Publish the row
    // immediately, but keep refined science/Welford gated until the
    // PhaseLedger interval proves itself.
    recover_proof_release(
        "ocxo_clockface_ready_science_initializing", true);
    return false;
  }

  // Legacy "hidden candidate" counters now count emitted recovery candidates.
  // The identity advances through the normal per-second path below; this gate
  // only observes how long proof convergence remains incomplete.
  g_recover_proof_hold_count++;
  g_recover_proof_hidden_candidate_count++;
  g_recover_proof_last_hidden_public_count =
      (uint32_t)(campaign_seconds + 1ULL);

  if (g_recover_proof_hidden_candidate_count >=
      CLOCKS_RECOVER_PROOF_TIMEOUT_CANDIDATES) {
    g_recover_proof_timeout_count++;
    if (CLOCKS_RECOVER_PROOF_TIMEOUT_RELEASE_DEGRADED) {
      recover_proof_release("ocxo_reattach_timeout_degraded_release", true);
      return false;
    }
    recover_proof_set_reason("ocxo_reattach_timeout_candidates_emitted");
  } else if (!g_recover_proof_last_ocxo1.clockface_ready &&
             !g_recover_proof_last_ocxo2.clockface_ready) {
    recover_proof_set_reason("emitting_without_both_ocxo_clockfaces");
  } else if (!g_recover_proof_last_ocxo1.clockface_ready) {
    recover_proof_set_reason("emitting_without_ocxo1_clockface");
  } else if (!g_recover_proof_last_ocxo2.clockface_ready) {
    recover_proof_set_reason("emitting_without_ocxo2_clockface");
  } else {
    recover_proof_set_reason("emitting_while_ocxo_science_initializes");
  }

  // Reattachment state remains active, but it no longer gates campaign record.
  return false;
}

static FLASHMEM bool recover_proof_degraded_science_hold_active(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_DEGRADED_HOLD);
  if (!g_recover_proof_degraded_active) return false;

  (void)recover_proof_refresh_ready();
  recover_proof_warning_publish_if_pending();
  const bool progressed =
      recover_proof_note_progress((uint32_t)campaign_seconds);

  // Stay degraded until the pre-Alpha proof court releases both lanes.  Merely
  // observing generic science_ready here is too late to admit this row into
  // Alpha's already-completed statistics transaction and is not a substitute for
  // explicit post-recovery ancestry.
  g_recover_proof_degraded_public_row_count++;
  g_recover_proof_degraded_window_row_count++;
  g_recover_proof_last_degraded_public_count =
      (uint32_t)campaign_seconds;

  if (progressed) {
    g_recover_proof_no_progress_row_count = 0;
    if (g_recover_proof_stalled) {
      g_recover_proof_stalled = false;
      g_recover_proof_progress_resume_count++;
      recover_proof_set_stall_reason("progress_resumed");
    }
  } else {
    g_recover_proof_no_progress_row_count++;
  }

  if (!g_recover_proof_stalled &&
      g_recover_proof_no_progress_row_count >=
          CLOCKS_RECOVER_PROOF_DEGRADED_STALL_CANDIDATES) {
    g_recover_proof_stalled = true;
    g_recover_proof_stall_count++;
    g_recover_proof_last_stall_public_count =
        (uint32_t)campaign_seconds;
    recover_proof_set_stall_reason(
        "ocxo_science_reattach_no_progress");
    recover_proof_publish_stalled_event();
  }

  recover_proof_set_reason(
      g_recover_proof_stalled
          ? "degraded_publication_science_stalled"
          : "degraded_publication_science_initializing");
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

static FLASHMEM void recover_proof_apply_degraded_science_hold(clock_science_row_t& row) {
  ocxo_science_row_suppress_for_recover_hold(row);
  g_recover_proof_degraded_science_suppressed_count++;
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
  // and science consumers inert while recovery/reattach bookends settle.
  ocxo_science_row_suppress_for_recover_hold(ocxo1_science);
  ocxo_science_row_suppress_for_recover_hold(ocxo2_science);

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
  // Dead-producer RECOVER can resume a known public clock ledger before the Pi has
  // supplied or restored a human campaign label; do not leave the campaign
  // continuity watchdog disarmed merely because the label is empty.
  return watchdog_campaign_publication_armed &&
         !clocks_watchdog_publication_blocked() &&
         campaign_state == clocks_campaign_state_t::STARTED &&
         campaign_seconds != 0ULL &&
         !request_start &&
         !request_stop &&
         !request_recover &&
         !request_rearm &&
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
    // conservative for recovery science fields: legitimate Welford values
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
  clocks_alpha_smartzero_delay_snapshot_t& s =
      g_beta_report_smartzero_delay_scratch;
  s = clocks_alpha_smartzero_delay_snapshot_t{};
  const bool snapshot_ok = clocks_alpha_smartzero_delay_snapshot(&s);
  const bool available = snapshot_ok && s.install_count != 0U;

  p.add("smartzero_delay_snapshot_ok", snapshot_ok);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
  clocks_alpha_ocxo_visible_origin_snapshot_t& s = g_beta_report_visible_origin_scratch;
  s = clocks_alpha_ocxo_visible_origin_snapshot_t{};
  const bool snapshot_ok =
      clocks_alpha_ocxo_visible_origin_snapshot(clock, &s);
  const bool available = snapshot_ok && (s.valid || s.pending);

  Payload obj;
  obj.add("snapshot_ok", snapshot_ok);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
  Payload visible_origin;
  payload_add_visible_origin_snapshot(visible_origin, "ocxo1",
                                      time_clock_id_t::OCXO1);
  payload_add_visible_origin_snapshot(visible_origin, "ocxo2",
                                      time_clock_id_t::OCXO2);
  p.add_object("visible_origin", visible_origin);
}

static FLASHMEM void payload_add_smartzero_summary(Payload& p) {
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
  interrupt_smartzero_snapshot_t& live = g_beta_report_live_smartzero_scratch;
  live = interrupt_smartzero_snapshot_t{};
  const bool live_snapshot_ok = interrupt_smartzero_live_snapshot(&live);
  p.add("live_smartzero_snapshot_ok", live_snapshot_ok);

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

  payload_add_prefixed_smartzero_compact(
      p, "live_smartzero", live, live_snapshot_ok);
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
  payload_add_smartzero_snapshot_object(
      p, "live_smartzero", live, live_snapshot_ok, true);

  Payload live_lanes;
  payload_add_smartzero_lane(live_lanes, "vclock", live.lanes[0]);
  payload_add_smartzero_lane(live_lanes, "ocxo1", live.lanes[1]);
  payload_add_smartzero_lane(live_lanes, "ocxo2", live.lanes[2]);
  p.add_object("smartzero", live_lanes);
}


static FLASHMEM void prediction_snapshot_for_pps(
    clocks_static_prediction_snapshot_t& out) {
  out = clocks_static_prediction_snapshot_t{};
  if (!clocks_static_prediction_pps_snapshot(&out)) {
    out = clocks_static_prediction_snapshot_t{};
  }
}

static FLASHMEM void prediction_snapshot_for_clock(
    time_clock_id_t clock,
    clocks_static_prediction_snapshot_t& out) {
  out = clocks_static_prediction_snapshot_t{};
  if (!clocks_static_prediction_snapshot(clock, &out)) {
    out = clocks_static_prediction_snapshot_t{};
  }
}


// ============================================================================
// Command-report statistics serializers and typed CLOCKS snapshots
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

// Report-only serializer: reusable Payload control blocks live in RAM1. The outer
// COMMAND custody excludes priority-16 TimePop/handoff entry for the complete
// response transaction; typed CLOCKS snapshots use independent RAM2 value state.
static FLASHMEM void report_add_welford_object(Payload& parent,
                                               const char* key,
                                               const welford_t& w) {
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
  Payload& stats = g_report_child_stats;
  stats.clear();
  stats.add("schema", "CLOCKS_INSTRUMENT_STATS_V2");
  stats.add("always_on", true);
  stats.add("owner", "ALPHA");
  stats.add("lifetime", "BOOT_TO_REBOOT_OR_STATS_RESET");
  stats.add("snapshot_ok", instrument.snapshot_ok);
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


  p.add_object("stats", stats);
  stats.clear();
}

static FLASHMEM clocks_fragment_welford_snapshot_t clocks_fragment_welford_snapshot(
    const welford_t& w) {
  clocks_fragment_welford_snapshot_t out{};
  out.n = w.n;
  out.mean = w.mean;
  out.m2 = w.m2;
  out.stddev = welford_stddev(w);
  out.stderr_value = welford_stderr(w);
  out.min = (w.n > 0U) ? w.min_val : 0.0;
  out.max = (w.n > 0U) ? w.max_val : 0.0;
  return out;
}

static FLASHMEM clocks_fragment_stats_clock_snapshot_t clocks_fragment_stats_clock(
    const welford_t& w,
    bool frequency_present,
    double ppb) {
  clocks_fragment_stats_clock_snapshot_t out{};
  out.welford = clocks_fragment_welford_snapshot(w);
  out.frequency_present = frequency_present;
  out.ppb = frequency_present ? ppb : 0.0;
  out.tau = frequency_present ? (1.0 + ppb / 1.0e9) : 1.0;
  return out;
}

static FLASHMEM clocks_fragment_ppb_value_snapshot_t clocks_fragment_ppb_value(
    const clocks_instrument_ppb_value_snapshot_t& source) {
  clocks_fragment_ppb_value_snapshot_t out{};
  out.sample_count = source.sample_count;
  out.ppb = source.sample_count != 0ULL ? source.ppb : 0.0;
  return out;
}

static FLASHMEM clocks_fragment_ppb_buckets_snapshot_t clocks_fragment_ppb_buckets(
    const clocks_instrument_ppb_buckets_snapshot_t& source) {
  clocks_fragment_ppb_buckets_snapshot_t out{};
  out.minute_10 = clocks_fragment_ppb_value(source.minute_10);
  out.minute_60 = clocks_fragment_ppb_value(source.minute_60);
  out.hour_8 = clocks_fragment_ppb_value(source.hour_8);
  out.hour_24 = clocks_fragment_ppb_value(source.hour_24);
  out.total = clocks_fragment_ppb_value(source.total);
  return out;
}

static void clocks_fragment_campaign_ppb_set(
    clocks_fragment_ppb_value_snapshot_t& out,
    uint64_t sample_count,
    double ppb) {
  out.sample_count = sample_count;
  out.ppb = sample_count != 0ULL ? ppb : 0.0;
}

static double clocks_fragment_campaign_dwt_ppb(uint64_t gnss_ns,
                                               uint64_t dwt_cycles) {
  const double expected_cycles =
      ((double)gnss_ns * (double)DWT_EXPECTED_PER_PPS) / 1.0e9;
  return ((double)dwt_cycles / expected_cycles - 1.0) * 1.0e9;
}

static void clocks_fragment_campaign_stats_snapshot(
    clocks_fragment_campaign_stats_snapshot_t& stats,
    uint64_t sample_count,
    uint64_t gnss_ns,
    uint64_t dwt_cycles,
    uint64_t ocxo1_ns,
    uint64_t ocxo2_ns) {
  stats = clocks_fragment_campaign_stats_snapshot_t{};
  if (sample_count == 0ULL || gnss_ns == 0ULL) return;

  clocks_fragment_campaign_ppb_set(stats.gnss, sample_count, 0.0);
  clocks_fragment_campaign_ppb_set(stats.vclock, sample_count, 0.0);

  if (dwt_cycles != 0ULL) {
    clocks_fragment_campaign_ppb_set(
        stats.dwt,
        sample_count,
        clocks_fragment_campaign_dwt_ppb(gnss_ns, dwt_cycles));
  }
  if (ocxo1_ns != 0ULL) {
    clocks_fragment_campaign_ppb_set(
        stats.ocxo1,
        sample_count,
        campaign_total_ppb_from_tau(
            campaign_total_tau_from_ratio(gnss_ns, ocxo1_ns)));
  }
  if (ocxo2_ns != 0ULL) {
    clocks_fragment_campaign_ppb_set(
        stats.ocxo2,
        sample_count,
        campaign_total_ppb_from_tau(
            campaign_total_tau_from_ratio(gnss_ns, ocxo2_ns)));
  }
}

static FLASHMEM clocks_fragment_tau_recovery_snapshot_t
clocks_fragment_tau_recovery_snapshot(
    const clocks_alpha_tau_snapshot_t& state) {
  clocks_fragment_tau_recovery_snapshot_t out{};
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

static void clocks_fragment_ppb_endpoint_from_alpha(
    clocks_fragment_ppb_endpoint_snapshot_t& out,
    const clocks_alpha_ppb_cumulative_endpoint_snapshot_t& source) {
  out = clocks_fragment_ppb_endpoint_snapshot_t{};
  out.reference_ns = source.reference_ns;
  out.dwt_error_cycles = source.dwt_error_cycles;
  out.ocxo1_error_ns = source.ocxo1_error_ns;
  out.ocxo2_error_ns = source.ocxo2_error_ns;
  out.rolling_sequence = source.rolling_sequence;
  out.interval_count = source.interval_count;
}

static void clocks_fragment_ppb_window_proof_from_alpha(
    clocks_fragment_ppb_window_proof_snapshot_t& out,
    const clocks_alpha_ppb_window_proof_snapshot_t& source) {
  out = clocks_fragment_ppb_window_proof_snapshot_t{};
  out.valid = source.valid;
  out.sample_count = source.sample_count;
  if (source.valid) {
    clocks_fragment_ppb_endpoint_from_alpha(out.anchor, source.anchor);
  }
}

static void clocks_fragment_ppb_checkpoint_from_alpha(
    clocks_fragment_ppb_checkpoint_delta_snapshot_t& out,
    const clocks_alpha_ppb_checkpoint_delta_snapshot_t& source) {
  out = clocks_fragment_ppb_checkpoint_delta_snapshot_t{};
  out.valid = source.valid;
  out.rolling_sequence = source.rolling_sequence;
  out.second_count = source.second_count;
  out.minute_count = source.minute_count;
  out.last_minute_key = source.last_minute_key;
  out.origin_valid = source.origin_valid;

  clocks_fragment_ppb_endpoint_from_alpha(out.current, source.current);
  if (source.origin_valid) {
    clocks_fragment_ppb_endpoint_from_alpha(out.origin, source.origin);
  }

  clocks_fragment_ppb_window_proof_from_alpha(out.minute_10, source.minute_10);
  clocks_fragment_ppb_window_proof_from_alpha(out.minute_60, source.minute_60);
  clocks_fragment_ppb_window_proof_from_alpha(out.hour_8, source.hour_8);
  clocks_fragment_ppb_window_proof_from_alpha(out.hour_24, source.hour_24);

  out.second_append_valid = source.second_append_valid;
  if (source.second_append_valid) {
    clocks_fragment_ppb_endpoint_from_alpha(
        out.second_append, source.second_append);
  }
  out.minute_append_valid = source.minute_append_valid;
  if (source.minute_append_valid) {
    clocks_fragment_ppb_endpoint_from_alpha(
        out.minute_append, source.minute_append);
  }
}

static FLASHMEM void clocks_fragment_stats_snapshot_from_instrument(
    clocks_fragment_stats_snapshot_t& out,
    const clocks_instrument_stats_snapshot_t& instrument) {
  out = clocks_fragment_stats_snapshot_t{};
  out.snapshot_ok = instrument.snapshot_ok;
  out.valid = instrument.valid;
  out.reset_count = instrument.reset_count;
  out.update_count = instrument.update_count;
  out.last_pps_sequence = instrument.last_pps_sequence;
  out.rolling_ppb_current_sequence = instrument.rolling_ppb_current_sequence;
  out.rolling_ppb_endpoint_admitted =
      instrument.rolling_ppb_endpoint_admitted;
  out.rolling_ppb_interval_advanced =
      instrument.rolling_ppb_interval_advanced;
  clocks_fragment_ppb_checkpoint_from_alpha(
      out.rolling_ppb_checkpoint, instrument.rolling_ppb_checkpoint);
  out.completed_row_coherent = instrument.completed_row_coherent;

  out.gnss = clocks_fragment_stats_clock(instrument.gnss_welford, true, 0.0);
  out.dwt = clocks_fragment_stats_clock(
      instrument.dwt_welford,
      instrument.dwt_frequency.valid,
      instrument.dwt_frequency.ppb);
  out.vclock = clocks_fragment_stats_clock(
      instrument.vclock_welford,
      instrument.vclock_frequency.valid,
      instrument.vclock_frequency.ppb);
  out.ocxo1 = clocks_fragment_stats_clock(
      instrument.ocxo1_welford,
      instrument.ocxo1_frequency.valid,
      instrument.ocxo1_frequency.ppb);
  out.ocxo2 = clocks_fragment_stats_clock(
      instrument.ocxo2_welford,
      instrument.ocxo2_frequency.valid,
      instrument.ocxo2_frequency.ppb);
  out.pps_witness = clocks_fragment_stats_clock(
      instrument.pps_witness_welford, false, 0.0);

  out.dwt.ppb_buckets = clocks_fragment_ppb_buckets(
      instrument.dwt_frequency.ppb_buckets);
  out.vclock.ppb_buckets = clocks_fragment_ppb_buckets(
      instrument.vclock_frequency.ppb_buckets);
  out.ocxo1.ppb_buckets = clocks_fragment_ppb_buckets(
      instrument.ocxo1_frequency.ppb_buckets);
  out.ocxo2.ppb_buckets = clocks_fragment_ppb_buckets(
      instrument.ocxo2_frequency.ppb_buckets);
  if (instrument.gnss_welford.n != 0ULL) {
    out.gnss.ppb_buckets.total.sample_count = instrument.gnss_welford.n;
    out.gnss.ppb_buckets.total.ppb = 0.0;
  }
  out.ocxo1_tau_state = clocks_fragment_tau_recovery_snapshot(
      instrument.ocxo1_tau_state);
  out.ocxo2_tau_state = clocks_fragment_tau_recovery_snapshot(
      instrument.ocxo2_tau_state);

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
  g_delta_clock_candidate_ocxo1 = delta_clock_candidate_state_t{};
  g_delta_clock_candidate_ocxo2 = delta_clock_candidate_state_t{};

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
  g_beta_clocks_fragment_instrument_stats = clocks_instrument_stats_snapshot_t{};
  g_clocks_restore_state = clocks_recovery_restore_state_t{};
  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  g_clocks_report_build_active = false;
  g_clocks_report_build_count = 0U;
  g_clocks_report_busy_reject_count = 0U;
  g_clocks_report_max_duration_cycles = 0U;

  g_recover_proof_last_ocxo1 =
      clocks_alpha_recover_proof_snapshot_t{};
  g_recover_proof_last_ocxo2 =
      clocks_alpha_recover_proof_snapshot_t{};

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
  if (!clocks_ocxo_counterledger_mode()) return;

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
         ledger.phase_valid &&
         ledger.phase_implied_counter32_at_pps == ledger.last_counter32 &&
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
  if (g_recover_proof_degraded_active) return 0U;
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

  // Preserve the exact private-PPS0 DWT geometry.  Public PPS1 can then form a
  // fully independent, window-aligned Delta increment rather than re-anchoring
  // from the PhaseLedger clockface.
  const uint32_t private_target_dwt = (uint32_t)g_dwt_at_pps_vclock;
  if (!delta_clock_candidate_capture_geometry(
          g_delta_clock_candidate_ocxo1,
          private_target_dwt,
          ocxo1_f.last_event_dwt,
          ocxo1_f.dwt_cycles_between_edges) ||
      !delta_clock_candidate_capture_geometry(
          g_delta_clock_candidate_ocxo2,
          private_target_dwt,
          ocxo2_f.last_event_dwt,
          ocxo2_f.dwt_cycles_between_edges)) {
    campaign_start_prologue_set_reason(
        "private_candidate_missing_delta_geometry");
    return false;
  }

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
  request_rearm = false;
  request_zero = false;
  flash_cut_clear_pending();

  // A failed START closes only the requested recording namespace. Always-on
  // instrument measurement/statistical state survives the campaign admission
  // failure.
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
          clocks_ocxo_counterledger_mode()
              ? "phaseledger_start_handoff_timeout"
              : "start_handoff_timeout");
      return true;
    }
    campaign_start_prologue_set_reason(
        clocks_ocxo_counterledger_mode()
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
        clocks_ocxo_counterledger_mode()
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
          ? (clocks_ocxo_counterledger_mode()
                 ? "phaseledger_private_pps0_refreshed_after_probe_reject"
                 : "private_pps0_refreshed_after_probe_reject")
          : (clocks_ocxo_counterledger_mode()
                 ? "phaseledger_private_pps0_seeded"
                 : "private_pps0_seeded"));
  return true;
}


static FLASHMEM void clocks_fragment_copy_text(char* dst,
                                     size_t capacity,
                                     const char* src) {
  if (!dst || capacity == 0U) return;
  safeCopy(dst, capacity, src ? src : "");
}

static FLASHMEM void clocks_fragment_clock_candidates_snapshot_from_row(
    clocks_fragment_clock_candidates_snapshot_t& out,
    const delta_clock_candidate_state_t& delta_state,
    uint32_t public_count,
    uint64_t phaseledger_ns,
    bool phaseledger_clockface_valid,
    const clocks_alpha_ocxo_counterledger_snapshot_t& phaseledger,
    const clock_science_row_t& delta_row) {
  out = clocks_fragment_clock_candidates_snapshot_t{};
  clocks_fragment_copy_text(out.published_source,
                           sizeof(out.published_source),
                           "COUNTERLEDGER_PHASELEDGER");

  out.phaseledger.available = phaseledger_clockface_valid;
  out.phaseledger.continuity_valid = phaseledger_clockface_valid;
  out.phaseledger.status = phaseledger_clockface_valid
      ? clocks_fragment_clock_candidate_status_t::ADVANCED
      : clocks_fragment_clock_candidate_status_t::UNAVAILABLE;
  out.phaseledger.start_public_count = delta_state.start_public_count;
  out.phaseledger.last_public_count = public_count;
  out.phaseledger.interval_count =
      public_count >= delta_state.start_public_count
          ? public_count - delta_state.start_public_count
          : 0U;
  out.phaseledger.ns = phaseledger_ns;
  out.phaseledger.residual_available =
      phaseledger_clockface_valid && phaseledger.refined_interval_valid;
  out.phaseledger.residual_ns = out.phaseledger.residual_available
      ? phaseledger.refined_fast_residual_ns
      : 0LL;
  out.phaseledger.residual_ns_exact =
      (double)out.phaseledger.residual_ns;

  out.delta_cycles.available =
      delta_state.seeded && delta_state.continuity_valid &&
      delta_state.last_public_count == public_count;
  out.delta_cycles.continuity_valid = delta_state.continuity_valid;
  out.delta_cycles.status = delta_state.status;
  out.delta_cycles.start_public_count = delta_state.start_public_count;
  out.delta_cycles.last_public_count = delta_state.last_public_count;
  out.delta_cycles.interval_count = delta_state.interval_count;
  out.delta_cycles.ns = delta_state.ns;
  out.delta_cycles.fractional_ns = delta_state.fractional_ns;
  // Candidate residuals describe the same selected-VCLOCK endpoint window as
  // the accumulated Delta clock.  The single full-OCXO-interval rate witness
  // remains separately available under ocxoN.science.delta_raw_*.
  out.delta_cycles.residual_available =
      delta_state.last_residual_available &&
      delta_state.last_public_count == public_count;
  out.delta_cycles.residual_ns = out.delta_cycles.residual_available
      ? delta_state.last_residual_ns
      : 0LL;
  out.delta_cycles.residual_ns_exact = out.delta_cycles.residual_available
      ? delta_state.last_residual_ns_exact
      : 0.0;
  (void)delta_row;

  out.comparable =
      out.phaseledger.available && out.delta_cycles.available;
  out.delta_cycles_minus_phaseledger_ns = out.comparable
      ? beta_signed_delta_u64(out.delta_cycles.ns, out.phaseledger.ns)
      : 0LL;
  out.residuals_comparable =
      out.phaseledger.residual_available &&
      out.delta_cycles.residual_available;
  out.delta_cycles_minus_phaseledger_residual_ns_exact =
      out.residuals_comparable
          ? out.delta_cycles.residual_ns_exact -
                out.phaseledger.residual_ns_exact
          : 0.0;
}

static FLASHMEM void clocks_fragment_science_snapshot_from_row(
    clocks_fragment_science_snapshot_t& out,
    const clock_science_row_t& row) {
  out = static_cast<const clocks_fragment_science_snapshot_t&>(row);
}

static FLASHMEM void clocks_fragment_raw_cycles_lane_snapshot(
    clocks_fragment_raw_cycles_lane_t& out,
    const clocks_static_prediction_snapshot_t& sample,
    bool forensics_snapshot_ok,
    const interrupt_delay_forensics_t& delay) {
  out = clocks_fragment_raw_cycles_lane_t{};
  out.snapshot_ok = sample.snapshot_ok;
  out.forensics_snapshot_ok = forensics_snapshot_ok;
  out.valid = sample.snapshot_ok && sample.valid;
  out.completed_interval_count = sample.completed_interval_count;
  out.observed_cycles = sample.actual_cycles;
  out.previous_observed_cycles = sample.static_prediction_cycles;
  out.residual_cycles = sample.static_residual_cycles;

  if (!forensics_snapshot_ok) {
    clocks_fragment_copy_text(out.delay_status,
                             sizeof(out.delay_status),
                             "SNAPSHOT_UNAVAILABLE");
    out.delay_detail_present = false;
    return;
  }

  const char* delay_status = interrupt_delay_verdict_str(delay.verdict);
  clocks_fragment_copy_text(out.delay_status,
                           sizeof(out.delay_status),
                           delay_status);
  out.delay_detail_present = true;

  out.residual_delay_valid = delay.residual_delay_valid && out.valid;
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

  clocks_fragment_copy_text(
      out.delay_by,
      sizeof(out.delay_by),
      (delay.valid && delay.delayed)
          ? interrupt_delay_cause_str(delay.delayed_by)
          : "NONE");
  out.residual_delay_cycles = out.residual_delay_valid
      ? delay.residual_delay_cycles
      : 0;
  clocks_fragment_copy_text(
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

static FLASHMEM void clocks_fragment_raw_cycles_snapshot(
    clocks_fragment_raw_cycles_snapshot_t& out,
    const clocks_static_prediction_snapshot_t& pps,
    const clocks_static_prediction_snapshot_t& vclock,
    const clocks_static_prediction_snapshot_t& ocxo1,
    const clocks_static_prediction_snapshot_t& ocxo2,
    const interrupt_delay_forensics_t& pps_delay,
    const clocks_alpha_lane_forensics_t& vclock_forensics,
    const clocks_alpha_lane_forensics_t& ocxo1_forensics,
    const clocks_alpha_lane_forensics_t& ocxo2_forensics) {
  out = clocks_fragment_raw_cycles_snapshot_t{};
  clocks_fragment_raw_cycles_lane_snapshot(out.pps, pps, true, pps_delay);
  clocks_fragment_raw_cycles_lane_snapshot(
      out.vclock, vclock, vclock_forensics.snapshot_ok,
      vclock_forensics.interrupt_delay);
  clocks_fragment_raw_cycles_lane_snapshot(
      out.ocxo1, ocxo1, ocxo1_forensics.snapshot_ok,
      ocxo1_forensics.interrupt_delay);
  clocks_fragment_raw_cycles_lane_snapshot(
      out.ocxo2, ocxo2, ocxo2_forensics.snapshot_ok,
      ocxo2_forensics.interrupt_delay);
}


// ============================================================================
// Zeroing
// ============================================================================

static void campaign_accounting_reset_common() {
  clocks_watchdog_disarm_campaign_publication();
  clocks_row_objection_clear();

  // Beta-local recording/accounting reset only. Instrument statistics survive
  // every campaign boundary.
  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_measured_gnss_ticks_64 = 0;
  ocxo2_measured_gnss_ticks_64 = 0;

  campaign_public_offsets_reset_to_current();
  pps_interval_residuals_reset();

}


void clocks_zero_all(void) {
  campaign_accounting_reset_common();
}

static void campaign_flash_cut_accounting_reset(void) {
  campaign_accounting_reset_common();
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
         request_recover || request_rearm || request_zero ||
         g_clocks_restore_requested ||
         interrupt_smartzero_running() ||
         clocks_alpha_epoch_install_in_progress();
}

static void campaign_flash_cut_commit_at_pps(void) {
  safeCopy(g_flash_cut_last_from_campaign,
           sizeof(g_flash_cut_last_from_campaign),
           campaign_name);

  g_flash_cut_last_raw_gnss_ns = current_raw_gnss_ns();
  g_flash_cut_last_raw_dwt_cycles = g_dwt_cycle_count_total;
  g_flash_cut_last_raw_ocxo1_ns = current_raw_ocxo_ns(time_clock_id_t::OCXO1);
  g_flash_cut_last_raw_ocxo2_ns = current_raw_ocxo_ns(time_clock_id_t::OCXO2);
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

  // Continuity surrender stops campaign publication while preserving the
  // always-on instrument measurement state. The watchdog gate prevents suspect
  // rows from feeding scientific accumulation until recovery clears custody.
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

  if (first) {
    clocks_fragment_watchdog_reset_publication_custody();
  }

  // The publication/watchdog gate holds further campaign science until the
  // lifecycle is recovered.
  return first;
}

static void clocks_watchdog_anomaly_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  const clocks_payload_custody_t payload_custody(clocks_payload_owner_t::EVENT);
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
  request_rearm = false;
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


static FLASHMEM void clocks_fragment_refresh_prediction_snapshots(void) {
  prediction_snapshot_for_pps(g_beta_clocks_fragment_pps_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::VCLOCK, g_beta_clocks_fragment_vclock_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::OCXO1, g_beta_clocks_fragment_ocxo1_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::OCXO2, g_beta_clocks_fragment_ocxo2_prediction);

  g_beta_clocks_fragment_vclock_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_clocks_fragment_ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  g_beta_clocks_fragment_ocxo2_forensics = clocks_alpha_lane_forensics_t{};
  if (!clocks_alpha_lane_forensics(
          time_clock_id_t::VCLOCK, &g_beta_clocks_fragment_vclock_forensics)) {
    g_beta_clocks_fragment_vclock_forensics = clocks_alpha_lane_forensics_t{};
  }
  if (!clocks_alpha_lane_forensics(
          time_clock_id_t::OCXO1, &g_beta_clocks_fragment_ocxo1_forensics)) {
    g_beta_clocks_fragment_ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  }
  if (!clocks_alpha_lane_forensics(
          time_clock_id_t::OCXO2, &g_beta_clocks_fragment_ocxo2_forensics)) {
    g_beta_clocks_fragment_ocxo2_forensics = clocks_alpha_lane_forensics_t{};
  }
}

static FLASHMEM void clocks_fragment_live_snapshot_fill(
    clocks_fragment_live_snapshot_t& out) {
  out = clocks_fragment_live_snapshot_t{};
  g_beta_clocks_fragment_instrument_stats = clocks_instrument_stats_snapshot_t{};
  out.snapshot_ok = clocks_alpha_instrument_stats_snapshot(
      &g_beta_clocks_fragment_instrument_stats);
  const clocks_instrument_stats_snapshot_t& instrument =
      g_beta_clocks_fragment_instrument_stats;

  const uint64_t logical_instrument_gnss_ns =
      instrument_continuity_gnss_ns(instrument.gnss_ns);
  const uint64_t logical_instrument_dwt_cycles =
      instrument_continuity_dwt_cycles(instrument.dwt_cycles);
  const uint64_t logical_instrument_ocxo1_ns =
      g_instrument_continuity_active
          ? campaign_public_from_offset(instrument.ocxo1_ns, g_instrument_ocxo1_offset)
          : instrument.ocxo1_ns;
  const uint64_t logical_instrument_ocxo2_ns =
      g_instrument_continuity_active
          ? campaign_public_from_offset(instrument.ocxo2_ns, g_instrument_ocxo2_offset)
          : instrument.ocxo2_ns;

  out.valid = instrument.valid;
  out.completed_row_coherent = instrument.completed_row_coherent;
  out.completed_pps_sequence = instrument.last_pps_sequence;
  out.instrument_age_seconds =
      (uint32_t)(logical_instrument_gnss_ns / CLOCKS_BETA_NS_PER_SECOND);

  out.instrument_gnss_ns = logical_instrument_gnss_ns;
  out.instrument_dwt_cycles = logical_instrument_dwt_cycles;
  out.instrument_ocxo1_ns = logical_instrument_ocxo1_ns;
  out.instrument_ocxo2_ns = logical_instrument_ocxo2_ns;
  out.instrument_pps_sequence = instrument.last_pps_sequence;

  out.dwt_cycles_per_second = instrument.dwt_cycles_per_second;
  out.dwt_at_pps_vclock = instrument.dwt_at_pps_vclock;
  out.counter32_at_pps_vclock = instrument.counter32_at_pps_vclock;

  clocks_fragment_refresh_prediction_snapshots();
  clocks_fragment_raw_cycles_snapshot(
      out.raw_cycles,
      g_beta_clocks_fragment_pps_prediction,
      g_beta_clocks_fragment_vclock_prediction,
      g_beta_clocks_fragment_ocxo1_prediction,
      g_beta_clocks_fragment_ocxo2_prediction,
      g_beta_clocks_fragment_vclock_forensics.interrupt_delay,
      g_beta_clocks_fragment_vclock_forensics,
      g_beta_clocks_fragment_ocxo1_forensics,
      g_beta_clocks_fragment_ocxo2_forensics);
  clocks_fragment_stats_snapshot_from_instrument(out.stats, instrument);
}

FLASHMEM bool clocks_fragment_snapshot_take(
    uint32_t completed_second_sequence,
    clocks_fragment_snapshot_t* out) {
  if (!out) return false;
  *out = clocks_fragment_snapshot_t{};
  clocks_fragment_live_snapshot_fill(out->live);

  // Publication-custody metadata only. Once public campaign time is advancing,
  // CLOCKS must wait for the exact matching Beta delta before releasing the
  // physical second. Dead-producer RECOVER does not create a campaign-less live
  // boundary: producer and campaign desired state are installed as one transaction.
  out->campaign_row_expected =
      campaign_state != clocks_campaign_state_t::STOPPED &&
      campaign_name[0] != '\0' &&
      campaign_seconds > 0ULL;

  clocks_fragment_campaign_snapshot_t* campaign_front =
      clocks_fragment_campaign_queue_front();
  if (campaign_front &&
      campaign_front->completed_second_sequence == completed_second_sequence) {
    out->campaign = *campaign_front;
    clocks_fragment_campaign_queue_release();
    g_clocks_fragment_campaign_record_take_count++;
    clocks_fragment_campaign_record_ready_retry_cancel();
  } else if (campaign_front &&
             (int32_t)(completed_second_sequence -
                       campaign_front->completed_second_sequence) > 0) {
    // A newer physical row may never step past older unconsumed campaign
    // testimony.  That would be silent custody loss, not coalescing.
    __builtin_trap();
  }

  return true;
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
  clocks_beta_pps_owner_guard_t operation_owner{};
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_ENTRY);
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_ENTRY);

  // Alpha enters Beta only after both post-PPS OCXO lanes have frozen this exact
  // completed row. Wake CLOCKS' PPS-triggered serializer now that the requested
  // identity is genuinely available. Campaign rows may still hold publication
  // below until their matching enrichment is frozen.
  if (completed_pps_sequence != 0U) {
    clocks_fragment_completed_row_ready(completed_pps_sequence);
  }

  if (request_stop) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_STOP_GATE);
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    clocks_row_objection_clear();
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    flash_cut_clear_pending();
    // STOP closes the recording namespace only.
    campaign_warmup_reset();
    return;
  }

  if (request_start) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_START_ZERO_GATE);
    clocks_finish_start_accounting();
    return;
  }

  if (request_zero) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_START_ZERO_GATE);
    (void)clocks_try_finish_pending_smartzero();
    return;
  }

  if (request_flash_cut) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_FLASH_CUT_GATE);
    campaign_flash_cut_commit_at_pps();
    return;
  }

  if (request_rearm) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_REARM_GATE);
    clocks_watchdog_disarm_campaign_publication();
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    clocks_row_objection_clear();

    if (campaign_state != clocks_campaign_state_t::STOPPED ||
        recover_gnss_ns == 0ULL || recover_dwt_ns == 0ULL ||
        recover_ocxo1_ns == 0ULL || recover_ocxo2_ns == 0ULL ||
        (recover_gnss_ns % CLOCKS_BETA_NS_PER_SECOND) != 0ULL) {
      __builtin_trap();
    }

    // The durable campaign remains active on the Pi.  This boundary restores
    // only Beta's recording coordinate on the already-proved live Alpha.  The
    // boundary row itself is not a campaign row; the next completed PPS is the
    // exact projected public N+1.
    clocks_fragment_recover_reset_publication_custody(false);
    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64 = recover_gnss_ns / 100ULL;
    ocxo1_measured_gnss_ticks_64 = recover_ocxo1_ns / 100ULL;
    ocxo2_measured_gnss_ticks_64 = recover_ocxo2_ns / 100ULL;
    campaign_seconds = recover_gnss_ns / CLOCKS_BETA_NS_PER_SECOND;

    pps_interval_residuals_reset();
    campaign_public_offsets_reset_for_projected_resume();
    recover_proof_reset("surviving_alpha_campaign_rearm");

    request_rearm = false;
    request_start = false;
    request_stop = false;
    request_recover = false;
    request_zero = false;
    flash_cut_clear_pending();
    campaign_state = clocks_campaign_state_t::STARTED;
    g_rearm_commit_count++;
    safeCopy(g_rearm_last_status, sizeof(g_rearm_last_status),
             "rearm_committed");
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
            recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE &&
        !clocks_alpha_installed_smartzero_backing_epoch()) {
      g_recover_lifecycle_dead_producer_restore_wait_count++;
      recover_lifecycle_set_reason("recover_dead_producer_restore_wait_smartzero");
      return;
    }

    if (g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE &&
        !g_recover_lifecycle_dead_producer_restore_epoch_ready) {
      g_recover_lifecycle_dead_producer_restore_epoch_ready = true;
      g_recover_lifecycle_dead_producer_restore_ready_count++;
      recover_lifecycle_set_reason("recover_dead_producer_restore_epoch_ready");
    }

    // SmartZero has already created and physically staggered the fresh OCXO
    // service grids for this firmware generation. Starting a second RECOVER-owned
    // rephase here would create a circular wait; splice the durable state onto the
    // installed newborn epoch directly.
    recover_lifecycle_set_reason("recover_dead_producer_restore_commit_ready");

    if (g_campaign_restore_state.valid) {
      if (!clocks_recovery_commit_statistics_and_clockfaces(
              g_campaign_restore_state)) {
        g_campaign_restore_failure_count++;
        clocks_ppb_restore_protocol_clear(true);
        recover_lifecycle_abort("recover_state_commit_failed");
        return;
      }
      g_campaign_restore_count++;
      clocks_ppb_restore_protocol_clear(false);
    }

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_measured_gnss_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_measured_gnss_ticks_64        = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    // Rebase only Beta's campaign presentation onto the genuinely new Alpha epoch.
    campaign_public_offsets_reset_for_projected_resume();
    interrupt_recover_reset_publication_custody();
    g_recover_lifecycle_gate_custody_reset_count++;
    // Cut Beta's interval totals at the same custody boundary.  No fixed row
    // quarantine is armed; the pre-Alpha proof court releases science when both
    // lanes establish a fresh lawful ancestry chain.
    pps_interval_residuals_reset();
    // Alpha-owned PPB/TAU/Welfords were restored from the durable desired state.

    request_recover = false;
    g_campaign_restore_state = clocks_recovery_restore_state_t{};
    flash_cut_clear_pending();
    g_recover_lifecycle_dead_producer_restore_commit_count++;
    recover_lifecycle_complete_at_pps();
    campaign_warmup_begin(campaign_warmup_mode_t::RECOVER);
    return;
  }

  if (g_clocks_restore_requested) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_RECOVER_GATE);
    if (!clocks_alpha_installed_smartzero_backing_epoch() ||
        clocks_alpha_epoch_install_in_progress()) {
      return;
    }

    if (!clocks_recovery_commit_statistics_and_clockfaces(
            g_clocks_restore_state)) {
      g_clocks_restore_requested = false;
      g_clocks_restore_failure_count++;
      clocks_ppb_restore_protocol_clear(true);
      return;
    }

    g_clocks_restore_requested = false;
    g_clocks_restore_commit_count++;
    g_clocks_restore_state = clocks_recovery_restore_state_t{};
    clocks_ppb_restore_protocol_clear(false);
    clocks_row_objection_clear();
    return;
  }

  if (clocks_campaign_recovery_lifecycle_active()) {
    g_recover_lifecycle_stale_gate_count++;
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_RECOVERING_NO_REQUEST_GATE);
    return;
  }

  if (clocks_watchdog_publication_blocked()) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_WATCHDOG_GATE);
    return;
  }
  if (campaign_state != clocks_campaign_state_t::STARTED) {
    // There is no campaign row to serialize while idle. Alpha already owns the
    // science-exclusion decision for its always-on statistics.
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_NOT_STARTED_GATE);
    clocks_row_objection_clear();
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
  const bool vclock_forensics_snapshot_ok =
      clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK,
                                  &vclock_forensics);
  const bool vclock_forensics_valid =
      vclock_forensics_snapshot_ok && vclock_forensics.valid;
  // Form the complete current-row forensic tuple before the START prologue
  // decides whether this candidate remains private PPS0 or becomes public PPS1.
  // The old ordering ran the warmup court first, so it could suppress the very
  // candidate needed to seed its observed bookends.
  clocks_alpha_lane_forensics_t& ocxo1_forensics = g_beta_pps_ocxo1_forensics;
  clocks_alpha_lane_forensics_t& ocxo2_forensics = g_beta_pps_ocxo2_forensics;
  ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  ocxo2_forensics = clocks_alpha_lane_forensics_t{};

  const bool ocxo1_forensics_snapshot_ok =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &ocxo1_forensics);
  const bool ocxo2_forensics_snapshot_ok =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &ocxo2_forensics);
  const bool ocxo1_forensics_valid =
      ocxo1_forensics_snapshot_ok && ocxo1_forensics.valid;
  const bool ocxo2_forensics_valid =
      ocxo2_forensics_snapshot_ok && ocxo2_forensics.valid;

  // The completed row is one immutable scientific identity.  Snapshot every
  // Alpha authority once, then prove that the PPS/VCLOCK bookend, both
  // PPS-synchronous CounterLedgers, both PhaseLedger suffixes, and both direct
  // adjacent OCXO edge intervals all belong to completed_pps_sequence.
  clocks_pps_vclock_edge_forensics_t& pps_vclock_edge_forensics =
      g_beta_pps_vclock_edge_forensics;
  pps_vclock_edge_forensics = clocks_pps_vclock_edge_forensics_t{};
  const bool pps_vclock_edge_forensics_snapshot_ok =
      clocks_alpha_pps_vclock_edge_forensics(&pps_vclock_edge_forensics);
  const bool pps_vclock_edge_forensics_valid =
      pps_vclock_edge_forensics_snapshot_ok &&
      pps_vclock_edge_forensics.valid;

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

  // RECOVER resumes an existing public timeline, so its proof convergence state is
  // observational here and candidate identities continue to be emitted.  Run
  // that court only after exact completed-row custody has been proven.
  (void)recover_proof_should_hold();

  // START consumes this exact candidate tuple privately while campaign_seconds
  // remains zero. The helper seeds PPS0 from the normal observed-edge inputs and
  // returns true until a following mature tuple can be released as public PPS1.
  // Welford, watchdog, payload construction, and publication remain below
  // this boundary and therefore cannot consume private startup evidence.
  if (campaign_warmup_consume_one_candidate_record(
          vclock_forensics_valid, vclock_forensics,
          ocxo1_forensics_valid, ocxo1_forensics,
          ocxo2_forensics_valid, ocxo2_forensics)) {
    clocks_row_objection_clear();
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_WARMUP_GATE);
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
  ocxo1_measured_gnss_ticks_64        = campaign_public_from_offset(
      current_raw_ocxo_ns(time_clock_id_t::OCXO1), g_campaign_public_ocxo1_offset) / 100ull;
  ocxo2_measured_gnss_ticks_64        = campaign_public_from_offset(
      current_raw_ocxo_ns(time_clock_id_t::OCXO2), g_campaign_public_ocxo2_offset) / 100ull;

  // Alpha already advanced the boot-lifetime instrument statistics for this
  // completed row before entering Beta.  The remaining work is campaign
  // presentation and candidate disposition.

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
  // Welford and stats, so authoritative fields tell one story.
  // The Alpha PPS-projection snapshots remain courtroom collateral.
  // public_gnss_ns/public_dwt_total and any RECOVER presentation transform
  // were captured before per-second consumers above.

  const uint64_t public_ocxo1_measured_ns = campaign_public_from_offset(
      current_raw_ocxo_measured_ns(time_clock_id_t::OCXO1),
      g_campaign_public_ocxo1_measured_offset);
  const uint64_t public_ocxo2_measured_ns = campaign_public_from_offset(
      current_raw_ocxo_measured_ns(time_clock_id_t::OCXO2),
      g_campaign_public_ocxo2_measured_offset);

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
  // the finite fresh OCXO ancestry timeout.  OCXO science/Welford input
  // stays inert until Alpha proves fresh post-recovery OCXO custody.
  const bool recover_degraded_science_hold =
      recover_proof_degraded_science_hold_active();

  const uint64_t public_ocxo1_counterledger_ns =
      campaign_public_counterledger_ns(
          time_clock_id_t::OCXO1, g_campaign_public_counterledger_ocxo1_offset);
  const uint64_t public_ocxo2_counterledger_ns =
      campaign_public_counterledger_ns(
          time_clock_id_t::OCXO2, g_campaign_public_counterledger_ocxo2_offset);
  const uint64_t public_ocxo1_ns = clocks_ocxo_counterledger_mode()
      ? public_ocxo1_counterledger_ns
      : science_render_public_clock_ns(
            public_gnss_ns, ocxo1_science, public_ocxo1_measured_ns);
  const uint64_t public_ocxo2_ns = clocks_ocxo_counterledger_mode()
      ? public_ocxo2_counterledger_ns
      : science_render_public_clock_ns(
            public_gnss_ns, ocxo2_science, public_ocxo2_measured_ns);

  const bool ocxo1_clockface_valid = clocks_ocxo_counterledger_mode()
      ? clocks_beta_counterledger_clockface_ready(ocxo1_counterledger,
                                                   completed_pps_sequence,
                                                   public_ocxo1_ns)
      : (public_ocxo1_ns != 0ULL && ocxo1_pps_projected_valid);
  const bool ocxo2_clockface_valid = clocks_ocxo_counterledger_mode()
      ? clocks_beta_counterledger_clockface_ready(ocxo2_counterledger,
                                                   completed_pps_sequence,
                                                   public_ocxo2_ns)
      : (public_ocxo2_ns != 0ULL && ocxo2_pps_projected_valid);

  // Advance the independent Delta-Cycles campaign clocks before any science
  // exclusion can suppress statistical/control mutation.  The candidate is
  // audit evidence; a missing Delta interval permanently breaks its continuity
  // until the next explicit campaign or RECOVER seed.
  delta_clock_candidate_advance(g_delta_clock_candidate_ocxo1,
                                public_count,
                                ocxo1_science,
                                ocxo1_forensics,
                                public_ocxo1_ns,
                                ocxo1_clockface_valid);
  delta_clock_candidate_advance(g_delta_clock_candidate_ocxo2,
                                public_count,
                                ocxo2_science,
                                ocxo2_forensics,
                                public_ocxo2_ns,
                                ocxo2_clockface_valid);

  clock_science_apply_counterledger_row(ocxo1_science,
                                        ocxo1_counterledger,
                                        public_gnss_ns);
  clock_science_apply_counterledger_row(ocxo2_science,
                                        ocxo2_counterledger,
                                        public_gnss_ns);

  // Published OCXO TAU/PPB are campaign clockface ratios, not physical-period
  // Delta ratios: public_ocxo_ns / public_gnss_ns.  RECOVER may have just
  // applied a one-time presentation transform so this ratio continues from the
  // Pi-projected campaign ledger instead of a fresh fresh-ancestry intercept.
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
    recover_proof_apply_degraded_science_hold(ocxo1_science);
    recover_proof_apply_degraded_science_hold(ocxo2_science);
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
      g_recover_proof_active ||
      g_recover_proof_degraded_active ||
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
  // coherent candidate becomes a durable campaign row; this court decides only
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
  }

  // Four local observed-interval snapshots feed raw_cycles.  The prior
  // completed interval is the prediction field; no duplicate prediction or
  // cycle-residual diagnostic object is serialized.
  prediction_snapshot_for_pps(g_beta_pps_cycle_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::VCLOCK, g_beta_vclock_cycle_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::OCXO1, g_beta_ocxo1_cycle_prediction);
  prediction_snapshot_for_clock(
      time_clock_id_t::OCXO2, g_beta_ocxo2_cycle_prediction);

  // Freeze one immutable completed campaign record for SYSTEM.  Beta owns the
  // lifecycle/science verdicts and values; SYSTEM alone decides how those facts
  // are named and nested in CLOCKS_FRAGMENT.
  g_campaign_record_last_public_count = public_count;
  g_campaign_record_last_public_gnss_ns = public_gnss_ns;
  g_campaign_record_last_public_dwt_total = public_dwt_total;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_BEGIN);
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_BUILD);

  // Producer owns one queue slot until commit.  Transport pressure may occupy
  // the other slot, but Beta never overwrites committed campaign testimony.
  clocks_fragment_campaign_snapshot_t* record_slot =
      clocks_fragment_campaign_queue_acquire_write();
  if (!record_slot) {
    campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_BACKLOG);
    g_clocks_fragment_campaign_record_backlog_count++;

    // Handoff saturation during a stable public campaign is observer loss, not
    // producer/science failure.  Do not overwrite either committed SPSC slot and
    // do not make the producer advance the consumer index.  Retire only this new
    // Pi observation at the producer boundary; Beta's campaign/public_count and
    // Alpha statistics continue.  The CLOCKS publisher will later retire older
    // undeliverable committed observations from the consumer side.
    if (clocks_watchdog_campaign_armed()) {
      clocks_fragment_campaign_note_observer_drop(
          completed_pps_sequence, public_count);
      return;
    }

    // Outside a stable STARTED campaign preserve the existing lifecycle court.
    // A recovery/start transition is not allowed to reinterpret structural
    // saturation as ordinary observer absence.
    clocks_watchdog_anomaly("clocks_fragment_campaign_record_backlog",
                            clocks_fragment_campaign_queue_front_sequence(),
                            completed_pps_sequence,
                            g_clocks_fragment_campaign_record_backlog_count,
                            public_count);
    return;
  }

  clocks_fragment_campaign_snapshot_t& record = *record_slot;
  record = clocks_fragment_campaign_snapshot_t{};
  record.present = true;
  record.completed_second_sequence = completed_pps_sequence;
  clocks_fragment_copy_text(record.campaign,
                           sizeof(record.campaign),
                           campaign_name);
  clocks_fragment_copy_text(record.campaign_state,
                           sizeof(record.campaign_state),
                           clocks_campaign_state_name(campaign_state));
  record.public_count = public_count;
  record.gnss_ns = public_gnss_ns;
  clocks_fragment_copy_text(record.disposition,
                           sizeof(record.disposition),
                           candidate_science_excluded
                               ? "SCIENCE_EXCLUDE"
                               : "ACCEPT");
  record.timeline_valid = recover_timeline_ready;
  record.ocxo_clockface_valid = recover_clockface_ready;
  record.ocxo_science_valid = recover_science_ready &&
                              !candidate_science_excluded;
  record.science_eligible = !candidate_science_excluded;
  record.control_eligible = !candidate_science_excluded;

  if (candidate_science_excluded) {
    record.rejection.present = true;
    record.rejection.reason_code = (uint32_t)candidate_objection.reason;
    clocks_fragment_copy_text(
        record.rejection.reason_name,
        sizeof(record.rejection.reason_name),
        clocks_row_objection_reason_name(candidate_objection.reason));
    clocks_fragment_copy_text(
        record.rejection.source,
        sizeof(record.rejection.source),
        clocks_row_objection_source_name(candidate_objection.source));
    record.rejection.lane_mask =
        ocxo_science_invalid_mask | candidate_objection.lane_mask;
  }

  // Recovery testimony remains conditional so ordinary steady-state records do
  // not carry an idle recovery transcript forever.
  const bool recovery_row =
      recover_transition_active ||
      g_recover_proof_active ||
      g_recover_proof_degraded_active ||
      recover_degraded_science_hold ||
      recover_science_quarantine_applied ||
      g_science_residual_quarantine_remaining != 0U ||
      g_recover_proof_stalled ||
      g_recover_continuity_align_last_public_count == public_count ||
      g_recover_proof_last_release_public_count == public_count;
  if (recovery_row) {
    record.recovery.present = true;
    record.recovery.generation = g_recover_request_count;
    record.recovery.transition_active = recover_transition_active;
    record.recovery.timeline_ready = recover_timeline_ready;
    record.recovery.clockface_ready = recover_clockface_ready;
    record.recovery.science_ready = recover_science_ready;
    record.recovery.science_quarantine_active =
        recover_science_quarantine_applied ||
        g_science_residual_quarantine_remaining != 0U;
    record.recovery.science_quarantine_remaining =
        g_science_residual_quarantine_remaining;
    record.recovery.degraded_active = g_recover_proof_degraded_active;
    record.recovery.proof_stalled = g_recover_proof_stalled;
  }

  record.dwt_cycles = public_dwt_total;
  record.ocxo1_ns = public_ocxo1_ns;
  clocks_fragment_clock_candidates_snapshot_from_row(
      record.ocxo1_clock_candidates,
      g_delta_clock_candidate_ocxo1,
      public_count,
      public_ocxo1_ns,
      ocxo1_clockface_valid,
      ocxo1_counterledger,
      ocxo1_science);
  clocks_fragment_science_snapshot_from_row(
      record.ocxo1_science, ocxo1_science);
  record.ocxo2_ns = public_ocxo2_ns;
  clocks_fragment_clock_candidates_snapshot_from_row(
      record.ocxo2_clock_candidates,
      g_delta_clock_candidate_ocxo2,
      public_count,
      public_ocxo2_ns,
      ocxo2_clockface_valid,
      ocxo2_counterledger,
      ocxo2_science);
  clocks_fragment_science_snapshot_from_row(
      record.ocxo2_science, ocxo2_science);

  clocks_fragment_campaign_stats_snapshot(
      record.stats,
      public_count,
      public_gnss_ns,
      public_dwt_total,
      public_ocxo1_ns,
      public_ocxo2_ns);

  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_PUBLISH);
  clocks_fragment_campaign_queue_commit_write();
  g_clocks_fragment_campaign_record_stage_count++;
  campaign_record_stage(CAMPAIGN_RECORD_STAGE_HANDOFF_READY);

  // The CLOCKS-owned physical tick normally already has this sequence pending. This
  // call coalesces the completed record into that publication, or schedules a
  // same-sequence copy if the observation-only fragment already escaped.
  clocks_fragment_campaign_row_ready(completed_pps_sequence);

  // A second notification 100 ms later is idempotent when CLOCKS consumed the
  // row normally. If CLOCKS' first ALAP arm was orphaned, the repeated signal
  // lets CLOCKS replace that stale arm before the next campaign candidate exists.
  clocks_fragment_campaign_record_ready_retry_arm();

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

static FLASHMEM Payload clocks_flash_cut_command_body(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    g_flash_cut_reject_count++;
    Payload err;
    err.add("error", "missing campaign");
    err.add("status", "flash_cut_rejected_missing_campaign");
    return err;
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
  return p;
}


static FLASHMEM Payload cmd_flash_cut(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  return clocks_flash_cut_command_body(args);
}

static FLASHMEM Payload cmd_start(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  if (g_ppb_restore_protocol_active) {
    Payload err;
    err.add("error", "Better-Buckets restore is staged");
    err.add("status", "start_rejected_ppb_restore_pending");
    return err;
  }
  if (g_clocks_restore_requested) {
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

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    return clocks_flash_cut_command_body(args);
  }

  if (request_rearm) {
    Payload err;
    err.add("error", "campaign rearm is active");
    err.add("status", "start_rejected_rearming");
    return err;
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
  return p;
}


static FLASHMEM Payload cmd_rearm(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_payload_numeric_integrity_reset();

  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    err.add("status", "rearm_rejected_missing_campaign");
    g_rearm_reject_count++;
    return err;
  }

  uint64_t dwt_ns = 0ULL;
  uint64_t gnss_ns = 0ULL;
  uint64_t ocxo1_ns = 0ULL;
  uint64_t ocxo2_ns = 0ULL;
  const clocks_payload_checked_status_t dwt_status =
      clocks_payload_try_get_u64_checked(args, "dwt_ns",
                                         "command_rearm_base", "args",
                                         "base", "dwt_ns", dwt_ns);
  const clocks_payload_checked_status_t gnss_status =
      clocks_payload_try_get_u64_checked(args, "gnss_ns",
                                         "command_rearm_base", "args",
                                         "base", "gnss_ns", gnss_ns);
  const clocks_payload_checked_status_t ocxo1_status =
      clocks_payload_try_get_u64_checked(args, "ocxo1_ns",
                                         "command_rearm_base", "args",
                                         "base", "ocxo1_ns", ocxo1_ns);
  const clocks_payload_checked_status_t ocxo2_status =
      clocks_payload_try_get_u64_checked(args, "ocxo2_ns",
                                         "command_rearm_base", "args",
                                         "base", "ocxo2_ns", ocxo2_ns);
  if (g_clocks_payload_numeric_integrity_failed) {
    g_rearm_reject_count++;
    return clocks_payload_numeric_reject_response(
        "rearm_rejected_numeric_integrity");
  }
  if (dwt_status != CLOCKS_PAYLOAD_FIELD_OK ||
      gnss_status != CLOCKS_PAYLOAD_FIELD_OK ||
      ocxo1_status != CLOCKS_PAYLOAD_FIELD_OK ||
      ocxo2_status != CLOCKS_PAYLOAD_FIELD_OK ||
      dwt_ns == 0ULL || gnss_ns == 0ULL ||
      ocxo1_ns == 0ULL || ocxo2_ns == 0ULL ||
      (gnss_ns % CLOCKS_BETA_NS_PER_SECOND) != 0ULL) {
    Payload err;
    err.add("error", "REARM requires positive projected dwt/gnss/ocxo clockfaces");
    err.add("status", "rearm_rejected_projection");
    g_rearm_reject_count++;
    return err;
  }

  const uint64_t base_count = gnss_ns / CLOCKS_BETA_NS_PER_SECOND;
  const uint64_t expected_first_public_count = base_count + 1ULL;
  const bool same_identity =
      g_rearm_request_count != 0U &&
      strcmp(name, g_rearm_last_campaign) == 0 &&
      base_count == g_rearm_last_base_count &&
      gnss_ns == g_rearm_last_base_gnss_ns &&
      dwt_ns == g_rearm_last_base_dwt_ns &&
      ocxo1_ns == g_rearm_last_base_ocxo1_ns &&
      ocxo2_ns == g_rearm_last_base_ocxo2_ns;

  if (request_rearm) {
    Payload p;
    p.add("status", same_identity
                        ? "rearm_already_active"
                        : "rearm_rejected_busy");
    p.add("idempotent", same_identity);
    p.add("campaign", campaign_name);
    p.add("requested_campaign", name);
    p.add("base_count", g_rearm_last_base_count);
    p.add("requested_base_count", base_count);
    p.add("expected_first_public_count",
          g_rearm_last_expected_first_public_count);
    if (!same_identity) {
      p.add("error", "different campaign rearm already active");
      g_rearm_reject_count++;
    }
    return p;
  }

  if (same_identity &&
      campaign_state == clocks_campaign_state_t::STARTED &&
      g_campaign_record_last_public_count >=
          g_rearm_last_expected_first_public_count) {
    Payload p;
    p.add("status", "rearm_already_completed");
    p.add("idempotent", true);
    p.add("campaign", campaign_name);
    p.add("base_count", g_rearm_last_base_count);
    p.add("expected_first_public_count",
          g_rearm_last_expected_first_public_count);
    p.add("last_public_count", g_campaign_record_last_public_count);
    p.add("alpha_mutated", false);
    return p;
  }

  if (campaign_state != clocks_campaign_state_t::STOPPED ||
      request_start || request_stop || request_recover || request_zero ||
      request_flash_cut || g_clocks_restore_requested ||
      g_ppb_restore_protocol_active ||
      clocks_campaign_recovery_lifecycle_active()) {
    Payload err;
    err.add("error", "campaign lifecycle is not idle for REARM");
    err.add("status", "rearm_rejected_busy");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    g_rearm_reject_count++;
    return err;
  }

  if (!clocks_alpha_installed_smartzero_backing_epoch() ||
      clocks_alpha_epoch_install_in_progress()) {
    Payload err;
    err.add("error", "surviving Alpha epoch is not ready");
    err.add("status", "rearm_rejected_alpha_not_ready");
    g_rearm_reject_count++;
    return err;
  }

  if (campaign_name[0] == '\0' || strcmp(campaign_name, name) != 0) {
    Payload err;
    err.add("error", "REARM campaign does not match retained Beta identity");
    err.add("status", "rearm_rejected_campaign_identity");
    err.add("campaign", campaign_name);
    err.add("requested_campaign", name);
    g_rearm_reject_count++;
    return err;
  }

  if (base_count < campaign_seconds) {
    Payload err;
    err.add("error", "REARM projection regresses retained campaign time");
    err.add("status", "rearm_rejected_count_regression");
    err.add("campaign_seconds", campaign_seconds);
    err.add("requested_base_count", base_count);
    g_rearm_reject_count++;
    return err;
  }

  recover_dwt_ns = dwt_ns;
  recover_gnss_ns = gnss_ns;
  recover_ocxo1_ns = ocxo1_ns;
  recover_ocxo2_ns = ocxo2_ns;
  request_rearm = true;
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  flash_cut_clear_pending();
  clocks_watchdog_disarm_campaign_publication();

  g_rearm_request_count++;
  g_rearm_last_base_count = base_count;
  g_rearm_last_expected_first_public_count = expected_first_public_count;
  g_rearm_last_base_gnss_ns = gnss_ns;
  g_rearm_last_base_dwt_ns = dwt_ns;
  g_rearm_last_base_ocxo1_ns = ocxo1_ns;
  g_rearm_last_base_ocxo2_ns = ocxo2_ns;
  safeCopy(g_rearm_last_campaign, sizeof(g_rearm_last_campaign), name);
  safeCopy(g_rearm_last_status, sizeof(g_rearm_last_status),
           "rearm_requested");

  Payload p;
  p.add("status", "rearm_requested");
  p.add("idempotent", false);
  p.add("campaign", campaign_name);
  p.add("base_count", base_count);
  p.add("expected_first_public_count", expected_first_public_count);
  p.add("recording_boundary", "NEXT_COMPLETED_PPS");
  p.add("alpha_mutated", false);
  p.add("instrument_statistics_preserved", true);
  p.add("service_epoch_preserved", true);
  p.add("rearm_mode", "SURVIVING_ALPHA_CAMPAIGN_REARM");
  return p;
}


static FLASHMEM Payload cmd_stop(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_ppb_restore_protocol_clear(true);
  g_clocks_restore_requested = false;
  g_clocks_restore_state = clocks_recovery_restore_state_t{};
  const bool had_live_smartzero = interrupt_smartzero_running();
  const bool had_pending_start = request_start;
  const bool had_pending_zero = request_zero;
  const bool had_pending_recover = request_recover;
  const bool had_pending_rearm = request_rearm;
  const bool had_recovering = clocks_campaign_recovery_lifecycle_active();

  interrupt_smartzero_abort();
  if (had_live_smartzero || had_pending_start || had_pending_zero) {
    clocks_alpha_smartzero_pending_clear();
  }

  request_start = false;
  request_zero = false;
  request_recover = false;
  request_rearm = false;
  flash_cut_clear_pending();

  Payload p;

  if (campaign_state == clocks_campaign_state_t::STARTED &&
      !had_pending_recover && !had_pending_rearm && !had_recovering) {
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

  // STOP while no campaign is running is a campaign-lifecycle abort. It must
  // not invalidate the installed epoch or alter always-on instrument state.
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_ppb_restore_protocol_clear(true);
  g_clocks_restore_requested = false;
  g_clocks_restore_state = clocks_recovery_restore_state_t{};
  if (clocks_campaign_recovery_lifecycle_active() || request_recover) {
    recover_lifecycle_abort("zero_command_abort_recover");
  }
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_rearm = false;
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

static bool clocks_ppb_restore_lifecycle_idle(void) {
  return campaign_state == clocks_campaign_state_t::STOPPED &&
         !request_start && !request_stop && !request_recover && !request_zero &&
         !request_flash_cut && !g_clocks_restore_requested &&
         !clocks_campaign_recovery_lifecycle_active();
}

static bool clocks_ppb_restore_parse_endpoint(
    const Payload& args,
    const char* prefix,
    clocks_alpha_ppb_cumulative_endpoint_snapshot_t& out) {
  char key[80];
  out = clocks_alpha_ppb_cumulative_endpoint_snapshot_t{};

  snprintf(key, sizeof(key), "%s_reference_ns", prefix);
  if (!restore_get_u64(args, key, out.reference_ns)) return false;
  snprintf(key, sizeof(key), "%s_dwt_error_cycles", prefix);
  if (!restore_get_double(args, key, out.dwt_error_cycles)) return false;
  snprintf(key, sizeof(key), "%s_ocxo1_error_ns", prefix);
  if (!restore_get_i64(args, key, out.ocxo1_error_ns)) return false;
  snprintf(key, sizeof(key), "%s_ocxo2_error_ns", prefix);
  if (!restore_get_i64(args, key, out.ocxo2_error_ns)) return false;
  snprintf(key, sizeof(key), "%s_rolling_sequence", prefix);
  if (!restore_get_u32(args, key, out.rolling_sequence)) return false;
  snprintf(key, sizeof(key), "%s_interval_count", prefix);
  if (!restore_get_u32(args, key, out.interval_count)) return false;
  return true;
}

static void clocks_ppb_export_add_endpoint(
    Payload& p,
    const char* prefix,
    const clocks_alpha_ppb_cumulative_endpoint_snapshot_t& endpoint) {
  char key[80];
  snprintf(key, sizeof(key), "%s_reference_ns", prefix);
  p.add(key, endpoint.reference_ns);
  snprintf(key, sizeof(key), "%s_dwt_error_cycles", prefix);
  p.add(key, toFixedDecimal(endpoint.dwt_error_cycles, 9));
  snprintf(key, sizeof(key), "%s_ocxo1_error_ns", prefix);
  p.add(key, endpoint.ocxo1_error_ns);
  snprintf(key, sizeof(key), "%s_ocxo2_error_ns", prefix);
  p.add(key, endpoint.ocxo2_error_ns);
  snprintf(key, sizeof(key), "%s_rolling_sequence", prefix);
  p.add(key, endpoint.rolling_sequence);
  snprintf(key, sizeof(key), "%s_interval_count", prefix);
  p.add(key, endpoint.interval_count);
}

static FLASHMEM Payload cmd_ppb_export_meta(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_PPB_EXPORT_META");

  clocks_alpha_ppb_export_snapshot_t snapshot{};
  if (!clocks_alpha_ppb_export_snapshot(&snapshot) || !snapshot.snapshot_ok) {
    Payload err;
    err.add("error", "Alpha Better-Buckets export snapshot unavailable");
    err.add("status", "ppb_export_snapshot_unavailable");
    return err;
  }

  Payload p;
  p.add("status", "ppb_export_ready");
  p.add("schema", "CLOCKS_PPB_FULL_RING_EXPORT_V1");
  p.add("read_only", true);
  p.add("reset_count", snapshot.reset_count);
  p.add("update_count", snapshot.update_count);
  p.add("current_sequence", snapshot.current_sequence);
  p.add("second_count", snapshot.second_count);
  p.add("minute_count", snapshot.minute_count);
  p.add("second_oldest_sequence", snapshot.second_oldest_sequence);
  p.add("second_newest_sequence", snapshot.second_newest_sequence);
  p.add("minute_oldest_sequence", snapshot.minute_oldest_sequence);
  p.add("minute_newest_sequence", snapshot.minute_newest_sequence);
  p.add("last_minute_key", snapshot.last_minute_key);
  p.add("origin_valid", snapshot.origin_valid);
  p.add("chunk_max_endpoints", CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS);
  clocks_ppb_export_add_endpoint(p, "current", snapshot.current);
  if (snapshot.origin_valid) {
    clocks_ppb_export_add_endpoint(p, "origin", snapshot.origin);
  }
  return p;
}

static FLASHMEM Payload cmd_ppb_export_chunk(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_PPB_EXPORT_CHUNK");

  const char* history = args.getString("history");
  uint32_t reset_count = 0U;
  uint32_t before_sequence = 0U;
  uint32_t count = 0U;
  if (!history || !*history ||
      !restore_get_u32(args, "reset_count", reset_count) ||
      !restore_get_u32(args, "before_sequence", before_sequence) ||
      !restore_get_u32(args, "count", count) ||
      count == 0U || count > CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS) {
    Payload err;
    err.add("error", "invalid Better-Buckets export chunk header");
    err.add("status", "ppb_export_chunk_rejected_header");
    return err;
  }

  const bool seconds = strcmp(history, "SECOND") == 0;
  const bool minutes = strcmp(history, "MINUTE") == 0;
  if (!seconds && !minutes) {
    Payload err;
    err.add("error", "history must be SECOND or MINUTE");
    err.add("status", "ppb_export_chunk_rejected_history");
    return err;
  }

  clocks_alpha_ppb_export_snapshot_t meta{};
  if (!clocks_alpha_ppb_export_snapshot(&meta) || !meta.snapshot_ok ||
      meta.reset_count != reset_count) {
    Payload err;
    err.add("error", "Alpha Better-Buckets export reset identity changed");
    err.add("status", "ppb_export_chunk_rejected_reset");
    err.add("requested_reset_count", reset_count);
    err.add("observed_reset_count", meta.reset_count);
    return err;
  }

  clocks_alpha_ppb_cumulative_endpoint_snapshot_t endpoints[
      CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS]{};
  const uint32_t returned = clocks_alpha_ppb_export_chunk(
      seconds, reset_count, before_sequence, endpoints, count);

  // An empty page is a lawful end-of-history response. No endpoint is invented.
  Payload p;
  p.add("status", "ppb_export_chunk");
  p.add("schema", "CLOCKS_PPB_FULL_RING_EXPORT_V1");
  p.add("history", history);
  p.add("reset_count", reset_count);
  p.add("before_sequence", before_sequence);
  p.add("count", returned);
  for (uint32_t i = 0U; i < returned; ++i) {
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "e%lu", (unsigned long)i);
    clocks_ppb_export_add_endpoint(p, prefix, endpoints[i]);
  }
  return p;
}

static FLASHMEM Payload cmd_ppb_restore_begin(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  if (!clocks_ppb_restore_lifecycle_idle()) {
    Payload err;
    err.add("error", "instrument lifecycle busy");
    err.add("status", "ppb_restore_begin_rejected_busy");
    return err;
  }

  clocks_alpha_ppb_restore_abort();
  clocks_ppb_restore_protocol_clear(false);

  clocks_alpha_ppb_restore_snapshot_t state{};
  if (!restore_get_u32(args, "rolling_sequence", state.rolling_sequence) ||
      !restore_get_u32(args, "second_count", state.expected_second_count) ||
      !restore_get_u32(args, "minute_count", state.expected_minute_count) ||
      !restore_get_u32(args, "last_minute_key", state.last_minute_key) ||
      !restore_get_bool(args, "origin_valid", state.origin_valid) ||
      !clocks_ppb_restore_parse_endpoint(args, "current", state.current) ||
      (state.origin_valid &&
       !clocks_ppb_restore_parse_endpoint(args, "origin", state.origin))) {
    Payload err;
    err.add("error", "invalid Better-Buckets restore metadata");
    err.add("status", "ppb_restore_begin_rejected_state");
    return err;
  }

  if (!clocks_alpha_ppb_restore_begin(&state)) {
    Payload err;
    err.add("error", "Alpha rejected Better-Buckets restore metadata");
    err.add("status", "ppb_restore_begin_rejected_alpha");
    return err;
  }

  g_ppb_restore_protocol_active = true;
  g_ppb_restore_protocol_committed = false;
  g_ppb_restore_protocol_sequence = state.rolling_sequence;
  g_ppb_restore_second_expected = state.expected_second_count;
  g_ppb_restore_minute_expected = state.expected_minute_count;

  Payload p;
  p.add("status", "ppb_restore_staging");
  p.add("rolling_sequence", state.rolling_sequence);
  p.add("second_count", state.expected_second_count);
  p.add("minute_count", state.expected_minute_count);
  p.add("second_capacity", clocks_alpha_ppb_second_capacity());
  p.add("minute_capacity", clocks_alpha_ppb_minute_capacity());
  p.add("chunk_max_endpoints", CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS);
  return p;
}

static FLASHMEM Payload cmd_ppb_restore_chunk(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  if (!g_ppb_restore_protocol_active || g_ppb_restore_protocol_committed) {
    Payload err;
    err.add("error", "Better-Buckets restore is not accepting chunks");
    err.add("status", "ppb_restore_chunk_rejected_state");
    return err;
  }

  const char* history = args.getString("history");
  uint32_t offset = 0U;
  uint32_t count = 0U;
  if (!history || !*history ||
      !restore_get_u32(args, "offset", offset) ||
      !restore_get_u32(args, "count", count) ||
      count == 0U || count > CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS) {
    Payload err;
    err.add("error", "invalid Better-Buckets chunk header");
    err.add("status", "ppb_restore_chunk_rejected_header");
    return err;
  }

  const bool seconds = strcmp(history, "SECOND") == 0;
  const bool minutes = strcmp(history, "MINUTE") == 0;
  if (!seconds && !minutes) {
    Payload err;
    err.add("error", "history must be SECOND or MINUTE");
    err.add("status", "ppb_restore_chunk_rejected_history");
    return err;
  }

  uint32_t& accepted = seconds
      ? g_ppb_restore_second_accepted : g_ppb_restore_minute_accepted;
  const uint32_t expected = seconds
      ? g_ppb_restore_second_expected : g_ppb_restore_minute_expected;

  // Command transport may retry after a lost response. A chunk wholly below
  // the accepted frontier is already committed and therefore idempotent.
  if (offset < accepted && offset + count <= accepted) {
    Payload p;
    p.add("status", "ppb_restore_chunk_duplicate");
    p.add("history", history);
    p.add("accepted", accepted);
    p.add("expected", expected);
    return p;
  }
  if (offset != accepted || offset + count > expected) {
    Payload err;
    err.add("error", "Better-Buckets chunk is out of order");
    err.add("status", "ppb_restore_chunk_rejected_order");
    err.add("history", history);
    err.add("offset", offset);
    err.add("accepted", accepted);
    err.add("expected", expected);
    return err;
  }

  clocks_alpha_ppb_cumulative_endpoint_snapshot_t endpoints[
      CLOCKS_PPB_RESTORE_CHUNK_MAX_ENDPOINTS]{};
  for (uint32_t i = 0U; i < count; ++i) {
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "e%lu", (unsigned long)i);
    if (!clocks_ppb_restore_parse_endpoint(args, prefix, endpoints[i])) {
      clocks_ppb_restore_protocol_clear(true);
      Payload err;
      err.add("error", "invalid Better-Buckets endpoint");
      err.add("status", "ppb_restore_chunk_rejected_endpoint");
      err.add("index", i);
      return err;
    }
  }

  for (uint32_t i = 0U; i < count; ++i) {
    const bool ok = seconds
        ? clocks_alpha_ppb_restore_append_second(&endpoints[i])
        : clocks_alpha_ppb_restore_append_minute(&endpoints[i]);
    if (!ok) {
      clocks_ppb_restore_protocol_clear(true);
      Payload err;
      err.add("error", "Alpha rejected Better-Buckets endpoint");
      err.add("status", "ppb_restore_chunk_rejected_alpha");
      err.add("index", i);
      return err;
    }
  }
  accepted += count;

  Payload p;
  p.add("status", "ppb_restore_chunk_accepted");
  p.add("history", history);
  p.add("accepted", accepted);
  p.add("expected", expected);
  return p;
}

static FLASHMEM Payload cmd_ppb_restore_commit(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  uint32_t rolling_sequence = 0U;
  if (!restore_get_u32(args, "rolling_sequence", rolling_sequence)) {
    Payload err;
    err.add("error", "missing rolling_sequence");
    err.add("status", "ppb_restore_commit_rejected_header");
    return err;
  }

  if (g_ppb_restore_protocol_committed &&
      rolling_sequence == g_ppb_restore_protocol_sequence) {
    Payload p;
    p.add("status", "ppb_restore_committed");
    p.add("rolling_sequence", rolling_sequence);
    p.add("duplicate", true);
    return p;
  }

  if (!g_ppb_restore_protocol_active ||
      rolling_sequence != g_ppb_restore_protocol_sequence ||
      g_ppb_restore_second_accepted != g_ppb_restore_second_expected ||
      g_ppb_restore_minute_accepted != g_ppb_restore_minute_expected) {
    Payload err;
    err.add("error", "Better-Buckets restore is incomplete");
    err.add("status", "ppb_restore_commit_rejected_incomplete");
    err.add("second_accepted", g_ppb_restore_second_accepted);
    err.add("second_expected", g_ppb_restore_second_expected);
    err.add("minute_accepted", g_ppb_restore_minute_accepted);
    err.add("minute_expected", g_ppb_restore_minute_expected);
    return err;
  }

  if (!clocks_alpha_ppb_restore_commit(rolling_sequence)) {
    clocks_ppb_restore_protocol_clear(true);
    Payload err;
    err.add("error", "Alpha rejected Better-Buckets commit");
    err.add("status", "ppb_restore_commit_rejected_alpha");
    return err;
  }

  g_ppb_restore_protocol_committed = true;
  Payload p;
  p.add("status", "ppb_restore_committed");
  p.add("rolling_sequence", rolling_sequence);
  p.add("second_count", g_ppb_restore_second_accepted);
  p.add("minute_count", g_ppb_restore_minute_accepted);
  return p;
}

static FLASHMEM Payload cmd_ppb_restore_abort(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  const uint32_t prior_sequence = g_ppb_restore_protocol_sequence;
  clocks_ppb_restore_protocol_clear(true);
  Payload p;
  p.add("status", "ppb_restore_aborted");
  p.add("rolling_sequence", prior_sequence);
  return p;
}

static FLASHMEM Payload cmd_restore_clocks_state(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  if (campaign_state != clocks_campaign_state_t::STOPPED ||
      request_start || request_stop || request_recover || request_zero ||
      request_flash_cut || g_clocks_restore_requested ||
      clocks_campaign_recovery_lifecycle_active()) {
    Payload err;
    err.add("error", "instrument lifecycle busy");
    err.add("status", "monitor_restore_rejected_busy");
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return err;
  }

  g_clocks_restore_state = clocks_recovery_restore_state_t{};
  if (!clocks_recovery_state_from_args(
          args, g_clocks_restore_state)) {
    g_clocks_restore_failure_count++;
    Payload err;
    err.add("error", "invalid structured recovery state");
    err.add("status", "monitor_restore_rejected_state");
    err.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
    return err;
  }

  if (g_clocks_restore_state.stats.update_count != 0U &&
      !clocks_alpha_ppb_restore_ready(g_clocks_restore_state.stats.update_count)) {
    const uint32_t rolling_sequence = g_clocks_restore_state.stats.update_count;
    g_clocks_restore_state = clocks_recovery_restore_state_t{};
    Payload pending;
    pending.add("status", "monitor_restore_requires_ppb_state");
    pending.add("rolling_sequence", rolling_sequence);
    pending.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
    return pending;
  }

  if (!clocks_alpha_installed_smartzero_backing_epoch() &&
      !interrupt_smartzero_running() &&
      !interrupt_smartzero_complete() &&
      !clocks_alpha_epoch_install_in_progress()) {
    if (!clocks_alpha_begin_smartzero_epoch("monitor_restore")) {
      g_clocks_restore_state = clocks_recovery_restore_state_t{};
      g_clocks_restore_failure_count++;
      clocks_ppb_restore_protocol_clear(true);
      Payload err;
      err.add("error", "failed to start SmartZero for monitor restore");
      err.add("status", "monitor_restore_rejected_smartzero");
      return err;
    }
  }

  g_clocks_restore_requested = true;
  g_clocks_restore_request_count++;

  Payload p;
  p.add("status", "monitor_restore_requested");
  p.add("boundary", "next_completed_pps_after_epoch_ready");
  p.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
  p.add("request_count", g_clocks_restore_request_count);
  p.add("epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  p.add("smartzero_running", interrupt_smartzero_running());
  return p;
}

static FLASHMEM Payload cmd_recover(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_payload_numeric_integrity_reset();

  if (g_clocks_restore_requested) {
    Payload err;
    err.add("error", "MONITOR restore is pending");
    err.add("status", "recover_rejected_monitor_restore_pending");
    return err;
  }

  g_campaign_restore_state = clocks_recovery_restore_state_t{};
  const bool recovery_state_supplied = args.has("restore_schema_version");
  if (!recovery_state_supplied) {
    Payload err;
    err.add("error", "RECOVER requires a complete dead-producer desired state");
    err.add("status", "recover_rejected_missing_restore_state");
    return err;
  }
  if (!clocks_recovery_state_from_args(
          args, g_campaign_restore_state)) {
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
      request_rearm ||
      clocks_campaign_recovery_lifecycle_active() ||
      g_recover_proof_active ||
      g_recover_proof_degraded_active ||
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
    p.add("recover_proof_active", (bool)g_recover_proof_active);
    p.add("recover_degraded_active",
          (bool)g_recover_proof_degraded_active);
    p.add("recover_clockface_ready",
          (bool)g_recover_proof_clockface_ready);
    p.add("recover_science_ready",
          (bool)g_recover_proof_science_ready);
    p.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return p;
  }

  // A transport retry may arrive after the exact recovery already completed.
  // Re-running the same identity would repeat the campaign splice and retire
  // publication custody a second time, so acknowledge it without touching state.
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
  // If this newborn lifetime already completed startup SmartZero, keep that
  // installed epoch; otherwise preserve the in-flight startup acquisition.
  if (clocks_alpha_installed_smartzero_backing_epoch()) {
    interrupt_smartzero_abort();
  }
  request_recover = true;
  request_start   = false;
  request_stop    = false;
  request_zero    = false;
  flash_cut_clear_pending();

  if (!recover_lifecycle_enter_from_command("recover_command_armed")) {
    recover_lifecycle_abort("recover_dead_producer_epoch_prepare_failed");

    Payload err;
    err.add("error", "failed to prepare newborn Alpha epoch for dead-producer restore");
    err.add("status", "recover_rejected_dead_producer_epoch_prepare");
    err.add("recovery_generation", g_recover_request_count);
    err.add("campaign_state", clocks_campaign_state_name(campaign_state));
    return err;
  }

  {
    if (!g_campaign_restore_state.valid) {
      recover_lifecycle_abort("recover_dead_producer_restore_requires_structured_state");
      Payload err;
      err.add("error", "dead-producer restore requires structured instrument state");
      err.add("status", "recover_rejected_dead_producer_restore_requires_state");
      err.add("recover_mode", "DEAD_PRODUCER_RESTORE");
      return err;
    }
    if (g_campaign_restore_state.stats.update_count != 0U &&
        !clocks_alpha_ppb_restore_ready(
            g_campaign_restore_state.stats.update_count)) {
      g_campaign_restore_state = clocks_recovery_restore_state_t{};
      recover_lifecycle_abort("recover_dead_producer_restore_requires_ppb_state");
      Payload err;
      err.add("error", "dead-producer restore requires committed Better-Buckets state");
      err.add("status", "recover_rejected_dead_producer_restore_requires_ppb_state");
      err.add("recover_mode", "DEAD_PRODUCER_RESTORE");
      return err;
    }
  }

  Payload p;
  p.add("status", "recover_requested");
  p.add("idempotent", false);
  p.add("recovery_generation", g_recover_request_count);
  p.add("instrument_statistics_preserved", false);
  p.add("instrument_statistics_restore_staged", g_campaign_restore_state.valid);
  p.add("instrument_statistics_restored", false);
  p.add("instrument_statistics_recovery_source", "CLOCKS_RECOVERY_SNAPSHOT");
  p.add("recovery_state_supplied", recovery_state_supplied);
  p.add("producer_restore_required", true);
  p.add("restore_schema_version", CLOCKS_STRUCTURED_RESTORE_VERSION);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count",
        g_recover_last_expected_first_public_count);
  p.add("campaign", campaign_name);
  p.add("campaign_supplied", g_recover_last_campaign_supplied);
  p.add("recover_last_campaign", g_recover_last_campaign);
  p.add("recover_status_report", "REPORT_RECOVERY");
  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("recover_lifecycle", clocks_campaign_recovery_lifecycle_active());
  p.add("recover_lifecycle_reason", g_recover_lifecycle_reason);
  p.add("recover_mode",
        recover_lifecycle_mode_name(g_recover_lifecycle_mode));
  p.add("recover_dead_producer_restore_active",
        g_recover_lifecycle_mode ==
            recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE &&
        clocks_campaign_recovery_lifecycle_active());
  p.add("recover_dead_producer_restore_epoch_ready",
        g_recover_lifecycle_dead_producer_restore_epoch_ready);
  p.add("recover_smartzero_running", interrupt_smartzero_running());
  p.add("recover_smartzero_complete", interrupt_smartzero_complete());
  p.add("recover_epoch_ready",
        clocks_alpha_installed_smartzero_backing_epoch());
  p.add("restore_court_ready", recover_restore_court_ready_now());
  p.add("recover_command_custody_reset_count",
        g_recover_lifecycle_command_custody_reset_count);
  p.add("interrupt_recover_publication_reset_count",
        interrupt_recover_publication_custody_reset_count());
  return p;
}

static FLASHMEM Payload cmd_recover_abort(const Payload& args) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_RECOVERY);

  Payload p;
  p.add("report", "CLOCKS_RECOVERY");
  p.add("schema", "CLOCKS_RECOVERY_COMPACT_V2");

  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);

  p.add("rearm_active", (bool)request_rearm);
  p.add("rearm_request_count", g_rearm_request_count);
  p.add("rearm_commit_count", g_rearm_commit_count);
  p.add("rearm_reject_count", g_rearm_reject_count);
  p.add("rearm_base_count", g_rearm_last_base_count);
  p.add("rearm_expected_first_public_count",
        g_rearm_last_expected_first_public_count);
  p.add("rearm_base_gnss_ns", g_rearm_last_base_gnss_ns);
  p.add("rearm_base_dwt_ns", g_rearm_last_base_dwt_ns);
  p.add("rearm_base_ocxo1_ns", g_rearm_last_base_ocxo1_ns);
  p.add("rearm_base_ocxo2_ns", g_rearm_last_base_ocxo2_ns);
  p.add("rearm_campaign", g_rearm_last_campaign);
  p.add("rearm_status", g_rearm_last_status);

  p.add("recover_lifecycle_active", clocks_campaign_recovery_lifecycle_active());
  p.add("recover_lifecycle_reason", g_recover_lifecycle_reason);
  p.add("recover_mode", recover_lifecycle_mode_name(g_recover_lifecycle_mode));
  p.add("producer_restore_active", clocks_campaign_recovery_lifecycle_active());
  p.add("recover_dead_producer_restore_active",
        g_recover_lifecycle_mode == recover_lifecycle_mode_t::DEAD_PRODUCER_RESTORE &&
        clocks_campaign_recovery_lifecycle_active());
  p.add("recover_dead_producer_restore_epoch_ready",
        g_recover_lifecycle_dead_producer_restore_epoch_ready);
  p.add("recover_dead_producer_restore_begin_count",
        g_recover_lifecycle_dead_producer_restore_begin_count);
  p.add("recover_dead_producer_restore_wait_count",
        g_recover_lifecycle_dead_producer_restore_wait_count);
  p.add("recover_dead_producer_restore_ready_count",
        g_recover_lifecycle_dead_producer_restore_ready_count);
  p.add("recover_dead_producer_restore_commit_count",
        g_recover_lifecycle_dead_producer_restore_commit_count);
  p.add("recover_smartzero_running", interrupt_smartzero_running());
  p.add("recover_smartzero_complete", interrupt_smartzero_complete());
  p.add("recover_epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  p.add("restore_court_ready", recover_restore_court_ready_now());

  p.add("recovery_generation", g_recover_request_count);
  p.add("request_count", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count",
        g_recover_last_expected_first_public_count);
  p.add("last_public_count", g_campaign_record_last_public_count);
  p.add("candidate_count", g_campaign_record_candidate_count);
  p.add("hidden_candidate_count", g_recover_proof_hidden_candidate_count);
  p.add("campaign_record_last_stage", g_campaign_record_last_stage);
  p.add("campaign_record_last_stage_name",
        campaign_record_stage_name(g_campaign_record_last_stage));
  p.add("campaign_record_pending",
        clocks_fragment_campaign_queue_depth() != 0U);
  p.add("campaign_record_pending_sequence",
        clocks_fragment_campaign_queue_front_sequence());
  p.add("campaign_record_stage_count",
        g_clocks_fragment_campaign_record_stage_count);
  p.add("campaign_record_take_count",
        g_clocks_fragment_campaign_record_take_count);
  p.add("campaign_record_backlog_count",
        g_clocks_fragment_campaign_record_backlog_count);
  p.add("campaign_record_observer_loss_seen",
        g_clocks_fragment_campaign_observer_loss_seen);
  p.add("campaign_record_observer_retired_through_sequence",
        g_clocks_fragment_campaign_observer_retired_through_sequence);
  p.add("campaign_record_observer_drop_count",
        g_clocks_fragment_campaign_observer_drop_count);
  p.add("campaign_record_observer_drop_first_sequence",
        g_clocks_fragment_campaign_observer_drop_first_sequence);
  p.add("campaign_record_observer_drop_last_sequence",
        g_clocks_fragment_campaign_observer_drop_last_sequence);
  p.add("campaign_record_observer_drop_last_public_count",
        g_clocks_fragment_campaign_observer_drop_last_public_count);
  p.add("campaign_record_observer_queue_retire_count",
        g_clocks_fragment_campaign_observer_queue_retire_count);
  p.add("campaign_record_observer_queue_retire_last_sequence",
        g_clocks_fragment_campaign_observer_queue_retire_last_sequence);
  p.add("campaign_record_ready_retry_armed",
        g_clocks_fragment_campaign_record_ready_retry_handle != TIMEPOP_INVALID_HANDLE);
  p.add("campaign_record_ready_retry_arm_count",
        g_clocks_fragment_campaign_record_ready_retry_arm_count);
  p.add("campaign_record_ready_retry_arm_failure_count",
        g_clocks_fragment_campaign_record_ready_retry_arm_failure_count);
  p.add("campaign_record_ready_retry_fire_count",
        g_clocks_fragment_campaign_record_ready_retry_fire_count);
  p.add("campaign_record_ready_retry_cancel_count",
        g_clocks_fragment_campaign_record_ready_retry_cancel_count);
  p.add("fragment_publication_publish_reject_count",
        g_clocks_fragment_publication_publish_reject_count);
  p.add("fragment_publication_retry_active",
        g_clocks_fragment_publication_retry_snapshot_valid);
  p.add("fragment_publication_retry_sequence",
        g_clocks_fragment_publication_retry_sequence);
  p.add("fragment_publication_retry_attempt_count",
        g_clocks_fragment_publication_retry_attempt_count);
  p.add("fragment_publication_retry_reason_id",
        g_clocks_fragment_publication_retry_snapshot_valid
            ? g_clocks_fragment_publication_retry_reason_id
            : CLOCKS_FRAGMENT_RETRY_REASON_NONE);
  p.add("fragment_publication_retry_reason",
        clocks_fragment_retry_reason_name(
            g_clocks_fragment_publication_retry_snapshot_valid
                ? g_clocks_fragment_publication_retry_reason_id
                : CLOCKS_FRAGMENT_RETRY_REASON_NONE));
  p.add("fragment_publication_retry_pi_only",
        g_clocks_fragment_publication_retry_pi_only);
  p.add("fragment_publication_retry_schedule_count",
        g_clocks_fragment_publication_retry_schedule_count);
  p.add("fragment_publication_retry_success_count",
        g_clocks_fragment_publication_retry_success_count);
  p.add("fragment_publication_campaign_row_embed_fail_count",
        g_clocks_fragment_publication_campaign_row_embed_fail_count);
  p.add("fragment_publication_service_arm_failures",
        g_clocks_fragment_publication_service_arm_failures);
  p.add("fragment_publication_active_retry_max_attempts",
        (uint32_t)CLOCKS_FRAGMENT_ACTIVE_RETRY_MAX_ATTEMPTS);
  p.add("fragment_publication_active_retry_exhaustion_count",
        g_clocks_fragment_publication_active_retry_exhaustion_count);
  p.add("fragment_publication_recovery_retry_abort_count",
        g_clocks_fragment_publication_recovery_retry_abort_count);
  p.add("fragment_publication_last_exhausted_sequence",
        g_clocks_fragment_publication_last_exhausted_sequence);
  p.add("fragment_publication_last_exhausted_attempt_count",
        g_clocks_fragment_publication_last_exhausted_attempt_count);
  p.add("fragment_publication_last_exhausted_reason_id",
        g_clocks_fragment_publication_last_exhausted_reason_id);
  p.add("fragment_publication_last_exhausted_reason",
        clocks_fragment_retry_reason_name(
            g_clocks_fragment_publication_last_exhausted_reason_id));
  p.add("fragment_publication_last_exhausted_pending_sequence",
        g_clocks_fragment_publication_last_exhausted_pending_sequence);
  p.add("fragment_publication_recovery_reset_count",
        g_clocks_fragment_publication_recovery_reset_count);
  p.add("fragment_publication_retired_retry_sequence",
        g_clocks_fragment_publication_recovery_retired_retry_sequence);
  p.add("fragment_publication_retired_pending_sequence",
        g_clocks_fragment_publication_recovery_retired_pending_sequence);
  p.add("fragment_publication_retired_campaign_sequence",
        g_clocks_fragment_publication_recovery_retired_campaign_sequence);
  p.add("fragment_publication_watchdog_reset_count",
        g_clocks_fragment_publication_watchdog_reset_count);
  p.add("fragment_publication_watchdog_retired_retry_sequence",
        g_clocks_fragment_publication_watchdog_retired_retry_sequence);
  p.add("fragment_publication_watchdog_retired_pending_sequence",
        g_clocks_fragment_publication_watchdog_retired_pending_sequence);
  p.add("fragment_publication_watchdog_retired_campaign_sequence",
        g_clocks_fragment_publication_watchdog_retired_campaign_sequence);
  p.add("fragment_publication_idle_retry_abandon_count",
        g_clocks_fragment_publication_idle_retry_abandon_count);
  p.add("fragment_publication_idle_retry_last_sequence",
        g_clocks_fragment_publication_idle_retry_last_sequence);
  p.add("fragment_publication_idle_retry_last_attempt_count",
        g_clocks_fragment_publication_idle_retry_last_attempt_count);
  p.add("fragment_publication_observer_retire_count",
        g_clocks_fragment_publication_observer_retire_count);
  p.add("fragment_publication_observer_retire_last_sequence",
        g_clocks_fragment_publication_observer_retire_last_sequence);

  p.add("recover_proof_active", (bool)g_recover_proof_active);
  p.add("recover_proof_degraded_active",
        (bool)g_recover_proof_degraded_active);
  p.add("recover_proof_reason", g_recover_proof_last_reason);
  p.add("recover_clockface_ready", (bool)g_recover_proof_clockface_ready);
  p.add("recover_science_ready", (bool)g_recover_proof_science_ready);
  p.add("recover_proof_stalled", (bool)g_recover_proof_stalled);
  p.add("recover_proof_attempts", g_recover_proof_attempt_count);
  p.add("recover_proof_warn_after_attempts",
        (uint32_t)CLOCKS_RECOVER_PROOF_WARN_AFTER_ATTEMPTS);
  p.add("recover_proof_warning_published", g_recover_proof_warning_published);
  p.add("recover_proof_release_count", g_recover_proof_release_count);
  p.add("recover_science_proof_release_count",
        g_recover_science_proof_release_count);
  p.add("recover_proof_last_release_pps_sequence",
        g_recover_proof_last_release_pps_sequence);
  p.add("degraded_no_progress_row_count",
        g_recover_proof_no_progress_row_count);
  p.add("degraded_last_progress_public_count",
        g_recover_proof_last_progress_public_count);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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
  clocks_payload_owner_assert(clocks_payload_owner_t::COMMAND);
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

static FLASHMEM void report_add_row_court_lane(
    Payload& parent,
    const char* key,
    time_clock_id_t clock,
    uint32_t court_sequence) {
  g_beta_counterledger_raw_scratch =
      clocks_alpha_ocxo_counterledger_snapshot_t{};
  const bool snapshot_ok = clocks_alpha_ocxo_counterledger_snapshot(
      clock, &g_beta_counterledger_raw_scratch);
  const clocks_alpha_ocxo_counterledger_snapshot_t& lane =
      g_beta_counterledger_raw_scratch;

  Payload detail;
  detail.add("snapshot_ok", snapshot_ok);
  detail.add("valid", lane.valid);
  detail.add("initialized", lane.initialized);
  detail.add("interval_valid", lane.interval_valid);
  detail.add("pps_sequence", lane.pps_sequence);
  detail.add("court_sequence_match",
             court_sequence != 0U && lane.pps_sequence == court_sequence);
  detail.add("sample_count", lane.sample_count);
  detail.add("last_counter32", lane.last_counter32);

  detail.add("phase_valid", lane.phase_valid);
  detail.add("phase_pending", lane.phase_pending);
  detail.add("phase_pps_sequence", lane.phase_pps_sequence);
  detail.add("phase_sequence_match",
             court_sequence != 0U &&
                 lane.phase_pps_sequence == court_sequence);
  detail.add("phase_lag_pps", lane.phase_lag_pps);
  detail.add("phase_implied_counter32_at_pps",
             lane.phase_implied_counter32_at_pps);
  detail.add("counter_identity_match",
             lane.phase_valid &&
                 lane.phase_implied_counter32_at_pps == lane.last_counter32);
  detail.add("phase_pending_depth", lane.phase_pending_depth);
  detail.add("phase_pending_oldest_pps_sequence",
             lane.phase_pending_oldest_pps_sequence);
  detail.add("phase_pending_newest_pps_sequence",
             lane.phase_pending_newest_pps_sequence);
  detail.add("phase_last_resolved_pps_sequence",
             lane.phase_pending_last_resolved_pps_sequence);
  detail.add("phase_last_resolve_reason_id",
             lane.last_phase_resolve_reason_id);
  detail.add("phase_last_resolve_reason",
             clocks_phaseledger_resolve_reason_name(
                 lane.last_phase_resolve_reason_id));
  detail.add("phase_last_resolve_source_id",
             lane.phase_last_resolve_source_id);
  detail.add("phase_last_resolve_source",
             clocks_phaseledger_resolve_source_name(
                 lane.phase_last_resolve_source_id));

  detail.add("refined_valid", lane.refined_valid);
  detail.add("refined_interval_valid", lane.refined_interval_valid);
  detail.add("refined_ns", lane.refined_ns);
  detail.add("refined_interval_ns", lane.refined_interval_ns);
  detail.add("last_sample_decision_id", lane.last_sample_decision_id);
  detail.add("last_sample_decision",
             clocks_counterledger_sample_decision_name(
                 lane.last_sample_decision_id));
  detail.add("last_sample_pps_sequence", lane.last_sample_pps_sequence);
  detail.add("last_sample_delta_ticks", lane.last_sample_delta_ticks);

  parent.add_object(key, detail);
}

static FLASHMEM Payload cmd_report_row_court(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_ROW_COURT");

  clocks_alpha_row_court_snapshot_t court{};
  const bool snapshot_ok = clocks_alpha_row_court_snapshot(&court);
  const uint32_t court_sequence =
      court.row_open ? court.row_sequence : court.selector_sequence;

  Payload p;
  p.add("report", "CLOCKS_ROW_COURT");
  p.add("schema", "CLOCKS_ALPHA_ROW_COURT_V1");
  p.add("read_only", true);
  p.add("snapshot_ok", snapshot_ok && court.snapshot_ok);
  p.add("court_sequence", court_sequence);
  p.add("epoch_ready", court.epoch_ready);

  Payload selector;
  selector.add("sequence", court.selector_sequence);
  selector.add("dwt", court.selector_dwt);
  selector.add("vclock_event_counter32", court.vclock_event_counter32);
  selector.add("vclock_event_dwt", court.vclock_event_dwt);
  selector.add("anchor_counter32", court.anchor_counter32);
  selector.add("counter_match", court.selector_vclock_counter_match);
  selector.add("dwt_match", court.selector_vclock_dwt_match);
  selector.add("match", court.selector_vclock_match);
  p.add_object("selector_vclock_court", selector);

  Payload row;
  row.add("open", court.row_open);
  row.add("sequence", court.row_sequence);
  row.add("ocxo1_complete", court.row_ocxo1_complete);
  row.add("ocxo2_complete", court.row_ocxo2_complete);
  row.add("missing_mask", court.row_missing_mask);
  row.add("missing_epoch_ready",
          (court.row_missing_mask & (1U << 0)) != 0U);
  row.add("missing_sequence",
          (court.row_missing_mask & (1U << 1)) != 0U);
  row.add("missing_ocxo1_previous_edge",
          (court.row_missing_mask & (1U << 2)) != 0U);
  row.add("missing_ocxo2_previous_edge",
          (court.row_missing_mask & (1U << 3)) != 0U);
  row.add("missing_pps_witness_sequence",
          (court.row_missing_mask & (1U << 4)) != 0U);
  row.add("missing_anchor_dwt",
          (court.row_missing_mask & (1U << 5)) != 0U);
  row.add("missing_anchor_gnss",
          (court.row_missing_mask & (1U << 6)) != 0U);
  row.add("missing_ocxo1_lane",
          (court.row_missing_mask & (1U << 7)) != 0U);
  row.add("missing_ocxo2_lane",
          (court.row_missing_mask & (1U << 8)) != 0U);
  p.add_object("row", row);

  Payload anchor_state;
  anchor_state.add("pps_witness_sequence", court.pps_witness_sequence);
  anchor_state.add("dwt", court.anchor_dwt);
  anchor_state.add("counter32", court.anchor_counter32);
  anchor_state.add("gnss_ns", court.anchor_gnss_ns);
  p.add_object("anchor", anchor_state);

  Payload lanes;
  lanes.add("last_ocxo1_pps_sequence", court.last_ocxo1_pps_sequence);
  lanes.add("last_ocxo2_pps_sequence", court.last_ocxo2_pps_sequence);
  lanes.add("ocxo1_ready", court.ocxo1_lane_ready);
  lanes.add("ocxo2_ready", court.ocxo2_lane_ready);
  lanes.add("ocxo1_waiting_for_phase", court.ocxo1_waiting_for_phase);
  lanes.add("ocxo2_waiting_for_phase", court.ocxo2_waiting_for_phase);
  report_add_row_court_lane(
      lanes, "ocxo1", time_clock_id_t::OCXO1, court_sequence);
  report_add_row_court_lane(
      lanes, "ocxo2", time_clock_id_t::OCXO2, court_sequence);
  p.add_object("lanes", lanes);

  p.add("campaign_state", clocks_campaign_state_name(campaign_state));
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("report_priority0_capture_live", true);
  p.add("report_priority16_excluded", true);
  return p;
}

static FLASHMEM Payload cmd_report_smartzero(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_report_build_guard_t guard;
  if (!guard.acquired) return clocks_report_busy_response("CLOCKS_SMARTZERO");

  Payload& built = g_report_smartzero_payload;
  built.clear();
  built.add("report", "CLOCKS_SMARTZERO");
  built.add("schema", "CLOCKS_SMARTZERO_REPORT_V1");
  built.add("read_only", true);
  built.add("campaign_state", clocks_campaign_state_name(campaign_state));
  built.add("campaign", campaign_name);
  built.add("campaign_seconds", campaign_seconds);
  built.add("zero_request_pending", (bool)request_zero);
  built.add("epoch_ready", clocks_alpha_installed_smartzero_backing_epoch());
  built.add("epoch_sequence", clocks_alpha_epoch_sequence());
  built.add("report_priority0_capture_live", true);
  built.add("report_priority16_excluded", true);
  payload_add_smartzero_summary(built);

  Payload response = built;
  built.clear();
  return response;
}

static FLASHMEM Payload cmd_report_clocks(const Payload&) {
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_ppb_restore_protocol_clear(true);
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
  const clocks_payload_custody_t payload_custody(
      clocks_payload_owner_t::COMMAND);
  clocks_stack_witness_reset();
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_STACK);
  Payload p;
  p.add("report", "CLOCKS_STACK_WITNESS");
  p.add("schema", "CLOCKS_STACK_WITNESS_REPORT_V1");
  p.add("reset", true);
  payload_add_stack_witness(p);
  return p;
}


// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",               cmd_start               },
  { "REARM",               cmd_rearm               },
  { "FLASH_CUT",           cmd_flash_cut           },
  { "STOP",                cmd_stop                },
  { "ZERO",                cmd_zero                },
  { "RECOVER",             cmd_recover             },
  { "PPB_EXPORT_META",     cmd_ppb_export_meta     },
  { "PPB_EXPORT_CHUNK",    cmd_ppb_export_chunk    },
  { "PPB_RESTORE_BEGIN",   cmd_ppb_restore_begin   },
  { "PPB_RESTORE_CHUNK",   cmd_ppb_restore_chunk   },
  { "PPB_RESTORE_COMMIT",  cmd_ppb_restore_commit  },
  { "PPB_RESTORE_ABORT",   cmd_ppb_restore_abort   },
  { "RESTORE_MONITOR",     cmd_restore_clocks_state     },
  { "RECOVER_ABORT",       cmd_recover_abort       },
  { "REPORT_CLOCKS",       cmd_report_clocks       },
  { "REPORT_STATS",        cmd_report_stats        },
  { "REPORT_SMARTZERO",    cmd_report_smartzero    },
  { "REPORT_ROW_COURT",    cmd_report_row_court    },
  { "STATS_RESET",         cmd_stats_reset         },
  { "REPORT_RECOVERY",     cmd_report_recovery     },
  { "STACK_WITNESS_RESET", cmd_stack_witness_reset },
  { "WATCHDOG_TEST",       cmd_watchdog_test       },
  { "INJECT_PROBLEM",      cmd_inject_problem      },
  { nullptr,                 nullptr                 }
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}
