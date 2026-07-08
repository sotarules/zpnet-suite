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
// Campaign lifecycle and watchdog behavior remain unchanged. Servo DAC updates
// are now planned from the 1 Hz science path but physically committed by a
// low-priority actuator sandbox so I2C failures cannot poison TIMEBASE.
// Servo inputs consume the same PPS-founded OCXO residual surface that feeds
// the OCXO Welfords, and DAC/TIMEBASE reports expose that provenance.
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
#include "process_system.h"

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
// epoch and learned timing state, change the campaign identity, and rebase
// Beta's campaign-public clocks/statistics at the next PPS/VCLOCK edge.
static volatile bool request_flash_cut = false;
static char flash_cut_campaign_name[64] = {0};

uint64_t recover_dwt_ns   = 0;
uint64_t recover_gnss_ns  = 0;
uint64_t recover_ocxo1_ns = 0;
uint64_t recover_ocxo2_ns = 0;

// Do not retain a copy of the most recent TIMEBASE_FRAGMENT here.
// The Pi persists the authoritative row, and keeping a second heap-backed
// Payload alive on Teensy made campaign publication unnecessarily fragile.

// Alpha-authored physical PPS witness DWT audit surface.  These are published
// into the TIMEBASE publication pair so Pi-side reports can compare physical PPS-to-PPS
// DWT intervals against the canonical PPS/VCLOCK DWT rail.
extern volatile uint32_t g_pps_dwt_at_edge;
extern volatile uint32_t g_pps_dwt_cycles_between_edges;
extern volatile bool     g_pps_dwt_cycles_between_edges_valid;

// Species-pure observed PPS/VCLOCK edge-to-edge interval. Delta Cycles uses
// this rail, not the smoother PPS/GPIO DWT-GNSS calibration, so the reference
// interval is formed by the same observed DWT-at-edge subtraction doctrine as
// OCXO.
extern volatile uint32_t g_pps_vclock_dwt_cycles_between_edges;
extern volatile bool     g_pps_vclock_dwt_cycles_between_edges_valid;

// Alpha OCXO PPS projection guard counters.  Alpha owns the fix and the
// counters; Beta only surfaces them through focused reports so the normal
// TIMEBASE pair remains lean and brutally honest.
extern uint32_t clocks_alpha_ocxo_projection_guard_legacy_wrap_count(time_clock_id_t clock);
extern uint32_t clocks_alpha_ocxo_projection_guard_sanity_reject_count(time_clock_id_t clock);

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

// Temporary safety valve while the lower-envelope report-only rail is being
// validated.  Pi-side clocks currently treats TIMEBASE_FRAGMENT and
// TIMEBASE_FORENSICS as an atomic pair and faults if a new fragment identity
// arrives while the prior identity has no forensics companion.  Therefore we
// must keep publishing the TIMEBASE_FORENSICS companion row, but we can make it
// deliberately tiny until the lower-envelope payload is sane.
static constexpr bool TIMEBASE_FORENSICS_PUBLISH_ENABLED = true;
static constexpr bool TIMEBASE_FORENSICS_MINIMAL_PAYLOAD_ENABLED = true;

// Ultra-slim raw-cycle companion fields for the 1 Hz TIMEBASE_FORENSICS row.
// Deliberately flat: no nested lane objects, no service objects, and no deep
// autopsy surfaces.  This is the next step above MINIMAL_PAIR_ONLY and is
// intended to preserve the Pi pair contract while giving raw_cycles enough
// cycle evidence to plot observed/EMA/FloorLine behavior.
//
// FloorLine V2 note:
//   MINIMAL_PAIR_ONLY is still the right 1 Hz TIMEBASE companion policy while
//   we keep the campaign row small.  But the raw_cycles report cannot see
//   FloorLine unless the flat micro fields are present:
//
//     v_raw/v_fl/v_pub, o1_raw/o1_fl/o1_pub, o2_raw/o2_fl/o2_pub
//
//   With this enabled, raw_cycles can directly verify whether the subscriber
//   published endpoint equals the FloorLine endpoint, without turning the full
//   deep forensic payload back on.
static constexpr bool TIMEBASE_FORENSICS_MICRO_RAW_CYCLES_ENABLED = false;
static constexpr bool TIMEBASE_FORENSICS_MINIMAL_HEALTH_FIELDS_ENABLED = false;

// The richer slim/full forensics builders remain compiled for focused future
// experiments, but they are not used by the 1 Hz campaign stream in this step.
static constexpr bool TIMEBASE_FORENSICS_SLIM_RAW_CYCLES_PAYLOAD_ENABLED = false;
static constexpr bool TIMEBASE_FORENSICS_FLOORLINE_PAYLOAD_ENABLED = false;

// TIMEBASE_FRAGMENT must stay comfortably below the Payload arena ceiling.
// Keep the 1 Hz fragment as the durable science spine and move/omit bulky
// courtroom detail.  Focused reports and TIMEBASE_FORENSICS carry the richer
// diagnostic surfaces while the fragment preserves the values needed by panels
// and raw_nanoseconds-style reports.
static constexpr bool TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED = true;
static constexpr bool TIMEBASE_FRAGMENT_PUBLISH_PREDICTION_ENABLED = false;


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
static constexpr uint32_t TIMEBASE_BUILD_STAGE_FLASH_CUT_GATE = 13;
static constexpr uint32_t TIMEBASE_BUILD_STAGE_RECOVER_REATTACH_GATE = 14;
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
static uint32_t g_timebase_flash_cut_gate_count = 0;
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
static uint32_t g_timebase_forensics_disabled_count = 0;
static uint32_t g_timebase_forensics_minimal_count = 0;
static uint32_t g_timebase_forensics_micro_raw_count = 0;
static uint32_t g_timebase_forensics_slim_count = 0;

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

// START handoff diagnostics.  These counters prove whether Beta captured the
// campaign-public zero offset from a row whose Alpha OCXO PPS projection was
// valid for the current PPS/VCLOCK identity.  A public-origin flag alone is
// not enough: the current row must also be authored from the PPS-row OCXO
// projection species, not from a fallback measured-edge surface.
static uint32_t g_start_handoff_check_count = 0;
static uint32_t g_start_handoff_wait_origin_count = 0;
static uint32_t g_start_handoff_wait_projection_count = 0;
static uint32_t g_start_handoff_commit_count = 0;
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
// These counters are Beta-local forensics only.  They do not authorize clock
// values; they explain why START was held or released.
static constexpr uint32_t CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS = 1U;
// Once PhaseLedger is the standard authority, do not let the science row
// silently fall back to integer CounterLedger quantization.  A missing
// refined interval is a startup/maturity fact and should publish as missing
// science rather than contaminating Welford/PPB with 100 ns stair-steps.
static constexpr bool CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL = true;
static uint32_t g_start_phaseledger_check_count = 0;
static uint32_t g_start_phaseledger_ready_count = 0;
static uint32_t g_start_phaseledger_wait_count = 0;
static uint32_t g_start_phaseledger_wait_snapshot_count = 0;
static uint32_t g_start_phaseledger_wait_capture_count = 0;
static uint32_t g_start_phaseledger_wait_integer_interval_count = 0;
static uint32_t g_start_phaseledger_wait_phase_count = 0;
static uint32_t g_start_phaseledger_wait_phase_lag_count = 0;
static uint32_t g_start_phaseledger_wait_refined_count = 0;
static uint32_t g_start_phaseledger_wait_refined_interval_count = 0;
static uint32_t g_start_phaseledger_wait_sequence_count = 0;
static uint32_t g_phaseledger_science_missing_refined_interval_count = 0;
static uint32_t g_phaseledger_science_missing_ocxo1_count = 0;
static uint32_t g_phaseledger_science_missing_ocxo2_count = 0;
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

static bool     g_timebase_last_ocxo1_pps_projected = false;
static bool     g_timebase_last_ocxo2_pps_projected = false;
static bool     g_timebase_last_ocxo1_pps_residual_valid = false;
static bool     g_timebase_last_ocxo2_pps_residual_valid = false;

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
static bool     g_flash_cut_last_dac1_ok = true;
static bool     g_flash_cut_last_dac2_ok = true;
static bool     g_flash_cut_last_servo_mode_supplied = false;
static servo_mode_t g_flash_cut_last_servo_mode = servo_mode_t::OFF;
static char     g_flash_cut_last_status[48] = {0};

// ============================================================================
// SYSTEM feature status — CLOCKS/Beta-owned readiness surfaces
// ============================================================================
//
// These are publication/science-pipeline facts.  They remain observational in
// this migration step; START/ZERO/RECOVER do not branch on them yet.

static system_feature_status_t g_clocks_feature_science_residuals =
    system_feature_status_t::ANOMALY;
static system_feature_status_t g_clocks_feature_timebase_publication =
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

static void clocks_beta_features_mark_initializing(void) {
  clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                 g_clocks_feature_science_residuals,
                                 system_feature_status_t::INITIALIZING,
                                 true);
  clocks_beta_feature_set_cached("TIMEBASE_PUBLICATION",
                                 g_clocks_feature_timebase_publication,
                                 system_feature_status_t::INITIALIZING,
                                 true);
}

void clocks_beta_features_init(void) {
  clocks_beta_cold_diagnostics_init();
  clocks_beta_features_mark_initializing();
}

// ============================================================================
// FEATURE_STATUS-driven campaign gate
// ============================================================================
//
// Pi SYSTEM now republishes the unified system readiness tree as FEATURE_STATUS.
// Beta consumes that stream directly, but the Teensy-side START gate must only
// depend on Teensy-owned readiness surfaces.
//
// Pi SYSTEM/CLOCKS remains responsible for PI/GNSS/environment/battery/network
// preflight before it sends CLOCKS.START or CLOCKS.RECOVER.  Duplicating those
// PI-owned gates inside Teensy CLOCKS creates a split-brain failure mode: Pi can
// observe all prerequisites as open while the Teensy rejects START because its
// cached FEATURE_STATUS snapshot lacks, or has stale status for, PI.GNSS.REPORT.
//
// Intentional non-requirements:
//   SCIENCE_RESIDUALS and TIMEBASE_PUBLICATION are not START-gate inputs here.
//   They are campaign products.  Requiring them before the first campaign row
//   would create a cold-boot deadlock.  They remain published feature-health
//   surfaces, but not admission prerequisites.
//
//   FLOORLINE, QTIMER_DWT_RULER, STATIC_PREDICTION, SCIENCE_RESIDUALS,
//   and TIMEBASE_PUBLICATION are also not admission prerequisites in this
//   phase.  They are witness/product surfaces whose definitions can legitimately
//   be INITIALIZING immediately after STOP/RECOVER or after a transport-induced
//   campaign abort.  Requiring them before START creates a trap: the campaign
//   cannot start because the start-prologue evidence is not yet mature, while
//   the evidence cannot mature because the campaign cannot start.
//
//   Campaign admission therefore remains attached only to custody/identity
//   rails: PPS/VCLOCK authority, QTimer counter custody, Counter32 lineage,
//   DWT calibration, SmartZero, Alpha epoch, and OCXO public origin.

struct campaign_feature_gate_requirement_t {
  const char* label;
  const char* host;
  const char* subsystem;
  const char* feature;
};

static constexpr campaign_feature_gate_requirement_t
    CAMPAIGN_FEATURE_GATE_REQUIREMENTS[] = {
  { "T_FEATURE",   "TEENSY", "SYSTEM",    "FEATURE_STATUS" },
  { "PPS/V_AUTH",  "TEENSY", "INTERRUPT", "PPS_VCLOCK_AUTHORITY" },
  { "QTIMER_CNT",  "TEENSY", "INTERRUPT", "QTIMER_COUNTER_CUSTODY" },
  { "CTR32_LINE",  "TEENSY", "INTERRUPT", "COUNTER32_LINEAGE" },
  { "DWT_CAL",     "TEENSY", "CLOCKS",    "DWT_CALIBRATION" },
  { "SMARTZERO",   "TEENSY", "CLOCKS",    "SMARTZERO" },
  { "ALPHA_EPOCH", "TEENSY", "CLOCKS",    "ALPHA_EPOCH" },
  { "OCXO_ORIGIN", "TEENSY", "CLOCKS",    "OCXO_PUBLIC_ORIGIN" },
};

static volatile bool     g_campaign_feature_gate_seen = false;
static volatile bool     g_campaign_feature_gate_open = false;
static volatile uint32_t g_campaign_feature_gate_update_count = 0;
static volatile uint32_t g_campaign_feature_gate_transition_count = 0;
static volatile uint32_t g_campaign_feature_gate_required_count =
    (uint32_t)(sizeof(CAMPAIGN_FEATURE_GATE_REQUIREMENTS) /
               sizeof(CAMPAIGN_FEATURE_GATE_REQUIREMENTS[0]));
static char g_campaign_feature_gate_reason[128] =
    "FEATURE_STATUS not yet received";
static char g_campaign_feature_gate_first_problem[32] = "";
static char g_campaign_feature_gate_static_prediction_status[24] = "---";

static const char* feature_status_lookup(const Payload& root,
                                         const char* host,
                                         const char* subsystem,
                                         const char* feature) {
  const Payload host_payload = root.getPayload(host);
  if (host_payload.empty()) return nullptr;

  const Payload subsystem_payload = host_payload.getPayload(subsystem);
  if (subsystem_payload.empty()) return nullptr;

  return subsystem_payload.getString(feature);
}

static bool feature_status_is_nominal(const char* status) {
  return status && strcasecmp(status, "NOMINAL") == 0;
}

static void campaign_feature_gate_set_reason(const char* reason,
                                             const char* first_problem = nullptr) {
  safeCopy(g_campaign_feature_gate_reason,
           sizeof(g_campaign_feature_gate_reason),
           reason ? reason : "");
  safeCopy(g_campaign_feature_gate_first_problem,
           sizeof(g_campaign_feature_gate_first_problem),
           first_problem ? first_problem : "");
}

static void campaign_feature_gate_recompute(const Payload& root) {
  const char* static_pred_status =
      feature_status_lookup(root, "TEENSY", "CLOCKS", "STATIC_PREDICTION");
  safeCopy(g_campaign_feature_gate_static_prediction_status,
           sizeof(g_campaign_feature_gate_static_prediction_status),
           static_pred_status ? static_pred_status : "---");

  bool ready = true;
  const campaign_feature_gate_requirement_t* failed = nullptr;

  for (const auto& req : CAMPAIGN_FEATURE_GATE_REQUIREMENTS) {
    const char* status =
        feature_status_lookup(root, req.host, req.subsystem, req.feature);
    if (!feature_status_is_nominal(status)) {
      ready = false;
      failed = &req;
      break;
    }
  }

  if (g_campaign_feature_gate_open != ready) {
    g_campaign_feature_gate_transition_count++;
  }

  g_campaign_feature_gate_open = ready;

  if (ready) {
    campaign_feature_gate_set_reason("NOMINAL");
  } else if (failed) {
    char reason[96];
    snprintf(reason, sizeof(reason), "%s not NOMINAL", failed->label);
    campaign_feature_gate_set_reason(reason, failed->label);
  } else {
    campaign_feature_gate_set_reason("campaign feature gate closed");
  }
}

static void on_feature_status(const Payload& payload) {
  g_campaign_feature_gate_seen = true;
  g_campaign_feature_gate_update_count++;
  campaign_feature_gate_recompute(payload);
}

static bool campaign_feature_gate_open(void) {
  return g_campaign_feature_gate_seen && g_campaign_feature_gate_open;
}

static FLASHMEM void payload_add_campaign_feature_gate(Payload& p) {
  p.add("campaign_gate_source", "FEATURE_STATUS");
  p.add("campaign_gate_open", campaign_feature_gate_open());
  p.add("campaign_gate_seen", (bool)g_campaign_feature_gate_seen);
  p.add("campaign_gate_reason", g_campaign_feature_gate_reason);
  p.add("campaign_gate_first_problem", g_campaign_feature_gate_first_problem);
  p.add("campaign_gate_update_count",
        (uint32_t)g_campaign_feature_gate_update_count);
  p.add("campaign_gate_transition_count",
        (uint32_t)g_campaign_feature_gate_transition_count);
  p.add("campaign_gate_required_count",
        (uint32_t)g_campaign_feature_gate_required_count);
  p.add("campaign_gate_requires_floorline", false);
  p.add("campaign_gate_requires_qtimer_dwt_ruler", false);
  p.add("campaign_gate_requires_static_prediction", false);
  p.add("campaign_gate_static_prediction_status",
        g_campaign_feature_gate_static_prediction_status);
  p.add("campaign_gate_requires_science_residuals", false);
  p.add("campaign_gate_requires_timebase_publication", false);
}

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
    case TIMEBASE_BUILD_STAGE_FLASH_CUT_GATE: return "FLASH_CUT_GATE";
    case TIMEBASE_BUILD_STAGE_RECOVER_REATTACH_GATE: return "RECOVER_REATTACH_GATE";
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
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_COMPACT = 1U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_SUMMARY = 2U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_RECOVERY = 3U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_STACK = 4U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_FORENSICS = 5U;
static constexpr uint32_t CLOCKS_STACK_CONTEXT_REPORT_FORENSICS_LANE = 6U;
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
    case CLOCKS_STACK_CONTEXT_REPORT_COMPACT: return "REPORT_COMPACT";
    case CLOCKS_STACK_CONTEXT_REPORT_SUMMARY: return "REPORT_SUMMARY";
    case CLOCKS_STACK_CONTEXT_REPORT_RECOVERY: return "REPORT_RECOVERY";
    case CLOCKS_STACK_CONTEXT_REPORT_STACK: return "REPORT_STACK";
    case CLOCKS_STACK_CONTEXT_REPORT_FORENSICS: return "REPORT_FORENSICS";
    case CLOCKS_STACK_CONTEXT_REPORT_FORENSICS_LANE: return "REPORT_FORENSICS_LANE";
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

static FLASHMEM uint32_t clocks_stack_witness_bytes_from_top(uint32_t sp) {
  const uint32_t dtcm_top = clocks_stack_witness_observed_dtcm_top(
      clocks_stack_witness_sp(), sp, 0U);
  return clocks_stack_witness_bytes_below_dtcm_top(sp, dtcm_top);
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
// Fixed TIMEBASE row burial is retired.  Campaign admission is now gated by
// Pi/Teensy readiness surfaces; once START/RECOVER is armed, the first public
// TIMEBASE pair is expected to be the first authoritative row.
//
// START may still wait before publishing if the live PPS-row OCXO projection
// needed to capture campaign-public zero has not yet arrived.  That is a
// pre-publication handoff wait, not skipped campaign time: campaign_seconds
// does not advance and no canonical TIMEBASE identities are hidden.
//
// RECOVER publishes the next PPS/VCLOCK row after the recovered base count.
// Recovery science residuals may be quarantined for population hygiene, but
// the TIMEBASE row itself is no longer suppressed.

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
// not hidden TIMEBASE rows: campaign_seconds remains zero.  The prologue
// establishes a valid conceptual PPS0/bookend, then probes the next candidate
// so public PPS1 can carry a fully qualified Delta/FloorLine science sample
// and enter Welford as n=1.
static constexpr uint32_t CLOCKS_START_PROLOGUE_FIT_ENDPOINT_GATE_CYCLES = 16U;
static constexpr uint32_t CLOCKS_START_PROLOGUE_MAX_PRIVATE_CANDIDATES = 0xFFFFFFFFUL;
// Bound the purely-private START handoff.  This is not science-row burial; it
// is a launch-acquisition watchdog so a failed OCXO public-origin/projection
// proof becomes a local aborted START instead of 90 seconds of TIMEBASE silence.
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
static volatile uint32_t g_start_prologue_pps0_v_fl = 0;
static volatile uint32_t g_start_prologue_pps0_o1_obs = 0;
static volatile uint32_t g_start_prologue_pps0_o1_fl = 0;
static volatile uint32_t g_start_prologue_pps0_o2_obs = 0;
static volatile uint32_t g_start_prologue_pps0_o2_fl = 0;

// START private-PPS0 continuity witness.  These are report-only counters and
// last-decision fields for the observed-DWT rail gate that decides whether a
// private PPS0 bookend is mature enough to let the next candidate become
// public PPS1.  In PhaseLedger mode the DWT rail remains only a launch witness;
// it does not author the public OCXO nanosecond clock.
static uint32_t g_start_prologue_continuity_check_count = 0;
static uint32_t g_start_prologue_continuity_pass_count = 0;
static uint32_t g_start_prologue_continuity_reject_count = 0;
static bool     g_start_prologue_last_continuity_ok = false;
static uint32_t g_start_prologue_last_reference_interval = 0;
static uint32_t g_start_prologue_last_vclock_interval = 0;
static uint32_t g_start_prologue_last_ocxo1_interval = 0;
static uint32_t g_start_prologue_last_ocxo2_interval = 0;
static int32_t  g_start_prologue_last_reference_minus_pps0 = 0;
static int32_t  g_start_prologue_last_vclock_minus_pps0 = 0;
static int32_t  g_start_prologue_last_ocxo1_minus_pps0 = 0;
static int32_t  g_start_prologue_last_ocxo2_minus_pps0 = 0;

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
// promoted to PPS_COUNTERLEDGER.  In traditional mode they let TIMEBASE carry
// a campaign-aligned CounterLedger candidate beside the existing projection/
// Delta path so long runs can prove whether the integer PPS-sampled ledger is
// stable.
static int64_t g_campaign_public_counterledger_ocxo1_offset = 0;
static int64_t g_campaign_public_counterledger_ocxo2_offset = 0;

// Canonical OCXO residual rendering.  Delta Cycles is now the authority:
// each OCXO observed one-second DWT interval is compared against the delayed
// GNSS/PPS one-second DWT interval covering the same physical second.  The
// public pps_residual object remains a compatibility rendering of the
// canonical Delta result:
//
//   delta:         fast = ref_gnss_cycles - ocxo_observed_cycles
//   pps_residual:  gnss_interval = 1e9, clock_interval = 1e9 - fast_ns
//
// FloorLine and projected-GNSS residuals remain preserved under science.delta_*
// and science.traditional_* for courtroom/report comparison only.
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

struct floorline_science_totals_t {
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

static floorline_science_totals_t g_floorline_science_ocxo1 DMAMEM = {};
static floorline_science_totals_t g_floorline_science_ocxo2 DMAMEM = {};

struct delta_residual_reference_t {
  bool     captured = false;
  bool     gnss_valid = false;
  bool     raw_valid = false;
  bool     floorline_valid = false;
  uint32_t public_count = 0;
  uint32_t gnss_interval_cycles = 0;
  uint32_t raw_interval_cycles = 0;
  uint32_t floorline_interval_cycles = 0;
  uint32_t floorline_fit_interval_cycles = 0;
  int32_t  floorline_fit_minus_endpoint_cycles = 0;
};

struct delta_residual_bookend_t {
  bool     floorline_prev_dwt_valid = false;
  uint32_t floorline_prev_dwt = 0;
};

static delta_residual_reference_t g_delta_previous_vclock_reference DMAMEM = {};
static delta_residual_bookend_t g_delta_vclock_bookends DMAMEM = {};
static delta_residual_bookend_t g_delta_ocxo1_bookends DMAMEM = {};
static delta_residual_bookend_t g_delta_ocxo2_bookends DMAMEM = {};

// RAM1 relief scratch.  These are per-Beta-cycle temporary witnesses that used
// to be automatic locals in START/prologue and 1 Hz publication paths.  Keeping
// them in RAM2 avoids compiler-generated stack memset in the same upper-DTCM
// neighborhood reported by CrashReport.
static clocks_alpha_lane_forensics_t g_beta_start_vclock_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t g_beta_start_ocxo1_forensics DMAMEM = {};
static clocks_alpha_lane_forensics_t g_beta_start_ocxo2_forensics DMAMEM = {};
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
static clocks_static_prediction_snapshot_t g_beta_pps_cycle_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_ocxo1_cycle_prediction DMAMEM = {};
static clocks_static_prediction_snapshot_t g_beta_ocxo2_cycle_prediction DMAMEM = {};

struct clock_science_row_t {
  bool     valid = false;
  bool     antecedents_complete = false;
  uint32_t clock_id = 0;
  uint32_t public_count = 0;

  uint32_t pps_vclock_dwt_at_edge = 0;
  uint64_t pps_vclock_gnss_ns_at_edge = 0;
  uint32_t projection_cps_cycles = 0;

  bool     vclock_floorline_valid = false;
  uint32_t vclock_floorline_dwt_at_edge = 0;
  uint32_t vclock_floorline_interval_cycles = 0;

  bool     clock_floorline_valid = false;
  uint32_t clock_floorline_dwt_at_edge = 0;
  uint32_t clock_floorline_interval_cycles = 0;
  uint32_t clock_floorline_endpoint_interval_cycles = 0;
  uint32_t clock_published_dwt_at_edge = 0;
  uint32_t clock_raw_dwt_at_edge = 0;
  uint32_t clock_observed_interval_cycles = 0;
  uint32_t clock_effective_interval_cycles = 0;
  int32_t  published_minus_floorline_cycles = 0;
  int32_t  raw_minus_floorline_cycles = 0;
  int32_t  floorline_minus_observed_interval_cycles = 0;

  // Delta residuals: OCXO one-second DWT interval minus the delayed
  // GNSS/PPS one-second DWT interval that covers the same physical second.
  // The *_residual_cycles fields are Dave-subtraction signed
  // (clock_interval - reference_interval).  The *_fast_residual_* mirrors
  // the project-wide sign convention: positive means the clock is fast.
  bool     delta_raw_valid = false;
  bool     delta_floorline_valid = false;
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
  uint32_t delta_floorline_reference_interval_cycles = 0;
  uint32_t delta_floorline_clock_interval_cycles = 0;
  int64_t  delta_floorline_residual_cycles = 0;
  int64_t  delta_floorline_fast_residual_cycles = 0;
  int64_t  delta_floorline_residual_ns = 0;
  int64_t  delta_floorline_fast_residual_ns = 0;
  double   delta_floorline_residual_ns_exact = 0.0;
  double   delta_floorline_fast_residual_ns_exact = 0.0;
  int64_t  delta_raw_fast_minus_traditional_ns = 0;
  int64_t  delta_floorline_fast_minus_traditional_ns = 0;
  int64_t  delta_raw_fast_minus_floorline_fast_ns = 0;

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
static floorline_science_totals_t g_beta_probe_floorline_o1 DMAMEM = {};
static floorline_science_totals_t g_beta_probe_floorline_o2 DMAMEM = {};

static void delta_residual_state_reset(void) {
  g_delta_previous_vclock_reference = delta_residual_reference_t{};
  g_delta_vclock_bookends = delta_residual_bookend_t{};
  g_delta_ocxo1_bookends = delta_residual_bookend_t{};
  g_delta_ocxo2_bookends = delta_residual_bookend_t{};
}

static void floorline_science_totals_reset(void) {
  g_floorline_science_ocxo1 = floorline_science_totals_t{};
  g_floorline_science_ocxo2 = floorline_science_totals_t{};
  delta_residual_state_reset();
}

// RECOVER deliberately cuts the campaign publication stream while preserving
// the installed service epoch.  Alpha re-primes OCXO edge state at the RECOVER
// gate; Beta additionally quarantines the first published science residual rows
// so no Welford sample can be formed from a pre-recovery previous edge or a
// one-edge bridge warm-up artifact.  TIMEBASE rows themselves are not hidden.
static constexpr uint32_t CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS = 2U;
static uint32_t g_science_residual_quarantine_remaining = 0;
static uint32_t g_science_residual_quarantine_begin_count = 0;
static uint32_t g_science_residual_quarantine_consumed_count = 0;
static uint32_t g_science_residual_quarantine_last_public_count = 0;

// RECOVER OCXO reattachment gate.  Alpha deliberately cuts OCXO measurement
// custody during warm recovery.  Beta initially treats recovered candidates as
// private elapsed seconds while waiting for both OCXO lanes to prove fresh
// reattachment evidence.  This gate must be finite: after timeout, campaign
// publication resumes in degraded mode and OCXO science remains quarantined/
// invalid until PhaseLedger/reattach evidence catches up.
static constexpr uint32_t CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES = 32U;
// Clean-recovery doctrine is now split across Teensy and Pi:
//   * Teensy must not hold publication forever while waiting for OCXO
//     reattachment; after the bounded private hold it releases degraded
//     public rows so the Pi can observe liveness and keep polling recovery
//     state.
//   * Pi clean-recovery logic discards those transitional/degraded pairs
//     until REPORT_RECOVERY and the fragment science rows prove clean.
// This preserves the no-persist-frozen-OCXO rule without converting a slow
// reattach into total TIMEBASE_FRAGMENT silence.
static constexpr bool     CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED = true;
static volatile bool     g_recover_reattach_active = false;
static volatile bool     g_recover_reattach_degraded_active = false;
static uint32_t          g_recover_reattach_begin_count = 0;
static uint32_t          g_recover_reattach_hold_count = 0;
static uint32_t          g_recover_reattach_release_count = 0;
static uint32_t          g_recover_reattach_timeout_count = 0;
static uint32_t          g_recover_reattach_degraded_release_count = 0;
static uint32_t          g_recover_reattach_degraded_clear_count = 0;
static uint32_t          g_recover_reattach_degraded_public_row_count = 0;
static uint32_t          g_recover_reattach_degraded_science_suppressed_count = 0;
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

// Recovery request flight recorder for Pi-side polling.  These values make
// REPORT_RECOVERY useful while core.py waits for the first public pair.
static uint32_t          g_recover_request_count = 0;
static uint64_t          g_recover_last_base_count = 0;
static uint64_t          g_recover_last_expected_first_public_count = 0;
static uint64_t          g_recover_last_base_gnss_ns = 0;
static uint64_t          g_recover_last_base_dwt_ns = 0;
static uint64_t          g_recover_last_base_ocxo1_ns = 0;
static uint64_t          g_recover_last_base_ocxo2_ns = 0;

// RECOVER presentation continuity.  Reattachment proves fresh OCXO custody,
// but the raw CounterLedger/PhaseLedger intercept can differ from the
// Pi-projected campaign ledger by a few hundred ns.  That is lawful evidence,
// but it makes panel-facing total TAU/PPB look like it restarted.  At the
// first clean recovered public row, apply one signed presentation offset so
// public OCXO ns lands on the recovered campaign ratio.  One-second science,
// Welfords, and servo NOW/MEAN inputs continue to consume fresh OCXO evidence.
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

static void pps_interval_residuals_reset(void) {
  floorline_science_totals_reset();
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

static uint64_t floorline_render_legacy_clock_interval_ns(
    const clock_science_row_t& row) {
  if (!row.valid) return 0ULL;

  if (clocks_ocxo_counterledger_mode_enabled()) {
    // CounterLedger residuals are clockface elapsed time: positive fast means
    // the OCXO ledger advanced more than the GNSS second.
    return row.clock_interval_ns;
  }

  // Traditional Delta rendering: positive fast means the physical clock period
  // was shorter than the GNSS reference second.
  const int64_t rendered =
      (int64_t)CLOCKS_BETA_NS_PER_SECOND - row.fast_residual_ns;
  return (rendered > 0) ? (uint64_t)rendered : 0ULL;
}

static uint64_t floorline_render_public_clock_ns(
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

static pps_interval_residuals_t floorline_interval_residuals_update(
    uint32_t public_count,
    const clock_science_row_t& ocxo1_science,
    const clock_science_row_t& ocxo2_science) {
  pps_interval_residuals_t r{};
  r.public_count = public_count;

  r.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;

  if (ocxo1_science.valid) {
    r.ocxo1_valid = true;
    r.ocxo1_interval_ns =
        floorline_render_legacy_clock_interval_ns(ocxo1_science);
    r.ocxo1_fast_residual_ns = ocxo1_science.fast_residual_ns;
  }

  if (ocxo2_science.valid) {
    r.ocxo2_valid = true;
    r.ocxo2_interval_ns =
        floorline_render_legacy_clock_interval_ns(ocxo2_science);
    r.ocxo2_fast_residual_ns = ocxo2_science.fast_residual_ns;
  }

  if (r.ocxo1_valid && r.ocxo2_valid) {
    clocks_beta_feature_set_cached("SCIENCE_RESIDUALS",
                                   g_clocks_feature_science_residuals,
                                   system_feature_status_t::NOMINAL);
  }
  return r;
}

static uint64_t current_raw_gnss_ns(void) {
  // At the selected PPS/VCLOCK edge, GNSS time is identity, not discovery.
  // The DWT projection path remains a forensic self-check in
  // TIMEBASE_FORENSICS.pps_vclock_edge; it must not author the public GNSS
  // ledger because doing so turns bridge rounding/projection error into a
  // common-mode OCXO residual.
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
  recover_continuity_set_reason("armed_for_first_clean_public_row");
}

static void recover_continuity_align_reset(const char* reason) {
  g_recover_continuity_align_pending = false;
  g_recover_continuity_align_requested_public_count = 0;
  recover_continuity_set_reason(reason ? reason : "reset");
}

static void recover_continuity_align_if_pending(uint32_t public_count,
                                                uint64_t public_gnss_ns) {
  if (!g_recover_continuity_align_pending) return;

  const uint64_t raw_o1 = current_raw_ocxo1_ns();
  const uint64_t raw_o2 = current_raw_ocxo2_ns();
  const uint64_t target_o1 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo1_ns);
  const uint64_t target_o2 =
      campaign_recover_project_ocxo_to_public_gnss(public_gnss_ns,
                                                   recover_ocxo2_ns);

  if (raw_o1 == 0ULL || raw_o2 == 0ULL ||
      target_o1 == 0ULL || target_o2 == 0ULL) {
    g_recover_continuity_align_pending = false;
    g_recover_continuity_align_failure_count++;
    g_recover_continuity_align_last_public_count = public_count;
    g_recover_continuity_ocxo1_target_ns = target_o1;
    g_recover_continuity_ocxo2_target_ns = target_o2;
    recover_continuity_set_reason("missing_raw_or_target");
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

  const uint64_t raw_c1 = current_raw_counterledger_ns(time_clock_id_t::OCXO1);
  const uint64_t raw_c2 = current_raw_counterledger_ns(time_clock_id_t::OCXO2);
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
         s.phase_pps_sequence != 0U &&
         s.phase_pps_sequence <= s.pps_sequence;
}

static bool campaign_start_phaseledger_lag_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
  return campaign_start_phaseledger_phase_ready(s) &&
         s.phase_lag_pps <= CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS;
}

static bool campaign_start_phaseledger_refined_ready(
    const clocks_alpha_ocxo_counterledger_snapshot_t& s) {
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

static void campaign_start_phaseledger_count_wait_reason(
    bool ocxo1_snapshot_ok,
    const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo1,
    bool ocxo2_snapshot_ok,
    const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo2,
    bool sequence_aligned) {
  if (!ocxo1_snapshot_ok || !ocxo2_snapshot_ok ||
      !ocxo1.valid || !ocxo2.valid ||
      !ocxo1.initialized || !ocxo2.initialized) {
    g_start_phaseledger_wait_snapshot_count++;
    return;
  }
  if (!campaign_start_phaseledger_capture_ready(ocxo1) ||
      !campaign_start_phaseledger_capture_ready(ocxo2)) {
    g_start_phaseledger_wait_capture_count++;
    return;
  }
  if (!campaign_start_phaseledger_integer_interval_ready(ocxo1) ||
      !campaign_start_phaseledger_integer_interval_ready(ocxo2)) {
    g_start_phaseledger_wait_integer_interval_count++;
    return;
  }
  if (!campaign_start_phaseledger_phase_ready(ocxo1) ||
      !campaign_start_phaseledger_phase_ready(ocxo2)) {
    g_start_phaseledger_wait_phase_count++;
    return;
  }
  if (!campaign_start_phaseledger_lag_ready(ocxo1) ||
      !campaign_start_phaseledger_lag_ready(ocxo2)) {
    g_start_phaseledger_wait_phase_lag_count++;
    return;
  }
  if (!campaign_start_phaseledger_refined_ready(ocxo1) ||
      !campaign_start_phaseledger_refined_ready(ocxo2)) {
    g_start_phaseledger_wait_refined_count++;
    return;
  }
  if (!campaign_start_phaseledger_refined_interval_ready(ocxo1) ||
      !campaign_start_phaseledger_refined_interval_ready(ocxo2)) {
    g_start_phaseledger_wait_refined_interval_count++;
    return;
  }
  if (!sequence_aligned) {
    g_start_phaseledger_wait_sequence_count++;
    return;
  }
}

static bool campaign_start_counterledger_maturity_ready(void) {
  g_start_phaseledger_check_count++;

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
    g_start_phaseledger_ready_count++;
    campaign_start_phaseledger_set_reason("phaseledger_mature");
    return true;
  }

  g_start_phaseledger_wait_count++;
  campaign_start_phaseledger_count_wait_reason(ocxo1_snapshot_ok, ocxo1,
                                               ocxo2_snapshot_ok, ocxo2,
                                               sequence_aligned);

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
  g_start_handoff_check_count++;

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
      g_start_handoff_wait_origin_count++;
    }
    if (!counterledger_ready) {
      // Historical counter name retained for report compatibility: in
      // CounterLedger/PhaseLedger mode this means the PPS-sampled hardware
      // ledger, capture custody, or refined PhaseLedger interval is not mature.
      g_start_handoff_wait_projection_count++;
    }
    return g_start_handoff_last_ready;
  }

  g_start_handoff_last_ready =
      origin_ready && ocxo1_projection_ready && ocxo2_projection_ready;

  if (!origin_ready) {
    g_start_handoff_wait_origin_count++;
  }
  if (!ocxo1_projection_ready || !ocxo2_projection_ready) {
    g_start_handoff_wait_projection_count++;
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
  g_start_prologue_pps0_v_fl = 0;
  g_start_prologue_pps0_o1_obs = 0;
  g_start_prologue_pps0_o1_fl = 0;
  g_start_prologue_pps0_o2_obs = 0;
  g_start_prologue_pps0_o2_fl = 0;
  g_start_prologue_last_continuity_ok = false;
  g_start_prologue_last_reference_interval = 0;
  g_start_prologue_last_vclock_interval = 0;
  g_start_prologue_last_ocxo1_interval = 0;
  g_start_prologue_last_ocxo2_interval = 0;
  g_start_prologue_last_reference_minus_pps0 = 0;
  g_start_prologue_last_vclock_minus_pps0 = 0;
  g_start_prologue_last_ocxo1_minus_pps0 = 0;
  g_start_prologue_last_ocxo2_minus_pps0 = 0;
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

static bool campaign_start_prologue_should_hold(void);
static void flash_cut_clear_pending(void);
static void ocxo_dac_pacing_abort_all(void);
static FLASHMEM void recover_reattach_begin(void);
static FLASHMEM void recover_reattach_reset(const char* reason);

static void campaign_warmup_begin(campaign_warmup_mode_t mode) {
  g_campaign_warmup_suppressed_total = 0;

  if (mode == campaign_warmup_mode_t::RECOVER) {
    // RECOVER no longer buries fixed rows, but it must prove OCXO
    // reattachment before public TIMEBASE resumes.  Install recovered offsets
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
    recover_reattach_reset("start_lifecycle");
    // START now has a science prologue instead of fixed row burial.  The
    // prologue consumes private candidate rows until it has established an
    // internal PPS 0 and the next candidate can publish as science-complete
    // public PPS 1.  campaign_seconds remains zero while this mode is active.
    // While this private window is open, INTERRUPT relaxes non-poisonous
    // publication courts so Alpha can build OCXO public-origin evidence.
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
  return g_campaign_warmup_mode != campaign_warmup_mode_t::NONE ||
         g_recover_reattach_active;
}

static void recover_reattach_set_reason(const char* reason) {
  safeCopy(g_recover_reattach_last_reason,
           sizeof(g_recover_reattach_last_reason),
           reason ? reason : "recover_reattach");
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

  return ocxo1_snapshot_ok && ocxo2_snapshot_ok &&
         g_recover_reattach_last_ocxo1.ready &&
         g_recover_reattach_last_ocxo2.ready;
}
static FLASHMEM void recover_reattach_reset(const char* reason) {
  recover_continuity_align_reset(reason ? reason : "reattach_reset");
  g_recover_reattach_active = false;
  g_recover_reattach_degraded_active = false;
  g_recover_reattach_hidden_candidate_count = 0;
  g_recover_reattach_last_hidden_public_count = 0;
  g_recover_reattach_last_degraded_public_count = 0;
  g_recover_reattach_last_ocxo1 = clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_reattach_last_ocxo2 = clocks_alpha_recover_reattach_snapshot_t{};
  recover_reattach_set_reason(reason ? reason : "reset");
}

static FLASHMEM void recover_reattach_begin(void) {
  recover_continuity_align_reset("waiting_for_ocxo_reattach");
  g_recover_reattach_active = true;
  g_recover_reattach_degraded_active = false;
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
    g_recover_reattach_degraded_release_count++;
    g_recover_reattach_last_degraded_release_public_count =
        g_recover_reattach_last_release_public_count;
    g_recover_reattach_last_degraded_public_count =
        g_recover_reattach_last_release_public_count;
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
  } else {
    recover_continuity_align_arm();
    if (g_recover_reattach_degraded_active) {
      g_recover_reattach_degraded_active = false;
      g_recover_reattach_degraded_clear_count++;
    }
  }

  recover_reattach_set_reason(reason ? reason : "ocxo_reattach_release");
}

static bool campaign_warmup_consume_one_candidate_record(void) {
  if (!campaign_warmup_active()) return false;

  if (g_campaign_warmup_mode == campaign_warmup_mode_t::START) {
    if (campaign_start_prologue_should_hold()) {
      // Private prologue, not a public skipped row.  The candidate was either
      // used as internal PPS 0/bookend evidence or rejected before campaign
      // time began.  If the prologue timed out and aborted START, leave the
      // warmup mode cleared instead of re-arming the hold gate.
      if (campaign_warmup_active()) {
        g_campaign_warmup_remaining = 1;
      }
      return true;
    }

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

  // RECOVER and NONE both publish immediately under the no-burial doctrine.
  // RECOVER-specific OCXO reattachment is handled by recover_reattach_should_hold();
  // unlike START warmup, that gate has a finite degraded-release policy.
  g_campaign_warmup_mode = campaign_warmup_mode_t::NONE;
  g_campaign_warmup_remaining = 0;
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

static uint64_t campaign_public_dwt_total(void) {
  return campaign_public_from_offset(g_dwt_cycle_count_total,
                                     g_campaign_public_dwt_offset);
}

static uint64_t campaign_public_gnss_ns(void) {
  return campaign_public_from_offset(current_raw_gnss_ns(),
                                     g_campaign_public_gnss_offset);
}

static uint64_t campaign_public_ocxo1_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo1_ns(),
                                     g_campaign_public_ocxo1_offset);
}

static uint64_t campaign_public_ocxo2_ns(void) {
  return campaign_public_from_offset(current_raw_ocxo2_ns(),
                                     g_campaign_public_ocxo2_offset);
}

static void recover_reattach_advance_hidden_candidate(void) {
  const uint64_t public_gnss_ns = campaign_public_gnss_ns();
  const uint64_t public_dwt_total = campaign_public_dwt_total();
  const uint64_t public_ocxo1_measured_ns = campaign_public_ocxo1_measured_ns();
  const uint64_t public_ocxo2_measured_ns = campaign_public_ocxo2_measured_ns();

  if (public_gnss_ns != 0ULL) {
    campaign_seconds = public_gnss_ns / CLOCKS_BETA_NS_PER_SECOND;
  } else {
    campaign_seconds++;
  }

  dwt_cycle_count_total = public_dwt_total;
  gnss_raw_64 = public_gnss_ns / 100ULL;
  if (public_ocxo1_measured_ns != 0ULL) {
    ocxo1_measured_gnss_ticks_64 = public_ocxo1_measured_ns / 100ULL;
  }
  if (public_ocxo2_measured_ns != 0ULL) {
    ocxo2_measured_gnss_ticks_64 = public_ocxo2_measured_ns / 100ULL;
  }

  g_recover_reattach_hidden_candidate_count++;
  g_recover_reattach_last_hidden_public_count =
      (uint32_t)campaign_seconds;
  g_campaign_warmup_suppressed_total++;
  g_timebase_warmup_suppressed_count++;
}

static FLASHMEM bool recover_reattach_should_hold(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_SHOULD_HOLD);
  if (!g_recover_reattach_active) return false;

  if (recover_reattach_refresh_ready()) {
    recover_reattach_release("ocxo_reattach_ready", false);
    return false;
  }

  if (g_recover_reattach_hidden_candidate_count >=
      CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES) {
    g_recover_reattach_timeout_count++;
    if (CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED) {
      recover_reattach_release("ocxo_reattach_timeout_degraded_release", true);
      return false;
    }

    recover_reattach_set_reason("ocxo_reattach_timeout_still_holding");
    return true;
  }

  g_recover_reattach_hold_count++;
  recover_reattach_advance_hidden_candidate();

  if (!g_recover_reattach_last_ocxo1.ready &&
      !g_recover_reattach_last_ocxo2.ready) {
    recover_reattach_set_reason("waiting_for_both_ocxo_lanes");
  } else if (!g_recover_reattach_last_ocxo1.ready) {
    recover_reattach_set_reason("waiting_for_ocxo1_lane");
  } else if (!g_recover_reattach_last_ocxo2.ready) {
    recover_reattach_set_reason("waiting_for_ocxo2_lane");
  } else {
    recover_reattach_set_reason("waiting_for_ocxo_reattach");
  }

  return true;
}

static FLASHMEM bool recover_reattach_degraded_science_hold_active(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_RECOVER_DEGRADED_HOLD);
  if (!g_recover_reattach_degraded_active) return false;

  if (recover_reattach_refresh_ready()) {
    g_recover_reattach_degraded_active = false;
    g_recover_reattach_degraded_clear_count++;
    // The row that proves reattachment is still a boundary row: it may carry
    // PhaseLedger/CounterLedger state formed across the degraded window.
    // Start a real science quarantine here so Welford/PPB/servo do not consume
    // the first reattachment transient.
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
    recover_reattach_set_reason("ocxo_reattach_ready_after_degraded_release");
    return false;
  }

  g_recover_reattach_degraded_public_row_count++;
  g_recover_reattach_last_degraded_public_count =
      (uint32_t)campaign_seconds;
  recover_reattach_set_reason("degraded_publication_waiting_for_ocxo_reattach");
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
  row.delta_floorline_valid = false;
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
// STARTED/name-owned state before the first public TIMEBASE row exists;
// those pre-row tribunal verdicts must remain local diagnostics.
static volatile bool watchdog_campaign_publication_armed = false;

// Sticky local circuit breaker.  WATCHDOG_ANOMALY is not merely a message to
// the Pi; it is a Teensy-side campaign-continuity surrender.  Once latched,
// Beta refuses to publish any more TIMEBASE rows from the current campaign
// until an explicit lifecycle boundary clears the latch.
static volatile bool     watchdog_campaign_surrendered = false;
static volatile uint32_t watchdog_campaign_surrender_count = 0;
static volatile uint32_t watchdog_anomaly_suppressed_unarmed_count = 0;
static volatile uint32_t watchdog_anomaly_verbose_publish_count = 0;
static volatile uint32_t watchdog_anomaly_legacy_publish_count = 0;

// Live servo-mode command handoff.
//
// CALIBRATE_OCXO remains the START/RECOVER campaign parameter.  SERVOS is the
// live operator control surface: foreground command context records the desired
// mode, and Beta applies it at the next safe PPS campaign boundary before the
// servo input is selected for that row.
static volatile bool          request_servo_mode_change = false;
static volatile servo_mode_t  requested_servo_mode = servo_mode_t::OFF;
static uint32_t               g_servo_mode_request_count = 0;
static uint32_t               g_servo_mode_commit_count = 0;
static servo_mode_t           g_servo_mode_last_requested = servo_mode_t::OFF;
static servo_mode_t           g_servo_mode_last_committed = servo_mode_t::OFF;

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
  return watchdog_campaign_publication_armed &&
         !clocks_watchdog_publication_blocked() &&
         campaign_state == clocks_campaign_state_t::STARTED &&
         campaign_name[0] != '\0' &&
         campaign_seconds != 0ULL &&
         !request_start &&
         !request_stop &&
         !request_recover &&
         !request_zero &&
         !request_flash_cut &&
         !campaign_warmup_active();
}

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


struct recover_welford_state_t {
  bool has_dwt = false;
  bool has_vclock = false;
  bool has_ocxo1 = false;
  bool has_ocxo2 = false;
  bool has_pps_witness = false;
  bool has_ocxo1_dac = false;
  bool has_ocxo2_dac = false;
  uint32_t restored_lane_count = 0;
  welford_t dwt = {};
  welford_t vclock = {};
  welford_t ocxo1 = {};
  welford_t ocxo2 = {};
  welford_t pps_witness = {};
  welford_t ocxo1_dac = {};
  welford_t ocxo2_dac = {};
};

static recover_welford_state_t g_recover_welfords_pending DMAMEM = {};
static uint32_t g_recover_welford_capture_count = 0;
static uint32_t g_recover_welford_restore_count = 0;
static uint32_t g_recover_welford_last_restored_lane_count = 0;
static uint32_t g_welford_gap_advance_count = 0;
static uint32_t g_welford_gap_advance_last_public_count = 0;
static uint32_t g_welford_gap_advance_last_lane_count = 0;

static FLASHMEM void clocks_beta_cold_diagnostics_init(void) {
  g_start_phaseledger_last_ocxo1 = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_start_phaseledger_last_ocxo2 = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_counterledger_raw_scratch = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo1_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo2_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_recover_reattach_last_ocxo1 = clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_reattach_last_ocxo2 = clocks_alpha_recover_reattach_snapshot_t{};
  g_recover_welfords_pending = recover_welford_state_t{};
}

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

static bool welford_advance_missing_to_n(welford_t& w, uint64_t target_n) {
  if (target_n <= w.n) return false;

  const double sd = welford_stddev(w);
  if (w.n == 0ULL) {
    w.mean = 0.0;
    w.min_val = 0.0;
    w.max_val = 0.0;
  }

  w.n = target_n;
  w.m2 = (target_n >= 2ULL)
      ? (sd * sd * (double)(target_n - 1ULL))
      : 0.0;
  return true;
}

static void welfords_note_gap_advance(uint32_t public_count,
                                      uint32_t advanced) {
  if (advanced == 0U) return;
  g_welford_gap_advance_count++;
  g_welford_gap_advance_last_public_count = public_count;
  g_welford_gap_advance_last_lane_count = advanced;
}

static uint32_t welfords_advance_non_ocxo_gap_before_row(uint32_t public_count) {
  // TIMEBASE gaps are canonical campaign seconds.  DWT/VCLOCK/DAC surfaces
  // remain valid during recovery because public GNSS/DWT/VCLOCK time and DAC
  // intent continue to advance.  OCXO Welfords are different: they must not be
  // gap-advanced until the current row has valid OCXO science, otherwise a
  // degraded/recovery hold can make N march while no OCXO sample exists.
  const uint64_t before_row_n =
      (public_count > 0U) ? (uint64_t)(public_count - 1U) : 0ULL;

  uint32_t advanced = 0;
  if (welford_advance_missing_to_n(welford_dwt, before_row_n)) advanced++;
  if (welford_advance_missing_to_n(welford_vclock, before_row_n)) advanced++;
  if (welford_advance_missing_to_n(welford_ocxo1_dac, before_row_n)) advanced++;
  if (welford_advance_missing_to_n(welford_ocxo2_dac, before_row_n)) advanced++;

  welfords_note_gap_advance(public_count, advanced);
  return advanced;
}

static uint32_t welfords_advance_ocxo_gap_before_valid_row(
    uint32_t public_count,
    bool ocxo1_valid,
    bool ocxo2_valid) {
  // Delta/CounterLedger OCXO science is a one-second interval surface and
  // intentionally lags the public PPS identity by one row.  Advance its
  // cardinality only when that lane is about to accept a real sample.
  const uint64_t before_row_ocxo_n =
      (public_count > 1U) ? (uint64_t)(public_count - 2U) : 0ULL;

  uint32_t advanced = 0;
  if (ocxo1_valid &&
      welford_advance_missing_to_n(welford_ocxo1, before_row_ocxo_n)) {
    advanced++;
  }
  if (ocxo2_valid &&
      welford_advance_missing_to_n(welford_ocxo2, before_row_ocxo_n)) {
    advanced++;
  }

  welfords_note_gap_advance(public_count, advanced);
  return advanced;
}

static FLASHMEM void welford_restore_from_summary(welford_t& w,
                                         uint64_t n,
                                         double mean,
                                         double stddev,
                                         double min_val,
                                         double max_val) {
  if (n == 0ULL) {
    welford_reset(w);
    return;
  }

  const double sd = (stddev >= 0.0) ? stddev : -stddev;
  w.n = n;
  w.mean = mean;
  w.m2 = (n >= 2ULL) ? (sd * sd * (double)(n - 1ULL)) : 0.0;
  w.min_val = min_val;
  w.max_val = max_val;
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

  if (!payload.tryGetDouble(key, out)) {
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

static FLASHMEM bool payload_try_get_welford_object(const Payload& obj,
                                           const char* path,
                                           const char* lane,
                                           welford_t& out) {
  if (g_clocks_payload_numeric_integrity_failed) return false;

  uint64_t n = 0;
  double mean = 0.0;
  if (clocks_payload_try_get_u64_checked(obj, "n",
                                         "recovery_welford",
                                         path,
                                         lane,
                                         "n",
                                         n) != CLOCKS_PAYLOAD_FIELD_OK ||
      clocks_payload_try_get_double_checked(obj, "mean",
                                            "recovery_welford",
                                            path,
                                            lane,
                                            "mean",
                                            mean) != CLOCKS_PAYLOAD_FIELD_OK) {
    return false;
  }

  double stddev = 0.0;
  const clocks_payload_checked_status_t stddev_status =
      clocks_payload_try_get_double_checked(obj, "stddev",
                                            "recovery_welford",
                                            path,
                                            lane,
                                            "stddev",
                                            stddev);
  if (stddev_status == CLOCKS_PAYLOAD_FIELD_INVALID) {
    return false;
  }
  if (stddev_status == CLOCKS_PAYLOAD_FIELD_MISSING) {
    double stderr_value = 0.0;
    const clocks_payload_checked_status_t stderr_status =
        clocks_payload_try_get_double_checked(obj, "stderr",
                                              "recovery_welford",
                                              path,
                                              lane,
                                              "stderr",
                                              stderr_value);
    if (stderr_status == CLOCKS_PAYLOAD_FIELD_INVALID) {
      return false;
    }
    if (stderr_status == CLOCKS_PAYLOAD_FIELD_OK && n > 0ULL) {
      stddev = stderr_value * sqrt((double)n);
    }
  }

  double min_val = mean;
  double max_val = mean;
  const clocks_payload_checked_status_t min_status =
      clocks_payload_try_get_double_checked(obj, "min",
                                            "recovery_welford",
                                            path,
                                            lane,
                                            "min",
                                            min_val);
  if (min_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;
  const clocks_payload_checked_status_t max_status =
      clocks_payload_try_get_double_checked(obj, "max",
                                            "recovery_welford",
                                            path,
                                            lane,
                                            "max",
                                            max_val);
  if (max_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;

  welford_restore_from_summary(out, n, mean, stddev, min_val, max_val);
  return true;
}

static FLASHMEM bool payload_try_get_welford_flat(const Payload& args,
                                         const char* prefix,
                                         const char* path,
                                         welford_t& out) {
  if (g_clocks_payload_numeric_integrity_failed) return false;

  char key[64];

  uint64_t n = 0;
  snprintf(key, sizeof(key), "%s_welford_n", prefix);
  if (clocks_payload_try_get_u64_checked(args, key,
                                         "recovery_welford_flat",
                                         path,
                                         prefix,
                                         "n",
                                         n) != CLOCKS_PAYLOAD_FIELD_OK) {
    return false;
  }

  double mean = 0.0;
  snprintf(key, sizeof(key), "%s_welford_mean", prefix);
  if (clocks_payload_try_get_double_checked(args, key,
                                            "recovery_welford_flat",
                                            path,
                                            prefix,
                                            "mean",
                                            mean) != CLOCKS_PAYLOAD_FIELD_OK) {
    return false;
  }

  double stddev = 0.0;
  snprintf(key, sizeof(key), "%s_welford_stddev", prefix);
  const clocks_payload_checked_status_t stddev_status =
      clocks_payload_try_get_double_checked(args, key,
                                            "recovery_welford_flat",
                                            path,
                                            prefix,
                                            "stddev",
                                            stddev);
  if (stddev_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;
  if (stddev_status == CLOCKS_PAYLOAD_FIELD_MISSING) {
    double stderr_value = 0.0;
    snprintf(key, sizeof(key), "%s_welford_stderr", prefix);
    const clocks_payload_checked_status_t stderr_status =
        clocks_payload_try_get_double_checked(args, key,
                                              "recovery_welford_flat",
                                              path,
                                              prefix,
                                              "stderr",
                                              stderr_value);
    if (stderr_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;
    if (stderr_status == CLOCKS_PAYLOAD_FIELD_OK && n > 0ULL) {
      stddev = stderr_value * sqrt((double)n);
    }
  }

  double min_val = mean;
  double max_val = mean;
  snprintf(key, sizeof(key), "%s_welford_min", prefix);
  const clocks_payload_checked_status_t min_status =
      clocks_payload_try_get_double_checked(args, key,
                                            "recovery_welford_flat",
                                            path,
                                            prefix,
                                            "min",
                                            min_val);
  if (min_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;
  snprintf(key, sizeof(key), "%s_welford_max", prefix);
  const clocks_payload_checked_status_t max_status =
      clocks_payload_try_get_double_checked(args, key,
                                            "recovery_welford_flat",
                                            path,
                                            prefix,
                                            "max",
                                            max_val);
  if (max_status == CLOCKS_PAYLOAD_FIELD_INVALID) return false;

  welford_restore_from_summary(out, n, mean, stddev, min_val, max_val);
  return true;
}

static FLASHMEM bool payload_try_get_stats_welford(const Payload& root,
                                          const char* root_path,
                                          const char* stats_key,
                                          welford_t& out) {
  if (g_clocks_payload_numeric_integrity_failed) return false;

  const Payload stats = root.getPayload("stats");
  if (stats.empty()) return false;

  const Payload lane = stats.getPayload(stats_key);
  if (lane.empty()) return false;

  const char* path = root_path ? root_path : "root";

  const Payload nested = lane.getPayload("welford");
  if (!nested.empty()) {
    if (payload_try_get_welford_object(nested, path, stats_key, out)) {
      return true;
    }
    if (g_clocks_payload_numeric_integrity_failed) return false;
  }

  return payload_try_get_welford_object(lane, path, stats_key, out);
}

static FLASHMEM bool payload_try_get_stats_dac_welford(const Payload& root,
                                              const char* root_path,
                                              const char* dac_key,
                                              welford_t& out) {
  if (g_clocks_payload_numeric_integrity_failed) return false;

  const Payload stats = root.getPayload("stats");
  if (stats.empty()) return false;

  const Payload dac = stats.getPayload("dac");
  if (dac.empty()) return false;

  const Payload lane = dac.getPayload(dac_key);
  if (lane.empty()) return false;

  const char* path = root_path ? root_path : "root";
  return payload_try_get_welford_object(lane, path, dac_key, out);
}

static FLASHMEM bool payload_try_get_recovery_welford(const Payload& args,
                                             const char* flat_prefix,
                                             const char* stats_key,
                                             bool dac_stats,
                                             welford_t& out) {
  if (g_clocks_payload_numeric_integrity_failed) return false;

  if (payload_try_get_welford_flat(args, flat_prefix, "args", out)) return true;
  if (g_clocks_payload_numeric_integrity_failed) return false;
  if (dac_stats) {
    if (payload_try_get_stats_dac_welford(args, "args", stats_key, out)) return true;
  } else if (payload_try_get_stats_welford(args, "args", stats_key, out)) {
    return true;
  }
  if (g_clocks_payload_numeric_integrity_failed) return false;

  const Payload fragment = args.getPayload("fragment");
  if (!fragment.empty()) {
    if (payload_try_get_welford_flat(fragment, flat_prefix, "args.fragment", out)) return true;
    if (g_clocks_payload_numeric_integrity_failed) return false;
    if (dac_stats) {
      if (payload_try_get_stats_dac_welford(fragment, "args.fragment", stats_key, out)) return true;
    } else if (payload_try_get_stats_welford(fragment, "args.fragment", stats_key, out)) {
      return true;
    }
    if (g_clocks_payload_numeric_integrity_failed) return false;
  }

  const Payload payload = args.getPayload("payload");
  if (!payload.empty()) {
    if (payload_try_get_welford_flat(payload, flat_prefix, "args.payload", out)) return true;
    if (g_clocks_payload_numeric_integrity_failed) return false;
    if (dac_stats) {
      if (payload_try_get_stats_dac_welford(payload, "args.payload", stats_key, out)) return true;
    } else if (payload_try_get_stats_welford(payload, "args.payload", stats_key, out)) {
      return true;
    }
    if (g_clocks_payload_numeric_integrity_failed) return false;

    const Payload payload_fragment = payload.getPayload("fragment");
    if (!payload_fragment.empty()) {
      if (payload_try_get_welford_flat(payload_fragment, flat_prefix,
                                       "args.payload.fragment", out)) return true;
      if (g_clocks_payload_numeric_integrity_failed) return false;
      if (dac_stats) {
        return payload_try_get_stats_dac_welford(payload_fragment,
                                                "args.payload.fragment",
                                                stats_key,
                                                out);
      }
      return payload_try_get_stats_welford(payload_fragment,
                                          "args.payload.fragment",
                                          stats_key,
                                          out);
    }
  }

  return false;
}

static FLASHMEM bool recover_welford_capture_lane(const Payload& args,
                                         const char* flat_prefix,
                                         const char* stats_key,
                                         bool dac_stats,
                                         bool& has_lane,
                                         welford_t& dst,
                                         uint32_t& count) {
  has_lane = payload_try_get_recovery_welford(args,
                                             flat_prefix,
                                             stats_key,
                                             dac_stats,
                                             dst);
  if (has_lane) count++;
  return has_lane;
}

static FLASHMEM bool recover_welfords_capture(const Payload& args) {
  g_recover_welfords_pending = recover_welford_state_t{};

  uint32_t count = 0;

#define RECOVER_WELFORD_CAPTURE_ONE(flat_prefix, stats_key, dac_stats, has_flag, dst) \
  do { \
    (void)recover_welford_capture_lane(args, flat_prefix, stats_key, dac_stats, \
                                       has_flag, dst, count); \
    if (g_clocks_payload_numeric_integrity_failed) { \
      g_recover_welfords_pending = recover_welford_state_t{}; \
      return false; \
    } \
  } while (0)

  RECOVER_WELFORD_CAPTURE_ONE("dwt", "dwt", false,
                              g_recover_welfords_pending.has_dwt,
                              g_recover_welfords_pending.dwt);
  RECOVER_WELFORD_CAPTURE_ONE("vclock", "vclock", false,
                              g_recover_welfords_pending.has_vclock,
                              g_recover_welfords_pending.vclock);
  RECOVER_WELFORD_CAPTURE_ONE("ocxo1", "ocxo1", false,
                              g_recover_welfords_pending.has_ocxo1,
                              g_recover_welfords_pending.ocxo1);
  RECOVER_WELFORD_CAPTURE_ONE("ocxo2", "ocxo2", false,
                              g_recover_welfords_pending.has_ocxo2,
                              g_recover_welfords_pending.ocxo2);
  RECOVER_WELFORD_CAPTURE_ONE("pps_witness", "pps_witness", false,
                              g_recover_welfords_pending.has_pps_witness,
                              g_recover_welfords_pending.pps_witness);
  RECOVER_WELFORD_CAPTURE_ONE("ocxo1_dac", "ocxo1", true,
                              g_recover_welfords_pending.has_ocxo1_dac,
                              g_recover_welfords_pending.ocxo1_dac);
  RECOVER_WELFORD_CAPTURE_ONE("ocxo2_dac", "ocxo2", true,
                              g_recover_welfords_pending.has_ocxo2_dac,
                              g_recover_welfords_pending.ocxo2_dac);

#undef RECOVER_WELFORD_CAPTURE_ONE

  g_recover_welfords_pending.restored_lane_count = count;
  if (count != 0U) {
    g_recover_welford_capture_count++;
  }
  return true;
}

static void recover_welfords_apply_pending(void) {
  if (g_recover_welfords_pending.restored_lane_count == 0U) {
    g_recover_welford_last_restored_lane_count = 0U;
    return;
  }

  if (g_recover_welfords_pending.has_dwt) {
    welford_dwt = g_recover_welfords_pending.dwt;
  }
  if (g_recover_welfords_pending.has_vclock) {
    welford_vclock = g_recover_welfords_pending.vclock;
  }
  if (g_recover_welfords_pending.has_ocxo1) {
    welford_ocxo1 = g_recover_welfords_pending.ocxo1;
  }
  if (g_recover_welfords_pending.has_ocxo2) {
    welford_ocxo2 = g_recover_welfords_pending.ocxo2;
  }
  if (g_recover_welfords_pending.has_pps_witness) {
    welford_pps_witness = g_recover_welfords_pending.pps_witness;
  }
  if (g_recover_welfords_pending.has_ocxo1_dac) {
    welford_ocxo1_dac = g_recover_welfords_pending.ocxo1_dac;
  }
  if (g_recover_welfords_pending.has_ocxo2_dac) {
    welford_ocxo2_dac = g_recover_welfords_pending.ocxo2_dac;
  }

  g_recover_welford_last_restored_lane_count =
      g_recover_welfords_pending.restored_lane_count;
  g_recover_welford_restore_count++;
  g_recover_welfords_pending = recover_welford_state_t{};
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

static FLASHMEM void payload_add_visible_origin_snapshot(Payload& parent,
                                                const char* key,
                                                time_clock_id_t clock) {
  clocks_alpha_ocxo_visible_origin_snapshot_t s{};
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

static FLASHMEM void publish_freq(Payload& p, const char* clock_name, double ppb_value) {
  char key[80];
  const double tau_value = 1.0 + ppb_value / 1e9;

  snprintf(key, sizeof(key), "%s_tau", clock_name);
  p.add(key, tau_value, 12);

  snprintf(key, sizeof(key), "%s_ppb", clock_name);
  p.add(key, ppb_value, 3);
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

static FLASHMEM void payload_add_welford_object(Payload& parent,
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

static FLASHMEM void payload_add_frequency_fields(Payload& obj, double ppb_value) {
  const double tau_value = 1.0 + ppb_value / 1e9;
  obj.add("tau", tau_value, 12);
  obj.add("ppb", ppb_value, 3);
}

static double campaign_total_tau_from_ratio(uint64_t reference_value,
                                            uint64_t clock_value) {
  if (reference_value == 0ULL) return 1.0;
  return (double)clock_value / (double)reference_value;
}

static double campaign_total_ppb_from_tau(double tau) {
  return (tau - 1.0) * 1.0e9;
}

static double campaign_total_ppb_from_ratio(uint64_t reference_value,
                                            uint64_t clock_value) {
  return campaign_total_ppb_from_tau(
      campaign_total_tau_from_ratio(reference_value, clock_value));
}

static double campaign_total_dwt_ppb(uint64_t public_gnss_ns,
                                     uint64_t public_dwt_total_cycles) {
  const uint64_t expected_cycles = dwt_ns_to_cycles(public_gnss_ns);
  if (expected_cycles == 0ULL) return 0.0;
  return campaign_total_ppb_from_ratio(expected_cycles,
                                       public_dwt_total_cycles);
}

static FLASHMEM void payload_add_stats_clock(Payload& parent,
                                    const char* key,
                                    const welford_t& w,
                                    bool include_frequency,
                                    double campaign_ppb = 0.0) {
  Payload obj;
  payload_add_welford_object(obj, "welford", w);
  if (include_frequency) {
    // Frequency fields are campaign-total ratios.  The Welford object remains
    // the per-second residual statistics surface.  Do not publish w.mean as
    // tau/ppb: that makes TAU/PPB indistinguishable from MEAN and hides the
    // accumulated campaign clock/GNSS ratio.
    payload_add_frequency_fields(obj, campaign_ppb);
  }
  parent.add_object(key, obj);
}

static FLASHMEM void payload_add_stats_ocxo_clock(Payload& parent,
                                         const char* key,
                                         const welford_t& w,
                                         const clock_science_row_t& science) {
  Payload obj;
  payload_add_welford_object(obj, "welford", w);

  // Keep the public stats object compact.  The panel-facing OCXO frequency
  // value is the continuity-aligned campaign clockface ratio published in
  // science.total_ppb/total_tau: public_ocxo_ns / public_gnss_ns.  Welford
  // continues to describe the per-second Delta/PhaseLedger residual
  // distribution.
  const double ppb = science.total_valid ? science.total_ppb : 0.0;
  payload_add_frequency_fields(obj, ppb);

  parent.add_object(key, obj);
}

static FLASHMEM void payload_add_stats_summary_hierarchical(Payload& p,
                                                   uint64_t public_gnss_ns,
                                                   uint64_t public_dwt_total,
                                                   const clock_science_row_t& ocxo1_science,
                                                   const clock_science_row_t& ocxo2_science) {
  Payload stats;
  payload_add_stats_clock(stats, "dwt", welford_dwt, true,
                          campaign_total_dwt_ppb(public_gnss_ns,
                                                 public_dwt_total));
  payload_add_stats_clock(stats, "vclock", welford_vclock, true, 0.0);
  payload_add_stats_ocxo_clock(stats, "ocxo1", welford_ocxo1, ocxo1_science);
  payload_add_stats_ocxo_clock(stats, "ocxo2", welford_ocxo2, ocxo2_science);
  payload_add_stats_clock(stats, "pps_witness", welford_pps_witness, false);

  Payload dac;
  payload_add_welford_object(dac, "ocxo1", welford_ocxo1_dac);
  payload_add_welford_object(dac, "ocxo2", welford_ocxo2_dac);
  stats.add_object("dac", dac);

  p.add_object("stats", stats);
}

static FLASHMEM void payload_add_prediction_lane_object(Payload& parent,
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

static FLASHMEM void payload_add_prediction_summary_hierarchical(Payload& p) {
  Payload prediction;
  payload_add_prediction_lane_object(prediction, "pps", prediction_snapshot_for_pps());
  payload_add_prediction_lane_object(prediction, "vclock", prediction_snapshot_for_clock(time_clock_id_t::VCLOCK));
  payload_add_prediction_lane_object(prediction, "ocxo1", prediction_snapshot_for_clock(time_clock_id_t::OCXO1));
  payload_add_prediction_lane_object(prediction, "ocxo2", prediction_snapshot_for_clock(time_clock_id_t::OCXO2));
  p.add_object("prediction", prediction);
}

static FLASHMEM const char* pps_vclock_edge_decision_name(uint32_t decision) {
  switch (decision) {
    case PPS_VCLOCK_EDGE_DECISION_LOWER_LAWFUL:
      return "LOWER_LAWFUL";
    case PPS_VCLOCK_EDGE_DECISION_PREDICTION_FALLBACK:
      return "PREDICTION_FALLBACK";
    case PPS_VCLOCK_EDGE_DECISION_PPS_PHASE_FALLBACK:
      return "PPS_PHASE_FALLBACK";
    case PPS_VCLOCK_EDGE_DECISION_OBSERVED_FALLBACK:
      return "OBSERVED_FALLBACK";
    default:
      return "NONE";
  }
}


static bool floorline_candidate_present(
    bool lane_valid,
    const clocks_alpha_lane_forensics_t& f) {
  // FloorLine is the lower-envelope successor to the retired regression
  // experiment.  Some live report paths still expose the stale
  // regression_valid bit as false even when the lower-envelope candidate
  // fields are fully populated and already used as the published DWT edge.
  // Gate TIMEBASE visibility on candidate presence, not that legacy flag.
  return lane_valid &&
         f.regression_inferred_dwt_at_event != 0U &&
         f.regression_observed_dwt_at_event != 0U &&
         f.regression_sample_count != 0U;
}

static FLASHMEM void payload_add_pps_vclock_edge_forensics(
    Payload& parent,
    bool available,
    const clocks_pps_vclock_edge_forensics_t& e) {
  Payload edge;
  edge.add("available", available);
  edge.add("valid", available && e.valid);
  edge.add("sequence", available ? e.sequence : 0U);
  edge.add("update_count", available ? e.update_count : 0U);
  edge.add("reject_count", available ? e.reject_count : 0U);

  edge.add("authority_dwt_at_edge", available ? e.authority_dwt_at_edge : 0U);
  edge.add("pps_dwt_at_edge", available ? e.pps_dwt_at_edge : 0U);
  edge.add("decision", pps_vclock_edge_decision_name(available ? e.decision : 0U));
  edge.add("decision_id", available ? e.decision : 0U);
  edge.add("invalid_mask", available ? e.invalid_mask : 0U);
  edge.add("authority_minus_pps_cycles",
           available ? e.authority_minus_pps_cycles : 0);
  edge.add("counter32_at_edge", available ? e.counter32_at_edge : 0U);
  edge.add("ch3_at_edge", available ? (uint32_t)e.ch3_at_edge : 0U);
  edge.add("dwt_cycles_between_edges",
           available ? e.dwt_cycles_between_edges : 0U);
  edge.add("effective_dwt_cycles_per_second",
           available ? e.effective_dwt_cycles_per_second : 0U);

  edge.add("gnss_self_map_valid", available && e.gnss_self_map_valid);
  edge.add("gnss_self_error_ok", available && e.gnss_self_error_ok);
  edge.add("gnss_self_error_gate_ns",
           available ? e.gnss_self_error_gate_ns : 0U);
  edge.add("expected_gnss_ns_at_edge",
           available ? e.expected_gnss_ns_at_edge : 0ULL);
  edge.add("mapped_gnss_ns_at_edge",
           available ? e.mapped_gnss_ns_at_edge : 0ULL);
  edge.add("gnss_self_error_ns", available ? e.gnss_self_error_ns : 0LL);
  parent.add_object("pps_vclock_edge", edge);
}

static FLASHMEM void payload_add_lane_forensics_object(Payload& parent,
                                              bool valid,
                                              const clocks_alpha_lane_forensics_t& f) {
  Payload forensics;
  forensics.add("valid", valid);
  forensics.add("update_count", valid ? f.update_count : 0U);
  forensics.add("last_event_dwt", valid ? f.last_event_dwt : 0U);
  forensics.add("last_event_counter32", valid ? f.last_event_counter32 : 0U);
  forensics.add("dwt_cycles_between_edges", valid ? f.dwt_cycles_between_edges : 0U);
  forensics.add("physical_measured_ns_at_edge",
                valid ? f.physical_measured_ns_at_edge : 0ULL);
  forensics.add("visible_ns_at_edge", valid ? f.visible_ns_at_edge : 0ULL);
  forensics.add("visible_origin_phase_valid",
                valid && f.visible_origin_phase_valid);
  forensics.add("visible_origin_phase_offset_ns",
                valid ? f.visible_origin_phase_offset_ns : 0U);
  forensics.add("counter32_delta_since_previous_event",
                valid ? f.counter32_delta_since_previous_event : 0U);

  // Raw/effective DWT authority proof.  This is the durable per-row evidence
  // needed by raw_cycles-style reports after the 500-cycle catastrophic gate.
  forensics.add("dwt_synthetic", valid && f.dwt_synthetic);
  forensics.add("dwt_original_at_event", valid ? f.dwt_original_at_event : 0U);
  forensics.add("dwt_predicted_at_event", valid ? f.dwt_predicted_at_event : 0U);
  forensics.add("dwt_ema_dwt_at_event", valid ? f.dwt_ema_dwt_at_event : 0U);
  forensics.add("dwt_used_at_event", valid ? f.dwt_used_at_event : 0U);
  forensics.add("dwt_isr_entry_raw", valid ? f.dwt_isr_entry_raw : 0U);
  forensics.add("dwt_event_from_isr_entry_raw",
                valid ? f.dwt_event_from_isr_entry_raw : 0U);
  forensics.add("dwt_isr_entry_to_event_correction_cycles",
                valid ? f.dwt_isr_entry_to_event_correction_cycles : 0);
  forensics.add("dwt_published_minus_event_cycles",
                valid ? f.dwt_published_minus_event_cycles : 0);
  forensics.add("dwt_used_minus_event_cycles",
                valid ? f.dwt_used_minus_event_cycles : 0);
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

  // PPS-Yardstick inference rail (Stage 1 -- observational).  Compact per-row
  // evidence: the unquantized PPS yardstick pair consumed, the inferred Q16
  // interval/endpoint, the gate verdict it WOULD have given, and chain-walk
  // audit, published beside the EMA surfaces above for side-by-side analysis.
  Payload yardstick;
  yardstick.add("valid", valid && f.dwt_yardstick_valid);
  yardstick.add("stale", valid && f.dwt_yardstick_stale);
  yardstick.add("seeded", valid && f.dwt_yardstick_seeded);
  yardstick.add("excursion", valid && f.dwt_yardstick_excursion);
  yardstick.add("pps_sequence", valid ? f.dwt_yardstick_pps_sequence : 0U);
  yardstick.add("pps_seq_delta", valid ? f.dwt_yardstick_pps_seq_delta : 0U);
  yardstick.add("g_now_cycles", valid ? f.dwt_yardstick_g_now_cycles : 0U);
  yardstick.add("g_prev_cycles", valid ? f.dwt_yardstick_g_prev_cycles : 0U);
  yardstick.add("inferred_interval_cycles",
                valid ? f.dwt_yardstick_inferred_interval_cycles : 0U);
  yardstick.add("observed_interval_cycles",
                valid ? f.dwt_yardstick_observed_interval_cycles : 0U);
  yardstick.add("inferred_minus_observed_cycles",
                valid ? f.dwt_yardstick_inferred_minus_observed_cycles : 0);
  yardstick.add("endpoint_minus_observed_cycles",
                valid ? f.dwt_yardstick_endpoint_minus_observed_cycles : 0);
  yardstick.add("gate_threshold_cycles",
                valid ? f.dwt_yardstick_gate_threshold_cycles : 0U);
  yardstick.add("gate_agree_count",
                valid ? f.dwt_yardstick_gate_agree_count : 0U);
  yardstick.add("gate_excursion_count",
                valid ? f.dwt_yardstick_gate_excursion_count : 0U);
  yardstick.add("authority", valid && f.dwt_yardstick_authority);
  yardstick.add("auth_error_cycles",
                valid ? f.dwt_yardstick_auth_error_cycles : 0);
  yardstick.add("auth_anchor_applied",
                valid && f.dwt_yardstick_auth_anchor_applied);
  forensics.add_object("dwt_yardstick", yardstick);

  // 1 kHz lower-envelope inference rail.  process_interrupt reuses the
  // regression_* transport fields for this report-only successor to the retired
  // linear regression experiment.  The interval field below is the computed
  // slope-derived one-second cycle count; dwt_interval_gate.observed_cycles is
  // the raw observed one-second interval used for side-by-side analysis.
  Payload lower_env;
  const bool lower_valid = floorline_candidate_present(valid, f);
  const uint32_t lower_inferred_interval_cycles = lower_valid
      ? (uint32_t)(((uint64_t)f.regression_slope_q16_cycles_per_sample *
                    1000ULL + 32768ULL) >> 16)
      : 0U;
  const uint32_t lower_observed_interval_cycles = valid
      ? f.dwt_interval_observed_cycles
      : 0U;
  lower_env.add("valid", lower_valid);
  lower_env.add("source", valid && f.dwt_interval_sample_accepted ? 1U : 0U);
  lower_env.add("reason", valid ? f.dwt_interval_reject_streak : 0U);
  lower_env.add("published", valid && f.dwt_interval_sample_accepted);
  lower_env.add("sample_total_count", lower_valid ? f.regression_sample_count : 0U);
  lower_env.add("sample_accepted_count", valid ? f.dwt_interval_accept_count : 0U);
  lower_env.add("sample_rejected_count", valid ? f.dwt_interval_reject_count : 0U);
  lower_env.add("bucket_accepted_count", valid ? f.dwt_interval_resync_count : 0U);
  lower_env.add("gate_cycles", valid ? f.dwt_interval_gate_threshold_cycles : 0U);
  lower_env.add("sequence", lower_valid ? f.regression_sequence : 0U);
  lower_env.add("sample_count", lower_valid ? f.regression_sample_count : 0U);
  lower_env.add("observed_edge_dwt",
                lower_valid ? f.regression_observed_dwt_at_event : 0U);
  lower_env.add("inferred_edge_dwt",
                lower_valid ? f.regression_inferred_dwt_at_event : 0U);
  lower_env.add("inferred_minus_observed_edge_cycles",
                lower_valid ? f.regression_inferred_minus_observed_cycles : 0);
  lower_env.add("observed_interval_cycles", lower_observed_interval_cycles);
  lower_env.add("inferred_interval_cycles", lower_inferred_interval_cycles);
  lower_env.add("inferred_minus_observed_interval_cycles",
                lower_valid
                    ? ((lower_inferred_interval_cycles >= lower_observed_interval_cycles)
                           ? (int32_t)(lower_inferred_interval_cycles - lower_observed_interval_cycles)
                           : -(int32_t)(lower_observed_interval_cycles - lower_inferred_interval_cycles))
                    : 0);
  lower_env.add("target_counter32_at_edge",
                lower_valid ? f.regression_target_counter32_at_event : 0U);
  lower_env.add("target_hardware16_at_edge",
                lower_valid ? (uint32_t)f.regression_target_hardware16_at_event : 0U);
  lower_env.add("observed_hardware16_at_edge",
                lower_valid ? (uint32_t)f.regression_observed_hardware16_at_event : 0U);
  lower_env.add("slope_q16_cycles_per_sample",
                lower_valid ? f.regression_slope_q16_cycles_per_sample : 0ULL);
  lower_env.add("slope_delta_q16_cycles_per_sample",
                lower_valid ? f.regression_slope_delta_q16_cycles_per_sample : 0LL);
  lower_env.add("fit_error_mean_q16_cycles",
                lower_valid ? f.regression_fit_error_mean_q16_cycles : 0);
  lower_env.add("fit_error_stddev_q16_cycles",
                lower_valid ? f.regression_fit_error_stddev_q16_cycles : 0U);
  lower_env.add("fit_error_min_cycles",
                lower_valid ? f.regression_fit_error_min_cycles : 0);
  lower_env.add("fit_error_max_cycles",
                lower_valid ? f.regression_fit_error_max_cycles : 0);
  lower_env.add("fit_error_gt_plus4_count",
                lower_valid ? f.regression_fit_error_gt_plus4_count : 0U);
  lower_env.add("fit_error_lt_minus4_count",
                lower_valid ? f.regression_fit_error_lt_minus4_count : 0U);
  lower_env.add("fit_error_abs_gt4_count",
                lower_valid ? f.regression_fit_error_abs_gt4_count : 0U);
  lower_env.add("reporting_only", false);
  forensics.add_object("lower_envelope", lower_env);


  // Keep the paired TIMEBASE_FORENSICS row lean enough to arrive before the
  // next fragment.  REPORT_DWT_CAPTURE carries the full SlipLedger dossier;
  // TIMEBASE carries only the durable campaign-row summary.
  if (valid && f.slipledger_active) {
    Payload slip;
    slip.add("active", true);
    slip.add("ticks", f.slipledger_ticks);
    slip.add("generation", f.slipledger_generation);
    slip.add("violation_count", f.slipledger_violation_count);
    slip.add("correction_count", f.slipledger_correction_count);
    slip.add("event_violation", f.slipledger_event_violation);
    slip.add("event_corrected", f.slipledger_event_corrected);
    slip.add("last_dwt_error_cycles", f.slipledger_last_dwt_error_cycles);
    slip.add("last_correction_ticks", f.slipledger_last_correction_ticks);
    slip.add("last_correction_dwt_error_cycles",
             f.slipledger_last_correction_dwt_error_cycles);
    slip.add("reason_code", f.slipledger_reason_code);
    slip.add("last_correction_reason_code",
             f.slipledger_last_correction_reason_code);
    forensics.add_object("slipledger", slip);
  }

  parent.add_object("forensics", forensics);
}

static FLASHMEM void payload_add_ocxo_service_object(Payload& parent,
                                            interrupt_subscriber_kind_t kind,
                                            bool valid,
                                            const clocks_alpha_lane_forensics_t& f) {
  Payload service;
  const interrupt_capture_diag_t* direct_diag = valid ? interrupt_last_diag(kind) : nullptr;
  const bool direct_valid = direct_diag && direct_diag->enabled &&
                            direct_diag->kind == kind &&
                            direct_diag->counter32_at_event == f.last_event_counter32;

  // Excursion hunt surface.  Keep this object numerically compact, but publish
  // the compare-service and perishable-fact custody fields needed to decide
  // whether a bad raw interval was born in compare presentation, low-word
  // generation math, or deferred fact association.
  service.add("class", valid ? f.diag_service_class : 0U);
  service.add("offset_ticks", valid ? f.diag_service_offset_signed_ticks : 0);
  service.add("offset_abs_ticks", valid ? f.diag_service_offset_abs_ticks : 0U);
  service.add("interpreted_late_ticks", valid ? f.diag_interpreted_late_ticks : 0U);
  service.add("early_ticks", valid ? f.diag_early_ticks : 0U);
  service.add("target_delta_mod65536_ticks",
              valid ? f.diag_target_delta_mod65536_ticks : 0U);
  service.add("arm_remaining_ticks", valid ? f.diag_arm_remaining_ticks : 0U);
  service.add("arm_to_isr_ticks", valid ? f.diag_arm_to_isr_ticks : 0U);
  service.add("arm_to_isr_dwt_cycles",
              valid ? f.diag_arm_to_isr_dwt_cycles : 0U);

  // Counter-vs-compare rail courtroom.  These are read directly from the
  // latest process_interrupt diagnostic because Alpha's durable lane snapshot
  // intentionally stays smaller than this temporary split-channel autopsy.
  service.add("arm_counter_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_arm_counter_low16 : 0U);
  service.add("arm_compare_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_arm_compare_low16 : 0U);
  service.add("arm_counter_minus_compare_ticks",
              direct_valid ? direct_diag->ocxo_arm_counter_minus_compare_ticks : 0U);
  service.add("arm_compare_remaining_ticks",
              direct_valid ? direct_diag->ocxo_arm_compare_remaining_ticks : 0U);
  service.add("isr_counter_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_isr_counter_low16 : 0U);
  service.add("isr_compare_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_isr_compare_low16 : 0U);
  service.add("isr_counter_minus_compare_ticks",
              direct_valid ? direct_diag->ocxo_isr_counter_minus_compare_ticks : 0U);
  service.add("compare_target_delta_mod65536_ticks",
              direct_valid ? direct_diag->ocxo_compare_delta_mod65536_ticks : 0U);
  service.add("compare_offset_ticks",
              direct_valid ? direct_diag->ocxo_compare_service_offset_signed_ticks : 0);
  service.add("compare_interpreted_late_ticks",
              direct_valid ? direct_diag->ocxo_compare_interpreted_late_ticks : 0U);
  service.add("compare_early_ticks",
              direct_valid ? direct_diag->ocxo_compare_early_ticks : 0U);
  service.add("compare_arm_to_isr_ticks",
              direct_valid ? direct_diag->ocxo_compare_arm_to_isr_ticks : 0U);

  service.add("perishable_fact_sequence",
              valid ? f.diag_perishable_fact_sequence : 0U);
  service.add("service_correction_cycles",
              valid ? f.diag_service_correction_cycles : 0);
  service.add("service_corrected_dwt_at_event",
              valid ? f.diag_service_corrected_dwt_at_event : 0U);
  service.add("fact_ring_overflow_count",
              valid ? f.diag_fact_ring_overflow_count : 0U);
  service.add("counter_delta_violation_count",
              valid ? f.diag_counter_delta_violation_count : 0U);
  service.add("last_bad_counter_delta",
              valid ? f.diag_last_bad_counter_delta : 0U);
  service.add("last_counter_delta_ticks",
              valid ? f.diag_last_counter_delta_ticks : 0U);
  service.add("sample_dwt_at_event",
              valid ? f.diag_sample_dwt_at_event : 0U);
  service.add("sample_counter32_at_event",
              valid ? f.diag_sample_counter32_at_event : 0U);
  parent.add_object("service", service);
}

// ============================================================================
// OCXO cycle-domain residual diagnostic
// ============================================================================
//
// Courtroom-only legacy diagnostic for the OCXO residual authority chain.
// The authoritative OCXO residual is now the delayed-reference FloorLine
// Delta Cycles residual carried in clock_science_row_t::delta_floorline_*;
// Welfords, tau, pps_residual, and servo input consume that same surface.
//
// This older object compares the GNSS/PPS static-prediction DWT interval and
// each OCXO static-prediction DWT interval in the same Teensy coordinate
// species.  It remains useful as a rough audit, but it is not the canonical
// delayed-VCLOCK-reference Delta residual.
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

static ocxo_cycle_residual_diag_t g_beta_ocxo1_cycle_residual_diag DMAMEM = {};
static ocxo_cycle_residual_diag_t g_beta_ocxo2_cycle_residual_diag DMAMEM = {};

static void ocxo_cycle_residual_diag_build(
    ocxo_cycle_residual_diag_t& d,
    const clocks_static_prediction_snapshot_t& gnss,
    const clocks_static_prediction_snapshot_t& clock,
    bool traditional_valid,
    int64_t traditional_fast_residual_ns) {
  d = ocxo_cycle_residual_diag_t{};

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
}

static FLASHMEM void payload_add_ocxo_pps_residual_object(Payload& parent,
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
  residual.add("source", clocks_ocxo_counterledger_mode_enabled()
      ? "COUNTERLEDGER_PPS_CAPTURE"
      : "DELTA_OBSERVED_DWT_EDGE");
  residual.add("floorline_preserved_under", "science.delta_floorline_fast_residual_ns");
  residual.add("traditional_preserved_under", "science.traditional_fast_residual_ns");
  parent.add_object("pps_residual", residual);
}

static FLASHMEM void payload_add_ocxo_cycle_residual_diag_object(
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


static int32_t beta_signed_delta_u32(uint32_t observed, uint32_t expected) {
  return (observed >= expected)
      ? (int32_t)(observed - expected)
      : -(int32_t)(expected - observed);
}

static uint32_t floorline_inferred_interval_cycles(
    bool present,
    const clocks_alpha_lane_forensics_t& f) {
  if (!present || f.regression_slope_q16_cycles_per_sample == 0ULL) {
    return 0U;
  }

  const uint64_t cycles =
      (f.regression_slope_q16_cycles_per_sample * 1000ULL + 32768ULL) >> 16;
  return (cycles > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;
}


static FLASHMEM void payload_add_micro_court_fields(
    Payload& parent,
    const char* prefix,
    bool valid,
    const clocks_alpha_lane_forensics_t& f) {
  char key[96];

  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    parent.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    parent.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    parent.add(key, value);
  };
  auto add_i64 = [&](const char* suffix, int64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    parent.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    parent.add(key, value ? value : "");
  };

  add_bool("court_valid", valid);
  add_u32("court_mask", valid ? f.dwt_publication_verdict_mask : 0U);
  const uint32_t court_reason_id = valid
      ? f.dwt_publication_verdict_reason_id
      : INTERRUPT_DWT_PUBLICATION_REASON_OK;
  add_u32("court_reason_id", court_reason_id);
  add_str("court_reason", interrupt_dwt_publication_reason_name(court_reason_id));
  add_u32("court_wd", valid ? f.dwt_publication_watchdog_count : 0U);
  add_u32("court_gate", valid ? f.dwt_publication_gate_cycles : 0U);
  add_u32("court_xgate",
          valid ? f.dwt_publication_cross_rail_gate_cycles : 0U);
  add_u32("court_svc_gate",
          valid ? f.dwt_publication_service_offset_gate_ticks : 0U);
  add_u32("court_exp_cnt",
          valid ? f.dwt_publication_expected_counter_delta_ticks : 0U);
  add_u32("court_obs_cnt",
          valid ? f.dwt_publication_observed_counter_delta_ticks : 0U);
  add_u32("court_exp_int",
          valid ? f.dwt_publication_expected_interval_cycles : 0U);
  add_u32("court_pub_int",
          valid ? f.dwt_publication_published_interval_cycles : 0U);
  add_u32("court_obs_int",
          valid ? f.dwt_publication_observed_interval_cycles : 0U);
  add_u32("court_fl_int",
          valid ? f.dwt_publication_floorline_interval_cycles : 0U);
  add_i32("court_pub_err",
          valid ? f.dwt_publication_published_interval_error_cycles : 0);
  add_i32("court_obs_err",
          valid ? f.dwt_publication_observed_interval_error_cycles : 0);
  add_i32("court_fl_err",
          valid ? f.dwt_publication_floorline_interval_error_cycles : 0);
  add_i32("court_pub_obs",
          valid ? f.dwt_publication_published_minus_observed_cycles : 0);
  add_i32("court_fl_obs",
          valid ? f.dwt_publication_floorline_minus_observed_cycles : 0);
  add_i32("court_svc_off",
          valid ? f.dwt_publication_service_offset_signed_ticks : 0);
  add_i64("court_gnss_err",
          valid ? f.dwt_publication_vclock_gnss_error_ns : 0LL);
}


static FLASHMEM void payload_add_floorline_object(Payload& parent,
                                         bool valid,
                                         const clocks_alpha_lane_forensics_t& f) {
  const bool floor_valid = floorline_candidate_present(valid, f);
  const uint32_t observed_interval =
      valid ? f.dwt_interval_observed_cycles : 0U;
  const uint32_t inferred_interval =
      floorline_inferred_interval_cycles(floor_valid, f);

  Payload floor;
  floor.add("valid", floor_valid);
  floor.add("reporting_only", false);
  floor.add("source", valid && f.dwt_interval_sample_accepted ? 1U : 0U);
  floor.add("reason", valid ? f.dwt_interval_reject_streak : 0U);
  floor.add("published", valid && f.dwt_interval_sample_accepted);
  floor.add("sample_accepted_count", valid ? f.dwt_interval_accept_count : 0U);
  floor.add("sample_rejected_count", valid ? f.dwt_interval_reject_count : 0U);
  floor.add("bucket_accepted_count", valid ? f.dwt_interval_resync_count : 0U);
  floor.add("gate_cycles", valid ? f.dwt_interval_gate_threshold_cycles : 0U);
  floor.add("sequence", floor_valid ? f.regression_sequence : 0U);
  floor.add("sample_count", floor_valid ? f.regression_sample_count : 0U);

  floor.add("observed_edge_dwt",
            floor_valid ? f.regression_observed_dwt_at_event : 0U);
  floor.add("inferred_edge_dwt",
            floor_valid ? f.regression_inferred_dwt_at_event : 0U);
  floor.add("inferred_minus_observed_edge_cycles",
            floor_valid ? f.regression_inferred_minus_observed_cycles : 0);

  floor.add("observed_interval_cycles", observed_interval);
  floor.add("inferred_interval_cycles", inferred_interval);
  floor.add("inferred_minus_observed_interval_cycles",
            floor_valid
                ? beta_signed_delta_u32(inferred_interval, observed_interval)
                : 0);

  floor.add("target_counter32_at_edge",
            floor_valid ? f.regression_target_counter32_at_event : 0U);
  floor.add("target_hardware16_at_edge",
            floor_valid ? (uint32_t)f.regression_target_hardware16_at_event : 0U);
  floor.add("observed_hardware16_at_edge",
            floor_valid ? (uint32_t)f.regression_observed_hardware16_at_event : 0U);

  floor.add("slope_q16_cycles_per_sample",
            floor_valid ? f.regression_slope_q16_cycles_per_sample : 0ULL);
  floor.add("slope_delta_q16_cycles_per_sample",
            floor_valid ? f.regression_slope_delta_q16_cycles_per_sample : 0LL);
  floor.add("fit_error_mean_q16_cycles",
            floor_valid ? f.regression_fit_error_mean_q16_cycles : 0);
  floor.add("fit_error_stddev_q16_cycles",
            floor_valid ? f.regression_fit_error_stddev_q16_cycles : 0U);
  floor.add("fit_error_min_cycles",
            floor_valid ? f.regression_fit_error_min_cycles : 0);
  floor.add("fit_error_max_cycles",
            floor_valid ? f.regression_fit_error_max_cycles : 0);
  floor.add("fit_error_abs_gt4_count",
            floor_valid ? f.regression_fit_error_abs_gt4_count : 0U);

  parent.add_object("floorline", floor);
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

static delta_residual_bookend_t* delta_residual_ocxo_bookends(
    time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_delta_ocxo1_bookends;
    case time_clock_id_t::OCXO2: return &g_delta_ocxo2_bookends;
    default:                    return nullptr;
  }
}

static bool delta_floorline_endpoint_interval(
    delta_residual_bookend_t& state,
    bool floorline_valid,
    uint32_t floorline_dwt_at_edge,
    uint32_t* out_interval_cycles) {
  if (out_interval_cycles) *out_interval_cycles = 0U;
  if (!floorline_valid || floorline_dwt_at_edge == 0U) {
    return false;
  }

  const bool have_interval = state.floorline_prev_dwt_valid;
  const uint32_t interval = floorline_dwt_at_edge - state.floorline_prev_dwt;
  state.floorline_prev_dwt = floorline_dwt_at_edge;
  state.floorline_prev_dwt_valid = true;

  if (!have_interval || interval == 0U) {
    return false;
  }
  if (out_interval_cycles) *out_interval_cycles = interval;
  return true;
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
  if (selected_pps_vclock_interval != 0U) {
    r.gnss_valid = true;
    r.gnss_interval_cycles = selected_pps_vclock_interval;
  }

  // Keep the VCLOCK measured rail as side evidence only. The canonical
  // reference for Delta Cycles is the exact observed selected PPS/VCLOCK
  // edge-to-edge interval: current snap.dwt_at_edge minus the previous
  // snap.dwt_at_edge.
  if (vclock_valid && vclock_f.dwt_interval_observed_cycles != 0U) {
    r.raw_valid = true;
    r.raw_interval_cycles = vclock_f.dwt_interval_observed_cycles;
  }

  const bool vclock_floorline_valid =
      floorline_candidate_present(vclock_valid, vclock_f);
  r.floorline_fit_interval_cycles = vclock_floorline_valid
      ? floorline_inferred_interval_cycles(vclock_floorline_valid, vclock_f)
      : 0U;
  (void)delta_floorline_endpoint_interval(
      g_delta_vclock_bookends,
      vclock_floorline_valid,
      vclock_floorline_valid ? vclock_f.regression_inferred_dwt_at_event : 0U,
      &r.floorline_interval_cycles);
  r.floorline_valid = (r.floorline_interval_cycles != 0U);
  if (r.floorline_valid && r.floorline_fit_interval_cycles != 0U) {
    r.floorline_fit_minus_endpoint_cycles =
        beta_signed_delta_u32(r.floorline_fit_interval_cycles,
                              r.floorline_interval_cycles);
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

  if (ref_count_aligned && ref.gnss_valid && clock_valid &&
      clock_f.dwt_interval_observed_cycles != 0U) {
    row.delta_raw_valid = true;
    row.delta_raw_reference_interval_cycles = ref.gnss_interval_cycles;
    row.delta_raw_clock_interval_cycles = clock_f.dwt_interval_observed_cycles;
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

  delta_residual_bookend_t* clock_bookends =
      delta_residual_ocxo_bookends((time_clock_id_t)((uint8_t)row.clock_id));
  uint32_t clock_floorline_endpoint_interval = 0U;
  if (clock_bookends) {
    (void)delta_floorline_endpoint_interval(
        *clock_bookends,
        row.clock_floorline_valid,
        row.clock_floorline_dwt_at_edge,
        &clock_floorline_endpoint_interval);
  }
  row.clock_floorline_endpoint_interval_cycles =
      clock_floorline_endpoint_interval;

  if (ref_count_aligned && ref.floorline_valid &&
      clock_floorline_endpoint_interval != 0U) {
    row.delta_floorline_valid = true;
    row.delta_floorline_reference_interval_cycles =
        ref.floorline_interval_cycles;
    row.delta_floorline_clock_interval_cycles =
        clock_floorline_endpoint_interval;
    row.delta_floorline_residual_cycles =
        (int64_t)row.delta_floorline_clock_interval_cycles -
        (int64_t)row.delta_floorline_reference_interval_cycles;
    row.delta_floorline_fast_residual_cycles =
        -row.delta_floorline_residual_cycles;
    const uint32_t floorline_delta_cps =
        row.delta_floorline_reference_interval_cycles
            ? row.delta_floorline_reference_interval_cycles
            : row.projection_cps_cycles;
    row.delta_floorline_residual_ns_exact =
        beta_dwt_cycles_to_gnss_ns_exact_signed(
            row.delta_floorline_residual_cycles,
            floorline_delta_cps);
    row.delta_floorline_fast_residual_ns_exact =
        -row.delta_floorline_residual_ns_exact;
    row.delta_floorline_residual_ns =
        beta_round_double_to_i64(row.delta_floorline_residual_ns_exact);
    row.delta_floorline_fast_residual_ns =
        beta_round_double_to_i64(row.delta_floorline_fast_residual_ns_exact);
  }
}

static void delta_residual_attach_comparisons(clock_science_row_t& row) {
  if (row.delta_raw_valid && row.traditional_valid) {
    row.delta_raw_fast_minus_traditional_ns =
        row.delta_raw_fast_residual_ns - row.traditional_fast_residual_ns;
  }
  if (row.delta_floorline_valid && row.traditional_valid) {
    row.delta_floorline_fast_minus_traditional_ns =
        row.delta_floorline_fast_residual_ns - row.traditional_fast_residual_ns;
  }
  if (row.delta_raw_valid && row.delta_floorline_valid) {
    row.delta_raw_fast_minus_floorline_fast_ns =
        row.delta_raw_fast_residual_ns - row.delta_floorline_fast_residual_ns;
  }
}



static void floorline_science_attach_totals(clock_science_row_t& row,
                                            const floorline_science_totals_t& totals) {
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
  if (!clocks_ocxo_counterledger_mode_enabled()) return;

  const bool use_phase = ledger.refined_interval_valid &&
                         ledger.refined_interval_ns != 0ULL;

  if (CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL && !use_phase) {
    g_phaseledger_science_missing_refined_interval_count++;
    g_phaseledger_science_last_missing_public_count = row.public_count;
    if (ledger.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO1)) {
      g_phaseledger_science_missing_ocxo1_count++;
    } else if (ledger.clock_id ==
               (uint32_t)((uint8_t)time_clock_id_t::OCXO2)) {
      g_phaseledger_science_missing_ocxo2_count++;
    }

    row.valid = false;
    row.antecedents_complete = false;
    row.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
    row.gnss_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
    row.clock_interval_ns = 0ULL;
    row.clock_interval_ns_exact = 0.0;
    row.fast_residual_ns = 0LL;
    row.fast_residual_ns_exact = 0.0;
    row.tau_1s = 1.0;
    row.ppb_1s = 0.0;
    return;
  }

  const bool interval_valid = use_phase ||
      (ledger.interval_valid && ledger.interval_ns != 0ULL);
  const uint64_t interval_ns = use_phase
      ? ledger.refined_interval_ns
      : ledger.interval_ns;
  const int64_t fast_residual_ns = use_phase
      ? ledger.refined_fast_residual_ns
      : ledger.fast_residual_ns;

  row.valid = ledger.valid && interval_valid;
  row.antecedents_complete = row.valid;
  row.prior_edge_gnss_ns = beta_i64_from_u64_saturating(public_gnss_ns) -
                           beta_i64_from_u64_saturating(CLOCKS_BETA_NS_PER_SECOND);
  row.current_edge_gnss_ns = beta_i64_from_u64_saturating(public_gnss_ns);
  row.prior_edge_gnss_ns_exact = (double)row.prior_edge_gnss_ns;
  row.current_edge_gnss_ns_exact = (double)public_gnss_ns;

  row.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
  row.gnss_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
  row.clock_interval_ns = row.valid ? interval_ns : 0ULL;
  row.clock_interval_ns_exact = row.valid
      ? (double)interval_ns
      : 0.0;
  row.fast_residual_ns = row.valid ? fast_residual_ns : 0LL;
  row.fast_residual_ns_exact = row.valid
      ? (double)fast_residual_ns
      : 0.0;
  row.tau_1s = (row.valid && interval_ns != 0ULL)
      ? ((double)interval_ns / (double)CLOCKS_BETA_NS_PER_SECOND)
      : 1.0;
  row.ppb_1s = row.valid
      ? campaign_total_ppb_from_tau(row.tau_1s)
      : 0.0;
}

static void floorline_science_build_ocxo(
    clock_science_row_t& row,
    time_clock_id_t clock,
    uint32_t public_count,
    uint64_t public_gnss_ns,
    const delta_residual_reference_t& delayed_reference,
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool clock_valid,
    const clocks_alpha_lane_forensics_t& clock_f,
    floorline_science_totals_t& totals) {
  row = clock_science_row_t{};
  row.clock_id = (uint32_t)((uint8_t)clock);
  row.public_count = public_count;
  row.pps_vclock_dwt_at_edge = (uint32_t)g_dwt_at_pps_vclock;
  row.pps_vclock_gnss_ns_at_edge = public_gnss_ns;
  row.projection_cps_cycles = (uint32_t)g_dwt_cycles_between_pps_vclock;

  row.vclock_floorline_valid = floorline_candidate_present(vclock_valid, vclock_f);
  row.vclock_floorline_dwt_at_edge = row.vclock_floorline_valid
      ? vclock_f.regression_inferred_dwt_at_event
      : 0U;
  row.vclock_floorline_interval_cycles = row.vclock_floorline_valid
      ? floorline_inferred_interval_cycles(row.vclock_floorline_valid, vclock_f)
      : 0U;

  row.clock_floorline_valid = floorline_candidate_present(clock_valid, clock_f);
  row.clock_floorline_dwt_at_edge = row.clock_floorline_valid
      ? clock_f.regression_inferred_dwt_at_event
      : 0U;
  row.clock_floorline_interval_cycles = row.clock_floorline_valid
      ? floorline_inferred_interval_cycles(row.clock_floorline_valid, clock_f)
      : 0U;
  row.clock_published_dwt_at_edge = clock_valid ? clock_f.dwt_used_at_event : 0U;
  row.clock_raw_dwt_at_edge = clock_valid ? clock_f.dwt_original_at_event : 0U;
  row.clock_observed_interval_cycles = clock_valid ? clock_f.dwt_interval_observed_cycles : 0U;
  row.clock_effective_interval_cycles = clock_valid ? clock_f.dwt_interval_effective_cycles : 0U;

  if (row.clock_floorline_valid) {
    row.published_minus_floorline_cycles =
        beta_signed_delta_u32(row.clock_published_dwt_at_edge,
                              row.clock_floorline_dwt_at_edge);
    row.raw_minus_floorline_cycles =
        beta_signed_delta_u32(row.clock_raw_dwt_at_edge,
                              row.clock_floorline_dwt_at_edge);
    row.floorline_minus_observed_interval_cycles =
        beta_signed_delta_u32(row.clock_floorline_interval_cycles,
                              row.clock_observed_interval_cycles);
  }

  delta_residual_apply_one(row, delayed_reference, clock_valid, clock_f);

  const bool traditional_antecedents_valid =
      row.clock_floorline_valid &&
      row.clock_floorline_interval_cycles != 0U &&
      row.projection_cps_cycles != 0U;

  if (traditional_antecedents_valid) {
    const int32_t current_delta_cycles = beta_dwt32_signed_delta_near(
        row.pps_vclock_dwt_at_edge, row.clock_floorline_dwt_at_edge);
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
        ((double)row.clock_floorline_interval_cycles *
         (double)CLOCKS_BETA_NS_PER_SECOND) /
        (double)row.projection_cps_cycles;
    row.traditional_gnss_interval_ns = beta_dwt_cycles_to_gnss_ns_rounded(
        (uint64_t)row.clock_floorline_interval_cycles,
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
                             (double)row.clock_floorline_interval_cycles;
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

  // Delta Cycles is canonical.  A row is science-valid when the OCXO
  // observed edge-to-edge DWT interval can be compared against the delayed
  // selected PPS/VCLOCK DWT-edge reference covering the same physical second.
  // FloorLine is retained as a side rail only; it no longer authors TIMEBASE
  // residuals.
  row.valid = row.delta_raw_valid;
  row.antecedents_complete = row.delta_raw_valid &&
                             row.delta_raw_reference_interval_cycles != 0U &&
                             row.delta_raw_clock_interval_cycles != 0U;

  delta_residual_attach_comparisons(row);

  if (!row.valid) {
    floorline_science_attach_totals(row, totals);
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
  floorline_science_attach_totals(row, totals);
  return;
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

static bool campaign_start_prologue_fetch_forensics(
    clocks_alpha_lane_forensics_t& vclock_f,
    clocks_alpha_lane_forensics_t& ocxo1_f,
    clocks_alpha_lane_forensics_t& ocxo2_f,
    bool& vclock_valid,
    bool& ocxo1_valid,
    bool& ocxo2_valid) {
  vclock_valid = clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK, &vclock_f);
  ocxo1_valid = clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &ocxo1_f);
  ocxo2_valid = clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &ocxo2_f);

  if (!vclock_valid || !ocxo1_valid || !ocxo2_valid ||
      vclock_f.dwt_interval_observed_cycles == 0U ||
      ocxo1_f.dwt_interval_observed_cycles == 0U ||
      ocxo2_f.dwt_interval_observed_cycles == 0U ||
      !g_pps_vclock_dwt_cycles_between_edges_valid ||
      g_pps_vclock_dwt_cycles_between_edges == 0U ||
      g_dwt_cycles_between_pps_vclock == 0U) {
    campaign_start_prologue_set_reason("waiting_for_observed_delta_bookends");
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
  const bool v_fl = floorline_candidate_present(vclock_valid, vclock_f);
  const bool o1_fl = floorline_candidate_present(ocxo1_valid, ocxo1_f);
  const bool o2_fl = floorline_candidate_present(ocxo2_valid, ocxo2_f);

  if (!vclock_valid || !ocxo1_valid || !ocxo2_valid ||
      vclock_f.dwt_interval_observed_cycles == 0U ||
      ocxo1_f.dwt_interval_observed_cycles == 0U ||
      ocxo2_f.dwt_interval_observed_cycles == 0U ||
      !g_pps_vclock_dwt_cycles_between_edges_valid ||
      g_pps_vclock_dwt_cycles_between_edges == 0U ||
      g_dwt_cycles_between_pps_vclock == 0U) {
    campaign_start_prologue_set_reason("private_candidate_missing_observed_delta");
    return false;
  }

  // This candidate becomes the current private PPS 0 candidate.  Reset the
  // public presentation offsets to this edge every time we consume a private
  // candidate; if the following candidate qualifies, public PPS 1 will be the
  // one-second interval from this hidden bookend.
  campaign_public_offsets_reset_to_current();

  const delta_residual_reference_t ref =
      delta_residual_capture_vclock_reference(0U, vclock_valid, vclock_f);

  uint32_t ocxo1_endpoint_interval = 0U;
  (void)delta_floorline_endpoint_interval(
      g_delta_ocxo1_bookends,
      o1_fl,
      ocxo1_f.regression_inferred_dwt_at_event,
      &ocxo1_endpoint_interval);
  uint32_t ocxo2_endpoint_interval = 0U;
  (void)delta_floorline_endpoint_interval(
      g_delta_ocxo2_bookends,
      o2_fl,
      ocxo2_f.regression_inferred_dwt_at_event,
      &ocxo2_endpoint_interval);

  g_start_prologue_pps0_pps_obs =
      ref.gnss_valid ? ref.gnss_interval_cycles : 0U;
  g_start_prologue_pps0_v_obs =
      vclock_valid ? vclock_f.dwt_interval_observed_cycles : 0U;
  g_start_prologue_pps0_v_fl = ref.floorline_interval_cycles;
  g_start_prologue_pps0_o1_obs =
      ocxo1_valid ? ocxo1_f.dwt_interval_observed_cycles : 0U;
  g_start_prologue_pps0_o1_fl = ocxo1_endpoint_interval;
  g_start_prologue_pps0_o2_obs =
      ocxo2_valid ? ocxo2_f.dwt_interval_observed_cycles : 0U;
  g_start_prologue_pps0_o2_fl = ocxo2_endpoint_interval;
  g_start_prologue_pps0_interval_valid =
      g_start_prologue_pps0_pps_obs != 0U &&
      ref.gnss_valid &&
      vclock_f.dwt_interval_observed_cycles != 0U &&
      ocxo1_f.dwt_interval_observed_cycles != 0U &&
      ocxo2_f.dwt_interval_observed_cycles != 0U;

  g_delta_previous_vclock_reference = ref;
  g_start_prologue_seeded = true;
  g_start_prologue_reference_ready = ref.gnss_valid;
  g_start_prologue_private_candidate_count++;
  g_start_prologue_last_private_count = g_start_prologue_private_candidate_count;
  campaign_start_prologue_set_reason(reason ? reason : "private_pps0_consumed");
  return true;
}

static bool campaign_start_prologue_endpoint_fit_ok(
    const clock_science_row_t& row) {
  return row.valid &&
         row.delta_raw_reference_interval_cycles != 0U &&
         row.delta_raw_clock_interval_cycles != 0U;
}

static bool campaign_start_prologue_reference_fit_ok(
    const delta_residual_reference_t& ref) {
  return ref.captured && ref.gnss_valid && ref.gnss_interval_cycles != 0U;
}

static bool campaign_start_prologue_interval_fit_ok(uint32_t current,
                                                    uint32_t previous) {
  if (current == 0U || previous == 0U) return false;
  return beta_abs_i32_within_gate(
      beta_signed_delta_u32(current, previous),
      CLOCKS_START_PROLOGUE_FIT_ENDPOINT_GATE_CYCLES);
}

static bool campaign_start_prologue_private_pps0_continuity_ok(
    bool vclock_valid,
    const clocks_alpha_lane_forensics_t& vclock_f,
    bool ocxo1_valid,
    const clocks_alpha_lane_forensics_t& ocxo1_f,
    bool ocxo2_valid,
    const clocks_alpha_lane_forensics_t& ocxo2_f) {
  g_start_prologue_continuity_check_count++;
  g_start_prologue_last_continuity_ok = false;

  const uint32_t current_reference_interval =
      g_pps_vclock_dwt_cycles_between_edges_valid
          ? (uint32_t)g_pps_vclock_dwt_cycles_between_edges
          : 0U;
  const uint32_t current_vclock_interval =
      vclock_valid ? vclock_f.dwt_interval_observed_cycles : 0U;
  const uint32_t current_ocxo1_interval =
      ocxo1_valid ? ocxo1_f.dwt_interval_observed_cycles : 0U;
  const uint32_t current_ocxo2_interval =
      ocxo2_valid ? ocxo2_f.dwt_interval_observed_cycles : 0U;

  g_start_prologue_last_reference_interval = current_reference_interval;
  g_start_prologue_last_vclock_interval = current_vclock_interval;
  g_start_prologue_last_ocxo1_interval = current_ocxo1_interval;
  g_start_prologue_last_ocxo2_interval = current_ocxo2_interval;
  g_start_prologue_last_reference_minus_pps0 =
      beta_signed_delta_u32(current_reference_interval,
                            g_start_prologue_pps0_pps_obs);
  g_start_prologue_last_vclock_minus_pps0 =
      beta_signed_delta_u32(current_vclock_interval,
                            g_start_prologue_pps0_v_obs);
  g_start_prologue_last_ocxo1_minus_pps0 =
      beta_signed_delta_u32(current_ocxo1_interval,
                            g_start_prologue_pps0_o1_obs);
  g_start_prologue_last_ocxo2_minus_pps0 =
      beta_signed_delta_u32(current_ocxo2_interval,
                            g_start_prologue_pps0_o2_obs);

  const auto reject = [](const char* reason) -> bool {
    g_start_prologue_continuity_reject_count++;
    campaign_start_prologue_set_reason(reason);
    return false;
  };

  if (!g_start_prologue_pps0_interval_valid) {
    return reject("private_pps0_interval_missing");
  }

  if (!campaign_start_prologue_interval_fit_ok(
          current_reference_interval,
          g_start_prologue_pps0_pps_obs)) {
    return reject("private_pps0_reference_interval_unsettled");
  }

  if (!vclock_valid ||
      !campaign_start_prologue_interval_fit_ok(
          current_vclock_interval,
          g_start_prologue_pps0_v_obs)) {
    return reject("private_pps0_vclock_interval_unsettled");
  }

  if (!ocxo1_valid ||
      !campaign_start_prologue_interval_fit_ok(
          current_ocxo1_interval,
          g_start_prologue_pps0_o1_obs)) {
    return reject("private_pps0_ocxo1_interval_unsettled");
  }

  if (!ocxo2_valid ||
      !campaign_start_prologue_interval_fit_ok(
          current_ocxo2_interval,
          g_start_prologue_pps0_o2_obs)) {
    return reject("private_pps0_ocxo2_interval_unsettled");
  }

  g_start_prologue_last_continuity_ok = true;
  g_start_prologue_continuity_pass_count++;
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

  if (!campaign_start_prologue_reference_fit_ok(g_delta_previous_vclock_reference)) {
    campaign_start_prologue_set_reason("private_gnss_reference_missing");
    return false;
  }

  // Public PPS 1 must not be allowed to inherit a startup/transient private
  // PPS 0 interval.  Delta Cycles intentionally compares the OCXO interval
  // published on the current PPS/VCLOCK row against the previous private
  // PPS/VCLOCK reference.  If that hidden predecessor was captured across
  // SmartZero/launch acquisition turbulence, the first public residual can be
  // poisoned even though all current-row antecedents are nonzero and lawful.
  // Require the private PPS0 interval and the candidate PPS1 interval to be
  // continuous on every observed rail; otherwise promote this candidate to the
  // new private PPS0 and try again on the next row.
  if (!campaign_start_prologue_private_pps0_continuity_ok(
          vclock_valid, vclock_f, ocxo1_valid, ocxo1_f,
          ocxo2_valid, ocxo2_f)) {
    return false;
  }

  const delta_residual_bookend_t saved_o1 = g_delta_ocxo1_bookends;
  const delta_residual_bookend_t saved_o2 = g_delta_ocxo2_bookends;
  g_beta_probe_floorline_o1 = floorline_science_totals_t{};
  g_beta_probe_floorline_o2 = floorline_science_totals_t{};

  clock_science_row_t& o1 = g_beta_probe_ocxo1_science_row;
  clock_science_row_t& o2 = g_beta_probe_ocxo2_science_row;
  floorline_science_build_ocxo(
      o1,
      time_clock_id_t::OCXO1,
      1U,
      CLOCKS_BETA_NS_PER_SECOND,
      g_delta_previous_vclock_reference,
      vclock_valid,
      vclock_f,
      ocxo1_valid,
      ocxo1_f,
      g_beta_probe_floorline_o1);
  floorline_science_build_ocxo(
      o2,
      time_clock_id_t::OCXO2,
      1U,
      CLOCKS_BETA_NS_PER_SECOND,
      g_delta_previous_vclock_reference,
      vclock_valid,
      vclock_f,
      ocxo2_valid,
      ocxo2_f,
      g_beta_probe_floorline_o2);

  // Probe only: restore the real bookends so the public path can consume this
  // exact candidate once, with Welfords/totals still at n=0.
  g_delta_ocxo1_bookends = saved_o1;
  g_delta_ocxo2_bookends = saved_o2;

  if (!o1.valid || !o2.valid) {
    campaign_start_prologue_set_reason("waiting_for_delta_observed_valid");
    return false;
  }
  if (!campaign_start_prologue_endpoint_fit_ok(o1)) {
    campaign_start_prologue_set_reason("ocxo1_observed_interval_missing");
    return false;
  }
  if (!campaign_start_prologue_endpoint_fit_ok(o2)) {
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
  request_servo_mode_change = false;
  requested_servo_mode = servo_mode_t::OFF;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  ocxo_dac_pacing_abort_all();
  timebase_invalidate();
}

static bool campaign_start_prologue_should_hold(void) {
  clocks_alpha_lane_forensics_t& vclock_f = g_beta_start_vclock_forensics;
  clocks_alpha_lane_forensics_t& ocxo1_f = g_beta_start_ocxo1_forensics;
  clocks_alpha_lane_forensics_t& ocxo2_f = g_beta_start_ocxo2_forensics;
  vclock_f = clocks_alpha_lane_forensics_t{};
  ocxo1_f = clocks_alpha_lane_forensics_t{};
  ocxo2_f = clocks_alpha_lane_forensics_t{};
  bool vclock_valid = false;
  bool ocxo1_valid = false;
  bool ocxo2_valid = false;

  if (clocks_ocxo_counterledger_mode_enabled()) {
    if (!campaign_start_handoff_ready()) {
      if (g_start_handoff_launch_wait_count != UINT32_MAX) {
        g_start_handoff_launch_wait_count++;
      }
      if (g_start_handoff_launch_wait_count >=
          CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES) {
        campaign_start_prologue_abort_launch("phaseledger_start_handoff_timeout");
        return true;
      }
      campaign_start_prologue_set_reason(g_start_phaseledger_last_reason);
      return true;
    }

    g_start_handoff_launch_wait_count = 0;

    if (!campaign_start_prologue_fetch_forensics(vclock_f, ocxo1_f, ocxo2_f,
                                                 vclock_valid, ocxo1_valid,
                                                 ocxo2_valid)) {
      return true;
    }

    // CounterLedger/PhaseLedger uses the hardware counter + phase suffix for
    // public OCXO ns, but public PPS1 is still an interval population sample.
    // Use the same private PPS0 -> candidate PPS1 observed-DWT continuity
    // witness that protects Delta Cycles.  DWT is not authoring the public
    // PhaseLedger value here; it is only deciding whether the launch bookend is
    // clean enough to enter the public campaign population.
    if (campaign_start_prologue_probe_public_pps1(vclock_valid, vclock_f,
                                                  ocxo1_valid, ocxo1_f,
                                                  ocxo2_valid, ocxo2_f)) {
      g_start_handoff_commit_count++;
      campaign_start_prologue_set_reason("phaseledger_release_public_pps1");
      return false;
    }

    const bool private_limit_reached =
        g_start_prologue_private_candidate_count >=
        CLOCKS_START_PROLOGUE_MAX_PRIVATE_CANDIDATES;
    if (private_limit_reached) {
      g_start_prologue_private_limit_count++;
    }

    (void)campaign_start_prologue_consume_private_candidate(
        vclock_valid,
        vclock_f,
        ocxo1_valid,
        ocxo1_f,
        ocxo2_valid,
        ocxo2_f,
        private_limit_reached
            ? "phaseledger_private_limit_still_holding"
            : (g_start_prologue_reference_ready
                  ? "phaseledger_private_pps0_refreshed_after_probe_reject"
                  : "phaseledger_private_pps0_seeded"));
    return true;
  }

  if (!campaign_start_handoff_ready()) {
    if (g_start_handoff_launch_wait_count != UINT32_MAX) {
      g_start_handoff_launch_wait_count++;
    }
    if (g_start_handoff_launch_wait_count >=
        CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES) {
      campaign_start_prologue_abort_launch("start_handoff_timeout");
      return true;
    }
    campaign_start_prologue_set_reason("waiting_for_start_handoff");
    return true;
  }
  g_start_handoff_launch_wait_count = 0;

  if (!campaign_start_prologue_fetch_forensics(vclock_f, ocxo1_f, ocxo2_f,
                                               vclock_valid, ocxo1_valid,
                                               ocxo2_valid)) {
    return true;
  }

  if (campaign_start_prologue_probe_public_pps1(vclock_valid, vclock_f,
                                                ocxo1_valid, ocxo1_f,
                                                ocxo2_valid, ocxo2_f)) {
    g_start_handoff_commit_count++;
    return false;
  }

  const bool private_limit_reached =
      g_start_prologue_private_candidate_count >=
      CLOCKS_START_PROLOGUE_MAX_PRIVATE_CANDIDATES;
  if (private_limit_reached) {
    g_start_prologue_private_limit_count++;
  }

  (void)campaign_start_prologue_consume_private_candidate(
      vclock_valid,
      vclock_f,
      ocxo1_valid,
      ocxo1_f,
      ocxo2_valid,
      ocxo2_f,
      private_limit_reached
          ? "private_limit_still_holding"
          : (g_start_prologue_reference_ready
                ? "private_pps0_refreshed_after_probe_reject"
                : "private_pps0_seeded"));
  return true;
}

static FLASHMEM void payload_add_clock_science_common(Payload& science,
                                             const clock_science_row_t& row) {
  const bool ocxo_science_row =
      row.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO1) ||
      row.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO2);

  if (TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED) {
    science.add("schema", "TIMEBASE_CLOCK_SCIENCE_COMPACT_V2");
    science.add("valid", row.valid);
    science.add("antecedents_complete", row.antecedents_complete);
    science.add("clock_id", row.clock_id);
    science.add("public_count", row.public_count);
    science.add("positive_means", "clock_fast");
    science.add("public_ns_mode", clocks_ocxo_public_ns_authority_name());

    // Compact mode must stay comfortably below the 16 KB Payload arena.
    // Keep only the scalar science needed by dashboards/analyzers.  Rich
    // provenance strings, projection antecedents, AlphaTau, traditional totals,
    // and recovery-continuity diagnostics live in TIMEBASE_FORENSICS or focused
    // reports.
    science.add("gnss_interval_ns", row.gnss_interval_ns);
    science.add("clock_interval_ns", row.clock_interval_ns);
    science.add("fast_residual_ns", row.fast_residual_ns);
    science.add("fast_residual_ns_exact", row.fast_residual_ns_exact, 6);
    science.add("tau_1s", row.tau_1s, 12);
    science.add("ppb_1s", row.ppb_1s, 6);

    // Campaign-total frequency summary.  These five fields are the panel-facing
    // public clockface ratio and must remain in TIMEBASE_FRAGMENT.
    science.add("total_valid", row.total_valid);
    science.add("total_sample_count", row.total_sample_count);
    science.add("total_fast_residual_ns", row.total_fast_residual_ns);
    science.add("total_tau", row.total_tau, 12);
    science.add("total_ppb", row.total_ppb, 6);

    // Canonical Delta residual identity.  Keep this numeric and compact; verbose
    // species labels and FloorLine/traditional comparison rails are forensics.
    const bool publish_delta = row.delta_reference_public_count != 0U ||
                               row.delta_publication_public_count != 0U ||
                               row.delta_raw_valid ||
                               row.delta_floorline_valid;
    if (publish_delta) {
      science.add("delta_reference_public_count", row.delta_reference_public_count);
      science.add("delta_publication_public_count", row.delta_publication_public_count);
      science.add("delta_raw_valid", row.delta_raw_valid);
      science.add("delta_raw_reference_interval_cycles",
                  row.delta_raw_reference_interval_cycles);
      science.add("delta_raw_clock_interval_cycles",
                  row.delta_raw_clock_interval_cycles);
      science.add("delta_raw_fast_residual_ns_exact",
                  row.delta_raw_fast_residual_ns_exact, 6);
    }
    return;
  }

  science.add("schema", "TIMEBASE_CLOCK_SCIENCE_V2");
  science.add("valid", row.valid);
  science.add("antecedents_complete", row.antecedents_complete);
  science.add("clock_id", row.clock_id);
  science.add("public_count", row.public_count);
  science.add("positive_means", "clock_fast");
  science.add("public_ns_mode", clocks_ocxo_public_ns_authority_name());
  science.add("residual_source",
              ocxo_science_row
                  ? (clocks_ocxo_counterledger_mode_enabled()
                         ? "COUNTERLEDGER_PPS_CAPTURE"
                         : "DELTA_OBSERVED_DWT_EDGE")
                  : "GNSS_IDENTITY");
  science.add("traditional_residual_source",
              ocxo_science_row ? "PROJECTED_GNSS_FLOORLINE_DWT_CPS" : "NONE");

  science.add("pps_vclock_dwt_at_edge", row.pps_vclock_dwt_at_edge);
  science.add("pps_vclock_gnss_ns_at_edge", row.pps_vclock_gnss_ns_at_edge);
  science.add("projection_cps_cycles", row.projection_cps_cycles);

  science.add("vclock_floorline_valid", row.vclock_floorline_valid);
  science.add("vclock_floorline_dwt_at_edge", row.vclock_floorline_dwt_at_edge);
  science.add("vclock_floorline_interval_cycles", row.vclock_floorline_interval_cycles);

  science.add("clock_floorline_valid", row.clock_floorline_valid);
  science.add("clock_floorline_dwt_at_edge", row.clock_floorline_dwt_at_edge);
  science.add("clock_floorline_interval_cycles", row.clock_floorline_interval_cycles);
  science.add("clock_published_dwt_at_edge", row.clock_published_dwt_at_edge);
  science.add("clock_raw_dwt_at_edge", row.clock_raw_dwt_at_edge);
  science.add("clock_observed_interval_cycles", row.clock_observed_interval_cycles);
  science.add("clock_effective_interval_cycles", row.clock_effective_interval_cycles);
  science.add("published_minus_floorline_cycles", row.published_minus_floorline_cycles);
  science.add("raw_minus_floorline_cycles", row.raw_minus_floorline_cycles);
  science.add("floorline_minus_observed_interval_cycles",
              row.floorline_minus_observed_interval_cycles);

  science.add("delta_alignment", "REFERENCE_SAME_TIMEBASE_ROW");
  science.add("delta_reference_species", "SELECTED_PPS_VCLOCK_DWT_EDGE_INTERVAL");
  science.add("delta_clock_species", "OCXO_OBSERVED_DWT_EDGE_INTERVAL");
  science.add("delta_positive_means", "clock_slow_for_residual_clock_minus_reference");
  science.add("delta_fast_positive_means", "clock_fast");
  science.add("delta_reference_public_count", row.delta_reference_public_count);
  science.add("delta_publication_public_count", row.delta_publication_public_count);

  science.add("delta_raw_valid", row.delta_raw_valid);
  science.add("delta_raw_reference_interval_cycles",
              row.delta_raw_reference_interval_cycles);
  science.add("delta_raw_clock_interval_cycles",
              row.delta_raw_clock_interval_cycles);
  science.add("delta_raw_residual_cycles", row.delta_raw_residual_cycles);
  science.add("delta_raw_fast_residual_cycles",
              row.delta_raw_fast_residual_cycles);
  science.add("delta_raw_residual_ns", row.delta_raw_residual_ns);
  science.add("delta_raw_fast_residual_ns",
              row.delta_raw_fast_residual_ns);
  science.add("delta_raw_residual_ns_exact",
              row.delta_raw_residual_ns_exact, 6);
  science.add("delta_raw_fast_residual_ns_exact",
              row.delta_raw_fast_residual_ns_exact, 6);

  science.add("delta_floorline_valid", row.delta_floorline_valid);
  science.add("delta_floorline_reference_interval_cycles",
              row.delta_floorline_reference_interval_cycles);
  science.add("delta_floorline_clock_interval_cycles",
              row.delta_floorline_clock_interval_cycles);
  science.add("delta_floorline_residual_cycles",
              row.delta_floorline_residual_cycles);
  science.add("delta_floorline_fast_residual_cycles",
              row.delta_floorline_fast_residual_cycles);
  science.add("delta_floorline_residual_ns",
              row.delta_floorline_residual_ns);
  science.add("delta_floorline_fast_residual_ns",
              row.delta_floorline_fast_residual_ns);
  science.add("delta_floorline_residual_ns_exact",
              row.delta_floorline_residual_ns_exact, 6);
  science.add("delta_floorline_fast_residual_ns_exact",
              row.delta_floorline_fast_residual_ns_exact, 6);

  science.add("delta_raw_fast_minus_traditional_ns",
              row.delta_raw_fast_minus_traditional_ns);
  science.add("delta_floorline_fast_minus_traditional_ns",
              row.delta_floorline_fast_minus_traditional_ns);
  science.add("delta_raw_fast_minus_floorline_fast_ns",
              row.delta_raw_fast_minus_floorline_fast_ns);

  science.add("prior_edge_gnss_ns", row.prior_edge_gnss_ns);
  science.add("current_edge_gnss_ns", row.current_edge_gnss_ns);
  science.add("prior_edge_gnss_ns_exact", row.prior_edge_gnss_ns_exact, 6);
  science.add("current_edge_gnss_ns_exact", row.current_edge_gnss_ns_exact, 6);
  science.add("gnss_interval_ns", row.gnss_interval_ns);
  science.add("clock_interval_ns", row.clock_interval_ns);
  science.add("fast_residual_ns", row.fast_residual_ns);
  science.add("gnss_interval_ns_exact", row.gnss_interval_ns_exact, 6);
  science.add("clock_interval_ns_exact", row.clock_interval_ns_exact, 6);
  science.add("fast_residual_ns_exact", row.fast_residual_ns_exact, 6);
  science.add("tau_1s", row.tau_1s, 12);
  science.add("ppb_1s", row.ppb_1s, 6);

  science.add("traditional_valid", row.traditional_valid);
  science.add("traditional_gnss_interval_ns", row.traditional_gnss_interval_ns);
  science.add("traditional_clock_interval_ns", row.traditional_clock_interval_ns);
  science.add("traditional_fast_residual_ns", row.traditional_fast_residual_ns);
  science.add("traditional_gnss_interval_ns_exact",
              row.traditional_gnss_interval_ns_exact, 6);
  science.add("traditional_clock_interval_ns_exact",
              row.traditional_clock_interval_ns_exact, 6);
  science.add("traditional_fast_residual_ns_exact",
              row.traditional_fast_residual_ns_exact, 6);
  science.add("traditional_tau_1s", row.traditional_tau_1s, 12);
  science.add("traditional_ppb_1s", row.traditional_ppb_1s, 6);

  const bool recovery_continuity_aligned =
      ocxo_science_row &&
      row.public_count != 0U &&
      row.public_count == g_recover_continuity_align_last_public_count;
  science.add("total_ratio_source",
              ocxo_science_row
                  ? (recovery_continuity_aligned
                         ? "RECOVERY_ALIGNED_PUBLIC_CLOCK_NS_OVER_GNSS_NS"
                         : "PUBLIC_CLOCK_NS_OVER_GNSS_NS")
                  : "GNSS_IDENTITY");
  science.add("total_valid", row.total_valid);
  science.add("total_sample_count", row.total_sample_count);
  science.add("total_clock_interval_ns", row.total_clock_interval_ns);
  science.add("total_gnss_interval_ns", row.total_gnss_interval_ns);
  science.add("total_fast_residual_ns", row.total_fast_residual_ns);
  science.add("total_clock_interval_ns_exact", row.total_clock_interval_ns_exact, 6);
  science.add("total_gnss_interval_ns_exact", row.total_gnss_interval_ns_exact, 6);
  science.add("total_fast_residual_ns_exact", row.total_fast_residual_ns_exact, 6);
  science.add("total_tau", row.total_tau, 12);
  science.add("total_ppb", row.total_ppb, 6);
  science.add("recovery_continuity_aligned", recovery_continuity_aligned);
  science.add("recovery_continuity_target_ns",
              recovery_continuity_aligned
                  ? (row.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO1)
                         ? g_recover_continuity_ocxo1_target_ns
                         : g_recover_continuity_ocxo2_target_ns)
                  : 0ULL);
  science.add("recovery_continuity_correction_ns",
              recovery_continuity_aligned
                  ? (row.clock_id == (uint32_t)((uint8_t)time_clock_id_t::OCXO1)
                         ? g_recover_continuity_ocxo1_correction_ns
                         : g_recover_continuity_ocxo2_correction_ns)
                  : 0LL);
  science.add("alpha_tau_valid", row.alpha_tau_valid);
  science.add("alpha_tau_sample_count", row.alpha_tau_sample_count);
  science.add("alpha_tau_interval_count", row.alpha_tau_interval_count);
  science.add("alpha_tau_last_pps_sequence", row.alpha_tau_last_pps_sequence);
  science.add("alpha_tau_last_interval_pps_sequence",
              row.alpha_tau_last_interval_pps_sequence);
  science.add("alpha_tau", row.alpha_tau, 12);
  science.add("alpha_tau_ppb", row.alpha_tau_ppb, 6);
  science.add("alpha_tau_stderr_ppb", row.alpha_tau_stderr_ppb, 6);
  science.add("alpha_tau_interval_mean_ppb",
              row.alpha_tau_interval_mean_ppb, 6);
  science.add("alpha_tau_interval_stderr_ppb",
              row.alpha_tau_interval_stderr_ppb, 6);
  science.add("alpha_tau_intercept_ns", row.alpha_tau_intercept_ns);
  science.add("alpha_tau_detrended_fast_residual_ns",
              row.alpha_tau_detrended_fast_residual_ns);

  science.add("traditional_total_valid", row.traditional_total_valid);
  science.add("traditional_total_sample_count", row.traditional_total_sample_count);
  science.add("traditional_total_clock_interval_ns",
              row.traditional_total_clock_interval_ns);
  science.add("traditional_total_gnss_interval_ns",
              row.traditional_total_gnss_interval_ns);
  science.add("traditional_total_fast_residual_ns",
              row.traditional_total_fast_residual_ns);
  science.add("traditional_total_clock_interval_ns_exact",
              row.traditional_total_clock_interval_ns_exact, 6);
  science.add("traditional_total_gnss_interval_ns_exact",
              row.traditional_total_gnss_interval_ns_exact, 6);
  science.add("traditional_total_fast_residual_ns_exact",
              row.traditional_total_fast_residual_ns_exact, 6);
  science.add("traditional_total_tau", row.traditional_total_tau, 12);
  science.add("traditional_total_ppb", row.traditional_total_ppb, 6);
}

static FLASHMEM void payload_add_vclock_science_object(Payload& lane,
                                              uint32_t public_count,
                                              uint64_t public_gnss_ns,
                                              bool vclock_valid,
                                              const clocks_alpha_lane_forensics_t& vclock_f) {
  Payload science;
  clock_science_row_t& row = g_beta_vclock_science_row;
  row = clock_science_row_t{};
  row.valid = true;
  row.antecedents_complete = true;
  row.clock_id = (uint32_t)((uint8_t)time_clock_id_t::VCLOCK);
  row.public_count = public_count;
  row.pps_vclock_dwt_at_edge = (uint32_t)g_dwt_at_pps_vclock;
  row.pps_vclock_gnss_ns_at_edge = public_gnss_ns;
  row.projection_cps_cycles = (uint32_t)g_dwt_cycles_between_pps_vclock;
  row.vclock_floorline_valid = floorline_candidate_present(vclock_valid, vclock_f);
  row.vclock_floorline_dwt_at_edge = row.vclock_floorline_valid
      ? vclock_f.regression_inferred_dwt_at_event
      : 0U;
  row.vclock_floorline_interval_cycles = row.vclock_floorline_valid
      ? floorline_inferred_interval_cycles(row.vclock_floorline_valid, vclock_f)
      : 0U;
  row.clock_floorline_valid = row.vclock_floorline_valid;
  row.clock_floorline_dwt_at_edge = row.vclock_floorline_dwt_at_edge;
  row.clock_floorline_interval_cycles = row.vclock_floorline_interval_cycles;
  row.clock_published_dwt_at_edge = vclock_valid ? vclock_f.dwt_used_at_event : 0U;
  row.clock_raw_dwt_at_edge = vclock_valid ? vclock_f.dwt_original_at_event : 0U;
  row.clock_observed_interval_cycles = vclock_valid ? vclock_f.dwt_interval_observed_cycles : 0U;
  row.clock_effective_interval_cycles = vclock_valid ? vclock_f.dwt_interval_effective_cycles : 0U;
  if (row.clock_floorline_valid) {
    row.published_minus_floorline_cycles =
        beta_signed_delta_u32(row.clock_published_dwt_at_edge,
                              row.clock_floorline_dwt_at_edge);
    row.raw_minus_floorline_cycles =
        beta_signed_delta_u32(row.clock_raw_dwt_at_edge,
                              row.clock_floorline_dwt_at_edge);
    row.floorline_minus_observed_interval_cycles =
        beta_signed_delta_u32(row.clock_floorline_interval_cycles,
                              row.clock_observed_interval_cycles);
  }
  row.prior_edge_gnss_ns = beta_i64_from_u64_saturating(public_gnss_ns) -
                           beta_i64_from_u64_saturating(CLOCKS_BETA_NS_PER_SECOND);
  row.current_edge_gnss_ns = beta_i64_from_u64_saturating(public_gnss_ns);
  row.prior_edge_gnss_ns_exact = (double)row.prior_edge_gnss_ns;
  row.current_edge_gnss_ns_exact = (double)public_gnss_ns;
  row.gnss_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
  row.clock_interval_ns = CLOCKS_BETA_NS_PER_SECOND;
  row.fast_residual_ns = 0;
  row.gnss_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
  row.clock_interval_ns_exact = (double)CLOCKS_BETA_NS_PER_SECOND;
  row.fast_residual_ns_exact = 0.0;
  row.tau_1s = 1.0;
  row.ppb_1s = 0.0;
  row.total_valid = public_gnss_ns != 0ULL;
  row.total_sample_count = public_count;
  row.total_clock_interval_ns = public_gnss_ns;
  row.total_gnss_interval_ns = public_gnss_ns;
  row.total_fast_residual_ns = 0;
  row.total_clock_interval_ns_exact = (double)public_gnss_ns;
  row.total_gnss_interval_ns_exact = (double)public_gnss_ns;
  row.total_fast_residual_ns_exact = 0.0;
  row.total_tau = 1.0;
  row.total_ppb = 0.0;

  payload_add_clock_science_common(science, row);
  science.add("edge_species", TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED ? "PPS_VCLOCK" : "PPS_VCLOCK_AUTHORITY");
  science.add("frequency_source", TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED ? "GNSS" : "GNSS_IDENTITY");
  lane.add_object("science", science);
}

static FLASHMEM void payload_add_ocxo_science_object(Payload& lane,
                                            const clock_science_row_t& row,
                                            bool reported_valid,
                                            uint64_t reported_gnss_interval_ns,
                                            uint64_t reported_clock_interval_ns,
                                            int64_t reported_fast_residual_ns) {
  Payload science;
  payload_add_clock_science_common(science, row);
  science.add("edge_species",
              TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED
                  ? "OBSERVED_DWT_EDGE"
                  : "OBSERVED_DWT_EDGE_AUTHORITY");
  science.add("frequency_source",
              clocks_ocxo_counterledger_mode_enabled()
                  ? "COUNTERLEDGER_PPS_CAPTURE"
                  : (TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED
                         ? "DELTA_OBSERVED_DWT_EDGE"
                         : "DELTA_CYCLES_OBSERVED_DWT_REFERENCE_DELAY_1"));
  if (!TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED) {
    science.add("delta_formula",
                "fast = delayed_gnss_pps_cycles - ocxo_observed_dwt_cycles");
    science.add("traditional_projection_formula",
                "pps_vclock_gnss_ns + signed(clock_floorline_dwt - pps_vclock_dwt) * 1e9 / projection_cps");
  }
  science.add("reported_pps_residual_valid", reported_valid);
  science.add("reported_pps_gnss_interval_ns",
              reported_valid ? reported_gnss_interval_ns : 0ULL);
  science.add("reported_pps_clock_interval_ns",
              reported_valid ? reported_clock_interval_ns : 0ULL);
  science.add("reported_pps_fast_residual_ns",
              reported_valid ? reported_fast_residual_ns : 0LL);
  science.add("reported_minus_canonical_residual_ns",
              (reported_valid && row.valid)
                  ? (reported_fast_residual_ns - row.fast_residual_ns)
                  : 0LL);
  science.add("reported_minus_observed_residual_ns",
              (reported_valid && row.valid)
                  ? (reported_fast_residual_ns - row.fast_residual_ns)
                  : 0LL);
  science.add("reported_minus_floorline_residual_ns",
              (reported_valid && row.delta_floorline_valid)
                  ? (reported_fast_residual_ns - row.delta_floorline_fast_residual_ns)
                  : 0LL);
  lane.add_object("science", science);
}

static FLASHMEM void payload_add_slim_dwt_lane_forensics(
    Payload& parent,
    bool valid,
    const clocks_alpha_lane_forensics_t& f) {
  Payload dwt;
  dwt.add("valid", valid);
  dwt.add("update_count", valid ? f.update_count : 0U);
  dwt.add("last_event_dwt", valid ? f.last_event_dwt : 0U);
  dwt.add("last_event_counter32", valid ? f.last_event_counter32 : 0U);
  dwt.add("counter32_delta_since_previous_event",
          valid ? f.counter32_delta_since_previous_event : 0U);

  dwt.add("cycles_between_edges", valid ? f.dwt_cycles_between_edges : 0U);
  dwt.add("original_at_event", valid ? f.dwt_original_at_event : 0U);
  dwt.add("predicted_at_event", valid ? f.dwt_predicted_at_event : 0U);
  dwt.add("ema_dwt_at_event", valid ? f.dwt_ema_dwt_at_event : 0U);
  dwt.add("used_at_event", valid ? f.dwt_used_at_event : 0U);
  dwt.add("isr_entry_raw", valid ? f.dwt_isr_entry_raw : 0U);
  dwt.add("event_from_isr_entry_raw",
          valid ? f.dwt_event_from_isr_entry_raw : 0U);
  dwt.add("isr_entry_to_event_correction_cycles",
          valid ? f.dwt_isr_entry_to_event_correction_cycles : 0);
  dwt.add("published_minus_event_cycles",
          valid ? f.dwt_published_minus_event_cycles : 0);
  dwt.add("used_minus_event_cycles",
          valid ? f.dwt_used_minus_event_cycles : 0);
  dwt.add("synthetic", valid && f.dwt_synthetic);
  dwt.add("synthetic_error_cycles",
          valid ? f.dwt_synthetic_error_cycles : 0);
  dwt.add("synthetic_threshold_cycles",
          valid ? f.dwt_synthetic_threshold_cycles : 0U);

  Payload gate;
  gate.add("valid", valid && f.dwt_interval_gate_valid);
  gate.add("accepted", valid && f.dwt_interval_sample_accepted);
  gate.add("rejected", valid && f.dwt_interval_sample_rejected);
  gate.add("ema_updated", valid && f.dwt_interval_ema_updated);
  gate.add("observed_cycles", valid ? f.dwt_interval_observed_cycles : 0U);
  gate.add("prediction_cycles", valid ? f.dwt_interval_prediction_cycles : 0U);
  gate.add("effective_cycles", valid ? f.dwt_interval_effective_cycles : 0U);
  gate.add("residual_cycles", valid ? f.dwt_interval_residual_cycles : 0);
  gate.add("threshold_cycles", valid ? f.dwt_interval_gate_threshold_cycles : 0U);
  gate.add("accept_count", valid ? f.dwt_interval_accept_count : 0U);
  gate.add("reject_count", valid ? f.dwt_interval_reject_count : 0U);
  gate.add("reject_streak", valid ? f.dwt_interval_reject_streak : 0U);
  dwt.add_object("interval_gate", gate);

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
  dwt.add_object("adjacency", adjacency);

  Payload yardstick;
  yardstick.add("valid", valid && f.dwt_yardstick_valid);
  yardstick.add("stale", valid && f.dwt_yardstick_stale);
  yardstick.add("seeded", valid && f.dwt_yardstick_seeded);
  yardstick.add("excursion", valid && f.dwt_yardstick_excursion);
  yardstick.add("pps_sequence", valid ? f.dwt_yardstick_pps_sequence : 0U);
  yardstick.add("pps_seq_delta", valid ? f.dwt_yardstick_pps_seq_delta : 0U);
  yardstick.add("g_now_cycles", valid ? f.dwt_yardstick_g_now_cycles : 0U);
  yardstick.add("g_prev_cycles", valid ? f.dwt_yardstick_g_prev_cycles : 0U);
  yardstick.add("inferred_interval_cycles",
                valid ? f.dwt_yardstick_inferred_interval_cycles : 0U);
  yardstick.add("observed_interval_cycles",
                valid ? f.dwt_yardstick_observed_interval_cycles : 0U);
  yardstick.add("inferred_minus_observed_cycles",
                valid ? f.dwt_yardstick_inferred_minus_observed_cycles : 0);
  yardstick.add("endpoint_minus_observed_cycles",
                valid ? f.dwt_yardstick_endpoint_minus_observed_cycles : 0);
  yardstick.add("gate_threshold_cycles",
                valid ? f.dwt_yardstick_gate_threshold_cycles : 0U);
  yardstick.add("gate_agree_count",
                valid ? f.dwt_yardstick_gate_agree_count : 0U);
  yardstick.add("gate_excursion_count",
                valid ? f.dwt_yardstick_gate_excursion_count : 0U);
  yardstick.add("authority", valid && f.dwt_yardstick_authority);
  yardstick.add("auth_error_cycles",
                valid ? f.dwt_yardstick_auth_error_cycles : 0);
  yardstick.add("auth_anchor_applied",
                valid && f.dwt_yardstick_auth_anchor_applied);
  dwt.add_object("yardstick", yardstick);

  if (TIMEBASE_FORENSICS_FLOORLINE_PAYLOAD_ENABLED) {
    payload_add_floorline_object(dwt, valid, f);
  }

  parent.add_object("dwt_forensics", dwt);
}

static FLASHMEM void payload_add_slim_pps_vclock_edge_forensics(
    Payload& parent,
    bool available,
    const clocks_pps_vclock_edge_forensics_t& e) {
  Payload edge;
  edge.add("available", available);
  edge.add("valid", available && e.valid);
  edge.add("sequence", available ? e.sequence : 0U);
  edge.add("update_count", available ? e.update_count : 0U);
  edge.add("reject_count", available ? e.reject_count : 0U);
  edge.add("decision", pps_vclock_edge_decision_name(available ? e.decision : 0U));
  edge.add("decision_id", available ? e.decision : 0U);
  edge.add("invalid_mask", available ? e.invalid_mask : 0U);

  edge.add("authority_dwt_at_edge", available ? e.authority_dwt_at_edge : 0U);
  edge.add("pps_dwt_at_edge", available ? e.pps_dwt_at_edge : 0U);
  edge.add("vclock_observed_dwt_at_edge",
           available ? e.vclock_observed_dwt_at_edge : 0U);
  edge.add("vclock_predicted_dwt_at_edge",
           available ? e.vclock_predicted_dwt_at_edge : 0U);
  edge.add("pps_projected_vclock_dwt_at_edge",
           available ? e.pps_projected_vclock_dwt_at_edge : 0U);

  edge.add("authority_minus_pps_cycles",
           available ? e.authority_minus_pps_cycles : 0);
  edge.add("authority_minus_vclock_observed_cycles",
           available ? e.authority_minus_vclock_observed_cycles : 0);
  edge.add("authority_minus_prediction_cycles",
           available ? e.authority_minus_prediction_cycles : 0);
  edge.add("prediction_minus_pps_projected_cycles",
           available ? e.prediction_minus_pps_projected_cycles : 0);
  edge.add("pps_projected_minus_observed_cycles",
           available ? e.pps_projected_minus_observed_cycles : 0);
  edge.add("observed_minus_prediction_cycles",
           available ? e.observed_minus_prediction_cycles : 0);

  edge.add("counter32_at_edge", available ? e.counter32_at_edge : 0U);
  edge.add("ch3_at_edge", available ? (uint32_t)e.ch3_at_edge : 0U);
  edge.add("dwt_cycles_between_edges",
           available ? e.dwt_cycles_between_edges : 0U);
  edge.add("effective_dwt_cycles_per_second",
           available ? e.effective_dwt_cycles_per_second : 0U);

  edge.add("gnss_self_map_valid", available && e.gnss_self_map_valid);
  edge.add("gnss_self_error_ok", available && e.gnss_self_error_ok);
  edge.add("gnss_self_error_ns", available ? e.gnss_self_error_ns : 0LL);

  parent.add_object("pps_vclock_edge", edge);
}

static FLASHMEM void payload_add_slim_compare_service(
    Payload& parent,
    interrupt_subscriber_kind_t kind,
    bool valid,
    const clocks_alpha_lane_forensics_t& f) {
  Payload service;
  service.add("class", valid ? f.diag_service_class : 0U);
  service.add("offset_ticks", valid ? f.diag_service_offset_signed_ticks : 0);
  service.add("offset_abs_ticks", valid ? f.diag_service_offset_abs_ticks : 0U);
  service.add("interpreted_late_ticks",
              valid ? f.diag_interpreted_late_ticks : 0U);
  service.add("early_ticks", valid ? f.diag_early_ticks : 0U);
  service.add("target_delta_mod65536_ticks",
              valid ? f.diag_target_delta_mod65536_ticks : 0U);
  service.add("arm_remaining_ticks",
              valid ? f.diag_arm_remaining_ticks : 0U);
  service.add("arm_to_isr_ticks",
              valid ? f.diag_arm_to_isr_ticks : 0U);
  service.add("arm_to_isr_dwt_cycles",
              valid ? f.diag_arm_to_isr_dwt_cycles : 0U);
  service.add("service_correction_cycles",
              valid ? f.diag_service_correction_cycles : 0);
  service.add("service_corrected_dwt_at_event",
              valid ? f.diag_service_corrected_dwt_at_event : 0U);
  service.add("perishable_fact_sequence",
              valid ? f.diag_perishable_fact_sequence : 0U);
  service.add("fact_ring_overflow_count",
              valid ? f.diag_fact_ring_overflow_count : 0U);
  service.add("counter_delta_violation_count",
              valid ? f.diag_counter_delta_violation_count : 0U);
  service.add("last_bad_counter_delta",
              valid ? f.diag_last_bad_counter_delta : 0U);
  service.add("last_counter_delta_ticks",
              valid ? f.diag_last_counter_delta_ticks : 0U);

  const interrupt_capture_diag_t* direct_diag =
      valid ? interrupt_last_diag(kind) : nullptr;
  const bool direct_valid = direct_diag && direct_diag->enabled &&
                            direct_diag->kind == kind &&
                            direct_diag->counter32_at_event == f.last_event_counter32;

  Payload compare;
  compare.add("direct_valid", direct_valid);
  compare.add("arm_counter_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_arm_counter_low16 : 0U);
  compare.add("arm_compare_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_arm_compare_low16 : 0U);
  compare.add("arm_counter_minus_compare_ticks",
              direct_valid ? direct_diag->ocxo_arm_counter_minus_compare_ticks : 0U);
  compare.add("arm_compare_remaining_ticks",
              direct_valid ? direct_diag->ocxo_arm_compare_remaining_ticks : 0U);
  compare.add("isr_counter_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_isr_counter_low16 : 0U);
  compare.add("isr_compare_low16",
              direct_valid ? (uint32_t)direct_diag->ocxo_isr_compare_low16 : 0U);
  compare.add("isr_counter_minus_compare_ticks",
              direct_valid ? direct_diag->ocxo_isr_counter_minus_compare_ticks : 0U);
  compare.add("compare_delta_mod65536_ticks",
              direct_valid ? direct_diag->ocxo_compare_delta_mod65536_ticks : 0U);
  compare.add("compare_service_offset_signed_ticks",
              direct_valid ? direct_diag->ocxo_compare_service_offset_signed_ticks : 0);
  compare.add("compare_interpreted_late_ticks",
              direct_valid ? direct_diag->ocxo_compare_interpreted_late_ticks : 0U);
  compare.add("compare_early_ticks",
              direct_valid ? direct_diag->ocxo_compare_early_ticks : 0U);
  compare.add("compare_arm_to_isr_ticks",
              direct_valid ? direct_diag->ocxo_compare_arm_to_isr_ticks : 0U);
  service.add_object("compare", compare);

  parent.add_object("compare_service", service);
}

static FLASHMEM void payload_add_slim_vclock_forensics(Payload& parent,
                                              uint64_t public_ns,
                                              bool valid,
                                              const clocks_alpha_lane_forensics_t& f) {
  Payload lane;
  lane.add("ns", public_ns);
  lane.add("public_ns", public_ns);
  payload_add_slim_dwt_lane_forensics(lane, valid, f);
  parent.add_object("vclock", lane);
}

static FLASHMEM void payload_add_slim_ocxo_forensics(
    Payload& parent,
    const char* key,
    uint64_t public_ns,
    uint64_t public_measured_ns,
    bool forensics_valid,
    const clocks_alpha_lane_forensics_t& f,
    bool pps_projection_valid,
    const clocks_alpha_ocxo_pps_projection_snapshot_t& pps_projection,
    bool pps_residual_valid,
    uint64_t pps_gnss_interval_ns,
    uint64_t pps_clock_interval_ns,
    int64_t pps_fast_residual_ns,
    const ocxo_cycle_residual_diag_t& cycle_diag) {
  Payload lane;
  lane.add("ns", public_ns);
  lane.add("public_ns", public_ns);
  lane.add("measured_gnss_ns", public_measured_ns);
  lane.add("pps_projected_valid", pps_projection_valid);
  lane.add("pps_projected_raw_ns",
           pps_projection_valid ? pps_projection.projected_ocxo_ns_at_pps : 0ULL);
  lane.add("pps_projection_source",
           pps_projection_valid ? pps_projection.source : 0U);

  payload_add_ocxo_pps_residual_object(lane,
                                       pps_residual_valid,
                                       pps_gnss_interval_ns,
                                       pps_clock_interval_ns,
                                       pps_fast_residual_ns);

  payload_add_ocxo_cycle_residual_diag_object(lane, cycle_diag);
  payload_add_slim_dwt_lane_forensics(lane, forensics_valid, f);

  const interrupt_subscriber_kind_t service_kind =
      (key && key[4] == '1')
          ? interrupt_subscriber_kind_t::OCXO1
          : interrupt_subscriber_kind_t::OCXO2;
  payload_add_slim_compare_service(lane, service_kind, forensics_valid, f);

  parent.add_object(key, lane);
}


static FLASHMEM void payload_add_timebase_pair_identity(Payload& p,
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
  p.add("pps_vclock_gnss_ns_at_edge", public_gnss_ns);
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

static FLASHMEM void payload_add_recovery_continuity_forensics(Payload& f,
                                                              uint32_t public_count) {
  if (g_recover_continuity_align_last_public_count != public_count) {
    return;
  }

  // Recovery-continuity proof is useful, but not hot-fragment science. Keep it
  // flat in the companion row: nested add_object_json() is exactly the Payload
  // operation that overflowed when this proof lived in TIMEBASE_FRAGMENT.
  f.add("rc_schema", "RECOVERY_CONTINUITY_V1");
  f.add("rc_aligned", true);
  f.add("rc_public_count", public_count);
  f.add("rc_align_count", g_recover_continuity_align_count);
  f.add("rc_requested_public_count",
        g_recover_continuity_align_requested_public_count);
  f.add("rc_reason", g_recover_continuity_last_reason);
  f.add("rc_o1_target_ns", g_recover_continuity_ocxo1_target_ns);
  f.add("rc_o2_target_ns", g_recover_continuity_ocxo2_target_ns);
  f.add("rc_o1_before_ns", g_recover_continuity_ocxo1_before_ns);
  f.add("rc_o2_before_ns", g_recover_continuity_ocxo2_before_ns);
  f.add("rc_o1_after_ns", g_recover_continuity_ocxo1_after_ns);
  f.add("rc_o2_after_ns", g_recover_continuity_ocxo2_after_ns);
  f.add("rc_o1_correction_ns", g_recover_continuity_ocxo1_correction_ns);
  f.add("rc_o2_correction_ns", g_recover_continuity_ocxo2_correction_ns);
}

static FLASHMEM void payload_add_vclock_fragment(Payload& p,
                                          uint64_t public_gnss_ns,
                                          uint32_t public_count,
                                          bool vclock_forensics_valid,
                                          const clocks_alpha_lane_forensics_t& vclock_forensics) {
  Payload lane;
  lane.add("ns", public_gnss_ns);
  lane.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);
  payload_add_vclock_science_object(lane,
                                    public_count,
                                    public_gnss_ns,
                                    vclock_forensics_valid,
                                    vclock_forensics);
  p.add_object("vclock", lane);
}

static FLASHMEM void payload_add_vclock_forensics(Payload& p,
                                         uint64_t public_gnss_ns,
                                         bool forensics_valid,
                                         const clocks_alpha_lane_forensics_t& f) {
  Payload lane;
  lane.add("ns", public_gnss_ns);
  lane.add("counter32_at_pps_vclock", (uint32_t)g_counter32_at_pps_vclock);

  // TIMEBASE_FORENSICS is now a compact per-row courtroom audit.  Deep
  // measurement, SpinCatch/SpinIdle, and regression surfaces remain available
  // through focused reports; the paired 1 Hz stream carries only the DWT
  // authority proof needed to explain the science row.
  payload_add_lane_forensics_object(lane, forensics_valid, f);
  p.add_object("vclock", lane);
}

static FLASHMEM const char* counterledger_phase_source_name(uint32_t source_id) {
  switch (source_id) {
    case 1U:
      return "PHYSICAL_PPS_TO_OBSERVED_OCXO_EDGE";
    default:
      return "NONE";
  }
}

static FLASHMEM void payload_add_counterledger_candidate_object(
    Payload& lane,
    const clocks_alpha_ocxo_counterledger_snapshot_t& c,
    uint64_t candidate_public_ns,
    uint64_t canonical_public_ns,
    uint64_t public_gnss_ns) {
  Payload obj;
  obj.add("enabled", clocks_ocxo_counterledger_report_enabled());
  obj.add("authority", clocks_ocxo_counterledger_mode_enabled());
  obj.add("report_only", !clocks_ocxo_counterledger_mode_enabled());
  obj.add("sample_source", "EPOCH_CAPTURE_COUNTER32");
  obj.add("pps_capture_exact", false);
  obj.add("valid", c.valid);
  obj.add("initialized", c.initialized);
  obj.add("interval_valid", c.interval_valid);
  obj.add("clock_id", c.clock_id);
  obj.add("pps_sequence", c.pps_sequence);
  obj.add("sample_count", c.sample_count);
  obj.add("zero_counter32", c.zero_counter32);
  obj.add("last_counter32", c.last_counter32);
  obj.add("ticks64", c.ticks64);
  obj.add("ns", c.ns);
  obj.add("interval_ticks", c.last_delta_ticks);
  obj.add("interval_ns", c.interval_ns);
  obj.add("fast_residual_ns", c.interval_valid ? c.fast_residual_ns : 0LL);
  obj.add("refined_valid", c.refined_valid);
  obj.add("refined_ns", c.refined_valid ? c.refined_ns : 0ULL);
  obj.add("refined_interval_valid", c.refined_interval_valid);
  obj.add("refined_interval_ns",
          c.refined_interval_valid ? c.refined_interval_ns : 0ULL);
  obj.add("refined_fast_residual_ns",
          c.refined_interval_valid ? c.refined_fast_residual_ns : 0LL);

  Payload phase;
  phase.add("valid", c.phase_valid);
  phase.add("pending", c.phase_pending);
  phase.add("source_id", c.phase_source_id);
  phase.add("source", counterledger_phase_source_name(c.phase_source_id));
  phase.add("pps_sequence", c.phase_pps_sequence);
  phase.add("lag_pps", c.phase_lag_pps);
  phase.add("pps_dwt_at_edge", c.phase_pps_dwt_at_edge);
  phase.add("prev_ocxo_dwt_at_edge", c.phase_prev_ocxo_dwt_at_edge);
  phase.add("next_ocxo_dwt_at_edge", c.phase_next_ocxo_dwt_at_edge);
  phase.add("ocxo_interval_cycles", c.phase_ocxo_interval_cycles);
  phase.add("pps_delta_cycles", c.phase_pps_delta_cycles);
  phase.add("phase_after_last_00_ns", c.phase_after_last_00_ns);
  phase.add("phase_to_next_00_ns", c.phase_to_next_00_ns);
  phase.add("raw_delta_ns", c.phase_raw_delta_ns);
  phase.add("unwrapped_delta_ns", c.phase_unwrapped_delta_ns);
  phase.add("unwrapped_carry_ticks", c.phase_unwrapped_carry_ticks);
  phase.add("wrap_event", c.phase_wrap_event);
  phase.add("wrap_count", c.phase_wrap_count);
  phase.add("near_boundary", c.phase_near_boundary);
  phase.add("resolve_count", c.phase_resolve_count);
  phase.add("pending_overwrite_count", c.phase_pending_overwrite_count);
  phase.add("invalid_count", c.phase_invalid_count);
  obj.add_object("phaseledger", phase);

  obj.add("candidate_public_ns", candidate_public_ns);
  obj.add("candidate_minus_public_ns",
          (candidate_public_ns && canonical_public_ns)
              ? beta_signed_delta_u64(candidate_public_ns, canonical_public_ns)
              : 0LL);
  obj.add("candidate_minus_gnss_ns",
          (candidate_public_ns && public_gnss_ns)
              ? beta_signed_delta_u64(candidate_public_ns, public_gnss_ns)
              : 0LL);
  obj.add("capture_available", c.last_capture_available);
  obj.add("capture_valid", c.last_capture_valid);
  obj.add("capture_lane_valid", c.last_capture_lane_valid);
  obj.add("capture_all_lanes_valid", c.last_capture_all_lanes_valid);
  obj.add("capture_sequence_match", c.last_capture_sequence_match);
  obj.add("capture_sequence", c.last_capture_sequence);
  obj.add("capture_window_cycles", c.last_capture_window_cycles);
  obj.add("update_count", c.update_count);
  obj.add("capture_missing_count", c.capture_missing_count);
  obj.add("capture_invalid_count", c.capture_invalid_count);
  obj.add("lane_capture_invalid_count", c.lane_capture_invalid_count);
  obj.add("sequence_mismatch_count", c.sequence_mismatch_count);
  obj.add("all_lanes_invalid_count", c.all_lanes_invalid_count);
  obj.add("interval_gap_count", c.interval_gap_count);
  obj.add("interval_implausible_count", c.interval_implausible_count);
  obj.add("last_implausible_delta_ticks", c.last_implausible_delta_ticks);
  obj.add("recover_reprime_count", c.recover_reprime_count);
  obj.add("plausible_min_delta_ticks", c.plausible_min_delta_ticks);
  obj.add("plausible_max_delta_ticks", c.plausible_max_delta_ticks);

  obj.add("block_window_seconds", c.block_window_seconds);
  obj.add("block_valid", c.block_valid);
  obj.add("block_start_pps_sequence", c.block_start_pps_sequence);
  obj.add("block_end_pps_sequence", c.block_end_pps_sequence);
  obj.add("block_interval_count", c.block_interval_count);
  obj.add("block_ticks", c.block_ticks);
  obj.add("block_fast_residual_sum_ns", c.block_fast_residual_sum_ns);
  obj.add("block_mean_fast_residual_ns", c.block_mean_fast_residual_ns, 6);
  obj.add("block_tau", c.block_tau, 12);
  obj.add("block_ppb", c.block_ppb, 6);
  obj.add("block_phase_valid", c.block_phase_valid);
  obj.add("block_ns_with_phase", c.block_ns_with_phase);
  obj.add("block_fast_residual_sum_ns_with_phase",
          c.block_fast_residual_sum_ns_with_phase);
  obj.add("block_mean_fast_residual_ns_with_phase",
          c.block_mean_fast_residual_ns_with_phase, 6);
  obj.add("block_tau_with_phase", c.block_tau_with_phase, 12);
  obj.add("block_ppb_with_phase", c.block_ppb_with_phase, 6);
  obj.add("completed_block_count", c.completed_block_count);
  obj.add("completed_block_valid", c.completed_block_valid);
  obj.add("completed_block_start_pps_sequence",
          c.completed_block_start_pps_sequence);
  obj.add("completed_block_end_pps_sequence",
          c.completed_block_end_pps_sequence);
  obj.add("completed_block_interval_count", c.completed_block_interval_count);
  obj.add("completed_block_ticks", c.completed_block_ticks);
  obj.add("completed_block_fast_residual_sum_ns",
          c.completed_block_fast_residual_sum_ns);
  obj.add("completed_block_mean_fast_residual_ns",
          c.completed_block_mean_fast_residual_ns, 6);
  obj.add("completed_block_tau", c.completed_block_tau, 12);
  obj.add("completed_block_ppb", c.completed_block_ppb, 6);
  obj.add("completed_block_phase_valid", c.completed_block_phase_valid);
  obj.add("completed_block_ns_with_phase", c.completed_block_ns_with_phase);
  obj.add("completed_block_fast_residual_sum_ns_with_phase",
          c.completed_block_fast_residual_sum_ns_with_phase);
  obj.add("completed_block_mean_fast_residual_ns_with_phase",
          c.completed_block_mean_fast_residual_ns_with_phase, 6);
  obj.add("completed_block_tau_with_phase",
          c.completed_block_tau_with_phase, 12);
  obj.add("completed_block_ppb_with_phase",
          c.completed_block_ppb_with_phase, 6);
  obj.add("block_gap_reset_count", c.block_gap_reset_count);
  lane.add_object("counterledger", obj);
}

static FLASHMEM void payload_add_ocxo_fragment(Payload& p,
                                      const char* key,
                                      uint64_t public_ns,
                                      uint64_t public_measured_ns,
                                      bool pps_projection_valid,
                                      uint32_t pps_projection_source,
                                      bool pps_residual_valid,
                                      uint64_t pps_gnss_interval_ns,
                                      uint64_t pps_clock_interval_ns,
                                      int64_t pps_fast_residual_ns,
                                      const clocks_alpha_ocxo_counterledger_snapshot_t& counterledger,
                                      uint64_t counterledger_public_ns,
                                      uint64_t public_gnss_ns,
                                      const clock_science_row_t& science_row) {
  Payload lane;

  // Compact science surface.  The canonical OCXO value is now rendered from
  // observed-DWT Delta Cycles campaign totals.  measured_gnss_ns and PPS
  // projection flags remain as legacy/forensic side-channels for dashboards/
  // report migration.
  lane.add("ns", public_ns);
  lane.add("ns_source_id", clocks_ocxo_counterledger_mode_enabled()
      ? 4U
      : (pps_residual_valid ? 3U : 1U));
  lane.add("ns_source_name",
           clocks_ocxo_counterledger_mode_enabled()
               ? "COUNTERLEDGER_PPS_CAPTURE"
               : (pps_residual_valid ? "DELTA_CYCLES_TOTAL" : "MEASURED_GNSS_FALLBACK"));
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

  if (clocks_ocxo_counterledger_report_enabled()) {
    payload_add_counterledger_candidate_object(lane,
                                               counterledger,
                                               counterledger_public_ns,
                                               public_ns,
                                               public_gnss_ns);
  }

  payload_add_ocxo_science_object(lane,
                                  science_row,
                                  pps_residual_valid,
                                  pps_gnss_interval_ns,
                                  pps_clock_interval_ns,
                                  pps_fast_residual_ns);

  p.add_object(key, lane);
}

static FLASHMEM void payload_add_ocxo_forensics(Payload& p,
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
  // clock value, source identity, the raw PPS-projection value, the Delta
  // residual rendering, the DWT gate courtroom, and the tiny OCXO service/counter ladder
  // proof.  Retired quiet-phase sample fields, measurement/window counters,
  // SpinCatch/SpinIdle, regression, and verbose projection deltas are report-only.
  lane.add("ns", public_ns);
  lane.add("ns_source_id", clocks_ocxo_counterledger_mode_enabled()
      ? 4U
      : (pps_residual_valid ? 3U : 1U));
  lane.add("ns_source_name",
           clocks_ocxo_counterledger_mode_enabled()
               ? "COUNTERLEDGER_PPS_CAPTURE"
               : (pps_residual_valid ? "DELTA_CYCLES_TOTAL" : "MEASURED_GNSS_FALLBACK"));
  lane.add("pps_projected_valid", pps_projection_valid);
  lane.add("pps_projected_raw_ns",
           pps_projection_valid ? pps_projection.projected_ocxo_ns_at_pps : 0ULL);
  lane.add("pps_projection_source", pps_projection_valid ? pps_projection.source : 0U);
  lane.add("measured_gnss_ns", public_measured_ns);
  if (key && key[4] == '1') {
    lane.add("physical_measured_gnss_ns_at_pps_vclock",
             (uint64_t)g_ocxo1_physical_measured_gnss_ns_at_pps_vclock);
    lane.add("visible_measured_gnss_ns_at_pps_vclock",
             (uint64_t)g_ocxo1_measured_gnss_ns_at_pps_vclock);
  } else {
    lane.add("physical_measured_gnss_ns_at_pps_vclock",
             (uint64_t)g_ocxo2_physical_measured_gnss_ns_at_pps_vclock);
    lane.add("visible_measured_gnss_ns_at_pps_vclock",
             (uint64_t)g_ocxo2_measured_gnss_ns_at_pps_vclock);
  }

  payload_add_ocxo_pps_residual_object(lane,
                                       pps_residual_valid,
                                       pps_gnss_interval_ns,
                                       pps_clock_interval_ns,
                                       pps_fast_residual_ns);

  // Temporarily suppress the same-yardstick residual object in
  // TIMEBASE_FORENSICS to make room for the OCXO compare-service excursion
  // dossier below.  The scalar science residual remains in pps_residual, and
  // the static prediction/object rails still expose the underlying cycles.
  (void)cycle_diag;

  payload_add_lane_forensics_object(lane, forensics_valid, f);
  const interrupt_subscriber_kind_t service_kind =
      (key && key[4] == '1')
          ? interrupt_subscriber_kind_t::OCXO1
          : interrupt_subscriber_kind_t::OCXO2;
  payload_add_ocxo_service_object(lane, service_kind, forensics_valid, f);
  p.add_object(key, lane);
}

// Servo source doctrine. MEAN and NOW consume whichever one-second residual
// surface is canonical for this build: Delta Cycles in traditional mode,
// CounterLedger PPS-capture residuals in CounterLedger mode. TOTAL consumes the
// published campaign clockface ratio (public_ocxo_ns / public_gnss_ns) and
// shapes it through the catch-up controller.
static constexpr uint32_t SERVO_INPUT_SOURCE_NONE = 0;
static constexpr uint32_t SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD = 1;
static constexpr uint32_t SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU = 2;
static constexpr uint32_t SERVO_INPUT_SOURCE_NOW_PPS_RESIDUAL = 3;

static const char* servo_input_source_name(uint32_t source) {
  switch (source) {
    case SERVO_INPUT_SOURCE_MEAN_PPS_RESIDUAL_WELFORD:
      return "PPS_RESIDUAL_WELFORD_MEAN_PPB";
    case SERVO_INPUT_SOURCE_TOTAL_PUBLIC_TAU:
      return "PUBLIC_CLOCK_NS_OVER_GNSS_NS_TOTAL_TAU";
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
  input.add("control_doctrine", "DELTA_RESIDUAL_MEAN_NOW_PUBLIC_RATIO_TOTAL");
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

static FLASHMEM void payload_add_dac_realization_object(Payload& parent,
                                               const char* key,
                                               const ocxo_dac_state_t& s) {
  Payload r;
  r.add("mode", ocxo_dac_realization_mode_runtime());
  r.add("reference_mode", OCXO_DAC_REFERENCE_MODE);
  r.add("external_vref_used", false);
  r.add("internal_ref_voltage", OCXO_DAC_INTERNAL_REF_VOLTAGE, 9);
  r.add("output_gain", OCXO_DAC_OUTPUT_GAIN, 3);
  r.add("output_full_scale_voltage", OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE, 9);
  r.add("dac_code_scale", OCXO_DAC_CODE_SCALE, 1);
  r.add("safe_max_output_voltage", OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE, 9);
  r.add("safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);
  r.add("fractional_target", s.dac_fractional, 6);
  r.add("fractional_target_voltage",
        ocxo_dac_voltage_from_code(s.dac_fractional), 9);
  r.add("current_hw_code", (uint32_t)s.dac_hw_code);
  r.add("current_hw_voltage",
        ocxo_dac_voltage_from_code((double)s.dac_hw_code), 9);
  r.add("rounded_hw_code",
        (uint32_t)ocxo_dac_rounded_hw_code_from_value(s.dac_fractional));
  r.add("rounded_hw_voltage",
        ocxo_dac_voltage_from_code(
            (double)ocxo_dac_rounded_hw_code_from_value(s.dac_fractional)),
        9);

  r.add("safe_ceiling_active", true);
  r.add("static_rounded_only", ocxo_dac_static_rounded_only_runtime());
  r.add("fractional_stream_possible",
        ocxo_dac_fractional_stream_possible_runtime());
  r.add("recurring_timer_possible", clocks_ocxo_dac_dither_started());
  r.add("operator_enabled", clocks_ocxo_dac_dither_operator_enabled());
  r.add("started", clocks_ocxo_dac_dither_started());
  r.add("service_pending", clocks_ocxo_dac_dither_service_pending());
  r.add("write_context", clocks_ocxo_dac_dither_context());
  r.add("actuator_context", clocks_ocxo_dac_actuator_context());
  r.add("actuator_service_pending", clocks_ocxo_dac_actuator_service_pending());
  r.add("actuator_service_arm_count", clocks_ocxo_dac_actuator_service_arm_count());
  r.add("actuator_service_arm_failures", clocks_ocxo_dac_actuator_service_arm_failures());
  r.add("actuator_commit_attempt_count", clocks_ocxo_dac_actuator_commit_attempt_count());
  r.add("actuator_commit_success_count", clocks_ocxo_dac_actuator_commit_success_count());
  r.add("actuator_commit_failure_count", clocks_ocxo_dac_actuator_commit_failure_count());
  r.add("servo_hold_reason", servo_hold_reason_name(s.servo_hold_reason));
  r.add("servo_hold_reason_id", (uint32_t)s.servo_hold_reason);
  r.add("servo_hold_count", s.servo_hold_count);
  r.add("servo_quarantine_reason", servo_hold_reason_name(s.servo_quarantine_reason));
  r.add("servo_quarantine_remaining", s.servo_quarantine_remaining);
  r.add("servo_quarantine_begin_count", s.servo_quarantine_begin_count);
  r.add("servo_quarantine_consumed_count", s.servo_quarantine_consumed_count);
  r.add("servo_commit_fault_hold_count", s.servo_commit_fault_hold_count);
  r.add("servo_request_overwrite_count", s.servo_request_overwrite_count);
  r.add("servo_request_install_count", s.servo_request_install_count);
  r.add("servo_request_dither_frame_install_count",
        s.servo_request_dither_frame_install_count);
  r.add("servo_request_static_install_count", s.servo_request_static_install_count);
  r.add("servo_request_static_write_failure_count",
        s.servo_request_static_write_failure_count);

  r.add("low_code", (uint32_t)s.dither_low_code);
  r.add("high_code", (uint32_t)s.dither_high_code);
  r.add("high_ms", (uint32_t)s.dither_high_ms);
  r.add("low_ms", (uint32_t)(1000U - s.dither_high_ms));
  r.add("active_this_frame", s.dither_active_this_frame);
  r.add("current_phase_high", s.dither_current_phase_high);
  r.add("program_dirty", s.dither_program_dirty);

  r.add("pending_hw_write", s.dither_pending_hw_write);
  r.add("pending_hw_code", (uint32_t)s.dither_pending_hw_code);
  r.add("pending_request_count", s.dither_pending_request_count);
  r.add("pending_overwrite_count", s.dither_pending_overwrite_count);

  r.add("frame_count", s.dither_frame_count);
  r.add("transition_count", s.dither_transition_count);
  r.add("write_count", s.dither_write_count);
  r.add("write_failure_count", s.dither_write_failure_count);
  r.add("skip_same_code_count", s.dither_skip_same_code_count);
  r.add("schedule_failure_count", s.dither_schedule_failure_count);

  r.add("service_count", s.dither_service_count);
  r.add("service_write_count", s.dither_service_write_count);
  r.add("service_skip_same_count", s.dither_service_skip_same_count);
  r.add("service_defer_count", s.dither_service_defer_count);

  parent.add_object(key, r);
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
                                    double pps_fast_residual_ppb,
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
  // NOW consumes the exact FloorLine value directly rather than averaging it
  // into MEAN or folding it into campaign-total tau.
  d.now_ppb = pps_residual_valid ? pps_fast_residual_ppb : 0.0;
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
static uint32_t        g_ocxo_dac_no_candidate_passes = 0;
static uint32_t        g_ocxo_dac_deferred_candidates = 0;
static uint32_t        g_ocxo_dac_schedule_failures = 0;

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
  g_ocxo_dac_last_schedule_second = campaign_seconds;
  g_ocxo_dac_last_winner = ocxo_dac_lane_id(dac);

  ocxo_dac_request_servo_target(dac, target, planned_step, campaign_seconds);
}

// ============================================================================
// Zeroing
// ============================================================================

static void campaign_accounting_reset_common(bool reset_servo_runtime) {
  clocks_watchdog_disarm_campaign_publication();

  // Beta-local accounting reset only.  Alpha owns the active time/epoch
  // projection. Do not invalidate it here: after SmartZero install the new
  // epoch has just been authored; during acquisition the old service epoch
  // must remain alive.  FLASH_CUT also uses this path, but with servo runtime
  // intentionally preserved so the hot control state survives the boundary.
  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_measured_gnss_ticks_64        = 0;
  ocxo2_measured_gnss_ticks_64        = 0;

  campaign_public_offsets_reset_to_current();

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
  welford_reset(welford_pps_witness);
  welford_reset(welford_ocxo1_dac);
  welford_reset(welford_ocxo2_dac);

  pps_interval_residuals_reset();
  clocks_beta_feature_set_cached("TIMEBASE_PUBLICATION",
                                 g_clocks_feature_timebase_publication,
                                 system_feature_status_t::INITIALIZING,
                                 true);

  if (reset_servo_runtime) {
    ocxo_dac_predictor_reset(ocxo1_dac);
    ocxo_dac_predictor_reset(ocxo2_dac);
    ocxo_dac_pacing_reset();
    ocxo1_dac.servo_hold_count = 0;
    ocxo2_dac.servo_hold_count = 0;
    ocxo1_dac.servo_hold_reason = SERVO_HOLD_NONE;
    ocxo2_dac.servo_hold_reason = SERVO_HOLD_NONE;
    ocxo1_dac.servo_quarantine_reason = SERVO_HOLD_NONE;
    ocxo2_dac.servo_quarantine_reason = SERVO_HOLD_NONE;
    ocxo1_dac.servo_quarantine_remaining = 0;
    ocxo2_dac.servo_quarantine_remaining = 0;
    ocxo1_dac.servo_quarantine_begin_count = 0;
    ocxo2_dac.servo_quarantine_begin_count = 0;
    ocxo1_dac.servo_quarantine_consumed_count = 0;
    ocxo2_dac.servo_quarantine_consumed_count = 0;
    ocxo1_dac.servo_commit_fault_hold_count = 0;
    ocxo2_dac.servo_commit_fault_hold_count = 0;
    ocxo1_dac.servo_request_overwrite_count = 0;
    ocxo2_dac.servo_request_overwrite_count = 0;
    ocxo1_dac.servo_request_install_count = 0;
    ocxo2_dac.servo_request_install_count = 0;
    ocxo1_dac.servo_request_dither_frame_install_count = 0;
    ocxo2_dac.servo_request_dither_frame_install_count = 0;
    ocxo1_dac.servo_request_static_install_count = 0;
    ocxo2_dac.servo_request_static_install_count = 0;
    ocxo1_dac.servo_request_static_write_failure_count = 0;
    ocxo2_dac.servo_request_static_write_failure_count = 0;
  }

  // Input diagnostics are campaign-row products, not plant state.  Clear them
  // even for FLASH_CUT so TIMEBASE/REPORT_DAC cannot surface stale residuals
  // across the campaign name boundary.
  servo_input_diag_reset(g_servo_input_ocxo1);
  servo_input_diag_reset(g_servo_input_ocxo2);
}

void clocks_zero_all(void) {
  campaign_accounting_reset_common(true);
}

static void campaign_flash_cut_accounting_reset(void) {
  campaign_accounting_reset_common(false);
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
// Flash Cut
// ============================================================================

static void flash_cut_clear_pending(void) {
  request_flash_cut = false;
  flash_cut_campaign_name[0] = '\0';
}

static bool flash_cut_busy(void) {
  return request_flash_cut || request_start || request_stop ||
         request_recover || request_zero || interrupt_smartzero_running() ||
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

  // This is the cut itself: Beta rebases campaign-public clocks/statistics at
  // the already-authored PPS/VCLOCK edge, then returns without emitting a
  // TIMEBASE pair for the boundary row.  The next PPS publishes count=1 and
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
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  flash_cut_clear_pending();
  request_servo_mode_change = false;
  requested_servo_mode = servo_mode_t::OFF;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  ocxo_dac_pacing_abort_all();
  campaign_warmup_reset();
  timebase_invalidate();
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
  ocxo_dac_pacing_abort_all();

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
  request_zero = false;
}

static void clocks_finish_start_accounting(void) {
  clocks_zero_all();
  request_zero = false;
  request_start = false;
  clocks_watchdog_clear_surrender_for_new_lifecycle();
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


static FLASHMEM void payload_add_servo_dac_values(Payload& parent) {
  // Ultra-compact durable DAC persistence.  TIMEBASE_FRAGMENT is the 1 Hz
  // science spine, so it must not carry DAC/dither courtroom detail.  Persist
  // only the commanded fractional DAC intent, the realized hardware code, and
  // the explicit servo/dither identity needed by the Pi config mirror.
  Payload dac;
  dac.add("schema", "TIMEBASE_DAC_PERSISTENCE_V2");
  dac.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  dac.add("servo_mode", servo_mode_str(calibrate_ocxo_mode));
  dac.add("servo_active", clocks_servo_active());
  dac.add("realization_mode", ocxo_dac_realization_mode_runtime());
  dac.add("dither_operator_enabled", clocks_ocxo_dac_dither_operator_enabled());

  dac.add("ocxo1_dac", ocxo1_dac.dac_fractional, 6);
  dac.add("ocxo2_dac", ocxo2_dac.dac_fractional, 6);
  dac.add("ocxo1_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  dac.add("ocxo2_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);

  parent.add_object("dac", dac);
}

// ----------------------------------------------------------------------------
// CLOCKS_DAC_TICK -- retired automatic DAC/servo telemetry
// ----------------------------------------------------------------------------
//
// Servo/DAC state is now piggybacked on TIMEBASE while servos are active
// through payload_add_servo_dac_values().  Focused command reports
// (REPORT_DAC / DITHER_STATUS) remain available on demand, but CLOCKS no
// longer emits an independent 1 Hz DAC_TICK publication from the PPS hot path.
//
// Keep this no-op helper so existing gate sites remain visually explicit:
// those sites are acknowledging DAC/servo transition points, not publishing
// a separate diagnostic stream.

static FLASHMEM void publish_dac_tick(const char*) {
  // Intentionally silent.
}

// ============================================================================
// clocks_beta_pps — invoked from alpha's pps_selector_callback
// ============================================================================

void clocks_beta_pps(void) {
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_ENTRY);
  g_timebase_pps_entry_count++;
  g_timebase_last_entry_campaign_seconds = campaign_seconds;
  timebase_build_stage(TIMEBASE_BUILD_STAGE_ENTRY);

  if (request_stop) {
    g_timebase_stop_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_STOP_GATE);
    const bool was_started = (campaign_state == clocks_campaign_state_t::STARTED);
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    flash_cut_clear_pending();
    calibrate_ocxo_mode = servo_mode_t::OFF;
    ocxo_dac_pacing_abort_all();
    campaign_warmup_reset();
    if (was_started) {
      timebase_invalidate();
    }
    publish_dac_tick("STOP_GATE");
    return;
  }

  if (request_start || request_zero) {
    g_timebase_start_zero_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_START_ZERO_GATE);
    (void)clocks_try_finish_pending_smartzero();
    publish_dac_tick("START_ZERO_GATE");
    return;
  }

  if (request_flash_cut) {
    g_timebase_flash_cut_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FLASH_CUT_GATE);
    campaign_flash_cut_commit_at_pps();
    publish_dac_tick("FLASH_CUT_GATE");
    return;
  }

  if (request_recover) {
    clocks_watchdog_disarm_campaign_publication();
    g_timebase_recover_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_RECOVER_GATE);
    clocks_watchdog_clear_surrender_for_new_lifecycle();
    request_zero = false;
    ocxo_dac_pacing_abort_all();
    ocxo_dac_predictor_reset(ocxo1_dac);
    ocxo_dac_predictor_reset(ocxo2_dac);

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

    // Alpha owns OCXO previous/pending edge machinery.  RECOVER must not allow
    // those surfaces to bridge the outage.  Beta then restores the campaign
    // statistics and quarantines the first public residual rows so Welford
    // receives only post-recovery, post-warmup intervals.
    clocks_alpha_recover_reprime_ocxo_state();
    interrupt_recover_reset_publication_custody();
    pps_interval_residuals_begin_recover_quarantine(
        CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
    recover_welfords_apply_pending();

    request_recover = false;
    flash_cut_clear_pending();
    campaign_state = clocks_campaign_state_t::STARTED;
    campaign_warmup_begin(campaign_warmup_mode_t::RECOVER);
    publish_dac_tick("RECOVER_GATE");
    return;
  }

  if (clocks_watchdog_publication_blocked()) {
    g_timebase_watchdog_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_WATCHDOG_GATE);
    publish_dac_tick("WATCHDOG_GATE");
    return;
  }
  if (campaign_state != clocks_campaign_state_t::STARTED) {
    g_timebase_not_started_gate_count++;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_NOT_STARTED_GATE);
    publish_dac_tick("NOT_STARTED_GATE");
    return;
  }

  // ── Publication handoff wait ──
  //
  // This is not row burial.  START may hold here while it builds a private
  // PPS0 and proves that the next candidate can become a science-complete
  // public PPS1.  Campaign time has not advanced, Welfords are still empty,
  // and no public TIMEBASE identity is skipped.  RECOVER publishes immediately
  // on the first PPS after the recovered base.
  if (campaign_warmup_consume_one_candidate_record()) {
    timebase_build_stage(TIMEBASE_BUILD_STAGE_WARMUP_GATE);
    publish_dac_tick("HANDOFF_WAIT_GATE");
    return;
  }

  if (recover_reattach_should_hold()) {
    timebase_build_stage(TIMEBASE_BUILD_STAGE_RECOVER_REATTACH_GATE);
    publish_dac_tick("RECOVER_REATTACH_GATE");
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

  clocks_alpha_lane_forensics_t& vclock_forensics = g_beta_pps_vclock_forensics;
  clocks_alpha_lane_forensics_t& ocxo1_forensics = g_beta_pps_ocxo1_forensics;
  clocks_alpha_lane_forensics_t& ocxo2_forensics = g_beta_pps_ocxo2_forensics;
  vclock_forensics = clocks_alpha_lane_forensics_t{};
  ocxo1_forensics = clocks_alpha_lane_forensics_t{};
  ocxo2_forensics = clocks_alpha_lane_forensics_t{};

  const bool vclock_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::VCLOCK, &vclock_forensics);
  const bool ocxo1_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO1, &ocxo1_forensics);
  const bool ocxo2_forensics_valid =
      clocks_alpha_lane_forensics(time_clock_id_t::OCXO2, &ocxo2_forensics);

  clocks_pps_vclock_edge_forensics_t& pps_vclock_edge_forensics =
      g_beta_pps_vclock_edge_forensics;
  pps_vclock_edge_forensics = clocks_pps_vclock_edge_forensics_t{};
  const bool pps_vclock_edge_forensics_valid =
      clocks_alpha_pps_vclock_edge_forensics(&pps_vclock_edge_forensics);

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
  const uint64_t public_gnss_ns   = campaign_public_gnss_ns();
  const uint64_t public_dwt_total = campaign_public_dwt_total();
  const uint32_t public_count = (uint32_t)campaign_seconds;

  recover_continuity_align_if_pending(public_count, public_gnss_ns);

  const uint64_t public_ocxo1_measured_ns = campaign_public_ocxo1_measured_ns();
  const uint64_t public_ocxo2_measured_ns = campaign_public_ocxo2_measured_ns();

  const bool ocxo1_pps_projected_valid =
      ocxo1_pps_projection_ok && ocxo1_pps_projection.valid &&
      ocxo1_pps_projection.projected_ocxo_ns_at_pps != 0;
  const bool ocxo2_pps_projected_valid =
      ocxo2_pps_projection_ok && ocxo2_pps_projection.valid &&
      ocxo2_pps_projection.projected_ocxo_ns_at_pps != 0;

  (void)welfords_advance_non_ocxo_gap_before_row(public_count);

  // Delta residuals are now same-row aligned. PhaseLedger may have its own
  // lawful suffix lag internally, but Beta must not compare the previous
  // PPS/VCLOCK interval against the current OCXO interval. Capture the
  // reference for this public row and use that same identity for OCXO science.
  const delta_residual_reference_t delta_reference_this_row =
      delta_residual_capture_vclock_reference(public_count,
                                              vclock_forensics_valid,
                                              vclock_forensics);
  const delta_residual_reference_t delta_reference_for_ocxo =
      delta_reference_this_row;

  // ── Delta Cycles canonical science ──
  //
  // Observed DWT-at-edge owns the canonical residual: each OCXO observed
  // interval is subtracted from the delayed observed PPS/VCLOCK DWT-edge
  // interval. FloorLine and projected-GNSS surfaces are preserved under
  // science.delta_floorline_* and science.traditional_* only; neither feeds
  // Welford.  Published OCXO TAU/PPB are re-authored below as simple
  // campaign public-ledger ratios.
  clock_science_row_t& ocxo1_floorline_science = g_beta_ocxo1_science_row;
  clock_science_row_t& ocxo2_floorline_science = g_beta_ocxo2_science_row;
  floorline_science_build_ocxo(ocxo1_floorline_science,
                               time_clock_id_t::OCXO1,
                               public_count,
                               public_gnss_ns,
                               delta_reference_for_ocxo,
                               vclock_forensics_valid,
                               vclock_forensics,
                               ocxo1_forensics_valid,
                               ocxo1_forensics,
                               g_floorline_science_ocxo1);
  floorline_science_build_ocxo(ocxo2_floorline_science,
                               time_clock_id_t::OCXO2,
                               public_count,
                               public_gnss_ns,
                               delta_reference_for_ocxo,
                               vclock_forensics_valid,
                               vclock_forensics,
                               ocxo2_forensics_valid,
                               ocxo2_forensics,
                               g_floorline_science_ocxo2);

  g_beta_ocxo1_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  g_beta_ocxo2_counterledger_row = clocks_alpha_ocxo_counterledger_snapshot_t{};
  (void)clocks_alpha_ocxo_counterledger_snapshot(time_clock_id_t::OCXO1,
                                                 &g_beta_ocxo1_counterledger_row);
  (void)clocks_alpha_ocxo_counterledger_snapshot(time_clock_id_t::OCXO2,
                                                 &g_beta_ocxo2_counterledger_row);
  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo1_counterledger =
      g_beta_ocxo1_counterledger_row;
  const clocks_alpha_ocxo_counterledger_snapshot_t& ocxo2_counterledger =
      g_beta_ocxo2_counterledger_row;

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
      : floorline_render_public_clock_ns(
            public_gnss_ns, ocxo1_floorline_science, public_ocxo1_measured_ns);
  const uint64_t public_ocxo2_ns = clocks_ocxo_counterledger_mode_enabled()
      ? public_ocxo2_counterledger_ns
      : floorline_render_public_clock_ns(
            public_gnss_ns, ocxo2_floorline_science, public_ocxo2_measured_ns);

  clock_science_apply_counterledger_row(ocxo1_floorline_science,
                                        ocxo1_counterledger,
                                        public_gnss_ns);
  clock_science_apply_counterledger_row(ocxo2_floorline_science,
                                        ocxo2_counterledger,
                                        public_gnss_ns);

  // Published OCXO TAU/PPB are campaign clockface ratios, not physical-period
  // Delta ratios: public_ocxo_ns / public_gnss_ns.  RECOVER may have just
  // applied a one-time presentation transform so this ratio continues from the
  // Pi-projected campaign ledger instead of a fresh reattachment intercept.
  // The one-second residuals and Welfords above remain Delta/PhaseLedger
  // evidence surfaces.
  clock_science_apply_campaign_public_ratio(ocxo1_floorline_science,
                                            public_gnss_ns,
                                            public_ocxo1_ns,
                                            public_count);
  clock_science_apply_campaign_public_ratio(ocxo2_floorline_science,
                                            public_gnss_ns,
                                            public_ocxo2_ns,
                                            public_count);

  clock_science_apply_alpha_tau(ocxo1_floorline_science,
                                ocxo1_alpha_tau_ok,
                                ocxo1_alpha_tau);
  clock_science_apply_alpha_tau(ocxo2_floorline_science,
                                ocxo2_alpha_tau_ok,
                                ocxo2_alpha_tau);

  if (recover_degraded_science_hold) {
    recover_reattach_apply_degraded_science_hold(ocxo1_floorline_science);
    recover_reattach_apply_degraded_science_hold(ocxo2_floorline_science);
  }

  (void)science_residual_quarantine_apply(public_count,
                                          ocxo1_floorline_science,
                                          ocxo2_floorline_science);

  ocxo1_measured_gnss_ticks_64 = public_ocxo1_ns / 100ULL;
  ocxo2_measured_gnss_ticks_64 = public_ocxo2_ns / 100ULL;

  g_delta_previous_vclock_reference = delta_reference_this_row;

  const pps_interval_residuals_t pps_residuals =
      floorline_interval_residuals_update(public_count,
                                          ocxo1_floorline_science,
                                          ocxo2_floorline_science);

  // ── Legacy cycle-domain residual diagnostics ──
  //
  // This older static-prediction diagnostic remains forensic-only.  Canonical
  // Delta residuals are carried in ocxoN.science.delta_raw_* and feed
  // Welford, tau, public pps_residual, and servo below; FloorLine remains
  // a comparison side rail in ocxoN.science.delta_floorline_*.
  g_beta_pps_cycle_prediction = prediction_snapshot_for_pps();
  g_beta_ocxo1_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO1);
  g_beta_ocxo2_cycle_prediction =
      prediction_snapshot_for_clock(time_clock_id_t::OCXO2);
  const clocks_static_prediction_snapshot_t& pps_cycle_prediction =
      g_beta_pps_cycle_prediction;
  const clocks_static_prediction_snapshot_t& ocxo1_cycle_prediction =
      g_beta_ocxo1_cycle_prediction;
  const clocks_static_prediction_snapshot_t& ocxo2_cycle_prediction =
      g_beta_ocxo2_cycle_prediction;
  ocxo_cycle_residual_diag_build(g_beta_ocxo1_cycle_residual_diag,
                                 pps_cycle_prediction, ocxo1_cycle_prediction,
                                 ocxo1_floorline_science.traditional_valid,
                                 ocxo1_floorline_science.traditional_fast_residual_ns);
  ocxo_cycle_residual_diag_build(g_beta_ocxo2_cycle_residual_diag,
                                 pps_cycle_prediction, ocxo2_cycle_prediction,
                                 ocxo2_floorline_science.traditional_valid,
                                 ocxo2_floorline_science.traditional_fast_residual_ns);
  const ocxo_cycle_residual_diag_t& ocxo1_cycle_residual_diag =
      g_beta_ocxo1_cycle_residual_diag;
  const ocxo_cycle_residual_diag_t& ocxo2_cycle_residual_diag =
      g_beta_ocxo2_cycle_residual_diag;

  // VCLOCK measurement Welford — bridge-interpolation self-test.
  // Sample in ns; mean == ppb under the "ns/sec == ppb" identity.
  if (g_vclock_measurement.gnss_ns_between_edges != 0) {
    welford_update(welford_vclock, (double)g_vclock_measurement.second_residual_ns);
  }

  (void)welfords_advance_ocxo_gap_before_valid_row(
      public_count,
      ocxo1_floorline_science.valid,
      ocxo2_floorline_science.valid);

  // OCXO Welfords consume the exact canonical Delta residual.  The integer
  // pps_residual.fast_residual_ns remains the public compatibility rendering;
  // Welford keeps the sub-ns Delta value so mean/stddev do not throw away
  // the same-yardstick cycle-domain precision.
  if (ocxo1_floorline_science.valid) {
    welford_update(welford_ocxo1,
                   ocxo1_floorline_science.fast_residual_ns_exact);
  }
  if (ocxo2_floorline_science.valid) {
    welford_update(welford_ocxo2,
                   ocxo2_floorline_science.fast_residual_ns_exact);
  }

  // PPS witness statistics are owned by the witness/PPS_PHASE path.
  // Beta publishes the accumulator but does not update it here.

  const bool ocxo1_total_slope_valid =
      pps_residuals.ocxo1_valid && ocxo1_floorline_science.total_valid;
  const bool ocxo2_total_slope_valid =
      pps_residuals.ocxo2_valid && ocxo2_floorline_science.total_valid;
  const double ocxo1_total_tau = ocxo1_floorline_science.total_valid
      ? ocxo1_floorline_science.total_tau
      : 1.0;
  const double ocxo2_total_tau = ocxo2_floorline_science.total_valid
      ? ocxo2_floorline_science.total_tau
      : 1.0;

  // SERVOS live command handoff.  Apply here so this same campaign row's
  // servo_input.selected_source and subsequent ocxo_calibration_servo() agree.
  clocks_commit_pending_servo_mode_change();

  servo_input_diag_update(g_servo_input_ocxo1,
                          pps_residuals.ocxo1_valid,
                          pps_residuals.gnss_interval_ns,
                          pps_residuals.ocxo1_interval_ns,
                          pps_residuals.ocxo1_fast_residual_ns,
                          ocxo1_floorline_science.fast_residual_ns_exact,
                          welford_ocxo1,
                          ocxo1_total_slope_valid,
                          ocxo1_total_tau);
  servo_input_diag_update(g_servo_input_ocxo2,
                          pps_residuals.ocxo2_valid,
                          pps_residuals.gnss_interval_ns,
                          pps_residuals.ocxo2_interval_ns,
                          pps_residuals.ocxo2_fast_residual_ns,
                          ocxo2_floorline_science.fast_residual_ns_exact,
                          welford_ocxo2,
                          ocxo2_total_slope_valid,
                          ocxo2_total_tau);

  timebase_build_stage(TIMEBASE_BUILD_STAGE_WELFORD);

  // Servo runs AFTER Delta residual/Welford updates so it sees this
  // second's canonical science residual sample.
  ocxo_calibration_servo();
  timebase_build_stage(TIMEBASE_BUILD_STAGE_SERVO);

  // DAC Welfords — track servo effort (the TEMPEST signal).
  welford_update(welford_ocxo1_dac, ocxo_dac_fractional_snapshot(ocxo1_dac));
  welford_update(welford_ocxo2_dac, ocxo_dac_fractional_snapshot(ocxo2_dac));
  timebase_build_stage(TIMEBASE_BUILD_STAGE_DAC_WELFORD);

  publish_dac_tick("STARTED");

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
  clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_BUILD);

  {
    Payload p;
    payload_add_timebase_pair_identity(p,
                                       "TIMEBASE_FRAGMENT_V4",
                                       "fragment_version",
                                       4U,
                                       "fragment",
                                       public_count,
                                       public_gnss_ns);
    p.add("paired_forensics_topic", "TIMEBASE_FORENSICS");
    p.add("paired_forensics_schema", "TIMEBASE_FORENSICS_V1");
    p.add("recover_degraded_active", (bool)g_recover_reattach_degraded_active);
    p.add("recover_degraded_science_hold", recover_degraded_science_hold);
    p.add("recover_reattach_reason", g_recover_reattach_last_reason);

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
      if (!TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED) {
        Payload science;
        const double dwt_ppb_1s =
            ((double)g_dwt_cycles_between_pps_vclock -
             (double)DWT_EXPECTED_PER_PPS) /
            (double)DWT_EXPECTED_PER_PPS * 1.0e9;
        const uint64_t expected_total_cycles = dwt_ns_to_cycles(public_gnss_ns);
        const double dwt_total_ppb = campaign_total_dwt_ppb(public_gnss_ns,
                                                            public_dwt_total);
        science.add("schema", "TIMEBASE_CLOCK_SCIENCE_V2");
        science.add("valid", g_dwt_cycles_between_pps_vclock != 0U);
        science.add("clock_id", 0U);
        science.add("edge_species", "PPS_GNSS_DWT_RULER");
        science.add("frequency_source", "DWT_CYCLES_OVER_IDEAL_CPU_CYCLES");
        science.add("cycles_between_edges", (uint32_t)g_dwt_cycles_between_pps_vclock);
        science.add("expected_cycles_between_edges", (uint32_t)DWT_EXPECTED_PER_PPS);
        science.add("second_residual_cycles",
                    (int32_t)((int64_t)g_dwt_cycles_between_pps_vclock -
                              (int64_t)DWT_EXPECTED_PER_PPS));
        science.add("tau_1s", 1.0 + dwt_ppb_1s / 1.0e9, 12);
        science.add("ppb_1s", dwt_ppb_1s, 6);
        science.add("total_cycles", public_dwt_total);
        science.add("total_expected_cycles", expected_total_cycles);
        science.add("total_tau", 1.0 + dwt_total_ppb / 1.0e9, 12);
        science.add("total_ppb", dwt_total_ppb, 6);
        dwt.add_object("science", science);
      }
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
      if (!TIMEBASE_FRAGMENT_COMPACT_SCIENCE_ENABLED) {
        Payload science;
        science.add("schema", "TIMEBASE_CLOCK_SCIENCE_V2");
        science.add("valid", (bool)g_pps_dwt_cycles_between_edges_valid);
        science.add("clock_id", 0U);
        science.add("edge_species", "PHYSICAL_PPS_WITNESS");
        science.add("frequency_source", "GNSS_IDENTITY_WITNESS");
        science.add("positive_means", "clock_fast");
        science.add("pps_dwt_at_edge", (uint32_t)g_pps_dwt_at_edge);
        science.add("pps_vclock_dwt_at_edge", (uint32_t)g_dwt_at_pps_vclock);
        science.add("pps_vclock_gnss_ns_at_edge", public_gnss_ns);
        science.add("pps_vclock_phase_cycles", (int32_t)g_pps_vclock_phase_cycles);
        science.add("dwt_cycles_between_edges",
                    g_pps_dwt_cycles_between_edges_valid
                        ? (uint32_t)g_pps_dwt_cycles_between_edges
                        : 0U);
        science.add("gnss_interval_ns", CLOCKS_BETA_NS_PER_SECOND);
        science.add("clock_interval_ns", CLOCKS_BETA_NS_PER_SECOND);
        science.add("fast_residual_ns", 0LL);
        science.add("gnss_interval_ns_exact", (double)CLOCKS_BETA_NS_PER_SECOND, 6);
        science.add("clock_interval_ns_exact", (double)CLOCKS_BETA_NS_PER_SECOND, 6);
        science.add("fast_residual_ns_exact", 0.0, 6);
        science.add("tau_1s", 1.0, 12);
        science.add("ppb_1s", 0.0, 6);
        pps.add_object("science", science);
      }
      p.add_object("pps", pps);
    }

    // Prediction remains available through focused reports and forensics.
    // Omit it from the 1 Hz fragment when compact mode is active; raw_cycles
    // can still recover intervals from PPS endpoints and micro raw-cycle fields.
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PREDICTION);
    if (TIMEBASE_FRAGMENT_PUBLISH_PREDICTION_ENABLED) {
      payload_add_prediction_summary_hierarchical(p);
    }

    timebase_build_stage(TIMEBASE_BUILD_STAGE_VCLOCK);
    payload_add_vclock_fragment(p,
                                public_gnss_ns,
                                public_count,
                                vclock_forensics_valid,
                                vclock_forensics);

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
                              pps_residuals.ocxo1_fast_residual_ns,
                              ocxo1_counterledger,
                              public_ocxo1_counterledger_ns,
                              public_gnss_ns,
                              ocxo1_floorline_science);

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
                              pps_residuals.ocxo2_fast_residual_ns,
                              ocxo2_counterledger,
                              public_ocxo2_counterledger_ns,
                              public_gnss_ns,
                              ocxo2_floorline_science);

    timebase_build_stage(TIMEBASE_BUILD_STAGE_STATS);
    payload_add_stats_summary_hierarchical(p,
                                           public_gnss_ns,
                                           public_dwt_total,
                                           ocxo1_floorline_science,
                                           ocxo2_floorline_science);

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
    // Heap discipline: do not retain/copy the just-built fragment on Teensy.
    g_timebase_assign_last_fragment_count++;
    g_timebase_last_assign_campaign_seconds = campaign_seconds;

    if (clocks_watchdog_publication_blocked()) {
      g_timebase_watchdog_gate_count++;
      timebase_build_stage(TIMEBASE_BUILD_STAGE_WATCHDOG_GATE);
      publish_dac_tick("WATCHDOG_GATE_PRE_FRAGMENT_PUBLISH");
      return;
    }

    g_timebase_publish_attempt_count++;
    g_timebase_last_publish_attempt_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PUBLISH_ATTEMPT);
    clocks_stack_witness_note_hot(CLOCKS_STACK_CONTEXT_BETA_PPS_PUBLISH);
    publish("TIMEBASE_FRAGMENT", p);

    g_timebase_publish_return_count++;
    g_timebase_last_publish_return_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_PUBLISH_RETURN);
  }

  if (!TIMEBASE_FORENSICS_PUBLISH_ENABLED) {
    g_timebase_forensics_disabled_count++;
    clocks_watchdog_arm_campaign_publication();
    return;
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
    f.add("paired_fragment_schema", "TIMEBASE_FRAGMENT_V4");
    f.add("paired_fragment_version", 4U);
    payload_add_recovery_continuity_forensics(f, public_count);

    if (TIMEBASE_FORENSICS_MINIMAL_PAYLOAD_ENABLED) {
      // Minimal companion mode: publish just enough forensic identity and
      // provenance to satisfy Pi-side pair assembly while the larger
      // TIMEBASE_FORENSICS body is temporarily out of the blast radius.
      //
      // This is intentionally not a clock-authority surface.  The full deep
      // forensics remain available through focused reports; this row exists so
      // TIMEBASE_FRAGMENT identity N always has a matching forensics identity N.
      g_timebase_forensics_minimal_count++;
      f.add("minimal", true);
      f.add("temporary_safety_mode", "MINIMAL_PAIR_ONLY");

      if (TIMEBASE_FORENSICS_MINIMAL_HEALTH_FIELDS_ENABLED) {
        f.add("pps_vclock_edge_available", pps_vclock_edge_forensics_valid);
        f.add("vclock_forensics_available", vclock_forensics_valid);
        f.add("ocxo1_forensics_available", ocxo1_forensics_valid);
        f.add("ocxo2_forensics_available", ocxo2_forensics_valid);
        f.add("ocxo1_pps_projected", ocxo1_pps_projected_valid);
        f.add("ocxo2_pps_projected", ocxo2_pps_projected_valid);
        f.add("ocxo1_pps_residual_valid", pps_residuals.ocxo1_valid);
        f.add("ocxo2_pps_residual_valid", pps_residuals.ocxo2_valid);
      }

      if (TIMEBASE_FORENSICS_MICRO_RAW_CYCLES_ENABLED) {
        // MICRO_RAW_CYCLES_V1
        //
        // Keep this branch brutally flat and bounded.  The previous
        // slim-forensics attempt still used nested Payload objects and enough
        // construction work to trip the Teensy's campaign-start memory edge.
        // These fields are selected specifically for raw_cycles /
        // raw_cycles_excursions:
        //
        //   *_obs     observed raw one-second DWT interval
        //   *_eff     effective/published interval after EMA/Yardstick gate
        //   *_res     observed-minus-prediction residual from the interval gate
        //   *_raw     original observed DWT edge candidate
        //   *_ema     EMA DWT edge candidate
        //   *_fl      FloorLine DWT edge candidate
        //   *_pub     published/canonical DWT edge
        //   *_orig    legacy alias for *_raw
        //   *_used    legacy alias for *_pub
        //   *_fl_cyc  FloorLine inferred one-second interval
        //   *_fl_err  FloorLine inferred edge minus observed edge
        //
        // No authority is changed here.  This is a compact telemetry tap only.
        g_timebase_forensics_micro_raw_count++;
        f.add("micro_raw_cycles", true);
        f.add("micro_schema", "MICRO_RAW_CYCLES_V1");
        f.add("court_schema", "DWT_PUBLICATION_COURT_V1");

        f.add("pps_obs", g_pps_dwt_cycles_between_edges_valid
                         ? g_pps_dwt_cycles_between_edges
                         : 0U);

        if (g_start_prologue_last_release_public_count == public_count &&
            g_start_prologue_pps0_interval_valid) {
          f.add("start_pps0_valid", true);
          f.add("pps_prev_obs", (uint32_t)g_start_prologue_pps0_pps_obs);
          f.add("v_prev_obs", (uint32_t)g_start_prologue_pps0_v_obs);
          f.add("v_prev_fl", (uint32_t)g_start_prologue_pps0_v_fl);
          f.add("o1_prev_obs", (uint32_t)g_start_prologue_pps0_o1_obs);
          f.add("o1_prev_fl", (uint32_t)g_start_prologue_pps0_o1_fl);
          f.add("o2_prev_obs", (uint32_t)g_start_prologue_pps0_o2_obs);
          f.add("o2_prev_fl", (uint32_t)g_start_prologue_pps0_o2_fl);
        }

        const bool v_ok = vclock_forensics_valid;
        const bool v_fl = floorline_candidate_present(v_ok, vclock_forensics);
        const uint32_t v_fl_cyc = v_fl
            ? floorline_inferred_interval_cycles(v_fl, vclock_forensics)
            : 0U;
        const uint32_t v_raw_dwt =
            v_ok ? vclock_forensics.dwt_original_at_event : 0U;
        const uint32_t v_ema_dwt =
            v_ok ? vclock_forensics.dwt_ema_dwt_at_event : 0U;
        const uint32_t v_fl_dwt =
            v_fl ? vclock_forensics.regression_inferred_dwt_at_event : 0U;
        const uint32_t v_pub_dwt =
            v_ok ? vclock_forensics.dwt_used_at_event : 0U;
        f.add("v_obs", v_ok ? vclock_forensics.dwt_interval_observed_cycles : 0U);
        f.add("v_eff", v_ok ? vclock_forensics.dwt_interval_effective_cycles : 0U);
        f.add("v_res", v_ok ? vclock_forensics.dwt_interval_residual_cycles : 0);
        f.add("v_raw", v_raw_dwt);
        f.add("v_ema", v_ema_dwt);
        f.add("v_fl", v_fl_dwt);
        f.add("v_pub", v_pub_dwt);
        f.add("v_orig", v_raw_dwt);
        f.add("v_used", v_pub_dwt);
        f.add("v_fl_cyc", v_fl_cyc);
        f.add("v_fl_err", v_fl ? vclock_forensics.regression_inferred_minus_observed_cycles : 0);
        f.add("v_fl_src", v_ok && vclock_forensics.dwt_interval_sample_accepted ? 1U : 0U);
        f.add("v_fl_reason", v_ok ? vclock_forensics.dwt_interval_reject_streak : 0U);
        f.add("v_fl_acc", v_ok ? vclock_forensics.dwt_interval_accept_count : 0U);
        f.add("v_fl_rej", v_ok ? vclock_forensics.dwt_interval_reject_count : 0U);
        f.add("v_fl_bkt", v_ok ? vclock_forensics.dwt_interval_resync_count : 0U);
        f.add("v_fl_ierr", v_ok ? vclock_forensics.dwt_interval_residual_cycles : 0);
        payload_add_micro_court_fields(f, "v", v_ok, vclock_forensics);

        const bool o1_ok = ocxo1_forensics_valid;
        const bool o1_fl = floorline_candidate_present(o1_ok, ocxo1_forensics);
        const uint32_t o1_fl_cyc = o1_fl
            ? floorline_inferred_interval_cycles(o1_fl, ocxo1_forensics)
            : 0U;
        const uint32_t o1_raw_dwt =
            o1_ok ? ocxo1_forensics.dwt_original_at_event : 0U;
        const uint32_t o1_ema_dwt =
            o1_ok ? ocxo1_forensics.dwt_ema_dwt_at_event : 0U;
        const uint32_t o1_fl_dwt =
            o1_fl ? ocxo1_forensics.regression_inferred_dwt_at_event : 0U;
        const uint32_t o1_pub_dwt =
            o1_ok ? ocxo1_forensics.dwt_used_at_event : 0U;
        f.add("o1_obs", o1_ok ? ocxo1_forensics.dwt_interval_observed_cycles : 0U);
        f.add("o1_eff", o1_ok ? ocxo1_forensics.dwt_interval_effective_cycles : 0U);
        f.add("o1_res", o1_ok ? ocxo1_forensics.dwt_interval_residual_cycles : 0);
        f.add("o1_raw", o1_raw_dwt);
        f.add("o1_ema", o1_ema_dwt);
        f.add("o1_fl", o1_fl_dwt);
        f.add("o1_pub", o1_pub_dwt);
        f.add("o1_orig", o1_raw_dwt);
        f.add("o1_used", o1_pub_dwt);
        f.add("o1_fl_cyc", o1_fl_cyc);
        f.add("o1_fl_err", o1_fl ? ocxo1_forensics.regression_inferred_minus_observed_cycles : 0);
        f.add("o1_fl_src", o1_ok && ocxo1_forensics.dwt_interval_sample_accepted ? 1U : 0U);
        f.add("o1_fl_reason", o1_ok ? ocxo1_forensics.dwt_interval_reject_streak : 0U);
        f.add("o1_fl_acc", o1_ok ? ocxo1_forensics.dwt_interval_accept_count : 0U);
        f.add("o1_fl_rej", o1_ok ? ocxo1_forensics.dwt_interval_reject_count : 0U);
        f.add("o1_fl_bkt", o1_ok ? ocxo1_forensics.dwt_interval_resync_count : 0U);
        f.add("o1_fl_ierr", o1_ok ? ocxo1_forensics.dwt_interval_residual_cycles : 0);
        payload_add_micro_court_fields(f, "o1", o1_ok, ocxo1_forensics);
        f.add("o1_pps_res", pps_residuals.ocxo1_valid
                            ? pps_residuals.ocxo1_fast_residual_ns
                            : 0LL);

        const bool o2_ok = ocxo2_forensics_valid;
        const bool o2_fl = floorline_candidate_present(o2_ok, ocxo2_forensics);
        const uint32_t o2_fl_cyc = o2_fl
            ? floorline_inferred_interval_cycles(o2_fl, ocxo2_forensics)
            : 0U;
        const uint32_t o2_raw_dwt =
            o2_ok ? ocxo2_forensics.dwt_original_at_event : 0U;
        const uint32_t o2_ema_dwt =
            o2_ok ? ocxo2_forensics.dwt_ema_dwt_at_event : 0U;
        const uint32_t o2_fl_dwt =
            o2_fl ? ocxo2_forensics.regression_inferred_dwt_at_event : 0U;
        const uint32_t o2_pub_dwt =
            o2_ok ? ocxo2_forensics.dwt_used_at_event : 0U;
        f.add("o2_obs", o2_ok ? ocxo2_forensics.dwt_interval_observed_cycles : 0U);
        f.add("o2_eff", o2_ok ? ocxo2_forensics.dwt_interval_effective_cycles : 0U);
        f.add("o2_res", o2_ok ? ocxo2_forensics.dwt_interval_residual_cycles : 0);
        f.add("o2_raw", o2_raw_dwt);
        f.add("o2_ema", o2_ema_dwt);
        f.add("o2_fl", o2_fl_dwt);
        f.add("o2_pub", o2_pub_dwt);
        f.add("o2_orig", o2_raw_dwt);
        f.add("o2_used", o2_pub_dwt);
        f.add("o2_fl_cyc", o2_fl_cyc);
        f.add("o2_fl_err", o2_fl ? ocxo2_forensics.regression_inferred_minus_observed_cycles : 0);
        f.add("o2_fl_src", o2_ok && ocxo2_forensics.dwt_interval_sample_accepted ? 1U : 0U);
        f.add("o2_fl_reason", o2_ok ? ocxo2_forensics.dwt_interval_reject_streak : 0U);
        f.add("o2_fl_acc", o2_ok ? ocxo2_forensics.dwt_interval_accept_count : 0U);
        f.add("o2_fl_rej", o2_ok ? ocxo2_forensics.dwt_interval_reject_count : 0U);
        f.add("o2_fl_bkt", o2_ok ? ocxo2_forensics.dwt_interval_resync_count : 0U);
        f.add("o2_fl_ierr", o2_ok ? ocxo2_forensics.dwt_interval_residual_cycles : 0);
        payload_add_micro_court_fields(f, "o2", o2_ok, ocxo2_forensics);
        f.add("o2_pps_res", pps_residuals.ocxo2_valid
                            ? pps_residuals.ocxo2_fast_residual_ns
                            : 0LL);
      }
    } else if (TIMEBASE_FORENSICS_SLIM_RAW_CYCLES_PAYLOAD_ENABLED) {
      // Curated campaign-row forensics: enough for raw_cycles /
      // raw_cycles_excursions, including EMA, Yardstick, and FloorLine cycle
      // surfaces, while omitting bulky deep-autopsy objects.
      g_timebase_forensics_slim_count++;
      f.add("minimal", false);
      f.add("slim_raw_cycles", true);
      f.add("floorline_included", TIMEBASE_FORENSICS_FLOORLINE_PAYLOAD_ENABLED);

      payload_add_slim_pps_vclock_edge_forensics(
          f, pps_vclock_edge_forensics_valid, pps_vclock_edge_forensics);

      timebase_build_stage(TIMEBASE_BUILD_STAGE_VCLOCK);
      payload_add_slim_vclock_forensics(f,
                                        public_gnss_ns,
                                        vclock_forensics_valid,
                                        vclock_forensics);

      timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO1);
      payload_add_slim_ocxo_forensics(f,
                                      "ocxo1",
                                      public_ocxo1_ns,
                                      public_ocxo1_measured_ns,
                                      ocxo1_forensics_valid,
                                      ocxo1_forensics,
                                      ocxo1_pps_projected_valid,
                                      ocxo1_pps_projection,
                                      pps_residuals.ocxo1_valid,
                                      pps_residuals.gnss_interval_ns,
                                      pps_residuals.ocxo1_interval_ns,
                                      pps_residuals.ocxo1_fast_residual_ns,
                                      ocxo1_cycle_residual_diag);

      timebase_build_stage(TIMEBASE_BUILD_STAGE_OCXO2);
      payload_add_slim_ocxo_forensics(f,
                                      "ocxo2",
                                      public_ocxo2_ns,
                                      public_ocxo2_measured_ns,
                                      ocxo2_forensics_valid,
                                      ocxo2_forensics,
                                      ocxo2_pps_projected_valid,
                                      ocxo2_pps_projection,
                                      pps_residuals.ocxo2_valid,
                                      pps_residuals.gnss_interval_ns,
                                      pps_residuals.ocxo2_interval_ns,
                                      pps_residuals.ocxo2_fast_residual_ns,
                                      ocxo2_cycle_residual_diag);
    } else {
      payload_add_pps_vclock_edge_forensics(f,
                                            pps_vclock_edge_forensics_valid,
                                            pps_vclock_edge_forensics);

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
                                 0ULL,
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
                                 0ULL,
                                 pps_residuals.ocxo2_valid,
                                 pps_residuals.gnss_interval_ns,
                                 pps_residuals.ocxo2_interval_ns,
                                 pps_residuals.ocxo2_fast_residual_ns,
                                 ocxo2_cycle_residual_diag);
    }

    g_timebase_forensics_build_complete_count++;
    g_timebase_last_forensics_build_complete_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_BUILD_COMPLETE);

    if (clocks_watchdog_publication_blocked()) {
      g_timebase_watchdog_gate_count++;
      timebase_build_stage(TIMEBASE_BUILD_STAGE_WATCHDOG_GATE);
      publish_dac_tick("WATCHDOG_GATE_PRE_FORENSICS_PUBLISH");
      return;
    }

    g_timebase_forensics_publish_attempt_count++;
    g_timebase_last_forensics_publish_attempt_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_ATTEMPT);
    publish("TIMEBASE_FORENSICS", f);

    g_timebase_forensics_publish_return_count++;
    g_timebase_last_forensics_publish_return_campaign_seconds = campaign_seconds;
    timebase_build_stage(TIMEBASE_BUILD_STAGE_FORENSICS_PUBLISH_RETURN);
    clocks_beta_feature_set_cached("TIMEBASE_PUBLICATION",
                                   g_clocks_feature_timebase_publication,
                                   system_feature_status_t::NOMINAL,
                                   true);
    clocks_watchdog_arm_campaign_publication();
  }
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
  // New system-config contract: { "ocxo1_dac": <code>, "ocxo2_dac": <code> }.
  // Retain the old command aliases so existing Pi-side callers do not break.
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
  clocks_payload_numeric_integrity_reset();

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
    err.add("campaign_state", "STOPPED");
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

  const char* servo_arg = nullptr;
  const bool servo_supplied =
      payload_try_get_nonempty_string(args, "calibrate_ocxo", servo_arg);

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  const bool dac1_supplied = payload_try_get_ocxo1_dac(args, "command.flash_cut", dac_val);
  if (dac1_supplied) {
    ocxo_dac_pacing_abort_all();
    dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
    if (dac1_ok) ocxo_dac_retry_reset(ocxo1_dac);
  }
  const bool dac2_supplied = payload_try_get_ocxo2_dac(args, "command.flash_cut", dac_val);
  if (dac2_supplied) {
    ocxo_dac_pacing_abort_all();
    dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);
    if (dac2_ok) ocxo_dac_retry_reset(ocxo2_dac);
  }

  if (g_clocks_payload_numeric_integrity_failed) {
    g_flash_cut_reject_count++;
    return clocks_payload_numeric_reject_response("flash_cut_rejected_numeric_integrity");
  }

  if (servo_supplied) {
    calibrate_ocxo_mode = servo_mode_parse(servo_arg);
  }
  if (!dac1_ok || !dac2_ok) {
    calibrate_ocxo_mode = servo_mode_t::OFF;
  }

  safeCopy(g_flash_cut_last_requested_campaign,
           sizeof(g_flash_cut_last_requested_campaign),
           name);
  safeCopy(flash_cut_campaign_name, sizeof(flash_cut_campaign_name), name);
  g_flash_cut_last_dac1_ok = dac1_ok;
  g_flash_cut_last_dac2_ok = dac2_ok;
  g_flash_cut_last_servo_mode_supplied = servo_supplied;
  g_flash_cut_last_servo_mode = calibrate_ocxo_mode;
  g_flash_cut_request_count++;
  request_flash_cut = true;
  safeCopy(g_flash_cut_last_status, sizeof(g_flash_cut_last_status),
           (!dac1_ok || !dac2_ok)
               ? "flash_cut_requested_dac_fault_servo_off"
               : "flash_cut_requested");

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
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("calibrate_ocxo_supplied", servo_supplied);
  return p;
}

static FLASHMEM Payload cmd_start(const Payload& args) {
  clocks_payload_numeric_integrity_reset();

  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    return cmd_flash_cut(args);
  }

  if (!campaign_feature_gate_open()) {
    Payload err;
    err.add("error", "campaign feature gate closed");
    err.add("status", "start_rejected_feature_gate");
    payload_add_campaign_feature_gate(err);
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);

  ocxo_dac_pacing_abort_all();
  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (payload_try_get_ocxo1_dac(args, "command.start", dac_val)) {
    dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
  }
  if (payload_try_get_ocxo2_dac(args, "command.start", dac_val)) {
    dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);
  }

  if (g_clocks_payload_numeric_integrity_failed) {
    return clocks_payload_numeric_reject_response("start_rejected_numeric_integrity");
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
  flash_cut_clear_pending();
  clocks_watchdog_clear_surrender_for_new_lifecycle();
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
  p.add("ocxo1_dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  p.add("ocxo1_dac_voltage",
        ocxo_dac_voltage_from_code((double)ocxo1_dac.dac_hw_code), 9);
  p.add("ocxo2_dac_voltage",
        ocxo_dac_voltage_from_code((double)ocxo2_dac.dac_hw_code), 9);
  p.add("dac_reference_mode", OCXO_DAC_REFERENCE_MODE);
  p.add("dac_safe_max_output_voltage", OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE, 9);
  p.add("dac_safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);
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
  flash_cut_clear_pending();

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
  clocks_watchdog_clear_surrender_for_new_lifecycle();
  request_servo_mode_change = false;
  requested_servo_mode = servo_mode_t::OFF;
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

static FLASHMEM Payload cmd_recover(const Payload& args) {
  clocks_payload_numeric_integrity_reset();

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
    return err;
  }

  recover_dwt_ns   = dwt_ns;
  recover_gnss_ns  = gnss_ns;
  recover_ocxo1_ns = ocxo1_ns;
  recover_ocxo2_ns = ocxo2_ns;

  const char* raw_servo_mode = nullptr;
  servo_mode_t recovered_servo_mode = servo_mode_t::OFF;
  const bool servo_supplied =
      payload_try_get_servo_mode(args, recovered_servo_mode, raw_servo_mode);
  calibrate_ocxo_mode = servo_supplied
      ? recovered_servo_mode
      : servo_mode_t::OFF;
  request_servo_mode_change = false;
  requested_servo_mode = calibrate_ocxo_mode;
  g_servo_mode_last_requested = calibrate_ocxo_mode;
  g_servo_mode_last_committed = calibrate_ocxo_mode;

  g_recover_request_count++;
  g_recover_last_base_count = recover_gnss_ns / CLOCKS_BETA_NS_PER_SECOND;
  g_recover_last_expected_first_public_count =
      g_recover_last_base_count + 1ULL;
  g_recover_last_base_gnss_ns = recover_gnss_ns;
  g_recover_last_base_dwt_ns = recover_dwt_ns;
  g_recover_last_base_ocxo1_ns = recover_ocxo1_ns;
  g_recover_last_base_ocxo2_ns = recover_ocxo2_ns;

  if (!recover_welfords_capture(args)) {
    return clocks_payload_numeric_reject_response(
        "recover_rejected_welford_numeric_integrity");
  }

  clocks_watchdog_disarm_campaign_publication();
  interrupt_smartzero_abort();
  request_recover = true;
  request_start   = false;
  request_stop    = false;
  request_zero    = false;
  flash_cut_clear_pending();

  Payload p;
  p.add("status", "recover_requested");
  p.add("welford_recovery_lane_count",
        g_recover_welfords_pending.restored_lane_count);
  p.add("welford_recovery_supplied",
        g_recover_welfords_pending.restored_lane_count != 0U);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count",
        g_recover_last_expected_first_public_count);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("calibrate_ocxo_supplied", servo_supplied);
  p.add("recover_status_report", "REPORT_RECOVERY");
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
  p.add("physical_measured_ns_at_edge", f.physical_measured_ns_at_edge);
  p.add("visible_ns_at_edge", f.visible_ns_at_edge);
  p.add("visible_origin_phase_valid", f.visible_origin_phase_valid);
  p.add("visible_origin_phase_offset_ns", f.visible_origin_phase_offset_ns);
  p.add("counter_nominal_ns_between_edges", f.counter_nominal_ns_between_edges);
  p.add("bridge_interval_valid", f.bridge_interval_valid);
  p.add("bridge_gnss_ns_between_edges", f.bridge_gnss_ns_between_edges);
  p.add("bridge_residual_ns", f.bridge_residual_ns);
  p.add("bridge_anchored", f.bridge_anchored);
  p.add("bridge_phi_cycles", f.bridge_phi_cycles);
  p.add("bridge_span_cycles", f.bridge_span_cycles);
  p.add("bridge_resolved_count", f.bridge_resolved_count);
  p.add("bridge_fallback_count", f.bridge_fallback_count);

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

  Payload dwt_authority;
  dwt_authority.add("synthetic", f.dwt_synthetic);
  dwt_authority.add("original_at_event", f.dwt_original_at_event);
  dwt_authority.add("predicted_at_event", f.dwt_predicted_at_event);
  dwt_authority.add("used_at_event", f.dwt_used_at_event);
  dwt_authority.add("isr_entry_raw", f.dwt_isr_entry_raw);
  dwt_authority.add("event_from_isr_entry_raw",
                    f.dwt_event_from_isr_entry_raw);
  dwt_authority.add("isr_entry_to_event_correction_cycles",
                    f.dwt_isr_entry_to_event_correction_cycles);
  dwt_authority.add("published_minus_event_cycles",
                    f.dwt_published_minus_event_cycles);
  dwt_authority.add("used_minus_event_cycles",
                    f.dwt_used_minus_event_cycles);
  dwt_authority.add("synthetic_error_cycles",
                    f.dwt_synthetic_error_cycles);
  p.add_object("dwt_authority", dwt_authority);


  Payload slipledger;
  slipledger.add("active", f.slipledger_active);
  slipledger.add("event_corrected", f.slipledger_event_corrected);
  slipledger.add("event_violation", f.slipledger_event_violation);
  slipledger.add("ticks", f.slipledger_ticks);
  slipledger.add("event_ticks", f.slipledger_event_ticks);
  slipledger.add("generation", f.slipledger_generation);
  slipledger.add("observe_count", f.slipledger_observe_count);
  slipledger.add("violation_count", f.slipledger_violation_count);
  slipledger.add("correction_count", f.slipledger_correction_count);
  slipledger.add("one_second_violation_count", f.slipledger_one_second_violation_count);
  slipledger.add("one_second_correction_count", f.slipledger_one_second_correction_count);
  slipledger.add("last_dwt_error_cycles", f.slipledger_last_dwt_error_cycles);
  slipledger.add("last_observed_dwt", f.slipledger_last_observed_dwt);
  slipledger.add("last_authored_dwt", f.slipledger_last_authored_dwt);
  slipledger.add("reason_code", f.slipledger_reason_code);
  slipledger.add("last_correction_reason_code", f.slipledger_last_correction_reason_code);
  slipledger.add("last_correction_ticks", f.slipledger_last_correction_ticks);
  slipledger.add("last_correction_dwt_error_cycles", f.slipledger_last_correction_dwt_error_cycles);
  p.add_object("slipledger", slipledger);

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

static FLASHMEM uint64_t report_cached_vclock_ns(void) {
  return g_timebase_last_public_gnss_ns != 0ULL
      ? g_timebase_last_public_gnss_ns
      : (uint64_t)g_gnss_ns_at_pps_vclock;
}

static FLASHMEM uint64_t report_cached_dwt64_cycles(void) {
  return g_timebase_last_public_dwt_total != 0ULL
      ? g_timebase_last_public_dwt_total
      : (uint64_t)g_dwt_cycle_count_total;
}

static FLASHMEM uint64_t report_cached_ocxo1_ns(void) {
  return g_timebase_last_public_ocxo1_ns != 0ULL
      ? g_timebase_last_public_ocxo1_ns
      : (uint64_t)g_ocxo1_measured_gnss_ns_at_pps_vclock;
}

static FLASHMEM uint64_t report_cached_ocxo2_ns(void) {
  return g_timebase_last_public_ocxo2_ns != 0ULL
      ? g_timebase_last_public_ocxo2_ns
      : (uint64_t)g_ocxo2_measured_gnss_ns_at_pps_vclock;
}

static FLASHMEM void add_summary_payload(Payload& p) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_SUMMARY);

  // Cached/report-only summary. Do not call process_time projection helpers
  // from the compact report path.
  Payload summary;
  const uint32_t report_dwt = DWT_CYCCNT;
  const uint64_t dwt64_cycles_at_report = report_cached_dwt64_cycles();
  const uint64_t vclock_ns = report_cached_vclock_ns();
  const uint64_t ocxo1_ns = report_cached_ocxo1_ns();
  const uint64_t ocxo2_ns = report_cached_ocxo2_ns();
  const bool vclock_ok = vclock_ns != 0ULL;
  const bool ocxo1_ok = ocxo1_ns != 0ULL;
  const bool ocxo2_ok = ocxo2_ns != 0ULL;

  summary.add("report_dwt", report_dwt);
  summary.add("report_source", "CACHED_LAST_PUBLIC_OR_ALPHA_ANCHOR");
  summary.add("projection_disabled", true);
  summary.add("dwt64_cycles", dwt64_cycles_at_report);
  summary.add("dwt64_ns", dwt_cycles_to_ns(dwt64_cycles_at_report));
  summary.add("gnss_ns", vclock_ok ? vclock_ns : 0ULL);
  summary.add("vclock_ns", vclock_ok ? vclock_ns : 0ULL);
  summary.add("ocxo1_measured_gnss_ns", ocxo1_ok ? ocxo1_ns : 0ULL);
  summary.add("ocxo2_measured_gnss_ns", ocxo2_ok ? ocxo2_ns : 0ULL);
  summary.add("ocxo1_ns", ocxo1_ok ? ocxo1_ns : 0ULL);
  summary.add("ocxo2_ns", ocxo2_ok ? ocxo2_ns : 0ULL);
  if (vclock_ok && ocxo1_ok) {
    summary.add("ocxo1_minus_vclock_ns", signed_delta_u64(ocxo1_ns, vclock_ns));
  }
  if (vclock_ok && ocxo2_ok) {
    summary.add("ocxo2_minus_vclock_ns", signed_delta_u64(ocxo2_ns, vclock_ns));
  }
  if (ocxo1_ok && ocxo2_ok) {
    summary.add("ocxo2_minus_ocxo1_ns", signed_delta_u64(ocxo2_ns, ocxo1_ns));
  }
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
  p.add("request_flash_cut", request_flash_cut);
  p.add("flash_cut_pending_campaign", flash_cut_campaign_name);
  p.add("flash_cut_request_count", g_flash_cut_request_count);
  p.add("flash_cut_commit_count", g_flash_cut_commit_count);
  p.add("flash_cut_reject_count", g_flash_cut_reject_count);
  p.add("flash_cut_busy_reject_count", g_flash_cut_busy_reject_count);
  p.add("flash_cut_last_status", g_flash_cut_last_status);
  p.add("flash_cut_last_requested_campaign", g_flash_cut_last_requested_campaign);
  p.add("flash_cut_last_from_campaign", g_flash_cut_last_from_campaign);
  p.add("flash_cut_last_to_campaign", g_flash_cut_last_to_campaign);
  p.add("flash_cut_last_boundary_pps_count", g_flash_cut_last_boundary_pps_count);
  p.add("flash_cut_last_raw_gnss_ns", g_flash_cut_last_raw_gnss_ns);
  p.add("flash_cut_last_raw_dwt_cycles", g_flash_cut_last_raw_dwt_cycles);
  p.add("flash_cut_last_raw_ocxo1_ns", g_flash_cut_last_raw_ocxo1_ns);
  p.add("flash_cut_last_raw_ocxo2_ns", g_flash_cut_last_raw_ocxo2_ns);
  p.add("flash_cut_last_dac1_ok", g_flash_cut_last_dac1_ok);
  p.add("flash_cut_last_dac2_ok", g_flash_cut_last_dac2_ok);
  p.add("flash_cut_last_servo_mode_supplied", g_flash_cut_last_servo_mode_supplied);
  p.add("flash_cut_last_servo_mode", servo_mode_str(g_flash_cut_last_servo_mode));
  p.add("watchdog_anomaly_active", watchdog_anomaly_active);
  p.add("watchdog_campaign_publication_armed",
        (bool)watchdog_campaign_publication_armed);
  p.add("watchdog_campaign_surrendered",
        (bool)watchdog_campaign_surrendered);
  p.add("watchdog_campaign_surrender_count",
        (uint32_t)watchdog_campaign_surrender_count);
  p.add("watchdog_anomaly_suppressed_unarmed_count",
        (uint32_t)watchdog_anomaly_suppressed_unarmed_count);
  p.add("watchdog_anomaly_verbose_publish_count",
        (uint32_t)watchdog_anomaly_verbose_publish_count);
  p.add("watchdog_anomaly_legacy_publish_count",
        (uint32_t)watchdog_anomaly_legacy_publish_count);
  p.add("watchdog_campaign_armed", clocks_watchdog_campaign_armed());
  p.add("watchdog_anomaly_publish_pending", watchdog_anomaly_publish_pending);
  p.add("watchdog_anomaly_sequence", watchdog_anomaly_sequence);
  p.add("watchdog_anomaly_reason", watchdog_anomaly_reason);
  p.add("payload_numeric_integrity_fail_count",
        (uint32_t)g_clocks_payload_numeric_integrity_fail_count);
  p.add("payload_numeric_last_reject_reason_id",
        (uint32_t)g_clocks_payload_numeric_last_reject_reason);
  p.add("payload_numeric_last_reject_reason",
        clocks_payload_numeric_reject_reason_name(
            g_clocks_payload_numeric_last_reject_reason));
  p.add("payload_numeric_last_path", g_clocks_payload_numeric_last_path);
  p.add("payload_numeric_last_key", g_clocks_payload_numeric_last_key);
  p.add("payload_numeric_last_token_preview",
        g_clocks_payload_numeric_last_token_preview);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  p.add("campaign_warmup_active", campaign_warmup_active());
  p.add("campaign_warmup_policy", "NO_FIXED_ROW_BURIAL");
  p.add("campaign_warmup_required", 0U);
  p.add("campaign_warmup_remaining", (uint32_t)g_campaign_warmup_remaining);
  p.add("campaign_warmup_suppressed_total",
        (uint32_t)g_campaign_warmup_suppressed_total);
  p.add("start_prologue_policy", "PRIVATE_PPS0_THEN_FIRST_SCIENCE_PPS1");
  p.add("start_prologue_seeded", g_start_prologue_seeded);
  p.add("start_prologue_reference_ready", g_start_prologue_reference_ready);
  p.add("start_prologue_private_candidate_count",
        g_start_prologue_private_candidate_count);
  p.add("start_prologue_release_count", g_start_prologue_release_count);
  p.add("start_prologue_private_limit_count",
        g_start_prologue_private_limit_count);
  p.add("start_handoff_launch_wait_count", g_start_handoff_launch_wait_count);
  p.add("start_handoff_timeout_count", g_start_handoff_timeout_count);
  p.add("start_handoff_timeout_candidates",
        (uint32_t)CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES);
  p.add("dwt_publication_launch_acquisition",
        interrupt_dwt_publication_launch_acquisition_active());
  p.add("start_prologue_last_private_count",
        g_start_prologue_last_private_count);
  p.add("start_prologue_last_release_public_count",
        g_start_prologue_last_release_public_count);
  p.add("start_prologue_fit_endpoint_gate_cycles",
        (uint32_t)CLOCKS_START_PROLOGUE_FIT_ENDPOINT_GATE_CYCLES);
  p.add("start_prologue_last_reason", g_start_prologue_last_reason);
  p.add("start_phaseledger_ready", g_start_phaseledger_last_ready);
  p.add("start_phaseledger_reason", g_start_phaseledger_last_reason);
  p.add("start_phaseledger_first_problem",
        g_start_phaseledger_last_first_problem);
  p.add("start_phaseledger_ocxo1_ready",
        g_start_phaseledger_last_ocxo1_ready);
  p.add("start_phaseledger_ocxo2_ready",
        g_start_phaseledger_last_ocxo2_ready);
  p.add("start_phaseledger_sequence_aligned",
        g_start_phaseledger_last_sequence_aligned);
  p.add("phaseledger_science_require_refined_interval",
        CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL);
  p.add("phaseledger_science_missing_refined_interval_count",
        g_phaseledger_science_missing_refined_interval_count);
  p.add("phaseledger_science_missing_ocxo1_count",
        g_phaseledger_science_missing_ocxo1_count);
  p.add("phaseledger_science_missing_ocxo2_count",
        g_phaseledger_science_missing_ocxo2_count);
  p.add("phaseledger_science_last_missing_public_count",
        g_phaseledger_science_last_missing_public_count);
  p.add("start_prologue_pps0_interval_valid",
        (bool)g_start_prologue_pps0_interval_valid);
  p.add("start_prologue_pps0_pps_obs",
        (uint32_t)g_start_prologue_pps0_pps_obs);
  p.add("start_prologue_pps0_v_obs",
        (uint32_t)g_start_prologue_pps0_v_obs);
  p.add("start_prologue_pps0_v_fl",
        (uint32_t)g_start_prologue_pps0_v_fl);
  p.add("start_prologue_pps0_o1_obs",
        (uint32_t)g_start_prologue_pps0_o1_obs);
  p.add("start_prologue_pps0_o1_fl",
        (uint32_t)g_start_prologue_pps0_o1_fl);
  p.add("start_prologue_pps0_o2_obs",
        (uint32_t)g_start_prologue_pps0_o2_obs);
  p.add("start_prologue_pps0_o2_fl",
        (uint32_t)g_start_prologue_pps0_o2_fl);
  p.add("start_prologue_continuity_check_count",
        g_start_prologue_continuity_check_count);
  p.add("start_prologue_continuity_pass_count",
        g_start_prologue_continuity_pass_count);
  p.add("start_prologue_continuity_reject_count",
        g_start_prologue_continuity_reject_count);
  p.add("start_prologue_last_continuity_ok",
        g_start_prologue_last_continuity_ok);
  p.add("start_prologue_last_reference_interval",
        g_start_prologue_last_reference_interval);
  p.add("start_prologue_last_vclock_interval",
        g_start_prologue_last_vclock_interval);
  p.add("start_prologue_last_ocxo1_interval",
        g_start_prologue_last_ocxo1_interval);
  p.add("start_prologue_last_ocxo2_interval",
        g_start_prologue_last_ocxo2_interval);
  p.add("start_prologue_last_reference_minus_pps0",
        g_start_prologue_last_reference_minus_pps0);
  p.add("start_prologue_last_vclock_minus_pps0",
        g_start_prologue_last_vclock_minus_pps0);
  p.add("start_prologue_last_ocxo1_minus_pps0",
        g_start_prologue_last_ocxo1_minus_pps0);
  p.add("start_prologue_last_ocxo2_minus_pps0",
        g_start_prologue_last_ocxo2_minus_pps0);
  p.add("recover_science_quarantine_required",
        (uint32_t)CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
  p.add("recover_science_quarantine_remaining",
        g_science_residual_quarantine_remaining);
  p.add("recover_science_quarantine_begin_count",
        g_science_residual_quarantine_begin_count);
  p.add("recover_science_quarantine_consumed_count",
        g_science_residual_quarantine_consumed_count);
  p.add("recover_science_quarantine_last_public_count",
        g_science_residual_quarantine_last_public_count);
  p.add("alpha_recover_reprime_count",
        clocks_alpha_recover_reprime_count());
  p.add("recover_reattach_active", (bool)g_recover_reattach_active);
  p.add("recover_reattach_degraded_active",
        (bool)g_recover_reattach_degraded_active);
  p.add("recover_reattach_timeout_release_degraded",
        CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED);
  p.add("recover_reattach_timeout_candidates",
        (uint32_t)CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES);
  p.add("recover_reattach_begin_count", g_recover_reattach_begin_count);
  p.add("recover_reattach_hold_count", g_recover_reattach_hold_count);
  p.add("recover_reattach_release_count", g_recover_reattach_release_count);
  p.add("recover_reattach_timeout_count", g_recover_reattach_timeout_count);
  p.add("recover_reattach_degraded_release_count",
        g_recover_reattach_degraded_release_count);
  p.add("recover_reattach_degraded_clear_count",
        g_recover_reattach_degraded_clear_count);
  p.add("recover_reattach_degraded_public_row_count",
        g_recover_reattach_degraded_public_row_count);
  p.add("recover_reattach_degraded_science_suppressed_count",
        g_recover_reattach_degraded_science_suppressed_count);
  p.add("recover_reattach_hidden_candidate_count",
        g_recover_reattach_hidden_candidate_count);
  p.add("recover_reattach_last_hidden_public_count",
        g_recover_reattach_last_hidden_public_count);
  p.add("recover_reattach_last_release_public_count",
        g_recover_reattach_last_release_public_count);
  p.add("recover_reattach_last_degraded_release_public_count",
        g_recover_reattach_last_degraded_release_public_count);
  p.add("recover_reattach_last_degraded_public_count",
        g_recover_reattach_last_degraded_public_count);
  p.add("recover_reattach_last_reason", g_recover_reattach_last_reason);
  p.add("recover_reattach_last_ready", (g_recover_reattach_last_ocxo1.ready && g_recover_reattach_last_ocxo2.ready));
  p.add("recover_reattach_last_origin_ready",
        true);
  p.add("recover_reattach_last_ocxo1_ready",
        g_recover_reattach_last_ocxo1.ready);
  p.add("recover_reattach_last_ocxo2_ready",
        g_recover_reattach_last_ocxo2.ready);

  payload_add_campaign_feature_gate(p);
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
  o1.add("dac_voltage",
         ocxo_dac_voltage_from_code((double)ocxo1_dac.dac_hw_code), 9);
  o1.add("dac_fractional_voltage",
         ocxo_dac_voltage_from_code(ocxo1_dac.dac_fractional), 9);
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
  o1.add("servo_hold_reason", servo_hold_reason_name(ocxo1_dac.servo_hold_reason));
  o1.add("servo_hold_reason_id", (uint32_t)ocxo1_dac.servo_hold_reason);
  o1.add("servo_hold_count", ocxo1_dac.servo_hold_count);
  o1.add("servo_quarantine_reason", servo_hold_reason_name(ocxo1_dac.servo_quarantine_reason));
  o1.add("servo_quarantine_remaining", ocxo1_dac.servo_quarantine_remaining);
  o1.add("servo_quarantine_begin_count", ocxo1_dac.servo_quarantine_begin_count);
  o1.add("servo_quarantine_consumed_count", ocxo1_dac.servo_quarantine_consumed_count);
  o1.add("servo_commit_fault_hold_count", ocxo1_dac.servo_commit_fault_hold_count);
  o1.add("servo_request_overwrite_count", ocxo1_dac.servo_request_overwrite_count);
  o1.add("servo_request_install_count", ocxo1_dac.servo_request_install_count);
  o1.add("servo_request_dither_frame_install_count",
         ocxo1_dac.servo_request_dither_frame_install_count);
  o1.add("servo_request_static_install_count", ocxo1_dac.servo_request_static_install_count);
  o1.add("servo_request_static_write_failure_count",
         ocxo1_dac.servo_request_static_write_failure_count);
  payload_add_servo_input_diag(o1, g_servo_input_ocxo1);
  payload_add_dac_realization_object(o1, "realization", ocxo1_dac);
  o1.add("dither_active_this_frame", ocxo1_dac.dither_active_this_frame);
  o1.add("dither_low_code", (uint32_t)ocxo1_dac.dither_low_code);
  o1.add("dither_high_code", (uint32_t)ocxo1_dac.dither_high_code);
  o1.add("dither_high_ms", (uint32_t)ocxo1_dac.dither_high_ms);
  o1.add("dither_pending_hw_write", ocxo1_dac.dither_pending_hw_write);
  o1.add("dither_pending_hw_code", (uint32_t)ocxo1_dac.dither_pending_hw_code);
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
  o2.add("dac_voltage",
         ocxo_dac_voltage_from_code((double)ocxo2_dac.dac_hw_code), 9);
  o2.add("dac_fractional_voltage",
         ocxo_dac_voltage_from_code(ocxo2_dac.dac_fractional), 9);
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
  o2.add("servo_hold_reason", servo_hold_reason_name(ocxo2_dac.servo_hold_reason));
  o2.add("servo_hold_reason_id", (uint32_t)ocxo2_dac.servo_hold_reason);
  o2.add("servo_hold_count", ocxo2_dac.servo_hold_count);
  o2.add("servo_quarantine_reason", servo_hold_reason_name(ocxo2_dac.servo_quarantine_reason));
  o2.add("servo_quarantine_remaining", ocxo2_dac.servo_quarantine_remaining);
  o2.add("servo_quarantine_begin_count", ocxo2_dac.servo_quarantine_begin_count);
  o2.add("servo_quarantine_consumed_count", ocxo2_dac.servo_quarantine_consumed_count);
  o2.add("servo_commit_fault_hold_count", ocxo2_dac.servo_commit_fault_hold_count);
  o2.add("servo_request_overwrite_count", ocxo2_dac.servo_request_overwrite_count);
  o2.add("servo_request_install_count", ocxo2_dac.servo_request_install_count);
  o2.add("servo_request_dither_frame_install_count",
         ocxo2_dac.servo_request_dither_frame_install_count);
  o2.add("servo_request_static_install_count", ocxo2_dac.servo_request_static_install_count);
  o2.add("servo_request_static_write_failure_count",
         ocxo2_dac.servo_request_static_write_failure_count);
  payload_add_servo_input_diag(o2, g_servo_input_ocxo2);
  payload_add_dac_realization_object(o2, "realization", ocxo2_dac);
  o2.add("dither_active_this_frame", ocxo2_dac.dither_active_this_frame);
  o2.add("dither_low_code", (uint32_t)ocxo2_dac.dither_low_code);
  o2.add("dither_high_code", (uint32_t)ocxo2_dac.dither_high_code);
  o2.add("dither_high_ms", (uint32_t)ocxo2_dac.dither_high_ms);
  o2.add("dither_pending_hw_write", ocxo2_dac.dither_pending_hw_write);
  o2.add("dither_pending_hw_code", (uint32_t)ocxo2_dac.dither_pending_hw_code);
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
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  p.add("reference_mode", OCXO_DAC_REFERENCE_MODE);
  p.add("external_vref_used", false);
  p.add("internal_ref_voltage", OCXO_DAC_INTERNAL_REF_VOLTAGE, 9);
  p.add("output_gain", OCXO_DAC_OUTPUT_GAIN, 3);
  p.add("output_full_scale_voltage", OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE, 9);
  p.add("dac_code_scale", OCXO_DAC_CODE_SCALE, 1);
  p.add("safe_max_output_voltage", OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE, 9);
  p.add("safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);
  p.add("static_rounded_only", ocxo_dac_static_rounded_only_runtime());
  p.add("fractional_stream_possible", ocxo_dac_fractional_stream_possible_runtime());
  p.add("recurring_timer_possible", clocks_ocxo_dac_dither_started());
  p.add("dither_operator_enabled", clocks_ocxo_dac_dither_operator_enabled());
  p.add("dither_service_pending", clocks_ocxo_dac_dither_service_pending());
  p.add("dither_context", clocks_ocxo_dac_dither_context());
  p.add("dither_global_frame_count", clocks_ocxo_dac_dither_global_frame_count());
  p.add("dither_global_schedule_failures", clocks_ocxo_dac_dither_global_schedule_failures());
  p.add("dither_service_arm_count", clocks_ocxo_dac_dither_service_arm_count());
  p.add("dither_service_arm_failures", clocks_ocxo_dac_dither_service_arm_failures());
  p.add("actuator_context", clocks_ocxo_dac_actuator_context());
  p.add("actuator_service_pending", clocks_ocxo_dac_actuator_service_pending());
  p.add("actuator_service_arm_count", clocks_ocxo_dac_actuator_service_arm_count());
  p.add("actuator_service_arm_failures", clocks_ocxo_dac_actuator_service_arm_failures());
  p.add("actuator_commit_attempt_count", clocks_ocxo_dac_actuator_commit_attempt_count());
  p.add("actuator_commit_success_count", clocks_ocxo_dac_actuator_commit_success_count());
  p.add("actuator_commit_failure_count", clocks_ocxo_dac_actuator_commit_failure_count());
  p.add("commit_scheduled", ocxo1_dac.pacing_pending || ocxo2_dac.pacing_pending);
  p.add("commit_selected_lane",
        (g_ocxo_dac_commit_selected == &ocxo1_dac)
            ? 1U
            : ((g_ocxo_dac_commit_selected == &ocxo2_dac) ? 2U : 0U));
  p.add("commit_target", g_ocxo_dac_commit_target, 6);
  p.add("commit_target_hw_code", (uint32_t)g_ocxo_dac_commit_target_hw_code);
  p.add("last_schedule_second", g_ocxo_dac_last_schedule_second);
  p.add("last_commit_second",
        (ocxo1_dac.pacing_last_commit_second > ocxo2_dac.pacing_last_commit_second)
            ? ocxo1_dac.pacing_last_commit_second
            : ocxo2_dac.pacing_last_commit_second);
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
    case 5:  return "PHASE_PROJECTED_RETIRED";
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
  counters.add("guard_legacy_wrap_count",
               clocks_alpha_ocxo_projection_guard_legacy_wrap_count(clock));
  counters.add("guard_sanity_reject_count",
               clocks_alpha_ocxo_projection_guard_sanity_reject_count(clock));
  counters.add("guard_total_count",
               clocks_alpha_ocxo_projection_guard_legacy_wrap_count(clock) +
               clocks_alpha_ocxo_projection_guard_sanity_reject_count(clock));
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


static uint64_t beta_abs_i64(int64_t value) {
  if (value >= 0) return (uint64_t)value;
  return (uint64_t)(-(value + 1)) + 1ULL;
}

static const char* ocxo_projection_guard_lane_verdict(bool snapshot_ok,
                                                      const clocks_alpha_ocxo_pps_projection_snapshot_t& s,
                                                      uint32_t legacy_wrap_count,
                                                      uint32_t sanity_reject_count) {
  if (legacy_wrap_count != 0U || sanity_reject_count != 0U) {
    return "GUARD_CAUGHT_PROJECTION_FAULT";
  }
  if (!snapshot_ok) return "SNAPSHOT_UNAVAILABLE";
  if (!s.valid) return "PROJECTION_NOT_READY_OR_INVALID";
  return "CLEAN";
}

static FLASHMEM void payload_add_ocxo_projection_guard_lane(Payload& parent,
                                                   const char* key,
                                                   time_clock_id_t clock) {
  clocks_alpha_ocxo_pps_projection_snapshot_t s{};
  const bool snapshot_ok = clocks_alpha_ocxo_pps_projection_snapshot(clock, &s);
  const uint32_t legacy_wrap_count =
      clocks_alpha_ocxo_projection_guard_legacy_wrap_count(clock);
  const uint32_t sanity_reject_count =
      clocks_alpha_ocxo_projection_guard_sanity_reject_count(clock);
  const uint32_t guard_total = legacy_wrap_count + sanity_reject_count;

  Payload lane;
  lane.add("snapshot_ok", snapshot_ok);
  lane.add("valid", s.valid);
  lane.add("clock_id", s.clock_id);
  lane.add("verdict", ocxo_projection_guard_lane_verdict(snapshot_ok, s,
                                                         legacy_wrap_count,
                                                         sanity_reject_count));
  lane.add("guard_total_count", guard_total);
  lane.add("guard_legacy_wrap_count", legacy_wrap_count);
  lane.add("guard_sanity_reject_count", sanity_reject_count);
  lane.add("guard_clean", guard_total == 0U);

  Payload identity;
  identity.add("source", s.source);
  identity.add("source_name", ocxo_pps_projection_source_name(s.source));
  identity.add("last_invalid_reason", s.last_invalid_reason);
  identity.add("last_invalid_reason_name",
               ocxo_pps_projection_invalid_reason_name(s.last_invalid_reason));
  identity.add("update_count", s.update_count);
  identity.add("compute_count", s.compute_count);
  identity.add("invalid_no_edge_count", s.invalid_no_edge_count);
  identity.add("invalid_no_interval_count", s.invalid_no_interval_count);
  identity.add("invalid_target_out_of_window_count",
               s.invalid_target_out_of_window_count);
  lane.add_object("identity", identity);

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
  lane.add_object("edge0", edge0);

  Payload edge1;
  edge1.add("dwt_at_edge", s.edge1_dwt_at_edge);
  edge1.add("counter32_at_edge", s.edge1_counter32_at_edge);
  edge1.add("ocxo_ns_at_edge", s.edge1_ocxo_ns_at_edge);
  edge1.add("measured_ns_at_edge", s.edge1_measured_ns_at_edge);
  lane.add_object("edge1", edge1);

  Payload projection;
  projection.add("interval_dwt_cycles", s.interval_dwt_cycles);
  projection.add("interval_ocxo_ns", s.interval_ocxo_ns);
  projection.add("target_delta_cycles", s.target_delta_cycles);
  projection.add("target_delta_raw_cycles", s.target_delta_raw_cycles);
  projection.add("target_remaining_cycles", s.target_remaining_cycles);
  projection.add("target_overrun_cycles", s.target_overrun_cycles);
  projection.add("projected_ocxo_ns_at_pps", s.projected_ocxo_ns_at_pps);
  projection.add("projected_minus_existing_pps_ns",
                 s.projected_minus_existing_pps_ns);
  projection.add("projected_minus_existing_pps_abs_ns",
                 beta_abs_i64(s.projected_minus_existing_pps_ns));
  projection.add("projected_minus_vclock_ns", s.projected_minus_vclock_ns);
  projection.add("projected_minus_vclock_abs_ns",
                 beta_abs_i64(s.projected_minus_vclock_ns));
  projection.add("target_delta_inside_interval",
                 s.interval_dwt_cycles != 0U &&
                 s.target_delta_cycles <= s.interval_dwt_cycles);
  lane.add_object("projection", projection);

  Payload static_projection;
  static_projection.add("latest_actual_interval_cycles",
                        s.latest_actual_interval_cycles);
  static_projection.add("completed_interval_count",
                        s.static_prediction_completed_interval_count);
  static_projection.add("valid", s.static_prediction_valid);
  static_projection.add("advance_limit", s.static_projection_advance_limit);
  static_projection.add("advance_total_count",
                        s.static_projection_advance_count);
  static_projection.add("last_advance_count",
                        s.last_static_projection_advance_count);
  static_projection.add("max_advance_count",
                        s.max_static_projection_advance_count);
  static_projection.add("max_target_overrun_cycles",
                        s.max_target_overrun_cycles);
  lane.add_object("static_projection", static_projection);

  parent.add_object(key, lane);
}

static FLASHMEM Payload cmd_report_ocxo_projection_guard(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_OCXO_PROJECTION_GUARD");
  p.add("description",
        "Alpha OCXO PPS-projection guard counters and last projection snapshot; report-only, not TIMEBASE");
  p.add("doctrine",
        "Alpha must reject modulo/wrap projection ghosts before Beta consumes public ns");
  p.add("beta_policy", "observe_only_no_projection_repair");
  p.add("fault_signature", "2^44_minus_small_ns_or_large_measured_minus_public_jump");
  payload_add_ocxo_projection_guard_lane(p, "ocxo1", time_clock_id_t::OCXO1);
  payload_add_ocxo_projection_guard_lane(p, "ocxo2", time_clock_id_t::OCXO2);
  const uint32_t total_guard_count =
      clocks_alpha_ocxo_projection_guard_legacy_wrap_count(time_clock_id_t::OCXO1) +
      clocks_alpha_ocxo_projection_guard_sanity_reject_count(time_clock_id_t::OCXO1) +
      clocks_alpha_ocxo_projection_guard_legacy_wrap_count(time_clock_id_t::OCXO2) +
      clocks_alpha_ocxo_projection_guard_sanity_reject_count(time_clock_id_t::OCXO2);
  p.add("guard_total_count", total_guard_count);
  p.add("guard_clean", total_guard_count == 0U);
  return p;
}

static FLASHMEM void payload_add_startup_handoff_payload(Payload& p);

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
  counters.add("forensics_disabled_count", g_timebase_forensics_disabled_count);
  counters.add("forensics_minimal_count", g_timebase_forensics_minimal_count);
  counters.add("forensics_micro_raw_count", g_timebase_forensics_micro_raw_count);
  counters.add("forensics_slim_count", g_timebase_forensics_slim_count);
  counters.add("recover_welford_capture_count", g_recover_welford_capture_count);
  counters.add("recover_welford_restore_count", g_recover_welford_restore_count);
  counters.add("recover_welford_last_restored_lane_count",
               g_recover_welford_last_restored_lane_count);
  counters.add("alpha_recover_reprime_count",
               clocks_alpha_recover_reprime_count());
  counters.add("recover_science_quarantine_required",
               (uint32_t)CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
  counters.add("recover_science_quarantine_begin_count",
               g_science_residual_quarantine_begin_count);
  counters.add("recover_science_quarantine_consumed_count",
               g_science_residual_quarantine_consumed_count);
  counters.add("recover_science_quarantine_remaining",
               g_science_residual_quarantine_remaining);
  counters.add("recover_science_quarantine_last_public_count",
               g_science_residual_quarantine_last_public_count);
  counters.add("recover_reattach_begin_count", g_recover_reattach_begin_count);
  counters.add("recover_reattach_hold_count", g_recover_reattach_hold_count);
  counters.add("recover_reattach_release_count", g_recover_reattach_release_count);
  counters.add("recover_reattach_timeout_count", g_recover_reattach_timeout_count);
  counters.add("recover_reattach_degraded_release_count",
               g_recover_reattach_degraded_release_count);
  counters.add("recover_reattach_degraded_clear_count",
               g_recover_reattach_degraded_clear_count);
  counters.add("recover_reattach_degraded_public_row_count",
               g_recover_reattach_degraded_public_row_count);
  counters.add("recover_reattach_degraded_science_suppressed_count",
               g_recover_reattach_degraded_science_suppressed_count);
  counters.add("recover_reattach_hidden_candidate_count",
               g_recover_reattach_hidden_candidate_count);
  counters.add("recover_reattach_last_hidden_public_count",
               g_recover_reattach_last_hidden_public_count);
  counters.add("recover_reattach_last_release_public_count",
               g_recover_reattach_last_release_public_count);
  counters.add("recover_reattach_last_degraded_release_public_count",
               g_recover_reattach_last_degraded_release_public_count);
  counters.add("recover_reattach_last_degraded_public_count",
               g_recover_reattach_last_degraded_public_count);
  counters.add("flash_cut_request_count", g_flash_cut_request_count);
  counters.add("flash_cut_commit_count", g_flash_cut_commit_count);
  counters.add("flash_cut_reject_count", g_flash_cut_reject_count);
  counters.add("flash_cut_busy_reject_count", g_flash_cut_busy_reject_count);
  counters.add("welford_gap_advance_count", g_welford_gap_advance_count);
  counters.add("welford_gap_advance_last_public_count",
               g_welford_gap_advance_last_public_count);
  counters.add("welford_gap_advance_last_lane_count",
               g_welford_gap_advance_last_lane_count);
  p.add_object("counters", counters);

  Payload gates;
  gates.add("forensics_publish_enabled", TIMEBASE_FORENSICS_PUBLISH_ENABLED);
  gates.add("forensics_minimal_payload_enabled", TIMEBASE_FORENSICS_MINIMAL_PAYLOAD_ENABLED);
  gates.add("forensics_micro_raw_cycles_enabled",
            TIMEBASE_FORENSICS_MICRO_RAW_CYCLES_ENABLED);
  gates.add("forensics_slim_raw_cycles_payload_enabled",
            TIMEBASE_FORENSICS_SLIM_RAW_CYCLES_PAYLOAD_ENABLED);
  gates.add("forensics_floorline_payload_enabled",
            TIMEBASE_FORENSICS_FLOORLINE_PAYLOAD_ENABLED);
  gates.add("stop_gate_count", g_timebase_stop_gate_count);
  gates.add("start_zero_gate_count", g_timebase_start_zero_gate_count);
  gates.add("recover_gate_count", g_timebase_recover_gate_count);
  gates.add("flash_cut_gate_count", g_timebase_flash_cut_gate_count);
  gates.add("watchdog_gate_count", g_timebase_watchdog_gate_count);
  gates.add("not_started_gate_count", g_timebase_not_started_gate_count);
  gates.add("warmup_suppressed_count", g_timebase_warmup_suppressed_count);
  gates.add("start_handoff_check_count", g_start_handoff_check_count);
  gates.add("start_handoff_wait_origin_count", g_start_handoff_wait_origin_count);
  gates.add("start_handoff_wait_projection_count", g_start_handoff_wait_projection_count);
  gates.add("start_handoff_commit_count", g_start_handoff_commit_count);
  gates.add("start_handoff_launch_wait_count", g_start_handoff_launch_wait_count);
  gates.add("start_handoff_timeout_count", g_start_handoff_timeout_count);
  gates.add("start_prologue_continuity_check_count",
            g_start_prologue_continuity_check_count);
  gates.add("start_prologue_continuity_pass_count",
            g_start_prologue_continuity_pass_count);
  gates.add("start_prologue_continuity_reject_count",
            g_start_prologue_continuity_reject_count);
  gates.add("start_prologue_last_continuity_ok",
            g_start_prologue_last_continuity_ok);
  gates.add("start_phaseledger_ready", g_start_phaseledger_last_ready);
  gates.add("start_phaseledger_reason", g_start_phaseledger_last_reason);
  gates.add("start_phaseledger_first_problem",
            g_start_phaseledger_last_first_problem);
  gates.add("dwt_publication_launch_acquisition",
            interrupt_dwt_publication_launch_acquisition_active());
  gates.add("campaign_state", campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  gates.add("warmup_active", campaign_warmup_active());
  gates.add("recover_reattach_active", (bool)g_recover_reattach_active);
  gates.add("recover_reattach_degraded_active",
            (bool)g_recover_reattach_degraded_active);
  gates.add("recover_reattach_last_reason", g_recover_reattach_last_reason);
  gates.add("recover_reattach_timeout_candidates",
            (uint32_t)CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES);
  gates.add("recover_reattach_timeout_release_degraded",
            CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED);
  gates.add("watchdog_campaign_publication_armed",
            (bool)watchdog_campaign_publication_armed);
  gates.add("watchdog_campaign_surrendered",
            (bool)watchdog_campaign_surrendered);
  gates.add("watchdog_publication_blocked",
            clocks_watchdog_publication_blocked());
  gates.add("watchdog_campaign_armed", clocks_watchdog_campaign_armed());
  gates.add("watchdog_anomaly_active", watchdog_anomaly_active);
  gates.add("request_start", request_start);
  gates.add("request_stop", request_stop);
  gates.add("request_recover", request_recover);
  gates.add("request_zero", request_zero);
  gates.add("request_flash_cut", request_flash_cut);
  p.add_object("gates", gates);

  Payload recover_reattach;
  recover_reattach.add("ready", (g_recover_reattach_last_ocxo1.ready && g_recover_reattach_last_ocxo2.ready));
  recover_reattach.add("active", (bool)g_recover_reattach_active);
  recover_reattach.add("degraded_active", (bool)g_recover_reattach_degraded_active);
  recover_reattach.add("last_reason", g_recover_reattach_last_reason);
  recover_reattach.add("origin_ready", true);
  recover_reattach.add("pps_vclock_ns", g_recover_reattach_last_ocxo1.expected_pps_vclock_ns);
  recover_reattach.add("recover_reprime_count",
                       g_recover_reattach_last_ocxo1.reprime_count);
  recover_reattach.add("ocxo1_ready", g_recover_reattach_last_ocxo1.ready);
  recover_reattach.add("ocxo1_projection_valid",
                       g_recover_reattach_last_ocxo1.projection_valid);
  recover_reattach.add("ocxo1_floorline_present",
                       g_recover_reattach_last_ocxo1.forensics_ready);
  recover_reattach.add("ocxo1_current_raw_ns",
                       g_recover_reattach_last_ocxo1.current_public_ns);
  recover_reattach.add("ocxo2_ready", g_recover_reattach_last_ocxo2.ready);
  recover_reattach.add("ocxo2_projection_valid",
                       g_recover_reattach_last_ocxo2.projection_valid);
  recover_reattach.add("ocxo2_floorline_present",
                       g_recover_reattach_last_ocxo2.forensics_ready);
  recover_reattach.add("ocxo2_current_raw_ns",
                       g_recover_reattach_last_ocxo2.current_public_ns);
  recover_reattach.add("degraded_release_count",
                       g_recover_reattach_degraded_release_count);
  recover_reattach.add("degraded_public_row_count",
                       g_recover_reattach_degraded_public_row_count);
  recover_reattach.add("degraded_science_suppressed_count",
                       g_recover_reattach_degraded_science_suppressed_count);
  recover_reattach.add("last_degraded_release_public_count",
                       g_recover_reattach_last_degraded_release_public_count);
  recover_reattach.add("last_degraded_public_count",
                       g_recover_reattach_last_degraded_public_count);
  p.add_object("recover_reattach", recover_reattach);

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

  Payload start_handoff;
  start_handoff.add("ready", g_start_handoff_last_ready);
  start_handoff.add("launch_wait_count", g_start_handoff_launch_wait_count);
  start_handoff.add("timeout_count", g_start_handoff_timeout_count);
  start_handoff.add("timeout_candidates",
                    (uint32_t)CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES);
  start_handoff.add("dwt_publication_launch_acquisition",
                    interrupt_dwt_publication_launch_acquisition_active());
  start_handoff.add("origin_ready", g_start_handoff_last_origin_ready);
  start_handoff.add("ocxo1_projection_ready", g_start_handoff_last_ocxo1_projection_ready);
  start_handoff.add("ocxo2_projection_ready", g_start_handoff_last_ocxo2_projection_ready);
  start_handoff.add("raw_gnss_ns", g_start_handoff_last_raw_gnss_ns);
  start_handoff.add("raw_ocxo1_ns", g_start_handoff_last_raw_ocxo1_ns);
  start_handoff.add("raw_ocxo2_ns", g_start_handoff_last_raw_ocxo2_ns);
  start_handoff.add("ocxo1_projected_ns", g_start_handoff_last_ocxo1_projected_ns);
  start_handoff.add("ocxo2_projected_ns", g_start_handoff_last_ocxo2_projected_ns);
  start_handoff.add("ocxo1_projection_vclock_ns", g_start_handoff_last_ocxo1_projection_vclock_ns);
  start_handoff.add("ocxo2_projection_vclock_ns", g_start_handoff_last_ocxo2_projection_vclock_ns);
  p.add_object("start_handoff", start_handoff);

  Payload startup;
  payload_add_startup_handoff_payload(startup);
  p.add_object("startup", startup);

  Payload flash_cut;
  flash_cut.add("pending", request_flash_cut);
  flash_cut.add("pending_campaign", flash_cut_campaign_name);
  flash_cut.add("last_status", g_flash_cut_last_status);
  flash_cut.add("last_requested_campaign", g_flash_cut_last_requested_campaign);
  flash_cut.add("last_from_campaign", g_flash_cut_last_from_campaign);
  flash_cut.add("last_to_campaign", g_flash_cut_last_to_campaign);
  flash_cut.add("last_boundary_pps_count", g_flash_cut_last_boundary_pps_count);
  flash_cut.add("last_raw_gnss_ns", g_flash_cut_last_raw_gnss_ns);
  flash_cut.add("last_raw_dwt_cycles", g_flash_cut_last_raw_dwt_cycles);
  flash_cut.add("last_raw_ocxo1_ns", g_flash_cut_last_raw_ocxo1_ns);
  flash_cut.add("last_raw_ocxo2_ns", g_flash_cut_last_raw_ocxo2_ns);
  flash_cut.add("last_dac1_ok", g_flash_cut_last_dac1_ok);
  flash_cut.add("last_dac2_ok", g_flash_cut_last_dac2_ok);
  flash_cut.add("last_servo_mode_supplied", g_flash_cut_last_servo_mode_supplied);
  flash_cut.add("last_servo_mode", servo_mode_str(g_flash_cut_last_servo_mode));
  p.add_object("flash_cut", flash_cut);

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
  gaps.add("forensics_disabled", !TIMEBASE_FORENSICS_PUBLISH_ENABLED);
  gaps.add("forensics_minimal_payload", TIMEBASE_FORENSICS_MINIMAL_PAYLOAD_ENABLED);
  gaps.add("forensics_micro_raw_cycles",
           TIMEBASE_FORENSICS_MICRO_RAW_CYCLES_ENABLED);
  gaps.add("forensics_slim_raw_cycles_payload",
           TIMEBASE_FORENSICS_SLIM_RAW_CYCLES_PAYLOAD_ENABLED);
  gaps.add("forensics_floorline_payload",
           TIMEBASE_FORENSICS_FLOORLINE_PAYLOAD_ENABLED);
  p.add_object("gaps", gaps);

  return p;
}

static FLASHMEM void payload_add_recover_reattach_flat_fields(
    Payload& p,
    const char* prefix,
    const clocks_alpha_recover_reattach_snapshot_t& s) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value ? value : "");
  };

  add_bool("ready", s.ready);
  add_u32("clock_id", s.clock_id);
  add_u32("reprime_count", s.reprime_count);
  add_bool("forensics_ready", s.forensics_ready);
  add_bool("forensics_valid", s.forensics_valid);
  add_u32("forensics_update_count", s.forensics_update_count);
  add_bool("edge_history_ready", s.edge_history_ready);
  add_u32("edge_history_update_count", s.edge_history_update_count);
  add_bool("projection_ready", s.projection_ready);
  add_bool("projection_valid", s.projection_valid);
  add_u32("projection_update_count", s.projection_update_count);
  add_bool("pps_vclock_match", s.pps_vclock_match);
  add_bool("public_ns_nonzero", s.public_ns_nonzero);
  add_bool("counterledger_mode", s.counterledger_mode);
  add_bool("counterledger_snapshot_ok", s.counterledger_snapshot_ok);
  add_bool("counterledger_valid", s.counterledger_valid);
  add_bool("counterledger_initialized", s.counterledger_initialized);
  add_bool("counterledger_capture_ready", s.counterledger_capture_ready);
  add_bool("counterledger_interval_valid", s.counterledger_interval_valid);
  add_bool("counterledger_phase_valid", s.counterledger_phase_valid);
  add_bool("counterledger_phase_lag_ok", s.counterledger_phase_lag_ok);
  add_bool("counterledger_refined_valid", s.counterledger_refined_valid);
  add_bool("counterledger_refined_interval_valid", s.counterledger_refined_interval_valid);
  add_u32("counterledger_sample_count", s.counterledger_sample_count);
  add_u32("counterledger_pps_sequence", s.counterledger_pps_sequence);
  add_u32("counterledger_phase_pps_sequence", s.counterledger_phase_pps_sequence);
  add_u32("counterledger_phase_lag_pps", s.counterledger_phase_lag_pps);
  add_u32("counterledger_last_delta_ticks", s.counterledger_last_delta_ticks);
  add_u32("counterledger_interval_implausible_count",
          s.counterledger_interval_implausible_count);
  add_u32("counterledger_last_implausible_delta_ticks",
          s.counterledger_last_implausible_delta_ticks);
  add_u32("counterledger_recover_reprime_count",
          s.counterledger_recover_reprime_count);
  add_u32("counterledger_plausible_min_delta_ticks",
          s.counterledger_plausible_min_delta_ticks);
  add_u32("counterledger_plausible_max_delta_ticks",
          s.counterledger_plausible_max_delta_ticks);

  // CounterLedger RECOVER instrumentation.  This is REPORT_RECOVERY-only and
  // intentionally stays out of the 1 Hz TIMEBASE_FRAGMENT spine.
  add_bool("counterledger_last_capture_available",
           s.counterledger_last_capture_available);
  add_bool("counterledger_last_capture_valid",
           s.counterledger_last_capture_valid);
  add_bool("counterledger_last_capture_lane_valid",
           s.counterledger_last_capture_lane_valid);
  add_bool("counterledger_last_capture_all_lanes_valid",
           s.counterledger_last_capture_all_lanes_valid);
  add_bool("counterledger_last_capture_sequence_match",
           s.counterledger_last_capture_sequence_match);
  add_u32("counterledger_last_capture_sequence",
          s.counterledger_last_capture_sequence);
  add_u32("counterledger_last_capture_window_cycles",
          s.counterledger_last_capture_window_cycles);
  add_u32("counterledger_capture_missing_count",
          s.counterledger_capture_missing_count);
  add_u32("counterledger_capture_invalid_count",
          s.counterledger_capture_invalid_count);
  add_u32("counterledger_lane_capture_invalid_count",
          s.counterledger_lane_capture_invalid_count);
  add_u32("counterledger_sequence_mismatch_count",
          s.counterledger_sequence_mismatch_count);
  add_u32("counterledger_all_lanes_invalid_count",
          s.counterledger_all_lanes_invalid_count);
  add_u32("counterledger_capture_gate_attempt_count",
          s.counterledger_capture_gate_attempt_count);
  add_u32("counterledger_capture_gate_ready_count",
          s.counterledger_capture_gate_ready_count);
  add_u32("counterledger_capture_gate_reject_count",
          s.counterledger_capture_gate_reject_count);
  add_u32("counterledger_capture_gate_reason_id",
          s.counterledger_capture_gate_reason_id);
  add_str("counterledger_capture_gate_reason",
          clocks_counterledger_capture_gate_reason_name(
              s.counterledger_capture_gate_reason_id));
  add_u32("counterledger_recover_capture_gate_count",
          s.counterledger_recover_capture_gate_count);
  add_u32("counterledger_recover_capture_ready_count",
          s.counterledger_recover_capture_ready_count);
  add_u32("counterledger_recover_capture_reject_count",
          s.counterledger_recover_capture_reject_count);

  add_u32("counterledger_recover_sample_attempt_count",
          s.counterledger_recover_sample_attempt_count);
  add_u32("counterledger_recover_sample_seed_count",
          s.counterledger_recover_sample_seed_count);
  add_u32("counterledger_recover_sample_interval_accept_count",
          s.counterledger_recover_sample_interval_accept_count);
  add_u32("counterledger_recover_sample_gap_reseed_count",
          s.counterledger_recover_sample_gap_reseed_count);
  add_u32("counterledger_recover_sample_implausible_reseed_count",
          s.counterledger_recover_sample_implausible_reseed_count);
  add_u32("counterledger_last_sample_decision_id",
          s.counterledger_last_sample_decision_id);
  add_str("counterledger_last_sample_decision",
          clocks_counterledger_sample_decision_name(
              s.counterledger_last_sample_decision_id));
  add_u32("counterledger_last_sample_pps_sequence",
          s.counterledger_last_sample_pps_sequence);
  add_u32("counterledger_last_sample_previous_pps_sequence",
          s.counterledger_last_sample_previous_pps_sequence);
  add_u32("counterledger_last_sample_delta_ticks",
          s.counterledger_last_sample_delta_ticks);

  add_u32("counterledger_recover_phase_resolve_attempt_count",
          s.counterledger_recover_phase_resolve_attempt_count);
  add_u32("counterledger_recover_phase_resolve_success_count",
          s.counterledger_recover_phase_resolve_success_count);
  add_u32("counterledger_recover_phase_resolve_unbracketed_count",
          s.counterledger_recover_phase_resolve_unbracketed_count);
  add_u32("counterledger_last_phase_resolve_reason_id",
          s.counterledger_last_phase_resolve_reason_id);
  add_str("counterledger_last_phase_resolve_reason",
          clocks_phaseledger_resolve_reason_name(
              s.counterledger_last_phase_resolve_reason_id));
  add_u32("counterledger_last_phase_resolve_pps_sequence",
          s.counterledger_last_phase_resolve_pps_sequence);
  add_u32("counterledger_last_phase_resolve_counter_delta_ticks",
          s.counterledger_last_phase_resolve_counter_delta_ticks);
  add_u32("counterledger_last_phase_resolve_interval_cycles",
          s.counterledger_last_phase_resolve_interval_cycles);
  add_u32("counterledger_last_phase_resolve_pps_delta_cycles",
          s.counterledger_last_phase_resolve_pps_delta_cycles);
  add_u32("counterledger_refined_interval_accept_count",
          s.counterledger_refined_interval_accept_count);
  add_u32("counterledger_recover_refined_interval_accept_count",
          s.counterledger_recover_refined_interval_accept_count);

  add_u64("counterledger_ns", s.counterledger_ns);
  add_u64("counterledger_interval_ns", s.counterledger_interval_ns);
  add_u64("counterledger_refined_ns", s.counterledger_refined_ns);
  add_u64("counterledger_refined_interval_ns", s.counterledger_refined_interval_ns);
}

static FLASHMEM Payload cmd_report_recovery(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_RECOVERY);

  // Cached and flat by design.  This report is called by Pi while recovery is
  // already stressed; do not refresh Alpha snapshots, call process_time, or
  // build nested diagnostic objects here.  The PPS path owns state refresh.
  Payload p;
  p.add("report", "CLOCKS_RECOVERY");
  p.add("schema", "CLOCKS_RECOVERY_FLAT_V2");
  p.add("description", "cached flat RECOVER wait/release/degraded-publication state");

  p.add("campaign_state", campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("warmup_active", campaign_warmup_active());
  p.add("warmup_mode", (uint32_t)((uint8_t)g_campaign_warmup_mode));
  p.add("recover_reattach_active", (bool)g_recover_reattach_active);
  p.add("recover_reattach_degraded_active", (bool)g_recover_reattach_degraded_active);
  p.add("recover_reattach_reason", g_recover_reattach_last_reason);
  const bool recover_clean_ready =
      !g_recover_reattach_active &&
      !g_recover_reattach_degraded_active &&
      g_recover_reattach_last_ocxo1.ready &&
      g_recover_reattach_last_ocxo2.ready &&
      g_science_residual_quarantine_remaining == 0U &&
      !watchdog_anomaly_active &&
      !clocks_watchdog_publication_blocked();
  p.add("recover_clean_ready", recover_clean_ready);
  p.add("recover_clean_blocked", !recover_clean_ready);
  p.add("interrupt_recover_publication_reset_count",
        interrupt_recover_publication_custody_reset_count());
  p.add("watchdog_anomaly_active", watchdog_anomaly_active);
  p.add("watchdog_campaign_surrendered", (bool)watchdog_campaign_surrendered);
  p.add("watchdog_publication_blocked", clocks_watchdog_publication_blocked());
  p.add("watchdog_campaign_armed", clocks_watchdog_campaign_armed());
  p.add("timebase_last_stage", g_timebase_last_stage);
  p.add("timebase_last_stage_name", timebase_build_stage_name(g_timebase_last_stage));

  p.add("request_count", g_recover_request_count);
  p.add("base_count", g_recover_last_base_count);
  p.add("expected_first_public_count", g_recover_last_expected_first_public_count);
  p.add("base_gnss_ns", g_recover_last_base_gnss_ns);
  p.add("base_dwt_ns", g_recover_last_base_dwt_ns);
  p.add("base_ocxo1_ns", g_recover_last_base_ocxo1_ns);
  p.add("base_ocxo2_ns", g_recover_last_base_ocxo2_ns);
  p.add("continuity_align_pending", (bool)g_recover_continuity_align_pending);
  p.add("continuity_align_count", g_recover_continuity_align_count);
  p.add("continuity_align_failure_count", g_recover_continuity_align_failure_count);
  p.add("continuity_align_requested_public_count",
        g_recover_continuity_align_requested_public_count);
  p.add("continuity_align_last_public_count",
        g_recover_continuity_align_last_public_count);
  p.add("continuity_align_reason", g_recover_continuity_last_reason);
  p.add("continuity_ocxo1_target_ns", g_recover_continuity_ocxo1_target_ns);
  p.add("continuity_ocxo2_target_ns", g_recover_continuity_ocxo2_target_ns);
  p.add("continuity_ocxo1_before_ns", g_recover_continuity_ocxo1_before_ns);
  p.add("continuity_ocxo2_before_ns", g_recover_continuity_ocxo2_before_ns);
  p.add("continuity_ocxo1_after_ns", g_recover_continuity_ocxo1_after_ns);
  p.add("continuity_ocxo2_after_ns", g_recover_continuity_ocxo2_after_ns);
  p.add("continuity_ocxo1_correction_ns",
        g_recover_continuity_ocxo1_correction_ns);
  p.add("continuity_ocxo2_correction_ns",
        g_recover_continuity_ocxo2_correction_ns);
  p.add("last_public_count", g_timebase_last_public_count);
  p.add("last_public_gnss_ns", g_timebase_last_public_gnss_ns);
  p.add("candidate_count", g_timebase_candidate_count);
  p.add("hidden_candidate_count", g_recover_reattach_hidden_candidate_count);
  p.add("last_hidden_public_count", g_recover_reattach_last_hidden_public_count);
  p.add("last_release_public_count", g_recover_reattach_last_release_public_count);
  p.add("timeout_candidates", (uint32_t)CLOCKS_RECOVER_REATTACH_TIMEOUT_CANDIDATES);
  p.add("timeout_release_degraded", CLOCKS_RECOVER_REATTACH_TIMEOUT_RELEASE_DEGRADED);
  p.add("begin_count", g_recover_reattach_begin_count);
  p.add("hold_count", g_recover_reattach_hold_count);
  p.add("release_count", g_recover_reattach_release_count);
  p.add("timeout_count", g_recover_reattach_timeout_count);
  p.add("degraded_release_count", g_recover_reattach_degraded_release_count);
  p.add("degraded_clear_count", g_recover_reattach_degraded_clear_count);
  p.add("degraded_public_row_count", g_recover_reattach_degraded_public_row_count);
  p.add("degraded_science_suppressed_count", g_recover_reattach_degraded_science_suppressed_count);
  p.add("last_degraded_release_public_count", g_recover_reattach_last_degraded_release_public_count);
  p.add("last_degraded_public_count", g_recover_reattach_last_degraded_public_count);
  p.add("science_quarantine_remaining", g_science_residual_quarantine_remaining);
  p.add("science_quarantine_begin_count", g_science_residual_quarantine_begin_count);
  p.add("science_quarantine_consumed_count", g_science_residual_quarantine_consumed_count);
  p.add("science_quarantine_last_public_count", g_science_residual_quarantine_last_public_count);

  p.add("reattach_ready", g_recover_reattach_last_ocxo1.ready && g_recover_reattach_last_ocxo2.ready);
  p.add("reattach_ocxo1_ready", g_recover_reattach_last_ocxo1.ready);
  p.add("reattach_ocxo2_ready", g_recover_reattach_last_ocxo2.ready);
  payload_add_recover_reattach_flat_fields(p, "o1", g_recover_reattach_last_ocxo1);
  payload_add_recover_reattach_flat_fields(p, "o2", g_recover_reattach_last_ocxo2);
  payload_add_stack_witness(p);
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_RECOVERY);
  return p;
}


static FLASHMEM Payload cmd_report_stack_tiny(const Payload&) {
  Payload p;
  const uint32_t current_sp = clocks_stack_witness_sp();
  const bool storage_safe = clocks_stack_witness_storage_safe();
  const uint32_t observed_dtcm_top = clocks_stack_witness_observed_dtcm_top(
      current_sp,
      storage_safe ? (uint32_t)g_clocks_stack_witness.last_sp : 0U,
      storage_safe ? (uint32_t)g_clocks_stack_witness.min_sp : 0U);
  const uint32_t current_bytes =
      clocks_stack_witness_bytes_below_dtcm_top(current_sp, observed_dtcm_top);

  p.add("report", "CLOCKS_STACK_TINY");
  p.add("schema", "CLOCKS_STACK_TINY_V2");
  p.add("current_sp", current_sp);
  p.add("dtcm_top", observed_dtcm_top);
  p.add("dtcm_top_source", "SP_ROUND_UP_GRANULE");
  p.add("dtcm_granule_bytes", CLOCKS_STACK_WITNESS_DTCM_GRANULE_BYTES);
  p.add("ram1_max_top", CLOCKS_STACK_WITNESS_RAM1_MAX_TOP);
  p.add("current_bytes_from_top", current_bytes);
  p.add("current_bytes_below_dtcm_top", current_bytes);
  p.add("storage_addr", (uint32_t)((uintptr_t)&g_clocks_stack_witness));
  p.add("storage_safe", storage_safe);
  p.add("enabled", CLOCKS_STACK_WITNESS_RECORD_COMMANDS && storage_safe);
  if (storage_safe) {
    p.add("count", (uint32_t)g_clocks_stack_witness.record_count);
    p.add("last_sp", (uint32_t)g_clocks_stack_witness.last_sp);
    p.add("min_sp", (uint32_t)g_clocks_stack_witness.min_sp);
    p.add("last_context_id", (uint32_t)g_clocks_stack_witness.last_context);
    p.add("min_context_id", (uint32_t)g_clocks_stack_witness.min_context);
  }
  return p;
}

static FLASHMEM Payload cmd_report_stack(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_STACK);
  Payload p;
  p.add("report", "CLOCKS_STACK_WITNESS");
  p.add("schema", "CLOCKS_STACK_WITNESS_REPORT_V1");
  p.add("description", "raw SP waterline for selected CLOCKS report/recovery paths");
  payload_add_stack_witness(p);
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_STACK);
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

static FLASHMEM void payload_add_interrupt_interval_check(
    Payload& parent,
    const char* key,
    const interrupt_integrity_interval_check_t& s) {
  Payload p;
  p.add("valid", s.valid);
  p.add("last_ok", s.last_ok);
  p.add("test_count", s.test_count);
  p.add("ok_count", s.ok_count);
  p.add("bad_count", s.bad_count);
  p.add("skipped_count", s.skipped_count);
  p.add("sequence", s.sequence);
  p.add("gate_cycles", s.gate_cycles);
  p.add("pps_interval_valid", s.pps_interval_valid);
  p.add("vclock_interval_valid", s.vclock_interval_valid);
  p.add("pps_interval_cycles", s.pps_interval_cycles);
  p.add("vclock_interval_cycles", s.vclock_interval_cycles);
  p.add("vclock_minus_pps_cycles", s.vclock_minus_pps_cycles);
  parent.add_object(key, p);
}

static FLASHMEM void payload_add_interrupt_counter_check(
    Payload& parent,
    const char* key,
    const interrupt_integrity_counter_check_t& s) {
  Payload p;
  p.add("valid", s.valid);
  p.add("last_ok", s.last_ok);
  p.add("test_count", s.test_count);
  p.add("ok_count", s.ok_count);
  p.add("bad_count", s.bad_count);
  p.add("skipped_count", s.skipped_count);
  p.add("sequence", s.sequence);
  p.add("expected_delta_ticks", s.expected_delta_ticks);
  p.add("observed_delta_ticks", s.observed_delta_ticks);
  p.add("observed_minus_expected_ticks", s.observed_minus_expected_ticks);
  p.add("previous_counter32", s.previous_counter32);
  p.add("current_counter32", s.current_counter32);
  parent.add_object(key, p);
}

static FLASHMEM void payload_add_alpha_ns_check(Payload& parent,
                                       const char* key,
                                       const clocks_alpha_integrity_ns_check_t& s) {
  Payload p;
  p.add("valid", s.valid);
  p.add("last_ok", s.last_ok);
  p.add("test_count", s.test_count);
  p.add("ok_count", s.ok_count);
  p.add("bad_count", s.bad_count);
  p.add("skipped_count", s.skipped_count);
  p.add("sequence", s.sequence);
  p.add("gate_ns", s.gate_ns);
  p.add("expected_ns", s.expected_ns);
  p.add("observed_ns", s.observed_ns);
  p.add("observed_minus_expected_ns", s.observed_minus_expected_ns);
  parent.add_object(key, p);
}

static FLASHMEM void payload_add_alpha_ocxo_integrity(
    Payload& parent,
    const char* key,
    const clocks_alpha_integrity_ocxo_check_t& s) {
  Payload p;
  payload_add_alpha_ns_check(p, "interval_equality", s.interval);
  p.add("previous_edge_valid", s.previous_edge_valid);
  p.add("previous_edge_projected_gnss_ns",
        s.previous_edge_projected_gnss_ns);
  p.add("previous_interval_valid", s.previous_interval_valid);
  p.add("previous_interval_ns", s.previous_interval_ns);
  p.add("current_interval_ns", s.current_interval_ns);
  parent.add_object(key, p);
}

static FLASHMEM Payload cmd_report_integrity(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_INTEGRITY");
  p.add("description",
        "Reporting-only draconian integrity counters; no repair, no mutation, no authority changes");

  interrupt_integrity_snapshot_t is{};
  const bool interrupt_ok = interrupt_integrity_snapshot(&is);
  Payload interrupt;
  interrupt.add("available", interrupt_ok);
  interrupt.add("valid", is.valid);
  interrupt.add("snapshot_count", is.snapshot_count);
  payload_add_interrupt_interval_check(interrupt,
                                       "vclock_pps_interval",
                                       is.vclock_pps_interval);
  payload_add_interrupt_counter_check(interrupt,
                                      "vclock_counter",
                                      is.vclock_counter);
  payload_add_interrupt_counter_check(interrupt,
                                      "ocxo1_counter",
                                      is.ocxo1_counter);
  payload_add_interrupt_counter_check(interrupt,
                                      "ocxo2_counter",
                                      is.ocxo2_counter);
  p.add_object("interrupt", interrupt);

  clocks_alpha_integrity_snapshot_t as{};
  const bool alpha_ok = clocks_alpha_integrity_snapshot(&as);
  Payload alpha;
  alpha.add("available", alpha_ok);
  alpha.add("valid", as.valid);
  alpha.add("snapshot_count", as.snapshot_count);
  payload_add_alpha_ns_check(alpha,
                             "vclock_gnss_self_map",
                             as.vclock_gnss_self_map);
  payload_add_alpha_ocxo_integrity(alpha,
                                   "ocxo1_projected_gnss_interval",
                                   as.ocxo1_projected_gnss_interval);
  payload_add_alpha_ocxo_integrity(alpha,
                                   "ocxo2_projected_gnss_interval",
                                   as.ocxo2_projected_gnss_interval);
  p.add_object("alpha", alpha);

  Payload doctrine;
  doctrine.add("reporting_only", true);
  doctrine.add("mutates_clock_authority", false);
  doctrine.add("vclock_pps_interval_gate_cycles", 10U);
  doctrine.add("counter_delta_expected_ticks", (uint32_t)VCLOCK_COUNTS_PER_SECOND);
  doctrine.add("alpha_exact_ns_gate", 0U);
  doctrine.add("alpha_ocxo_interval_gate_ns", 5U);
  doctrine.add("notes",
               "Bad counters are diagnostic evidence only; PPS-vs-VCLOCK compares physical PPS interval against observed latency-corrected VCLOCK interval, not EMA-published VCLOCK");
  p.add_object("doctrine", doctrine);
  return p;
}


static FLASHMEM void payload_add_phaseledger_start_lane(
    Payload& parent,
    const char* key,
    const clocks_alpha_ocxo_counterledger_snapshot_t& s,
    bool ready) {
  Payload lane;
  lane.add("ready", ready);
  lane.add("valid", s.valid);
  lane.add("initialized", s.initialized);
  lane.add("sample_count", s.sample_count);
  lane.add("pps_sequence", s.pps_sequence);

  Payload capture;
  capture.add("available", s.last_capture_available);
  capture.add("valid", s.last_capture_valid);
  capture.add("lane_valid", s.last_capture_lane_valid);
  capture.add("all_lanes_valid", s.last_capture_all_lanes_valid);
  capture.add("sequence_match", s.last_capture_sequence_match);
  capture.add("sequence", s.last_capture_sequence);
  capture.add("window_cycles", s.last_capture_window_cycles);
  lane.add_object("capture", capture);

  Payload integer;
  integer.add("interval_valid", s.interval_valid);
  integer.add("interval_ns", s.interval_valid ? s.interval_ns : 0ULL);
  integer.add("fast_residual_ns", s.interval_valid ? s.fast_residual_ns : 0LL);
  integer.add("last_delta_ticks", s.last_delta_ticks);
  integer.add("ticks64", s.ticks64);
  integer.add("ns", s.ns);
  integer.add("interval_gap_count", s.interval_gap_count);
  integer.add("interval_implausible_count", s.interval_implausible_count);
  integer.add("last_implausible_delta_ticks", s.last_implausible_delta_ticks);
  integer.add("recover_reprime_count", s.recover_reprime_count);
  integer.add("plausible_min_delta_ticks", s.plausible_min_delta_ticks);
  integer.add("plausible_max_delta_ticks", s.plausible_max_delta_ticks);
  lane.add_object("counterledger", integer);

  Payload phase;
  phase.add("valid", s.phase_valid);
  phase.add("pending", s.phase_pending);
  phase.add("source_id", s.phase_source_id);
  phase.add("source", counterledger_phase_source_name(s.phase_source_id));
  phase.add("pps_sequence", s.phase_pps_sequence);
  phase.add("lag_pps", s.phase_lag_pps);
  phase.add("expected_lag_pps",
            (uint32_t)CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS);
  phase.add("lag_ok",
            s.phase_lag_pps <= CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS);
  phase.add("near_boundary", s.phase_near_boundary);
  phase.add("after_last_00_ns", s.phase_after_last_00_ns);
  phase.add("to_next_00_ns", s.phase_to_next_00_ns);
  phase.add("raw_delta_ns", s.phase_raw_delta_ns);
  phase.add("unwrapped_delta_ns", s.phase_unwrapped_delta_ns);
  phase.add("unwrapped_carry_ticks", s.phase_unwrapped_carry_ticks);
  phase.add("wrap_event", s.phase_wrap_event);
  phase.add("wrap_count", s.phase_wrap_count);
  phase.add("resolve_count", s.phase_resolve_count);
  phase.add("pending_overwrite_count", s.phase_pending_overwrite_count);
  phase.add("invalid_count", s.phase_invalid_count);
  phase.add("pps_dwt_at_edge", s.phase_pps_dwt_at_edge);
  phase.add("prev_ocxo_dwt_at_edge", s.phase_prev_ocxo_dwt_at_edge);
  phase.add("next_ocxo_dwt_at_edge", s.phase_next_ocxo_dwt_at_edge);
  phase.add("ocxo_interval_cycles", s.phase_ocxo_interval_cycles);
  phase.add("pps_delta_cycles", s.phase_pps_delta_cycles);
  lane.add_object("phaseledger", phase);

  Payload refined;
  refined.add("valid", s.refined_valid);
  refined.add("ns", s.refined_valid ? s.refined_ns : 0ULL);
  refined.add("interval_valid", s.refined_interval_valid);
  refined.add("interval_ns", s.refined_interval_valid ? s.refined_interval_ns : 0ULL);
  refined.add("fast_residual_ns",
              s.refined_interval_valid ? s.refined_fast_residual_ns : 0LL);
  lane.add_object("refined", refined);

  parent.add_object(key, lane);
}

static FLASHMEM void payload_add_startup_handoff_payload(Payload& p) {
  p.add("report_scope", "START_HANDOFF_PHASELEDGER");
  p.add("authority", clocks_ocxo_public_ns_authority_name());
  p.add("phaseledger_standard_mode", clocks_ocxo_counterledger_mode_enabled());
  p.add("policy",
        "PRIVATE_PPS0_CONTINUITY_PLUS_COUNTERLEDGER_PHASELEDGER_MATURITY");
  p.add("dwt_role",
        "launch_witness_only_not_phaseledger_clock_authority");

  Payload prologue;
  prologue.add("seeded", g_start_prologue_seeded);
  prologue.add("reference_ready", g_start_prologue_reference_ready);
  prologue.add("last_reason", g_start_prologue_last_reason);
  prologue.add("private_candidate_count",
               g_start_prologue_private_candidate_count);
  prologue.add("last_private_count", g_start_prologue_last_private_count);
  prologue.add("release_count", g_start_prologue_release_count);
  prologue.add("last_release_public_count",
               g_start_prologue_last_release_public_count);
  prologue.add("private_limit_count", g_start_prologue_private_limit_count);
  prologue.add("fit_endpoint_gate_cycles",
               (uint32_t)CLOCKS_START_PROLOGUE_FIT_ENDPOINT_GATE_CYCLES);
  prologue.add("timeout_candidates",
               (uint32_t)CLOCKS_START_HANDOFF_TIMEOUT_CANDIDATES);
  prologue.add("launch_wait_count", g_start_handoff_launch_wait_count);
  prologue.add("timeout_count", g_start_handoff_timeout_count);
  prologue.add("dwt_publication_launch_acquisition",
               interrupt_dwt_publication_launch_acquisition_active());
  p.add_object("prologue", prologue);

  Payload continuity;
  continuity.add("pps0_interval_valid",
                 (bool)g_start_prologue_pps0_interval_valid);
  continuity.add("pps0_reference_interval",
                 (uint32_t)g_start_prologue_pps0_pps_obs);
  continuity.add("pps0_vclock_interval",
                 (uint32_t)g_start_prologue_pps0_v_obs);
  continuity.add("pps0_ocxo1_interval",
                 (uint32_t)g_start_prologue_pps0_o1_obs);
  continuity.add("pps0_ocxo2_interval",
                 (uint32_t)g_start_prologue_pps0_o2_obs);
  continuity.add("current_reference_interval",
                 g_pps_vclock_dwt_cycles_between_edges_valid
                     ? (uint32_t)g_pps_vclock_dwt_cycles_between_edges
                     : 0U);
  continuity.add("current_vclock_interval",
                 g_start_prologue_last_vclock_interval);
  continuity.add("current_ocxo1_interval",
                 g_start_prologue_last_ocxo1_interval);
  continuity.add("current_ocxo2_interval",
                 g_start_prologue_last_ocxo2_interval);
  continuity.add("selected_pps_vclock_interval_valid",
                 (bool)g_pps_vclock_dwt_cycles_between_edges_valid);
  continuity.add("check_count", g_start_prologue_continuity_check_count);
  continuity.add("pass_count", g_start_prologue_continuity_pass_count);
  continuity.add("reject_count", g_start_prologue_continuity_reject_count);
  continuity.add("last_ok", g_start_prologue_last_continuity_ok);
  continuity.add("reference_minus_pps0",
                 g_start_prologue_last_reference_minus_pps0);
  continuity.add("vclock_minus_pps0",
                 g_start_prologue_last_vclock_minus_pps0);
  continuity.add("ocxo1_minus_pps0",
                 g_start_prologue_last_ocxo1_minus_pps0);
  continuity.add("ocxo2_minus_pps0",
                 g_start_prologue_last_ocxo2_minus_pps0);
  p.add_object("private_pps0_continuity", continuity);

  Payload phaseledger;
  phaseledger.add("ready", g_start_phaseledger_last_ready);
  phaseledger.add("reason", g_start_phaseledger_last_reason);
  phaseledger.add("first_problem", g_start_phaseledger_last_first_problem);
  phaseledger.add("check_count", g_start_phaseledger_check_count);
  phaseledger.add("ready_count", g_start_phaseledger_ready_count);
  phaseledger.add("wait_count", g_start_phaseledger_wait_count);
  phaseledger.add("wait_snapshot_count",
                  g_start_phaseledger_wait_snapshot_count);
  phaseledger.add("wait_capture_count",
                  g_start_phaseledger_wait_capture_count);
  phaseledger.add("wait_integer_interval_count",
                  g_start_phaseledger_wait_integer_interval_count);
  phaseledger.add("wait_phase_count", g_start_phaseledger_wait_phase_count);
  phaseledger.add("wait_phase_lag_count",
                  g_start_phaseledger_wait_phase_lag_count);
  phaseledger.add("wait_refined_count",
                  g_start_phaseledger_wait_refined_count);
  phaseledger.add("wait_refined_interval_count",
                  g_start_phaseledger_wait_refined_interval_count);
  phaseledger.add("wait_sequence_count",
                  g_start_phaseledger_wait_sequence_count);
  phaseledger.add("ocxo1_ready", g_start_phaseledger_last_ocxo1_ready);
  phaseledger.add("ocxo2_ready", g_start_phaseledger_last_ocxo2_ready);
  phaseledger.add("sequence_aligned",
                  g_start_phaseledger_last_sequence_aligned);
  phaseledger.add("expected_lag_pps",
                  (uint32_t)CLOCKS_START_PHASELEDGER_EXPECTED_LAG_PPS);
  phaseledger.add("science_require_refined_interval",
                  CLOCKS_PHASELEDGER_SCIENCE_REQUIRE_REFINED_INTERVAL);
  phaseledger.add("science_missing_refined_interval_count",
                  g_phaseledger_science_missing_refined_interval_count);
  phaseledger.add("science_missing_ocxo1_count",
                  g_phaseledger_science_missing_ocxo1_count);
  phaseledger.add("science_missing_ocxo2_count",
                  g_phaseledger_science_missing_ocxo2_count);
  phaseledger.add("science_last_missing_public_count",
                  g_phaseledger_science_last_missing_public_count);
  payload_add_phaseledger_start_lane(phaseledger,
                                     "ocxo1",
                                     g_start_phaseledger_last_ocxo1,
                                     g_start_phaseledger_last_ocxo1_ready);
  payload_add_phaseledger_start_lane(phaseledger,
                                     "ocxo2",
                                     g_start_phaseledger_last_ocxo2,
                                     g_start_phaseledger_last_ocxo2_ready);
  p.add_object("phaseledger", phaseledger);
}

static FLASHMEM Payload cmd_report_startup(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_STARTUP_HANDOFF");
  p.add("description",
        "START prologue / private PPS0 / PhaseLedger maturity flight recorder");
  payload_add_startup_handoff_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_COMPACT);
  Payload p;
  p.add("report", "CLOCKS_COMPACT");
  p.add("subreports", "REPORT_STATUS REPORT_SUMMARY REPORT_EPOCH REPORT_SMARTZERO REPORT_INSTALLED_SMARTZERO REPORT_LIVE_SMARTZERO REPORT_FORENSICS REPORT_FORENSICS_VCLOCK REPORT_FORENSICS_OCXO1 REPORT_FORENSICS_OCXO2 REPORT_OCXO_PPS_PROJECTION REPORT_OCXO_PROJECTION_GUARD REPORT_TIMEBASE_PUBLISH REPORT_RECOVERY REPORT_RECOVER_STATUS REPORT_RECOVER REPORT_STACK_TINY REPORT_STACK_WITNESS REPORT_STACK REPORT_STARTUP REPORT_INTEGRITY REPORT_ALPHA_FLOW REPORT_ALPHA_FLOW_VCLOCK REPORT_ALPHA_FLOW_OCXO1 REPORT_ALPHA_FLOW_OCXO2 REPORT_PREDICTION REPORT_STATS REPORT_DAC DITHER_STATUS DITHER_ENABLE DITHER_DISABLE");
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
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_COMPACT);
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

static FLASHMEM Payload cmd_report_gate(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_CAMPAIGN_GATE");
  payload_add_campaign_feature_gate(p);
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

static void cached_report_clock_tuple(uint32_t& report_dwt,
                                      bool& vclock_ok,
                                      uint64_t& vclock_ns,
                                      bool& ocxo1_ok,
                                      uint64_t& ocxo1_ns,
                                      bool& ocxo2_ok,
                                      uint64_t& ocxo2_ns) {
  report_dwt = DWT_CYCCNT;
  vclock_ns = report_cached_vclock_ns();
  ocxo1_ns = report_cached_ocxo1_ns();
  ocxo2_ns = report_cached_ocxo2_ns();
  vclock_ok = vclock_ns != 0ULL;
  ocxo1_ok = ocxo1_ns != 0ULL;
  ocxo2_ok = ocxo2_ns != 0ULL;
}

static FLASHMEM Payload cmd_report_forensics(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_FORENSICS);
  Payload p;
  p.add("report", "CLOCKS_FORENSICS");
  p.add("report_source", "CACHED_NO_PROCESS_TIME_PROJECTION");
  uint32_t report_dwt = 0;
  bool vclock_ok = false;
  bool ocxo1_ok = false;
  bool ocxo2_ok = false;
  uint64_t vclock_ns = 0;
  uint64_t ocxo1_ns = 0;
  uint64_t ocxo2_ns = 0;
  cached_report_clock_tuple(report_dwt, vclock_ok, vclock_ns,
                            ocxo1_ok, ocxo1_ns, ocxo2_ok, ocxo2_ns);
  add_clock_forensics_payload(p, report_dwt,
                              vclock_ok ? vclock_ns : 0ULL, vclock_ok,
                              ocxo1_ok ? ocxo1_ns : 0ULL, ocxo1_ok,
                              ocxo2_ok ? ocxo2_ns : 0ULL, ocxo2_ok);
  return p;
}

static FLASHMEM Payload cmd_report_forensics_vclock(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_FORENSICS_LANE);
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_VCLOCK");
  p.add("report_source", "CACHED_NO_PROCESS_TIME_PROJECTION");
  uint32_t report_dwt = 0;
  bool vclock_ok = false;
  bool ocxo1_ok = false;
  bool ocxo2_ok = false;
  uint64_t vclock_ns = 0;
  uint64_t ocxo1_ns = 0;
  uint64_t ocxo2_ns = 0;
  cached_report_clock_tuple(report_dwt, vclock_ok, vclock_ns,
                            ocxo1_ok, ocxo1_ns, ocxo2_ok, ocxo2_ns);
  add_single_clock_forensics_payload(p, "vclock", time_clock_id_t::VCLOCK,
                                     report_dwt, vclock_ns, vclock_ok,
                                     vclock_ns, vclock_ok);
  return p;
}

static FLASHMEM Payload cmd_report_forensics_ocxo1(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_FORENSICS_LANE);
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_OCXO1");
  p.add("report_source", "CACHED_NO_PROCESS_TIME_PROJECTION");
  uint32_t report_dwt = 0;
  bool vclock_ok = false;
  bool ocxo1_ok = false;
  bool ocxo2_ok = false;
  uint64_t vclock_ns = 0;
  uint64_t ocxo1_ns = 0;
  uint64_t ocxo2_ns = 0;
  cached_report_clock_tuple(report_dwt, vclock_ok, vclock_ns,
                            ocxo1_ok, ocxo1_ns, ocxo2_ok, ocxo2_ns);
  add_single_clock_forensics_payload(p, "ocxo1", time_clock_id_t::OCXO1,
                                     report_dwt, vclock_ns, vclock_ok,
                                     ocxo1_ns, ocxo1_ok);
  return p;
}

static FLASHMEM Payload cmd_report_forensics_ocxo2(const Payload&) {
  clocks_stack_witness_note_command(CLOCKS_STACK_CONTEXT_REPORT_FORENSICS_LANE);
  Payload p;
  p.add("report", "CLOCKS_FORENSICS_OCXO2");
  p.add("report_source", "CACHED_NO_PROCESS_TIME_PROJECTION");
  uint32_t report_dwt = 0;
  bool vclock_ok = false;
  bool ocxo1_ok = false;
  bool ocxo2_ok = false;
  uint64_t vclock_ns = 0;
  uint64_t ocxo1_ns = 0;
  uint64_t ocxo2_ns = 0;
  cached_report_clock_tuple(report_dwt, vclock_ok, vclock_ns,
                            ocxo1_ok, ocxo1_ns, ocxo2_ok, ocxo2_ns);
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

  const uint64_t public_gnss_ns = campaign_public_gnss_ns();
  publish_freq(p, "dwt",
               campaign_total_dwt_ppb(public_gnss_ns,
                                      campaign_public_dwt_total()));
  publish_freq(p, "vclock", 0.0);

  clocks_alpha_tau_snapshot_t ocxo1_tau{};
  clocks_alpha_tau_snapshot_t ocxo2_tau{};
  const bool ocxo1_tau_ok = clocks_alpha_ocxo_tau_snapshot(
      time_clock_id_t::OCXO1, &ocxo1_tau);
  const bool ocxo2_tau_ok = clocks_alpha_ocxo_tau_snapshot(
      time_clock_id_t::OCXO2, &ocxo2_tau);

  const uint64_t report_ocxo1_ns = campaign_public_ocxo1_ns();
  const uint64_t report_ocxo2_ns = campaign_public_ocxo2_ns();
  const bool report_ocxo1_frequency_valid =
      public_gnss_ns != 0ULL && report_ocxo1_ns != 0ULL;
  const bool report_ocxo2_frequency_valid =
      public_gnss_ns != 0ULL && report_ocxo2_ns != 0ULL;

  publish_freq(p, "ocxo1",
               report_ocxo1_frequency_valid
                   ? campaign_total_ppb_from_ratio(public_gnss_ns,
                                                   report_ocxo1_ns)
                   : 0.0);
  publish_freq(p, "ocxo2",
               report_ocxo2_frequency_valid
                   ? campaign_total_ppb_from_ratio(public_gnss_ns,
                                                   report_ocxo2_ns)
                   : 0.0);
  p.add("frequency_source", "PUBLIC_CLOCK_NS_OVER_GNSS_NS");
  p.add("frequency_source_detail",
        (g_recover_continuity_align_count != 0U)
            ? "RECOVERY_CONTINUITY_ALIGNED_PUBLIC_CLOCKFACE"
            : "CAMPAIGN_PUBLIC_CLOCKFACE");
  p.add("ocxo1_frequency_valid", report_ocxo1_frequency_valid);
  p.add("ocxo2_frequency_valid", report_ocxo2_frequency_valid);
  p.add("ocxo1_public_ns", report_ocxo1_ns);
  p.add("ocxo2_public_ns", report_ocxo2_ns);
  p.add("recovery_continuity_align_count", g_recover_continuity_align_count);
  p.add("recovery_continuity_align_last_public_count",
        g_recover_continuity_align_last_public_count);
  p.add("recovery_continuity_ocxo1_correction_ns",
        g_recover_continuity_ocxo1_correction_ns);
  p.add("recovery_continuity_ocxo2_correction_ns",
        g_recover_continuity_ocxo2_correction_ns);
  p.add("ocxo1_alpha_tau_valid", ocxo1_tau_ok && ocxo1_tau.valid);
  p.add("ocxo2_alpha_tau_valid", ocxo2_tau_ok && ocxo2_tau.valid);
  p.add("ocxo1_alpha_tau_samples", ocxo1_tau.sample_count);
  p.add("ocxo2_alpha_tau_samples", ocxo2_tau.sample_count);
  p.add("ocxo1_alpha_tau_intervals", ocxo1_tau.interval_count);
  p.add("ocxo2_alpha_tau_intervals", ocxo2_tau.interval_count);
  p.add("ocxo1_alpha_tau_stderr_ppb", ocxo1_tau.stderr_ppb, 6);
  p.add("ocxo2_alpha_tau_stderr_ppb", ocxo2_tau.stderr_ppb, 6);
  p.add("recover_welford_capture_count", g_recover_welford_capture_count);
  p.add("recover_welford_restore_count", g_recover_welford_restore_count);
  p.add("recover_welford_last_restored_lane_count",
        g_recover_welford_last_restored_lane_count);
  p.add("alpha_recover_reprime_count",
        clocks_alpha_recover_reprime_count());
  p.add("recover_science_quarantine_required",
        (uint32_t)CLOCKS_RECOVER_SCIENCE_QUARANTINE_ROWS);
  p.add("recover_science_quarantine_begin_count",
        g_science_residual_quarantine_begin_count);
  p.add("recover_science_quarantine_consumed_count",
        g_science_residual_quarantine_consumed_count);
  p.add("recover_science_quarantine_remaining",
        g_science_residual_quarantine_remaining);
  p.add("recover_science_quarantine_last_public_count",
        g_science_residual_quarantine_last_public_count);
  p.add("welford_gap_advance_count", g_welford_gap_advance_count);
  p.add("welford_gap_advance_last_public_count",
        g_welford_gap_advance_last_public_count);
  p.add("welford_gap_advance_last_lane_count",
        g_welford_gap_advance_last_lane_count);
  return p;
}



static FLASHMEM void payload_add_dither_status_lane_compact(
    Payload& parent,
    const char* key,
    const ocxo_dac_state_t& dac) {
  Payload lane;
  lane.add("dac", dac.dac_fractional, 6);
  lane.add("hw_code", (uint32_t)dac.dac_hw_code);
  lane.add("voltage", ocxo_dac_voltage_from_code((double)dac.dac_hw_code), 9);

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

static FLASHMEM void payload_add_dither_status_compact(Payload& p) {
  p.add("schema", "CLOCKS_DITHER_STATUS_V2");
  p.add("report", "CLOCKS_DITHER_STATUS");
  p.add("status", "ok");

  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("servo_active", clocks_servo_active());
  p.add("dither_operator_enabled", clocks_ocxo_dac_dither_operator_enabled());
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  p.add("static_rounded_only", ocxo_dac_static_rounded_only_runtime());
  p.add("fractional_stream_possible", ocxo_dac_fractional_stream_possible_runtime());
  p.add("started", clocks_ocxo_dac_dither_started());
  p.add("service_pending", clocks_ocxo_dac_dither_service_pending());
  p.add("write_context", clocks_ocxo_dac_dither_context());
  p.add("actuator_context", clocks_ocxo_dac_actuator_context());
  p.add("actuator_service_pending", clocks_ocxo_dac_actuator_service_pending());
  p.add("actuator_service_arm_count", clocks_ocxo_dac_actuator_service_arm_count());
  p.add("actuator_service_arm_failures", clocks_ocxo_dac_actuator_service_arm_failures());
  p.add("actuator_commit_attempt_count", clocks_ocxo_dac_actuator_commit_attempt_count());
  p.add("actuator_commit_success_count", clocks_ocxo_dac_actuator_commit_success_count());
  p.add("actuator_commit_failure_count", clocks_ocxo_dac_actuator_commit_failure_count());

  p.add("global_frame_count", clocks_ocxo_dac_dither_global_frame_count());
  p.add("global_schedule_failures", clocks_ocxo_dac_dither_global_schedule_failures());
  p.add("service_arm_count", clocks_ocxo_dac_dither_service_arm_count());
  p.add("service_arm_failures", clocks_ocxo_dac_dither_service_arm_failures());
  p.add("ad5693r_init_ok", g_ad5693r_init_ok);
  p.add("safe_max_hw_code", (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE);

  payload_add_dither_status_lane_compact(p, "ocxo1", ocxo1_dac);
  payload_add_dither_status_lane_compact(p, "ocxo2", ocxo2_dac);
}

static FLASHMEM Payload cmd_dither_status(const Payload&) {
  Payload p;
  payload_add_dither_status_compact(p);
  return p;
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

static FLASHMEM Payload cmd_report_dac(const Payload&) {
  Payload p;
  p.add("report", "CLOCKS_DAC");
  add_dac_payload(p);
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
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("ocxo1_dac_hw_code", (uint32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw_code", (uint32_t)ocxo2_dac.dac_hw_code);
  p.add("ocxo1_dac_voltage",
        ocxo_dac_voltage_from_code((double)ocxo1_dac.dac_hw_code), 9);
  p.add("ocxo2_dac_voltage",
        ocxo_dac_voltage_from_code((double)ocxo2_dac.dac_hw_code), 9);
  p.add("realization_mode", ocxo_dac_realization_mode_runtime());
  p.add("reference_mode", OCXO_DAC_REFERENCE_MODE);
  p.add("external_vref_used", false);
  p.add("internal_ref_voltage", OCXO_DAC_INTERNAL_REF_VOLTAGE, 9);
  p.add("output_gain", OCXO_DAC_OUTPUT_GAIN, 3);
  p.add("output_full_scale_voltage", OCXO_DAC_OUTPUT_FULL_SCALE_VOLTAGE, 9);
  p.add("dac_code_scale", OCXO_DAC_CODE_SCALE, 1);
  p.add("safe_max_output_voltage", OCXO_DAC_SAFE_MAX_OUTPUT_VOLTAGE, 9);
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
  { "START",             cmd_start             },
  { "FLASH_CUT",         cmd_flash_cut         },
  { "FLASHCUT",          cmd_flash_cut         },
  { "STOP",              cmd_stop              },
  { "ZERO",              cmd_zero              },
  { "RECOVER",           cmd_recover           },
  { "SERVOS",            cmd_servos            },
  { "REPORT",            cmd_report            },
  { "REPORT_STATUS",     cmd_report_status     },
  { "REPORT_GATE",       cmd_report_gate       },
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
  { "REPORT_OCXO_PROJECTION_GUARD", cmd_report_ocxo_projection_guard },
  { "REPORT_TIMEBASE_PUBLISH", cmd_report_timebase_publish },
  { "REPORT_RECOVERY",         cmd_report_recovery         },
  { "REPORT_RECOVER_STATUS",   cmd_report_recovery         },
  { "REPORT_RECOVER",          cmd_report_recovery         },
  { "REPORT_STACK_TINY",       cmd_report_stack_tiny       },
  { "REPORT_STACK_RAW",        cmd_report_stack_tiny       },
  { "REPORT_STACK_WITNESS",    cmd_report_stack            },
  { "REPORT_STACK",            cmd_report_stack            },
  { "STACK_WITNESS_RESET",     cmd_stack_witness_reset     },
  { "REPORT_STARTUP",          cmd_report_startup          },
  { "REPORT_INTEGRITY",        cmd_report_integrity        },
  { "REPORT_ALPHA_FLOW",       cmd_report_alpha_flow       },
  { "REPORT_ALPHA_FLOW_VCLOCK", cmd_report_alpha_flow_vclock },
  { "REPORT_ALPHA_FLOW_OCXO1",  cmd_report_alpha_flow_ocxo1  },
  { "REPORT_ALPHA_FLOW_OCXO2",  cmd_report_alpha_flow_ocxo2  },
  { "REPORT_PREDICTION",       cmd_report_prediction       },
  { "REPORT_STATS",      cmd_report_stats      },
  { "REPORT_DAC",        cmd_report_dac        },
  { "DITHER_STATUS",     cmd_dither_status     },
  { "DITHER_ENABLE",     cmd_dither_enable     },
  { "DITHER_DISABLE",    cmd_dither_disable    },
  { "WATCHDOG_TEST",     cmd_watchdog_test     },
  { "SET_DAC",           cmd_set_dac           },
  { nullptr,              nullptr               }
};

static const process_subscription_entry_t CLOCKS_SUBSCRIPTIONS[] = {
  { "TIMEBASE_FRAGMENT", on_timebase_fragment },
  { "FEATURE_STATUS",    on_feature_status    },
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
