// ============================================================================
// process_interrupt.cpp
// ============================================================================
//
// PPS and PPS_VCLOCK doctrine (this file authors both):
//
//   • PPS         — physical GPIO edge facts.  The DWT/counter32/ch3 read in
//                   the PPS GPIO ISR, latency-adjusted into event coordinates
//                   that represent the actual electrical PPS moment.  Used for
//                   diagnostics, audit, and any rail that needs the truth
//                   about the pulse itself.
//
//   • PPS_VCLOCK  — the VCLOCK edge selected by the physical PPS pulse.  This
//                   is the canonical timing authority of the system.  Its
//                   counter32/ch3 are the VCLOCK-counter identity of the
//                   chosen edge; its dwt is that same edge's DWT coordinate;
//                   its gnss_ns is VCLOCK-authored (advances by exactly 1e9
//                   per PPS) and is therefore quantized to 100 ns — the ns
//                   value always ends in "00".  PPS_VCLOCK ns is computed
//                   from VCLOCK counter ticks × 100, never from DWT.
//
//   • _raw rule   — _raw is reserved for one thing: ARM_DWT_CYCCNT captured
//                   as the first instruction of an ISR.  The moment a value
//                   is latency-adjusted, the _raw is gone.  _raw values do
//                   NOT propagate.  They live in the ISR stack frame and die
//                   there.  Nothing in a data structure, subscription payload,
//                   or TIMEBASE fragment carries _raw.
//
// TimePop is the principled exception to the "no DWT conversion in
// PPS_VCLOCK" rule: it projects CH2 fire facts onto the PPS_VCLOCK timeline
// for scheduling diagnostics. OCXO one-second subscribers receive authored
// OCXO edge facts whose DWT coordinate is EMA-predicted from the lane's
// completed one-second intervals; CLOCKS/Alpha owns OCXO measured-GNSS
// interval construction from consecutive OCXO edge DWTs.
//
// Lanes:
//   VCLOCK : TimePop recurring client on QTimer1 CH2 (10 MHz VCLOCK domain)
//   OCXO1  : QTimer2 CH0, PCS=0, one-second edge compare only
//   OCXO2  : QTimer3 CH3, PCS=3, one-second edge compare only
//   TimePop: QTimer1 CH2, varied compare intervals (foreground scheduler)
//
// The PPS GPIO ISR captures DWT as its first instruction, then reads the
// QTimer1 CH0 low-word counter.  process_interrupt maps that low-word
// hardware observation into the private synthetic 32-bit VCLOCK identity.
// When a rebootstrap is pending, it records the selected VCLOCK edge.
// Native QTimer1 CH2 custody now consumes that pending edge after TimePop has
// processed a CH2 compare event, back-projects to the selected edge, and
// publishes the canonical PPS_VCLOCK epoch.  Native CH2 also owns steady-state
// VCLOCK one-second fact authorship, VCLOCK SmartZero sampling, PPS_RELAY
// deassert, fact-drain arming, and passive 16-bit rollover tending.
// ============================================================================

#include "process_interrupt.h"
#include "process_clocks.h"

#include "config.h"
#include "debug.h"
#include "process.h"
#include "payload.h"
#include "timepop.h"
#include "process_timepop.h"

#include <Arduino.h>
#include "imxrt.h"
#include <math.h>
#include <stdio.h>
#include <strings.h>
#include <climits>


// ============================================================================
// ISR / hot-path placement
// ============================================================================
//
// Interrupt helpers and globals use normal compiler/linker placement so the
// timing code remains easier to read and reason about.

// process_clocks owns watchdog publication/stop semantics.  process_interrupt
// only raises hard timing-identity faults through this narrow boundary.
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3);

// ============================================================================
// Constants
// ============================================================================

// The GPIO/QTimer latency constants are measured by the latency calibration
// process as full software-stimulus-to-ISR paths.  process_interrupt needs the
// event-coordinate ISR-entry correction, so subtract the known software pin
// launch overhead from those totals.
static constexpr uint32_t STIMULUS_LAUNCH_LATENCY_CYCLES = 5;

// PPS_VCLOCK epoch selection: the canonical edge is the first VCLOCK edge
// after the physical PPS pulse.  Counter32 offset is currently 0 because
// the GPIO ISR's bus-delayed counter read is already that selected edge on
// this hardware.  Tunable named constants so live measurements can adjust.
static constexpr uint32_t VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS = 1;
static constexpr int32_t  VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS   = 0;
// The PPS GPIO ISR reads the VCLOCK low word after the selected edge has
// already matured.  Empirically this observation is two 10 MHz ticks later
// than the sacred first-after-PPS VCLOCK edge.
static constexpr uint32_t VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED = 2;
static constexpr int64_t  GNSS_NS_PER_SECOND                    = 1000000000LL;

// DWT→PPS_VCLOCK anchor selection for OCXO/TimePop event projection.
// A DWT event near a PPS boundary may be serviced after the PPS ISR has
// published the next anchor.  The latest anchor can therefore be coherent but
// semantically wrong for that event.  Keep a tiny immutable history and choose
// the newest anchor that makes the event land in a lawful post-anchor window.
static constexpr uint32_t PVC_ANCHOR_RING_SIZE = 4;
static constexpr uint64_t PVC_ANCHOR_MAX_EVENT_OFFSET_NS = 1250000000ULL;

static constexpr uint32_t ANCHOR_SELECT_NONE     = 0;
static constexpr uint32_t ANCHOR_SELECT_LATEST   = 1;
static constexpr uint32_t ANCHOR_SELECT_PREVIOUS = 2;
static constexpr uint32_t ANCHOR_SELECT_OLDER    = 3;
static constexpr uint32_t ANCHOR_SELECT_FAILED   = 4;

static constexpr uint32_t ANCHOR_FAIL_RING_UNSTABLE = 1u << 0;
static constexpr uint32_t ANCHOR_FAIL_NO_ANCHORS    = 1u << 1;
static constexpr uint32_t ANCHOR_FAIL_BAD_ANCHOR    = 1u << 2;
static constexpr uint32_t ANCHOR_FAIL_NO_CPS        = 1u << 3;
static constexpr uint32_t ANCHOR_FAIL_DELTA_RANGE   = 1u << 4;
static constexpr uint32_t ANCHOR_FAIL_NO_PLAUSIBLE  = 1u << 5;

// VCLOCK endpoint repair is currently disabled / witness-only.
//
// The previous active diagnostic path proved useful but could destabilize the
// foreground path while we are still validating the new VCLOCK-domain anchor
// architecture.  Leave the constants and report surface in place, but keep the
// ISR hot path passive until we explicitly re-enable the experiment.
static constexpr bool     VCLOCK_DWT_REPAIR_DIAG_ENABLED = false;
static constexpr uint32_t VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES = 10;
static constexpr uint32_t VCLOCK_DWT_REPAIR_MIN_HISTORY_COUNT = 8;
static constexpr uint32_t VCLOCK_DWT_REPAIR_MAX_PREDICTION_RESIDUAL_CYCLES = 25;

// Epoch-ready PPS capture packet.  The ISR captures the first-instruction DWT
// and the three lane hardware counters in one tiny custody window.  DWT runs
// at ~1.008 GHz, so 101 cycles is roughly one 10 MHz tick.  The packet remains
// available for the entire following second; ZERO can select it asynchronously.
static constexpr uint32_t EPOCH_CAPTURE_MAX_WINDOW_CYCLES =
    (DWT_EXPECTED_PER_PPS + (VCLOCK_COUNTS_PER_SECOND - 1U)) /
    VCLOCK_COUNTS_PER_SECOND;

// VCLOCK/relay TimePop heartbeat.
//
// VCLOCK_HEARTBEAT remains a critical recurring TimePop ISR client on the
// sovereign QTimer1 CH2 rail, but it is no longer a rollover owner.  VCLOCK still uses this rail deliberately:
// TimePop's same-deadline coalescing keeps VCLOCK facts on one perishable ISR
// surface and avoids introducing a competing preemptive VCLOCK compare.
static constexpr uint64_t VCLOCK_HEARTBEAT_PERIOD_NS = 1000000ULL;
static constexpr const char* VCLOCK_HEARTBEAT_NAME = "VCLOCK_HEARTBEAT";

// PPS relay ownership lives in process_interrupt because the visible relay edge
// is a physical PPS witness.  The rising edge is asserted directly inside the
// PPS GPIO ISR; the 500 ms deassert is driven by intrinsic QTimer1 CH2
// elapsed-tick service so relay-off no longer depends on VCLOCK_HEARTBEAT
// callback dispatch.
static constexpr uint64_t PPS_RELAY_OFF_NS = 500000000ULL;
static constexpr uint32_t PPS_RELAY_OFF_CADENCE_TICKS =
    (uint32_t)(PPS_RELAY_OFF_NS / VCLOCK_HEARTBEAT_PERIOD_NS);
static_assert((PPS_RELAY_OFF_NS % VCLOCK_HEARTBEAT_PERIOD_NS) == 0ULL,
              "PPS relay off interval must be an integer VCLOCK_HEARTBEAT period");
static_assert(PPS_RELAY_OFF_CADENCE_TICKS == 500U,
              "PPS relay off interval should be 500 cadence ticks");

// OCXO DWT authorship.  OCXO1 and OCXO2 live on separate QuadTimer
// modules/vectors.  Each OCXO ISR still captures ARM_DWT_CYCCNT as first
// instruction for private evidence, but the subscriber-facing OCXO DWT is now
// the lane-local EMA prediction of the one-second edge.  The raw ISR capture
// is diagnostic only.
static constexpr uint32_t OCXO_DWT_SOURCE_NONE = 0;
static constexpr uint32_t OCXO_DWT_SOURCE_ISR_ENTRY = 1;
static constexpr uint32_t OCXO_DWT_SOURCE_EMA_PREDICTED = 2;

// EMA authority for OCXO one-second DWT publication.  The ISR still maintains
// a 1 kHz rollover ladder because the 16-bit QuadTimer cannot compare a full
// second directly.  Only the 1000th rollover fact is deferred to foreground;
// intermediate 1 kHz facts do not enter a perishable ring.  The published edge
// DWT is last-published + EMA(observed one-second interval), smoothing the
// common-mode residuals that defeated the service-forensics experiment.
static constexpr bool     OCXO_EMA_DWT_AUTHORITY_ENABLED = false;
static constexpr uint32_t OCXO_EMA_SHIFT = 4;  // alpha = 1/16
static constexpr uint32_t OCXO_EMA_ALPHA_NUMERATOR = 1;
static constexpr uint32_t OCXO_EMA_ALPHA_DENOMINATOR = (1U << OCXO_EMA_SHIFT);

// OCXO lane-local cadence.  OCXO1/OCXO2 own a small 1 kHz rollover-only
// compare ladder in their own 10 MHz domains.  Every compare ISR schedules
// the next +10,000 tick target.  Every 1000th local cadence fact becomes the
// public one-second OCXO event consumed by Alpha; the other 999 ticks are
// local rollover maintenance only.
static constexpr uint32_t OCXO_CADENCE_INTERVAL_TICKS = 10000U;
static constexpr uint32_t OCXO_WITNESS_ONE_SECOND_COUNTS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS == 10000000U,
              "OCXO witness edge interval must be one 10 MHz second");
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS ==
              (OCXO_CADENCE_INTERVAL_TICKS * TICKS_PER_SECOND_EVENT),
              "OCXO one-second event must be exactly 1000 local cadence samples");
static constexpr uint32_t OCXO_WITNESS_ARM_WINDOW_TICKS = OCXO_CADENCE_INTERVAL_TICKS;
static constexpr uint32_t OCXO_WITNESS_MIN_ARM_LEAD_TICKS = 64U;

// Quiet-phase OCXO custody — retired/dormant.
//
// The OCXO local compare ladders still hop at 1 kHz because a 10 MHz / 1 s
// hardware compare does not fit in the 16-bit QuadTimer compare span.  The
// quiet-phase experiment is now disabled; the lane publishes only logical
// rollover events, and the subscriber-facing DWT is EMA-predicted.  The constants
// remain visible as report/back-compat surfaces while the old experiment is
// unwound.
static constexpr bool     OCXO_QUIET_PHASE_SAMPLING_ENABLED = true;
static constexpr uint32_t OCXO_QUIET_PHASE_PERIOD_TICKS = OCXO_CADENCE_INTERVAL_TICKS;
static constexpr uint32_t OCXO1_QUIET_PHASE_TICKS = 2500U;  // +250 us
static constexpr uint32_t OCXO2_QUIET_PHASE_TICKS = 7500U;  // +750 us
static constexpr uint32_t OCXO_QUIET_PHASE_MIN_LEAD_TICKS =
    OCXO_WITNESS_MIN_ARM_LEAD_TICKS;
static_assert(OCXO_QUIET_PHASE_PERIOD_TICKS == 10000U,
              "OCXO quiet phase period must be one VCLOCK millisecond");
static_assert(OCXO1_QUIET_PHASE_TICKS < OCXO_QUIET_PHASE_PERIOD_TICKS,
              "OCXO1 quiet phase must be inside the millisecond cell");
static_assert(OCXO2_QUIET_PHASE_TICKS < OCXO_QUIET_PHASE_PERIOD_TICKS,
              "OCXO2 quiet phase must be inside the millisecond cell");

static constexpr uint32_t OCXO_CADENCE_REASON_NONE          = 0;
static constexpr uint32_t OCXO_CADENCE_REASON_BOOTSTRAP     = 1;
static constexpr uint32_t OCXO_CADENCE_REASON_START         = 2;
static constexpr uint32_t OCXO_CADENCE_REASON_SMARTZERO     = 3;
static constexpr uint32_t OCXO_CADENCE_REASON_LOGICAL_GRID  = 4;
static constexpr uint32_t OCXO_CADENCE_REASON_LOGICAL_ZERO  = 5;
static constexpr uint32_t OCXO_CADENCE_REASON_REARM         = 6;
static constexpr uint32_t OCXO_CADENCE_REASON_STOP          = 7;

// OCXO witness scheduling/report classifications.  These are instrumentation
// only: they make VCLOCK_HEARTBEAT arm decisions and OCXO compare-service timing
// visible without changing whether an event is published.
static constexpr uint32_t OCXO_SCHEDULE_DECISION_NONE               = 0;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_INACTIVE           = 1;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_ALREADY_ARMED      = 2;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TARGET_INITIALIZED = 3;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TARGET_ADVANCED    = 4;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TOO_CLOSE          = 5;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW     = 6;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_ARMED              = 7;

static constexpr uint32_t OCXO_SERVICE_CLASS_NONE                   = 0;
static constexpr uint32_t OCXO_SERVICE_CLASS_FALSE_IRQ              = 1;
static constexpr uint32_t OCXO_SERVICE_CLASS_EARLY_PRETARGET        = 2;
static constexpr uint32_t OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET     = 3;

// Linear regression is intentionally disabled for the quiet-phase custody
// checkpoint.  The code surface remains compiled for later reactivation, but
// no VCLOCK/OCXO samples are fed and no subscriber-facing diagnostic result is
// populated.  This removes regression as a confounder while we validate whether
// phase-separated OCXO custody alone collapses the raw cycle residuals.
static constexpr bool     OCXO_LINEAR_REGRESSION_ENABLED = false;
static constexpr uint32_t REGRESSION_SAMPLES_PER_SECOND = TICKS_PER_SECOND_EVENT;
static constexpr uint32_t REGRESSION_COUNTER_DELTA_TICKS = OCXO_CADENCE_INTERVAL_TICKS;
static constexpr uint32_t REGRESSION_MIN_SAMPLE_COUNT = 16;
static constexpr int32_t  REGRESSION_FIT_ERROR_THRESHOLD_CYCLES = 4;
static constexpr double   REGRESSION_Q16_SCALE = 65536.0;

// REGRESSION_SAMPLES report discipline.
//
// This command is deliberately live-only and allocates no retained 1000-sample
// windows.  A prior diagnostic build retained per-sample windows for every
// lane and destabilized the runtime by consuming too much RAM.  This version
// only inspects the existing live regression window already present in the
// baseline HEALTH: EXCELLENT build.
static constexpr uint32_t REGRESSION_SAMPLE_REPORT_SLICE_LIMIT = 8;
static constexpr uint32_t REGRESSION_SAMPLE_REPORT_EXTREME_LIMIT = 4;

static_assert(REGRESSION_SAMPLES_PER_SECOND == 1000U,
              "Regression windows are expected to be 1 kHz / one second");

// VCLOCK regression remains disabled until it gets a non-dropping double-buffered
// 1000-sample window.  The old 32-slot fact ring proved too small for the 1 kHz
// VCLOCK rail and must not participate in timing authority.
static constexpr bool VCLOCK_LINEAR_REGRESSION_ENABLED = false;

// Regression is experimental/diagnostic in this build.  The subscriber-facing
// DWT stays traditional while regression_inferred_dwt_at_event exposes what the
// fitted edge would have been.
static constexpr bool LINEAR_REGRESSION_AUTHORED_DWT = false;
static constexpr bool LINEAR_REGRESSION_DIAGNOSTIC_ONLY = false;

// ============================================================================
// Symmetric OCXO disable matrix (temporary A/B experiment surface)
// ============================================================================
//
// These compile-time switches silence OCXO lanes at the lowest practical
// firmware level while leaving VCLOCK untouched.  A disabled OCXO lane:
//   * does not initialize its QTimer input/compare channel,
//   * does not enable its NVIC IRQ,
//   * does not start local cadence,
//   * does not enqueue/drain perishable facts,
//   * does not publish OCXO one-second events.
//
// Disabled OCXO lanes still receive an explicit synthetic SmartZero proof,
// copied from the locked VCLOCK proof, so Alpha's existing three-lane epoch
// invariant can complete without changing process_clocks.
//
// Normal baseline: both false.  For the VCLOCK-only test set both true.
static constexpr bool OCXO1_DISABLED = false;
static constexpr bool OCXO2_DISABLED = false;
static constexpr bool OCXO_DISABLE_EXPERIMENT = OCXO1_DISABLED || OCXO2_DISABLED;
static constexpr uint32_t OCXO_DISABLED_COUNT =
    (OCXO1_DISABLED ? 1U : 0U) + (OCXO2_DISABLED ? 1U : 0U);

// ============================================================================
// Step 0 migration baseline — report-only safety rails
// ============================================================================
//
// These constants and mirrors do not change interrupt behavior. They simply
// make the current timing architecture and applied NVIC priorities visible in
// INTERRUPT.REPORT before any future migration step touches cadence ownership
// or OCXO event authorship.

static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY  = 0;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY  = 16;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY  = 16;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY = 0;


// ============================================================================
// Passive ISR sanity witness
// ============================================================================
//
// This is intentionally report-only.  It observes event facts already captured
// by an ISR and records only the smallest standard sanity surface:
//
//   CNTR correct/incorrect count + last expected/actual
//   DWT  correct/incorrect count + last expected/actual
//
// It never repairs, suppresses, re-arms, publishes, or changes timing truth.
// Thresholds are deliberately internal constants so the report stays readable.
// CNTR expectations are compensated for known read latency; after compensation
// the expected and actual counter values must match exactly.

static constexpr uint32_t ISR_SANITY_DWT_ERROR_THRESHOLD_CYCLES = 1000U;
static constexpr uint32_t ISR_SANITY_VCLOCK_CNTR_READ_LATENCY_TICKS = 2U;
static constexpr uint32_t ISR_SANITY_OCXO_CNTR_READ_LATENCY_TICKS = 1U;
// OCXO CNTR reads occur after first-instruction DWT capture and a slow
// peripheral counter read.  The exact observed low word may therefore be the
// compare tooth plus a tiny positive read-latency advance.  This is not a
// broad +/- window: negative/early offsets remain hard failures, and anything
// beyond this small positive read-latency band is counted incorrect.
static constexpr uint32_t ISR_SANITY_OCXO_CNTR_MIN_READ_LATENCY_TICKS = 0U;
static constexpr uint32_t ISR_SANITY_OCXO_CNTR_MAX_READ_LATENCY_TICKS = 2U;
static constexpr uint32_t ISR_SANITY_PPS_CNTR_READ_LATENCY_TICKS = 0U;

struct isr_sanity_diag_t {
  uint32_t cntr_correct_count = 0;
  uint32_t cntr_incorrect_count = 0;
  uint32_t cntr_expected = 0;
  uint32_t cntr_actual = 0;

  uint32_t dwt_correct_count = 0;
  uint32_t dwt_incorrect_count = 0;
  uint32_t dwt_expected = 0;
  uint32_t dwt_actual = 0;
};

static isr_sanity_diag_t g_isr_sanity_vclock_ch2 = {};
static isr_sanity_diag_t g_isr_sanity_ocxo1 = {};
static isr_sanity_diag_t g_isr_sanity_ocxo2 = {};
static isr_sanity_diag_t g_isr_sanity_pps = {};

// VCLOCK sanity is recorded on public one-second CH2 facts, not every CH2
// scheduler fire.  CH2 also carries TimePop scheduling traffic, and applying a
// single DWT interval expectation to mixed CH2 events creates false errors.
static bool     g_isr_sanity_vclock_ch2_prev_valid = false;
static uint32_t g_isr_sanity_vclock_ch2_prev_counter32 = 0;
static uint32_t g_isr_sanity_vclock_ch2_prev_dwt = 0;

static bool     g_isr_sanity_pps_prev_valid = false;
static uint32_t g_isr_sanity_pps_prev_counter32 = 0;
static uint32_t g_isr_sanity_pps_prev_dwt = 0;

static inline uint32_t isr_sanity_abs_i32(int32_t value) {
  return (uint32_t)(value < 0 ? -(int64_t)value : (int64_t)value);
}

static void isr_sanity_reset(isr_sanity_diag_t& s) {
  s = isr_sanity_diag_t{};
}

static void isr_sanity_record_cntr(isr_sanity_diag_t& s,
                                   bool has_expectation,
                                   uint32_t expected,
                                   uint32_t actual) {
  if (!has_expectation) return;

  s.cntr_expected = expected;
  s.cntr_actual = actual;

  if (actual == expected) {
    s.cntr_correct_count++;
  } else {
    s.cntr_incorrect_count++;
  }
}

static void isr_sanity_record_cntr_latency_band(isr_sanity_diag_t& s,
                                                bool has_expectation,
                                                uint16_t target_low16,
                                                uint16_t actual_low16,
                                                uint32_t min_latency_ticks,
                                                uint32_t max_latency_ticks,
                                                uint32_t preferred_latency_ticks) {
  if (!has_expectation) return;

  const uint16_t preferred_expected =
      (uint16_t)(target_low16 + (uint16_t)preferred_latency_ticks);
  uint16_t matched_expected = preferred_expected;
  bool matched = false;

  for (uint32_t latency = min_latency_ticks;
       latency <= max_latency_ticks;
       latency++) {
    const uint16_t candidate = (uint16_t)(target_low16 + (uint16_t)latency);
    if (actual_low16 == candidate) {
      matched_expected = candidate;
      matched = true;
      break;
    }
  }

  s.cntr_expected = (uint32_t)matched_expected;
  s.cntr_actual = (uint32_t)actual_low16;

  if (matched) {
    s.cntr_correct_count++;
  } else {
    s.cntr_incorrect_count++;
  }
}

static void isr_sanity_record_dwt(isr_sanity_diag_t& s,
                                  bool has_expectation,
                                  uint32_t expected,
                                  uint32_t actual) {
  if (!has_expectation) return;

  s.dwt_expected = expected;
  s.dwt_actual = actual;

  const int32_t delta = (int32_t)((int64_t)actual - (int64_t)expected);
  if (isr_sanity_abs_i32(delta) <= ISR_SANITY_DWT_ERROR_THRESHOLD_CYCLES) {
    s.dwt_correct_count++;
  } else {
    s.dwt_incorrect_count++;
  }
}

static isr_sanity_diag_t* isr_sanity_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return &g_isr_sanity_vclock_ch2;
  if (kind == interrupt_subscriber_kind_t::OCXO1)  return &g_isr_sanity_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2)  return &g_isr_sanity_ocxo2;
  return nullptr;
}


// ============================================================================
// PPS / PPS_VCLOCK doctrine
// ============================================================================
//
// pps_t and pps_vclock_t are defined publicly in process_interrupt.h.
// Both structs carry only event-coordinate (latency-adjusted) values; no
// _raw is published.  They share a sequence number — they describe the
// same physical edge from two perspectives (physical truth vs canonical
// VCLOCK-selected edge).

// ============================================================================
// Snapshot store (seqlock)
// ============================================================================

struct snapshot_store_t {
  volatile uint32_t seq = 0;

  volatile uint32_t pps_sequence              = 0;
  volatile uint32_t pps_dwt_at_edge           = 0;
  volatile uint32_t pps_counter32_at_edge     = 0;
  volatile uint16_t pps_ch3_at_edge           = 0;

  volatile uint32_t pvc_sequence              = 0;
  volatile uint32_t pvc_dwt_at_edge           = 0;
  volatile uint32_t pvc_counter32_at_edge     = 0;
  volatile uint16_t pvc_ch3_at_edge           = 0;
};

static snapshot_store_t g_store;

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static void store_publish(const pps_t& pps, const pps_vclock_t& pvc) {
  g_store.seq++;
  dmb_barrier();
  g_store.pps_sequence          = pps.sequence;
  g_store.pps_dwt_at_edge       = pps.dwt_at_edge;
  g_store.pps_counter32_at_edge = pps.counter32_at_edge;
  g_store.pps_ch3_at_edge       = pps.ch3_at_edge;
  g_store.pvc_sequence          = pvc.sequence;
  g_store.pvc_dwt_at_edge       = pvc.dwt_at_edge;
  g_store.pvc_counter32_at_edge = pvc.counter32_at_edge;
  g_store.pvc_ch3_at_edge       = pvc.ch3_at_edge;
  dmb_barrier();
  g_store.seq++;
}

static bool store_load(pps_t& pps, pps_vclock_t& pvc) {
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_store.seq;
    dmb_barrier();
    pps.sequence          = g_store.pps_sequence;
    pps.dwt_at_edge       = g_store.pps_dwt_at_edge;
    pps.counter32_at_edge = g_store.pps_counter32_at_edge;
    pps.ch3_at_edge       = g_store.pps_ch3_at_edge;
    pvc.sequence          = g_store.pvc_sequence;
    pvc.dwt_at_edge       = g_store.pvc_dwt_at_edge;
    pvc.counter32_at_edge = g_store.pvc_counter32_at_edge;
    pvc.ch3_at_edge       = g_store.pvc_ch3_at_edge;
    pvc.gnss_ns_at_edge   = -1;  // GNSS labels are CLOCKS-owned.
    dmb_barrier();
    const uint32_t s2 = g_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }
  return false;
}

static pps_vclock_t store_load_pvc(void) {
  pps_t pps;
  pps_vclock_t pvc;
  store_load(pps, pvc);
  return pvc;
}

// ============================================================================
// Epoch-ready PPS capture packet (seqlock)
// ============================================================================
//
// This packet is authored on every PPS GPIO ISR from the ISR's opening custody
// window.  The 16-bit hardware reads are retained only as forensic evidence;
// runtime consumers use the translated process_interrupt-owned synthetic
// 32-bit lane coordinates.  ZERO consumers select this already-authored packet
// later.

struct epoch_capture_store_t {
  volatile uint32_t seq = 0;

  volatile bool     valid = false;
  volatile uint32_t sequence = 0;
  volatile uint32_t capture_dwt_start_raw = 0;
  volatile uint32_t capture_dwt_after_vclock_raw = 0;
  volatile uint32_t capture_dwt_end_raw = 0;
  volatile uint32_t capture_window_cycles = 0;
  volatile uint32_t vclock_read_offset_cycles = 0;

  // Latency-adjusted DWT coordinate of the selected PPS_VCLOCK edge,
  // derived directly from the same first-instruction PPS ISR raw capture
  // that authored this epoch-ready packet.  CLOCKS ZERO consumes this field
  // instead of requiring the most recent published PPS_VCLOCK snapshot to
  // have caught up to this GPIO edge.
  volatile uint32_t vclock_dwt_at_edge = 0;

  volatile bool     vclock_capture_valid = false;
  volatile bool     all_lanes_capture_valid = false;

  volatile uint16_t vclock_hardware16_observed = 0;
  volatile uint16_t vclock_hardware16_selected = 0;
  volatile uint16_t ocxo1_hardware16 = 0;
  volatile uint16_t ocxo2_hardware16 = 0;

  volatile uint32_t vclock_counter32 = 0;
  volatile uint32_t ocxo1_counter32 = 0;
  volatile uint32_t ocxo2_counter32 = 0;
};

static epoch_capture_store_t g_epoch_capture_store;

static void epoch_capture_publish(const interrupt_epoch_capture_t& cap) {
  g_epoch_capture_store.seq++;
  dmb_barrier();

  g_epoch_capture_store.valid = cap.valid;
  g_epoch_capture_store.sequence = cap.sequence;
  g_epoch_capture_store.capture_dwt_start_raw = cap.capture_dwt_start_raw;
  g_epoch_capture_store.capture_dwt_after_vclock_raw = cap.capture_dwt_after_vclock_raw;
  g_epoch_capture_store.capture_dwt_end_raw = cap.capture_dwt_end_raw;
  g_epoch_capture_store.capture_window_cycles = cap.capture_window_cycles;
  g_epoch_capture_store.vclock_read_offset_cycles = cap.vclock_read_offset_cycles;
  g_epoch_capture_store.vclock_dwt_at_edge = cap.vclock_dwt_at_edge;
  g_epoch_capture_store.vclock_capture_valid = cap.vclock_capture_valid;
  g_epoch_capture_store.all_lanes_capture_valid = cap.all_lanes_capture_valid;
  g_epoch_capture_store.vclock_hardware16_observed = cap.vclock_hardware16_observed;
  g_epoch_capture_store.vclock_hardware16_selected = cap.vclock_hardware16_selected;
  g_epoch_capture_store.ocxo1_hardware16 = cap.ocxo1_hardware16;
  g_epoch_capture_store.ocxo2_hardware16 = cap.ocxo2_hardware16;
  g_epoch_capture_store.vclock_counter32 = cap.vclock_counter32;
  g_epoch_capture_store.ocxo1_counter32 = cap.ocxo1_counter32;
  g_epoch_capture_store.ocxo2_counter32 = cap.ocxo2_counter32;

  dmb_barrier();
  g_epoch_capture_store.seq++;
}

bool interrupt_last_epoch_capture(interrupt_epoch_capture_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_epoch_capture_store.seq;
    dmb_barrier();

    out->valid = g_epoch_capture_store.valid;
    out->sequence = g_epoch_capture_store.sequence;
    out->capture_dwt_start_raw = g_epoch_capture_store.capture_dwt_start_raw;
    out->capture_dwt_after_vclock_raw = g_epoch_capture_store.capture_dwt_after_vclock_raw;
    out->capture_dwt_end_raw = g_epoch_capture_store.capture_dwt_end_raw;
    out->capture_window_cycles = g_epoch_capture_store.capture_window_cycles;
    out->vclock_read_offset_cycles = g_epoch_capture_store.vclock_read_offset_cycles;
    out->vclock_dwt_at_edge = g_epoch_capture_store.vclock_dwt_at_edge;
    out->vclock_capture_valid = g_epoch_capture_store.vclock_capture_valid;
    out->all_lanes_capture_valid = g_epoch_capture_store.all_lanes_capture_valid;
    out->vclock_hardware16_observed = g_epoch_capture_store.vclock_hardware16_observed;
    out->vclock_hardware16_selected = g_epoch_capture_store.vclock_hardware16_selected;
    out->ocxo1_hardware16 = g_epoch_capture_store.ocxo1_hardware16;
    out->ocxo2_hardware16 = g_epoch_capture_store.ocxo2_hardware16;
    out->vclock_counter32 = g_epoch_capture_store.vclock_counter32;
    out->ocxo1_counter32 = g_epoch_capture_store.ocxo1_counter32;
    out->ocxo2_counter32 = g_epoch_capture_store.ocxo2_counter32;

    dmb_barrier();
    const uint32_t s2 = g_epoch_capture_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out->valid;
  }

  return false;
}

static pps_edge_dispatch_fn g_pps_edge_dispatch = nullptr;
static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// DWT cycles-per-second compatibility
// ============================================================================
//
// process_interrupt no longer asks process_time for dynamic CPS.  The legacy
// interrupt_dynamic_cps() now mirrors CLOCKS static PPS/GPIO-derived
// DWT cycles-per-GNSS-second.  Gamma/dynamic prediction has been retired.

static uint32_t interrupt_vclock_cycles_per_second(void);

uint32_t interrupt_dynamic_cps(void) {
  return interrupt_vclock_cycles_per_second();
}

// ============================================================================
// PPS_VCLOCK anchor history (for DWT→GNSS projection)
// ============================================================================
//
// The latest PPS_VCLOCK anchor is not always the correct anchor for a lower-
// priority OCXO/TimePop event.  If an event occurs just before a PPS boundary
// but is serviced after the priority-0 PPS ISR publishes the next anchor,
// projecting against the latest anchor turns "slightly before" into a huge
// unsigned DWT delta.  The ring preserves recent immutable anchors so the
// projector can choose the newest anchor that makes the event physically
// plausible.

struct pvc_anchor_record_t {
  uint32_t sequence = 0;
  uint32_t dwt_at_edge = 0;
  uint32_t counter32_at_edge = 0;
  int64_t  gnss_ns_at_edge = -1;
  uint32_t cps = 0;
  bool     cps_valid = false;
};

struct bridge_projection_t {
  int64_t  gnss_ns = -1;
  uint32_t anchor_sequence_used = 0;
  uint32_t anchor_age_slots = 0;
  uint32_t anchor_selection_kind = ANCHOR_SELECT_FAILED;
  uint32_t anchor_dwt_at_edge = 0;
  int64_t  anchor_gnss_ns_at_edge = -1;
  uint32_t anchor_cps = 0;
  uint64_t anchor_ns_delta = 0;
  uint32_t anchor_failure_mask = 0;
};

struct bridge_anchor_stats_t {
  uint32_t latest_count = 0;
  uint32_t previous_count = 0;
  uint32_t older_count = 0;
  uint32_t failed_count = 0;
  uint32_t last_selection_kind = ANCHOR_SELECT_NONE;
  uint32_t last_anchor_age_slots = 0;
  uint32_t last_anchor_sequence_used = 0;
  uint64_t last_anchor_ns_delta = 0;
  uint32_t last_anchor_failure_mask = 0;
};

static volatile uint32_t g_pvc_anchor_seq = 0;
static volatile uint32_t g_pvc_anchor_head = 0;
static volatile uint32_t g_pvc_anchor_count = 0;
static volatile bool     g_pvc_anchor_reset_pending = false;
static pvc_anchor_record_t g_pvc_anchor_ring[PVC_ANCHOR_RING_SIZE];

// Report-only accounting for CLOCKS/Alpha GNSS-label annotations.
// The label API does not create anchors or change event custody; these counters
// only make successful/missed annotations visible in INTERRUPT.REPORT_BRIDGE.
static uint32_t g_pvc_anchor_label_update_count = 0;
static uint32_t g_pvc_anchor_label_miss_count = 0;
static uint32_t g_pvc_anchor_label_invalid_count = 0;
static uint32_t g_pvc_anchor_label_last_sequence = 0;
static uint32_t g_pvc_anchor_label_last_counter32 = 0;
static int64_t  g_pvc_anchor_label_last_gnss_ns = -1;
static uint32_t g_pvc_anchor_label_last_cps = 0;
static uint32_t g_pvc_anchor_label_last_match_age_slots = 0xFFFFFFFFUL;
static uint32_t g_pvc_anchor_label_last_match_index = 0xFFFFFFFFUL;
static bool     g_pvc_anchor_label_last_success = false;

static bridge_anchor_stats_t g_bridge_stats_timepop = {};
static bridge_anchor_stats_t g_bridge_stats_ocxo1 = {};
static bridge_anchor_stats_t g_bridge_stats_ocxo2 = {};

static void pvc_anchor_ring_reset(void) {
  g_pvc_anchor_seq++;
  dmb_barrier();

  g_pvc_anchor_head = 0;
  g_pvc_anchor_count = 0;
  g_pvc_anchor_reset_pending = false;
  g_pvc_anchor_label_update_count = 0;
  g_pvc_anchor_label_miss_count = 0;
  g_pvc_anchor_label_invalid_count = 0;
  g_pvc_anchor_label_last_sequence = 0;
  g_pvc_anchor_label_last_counter32 = 0;
  g_pvc_anchor_label_last_gnss_ns = -1;
  g_pvc_anchor_label_last_cps = 0;
  g_pvc_anchor_label_last_match_age_slots = 0xFFFFFFFFUL;
  g_pvc_anchor_label_last_match_index = 0xFFFFFFFFUL;
  g_pvc_anchor_label_last_success = false;
  for (uint32_t i = 0; i < PVC_ANCHOR_RING_SIZE; i++) {
    g_pvc_anchor_ring[i] = pvc_anchor_record_t{};
  }

  dmb_barrier();
  g_pvc_anchor_seq++;
}

static void pvc_anchor_publish(const pps_vclock_t& pvc) {
  const uint32_t next = (g_pvc_anchor_count == 0)
      ? 0
      : ((g_pvc_anchor_head + 1u) % PVC_ANCHOR_RING_SIZE);

  g_pvc_anchor_seq++;
  dmb_barrier();

  g_pvc_anchor_ring[next].sequence = pvc.sequence;
  g_pvc_anchor_ring[next].dwt_at_edge = pvc.dwt_at_edge;
  g_pvc_anchor_ring[next].counter32_at_edge = pvc.counter32_at_edge;
  g_pvc_anchor_ring[next].gnss_ns_at_edge = pvc.gnss_ns_at_edge;

  // Store a reconstructive CPS with each anchor without consulting process_time.
  // The newest anchor uses the just-measured previous PPS_VCLOCK interval as
  // its forward projection denominator, matching the old random-walk policy.
  uint32_t cps = 0;
  bool cps_valid = false;
  if (g_pvc_anchor_count > 0) {
    const pvc_anchor_record_t& prev = g_pvc_anchor_ring[g_pvc_anchor_head];
    if (prev.sequence != 0) {
      cps = pvc.dwt_at_edge - prev.dwt_at_edge;
      cps_valid = (cps != 0);
    }
  }
  g_pvc_anchor_ring[next].cps = cps;
  g_pvc_anchor_ring[next].cps_valid = cps_valid;

  g_pvc_anchor_head = next;
  if (g_pvc_anchor_count < PVC_ANCHOR_RING_SIZE) {
    g_pvc_anchor_count++;
  }

  dmb_barrier();
  g_pvc_anchor_seq++;
}

static bool pvc_anchor_snapshot(pvc_anchor_record_t* out, uint32_t& out_count) {
  out_count = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_pvc_anchor_seq;
    dmb_barrier();

    const uint32_t count = g_pvc_anchor_count;
    const uint32_t head = g_pvc_anchor_head;
    const uint32_t bounded = (count > PVC_ANCHOR_RING_SIZE) ? PVC_ANCHOR_RING_SIZE : count;

    for (uint32_t age = 0; age < bounded; age++) {
      const uint32_t idx = (head + PVC_ANCHOR_RING_SIZE - age) % PVC_ANCHOR_RING_SIZE;
      out[age].sequence = g_pvc_anchor_ring[idx].sequence;
      out[age].dwt_at_edge = g_pvc_anchor_ring[idx].dwt_at_edge;
      out[age].counter32_at_edge = g_pvc_anchor_ring[idx].counter32_at_edge;
      out[age].gnss_ns_at_edge = g_pvc_anchor_ring[idx].gnss_ns_at_edge;
      out[age].cps = g_pvc_anchor_ring[idx].cps;
      out[age].cps_valid = g_pvc_anchor_ring[idx].cps_valid;
    }

    dmb_barrier();
    const uint32_t s2 = g_pvc_anchor_seq;
    if (s1 == s2 && (s1 & 1u) == 0u) {
      out_count = bounded;
      return true;
    }
  }

  return false;
}

// Label an already-authored PPS_VCLOCK anchor with CLOCKS/Alpha's campaign
// GNSS identity and PPS/GPIO-derived static DWT cycles-per-second ruler.
// This is deliberately a ring annotation only: process_interrupt still owns
// anchor selection and DWT-to-GNSS projection for subscriber events, but it
// does not invent campaign GNSS labels locally.
void interrupt_pps_vclock_label_anchor(uint32_t sequence,
                                       uint32_t counter32_at_edge,
                                       uint64_t gnss_ns_at_edge,
                                       uint32_t dwt_cycles_per_second) {
  if (sequence == 0 || dwt_cycles_per_second == 0) {
    g_pvc_anchor_label_invalid_count++;
    g_pvc_anchor_label_last_sequence = sequence;
    g_pvc_anchor_label_last_counter32 = counter32_at_edge;
    g_pvc_anchor_label_last_gnss_ns = (int64_t)gnss_ns_at_edge;
    g_pvc_anchor_label_last_cps = dwt_cycles_per_second;
    g_pvc_anchor_label_last_match_age_slots = 0xFFFFFFFFUL;
    g_pvc_anchor_label_last_match_index = 0xFFFFFFFFUL;
    g_pvc_anchor_label_last_success = false;
    return;
  }

  uint32_t primask = 0;
  __asm__ volatile ("mrs %0, primask" : "=r" (primask) :: "memory");
  __disable_irq();

  uint32_t match = PVC_ANCHOR_RING_SIZE;
  uint32_t match_age = 0xFFFFFFFFUL;
  const uint32_t count = g_pvc_anchor_count;
  const uint32_t bounded =
      (count > PVC_ANCHOR_RING_SIZE) ? PVC_ANCHOR_RING_SIZE : count;
  const uint32_t head = g_pvc_anchor_head;

  for (uint32_t age = 0; age < bounded; age++) {
    const uint32_t idx =
        (head + PVC_ANCHOR_RING_SIZE - age) % PVC_ANCHOR_RING_SIZE;
    const pvc_anchor_record_t& a = g_pvc_anchor_ring[idx];
    if (a.sequence == sequence && a.counter32_at_edge == counter32_at_edge) {
      match = idx;
      match_age = age;
      break;
    }
  }

  g_pvc_anchor_label_last_sequence = sequence;
  g_pvc_anchor_label_last_counter32 = counter32_at_edge;
  g_pvc_anchor_label_last_gnss_ns = (int64_t)gnss_ns_at_edge;
  g_pvc_anchor_label_last_cps = dwt_cycles_per_second;
  g_pvc_anchor_label_last_match_age_slots = match_age;
  g_pvc_anchor_label_last_match_index = match;

  if (match < PVC_ANCHOR_RING_SIZE) {
    g_pvc_anchor_seq++;
    dmb_barrier();

    g_pvc_anchor_ring[match].gnss_ns_at_edge = (int64_t)gnss_ns_at_edge;
    g_pvc_anchor_ring[match].cps = dwt_cycles_per_second;
    g_pvc_anchor_ring[match].cps_valid = true;

    dmb_barrier();
    g_pvc_anchor_seq++;

    g_pvc_anchor_label_update_count++;
    g_pvc_anchor_label_last_success = true;
  } else {
    g_pvc_anchor_label_miss_count++;
    g_pvc_anchor_label_last_success = false;
  }

  if ((primask & 1u) == 0) {
    __enable_irq();
  }
}

static void bridge_projection_copy_to_diag(interrupt_capture_diag_t& diag,
                                           const bridge_projection_t& proj) {
  diag.anchor_sequence_used = proj.anchor_sequence_used;
  diag.anchor_age_slots = proj.anchor_age_slots;
  diag.anchor_selection_kind = proj.anchor_selection_kind;
  diag.anchor_dwt_at_edge = proj.anchor_dwt_at_edge;
  diag.anchor_gnss_ns_at_edge = proj.anchor_gnss_ns_at_edge;
  diag.anchor_cps = proj.anchor_cps;
  diag.anchor_ns_delta = proj.anchor_ns_delta;
  diag.anchor_failure_mask = proj.anchor_failure_mask;
}

static bridge_anchor_stats_t* bridge_stats_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::TIMEPOP) return &g_bridge_stats_timepop;
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_bridge_stats_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_bridge_stats_ocxo2;
  return nullptr;
}

static void bridge_projection_record_stats(interrupt_subscriber_kind_t kind,
                                           const bridge_projection_t& proj) {
  bridge_anchor_stats_t* s = bridge_stats_for(kind);
  if (!s) return;

  if (proj.anchor_selection_kind == ANCHOR_SELECT_LATEST) {
    s->latest_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_PREVIOUS) {
    s->previous_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_OLDER) {
    s->older_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_FAILED) {
    s->failed_count++;
  }

  s->last_selection_kind = proj.anchor_selection_kind;
  s->last_anchor_age_slots = proj.anchor_age_slots;
  s->last_anchor_sequence_used = proj.anchor_sequence_used;
  s->last_anchor_ns_delta = proj.anchor_ns_delta;
  s->last_anchor_failure_mask = proj.anchor_failure_mask;
}

// ============================================================================
// EDGE — PPS GPIO heartbeat
// ============================================================================
//
// State owned by the EDGE command:
//   • g_pps_gpio_heartbeat — running tally of PPS GPIO edges plus the last
//     edge's PPS_VCLOCK DWT and gnss_ns.  Updated in the PPS GPIO ISR.
//   • g_gpio_irq_count   — total PPS GPIO ISR entries.
//   • g_gpio_miss_count  — incremented when a GPIO ISR ran but did not
//     identify a PPS edge to process.
//
// The snapshot store (g_store, declared elsewhere in this file) holds the
// last pps_t and pps_vclock_t.  cmd_edge reads it via store_load() and
// publishes both views alongside the heartbeat tally.
//
// EDGE is a heartbeat plus a snapshot of "the last edge from two perspectives."

struct pps_gpio_heartbeat_t {
  uint32_t edge_count   = 0;
  uint32_t last_dwt     = 0;     // pps_vclock.dwt_at_edge of most recent edge
  int64_t  last_gnss_ns = -1;    // GNSS labels are CLOCKS-owned; unavailable here.
};
static pps_gpio_heartbeat_t g_pps_gpio_heartbeat;

static uint32_t g_gpio_irq_count = 0;
static uint32_t g_gpio_miss_count = 0;

// PPS post-ISR drain.  The GPIO ISR still captures the physical PPS witness
// facts and the small three-counter custody window immediately, because those
// are the timing facts.  Publication of the epoch packet and diagnostic entry
// callback are deferred to a TimePop ASAP slot so the priority-0 PPS ISR stays
// as short as possible.
struct pps_post_isr_mailbox_t {
  volatile bool pending = false;
  volatile uint32_t arm_count = 0;
  volatile uint32_t drain_count = 0;
  volatile uint32_t overwrite_count = 0;
  volatile uint32_t asap_fail_count = 0;
  volatile uint32_t last_sequence = 0;
  volatile uint32_t last_capture_window_cycles = 0;
  interrupt_epoch_capture_t cap{};
  uint32_t isr_entry_dwt_raw = 0;
};

static pps_post_isr_mailbox_t g_pps_post_isr = {};
static void pps_post_isr_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*);

static volatile bool     g_pps_relay_timer_active = false;
static volatile bool     g_pps_relay_deassert_arm_pending = false;
static volatile uint32_t g_pps_relay_deassert_countdown_ticks = 0;
static volatile uint32_t g_pps_relay_assert_count = 0;
static volatile uint32_t g_pps_relay_deassert_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_fail_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_skip_count = 0;
static volatile uint32_t g_pps_relay_last_assert_sequence = 0;
static volatile bool     g_pps_relay_pin_initialized = false;

// Intrinsic CH2 relay deassertion.  This is the first deliberately tiny
// function moved out of VCLOCK_HEARTBEAT.  CH2 may fire for many TimePop
// reasons, so this counts elapsed VCLOCK ticks and decrements the relay
// countdown only for full 1 ms cells.
static volatile bool     g_pps_relay_ch2_tick_valid = false;
static volatile uint32_t g_pps_relay_ch2_last_counter32 = 0;
static volatile uint32_t g_pps_relay_ch2_tick_count = 0;
static volatile uint32_t g_pps_relay_ch2_catchup_count = 0;

static void pps_relay_assert_from_isr(uint32_t sequence);
static void pps_relay_ch2_tick(uint32_t vclock_counter32);

// ============================================================================
// PPS witness + VCLOCK-domain canonical epoch latch
// ============================================================================
//
// PPS GPIO is a witness/selector only.  It never authors the public
// PPS/VCLOCK DWT coordinate.  The super-canonical PPS/VCLOCK DWT coordinate is now authored from the
// TimePop CH2 fire fact of the VCLOCK cadence client.  Every public DWT timing
// fact therefore lives on the same TimePop/CH2 event-coordinate rail.
//
// During rebootstrap, the PPS ISR selects the VCLOCK counter identity of the
// sacred edge.  The next TimePop VCLOCK cadence event back-projects from its
// own shared CH2 fire fact to author the selected edge's DWT.  After bootstrap,
// every 1000th phase-locked TimePop cadence event authors the next canonical
// PPS/VCLOCK bookend directly.

struct vclock_epoch_latch_t {
  bool     pending = false;
  pps_t    pps{};

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge = 0;
  uint32_t observed_counter32 = 0;
  uint16_t observed_ch3 = 0;
  uint32_t observed_ticks_after_selected = 0;
  uint32_t first_cadence_counter32 = 0;
  uint32_t first_cadence_dwt = 0;
  uint32_t backdate_ticks = 0;
  uint32_t backdate_cycles = 0;
  uint32_t sacred_dwt = 0;
};

static pps_t g_last_pps_witness = {};
static bool  g_last_pps_witness_valid = false;
static vclock_epoch_latch_t g_vclock_epoch_latch = {};

// Intrinsic CH2 PPS/VCLOCK selected-epoch publication.
//
// The PPS GPIO ISR remains the selector of the sacred VCLOCK edge identity.
// Native CH2 now consumes that pending latch after TimePop has processed a CH2
// compare event, then back-projects from the CH2 event DWT to the selected
// PPS_VCLOCK edge.  VCLOCK_HEARTBEAT no longer authors bootstrap/rebootstrap
// publication; it only observes pending epoch work and steps aside.
static constexpr bool     VCLOCK_CH2_EPOCH_NATIVE_ENABLED = true;
static constexpr uint32_t VCLOCK_CH2_EPOCH_REJECT_NONE = 0;
static constexpr uint32_t VCLOCK_CH2_EPOCH_REJECT_BACKDATE_ZERO = 1;
static constexpr uint32_t VCLOCK_CH2_EPOCH_REJECT_BACKDATE_TOO_LARGE = 2;

static const char* vclock_ch2_epoch_reject_reason_name(uint32_t reason) {
  switch (reason) {
    case VCLOCK_CH2_EPOCH_REJECT_BACKDATE_ZERO: return "BACKDATE_ZERO";
    case VCLOCK_CH2_EPOCH_REJECT_BACKDATE_TOO_LARGE: return "BACKDATE_TOO_LARGE";
    default: return "NONE";
  }
}

static volatile uint32_t g_vclock_ch2_epoch_native_service_count = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_publish_count = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_bad_backdate_count = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_reject_reason = VCLOCK_CH2_EPOCH_REJECT_NONE;
static volatile uint32_t g_vclock_ch2_epoch_native_dispatch_arm_count = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_dispatch_arm_fail_count = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_selected_counter32 = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_service_counter32 = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_backdate_ticks = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_backdate_cycles = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_sacred_dwt = 0;
static volatile uint32_t g_vclock_ch2_epoch_native_last_sequence = 0;
static volatile uint32_t g_vclock_heartbeat_epoch_authority_retired_count = 0;
static volatile uint32_t g_vclock_heartbeat_epoch_pending_skip_count = 0;

struct dwt_repair_diag_t {
  bool     valid = false;
  bool     candidate = false;
  bool     synthetic = false;   // operational replacement applied; currently false
  uint32_t original_dwt = 0;
  uint32_t predicted_dwt = 0;
  uint32_t used_dwt = 0;
  int32_t  error_cycles = 0;
  const char* reason = "none";
};

struct vclock_repair_stats_t {
  uint32_t candidate_count = 0;
  uint32_t applied_count = 0;
  uint32_t consecutive_candidate_count = 0;
  uint32_t last_original_dwt = 0;
  uint32_t last_predicted_dwt = 0;
  uint32_t last_used_dwt = 0;
  int32_t  last_error_cycles = 0;
  uint32_t max_abs_error_cycles = 0;
  bool     last_candidate = false;
  bool     last_synthetic = false;

};

static vclock_repair_stats_t g_vclock_repair_stats = {};

// ============================================================================
// VCLOCK GNSS labels
// ============================================================================
//
// process_interrupt intentionally does not own campaign GNSS labels. It owns
// event custody: DWT coordinates, VCLOCK/OCXO counter identities, low-word
// phases, and phase diagnostics. CLOCKS/Alpha/Beta own the campaign GNSS
// timeline and publish pps_ns / pps_vclock_ns / vclock_ns in TIMEBASE_FRAGMENT.
//
// Returning -1 here is deliberate. A local VCLOCK-relative nanosecond value
// would be worse than unavailable because it could be mistaken for CLOCKS'
// campaign GNSS ledger.

static int64_t vclock_gnss_from_counter32(uint32_t) {
  return -1;
}

static uint32_t pvc_anchor_latest_cps(void) {
  if (g_pvc_anchor_count == 0) return 0;
  const pvc_anchor_record_t& a = g_pvc_anchor_ring[g_pvc_anchor_head];
  return (a.cps_valid && a.cps != 0) ? a.cps : 0;
}

static uint32_t interrupt_vclock_cycles_per_second(void) {
  const uint32_t calibrated = clocks_dwt_cycles_per_gnss_second();
  if (calibrated != 0) return calibrated;

  const uint32_t anchor_cps = pvc_anchor_latest_cps();
  if (anchor_cps != 0) return anchor_cps;

  return DWT_EXPECTED_PER_PPS;
}

static uint32_t vclock_cycles_for_ticks(uint32_t vclock_ticks) {
  const uint32_t cycles = interrupt_vclock_cycles_per_second();

  return (uint32_t)(((uint64_t)cycles * (uint64_t)vclock_ticks +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static uint32_t pps_vclock_phase_cycles_from_edges(const pps_t& pps,
                                                   const pps_vclock_t& pvc) {
  // Scalar PPS→VCLOCK phase, by definition less than one 10 MHz tick.
  //
  // pvc.dwt_at_edge may be any later VCLOCK edge authored on the TimePop rail.
  // Since all VCLOCK edges are separated by exactly one 10 MHz tick, the
  // phase from physical PPS to the selected first-after-PPS VCLOCK edge is the
  // DWT delta to any VCLOCK edge, modulo the DWT cycles in one 100 ns tick.
  //
  // Use scaled integer arithmetic rather than truncating cycles_per_tick:
  //   phase_scaled = (delta_dwt * 10,000,000) mod dwt_cycles_per_second
  //   phase_cycles = round(phase_scaled / 10,000,000)
  const uint32_t cycles = interrupt_vclock_cycles_per_second();
  const uint32_t delta_dwt = pvc.dwt_at_edge - pps.dwt_at_edge;
  const uint64_t phase_scaled =
      ((uint64_t)delta_dwt * (uint64_t)VCLOCK_COUNTS_PER_SECOND) %
      (uint64_t)cycles;

  return (uint32_t)((phase_scaled +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static dwt_repair_diag_t vclock_endpoint_repair_diagnostic(uint32_t observed_dwt) {
  dwt_repair_diag_t r{};
  r.valid = false;
  r.candidate = false;
  r.synthetic = false;
  r.original_dwt = observed_dwt;
  r.predicted_dwt = 0;
  r.used_dwt = observed_dwt;
  r.error_cycles = 0;
  r.reason = "disabled";

  g_vclock_repair_stats.last_candidate = false;
  g_vclock_repair_stats.last_synthetic = false;
  g_vclock_repair_stats.last_original_dwt = observed_dwt;
  g_vclock_repair_stats.last_predicted_dwt = 0;
  g_vclock_repair_stats.last_used_dwt = observed_dwt;
  g_vclock_repair_stats.last_error_cycles = 0;
  g_vclock_repair_stats.consecutive_candidate_count = 0;

  if (!VCLOCK_DWT_REPAIR_DIAG_ENABLED) {
    return r;
  }

  return r;
}

static void publish_vclock_domain_pps_vclock(const pps_t& pps,
                                             uint32_t sequence,
                                             uint32_t dwt_at_edge,
                                             uint32_t counter32_at_edge,
                                             uint16_t ch3_at_edge) {
  pps_vclock_t pvc;
  pvc.sequence = sequence;
  pvc.dwt_at_edge = dwt_at_edge;
  pvc.counter32_at_edge = counter32_at_edge;
  pvc.ch3_at_edge = ch3_at_edge;
  pvc.gnss_ns_at_edge = -1;  // GNSS labels are CLOCKS-owned.

  g_pps_gpio_heartbeat.last_dwt = pvc.dwt_at_edge;
  g_pps_gpio_heartbeat.last_gnss_ns = -1;

  if (g_pvc_anchor_reset_pending) {
    pvc_anchor_ring_reset();
  }

  store_publish(pps, pvc);
  pvc_anchor_publish(pvc);
}

static bool publish_selected_epoch_from_vclock_cadence(uint32_t cadence_counter32,
                                                       uint32_t cadence_event_dwt) {
  if (!g_vclock_epoch_latch.pending) return false;

  const uint32_t selected_counter32 = g_vclock_epoch_latch.counter32_at_edge;
  const uint32_t backdate_ticks = cadence_counter32 - selected_counter32;

  g_vclock_ch2_epoch_native_last_selected_counter32 = selected_counter32;
  g_vclock_ch2_epoch_native_last_service_counter32 = cadence_counter32;
  g_vclock_ch2_epoch_native_last_backdate_ticks = backdate_ticks;

  if (backdate_ticks == 0) {
    g_vclock_ch2_epoch_native_bad_backdate_count++;
    g_vclock_ch2_epoch_native_last_reject_reason =
        VCLOCK_CH2_EPOCH_REJECT_BACKDATE_ZERO;
    return false;
  }

  if (backdate_ticks > VCLOCK_COUNTS_PER_SECOND) {
    g_vclock_ch2_epoch_native_bad_backdate_count++;
    g_vclock_ch2_epoch_native_last_reject_reason =
        VCLOCK_CH2_EPOCH_REJECT_BACKDATE_TOO_LARGE;
    return false;
  }

  const uint32_t backdate_cycles = vclock_cycles_for_ticks(backdate_ticks);
  const uint32_t anchor_dwt = cadence_event_dwt - backdate_cycles;

  g_vclock_epoch_latch.first_cadence_counter32 = cadence_counter32;
  g_vclock_epoch_latch.first_cadence_dwt = cadence_event_dwt;
  g_vclock_epoch_latch.backdate_ticks = backdate_ticks;
  g_vclock_epoch_latch.backdate_cycles = backdate_cycles;
  g_vclock_epoch_latch.sacred_dwt = anchor_dwt;

  g_vclock_ch2_epoch_native_last_backdate_cycles = backdate_cycles;
  g_vclock_ch2_epoch_native_last_sacred_dwt = anchor_dwt;
  g_vclock_ch2_epoch_native_last_sequence = g_vclock_epoch_latch.pps.sequence;

  publish_vclock_domain_pps_vclock(g_vclock_epoch_latch.pps,
                                    g_vclock_epoch_latch.pps.sequence,
                                    anchor_dwt,
                                    selected_counter32,
                                    g_vclock_epoch_latch.ch3_at_edge);

  g_vclock_epoch_latch.pending = false;
  g_vclock_ch2_epoch_native_last_reject_reason = VCLOCK_CH2_EPOCH_REJECT_NONE;
  g_vclock_ch2_epoch_native_publish_count++;

  if (g_pps_edge_dispatch) {
    const timepop_handle_t h =
        timepop_arm_asap(pps_edge_dispatch_trampoline,
                         nullptr,
                         "PPS_VCLOCK_EPOCH_DISPATCH");
    if (h == TIMEPOP_INVALID_HANDLE) {
      g_vclock_ch2_epoch_native_dispatch_arm_fail_count++;
    } else {
      g_vclock_ch2_epoch_native_dispatch_arm_count++;
    }
  }

  return true;
}

// ============================================================================
// Latency adjusters — convert raw ISR-entry DWT to event coordinates
// ============================================================================
//
// These are the ONLY producers of event-coordinate DWT values, and they
// are the ONLY place in the codebase that applies hardware latency math.
// Called exactly once per ISR, on the first-instruction _raw capture.
// All downstream code consumes the returned value as event-coordinate
// truth and applies no further adjustment.

static inline uint32_t pps_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (GPIO_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

static inline uint32_t pps_vclock_dwt_from_pps_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw + (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
}

static inline uint32_t qtimer_event_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (QTIMER_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

// ============================================================================
// DWT → GNSS conversion (the OCXO/TimePop exception)
// ============================================================================

static bridge_projection_t interrupt_dwt_to_vclock_gnss_projection(uint32_t dwt_at_event) {
  bridge_projection_t out{};
  out.gnss_ns = -1;
  out.anchor_selection_kind = ANCHOR_SELECT_FAILED;

  pvc_anchor_record_t anchors[PVC_ANCHOR_RING_SIZE];
  uint32_t count = 0;
  if (!pvc_anchor_snapshot(anchors, count)) {
    out.anchor_failure_mask |= ANCHOR_FAIL_RING_UNSTABLE;
    return out;
  }

  if (count == 0) {
    out.anchor_failure_mask |= ANCHOR_FAIL_NO_ANCHORS;
    return out;
  }

  for (uint32_t age = 0; age < count; age++) {
    const pvc_anchor_record_t& a = anchors[age];

    if (a.sequence == 0 || a.gnss_ns_at_edge < 0) {
      out.anchor_failure_mask |= ANCHOR_FAIL_BAD_ANCHOR;
      continue;
    }

    if (!a.cps_valid || a.cps == 0) {
      out.anchor_failure_mask |= ANCHOR_FAIL_NO_CPS;
      continue;
    }
    const uint32_t cps = a.cps;

    const uint32_t dwt_delta = dwt_at_event - a.dwt_at_edge;
    const uint64_t ns_delta =
        ((uint64_t)dwt_delta * (uint64_t)GNSS_NS_PER_SECOND + (uint64_t)cps / 2ULL) /
        (uint64_t)cps;

    if (ns_delta > PVC_ANCHOR_MAX_EVENT_OFFSET_NS) {
      out.anchor_failure_mask |= ANCHOR_FAIL_DELTA_RANGE;
      continue;
    }

    out.gnss_ns = a.gnss_ns_at_edge + (int64_t)ns_delta;
    out.anchor_sequence_used = a.sequence;
    out.anchor_age_slots = age;
    out.anchor_selection_kind = (age == 0)
        ? ANCHOR_SELECT_LATEST
        : ((age == 1) ? ANCHOR_SELECT_PREVIOUS : ANCHOR_SELECT_OLDER);
    out.anchor_dwt_at_edge = a.dwt_at_edge;
    out.anchor_gnss_ns_at_edge = a.gnss_ns_at_edge;
    out.anchor_cps = cps;
    out.anchor_ns_delta = ns_delta;
    return out;
  }

  out.anchor_failure_mask |= ANCHOR_FAIL_NO_PLAUSIBLE;
  return out;
}

static int64_t interrupt_project_dwt_to_vclock_gnss_ns(uint32_t dwt_at_event) {
  return interrupt_dwt_to_vclock_gnss_projection(dwt_at_event).gnss_ns;
}

// ============================================================================
// Subscriber runtime
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};
  bool subscribed = false;
  bool active = false;
  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};
  bool has_fired = false;
  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;
};

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK", interrupt_provider_kind_t::QTIMER1, interrupt_lane_t::QTIMER1_CH2_COMP },
  { interrupt_subscriber_kind_t::OCXO1,  "OCXO1",  interrupt_provider_kind_t::QTIMER2, interrupt_lane_t::QTIMER2_CH0_COMP },
  { interrupt_subscriber_kind_t::OCXO2,  "OCXO2",  interrupt_provider_kind_t::QTIMER3, interrupt_lane_t::QTIMER3_CH3_COMP },
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;
static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;

static volatile uint32_t g_step0_qtimer1_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_qtimer2_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_qtimer3_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_gpio6789_priority_applied = 0xFFFFFFFFUL;

static volatile bool g_step0_qtimer1_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_qtimer2_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_qtimer3_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_gpio6789_configured_by_interrupt = false;


static volatile bool     g_pps_rebootstrap_pending = false;
static volatile uint32_t g_pps_rebootstrap_count   = 0;

static interrupt_subscriber_runtime_t* g_rt_vclock = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1  = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2  = nullptr;

static volatile interrupt_pps_entry_latency_handler_fn
    g_pps_entry_latency_handler = nullptr;



// ============================================================================
// Per-lane state
// ============================================================================

struct vclock_lane_t {
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};
static vclock_lane_t g_vclock_lane;

struct synthetic_clock32_t;
struct ocxo_runtime_context_t;

struct ocxo_lane_t {
  const char* name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;
  uint8_t      pcs = 0;
  int          input_pin = -1;
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;

  // Lane-local 1 kHz rollover cadence.  This is the device-centric substrate
  // that will later carry the regression window.  The compare target is a
  // process_interrupt-authored synthetic counter32 identity; the low 16 bits
  // are merely the hardware projection used to arm the local QTimer channel.
  bool     cadence_enabled = false;
  bool     cadence_armed = false;
  bool     cadence_epoch_valid = false;
  uint32_t cadence_epoch_counter32 = 0;

  // Configured quiet phase of this OCXO sample ladder inside the VCLOCK
  // millisecond cell.  This is a subscriber contract field: process_clocks may
  // back-project the clean observed sample to the logical one-second boundary.
  bool     cadence_sample_phase_valid = false;
  uint32_t cadence_sample_phase_ticks = 0;
  uint32_t cadence_sample_phase_us = 0;
  uint32_t cadence_sample_phase_ns = 0;
  uint32_t cadence_sample_period_ticks = 0;
  uint32_t cadence_phase_align_start_count = 0;
  uint32_t cadence_last_phase_align_vclock_counter32 = 0;
  uint32_t cadence_last_phase_align_vclock_phase_ticks = 0;
  uint32_t cadence_last_phase_align_ticks_until_target = 0;
  uint32_t cadence_last_phase_align_ocxo_counter32 = 0;

  uint32_t cadence_next_counter32 = 0;
  uint16_t cadence_next_low16 = 0;
  uint32_t cadence_last_target_counter32 = 0;
  uint16_t cadence_last_target_low16 = 0;
  uint16_t cadence_last_service_low16 = 0;
  uint32_t cadence_arm_count = 0;
  uint32_t cadence_rearm_count = 0;
  uint32_t cadence_fire_count = 0;
  uint32_t cadence_false_irq_count = 0;
  uint32_t cadence_one_second_due_count = 0;
  uint32_t cadence_last_fire_dwt = 0;
  uint32_t cadence_last_isr_entry_dwt_raw = 0;
  int32_t  cadence_last_service_offset_signed_ticks = 0;
  uint32_t cadence_last_service_offset_abs_ticks = 0;
  uint32_t cadence_last_interpreted_late_ticks = 0;
  uint32_t cadence_last_early_ticks = 0;
  bool     cadence_last_was_early = false;
  bool     cadence_last_one_second_due = false;
  uint32_t cadence_last_program_csctrl_before = 0;
  uint32_t cadence_last_program_csctrl_after = 0;
  bool     cadence_last_program_flag_before = false;
  bool     cadence_last_program_flag_after = false;
  bool     cadence_last_program_enabled_before = false;
  bool     cadence_last_program_enabled_after = false;
  uint32_t cadence_last_reason = OCXO_CADENCE_REASON_NONE;
  uint32_t cadence_user_callback_count = 0;

  // Once-per-second OCXO witness surface.  This is now derived from the 1000th
  // lane-local cadence sample rather than armed as a separate compare target.
  bool     witness_target_initialized = false;
  bool     witness_armed = false;
  uint32_t witness_target_counter32 = 0;
  uint16_t witness_target_low16 = 0;
  uint32_t witness_arm_count = 0;
  uint32_t witness_fire_count = 0;
  uint32_t witness_false_irq_count = 0;
  uint32_t witness_missed_target_count = 0;
  uint32_t witness_late_arm_count = 0;
  uint32_t witness_last_arm_remaining_ticks = 0;
  uint32_t witness_last_event_dwt = 0;
  uint32_t witness_last_event_counter32 = 0;

  // Legacy ambiguous surface retained for continuity.  This is the raw
  // 16-bit modular delta: isr_counter_low16 - target_low16 (mod 65536).
  // Values > 32767 mean early service, not very-late service.
  uint32_t witness_last_late_ticks = 0;

  // VCLOCK_HEARTBEAT arm-decision diagnostics.
  uint32_t witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_NONE;
  uint32_t witness_schedule_last_current_counter32 = 0;
  uint32_t witness_schedule_last_target_counter32 = 0;
  uint32_t witness_schedule_last_remaining_ticks = 0;
  uint32_t witness_schedule_last_phase_ticks = 0;
  uint32_t witness_schedule_last_ticks_until_arm_window = 0;
  uint16_t witness_schedule_last_current_low16 = 0;
  uint16_t witness_schedule_last_target_low16 = 0;

  // Arming snapshots.
  uint32_t witness_last_arm_dwt_raw = 0;
  uint32_t witness_last_arm_counter32 = 0;
  uint16_t witness_last_arm_low16 = 0;
  uint32_t witness_last_arm_target_counter32 = 0;
  uint16_t witness_last_arm_target_low16 = 0;
  uint32_t witness_last_arm_to_isr_ticks = 0;
  uint32_t witness_last_arm_to_isr_dwt_cycles = 0;
  uint32_t witness_last_program_csctrl_before = 0;
  uint32_t witness_last_program_csctrl_after = 0;
  bool     witness_last_program_flag_before = false;
  bool     witness_last_program_flag_after = false;
  bool     witness_last_program_enabled_before = false;
  bool     witness_last_program_enabled_after = false;

  // Compare-service diagnostics.  These are report-only and do not change
  // publish/compare behavior.
  uint32_t witness_last_isr_csctrl_entry = 0;
  uint32_t witness_last_isr_csctrl_after_disable = 0;
  bool     witness_last_isr_compare_flag_entry = false;
  bool     witness_last_isr_compare_enabled_entry = false;
  bool     witness_last_isr_compare_flag_after_disable = false;
  bool     witness_last_isr_compare_enabled_after_disable = false;
  bool     witness_last_irq_had_armed = false;
  bool     witness_last_irq_had_active_rt = false;

  uint16_t witness_last_target_low16 = 0;
  uint16_t witness_last_isr_counter_low16 = 0;
  uint32_t witness_last_target_delta_mod65536_ticks = 0;
  uint32_t witness_last_interpreted_late_ticks = 0;
  uint32_t witness_last_early_ticks = 0;
  int32_t  witness_last_service_offset_signed_ticks = 0;
  uint32_t witness_last_service_offset_abs_ticks = 0;
  bool     witness_last_service_was_early = false;
  bool     witness_last_service_was_on_or_after_target = false;
  uint32_t witness_last_service_class = OCXO_SERVICE_CLASS_NONE;

  // Deferred perishable-fact diagnostics.  The ISR captures these facts, but
  // foreground ASAP interprets them and updates this report surface.
  uint32_t witness_last_fact_sequence = 0;
  int32_t  witness_last_service_correction_cycles = 0;
  uint32_t witness_last_service_corrected_dwt = 0;
  bool     witness_last_fact_one_second_due = false;
  uint32_t witness_fact_enqueue_count = 0;
  uint32_t witness_fact_drain_count = 0;
  uint32_t witness_fact_overflow_count = 0;
  uint32_t witness_fact_high_water = 0;
  uint32_t witness_fact_asap_arm_count = 0;
  uint32_t witness_fact_asap_fail_count = 0;

  // Canonical one-second custody audit.  A normal OCXO event stream advances
  // exactly 10,000,000 ticks per published edge.
  bool     witness_previous_event_counter32_valid = false;
  uint32_t witness_previous_event_counter32 = 0;
  uint32_t witness_last_counter_delta_ticks = 0;
  uint32_t witness_counter_delta_violation_count = 0;
  uint32_t witness_last_bad_counter_delta = 0;

  uint32_t witness_service_count = 0;
  uint32_t witness_early_service_count = 0;
  uint32_t witness_on_or_after_service_count = 0;
  uint32_t witness_valid_publish_count = 0;
  uint32_t witness_early_service_published_count = 0;
  bool     witness_last_event_published = false;

  // Rollover-only EMA DWT authority.  The one-second event delivered to Alpha
  // uses ema_last_emitted_dwt, not the raw ISR-service DWT.  The observed ISR
  // endpoint is retained only as a diagnostic and as the interval sample that
  // updates the EMA predictor.
  bool     ema_initialized = false;
  bool     ema_interval_valid = false;
  uint32_t ema_last_observed_dwt = 0;
  uint32_t ema_last_emitted_dwt = 0;
  uint32_t ema_last_observed_interval_cycles = 0;
  uint32_t ema_interval_cycles = 0;
  uint32_t ema_last_predicted_dwt = 0;
  int32_t  ema_last_error_cycles = 0;
  uint32_t ema_max_abs_error_cycles = 0;
  uint32_t ema_update_count = 0;
};
static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind);
static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane);
static void ocxo_lane_program_compare(ocxo_lane_t& lane, uint16_t target_low16);
static void ocxo_lane_disable_compare(ocxo_lane_t& lane);
static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane);
static inline bool ocxo_lane_compare_flag_pending(const ocxo_lane_t& lane);
static void ocxo_qtimer_diag_record(ocxo_runtime_context_t& ctx,
                                    uint32_t isr_entry_dwt_raw,
                                    uint32_t event_dwt,
                                    uint32_t legacy_late_ticks,
                                    uint32_t interpreted_late_ticks,
                                    uint32_t early_ticks,
                                    int32_t service_offset_signed_ticks,
                                    uint32_t service_offset_abs_ticks,
                                    uint32_t target_delta_mod65536_ticks,
                                    uint16_t target_low16,
                                    uint16_t isr_counter_low16,
                                    bool ema_predicted);
static bool ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t kind,
                                          ocxo_lane_t& lane,
                                          synthetic_clock32_t& clock32,
                                          uint32_t reason);
static bool ocxo_lane_start_local_cadence_at_target(interrupt_subscriber_kind_t kind,
                                                    ocxo_lane_t& lane,
                                                    synthetic_clock32_t& clock32,
                                                    uint32_t target_counter32,
                                                    uint32_t reason);
static uint32_t ocxo_one_second_next_target_after(
    uint32_t epoch_counter32,
    uint32_t current_counter32);
static void ocxo_lane_program_local_cadence_compare(
    ocxo_lane_t& lane,
    uint32_t target_counter32,
    uint32_t reason);
static void ocxo_lane_stop_local_cadence(ocxo_lane_t& lane, uint32_t reason);
static void ocxo_lane_ema_reset(ocxo_lane_t& lane);
static void ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t kind,
                                           ocxo_lane_t& lane,
                                           synthetic_clock32_t& clock32,
                                           uint32_t epoch_counter32,
                                           uint32_t reason);
static void ocxo_lane_maybe_arm_one_second_compare(
    interrupt_subscriber_kind_t kind,
    ocxo_lane_t& lane,
    synthetic_clock32_t& clock32,
    uint32_t reason);

// ============================================================================
// QTimer1 CH0 low-word counter
// ============================================================================
//
// CH0 is the only VCLOCK hardware counter used by process_interrupt now.
// CH1 is deliberately not cascaded; it remains free for a future dedicated
// VCLOCK-domain compare rail.  Any public 32-bit VCLOCK identity is synthetic
// and authored below.

static inline uint16_t qtimer1_ch0_counter_now(void) {
  return IMXRT_TMR1.CH[0].CNTR;
}

// ============================================================================
// Private synthetic 32-bit clock identities
// ============================================================================
//
// Each QTimer-derived clock domain has a private process_interrupt-owned
// synthetic 32-bit identity.  The public contract is simple: subscribers get
// counter32_at_event only in the interrupt event payload.  They do not read
// timer hardware, and they do not author counter identities themselves.
//
// For VCLOCK, the synthetic identity is mapped to QTimer1 CH0's 16-bit
// hardware counter by a low-word anchor established at an exact edge.  CH1 is
// no longer cascaded.  TimePop schedules by synthetic deadlines while
// process_interrupt alone translates those deadlines into low-word hardware
// compare values.  For OCXO lanes, the synthetic identity is advanced by the
// exact 10 MHz tick interval of the process_interrupt-owned cadence.

struct synthetic_clock32_t {
  bool     zeroed = false;
  uint64_t zero_ns = 0;
  uint32_t zero_counter32 = 0;
  uint32_t current_counter32 = 0;
  uint64_t current_ns = 0;
  uint16_t hardware16 = 0;
  uint32_t zero_count = 0;
  uint32_t minder_update_count = 0;

  bool     pending_zero = false;
  uint64_t pending_zero_ns = 0;
  uint32_t pending_zero_counter32 = 0;
  uint32_t pending_zero_count = 0;
};

struct vclock_synthetic_clock32_t : synthetic_clock32_t {
  uint16_t hardware_low16_at_zero = 0;
  uint16_t hardware_low16_at_current = 0;
  bool     hardware_anchor_valid = false;
  uint32_t hardware_anchor_update_count = 0;
};

static vclock_synthetic_clock32_t g_vclock_clock32;
static synthetic_clock32_t        g_ocxo1_clock32;
static synthetic_clock32_t        g_ocxo2_clock32;

static volatile uint32_t g_qtimer1_ch1_sequence = 0;
static volatile uint32_t g_qtimer1_ch1_target_counter32 = 0;
static volatile uint32_t g_qtimer1_ch1_next_compare_counter32 = 0;
static volatile uint32_t g_qtimer1_ch1_arm_count = 0;
static volatile uint32_t g_qtimer1_ch1_fire_count = 0;
static volatile uint32_t g_qtimer1_ch1_hop_count = 0;
static volatile bool     g_qtimer1_ch1_active = false;
static volatile interrupt_qtimer1_ch1_handler_fn g_qtimer1_ch1_handler = nullptr;

static volatile uint32_t g_qtimer1_ch2_last_target_counter32 = 0;
static volatile uint32_t g_qtimer1_ch2_arm_count = 0;

struct ocxo_qtimer_diag_t {
  volatile uint32_t count = 0;
  volatile uint32_t late_ticks = 0;  // legacy raw modular delta
  volatile uint32_t interpreted_late_ticks = 0;
  volatile uint32_t early_ticks = 0;
  volatile int32_t  service_offset_signed_ticks = 0;
  volatile uint32_t service_offset_abs_ticks = 0;
  volatile uint32_t target_delta_mod65536_ticks = 0;
  volatile uint16_t target_low16 = 0;
  volatile uint16_t isr_counter_low16 = 0;
  volatile uint32_t dwt_raw = 0;
  volatile uint32_t event_dwt = 0;
  volatile uint32_t dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;
};

static ocxo_qtimer_diag_t g_ocxo1_qtimer_diag = {};
static ocxo_qtimer_diag_t g_ocxo2_qtimer_diag = {};

struct ocxo_runtime_context_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane_id = interrupt_lane_t::NONE;
  const char* name = nullptr;
  const char* prefix = nullptr;
  const char* cadence_source = nullptr;
  const char* counter_source = nullptr;
  const char* dwt_authority = nullptr;
  const char* fact_drain_name = nullptr;
  ocxo_lane_t* lane = nullptr;
  synthetic_clock32_t* clock32 = nullptr;
  interrupt_subscriber_runtime_t** rt_slot = nullptr;
  ocxo_qtimer_diag_t* qtimer_diag = nullptr;
};

static ocxo_runtime_context_t g_ocxo1_ctx = {
  interrupt_subscriber_kind_t::OCXO1,
  interrupt_provider_kind_t::QTIMER2,
  interrupt_lane_t::QTIMER2_CH0_COMP,
  "OCXO1",
  "ocxo1",
  "QTIMER2_CH0_ONE_SECOND_EDGE_COMPARE",
  "QTIMER2_CH0_LOCAL_SYNTHETIC_COUNTER32",
  "QTIMER2_CH0_ISR_ENTRY_DWT",
  "OCXO1_FACT_DRAIN",
  &g_ocxo1_lane,
  &g_ocxo1_clock32,
  &g_rt_ocxo1,
  &g_ocxo1_qtimer_diag,
};

static ocxo_runtime_context_t g_ocxo2_ctx = {
  interrupt_subscriber_kind_t::OCXO2,
  interrupt_provider_kind_t::QTIMER3,
  interrupt_lane_t::QTIMER3_CH3_COMP,
  "OCXO2",
  "ocxo2",
  "QTIMER3_CH3_ONE_SECOND_EDGE_COMPARE",
  "QTIMER3_CH3_LOCAL_SYNTHETIC_COUNTER32",
  "QTIMER3_CH3_ISR_ENTRY_DWT",
  "OCXO2_FACT_DRAIN",
  &g_ocxo2_lane,
  &g_ocxo2_clock32,
  &g_rt_ocxo2,
  &g_ocxo2_qtimer_diag,
};

static ocxo_runtime_context_t* ocxo_context_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_ctx;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_ctx;
  return nullptr;
}

static bool ocxo_kind_disabled(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return OCXO1_DISABLED;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return OCXO2_DISABLED;
  return false;
}

static bool ocxo_context_disabled(const ocxo_runtime_context_t& ctx) {
  return ocxo_kind_disabled(ctx.kind);
}

static interrupt_subscriber_runtime_t* ocxo_runtime_for(const ocxo_runtime_context_t& ctx) {
  return ctx.rt_slot ? *ctx.rt_slot : nullptr;
}

static volatile bool     g_vclock_heartbeat_armed = false;
static volatile uint32_t g_vclock_heartbeat_arm_count = 0;
static volatile uint32_t g_vclock_heartbeat_arm_failures = 0;
static volatile uint32_t g_vclock_heartbeat_fire_count = 0;
static volatile uint32_t g_vclock_heartbeat_vclock_ticks = 0;
static volatile uint32_t g_vclock_heartbeat_ocxo1_rollover_updates_retired = 0;
static volatile uint32_t g_vclock_heartbeat_ocxo2_rollover_updates_retired = 0;
static volatile uint16_t g_vclock_heartbeat_last_vclock_hw16 = 0;
static volatile uint16_t g_vclock_heartbeat_last_ocxo1_hw16_retired = 0;
static volatile uint16_t g_vclock_heartbeat_last_ocxo2_hw16_retired = 0;
static volatile uint32_t g_vclock_heartbeat_last_vclock_counter32 = 0;
static volatile uint32_t g_vclock_heartbeat_last_ocxo1_counter32_retired = 0;
static volatile uint32_t g_vclock_heartbeat_last_ocxo2_counter32_retired = 0;

// Passive CH2 rollover tend — surgical coexistence layer.
//
// This is deliberately not a scheduled service and not an event source.  It
// runs from the QTimer1 CH2 ISR path and merely refreshes process_interrupt's
// synthetic 32-bit ambient projections from current 16-bit hardware reads.
// Existing VCLOCK_HEARTBEAT and OCXO local cadence event authorship remain intact.
static constexpr bool     CH2_IMPLICIT_ROLLOVER_ENABLED = true;
static volatile uint32_t g_ch2_implicit_rollover_count = 0;
static volatile uint32_t g_ch2_implicit_rollover_vclock_updates = 0;
static volatile uint32_t g_ch2_implicit_rollover_ocxo1_updates = 0;
static volatile uint32_t g_ch2_implicit_rollover_ocxo2_updates = 0;
static volatile uint16_t g_ch2_implicit_rollover_last_vclock_hw16 = 0;
static volatile uint16_t g_ch2_implicit_rollover_last_ocxo1_hw16 = 0;
static volatile uint16_t g_ch2_implicit_rollover_last_ocxo2_hw16 = 0;
static volatile uint32_t g_ch2_implicit_rollover_last_vclock_counter32 = 0;
static volatile uint32_t g_ch2_implicit_rollover_last_ocxo1_counter32 = 0;
static volatile uint32_t g_ch2_implicit_rollover_last_ocxo2_counter32 = 0;

// Intrinsic CH2 VCLOCK one-second handoff.
//
// CH2 owns VCLOCK one-second fact authorship, but the first post-epoch target
// is deliberately seeded on the legacy heartbeat-compatible grid.  The prior
// attempt to seed the first target directly at selected_epoch + 10,000,000
// changed the first public bookend phase and destabilized the command path.
// This version migrates first-bookend publication to CH2 while preserving the
// old first-bookend identity; after that CH2 advances by exact +10,000,000
// gear teeth as before.
static volatile bool     g_vclock_ch2_one_second_enabled = false;
static volatile uint32_t g_vclock_ch2_one_second_next_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_enable_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_reset_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_service_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_enqueue_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_late_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_late_max_ticks = 0;
static volatile uint32_t g_vclock_ch2_one_second_catchup_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_drop_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_last_target_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_last_service_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_last_dwt = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_enable_count = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_base_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_service_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_target_counter32 = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_tick_mod_seed = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_remaining_cells = 0;
static volatile uint32_t g_vclock_ch2_one_second_epoch_residual_ticks = 0;
static volatile uint32_t g_vclock_ch2_one_second_heartbeat_enable_count = 0;
static volatile uint32_t g_vclock_heartbeat_one_second_legacy_count = 0;
static volatile uint32_t g_vclock_heartbeat_one_second_handoff_skip_count = 0;
static volatile uint32_t g_vclock_heartbeat_one_second_fallback_count = 0;

// Intrinsic CH2 VCLOCK SmartZero sampling.
//
// SmartZero needs adjacent +10,000-tick VCLOCK samples. CH2 fires for the
// TimePop rail, not exclusively for a 1 kHz heartbeat, so this surface is a
// fixed-grid sampler rather than an every-CH2-fire sampler. The first CH2
// event while SmartZero owns VCLOCK seeds the grid; subsequent samples are
// authored at exact +10,000 counter identities and DWT is back-projected if
// the CH2 service event is slightly late.
static constexpr bool     VCLOCK_CH2_SMARTZERO_NATIVE_ENABLED = true;
static volatile bool     g_vclock_ch2_smartzero_seeded = false;
static volatile uint32_t g_vclock_ch2_smartzero_next_counter32 = 0;
static volatile uint32_t g_vclock_ch2_smartzero_reset_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_seed_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_deactivate_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_service_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_transaction_skip_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_feed_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_late_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_late_max_ticks = 0;
static volatile uint32_t g_vclock_ch2_smartzero_resync_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_drop_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_waiting_for_cps_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_first_sample_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_accepted_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_rejected_dwt_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_rejected_counter_count = 0;
static volatile uint32_t g_vclock_ch2_smartzero_last_target_counter32 = 0;
static volatile uint32_t g_vclock_ch2_smartzero_last_service_counter32 = 0;
static volatile uint32_t g_vclock_ch2_smartzero_last_dwt = 0;
static volatile uint32_t g_vclock_ch2_smartzero_last_late_ticks = 0;
static volatile uint32_t g_vclock_heartbeat_smartzero_authority_retired_count = 0;

// Intrinsic CH2 VCLOCK fact-drain arming.
//
// CH2 now authors steady-state VCLOCK one-second facts before TimePop handles
// the same compare event.  The perishable fact still drains through the safe
// TimePop ASAP path, but the arm decision no longer waits for VCLOCK_HEARTBEAT
// to be dispatched.  To preserve TimePop ordering, CH2 arms the drain only
// after the registered TimePop CH2 handler has returned.
static volatile uint32_t g_vclock_ch2_fact_drain_service_count = 0;
static volatile uint32_t g_vclock_ch2_fact_drain_arm_count = 0;
static volatile uint32_t g_vclock_heartbeat_fact_drain_authority_retired_count = 0;

static inline uint32_t clock32_from_ns(uint64_t ns) {
  return (uint32_t)((ns / 100ULL) & 0xFFFFFFFFULL);
}

uint32_t interrupt_clock32_from_ns(uint64_t ns) {
  return clock32_from_ns(ns);
}

static synthetic_clock32_t* synthetic_clock_for_kind(interrupt_subscriber_kind_t kind) {
  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  return ctx ? ctx->clock32 : nullptr;
}

static void synthetic_clock_zero(synthetic_clock32_t& c, uint64_t ns) {
  c.zeroed = true;
  c.zero_ns = ns;
  c.zero_counter32 = clock32_from_ns(ns);
  c.current_counter32 = c.zero_counter32;
  c.current_ns = ns;
  c.hardware16 = (uint16_t)c.current_counter32;
  c.zero_count++;
  c.pending_zero = false;
}

static void vclock_clock_anchor_hardware_low16(uint32_t synthetic_counter32,
                                                 uint16_t hardware_low16) {
  g_vclock_clock32.current_counter32 = synthetic_counter32;
  g_vclock_clock32.current_ns = (uint64_t)synthetic_counter32 * 100ULL;
  g_vclock_clock32.hardware16 = hardware_low16;
  g_vclock_clock32.hardware_low16_at_current = hardware_low16;
  g_vclock_clock32.hardware_anchor_valid = true;
  g_vclock_clock32.hardware_anchor_update_count++;
}

static void vclock_clock_zero_at_hardware_low16(uint64_t ns,
                                                uint16_t hardware_low16) {
  g_vclock_clock32.zeroed = true;
  g_vclock_clock32.zero_ns = ns;
  g_vclock_clock32.zero_counter32 = clock32_from_ns(ns);
  g_vclock_clock32.hardware_low16_at_zero = hardware_low16;
  g_vclock_clock32.zero_count++;
  g_vclock_clock32.pending_zero = false;
  vclock_clock_anchor_hardware_low16(g_vclock_clock32.zero_counter32,
                                     hardware_low16);
}

static void synthetic_clock_bootstrap_from_hw16(synthetic_clock32_t& c, uint16_t hardware16);
static void vclock_clock_bootstrap_from_hw16(uint16_t hardware16);
static uint32_t project_counter32_from_hw16(const synthetic_clock32_t& c, uint16_t hardware16);
static inline void synthetic_clock_consume_pending_zero_if_any(synthetic_clock32_t& c);
static void cadence_regression_reset_kind(interrupt_subscriber_kind_t kind);
static void vclock_fact_ring_reset(void);
static void vclock_ch2_one_second_disable(void);
static void vclock_ch2_one_second_enable_after_heartbeat(uint32_t last_counter32);
static void vclock_ch2_one_second_enable_from_epoch_legacy_grid(
    uint32_t selected_counter32,
    uint32_t service_counter32,
    uint32_t tick_mod_seed);
static bool vclock_ch2_one_second_already_served_for(uint32_t counter32);
static void vclock_ch2_one_second_service(uint32_t qtimer_event_dwt,
                                          uint32_t service_counter32);
static void vclock_ch2_smartzero_reset_attempt_state(void);
static void vclock_ch2_smartzero_deactivate(void);
static void vclock_ch2_smartzero_service(uint32_t qtimer_event_dwt,
                                         uint32_t service_counter32);

static uint32_t vclock_synthetic_from_hardware_low16(uint16_t hardware_low16) {
  if (!g_vclock_clock32.zeroed) return (uint32_t)hardware_low16;

  if (g_vclock_clock32.hardware_anchor_valid) {
    return g_vclock_clock32.current_counter32 +
           (uint32_t)((uint16_t)(hardware_low16 -
                                g_vclock_clock32.hardware_low16_at_current));
  }

  return g_vclock_clock32.zero_counter32 +
         (uint32_t)((uint16_t)(hardware_low16 -
                              g_vclock_clock32.hardware_low16_at_zero));
}

static uint16_t vclock_hardware_low16_from_synthetic(uint32_t synthetic_counter32) {
  if (!g_vclock_clock32.zeroed) return (uint16_t)(synthetic_counter32 & 0xFFFFU);

  return (uint16_t)(g_vclock_clock32.hardware_low16_at_zero +
                   (uint16_t)(synthetic_counter32 -
                              g_vclock_clock32.zero_counter32));
}

bool interrupt_clock32_zero_from_ns(interrupt_subscriber_kind_t kind, uint64_t ns) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    vclock_clock_zero_at_hardware_low16(ns, qtimer1_ch0_counter_now());
    g_vclock_lane.logical_count32_at_last_second = g_vclock_clock32.current_counter32;
    vclock_ch2_one_second_disable();
    cadence_regression_reset_kind(interrupt_subscriber_kind_t::VCLOCK);
    vclock_fact_ring_reset();
    return true;
  }

  synthetic_clock32_t* c = synthetic_clock_for_kind(kind);
  if (!c) return false;
  synthetic_clock_zero(*c, ns);

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (lane) {
    lane->logical_count32_at_last_second = c->current_counter32;
    lane->tick_mod_1000 = 0;
    ocxo_lane_install_logical_grid(kind, *lane, *c, c->zero_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_ZERO);
  }
  return true;
}

bool interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t kind, uint64_t ns) {
  const uint32_t counter32 = clock32_from_ns(ns);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_clock32.pending_zero = true;
    g_vclock_clock32.pending_zero_ns = ns;
    g_vclock_clock32.pending_zero_counter32 = counter32;
    g_vclock_clock32.pending_zero_count++;
    vclock_ch2_one_second_disable();
    cadence_regression_reset_kind(interrupt_subscriber_kind_t::VCLOCK);
    vclock_fact_ring_reset();
    return true;
  }

  synthetic_clock32_t* c = synthetic_clock_for_kind(kind);
  if (!c) return false;
  c->pending_zero = true;
  c->pending_zero_ns = ns;
  c->pending_zero_counter32 = counter32;
  c->pending_zero_count++;
  return true;
}

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32) {
  if (!OCXO1_DISABLED && g_ocxo1_lane.initialized) {
    ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t::OCXO1,
                                   g_ocxo1_lane,
                                   g_ocxo1_clock32,
                                   ocxo1_epoch_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_GRID);
  }

  if (!OCXO2_DISABLED && g_ocxo2_lane.initialized) {
    ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t::OCXO2,
                                   g_ocxo2_lane,
                                   g_ocxo2_clock32,
                                   ocxo2_epoch_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_GRID);
  }
}

static inline void synthetic_clock_consume_pending_zero_if_any(synthetic_clock32_t& c) {
  if (!c.pending_zero) return;
  synthetic_clock_zero(c, c.pending_zero_ns);
}

static inline uint32_t synthetic_clock_advance_at_hardware(synthetic_clock32_t& c,
                                                           uint32_t ticks,
                                                           uint16_t hardware16_at_event) {
  synthetic_clock_consume_pending_zero_if_any(c);
  c.current_counter32 += ticks;
  c.current_ns += (uint64_t)ticks * 100ULL;
  c.hardware16 = hardware16_at_event;
  return c.current_counter32;
}

uint32_t interrupt_qtimer1_counter32_now(void)   { return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now()); }
uint16_t interrupt_qtimer2_ch0_counter_now(void) {
  return OCXO1_DISABLED ? 0U : IMXRT_TMR2.CH[0].CNTR;
}
uint16_t interrupt_qtimer3_ch3_counter_now(void) {
  return OCXO2_DISABLED ? 0U : IMXRT_TMR3.CH[3].CNTR;
}

// Forward declarations for compare helpers defined below.
static void qtimer2_ch0_program_compare(uint16_t target_low16);
static void qtimer3_program_compare(uint8_t ch, uint16_t target_low16);

static inline uint32_t cadence_mod_for_ticks(uint32_t ticks, uint32_t interval) {
  if (interval == 0) return 0;
  return (ticks / interval) % TICKS_PER_SECOND_EVENT;
}

static uint32_t project_counter32_from_hw16(const synthetic_clock32_t& c,
                                            uint16_t hardware16) {
  // Boot-safety rule: before a lane has been explicitly born/zeroed, raw
  // hardware low-word observations are still safe diagnostic coordinates.
  // Do not project from an uninitialized synthetic anchor.  This preserves
  // the old pre-epoch behavior and keeps early PPS/REPORT paths harmless.
  if (!c.zeroed) return (uint32_t)hardware16;

  return c.current_counter32 + (uint32_t)((uint16_t)(hardware16 - c.hardware16));
}

static uint64_t project_ns64_from_hw16(const synthetic_clock32_t& c,
                                       uint16_t hardware16) {
  // Same boot-safety rule as COUNT32 projection.  This value is not global
  // truth before ZERO; it is only a harmless local low-word-derived surface.
  if (!c.zeroed) return (uint64_t)hardware16 * 100ULL;

  const uint32_t delta = (uint32_t)((uint16_t)(hardware16 - c.hardware16));
  return c.current_ns + (uint64_t)delta * 100ULL;
}

static void synthetic_clock_bootstrap_from_hw16(synthetic_clock32_t& c,
                                                uint16_t hardware16) {
  // This is not a user ZERO.  It merely gives the synthetic extender a
  // coherent birth anchor so early ISR/report paths never project from an
  // all-zero default object.
  c.zeroed = true;
  c.zero_ns = (uint64_t)hardware16 * 100ULL;
  c.zero_counter32 = (uint32_t)hardware16;
  c.current_counter32 = (uint32_t)hardware16;
  c.current_ns = (uint64_t)hardware16 * 100ULL;
  c.hardware16 = hardware16;
  c.pending_zero = false;
}

static void vclock_clock_bootstrap_from_hw16(uint16_t hardware16) {
  // Birth the VCLOCK synthetic layer without declaring a logical ZERO.  The
  // PPS/VCLOCK rebootstrap and CLOCKS.ZERO paths may later install the real
  // coordinate origin, but the interrupt layer is safe immediately after IRQs
  // go live.
  g_vclock_clock32.zeroed = true;
  g_vclock_clock32.zero_ns = (uint64_t)hardware16 * 100ULL;
  g_vclock_clock32.zero_counter32 = (uint32_t)hardware16;
  g_vclock_clock32.hardware_low16_at_zero = hardware16;
  g_vclock_clock32.pending_zero = false;
  vclock_clock_anchor_hardware_low16((uint32_t)hardware16, hardware16);
  g_vclock_lane.logical_count32_at_last_second = (uint32_t)hardware16;
}

static void vclock_clock_tend_from_hardware_low16(uint16_t hardware_low16) {
  if (!g_vclock_clock32.zeroed) {
    vclock_clock_bootstrap_from_hw16(hardware_low16);
    return;
  }

  const uint32_t counter32 = vclock_synthetic_from_hardware_low16(hardware_low16);
  vclock_clock_anchor_hardware_low16(counter32, hardware_low16);
  g_vclock_clock32.minder_update_count++;
}

static void synthetic_clock_tend_from_hw16(synthetic_clock32_t& c,
                                           uint16_t hardware16) {
  if (!c.zeroed) {
    synthetic_clock_bootstrap_from_hw16(c, hardware16);
    return;
  }

  synthetic_clock_consume_pending_zero_if_any(c);
  const uint32_t counter32 = project_counter32_from_hw16(c, hardware16);
  const uint32_t delta = counter32 - c.current_counter32;
  c.current_counter32 = counter32;
  c.current_ns += (uint64_t)delta * 100ULL;
  c.hardware16 = hardware16;
  c.minder_update_count++;
}

static void synthetic_clock_observe_hw16_no_pending_zero(
    synthetic_clock32_t& c,
    uint16_t hardware16) {
  // Passive rollover-only observation.  Unlike synthetic_clock_tend_from_hw16(),
  // this does not consume pending ZERO requests.  That keeps the implicit CH2
  // seatbelt from changing existing ZERO/SmartZero ownership semantics while
  // still refreshing the 16-bit-to-32-bit extension whenever CH2 fires.
  if (!c.zeroed) {
    synthetic_clock_bootstrap_from_hw16(c, hardware16);
    return;
  }

  const uint32_t counter32 = project_counter32_from_hw16(c, hardware16);
  const uint32_t delta = counter32 - c.current_counter32;
  c.current_counter32 = counter32;
  c.current_ns += (uint64_t)delta * 100ULL;
  c.hardware16 = hardware16;
  c.minder_update_count++;
}

static void ocxo_lane_maybe_arm_one_second_compare(
    interrupt_subscriber_kind_t,
    ocxo_lane_t& lane,
    synthetic_clock32_t& clock32,
    uint32_t reason) {
  if (!lane.initialized || !lane.cadence_enabled) return;
  if (lane.cadence_armed) return;

  const uint32_t remaining = lane.cadence_next_counter32 - clock32.current_counter32;
  if (remaining == 0 || remaining > 0x7FFFFFFFUL) {
    const uint32_t next_target = lane.cadence_epoch_valid
        ? ocxo_one_second_next_target_after(lane.cadence_epoch_counter32,
                                            clock32.current_counter32)
        : (clock32.current_counter32 + OCXO_WITNESS_ONE_SECOND_COUNTS);
    lane.cadence_next_counter32 = next_target;
    lane.cadence_next_low16 = (uint16_t)(next_target & 0xFFFFU);
    lane.compare_target = lane.cadence_next_low16;
    lane.witness_target_counter32 = next_target;
    lane.witness_target_low16 = lane.cadence_next_low16;
    lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_TARGET_ADVANCED;
    return;
  }

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  lane.witness_schedule_last_current_counter32 = clock32.current_counter32;
  lane.witness_schedule_last_target_counter32 = lane.cadence_next_counter32;
  lane.witness_schedule_last_remaining_ticks = remaining;
  lane.witness_schedule_last_current_low16 = hw16;
  lane.witness_schedule_last_target_low16 = lane.cadence_next_low16;
  lane.witness_schedule_last_phase_ticks =
      lane.cadence_epoch_valid ? (clock32.current_counter32 - lane.cadence_epoch_counter32) : 0U;

  if (remaining > OCXO_WITNESS_ARM_WINDOW_TICKS) {
    lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW;
    lane.witness_schedule_last_ticks_until_arm_window =
        remaining - OCXO_WITNESS_ARM_WINDOW_TICKS;
    return;
  }

  if (remaining < OCXO_WITNESS_MIN_ARM_LEAD_TICKS) {
    lane.witness_late_arm_count++;
    lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_TOO_CLOSE;
    lane.witness_schedule_last_ticks_until_arm_window = 0;
    const uint32_t next_target = lane.cadence_epoch_valid
        ? ocxo_one_second_next_target_after(lane.cadence_epoch_counter32,
                                            clock32.current_counter32)
        : (clock32.current_counter32 + OCXO_WITNESS_ONE_SECOND_COUNTS);
    lane.cadence_next_counter32 = next_target;
    lane.cadence_next_low16 = (uint16_t)(next_target & 0xFFFFU);
    lane.compare_target = lane.cadence_next_low16;
    lane.witness_target_counter32 = next_target;
    lane.witness_target_low16 = lane.cadence_next_low16;
    return;
  }

  lane.witness_last_arm_dwt_raw = ARM_DWT_CYCCNT;
  lane.witness_last_arm_counter32 = clock32.current_counter32;
  lane.witness_last_arm_low16 = hw16;
  lane.witness_last_arm_target_counter32 = lane.cadence_next_counter32;
  lane.witness_last_arm_target_low16 = lane.cadence_next_low16;
  lane.witness_last_arm_remaining_ticks = remaining;
  lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_ARMED;
  lane.witness_schedule_last_ticks_until_arm_window = 0;
  ocxo_lane_program_local_cadence_compare(lane, lane.cadence_next_counter32,
                                          reason);
}

static void interrupt_ch2_implicit_rollover_tend(void) {
  if (!CH2_IMPLICIT_ROLLOVER_ENABLED || !g_interrupt_hw_ready) return;

  g_ch2_implicit_rollover_count++;

  // VCLOCK: refresh the process_interrupt-owned synthetic 32-bit extender
  // from the passive CH0 low word.  This is an ambient rollover observation,
  // not an event fact, and it does not publish/dispatch anything.
  const uint16_t vclock_hw16 = qtimer1_ch0_counter_now();
  vclock_clock_tend_from_hardware_low16(vclock_hw16);
  g_ch2_implicit_rollover_vclock_updates++;
  g_ch2_implicit_rollover_last_vclock_hw16 = vclock_hw16;
  g_ch2_implicit_rollover_last_vclock_counter32 =
      g_vclock_clock32.current_counter32;

  // OCXO lanes: opportunistically refresh the low-word extenders.  The
  // existing QTimer2/QTimer3 cadence ladders still own OCXO scheduled event
  // identity and one-second publication; this path is only rollover insurance.
  if (!OCXO1_DISABLED && g_ocxo1_lane.initialized) {
    const uint16_t ocxo1_hw16 = IMXRT_TMR2.CH[0].CNTR;
    synthetic_clock_observe_hw16_no_pending_zero(g_ocxo1_clock32, ocxo1_hw16);
    g_ch2_implicit_rollover_ocxo1_updates++;
    g_ch2_implicit_rollover_last_ocxo1_hw16 = ocxo1_hw16;
    g_ch2_implicit_rollover_last_ocxo1_counter32 =
        g_ocxo1_clock32.current_counter32;
    ocxo_lane_maybe_arm_one_second_compare(interrupt_subscriber_kind_t::OCXO1,
                                           g_ocxo1_lane,
                                           g_ocxo1_clock32,
                                           OCXO_CADENCE_REASON_REARM);
  }

  if (!OCXO2_DISABLED && g_ocxo2_lane.initialized) {
    const uint16_t ocxo2_hw16 = IMXRT_TMR3.CH[3].CNTR;
    synthetic_clock_observe_hw16_no_pending_zero(g_ocxo2_clock32, ocxo2_hw16);
    g_ch2_implicit_rollover_ocxo2_updates++;
    g_ch2_implicit_rollover_last_ocxo2_hw16 = ocxo2_hw16;
    g_ch2_implicit_rollover_last_ocxo2_counter32 =
        g_ocxo2_clock32.current_counter32;
    ocxo_lane_maybe_arm_one_second_compare(interrupt_subscriber_kind_t::OCXO2,
                                           g_ocxo2_lane,
                                           g_ocxo2_clock32,
                                           OCXO_CADENCE_REASON_REARM);
  }
}

bool interrupt_clock_snapshot(interrupt_subscriber_kind_t kind, interrupt_clock_snapshot_t* out) {
  if (!out) return false;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    const uint16_t hw = qtimer1_ch0_counter_now();
    out->hardware16 = hw;
    out->counter32 = vclock_synthetic_from_hardware_low16(hw);
    out->ns64 = project_ns64_from_hw16(g_vclock_clock32, hw);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    if (OCXO1_DISABLED) return false;
    const uint16_t hw = IMXRT_TMR2.CH[0].CNTR;
    out->hardware16 = hw;
    out->counter32 = project_counter32_from_hw16(g_ocxo1_clock32, hw);
    out->ns64 = project_ns64_from_hw16(g_ocxo1_clock32, hw);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    if (OCXO2_DISABLED) return false;
    const uint16_t hw = IMXRT_TMR3.CH[3].CNTR;
    out->hardware16 = hw;
    out->counter32 = project_counter32_from_hw16(g_ocxo2_clock32, hw);
    out->ns64 = project_ns64_from_hw16(g_ocxo2_clock32, hw);
    return true;
  }

  return false;
}


// ============================================================================
// SmartZero(R) — mathematically-qualified logical zero acquisition
// ============================================================================
//
// The old epoch packet trusted a latency-adjusted ISR capture. SmartZero does
// not.  It observes consecutive 1 kHz edge captures and accepts a lane only
// when the DWT interval matches the current PPS/GPIO-derived cycles-per-second
// ruler within a tight cycle window.  The later edge of the accepted pair is
// the lane's canonical zero anchor.  Acquisition is strictly serial:
// VCLOCK -> OCXO1 -> OCXO2.

static constexpr uint32_t SMARTZERO_INTERVAL_TICKS = SMARTZERO_COUNTER_DELTA_TICKS;
static constexpr int32_t  SMARTZERO_INTERVAL_TOLERANCE_CYCLES =
    SMARTZERO_DEFAULT_TOLERANCE_CYCLES;

struct smartzero_lane_runtime_t {
  interrupt_smartzero_lane_snapshot_t pub{};
  bool previous_present = false;
};

struct smartzero_runtime_t {
  volatile uint32_t seq = 0;
  interrupt_smartzero_phase_t phase = interrupt_smartzero_phase_t::IDLE;
  bool running = false;
  bool complete = false;
  uint32_t sequence = 0;
  uint32_t begin_count = 0;
  uint32_t complete_count = 0;
  uint32_t abort_count = 0;
  uint32_t current_lane_index = 0;
  smartzero_lane_runtime_t lanes[SMARTZERO_LANE_COUNT];
};

static smartzero_runtime_t g_smartzero = {};

static interrupt_subscriber_kind_t smartzero_kind_for_index(uint32_t index) {
  switch (index) {
    case 0: return interrupt_subscriber_kind_t::VCLOCK;
    case 1: return interrupt_subscriber_kind_t::OCXO1;
    case 2: return interrupt_subscriber_kind_t::OCXO2;
    default: return interrupt_subscriber_kind_t::NONE;
  }
}

static int smartzero_index_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return 0;
  if (kind == interrupt_subscriber_kind_t::OCXO1)  return 1;
  if (kind == interrupt_subscriber_kind_t::OCXO2)  return 2;
  return -1;
}

static bool smartzero_lane_enabled(uint32_t index) {
  const interrupt_subscriber_kind_t kind = smartzero_kind_for_index(index);
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return true;
  return !ocxo_kind_disabled(kind);
}

static bool smartzero_find_next_enabled_lane(uint32_t start_index,
                                             uint32_t* out_index) {
  for (uint32_t i = start_index; i < SMARTZERO_LANE_COUNT; i++) {
    if (smartzero_lane_enabled(i)) {
      if (out_index) *out_index = i;
      return true;
    }
  }
  return false;
}

static void smartzero_install_disabled_surrogate_locked(uint32_t index) {
  if (index == 0 || index >= SMARTZERO_LANE_COUNT) return;
  if (smartzero_lane_enabled(index)) return;

  // VCLOCK is never disabled and must already be locked before SmartZero can
  // complete.  Use it as the neutral synthetic proof source for disabled OCXO
  // lanes.  Disabled lanes never publish events; this is only an epoch-gate
  // compatibility proof for Alpha's existing three-lane invariant.
  const interrupt_smartzero_lane_snapshot_t& src = g_smartzero.lanes[0].pub;
  smartzero_lane_runtime_t& dst_runtime = g_smartzero.lanes[index];
  interrupt_smartzero_lane_snapshot_t& dst = dst_runtime.pub;

  dst_runtime.previous_present = false;
  dst.kind = smartzero_kind_for_index(index);
  dst.state = interrupt_smartzero_lane_state_t::LOCKED;
  dst.last_decision = interrupt_smartzero_decision_t::ACCEPTED;
  dst.sample_count = 0;
  dst.interval_attempt_count = 0;
  dst.accepted_count = 1;
  dst.rejected_count = 0;
  dst.waiting_for_cps_count = 0;
  dst.expected_interval_cycles = src.expected_interval_cycles;
  dst.tolerance_cycles = SMARTZERO_INTERVAL_TOLERANCE_CYCLES;
  dst.required_counter_delta_ticks = SMARTZERO_INTERVAL_TICKS;
  dst.cps_used = src.cps_used;
  dst.last_sample_dwt = src.last_sample_dwt;
  dst.last_sample_counter32 = src.last_sample_counter32;
  dst.last_sample_hardware16 = src.last_sample_hardware16;
  dst.previous_sample_dwt = src.previous_sample_dwt;
  dst.previous_sample_counter32 = src.previous_sample_counter32;
  dst.previous_sample_hardware16 = src.previous_sample_hardware16;
  dst.last_interval_cycles = src.last_interval_cycles;
  dst.last_interval_error_cycles = src.last_interval_error_cycles;
  dst.max_abs_interval_error_cycles = src.max_abs_interval_error_cycles;
  dst.last_counter_delta_ticks = src.last_counter_delta_ticks;
  dst.anchor_dwt = src.anchor_dwt;
  dst.anchor_counter32 = src.anchor_counter32;
  dst.anchor_hardware16 = src.anchor_hardware16;
  dst.anchor_pair_previous_dwt = src.anchor_pair_previous_dwt;
  dst.anchor_pair_previous_counter32 = src.anchor_pair_previous_counter32;
  dst.arm_count = 0;
  dst.fire_count = 0;
  dst.next_target_counter32 = 0;
}

static void smartzero_install_disabled_surrogates_locked(void) {
  smartzero_install_disabled_surrogate_locked(1);
  smartzero_install_disabled_surrogate_locked(2);
}

static const char* smartzero_decision_name(interrupt_smartzero_decision_t d) {
  switch (d) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:  return "WAITING_FOR_CPS";
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:     return "FIRST_SAMPLE";
    case interrupt_smartzero_decision_t::ACCEPTED:         return "ACCEPTED";
    case interrupt_smartzero_decision_t::REJECTED_DWT:     return "REJECTED_DWT";
    case interrupt_smartzero_decision_t::REJECTED_COUNTER: return "REJECTED_COUNTER";
    default:                                               return "NONE";
  }
}

static const char* smartzero_lane_state_name(interrupt_smartzero_lane_state_t s) {
  switch (s) {
    case interrupt_smartzero_lane_state_t::ACQUIRING: return "ACQUIRING";
    case interrupt_smartzero_lane_state_t::LOCKED:    return "LOCKED";
    default:                                         return "IDLE";
  }
}

static const char* smartzero_phase_name(interrupt_smartzero_phase_t p) {
  switch (p) {
    case interrupt_smartzero_phase_t::RUNNING:  return "RUNNING";
    case interrupt_smartzero_phase_t::COMPLETE: return "COMPLETE";
    case interrupt_smartzero_phase_t::ABORTED:  return "ABORTED";
    default:                                    return "IDLE";
  }
}

static inline void smartzero_write_begin(void) {
  g_smartzero.seq++;
  dmb_barrier();
}

static inline void smartzero_write_end(void) {
  dmb_barrier();
  g_smartzero.seq++;
}

static void smartzero_reset_lane(uint32_t index) {
  smartzero_lane_runtime_t& r = g_smartzero.lanes[index];
  r = smartzero_lane_runtime_t{};
  r.pub.kind = smartzero_kind_for_index(index);
  r.pub.state = interrupt_smartzero_lane_state_t::IDLE;
  r.pub.last_decision = interrupt_smartzero_decision_t::NONE;
  r.pub.tolerance_cycles = SMARTZERO_INTERVAL_TOLERANCE_CYCLES;
  r.pub.required_counter_delta_ticks = SMARTZERO_INTERVAL_TICKS;
}

static bool smartzero_is_current_lane(interrupt_subscriber_kind_t kind) {
  if (!g_smartzero.running || g_smartzero.complete) return false;
  return smartzero_kind_for_index(g_smartzero.current_lane_index) == kind;
}

static uint32_t smartzero_expected_interval_cycles(uint32_t* out_cps) {
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  if (out_cps) *out_cps = cps;

  // SmartZero's admissibility ruler is intentionally PPS/GPIO-derived.  The
  // fallback DWT_EXPECTED_PER_PPS is close, but ZERO authority should wait for
  // the physical PPS witness to give us a live one-second ruler.
  if (!clocks_dwt_calibration_valid() || cps == 0) return 0;

  return (uint32_t)(((uint64_t)cps +
                     (uint64_t)SMARTZERO_SAMPLE_RATE_HZ / 2ULL) /
                    (uint64_t)SMARTZERO_SAMPLE_RATE_HZ);
}

static void smartzero_arm_ocxo_lane(interrupt_subscriber_kind_t kind) {
  const int idx = smartzero_index_for_kind(kind);
  if (idx < 0) return;

  if (ocxo_kind_disabled(kind)) return;

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized) return;

  // SmartZero now rides the same lane-local 1 kHz compare cadence that will
  // later feed regression.  The compare target is the authored sample
  // identity; service-time low-word reads remain diagnostics only.
  ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_SMARTZERO);

  const uint16_t hw16 = ocxo_lane_counter_now(*lane);
  synthetic_clock_tend_from_hw16(*clock32, hw16);
  const uint32_t target_counter32 =
      clock32->current_counter32 + SMARTZERO_INTERVAL_TICKS;

  smartzero_lane_runtime_t& r = g_smartzero.lanes[idx];
  r.pub.next_target_counter32 = target_counter32;
  r.pub.arm_count++;

  (void)ocxo_lane_start_local_cadence_at_target(kind, *lane, *clock32,
                                                target_counter32,
                                                OCXO_CADENCE_REASON_SMARTZERO);
}

static void smartzero_arm_current_lane(void) {
  const interrupt_subscriber_kind_t kind =
      smartzero_kind_for_index(g_smartzero.current_lane_index);
  if (kind == interrupt_subscriber_kind_t::OCXO1 ||
      kind == interrupt_subscriber_kind_t::OCXO2) {
    smartzero_arm_ocxo_lane(kind);
  }
}

static void smartzero_advance_or_complete(void) {
  uint32_t next_index = 0;
  if (!smartzero_find_next_enabled_lane(g_smartzero.current_lane_index + 1U,
                                        &next_index)) {
    smartzero_install_disabled_surrogates_locked();
    g_smartzero.running = false;
    g_smartzero.complete = true;
    g_smartzero.phase = interrupt_smartzero_phase_t::COMPLETE;
    g_smartzero.complete_count++;
    return;
  }

  g_smartzero.current_lane_index = next_index;
  smartzero_lane_runtime_t& next = g_smartzero.lanes[g_smartzero.current_lane_index];
  next.pub.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  next.pub.last_decision = interrupt_smartzero_decision_t::NONE;
  next.previous_present = false;
  smartzero_arm_current_lane();
}

static bool smartzero_feed_sample(interrupt_subscriber_kind_t kind,
                                  uint32_t event_dwt,
                                  uint32_t counter32,
                                  uint16_t hardware16) {
  if (!smartzero_is_current_lane(kind)) return false;

  const int idx = smartzero_index_for_kind(kind);
  if (idx < 0) return false;

  uint32_t cps = 0;
  const uint32_t expected = smartzero_expected_interval_cycles(&cps);

  smartzero_write_begin();

  smartzero_lane_runtime_t& r = g_smartzero.lanes[idx];
  interrupt_smartzero_lane_snapshot_t& z = r.pub;

  z.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  z.sample_count++;
  z.cps_used = cps;
  z.expected_interval_cycles = expected;
  z.last_sample_dwt = event_dwt;
  z.last_sample_counter32 = counter32;
  z.last_sample_hardware16 = hardware16;

  if (expected == 0) {
    z.waiting_for_cps_count++;
    z.last_decision = interrupt_smartzero_decision_t::WAITING_FOR_CPS;
    r.previous_present = false;
    smartzero_write_end();
    return false;
  }

  if (!r.previous_present) {
    z.previous_sample_dwt = event_dwt;
    z.previous_sample_counter32 = counter32;
    z.previous_sample_hardware16 = hardware16;
    z.last_decision = interrupt_smartzero_decision_t::FIRST_SAMPLE;
    r.previous_present = true;
    smartzero_write_end();
    return false;
  }

  const uint32_t previous_dwt = z.previous_sample_dwt;
  const uint32_t previous_counter32 = z.previous_sample_counter32;
  const uint16_t previous_hw16 = z.previous_sample_hardware16;

  const uint32_t interval_cycles = event_dwt - previous_dwt;
  const uint32_t counter_delta = counter32 - previous_counter32;
  const int32_t error_cycles =
      (int32_t)((int64_t)interval_cycles - (int64_t)expected);
  const uint32_t abs_error = (uint32_t)(error_cycles < 0
      ? -(int64_t)error_cycles
      :  (int64_t)error_cycles);

  z.interval_attempt_count++;
  z.last_interval_cycles = interval_cycles;
  z.last_interval_error_cycles = error_cycles;
  z.last_counter_delta_ticks = counter_delta;
  if (abs_error > z.max_abs_interval_error_cycles) {
    z.max_abs_interval_error_cycles = abs_error;
  }

  const bool counter_ok = (counter_delta == SMARTZERO_INTERVAL_TICKS);
  const bool dwt_ok = ((int32_t)abs_error <= SMARTZERO_INTERVAL_TOLERANCE_CYCLES);

  if (counter_ok && dwt_ok) {
    bool ocxo_cadence_reauthored = false;

    z.accepted_count++;
    z.state = interrupt_smartzero_lane_state_t::LOCKED;
    z.last_decision = interrupt_smartzero_decision_t::ACCEPTED;
    z.anchor_dwt = event_dwt;
    z.anchor_counter32 = counter32;
    z.anchor_hardware16 = hardware16;
    z.anchor_pair_previous_dwt = previous_dwt;
    z.anchor_pair_previous_counter32 = previous_counter32;

    if (kind == interrupt_subscriber_kind_t::OCXO1 ||
        kind == interrupt_subscriber_kind_t::OCXO2) {
      ocxo_lane_t* lane = ocxo_lane_for(kind);
      synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
      if (lane && clock32) {
        ocxo_lane_install_logical_grid(kind, *lane, *clock32, counter32,
                                       OCXO_CADENCE_REASON_SMARTZERO);
        ocxo_cadence_reauthored = true;
        if (!lane->active) {
          ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_SMARTZERO);
        }
      }
    }

    smartzero_advance_or_complete();
    smartzero_write_end();
    return ocxo_cadence_reauthored;
  }

  z.rejected_count++;
  z.last_decision = counter_ok
      ? interrupt_smartzero_decision_t::REJECTED_DWT
      : interrupt_smartzero_decision_t::REJECTED_COUNTER;

  // Roll the observation window forward.  If this sample was the bad one, the
  // next interval will reject; if it was good, the next good edge can lock.
  z.previous_sample_dwt = event_dwt;
  z.previous_sample_counter32 = counter32;
  z.previous_sample_hardware16 = hardware16;
  (void)previous_hw16;

  smartzero_write_end();
  return false;
}

// OCXO SmartZero samples are now served by the same lane-local 1 kHz
// compare cadence used for normal rollover custody.  There is intentionally
// no separate SmartZero ISR path here; the unified OCXO cadence ISR feeds
// smartzero_feed_sample() with the authored compare-target identity.

bool interrupt_smartzero_begin(void) {
  if (!g_interrupt_runtime_ready || !g_interrupt_hw_ready) return false;

  smartzero_write_begin();

  g_smartzero.running = true;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::RUNNING;
  g_smartzero.sequence++;
  g_smartzero.begin_count++;
  g_smartzero.current_lane_index = 0;
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }
  g_smartzero.lanes[0].pub.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  vclock_ch2_smartzero_reset_attempt_state();

  // SmartZero takes temporary custody of the OCXO local compare cadence only
  // when each OCXO lane becomes current.  Stop any running OCXO cadence now so
  // the acquisition sequence can re-author clean +10,000-tick sample ladders.
  if (!OCXO1_DISABLED) ocxo_lane_stop_local_cadence(g_ocxo1_lane, OCXO_CADENCE_REASON_SMARTZERO);
  if (!OCXO2_DISABLED) ocxo_lane_stop_local_cadence(g_ocxo2_lane, OCXO_CADENCE_REASON_SMARTZERO);

  smartzero_write_end();
  return true;
}

void interrupt_smartzero_abort(void) {
  smartzero_write_begin();
  g_smartzero.running = false;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::ABORTED;
  g_smartzero.abort_count++;
  g_smartzero.current_lane_index = SMARTZERO_LANE_COUNT;

  // Live acquisition state is not the installed proof.  When a live attempt
  // is aborted, clear its lane proof surface so reports do not show stale
  // LOCKED anchors under smartzero_complete=false.  CLOCKS/Alpha retains the
  // installed SmartZero proof separately.
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }

  smartzero_write_end();
  vclock_ch2_smartzero_deactivate();

  // SmartZero may temporarily stop/re-author the OCXO local compare cadence
  // while acquiring lane proofs.  Aborting the *live acquisition attempt* must
  // not silently disable active clock service.  If an OCXO lane is active,
  // restore its normal local 1 kHz cadence from the currently installed grid;
  // if it is inactive, leave it stopped as before.
  if (!OCXO1_DISABLED) {
    if (g_ocxo1_lane.active) {
      (void)ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t::OCXO1,
                                          g_ocxo1_lane,
                                          g_ocxo1_clock32,
                                          OCXO_CADENCE_REASON_START);
    } else {
      ocxo_lane_stop_local_cadence(g_ocxo1_lane, OCXO_CADENCE_REASON_STOP);
    }
  }

  if (!OCXO2_DISABLED) {
    if (g_ocxo2_lane.active) {
      (void)ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t::OCXO2,
                                          g_ocxo2_lane,
                                          g_ocxo2_clock32,
                                          OCXO_CADENCE_REASON_START);
    } else {
      ocxo_lane_stop_local_cadence(g_ocxo2_lane, OCXO_CADENCE_REASON_STOP);
    }
  }
}

bool interrupt_smartzero_running(void) {
  return g_smartzero.running && !g_smartzero.complete;
}

bool interrupt_smartzero_complete(void) {
  return g_smartzero.complete;
}

bool interrupt_smartzero_live_snapshot(interrupt_smartzero_snapshot_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = g_smartzero.seq;
    dmb_barrier();

    interrupt_smartzero_snapshot_t local{};
    local.phase = g_smartzero.phase;
    local.running = g_smartzero.running;
    local.complete = g_smartzero.complete;
    local.aborted = (g_smartzero.phase == interrupt_smartzero_phase_t::ABORTED);
    local.sequence = g_smartzero.sequence;
    local.begin_count = g_smartzero.begin_count;
    local.complete_count = g_smartzero.complete_count;
    local.abort_count = g_smartzero.abort_count;
    local.current_lane_index = g_smartzero.current_lane_index;
    local.current_lane = g_smartzero.running
        ? smartzero_kind_for_index(g_smartzero.current_lane_index)
        : interrupt_subscriber_kind_t::NONE;
    local.tolerance_cycles = SMARTZERO_INTERVAL_TOLERANCE_CYCLES;
    local.sample_rate_hz = SMARTZERO_SAMPLE_RATE_HZ;
    local.counter_delta_ticks = SMARTZERO_INTERVAL_TICKS;
    for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
      local.lanes[i] = g_smartzero.lanes[i].pub;
    }

    dmb_barrier();
    const uint32_t seq2 = g_smartzero.seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.complete;
    }
  }

  *out = interrupt_smartzero_snapshot_t{};
  return false;
}

bool interrupt_smartzero_snapshot(interrupt_smartzero_snapshot_t* out) {
  return interrupt_smartzero_live_snapshot(out);
}

// ============================================================================
// Compare channel helpers
// ============================================================================

static inline void qtimer1_ch1_clear_compare_flag(void) {
  IMXRT_TMR1.CH[1].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer1_ch1_program_compare(uint16_t target_low16) {
  qtimer1_ch1_clear_compare_flag();
  IMXRT_TMR1.CH[1].COMP1  = target_low16;
  IMXRT_TMR1.CH[1].CMPLD1 = target_low16;
  qtimer1_ch1_clear_compare_flag();
  IMXRT_TMR1.CH[1].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer1_ch1_disable_compare_hw(void) {
  IMXRT_TMR1.CH[1].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_ch1_clear_compare_flag();
}

static inline void qtimer1_ch2_clear_compare_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer1_ch2_program_compare(uint16_t target_low16) {
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].COMP1  = target_low16;
  IMXRT_TMR1.CH[2].CMPLD1 = target_low16;
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

// QuadTimer compare flags are cleared by writing the flag bits low in CSCTRL.
// Match the QTimer1 CH1/CH2 clear idiom.
static inline void qtimer2_ch0_clear_compare_flag(void) {
  IMXRT_TMR2.CH[0].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)IMXRT_TMR2.CH[0].CSCTRL;
}

static inline void qtimer2_ch0_program_compare(uint16_t target_low16) {
  qtimer2_ch0_clear_compare_flag();
  IMXRT_TMR2.CH[0].COMP1  = target_low16;
  IMXRT_TMR2.CH[0].CMPLD1 = target_low16;
  qtimer2_ch0_clear_compare_flag();
  IMXRT_TMR2.CH[0].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer2_ch0_disable_compare(void) {
  IMXRT_TMR2.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer2_ch0_clear_compare_flag();
  qtimer2_ch0_clear_compare_flag();
}

// QuadTimer compare flags are cleared by writing the flag bits low in CSCTRL.
// Match the QTimer1 CH1/CH2 clear idiom.
static inline void qtimer3_clear_compare_flag(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)IMXRT_TMR3.CH[ch].CSCTRL;
}

static inline void qtimer3_program_compare(uint8_t ch, uint16_t target_low16) {
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].COMP1  = target_low16;
  IMXRT_TMR3.CH[ch].CMPLD1 = target_low16;
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer3_disable_compare(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer3_clear_compare_flag(ch);
  qtimer3_clear_compare_flag(ch);
}

// ============================================================================
// Subscriber lookup helpers
// ============================================================================

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].desc && g_subscribers[i].desc->kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind) {
  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  return ctx ? ctx->lane : nullptr;
}

static const char* dispatch_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1:  return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2:  return "OCXO2_DISPATCH";
    default: return "";
  }
}

// ============================================================================
// Shared 1 kHz linear regression engine
// ============================================================================

struct cadence_regression_sample_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  uint32_t observed_dwt_at_event = 0;
  uint32_t target_counter32_at_event = 0;
  uint16_t target_hardware16_at_event = 0;
  uint16_t observed_hardware16_at_event = 0;
  bool one_second_due = false;
};

struct cadence_regression_result_t {
  bool valid = false;
  uint32_t sequence = 0;
  uint32_t sample_count = 0;

  uint32_t observed_dwt_at_event = 0;
  uint32_t inferred_dwt_at_event = 0;
  int32_t inferred_minus_observed_cycles = 0;

  uint32_t target_counter32_at_event = 0;
  uint16_t target_hardware16_at_event = 0;
  uint16_t observed_hardware16_at_event = 0;

  uint64_t slope_q16_cycles_per_sample = 0;
  int64_t slope_delta_q16_cycles_per_sample = 0;
  int32_t fit_error_mean_q16_cycles = 0;
  uint32_t fit_error_stddev_q16_cycles = 0;
  int32_t fit_error_min_cycles = 0;
  int32_t fit_error_max_cycles = 0;
  uint32_t fit_error_gt_plus4_count = 0;
  uint32_t fit_error_lt_minus4_count = 0;
  uint32_t fit_error_abs_gt4_count = 0;
};

struct cadence_regression_lane_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  bool window_active = false;
  uint32_t sequence = 0;
  uint32_t sample_total_count = 0;
  uint32_t reset_count = 0;
  uint32_t insufficient_count = 0;
  uint32_t counter_discontinuity_count = 0;

  uint32_t sample_count = 0;
  uint32_t base_counter32 = 0;
  uint32_t base_observed_dwt = 0;
  uint32_t last_counter32 = 0;
  uint32_t last_observed_dwt = 0;

  bool previous_slope_valid = false;
  double previous_slope_cycles_per_sample = 0.0;

  int32_t observed_dwt_rel[REGRESSION_SAMPLES_PER_SECOND]{};
  cadence_regression_result_t last_result{};
};

static cadence_regression_lane_t g_regression_vclock = {};
static cadence_regression_lane_t g_regression_ocxo1 = {};
static cadence_regression_lane_t g_regression_ocxo2 = {};

static int32_t regression_round_i32(double value) {
  return (int32_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
}

static uint64_t regression_round_u64_q16(double value) {
  if (value <= 0.0) return 0ULL;
  return (uint64_t)(value * REGRESSION_Q16_SCALE + 0.5);
}

static uint32_t regression_round_u32_q16(double value) {
  if (value <= 0.0) return 0U;
  const double scaled = value * REGRESSION_Q16_SCALE + 0.5;
  return (scaled > (double)UINT32_MAX) ? UINT32_MAX : (uint32_t)scaled;
}

static int32_t regression_round_i32_q16(double value) {
  const double scaled = value * REGRESSION_Q16_SCALE;
  if (scaled > (double)INT32_MAX) return INT32_MAX;
  if (scaled < (double)INT32_MIN) return INT32_MIN;
  return regression_round_i32(scaled);
}

static int64_t regression_round_i64_q16(double value) {
  const double scaled = value * REGRESSION_Q16_SCALE;
  return (int64_t)((scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5));
}

static cadence_regression_lane_t*
cadence_regression_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return &g_regression_vclock;
  if (kind == interrupt_subscriber_kind_t::OCXO1)  return &g_regression_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2)  return &g_regression_ocxo2;
  return nullptr;
}

static const cadence_regression_lane_t*
cadence_regression_for_const(interrupt_subscriber_kind_t kind) {
  return cadence_regression_for(kind);
}

static void cadence_regression_reset_window(cadence_regression_lane_t& r) {
  r.window_active = false;
  r.sample_count = 0;
  r.base_counter32 = 0;
  r.base_observed_dwt = 0;
  r.last_counter32 = 0;
  r.last_observed_dwt = 0;
}

static void cadence_regression_reset_kind(interrupt_subscriber_kind_t kind) {
  cadence_regression_lane_t* r = cadence_regression_for(kind);
  if (!r) return;

  const interrupt_subscriber_kind_t saved_kind = r->kind;
  const uint32_t prior_reset_count = r->reset_count;
  *r = cadence_regression_lane_t{};
  r->kind = (saved_kind == interrupt_subscriber_kind_t::NONE) ? kind : saved_kind;
  r->reset_count = prior_reset_count + 1U;
}

static void cadence_regression_reset_all(void) {
  g_regression_vclock = cadence_regression_lane_t{};
  g_regression_ocxo1 = cadence_regression_lane_t{};
  g_regression_ocxo2 = cadence_regression_lane_t{};
  g_regression_vclock.kind = interrupt_subscriber_kind_t::VCLOCK;
  g_regression_ocxo1.kind = interrupt_subscriber_kind_t::OCXO1;
  g_regression_ocxo2.kind = interrupt_subscriber_kind_t::OCXO2;
}

static cadence_regression_result_t
cadence_regression_result_from_sample(cadence_regression_lane_t& r,
                                      const cadence_regression_sample_t& sample,
                                      bool valid) {
  cadence_regression_result_t out{};
  out.valid = valid;
  out.sequence = r.sequence;
  out.sample_count = r.sample_count;
  out.observed_dwt_at_event = sample.observed_dwt_at_event;
  out.inferred_dwt_at_event = sample.observed_dwt_at_event;
  out.inferred_minus_observed_cycles = 0;
  out.target_counter32_at_event = sample.target_counter32_at_event;
  out.target_hardware16_at_event = sample.target_hardware16_at_event;
  out.observed_hardware16_at_event = sample.observed_hardware16_at_event;
  return out;
}

static cadence_regression_result_t
cadence_regression_finalize(cadence_regression_lane_t& r,
                            const cadence_regression_sample_t& event_sample) {
  const uint32_t n = r.sample_count;
  if (n < REGRESSION_MIN_SAMPLE_COUNT) {
    r.insufficient_count++;
    cadence_regression_result_t out =
        cadence_regression_result_from_sample(r, event_sample, false);
    r.last_result = out;
    return out;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;

  for (uint32_t i = 0; i < n; i++) {
    const double x = (double)i;
    const double y = (double)r.observed_dwt_rel[i];
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }

  const double denom = (double)n * sum_xx - sum_x * sum_x;
  if (denom == 0.0) {
    r.insufficient_count++;
    cadence_regression_result_t out =
        cadence_regression_result_from_sample(r, event_sample, false);
    r.last_result = out;
    return out;
  }

  const double slope = ((double)n * sum_xy - sum_x * sum_y) / denom;
  const double intercept = (sum_y - slope * sum_x) / (double)n;
  const double x_event = (double)(n - 1U);
  const double inferred_rel_d = intercept + slope * x_event;
  const int32_t inferred_rel = regression_round_i32(inferred_rel_d);

  double mean = 0.0;
  double m2 = 0.0;
  int32_t min_err = INT32_MAX;
  int32_t max_err = INT32_MIN;
  uint32_t gt_plus4 = 0;
  uint32_t lt_minus4 = 0;
  uint32_t abs_gt4 = 0;

  for (uint32_t i = 0; i < n; i++) {
    const double estimate = intercept + slope * (double)i;
    const double err_d = estimate - (double)r.observed_dwt_rel[i];
    const int32_t err = regression_round_i32(err_d);

    const double d1 = err_d - mean;
    mean += d1 / (double)(i + 1U);
    const double d2 = err_d - mean;
    m2 += d1 * d2;

    if (err < min_err) min_err = err;
    if (err > max_err) max_err = err;
    if (err_d > (double)REGRESSION_FIT_ERROR_THRESHOLD_CYCLES) gt_plus4++;
    if (err_d < (double)(-REGRESSION_FIT_ERROR_THRESHOLD_CYCLES)) lt_minus4++;
    if (fabs(err_d) > (double)REGRESSION_FIT_ERROR_THRESHOLD_CYCLES) abs_gt4++;
  }

  const double variance = (n >= 2U) ? (m2 / (double)(n - 1U)) : 0.0;
  const double stddev = sqrt(variance < 0.0 ? 0.0 : variance);

  r.sequence++;
  cadence_regression_result_t out{};
  out.valid = true;
  out.sequence = r.sequence;
  out.sample_count = n;
  out.observed_dwt_at_event = event_sample.observed_dwt_at_event;
  out.inferred_dwt_at_event = r.base_observed_dwt + (uint32_t)inferred_rel;
  out.inferred_minus_observed_cycles =
      (int32_t)(out.inferred_dwt_at_event - event_sample.observed_dwt_at_event);
  out.target_counter32_at_event = event_sample.target_counter32_at_event;
  out.target_hardware16_at_event = event_sample.target_hardware16_at_event;
  out.observed_hardware16_at_event = event_sample.observed_hardware16_at_event;
  out.slope_q16_cycles_per_sample = regression_round_u64_q16(slope);
  out.slope_delta_q16_cycles_per_sample = r.previous_slope_valid
      ? regression_round_i64_q16(slope - r.previous_slope_cycles_per_sample)
      : 0;
  out.fit_error_mean_q16_cycles = regression_round_i32_q16(mean);
  out.fit_error_stddev_q16_cycles = regression_round_u32_q16(stddev);
  out.fit_error_min_cycles = min_err;
  out.fit_error_max_cycles = max_err;
  out.fit_error_gt_plus4_count = gt_plus4;
  out.fit_error_lt_minus4_count = lt_minus4;
  out.fit_error_abs_gt4_count = abs_gt4;

  r.previous_slope_cycles_per_sample = slope;
  r.previous_slope_valid = true;
  r.last_result = out;
  return out;
}

static cadence_regression_result_t
cadence_regression_feed_sample(interrupt_subscriber_kind_t kind,
                               const cadence_regression_sample_t& sample) {
  cadence_regression_lane_t* r = cadence_regression_for(kind);
  if (!r) return cadence_regression_result_t{};

  r->sample_total_count++;

  if (!r->window_active || r->sample_count >= REGRESSION_SAMPLES_PER_SECOND) {
    cadence_regression_reset_window(*r);
    r->window_active = true;
    r->base_counter32 = sample.target_counter32_at_event;
    r->base_observed_dwt = sample.observed_dwt_at_event;
  } else {
    const uint32_t expected_counter =
        r->base_counter32 + r->sample_count * REGRESSION_COUNTER_DELTA_TICKS;
    if (sample.target_counter32_at_event != expected_counter) {
      r->counter_discontinuity_count++;
      cadence_regression_reset_window(*r);
      r->window_active = true;
      r->base_counter32 = sample.target_counter32_at_event;
      r->base_observed_dwt = sample.observed_dwt_at_event;
    }
  }

  const uint32_t idx = r->sample_count;
  if (idx < REGRESSION_SAMPLES_PER_SECOND) {
    r->observed_dwt_rel[idx] =
        (int32_t)(sample.observed_dwt_at_event - r->base_observed_dwt);
    r->sample_count++;
  }

  r->last_counter32 = sample.target_counter32_at_event;
  r->last_observed_dwt = sample.observed_dwt_at_event;

  if (!sample.one_second_due) {
    return cadence_regression_result_t{};
  }

  cadence_regression_result_t out = cadence_regression_finalize(*r, sample);
  cadence_regression_reset_window(*r);
  return out;
}

static void copy_regression_diag(interrupt_capture_diag_t& diag,
                                 const cadence_regression_result_t* r) {
  if (!r) return;

  diag.regression_valid = r->valid;
  diag.regression_sequence = r->sequence;
  diag.regression_sample_count = r->sample_count;
  diag.regression_observed_dwt_at_event = r->observed_dwt_at_event;
  diag.regression_inferred_dwt_at_event = r->inferred_dwt_at_event;
  diag.regression_inferred_minus_observed_cycles =
      r->inferred_minus_observed_cycles;
  diag.regression_target_counter32_at_event = r->target_counter32_at_event;
  diag.regression_target_hardware16_at_event = r->target_hardware16_at_event;
  diag.regression_observed_hardware16_at_event =
      r->observed_hardware16_at_event;
  diag.regression_slope_q16_cycles_per_sample =
      r->slope_q16_cycles_per_sample;
  diag.regression_slope_delta_q16_cycles_per_sample =
      r->slope_delta_q16_cycles_per_sample;
  diag.regression_fit_error_mean_q16_cycles =
      r->fit_error_mean_q16_cycles;
  diag.regression_fit_error_stddev_q16_cycles =
      r->fit_error_stddev_q16_cycles;
  diag.regression_fit_error_min_cycles = r->fit_error_min_cycles;
  diag.regression_fit_error_max_cycles = r->fit_error_max_cycles;
  diag.regression_fit_error_gt_plus4_count = r->fit_error_gt_plus4_count;
  diag.regression_fit_error_lt_minus4_count = r->fit_error_lt_minus4_count;
  diag.regression_fit_error_abs_gt4_count = r->fit_error_abs_gt4_count;

  // Diagnostic-only build: do not promote the fitted DWT into any legacy
  // correction/authority field.  Alpha/Beta can compare diag.dwt_at_event
  // against regression_inferred_dwt_at_event without changing physics math.
}

// ============================================================================
// Diag fill + event dispatch
// ============================================================================

static void fill_diag(interrupt_capture_diag_t& diag,
                      const interrupt_subscriber_runtime_t& rt,
                      const interrupt_event_t& event) {
  diag.enabled  = true;
  diag.provider = rt.desc->provider;
  diag.lane     = rt.desc->lane;
  diag.kind     = rt.desc->kind;
  diag.dwt_at_event       = event.dwt_at_event;
  diag.gnss_ns_at_event   = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    diag.pps_edge_sequence          = g_pps_gpio_heartbeat.edge_count;
    diag.pps_edge_dwt_isr_entry_raw = g_pps_gpio_heartbeat.last_dwt;  // legacy field; carries pvc.dwt_at_edge
    diag.pps_edge_gnss_ns           = g_pps_gpio_heartbeat.last_gnss_ns;
    diag.pps_edge_minus_event_ns =
        (g_pps_gpio_heartbeat.last_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
            ? (g_pps_gpio_heartbeat.last_gnss_ns - (int64_t)event.gnss_ns_at_event)
            : 0;
    diag.pps_coincidence_cycles = event.pps_coincidence_cycles;
    diag.pps_coincidence_valid  = event.pps_coincidence_valid;
  }

  if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1 ||
      rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) {
    const ocxo_lane_t* lane = ocxo_lane_for(rt.desc->kind);
    if (lane) {
      diag.ocxo_service_class = lane->witness_last_service_class;
      diag.ocxo_service_offset_signed_ticks =
          lane->witness_last_service_offset_signed_ticks;
      diag.ocxo_service_offset_abs_ticks =
          lane->witness_last_service_offset_abs_ticks;
      diag.ocxo_interpreted_late_ticks =
          lane->witness_last_interpreted_late_ticks;
      diag.ocxo_early_ticks = lane->witness_last_early_ticks;
      diag.ocxo_target_delta_mod65536_ticks =
          lane->witness_last_target_delta_mod65536_ticks;
      diag.ocxo_arm_remaining_ticks = lane->witness_last_arm_remaining_ticks;
      diag.ocxo_arm_to_isr_ticks = lane->witness_last_arm_to_isr_ticks;
      diag.ocxo_arm_to_isr_dwt_cycles = lane->witness_last_arm_to_isr_dwt_cycles;
      diag.ocxo_perishable_fact_sequence = lane->witness_last_fact_sequence;
      diag.ocxo_service_correction_cycles =
          lane->witness_last_service_correction_cycles;
      // Traditional-authority build: do not populate the legacy
      // service-corrected endpoint field.  Alpha treats zero here as
      // "no replacement" and consumes event.dwt_at_event directly.
      // The scalar correction magnitude remains visible in
      // ocxo_service_correction_cycles; the regression-inferred endpoint is
      // published in the regression_* diagnostic fields.
      diag.ocxo_service_corrected_dwt_at_event = 0U;
      diag.ocxo_fact_ring_overflow_count = lane->witness_fact_overflow_count;
      diag.ocxo_counter_delta_violation_count =
          lane->witness_counter_delta_violation_count;
      diag.ocxo_last_bad_counter_delta = lane->witness_last_bad_counter_delta;
      diag.ocxo_last_counter_delta_ticks = lane->witness_last_counter_delta_ticks;
      diag.ocxo_sample_phase_valid = lane->cadence_sample_phase_valid;
      diag.ocxo_sample_phase_ticks = lane->cadence_sample_phase_ticks;
      diag.ocxo_sample_phase_ns = lane->cadence_sample_phase_ns;
      diag.ocxo_sample_phase_us = lane->cadence_sample_phase_us;
      diag.ocxo_sample_period_ticks = lane->cadence_sample_period_ticks;
      diag.ocxo_sample_dwt_at_event = event.dwt_at_event;
      diag.ocxo_sample_counter32_at_event = event.counter32_at_event;
    }
  }
}

static void maybe_dispatch_event(interrupt_subscriber_runtime_t& rt) {
  if (!rt.subscribed || !rt.sub.on_event) return;
  rt.dispatch_count++;
  rt.sub.on_event(rt.last_event, &rt.last_diag, rt.sub.user_data);
}

static void deferred_dispatch_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;
  maybe_dispatch_event(*rt);
}

static void emit_one_second_event(interrupt_subscriber_runtime_t& rt,
                                  uint32_t dwt_at_event,
                                  uint32_t authored_counter32,
                                  uint32_t pps_coincidence_cycles = 0,
                                  bool     pps_coincidence_valid  = false,
                                  const dwt_repair_diag_t* repair = nullptr,
                                  bool dispatch_immediately = false,
                                  const cadence_regression_result_t* regression = nullptr) {
  if (!rt.active) return;

  interrupt_event_t event {};
  event.kind         = rt.desc->kind;
  event.provider     = rt.desc->provider;
  event.lane         = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.counter32_at_event     = authored_counter32;
  event.pps_coincidence_cycles = pps_coincidence_cycles;
  event.pps_coincidence_valid  = pps_coincidence_valid;

  // VCLOCK remains unavailable here because process_interrupt does not
  // invent campaign GNSS labels for the sovereign PPS/VCLOCK rail.
  // OCXO lanes normalize the authored sample DWT through the labeled
  // PPS_VCLOCK anchor ring. For OCXO this is a subscriber-visible
  // GNSS-at-sample witness only; Alpha has not yet promoted it to
  // canonical residual/servo truth.
  bridge_projection_t bridge{};
  int64_t gnss_ns = -1;
  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    gnss_ns = vclock_gnss_from_counter32(authored_counter32);
    bridge.anchor_selection_kind = ANCHOR_SELECT_NONE;
  } else {
    bridge = interrupt_dwt_to_vclock_gnss_projection(dwt_at_event);
    gnss_ns = bridge.gnss_ns;
    bridge_projection_record_stats(rt.desc->kind, bridge);
  }
  event.gnss_ns_at_event = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
  event.status = interrupt_event_status_t::OK;

  interrupt_capture_diag_t diag {};
  fill_diag(diag, rt, event);
  copy_regression_diag(diag, regression);
  if (repair) {
    diag.dwt_synthetic = repair->synthetic;
    diag.dwt_repair_candidate = repair->candidate;
    diag.dwt_original_at_event = repair->original_dwt;
    diag.dwt_predicted_at_event = repair->predicted_dwt;
    diag.dwt_used_at_event = repair->used_dwt;
    diag.dwt_synthetic_error_cycles = repair->error_cycles;
    diag.dwt_synthetic_threshold_cycles = VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES;
    diag.dwt_synthetic_reason = repair->reason;
  }
  if (rt.desc->kind != interrupt_subscriber_kind_t::VCLOCK) {
    bridge_projection_copy_to_diag(diag, bridge);
  }

  rt.last_event = event;
  rt.last_diag  = diag;
  rt.has_fired  = true;
  rt.event_count++;

  if (dispatch_immediately) {
    maybe_dispatch_event(rt);
  } else {
    timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
  }
}

// ============================================================================
// VCLOCK perishable cadence fact ring
// ============================================================================

static constexpr uint32_t VCLOCK_PERISHABLE_FACT_RING_SIZE = 32;
static constexpr const char* VCLOCK_FACT_DRAIN_NAME = "VCLOCK_FACT_DRAIN";

struct vclock_perishable_fact_t {
  uint32_t sequence = 0;
  uint32_t observed_dwt_at_event = 0;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t observed_low16 = 0;
  bool one_second_due = false;
  bool witness_pps_valid = false;
  pps_t witness_pps{};
  uint32_t fallback_sequence = 0;
};

struct vclock_perishable_ring_t {
  vclock_perishable_fact_t facts[VCLOCK_PERISHABLE_FACT_RING_SIZE]{};
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t count = 0;
  volatile bool drain_armed = false;
  volatile uint32_t sequence = 0;
  volatile uint32_t enqueue_count = 0;
  volatile uint32_t drain_count = 0;
  volatile uint32_t overflow_count = 0;
  volatile uint32_t high_water = 0;
  volatile uint32_t asap_arm_count = 0;
  volatile uint32_t asap_fail_count = 0;
};

static vclock_perishable_ring_t g_vclock_fact_ring = {};
static void vclock_fact_drain_callback(timepop_ctx_t*, timepop_diag_t*, void*);

static void vclock_fact_ring_reset(void) {
  for (uint32_t i = 0; i < VCLOCK_PERISHABLE_FACT_RING_SIZE; i++) {
    g_vclock_fact_ring.facts[i] = vclock_perishable_fact_t{};
  }
  g_vclock_fact_ring.head = 0;
  g_vclock_fact_ring.tail = 0;
  g_vclock_fact_ring.count = 0;
  g_vclock_fact_ring.drain_armed = false;
  g_vclock_fact_ring.sequence = 0;
  g_vclock_fact_ring.enqueue_count = 0;
  g_vclock_fact_ring.drain_count = 0;
  g_vclock_fact_ring.overflow_count = 0;
  g_vclock_fact_ring.high_water = 0;
  g_vclock_fact_ring.asap_arm_count = 0;
  g_vclock_fact_ring.asap_fail_count = 0;
}

static bool vclock_fact_ring_pop(vclock_perishable_fact_t& out) {
  __disable_irq();
  if (g_vclock_fact_ring.count == 0) {
    g_vclock_fact_ring.drain_armed = false;
    __enable_irq();
    return false;
  }

  out = g_vclock_fact_ring.facts[g_vclock_fact_ring.tail];
  g_vclock_fact_ring.tail =
      (g_vclock_fact_ring.tail + 1U) % VCLOCK_PERISHABLE_FACT_RING_SIZE;
  g_vclock_fact_ring.count--;
  g_vclock_fact_ring.drain_count++;
  if (g_vclock_fact_ring.count == 0) {
    g_vclock_fact_ring.drain_armed = false;
  }
  __enable_irq();
  return true;
}

static bool vclock_fact_ring_push_from_isr(vclock_perishable_fact_t fact) {
  if (g_vclock_fact_ring.count >= VCLOCK_PERISHABLE_FACT_RING_SIZE) {
    g_vclock_fact_ring.overflow_count++;
    g_vclock_lane.miss_count++;
    return false;
  }

  fact.sequence = ++g_vclock_fact_ring.sequence;
  g_vclock_fact_ring.facts[g_vclock_fact_ring.head] = fact;
  g_vclock_fact_ring.head =
      (g_vclock_fact_ring.head + 1U) % VCLOCK_PERISHABLE_FACT_RING_SIZE;
  g_vclock_fact_ring.count++;
  g_vclock_fact_ring.enqueue_count++;
  if (g_vclock_fact_ring.count > g_vclock_fact_ring.high_water) {
    g_vclock_fact_ring.high_water = g_vclock_fact_ring.count;
  }

  if (!g_vclock_fact_ring.drain_armed) {
    g_vclock_fact_ring.drain_armed = true;
    g_vclock_fact_ring.asap_arm_count++;
    const timepop_handle_t h =
        timepop_arm_asap(vclock_fact_drain_callback, nullptr,
                         VCLOCK_FACT_DRAIN_NAME);
    if (h == TIMEPOP_INVALID_HANDLE) {
      g_vclock_fact_ring.drain_armed = false;
      g_vclock_fact_ring.asap_fail_count++;
      return false;
    }
  }

  return true;
}

static bool vclock_fact_ring_push_no_arm_from_isr(vclock_perishable_fact_t fact) {
  if (g_vclock_fact_ring.count >= VCLOCK_PERISHABLE_FACT_RING_SIZE) {
    g_vclock_fact_ring.overflow_count++;
    g_vclock_lane.miss_count++;
    return false;
  }

  fact.sequence = ++g_vclock_fact_ring.sequence;
  g_vclock_fact_ring.facts[g_vclock_fact_ring.head] = fact;
  g_vclock_fact_ring.head =
      (g_vclock_fact_ring.head + 1U) % VCLOCK_PERISHABLE_FACT_RING_SIZE;
  g_vclock_fact_ring.count++;
  g_vclock_fact_ring.enqueue_count++;
  if (g_vclock_fact_ring.count > g_vclock_fact_ring.high_water) {
    g_vclock_fact_ring.high_water = g_vclock_fact_ring.count;
  }
  return true;
}

static bool vclock_fact_ring_arm_drain_from_timepop(void) {
  if (g_vclock_fact_ring.count == 0 || g_vclock_fact_ring.drain_armed) return false;

  g_vclock_fact_ring.drain_armed = true;
  g_vclock_fact_ring.asap_arm_count++;
  const timepop_handle_t h =
      timepop_arm_asap(vclock_fact_drain_callback, nullptr,
                       VCLOCK_FACT_DRAIN_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_vclock_fact_ring.drain_armed = false;
    g_vclock_fact_ring.asap_fail_count++;
    return false;
  }

  return true;
}

static void vclock_ch2_fact_drain_arm_after_timepop(void) {
  if (!g_interrupt_runtime_ready) return;

  g_vclock_ch2_fact_drain_service_count++;
  if (vclock_fact_ring_arm_drain_from_timepop()) {
    g_vclock_ch2_fact_drain_arm_count++;
  }
}

static void vclock_ch2_one_second_disable(void) {
  if (g_vclock_ch2_one_second_enabled ||
      g_vclock_ch2_one_second_next_counter32 != 0) {
    g_vclock_ch2_one_second_reset_count++;
  }
  g_vclock_ch2_one_second_enabled = false;
  g_vclock_ch2_one_second_next_counter32 = 0;
}

static void vclock_ch2_one_second_enable_from_epoch_legacy_grid(
    uint32_t selected_counter32,
    uint32_t service_counter32,
    uint32_t tick_mod_seed) {
  // Preserve the last-known-good first-bookend identity while moving the
  // authorship path itself out of VCLOCK_HEARTBEAT.  In the old path, the
  // heartbeat consumed the epoch at service_counter32, seeded tick_mod_1000
  // from selected_to_service_ticks / 10,000, and published the first bookend
  // when that modulo reached 1000.  Seed CH2 to that same counter identity.
  const uint32_t bounded_mod = tick_mod_seed % TICKS_PER_SECOND_EVENT;
  const uint32_t remaining_cells = TICKS_PER_SECOND_EVENT - bounded_mod;
  const uint32_t target_counter32 =
      service_counter32 + remaining_cells * VCLOCK_INTERVAL_COUNTS;
  const uint32_t sacred_target =
      selected_counter32 + (uint32_t)VCLOCK_COUNTS_PER_SECOND;

  g_vclock_ch2_one_second_next_counter32 = target_counter32;
  g_vclock_ch2_one_second_enabled = true;
  g_vclock_ch2_one_second_enable_count++;
  g_vclock_ch2_one_second_epoch_enable_count++;
  g_vclock_ch2_one_second_epoch_base_counter32 = selected_counter32;
  g_vclock_ch2_one_second_epoch_service_counter32 = service_counter32;
  g_vclock_ch2_one_second_epoch_target_counter32 = target_counter32;
  g_vclock_ch2_one_second_epoch_tick_mod_seed = bounded_mod;
  g_vclock_ch2_one_second_epoch_remaining_cells = remaining_cells;
  g_vclock_ch2_one_second_epoch_residual_ticks =
      target_counter32 - sacred_target;
}

static void vclock_ch2_one_second_enable_after_heartbeat(uint32_t last_counter32) {
  // Emergency fallback re-seed only. Normal bootstrap and steady state should
  // be armed from the epoch-compatible CH2 path above.
  g_vclock_ch2_one_second_next_counter32 =
      last_counter32 + (uint32_t)VCLOCK_COUNTS_PER_SECOND;
  g_vclock_ch2_one_second_enabled = true;
  g_vclock_ch2_one_second_enable_count++;
  g_vclock_ch2_one_second_heartbeat_enable_count++;
}

static bool vclock_ch2_one_second_already_served_for(uint32_t counter32) {
  if (!g_vclock_ch2_one_second_enabled) return false;

  // Normal CH2 ownership means the ISR has already advanced the next gear tooth
  // beyond this heartbeat's counter identity before TimePop invokes the callback.
  return (int32_t)(g_vclock_ch2_one_second_next_counter32 - counter32) > 0;
}

static void vclock_ch2_one_second_service(uint32_t qtimer_event_dwt,
                                          uint32_t service_counter32) {
  if (!g_vclock_ch2_one_second_enabled) return;
  if (!g_vclock_lane.active || !g_rt_vclock || !g_rt_vclock->active) return;

  const uint32_t target_counter32 = g_vclock_ch2_one_second_next_counter32;
  if ((int32_t)(service_counter32 - target_counter32) < 0) return;

  const uint32_t late_ticks = service_counter32 - target_counter32;
  uint32_t authored_dwt = qtimer_event_dwt;
  if (late_ticks != 0) {
    g_vclock_ch2_one_second_late_count++;
    if (late_ticks > g_vclock_ch2_one_second_late_max_ticks) {
      g_vclock_ch2_one_second_late_max_ticks = late_ticks;
    }
    authored_dwt -= vclock_cycles_for_ticks(late_ticks);
  }

  const pps_t witness_pps = g_last_pps_witness_valid ? g_last_pps_witness : pps_t{};

  vclock_perishable_fact_t fact{};
  fact.observed_dwt_at_event = authored_dwt;
  fact.target_counter32 = target_counter32;
  fact.target_low16 = vclock_hardware_low16_from_synthetic(target_counter32);
  fact.observed_low16 = (uint16_t)(service_counter32 & 0xFFFFU);
  fact.one_second_due = true;
  fact.witness_pps_valid = g_last_pps_witness_valid;
  fact.witness_pps = witness_pps;
  fact.fallback_sequence = g_pps_gpio_heartbeat.edge_count + 1U;

  g_vclock_ch2_one_second_service_count++;
  g_vclock_ch2_one_second_last_target_counter32 = target_counter32;
  g_vclock_ch2_one_second_last_service_counter32 = service_counter32;
  g_vclock_ch2_one_second_last_dwt = authored_dwt;

  // Passive ISR sanity witness for the public VCLOCK one-second fact.  CNTR
  // sanity verifies the ambient counter observation after known CH0 read
  // latency.  DWT sanity compares consecutive authored one-second DWT intervals
  // against the exact VCLOCK tick interval between their target identities.
  const uint32_t vclock_cntr_expected =
      service_counter32 + ISR_SANITY_VCLOCK_CNTR_READ_LATENCY_TICKS;
  const uint32_t vclock_cntr_actual = g_vclock_clock32.current_counter32;
  isr_sanity_record_cntr(g_isr_sanity_vclock_ch2,
                         true,
                         vclock_cntr_expected,
                         vclock_cntr_actual);

  if (g_isr_sanity_vclock_ch2_prev_valid) {
    const uint32_t expected_dwt_delta = vclock_cycles_for_ticks(
        target_counter32 - g_isr_sanity_vclock_ch2_prev_counter32);
    const uint32_t actual_dwt_delta =
        authored_dwt - g_isr_sanity_vclock_ch2_prev_dwt;
    isr_sanity_record_dwt(g_isr_sanity_vclock_ch2,
                          true,
                          expected_dwt_delta,
                          actual_dwt_delta);
  }
  g_isr_sanity_vclock_ch2_prev_valid = true;
  g_isr_sanity_vclock_ch2_prev_counter32 = target_counter32;
  g_isr_sanity_vclock_ch2_prev_dwt = authored_dwt;

  if (vclock_fact_ring_push_no_arm_from_isr(fact)) {
    g_vclock_ch2_one_second_enqueue_count++;
  } else {
    // Leave the target armed so the heartbeat fallback path can still publish
    // the one-second event rather than silently losing a public bookend.
    g_vclock_ch2_one_second_drop_count++;
    return;
  }

  // Advance as a gear, not as a service-time rubber band.  If service is ever
  // more than one second late, skip stale teeth diagnostically instead of
  // trying to flood the drain with catch-up publications.
  uint32_t next = target_counter32 + (uint32_t)VCLOCK_COUNTS_PER_SECOND;
  while ((int32_t)(service_counter32 - next) >= 0) {
    next += (uint32_t)VCLOCK_COUNTS_PER_SECOND;
    g_vclock_ch2_one_second_catchup_count++;
    g_vclock_ch2_one_second_drop_count++;
  }
  g_vclock_ch2_one_second_next_counter32 = next;
}

static void vclock_ch2_smartzero_reset_attempt_state(void) {
  g_vclock_ch2_smartzero_seeded = false;
  g_vclock_ch2_smartzero_next_counter32 = 0;
  g_vclock_ch2_smartzero_reset_count++;
}

static void vclock_ch2_smartzero_deactivate(void) {
  if (g_vclock_ch2_smartzero_seeded || g_vclock_ch2_smartzero_next_counter32 != 0) {
    g_vclock_ch2_smartzero_deactivate_count++;
  }
  g_vclock_ch2_smartzero_seeded = false;
  g_vclock_ch2_smartzero_next_counter32 = 0;
}

static void vclock_ch2_smartzero_record_decision(interrupt_smartzero_decision_t d) {
  switch (d) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:
      g_vclock_ch2_smartzero_waiting_for_cps_count++;
      break;
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:
      g_vclock_ch2_smartzero_first_sample_count++;
      break;
    case interrupt_smartzero_decision_t::ACCEPTED:
      g_vclock_ch2_smartzero_accepted_count++;
      break;
    case interrupt_smartzero_decision_t::REJECTED_DWT:
      g_vclock_ch2_smartzero_rejected_dwt_count++;
      break;
    case interrupt_smartzero_decision_t::REJECTED_COUNTER:
      g_vclock_ch2_smartzero_rejected_counter_count++;
      break;
    default:
      break;
  }
}

static void vclock_ch2_smartzero_service(uint32_t qtimer_event_dwt,
                                         uint32_t service_counter32) {
  if (!VCLOCK_CH2_SMARTZERO_NATIVE_ENABLED) return;

  if ((g_smartzero.seq & 1U) != 0) {
    g_vclock_ch2_smartzero_transaction_skip_count++;
    return;
  }

  if (!smartzero_is_current_lane(interrupt_subscriber_kind_t::VCLOCK)) {
    vclock_ch2_smartzero_deactivate();
    return;
  }

  g_vclock_ch2_smartzero_service_count++;

  uint32_t target_counter32 = g_vclock_ch2_smartzero_next_counter32;
  if (!g_vclock_ch2_smartzero_seeded) {
    target_counter32 = service_counter32;
    g_vclock_ch2_smartzero_seeded = true;
    g_vclock_ch2_smartzero_seed_count++;
  } else if ((int32_t)(service_counter32 - target_counter32) < 0) {
    return;
  }

  const uint32_t late_ticks = service_counter32 - target_counter32;
  if (late_ticks >= SMARTZERO_INTERVAL_TICKS) {
    // Do not feed a stale tooth that would force SmartZero to observe a
    // non-adjacent counter delta. Re-seed on the next CH2 event instead.
    g_vclock_ch2_smartzero_resync_count++;
    g_vclock_ch2_smartzero_drop_count++;
    g_vclock_ch2_smartzero_seeded = false;
    g_vclock_ch2_smartzero_next_counter32 = 0;
    return;
  }

  uint32_t authored_dwt = qtimer_event_dwt;
  if (late_ticks != 0) {
    g_vclock_ch2_smartzero_late_count++;
    if (late_ticks > g_vclock_ch2_smartzero_late_max_ticks) {
      g_vclock_ch2_smartzero_late_max_ticks = late_ticks;
    }
    authored_dwt -= vclock_cycles_for_ticks(late_ticks);
  }

  const uint16_t target_low16 = vclock_hardware_low16_from_synthetic(target_counter32);
  g_vclock_ch2_smartzero_last_target_counter32 = target_counter32;
  g_vclock_ch2_smartzero_last_service_counter32 = service_counter32;
  g_vclock_ch2_smartzero_last_dwt = authored_dwt;
  g_vclock_ch2_smartzero_last_late_ticks = late_ticks;

  g_vclock_ch2_smartzero_feed_count++;
  (void)smartzero_feed_sample(interrupt_subscriber_kind_t::VCLOCK,
                              authored_dwt,
                              target_counter32,
                              target_low16);

  const int idx = smartzero_index_for_kind(interrupt_subscriber_kind_t::VCLOCK);
  if (idx >= 0) {
    vclock_ch2_smartzero_record_decision(g_smartzero.lanes[idx].pub.last_decision);
  }

  if (!smartzero_is_current_lane(interrupt_subscriber_kind_t::VCLOCK)) {
    vclock_ch2_smartzero_deactivate();
    return;
  }

  g_vclock_ch2_smartzero_next_counter32 =
      target_counter32 + SMARTZERO_INTERVAL_TICKS;
}

static void vclock_ch2_epoch_native_service(uint32_t qtimer_event_dwt,
                                             uint32_t service_counter32) {
  if (!VCLOCK_CH2_EPOCH_NATIVE_ENABLED) return;
  if (!g_interrupt_runtime_ready || !g_interrupt_hw_ready) return;
  if (!g_vclock_epoch_latch.pending) return;

  g_vclock_ch2_epoch_native_service_count++;

  const uint32_t selected_counter32 = g_vclock_epoch_latch.counter32_at_edge;
  const uint32_t selected_to_service_ticks = service_counter32 - selected_counter32;
  g_vclock_ch2_epoch_native_last_selected_counter32 = selected_counter32;
  g_vclock_ch2_epoch_native_last_service_counter32 = service_counter32;
  g_vclock_ch2_epoch_native_last_backdate_ticks = selected_to_service_ticks;

  const bool published =
      publish_selected_epoch_from_vclock_cadence(service_counter32,
                                                 qtimer_event_dwt);
  if (!published) return;

  // Preserve the old heartbeat phase semantics, but author them from the same
  // native CH2 service fact that published the selected epoch.  The next
  // heartbeat will increment from this millisecond-cell position; it will not
  // decide the epoch itself.
  const uint32_t tick_mod_seed =
      (selected_to_service_ticks / VCLOCK_INTERVAL_COUNTS) %
      TICKS_PER_SECOND_EVENT;
  g_vclock_lane.tick_mod_1000 = tick_mod_seed;
  g_vclock_lane.logical_count32_at_last_second = service_counter32;
  g_vclock_lane.compare_target =
      (uint16_t)(g_vclock_epoch_latch.ch3_at_edge +
                 (uint16_t)VCLOCK_INTERVAL_COUNTS);

  // Move the first post-epoch publication path to CH2 without changing the
  // first public bookend identity from the last-known-good build.  The first
  // exact selected+1s target remains a future goal, but the previous attempt
  // changed boot ordering enough to clobber command service.
  vclock_ch2_one_second_enable_from_epoch_legacy_grid(selected_counter32,
                                                       service_counter32,
                                                       tick_mod_seed);
}

static void vclock_apply_perishable_fact_deferred(
    const vclock_perishable_fact_t& fact) {
  if (!fact.one_second_due || !g_rt_vclock || !g_rt_vclock->active) {
    return;
  }

  const uint32_t inferred_dwt = fact.observed_dwt_at_event;

  uint32_t coincidence_cycles = 0;
  bool coincidence_valid = false;
  if (fact.witness_pps_valid && fact.witness_pps.sequence > 0) {
    const int32_t cycles_since_pps =
        (int32_t)(inferred_dwt - fact.witness_pps.dwt_at_edge);
    if (llabs((long long)cycles_since_pps) <
        DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
      coincidence_cycles = (uint32_t)(cycles_since_pps >= 0
          ? cycles_since_pps
          : -cycles_since_pps);
      coincidence_valid = true;
    }
  }

  const dwt_repair_diag_t repair =
      vclock_endpoint_repair_diagnostic(fact.observed_dwt_at_event);

  const pps_t witness_pps = fact.witness_pps_valid ? fact.witness_pps : pps_t{};
  publish_vclock_domain_pps_vclock(witness_pps,
                                    (witness_pps.sequence != 0)
                                        ? witness_pps.sequence
                                        : fact.fallback_sequence,
                                    inferred_dwt,
                                    fact.target_counter32,
                                    fact.target_low16);

  emit_one_second_event(*g_rt_vclock,
                        inferred_dwt,
                        fact.target_counter32,
                        coincidence_cycles,
                        coincidence_valid,
                        &repair,
                        true,
                        nullptr);

  if (g_pps_edge_dispatch) {
    timepop_arm_asap(pps_edge_dispatch_trampoline,
                     nullptr,
                     "PPS_VCLOCK_DISPATCH");
  }
}

static void vclock_fact_drain_callback(timepop_ctx_t*,
                                       timepop_diag_t*,
                                       void*) {
  for (;;) {
    vclock_perishable_fact_t fact{};
    if (!vclock_fact_ring_pop(fact)) break;
    vclock_apply_perishable_fact_deferred(fact);
  }
}


// ============================================================================
// Cadence surfaces — VCLOCK TimePop heartbeat + OCXO lane-local compare custody
// ============================================================================

static const char* ocxo_schedule_decision_name(uint32_t decision) {
  switch (decision) {
    case OCXO_SCHEDULE_DECISION_INACTIVE: return "INACTIVE";
    case OCXO_SCHEDULE_DECISION_ALREADY_ARMED: return "ALREADY_ARMED";
    case OCXO_SCHEDULE_DECISION_TARGET_INITIALIZED: return "TARGET_INITIALIZED";
    case OCXO_SCHEDULE_DECISION_TARGET_ADVANCED: return "TARGET_ADVANCED";
    case OCXO_SCHEDULE_DECISION_TOO_CLOSE: return "TOO_CLOSE";
    case OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW: return "OUTSIDE_WINDOW";
    case OCXO_SCHEDULE_DECISION_ARMED: return "ARMED";
    default: return "NONE";
  }
}

static const char* ocxo_service_class_name(uint32_t service_class) {
  switch (service_class) {
    case OCXO_SERVICE_CLASS_FALSE_IRQ: return "FALSE_IRQ";
    case OCXO_SERVICE_CLASS_EARLY_PRETARGET: return "EARLY_PRETARGET";
    case OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET: return "ON_OR_AFTER_TARGET";
    default: return "NONE";
  }
}

static const char* ocxo_cadence_reason_name(uint32_t reason) {
  switch (reason) {
    case OCXO_CADENCE_REASON_BOOTSTRAP:    return "BOOTSTRAP";
    case OCXO_CADENCE_REASON_START:        return "START";
    case OCXO_CADENCE_REASON_SMARTZERO:    return "SMARTZERO";
    case OCXO_CADENCE_REASON_LOGICAL_GRID: return "LOGICAL_GRID";
    case OCXO_CADENCE_REASON_LOGICAL_ZERO: return "LOGICAL_ZERO";
    case OCXO_CADENCE_REASON_REARM:        return "REARM";
    case OCXO_CADENCE_REASON_STOP:         return "STOP";
    default:                               return "NONE";
  }
}

static uint32_t ocxo_quiet_phase_ticks_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return OCXO1_QUIET_PHASE_TICKS;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return OCXO2_QUIET_PHASE_TICKS;
  return 0U;
}

static uint32_t ocxo_phase_ticks_to_us(uint32_t phase_ticks) {
  return (uint32_t)(((uint64_t)phase_ticks * 100ULL + 500ULL) / 1000ULL);
}

static uint32_t ocxo_phase_ticks_to_ns(uint32_t phase_ticks) {
  return phase_ticks * 100U;
}

static uint32_t ocxo_quiet_phase_next_target(interrupt_subscriber_kind_t kind,
                                             ocxo_lane_t& lane,
                                             const synthetic_clock32_t& clock32) {
  const uint32_t desired_phase = ocxo_quiet_phase_ticks_for(kind);
  const uint32_t vclock_now = interrupt_vclock_counter32_observe_ambient();
  const uint32_t vclock_phase = vclock_now % OCXO_QUIET_PHASE_PERIOD_TICKS;

  uint32_t ticks_until =
      (desired_phase + OCXO_QUIET_PHASE_PERIOD_TICKS - vclock_phase) %
      OCXO_QUIET_PHASE_PERIOD_TICKS;

  // A phase exactly "now" or too close to program safely belongs to the next
  // millisecond cell.  This keeps first arming out of the hardware race window.
  if (ticks_until < OCXO_QUIET_PHASE_MIN_LEAD_TICKS) {
    ticks_until += OCXO_QUIET_PHASE_PERIOD_TICKS;
  }

  lane.cadence_sample_phase_valid = true;
  lane.cadence_sample_phase_ticks = desired_phase;
  lane.cadence_sample_phase_us = ocxo_phase_ticks_to_us(desired_phase);
  lane.cadence_sample_phase_ns = ocxo_phase_ticks_to_ns(desired_phase);
  lane.cadence_sample_period_ticks = OCXO_QUIET_PHASE_PERIOD_TICKS;
  lane.cadence_phase_align_start_count++;
  lane.cadence_last_phase_align_vclock_counter32 = vclock_now;
  lane.cadence_last_phase_align_vclock_phase_ticks = vclock_phase;
  lane.cadence_last_phase_align_ticks_until_target = ticks_until;
  lane.cadence_last_phase_align_ocxo_counter32 = clock32.current_counter32;

  return clock32.current_counter32 + ticks_until;
}

static uint32_t ocxo_grid_next_target_after(uint32_t epoch_counter32,
                                            uint32_t current_counter32) {
  const uint32_t delta = current_counter32 - epoch_counter32;

  // If the epoch appears to be in the modular future, use the first cadence
  // target after the epoch.  This can only happen across a 32-bit wrap or when
  // a caller deliberately re-authors a future logical grid.
  if (delta > 0x7FFFFFFFUL) {
    return epoch_counter32 + OCXO_CADENCE_INTERVAL_TICKS;
  }

  const uint32_t steps_completed = delta / OCXO_CADENCE_INTERVAL_TICKS;
  return epoch_counter32 + ((steps_completed + 1U) * OCXO_CADENCE_INTERVAL_TICKS);
}

static void ocxo_lane_program_local_cadence_compare(ocxo_lane_t& lane,
                                                    uint32_t target_counter32,
                                                    uint32_t reason) {
  const uint16_t target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);

  lane.cadence_last_program_csctrl_before = lane.module->CH[lane.channel].CSCTRL;
  lane.cadence_last_program_flag_before =
      (lane.cadence_last_program_csctrl_before & TMR_CSCTRL_TCF1) != 0;
  lane.cadence_last_program_enabled_before =
      (lane.cadence_last_program_csctrl_before & TMR_CSCTRL_TCF1EN) != 0;

  lane.cadence_next_counter32 = target_counter32;
  lane.cadence_next_low16 = target_low16;
  lane.compare_target = target_low16;
  lane.witness_armed = true;
  lane.witness_target_initialized = true;
  lane.witness_target_counter32 = target_counter32;
  lane.witness_target_low16 = target_low16;

  ocxo_lane_program_compare(lane, target_low16);

  lane.cadence_last_program_csctrl_after = lane.module->CH[lane.channel].CSCTRL;
  lane.cadence_last_program_flag_after =
      (lane.cadence_last_program_csctrl_after & TMR_CSCTRL_TCF1) != 0;
  lane.cadence_last_program_enabled_after =
      (lane.cadence_last_program_csctrl_after & TMR_CSCTRL_TCF1EN) != 0;
  lane.cadence_armed = true;
  lane.cadence_arm_count++;
  if (reason == OCXO_CADENCE_REASON_REARM) {
    lane.cadence_rearm_count++;
  }
  lane.cadence_last_reason = reason;
}

static bool ocxo_lane_start_local_cadence_at_target(interrupt_subscriber_kind_t,
                                                    ocxo_lane_t& lane,
                                                    synthetic_clock32_t& clock32,
                                                    uint32_t target_counter32,
                                                    uint32_t reason) {
  if (!lane.initialized) return false;

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);

  lane.cadence_enabled = true;
  lane.cadence_last_reason = reason;
  lane.witness_last_arm_dwt_raw = ARM_DWT_CYCCNT;
  lane.witness_last_arm_counter32 = clock32.current_counter32;
  lane.witness_last_arm_low16 = hw16;
  lane.witness_last_arm_target_counter32 = target_counter32;
  lane.witness_last_arm_target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);
  lane.witness_last_arm_remaining_ticks = target_counter32 - clock32.current_counter32;
  lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_ARMED;
  lane.witness_schedule_last_current_counter32 = clock32.current_counter32;
  lane.witness_schedule_last_target_counter32 = target_counter32;
  lane.witness_schedule_last_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  lane.witness_schedule_last_current_low16 = hw16;
  lane.witness_schedule_last_target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);
  lane.witness_schedule_last_phase_ticks =
      lane.cadence_epoch_valid ? (clock32.current_counter32 - lane.cadence_epoch_counter32) : 0U;
  lane.witness_schedule_last_ticks_until_arm_window = 0;

  ocxo_lane_program_local_cadence_compare(lane, target_counter32, reason);
  return true;
}

static uint32_t ocxo_one_second_next_target_after(
    uint32_t epoch_counter32,
    uint32_t current_counter32) {
  const uint32_t delta = current_counter32 - epoch_counter32;
  if (delta > 0x7FFFFFFFUL) {
    return epoch_counter32 + OCXO_WITNESS_ONE_SECOND_COUNTS;
  }
  const uint32_t seconds_completed = delta / OCXO_WITNESS_ONE_SECOND_COUNTS;
  return epoch_counter32 + ((seconds_completed + 1U) * OCXO_WITNESS_ONE_SECOND_COUNTS);
}

static void ocxo_lane_prepare_one_second_target(
    interrupt_subscriber_kind_t kind,
    ocxo_lane_t& lane,
    synthetic_clock32_t& clock32,
    uint32_t target_counter32,
    uint32_t reason) {
  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);

  lane.cadence_enabled = true;
  lane.cadence_armed = false;
  lane.witness_armed = false;
  lane.cadence_next_counter32 = target_counter32;
  lane.cadence_next_low16 = (uint16_t)(target_counter32 & 0xFFFFU);
  lane.compare_target = lane.cadence_next_low16;
  lane.cadence_last_reason = reason;
  lane.witness_target_initialized = true;
  lane.witness_target_counter32 = target_counter32;
  lane.witness_target_low16 = lane.cadence_next_low16;
  lane.witness_last_arm_counter32 = clock32.current_counter32;
  lane.witness_last_arm_low16 = hw16;
  lane.witness_last_arm_target_counter32 = target_counter32;
  lane.witness_last_arm_target_low16 = lane.cadence_next_low16;
  lane.witness_last_arm_remaining_ticks = target_counter32 - clock32.current_counter32;
  lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW;
  lane.witness_schedule_last_current_counter32 = clock32.current_counter32;
  lane.witness_schedule_last_target_counter32 = target_counter32;
  lane.witness_schedule_last_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  lane.witness_schedule_last_current_low16 = hw16;
  lane.witness_schedule_last_target_low16 = lane.cadence_next_low16;
  lane.witness_schedule_last_phase_ticks =
      lane.cadence_epoch_valid ? (clock32.current_counter32 - lane.cadence_epoch_counter32) : 0U;
  lane.witness_schedule_last_ticks_until_arm_window =
      (lane.witness_last_arm_remaining_ticks > OCXO_WITNESS_ARM_WINDOW_TICKS)
          ? (lane.witness_last_arm_remaining_ticks - OCXO_WITNESS_ARM_WINDOW_TICKS)
          : 0U;

  // Do not program the 16-bit OCXO compare yet unless the target is close
  // enough to be unambiguous.  CH2 implicit rollover tending will call
  // ocxo_lane_maybe_arm_one_second_compare() until the one-second edge enters
  // the safe arm window.
  ocxo_lane_disable_compare(lane);
  ocxo_lane_maybe_arm_one_second_compare(kind, lane, clock32, reason);
}

static bool ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t kind,
                                          ocxo_lane_t& lane,
                                          synthetic_clock32_t& clock32,
                                          uint32_t reason) {
  if (!lane.initialized) return false;

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);

  uint32_t target = 0;
  if (lane.cadence_epoch_valid) {
    target = ocxo_one_second_next_target_after(lane.cadence_epoch_counter32,
                                               clock32.current_counter32);
  } else {
    // Pre-logical-grid fallback: produce one one-second edge from the current
    // ambient OCXO coordinate.  CLOCKS normally installs the logical grid
    // immediately after SmartZero, so this should be transient.
    target = clock32.current_counter32 + OCXO_WITNESS_ONE_SECOND_COUNTS;
  }

  if (OCXO_QUIET_PHASE_SAMPLING_ENABLED &&
      (kind == interrupt_subscriber_kind_t::OCXO1 ||
       kind == interrupt_subscriber_kind_t::OCXO2)) {
    // Quiet phase still describes where the one-second edge family should land
    // inside the VCLOCK millisecond cell.  It no longer implies a native OCXO
    // 1 kHz compare ladder in steady state.
    lane.cadence_sample_phase_valid = true;
    lane.cadence_sample_phase_ticks = ocxo_quiet_phase_ticks_for(kind);
    lane.cadence_sample_phase_us = ocxo_phase_ticks_to_us(lane.cadence_sample_phase_ticks);
    lane.cadence_sample_phase_ns = ocxo_phase_ticks_to_ns(lane.cadence_sample_phase_ticks);
    lane.cadence_sample_period_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
  } else {
    lane.cadence_sample_phase_valid = false;
    lane.cadence_sample_phase_ticks = 0;
    lane.cadence_sample_phase_us = 0;
    lane.cadence_sample_phase_ns = 0;
    lane.cadence_sample_period_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
  }

  if (!lane.phase_bootstrapped) {
    lane.phase_bootstrapped = true;
    lane.bootstrap_count++;
  }

  lane.tick_mod_1000 = lane.cadence_epoch_valid
      ? ((target - lane.cadence_epoch_counter32) / OCXO_CADENCE_INTERVAL_TICKS) %
            TICKS_PER_SECOND_EVENT
      : 0U;

  ocxo_lane_prepare_one_second_target(kind, lane, clock32, target, reason);
  return lane.cadence_enabled;
}

static void ocxo_lane_ema_reset(ocxo_lane_t& lane) {
  lane.ema_initialized = false;
  lane.ema_interval_valid = false;
  lane.ema_last_observed_dwt = 0;
  lane.ema_last_emitted_dwt = 0;
  lane.ema_last_observed_interval_cycles = 0;
  lane.ema_interval_cycles = 0;
  lane.ema_last_predicted_dwt = 0;
  lane.ema_last_error_cycles = 0;
  lane.ema_max_abs_error_cycles = 0;
  lane.ema_update_count = 0;
}

static void ocxo_lane_stop_local_cadence(ocxo_lane_t& lane, uint32_t reason) {
  lane.cadence_enabled = false;
  lane.cadence_armed = false;
  lane.witness_armed = false;
  lane.cadence_last_reason = reason;
  ocxo_lane_disable_compare(lane);
}

static void ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t kind,
                                           ocxo_lane_t& lane,
                                           synthetic_clock32_t& clock32,
                                           uint32_t epoch_counter32,
                                           uint32_t reason) {
  lane.cadence_epoch_valid = true;
  lane.cadence_epoch_counter32 = epoch_counter32;
  lane.tick_mod_1000 = 0;
  ocxo_lane_ema_reset(lane);
  lane.witness_previous_event_counter32_valid = false;
  lane.witness_previous_event_counter32 = 0;
  lane.witness_last_counter_delta_ticks = 0;
  cadence_regression_reset_kind(kind);

  if (lane.active || smartzero_is_current_lane(kind)) {
    (void)ocxo_lane_start_local_cadence(kind, lane, clock32, reason);
  }
}

// The old VCLOCK_HEARTBEAT-driven OCXO arming path has been retired.  OCXO1 and
// OCXO2 now maintain their own one-second compare custody on their local QTimer
// channels.  Relay deassertion, bootstrap/rebootstrap selected-epoch
// publication, steady-state VCLOCK one-second bookends, VCLOCK SmartZero
// sampling, and VCLOCK fact-drain arming now run from intrinsic CH2.
// VCLOCK_HEARTBEAT remains only the fallback and reporting heartbeat.

static void vclock_heartbeat_timepop_callback(timepop_ctx_t* ctx,
                                            timepop_diag_t*,
                                            void*) {
  if (!ctx || !g_interrupt_hw_ready) return;

  const uint32_t qtimer_event_dwt = ctx->fire_dwt_cyccnt;
  const uint32_t cadence_counter32 = ctx->fire_vclock_raw;
  const uint16_t fired_low16 = (uint16_t)(cadence_counter32 & 0xFFFFU);

  g_vclock_heartbeat_fire_count++;

  // VCLOCK_HEARTBEAT is a scheduled housekeeping/event-authoring client,
  // not a rollover owner and no longer the steady-state VCLOCK one-second
  // authority.  Low-word rollover extension is handled only by
  // interrupt_ch2_implicit_rollover_tend(), which runs before TimePop gets
  // this CH2 event.  Keep the heartbeat's authored counter identity for
  // bootstrap/failsafe bookends and reports, but do not refresh the
  // synthetic clock32 extender here.  Relay deassert countdown, steady-state
  // VCLOCK one-second authorship, VCLOCK SmartZero sampling, bootstrap/
  // rebootstrap selected-epoch publication, and fact-drain arming have moved
  // to intrinsic/native CH2 service.
  g_vclock_lane.logical_count32_at_last_second = cadence_counter32;
  g_vclock_heartbeat_vclock_ticks++;
  g_vclock_heartbeat_last_vclock_hw16 = fired_low16;
  g_vclock_heartbeat_last_vclock_counter32 = cadence_counter32;

  // VCLOCK SmartZero sampling has moved to intrinsic CH2 fixed-grid service.
  // Keep a retired-authority counter so reports can prove the heartbeat saw
  // acquisition opportunities without feeding them.
  if (smartzero_is_current_lane(interrupt_subscriber_kind_t::VCLOCK)) {
    g_vclock_heartbeat_smartzero_authority_retired_count++;
  }

  // CH2 now arms the VCLOCK fact drain after TimePop's CH2 handler returns.
  // If a pending fact is still visible here, leave it alone and count the
  // observation; heartbeat is no longer the drain-arm authority.
  if (g_vclock_fact_ring.count != 0) {
    g_vclock_heartbeat_fact_drain_authority_retired_count++;
  }

  if (g_rt_vclock) {
    g_rt_vclock->irq_count++;
  }
  g_vclock_lane.irq_count++;

  if (g_vclock_lane.active && g_rt_vclock && g_rt_vclock->active) {
    if (!g_vclock_lane.phase_bootstrapped) {
      g_vclock_lane.phase_bootstrapped = true;
      g_vclock_lane.bootstrap_count++;
      g_vclock_lane.tick_mod_1000 = 0;
    }

    g_vclock_lane.cadence_hits_total++;

    if (g_vclock_epoch_latch.pending) {
      // Native CH2 tail now publishes bootstrap/rebootstrap selected epochs.
      // The heartbeat deliberately does not consume the latch or advance the
      // one-second modulo while the selected epoch is pending; the CH2 service
      // that publishes the epoch will seed tick_mod_1000 from its own event
      // facts after TimePop returns.
      g_vclock_heartbeat_epoch_authority_retired_count++;
      g_vclock_heartbeat_epoch_pending_skip_count++;
      return;
    }

    if (++g_vclock_lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
      g_vclock_lane.tick_mod_1000 = 0;

      if (vclock_ch2_one_second_already_served_for(cadence_counter32)) {
        // Normal migrated path: native CH2 authored and enqueued the one-second
        // fact before TimePop invoked this heartbeat callback.
        g_vclock_heartbeat_one_second_handoff_skip_count++;
      } else {
        // Bootstrap/failsafe path.  The first one-second bookend after startup,
        // or any unexpected CH2 handoff miss, uses the exact legacy heartbeat
        // publication path.  That preserves campaign-start and ZERO behavior.
        if (g_vclock_ch2_one_second_enabled) {
          g_vclock_heartbeat_one_second_fallback_count++;
        } else {
          g_vclock_heartbeat_one_second_legacy_count++;
        }

        // PPS_VCLOCK / PPS coincidence diagnostic.
        uint32_t coincidence_cycles = 0;
        bool     coincidence_valid  = false;
        {
          const pps_vclock_t pvc = store_load_pvc();
          if (pvc.sequence > 0) {
            const int32_t cycles_since_pvc =
                (int32_t)(qtimer_event_dwt - pvc.dwt_at_edge);
            if (llabs((long long)cycles_since_pvc) <
                DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
              coincidence_cycles =
                  (uint32_t)(cycles_since_pvc >= 0 ? cycles_since_pvc
                                                   : -cycles_since_pvc);
              coincidence_valid = true;
            }
          }
        }

        const pps_t witness_pps = g_last_pps_witness_valid
            ? g_last_pps_witness
            : pps_t{};
        const dwt_repair_diag_t repair =
            vclock_endpoint_repair_diagnostic(qtimer_event_dwt);
        publish_vclock_domain_pps_vclock(witness_pps,
                                          (witness_pps.sequence != 0)
                                              ? witness_pps.sequence
                                              : (g_pps_gpio_heartbeat.edge_count + 1),
                                          qtimer_event_dwt,
                                          cadence_counter32,
                                          fired_low16);

        emit_one_second_event(*g_rt_vclock, qtimer_event_dwt,
                              cadence_counter32,
                              coincidence_cycles, coincidence_valid, &repair);

        if (g_pps_edge_dispatch) {
          timepop_arm_asap(pps_edge_dispatch_trampoline,
                           nullptr,
                           "PPS_VCLOCK_DISPATCH");
        }

        vclock_ch2_one_second_enable_after_heartbeat(cadence_counter32);
      }
    }
  } else {
    g_vclock_lane.miss_count++;
  }

  // OCXO lanes are intentionally absent here.  Their 16-bit rollover cadence is
  // now device-local on QTimer2/QTimer3; this TimePop callback is now only the
  // VCLOCK fallback/reporting heartbeat surface.
}

static bool vclock_heartbeat_arm_timepop(void) {
  if (!g_interrupt_hw_ready || !g_interrupt_runtime_ready) return false;

  timepop_cancel_by_name(VCLOCK_HEARTBEAT_NAME);
  const timepop_handle_t h =
      timepop_arm_recurring_isr_with_priority(VCLOCK_HEARTBEAT_PERIOD_NS,
                                              vclock_heartbeat_timepop_callback,
                                              nullptr,
                                              VCLOCK_HEARTBEAT_NAME,
                                              TIMEPOP_PRIORITY_FIRST);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_vclock_heartbeat_armed = false;
    g_vclock_heartbeat_arm_failures++;
    return false;
  }

  g_vclock_heartbeat_armed = true;
  g_vclock_heartbeat_arm_count++;
  g_vclock_lane.phase_bootstrapped = true;
  g_vclock_lane.bootstrap_count++;
  return true;
}

// ============================================================================
// OCXO lanes — QTimer2 CH0 / QTimer3 CH3
// ============================================================================
//
static void ocxo_lane_program_compare(ocxo_lane_t& lane, uint16_t target_low16) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_program_compare(target_low16);
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_program_compare(lane.channel, target_low16);
  }
}

static void ocxo_lane_disable_compare(ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_disable_compare();
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_disable_compare(lane.channel);
  }
}

// ISR discipline for OCXO lanes is intentionally flat and tiny:
//   1. capture first-instruction DWT,
//   2. clear the compare flag,
//   3. compute the latency-adjusted event DWT,
//   4. re-arm the next 1 kHz compare,
//   5. refresh the synthetic 32-bit lane anchor,
//   6. enqueue the perishable fact for foreground interpretation.
//
// Dynamic 100 Hz prediction has been retired. The old single-slot OCXO
// deferred edge mailbox is gone; the one-second fact ring is the sole OCXO
// ISR-to-foreground handoff.
// ============================================================================

// ----------------------------------------------------------------------------
// OCXO one-second fact ring — ISR capture, ASAP interpretation.
// ----------------------------------------------------------------------------
//
// The former build enqueued every 1 kHz OCXO cadence sample and asked
// foreground TimePop ASAP to drain/interpret them. That proved too expensive.
// The rollover-only build keeps the 1 kHz hardware ladder but enqueues only
// the 1000th sample that represents an OCXO one-second event. Intermediate
// cadence ticks rearm the local compare and update small lane counters only.
//
// The ring remains per-lane and loss-visible, but its normal rate is now 1 Hz.

static constexpr uint32_t OCXO_PERISHABLE_FACT_RING_SIZE = 32;

struct interrupt_perishable_fact_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  uint32_t sequence = 0;

  // Captured as first ISR instruction.  This remains private custody evidence;
  // published event DWT remains latency-adjusted service_dwt_at_event.
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t service_dwt_at_event = 0;

  // Authored target identity.  This is the event identity, not an ambient read.
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;

  // Hardware observation at ISR service time.
  uint16_t service_counter_low16 = 0;
  int16_t service_offset_ticks = 0;
  uint32_t service_offset_abs_ticks = 0;
  uint32_t interpreted_late_ticks = 0;
  uint32_t early_ticks = 0;
  uint32_t target_delta_mod65536_ticks = 0;

  // Arming/service forensics captured before interpretation.
  uint32_t arm_remaining_ticks = 0;
  uint32_t arm_to_isr_ticks = 0;
  uint32_t arm_to_isr_dwt_cycles = 0;
  uint32_t csctrl_entry = 0;
  uint32_t csctrl_after_disable = 0;
  bool compare_flag_seen = false;
  bool compare_enabled_entry = false;
  bool compare_flag_after_disable = false;
  bool compare_enabled_after_disable = false;
  bool had_armed = false;
  bool had_active_rt = false;

  uint32_t service_class = OCXO_SERVICE_CLASS_NONE;
  bool one_second_due = false;
  bool exact_target_identity = true;

  bool sample_phase_valid = false;
  uint32_t sample_phase_ticks = 0;
  uint32_t sample_phase_us = 0;
  uint32_t sample_phase_ns = 0;
  uint32_t sample_period_ticks = 0;
};

struct ocxo_perishable_ring_t {
  interrupt_perishable_fact_t facts[OCXO_PERISHABLE_FACT_RING_SIZE]{};
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t count = 0;
  volatile bool drain_armed = false;
  volatile uint32_t sequence = 0;
  volatile uint32_t enqueue_count = 0;
  volatile uint32_t drain_count = 0;
  volatile uint32_t overflow_count = 0;
  volatile uint32_t high_water = 0;
  volatile uint32_t asap_arm_count = 0;
  volatile uint32_t asap_fail_count = 0;
};

static ocxo_perishable_ring_t g_ocxo1_fact_ring = {};
static ocxo_perishable_ring_t g_ocxo2_fact_ring = {};

static ocxo_perishable_ring_t&
ocxo_fact_ring_for(ocxo_runtime_context_t& ctx) {
  return (ctx.kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_fact_ring
      : g_ocxo2_fact_ring;
}

static const ocxo_perishable_ring_t&
ocxo_fact_ring_for(const ocxo_runtime_context_t& ctx) {
  return (ctx.kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_fact_ring
      : g_ocxo2_fact_ring;
}

static const char* ocxo_fact_drain_name(const ocxo_runtime_context_t& ctx) {
  return ctx.fact_drain_name ? ctx.fact_drain_name : ctx.name;
}

static void ocxo_fact_ring_reset(ocxo_runtime_context_t& ctx) {
  ocxo_perishable_ring_t& r = ocxo_fact_ring_for(ctx);
  for (uint32_t i = 0; i < OCXO_PERISHABLE_FACT_RING_SIZE; i++) {
    r.facts[i] = interrupt_perishable_fact_t{};
  }
  r.head = 0;
  r.tail = 0;
  r.count = 0;
  r.drain_armed = false;
  r.sequence = 0;
  r.enqueue_count = 0;
  r.drain_count = 0;
  r.overflow_count = 0;
  r.high_water = 0;
  r.asap_arm_count = 0;
  r.asap_fail_count = 0;
}

static int32_t dwt_cycles_for_ocxo_ticks_signed(int32_t ticks) {
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  if (cps == 0 || ticks == 0) return 0;

  const int64_t numerator = (int64_t)ticks * (int64_t)cps;
  const int64_t denom = (int64_t)OCXO_WITNESS_ONE_SECOND_COUNTS;
  if (numerator >= 0) {
    return (int32_t)((numerator + denom / 2) / denom);
  }
  return (int32_t)(-(((-numerator) + denom / 2) / denom));
}

static bool ocxo_fact_ring_push_from_isr(ocxo_runtime_context_t& ctx,
                                         const interrupt_perishable_fact_t& fact);
static void ocxo_fact_drain_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data);

static bool ocxo_fact_ring_pop(ocxo_runtime_context_t& ctx,
                               interrupt_perishable_fact_t& out) {
  ocxo_perishable_ring_t& r = ocxo_fact_ring_for(ctx);

  __disable_irq();
  if (r.count == 0) {
    r.drain_armed = false;
    __enable_irq();
    return false;
  }

  out = r.facts[r.tail];
  r.tail = (r.tail + 1U) % OCXO_PERISHABLE_FACT_RING_SIZE;
  r.count--;
  r.drain_count++;
  if (r.count == 0) {
    r.drain_armed = false;
  }
  __enable_irq();
  return true;
}

static bool ocxo_fact_ring_push_from_isr(ocxo_runtime_context_t& ctx,
                                         const interrupt_perishable_fact_t& fact) {
  ocxo_perishable_ring_t& r = ocxo_fact_ring_for(ctx);
  ocxo_lane_t* lane = ctx.lane;

  if (r.count >= OCXO_PERISHABLE_FACT_RING_SIZE) {
    r.overflow_count++;
    if (lane) {
      lane->witness_fact_overflow_count = r.overflow_count;
    }
    return false;
  }

  r.facts[r.head] = fact;
  r.head = (r.head + 1U) % OCXO_PERISHABLE_FACT_RING_SIZE;
  r.count++;
  r.enqueue_count++;
  if (r.count > r.high_water) r.high_water = r.count;

  if (lane) {
    lane->witness_fact_enqueue_count = r.enqueue_count;
    lane->witness_fact_overflow_count = r.overflow_count;
    lane->witness_fact_high_water = r.high_water;
  }

  if (!r.drain_armed) {
    r.drain_armed = true;
    r.asap_arm_count++;
    if (lane) lane->witness_fact_asap_arm_count = r.asap_arm_count;

    const timepop_handle_t h =
        timepop_arm_asap(ocxo_fact_drain_callback, &ctx, ocxo_fact_drain_name(ctx));
    if (h == TIMEPOP_INVALID_HANDLE) {
      r.drain_armed = false;
      r.asap_fail_count++;
      if (lane) lane->witness_fact_asap_fail_count = r.asap_fail_count;
      return false;
    }
  }

  return true;
}


static inline bool ocxo_lane_compare_flag_pending(const ocxo_lane_t& lane) {
  return (lane.module->CH[lane.channel].CSCTRL & TMR_CSCTRL_TCF1) != 0;
}

static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].CNTR;
}

static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_clear_compare_flag();
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_clear_compare_flag(lane.channel);
  }
}

static void ocxo_qtimer_diag_record(ocxo_runtime_context_t& ctx,
                                    uint32_t isr_entry_dwt_raw,
                                    uint32_t event_dwt,
                                    uint32_t legacy_late_ticks,
                                    uint32_t interpreted_late_ticks = 0,
                                    uint32_t early_ticks = 0,
                                    int32_t service_offset_signed_ticks = 0,
                                    uint32_t service_offset_abs_ticks = 0,
                                    uint32_t target_delta_mod65536_ticks = 0,
                                    uint16_t target_low16 = 0,
                                    uint16_t isr_counter_low16 = 0,
                                    bool ema_predicted = false) {
  if (!ctx.qtimer_diag) return;

  ocxo_qtimer_diag_t& d = *ctx.qtimer_diag;
  d.count++;
  d.dwt_raw = isr_entry_dwt_raw;
  d.event_dwt = event_dwt;
  d.dwt_coordinate_source = ema_predicted
      ? OCXO_DWT_SOURCE_EMA_PREDICTED
      : OCXO_DWT_SOURCE_ISR_ENTRY;
  d.late_ticks = legacy_late_ticks;
  d.interpreted_late_ticks = interpreted_late_ticks;
  d.early_ticks = early_ticks;
  d.service_offset_signed_ticks = service_offset_signed_ticks;
  d.service_offset_abs_ticks = service_offset_abs_ticks;
  d.target_delta_mod65536_ticks = target_delta_mod65536_ticks;
  d.target_low16 = target_low16;
  d.isr_counter_low16 = isr_counter_low16;
}

static uint32_t ocxo_ema_abs_i32(int32_t value) {
  return (uint32_t)(value < 0 ? -(int64_t)value : (int64_t)value);
}

static int32_t ocxo_ema_round_shift(int32_t value, uint32_t shift) {
  if (shift == 0) return value;
  const int32_t half = (int32_t)(1U << (shift - 1U));
  if (value >= 0) return (value + half) >> shift;
  return -(((-value) + half) >> shift);
}

static uint32_t ocxo_lane_ema_predict_dwt(ocxo_lane_t& lane,
                                          uint32_t observed_dwt,
                                          bool* out_synthetic,
                                          int32_t* out_error_cycles) {
  if (out_synthetic) *out_synthetic = false;
  if (out_error_cycles) *out_error_cycles = 0;

  if (!OCXO_EMA_DWT_AUTHORITY_ENABLED) {
    return observed_dwt;
  }

  if (!lane.ema_initialized) {
    lane.ema_initialized = true;
    lane.ema_last_observed_dwt = observed_dwt;
    lane.ema_last_emitted_dwt = observed_dwt;
    lane.ema_last_predicted_dwt = observed_dwt;
    lane.ema_last_observed_interval_cycles = 0;
    lane.ema_last_error_cycles = 0;
    lane.ema_update_count++;
    return observed_dwt;
  }

  const uint32_t observed_interval = observed_dwt - lane.ema_last_observed_dwt;
  lane.ema_last_observed_interval_cycles = observed_interval;

  if (!lane.ema_interval_valid || lane.ema_interval_cycles == 0) {
    lane.ema_interval_cycles = observed_interval;
    lane.ema_interval_valid = true;
  } else {
    const int32_t err = (int32_t)((int64_t)observed_interval -
                                  (int64_t)lane.ema_interval_cycles);
    lane.ema_interval_cycles = (uint32_t)((int64_t)lane.ema_interval_cycles +
        (int64_t)ocxo_ema_round_shift(err, OCXO_EMA_SHIFT));
  }

  const uint32_t predicted = lane.ema_last_emitted_dwt + lane.ema_interval_cycles;
  const int32_t prediction_error = (int32_t)(observed_dwt - predicted);
  lane.ema_last_error_cycles = prediction_error;
  const uint32_t abs_error = ocxo_ema_abs_i32(prediction_error);
  if (abs_error > lane.ema_max_abs_error_cycles) {
    lane.ema_max_abs_error_cycles = abs_error;
  }

  lane.ema_last_observed_dwt = observed_dwt;
  lane.ema_last_emitted_dwt = predicted;
  lane.ema_last_predicted_dwt = predicted;
  lane.ema_update_count++;

  if (out_synthetic) *out_synthetic = true;
  if (out_error_cycles) *out_error_cycles = prediction_error;
  return predicted;
}

static void ocxo_apply_perishable_fact_deferred(
    ocxo_runtime_context_t& ctx,
    const interrupt_perishable_fact_t& fact) {
  ocxo_lane_t* lane = ctx.lane;
  if (!lane) return;

  interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);

  // First-pass lane-local cadence user callback hook.  Linear regression is
  // disabled for this checkpoint, so this remains a visible no-op count.
  lane->cadence_user_callback_count++;

  const ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx);
  lane->witness_fact_drain_count = ring.drain_count;
  lane->witness_fact_overflow_count = ring.overflow_count;
  lane->witness_fact_high_water = ring.high_water;
  lane->witness_fact_asap_arm_count = ring.asap_arm_count;
  lane->witness_fact_asap_fail_count = ring.asap_fail_count;

  const int32_t correction_cycles =
      dwt_cycles_for_ocxo_ticks_signed((int32_t)fact.service_offset_ticks);
  const uint32_t corrected_dwt =
      (uint32_t)((int64_t)fact.service_dwt_at_event -
                 (int64_t)correction_cycles);

  lane->witness_last_fact_sequence = fact.sequence;
  lane->witness_last_fact_one_second_due = fact.one_second_due;
  lane->witness_last_isr_csctrl_entry = fact.csctrl_entry;
  lane->witness_last_isr_compare_flag_entry = fact.compare_flag_seen;
  lane->witness_last_isr_compare_enabled_entry = fact.compare_enabled_entry;
  lane->witness_last_irq_had_armed = fact.had_armed;
  lane->witness_last_irq_had_active_rt = fact.had_active_rt;
  lane->witness_last_target_low16 = fact.target_low16;
  lane->witness_last_isr_counter_low16 = fact.service_counter_low16;
  lane->witness_last_target_delta_mod65536_ticks =
      fact.target_delta_mod65536_ticks;
  lane->witness_last_interpreted_late_ticks = fact.interpreted_late_ticks;
  lane->witness_last_early_ticks = fact.early_ticks;
  lane->witness_last_service_offset_signed_ticks =
      (int32_t)fact.service_offset_ticks;
  lane->witness_last_service_offset_abs_ticks = fact.service_offset_abs_ticks;
  lane->witness_last_service_was_early = (fact.service_offset_ticks < 0);
  lane->witness_last_service_was_on_or_after_target =
      (fact.service_offset_ticks >= 0);
  lane->witness_last_arm_to_isr_ticks = fact.arm_to_isr_ticks;
  lane->witness_last_arm_to_isr_dwt_cycles = fact.arm_to_isr_dwt_cycles;
  lane->witness_last_service_class = fact.service_class;
  lane->witness_last_isr_csctrl_after_disable = fact.csctrl_after_disable;
  lane->witness_last_isr_compare_flag_after_disable =
      fact.compare_flag_after_disable;
  lane->witness_last_isr_compare_enabled_after_disable =
      fact.compare_enabled_after_disable;
  lane->witness_last_service_correction_cycles = correction_cycles;
  lane->witness_last_service_corrected_dwt = corrected_dwt;
  lane->witness_last_late_ticks = fact.target_delta_mod65536_ticks;
  lane->witness_last_event_published = false;

  lane->witness_service_count++;
  if (fact.service_class == OCXO_SERVICE_CLASS_FALSE_IRQ) {
    lane->witness_false_irq_count++;
  } else if (fact.service_offset_ticks < 0) {
    lane->witness_early_service_count++;
  } else {
    lane->witness_on_or_after_service_count++;
  }

  // Linear regression is disabled and the OCXO per-sample forensic experiment
  // has been retired from the hot path.  Only one-second rollover facts reach
  // this foreground drain.

  if (!fact.one_second_due || !rt || !rt->active) {
    return;
  }

  bool ema_synthetic = false;
  int32_t ema_error_cycles = 0;
  const uint32_t observed_dwt = fact.service_dwt_at_event;

  // A skipped or duplicated one-second target is a custody discontinuity, not
  // an interval sample.  Do not let it poison the EMA predictor.
  if (lane->witness_previous_event_counter32_valid) {
    const uint32_t prior_delta =
        fact.target_counter32 - lane->witness_previous_event_counter32;
    if (prior_delta != OCXO_WITNESS_ONE_SECOND_COUNTS) {
      ocxo_lane_ema_reset(*lane);
    }
  }

  const uint32_t published_dwt = ocxo_lane_ema_predict_dwt(
      *lane, observed_dwt, &ema_synthetic, &ema_error_cycles);

  ocxo_qtimer_diag_record(ctx,
                          fact.isr_entry_dwt_raw,
                          published_dwt,
                          fact.target_delta_mod65536_ticks,
                          fact.interpreted_late_ticks,
                          fact.early_ticks,
                          (int32_t)fact.service_offset_ticks,
                          fact.service_offset_abs_ticks,
                          fact.target_delta_mod65536_ticks,
                          fact.target_low16,
                          fact.service_counter_low16,
                          ema_synthetic);

  lane->witness_fire_count++;
  lane->irq_count++;
  lane->logical_count32_at_last_second = fact.target_counter32;

  lane->witness_last_event_dwt = published_dwt;
  lane->witness_last_event_counter32 = fact.target_counter32;
  lane->witness_last_event_published = true;
  lane->witness_valid_publish_count++;
  if (fact.service_offset_ticks < 0) {
    lane->witness_early_service_published_count++;
  }

  if (lane->witness_previous_event_counter32_valid) {
    const uint32_t delta =
        fact.target_counter32 - lane->witness_previous_event_counter32;
    lane->witness_last_counter_delta_ticks = delta;
    if (delta != OCXO_WITNESS_ONE_SECOND_COUNTS) {
      lane->witness_counter_delta_violation_count++;
      lane->witness_last_bad_counter_delta = delta;
    }
  } else {
    lane->witness_last_counter_delta_ticks = 0;
  }
  lane->witness_previous_event_counter32 = fact.target_counter32;
  lane->witness_previous_event_counter32_valid = true;

  dwt_repair_diag_t ema_diag{};
  ema_diag.valid = true;
  ema_diag.candidate = ema_synthetic;
  ema_diag.synthetic = ema_synthetic;
  ema_diag.original_dwt = observed_dwt;
  ema_diag.predicted_dwt = published_dwt;
  ema_diag.used_dwt = published_dwt;
  ema_diag.error_cycles = ema_error_cycles;
  ema_diag.reason = ema_synthetic ? "ocxo_ema" : "ocxo_ema_init";

  // We are already in TimePop ASAP/foreground context. Dispatch the authored
  // rollover OCXO edge directly from the fact drain.
  emit_one_second_event(*rt,
                        published_dwt,
                        fact.target_counter32,
                        0,
                        false,
                        &ema_diag,
                        true,
                        nullptr);
}

static void ocxo_fact_drain_callback(timepop_ctx_t*,
                                     timepop_diag_t*,
                                     void* user_data) {
  auto* ctx = static_cast<ocxo_runtime_context_t*>(user_data);
  if (!ctx || !ctx->lane) return;

  for (;;) {
    interrupt_perishable_fact_t fact{};
    if (!ocxo_fact_ring_pop(*ctx, fact)) break;
    ocxo_apply_perishable_fact_deferred(*ctx, fact);
  }
}

struct ocxo_cadence_isr_sample_t {
  interrupt_subscriber_runtime_t* rt = nullptr;

  uint32_t isr_entry_dwt_raw = 0;
  uint32_t isr_csctrl_entry = 0;
  uint32_t isr_csctrl_after_program = 0;
  uint32_t event_dwt = 0;

  bool had_armed = false;
  bool had_active_rt = false;
  bool was_smartzero_current = false;

  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t service_counter_low16 = 0;
  uint32_t target_delta_mod65536_ticks = 0;

  int16_t service_offset_signed_ticks = 0;
  bool service_was_early = false;
  uint32_t early_ticks = 0;
  uint32_t interpreted_late_ticks = 0;
  uint32_t service_offset_abs_ticks = 0;

  uint32_t arm_remaining_ticks = 0;
  uint32_t arm_to_isr_ticks = 0;
  uint32_t arm_to_isr_dwt_cycles = 0;

  bool one_second_due = false;
};

static bool ocxo_cadence_classify_irq(ocxo_runtime_context_t& ctx,
                                      uint32_t isr_csctrl_entry) {
  ocxo_lane_t& lane = *ctx.lane;

  if ((isr_csctrl_entry & TMR_CSCTRL_TCF1) == 0) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    return false;
  }

  if (!lane.cadence_enabled || !lane.cadence_armed) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    ocxo_lane_clear_compare_flag(lane);
    ocxo_lane_stop_local_cadence(lane, OCXO_CADENCE_REASON_STOP);
    return false;
  }

  return true;
}

static ocxo_cadence_isr_sample_t ocxo_cadence_capture_perishable_facts(
    ocxo_runtime_context_t& ctx,
    uint32_t isr_entry_dwt_raw,
    uint32_t isr_csctrl_entry) {
  ocxo_lane_t& lane = *ctx.lane;

  ocxo_cadence_isr_sample_t sample{};
  sample.rt = ocxo_runtime_for(ctx);
  sample.isr_entry_dwt_raw = isr_entry_dwt_raw;
  sample.isr_csctrl_entry = isr_csctrl_entry;
  sample.had_armed = lane.cadence_armed;
  sample.had_active_rt = (sample.rt && sample.rt->active && lane.active);
  sample.was_smartzero_current = smartzero_is_current_lane(ctx.kind);

  if (sample.rt) sample.rt->irq_count++;

  sample.target_counter32 = lane.cadence_next_counter32;
  sample.target_low16 = (uint16_t)(sample.target_counter32 & 0xFFFFU);
  sample.service_counter_low16 = ocxo_lane_counter_now(lane);
  sample.target_delta_mod65536_ticks =
      (uint32_t)((uint16_t)(sample.service_counter_low16 - sample.target_low16));

  sample.service_offset_signed_ticks =
      (int16_t)((uint16_t)sample.target_delta_mod65536_ticks);
  sample.service_was_early = (sample.service_offset_signed_ticks < 0);
  sample.early_ticks = sample.service_was_early
      ? (uint32_t)(-(int32_t)sample.service_offset_signed_ticks)
      : 0U;
  sample.interpreted_late_ticks = sample.service_was_early
      ? 0U
      : (uint32_t)sample.service_offset_signed_ticks;
  sample.service_offset_abs_ticks = sample.service_was_early
      ? sample.early_ticks
      : sample.interpreted_late_ticks;

  sample.arm_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  sample.arm_to_isr_ticks =
      (uint32_t)((uint16_t)(sample.service_counter_low16 -
                            lane.witness_last_arm_low16));
  sample.arm_to_isr_dwt_cycles =
      isr_entry_dwt_raw - lane.witness_last_arm_dwt_raw;

  // Passive ISR sanity witness.  CNTR sanity compensates the compare
  // tooth for the tiny positive latency of reading a 10 MHz QuadTimer counter
  // after first-instruction DWT capture.  This remains exact after
  // compensation: only target+[0..2] ticks is accepted as a lawful CNTR read;
  // negative/early values or larger positive offsets are real errors.
  // DWT sanity compares arming-to-ISR cycles against elapsed OCXO ticks over
  // the same span.
  isr_sanity_diag_t* sanity = isr_sanity_for_kind(ctx.kind);
  if (sanity) {
    isr_sanity_record_cntr_latency_band(
        *sanity,
        sample.had_armed,
        sample.target_low16,
        sample.service_counter_low16,
        ISR_SANITY_OCXO_CNTR_MIN_READ_LATENCY_TICKS,
        ISR_SANITY_OCXO_CNTR_MAX_READ_LATENCY_TICKS,
        ISR_SANITY_OCXO_CNTR_READ_LATENCY_TICKS);

    const uint32_t expected_dwt_delta =
        (uint32_t)dwt_cycles_for_ocxo_ticks_signed(
            (int32_t)sample.arm_to_isr_ticks);
    isr_sanity_record_dwt(*sanity,
                          sample.had_armed,
                          expected_dwt_delta,
                          sample.arm_to_isr_dwt_cycles);
  }

  return sample;
}

static void ocxo_cadence_acknowledge_and_timestamp(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  // Acknowledge hardware immediately; the next cadence target is programmed
  // before any foreground work is requested.
  ocxo_lane_clear_compare_flag(*ctx.lane);
  sample.event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(sample.isr_entry_dwt_raw);
}

static void ocxo_cadence_update_synthetic_identity(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  ocxo_lane_t& lane = *ctx.lane;

  // Authored sample identity.  The compare target, not the service-time low16
  // read, is the synthetic counter32 coordinate of this cadence sample.
  if (ctx.clock32) {
    ctx.clock32->current_counter32 = sample.target_counter32;
    ctx.clock32->current_ns = (uint64_t)sample.target_counter32 * 100ULL;
    ctx.clock32->hardware16 = sample.target_low16;
    ctx.clock32->minder_update_count++;
  }

  lane.logical_count32_at_last_second = sample.target_counter32;
  lane.compare_target = sample.target_low16;
  lane.cadence_hits_total++;
  lane.cadence_fire_count++;
  lane.cadence_last_target_counter32 = sample.target_counter32;
  lane.cadence_last_target_low16 = sample.target_low16;
  lane.cadence_last_service_low16 = sample.service_counter_low16;
  lane.cadence_last_fire_dwt = sample.event_dwt;
  lane.cadence_last_isr_entry_dwt_raw = sample.isr_entry_dwt_raw;
  lane.cadence_last_service_offset_signed_ticks =
      (int32_t)sample.service_offset_signed_ticks;
  lane.cadence_last_service_offset_abs_ticks = sample.service_offset_abs_ticks;
  lane.cadence_last_interpreted_late_ticks = sample.interpreted_late_ticks;
  lane.cadence_last_early_ticks = sample.early_ticks;
  lane.cadence_last_was_early = sample.service_was_early;
  lane.witness_last_arm_to_isr_ticks = sample.arm_to_isr_ticks;
  lane.witness_last_arm_to_isr_dwt_cycles = sample.arm_to_isr_dwt_cycles;
}

static void ocxo_cadence_update_sample_phase(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  ocxo_lane_t& lane = *ctx.lane;

  if (!lane.phase_bootstrapped) {
    lane.phase_bootstrapped = true;
    lane.bootstrap_count++;
    lane.tick_mod_1000 = 0;
  }

  if (sample.was_smartzero_current) {
    // SmartZero is the only remaining user of short +10,000-tick OCXO samples.
    // Normal OCXO operation no longer uses a native 1 kHz rollover ladder.
    if (++lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
      lane.tick_mod_1000 = 0;
    }
    sample.one_second_due = false;
  } else {
    // In steady state each OCXO compare is a one-second edge capture.
    sample.one_second_due = true;
    lane.cadence_one_second_due_count++;
    lane.tick_mod_1000 = 0;
  }
  lane.cadence_last_one_second_due = sample.one_second_due;
}

static bool ocxo_cadence_feed_smartzero(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  if (sample.was_smartzero_current) {
    const int idx = smartzero_index_for_kind(ctx.kind);
    if (idx >= 0) {
      smartzero_write_begin();
      g_smartzero.lanes[idx].pub.fire_count++;
      smartzero_write_end();
    }
  }

  return smartzero_feed_sample(ctx.kind,
                               sample.event_dwt,
                               sample.target_counter32,
                               sample.target_low16);
}

static void ocxo_cadence_rearm_or_stop(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample,
    bool cadence_reauthored_by_smartzero) {
  ocxo_lane_t& lane = *ctx.lane;

  if (cadence_reauthored_by_smartzero) {
    // SmartZero acceptance can re-author this OCXO lane's logical grid from
    // inside smartzero_feed_sample().  The accepted lane now transitions into
    // normal one-second edge-capture mode; do not overwrite the newly prepared
    // target with the just-serviced SmartZero sample +10,000.
    sample.isr_csctrl_after_program = lane.module->CH[lane.channel].CSCTRL;
    return;
  }

  const bool keep_for_smartzero = smartzero_is_current_lane(ctx.kind);
  const bool keep_running = lane.active || keep_for_smartzero;
  if (!keep_running) {
    ocxo_lane_stop_local_cadence(lane, OCXO_CADENCE_REASON_STOP);
    sample.isr_csctrl_after_program = lane.module->CH[lane.channel].CSCTRL;
    return;
  }

  uint32_t next_target = 0;
  if (keep_for_smartzero) {
    // SmartZero still needs adjacent +10,000-tick OCXO samples to prove the
    // lane.  This is an acquisition-only ladder, not steady-state cadence.
    next_target = sample.target_counter32 + OCXO_CADENCE_INTERVAL_TICKS;
    const int idx = smartzero_index_for_kind(ctx.kind);
    if (idx >= 0) {
      smartzero_write_begin();
      g_smartzero.lanes[idx].pub.next_target_counter32 = next_target;
      g_smartzero.lanes[idx].pub.arm_count++;
      smartzero_write_end();
    }

    lane.witness_last_arm_dwt_raw = ARM_DWT_CYCCNT;
    lane.witness_last_arm_counter32 = sample.target_counter32;
    lane.witness_last_arm_low16 = sample.target_low16;
    lane.witness_last_arm_target_counter32 = next_target;
    lane.witness_last_arm_target_low16 = (uint16_t)(next_target & 0xFFFFU);
    lane.witness_last_arm_remaining_ticks = OCXO_CADENCE_INTERVAL_TICKS;
    ocxo_lane_program_local_cadence_compare(lane, next_target,
                                            OCXO_CADENCE_REASON_REARM);
  } else {
    next_target = sample.target_counter32 + OCXO_WITNESS_ONE_SECOND_COUNTS;
    ocxo_lane_prepare_one_second_target(ctx.kind, lane, *ctx.clock32,
                                        next_target,
                                        OCXO_CADENCE_REASON_REARM);
  }

  sample.isr_csctrl_after_program = lane.module->CH[lane.channel].CSCTRL;
}

static interrupt_perishable_fact_t ocxo_cadence_build_perishable_fact(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx);

  interrupt_perishable_fact_t fact{};
  fact.kind = ctx.kind;
  fact.provider = ctx.provider;
  fact.lane = ctx.lane_id;
  fact.sequence = ++ring.sequence;
  fact.isr_entry_dwt_raw = sample.isr_entry_dwt_raw;
  fact.service_dwt_at_event = sample.event_dwt;
  fact.target_counter32 = sample.target_counter32;
  fact.target_low16 = sample.target_low16;
  fact.service_counter_low16 = sample.service_counter_low16;
  fact.service_offset_ticks = sample.service_offset_signed_ticks;
  fact.service_offset_abs_ticks = sample.service_offset_abs_ticks;
  fact.interpreted_late_ticks = sample.interpreted_late_ticks;
  fact.early_ticks = sample.early_ticks;
  fact.target_delta_mod65536_ticks = sample.target_delta_mod65536_ticks;
  fact.arm_remaining_ticks = sample.arm_remaining_ticks;
  fact.arm_to_isr_ticks = sample.arm_to_isr_ticks;
  fact.arm_to_isr_dwt_cycles = sample.arm_to_isr_dwt_cycles;
  fact.csctrl_entry = sample.isr_csctrl_entry;
  fact.csctrl_after_disable = sample.isr_csctrl_after_program;
  fact.compare_flag_seen = true;
  fact.compare_enabled_entry =
      (sample.isr_csctrl_entry & TMR_CSCTRL_TCF1EN) != 0;
  fact.compare_flag_after_disable =
      (sample.isr_csctrl_after_program & TMR_CSCTRL_TCF1) != 0;
  fact.compare_enabled_after_disable =
      (sample.isr_csctrl_after_program & TMR_CSCTRL_TCF1EN) != 0;
  fact.had_armed = sample.had_armed;
  fact.had_active_rt = sample.had_active_rt;
  fact.one_second_due = sample.one_second_due && sample.had_active_rt;
  fact.service_class = sample.had_armed
      ? (sample.service_was_early ? OCXO_SERVICE_CLASS_EARLY_PRETARGET
                                  : OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET)
      : OCXO_SERVICE_CLASS_FALSE_IRQ;
  fact.exact_target_identity = true;
  fact.sample_phase_valid = ctx.lane->cadence_sample_phase_valid;
  fact.sample_phase_ticks = ctx.lane->cadence_sample_phase_ticks;
  fact.sample_phase_us = ctx.lane->cadence_sample_phase_us;
  fact.sample_phase_ns = ctx.lane->cadence_sample_phase_ns;
  fact.sample_period_ticks = ctx.lane->cadence_sample_period_ticks;

  return fact;
}

static void ocxo_cadence_update_one_second_witness(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  if (!sample.one_second_due) return;
  ctx.lane->witness_target_counter32 = sample.target_counter32;
  ctx.lane->witness_target_low16 = sample.target_low16;
}

static void ocxo_cadence_enqueue_fact(
    ocxo_runtime_context_t& ctx,
    const interrupt_perishable_fact_t& fact) {
  ocxo_lane_t& lane = *ctx.lane;

  if (!ocxo_fact_ring_push_from_isr(ctx, fact)) {
    // The ring reports the overflow.  Count the lane miss as well so the
    // existing lane report shows that a hardware event was not handed off.
    lane.miss_count++;
  }
}

static void ocxo_cadence_compare_isr(ocxo_runtime_context_t& ctx,
                                     uint32_t isr_entry_dwt_raw) {
  ocxo_lane_t& lane = *ctx.lane;

  const uint32_t isr_csctrl_entry = lane.module->CH[lane.channel].CSCTRL;
  if (!ocxo_cadence_classify_irq(ctx, isr_csctrl_entry)) return;

  ocxo_cadence_isr_sample_t sample =
      ocxo_cadence_capture_perishable_facts(ctx,
                                            isr_entry_dwt_raw,
                                            isr_csctrl_entry);
  ocxo_cadence_acknowledge_and_timestamp(ctx, sample);
  ocxo_cadence_update_synthetic_identity(ctx, sample);
  ocxo_cadence_update_sample_phase(ctx, sample);

  // Preserve the existing SmartZero custody rule: feed the sample before
  // deciding whether the local cadence remains owned by SmartZero or by the
  // normal active lane.
  const bool cadence_reauthored_by_smartzero =
      ocxo_cadence_feed_smartzero(ctx, sample);
  ocxo_cadence_rearm_or_stop(ctx, sample, cadence_reauthored_by_smartzero);
  ocxo_cadence_update_one_second_witness(ctx, sample);

  if (sample.one_second_due && sample.had_active_rt) {
    interrupt_perishable_fact_t fact =
        ocxo_cadence_build_perishable_fact(ctx, sample);
    ocxo_cadence_enqueue_fact(ctx, fact);
  }
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_compare_isr(g_ocxo1_ctx, isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_compare_isr(g_ocxo2_ctx, isr_entry_dwt_raw);
}

// ============================================================================
// QTimer1 vector — dispatches CH2 (TimePop)
// ============================================================================

static volatile interrupt_qtimer1_ch2_handler_fn g_qtimer1_ch2_handler = nullptr;

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb) {
  g_qtimer1_ch2_handler = cb;
}

// ============================================================================
// PPS GPIO ISR-entry listener — hosted diagnostics hook for process_witness
// ============================================================================

void interrupt_register_pps_entry_latency_handler(
    interrupt_pps_entry_latency_handler_fn cb) {
  g_pps_entry_latency_handler = cb;
}

// ============================================================================
// QTimer1 CH1 compare service — hosted rail for process_witness
// ============================================================================

void interrupt_register_qtimer1_ch1_handler(interrupt_qtimer1_ch1_handler_fn cb) {
  g_qtimer1_ch1_handler = cb;
}

static void qtimer1_ch1_schedule_next_hop(void) {
  if (!g_qtimer1_ch1_active) return;

  const uint32_t now = vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
  const uint32_t remaining = g_qtimer1_ch1_target_counter32 - now;

  // If the target is behind us in modular time, stop rather than generating
  // misleading event facts.  The caller can re-arm from a fresh PPS anchor.
  if (remaining == 0 || remaining > 0x7FFFFFFFUL) {
    g_qtimer1_ch1_active = false;
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  static constexpr uint32_t CH1_MAX_COMPARE_STEP_TICKS = 49123U;

  const uint32_t step =
      (remaining > CH1_MAX_COMPARE_STEP_TICKS)
          ? CH1_MAX_COMPARE_STEP_TICKS
          : remaining;

  g_qtimer1_ch1_next_compare_counter32 = now + step;

  const uint16_t hardware_target =
      vclock_hardware_low16_from_synthetic(g_qtimer1_ch1_next_compare_counter32);
  qtimer1_ch1_program_compare(hardware_target);
}

bool interrupt_qtimer1_ch1_arm_compare(uint32_t target_counter32) {
  if (!g_interrupt_hw_ready) return false;

  g_qtimer1_ch1_target_counter32 = target_counter32;
  g_qtimer1_ch1_active = true;
  g_qtimer1_ch1_arm_count++;

  qtimer1_ch1_schedule_next_hop();
  return g_qtimer1_ch1_active;
}

void interrupt_qtimer1_ch1_disable_compare(void) {
  g_qtimer1_ch1_active = false;
  qtimer1_ch1_disable_compare_hw();
}

uint16_t interrupt_qtimer1_ch1_counter_now(void) { return IMXRT_TMR1.CH[1].CNTR; }
uint16_t interrupt_qtimer1_ch1_comp1_now(void)   { return IMXRT_TMR1.CH[1].COMP1; }
uint16_t interrupt_qtimer1_ch1_csctrl_now(void)  { return IMXRT_TMR1.CH[1].CSCTRL; }

// ============================================================================
// TimePop hardware service API
// ============================================================================

uint32_t interrupt_vclock_counter32_observe_ambient(void) {
  return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
}

void interrupt_qtimer1_ch2_arm_compare(uint32_t target_counter32) {
  g_qtimer1_ch2_last_target_counter32 = target_counter32;
  g_qtimer1_ch2_arm_count++;
  const uint16_t hardware_target = vclock_hardware_low16_from_synthetic(target_counter32);
  qtimer1_ch2_program_compare(hardware_target);
}

uint16_t interrupt_qtimer1_ch2_counter_now(void) { return IMXRT_TMR1.CH[2].CNTR; }
uint16_t interrupt_qtimer1_ch2_comp1_now(void)   { return IMXRT_TMR1.CH[2].COMP1; }
uint16_t interrupt_qtimer1_ch2_csctrl_now(void)  { return IMXRT_TMR1.CH[2].CSCTRL; }

static void qtimer1_ch1_bridge_isr(uint32_t isr_entry_dwt_raw) {
  qtimer1_ch1_clear_compare_flag();

  if (!g_qtimer1_ch1_active) {
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  const uint32_t fired_counter32 = g_qtimer1_ch1_next_compare_counter32;

  if (fired_counter32 != g_qtimer1_ch1_target_counter32) {
    g_qtimer1_ch1_hop_count++;
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  g_qtimer1_ch1_active = false;
  qtimer1_ch1_disable_compare_hw();
  g_qtimer1_ch1_fire_count++;

  const uint32_t qtimer_event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const int64_t gnss_ns = vclock_gnss_from_counter32(fired_counter32);

  interrupt_qtimer1_ch1_compare_event_t event{};
  event.sequence = ++g_qtimer1_ch1_sequence;
  event.target_counter32 = g_qtimer1_ch1_target_counter32;
  event.counter32_at_event = fired_counter32;
  event.counter32_residual_ticks =
      (int32_t)(fired_counter32 - g_qtimer1_ch1_target_counter32);
  event.isr_entry_dwt_raw = isr_entry_dwt_raw;
  event.dwt_at_event = qtimer_event_dwt;
  event.gnss_ns_at_event = gnss_ns;

  if (g_qtimer1_ch1_handler) {
    g_qtimer1_ch1_handler(event);
  }
}

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  if (IMXRT_TMR1.CH[1].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch1_bridge_isr(isr_entry_dwt_raw);
  }
  else if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_clear_compare_flag();
    const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
    // Snapshot the fired target before TimePop is invoked; the handler may arm
    // the next compare and update g_qtimer1_ch2_last_target_counter32.
    const uint32_t ch2_event_counter32 = g_qtimer1_ch2_last_target_counter32;
    interrupt_ch2_implicit_rollover_tend();

    pps_relay_ch2_tick(g_vclock_clock32.current_counter32);
    vclock_ch2_one_second_service(qtimer_event_dwt,
                                  ch2_event_counter32);
    vclock_ch2_smartzero_service(qtimer_event_dwt,
                                  ch2_event_counter32);
    if (g_qtimer1_ch2_handler) {
      const bridge_projection_t bridge = interrupt_dwt_to_vclock_gnss_projection(qtimer_event_dwt);
      const int64_t  gnss_ns = bridge.gnss_ns;
      bridge_projection_record_stats(interrupt_subscriber_kind_t::TIMEPOP, bridge);

      interrupt_event_t event{};
      event.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      event.provider = interrupt_provider_kind_t::QTIMER1;
      event.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      event.status   = interrupt_event_status_t::OK;
      event.dwt_at_event       = qtimer_event_dwt;
      event.gnss_ns_at_event   = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
      // CH2 compare event identity is the synthetic deadline that
      // process_interrupt armed in hardware on TimePop's behalf.  This is an
      // event fact, not a post-event ambient read.
      event.counter32_at_event = ch2_event_counter32;

      interrupt_capture_diag_t diag{};
      diag.enabled  = true;
      diag.provider = interrupt_provider_kind_t::QTIMER1;
      diag.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      diag.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      diag.dwt_at_event       = qtimer_event_dwt;
      diag.gnss_ns_at_event   = event.gnss_ns_at_event;
      diag.counter32_at_event = event.counter32_at_event;
      bridge_projection_copy_to_diag(diag, bridge);

      g_qtimer1_ch2_handler(event, diag);
    }

    // Bootstrap/rebootstrap PPS_VCLOCK publication is also native CH2 custody
    // now.  Run it only after TimePop has consumed the compare event, matching
    // the safe ordering used for VCLOCK fact-drain arming.
    vclock_ch2_epoch_native_service(qtimer_event_dwt,
                                    ch2_event_counter32);

    // CH2-authored VCLOCK facts are now drain-armed from native CH2 custody,
    // but only after TimePop has consumed the compare event.  This preserves
    // the non-reentrant boundary that kept the previous heartbeat arm safe.
    vclock_ch2_fact_drain_arm_after_timepop();
  }
  // CH1/CH2 share one vector.  If multiple flags are pending, the
  // unhandled flag remains set and NVIC re-dispatches with a fresh
  // first-instruction _raw capture.  VCLOCK cadence no longer owns CH3; it
  // is a normal TimePop client on CH2.
}

// ============================================================================
// PPS GPIO ISR — authors PPS and PPS_VCLOCK from the same _raw capture
// ============================================================================
//
// The _raw is the first-instruction capture.  It is converted IMMEDIATELY
// into PPS and PPS_VCLOCK event-coordinate values and never propagated.

static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_pps_edge_dispatch) return;

  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  g_pps_edge_dispatch(snap);
}


static void pps_relay_assert_from_isr(uint32_t sequence) {
  // PPS relay HIGH is a physical PPS witness and stays in ISR context.
  // Relay LOW is intentionally *not* scheduled through TimePop.  Instead,
  // intrinsic QTimer1 CH2 service counts elapsed 1 ms VCLOCK cells and
  // deasserts the relay when the counter reaches zero.  This removes
  // relay-off from TimePop callback/ASAP/one-shot slot mutation entirely.
  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  g_pps_relay_assert_count++;
  g_pps_relay_last_assert_sequence = sequence;
  g_pps_relay_deassert_countdown_ticks = PPS_RELAY_OFF_CADENCE_TICKS;
  g_pps_relay_ch2_last_counter32 =
      vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
  g_pps_relay_ch2_tick_valid = true;
  g_pps_relay_deassert_arm_pending = false;
  g_pps_relay_timer_active = true;
  g_pps_relay_deassert_arm_count++;
}

static void pps_relay_ch2_tick(uint32_t vclock_counter32) {
  if (!g_interrupt_hw_ready) return;

  uint32_t ticks = g_pps_relay_deassert_countdown_ticks;
  if (!g_pps_relay_timer_active || ticks == 0) {
    g_pps_relay_ch2_tick_valid = false;
    return;
  }

  if (!g_pps_relay_ch2_tick_valid) {
    g_pps_relay_ch2_last_counter32 = vclock_counter32;
    g_pps_relay_ch2_tick_valid = true;
    return;
  }

  const uint32_t elapsed_ticks =
      vclock_counter32 - g_pps_relay_ch2_last_counter32;
  if (elapsed_ticks < VCLOCK_INTERVAL_COUNTS) return;

  const uint32_t elapsed_cells = elapsed_ticks / VCLOCK_INTERVAL_COUNTS;
  if (elapsed_cells > 1U) {
    g_pps_relay_ch2_catchup_count += (elapsed_cells - 1U);
  }

  if (elapsed_cells >= ticks) {
    g_pps_relay_deassert_countdown_ticks = 0;
    g_pps_relay_ch2_tick_valid = false;
    g_pps_relay_ch2_tick_count += ticks;
    digitalWriteFast(GNSS_PPS_RELAY, LOW);
    g_pps_relay_timer_active = false;
    g_pps_relay_deassert_count++;
    return;
  }

  ticks -= elapsed_cells;
  g_pps_relay_deassert_countdown_ticks = ticks;
  g_pps_relay_ch2_last_counter32 += elapsed_cells * VCLOCK_INTERVAL_COUNTS;
  g_pps_relay_ch2_tick_count += elapsed_cells;
}

static void pps_post_isr_publish_inline(const interrupt_epoch_capture_t& cap,
                                        uint32_t isr_entry_dwt_raw) {
  epoch_capture_publish(cap);

  if (g_pps_entry_latency_handler) {
    g_pps_entry_latency_handler(cap.sequence, isr_entry_dwt_raw);
  }
}

static void pps_post_isr_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  interrupt_epoch_capture_t cap{};
  uint32_t isr_entry_dwt_raw = 0;
  bool had_sample = false;

  __disable_irq();
  had_sample = g_pps_post_isr.pending;
  if (had_sample) {
    cap = g_pps_post_isr.cap;
    isr_entry_dwt_raw = g_pps_post_isr.isr_entry_dwt_raw;
    g_pps_post_isr.pending = false;
  }
  __enable_irq();

  if (!had_sample) return;

  g_pps_post_isr.drain_count++;
  pps_post_isr_publish_inline(cap, isr_entry_dwt_raw);
}

static void pps_post_isr_defer(const interrupt_epoch_capture_t& cap,
                               uint32_t isr_entry_dwt_raw) {
  if (!g_interrupt_runtime_ready) {
    pps_post_isr_publish_inline(cap, isr_entry_dwt_raw);
    return;
  }

  __disable_irq();
  if (g_pps_post_isr.pending) {
    g_pps_post_isr.overwrite_count++;
  }
  g_pps_post_isr.cap = cap;
  g_pps_post_isr.isr_entry_dwt_raw = isr_entry_dwt_raw;
  g_pps_post_isr.pending = true;
  g_pps_post_isr.arm_count++;
  g_pps_post_isr.last_sequence = cap.sequence;
  g_pps_post_isr.last_capture_window_cycles = cap.capture_window_cycles;
  __enable_irq();

  const timepop_handle_t h =
      timepop_arm_asap(pps_post_isr_asap_callback, nullptr, "PPS_POST_ISR");
  if (h == TIMEPOP_INVALID_HANDLE) {
    interrupt_epoch_capture_t fallback{};
    uint32_t fallback_raw = 0;
    bool had_sample = false;

    __disable_irq();
    had_sample = g_pps_post_isr.pending;
    if (had_sample) {
      fallback = g_pps_post_isr.cap;
      fallback_raw = g_pps_post_isr.isr_entry_dwt_raw;
      g_pps_post_isr.pending = false;
      g_pps_post_isr.asap_fail_count++;
    }
    __enable_irq();

    if (had_sample) {
      pps_post_isr_publish_inline(fallback, fallback_raw);
    }
  }
}

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  // PPS GPIO is a witness/selector only.  It captures the physical PPS facts
  // and, during rebootstrap, selects the VCLOCK-domain edge identity.  It does
  // NOT author the public PPS/VCLOCK DWT coordinate; that coordinate is later
  // authored by the TimePop VCLOCK cadence client on QTimer1 CH2 so all public
  // DWT captures live in one TimePop event-coordinate system.
  //
  // Zero-offset capture doctrine: immediately after the first-instruction
  // DWT raw capture, read the three 10 MHz lane counters in one custody
  // window.  The raw 16-bit reads are retained as forensic-only evidence;
  // runtime timing math consumes the synthetic 32-bit lane coordinates below.
  const uint32_t epoch_capture_start_raw = isr_entry_dwt_raw;
  const uint16_t hardware_low16 = qtimer1_ch0_counter_now();
  const uint32_t epoch_capture_after_vclock_raw = ARM_DWT_CYCCNT;

  // Defensive boot rule: VCLOCK is mandatory and QTimer1 CH0 is initialized
  // before IRQs are enabled.  OCXO raw reads are desirable, but they are not
  // allowed to become a boot-time dependency unless QTimer3 lane hardware is
  // known initialized.  This keeps PPS witness/selector work alive even if an
  // OCXO lane is not ready yet.
  const bool ocxo1_capture_hw_ready =
      g_interrupt_hw_ready && !OCXO1_DISABLED && g_ocxo1_lane.initialized;
  const bool ocxo2_capture_hw_ready =
      g_interrupt_hw_ready && !OCXO2_DISABLED && g_ocxo2_lane.initialized;
  const bool ocxo_capture_hw_ready =
      (!OCXO1_DISABLED || ocxo1_capture_hw_ready) &&
      (!OCXO2_DISABLED || ocxo2_capture_hw_ready);
  const uint16_t ocxo1_hardware16 = ocxo1_capture_hw_ready ? IMXRT_TMR2.CH[0].CNTR : 0;
  const uint16_t ocxo2_hardware16 = ocxo2_capture_hw_ready ? IMXRT_TMR3.CH[3].CNTR : 0;
  const uint32_t epoch_capture_end_raw = ARM_DWT_CYCCNT;

  const uint16_t ch3_now = hardware_low16;

  const uint16_t selected_low16 =
      (uint16_t)(hardware_low16 -
                 (uint16_t)VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED);

  if (g_vclock_clock32.pending_zero) {
    vclock_clock_zero_at_hardware_low16(g_vclock_clock32.pending_zero_ns,
                                        selected_low16);
  }

  const uint32_t observed_counter32 =
      vclock_synthetic_from_hardware_low16(hardware_low16) +
      (uint32_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  const uint32_t selected_counter32 =
      observed_counter32 - VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED;

  const uint32_t ocxo1_counter32 = ocxo1_capture_hw_ready
      ? project_counter32_from_hw16(g_ocxo1_clock32, ocxo1_hardware16)
      : 0;
  const uint32_t ocxo2_counter32 = ocxo2_capture_hw_ready
      ? project_counter32_from_hw16(g_ocxo2_clock32, ocxo2_hardware16)
      : 0;

  g_gpio_irq_count++;
  g_pps_gpio_heartbeat.edge_count++;
  pps_relay_assert_from_isr(g_pps_gpio_heartbeat.edge_count);

  interrupt_epoch_capture_t epoch_cap{};
  epoch_cap.sequence = g_pps_gpio_heartbeat.edge_count;
  epoch_cap.capture_dwt_start_raw = epoch_capture_start_raw;
  epoch_cap.capture_dwt_after_vclock_raw = epoch_capture_after_vclock_raw;
  epoch_cap.capture_dwt_end_raw = epoch_capture_end_raw;
  epoch_cap.capture_window_cycles = epoch_capture_end_raw - epoch_capture_start_raw;
  epoch_cap.vclock_read_offset_cycles =
      epoch_capture_after_vclock_raw - epoch_capture_start_raw;
  epoch_cap.vclock_dwt_at_edge =
      pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);
  epoch_cap.vclock_capture_valid =
      epoch_cap.vclock_read_offset_cycles <= EPOCH_CAPTURE_MAX_WINDOW_CYCLES;
  epoch_cap.all_lanes_capture_valid =
      ocxo_capture_hw_ready &&
      epoch_cap.capture_window_cycles <= EPOCH_CAPTURE_MAX_WINDOW_CYCLES;
  // A packet is operationally selectable only after interrupt runtime has
  // finished initialization.  This prevents a boot-time partial capture from
  // being mistaken for a CLOCKS.ZERO epoch packet.
  epoch_cap.valid = g_interrupt_runtime_ready && epoch_cap.vclock_capture_valid;
  epoch_cap.vclock_hardware16_observed = hardware_low16;
  epoch_cap.vclock_hardware16_selected = selected_low16;
  epoch_cap.ocxo1_hardware16 = ocxo1_hardware16;
  epoch_cap.ocxo2_hardware16 = ocxo2_hardware16;
  epoch_cap.vclock_counter32 = selected_counter32;
  epoch_cap.ocxo1_counter32 = ocxo1_counter32;
  epoch_cap.ocxo2_counter32 = ocxo2_counter32;

  pps_t pps;
  pps.sequence          = g_pps_gpio_heartbeat.edge_count;
  pps.dwt_at_edge       = pps_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  pps.counter32_at_edge = observed_counter32;
  pps.ch3_at_edge       = ch3_now;

  g_last_pps_witness = pps;
  g_last_pps_witness_valid = true;

  // Passive PPS ISR sanity witness.  Once a prior PPS exists, CNTR sanity
  // checks the observed VCLOCK counter interval against exactly one 10 MHz
  // second, while DWT sanity checks the physical PPS DWT interval against the
  // current CLOCKS/PPS-derived cycles-per-second ruler.
  if (g_isr_sanity_pps_prev_valid) {
    const uint32_t observed_counter_delta =
        observed_counter32 - g_isr_sanity_pps_prev_counter32;
    isr_sanity_record_cntr(
        g_isr_sanity_pps,
        true,
        (uint32_t)VCLOCK_COUNTS_PER_SECOND +
            ISR_SANITY_PPS_CNTR_READ_LATENCY_TICKS,
        observed_counter_delta);

    const uint32_t observed_dwt_delta =
        pps.dwt_at_edge - g_isr_sanity_pps_prev_dwt;
    const uint32_t expected_dwt_delta = interrupt_vclock_cycles_per_second();
    isr_sanity_record_dwt(g_isr_sanity_pps,
                          expected_dwt_delta != 0,
                          expected_dwt_delta,
                          observed_dwt_delta);
  }
  g_isr_sanity_pps_prev_valid = true;
  g_isr_sanity_pps_prev_counter32 = observed_counter32;
  g_isr_sanity_pps_prev_dwt = pps.dwt_at_edge;

  // Defer epoch-packet publication and PPS entry-latency diagnostic callback.
  // The witness facts above remain immediate because VCLOCK steady-state and
  // rebootstrap consume them.
  pps_post_isr_defer(epoch_cap, isr_entry_dwt_raw);

  // During rebootstrap, PPS selects the sacred VCLOCK edge identity.  Native
  // CH2 custody consumes this pending latch after TimePop has processed the
  // next CH2 compare fact, back-projects to the selected edge, and publishes
  // the canonical PPS_VCLOCK epoch.
  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;

    g_vclock_epoch_latch.pending = true;
    g_vclock_epoch_latch.pps = pps;
    g_vclock_epoch_latch.counter32_at_edge = selected_counter32;
    g_vclock_epoch_latch.ch3_at_edge = selected_low16;
    g_vclock_epoch_latch.observed_counter32 = observed_counter32;
    g_vclock_epoch_latch.observed_ch3 = ch3_now;
    g_vclock_epoch_latch.observed_ticks_after_selected =
        VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED;
    g_vclock_epoch_latch.first_cadence_counter32 = 0;
    g_vclock_epoch_latch.first_cadence_dwt = 0;
    g_vclock_epoch_latch.backdate_ticks = 0;
    g_vclock_epoch_latch.backdate_cycles = 0;
    g_vclock_epoch_latch.sacred_dwt = 0;

    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.compare_target =
        (uint16_t)(selected_low16 + (uint16_t)VCLOCK_INTERVAL_COUNTS);
    g_vclock_lane.logical_count32_at_last_second = selected_counter32;
    vclock_clock_anchor_hardware_low16(selected_counter32, selected_low16);
  }
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw);
}

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn) {
  g_pps_edge_dispatch = fn;
}

// ============================================================================
// PPS / PPS_VCLOCK accessors
// ============================================================================
//
// interrupt_last_pps_vclock() — canonical PPS_VCLOCK epoch.
// interrupt_last_pps_edge()   — legacy projection, for back-compat with
//                               consumers that still take pps_edge_snapshot_t.
//                               Field map documented inline below.
//
// There is intentionally no public interrupt_last_pps() accessor.  Physical
// PPS GPIO DWT is a private witness/audit fact, not a timing primitive.

pps_vclock_t interrupt_last_pps_vclock(void) {
  return store_load_pvc();
}

// Legacy projection.  Field map:
//   snapshot.dwt_at_edge       <- pvc.dwt_at_edge       (PPS_VCLOCK)
//   snapshot.counter32_at_edge <- pvc.counter32_at_edge (PPS_VCLOCK)
//   snapshot.ch3_at_edge       <- pvc.ch3_at_edge       (PPS_VCLOCK)
//   snapshot.gnss_ns_at_edge   <- -1 (GNSS labels are CLOCKS-owned)
//   snapshot.physical_pps_*    <- pps.*                 (physical facts)
//   snapshot.dwt_raw_at_edge   <- pvc.dwt_at_edge       (legacy alias;
//                                                        misnamed, held
//                                                        for API continuity)

pps_edge_snapshot_t interrupt_last_pps_edge(void) {
  pps_t pps;
  pps_vclock_t pvc;
  store_load(pps, pvc);

  pps_edge_snapshot_t out{};
  out.sequence          = pvc.sequence;
  out.dwt_at_edge       = pvc.dwt_at_edge;
  out.dwt_raw_at_edge   = pvc.dwt_at_edge;     // legacy alias, NOT raw
  out.counter32_at_edge = pvc.counter32_at_edge;
  out.ch3_at_edge       = pvc.ch3_at_edge;
  out.gnss_ns_at_edge   = -1;  // GNSS labels are CLOCKS-owned.

  out.physical_pps_dwt_raw_at_edge        = pps.dwt_at_edge;  // legacy field; carries event coord
  out.physical_pps_dwt_normalized_at_edge = pps.dwt_at_edge;
  out.physical_pps_counter32_at_read      = pps.counter32_at_edge;
  out.physical_pps_ch3_at_read            = pps.ch3_at_edge;

  out.vclock_epoch_counter32              = pvc.counter32_at_edge;
  out.vclock_epoch_ch3                    = pvc.ch3_at_edge;
  out.vclock_epoch_ticks_after_pps        = VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS;
  out.vclock_epoch_counter32_offset_ticks = VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  out.vclock_epoch_dwt_offset_cycles      = CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
  out.vclock_epoch_selected               = true;
  return out;
}


interrupt_pps_edge_heartbeat_t interrupt_pps_edge_heartbeat(void) {
  interrupt_pps_edge_heartbeat_t out{};
  out.edge_count = g_pps_gpio_heartbeat.edge_count;
  out.last_dwt = g_pps_gpio_heartbeat.last_dwt;
  out.last_gnss_ns = g_pps_gpio_heartbeat.last_gnss_ns;
  out.gpio_irq_count = g_gpio_irq_count;
  out.gpio_miss_count = g_gpio_miss_count;
  return out;
}

// ============================================================================
// Subscribe / start / stop / etc.
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_runtime_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE || !sub.on_event) return false;
  interrupt_subscriber_runtime_t* rt = runtime_for(sub.kind);
  if (!rt || !rt->desc) return false;
  rt->sub = sub;
  rt->subscribed = true;
  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->start_count++;
  cadence_regression_reset_kind(kind);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    rt->active = true;
    g_vclock_lane.active = true;
    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    vclock_ch2_one_second_disable();
    // VCLOCK_HEARTBEAT is armed once during process_interrupt_init() and is the
    // only 1 ms TimePop cadence source.  Starting VCLOCK must not arm a second
    // cadence slot.
    return g_vclock_heartbeat_armed || vclock_heartbeat_arm_timepop();
  }

  if (ocxo_kind_disabled(kind)) {
    rt->active = false;
    ocxo_lane_t* disabled_lane = ocxo_lane_for(kind);
    if (disabled_lane) {
      disabled_lane->active = false;
      disabled_lane->cadence_enabled = false;
      disabled_lane->cadence_armed = false;
      disabled_lane->witness_armed = false;
    }
    return true;
  }

  rt->active = true;
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized) return false;
  lane->active = true;
  lane->phase_bootstrapped = true;
  lane->tick_mod_1000 = 0;
  ocxo_lane_ema_reset(*lane);

  // OCXO lanes now own only their local one-second edge compare.  Start the
  // one-second edge compare from the installed logical grid when available;
  // otherwise birth from the current low-word observation.
  return ocxo_lane_start_local_cadence(kind, *lane,
                                       *synthetic_clock_for_kind(kind),
                                       OCXO_CADENCE_REASON_START);
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = false;
  rt->stop_count++;
  cadence_regression_reset_kind(kind);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    g_vclock_lane.phase_bootstrapped = false;
    vclock_ch2_one_second_disable();
    // Do not cancel VCLOCK_HEARTBEAT here; it is the system-wide substrate
    // cadence for VCLOCK and the OCXO passive counter domains.
    return true;
  }
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return false;
  lane->active = false;
  if (ocxo_kind_disabled(kind)) {
    lane->cadence_enabled = false;
    lane->cadence_armed = false;
    lane->witness_armed = false;
    return true;
  }
  ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_STOP);
  return true;
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
  vclock_ch2_one_second_disable();
  cadence_regression_reset_kind(interrupt_subscriber_kind_t::VCLOCK);
  vclock_fact_ring_reset();
  // Rebootstrap selects a fresh PPS_VCLOCK edge identity.
  g_pvc_anchor_reset_pending = true;
}

bool interrupt_pps_rebootstrap_pending(void) { return g_pps_rebootstrap_pending; }

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_diag;
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {}

// ============================================================================
// Hardware + runtime init
// ============================================================================

static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IMXRT_TMR1.CH[0].CTRL = 0;
  IMXRT_TMR1.CH[1].CTRL = 0;

  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[0].LOAD   = 0;
  IMXRT_TMR1.CH[0].CNTR   = 0;
  IMXRT_TMR1.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  // CH1 is an independent VCLOCK-domain compare rail hosted by
  // process_interrupt for clients such as process_witness.  It is NOT a
  // cascade high word.
  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer1_ch1_disable_compare_hw();

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;

  (void)IMXRT_TMR1.CH[0].CNTR;
}

static void qtimer1_init_ch2_scheduler(void) {
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
}

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR2;
  g_ocxo1_lane.channel = 0;
  g_ocxo1_lane.pcs = 0;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = 3;
  g_ocxo2_lane.pcs = 3;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  g_pps_relay_pin_initialized = true;

  if (!OCXO1_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
    *(portConfigRegister(OCXO1_PIN)) = 1;
    IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;

    IMXRT_TMR2.CH[0].CTRL = 0; IMXRT_TMR2.CH[0].SCTRL = 0;
    IMXRT_TMR2.CH[0].CSCTRL = 0; IMXRT_TMR2.CH[0].LOAD = 0;
    IMXRT_TMR2.CH[0].CNTR = 0; IMXRT_TMR2.CH[0].COMP1 = 0xFFFF;
    IMXRT_TMR2.CH[0].CMPLD1 = 0xFFFF; IMXRT_TMR2.CH[0].CMPLD2 = 0;
    IMXRT_TMR2.CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo1_lane.pcs);
    qtimer2_ch0_disable_compare();
    IMXRT_TMR2.ENBL |= 0x0001;
  }

  if (!OCXO2_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
    *(portConfigRegister(OCXO2_PIN)) = 1;
    IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

    IMXRT_TMR3.CH[3].CTRL = 0; IMXRT_TMR3.CH[3].SCTRL = 0;
    IMXRT_TMR3.CH[3].CSCTRL = 0; IMXRT_TMR3.CH[3].LOAD = 0;
    IMXRT_TMR3.CH[3].CNTR = 0; IMXRT_TMR3.CH[3].COMP1 = 0xFFFF;
    IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
    IMXRT_TMR3.CH[3].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo2_lane.pcs);
    qtimer3_disable_compare(3);
    IMXRT_TMR3.ENBL |= (uint16_t)(1U << 3);
  }

  g_ocxo1_lane.initialized = !OCXO1_DISABLED;
  g_ocxo2_lane.initialized = !OCXO2_DISABLED;

  // QTimer1 synchronized channel start: ENBL=0 first, configure CH0/CH1/CH2
  // fully, then a single ENBL write starts the active VCLOCK-domain channels
  // on the same edge.  CH2 is the only enabled compare channel; VCLOCK cadence
  // is expressed as a normal TimePop client.
  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();
  g_vclock_lane.initialized = true;
  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;
  IMXRT_TMR1.CH[2].CNTR = 0;
  // Only CH0 (passive VCLOCK counter) and CH2 (TimePop scheduler compare)
  // are enabled.  VCLOCK cadence is now a TimePop client; CH3 remains off.
  IMXRT_TMR1.ENBL = 0x05;

  g_interrupt_hw_ready = true;
}

static void runtime_init_subscribers(void) {
  g_subscriber_count = 0;
  g_rt_vclock = nullptr;
  g_rt_ocxo1 = nullptr;
  g_rt_ocxo2 = nullptr;

  for (auto& rt : g_subscribers) rt = interrupt_subscriber_runtime_t{};

  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
    rt = interrupt_subscriber_runtime_t{};
    rt.desc = &DESCRIPTORS[i];
    if      (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) g_rt_vclock = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1)  g_rt_ocxo1  = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO2)  g_rt_ocxo2  = &rt;
  }
}

static void runtime_reset_pps_state(void) {
  g_pps_gpio_heartbeat = pps_gpio_heartbeat_t{};
  g_gpio_irq_count = 0;
  g_gpio_miss_count = 0;
  g_pps_post_isr = pps_post_isr_mailbox_t{};
  g_pps_relay_timer_active = false;
  g_pps_relay_deassert_arm_pending = false;
  g_pps_relay_deassert_countdown_ticks = 0;
  g_pps_relay_assert_count = 0;
  g_pps_relay_deassert_count = 0;
  g_pps_relay_deassert_arm_count = 0;
  g_pps_relay_deassert_arm_fail_count = 0;
  g_pps_relay_deassert_arm_skip_count = 0;
  g_pps_relay_last_assert_sequence = 0;
  g_pps_relay_ch2_tick_valid = false;
  g_pps_relay_ch2_last_counter32 = 0;
  g_pps_relay_ch2_tick_count = 0;
  g_pps_relay_ch2_catchup_count = 0;
  g_last_pps_witness = pps_t{};
  g_last_pps_witness_valid = false;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
  g_vclock_repair_stats = vclock_repair_stats_t{};
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;
}

static void runtime_reset_snapshot_and_bridge_state(void) {
  g_store = snapshot_store_t{};
  g_epoch_capture_store = epoch_capture_store_t{};
  pvc_anchor_ring_reset();
  g_bridge_stats_timepop = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo1 = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo2 = bridge_anchor_stats_t{};
  g_pps_edge_dispatch = nullptr;
  g_pps_entry_latency_handler = nullptr;
}

static void runtime_reset_smartzero_state(void) {
  g_smartzero = smartzero_runtime_t{};
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }
}

static void runtime_reset_clock32_state(void) {
  g_vclock_clock32 = vclock_synthetic_clock32_t{};
  g_ocxo1_clock32 = synthetic_clock32_t{};
  g_ocxo2_clock32 = synthetic_clock32_t{};
}

static void runtime_bootstrap_synthetic_clocks_if_ready(void) {
  // Defensive birth anchors.  These are not logical ZERO operations; they
  // simply make process_interrupt's synthetic coordinate extenders safe before
  // CLOCKS has installed a user/campaign epoch.
  if (!g_interrupt_hw_ready) return;

  vclock_clock_bootstrap_from_hw16(qtimer1_ch0_counter_now());
  if (!OCXO1_DISABLED && g_ocxo1_lane.initialized) {
    synthetic_clock_bootstrap_from_hw16(g_ocxo1_clock32, IMXRT_TMR2.CH[0].CNTR);
    g_ocxo1_lane.logical_count32_at_last_second = g_ocxo1_clock32.current_counter32;
  }
  if (!OCXO2_DISABLED && g_ocxo2_lane.initialized) {
    synthetic_clock_bootstrap_from_hw16(g_ocxo2_clock32, IMXRT_TMR3.CH[3].CNTR);
    g_ocxo2_lane.logical_count32_at_last_second = g_ocxo2_clock32.current_counter32;
  }
}

static void runtime_reset_qtimer_host_state(void) {
  g_qtimer1_ch1_sequence = 0;
  g_qtimer1_ch1_target_counter32 = 0;
  g_qtimer1_ch1_next_compare_counter32 = 0;
  g_qtimer1_ch1_arm_count = 0;
  g_qtimer1_ch1_fire_count = 0;
  g_qtimer1_ch1_hop_count = 0;
  g_qtimer1_ch1_active = false;
  g_qtimer1_ch1_handler = nullptr;
  g_qtimer1_ch2_last_target_counter32 = 0;
  g_qtimer1_ch2_arm_count = 0;
  g_ocxo1_qtimer_diag = ocxo_qtimer_diag_t{};
  g_ocxo2_qtimer_diag = ocxo_qtimer_diag_t{};
}

static void runtime_reset_vclock_heartbeat_state(void) {
  g_vclock_heartbeat_armed = false;
  g_vclock_heartbeat_arm_count = 0;
  g_vclock_heartbeat_arm_failures = 0;
  g_vclock_heartbeat_fire_count = 0;
  g_vclock_heartbeat_vclock_ticks = 0;
  g_vclock_heartbeat_ocxo1_rollover_updates_retired = 0;
  g_vclock_heartbeat_ocxo2_rollover_updates_retired = 0;
  g_vclock_heartbeat_last_vclock_hw16 = 0;
  g_vclock_heartbeat_last_ocxo1_hw16_retired = 0;
  g_vclock_heartbeat_last_ocxo2_hw16_retired = 0;
  g_vclock_heartbeat_last_vclock_counter32 = 0;
  g_vclock_heartbeat_last_ocxo1_counter32_retired = 0;
  g_vclock_heartbeat_last_ocxo2_counter32_retired = 0;

  g_ch2_implicit_rollover_count = 0;
  g_ch2_implicit_rollover_vclock_updates = 0;
  g_ch2_implicit_rollover_ocxo1_updates = 0;
  g_ch2_implicit_rollover_ocxo2_updates = 0;
  g_ch2_implicit_rollover_last_vclock_hw16 = 0;
  g_ch2_implicit_rollover_last_ocxo1_hw16 = 0;
  g_ch2_implicit_rollover_last_ocxo2_hw16 = 0;
  g_ch2_implicit_rollover_last_vclock_counter32 = 0;
  g_ch2_implicit_rollover_last_ocxo1_counter32 = 0;
  g_ch2_implicit_rollover_last_ocxo2_counter32 = 0;

  g_vclock_ch2_one_second_enabled = false;
  g_vclock_ch2_one_second_next_counter32 = 0;
  g_vclock_ch2_one_second_enable_count = 0;
  g_vclock_ch2_one_second_reset_count = 0;
  g_vclock_ch2_one_second_service_count = 0;
  g_vclock_ch2_one_second_enqueue_count = 0;
  g_vclock_ch2_one_second_late_count = 0;
  g_vclock_ch2_one_second_late_max_ticks = 0;
  g_vclock_ch2_one_second_catchup_count = 0;
  g_vclock_ch2_one_second_drop_count = 0;
  g_vclock_ch2_one_second_last_target_counter32 = 0;
  g_vclock_ch2_one_second_last_service_counter32 = 0;
  g_vclock_ch2_one_second_last_dwt = 0;
  g_vclock_ch2_one_second_epoch_enable_count = 0;
  g_vclock_ch2_one_second_epoch_base_counter32 = 0;
  g_vclock_ch2_one_second_epoch_service_counter32 = 0;
  g_vclock_ch2_one_second_epoch_target_counter32 = 0;
  g_vclock_ch2_one_second_epoch_tick_mod_seed = 0;
  g_vclock_ch2_one_second_epoch_remaining_cells = 0;
  g_vclock_ch2_one_second_epoch_residual_ticks = 0;
  g_vclock_ch2_one_second_heartbeat_enable_count = 0;
  g_vclock_heartbeat_one_second_legacy_count = 0;
  g_vclock_heartbeat_one_second_handoff_skip_count = 0;
  g_vclock_heartbeat_one_second_fallback_count = 0;

  g_vclock_ch2_smartzero_seeded = false;
  g_vclock_ch2_smartzero_next_counter32 = 0;
  g_vclock_ch2_smartzero_reset_count = 0;
  g_vclock_ch2_smartzero_seed_count = 0;
  g_vclock_ch2_smartzero_deactivate_count = 0;
  g_vclock_ch2_smartzero_service_count = 0;
  g_vclock_ch2_smartzero_transaction_skip_count = 0;
  g_vclock_ch2_smartzero_feed_count = 0;
  g_vclock_ch2_smartzero_late_count = 0;
  g_vclock_ch2_smartzero_late_max_ticks = 0;
  g_vclock_ch2_smartzero_resync_count = 0;
  g_vclock_ch2_smartzero_drop_count = 0;
  g_vclock_ch2_smartzero_waiting_for_cps_count = 0;
  g_vclock_ch2_smartzero_first_sample_count = 0;
  g_vclock_ch2_smartzero_accepted_count = 0;
  g_vclock_ch2_smartzero_rejected_dwt_count = 0;
  g_vclock_ch2_smartzero_rejected_counter_count = 0;
  g_vclock_ch2_smartzero_last_target_counter32 = 0;
  g_vclock_ch2_smartzero_last_service_counter32 = 0;
  g_vclock_ch2_smartzero_last_dwt = 0;
  g_vclock_ch2_smartzero_last_late_ticks = 0;
  g_vclock_heartbeat_smartzero_authority_retired_count = 0;

  g_vclock_ch2_fact_drain_service_count = 0;
  g_vclock_ch2_fact_drain_arm_count = 0;
  g_vclock_heartbeat_fact_drain_authority_retired_count = 0;
  g_vclock_ch2_epoch_native_service_count = 0;
  g_vclock_ch2_epoch_native_publish_count = 0;
  g_vclock_ch2_epoch_native_bad_backdate_count = 0;
  g_vclock_ch2_epoch_native_last_reject_reason = VCLOCK_CH2_EPOCH_REJECT_NONE;
  g_vclock_ch2_epoch_native_dispatch_arm_count = 0;
  g_vclock_ch2_epoch_native_dispatch_arm_fail_count = 0;
  g_vclock_ch2_epoch_native_last_selected_counter32 = 0;
  g_vclock_ch2_epoch_native_last_service_counter32 = 0;
  g_vclock_ch2_epoch_native_last_backdate_ticks = 0;
  g_vclock_ch2_epoch_native_last_backdate_cycles = 0;
  g_vclock_ch2_epoch_native_last_sacred_dwt = 0;
  g_vclock_ch2_epoch_native_last_sequence = 0;
  g_vclock_heartbeat_epoch_authority_retired_count = 0;
  g_vclock_heartbeat_epoch_pending_skip_count = 0;

  isr_sanity_reset(g_isr_sanity_vclock_ch2);
  isr_sanity_reset(g_isr_sanity_ocxo1);
  isr_sanity_reset(g_isr_sanity_ocxo2);
  isr_sanity_reset(g_isr_sanity_pps);
  g_isr_sanity_vclock_ch2_prev_valid = false;
  g_isr_sanity_vclock_ch2_prev_counter32 = 0;
  g_isr_sanity_vclock_ch2_prev_dwt = 0;
  g_isr_sanity_pps_prev_valid = false;
  g_isr_sanity_pps_prev_counter32 = 0;
  g_isr_sanity_pps_prev_dwt = 0;

}

static void runtime_reset_ocxo_fact_rings(void) {
  if (!OCXO1_DISABLED) ocxo_fact_ring_reset(g_ocxo1_ctx);
  if (!OCXO2_DISABLED) ocxo_fact_ring_reset(g_ocxo2_ctx);
}

static void runtime_reset_for_init(void) {
  runtime_init_subscribers();
  runtime_reset_pps_state();
  runtime_reset_snapshot_and_bridge_state();
  runtime_reset_clock32_state();
  runtime_reset_smartzero_state();
  runtime_bootstrap_synthetic_clocks_if_ready();
  runtime_reset_qtimer_host_state();
  runtime_reset_vclock_heartbeat_state();
  vclock_fact_ring_reset();
  runtime_reset_ocxo_fact_rings();
  cadence_regression_reset_all();
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  runtime_reset_for_init();

  g_interrupt_runtime_ready = true;
  (void)vclock_heartbeat_arm_timepop();
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  // VCLOCK/TimePop QTimer1 is the sovereign timing rail.  Keep the entire
  // shared QTimer1 vector at highest priority; same-vector logical ordering
  // is handled inside TimePop/process_interrupt.
  NVIC_SET_PRIORITY(IRQ_QTIMER1, INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);
  g_step0_qtimer1_priority_applied = INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY;
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);
  g_step0_qtimer1_irq_enabled_by_interrupt = true;

  if (!OCXO1_DISABLED) {
    attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER2, INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);
    g_step0_qtimer2_priority_applied = INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY;
    NVIC_ENABLE_IRQ(IRQ_QTIMER2);
    g_step0_qtimer2_irq_enabled_by_interrupt = true;
  } else {
    g_step0_qtimer2_priority_applied = 0xFFFFFFFFUL;
    g_step0_qtimer2_irq_enabled_by_interrupt = false;
  }

  if (!OCXO2_DISABLED) {
    attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER3, INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);
    g_step0_qtimer3_priority_applied = INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY;
    NVIC_ENABLE_IRQ(IRQ_QTIMER3);
    g_step0_qtimer3_irq_enabled_by_interrupt = true;
  } else {
    g_step0_qtimer3_priority_applied = 0xFFFFFFFFUL;
    g_step0_qtimer3_irq_enabled_by_interrupt = false;
  }

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  // PPS GPIO is now a witness/selector, not the highest-priority timing
  // interpolation rail.  Keep it below QTimer1 so VCLOCK/TimePop event facts
  // are not delayed by PPS witness work.
  NVIC_SET_PRIORITY(IRQ_GPIO6789, INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);
  g_step0_gpio6789_priority_applied = INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY;
  g_step0_gpio6789_configured_by_interrupt = true;

  g_interrupt_irqs_enabled = true;
}


struct payload_prefix_t {
  Payload& p;
  const char* prefix;
  char key[96];

  payload_prefix_t(Payload& payload, const char* prefix_name)
      : p(payload), prefix(prefix_name ? prefix_name : "") {}

  void make_key(const char* suffix) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix ? suffix : "");
  }

  void add_bool(const char* suffix, bool value) {
    make_key(suffix);
    p.add(key, value);
  }

  void add_u32(const char* suffix, uint32_t value) {
    make_key(suffix);
    p.add(key, value);
  }

  void add_i32(const char* suffix, int32_t value) {
    make_key(suffix);
    p.add(key, value);
  }

  void add_u64(const char* suffix, uint64_t value) {
    make_key(suffix);
    p.add(key, value);
  }

  void add_i64(const char* suffix, int64_t value) {
    make_key(suffix);
    p.add(key, value);
  }

  void add_str(const char* suffix, const char* value) {
    make_key(suffix);
    p.add(key, value ? value : "");
  }
};

static void add_isr_sanity_payload(Payload& p,
                                   const char* prefix,
                                   const isr_sanity_diag_t& s) {
  payload_prefix_t out(p, prefix);

  out.add_u32("cntr_correct_count", s.cntr_correct_count);
  out.add_u32("cntr_incorrect_count", s.cntr_incorrect_count);
  out.add_u32("cntr_expected", s.cntr_expected);
  out.add_u32("cntr_actual", s.cntr_actual);

  out.add_u32("dwt_correct_count", s.dwt_correct_count);
  out.add_u32("dwt_incorrect_count", s.dwt_incorrect_count);
  out.add_u32("dwt_expected", s.dwt_expected);
  out.add_u32("dwt_actual", s.dwt_actual);
}

static void add_bridge_stats_payload(Payload& p,
                                     const char* prefix,
                                     const bridge_anchor_stats_t& s) {
  payload_prefix_t out(p, prefix);

  out.add_u32("anchor_latest_count", s.latest_count);
  out.add_u32("anchor_previous_count", s.previous_count);
  out.add_u32("anchor_older_count", s.older_count);
  out.add_u32("anchor_failed_count", s.failed_count);
  out.add_u32("anchor_last_selection_kind", s.last_selection_kind);
  out.add_u32("anchor_last_age_slots", s.last_anchor_age_slots);
  out.add_u32("anchor_last_sequence_used", s.last_anchor_sequence_used);
  out.add_u64("anchor_last_ns_delta", s.last_anchor_ns_delta);
  out.add_u32("anchor_last_failure_mask", s.last_anchor_failure_mask);
}

static void add_regression_payload(Payload& p,
                                   const char* prefix,
                                   const cadence_regression_lane_t& r) {
  payload_prefix_t out(p, prefix);
  const cadence_regression_result_t& last = r.last_result;

  out.add_bool("regression_enabled",
               r.kind == interrupt_subscriber_kind_t::VCLOCK
                   ? VCLOCK_LINEAR_REGRESSION_ENABLED
                   : OCXO_LINEAR_REGRESSION_ENABLED);
  out.add_u32("regression_samples_per_second", REGRESSION_SAMPLES_PER_SECOND);
  out.add_u32("regression_sample_total_count", r.sample_total_count);
  out.add_u32("regression_window_sample_count", r.sample_count);
  out.add_u32("regression_reset_count", r.reset_count);
  out.add_u32("regression_insufficient_count", r.insufficient_count);
  out.add_u32("regression_counter_discontinuity_count",
              r.counter_discontinuity_count);

  out.add_bool("regression_last_valid", last.valid);
  out.add_u32("regression_last_sequence", last.sequence);
  out.add_u32("regression_last_sample_count", last.sample_count);
  out.add_u32("regression_last_observed_dwt", last.observed_dwt_at_event);
  out.add_u32("regression_last_inferred_dwt", last.inferred_dwt_at_event);
  out.add_i32("regression_last_inferred_minus_observed_cycles",
              last.inferred_minus_observed_cycles);
  out.add_u32("regression_last_target_counter32",
              last.target_counter32_at_event);
  out.add_u32("regression_last_target_hardware16",
              (uint32_t)last.target_hardware16_at_event);
  out.add_u32("regression_last_observed_hardware16",
              (uint32_t)last.observed_hardware16_at_event);
  out.add_u64("regression_last_slope_q16_cycles_per_sample",
              last.slope_q16_cycles_per_sample);
  out.add_i64("regression_last_slope_delta_q16_cycles_per_sample",
              last.slope_delta_q16_cycles_per_sample);
  out.add_i32("regression_last_fit_error_mean_q16_cycles",
              last.fit_error_mean_q16_cycles);
  out.add_u32("regression_last_fit_error_stddev_q16_cycles",
              last.fit_error_stddev_q16_cycles);
  out.add_i32("regression_last_fit_error_min_cycles",
              last.fit_error_min_cycles);
  out.add_i32("regression_last_fit_error_max_cycles",
              last.fit_error_max_cycles);
  out.add_u32("regression_last_fit_error_gt_plus4_count",
              last.fit_error_gt_plus4_count);
  out.add_u32("regression_last_fit_error_lt_minus4_count",
              last.fit_error_lt_minus4_count);
  out.add_u32("regression_last_fit_error_abs_gt4_count",
              last.fit_error_abs_gt4_count);
}

static uint32_t regression_abs_i32(int32_t value) {
  return (uint32_t)(value < 0 ? -(int64_t)value : (int64_t)value);
}

static interrupt_subscriber_kind_t regression_kind_from_lane_arg(const char* lane) {
  if (!lane || !*lane) return interrupt_subscriber_kind_t::NONE;
  if (!strcasecmp(lane, "VCLOCK") || !strcasecmp(lane, "VCLK")) {
    return interrupt_subscriber_kind_t::VCLOCK;
  }
  if (!strcasecmp(lane, "OCXO1") || !strcasecmp(lane, "O1")) {
    return interrupt_subscriber_kind_t::OCXO1;
  }
  if (!strcasecmp(lane, "OCXO2") || !strcasecmp(lane, "O2")) {
    return interrupt_subscriber_kind_t::OCXO2;
  }
  return interrupt_subscriber_kind_t::NONE;
}

static bool regression_live_fit_params(const cadence_regression_lane_t& r,
                                       double& slope,
                                       double& intercept) {
  const uint32_t n = r.sample_count;
  if (n < REGRESSION_MIN_SAMPLE_COUNT || n > REGRESSION_SAMPLES_PER_SECOND) {
    slope = 0.0;
    intercept = 0.0;
    return false;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;

  for (uint32_t i = 0; i < n; i++) {
    const double x = (double)i;
    const double y = (double)r.observed_dwt_rel[i];
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }

  const double denom = (double)n * sum_xx - sum_x * sum_x;
  if (denom == 0.0) {
    slope = 0.0;
    intercept = 0.0;
    return false;
  }

  slope = ((double)n * sum_xy - sum_x * sum_y) / denom;
  intercept = (sum_y - slope * sum_x) / (double)n;
  return true;
}

static int32_t regression_live_fit_error_cycles(const cadence_regression_lane_t& r,
                                                uint32_t index,
                                                double slope,
                                                double intercept) {
  if (index >= r.sample_count || index >= REGRESSION_SAMPLES_PER_SECOND) return 0;
  const double estimate = intercept + slope * (double)index;
  const double err_d = estimate - (double)r.observed_dwt_rel[index];
  return regression_round_i32(err_d);
}

static void regression_append_csv_u32(char* buf, size_t len,
                                      uint32_t& used,
                                      uint32_t value) {
  if (used >= len) return;
  const int n = snprintf(buf + used, len - used,
                         used == 0 ? "%lu" : ",%lu",
                         (unsigned long)value);
  if (n > 0) {
    const uint32_t add = (uint32_t)n;
    used = (add >= (len - used)) ? (uint32_t)(len - 1U) : (used + add);
  }
}

static void regression_append_csv_i32(char* buf, size_t len,
                                      uint32_t& used,
                                      int32_t value) {
  if (used >= len) return;
  const int n = snprintf(buf + used, len - used,
                         used == 0 ? "%ld" : ",%ld",
                         (long)value);
  if (n > 0) {
    const uint32_t add = (uint32_t)n;
    used = (add >= (len - used)) ? (uint32_t)(len - 1U) : (used + add);
  }
}

static void add_regression_sample_scalar(Payload& p,
                                         const cadence_regression_lane_t& r,
                                         uint32_t index,
                                         uint32_t ordinal,
                                         bool fit_valid,
                                         double slope,
                                         double intercept) {
  if (index >= r.sample_count || index >= REGRESSION_SAMPLES_PER_SECOND) return;

  char key[64];
  const uint32_t target_counter32 =
      r.base_counter32 + index * REGRESSION_COUNTER_DELTA_TICKS;
  const int32_t observed_rel = r.observed_dwt_rel[index];
  const uint32_t observed_dwt = r.base_observed_dwt + (uint32_t)observed_rel;
  const int32_t fit_error = fit_valid
      ? regression_live_fit_error_cycles(r, index, slope, intercept)
      : 0;

  snprintf(key, sizeof(key), "sample_%02lu_i", (unsigned long)ordinal);
  p.add(key, index);
  snprintf(key, sizeof(key), "sample_%02lu_target_counter32", (unsigned long)ordinal);
  p.add(key, target_counter32);
  snprintf(key, sizeof(key), "sample_%02lu_observed_dwt_rel", (unsigned long)ordinal);
  p.add(key, observed_rel);
  snprintf(key, sizeof(key), "sample_%02lu_observed_dwt", (unsigned long)ordinal);
  p.add(key, observed_dwt);
  snprintf(key, sizeof(key), "sample_%02lu_fit_valid", (unsigned long)ordinal);
  p.add(key, fit_valid);
  snprintf(key, sizeof(key), "sample_%02lu_fit_error_cycles", (unsigned long)ordinal);
  p.add(key, fit_error);
  snprintf(key, sizeof(key), "sample_%02lu_abs_fit_error_cycles", (unsigned long)ordinal);
  p.add(key, fit_valid ? regression_abs_i32(fit_error) : 0U);
}

static void add_regression_extreme_csv(Payload& p,
                                       const cadence_regression_lane_t& r,
                                       bool best,
                                       bool fit_valid,
                                       double slope,
                                       double intercept) {
  uint32_t chosen[REGRESSION_SAMPLE_REPORT_EXTREME_LIMIT]{};
  uint32_t scores[REGRESSION_SAMPLE_REPORT_EXTREME_LIMIT]{};
  uint32_t chosen_count = 0;

  if (fit_valid) {
    for (uint32_t i = 0; i < r.sample_count && i < REGRESSION_SAMPLES_PER_SECOND; i++) {
      const int32_t err = regression_live_fit_error_cycles(r, i, slope, intercept);
      const uint32_t score = regression_abs_i32(err);
      uint32_t pos = chosen_count;
      if (best) {
        while (pos > 0 && score < scores[pos - 1U]) pos--;
      } else {
        while (pos > 0 && score > scores[pos - 1U]) pos--;
      }
      if (pos >= REGRESSION_SAMPLE_REPORT_EXTREME_LIMIT) continue;
      if (chosen_count < REGRESSION_SAMPLE_REPORT_EXTREME_LIMIT) chosen_count++;
      for (uint32_t j = chosen_count - 1U; j > pos; j--) {
        chosen[j] = chosen[j - 1U];
        scores[j] = scores[j - 1U];
      }
      chosen[pos] = i;
      scores[pos] = score;
    }
  }

  char idx_csv[96] = {};
  char abs_csv[96] = {};
  char err_csv[96] = {};
  uint32_t idx_used = 0, abs_used = 0, err_used = 0;
  for (uint32_t i = 0; i < chosen_count; i++) {
    const uint32_t idx = chosen[i];
    const int32_t err = regression_live_fit_error_cycles(r, idx, slope, intercept);
    regression_append_csv_u32(idx_csv, sizeof(idx_csv), idx_used, idx);
    regression_append_csv_u32(abs_csv, sizeof(abs_csv), abs_used, scores[i]);
    regression_append_csv_i32(err_csv, sizeof(err_csv), err_used, err);
  }

  p.add(best ? "best_abs_error_count" : "worst_abs_error_count", chosen_count);
  p.add(best ? "best_abs_error_indices" : "worst_abs_error_indices", idx_csv);
  p.add(best ? "best_abs_error_abs_cycles" : "worst_abs_error_abs_cycles", abs_csv);
  p.add(best ? "best_abs_error_cycles" : "worst_abs_error_cycles", err_csv);
}

// ============================================================================
// Commands
// ============================================================================

static void add_priority_payload(Payload& p) {
  p.add("qtimer1_sovereign_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);
  p.add("qtimer1_sovereign_priority_applied", g_step0_qtimer1_priority_applied);
  p.add("qtimer1_sovereign_priority_ok",
        g_step0_qtimer1_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);

  p.add("qtimer2_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);
  p.add("qtimer2_priority_applied", g_step0_qtimer2_priority_applied);
  p.add("qtimer2_priority_ok",
        OCXO1_DISABLED ||
        g_step0_qtimer2_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);

  p.add("qtimer3_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);
  p.add("qtimer3_priority_applied", g_step0_qtimer3_priority_applied);
  p.add("qtimer3_priority_ok",
        OCXO2_DISABLED ||
        g_step0_qtimer3_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);

  p.add("gpio6789_priority_expected", INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);
  p.add("gpio6789_priority_applied", g_step0_gpio6789_priority_applied);
  p.add("gpio6789_priority_ok",
        g_step0_gpio6789_priority_applied == INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);

  p.add("irq_qtimer1_enabled_by_interrupt", g_step0_qtimer1_irq_enabled_by_interrupt);
  p.add("irq_qtimer2_enabled_by_interrupt", g_step0_qtimer2_irq_enabled_by_interrupt);
  p.add("irq_qtimer3_enabled_by_interrupt", g_step0_qtimer3_irq_enabled_by_interrupt);
  p.add("irq_gpio6789_configured_by_interrupt", g_step0_gpio6789_configured_by_interrupt);
}

static void add_runtime_payload(Payload& p) {
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("ocxo_disable_experiment", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_disabled_count", OCXO_DISABLED_COUNT);
  p.add("ocxo1_disabled", OCXO1_DISABLED);
  p.add("ocxo2_disabled", OCXO2_DISABLED);
  p.add("ocxo1_irq_qtimer2_disabled", OCXO1_DISABLED);
  p.add("ocxo2_irq_qtimer3_disabled", OCXO2_DISABLED);
  p.add("ocxo_smartzero_surrogates_enabled", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_quiet_phase_sampling_enabled", OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo_rollover_only_mode", !OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo_ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  p.add("ocxo_ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  p.add("ocxo1_quiet_phase_ticks", OCXO1_QUIET_PHASE_TICKS);
  p.add("ocxo1_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO1_QUIET_PHASE_TICKS));
  p.add("ocxo2_quiet_phase_ticks", OCXO2_QUIET_PHASE_TICKS);
  p.add("ocxo2_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO2_QUIET_PHASE_TICKS));
  p.add("timing_arch_step", "CH2_NATIVE_VCLOCK_FIRST_1S_LEGACY_GRID");
  p.add("timing_arch_behavior_changed", true);
  p.add("isr_sanity_witness_enabled", true);
  p.add("isr_sanity_witness_policy", "REPORT_ONLY_NO_REPAIR_NO_VETO");
  p.add("timing_arch_vclock_authority", "QTIMER1_CH2_NATIVE_ROLLOVER_RELAY_EPOCH_1S_SMARTZERO_DRAIN_ARM");
  p.add("timing_arch_ocxo_model", "LOCAL_QTIMER_ONE_SECOND_EDGE_CAPTURE");
  p.add("timing_arch_ocxo_migration_stage", "EMA_DWT_AUTHORITY");
  p.add("subscriber_count", g_subscriber_count);
  p.add("single_cadence_agent", false);
  p.add("vclock_heartbeat_isr_mode", true);
  p.add("ch2_implicit_rollover_enabled", CH2_IMPLICIT_ROLLOVER_ENABLED);
  p.add("ch2_implicit_rollover_policy",
        "PASSIVE_COEXISTS_WITH_VCLOCK_HEARTBEAT_AND_OCXO_LOCAL_CADENCE");
  p.add("vclock_heartbeat_retained", true);
  p.add("vclock_heartbeat_rollover_owner", false);
  p.add("ocxo_native_1khz_cadence_retired", true);
  p.add("ocxo_compare_live_requires_enabled_and_flag", true);
  p.add("lane_report_command", "INTERRUPT.REPORT_LANES");
  p.add("single_lane_report_command", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
  p.add("regression_samples_report_command", "disabled");
}

static void add_ocxo_cadence_report_payload(Payload& p,
                                           const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  payload_prefix_t out(p, ctx.prefix);

  out.add_bool("cadence_enabled", lane.cadence_enabled);
  out.add_bool("cadence_armed", lane.cadence_armed);
  out.add_bool("cadence_epoch_valid", lane.cadence_epoch_valid);
  out.add_u32("cadence_epoch_counter32", lane.cadence_epoch_counter32);
  out.add_bool("cadence_sample_phase_valid", lane.cadence_sample_phase_valid);
  out.add_u32("cadence_sample_phase_ticks", lane.cadence_sample_phase_ticks);
  out.add_u32("cadence_sample_phase_us", lane.cadence_sample_phase_us);
  out.add_u32("cadence_sample_phase_ns", lane.cadence_sample_phase_ns);
  out.add_u32("cadence_sample_period_ticks", lane.cadence_sample_period_ticks);
  out.add_u32("cadence_next_counter32", lane.cadence_next_counter32);
  out.add_u32("cadence_fire_count", lane.cadence_fire_count);
  out.add_u32("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  out.add_u32("cadence_false_irq_count", lane.cadence_false_irq_count);
  out.add_u32("cadence_last_reason", lane.cadence_last_reason);
  out.add_str("cadence_last_reason_name",
              ocxo_cadence_reason_name(lane.cadence_last_reason));
}

static void add_vclock_heartbeat_payload(Payload& p) {
  p.add("vclock_heartbeat_period_ns", (uint64_t)VCLOCK_HEARTBEAT_PERIOD_NS);
  p.add("vclock_heartbeat_armed", g_vclock_heartbeat_armed);
  p.add("vclock_heartbeat_arm_count", g_vclock_heartbeat_arm_count);
  p.add("vclock_heartbeat_arm_failures", g_vclock_heartbeat_arm_failures);
  p.add("vclock_heartbeat_fire_count", g_vclock_heartbeat_fire_count);
  p.add("vclock_heartbeat_vclock_ticks", g_vclock_heartbeat_vclock_ticks);
  p.add("vclock_heartbeat_ocxo1_rollover_updates_retired", g_vclock_heartbeat_ocxo1_rollover_updates_retired);
  p.add("vclock_heartbeat_ocxo2_rollover_updates_retired", g_vclock_heartbeat_ocxo2_rollover_updates_retired);
  p.add("vclock_heartbeat_last_vclock_counter32", g_vclock_heartbeat_last_vclock_counter32);
  p.add("vclock_heartbeat_ocxo_rollover_retired", true);
  p.add("vclock_heartbeat_one_second_authority_retired", true);
  p.add("vclock_heartbeat_one_second_legacy_count", g_vclock_heartbeat_one_second_legacy_count);
  p.add("vclock_heartbeat_one_second_handoff_skip_count", g_vclock_heartbeat_one_second_handoff_skip_count);
  p.add("vclock_heartbeat_one_second_fallback_count", g_vclock_heartbeat_one_second_fallback_count);
  p.add("vclock_heartbeat_smartzero_authority_retired", true);
  p.add("vclock_heartbeat_smartzero_authority_retired_count", g_vclock_heartbeat_smartzero_authority_retired_count);
  p.add("vclock_heartbeat_fact_drain_authority_retired", true);
  p.add("vclock_heartbeat_fact_drain_authority_retired_count", g_vclock_heartbeat_fact_drain_authority_retired_count);
  p.add("vclock_heartbeat_epoch_authority_retired", true);
  p.add("vclock_heartbeat_epoch_authority_retired_count", g_vclock_heartbeat_epoch_authority_retired_count);
  p.add("vclock_heartbeat_epoch_pending_skip_count", g_vclock_heartbeat_epoch_pending_skip_count);
  p.add("vclock_ch2_epoch_native_enabled", VCLOCK_CH2_EPOCH_NATIVE_ENABLED);
  p.add("vclock_ch2_epoch_native_service_count", g_vclock_ch2_epoch_native_service_count);
  p.add("vclock_ch2_epoch_native_publish_count", g_vclock_ch2_epoch_native_publish_count);
  p.add("vclock_ch2_epoch_native_bad_backdate_count", g_vclock_ch2_epoch_native_bad_backdate_count);
  p.add("vclock_ch2_epoch_native_last_reject_reason", g_vclock_ch2_epoch_native_last_reject_reason);
  p.add("vclock_ch2_epoch_native_last_reject_reason_name", vclock_ch2_epoch_reject_reason_name(g_vclock_ch2_epoch_native_last_reject_reason));
  p.add("vclock_ch2_epoch_native_dispatch_arm_count", g_vclock_ch2_epoch_native_dispatch_arm_count);
  p.add("vclock_ch2_epoch_native_dispatch_arm_fail_count", g_vclock_ch2_epoch_native_dispatch_arm_fail_count);
  p.add("vclock_ch2_epoch_native_last_sequence", g_vclock_ch2_epoch_native_last_sequence);
  p.add("vclock_ch2_epoch_native_last_selected_counter32", g_vclock_ch2_epoch_native_last_selected_counter32);
  p.add("vclock_ch2_epoch_native_last_service_counter32", g_vclock_ch2_epoch_native_last_service_counter32);
  p.add("vclock_ch2_epoch_native_last_backdate_ticks", g_vclock_ch2_epoch_native_last_backdate_ticks);
  p.add("vclock_ch2_epoch_native_last_backdate_cycles", g_vclock_ch2_epoch_native_last_backdate_cycles);
  p.add("vclock_ch2_epoch_native_last_sacred_dwt", g_vclock_ch2_epoch_native_last_sacred_dwt);
  p.add("vclock_ch2_fact_drain_native_enabled", true);
  p.add("vclock_ch2_fact_drain_service_count", g_vclock_ch2_fact_drain_service_count);
  p.add("vclock_ch2_fact_drain_arm_count", g_vclock_ch2_fact_drain_arm_count);

  p.add("vclock_ch2_smartzero_native_enabled", VCLOCK_CH2_SMARTZERO_NATIVE_ENABLED);
  p.add("vclock_ch2_smartzero_seeded", g_vclock_ch2_smartzero_seeded);
  p.add("vclock_ch2_smartzero_next_counter32", g_vclock_ch2_smartzero_next_counter32);
  p.add("vclock_ch2_smartzero_reset_count", g_vclock_ch2_smartzero_reset_count);
  p.add("vclock_ch2_smartzero_seed_count", g_vclock_ch2_smartzero_seed_count);
  p.add("vclock_ch2_smartzero_deactivate_count", g_vclock_ch2_smartzero_deactivate_count);
  p.add("vclock_ch2_smartzero_service_count", g_vclock_ch2_smartzero_service_count);
  p.add("vclock_ch2_smartzero_transaction_skip_count", g_vclock_ch2_smartzero_transaction_skip_count);
  p.add("vclock_ch2_smartzero_feed_count", g_vclock_ch2_smartzero_feed_count);
  p.add("vclock_ch2_smartzero_late_count", g_vclock_ch2_smartzero_late_count);
  p.add("vclock_ch2_smartzero_late_max_ticks", g_vclock_ch2_smartzero_late_max_ticks);
  p.add("vclock_ch2_smartzero_resync_count", g_vclock_ch2_smartzero_resync_count);
  p.add("vclock_ch2_smartzero_drop_count", g_vclock_ch2_smartzero_drop_count);
  p.add("vclock_ch2_smartzero_waiting_for_cps_count", g_vclock_ch2_smartzero_waiting_for_cps_count);
  p.add("vclock_ch2_smartzero_first_sample_count", g_vclock_ch2_smartzero_first_sample_count);
  p.add("vclock_ch2_smartzero_accepted_count", g_vclock_ch2_smartzero_accepted_count);
  p.add("vclock_ch2_smartzero_rejected_dwt_count", g_vclock_ch2_smartzero_rejected_dwt_count);
  p.add("vclock_ch2_smartzero_rejected_counter_count", g_vclock_ch2_smartzero_rejected_counter_count);
  p.add("vclock_ch2_smartzero_last_target_counter32", g_vclock_ch2_smartzero_last_target_counter32);
  p.add("vclock_ch2_smartzero_last_service_counter32", g_vclock_ch2_smartzero_last_service_counter32);
  p.add("vclock_ch2_smartzero_last_dwt", g_vclock_ch2_smartzero_last_dwt);
  p.add("vclock_ch2_smartzero_last_late_ticks", g_vclock_ch2_smartzero_last_late_ticks);

  p.add("vclock_ch2_one_second_enabled", g_vclock_ch2_one_second_enabled);
  p.add("vclock_ch2_one_second_next_counter32", g_vclock_ch2_one_second_next_counter32);
  p.add("vclock_ch2_one_second_enable_count", g_vclock_ch2_one_second_enable_count);
  p.add("vclock_ch2_one_second_epoch_enable_count", g_vclock_ch2_one_second_epoch_enable_count);
  p.add("vclock_ch2_one_second_epoch_base_counter32", g_vclock_ch2_one_second_epoch_base_counter32);
  p.add("vclock_ch2_one_second_epoch_service_counter32", g_vclock_ch2_one_second_epoch_service_counter32);
  p.add("vclock_ch2_one_second_epoch_target_counter32", g_vclock_ch2_one_second_epoch_target_counter32);
  p.add("vclock_ch2_one_second_epoch_tick_mod_seed", g_vclock_ch2_one_second_epoch_tick_mod_seed);
  p.add("vclock_ch2_one_second_epoch_remaining_cells", g_vclock_ch2_one_second_epoch_remaining_cells);
  p.add("vclock_ch2_one_second_epoch_residual_ticks", g_vclock_ch2_one_second_epoch_residual_ticks);
  p.add("vclock_ch2_one_second_heartbeat_enable_count", g_vclock_ch2_one_second_heartbeat_enable_count);
  p.add("vclock_ch2_one_second_reset_count", g_vclock_ch2_one_second_reset_count);
  p.add("vclock_ch2_one_second_service_count", g_vclock_ch2_one_second_service_count);
  p.add("vclock_ch2_one_second_enqueue_count", g_vclock_ch2_one_second_enqueue_count);
  p.add("vclock_ch2_one_second_late_count", g_vclock_ch2_one_second_late_count);
  p.add("vclock_ch2_one_second_late_max_ticks", g_vclock_ch2_one_second_late_max_ticks);
  p.add("vclock_ch2_one_second_catchup_count", g_vclock_ch2_one_second_catchup_count);
  p.add("vclock_ch2_one_second_drop_count", g_vclock_ch2_one_second_drop_count);
  p.add("vclock_ch2_one_second_last_target_counter32", g_vclock_ch2_one_second_last_target_counter32);
  p.add("vclock_ch2_one_second_last_service_counter32", g_vclock_ch2_one_second_last_service_counter32);
  p.add("vclock_ch2_one_second_last_dwt", g_vclock_ch2_one_second_last_dwt);

  p.add("ch2_implicit_rollover_enabled", CH2_IMPLICIT_ROLLOVER_ENABLED);
  p.add("ch2_implicit_rollover_count", g_ch2_implicit_rollover_count);
  p.add("ch2_implicit_rollover_vclock_updates", g_ch2_implicit_rollover_vclock_updates);
  p.add("ch2_implicit_rollover_ocxo1_updates", g_ch2_implicit_rollover_ocxo1_updates);
  p.add("ch2_implicit_rollover_ocxo2_updates", g_ch2_implicit_rollover_ocxo2_updates);
  p.add("ch2_implicit_rollover_last_vclock_hw16", (uint32_t)g_ch2_implicit_rollover_last_vclock_hw16);
  p.add("ch2_implicit_rollover_last_ocxo1_hw16", (uint32_t)g_ch2_implicit_rollover_last_ocxo1_hw16);
  p.add("ch2_implicit_rollover_last_ocxo2_hw16", (uint32_t)g_ch2_implicit_rollover_last_ocxo2_hw16);
  p.add("ch2_implicit_rollover_last_vclock_counter32", g_ch2_implicit_rollover_last_vclock_counter32);
  p.add("ch2_implicit_rollover_last_ocxo1_counter32", g_ch2_implicit_rollover_last_ocxo1_counter32);
  p.add("ch2_implicit_rollover_last_ocxo2_counter32", g_ch2_implicit_rollover_last_ocxo2_counter32);

  p.add("ocxo_cadence_interval_ticks", OCXO_CADENCE_INTERVAL_TICKS);
  p.add("ocxo_one_second_edge_interval_ticks", OCXO_WITNESS_ONE_SECOND_COUNTS);
  p.add("ocxo_cadence_samples_per_second", TICKS_PER_SECOND_EVENT);
  p.add("ocxo_quiet_phase_sampling_enabled", OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo_rollover_only_mode", !OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo_ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  p.add("ocxo_ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  p.add("ocxo_quiet_phase_period_ticks", OCXO_QUIET_PHASE_PERIOD_TICKS);
  p.add("ocxo1_quiet_phase_ticks", OCXO1_QUIET_PHASE_TICKS);
  p.add("ocxo1_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO1_QUIET_PHASE_TICKS));
  p.add("ocxo2_quiet_phase_ticks", OCXO2_QUIET_PHASE_TICKS);
  p.add("ocxo2_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO2_QUIET_PHASE_TICKS));
  add_ocxo_cadence_report_payload(p, g_ocxo1_ctx);
  add_ocxo_cadence_report_payload(p, g_ocxo2_ctx);
}
static void add_pps_payload(Payload& p) {
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);
  add_isr_sanity_payload(p, "pps_isr_sanity", g_isr_sanity_pps);

  p.add("pps_post_isr_deferred", true);
  p.add("pps_post_isr_pending", g_pps_post_isr.pending);
  p.add("pps_post_isr_arm_count", g_pps_post_isr.arm_count);
  p.add("pps_post_isr_drain_count", g_pps_post_isr.drain_count);
  p.add("pps_post_isr_overwrite_count", g_pps_post_isr.overwrite_count);
  p.add("pps_post_isr_asap_fail_count", g_pps_post_isr.asap_fail_count);
  p.add("pps_post_isr_last_sequence", g_pps_post_isr.last_sequence);
  p.add("pps_post_isr_last_capture_window_cycles",
        g_pps_post_isr.last_capture_window_cycles);

  p.add("pps_relay_owner", "process_interrupt");
  p.add("pps_relay_assert_in_isr", true);
  p.add("pps_relay_deassert_in_timepop", false);
  p.add("pps_relay_deassert_in_vclock_heartbeat", false);
  p.add("pps_relay_deassert_in_ch2", true);
  p.add("pps_relay_rearm_deassert_every_pps", true);
  p.add("pps_relay_off_ns", (uint64_t)PPS_RELAY_OFF_NS);
  p.add("pps_relay_off_cadence_ticks", PPS_RELAY_OFF_CADENCE_TICKS);
  p.add("pps_relay_pin_initialized", g_pps_relay_pin_initialized);
  p.add("pps_relay_timer_active", g_pps_relay_timer_active);
  p.add("pps_relay_deassert_arm_pending", g_pps_relay_deassert_arm_pending);
  p.add("pps_relay_deassert_countdown_ticks", g_pps_relay_deassert_countdown_ticks);
  p.add("pps_relay_assert_count", g_pps_relay_assert_count);
  p.add("pps_relay_deassert_count", g_pps_relay_deassert_count);
  p.add("pps_relay_deassert_arm_count", g_pps_relay_deassert_arm_count);
  p.add("pps_relay_deassert_arm_fail_count", g_pps_relay_deassert_arm_fail_count);
  p.add("pps_relay_deassert_arm_skip_count", g_pps_relay_deassert_arm_skip_count);
  p.add("pps_relay_last_assert_sequence", g_pps_relay_last_assert_sequence);
  p.add("pps_relay_ch2_tick_valid", g_pps_relay_ch2_tick_valid);
  p.add("pps_relay_ch2_last_counter32", g_pps_relay_ch2_last_counter32);
  p.add("pps_relay_ch2_tick_count", g_pps_relay_ch2_tick_count);
  p.add("pps_relay_ch2_catchup_count", g_pps_relay_ch2_catchup_count);

  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count",   g_pps_rebootstrap_count);
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);
  p.add("vclock_epoch_latch_counter32", g_vclock_epoch_latch.counter32_at_edge);
  p.add("vclock_epoch_latch_observed_counter32", g_vclock_epoch_latch.observed_counter32);
  p.add("vclock_epoch_latch_first_cadence_counter32", g_vclock_epoch_latch.first_cadence_counter32);
  p.add("vclock_epoch_latch_first_cadence_dwt", g_vclock_epoch_latch.first_cadence_dwt);
  p.add("vclock_epoch_latch_backdate_ticks", g_vclock_epoch_latch.backdate_ticks);
  p.add("vclock_epoch_latch_backdate_cycles", g_vclock_epoch_latch.backdate_cycles);
  p.add("vclock_epoch_latch_sacred_dwt", g_vclock_epoch_latch.sacred_dwt);
  p.add("vclock_ch2_epoch_native_enabled", VCLOCK_CH2_EPOCH_NATIVE_ENABLED);
  p.add("vclock_ch2_epoch_native_service_count", g_vclock_ch2_epoch_native_service_count);
  p.add("vclock_ch2_epoch_native_publish_count", g_vclock_ch2_epoch_native_publish_count);
  p.add("vclock_ch2_epoch_native_bad_backdate_count", g_vclock_ch2_epoch_native_bad_backdate_count);
  p.add("vclock_ch2_epoch_native_last_reject_reason", g_vclock_ch2_epoch_native_last_reject_reason);
  p.add("vclock_ch2_epoch_native_last_reject_reason_name", vclock_ch2_epoch_reject_reason_name(g_vclock_ch2_epoch_native_last_reject_reason));
  p.add("vclock_ch2_epoch_native_dispatch_arm_count", g_vclock_ch2_epoch_native_dispatch_arm_count);
  p.add("vclock_ch2_epoch_native_dispatch_arm_fail_count", g_vclock_ch2_epoch_native_dispatch_arm_fail_count);
  p.add("vclock_ch2_epoch_native_last_sequence", g_vclock_ch2_epoch_native_last_sequence);
  p.add("vclock_ch2_epoch_native_last_selected_counter32", g_vclock_ch2_epoch_native_last_selected_counter32);
  p.add("vclock_ch2_epoch_native_last_service_counter32", g_vclock_ch2_epoch_native_last_service_counter32);
  p.add("vclock_ch2_epoch_native_last_backdate_ticks", g_vclock_ch2_epoch_native_last_backdate_ticks);
  p.add("vclock_ch2_epoch_native_last_backdate_cycles", g_vclock_ch2_epoch_native_last_backdate_cycles);
  p.add("vclock_ch2_epoch_native_last_sacred_dwt", g_vclock_ch2_epoch_native_last_sacred_dwt);

  pps_t pps; pps_vclock_t pvc;
  store_load(pps, pvc);

  p.add("pps_sequence",          pps.sequence);
  p.add("pps_dwt_at_edge",       pps.dwt_at_edge);
  p.add("pps_counter32_at_edge", pps.counter32_at_edge);
  p.add("pps_ch3_at_edge",       (uint32_t)pps.ch3_at_edge);

  p.add("pps_vclock_sequence",          pvc.sequence);
  p.add("pps_vclock_dwt_at_edge",       pvc.dwt_at_edge);
  p.add("pps_vclock_counter32_at_edge", pvc.counter32_at_edge);
  p.add("pps_vclock_ch3_at_edge",       (uint32_t)pvc.ch3_at_edge);
  p.add("pps_vclock_phase_cycles", pps_vclock_phase_cycles_from_edges(pps, pvc));

  p.add("pps_edge_dispatch_registered", g_pps_edge_dispatch != nullptr);
}

static void add_epoch_capture_payload(Payload& p) {
  interrupt_epoch_capture_t epoch_cap{};
  const bool epoch_cap_ok = interrupt_last_epoch_capture(&epoch_cap);
  p.add("epoch_capture_available", epoch_cap_ok);
  p.add("epoch_capture_valid", epoch_cap.valid);
  p.add("epoch_capture_sequence", epoch_cap.sequence);
  p.add("epoch_capture_window_cycles", epoch_cap.capture_window_cycles);
  p.add("epoch_capture_max_window_cycles", EPOCH_CAPTURE_MAX_WINDOW_CYCLES);
  p.add("epoch_capture_vclock_read_offset_cycles", epoch_cap.vclock_read_offset_cycles);
  p.add("epoch_capture_vclock_dwt_at_edge", epoch_cap.vclock_dwt_at_edge);
  p.add("epoch_capture_vclock_valid", epoch_cap.vclock_capture_valid);
  p.add("epoch_capture_all_lanes_valid", epoch_cap.all_lanes_capture_valid);
  p.add("epoch_capture_vclock_hardware16_observed", (uint32_t)epoch_cap.vclock_hardware16_observed);
  p.add("epoch_capture_vclock_hardware16_selected", (uint32_t)epoch_cap.vclock_hardware16_selected);
  p.add("epoch_capture_vclock_counter32", epoch_cap.vclock_counter32);
  p.add("epoch_capture_ocxo1_counter32", epoch_cap.ocxo1_counter32);
  p.add("epoch_capture_ocxo2_counter32", epoch_cap.ocxo2_counter32);
}

static void add_dynamic_cps_payload(Payload& p) {
  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  p.add("dynamic_cps_owner", "CLOCKS_STATIC_PPS");
  p.add("dynamic_cps", dynamic_cps);
  p.add("dynamic_cps_valid", dynamic_cps != 0);
  p.add("dynamic_cps_pps_sequence", g_pps_gpio_heartbeat.edge_count);
  p.add("dynamic_cps_last_pvc_dwt_at_edge", g_pps_gpio_heartbeat.last_dwt);
  p.add("dynamic_cps_last_reseed_value", dynamic_cps);
  p.add("dynamic_cps_last_reseed_was_computed", dynamic_cps != 0);
}


static void add_smartzero_lane_payload(Payload& parent,
                                       const char* key,
                                       const interrupt_smartzero_lane_snapshot_t& z) {
  Payload p;
  p.add("kind", interrupt_subscriber_kind_str(z.kind));
  p.add("state", smartzero_lane_state_name(z.state));
  p.add("last_decision", smartzero_decision_name(z.last_decision));
  p.add("sample_count", z.sample_count);
  p.add("interval_attempt_count", z.interval_attempt_count);
  p.add("accepted_count", z.accepted_count);
  p.add("rejected_count", z.rejected_count);
  p.add("waiting_for_cps_count", z.waiting_for_cps_count);
  p.add("cps_used", z.cps_used);
  p.add("expected_interval_cycles", z.expected_interval_cycles);
  p.add("tolerance_cycles", z.tolerance_cycles);
  p.add("required_counter_delta_ticks", z.required_counter_delta_ticks);
  p.add("last_sample_dwt", z.last_sample_dwt);
  p.add("last_sample_counter32", z.last_sample_counter32);
  p.add("last_sample_hardware16", (uint32_t)z.last_sample_hardware16);
  p.add("previous_sample_dwt", z.previous_sample_dwt);
  p.add("previous_sample_counter32", z.previous_sample_counter32);
  p.add("previous_sample_hardware16", (uint32_t)z.previous_sample_hardware16);
  p.add("last_interval_cycles", z.last_interval_cycles);
  p.add("last_interval_error_cycles", z.last_interval_error_cycles);
  p.add("max_abs_interval_error_cycles", z.max_abs_interval_error_cycles);
  p.add("last_counter_delta_ticks", z.last_counter_delta_ticks);
  p.add("anchor_dwt", z.anchor_dwt);
  p.add("anchor_counter32", z.anchor_counter32);
  p.add("anchor_hardware16", (uint32_t)z.anchor_hardware16);
  p.add("anchor_pair_previous_dwt", z.anchor_pair_previous_dwt);
  p.add("anchor_pair_previous_counter32", z.anchor_pair_previous_counter32);
  p.add("arm_count", z.arm_count);
  p.add("fire_count", z.fire_count);
  p.add("next_target_counter32", z.next_target_counter32);
  parent.add_object(key, p);
}

static void add_smartzero_payload(Payload& p) {
  interrupt_smartzero_snapshot_t z{};
  (void)interrupt_smartzero_live_snapshot(&z);

  // Explicit live-acquisition surface.  This is NOT the installed epoch proof.
  p.add("live_smartzero_phase", smartzero_phase_name(z.phase));
  p.add("live_smartzero_running", z.running);
  p.add("live_smartzero_complete", z.complete);
  p.add("live_smartzero_aborted", z.aborted);
  p.add("live_smartzero_sequence", z.sequence);
  p.add("live_smartzero_begin_count", z.begin_count);
  p.add("live_smartzero_complete_count", z.complete_count);
  p.add("live_smartzero_abort_count", z.abort_count);
  p.add("live_smartzero_current_lane_index", z.current_lane_index);
  p.add("live_smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));
  p.add("live_smartzero_sample_rate_hz", z.sample_rate_hz);
  p.add("live_smartzero_counter_delta_ticks", z.counter_delta_ticks);
  p.add("live_smartzero_tolerance_cycles", z.tolerance_cycles);

  // Legacy aliases retained for existing tooling.  These names refer to the
  // live acquisition attempt only; CLOCKS reports the installed proof.
  p.add("smartzero_phase", smartzero_phase_name(z.phase));
  p.add("smartzero_running", z.running);
  p.add("smartzero_complete", z.complete);
  p.add("smartzero_aborted", z.aborted);
  p.add("smartzero_sequence", z.sequence);
  p.add("smartzero_begin_count", z.begin_count);
  p.add("smartzero_complete_count", z.complete_count);
  p.add("smartzero_abort_count", z.abort_count);
  p.add("smartzero_current_lane_index", z.current_lane_index);
  p.add("smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));
  p.add("smartzero_sample_rate_hz", z.sample_rate_hz);
  p.add("smartzero_counter_delta_ticks", z.counter_delta_ticks);
  p.add("smartzero_tolerance_cycles", z.tolerance_cycles);

  Payload live_lanes;
  add_smartzero_lane_payload(live_lanes, "vclock", z.lanes[0]);
  add_smartzero_lane_payload(live_lanes, "ocxo1", z.lanes[1]);
  add_smartzero_lane_payload(live_lanes, "ocxo2", z.lanes[2]);
  p.add_object("live_smartzero", live_lanes);

  Payload legacy_lanes;
  add_smartzero_lane_payload(legacy_lanes, "vclock", z.lanes[0]);
  add_smartzero_lane_payload(legacy_lanes, "ocxo1", z.lanes[1]);
  add_smartzero_lane_payload(legacy_lanes, "ocxo2", z.lanes[2]);
  p.add_object("smartzero", legacy_lanes);  // legacy live alias
}

static void add_vclock_clock32_payload(Payload& p, const char* prefix) {
  payload_prefix_t out(p, prefix);

  out.add_bool("clock32_zeroed", g_vclock_clock32.zeroed);
  out.add_u64("clock32_zero_ns", g_vclock_clock32.zero_ns);
  out.add_u32("clock32_zero_counter32", g_vclock_clock32.zero_counter32);
  out.add_u32("clock32_current", g_vclock_clock32.current_counter32);
  out.add_u32("clock32_hardware_low16_at_zero", (uint32_t)g_vclock_clock32.hardware_low16_at_zero);
  out.add_u32("clock32_hardware_low16_at_current", (uint32_t)g_vclock_clock32.hardware_low16_at_current);
  out.add_bool("clock32_hardware_anchor_valid", g_vclock_clock32.hardware_anchor_valid);
  out.add_u32("clock32_hardware_anchor_update_count", g_vclock_clock32.hardware_anchor_update_count);
  out.add_bool("clock32_pending_zero", g_vclock_clock32.pending_zero);
  out.add_u32("clock32_zero_count", g_vclock_clock32.zero_count);
  out.add_u32("clock32_minder_update_count", g_vclock_clock32.minder_update_count);
  out.add_u32("clock32_pending_zero_count", g_vclock_clock32.pending_zero_count);
}
static void add_ocxo_clock32_payload(Payload& p,
                                     const char* prefix,
                                     const synthetic_clock32_t& clock32) {
  payload_prefix_t out(p, prefix);

  out.add_bool("clock32_zeroed", clock32.zeroed);
  out.add_u64("clock32_zero_ns", clock32.zero_ns);
  out.add_u32("clock32_zero_counter32", clock32.zero_counter32);
  out.add_u32("clock32_current", clock32.current_counter32);
  out.add_bool("clock32_pending_zero", clock32.pending_zero);
  out.add_u32("clock32_zero_count", clock32.zero_count);
  out.add_u32("clock32_minder_update_count", clock32.minder_update_count);
}
static void add_runtime_lane_summary(Payload& p,
                                     const char* prefix,
                                     const interrupt_subscriber_runtime_t* rt) {
  payload_prefix_t out(p, prefix);

  out.add_bool("subscribed", rt ? rt->subscribed : false);
  out.add_bool("active", rt ? rt->active : false);
  out.add_bool("has_fired", rt ? rt->has_fired : false);
  out.add_u32("start_count", rt ? rt->start_count : 0U);
  out.add_u32("stop_count", rt ? rt->stop_count : 0U);
  out.add_u32("irq_count", rt ? rt->irq_count : 0U);
  out.add_u32("dispatch_count", rt ? rt->dispatch_count : 0U);
  out.add_u32("event_count", rt ? rt->event_count : 0U);
  if (rt && rt->has_fired) {
    out.add_u32("last_event_dwt", rt->last_event.dwt_at_event);
    out.add_u32("last_event_counter32", rt->last_event.counter32_at_event);
    out.add_u64("last_event_gnss_ns_at_event", rt->last_event.gnss_ns_at_event);
    out.add_bool("last_event_gnss_ns_available", rt->last_event.gnss_ns_at_event != 0);
    out.add_u32("last_event_status", (uint32_t)((uint8_t)rt->last_event.status));
    out.add_u64("last_diag_gnss_ns_at_event", rt->last_diag.gnss_ns_at_event);
    out.add_bool("last_diag_gnss_ns_available", rt->last_diag.gnss_ns_at_event != 0);
    out.add_bool("last_diag_gnss_projection_valid",
                 rt->last_diag.gnss_ns_at_event != 0 &&
                 rt->last_diag.anchor_failure_mask == 0);
    out.add_u32("last_diag_anchor_selection_kind", rt->last_diag.anchor_selection_kind);
    out.add_u32("last_diag_anchor_failure_mask", rt->last_diag.anchor_failure_mask);
  } else {
    out.add_u32("last_event_dwt", 0);
    out.add_u32("last_event_counter32", 0);
    out.add_u64("last_event_gnss_ns_at_event", 0);
    out.add_bool("last_event_gnss_ns_available", false);
    out.add_u32("last_event_status", 0);
    out.add_u64("last_diag_gnss_ns_at_event", 0);
    out.add_bool("last_diag_gnss_ns_available", false);
    out.add_bool("last_diag_gnss_projection_valid", false);
    out.add_u32("last_diag_anchor_selection_kind", 0);
    out.add_u32("last_diag_anchor_failure_mask", 0);
  }
}
static void add_vclock_lane_payload(Payload& p, bool detailed) {
  p.add("lane", "VCLOCK");
  p.add("kind", "VCLOCK");
  p.add("provider", "QTIMER1");
  p.add("hardware_lane", "QTIMER1_CH2_COMP");
  p.add("cadence_source", "QTIMER1_CH2_NATIVE_EPOCH_ONE_SECOND_PLUS_HEARTBEAT_FALLBACK");
  p.add("counter_source", "QTIMER1_CH0_SYNTHETIC_COUNTER32");
  p.add("event_source", "QTIMER1_CH2_INTRINSIC_EPOCH_AND_ONE_SECOND");
  p.add("dwt_authority", "QTIMER1_CH2_TIMEPOP_EVENT_DWT");

  add_runtime_lane_summary(p, "vclock", g_rt_vclock);
  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count", g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_compare_target", (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_tick_mod_1000", g_vclock_lane.tick_mod_1000);
  p.add("vclock_logical_count32", g_vclock_lane.logical_count32_at_last_second);
  p.add("vclock_ch2_one_second_enabled", g_vclock_ch2_one_second_enabled);
  p.add("vclock_ch2_one_second_next_counter32", g_vclock_ch2_one_second_next_counter32);
  p.add("vclock_ch2_one_second_epoch_enable_count", g_vclock_ch2_one_second_epoch_enable_count);
  p.add("vclock_ch2_one_second_epoch_base_counter32", g_vclock_ch2_one_second_epoch_base_counter32);
  p.add("vclock_ch2_one_second_epoch_service_counter32", g_vclock_ch2_one_second_epoch_service_counter32);
  p.add("vclock_ch2_one_second_epoch_target_counter32", g_vclock_ch2_one_second_epoch_target_counter32);
  p.add("vclock_ch2_one_second_epoch_tick_mod_seed", g_vclock_ch2_one_second_epoch_tick_mod_seed);
  p.add("vclock_ch2_one_second_epoch_remaining_cells", g_vclock_ch2_one_second_epoch_remaining_cells);
  p.add("vclock_ch2_one_second_epoch_residual_ticks", g_vclock_ch2_one_second_epoch_residual_ticks);
  p.add("vclock_ch2_one_second_heartbeat_enable_count", g_vclock_ch2_one_second_heartbeat_enable_count);
  p.add("vclock_ch2_one_second_service_count", g_vclock_ch2_one_second_service_count);
  p.add("vclock_ch2_one_second_enqueue_count", g_vclock_ch2_one_second_enqueue_count);
  p.add("vclock_ch2_one_second_late_count", g_vclock_ch2_one_second_late_count);
  p.add("vclock_ch2_one_second_late_max_ticks", g_vclock_ch2_one_second_late_max_ticks);
  p.add("vclock_heartbeat_one_second_legacy_count", g_vclock_heartbeat_one_second_legacy_count);
  p.add("vclock_heartbeat_one_second_handoff_skip_count", g_vclock_heartbeat_one_second_handoff_skip_count);
  p.add("vclock_heartbeat_one_second_fallback_count", g_vclock_heartbeat_one_second_fallback_count);
  p.add("vclock_heartbeat_smartzero_authority_retired", true);
  p.add("vclock_heartbeat_smartzero_authority_retired_count", g_vclock_heartbeat_smartzero_authority_retired_count);
  p.add("vclock_heartbeat_fact_drain_authority_retired", true);
  p.add("vclock_heartbeat_fact_drain_authority_retired_count", g_vclock_heartbeat_fact_drain_authority_retired_count);
  p.add("vclock_heartbeat_epoch_authority_retired", true);
  p.add("vclock_heartbeat_epoch_authority_retired_count", g_vclock_heartbeat_epoch_authority_retired_count);
  p.add("vclock_heartbeat_epoch_pending_skip_count", g_vclock_heartbeat_epoch_pending_skip_count);
  p.add("vclock_ch2_epoch_native_enabled", VCLOCK_CH2_EPOCH_NATIVE_ENABLED);
  p.add("vclock_ch2_epoch_native_service_count", g_vclock_ch2_epoch_native_service_count);
  p.add("vclock_ch2_epoch_native_publish_count", g_vclock_ch2_epoch_native_publish_count);
  p.add("vclock_ch2_epoch_native_bad_backdate_count", g_vclock_ch2_epoch_native_bad_backdate_count);
  p.add("vclock_ch2_epoch_native_last_reject_reason", g_vclock_ch2_epoch_native_last_reject_reason);
  p.add("vclock_ch2_epoch_native_last_reject_reason_name", vclock_ch2_epoch_reject_reason_name(g_vclock_ch2_epoch_native_last_reject_reason));
  p.add("vclock_ch2_epoch_native_dispatch_arm_count", g_vclock_ch2_epoch_native_dispatch_arm_count);
  p.add("vclock_ch2_epoch_native_dispatch_arm_fail_count", g_vclock_ch2_epoch_native_dispatch_arm_fail_count);
  p.add("vclock_ch2_epoch_native_last_sequence", g_vclock_ch2_epoch_native_last_sequence);
  p.add("vclock_ch2_epoch_native_last_selected_counter32", g_vclock_ch2_epoch_native_last_selected_counter32);
  p.add("vclock_ch2_epoch_native_last_service_counter32", g_vclock_ch2_epoch_native_last_service_counter32);
  p.add("vclock_ch2_epoch_native_last_backdate_ticks", g_vclock_ch2_epoch_native_last_backdate_ticks);
  p.add("vclock_ch2_epoch_native_last_backdate_cycles", g_vclock_ch2_epoch_native_last_backdate_cycles);
  p.add("vclock_ch2_epoch_native_last_sacred_dwt", g_vclock_ch2_epoch_native_last_sacred_dwt);
  p.add("vclock_ch2_fact_drain_native_enabled", true);
  p.add("vclock_ch2_fact_drain_service_count", g_vclock_ch2_fact_drain_service_count);
  p.add("vclock_ch2_fact_drain_arm_count", g_vclock_ch2_fact_drain_arm_count);
  p.add("vclock_ch2_smartzero_native_enabled", VCLOCK_CH2_SMARTZERO_NATIVE_ENABLED);
  p.add("vclock_ch2_smartzero_seeded", g_vclock_ch2_smartzero_seeded);
  p.add("vclock_ch2_smartzero_next_counter32", g_vclock_ch2_smartzero_next_counter32);
  p.add("vclock_ch2_smartzero_service_count", g_vclock_ch2_smartzero_service_count);
  p.add("vclock_ch2_smartzero_transaction_skip_count", g_vclock_ch2_smartzero_transaction_skip_count);
  p.add("vclock_ch2_smartzero_feed_count", g_vclock_ch2_smartzero_feed_count);
  p.add("vclock_ch2_smartzero_late_count", g_vclock_ch2_smartzero_late_count);
  p.add("vclock_ch2_smartzero_late_max_ticks", g_vclock_ch2_smartzero_late_max_ticks);
  p.add("vclock_ch2_smartzero_resync_count", g_vclock_ch2_smartzero_resync_count);
  p.add("vclock_ch2_smartzero_drop_count", g_vclock_ch2_smartzero_drop_count);
  p.add("vclock_ch2_smartzero_accepted_count", g_vclock_ch2_smartzero_accepted_count);
  p.add("vclock_ch2_smartzero_rejected_dwt_count", g_vclock_ch2_smartzero_rejected_dwt_count);
  p.add("vclock_ch2_smartzero_rejected_counter_count", g_vclock_ch2_smartzero_rejected_counter_count);
  add_isr_sanity_payload(p, "isr_sanity", g_isr_sanity_vclock_ch2);

  if (!detailed) return;

  p.add("vclock_dwt_repair_enabled", false);
  p.add("vclock_dwt_repair_threshold_cycles", VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES);
  p.add("vclock_dwt_repair_min_history_count", VCLOCK_DWT_REPAIR_MIN_HISTORY_COUNT);
  p.add("vclock_dwt_repair_max_prediction_residual_cycles", VCLOCK_DWT_REPAIR_MAX_PREDICTION_RESIDUAL_CYCLES);
  p.add("vclock_dwt_repair_candidate_count", g_vclock_repair_stats.candidate_count);
  p.add("vclock_dwt_repair_applied_count", g_vclock_repair_stats.applied_count);
  p.add("vclock_dwt_repair_consecutive_candidate_count", g_vclock_repair_stats.consecutive_candidate_count);
  p.add("vclock_dwt_repair_last_candidate", g_vclock_repair_stats.last_candidate);
  p.add("vclock_dwt_repair_last_synthetic", g_vclock_repair_stats.last_synthetic);
  p.add("vclock_dwt_repair_last_original_dwt", g_vclock_repair_stats.last_original_dwt);
  p.add("vclock_dwt_repair_last_predicted_dwt", g_vclock_repair_stats.last_predicted_dwt);
  p.add("vclock_dwt_repair_last_used_dwt", g_vclock_repair_stats.last_used_dwt);
  p.add("vclock_dwt_repair_last_error_cycles", g_vclock_repair_stats.last_error_cycles);
  p.add("vclock_dwt_repair_max_abs_error_cycles", g_vclock_repair_stats.max_abs_error_cycles);

  add_vclock_clock32_payload(p, "vclock");
  p.add("vclock_perishable_fact_ring_size", VCLOCK_PERISHABLE_FACT_RING_SIZE);
  p.add("vclock_perishable_fact_ring_count", g_vclock_fact_ring.count);
  p.add("vclock_perishable_fact_enqueue_count", g_vclock_fact_ring.enqueue_count);
  p.add("vclock_perishable_fact_drain_count", g_vclock_fact_ring.drain_count);
  p.add("vclock_perishable_fact_overflow_count", g_vclock_fact_ring.overflow_count);
  p.add("vclock_perishable_fact_high_water", g_vclock_fact_ring.high_water);
  p.add("vclock_perishable_fact_asap_arm_count", g_vclock_fact_ring.asap_arm_count);
  p.add("vclock_perishable_fact_asap_fail_count", g_vclock_fact_ring.asap_fail_count);
  p.add("vclock_perishable_fact_drain_armed", g_vclock_fact_ring.drain_armed);
  p.add("vclock_ch2_smartzero_reset_count", g_vclock_ch2_smartzero_reset_count);
  p.add("vclock_ch2_smartzero_seed_count", g_vclock_ch2_smartzero_seed_count);
  p.add("vclock_ch2_smartzero_deactivate_count", g_vclock_ch2_smartzero_deactivate_count);
  p.add("vclock_ch2_smartzero_waiting_for_cps_count", g_vclock_ch2_smartzero_waiting_for_cps_count);
  p.add("vclock_ch2_smartzero_first_sample_count", g_vclock_ch2_smartzero_first_sample_count);
  p.add("vclock_ch2_smartzero_last_target_counter32", g_vclock_ch2_smartzero_last_target_counter32);
  p.add("vclock_ch2_smartzero_last_service_counter32", g_vclock_ch2_smartzero_last_service_counter32);
  p.add("vclock_ch2_smartzero_last_dwt", g_vclock_ch2_smartzero_last_dwt);
  p.add("vclock_ch2_smartzero_last_late_ticks", g_vclock_ch2_smartzero_last_late_ticks);
  add_regression_payload(p, "vclock", g_regression_vclock);
  p.add("qtimer1_ch1_active", g_qtimer1_ch1_active);
  p.add("qtimer1_ch1_sequence", g_qtimer1_ch1_sequence);
  p.add("qtimer1_ch1_target_counter32", g_qtimer1_ch1_target_counter32);
  p.add("qtimer1_ch1_next_compare_counter32", g_qtimer1_ch1_next_compare_counter32);
  p.add("qtimer1_ch1_arm_count", g_qtimer1_ch1_arm_count);
  p.add("qtimer1_ch1_fire_count", g_qtimer1_ch1_fire_count);
  p.add("qtimer1_ch1_hop_count", g_qtimer1_ch1_hop_count);
  p.add("qtimer1_ch2_last_target_counter32", g_qtimer1_ch2_last_target_counter32);
  p.add("qtimer1_ch2_arm_count", g_qtimer1_ch2_arm_count);
}

static void add_ocxo_identity_payload(Payload& p,
                                      const ocxo_runtime_context_t& ctx) {
  p.add("lane", ctx.name);
  p.add("kind", ctx.name);
  p.add("provider", interrupt_provider_kind_str(ctx.provider));
  p.add("hardware_lane", interrupt_lane_str(ctx.lane_id));
  p.add("cadence_source", ctx.cadence_source);
  p.add("counter_source", ctx.counter_source);
  p.add("event_source", "LOCAL_CADENCE_1000TH_ROLLOVER_EMA_EDGE");
  p.add("dwt_authority", ctx.dwt_authority);
}

static void add_ocxo_lane_basic_payload(payload_prefix_t& out,
                                        const ocxo_lane_t& lane) {
  out.add_bool("initialized", lane.initialized);
  out.add_bool("active", lane.active);
  out.add_u32("irq_count", lane.irq_count);
  out.add_u32("miss_count", lane.miss_count);
  out.add_u32("bootstrap_count", lane.bootstrap_count);
  out.add_u32("cadence_hits_total", lane.cadence_hits_total);
  out.add_bool("cadence_enabled", lane.cadence_enabled);
  out.add_bool("cadence_armed", lane.cadence_armed);
  out.add_bool("cadence_epoch_valid", lane.cadence_epoch_valid);
  out.add_u32("cadence_epoch_counter32", lane.cadence_epoch_counter32);
  out.add_u32("cadence_interval_ticks", OCXO_CADENCE_INTERVAL_TICKS);
  out.add_bool("cadence_sample_phase_valid", lane.cadence_sample_phase_valid);
  out.add_u32("cadence_sample_phase_ticks", lane.cadence_sample_phase_ticks);
  out.add_u32("cadence_sample_phase_us", lane.cadence_sample_phase_us);
  out.add_u32("cadence_sample_phase_ns", lane.cadence_sample_phase_ns);
  out.add_u32("cadence_sample_period_ticks", lane.cadence_sample_period_ticks);
  out.add_u32("cadence_phase_align_start_count", lane.cadence_phase_align_start_count);
  out.add_u32("cadence_last_phase_align_vclock_counter32", lane.cadence_last_phase_align_vclock_counter32);
  out.add_u32("cadence_last_phase_align_vclock_phase_ticks", lane.cadence_last_phase_align_vclock_phase_ticks);
  out.add_u32("cadence_last_phase_align_ticks_until_target", lane.cadence_last_phase_align_ticks_until_target);
  out.add_u32("cadence_last_phase_align_ocxo_counter32", lane.cadence_last_phase_align_ocxo_counter32);
  out.add_u32("cadence_next_counter32", lane.cadence_next_counter32);
  out.add_u32("cadence_next_low16", (uint32_t)lane.cadence_next_low16);
  out.add_u32("cadence_fire_count", lane.cadence_fire_count);
  out.add_u32("cadence_arm_count", lane.cadence_arm_count);
  out.add_u32("cadence_rearm_count", lane.cadence_rearm_count);
  out.add_u32("cadence_false_irq_count", lane.cadence_false_irq_count);
  out.add_u32("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  out.add_u32("cadence_user_callback_count", lane.cadence_user_callback_count);
  out.add_u32("cadence_last_reason", lane.cadence_last_reason);
  out.add_str("cadence_last_reason_name",
              ocxo_cadence_reason_name(lane.cadence_last_reason));
  out.add_u32("compare_target", (uint32_t)lane.compare_target);
  out.add_u32("tick_mod_1000", lane.tick_mod_1000);
  out.add_u32("logical_count32", lane.logical_count32_at_last_second);

  out.add_bool("witness_target_initialized", lane.witness_target_initialized);
  out.add_bool("witness_armed", lane.witness_armed);
  out.add_u32("witness_arm_count", lane.witness_arm_count);
  out.add_u32("witness_fire_count", lane.witness_fire_count);
  out.add_u32("witness_false_irq_count", lane.witness_false_irq_count);
  out.add_u32("witness_missed_target_count", lane.witness_missed_target_count);
  out.add_u32("witness_late_arm_count", lane.witness_late_arm_count);
  out.add_u32("witness_target_counter32", lane.witness_target_counter32);
  out.add_u32("witness_last_event_dwt", lane.witness_last_event_dwt);
  out.add_u32("witness_last_event_counter32", lane.witness_last_event_counter32);
  out.add_u32("witness_last_late_ticks", lane.witness_last_late_ticks);
  out.add_u32("witness_last_arm_remaining_ticks", lane.witness_last_arm_remaining_ticks);
}

static void add_ocxo_witness_service_payload(payload_prefix_t& out,
                                             const ocxo_lane_t& lane) {
  out.add_u32("witness_last_target_delta_mod65536_ticks",
              lane.witness_last_target_delta_mod65536_ticks);
  out.add_i32("witness_last_service_offset_signed_ticks",
              lane.witness_last_service_offset_signed_ticks);
  out.add_u32("witness_last_early_ticks", lane.witness_last_early_ticks);
  out.add_u32("witness_last_interpreted_late_ticks",
              lane.witness_last_interpreted_late_ticks);
  out.add_u32("witness_last_service_offset_abs_ticks",
              lane.witness_last_service_offset_abs_ticks);
  out.add_bool("witness_last_service_was_early",
               lane.witness_last_service_was_early);
  out.add_bool("witness_last_service_was_on_or_after_target",
               lane.witness_last_service_was_on_or_after_target);
  out.add_bool("witness_last_event_published", lane.witness_last_event_published);
  out.add_u32("witness_last_service_class", lane.witness_last_service_class);
  out.add_str("witness_last_service_class_name",
              ocxo_service_class_name(lane.witness_last_service_class));
  out.add_u32("witness_service_count", lane.witness_service_count);
  out.add_u32("witness_early_service_count", lane.witness_early_service_count);
  out.add_u32("witness_on_or_after_service_count",
              lane.witness_on_or_after_service_count);
  out.add_u32("witness_valid_publish_count", lane.witness_valid_publish_count);
  out.add_u32("witness_early_service_published_count",
              lane.witness_early_service_published_count);
  out.add_u32("witness_last_fact_sequence", lane.witness_last_fact_sequence);
  out.add_bool("witness_last_fact_one_second_due",
               lane.witness_last_fact_one_second_due);
  out.add_i32("witness_last_service_correction_cycles",
              lane.witness_last_service_correction_cycles);
  out.add_u32("witness_last_service_corrected_dwt",
              lane.witness_last_service_corrected_dwt);
  out.add_u32("witness_last_counter_delta_ticks",
              lane.witness_last_counter_delta_ticks);
  out.add_u32("witness_counter_delta_violation_count",
              lane.witness_counter_delta_violation_count);
  out.add_u32("witness_last_bad_counter_delta",
              lane.witness_last_bad_counter_delta);
  out.add_u32("witness_fact_enqueue_count", lane.witness_fact_enqueue_count);
  out.add_u32("witness_fact_drain_count", lane.witness_fact_drain_count);
  out.add_u32("witness_fact_overflow_count", lane.witness_fact_overflow_count);
  out.add_u32("witness_fact_high_water", lane.witness_fact_high_water);
  out.add_u32("witness_fact_asap_arm_count", lane.witness_fact_asap_arm_count);
  out.add_u32("witness_fact_asap_fail_count", lane.witness_fact_asap_fail_count);
  out.add_bool("ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  out.add_u32("ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  out.add_u32("ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  out.add_bool("ema_initialized", lane.ema_initialized);
  out.add_bool("ema_interval_valid", lane.ema_interval_valid);
  out.add_u32("ema_update_count", lane.ema_update_count);
  out.add_u32("ema_last_observed_dwt", lane.ema_last_observed_dwt);
  out.add_u32("ema_last_emitted_dwt", lane.ema_last_emitted_dwt);
  out.add_u32("ema_last_observed_interval_cycles",
              lane.ema_last_observed_interval_cycles);
  out.add_u32("ema_interval_cycles", lane.ema_interval_cycles);
  out.add_u32("ema_last_predicted_dwt", lane.ema_last_predicted_dwt);
  out.add_i32("ema_last_error_cycles", lane.ema_last_error_cycles);
  out.add_u32("ema_max_abs_error_cycles", lane.ema_max_abs_error_cycles);
  out.add_u32("witness_schedule_last_decision",
              lane.witness_schedule_last_decision);
  out.add_str("witness_schedule_last_decision_name",
              ocxo_schedule_decision_name(lane.witness_schedule_last_decision));
}

static void add_ocxo_cadence_sample_payload(payload_prefix_t& out,
                                            const ocxo_lane_t& lane) {
  out.add_u32("cadence_last_target_counter32", lane.cadence_last_target_counter32);
  out.add_u32("cadence_last_target_low16", (uint32_t)lane.cadence_last_target_low16);
  out.add_u32("cadence_last_service_low16", (uint32_t)lane.cadence_last_service_low16);
  out.add_u32("cadence_last_fire_dwt", lane.cadence_last_fire_dwt);
  out.add_u32("cadence_last_isr_entry_dwt_raw", lane.cadence_last_isr_entry_dwt_raw);
  out.add_i32("cadence_last_service_offset_signed_ticks",
              lane.cadence_last_service_offset_signed_ticks);
  out.add_u32("cadence_last_service_offset_abs_ticks",
              lane.cadence_last_service_offset_abs_ticks);
  out.add_u32("cadence_last_interpreted_late_ticks",
              lane.cadence_last_interpreted_late_ticks);
  out.add_u32("cadence_last_early_ticks", lane.cadence_last_early_ticks);
  out.add_bool("cadence_last_was_early", lane.cadence_last_was_early);
  out.add_bool("cadence_last_one_second_due", lane.cadence_last_one_second_due);
  out.add_bool("ema_initialized", lane.ema_initialized);
  out.add_u32("ema_update_count", lane.ema_update_count);
  out.add_u32("ema_last_observed_interval_cycles",
              lane.ema_last_observed_interval_cycles);
  out.add_u32("ema_interval_cycles", lane.ema_interval_cycles);
  out.add_u32("ema_last_predicted_dwt", lane.ema_last_predicted_dwt);
  out.add_i32("ema_last_error_cycles", lane.ema_last_error_cycles);
}

static void add_ocxo_witness_detail_payload(payload_prefix_t& out,
                                            const ocxo_lane_t& lane) {
  out.add_u32("witness_schedule_last_current_counter32",
              lane.witness_schedule_last_current_counter32);
  out.add_u32("witness_schedule_last_target_counter32",
              lane.witness_schedule_last_target_counter32);
  out.add_u32("witness_schedule_last_remaining_ticks",
              lane.witness_schedule_last_remaining_ticks);
  out.add_u32("witness_schedule_last_phase_ticks",
              lane.witness_schedule_last_phase_ticks);
  out.add_u32("witness_schedule_last_ticks_until_arm_window",
              lane.witness_schedule_last_ticks_until_arm_window);
  out.add_u32("witness_schedule_last_current_low16",
              (uint32_t)lane.witness_schedule_last_current_low16);
  out.add_u32("witness_schedule_last_target_low16",
              (uint32_t)lane.witness_schedule_last_target_low16);

  out.add_u32("witness_last_arm_dwt_raw", lane.witness_last_arm_dwt_raw);
  out.add_u32("witness_last_arm_counter32", lane.witness_last_arm_counter32);
  out.add_u32("witness_last_arm_low16", (uint32_t)lane.witness_last_arm_low16);
  out.add_u32("witness_last_arm_target_counter32",
              lane.witness_last_arm_target_counter32);
  out.add_u32("witness_last_arm_target_low16",
              (uint32_t)lane.witness_last_arm_target_low16);
  out.add_u32("witness_last_arm_to_isr_ticks",
              lane.witness_last_arm_to_isr_ticks);
  out.add_u32("witness_last_arm_to_isr_dwt_cycles",
              lane.witness_last_arm_to_isr_dwt_cycles);
  out.add_i32("witness_last_arm_remaining_minus_early_ticks",
              (int32_t)lane.witness_last_arm_remaining_ticks -
              (int32_t)lane.witness_last_early_ticks);

  out.add_u32("witness_last_program_csctrl_before",
              lane.witness_last_program_csctrl_before);
  out.add_u32("witness_last_program_csctrl_after",
              lane.witness_last_program_csctrl_after);
  out.add_bool("witness_last_program_flag_before",
               lane.witness_last_program_flag_before);
  out.add_bool("witness_last_program_flag_after",
               lane.witness_last_program_flag_after);
  out.add_bool("witness_last_program_enabled_before",
               lane.witness_last_program_enabled_before);
  out.add_bool("witness_last_program_enabled_after",
               lane.witness_last_program_enabled_after);

  out.add_u32("cadence_last_program_csctrl_before",
              lane.cadence_last_program_csctrl_before);
  out.add_u32("cadence_last_program_csctrl_after",
              lane.cadence_last_program_csctrl_after);
  out.add_bool("cadence_last_program_flag_before",
               lane.cadence_last_program_flag_before);
  out.add_bool("cadence_last_program_flag_after",
               lane.cadence_last_program_flag_after);
  out.add_bool("cadence_last_program_enabled_before",
               lane.cadence_last_program_enabled_before);
  out.add_bool("cadence_last_program_enabled_after",
               lane.cadence_last_program_enabled_after);

  out.add_u32("witness_last_isr_csctrl_entry",
              lane.witness_last_isr_csctrl_entry);
  out.add_u32("witness_last_isr_csctrl_after_disable",
              lane.witness_last_isr_csctrl_after_disable);
  out.add_bool("witness_last_isr_compare_flag_entry",
               lane.witness_last_isr_compare_flag_entry);
  out.add_bool("witness_last_isr_compare_enabled_entry",
               lane.witness_last_isr_compare_enabled_entry);
  out.add_bool("witness_last_isr_compare_flag_after_disable",
               lane.witness_last_isr_compare_flag_after_disable);
  out.add_bool("witness_last_isr_compare_enabled_after_disable",
               lane.witness_last_isr_compare_enabled_after_disable);
  out.add_bool("witness_last_irq_had_armed", lane.witness_last_irq_had_armed);
  out.add_bool("witness_last_irq_had_active_rt",
               lane.witness_last_irq_had_active_rt);

  out.add_u32("witness_last_target_low16",
              (uint32_t)lane.witness_last_target_low16);
  out.add_u32("witness_last_isr_counter_low16",
              (uint32_t)lane.witness_last_isr_counter_low16);
  out.add_bool("witness_last_service_early_immediate_after_arm",
               lane.witness_last_service_was_early &&
               lane.witness_last_arm_to_isr_ticks <= 256U);
}

static void add_ocxo_qtimer_payload(payload_prefix_t& out,
                                    const ocxo_runtime_context_t& ctx,
                                    const ocxo_qtimer_diag_t& qdiag) {
  const ocxo_lane_t& lane = *ctx.lane;
  const uint16_t csctrl = lane.module->CH[lane.channel].CSCTRL;
  const bool compare_enabled = (csctrl & TMR_CSCTRL_TCF1EN) != 0;
  const bool compare_flag = (csctrl & TMR_CSCTRL_TCF1) != 0;

  out.add_u32("qtimer_count", qdiag.count);
  out.add_u32("qtimer_counter", (uint32_t)lane.module->CH[lane.channel].CNTR);
  out.add_u32("qtimer_comp1", (uint32_t)lane.module->CH[lane.channel].COMP1);
  out.add_u32("qtimer_csctrl", (uint32_t)csctrl);
  out.add_bool("qtimer_compare_enabled", compare_enabled);
  out.add_bool("qtimer_compare_flag", compare_flag);
  out.add_bool("qtimer_compare_live", compare_enabled && compare_flag);
  out.add_u32("qtimer_last_late_ticks", qdiag.late_ticks);
  out.add_u32("qtimer_last_interpreted_late_ticks", qdiag.interpreted_late_ticks);
  out.add_u32("qtimer_last_early_ticks", qdiag.early_ticks);
  out.add_i32("qtimer_last_service_offset_signed_ticks",
              qdiag.service_offset_signed_ticks);
  out.add_u32("qtimer_last_service_offset_abs_ticks",
              qdiag.service_offset_abs_ticks);
  out.add_u32("qtimer_last_target_delta_mod65536_ticks",
              qdiag.target_delta_mod65536_ticks);
  out.add_u32("qtimer_last_target_low16", (uint32_t)qdiag.target_low16);
  out.add_u32("qtimer_last_isr_counter_low16",
              (uint32_t)qdiag.isr_counter_low16);
  out.add_u32("qtimer_last_dwt_raw", qdiag.dwt_raw);
  out.add_u32("qtimer_last_event_dwt", qdiag.event_dwt);
  out.add_u32("qtimer_last_dwt_coordinate_source",
              qdiag.dwt_coordinate_source);
}

static void add_ocxo_perishable_ring_payload(payload_prefix_t& out,
                                             const ocxo_runtime_context_t& ctx) {
  const ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx);

  out.add_u32("perishable_fact_ring_size", OCXO_PERISHABLE_FACT_RING_SIZE);
  out.add_u32("perishable_fact_ring_count", ring.count);
  out.add_u32("perishable_fact_enqueue_count", ring.enqueue_count);
  out.add_u32("perishable_fact_drain_count", ring.drain_count);
  out.add_u32("perishable_fact_overflow_count", ring.overflow_count);
  out.add_u32("perishable_fact_high_water", ring.high_water);
  out.add_u32("perishable_fact_asap_arm_count", ring.asap_arm_count);
  out.add_u32("perishable_fact_asap_fail_count", ring.asap_fail_count);
  out.add_bool("perishable_fact_drain_armed", ring.drain_armed);
}

static void add_ocxo_lane_payload(Payload& p,
                                  const ocxo_runtime_context_t& ctx,
                                  bool detailed) {
  const ocxo_lane_t& lane = *ctx.lane;
  const synthetic_clock32_t& clock32 = *ctx.clock32;
  const interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  const ocxo_qtimer_diag_t& qdiag = *ctx.qtimer_diag;
  payload_prefix_t out(p, ctx.prefix);

  add_ocxo_identity_payload(p, ctx);
  add_runtime_lane_summary(p, ctx.prefix, rt);
  add_ocxo_lane_basic_payload(out, lane);
  add_ocxo_witness_service_payload(out, lane);
  add_ocxo_cadence_sample_payload(out, lane);
  const isr_sanity_diag_t* sanity = isr_sanity_for_kind(ctx.kind);
  if (sanity) {
    add_isr_sanity_payload(p, "isr_sanity", *sanity);
  }

  if (!detailed) return;

  add_ocxo_witness_detail_payload(out, lane);
  add_ocxo_clock32_payload(p, ctx.prefix, clock32);
  add_ocxo_qtimer_payload(out, ctx, qdiag);
  add_ocxo_perishable_ring_payload(out, ctx);
  const cadence_regression_lane_t* regression = cadence_regression_for_const(ctx.kind);
  if (regression) add_regression_payload(p, ctx.prefix, *regression);
}
static void add_ocxo_compact_payload(Payload& p,
                                     const ocxo_runtime_context_t& ctx) {
  const interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  const ocxo_lane_t& lane = *ctx.lane;
  const ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx);
  payload_prefix_t out(p, ctx.prefix);

  out.add_u32("event_count", rt ? rt->event_count : 0U);
  out.add_u32("dispatch_count", rt ? rt->dispatch_count : 0U);
  out.add_u64("last_event_gnss_ns_at_event",
              (rt && rt->has_fired) ? rt->last_event.gnss_ns_at_event : 0ULL);
  out.add_bool("last_event_gnss_ns_available",
               rt && rt->has_fired && rt->last_event.gnss_ns_at_event != 0);
  out.add_u64("last_diag_gnss_ns_at_event",
              (rt && rt->has_fired) ? rt->last_diag.gnss_ns_at_event : 0ULL);
  out.add_bool("last_diag_gnss_projection_valid",
               rt && rt->has_fired &&
               rt->last_diag.gnss_ns_at_event != 0 &&
               rt->last_diag.anchor_failure_mask == 0);
  out.add_u32("irq_count", lane.irq_count);
  out.add_u32("miss_count", lane.miss_count);
  out.add_bool("cadence_enabled", lane.cadence_enabled);
  out.add_bool("cadence_armed", lane.cadence_armed);
  out.add_u32("cadence_fire_count", lane.cadence_fire_count);
  out.add_u32("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  out.add_u32("cadence_false_irq_count", lane.cadence_false_irq_count);
  out.add_u32("cadence_next_counter32", lane.cadence_next_counter32);
  out.add_bool("cadence_sample_phase_valid", lane.cadence_sample_phase_valid);
  out.add_u32("cadence_sample_phase_ticks", lane.cadence_sample_phase_ticks);
  out.add_u32("cadence_sample_phase_us", lane.cadence_sample_phase_us);
  out.add_i32("cadence_service_offset_ticks",
              lane.cadence_last_service_offset_signed_ticks);
  out.add_i32("service_offset_ticks", lane.witness_last_service_offset_signed_ticks);
  out.add_u32("counter_delta_violation_count", lane.witness_counter_delta_violation_count);
  out.add_u32("missed_target_count", lane.witness_missed_target_count);
  out.add_u32("false_irq_count", lane.witness_false_irq_count);
  out.add_u32("late_arm_count", lane.witness_late_arm_count);
  out.add_u32("fact_enqueue_count", ring.enqueue_count);
  out.add_u32("fact_drain_count", ring.drain_count);
  out.add_u32("fact_overflow_count", ring.overflow_count);
  out.add_u32("fact_high_water", ring.high_water);
  out.add_u32("fact_asap_fail_count", ring.asap_fail_count);
  out.add_bool("ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  out.add_bool("ema_initialized", lane.ema_initialized);
  out.add_u32("ema_update_count", lane.ema_update_count);
  out.add_u32("ema_interval_cycles", lane.ema_interval_cycles);
  out.add_i32("ema_last_error_cycles", lane.ema_last_error_cycles);
  out.add_bool("linear_regression_enabled", OCXO_LINEAR_REGRESSION_ENABLED);

  const cadence_regression_lane_t* regression = cadence_regression_for_const(ctx.kind);
  if (regression) {
    out.add_bool("regression_last_valid", regression->last_result.valid);
    out.add_u32("regression_last_sample_count", regression->last_result.sample_count);
    out.add_i32("regression_last_inferred_minus_observed_cycles",
                regression->last_result.inferred_minus_observed_cycles);
    out.add_u32("regression_last_fit_error_stddev_q16_cycles",
                regression->last_result.fit_error_stddev_q16_cycles);
    out.add_u32("regression_last_fit_error_abs_gt4_count",
                regression->last_result.fit_error_abs_gt4_count);
  }
}

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_COMPACT");
  p.add("subreports",
        "REPORT_STATUS REPORT_PPS REPORT_CADENCE REPORT_SMARTZERO "
        "REPORT_BRIDGE REPORT_LANES REPORT_LANE");
  p.add("regression_samples_report_command", "disabled");

  // Keep the default INTERRUPT.REPORT deliberately small.  The former
  // monolithic report carried PPS, epoch capture, SmartZero lane proof,
  // bridge, cadence, and lane forensics all at once; after the perishable
  // fact pass it can exceed the transport/payload budget.  This compact
  // report is a health dashboard only.  Heavy surfaces are explicit opt-in
  // subreports below.
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("ocxo_disable_experiment", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_disabled_count", OCXO_DISABLED_COUNT);
  p.add("ocxo1_disabled", OCXO1_DISABLED);
  p.add("ocxo2_disabled", OCXO2_DISABLED);
  p.add("ocxo1_irq_qtimer2_disabled", OCXO1_DISABLED);
  p.add("ocxo2_irq_qtimer3_disabled", OCXO2_DISABLED);
  p.add("ocxo_smartzero_surrogates_enabled", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_quiet_phase_sampling_enabled", OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo1_quiet_phase_ticks", OCXO1_QUIET_PHASE_TICKS);
  p.add("ocxo1_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO1_QUIET_PHASE_TICKS));
  p.add("ocxo2_quiet_phase_ticks", OCXO2_QUIET_PHASE_TICKS);
  p.add("ocxo2_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO2_QUIET_PHASE_TICKS));
  p.add("timing_arch_step", "CH2_NATIVE_VCLOCK_FIRST_1S_LEGACY_GRID");
  p.add("timing_arch_behavior_changed", true);
  p.add("isr_sanity_witness_enabled", true);
  p.add("isr_sanity_witness_policy", "REPORT_ONLY_NO_REPAIR_NO_VETO");
  p.add("linear_regression_authored_dwt", LINEAR_REGRESSION_AUTHORED_DWT);
  p.add("linear_regression_diagnostic_only", LINEAR_REGRESSION_DIAGNOSTIC_ONLY);
  p.add("vclock_linear_regression_enabled", VCLOCK_LINEAR_REGRESSION_ENABLED);
  p.add("ocxo_linear_regression_enabled", OCXO_LINEAR_REGRESSION_ENABLED);
  p.add("single_cadence_agent", false);
  p.add("vclock_heartbeat_isr_mode", true);
  p.add("vclock_heartbeat_retained", true);
  p.add("ocxo_lane_local_cadence", false);
  p.add("ocxo_one_second_edge_compare", true);
  p.add("ocxo_native_1khz_cadence_retired", true);
  p.add("ocxo_perishable_fact_capture", true);
  p.add("ocxo_fact_ring_size", OCXO_PERISHABLE_FACT_RING_SIZE);

  p.add("qtimer1_priority_ok",
        g_step0_qtimer1_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);
  p.add("qtimer2_priority_ok",
        OCXO1_DISABLED ||
        g_step0_qtimer2_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);
  p.add("qtimer3_priority_ok",
        OCXO2_DISABLED ||
        g_step0_qtimer3_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);
  p.add("gpio6789_priority_ok",
        g_step0_gpio6789_priority_applied == INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);

  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("pps_post_isr_pending", g_pps_post_isr.pending);
  p.add("pps_post_isr_overwrite_count", g_pps_post_isr.overwrite_count);
  p.add("pps_post_isr_asap_fail_count", g_pps_post_isr.asap_fail_count);
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);

  p.add("vclock_heartbeat_armed", g_vclock_heartbeat_armed);
  p.add("vclock_heartbeat_fire_count", g_vclock_heartbeat_fire_count);
  p.add("vclock_heartbeat_arm_failures", g_vclock_heartbeat_arm_failures);
  p.add("vclock_heartbeat_vclock_ticks", g_vclock_heartbeat_vclock_ticks);
  p.add("vclock_heartbeat_ocxo1_rollover_updates_retired", g_vclock_heartbeat_ocxo1_rollover_updates_retired);
  p.add("vclock_heartbeat_ocxo2_rollover_updates_retired", g_vclock_heartbeat_ocxo2_rollover_updates_retired);
  p.add("ch2_implicit_rollover_enabled", CH2_IMPLICIT_ROLLOVER_ENABLED);
  p.add("ch2_implicit_rollover_count", g_ch2_implicit_rollover_count);
  p.add("ch2_implicit_rollover_vclock_updates", g_ch2_implicit_rollover_vclock_updates);
  p.add("ch2_implicit_rollover_ocxo1_updates", g_ch2_implicit_rollover_ocxo1_updates);
  p.add("ch2_implicit_rollover_ocxo2_updates", g_ch2_implicit_rollover_ocxo2_updates);

  interrupt_smartzero_snapshot_t z{};
  (void)interrupt_smartzero_live_snapshot(&z);
  p.add("live_smartzero_phase", smartzero_phase_name(z.phase));
  p.add("live_smartzero_running", z.running);
  p.add("live_smartzero_complete", z.complete);
  p.add("live_smartzero_aborted", z.aborted);
  p.add("live_smartzero_sequence", z.sequence);
  p.add("live_smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));

  p.add("vclock_event_count", g_rt_vclock ? g_rt_vclock->event_count : 0U);
  p.add("vclock_dispatch_count", g_rt_vclock ? g_rt_vclock->dispatch_count : 0U);
  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_linear_regression_enabled", VCLOCK_LINEAR_REGRESSION_ENABLED);
  p.add("vclock_fact_ring_size", VCLOCK_PERISHABLE_FACT_RING_SIZE);
  p.add("vclock_fact_enqueue_count", g_vclock_fact_ring.enqueue_count);
  p.add("vclock_fact_drain_count", g_vclock_fact_ring.drain_count);
  p.add("vclock_fact_overflow_count", g_vclock_fact_ring.overflow_count);
  p.add("vclock_fact_high_water", g_vclock_fact_ring.high_water);
  p.add("vclock_fact_asap_fail_count", g_vclock_fact_ring.asap_fail_count);
  p.add("vclock_regression_last_valid", g_regression_vclock.last_result.valid);
  p.add("vclock_regression_last_sample_count", g_regression_vclock.last_result.sample_count);
  p.add("vclock_regression_last_inferred_minus_observed_cycles",
        g_regression_vclock.last_result.inferred_minus_observed_cycles);
  p.add("vclock_regression_last_fit_error_stddev_q16_cycles",
        g_regression_vclock.last_result.fit_error_stddev_q16_cycles);
  p.add("vclock_regression_last_fit_error_abs_gt4_count",
        g_regression_vclock.last_result.fit_error_abs_gt4_count);

  add_ocxo_compact_payload(p, g_ocxo1_ctx);
  add_ocxo_compact_payload(p, g_ocxo2_ctx);

  return p;
}
static Payload cmd_report_status(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_STATUS");
  add_runtime_payload(p);
  add_priority_payload(p);
  return p;
}

static Payload cmd_report_pps(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_PPS");
  add_pps_payload(p);
  add_epoch_capture_payload(p);
  return p;
}

static Payload cmd_report_cadence(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_CADENCE");
  add_vclock_heartbeat_payload(p);
  add_dynamic_cps_payload(p);
  return p;
}

static Payload cmd_report_smartzero(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_SMARTZERO");
  add_smartzero_payload(p);
  return p;
}

static void add_pvc_anchor_entry_payload(Payload& p,
                                         const char* prefix,
                                         const pvc_anchor_record_t& a,
                                         bool present) {
  payload_prefix_t out(p, prefix);
  const bool label_present = present && a.gnss_ns_at_edge >= 0;
  const bool cps_present = present && a.cps_valid && a.cps != 0;

  out.add_bool("present", present);
  out.add_bool("gnss_label_present", label_present);
  out.add_bool("cps_valid", cps_present);
  out.add_bool("usable", label_present && cps_present);
  out.add_u32("sequence", present ? a.sequence : 0U);
  out.add_u32("dwt_at_edge", present ? a.dwt_at_edge : 0U);
  out.add_u32("counter32_at_edge", present ? a.counter32_at_edge : 0U);
  out.add_i64("gnss_ns_at_edge", present ? a.gnss_ns_at_edge : -1);
  out.add_u32("cps", present ? a.cps : 0U);
}

static void add_bridge_payload(Payload& p) {
  p.add("pvc_anchor_ring_count", g_pvc_anchor_count);
  p.add("pvc_anchor_ring_head", g_pvc_anchor_head);
  p.add("pvc_anchor_ring_seq", g_pvc_anchor_seq);
  p.add("pvc_anchor_reset_pending", g_pvc_anchor_reset_pending);

  pvc_anchor_record_t anchors[PVC_ANCHOR_RING_SIZE];
  uint32_t snapshot_count = 0;
  const bool snapshot_ok = pvc_anchor_snapshot(anchors, snapshot_count);
  uint32_t label_count = 0;
  uint32_t cps_valid_count = 0;
  uint32_t usable_count = 0;

  if (snapshot_ok) {
    for (uint32_t i = 0; i < snapshot_count; i++) {
      const bool has_label = anchors[i].gnss_ns_at_edge >= 0;
      const bool has_cps = anchors[i].cps_valid && anchors[i].cps != 0;
      if (has_label) label_count++;
      if (has_cps) cps_valid_count++;
      if (has_label && has_cps) usable_count++;
    }
  }

  p.add("pvc_anchor_snapshot_ok", snapshot_ok);
  p.add("pvc_anchor_snapshot_count", snapshot_ok ? snapshot_count : 0U);
  p.add("pvc_anchor_label_count", label_count);
  p.add("pvc_anchor_cps_valid_count", cps_valid_count);
  p.add("pvc_anchor_usable_count", usable_count);
  p.add("pvc_anchor_label_update_count", g_pvc_anchor_label_update_count);
  p.add("pvc_anchor_label_miss_count", g_pvc_anchor_label_miss_count);
  p.add("pvc_anchor_label_invalid_count", g_pvc_anchor_label_invalid_count);
  p.add("pvc_anchor_label_last_success", g_pvc_anchor_label_last_success);
  p.add("pvc_anchor_label_last_sequence", g_pvc_anchor_label_last_sequence);
  p.add("pvc_anchor_label_last_counter32", g_pvc_anchor_label_last_counter32);
  p.add("pvc_anchor_label_last_gnss_ns", g_pvc_anchor_label_last_gnss_ns);
  p.add("pvc_anchor_label_last_cps", g_pvc_anchor_label_last_cps);
  p.add("pvc_anchor_label_last_match_age_slots",
        g_pvc_anchor_label_last_match_age_slots);
  p.add("pvc_anchor_label_last_match_index",
        g_pvc_anchor_label_last_match_index);

  add_pvc_anchor_entry_payload(p, "pvc_anchor_latest",
                               snapshot_ok && snapshot_count > 0
                                   ? anchors[0]
                                   : pvc_anchor_record_t{},
                               snapshot_ok && snapshot_count > 0);
  add_pvc_anchor_entry_payload(p, "pvc_anchor_previous",
                               snapshot_ok && snapshot_count > 1
                                   ? anchors[1]
                                   : pvc_anchor_record_t{},
                               snapshot_ok && snapshot_count > 1);
  add_pvc_anchor_entry_payload(p, "pvc_anchor_older2",
                               snapshot_ok && snapshot_count > 2
                                   ? anchors[2]
                                   : pvc_anchor_record_t{},
                               snapshot_ok && snapshot_count > 2);
  add_pvc_anchor_entry_payload(p, "pvc_anchor_older3",
                               snapshot_ok && snapshot_count > 3
                                   ? anchors[3]
                                   : pvc_anchor_record_t{},
                               snapshot_ok && snapshot_count > 3);

  add_bridge_stats_payload(p, "timepop", g_bridge_stats_timepop);
  add_bridge_stats_payload(p, "ocxo1", g_bridge_stats_ocxo1);
  add_bridge_stats_payload(p, "ocxo2", g_bridge_stats_ocxo2);
}

static Payload cmd_report_bridge(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_BRIDGE");
  add_bridge_payload(p);
  return p;
}

static void add_lane_summary_object(Payload& parent,
                                    const char* key,
                                    const char* name,
                                    const interrupt_subscriber_runtime_t* rt,
                                    uint32_t irq_count,
                                    uint32_t miss_count,
                                    uint32_t tick_mod_1000,
                                    uint32_t logical_count32) {
  Payload lane;
  lane.add("lane", name);
  lane.add("subscribed", rt ? rt->subscribed : false);
  lane.add("active", rt ? rt->active : false);
  lane.add("has_fired", rt ? rt->has_fired : false);
  lane.add("event_count", rt ? rt->event_count : 0U);
  lane.add("dispatch_count", rt ? rt->dispatch_count : 0U);
  lane.add("irq_count", irq_count);
  lane.add("miss_count", miss_count);
  lane.add("tick_mod_1000", tick_mod_1000);
  lane.add("logical_count32", logical_count32);
  if (rt && rt->has_fired) {
    lane.add("last_event_dwt", rt->last_event.dwt_at_event);
    lane.add("last_event_counter32", rt->last_event.counter32_at_event);
    lane.add("last_event_gnss_ns_at_event", rt->last_event.gnss_ns_at_event);
    lane.add("last_event_gnss_ns_available", rt->last_event.gnss_ns_at_event != 0);
  } else {
    lane.add("last_event_dwt", 0U);
    lane.add("last_event_counter32", 0U);
    lane.add("last_event_gnss_ns_at_event", 0ULL);
    lane.add("last_event_gnss_ns_available", false);
  }
  // Only VCLOCK uses this generic summary helper today.  Keep the compact
  // sanity surface here so REPORT_LANES can reveal trouble without the full
  // lane detail report.
  lane.add("isr_sanity_cntr_incorrect_count", g_isr_sanity_vclock_ch2.cntr_incorrect_count);
  lane.add("isr_sanity_dwt_incorrect_count", g_isr_sanity_vclock_ch2.dwt_incorrect_count);
  parent.add_object(key, lane);
}

static void add_ocxo_lane_summary_object(Payload& parent,
                                         const char* key,
                                         const ocxo_runtime_context_t& ctx) {
  const interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  const ocxo_lane_t& lane = *ctx.lane;
  const ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx);

  Payload o;
  o.add("lane", ctx.name);
  o.add("subscribed", rt ? rt->subscribed : false);
  o.add("active", rt ? rt->active : false);
  o.add("has_fired", rt ? rt->has_fired : false);
  o.add("event_count", rt ? rt->event_count : 0U);
  o.add("dispatch_count", rt ? rt->dispatch_count : 0U);
  o.add("irq_count", lane.irq_count);
  o.add("miss_count", lane.miss_count);
  o.add("tick_mod_1000", lane.tick_mod_1000);
  o.add("logical_count32", lane.logical_count32_at_last_second);
  o.add("cadence_enabled", lane.cadence_enabled);
  o.add("cadence_armed", lane.cadence_armed);
  o.add("cadence_epoch_valid", lane.cadence_epoch_valid);
  o.add("cadence_epoch_counter32", lane.cadence_epoch_counter32);
  o.add("cadence_next_counter32", lane.cadence_next_counter32);
  o.add("cadence_fire_count", lane.cadence_fire_count);
  o.add("cadence_arm_count", lane.cadence_arm_count);
  o.add("cadence_rearm_count", lane.cadence_rearm_count);
  o.add("cadence_false_irq_count", lane.cadence_false_irq_count);
  o.add("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  o.add("cadence_last_service_offset_ticks",
        lane.cadence_last_service_offset_signed_ticks);
  o.add("cadence_last_reason", lane.cadence_last_reason);
  o.add("cadence_last_reason_name",
        ocxo_cadence_reason_name(lane.cadence_last_reason));
  o.add("witness_arm_count", lane.witness_arm_count);
  o.add("witness_fire_count", lane.witness_fire_count);
  o.add("witness_last_counter_delta_ticks", lane.witness_last_counter_delta_ticks);
  o.add("witness_counter_delta_violation_count", lane.witness_counter_delta_violation_count);
  o.add("witness_missed_target_count", lane.witness_missed_target_count);
  o.add("witness_false_irq_count", lane.witness_false_irq_count);
  o.add("witness_late_arm_count", lane.witness_late_arm_count);
  o.add("witness_service_offset_ticks", lane.witness_last_service_offset_signed_ticks);
  o.add("witness_service_class", lane.witness_last_service_class);
  o.add("witness_service_class_name", ocxo_service_class_name(lane.witness_last_service_class));
  o.add("fact_enqueue_count", ring.enqueue_count);
  o.add("fact_drain_count", ring.drain_count);
  o.add("fact_overflow_count", ring.overflow_count);
  o.add("fact_high_water", ring.high_water);
  o.add("fact_asap_fail_count", ring.asap_fail_count);
  const isr_sanity_diag_t* sanity = isr_sanity_for_kind(ctx.kind);
  if (sanity) {
    o.add("isr_sanity_cntr_incorrect_count", sanity->cntr_incorrect_count);
    o.add("isr_sanity_dwt_incorrect_count", sanity->dwt_incorrect_count);
  }
  if (rt && rt->has_fired) {
    o.add("last_event_dwt", rt->last_event.dwt_at_event);
    o.add("last_event_counter32", rt->last_event.counter32_at_event);
    o.add("last_event_gnss_ns_at_event", rt->last_event.gnss_ns_at_event);
    o.add("last_event_gnss_ns_available", rt->last_event.gnss_ns_at_event != 0);
    o.add("last_diag_gnss_ns_at_event", rt->last_diag.gnss_ns_at_event);
    o.add("last_diag_gnss_projection_valid",
          rt->last_diag.gnss_ns_at_event != 0 &&
          rt->last_diag.anchor_failure_mask == 0);
    o.add("last_diag_anchor_selection_kind", rt->last_diag.anchor_selection_kind);
    o.add("last_diag_anchor_failure_mask", rt->last_diag.anchor_failure_mask);
  } else {
    o.add("last_event_dwt", 0U);
    o.add("last_event_counter32", 0U);
    o.add("last_event_gnss_ns_at_event", 0ULL);
    o.add("last_event_gnss_ns_available", false);
    o.add("last_diag_gnss_ns_at_event", 0ULL);
    o.add("last_diag_gnss_projection_valid", false);
    o.add("last_diag_anchor_selection_kind", 0U);
    o.add("last_diag_anchor_failure_mask", 0U);
  }
  parent.add_object(key, o);
}

static Payload cmd_report_lanes(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_LANES");
  p.add("lane_count", 3);
  p.add("detail_command", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");

  add_lane_summary_object(p,
                          "vclock",
                          "VCLOCK",
                          g_rt_vclock,
                          g_vclock_lane.irq_count,
                          g_vclock_lane.miss_count,
                          g_vclock_lane.tick_mod_1000,
                          g_vclock_lane.logical_count32_at_last_second);

  add_ocxo_lane_summary_object(p,
                               "ocxo1",
                               g_ocxo1_ctx);

  add_ocxo_lane_summary_object(p,
                               "ocxo2",
                               g_ocxo2_ctx);

  return p;
}

static Payload cmd_report_lane(const Payload& args) {
  const char* lane = args.getString("lane");
  if (!lane || !*lane) lane = args.getString("name");

  Payload p;
  p.add("report", "lane_detail");

  if (!lane || !*lane) {
    p.add("error", "missing lane parameter");
    p.add("usage", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
    return p;
  }

  if (!strcasecmp(lane, "VCLOCK") || !strcasecmp(lane, "VCLK")) {
    add_vclock_lane_payload(p, true);
    return p;
  }

  if (!strcasecmp(lane, "OCXO1") || !strcasecmp(lane, "O1")) {
    add_ocxo_lane_payload(p, g_ocxo1_ctx, true);
    return p;
  }

  if (!strcasecmp(lane, "OCXO2") || !strcasecmp(lane, "O2")) {
    add_ocxo_lane_payload(p, g_ocxo2_ctx, true);
    return p;
  }

  p.add("error", "unknown lane");
  p.add("lane", lane);
  p.add("usage", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
  return p;
}

static Payload cmd_regression_samples(const Payload& args) {
  const char* lane = args.getString("lane");
  if (!lane || !*lane) lane = args.getString("name");

  Payload p;
  p.add("report", "INTERRUPT_REGRESSION_SAMPLES");
  p.add("usage", "INTERRUPT.REGRESSION_SAMPLES lane=OCXO1|OCXO2|VCLOCK offset=0 count=8 threshold=4");
  p.add("implementation", "LIVE_ONLY_NO_RETAINED_WINDOWS");
  p.add("memory_policy", "NO_NEW_SAMPLE_BUFFERS");
  p.add("completed_window_available", false);

  const interrupt_subscriber_kind_t kind = regression_kind_from_lane_arg(lane);
  if (kind == interrupt_subscriber_kind_t::NONE) {
    p.add("error", lane && *lane ? "unknown lane" : "missing lane parameter");
    if (lane) p.add("lane", lane);
    return p;
  }

  const cadence_regression_lane_t* r = cadence_regression_for_const(kind);
  if (!r) {
    p.add("error", "no regression lane");
    p.add("lane", interrupt_subscriber_kind_str(kind));
    return p;
  }

  uint32_t offset = args.getUInt("offset", 0U);
  uint32_t count = args.getUInt("count", REGRESSION_SAMPLE_REPORT_SLICE_LIMIT);
  uint32_t threshold = args.getUInt(
      "threshold", (uint32_t)REGRESSION_FIT_ERROR_THRESHOLD_CYCLES);
  if (count == 0 || count > REGRESSION_SAMPLE_REPORT_SLICE_LIMIT) {
    count = REGRESSION_SAMPLE_REPORT_SLICE_LIMIT;
  }

  const cadence_regression_result_t& last = r->last_result;
  const uint32_t n = (r->sample_count > REGRESSION_SAMPLES_PER_SECOND)
      ? REGRESSION_SAMPLES_PER_SECOND
      : r->sample_count;

  double slope = 0.0;
  double intercept = 0.0;
  const bool fit_valid = regression_live_fit_params(*r, slope, intercept);

  int32_t fit_min = 0;
  int32_t fit_max = 0;
  uint32_t abs_le4 = 0;
  uint32_t abs_le8 = 0;
  uint32_t abs_le12 = 0;
  uint32_t abs_le20 = 0;
  uint32_t abs_le50 = 0;
  uint32_t abs_le100 = 0;
  uint32_t abs_gt100 = 0;
  uint32_t gate_accepted = 0;

  if (fit_valid) {
    fit_min = INT32_MAX;
    fit_max = INT32_MIN;
    for (uint32_t i = 0; i < n; i++) {
      const int32_t err = regression_live_fit_error_cycles(*r, i, slope, intercept);
      const uint32_t abs_err = regression_abs_i32(err);
      if (err < fit_min) fit_min = err;
      if (err > fit_max) fit_max = err;
      if (abs_err <= 4U) abs_le4++;
      if (abs_err <= 8U) abs_le8++;
      if (abs_err <= 12U) abs_le12++;
      if (abs_err <= 20U) abs_le20++;
      if (abs_err <= 50U) abs_le50++;
      if (abs_err <= 100U) abs_le100++; else abs_gt100++;
      if (abs_err <= threshold) gate_accepted++;
    }
  }

  const uint32_t last_index = (n > 0) ? (n - 1U) : 0U;
  const int32_t inferred_rel = fit_valid
      ? regression_round_i32(intercept + slope * (double)last_index)
      : 0;
  const uint32_t inferred_dwt = fit_valid
      ? (r->base_observed_dwt + (uint32_t)inferred_rel)
      : 0U;
  const int32_t inferred_minus_observed = (fit_valid && n > 0)
      ? (int32_t)(inferred_dwt - r->last_observed_dwt)
      : 0;

  p.add("lane", interrupt_subscriber_kind_str(kind));
  p.add("regression_samples_per_second", REGRESSION_SAMPLES_PER_SECOND);
  p.add("regression_counter_delta_ticks", REGRESSION_COUNTER_DELTA_TICKS);
  p.add("linear_regression_authored_dwt", LINEAR_REGRESSION_AUTHORED_DWT);
  p.add("linear_regression_diagnostic_only", LINEAR_REGRESSION_DIAGNOSTIC_ONLY);
  p.add("sample_total_count", r->sample_total_count);
  p.add("live_window_sample_count", n);
  p.add("window_sample_count", n);
  p.add("reset_count", r->reset_count);
  p.add("insufficient_count", r->insufficient_count);
  p.add("counter_discontinuity_count", r->counter_discontinuity_count);

  p.add("regression_last_valid", last.valid);
  p.add("regression_last_sequence", last.sequence);
  p.add("regression_last_sample_count", last.sample_count);
  p.add("regression_last_observed_dwt", last.observed_dwt_at_event);
  p.add("regression_last_inferred_dwt", last.inferred_dwt_at_event);
  p.add("regression_last_inferred_minus_observed_cycles",
        last.inferred_minus_observed_cycles);

  p.add("report_source", fit_valid ? "live_window_provisional_fit" : "live_window_raw");
  p.add("selected_window_is_live", true);
  p.add("selected_window_is_completed", false);
  p.add("live_window_fit_valid", fit_valid);
  p.add("regression_window_valid", n > 0);
  p.add("regression_window_sequence", r->sequence);
  p.add("regression_window_sample_count", n);
  p.add("regression_window_base_counter32", r->base_counter32);
  p.add("regression_window_base_observed_dwt", r->base_observed_dwt);
  p.add("regression_window_event_target_counter32", r->last_counter32);
  p.add("regression_window_event_observed_dwt", r->last_observed_dwt);
  p.add("regression_window_event_inferred_dwt", inferred_dwt);
  p.add("regression_window_event_inferred_minus_observed_cycles",
        inferred_minus_observed);

  p.add("fit_error_min_cycles", fit_valid ? fit_min : 0);
  p.add("fit_error_max_cycles", fit_valid ? fit_max : 0);
  p.add("abs_error_le4_count", abs_le4);
  p.add("abs_error_le8_count", abs_le8);
  p.add("abs_error_le12_count", abs_le12);
  p.add("abs_error_le20_count", abs_le20);
  p.add("abs_error_le50_count", abs_le50);
  p.add("abs_error_le100_count", abs_le100);
  p.add("abs_error_gt100_count", abs_gt100);
  p.add("gate_threshold_cycles", threshold);
  p.add("gate_accepted_count", fit_valid ? gate_accepted : 0U);
  p.add("gate_rejected_count", fit_valid ? (n - gate_accepted) : 0U);

  p.add("sample_slice_offset", offset);
  p.add("sample_slice_count_requested", count);
  p.add("sample_slice_limit", REGRESSION_SAMPLE_REPORT_SLICE_LIMIT);
  p.add("sample_encoding", "flat_scalar_fields_no_service_offsets");

  if (n > 0) {
    if (offset >= n) offset = n - 1U;
    if (offset + count > n) count = n - offset;
    p.add("sample_slice_count_returned", count);
    for (uint32_t i = 0; i < count; i++) {
      add_regression_sample_scalar(p, *r, offset + i, i, fit_valid, slope, intercept);
    }
  } else {
    p.add("sample_slice_count_returned", 0U);
  }

  add_regression_extreme_csv(p, *r, true, fit_valid, slope, intercept);
  add_regression_extreme_csv(p, *r, false, fit_valid, slope, intercept);
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
  { "REPORT_STATUS",   cmd_report_status   },
  { "REPORT_PPS",      cmd_report_pps      },
  { "REPORT_CADENCE",  cmd_report_cadence  },
  { "REPORT_SMARTZERO", cmd_report_smartzero },
  { "REPORT_BRIDGE",   cmd_report_bridge   },
  { "REPORT_LANES",    cmd_report_lanes    },
  { "REPORT_LANE",     cmd_report_lane     },
  { nullptr,           nullptr             }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}


// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:  return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:   return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:   return "OCXO2";
    case interrupt_subscriber_kind_t::TIMEPOP: return "TIMEPOP";
    default:                                   return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1:  return "QTIMER1";
    case interrupt_provider_kind_t::QTIMER2:  return "QTIMER2";
    case interrupt_provider_kind_t::QTIMER3:  return "QTIMER3";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH1_COMP: return "QTIMER1_CH1_COMP";
    case interrupt_lane_t::QTIMER1_CH2_COMP: return "QTIMER1_CH2_COMP";
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER2_CH0_COMP: return "QTIMER2_CH0_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}
