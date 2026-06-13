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
//                   is latency-adjusted, the _raw is gone.  Raw ISR-entry DWT
//                   may now appear only on the explicit diagnostic field
//                   dwt_isr_entry_raw; it is evidence, never event authority.
//
// TimePop is the principled exception to the "no DWT conversion in
// PPS_VCLOCK" rule: it projects legacy-CH2-API / actual-CH1 fire facts onto the PPS_VCLOCK timeline
// for scheduling diagnostics. VCLOCK and OCXO one-second subscribers receive
// authored edge facts whose DWT coordinates may be EMA-predicted from completed
// one-second intervals; counter32/GNSS identity remains exact authority.
// CLOCKS/Alpha owns measured-GNSS interval construction from consecutive
// authored edge DWTs.
//
// Lanes:
//   VCLOCK : QTimer1 CH0 counter + CH1 compare (TimePop legacy CH2 API wraps CH1)
//   OCXO1  : QTimer2 CH0 counter + CH1 compare
//   OCXO2  : QTimer3 CH3 physical adapter (pin 15 routes to TIMER3; pin 19/TIMER0 is SMBus SCL)
//   TimePop: QTimer1 CH1 varied compare intervals (foreground scheduler)
//
// The PPS GPIO ISR captures DWT as its first instruction, then reads the
// QTimer1 CH0 low-word counter.  process_interrupt maps that low-word
// hardware observation into the private synthetic 32-bit VCLOCK identity.
// When a rebootstrap is pending, it records the selected VCLOCK edge.
// Native QTimer1 CH1 custody now consumes that pending edge after TimePop has
// processed a legacy-CH2-API / actual-CH1 compare event, back-projects to the selected edge, and
// publishes the canonical PPS_VCLOCK epoch.  Native CH1 also owns steady-state
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

// Memory-pressure trim profile.  Keep the new priority-handoff forensics intact,
// but retire bulky legacy diagnostic buffers that are no longer part of timing
// authority.
static constexpr bool INTERRUPT_REDUCED_DIAGNOSTICS = true;

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

// VCLOCK DWT authorship.  VCLOCK counter32 and GNSS identity remain exact
// authorities; only the DWT coordinate assigned to the exact one-second edge
// is EMA-authored to suppress the known QTimer 4-cycle quantization floor.
static constexpr bool     VCLOCK_EMA_DWT_AUTHORITY_ENABLED = true;
static constexpr uint32_t VCLOCK_EMA_SHIFT = 4;  // alpha = 1/16
static constexpr uint32_t VCLOCK_EMA_ALPHA_NUMERATOR = 1;
static constexpr uint32_t VCLOCK_EMA_ALPHA_DENOMINATOR = (1U << VCLOCK_EMA_SHIFT);
static constexpr uint32_t VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES = 32;

// DWT interval admissibility gate.  Thermal drift of the Teensy DWT ruler is
// slow, so a one-second interval residual above this threshold is treated as
// an ISR/service excursion rather than a legitimate clock change.  Rejected
// samples are published as projected DWT endpoints and do not update the EMA.
static constexpr uint32_t DWT_EMA_INTERVAL_GATE_THRESHOLD_CYCLES = 500U;
static constexpr uint32_t VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES =
    DWT_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
static constexpr uint32_t OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES =
    DWT_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;

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
// sovereign QTimer1 CH1 rail, but it is no longer a rollover owner.  VCLOCK still uses this rail deliberately:
// TimePop's same-deadline coalescing keeps VCLOCK facts on one perishable ISR
// surface and avoids introducing a competing preemptive VCLOCK compare.
static constexpr uint64_t VCLOCK_HEARTBEAT_PERIOD_NS = 1000000ULL;
static constexpr const char* VCLOCK_HEARTBEAT_NAME = "VCLOCK_HEARTBEAT";

// PPS relay ownership lives in process_interrupt because the visible relay edge
// is a physical PPS witness.  The rising edge is asserted directly inside the
// PPS GPIO ISR; the 500 ms deassert is driven by intrinsic QTimer1 CH1
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

// EMA authority for OCXO one-second DWT publication.  In steady state each
// OCXO lane publishes a single local one-second edge compare.  The 16-bit
// hardware compare is armed only when the authored one-second target is inside
// a safe low-word window; CH2 implicit rollover tending keeps the synthetic
// 32-bit lane projection fresh between those one-second edges.  SmartZero is
// the remaining short-cadence user and temporarily owns +10,000-tick samples
// during acquisition only.  The subscriber-facing edge DWT is last-published
// + EMA(observed one-second interval), smoothing service/custody excursions
// while preserving raw ISR DWT as diagnostics.
static constexpr bool     OCXO_EMA_DWT_AUTHORITY_ENABLED = true;
static constexpr uint32_t OCXO_EMA_SHIFT = 7;  // alpha = 1/128 MULE CHANGE
static constexpr uint32_t OCXO_EMA_ALPHA_NUMERATOR = 1;
static constexpr uint32_t OCXO_EMA_ALPHA_DENOMINATOR = (1U << OCXO_EMA_SHIFT);

// PPS-Yardstick OCXO DWT inference (Stage 1 -- OBSERVATIONAL ONLY).
//
// Model: the physical PPS-to-PPS DWT interval (GPIO path, unquantized) is the
// live yardstick for Teensy CPU-clock drift.  The OCXO's own second is stable
// to ~1e-11, so the only lawful second-to-second change in its measured DWT
// interval is that same yardstick drift:
//
//   O_inferred[n] = O_chain[n-1] * G_now / G_prev
//
// carried in Q16 fixed point so sub-cycle truth accumulates instead of being
// re-quantized each second.  The QTimer-measured interval (4-cycle lattice)
// is demoted to validator: agreement within the gate band confirms the
// inferred value; disagreement marks an excursion of the ISR/service
// displacement class.
//
// In Stage 1 the EMA remains the published DWT authority.  The yardstick rail
// runs in parallel and publishes its full audit through dwt_repair_diag_t /
// interrupt_capture_diag_t and the lane report, so live campaigns can decide
// the gate threshold, PPS-jitter handling, and chain seed/anchor policy
// before any authority flip.
static constexpr bool     OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED = true;  // Stage 2: yardstick is authority
static constexpr uint32_t OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES = 16;
// Stage 2 anchored authority rail.  The published endpoint is a PI-anchored
// yardstick chain: each second the anchored interval is scaled by the same
// PPS yardstick step as the free chain, the endpoint advances by it, and on
// gate-agree seconds the observed endpoint corrects the loop:
//
//   err       = observed - anchored_endpoint        (Q16)
//   endpoint += err >> ANCHOR_P_SHIFT               (proportional, phase)
//   interval += err >> ANCHOR_I_SHIFT               (integral, frequency)
//
// Plover2 measured a +2 cycle/seed-bias interval and a +1.76 cycles/s free
// chain walk.  A proportional-only anchor would park K * bias cycles off
// truth; the integral term drives steady-state error to ZERO under constant
// interval bias and slowly absorbs real per-lane oven drift -- it is what
// makes the two lanes individual.  With P=1/8 and I=1/256 at 1 Hz the loop
// is critically damped (zeta = 1) and the +/-2 cycle observed lattice noise
// reaches the published endpoint attenuated to ~0.25 cycles.  On excursion
// seconds no correction is applied: the rail coasts on pure inference (the
// excursion guard).  Unlike the EMA, the yardstick feed-forward carries the
// thermal slope, so the loop only ever fights zero-mean noise and bias --
// there is no systematic lag and no integer dead zone.
// Retuned 2026-06-12 from Zero3 evidence: at P=1/8, I=1/256 the loop's
// standing endpoint error is ~8x any interval mistune and its ~100 s natural
// period sat next to the campaign's thermal oscillation; the published
// endpoint wandered +/-25 cycles common-mode (aerr columns, raw_cycles).
// P=1/2 caps the standing error at ~2x interval mistune; I=1/64 drops the
// loop period to ~16 s (zeta = 2, overdamped) so it recovers between thermal
// reversals.  Cost: ~1 extra cycle of lattice noise in the published
// endpoint, against the +/-25 it eliminates.
static constexpr uint32_t OCXO_YARDSTICK_ANCHOR_P_SHIFT = 1;   // 1/2
static constexpr uint32_t OCXO_YARDSTICK_ANCHOR_I_SHIFT = 6;   // 1/64

// Anchor acquisition gear shift.  A cold or re-seeded anchor rail carries the
// seed's quantization bias in its interval; with tracking-gain integral
// (1/256) the loop parks at ~8x that bias for tens of seconds (P-term
// balance, tau ~ 30 s) -- demonstrably unconverged state that correctly
// blocks zero-worthiness but slows cold-boot readiness.  For the first
// OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS agree seconds after any seed or
// reseed the integral runs at acquisition gain (1/32, tau ~ 4 s), then
// drops to tracking gain.  Qualification thresholds are unchanged: the gate
// demands the same settled evidence; the loop simply earns it sooner.
static constexpr uint32_t OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS = 32;
static constexpr uint32_t OCXO_YARDSTICK_ANCHOR_FAST_I_SHIFT = 4;  // 1/16

// ============================================================================
// SmartZero Generation 2 ("ZeroWorthy") -- convergence-qualified zero, Stage A
// ============================================================================
//
// Doctrine: zero is the SELECTION of an already-qualified canonical second,
// not the acquisition of new proof.  The v1 SmartZero pair test certifies
// that one millisecond of custody was mechanically clean (a glitch detector,
// ~4 ppm stringency over its 1 ms baseline); it cannot see common-mode
// displacement, lattice phase, or post-restart oven retrace -- all of which
// it enshrines into the epoch.  Gen-2 instead maintains a continuous
// per-lane ZERO-WORTHY verdict from the yardstick authority rail:
//
//   A lane is zero-worthy when, for ZERO_WORTHY_STREAK_SECONDS consecutive
//   seconds, every second was a clean gate-agree second (no excursion, no
//   stale yardstick, no adjacency fault, no reseed), the PI anchor loop
//   error stayed within ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES, and the
//   window sum of loop errors stayed within
//   ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES.
//
// The window-sum criterion is the retrace detector: the PI loop converts a
// drifting oven into a persistent small same-sign error -- each second
// individually innocent, the sum guilty (Plover3's post-flash retrace ran
// ~+4 cycles/s of sustained loop error and must disqualify zeroing).
//
// An 8-second streak at gate threshold 16 over one-second baselines is an
// effective parts-per-billion, two-independent-witness qualification; a
// displacement common to both edges of one v1 millisecond pair cannot
// survive eight seconds of agreement between the QTimer lane rail and the
// PPS/GPIO yardstick without being real physics.
//
// Stage A is OBSERVATIONAL: verdicts, streaks, and disqualification
// forensics are maintained and reported (INTERRUPT.REPORT_LANE
// section=zero_worthy), and epoch capture packets are counted as
// qualified/unqualified, but ZERO authority remains with v1 SmartZero until
// SMARTZERO2_SELECTION_AUTHORITY_ENABLED flips after live validation.
static constexpr bool     SMARTZERO2_SELECTION_AUTHORITY_ENABLED = false;
static constexpr uint32_t ZERO_WORTHY_STREAK_SECONDS = 8;
// Bounds recalibrated from live Zero1 evidence (2026-06-12): at tracking
// gain the anchor loop error on hardware carries the physical PPS witness
// jitter through the yardstick feed-forward, so steady-state |auth_error|
// runs hotter than the noiseless simulation predicted and a 6-cycle bound
// thrashed on error magnitude.  10/40 keeps the same shape (window mean
// bound = magnitude/2) while remaining well inside the 16-cycle excursion
// gate; the raw_cycles anchor-error histogram and policy table are the
// instruments for refining these from campaign data.
static constexpr int32_t  ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES = 10;
static constexpr int32_t  ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES = 40;
// Coherent repeated excursions mean the OCXO genuinely moved (servo step /
// retrace), not that the measurement glitched; reseed the chain after this
// many coherent excursions instead of diverging forever.
static constexpr uint32_t OCXO_YARDSTICK_COHERENT_RESEED_STREAK = 3;
// Chain endpoint is carried modulo 2^48 in Q16, which is exactly modulo 2^32
// DWT cycles -- wrap arithmetic stays consistent with the uint32 DWT domain.
static constexpr uint64_t YARDSTICK_ENDPOINT_Q16_MASK = (1ULL << 48) - 1ULL;
// A one-second PPS yardstick interval qualifies only if the VCLOCK counter
// advanced one nominal second within this tick tolerance (100 us).  Outside
// the band the interval is a dropout/multi-second artifact, not a yardstick.
static constexpr uint32_t PPS_YARDSTICK_COUNTER_TOLERANCE_TICKS = 1000U;
// Sanity bound on the per-second yardstick step.  |G_now - G_prev| beyond
// ~100 ppm/s is physically impossible thermal slew and also protects the
// 64-bit interval-scaling product from overflow.
static constexpr uint32_t PPS_YARDSTICK_MAX_ABS_DELTA_G_CYCLES = 100000U;

// OCXO local target constants.  Normal OCXO operation is one-second edge
// compare custody: each public event is exactly 10,000,000 OCXO ticks after
// the previous authored target.  OCXO_CADENCE_INTERVAL_TICKS remains the
// SmartZero acquisition sample spacing and the low-word arm-window scale; it
// is not a steady-state 1 kHz OCXO publication ladder.
static constexpr uint32_t OCXO_CADENCE_INTERVAL_TICKS = 10000U;
static constexpr uint32_t OCXO_WITNESS_ONE_SECOND_COUNTS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS == 10000000U,
              "OCXO witness edge interval must be one 10 MHz second");
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS ==
              (OCXO_CADENCE_INTERVAL_TICKS * TICKS_PER_SECOND_EVENT),
              "OCXO one-second interval must equal 1000 SmartZero sample ticks");
static constexpr uint32_t OCXO_WITNESS_ARM_WINDOW_TICKS = OCXO_CADENCE_INTERVAL_TICKS;
static constexpr uint32_t OCXO_WITNESS_MIN_ARM_LEAD_TICKS = 64U;

// OCXO quiet-phase report surface.
//
// These constants describe the nominal phase placement of OCXO lane samples
// inside the VCLOCK millisecond cell.  They are retained as report/back-compat
// surfaces, but steady-state OCXO publication is now the local one-second edge
// compare.  SmartZero may still use +10,000-tick acquisition samples.
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
static constexpr bool REGRESSION_ANY_ENABLED =
    VCLOCK_LINEAR_REGRESSION_ENABLED || OCXO_LINEAR_REGRESSION_ENABLED;
static constexpr uint32_t REGRESSION_RETAINED_SAMPLE_COUNT =
    REGRESSION_ANY_ENABLED ? REGRESSION_SAMPLES_PER_SECOND : 1U;


// Regression is experimental/diagnostic in this build.  The subscriber-facing
// DWT stays traditional while regression_inferred_dwt_at_event exposes what the
// fitted edge would have been.
static constexpr bool LINEAR_REGRESSION_AUTHORED_DWT = false;
static constexpr bool LINEAR_REGRESSION_DIAGNOSTIC_ONLY = false;

// ============================================================================
// Raw ISR-entry DWT interval probe
// ============================================================================
//
// Diagnostic-only truth recorder.  This deliberately captures the simplest
// possible one-second interval surface:
//
//   interval[n] = isr_entry_dwt_raw[n] - isr_entry_dwt_raw[n-1]
//
// No latency correction, no event-coordinate conversion, no EMA, no bridge
// projection, and no subscriber DWT authority participate.  The report exists
// to compare literal first-instruction ISR-entry timing for VCLOCK, OCXO1, and
// OCXO2 with the richer event-coordinate custody chain.
static constexpr uint32_t ISR_RAW_DWT_INTERVAL_RING_SIZE = 128U;

struct isr_raw_dwt_interval_probe_t {
  uint32_t endpoint_count = 0;
  uint32_t interval_count = 0;
  uint32_t write_index = 0;  // next interval slot to write
  bool     last_dwt_valid = false;
  uint32_t last_isr_entry_dwt_raw = 0;
  bool     last_interval_valid = false;
  uint32_t last_interval_cycles = 0;
  int32_t  last_interval_delta_cycles = 0;
  uint32_t intervals[ISR_RAW_DWT_INTERVAL_RING_SIZE]{};
};

static isr_raw_dwt_interval_probe_t g_raw_dwt_probe_vclock = {};
static isr_raw_dwt_interval_probe_t g_raw_dwt_probe_ocxo1 = {};
static isr_raw_dwt_interval_probe_t g_raw_dwt_probe_ocxo2 = {};

static isr_raw_dwt_interval_probe_t*
isr_raw_dwt_probe_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return &g_raw_dwt_probe_vclock;
  if (kind == interrupt_subscriber_kind_t::OCXO1)  return &g_raw_dwt_probe_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2)  return &g_raw_dwt_probe_ocxo2;
  return nullptr;
}

static const isr_raw_dwt_interval_probe_t*
isr_raw_dwt_probe_for_const(interrupt_subscriber_kind_t kind) {
  return isr_raw_dwt_probe_for(kind);
}

static void isr_raw_dwt_probe_reset(interrupt_subscriber_kind_t kind) {
  isr_raw_dwt_interval_probe_t* p = isr_raw_dwt_probe_for(kind);
  if (!p) return;
  *p = isr_raw_dwt_interval_probe_t{};
}

static void isr_raw_dwt_probe_reset_all(void) {
  isr_raw_dwt_probe_reset(interrupt_subscriber_kind_t::VCLOCK);
  isr_raw_dwt_probe_reset(interrupt_subscriber_kind_t::OCXO1);
  isr_raw_dwt_probe_reset(interrupt_subscriber_kind_t::OCXO2);
}

static void isr_raw_dwt_probe_record(interrupt_subscriber_kind_t kind,
                                     uint32_t isr_entry_dwt_raw) {
  isr_raw_dwt_interval_probe_t* p = isr_raw_dwt_probe_for(kind);
  if (!p) return;

  p->endpoint_count++;

  if (p->last_dwt_valid) {
    const uint32_t interval = isr_entry_dwt_raw - p->last_isr_entry_dwt_raw;
    const bool have_previous_interval = p->last_interval_valid;

    p->intervals[p->write_index] = interval;
    p->write_index =
        (p->write_index + 1U) % ISR_RAW_DWT_INTERVAL_RING_SIZE;
    if (p->interval_count < ISR_RAW_DWT_INTERVAL_RING_SIZE) {
      p->interval_count++;
    }

    p->last_interval_delta_cycles = have_previous_interval
        ? (int32_t)((int64_t)interval - (int64_t)p->last_interval_cycles)
        : 0;
    p->last_interval_cycles = interval;
    p->last_interval_valid = true;
  }

  p->last_isr_entry_dwt_raw = isr_entry_dwt_raw;
  p->last_dwt_valid = true;
}


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
// Priority handoff migration baseline — safety rails
// ============================================================================
//
// These constants and mirrors make the current capture/handoff architecture and
// applied NVIC priorities visible in INTERRUPT.REPORT.  This release changes
// interrupt behavior deliberately: priority 0 becomes reentry-safe capture,
// and priority 16 performs custody continuation.

// Priority doctrine for the reentry-safe capture / priority-handoff migration.
//
// Priority 0 is the sacred capture tier.  It must capture immutable hardware
// facts, defuse the interrupt source, enqueue a reentry-safe capture packet,
// pend the handoff tier, and exit.  Priority 16 is the handoff tier: still
// interrupt context, still before foreground/ASAP, but no longer able to delay
// sacred priority-0 clock capture.
static constexpr uint32_t INTERRUPT_PRIORITY_CAPTURE        = 0;
static constexpr uint32_t INTERRUPT_PRIORITY_HANDOFF        = 16;
static constexpr uint32_t INTERRUPT_PRIORITY_BACKGROUND_IRQ = 32;

static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY  = INTERRUPT_PRIORITY_CAPTURE;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY  = INTERRUPT_PRIORITY_CAPTURE;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY  = INTERRUPT_PRIORITY_CAPTURE;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY = INTERRUPT_PRIORITY_CAPTURE;

// TimePop currently owns IRQ_SOFTWARE(70) for its POC handoff.  process_interrupt
// deliberately uses an adjacent normal external IRQ so the two POCs can coexist
// until TimePop's duplicate handoff layer is retired.
static constexpr uint32_t     INTERRUPT_HANDOFF_IRQ_NUMBER = 71U;
static constexpr IRQ_NUMBER_t INTERRUPT_HANDOFF_IRQ =
    (IRQ_NUMBER_t)INTERRUPT_HANDOFF_IRQ_NUMBER;
static constexpr const char*  INTERRUPT_HANDOFF_IRQ_NAME = "NVIC_EXTERNAL_71";
static constexpr const char*  INTERRUPT_CAPTURE_DISCIPLINE = "REENTRY_SAFE_CAPTURE";
static constexpr const char*  INTERRUPT_HANDOFF_MECHANISM = "NVIC_ISPR_PRIORITY_HANDOFF";
static constexpr uint32_t     INTERRUPT_HANDOFF_DRAIN_BUDGET = 32U;

static constexpr uint32_t HANDOFF_QTIMER1_CH1_RING_SIZE = 8U;
static constexpr uint32_t HANDOFF_QTIMER1_CH2_RING_SIZE = 16U;
static constexpr uint32_t HANDOFF_OCXO_RING_SIZE = 8U;
static constexpr uint32_t HANDOFF_PPS_RING_SIZE = 4U;


// ============================================================================
// Uniform QuadTimer lane doctrine — experiment
// ============================================================================
//
// Primary clock lanes now use the same hardware channel roles:
//   CH0 = passive 16-bit counter observation
//   CH1 = active compare rail
//
// The public TimePop API still uses its historical "CH2" function names so
// process_timepop does not need to move in this experiment; those calls now
// target QTimer1 CH1 internally.  The legacy hosted QTimer1 CH1 service rail
// is disabled in this build so QTimer1 does not carry a second active compare
// geometry while we test the clock-lane equivalence claim.
static constexpr uint8_t QTIMER_CLOCK_COUNTER_CH = 0;
static constexpr uint8_t QTIMER_CLOCK_COMPARE_CH = 1;
static constexpr uint8_t QTIMER1_AUX_COMPARE_CH  = 2;  // retired/off
static constexpr uint8_t QTIMER_CLOCK_PCS        = 0;
// OCXO2 physical adapter:
//
// Teensy pin 15 is physically routed to QTimer3 TIMER3, not TIMER0.  Pin 19
// would be the clean QTimer3 TIMER0 sibling, but it is occupied by SMBus SCL
// in this system.  Therefore OCXO2 deliberately remains a documented physical
// exception: QTimer3 CH3 counter+compare, PCS=3.
static constexpr uint8_t QTIMER3_OCXO2_PHYSICAL_CH  = 3;
static constexpr uint8_t QTIMER3_OCXO2_PHYSICAL_PCS = 3;
static constexpr const char* QTIMER_UNIFORM_DOCTRINE =
    "CLOCK_CH0_COUNTER_CH1_COMPARE_EXCEPT_OCXO2_PIN15_QTIMER3_CH3_ADAPTER";

// ============================================================================
// SpinCatch landing-only witness — symmetric OCXO lead tuning
// ============================================================================
//
// SpinCatch looping is now system-wide TimePop idle work.  The per-lane surface
// below is landing-only and deliberately symmetric for OCXO1/OCXO2: after each
// published OCXO one-second fact, schedule a low-duty TimePop callback before
// the next already-authored target.  The callback records DWT/counter32 landing
// facts and immediately returns.  It does not spin, does not reprogram or clear
// any OCXO compare register, does not alter event DWT authority, and does not
// expand interrupt_capture_diag_t.

static constexpr bool SPINCATCH_REPORT_SHELL_SUPPORTED = true;
static constexpr bool SPINCATCH_PLANNER_ENABLED = false;
static constexpr bool SPINCATCH_VCLOCK_ENABLED = false;
static constexpr bool SPINCATCH_OCXO_LANDING_ONLY_ENABLED = false;
static constexpr const char* SPINCATCH_REPORT_MODE = "RETIRED_MEMORY_TRIM";
static constexpr uint32_t SPINCATCH_APPROACH_US = 10U;
static constexpr uint32_t SPINCATCH_APPROACH_TICKS = SPINCATCH_APPROACH_US * 10U;
static constexpr uint32_t SPINCATCH_OCXO_LANDING_DUTY_DIVISOR = 1U;
static constexpr uint32_t SPINCATCH_LANDING_MIN_LEAD_TICKS = 128U;
static constexpr uint32_t SPINCATCH_LANDING_ATTEMPT_RING_SIZE = 1U;
static constexpr uint32_t SPINCATCH_LANDING_STALE_TARGET_PERIODS = 2U;
static_assert(SPINCATCH_APPROACH_TICKS == 100U,
              "SpinCatch 10 us approach must be 100 ticks at 10 MHz");

// ============================================================================
// SpinIdle ISR-entry witness
// ============================================================================
//
// Step 6.1 turns SpinCatch into an ISR-side admissibility test against the
// global TimePop-owned idle DWT witness.  This remains diagnostic only: it
// does not repair, replace, veto, or re-author any event DWT.

static constexpr bool     SPINIDLE_ISR_WITNESS_SUPPORTED = true;
static constexpr bool     SPINIDLE_ISR_WITNESS_ENABLED = true;
static constexpr uint32_t SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES = 100U;

struct spinidle_isr_capture_t {
  bool supported = SPINIDLE_ISR_WITNESS_SUPPORTED;
  bool enabled = SPINIDLE_ISR_WITNESS_ENABLED;
  bool shadow_valid = false;
  uint32_t shadow_dwt = 0;
  uint32_t shadow_to_isr_entry_cycles = 0;
  uint32_t threshold_cycles = SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES;
};

static inline spinidle_isr_capture_t spinidle_capture_at_isr_entry(
    uint32_t isr_entry_dwt_raw) {
  spinidle_isr_capture_t c{};
  c.shadow_dwt = (uint32_t)g_timepop_idle_witness_shadow_dwt;
  c.threshold_cycles = SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES;
  c.shadow_to_isr_entry_cycles = c.shadow_dwt
      ? (uint32_t)(isr_entry_dwt_raw - c.shadow_dwt)
      : 0xFFFFFFFFUL;
  c.shadow_valid = c.enabled && c.shadow_dwt != 0 &&
      c.shadow_to_isr_entry_cycles <= c.threshold_cycles;
  return c;
}

static inline void spinidle_copy_to_diag(interrupt_capture_diag_t& diag,
                                         const spinidle_isr_capture_t& c) {
  diag.spinidle_shadow_valid = c.shadow_valid;
  diag.spinidle_shadow_dwt = c.shadow_dwt;
  diag.spinidle_shadow_to_isr_entry_cycles = c.shadow_to_isr_entry_cycles;
  diag.spinidle_shadow_valid_threshold_cycles = c.threshold_cycles;
}

static spinidle_isr_capture_t g_spinidle_pps_last_capture = {};
static spinidle_isr_capture_t g_spinidle_timepop_last_capture = {};


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
// PPS yardstick store (seqlock) -- physical PPS-to-PPS DWT interval pair
// ============================================================================
//
// Authored once per PPS edge from the GPIO ISR.  G_now and G_prev are
// consecutive physical one-second DWT intervals measured on the unquantized
// GPIO path.  Their ratio is the live CPU-clock (yardstick) drift consumed by
// the OCXO yardstick inference rail.  An interval qualifies only when the
// VCLOCK counter delta across the same edge pair is one nominal second within
// PPS_YARDSTICK_COUNTER_TOLERANCE_TICKS; a non-qualifying edge clears the
// pair so a GNSS dropout cannot masquerade as thermal drift.

struct pps_yardstick_store_t {
  volatile uint32_t seq = 0;
  volatile bool     valid = false;             // both intervals present
  volatile uint32_t sequence = 0;              // PPS edge count closing G_now
  volatile uint32_t interval_now_cycles = 0;   // G_now
  volatile uint32_t interval_prev_cycles = 0;  // G_prev
  volatile uint32_t accept_count = 0;
  volatile uint32_t reject_count = 0;
  volatile uint32_t reset_count = 0;
};

static pps_yardstick_store_t g_pps_yardstick;

struct pps_yardstick_snapshot_t {
  bool     valid = false;
  uint32_t sequence = 0;
  uint32_t interval_now_cycles = 0;
  uint32_t interval_prev_cycles = 0;
  uint32_t accept_count = 0;
  uint32_t reject_count = 0;
  uint32_t reset_count = 0;
};

// Single writer (PPS GPIO ISR); foreground readers via pps_yardstick_load.
static void pps_yardstick_record_interval(uint32_t pps_sequence,
                                          uint32_t interval_cycles,
                                          uint32_t counter_delta_ticks) {
  const uint32_t nominal = (uint32_t)VCLOCK_COUNTS_PER_SECOND;
  const uint32_t tick_error = (counter_delta_ticks >= nominal)
      ? counter_delta_ticks - nominal
      : nominal - counter_delta_ticks;

  g_pps_yardstick.seq++;
  dmb_barrier();
  if (tick_error <= PPS_YARDSTICK_COUNTER_TOLERANCE_TICKS &&
      interval_cycles != 0) {
    g_pps_yardstick.interval_prev_cycles = g_pps_yardstick.interval_now_cycles;
    g_pps_yardstick.interval_now_cycles = interval_cycles;
    g_pps_yardstick.valid = (g_pps_yardstick.interval_prev_cycles != 0);
    g_pps_yardstick.sequence = pps_sequence;
    g_pps_yardstick.accept_count++;
  } else {
    g_pps_yardstick.interval_prev_cycles = 0;
    g_pps_yardstick.interval_now_cycles = 0;
    g_pps_yardstick.valid = false;
    g_pps_yardstick.sequence = pps_sequence;
    g_pps_yardstick.reject_count++;
  }
  dmb_barrier();
  g_pps_yardstick.seq++;
}

static bool pps_yardstick_load(pps_yardstick_snapshot_t& out) {
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_pps_yardstick.seq;
    dmb_barrier();
    out.valid = g_pps_yardstick.valid;
    out.sequence = g_pps_yardstick.sequence;
    out.interval_now_cycles = g_pps_yardstick.interval_now_cycles;
    out.interval_prev_cycles = g_pps_yardstick.interval_prev_cycles;
    out.accept_count = g_pps_yardstick.accept_count;
    out.reject_count = g_pps_yardstick.reject_count;
    out.reset_count = g_pps_yardstick.reset_count;
    dmb_barrier();
    const uint32_t s2 = g_pps_yardstick.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }
  out = pps_yardstick_snapshot_t{};
  return false;
}

static void pps_yardstick_reset(void) {
  g_pps_yardstick.seq++;
  dmb_barrier();
  g_pps_yardstick.valid = false;
  g_pps_yardstick.interval_now_cycles = 0;
  g_pps_yardstick.interval_prev_cycles = 0;
  g_pps_yardstick.reset_count++;
  dmb_barrier();
  g_pps_yardstick.seq++;
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

// SmartZero Gen-2 system-level worthiness evidence.  all_lanes_worthy is
// authored in deferred dispatch context (lane fact drain) and read by the
// PPS ISR when stamping epoch capture qualification counters; it is a single
// volatile bool, so the cross-context read is coherent.
struct smartzero2_system_t {
  volatile bool all_lanes_worthy = false;
  uint32_t all_worthy_second_count = 0;
  uint32_t all_worthy_streak_seconds = 0;
  uint32_t first_all_worthy_pps_sequence = 0;
  uint32_t last_pps_sequence_checked = 0;
  uint32_t epoch_captures_qualified = 0;
  uint32_t epoch_captures_unqualified = 0;
};
static smartzero2_system_t g_smartzero2;

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
  // Gen-2 evidence: would this capture packet have been selection-eligible?
  if (g_smartzero2.all_lanes_worthy) {
    g_smartzero2.epoch_captures_qualified++;
  } else {
    g_smartzero2.epoch_captures_unqualified++;
  }
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
static void pps_edge_dispatch_arm_or_call_from_foreground(const char* name);

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
  bool     synthetic = false;
  uint32_t original_dwt = 0;
  uint32_t predicted_dwt = 0;
  uint32_t used_dwt = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t event_dwt_from_isr_entry_raw = 0;
  int32_t  isr_entry_to_event_correction_cycles = 0;
  int32_t  error_cycles = 0;
  uint32_t threshold_cycles = 0;
  const char* reason = "none";

  // One-second DWT interval gate audit.  original_dwt/predicted_dwt/used_dwt
  // are endpoint facts; these fields expose the interval decision that
  // protected the EMA from a raw service-time excursion.
  bool     interval_gate_valid = false;
  bool     interval_sample_accepted = false;
  bool     interval_sample_rejected = false;
  bool     interval_ema_updated = false;
  uint32_t interval_observed_cycles = 0;
  uint32_t interval_prediction_cycles = 0;
  uint32_t interval_effective_cycles = 0;
  int32_t  interval_residual_cycles = 0;
  uint32_t interval_gate_threshold_cycles = 0;
  uint32_t interval_accept_count = 0;
  uint32_t interval_reject_count = 0;
  bool     interval_resync_applied = false;
  uint32_t interval_resync_count = 0;
  uint32_t interval_reject_streak = 0;

  // OCXO event-lineage audit.  A normal OCXO one-second event must advance
  // exactly OCXO_WITNESS_ONE_SECOND_COUNTS target ticks from the previous
  // published OCXO event.  If this adjacency check fails, the observed DWT
  // interval is custody evidence only: publish the projected endpoint, do not
  // update/resync the EMA, and expose the original endpoint for forensics.
  bool     interval_adjacency_gate_valid = false;
  bool     interval_adjacency_ok = false;
  bool     interval_adjacency_rejected = false;
  uint32_t interval_counter_delta_ticks = 0;
  uint32_t interval_expected_counter_delta_ticks = 0;
  uint32_t interval_adjacency_reject_count = 0;

  // PPS-Yardstick inference audit (Stage 1 -- observational).  These fields
  // run in parallel with the EMA decision above; they change no authority.
  bool     yardstick_valid = false;
  bool     yardstick_stale = false;
  bool     yardstick_seeded_this_event = false;
  bool     yardstick_excursion = false;
  uint32_t yardstick_pps_sequence = 0;
  uint32_t yardstick_pps_seq_delta = 0;
  uint32_t yardstick_g_now_cycles = 0;
  uint32_t yardstick_g_prev_cycles = 0;
  uint32_t yardstick_inferred_interval_cycles = 0;
  uint32_t yardstick_observed_interval_cycles = 0;
  int32_t  yardstick_inferred_minus_observed_cycles = 0;
  uint32_t yardstick_inferred_endpoint_dwt = 0;
  uint32_t yardstick_inferred_endpoint_frac_q16 = 0;
  int32_t  yardstick_endpoint_minus_observed_cycles = 0;
  uint32_t yardstick_gate_threshold_cycles = 0;
  uint32_t yardstick_gate_agree_count = 0;
  uint32_t yardstick_gate_excursion_count = 0;

  // Stage 2 authority audit.  ema_dwt_at_event preserves the EMA rail's
  // would-have-been endpoint as a diagnostic (the EMA no longer publishes).
  // The yardstick_auth_* fields expose the anchored publication ledger:
  // endpoint, PI loop error, and whether the anchor correction fired.
  bool     yardstick_authority = false;
  uint32_t ema_dwt_at_event = 0;
  uint32_t yardstick_auth_endpoint_dwt = 0;
  uint32_t yardstick_auth_endpoint_frac_q16 = 0;
  int32_t  yardstick_auth_error_cycles = 0;
  bool     yardstick_auth_anchor_applied = false;
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
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK", interrupt_provider_kind_t::QTIMER1, interrupt_lane_t::QTIMER1_CH1_COMP },
  { interrupt_subscriber_kind_t::OCXO1,  "OCXO1",  interrupt_provider_kind_t::QTIMER2, interrupt_lane_t::QTIMER2_CH1_COMP },
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
// SpinCatch report-only lane state
// ============================================================================

struct spincatch_lane_report_t {
  bool supported = SPINCATCH_REPORT_SHELL_SUPPORTED;
  bool enabled = false;
  bool planner_enabled = SPINCATCH_PLANNER_ENABLED;
  uint32_t success_count = 0;
  uint32_t missed_count = 0;
  uint32_t timeout_count = 0;
  uint32_t clash_count = 0;

  // Planner surface.  These fields are computed from already-authored lane
  // targets.  They do not alter compare registers or event authority.
  bool     planner_valid = false;
  uint32_t planner_count = 0;
  uint32_t planner_skip_count = 0;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint32_t approach_counter32 = 0;
  uint16_t approach_low16 = 0;
  uint32_t approach_ticks = SPINCATCH_APPROACH_TICKS;
  uint32_t approach_us = SPINCATCH_APPROACH_US;
  uint32_t approach_dwt_cycles = 0;
  uint32_t now_counter32 = 0;
  uint32_t ticks_until_target = 0;

  // Landing-only surface.  OCXO1 and OCXO2 schedule the same TimePop
  // callback pattern per one-second event; callbacks record landing facts and return immediately.
  bool     landing_only_enabled = false;
  bool     landing_pending = false;
  bool     landing_last_success = false;
  bool     landing_last_target_changed = false;
  uint32_t landing_duty_divisor = 0;
  uint32_t landing_duty_counter = 0;
  uint32_t landing_schedule_count = 0;
  uint32_t landing_schedule_skip_count = 0;
  uint32_t landing_schedule_fail_count = 0;
  uint32_t landing_count = 0;
  uint32_t landing_too_close_count = 0;
  uint32_t landing_target_changed_count = 0;
  uint32_t landing_attempt_ring_size = SPINCATCH_LANDING_ATTEMPT_RING_SIZE;
  uint32_t landing_attempt_active_count = 0;
  uint32_t landing_attempt_stale_count = 0;
  uint32_t landing_attempt_duplicate_skip_count = 0;
  uint32_t landing_attempt_ring_full_count = 0;
  uint32_t landing_attempt_generation = 0;
  uint32_t landing_target_counter32 = 0;
  uint16_t landing_target_low16 = 0;
  uint32_t landing_scheduled_counter32 = 0;
  uint32_t landing_scheduled_ticks_until_target = 0;
  uint64_t landing_scheduled_delay_ns = 0;
  uint32_t landing_scheduled_dwt = 0;
  uint32_t landing_dwt = 0;
  uint32_t landing_counter32 = 0;
  int32_t  landing_offset_ticks = 0;
  uint32_t landing_ticks_until_target = 0;
  uint32_t landing_fire_vclock_raw = 0;
  uint32_t landing_fire_dwt = 0;
  timepop_handle_t landing_handle = TIMEPOP_INVALID_HANDLE;
};

static spincatch_lane_report_t g_spincatch_vclock = {};
static spincatch_lane_report_t g_spincatch_ocxo1 = {};
static spincatch_lane_report_t g_spincatch_ocxo2 = {};

// Active-token flags are deliberately not armed in Step 5.  Landing-only does
// not use the shadow handoff yet; it only schedules a foreground callback.
static volatile bool g_spincatch_vclock_capture_active = false;
static volatile bool g_spincatch_ocxo1_capture_active = false;
static volatile bool g_spincatch_ocxo2_capture_active = false;

struct spincatch_landing_context_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  uint32_t attempt_index = 0;
  uint32_t generation = 0;
};

struct spincatch_landing_attempt_t {
  bool active = false;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  uint32_t generation = 0;
  timepop_handle_t handle = TIMEPOP_INVALID_HANDLE;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint32_t scheduled_counter32 = 0;
  uint32_t scheduled_ticks_until_target = 0;
  uint64_t scheduled_delay_ns = 0;
  uint32_t scheduled_dwt = 0;
};

static spincatch_landing_attempt_t
    g_spincatch_ocxo1_attempts[SPINCATCH_LANDING_ATTEMPT_RING_SIZE] = {};
static spincatch_landing_attempt_t
    g_spincatch_ocxo2_attempts[SPINCATCH_LANDING_ATTEMPT_RING_SIZE] = {};
static spincatch_landing_context_t
    g_spincatch_ocxo1_landing_contexts[SPINCATCH_LANDING_ATTEMPT_RING_SIZE] = {};
static spincatch_landing_context_t
    g_spincatch_ocxo2_landing_contexts[SPINCATCH_LANDING_ATTEMPT_RING_SIZE] = {};

static bool spincatch_enabled_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return SPINCATCH_VCLOCK_ENABLED;
  if (kind == interrupt_subscriber_kind_t::OCXO1 ||
      kind == interrupt_subscriber_kind_t::OCXO2) {
    return SPINCATCH_OCXO_LANDING_ONLY_ENABLED;
  }
  return false;
}

static uint32_t spincatch_landing_duty_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1 ||
      kind == interrupt_subscriber_kind_t::OCXO2) {
    return SPINCATCH_OCXO_LANDING_DUTY_DIVISOR;
  }
  return 0U;
}

static spincatch_lane_report_t* spincatch_report_for_kind(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return &g_spincatch_vclock;
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_spincatch_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_spincatch_ocxo2;
  return nullptr;
}

static volatile bool* spincatch_capture_active_for_kind(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return &g_spincatch_vclock_capture_active;
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_spincatch_ocxo1_capture_active;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_spincatch_ocxo2_capture_active;
  return nullptr;
}

static inline void spincatch_note_isr_entry(
    interrupt_subscriber_kind_t kind,
    uint32_t target_counter32,
    uint16_t target_low16,
    uint32_t isr_entry_dwt_raw) {
  volatile bool* active = spincatch_capture_active_for_kind(kind);
  if (!active || !*active) {
    (void)target_counter32;
    (void)target_low16;
    (void)isr_entry_dwt_raw;
    return;
  }

  // No lane can reach this branch in Step 5.  Future steps will install the
  // shadow capture handoff here after landing-only is proven stable.
  (void)target_counter32;
  (void)target_low16;
  (void)isr_entry_dwt_raw;
}

static void spincatch_report_reset_all(void) {
  g_spincatch_vclock = spincatch_lane_report_t{};
  g_spincatch_ocxo1 = spincatch_lane_report_t{};
  g_spincatch_ocxo2 = spincatch_lane_report_t{};

  g_spincatch_vclock.enabled = spincatch_enabled_for_kind(interrupt_subscriber_kind_t::VCLOCK);
  g_spincatch_ocxo1.enabled = spincatch_enabled_for_kind(interrupt_subscriber_kind_t::OCXO1);
  g_spincatch_ocxo2.enabled = spincatch_enabled_for_kind(interrupt_subscriber_kind_t::OCXO2);

  g_spincatch_ocxo1.landing_only_enabled = SPINCATCH_OCXO_LANDING_ONLY_ENABLED;
  g_spincatch_ocxo1.landing_duty_divisor = SPINCATCH_OCXO_LANDING_DUTY_DIVISOR;
  g_spincatch_ocxo1.landing_attempt_ring_size = SPINCATCH_LANDING_ATTEMPT_RING_SIZE;
  g_spincatch_ocxo2.landing_only_enabled = SPINCATCH_OCXO_LANDING_ONLY_ENABLED;
  g_spincatch_ocxo2.landing_duty_divisor = SPINCATCH_OCXO_LANDING_DUTY_DIVISOR;
  g_spincatch_ocxo2.landing_attempt_ring_size = SPINCATCH_LANDING_ATTEMPT_RING_SIZE;

  for (uint32_t i = 0; i < SPINCATCH_LANDING_ATTEMPT_RING_SIZE; i++) {
    g_spincatch_ocxo1_attempts[i] = spincatch_landing_attempt_t{};
    g_spincatch_ocxo2_attempts[i] = spincatch_landing_attempt_t{};
    g_spincatch_ocxo1_landing_contexts[i] = spincatch_landing_context_t{};
    g_spincatch_ocxo2_landing_contexts[i] = spincatch_landing_context_t{};
    g_spincatch_ocxo1_landing_contexts[i].kind = interrupt_subscriber_kind_t::OCXO1;
    g_spincatch_ocxo2_landing_contexts[i].kind = interrupt_subscriber_kind_t::OCXO2;
    g_spincatch_ocxo1_landing_contexts[i].attempt_index = i;
    g_spincatch_ocxo2_landing_contexts[i].attempt_index = i;
  }

  g_spincatch_vclock_capture_active = false;
  g_spincatch_ocxo1_capture_active = false;
  g_spincatch_ocxo2_capture_active = false;
}


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

  // VCLOCK EMA DWT authority.  This does not smooth VCLOCK counter32 or GNSS
  // identity; it only dequantizes the DWT coordinate assigned to the exact
  // PPS/VCLOCK one-second edge.
  bool     ema_initialized = false;
  bool     ema_interval_valid = false;
  uint32_t ema_last_observed_dwt = 0;
  uint32_t ema_last_emitted_dwt = 0;
  uint32_t ema_last_observed_interval_cycles = 0;
  uint32_t ema_interval_cycles = 0;
  uint32_t ema_last_predicted_dwt = 0;
  uint32_t ema_last_interval_prediction_cycles = 0;
  uint32_t ema_last_effective_interval_cycles = 0;
  int32_t  ema_last_error_cycles = 0;
  int32_t  ema_last_interval_residual_cycles = 0;
  uint32_t ema_max_abs_error_cycles = 0;
  uint32_t ema_update_count = 0;
  uint32_t ema_gate_accept_count = 0;
  uint32_t ema_gate_reject_count = 0;
  bool     ema_last_interval_gate_valid = false;
  bool     ema_last_interval_accepted = false;
  bool     ema_last_interval_rejected = false;
  bool     ema_last_interval_ema_updated = false;
  bool     ema_last_interval_resync_applied = false;
  uint32_t ema_gate_reject_streak = 0;
  uint32_t ema_gate_resync_count = 0;
  uint32_t ema_last_rejected_observed_interval_cycles = 0;
};
static vclock_lane_t g_vclock_lane;

static uint32_t dwt_ema_abs_i32(int32_t value) {
  return (uint32_t)(value < 0 ? -(int64_t)value : (int64_t)value);
}

static int32_t dwt_ema_round_shift(int32_t value, uint32_t shift) {
  if (shift == 0) return value;
  const int32_t half = (int32_t)(1U << (shift - 1U));
  if (value >= 0) return (value + half) >> shift;
  return -(((-value) + half) >> shift);
}

static void vclock_lane_ema_reset(vclock_lane_t& lane) {
  lane.ema_initialized = false;
  lane.ema_interval_valid = false;
  lane.ema_last_observed_dwt = 0;
  lane.ema_last_emitted_dwt = 0;
  lane.ema_last_observed_interval_cycles = 0;
  lane.ema_interval_cycles = 0;
  lane.ema_last_predicted_dwt = 0;
  lane.ema_last_interval_prediction_cycles = 0;
  lane.ema_last_effective_interval_cycles = 0;
  lane.ema_last_error_cycles = 0;
  lane.ema_last_interval_residual_cycles = 0;
  lane.ema_max_abs_error_cycles = 0;
  lane.ema_update_count = 0;
  lane.ema_gate_accept_count = 0;
  lane.ema_gate_reject_count = 0;
  lane.ema_last_interval_gate_valid = false;
  lane.ema_last_interval_accepted = false;
  lane.ema_last_interval_rejected = false;
  lane.ema_last_interval_ema_updated = false;
  lane.ema_last_interval_resync_applied = false;
  lane.ema_gate_reject_streak = 0;
  lane.ema_gate_resync_count = 0;
  lane.ema_last_rejected_observed_interval_cycles = 0;
  g_vclock_repair_stats = vclock_repair_stats_t{};
}

static void dwt_repair_diag_set_interval(dwt_repair_diag_t* out,
                                         bool gate_valid,
                                         bool accepted,
                                         bool rejected,
                                         bool ema_updated,
                                         uint32_t observed_cycles,
                                         uint32_t prediction_cycles,
                                         uint32_t effective_cycles,
                                         int32_t residual_cycles,
                                         uint32_t threshold_cycles,
                                         uint32_t accept_count,
                                         uint32_t reject_count,
                                         bool resync_applied,
                                         uint32_t resync_count,
                                         uint32_t reject_streak) {
  if (!out) return;
  out->interval_gate_valid = gate_valid;
  out->interval_sample_accepted = accepted;
  out->interval_sample_rejected = rejected;
  out->interval_ema_updated = ema_updated;
  out->interval_observed_cycles = observed_cycles;
  out->interval_prediction_cycles = prediction_cycles;
  out->interval_effective_cycles = effective_cycles;
  out->interval_residual_cycles = residual_cycles;
  out->interval_gate_threshold_cycles = threshold_cycles;
  out->interval_accept_count = accept_count;
  out->interval_reject_count = reject_count;
  out->interval_resync_applied = resync_applied;
  out->interval_resync_count = resync_count;
  out->interval_reject_streak = reject_streak;
}

static uint32_t vclock_lane_ema_predict_dwt(vclock_lane_t& lane,
                                            uint32_t observed_dwt,
                                            bool* out_synthetic,
                                            int32_t* out_error_cycles,
                                            dwt_repair_diag_t* out_diag = nullptr) {
  if (out_synthetic) *out_synthetic = false;
  if (out_error_cycles) *out_error_cycles = 0;
  if (out_diag) {
    out_diag->interval_gate_threshold_cycles =
        VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
  }

  if (!VCLOCK_EMA_DWT_AUTHORITY_ENABLED) {
    lane.ema_last_observed_dwt = observed_dwt;
    lane.ema_last_emitted_dwt = observed_dwt;
    lane.ema_last_predicted_dwt = observed_dwt;
    dwt_repair_diag_set_interval(out_diag, false, false, false, false,
                                 0, 0, 0, 0,
                                 VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                                 lane.ema_gate_accept_count,
                                 lane.ema_gate_reject_count,
                                 false,
                                 lane.ema_gate_resync_count,
                                 lane.ema_gate_reject_streak);
    return observed_dwt;
  }

  if (!lane.ema_initialized) {
    lane.ema_initialized = true;
    lane.ema_last_observed_dwt = observed_dwt;
    lane.ema_last_emitted_dwt = observed_dwt;
    lane.ema_last_predicted_dwt = observed_dwt;
    lane.ema_last_observed_interval_cycles = 0;
    lane.ema_last_interval_prediction_cycles = 0;
    lane.ema_last_effective_interval_cycles = 0;
    lane.ema_last_error_cycles = 0;
    lane.ema_last_interval_residual_cycles = 0;
    lane.ema_last_interval_gate_valid = false;
    lane.ema_last_interval_accepted = false;
    lane.ema_last_interval_rejected = false;
    lane.ema_last_interval_ema_updated = false;
    lane.ema_last_interval_resync_applied = false;
    lane.ema_gate_reject_streak = 0;
    lane.ema_last_rejected_observed_interval_cycles = 0;
    lane.ema_update_count++;

    g_vclock_repair_stats.last_original_dwt = observed_dwt;
    g_vclock_repair_stats.last_predicted_dwt = observed_dwt;
    g_vclock_repair_stats.last_used_dwt = observed_dwt;
    g_vclock_repair_stats.last_error_cycles = 0;
    g_vclock_repair_stats.last_candidate = false;
    g_vclock_repair_stats.last_synthetic = false;
    dwt_repair_diag_set_interval(out_diag, false, false, false, false,
                                 0, 0, 0, 0,
                                 VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                                 lane.ema_gate_accept_count,
                                 lane.ema_gate_reject_count,
                                 false,
                                 lane.ema_gate_resync_count,
                                 lane.ema_gate_reject_streak);
    return observed_dwt;
  }

  const uint32_t observed_interval = observed_dwt - lane.ema_last_observed_dwt;
  const bool have_prediction = lane.ema_interval_valid &&
                               lane.ema_interval_cycles != 0;
  const uint32_t prediction_interval = have_prediction
      ? lane.ema_interval_cycles
      : 0U;
  int32_t interval_residual = 0;
  bool gate_valid = false;
  bool accepted = false;
  bool rejected = false;
  bool ema_updated = false;

  lane.ema_last_observed_interval_cycles = observed_interval;
  bool resync_applied = false;
  uint32_t effective_interval = have_prediction ? prediction_interval : observed_interval;

  if (!have_prediction) {
    // First completed interval seeds the ruler.  There is no lane-local prior
    // interval yet, so no threshold decision is meaningful on this sample.
    lane.ema_interval_cycles = observed_interval;
    lane.ema_interval_valid = (observed_interval != 0);
    accepted = lane.ema_interval_valid;
    ema_updated = lane.ema_interval_valid;
    if (accepted) {
      lane.ema_gate_accept_count++;
      lane.ema_gate_reject_streak = 0;
      lane.ema_last_rejected_observed_interval_cycles = 0;
    }
    effective_interval = lane.ema_interval_cycles;
  } else {
    gate_valid = true;
    interval_residual = (int32_t)((int64_t)observed_interval -
                                  (int64_t)prediction_interval);
    const uint32_t abs_residual = dwt_ema_abs_i32(interval_residual);
    if (abs_residual > VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES) {
      rejected = true;
      lane.ema_gate_reject_count++;

      const bool coherent_reject =
          lane.ema_gate_reject_streak > 0 &&
          dwt_ema_abs_i32((int32_t)((int64_t)observed_interval -
                                    (int64_t)lane.ema_last_rejected_observed_interval_cycles)) <=
              VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
      lane.ema_gate_reject_streak++;
      lane.ema_last_rejected_observed_interval_cycles = observed_interval;

      // Still emit this row using the old effective ruler.  If multiple raw
      // intervals in a row agree with each other while disagreeing with the
      // current predictor, the predictor was probably born from a bad seed or
      // has fallen behind a real state change.  Re-seed only for the NEXT row.
      if (coherent_reject) {
        lane.ema_interval_cycles = observed_interval;
        lane.ema_interval_valid = (observed_interval != 0);
        lane.ema_gate_resync_count++;
        resync_applied = true;
      }
    } else {
      accepted = true;
      ema_updated = true;
      lane.ema_interval_cycles = (uint32_t)((int64_t)lane.ema_interval_cycles +
          (int64_t)dwt_ema_round_shift(interval_residual, VCLOCK_EMA_SHIFT));
      lane.ema_gate_accept_count++;
      lane.ema_gate_reject_streak = 0;
      lane.ema_last_rejected_observed_interval_cycles = 0;
      effective_interval = lane.ema_interval_cycles;
    }
  }

  const uint32_t previous_emitted_dwt = lane.ema_last_emitted_dwt;
  const uint32_t predicted = previous_emitted_dwt + effective_interval;
  const int32_t prediction_error = (int32_t)(observed_dwt - predicted);
  const uint32_t abs_error = dwt_ema_abs_i32(prediction_error);

  lane.ema_last_error_cycles = prediction_error;
  lane.ema_last_interval_residual_cycles = interval_residual;
  lane.ema_last_interval_prediction_cycles = prediction_interval;
  lane.ema_last_effective_interval_cycles = effective_interval;
  lane.ema_last_interval_gate_valid = gate_valid;
  lane.ema_last_interval_accepted = accepted;
  lane.ema_last_interval_rejected = rejected;
  lane.ema_last_interval_ema_updated = ema_updated;
  lane.ema_last_interval_resync_applied = resync_applied;
  if (abs_error > lane.ema_max_abs_error_cycles) {
    lane.ema_max_abs_error_cycles = abs_error;
  }

  // Always advance the observed endpoint.  A single late ISR endpoint poisons
  // the raw interval on both sides of that endpoint; updating the raw endpoint
  // lets the following recovery interval be rejected once, then normal service
  // resumes without permanently locking the EMA to a stale observation.
  lane.ema_last_observed_dwt = observed_dwt;
  lane.ema_last_emitted_dwt = predicted;
  lane.ema_last_predicted_dwt = predicted;
  lane.ema_update_count++;

  const bool synthetic = (predicted != observed_dwt);
  if (synthetic) {
    g_vclock_repair_stats.candidate_count++;
    g_vclock_repair_stats.applied_count++;
    g_vclock_repair_stats.consecutive_candidate_count++;
  } else {
    g_vclock_repair_stats.consecutive_candidate_count = 0;
  }
  g_vclock_repair_stats.last_original_dwt = observed_dwt;
  g_vclock_repair_stats.last_predicted_dwt = predicted;
  g_vclock_repair_stats.last_used_dwt = predicted;
  g_vclock_repair_stats.last_error_cycles = prediction_error;
  if (abs_error > g_vclock_repair_stats.max_abs_error_cycles) {
    g_vclock_repair_stats.max_abs_error_cycles = abs_error;
  }
  g_vclock_repair_stats.last_candidate = synthetic;
  g_vclock_repair_stats.last_synthetic = synthetic;

  if (out_synthetic) *out_synthetic = synthetic;
  if (out_error_cycles) *out_error_cycles = prediction_error;
  dwt_repair_diag_set_interval(out_diag, gate_valid, accepted, rejected,
                               ema_updated, observed_interval,
                               prediction_interval, effective_interval,
                               interval_residual,
                               VCLOCK_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                               lane.ema_gate_accept_count,
                               lane.ema_gate_reject_count,
                               resync_applied,
                               lane.ema_gate_resync_count,
                               lane.ema_gate_reject_streak);
  return predicted;
}

struct synthetic_clock32_t;
struct ocxo_runtime_context_t;

struct ocxo_lane_t {
  const char* name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;  // legacy alias for active compare channel
  uint8_t      counter_channel = 0;  // passive counter channel
  uint8_t      compare_channel = 0;  // active compare channel
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

  // Lane-local compare custody.  In steady state this is a one-second edge
  // target; during SmartZero it temporarily becomes a +10,000-tick acquisition
  // target.  The compare target is a process_interrupt-authored synthetic
  // counter32 identity; the low 16 bits are merely the hardware projection
  // used to arm the local QTimer channel.
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

  // Once-per-second OCXO witness surface.  In steady state this is the local
  // one-second compare target itself.  During SmartZero this surface remains
  // quiet until acquisition re-authors the normal one-second grid.
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
  uint32_t ema_last_interval_prediction_cycles = 0;
  uint32_t ema_last_effective_interval_cycles = 0;
  int32_t  ema_last_error_cycles = 0;
  int32_t  ema_last_interval_residual_cycles = 0;
  uint32_t ema_max_abs_error_cycles = 0;
  uint32_t ema_update_count = 0;
  uint32_t ema_gate_accept_count = 0;
  uint32_t ema_gate_reject_count = 0;
  bool     ema_last_interval_gate_valid = false;
  bool     ema_last_interval_accepted = false;
  bool     ema_last_interval_rejected = false;
  bool     ema_last_interval_ema_updated = false;
  bool     ema_last_interval_resync_applied = false;
  uint32_t ema_gate_reject_streak = 0;
  uint32_t ema_gate_resync_count = 0;
  uint32_t ema_last_rejected_observed_interval_cycles = 0;

  // OCXO event-lineage diagnostics.  These are separate from the DWT EMA
  // amplitude gate: they answer whether this one-second sample was actually
  // adjacent to the previous published OCXO target identity.
  bool     ema_last_adjacency_gate_valid = false;
  bool     ema_last_adjacency_ok = false;
  bool     ema_last_adjacency_rejected = false;
  uint32_t ema_last_counter_delta_ticks = 0;
  uint32_t ema_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
  uint32_t ema_adjacency_reject_count = 0;

  // PPS-Yardstick inference rail (Stage 1 -- observational, not authority).
  // The chain is the inferred OCXO DWT ledger: interval and free-running
  // endpoint carried in Q16 fixed-point cycles (endpoint modulo 2^48 Q16 ==
  // 2^32 DWT cycles).  This rail keeps its own previous-observed baseline so
  // it never depends on EMA internals.
  bool     yardstick_chain_valid = false;
  uint64_t yardstick_chain_endpoint_q16 = 0;
  uint64_t yardstick_chain_interval_q16 = 0;
  bool     yardstick_last_used_pps_sequence_valid = false;
  uint32_t yardstick_last_used_pps_sequence = 0;
  bool     yardstick_prev_observed_valid = false;
  uint32_t yardstick_prev_observed_dwt = 0;

  uint32_t yardstick_update_count = 0;
  uint32_t yardstick_seed_count = 0;
  uint32_t yardstick_stale_hold_count = 0;
  uint32_t yardstick_gate_agree_count = 0;
  uint32_t yardstick_gate_excursion_count = 0;
  uint32_t yardstick_adjacency_reseed_count = 0;
  uint32_t yardstick_coherent_reseed_count = 0;
  uint32_t yardstick_excursion_streak = 0;
  uint32_t yardstick_last_excursion_observed_interval_cycles = 0;

  bool     yardstick_last_valid = false;
  bool     yardstick_last_stale = false;
  bool     yardstick_last_excursion = false;
  uint32_t yardstick_last_g_now_cycles = 0;
  uint32_t yardstick_last_g_prev_cycles = 0;
  uint32_t yardstick_last_pps_seq_delta = 0;
  uint32_t yardstick_last_inferred_interval_cycles = 0;
  uint32_t yardstick_last_observed_interval_cycles = 0;
  int32_t  yardstick_last_inferred_minus_observed_cycles = 0;
  int32_t  yardstick_last_endpoint_minus_observed_cycles = 0;

  // Steady-state audit accumulators (agree seconds only; excursion seconds
  // are by definition observed-side corruption and would poison the
  // seed-bias / chain-walk evidence these exist to collect).
  int32_t  yardstick_endpoint_minus_observed_min_cycles = 0;
  int32_t  yardstick_endpoint_minus_observed_max_cycles = 0;
  uint32_t yardstick_max_abs_inferred_minus_observed_cycles = 0;
  int64_t  yardstick_sum_inferred_minus_observed_cycles = 0;

  // Stage 2 anchored authority rail (publishes dwt_at_event when
  // OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED).  Separate Q16 state from the free
  // chain above: the free chain stays a pure physics audit; the anchored
  // rail is the PI-corrected publication ledger.
  bool     yardstick_auth_valid = false;
  uint64_t yardstick_auth_endpoint_q16 = 0;
  uint64_t yardstick_auth_interval_q16 = 0;
  uint32_t yardstick_auth_publish_count = 0;
  uint32_t yardstick_auth_agree_publish_count = 0;
  uint32_t yardstick_auth_excursion_publish_count = 0;
  uint32_t yardstick_auth_stale_publish_count = 0;
  uint32_t yardstick_auth_observed_fallback_count = 0;
  uint32_t yardstick_auth_anchor_correction_count = 0;
  int32_t  yardstick_auth_last_error_cycles = 0;
  uint32_t yardstick_auth_last_published_dwt = 0;
  uint32_t yardstick_auth_fast_lock_remaining_seconds = 0;

  // SmartZero Gen-2 zero-worthiness (Stage A, observational).  Ring of the
  // last ZERO_WORTHY_STREAK_SECONDS clean-second anchor loop errors; any
  // unclean second empties the ring.  zw_worthy requires a full ring with a
  // bounded window sum.
  int32_t  zw_error_ring[ZERO_WORTHY_STREAK_SECONDS] = {};
  uint8_t  zw_ring_index = 0;
  uint8_t  zw_ring_count = 0;
  int32_t  zw_window_error_sum = 0;
  uint32_t zw_window_max_abs_error = 0;
  uint32_t zw_clean_streak_seconds = 0;
  bool     zw_worthy = false;
  uint32_t zw_worthy_streak_seconds = 0;
  uint32_t zw_seconds_evaluated = 0;
  uint32_t zw_worthy_second_count = 0;
  uint32_t zw_transition_count = 0;
  uint32_t zw_first_worthy_after_seconds = 0;  // 0 = never yet worthy
  const char* zw_last_disqualify_reason = "zw_never_evaluated";
  uint32_t zw_disqualify_rail_invalid_count = 0;
  uint32_t zw_disqualify_excursion_count = 0;
  uint32_t zw_disqualify_stale_count = 0;
  uint32_t zw_disqualify_seed_count = 0;
  uint32_t zw_disqualify_adjacency_count = 0;
  uint32_t zw_disqualify_no_interval_count = 0;
  uint32_t zw_disqualify_error_magnitude_count = 0;
  uint32_t zw_disqualify_error_sum_count = 0;
  uint32_t zw_disqualify_streak_building_count = 0;
  uint32_t zw_disqualify_fast_lock_count = 0;
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
static void ocxo_lane_yardstick_reset(ocxo_lane_t& lane);
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
  interrupt_lane_t::QTIMER2_CH1_COMP,
  "OCXO1",
  "ocxo1",
  "QTIMER2_CH1_ONE_SECOND_EDGE_COMPARE_CH0_COUNTER",
  "QTIMER2_CH0_LOCAL_SYNTHETIC_COUNTER32",
  "QTIMER2_CH1_EMA_PREDICTED_DWT",
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
  "QTIMER3_CH3_ONE_SECOND_EDGE_COMPARE_PHYSICAL_ADAPTER",
  "QTIMER3_CH3_LOCAL_SYNTHETIC_COUNTER32_PIN15_ADAPTER",
  "QTIMER3_CH3_EMA_PREDICTED_DWT",
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
// runs from the QTimer1 CH1 ISR path and merely refreshes process_interrupt's
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
static void vclock_ch2_one_second_service(uint32_t isr_entry_dwt_raw,
                                          uint32_t qtimer_event_dwt,
                                          uint32_t service_counter32,
                                          const spinidle_isr_capture_t& spinidle);
static void vclock_ch2_smartzero_reset_attempt_state(void);
static void vclock_ch2_smartzero_deactivate(void);
static void vclock_ch2_smartzero_service(uint32_t qtimer_event_dwt,
                                         uint32_t service_counter32);

static void interrupt_handoff_configure(void);
static void interrupt_handoff_request_from_capture_isr(const char* context);
static void interrupt_handoff_service_isr(void);

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
    vclock_lane_ema_reset(g_vclock_lane);
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
    vclock_lane_ema_reset(g_vclock_lane);
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
  return OCXO1_DISABLED ? 0U : IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR;
}
uint16_t interrupt_qtimer3_ch0_counter_now(void) {
  return OCXO2_DISABLED ? 0U : IMXRT_TMR3.CH[QTIMER_CLOCK_COUNTER_CH].CNTR;
}
uint16_t interrupt_qtimer3_ch3_counter_now(void) {
  // OCXO2 physical adapter: pin 15 is QTimer3 TIMER3.
  return OCXO2_DISABLED ? 0U : IMXRT_TMR3.CH[QTIMER3_OCXO2_PHYSICAL_CH].CNTR;
}

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
    const uint16_t ocxo1_hw16 = IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR;
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
    const uint16_t ocxo2_hw16 = IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CNTR;
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
    const uint16_t hw = IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR;
    out->hardware16 = hw;
    out->counter32 = project_counter32_from_hw16(g_ocxo1_clock32, hw);
    out->ns64 = project_ns64_from_hw16(g_ocxo1_clock32, hw);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    if (OCXO2_DISABLED) return false;
    const uint16_t hw = IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CNTR;
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

  // SmartZero temporarily rides the same OCXO compare ISR with +10,000-tick
  // acquisition samples.  The compare target is the authored sample identity;
  // service-time low-word reads remain diagnostics only.
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

// OCXO SmartZero samples are served by the same local OCXO compare ISR used
// for steady-state one-second edge capture.  There is intentionally no
// separate SmartZero ISR path here; SmartZero temporarily changes the next
// authored target spacing to +10,000 ticks and feeds smartzero_feed_sample()
// with that authored compare-target identity.

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

  // SmartZero may temporarily stop/re-author the OCXO local compare path
  // while acquiring lane proofs.  Aborting the *live acquisition attempt* must
  // not silently disable active clock service.  If an OCXO lane is active,
  // restore its normal one-second edge compare from the currently installed
  // grid; if it is inactive, leave it stopped as before.
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

static inline void qtimer_clear_compare_flag(IMXRT_TMR_t& module, uint8_t ch) {
  module.CH[ch].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)module.CH[ch].CSCTRL;
}

static inline void qtimer_program_compare(IMXRT_TMR_t& module,
                                          uint8_t ch,
                                          uint16_t target_low16) {
  qtimer_clear_compare_flag(module, ch);
  module.CH[ch].COMP1  = target_low16;
  module.CH[ch].CMPLD1 = target_low16;
  qtimer_clear_compare_flag(module, ch);
  module.CH[ch].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer_disable_compare(IMXRT_TMR_t& module, uint8_t ch) {
  module.CH[ch].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer_clear_compare_flag(module, ch);
  qtimer_clear_compare_flag(module, ch);
}

// Legacy QTimer1 CH1 hosted-rail API is deliberately disabled while the
// uniform clock-lane experiment is active.  QTimer1 CH1 is now the sole VCLOCK
// compare rail used by TimePop through the historical CH2 API.
static inline void qtimer1_ch1_clear_compare_flag(void) {
  qtimer_clear_compare_flag(IMXRT_TMR1, QTIMER1_AUX_COMPARE_CH);
}

static inline void qtimer1_ch1_program_compare(uint16_t) {
  // Disabled: do not program QTimer1 CH2 or the uniform CH1 rail.
}

static inline void qtimer1_ch1_disable_compare_hw(void) {
  qtimer_disable_compare(IMXRT_TMR1, QTIMER1_AUX_COMPARE_CH);
}

// Legacy QTimer1 CH2 TimePop API now drives the uniform CH1 compare channel.
// The name remains so process_timepop does not need to be part of this patch.
static inline void qtimer1_ch2_clear_compare_flag(void) {
  qtimer_clear_compare_flag(IMXRT_TMR1, QTIMER_CLOCK_COMPARE_CH);
}

static inline void qtimer1_ch2_program_compare(uint16_t target_low16) {
  qtimer_program_compare(IMXRT_TMR1, QTIMER_CLOCK_COMPARE_CH, target_low16);
}

static inline void ocxo_compare_clear_flag(const ocxo_lane_t& lane) {
  qtimer_clear_compare_flag(*lane.module, lane.compare_channel);
}

static inline void ocxo_compare_program(const ocxo_lane_t& lane,
                                        uint16_t target_low16) {
  qtimer_program_compare(*lane.module, lane.compare_channel, target_low16);
}

static inline void ocxo_compare_disable(const ocxo_lane_t& lane) {
  qtimer_disable_compare(*lane.module, lane.compare_channel);
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

  int32_t observed_dwt_rel[REGRESSION_RETAINED_SAMPLE_COUNT]{};
  cadence_regression_result_t last_result{};
};

static cadence_regression_lane_t g_regression_vclock = {};
static cadence_regression_lane_t g_regression_ocxo1 = {};
static cadence_regression_lane_t g_regression_ocxo2 = {};

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
      diag.ocxo_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
      diag.ocxo_counter_adjacency_valid = lane->ema_last_adjacency_gate_valid;
      diag.ocxo_counter_adjacency_ok = lane->ema_last_adjacency_ok;
      diag.ocxo_counter_adjacency_rejected = lane->ema_last_adjacency_rejected;
      diag.ocxo_counter_adjacency_reject_count = lane->ema_adjacency_reject_count;
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
                                  const cadence_regression_result_t* regression = nullptr,
                                  const spinidle_isr_capture_t* spinidle = nullptr) {
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
  if (spinidle) {
    spinidle_copy_to_diag(diag, *spinidle);
  }
  if (repair) {
    diag.dwt_synthetic = repair->synthetic;
    diag.dwt_repair_candidate = repair->candidate;
    diag.dwt_original_at_event = repair->original_dwt;
    diag.dwt_predicted_at_event = repair->predicted_dwt;
    diag.dwt_used_at_event = repair->used_dwt;
    diag.dwt_isr_entry_raw = repair->isr_entry_dwt_raw;
    diag.dwt_event_from_isr_entry_raw = repair->event_dwt_from_isr_entry_raw;
    diag.dwt_isr_entry_to_event_correction_cycles =
        repair->isr_entry_to_event_correction_cycles;
    diag.dwt_published_minus_event_cycles =
        (int32_t)(event.dwt_at_event - repair->event_dwt_from_isr_entry_raw);
    diag.dwt_used_minus_event_cycles =
        (int32_t)(repair->used_dwt - repair->event_dwt_from_isr_entry_raw);
    diag.dwt_synthetic_error_cycles = repair->error_cycles;
    diag.dwt_synthetic_threshold_cycles = repair->threshold_cycles;
    diag.dwt_synthetic_reason = repair->reason;
    diag.dwt_interval_gate_valid = repair->interval_gate_valid;
    diag.dwt_interval_sample_accepted = repair->interval_sample_accepted;
    diag.dwt_interval_sample_rejected = repair->interval_sample_rejected;
    diag.dwt_interval_ema_updated = repair->interval_ema_updated;
    diag.dwt_interval_observed_cycles = repair->interval_observed_cycles;
    diag.dwt_interval_prediction_cycles = repair->interval_prediction_cycles;
    diag.dwt_interval_effective_cycles = repair->interval_effective_cycles;
    diag.dwt_interval_residual_cycles = repair->interval_residual_cycles;
    diag.dwt_interval_gate_threshold_cycles = repair->interval_gate_threshold_cycles;
    diag.dwt_interval_accept_count = repair->interval_accept_count;
    diag.dwt_interval_reject_count = repair->interval_reject_count;
    diag.dwt_interval_resync_applied = repair->interval_resync_applied;
    diag.dwt_interval_resync_count = repair->interval_resync_count;
    diag.dwt_interval_reject_streak = repair->interval_reject_streak;
    diag.dwt_interval_adjacency_gate_valid = repair->interval_adjacency_gate_valid;
    diag.dwt_interval_adjacency_ok = repair->interval_adjacency_ok;
    diag.dwt_interval_adjacency_rejected = repair->interval_adjacency_rejected;
    diag.dwt_interval_counter_delta_ticks = repair->interval_counter_delta_ticks;
    diag.dwt_interval_expected_counter_delta_ticks =
        repair->interval_expected_counter_delta_ticks;
    diag.dwt_interval_adjacency_reject_count =
        repair->interval_adjacency_reject_count;
    diag.dwt_yardstick_valid = repair->yardstick_valid;
    diag.dwt_yardstick_stale = repair->yardstick_stale;
    diag.dwt_yardstick_seeded = repair->yardstick_seeded_this_event;
    diag.dwt_yardstick_excursion = repair->yardstick_excursion;
    diag.dwt_yardstick_pps_sequence = repair->yardstick_pps_sequence;
    diag.dwt_yardstick_pps_seq_delta = repair->yardstick_pps_seq_delta;
    diag.dwt_yardstick_g_now_cycles = repair->yardstick_g_now_cycles;
    diag.dwt_yardstick_g_prev_cycles = repair->yardstick_g_prev_cycles;
    diag.dwt_yardstick_inferred_interval_cycles =
        repair->yardstick_inferred_interval_cycles;
    diag.dwt_yardstick_observed_interval_cycles =
        repair->yardstick_observed_interval_cycles;
    diag.dwt_yardstick_inferred_minus_observed_cycles =
        repair->yardstick_inferred_minus_observed_cycles;
    diag.dwt_yardstick_inferred_endpoint_dwt =
        repair->yardstick_inferred_endpoint_dwt;
    diag.dwt_yardstick_inferred_endpoint_frac_q16 =
        repair->yardstick_inferred_endpoint_frac_q16;
    diag.dwt_yardstick_endpoint_minus_observed_cycles =
        repair->yardstick_endpoint_minus_observed_cycles;
    diag.dwt_yardstick_gate_threshold_cycles =
        repair->yardstick_gate_threshold_cycles;
    diag.dwt_yardstick_gate_agree_count = repair->yardstick_gate_agree_count;
    diag.dwt_yardstick_gate_excursion_count =
        repair->yardstick_gate_excursion_count;
    diag.dwt_yardstick_authority = repair->yardstick_authority;
    diag.dwt_ema_dwt_at_event = repair->ema_dwt_at_event;
    diag.dwt_yardstick_auth_endpoint_dwt =
        repair->yardstick_auth_endpoint_dwt;
    diag.dwt_yardstick_auth_endpoint_frac_q16 =
        repair->yardstick_auth_endpoint_frac_q16;
    diag.dwt_yardstick_auth_error_cycles =
        repair->yardstick_auth_error_cycles;
    diag.dwt_yardstick_auth_anchor_applied =
        repair->yardstick_auth_anchor_applied;
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

static constexpr uint32_t VCLOCK_PERISHABLE_FACT_RING_SIZE = 8;
static constexpr const char* VCLOCK_FACT_DRAIN_NAME = "VCLOCK_FACT_DRAIN";

struct vclock_perishable_fact_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t observed_dwt_at_event = 0;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t observed_low16 = 0;
  bool one_second_due = false;
  bool witness_pps_valid = false;
  pps_t witness_pps{};
  uint32_t fallback_sequence = 0;
  spinidle_isr_capture_t spinidle{};
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

static void vclock_ch2_one_second_service(uint32_t isr_entry_dwt_raw,
                                          uint32_t qtimer_event_dwt,
                                          uint32_t service_counter32,
                                          const spinidle_isr_capture_t& spinidle) {
  if (!g_vclock_ch2_one_second_enabled) return;
  if (!g_vclock_lane.active || !g_rt_vclock || !g_rt_vclock->active) return;

  const uint32_t target_counter32 = g_vclock_ch2_one_second_next_counter32;
  if ((int32_t)(service_counter32 - target_counter32) < 0) return;

  isr_raw_dwt_probe_record(interrupt_subscriber_kind_t::VCLOCK,
                           isr_entry_dwt_raw);

  const uint16_t target_low16 = vclock_hardware_low16_from_synthetic(target_counter32);
  spincatch_note_isr_entry(interrupt_subscriber_kind_t::VCLOCK,
                           target_counter32,
                           target_low16,
                           isr_entry_dwt_raw);

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
  fact.isr_entry_dwt_raw = isr_entry_dwt_raw;
  fact.observed_dwt_at_event = authored_dwt;
  fact.target_counter32 = target_counter32;
  fact.target_low16 = target_low16;
  fact.observed_low16 = (uint16_t)(service_counter32 & 0xFFFFU);
  fact.one_second_due = true;
  fact.witness_pps_valid = g_last_pps_witness_valid;
  fact.witness_pps = witness_pps;
  fact.fallback_sequence = g_pps_gpio_heartbeat.edge_count + 1U;
  fact.spinidle = spinidle;

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

  const uint32_t observed_dwt = fact.observed_dwt_at_event;
  bool ema_synthetic = false;
  int32_t ema_error_cycles = 0;
  dwt_repair_diag_t ema_diag{};
  ema_diag.valid = true;
  ema_diag.original_dwt = observed_dwt;
  ema_diag.isr_entry_dwt_raw = fact.isr_entry_dwt_raw;
  ema_diag.event_dwt_from_isr_entry_raw = observed_dwt;
  ema_diag.isr_entry_to_event_correction_cycles =
      (int32_t)(observed_dwt - fact.isr_entry_dwt_raw);
  ema_diag.threshold_cycles = VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES;
  const uint32_t published_dwt = vclock_lane_ema_predict_dwt(
      g_vclock_lane, observed_dwt, &ema_synthetic, &ema_error_cycles, &ema_diag);

  uint32_t coincidence_cycles = 0;
  bool coincidence_valid = false;
  if (fact.witness_pps_valid && fact.witness_pps.sequence > 0) {
    const int32_t cycles_since_pps =
        (int32_t)(published_dwt - fact.witness_pps.dwt_at_edge);
    if (llabs((long long)cycles_since_pps) <
        DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
      coincidence_cycles = (uint32_t)(cycles_since_pps >= 0
          ? cycles_since_pps
          : -cycles_since_pps);
      coincidence_valid = true;
    }
  }

  ema_diag.candidate = ema_synthetic;
  ema_diag.synthetic = ema_synthetic;
  ema_diag.predicted_dwt = published_dwt;
  ema_diag.used_dwt = published_dwt;
  ema_diag.error_cycles = ema_error_cycles;
  ema_diag.reason = ema_diag.interval_sample_rejected
      ? "vclock_ema_gate_reject"
      : (ema_synthetic ? "vclock_ema" : "vclock_ema_init");

  const pps_t witness_pps = fact.witness_pps_valid ? fact.witness_pps : pps_t{};
  publish_vclock_domain_pps_vclock(witness_pps,
                                    (witness_pps.sequence != 0)
                                        ? witness_pps.sequence
                                        : fact.fallback_sequence,
                                    published_dwt,
                                    fact.target_counter32,
                                    fact.target_low16);

  emit_one_second_event(*g_rt_vclock,
                        published_dwt,
                        fact.target_counter32,
                        coincidence_cycles,
                        coincidence_valid,
                        &ema_diag,
                        true,
                        nullptr,
                        &fact.spinidle);

  pps_edge_dispatch_arm_or_call_from_foreground("PPS_VCLOCK_DISPATCH");
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

static void ocxo_lane_program_local_cadence_compare(ocxo_lane_t& lane,
                                                    uint32_t target_counter32,
                                                    uint32_t reason) {
  const uint16_t target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);

  lane.cadence_last_program_csctrl_before = lane.module->CH[lane.compare_channel].CSCTRL;
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

  lane.cadence_last_program_csctrl_after = lane.module->CH[lane.compare_channel].CSCTRL;
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
  lane.ema_last_interval_prediction_cycles = 0;
  lane.ema_last_effective_interval_cycles = 0;
  lane.ema_last_error_cycles = 0;
  lane.ema_last_interval_residual_cycles = 0;
  lane.ema_max_abs_error_cycles = 0;
  lane.ema_update_count = 0;
  lane.ema_gate_accept_count = 0;
  lane.ema_gate_reject_count = 0;
  lane.ema_last_interval_gate_valid = false;
  lane.ema_last_interval_accepted = false;
  lane.ema_last_interval_rejected = false;
  lane.ema_last_interval_ema_updated = false;
  lane.ema_last_interval_resync_applied = false;
  lane.ema_gate_reject_streak = 0;
  lane.ema_gate_resync_count = 0;
  lane.ema_last_rejected_observed_interval_cycles = 0;
  lane.ema_last_adjacency_gate_valid = false;
  lane.ema_last_adjacency_ok = false;
  lane.ema_last_adjacency_rejected = false;
  lane.ema_last_counter_delta_ticks = 0;
  lane.ema_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
  lane.ema_adjacency_reject_count = 0;
}

static void ocxo_lane_yardstick_reset(ocxo_lane_t& lane) {
  lane.yardstick_chain_valid = false;
  lane.yardstick_chain_endpoint_q16 = 0;
  lane.yardstick_chain_interval_q16 = 0;
  lane.yardstick_last_used_pps_sequence_valid = false;
  lane.yardstick_last_used_pps_sequence = 0;
  lane.yardstick_prev_observed_valid = false;
  lane.yardstick_prev_observed_dwt = 0;
  lane.yardstick_update_count = 0;
  lane.yardstick_seed_count = 0;
  lane.yardstick_stale_hold_count = 0;
  lane.yardstick_gate_agree_count = 0;
  lane.yardstick_gate_excursion_count = 0;
  lane.yardstick_adjacency_reseed_count = 0;
  lane.yardstick_coherent_reseed_count = 0;
  lane.yardstick_excursion_streak = 0;
  lane.yardstick_last_excursion_observed_interval_cycles = 0;
  lane.yardstick_last_valid = false;
  lane.yardstick_last_stale = false;
  lane.yardstick_last_excursion = false;
  lane.yardstick_last_g_now_cycles = 0;
  lane.yardstick_last_g_prev_cycles = 0;
  lane.yardstick_last_pps_seq_delta = 0;
  lane.yardstick_last_inferred_interval_cycles = 0;
  lane.yardstick_last_observed_interval_cycles = 0;
  lane.yardstick_last_inferred_minus_observed_cycles = 0;
  lane.yardstick_last_endpoint_minus_observed_cycles = 0;
  lane.yardstick_endpoint_minus_observed_min_cycles = 0;
  lane.yardstick_endpoint_minus_observed_max_cycles = 0;
  lane.yardstick_max_abs_inferred_minus_observed_cycles = 0;
  lane.yardstick_sum_inferred_minus_observed_cycles = 0;
  lane.yardstick_auth_valid = false;
  lane.yardstick_auth_endpoint_q16 = 0;
  lane.yardstick_auth_interval_q16 = 0;
  lane.yardstick_auth_publish_count = 0;
  lane.yardstick_auth_agree_publish_count = 0;
  lane.yardstick_auth_excursion_publish_count = 0;
  lane.yardstick_auth_stale_publish_count = 0;
  lane.yardstick_auth_observed_fallback_count = 0;
  lane.yardstick_auth_anchor_correction_count = 0;
  lane.yardstick_auth_last_error_cycles = 0;
  lane.yardstick_auth_last_published_dwt = 0;
  lane.yardstick_auth_fast_lock_remaining_seconds = 0;

  for (uint32_t i = 0; i < ZERO_WORTHY_STREAK_SECONDS; i++) {
    lane.zw_error_ring[i] = 0;
  }
  lane.zw_ring_index = 0;
  lane.zw_ring_count = 0;
  lane.zw_window_error_sum = 0;
  lane.zw_window_max_abs_error = 0;
  lane.zw_clean_streak_seconds = 0;
  lane.zw_worthy = false;
  lane.zw_worthy_streak_seconds = 0;
  lane.zw_seconds_evaluated = 0;
  lane.zw_worthy_second_count = 0;
  lane.zw_transition_count = 0;
  lane.zw_first_worthy_after_seconds = 0;
  lane.zw_last_disqualify_reason = "zw_reset";
  lane.zw_disqualify_rail_invalid_count = 0;
  lane.zw_disqualify_excursion_count = 0;
  lane.zw_disqualify_stale_count = 0;
  lane.zw_disqualify_seed_count = 0;
  lane.zw_disqualify_adjacency_count = 0;
  lane.zw_disqualify_no_interval_count = 0;
  lane.zw_disqualify_error_magnitude_count = 0;
  lane.zw_disqualify_error_sum_count = 0;
  lane.zw_disqualify_streak_building_count = 0;
  lane.zw_disqualify_fast_lock_count = 0;
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
  ocxo_lane_yardstick_reset(lane);
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
        bool ema_synthetic = false;
        int32_t ema_error_cycles = 0;
        dwt_repair_diag_t ema_diag{};
        ema_diag.valid = true;
        ema_diag.original_dwt = qtimer_event_dwt;
        ema_diag.isr_entry_dwt_raw = 0;
        ema_diag.event_dwt_from_isr_entry_raw = qtimer_event_dwt;
        ema_diag.isr_entry_to_event_correction_cycles = 0;
        ema_diag.threshold_cycles = VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES;
        const uint32_t published_dwt = vclock_lane_ema_predict_dwt(
            g_vclock_lane, qtimer_event_dwt, &ema_synthetic, &ema_error_cycles, &ema_diag);
        ema_diag.candidate = ema_synthetic;
        ema_diag.synthetic = ema_synthetic;
        ema_diag.predicted_dwt = published_dwt;
        ema_diag.used_dwt = published_dwt;
        ema_diag.error_cycles = ema_error_cycles;
        ema_diag.reason = ema_diag.interval_sample_rejected
            ? "vclock_ema_gate_reject_fallback"
            : (ema_synthetic ? "vclock_ema_fallback" : "vclock_ema_init_fallback");
        publish_vclock_domain_pps_vclock(witness_pps,
                                          (witness_pps.sequence != 0)
                                              ? witness_pps.sequence
                                              : (g_pps_gpio_heartbeat.edge_count + 1),
                                          published_dwt,
                                          cadence_counter32,
                                          fired_low16);

        emit_one_second_event(*g_rt_vclock, published_dwt,
                              cadence_counter32,
                              coincidence_cycles, coincidence_valid, &ema_diag);

        pps_edge_dispatch_arm_or_call_from_foreground("PPS_VCLOCK_DISPATCH");

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
// OCXO lanes — OCXO1 uniform CH0/CH1; OCXO2 pin15 physical adapter on QTimer3 CH3
// ============================================================================
//
static void ocxo_lane_program_compare(ocxo_lane_t& lane, uint16_t target_low16) {
  ocxo_compare_program(lane, target_low16);
}

static void ocxo_lane_disable_compare(ocxo_lane_t& lane) {
  ocxo_compare_disable(lane);
}

// ISR discipline for OCXO lanes is reentry-safe capture only:
//   1. capture first-instruction DWT,
//   2. snapshot only perishable hardware/state facts,
//   3. clear/defuse the compare flag,
//   4. enqueue the immutable capture packet,
//   5. pend the priority-16 handoff tier and exit.
//
// The handoff tier computes event-DWT, service offset, SmartZero state,
// synthetic identity updates, rearm/stop policy, and foreground fact enqueue.
// ============================================================================

// ----------------------------------------------------------------------------
// OCXO one-second fact ring — ISR capture, ASAP interpretation.
// ----------------------------------------------------------------------------
//
// The former build enqueued every 1 kHz OCXO cadence sample and asked
// foreground TimePop ASAP to drain/interpret them. That proved too expensive.
// Steady-state OCXO now enqueues only the local one-second edge compare.
// SmartZero may temporarily use +10,000-tick acquisition samples, but those
// samples do not publish as OCXO one-second events until SmartZero re-authors
// the normal one-second grid.
//
// The ring remains per-lane and loss-visible, with a normal publication rate
// of 1 Hz per active OCXO lane.

static constexpr uint32_t OCXO_PERISHABLE_FACT_RING_SIZE = 8;

struct interrupt_perishable_fact_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  uint32_t sequence = 0;

  // Captured as first ISR instruction.  This remains private custody evidence;
  // published event DWT remains latency-adjusted service_dwt_at_event.
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t service_dwt_at_event = 0;
  spinidle_isr_capture_t spinidle{};

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
  return (lane.module->CH[lane.compare_channel].CSCTRL & TMR_CSCTRL_TCF1) != 0;
}

static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.counter_channel].CNTR;
}

static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane) {
  ocxo_compare_clear_flag(lane);
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
                                          bool counter_adjacency_valid,
                                          uint32_t counter_delta_ticks,
                                          bool* out_synthetic,
                                          int32_t* out_error_cycles,
                                          dwt_repair_diag_t* out_diag = nullptr) {
  if (out_synthetic) *out_synthetic = false;
  if (out_error_cycles) *out_error_cycles = 0;

  const bool counter_adjacency_ok =
      !counter_adjacency_valid ||
      counter_delta_ticks == OCXO_WITNESS_ONE_SECOND_COUNTS;

  lane.ema_last_adjacency_gate_valid = counter_adjacency_valid;
  lane.ema_last_adjacency_ok = counter_adjacency_ok;
  lane.ema_last_adjacency_rejected = false;
  lane.ema_last_counter_delta_ticks = counter_adjacency_valid ? counter_delta_ticks : 0U;
  lane.ema_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;

  if (out_diag) {
    out_diag->interval_gate_threshold_cycles =
        OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
    out_diag->interval_adjacency_gate_valid = counter_adjacency_valid;
    out_diag->interval_adjacency_ok = counter_adjacency_ok;
    out_diag->interval_adjacency_rejected = false;
    out_diag->interval_counter_delta_ticks = counter_adjacency_valid ? counter_delta_ticks : 0U;
    out_diag->interval_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
    out_diag->interval_adjacency_reject_count = lane.ema_adjacency_reject_count;
  }

  if (!OCXO_EMA_DWT_AUTHORITY_ENABLED) {
    lane.ema_last_observed_dwt = observed_dwt;
    lane.ema_last_emitted_dwt = observed_dwt;
    lane.ema_last_predicted_dwt = observed_dwt;
    dwt_repair_diag_set_interval(out_diag, false, false, false, false,
                                 0, 0, 0, 0,
                                 OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                                 lane.ema_gate_accept_count,
                                 lane.ema_gate_reject_count,
                                 false,
                                 lane.ema_gate_resync_count,
                                 lane.ema_gate_reject_streak);
    return observed_dwt;
  }

  if (!lane.ema_initialized) {
    lane.ema_initialized = true;
    lane.ema_last_observed_dwt = observed_dwt;
    lane.ema_last_emitted_dwt = observed_dwt;
    lane.ema_last_predicted_dwt = observed_dwt;
    lane.ema_last_observed_interval_cycles = 0;
    lane.ema_last_interval_prediction_cycles = 0;
    lane.ema_last_effective_interval_cycles = 0;
    lane.ema_last_error_cycles = 0;
    lane.ema_last_interval_residual_cycles = 0;
    lane.ema_last_interval_gate_valid = false;
    lane.ema_last_interval_accepted = false;
    lane.ema_last_interval_rejected = false;
    lane.ema_last_interval_ema_updated = false;
    lane.ema_last_interval_resync_applied = false;
    lane.ema_gate_reject_streak = 0;
    lane.ema_last_rejected_observed_interval_cycles = 0;
    lane.ema_update_count++;
    dwt_repair_diag_set_interval(out_diag, false, false, false, false,
                                 0, 0, 0, 0,
                                 OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                                 lane.ema_gate_accept_count,
                                 lane.ema_gate_reject_count,
                                 false,
                                 lane.ema_gate_resync_count,
                                 lane.ema_gate_reject_streak);
    return observed_dwt;
  }

  const uint32_t observed_interval = observed_dwt - lane.ema_last_observed_dwt;
  const bool have_prediction = lane.ema_interval_valid &&
                               lane.ema_interval_cycles != 0;
  const uint32_t fallback_prediction_interval = interrupt_vclock_cycles_per_second();
  const bool have_projection_interval = have_prediction || fallback_prediction_interval != 0;
  const uint32_t prediction_interval = have_prediction
      ? lane.ema_interval_cycles
      : fallback_prediction_interval;
  int32_t interval_residual = 0;
  bool gate_valid = false;
  bool accepted = false;
  bool rejected = false;
  bool ema_updated = false;

  lane.ema_last_observed_interval_cycles = observed_interval;
  bool resync_applied = false;
  uint32_t effective_interval = have_prediction ? prediction_interval : observed_interval;

  if (counter_adjacency_valid && !counter_adjacency_ok) {
    // Mechanical lineage fault: this one-second sample is not adjacent to the
    // previous published OCXO target identity.  The DWT endpoint is still real
    // custody evidence, but the interval formed from it is not a lawful clock
    // interval.  Preserve the EMA ruler, publish a projected endpoint, and
    // advance the raw-observed baseline below so the next adjacent sample can
    // recover naturally.
    gate_valid = have_projection_interval;
    rejected = true;
    ema_updated = false;
    resync_applied = false;
    lane.ema_last_adjacency_rejected = true;
    lane.ema_adjacency_reject_count++;
    lane.ema_gate_reject_streak = 0;
    lane.ema_last_rejected_observed_interval_cycles = 0;
    effective_interval = have_projection_interval ? prediction_interval : observed_interval;
    interval_residual = have_projection_interval
        ? (int32_t)((int64_t)observed_interval - (int64_t)prediction_interval)
        : 0;
  } else if (!have_prediction) {
    lane.ema_interval_cycles = observed_interval;
    lane.ema_interval_valid = (observed_interval != 0);
    accepted = lane.ema_interval_valid;
    ema_updated = lane.ema_interval_valid;
    if (accepted) {
      lane.ema_gate_accept_count++;
      lane.ema_gate_reject_streak = 0;
      lane.ema_last_rejected_observed_interval_cycles = 0;
    }
    effective_interval = lane.ema_interval_cycles;
  } else {
    gate_valid = true;
    interval_residual = (int32_t)((int64_t)observed_interval -
                                  (int64_t)prediction_interval);
    const uint32_t abs_residual = ocxo_ema_abs_i32(interval_residual);
    if (abs_residual > OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES) {
      rejected = true;
      lane.ema_gate_reject_count++;

      const bool coherent_reject =
          lane.ema_gate_reject_streak > 0 &&
          ocxo_ema_abs_i32((int32_t)((int64_t)observed_interval -
                                     (int64_t)lane.ema_last_rejected_observed_interval_cycles)) <=
              OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
      lane.ema_gate_reject_streak++;
      lane.ema_last_rejected_observed_interval_cycles = observed_interval;

      // Still emit this row using the old effective ruler.  Coherent repeated
      // rejects are evidence that the predictor seed is wrong, not that the
      // clock stopped.  Re-seed only for the NEXT row.
      if (coherent_reject) {
        lane.ema_interval_cycles = observed_interval;
        lane.ema_interval_valid = (observed_interval != 0);
        lane.ema_gate_resync_count++;
        resync_applied = true;
      }
    } else {
      accepted = true;
      ema_updated = true;
      lane.ema_interval_cycles = (uint32_t)((int64_t)lane.ema_interval_cycles +
          (int64_t)ocxo_ema_round_shift(interval_residual, OCXO_EMA_SHIFT));
      lane.ema_gate_accept_count++;
      lane.ema_gate_reject_streak = 0;
      lane.ema_last_rejected_observed_interval_cycles = 0;
      effective_interval = lane.ema_interval_cycles;
    }
  }

  const uint32_t previous_emitted_dwt = lane.ema_last_emitted_dwt;
  const uint32_t predicted = previous_emitted_dwt + effective_interval;
  const int32_t prediction_error = (int32_t)(observed_dwt - predicted);
  const uint32_t abs_error = ocxo_ema_abs_i32(prediction_error);

  lane.ema_last_error_cycles = prediction_error;
  lane.ema_last_interval_residual_cycles = interval_residual;
  lane.ema_last_interval_prediction_cycles = prediction_interval;
  lane.ema_last_effective_interval_cycles = effective_interval;
  lane.ema_last_interval_gate_valid = gate_valid;
  lane.ema_last_interval_accepted = accepted;
  lane.ema_last_interval_rejected = rejected;
  lane.ema_last_interval_ema_updated = ema_updated;
  lane.ema_last_interval_resync_applied = resync_applied;
  if (abs_error > lane.ema_max_abs_error_cycles) {
    lane.ema_max_abs_error_cycles = abs_error;
  }

  lane.ema_last_observed_dwt = observed_dwt;
  lane.ema_last_emitted_dwt = predicted;
  lane.ema_last_predicted_dwt = predicted;
  lane.ema_update_count++;

  const bool synthetic = (predicted != observed_dwt);
  if (out_synthetic) *out_synthetic = synthetic;
  if (out_error_cycles) *out_error_cycles = prediction_error;
  dwt_repair_diag_set_interval(out_diag, gate_valid, accepted, rejected,
                               ema_updated, observed_interval,
                               prediction_interval, effective_interval,
                               interval_residual,
                               OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES,
                               lane.ema_gate_accept_count,
                               lane.ema_gate_reject_count,
                               resync_applied,
                               lane.ema_gate_resync_count,
                               lane.ema_gate_reject_streak);
  if (out_diag) {
    out_diag->interval_adjacency_gate_valid = counter_adjacency_valid;
    out_diag->interval_adjacency_ok = counter_adjacency_ok;
    out_diag->interval_adjacency_rejected = lane.ema_last_adjacency_rejected;
    out_diag->interval_counter_delta_ticks = counter_adjacency_valid ? counter_delta_ticks : 0U;
    out_diag->interval_expected_counter_delta_ticks = OCXO_WITNESS_ONE_SECOND_COUNTS;
    out_diag->interval_adjacency_reject_count = lane.ema_adjacency_reject_count;
  }
  return predicted;
}

// ============================================================================
// PPS-Yardstick OCXO DWT inference (Stage 1 -- OBSERVATIONAL ONLY)
// ============================================================================
//
// Parallel rail beside the OCXO EMA authority.  Nothing this function
// computes changes the published dwt_at_event in Stage 1; every result is
// recorded in lane state and dwt_repair_diag_t so live campaigns can
// adjudicate the Stage 2 authority flip.
//
// Per event:
//   1. Load the PPS yardstick pair (G_now, G_prev).  A non-advancing PPS
//      sequence, an unusable pair, or an insane step is STALE: the chain
//      interval holds (pure last-second prediction) and is flagged.
//   2. Scale the chain interval by the yardstick step:
//        O' = O * G_now / G_prev = O + (O * (G_now - G_prev)) / G_prev
//      computed exactly in 64-bit Q16 (the delta form avoids overflow; it is
//      the exact ratio, not a first-order approximation).
//   3. Advance the free-running chain endpoint by the inferred interval.
//   4. Validate against the QTimer-observed interval (4-cycle lattice):
//      agreement within OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES confirms the
//      inference; a violation is an excursion of the ISR/service
//      displacement class.
//   5. Coherent repeated excursions (observed intervals agreeing with each
//      other) mean the OCXO genuinely moved; reseed the chain from the
//      observed surface instead of diverging.
//
// Known Stage-1 limitation, deliberately measured rather than masked: the
// chain interval is seeded from ONE quantized observation and may carry up
// to ~one lattice step of constant seed bias.  Seed bias appears as (a) a
// constant offset in the agree-second inferred-minus-observed mean and (b) a
// linear endpoint walk at that same rate.  Stage-1 telemetry exposes both so
// Stage 2 can choose the seed/anchor policy from data.

// SmartZero Gen-2 per-second worthiness classification.  Exactly one kind is
// assigned per observed second by ocxo_lane_yardstick_observe.
enum class zero_worthy_second_kind_t : uint8_t {
  AGREE,                 // clean gate-agree second, fresh yardstick
  EXCURSION,             // gate excursion (incl. both halves of a paired transient)
  STALE,                 // no fresh PPS yardstick this second
  SEED,                  // chain (re)seeded from observed this second
  RAIL_INVALID,          // chain not yet valid (init)
  ADJACENCY_FAULT,       // non-1-second counter lineage
  NO_OBSERVED_INTERVAL,  // chained but no prior observed endpoint (degenerate)
};

static void ocxo_lane_zero_worthy_update(ocxo_lane_t& lane,
                                         zero_worthy_second_kind_t kind,
                                         int32_t auth_error_cycles,
                                         bool anchor_fast_lock_active) {
  lane.zw_seconds_evaluated++;
  const bool was_worthy = lane.zw_worthy;

  const char* disqualify = nullptr;
  switch (kind) {
    case zero_worthy_second_kind_t::AGREE:
      break;
    case zero_worthy_second_kind_t::EXCURSION:
      disqualify = "zw_gate_excursion";
      lane.zw_disqualify_excursion_count++;
      break;
    case zero_worthy_second_kind_t::STALE:
      disqualify = "zw_yardstick_stale";
      lane.zw_disqualify_stale_count++;
      break;
    case zero_worthy_second_kind_t::SEED:
      disqualify = "zw_chain_seed";
      lane.zw_disqualify_seed_count++;
      break;
    case zero_worthy_second_kind_t::RAIL_INVALID:
      disqualify = "zw_rail_invalid";
      lane.zw_disqualify_rail_invalid_count++;
      break;
    case zero_worthy_second_kind_t::ADJACENCY_FAULT:
      disqualify = "zw_counter_adjacency";
      lane.zw_disqualify_adjacency_count++;
      break;
    case zero_worthy_second_kind_t::NO_OBSERVED_INTERVAL:
      disqualify = "zw_no_observed_interval";
      lane.zw_disqualify_no_interval_count++;
      break;
  }

  if (!disqualify && kind == zero_worthy_second_kind_t::AGREE) {
    if (anchor_fast_lock_active) {
      // Acquisition-gain seconds are definitionally not converged state:
      // the hot integral (1/32) passes lattice and PPS-witness noise into
      // the interval at 8x tracking gain.  They are excluded from
      // worthiness eligibility so they neither qualify a zero nor pollute
      // the magnitude/sum disqualification statistics.
      disqualify = "zw_fast_lock_acquisition";
      lane.zw_disqualify_fast_lock_count++;
    }
  }
  if (!disqualify && kind == zero_worthy_second_kind_t::AGREE) {
    const int32_t abs_err = auth_error_cycles < 0
        ? -auth_error_cycles : auth_error_cycles;
    if (abs_err > ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES) {
      disqualify = "zw_auth_error_magnitude";
      lane.zw_disqualify_error_magnitude_count++;
    }
  }

  if (disqualify) {
    // Any unclean second empties the window: the streak restarts from zero.
    for (uint32_t i = 0; i < ZERO_WORTHY_STREAK_SECONDS; i++) {
      lane.zw_error_ring[i] = 0;
    }
    lane.zw_ring_index = 0;
    lane.zw_ring_count = 0;
    lane.zw_window_error_sum = 0;
    lane.zw_window_max_abs_error = 0;
    lane.zw_clean_streak_seconds = 0;
    lane.zw_worthy = false;
    lane.zw_worthy_streak_seconds = 0;
    lane.zw_last_disqualify_reason = disqualify;
    if (was_worthy) lane.zw_transition_count++;
    return;
  }

  // Clean second: push the anchor loop error into the ring.
  if (lane.zw_ring_count == ZERO_WORTHY_STREAK_SECONDS) {
    lane.zw_window_error_sum -= lane.zw_error_ring[lane.zw_ring_index];
  } else {
    lane.zw_ring_count++;
  }
  lane.zw_error_ring[lane.zw_ring_index] = auth_error_cycles;
  lane.zw_window_error_sum += auth_error_cycles;
  lane.zw_ring_index =
      (uint8_t)((lane.zw_ring_index + 1U) % ZERO_WORTHY_STREAK_SECONDS);
  lane.zw_clean_streak_seconds++;

  uint32_t max_abs = 0;
  for (uint32_t i = 0; i < lane.zw_ring_count; i++) {
    const int32_t v = lane.zw_error_ring[i];
    const uint32_t a = (uint32_t)(v < 0 ? -v : v);
    if (a > max_abs) max_abs = a;
  }
  lane.zw_window_max_abs_error = max_abs;

  bool worthy = false;
  if (lane.zw_ring_count < ZERO_WORTHY_STREAK_SECONDS) {
    lane.zw_last_disqualify_reason = "zw_streak_building";
    lane.zw_disqualify_streak_building_count++;
  } else {
    const int32_t abs_sum = lane.zw_window_error_sum < 0
        ? -lane.zw_window_error_sum : lane.zw_window_error_sum;
    if (abs_sum > ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES) {
      // The retrace detector: persistent same-sign loop error means the
      // oven is still moving; the anchor would memorialize a transient.
      lane.zw_last_disqualify_reason = "zw_error_sum_retrace";
      lane.zw_disqualify_error_sum_count++;
    } else {
      worthy = true;
    }
  }

  lane.zw_worthy = worthy;
  if (worthy) {
    lane.zw_worthy_second_count++;
    lane.zw_worthy_streak_seconds++;
    if (lane.zw_first_worthy_after_seconds == 0) {
      lane.zw_first_worthy_after_seconds = lane.zw_seconds_evaluated;
    }
  } else {
    lane.zw_worthy_streak_seconds = 0;
  }
  if (worthy != was_worthy) lane.zw_transition_count++;
}

// System-level all-lanes verdict.  Disabled lanes are vacuously worthy,
// mirroring v1 SmartZero's synthetic proof for disabled lanes.  Counting is
// keyed to the PPS yardstick sequence so each qualified GNSS second is
// counted exactly once even though both lanes call through here.
static void smartzero2_system_update(uint32_t pps_sequence) {
  ocxo_lane_t* l1 = ocxo_lane_for(interrupt_subscriber_kind_t::OCXO1);
  ocxo_lane_t* l2 = ocxo_lane_for(interrupt_subscriber_kind_t::OCXO2);
  const bool l1_ok =
      ocxo_kind_disabled(interrupt_subscriber_kind_t::OCXO1) ||
      (l1 && l1->zw_worthy);
  const bool l2_ok =
      ocxo_kind_disabled(interrupt_subscriber_kind_t::OCXO2) ||
      (l2 && l2->zw_worthy);
  const bool all = l1_ok && l2_ok;
  g_smartzero2.all_lanes_worthy = all;

  if (pps_sequence != 0 &&
      pps_sequence != g_smartzero2.last_pps_sequence_checked) {
    g_smartzero2.last_pps_sequence_checked = pps_sequence;
    if (all) {
      g_smartzero2.all_worthy_second_count++;
      g_smartzero2.all_worthy_streak_seconds++;
      if (g_smartzero2.first_all_worthy_pps_sequence == 0) {
        g_smartzero2.first_all_worthy_pps_sequence = pps_sequence;
      }
    } else {
      g_smartzero2.all_worthy_streak_seconds = 0;
    }
  }
}

static uint32_t ocxo_lane_yardstick_observe(ocxo_lane_t& lane,
                                            uint32_t observed_dwt,
                                            bool counter_adjacency_valid,
                                            uint32_t counter_delta_ticks,
                                            dwt_repair_diag_t* out_diag,
                                            const char** out_reason) {
  const char* reason = "ocxo_yardstick_init";
  uint32_t published_dwt = observed_dwt;

  const bool counter_adjacency_ok =
      !counter_adjacency_valid ||
      counter_delta_ticks == OCXO_WITNESS_ONE_SECOND_COUNTS;

  pps_yardstick_snapshot_t y{};
  const bool y_loaded = pps_yardstick_load(y);
  const bool y_usable = y_loaded && y.valid &&
                        y.interval_now_cycles != 0 &&
                        y.interval_prev_cycles != 0;

  uint32_t pps_seq_delta = 0;
  if (y_usable) {
    pps_seq_delta = lane.yardstick_last_used_pps_sequence_valid
        ? (y.sequence - lane.yardstick_last_used_pps_sequence)
        : 1U;
  }

  const int64_t delta_g = y_usable
      ? (int64_t)y.interval_now_cycles - (int64_t)y.interval_prev_cycles
      : 0;
  const bool delta_g_sane =
      (delta_g >= -(int64_t)PPS_YARDSTICK_MAX_ABS_DELTA_G_CYCLES) &&
      (delta_g <=  (int64_t)PPS_YARDSTICK_MAX_ABS_DELTA_G_CYCLES);

  const bool stale = !y_usable || pps_seq_delta == 0 || !delta_g_sane;

  // Observed one-second interval against this rail's own previous endpoint.
  const bool have_observed_interval = lane.yardstick_prev_observed_valid;
  const uint32_t observed_interval = have_observed_interval
      ? (observed_dwt - lane.yardstick_prev_observed_dwt)
      : 0U;

  bool seeded = false;
  bool excursion = false;
  bool anchor_applied = false;
  int32_t auth_error_cycles = 0;
  int32_t inferred_minus_observed = 0;
  int32_t endpoint_minus_observed = 0;
  uint32_t inferred_interval_rounded = 0;
  zero_worthy_second_kind_t zw_kind =
      zero_worthy_second_kind_t::RAIL_INVALID;
  bool zw_fast_lock_active = false;

  if (counter_adjacency_valid && !counter_adjacency_ok) {
    // Non-adjacent target identity: the interval ending at this edge is not
    // a lawful one-second sample.  Drop both rails; the next adjacent pair
    // reseeds them.  The edge itself is real and is published as observed.
    lane.yardstick_chain_valid = false;
    lane.yardstick_auth_valid = false;
    lane.yardstick_adjacency_reseed_count++;
    lane.yardstick_excursion_streak = 0;
    lane.yardstick_auth_observed_fallback_count++;
    reason = "ocxo_yardstick_adjacency_observed";
    zw_kind = zero_worthy_second_kind_t::ADJACENCY_FAULT;
  } else if (!lane.yardstick_chain_valid) {
    if (have_observed_interval && observed_interval != 0) {
      // Seed both rails from the first lawful observed interval (quantized;
      // the anchor's integral term absorbs the seed bias in steady state).
      lane.yardstick_chain_valid = true;
      lane.yardstick_chain_interval_q16 = ((uint64_t)observed_interval) << 16;
      lane.yardstick_chain_endpoint_q16 =
          (((uint64_t)observed_dwt) << 16) & YARDSTICK_ENDPOINT_Q16_MASK;
      lane.yardstick_auth_valid = true;
      lane.yardstick_auth_interval_q16 = ((uint64_t)observed_interval) << 16;
      lane.yardstick_auth_endpoint_q16 =
          (((uint64_t)observed_dwt) << 16) & YARDSTICK_ENDPOINT_Q16_MASK;
      lane.yardstick_seed_count++;
      lane.yardstick_excursion_streak = 0;
      seeded = true;
      lane.yardstick_auth_fast_lock_remaining_seconds =
          OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS;
      inferred_interval_rounded = observed_interval;
      reason = "ocxo_yardstick_seed";
      zw_kind = zero_worthy_second_kind_t::SEED;
    } else {
      lane.yardstick_auth_observed_fallback_count++;
      reason = "ocxo_yardstick_init";
      zw_kind = zero_worthy_second_kind_t::RAIL_INVALID;
    }
  } else {
    // ---- Free chain (Stage 1 physics audit, unchanged) ----
    zw_fast_lock_active =
        lane.yardstick_auth_fast_lock_remaining_seconds > 0U;
    if (!stale) {
      const int64_t correction_q16 =
          ((int64_t)lane.yardstick_chain_interval_q16 * delta_g) /
          (int64_t)y.interval_prev_cycles;
      lane.yardstick_chain_interval_q16 =
          (uint64_t)((int64_t)lane.yardstick_chain_interval_q16 +
                     correction_q16);
    } else {
      lane.yardstick_stale_hold_count++;
    }

    lane.yardstick_chain_endpoint_q16 =
        (lane.yardstick_chain_endpoint_q16 +
         lane.yardstick_chain_interval_q16) &
        YARDSTICK_ENDPOINT_Q16_MASK;

    inferred_interval_rounded =
        (uint32_t)((lane.yardstick_chain_interval_q16 + 0x8000ULL) >> 16);

    // ---- Anchored authority rail: scale and advance with the same step ----
    if (!stale) {
      const int64_t auth_correction_q16 =
          ((int64_t)lane.yardstick_auth_interval_q16 * delta_g) /
          (int64_t)y.interval_prev_cycles;
      lane.yardstick_auth_interval_q16 =
          (uint64_t)((int64_t)lane.yardstick_auth_interval_q16 +
                     auth_correction_q16);
    }
    lane.yardstick_auth_endpoint_q16 =
        (lane.yardstick_auth_endpoint_q16 +
         lane.yardstick_auth_interval_q16) &
        YARDSTICK_ENDPOINT_Q16_MASK;

    if (have_observed_interval && observed_interval != 0) {
      inferred_minus_observed =
          (int32_t)(inferred_interval_rounded - observed_interval);
      const uint32_t abs_imo = (uint32_t)(inferred_minus_observed < 0
          ? -(int64_t)inferred_minus_observed
          : (int64_t)inferred_minus_observed);
      excursion = abs_imo > OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES;

      const uint32_t inferred_endpoint_dwt =
          (uint32_t)(lane.yardstick_chain_endpoint_q16 >> 16);
      endpoint_minus_observed =
          (int32_t)(inferred_endpoint_dwt - observed_dwt);

      // PI loop error: observed minus advanced anchored endpoint, in Q16,
      // sign-extended from the 48-bit modular endpoint domain.
      int64_t err_q16 =
          (int64_t)(((((uint64_t)observed_dwt) << 16) -
                     lane.yardstick_auth_endpoint_q16) &
                    YARDSTICK_ENDPOINT_Q16_MASK);
      if (err_q16 > (int64_t)(YARDSTICK_ENDPOINT_Q16_MASK >> 1)) {
        err_q16 -= (int64_t)(YARDSTICK_ENDPOINT_Q16_MASK + 1ULL);
      }
      auth_error_cycles = (int32_t)(err_q16 >> 16);

      if (excursion) {
        lane.yardstick_gate_excursion_count++;

        const int32_t coherence = (int32_t)(observed_interval -
            lane.yardstick_last_excursion_observed_interval_cycles);
        const uint32_t abs_coherence = (uint32_t)(coherence < 0
            ? -(int64_t)coherence
            : (int64_t)coherence);
        const bool coherent = lane.yardstick_excursion_streak > 0 &&
            abs_coherence <= OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES;
        lane.yardstick_excursion_streak++;
        lane.yardstick_last_excursion_observed_interval_cycles =
            observed_interval;

        if (coherent &&
            lane.yardstick_excursion_streak >=
                OCXO_YARDSTICK_COHERENT_RESEED_STREAK) {
          lane.yardstick_chain_interval_q16 =
              ((uint64_t)observed_interval) << 16;
          lane.yardstick_chain_endpoint_q16 =
              (((uint64_t)observed_dwt) << 16) & YARDSTICK_ENDPOINT_Q16_MASK;
          lane.yardstick_auth_interval_q16 =
              ((uint64_t)observed_interval) << 16;
          lane.yardstick_auth_endpoint_q16 =
              (((uint64_t)observed_dwt) << 16) & YARDSTICK_ENDPOINT_Q16_MASK;
          lane.yardstick_coherent_reseed_count++;
          lane.yardstick_excursion_streak = 0;
          lane.yardstick_auth_fast_lock_remaining_seconds =
              OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS;
        }

        // Excursion guard: the observed edge is quarantined evidence; the
        // anchored rail coasts on pure inference and publishes it.
        lane.yardstick_auth_excursion_publish_count++;
        reason = "ocxo_yardstick_excursion_guard";
        zw_kind = zero_worthy_second_kind_t::EXCURSION;
      } else {
        lane.yardstick_gate_agree_count++;
        lane.yardstick_excursion_streak = 0;

        lane.yardstick_sum_inferred_minus_observed_cycles +=
            (int64_t)inferred_minus_observed;
        if (abs_imo > lane.yardstick_max_abs_inferred_minus_observed_cycles) {
          lane.yardstick_max_abs_inferred_minus_observed_cycles = abs_imo;
        }
        if (lane.yardstick_gate_agree_count == 1U ||
            endpoint_minus_observed <
                lane.yardstick_endpoint_minus_observed_min_cycles) {
          lane.yardstick_endpoint_minus_observed_min_cycles =
              endpoint_minus_observed;
        }
        if (lane.yardstick_gate_agree_count == 1U ||
            endpoint_minus_observed >
                lane.yardstick_endpoint_minus_observed_max_cycles) {
          lane.yardstick_endpoint_minus_observed_max_cycles =
              endpoint_minus_observed;
        }

        // Anchor correction (agree seconds only).
        lane.yardstick_auth_endpoint_q16 =
            (uint64_t)((int64_t)lane.yardstick_auth_endpoint_q16 +
                       (err_q16 >> OCXO_YARDSTICK_ANCHOR_P_SHIFT)) &
            YARDSTICK_ENDPOINT_Q16_MASK;
        const uint32_t anchor_i_shift =
            lane.yardstick_auth_fast_lock_remaining_seconds > 0U
                ? OCXO_YARDSTICK_ANCHOR_FAST_I_SHIFT
                : OCXO_YARDSTICK_ANCHOR_I_SHIFT;
        lane.yardstick_auth_interval_q16 =
            (uint64_t)((int64_t)lane.yardstick_auth_interval_q16 +
                       (err_q16 >> anchor_i_shift));
        if (lane.yardstick_auth_fast_lock_remaining_seconds > 0U) {
          lane.yardstick_auth_fast_lock_remaining_seconds--;
        }
        lane.yardstick_auth_anchor_correction_count++;
        anchor_applied = true;

        lane.yardstick_auth_agree_publish_count++;
        reason = stale ? "ocxo_yardstick_stale_hold" : "ocxo_yardstick";
        zw_kind = stale ? zero_worthy_second_kind_t::STALE
                        : zero_worthy_second_kind_t::AGREE;
      }
      if (stale && !excursion) {
        lane.yardstick_auth_stale_publish_count++;
        reason = "ocxo_yardstick_stale_hold";
      }
    } else if (stale) {
      lane.yardstick_auth_stale_publish_count++;
      reason = "ocxo_yardstick_stale_hold";
      zw_kind = zero_worthy_second_kind_t::NO_OBSERVED_INTERVAL;
    } else {
      reason = "ocxo_yardstick";
      zw_kind = zero_worthy_second_kind_t::NO_OBSERVED_INTERVAL;
    }

    published_dwt =
        (uint32_t)((lane.yardstick_auth_endpoint_q16 + 0x8000ULL) >> 16);
    lane.yardstick_auth_publish_count++;
    lane.yardstick_update_count++;
  }

  // Consume the yardstick step and advance this rail's observed baseline.
  if (y_usable && pps_seq_delta != 0) {
    lane.yardstick_last_used_pps_sequence = y.sequence;
    lane.yardstick_last_used_pps_sequence_valid = true;
  }
  lane.yardstick_prev_observed_dwt = observed_dwt;
  lane.yardstick_prev_observed_valid = true;

  ocxo_lane_zero_worthy_update(lane, zw_kind, auth_error_cycles,
                               zw_fast_lock_active);
  smartzero2_system_update(y_usable ? y.sequence : 0U);

  lane.yardstick_last_valid = lane.yardstick_chain_valid;
  lane.yardstick_last_stale = stale;
  lane.yardstick_last_excursion = excursion;
  lane.yardstick_last_g_now_cycles = y_usable ? y.interval_now_cycles : 0U;
  lane.yardstick_last_g_prev_cycles = y_usable ? y.interval_prev_cycles : 0U;
  lane.yardstick_last_pps_seq_delta = pps_seq_delta;
  lane.yardstick_last_inferred_interval_cycles = inferred_interval_rounded;
  lane.yardstick_last_observed_interval_cycles = observed_interval;
  lane.yardstick_last_inferred_minus_observed_cycles = inferred_minus_observed;
  lane.yardstick_last_endpoint_minus_observed_cycles = endpoint_minus_observed;
  lane.yardstick_auth_last_error_cycles = auth_error_cycles;
  lane.yardstick_auth_last_published_dwt = published_dwt;

  if (out_diag) {
    out_diag->yardstick_valid = lane.yardstick_chain_valid;
    out_diag->yardstick_stale = stale;
    out_diag->yardstick_seeded_this_event = seeded;
    out_diag->yardstick_excursion = excursion;
    out_diag->yardstick_pps_sequence = y_usable ? y.sequence : 0U;
    out_diag->yardstick_pps_seq_delta = pps_seq_delta;
    out_diag->yardstick_g_now_cycles = y_usable ? y.interval_now_cycles : 0U;
    out_diag->yardstick_g_prev_cycles = y_usable ? y.interval_prev_cycles : 0U;
    out_diag->yardstick_inferred_interval_cycles = inferred_interval_rounded;
    out_diag->yardstick_observed_interval_cycles = observed_interval;
    out_diag->yardstick_inferred_minus_observed_cycles =
        inferred_minus_observed;
    out_diag->yardstick_inferred_endpoint_dwt =
        (uint32_t)(lane.yardstick_chain_endpoint_q16 >> 16);
    out_diag->yardstick_inferred_endpoint_frac_q16 =
        (uint32_t)(lane.yardstick_chain_endpoint_q16 & 0xFFFFULL);
    out_diag->yardstick_endpoint_minus_observed_cycles =
        endpoint_minus_observed;
    out_diag->yardstick_gate_threshold_cycles =
        OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES;
    out_diag->yardstick_gate_agree_count = lane.yardstick_gate_agree_count;
    out_diag->yardstick_gate_excursion_count =
        lane.yardstick_gate_excursion_count;
    out_diag->yardstick_authority = OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED;
    out_diag->yardstick_auth_endpoint_dwt =
        (uint32_t)(lane.yardstick_auth_endpoint_q16 >> 16);
    out_diag->yardstick_auth_endpoint_frac_q16 =
        (uint32_t)(lane.yardstick_auth_endpoint_q16 & 0xFFFFULL);
    out_diag->yardstick_auth_error_cycles = auth_error_cycles;
    out_diag->yardstick_auth_anchor_applied = anchor_applied;
  }

  if (out_reason) *out_reason = reason;
  return published_dwt;
}

static bool spincatch_current_target_for_kind(interrupt_subscriber_kind_t kind,
                                              uint32_t& target_counter32) {
  target_counter32 = 0;

  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  if (!ctx || !ctx->lane) return false;

  const ocxo_lane_t& lane = *ctx->lane;
  if (!lane.initialized || !lane.cadence_enabled ||
      !lane.witness_target_initialized || lane.cadence_next_counter32 == 0) {
    return false;
  }

  target_counter32 = lane.cadence_next_counter32;
  return true;
}

static spincatch_landing_attempt_t* spincatch_attempts_for_kind(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return g_spincatch_ocxo1_attempts;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return g_spincatch_ocxo2_attempts;
  return nullptr;
}

static spincatch_landing_context_t* spincatch_landing_contexts_for_kind(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return g_spincatch_ocxo1_landing_contexts;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return g_spincatch_ocxo2_landing_contexts;
  return nullptr;
}

static uint32_t spincatch_landing_attempt_active_count(
    interrupt_subscriber_kind_t kind) {
  spincatch_landing_attempt_t* attempts = spincatch_attempts_for_kind(kind);
  if (!attempts) return 0;

  uint32_t count = 0;
  for (uint32_t i = 0; i < SPINCATCH_LANDING_ATTEMPT_RING_SIZE; i++) {
    if (attempts[i].active) count++;
  }
  return count;
}

static void spincatch_landing_refresh_pending(spincatch_lane_report_t& s,
                                              interrupt_subscriber_kind_t kind) {
  const uint32_t active = spincatch_landing_attempt_active_count(kind);
  s.landing_attempt_active_count = active;
  s.landing_pending = active != 0;
}

static void spincatch_landing_retire_stale_attempts(
    interrupt_subscriber_kind_t kind,
    spincatch_lane_report_t& s,
    uint32_t current_target_counter32) {
  spincatch_landing_attempt_t* attempts = spincatch_attempts_for_kind(kind);
  if (!attempts) return;

  const uint32_t stale_ticks =
      OCXO_WITNESS_ONE_SECOND_COUNTS * SPINCATCH_LANDING_STALE_TARGET_PERIODS;

  for (uint32_t i = 0; i < SPINCATCH_LANDING_ATTEMPT_RING_SIZE; i++) {
    spincatch_landing_attempt_t& a = attempts[i];
    if (!a.active) continue;
    if ((uint32_t)(current_target_counter32 - a.target_counter32) <= stale_ticks) continue;

    a.active = false;
    a.handle = TIMEPOP_INVALID_HANDLE;
    s.landing_attempt_stale_count++;
    s.missed_count++;
    s.landing_target_changed_count++;
    s.landing_last_success = false;
    s.landing_last_target_changed = true;
  }

  spincatch_landing_refresh_pending(s, kind);
}

static bool spincatch_landing_attempt_exists(interrupt_subscriber_kind_t kind,
                                             uint32_t target_counter32) {
  spincatch_landing_attempt_t* attempts = spincatch_attempts_for_kind(kind);
  if (!attempts) return false;

  for (uint32_t i = 0; i < SPINCATCH_LANDING_ATTEMPT_RING_SIZE; i++) {
    if (attempts[i].active && attempts[i].target_counter32 == target_counter32) {
      return true;
    }
  }
  return false;
}

static spincatch_landing_attempt_t* spincatch_landing_alloc_attempt(
    interrupt_subscriber_kind_t kind,
    uint32_t* out_index) {
  spincatch_landing_attempt_t* attempts = spincatch_attempts_for_kind(kind);
  if (!attempts) return nullptr;

  for (uint32_t i = 0; i < SPINCATCH_LANDING_ATTEMPT_RING_SIZE; i++) {
    if (!attempts[i].active) {
      if (out_index) *out_index = i;
      return &attempts[i];
    }
  }
  return nullptr;
}

static void spincatch_landing_callback(timepop_ctx_t* ctx,
                                       timepop_diag_t*,
                                       void* user_data) {
  auto* landing_ctx = static_cast<spincatch_landing_context_t*>(user_data);
  if (!landing_ctx) return;
  if (landing_ctx->attempt_index >= SPINCATCH_LANDING_ATTEMPT_RING_SIZE) return;

  spincatch_lane_report_t* s = spincatch_report_for_kind(landing_ctx->kind);
  spincatch_landing_attempt_t* attempts = spincatch_attempts_for_kind(landing_ctx->kind);
  if (!s || !attempts) return;

  spincatch_landing_attempt_t& attempt = attempts[landing_ctx->attempt_index];
  if (!attempt.active || attempt.generation != landing_ctx->generation) {
    // Stale callback from a retired/reused attempt.  The attempt ring makes
    // this harmless; do not let it perturb the current lane result.
    spincatch_landing_refresh_pending(*s, landing_ctx->kind);
    return;
  }

  attempt.active = false;
  attempt.handle = TIMEPOP_INVALID_HANDLE;
  s->landing_count++;

  interrupt_clock_snapshot_t snap{};
  const bool snap_ok = interrupt_clock_snapshot(landing_ctx->kind, &snap);

  const uint32_t landing_counter32 = snap_ok ? snap.counter32 : 0U;
  const uint32_t target_counter32 = attempt.target_counter32;
  uint32_t current_target = 0;
  const bool current_target_ok =
      spincatch_current_target_for_kind(landing_ctx->kind, current_target);
  const bool target_changed = current_target_ok && current_target != target_counter32;

  const int32_t signed_until_target =
      (int32_t)(target_counter32 - landing_counter32);
  const bool before_target = signed_until_target > 0;
  const bool success = snap_ok && before_target;

  s->landing_last_success = success;
  s->landing_last_target_changed = target_changed;
  s->landing_dwt = (ctx && ctx->fire_dwt_cyccnt != 0)
      ? ctx->fire_dwt_cyccnt
      : ARM_DWT_CYCCNT;
  s->landing_counter32 = landing_counter32;
  s->landing_offset_ticks = (int32_t)(landing_counter32 - target_counter32);
  s->landing_ticks_until_target = before_target ? (uint32_t)signed_until_target : 0U;
  s->landing_fire_vclock_raw = ctx ? ctx->fire_vclock_raw : 0U;
  s->landing_fire_dwt = ctx ? ctx->fire_dwt_cyccnt : 0U;
  s->landing_target_counter32 = target_counter32;
  s->landing_target_low16 = attempt.target_low16;

  if (success) {
    s->success_count++;
  } else {
    s->missed_count++;
    if (target_changed) {
      s->landing_target_changed_count++;
    }
  }

  spincatch_landing_refresh_pending(*s, landing_ctx->kind);
}

static void spincatch_maybe_schedule_ocxo_landing(
    ocxo_runtime_context_t& ctx,
    const interrupt_perishable_fact_t& fact) {
  if (!SPINCATCH_OCXO_LANDING_ONLY_ENABLED) return;
  if (ctx.kind != interrupt_subscriber_kind_t::OCXO1 &&
      ctx.kind != interrupt_subscriber_kind_t::OCXO2) return;
  if (!fact.one_second_due) return;
  if (!ctx.lane || !ctx.clock32) return;

  spincatch_lane_report_t* s = spincatch_report_for_kind(ctx.kind);
  spincatch_landing_context_t* contexts =
      spincatch_landing_contexts_for_kind(ctx.kind);
  if (!s || !contexts) return;

  s->enabled = true;
  s->landing_only_enabled = true;
  s->landing_duty_divisor = SPINCATCH_OCXO_LANDING_DUTY_DIVISOR;
  s->landing_attempt_ring_size = SPINCATCH_LANDING_ATTEMPT_RING_SIZE;
  s->landing_duty_counter++;

  const ocxo_lane_t& lane = *ctx.lane;
  const bool target_valid =
      lane.initialized && lane.cadence_enabled &&
      lane.witness_target_initialized && lane.cadence_next_counter32 != 0;
  if (!target_valid) {
    s->landing_schedule_skip_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  const uint32_t target_counter32 = lane.cadence_next_counter32;
  spincatch_landing_retire_stale_attempts(ctx.kind, *s, target_counter32);

  const uint32_t duty = spincatch_landing_duty_for_kind(ctx.kind);
  if (duty != 0 && (s->landing_duty_counter % duty) != 0) {
    s->landing_schedule_skip_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  if (spincatch_landing_attempt_exists(ctx.kind, target_counter32)) {
    s->landing_attempt_duplicate_skip_count++;
    s->landing_schedule_skip_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  interrupt_clock_snapshot_t snap{};
  if (!interrupt_clock_snapshot(ctx.kind, &snap)) {
    s->landing_schedule_skip_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  const uint32_t remaining_ticks = target_counter32 - snap.counter32;
  if (remaining_ticks == 0 || remaining_ticks > 0x7FFFFFFFUL ||
      remaining_ticks <= (SPINCATCH_APPROACH_TICKS + SPINCATCH_LANDING_MIN_LEAD_TICKS)) {
    s->landing_too_close_count++;
    s->landing_schedule_skip_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  uint32_t attempt_index = 0;
  spincatch_landing_attempt_t* attempt =
      spincatch_landing_alloc_attempt(ctx.kind, &attempt_index);
  if (!attempt) {
    s->landing_attempt_ring_full_count++;
    s->landing_schedule_skip_count++;
    s->clash_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  const uint32_t delay_ticks = remaining_ticks - SPINCATCH_APPROACH_TICKS;
  const uint64_t delay_ns = (uint64_t)delay_ticks * 100ULL;
  const uint32_t generation = ++s->landing_attempt_generation;

  contexts[attempt_index].kind = ctx.kind;
  contexts[attempt_index].attempt_index = attempt_index;
  contexts[attempt_index].generation = generation;

  const timepop_handle_t h =
      timepop_arm(delay_ns,
                  false,
                  spincatch_landing_callback,
                  &contexts[attempt_index],
                  nullptr);
  if (h == TIMEPOP_INVALID_HANDLE) {
    s->landing_schedule_fail_count++;
    spincatch_landing_refresh_pending(*s, ctx.kind);
    return;
  }

  *attempt = spincatch_landing_attempt_t{};
  attempt->active = true;
  attempt->kind = ctx.kind;
  attempt->generation = generation;
  attempt->handle = h;
  attempt->target_counter32 = target_counter32;
  attempt->target_low16 = lane.cadence_next_low16;
  attempt->scheduled_counter32 = snap.counter32;
  attempt->scheduled_ticks_until_target = remaining_ticks;
  attempt->scheduled_delay_ns = delay_ns;
  attempt->scheduled_dwt = ARM_DWT_CYCCNT;

  s->landing_handle = h;
  s->landing_schedule_count++;
  s->landing_target_counter32 = target_counter32;
  s->landing_target_low16 = lane.cadence_next_low16;
  s->landing_scheduled_counter32 = snap.counter32;
  s->landing_scheduled_ticks_until_target = remaining_ticks;
  s->landing_scheduled_delay_ns = delay_ns;
  s->landing_scheduled_dwt = attempt->scheduled_dwt;
  spincatch_landing_refresh_pending(*s, ctx.kind);
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

  const uint32_t observed_dwt = corrected_dwt;

  dwt_repair_diag_t ema_diag{};
  ema_diag.valid = true;
  ema_diag.original_dwt = observed_dwt;
  ema_diag.isr_entry_dwt_raw = fact.isr_entry_dwt_raw;
  ema_diag.event_dwt_from_isr_entry_raw = observed_dwt;
  ema_diag.isr_entry_to_event_correction_cycles =
      (int32_t)(observed_dwt - fact.isr_entry_dwt_raw);

  // A skipped or duplicated one-second target is a custody discontinuity, not
  // an interval sample.  The EMA function uses this adjacency fact to publish
  // a projected endpoint while preserving the raw observed DWT as evidence.
  const bool counter_adjacency_valid =
      lane->witness_previous_event_counter32_valid;
  const uint32_t counter_delta_ticks = counter_adjacency_valid
      ? (fact.target_counter32 - lane->witness_previous_event_counter32)
      : 0U;

  // EMA rail (diagnostic since Stage 2).  Still computed every second with
  // full gate/adjacency audit so TIMEBASE can compare it against the
  // yardstick authority; its endpoint no longer publishes.
  const uint32_t ema_published_dwt = ocxo_lane_ema_predict_dwt(
      *lane, observed_dwt, counter_adjacency_valid, counter_delta_ticks,
      &ema_synthetic, &ema_error_cycles, &ema_diag);

  // PPS-Yardstick anchored authority rail (Stage 2).  Free chain remains the
  // physics audit; the anchored PI rail produces the published endpoint.
  const char* yardstick_reason = "ocxo_yardstick_init";
  const uint32_t yardstick_published_dwt = ocxo_lane_yardstick_observe(
      *lane, observed_dwt, counter_adjacency_valid, counter_delta_ticks,
      &ema_diag, &yardstick_reason);

  const uint32_t published_dwt = OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED
      ? yardstick_published_dwt
      : ema_published_dwt;
  const bool published_synthetic = (published_dwt != observed_dwt);
  ema_diag.ema_dwt_at_event = ema_published_dwt;

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
                          published_synthetic);

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

  // Truthful authority audit.  Under yardstick authority the published
  // endpoint, its error against the observed edge, the gate threshold, and
  // the reason string all describe the yardstick rail; the EMA rail's
  // endpoint and full interval-gate audit remain in the diag as diagnostics.
  ema_diag.candidate = published_synthetic;
  ema_diag.synthetic = published_synthetic;
  ema_diag.predicted_dwt = published_dwt;
  ema_diag.used_dwt = published_dwt;
  if (OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED) {
    ema_diag.error_cycles = (int32_t)(published_dwt - observed_dwt);
    ema_diag.threshold_cycles = OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES;
    ema_diag.reason = yardstick_reason;
  } else {
    ema_diag.error_cycles = ema_error_cycles;
    ema_diag.threshold_cycles = OCXO_EMA_INTERVAL_GATE_THRESHOLD_CYCLES;
    ema_diag.reason = ema_diag.interval_adjacency_rejected
        ? "ocxo_counter_adjacency_reject_projected"
        : (ema_diag.interval_sample_rejected
              ? "ocxo_ema_gate_reject"
              : (ema_synthetic ? "ocxo_ema" : "ocxo_ema_init"));
  }

  // We are already in TimePop ASAP/foreground context. Dispatch the authored
  // rollover OCXO edge directly from the fact drain.
  emit_one_second_event(*rt,
                        published_dwt,
                        fact.target_counter32,
                        0,
                        false,
                        &ema_diag,
                        true,
                        nullptr,
                        &fact.spinidle);

  // Schedule at most one low-duty landing-only callback for the next
  // already-authored OCXO target.  OCXO1 and OCXO2 use identical scheduling,
  // attempt-ring, and reporting rules.  This records the TimePop landing facts
  // only; it never touches the OCXO compare hardware.
  spincatch_maybe_schedule_ocxo_landing(ctx, fact);
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
  spinidle_isr_capture_t spinidle{};
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

// ============================================================================
// Priority handoff tier — reentry-safe capture packets
// ============================================================================
//
// Priority-0 ISRs capture immutable packets only.  The handoff tier runs at
// priority 16 and performs the former ISR continuation work: event authorship,
// rollover tending, SmartZero feed/rearm, TimePop delivery, and ASAP drain
// arming.  Per-source rings make priority-0 capture reentry-safe: a later
// interrupt cannot overwrite an earlier packet before the handoff tier owns it.

struct interrupt_handoff_source_diag_t {
  volatile uint32_t capture_count = 0;
  volatile uint32_t enqueue_count = 0;
  volatile uint32_t dequeue_count = 0;
  volatile uint32_t overrun_count = 0;
  volatile uint32_t high_water = 0;
  volatile uint32_t pending_count = 0;
  volatile uint32_t last_sequence = 0;
  volatile uint32_t last_capture_dwt = 0;
  volatile uint32_t last_handoff_entry_dwt = 0;
  volatile uint32_t last_capture_to_handoff_cycles = 0;
  volatile uint32_t max_capture_to_handoff_cycles = 0;
  volatile uint32_t last_priority0_body_cycles = 0;
  volatile uint32_t max_priority0_body_cycles = 0;
  volatile uint32_t last_handoff_body_cycles = 0;
  volatile uint32_t max_handoff_body_cycles = 0;
};

struct interrupt_handoff_diag_t {
  volatile bool configured = false;
  volatile bool running = false;
  volatile bool pending = false;
  volatile uint32_t configure_count = 0;
  volatile uint32_t request_count = 0;
  volatile uint32_t entry_count = 0;
  volatile uint32_t exit_count = 0;
  volatile uint32_t served_request_count = 0;
  volatile uint32_t unserved_request_count = 0;
  volatile uint32_t spurious_entry_count = 0;
  volatile uint32_t reentry_count = 0;
  volatile uint32_t repend_count = 0;
  volatile uint32_t drain_pass_count = 0;
  volatile uint32_t drain_budget_exhausted_count = 0;
  volatile uint32_t last_request_dwt = 0;
  volatile uint32_t last_entry_dwt = 0;
  volatile uint32_t last_exit_dwt = 0;
  volatile uint32_t last_latency_cycles = 0;
  volatile uint32_t min_latency_cycles = UINT32_MAX;
  volatile uint32_t max_latency_cycles = 0;
  volatile uint64_t latency_cycles_sum = 0;
  volatile uint32_t latency_sample_count = 0;
  volatile uint32_t last_body_cycles = 0;
  volatile uint32_t max_body_cycles = 0;
  volatile uint32_t last_request_ipsr = 0;
  volatile uint32_t entry_ipsr = 0;
  volatile uint32_t last_request_primask = 0;
  volatile uint32_t entry_primask = 0;
  volatile const char* last_request_context = nullptr;
};

static interrupt_handoff_diag_t g_interrupt_handoff = {};
static interrupt_handoff_source_diag_t g_handoff_qtimer1_ch1 = {};
static interrupt_handoff_source_diag_t g_handoff_qtimer1_ch2 = {};
static interrupt_handoff_source_diag_t g_handoff_ocxo1 = {};
static interrupt_handoff_source_diag_t g_handoff_ocxo2 = {};
static interrupt_handoff_source_diag_t g_handoff_pps = {};
static volatile uint32_t g_interrupt_capture_sequence = 0;
static volatile uint32_t g_pps_capture_sequence = 0;

struct qtimer1_ch1_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t capture_exit_dwt = 0;
  uint32_t csctrl_entry = 0;
  bool active_at_capture = false;
  uint32_t target_counter32 = 0;
  uint32_t next_compare_counter32 = 0;
  spinidle_isr_capture_t spinidle{};
};

struct qtimer1_ch2_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t ch2_event_counter32 = 0;
  uint32_t capture_exit_dwt = 0;
  uint32_t csctrl_entry = 0;
  spinidle_isr_capture_t spinidle{};
};

struct ocxo_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  uint32_t capture_exit_dwt = 0;
  spinidle_isr_capture_t spinidle{};

  // Raw/near-raw capture facts.  Priority 0 deliberately avoids service
  // classification, event-DWT authorship, SmartZero decisions, synthetic
  // identity updates, and rearm policy.
  uint32_t csctrl_entry = 0;
  uint16_t service_counter_low16 = 0;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  bool cadence_enabled = false;
  bool cadence_armed = false;
  bool lane_active = false;
  bool rt_present = false;
  bool rt_active = false;
  uint32_t arm_remaining_ticks = 0;
  uint16_t arm_low16 = 0;
  uint32_t arm_dwt_raw = 0;
};

struct pps_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t capture_after_vclock_raw = 0;
  uint32_t capture_end_raw = 0;
  uint32_t capture_exit_dwt = 0;
  uint16_t vclock_hardware16_observed = 0;
  uint16_t ocxo1_hardware16 = 0;
  uint16_t ocxo2_hardware16 = 0;
  bool ocxo1_capture_hw_ready = false;
  bool ocxo2_capture_hw_ready = false;
  bool ocxo_capture_hw_ready = false;
  bool rebootstrap_pending_at_capture = false;
  spinidle_isr_capture_t spinidle{};
};

template <typename T, uint32_t N>
struct interrupt_capture_ring_t {
  T items[N]{};
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t count = 0;
};

static interrupt_capture_ring_t<qtimer1_ch1_capture_packet_t,
                                HANDOFF_QTIMER1_CH1_RING_SIZE>
    g_qtimer1_ch1_capture_ring;
static interrupt_capture_ring_t<qtimer1_ch2_capture_packet_t,
                                HANDOFF_QTIMER1_CH2_RING_SIZE>
    g_qtimer1_ch2_capture_ring;
static interrupt_capture_ring_t<ocxo_capture_packet_t, HANDOFF_OCXO_RING_SIZE>
    g_ocxo1_capture_ring;
static interrupt_capture_ring_t<ocxo_capture_packet_t, HANDOFF_OCXO_RING_SIZE>
    g_ocxo2_capture_ring;
static interrupt_capture_ring_t<pps_capture_packet_t, HANDOFF_PPS_RING_SIZE>
    g_pps_capture_ring;

static inline volatile uint32_t& interrupt_nvic_ispr_word(uint32_t irq) {
  volatile uint32_t* const ispr = (volatile uint32_t*)0xE000E200UL;
  return ispr[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_iser_word(uint32_t irq) {
  volatile uint32_t* const iser = (volatile uint32_t*)0xE000E100UL;
  return iser[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_icpr_word(uint32_t irq) {
  volatile uint32_t* const icpr = (volatile uint32_t*)0xE000E280UL;
  return icpr[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_iabr_word(uint32_t irq) {
  volatile uint32_t* const iabr = (volatile uint32_t*)0xE000E300UL;
  return iabr[irq >> 5];
}

static inline uint32_t interrupt_nvic_irq_mask(uint32_t irq) {
  return 1UL << (irq & 31U);
}

static inline void interrupt_handoff_barrier(void) {
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");
}

static inline uint32_t interrupt_ipsr(void) {
  uint32_t v = 0;
  __asm__ volatile ("mrs %0, ipsr" : "=r" (v) :: "memory");
  return v;
}

static inline uint32_t interrupt_primask(void) {
  uint32_t v = 0;
  __asm__ volatile ("mrs %0, primask" : "=r" (v) :: "memory");
  return v;
}

template <typename T, uint32_t N>
static bool capture_ring_push_from_priority0(
    interrupt_capture_ring_t<T, N>& ring,
    interrupt_handoff_source_diag_t& diag,
    T packet) {
  if (ring.count >= N) {
    diag.overrun_count++;
    return false;
  }

  packet.sequence = ++g_interrupt_capture_sequence;
  ring.items[ring.head] = packet;
  ring.head = (ring.head + 1U) % N;
  ring.count++;

  diag.capture_count++;
  diag.enqueue_count++;
  diag.pending_count = ring.count;
  diag.last_sequence = packet.sequence;
  diag.last_capture_dwt = packet.isr_entry_dwt_raw;
  if (ring.count > diag.high_water) diag.high_water = ring.count;
  return true;
}

template <typename T, uint32_t N>
static bool capture_ring_pop_for_handoff(
    interrupt_capture_ring_t<T, N>& ring,
    interrupt_handoff_source_diag_t& diag,
    T& out) {
  __disable_irq();
  if (ring.count == 0) {
    __enable_irq();
    return false;
  }
  out = ring.items[ring.tail];
  ring.tail = (ring.tail + 1U) % N;
  ring.count--;
  diag.dequeue_count++;
  diag.pending_count = ring.count;
  __enable_irq();
  return true;
}

template <typename T, uint32_t N>
static bool capture_ring_peek_sequence(
    interrupt_capture_ring_t<T, N>& ring,
    uint32_t& seq) {
  __disable_irq();
  if (ring.count == 0) {
    __enable_irq();
    return false;
  }
  seq = ring.items[ring.tail].sequence;
  __enable_irq();
  return true;
}

static bool interrupt_handoff_any_pending(void) {
  return g_qtimer1_ch1_capture_ring.count != 0 ||
         g_qtimer1_ch2_capture_ring.count != 0 ||
         g_ocxo1_capture_ring.count != 0 ||
         g_ocxo2_capture_ring.count != 0 ||
         g_pps_capture_ring.count != 0;
}

static void interrupt_handoff_note_priority0_body(
    interrupt_handoff_source_diag_t& diag,
    uint32_t isr_entry_dwt_raw) {
  const uint32_t body = ARM_DWT_CYCCNT - isr_entry_dwt_raw;
  diag.last_priority0_body_cycles = body;
  if (body > diag.max_priority0_body_cycles) {
    diag.max_priority0_body_cycles = body;
  }
}

static void interrupt_handoff_note_source_entry(
    interrupt_handoff_source_diag_t& diag,
    uint32_t capture_dwt,
    uint32_t handoff_entry_dwt) {
  const uint32_t latency = handoff_entry_dwt - capture_dwt;
  diag.last_handoff_entry_dwt = handoff_entry_dwt;
  diag.last_capture_to_handoff_cycles = latency;
  if (latency > diag.max_capture_to_handoff_cycles) {
    diag.max_capture_to_handoff_cycles = latency;
  }
}

static void interrupt_handoff_note_source_body(
    interrupt_handoff_source_diag_t& diag,
    uint32_t start_dwt) {
  const uint32_t body = ARM_DWT_CYCCNT - start_dwt;
  diag.last_handoff_body_cycles = body;
  if (body > diag.max_handoff_body_cycles) {
    diag.max_handoff_body_cycles = body;
  }
}

static void interrupt_handoff_request_from_capture_isr(const char* context) {
  g_interrupt_handoff.request_count++;
  g_interrupt_handoff.pending = true;
  g_interrupt_handoff.last_request_dwt = ARM_DWT_CYCCNT;
  g_interrupt_handoff.last_request_context = context;
  g_interrupt_handoff.last_request_ipsr = interrupt_ipsr();
  g_interrupt_handoff.last_request_primask = interrupt_primask();

  interrupt_nvic_ispr_word(INTERRUPT_HANDOFF_IRQ_NUMBER) =
      interrupt_nvic_irq_mask(INTERRUPT_HANDOFF_IRQ_NUMBER);
  interrupt_handoff_barrier();
}

static void interrupt_handoff_configure(void) {
  if (g_interrupt_handoff.configured) return;

  attachInterruptVector(INTERRUPT_HANDOFF_IRQ, interrupt_handoff_service_isr);
  NVIC_SET_PRIORITY(INTERRUPT_HANDOFF_IRQ, INTERRUPT_PRIORITY_HANDOFF);
  interrupt_nvic_icpr_word(INTERRUPT_HANDOFF_IRQ_NUMBER) =
      interrupt_nvic_irq_mask(INTERRUPT_HANDOFF_IRQ_NUMBER);
  interrupt_handoff_barrier();
  NVIC_ENABLE_IRQ(INTERRUPT_HANDOFF_IRQ);

  g_interrupt_handoff.configured = true;
  g_interrupt_handoff.configure_count++;
}

static void interrupt_handoff_process_qtimer1_ch1(
    const qtimer1_ch1_capture_packet_t& packet);
static void interrupt_handoff_process_qtimer1_ch2(
    const qtimer1_ch2_capture_packet_t& packet);
static void interrupt_handoff_process_ocxo(ocxo_runtime_context_t& ctx,
                                           const ocxo_capture_packet_t& packet);
static void interrupt_handoff_process_pps(const pps_capture_packet_t& packet);

static bool interrupt_handoff_drain_one_oldest(uint32_t handoff_entry_dwt) {
  bool have = false;
  uint32_t best_seq = 0xFFFFFFFFUL;
  uint32_t seq = 0;
  enum source_t : uint8_t { SRC_NONE, SRC_CH1, SRC_CH2, SRC_OCXO1, SRC_OCXO2, SRC_PPS };
  source_t best = SRC_NONE;

  if (capture_ring_peek_sequence(g_qtimer1_ch1_capture_ring, seq) && seq < best_seq) {
    best_seq = seq; best = SRC_CH1; have = true;
  }
  if (capture_ring_peek_sequence(g_qtimer1_ch2_capture_ring, seq) && seq < best_seq) {
    best_seq = seq; best = SRC_CH2; have = true;
  }
  if (capture_ring_peek_sequence(g_ocxo1_capture_ring, seq) && seq < best_seq) {
    best_seq = seq; best = SRC_OCXO1; have = true;
  }
  if (capture_ring_peek_sequence(g_ocxo2_capture_ring, seq) && seq < best_seq) {
    best_seq = seq; best = SRC_OCXO2; have = true;
  }
  if (capture_ring_peek_sequence(g_pps_capture_ring, seq) && seq < best_seq) {
    best_seq = seq; best = SRC_PPS; have = true;
  }
  if (!have) return false;

  const uint32_t body_start = ARM_DWT_CYCCNT;
  switch (best) {
    case SRC_CH1: {
      qtimer1_ch1_capture_packet_t packet{};
      if (capture_ring_pop_for_handoff(g_qtimer1_ch1_capture_ring,
                                       g_handoff_qtimer1_ch1,
                                       packet)) {
        interrupt_handoff_note_source_entry(g_handoff_qtimer1_ch1,
                                            packet.isr_entry_dwt_raw,
                                            handoff_entry_dwt);
        interrupt_handoff_process_qtimer1_ch1(packet);
        interrupt_handoff_note_source_body(g_handoff_qtimer1_ch1, body_start);
        return true;
      }
      break;
    }
    case SRC_CH2: {
      qtimer1_ch2_capture_packet_t packet{};
      if (capture_ring_pop_for_handoff(g_qtimer1_ch2_capture_ring,
                                       g_handoff_qtimer1_ch2,
                                       packet)) {
        interrupt_handoff_note_source_entry(g_handoff_qtimer1_ch2,
                                            packet.isr_entry_dwt_raw,
                                            handoff_entry_dwt);
        interrupt_handoff_process_qtimer1_ch2(packet);
        interrupt_handoff_note_source_body(g_handoff_qtimer1_ch2, body_start);
        return true;
      }
      break;
    }
    case SRC_OCXO1: {
      ocxo_capture_packet_t packet{};
      if (capture_ring_pop_for_handoff(g_ocxo1_capture_ring,
                                       g_handoff_ocxo1,
                                       packet)) {
        interrupt_handoff_note_source_entry(g_handoff_ocxo1,
                                            packet.isr_entry_dwt_raw,
                                            handoff_entry_dwt);
        interrupt_handoff_process_ocxo(g_ocxo1_ctx, packet);
        interrupt_handoff_note_source_body(g_handoff_ocxo1, body_start);
        return true;
      }
      break;
    }
    case SRC_OCXO2: {
      ocxo_capture_packet_t packet{};
      if (capture_ring_pop_for_handoff(g_ocxo2_capture_ring,
                                       g_handoff_ocxo2,
                                       packet)) {
        interrupt_handoff_note_source_entry(g_handoff_ocxo2,
                                            packet.isr_entry_dwt_raw,
                                            handoff_entry_dwt);
        interrupt_handoff_process_ocxo(g_ocxo2_ctx, packet);
        interrupt_handoff_note_source_body(g_handoff_ocxo2, body_start);
        return true;
      }
      break;
    }
    case SRC_PPS: {
      pps_capture_packet_t packet{};
      if (capture_ring_pop_for_handoff(g_pps_capture_ring,
                                       g_handoff_pps,
                                       packet)) {
        interrupt_handoff_note_source_entry(g_handoff_pps,
                                            packet.isr_entry_dwt_raw,
                                            handoff_entry_dwt);
        interrupt_handoff_process_pps(packet);
        interrupt_handoff_note_source_body(g_handoff_pps, body_start);
        return true;
      }
      break;
    }
    default:
      break;
  }
  return false;
}

static void interrupt_handoff_service_isr(void) {
  const uint32_t entry_dwt = ARM_DWT_CYCCNT;  // first instruction: handoff latency proof.

  if (g_interrupt_handoff.running) {
    g_interrupt_handoff.reentry_count++;
    return;
  }

  g_interrupt_handoff.running = true;
  g_interrupt_handoff.entry_count++;
  g_interrupt_handoff.last_entry_dwt = entry_dwt;
  g_interrupt_handoff.entry_ipsr = interrupt_ipsr();
  g_interrupt_handoff.entry_primask = interrupt_primask();

  if (g_interrupt_handoff.pending) {
    g_interrupt_handoff.pending = false;
    g_interrupt_handoff.served_request_count++;
    const uint32_t latency = entry_dwt - g_interrupt_handoff.last_request_dwt;
    g_interrupt_handoff.last_latency_cycles = latency;
    if (latency < g_interrupt_handoff.min_latency_cycles) {
      g_interrupt_handoff.min_latency_cycles = latency;
    }
    if (latency > g_interrupt_handoff.max_latency_cycles) {
      g_interrupt_handoff.max_latency_cycles = latency;
    }
    g_interrupt_handoff.latency_cycles_sum += (uint64_t)latency;
    g_interrupt_handoff.latency_sample_count++;
  } else if (!interrupt_handoff_any_pending()) {
    g_interrupt_handoff.spurious_entry_count++;
  }

  uint32_t drained = 0;
  while (drained < INTERRUPT_HANDOFF_DRAIN_BUDGET &&
         interrupt_handoff_drain_one_oldest(entry_dwt)) {
    drained++;
  }
  g_interrupt_handoff.drain_pass_count++;

  if (interrupt_handoff_any_pending()) {
    g_interrupt_handoff.drain_budget_exhausted_count++;
    g_interrupt_handoff.repend_count++;
    g_interrupt_handoff.pending = true;
    interrupt_nvic_ispr_word(INTERRUPT_HANDOFF_IRQ_NUMBER) =
        interrupt_nvic_irq_mask(INTERRUPT_HANDOFF_IRQ_NUMBER);
    interrupt_handoff_barrier();
  }

  g_interrupt_handoff.running = false;
  g_interrupt_handoff.exit_count++;
  g_interrupt_handoff.last_exit_dwt = ARM_DWT_CYCCNT;
  g_interrupt_handoff.last_body_cycles =
      g_interrupt_handoff.last_exit_dwt - entry_dwt;
  if (g_interrupt_handoff.last_body_cycles > g_interrupt_handoff.max_body_cycles) {
    g_interrupt_handoff.max_body_cycles = g_interrupt_handoff.last_body_cycles;
  }

  if (g_interrupt_handoff.request_count >= g_interrupt_handoff.served_request_count) {
    g_interrupt_handoff.unserved_request_count =
        g_interrupt_handoff.request_count - g_interrupt_handoff.served_request_count;
  }
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
    sample.isr_csctrl_after_program = lane.module->CH[lane.compare_channel].CSCTRL;
    return;
  }

  const bool keep_for_smartzero = smartzero_is_current_lane(ctx.kind);
  const bool keep_running = lane.active || keep_for_smartzero;
  if (!keep_running) {
    ocxo_lane_stop_local_cadence(lane, OCXO_CADENCE_REASON_STOP);
    sample.isr_csctrl_after_program = lane.module->CH[lane.compare_channel].CSCTRL;
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

  sample.isr_csctrl_after_program = lane.module->CH[lane.compare_channel].CSCTRL;
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
  fact.spinidle = sample.spinidle;
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

static void ocxo_cadence_capture_priority0(ocxo_runtime_context_t& ctx,
                                           uint32_t isr_entry_dwt_raw) {
  ocxo_lane_t& lane = *ctx.lane;
  interrupt_handoff_source_diag_t& diag =
      (ctx.kind == interrupt_subscriber_kind_t::OCXO1)
          ? g_handoff_ocxo1
          : g_handoff_ocxo2;

  const uint32_t csctrl_entry = lane.module->CH[lane.compare_channel].CSCTRL;

  // False/unowned IRQs are defused in the capture tier; there is no meaningful
  // edge packet to hand off.
  if ((csctrl_entry & TMR_CSCTRL_TCF1) == 0) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    interrupt_handoff_note_priority0_body(diag, isr_entry_dwt_raw);
    return;
  }

  if (!lane.cadence_enabled || !lane.cadence_armed) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    ocxo_lane_clear_compare_flag(lane);
    ocxo_lane_disable_compare(lane);
    lane.cadence_armed = false;
    lane.witness_armed = false;
    interrupt_handoff_note_priority0_body(diag, isr_entry_dwt_raw);
    return;
  }

  ocxo_capture_packet_t packet{};
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.kind = ctx.kind;
  packet.csctrl_entry = csctrl_entry;
  packet.service_counter_low16 = ocxo_lane_counter_now(lane);
  packet.target_counter32 = lane.cadence_next_counter32;
  packet.target_low16 = (uint16_t)(packet.target_counter32 & 0xFFFFU);
  packet.cadence_enabled = lane.cadence_enabled;
  packet.cadence_armed = lane.cadence_armed;
  packet.lane_active = lane.active;
  interrupt_subscriber_runtime_t* const rt_at_capture = ocxo_runtime_for(ctx);
  packet.rt_present = (rt_at_capture != nullptr);
  packet.rt_active = rt_at_capture && rt_at_capture->active;
  packet.arm_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  packet.arm_low16 = lane.witness_last_arm_low16;
  packet.arm_dwt_raw = lane.witness_last_arm_dwt_raw;

  // Defuse the hardware source immediately.  Compare rearm/stop decisions are
  // handoff-tier custody.
  ocxo_lane_clear_compare_flag(lane);

  // SpinIdle is an ISR-entry witness; capture it only after the truly
  // perishable hardware facts are already snapshotted.
  packet.spinidle = spinidle_capture_at_isr_entry(isr_entry_dwt_raw);
  packet.capture_exit_dwt = ARM_DWT_CYCCNT;

  interrupt_capture_ring_t<ocxo_capture_packet_t, HANDOFF_OCXO_RING_SIZE>& ring =
      (ctx.kind == interrupt_subscriber_kind_t::OCXO1)
          ? g_ocxo1_capture_ring
          : g_ocxo2_capture_ring;

  if (capture_ring_push_from_priority0(ring, diag, packet)) {
    interrupt_handoff_request_from_capture_isr(ctx.kind == interrupt_subscriber_kind_t::OCXO1
        ? "qtimer2_ocxo1_capture"
        : "qtimer3_ocxo2_capture");
  } else {
    lane.miss_count++;
  }
  interrupt_handoff_note_priority0_body(diag, isr_entry_dwt_raw);
}

static void interrupt_handoff_process_ocxo(ocxo_runtime_context_t& ctx,
                                           const ocxo_capture_packet_t& packet) {
  ocxo_lane_t& lane = *ctx.lane;

  ocxo_cadence_isr_sample_t sample{};
  sample.rt = ocxo_runtime_for(ctx);
  sample.isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  sample.spinidle = packet.spinidle;
  sample.isr_csctrl_entry = packet.csctrl_entry;
  sample.had_armed = packet.cadence_armed;
  sample.had_active_rt = packet.rt_active && packet.lane_active;
  sample.was_smartzero_current = smartzero_is_current_lane(ctx.kind);
  sample.target_counter32 = packet.target_counter32;
  sample.target_low16 = packet.target_low16;
  sample.service_counter_low16 = packet.service_counter_low16;
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
  sample.arm_remaining_ticks = packet.arm_remaining_ticks;
  sample.arm_to_isr_ticks =
      (uint32_t)((uint16_t)(sample.service_counter_low16 - packet.arm_low16));
  sample.arm_to_isr_dwt_cycles =
      sample.isr_entry_dwt_raw - packet.arm_dwt_raw;
  sample.event_dwt = qtimer_event_dwt_from_isr_entry_raw(sample.isr_entry_dwt_raw);

  if (sample.rt) sample.rt->irq_count++;
  spincatch_note_isr_entry(ctx.kind,
                           sample.target_counter32,
                           sample.target_low16,
                           sample.isr_entry_dwt_raw);

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

  ocxo_cadence_update_synthetic_identity(ctx, sample);
  ocxo_cadence_update_sample_phase(ctx, sample);

  if (sample.one_second_due && sample.had_active_rt) {
    isr_raw_dwt_probe_record(ctx.kind, sample.isr_entry_dwt_raw);
  }

  const bool cadence_reauthored_by_smartzero =
      ocxo_cadence_feed_smartzero(ctx, sample);
  ocxo_cadence_rearm_or_stop(ctx, sample, cadence_reauthored_by_smartzero);
  ocxo_cadence_update_one_second_witness(ctx, sample);

  if (sample.one_second_due && sample.had_active_rt) {
    interrupt_perishable_fact_t fact =
        ocxo_cadence_build_perishable_fact(ctx, sample);
    ocxo_cadence_enqueue_fact(ctx, fact);
  }

  lane.cadence_last_one_second_due = sample.one_second_due;
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_capture_priority0(g_ocxo1_ctx, isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_capture_priority0(g_ocxo2_ctx, isr_entry_dwt_raw);
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
  (void)target_counter32;
  g_qtimer1_ch1_arm_count++;
  g_qtimer1_ch1_active = false;
  return false;
}

void interrupt_qtimer1_ch1_disable_compare(void) {
  g_qtimer1_ch1_active = false;
  qtimer1_ch1_disable_compare_hw();
}

uint16_t interrupt_qtimer1_ch1_counter_now(void) { return IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CNTR; }
uint16_t interrupt_qtimer1_ch1_comp1_now(void)   { return IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].COMP1; }
uint16_t interrupt_qtimer1_ch1_csctrl_now(void)  { return IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CSCTRL; }

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

uint16_t interrupt_qtimer1_ch2_counter_now(void) { return IMXRT_TMR1.CH[QTIMER_CLOCK_COMPARE_CH].CNTR; }
uint16_t interrupt_qtimer1_ch2_comp1_now(void)   { return IMXRT_TMR1.CH[QTIMER_CLOCK_COMPARE_CH].COMP1; }
uint16_t interrupt_qtimer1_ch2_csctrl_now(void)  { return IMXRT_TMR1.CH[QTIMER_CLOCK_COMPARE_CH].CSCTRL; }

static void interrupt_handoff_process_qtimer1_ch1(
    const qtimer1_ch1_capture_packet_t& packet) {
  if (!packet.active_at_capture) {
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  const uint32_t fired_counter32 = packet.next_compare_counter32;

  if (fired_counter32 != packet.target_counter32) {
    g_qtimer1_ch1_hop_count++;
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  g_qtimer1_ch1_active = false;
  qtimer1_ch1_disable_compare_hw();
  g_qtimer1_ch1_fire_count++;

  const uint32_t qtimer_event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(packet.isr_entry_dwt_raw);
  const int64_t gnss_ns = vclock_gnss_from_counter32(fired_counter32);

  interrupt_qtimer1_ch1_compare_event_t event{};
  event.sequence = ++g_qtimer1_ch1_sequence;
  event.target_counter32 = packet.target_counter32;
  event.counter32_at_event = fired_counter32;
  event.counter32_residual_ticks =
      (int32_t)(fired_counter32 - packet.target_counter32);
  event.isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  event.dwt_at_event = qtimer_event_dwt;
  event.gnss_ns_at_event = gnss_ns;

  if (g_qtimer1_ch1_handler) {
    g_qtimer1_ch1_handler(event);
  }
}

static void interrupt_handoff_process_qtimer1_ch2(
    const qtimer1_ch2_capture_packet_t& packet) {
  const uint32_t isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const uint32_t ch2_event_counter32 = packet.ch2_event_counter32;
  const spinidle_isr_capture_t& spinidle = packet.spinidle;

  interrupt_ch2_implicit_rollover_tend();

  pps_relay_ch2_tick(g_vclock_clock32.current_counter32);
  vclock_ch2_one_second_service(isr_entry_dwt_raw,
                                qtimer_event_dwt,
                                ch2_event_counter32,
                                spinidle);
  vclock_ch2_smartzero_service(qtimer_event_dwt,
                               ch2_event_counter32);
  if (g_qtimer1_ch2_handler) {
    const bridge_projection_t bridge = interrupt_dwt_to_vclock_gnss_projection(qtimer_event_dwt);
    const int64_t  gnss_ns = bridge.gnss_ns;
    bridge_projection_record_stats(interrupt_subscriber_kind_t::TIMEPOP, bridge);

    interrupt_event_t event{};
    event.kind     = interrupt_subscriber_kind_t::TIMEPOP;
    event.provider = interrupt_provider_kind_t::QTIMER1;
    event.lane     = interrupt_lane_t::QTIMER1_CH1_COMP;
    event.status   = interrupt_event_status_t::OK;
    event.dwt_at_event       = qtimer_event_dwt;
    event.gnss_ns_at_event   = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
    event.counter32_at_event = ch2_event_counter32;

    interrupt_capture_diag_t diag{};
    diag.enabled  = true;
    diag.provider = interrupt_provider_kind_t::QTIMER1;
    diag.lane     = interrupt_lane_t::QTIMER1_CH1_COMP;
    diag.kind     = interrupt_subscriber_kind_t::TIMEPOP;
    diag.dwt_at_event       = qtimer_event_dwt;
    diag.gnss_ns_at_event   = event.gnss_ns_at_event;
    diag.counter32_at_event = event.counter32_at_event;
    diag.dwt_isr_entry_raw  = isr_entry_dwt_raw;
    diag.dwt_event_from_isr_entry_raw = qtimer_event_dwt;
    diag.dwt_isr_entry_to_event_correction_cycles =
        (int32_t)(qtimer_event_dwt - isr_entry_dwt_raw);
    diag.dwt_published_minus_event_cycles = 0;
    diag.dwt_used_minus_event_cycles = 0;
    spinidle_copy_to_diag(diag, spinidle);
    g_spinidle_timepop_last_capture = spinidle;
    bridge_projection_copy_to_diag(diag, bridge);

    g_qtimer1_ch2_handler(event, diag);
  }

  vclock_ch2_epoch_native_service(qtimer_event_dwt,
                                  ch2_event_counter32);
  vclock_ch2_fact_drain_arm_after_timepop();
}

static void qtimer1_ch2_capture_priority0(uint32_t isr_entry_dwt_raw,
                                            uint32_t csctrl_entry) {
  qtimer1_ch2_clear_compare_flag();

  qtimer1_ch2_capture_packet_t packet{};
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.ch2_event_counter32 = g_qtimer1_ch2_last_target_counter32;
  packet.csctrl_entry = csctrl_entry;

  // SpinIdle is diagnostic evidence. Capture it only after the indispensable
  // CH2 flag/target facts have been snapshotted and the source defused.
  packet.spinidle = spinidle_capture_at_isr_entry(isr_entry_dwt_raw);
  packet.capture_exit_dwt = ARM_DWT_CYCCNT;

  if (capture_ring_push_from_priority0(g_qtimer1_ch2_capture_ring,
                                       g_handoff_qtimer1_ch2,
                                       packet)) {
    interrupt_handoff_request_from_capture_isr("qtimer1_ch2_capture");
  }
  interrupt_handoff_note_priority0_body(g_handoff_qtimer1_ch2,
                                        isr_entry_dwt_raw);
}

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  // Uniform doctrine: QTimer1 CH1 is the sole VCLOCK/TimePop compare rail.
  const uint32_t primary_csctrl = IMXRT_TMR1.CH[QTIMER_CLOCK_COMPARE_CH].CSCTRL;
  if (primary_csctrl & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_capture_priority0(isr_entry_dwt_raw, primary_csctrl);
    return;
  }

  // Defensive defuse for the retired auxiliary CH2 rail.
  const uint32_t aux_csctrl = IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CSCTRL;
  if (aux_csctrl & TMR_CSCTRL_TCF1) {
    qtimer1_ch1_clear_compare_flag();
  }
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

static void pps_edge_dispatch_arm_or_call_from_foreground(const char* name) {
  if (!g_pps_edge_dispatch) return;

  const timepop_handle_t h =
      timepop_arm_asap(pps_edge_dispatch_trampoline,
                       nullptr,
                       name ? name : "PPS_VCLOCK_DISPATCH");
  if (h == TIMEPOP_INVALID_HANDLE) {
    // We are already in foreground/deferred context on the steady-state
    // VCLOCK publication path.  Dropping the dispatch here creates a missing
    // TIMEBASE row, so fall back to a direct callback instead of losing the
    // already-authored PPS/VCLOCK bookend.
    pps_edge_dispatch_trampoline(nullptr, nullptr, nullptr);
  }
}


static void pps_relay_assert_from_isr(uint32_t sequence) {
  // PPS relay HIGH is a physical PPS witness and stays in ISR context.
  // Relay LOW is intentionally *not* scheduled through TimePop.  Instead,
  // intrinsic QTimer1 CH1 service counts elapsed 1 ms VCLOCK cells and
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

static void interrupt_handoff_process_pps(const pps_capture_packet_t& packet) {
  const uint32_t isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  const spinidle_isr_capture_t& spinidle = packet.spinidle;
  g_spinidle_pps_last_capture = spinidle;

  const uint32_t epoch_capture_start_raw = isr_entry_dwt_raw;
  const uint16_t hardware_low16 = packet.vclock_hardware16_observed;
  const uint32_t epoch_capture_after_vclock_raw = packet.capture_after_vclock_raw;
  const bool ocxo_capture_hw_ready = packet.ocxo_capture_hw_ready;
  const uint16_t ocxo1_hardware16 = packet.ocxo1_hardware16;
  const uint16_t ocxo2_hardware16 = packet.ocxo2_hardware16;
  const uint32_t epoch_capture_end_raw = packet.capture_end_raw;

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

  const uint32_t ocxo1_counter32 = packet.ocxo1_capture_hw_ready
      ? project_counter32_from_hw16(g_ocxo1_clock32, ocxo1_hardware16)
      : 0;
  const uint32_t ocxo2_counter32 = packet.ocxo2_capture_hw_ready
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

    // Author the PPS yardstick pair for the OCXO yardstick inference rail.
    // observed_dwt_delta is the unquantized GPIO-path one-second interval;
    // observed_counter_delta qualifies it as one nominal VCLOCK second.
    pps_yardstick_record_interval(pps.sequence,
                                  observed_dwt_delta,
                                  observed_counter_delta);
  }
  g_isr_sanity_pps_prev_valid = true;
  g_isr_sanity_pps_prev_counter32 = observed_counter32;
  g_isr_sanity_pps_prev_dwt = pps.dwt_at_edge;

  pps_post_isr_defer(epoch_cap, isr_entry_dwt_raw);

  if (packet.rebootstrap_pending_at_capture && g_pps_rebootstrap_pending) {
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

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  pps_capture_packet_t packet{};
  packet.sequence = ++g_pps_capture_sequence;
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;

  packet.vclock_hardware16_observed = qtimer1_ch0_counter_now();
  packet.capture_after_vclock_raw = ARM_DWT_CYCCNT;

  packet.ocxo1_capture_hw_ready =
      g_interrupt_hw_ready && !OCXO1_DISABLED && g_ocxo1_lane.initialized;
  packet.ocxo2_capture_hw_ready =
      g_interrupt_hw_ready && !OCXO2_DISABLED && g_ocxo2_lane.initialized;
  packet.ocxo_capture_hw_ready =
      (OCXO1_DISABLED || packet.ocxo1_capture_hw_ready) &&
      (OCXO2_DISABLED || packet.ocxo2_capture_hw_ready);
  packet.ocxo1_hardware16 = packet.ocxo1_capture_hw_ready ? IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR : 0;
  packet.ocxo2_hardware16 = packet.ocxo2_capture_hw_ready ? IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CNTR : 0;
  packet.capture_end_raw = ARM_DWT_CYCCNT;
  packet.spinidle = spinidle_capture_at_isr_entry(isr_entry_dwt_raw);
  packet.capture_exit_dwt = ARM_DWT_CYCCNT;
  packet.rebootstrap_pending_at_capture = g_pps_rebootstrap_pending;

  if (capture_ring_push_from_priority0(g_pps_capture_ring,
                                       g_handoff_pps,
                                       packet)) {
    interrupt_handoff_request_from_capture_isr("pps_gpio_capture");
  } else {
    g_gpio_miss_count++;
  }
  interrupt_handoff_note_priority0_body(g_handoff_pps, isr_entry_dwt_raw);
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
  out.spinidle_shadow_valid = g_spinidle_pps_last_capture.shadow_valid;
  out.spinidle_shadow_dwt = g_spinidle_pps_last_capture.shadow_dwt;
  out.spinidle_shadow_to_isr_entry_cycles =
      g_spinidle_pps_last_capture.shadow_to_isr_entry_cycles;
  out.spinidle_shadow_valid_threshold_cycles =
      g_spinidle_pps_last_capture.threshold_cycles;

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
    vclock_lane_ema_reset(g_vclock_lane);
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
  ocxo_lane_yardstick_reset(*lane);

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
  vclock_lane_ema_reset(g_vclock_lane);
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

static void qtimer_init_count_compare_channel(IMXRT_TMR_t& module,
                                                uint8_t ch,
                                                uint8_t pcs) {
  module.CH[ch].CTRL   = 0;
  module.CH[ch].SCTRL  = 0;
  module.CH[ch].CSCTRL = 0;
  module.CH[ch].LOAD   = 0;
  module.CH[ch].CNTR   = 0;
  module.CH[ch].COMP1  = 0xFFFF;
  module.CH[ch].CMPLD1 = 0xFFFF;
  module.CH[ch].CMPLD2 = 0;
  module.CH[ch].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(pcs);
}

static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  qtimer_init_count_compare_channel(IMXRT_TMR1, QTIMER_CLOCK_COUNTER_CH, 0);
  qtimer_init_count_compare_channel(IMXRT_TMR1, QTIMER_CLOCK_COMPARE_CH, 0);

  // Uniform clock role: QTimer1 CH0 is the VCLOCK low-word counter; QTimer1
  // CH1 is the active compare rail used by TimePop through the legacy CH2 API.
  qtimer_disable_compare(IMXRT_TMR1, QTIMER_CLOCK_COMPARE_CH);

  IMXRT_TMR1.CH[QTIMER_CLOCK_COUNTER_CH].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[QTIMER_CLOCK_COUNTER_CH].CSCTRL |=  TMR_CSCTRL_TCF1;

  (void)IMXRT_TMR1.CH[QTIMER_CLOCK_COUNTER_CH].CNTR;
}

static void qtimer1_init_ch2_scheduler(void) {
  // Historical name retained; actual scheduler hardware is QTimer1 CH1.
  // Keep QTimer1 CH2 explicitly off so VCLOCK has only CH0 counter + CH1 compare.
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].SCTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CSCTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].LOAD = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].COMP1 = 0xFFFF;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CMPLD2 = 0;
}

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR2;
  g_ocxo1_lane.channel = QTIMER_CLOCK_COMPARE_CH;
  g_ocxo1_lane.counter_channel = QTIMER_CLOCK_COUNTER_CH;
  g_ocxo1_lane.compare_channel = QTIMER_CLOCK_COMPARE_CH;
  g_ocxo1_lane.pcs = QTIMER_CLOCK_PCS;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = QTIMER3_OCXO2_PHYSICAL_CH;
  g_ocxo2_lane.counter_channel = QTIMER3_OCXO2_PHYSICAL_CH;
  g_ocxo2_lane.compare_channel = QTIMER3_OCXO2_PHYSICAL_CH;
  g_ocxo2_lane.pcs = QTIMER3_OCXO2_PHYSICAL_PCS;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  g_pps_relay_pin_initialized = true;

  if (!OCXO1_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
    *(portConfigRegister(OCXO1_PIN)) = 1;
    #if defined(IOMUXC_QTIMER2_TIMER0_SELECT_INPUT)
    IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;
    #endif

    IMXRT_TMR2.ENBL &= ~(uint16_t)((1U << QTIMER_CLOCK_COUNTER_CH) |
                                   (1U << QTIMER_CLOCK_COMPARE_CH));
    qtimer_init_count_compare_channel(IMXRT_TMR2, QTIMER_CLOCK_COUNTER_CH,
                                      g_ocxo1_lane.pcs);
    qtimer_init_count_compare_channel(IMXRT_TMR2, QTIMER_CLOCK_COMPARE_CH,
                                      g_ocxo1_lane.pcs);
    ocxo_lane_disable_compare(g_ocxo1_lane);
    IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR = 0;
    IMXRT_TMR2.CH[QTIMER_CLOCK_COMPARE_CH].CNTR = 0;
    IMXRT_TMR2.ENBL |= (uint16_t)((1U << QTIMER_CLOCK_COUNTER_CH) |
                                  (1U << QTIMER_CLOCK_COMPARE_CH));
  }

  if (!OCXO2_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
    *(portConfigRegister(OCXO2_PIN)) = 1;
    #if defined(IOMUXC_QTIMER3_TIMER0_SELECT_INPUT)
    IOMUXC_QTIMER3_TIMER0_SELECT_INPUT = 0;
    #endif
    #if defined(IOMUXC_QTIMER3_TIMER3_SELECT_INPUT)
    IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;
    #endif

    IMXRT_TMR3.ENBL &= ~(uint16_t)((1U << QTIMER_CLOCK_COUNTER_CH) |
                                   (1U << QTIMER_CLOCK_COMPARE_CH) |
                                   (1U << QTIMER3_OCXO2_PHYSICAL_CH));

    // Defuse the failed uniform CH0/CH1 attempt explicitly.  Pin 15 does not
    // feed QTimer3 TIMER0 on this board; CH0/CH1 remain off for OCXO2.
    IMXRT_TMR3.CH[QTIMER_CLOCK_COUNTER_CH].CTRL = 0;
    IMXRT_TMR3.CH[QTIMER_CLOCK_COUNTER_CH].SCTRL = 0;
    IMXRT_TMR3.CH[QTIMER_CLOCK_COUNTER_CH].CSCTRL = 0;
    IMXRT_TMR3.CH[QTIMER_CLOCK_COMPARE_CH].CTRL = 0;
    IMXRT_TMR3.CH[QTIMER_CLOCK_COMPARE_CH].SCTRL = 0;
    IMXRT_TMR3.CH[QTIMER_CLOCK_COMPARE_CH].CSCTRL = 0;

    qtimer_init_count_compare_channel(IMXRT_TMR3,
                                      QTIMER3_OCXO2_PHYSICAL_CH,
                                      g_ocxo2_lane.pcs);
    ocxo_lane_disable_compare(g_ocxo2_lane);
    IMXRT_TMR3.CH[QTIMER3_OCXO2_PHYSICAL_CH].CNTR = 0;
    IMXRT_TMR3.ENBL |= (uint16_t)(1U << QTIMER3_OCXO2_PHYSICAL_CH);
  }

  g_ocxo1_lane.initialized = !OCXO1_DISABLED;
  g_ocxo2_lane.initialized = !OCXO2_DISABLED;

  // QTimer1 synchronized channel start: ENBL=0 first, configure CH0/CH1
  // fully, then a single ENBL write starts the VCLOCK counter and the uniform
  // CH1 TimePop compare rail together.
  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();
  g_vclock_lane.initialized = true;
  IMXRT_TMR1.CH[QTIMER_CLOCK_COUNTER_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER_CLOCK_COMPARE_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER1_AUX_COMPARE_CH].CNTR = 0;
  IMXRT_TMR1.ENBL = (uint16_t)((1U << QTIMER_CLOCK_COUNTER_CH) |
                               (1U << QTIMER_CLOCK_COMPARE_CH));

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
    synthetic_clock_bootstrap_from_hw16(g_ocxo1_clock32, IMXRT_TMR2.CH[QTIMER_CLOCK_COUNTER_CH].CNTR);
    g_ocxo1_lane.logical_count32_at_last_second = g_ocxo1_clock32.current_counter32;
  }
  if (!OCXO2_DISABLED && g_ocxo2_lane.initialized) {
    synthetic_clock_bootstrap_from_hw16(g_ocxo2_clock32, IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CNTR);
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
  vclock_lane_ema_reset(g_vclock_lane);
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
  pps_yardstick_reset();
  g_spinidle_pps_last_capture = spinidle_isr_capture_t{};
  g_spinidle_timepop_last_capture = spinidle_isr_capture_t{};
  isr_raw_dwt_probe_reset_all();

  g_interrupt_handoff = interrupt_handoff_diag_t{};
  g_handoff_qtimer1_ch1 = interrupt_handoff_source_diag_t{};
  g_handoff_qtimer1_ch2 = interrupt_handoff_source_diag_t{};
  g_handoff_ocxo1 = interrupt_handoff_source_diag_t{};
  g_handoff_ocxo2 = interrupt_handoff_source_diag_t{};
  g_handoff_pps = interrupt_handoff_source_diag_t{};
  g_qtimer1_ch1_capture_ring = interrupt_capture_ring_t<qtimer1_ch1_capture_packet_t, HANDOFF_QTIMER1_CH1_RING_SIZE>{};
  g_qtimer1_ch2_capture_ring = interrupt_capture_ring_t<qtimer1_ch2_capture_packet_t, HANDOFF_QTIMER1_CH2_RING_SIZE>{};
  g_ocxo1_capture_ring = interrupt_capture_ring_t<ocxo_capture_packet_t, HANDOFF_OCXO_RING_SIZE>{};
  g_ocxo2_capture_ring = interrupt_capture_ring_t<ocxo_capture_packet_t, HANDOFF_OCXO_RING_SIZE>{};
  g_pps_capture_ring = interrupt_capture_ring_t<pps_capture_packet_t, HANDOFF_PPS_RING_SIZE>{};
  g_interrupt_capture_sequence = 0;
  g_pps_capture_sequence = 0;

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
  spincatch_report_reset_all();
  vclock_fact_ring_reset();
  runtime_reset_ocxo_fact_rings();
  cadence_regression_reset_all();
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  runtime_reset_for_init();

  g_interrupt_runtime_ready = true;
  interrupt_handoff_configure();
  (void)vclock_heartbeat_arm_timepop();
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  interrupt_handoff_configure();

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

  void add_string(const char* suffix, const char* value) {
    make_key(suffix);
    p.add(key, value ? value : "");
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

static FLASHMEM void spincatch_plan_from_target(spincatch_lane_report_t& s,
                                       bool target_valid,
                                       uint32_t target_counter32,
                                       uint16_t target_low16,
                                       uint32_t now_counter32) {
  s.approach_ticks = SPINCATCH_APPROACH_TICKS;
  s.approach_us = SPINCATCH_APPROACH_US;
  s.now_counter32 = now_counter32;

  if (!target_valid) {
    s.planner_valid = false;
    s.target_counter32 = 0;
    s.target_low16 = 0;
    s.approach_counter32 = 0;
    s.approach_low16 = 0;
    s.approach_dwt_cycles = 0;
    s.ticks_until_target = 0;
    s.planner_skip_count++;
    return;
  }

  s.planner_valid = true;
  s.target_counter32 = target_counter32;
  s.target_low16 = target_low16;
  s.approach_counter32 = target_counter32 - SPINCATCH_APPROACH_TICKS;
  s.approach_low16 = (uint16_t)(target_low16 - (uint16_t)SPINCATCH_APPROACH_TICKS);
  s.approach_dwt_cycles = vclock_cycles_for_ticks(SPINCATCH_APPROACH_TICKS);
  s.ticks_until_target = target_counter32 - now_counter32;
  s.planner_count++;
}

static FLASHMEM void spincatch_update_planner_for_report(interrupt_subscriber_kind_t kind) {
  spincatch_lane_report_t* s = spincatch_report_for_kind(kind);
  if (!s || !SPINCATCH_PLANNER_ENABLED) return;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    const bool target_valid =
        g_vclock_ch2_one_second_enabled &&
        g_vclock_ch2_one_second_next_counter32 != 0;
    const uint32_t target = g_vclock_ch2_one_second_next_counter32;
    const uint16_t low16 = target_valid
        ? vclock_hardware_low16_from_synthetic(target)
        : 0U;
    const uint32_t now = interrupt_vclock_counter32_observe_ambient();
    spincatch_plan_from_target(*s, target_valid, target, low16, now);
    return;
  }

  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  if (!ctx || !ctx->lane || !ctx->clock32) {
    spincatch_plan_from_target(*s, false, 0, 0, 0);
    return;
  }

  const ocxo_lane_t& lane = *ctx->lane;
  const bool target_valid =
      lane.initialized &&
      lane.cadence_enabled &&
      lane.witness_target_initialized &&
      lane.cadence_next_counter32 != 0;
  spincatch_plan_from_target(*s,
                             target_valid,
                             lane.cadence_next_counter32,
                             target_valid ? lane.cadence_next_low16 : 0U,
                             ctx->clock32->current_counter32);
}

static FLASHMEM void add_spincatch_report_payload(Payload& p,
                                         interrupt_subscriber_kind_t kind) {
  spincatch_update_planner_for_report(kind);
  spincatch_lane_report_t* s_mut = spincatch_report_for_kind(kind);
  if (s_mut) spincatch_landing_refresh_pending(*s_mut, kind);
  const spincatch_lane_report_t* s = s_mut;

  p.add("spincatch_supported", s ? s->supported : false);
  p.add("spincatch_enabled", s ? s->enabled : false);
  p.add("spincatch_planner_enabled", s ? s->planner_enabled : false);
  p.add("spincatch_mode", SPINCATCH_REPORT_MODE);
  p.add("spincatch_idle_witness_shadow_dwt",
        (uint32_t)g_timepop_idle_witness_shadow_dwt);
  p.add("spincatch_idle_witness_age_cycles_at_report",
        g_timepop_idle_witness_shadow_dwt
            ? (uint32_t)(ARM_DWT_CYCCNT - g_timepop_idle_witness_shadow_dwt)
            : 0U);
  p.add("spincatch_shadow_valid_threshold_cycles",
        SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
  p.add("spincatch_success_count", s ? s->success_count : 0U);
  p.add("spincatch_missed_count", s ? s->missed_count : 0U);
  p.add("spincatch_timeout_count", s ? s->timeout_count : 0U);
  p.add("spincatch_clash_count", s ? s->clash_count : 0U);

  p.add("spincatch_planner_valid", s ? s->planner_valid : false);
  p.add("spincatch_planner_count", s ? s->planner_count : 0U);
  p.add("spincatch_planner_skip_count", s ? s->planner_skip_count : 0U);
  p.add("spincatch_target_counter32", s ? s->target_counter32 : 0U);
  p.add("spincatch_target_low16", s ? (uint32_t)s->target_low16 : 0U);
  p.add("spincatch_approach_counter32", s ? s->approach_counter32 : 0U);
  p.add("spincatch_approach_low16", s ? (uint32_t)s->approach_low16 : 0U);
  p.add("spincatch_approach_ticks", s ? s->approach_ticks : 0U);
  p.add("spincatch_approach_us", s ? s->approach_us : 0U);
  p.add("spincatch_approach_dwt_cycles", s ? s->approach_dwt_cycles : 0U);
  p.add("spincatch_now_counter32", s ? s->now_counter32 : 0U);
  p.add("spincatch_ticks_until_target", s ? s->ticks_until_target : 0U);

  p.add("spincatch_landing_only_enabled", s ? s->landing_only_enabled : false);
  p.add("spincatch_landing_pending", s ? s->landing_pending : false);
  p.add("spincatch_landing_last_success", s ? s->landing_last_success : false);
  p.add("spincatch_landing_last_target_changed", s ? s->landing_last_target_changed : false);
  p.add("spincatch_landing_duty_divisor", s ? s->landing_duty_divisor : 0U);
  p.add("spincatch_landing_duty_counter", s ? s->landing_duty_counter : 0U);
  p.add("spincatch_landing_schedule_count", s ? s->landing_schedule_count : 0U);
  p.add("spincatch_landing_schedule_skip_count", s ? s->landing_schedule_skip_count : 0U);
  p.add("spincatch_landing_schedule_fail_count", s ? s->landing_schedule_fail_count : 0U);
  p.add("spincatch_landing_count", s ? s->landing_count : 0U);
  p.add("spincatch_landing_too_close_count", s ? s->landing_too_close_count : 0U);
  p.add("spincatch_landing_target_changed_count", s ? s->landing_target_changed_count : 0U);
  p.add("spincatch_landing_attempt_ring_size", s ? s->landing_attempt_ring_size : 0U);
  p.add("spincatch_landing_attempt_active_count", s ? s->landing_attempt_active_count : 0U);
  p.add("spincatch_landing_attempt_stale_count", s ? s->landing_attempt_stale_count : 0U);
  p.add("spincatch_landing_attempt_duplicate_skip_count", s ? s->landing_attempt_duplicate_skip_count : 0U);
  p.add("spincatch_landing_attempt_ring_full_count", s ? s->landing_attempt_ring_full_count : 0U);
  p.add("spincatch_landing_attempt_generation", s ? s->landing_attempt_generation : 0U);
  p.add("spincatch_landing_target_counter32", s ? s->landing_target_counter32 : 0U);
  p.add("spincatch_landing_target_low16", s ? (uint32_t)s->landing_target_low16 : 0U);
  p.add("spincatch_landing_scheduled_counter32", s ? s->landing_scheduled_counter32 : 0U);
  p.add("spincatch_landing_scheduled_ticks_until_target", s ? s->landing_scheduled_ticks_until_target : 0U);
  p.add("spincatch_landing_scheduled_delay_ns", s ? s->landing_scheduled_delay_ns : 0ULL);
  p.add("spincatch_landing_scheduled_dwt", s ? s->landing_scheduled_dwt : 0U);
  p.add("spincatch_landing_dwt", s ? s->landing_dwt : 0U);
  p.add("spincatch_landing_counter32", s ? s->landing_counter32 : 0U);
  p.add("spincatch_landing_offset_ticks", s ? s->landing_offset_ticks : 0);
  p.add("spincatch_landing_ticks_until_target", s ? s->landing_ticks_until_target : 0U);
  p.add("spincatch_landing_fire_vclock_raw", s ? s->landing_fire_vclock_raw : 0U);
  p.add("spincatch_landing_fire_dwt", s ? s->landing_fire_dwt : 0U);
}

static FLASHMEM void add_isr_sanity_payload(Payload& p,
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

static FLASHMEM void add_bridge_stats_payload(Payload& p,
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

static FLASHMEM void add_regression_payload(Payload& p,
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

static FLASHMEM interrupt_subscriber_kind_t regression_kind_from_lane_arg(const char* lane) {
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


static FLASHMEM bool isr_raw_dwt_probe_snapshot(
    interrupt_subscriber_kind_t kind,
    isr_raw_dwt_interval_probe_t& out) {
  const isr_raw_dwt_interval_probe_t* src = isr_raw_dwt_probe_for_const(kind);
  if (!src) {
    out = isr_raw_dwt_interval_probe_t{};
    return false;
  }

  uint32_t primask = 0;
  __asm__ volatile ("mrs %0, primask" : "=r" (primask) :: "memory");
  __disable_irq();
  out = *src;
  if ((primask & 1U) == 0U) {
    __enable_irq();
  }

  return true;
}

static FLASHMEM uint32_t isr_raw_dwt_probe_chrono_index(
    const isr_raw_dwt_interval_probe_t& s,
    uint32_t ordinal) {
  if (s.interval_count == 0) return 0;
  const uint32_t start =
      (s.write_index + ISR_RAW_DWT_INTERVAL_RING_SIZE - s.interval_count) %
      ISR_RAW_DWT_INTERVAL_RING_SIZE;
  return (start + ordinal) % ISR_RAW_DWT_INTERVAL_RING_SIZE;
}

static FLASHMEM void isr_raw_dwt_probe_append_csv_u32(char* buf,
                                             size_t len,
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

static FLASHMEM void isr_raw_dwt_probe_append_csv_i32(char* buf,
                                             size_t len,
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

static FLASHMEM void isr_raw_dwt_probe_add_summary(Payload& p,
                                          const isr_raw_dwt_interval_probe_t& s,
                                          bool include_csv) {
  const uint32_t n = s.interval_count;

  double mean = 0.0;
  double m2 = 0.0;
  uint32_t min_v = UINT32_MAX;
  uint32_t max_v = 0;

  double fd_mean = 0.0;
  double fd_m2 = 0.0;
  int32_t fd_min = INT32_MAX;
  int32_t fd_max = INT32_MIN;
  uint32_t fd_n = 0;
  uint32_t prev = 0;

  static char interval_csv[2048];
  static char first_difference_csv[1536];
  if (include_csv) {
    interval_csv[0] = '\0';
    first_difference_csv[0] = '\0';
  }
  uint32_t interval_used = 0;
  uint32_t fd_used = 0;

  for (uint32_t i = 0; i < n; i++) {
    const uint32_t idx = isr_raw_dwt_probe_chrono_index(s, i);
    const uint32_t v = s.intervals[idx];

    if (v < min_v) min_v = v;
    if (v > max_v) max_v = v;

    const double d1 = (double)v - mean;
    mean += d1 / (double)(i + 1U);
    const double d2 = (double)v - mean;
    m2 += d1 * d2;

    if (include_csv) {
      isr_raw_dwt_probe_append_csv_u32(interval_csv,
                                       sizeof(interval_csv),
                                       interval_used,
                                       v);
    }

    if (i > 0) {
      const int32_t fd = (int32_t)((int64_t)v - (int64_t)prev);
      if (fd < fd_min) fd_min = fd;
      if (fd > fd_max) fd_max = fd;
      fd_n++;

      const double fd_d1 = (double)fd - fd_mean;
      fd_mean += fd_d1 / (double)fd_n;
      const double fd_d2 = (double)fd - fd_mean;
      fd_m2 += fd_d1 * fd_d2;

      if (include_csv) {
        isr_raw_dwt_probe_append_csv_i32(first_difference_csv,
                                         sizeof(first_difference_csv),
                                         fd_used,
                                         fd);
      }
    }

    prev = v;
  }

  const double variance = (n >= 2U) ? (m2 / (double)(n - 1U)) : 0.0;
  const double stddev = sqrt(variance < 0.0 ? 0.0 : variance);
  const double se = (n > 0) ? (stddev / sqrt((double)n)) : 0.0;

  const double fd_variance =
      (fd_n >= 2U) ? (fd_m2 / (double)(fd_n - 1U)) : 0.0;
  const double fd_stddev = sqrt(fd_variance < 0.0 ? 0.0 : fd_variance);
  const double fd_stderr = (fd_n > 0) ? (fd_stddev / sqrt((double)fd_n)) : 0.0;

  p.add("endpoint_count", s.endpoint_count);
  p.add("interval_count", n);
  p.add("interval_capacity", ISR_RAW_DWT_INTERVAL_RING_SIZE);
  p.add("write_index", s.write_index);
  p.add("last_dwt_valid", s.last_dwt_valid);
  p.add("last_isr_entry_dwt_raw", s.last_isr_entry_dwt_raw);
  p.add("last_interval_valid", s.last_interval_valid);
  p.add("last_interval_cycles", s.last_interval_cycles);
  p.add("last_interval_delta_cycles", s.last_interval_delta_cycles);

  p.add("interval_mean_cycles", (double)((n > 0) ? mean : 0.0));
  p.add("interval_stddev_cycles", (double)((n >= 2U) ? stddev : 0.0));
  p.add("interval_stderr_cycles", (double)((n > 0) ? se : 0.0));
  p.add("interval_min_cycles", (n > 0) ? min_v : 0U);
  p.add("interval_max_cycles", (n > 0) ? max_v : 0U);

  p.add("first_difference_count", fd_n);
  p.add("first_difference_mean_cycles", (double)((fd_n > 0) ? fd_mean : 0.0));
  p.add("first_difference_stddev_cycles", (double)((fd_n >= 2U) ? fd_stddev : 0.0));
  p.add("first_difference_stderr_cycles", (double)((fd_n > 0) ? fd_stderr : 0.0));
  p.add("first_difference_min_cycles", (fd_n > 0) ? fd_min : 0);
  p.add("first_difference_max_cycles", (fd_n > 0) ? fd_max : 0);

  if (include_csv) {
    p.add("sample_encoding", "csv_chronological_oldest_to_newest");
    p.add("interval_cycles_csv", interval_csv);
    p.add("first_difference_cycles_csv", first_difference_csv);
  }
}

static FLASHMEM Payload cmd_report_ocxo_isr_raw_dwt(const Payload& args) {
  const char* lane_arg = args.getString("lane");
  if (!lane_arg || !*lane_arg) lane_arg = args.getString("name");

  Payload p;
  p.add("report", "INTERRUPT_OCXO_ISR_RAW_DWT");
  p.add("schema", "ISR_RAW_DWT_INTERVAL_RING_V1");
  p.add("doctrine", "literal_first_instruction_isr_entry_dwt_intervals_no_correction_no_projection_no_ema");
  p.add("usage", "INTERRUPT.REPORT_OCXO_ISR_RAW_DWT lane=VCLOCK|OCXO1|OCXO2");
  p.add("ring_capacity", ISR_RAW_DWT_INTERVAL_RING_SIZE);

  const bool want_all =
      !lane_arg || !*lane_arg || !strcasecmp(lane_arg, "ALL");
  if (want_all) {
    p.add("lane", "ALL");
    p.add("note", "lane=ALL returns compact summaries only; request one lane for CSV samples");

    Payload vclock;
    isr_raw_dwt_interval_probe_t v{};
    (void)isr_raw_dwt_probe_snapshot(interrupt_subscriber_kind_t::VCLOCK, v);
    vclock.add("lane", "VCLOCK");
    isr_raw_dwt_probe_add_summary(vclock, v, false);
    p.add_object("vclock", vclock);

    Payload ocxo1;
    isr_raw_dwt_interval_probe_t o1{};
    (void)isr_raw_dwt_probe_snapshot(interrupt_subscriber_kind_t::OCXO1, o1);
    ocxo1.add("lane", "OCXO1");
    isr_raw_dwt_probe_add_summary(ocxo1, o1, false);
    p.add_object("ocxo1", ocxo1);

    Payload ocxo2;
    isr_raw_dwt_interval_probe_t o2{};
    (void)isr_raw_dwt_probe_snapshot(interrupt_subscriber_kind_t::OCXO2, o2);
    ocxo2.add("lane", "OCXO2");
    isr_raw_dwt_probe_add_summary(ocxo2, o2, false);
    p.add_object("ocxo2", ocxo2);
    return p;
  }

  const interrupt_subscriber_kind_t kind = regression_kind_from_lane_arg(lane_arg);
  if (kind != interrupt_subscriber_kind_t::VCLOCK &&
      kind != interrupt_subscriber_kind_t::OCXO1 &&
      kind != interrupt_subscriber_kind_t::OCXO2) {
    p.add("error", "unknown lane");
    p.add("lane", lane_arg ? lane_arg : "");
    return p;
  }

  isr_raw_dwt_interval_probe_t snapshot{};
  const bool ok = isr_raw_dwt_probe_snapshot(kind, snapshot);

  p.add("lane", interrupt_subscriber_kind_str(kind));
  p.add("snapshot_ok", ok);
  p.add("source_field", "ARM_DWT_CYCCNT captured as first instruction of the lane's one-second ISR service");
  p.add("vclock_note", "VCLOCK records only native one-second CH2 service facts, not every 1ms TimePop fire");
  p.add("ocxo_note", "OCXO records only active steady-state one-second facts, not SmartZero 1kHz acquisition samples");
  isr_raw_dwt_probe_add_summary(p, snapshot, true);
  return p;
}


// ============================================================================
// Commands
// ============================================================================

static FLASHMEM Payload cmd_report_qtimer_regs(const Payload& args) {
  Payload p;
  p.add("report", "INTERRUPT_QTIMER_REGS");
  p.add("format", "compact_lane_flat_raw_u32_v3");
  p.add("hex", false);
  p.add("doctrine", QTIMER_UNIFORM_DOCTRINE);

  const char* lane_arg = args.getString("lane");
  if (!lane_arg) lane_arg = "";

  const interrupt_subscriber_kind_t lane = regression_kind_from_lane_arg(lane_arg);
  const bool want_all = (!lane_arg || !*lane_arg || !strcasecmp(lane_arg, "ALL"));

  if (want_all) {
    p.add("usage", "INTERRUPT.REPORT_QTIMER_REGS lane=VCLOCK|OCXO1|OCXO2");
    p.add("note", "Compact default intentionally omits register dumps; request one lane to avoid payload/transport limits");
    p.add("vclock", "QTIMER1 CH0 counter / CH1 compare");
    p.add("ocxo1", "QTIMER2 CH0 counter / CH1 compare");
    p.add("ocxo2", "QTIMER3 CH3 physical adapter on pin15; CH0/CH1 unavailable because pin19 is SMBus SCL");
    p.add("vclock_ready", g_interrupt_hw_ready);
    p.add("ocxo1_ready", !OCXO1_DISABLED && g_ocxo1_lane.initialized);
    p.add("ocxo2_ready", !OCXO2_DISABLED && g_ocxo2_lane.initialized);
    return p;
  }

  if (lane == interrupt_subscriber_kind_t::VCLOCK) {
    p.add("lane", "VCLOCK");
    p.add("module", "QTIMER1");
    p.add("ready", g_interrupt_hw_ready);
    p.add("counter_ch", 0U);
    p.add("compare_ch", 1U);
    p.add("pcs", 0U);
    p.add("enbl", (uint32_t)IMXRT_TMR1.ENBL);

    p.add("counter_CTRL",   (uint32_t)IMXRT_TMR1.CH[0].CTRL);
    p.add("counter_SCTRL",  (uint32_t)IMXRT_TMR1.CH[0].SCTRL);
    p.add("counter_CSCTRL", (uint32_t)IMXRT_TMR1.CH[0].CSCTRL);
    p.add("counter_LOAD",   (uint32_t)IMXRT_TMR1.CH[0].LOAD);
    p.add("counter_CNTR",   (uint32_t)IMXRT_TMR1.CH[0].CNTR);
    p.add("counter_COMP1",  (uint32_t)IMXRT_TMR1.CH[0].COMP1);
    p.add("counter_CMPLD1", (uint32_t)IMXRT_TMR1.CH[0].CMPLD1);
    p.add("counter_CMPLD2", (uint32_t)IMXRT_TMR1.CH[0].CMPLD2);

    p.add("compare_CTRL",   (uint32_t)IMXRT_TMR1.CH[1].CTRL);
    p.add("compare_SCTRL",  (uint32_t)IMXRT_TMR1.CH[1].SCTRL);
    p.add("compare_CSCTRL", (uint32_t)IMXRT_TMR1.CH[1].CSCTRL);
    p.add("compare_LOAD",   (uint32_t)IMXRT_TMR1.CH[1].LOAD);
    p.add("compare_CNTR",   (uint32_t)IMXRT_TMR1.CH[1].CNTR);
    p.add("compare_COMP1",  (uint32_t)IMXRT_TMR1.CH[1].COMP1);
    p.add("compare_CMPLD1", (uint32_t)IMXRT_TMR1.CH[1].CMPLD1);
    p.add("compare_CMPLD2", (uint32_t)IMXRT_TMR1.CH[1].CMPLD2);

    p.add("pin_mux", (uint32_t)(*portConfigRegister(10)));
    p.add("public_api_note", "TimePop compatibility API name may still say CH2; hardware doctrine reports CH1 compare");
    return p;
  }

  if (lane == interrupt_subscriber_kind_t::OCXO1) {
    p.add("lane", "OCXO1");
    p.add("module", "QTIMER2");
    p.add("ready", !OCXO1_DISABLED && g_ocxo1_lane.initialized);
    p.add("counter_ch", 0U);
    p.add("compare_ch", 1U);
    p.add("pcs", (uint32_t)g_ocxo1_lane.pcs);
    p.add("enbl", (uint32_t)IMXRT_TMR2.ENBL);

    p.add("counter_CTRL",   (uint32_t)IMXRT_TMR2.CH[0].CTRL);
    p.add("counter_SCTRL",  (uint32_t)IMXRT_TMR2.CH[0].SCTRL);
    p.add("counter_CSCTRL", (uint32_t)IMXRT_TMR2.CH[0].CSCTRL);
    p.add("counter_LOAD",   (uint32_t)IMXRT_TMR2.CH[0].LOAD);
    p.add("counter_CNTR",   (uint32_t)IMXRT_TMR2.CH[0].CNTR);
    p.add("counter_COMP1",  (uint32_t)IMXRT_TMR2.CH[0].COMP1);
    p.add("counter_CMPLD1", (uint32_t)IMXRT_TMR2.CH[0].CMPLD1);
    p.add("counter_CMPLD2", (uint32_t)IMXRT_TMR2.CH[0].CMPLD2);

    p.add("compare_CTRL",   (uint32_t)IMXRT_TMR2.CH[1].CTRL);
    p.add("compare_SCTRL",  (uint32_t)IMXRT_TMR2.CH[1].SCTRL);
    p.add("compare_CSCTRL", (uint32_t)IMXRT_TMR2.CH[1].CSCTRL);
    p.add("compare_LOAD",   (uint32_t)IMXRT_TMR2.CH[1].LOAD);
    p.add("compare_CNTR",   (uint32_t)IMXRT_TMR2.CH[1].CNTR);
    p.add("compare_COMP1",  (uint32_t)IMXRT_TMR2.CH[1].COMP1);
    p.add("compare_CMPLD1", (uint32_t)IMXRT_TMR2.CH[1].CMPLD1);
    p.add("compare_CMPLD2", (uint32_t)IMXRT_TMR2.CH[1].CMPLD2);

    p.add("pin_mux", (uint32_t)(*portConfigRegister(OCXO1_PIN)));
#if defined(IOMUXC_QTIMER2_TIMER0_SELECT_INPUT)
    p.add("timer0_select_input", (uint32_t)IOMUXC_QTIMER2_TIMER0_SELECT_INPUT);
#endif
    return p;
  }

  if (lane == interrupt_subscriber_kind_t::OCXO2) {
    p.add("lane", "OCXO2");
    p.add("module", "QTIMER3");
    p.add("ready", !OCXO2_DISABLED && g_ocxo2_lane.initialized);
    p.add("physical_adapter", true);
    p.add("adapter_reason", "pin15 routes to QTimer3 TIMER3; pin19/TIMER0 is SMBus SCL");
    p.add("counter_ch", (uint32_t)g_ocxo2_lane.counter_channel);
    p.add("compare_ch", (uint32_t)g_ocxo2_lane.compare_channel);
    p.add("pcs", (uint32_t)g_ocxo2_lane.pcs);
    p.add("enbl", (uint32_t)IMXRT_TMR3.ENBL);

    p.add("counter_CTRL",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CTRL);
    p.add("counter_SCTRL",  (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].SCTRL);
    p.add("counter_CSCTRL", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CSCTRL);
    p.add("counter_LOAD",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].LOAD);
    p.add("counter_CNTR",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CNTR);
    p.add("counter_COMP1",  (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].COMP1);
    p.add("counter_CMPLD1", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CMPLD1);
    p.add("counter_CMPLD2", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.counter_channel].CMPLD2);

    p.add("compare_CTRL",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].CTRL);
    p.add("compare_SCTRL",  (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].SCTRL);
    p.add("compare_CSCTRL", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].CSCTRL);
    p.add("compare_LOAD",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].LOAD);
    p.add("compare_CNTR",   (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].CNTR);
    p.add("compare_COMP1",  (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].COMP1);
    p.add("compare_CMPLD1", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].CMPLD1);
    p.add("compare_CMPLD2", (uint32_t)IMXRT_TMR3.CH[g_ocxo2_lane.compare_channel].CMPLD2);

    p.add("pin_mux", (uint32_t)(*portConfigRegister(OCXO2_PIN)));
#if defined(IOMUXC_QTIMER3_TIMER0_SELECT_INPUT)
    p.add("timer0_select_input", (uint32_t)IOMUXC_QTIMER3_TIMER0_SELECT_INPUT);
#endif
#if defined(IOMUXC_QTIMER3_TIMER3_SELECT_INPUT)
    p.add("timer3_select_input", (uint32_t)IOMUXC_QTIMER3_TIMER3_SELECT_INPUT);
#endif
    return p;
  }

  p.add("error", "unknown lane");
  p.add("lane", lane_arg ? lane_arg : "");
  p.add("usage", "INTERRUPT.REPORT_QTIMER_REGS lane=VCLOCK|OCXO1|OCXO2");
  return p;
}

static FLASHMEM void add_priority_payload(Payload& p) {
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
  p.add("capture_priority", INTERRUPT_PRIORITY_CAPTURE);
  p.add("handoff_priority", INTERRUPT_PRIORITY_HANDOFF);
  p.add("background_irq_priority", INTERRUPT_PRIORITY_BACKGROUND_IRQ);
  p.add("handoff_irq", INTERRUPT_HANDOFF_IRQ_NUMBER);
  p.add("handoff_irq_name", INTERRUPT_HANDOFF_IRQ_NAME);
}

static FLASHMEM void add_runtime_payload(Payload& p) {
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("reduced_diagnostics_enabled", INTERRUPT_REDUCED_DIAGNOSTICS);
  p.add("regression_retained_sample_count", REGRESSION_RETAINED_SAMPLE_COUNT);
  p.add("spincatch_diagnostics_retired", !SPINCATCH_OCXO_LANDING_ONLY_ENABLED);
  p.add("ocxo_disable_experiment", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_disabled_count", OCXO_DISABLED_COUNT);
  p.add("ocxo1_disabled", OCXO1_DISABLED);
  p.add("ocxo2_disabled", OCXO2_DISABLED);
  p.add("ocxo1_irq_qtimer2_disabled", OCXO1_DISABLED);
  p.add("ocxo2_irq_qtimer3_disabled", OCXO2_DISABLED);
  p.add("ocxo_smartzero_surrogates_enabled", OCXO_DISABLE_EXPERIMENT);
  p.add("ocxo_quiet_phase_sampling_enabled", OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("ocxo_rollover_only_mode", !OCXO_QUIET_PHASE_SAMPLING_ENABLED);
  p.add("vclock_ema_dwt_authority_enabled", VCLOCK_EMA_DWT_AUTHORITY_ENABLED);
  p.add("vclock_ema_alpha_numerator", VCLOCK_EMA_ALPHA_NUMERATOR);
  p.add("vclock_ema_alpha_denominator", VCLOCK_EMA_ALPHA_DENOMINATOR);
  p.add("vclock_ema_audit_threshold_cycles", VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES);
  p.add("ocxo_ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  p.add("ocxo_ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  p.add("ocxo_yardstick_dwt_authority_enabled", OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_yardstick_gate_threshold_cycles", OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES);
  p.add("ocxo_yardstick_anchor_p_shift", OCXO_YARDSTICK_ANCHOR_P_SHIFT);
  p.add("ocxo_yardstick_anchor_i_shift", OCXO_YARDSTICK_ANCHOR_I_SHIFT);
  p.add("ocxo_yardstick_anchor_fast_lock_seconds",
        OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS);
  p.add("ocxo_yardstick_anchor_fast_i_shift",
        OCXO_YARDSTICK_ANCHOR_FAST_I_SHIFT);
  p.add("smartzero2_selection_authority_enabled",
        SMARTZERO2_SELECTION_AUTHORITY_ENABLED);
  p.add("zero_worthy_streak_seconds", ZERO_WORTHY_STREAK_SECONDS);
  p.add("zero_worthy_max_abs_auth_error_cycles",
        ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES);
  p.add("zero_worthy_max_abs_error_sum_cycles",
        ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES);
  p.add("ocxo1_quiet_phase_ticks", OCXO1_QUIET_PHASE_TICKS);
  p.add("ocxo1_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO1_QUIET_PHASE_TICKS));
  p.add("ocxo2_quiet_phase_ticks", OCXO2_QUIET_PHASE_TICKS);
  p.add("ocxo2_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO2_QUIET_PHASE_TICKS));
  p.add("timing_arch_step", "UNIFORM_CH0_COUNTER_CH1_COMPARE_WITH_OCXO2_PIN15_CH3_ADAPTER");
  p.add("timing_arch_behavior_changed", true);
  p.add("isr_sanity_witness_enabled", true);
  p.add("isr_sanity_witness_policy", "REPORT_ONLY_NO_REPAIR_NO_VETO");
  p.add("timing_arch_vclock_authority", "QTIMER1_CH0_COUNTER_CH1_COMPARE_ROLLOVER_RELAY_EPOCH_1S_SMARTZERO_DRAIN_ARM");
  p.add("timing_arch_ocxo_model", "OCXO1_CH0_CH1_OCXO2_PIN15_QTIMER3_CH3_PHYSICAL_ADAPTER");
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
  p.add("reduced_diagnostics_enabled", INTERRUPT_REDUCED_DIAGNOSTICS);
  p.add("regression_retained_sample_count", REGRESSION_RETAINED_SAMPLE_COUNT);
  p.add("spincatch_diagnostics_retired", !SPINCATCH_OCXO_LANDING_ONLY_ENABLED);
  p.add("capture_discipline", INTERRUPT_CAPTURE_DISCIPLINE);
  p.add("handoff_mechanism", INTERRUPT_HANDOFF_MECHANISM);
  p.add("handoff_irq", INTERRUPT_HANDOFF_IRQ_NUMBER);
  p.add("handoff_priority", INTERRUPT_PRIORITY_HANDOFF);
  p.add("handoff_report_command", "INTERRUPT.REPORT_HANDOFF");
  p.add("ocxo_isr_raw_dwt_report_command", "INTERRUPT.REPORT_OCXO_ISR_RAW_DWT lane=VCLOCK|OCXO1|OCXO2");
}

static FLASHMEM void add_ocxo_cadence_report_payload(Payload& p,
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

static FLASHMEM void add_vclock_heartbeat_payload(Payload& p) {
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
  p.add("vclock_ema_dwt_authority_enabled", VCLOCK_EMA_DWT_AUTHORITY_ENABLED);
  p.add("vclock_ema_alpha_numerator", VCLOCK_EMA_ALPHA_NUMERATOR);
  p.add("vclock_ema_alpha_denominator", VCLOCK_EMA_ALPHA_DENOMINATOR);
  p.add("vclock_ema_audit_threshold_cycles", VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES);
  p.add("ocxo_ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  p.add("ocxo_ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  p.add("ocxo_yardstick_dwt_authority_enabled", OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED);
  p.add("ocxo_yardstick_gate_threshold_cycles", OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES);
  p.add("ocxo_yardstick_anchor_p_shift", OCXO_YARDSTICK_ANCHOR_P_SHIFT);
  p.add("ocxo_yardstick_anchor_i_shift", OCXO_YARDSTICK_ANCHOR_I_SHIFT);
  p.add("ocxo_yardstick_anchor_fast_lock_seconds",
        OCXO_YARDSTICK_ANCHOR_FAST_LOCK_SECONDS);
  p.add("ocxo_yardstick_anchor_fast_i_shift",
        OCXO_YARDSTICK_ANCHOR_FAST_I_SHIFT);
  p.add("smartzero2_selection_authority_enabled",
        SMARTZERO2_SELECTION_AUTHORITY_ENABLED);
  p.add("zero_worthy_streak_seconds", ZERO_WORTHY_STREAK_SECONDS);
  p.add("zero_worthy_max_abs_auth_error_cycles",
        ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES);
  p.add("zero_worthy_max_abs_error_sum_cycles",
        ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES);
  p.add("ocxo_quiet_phase_period_ticks", OCXO_QUIET_PHASE_PERIOD_TICKS);
  p.add("ocxo1_quiet_phase_ticks", OCXO1_QUIET_PHASE_TICKS);
  p.add("ocxo1_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO1_QUIET_PHASE_TICKS));
  p.add("ocxo2_quiet_phase_ticks", OCXO2_QUIET_PHASE_TICKS);
  p.add("ocxo2_quiet_phase_us", ocxo_phase_ticks_to_us(OCXO2_QUIET_PHASE_TICKS));
  add_ocxo_cadence_report_payload(p, g_ocxo1_ctx);
  add_ocxo_cadence_report_payload(p, g_ocxo2_ctx);
}
static FLASHMEM void add_pps_payload(Payload& p) {
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);
  p.add("pps_spinidle_shadow_valid", g_spinidle_pps_last_capture.shadow_valid);
  p.add("pps_spinidle_shadow_dwt", g_spinidle_pps_last_capture.shadow_dwt);
  p.add("pps_spinidle_shadow_to_isr_entry_cycles",
        g_spinidle_pps_last_capture.shadow_to_isr_entry_cycles);
  p.add("pps_spinidle_shadow_valid_threshold_cycles",
        g_spinidle_pps_last_capture.threshold_cycles);
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

static FLASHMEM void add_epoch_capture_payload(Payload& p) {
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

static FLASHMEM void add_dynamic_cps_payload(Payload& p) {
  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  p.add("dynamic_cps_owner", "CLOCKS_STATIC_PPS");
  p.add("dynamic_cps", dynamic_cps);
  p.add("dynamic_cps_valid", dynamic_cps != 0);
  p.add("dynamic_cps_pps_sequence", g_pps_gpio_heartbeat.edge_count);
  p.add("dynamic_cps_last_pvc_dwt_at_edge", g_pps_gpio_heartbeat.last_dwt);
  p.add("dynamic_cps_last_reseed_value", dynamic_cps);
  p.add("dynamic_cps_last_reseed_was_computed", dynamic_cps != 0);
}


static FLASHMEM void add_smartzero_lane_payload(Payload& parent,
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

static FLASHMEM void add_smartzero_payload(Payload& p) {
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

static FLASHMEM void add_vclock_clock32_payload(Payload& p, const char* prefix) {
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
static FLASHMEM void add_ocxo_clock32_payload(Payload& p,
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
static FLASHMEM void add_runtime_lane_summary(Payload& p,
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
    out.add_bool("last_diag_dwt_synthetic", rt->last_diag.dwt_synthetic);
    out.add_u32("last_diag_dwt_original_at_event", rt->last_diag.dwt_original_at_event);
    out.add_u32("last_diag_dwt_predicted_at_event", rt->last_diag.dwt_predicted_at_event);
    out.add_u32("last_diag_dwt_used_at_event", rt->last_diag.dwt_used_at_event);
    out.add_u32("last_diag_dwt_isr_entry_raw", rt->last_diag.dwt_isr_entry_raw);
    out.add_i32("last_diag_dwt_synthetic_error_cycles", rt->last_diag.dwt_synthetic_error_cycles);
    out.add_bool("last_diag_spinidle_shadow_valid", rt->last_diag.spinidle_shadow_valid);
    out.add_u32("last_diag_spinidle_shadow_dwt", rt->last_diag.spinidle_shadow_dwt);
    out.add_u32("last_diag_spinidle_shadow_to_isr_entry_cycles",
                rt->last_diag.spinidle_shadow_to_isr_entry_cycles);
    out.add_u32("last_diag_spinidle_shadow_valid_threshold_cycles",
                rt->last_diag.spinidle_shadow_valid_threshold_cycles);
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
    out.add_bool("last_diag_dwt_synthetic", false);
    out.add_u32("last_diag_dwt_original_at_event", 0);
    out.add_u32("last_diag_dwt_predicted_at_event", 0);
    out.add_u32("last_diag_dwt_used_at_event", 0);
    out.add_u32("last_diag_dwt_isr_entry_raw", 0);
    out.add_i32("last_diag_dwt_synthetic_error_cycles", 0);
    out.add_bool("last_diag_spinidle_shadow_valid", false);
    out.add_u32("last_diag_spinidle_shadow_dwt", 0);
    out.add_u32("last_diag_spinidle_shadow_to_isr_entry_cycles", 0);
    out.add_u32("last_diag_spinidle_shadow_valid_threshold_cycles",
                SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
    out.add_u32("last_diag_anchor_selection_kind", 0);
    out.add_u32("last_diag_anchor_failure_mask", 0);
  }
}
static FLASHMEM void add_ocxo_identity_payload(Payload& p,
                                      const ocxo_runtime_context_t& ctx) {
  p.add("lane", ctx.name);
  p.add("kind", ctx.name);
  p.add("provider", interrupt_provider_kind_str(ctx.provider));
  p.add("hardware_lane", interrupt_lane_str(ctx.lane_id));
  p.add("cadence_source", ctx.cadence_source);
  p.add("counter_source", ctx.counter_source);
  p.add("event_source", "LOCAL_ONE_SECOND_EDGE_COMPARE_EMA_DWT");
  p.add("dwt_authority", ctx.dwt_authority);
}

static FLASHMEM void add_ocxo_lane_basic_payload(payload_prefix_t& out,
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

static FLASHMEM void add_ocxo_witness_service_payload(payload_prefix_t& out,
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
  out.add_bool("ema_last_adjacency_gate_valid",
               lane.ema_last_adjacency_gate_valid);
  out.add_bool("ema_last_adjacency_ok", lane.ema_last_adjacency_ok);
  out.add_bool("ema_last_adjacency_rejected",
               lane.ema_last_adjacency_rejected);
  out.add_u32("ema_last_counter_delta_ticks",
              lane.ema_last_counter_delta_ticks);
  out.add_u32("ema_expected_counter_delta_ticks",
              lane.ema_expected_counter_delta_ticks);
  out.add_u32("ema_adjacency_reject_count",
              lane.ema_adjacency_reject_count);
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

static FLASHMEM void add_ocxo_witness_detail_payload(payload_prefix_t& out,
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

static FLASHMEM void add_ocxo_qtimer_payload(payload_prefix_t& out,
                                    const ocxo_runtime_context_t& ctx,
                                    const ocxo_qtimer_diag_t& qdiag) {
  const ocxo_lane_t& lane = *ctx.lane;
  const uint16_t csctrl = lane.module->CH[lane.compare_channel].CSCTRL;
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

static FLASHMEM void add_ocxo_perishable_ring_payload(payload_prefix_t& out,
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

static FLASHMEM void add_ocxo_compact_payload(Payload& p,
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
  out.add_bool("last_diag_dwt_synthetic",
               rt && rt->has_fired && rt->last_diag.dwt_synthetic);
  out.add_u32("last_diag_dwt_original_at_event",
              (rt && rt->has_fired) ? rt->last_diag.dwt_original_at_event : 0U);
  out.add_u32("last_diag_dwt_predicted_at_event",
              (rt && rt->has_fired) ? rt->last_diag.dwt_predicted_at_event : 0U);
  out.add_u32("last_diag_dwt_used_at_event",
              (rt && rt->has_fired) ? rt->last_diag.dwt_used_at_event : 0U);
  out.add_u32("last_diag_dwt_isr_entry_raw",
              (rt && rt->has_fired) ? rt->last_diag.dwt_isr_entry_raw : 0U);
  out.add_u32("last_diag_dwt_event_from_isr_entry_raw",
              (rt && rt->has_fired)
                  ? rt->last_diag.dwt_event_from_isr_entry_raw
                  : 0U);
  out.add_i32("last_diag_dwt_isr_entry_to_event_correction_cycles",
              (rt && rt->has_fired)
                  ? rt->last_diag.dwt_isr_entry_to_event_correction_cycles
                  : 0);
  out.add_i32("last_diag_dwt_published_minus_event_cycles",
              (rt && rt->has_fired)
                  ? rt->last_diag.dwt_published_minus_event_cycles
                  : 0);
  out.add_i32("last_diag_dwt_used_minus_event_cycles",
              (rt && rt->has_fired)
                  ? rt->last_diag.dwt_used_minus_event_cycles
                  : 0);
  out.add_i32("last_diag_dwt_synthetic_error_cycles",
              (rt && rt->has_fired) ? rt->last_diag.dwt_synthetic_error_cycles : 0);
  out.add_bool("last_diag_spinidle_shadow_valid",
               rt && rt->has_fired && rt->last_diag.spinidle_shadow_valid);
  out.add_u32("last_diag_spinidle_shadow_dwt",
              (rt && rt->has_fired) ? rt->last_diag.spinidle_shadow_dwt : 0U);
  out.add_u32("last_diag_spinidle_shadow_to_isr_entry_cycles",
              (rt && rt->has_fired)
                  ? rt->last_diag.spinidle_shadow_to_isr_entry_cycles
                  : 0U);
  out.add_u32("last_diag_spinidle_shadow_valid_threshold_cycles",
              (rt && rt->has_fired)
                  ? rt->last_diag.spinidle_shadow_valid_threshold_cycles
                  : SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
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
  out.add_bool("ema_last_adjacency_rejected", lane.ema_last_adjacency_rejected);
  out.add_u32("ema_last_counter_delta_ticks", lane.ema_last_counter_delta_ticks);
  out.add_u32("ema_expected_counter_delta_ticks", lane.ema_expected_counter_delta_ticks);
  out.add_u32("ema_adjacency_reject_count", lane.ema_adjacency_reject_count);
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

static FLASHMEM void add_handoff_source_payload(Payload& p,
                                       const char* prefix,
                                       const interrupt_handoff_source_diag_t& d,
                                       uint32_t capacity) {
  payload_prefix_t out(p, prefix);
  out.add_u32("capture_ring_capacity", capacity);
  out.add_u32("capture_count", d.capture_count);
  out.add_u32("capture_enqueue_count", d.enqueue_count);
  out.add_u32("capture_dequeue_count", d.dequeue_count);
  out.add_u32("capture_overrun_count", d.overrun_count);
  out.add_u32("capture_high_water", d.high_water);
  out.add_u32("capture_pending_count", d.pending_count);
  out.add_u32("last_sequence", d.last_sequence);
  out.add_u32("last_capture_dwt", d.last_capture_dwt);
  out.add_u32("last_handoff_entry_dwt", d.last_handoff_entry_dwt);
  out.add_u32("capture_to_handoff_cycles_last", d.last_capture_to_handoff_cycles);
  out.add_u32("capture_to_handoff_cycles_max", d.max_capture_to_handoff_cycles);
  out.add_u32("priority0_body_cycles_last", d.last_priority0_body_cycles);
  out.add_u32("priority0_body_cycles_max", d.max_priority0_body_cycles);
  out.add_u32("handoff_body_cycles_last", d.last_handoff_body_cycles);
  out.add_u32("handoff_body_cycles_max", d.max_handoff_body_cycles);
}

static FLASHMEM void add_handoff_payload(Payload& p) {
  const uint32_t irq = INTERRUPT_HANDOFF_IRQ_NUMBER;
  const uint32_t mask = interrupt_nvic_irq_mask(irq);
  const uint32_t iser = interrupt_nvic_iser_word(irq);
  const uint32_t ispr = interrupt_nvic_ispr_word(irq);
  const uint32_t iabr = interrupt_nvic_iabr_word(irq);

  p.add("capture_discipline", INTERRUPT_CAPTURE_DISCIPLINE);
  p.add("handoff_mechanism", INTERRUPT_HANDOFF_MECHANISM);
  p.add("handoff_irq_name", INTERRUPT_HANDOFF_IRQ_NAME);
  p.add("handoff_irq", INTERRUPT_HANDOFF_IRQ_NUMBER);
  p.add("handoff_irq_index", irq >> 5);
  p.add("handoff_irq_mask", mask);
  p.add("handoff_priority", INTERRUPT_PRIORITY_HANDOFF);
  p.add("handoff_configured", g_interrupt_handoff.configured);
  p.add("handoff_configure_count", g_interrupt_handoff.configure_count);
  p.add("handoff_running", g_interrupt_handoff.running);
  p.add("handoff_pending", g_interrupt_handoff.pending);
  p.add("handoff_enabled_bit", (iser & mask) != 0);
  p.add("handoff_pending_bit", (ispr & mask) != 0);
  p.add("handoff_active_bit", (iabr & mask) != 0);
  p.add("handoff_iser_word", iser);
  p.add("handoff_ispr_word", ispr);
  p.add("handoff_iabr_word", iabr);
  p.add("handoff_request_count", g_interrupt_handoff.request_count);
  p.add("handoff_entry_count", g_interrupt_handoff.entry_count);
  p.add("handoff_exit_count", g_interrupt_handoff.exit_count);
  p.add("handoff_served_request_count", g_interrupt_handoff.served_request_count);
  p.add("handoff_unserved_request_count", g_interrupt_handoff.unserved_request_count);
  p.add("handoff_spurious_entry_count", g_interrupt_handoff.spurious_entry_count);
  p.add("handoff_reentry_count", g_interrupt_handoff.reentry_count);
  p.add("handoff_repend_count", g_interrupt_handoff.repend_count);
  p.add("handoff_drain_budget", INTERRUPT_HANDOFF_DRAIN_BUDGET);
  p.add("handoff_drain_pass_count", g_interrupt_handoff.drain_pass_count);
  p.add("handoff_drain_budget_exhausted_count",
        g_interrupt_handoff.drain_budget_exhausted_count);
  p.add("handoff_last_request_context",
        (const char*)(g_interrupt_handoff.last_request_context
                          ? g_interrupt_handoff.last_request_context
                          : ""));
  p.add("handoff_last_request_dwt", g_interrupt_handoff.last_request_dwt);
  p.add("handoff_last_entry_dwt", g_interrupt_handoff.last_entry_dwt);
  p.add("handoff_last_exit_dwt", g_interrupt_handoff.last_exit_dwt);
  p.add("handoff_last_latency_cycles", g_interrupt_handoff.last_latency_cycles);
  p.add("handoff_min_latency_cycles",
        g_interrupt_handoff.min_latency_cycles == UINT32_MAX
            ? 0U
            : g_interrupt_handoff.min_latency_cycles);
  p.add("handoff_max_latency_cycles", g_interrupt_handoff.max_latency_cycles);
  p.add("handoff_mean_latency_cycles",
        g_interrupt_handoff.latency_sample_count
            ? (uint32_t)(g_interrupt_handoff.latency_cycles_sum /
                         (uint64_t)g_interrupt_handoff.latency_sample_count)
            : 0U);
  p.add("handoff_latency_sample_count", g_interrupt_handoff.latency_sample_count);
  p.add("handoff_last_body_cycles", g_interrupt_handoff.last_body_cycles);
  p.add("handoff_max_body_cycles", g_interrupt_handoff.max_body_cycles);
  p.add("handoff_last_request_ipsr", g_interrupt_handoff.last_request_ipsr);
  p.add("handoff_entry_ipsr", g_interrupt_handoff.entry_ipsr);
  p.add("handoff_last_request_primask", g_interrupt_handoff.last_request_primask);
  p.add("handoff_entry_primask", g_interrupt_handoff.entry_primask);

  add_handoff_source_payload(p, "handoff_qtimer1_ch1", g_handoff_qtimer1_ch1,
                             HANDOFF_QTIMER1_CH1_RING_SIZE);
  add_handoff_source_payload(p, "handoff_qtimer1_ch2", g_handoff_qtimer1_ch2,
                             HANDOFF_QTIMER1_CH2_RING_SIZE);
  add_handoff_source_payload(p, "handoff_ocxo1", g_handoff_ocxo1,
                             HANDOFF_OCXO_RING_SIZE);
  add_handoff_source_payload(p, "handoff_ocxo2", g_handoff_ocxo2,
                             HANDOFF_OCXO_RING_SIZE);
  add_handoff_source_payload(p, "handoff_pps", g_handoff_pps,
                             HANDOFF_PPS_RING_SIZE);
}

static FLASHMEM Payload cmd_report(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_COMPACT");
  p.add("subreports",
        "REPORT_STATUS REPORT_PPS REPORT_CADENCE REPORT_SMARTZERO "
        "REPORT_BRIDGE REPORT_HANDOFF REPORT_LANES REPORT_LANE");
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
  p.add("timing_arch_step", "UNIFORM_CH0_COUNTER_CH1_COMPARE_WITH_OCXO2_PIN15_CH3_ADAPTER");
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
  p.add("capture_discipline", INTERRUPT_CAPTURE_DISCIPLINE);
  p.add("handoff_mechanism", INTERRUPT_HANDOFF_MECHANISM);
  p.add("handoff_irq", INTERRUPT_HANDOFF_IRQ_NUMBER);
  p.add("handoff_priority", INTERRUPT_PRIORITY_HANDOFF);
  p.add("handoff_configured", g_interrupt_handoff.configured);
  p.add("handoff_request_count", g_interrupt_handoff.request_count);
  p.add("handoff_entry_count", g_interrupt_handoff.entry_count);
  p.add("handoff_exit_count", g_interrupt_handoff.exit_count);
  p.add("handoff_unserved_request_count", g_interrupt_handoff.unserved_request_count);
  p.add("handoff_reentry_count", g_interrupt_handoff.reentry_count);
  p.add("handoff_spurious_entry_count", g_interrupt_handoff.spurious_entry_count);
  p.add("handoff_max_latency_cycles", g_interrupt_handoff.max_latency_cycles);
  p.add("handoff_max_body_cycles", g_interrupt_handoff.max_body_cycles);
  p.add("handoff_qtimer1_ch2_overrun_count", g_handoff_qtimer1_ch2.overrun_count);
  p.add("handoff_ocxo1_overrun_count", g_handoff_ocxo1.overrun_count);
  p.add("handoff_ocxo2_overrun_count", g_handoff_ocxo2.overrun_count);
  p.add("handoff_pps_overrun_count", g_handoff_pps.overrun_count);

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
  p.add("vclock_ema_dwt_authority_enabled", VCLOCK_EMA_DWT_AUTHORITY_ENABLED);
  p.add("vclock_ema_initialized", g_vclock_lane.ema_initialized);
  p.add("vclock_ema_update_count", g_vclock_lane.ema_update_count);
  p.add("vclock_ema_interval_cycles", g_vclock_lane.ema_interval_cycles);
  p.add("vclock_ema_last_error_cycles", g_vclock_lane.ema_last_error_cycles);
  p.add("vclock_ema_max_abs_error_cycles", g_vclock_lane.ema_max_abs_error_cycles);
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
static FLASHMEM Payload cmd_report_status(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_STATUS");
  add_runtime_payload(p);
  add_priority_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_pps(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_PPS");
  add_pps_payload(p);
  add_epoch_capture_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_cadence(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_CADENCE");
  add_vclock_heartbeat_payload(p);
  add_dynamic_cps_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_smartzero(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_SMARTZERO");
  add_smartzero_payload(p);
  return p;
}

static FLASHMEM void add_pvc_anchor_entry_payload(Payload& p,
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

static FLASHMEM void add_bridge_payload(Payload& p) {
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

static FLASHMEM Payload cmd_report_handoff(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_HANDOFF");
  add_handoff_payload(p);
  return p;
}

static FLASHMEM Payload cmd_report_bridge(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_BRIDGE");
  add_bridge_payload(p);
  return p;
}

static FLASHMEM void add_lane_summary_object(Payload& parent,
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
  if (rt && rt->has_fired) {
    lane.add("last_diag_spinidle_shadow_valid", rt->last_diag.spinidle_shadow_valid);
    lane.add("last_diag_spinidle_shadow_dwt", rt->last_diag.spinidle_shadow_dwt);
    lane.add("last_diag_spinidle_shadow_to_isr_entry_cycles",
             rt->last_diag.spinidle_shadow_to_isr_entry_cycles);
    lane.add("last_diag_spinidle_shadow_valid_threshold_cycles",
             rt->last_diag.spinidle_shadow_valid_threshold_cycles);
    lane.add("last_diag_dwt_synthetic", rt->last_diag.dwt_synthetic);
    lane.add("last_diag_dwt_original_at_event", rt->last_diag.dwt_original_at_event);
    lane.add("last_diag_dwt_predicted_at_event", rt->last_diag.dwt_predicted_at_event);
    lane.add("last_diag_dwt_used_at_event", rt->last_diag.dwt_used_at_event);
    lane.add("last_diag_dwt_isr_entry_raw", rt->last_diag.dwt_isr_entry_raw);
    lane.add("last_diag_dwt_synthetic_error_cycles", rt->last_diag.dwt_synthetic_error_cycles);
  } else {
    lane.add("last_diag_spinidle_shadow_valid", false);
    lane.add("last_diag_spinidle_shadow_dwt", 0U);
    lane.add("last_diag_spinidle_shadow_to_isr_entry_cycles", 0U);
    lane.add("last_diag_spinidle_shadow_valid_threshold_cycles",
             SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
    lane.add("last_diag_dwt_synthetic", false);
    lane.add("last_diag_dwt_original_at_event", 0U);
    lane.add("last_diag_dwt_predicted_at_event", 0U);
    lane.add("last_diag_dwt_used_at_event", 0U);
    lane.add("last_diag_dwt_isr_entry_raw", 0U);
    lane.add("last_diag_dwt_synthetic_error_cycles", 0);
  }

  // Only VCLOCK uses this generic summary helper today.  Keep the compact
  // sanity surface here so REPORT_LANES can reveal trouble without the full
  // lane detail report.
  lane.add("isr_sanity_cntr_incorrect_count", g_isr_sanity_vclock_ch2.cntr_incorrect_count);
  lane.add("isr_sanity_dwt_incorrect_count", g_isr_sanity_vclock_ch2.dwt_incorrect_count);
  parent.add_object(key, lane);
}

static FLASHMEM void add_ocxo_lane_summary_object(Payload& parent,
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
  o.add("ema_last_adjacency_rejected", lane.ema_last_adjacency_rejected);
  o.add("ema_last_counter_delta_ticks", lane.ema_last_counter_delta_ticks);
  o.add("ema_expected_counter_delta_ticks", lane.ema_expected_counter_delta_ticks);
  o.add("ema_adjacency_reject_count", lane.ema_adjacency_reject_count);
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
    o.add("last_diag_spinidle_shadow_valid", rt->last_diag.spinidle_shadow_valid);
    o.add("last_diag_spinidle_shadow_dwt", rt->last_diag.spinidle_shadow_dwt);
    o.add("last_diag_spinidle_shadow_to_isr_entry_cycles",
          rt->last_diag.spinidle_shadow_to_isr_entry_cycles);
    o.add("last_diag_spinidle_shadow_valid_threshold_cycles",
          rt->last_diag.spinidle_shadow_valid_threshold_cycles);
    o.add("last_diag_anchor_selection_kind", rt->last_diag.anchor_selection_kind);
    o.add("last_diag_anchor_failure_mask", rt->last_diag.anchor_failure_mask);
  } else {
    o.add("last_event_dwt", 0U);
    o.add("last_event_counter32", 0U);
    o.add("last_event_gnss_ns_at_event", 0ULL);
    o.add("last_event_gnss_ns_available", false);
    o.add("last_diag_gnss_ns_at_event", 0ULL);
    o.add("last_diag_gnss_projection_valid", false);
    o.add("last_diag_spinidle_shadow_valid", false);
    o.add("last_diag_spinidle_shadow_dwt", 0U);
    o.add("last_diag_spinidle_shadow_to_isr_entry_cycles", 0U);
    o.add("last_diag_spinidle_shadow_valid_threshold_cycles",
          SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
    o.add("last_diag_anchor_selection_kind", 0U);
    o.add("last_diag_anchor_failure_mask", 0U);
  }
  parent.add_object(key, o);
}

static FLASHMEM void add_idle_dwt_witness_payload(Payload& p) {
  timepop_idle_witness_snapshot_t idle{};
  timepop_idle_witness_snapshot(&idle);
  p.add("idle_dwt_witness_supported", idle.supported);
  p.add("idle_dwt_witness_enabled", idle.enabled);
  p.add("idle_dwt_witness_running", idle.running);
  p.add("idle_dwt_shadow_dwt", idle.shadow_dwt);
  p.add("idle_dwt_shadow_age_cycles_at_report",
        idle.shadow_dwt ? (uint32_t)(ARM_DWT_CYCCNT - idle.shadow_dwt) : 0U);
  p.add("idle_dwt_witness_enter_count", idle.enter_count);
  p.add("idle_dwt_witness_exit_count", idle.exit_count);
  p.add("idle_dwt_witness_pending_exit_count", idle.pending_exit_count);
  p.add("spinidle_isr_witness_supported", SPINIDLE_ISR_WITNESS_SUPPORTED);
  p.add("spinidle_isr_witness_enabled", SPINIDLE_ISR_WITNESS_ENABLED);
  p.add("spinidle_shadow_valid_threshold_cycles",
        SPINIDLE_SHADOW_VALID_THRESHOLD_CYCLES);
  p.add("idle_dwt_witness_last_enter_dwt", idle.last_enter_dwt);
  p.add("idle_dwt_witness_last_exit_dwt", idle.last_exit_dwt);
}


// ============================================================================
// Sectioned lane reports
// ============================================================================
//
// REPORT_LANE used to return every lane detail surface at once.  That payload
// is now large enough to overrun the command transport on the Pi side.  Keep
// the default single-lane report compact and require callers to request one
// bounded section at a time.

static const char* REPORT_LANE_VCLOCK_SECTIONS =
    "summary ema ch2 epoch smartzero fact_ring clock32 qtimer regression spincatch isr";
static const char* REPORT_LANE_OCXO_SECTIONS =
    "summary ema yardstick zero_worthy service scheduler witness qtimer ring clock32 regression spincatch isr";

static FLASHMEM const char* report_lane_section_arg(const Payload& args) {
  const char* section = args.getString("section");
  if (!section || !*section) section = args.getString("detail");
  if (!section || !*section) section = "summary";
  return section;
}

static FLASHMEM void add_report_lane_header(Payload& p,
                                   const char* lane,
                                   const char* section,
                                   const char* sections) {
  p.add("report", "lane_section");
  p.add("lane", lane ? lane : "");
  p.add("section", section ? section : "summary");
  p.add("available_sections", sections ? sections : "");
  p.add("usage", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2 section=<section>");
}

static FLASHMEM bool report_lane_section_is(const char* section, const char* name) {
  return section && name && !strcasecmp(section, name);
}

static FLASHMEM void add_vclock_lane_summary_section(Payload& p) {
  p.add("kind", "VCLOCK");
  p.add("provider", "QTIMER1");
  p.add("hardware_lane", "QTIMER1_CH1_COMP");
  p.add("cadence_source", "QTIMER1_CH1_UNIFORM_NATIVE_EPOCH_ONE_SECOND_PLUS_HEARTBEAT_FALLBACK");
  p.add("counter_source", "QTIMER1_CH0_SYNTHETIC_COUNTER32");
  p.add("event_source", "QTIMER1_CH1_INTRINSIC_EPOCH_AND_ONE_SECOND");
  p.add("dwt_authority", "QTIMER1_CH1_EMA_PREDICTED_DWT");

  add_runtime_lane_summary(p, "vclock", g_rt_vclock);
  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count", g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_tick_mod_1000", g_vclock_lane.tick_mod_1000);
  p.add("vclock_logical_count32", g_vclock_lane.logical_count32_at_last_second);
  p.add("vclock_ema_initialized", g_vclock_lane.ema_initialized);
  p.add("vclock_ema_interval_valid", g_vclock_lane.ema_interval_valid);
  p.add("vclock_ema_update_count", g_vclock_lane.ema_update_count);
  p.add("vclock_ema_interval_cycles", g_vclock_lane.ema_interval_cycles);
  p.add("vclock_ema_last_error_cycles", g_vclock_lane.ema_last_error_cycles);
  p.add("vclock_fact_ring_count", g_vclock_fact_ring.count);
  p.add("vclock_fact_enqueue_count", g_vclock_fact_ring.enqueue_count);
  p.add("vclock_fact_drain_count", g_vclock_fact_ring.drain_count);
  p.add("vclock_fact_overflow_count", g_vclock_fact_ring.overflow_count);
  p.add("vclock_fact_high_water", g_vclock_fact_ring.high_water);
  add_isr_sanity_payload(p, "isr_sanity", g_isr_sanity_vclock_ch2);
}

static FLASHMEM void add_vclock_lane_ema_section(Payload& p) {
  p.add("vclock_ema_dwt_authority_enabled", VCLOCK_EMA_DWT_AUTHORITY_ENABLED);
  p.add("vclock_ema_alpha_numerator", VCLOCK_EMA_ALPHA_NUMERATOR);
  p.add("vclock_ema_alpha_denominator", VCLOCK_EMA_ALPHA_DENOMINATOR);
  p.add("vclock_ema_audit_threshold_cycles", VCLOCK_EMA_AUDIT_THRESHOLD_CYCLES);
  p.add("vclock_ema_initialized", g_vclock_lane.ema_initialized);
  p.add("vclock_ema_interval_valid", g_vclock_lane.ema_interval_valid);
  p.add("vclock_ema_update_count", g_vclock_lane.ema_update_count);
  p.add("vclock_ema_last_observed_dwt", g_vclock_lane.ema_last_observed_dwt);
  p.add("vclock_ema_last_emitted_dwt", g_vclock_lane.ema_last_emitted_dwt);
  p.add("vclock_ema_last_observed_interval_cycles", g_vclock_lane.ema_last_observed_interval_cycles);
  p.add("vclock_ema_interval_cycles", g_vclock_lane.ema_interval_cycles);
  p.add("vclock_ema_last_predicted_dwt", g_vclock_lane.ema_last_predicted_dwt);
  p.add("vclock_ema_last_interval_prediction_cycles", g_vclock_lane.ema_last_interval_prediction_cycles);
  p.add("vclock_ema_last_effective_interval_cycles", g_vclock_lane.ema_last_effective_interval_cycles);
  p.add("vclock_ema_last_interval_residual_cycles", g_vclock_lane.ema_last_interval_residual_cycles);
  p.add("vclock_ema_last_error_cycles", g_vclock_lane.ema_last_error_cycles);
  p.add("vclock_ema_max_abs_error_cycles", g_vclock_lane.ema_max_abs_error_cycles);
  p.add("vclock_ema_gate_accept_count", g_vclock_lane.ema_gate_accept_count);
  p.add("vclock_ema_gate_reject_count", g_vclock_lane.ema_gate_reject_count);
  p.add("vclock_ema_gate_resync_count", g_vclock_lane.ema_gate_resync_count);
  p.add("vclock_ema_gate_reject_streak", g_vclock_lane.ema_gate_reject_streak);

  if (g_rt_vclock && g_rt_vclock->has_fired) {
    const interrupt_capture_diag_t& d = g_rt_vclock->last_diag;
    p.add("last_diag_dwt_synthetic", d.dwt_synthetic);
    p.add("last_diag_dwt_original_at_event", d.dwt_original_at_event);
    p.add("last_diag_dwt_predicted_at_event", d.dwt_predicted_at_event);
    p.add("last_diag_dwt_used_at_event", d.dwt_used_at_event);
    p.add("last_diag_dwt_isr_entry_raw", d.dwt_isr_entry_raw);
    p.add("last_diag_dwt_event_from_isr_entry_raw",
          d.dwt_event_from_isr_entry_raw);
    p.add("last_diag_dwt_isr_entry_to_event_correction_cycles",
          d.dwt_isr_entry_to_event_correction_cycles);
    p.add("last_diag_dwt_published_minus_event_cycles",
          d.dwt_published_minus_event_cycles);
    p.add("last_diag_dwt_used_minus_event_cycles",
          d.dwt_used_minus_event_cycles);
    p.add("last_diag_dwt_synthetic_error_cycles", d.dwt_synthetic_error_cycles);
    p.add("last_diag_dwt_interval_gate_valid", d.dwt_interval_gate_valid);
    p.add("last_diag_dwt_interval_sample_accepted", d.dwt_interval_sample_accepted);
    p.add("last_diag_dwt_interval_sample_rejected", d.dwt_interval_sample_rejected);
    p.add("last_diag_dwt_interval_ema_updated", d.dwt_interval_ema_updated);
    p.add("last_diag_dwt_interval_observed_cycles", d.dwt_interval_observed_cycles);
    p.add("last_diag_dwt_interval_prediction_cycles", d.dwt_interval_prediction_cycles);
    p.add("last_diag_dwt_interval_effective_cycles", d.dwt_interval_effective_cycles);
    p.add("last_diag_dwt_interval_residual_cycles", d.dwt_interval_residual_cycles);
    p.add("last_diag_dwt_interval_reject_streak", d.dwt_interval_reject_streak);
  }

  p.add("vclock_dwt_repair_enabled", false);
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
}

static FLASHMEM void add_vclock_lane_ch2_section(Payload& p) {
  p.add("vclock_ch2_one_second_enabled", g_vclock_ch2_one_second_enabled);
  p.add("vclock_ch2_one_second_next_counter32", g_vclock_ch2_one_second_next_counter32);
  p.add("vclock_ch2_one_second_enable_count", g_vclock_ch2_one_second_enable_count);
  p.add("vclock_ch2_one_second_epoch_enable_count", g_vclock_ch2_one_second_epoch_enable_count);
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
  p.add("vclock_ch2_fact_drain_service_count", g_vclock_ch2_fact_drain_service_count);
  p.add("vclock_ch2_fact_drain_arm_count", g_vclock_ch2_fact_drain_arm_count);
  p.add("vclock_heartbeat_one_second_legacy_count", g_vclock_heartbeat_one_second_legacy_count);
  p.add("vclock_heartbeat_one_second_handoff_skip_count", g_vclock_heartbeat_one_second_handoff_skip_count);
  p.add("vclock_heartbeat_one_second_fallback_count", g_vclock_heartbeat_one_second_fallback_count);
}

static FLASHMEM void add_vclock_lane_epoch_section(Payload& p) {
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
  p.add("vclock_heartbeat_epoch_authority_retired_count", g_vclock_heartbeat_epoch_authority_retired_count);
  p.add("vclock_heartbeat_epoch_pending_skip_count", g_vclock_heartbeat_epoch_pending_skip_count);
}

static FLASHMEM void add_vclock_lane_smartzero_section(Payload& p) {
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
}

static FLASHMEM void add_vclock_lane_fact_ring_section(Payload& p) {
  p.add("vclock_perishable_fact_ring_size", VCLOCK_PERISHABLE_FACT_RING_SIZE);
  p.add("vclock_perishable_fact_ring_count", g_vclock_fact_ring.count);
  p.add("vclock_perishable_fact_enqueue_count", g_vclock_fact_ring.enqueue_count);
  p.add("vclock_perishable_fact_drain_count", g_vclock_fact_ring.drain_count);
  p.add("vclock_perishable_fact_overflow_count", g_vclock_fact_ring.overflow_count);
  p.add("vclock_perishable_fact_high_water", g_vclock_fact_ring.high_water);
  p.add("vclock_perishable_fact_asap_arm_count", g_vclock_fact_ring.asap_arm_count);
  p.add("vclock_perishable_fact_asap_fail_count", g_vclock_fact_ring.asap_fail_count);
  p.add("vclock_perishable_fact_drain_armed", g_vclock_fact_ring.drain_armed);
}

static FLASHMEM void add_vclock_lane_qtimer_section(Payload& p) {
  p.add("qtimer1_ch1_active", g_qtimer1_ch1_active);
  p.add("qtimer1_ch1_sequence", g_qtimer1_ch1_sequence);
  p.add("qtimer1_ch1_target_counter32", g_qtimer1_ch1_target_counter32);
  p.add("qtimer1_ch1_next_compare_counter32", g_qtimer1_ch1_next_compare_counter32);
  p.add("qtimer1_ch1_arm_count", g_qtimer1_ch1_arm_count);
  p.add("qtimer1_ch1_fire_count", g_qtimer1_ch1_fire_count);
  p.add("qtimer1_ch1_hop_count", g_qtimer1_ch1_hop_count);
  p.add("qtimer1_ch2_last_target_counter32", g_qtimer1_ch2_last_target_counter32);
  p.add("qtimer1_ch2_arm_count", g_qtimer1_ch2_arm_count);
  p.add("ch2_implicit_rollover_enabled", CH2_IMPLICIT_ROLLOVER_ENABLED);
  p.add("ch2_implicit_rollover_count", g_ch2_implicit_rollover_count);
  p.add("ch2_implicit_rollover_vclock_updates", g_ch2_implicit_rollover_vclock_updates);
  p.add("ch2_implicit_rollover_last_vclock_hw16", (uint32_t)g_ch2_implicit_rollover_last_vclock_hw16);
  p.add("ch2_implicit_rollover_last_vclock_counter32", g_ch2_implicit_rollover_last_vclock_counter32);
}

static FLASHMEM bool add_vclock_lane_section(Payload& p, const char* section) {
  if (report_lane_section_is(section, "summary")) {
    add_vclock_lane_summary_section(p);
  } else if (report_lane_section_is(section, "ema")) {
    add_vclock_lane_ema_section(p);
  } else if (report_lane_section_is(section, "ch2")) {
    add_vclock_lane_ch2_section(p);
  } else if (report_lane_section_is(section, "epoch")) {
    add_vclock_lane_epoch_section(p);
  } else if (report_lane_section_is(section, "smartzero")) {
    add_vclock_lane_smartzero_section(p);
  } else if (report_lane_section_is(section, "fact_ring") ||
             report_lane_section_is(section, "ring")) {
    add_vclock_lane_fact_ring_section(p);
  } else if (report_lane_section_is(section, "clock32")) {
    add_vclock_clock32_payload(p, "vclock");
  } else if (report_lane_section_is(section, "qtimer")) {
    add_vclock_lane_qtimer_section(p);
  } else if (report_lane_section_is(section, "regression")) {
    add_regression_payload(p, "vclock", g_regression_vclock);
  } else if (report_lane_section_is(section, "spincatch")) {
    add_spincatch_report_payload(p, interrupt_subscriber_kind_t::VCLOCK);
  } else if (report_lane_section_is(section, "isr")) {
    add_isr_sanity_payload(p, "isr_sanity", g_isr_sanity_vclock_ch2);
  } else {
    return false;
  }
  return true;
}

static FLASHMEM void add_ocxo_lane_summary_section(Payload& p,
                                          const ocxo_runtime_context_t& ctx) {
  add_ocxo_identity_payload(p, ctx);
  add_ocxo_compact_payload(p, ctx);
}

static FLASHMEM void add_ocxo_lane_ema_section(Payload& p,
                                      const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  const interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  payload_prefix_t out(p, ctx.prefix);

  out.add_bool("ema_dwt_authority_enabled", OCXO_EMA_DWT_AUTHORITY_ENABLED);
  out.add_u32("ema_alpha_numerator", OCXO_EMA_ALPHA_NUMERATOR);
  out.add_u32("ema_alpha_denominator", OCXO_EMA_ALPHA_DENOMINATOR);
  out.add_bool("ema_initialized", lane.ema_initialized);
  out.add_bool("ema_interval_valid", lane.ema_interval_valid);
  out.add_u32("ema_update_count", lane.ema_update_count);
  out.add_u32("ema_last_observed_dwt", lane.ema_last_observed_dwt);
  out.add_u32("ema_last_emitted_dwt", lane.ema_last_emitted_dwt);
  out.add_u32("ema_last_observed_interval_cycles", lane.ema_last_observed_interval_cycles);
  out.add_u32("ema_interval_cycles", lane.ema_interval_cycles);
  out.add_u32("ema_last_predicted_dwt", lane.ema_last_predicted_dwt);
  out.add_u32("ema_last_interval_prediction_cycles", lane.ema_last_interval_prediction_cycles);
  out.add_u32("ema_last_effective_interval_cycles", lane.ema_last_effective_interval_cycles);
  out.add_i32("ema_last_interval_residual_cycles", lane.ema_last_interval_residual_cycles);
  out.add_i32("ema_last_error_cycles", lane.ema_last_error_cycles);
  out.add_u32("ema_max_abs_error_cycles", lane.ema_max_abs_error_cycles);
  out.add_u32("ema_gate_accept_count", lane.ema_gate_accept_count);
  out.add_u32("ema_gate_reject_count", lane.ema_gate_reject_count);
  out.add_u32("ema_gate_resync_count", lane.ema_gate_resync_count);
  out.add_u32("ema_gate_reject_streak", lane.ema_gate_reject_streak);
  out.add_bool("ema_last_adjacency_gate_valid", lane.ema_last_adjacency_gate_valid);
  out.add_bool("ema_last_adjacency_ok", lane.ema_last_adjacency_ok);
  out.add_bool("ema_last_adjacency_rejected", lane.ema_last_adjacency_rejected);
  out.add_u32("ema_last_counter_delta_ticks", lane.ema_last_counter_delta_ticks);
  out.add_u32("ema_expected_counter_delta_ticks", lane.ema_expected_counter_delta_ticks);
  out.add_u32("ema_adjacency_reject_count", lane.ema_adjacency_reject_count);

  if (rt && rt->has_fired) {
    const interrupt_capture_diag_t& d = rt->last_diag;
    out.add_bool("last_diag_dwt_synthetic", d.dwt_synthetic);
    out.add_u32("last_diag_dwt_original_at_event", d.dwt_original_at_event);
    out.add_u32("last_diag_dwt_predicted_at_event", d.dwt_predicted_at_event);
    out.add_u32("last_diag_dwt_used_at_event", d.dwt_used_at_event);
    out.add_u32("last_diag_dwt_isr_entry_raw", d.dwt_isr_entry_raw);
    out.add_i32("last_diag_dwt_synthetic_error_cycles", d.dwt_synthetic_error_cycles);
    out.add_bool("last_diag_dwt_interval_adjacency_gate_valid", d.dwt_interval_adjacency_gate_valid);
    out.add_bool("last_diag_dwt_interval_adjacency_ok", d.dwt_interval_adjacency_ok);
    out.add_bool("last_diag_dwt_interval_adjacency_rejected", d.dwt_interval_adjacency_rejected);
    out.add_u32("last_diag_dwt_interval_counter_delta_ticks", d.dwt_interval_counter_delta_ticks);
    out.add_u32("last_diag_dwt_interval_expected_counter_delta_ticks", d.dwt_interval_expected_counter_delta_ticks);
    out.add_u32("last_diag_dwt_interval_adjacency_reject_count", d.dwt_interval_adjacency_reject_count);
    out.add_bool("last_diag_dwt_interval_gate_valid", d.dwt_interval_gate_valid);
    out.add_bool("last_diag_dwt_interval_sample_accepted", d.dwt_interval_sample_accepted);
    out.add_bool("last_diag_dwt_interval_sample_rejected", d.dwt_interval_sample_rejected);
    out.add_bool("last_diag_dwt_interval_ema_updated", d.dwt_interval_ema_updated);
    out.add_u32("last_diag_dwt_interval_observed_cycles", d.dwt_interval_observed_cycles);
    out.add_u32("last_diag_dwt_interval_prediction_cycles", d.dwt_interval_prediction_cycles);
    out.add_u32("last_diag_dwt_interval_effective_cycles", d.dwt_interval_effective_cycles);
    out.add_i32("last_diag_dwt_interval_residual_cycles", d.dwt_interval_residual_cycles);
    out.add_u32("last_diag_dwt_interval_reject_streak", d.dwt_interval_reject_streak);
  }
}

static FLASHMEM void add_ocxo_lane_zero_worthy_section(Payload& p,
                                       const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  payload_prefix_t out(p, ctx.prefix);

  // Qualification policy constants.
  out.add_bool("zw_selection_authority_enabled",
               SMARTZERO2_SELECTION_AUTHORITY_ENABLED);
  out.add_u32("zw_streak_seconds_required", ZERO_WORTHY_STREAK_SECONDS);
  out.add_i32("zw_max_abs_auth_error_cycles",
              ZERO_WORTHY_MAX_ABS_AUTH_ERROR_CYCLES);
  out.add_i32("zw_max_abs_error_sum_cycles",
              ZERO_WORTHY_MAX_ABS_ERROR_SUM_CYCLES);

  // Live verdict.
  out.add_bool("zw_worthy", lane.zw_worthy);
  out.add_u32("zw_worthy_streak_seconds", lane.zw_worthy_streak_seconds);
  out.add_u32("zw_clean_streak_seconds", lane.zw_clean_streak_seconds);
  out.add_u32("zw_ring_count", lane.zw_ring_count);
  out.add_i32("zw_window_error_sum_cycles", lane.zw_window_error_sum);
  out.add_u32("zw_window_max_abs_error_cycles", lane.zw_window_max_abs_error);
  out.add_string("zw_last_disqualify_reason", lane.zw_last_disqualify_reason);

  // Efficacy evidence.
  out.add_u32("zw_seconds_evaluated", lane.zw_seconds_evaluated);
  out.add_u32("zw_worthy_second_count", lane.zw_worthy_second_count);
  out.add_u32("zw_first_worthy_after_seconds",
              lane.zw_first_worthy_after_seconds);
  out.add_u32("zw_transition_count", lane.zw_transition_count);

  // Disqualification forensics.
  out.add_u32("zw_dq_rail_invalid", lane.zw_disqualify_rail_invalid_count);
  out.add_u32("zw_dq_excursion", lane.zw_disqualify_excursion_count);
  out.add_u32("zw_dq_stale", lane.zw_disqualify_stale_count);
  out.add_u32("zw_dq_seed", lane.zw_disqualify_seed_count);
  out.add_u32("zw_dq_adjacency", lane.zw_disqualify_adjacency_count);
  out.add_u32("zw_dq_no_interval", lane.zw_disqualify_no_interval_count);
  out.add_u32("zw_dq_error_magnitude",
              lane.zw_disqualify_error_magnitude_count);
  out.add_u32("zw_dq_error_sum_retrace", lane.zw_disqualify_error_sum_count);
  out.add_u32("zw_dq_streak_building",
              lane.zw_disqualify_streak_building_count);
  out.add_u32("zw_dq_fast_lock_acquisition",
              lane.zw_disqualify_fast_lock_count);
  out.add_u32("zw_fast_lock_remaining_seconds",
              lane.yardstick_auth_fast_lock_remaining_seconds);

  // System-level verdict (unprefixed: one truth, reported on either lane).
  p.add("smartzero2_all_lanes_worthy", (bool)g_smartzero2.all_lanes_worthy);
  p.add("smartzero2_all_worthy_streak_seconds",
        g_smartzero2.all_worthy_streak_seconds);
  p.add("smartzero2_all_worthy_second_count",
        g_smartzero2.all_worthy_second_count);
  p.add("smartzero2_first_all_worthy_pps_sequence",
        g_smartzero2.first_all_worthy_pps_sequence);
  p.add("smartzero2_epoch_captures_qualified",
        g_smartzero2.epoch_captures_qualified);
  p.add("smartzero2_epoch_captures_unqualified",
        g_smartzero2.epoch_captures_unqualified);
}

static FLASHMEM void add_ocxo_lane_yardstick_section(Payload& p,
                                      const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  payload_prefix_t out(p, ctx.prefix);

  out.add_bool("yardstick_dwt_authority_enabled",
               OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED);
  out.add_u32("yardstick_gate_threshold_cycles",
              OCXO_YARDSTICK_GATE_THRESHOLD_CYCLES);
  out.add_bool("yardstick_chain_valid", lane.yardstick_chain_valid);
  out.add_u32("yardstick_update_count", lane.yardstick_update_count);
  out.add_u32("yardstick_seed_count", lane.yardstick_seed_count);
  out.add_u32("yardstick_stale_hold_count", lane.yardstick_stale_hold_count);
  out.add_u32("yardstick_gate_agree_count", lane.yardstick_gate_agree_count);
  out.add_u32("yardstick_gate_excursion_count",
              lane.yardstick_gate_excursion_count);
  out.add_u32("yardstick_excursion_streak", lane.yardstick_excursion_streak);
  out.add_u32("yardstick_adjacency_reseed_count",
              lane.yardstick_adjacency_reseed_count);
  out.add_u32("yardstick_coherent_reseed_count",
              lane.yardstick_coherent_reseed_count);
  out.add_bool("yardstick_last_stale", lane.yardstick_last_stale);
  out.add_bool("yardstick_last_excursion", lane.yardstick_last_excursion);
  out.add_u32("yardstick_last_g_now_cycles", lane.yardstick_last_g_now_cycles);
  out.add_u32("yardstick_last_g_prev_cycles",
              lane.yardstick_last_g_prev_cycles);
  out.add_u32("yardstick_last_pps_seq_delta",
              lane.yardstick_last_pps_seq_delta);
  out.add_u32("yardstick_last_inferred_interval_cycles",
              lane.yardstick_last_inferred_interval_cycles);
  out.add_u32("yardstick_last_observed_interval_cycles",
              lane.yardstick_last_observed_interval_cycles);
  out.add_i32("yardstick_last_inferred_minus_observed_cycles",
              lane.yardstick_last_inferred_minus_observed_cycles);
  out.add_i32("yardstick_last_endpoint_minus_observed_cycles",
              lane.yardstick_last_endpoint_minus_observed_cycles);
  out.add_i32("yardstick_endpoint_minus_observed_min_cycles",
              lane.yardstick_endpoint_minus_observed_min_cycles);
  out.add_i32("yardstick_endpoint_minus_observed_max_cycles",
              lane.yardstick_endpoint_minus_observed_max_cycles);
  out.add_u32("yardstick_max_abs_inferred_minus_observed_cycles",
              lane.yardstick_max_abs_inferred_minus_observed_cycles);

  // Mean of inferred-minus-observed over agree seconds, in millicycles:
  // the campaign estimate of chain seed bias.
  const int64_t agree_n = (int64_t)lane.yardstick_gate_agree_count;
  const int64_t mean_millicycles = (agree_n > 0)
      ? (lane.yardstick_sum_inferred_minus_observed_cycles * 1000) / agree_n
      : 0;
  out.add_i32("yardstick_mean_inferred_minus_observed_millicycles",
              (int32_t)mean_millicycles);

  // Stage 2 anchored authority rail.
  out.add_bool("yardstick_authority_enabled", OCXO_YARDSTICK_DWT_AUTHORITY_ENABLED);
  out.add_u32("yardstick_anchor_p_shift", OCXO_YARDSTICK_ANCHOR_P_SHIFT);
  out.add_u32("yardstick_anchor_i_shift", OCXO_YARDSTICK_ANCHOR_I_SHIFT);
  out.add_bool("yardstick_auth_valid", lane.yardstick_auth_valid);
  out.add_u32("yardstick_auth_publish_count", lane.yardstick_auth_publish_count);
  out.add_u32("yardstick_auth_agree_publish_count",
              lane.yardstick_auth_agree_publish_count);
  out.add_u32("yardstick_auth_excursion_publish_count",
              lane.yardstick_auth_excursion_publish_count);
  out.add_u32("yardstick_auth_stale_publish_count",
              lane.yardstick_auth_stale_publish_count);
  out.add_u32("yardstick_auth_observed_fallback_count",
              lane.yardstick_auth_observed_fallback_count);
  out.add_u32("yardstick_auth_anchor_correction_count",
              lane.yardstick_auth_anchor_correction_count);
  out.add_i32("yardstick_auth_last_error_cycles",
              lane.yardstick_auth_last_error_cycles);
  out.add_u32("yardstick_auth_fast_lock_remaining_seconds",
              lane.yardstick_auth_fast_lock_remaining_seconds);
  out.add_u32("yardstick_auth_last_published_dwt",
              lane.yardstick_auth_last_published_dwt);

  // Shared PPS yardstick store state.
  pps_yardstick_snapshot_t y{};
  const bool y_loaded = pps_yardstick_load(y);
  out.add_bool("pps_yardstick_loaded", y_loaded);
  out.add_bool("pps_yardstick_valid", y.valid);
  out.add_u32("pps_yardstick_sequence", y.sequence);
  out.add_u32("pps_yardstick_interval_now_cycles", y.interval_now_cycles);
  out.add_u32("pps_yardstick_interval_prev_cycles", y.interval_prev_cycles);
  out.add_u32("pps_yardstick_accept_count", y.accept_count);
  out.add_u32("pps_yardstick_reject_count", y.reject_count);
  out.add_u32("pps_yardstick_reset_count", y.reset_count);
}

static FLASHMEM void add_ocxo_lane_scheduler_section(Payload& p,
                                            const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  payload_prefix_t out(p, ctx.prefix);
  add_ocxo_lane_basic_payload(out, lane);
  out.add_u32("witness_schedule_last_decision", lane.witness_schedule_last_decision);
  out.add_str("witness_schedule_last_decision_name", ocxo_schedule_decision_name(lane.witness_schedule_last_decision));
  out.add_u32("witness_schedule_last_current_counter32", lane.witness_schedule_last_current_counter32);
  out.add_u32("witness_schedule_last_target_counter32", lane.witness_schedule_last_target_counter32);
  out.add_u32("witness_schedule_last_remaining_ticks", lane.witness_schedule_last_remaining_ticks);
  out.add_u32("witness_schedule_last_phase_ticks", lane.witness_schedule_last_phase_ticks);
  out.add_u32("witness_schedule_last_ticks_until_arm_window", lane.witness_schedule_last_ticks_until_arm_window);
  out.add_u32("witness_schedule_last_current_low16", (uint32_t)lane.witness_schedule_last_current_low16);
  out.add_u32("witness_schedule_last_target_low16", (uint32_t)lane.witness_schedule_last_target_low16);
}

static FLASHMEM bool add_ocxo_lane_section(Payload& p,
                                  const ocxo_runtime_context_t& ctx,
                                  const char* section) {
  payload_prefix_t out(p, ctx.prefix);
  if (report_lane_section_is(section, "summary")) {
    add_ocxo_lane_summary_section(p, ctx);
  } else if (report_lane_section_is(section, "ema")) {
    add_ocxo_lane_ema_section(p, ctx);
  } else if (report_lane_section_is(section, "yardstick")) {
    add_ocxo_lane_yardstick_section(p, ctx);
  } else if (report_lane_section_is(section, "zero_worthy")) {
    add_ocxo_lane_zero_worthy_section(p, ctx);
  } else if (report_lane_section_is(section, "service")) {
    add_ocxo_witness_service_payload(out, *ctx.lane);
  } else if (report_lane_section_is(section, "scheduler")) {
    add_ocxo_lane_scheduler_section(p, ctx);
  } else if (report_lane_section_is(section, "witness")) {
    add_ocxo_witness_detail_payload(out, *ctx.lane);
  } else if (report_lane_section_is(section, "qtimer")) {
    add_ocxo_qtimer_payload(out, ctx, *ctx.qtimer_diag);
  } else if (report_lane_section_is(section, "ring") ||
             report_lane_section_is(section, "fact_ring")) {
    add_ocxo_perishable_ring_payload(out, ctx);
  } else if (report_lane_section_is(section, "clock32")) {
    add_ocxo_clock32_payload(p, ctx.prefix, *ctx.clock32);
  } else if (report_lane_section_is(section, "regression")) {
    const cadence_regression_lane_t* regression = cadence_regression_for_const(ctx.kind);
    if (regression) add_regression_payload(p, ctx.prefix, *regression);
  } else if (report_lane_section_is(section, "spincatch")) {
    add_spincatch_report_payload(p, ctx.kind);
  } else if (report_lane_section_is(section, "isr")) {
    const isr_sanity_diag_t* sanity = isr_sanity_for_kind(ctx.kind);
    if (sanity) add_isr_sanity_payload(p, "isr_sanity", *sanity);
  } else {
    return false;
  }
  return true;
}

static FLASHMEM Payload cmd_report_lanes(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_LANES");
  p.add("lane_count", 3);
  p.add("detail_command", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2 section=summary|ema|...");
  add_idle_dwt_witness_payload(p);

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

static FLASHMEM Payload cmd_report_lane(const Payload& args) {
  const char* lane = args.getString("lane");
  if (!lane || !*lane) lane = args.getString("name");
  const char* section = report_lane_section_arg(args);

  Payload p;

  if (!lane || !*lane) {
    add_report_lane_header(p, "", section, "");
    p.add("error", "missing lane parameter");
    return p;
  }

  if (!strcasecmp(lane, "VCLOCK") || !strcasecmp(lane, "VCLK")) {
    add_report_lane_header(p, "VCLOCK", section, REPORT_LANE_VCLOCK_SECTIONS);
    if (!add_vclock_lane_section(p, section)) {
      p.add("error", "unknown section for VCLOCK");
    }
    return p;
  }

  if (!strcasecmp(lane, "OCXO1") || !strcasecmp(lane, "O1")) {
    add_report_lane_header(p, "OCXO1", section, REPORT_LANE_OCXO_SECTIONS);
    if (!add_ocxo_lane_section(p, g_ocxo1_ctx, section)) {
      p.add("error", "unknown section for OCXO1");
    }
    return p;
  }

  if (!strcasecmp(lane, "OCXO2") || !strcasecmp(lane, "O2")) {
    add_report_lane_header(p, "OCXO2", section, REPORT_LANE_OCXO_SECTIONS);
    if (!add_ocxo_lane_section(p, g_ocxo2_ctx, section)) {
      p.add("error", "unknown section for OCXO2");
    }
    return p;
  }

  add_report_lane_header(p, lane, section, "");
  p.add("error", "unknown lane");
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
  { "REPORT_STATUS",   cmd_report_status   },
  { "REPORT_PPS",      cmd_report_pps      },
  { "REPORT_CADENCE",  cmd_report_cadence  },
  { "REPORT_SMARTZERO", cmd_report_smartzero },
  { "REPORT_BRIDGE",   cmd_report_bridge   },
  { "REPORT_HANDOFF", cmd_report_handoff },
  { "REPORT_LANES",    cmd_report_lanes    },
  { "REPORT_LANE",     cmd_report_lane     },
  { "REPORT_QTIMER_REGS", cmd_report_qtimer_regs },
  { "REPORT_OCXO_ISR_RAW_DWT", cmd_report_ocxo_isr_raw_dwt },
  { nullptr,           nullptr             }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

FLASHMEM void process_interrupt_register(void) {
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
    case interrupt_lane_t::QTIMER2_CH1_COMP: return "QTIMER2_CH1_COMP";
    case interrupt_lane_t::QTIMER3_CH0_COMP: return "QTIMER3_CH0_COMP";
    case interrupt_lane_t::QTIMER3_CH1_COMP: return "QTIMER3_CH1_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}