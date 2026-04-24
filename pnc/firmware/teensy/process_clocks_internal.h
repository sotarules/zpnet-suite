// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// Doctrine:
//
//   Teensy owns every statistical quantity published in TIMEBASE_FRAGMENT.
//   The Pi does not compute derived stats; it transcribes what the Teensy
//   says.  This prevents diffusion of authority and gives every downstream
//   consumer a single well-defined source of truth.
//
// Epoch authority:
//
//   The canonical epoch is VCLOCK-domain PPS.  At INIT / ZERO / START,
//   alpha calls interrupt_request_pps_rebootstrap() + sets
//   g_epoch_pending.  The next physical PPS GPIO edge is captured by
//   process_interrupt, but the public snapshot it publishes defines
//   "PPS" as the selected VCLOCK epoch edge: currently the first VCLOCK
//   edge after the physical PPS pulse, latency-adjusted into DWT space.
//
//   Alpha installs the epoch from that canonical snapshot verbatim:
//     g_epoch_dwt_at_pps = snap.dwt_at_edge
//     g_epoch_qtimer_at_pps = snap.counter32_at_edge
//     g_gnss_ns_count_at_pps = 0
//     g_epoch_pps_edge_sequence = snap.sequence
//
//   The raw physical GPIO PPS facts remain available in snap.physical_pps_*
//   only for audit/debugging.  Alpha must not apply the old
//   VCLOCK_EPOCH_TICK_OFFSET to snap.counter32_at_edge; process_interrupt
//   already published the chosen VCLOCK epoch identity.
//
//   After install, vclock_callback becomes a pure per-second advancer:
//   each invocation advances g_gnss_ns_count_at_pps by exactly 1e9,
//   and the VCLOCK cadence it responds to is phase-locked to the selected
//   VCLOCK epoch.
//
// // Statistical surface (standardized):
//
//   Every Welford accumulator published in the fragment uses the same
//   suffix set:
//
//     <prefix>_welford_n        — uint64 sample count
//     <prefix>_welford_mean     — double, in the semantic unit of the signal
//     <prefix>_welford_stddev   — double, same unit
//     <prefix>_welford_stderr   — double, same unit  (= stddev / sqrt(n))
//     <prefix>_welford_min      — double, same unit
//     <prefix>_welford_max      — double, same unit
//
//   Published Welford prefixes (seven total):
//
//     dwt_welford        — Teensy CPU XTAL offset samples (ppb)
//     vclock_welford     — bridge interpolation residual samples (ns)
//     ocxo1_welford      — OCXO1 per-second residual samples (ns)
//     ocxo2_welford      — OCXO2 per-second residual samples (ns)
//     pps_witness_welford — PPS/VCLOCK phase-error samples (ns).
//                          SOURCE: snap.counter32_at_edge minus the
//                          projected post-epoch expected counter32
//                          (g_epoch_qtimer_at_pps + edges-since-epoch ×
//                          VCLOCK_COUNTS_PER_SECOND), scaled to ns.
//                          Independent of foreground callback ordering.
//                          Steady-state mean = GPIO ISR entry latency;
//                          steady-state SD = jitter floor.
//     ocxo1_dac_welford  — OCXO1 DAC fractional code samples (LSB)
//     ocxo2_dac_welford  — OCXO2 DAC fractional code samples (LSB)
//
//   Every clock with a frequency interpretation additionally publishes:
//
//     <clock>_tau        — 1.0 + ppb / 1e9  (fractional frequency)
//     <clock>_ppb        — parts per billion  (== welford.mean for ns-unit
//                          clocks; == welford.mean for the DWT clock
//                          because the DWT Welford samples are fed as ppb)
//
//   Frequency-bearing clocks (four total):
//     dwt, vclock, ocxo1, ocxo2
//
//   (pps_witness is a phase-error measurement; has no frequency
//   interpretation.)
//
// Sign convention:
//
//   Uniform across all clocks:
//
//     positive ppb  →  this clock is RUNNING FAST vs the GNSS reference
//     negative ppb  →  this clock is RUNNING SLOW vs the GNSS reference
//
//   This is the standard atomic-clock convention.  It holds for DWT,
//   VCLOCK, OCXO1, OCXO2 alike.  Residual-ns mean has the same sign
//   semantics: positive mean → clock fast (fires early).
//
// Authorship map (post-refactor — all "_at_pps" globals are now
// authored from priority-0 GPIO ISR snap captures in
// pps_edge_callback.  The VCLOCK-callback authorship path was a
// misnomer: "at_pps" borrowed credibility that CH3-captured values
// did not earn):
//
//   - g_gnss_ns_count_at_pps       pure synthetic counter.  Set to 0
//                                  at epoch install (PPS edge IS time
//                                  zero).  Advanced by exactly +1e9
//                                  in alpha::vclock_callback per
//                                  VCLOCK one-second event.  No
//                                  physical capture involved.
//   - g_dwt_cycle_count_at_pps     snap.dwt_at_edge.  Canonical
//                                  VCLOCK-domain PPS epoch DWT selected
//                                  by the physical PPS pulse in
//                                  process_interrupt.
//   - g_dwt_cycle_count_between_pps  Difference of consecutive
//                                    snap.dwt_at_edge values across
//                                    two PPS edges.  Authored in
//                                    alpha::pps_edge_callback.
//                                    Preemption-proof.
//   - g_qtimer_at_pps              snap.counter32_at_edge.  Canonical
//                                  VCLOCK-domain PPS epoch counter
//                                  identity selected by the physical PPS
//                                  pulse in process_interrupt.
//   - g_epoch_dwt_at_pps           snap.dwt_at_edge at install.
//   - g_epoch_qtimer_at_pps        snap.counter32_at_edge at install.
//   - g_epoch_pps_edge_sequence    snap.sequence at install (for
//                                  projecting expected counter32 at
//                                  each subsequent edge).
//
// VCLOCK as measured peer of OCXO:
//
//   VCLOCK is a measured peer of OCXO1/OCXO2.  The same clock_state_t
//   and clock_measurement_t structs are used for VCLOCK, OCXO1, OCXO2.
//   clocks_apply_edge has an identical body across all three.
//
//   VCLOCK occupies a special position: it is both the GNSS-locked
//   10 MHz timebase for the QTimer1 CH0/CH1/CH2/CH3 hardware, AND a
//   peer clock being measured.  Its CH3 one-second event IS the
//   reference "1 Hz has elapsed" signal the system uses for the
//   synthetic g_gnss_ns_count_at_pps advance — so by construction
//   its own second_residual_ns should sit very close to zero with
//   very small stddev.  Any non-zero mean is a diagnostic signal
//   about bridge / DWT bookkeeping rather than about the reference
//   clock itself.  (Note: the DWT↔GNSS bridge anchor is now authored
//   by pps_edge_callback from priority-0 PPS GPIO captures, not by
//   VCLOCK — that path was retired because CH3 is preemptible.)
//
// ============================================================================

#pragma once

#include "config.h"
#include "payload.h"
#include "time.h"
#include "process_interrupt.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>

// ============================================================================
// DWT register defines
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)
#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// ============================================================================
// DWT nanosecond conversion helpers
// ============================================================================

static inline uint64_t dwt_cycles_to_ns(uint64_t cycles) {
  return (cycles * DWT_NS_NUM) / DWT_NS_DEN;
}

static inline uint64_t dwt_ns_to_cycles(uint64_t ns) {
  return (ns * DWT_NS_DEN) / DWT_NS_NUM;
}

// ============================================================================
// Always-on DWT↔GNSS anchor state (alpha-owned, beta-readable)
//
// All five of these globals are authored by alpha::pps_edge_callback
// from priority-0 GPIO ISR captures (post-refactor).  They are
// preemption-proof: the captures are taken at the FIRST INSTRUCTION
// of the GPIO ISR, and foreground reads of snap data later are
// transparent to that capture.
// ============================================================================

// Synthetic GNSS ns counter: 0 at epoch install, +1e9 per VCLOCK
// one-second event.  Represents "GNSS ns at the PPS moment this
// second corresponds to."  No physical capture — pure arithmetic.
extern volatile uint64_t g_gnss_ns_count_at_pps;

// Canonical DWT_CYCCNT coordinate of the most recent PPS epoch
// (snap.dwt_at_edge).  Under the VCLOCK-domain architecture this is the
// selected VCLOCK edge after the physical PPS pulse, not the raw GPIO ISR
// capture.
extern volatile uint32_t g_dwt_cycle_count_at_pps;

// Cumulative sum of dwt_cycles_between_pps across the campaign.
// Advanced in pps_edge_callback by the latest measurement.
extern volatile uint64_t g_dwt_cycle_count_total;

// Most recent one-second measurement of DWT cycles between consecutive
// canonical PPS epochs (snap[n].dwt_at_edge - snap[n-1].dwt_at_edge).
// These epochs are VCLOCK-domain edges selected by physical PPS pulses.
// Feeds dwt_effective_cycles_per_second().
extern volatile uint32_t g_dwt_cycle_count_between_pps;

// Prediction residual (diagnostic).  Signed difference between the
// current second's measured DWT cycles and the previous second's
// measured cycles.  The time.cpp bridge's implicit one-step
// predictor for second N is "rate[N] = rate[N-1]", so this IS the
// measured - predicted error of that predictor.  Near zero in
// steady state; reveals thermal drift rate and per-second anomalies.
// Zero until two consecutive measurements exist since epoch install.
extern volatile int32_t  g_dwt_prediction_residual_cycles;

// Cascaded 32-bit QTimer1 CH0/CH1 counter identity of the most recent
// canonical PPS epoch (snap.counter32_at_edge).  This is the VCLOCK
// hardware-counter position selected by the physical PPS pulse.
extern volatile uint32_t g_qtimer_at_pps;

// Diagnostic — last seen counter32_at_event (CH3 ISR capture).
// Distinct from g_qtimer_at_pps: this one IS authored from the
// CH3 ISR's event.counter32_at_event, retained for VCLOCK-phase
// cross-checks; its name honestly says "event" not "pps."
extern volatile uint32_t g_last_pps_event_counter32_at_event;

// Returns the most recent one-second measurement of DWT cycles
// between consecutive PPS GPIO edges.  Semantic label for callers
// that want the "effective cycles per second" framing (the DWT↔GNSS
// bridge in time.cpp is the primary consumer).
static inline uint32_t dwt_effective_cycles_per_second(void) {
  return g_dwt_cycle_count_between_pps;
}

// ============================================================================
// Clock common tolerance
// ============================================================================
//
// Window-error tolerance applies symmetrically to all measured clocks
// (VCLOCK, OCXO1, OCXO2).  A window-error magnitude exceeding this
// constant on the bridge-derived between-edges interval increments
// window_mismatches.

static constexpr int64_t CLOCK_WINDOW_TOLERANCE_NS = 500LL;

// ============================================================================
// Clock state — uniform shape for all measured clocks (VCLOCK, OCXO*)
// ============================================================================
//
// VCLOCK and OCXO1/OCXO2 use field-for-field identical struct shapes.
// Populated by alpha's clocks_apply_edge, called for all three.
//

struct clock_state_t {
  // Cumulative authored ns count at the edge being applied.  Advances
  // by exactly 1e9 each apply_edge call after the first.
  volatile uint64_t ns_count_at_edge;

  // GNSS ns at the edge as reported by the bridge for this event.
  volatile uint64_t gnss_ns_at_edge;

  // Cumulative authored ns count refreshed at PPS (canonical advance,
  // phase-adjusted for OCXOs, equals g_gnss_ns_count_at_pps for VCLOCK).
  volatile uint64_t ns_count_at_pps;

  // (gnss_ns_at_edge − ns_count_at_edge).  For VCLOCK this should be
  // near zero after the first edge.  For OCXO this accumulates the
  // crystal's phase drift from the GNSS reference.
  volatile int64_t  phase_offset_ns;

  // Set true on the first apply_edge call.
  volatile bool     zero_established;

  // Window-check accounting — the window is one bridge-derived
  // between-edges interval.  Positive window_error_ns means this edge
  // came LATE (gnss_ns_between > 1e9, clock running slow).
  volatile uint32_t window_checks;
  volatile uint32_t window_mismatches;
  volatile int64_t  window_error_ns;
};

struct clock_measurement_t {
  // Bridge-derived ns since previous edge minus 1e9.
  // Positive → clock fired early this second (running fast).
  // Negative → clock fired late this second (running slow).
  volatile int64_t  second_residual_ns;

  // Bridge-derived ns elapsed between this edge and the previous one.
  volatile uint64_t gnss_ns_between_edges;

  // ISR-captured DWT at this edge.
  volatile uint32_t dwt_at_edge;

  // DWT cycles elapsed between this edge and the previous one.
  volatile uint32_t dwt_cycles_between_edges;

  // Previous-edge tracking, used to compute the deltas above.
  volatile uint64_t prev_gnss_ns_at_edge;
  volatile uint32_t prev_dwt_at_edge;
};

extern clock_state_t       g_vclock_clock;
extern clock_measurement_t g_vclock_measurement;

extern clock_state_t       g_ocxo1_clock;
extern clock_state_t       g_ocxo2_clock;
extern clock_measurement_t g_ocxo1_measurement;
extern clock_measurement_t g_ocxo2_measurement;

// ============================================================================
// Last-known interrupt diagnostics (alpha-owned, beta-readable)
// ============================================================================

extern interrupt_capture_diag_t g_pps_interrupt_diag;
extern interrupt_capture_diag_t g_ocxo1_interrupt_diag;
extern interrupt_capture_diag_t g_ocxo2_interrupt_diag;

static inline void clocks_capture_interrupt_diag(interrupt_capture_diag_t& dst,
                                                 const interrupt_capture_diag_t* src) {
  if (!src) {
    dst = interrupt_capture_diag_t{};
    dst.enabled = false;
    return;
  }
  dst = *src;
}

// ============================================================================
// AD5693R init
// ============================================================================

extern bool g_ad5693r_init_ok;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;
  uint16_t dac_hw_code;
  uint32_t dac_min;
  uint32_t dac_max;

  double   servo_last_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;

  bool     servo_predictor_initialized;
  double   servo_last_raw_residual;
  double   servo_filtered_residual;
  double   servo_filtered_slope;
  double   servo_predicted_residual;
  uint32_t servo_predictor_updates;

  bool     pacing_pending;
  double   pacing_pending_target;
  double   pacing_pending_step;
  uint16_t pacing_pending_hw_code;
  uint64_t pacing_pending_since_second;
  uint64_t pacing_last_request_second;
  uint64_t pacing_last_commit_second;
  uint32_t pacing_intents;
  uint32_t pacing_deferred_count;
  uint32_t pacing_commit_count;
  uint32_t pacing_skip_small_delta_count;

  bool     io_last_write_ok;
  bool     io_fault_latched;
  uint32_t io_write_attempts;
  uint32_t io_write_successes;
  uint32_t io_write_failures;
  uint16_t io_last_attempted_hw_code;
  uint16_t io_last_good_hw_code;
  uint8_t  io_last_failure_stage;
};

extern ocxo_dac_state_t ocxo1_dac;
extern ocxo_dac_state_t ocxo2_dac;

// ============================================================================
// OCXO servo mode
// ============================================================================

enum class servo_mode_t : uint8_t {
  OFF   = 0,
  TOTAL = 1,
  NOW   = 2,
};

extern servo_mode_t calibrate_ocxo_mode;

const char* servo_mode_str(servo_mode_t mode);
servo_mode_t servo_mode_parse(const char* s);

static constexpr int32_t  SERVO_MAX_STEP                = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS          = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES             = 10;
static constexpr uint16_t SERVO_MIN_DAC_CODE_DELTA_LSB  = 1;

bool ocxo_dac_set(ocxo_dac_state_t& s, double value);
void ocxo_dac_predictor_reset(ocxo_dac_state_t& s);
void ocxo_dac_io_reset(ocxo_dac_state_t& s);
void ocxo_dac_retry_reset(ocxo_dac_state_t& s);

// ============================================================================
// Campaign state
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED
};

extern volatile clocks_campaign_state_t campaign_state;
extern char     campaign_name[64];
extern uint64_t campaign_seconds;

extern volatile bool request_start;
extern volatile bool request_stop;
extern volatile bool request_recover;
extern volatile bool request_zero;

extern uint64_t recover_dwt_ns;
extern uint64_t recover_gnss_ns;
extern uint64_t recover_ocxo1_ns;
extern uint64_t recover_ocxo2_ns;

// ============================================================================
// Welford — unified accumulator
// ============================================================================
//
// One struct, one API, used for every published Welford accumulator.
// Samples are stored in the semantic unit of the signal being measured
// (ppb for frequency clocks, ns for phase offsets, LSB for DAC codes).
//
// Global Welford instances, one per published prefix:
//
//   welford_dwt          — Teensy CPU XTAL offset, in ppb
//   welford_vclock       — bridge interpolation residual, in ns
//   welford_ocxo1        — OCXO1 per-second residual, in ns
//   welford_ocxo2        — OCXO2 per-second residual, in ns
//   welford_pps_witness  — PPS/VCLOCK phase error, in ns
//                          (counter32-based, ordering-independent)
//   welford_ocxo1_dac    — OCXO1 DAC fractional code, in LSB
//   welford_ocxo2_dac    — OCXO2 DAC fractional code, in LSB
//

struct welford_t {
  uint64_t n;
  double   mean;
  double   m2;
  double   min_val;
  double   max_val;
};

extern welford_t welford_dwt;
extern welford_t welford_vclock;
extern welford_t welford_ocxo1;
extern welford_t welford_ocxo2;
extern welford_t welford_pps_witness;
extern welford_t welford_ocxo1_dac;
extern welford_t welford_ocxo2_dac;

void   welford_reset(welford_t& w);
void   welford_update(welford_t& w, double sample);
double welford_stddev(const welford_t& w);
double welford_stderr(const welford_t& w);

// ============================================================================
// Campaign-scoped accumulators
// ============================================================================

extern uint64_t dwt_cycle_count_total;
extern uint64_t gnss_raw_64;
extern uint64_t ocxo1_ticks_64;
extern uint64_t ocxo2_ticks_64;

// ============================================================================
// Watchdog anomaly latch
// ============================================================================

extern volatile bool     watchdog_anomaly_active;
extern volatile bool     watchdog_anomaly_publish_pending;
extern volatile uint32_t watchdog_anomaly_sequence;
extern char              watchdog_anomaly_reason[64];
extern volatile uint32_t watchdog_anomaly_detail0;
extern volatile uint32_t watchdog_anomaly_detail1;
extern volatile uint32_t watchdog_anomaly_detail2;
extern volatile uint32_t watchdog_anomaly_detail3;
extern volatile uint32_t watchdog_anomaly_trigger_dwt;

// ============================================================================
// Beta entry points
// ============================================================================

void clocks_beta_pps(void);
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0 = 0,
                             uint32_t detail1 = 0,
                             uint32_t detail2 = 0,
                             uint32_t detail3 = 0);

// ============================================================================
// Alpha accessors visible to beta
// ============================================================================

// Returns the state of alpha's PPS-anchored epoch install handshake.
// Beta waits on this transitioning to false during ZERO/START.  When
// false, the canonical clock state has been aligned to a physical PPS
// edge.
bool clocks_epoch_pending(void);

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void);