// ============================================================================
// process_clocks_internal.h - Shared Internal State (Alpha <-> Beta)
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
//   PPS is a witness and selector.  process_interrupt observes the physical
//   PPS edge, selects/authors the corresponding PPS/VCLOCK edge, and publishes
//   a snapshot whose synthetic counter32 identity already matches the 64-bit
//   nanosecond ledger.
//
//   Alpha installs that selected PPS/VCLOCK snapshot directly:
//     g_gnss_ns_at_pps_vclock = 0
//     g_dwt_at_pps_vclock = snap.dwt_at_edge
//     g_counter32_at_pps_vclock = snap.counter32_at_edge
//
//   After install, VCLOCK events advance the synthetic ns ledger by +1e9 and
//   subsequent PPS-selected PPS/VCLOCK snapshots refresh the DWT/GNSS bridge
//   anchor.  Raw physical PPS facts remain diagnostics.
//
// Statistical surface (standardized):
//
//   Every Welford accumulator published in the fragment uses the same
//   suffix set:
//
//     <prefix>_welford_n        - uint64 sample count
//     <prefix>_welford_mean     - double, in the semantic unit of the signal
//     <prefix>_welford_stddev   - double, same unit
//     <prefix>_welford_stderr   - double, same unit (= stddev / sqrt(n))
//     <prefix>_welford_min      - double, same unit
//     <prefix>_welford_max      - double, same unit
//
//   Published Welford prefixes (seven total):
//
//     dwt_welford         - Teensy CPU XTAL offset samples (ppb)
//     vclock_welford      - bridge interpolation residual samples (ns)
//     ocxo1_welford       - OCXO1 per-second residual samples (ns)
//     ocxo2_welford       - OCXO2 per-second residual samples (ns)
//     pps_witness_welford - reserved PPS/VCLOCK phase-error surface (ns)
//     ocxo1_dac_welford   - OCXO1 DAC fractional code samples (LSB)
//     ocxo2_dac_welford   - OCXO2 DAC fractional code samples (LSB)
//
// Authorship map:
//
//   - g_gnss_ns_at_pps_vclock         Pure synthetic counter. Set to 0 at
//                                     epoch install and advanced by exactly
//                                     +1e9 in alpha::vclock_callback per
//                                     VCLOCK one-second event.
//   - g_dwt_at_pps_vclock             DWT coordinate of the selected
//                                     PPS/VCLOCK edge.
//   - g_dwt_cycles_between_pps_vclock Difference of consecutive selected
//                                     PPS/VCLOCK DWT coordinates.
//   - g_counter32_at_pps_vclock       Synthetic VCLOCK counter identity of
//                                     the selected PPS/VCLOCK edge.
//
// VCLOCK as measured peer of OCXO:
//
//   VCLOCK is both the sovereign GNSS-disciplined 10 MHz timebase and a peer
//   clock measured with the same clock_state_t / clock_measurement_t shape as
//   OCXO1 and OCXO2.  Any non-zero VCLOCK residual is diagnostic evidence
//   about bridge / DWT bookkeeping, not about the reference clock itself.
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
// Always-on DWT-GNSS anchor state (alpha-owned, beta-readable)
//
// All five of these globals are authored by alpha::pps_selector_callback
// from priority-0 GPIO ISR captures (post-refactor).  They are
// preemption-proof: the captures are taken at the FIRST INSTRUCTION
// of the GPIO ISR, and foreground reads of snap data later are
// transparent to that capture.
// ============================================================================

// Synthetic GNSS ns counter: 0 at epoch install, +1e9 per VCLOCK
// one-second event.  Represents "GNSS ns at the PPS moment this
// second corresponds to."  No physical capture — pure arithmetic.
extern volatile uint64_t g_gnss_ns_at_pps_vclock;

// Canonical DWT_CYCCNT coordinate of the most recent selected PPS/VCLOCK epoch
// (snap.dwt_at_edge).  Under the VCLOCK-domain architecture this is the
// selected VCLOCK edge after the physical PPS pulse, not the raw GPIO ISR
// capture.
extern volatile uint32_t g_dwt_at_pps_vclock;

// Cumulative sum of dwt_cycles_between_pps across the campaign.
// Advanced in pps_selector_callback by the latest measurement.
extern volatile uint64_t g_dwt_cycle_count_total;

// Most recent one-second measurement of DWT cycles between consecutive
// selected PPS/VCLOCK epochs (snap[n].dwt_at_edge - snap[n-1].dwt_at_edge).
// These epochs are VCLOCK-domain edges selected by physical PPS pulses.
// Feeds dwt_effective_cycles_per_pps_vclock_second().
extern volatile uint32_t g_dwt_cycles_between_pps_vclock;

// process_interrupt-authored synthetic 32-bit VCLOCK identity of the most
// recent canonical PPS/VCLOCK epoch (snap.counter32_at_edge).  This is the compact
// clock identity selected by the physical PPS pulse; it is not an ambient
// hardware read.
extern volatile uint32_t g_counter32_at_pps_vclock;

// Diagnostic — last seen counter32_at_event (CH3 ISR capture).
// Distinct from g_counter32_at_pps_vclock: this one IS authored from the
// CH3 ISR's event.counter32_at_event, retained for VCLOCK-phase
// cross-checks; its name honestly says "event" not "pps."
extern volatile uint32_t g_last_vclock_event_counter32_at_event;

// Returns the most recent one-second measurement of DWT cycles
// between consecutive selected PPS/VCLOCK edges.  Semantic label for callers
// that want the "effective cycles per second" framing (the DWT-GNSS
// bridge in time.cpp is the primary consumer).
static inline uint32_t dwt_effective_cycles_per_pps_vclock_second(void) {
  return g_dwt_cycles_between_pps_vclock;
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

  // Cumulative authored ns count refreshed at PPS/VCLOCK (canonical advance,
  // phase-adjusted for OCXOs, equals g_gnss_ns_at_pps_vclock for VCLOCK).
  volatile uint64_t ns_count_at_pps_vclock;

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

extern interrupt_capture_diag_t g_pps_witness_diag;
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
// Campaign warmup suppression
// ============================================================================
//
// A newly installed/recovered campaign deliberately suppresses the first N
// PPS-driven TIMEBASE_FRAGMENT publications. Alpha continues to measure and
// refine its internal timing state during this quiet period; beta simply
// refuses to call those records canonical campaign output.
//
// For START, public campaign identity begins after warmup: the first emitted
// fragment is teensy_pps_vclock_count=1 and gnss_ns=1e9.
//
// For RECOVER, the suppressed records are treated as real elapsed campaign
// seconds. The first emitted fragment therefore appears after a deliberate
// canonical gap, preserving the recovered absolute PPS identity.

static constexpr uint32_t CLOCKS_CAMPAIGN_WARMUP_SUPPRESS_PPS = 5;

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