// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// Doctrine note (post rolling-integration removal):
//
//   - g_dwt_cycle_count_at_pps       is event.dwt_at_event from the most
//                                    recent VCLOCK one-second event — the
//                                    ISR's first-instruction DWT capture
//                                    on the 1000th tick of the VCLOCK
//                                    1 kHz cadence.  Honest fact; no
//                                    synthesis.
//
//   - g_dwt_cycle_count_between_pps  is the one-second subtraction of
//                                    consecutive event DWT captures,
//                                    computed in alpha's vclock_callback.
//                                    Wraps correctly across DWT_CYCCNT
//                                    rollover (~4.26 s at 1008 MHz).
//
//   - g_dwt_cycle_count_next_second_prediction  is just a mirror of
//                                               g_dwt_cycle_count_between_pps;
//                                               the measured delta is
//                                               its own best prediction.
//
//   - g_dwt_cycle_count_next_second_adjustment  is vestigial (always 0).
//                                               Retained only so the
//                                               dwt_effective_cycles_per_second
//                                               helper's add signature
//                                               stays stable.
//
//   - g_qtimer_at_pps                is event.counter32_at_event — the
//                                    authored 32-bit VCLOCK count that
//                                    advances by exactly 10,000,000 per
//                                    VCLOCK one-second event.  Not an
//                                    ambient counter read.
//
// Doctrine note (VCLOCK as measured peer of OCXO):
//
//   VCLOCK is now treated as a real measured clock peer to OCXO1/OCXO2.
//   Its measurement state (vclock_clock_state_t / vclock_measurement_t)
//   is field-for-field identical in shape to ocxo_clock_state_t /
//   ocxo_measurement_t.  Per-event measurement runs through
//   vclock_apply_edge() in alpha, parallel to ocxo_apply_edge().
//
//   Self-test property:
//     Because the VCLOCK lane drives the bridge anchor itself, the
//     bridge-derived per-event delta should always equal exactly 1e9
//     (modulo DWT measurement noise and dwt_effective_cycles_per_second
//     estimation noise).  Any non-zero second_residual_ns is a
//     diagnostic signal about DWT bookkeeping rather than about the
//     clock itself.
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
// ============================================================================

extern volatile uint64_t g_gnss_ns_count_at_pps;

extern volatile uint32_t g_dwt_cycle_count_at_pps;
extern volatile uint64_t g_dwt_cycle_count_total;
extern volatile uint32_t g_dwt_cycle_count_between_pps;
extern volatile uint32_t g_dwt_cycle_count_next_second_prediction;
extern volatile int32_t  g_dwt_cycle_count_next_second_adjustment;  // vestigial; always 0
extern volatile uint64_t g_dwt_model_pps_count;

// Authored 32-bit VCLOCK count at the most recent VCLOCK one-second
// event that drove vclock_callback.
extern volatile uint32_t g_qtimer_at_pps;

// Diagnostics.
extern volatile uint32_t g_last_pps_event_counter32_at_event;

// ============================================================================
// VCLOCK / OCXO common tolerance
// ============================================================================
//
// Window-error tolerance applies symmetrically to all measured clocks
// (VCLOCK, OCXO1, OCXO2).  A window-error magnitude exceeding this
// constant on the bridge-derived between-edges interval increments
// window_mismatches.

static constexpr int64_t CLOCK_WINDOW_TOLERANCE_NS = 500LL;

// Backward-compat alias for existing references.
static constexpr int64_t OCXO_WINDOW_TOLERANCE_NS = CLOCK_WINDOW_TOLERANCE_NS;

// ============================================================================
// VCLOCK canonical state — symmetric with OCXO
// ============================================================================
//
// VCLOCK is a measured peer of OCXO1/OCXO2.  These structs are
// field-for-field identical in shape to ocxo_clock_state_t /
// ocxo_measurement_t, populated by vclock_apply_edge() in alpha.
//

struct vclock_clock_state_t {
  // Cumulative authored ns count at the VCLOCK edge being applied
  // (advances by exactly 1e9 each call to vclock_apply_edge after the
  // first).  Same role as ocxo_clock_state_t::ns_count_at_edge.
  volatile uint64_t ns_count_at_edge;

  // GNSS ns at the edge as reported by the bridge for this VCLOCK
  // event.  Carried verbatim from event.gnss_ns_at_event.
  volatile uint64_t gnss_ns_at_edge;

  // Cumulative authored ns count refreshed at PPS (canonical advance).
  volatile uint64_t ns_count_at_pps;

  // (gnss_ns_at_edge − ns_count_at_edge).  For VCLOCK this should be
  // very close to zero on every edge after the first; non-zero values
  // surface bridge-interpolation drift.
  volatile int64_t  phase_offset_ns;

  // Set true on the first vclock_apply_edge call.
  volatile bool     zero_established;

  // Window-check accounting (window = one bridge-derived between-edges
  // interval).  Mirrors OCXO semantics.
  volatile uint32_t window_checks;
  volatile uint32_t window_mismatches;
  volatile int64_t  window_expected_ns;
  volatile int64_t  window_actual_ns;
  volatile int64_t  window_error_ns;
};

extern vclock_clock_state_t g_vclock_clock;

struct vclock_measurement_t {
  // Bridge-derived ns since previous edge minus 1e9.  Self-test signal:
  // for VCLOCK this should be near zero, surfacing only DWT bookkeeping
  // noise.  Same role as ocxo_measurement_t::second_residual_ns.
  volatile int64_t  second_residual_ns;

  // Bridge-derived ns elapsed between this edge and the previous one.
  volatile uint64_t gnss_ns_between_edges;

  // ISR-captured DWT at this edge.
  volatile uint32_t dwt_at_edge;

  // DWT cycles elapsed between this edge and the previous one.  Same
  // semantics as ocxo_measurement_t::dwt_cycles_between_edges.  Note
  // that g_dwt_cycle_count_between_pps tracks the same physical
  // quantity through a separate path (one-second subtraction in
  // vclock_callback) and is preserved for legacy callers.
  volatile uint32_t dwt_cycles_between_edges;

  // Previous-edge tracking, used to compute the deltas above.
  volatile uint64_t prev_gnss_ns_at_edge;
  volatile uint32_t prev_dwt_at_edge;
};

extern vclock_measurement_t g_vclock_measurement;

// ============================================================================
// OCXO canonical state — smart-zero phase model (unchanged)
// ============================================================================

struct ocxo_clock_state_t {
  volatile uint64_t ns_count_at_edge;
  volatile uint64_t gnss_ns_at_edge;
  volatile uint64_t ns_count_at_pps;
  volatile int64_t  phase_offset_ns;
  volatile bool     zero_established;

  volatile uint32_t window_checks;
  volatile uint32_t window_mismatches;
  volatile int64_t  window_expected_ns;
  volatile int64_t  window_actual_ns;
  volatile int64_t  window_error_ns;
};

extern ocxo_clock_state_t g_ocxo1_clock;
extern ocxo_clock_state_t g_ocxo2_clock;

struct ocxo_measurement_t {
  volatile int64_t  second_residual_ns;
  volatile uint64_t gnss_ns_between_edges;
  volatile uint32_t dwt_at_edge;
  volatile uint32_t dwt_cycles_between_edges;
  volatile uint64_t prev_gnss_ns_at_edge;
  volatile uint32_t prev_dwt_at_edge;
};

extern ocxo_measurement_t g_ocxo1_measurement;
extern ocxo_measurement_t g_ocxo2_measurement;

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
// Simple helpers
// ============================================================================

static inline uint32_t dwt_effective_cycles_per_second(void) {
  return (uint32_t)((int64_t)g_dwt_cycle_count_next_second_prediction +
                    (int64_t)g_dwt_cycle_count_next_second_adjustment);
}

// ============================================================================
// AD5693R init
// ============================================================================

extern bool g_ad5693r_init_ok;

// ============================================================================
// OCXO DAC state — dual oscillator (unchanged)
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
// OCXO servo mode (unchanged)
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
// Residual tracking
// ============================================================================
//
// residual_dwt          — DWT residual: (measured cycles between VCLOCK
//                         one-second events) - DWT_EXPECTED_PER_PPS.
// residual_vclock       — VCLOCK measurement residual: bridge-derived
//                         ns-between-edges minus 1e9.  Self-test for
//                         the bridge interpolation.  Should be near
//                         zero with very small stddev.
// residual_pps_witness  — GPIO PPS witness offset from the authored
//                         g_gnss_ns_count_at_pps, accumulated from
//                         pps_edge_callback.  Welford mean reveals DC
//                         bias (boot-determined V→PPS phase + the
//                         epoch-install off-by-one); stddev reveals
//                         GPIO detection latency jitter.
// residual_ocxo1        — OCXO1 second-to-second drift residual.
// residual_ocxo2        — OCXO2 second-to-second drift residual.
//

struct pps_residual_t {
  int64_t  residual;
  uint64_t n;
  double   mean;
  double   m2;
  double   min_val;
  double   max_val;
};

extern pps_residual_t residual_dwt;
extern pps_residual_t residual_vclock;
extern pps_residual_t residual_pps_witness;
extern pps_residual_t residual_ocxo1;
extern pps_residual_t residual_ocxo2;

void residual_reset(pps_residual_t& r);
void residual_update_sample(pps_residual_t& r, int64_t residual);
double residual_stddev(const pps_residual_t& r);
double residual_stderr(const pps_residual_t& r);

// ============================================================================
// DAC Welford tracking (unchanged)
// ============================================================================

struct dac_welford_t {
  uint64_t n;
  double   mean;
  double   m2;
  double   min_val;
  double   max_val;
};

extern dac_welford_t dac_welford_ocxo1;
extern dac_welford_t dac_welford_ocxo2;

void dac_welford_reset(dac_welford_t& w);
void dac_welford_update(dac_welford_t& w, double value);
double dac_welford_stddev(const dac_welford_t& w);
double dac_welford_stderr(const dac_welford_t& w);

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
// Zeroing
// ============================================================================

void clocks_zero_all(void);