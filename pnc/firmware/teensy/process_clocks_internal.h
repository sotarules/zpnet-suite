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
// Architecture (post-scorched-earth):
//
//   process_interrupt is a stateless ISR custodian.  It captures _raw,
//   latency-adjusts to event-coordinate DWT, computes gnss_ns_at_edge
//   (sequence × 1e9 for PPS_VCLOCK; bridge-projected for others), and
//   dispatches a tiny `interrupt_event_t` to the registered subscriber.
//
//   Alpha is the keeper of the clocks.  It subscribes to PPS_VCLOCK,
//   VCLOCK, OCXO1, OCXO2 — receiving {gnss_ns_at_edge, dwt_at_edge,
//   sequence} on each.  It maintains the canonical PPS_VCLOCK slot
//   (read by time.cpp's bridge), the canonical rate
//   (g_dwt_cycle_count_between_pps), and all per-clock state.
//
//   time.cpp is three stateless functions reading alpha's slot.
//
// Epoch authority:
//
//   Alpha calls interrupt_request_pps_rebootstrap() at INIT/ZERO/START
//   and sets g_epoch_pending.  The next physical PPS GPIO edge:
//     • Re-phases the VCLOCK CH3 cadence (in-ISR).
//     • Resets process_interrupt's PPS_VCLOCK sequence to 0 so the
//       dispatched event carries gnss_ns_at_edge = 0.
//     • Dispatches the PPS_VCLOCK event to alpha.
//   Alpha's pps_vclock_callback installs the epoch from the event:
//     g_dwt_cycle_count_at_pps  = event.dwt_at_edge
//     g_gnss_ns_count_at_pps    = 0
//   And publishes the PPS_VCLOCK slot for time.cpp consumption.
//
// Sign convention:
//
//   positive ppb  →  this clock is RUNNING FAST vs the GNSS reference
//   negative ppb  →  this clock is RUNNING SLOW vs the GNSS reference
//
//   Holds for DWT, VCLOCK, OCXO1, OCXO2.  Residual-ns mean has the same
//   sign semantics: positive mean → clock fast (fires early).
//
// Authorship map:
//
//   - g_gnss_ns_count_at_pps      pure synthetic counter.  Set to 0 at
//                                 epoch install (PPS edge IS time zero).
//                                 Advanced by exactly +1e9 per VCLOCK
//                                 one-second event.
//   - g_dwt_cycle_count_at_pps    event.dwt_at_edge from PPS_VCLOCK
//                                 callback.  Canonical PPS DWT.
//   - g_dwt_cycle_count_between_pps  Difference of consecutive
//                                    event.dwt_at_edge values across two
//                                    PPS_VCLOCK edges.  This IS the rate
//                                    that time.cpp consumes via
//                                    alpha_dwt_cycles_per_second().
//   - g_dwt_cycle_count_total     Cumulative sum of dwt_between_pps.
//
// VCLOCK as measured peer of OCXO:
//
//   Same clock_state_t / clock_measurement_t / clocks_apply_edge body
//   for VCLOCK, OCXO1, OCXO2.  VCLOCK occupies a special position: it
//   is both the GNSS-locked 10 MHz timebase that authors VCLOCK_COUNTS_
//   PER_SECOND ticks per second, AND a peer clock being measured.  Its
//   own second_residual_ns should sit very close to zero.  Any non-zero
//   mean is a diagnostic signal about bridge/DWT bookkeeping.
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
// Always-on canonical anchor state (alpha-owned, beta-readable)
// ============================================================================

// Synthetic GNSS ns counter: 0 at epoch install, +1e9 per VCLOCK
// one-second event.  Represents "GNSS ns at the PPS moment this
// second corresponds to."  No physical capture — pure arithmetic.
extern volatile uint64_t g_gnss_ns_count_at_pps;

// Canonical DWT_CYCCNT coordinate of the most recent PPS_VCLOCK edge
// (event.dwt_at_edge from the PPS_VCLOCK callback).
extern volatile uint32_t g_dwt_cycle_count_at_pps;

// Cumulative sum of dwt_between_pps across the campaign.
extern volatile uint64_t g_dwt_cycle_count_total;

// Most recent one-second measurement of DWT cycles between consecutive
// PPS_VCLOCK edges.  This IS the rate consumed by time.cpp's bridge via
// alpha_dwt_cycles_per_second().
extern volatile uint32_t g_dwt_cycle_count_between_pps;

// Prediction residual (diagnostic).  Signed difference between this
// second's measured cycles and the previous second's measured cycles.
// Near zero in steady state; reveals thermal drift and per-second
// anomalies.
extern volatile int32_t  g_dwt_prediction_residual_cycles;

// ============================================================================
// Last-known events (alpha-owned, beta-readable, foreground-only writers)
// ============================================================================
//
// One slot per subscription.  Beta reads these for fragment publication.
// Single-threaded foreground writers — no seqlock.

extern interrupt_event_t g_last_pps_vclock_event;
extern interrupt_event_t g_last_vclock_event;
extern interrupt_event_t g_last_ocxo1_event;
extern interrupt_event_t g_last_ocxo2_event;

// ============================================================================
// Clock common tolerance
// ============================================================================

static constexpr int64_t CLOCK_WINDOW_TOLERANCE_NS = 500LL;

// ============================================================================
// Clock state — uniform shape for all measured clocks (VCLOCK, OCXO*)
// ============================================================================

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

  // Event-coordinate DWT at this edge.
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

bool clocks_epoch_pending(void);

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void);