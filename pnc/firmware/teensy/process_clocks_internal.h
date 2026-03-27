#pragma once

// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// v24: Servo mode enum (MEAN/TOTAL) replaces bool calibrate_ocxo_active.
//
//   calibrate_ocxo_active (bool) is replaced by calibrate_ocxo_mode
//   (servo_mode_t enum).  MEAN targets Welford mean residual → 0.
//   TOTAL targets cumulative campaign tick deficit → 0 (tau → 1.0).
//
// v23: 10 MHz single-edge QTimer1 migration.
//
//   GNSS_EDGE_DIVISOR eliminated.  QTimer1 now counts rising edges
//   only (CM=1, 10 MHz).  The raw counter IS the 10 MHz tick count.
//   ISR_GNSS_RAW_EXPECTED = TICKS_10MHZ_PER_SECOND = 10,000,000.
//   gnss_ticks_64_get() returns gnss_raw_64 directly.
//
// v20: PPS rejection recovery watchdog diagnostics.
// v19: Spin capture with shadow-write TDC + timeout protection.
// v14: Symmetric GPT architecture for both OCXOs.
//
// ============================================================================

#include "config.h"
#include "payload.h"
#include "timepop.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ============================================================================
// DWT register defines (needed by both alpha and beta)
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
// OCXO phase bias maps (empirical, topology-driven)
//
// These literals intentionally live in the internal header so bracket/bias
// tuning is centralized and visible.  Alpha uses these maps to translate a
// raw bracket width (in DWT cycles) into a phase-domain bias (in ns).
//
// Design notes:
//   - Classification stays in cycles because bracket topology is a DWT-loop
//     phenomenon.
//   - Bias is expressed in nanoseconds because phase math lives in GNSS ns.
//   - Unknown bracket classes intentionally fall back to 0 ns so analyzer
//     tooling can surface new modes without hiding them.
//   - Arrays are used instead of switch statements so additional cases can be
//     added without changing lookup structure.
// ============================================================================

struct phase_bias_rule_t {
  uint32_t bracket_cycles;
  int32_t  phase_bias_ns;
};

// Separate phase-local bias maps are intentional. Phase #1 and phase #2 are
// observed under different loop geometry, so each phase gets its own bracket→bias
// table. Arrays are open-ended: add more {cycles,bias_ns} pairs as new stable
// modes emerge.
static constexpr phase_bias_rule_t OCXO_PHASE1_BIAS_MAP[] = {
  {274u,  0},
  {278u, -4},
};

static constexpr phase_bias_rule_t OCXO_PHASE2_BIAS_MAP[] = {
  {243u,  0},
  {255u,  0},
  {282u,  0},
};

static constexpr size_t OCXO_PHASE1_BIAS_MAP_COUNT =
    sizeof(OCXO_PHASE1_BIAS_MAP) / sizeof(OCXO_PHASE1_BIAS_MAP[0]);
static constexpr size_t OCXO_PHASE2_BIAS_MAP_COUNT =
    sizeof(OCXO_PHASE2_BIAS_MAP) / sizeof(OCXO_PHASE2_BIAS_MAP[0]);

static inline int32_t phase_bias_ns_lookup(const phase_bias_rule_t* rules,
                                           size_t rule_count,
                                           uint32_t bracket_cycles) {
  for (size_t i = 0; i < rule_count; ++i) {
    if (rules[i].bracket_cycles == bracket_cycles) {
      return rules[i].phase_bias_ns;
    }
  }
  return 0;
}

static inline int32_t phase_bias_from_bracket_cycles(uint8_t phase_ordinal,
                                                     uint32_t bracket_cycles) {
  if (phase_ordinal == 1u) {
    return phase_bias_ns_lookup(OCXO_PHASE1_BIAS_MAP,
                                OCXO_PHASE1_BIAS_MAP_COUNT,
                                bracket_cycles);
  }
  if (phase_ordinal == 2u) {
    return phase_bias_ns_lookup(OCXO_PHASE2_BIAS_MAP,
                                OCXO_PHASE2_BIAS_MAP_COUNT,
                                bracket_cycles);
  }
  return 0;
}

// ============================================================================
// Continuous DWT-to-GNSS Calibration (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t  g_dwt_at_last_pps;
extern volatile uint32_t  g_dwt_cycles_per_gnss_s;
extern volatile bool      g_dwt_cal_has_prev;
extern volatile bool      g_dwt_cal_valid;
extern volatile uint64_t  g_dwt_cal_pps_count;

// ============================================================================
// AD5693R init
// ============================================================================

extern bool g_ad5693r_init_ok;

// ============================================================================
// ISR-level hardware snapshots (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t isr_snap_dwt;
extern volatile uint32_t isr_snap_gnss;
extern volatile uint32_t isr_snap_ocxo1;
extern volatile uint32_t isr_snap_ocxo2;

extern volatile uint32_t isr_prev_dwt;
extern volatile uint32_t isr_prev_gnss;
extern volatile uint32_t isr_prev_ocxo1;
extern volatile uint32_t isr_prev_ocxo2;

extern volatile int32_t  isr_residual_dwt;
extern volatile int32_t  isr_residual_gnss;
extern volatile int32_t  isr_residual_ocxo1;
extern volatile int32_t  isr_residual_ocxo2;
extern volatile bool     isr_residual_valid;

// ============================================================================
// ISR constants
// ============================================================================

static constexpr uint32_t ISR_DWT_EXPECTED  = DWT_EXPECTED_PER_PPS;

// v23: GNSS QTimer1 now counts rising edges only — 10 MHz.
// Previously dual-edge (20 MHz) with GNSS_EDGE_DIVISOR = 2.
static constexpr uint32_t ISR_GNSS_RAW_EXPECTED = TICKS_10MHZ_PER_SECOND;

// Both OCXOs: GPT single-edge — 10 MHz expected
static constexpr uint32_t ISR_OCXO1_EXPECTED = TICKS_10MHZ_PER_SECOND;
static constexpr uint32_t ISR_OCXO2_EXPECTED = TICKS_10MHZ_PER_SECOND;

// ============================================================================
// PPS diagnostics (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t diag_pps_rejected_total;
extern volatile uint32_t diag_pps_rejected_remainder;
extern volatile uint32_t diag_pps_scheduled_stuck;
extern volatile uint32_t diag_pps_watchdog_recoveries;
extern volatile uint32_t diag_pps_asap_arm_failures;
extern volatile uint32_t diag_pps_asap_armed;
extern volatile uint32_t diag_pps_asap_dispatched;
extern volatile uint32_t diag_pps_stuck_since_dwt;
extern volatile uint32_t diag_pps_stuck_max;

// ── PPS rejection recovery diagnostics (v20) ──
extern volatile uint32_t diag_pps_reject_consecutive;
extern volatile uint32_t diag_pps_reject_recoveries;
extern volatile uint32_t diag_pps_reject_max_run;

// ============================================================================
// QTimer1 read diagnostics (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t diag_qread_total;
extern volatile uint32_t diag_qread_same_hi;
extern volatile uint32_t diag_qread_retry_hi_changed;

extern volatile uint32_t diag_qread_last_hi1;
extern volatile uint32_t diag_qread_last_hi2;
extern volatile uint32_t diag_qread_last_lo;
extern volatile uint32_t diag_qread_last_lo2;

// ============================================================================
// PPS state
// ============================================================================

extern volatile bool pps_fired;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;      // servo works in double, truncated to uint16_t at I2C write
  uint16_t dac_hw_code;         // actual integer code last written to AD5693R
  uint32_t dac_min;             // 0
  uint32_t dac_max;             // 65535
  double   servo_last_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;
};

extern ocxo_dac_state_t ocxo1_dac;
extern ocxo_dac_state_t ocxo2_dac;

// ============================================================================
// OCXO servo mode — campaign-level calibration strategy
//
//   OFF:   servo disabled
//   MEAN:  target Welford mean residual → 0 (instantaneous PPB)
//   TOTAL: target cumulative tick deficit → 0 (tau → 1.0)
//   NOW:   phase-capture per-second residual → 0 (~3 ns resolution)
// ============================================================================

enum class servo_mode_t : uint8_t {
  OFF   = 0,
  MEAN  = 1,
  TOTAL = 2,
  NOW   = 3,
};

extern servo_mode_t calibrate_ocxo_mode;

// Helper: returns the mode as a string for telemetry ("OFF", "MEAN", "TOTAL", "NOW")
const char* servo_mode_str(servo_mode_t mode);

// Helper: parse from command arg string; returns OFF if unrecognized
servo_mode_t servo_mode_parse(const char* s);

static constexpr int32_t  SERVO_MAX_STEP       = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;

void ocxo_dac_set(ocxo_dac_state_t& s, double value);

// ============================================================================
// Campaign state (beta-owned, alpha-readable for ns_now guards)
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

extern uint64_t recover_dwt_ns;
extern uint64_t recover_gnss_ns;
extern uint64_t recover_ocxo1_ns;
extern uint64_t recover_ocxo2_ns;

// ============================================================================
// PPS residual tracking — Welford's, callback-level
// ============================================================================

static constexpr int64_t DWT_EXPECTED_PER_PPS_I = (int64_t)DWT_EXPECTED_PER_PPS;
static constexpr int64_t GNSS_EXPECTED_PER_PPS  = (int64_t)TICKS_10MHZ_PER_SECOND;
static constexpr int64_t OCXO_EXPECTED_PER_PPS  = (int64_t)TICKS_10MHZ_PER_SECOND;

struct pps_residual_t {
  uint64_t ticks_at_last_pps;
  int64_t  delta;
  int64_t  residual;
  bool     valid;
  uint64_t n;
  double   mean;
  double   m2;
};

extern pps_residual_t residual_dwt;
extern pps_residual_t residual_gnss;
extern pps_residual_t residual_ocxo1;
extern pps_residual_t residual_ocxo2;

void residual_reset(pps_residual_t& r);
void residual_update(pps_residual_t& r, uint64_t ticks_now, int64_t expected);
double residual_stddev(const pps_residual_t& r);
double residual_stderr(const pps_residual_t& r);

// ============================================================================
// Trend-aware prediction tracking
// ============================================================================

struct prediction_tracker_t {
  uint32_t prev_delta;
  uint32_t predicted;
  int32_t  pred_residual;
  uint8_t  history_count;
  bool     predicted_valid;
  uint64_t n;
  double   mean;
  double   m2;
};

extern prediction_tracker_t pred_dwt;
extern prediction_tracker_t pred_ocxo1;
extern prediction_tracker_t pred_ocxo2;

void prediction_reset(prediction_tracker_t& p);
void prediction_seed(prediction_tracker_t& p, uint32_t value);
void prediction_update(prediction_tracker_t& p, uint32_t actual_delta);
double prediction_stddev(const prediction_tracker_t& p);
double prediction_stderr(const prediction_tracker_t& p);

// ============================================================================
// DAC Welford tracking — campaign-scoped cumulative statistics
//
// Tracks the servo's DAC output value (dac_fractional) every PPS second.
// The mean is the true campaign mean — no windowing, no decay.
// Used by the TEMPEST DAC test to detect environmental drift via
// correlated DAC mean shifts across campaigns.
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
// Spin Capture — nano-precise DWT anchoring with shadow-write TDC
//
// Two distinct timeout failure modes:
//
//   1. Nano-spin timeout: TimePop's DWT spin exceeded its budget.
//      Fields: nano_timed_out (this cycle), nano_timeouts (lifetime count).
//
//   2. Shadow-loop timeout: shadow-write loop exceeded SPIN_LOOP_TIMEOUT_CYCLES.
//      Fields: shadow_timed_out (this cycle), shadow_timeouts (lifetime).
//
// ============================================================================

struct spin_capture_t {
  // ── Set by the arming code (scheduled context) ──
  uint32_t  target_dwt;
  int64_t   target_gnss_ns;
  bool      armed;

  // ── Set by the nano-spin callback (ISR context) ──
  uint32_t  landed_dwt;
  int64_t   landed_gnss_ns;
  int32_t   spin_error;
  uint32_t  shadow_dwt;
  bool      completed;
  bool      nano_timed_out;
  bool      shadow_timed_out;

  // ── Set by the edge handler (scheduled context) ──
  uint32_t  isr_dwt;
  int32_t   delta_cycles;
  int32_t   approach_cycles;
  uint32_t  corrected_dwt;
  int32_t   tdc_correction;
  bool      valid;

  // ── Diagnostics (monotonic, never reset) ──
  uint32_t  arms;
  uint32_t  arm_failures;
  uint32_t  completions;
  uint32_t  nano_timeouts;
  uint32_t  shadow_timeouts;
  uint32_t  misses;
};

extern spin_capture_t pps_spin;

// ============================================================================
// OCXO Phase Capture — raw first, adjusted second
//
// Each PPS defines a canonical GNSS anchor. For each OCXO, alpha captures the
// first two post-PPS 10 MHz edges and publishes a forensic record for both:
//
//   1. Raw observables
//      - dwt_before / dwt_after bracket in DWT cycles
//      - raw_elapsed_ns = ns from PPS anchor to raw delimiter (dwt_before)
//      - raw_phase_offset_ns = raw_elapsed_ns mod 100
//
//   2. Adjusted observables
//      - phase_bias_ns = empirically derived bracket-topology bias
//      - adjusted_phase_signed_ns = raw_phase_offset_ns + phase_bias_ns
//      - phase_offset_ns = wrapped adjusted phase in [0, 99]
//
// The field names dwt_correction_cycles and dwt_at_edge are retained for
// compatibility with existing telemetry/readout code. In the current model:
//
//   dwt_correction_cycles = legacy / forensic only
//   dwt_at_edge           = canonical raw delimiter (currently dwt_before)
//
// Downstream residual math should treat phase_offset_ns as the authoritative
// per-second adjusted phase for servo/control use, while preserving the raw
// fields for forensics and analyzer work.
// ============================================================================

struct ocxo_phase_capture_t {
  // ── PPS anchor ──
  uint32_t dwt_at_pps;               // DWT at PPS edge (ISR-compensated)
  uint64_t pps_gnss_ns;              // absolute GNSS ns at this PPS (beta-owned)

  // ── OCXO1 phase #1 (first observed post-PPS edge) ──
  uint32_t ocxo1_dwt_before;
  uint32_t ocxo1_dwt_after;
  uint32_t ocxo1_gpt_before;
  uint32_t ocxo1_gpt_after;
  uint32_t ocxo1_dwt_bracket_cycles;
  uint32_t ocxo1_dwt_correction_cycles;      // legacy / forensic only
  uint32_t ocxo1_dwt_at_edge;                // canonical raw delimiter (currently dwt_before)
  uint32_t ocxo1_dwt_elapsed;                // legacy mirror of raw delimiter elapsed cycles
  uint32_t ocxo1_raw_elapsed_ns;
  uint32_t ocxo1_raw_phase_offset_ns;
  int32_t  ocxo1_phase_bias_ns;
  int32_t  ocxo1_adjusted_phase_signed_ns;
  uint32_t ocxo1_phase_offset_ns;            // wrapped adjusted phase offset [0,99]
  uint64_t ocxo1_edge_gnss_ns;               // pps_gnss_ns + adjusted phase offset

  // ── OCXO1 phase #2 (second observed edge, one more 10 MHz window later) ──
  uint32_t ocxo1_phase2_dwt_before;
  uint32_t ocxo1_phase2_dwt_after;
  uint32_t ocxo1_phase2_gpt_before;
  uint32_t ocxo1_phase2_gpt_after;
  uint32_t ocxo1_phase2_dwt_bracket_cycles;
  uint32_t ocxo1_phase2_dwt_correction_cycles;   // legacy / forensic only
  uint32_t ocxo1_phase2_dwt_at_edge;             // canonical raw delimiter (currently dwt_before)
  uint32_t ocxo1_phase2_dwt_elapsed;             // legacy mirror of raw delimiter elapsed cycles
  uint32_t ocxo1_phase2_raw_elapsed_ns;
  uint32_t ocxo1_phase2_raw_phase_offset_ns;
  int32_t  ocxo1_phase2_phase_bias_ns;
  int32_t  ocxo1_phase2_adjusted_phase_signed_ns;
  uint32_t ocxo1_phase2_phase_offset_ns;         // wrapped adjusted phase offset [0,99]
  uint64_t ocxo1_phase2_edge_gnss_ns;
  int32_t  ocxo1_phase_pair_delta_ns;            // raw wrapped delta: phase2_raw - phase1_raw
  int32_t  ocxo1_adjusted_phase_pair_delta_ns;   // adjusted wrapped delta: phase2_adj - phase1_adj

  // ── OCXO2 phase #1 ──
  uint32_t ocxo2_dwt_before;
  uint32_t ocxo2_dwt_after;
  uint32_t ocxo2_gpt_before;
  uint32_t ocxo2_gpt_after;
  uint32_t ocxo2_dwt_bracket_cycles;
  uint32_t ocxo2_dwt_correction_cycles;      // legacy / forensic only
  uint32_t ocxo2_dwt_at_edge;                // canonical raw delimiter (currently dwt_before)
  uint32_t ocxo2_dwt_elapsed;                // legacy mirror of raw delimiter elapsed cycles
  uint32_t ocxo2_raw_elapsed_ns;
  uint32_t ocxo2_raw_phase_offset_ns;
  int32_t  ocxo2_phase_bias_ns;
  int32_t  ocxo2_adjusted_phase_signed_ns;
  uint32_t ocxo2_phase_offset_ns;            // wrapped adjusted phase offset [0,99]
  uint64_t ocxo2_edge_gnss_ns;               // pps_gnss_ns + adjusted phase offset

  // ── OCXO2 phase #2 ──
  uint32_t ocxo2_phase2_dwt_before;
  uint32_t ocxo2_phase2_dwt_after;
  uint32_t ocxo2_phase2_gpt_before;
  uint32_t ocxo2_phase2_gpt_after;
  uint32_t ocxo2_phase2_dwt_bracket_cycles;
  uint32_t ocxo2_phase2_dwt_correction_cycles;   // legacy / forensic only
  uint32_t ocxo2_phase2_dwt_at_edge;             // canonical raw delimiter (currently dwt_before)
  uint32_t ocxo2_phase2_dwt_elapsed;             // legacy mirror of raw delimiter elapsed cycles
  uint32_t ocxo2_phase2_raw_elapsed_ns;
  uint32_t ocxo2_phase2_raw_phase_offset_ns;
  int32_t  ocxo2_phase2_phase_bias_ns;
  int32_t  ocxo2_phase2_adjusted_phase_signed_ns;
  uint32_t ocxo2_phase2_phase_offset_ns;         // wrapped adjusted phase offset [0,99]
  uint64_t ocxo2_phase2_edge_gnss_ns;
  int32_t  ocxo2_phase_pair_delta_ns;            // raw wrapped delta: phase2_raw - phase1_raw
  int32_t  ocxo2_adjusted_phase_pair_delta_ns;   // adjusted wrapped delta: phase2_adj - phase1_adj

  // ── Two-phase validation ──
  bool     phase_pair_valid;         // true when phase #1 / phase #2 comparison is valid

  // ── Per-second differencing (legacy downstream path; still published) ──
  uint64_t prev_ocxo1_edge_gnss_ns;
  uint64_t prev_ocxo2_edge_gnss_ns;
  int64_t  ocxo1_gnss_ns_per_pps;
  int64_t  ocxo2_gnss_ns_per_pps;
  int64_t  ocxo1_residual_ns;
  int64_t  ocxo2_residual_ns;
  bool     residual_valid;

  // ── Servo before/after snapshot (written by beta after servo runs) ──
  uint16_t ocxo1_dac_before;
  uint16_t ocxo1_dac_after;
  uint16_t ocxo2_dac_before;
  uint16_t ocxo2_dac_after;

  // ── Lifecycle ──
  bool     valid;
  uint32_t captures;
};

extern ocxo_phase_capture_t ocxo_phase;

// ============================================================================
// 64-bit accumulators (campaign-scoped)
// ============================================================================

extern uint64_t dwt_cycles_64;
extern uint32_t prev_dwt_at_pps;

extern uint64_t ocxo1_ticks_64;
extern uint32_t prev_ocxo1_at_pps;

extern uint64_t ocxo2_ticks_64;
extern uint32_t prev_ocxo2_at_pps;

extern uint64_t gnss_raw_64;
extern uint32_t prev_gnss_at_pps;

// v23: gnss_raw_64 IS 10 MHz ticks — no divisor needed.
inline uint64_t gnss_ticks_64_get(void) {
  return gnss_raw_64;
}

// ============================================================================
// Rolling 64-bit extensions
// ============================================================================

extern uint64_t dwt_rolling_64;
extern uint32_t dwt_rolling_32;

extern uint64_t gnss_rolling_raw_64;
extern uint32_t gnss_rolling_32;

extern uint64_t ocxo1_rolling_64;
extern uint32_t ocxo1_rolling_32;

extern uint64_t ocxo2_rolling_64;
extern uint32_t ocxo2_rolling_32;

// ============================================================================
// QTimer1 read helper (alpha-owned hardware, used for GNSS)
// ============================================================================

uint32_t qtimer1_read_32(void);

// ============================================================================
// Relay state (alpha-owned)
// ============================================================================

extern volatile bool relay_arm_pending;
extern volatile bool relay_timer_active;

extern volatile uint32_t dbg_post_loop_dwt;
extern volatile uint32_t dbg_post_loop_shadow;
extern volatile uint32_t dbg_post_loop_isr_cap;
extern volatile uint32_t dbg_post_loop_isr_snap;

// ============================================================================
// PPS callback scheduling (alpha-owned)
// ============================================================================

extern volatile bool pps_scheduled;

// ============================================================================
// Watchdog anomaly latch (beta-owned, alpha-readable)
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
// Beta entry points — called from alpha / commands
// ============================================================================

void clocks_beta_pps(void);
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0 = 0,
                             uint32_t detail1 = 0,
                             uint32_t detail2 = 0,
                             uint32_t detail3 = 0);

// ============================================================================
// Zeroing (called by beta campaign commands, reads alpha ISR state)
// ============================================================================

void clocks_zero_all(void);