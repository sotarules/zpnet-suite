#pragma once

// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
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
// Continuous DWT-to-GNSS Calibration (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t  g_dwt_at_last_pps;
extern volatile uint32_t  g_dwt_cycles_per_gnss_s;
extern volatile bool      g_dwt_cal_has_prev;
extern volatile bool      g_dwt_cal_valid;
extern volatile uint64_t  g_dwt_cal_pps_count;

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

// GNSS: QTimer1 CM(2) counts both edges — 20 MHz raw expected
static constexpr uint32_t GNSS_EDGE_DIVISOR     = 2;
static constexpr uint32_t ISR_GNSS_RAW_EXPECTED = TICKS_10MHZ_PER_SECOND * GNSS_EDGE_DIVISOR;

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
// PPS state
// ============================================================================

extern volatile bool pps_fired;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;
  uint32_t dither_cycle;        // retained for compatibility, no longer used
  double   dither_accum;        // Bresenham accumulator
  uint32_t dither_high_count;
  uint32_t dither_low_count;
  int32_t  servo_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;
};

extern ocxo_dac_state_t ocxo1_dac;
extern ocxo_dac_state_t ocxo2_dac;

static constexpr uint32_t DITHER_PERIOD = 1000;

extern bool calibrate_ocxo_active;

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
  uint32_t prev_prev_delta;
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
// Spin Capture — nano-precise DWT anchoring with shadow-write TDC
//
// Two distinct timeout failure modes:
//
//   1. Nano-spin timeout: TimePop's DWT spin exceeded NANO_SPIN_MAX_CYCLES.
//      Fields: nano_timed_out (this cycle), nano_timeouts (lifetime count).
//
//   2. Shadow-loop timeout: shadow-write loop exceeded SPIN_LOOP_TIMEOUT_CYCLES.
//      Fields: shadow_timed_out (this cycle), shadow_timeouts (lifetime).
//
// The same struct will be reused for 10 KHz VCLOCK captures.
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

inline uint64_t gnss_ticks_64_get(void) {
  return gnss_raw_64 / GNSS_EDGE_DIVISOR;
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
