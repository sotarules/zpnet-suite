#pragma once

// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// v14: Symmetric GPT architecture for both OCXOs.
//
//   OCXO1 on GPT1  (pin 25, 10 MHz single-edge, 32-bit native).
//   OCXO2 on GPT2  (pin 14, 10 MHz single-edge, 32-bit native).
//   GNSS  on QTimer1 ch0+ch1 (pin 10, 20 MHz dual-edge, cascaded 32-bit).
//
//   Both OCXOs now have clean single-edge counting — no dual-edge
//   phase aliasing artifact.  GNSS VCLOCK moves to QTimer1 where
//   the dual-edge alternation is harmless: GNSS authority comes
//   from PPS count, not VCLOCK precision.  VCLOCK serves only as
//   a PPS sanity check.
//
//   GPT2 is no longer available for TimePop internal scheduling.
//   TimePop uses DWT-based deadline conversion (already implemented).
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
// 64-bit accumulators (campaign-scoped)
//
// Both OCXOs: GPT single-edge, accumulate 10 MHz ticks directly.
// GNSS: QTimer1 dual-edge, accumulates raw 20 MHz then divides.
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
// Beta entry point — called from alpha's pps_asap_callback
// ============================================================================

void clocks_beta_pps(void);

// ============================================================================
// Zeroing (called by beta campaign commands, reads alpha ISR state)
// ============================================================================

void clocks_zero_all(void);