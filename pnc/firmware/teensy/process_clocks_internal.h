#pragma once

// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// v29: OCXO phase capture — shadow-write TDC architecture.
//
//   ocxo_phase_capture_t gains shadow_dwt and delta_cycles fields for
//   each OCXO, matching the PPS spin capture TDC pattern.  The GPT ISRs
//   now capture the shared shadow DWT alongside their own DWT read.
//   Beta uses the shadow-corrected DWT for edge timestamp computation.
//
//   New externs: ocxo_phase_shadow_dwt (shared shadow variable),
//   ocxoN_phase_shadow_dwt (per-ISR shadow captures),
//   diag_ocxo_phase_spin_timeouts.
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
#include "time.h"

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

static inline uint64_t dwt_ns_to_cycles(uint64_t ns)
{
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

// Ticks ahead of PPS snapshot to arm GPT output compare.
// Armed in pps_isr; must be large enough that the compare fires
// AFTER the ASAP callback has started its shadow-write loop.
// At 10 MHz, 150 ticks = 15 µs.  The ASAP callback starts ~2-3 µs
// after PPS and reaches the shadow loop within ~1 µs, so 15 µs
// provides comfortable margin.
static constexpr uint32_t OCXO_PHASE_ARM_OFFSET_TICKS = 150;

// ============================================================================
// PPS diagnostics (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint32_t diag_gpt1_isr_fires;
extern volatile uint32_t diag_gpt2_isr_fires;

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

// ── Phase capture diagnostics (alpha-owned, beta-readable) ──
extern volatile uint32_t diag_ocxo1_phase_captures;
extern volatile uint32_t diag_ocxo2_phase_captures;
extern volatile uint32_t diag_ocxo1_phase_misses;
extern volatile uint32_t diag_ocxo2_phase_misses;
extern volatile uint32_t diag_ocxo_phase_spin_timeouts;

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
  uint32_t        target_dwt;
  int64_t         target_gnss_ns;
  bool            armed;
  timepop_handle_t handle;

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
// OCXO phase capture — v29 shadow-write TDC architecture
//
// Each GPT fires a one-shot output-compare ISR on the first OCXO
// edge after PPS.  The ISR captures DWT_CYCCNT and the current shadow
// DWT from the ASAP callback's shadow-write loop.  The delta
// (isr_dwt - shadow_dwt) provides per-sample ISR latency for TDC
// correction, identical to the PPS spin capture mechanism.
//
// Beta converts the shadow-corrected DWT capture into canonical
// PPS-anchored GNSS nanoseconds, derives phase offset in [0,100), and
// computes the authoritative per-second residual from consecutive edge
// timestamps.
//
// Diagnostic fields (isr_dwt, shadow_dwt, delta_cycles, gpt_at_fire)
// enable ISR latency histogram analysis via gpt_phase_tdc analyzer.
// ============================================================================

struct ocxo_phase_capture_t {
  // ── PPS anchor ──
  uint32_t dwt_at_pps;               // best-available DWT at PPS edge

  // ── OCXO1 shadow-write TDC capture ──
  uint32_t ocxo1_isr_dwt;            // raw DWT_CYCCNT from GPT1 ISR
  uint32_t ocxo1_shadow_dwt;         // shadow DWT captured by GPT1 ISR
  int32_t  ocxo1_delta_cycles;       // isr_dwt - shadow_dwt (TDC measurement)
  uint32_t ocxo1_gpt_at_fire;        // GPT1_CNT at ISR entry (diagnostic)
  uint32_t ocxo1_dwt_elapsed = 0;
  uint32_t ocxo1_elapsed_ns = 0;
  uint32_t ocxo1_phase_offset_ns;    // elapsed_ns % 100
  bool     ocxo1_captured;           // GPT1 ISR fired successfully
  bool     ocxo1_valid;              // phase computation completed

  // ── OCXO2 shadow-write TDC capture ──
  uint32_t ocxo2_isr_dwt;            // raw DWT_CYCCNT from GPT2 ISR
  uint32_t ocxo2_shadow_dwt;         // shadow DWT captured by GPT2 ISR
  int32_t  ocxo2_delta_cycles;       // isr_dwt - shadow_dwt (TDC measurement)
  uint32_t ocxo2_gpt_at_fire;        // GPT2_CNT at ISR entry (diagnostic)
  uint32_t ocxo2_dwt_elapsed = 0;
  uint32_t ocxo2_elapsed_ns = 0;
  uint32_t ocxo2_phase_offset_ns;    // elapsed_ns % 100
  bool     ocxo2_captured;           // GPT2 ISR fired successfully
  bool     ocxo2_valid;              // phase computation completed

  // ── Combined validity ──
  bool     detector_valid;            // both OCXOs valid

  // ── Canonical absolute edge timestamps (beta-owned) ──
  uint64_t pps_gnss_ns;
  uint64_t ocxo1_edge_gnss_ns;
  uint64_t ocxo2_edge_gnss_ns;

  // ── Per-second differencing (beta-owned) ──
  uint64_t prev_ocxo1_edge_gnss_ns;
  uint64_t prev_ocxo2_edge_gnss_ns;
  int64_t  ocxo1_gnss_ns_per_pps;
  int64_t  ocxo2_gnss_ns_per_pps;
  int64_t  ocxo1_residual_ns;
  int64_t  ocxo2_residual_ns;
  bool     residual_valid;

  // ── Servo before/after snapshot (beta-owned) ──
  uint16_t ocxo1_dac_before;
  uint16_t ocxo1_dac_after;
  uint16_t ocxo2_dac_before;
  uint16_t ocxo2_dac_after;
};

extern ocxo_phase_capture_t ocxo_phase;

// ── GPT ISR capture state (alpha-owned, read by pps_asap_callback) ──
extern volatile uint32_t ocxo_phase_shadow_dwt;    // shared shadow variable
extern volatile uint32_t ocxo1_phase_isr_dwt;
extern volatile uint32_t ocxo1_phase_shadow_dwt;   // shadow captured by GPT1 ISR
extern volatile bool     ocxo1_phase_captured;
extern volatile uint32_t ocxo1_phase_gpt_at_fire;
extern volatile uint32_t ocxo2_phase_isr_dwt;
extern volatile uint32_t ocxo2_phase_shadow_dwt;   // shadow captured by GPT2 ISR
extern volatile bool     ocxo2_phase_captured;
extern volatile uint32_t ocxo2_phase_gpt_at_fire;

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

static inline bool ocxo_edge_dwt_to_gnss_ns(
  uint32_t dwt_event,
  uint64_t pps_gnss_ns,
  uint32_t dwt_at_pps,
  uint32_t dwt_cycles_per_pps,
  uint64_t& out_gnss_ns
) {
  // pps_gnss_ns is (pps_count - 1) * 1e9, so pps_count = pps_gnss_ns / 1e9 + 1
  const uint32_t pps_count = (uint32_t)(pps_gnss_ns / 1000000000ULL) + 1;

  const int64_t edge_signed = time_dwt_to_gnss_ns(
    dwt_event, dwt_at_pps, dwt_cycles_per_pps, pps_count
  );

  if (edge_signed < 0) {
    out_gnss_ns = 0;
    return false;
  }

  out_gnss_ns = (uint64_t)edge_signed;
  return true;
}

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