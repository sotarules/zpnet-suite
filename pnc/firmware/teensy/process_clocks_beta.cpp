// ============================================================================
// process_clocks_beta.cpp — Campaign Layer
// ============================================================================
//
// v15: Random walk prediction model.
//
//   Empirical testing over 38,000+ seconds (Baseline4 campaign) showed
//   that a random walk predictor (predicted = prev_delta) produces 1.67x
//   lower prediction stddev than linear extrapolation (predicted =
//   2*prev - prev_prev) for DWT.  For OCXOs the difference is negligible
//   since their deltas are near-constant when servo-locked.
//
//   The random walk model assumes the crystal's current rate is the best
//   estimate of its next rate.  Linear extrapolation assumes trends
//   continue, but thermal drift is a random walk — when the trend
//   reverses, the extrapolator overshoots in both directions.
//
//   Changes from v14:
//     • prediction_tracker_t: prev_prev_delta removed
//     • prediction_update(): predicted = prev_delta (random walk)
//     • prediction_seed(): simplified, history_count = 2
//     • Welford gate: history_count >= 2 (scores one second earlier)
//     • prediction_reset(): prev_prev_delta removed
//
// v14: Symmetric GPT counting for both OCXOs.
//
//   OCXO1 on GPT1 (10 MHz single-edge, 32-bit native).
//   OCXO2 on GPT2 (10 MHz single-edge, 32-bit native).
//   GNSS  on QTimer1 (10 MHz single-edge, cascaded 32-bit).
//
//   All three 10 MHz domains accumulate ticks directly.
//   No divisor or domain translation required.
//
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"

#include "debug.h"
#include "timebase.h"
#include "time.h"

#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"
#include "util.h"
#include "ad5693r.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <stdlib.h>
#include <climits>

// ============================================================================
// Campaign State — definitions
// ============================================================================

volatile clocks_campaign_state_t campaign_state =
  clocks_campaign_state_t::STOPPED;

char     campaign_name[64] = {0};
uint64_t campaign_seconds  = 0;

volatile bool request_start   = false;
volatile bool request_stop    = false;
volatile bool request_recover = false;

uint64_t recover_dwt_ns   = 0;
uint64_t recover_gnss_ns  = 0;
uint64_t recover_ocxo1_ns = 0;
uint64_t recover_ocxo2_ns = 0;

volatile bool     watchdog_anomaly_active          = false;
volatile bool     watchdog_anomaly_publish_pending = false;
volatile uint32_t watchdog_anomaly_sequence        = 0;
char              watchdog_anomaly_reason[64]      = {0};
volatile uint32_t watchdog_anomaly_detail0         = 0;
volatile uint32_t watchdog_anomaly_detail1         = 0;
volatile uint32_t watchdog_anomaly_detail2         = 0;
volatile uint32_t watchdog_anomaly_detail3         = 0;
volatile uint32_t watchdog_anomaly_trigger_dwt     = 0;

// ============================================================================
// PPS residual tracking — definitions
// ============================================================================

pps_residual_t residual_dwt   = {};
pps_residual_t residual_gnss  = {};
pps_residual_t residual_ocxo1 = {};
pps_residual_t residual_ocxo2 = {};

void residual_reset(pps_residual_t& r) {
  r.ticks_at_last_pps = 0;
  r.delta             = 0;
  r.residual          = 0;
  r.valid             = false;
  r.n                 = 0;
  r.mean              = 0.0;
  r.m2                = 0.0;
}

void residual_update(pps_residual_t& r, uint64_t ticks_now, int64_t expected) {
  if (r.ticks_at_last_pps > 0) {
    r.delta    = (int64_t)(ticks_now - r.ticks_at_last_pps);
    r.residual = r.delta - expected;
    r.valid    = true;
    r.n++;
    const double x      = (double)r.residual;
    const double delta  = x - r.mean;
    r.mean += delta / (double)r.n;
    const double delta2 = x - r.mean;
    r.m2 += delta * delta2;
  } else {
    r.delta    = 0;
    r.residual = 0;
    r.valid    = false;
  }
  r.ticks_at_last_pps = ticks_now;
}

double residual_stddev(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) : 0.0;
}

double residual_stderr(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) / sqrt((double)r.n) : 0.0;
}

// ============================================================================
// Random walk prediction tracking — definitions
//
// v15: Random walk model.  predicted = prev_delta.
// Empirically 1.67x lower stddev than linear extrapolation for DWT.
// ============================================================================

prediction_tracker_t pred_dwt   = {};
prediction_tracker_t pred_ocxo1 = {};
prediction_tracker_t pred_ocxo2 = {};

void prediction_reset(prediction_tracker_t& p) {
  p.prev_delta      = 0;
  p.predicted       = 0;
  p.pred_residual   = 0;
  p.history_count   = 0;
  p.predicted_valid = false;
  p.n               = 0;
  p.mean            = 0.0;
  p.m2              = 0.0;
}

void prediction_seed(prediction_tracker_t& p, uint32_t value) {
  p.prev_delta      = value;
  p.predicted       = value;
  p.predicted_valid = true;
  p.history_count   = 2;
}

void prediction_update(prediction_tracker_t& p, uint32_t actual_delta) {
  // Score the prediction if we have a valid one
  if (p.predicted_valid && p.history_count >= 2) {
    p.pred_residual = (int32_t)actual_delta - (int32_t)p.predicted;
    p.n++;
    const double x  = (double)p.pred_residual;
    const double d1 = x - p.mean;
    p.mean += d1 / (double)p.n;
    const double d2 = x - p.mean;
    p.m2 += d1 * d2;
  }

  // Random walk: next prediction is this delta
  p.prev_delta = actual_delta;
  if (p.history_count < 255) p.history_count++;

  if (p.history_count >= 1) {
    p.predicted       = p.prev_delta;
    p.predicted_valid = true;
  } else {
    p.predicted_valid = false;
  }
}

double prediction_stddev(const prediction_tracker_t& p) {
  return (p.n >= 2) ? sqrt(p.m2 / (double)(p.n - 1)) : 0.0;
}

double prediction_stderr(const prediction_tracker_t& p) {
  return (p.n >= 2) ? sqrt(p.m2 / (double)(p.n - 1)) / sqrt((double)p.n) : 0.0;
}

// ============================================================================
// DAC Welford tracking — definitions
//
// Cumulative Welford's over dac_fractional, one sample per PPS second.
// Campaign-scoped: reset on START and RECOVER.
// ============================================================================

dac_welford_t dac_welford_ocxo1 = {};
dac_welford_t dac_welford_ocxo2 = {};

void dac_welford_reset(dac_welford_t& w) {
  w.n       = 0;
  w.mean    = 0.0;
  w.m2      = 0.0;
  w.min_val = 1e30;
  w.max_val = -1e30;
}

void dac_welford_update(dac_welford_t& w, double value) {
  w.n++;
  const double d1 = value - w.mean;
  w.mean += d1 / (double)w.n;
  const double d2 = value - w.mean;
  w.m2 += d1 * d2;
  if (value < w.min_val) w.min_val = value;
  if (value > w.max_val) w.max_val = value;
}

double dac_welford_stddev(const dac_welford_t& w) {
  return (w.n >= 2) ? sqrt(w.m2 / (double)(w.n - 1)) : 0.0;
}

double dac_welford_stderr(const dac_welford_t& w) {
  return (w.n >= 2) ? sqrt(w.m2 / (double)(w.n - 1)) / sqrt((double)w.n) : 0.0;
}

// ============================================================================
// 64-bit accumulators — definitions
//
// All three external clocks accumulate 10 MHz ticks directly.
// GNSS: QTimer1 single-edge (v23), same as OCXOs on GPT.
// ============================================================================

uint64_t dwt_cycles_64     = 0;
uint32_t prev_dwt_at_pps   = 0;

uint64_t ocxo1_ticks_64    = 0;
uint32_t prev_ocxo1_at_pps = 0;

uint64_t ocxo2_ticks_64    = 0;
uint32_t prev_ocxo2_at_pps = 0;

uint64_t gnss_raw_64       = 0;
uint32_t prev_gnss_at_pps  = 0;

// ============================================================================
// Zeroing (campaign-scoped — fresh start only)
// ============================================================================

void clocks_zero_all(void) {
  timebase_invalidate();

  dwt_cycles_64     = 0;
  prev_dwt_at_pps   = (pps_spin.valid && pps_spin.tdc_correction >= 0)
                        ? pps_spin.corrected_dwt
                        : (isr_snap_dwt - ISR_ENTRY_DWT_CYCLES);

  ocxo1_ticks_64    = 0;
  prev_ocxo1_at_pps = isr_snap_ocxo1;

  ocxo2_ticks_64    = 0;
  prev_ocxo2_at_pps = isr_snap_ocxo2;

  gnss_raw_64       = 0;
  prev_gnss_at_pps  = isr_snap_gnss;

  campaign_seconds  = 0;

  dwt_rolling_64       = 0;  dwt_rolling_32       = DWT_CYCCNT;
  gnss_rolling_raw_64  = 0;  gnss_rolling_32      = qtimer1_read_32();
  ocxo1_rolling_64     = 0;  ocxo1_rolling_32     = GPT1_CNT;
  ocxo2_rolling_64     = 0;  ocxo2_rolling_32     = GPT2_CNT;

  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo1);
  residual_reset(residual_ocxo2);

  ocxo_phase = {};

  prediction_reset(pred_dwt);
  prediction_reset(pred_ocxo1);
  prediction_reset(pred_ocxo2);

  // Reset PPS rejection state — pre-campaign noise must not carry over
  diag_pps_reject_consecutive = 0;
  isr_residual_valid = false;

  prediction_seed(pred_dwt,
    g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : DWT_EXPECTED_PER_PPS);

  dac_welford_reset(dac_welford_ocxo1);
  dac_welford_reset(dac_welford_ocxo2);

  isr_prev_dwt       = isr_snap_dwt;
  isr_prev_gnss      = isr_snap_gnss;
  isr_prev_ocxo1     = isr_snap_ocxo1;
  isr_prev_ocxo2     = isr_snap_ocxo2;
  isr_residual_valid = false;

  pps_fired = false;
}

// ============================================================================
// OCXO calibration servo — unified for both OCXOs
// ============================================================================

// MEAN mode: target Welford mean residual → 0
static void ocxo_servo_mean(ocxo_dac_state_t& dac, pps_residual_t& res) {
  if (res.n < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  double mean_residual = res.mean;
  dac.servo_last_residual = mean_residual;

  if (fabs(mean_residual) < 0.01) return;

  double step = -mean_residual * 0.1;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
  dac.servo_settle_count = 0;

  residual_reset(res);
}

// TOTAL mode: target cumulative tick deficit → 0 (tau → 1.0)
//
// Error signal is (ocxo_ticks - gnss_ticks) in 10 MHz ticks.
// Each tick = 100 ns.  Positive = OCXO ran fast, steer down.
static void ocxo_servo_total(ocxo_dac_state_t& dac, uint64_t ocxo_ticks, uint64_t gnss_ticks) {
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  double total_error = (double)((int64_t)ocxo_ticks - (int64_t)gnss_ticks);
  dac.servo_last_residual = total_error;

  double rate_error = total_error / (double)campaign_seconds;

  if (fabs(rate_error) < 1e-6) return;

  double step = -rate_error * 50.0;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
  dac.servo_settle_count = 0;
}

// NOW mode: phase-first per-second residual → 0
//
// Uses the authoritative per-second residual derived from consecutive absolute canonical edge timestamps.
//
// Beta computes the absolute GNSS nanosecond timestamp of the canonical
// first post-PPS edge each second:
//
//   edge_gnss_ns = pps_gnss_ns + phase_offset_ns
//
// and then derives:
//
//   residual_ns = (edge_gnss_ns_this - edge_gnss_ns_prev) - 1e9
//
// Positive residual means the OCXO edge moved later (slow); negative
// residual means the OCXO edge moved earlier (fast).
//
// The gain constant NOW_NS_PER_DAC_LSB is the empirical relationship
// between one DAC code step and the resulting change in residual_ns.
// Positive DAC voltage slows these OCXOs; negative DAC voltage speeds them.
//
// The servo applies a proportional correction every second with no
// settle delay — the measurement is precise enough to act immediately.
static constexpr double NOW_NS_PER_DAC_LSB = 10.0;
static constexpr double NOW_MIN_RESIDUAL_NS = 5.0;

static void ocxo_servo_now(ocxo_dac_state_t& dac, int64_t residual_ns) {
  if (!ocxo_phase.residual_valid) return;
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;

  double residual = (double)residual_ns;
  dac.servo_last_residual = residual;

  if (fabs(residual) < NOW_MIN_RESIDUAL_NS) return;

  // Negative residual = OCXO fast = need to steer DAC down
  // Positive residual = OCXO slow = need to steer DAC up
  double step = -residual / NOW_NS_PER_DAC_LSB;

  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
}

static void ocxo_calibration_servo(void) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  if (calibrate_ocxo_mode == servo_mode_t::MEAN) {
    ocxo_servo_mean(ocxo1_dac, residual_ocxo1);
    ocxo_servo_mean(ocxo2_dac, residual_ocxo2);
  } else if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    ocxo_servo_now(ocxo1_dac, ocxo_phase.ocxo1_residual_ns);
    ocxo_servo_now(ocxo2_dac, ocxo_phase.ocxo2_residual_ns);
  } else {
    ocxo_servo_total(ocxo1_dac, ocxo1_ticks_64, gnss_raw_64);
    ocxo_servo_total(ocxo2_dac, ocxo2_ticks_64, gnss_raw_64);
  }
}

// ============================================================================
// Watchdog anomaly publication / stop helpers
// ============================================================================

static void clocks_force_stop_campaign(void) {
  campaign_state = clocks_campaign_state_t::STOPPED;
  request_start = false;
  request_stop = false;
  request_recover = false;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  pps_scheduled = false;
  pps_fired = false;
  timebase_invalidate();
}

static void clocks_watchdog_anomaly_callback(timepop_ctx_t*, void*) {
  if (!watchdog_anomaly_publish_pending) {
    clocks_force_stop_campaign();
    return;
  }

  Payload p;
  p.add("sequence",         watchdog_anomaly_sequence);
  p.add("reason",           watchdog_anomaly_reason);
  p.add("campaign",         campaign_name);
  p.add("campaign_seconds", campaign_seconds);
  p.add("trigger_dwt",      watchdog_anomaly_trigger_dwt);
  p.add("detail0",          watchdog_anomaly_detail0);
  p.add("detail1",          watchdog_anomaly_detail1);

  p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));
  p.add("gnss_ns_now",      clocks_gnss_ns_now());
  p.add("dwt_ns_now",       clocks_dwt_ns_now());

  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_ocxo1", isr_residual_ocxo1);
  p.add("isr_residual_ocxo2", isr_residual_ocxo2);

  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);

  publish("WATCHDOG_ANOMALY", p);

  watchdog_anomaly_publish_pending = false;
  clocks_force_stop_campaign();
}

void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3) {
  if (!watchdog_anomaly_active) {
    watchdog_anomaly_sequence++;
    safeCopy(watchdog_anomaly_reason, sizeof(watchdog_anomaly_reason),
             (reason && *reason) ? reason : "watchdog_anomaly");
    watchdog_anomaly_detail0 = detail0;
    watchdog_anomaly_detail1 = detail1;
    watchdog_anomaly_detail2 = detail2;
    watchdog_anomaly_detail3 = detail3;
    watchdog_anomaly_trigger_dwt = DWT_CYCCNT;
    watchdog_anomaly_publish_pending = true;
  }

  watchdog_anomaly_active = true;
  pps_scheduled = false;

  timepop_handle_t h = timepop_arm(0, false, clocks_watchdog_anomaly_callback, nullptr, "clocks-anomaly");
  if (h == TIMEPOP_INVALID_HANDLE) {
    diag_pps_asap_arm_failures++;
    watchdog_anomaly_publish_pending = false;
    clocks_force_stop_campaign();
  }
}

// ============================================================================
// clocks_beta_pps — called from alpha's pps_asap_callback
// ============================================================================

void clocks_beta_pps(void) {

  // ── Command requests override anomaly latch ──
  // START/STOP/RECOVER are authoritative Pi-side directives.
  // They must be processed even if watchdog_anomaly_active was
  // re-latched by the ISR between the command handler clearing
  // it and this callback firing.
  if (request_stop) {
    watchdog_anomaly_active = false;
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop   = false;
    pps_fired      = true;
    calibrate_ocxo_mode = servo_mode_t::OFF;
    timebase_invalidate();
    return;
  }

  if (request_recover) {
    watchdog_anomaly_active = false;

    timebase_invalidate();

    dwt_cycles_64     = dwt_ns_to_cycles(recover_dwt_ns);
    prev_dwt_at_pps   = (pps_spin.valid && pps_spin.tdc_correction >= 0)
                          ? pps_spin.corrected_dwt
                          : (isr_snap_dwt - ISR_ENTRY_DWT_CYCLES);

    ocxo1_ticks_64    = recover_ocxo1_ns / 100ull;
    prev_ocxo1_at_pps = isr_snap_ocxo1;

    ocxo2_ticks_64    = recover_ocxo2_ns / 100ull;
    prev_ocxo2_at_pps = isr_snap_ocxo2;

    gnss_raw_64       = recover_gnss_ns / 100ull;
    prev_gnss_at_pps  = isr_snap_gnss;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    dwt_rolling_64       = 0;  dwt_rolling_32       = DWT_CYCCNT;
    gnss_rolling_raw_64  = 0;  gnss_rolling_32      = qtimer1_read_32();
    ocxo1_rolling_64     = 0;  ocxo1_rolling_32     = GPT1_CNT;
    ocxo2_rolling_64     = 0;  ocxo2_rolling_32     = GPT2_CNT;

    residual_reset(residual_dwt);
    residual_reset(residual_gnss);
    residual_reset(residual_ocxo1);
    residual_reset(residual_ocxo2);

    ocxo_phase = {};

    prediction_reset(pred_dwt);
    prediction_reset(pred_ocxo1);
    prediction_reset(pred_ocxo2);

    prediction_seed(pred_dwt,
      g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : DWT_EXPECTED_PER_PPS);

    dac_welford_reset(dac_welford_ocxo1);
    dac_welford_reset(dac_welford_ocxo2);

    isr_residual_valid = false;
    pps_fired          = true;
    diag_pps_reject_consecutive = 0;

    campaign_state  = clocks_campaign_state_t::STARTED;
    request_recover = false;

    pps_scheduled = false;
    return;
  }

  if (request_start) {
    watchdog_anomaly_active = false;
    clocks_zero_all();
    campaign_state = clocks_campaign_state_t::STARTED;
    request_start  = false;
    pps_scheduled = false;
    return;
  }

  if (watchdog_anomaly_active) {
    pps_scheduled = false;
    return;
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {

    if (!isr_residual_valid) {
      isr_residual_valid = true;
    }

    pps_fired = false;

    // ── DWT ──
    // TDC-corrected when spin capture succeeded; ISR-compensated fallback otherwise.
    uint32_t dwt_raw_at_pps =
      (pps_spin.valid && pps_spin.tdc_correction >= 0)
        ? pps_spin.corrected_dwt
        : (isr_snap_dwt - ISR_ENTRY_DWT_CYCLES);
    uint32_t dwt_delta = dwt_raw_at_pps - prev_dwt_at_pps;

    dwt_cycles_64   += dwt_delta;
    prev_dwt_at_pps  = dwt_raw_at_pps;

    const uint64_t pps_dwt_cycles = dwt_cycles_64;
    const uint64_t pps_dwt_ns     = dwt_cycles_to_ns(pps_dwt_cycles);

    // ── OCXO1: GPT1, single-edge, 32-bit delta ──
    uint32_t ocxo1_delta = isr_snap_ocxo1 - prev_ocxo1_at_pps;
    ocxo1_ticks_64    += ocxo1_delta;
    prev_ocxo1_at_pps  = isr_snap_ocxo1;

    const uint64_t pps_ocxo1_ticks = ocxo1_ticks_64;
    const uint64_t pps_ocxo1_ns    = pps_ocxo1_ticks * 100ull;

    // ── OCXO2: GPT2, single-edge, 32-bit delta ──
    uint32_t ocxo2_delta = isr_snap_ocxo2 - prev_ocxo2_at_pps;
    ocxo2_ticks_64    += ocxo2_delta;
    prev_ocxo2_at_pps  = isr_snap_ocxo2;

    const uint64_t pps_ocxo2_ticks = ocxo2_ticks_64;
    const uint64_t pps_ocxo2_ns    = pps_ocxo2_ticks * 100ull;

    // GNSS: QTimer1, single-edge 10 MHz, 32-bit delt
    uint32_t gnss_raw_delta = isr_snap_gnss - prev_gnss_at_pps;
    gnss_raw_64      += gnss_raw_delta;
    prev_gnss_at_pps  = isr_snap_gnss;

    campaign_seconds++;

    const uint64_t pps_gnss_ns    = campaign_seconds * NS_PER_SECOND;
    const uint64_t pps_gnss_ticks = campaign_seconds * (uint64_t)TICKS_10MHZ_PER_SECOND;

    const uint32_t dwt_cycles_per_pps_snapshot =
      pred_dwt.predicted_valid ? pred_dwt.predicted :
      (g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : DWT_EXPECTED_PER_PPS);

    // ── Residual updates ──
    residual_update(residual_dwt,   pps_dwt_cycles,   DWT_EXPECTED_PER_PPS_I);
    residual_update(residual_gnss,  pps_gnss_ticks,   GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo1, pps_ocxo1_ticks,  OCXO_EXPECTED_PER_PPS);
    residual_update(residual_ocxo2, pps_ocxo2_ticks,  OCXO_EXPECTED_PER_PPS);

    // ── Prediction tracking ──
    prediction_update(pred_dwt,   dwt_delta);
    prediction_update(pred_ocxo1, ocxo1_delta);
    prediction_update(pred_ocxo2, ocxo2_delta);

    // Snapshot DAC state before servo (for telemetry)
    const uint16_t dac1_before = ocxo1_dac.dac_hw_code;
    const uint16_t dac2_before = ocxo2_dac.dac_hw_code;

    // ── Absolute GNSS timestamps for canonical first post-PPS OCXO edges ──
    ocxo_phase.pps_gnss_ns = pps_gnss_ns;

    ocxo_phase.ocxo1_edge_gnss_ns = pps_gnss_ns + (uint64_t)ocxo_phase.ocxo1_phase_offset_ns;
    ocxo_phase.ocxo2_edge_gnss_ns = pps_gnss_ns + (uint64_t)ocxo_phase.ocxo2_phase_offset_ns;
    ocxo_phase.ocxo1_phase2_edge_gnss_ns = pps_gnss_ns + (uint64_t)ocxo_phase.ocxo1_phase2_phase_offset_ns;
    ocxo_phase.ocxo2_phase2_edge_gnss_ns = pps_gnss_ns + (uint64_t)ocxo_phase.ocxo2_phase2_phase_offset_ns;

    if (ocxo_phase.captures >= 2 && ocxo_phase.prev_ocxo1_edge_gnss_ns != 0 && ocxo_phase.prev_ocxo2_edge_gnss_ns != 0) {
      const int64_t ocxo1_gnss_ns_per_pps =
        (int64_t)(ocxo_phase.ocxo1_edge_gnss_ns - ocxo_phase.prev_ocxo1_edge_gnss_ns);
      const int64_t ocxo2_gnss_ns_per_pps =
        (int64_t)(ocxo_phase.ocxo2_edge_gnss_ns - ocxo_phase.prev_ocxo2_edge_gnss_ns);

      ocxo_phase.ocxo1_gnss_ns_per_pps = ocxo1_gnss_ns_per_pps;
      ocxo_phase.ocxo2_gnss_ns_per_pps = ocxo2_gnss_ns_per_pps;
      ocxo_phase.ocxo1_residual_ns = ocxo1_gnss_ns_per_pps - (int64_t)NS_PER_SECOND;
      ocxo_phase.ocxo2_residual_ns = ocxo2_gnss_ns_per_pps - (int64_t)NS_PER_SECOND;
      ocxo_phase.residual_valid = true;
    } else {
      ocxo_phase.ocxo1_gnss_ns_per_pps = 0;
      ocxo_phase.ocxo2_gnss_ns_per_pps = 0;
      ocxo_phase.ocxo1_residual_ns = 0;
      ocxo_phase.ocxo2_residual_ns = 0;
      ocxo_phase.residual_valid = false;
    }

    ocxo_phase.prev_ocxo1_edge_gnss_ns = ocxo_phase.ocxo1_edge_gnss_ns;
    ocxo_phase.prev_ocxo2_edge_gnss_ns = ocxo_phase.ocxo2_edge_gnss_ns;

    ocxo_calibration_servo();

    // Snapshot DAC state after servo (for telemetry)
    ocxo_phase.ocxo1_dac_before = dac1_before;
    ocxo_phase.ocxo1_dac_after  = ocxo1_dac.dac_hw_code;
    ocxo_phase.ocxo2_dac_before = dac2_before;
    ocxo_phase.ocxo2_dac_after  = ocxo2_dac.dac_hw_code;

    // ── DAC Welford update (TEMPEST DAC test) ──
    dac_welford_update(dac_welford_ocxo1, ocxo1_dac.dac_fractional);
    dac_welford_update(dac_welford_ocxo2, ocxo2_dac.dac_fractional);

    // ── Build and publish TIMEBASE_FRAGMENT ──
    Payload p;
    p.add("campaign",         campaign_name);

    p.add("dwt_cycles",       pps_dwt_cycles);
    p.add("dwt_ns",           pps_dwt_ns);
    p.add("gnss_ns",          pps_gnss_ns);
    p.add("ocxo1_ns",         pps_ocxo1_ns);
    p.add("ocxo2_ns",         pps_ocxo2_ns);
    p.add("teensy_pps_count", campaign_seconds);
    p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));
    p.add("dwt_cyccnt_at_pps", (uint32_t)dwt_raw_at_pps);
    p.add("qtimer_at_pps",    (uint32_t)isr_snap_gnss);

    p.add("dwt_cycles_per_pps", (uint64_t)dwt_cycles_per_pps_snapshot);

    p.add("dwt_delta_raw",    (uint64_t)dwt_delta);
    p.add("ocxo1_delta_raw",  (uint64_t)ocxo1_delta);
    p.add("ocxo2_delta_raw",  (uint64_t)ocxo2_delta);
    p.add("gnss_raw_delta",   (uint64_t)gnss_raw_delta);

    p.add("dwt_pps_residual",   residual_dwt.residual);
    p.add("gnss_pps_residual",  residual_gnss.residual);
    p.add("ocxo1_pps_residual", residual_ocxo1.residual);
    p.add("ocxo2_pps_residual", residual_ocxo2.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo1", isr_residual_ocxo1);
    p.add("isr_residual_ocxo2", isr_residual_ocxo2);
    p.add("isr_residual_valid", isr_residual_valid);

    p.add("dwt_pred_residual",  pred_dwt.pred_residual);
    p.add("dwt_pred_mean",      pred_dwt.mean);
    p.add("dwt_pred_stddev",    prediction_stddev(pred_dwt));
    p.add("dwt_pred_n",         pred_dwt.n);

    p.add("ocxo1_pred_residual", pred_ocxo1.pred_residual);
    p.add("ocxo1_pred_mean",     pred_ocxo1.mean);
    p.add("ocxo1_pred_stddev",   prediction_stddev(pred_ocxo1));
    p.add("ocxo1_pred_n",        pred_ocxo1.n);

    p.add("ocxo2_pred_residual", pred_ocxo2.pred_residual);
    p.add("ocxo2_pred_mean",     pred_ocxo2.mean);
    p.add("ocxo2_pred_stddev",   prediction_stddev(pred_ocxo2));
    p.add("ocxo2_pred_n",        pred_ocxo2.n);

    p.add("ocxo1_dac",         ocxo1_dac.dac_fractional);
    p.add("ocxo2_dac",         ocxo2_dac.dac_fractional);
    p.add("ocxo1_dac_hw",      (int32_t)ocxo1_dac.dac_hw_code);
    p.add("ocxo2_dac_hw",      (int32_t)ocxo2_dac.dac_hw_code);

    // ── Phase-first OCXO measurement (v26) ──
    p.add("ocxo1_raw_phase_offset_ns",       (int32_t)ocxo_phase.ocxo1_raw_phase_offset_ns);
    p.add("ocxo1_phase_bias_ns",             ocxo_phase.ocxo1_phase_bias_ns);
    p.add("ocxo1_adjusted_phase_signed_ns",     ocxo_phase.ocxo1_adjusted_phase_signed_ns);
    p.add("ocxo1_phase_offset_ns",           (int32_t)ocxo_phase.ocxo1_phase_offset_ns);
    p.add("ocxo2_raw_phase_offset_ns",       (int32_t)ocxo_phase.ocxo2_raw_phase_offset_ns);
    p.add("ocxo2_phase_bias_ns",             ocxo_phase.ocxo2_phase_bias_ns);
    p.add("ocxo2_adjusted_phase_signed_ns",     ocxo_phase.ocxo2_adjusted_phase_signed_ns);
    p.add("ocxo2_phase_offset_ns",           (int32_t)ocxo_phase.ocxo2_phase_offset_ns);
    p.add("ocxo1_raw_elapsed_ns",            (int32_t)ocxo_phase.ocxo1_raw_elapsed_ns);
    p.add("ocxo2_raw_elapsed_ns",            (int32_t)ocxo_phase.ocxo2_raw_elapsed_ns);
    p.add("ocxo1_edge_gnss_ns",      ocxo_phase.ocxo1_edge_gnss_ns);
    p.add("ocxo2_edge_gnss_ns",      ocxo_phase.ocxo2_edge_gnss_ns);
    p.add("ocxo1_dwt_before",        ocxo_phase.ocxo1_dwt_before);
    p.add("ocxo1_dwt_after",         ocxo_phase.ocxo1_dwt_after);
    p.add("ocxo1_gpt_before",        ocxo_phase.ocxo1_gpt_before);
    p.add("ocxo1_gpt_after",         ocxo_phase.ocxo1_gpt_after);
    p.add("ocxo1_dwt_bracket_cycles",ocxo_phase.ocxo1_dwt_bracket_cycles);
    p.add("ocxo1_dwt_correction_cycles", ocxo_phase.ocxo1_dwt_correction_cycles);
    p.add("ocxo1_dwt_at_edge",       ocxo_phase.ocxo1_dwt_at_edge);
    p.add("ocxo2_dwt_before",        ocxo_phase.ocxo2_dwt_before);
    p.add("ocxo2_dwt_after",         ocxo_phase.ocxo2_dwt_after);
    p.add("ocxo2_gpt_before",        ocxo_phase.ocxo2_gpt_before);
    p.add("ocxo2_gpt_after",         ocxo_phase.ocxo2_gpt_after);
    p.add("ocxo2_dwt_bracket_cycles",ocxo_phase.ocxo2_dwt_bracket_cycles);
    p.add("ocxo2_dwt_correction_cycles", ocxo_phase.ocxo2_dwt_correction_cycles);
    p.add("ocxo2_dwt_at_edge",       ocxo_phase.ocxo2_dwt_at_edge);

    p.add("ocxo1_phase2_raw_phase_offset_ns", (int32_t)ocxo_phase.ocxo1_phase2_raw_phase_offset_ns);
    p.add("ocxo1_phase2_phase_bias_ns",       ocxo_phase.ocxo1_phase2_phase_bias_ns);
    p.add("ocxo1_phase2_adjusted_phase_signed_ns", ocxo_phase.ocxo1_phase2_adjusted_phase_signed_ns);
    p.add("ocxo1_phase2_phase_offset_ns",     (int32_t)ocxo_phase.ocxo1_phase2_phase_offset_ns);
    p.add("ocxo1_phase2_raw_elapsed_ns",      (int32_t)ocxo_phase.ocxo1_phase2_raw_elapsed_ns);
    p.add("ocxo1_phase2_edge_gnss_ns",    ocxo_phase.ocxo1_phase2_edge_gnss_ns);
    p.add("ocxo1_phase2_dwt_before",      ocxo_phase.ocxo1_phase2_dwt_before);
    p.add("ocxo1_phase2_dwt_after",       ocxo_phase.ocxo1_phase2_dwt_after);
    p.add("ocxo1_phase2_gpt_before",      ocxo_phase.ocxo1_phase2_gpt_before);
    p.add("ocxo1_phase2_gpt_after",       ocxo_phase.ocxo1_phase2_gpt_after);
    p.add("ocxo1_phase2_dwt_bracket_cycles", ocxo_phase.ocxo1_phase2_dwt_bracket_cycles);
    p.add("ocxo1_phase2_dwt_correction_cycles", ocxo_phase.ocxo1_phase2_dwt_correction_cycles);
    p.add("ocxo1_phase2_dwt_at_edge",     ocxo_phase.ocxo1_phase2_dwt_at_edge);
    p.add("ocxo1_phase_pair_delta_ns",             ocxo_phase.ocxo1_phase_pair_delta_ns);
    p.add("ocxo1_adjusted_phase_pair_delta_ns",    ocxo_phase.ocxo1_adjusted_phase_pair_delta_ns);

    p.add("ocxo2_phase2_raw_phase_offset_ns", (int32_t)ocxo_phase.ocxo2_phase2_raw_phase_offset_ns);
    p.add("ocxo2_phase2_phase_bias_ns",       ocxo_phase.ocxo2_phase2_phase_bias_ns);
    p.add("ocxo2_phase2_adjusted_phase_signed_ns", ocxo_phase.ocxo2_phase2_adjusted_phase_signed_ns);
    p.add("ocxo2_phase2_phase_offset_ns",     (int32_t)ocxo_phase.ocxo2_phase2_phase_offset_ns);
    p.add("ocxo2_phase2_raw_elapsed_ns",      (int32_t)ocxo_phase.ocxo2_phase2_raw_elapsed_ns);
    p.add("ocxo2_phase2_edge_gnss_ns",    ocxo_phase.ocxo2_phase2_edge_gnss_ns);
    p.add("ocxo2_phase2_dwt_before",      ocxo_phase.ocxo2_phase2_dwt_before);
    p.add("ocxo2_phase2_dwt_after",       ocxo_phase.ocxo2_phase2_dwt_after);
    p.add("ocxo2_phase2_gpt_before",      ocxo_phase.ocxo2_phase2_gpt_before);
    p.add("ocxo2_phase2_gpt_after",       ocxo_phase.ocxo2_phase2_gpt_after);
    p.add("ocxo2_phase2_dwt_bracket_cycles", ocxo_phase.ocxo2_phase2_dwt_bracket_cycles);
    p.add("ocxo2_phase2_dwt_correction_cycles", ocxo_phase.ocxo2_phase2_dwt_correction_cycles);
    p.add("ocxo2_phase2_dwt_at_edge",     ocxo_phase.ocxo2_phase2_dwt_at_edge);
    p.add("ocxo2_phase_pair_delta_ns",             ocxo_phase.ocxo2_phase_pair_delta_ns);
    p.add("ocxo2_adjusted_phase_pair_delta_ns",    ocxo_phase.ocxo2_adjusted_phase_pair_delta_ns);

    p.add("phase_pair_valid",      (bool)ocxo_phase.phase_pair_valid);
    p.add("ocxo1_gnss_ns_per_pps", ocxo_phase.ocxo1_gnss_ns_per_pps);
    p.add("ocxo2_gnss_ns_per_pps", ocxo_phase.ocxo2_gnss_ns_per_pps);
    p.add("ocxo1_residual_ns",     ocxo_phase.ocxo1_residual_ns);
    p.add("ocxo2_residual_ns",     ocxo_phase.ocxo2_residual_ns);
    p.add("phase_residual_valid",  (bool)ocxo_phase.residual_valid);

    p.add("ocxo1_dac_before",        (int32_t)ocxo_phase.ocxo1_dac_before);
    p.add("ocxo1_dac_after",         (int32_t)ocxo_phase.ocxo1_dac_after);
    p.add("ocxo2_dac_before",        (int32_t)ocxo_phase.ocxo2_dac_before);
    p.add("ocxo2_dac_after",         (int32_t)ocxo_phase.ocxo2_dac_after);

    p.add("ocxo1_dac_n",      (int32_t)dac_welford_ocxo1.n);
    p.add_fmt("ocxo1_dac_mean",   "%.6f", dac_welford_ocxo1.mean);
    p.add_fmt("ocxo1_dac_stddev", "%.6f", dac_welford_stddev(dac_welford_ocxo1));
    p.add_fmt("ocxo1_dac_stderr", "%.6f", dac_welford_stderr(dac_welford_ocxo1));
    p.add_fmt("ocxo1_dac_min",    "%.6f", dac_welford_ocxo1.min_val);
    p.add_fmt("ocxo1_dac_max",    "%.6f", dac_welford_ocxo1.max_val);

    p.add("ocxo2_dac_n",      (int32_t)dac_welford_ocxo2.n);
    p.add_fmt("ocxo2_dac_mean",   "%.6f", dac_welford_ocxo2.mean);
    p.add_fmt("ocxo2_dac_stddev", "%.6f", dac_welford_stddev(dac_welford_ocxo2));
    p.add_fmt("ocxo2_dac_stderr", "%.6f", dac_welford_stderr(dac_welford_ocxo2));
    p.add_fmt("ocxo2_dac_min",    "%.6f", dac_welford_ocxo2.min_val);
    p.add_fmt("ocxo2_dac_max",    "%.6f", dac_welford_ocxo2.max_val);

    p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
    p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);
    p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);

    p.add("diag_pps_rejected_total",     diag_pps_rejected_total);
    p.add("diag_pps_rejected_remainder", diag_pps_rejected_remainder);

    // Spin capture (subset — forensic, persisted in TIMEBASE)
    p.add("spin_valid",             pps_spin.valid);
    p.add("spin_approach_cycles",   pps_spin.approach_cycles);
    p.add("spin_delta_cycles",      pps_spin.delta_cycles);
    p.add("spin_error_cycles",      pps_spin.spin_error);
    p.add("spin_isr_dwt",           pps_spin.isr_dwt);
    p.add("spin_landed_dwt",        pps_spin.landed_dwt);
    p.add("spin_shadow_dwt",        pps_spin.shadow_dwt);
    p.add("spin_corrected_dwt",     pps_spin.corrected_dwt);
    p.add("spin_tdc_correction",    pps_spin.tdc_correction);
    p.add("spin_nano_timed_out",    pps_spin.nano_timed_out);
    p.add("spin_shadow_timed_out",  pps_spin.shadow_timed_out);

    p.add("dwt_cal_valid",     g_dwt_cal_valid);
    p.add("dwt_cal_pps_count", g_dwt_cal_pps_count);

    publish("TIMEBASE_FRAGMENT", p);
  }
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_start(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);

  double dac_val;
  if (args.tryGetDouble("set_dac1", dac_val))  ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_mode = servo_mode_parse(args.getString("calibrate_ocxo"));
  if (calibrate_ocxo_mode != servo_mode_t::OFF) {
    ocxo1_dac.servo_last_step = 0.0; ocxo1_dac.servo_last_residual = 0.0;
    ocxo1_dac.servo_settle_count = 0; ocxo1_dac.servo_adjustments = 0;
    ocxo2_dac.servo_last_step = 0.0; ocxo2_dac.servo_last_residual = 0.0;
    ocxo2_dac.servo_settle_count = 0; ocxo2_dac.servo_adjustments = 0;
  }

  watchdog_anomaly_active = false;
  watchdog_anomaly_publish_pending = false;
  watchdog_anomaly_reason[0] = '\0';

  request_start = true;
  request_stop  = false;

  Payload p;
  p.add("status",         "start_requested");
  p.add("ocxo1_dac",      ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac",      ocxo2_dac.dac_fractional);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  return p;
}

static Payload cmd_stop(const Payload&) {
  watchdog_anomaly_active = false;
  watchdog_anomaly_publish_pending = false;
  watchdog_anomaly_reason[0] = '\0';

  request_stop  = true;
  request_start = false;
  Payload p;
  p.add("status", "stop_requested");
  return p;
}

static Payload cmd_recover(const Payload& args) {
  const char* s_dwt_ns = args.getString("dwt_ns");
  const char* s_gnss   = args.getString("gnss_ns");
  const char* s_ocxo1  = args.getString("ocxo1_ns");
  const char* s_ocxo2  = args.getString("ocxo2_ns");

  if (!s_dwt_ns || !s_gnss || !s_ocxo1 || !s_ocxo2) {
    Payload err;
    err.add("error", "missing recovery parameters (dwt_ns, gnss_ns, ocxo1_ns, ocxo2_ns)");
    return err;
  }

  const char* name = args.getString("campaign");
  if (name && *name) safeCopy(campaign_name, sizeof(campaign_name), name);

  recover_dwt_ns   = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns  = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo1_ns = strtoull(s_ocxo1,  nullptr, 10);
  recover_ocxo2_ns = strtoull(s_ocxo2,  nullptr, 10);

  double dac_val;
  if (args.tryGetDouble("set_dac1", dac_val))  ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_mode = servo_mode_parse(args.getString("calibrate_ocxo"));
  if (calibrate_ocxo_mode != servo_mode_t::OFF) {
    ocxo1_dac.servo_last_step = 0.0; ocxo1_dac.servo_last_residual = 0.0;
    ocxo1_dac.servo_settle_count = 0;
    ocxo2_dac.servo_last_step = 0.0; ocxo2_dac.servo_last_residual = 0.0;
    ocxo2_dac.servo_settle_count = 0;
  }

  watchdog_anomaly_active = false;
  watchdog_anomaly_publish_pending = false;
  watchdog_anomaly_reason[0] = '\0';

  request_recover = true;
  request_start   = false;
  request_stop    = false;

  Payload p;
  p.add("status", "recover_requested");
  return p;
}

static Payload cmd_watchdog_test(const Payload&) {
  clocks_watchdog_anomaly("watchdog_test");
  Payload p;
  p.add("status", "watchdog_anomaly_requested");
  return p;
}

// ============================================================================
// Helpers — residual & prediction stats into a Payload
// ============================================================================

static void report_residual(Payload& p, const char* prefix, const pps_residual_t& r) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pps_valid",    prefix); p.add(key, r.valid);
  snprintf(key, sizeof(key), "%s_pps_delta",    prefix); p.add(key, r.delta);
  snprintf(key, sizeof(key), "%s_pps_residual", prefix); p.add(key, r.residual);
  snprintf(key, sizeof(key), "%s_pps_n",        prefix); p.add(key, r.n);
  snprintf(key, sizeof(key), "%s_pps_mean",     prefix); p.add_fmt(key, "%.3f", r.mean);
  snprintf(key, sizeof(key), "%s_pps_stddev",   prefix); p.add_fmt(key, "%.3f", residual_stddev(r));
  snprintf(key, sizeof(key), "%s_pps_stderr",   prefix); p.add_fmt(key, "%.3f", residual_stderr(r));
}

static void report_prediction(Payload& p, const char* prefix, const prediction_tracker_t& t) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pred_residual",  prefix); p.add(key, t.pred_residual);
  snprintf(key, sizeof(key), "%s_pred_n",         prefix); p.add(key, t.n);
  snprintf(key, sizeof(key), "%s_pred_mean",      prefix); p.add_fmt(key, "%.3f", t.mean);
  snprintf(key, sizeof(key), "%s_pred_stddev",    prefix); p.add_fmt(key, "%.3f", prediction_stddev(t));
  snprintf(key, sizeof(key), "%s_pred_stderr",    prefix); p.add_fmt(key, "%.3f", prediction_stderr(t));
  snprintf(key, sizeof(key), "%s_pred_valid",     prefix); p.add(key, t.predicted_valid);
  snprintf(key, sizeof(key), "%s_pred_predicted", prefix); p.add(key, t.predicted);
  snprintf(key, sizeof(key), "%s_delta_raw",      prefix); p.add(key, t.prev_delta);
}


static void report_dac_welford(Payload& p, const char* prefix, const dac_welford_t& w) {
  char key[48];
  snprintf(key, sizeof(key), "%s_dac_n",      prefix); p.add(key, (int32_t)w.n);
  snprintf(key, sizeof(key), "%s_dac_mean",    prefix); p.add_fmt(key, "%.6f", w.mean);
  snprintf(key, sizeof(key), "%s_dac_stddev",  prefix); p.add_fmt(key, "%.6f", dac_welford_stddev(w));
  snprintf(key, sizeof(key), "%s_dac_stderr",  prefix); p.add_fmt(key, "%.6f", dac_welford_stderr(w));
  snprintf(key, sizeof(key), "%s_dac_min",     prefix); p.add_fmt(key, "%.6f", w.min_val);
  snprintf(key, sizeof(key), "%s_dac_max",     prefix); p.add_fmt(key, "%.6f", w.max_val);
}
// ============================================================================
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  p.add("campaign", campaign_name);
  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign_seconds", campaign_seconds);
  }

  p.add("watchdog_anomaly_active",  (bool)watchdog_anomaly_active);
  p.add("watchdog_anomaly_pending", (bool)watchdog_anomaly_publish_pending);
  p.add("watchdog_anomaly_sequence", watchdog_anomaly_sequence);
  p.add("watchdog_anomaly_reason", watchdog_anomaly_reason);
  p.add("watchdog_anomaly_detail0", watchdog_anomaly_detail0);
  p.add("watchdog_anomaly_detail1", watchdog_anomaly_detail1);
  p.add("watchdog_anomaly_detail2", watchdog_anomaly_detail2);
  p.add("watchdog_anomaly_detail3", watchdog_anomaly_detail3);
  p.add("watchdog_anomaly_trigger_dwt", watchdog_anomaly_trigger_dwt);

  p.add("request_start",   request_start);
  p.add("request_stop",    request_stop);
  p.add("request_recover", request_recover);

  const uint64_t dwt_cycles = clocks_dwt_cycles_now();
  const uint64_t dwt_ns     = clocks_dwt_ns_now();
  const uint64_t gnss_ns    = clocks_gnss_ns_now();
  const uint64_t ocxo1_ns   = clocks_ocxo1_ns_now();
  const uint64_t ocxo2_ns   = clocks_ocxo2_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo1_ns_now",   ocxo1_ns);
  p.add("ocxo2_ns_now",   ocxo2_ns);
  p.add("gnss_lock",      digitalRead(GNSS_LOCK_PIN));

  const int64_t tb_gnss  = timebase_now_ns(timebase_domain_t::GNSS);
  const int64_t tb_dwt   = timebase_now_ns(timebase_domain_t::DWT);
  const int64_t tb_ocxo1 = timebase_now_ns(timebase_domain_t::OCXO1);
  const int64_t tb_ocxo2 = timebase_now_ns(timebase_domain_t::OCXO2);

  p.add("timebase_gnss_ns",  tb_gnss);
  p.add("timebase_dwt_ns",   tb_dwt);
  p.add("timebase_ocxo1_ns", tb_ocxo1);
  p.add("timebase_ocxo2_ns", tb_ocxo2);
  p.add("timebase_valid",    timebase_valid());

  report_residual(p, "dwt",   residual_dwt);
  report_residual(p, "gnss",  residual_gnss);
  report_residual(p, "ocxo1", residual_ocxo1);
  report_residual(p, "ocxo2", residual_ocxo2);

  report_prediction(p, "dwt",   pred_dwt);
  report_prediction(p, "ocxo1", pred_ocxo1);
  report_prediction(p, "ocxo2", pred_ocxo2);

  report_dac_welford(p, "ocxo1", dac_welford_ocxo1);
  report_dac_welford(p, "ocxo2", dac_welford_ocxo2);

  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo1", isr_residual_ocxo1);
  p.add("isr_residual_ocxo2", isr_residual_ocxo2);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  p.add("ocxo1_dac",               ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac",               ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_hw",            (int32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw",            (int32_t)ocxo2_dac.dac_hw_code);

  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);
  p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);
  p.add("ocxo1_servo_last_step",   ocxo1_dac.servo_last_step);
  p.add("ocxo2_servo_last_step",   ocxo2_dac.servo_last_step);
  p.add("ocxo1_servo_last_residual", ocxo1_dac.servo_last_residual);
  p.add("ocxo2_servo_last_residual", ocxo2_dac.servo_last_residual);
  p.add("ocxo1_servo_settle_count",  ocxo1_dac.servo_settle_count);
  p.add("ocxo2_servo_settle_count",  ocxo2_dac.servo_settle_count);

  p.add("dwt_cal_cycles_per_s",  g_dwt_cycles_per_gnss_s);
  p.add("dwt_cal_valid",         g_dwt_cal_valid);
  p.add("dwt_cal_pps_count",     g_dwt_cal_pps_count);

  if (campaign_state == clocks_campaign_state_t::STARTED && gnss_ns > 0) {
    p.add_fmt("tau_dwt",   "%.12f", (double)dwt_ns   / (double)gnss_ns);
    p.add_fmt("tau_ocxo1", "%.12f", (double)ocxo1_ns / (double)gnss_ns);
    p.add_fmt("tau_ocxo2", "%.12f", (double)ocxo2_ns / (double)gnss_ns);
    p.add_fmt("dwt_ppb",   "%.3f", ((double)((int64_t)dwt_ns   - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
    p.add_fmt("ocxo1_ppb", "%.3f", ((double)((int64_t)ocxo1_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
    p.add_fmt("ocxo2_ppb", "%.3f", ((double)((int64_t)ocxo2_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
  } else {
    p.add("tau_dwt", 0.0); p.add("tau_ocxo1", 0.0); p.add("tau_ocxo2", 0.0);
    p.add("dwt_ppb", 0.0); p.add("ocxo1_ppb", 0.0); p.add("ocxo2_ppb", 0.0);
  }

  return p;
}

// ============================================================================
// CLOCKS_INFO — forensic / diagnostic surface
//
// Added forensic fields:
//   - time_pps_count
//   - gnss_raw_now
//   - gnss_raw_64
//   - gnss_ticks_64
//   - gnss_rolling_raw_64
//   - gnss_rolling_32
//   - gnss_raw_since_pps
//   - isr_snap_*
//   - isr_prev_*
//   - isr_residual_valid
//   - isr_residual_*
//
// Notes:
//   - gnss_raw_now is the instantaneous qtimer1_read_32() raw value
//   - gnss_raw_64 is the campaign-scoped accumulated raw GNSS count
//   - gnss_ticks_64 is gnss_raw_64 (10 MHz, no divisor)
//   - gnss_raw_since_pps is current raw delta from the most recent PPS snapshot
// ============================================================================

static Payload cmd_clocks_info(const Payload&) {
  Payload p;

  const uint32_t gnss_raw_now = qtimer1_read_32();
  const uint32_t gnss_raw_since_pps = gnss_raw_now - (uint32_t)isr_snap_gnss;

  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  p.add("campaign", campaign_name);
  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign_seconds", campaign_seconds);
  }

  p.add("watchdog_anomaly_active",  (bool)watchdog_anomaly_active);
  p.add("watchdog_anomaly_pending", (bool)watchdog_anomaly_publish_pending);
  p.add("watchdog_anomaly_sequence", watchdog_anomaly_sequence);
  p.add("watchdog_anomaly_reason", watchdog_anomaly_reason);
  p.add("watchdog_anomaly_detail0", watchdog_anomaly_detail0);
  p.add("watchdog_anomaly_detail1", watchdog_anomaly_detail1);
  p.add("watchdog_anomaly_detail2", watchdog_anomaly_detail2);
  p.add("watchdog_anomaly_detail3", watchdog_anomaly_detail3);
  p.add("watchdog_anomaly_trigger_dwt", watchdog_anomaly_trigger_dwt);

  p.add("ad5693r_init_ok", g_ad5693r_init_ok);

  p.add("time_pps_count", time_pps_count());

  p.add("dwt_cycles_now", clocks_dwt_cycles_now());
  p.add("dwt_ns_now",     clocks_dwt_ns_now());
  p.add("gnss_ns_now",    clocks_gnss_ns_now());
  p.add("ocxo1_ns_now",   clocks_ocxo1_ns_now());
  p.add("ocxo2_ns_now",   clocks_ocxo2_ns_now());

  // ── Raw / rolling GNSS forensic surface ──
  p.add("gnss_raw_now",         gnss_raw_now);
  p.add("gnss_raw_64",          gnss_raw_64);
  p.add("gnss_ticks_64",        gnss_ticks_64_get());
  p.add("gnss_rolling_raw_64",  gnss_rolling_raw_64);
  p.add("gnss_rolling_32",      gnss_rolling_32);
  p.add("gnss_raw_since_pps",   gnss_raw_since_pps);

  p.add("timebase_gnss_ns",  timebase_now_ns(timebase_domain_t::GNSS));
  p.add("timebase_dwt_ns",   timebase_now_ns(timebase_domain_t::DWT));
  p.add("timebase_ocxo1_ns", timebase_now_ns(timebase_domain_t::OCXO1));
  p.add("timebase_ocxo2_ns", timebase_now_ns(timebase_domain_t::OCXO2));
  p.add("timebase_valid",            timebase_valid());
  p.add("timebase_conversion_valid", timebase_conversion_valid());

  const timebase_fragment_t* tb_frag = timebase_last_fragment();
  if (tb_frag && tb_frag->valid) {
    p.add("timebase_fragment_valid",              true);
    p.add("timebase_fragment_gnss_ns",            (uint64_t)tb_frag->gnss_ns);
    p.add("timebase_fragment_dwt_ns",             (uint64_t)tb_frag->dwt_ns);
    p.add("timebase_fragment_dwt_cycles",         (uint64_t)tb_frag->dwt_cycles);
    p.add("timebase_fragment_ocxo1_ns",           (uint64_t)tb_frag->ocxo1_ns);
    p.add("timebase_fragment_ocxo2_ns",           (uint64_t)tb_frag->ocxo2_ns);
    p.add("timebase_fragment_pps_count",          (uint32_t)tb_frag->pps_count);
    p.add("timebase_fragment_isr_residual_dwt",   (int32_t)tb_frag->isr_residual_dwt);
    p.add("timebase_fragment_isr_residual_gnss",  (int32_t)tb_frag->isr_residual_gnss);
    p.add("timebase_fragment_isr_residual_ocxo1", (int32_t)tb_frag->isr_residual_ocxo1);
    p.add("timebase_fragment_isr_residual_ocxo2", (int32_t)tb_frag->isr_residual_ocxo2);
  } else {
    p.add("timebase_fragment_valid", false);
  }

  p.add("dwt_cal_cycles_per_s",  g_dwt_cycles_per_gnss_s);
  p.add("dwt_cal_valid",         g_dwt_cal_valid);
  p.add("dwt_cal_pps_count",     g_dwt_cal_pps_count);

  // ── ISR PPS-edge forensics ──
  p.add("isr_snap_dwt",         isr_snap_dwt);
  p.add("isr_snap_gnss",        isr_snap_gnss);
  p.add("isr_snap_ocxo1",       isr_snap_ocxo1);
  p.add("isr_snap_ocxo2",       isr_snap_ocxo2);

  p.add("isr_prev_dwt",         isr_prev_dwt);
  p.add("isr_prev_gnss",        isr_prev_gnss);
  p.add("isr_prev_ocxo1",       isr_prev_ocxo1);
  p.add("isr_prev_ocxo2",       isr_prev_ocxo2);

  p.add("isr_residual_valid",   isr_residual_valid);
  p.add("isr_residual_dwt",     isr_residual_dwt);
  p.add("isr_residual_gnss",    isr_residual_gnss);
  p.add("isr_residual_ocxo1",   isr_residual_ocxo1);
  p.add("isr_residual_ocxo2",   isr_residual_ocxo2);

  // ── PPS rejection diagnostics ──
  p.add("pps_rejected_total",      diag_pps_rejected_total);
  p.add("pps_rejected_remainder",  diag_pps_rejected_remainder);
  p.add("pps_reject_consecutive",  diag_pps_reject_consecutive);
  p.add("pps_reject_recoveries",   diag_pps_reject_recoveries);
  p.add("pps_reject_max_run",      diag_pps_reject_max_run);

  // ── PPS ASAP pipeline diagnostics ──
  p.add("pps_scheduled_stuck",     diag_pps_scheduled_stuck);
  p.add("pps_watchdog_recoveries", diag_pps_watchdog_recoveries);
  p.add("pps_asap_arm_failures",   diag_pps_asap_arm_failures);
  p.add("pps_asap_armed",          diag_pps_asap_armed);
  p.add("pps_asap_dispatched",     diag_pps_asap_dispatched);
  p.add("pps_stuck_since_dwt",     diag_pps_stuck_since_dwt);
  p.add("pps_stuck_max",           diag_pps_stuck_max);
  p.add("pps_scheduled",           (bool)pps_scheduled);

  // ── Spin capture forensics ──
  //
  // Two timeout classes:
  //   nano_*   — TimePop nano-spin couldn't reach target DWT
  //   shadow_* — shadow-write loop PPS never arrived
  //
  p.add("spin_valid",             pps_spin.valid);
  p.add("spin_armed",             pps_spin.armed);
  p.add("spin_completed",         pps_spin.completed);
  p.add("spin_target_dwt",        pps_spin.target_dwt);
  p.add("spin_landed_dwt",        pps_spin.landed_dwt);
  p.add("spin_landed_gnss_ns",    pps_spin.landed_gnss_ns);
  p.add("spin_error_cycles",      pps_spin.spin_error);
  p.add("spin_shadow_dwt",        pps_spin.shadow_dwt);
  p.add("spin_isr_dwt",           pps_spin.isr_dwt);
  p.add("spin_delta_cycles",      pps_spin.delta_cycles);
  p.add("spin_approach_cycles",   pps_spin.approach_cycles);
  p.add("spin_corrected_dwt",     pps_spin.corrected_dwt);
  p.add("spin_tdc_correction",    pps_spin.tdc_correction);

  // Nano-spin timeout (couldn't reach target DWT)
  p.add("spin_nano_timed_out",    pps_spin.nano_timed_out);
  p.add("spin_nano_timeouts",     pps_spin.nano_timeouts);

  // Shadow-loop timeout (PPS never arrived)
  p.add("spin_shadow_timed_out",  pps_spin.shadow_timed_out);
  p.add("spin_shadow_timeouts",   pps_spin.shadow_timeouts);

  // Lifetime counters
  p.add("spin_arms",              pps_spin.arms);
  p.add("spin_arm_failures",      pps_spin.arm_failures);
  p.add("spin_completions",       pps_spin.completions);
  p.add("spin_misses",            pps_spin.misses);

  // ── DEBUG: shadow capture forensics ──
  p.add("dbg_post_dwt",      dbg_post_loop_dwt);
  p.add("dbg_post_shadow",   dbg_post_loop_shadow);
  p.add("dbg_post_isr_cap",  dbg_post_loop_isr_cap);
  p.add("dbg_post_isr_snap", dbg_post_loop_isr_snap);

  // ── QTimer1 read diagnostics ──
  p.add("qread_total",            diag_qread_total);
  p.add("qread_same_hi",          diag_qread_same_hi);
  p.add("qread_retry_hi_changed", diag_qread_retry_hi_changed);
  p.add("qread_last_hi1",         diag_qread_last_hi1);
  p.add("qread_last_hi2",         diag_qread_last_hi2);
  p.add("qread_last_lo",          diag_qread_last_lo);
  p.add("qread_last_lo2",         diag_qread_last_lo2);

  // ── OCXO phase capture (v26) ──
  p.add("phase_valid",                  (bool)ocxo_phase.valid);
  p.add("phase_captures",               ocxo_phase.captures);
  p.add("phase_dwt_at_pps",             ocxo_phase.dwt_at_pps);
  p.add("phase_pps_gnss_ns",            ocxo_phase.pps_gnss_ns);

  p.add("phase_ocxo1_dwt_at_edge",      ocxo_phase.ocxo1_dwt_at_edge);
  p.add("phase_ocxo1_dwt_elapsed",      ocxo_phase.ocxo1_dwt_elapsed);
  p.add("phase_ocxo1_raw_elapsed_ns",      ocxo_phase.ocxo1_raw_elapsed_ns);
  p.add("phase_ocxo1_raw_phase_offset_ns",  ocxo_phase.ocxo1_raw_phase_offset_ns);
  p.add("phase_ocxo1_phase_bias_ns",        ocxo_phase.ocxo1_phase_bias_ns);
  p.add("phase_ocxo1_adjusted_phase_signed",ocxo_phase.ocxo1_adjusted_phase_signed_ns);
  p.add("phase_ocxo1_phase_offset_ns",      ocxo_phase.ocxo1_phase_offset_ns);
  p.add("phase_ocxo1_edge_gnss_ns",     ocxo_phase.ocxo1_edge_gnss_ns);
  p.add("phase_prev_ocxo1_edge_gnss_ns",ocxo_phase.prev_ocxo1_edge_gnss_ns);
  p.add("phase_ocxo1_dwt_before",        ocxo_phase.ocxo1_dwt_before);
  p.add("phase_ocxo1_dwt_after",         ocxo_phase.ocxo1_dwt_after);
  p.add("phase_ocxo1_gpt_before",        ocxo_phase.ocxo1_gpt_before);
  p.add("phase_ocxo1_gpt_after",         ocxo_phase.ocxo1_gpt_after);
  p.add("phase_ocxo1_dwt_bracket_cycles",ocxo_phase.ocxo1_dwt_bracket_cycles);
  p.add("phase_ocxo1_dwt_correction_cycles",ocxo_phase.ocxo1_dwt_correction_cycles);

  p.add("phase_ocxo2_dwt_at_edge",      ocxo_phase.ocxo2_dwt_at_edge);
  p.add("phase_ocxo2_dwt_elapsed",      ocxo_phase.ocxo2_dwt_elapsed);
  p.add("phase_ocxo2_raw_elapsed_ns",      ocxo_phase.ocxo2_raw_elapsed_ns);
  p.add("phase_ocxo2_raw_phase_offset_ns",  ocxo_phase.ocxo2_raw_phase_offset_ns);
  p.add("phase_ocxo2_phase_bias_ns",        ocxo_phase.ocxo2_phase_bias_ns);
  p.add("phase_ocxo2_adjusted_phase_signed",ocxo_phase.ocxo2_adjusted_phase_signed_ns);
  p.add("phase_ocxo2_phase_offset_ns",      ocxo_phase.ocxo2_phase_offset_ns);
  p.add("phase_ocxo2_edge_gnss_ns",     ocxo_phase.ocxo2_edge_gnss_ns);
  p.add("phase_prev_ocxo2_edge_gnss_ns",ocxo_phase.prev_ocxo2_edge_gnss_ns);
  p.add("phase_ocxo2_dwt_before",        ocxo_phase.ocxo2_dwt_before);
  p.add("phase_ocxo2_dwt_after",         ocxo_phase.ocxo2_dwt_after);
  p.add("phase_ocxo2_gpt_before",        ocxo_phase.ocxo2_gpt_before);
  p.add("phase_ocxo2_gpt_after",         ocxo_phase.ocxo2_gpt_after);
  p.add("phase_ocxo2_dwt_bracket_cycles",ocxo_phase.ocxo2_dwt_bracket_cycles);
  p.add("phase_ocxo2_dwt_correction_cycles",ocxo_phase.ocxo2_dwt_correction_cycles);
  p.add("phase_ocxo1_phase2_raw_phase_offset_ns", ocxo_phase.ocxo1_phase2_raw_phase_offset_ns);
  p.add("phase_ocxo1_phase2_phase_bias_ns",       ocxo_phase.ocxo1_phase2_phase_bias_ns);
  p.add("phase_ocxo1_phase2_adjusted_phase_signed", ocxo_phase.ocxo1_phase2_adjusted_phase_signed_ns);
  p.add("phase_ocxo1_phase2_phase_offset_ns",     ocxo_phase.ocxo1_phase2_phase_offset_ns);
  p.add("phase_ocxo1_phase2_raw_elapsed_ns",      ocxo_phase.ocxo1_phase2_raw_elapsed_ns);
  p.add("phase_ocxo1_phase2_edge_gnss_ns",    ocxo_phase.ocxo1_phase2_edge_gnss_ns);
  p.add("phase_ocxo1_phase2_dwt_before",      ocxo_phase.ocxo1_phase2_dwt_before);
  p.add("phase_ocxo1_phase2_dwt_after",       ocxo_phase.ocxo1_phase2_dwt_after);
  p.add("phase_ocxo1_phase2_gpt_before",      ocxo_phase.ocxo1_phase2_gpt_before);
  p.add("phase_ocxo1_phase2_gpt_after",       ocxo_phase.ocxo1_phase2_gpt_after);
  p.add("phase_ocxo1_phase2_dwt_bracket_cycles", ocxo_phase.ocxo1_phase2_dwt_bracket_cycles);
  p.add("phase_ocxo1_phase2_dwt_correction_cycles", ocxo_phase.ocxo1_phase2_dwt_correction_cycles);
  p.add("phase_ocxo1_phase2_dwt_at_edge",     ocxo_phase.ocxo1_phase2_dwt_at_edge);
  p.add("phase_ocxo1_phase_pair_delta_ns",             ocxo_phase.ocxo1_phase_pair_delta_ns);
  p.add("phase_ocxo1_adjusted_phase_pair_delta_ns",    ocxo_phase.ocxo1_adjusted_phase_pair_delta_ns);
  p.add("phase_ocxo2_phase2_raw_phase_offset_ns", ocxo_phase.ocxo2_phase2_raw_phase_offset_ns);
  p.add("phase_ocxo2_phase2_phase_bias_ns",       ocxo_phase.ocxo2_phase2_phase_bias_ns);
  p.add("phase_ocxo2_phase2_adjusted_phase_signed", ocxo_phase.ocxo2_phase2_adjusted_phase_signed_ns);
  p.add("phase_ocxo2_phase2_phase_offset_ns",     ocxo_phase.ocxo2_phase2_phase_offset_ns);
  p.add("phase_ocxo2_phase2_raw_elapsed_ns",      ocxo_phase.ocxo2_phase2_raw_elapsed_ns);
  p.add("phase_ocxo2_phase2_edge_gnss_ns",    ocxo_phase.ocxo2_phase2_edge_gnss_ns);
  p.add("phase_ocxo2_phase2_dwt_before",      ocxo_phase.ocxo2_phase2_dwt_before);
  p.add("phase_ocxo2_phase2_dwt_after",       ocxo_phase.ocxo2_phase2_dwt_after);
  p.add("phase_ocxo2_phase2_gpt_before",      ocxo_phase.ocxo2_phase2_gpt_before);
  p.add("phase_ocxo2_phase2_gpt_after",       ocxo_phase.ocxo2_phase2_gpt_after);
  p.add("phase_ocxo2_phase2_dwt_bracket_cycles", ocxo_phase.ocxo2_phase2_dwt_bracket_cycles);
  p.add("phase_ocxo2_phase2_dwt_correction_cycles", ocxo_phase.ocxo2_phase2_dwt_correction_cycles);
  p.add("phase_ocxo2_phase2_dwt_at_edge",     ocxo_phase.ocxo2_phase2_dwt_at_edge);
  p.add("phase_ocxo2_phase_pair_delta_ns",             ocxo_phase.ocxo2_phase_pair_delta_ns);
  p.add("phase_ocxo2_adjusted_phase_pair_delta_ns",    ocxo_phase.ocxo2_adjusted_phase_pair_delta_ns);
  p.add("phase_pair_valid",                   (bool)ocxo_phase.phase_pair_valid);

  p.add("phase_residual_valid",         (bool)ocxo_phase.residual_valid);
  p.add("phase_ocxo1_gnss_ns_per_pps",  ocxo_phase.ocxo1_gnss_ns_per_pps);
  p.add("phase_ocxo2_gnss_ns_per_pps",  ocxo_phase.ocxo2_gnss_ns_per_pps);
  p.add("phase_ocxo1_residual_ns",      ocxo_phase.ocxo1_residual_ns);
  p.add("phase_ocxo2_residual_ns",      ocxo_phase.ocxo2_residual_ns);

  return p;
}

// ============================================================================
// INTERP_TEST — uses DWT interpolation against GNSS tick position
//
// NOTE: qtimer_at_pps is the authoritative GNSS VCLOCK PPS snapshot.
// INTERP_TEST uses the campaign GNSS ticks from the timebase fragment
// plus QTimer1 raw position for intra-second GNSS ground truth.
// ============================================================================

static Payload cmd_interp_test(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  const int32_t count = args.getInt("count", 10);
  const int32_t n = (count < 1) ? 1 : (count > 100) ? 100 : count;

  int64_t  w_n = 0; double w_mean = 0.0, w_m2 = 0.0;
  int64_t  err_min = INT64_MAX, err_max = INT64_MIN;
  int32_t  outside_100ns = 0;

  uint64_t s0_frag_gnss_ns = 0; uint32_t s0_frag_dwt_cyccnt_at_pps = 0;
  uint32_t s0_frag_dwt_cycles_per_pps = 0, s0_pps_count = 0;
  uint32_t s0_dwt_cyccnt_now = 0, s0_dwt_elapsed = 0;
  uint64_t s0_dwt_ns_into_second = 0, s0_interp_gnss_ns = 0;
  uint32_t s0_gnss_raw_since_pps = 0;
  uint64_t s0_vclock_ns_into_second = 0, s0_vclock_gnss_ns = 0;
  int64_t  s0_error_ns = 0; double s0_position = 0.0;

  for (int32_t i = 0; i < n; i++) {
    const timebase_fragment_t* frag = timebase_last_fragment();
    if (!frag || !frag->valid) { p.add("error", "timebase not valid"); return p; }

    const uint64_t frag_gnss_ns            = frag->gnss_ns;
    const uint32_t frag_pps_count          = frag->pps_count;
    const uint32_t frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
    const uint32_t frag_dwt_cycles_per_pps = (uint32_t)frag->dwt_cycles_per_pps;

    if (frag_dwt_cycles_per_pps == 0) {
      p.add("error", "fragment missing dwt_cycles_per_pps"); return p;
    }

    const uint32_t dwt_now  = ARM_DWT_CYCCNT;
    const uint32_t gnss_now = qtimer1_read_32();

    const int64_t interp_signed = timebase_gnss_ns_from_dwt(
      dwt_now, frag_gnss_ns, frag_dwt_cyccnt_at_pps, frag_dwt_cycles_per_pps);
    if (interp_signed < 0) { p.add("error", "timebase_gnss_ns_from_dwt failed"); return p; }
    const uint64_t interp_gnss_ns = (uint64_t)interp_signed;

    // GNSS VCLOCK is on QTimer1 (10 MHz single-edge).  Raw IS 10 MHz ticks.
    const uint32_t gnss_raw_since_pps = gnss_now - (uint32_t)isr_snap_gnss;
    const uint64_t vclock_ns_into_second = (uint64_t)gnss_raw_since_pps * 100ULL;
    const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;
    const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;
    const uint32_t dwt_elapsed = dwt_now - frag_dwt_cyccnt_at_pps;
    const uint64_t dwt_ns_into_second =
      (uint64_t)dwt_elapsed * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;

    w_n++;
    const double x = (double)error_ns, d1 = x - w_mean;
    w_mean += d1 / (double)w_n;
    w_m2 += (x - w_mean) * d1;
    if (error_ns < err_min) err_min = error_ns;
    if (error_ns > err_max) err_max = error_ns;
    if (error_ns > 100 || error_ns < -100) outside_100ns++;

    if (i == 0) {
      s0_frag_gnss_ns = frag_gnss_ns; s0_frag_dwt_cyccnt_at_pps = frag_dwt_cyccnt_at_pps;
      s0_frag_dwt_cycles_per_pps = frag_dwt_cycles_per_pps;
      s0_pps_count = frag_pps_count; s0_dwt_cyccnt_now = dwt_now;
      s0_dwt_elapsed = dwt_elapsed; s0_dwt_ns_into_second = dwt_ns_into_second;
      s0_interp_gnss_ns = interp_gnss_ns; s0_gnss_raw_since_pps = gnss_raw_since_pps;
      s0_vclock_ns_into_second = vclock_ns_into_second; s0_vclock_gnss_ns = vclock_gnss_ns;
      s0_error_ns = error_ns; s0_position = (double)vclock_ns_into_second / 1000000000.0;
    }
  }

  const double w_stddev = (w_n >= 2) ? sqrt(w_m2 / (double)(w_n - 1)) : 0.0;
  p.add("samples", (int32_t)w_n); p.add("error_mean_ns", w_mean);
  p.add("error_stddev_ns", w_stddev);
  p.add("error_stderr_ns", (w_n >= 2) ? w_stddev / sqrt((double)w_n) : 0.0);
  p.add("error_min_ns", err_min); p.add("error_max_ns", err_max);
  p.add("outside_100ns", outside_100ns);
  p.add("s0_frag_gnss_ns", s0_frag_gnss_ns);
  p.add("s0_frag_dwt_cyccnt_at_pps", s0_frag_dwt_cyccnt_at_pps);
  p.add("s0_frag_dwt_cycles_per_pps", s0_frag_dwt_cycles_per_pps);
  p.add("s0_pps_count", s0_pps_count);
  p.add("s0_dwt_cyccnt_now", s0_dwt_cyccnt_now);
  p.add("s0_dwt_elapsed", s0_dwt_elapsed);
  p.add("s0_dwt_ns_into_second", s0_dwt_ns_into_second);
  p.add("s0_interp_gnss_ns", s0_interp_gnss_ns);
  p.add("s0_gnss_raw_since_pps", s0_gnss_raw_since_pps);
  p.add("s0_vclock_ns_into_second", s0_vclock_ns_into_second);
  p.add("s0_vclock_gnss_ns", s0_vclock_gnss_ns);
  p.add("s0_error_ns", s0_error_ns); p.add("s0_position", s0_position);
  p.add("campaign_seconds", campaign_seconds);
  p.add("pps_rejected_total", diag_pps_rejected_total);
  return p;
}

// ============================================================================
// INTERP_PROOF — Head-to-head interpolation accuracy test
//
// Compares two interpolation strategies against VCLOCK ground truth
// at the same instant:
//
//   Path A (PPS anchor):
//     Interpolate from the PPS-edge DWT using dwt_cycles_per_pps.
//     This is the current production path — the interpolation window
//     is up to 1 second (wherever in the second the query lands).
//
//   Path B (Spin anchor):
//     Interpolate from the spin capture's landed_dwt / landed_gnss_ns.
//     The spin lands ~50 µs before the PPS edge, so the interpolation
//     window is the time since the spin landing.  Currently this is
//     still up to ~1 second (the spin only fires once per PPS), but
//     the anchor itself is more precise (2.3 ns jitter vs ~47-51
//     cycle ISR latency).
//
//     When 10 KHz anchoring is added, the spin anchor will update
//     every 100 µs, reducing the interpolation window from ~500 ms
//     (average) to ~50 µs — a 10,000x reduction.
//
//   Ground truth:
//     GNSS VCLOCK (QTimer1) ticks since PPS, multiplied by 100 ns.
//     This gives the true position on a 100 ns grid.
//
// The test accumulates independent Welford stats for both paths,
// plus position correlation for each.  The comparison answers:
// "Is the spin anchor better than the PPS anchor, and by how much?"
//
// This is a forensic/validation tool, not a production path.
// It runs on command and does not affect TIMEBASE production.
// ============================================================================

// ── Accumulators (persistent across calls, reset on command) ──

// Path A: PPS anchor (current production interpolation)
static int64_t proof_a_n = 0;
static double  proof_a_mean = 0.0, proof_a_m2 = 0.0;
static int64_t proof_a_min = 0, proof_a_max = 0;
static int32_t proof_a_outside_100 = 0;
static double  proof_a_pos_mean = 0.0, proof_a_pos_m2 = 0.0, proof_a_covar = 0.0;

// Path B: Spin anchor (TDC-corrected spin landing)
static int64_t proof_b_n = 0;
static double  proof_b_mean = 0.0, proof_b_m2 = 0.0;
static int64_t proof_b_min = 0, proof_b_max = 0;
static int32_t proof_b_outside_100 = 0;
static double  proof_b_pos_mean = 0.0, proof_b_pos_m2 = 0.0, proof_b_covar = 0.0;

static void proof_reset(void) {
  proof_a_n = 0; proof_a_mean = 0.0; proof_a_m2 = 0.0;
  proof_a_min = 0; proof_a_max = 0; proof_a_outside_100 = 0;
  proof_a_pos_mean = 0.0; proof_a_pos_m2 = 0.0; proof_a_covar = 0.0;

  proof_b_n = 0; proof_b_mean = 0.0; proof_b_m2 = 0.0;
  proof_b_min = 0; proof_b_max = 0; proof_b_outside_100 = 0;
  proof_b_pos_mean = 0.0; proof_b_pos_m2 = 0.0; proof_b_covar = 0.0;
}

// Welford update helper (inline, no allocation)
static inline void proof_welford(
  int64_t& n, double& mean, double& m2,
  int64_t& err_min, int64_t& err_max, int32_t& outside,
  double& pos_mean, double& pos_m2, double& covar,
  int64_t error_ns, double position
) {
  n++;
  const double x = (double)error_ns;
  const double d1 = x - mean;
  mean += d1 / (double)n;
  const double d2 = x - mean;
  m2 += d1 * d2;

  if (n == 1) { err_min = error_ns; err_max = error_ns; }
  else {
    if (error_ns < err_min) err_min = error_ns;
    if (error_ns > err_max) err_max = error_ns;
  }
  if (error_ns > 100 || error_ns < -100) outside++;

  const double pd1 = position - pos_mean;
  pos_mean += pd1 / (double)n;
  pos_m2 += (position - pos_mean) * pd1;
  covar += d2 * pd1;
}

static Payload cmd_interp_proof(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign"); return p;
  }

  if (args.getInt("reset", 0)) {
    proof_reset();
  }

  // ── Single-point capture: DWT + VCLOCK as close together as possible ──
  const uint32_t dwt_now  = DWT_CYCCNT;
  const uint32_t gnss_now = qtimer1_read_32();

  // ── Fragment (the PPS-edge truth) ──
  const timebase_fragment_t* frag = timebase_last_fragment();
  if (!frag || !frag->valid) {
    p.add("error", "timebase not valid"); return p;
  }

  const uint64_t frag_gnss_ns            = frag->gnss_ns;
  const uint32_t frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
  const uint32_t frag_dwt_cycles_per_pps = (uint32_t)frag->dwt_cycles_per_pps;

  if (frag_dwt_cycles_per_pps == 0) {
    p.add("error", "fragment missing dwt_cycles_per_pps"); return p;
  }

  // ── Ground truth: VCLOCK position ──
  const uint32_t gnss_raw_since_pps = gnss_now - (uint32_t)isr_snap_gnss;
  const uint64_t vclock_ns_into_second = (uint64_t)gnss_raw_since_pps * 100ULL;
  const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;
  const double   position = (double)vclock_ns_into_second / 1000000000.0;

  // ── Path A: PPS anchor interpolation (current production path) ──
  const uint32_t dwt_elapsed_a = dwt_now - frag_dwt_cyccnt_at_pps;
  const uint64_t dwt_ns_into_second_a =
    (uint64_t)dwt_elapsed_a * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;
  const uint64_t interp_a = frag_gnss_ns + dwt_ns_into_second_a;
  const int64_t  error_a  = (int64_t)interp_a - (int64_t)vclock_gnss_ns;

  proof_welford(
    proof_a_n, proof_a_mean, proof_a_m2,
    proof_a_min, proof_a_max, proof_a_outside_100,
    proof_a_pos_mean, proof_a_pos_m2, proof_a_covar,
    error_a, position
  );

  // ── Path B: Spin anchor interpolation ──
  //
  // Use the spin capture's TDC-corrected DWT and GNSS ns as the anchor.
  // Interpolate forward from there using the same rate.
  //
  // The spin landed ~50 µs before the PPS edge.  If the query arrives
  // at position 0.6 in the second, the PPS anchor is 0.6s away but
  // the spin anchor is 0.6 + 0.00005 = 0.60005s away.  So the spin
  // anchor's interpolation window is slightly LONGER than the PPS
  // anchor's — the advantage isn't window size (yet), it's anchor
  // precision.
  //
  // When 10 KHz anchoring is added, the most recent anchor will be
  // at most 100 µs old, and Path B's window shrinks to ~50 µs.

  int64_t error_b  = 0;

  if (pps_spin.valid && pps_spin.tdc_correction >= 0) {
    // spin_gnss is approximate: landed_gnss_ns is the time.h estimate
    // at spin landing, adjusted forward by the TDC correction.
    // A simpler and more robust approach: the spin landing is
    // (delta_cycles) DWT cycles before the ISR snapshot.
    // The corrected_dwt is the true PPS-edge DWT.
    // So the corrected anchor is: {frag_gnss_ns, corrected_dwt}
    // — same GNSS time as the PPS edge, but a more precise DWT.

    const uint32_t dwt_elapsed_b = dwt_now - pps_spin.corrected_dwt;
    const uint64_t dwt_ns_into_second_b =
      (uint64_t)dwt_elapsed_b * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;
    const uint64_t interp_b = frag_gnss_ns + dwt_ns_into_second_b;
    error_b = (int64_t)interp_b - (int64_t)vclock_gnss_ns;

    proof_welford(
      proof_b_n, proof_b_mean, proof_b_m2,
      proof_b_min, proof_b_max, proof_b_outside_100,
      proof_b_pos_mean, proof_b_pos_m2, proof_b_covar,
      error_b, position
    );
  }

  // ── Emit results ──

  const double a_stddev = (proof_a_n >= 2) ? sqrt(proof_a_m2 / (double)(proof_a_n - 1)) : 0.0;
  const double a_pos_sd = (proof_a_n >= 2) ? sqrt(proof_a_pos_m2 / (double)(proof_a_n - 1)) : 0.0;
  const double a_corr   = (proof_a_n >= 2 && a_stddev > 0.0 && a_pos_sd > 0.0)
    ? (proof_a_covar / (double)(proof_a_n - 1)) / (a_stddev * a_pos_sd) : 0.0;

  // Path A results
  p.add("a_n",       (int32_t)proof_a_n);
  p.add("a_err",     error_a);
  p.add("a_mean",    proof_a_mean);
  p.add("a_stddev",  a_stddev);
  p.add("a_stderr",  (proof_a_n >= 2) ? a_stddev / sqrt((double)proof_a_n) : 0.0);
  p.add("a_min",     proof_a_min);
  p.add("a_max",     proof_a_max);
  p.add("a_out100",  proof_a_outside_100);
  p.add("a_corr",    a_corr);

  // Path B results
  if (proof_b_n > 0) {
    const double b_stddev = (proof_b_n >= 2) ? sqrt(proof_b_m2 / (double)(proof_b_n - 1)) : 0.0;
    const double b_pos_sd = (proof_b_n >= 2) ? sqrt(proof_b_pos_m2 / (double)(proof_b_n - 1)) : 0.0;
    const double b_corr   = (proof_b_n >= 2 && b_stddev > 0.0 && b_pos_sd > 0.0)
      ? (proof_b_covar / (double)(proof_b_n - 1)) / (b_stddev * b_pos_sd) : 0.0;

    p.add("b_n",       (int32_t)proof_b_n);
    p.add("b_err",     error_b);
    p.add("b_mean",    b_stddev > 0 ? proof_b_mean : 0.0);
    p.add("b_stddev",  b_stddev);
    p.add("b_stderr",  (proof_b_n >= 2) ? b_stddev / sqrt((double)proof_b_n) : 0.0);
    p.add("b_min",     proof_b_min);
    p.add("b_max",     proof_b_max);
    p.add("b_out100",  proof_b_outside_100);
    p.add("b_corr",    b_corr);
  } else {
    p.add("b_n", (int32_t)0);
  }

  // Common context
  p.add("position",         position);
  p.add("campaign_seconds", campaign_seconds);
  p.add("pps_count",        frag->pps_count);

  return p;
}

static Payload cmd_set_dac(const Payload& args) {
  double dac_val;
  if (args.tryGetDouble("set_dac1", dac_val))  ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val))  ocxo_dac_set(ocxo2_dac, dac_val);

  Payload p;
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  return p;
}

static Payload cmd_dac_test(const Payload& args) {
  uint8_t addr = AD5693R_ADDR_OCXO1;
  const char* target = args.getString("target");
  if (target && strcmp(target, "2") == 0) addr = AD5693R_ADDR_OCXO2;

  Payload p;
  p.add("addr", (int32_t)addr);

  // Optional control register write
  const char* ctrl_str = args.getString("ctrl");
  if (ctrl_str) {
    uint16_t ctrl = (uint16_t)strtoul(ctrl_str, nullptr, 0);
    bool ctrl_ok = ad5693r_write_ctrl(addr, ctrl);
    p.add("ctrl", (int32_t)ctrl);
    p.add("ctrl_ack", ctrl_ok);
  }

  // Optional DAC value write
  double dac_val;
  if (args.tryGetDouble("value", dac_val)) {
    uint16_t val = (uint16_t)dac_val;
    bool val_ok = ad5693r_write(addr, val);
    p.add("value", (int32_t)val);
    p.add("value_ack", val_ok);
  }

  return p;
}

static Payload cmd_dac_probe(const Payload& args) {
  uint8_t addr = AD5693R_ADDR_OCXO1;
  const char* target = args.getString("target");
  if (target && strcmp(target, "2") == 0) addr = AD5693R_ADDR_OCXO2;

  Payload p;
  p.add("addr", (int32_t)addr);

  const char* ctrl_str = args.getString("ctrl");
  if (ctrl_str) {
    uint16_t ctrl = (uint16_t)strtoul(ctrl_str, nullptr, 0);
    bool ctrl_ok = ad5693r_write_ctrl(addr, ctrl);
    p.add("ctrl", (int32_t)ctrl);
    p.add("ctrl_ack", ctrl_ok);
  }

  double dac_val;
  if (args.tryGetDouble("value", dac_val)) {
    uint16_t val = (uint16_t)dac_val;
    bool val_ok = ad5693r_write(addr, val);
    p.add("value", (int32_t)val);
    p.add("value_ack", val_ok);
  }

  uint16_t readback = 0;
  bool rb_ok = ad5693r_read_input_register(addr, readback);
  p.add("readback_ack", rb_ok);
  if (rb_ok) {
    p.add("readback", (int32_t)readback);
  }

  return p;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",        cmd_start        },
  { "STOP",         cmd_stop         },
  { "RECOVER",      cmd_recover      },
  { "REPORT",       cmd_report       },
  { "CLOCKS_INFO",  cmd_clocks_info  },
  { "WATCHDOG_TEST", cmd_watchdog_test },
  { "INTERP_TEST",  cmd_interp_test  },
  { "INTERP_PROOF", cmd_interp_proof },
  { "SET_DAC",      cmd_set_dac      },
  { "DAC_TEST",     cmd_dac_test     },
  { "DAC_PROBE",     cmd_dac_probe     },
  { nullptr,        nullptr          }
};

static const process_subscription_entry_t CLOCKS_SUBSCRIPTIONS[] = {
  { "TIMEBASE_FRAGMENT", on_timebase_fragment },
  { nullptr, nullptr },
};

const process_subscription_entry_t* timebase_subscriptions(void) {
  return CLOCKS_SUBSCRIPTIONS;
}

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = CLOCKS_SUBSCRIPTIONS
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}