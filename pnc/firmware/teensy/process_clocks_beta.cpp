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
#include "isr_dwt_compensate.h"
#include "tdc_correction.h"

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
  prev_dwt_at_pps   = pps_dwt_at_edge(isr_snap_dwt);

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
    prev_dwt_at_pps   = pps_dwt_at_edge(isr_snap_dwt);

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

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    return;
  }

  if (!isr_residual_valid) {
    isr_residual_valid = true;
  }

  pps_fired = false;

  // ── DWT ──
  uint32_t dwt_raw_at_pps = pps_dwt_at_edge(isr_snap_dwt);
  uint32_t dwt_delta = dwt_raw_at_pps - prev_dwt_at_pps;

  dwt_cycles_64    += dwt_delta;
  prev_dwt_at_pps   = dwt_raw_at_pps;

  const uint64_t pps_dwt_cycles = dwt_cycles_64;
  const uint64_t pps_dwt_ns     = dwt_cycles_to_ns(pps_dwt_cycles);

  // ── OCXO1 ──
  uint32_t ocxo1_delta = isr_snap_ocxo1 - prev_ocxo1_at_pps;
  ocxo1_ticks_64    += ocxo1_delta;
  prev_ocxo1_at_pps  = isr_snap_ocxo1;

  const uint64_t pps_ocxo1_ticks = ocxo1_ticks_64;
  const uint64_t pps_ocxo1_ns    = pps_ocxo1_ticks * 100ull;

  // ── OCXO2 ──
  uint32_t ocxo2_delta = isr_snap_ocxo2 - prev_ocxo2_at_pps;
  ocxo2_ticks_64    += ocxo2_delta;
  prev_ocxo2_at_pps  = isr_snap_ocxo2;

  const uint64_t pps_ocxo2_ticks = ocxo2_ticks_64;
  const uint64_t pps_ocxo2_ns    = pps_ocxo2_ticks * 100ull;

  // ── GNSS ──
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

  // Snapshot DAC state before servo
  const uint16_t dac1_before = ocxo1_dac.dac_hw_code;
  const uint16_t dac2_before = ocxo2_dac.dac_hw_code;

  // ── Absolute GNSS timestamps for canonical winning OCXO edges ──
  ocxo_phase.pps_gnss_ns = pps_gnss_ns;

  const uint32_t ocxo1_edge_dwt = ocxo_phase.ocxo1_edge_dwt;
  const uint32_t ocxo2_edge_dwt = ocxo_phase.ocxo2_edge_dwt;

  const bool ocxo1_edge_valid = ocxo_phase.ocxo1_captured &&
    ocxo_edge_dwt_to_gnss_ns(
      ocxo1_edge_dwt,
      pps_gnss_ns,
      dwt_raw_at_pps,
      dwt_cycles_per_pps_snapshot,
      ocxo_phase.ocxo1_edge_gnss_ns
    );

  const bool ocxo2_edge_valid = ocxo_phase.ocxo2_captured &&
    ocxo_edge_dwt_to_gnss_ns(
      ocxo2_edge_dwt,
      pps_gnss_ns,
      dwt_raw_at_pps,
      dwt_cycles_per_pps_snapshot,
      ocxo_phase.ocxo2_edge_gnss_ns
    );

  ocxo_phase.ocxo1_valid = ocxo1_edge_valid;
  ocxo_phase.ocxo2_valid = ocxo2_edge_valid;
  ocxo_phase.detector_valid = ocxo1_edge_valid && ocxo2_edge_valid;

  if (ocxo_phase.detector_valid) {
    const int64_t phase1 =
      (int64_t)ocxo_phase.ocxo1_edge_gnss_ns - (int64_t)pps_gnss_ns;

    const int64_t phase2 =
      (int64_t)ocxo_phase.ocxo2_edge_gnss_ns - (int64_t)pps_gnss_ns;

    // Normalize to [0, 100 ns)
    ocxo_phase.ocxo1_phase_offset_ns =
      (int32_t)((phase1 % 100 + 100) % 100);

    ocxo_phase.ocxo2_phase_offset_ns =
      (int32_t)((phase2 % 100 + 100) % 100);

    if (ocxo_phase.prev_ocxo1_edge_gnss_ns != 0 &&
        ocxo_phase.prev_ocxo2_edge_gnss_ns != 0) {
      ocxo_phase.ocxo1_gnss_ns_per_pps =
        (int64_t)(ocxo_phase.ocxo1_edge_gnss_ns - ocxo_phase.prev_ocxo1_edge_gnss_ns);
      ocxo_phase.ocxo2_gnss_ns_per_pps =
        (int64_t)(ocxo_phase.ocxo2_edge_gnss_ns - ocxo_phase.prev_ocxo2_edge_gnss_ns);

      ocxo_phase.ocxo1_residual_ns =
        ocxo_phase.ocxo1_gnss_ns_per_pps - (int64_t)NS_PER_SECOND;
      ocxo_phase.ocxo2_residual_ns =
        ocxo_phase.ocxo2_gnss_ns_per_pps - (int64_t)NS_PER_SECOND;
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

  } else {
    ocxo_phase.ocxo1_edge_gnss_ns = 0;
    ocxo_phase.ocxo2_edge_gnss_ns = 0;
    ocxo_phase.ocxo1_phase_offset_ns = 0;
    ocxo_phase.ocxo2_phase_offset_ns = 0;
    ocxo_phase.ocxo1_gnss_ns_per_pps = 0;
    ocxo_phase.ocxo2_gnss_ns_per_pps = 0;
    ocxo_phase.ocxo1_residual_ns = 0;
    ocxo_phase.ocxo2_residual_ns = 0;
    ocxo_phase.residual_valid = false;
  }

  ocxo_calibration_servo();

  // Snapshot DAC state after servo
  ocxo_phase.ocxo1_dac_before = dac1_before;
  ocxo_phase.ocxo1_dac_after  = ocxo1_dac.dac_hw_code;
  ocxo_phase.ocxo2_dac_before = dac2_before;
  ocxo_phase.ocxo2_dac_after  = ocxo2_dac.dac_hw_code;

  // ── DAC Welford ──
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

  p.add("phase_detector_valid",   (bool)ocxo_phase.detector_valid);

  // ─────────────────────────────────────────────
  // Phase — canonical record (NEW)
  // ─────────────────────────────────────────────

  // Absolute edge timestamps (GNSS domain)
  p.add("ocxo1_edge_gnss_ns", ocxo_phase.ocxo1_edge_gnss_ns);
  p.add("ocxo2_edge_gnss_ns", ocxo_phase.ocxo2_edge_gnss_ns);

  // Phase offsets (0–100 ns)
  p.add("ocxo1_phase_offset_ns", (int32_t)ocxo_phase.ocxo1_phase_offset_ns);
  p.add("ocxo2_phase_offset_ns", (int32_t)ocxo_phase.ocxo2_phase_offset_ns);

  // Per-second edge delta (ns)
  p.add("ocxo1_gnss_ns_per_pps", ocxo_phase.ocxo1_gnss_ns_per_pps);
  p.add("ocxo2_gnss_ns_per_pps", ocxo_phase.ocxo2_gnss_ns_per_pps);

  // Residuals (servo input)
  p.add("ocxo1_residual_ns", ocxo_phase.ocxo1_residual_ns);
  p.add("ocxo2_residual_ns", ocxo_phase.ocxo2_residual_ns);
  p.add("phase_residual_valid", (bool)ocxo_phase.residual_valid);

  p.add("ocxo1_dac_before",        (int32_t)ocxo_phase.ocxo1_dac_before);
  p.add("ocxo1_dac_after",         (int32_t)ocxo_phase.ocxo1_dac_after);
  p.add("ocxo2_dac_before",        (int32_t)ocxo_phase.ocxo2_dac_before);
  p.add("ocxo2_dac_after",         (int32_t)ocxo_phase.ocxo2_dac_after);

  // Shadow-write TDC forensics (v29)
  p.add("ocxo1_phase_isr_dwt",      ocxo_phase.ocxo1_isr_dwt);
  p.add("ocxo1_phase_shadow_dwt",   ocxo_phase.ocxo1_shadow_dwt);
  p.add("ocxo1_phase_edge_dwt",     ocxo_phase.ocxo1_edge_dwt);
  p.add("ocxo2_phase_isr_dwt",      ocxo_phase.ocxo2_isr_dwt);
  p.add("ocxo2_phase_shadow_dwt",   ocxo_phase.ocxo2_shadow_dwt);
  p.add("ocxo2_phase_edge_dwt",     ocxo_phase.ocxo2_edge_dwt);
  p.add("diag_ocxo_phase_spin_timeouts", diag_ocxo_phase_spin_timeouts);

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

  p.add("spin_valid",             pps_spin.valid);
  p.add("spin_approach_cycles",   pps_spin.approach_cycles);
  p.add("spin_error_cycles",      pps_spin.spin_error);
  p.add("spin_isr_dwt",           pps_spin.isr_dwt);
  p.add("spin_landed_dwt",        pps_spin.landed_dwt);
  p.add("spin_shadow_dwt",        pps_spin.shadow_dwt);
  p.add("spin_edge_dwt",          pps_spin.edge_dwt);
  p.add("spin_nano_timed_out",    pps_spin.nano_timed_out);
  p.add("spin_shadow_timed_out",  pps_spin.shadow_timed_out);

  p.add("dwt_cal_valid",     g_dwt_cal_valid);
  p.add("dwt_cal_pps_count", g_dwt_cal_pps_count);

  // ── TIME_TEST self-audit ──
  p.add("time_test_valid",            time_test.valid);
  p.add("time_test_residual_ns",      time_test.residual_ns);
  p.add("time_test_computed_gnss_ns", time_test.computed_gnss_ns);
  p.add("time_test_vclock_gnss_ns",   time_test.vclock_gnss_ns);
  p.add("time_test_isr_dwt",         time_test.isr_dwt);
  p.add("time_test_edge_dwt",        time_test.edge_dwt);
  p.add("time_test_vclock_at_fire",   time_test.vclock_at_fire);
  p.add("time_test_tests_run",       time_test.tests_run);
  p.add("time_test_tests_valid",     time_test.tests_valid);
  p.add("time_test_tests_time_invalid", time_test.tests_time_invalid);

  publish("TIMEBASE_FRAGMENT", p);
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

// ============================================================================
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  // ─────────────────────────────────────────────
  // Campaign state
  // ─────────────────────────────────────────────
  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  p.add("campaign", campaign_name);
  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign_seconds", campaign_seconds);
  }

  // ─────────────────────────────────────────────
  // Core clocks (absolute)
  // ─────────────────────────────────────────────
  const uint64_t dwt_ns   = clocks_dwt_ns_now();
  const uint64_t gnss_ns  = clocks_gnss_ns_now();
  const uint64_t ocxo1_ns = clocks_ocxo1_ns_now();
  const uint64_t ocxo2_ns = clocks_ocxo2_ns_now();

  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo1_ns_now",   ocxo1_ns);
  p.add("ocxo2_ns_now",   ocxo2_ns);
  p.add("gnss_lock",      digitalRead(GNSS_LOCK_PIN));

  // ─────────────────────────────────────────────
  // Phase detector — THE NEW TRUTH LAYER
  // ─────────────────────────────────────────────
  p.add("phase_detector_valid", (bool)ocxo_phase.detector_valid);

  // Absolute edge timestamps (GNSS domain)
  p.add("ocxo1_edge_gnss_ns", ocxo_phase.ocxo1_edge_gnss_ns);
  p.add("ocxo2_edge_gnss_ns", ocxo_phase.ocxo2_edge_gnss_ns);

  // Phase offsets (ns within 100 ns window)
  p.add("ocxo1_phase_offset_ns", (int32_t)ocxo_phase.ocxo1_phase_offset_ns);
  p.add("ocxo2_phase_offset_ns", (int32_t)ocxo_phase.ocxo2_phase_offset_ns);

  // Per-second edge deltas
  p.add("ocxo1_gnss_ns_per_pps", ocxo_phase.ocxo1_gnss_ns_per_pps);
  p.add("ocxo2_gnss_ns_per_pps", ocxo_phase.ocxo2_gnss_ns_per_pps);

  // Residuals — THIS IS THE SERVO INPUT
  p.add("ocxo1_residual_ns", ocxo_phase.ocxo1_residual_ns);
  p.add("ocxo2_residual_ns", ocxo_phase.ocxo2_residual_ns);
  p.add("phase_residual_valid", (bool)ocxo_phase.residual_valid);

  // Capture diagnostics (minimal)
  p.add("ocxo1_phase_captured", (bool)ocxo_phase.ocxo1_captured);
  p.add("ocxo2_phase_captured", (bool)ocxo_phase.ocxo2_captured);
  p.add("diag_ocxo1_phase_captures", diag_ocxo1_phase_captures);
  p.add("diag_ocxo1_phase_misses",   diag_ocxo1_phase_misses);
  p.add("diag_ocxo2_phase_captures", diag_ocxo2_phase_captures);
  p.add("diag_ocxo2_phase_misses",   diag_ocxo2_phase_misses);

  // ─────────────────────────────────────────────
  // PPS residuals (legacy but useful)
  // ─────────────────────────────────────────────
  report_residual(p, "dwt",   residual_dwt);
  report_residual(p, "gnss",  residual_gnss);
  report_residual(p, "ocxo1", residual_ocxo1);
  report_residual(p, "ocxo2", residual_ocxo2);

  // ─────────────────────────────────────────────
  // Prediction (sanity / interpolation health)
  // ─────────────────────────────────────────────
  report_prediction(p, "dwt",   pred_dwt);
  report_prediction(p, "ocxo1", pred_ocxo1);
  report_prediction(p, "ocxo2", pred_ocxo2);

  // ─────────────────────────────────────────────
  // DAC / servo state
  // ─────────────────────────────────────────────
  p.add("ocxo1_dac",    ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac",    ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_hw", (int32_t)ocxo1_dac.dac_hw_code);
  p.add("ocxo2_dac_hw", (int32_t)ocxo2_dac.dac_hw_code);

  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);
  p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);
  p.add("ocxo1_servo_last_residual", ocxo1_dac.servo_last_residual);
  p.add("ocxo2_servo_last_residual", ocxo2_dac.servo_last_residual);
  p.add("ocxo1_servo_last_step", ocxo1_dac.servo_last_step);
  p.add("ocxo2_servo_last_step", ocxo2_dac.servo_last_step);

  // ─────────────────────────────────────────────
  // Timebase (for cross-domain sanity)
  // ─────────────────────────────────────────────
  p.add("timebase_valid", timebase_valid());
  p.add("timebase_gnss_ns",  timebase_now_ns(timebase_domain_t::GNSS));
  p.add("timebase_dwt_ns",   timebase_now_ns(timebase_domain_t::DWT));
  p.add("timebase_ocxo1_ns", timebase_now_ns(timebase_domain_t::OCXO1));
  p.add("timebase_ocxo2_ns", timebase_now_ns(timebase_domain_t::OCXO2));

  // ─────────────────────────────────────────────
  // Calibration
  // ─────────────────────────────────────────────
  p.add("dwt_cal_valid",     g_dwt_cal_valid);
  p.add("dwt_cal_cycles_per_s", g_dwt_cycles_per_gnss_s);
  p.add("dwt_cal_pps_count", g_dwt_cal_pps_count);

  // ─────────────────────────────────────────────
  // Watchdog
  // ─────────────────────────────────────────────
  p.add("watchdog_anomaly_active",  (bool)watchdog_anomaly_active);
  p.add("watchdog_anomaly_pending", (bool)watchdog_anomaly_publish_pending);
  p.add("watchdog_anomaly_sequence", watchdog_anomaly_sequence);
  p.add("watchdog_anomaly_reason", watchdog_anomaly_reason);

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

  p.add("pps_rejected_total",      diag_pps_rejected_total);
  p.add("pps_rejected_remainder",  diag_pps_rejected_remainder);
  p.add("pps_reject_consecutive",  diag_pps_reject_consecutive);
  p.add("pps_reject_recoveries",   diag_pps_reject_recoveries);
  p.add("pps_reject_max_run",      diag_pps_reject_max_run);

  p.add("pps_scheduled_stuck",     diag_pps_scheduled_stuck);
  p.add("pps_watchdog_recoveries", diag_pps_watchdog_recoveries);
  p.add("pps_asap_arm_failures",   diag_pps_asap_arm_failures);
  p.add("pps_asap_armed",          diag_pps_asap_armed);
  p.add("pps_asap_dispatched",     diag_pps_asap_dispatched);
  p.add("pps_stuck_since_dwt",     diag_pps_stuck_since_dwt);
  p.add("pps_stuck_max",           diag_pps_stuck_max);
  p.add("pps_scheduled",           (bool)pps_scheduled);

  p.add("spin_valid",             pps_spin.valid);
  p.add("spin_armed",             pps_spin.armed);
  p.add("spin_handle",            (uint32_t)pps_spin.handle);
  p.add("spin_completed",         pps_spin.completed);
  p.add("spin_target_dwt",        pps_spin.target_dwt);
  p.add("spin_landed_dwt",        pps_spin.landed_dwt);
  p.add("spin_landed_gnss_ns",    pps_spin.landed_gnss_ns);
  p.add("spin_error_cycles",      pps_spin.spin_error);
  p.add("spin_shadow_dwt",        pps_spin.shadow_dwt);
  p.add("spin_isr_dwt",           pps_spin.isr_dwt);
  p.add("spin_approach_cycles",   pps_spin.approach_cycles);
  p.add("spin_edge_dwt",          pps_spin.edge_dwt);
  p.add("spin_nano_timed_out",    pps_spin.nano_timed_out);
  p.add("spin_nano_timeouts",     pps_spin.nano_timeouts);
  p.add("spin_shadow_timed_out",  pps_spin.shadow_timed_out);
  p.add("spin_shadow_timeouts",   pps_spin.shadow_timeouts);
  p.add("spin_arms",              pps_spin.arms);
  p.add("spin_arm_failures",      pps_spin.arm_failures);
  p.add("spin_completions",       pps_spin.completions);
  p.add("spin_misses",            pps_spin.misses);

  p.add("diag_ocxo_phase_spin_timeouts", diag_ocxo_phase_spin_timeouts);

  p.add("dbg_post_dwt",      dbg_post_loop_dwt);
  p.add("dbg_post_shadow",   dbg_post_loop_shadow);
  p.add("dbg_post_isr_cap",  dbg_post_loop_isr_cap);
  p.add("dbg_post_isr_snap", dbg_post_loop_isr_snap);

  p.add("qread_total",            diag_qread_total);
  p.add("qread_same_hi",          diag_qread_same_hi);
  p.add("qread_retry_hi_changed", diag_qread_retry_hi_changed);
  p.add("qread_last_hi1",         diag_qread_last_hi1);
  p.add("qread_last_hi2",         diag_qread_last_hi2);
  p.add("qread_last_lo",          diag_qread_last_lo);
  p.add("qread_last_lo2",         diag_qread_last_lo2);

  return p;
}

static Payload cmd_phase_info(const Payload&) {
  Payload p;

  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  p.add("campaign", campaign_name);
  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign_seconds", campaign_seconds);
  }

  p.add("phase_detector_valid", (bool)ocxo_phase.detector_valid);
  p.add("dwt_at_pps",          ocxo_phase.dwt_at_pps);

  p.add("diag_gpt1_isr_fires", diag_gpt1_isr_fires);
  p.add("diag_gpt2_isr_fires", diag_gpt2_isr_fires);

  p.add("ocxo1_captured",      (bool)ocxo_phase.ocxo1_captured);
  p.add("ocxo1_valid",         (bool)ocxo_phase.ocxo1_valid);
  p.add("ocxo1_isr_dwt",       ocxo_phase.ocxo1_isr_dwt);
  p.add("ocxo1_gpt_at_fire",   ocxo_phase.ocxo1_gpt_at_fire);
  p.add("ocxo1_shadow_dwt",    ocxo_phase.ocxo1_shadow_dwt);
  p.add("ocxo1_edge_dwt",      ocxo_phase.ocxo1_edge_dwt);
  p.add("ocxo1_dwt_elapsed",   ocxo_phase.ocxo1_dwt_elapsed);
  p.add("ocxo1_elapsed_ns",    ocxo_phase.ocxo1_elapsed_ns);
  p.add("ocxo1_phase_offset_ns", (int32_t)ocxo_phase.ocxo1_phase_offset_ns);
  p.add("ocxo1_edge_gnss_ns",  ocxo_phase.ocxo1_edge_gnss_ns);

  p.add("ocxo2_captured",      (bool)ocxo_phase.ocxo2_captured);
  p.add("ocxo2_valid",         (bool)ocxo_phase.ocxo2_valid);
  p.add("ocxo2_isr_dwt",       ocxo_phase.ocxo2_isr_dwt);
  p.add("ocxo2_gpt_at_fire",   ocxo_phase.ocxo2_gpt_at_fire);
  p.add("ocxo2_shadow_dwt",    ocxo_phase.ocxo2_shadow_dwt);
  p.add("ocxo2_edge_dwt",      ocxo_phase.ocxo2_edge_dwt);
  p.add("ocxo2_dwt_elapsed",   ocxo_phase.ocxo2_dwt_elapsed);
  p.add("ocxo2_elapsed_ns",    ocxo_phase.ocxo2_elapsed_ns);
  p.add("ocxo2_phase_offset_ns", (int32_t)ocxo_phase.ocxo2_phase_offset_ns);
  p.add("ocxo2_edge_gnss_ns",  ocxo_phase.ocxo2_edge_gnss_ns);

  p.add("ocxo1_residual_ns",   ocxo_phase.ocxo1_residual_ns);
  p.add("ocxo2_residual_ns",   ocxo_phase.ocxo2_residual_ns);
  p.add("residual_valid",      (bool)ocxo_phase.residual_valid);

  p.add("captures_ocxo1",      diag_ocxo1_phase_captures);
  p.add("misses_ocxo1",        diag_ocxo1_phase_misses);
  p.add("captures_ocxo2",      diag_ocxo2_phase_captures);
  p.add("misses_ocxo2",        diag_ocxo2_phase_misses);
  p.add("diag_ocxo_phase_spin_timeouts", diag_ocxo_phase_spin_timeouts);

  return p;
}

static Payload cmd_time_test(const Payload&) {
  Payload p;
  p.add("valid",              time_test.valid);
  p.add("residual_ns",        time_test.residual_ns);
  p.add("computed_gnss_ns",   time_test.computed_gnss_ns);
  p.add("vclock_gnss_ns",     time_test.vclock_gnss_ns);
  p.add("isr_dwt",           time_test.isr_dwt);
  p.add("edge_dwt",          time_test.edge_dwt);
  p.add("vclock_at_fire",     time_test.vclock_at_fire);
  p.add("captured",           time_test.captured);
  p.add("tests_run",         time_test.tests_run);
  p.add("tests_valid",       time_test.tests_valid);
  p.add("tests_time_invalid", time_test.tests_time_invalid);
  p.add("ch3_isr_fires",     time_test.ch3_isr_fires);
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

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",        cmd_start        },
  { "STOP",         cmd_stop         },
  { "RECOVER",      cmd_recover      },
  { "REPORT",       cmd_report       },
  { "CLOCKS_INFO",  cmd_clocks_info  },
  { "PHASE_INFO",   cmd_phase_info   },
  { "WATCHDOG_TEST",cmd_watchdog_test },
  { "TIME_TEST",    cmd_time_test    },
  { "SET_DAC",      cmd_set_dac      },
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
