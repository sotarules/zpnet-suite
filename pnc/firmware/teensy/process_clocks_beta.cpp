// ============================================================================
// process_clocks_beta.cpp — Campaign Layer
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_interrupt.h"

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
volatile bool request_zero    = false;

uint64_t recover_dwt_ns   = 0;
uint64_t recover_gnss_ns  = 0;
uint64_t recover_ocxo1_ns = 0;
uint64_t recover_ocxo2_ns = 0;

static volatile bool zero_handshake_in_flight = false;

// Most recently published TIMEBASE_FRAGMENT, retained for cmd_report.
static Payload g_last_fragment;

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
// Residual tracking
// ============================================================================

pps_residual_t residual_dwt   = {};
pps_residual_t residual_vclock = {};
pps_residual_t residual_ocxo1 = {};
pps_residual_t residual_ocxo2 = {};

void residual_reset(pps_residual_t& r) {
  r.residual = 0;
  r.n = 0;
  r.mean = 0.0;
  r.m2 = 0.0;
  r.min_val = 1e30;
  r.max_val = -1e30;
}

void residual_update_sample(pps_residual_t& r, int64_t residual) {
  r.residual = residual;
  r.n++;
  const double x = (double)residual;
  const double d1 = x - r.mean;
  r.mean += d1 / (double)r.n;
  const double d2 = x - r.mean;
  r.m2 += d1 * d2;
  if (x < r.min_val) r.min_val = x;
  if (x > r.max_val) r.max_val = x;
}

double residual_stddev(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) : 0.0;
}

double residual_stderr(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) / sqrt((double)r.n) : 0.0;
}

// ============================================================================
// DAC Welford tracking
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


static void clocks_payload_add_interrupt_bridge_diag(Payload& p,
                                                     const char* prefix,
                                                     const interrupt_capture_diag_t& diag) {
  char key[96];

  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  auto add_i64 = [&](const char* suffix, int64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  // Canonical reconstructed event facts and bridge/reconciliation diagnostics.
  add_u32("dwt_at_event", diag.dwt_at_event);
  add_u64("gnss_ns_at_event_raw", diag.gnss_ns_at_event_raw);
  add_u64("gnss_ns_at_event_final", diag.gnss_ns_at_event_final);
  add_i64("gnss_ns_at_event_delta", diag.gnss_ns_at_event_delta);

  add_bool("bridge_valid", diag.bridge_valid);
  add_bool("bridge_within_tolerance", diag.bridge_within_tolerance);
  add_bool("bridge_skipped_invalid", diag.bridge_skipped_invalid);
  add_bool("bridge_used_prediction", diag.bridge_used_prediction);

  add_u64("bridge_gnss_ns_raw", diag.bridge_gnss_ns_raw);
  add_u64("bridge_gnss_ns_target", diag.bridge_gnss_ns_target);
  add_u64("bridge_gnss_ns_final", diag.bridge_gnss_ns_final);
  add_i64("bridge_raw_error_ns", diag.bridge_raw_error_ns);
  add_i64("bridge_final_error_ns", diag.bridge_final_error_ns);
}

// ── Predictive servo tuning ──
//
// Both TOTAL and NOW drive a filtered residual *and* a filtered mean slope
// toward zero.  The controller projects a short horizon into the future and
// adjusts the DAC toward that projected residual rather than reacting only to
// the current residual.
//

static constexpr uint32_t NOW_WINDOW_SIZE = 5;

static constexpr double SERVO_NS_PER_DAC_LSB = 100.0;
static constexpr double SERVO_MIN_RESIDUAL_NS = 5.0;
static constexpr double SERVO_MIN_PREDICTED_NS = 2.0;

static constexpr double SERVO_TOTAL_RESIDUAL_ALPHA = 0.08;
static constexpr double SERVO_TOTAL_SLOPE_ALPHA    = 0.12;
static constexpr double SERVO_TOTAL_HORIZON_S      = 8.0;

static constexpr double SERVO_NOW_RESIDUAL_ALPHA = 0.25;
static constexpr double SERVO_NOW_SLOPE_ALPHA    = 0.35;
static constexpr double SERVO_NOW_HORIZON_S      = 3.0;

struct now_window_t {
  int64_t  samples[NOW_WINDOW_SIZE];
  uint32_t count;
  uint32_t head;
};

static now_window_t g_now_window_ocxo1 = {};
static now_window_t g_now_window_ocxo2 = {};

static void now_window_reset(now_window_t& w) {
  for (uint32_t i = 0; i < NOW_WINDOW_SIZE; i++) w.samples[i] = 0;
  w.count = 0;
  w.head = 0;
}


static void ocxo_servo_latch_dac_fault(ocxo_dac_state_t& dac) {
  dac.io_fault_latched = true;
  calibrate_ocxo_mode = servo_mode_t::OFF;
}

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void) {
  timebase_invalidate();

  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;

  residual_reset(residual_dwt);
  residual_reset(residual_vclock);
  residual_reset(residual_ocxo1);
  residual_reset(residual_ocxo2);

  dac_welford_reset(dac_welford_ocxo1);
  dac_welford_reset(dac_welford_ocxo2);

  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);

  now_window_reset(g_now_window_ocxo1);
  now_window_reset(g_now_window_ocxo2);
}

// ============================================================================
// Servo logic
// ============================================================================
//
// TOTAL: uses the campaign-wide residual mean as the residual signal.
// NOW:   uses the mean of the recent NOW window as the residual signal.
//
// Both modes use the same predictive controller:
//   1. low-pass filter the residual signal
//   2. low-pass filter the per-second slope of that residual
//   3. project a short horizon into the future
//   4. drive the projected residual toward zero
//
// Sign convention:
//   Positive residual = clock fast → lower DAC
//   Negative residual = clock slow → raise DAC
//

static void ocxo_servo_predictive(ocxo_dac_state_t& dac,
                                  double residual_signal_ns,
                                  double residual_alpha,
                                  double slope_alpha,
                                  double horizon_s) {
  dac.servo_last_residual = residual_signal_ns;

  if (!dac.servo_predictor_initialized) {
    dac.servo_predictor_initialized = true;
    dac.servo_last_raw_residual = residual_signal_ns;
    dac.servo_filtered_residual = residual_signal_ns;
    dac.servo_filtered_slope = 0.0;
    dac.servo_predicted_residual = residual_signal_ns;
    dac.servo_predictor_updates = 1;
    return;
  }

  const double raw_slope = residual_signal_ns - dac.servo_last_raw_residual;
  dac.servo_last_raw_residual = residual_signal_ns;

  dac.servo_filtered_residual =
      (1.0 - residual_alpha) * dac.servo_filtered_residual +
      residual_alpha * residual_signal_ns;

  dac.servo_filtered_slope =
      (1.0 - slope_alpha) * dac.servo_filtered_slope +
      slope_alpha * raw_slope;

  dac.servo_predicted_residual =
      dac.servo_filtered_residual + dac.servo_filtered_slope * horizon_s;

  dac.servo_predictor_updates++;

  if (fabs(dac.servo_filtered_residual) < SERVO_MIN_RESIDUAL_NS &&
      fabs(dac.servo_predicted_residual) < SERVO_MIN_PREDICTED_NS) {
    return;
  }

  double step = -dac.servo_predicted_residual / SERVO_NS_PER_DAC_LSB;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  if (fabs(step) < 1.0) {
    // Do not thrash the DAC for sub-LSB fractional intent.
    return;
  }

  // MULE dry run: do not touch the DAC bus
  dac.servo_last_step = step;
  return;

  if (!ocxo_dac_set(dac, dac.dac_fractional + step)) {
    dac.servo_last_step = 0.0;
    ocxo_servo_latch_dac_fault(dac);
    return;
  }

  dac.servo_last_step = step;
  dac.servo_adjustments++;
}

static double now_window_mean(const now_window_t& w) {
  if (w.count == 0) return 0.0;
  double sum = 0.0;
  for (uint32_t i = 0; i < w.count; i++) sum += (double)w.samples[i];
  return sum / (double)w.count;
}

static void now_window_push(now_window_t& w, int64_t sample) {
  w.samples[w.head] = sample;
  w.head = (w.head + 1) % NOW_WINDOW_SIZE;
  if (w.count < NOW_WINDOW_SIZE) w.count++;
}

static void ocxo_servo_total(ocxo_dac_state_t& dac, const pps_residual_t& residuals) {
  if (residuals.n < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  ocxo_servo_predictive(dac,
                        residuals.mean,
                        SERVO_TOTAL_RESIDUAL_ALPHA,
                        SERVO_TOTAL_SLOPE_ALPHA,
                        SERVO_TOTAL_HORIZON_S);

  dac.servo_settle_count = 0;
}

static void ocxo_servo_now(ocxo_dac_state_t& dac, const now_window_t& window) {
  if (window.count < NOW_WINDOW_SIZE) return;
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;

  ocxo_servo_predictive(dac,
                        now_window_mean(window),
                        SERVO_NOW_RESIDUAL_ALPHA,
                        SERVO_NOW_SLOPE_ALPHA,
                        SERVO_NOW_HORIZON_S);
}

static void ocxo_calibration_servo(void) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  if (calibrate_ocxo_mode == servo_mode_t::NOW) {
    ocxo_servo_now(ocxo1_dac, g_now_window_ocxo1);
    ocxo_servo_now(ocxo2_dac, g_now_window_ocxo2);
  } else {
    ocxo_servo_total(ocxo1_dac, residual_ocxo1);
    ocxo_servo_total(ocxo2_dac, residual_ocxo2);
  }
}

// ============================================================================
// Watchdog helpers
// ============================================================================

static void clocks_force_stop_campaign(void) {
  campaign_state = clocks_campaign_state_t::STOPPED;
  request_start = false;
  request_stop = false;
  request_recover = false;
  request_zero = false;
  zero_handshake_in_flight = false;
  calibrate_ocxo_mode = servo_mode_t::OFF;
  timebase_invalidate();
}

static void clocks_watchdog_anomaly_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
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
  p.add("detail2",          watchdog_anomaly_detail2);
  p.add("detail3",          watchdog_anomaly_detail3);

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

  timepop_handle_t h =
      timepop_arm_asap(clocks_watchdog_anomaly_callback, nullptr, "clocks-anomaly");

  if (h == TIMEPOP_INVALID_HANDLE) {
    watchdog_anomaly_publish_pending = false;
    clocks_force_stop_campaign();
  }
}

// ============================================================================
// clocks_beta_pps
// ============================================================================

void clocks_beta_pps(void) {
  if (request_zero || request_start) {
    if (!zero_handshake_in_flight) {
      interrupt_request_pps_zero();
      zero_handshake_in_flight = true;
      return;
    }

    if (interrupt_pps_zero_pending()) {
      return;
    }

    clocks_zero_all();

    zero_handshake_in_flight = false;
    request_zero = false;

    if (request_start) {
      watchdog_anomaly_active = false;
      campaign_state = clocks_campaign_state_t::STARTED;
      request_start = false;
    }

    return;
  }

  if (request_stop) {
    watchdog_anomaly_active = false;
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop = false;
    request_zero = false;
    zero_handshake_in_flight = false;
    calibrate_ocxo_mode = servo_mode_t::OFF;
    timebase_invalidate();
    return;
  }

  if (request_recover) {
    watchdog_anomaly_active = false;

    request_zero = false;
    zero_handshake_in_flight = false;

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_ticks_64        = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    request_recover = false;
    campaign_state = clocks_campaign_state_t::STARTED;
    return;
  }

  if (watchdog_anomaly_active) {
    return;
  }

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    return;
  }

  campaign_seconds++;

  dwt_cycle_count_total = g_dwt_cycle_count_total;
  gnss_raw_64           = g_gnss_ns_count_at_pps / 100ull;
  ocxo1_ticks_64        = g_ocxo1_clock.ns_count_at_pps / 100ull;
  ocxo2_ticks_64        = g_ocxo2_clock.ns_count_at_pps / 100ull;

  residual_update_sample(
      residual_dwt,
      (int64_t)dwt_effective_cycles_per_second() - (int64_t)DWT_EXPECTED_PER_PPS);

  if (g_vclock_measurement.prev_counter32_at_pps_event != 0) {
    residual_update_sample(residual_vclock, g_vclock_measurement.second_residual_ns);
  }

  if (g_ocxo1_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo1, g_ocxo1_measurement.second_residual_ns);
    now_window_push(g_now_window_ocxo1, g_ocxo1_measurement.second_residual_ns);
  }

  if (g_ocxo2_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo2, g_ocxo2_measurement.second_residual_ns);
    now_window_push(g_now_window_ocxo2, g_ocxo2_measurement.second_residual_ns);
  }

  ocxo_calibration_servo();

  dac_welford_update(dac_welford_ocxo1, ocxo1_dac.dac_fractional);
  dac_welford_update(dac_welford_ocxo2, ocxo2_dac.dac_fractional);

  Payload p;
  p.add("campaign", campaign_name);

  p.add("gnss_ns",  g_gnss_ns_count_at_pps);
  p.add("ocxo1_ns", g_ocxo1_clock.ns_count_at_pps);
  p.add("ocxo2_ns", g_ocxo2_clock.ns_count_at_pps);

  p.add("teensy_pps_count", campaign_seconds);

  p.add("dwt_cycle_count_at_pps", g_dwt_cycle_count_at_pps);
  p.add("dwt_cycle_count_total", g_dwt_cycle_count_total);
  p.add("dwt_cycle_count_between_pps", g_dwt_cycle_count_between_pps);
  p.add("dwt_cycle_count_next_second_prediction", g_dwt_cycle_count_next_second_prediction);
  p.add("dwt_cycle_count_next_second_adjustment", g_dwt_cycle_count_next_second_adjustment);
  p.add("dwt_model_pps_count", g_dwt_model_pps_count);
  p.add("qtimer_at_pps", g_qtimer_at_pps);
  p.add("pps_event_counter32_at_event", g_last_pps_event_counter32_at_event);

  p.add("vclock_counter32_at_pps_event", g_vclock_clock.counter32_at_pps_event);
  p.add("vclock_counter32_at_pps_expected", g_vclock_clock.counter32_at_pps_expected);
  p.add("vclock_counter32_error_at_pps", g_vclock_clock.counter32_error_at_pps);
  p.add("vclock_ticks_between_pps", g_vclock_measurement.ticks_between_pps);
  p.add("vclock_ns_between_pps", g_vclock_measurement.ns_between_pps);
  p.add("vclock_ns", g_vclock_clock.ns_count_at_pps);
  p.add("vclock_ns_expected", g_vclock_clock.ns_count_expected_at_pps);
  p.add("vclock_second_residual_ns", g_vclock_measurement.second_residual_ns);
  p.add("vclock_zero_established", g_vclock_clock.zero_established);

  p.add("ocxo1_gnss_ns_at_edge", g_ocxo1_clock.gnss_ns_at_edge);
  p.add("ocxo1_gnss_ns_between_edges", g_ocxo1_measurement.gnss_ns_between_edges);
  p.add("ocxo1_dwt_at_edge", g_ocxo1_measurement.dwt_at_edge);
  p.add("ocxo1_dwt_cycles_between_edges", g_ocxo1_measurement.dwt_cycles_between_edges);
  p.add("ocxo1_ns_count_at_edge", g_ocxo1_clock.ns_count_at_edge);
  p.add("ocxo1_ns_count_at_pps", g_ocxo1_clock.ns_count_at_pps);

  p.add("ocxo2_gnss_ns_at_edge", g_ocxo2_clock.gnss_ns_at_edge);
  p.add("ocxo2_gnss_ns_between_edges", g_ocxo2_measurement.gnss_ns_between_edges);
  p.add("ocxo2_dwt_at_edge", g_ocxo2_measurement.dwt_at_edge);
  p.add("ocxo2_dwt_cycles_between_edges", g_ocxo2_measurement.dwt_cycles_between_edges);
  p.add("ocxo2_ns_count_at_edge", g_ocxo2_clock.ns_count_at_edge);
  p.add("ocxo2_ns_count_at_pps", g_ocxo2_clock.ns_count_at_pps);

  p.add("ocxo1_second_residual_ns", g_ocxo1_measurement.second_residual_ns);
  p.add("ocxo2_second_residual_ns", g_ocxo2_measurement.second_residual_ns);

  // Smart-zero phase offset (cumulative, unwrapped)
  p.add("ocxo1_phase_offset_ns", g_ocxo1_clock.phase_offset_ns);
  p.add("ocxo1_zero_established", g_ocxo1_clock.zero_established);
  p.add("ocxo2_phase_offset_ns", g_ocxo2_clock.phase_offset_ns);
  p.add("ocxo2_zero_established", g_ocxo2_clock.zero_established);

  // Per-window viability (replaces cumulative viability)
  p.add("ocxo1_window_checks", g_ocxo1_clock.window_checks);
  p.add("ocxo1_window_mismatches", g_ocxo1_clock.window_mismatches);
  p.add("ocxo1_window_error_ns", g_ocxo1_clock.window_error_ns);

  p.add("ocxo2_window_checks", g_ocxo2_clock.window_checks);
  p.add("ocxo2_window_mismatches", g_ocxo2_clock.window_mismatches);
  p.add("ocxo2_window_error_ns", g_ocxo2_clock.window_error_ns);

  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo1_dac_fault_latched", ocxo1_dac.io_fault_latched);
  p.add("ocxo1_dac_write_attempts", ocxo1_dac.io_write_attempts);
  p.add("ocxo1_dac_write_successes", ocxo1_dac.io_write_successes);
  p.add("ocxo1_dac_write_failures", ocxo1_dac.io_write_failures);
  p.add("ocxo1_dac_last_attempted_hw_code", ocxo1_dac.io_last_attempted_hw_code);
  p.add("ocxo1_dac_last_good_hw_code", ocxo1_dac.io_last_good_hw_code);
  p.add("ocxo1_dac_last_failure_stage", (uint32_t)ocxo1_dac.io_last_failure_stage);

  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("ocxo2_dac_fault_latched", ocxo2_dac.io_fault_latched);
  p.add("ocxo2_dac_write_attempts", ocxo2_dac.io_write_attempts);
  p.add("ocxo2_dac_write_successes", ocxo2_dac.io_write_successes);
  p.add("ocxo2_dac_write_failures", ocxo2_dac.io_write_failures);
  p.add("ocxo2_dac_last_attempted_hw_code", ocxo2_dac.io_last_attempted_hw_code);
  p.add("ocxo2_dac_last_good_hw_code", ocxo2_dac.io_last_good_hw_code);
  p.add("ocxo2_dac_last_failure_stage", (uint32_t)ocxo2_dac.io_last_failure_stage);

  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  p.add("ocxo1_servo_filtered_residual_ns", ocxo1_dac.servo_filtered_residual, 6);
  p.add("ocxo1_servo_filtered_slope_ns_per_s", ocxo1_dac.servo_filtered_slope, 6);
  p.add("ocxo1_servo_predicted_residual_ns", ocxo1_dac.servo_predicted_residual, 6);
  p.add("ocxo1_servo_predictor_updates", ocxo1_dac.servo_predictor_updates);
  p.add("ocxo1_servo_last_step", ocxo1_dac.servo_last_step, 6);

  p.add("ocxo2_servo_filtered_residual_ns", ocxo2_dac.servo_filtered_residual, 6);
  p.add("ocxo2_servo_filtered_slope_ns_per_s", ocxo2_dac.servo_filtered_slope, 6);
  p.add("ocxo2_servo_predicted_residual_ns", ocxo2_dac.servo_predicted_residual, 6);
  p.add("ocxo2_servo_predictor_updates", ocxo2_dac.servo_predictor_updates);
  p.add("ocxo2_servo_last_step", ocxo2_dac.servo_last_step, 6);

  clocks_payload_add_interrupt_diag(p, "pps_diag", g_pps_interrupt_diag);
  clocks_payload_add_interrupt_bridge_diag(p, "pps_diag", g_pps_interrupt_diag);

  clocks_payload_add_interrupt_diag(p, "ocxo1_diag", g_ocxo1_interrupt_diag);
  clocks_payload_add_interrupt_bridge_diag(p, "ocxo1_diag", g_ocxo1_interrupt_diag);

  clocks_payload_add_interrupt_diag(p, "ocxo2_diag", g_ocxo2_interrupt_diag);
  clocks_payload_add_interrupt_bridge_diag(p, "ocxo2_diag", g_ocxo2_interrupt_diag);

  // Campaign-cumulative tau and ppb for all clock domains.
  //
  // Tau is the dimensionless rate ratio of each clock relative to GNSS.
  // A perfect clock has tau = 1.0 and ppb = 0.
  //
  // These are derived from the Welford mean of per-second residuals,
  // which measures rate directly without initial-phase-offset contamination.
  //
  // DWT residual is in cycles: ppb = mean_cycles / DWT_EXPECTED * 1e9
  // OCXO residual is in ns:   ppb = mean_ns  (since 1 ppb = 1 ns/s)

  if (residual_dwt.n > 0) {
    const double dwt_ppb_val = residual_dwt.mean / (double)DWT_EXPECTED_PER_PPS * 1e9;
    const double dwt_tau_val = 1.0 + dwt_ppb_val / 1e9;
    p.add("dwt_tau", dwt_tau_val, 12);
    p.add("dwt_ppb", dwt_ppb_val, 3);
  }

  if (residual_vclock.n > 0) {
    const double vclock_ppb_val = residual_vclock.mean;
    const double vclock_tau_val = 1.0 + vclock_ppb_val / 1e9;
    p.add("vclock_tau", vclock_tau_val, 12);
    p.add("vclock_ppb", vclock_ppb_val, 3);
  }

  if (residual_ocxo1.n > 0) {
    const double ocxo1_ppb_val = residual_ocxo1.mean;
    const double ocxo1_tau_val = 1.0 + ocxo1_ppb_val / 1e9;
    p.add("ocxo1_tau", ocxo1_tau_val, 12);
    p.add("ocxo1_ppb", ocxo1_ppb_val, 3);
  }

  if (residual_ocxo2.n > 0) {
    const double ocxo2_ppb_val = residual_ocxo2.mean;
    const double ocxo2_tau_val = 1.0 + ocxo2_ppb_val / 1e9;
    p.add("ocxo2_tau", ocxo2_tau_val, 12);
    p.add("ocxo2_ppb", ocxo2_ppb_val, 3);
  }

  // ── Campaign-cumulative Welford statistics ──
  //
  // Three residual Welfords (per-second clock error):
  //   dwt_welford_*    — DWT cycles residual (cycles above/below DWT_EXPECTED_PER_PPS)
  //   ocxo1_welford_*  — OCXO1 second residual (ns, = ppb)
  //   ocxo2_welford_*  — OCXO2 second residual (ns, = ppb)
  //
  // Two DAC Welfords (servo effort — the TEMPEST instrument):
  //   ocxo1_dac_welford_*  — OCXO1 DAC fractional value
  //   ocxo2_dac_welford_*  — OCXO2 DAC fractional value

  // DWT residual Welford (cycles)
  p.add("dwt_welford_n", residual_dwt.n);
  p.add("dwt_welford_mean", residual_dwt.mean, 3);
  p.add("dwt_welford_stddev", residual_stddev(residual_dwt), 3);
  p.add("dwt_welford_stderr", residual_stderr(residual_dwt), 3);
  p.add("dwt_welford_min", residual_dwt.min_val, 3);
  p.add("dwt_welford_max", residual_dwt.max_val, 3);

  // VCLOCK residual Welford (ns)
  p.add("vclock_welford_n", residual_vclock.n);
  p.add("vclock_welford_mean", residual_vclock.mean, 3);
  p.add("vclock_welford_stddev", residual_stddev(residual_vclock), 3);
  p.add("vclock_welford_stderr", residual_stderr(residual_vclock), 3);
  p.add("vclock_welford_min", residual_vclock.min_val, 3);
  p.add("vclock_welford_max", residual_vclock.max_val, 3);

  // OCXO1 residual Welford (ns)
  p.add("ocxo1_welford_n", residual_ocxo1.n);
  p.add("ocxo1_welford_mean", residual_ocxo1.mean, 3);
  p.add("ocxo1_welford_stddev", residual_stddev(residual_ocxo1), 3);
  p.add("ocxo1_welford_stderr", residual_stderr(residual_ocxo1), 3);
  p.add("ocxo1_welford_min", residual_ocxo1.min_val, 3);
  p.add("ocxo1_welford_max", residual_ocxo1.max_val, 3);

  // OCXO2 residual Welford (ns)
  p.add("ocxo2_welford_n", residual_ocxo2.n);
  p.add("ocxo2_welford_mean", residual_ocxo2.mean, 3);
  p.add("ocxo2_welford_stddev", residual_stddev(residual_ocxo2), 3);
  p.add("ocxo2_welford_stderr", residual_stderr(residual_ocxo2), 3);
  p.add("ocxo2_welford_min", residual_ocxo2.min_val, 3);
  p.add("ocxo2_welford_max", residual_ocxo2.max_val, 3);

  // OCXO1 DAC Welford
  p.add("ocxo1_dac_welford_n", dac_welford_ocxo1.n);
  p.add("ocxo1_dac_welford_mean", dac_welford_ocxo1.mean, 6);
  p.add("ocxo1_dac_welford_stddev", dac_welford_stddev(dac_welford_ocxo1), 6);
  p.add("ocxo1_dac_welford_stderr", dac_welford_stderr(dac_welford_ocxo1), 6);
  p.add("ocxo1_dac_welford_min", dac_welford_ocxo1.min_val, 6);
  p.add("ocxo1_dac_welford_max", dac_welford_ocxo1.max_val, 6);

  // OCXO2 DAC Welford
  p.add("ocxo2_dac_welford_n", dac_welford_ocxo2.n);
  p.add("ocxo2_dac_welford_mean", dac_welford_ocxo2.mean, 6);
  p.add("ocxo2_dac_welford_stddev", dac_welford_stddev(dac_welford_ocxo2), 6);
  p.add("ocxo2_dac_welford_stderr", dac_welford_stderr(dac_welford_ocxo2), 6);
  p.add("ocxo2_dac_welford_min", dac_welford_ocxo2.min_val, 6);
  p.add("ocxo2_dac_welford_max", dac_welford_ocxo2.max_val, 6);

  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  p.add("time_anchor_ok", snap.ok);
  p.add("time_anchor_valid", snap.valid);
  p.add("time_anchor_pps_count", snap.pps_count);
  p.add("time_anchor_qtimer_at_pps", snap.qtimer_at_pps);
  p.add("time_anchor_dwt_at_pps", snap.dwt_at_pps);
  p.add("time_anchor_dwt_cycles_per_s", snap.dwt_cycles_per_s);

  p.add("last_pps_live_qtimer_read", g_last_pps_live_qtimer_read);
  p.add("last_pps_live_qtimer_minus_geared", g_last_pps_live_qtimer_minus_geared);

  g_last_fragment = p;
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

  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);

  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (args.tryGetDouble("set_dac1", dac_val)) dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_mode = servo_mode_parse(args.getString("calibrate_ocxo"));
  if (!dac1_ok || !dac2_ok) {
    calibrate_ocxo_mode = servo_mode_t::OFF;
  }

  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);

  request_start = true;
  request_stop = false;
  request_recover = false;

  Payload p;
  p.add("status", (!dac1_ok || !dac2_ok) ? "start_requested_dac_fault" : "start_requested");
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));
  return p;
}

static Payload cmd_stop(const Payload&) {
  request_stop = true;
  request_start = false;
  request_zero = false;
  Payload p;
  p.add("status", "stop_requested");
  return p;
}

static Payload cmd_zero(const Payload&) {
  request_zero = true;
  request_start = false;
  request_stop = false;
  request_recover = false;

  Payload p;
  p.add("status", "zero_requested");
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

  recover_dwt_ns   = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns  = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo1_ns = strtoull(s_ocxo1,  nullptr, 10);
  recover_ocxo2_ns = strtoull(s_ocxo2,  nullptr, 10);

  request_recover = true;
  request_start   = false;
  request_stop    = false;
  request_zero    = false;

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

static Payload cmd_report(const Payload&) {
  // The CLOCKS report IS the most recent TIMEBASE_FRAGMENT,
  // augmented with live process state for debugging.

  Payload p = g_last_fragment.clone();

  p.add("campaign_state",
        campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  p.add("request_start", request_start);
  p.add("request_stop", request_stop);
  p.add("request_recover", request_recover);
  p.add("request_zero", request_zero);
  p.add("zero_handshake_in_flight", zero_handshake_in_flight);
  p.add("interrupt_pps_zero_pending", interrupt_pps_zero_pending());

  return p;
}

static Payload cmd_set_dac(const Payload& args) {
  double dac_val;
  bool dac1_ok = true;
  bool dac2_ok = true;
  if (args.tryGetDouble("set_dac1", dac_val)) dac1_ok = ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) dac2_ok = ocxo_dac_set(ocxo2_dac, dac_val);

  Payload p;
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("ocxo1_dac_last_write_ok", ocxo1_dac.io_last_write_ok);
  p.add("ocxo2_dac_last_write_ok", ocxo2_dac.io_last_write_ok);
  p.add("status", (dac1_ok && dac2_ok) ? "ok" : "dac_write_fault");
  return p;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",         cmd_start         },
  { "STOP",          cmd_stop          },
  { "ZERO",          cmd_zero          },
  { "RECOVER",       cmd_recover       },
  { "REPORT",        cmd_report        },
  { "WATCHDOG_TEST", cmd_watchdog_test },
  { "SET_DAC",       cmd_set_dac       },
  { nullptr,         nullptr           }
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