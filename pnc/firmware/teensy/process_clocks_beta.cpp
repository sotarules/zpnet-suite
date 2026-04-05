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

// Sacred PPS-boundary handshake state.
// True means clocks has asked interrupt to consummate PPS ZERO on a real PPS
// edge and is now waiting for interrupt_pps_zero_pending() to fall false.
static volatile bool zero_handshake_in_flight = false;

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
pps_residual_t residual_ocxo1 = {};
pps_residual_t residual_ocxo2 = {};

void residual_reset(pps_residual_t& r) {
  r.residual = 0;
  r.n = 0;
  r.mean = 0.0;
  r.m2 = 0.0;
}

void residual_update_sample(pps_residual_t& r, int64_t residual) {
  r.residual = residual;
  r.n++;
  const double x = (double)residual;
  const double d1 = x - r.mean;
  r.mean += d1 / (double)r.n;
  const double d2 = x - r.mean;
  r.m2 += d1 * d2;
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

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void) {
  timebase_invalidate();

  campaign_seconds = 0;

  dwt_cycles_64  = 0;
  gnss_raw_64    = 0;
  ocxo1_ticks_64 = 0;
  ocxo2_ticks_64 = 0;

  residual_reset(residual_dwt);
  residual_reset(residual_ocxo1);
  residual_reset(residual_ocxo2);

  dac_welford_reset(dac_welford_ocxo1);
  dac_welford_reset(dac_welford_ocxo2);

  g_gnss_ns_count_at_pps = 0;
  g_dwt_cycle_count_at_pps = DWT_CYCCNT;
  g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_model_pps_count = 0;

  g_ocxo1_clock = {};
  g_ocxo2_clock = {};
  g_ocxo1_measurement = {};
  g_ocxo2_measurement = {};

  g_pps_interrupt_diag = {};
  g_ocxo1_interrupt_diag = {};
  g_ocxo2_interrupt_diag = {};

  dwt_rolling_64 = 0;
  dwt_rolling_32 = DWT_CYCCNT;

  gnss_rolling_raw_64 = 0;
  gnss_rolling_32 = qtimer1_read_32();

  ocxo1_rolling_64 = 0;
  ocxo1_rolling_32 = interrupt_gpt1_counter_now();

  ocxo2_rolling_64 = 0;
  ocxo2_rolling_32 = interrupt_gpt2_counter_now();
}

// ============================================================================
// Servo logic
// ============================================================================

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

static void ocxo_servo_total(ocxo_dac_state_t& dac, uint64_t ocxo_ns, uint64_t gnss_ns) {
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  double total_error = (double)((int64_t)ocxo_ns - (int64_t)gnss_ns);
  dac.servo_last_residual = total_error;

  double rate_error = total_error / (double)campaign_seconds;

  if (fabs(rate_error) < 1e-6) return;

  double step = -rate_error * 0.05;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
  dac.servo_settle_count = 0;
}

static constexpr double NOW_NS_PER_DAC_LSB = 10.0;
static constexpr double NOW_MIN_RESIDUAL_NS = 5.0;

static void ocxo_servo_now(ocxo_dac_state_t& dac, int64_t residual_ns) {
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;
  if (residual_ns == 0) return;

  double residual = (double)residual_ns;
  dac.servo_last_residual = residual;

  if (fabs(residual) < NOW_MIN_RESIDUAL_NS) return;

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
    ocxo_servo_now(ocxo1_dac, g_ocxo1_measurement.second_residual_ns);
    ocxo_servo_now(ocxo2_dac, g_ocxo2_measurement.second_residual_ns);
  } else {
    ocxo_servo_total(ocxo1_dac, g_ocxo1_clock.ns_count_at_pps, g_gnss_ns_count_at_pps);
    ocxo_servo_total(ocxo2_dac, g_ocxo2_clock.ns_count_at_pps, g_gnss_ns_count_at_pps);
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

  timepop_handle_t h = timepop_arm(0, false, clocks_watchdog_anomaly_callback, nullptr, "clocks-anomaly");
  if (h == TIMEPOP_INVALID_HANDLE) {
    watchdog_anomaly_publish_pending = false;
    clocks_force_stop_campaign();
  }
}

// ============================================================================
// clocks_beta_pps
// ============================================================================
//
// Unified request contract:
//
//   • every request_foo is a latched intent
//   • consummated only by the lawful boundary handler
//   • cleared only after the transition has actually occurred
//
// request_zero:
//   sacred PPS-boundary contract via interrupt_request_pps_zero()
//
// request_start:
//   waits for the same sacred zero/reset contract to complete before campaign
//   actually becomes STARTED
//
// request_stop:
//   consummated on this lawful PPS callback turn
//
// request_recover:
//   consummated on this lawful PPS callback turn
//
// Important with the new interrupt PPS model:
//   • interrupt_start(PPS) no longer invents a synthetic first tooth
//   • the interrupt layer only establishes its PPS drumbeat from an actual PPS edge
//   • therefore this clocks layer must treat handshake completion as:
//       "interrupt consumed a real PPS edge and cleared zero_pending"
//
// ============================================================================

void clocks_beta_pps(void) {
  // --------------------------------------------------------------------------
  // ZERO / START sacred PPS-boundary handshake
  // --------------------------------------------------------------------------
  //
  // Handshake states:
  //
  //   zero_handshake_in_flight == false
  //     → clocks layer has not yet asked interrupt layer to perform sacred PPS zero
  //
  //   zero_handshake_in_flight == true
  //     → clocks layer has asked, and is now waiting for interrupt layer to
  //       consummate that request on a real PPS edge and clear its pending flag
  //
  // Completion:
  //   once interrupt_pps_zero_pending() becomes false again, the interrupt
  //   layer has completed the sacred PPS-boundary reset on an actual PPS edge
  //   and restarted its geared PPS drumbeat from that edge. We then zero local
  //   clocks state here, on this same lawful PPS callback turn.
  //
  // --------------------------------------------------------------------------

  if (request_zero || request_start) {
    if (!zero_handshake_in_flight) {
      interrupt_request_pps_zero();
      zero_handshake_in_flight = true;
      return;
    }

    if (interrupt_pps_zero_pending()) {
      return;
    }

    // Sacred PPS zero has now been consummated by interrupt on a real PPS edge.
    // This is the lawful moment to zero the local campaign-layer state.
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

  // --------------------------------------------------------------------------
  // STOP — lawful PPS callback-turn consummation
  // --------------------------------------------------------------------------
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

  // --------------------------------------------------------------------------
  // RECOVER — lawful PPS callback-turn consummation
  // --------------------------------------------------------------------------
  if (request_recover) {
    watchdog_anomaly_active = false;

    request_zero = false;
    zero_handshake_in_flight = false;

    dwt_cycles_64  = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64    = recover_gnss_ns / 100ull;
    ocxo1_ticks_64 = recover_ocxo1_ns / 100ull;
    ocxo2_ticks_64 = recover_ocxo2_ns / 100ull;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    request_recover = false;
    campaign_state = clocks_campaign_state_t::STARTED;
    return;
  }

  // --------------------------------------------------------------------------
  // Watchdog / campaign gating
  // --------------------------------------------------------------------------
  if (watchdog_anomaly_active) {
    return;
  }

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    return;
  }

  // --------------------------------------------------------------------------
  // Normal started-campaign PPS work
  // --------------------------------------------------------------------------
  campaign_seconds++;

  dwt_cycles_64  = (uint64_t)g_dwt_cycle_count_at_pps;
  gnss_raw_64    = g_gnss_ns_count_at_pps / 100ull;
  ocxo1_ticks_64 = g_ocxo1_clock.ns_count_at_pps / 100ull;
  ocxo2_ticks_64 = g_ocxo2_clock.ns_count_at_pps / 100ull;

  residual_update_sample(
      residual_dwt,
      (int64_t)dwt_effective_cycles_per_second() - (int64_t)DWT_EXPECTED_PER_PPS);

  if (g_ocxo1_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo1, g_ocxo1_measurement.second_residual_ns / 100);
  }

  if (g_ocxo2_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo2, g_ocxo2_measurement.second_residual_ns / 100);
  }

  ocxo_calibration_servo();

  dac_welford_update(dac_welford_ocxo1, ocxo1_dac.dac_fractional);
  dac_welford_update(dac_welford_ocxo2, ocxo2_dac.dac_fractional);

  Payload p;
  p.add("campaign", campaign_name);

  p.add("gnss_ns",  g_gnss_ns_count_at_pps);
  p.add("dwt_ns",   dwt_cycles_to_ns((uint64_t)g_dwt_cycle_count_at_pps));
  p.add("ocxo1_ns", g_ocxo1_clock.ns_count_at_pps);
  p.add("ocxo2_ns", g_ocxo2_clock.ns_count_at_pps);

  p.add("teensy_pps_count", campaign_seconds);

  p.add("dwt_cycle_count_at_pps", g_dwt_cycle_count_at_pps);
  p.add("dwt_cycle_count_next_second_prediction", g_dwt_cycle_count_next_second_prediction);
  p.add("dwt_cycle_count_next_second_adjustment", g_dwt_cycle_count_next_second_adjustment);

  p.add("ocxo1_gnss_ns_at_edge", g_ocxo1_clock.gnss_ns_at_edge);
  p.add("ocxo1_ns_count_at_edge", g_ocxo1_clock.ns_count_at_edge);
  p.add("ocxo1_ns_count_next_second_prediction", g_ocxo1_clock.ns_count_next_second_prediction);
  p.add("ocxo1_ns_count_at_pps", g_ocxo1_clock.ns_count_at_pps);

  p.add("ocxo2_gnss_ns_at_edge", g_ocxo2_clock.gnss_ns_at_edge);
  p.add("ocxo2_ns_count_at_edge", g_ocxo2_clock.ns_count_at_edge);
  p.add("ocxo2_ns_count_next_second_prediction", g_ocxo2_clock.ns_count_next_second_prediction);
  p.add("ocxo2_ns_count_at_pps", g_ocxo2_clock.ns_count_at_pps);

  p.add("ocxo1_second_residual_ns", g_ocxo1_measurement.second_residual_ns);
  p.add("ocxo2_second_residual_ns", g_ocxo2_measurement.second_residual_ns);

  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  clocks_payload_add_interrupt_diag(p, "pps_diag", g_pps_interrupt_diag);
  clocks_payload_add_interrupt_diag(p, "ocxo1_diag", g_ocxo1_interrupt_diag);
  clocks_payload_add_interrupt_diag(p, "ocxo2_diag", g_ocxo2_interrupt_diag);

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
  if (args.tryGetDouble("set_dac1", dac_val)) ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_mode = servo_mode_parse(args.getString("calibrate_ocxo"));

  request_start = true;
  request_stop = false;
  request_recover = false;

  Payload p;
  p.add("status", "start_requested");
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
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
  Payload p;

  p.add("campaign_state",
        campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");
  p.add("campaign", campaign_name);
  p.add("campaign_seconds", campaign_seconds);

  p.add("request_start", request_start);
  p.add("request_stop", request_stop);
  p.add("request_recover", request_recover);
  p.add("request_zero", request_zero);
  p.add("zero_handshake_in_flight", zero_handshake_in_flight);
  p.add("interrupt_pps_zero_pending", interrupt_pps_zero_pending());

  p.add("gnss_ns_count_at_pps", g_gnss_ns_count_at_pps);

  p.add("dwt_cycle_count_at_pps", g_dwt_cycle_count_at_pps);
  p.add("dwt_cycle_count_next_second_prediction", g_dwt_cycle_count_next_second_prediction);
  p.add("dwt_cycle_count_next_second_adjustment", g_dwt_cycle_count_next_second_adjustment);
  p.add("dwt_model_pps_count", g_dwt_model_pps_count);

  p.add("ocxo1_gnss_ns_at_edge", g_ocxo1_clock.gnss_ns_at_edge);
  p.add("ocxo1_ns_count_at_edge", g_ocxo1_clock.ns_count_at_edge);
  p.add("ocxo1_ns_count_next_second_prediction", g_ocxo1_clock.ns_count_next_second_prediction);
  p.add("ocxo1_ns_count_at_pps", g_ocxo1_clock.ns_count_at_pps);

  p.add("ocxo2_gnss_ns_at_edge", g_ocxo2_clock.gnss_ns_at_edge);
  p.add("ocxo2_ns_count_at_edge", g_ocxo2_clock.ns_count_at_edge);
  p.add("ocxo2_ns_count_next_second_prediction", g_ocxo2_clock.ns_count_next_second_prediction);
  p.add("ocxo2_ns_count_at_pps", g_ocxo2_clock.ns_count_at_pps);

  p.add("ocxo1_second_residual_ns", g_ocxo1_measurement.second_residual_ns);
  p.add("ocxo2_second_residual_ns", g_ocxo2_measurement.second_residual_ns);

  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
  p.add("calibrate_ocxo", servo_mode_str(calibrate_ocxo_mode));

  p.add("timebase_valid", timebase_valid());
  p.add("timebase_gnss_ns",  timebase_now_ns(timebase_domain_t::GNSS));
  p.add("timebase_dwt_ns",   timebase_now_ns(timebase_domain_t::DWT));
  p.add("timebase_ocxo1_ns", timebase_now_ns(timebase_domain_t::OCXO1));
  p.add("timebase_ocxo2_ns", timebase_now_ns(timebase_domain_t::OCXO2));

  return p;
}

static Payload cmd_set_dac(const Payload& args) {
  double dac_val;
  if (args.tryGetDouble("set_dac1", dac_val)) ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  Payload p;
  p.add("ocxo1_dac", ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac", ocxo2_dac.dac_fractional);
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