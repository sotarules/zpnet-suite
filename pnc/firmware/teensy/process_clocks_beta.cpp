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
//
// IMPORTANT:
// Canonical always-on physics clocks are no longer zeroed here.
// Alpha owns epoch installation on the lawful PPS edge.
// Beta zeroing is campaign-layer reset only.
//

void clocks_zero_all(void) {
  timebase_invalidate();

  campaign_seconds = 0;

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;

  residual_reset(residual_dwt);
  residual_reset(residual_ocxo1);
  residual_reset(residual_ocxo2);

  dac_welford_reset(dac_welford_ocxo1);
  dac_welford_reset(dac_welford_ocxo2);
}

// ============================================================================
// Servo logic
// ============================================================================

static void ocxo_servo_mean(ocxo_dac_state_t& dac, pps_residual_t& per_second_residuals) {
  if (per_second_residuals.n < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  const double mean_residual_ns = per_second_residuals.mean;
  dac.servo_last_residual = mean_residual_ns;

  if (fabs(mean_residual_ns) < 0.01) return;

  // Per-second residuals are now in full nanoseconds, so use a much gentler
  // control law than the legacy surrogate signal. The sign here is chosen so
  // that negative residuals move DAC downward rather than rail upward.
  double step = mean_residual_ns * 0.01;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
  dac.servo_settle_count = 0;

  residual_reset(per_second_residuals);
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

static constexpr double NOW_NS_PER_DAC_LSB = 100.0;
static constexpr double NOW_MIN_RESIDUAL_NS = 5.0;

static void ocxo_servo_now(ocxo_dac_state_t& dac, int64_t per_second_residual_ns) {
  if (campaign_seconds < SERVO_MIN_SAMPLES) return;
  if (per_second_residual_ns == 0) return;

  const double residual_ns = (double)per_second_residual_ns;
  dac.servo_last_residual = residual_ns;

  if (fabs(residual_ns) < NOW_MIN_RESIDUAL_NS) return;

  // Per-second OCXO residual is authoritative. Use conservative gain for the
  // first hardware pass and choose polarity so negative residuals do not drive
  // DAC monotonically upward.
  double step = residual_ns / NOW_NS_PER_DAC_LSB;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_last_step = step;
  dac.servo_adjustments++;
}

static void ocxo_calibration_servo(void) {
  if (calibrate_ocxo_mode == servo_mode_t::OFF) return;

  // Servo mode semantics:
  //   MEAN  = Welford mean of the per-second OCXO residual stream
  //   NOW   = immediate per-second OCXO residual
  //   TOTAL = cumulative OCXO-vs-GNSS divergence at PPS
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
//   piggybacks on the same sacred zero/reset contract before the campaign
//   actually becomes STARTED
//
// request_stop:
//   consummated on this lawful PPS callback turn
//
// request_recover:
//   consummated on this lawful PPS callback turn
//
// Important with the alpha epoch model:
//
//   • alpha now owns canonical epoch installation on the lawful PPS edge
//   • beta no longer tries to zero the always-on physics clocks directly
//   • handshake completion means:
//       "interrupt consummated PPS zero on a real PPS edge, and alpha
//        installed the new canonical epoch on that same lawful turn"
//
// ============================================================================

void clocks_beta_pps(void) {
  // --------------------------------------------------------------------------
  // ZERO / START sacred PPS-boundary handshake
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

    // Sacred PPS zero has now been consummated on a real PPS edge.
    // Alpha has already installed the new canonical epoch.
    // This is the lawful moment to reset campaign-layer state only.
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

    dwt_cycle_count_total = dwt_ns_to_cycles(recover_dwt_ns);
    gnss_raw_64           = recover_gnss_ns / 100ull;
    ocxo1_ticks_64        = recover_ocxo1_ns / 100ull;
    ocxo2_ticks_64        = recover_ocxo2_ns / 100ull;

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

  // Campaign accumulators — all derived from ISR-captured facts.
  dwt_cycle_count_total = g_dwt_cycle_count_total;
  gnss_raw_64           = g_gnss_ns_count_at_pps / 100ull;
  ocxo1_ticks_64        = g_ocxo1_clock.ns_count_at_pps / 100ull;
  ocxo2_ticks_64        = g_ocxo2_clock.ns_count_at_pps / 100ull;

  residual_update_sample(
      residual_dwt,
      (int64_t)dwt_effective_cycles_per_second() - (int64_t)DWT_EXPECTED_PER_PPS);

  if (g_ocxo1_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo1, g_ocxo1_measurement.second_residual_ns);
  }

  if (g_ocxo2_measurement.prev_gnss_ns_at_edge != 0) {
    residual_update_sample(residual_ocxo2, g_ocxo2_measurement.second_residual_ns);
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
  p.add("dwt_cycle_count_next_second_prediction", g_dwt_cycle_count_next_second_prediction);
  p.add("dwt_cycle_count_next_second_adjustment", g_dwt_cycle_count_next_second_adjustment);

  p.add("ocxo1_gnss_ns_at_edge", g_ocxo1_clock.gnss_ns_at_edge);
  p.add("ocxo1_ns_count_at_edge", g_ocxo1_clock.ns_count_at_edge);
  p.add("ocxo1_ns_count_at_pps", g_ocxo1_clock.ns_count_at_pps);

  p.add("ocxo2_gnss_ns_at_edge", g_ocxo2_clock.gnss_ns_at_edge);
  p.add("ocxo2_ns_count_at_edge", g_ocxo2_clock.ns_count_at_edge);
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
  p.add("dwt_cycle_count_total", g_dwt_cycle_count_total);
  p.add("dwt_cycle_count_next_second_prediction", g_dwt_cycle_count_next_second_prediction);
  p.add("dwt_cycle_count_next_second_adjustment", g_dwt_cycle_count_next_second_adjustment);
  p.add("dwt_model_pps_count", g_dwt_model_pps_count);

  p.add("qtimer_at_pps", g_qtimer_at_pps);

  p.add("ocxo1_gnss_ns_at_edge", g_ocxo1_clock.gnss_ns_at_edge);
  p.add("ocxo1_ns_count_at_edge", g_ocxo1_clock.ns_count_at_edge);
  p.add("ocxo1_ns_count_at_pps", g_ocxo1_clock.ns_count_at_pps);

  p.add("ocxo2_gnss_ns_at_edge", g_ocxo2_clock.gnss_ns_at_edge);
  p.add("ocxo2_ns_count_at_edge", g_ocxo2_clock.ns_count_at_edge);
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