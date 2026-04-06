// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// Time source discipline — gears, not rubber bands:
//
//   All canonical time values are derived from ISR-captured facts or from
//   pure arithmetic on those facts.  No live counter reads are used for
//   canonical state.
//
//   The QTimer value for time_pps_update() is geared:
//     g_qtimer_at_pps += TICKS_10MHZ_PER_SECOND
//   The VCLOCK runs on the GNSS 10 MHz input, so this is exact by definition.
//   The initial QTimer value is captured once at epoch installation — the
//   sole live QTimer read, a boot-time boundary crossing.
//
//   The DWT adjustment timer reads DWT_CYCCNT directly at 10 kHz.  This is
//   a legitimate real-time feedback loop on the DWT bridge substrate.
//
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

// ============================================================================
// Constants
// ============================================================================

static constexpr uint64_t PPS_RELAY_OFF_NS    = 500000000ULL;
static constexpr uint64_t DWT_ADJUST_TIMER_NS = 100000ULL; // 10 kHz
static constexpr int64_t  DWT_ADJUST_THRESHOLD_NS = 2LL;
static constexpr uint64_t NS_PER_SECOND_U64   = 1000000000ULL;

// ============================================================================
// Always-on state
// ============================================================================

volatile uint64_t g_gnss_ns_count_at_pps = 0;

volatile uint32_t g_dwt_cycle_count_at_pps = 0;
volatile uint32_t g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
volatile int32_t  g_dwt_cycle_count_next_second_adjustment = 0;
volatile uint64_t g_dwt_model_pps_count = 0;

// Geared QTimer — advanced by TICKS_10MHZ_PER_SECOND each PPS.
// Initialized once from a live read at epoch installation, then
// pure arithmetic thereafter.  Never read from hardware again.
volatile uint32_t g_qtimer_at_pps = 0;

ocxo_clock_state_t g_ocxo1_clock = {};
ocxo_clock_state_t g_ocxo2_clock = {};

ocxo_measurement_t g_ocxo1_measurement = {};
ocxo_measurement_t g_ocxo2_measurement = {};

interrupt_capture_diag_t g_pps_interrupt_diag = {};
interrupt_capture_diag_t g_ocxo1_interrupt_diag = {};
interrupt_capture_diag_t g_ocxo2_interrupt_diag = {};

bool g_ad5693r_init_ok = false;

// ============================================================================
// Alpha epoch state
// ============================================================================
//
// This is the canonical zero-based epoch for the always-on physics layer.
// It is established only on a lawful PPS edge.
//
// The physical raw values captured here are not themselves the canonical
// clocks; they are the shared boundary facts from which the canonical
// synthetic clocks are defined.
//

enum class clocks_epoch_reason_t : uint8_t {
  NONE   = 0,
  INIT   = 1,
  ZERO   = 2,
  START  = 3,
};

static volatile bool g_epoch_pending = false;
static volatile bool g_epoch_initialized = false;
static volatile uint32_t g_epoch_generation = 0;
static volatile clocks_epoch_reason_t g_epoch_reason = clocks_epoch_reason_t::NONE;

// Raw physical anchors captured at the consummating PPS boundary.
static volatile uint32_t g_epoch_dwt_at_pps = 0;
static volatile uint32_t g_epoch_qtimer_at_pps = 0;

// Canonical PPS count within current epoch (0 at epoch PPS, then 1, 2, 3...).
static volatile uint64_t g_epoch_pps_index = 0;

// ============================================================================
// DAC state
// ============================================================================

ocxo_dac_state_t ocxo1_dac = {
  (double)AD5693R_DAC_DEFAULT, AD5693R_DAC_DEFAULT, 0, 65535,
  0, 0.0, 0, 0
};

ocxo_dac_state_t ocxo2_dac = {
  (double)AD5693R_DAC_DEFAULT, AD5693R_DAC_DEFAULT, 0, 65535,
  0, 0.0, 0, 0
};

servo_mode_t calibrate_ocxo_mode = servo_mode_t::OFF;

const char* servo_mode_str(servo_mode_t mode) {
  switch (mode) {
    case servo_mode_t::MEAN:  return "MEAN";
    case servo_mode_t::TOTAL: return "TOTAL";
    case servo_mode_t::NOW:   return "NOW";
    default:                  return "OFF";
  }
}

servo_mode_t servo_mode_parse(const char* s) {
  if (!s || !*s) return servo_mode_t::OFF;
  if (strcmp(s, "MEAN")  == 0) return servo_mode_t::MEAN;
  if (strcmp(s, "TOTAL") == 0) return servo_mode_t::TOTAL;
  if (strcmp(s, "NOW")   == 0) return servo_mode_t::NOW;
  return servo_mode_t::OFF;
}

void ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  if (value < (double)s.dac_min) value = (double)s.dac_min;
  if (value > (double)s.dac_max) value = (double)s.dac_max;
  s.dac_fractional = value;
  s.dac_hw_code = (uint16_t)value;

  if (&s == &ocxo1_dac) {
    ad5693r_write_input(AD5693R_ADDR_OCXO1, s.dac_hw_code);
    ad5693r_update_dac(AD5693R_ADDR_OCXO1);
  } else {
    ad5693r_write_input(AD5693R_ADDR_OCXO2, s.dac_hw_code);
    ad5693r_update_dac(AD5693R_ADDR_OCXO2);
  }
}

// ============================================================================
// Campaign accumulators
// ============================================================================

uint64_t dwt_cycles_64  = 0;
uint64_t gnss_raw_64    = 0;
uint64_t ocxo1_ticks_64 = 0;
uint64_t ocxo2_ticks_64 = 0;

// ============================================================================
// Relay state
// ============================================================================

static volatile bool relay_timer_active = false;

static void pps_relay_deassert(timepop_ctx_t*, timepop_diag_t*, void*) {
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  relay_timer_active = false;
}

// ============================================================================
// DWT
// ============================================================================

static inline void dwt_enable(void) {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

// ============================================================================
// QTimer1 read — boot-time boundary crossing ONLY
// ============================================================================
//
// This function reads the live QTimer1 cascaded 32-bit counter.
// It is called ONCE at epoch installation to establish the initial
// geared QTimer anchor.  After that, g_qtimer_at_pps is advanced
// by TICKS_10MHZ_PER_SECOND each PPS — pure arithmetic, no read.
//
// This is a boundary crossing in the ZPNet Standards sense:
// the QTimer hardware is an unowned external system at boot time,
// before the geared model takes over.

static uint32_t qtimer1_read_32_once(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;

  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;

  if (hi1 != hi2) {
    return ((uint32_t)hi2 << 16) | (uint32_t)lo2;
  }
  return ((uint32_t)hi1 << 16) | (uint32_t)lo1;
}

// ============================================================================
// Epoch helpers
// ============================================================================

static void alpha_request_epoch_zero(clocks_epoch_reason_t reason) {
  g_epoch_pending = true;
  g_epoch_reason = reason;
}

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  g_gnss_ns_count_at_pps = 0;

  g_dwt_cycle_count_at_pps = 0;
  g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_model_pps_count = 0;
  g_qtimer_at_pps = 0;

  g_ocxo1_clock = {};
  g_ocxo2_clock = {};

  g_ocxo1_measurement = {};
  g_ocxo2_measurement = {};

  g_pps_interrupt_diag = {};
  g_ocxo1_interrupt_diag = {};
  g_ocxo2_interrupt_diag = {};

  dwt_cycles_64  = 0;
  gnss_raw_64    = 0;
  ocxo1_ticks_64 = 0;
  ocxo2_ticks_64 = 0;
}

static void alpha_install_new_epoch_from_pps(const interrupt_event_t& pps_event) {
  g_epoch_initialized = true;
  g_epoch_pending = false;
  g_epoch_generation++;
  g_epoch_pps_index = 0;

  g_epoch_dwt_at_pps = pps_event.dwt_at_event;

  // The sole live QTimer read — boot-time boundary crossing.
  // After this, g_qtimer_at_pps is advanced by pure arithmetic.
  g_epoch_qtimer_at_pps = qtimer1_read_32_once();

  alpha_reset_canonical_clock_state_for_new_epoch();

  // Canonical PPS-origin values are zero at the epoch edge.
  g_gnss_ns_count_at_pps = 0;
  g_dwt_cycle_count_at_pps = pps_event.dwt_at_event;
  g_dwt_model_pps_count = 0;
  g_qtimer_at_pps = g_epoch_qtimer_at_pps;

  g_ocxo1_clock.gnss_ns_at_edge = 0;
  g_ocxo1_clock.ns_count_at_edge = 0;
  g_ocxo1_clock.ns_count_next_second_prediction = NS_PER_SECOND_U64;
  g_ocxo1_clock.ns_count_at_pps = 0;

  g_ocxo2_clock.gnss_ns_at_edge = 0;
  g_ocxo2_clock.ns_count_at_edge = 0;
  g_ocxo2_clock.ns_count_next_second_prediction = NS_PER_SECOND_U64;
  g_ocxo2_clock.ns_count_at_pps = 0;

  // Restart the universal local time epoch at this lawful PPS edge.
  time_pps_epoch_reset(pps_event.dwt_at_event, g_epoch_qtimer_at_pps);
}

// ============================================================================
// Helper: derive DWT adjustment at 10 kHz
// ============================================================================
//
// This reads DWT_CYCCNT directly — a legitimate real-time feedback loop
// on the DWT bridge substrate.  The DWT is not a canonical time source;
// it is the interpolation substrate between PPS edges, and this timer
// keeps the interpolation rate prediction accurate mid-second.

static void dwt_adjustment_timer_callback(timepop_ctx_t* ctx, timepop_diag_t*, void*) {
  if (g_gnss_ns_count_at_pps == 0) return;
  if (!time_valid()) return;

  const uint32_t current_dwt = DWT_CYCCNT;
  const uint64_t elapsed_gnss_ns = ctx->fire_gnss_ns - g_gnss_ns_count_at_pps;
  if (elapsed_gnss_ns == 0) return;

  const int64_t effective =
      (int64_t)g_dwt_cycle_count_next_second_prediction +
      (int64_t)g_dwt_cycle_count_next_second_adjustment;

  const int64_t predicted_dwt =
      (int64_t)g_dwt_cycle_count_at_pps +
      ((int64_t)elapsed_gnss_ns * effective) / 1000000000LL;

  const int64_t residual_cycles = (int64_t)current_dwt - predicted_dwt;
  const int64_t residual_ns = (int64_t)dwt_cycles_to_ns((uint64_t)llabs(residual_cycles));

  if (residual_ns >= DWT_ADJUST_THRESHOLD_NS) {
    const int64_t candidate_total =
        ((int64_t)(current_dwt - g_dwt_cycle_count_at_pps) * 1000000000LL) /
        (int64_t)elapsed_gnss_ns;

    g_dwt_cycle_count_next_second_adjustment =
        (int32_t)(candidate_total - (int64_t)g_dwt_cycle_count_next_second_prediction);
  }
}

// ============================================================================
// Interrupt subscribers
// ============================================================================

static void pps_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  static uint32_t prev_dwt_pps = 0;

  clocks_capture_interrupt_diag(g_pps_interrupt_diag, diag);

  // --------------------------------------------------------------------------
  // Epoch establishment — only on a lawful PPS edge
  // --------------------------------------------------------------------------
  if (!g_epoch_initialized || g_epoch_pending) {
    alpha_install_new_epoch_from_pps(event);

    prev_dwt_pps = event.dwt_at_event;

    if (campaign_state == clocks_campaign_state_t::STARTED ||
        request_start || request_stop || request_recover || request_zero) {
      clocks_beta_pps();
    }

    digitalWriteFast(GNSS_PPS_RELAY, HIGH);

    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
    return;
  }

  // --------------------------------------------------------------------------
  // Normal PPS advancement within current epoch
  // --------------------------------------------------------------------------
  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps = g_epoch_pps_index * NS_PER_SECOND_U64;
  g_dwt_cycle_count_at_pps = event.dwt_at_event;

  // Geared QTimer — pure arithmetic, no live read.
  // VCLOCK runs on the GNSS 10 MHz input, so exactly TICKS_10MHZ_PER_SECOND
  // ticks elapse between consecutive PPS edges by definition.
  g_qtimer_at_pps += TICKS_10MHZ_PER_SECOND;

  if (g_dwt_model_pps_count > 0) {
    const uint32_t actual_delta = event.dwt_at_event - prev_dwt_pps;
    g_dwt_cycle_count_next_second_prediction = actual_delta;
  }

  prev_dwt_pps = event.dwt_at_event;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_model_pps_count++;

  if (g_ocxo1_clock.gnss_ns_at_edge != 0 || g_ocxo1_measurement.prev_gnss_ns_at_edge == 0) {
    const uint64_t gnss_since = g_gnss_ns_count_at_pps - g_ocxo1_clock.gnss_ns_at_edge;
    g_ocxo1_clock.ns_count_at_pps =
        g_ocxo1_clock.ns_count_at_edge +
        (gnss_since * g_ocxo1_clock.ns_count_next_second_prediction) / 1000000000ULL;
  } else {
    g_ocxo1_clock.ns_count_at_pps = 0;
  }

  if (g_ocxo2_clock.gnss_ns_at_edge != 0 || g_ocxo2_measurement.prev_gnss_ns_at_edge == 0) {
    const uint64_t gnss_since = g_gnss_ns_count_at_pps - g_ocxo2_clock.gnss_ns_at_edge;
    g_ocxo2_clock.ns_count_at_pps =
        g_ocxo2_clock.ns_count_at_edge +
        (gnss_since * g_ocxo2_clock.ns_count_next_second_prediction) / 1000000000ULL;
  } else {
    g_ocxo2_clock.ns_count_at_pps = 0;
  }

  // Update the universal time anchor — geared QTimer, ISR-captured DWT.
  time_pps_update(
    g_dwt_cycle_count_at_pps,
    dwt_effective_cycles_per_second(),
    g_qtimer_at_pps);

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }

  // If beta just initiated a zero handshake, request alpha epoch reset.
  // This must be checked AFTER clocks_beta_pps() so beta has had a chance
  // to call interrupt_request_pps_zero() on this turn.
  if (request_zero && !g_epoch_pending) {
    alpha_request_epoch_zero(clocks_epoch_reason_t::ZERO);
  }

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);

  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }
}

static void ocxo1_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  clocks_capture_interrupt_diag(g_ocxo1_interrupt_diag, diag);

  // Canonical GNSS epoch is local and zero-based.
  //
  // The OCXO nanosecond count at edge is not interrogated from hardware.
  // It is known by contract because process_interrupt armed OCR3 to fire on
  // a specific 10 MHz count and supplies that just-matched count here.
  g_ocxo1_clock.gnss_ns_at_edge = event.gnss_ns_at_event;
  g_ocxo1_clock.ns_count_at_edge = (uint64_t)event.counter32_at_event * 100ULL;

  if (g_ocxo1_measurement.prev_gnss_ns_at_edge != 0) {
    const int64_t period_ns =
        (int64_t)g_ocxo1_clock.gnss_ns_at_edge -
        (int64_t)g_ocxo1_measurement.prev_gnss_ns_at_edge;

    g_ocxo1_clock.ns_count_next_second_prediction = (uint64_t)period_ns;
    g_ocxo1_measurement.second_residual_ns = period_ns - 1000000000LL;
  } else {
    g_ocxo1_clock.ns_count_next_second_prediction = 1000000000ULL;
    g_ocxo1_measurement.second_residual_ns = 0;
  }

  g_ocxo1_measurement.prev_gnss_ns_at_edge = g_ocxo1_clock.gnss_ns_at_edge;
}

static void ocxo2_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  clocks_capture_interrupt_diag(g_ocxo2_interrupt_diag, diag);

  // The OCXO nanosecond count at edge is the just-matched OCR3 count supplied
  // by process_interrupt, expressed in nanoseconds.
  g_ocxo2_clock.gnss_ns_at_edge = event.gnss_ns_at_event;
  g_ocxo2_clock.ns_count_at_edge = (uint64_t)event.counter32_at_event * 100ULL;

  if (g_ocxo2_measurement.prev_gnss_ns_at_edge != 0) {
    const int64_t period_ns =
        (int64_t)g_ocxo2_clock.gnss_ns_at_edge -
        (int64_t)g_ocxo2_measurement.prev_gnss_ns_at_edge;

    g_ocxo2_clock.ns_count_next_second_prediction = (uint64_t)period_ns;
    g_ocxo2_measurement.second_residual_ns = period_ns - 1000000000LL;
  } else {
    g_ocxo2_clock.ns_count_next_second_prediction = 1000000000ULL;
    g_ocxo2_measurement.second_residual_ns = 0;
  }

  g_ocxo2_measurement.prev_gnss_ns_at_edge = g_ocxo2_clock.gnss_ns_at_edge;
}

// ============================================================================
// Initialization — Phase 1 (hardware only)
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
}

// ============================================================================
// Initialization — Phase 2
// ============================================================================

void process_clocks_init(void) {
  time_init();
  timebase_init();

  // Startup uses the same sacred epoch model as ZERO/START.
  alpha_request_epoch_zero(clocks_epoch_reason_t::INIT);

  g_ad5693r_init_ok = ad5693r_init();

  ocxo_dac_set(ocxo1_dac, (double)AD5693R_DAC_DEFAULT);
  ocxo_dac_set(ocxo2_dac, (double)AD5693R_DAC_DEFAULT);

  pinMode(GNSS_LOCK_PIN,  INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  interrupt_subscription_t pps_sub {};
  pps_sub.kind = interrupt_subscriber_kind_t::PPS;
  pps_sub.on_event = pps_callback;
  pps_sub.user_data = nullptr;
  interrupt_subscribe(pps_sub);
  interrupt_start(interrupt_subscriber_kind_t::PPS);

  interrupt_subscription_t ocxo1_sub {};
  ocxo1_sub.kind = interrupt_subscriber_kind_t::OCXO1;
  ocxo1_sub.on_event = ocxo1_callback;
  ocxo1_sub.user_data = nullptr;
  interrupt_subscribe(ocxo1_sub);
  interrupt_start(interrupt_subscriber_kind_t::OCXO1);

  interrupt_subscription_t ocxo2_sub {};
  ocxo2_sub.kind = interrupt_subscriber_kind_t::OCXO2;
  ocxo2_sub.on_event = ocxo2_callback;
  ocxo2_sub.user_data = nullptr;
  interrupt_subscribe(ocxo2_sub);
  interrupt_start(interrupt_subscriber_kind_t::OCXO2);

  timepop_arm(DWT_ADJUST_TIMER_NS, true, dwt_adjustment_timer_callback, nullptr, "dwt-adjust");
}