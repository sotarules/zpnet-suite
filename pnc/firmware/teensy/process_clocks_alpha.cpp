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
//
//   Epoch installation uses the PPS event contract supplied by
//   process_interrupt.  The consummating PPS event carries both:
//
//     • dwt_at_event      — reconstructed DWT time of the PPS edge
//     • counter32_at_event — VCLOCK position corresponding to that same edge
//
//   No ambient QTimer read is permitted at epoch installation, because that
//   would bless a later instant as though it were the PPS boundary.
//
//   The DWT adjustment timer reads DWT_CYCCNT directly at 10 kHz.  This is
//   a legitimate real-time feedback loop on the DWT bridge substrate.
//
// Smart-zero OCXO phase model:
//
//   OCXO ns_count_at_edge is a pure OCXO-domain counter that advances by
//   exactly 1,000,000,000 per lawful one-second OCXO edge.  It does not
//   incorporate GNSS measurements.  Phase offset is the cumulative
//   difference: gnss_ns_at_edge - ns_count_at_edge.
//
//   Per-window viability checks whether each GNSS-measured OCXO second
//   approximates 1e9 ns.  This replaces the old cumulative viability
//   audit, which was answering the wrong question.
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
volatile uint64_t g_dwt_cycle_count_total = 0;
volatile uint32_t g_dwt_cycle_count_between_pps = 0;
volatile uint32_t g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
volatile int32_t  g_dwt_cycle_count_next_second_adjustment = 0;
volatile uint64_t g_dwt_model_pps_count = 0;

// Lawful VCLOCK anchor captured at each PPS event.
// Unlike the historical geared follower, this is now the actual event-carried
// CH0/CH1 position corresponding to the PPS boundary.
volatile uint32_t g_qtimer_at_pps = 0;
volatile uint32_t g_last_pps_event_counter32_at_event = 0;

volatile int32_t g_last_pps_live_qtimer_minus_geared = 0;
volatile uint32_t g_last_pps_live_qtimer_read = 0;

vclock_clock_state_t g_vclock_clock = {};
vclock_measurement_t g_vclock_measurement = {};

ocxo_clock_state_t g_ocxo1_clock = {};
ocxo_clock_state_t g_ocxo2_clock = {};

ocxo_measurement_t g_ocxo1_measurement = {};
ocxo_measurement_t g_ocxo2_measurement = {};

interrupt_capture_diag_t g_pps_interrupt_diag = {};
interrupt_capture_diag_t g_ocxo1_interrupt_diag = {};
interrupt_capture_diag_t g_ocxo2_interrupt_diag = {};

bool g_ad5693r_init_ok = false;

volatile uint32_t g_epoch_live_qtimer_read = 0;
volatile int32_t  g_epoch_live_qtimer_minus_event = 0;

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
  0, 0.0, 0, 0,
  false, 0.0, 0.0, 0.0, 0.0, 0
};

ocxo_dac_state_t ocxo2_dac = {
  (double)AD5693R_DAC_DEFAULT, AD5693R_DAC_DEFAULT, 0, 65535,
  0, 0.0, 0, 0,
  false, 0.0, 0.0, 0.0, 0.0, 0
};

servo_mode_t calibrate_ocxo_mode = servo_mode_t::OFF;

const char* servo_mode_str(servo_mode_t mode) {
  switch (mode) {
    case servo_mode_t::TOTAL: return "TOTAL";
    case servo_mode_t::NOW:   return "NOW";
    default:                  return "OFF";
  }
}

servo_mode_t servo_mode_parse(const char* s) {
  if (!s || !*s) return servo_mode_t::OFF;
  if (strcmp(s, "TOTAL") == 0) return servo_mode_t::TOTAL;
  if (strcmp(s, "MEAN")  == 0) return servo_mode_t::TOTAL;  // legacy compat
  if (strcmp(s, "NOW")   == 0) return servo_mode_t::NOW;
  return servo_mode_t::OFF;
}

void ocxo_dac_predictor_reset(ocxo_dac_state_t& s) {
  s.servo_last_step = 0.0;
  s.servo_last_residual = 0.0;
  s.servo_settle_count = 0;
  s.servo_adjustments = 0;
  s.servo_predictor_initialized = false;
  s.servo_last_raw_residual = 0.0;
  s.servo_filtered_residual = 0.0;
  s.servo_filtered_slope = 0.0;
  s.servo_predicted_residual = 0.0;
  s.servo_predictor_updates = 0;
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

uint64_t dwt_cycle_count_total = 0;
uint64_t gnss_raw_64           = 0;
uint64_t ocxo1_ticks_64        = 0;
uint64_t ocxo2_ticks_64        = 0;

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
// Epoch helpers
// ============================================================================

static void alpha_request_epoch_zero(clocks_epoch_reason_t reason) {
  g_epoch_pending = true;
  g_epoch_reason = reason;
}

static inline bool alpha_ocxo_edges_are_canonical(void) {
  return g_epoch_initialized && !g_epoch_pending;
}

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  g_gnss_ns_count_at_pps = 0;

  g_dwt_cycle_count_at_pps = 0;
  g_dwt_cycle_count_total = 0;
  g_dwt_cycle_count_between_pps = 0;
  g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_model_pps_count = 0;
  g_qtimer_at_pps = 0;
  g_vclock_clock = {};
  g_vclock_measurement = {};

  g_ocxo1_clock = {};
  g_ocxo2_clock = {};

  g_ocxo1_measurement = {};
  g_ocxo2_measurement = {};

  g_pps_interrupt_diag = {};
  g_ocxo1_interrupt_diag = {};
  g_ocxo2_interrupt_diag = {};

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;
}

static void alpha_install_new_epoch_from_pps(const interrupt_event_t& pps_event) {
  g_epoch_initialized = true;
  g_epoch_pending = false;
  g_epoch_generation++;
  g_epoch_pps_index = 0;

  g_epoch_dwt_at_pps = pps_event.dwt_at_event;
  g_epoch_qtimer_at_pps = pps_event.counter32_at_event;

  const uint32_t live_qtimer_now = interrupt_qtimer1_counter32_now();   // or equivalent helper
  g_epoch_live_qtimer_read = live_qtimer_now;
  g_epoch_live_qtimer_minus_event = (int32_t)(live_qtimer_now - pps_event.counter32_at_event);

  alpha_reset_canonical_clock_state_for_new_epoch();

  g_gnss_ns_count_at_pps = 0;
  g_dwt_cycle_count_at_pps = pps_event.dwt_at_event;
  g_dwt_cycle_count_total = 0;
  g_dwt_cycle_count_between_pps = 0;
  g_dwt_model_pps_count = 0;
  g_qtimer_at_pps = g_epoch_qtimer_at_pps;

  g_vclock_clock.counter32_at_pps_event = pps_event.counter32_at_event;
  g_vclock_clock.counter32_at_pps_expected = pps_event.counter32_at_event;
  g_vclock_clock.counter32_error_at_pps = 0;
  g_vclock_clock.ns_count_at_pps = 0;
  g_vclock_clock.ns_count_expected_at_pps = 0;
  g_vclock_clock.zero_established = true;

  g_vclock_measurement.ticks_between_pps = 0;
  g_vclock_measurement.ns_between_pps = 0;
  g_vclock_measurement.second_residual_ns = 0;
  g_vclock_measurement.prev_counter32_at_pps_event = pps_event.counter32_at_event;

  // OCXO smart-zero: aggregate reset already zeroed g_ocxo1_clock and
  // g_ocxo2_clock via = {}.  zero_established starts false.  The first
  // post-epoch OCXO one-second edge will seed the phase offset ledger.

  time_pps_epoch_reset(pps_event.dwt_at_event, g_epoch_qtimer_at_pps);
}

// ============================================================================
// Helper: derive DWT adjustment at 10 kHz
// ============================================================================

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

  if (diag) g_pps_interrupt_diag = *diag;
  else g_pps_interrupt_diag = {};
  g_last_pps_event_counter32_at_event = event.counter32_at_event;

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

  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps = g_epoch_pps_index * NS_PER_SECOND_U64;
  g_dwt_cycle_count_at_pps = event.dwt_at_event;

  // VCLOCK is now tracked on two rails:
  //   1) actual lawful CH0/CH1 position carried by the PPS event
  //   2) synthetic/geared expectation advanced by exactly 10,000,000 ticks
  const uint32_t actual_qtimer_at_pps = event.counter32_at_event;
  const uint32_t expected_qtimer_at_pps =
      g_vclock_clock.zero_established
          ? (g_vclock_clock.counter32_at_pps_expected + TICKS_10MHZ_PER_SECOND)
          : actual_qtimer_at_pps;

  g_qtimer_at_pps = actual_qtimer_at_pps;
  g_vclock_clock.counter32_at_pps_event = actual_qtimer_at_pps;
  g_vclock_clock.counter32_at_pps_expected = expected_qtimer_at_pps;
  g_vclock_clock.counter32_error_at_pps =
      (int32_t)(actual_qtimer_at_pps - expected_qtimer_at_pps);

  if (g_vclock_measurement.prev_counter32_at_pps_event != 0) {
    const uint32_t ticks_between_pps =
        actual_qtimer_at_pps - g_vclock_measurement.prev_counter32_at_pps_event;
    const uint64_t ns_between_pps = (uint64_t)ticks_between_pps * 100ULL;

    g_vclock_measurement.ticks_between_pps = ticks_between_pps;
    g_vclock_measurement.ns_between_pps = ns_between_pps;
    g_vclock_measurement.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)ns_between_pps;

    g_vclock_clock.ns_count_at_pps += ns_between_pps;
    g_vclock_clock.ns_count_expected_at_pps += NS_PER_SECOND_U64;
  }

  g_vclock_measurement.prev_counter32_at_pps_event = actual_qtimer_at_pps;
  g_vclock_clock.zero_established = true;

  const uint32_t live_qtimer_now = interrupt_qtimer1_counter32_now();   // same helper
  g_last_pps_live_qtimer_read = live_qtimer_now;
  g_last_pps_live_qtimer_minus_geared = (int32_t)(live_qtimer_now - expected_qtimer_at_pps);

  {
    const uint32_t delta32 = event.dwt_at_event - prev_dwt_pps;
    g_dwt_cycle_count_total += (uint64_t)delta32;
    g_dwt_cycle_count_between_pps = delta32;

    if (g_dwt_model_pps_count > 0) {
      g_dwt_cycle_count_next_second_prediction = delta32;
    }
  }

  prev_dwt_pps = event.dwt_at_event;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_model_pps_count++;

  // Project OCXO ns to PPS boundary using the phase offset ledger.
  //
  //   ns_count_at_pps = gnss_ns_at_pps - phase_offset_ns
  //
  // This gives the OCXO's own ns total, projected to the PPS instant
  // via the last-known cumulative phase relationship.

  if (g_ocxo1_clock.zero_established) {
    g_ocxo1_clock.ns_count_at_pps =
        (uint64_t)((int64_t)g_gnss_ns_count_at_pps - g_ocxo1_clock.phase_offset_ns);
  } else {
    g_ocxo1_clock.ns_count_at_pps = 0;
  }

  if (g_ocxo2_clock.zero_established) {
    g_ocxo2_clock.ns_count_at_pps =
        (uint64_t)((int64_t)g_gnss_ns_count_at_pps - g_ocxo2_clock.phase_offset_ns);
  } else {
    g_ocxo2_clock.ns_count_at_pps = 0;
  }

  time_pps_update(
    g_dwt_cycle_count_at_pps,
    dwt_effective_cycles_per_second(),
    g_qtimer_at_pps);

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }

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

  if (diag) g_ocxo1_interrupt_diag = *diag;
  else g_ocxo1_interrupt_diag = {};

  if (!alpha_ocxo_edges_are_canonical()) {
    return;
  }

  const uint32_t dwt_at_edge = event.dwt_at_event;
  const uint64_t gnss_ns_at_edge = event.gnss_ns_at_event;

  g_ocxo1_clock.gnss_ns_at_edge = gnss_ns_at_edge;
  g_ocxo1_measurement.dwt_at_edge = dwt_at_edge;

  if (g_ocxo1_measurement.prev_gnss_ns_at_edge == 0) {
    // ── First one-second OCXO edge after epoch zero ──
    //
    // The OCXO has counted exactly 10,000,000 ticks since PPS zero.
    // In the OCXO's own domain, that is exactly 1,000,000,000 ns.
    // The GNSS measurement of this edge seeds the phase offset ledger.

    g_ocxo1_clock.ns_count_at_edge = NS_PER_SECOND_U64;
    g_ocxo1_clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)NS_PER_SECOND_U64;
    g_ocxo1_clock.zero_established = true;

    g_ocxo1_measurement.gnss_ns_between_edges = 0;
    g_ocxo1_measurement.dwt_cycles_between_edges = 0;
    g_ocxo1_measurement.second_residual_ns = 0;
  } else {
    // ── Subsequent one-second OCXO edges ──
    //
    // OCXO ns advances deterministically by 1e9 per OCXO second.
    // Phase offset is the cumulative GNSS-vs-OCXO difference.

    g_ocxo1_clock.ns_count_at_edge += NS_PER_SECOND_U64;
    g_ocxo1_clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)g_ocxo1_clock.ns_count_at_edge;

    const uint64_t gnss_ns_between_edges =
        gnss_ns_at_edge - g_ocxo1_measurement.prev_gnss_ns_at_edge;
    const uint32_t dwt_cycles_between_edges =
        dwt_at_edge - g_ocxo1_measurement.prev_dwt_at_edge;

    g_ocxo1_measurement.gnss_ns_between_edges = gnss_ns_between_edges;
    g_ocxo1_measurement.dwt_cycles_between_edges = dwt_cycles_between_edges;
    g_ocxo1_measurement.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;

    // Per-window viability audit: did this OCXO second approximate 1e9 ns?
    g_ocxo1_clock.window_expected_ns = (int64_t)NS_PER_SECOND_U64;
    g_ocxo1_clock.window_actual_ns = (int64_t)gnss_ns_between_edges;
    g_ocxo1_clock.window_error_ns =
        (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    g_ocxo1_clock.window_checks++;
    if (llabs(g_ocxo1_clock.window_error_ns) > OCXO_WINDOW_TOLERANCE_NS) {
      g_ocxo1_clock.window_mismatches++;
    }
  }

  g_ocxo1_measurement.prev_gnss_ns_at_edge = gnss_ns_at_edge;
  g_ocxo1_measurement.prev_dwt_at_edge = dwt_at_edge;
}

static void ocxo2_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  if (diag) g_ocxo2_interrupt_diag = *diag;
  else g_ocxo2_interrupt_diag = {};

  if (!alpha_ocxo_edges_are_canonical()) {
    return;
  }

  const uint32_t dwt_at_edge = event.dwt_at_event;
  const uint64_t gnss_ns_at_edge = event.gnss_ns_at_event;

  g_ocxo2_clock.gnss_ns_at_edge = gnss_ns_at_edge;
  g_ocxo2_measurement.dwt_at_edge = dwt_at_edge;

  if (g_ocxo2_measurement.prev_gnss_ns_at_edge == 0) {
    g_ocxo2_clock.ns_count_at_edge = NS_PER_SECOND_U64;
    g_ocxo2_clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)NS_PER_SECOND_U64;
    g_ocxo2_clock.zero_established = true;

    g_ocxo2_measurement.gnss_ns_between_edges = 0;
    g_ocxo2_measurement.dwt_cycles_between_edges = 0;
    g_ocxo2_measurement.second_residual_ns = 0;
  } else {
    g_ocxo2_clock.ns_count_at_edge += NS_PER_SECOND_U64;
    g_ocxo2_clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)g_ocxo2_clock.ns_count_at_edge;

    const uint64_t gnss_ns_between_edges =
        gnss_ns_at_edge - g_ocxo2_measurement.prev_gnss_ns_at_edge;
    const uint32_t dwt_cycles_between_edges =
        dwt_at_edge - g_ocxo2_measurement.prev_dwt_at_edge;

    g_ocxo2_measurement.gnss_ns_between_edges = gnss_ns_between_edges;
    g_ocxo2_measurement.dwt_cycles_between_edges = dwt_cycles_between_edges;
    g_ocxo2_measurement.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;

    g_ocxo2_clock.window_expected_ns = (int64_t)NS_PER_SECOND_U64;
    g_ocxo2_clock.window_actual_ns = (int64_t)gnss_ns_between_edges;
    g_ocxo2_clock.window_error_ns =
        (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    g_ocxo2_clock.window_checks++;
    if (llabs(g_ocxo2_clock.window_error_ns) > OCXO_WINDOW_TOLERANCE_NS) {
      g_ocxo2_clock.window_mismatches++;
    }
  }

  g_ocxo2_measurement.prev_gnss_ns_at_edge = gnss_ns_at_edge;
  g_ocxo2_measurement.prev_dwt_at_edge = dwt_at_edge;
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