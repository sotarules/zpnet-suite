// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// Dispatch doctrine:
//
//   GNSS clock state is VCLOCK-driven. The vclock_callback subscribes to the
//   VCLOCK integrator's synthetic 1-second boundary and advances all
//   canonical clock state (epoch install. Also, advances, g_gnss_ns_count_at_pps,
//   g_dwt_cycle_count_at_pps, g_qtimer_at_pps, time_pps_update, OCXO
//   phase ledger).  This is the low-latency, low-jitter path — DWT is
//   captured as the first instruction of the CH3 ISR and every
//   downstream value is ISR-captured fact or pure arithmetic.
//
//   TIMEBASE_FRAGMENT dispatch is driven by the physical PPS GPIO edge.
//   pps_edge_callback is registered with process_interrupt and fires
//   ~tens-of-microseconds after each physical edge (via timepop_arm_asap).
//   Its job is narrow: refresh the gnss_ns_at_isr diagnostic from the
//   pps_edge_snapshot_t and call clocks_beta_pps() to assemble and
//   publish the fragment.
//
//   Temporal ordering: within any given real-world second, VCLOCK
//   synthetic boundary N fires ~40 ms before physical PPS edge N (CH3
//   bootstrap phase offset).  So when the physical edge arrives and
//   triggers fragment dispatch, all clock state has just been advanced
//   by vclock_callback.  The fragment assembles from fresh, coherent
//   data and carries this PPS's gnss_ns_at_edge in the witness field.
//
//   OCXO subscribers (ocxo1_callback, ocxo2_callback) remain unchanged:
//   they fire on their own QTimer3 boundary events and update the OCXO
//   phase ledger independently.
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
#include <strings.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t PPS_RELAY_OFF_NS  = 500000000ULL;

// ============================================================================
// Always-on state
// ============================================================================

volatile uint64_t g_gnss_ns_count_at_pps = 0;

volatile uint32_t g_dwt_cycle_count_at_pps = 0;
volatile uint64_t g_dwt_cycle_count_total = 0;
volatile uint32_t g_dwt_cycle_count_between_pps = 0;
volatile uint32_t g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
volatile int32_t  g_dwt_cycle_count_next_second_adjustment = 0;  // vestigial
volatile uint64_t g_dwt_model_pps_count = 0;

volatile uint32_t g_qtimer_at_pps = 0;
volatile uint32_t g_last_pps_event_counter32_at_event = 0;

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

// ============================================================================
// Alpha epoch state
// ============================================================================

enum class clocks_epoch_reason_t : uint8_t {
  NONE  = 0,
  INIT  = 1,
  ZERO  = 2,
  START = 3,
};

static volatile bool g_epoch_pending = false;
static volatile bool g_epoch_initialized = false;
static volatile uint32_t g_epoch_generation = 0;
static volatile clocks_epoch_reason_t g_epoch_reason = clocks_epoch_reason_t::NONE;

static volatile uint32_t g_epoch_dwt_at_pps = 0;
static volatile uint32_t g_epoch_qtimer_at_pps = 0;
static volatile uint64_t g_epoch_pps_index = 0;

// ============================================================================
// DAC state
// ============================================================================

static ocxo_dac_state_t make_default_ocxo_dac_state() {
  ocxo_dac_state_t s = {};
  s.dac_fractional = (double)AD5693R_DAC_DEFAULT;
  s.dac_hw_code = AD5693R_DAC_DEFAULT;
  s.dac_min = 0;
  s.dac_max = 65535;
  s.io_last_write_ok = true;
  s.io_last_attempted_hw_code = AD5693R_DAC_DEFAULT;
  s.io_last_good_hw_code = AD5693R_DAC_DEFAULT;
  return s;
}

ocxo_dac_state_t ocxo1_dac = make_default_ocxo_dac_state();
ocxo_dac_state_t ocxo2_dac = make_default_ocxo_dac_state();

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
  if (!strcasecmp(s, "TOTAL")) return servo_mode_t::TOTAL;
  if (!strcasecmp(s, "NOW"))   return servo_mode_t::NOW;
  return servo_mode_t::OFF;
}

// Forward decls.
static bool ocxo_dac_write_hw(ocxo_dac_state_t& s, double value);

bool ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  return ocxo_dac_write_hw(s, value);
}

void ocxo_dac_predictor_reset(ocxo_dac_state_t& s) {
  s.servo_predictor_initialized = false;
  s.servo_last_raw_residual = 0.0;
  s.servo_filtered_residual = 0.0;
  s.servo_filtered_slope = 0.0;
  s.servo_predicted_residual = 0.0;
  s.servo_predictor_updates = 0;
}

void ocxo_dac_io_reset(ocxo_dac_state_t& s) {
  s.io_last_write_ok = true;
  s.io_fault_latched = false;
  s.io_last_failure_stage = 0;
}

void ocxo_dac_retry_reset(ocxo_dac_state_t& s) {
  s.io_last_write_ok = true;
  s.io_fault_latched = false;
  s.io_last_failure_stage = 0;
}

static bool ocxo_dac_write_hw(ocxo_dac_state_t& s, double value) {
  if (value < (double)s.dac_min) value = (double)s.dac_min;
  if (value > (double)s.dac_max) value = (double)s.dac_max;

  uint16_t hw_code = (uint16_t)(value + 0.5);
  s.io_last_attempted_hw_code = hw_code;

  if (hw_code == s.dac_hw_code) {
    s.dac_fractional = value;
    s.io_last_write_ok = true;
    s.io_last_failure_stage = 0;
    return true;
  }

  s.io_write_attempts++;

  if (!g_ad5693r_init_ok) {
    s.io_last_write_ok = false;
    s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 3;
    return false;
  }

  const uint8_t addr = (&s == &ocxo1_dac) ? AD5693R_ADDR_OCXO1 : AD5693R_ADDR_OCXO2;

  if (!ad5693r_write_input(addr, hw_code)) {
    s.io_last_write_ok = false;
    s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 1;
    return false;
  }

  if (!ad5693r_update_dac(addr)) {
    s.io_last_write_ok = false;
    s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 2;
    return false;
  }

  s.dac_fractional = value;
  s.dac_hw_code = hw_code;
  s.io_last_write_ok = true;
  s.io_write_successes++;
  s.io_last_good_hw_code = hw_code;
  s.io_last_failure_stage = 0;
  return true;
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
// DWT enable
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

static void alpha_install_new_epoch_from_vclock_boundary(
    const interrupt_event_t& vclock_event) {
  g_epoch_initialized = true;
  g_epoch_pending = false;
  g_epoch_generation++;
  g_epoch_pps_index = 0;

  g_epoch_dwt_at_pps = vclock_event.dwt_at_event;
  g_epoch_qtimer_at_pps = vclock_event.counter32_at_event;

  alpha_reset_canonical_clock_state_for_new_epoch();

  g_gnss_ns_count_at_pps = 0;
  g_dwt_cycle_count_at_pps = vclock_event.dwt_at_event;
  g_qtimer_at_pps = g_epoch_qtimer_at_pps;

  g_vclock_clock.counter32_at_pps_event = vclock_event.counter32_at_event;
  g_vclock_clock.counter32_at_pps_expected = vclock_event.counter32_at_event;
  g_vclock_clock.counter32_error_at_pps = 0;
  g_vclock_clock.ns_count_at_pps = 0;
  g_vclock_clock.ns_count_expected_at_pps = 0;
  g_vclock_clock.zero_established = true;

  g_vclock_measurement.ticks_between_pps = 0;
  g_vclock_measurement.ns_between_pps = 0;
  g_vclock_measurement.second_residual_ns = 0;
  g_vclock_measurement.prev_counter32_at_pps_event = vclock_event.counter32_at_event;

  // Establish the bridge anchor.  From this moment, time_dwt_to_gnss_ns()
  // returns valid GNSS nanoseconds for any DWT in the recent past or future,
  // including the OCXO integrator's synthetic boundaries and the physical
  // PPS GPIO edge captured in the GPIO ISR.
  time_pps_epoch_reset(vclock_event.dwt_at_event, g_epoch_qtimer_at_pps);
}

// ============================================================================
// VCLOCK subscriber — driven by VCLOCK synthetic boundary (QTimer1 CH3)
// ============================================================================
//
// This is the canonical GNSS clock advancement callback.  It fires on
// every VCLOCK synthetic 1-second boundary (every 1,008,000 ticks of
// CH3 counting GNSS 10 MHz) and advances all clock state accordingly.
//
// It does NOT publish TIMEBASE_FRAGMENT.  Fragment dispatch is triggered
// by the physical PPS GPIO edge via pps_edge_callback (see below).
//

static void vclock_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  if (diag) g_pps_interrupt_diag = *diag;
  else      g_pps_interrupt_diag = {};

  g_last_pps_event_counter32_at_event = event.counter32_at_event;

  // ── Epoch install (or re-install on request_zero) ──
  // The first VCLOCK boundary always lands here because the bridge cannot
  // be valid until we install the epoch.  After install, the bridge IS
  // valid, and subsequent boundaries flow through the "normal advance"
  // path with bridge-translated gnss_ns_at_event values.
  if (!g_epoch_initialized || g_epoch_pending) {
    alpha_install_new_epoch_from_vclock_boundary(event);

    digitalWriteFast(GNSS_PPS_RELAY, HIGH);
    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
    return;
  }

  // ── Normal advance ──
  // event.gnss_ns_at_event is the bridge-translated GNSS ns at this
  // synthetic boundary.  We use it to advance the canonical PPS clock.
  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps = event.gnss_ns_at_event;
  g_dwt_cycle_count_at_pps = event.dwt_at_event;
  g_qtimer_at_pps = event.counter32_at_event;

  // VCLOCK bookkeeping — by construction one synthetic second elapsed.
  g_vclock_measurement.ticks_between_pps = TICKS_10MHZ_PER_SECOND;
  g_vclock_measurement.ns_between_pps = NS_PER_SECOND_U64;
  g_vclock_measurement.second_residual_ns = 0;
  g_vclock_clock.counter32_at_pps_event = event.counter32_at_event;
  g_vclock_clock.counter32_at_pps_expected = event.counter32_at_event;
  g_vclock_clock.counter32_error_at_pps = 0;
  g_vclock_clock.ns_count_at_pps = event.gnss_ns_at_event;
  g_vclock_clock.ns_count_expected_at_pps += NS_PER_SECOND_U64;
  g_vclock_measurement.prev_counter32_at_pps_event = event.counter32_at_event;

  // DWT cycles between consecutive synthetic boundaries — sourced from
  // the integrator's SMA-derived avg cycles per second.
  const uint64_t integrator_cps = interrupt_vclock_cycles_per_second();
  if (integrator_cps > 0 && integrator_cps < (uint64_t)UINT32_MAX) {
    g_dwt_cycle_count_between_pps = (uint32_t)integrator_cps;
    g_dwt_cycle_count_next_second_prediction = (uint32_t)integrator_cps;
  } else {
    g_dwt_cycle_count_between_pps = DWT_EXPECTED_PER_PPS;
    g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
  }
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_cycle_count_total += (uint64_t)g_dwt_cycle_count_between_pps;
  g_dwt_model_pps_count++;

  // ── GPIO PPS witness → residual_vclock ──
  // The witness offset is the difference between the physical GPIO PPS
  // edge's GNSS ns and the most recent synthetic boundary's GNSS ns.
  // Under the new doctrine, gpio_minus_synthetic_ns is vestigial (always
  // zero) — the residual channel will be retired in a follow-up commit.
  if (g_pps_interrupt_diag.gpio_edge_count > 0 &&
      g_pps_interrupt_diag.gnss_ns_at_isr > 0) {
    residual_update_sample(residual_vclock,
                           g_pps_interrupt_diag.gpio_minus_synthetic_ns);
  }

  // ── OCXO ns projection to PPS boundary via phase ledger ──
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

  // Update the time bridge with our new authoritative anchor.
  time_pps_update(g_dwt_cycle_count_at_pps,
                  dwt_effective_cycles_per_second(),
                  g_qtimer_at_pps);

  if (request_zero && !g_epoch_pending) {
    alpha_request_epoch_zero(clocks_epoch_reason_t::ZERO);
  }

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }

  // NOTE: clocks_beta_pps() is NOT called here.  TIMEBASE_FRAGMENT
  // dispatch is triggered by the physical PPS GPIO edge via
  // pps_edge_callback.  By the time that callback fires (~40 ms after
  // this vclock_callback completes), all the state advanced above is
  // in place and the fragment can assemble from fresh data.
}

// ============================================================================
// Physical PPS edge callback — drives TIMEBASE_FRAGMENT dispatch
// ============================================================================
//
// Fires once per physical PPS rising edge via timepop_arm_asap-deferred
// dispatch from the GPIO ISR.  Narrow responsibilities:
//
//   1. Stash snap.gnss_ns_at_edge into the diag's gnss_ns_at_isr field
//      so the outgoing TIMEBASE_FRAGMENT carries the canonical "when
//      did the physical PPS edge occur" diagnostic for this PPS.
//
//   2. Call clocks_beta_pps() to assemble and publish the fragment.
//      clocks_beta_pps reads from VCLOCK-maintained clock state (which
//      was just advanced ~40 ms ago by vclock_callback) plus the diag
//      we just refreshed.
//
// Campaign gating (start / stop / recover / zero) is handled by
// clocks_beta_pps itself, same as before.
//

static void pps_edge_callback(const pps_edge_snapshot_t& snap) {
  // Refresh the diagnostic that will ride in the outgoing fragment.
  // The VCLOCK subscriber's diag was populated with a stale witness
  // value (from the previous edge that fired before vclock_callback);
  // overwrite with the authoritative snapshot of THIS edge.
  if (snap.gnss_ns_at_edge >= 0) {
    g_pps_interrupt_diag.gnss_ns_at_isr = snap.gnss_ns_at_edge;
  }
  g_pps_interrupt_diag.gpio_last_dwt = snap.dwt_at_edge;
  g_pps_interrupt_diag.gpio_edge_count = snap.sequence;

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }
}

// ============================================================================
// OCXO subscribers — driven by per-OCXO rolling integrator boundaries
// ============================================================================
//
// event.dwt_at_event       — synthetic DWT at the OCXO's Nth one-second edge
// event.gnss_ns_at_event   — bridge-translated GNSS ns at that same edge
// event.counter32_at_event — software-extended logical OCXO tick count
//
// Alpha trusts gnss_ns_at_event directly.  No override.
//

static void ocxo_apply_edge(ocxo_clock_state_t& clock,
                            ocxo_measurement_t& meas,
                            uint32_t dwt_at_edge,
                            uint64_t gnss_ns_at_edge) {

  clock.gnss_ns_at_edge = gnss_ns_at_edge;
  meas.dwt_at_edge = dwt_at_edge;

  if (meas.prev_gnss_ns_at_edge == 0) {
    // First post-epoch OCXO edge: seed the phase offset ledger.
    clock.ns_count_at_edge = NS_PER_SECOND_U64;
    clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)NS_PER_SECOND_U64;
    clock.zero_established = true;

    meas.gnss_ns_between_edges = 0;
    meas.dwt_cycles_between_edges = 0;
    meas.second_residual_ns = 0;
  } else {
    clock.ns_count_at_edge += NS_PER_SECOND_U64;
    clock.phase_offset_ns =
        (int64_t)gnss_ns_at_edge - (int64_t)clock.ns_count_at_edge;

    const uint64_t gnss_ns_between_edges =
        gnss_ns_at_edge - meas.prev_gnss_ns_at_edge;
    const uint32_t dwt_cycles_between_edges =
        dwt_at_edge - meas.prev_dwt_at_edge;

    meas.gnss_ns_between_edges = gnss_ns_between_edges;
    meas.dwt_cycles_between_edges = dwt_cycles_between_edges;
    meas.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;

    clock.window_expected_ns = (int64_t)NS_PER_SECOND_U64;
    clock.window_actual_ns = (int64_t)gnss_ns_between_edges;
    clock.window_error_ns =
        (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    clock.window_checks++;
    if (llabs(clock.window_error_ns) > OCXO_WINDOW_TOLERANCE_NS) {
      clock.window_mismatches++;
    }
  }

  meas.prev_gnss_ns_at_edge = gnss_ns_at_edge;
  meas.prev_dwt_at_edge = dwt_at_edge;
}

static void ocxo1_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  if (diag) g_ocxo1_interrupt_diag = *diag;
  else      g_ocxo1_interrupt_diag = {};

  if (!alpha_ocxo_edges_are_canonical()) return;

  // Skip events whose gnss_ns_at_event is 0 (bridge wasn't valid at
  // emission time).  This can only happen for OCXO boundaries that
  // fired before alpha installed the epoch; should be rare and benign.
  if (event.gnss_ns_at_event == 0) return;

  ocxo_apply_edge(g_ocxo1_clock, g_ocxo1_measurement,
                  event.dwt_at_event, event.gnss_ns_at_event);
}

static void ocxo2_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  if (diag) g_ocxo2_interrupt_diag = *diag;
  else      g_ocxo2_interrupt_diag = {};

  if (!alpha_ocxo_edges_are_canonical()) return;
  if (event.gnss_ns_at_event == 0) return;

  ocxo_apply_edge(g_ocxo2_clock, g_ocxo2_measurement,
                  event.dwt_at_event, event.gnss_ns_at_event);
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

  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);
  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);

  (void)ocxo_dac_set(ocxo1_dac, (double)AD5693R_DAC_DEFAULT);
  (void)ocxo_dac_set(ocxo2_dac, (double)AD5693R_DAC_DEFAULT);

  pinMode(GNSS_LOCK_PIN, INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  // VCLOCK subscriber — drives GNSS clock state via synthetic boundary.
  interrupt_subscription_t vclock_sub {};
  vclock_sub.kind = interrupt_subscriber_kind_t::VCLOCK;
  vclock_sub.on_event = vclock_callback;
  vclock_sub.user_data = nullptr;
  interrupt_subscribe(vclock_sub);
  interrupt_start(interrupt_subscriber_kind_t::VCLOCK);

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

  // Physical PPS GPIO edge — drives TIMEBASE_FRAGMENT dispatch only.
  interrupt_pps_edge_register_dispatch(pps_edge_callback);

  // The 10 kHz dwt_adjustment_timer is RETIRED.  The VCLOCK rolling
  // integrator's avg_cycles_per_second is now the prediction source.
}