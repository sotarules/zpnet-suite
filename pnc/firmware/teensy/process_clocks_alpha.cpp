// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer (PPS-anchored)
// ============================================================================
//
// Doctrine:
//
//   GNSS clock state is PPS-authored, VCLOCK-cadenced.
//
//   Alpha subscribes to four kinds of events from process_interrupt:
//     PPS_VCLOCK — the canonical PPS edge.  Sequence × 1e9 = gnss_ns.
//     VCLOCK     — the GNSS-locked 10 MHz reference, every second.
//     OCXO1      — OCXO1's "second has elapsed" signal.
//     OCXO2      — OCXO2's "second has elapsed" signal.
//
//   Each event carries {gnss_ns_at_edge, dwt_at_edge, counter32_at_edge,
//   sequence} — that's the entire payload.
//
//   The pps_vclock_callback is the canonical state author:
//     • On rebootstrap: install the new epoch from the event verbatim,
//       and zero the synthetic counter32s in tandem with the ns counters.
//     • Otherwise: compute dwt_between_pps = current.dwt - prev.dwt.
//       This IS the rate.  Publish to the seqlock-protected PPS_VCLOCK
//       slot for time.cpp + TimePop consumption.
//
//   The vclock_callback handles per-second concerns:
//     • Advance g_gnss_ns_count_at_pps by exactly +1e9.
//     • Run the VCLOCK-as-peer-clock measurement via clocks_apply_edge.
//     • Refresh OCXO phase ledgers.
//
//   The OCXO callbacks run clocks_apply_edge for OCXO1/OCXO2.  Their
//   gnss_ns_at_edge is bridge-projected by process_interrupt's ISR
//   (one bridge, one rate, one truth).
//
//   Sign convention: positive second_residual_ns → clock fired EARLY
//   (running fast).  Same across all clocks.
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

static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t PPS_RELAY_OFF_NS  = 500000000ULL;

static_assert(NS_PER_SECOND_U64 ==
              (uint64_t)VCLOCK_COUNTS_PER_SECOND * 100ULL,
              "VCLOCK pulse identity broken: NS_PER_SECOND_U64 != "
              "VCLOCK_COUNTS_PER_SECOND * 100");

// ============================================================================
// Published canonical state (alpha-owned, beta-readable)
// ============================================================================

volatile uint64_t g_gnss_ns_count_at_pps         = 0;
volatile uint32_t g_dwt_cycle_count_at_pps       = 0;
volatile uint64_t g_dwt_cycle_count_total        = 0;
volatile uint32_t g_dwt_cycle_count_between_pps  = DWT_EXPECTED_PER_PPS;
volatile int32_t  g_dwt_prediction_residual_cycles = 0;

clock_state_t       g_vclock_clock        = {};
clock_measurement_t g_vclock_measurement  = {};

clock_state_t       g_ocxo1_clock         = {};
clock_state_t       g_ocxo2_clock         = {};
clock_measurement_t g_ocxo1_measurement   = {};
clock_measurement_t g_ocxo2_measurement   = {};

interrupt_event_t g_last_pps_vclock_event = {};
interrupt_event_t g_last_vclock_event     = {};
interrupt_event_t g_last_ocxo1_event      = {};
interrupt_event_t g_last_ocxo2_event      = {};

bool g_ad5693r_init_ok = false;

// ============================================================================
// PPS_VCLOCK slot — alpha-authored, time.cpp + TimePop consumed
// ============================================================================
//
// Seqlock-protected.  Single foreground writer (pps_vclock_callback).
// Multiple readers, including TimePop's TICK callback context.

static volatile uint32_t g_alpha_slot_seq               = 0;
static volatile int64_t  g_alpha_slot_gnss_ns           = 0;
static volatile uint32_t g_alpha_slot_dwt_at_edge       = 0;
static volatile uint32_t g_alpha_slot_counter32_at_edge = 0;
static volatile uint32_t g_alpha_slot_sequence          = 0;

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static void alpha_pps_vclock_publish(int64_t  gnss_ns,
                                     uint32_t dwt_at_edge,
                                     uint32_t counter32_at_edge,
                                     uint32_t sequence) {
  g_alpha_slot_seq++;
  dmb_barrier();
  g_alpha_slot_gnss_ns           = gnss_ns;
  g_alpha_slot_dwt_at_edge       = dwt_at_edge;
  g_alpha_slot_counter32_at_edge = counter32_at_edge;
  g_alpha_slot_sequence          = sequence;
  dmb_barrier();
  g_alpha_slot_seq++;
}

alpha_pps_vclock_slot_t alpha_pps_vclock_load(void) {
  alpha_pps_vclock_slot_t out{};
  for (int i = 0; i < 4; i++) {
    const uint32_t s1 = g_alpha_slot_seq;
    dmb_barrier();
    out.gnss_ns_at_edge   = g_alpha_slot_gnss_ns;
    out.dwt_at_edge       = g_alpha_slot_dwt_at_edge;
    out.counter32_at_edge = g_alpha_slot_counter32_at_edge;
    out.sequence          = g_alpha_slot_sequence;
    dmb_barrier();
    const uint32_t s2 = g_alpha_slot_seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }
  return alpha_pps_vclock_slot_t{};   // sequence == 0 → invalid
}

uint32_t alpha_dwt_cycles_per_second(void) {
  return g_dwt_cycle_count_between_pps;
}

// ============================================================================
// Epoch state
// ============================================================================

enum class clocks_epoch_reason_t : uint8_t {
  NONE  = 0,
  INIT  = 1,
  ZERO  = 2,
  START = 3,
};

static volatile bool g_epoch_pending     = false;
static volatile bool g_epoch_initialized = false;
static volatile uint32_t g_epoch_generation = 0;
static volatile clocks_epoch_reason_t g_epoch_reason = clocks_epoch_reason_t::NONE;

static volatile uint32_t g_epoch_dwt_at_pps        = 0;
static volatile uint64_t g_epoch_pps_index         = 0;
static volatile uint32_t g_epoch_pps_edge_sequence = 0;

static volatile uint32_t g_prev_dwt_at_vclock_event = 0;

static volatile uint32_t g_prev_pps_dwt_at_edge       = 0;
static volatile bool     g_prev_pps_dwt_at_edge_valid = false;

static volatile uint32_t g_prev_dwt_between_pps       = 0;
static volatile bool     g_prev_dwt_between_pps_valid = false;

static volatile uint32_t g_vclock_event_count = 0;

// ============================================================================
// OCXO DAC defaults
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

  const uint8_t addr = (&s == &ocxo1_dac) ?
      AD5693R_ADDR_OCXO1 : AD5693R_ADDR_OCXO2;

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
// Beta-facing shadow counts
// ============================================================================

uint64_t dwt_cycle_count_total = 0;
uint64_t gnss_raw_64           = 0;
uint64_t ocxo1_ticks_64        = 0;
uint64_t ocxo2_ticks_64        = 0;

// ============================================================================
// PPS relay
// ============================================================================

static volatile bool relay_timer_active = false;

static void pps_relay_deassert(timepop_ctx_t*, timepop_diag_t*, void*) {
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  relay_timer_active = false;
}

static inline void pps_relay_pulse(void) {
  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }
}

// ============================================================================
// DWT enable + epoch helpers
// ============================================================================

static inline void dwt_enable(void) {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

static void alpha_request_epoch_zero(clocks_epoch_reason_t reason) {
  g_epoch_pending = true;
  g_epoch_reason  = reason;
  interrupt_request_pps_rebootstrap();
}

bool clocks_epoch_pending(void) {
  return g_epoch_pending;
}

static inline bool alpha_ocxo_edges_are_canonical(void) {
  return g_epoch_initialized && !g_epoch_pending;
}

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  g_gnss_ns_count_at_pps          = 0;
  g_dwt_cycle_count_at_pps        = 0;
  g_dwt_cycle_count_total         = 0;
  g_dwt_cycle_count_between_pps   = DWT_EXPECTED_PER_PPS;
  g_prev_dwt_at_vclock_event      = 0;
  g_prev_pps_dwt_at_edge          = 0;
  g_prev_pps_dwt_at_edge_valid    = false;
  g_prev_dwt_between_pps          = 0;
  g_prev_dwt_between_pps_valid    = false;
  g_dwt_prediction_residual_cycles = 0;
  g_vclock_event_count            = 0;

  g_vclock_clock        = {};
  g_vclock_measurement  = {};
  g_ocxo1_clock         = {};
  g_ocxo2_clock         = {};
  g_ocxo1_measurement   = {};
  g_ocxo2_measurement   = {};

  g_last_pps_vclock_event = {};
  g_last_vclock_event     = {};
  g_last_ocxo1_event      = {};
  g_last_ocxo2_event      = {};

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);

  // Zero the three synthetic 32-bit counters in tandem with the ns
  // counters above.  Process_interrupt owns them; alpha invokes the
  // single zero-method privilege.
  interrupt_synthetic_counters_zero();
}

// ============================================================================
// Epoch install — PPS-anchored, ONE entry point
// ============================================================================

static void alpha_install_new_epoch_from_pps_event(const interrupt_event_t& event) {
  alpha_reset_canonical_clock_state_for_new_epoch();

  g_epoch_dwt_at_pps        = event.dwt_at_edge;
  g_epoch_pps_edge_sequence = event.sequence;
  g_epoch_pps_index         = 0;

  g_dwt_cycle_count_at_pps     = event.dwt_at_edge;
  g_prev_dwt_at_vclock_event   = event.dwt_at_edge;
  g_prev_pps_dwt_at_edge       = event.dwt_at_edge;
  g_prev_pps_dwt_at_edge_valid = true;

  g_epoch_initialized = true;
  g_epoch_pending     = false;
  g_epoch_generation++;
}

// ============================================================================
// Per-edge measurement — one body, used by VCLOCK and both OCXOs
// ============================================================================

static void clocks_apply_edge(clock_state_t& clock,
                              clock_measurement_t& meas,
                              uint32_t dwt_at_edge,
                              uint64_t gnss_ns_at_edge) {

  clock.gnss_ns_at_edge = gnss_ns_at_edge;
  meas.dwt_at_edge      = dwt_at_edge;

  if (meas.prev_gnss_ns_at_edge == 0) {
    clock.ns_count_at_edge  = NS_PER_SECOND_U64;
    clock.phase_offset_ns   =
        (int64_t)gnss_ns_at_edge - (int64_t)NS_PER_SECOND_U64;
    clock.zero_established  = true;

    meas.gnss_ns_between_edges    = 0;
    meas.dwt_cycles_between_edges = 0;
    meas.second_residual_ns       = 0;
  } else {
    clock.ns_count_at_edge += NS_PER_SECOND_U64;
    clock.phase_offset_ns   =
        (int64_t)gnss_ns_at_edge - (int64_t)clock.ns_count_at_edge;

    const uint64_t gnss_ns_between_edges =
        gnss_ns_at_edge - meas.prev_gnss_ns_at_edge;
    const uint32_t dwt_cycles_between_edges =
        dwt_at_edge - meas.prev_dwt_at_edge;

    meas.gnss_ns_between_edges    = gnss_ns_between_edges;
    meas.dwt_cycles_between_edges = dwt_cycles_between_edges;

    meas.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;

    clock.window_error_ns =
        (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    clock.window_checks++;
    if (llabs(clock.window_error_ns) > CLOCK_WINDOW_TOLERANCE_NS) {
      clock.window_mismatches++;
    }
  }

  meas.prev_gnss_ns_at_edge = gnss_ns_at_edge;
  meas.prev_dwt_at_edge     = dwt_at_edge;
}

// ============================================================================
// VCLOCK callback — pure per-second advancer
// ============================================================================

static void vclock_callback(const interrupt_event_t& event, void*) {
  g_last_vclock_event = event;

  if (!g_epoch_initialized || g_epoch_pending) return;

  g_vclock_event_count++;

  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps += NS_PER_SECOND_U64;

  g_prev_dwt_at_vclock_event = event.dwt_at_edge;

  const uint64_t gnss_ns =
      (event.gnss_ns_at_edge >= 0) ? (uint64_t)event.gnss_ns_at_edge : 0;
  if (gnss_ns != 0) {
    clocks_apply_edge(g_vclock_clock, g_vclock_measurement,
                      event.dwt_at_edge, gnss_ns);
  }

  g_vclock_clock.ns_count_at_pps = g_gnss_ns_count_at_pps;

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
}

// ============================================================================
// PPS_VCLOCK callback — canonical state author
// ============================================================================

static void pps_vclock_callback(const interrupt_event_t& event, void*) {
  g_last_pps_vclock_event = event;

  // Step 1: arm a future rebootstrap if a request is outstanding.
  if ((request_zero || request_start) && !g_epoch_pending) {
    alpha_request_epoch_zero(request_start
                               ? clocks_epoch_reason_t::START
                               : clocks_epoch_reason_t::ZERO);
  }

  // Step 2: install the epoch from this event if pending.  On install,
  // skip the bookend block (the install edge is the start of rate
  // measurement, not the end), publish the slot at install gnss_ns=0,
  // hand to beta, and return.
  if (g_epoch_pending && !interrupt_pps_rebootstrap_pending()) {
    alpha_install_new_epoch_from_pps_event(event);
    alpha_pps_vclock_publish(event.gnss_ns_at_edge,
                             event.dwt_at_edge,
                             event.counter32_at_edge,
                             event.sequence);
    pps_relay_pulse();
    if (campaign_state == clocks_campaign_state_t::STARTED ||
        request_start || request_stop || request_recover || request_zero) {
      clocks_beta_pps();
    }
    return;
  }

  // Step 3: bookend rate.
  if (g_epoch_initialized && !g_epoch_pending && g_prev_pps_dwt_at_edge_valid) {
    const uint32_t dwt_between_pps = event.dwt_at_edge - g_prev_pps_dwt_at_edge;

    if (g_prev_dwt_between_pps_valid) {
      g_dwt_prediction_residual_cycles =
          (int32_t)((int64_t)dwt_between_pps - (int64_t)g_prev_dwt_between_pps);
    } else {
      g_dwt_prediction_residual_cycles = 0;
    }
    g_prev_dwt_between_pps       = dwt_between_pps;
    g_prev_dwt_between_pps_valid = true;

    g_dwt_cycle_count_between_pps = dwt_between_pps;
    g_dwt_cycle_count_total      += (uint64_t)dwt_between_pps;
    g_prev_pps_dwt_at_edge        = event.dwt_at_edge;
    g_dwt_cycle_count_at_pps      = event.dwt_at_edge;
  }

  // Step 4: publish the PPS_VCLOCK slot — for time.cpp's bridge AND
  // TimePop's deadline arming.
  alpha_pps_vclock_publish(event.gnss_ns_at_edge,
                           event.dwt_at_edge,
                           event.counter32_at_edge,
                           event.sequence);

  // Step 5: hand to beta.
  pps_relay_pulse();

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }
}

// ============================================================================
// OCXO callbacks — apply edge via shared template
// ============================================================================

static void ocxo1_callback(const interrupt_event_t& event, void*) {
  g_last_ocxo1_event = event;

  if (!alpha_ocxo_edges_are_canonical()) return;
  if (event.gnss_ns_at_edge < 0)         return;

  clocks_apply_edge(g_ocxo1_clock, g_ocxo1_measurement,
                    event.dwt_at_edge, (uint64_t)event.gnss_ns_at_edge);
}

static void ocxo2_callback(const interrupt_event_t& event, void*) {
  g_last_ocxo2_event = event;

  if (!alpha_ocxo_edges_are_canonical()) return;
  if (event.gnss_ns_at_edge < 0)         return;

  clocks_apply_edge(g_ocxo2_clock, g_ocxo2_measurement,
                    event.dwt_at_edge, (uint64_t)event.gnss_ns_at_edge);
}

// ============================================================================
// Init
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
}

void process_clocks_init(void) {
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

  interrupt_subscription_t pps_sub {};
  pps_sub.kind     = interrupt_subscriber_kind_t::PPS_VCLOCK;
  pps_sub.on_event = pps_vclock_callback;
  interrupt_subscribe(pps_sub);
  interrupt_start(interrupt_subscriber_kind_t::PPS_VCLOCK);

  interrupt_subscription_t vclock_sub {};
  vclock_sub.kind     = interrupt_subscriber_kind_t::VCLOCK;
  vclock_sub.on_event = vclock_callback;
  interrupt_subscribe(vclock_sub);
  interrupt_start(interrupt_subscriber_kind_t::VCLOCK);

  interrupt_subscription_t ocxo1_sub {};
  ocxo1_sub.kind     = interrupt_subscriber_kind_t::OCXO1;
  ocxo1_sub.on_event = ocxo1_callback;
  interrupt_subscribe(ocxo1_sub);
  interrupt_start(interrupt_subscriber_kind_t::OCXO1);

  interrupt_subscription_t ocxo2_sub {};
  ocxo2_sub.kind     = interrupt_subscriber_kind_t::OCXO2;
  ocxo2_sub.on_event = ocxo2_callback;
  interrupt_subscribe(ocxo2_sub);
  interrupt_start(interrupt_subscriber_kind_t::OCXO2);
}