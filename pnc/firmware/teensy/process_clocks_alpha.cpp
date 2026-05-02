// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// Alpha consumes process_interrupt event facts directly.  The subscription
// contract is intentionally small: GNSS ns at the edge, latency-adjusted DWT
// at the edge, synthetic counter32 at the edge, and an optional diagnostic
// mirror.  No slot staging, no installed event wrappers, no late hardware
// reads masquerading as event truth.
//
// PPS is a selector/witness. process_interrupt authors the selected PPS/VCLOCK
// edge, and alpha treats that selected VCLOCK edge as the canonical epoch and
// DWT-GNSS bridge anchor. VCLOCK/OCXO callbacks own per-edge clock measurements.
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_interrupt.h"
#include "process_epoch.h"

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

volatile uint64_t g_gnss_ns_at_pps_vclock = 0;

volatile uint32_t g_dwt_at_pps_vclock = 0;
volatile uint64_t g_dwt_cycle_count_total = 0;
volatile uint32_t g_dwt_cycles_between_pps_vclock = DWT_EXPECTED_PER_PPS;

// Alpha-owned DWT64 logical clock.  This is the one true DWT64 clock surface.
// It is not physically writable; process_epoch zeros it by installing an epoch
// origin in this raw extended ledger.  The raw ledger itself is anchored by
// DWT32 observations and advanced at each PPS/VCLOCK bookend.
static volatile bool     g_dwt64_raw_anchor_valid = false;
static volatile uint64_t g_dwt64_raw_cycles_at_anchor = 0;
static volatile uint32_t g_dwt64_dwt32_at_anchor = 0;
static volatile bool     g_dwt64_epoch_valid = false;
static volatile uint64_t g_dwt64_epoch_raw_cycles = 0;
static volatile uint32_t g_dwt64_epoch_reset_count = 0;
static volatile uint32_t g_dwt64_epoch_reset_failures = 0;
static volatile bool     g_dwt_calibration_valid = false;

volatile uint32_t g_counter32_at_pps_vclock = 0;
volatile uint32_t g_last_vclock_event_counter32_at_event = 0;

clock_state_t       g_vclock_clock = {};
clock_measurement_t g_vclock_measurement = {};

clock_state_t       g_ocxo1_clock = {};
clock_state_t       g_ocxo2_clock = {};

clock_measurement_t g_ocxo1_measurement = {};
clock_measurement_t g_ocxo2_measurement = {};

interrupt_capture_diag_t g_pps_witness_diag = {};
interrupt_capture_diag_t g_ocxo1_interrupt_diag = {};
interrupt_capture_diag_t g_ocxo2_interrupt_diag = {};

bool g_ad5693r_init_ok = false;

// ============================================================================
// Epoch state
// ============================================================================

static volatile bool g_epoch_initialized = false;

// Last VCLOCK event DWT, retained only for PPS-vs-VCLOCK phase diagnostics.
static volatile uint32_t g_prev_dwt_at_vclock_event = 0;

// ── Canonical one-second subtraction state for DWT cycles between PPS/VCLOCK anchors ──
//
// Authored by pps_selector_callback from process_interrupt's PPS/VCLOCK
// snapshot.  PPS remains the selector/witness, but the public PPS/VCLOCK DWT
// coordinate is now authored by the VCLOCK/QTimer1 CH3 path, so consecutive
// snap.dwt_at_edge values live in the same event-coordinate species as the
// 1 kHz VCLOCK cadence samples.
static volatile uint32_t g_prev_pps_vclock_dwt_at_edge = 0;
static volatile bool     g_prev_pps_vclock_dwt_at_edge_valid = false;

// VCLOCK event counter — increments on every vclock_callback invocation
// AFTER epoch install.  Read by pps_selector_callback to cross-check that no
// VCLOCK event slipped in between the GPIO ISR and the foreground
// pps_selector_callback.
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
// Alpha-owned DWT64 logical clock
// ============================================================================

static inline uint64_t dwt64_raw_now_unlocked(void) {
  if (!g_dwt64_raw_anchor_valid) {
    return (uint64_t)DWT_CYCCNT;
  }
  return g_dwt64_raw_cycles_at_anchor +
         (uint64_t)((uint32_t)(DWT_CYCCNT - g_dwt64_dwt32_at_anchor));
}

static inline uint64_t dwt64_logical_now_unlocked(void) {
  const uint64_t raw_now = dwt64_raw_now_unlocked();
  return g_dwt64_epoch_valid
      ? (raw_now - g_dwt64_epoch_raw_cycles)
      : raw_now;
}

static void dwt64_anchor_reset_to_dwt32(uint32_t dwt32_at_anchor,
                                        uint64_t raw_cycles_at_anchor) {
  g_dwt64_raw_cycles_at_anchor = raw_cycles_at_anchor;
  g_dwt64_dwt32_at_anchor = dwt32_at_anchor;
  g_dwt64_raw_anchor_valid = true;
}

static void dwt64_anchor_advance_to_dwt32(uint32_t dwt32_at_anchor) {
  if (!g_dwt64_raw_anchor_valid) {
    dwt64_anchor_reset_to_dwt32(dwt32_at_anchor, (uint64_t)dwt32_at_anchor);
    return;
  }

  const uint32_t delta = dwt32_at_anchor - g_dwt64_dwt32_at_anchor;
  g_dwt64_raw_cycles_at_anchor += (uint64_t)delta;
  g_dwt64_dwt32_at_anchor = dwt32_at_anchor;
}

bool clocks_dwt64_epoch_reset_at_dwt32(uint32_t epoch_dwt32,
                                       uint64_t* out_raw_epoch_dwt64) {
  if (!g_dwt64_raw_anchor_valid) {
    dwt64_anchor_reset_to_dwt32(epoch_dwt32, (uint64_t)epoch_dwt32);
  }

  const uint64_t raw_epoch =
      g_dwt64_raw_cycles_at_anchor +
      (uint64_t)((uint32_t)(epoch_dwt32 - g_dwt64_dwt32_at_anchor));

  // Re-anchor the raw DWT64 ledger exactly at the epoch edge.  This makes
  // immediate post-epoch reads cheap and removes dependence on the previous
  // epoch's anchor after ZERO.
  dwt64_anchor_reset_to_dwt32(epoch_dwt32, raw_epoch);

  g_dwt64_epoch_raw_cycles = raw_epoch;
  g_dwt64_epoch_valid = true;
  g_dwt64_epoch_reset_count++;

  if (out_raw_epoch_dwt64) {
    *out_raw_epoch_dwt64 = raw_epoch;
  }
  return true;
}

uint64_t clocks_dwt_cycles_now(void) {
  return dwt64_logical_now_unlocked();
}


uint32_t clocks_dwt_cycles_per_gnss_second(void) {
  return g_dwt_calibration_valid ? g_dwt_cycles_between_pps_vclock : 0;
}

bool clocks_dwt_calibration_valid(void) {
  return g_dwt_calibration_valid;
}

uint64_t clocks_gnss_ns_now(void) {
  const int64_t ns = time_gnss_ns_now();
  return (ns >= 0) ? (uint64_t)ns : g_gnss_ns_at_pps_vclock;
}

uint64_t clocks_gnss_ticks_now(void) {
  return clocks_gnss_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
}

uint64_t clocks_ocxo1_ns_now(void) {
  return g_ocxo1_clock.ns_count_at_pps_vclock;
}

uint64_t clocks_ocxo1_ticks_now(void) {
  return clocks_ocxo1_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
}

uint64_t clocks_ocxo2_ns_now(void) {
  return g_ocxo2_clock.ns_count_at_pps_vclock;
}

uint64_t clocks_ocxo2_ticks_now(void) {
  return clocks_ocxo2_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
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

  g_dwt64_raw_anchor_valid = true;
  g_dwt64_raw_cycles_at_anchor = 0;
  g_dwt64_dwt32_at_anchor = DWT_CYCCNT;
  g_dwt64_epoch_valid = false;
  g_dwt64_epoch_raw_cycles = 0;
  g_dwt64_epoch_reset_count = 0;
  g_dwt64_epoch_reset_failures = 0;
  g_dwt_calibration_valid = false;
}

bool clocks_epoch_pending(void) {
  return process_epoch_pending();
}

static inline bool epoch_ready(void) {
  return g_epoch_initialized && process_epoch_finalized();
}

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  g_gnss_ns_at_pps_vclock = 0;
  g_dwt_at_pps_vclock = 0;
  g_dwt_cycle_count_total = 0;
  g_dwt_cycles_between_pps_vclock = DWT_EXPECTED_PER_PPS;
  g_dwt_calibration_valid = false;
  g_counter32_at_pps_vclock = 0;
  g_prev_dwt_at_vclock_event = 0;
  g_prev_pps_vclock_dwt_at_edge = 0;
  g_prev_pps_vclock_dwt_at_edge_valid = false;
  g_vclock_event_count = 0;

  g_vclock_clock = {};
  g_vclock_measurement = {};
  g_ocxo1_clock = {};
  g_ocxo2_clock = {};
  g_ocxo1_measurement = {};
  g_ocxo2_measurement = {};
  g_pps_witness_diag = {};
  g_ocxo1_interrupt_diag = {};
  g_ocxo2_interrupt_diag = {};

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_ticks_64        = 0;
  ocxo2_ticks_64        = 0;

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
}

// ============================================================================
// Epoch install
// ============================================================================
// The PPS snapshot projects the canonical PPS/VCLOCK edge selected by
// process_interrupt.  Alpha consumes it directly and seeds the DWT/GNSS bridge.
// ============================================================================

void clocks_alpha_epoch_installed(const epoch_fact_t& fact) {
  alpha_reset_canonical_clock_state_for_new_epoch();

  g_gnss_ns_at_pps_vclock = fact.epoch_ns;
  g_dwt_at_pps_vclock = fact.epoch_dwt;
  g_counter32_at_pps_vclock = fact.vclock_counter32_at_epoch;
  g_prev_dwt_at_vclock_event = fact.epoch_dwt;
  g_prev_pps_vclock_dwt_at_edge = fact.epoch_dwt;
  g_prev_pps_vclock_dwt_at_edge_valid = true;

  // process_epoch has already installed the DWT64 epoch origin.  Re-anchor
  // the raw DWT64 ledger at the same selected edge so the DWT64 logical clock
  // starts at exactly zero and extends continuously from this epoch.
  if (g_dwt64_epoch_valid) {
    dwt64_anchor_reset_to_dwt32(fact.epoch_dwt, g_dwt64_epoch_raw_cycles);
  }

  g_epoch_initialized = true;
}

// ============================================================================
// Direct event application
// ============================================================================

static void clocks_apply_edge(clock_state_t& clock,
                              clock_measurement_t& meas,
                              const interrupt_event_t& event) {
  clock.gnss_ns_at_edge = event.gnss_ns_at_event;
  meas.dwt_at_edge = event.dwt_at_event;

  if (meas.prev_gnss_ns_at_edge == 0) {
    clock.ns_count_at_edge = NS_PER_SECOND_U64;
    clock.phase_offset_ns =
        (int64_t)event.gnss_ns_at_event - (int64_t)NS_PER_SECOND_U64;
    clock.zero_established = true;

    meas.gnss_ns_between_edges = 0;
    meas.dwt_cycles_between_edges = 0;
    meas.second_residual_ns = 0;
  } else {
    clock.ns_count_at_edge += NS_PER_SECOND_U64;
    clock.phase_offset_ns =
        (int64_t)event.gnss_ns_at_event - (int64_t)clock.ns_count_at_edge;

    const uint64_t gnss_ns_between_edges =
        event.gnss_ns_at_event - meas.prev_gnss_ns_at_edge;
    const uint32_t dwt_cycles_between_edges =
        event.dwt_at_event - meas.prev_dwt_at_edge;

    meas.gnss_ns_between_edges = gnss_ns_between_edges;
    meas.dwt_cycles_between_edges = dwt_cycles_between_edges;

    // Positive residual: edge arrived early, clock running fast.
    meas.second_residual_ns =
        (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;

    clock.window_error_ns =
        (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    clock.window_checks++;
    if (llabs(clock.window_error_ns) > CLOCK_WINDOW_TOLERANCE_NS) {
      clock.window_mismatches++;
    }
  }

  meas.prev_gnss_ns_at_edge = event.gnss_ns_at_event;
  meas.prev_dwt_at_edge = event.dwt_at_event;
}

static inline bool usable_clock_event(const interrupt_event_t& event) {
  return epoch_ready() && event.gnss_ns_at_event != 0;
}

static void apply_ocxo_event(clock_state_t& clock,
                             clock_measurement_t& meas,
                             interrupt_capture_diag_t& diag_dst,
                             const interrupt_event_t& event,
                             const interrupt_capture_diag_t* diag) {
  clocks_capture_interrupt_diag(diag_dst, diag);
  if (!usable_clock_event(event)) return;
  clocks_apply_edge(clock, meas, event);
}

static void refresh_ocxo_pps_ledgers(void) {
  g_ocxo1_clock.ns_count_at_pps_vclock = g_ocxo1_clock.zero_established
      ? (uint64_t)((int64_t)g_gnss_ns_at_pps_vclock - g_ocxo1_clock.phase_offset_ns)
      : 0;

  g_ocxo2_clock.ns_count_at_pps_vclock = g_ocxo2_clock.zero_established
      ? (uint64_t)((int64_t)g_gnss_ns_at_pps_vclock - g_ocxo2_clock.phase_offset_ns)
      : 0;
}

static void vclock_callback(const interrupt_event_t& event,
                            const interrupt_capture_diag_t* diag,
                            void*) {
  clocks_capture_interrupt_diag(g_pps_witness_diag, diag);
  g_last_vclock_event_counter32_at_event = event.counter32_at_event;

  if (!epoch_ready()) return;

  g_vclock_event_count++;
  if (event.gnss_ns_at_event != 0) {
    g_gnss_ns_at_pps_vclock = event.gnss_ns_at_event;
  }
  g_prev_dwt_at_vclock_event = event.dwt_at_event;

  if (event.gnss_ns_at_event != 0) {
    clocks_apply_edge(g_vclock_clock, g_vclock_measurement, event);
  }

  g_vclock_clock.ns_count_at_pps_vclock = g_gnss_ns_at_pps_vclock;
  refresh_ocxo_pps_ledgers();
}

static void ocxo1_callback(const interrupt_event_t& event,
                           const interrupt_capture_diag_t* diag,
                           void*) {
  apply_ocxo_event(g_ocxo1_clock, g_ocxo1_measurement,
                   g_ocxo1_interrupt_diag, event, diag);
}

static void ocxo2_callback(const interrupt_event_t& event,
                           const interrupt_capture_diag_t* diag,
                           void*) {
  apply_ocxo_event(g_ocxo2_clock, g_ocxo2_measurement,
                   g_ocxo2_interrupt_diag, event, diag);
}

// ============================================================================
// PPS selector callback: PPS/VCLOCK epoch anchor, bridge update, beta handoff
// ============================================================================

static void publish_pps_witness_diag(const pps_edge_snapshot_t& snap) {
  g_pps_witness_diag.pps_edge_sequence = snap.sequence;
  g_pps_witness_diag.pps_edge_dwt_isr_entry_raw = snap.physical_pps_dwt_raw_at_edge;
  g_pps_witness_diag.pps_edge_gnss_ns = snap.gnss_ns_at_edge;
  g_pps_witness_diag.pps_edge_minus_event_ns =
      (snap.gnss_ns_at_edge >= 0)
          ? (snap.gnss_ns_at_edge - (int64_t)g_gnss_ns_at_pps_vclock)
          : 0;

  const uint32_t dwt_cycles_from_vclock =
      snap.dwt_at_edge - g_prev_dwt_at_vclock_event;
  int64_t ns_from_vclock = 0;
  const uint32_t dwt_per_sec = dwt_effective_cycles_per_pps_vclock_second();
  if (dwt_per_sec > 0) {
    ns_from_vclock =
        (int64_t)(((uint64_t)dwt_cycles_from_vclock * NS_PER_SECOND_U64 +
                   (uint64_t)dwt_per_sec / 2ULL) /
                  (uint64_t)dwt_per_sec);
  }

  g_pps_witness_diag.pps_edge_dwt_cycles_from_vclock = dwt_cycles_from_vclock;
  g_pps_witness_diag.pps_edge_ns_from_vclock = ns_from_vclock;
  g_pps_witness_diag.pps_edge_vclock_event_count = g_vclock_event_count;
}

static void update_pps_vclock_bridge_anchor(const pps_edge_snapshot_t& snap) {
  if (!epoch_ready() || !g_prev_pps_vclock_dwt_at_edge_valid) return;

  const uint32_t dwt_between_pps = snap.dwt_at_edge - g_prev_pps_vclock_dwt_at_edge;

  if (snap.gnss_ns_at_edge >= 0) {
    g_gnss_ns_at_pps_vclock = (uint64_t)snap.gnss_ns_at_edge;
  }

  g_dwt_cycles_between_pps_vclock = dwt_between_pps;
  g_dwt_cycle_count_total += (uint64_t)dwt_between_pps;
  dwt64_anchor_advance_to_dwt32(snap.dwt_at_edge);
  g_dwt_calibration_valid = true;
  g_prev_pps_vclock_dwt_at_edge = snap.dwt_at_edge;

  g_dwt_at_pps_vclock = snap.dwt_at_edge;
  g_counter32_at_pps_vclock = snap.counter32_at_edge;

  time_pps_vclock_update(snap.dwt_at_edge,
                  dwt_effective_cycles_per_pps_vclock_second(),
                  snap.counter32_at_edge);
}

static void maybe_publish_fragment(void) {
  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }
}

static void pps_selector_callback(const pps_edge_snapshot_t& snap) {
  // EPOCH ZERO is intentionally synchronous here: the selected PPS/VCLOCK
  // snapshot is the sacred edge, so process_epoch gets first chance to install
  // its compensated zero before alpha treats the edge as an ordinary bridge
  // update.
  const bool installed_epoch = process_epoch_on_pps_vclock_snapshot(snap);

  publish_pps_witness_diag(snap);
  if (!installed_epoch) {
    update_pps_vclock_bridge_anchor(snap);
  }

  pps_relay_pulse();
  maybe_publish_fragment();
}

// ============================================================================
// Init
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
}

static void subscribe_clock(interrupt_subscriber_kind_t kind,
                            interrupt_subscriber_event_fn callback) {
  interrupt_subscription_t sub{};
  sub.kind = kind;
  sub.on_event = callback;
  sub.user_data = nullptr;
  interrupt_subscribe(sub);
  interrupt_start(kind);
}

void process_clocks_init(void) {
  time_init();
  timebase_init();

  // process_epoch owns epoch/ZERO choreography.  Startup requests a fresh
  // PPS/VCLOCK-selected epoch through the same controller used by ZERO/START.
  (void)process_epoch_request_zero(epoch_reason_t::STARTUP);

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

  subscribe_clock(interrupt_subscriber_kind_t::VCLOCK, vclock_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO1, ocxo1_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO2, ocxo2_callback);

  interrupt_pps_edge_register_dispatch(pps_selector_callback);
}