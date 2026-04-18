// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// Dispatch doctrine:
//
//   GNSS clock state is VCLOCK-driven.  The vclock_callback subscribes to
//   the VCLOCK lane's one-second event (QTimer1 CH3 cadence, every 1000
//   ticks at 10 MHz = 1 s of real time) and advances all canonical clock
//   state: epoch install, g_gnss_ns_count_at_pps, g_dwt_cycle_count_at_pps,
//   g_qtimer_at_pps, time_pps_update, OCXO phase ledger, and now the
//   measured-VCLOCK phase ledger as well.
//
//   GNSS nanoseconds are AUTHORED from VCLOCK pulses.  VCLOCK is the
//   GNSS-disciplined 10 MHz reference — the time-locked physical
//   quantity.  Each VCLOCK pulse is exactly 100 ns of GNSS time; each
//   one-second event represents exactly VCLOCK_COUNTS_PER_SECOND
//   pulses, i.e. 1e9 ns.
//
//   DWT cycles-between-events is computed by simple one-second
//   subtraction of consecutive event.dwt_at_event values — the honest
//   measurement.
//
//   The physical PPS GPIO edge is demoted to two narrow roles:
//     1. A diagnostic witness (pps_edge_gnss_ns in the fragment).
//     2. The trigger for TIMEBASE_FRAGMENT publication.
//
//   The PPS witness fields are refreshed coherently in pps_edge_callback()
//   from the authoritative pps_edge_snapshot_t.
//
// VCLOCK as measured peer of OCXO (new):
//
//   vclock_apply_edge() is called from vclock_callback in the canonical
//   advance branch.  It is structurally identical to ocxo_apply_edge():
//   same signature, same first-edge handling, same subsequent-edge math.
//   The result is that g_vclock_clock and g_vclock_measurement now hold
//   real measurements rather than the fictional constants the previous
//   code wrote.
//
//   For VCLOCK, second_residual_ns surfaces bridge-interpolation
//   inconsistency rather than crystal drift.  It should sit very close
//   to zero with very small stddev.  If it ever deviates, something is
//   wrong with our DWT bookkeeping or with the dwt_effective_cycles_per_second
//   estimator.
//
// Bridge-independent phase diagnostics:
//
//   pps_edge_callback also stashes three fields that describe the phase
//   relationship between the physical PPS edge and the most recent
//   VCLOCK one-second event, WITHOUT going through the DWT↔GNSS bridge:
//
//     pps_edge_dwt_cycles_from_vclock — raw DWT subtraction
//     pps_edge_ns_from_vclock         — same, in nanoseconds
//     pps_edge_vclock_event_count     — vclock_callback entry count at
//                                        pps_edge_callback read time
//
//   These fields are pure ISR-captured facts and arithmetic.  They
//   survive even if the bridge anchor is mis-authored, and they reveal
//   the boot-determined phase offset between the VCLOCK cadence and
//   the physical PPS cadence.
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

static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;
static constexpr uint64_t PPS_RELAY_OFF_NS  = 500000000ULL;

static_assert(NS_PER_SECOND_U64 ==
              (uint64_t)VCLOCK_COUNTS_PER_SECOND * 100ULL,
              "VCLOCK pulse identity broken: NS_PER_SECOND_U64 != "
              "VCLOCK_COUNTS_PER_SECOND * 100");

// ============================================================================
// Published canonical state (alpha-owned, beta-readable)
// ============================================================================

volatile uint64_t g_gnss_ns_count_at_pps = 0;

volatile uint32_t g_dwt_cycle_count_at_pps = 0;
volatile uint64_t g_dwt_cycle_count_total = 0;
volatile uint32_t g_dwt_cycle_count_between_pps = 0;
volatile uint32_t g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
volatile int32_t  g_dwt_cycle_count_next_second_adjustment = 0;
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
// Epoch state
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

// One-second subtraction state for VCLOCK DWT cycles-between-events.
// Seeded at epoch install; updated each VCLOCK one-second event.
//
// This is independent of g_vclock_measurement.prev_dwt_at_edge:
// they happen to track the same physical quantity but feed different
// downstream consumers (this one feeds g_dwt_cycle_count_between_pps
// and friends; the measurement field feeds vclock_apply_edge).
static volatile uint32_t g_prev_dwt_at_vclock_event = 0;

// VCLOCK entry counter — increments on every vclock_callback invocation,
// including the epoch-install invocation.  Read by pps_edge_callback to
// cross-check that no VCLOCK event slipped in between the GPIO ISR and
// the foreground pps_edge_callback.
//
// Deliberately NOT reset across epoch resets.  This is a pure diagnostic
// counting vclock_callback entries since boot; preserving it across epoch
// resets makes warmup behavior visible (e.g. "campaign started at entry
// 47" shows 47 VCLOCK events happened during pre-STARTED warmup).  The
// Pi-side race check still works: per-fragment delta should be exactly 1.
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
  g_prev_dwt_at_vclock_event = 0;

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

static void alpha_install_new_epoch_from_vclock_event(
    const interrupt_event_t& vclock_event) {
  g_epoch_initialized = true;
  g_epoch_pending = false;
  g_epoch_generation++;
  g_epoch_pps_index = 1;

  g_epoch_dwt_at_pps = vclock_event.dwt_at_event;
  g_epoch_qtimer_at_pps = vclock_event.counter32_at_event;

  alpha_reset_canonical_clock_state_for_new_epoch();

  g_gnss_ns_count_at_pps = NS_PER_SECOND_U64;
  g_dwt_cycle_count_at_pps = vclock_event.dwt_at_event;
  g_qtimer_at_pps = g_epoch_qtimer_at_pps;

  // Seed the one-second subtraction state.  No measurement is available
  // yet; publish DWT_EXPECTED_PER_PPS as a neutral placeholder until the
  // next VCLOCK event yields a real delta.
  g_prev_dwt_at_vclock_event = vclock_event.dwt_at_event;
  g_dwt_cycle_count_between_pps = DWT_EXPECTED_PER_PPS;
  g_dwt_cycle_count_next_second_prediction = DWT_EXPECTED_PER_PPS;
  g_dwt_cycle_count_next_second_adjustment = 0;

  // VCLOCK measurement state at install: leave zero_established=false and
  // prev_*=0 so the first vclock_apply_edge call (next VCLOCK event) takes
  // the first-edge branch, mirroring OCXO bootstrap semantics.

  time_pps_epoch_reset(vclock_event.dwt_at_event, g_epoch_qtimer_at_pps);
}

// ============================================================================
// Per-edge measurement — symmetric template (shared shape with OCXO)
// ============================================================================

static void vclock_apply_edge(vclock_clock_state_t& clock,
                              vclock_measurement_t& meas,
                              uint32_t dwt_at_edge,
                              uint64_t gnss_ns_at_edge) {

  clock.gnss_ns_at_edge = gnss_ns_at_edge;
  meas.dwt_at_edge = dwt_at_edge;

  if (meas.prev_gnss_ns_at_edge == 0) {
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
    if (llabs(clock.window_error_ns) > CLOCK_WINDOW_TOLERANCE_NS) {
      clock.window_mismatches++;
    }
  }

  meas.prev_gnss_ns_at_edge = gnss_ns_at_edge;
  meas.prev_dwt_at_edge = dwt_at_edge;
}

// ============================================================================
// VCLOCK callback — the authoritative GNSS-ns advancer
// ============================================================================

static void vclock_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  // Increment the VCLOCK entry counter at the very top, before any branch.
  // This counts every vclock_callback invocation, including the epoch-
  // install invocation.  Writing a volatile uint32_t is atomic on Cortex-M7.
  g_vclock_event_count++;

  if (diag) g_pps_interrupt_diag = *diag;
  else      g_pps_interrupt_diag = {};

  g_last_pps_event_counter32_at_event = event.counter32_at_event;

  if (!g_epoch_initialized || g_epoch_pending) {
    alpha_install_new_epoch_from_vclock_event(event);

    digitalWriteFast(GNSS_PPS_RELAY, HIGH);
    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
    return;
  }

  // ── Canonical per-second advance ──
  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps += NS_PER_SECOND_U64;
  g_dwt_cycle_count_at_pps = event.dwt_at_event;
  g_qtimer_at_pps = event.counter32_at_event;

  // ── DWT cycles between VCLOCK one-second events: one-second subtraction ──
  //
  // Honest measurement: the number of DWT cycles elapsed between this
  // event's ISR-entry DWT capture and the previous one.  The uint32_t
  // subtraction wraps correctly across DWT_CYCCNT rollover (~4.26 s
  // at 1008 MHz, well beyond one second).
  const uint32_t dwt_between =
      event.dwt_at_event - g_prev_dwt_at_vclock_event;
  g_prev_dwt_at_vclock_event = event.dwt_at_event;

  g_dwt_cycle_count_between_pps = dwt_between;
  g_dwt_cycle_count_next_second_prediction = dwt_between;
  g_dwt_cycle_count_next_second_adjustment = 0;
  g_dwt_cycle_count_total += (uint64_t)dwt_between;
  g_dwt_model_pps_count++;

  // ── VCLOCK measurement (symmetric peer to OCXO) ──
  //
  // Skip when the bridge produced 0 (warmup / bridge not yet valid).
  // Mirrors the OCXO callback's `if (event.gnss_ns_at_event == 0)` guard.
  if (event.gnss_ns_at_event != 0) {
    vclock_apply_edge(g_vclock_clock, g_vclock_measurement,
                      event.dwt_at_event, event.gnss_ns_at_event);
  }

  // VCLOCK ns_count_at_pps is just the authored cumulative count
  // (no phase-offset adjustment because VCLOCK is the reference).
  g_vclock_clock.ns_count_at_pps = g_gnss_ns_count_at_pps;

  // ── Residual tracking from GPIO PPS witness offset ──
  if (g_pps_interrupt_diag.pps_edge_sequence > 0 &&
      g_pps_interrupt_diag.pps_edge_gnss_ns >= 0) {
    residual_update_sample(residual_pps_witness,
                           g_pps_interrupt_diag.pps_edge_minus_event_ns);
  }

  // ── OCXO phase ledger refresh at VCLOCK cadence ──
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

  time_pps_update(g_dwt_cycle_count_at_pps,
                  dwt_effective_cycles_per_second(),
                  g_qtimer_at_pps);

  if ((request_zero || request_start) && !g_epoch_pending) {
    alpha_request_epoch_zero(request_start
                               ? clocks_epoch_reason_t::START
                               : clocks_epoch_reason_t::ZERO);
  }

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }
}

// ============================================================================
// Physical PPS GPIO edge — witness + TIMEBASE_FRAGMENT publication trigger
// ============================================================================

static void pps_edge_callback(const pps_edge_snapshot_t& snap) {
  // Stash the authoritative witness fields from the snapshot.
  g_pps_interrupt_diag.pps_edge_sequence = snap.sequence;
  g_pps_interrupt_diag.pps_edge_dwt_isr_entry_raw = snap.dwt_at_edge;
  g_pps_interrupt_diag.pps_edge_gnss_ns = snap.gnss_ns_at_edge;
  g_pps_interrupt_diag.pps_edge_minus_event_ns =
      (snap.gnss_ns_at_edge >= 0)
          ? (snap.gnss_ns_at_edge - (int64_t)g_gnss_ns_count_at_pps)
          : 0;

  // ── Bridge-independent VCLOCK-to-edge phase diagnostics ──
  //
  // Sample the two pieces of VCLOCK-authored state into local variables
  // before computing derived values, so the three published fields are
  // consistent with each other.  vclock_callback is an ISR (priority 16)
  // and may preempt this foreground callback — a mid-read interleave
  // would produce a self-labeling anomaly (count jumps by +1, dwt delta
  // drops to near zero) rather than silent corruption.
  const uint32_t vclock_count_local = g_vclock_event_count;
  const uint32_t prev_dwt_local     = g_prev_dwt_at_vclock_event;
  const uint32_t dwt_per_sec_local  = dwt_effective_cycles_per_second();

  // Raw DWT delta: how far past the most recent VCLOCK one-second event
  // did the physical PPS edge fire?  Pure uint32_t subtraction; wraps
  // correctly across DWT_CYCCNT rollover.
  const uint32_t dwt_cycles_from_vclock =
      snap.dwt_at_edge - prev_dwt_local;

  // Same in nanoseconds, using the same divisor the bridge uses.
  // Guard against division by zero during boot / before first real delta.
  int64_t ns_from_vclock = 0;
  if (dwt_per_sec_local > 0) {
    ns_from_vclock =
        (int64_t)(((uint64_t)dwt_cycles_from_vclock * NS_PER_SECOND_U64
                   + (uint64_t)dwt_per_sec_local / 2)
                  / (uint64_t)dwt_per_sec_local);
  }

  g_pps_interrupt_diag.pps_edge_dwt_cycles_from_vclock = dwt_cycles_from_vclock;
  g_pps_interrupt_diag.pps_edge_ns_from_vclock = ns_from_vclock;
  g_pps_interrupt_diag.pps_edge_vclock_event_count = vclock_count_local;

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }
}

// ============================================================================
// OCXO one-second events — phase ledger advance
// ============================================================================

static void ocxo_apply_edge(ocxo_clock_state_t& clock,
                            ocxo_measurement_t& meas,
                            uint32_t dwt_at_edge,
                            uint64_t gnss_ns_at_edge) {

  clock.gnss_ns_at_edge = gnss_ns_at_edge;
  meas.dwt_at_edge = dwt_at_edge;

  if (meas.prev_gnss_ns_at_edge == 0) {
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
    if (llabs(clock.window_error_ns) > CLOCK_WINDOW_TOLERANCE_NS) {
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
// Init
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
}

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

  interrupt_pps_edge_register_dispatch(pps_edge_callback);
}