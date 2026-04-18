// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer (PPS-anchored)
// ============================================================================
//
// Dispatch doctrine:
//
//   GNSS clock state is VCLOCK-driven AND PPS-anchored.
//
//   The vclock_callback subscribes to the VCLOCK lane's one-second event
//   (QTimer1 CH3 cadence, every 1000 ticks at 10 MHz = 1 s of real time)
//   and advances canonical clock state every second: g_gnss_ns_count_at_pps,
//   g_dwt_cycle_count_at_pps, g_qtimer_at_pps, time_pps_update, OCXO phase
//   ledger, and the measured VCLOCK phase ledger.
//
//   The VCLOCK cadence itself is phase-locked to the physical PPS edge
//   by a one-shot rebootstrap mechanism in process_interrupt.cpp.  When
//   alpha needs a fresh epoch (at INIT, ZERO, or START), it calls
//   interrupt_request_pps_rebootstrap() and sets g_epoch_pending.  The
//   NEXT physical PPS GPIO edge:
//
//     1. Is captured in the GPIO ISR with (DWT, counter32, ch3) as
//        first instructions.
//     2. Triggers an in-ISR re-phasing of the VCLOCK cadence so that
//        the first subsequent CH3 compare-match fires exactly 1 ms
//        after the PPS moment, and the first subsequent one-second
//        event fires exactly 1 s after the PPS moment.
//     3. Fires the pps_edge_callback, which — seeing g_epoch_pending
//        — installs the epoch from the snapshot.
//
//   From that point on, every VCLOCK one-second event is coincident
//   with a physical PPS edge (to within GPIO ISR latency).  The PPS
//   witness measurement thereafter measures only that residual latency
//   and its jitter — the hallmark of a true PPS/VCLOCK phase lock.
//
//   "Pulse identity" is thus established by fiat: the PPS edge that
//   triggered the rebootstrap DEFINES which VCLOCK tick is the second
//   boundary.  From the system's point of view, PPS is now phase-
//   locked to VCLOCK — because we defined it that way.
//
// GNSS nanoseconds are AUTHORED from VCLOCK pulses.  VCLOCK is the
// GNSS-disciplined 10 MHz reference.  Each VCLOCK pulse is exactly
// 100 ns of GNSS time; each one-second event represents exactly
// VCLOCK_COUNTS_PER_SECOND pulses, i.e. 1e9 ns.
//
// DWT cycles-between-events is computed by simple one-second
// subtraction of consecutive event.dwt_at_event values — the honest
// measurement.
//
// Per-edge measurement — symmetric template:
//
//   clocks_apply_edge(clock_state_t&, clock_measurement_t&, dwt, gnss_ns)
//   is used identically for VCLOCK, OCXO1, OCXO2.  One function body,
//   three call sites.
//
//   Sign convention: positive second_residual_ns → clock fired EARLY
//   (running fast).  Same across all clocks.
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
volatile uint32_t g_dwt_cycle_count_between_pps = DWT_EXPECTED_PER_PPS;

volatile uint32_t g_qtimer_at_pps = 0;
volatile uint32_t g_last_pps_event_counter32_at_event = 0;

clock_state_t       g_vclock_clock = {};
clock_measurement_t g_vclock_measurement = {};

clock_state_t       g_ocxo1_clock = {};
clock_state_t       g_ocxo2_clock = {};

clock_measurement_t g_ocxo1_measurement = {};
clock_measurement_t g_ocxo2_measurement = {};

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

// Physical PPS GPIO edge sequence at the moment of epoch install.
// Used by the PPS witness phase measurement to project the expected
// hardware-counter value at each subsequent edge.
static volatile uint32_t g_epoch_pps_edge_sequence = 0;

// One-second subtraction state for VCLOCK DWT cycles-between-events.
// Seeded at epoch install; updated each VCLOCK one-second event.
static volatile uint32_t g_prev_dwt_at_vclock_event = 0;

// VCLOCK entry counter — increments on every vclock_callback invocation
// AFTER epoch install.  Read by pps_edge_callback to cross-check that no
// VCLOCK event slipped in between the GPIO ISR and the foreground
// pps_edge_callback.
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
  g_epoch_reason = reason;
  // Arm the GPIO ISR to re-phase the VCLOCK cadence on the next PPS
  // edge.  Without this, the epoch pending flag would be noticed by
  // pps_edge_callback but the CH3 cadence would continue its boot-
  // random phase — defeating the entire purpose of anchoring.
  interrupt_request_pps_rebootstrap();
}

// Accessor for beta — inspects the alpha-owned epoch pending flag.
// Beta waits on this during ZERO/START handshakes; when it clears,
// the PPS-anchored epoch has been installed and beta can finalize.
bool clocks_epoch_pending(void) {
  return g_epoch_pending;
}

static inline bool alpha_ocxo_edges_are_canonical(void) {
  return g_epoch_initialized && !g_epoch_pending;
}

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  g_gnss_ns_count_at_pps = 0;
  g_dwt_cycle_count_at_pps = 0;
  g_dwt_cycle_count_total = 0;
  g_dwt_cycle_count_between_pps = DWT_EXPECTED_PER_PPS;
  g_qtimer_at_pps = 0;
  g_prev_dwt_at_vclock_event = 0;
  g_vclock_event_count = 0;

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

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
  welford_reset(welford_pps_coincidence);
}

// ============================================================================
// Epoch install — PPS-anchored, ONE entry point
// ============================================================================
//
// Called from pps_edge_callback when g_epoch_pending is true.  By the
// time we're here, the GPIO ISR has already re-phased the VCLOCK cadence
// (via the interrupt_request_pps_rebootstrap mechanism), so the next
// vclock_callback will fire exactly 1 second after the PPS moment
// represented by `snap`.
//
// The epoch IS the PPS moment:
//   g_gnss_ns_count_at_pps = 0      (the PPS edge IS time zero)
//   g_dwt_cycle_count_at_pps = snap.dwt_at_edge
//   g_epoch_qtimer_at_pps = snap.counter32_at_edge
//   g_epoch_pps_edge_sequence = snap.sequence
//
static void alpha_install_new_epoch_from_pps_snapshot(
    const pps_edge_snapshot_t& snap) {
  alpha_reset_canonical_clock_state_for_new_epoch();

  g_epoch_dwt_at_pps        = snap.dwt_at_edge;
  g_epoch_qtimer_at_pps     = snap.counter32_at_edge;
  g_epoch_pps_edge_sequence = snap.sequence;
  g_epoch_pps_index         = 0;

  g_dwt_cycle_count_at_pps = snap.dwt_at_edge;
  g_qtimer_at_pps          = snap.counter32_at_edge;
  g_prev_dwt_at_vclock_event = snap.dwt_at_edge;

  time_pps_epoch_reset(snap.dwt_at_edge, snap.counter32_at_edge);

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

    // Sign convention: positive second_residual_ns → clock fired EARLY
    // (gnss_ns_between < 1e9), meaning the clock is running FAST.
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
  meas.prev_dwt_at_edge = dwt_at_edge;
}

// ============================================================================
// VCLOCK callback — pure per-second advancer (epoch install moved to PPS)
// ============================================================================

static void vclock_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  // Stash the VCLOCK-event diag (contains PPS witness fields refreshed
  // by process_interrupt's fill_diag at event emission time).
  if (diag) g_pps_interrupt_diag = *diag;
  else      g_pps_interrupt_diag = {};

  g_last_pps_event_counter32_at_event = event.counter32_at_event;

  // Defensive guard: if the epoch has not yet been installed by the
  // first PPS edge, do nothing.  The VCLOCK cadence may still be in
  // its pre-rebootstrap phase; any state advance here would poison
  // the epoch.
  if (!g_epoch_initialized || g_epoch_pending) {
    return;
  }

  g_vclock_event_count++;

  // ── Canonical per-second advance ──
  g_epoch_pps_index++;
  g_gnss_ns_count_at_pps += NS_PER_SECOND_U64;
  g_dwt_cycle_count_at_pps = event.dwt_at_event;
  g_qtimer_at_pps = event.counter32_at_event;

  // ── DWT cycles between VCLOCK one-second events ──
  //
  // Honest measurement: one-second subtraction of consecutive event
  // DWT captures.  uint32_t subtraction wraps correctly across DWT
  // rollover (~4.26 s at 1008 MHz).
  const uint32_t dwt_between =
      event.dwt_at_event - g_prev_dwt_at_vclock_event;
  g_prev_dwt_at_vclock_event = event.dwt_at_event;

  g_dwt_cycle_count_between_pps = dwt_between;
  g_dwt_cycle_count_total += (uint64_t)dwt_between;

  // ── VCLOCK measurement (peer to OCXO) ──
  if (event.gnss_ns_at_event != 0) {
    clocks_apply_edge(g_vclock_clock, g_vclock_measurement,
                      event.dwt_at_event, event.gnss_ns_at_event);
  }

  // VCLOCK ns_count_at_pps is just the authored cumulative count
  // (no phase-offset adjustment because VCLOCK is the reference).
  g_vclock_clock.ns_count_at_pps = g_gnss_ns_count_at_pps;

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

  // ── PPS/VCLOCK coincidence welford ──
  //
  // The coincidence measurement (ISR-delta between the VCLOCK
  // one-second event and the most recent PPS edge) is captured by
  // process_interrupt's vclock_cadence_isr and carried on the event.
  // Convert cycles to ns using the current measured DWT rate and
  // feed the welford.  Invalid measurements (pre-anchor, or beyond
  // threshold) are skipped — an invalid measurement is not a sample.
  if (event.pps_coincidence_valid) {
    const uint32_t rate = dwt_effective_cycles_per_second();
    if (rate > 0) {
      const int64_t coincidence_ns =
          (int64_t)(((uint64_t)event.pps_coincidence_cycles * NS_PER_SECOND_U64
                     + (uint64_t)rate / 2)
                    / (uint64_t)rate);
      welford_update(welford_pps_coincidence, (double)coincidence_ns);
    }
  }
}

// ============================================================================
// Physical PPS GPIO edge — epoch anchor + witness + fragment dispatch
// ============================================================================
//
// This runs in foreground context, armed by the GPIO ISR via
// timepop_arm_asap.  By the time we're here, the GPIO ISR has already:
//   • Captured (DWT, counter32, ch3) into the snapshot.
//   • If rebootstrap was pending, re-phased the VCLOCK CH3 cadence.
//
// Our responsibilities (in order):
//   1. If ZERO or START is requested and epoch is stable, arm a
//      rebootstrap so the NEXT PPS edge re-anchors.  (Does not affect
//      the current edge.)
//   2. If the epoch is pending, install it from this snapshot.
//   3. Compute and stash the PPS witness diagnostic fields.
//   4. Trigger TIMEBASE_FRAGMENT publication via clocks_beta_pps().
//
// The PPS/VCLOCK coincidence Welford is NOT fed here.  It is fed in
// vclock_callback from the per-event coincidence cycles measured in
// vclock_cadence_isr.  See welford_pps_coincidence.
//
static void pps_edge_callback(const pps_edge_snapshot_t& snap) {

  // ── Step 1: arm a future rebootstrap if a request is outstanding ──
  //
  // We check this BEFORE installing the epoch from the current snap,
  // because if ZERO was requested after the current PPS edge fired in
  // hardware (but before we got to foreground), we don't want to
  // consume this edge — we want the NEXT one.  alpha_request_epoch_zero
  // sets both g_epoch_pending AND arms the rebootstrap flag.
  if ((request_zero || request_start) && !g_epoch_pending) {
    alpha_request_epoch_zero(request_start
                               ? clocks_epoch_reason_t::START
                               : clocks_epoch_reason_t::ZERO);
    // Do NOT install now — this PPS edge was not rebootstrap-aligned.
    // Fall through to publish the fragment with pre-install state;
    // the next edge will consume the pending request.
  }

  // ── Step 2: install the epoch from this snapshot if pending ──
  //
  // By invariant, if we're here with g_epoch_pending and the GPIO ISR
  // successfully processed a rebootstrap, this snap describes that very
  // edge.  The VCLOCK cadence is now phase-locked to this edge.
  if (g_epoch_pending && !interrupt_pps_rebootstrap_pending()) {
    alpha_install_new_epoch_from_pps_snapshot(snap);
  }

  // ── Step 3: stash witness fields for the fragment ──
  //
  // pps_edge_minus_event_ns is the bridge-vs-authored offset: how far
  // apart is the bridge-derived GPIO edge timestamp from the authored
  // GNSS ns counter?  Its interpretation depends on whether the VCLOCK
  // one-second event for this second has been processed in foreground
  // yet (it might not have been — see vclock_callback ordering notes).
  // We publish it for diagnostic continuity but do NOT feed it to the
  // welford, because its variance includes foreground ordering noise.
  g_pps_interrupt_diag.pps_edge_sequence          = snap.sequence;
  g_pps_interrupt_diag.pps_edge_dwt_isr_entry_raw = snap.dwt_at_edge;
  g_pps_interrupt_diag.pps_edge_gnss_ns           = snap.gnss_ns_at_edge;
  g_pps_interrupt_diag.pps_edge_minus_event_ns =
      (snap.gnss_ns_at_edge >= 0)
          ? (snap.gnss_ns_at_edge - (int64_t)g_gnss_ns_count_at_pps)
          : 0;

  // Bridge-independent VCLOCK-to-edge phase diagnostics.
  const uint32_t vclock_count_local = g_vclock_event_count;
  const uint32_t prev_dwt_local     = g_prev_dwt_at_vclock_event;
  const uint32_t dwt_per_sec_local  = dwt_effective_cycles_per_second();
  const uint32_t dwt_cycles_from_vclock =
      snap.dwt_at_edge - prev_dwt_local;
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

  // ── Step 4: publish TIMEBASE_FRAGMENT + assert relay ──
  //
  // The PPS/VCLOCK witness Welford (welford_pps_coincidence) is no
  // longer fed here — it is fed in vclock_callback from the per-event
  // coincidence measurement captured in process_interrupt's
  // vclock_cadence_isr.  That measurement is pure ISR choreography:
  // the raw DWT-cycles delta between the VCLOCK one-second event's
  // ISR entry and this PPS edge's ISR entry.  See
  // interrupt_event_t.pps_coincidence_cycles.
  pps_relay_pulse();

  if (campaign_state == clocks_campaign_state_t::STARTED ||
      request_start || request_stop || request_recover || request_zero) {
    clocks_beta_pps();
  }
}

// ============================================================================
// OCXO one-second events — phase ledger advance via shared clocks_apply_edge
// ============================================================================

static void ocxo1_callback(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  if (diag) g_ocxo1_interrupt_diag = *diag;
  else      g_ocxo1_interrupt_diag = {};

  if (!alpha_ocxo_edges_are_canonical()) return;
  if (event.gnss_ns_at_event == 0) return;

  clocks_apply_edge(g_ocxo1_clock, g_ocxo1_measurement,
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

  clocks_apply_edge(g_ocxo2_clock, g_ocxo2_measurement,
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

  // Arm the first PPS-anchored epoch install.  No VCLOCK-event bootstrap
  // path exists anymore — the first PPS edge WILL be the anchor.
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