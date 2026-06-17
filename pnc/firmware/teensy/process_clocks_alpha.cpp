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
// OCXO events are delivered as rollover-only one-second events whose DWT
// coordinate is authored by process_interrupt from completed one-second
// intervals.  The older quiet-zone/sample-phase back-projection path is retired:
// Alpha now treats OCXO subscriber events as ordinary clock-edge facts and never
// subtracts the historical 250 us / 750 us offsets from DWT or counter identity.
//
// PPS is a selector/witness. process_interrupt authors the selected PPS/VCLOCK
// edge identity, while the physical PPS/GPIO DWT witness supplies the smooth
// one-second DWT/GNSS slope. VCLOCK/OCXO callbacks own per-edge clock
// measurements.
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
#include "process_timepop.h"
#include "config.h"
#include "util.h"
#include "ad5693r.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <stdlib.h>
#include <strings.h>

// process_time.cpp defines this reset hook; time.h may not expose it on older
// branches, so Alpha declares the narrow symbol it needs when SmartZero makes
// the old projection bases invalid.
void time_clock_reset_all(void);

static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;

static_assert(NS_PER_SECOND_U64 ==
              (uint64_t)VCLOCK_COUNTS_PER_SECOND * 100ULL,
              "VCLOCK pulse identity broken: NS_PER_SECOND_U64 != "
              "VCLOCK_COUNTS_PER_SECOND * 100");

// ============================================================================
// Published canonical state (alpha-owned, beta-readable)
// ============================================================================

volatile uint64_t g_gnss_ns_at_pps_vclock = 0;
volatile uint64_t g_ocxo1_measured_gnss_ns_at_pps_vclock = 0;
volatile uint64_t g_ocxo2_measured_gnss_ns_at_pps_vclock = 0;

// Physical/raw bridge-measured OCXO ledgers at the PPS/VCLOCK row.  The public
// g_ocxo*_measured_gnss_ns_at_pps_vclock values above are now visible-origin
// normalized; these retain the unnormalized evidence for forensics.
volatile uint64_t g_ocxo1_physical_measured_gnss_ns_at_pps_vclock = 0;
volatile uint64_t g_ocxo2_physical_measured_gnss_ns_at_pps_vclock = 0;

volatile uint32_t g_dwt_at_pps_vclock = 0;
volatile uint64_t g_dwt_cycle_count_total = 0;
volatile uint32_t g_dwt_cycles_between_pps_vclock = DWT_EXPECTED_PER_PPS;

// Alpha-owned DWT64 logical clock.  This is the one true DWT64 clock surface.
// It is not physically writable; CLOCKS.ZERO installs a logical epoch origin
// in this raw extended ledger.  The raw ledger itself is anchored by DWT32
// observations and advanced at each PPS/VCLOCK bookend.
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
// Logical-zero / PPS-VCLOCK anchor state
// ============================================================================

static volatile bool g_epoch_initialized = false;
// Set only while a completed live SmartZero proof is being committed into a
// new science epoch.  Event consumers treat the epoch as not ready during this
// short transaction so they cannot observe half-rebased Alpha state.
static volatile bool g_alpha_epoch_install_in_progress = false;

// Last VCLOCK event DWT, retained only for PPS-vs-VCLOCK diagnostics.
//
// g_prev_dwt_at_vclock_event is the authored subscriber DWT.  After VCLOCK EMA
// authority, that value is intentionally synthetic/smoothed and remains the
// correct timing ruler.  PPS→VCLOCK physical phase, however, must be computed
// from the observed/original VCLOCK DWT carried in interrupt diagnostics, not
// from the authored EMA coordinate.
static volatile uint32_t g_prev_dwt_at_vclock_event = 0;
static volatile bool     g_prev_observed_dwt_at_vclock_event_valid = false;
static volatile uint32_t g_prev_observed_dwt_at_vclock_event = 0;
static volatile bool     g_observed_vclock_dwt_cycles_between_edges_valid = false;
static volatile uint32_t g_observed_vclock_dwt_cycles_between_edges = 0;

// ── Canonical one-second subtraction state for DWT cycles between PPS/VCLOCK anchors ──
//
// Authored by pps_selector_callback from process_interrupt's PPS/VCLOCK
// snapshot.  PPS remains the selector/witness.  The operational DWT/GNSS
// cycles-per-second slope now prefers the physical PPS/GPIO witness interval
// because the VCLOCK/QTimer event rail is quantized to a 4-cycle lattice.
static volatile uint32_t g_prev_pps_vclock_dwt_at_edge = 0;
static volatile bool     g_prev_pps_vclock_dwt_at_edge_valid = false;

// Physical PPS witness DWT audit surface for TIMEBASE.  This is the
// physical GPIO PPS event-coordinate DWT captured by process_interrupt
// and exposed through pps_edge_snapshot_t.  It is intentionally separate
// from the canonical PPS/VCLOCK DWT edge.
volatile uint32_t g_pps_dwt_at_edge = 0;
volatile uint32_t g_pps_dwt_cycles_between_edges = 0;
volatile bool     g_pps_dwt_cycles_between_edges_valid = false;
volatile int32_t  g_pps_vclock_phase_cycles = 0;

static volatile uint32_t g_pps_vclock_edge_forensics_seq = 0;
static clocks_pps_vclock_edge_forensics_t g_pps_vclock_edge_forensics = {};

static volatile uint32_t g_prev_pps_dwt_at_edge = 0;
static volatile bool     g_prev_pps_dwt_at_edge_valid = false;

static uint32_t alpha_pps_vclock_phase_cycles_from_edges(uint32_t pps_dwt_at_edge,
                                                         uint32_t pvc_dwt_at_edge) {
  // Scalar PPS→VCLOCK phase.  process_interrupt selects the first VCLOCK edge
  // after PPS by counter identity, but the DWT-bearing VCLOCK fact may be a
  // later VCLOCK edge.  Reduce the PPS→VCLOCK DWT delta into one 10 MHz cell
  // using scaled integer arithmetic so we do not truncate cycles-per-100ns.
  const uint32_t cps = dwt_effective_cycles_per_pps_vclock_second()
      ? dwt_effective_cycles_per_pps_vclock_second()
      : (uint32_t)DWT_EXPECTED_PER_PPS;
  const uint32_t delta_dwt = pvc_dwt_at_edge - pps_dwt_at_edge;
  const uint64_t phase_scaled =
      ((uint64_t)delta_dwt * (uint64_t)VCLOCK_COUNTS_PER_SECOND) %
      (uint64_t)cps;

  return (uint32_t)((phase_scaled +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static inline void clocks_alpha_dmb(void);

static uint64_t alpha_pps_vclock_abs_i64(int64_t value) {
  return (value >= 0) ? (uint64_t)value : (uint64_t)(-value);
}

static void alpha_pps_vclock_edge_forensics_reset(void) {
  g_pps_vclock_edge_forensics_seq++;
  clocks_alpha_dmb();
  g_pps_vclock_edge_forensics = clocks_pps_vclock_edge_forensics_t{};
  clocks_alpha_dmb();
  g_pps_vclock_edge_forensics_seq++;
}

static void alpha_pps_vclock_edge_forensics_publish(
    const pps_edge_snapshot_t& snap,
    bool gnss_self_map_valid,
    uint64_t expected_gnss_ns,
    uint64_t mapped_gnss_ns,
    int64_t gnss_self_error_ns,
    uint32_t dwt_cycles_between_edges,
    uint32_t effective_dwt_cycles_per_second,
    bool counter_identity_valid,
    uint64_t counter_identity_ns) {
  const uint32_t prior_update_count = g_pps_vclock_edge_forensics.update_count;
  const pps_vclock_edge_authority_t& a = snap.vclock_edge_authority;

  clocks_pps_vclock_edge_forensics_t local{};
  local.valid = a.valid;
  local.sequence = snap.sequence;
  local.update_count = prior_update_count + 1U;
  local.reject_count = a.reject_count;

  local.authority_dwt_at_edge = snap.dwt_at_edge;
  local.pps_dwt_at_edge = snap.physical_pps_dwt_normalized_at_edge;
  local.vclock_observed_dwt_at_edge = a.vclock_observed_dwt_at_edge;
  local.vclock_predicted_dwt_at_edge = a.vclock_predicted_dwt_at_edge;
  local.pps_projected_vclock_dwt_at_edge =
      a.pps_projected_vclock_dwt_at_edge;

  local.observed_phase_valid = a.observed_phase_valid;
  local.learned_phase_valid = a.learned_phase_valid;
  local.observed_phase_cycles = a.observed_phase_cycles;
  local.learned_phase_cycles = a.learned_phase_cycles;

  local.gate_cycles = a.gate_cycles;
  local.agreement_span_cycles = a.agreement_span_cycles;
  local.decision = a.decision;
  local.invalid_mask = a.invalid_mask;

  local.authority_minus_pps_cycles = a.authority_minus_pps_cycles;
  local.authority_minus_vclock_observed_cycles =
      a.authority_minus_vclock_observed_cycles;
  local.authority_minus_prediction_cycles =
      a.authority_minus_prediction_cycles;
  local.prediction_minus_pps_projected_cycles =
      a.prediction_minus_pps_projected_cycles;
  local.pps_projected_minus_observed_cycles =
      a.pps_projected_minus_observed_cycles;
  local.observed_minus_prediction_cycles = a.observed_minus_prediction_cycles;

  local.counter32_at_edge = snap.counter32_at_edge;
  local.ch3_at_edge = snap.ch3_at_edge;
  local.dwt_cycles_per_second = a.dwt_cycles_per_second;
  local.dwt_cycles_between_edges = dwt_cycles_between_edges;
  local.effective_dwt_cycles_per_second = effective_dwt_cycles_per_second;

  local.gnss_self_map_valid = gnss_self_map_valid;
  local.gnss_self_error_gate_ns = 1U;
  local.expected_gnss_ns_at_edge = expected_gnss_ns;
  local.mapped_gnss_ns_at_edge = mapped_gnss_ns;
  local.gnss_self_error_ns = gnss_self_error_ns;
  local.gnss_self_error_ok = gnss_self_map_valid &&
      alpha_pps_vclock_abs_i64(gnss_self_error_ns) <=
          (uint64_t)local.gnss_self_error_gate_ns;

  local.counter_identity_valid = counter_identity_valid;
  local.counter_identity_gnss_ns_at_edge =
      counter_identity_valid ? counter_identity_ns : 0ULL;
  local.counter_identity_minus_expected_ns = counter_identity_valid
      ? ((counter_identity_ns >= expected_gnss_ns)
             ? (int64_t)(counter_identity_ns - expected_gnss_ns)
             : -(int64_t)(expected_gnss_ns - counter_identity_ns))
      : 0LL;

  g_pps_vclock_edge_forensics_seq++;
  clocks_alpha_dmb();
  g_pps_vclock_edge_forensics = local;
  clocks_alpha_dmb();
  g_pps_vclock_edge_forensics_seq++;
}

bool clocks_alpha_pps_vclock_edge_forensics(
    clocks_pps_vclock_edge_forensics_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = g_pps_vclock_edge_forensics_seq;
    clocks_alpha_dmb();
    clocks_pps_vclock_edge_forensics_t local = g_pps_vclock_edge_forensics;
    clocks_alpha_dmb();
    const uint32_t seq2 = g_pps_vclock_edge_forensics_seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.update_count != 0;
    }
  }

  *out = clocks_pps_vclock_edge_forensics_t{};
  return false;
}

// VCLOCK event counter — increments on every vclock_callback invocation
// AFTER epoch install.  Read by pps_selector_callback to cross-check that no
// VCLOCK event slipped in between the GPIO ISR and the foreground
// pps_selector_callback.
static volatile uint32_t g_vclock_event_count = 0;

// ============================================================================
// OCXO DAC defaults
// ============================================================================

// The DAC target remains a real-valued control/persistence surface, but the
// AD5693R receives only the nearest static integer code.  There is no
// fractional-code realization layer in this build.  The DACs are configured
// for AD5693R internal 2.5 V reference with 2× gain; external VREF is not used.
// A hard 3.3 V equivalent ceiling is enforced at the DAC-code boundary.
//
// Diagnostic kill switch: when false, CLOCKS may still retain DAC intent in
// memory/reporting, but it must not initialize, write, update, latch, or
// otherwise transact with the physical AD5693R DACs.  This is deliberately
// stronger than disabling dithering: it makes DAC voltage authorship impossible
// inside firmware so OCXO residual tests can isolate whether any DAC touch
// perturbs the analog/control-voltage environment.
static constexpr bool OCXO_DAC_HARDWARE_TOUCH_ENABLED = true;

static ocxo_dac_state_t make_default_ocxo_dac_state() {
  ocxo_dac_state_t s = {};
  s.dac_fractional = (double)AD5693R_DAC_DEFAULT;
  s.dac_hw_code = AD5693R_DAC_DEFAULT;
  s.dac_min = AD5693R_DAC_MIN;
  s.dac_max = OCXO_DAC_SAFE_MAX_HW_CODE;
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
    case servo_mode_t::MEAN:  return "MEAN";
    case servo_mode_t::TOTAL: return "TOTAL";
    case servo_mode_t::NOW:   return "NOW";
    default:                  return "OFF";
  }
}

servo_mode_t servo_mode_parse(const char* s) {
  if (!s || !*s) return servo_mode_t::OFF;
  if (!strcasecmp(s, "MEAN"))  return servo_mode_t::MEAN;
  if (!strcasecmp(s, "TOTAL")) return servo_mode_t::TOTAL;
  if (!strcasecmp(s, "NOW"))   return servo_mode_t::NOW;
  return servo_mode_t::OFF;
}

bool ocxo_dac_set_desired(ocxo_dac_state_t& s, double value) {
  const double target = ocxo_dac_clamp_real_value(value);
  const uint16_t hw_code = ocxo_dac_rounded_hw_code_from_value(target);

  s.dac_fractional = target;
  return ocxo_dac_write_hw_code(s, hw_code, true);
}

bool ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  return ocxo_dac_set_desired(s, value);
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

bool ocxo_dac_write_hw_code(ocxo_dac_state_t& s,
                            uint16_t hw_code,
                            bool latch_fault) {
  // Drop-dead EFC protection: regardless of caller state, servo math, config,
  // or a corrupt dac_max field, never author a hardware code above the
  // 3.3 V equivalent ceiling for the internal-VREF/2× DAC span.
  if ((uint32_t)hw_code < s.dac_min) hw_code = (uint16_t)s.dac_min;
  if ((uint32_t)hw_code > (uint32_t)OCXO_DAC_SAFE_MAX_HW_CODE) {
    hw_code = OCXO_DAC_SAFE_MAX_HW_CODE;
  }
  if ((uint32_t)hw_code > s.dac_max) hw_code = (uint16_t)s.dac_max;
  s.io_last_attempted_hw_code = hw_code;

  if (!OCXO_DAC_HARDWARE_TOUCH_ENABLED) {
    // Diagnostic no-touch mode:
    //   - no ad5693r_init() requirement
    //   - no I2C write_input
    //   - no DAC update/latch
    //   - no claim that the physical DAC code changed
    //
    // Return success so START/SET_DAC/servo can proceed while the hardware
    // output remains whatever the board/power state left it at.
    s.io_last_write_ok = true;
    s.io_last_failure_stage = 0;
    (void)latch_fault;
    return true;
  }

  if (hw_code == s.dac_hw_code) {
    s.io_last_write_ok = true;
    s.io_last_failure_stage = 0;
    return true;
  }

  s.io_write_attempts++;

  if (!g_ad5693r_init_ok) {
    s.io_last_write_ok = false;
    if (latch_fault) s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 3;
    return false;
  }

  const uint8_t addr = (&s == &ocxo1_dac) ?
      AD5693R_ADDR_OCXO1 : AD5693R_ADDR_OCXO2;

  if (!ad5693r_write_input(addr, hw_code)) {
    s.io_last_write_ok = false;
    if (latch_fault) s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 1;
    return false;
  }

  if (!ad5693r_update_dac(addr)) {
    s.io_last_write_ok = false;
    if (latch_fault) s.io_fault_latched = true;
    s.io_write_failures++;
    s.io_last_failure_stage = 2;
    return false;
  }

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

uint64_t clocks_dwt_cycles_at_dwt(uint32_t dwt32) {
  const uint64_t raw_at_dwt = g_dwt64_raw_anchor_valid
      ? (g_dwt64_raw_cycles_at_anchor +
         (uint64_t)((uint32_t)(dwt32 - g_dwt64_dwt32_at_anchor)))
      : (uint64_t)dwt32;

  return g_dwt64_epoch_valid
      ? (raw_at_dwt - g_dwt64_epoch_raw_cycles)
      : raw_at_dwt;
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
  return g_gnss_ns_at_pps_vclock;
}

uint64_t clocks_gnss_ticks_now(void) {
  return clocks_gnss_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
}

uint64_t clocks_ocxo1_measured_gnss_ns_now(void) {
  return g_ocxo1_measured_gnss_ns_at_pps_vclock;
}

uint64_t clocks_ocxo1_measured_gnss_ticks_now(void) {
  return clocks_ocxo1_measured_gnss_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
}

uint64_t clocks_ocxo2_measured_gnss_ns_now(void) {
  return g_ocxo2_measured_gnss_ns_at_pps_vclock;
}

uint64_t clocks_ocxo2_measured_gnss_ticks_now(void) {
  return clocks_ocxo2_measured_gnss_ns_now() / (uint64_t)NS_PER_10MHZ_TICK;
}

uint64_t clocks_ocxo1_nominal_ns_now(void) {
  return clocks_ocxo1_measured_gnss_ns_now();
}

uint64_t clocks_ocxo1_nominal_ticks_now(void) {
  return clocks_ocxo1_measured_gnss_ticks_now();
}

uint64_t clocks_ocxo2_nominal_ns_now(void) {
  return clocks_ocxo2_measured_gnss_ns_now();
}

uint64_t clocks_ocxo2_nominal_ticks_now(void) {
  return clocks_ocxo2_measured_gnss_ticks_now();
}

// ============================================================================
// Beta-facing shadow counts
// ============================================================================

uint64_t dwt_cycle_count_total = 0;
uint64_t gnss_raw_64           = 0;
uint64_t ocxo1_measured_gnss_ticks_64        = 0;
uint64_t ocxo2_measured_gnss_ticks_64        = 0;

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
  return interrupt_smartzero_running() || g_alpha_epoch_install_in_progress;
}

static inline bool epoch_ready(void) {
  return g_epoch_initialized && !g_alpha_epoch_install_in_progress;
}

static inline uint64_t nominal_ns_from_counter32_epoch(uint32_t counter32,
                                               uint32_t epoch_counter32) {
  // Legacy bounded-window helper.  Long-lived lane ledgers must use the
  // alpha_ticks64_* path so 10 MHz time does not wrap at 2^32 ticks.
  return (uint64_t)((uint32_t)(counter32 - epoch_counter32)) *
         (uint64_t)NS_PER_10MHZ_TICK;
}

// ============================================================================
// Alpha integrity counters — reporting only
// ============================================================================

// VCLOCK self-map is a tautological public-coordinate math check and should
// remain exact.  OCXO projected-GNSS interval equality is a physics/rounding
// diagnostic, so give it a small gate to avoid counting normal integer-ns
// projection fuzz as pathology.
static constexpr uint32_t ALPHA_INTEGRITY_EXACT_NS_GATE = 0U;
static constexpr uint32_t ALPHA_INTEGRITY_OCXO_INTERVAL_GATE_NS = 5U;

static clocks_alpha_integrity_snapshot_t g_alpha_integrity = {};

static int64_t alpha_integrity_signed_delta_ns(uint64_t observed,
                                               uint64_t expected) {
  return observed >= expected
      ? (int64_t)(observed - expected)
      : -(int64_t)(expected - observed);
}

static uint64_t alpha_integrity_abs_i64(int64_t value) {
  return value >= 0 ? (uint64_t)value : (uint64_t)(-value);
}

static interrupt_subscriber_kind_t alpha_integrity_interrupt_kind(
    time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK:
      return interrupt_subscriber_kind_t::VCLOCK;
    case time_clock_id_t::OCXO1:
      return interrupt_subscriber_kind_t::OCXO1;
    case time_clock_id_t::OCXO2:
      return interrupt_subscriber_kind_t::OCXO2;
    default:
      return interrupt_subscriber_kind_t::NONE;
  }
}

static void alpha_integrity_record_ns_check(
    clocks_alpha_integrity_ns_check_t& s,
    uint32_t sequence,
    bool input_valid,
    uint64_t expected_ns,
    uint64_t observed_ns,
    uint32_t gate_ns) {
  s.valid = true;
  s.sequence = sequence;
  s.gate_ns = gate_ns;

  if (!input_valid) {
    s.skipped_count++;
    s.last_ok = false;
    s.expected_ns = expected_ns;
    s.observed_ns = observed_ns;
    s.observed_minus_expected_ns = 0;
    return;
  }

  const int64_t error_ns =
      alpha_integrity_signed_delta_ns(observed_ns, expected_ns);
  s.expected_ns = expected_ns;
  s.observed_ns = observed_ns;
  s.observed_minus_expected_ns = error_ns;
  s.test_count++;

  if (alpha_integrity_abs_i64(error_ns) <= (uint64_t)gate_ns) {
    s.ok_count++;
    s.last_ok = true;
  } else {
    s.bad_count++;
    s.last_ok = false;
  }
}

static clocks_alpha_integrity_ocxo_check_t*
alpha_integrity_ocxo_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1:
      return &g_alpha_integrity.ocxo1_projected_gnss_interval;
    case time_clock_id_t::OCXO2:
      return &g_alpha_integrity.ocxo2_projected_gnss_interval;
    default:
      return nullptr;
  }
}

static void alpha_integrity_note_vclock_gnss_self_map(uint32_t sequence,
                                                       uint32_t dwt_at_edge,
                                                       uint64_t expected_ns) {
  uint64_t mapped_ns = 0;
  const bool mapped_ok =
      time_clock_ns_at_dwt(time_clock_id_t::VCLOCK, dwt_at_edge, &mapped_ns);
  alpha_integrity_record_ns_check(
      g_alpha_integrity.vclock_gnss_self_map,
      sequence,
      mapped_ok,
      expected_ns,
      mapped_ok ? mapped_ns : 0ULL,
      ALPHA_INTEGRITY_EXACT_NS_GATE);
}

static void alpha_integrity_note_ocxo_projected_gnss_second(
    time_clock_id_t clock,
    uint32_t sequence,
    uint32_t dwt_at_edge) {
  clocks_alpha_integrity_ocxo_check_t* s =
      alpha_integrity_ocxo_store(clock);
  if (!s) return;

  uint64_t projected_ns = 0;
  const bool projected_ok =
      time_clock_ns_at_dwt(time_clock_id_t::VCLOCK, dwt_at_edge,
                           &projected_ns);
  if (!projected_ok) {
    alpha_integrity_record_ns_check(
        s->interval, sequence, false, 0ULL, 0ULL,
        ALPHA_INTEGRITY_OCXO_INTERVAL_GATE_NS);
    return;
  }

  if (!s->previous_edge_valid || projected_ns < s->previous_edge_projected_gnss_ns) {
    s->previous_edge_valid = true;
    s->previous_edge_projected_gnss_ns = projected_ns;
    alpha_integrity_record_ns_check(
        s->interval, sequence, false, 0ULL, projected_ns,
        ALPHA_INTEGRITY_OCXO_INTERVAL_GATE_NS);
    return;
  }

  const uint64_t current_interval =
      projected_ns - s->previous_edge_projected_gnss_ns;
  s->current_interval_ns = current_interval;
  s->previous_edge_projected_gnss_ns = projected_ns;

  if (!s->previous_interval_valid) {
    s->previous_interval_valid = true;
    s->previous_interval_ns = current_interval;
    alpha_integrity_record_ns_check(
        s->interval, sequence, false, current_interval, current_interval,
        ALPHA_INTEGRITY_OCXO_INTERVAL_GATE_NS);
    return;
  }

  const uint64_t expected_interval = s->previous_interval_ns;
  alpha_integrity_record_ns_check(
      s->interval, sequence, true, expected_interval, current_interval,
      ALPHA_INTEGRITY_OCXO_INTERVAL_GATE_NS);
  s->previous_interval_ns = current_interval;
}

bool clocks_alpha_integrity_snapshot(clocks_alpha_integrity_snapshot_t* out) {
  if (!out) return false;
  g_alpha_integrity.snapshot_count++;
  g_alpha_integrity.valid =
      g_alpha_integrity.vclock_gnss_self_map.valid ||
      g_alpha_integrity.ocxo1_projected_gnss_interval.interval.valid ||
      g_alpha_integrity.ocxo2_projected_gnss_interval.interval.valid;
  *out = g_alpha_integrity;
  return out->valid;
}

// ============================================================================
// Alpha clock forensics
// ============================================================================
//
// This is intentionally report-only.  Alpha records the exact event facts and
// zero-offset-relative arithmetic it applied for each lane before handing that
// event to process_time's generalized clock projection model.  This lets REPORT
// distinguish event/zero-offset errors from projection-to-report-DWT errors.

struct alpha_lane_forensics_store_t {
  volatile uint32_t seq = 0;
  bool     valid = false;
  uint32_t update_count = 0;

  uint32_t last_event_dwt = 0;
  uint32_t last_event_counter32 = 0;

  bool     zero_offset_valid = false;
  uint32_t zero_offset_counter32 = 0;
  uint32_t counter32_delta_since_zero_offset = 0;
  uint32_t counter32_delta_since_previous_event = 0;
  uint64_t logical_ticks64_since_zero = 0;
  uint64_t nominal_ns64_since_zero = 0;

  // Legacy names retained for report/back-compat surfaces.
  uint32_t epoch_counter32 = 0;
  uint32_t counter32_delta_since_epoch = 0;
  uint64_t nominal_ns_from_counter32_epoch = 0;

  // Compatibility mirror.  When process_interrupt supplies a direct
  // GNSS timestamp, event_gnss_ns carries it; otherwise it falls back to
  // the Alpha ledger coordinate used for this lane.
  uint64_t event_gnss_ns = 0;
  uint64_t previous_event_gnss_ns = 0;

  // Process_interrupt-authored GNSS witness for the actual subscriber event
  // sample.  For OCXO lanes this is the quiet-zone sample timestamp, not the
  // Alpha phase-backprojected logical one-second boundary.  These fields are
  // passive forensics only; they do not drive residuals or servo behavior.
  bool     sample_gnss_ns_at_event_available = false;
  bool     previous_sample_gnss_ns_at_event_available = false;
  uint64_t sample_gnss_ns_at_event = 0;
  uint64_t previous_sample_gnss_ns_at_event = 0;

  int64_t  phase_offset_ns = 0;

  uint64_t physical_measured_ns_at_edge = 0;
  uint64_t visible_ns_at_edge = 0;
  bool     visible_origin_phase_valid = false;
  uint32_t visible_origin_phase_offset_ns = 0;

  uint64_t counter_nominal_ns_between_edges = 0;
  uint64_t bridge_gnss_ns_between_edges = 0;
  int64_t  bridge_residual_ns = 0;
  bool     bridge_interval_valid = false;
  bool     bridge_anchored = false;
  int32_t  bridge_phi_cycles = 0;
  uint32_t bridge_span_cycles = 0;
  uint32_t bridge_resolved_count = 0;
  uint32_t bridge_fallback_count = 0;

  uint64_t ns_between_edges = 0;
  uint32_t dwt_cycles_between_edges = 0;

  bool     dwt_synthetic = false;
  bool     dwt_repair_candidate = false;
  uint32_t dwt_original_at_event = 0;
  uint32_t dwt_predicted_at_event = 0;
  uint32_t dwt_used_at_event = 0;
  uint32_t dwt_isr_entry_raw = 0;
  uint32_t dwt_event_from_isr_entry_raw = 0;
  int32_t  dwt_isr_entry_to_event_correction_cycles = 0;
  int32_t  dwt_published_minus_event_cycles = 0;
  int32_t  dwt_used_minus_event_cycles = 0;
  int32_t  dwt_synthetic_error_cycles = 0;
  uint32_t dwt_synthetic_threshold_cycles = 0;
  bool     dwt_interval_gate_valid = false;
  bool     dwt_interval_sample_accepted = false;
  bool     dwt_interval_sample_rejected = false;
  bool     dwt_interval_ema_updated = false;
  uint32_t dwt_interval_observed_cycles = 0;
  uint32_t dwt_interval_prediction_cycles = 0;
  uint32_t dwt_interval_effective_cycles = 0;
  int32_t  dwt_interval_residual_cycles = 0;
  uint32_t dwt_interval_gate_threshold_cycles = 0;
  uint32_t dwt_interval_accept_count = 0;
  uint32_t dwt_interval_reject_count = 0;
  bool     dwt_interval_resync_applied = false;
  uint32_t dwt_interval_resync_count = 0;
  uint32_t dwt_interval_reject_streak = 0;

  bool     dwt_interval_adjacency_gate_valid = false;
  bool     dwt_interval_adjacency_ok = false;
  bool     dwt_interval_adjacency_rejected = false;
  uint32_t dwt_interval_counter_delta_ticks = 0;
  uint32_t dwt_interval_expected_counter_delta_ticks = 0;
  uint32_t dwt_interval_adjacency_reject_count = 0;

  // process_interrupt-authored PPS-Yardstick inference audit (Stage 1 --
  // observational rail).  dwt_at_event remains EMA-authored; these fields
  // carry the parallel yardstick surface per row so TIMEBASE/raw_cycles can
  // adjudicate the Stage 2 authority flip side-by-side with the EMA math.
  bool     dwt_yardstick_valid = false;
  bool     dwt_yardstick_stale = false;
  bool     dwt_yardstick_seeded = false;
  bool     dwt_yardstick_excursion = false;
  uint32_t dwt_yardstick_pps_sequence = 0;
  uint32_t dwt_yardstick_pps_seq_delta = 0;
  uint32_t dwt_yardstick_g_now_cycles = 0;
  uint32_t dwt_yardstick_g_prev_cycles = 0;
  uint32_t dwt_yardstick_inferred_interval_cycles = 0;
  uint32_t dwt_yardstick_observed_interval_cycles = 0;
  int32_t  dwt_yardstick_inferred_minus_observed_cycles = 0;
  uint32_t dwt_yardstick_inferred_endpoint_dwt = 0;
  uint32_t dwt_yardstick_inferred_endpoint_frac_q16 = 0;
  int32_t  dwt_yardstick_endpoint_minus_observed_cycles = 0;
  uint32_t dwt_yardstick_gate_threshold_cycles = 0;
  uint32_t dwt_yardstick_gate_agree_count = 0;
  uint32_t dwt_yardstick_gate_excursion_count = 0;
  bool     dwt_yardstick_authority = false;
  uint32_t dwt_ema_dwt_at_event = 0;
  uint32_t dwt_yardstick_auth_endpoint_dwt = 0;
  uint32_t dwt_yardstick_auth_endpoint_frac_q16 = 0;
  int32_t  dwt_yardstick_auth_error_cycles = 0;
  bool     dwt_yardstick_auth_anchor_applied = false;

  bool     slipledger_active = false;
  bool     slipledger_event_corrected = false;
  bool     slipledger_event_violation = false;
  int32_t  slipledger_ticks = 0;
  int32_t  slipledger_event_ticks = 0;
  uint32_t slipledger_generation = 0;
  uint32_t slipledger_observe_count = 0;
  uint32_t slipledger_ok_count = 0;
  uint32_t slipledger_violation_count = 0;
  uint32_t slipledger_correction_count = 0;
  uint32_t slipledger_noop_violation_count = 0;
  uint32_t slipledger_early_count = 0;
  uint32_t slipledger_late_count = 0;
  uint32_t slipledger_one_second_observe_count = 0;
  uint32_t slipledger_one_second_ok_count = 0;
  uint32_t slipledger_one_second_violation_count = 0;
  uint32_t slipledger_one_second_correction_count = 0;
  uint32_t slipledger_last_expected_dwt = 0;
  uint32_t slipledger_last_observed_dwt = 0;
  uint32_t slipledger_last_authored_dwt = 0;
  uint32_t slipledger_last_expected_interval_cycles = 0;
  uint32_t slipledger_last_observed_interval_cycles = 0;
  int32_t  slipledger_last_dwt_error_cycles = 0;
  uint32_t slipledger_last_target_counter32 = 0;
  uint16_t slipledger_last_hardware_target_low16 = 0;
  uint16_t slipledger_last_ambient_low16 = 0;
  uint32_t slipledger_last_tick_mod = 0;
  uint32_t slipledger_reason_code = 0;
  uint32_t slipledger_last_correction_reason_code = 0;
  int32_t  slipledger_last_correction_ticks = 0;
  int32_t  slipledger_last_correction_dwt_error_cycles = 0;

  int64_t  second_residual_ns = 0;
  int64_t  window_error_ns = 0;
  uint32_t window_checks = 0;
  uint32_t window_mismatches = 0;

  uint32_t diag_anchor_sequence_used = 0;
  uint32_t diag_anchor_age_slots = 0;
  uint32_t diag_anchor_selection_kind = 0;
  uint32_t diag_anchor_dwt_at_edge = 0;
  int64_t  diag_anchor_gnss_ns_at_edge = -1;
  uint32_t diag_anchor_cps = 0;
  uint64_t diag_anchor_ns_delta = 0;
  uint32_t diag_anchor_failure_mask = 0;

  uint32_t diag_service_class = 0;
  int32_t  diag_service_offset_signed_ticks = 0;
  uint32_t diag_service_offset_abs_ticks = 0;
  uint32_t diag_interpreted_late_ticks = 0;
  uint32_t diag_early_ticks = 0;
  uint32_t diag_target_delta_mod65536_ticks = 0;
  uint32_t diag_arm_remaining_ticks = 0;
  uint32_t diag_arm_to_isr_ticks = 0;
  uint32_t diag_arm_to_isr_dwt_cycles = 0;

  uint32_t diag_perishable_fact_sequence = 0;
  int32_t  diag_service_correction_cycles = 0;
  uint32_t diag_service_corrected_dwt_at_event = 0;
  uint32_t diag_fact_ring_overflow_count = 0;
  uint32_t diag_counter_delta_violation_count = 0;
  uint32_t diag_last_bad_counter_delta = 0;
  uint32_t diag_last_counter_delta_ticks = 0;

  bool     diag_sample_phase_valid = false;
  uint32_t diag_sample_phase_ticks = 0;
  uint32_t diag_sample_phase_ns = 0;
  uint32_t diag_sample_phase_us = 0;
  uint32_t diag_sample_period_ticks = 0;
  uint32_t diag_sample_dwt_at_event = 0;
  uint32_t diag_sample_counter32_at_event = 0;
  uint32_t diag_boundary_dwt_at_event = 0;
  uint32_t diag_boundary_counter32_at_event = 0;
  int32_t  diag_boundary_correction_cycles = 0;

  bool     spinidle_shadow_valid = false;
  uint32_t spinidle_shadow_dwt = 0;
  uint32_t spinidle_shadow_to_isr_entry_cycles = 0;
  uint32_t spinidle_shadow_valid_threshold_cycles = 0;

  bool     regression_valid = false;
  uint32_t regression_sequence = 0;
  uint32_t regression_sample_count = 0;
  uint32_t regression_observed_dwt_at_event = 0;
  uint32_t regression_inferred_dwt_at_event = 0;
  int32_t  regression_inferred_minus_observed_cycles = 0;
  uint32_t regression_target_counter32_at_event = 0;
  uint16_t regression_target_hardware16_at_event = 0;
  uint16_t regression_observed_hardware16_at_event = 0;
  uint64_t regression_slope_q16_cycles_per_sample = 0;
  int64_t  regression_slope_delta_q16_cycles_per_sample = 0;
  int32_t  regression_fit_error_mean_q16_cycles = 0;
  uint32_t regression_fit_error_stddev_q16_cycles = 0;
  int32_t  regression_fit_error_min_cycles = 0;
  int32_t  regression_fit_error_max_cycles = 0;
  uint32_t regression_fit_error_gt_plus4_count = 0;
  uint32_t regression_fit_error_lt_minus4_count = 0;
  uint32_t regression_fit_error_abs_gt4_count = 0;
};

static alpha_lane_forensics_store_t g_vclock_forensics = {};
static alpha_lane_forensics_store_t g_ocxo1_forensics = {};
static alpha_lane_forensics_store_t g_ocxo2_forensics = {};


// Report-only event-flow forensics.  These counters answer one narrow question:
// did an interrupt-authored subscriber event reach Alpha, survive Alpha's gates,
// update the lane measurement stores, publish Alpha forensics, and later get
// snapshotted by Beta?  This surface deliberately stays out of TIMEBASE.
static constexpr uint32_t ALPHA_FLOW_STAGE_NONE                 = 0;
static constexpr uint32_t ALPHA_FLOW_STAGE_CALLBACK_ENTRY       = 1;
static constexpr uint32_t ALPHA_FLOW_STAGE_REJECT_EPOCH         = 2;
static constexpr uint32_t ALPHA_FLOW_STAGE_CALLBACK_ACCEPTED    = 3;
static constexpr uint32_t ALPHA_FLOW_STAGE_APPLY_ENTRY          = 4;
static constexpr uint32_t ALPHA_FLOW_STAGE_PHASE_PROJECTED      = 5;
static constexpr uint32_t ALPHA_FLOW_STAGE_TICKS64_OK           = 6;
static constexpr uint32_t ALPHA_FLOW_STAGE_TICKS64_FAIL         = 7;
static constexpr uint32_t ALPHA_FLOW_STAGE_MEASURED_SECOND      = 8;
static constexpr uint32_t ALPHA_FLOW_STAGE_TIME_UPDATE          = 9;
static constexpr uint32_t ALPHA_FLOW_STAGE_STATIC_PREDICTION    = 10;
static constexpr uint32_t ALPHA_FLOW_STAGE_FORENSICS_PUBLISH    = 11;
static constexpr uint32_t ALPHA_FLOW_STAGE_COMPLETE             = 12;
static constexpr uint32_t ALPHA_FLOW_STAGE_FORENSICS_RESET      = 13;
static constexpr uint32_t ALPHA_FLOW_STAGE_FORENSICS_SNAPSHOT   = 14;
static constexpr uint32_t ALPHA_FLOW_STAGE_MEASURED_STORE_FAIL  = 15;

struct alpha_event_flow_store_t {
  uint32_t clock_id = 0;

  uint32_t forensics_reset_count = 0;
  uint32_t callback_entry_count = 0;
  uint32_t callback_diag_present_count = 0;
  uint32_t callback_diag_missing_count = 0;
  uint32_t callback_accepted_count = 0;
  uint32_t callback_rejected_epoch_not_ready_count = 0;

  uint32_t apply_entry_count = 0;
  uint32_t apply_phase_projected_count = 0;
  uint32_t apply_ticks64_success_count = 0;
  uint32_t apply_ticks64_failure_count = 0;
  uint32_t apply_measured_second_count = 0;
  uint32_t apply_measured_store_missing_count = 0;
  uint32_t apply_time_update_count = 0;
  uint32_t apply_static_prediction_count = 0;
  uint32_t apply_complete_count = 0;

  uint32_t forensics_publish_count = 0;
  uint32_t forensics_publish_missing_store_count = 0;
  uint32_t forensics_snapshot_request_count = 0;
  uint32_t forensics_snapshot_consistent_count = 0;
  uint32_t forensics_snapshot_valid_true_count = 0;
  uint32_t forensics_snapshot_valid_false_count = 0;
  uint32_t forensics_snapshot_retry_fail_count = 0;
  uint32_t forensics_snapshot_missing_store_count = 0;

  uint32_t last_stage = ALPHA_FLOW_STAGE_NONE;
  uint32_t last_failure_stage = ALPHA_FLOW_STAGE_NONE;

  uint32_t last_callback_dwt_at_event = 0;
  uint32_t last_callback_counter32_at_event = 0;
  uint64_t last_callback_gnss_ns_at_event = 0;
  bool     last_callback_gnss_ns_available = false;
  bool     last_callback_diag_present = false;
  uint32_t last_callback_diag_anchor_selection_kind = 0;
  uint32_t last_callback_diag_anchor_failure_mask = 0;
  uint32_t last_callback_diag_service_class = 0;
  int32_t  last_callback_diag_service_offset_ticks = 0;
  uint32_t last_callback_diag_perishable_fact_sequence = 0;
  bool     last_callback_sample_phase_valid = false;
  uint32_t last_callback_sample_phase_ticks = 0;

  uint32_t last_rejected_dwt_at_event = 0;
  uint32_t last_rejected_counter32_at_event = 0;
  uint64_t last_rejected_gnss_ns_at_event = 0;

  uint32_t last_applied_dwt_at_event = 0;
  uint32_t last_applied_counter32_at_event = 0;
  uint32_t last_applied_phase_ticks = 0;
  uint32_t last_applied_phase_cycles = 0;
  uint32_t last_applied_dwt_cycles_between_edges = 0;
  uint64_t last_applied_gnss_ns_between_edges = 0;
  int64_t  last_applied_second_residual_ns = 0;
  uint64_t last_applied_ns_now = 0;
  uint32_t last_applied_counter32_delta_since_previous_event = 0;

  bool     last_forensics_store_valid = false;
  uint32_t last_forensics_update_count = 0;
  uint32_t last_forensics_seq = 0;
  uint32_t last_forensics_last_event_dwt = 0;
  uint32_t last_forensics_last_event_counter32 = 0;
  bool     last_forensics_sample_gnss_available = false;
  uint64_t last_forensics_sample_gnss_ns_at_event = 0;

  bool     last_snapshot_return_value = false;
  bool     last_snapshot_store_valid = false;
  uint32_t last_snapshot_update_count = 0;
  uint32_t last_snapshot_seq = 0;
};

static alpha_event_flow_store_t g_vclock_event_flow = {};
static alpha_event_flow_store_t g_ocxo1_event_flow  = {};
static alpha_event_flow_store_t g_ocxo2_event_flow  = {};

static alpha_event_flow_store_t* alpha_event_flow_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return &g_vclock_event_flow;
    case time_clock_id_t::OCXO1:  return &g_ocxo1_event_flow;
    case time_clock_id_t::OCXO2:  return &g_ocxo2_event_flow;
    default:                     return nullptr;
  }
}

static void alpha_event_flow_note_callback(time_clock_id_t clock,
                                           const interrupt_event_t& event,
                                           const interrupt_capture_diag_t* diag) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->callback_entry_count++;
  f->last_stage = ALPHA_FLOW_STAGE_CALLBACK_ENTRY;
  f->last_callback_dwt_at_event = event.dwt_at_event;
  f->last_callback_counter32_at_event = event.counter32_at_event;
  f->last_callback_gnss_ns_at_event = event.gnss_ns_at_event;
  f->last_callback_gnss_ns_available = (event.gnss_ns_at_event != 0);
  f->last_callback_diag_present = (diag != nullptr);
  if (diag) {
    f->callback_diag_present_count++;
    f->last_callback_diag_anchor_selection_kind = diag->anchor_selection_kind;
    f->last_callback_diag_anchor_failure_mask = diag->anchor_failure_mask;
    f->last_callback_diag_service_class = diag->ocxo_service_class;
    f->last_callback_diag_service_offset_ticks = diag->ocxo_service_offset_signed_ticks;
    f->last_callback_diag_perishable_fact_sequence = diag->ocxo_perishable_fact_sequence;
    // Retired quiet-phase/sample-offset doctrine: process_interrupt may still
    // carry legacy sample-phase fields, but Alpha no longer treats them as an
    // active transformation or reports them as accepted callback state.
    f->last_callback_sample_phase_valid = false;
    f->last_callback_sample_phase_ticks = 0;
  } else {
    f->callback_diag_missing_count++;
    f->last_callback_diag_anchor_selection_kind = 0;
    f->last_callback_diag_anchor_failure_mask = 0;
    f->last_callback_diag_service_class = 0;
    f->last_callback_diag_service_offset_ticks = 0;
    f->last_callback_diag_perishable_fact_sequence = 0;
    f->last_callback_sample_phase_valid = false;
    f->last_callback_sample_phase_ticks = 0;
  }
}

static void alpha_event_flow_note_reject_epoch(time_clock_id_t clock,
                                               const interrupt_event_t& event) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->callback_rejected_epoch_not_ready_count++;
  f->last_stage = ALPHA_FLOW_STAGE_REJECT_EPOCH;
  f->last_failure_stage = ALPHA_FLOW_STAGE_REJECT_EPOCH;
  f->last_rejected_dwt_at_event = event.dwt_at_event;
  f->last_rejected_counter32_at_event = event.counter32_at_event;
  f->last_rejected_gnss_ns_at_event = event.gnss_ns_at_event;
}

static void alpha_event_flow_note_callback_accepted(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->callback_accepted_count++;
  f->last_stage = ALPHA_FLOW_STAGE_CALLBACK_ACCEPTED;
}

static void alpha_event_flow_note_apply_entry(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_entry_count++;
  f->last_stage = ALPHA_FLOW_STAGE_APPLY_ENTRY;
}

static void alpha_event_flow_note_ticks64(time_clock_id_t clock,
                                          bool ok,
                                          const interrupt_event_t& applied_event,
                                          uint32_t delta_since_previous_event) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  if (ok) {
    f->apply_ticks64_success_count++;
    f->last_stage = ALPHA_FLOW_STAGE_TICKS64_OK;
  } else {
    f->apply_ticks64_failure_count++;
    f->last_stage = ALPHA_FLOW_STAGE_TICKS64_FAIL;
    f->last_failure_stage = ALPHA_FLOW_STAGE_TICKS64_FAIL;
  }
  f->last_applied_dwt_at_event = applied_event.dwt_at_event;
  f->last_applied_counter32_at_event = applied_event.counter32_at_event;
  f->last_applied_counter32_delta_since_previous_event = delta_since_previous_event;
}

static void alpha_event_flow_note_measured_store_missing(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_measured_store_missing_count++;
  f->last_stage = ALPHA_FLOW_STAGE_MEASURED_STORE_FAIL;
  f->last_failure_stage = ALPHA_FLOW_STAGE_MEASURED_STORE_FAIL;
}

static void alpha_event_flow_note_measured(time_clock_id_t clock,
                                           uint32_t dwt_cycles,
                                           uint64_t interval_ns,
                                           int64_t residual_ns,
                                           uint64_t ns_now) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_measured_second_count++;
  f->last_stage = ALPHA_FLOW_STAGE_MEASURED_SECOND;
  f->last_applied_dwt_cycles_between_edges = dwt_cycles;
  f->last_applied_gnss_ns_between_edges = interval_ns;
  f->last_applied_second_residual_ns = residual_ns;
  f->last_applied_ns_now = ns_now;
}

static void alpha_event_flow_note_time_update(time_clock_id_t clock, uint64_t ns_now) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_time_update_count++;
  f->last_stage = ALPHA_FLOW_STAGE_TIME_UPDATE;
  f->last_applied_ns_now = ns_now;
}

static void alpha_event_flow_note_static_prediction(time_clock_id_t clock,
                                                    uint32_t dwt_cycles) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_static_prediction_count++;
  f->last_stage = ALPHA_FLOW_STAGE_STATIC_PREDICTION;
  f->last_applied_dwt_cycles_between_edges = dwt_cycles;
}

static void alpha_event_flow_note_forensics_reset(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->forensics_reset_count++;
  f->last_stage = ALPHA_FLOW_STAGE_FORENSICS_RESET;
  f->last_forensics_store_valid = false;
  f->last_forensics_update_count = 0;
  f->last_forensics_seq = 0;
}

static void alpha_event_flow_note_forensics_publish(time_clock_id_t clock,
                                                    const alpha_lane_forensics_store_t& s) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->forensics_publish_count++;
  f->last_stage = ALPHA_FLOW_STAGE_FORENSICS_PUBLISH;
  f->last_forensics_store_valid = s.valid;
  f->last_forensics_update_count = s.update_count;
  f->last_forensics_seq = s.seq;
  f->last_forensics_last_event_dwt = s.last_event_dwt;
  f->last_forensics_last_event_counter32 = s.last_event_counter32;
  f->last_forensics_sample_gnss_available = s.sample_gnss_ns_at_event_available;
  f->last_forensics_sample_gnss_ns_at_event = s.sample_gnss_ns_at_event;
}

static void alpha_event_flow_note_forensics_missing_store(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->forensics_publish_missing_store_count++;
  f->last_failure_stage = ALPHA_FLOW_STAGE_FORENSICS_PUBLISH;
}

static void alpha_event_flow_note_complete(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_complete_count++;
  f->last_stage = ALPHA_FLOW_STAGE_COMPLETE;
}

static void alpha_event_flow_note_snapshot_request(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->forensics_snapshot_request_count++;
}

static void alpha_event_flow_note_snapshot(time_clock_id_t clock,
                                           bool consistent,
                                           bool return_value,
                                           bool store_valid,
                                           uint32_t update_count,
                                           uint32_t seq) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  if (consistent) {
    f->forensics_snapshot_consistent_count++;
    if (store_valid) f->forensics_snapshot_valid_true_count++;
    else f->forensics_snapshot_valid_false_count++;
    f->last_stage = ALPHA_FLOW_STAGE_FORENSICS_SNAPSHOT;
  } else {
    f->forensics_snapshot_retry_fail_count++;
    f->last_failure_stage = ALPHA_FLOW_STAGE_FORENSICS_SNAPSHOT;
  }
  f->last_snapshot_return_value = return_value;
  f->last_snapshot_store_valid = store_valid;
  f->last_snapshot_update_count = update_count;
  f->last_snapshot_seq = seq;
}

static void alpha_event_flow_note_snapshot_missing_store(time_clock_id_t clock) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->forensics_snapshot_missing_store_count++;
  f->last_failure_stage = ALPHA_FLOW_STAGE_FORENSICS_SNAPSHOT;
}

bool clocks_alpha_event_flow_snapshot(time_clock_id_t clock,
                                      clocks_alpha_event_flow_snapshot_t* out) {
  if (!out) return false;
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return false;

  clocks_alpha_event_flow_snapshot_t local{};
  local.clock_id = (uint32_t)((uint8_t)clock);
  local.forensics_reset_count = f->forensics_reset_count;
  local.callback_entry_count = f->callback_entry_count;
  local.callback_diag_present_count = f->callback_diag_present_count;
  local.callback_diag_missing_count = f->callback_diag_missing_count;
  local.callback_accepted_count = f->callback_accepted_count;
  local.callback_rejected_epoch_not_ready_count = f->callback_rejected_epoch_not_ready_count;
  local.apply_entry_count = f->apply_entry_count;
  local.apply_phase_projected_count = f->apply_phase_projected_count;
  local.apply_ticks64_success_count = f->apply_ticks64_success_count;
  local.apply_ticks64_failure_count = f->apply_ticks64_failure_count;
  local.apply_measured_second_count = f->apply_measured_second_count;
  local.apply_measured_store_missing_count = f->apply_measured_store_missing_count;
  local.apply_time_update_count = f->apply_time_update_count;
  local.apply_static_prediction_count = f->apply_static_prediction_count;
  local.apply_complete_count = f->apply_complete_count;
  local.forensics_publish_count = f->forensics_publish_count;
  local.forensics_publish_missing_store_count = f->forensics_publish_missing_store_count;
  local.forensics_snapshot_request_count = f->forensics_snapshot_request_count;
  local.forensics_snapshot_consistent_count = f->forensics_snapshot_consistent_count;
  local.forensics_snapshot_valid_true_count = f->forensics_snapshot_valid_true_count;
  local.forensics_snapshot_valid_false_count = f->forensics_snapshot_valid_false_count;
  local.forensics_snapshot_retry_fail_count = f->forensics_snapshot_retry_fail_count;
  local.forensics_snapshot_missing_store_count = f->forensics_snapshot_missing_store_count;
  local.last_stage = f->last_stage;
  local.last_failure_stage = f->last_failure_stage;
  local.last_callback_dwt_at_event = f->last_callback_dwt_at_event;
  local.last_callback_counter32_at_event = f->last_callback_counter32_at_event;
  local.last_callback_gnss_ns_at_event = f->last_callback_gnss_ns_at_event;
  local.last_callback_gnss_ns_available = f->last_callback_gnss_ns_available;
  local.last_callback_diag_present = f->last_callback_diag_present;
  local.last_callback_diag_anchor_selection_kind = f->last_callback_diag_anchor_selection_kind;
  local.last_callback_diag_anchor_failure_mask = f->last_callback_diag_anchor_failure_mask;
  local.last_callback_diag_service_class = f->last_callback_diag_service_class;
  local.last_callback_diag_service_offset_ticks = f->last_callback_diag_service_offset_ticks;
  local.last_callback_diag_perishable_fact_sequence = f->last_callback_diag_perishable_fact_sequence;
  local.last_callback_sample_phase_valid = f->last_callback_sample_phase_valid;
  local.last_callback_sample_phase_ticks = f->last_callback_sample_phase_ticks;
  local.last_rejected_dwt_at_event = f->last_rejected_dwt_at_event;
  local.last_rejected_counter32_at_event = f->last_rejected_counter32_at_event;
  local.last_rejected_gnss_ns_at_event = f->last_rejected_gnss_ns_at_event;
  local.last_applied_dwt_at_event = f->last_applied_dwt_at_event;
  local.last_applied_counter32_at_event = f->last_applied_counter32_at_event;
  local.last_applied_phase_ticks = f->last_applied_phase_ticks;
  local.last_applied_phase_cycles = f->last_applied_phase_cycles;
  local.last_applied_dwt_cycles_between_edges = f->last_applied_dwt_cycles_between_edges;
  local.last_applied_gnss_ns_between_edges = f->last_applied_gnss_ns_between_edges;
  local.last_applied_second_residual_ns = f->last_applied_second_residual_ns;
  local.last_applied_ns_now = f->last_applied_ns_now;
  local.last_applied_counter32_delta_since_previous_event = f->last_applied_counter32_delta_since_previous_event;
  local.last_forensics_store_valid = f->last_forensics_store_valid;
  local.last_forensics_update_count = f->last_forensics_update_count;
  local.last_forensics_seq = f->last_forensics_seq;
  local.last_forensics_last_event_dwt = f->last_forensics_last_event_dwt;
  local.last_forensics_last_event_counter32 = f->last_forensics_last_event_counter32;
  local.last_forensics_sample_gnss_available = f->last_forensics_sample_gnss_available;
  local.last_forensics_sample_gnss_ns_at_event = f->last_forensics_sample_gnss_ns_at_event;
  local.last_snapshot_return_value = f->last_snapshot_return_value;
  local.last_snapshot_store_valid = f->last_snapshot_store_valid;
  local.last_snapshot_update_count = f->last_snapshot_update_count;
  local.last_snapshot_seq = f->last_snapshot_seq;
  *out = local;
  return true;
}

// Alpha owns the long logical tick ledgers for the three 10 MHz lanes.
// process_interrupt emits compact synthetic counter32 event identities; alpha
// subtracts the per-lane zero-offset counter32 once, then extends subsequent
// event-to-event deltas into 64-bit tick totals so a campaign cannot wrap at
// the 32-bit / 10 MHz boundary (~429.5 s).
struct alpha_lane_logical_ticks_t {
  uint32_t zero_counter32 = 0;
  uint32_t last_counter32 = 0;
  uint32_t counter32_delta_since_previous_event = 0;
  uint64_t ticks64 = 0;
  uint32_t update_count = 0;
};

static alpha_lane_logical_ticks_t g_vclock_ticks64 = {};
static alpha_lane_logical_ticks_t g_ocxo1_ticks64 = {};
static alpha_lane_logical_ticks_t g_ocxo2_ticks64 = {};

// OCXO public nanosecond ledgers are measured GNSS-elapsed clocks, not merely
// counter-ticks × 100 ns and not ideal +1e9-per-edge nominal ledgers. The raw
// tick ledger remains the identity surface. The measured ledger advances by
// the projected GNSS duration between consecutive OCXO one-second edges.
// ============================================================================
// GNSS bridge anchors: bookend interpolation for the science residual
// ============================================================================
//
// The OCXO measured-second math previously dead-reckoned: it converted the
// ENTIRE ~1.008e9-cycle OCXO second to nanoseconds through one raw, possibly
// stale cycles-per-second sample, and accumulated the result into a ledger
// that GNSS truth never re-entered.  Any mismatch between that single cps
// sample and the true average XTAL rate over the OCXO's own second landed
// one-for-one (cycles ~= ns) in the science residual -- common-mode across
// both lanes (same sample), thermally correlated, +/-5..14 ns at slope
// reversals (Zero3).  Two ovens were blamed for a naked crystal.
//
// The replacement is the bookend construction: per second we record one
// bridge anchor -- the physical PPS witness DWT paired with its exact GNSS
// nanosecond coordinate (epoch + k*1e9; GNSS seconds are truth by
// definition).  An OCXO edge's GNSS-ns is then interpolated between the two
// anchors that BRACKET it, using the cps measured across that very bracket:
//
//   ns(E) = ns(A_n) + (E - dwt(A_n)) * (ns(A_n+1) - ns(A_n))
//                                    / (dwt(A_n+1) - dwt(A_n))
//
// Anchor-to-anchor distance contributes exactly k*1e9 with ZERO ruler
// dependence; the wandering XTAL touches only the short phase stub, scaled
// by the phase fraction.  Each bookend is nailed to ~a cycle, the measured
// ledger is computed absolutely (errors no longer integrate), and Teensy
// thermal drift is neutralized by construction.  The closing anchor arrives
// one second after the edge, so resolved measurements lag one second --
// which costs the Welford science nothing.
struct alpha_bridge_anchor_t {
  bool     valid = false;
  uint32_t dwt = 0;
  uint64_t gnss_ns = 0;
};
static constexpr uint32_t ALPHA_BRIDGE_ANCHOR_RING_SIZE = 4U;
// A pending edge unresolved after this many subsequent OCXO edges falls back
// to legacy ratio conversion (GNSS holdover), truthfully counted.
static constexpr uint32_t ALPHA_BRIDGE_PENDING_MAX_AGE_EVENTS = 3U;
static alpha_bridge_anchor_t g_bridge_anchors[ALPHA_BRIDGE_ANCHOR_RING_SIZE] = {};
static uint32_t g_bridge_anchor_total = 0;

struct alpha_measured_ns_clock_t {
  bool     initialized = false;
  uint64_t ns_at_edge = 0;        // GNSS ns at the most recent RESOLVED edge
  uint32_t dwt_at_edge = 0;       // DWT of the most recent RESOLVED edge
  uint64_t dwt64_at_edge = 0;

  // Newest edge awaiting its closing bridge anchor (one-second resolution lag).
  bool     pending_valid = false;
  uint32_t pending_edge_dwt = 0;
  uint32_t pending_age_events = 0;

  // Bridge evidence.
  bool     last_resolved_via_bridge = false;
  uint32_t bridge_resolved_count = 0;
  uint32_t bridge_fallback_count = 0;
  int32_t  bridge_last_phi_cycles = 0;
  uint32_t bridge_last_span_cycles = 0;
};

static alpha_measured_ns_clock_t g_ocxo1_measured_ns = {};
static alpha_measured_ns_clock_t g_ocxo2_measured_ns = {};

static inline void clocks_alpha_dmb(void) {
  __asm__ volatile ("dmb" ::: "memory");
}


// ============================================================================
// Static one-second prediction audit
// ============================================================================
//
// Gamma/dynamic prediction is retired.  This compact audit records the honest
// static model: the prior completed interval predicts the next one.  Four
// symmetric rails are tracked independently:
//   PPS    — physical GPIO PPS edge-to-edge DWT interval
//   VCLOCK — canonical PPS/VCLOCK lattice edge-to-edge DWT interval
//   OCXO1  — OCXO1 authored edge-to-edge DWT interval
//   OCXO2  — OCXO2 authored edge-to-edge DWT interval

struct alpha_static_prediction_store_t {
  volatile uint32_t seq = 0;
  bool     valid = false;
  uint32_t completed_interval_count = 0;
  uint32_t last_actual_cycles = 0;
  uint32_t static_prediction_cycles = 0;
  uint32_t actual_cycles = 0;
  int32_t  static_residual_cycles = 0;
};

static alpha_static_prediction_store_t g_static_prediction_pps = {};
static alpha_static_prediction_store_t g_static_prediction_vclock = {};
static alpha_static_prediction_store_t g_static_prediction_ocxo1 = {};
static alpha_static_prediction_store_t g_static_prediction_ocxo2 = {};

static alpha_static_prediction_store_t* alpha_static_prediction_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return &g_static_prediction_vclock;
    case time_clock_id_t::OCXO1:  return &g_static_prediction_ocxo1;
    case time_clock_id_t::OCXO2:  return &g_static_prediction_ocxo2;
    default:                     return nullptr;
  }
}

static void alpha_static_prediction_reset_store(alpha_static_prediction_store_t& s) {
  s.seq++;
  clocks_alpha_dmb();

  s.valid = false;
  s.completed_interval_count = 0;
  s.last_actual_cycles = 0;
  s.static_prediction_cycles = 0;
  s.actual_cycles = 0;
  s.static_residual_cycles = 0;

  clocks_alpha_dmb();
  s.seq++;
}

void clocks_static_prediction_reset_all(void) {
  alpha_static_prediction_reset_store(g_static_prediction_pps);
  alpha_static_prediction_reset_store(g_static_prediction_vclock);
  alpha_static_prediction_reset_store(g_static_prediction_ocxo1);
  alpha_static_prediction_reset_store(g_static_prediction_ocxo2);
}

static void alpha_static_prediction_record(time_clock_id_t clock,
                                           uint32_t actual_cycles) {
  if (actual_cycles == 0) return;
  alpha_static_prediction_store_t* s = alpha_static_prediction_store(clock);
  if (!s) return;

  const uint32_t prior_actual = s->last_actual_cycles;
  const bool have_prior = (prior_actual != 0);

  s->seq++;
  clocks_alpha_dmb();

  s->completed_interval_count++;
  s->valid = have_prior;
  s->static_prediction_cycles = have_prior ? prior_actual : 0U;
  s->actual_cycles = actual_cycles;
  s->static_residual_cycles = have_prior
      ? (int32_t)((int64_t)actual_cycles - (int64_t)prior_actual)
      : 0;
  s->last_actual_cycles = actual_cycles;

  clocks_alpha_dmb();
  s->seq++;
}

static void alpha_static_prediction_record_pps(uint32_t actual_cycles) {
  if (actual_cycles == 0) return;

  alpha_static_prediction_store_t& s = g_static_prediction_pps;
  const uint32_t prior_actual = s.last_actual_cycles;
  const bool have_prior = (prior_actual != 0);

  s.seq++;
  clocks_alpha_dmb();

  s.completed_interval_count++;
  s.valid = have_prior;
  s.static_prediction_cycles = have_prior ? prior_actual : 0U;
  s.actual_cycles = actual_cycles;
  s.static_residual_cycles = have_prior
      ? (int32_t)((int64_t)actual_cycles - (int64_t)prior_actual)
      : 0;
  s.last_actual_cycles = actual_cycles;

  clocks_alpha_dmb();
  s.seq++;
}

static uint32_t alpha_static_prediction_clock_id(time_clock_id_t clock) {
  return (uint32_t)((uint8_t)clock);
}

bool clocks_static_prediction_snapshot(time_clock_id_t clock,
                                       clocks_static_prediction_snapshot_t* out) {
  if (!out) return false;
  alpha_static_prediction_store_t* s = alpha_static_prediction_store(clock);
  if (!s) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    clocks_alpha_dmb();

    clocks_static_prediction_snapshot_t local{};
    local.clock_id = alpha_static_prediction_clock_id(clock);
    local.valid = s->valid;
    local.completed_interval_count = s->completed_interval_count;
    local.static_prediction_cycles = s->static_prediction_cycles;
    local.actual_cycles = s->actual_cycles;
    local.static_residual_cycles = s->static_residual_cycles;

    clocks_alpha_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.valid;
    }
  }

  *out = clocks_static_prediction_snapshot_t{};
  return false;
}

static bool alpha_static_prediction_snapshot_store(const alpha_static_prediction_store_t& store,
                                                   uint32_t clock_id,
                                                   clocks_static_prediction_snapshot_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = store.seq;
    clocks_alpha_dmb();

    clocks_static_prediction_snapshot_t local{};
    local.clock_id = clock_id;
    local.valid = store.valid;
    local.completed_interval_count = store.completed_interval_count;
    local.static_prediction_cycles = store.static_prediction_cycles;
    local.actual_cycles = store.actual_cycles;
    local.static_residual_cycles = store.static_residual_cycles;

    clocks_alpha_dmb();
    const uint32_t seq2 = store.seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.valid;
    }
  }

  *out = clocks_static_prediction_snapshot_t{};
  return false;
}

bool clocks_static_prediction_pps_snapshot(clocks_static_prediction_snapshot_t* out) {
  return alpha_static_prediction_snapshot_store(g_static_prediction_pps, 0U, out);
}

// ============================================================================
// Report-only OCXO PPS-edge projection surface
// ============================================================================
//
// Step A for the PPS-founded OCXO clock standard.  Alpha records OCXO
// edge facts and, once per PPS/VCLOCK row, computes what the OCXO
// clock's own nanosecond ledger would have read at that PPS/VCLOCK DWT
// coordinate.  This is not yet promoted into g_ocxo*_measured_gnss_ns_at_pps_vclock
// and it does not feed TIMEBASE/Welfords/servo.

static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_SOURCE_NONE = 0;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_SOURCE_ACTUAL_BRACKET = 1;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_SOURCE_STATIC_NEXT_EDGE = 2;

static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_INVALID_NONE = 0;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_EDGE = 1;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_INTERVAL = 2;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_INVALID_TARGET_OUT_OF_WINDOW = 3;

// Keep the public snapshot schema unchanged for this surgical fix: new wrap
// and sanity failures are counted on alpha-local guard counters and reported
// through the existing TARGET_OUT_OF_WINDOW invalid bucket until we wire a
// dedicated diagnostic report.
static constexpr uint64_t ALPHA_OCXO_PPS_PROJECTION_SANITY_WINDOW_NS = 100000000ULL;
static constexpr uint32_t ALPHA_OCXO_PPS_PROJECTION_MAX_SIGNED_DWT_SECONDS = 3U;

// Static PPS projection may find that the latest observed OCXO edge is one or
// two synthetic one-second intervals behind the PPS/VCLOCK target.  That is a
// custody timing phase issue, not an OCXO event failure.  Advance the static
// edge a small bounded number of intervals before declaring the target out of
// window.  Keep this intentionally small so a truly backward or stale DWT
// relationship still fails loudly.
static constexpr uint32_t ALPHA_OCXO_PPS_STATIC_ADVANCE_LIMIT = 2U;

struct alpha_ocxo_edge_fact_t {
  bool     valid = false;
  uint32_t dwt_at_edge = 0;
  uint64_t dwt64_at_edge = 0;
  uint32_t counter32_at_edge = 0;
  uint64_t ocxo_ns_at_edge = 0;
  uint64_t measured_ns_at_edge = 0;
  bool     sample_gnss_available = false;
  uint64_t sample_gnss_ns_at_event = 0;
  bool     boundary_gnss_available = false;
  uint64_t boundary_gnss_ns_at_edge = 0;
  uint32_t sample_phase_ticks = 0;
  uint32_t phase_cycles = 0;
};

struct alpha_ocxo_edge_history_t {
  bool current_valid = false;
  bool previous_valid = false;
  alpha_ocxo_edge_fact_t current = {};
  alpha_ocxo_edge_fact_t previous = {};
  uint32_t update_count = 0;
};

struct alpha_ocxo_pps_projection_store_t {
  volatile uint32_t seq = 0;
  clocks_alpha_ocxo_pps_projection_snapshot_t v = {};
};

struct alpha_ocxo_pps_projection_guard_t {
  uint32_t measured_projection_signed_backward_count = 0;
  uint32_t measured_projection_legacy_unsigned_wrap_count = 0;
  uint32_t measured_projection_reject_count = 0;
  uint32_t pps_projection_signed_backward_count = 0;
  uint32_t pps_projection_legacy_unsigned_wrap_count = 0;
  uint32_t pps_projection_sanity_reject_count = 0;

  uint32_t last_pps_sequence = 0;
  uint32_t last_target_dwt32 = 0;
  uint32_t last_reference_dwt32 = 0;
  int32_t  last_signed_delta_cycles = 0;
  uint32_t last_legacy_unsigned_delta_cycles = 0;

  // Projection-vs-measured is a diagnostic offset, not an absolute validity
  // invariant.  The nominal OCXO ledger and measured GNSS-elapsed ledger may
  // legitimately have a large fixed separation after SmartZero/START base
  // selection.  The pathology is a sudden jump in that separation, not the
  // separation itself.
  bool     last_projected_minus_measured_valid = false;
  int64_t  last_projected_minus_measured_ns = 0;
  int64_t  last_projected_minus_measured_jump_ns = 0;
};

static alpha_ocxo_edge_history_t g_ocxo1_edge_history = {};
static alpha_ocxo_edge_history_t g_ocxo2_edge_history = {};
static alpha_ocxo_pps_projection_store_t g_ocxo1_pps_projection = {};
static alpha_ocxo_pps_projection_store_t g_ocxo2_pps_projection = {};
static alpha_ocxo_pps_projection_guard_t g_ocxo1_pps_projection_guard = {};
static alpha_ocxo_pps_projection_guard_t g_ocxo2_pps_projection_guard = {};

static alpha_ocxo_edge_history_t* alpha_ocxo_edge_history(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_ocxo1_edge_history;
    case time_clock_id_t::OCXO2: return &g_ocxo2_edge_history;
    default:                    return nullptr;
  }
}

static alpha_ocxo_pps_projection_store_t* alpha_ocxo_pps_projection_store(
    time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_ocxo1_pps_projection;
    case time_clock_id_t::OCXO2: return &g_ocxo2_pps_projection;
    default:                    return nullptr;
  }
}

static int64_t alpha_signed_delta_u64(uint64_t a, uint64_t b) {
  return (a >= b) ? (int64_t)(a - b) : -(int64_t)(b - a);
}

static alpha_ocxo_pps_projection_guard_t* alpha_ocxo_pps_projection_guard(
    time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_ocxo1_pps_projection_guard;
    case time_clock_id_t::OCXO2: return &g_ocxo2_pps_projection_guard;
    default:                    return nullptr;
  }
}

static uint64_t alpha_abs_u64_from_i64(int64_t value) {
  return (value >= 0) ? (uint64_t)value : (uint64_t)(-value);
}

static int32_t alpha_dwt32_signed_delta_near(uint32_t from_dwt32,
                                             uint32_t to_dwt32) {
  const uint32_t u = (uint32_t)(to_dwt32 - from_dwt32);
  if (u <= 0x7FFFFFFFUL) {
    return (int32_t)u;
  }
  const uint32_t magnitude = (uint32_t)((~u) + 1U);
  if (magnitude == 0x80000000UL) {
    return (int32_t)(-2147483647 - 1);
  }
  return -(int32_t)magnitude;
}

static uint64_t alpha_dwt_cycles_to_gnss_ns_u64(uint64_t dwt_cycles) {
  const uint32_t dwt_per_second = dwt_effective_cycles_per_pps_vclock_second();
  if (dwt_per_second == 0) {
    clocks_watchdog_anomaly("alpha_dwt_cps_zero_u64",
                            (uint32_t)dwt_cycles,
                            (uint32_t)(dwt_cycles >> 32),
                            0,
                            0);
    return 0;
  }
  return (dwt_cycles * NS_PER_SECOND_U64 +
          (uint64_t)dwt_per_second / 2ULL) /
         (uint64_t)dwt_per_second;
}

static int64_t alpha_dwt_cycles_to_gnss_ns_i64(int64_t dwt_cycles) {
  const uint64_t abs_cycles = alpha_abs_u64_from_i64(dwt_cycles);
  const uint64_t abs_ns = alpha_dwt_cycles_to_gnss_ns_u64(abs_cycles);
  return (dwt_cycles >= 0) ? (int64_t)abs_ns : -(int64_t)abs_ns;
}

static void alpha_bridge_anchor_record(uint32_t dwt, uint64_t gnss_ns) {
  g_bridge_anchors[g_bridge_anchor_total % ALPHA_BRIDGE_ANCHOR_RING_SIZE] =
      alpha_bridge_anchor_t{true, dwt, gnss_ns};
  g_bridge_anchor_total++;
}

static void alpha_bridge_anchor_reset(void) {
  for (uint32_t i = 0; i < ALPHA_BRIDGE_ANCHOR_RING_SIZE; i++) {
    g_bridge_anchors[i] = alpha_bridge_anchor_t{};
  }
  g_bridge_anchor_total = 0;
}

// Interpolate the GNSS nanosecond coordinate of edge_dwt between the two
// recorded anchors that bracket it.  Returns false if no bracketing pair is
// available yet (the closing anchor has not arrived, or anchors are stale).
static bool alpha_bridge_interpolate_gnss_ns(uint32_t edge_dwt,
                                             uint64_t* out_gnss_ns,
                                             int32_t* out_phi_cycles,
                                             uint32_t* out_span_cycles) {
  if (g_bridge_anchor_total < 2U) return false;
  const uint32_t available =
      g_bridge_anchor_total < ALPHA_BRIDGE_ANCHOR_RING_SIZE
          ? g_bridge_anchor_total
          : ALPHA_BRIDGE_ANCHOR_RING_SIZE;
  // Walk consecutive anchor pairs oldest -> newest.
  for (uint32_t k = g_bridge_anchor_total - available;
       k + 1U < g_bridge_anchor_total; k++) {
    const alpha_bridge_anchor_t& lo =
        g_bridge_anchors[k % ALPHA_BRIDGE_ANCHOR_RING_SIZE];
    const alpha_bridge_anchor_t& hi =
        g_bridge_anchors[(k + 1U) % ALPHA_BRIDGE_ANCHOR_RING_SIZE];
    if (!lo.valid || !hi.valid) continue;
    const int32_t from_lo = alpha_dwt32_signed_delta_near(lo.dwt, edge_dwt);
    const int32_t to_hi = alpha_dwt32_signed_delta_near(edge_dwt, hi.dwt);
    if (from_lo < 0 || to_hi <= 0) continue;
    const uint32_t span_cycles = hi.dwt - lo.dwt;
    if (span_cycles == 0 || hi.gnss_ns <= lo.gnss_ns) continue;
    const uint64_t span_ns = hi.gnss_ns - lo.gnss_ns;
    const uint64_t phi = (uint64_t)(uint32_t)from_lo;
    const uint64_t interp_ns =
        lo.gnss_ns +
        (phi * span_ns + (uint64_t)span_cycles / 2ULL) /
            (uint64_t)span_cycles;
    if (out_gnss_ns) *out_gnss_ns = interp_ns;
    if (out_phi_cycles) *out_phi_cycles = from_lo;
    if (out_span_cycles) *out_span_cycles = span_cycles;
    return true;
  }
  return false;
}

static uint32_t alpha_projection_signed_window_cycles(void) {
  const uint32_t cps = dwt_effective_cycles_per_pps_vclock_second();
  const uint32_t base = cps ? cps : (uint32_t)DWT_EXPECTED_PER_PPS;
  return base * ALPHA_OCXO_PPS_PROJECTION_MAX_SIGNED_DWT_SECONDS;
}

static void alpha_projection_guard_remember_delta(time_clock_id_t clock,
                                                  uint32_t pps_sequence,
                                                  uint32_t target_dwt32,
                                                  uint32_t reference_dwt32,
                                                  int32_t signed_delta_cycles,
                                                  uint32_t legacy_unsigned_delta_cycles) {
  alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  if (!g) return;
  g->last_pps_sequence = pps_sequence;
  g->last_target_dwt32 = target_dwt32;
  g->last_reference_dwt32 = reference_dwt32;
  g->last_signed_delta_cycles = signed_delta_cycles;
  g->last_legacy_unsigned_delta_cycles = legacy_unsigned_delta_cycles;
}

static void alpha_projection_guard_note_measured_backward(time_clock_id_t clock,
                                                          uint32_t target_dwt32,
                                                          uint32_t reference_dwt32,
                                                          int32_t signed_delta_cycles,
                                                          uint32_t legacy_unsigned_delta_cycles) {
  alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  if (!g) return;
  g->measured_projection_signed_backward_count++;
  if (legacy_unsigned_delta_cycles > alpha_projection_signed_window_cycles()) {
    g->measured_projection_legacy_unsigned_wrap_count++;
  }
  alpha_projection_guard_remember_delta(clock, 0, target_dwt32, reference_dwt32,
                                        signed_delta_cycles, legacy_unsigned_delta_cycles);
}

static void alpha_projection_guard_note_pps_backward(time_clock_id_t clock,
                                                     uint32_t pps_sequence,
                                                     uint32_t target_dwt32,
                                                     uint32_t reference_dwt32,
                                                     int32_t signed_delta_cycles,
                                                     uint32_t legacy_unsigned_delta_cycles) {
  alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  if (!g) return;
  g->pps_projection_signed_backward_count++;
  if (legacy_unsigned_delta_cycles > alpha_projection_signed_window_cycles()) {
    g->pps_projection_legacy_unsigned_wrap_count++;
  }
  alpha_projection_guard_remember_delta(clock, pps_sequence, target_dwt32, reference_dwt32,
                                        signed_delta_cycles, legacy_unsigned_delta_cycles);
}

static void alpha_projection_guard_note_sanity_reject(time_clock_id_t clock,
                                                      uint32_t pps_sequence,
                                                      uint32_t target_dwt32,
                                                      uint32_t reference_dwt32,
                                                      int64_t projected_minus_measured_ns) {
  alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  if (!g) return;
  g->pps_projection_sanity_reject_count++;
  g->last_pps_sequence = pps_sequence;
  g->last_target_dwt32 = target_dwt32;
  g->last_reference_dwt32 = reference_dwt32;
  g->last_projected_minus_measured_ns = projected_minus_measured_ns;
}

uint32_t clocks_alpha_ocxo_projection_guard_legacy_wrap_count(time_clock_id_t clock) {
  const alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  return g ? (g->measured_projection_legacy_unsigned_wrap_count +
              g->pps_projection_legacy_unsigned_wrap_count) : 0U;
}

uint32_t clocks_alpha_ocxo_projection_guard_sanity_reject_count(time_clock_id_t clock) {
  const alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
  return g ? g->pps_projection_sanity_reject_count : 0U;
}

static void alpha_ocxo_pps_projection_reset_store(
    alpha_ocxo_pps_projection_store_t& s, uint32_t clock_id) {
  s.seq++;
  clocks_alpha_dmb();
  s.v = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  s.v.clock_id = clock_id;
  clocks_alpha_dmb();
  s.seq++;
}

static void alpha_ocxo_pps_projection_reset_all(void) {
  g_ocxo1_edge_history = alpha_ocxo_edge_history_t{};
  g_ocxo2_edge_history = alpha_ocxo_edge_history_t{};
  g_ocxo1_pps_projection_guard = alpha_ocxo_pps_projection_guard_t{};
  g_ocxo2_pps_projection_guard = alpha_ocxo_pps_projection_guard_t{};
  alpha_ocxo_pps_projection_reset_store(
      g_ocxo1_pps_projection, (uint32_t)((uint8_t)time_clock_id_t::OCXO1));
  alpha_ocxo_pps_projection_reset_store(
      g_ocxo2_pps_projection, (uint32_t)((uint8_t)time_clock_id_t::OCXO2));
}

static void alpha_ocxo_edge_history_install_zero(time_clock_id_t clock,
                                                 uint32_t dwt_at_edge,
                                                 uint32_t counter32_at_edge) {
  alpha_ocxo_edge_history_t* h = alpha_ocxo_edge_history(clock);
  if (!h) return;

  *h = alpha_ocxo_edge_history_t{};
  h->current_valid = true;
  h->current.valid = true;
  h->current.dwt_at_edge = dwt_at_edge;
  h->current.dwt64_at_edge = clocks_dwt_cycles_at_dwt(dwt_at_edge);
  h->current.counter32_at_edge = counter32_at_edge;
  h->current.ocxo_ns_at_edge = 0;
  h->current.measured_ns_at_edge = 0;
}

static void alpha_ocxo_edge_history_record(time_clock_id_t clock,
                                           uint32_t dwt_at_edge,
                                           uint32_t counter32_at_edge,
                                           uint64_t ocxo_ns_at_edge,
                                           uint64_t measured_ns_at_edge,
                                           bool sample_gnss_available,
                                           uint64_t sample_gnss_ns_at_event,
                                           bool boundary_gnss_available,
                                           uint64_t boundary_gnss_ns_at_edge,
                                           uint32_t sample_phase_ticks,
                                           uint32_t phase_cycles) {
  alpha_ocxo_edge_history_t* h = alpha_ocxo_edge_history(clock);
  if (!h) return;

  alpha_ocxo_edge_fact_t fact{};
  fact.valid = true;
  fact.dwt_at_edge = dwt_at_edge;
  fact.dwt64_at_edge = h->current_valid
      ? (h->current.dwt64_at_edge +
         (uint64_t)((uint32_t)(dwt_at_edge - h->current.dwt_at_edge)))
      : clocks_dwt_cycles_at_dwt(dwt_at_edge);
  fact.counter32_at_edge = counter32_at_edge;
  fact.ocxo_ns_at_edge = ocxo_ns_at_edge;
  fact.measured_ns_at_edge = measured_ns_at_edge;
  fact.sample_gnss_available = sample_gnss_available;
  fact.sample_gnss_ns_at_event = sample_gnss_ns_at_event;
  fact.boundary_gnss_available = boundary_gnss_available;
  fact.boundary_gnss_ns_at_edge = boundary_gnss_ns_at_edge;
  fact.sample_phase_ticks = sample_phase_ticks;
  fact.phase_cycles = phase_cycles;

  h->previous = h->current;
  h->previous_valid = h->current_valid;
  h->current = fact;
  h->current_valid = true;
  h->update_count++;
}

static alpha_ocxo_edge_fact_t alpha_ocxo_project_static_next_edge(
    const alpha_ocxo_edge_fact_t& edge,
    uint32_t interval_cycles) {
  alpha_ocxo_edge_fact_t projected = edge;
  projected.dwt_at_edge = edge.dwt_at_edge + interval_cycles;
  projected.dwt64_at_edge = edge.dwt64_at_edge + (uint64_t)interval_cycles;
  projected.counter32_at_edge =
      edge.counter32_at_edge + (uint32_t)VCLOCK_COUNTS_PER_SECOND;
  projected.ocxo_ns_at_edge = edge.ocxo_ns_at_edge + NS_PER_SECOND_U64;

  // This edge is a projection, not an observed Alpha measured-ledger edge.
  projected.measured_ns_at_edge = 0;
  projected.sample_gnss_available = false;
  projected.sample_gnss_ns_at_event = 0;
  projected.boundary_gnss_available = false;
  projected.boundary_gnss_ns_at_edge = 0;
  return projected;
}

static uint32_t alpha_ocxo_latest_actual_interval_cycles(
    time_clock_id_t clock,
    uint32_t* out_completed_interval_count,
    bool* out_static_prediction_valid) {
  const alpha_static_prediction_store_t* s = alpha_static_prediction_store(clock);
  if (!s) {
    if (out_completed_interval_count) *out_completed_interval_count = 0;
    if (out_static_prediction_valid) *out_static_prediction_valid = false;
    return 0;
  }

  if (out_completed_interval_count) {
    *out_completed_interval_count = s->completed_interval_count;
  }
  if (out_static_prediction_valid) {
    *out_static_prediction_valid = s->valid;
  }
  return s->actual_cycles;
}

static void alpha_ocxo_pps_projection_set_invalid(
    time_clock_id_t clock,
    uint32_t reason,
    uint32_t pps_sequence,
    uint32_t pps_dwt_at_edge,
    uint64_t pps_vclock_ns,
    uint64_t existing_pps_ns,
    const alpha_ocxo_edge_fact_t* edge0 = nullptr,
    const alpha_ocxo_edge_fact_t* edge1 = nullptr,
    uint32_t latest_actual_interval_cycles = 0,
    uint32_t completed_interval_count = 0,
    bool static_prediction_valid = false,
    uint32_t target_delta_raw_cycles = 0,
    uint32_t target_overrun_cycles = 0,
    uint32_t last_static_advance_count = 0) {
  alpha_ocxo_pps_projection_store_t* s = alpha_ocxo_pps_projection_store(clock);
  if (!s) return;

  const uint32_t prior_update_count = s->v.update_count;
  const uint32_t prior_compute_count = s->v.compute_count;
  const uint32_t no_edge_count = s->v.invalid_no_edge_count;
  const uint32_t no_interval_count = s->v.invalid_no_interval_count;
  const uint32_t out_of_window_count = s->v.invalid_target_out_of_window_count;
  const uint32_t prior_static_advance_total = s->v.static_projection_advance_count;
  const uint32_t prior_static_advance_max = s->v.max_static_projection_advance_count;
  const uint32_t prior_target_overrun_max = s->v.max_target_overrun_cycles;

  clocks_alpha_ocxo_pps_projection_snapshot_t local{};
  local.clock_id = (uint32_t)((uint8_t)clock);
  local.update_count = prior_update_count + 1U;
  local.compute_count = prior_compute_count + 1U;
  local.invalid_no_edge_count = no_edge_count +
      ((reason == ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_EDGE) ? 1U : 0U);
  local.invalid_no_interval_count = no_interval_count +
      ((reason == ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_INTERVAL) ? 1U : 0U);
  local.invalid_target_out_of_window_count = out_of_window_count +
      ((reason == ALPHA_OCXO_PPS_PROJECTION_INVALID_TARGET_OUT_OF_WINDOW) ? 1U : 0U);
  local.valid = false;
  local.source = ALPHA_OCXO_PPS_PROJECTION_SOURCE_NONE;
  local.last_invalid_reason = reason;
  local.pps_sequence = pps_sequence;
  local.pps_dwt_at_edge = pps_dwt_at_edge;
  local.pps_vclock_ns = pps_vclock_ns;

  if (edge0) {
    local.edge0_dwt_at_edge = edge0->dwt_at_edge;
    local.edge0_counter32_at_edge = edge0->counter32_at_edge;
    local.edge0_ocxo_ns_at_edge = edge0->ocxo_ns_at_edge;
    local.edge0_measured_ns_at_edge = edge0->measured_ns_at_edge;
    local.edge0_sample_gnss_available = edge0->sample_gnss_available;
    local.edge0_sample_gnss_ns_at_event = edge0->sample_gnss_ns_at_event;
    local.edge0_boundary_gnss_available = edge0->boundary_gnss_available;
    local.edge0_boundary_gnss_ns_at_edge = edge0->boundary_gnss_ns_at_edge;
  }
  if (edge1) {
    local.edge1_dwt_at_edge = edge1->dwt_at_edge;
    local.edge1_counter32_at_edge = edge1->counter32_at_edge;
    local.edge1_ocxo_ns_at_edge = edge1->ocxo_ns_at_edge;
    local.edge1_measured_ns_at_edge = edge1->measured_ns_at_edge;
    local.edge1_sample_gnss_available = edge1->sample_gnss_available;
    local.edge1_sample_gnss_ns_at_event = edge1->sample_gnss_ns_at_event;
    local.edge1_boundary_gnss_available = edge1->boundary_gnss_available;
    local.edge1_boundary_gnss_ns_at_edge = edge1->boundary_gnss_ns_at_edge;
  }

  local.projected_minus_existing_pps_ns = 0;
  local.projected_minus_vclock_ns = 0;
  local.latest_actual_interval_cycles = latest_actual_interval_cycles;
  local.static_prediction_completed_interval_count = completed_interval_count;
  local.static_prediction_valid = static_prediction_valid;
  local.static_projection_advance_count =
      prior_static_advance_total + last_static_advance_count;
  local.last_static_projection_advance_count = last_static_advance_count;
  local.max_static_projection_advance_count =
      (last_static_advance_count > prior_static_advance_max)
          ? last_static_advance_count
          : prior_static_advance_max;
  local.static_projection_advance_limit = ALPHA_OCXO_PPS_STATIC_ADVANCE_LIMIT;
  local.target_delta_raw_cycles = target_delta_raw_cycles;
  local.target_overrun_cycles = target_overrun_cycles;
  local.max_target_overrun_cycles =
      (target_overrun_cycles > prior_target_overrun_max)
          ? target_overrun_cycles
          : prior_target_overrun_max;

  (void)existing_pps_ns;

  s->seq++;
  clocks_alpha_dmb();
  s->v = local;
  clocks_alpha_dmb();
  s->seq++;
}

static bool alpha_ocxo_pps_projection_build(
    time_clock_id_t clock,
    uint32_t source,
    const alpha_ocxo_edge_fact_t& edge0,
    const alpha_ocxo_edge_fact_t& edge1,
    uint32_t interval_cycles,
    uint32_t target_delta_cycles,
    uint32_t pps_sequence,
    uint32_t pps_dwt_at_edge,
    uint64_t pps_vclock_ns,
    uint64_t existing_pps_ns,
    uint32_t latest_actual_interval_cycles,
    uint32_t completed_interval_count,
    bool static_prediction_valid,
    uint32_t last_static_advance_count,
    uint32_t target_delta_raw_cycles,
    uint32_t target_overrun_cycles) {
  alpha_ocxo_pps_projection_store_t* s = alpha_ocxo_pps_projection_store(clock);
  if (!s || interval_cycles == 0) return false;

  const uint64_t interval_ocxo_ns = (edge1.ocxo_ns_at_edge >= edge0.ocxo_ns_at_edge)
      ? (edge1.ocxo_ns_at_edge - edge0.ocxo_ns_at_edge)
      : 0ULL;
  if (interval_ocxo_ns == 0) return false;

  const uint64_t projected_delta_ns =
      ((uint64_t)target_delta_cycles * interval_ocxo_ns +
       (uint64_t)interval_cycles / 2ULL) /
      (uint64_t)interval_cycles;
  const uint64_t projected_ns = edge0.ocxo_ns_at_edge + projected_delta_ns;

  const int64_t projected_minus_existing =
      alpha_signed_delta_u64(projected_ns, existing_pps_ns);

  // Guard the *jump* in the projection-vs-measured diagnostic offset, not the
  // absolute offset itself.  IRQ5 proved the absolute offset can be hundreds of
  // milliseconds after a clean START/SmartZero base selection while the target
  // geometry is perfectly valid.  The original 2^44 pathology was a one-row
  // discontinuity in this offset, so that is the invariant we enforce.
  alpha_ocxo_pps_projection_guard_t* guard =
      alpha_ocxo_pps_projection_guard(clock);
  if (existing_pps_ns != 0 && guard &&
      guard->last_projected_minus_measured_valid) {
    const int64_t offset_jump =
        projected_minus_existing - guard->last_projected_minus_measured_ns;
    guard->last_projected_minus_measured_jump_ns = offset_jump;

    if (alpha_abs_u64_from_i64(offset_jump) >
        ALPHA_OCXO_PPS_PROJECTION_SANITY_WINDOW_NS) {
      alpha_projection_guard_note_sanity_reject(clock,
                                                pps_sequence,
                                                pps_dwt_at_edge,
                                                edge0.dwt_at_edge,
                                                offset_jump);
      // Heal: adopt the new offset as the baseline so this guard rejects a
      // single discontinuity, not every subsequent second.  A non-healing
      // latch here turned a one-time ledger-units change into a permanent
      // projection outage (Envelope1).  Defenseless doctrine: a guard may
      // veto one bad row, never silently disable the surface forever.
      guard->last_projected_minus_measured_ns = projected_minus_existing;
      guard->last_projected_minus_measured_jump_ns = offset_jump;
      alpha_ocxo_pps_projection_set_invalid(
          clock,
          ALPHA_OCXO_PPS_PROJECTION_INVALID_TARGET_OUT_OF_WINDOW,
          pps_sequence,
          pps_dwt_at_edge,
          pps_vclock_ns,
          existing_pps_ns,
          &edge0,
          &edge1,
          latest_actual_interval_cycles,
          completed_interval_count,
          static_prediction_valid,
          target_delta_raw_cycles,
          target_overrun_cycles,
          last_static_advance_count);
      return true;
    }
  }

  clocks_alpha_ocxo_pps_projection_snapshot_t local = s->v;
  local.valid = true;
  local.clock_id = (uint32_t)((uint8_t)clock);
  local.update_count++;
  local.compute_count++;
  local.source = source;
  local.last_invalid_reason = ALPHA_OCXO_PPS_PROJECTION_INVALID_NONE;
  local.pps_sequence = pps_sequence;
  local.pps_dwt_at_edge = pps_dwt_at_edge;
  local.pps_vclock_ns = pps_vclock_ns;

  local.edge0_dwt_at_edge = edge0.dwt_at_edge;
  local.edge0_counter32_at_edge = edge0.counter32_at_edge;
  local.edge0_ocxo_ns_at_edge = edge0.ocxo_ns_at_edge;
  local.edge0_measured_ns_at_edge = edge0.measured_ns_at_edge;
  local.edge0_sample_gnss_available = edge0.sample_gnss_available;
  local.edge0_sample_gnss_ns_at_event = edge0.sample_gnss_ns_at_event;
  local.edge0_boundary_gnss_available = edge0.boundary_gnss_available;
  local.edge0_boundary_gnss_ns_at_edge = edge0.boundary_gnss_ns_at_edge;

  local.edge1_dwt_at_edge = edge1.dwt_at_edge;
  local.edge1_counter32_at_edge = edge1.counter32_at_edge;
  local.edge1_ocxo_ns_at_edge = edge1.ocxo_ns_at_edge;
  local.edge1_measured_ns_at_edge = edge1.measured_ns_at_edge;
  local.edge1_sample_gnss_available = edge1.sample_gnss_available;
  local.edge1_sample_gnss_ns_at_event = edge1.sample_gnss_ns_at_event;
  local.edge1_boundary_gnss_available = edge1.boundary_gnss_available;
  local.edge1_boundary_gnss_ns_at_edge = edge1.boundary_gnss_ns_at_edge;

  local.interval_dwt_cycles = interval_cycles;
  local.interval_ocxo_ns = interval_ocxo_ns;
  local.target_delta_cycles = target_delta_cycles;
  local.target_remaining_cycles = interval_cycles - target_delta_cycles;
  local.projected_ocxo_ns_at_pps = projected_ns;
  local.projected_minus_existing_pps_ns = projected_minus_existing;
  if (guard && existing_pps_ns != 0) {
    guard->last_projected_minus_measured_valid = true;
    guard->last_projected_minus_measured_ns = projected_minus_existing;
    guard->last_projected_minus_measured_jump_ns = 0;
  }
  local.projected_minus_vclock_ns =
      alpha_signed_delta_u64(projected_ns, pps_vclock_ns);
  local.latest_actual_interval_cycles = latest_actual_interval_cycles;
  local.static_prediction_completed_interval_count = completed_interval_count;
  local.static_prediction_valid = static_prediction_valid;
  local.static_projection_advance_count += last_static_advance_count;
  local.last_static_projection_advance_count = last_static_advance_count;
  if (last_static_advance_count > local.max_static_projection_advance_count) {
    local.max_static_projection_advance_count = last_static_advance_count;
  }
  local.static_projection_advance_limit = ALPHA_OCXO_PPS_STATIC_ADVANCE_LIMIT;
  local.target_delta_raw_cycles = target_delta_raw_cycles;
  local.target_overrun_cycles = target_overrun_cycles;
  if (target_overrun_cycles > local.max_target_overrun_cycles) {
    local.max_target_overrun_cycles = target_overrun_cycles;
  }

  s->seq++;
  clocks_alpha_dmb();
  s->v = local;
  clocks_alpha_dmb();
  s->seq++;
  return true;
}

static void alpha_ocxo_pps_projection_compute(time_clock_id_t clock,
                                              uint32_t pps_sequence,
                                              uint32_t pps_dwt_at_edge,
                                              uint64_t pps_vclock_ns,
                                              uint64_t existing_pps_ns) {
  const alpha_ocxo_edge_history_t* h = alpha_ocxo_edge_history(clock);
  if (!h || !h->current_valid) {
    alpha_ocxo_pps_projection_set_invalid(
        clock, ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_EDGE,
        pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns);
    return;
  }

  uint32_t completed_interval_count = 0;
  bool static_prediction_valid = false;
  const uint32_t latest_actual_interval_cycles =
      alpha_ocxo_latest_actual_interval_cycles(
          clock, &completed_interval_count, &static_prediction_valid);

  // Actual bracket first.  This is an ordered modular-DWT relationship between
  // two consecutive OCXO edge facts.  Unsigned subtraction is the correct
  // geometry here, including across a DWT32 wrap.  Do not apply the "signed
  // near" helper as a veto: a lawful forward target may occupy a modular
  // position that looks negative in half-range signed arithmetic.
  if (h->previous_valid && h->previous.valid && h->current.valid &&
      h->current.dwt64_at_edge >= h->previous.dwt64_at_edge) {
    const uint64_t actual_interval64 =
        h->current.dwt64_at_edge - h->previous.dwt64_at_edge;
    const uint32_t target_delta_from_prev =
        pps_dwt_at_edge - h->previous.dwt_at_edge;

    if (actual_interval64 != 0 &&
        actual_interval64 <= 0xFFFFFFFFULL &&
        (uint64_t)target_delta_from_prev <= actual_interval64) {
      if (alpha_ocxo_pps_projection_build(
              clock, ALPHA_OCXO_PPS_PROJECTION_SOURCE_ACTUAL_BRACKET,
              h->previous, h->current, (uint32_t)actual_interval64,
              target_delta_from_prev,
              pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
              latest_actual_interval_cycles, completed_interval_count,
              static_prediction_valid,
              0U, target_delta_from_prev, 0U)) {
        return;
      }
    }
  }

  if (latest_actual_interval_cycles == 0) {
    const uint32_t legacy_delta = pps_dwt_at_edge - h->current.dwt_at_edge;
    alpha_ocxo_pps_projection_set_invalid(
        clock, ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_INTERVAL,
        pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
        &h->current, h->previous_valid ? &h->previous : nullptr,
        latest_actual_interval_cycles, completed_interval_count,
        static_prediction_valid,
        legacy_delta, 0U, 0U);
    return;
  }

  // Static projection path.
  //
  // The PPS/VCLOCK target can be one or two OCXO one-second intervals beyond
  // the most recent observed OCXO edge.  At 1.008 GHz, two legal forward DWT
  // intervals can exceed 2^31 cycles, so signed-near DWT classification is the
  // wrong validity test.  Treat DWT32 as an ordered modular coordinate and
  // repeatedly advance the synthetic edge by the measured one-second interval.
  // If the target lands inside the bounded projected interval, the projection
  // is valid.  If it still does not land after ALPHA_OCXO_PPS_STATIC_ADVANCE_LIMIT
  // advances, only then declare TARGET_OUT_OF_WINDOW.
  const uint32_t raw_target_delta =
      pps_dwt_at_edge - h->current.dwt_at_edge;
  const uint32_t raw_target_overrun =
      (raw_target_delta > latest_actual_interval_cycles)
          ? (raw_target_delta - latest_actual_interval_cycles)
          : 0U;

  alpha_ocxo_edge_fact_t edge0 = h->current;
  uint32_t target_delta = raw_target_delta;
  uint32_t static_advance_count = 0;

  while (target_delta > latest_actual_interval_cycles &&
         static_advance_count < ALPHA_OCXO_PPS_STATIC_ADVANCE_LIMIT) {
    edge0 = alpha_ocxo_project_static_next_edge(edge0,
                                                latest_actual_interval_cycles);
    static_advance_count++;
    target_delta = pps_dwt_at_edge - edge0.dwt_at_edge;
  }

  if (target_delta > latest_actual_interval_cycles) {
    const uint32_t unresolved_overrun =
        target_delta - latest_actual_interval_cycles;
    alpha_ocxo_pps_projection_set_invalid(
        clock, ALPHA_OCXO_PPS_PROJECTION_INVALID_TARGET_OUT_OF_WINDOW,
        pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
        &edge0, nullptr,
        latest_actual_interval_cycles, completed_interval_count,
        static_prediction_valid,
        raw_target_delta, unresolved_overrun, static_advance_count);
    return;
  }

  alpha_ocxo_edge_fact_t projected_next =
      alpha_ocxo_project_static_next_edge(edge0, latest_actual_interval_cycles);

  if (!alpha_ocxo_pps_projection_build(
          clock, ALPHA_OCXO_PPS_PROJECTION_SOURCE_STATIC_NEXT_EDGE,
          edge0, projected_next, latest_actual_interval_cycles,
          target_delta, pps_sequence, pps_dwt_at_edge, pps_vclock_ns,
          existing_pps_ns, latest_actual_interval_cycles,
          completed_interval_count, static_prediction_valid,
          static_advance_count, raw_target_delta, raw_target_overrun)) {
    alpha_ocxo_pps_projection_set_invalid(
        clock, ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_INTERVAL,
        pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
        &edge0, &projected_next,
        latest_actual_interval_cycles, completed_interval_count,
        static_prediction_valid,
        raw_target_delta, raw_target_overrun, static_advance_count);
  }
}

bool clocks_alpha_ocxo_pps_projection_snapshot(
    time_clock_id_t clock,
    clocks_alpha_ocxo_pps_projection_snapshot_t* out) {
  if (!out) return false;
  alpha_ocxo_pps_projection_store_t* s = alpha_ocxo_pps_projection_store(clock);
  if (!s) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    clocks_alpha_dmb();
    clocks_alpha_ocxo_pps_projection_snapshot_t local = s->v;
    clocks_alpha_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.valid;
    }
  }

  *out = clocks_alpha_ocxo_pps_projection_snapshot_t{};
  return false;
}

static bool alpha_ocxo_pps_projection_visible_ns(time_clock_id_t clock,
                                                 uint64_t* out_ns) {
  if (!out_ns) return false;
  *out_ns = 0ULL;

  clocks_alpha_ocxo_pps_projection_snapshot_t s{};
  if (!clocks_alpha_ocxo_pps_projection_snapshot(clock, &s)) return false;
  if (!s.valid || s.projected_ocxo_ns_at_pps == 0ULL) return false;

  *out_ns = s.projected_ocxo_ns_at_pps;
  return true;
}

static alpha_lane_forensics_store_t* alpha_forensics_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return &g_vclock_forensics;
    case time_clock_id_t::OCXO1:  return &g_ocxo1_forensics;
    case time_clock_id_t::OCXO2:  return &g_ocxo2_forensics;
    default:                     return nullptr;
  }
}

static alpha_lane_logical_ticks_t* alpha_ticks64_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::VCLOCK: return &g_vclock_ticks64;
    case time_clock_id_t::OCXO1:  return &g_ocxo1_ticks64;
    case time_clock_id_t::OCXO2:  return &g_ocxo2_ticks64;
    default:                     return nullptr;
  }
}

static void alpha_ticks64_reset_store(alpha_lane_logical_ticks_t& s) {
  s = alpha_lane_logical_ticks_t{};
}

static void alpha_ticks64_reset_all(void) {
  alpha_ticks64_reset_store(g_vclock_ticks64);
  alpha_ticks64_reset_store(g_ocxo1_ticks64);
  alpha_ticks64_reset_store(g_ocxo2_ticks64);
}

static void alpha_ticks64_install_zero(alpha_lane_logical_ticks_t& s,
                                       uint32_t zero_counter32) {
  s = alpha_lane_logical_ticks_t{};
  s.zero_counter32 = zero_counter32;
  s.last_counter32 = zero_counter32;
}

static bool alpha_ticks64_apply_event(time_clock_id_t clock,
                                      uint32_t event_counter32,
                                      uint32_t fallback_zero_offset_counter32,
                                      uint64_t* out_ticks64,
                                      uint64_t* out_ns64,
                                      uint32_t* out_delta_since_zero_offset,
                                      uint32_t* out_delta_since_previous_event) {
  alpha_lane_logical_ticks_t* s = alpha_ticks64_store(clock);
  if (!s) return false;

  const uint32_t delta_since_zero = event_counter32 - s->zero_counter32;
  const uint32_t delta_since_previous = event_counter32 - s->last_counter32;

  s->ticks64 += (uint64_t)delta_since_previous;
  s->last_counter32 = event_counter32;
  s->counter32_delta_since_previous_event = delta_since_previous;
  s->update_count++;

  if (out_ticks64) *out_ticks64 = s->ticks64;
  if (out_ns64) *out_ns64 = s->ticks64 * (uint64_t)NS_PER_10MHZ_TICK;
  if (out_delta_since_zero_offset) *out_delta_since_zero_offset = delta_since_zero;
  if (out_delta_since_previous_event) *out_delta_since_previous_event = delta_since_previous;

  (void)fallback_zero_offset_counter32;
  return true;
}

static bool alpha_zero_offset_valid(time_clock_id_t clock) {
  return alpha_ticks64_store(clock) != nullptr;
}

static bool alpha_ticks64_project_counter32(time_clock_id_t clock,
                                            uint32_t counter32,
                                            uint64_t* out_ticks64,
                                            uint64_t* out_ns64) {
  const alpha_lane_logical_ticks_t* s = alpha_ticks64_store(clock);
  if (!s) return false;

  const uint64_t ticks =
      s->ticks64 + (uint64_t)((uint32_t)(counter32 - s->last_counter32));

  if (out_ticks64) *out_ticks64 = ticks;
  if (out_ns64) *out_ns64 = ticks * (uint64_t)NS_PER_10MHZ_TICK;
  return true;
}

static void alpha_measured_ns_reset_all(void) {
  alpha_bridge_anchor_reset();
  g_ocxo1_measured_ns = alpha_measured_ns_clock_t{};
  g_ocxo2_measured_ns = alpha_measured_ns_clock_t{};
}

static alpha_measured_ns_clock_t* alpha_measured_ns_store(time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_ocxo1_measured_ns;
    case time_clock_id_t::OCXO2: return &g_ocxo2_measured_ns;
    default:                    return nullptr;
  }
}

static uint64_t alpha_dwt_cycles_to_gnss_ns(uint32_t dwt_cycles) {
  return alpha_dwt_cycles_to_gnss_ns_u64((uint64_t)dwt_cycles);
}

static uint64_t alpha_ocxo_seed_ns_at_first_edge(uint32_t raw_edge_dwt) {
  const uint32_t epoch_dwt = clocks_alpha_epoch_last_dwt_at_edge();
  if (epoch_dwt == 0) return 0;
  return alpha_dwt_cycles_to_gnss_ns(raw_edge_dwt - epoch_dwt);
}

struct alpha_ocxo_visible_origin_state_t {
  volatile uint32_t seq = 0;
  clocks_alpha_ocxo_visible_origin_snapshot_t v = {};
};

static alpha_ocxo_visible_origin_state_t g_ocxo1_visible_origin = {};
static alpha_ocxo_visible_origin_state_t g_ocxo2_visible_origin = {};

static alpha_ocxo_visible_origin_state_t* alpha_ocxo_visible_origin_store(
    time_clock_id_t clock) {
  switch (clock) {
    case time_clock_id_t::OCXO1: return &g_ocxo1_visible_origin;
    case time_clock_id_t::OCXO2: return &g_ocxo2_visible_origin;
    default:                    return nullptr;
  }
}

static uint32_t alpha_ocxo_visible_origin_clock_id(time_clock_id_t clock) {
  return (uint32_t)((uint8_t)clock);
}

static void alpha_ocxo_visible_origin_publish(
    alpha_ocxo_visible_origin_state_t& s,
    const clocks_alpha_ocxo_visible_origin_snapshot_t& local) {
  s.seq++;
  clocks_alpha_dmb();
  s.v = local;
  clocks_alpha_dmb();
  s.seq++;
}

static void alpha_ocxo_visible_origin_reset_store(
    alpha_ocxo_visible_origin_state_t& s,
    time_clock_id_t clock) {
  clocks_alpha_ocxo_visible_origin_snapshot_t local{};
  local.clock_id = alpha_ocxo_visible_origin_clock_id(clock);
  alpha_ocxo_visible_origin_publish(s, local);
}

static void alpha_ocxo_visible_origin_reset_all(void) {
  alpha_ocxo_visible_origin_reset_store(g_ocxo1_visible_origin,
                                        time_clock_id_t::OCXO1);
  alpha_ocxo_visible_origin_reset_store(g_ocxo2_visible_origin,
                                        time_clock_id_t::OCXO2);
}

static uint32_t alpha_ocxo_visible_origin_select_cps(
    const interrupt_smartzero_lane_snapshot_t& vclock,
    const interrupt_smartzero_lane_snapshot_t& ocxo) {
  if (ocxo.cps_used != 0U) return ocxo.cps_used;
  if (vclock.cps_used != 0U) return vclock.cps_used;

  const uint32_t live_cps = dwt_effective_cycles_per_pps_vclock_second();
  return live_cps ? live_cps : (uint32_t)DWT_EXPECTED_PER_PPS;
}

static uint64_t alpha_ocxo_visible_origin_delta_ns(
    uint32_t elapsed_cycles,
    uint32_t dwt_cycles_per_second) {
  if (dwt_cycles_per_second == 0U) return 0ULL;
  return ((uint64_t)elapsed_cycles * NS_PER_SECOND_U64 +
          (uint64_t)dwt_cycles_per_second / 2ULL) /
         (uint64_t)dwt_cycles_per_second;
}

static void alpha_ocxo_visible_origin_capture_from_smartzero(
    time_clock_id_t clock,
    uint32_t epoch_sequence,
    uint32_t smartzero_sequence,
    uint32_t pps_vclock_dwt,
    uint32_t ocxo_anchor_dwt,
    uint32_t dwt_cycles_per_second) {
  alpha_ocxo_visible_origin_state_t* s =
      alpha_ocxo_visible_origin_store(clock);
  if (!s) return;

  const bool basis_valid =
      pps_vclock_dwt != 0U &&
      ocxo_anchor_dwt != 0U &&
      dwt_cycles_per_second != 0U;
  const uint32_t elapsed_cycles = basis_valid
      ? (uint32_t)(ocxo_anchor_dwt - pps_vclock_dwt)
      : 0U;
  const uint64_t elapsed_ns = basis_valid
      ? alpha_ocxo_visible_origin_delta_ns(elapsed_cycles,
                                           dwt_cycles_per_second)
      : 0ULL;
  const uint32_t phase_ns = basis_valid
      ? (uint32_t)(elapsed_ns % (uint64_t)NS_PER_10MHZ_TICK)
      : 0U;

  clocks_alpha_ocxo_visible_origin_snapshot_t local{};
  local.valid = basis_valid;
  local.pending = !basis_valid;
  local.phase_offset_in_range =
      basis_valid && phase_ns < (uint32_t)NS_PER_10MHZ_TICK;
  local.clock_id = alpha_ocxo_visible_origin_clock_id(clock);
  local.epoch_sequence = epoch_sequence;
  local.smartzero_sequence = smartzero_sequence;
  local.capture_count = s->v.capture_count + (basis_valid ? 1U : 0U);
  local.pps_vclock_dwt = pps_vclock_dwt;
  local.ocxo_anchor_dwt = ocxo_anchor_dwt;
  local.dwt_cycles_per_second = dwt_cycles_per_second;
  local.elapsed_cycles_since_pps_vclock = elapsed_cycles;
  local.elapsed_ns_since_pps_vclock = elapsed_ns;
  local.phase_offset_ns = phase_ns;
  alpha_ocxo_visible_origin_publish(*s, local);
}

bool clocks_alpha_ocxo_visible_origin_snapshot(
    time_clock_id_t clock,
    clocks_alpha_ocxo_visible_origin_snapshot_t* out) {
  if (!out) return false;
  alpha_ocxo_visible_origin_state_t* s =
      alpha_ocxo_visible_origin_store(clock);
  if (!s) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    clocks_alpha_dmb();
    clocks_alpha_ocxo_visible_origin_snapshot_t local = s->v;
    clocks_alpha_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.valid || local.pending;
    }
  }

  *out = clocks_alpha_ocxo_visible_origin_snapshot_t{};
  return false;
}

static bool alpha_ocxo_visible_origin_phase_offset_ns(time_clock_id_t clock,
                                                      uint32_t* out_phase_ns) {
  alpha_ocxo_visible_origin_state_t* s =
      alpha_ocxo_visible_origin_store(clock);
  if (!s) return false;

  const clocks_alpha_ocxo_visible_origin_snapshot_t local = s->v;
  const bool ok =
      local.valid &&
      local.phase_offset_in_range &&
      local.phase_offset_ns < (uint32_t)NS_PER_10MHZ_TICK;
  if (ok && out_phase_ns) {
    *out_phase_ns = local.phase_offset_ns;
  }
  return ok;
}

static uint64_t alpha_ocxo_visible_ns_from_physical(time_clock_id_t clock,
                                                    uint64_t physical_ns) {
  uint32_t phase_ns = 0;
  if (!alpha_ocxo_visible_origin_phase_offset_ns(clock, &phase_ns)) {
    return physical_ns;
  }

  // phase_offset_ns is the OCXO edge displacement after the selected
  // PPS/VCLOCK origin inside one 10 MHz cell.  Adding it makes the OCXO
  // coordinate project back to the visible VCLOCK/PPS origin instead of
  // exposing the arbitrary starting phase.
  return physical_ns + (uint64_t)phase_ns;
}

static int64_t alpha_ocxo_signed_delta_ns(uint64_t target_ns,
                                          uint64_t source_ns) {
  return (target_ns >= source_ns)
      ? (int64_t)(target_ns - source_ns)
      : -(int64_t)(source_ns - target_ns);
}

static uint64_t alpha_ocxo_apply_signed_offset_ns(uint64_t ns,
                                                  int64_t offset_ns) {
  if (offset_ns >= 0) {
    return ns + (uint64_t)offset_ns;
  }

  const uint64_t magnitude = (uint64_t)(-offset_ns);
  return (ns >= magnitude) ? (ns - magnitude) : 0ULL;
}

static bool alpha_ocxo_visible_origin_public_offset_ns(time_clock_id_t clock,
                                                       int64_t* out_offset_ns) {
  alpha_ocxo_visible_origin_state_t* s =
      alpha_ocxo_visible_origin_store(clock);
  if (!s) return false;

  const clocks_alpha_ocxo_visible_origin_snapshot_t local = s->v;
  if (!local.public_origin_valid) return false;
  if (out_offset_ns) *out_offset_ns = local.public_origin_offset_ns;
  return true;
}

static uint64_t alpha_ocxo_public_ns_from_visible_origin(time_clock_id_t clock,
                                                         uint64_t visible_ns) {
  int64_t offset_ns = 0;
  if (!alpha_ocxo_visible_origin_public_offset_ns(clock, &offset_ns)) {
    return visible_ns;
  }
  return alpha_ocxo_apply_signed_offset_ns(visible_ns, offset_ns);
}

static void alpha_ocxo_visible_origin_maybe_capture_public_origin(
    time_clock_id_t clock,
    uint32_t pps_sequence,
    uint64_t vclock_ns,
    uint64_t ocxo_visible_ns_before_offset) {
  alpha_ocxo_visible_origin_state_t* s =
      alpha_ocxo_visible_origin_store(clock);
  if (!s) return;

  clocks_alpha_ocxo_visible_origin_snapshot_t local = s->v;
  if (!local.valid || local.pending || local.public_origin_valid) return;
  if (vclock_ns == 0ULL || ocxo_visible_ns_before_offset == 0ULL) return;

  const int64_t offset_ns =
      alpha_ocxo_signed_delta_ns(vclock_ns, ocxo_visible_ns_before_offset);
  const uint64_t after_offset =
      alpha_ocxo_apply_signed_offset_ns(ocxo_visible_ns_before_offset,
                                        offset_ns);

  local.public_origin_valid = true;
  local.public_origin_capture_count++;
  local.public_origin_pps_sequence = pps_sequence;
  local.public_origin_vclock_ns = vclock_ns;
  local.public_origin_ocxo_ns_before_offset = ocxo_visible_ns_before_offset;
  local.public_origin_offset_ns = offset_ns;
  local.public_origin_ocxo_ns_after_offset = after_offset;

  alpha_ocxo_visible_origin_publish(*s, local);
}

static uint64_t alpha_ocxo_apply_measured_second(time_clock_id_t clock,
                                                 uint32_t raw_edge_dwt,
                                                 uint32_t* out_dwt_cycles,
                                                 uint64_t* out_real_interval_ns,
                                                 int64_t* out_residual_fast_ns) {
  alpha_measured_ns_clock_t* m = alpha_measured_ns_store(clock);
  if (!m) {
    alpha_event_flow_note_measured_store_missing(clock);
    clocks_watchdog_anomaly("alpha_ocxo_measured_store_missing",
                            (uint32_t)((uint8_t)clock), raw_edge_dwt, 0, 0);
    return 0;
  }

  if (out_dwt_cycles) *out_dwt_cycles = 0;
  if (out_real_interval_ns) *out_real_interval_ns = 0;
  if (out_residual_fast_ns) *out_residual_fast_ns = 0;

  // Resolve the pending edge against the bridge anchors.  The closing
  // anchor for edge E arrives one second after E, and always before the
  // next OCXO edge in normal operation, so resolution happens here with a
  // one-edge lag.  Resolved coordinates are ABSOLUTE GNSS interpolations:
  // the measured ledger is re-anchored to truth every second and errors do
  // not integrate.
  if (m->pending_valid) {
    uint64_t pending_gnss_ns = 0;
    int32_t phi_cycles = 0;
    uint32_t span_cycles = 0;
    const bool bracketed = alpha_bridge_interpolate_gnss_ns(
        m->pending_edge_dwt, &pending_gnss_ns, &phi_cycles, &span_cycles);

    bool resolve_legacy = false;
    if (!bracketed) {
      m->pending_age_events++;
      if (m->pending_age_events >= ALPHA_BRIDGE_PENDING_MAX_AGE_EVENTS) {
        // GNSS holdover: anchors stalled.  Fall back to legacy whole-second
        // ratio conversion for this edge, truthfully counted.  This is the
        // pre-bridge math and carries its ruler exposure; the counter is the
        // audit trail.
        resolve_legacy = true;
      }
    }

    if (bracketed || resolve_legacy) {
      uint64_t resolved_ns = pending_gnss_ns;
      if (resolve_legacy) {
        const uint32_t legacy_cycles = m->pending_edge_dwt - m->dwt_at_edge;
        resolved_ns = m->initialized
            ? m->ns_at_edge + alpha_dwt_cycles_to_gnss_ns(legacy_cycles)
            : alpha_ocxo_seed_ns_at_first_edge(m->pending_edge_dwt);
        m->bridge_fallback_count++;
        m->last_resolved_via_bridge = false;
      } else {
        m->bridge_resolved_count++;
        m->last_resolved_via_bridge = true;
        m->bridge_last_phi_cycles = phi_cycles;
        m->bridge_last_span_cycles = span_cycles;
      }

      if (m->initialized) {
        const uint32_t dwt_cycles = m->pending_edge_dwt - m->dwt_at_edge;
        if (resolved_ns > m->ns_at_edge) {
          const uint64_t real_interval_ns = resolved_ns - m->ns_at_edge;
          const int64_t residual_fast_ns =
              (int64_t)NS_PER_SECOND_U64 - (int64_t)real_interval_ns;
          if (out_dwt_cycles) *out_dwt_cycles = dwt_cycles;
          if (out_real_interval_ns) *out_real_interval_ns = real_interval_ns;
          if (out_residual_fast_ns) *out_residual_fast_ns = residual_fast_ns;
        } else {
          clocks_watchdog_anomaly("alpha_bridge_nonmonotonic_ns",
                                  (uint32_t)((uint8_t)clock),
                                  m->pending_edge_dwt,
                                  (uint32_t)resolved_ns,
                                  (uint32_t)m->ns_at_edge);
        }
        m->dwt64_at_edge += (uint64_t)dwt_cycles;
      } else {
        m->initialized = true;
        m->dwt64_at_edge = clocks_dwt_cycles_at_dwt(m->pending_edge_dwt);
      }
      m->ns_at_edge = resolved_ns;
      m->dwt_at_edge = m->pending_edge_dwt;
      m->pending_valid = false;
      m->pending_age_events = 0;
    }
  }

  // Stash this edge as the new pending observation.  If an old pending is
  // still unresolved (should not happen below the age guard), force the
  // legacy path next call rather than silently dropping evidence.
  if (!m->pending_valid) {
    m->pending_edge_dwt = raw_edge_dwt;
    m->pending_valid = true;
    m->pending_age_events = 0;
  } else {
    m->pending_age_events = ALPHA_BRIDGE_PENDING_MAX_AGE_EVENTS;
  }

  if (!m->initialized) {
    // Display-only continuity during the one-to-two second bridge warm-up.
    return alpha_ocxo_seed_ns_at_first_edge(raw_edge_dwt);
  }
  return m->ns_at_edge;
}

static uint64_t alpha_ocxo_project_measured_ns_to_dwt(time_clock_id_t clock,
                                                      uint32_t target_dwt) {
  const alpha_measured_ns_clock_t* m = alpha_measured_ns_store(clock);
  if (!m) {
    clocks_watchdog_anomaly("alpha_ocxo_project_store_missing",
                            (uint32_t)((uint8_t)clock), target_dwt, 0, 0);
    return 0;
  }

  if (!m->initialized) return 0;

  const int32_t signed_delta_cycles =
      alpha_dwt32_signed_delta_near(m->dwt_at_edge, target_dwt);
  const uint32_t legacy_unsigned_delta = target_dwt - m->dwt_at_edge;

  if (signed_delta_cycles < 0) {
    alpha_projection_guard_note_measured_backward(clock,
                                                  target_dwt,
                                                  m->dwt_at_edge,
                                                  signed_delta_cycles,
                                                  legacy_unsigned_delta);
  }

  if (alpha_abs_u64_from_i64((int64_t)signed_delta_cycles) >
      (uint64_t)alpha_projection_signed_window_cycles()) {
    alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
    if (g) g->measured_projection_reject_count++;
    clocks_watchdog_anomaly("alpha_ocxo_project_dwt_window",
                            (uint32_t)((uint8_t)clock),
                            target_dwt,
                            m->dwt_at_edge,
                            legacy_unsigned_delta);
    return m->ns_at_edge;
  }

  const int64_t delta_ns =
      alpha_dwt_cycles_to_gnss_ns_i64((int64_t)signed_delta_cycles);

  if (delta_ns < 0) {
    const uint64_t back_ns = (uint64_t)(-delta_ns);
    if (back_ns > m->ns_at_edge) {
      alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
      if (g) g->measured_projection_reject_count++;
      return 0;
    }
    return m->ns_at_edge - back_ns;
  }

  return m->ns_at_edge + (uint64_t)delta_ns;
}

// Project the OCXO measured ledger to target_dwt using the newest pending
// OCXO edge when the current PPS bridge anchor has already bracketed it.
// This is display/sample-side only: it does not consume the pending edge,
// does not advance the measured ledger, and does not affect the OCXO callback
// path.  The callback still resolves the pending edge as the authoritative
// per-edge measurement when it runs.
static uint64_t alpha_ocxo_project_measured_ns_to_dwt_live(time_clock_id_t clock,
                                                           uint32_t target_dwt) {
  const alpha_measured_ns_clock_t* m = alpha_measured_ns_store(clock);
  if (!m || !m->pending_valid) {
    return alpha_ocxo_project_measured_ns_to_dwt(clock, target_dwt);
  }

  uint64_t pending_gnss_ns = 0;
  int32_t phi_cycles = 0;
  uint32_t span_cycles = 0;
  if (!alpha_bridge_interpolate_gnss_ns(m->pending_edge_dwt,
                                        &pending_gnss_ns,
                                        &phi_cycles,
                                        &span_cycles)) {
    return alpha_ocxo_project_measured_ns_to_dwt(clock, target_dwt);
  }

  const int32_t signed_delta_cycles =
      alpha_dwt32_signed_delta_near(m->pending_edge_dwt, target_dwt);
  const uint32_t legacy_unsigned_delta = target_dwt - m->pending_edge_dwt;

  if (signed_delta_cycles < 0) {
    alpha_projection_guard_note_measured_backward(clock,
                                                  target_dwt,
                                                  m->pending_edge_dwt,
                                                  signed_delta_cycles,
                                                  legacy_unsigned_delta);
  }

  if (alpha_abs_u64_from_i64((int64_t)signed_delta_cycles) >
      (uint64_t)alpha_projection_signed_window_cycles()) {
    alpha_ocxo_pps_projection_guard_t* g = alpha_ocxo_pps_projection_guard(clock);
    if (g) g->measured_projection_reject_count++;
    clocks_watchdog_anomaly("alpha_ocxo_live_project_dwt_window",
                            (uint32_t)((uint8_t)clock),
                            target_dwt,
                            m->pending_edge_dwt,
                            legacy_unsigned_delta);
    return pending_gnss_ns;
  }

  const int64_t delta_ns =
      alpha_dwt_cycles_to_gnss_ns_i64((int64_t)signed_delta_cycles);

  if (delta_ns < 0) {
    const uint64_t back_ns = (uint64_t)(-delta_ns);
    return (back_ns > pending_gnss_ns) ? 0ULL : (pending_gnss_ns - back_ns);
  }

  return pending_gnss_ns + (uint64_t)delta_ns;
}

static void alpha_forensics_reset_store(alpha_lane_forensics_store_t& s) {
  s.seq++;
  clocks_alpha_dmb();

  s.valid = false;
  s.update_count = 0;
  s.last_event_dwt = 0;
  s.last_event_counter32 = 0;
  s.zero_offset_valid = false;
  s.zero_offset_counter32 = 0;
  s.counter32_delta_since_zero_offset = 0;
  s.counter32_delta_since_previous_event = 0;
  s.logical_ticks64_since_zero = 0;
  s.nominal_ns64_since_zero = 0;
  s.epoch_counter32 = 0;
  s.counter32_delta_since_epoch = 0;
  s.nominal_ns_from_counter32_epoch = 0;
  s.event_gnss_ns = 0;
  s.previous_event_gnss_ns = 0;
  s.sample_gnss_ns_at_event_available = false;
  s.previous_sample_gnss_ns_at_event_available = false;
  s.sample_gnss_ns_at_event = 0;
  s.previous_sample_gnss_ns_at_event = 0;
  s.phase_offset_ns = 0;
  s.physical_measured_ns_at_edge = 0;
  s.visible_ns_at_edge = 0;
  s.visible_origin_phase_valid = false;
  s.visible_origin_phase_offset_ns = 0;
  s.counter_nominal_ns_between_edges = 0;
  s.bridge_gnss_ns_between_edges = 0;
  s.bridge_residual_ns = 0;
  s.bridge_interval_valid = false;
  s.bridge_anchored = false;
  s.bridge_phi_cycles = 0;
  s.bridge_span_cycles = 0;
  s.bridge_resolved_count = 0;
  s.bridge_fallback_count = 0;
  s.ns_between_edges = 0;
  s.dwt_cycles_between_edges = 0;
  s.dwt_synthetic = false;
  s.dwt_repair_candidate = false;
  s.dwt_original_at_event = 0;
  s.dwt_predicted_at_event = 0;
  s.dwt_used_at_event = 0;
  s.dwt_isr_entry_raw = 0;
  s.dwt_event_from_isr_entry_raw = 0;
  s.dwt_isr_entry_to_event_correction_cycles = 0;
  s.dwt_published_minus_event_cycles = 0;
  s.dwt_used_minus_event_cycles = 0;
  s.dwt_synthetic_error_cycles = 0;
  s.dwt_synthetic_threshold_cycles = 0;
  s.dwt_interval_gate_valid = false;
  s.dwt_interval_sample_accepted = false;
  s.dwt_interval_sample_rejected = false;
  s.dwt_interval_ema_updated = false;
  s.dwt_interval_observed_cycles = 0;
  s.dwt_interval_prediction_cycles = 0;
  s.dwt_interval_effective_cycles = 0;
  s.dwt_interval_residual_cycles = 0;
  s.dwt_interval_gate_threshold_cycles = 0;
  s.dwt_interval_accept_count = 0;
  s.dwt_interval_reject_count = 0;
  s.dwt_interval_resync_applied = false;
  s.dwt_interval_resync_count = 0;
  s.dwt_interval_reject_streak = 0;
  s.dwt_interval_adjacency_gate_valid = false;
  s.dwt_interval_adjacency_ok = false;
  s.dwt_interval_adjacency_rejected = false;
  s.dwt_interval_counter_delta_ticks = 0;
  s.dwt_interval_expected_counter_delta_ticks = 0;
  s.dwt_interval_adjacency_reject_count = 0;
  s.dwt_yardstick_valid = false;
  s.dwt_yardstick_stale = false;
  s.dwt_yardstick_seeded = false;
  s.dwt_yardstick_excursion = false;
  s.dwt_yardstick_pps_sequence = 0;
  s.dwt_yardstick_pps_seq_delta = 0;
  s.dwt_yardstick_g_now_cycles = 0;
  s.dwt_yardstick_g_prev_cycles = 0;
  s.dwt_yardstick_inferred_interval_cycles = 0;
  s.dwt_yardstick_observed_interval_cycles = 0;
  s.dwt_yardstick_inferred_minus_observed_cycles = 0;
  s.dwt_yardstick_inferred_endpoint_dwt = 0;
  s.dwt_yardstick_inferred_endpoint_frac_q16 = 0;
  s.dwt_yardstick_endpoint_minus_observed_cycles = 0;
  s.dwt_yardstick_gate_threshold_cycles = 0;
  s.dwt_yardstick_gate_agree_count = 0;
  s.dwt_yardstick_gate_excursion_count = 0;
  s.dwt_yardstick_authority = false;
  s.dwt_ema_dwt_at_event = 0;
  s.dwt_yardstick_auth_endpoint_dwt = 0;
  s.dwt_yardstick_auth_endpoint_frac_q16 = 0;
  s.dwt_yardstick_auth_error_cycles = 0;
  s.dwt_yardstick_auth_anchor_applied = false;
  s.slipledger_active = false;
  s.slipledger_event_corrected = false;
  s.slipledger_event_violation = false;
  s.slipledger_ticks = 0;
  s.slipledger_event_ticks = 0;
  s.slipledger_generation = 0;
  s.slipledger_observe_count = 0;
  s.slipledger_ok_count = 0;
  s.slipledger_violation_count = 0;
  s.slipledger_correction_count = 0;
  s.slipledger_noop_violation_count = 0;
  s.slipledger_early_count = 0;
  s.slipledger_late_count = 0;
  s.slipledger_one_second_observe_count = 0;
  s.slipledger_one_second_ok_count = 0;
  s.slipledger_one_second_violation_count = 0;
  s.slipledger_one_second_correction_count = 0;
  s.slipledger_last_expected_dwt = 0;
  s.slipledger_last_observed_dwt = 0;
  s.slipledger_last_authored_dwt = 0;
  s.slipledger_last_expected_interval_cycles = 0;
  s.slipledger_last_observed_interval_cycles = 0;
  s.slipledger_last_dwt_error_cycles = 0;
  s.slipledger_last_target_counter32 = 0;
  s.slipledger_last_hardware_target_low16 = 0;
  s.slipledger_last_ambient_low16 = 0;
  s.slipledger_last_tick_mod = 0;
  s.slipledger_reason_code = 0;
  s.slipledger_last_correction_reason_code = 0;
  s.slipledger_last_correction_ticks = 0;
  s.slipledger_last_correction_dwt_error_cycles = 0;
  s.second_residual_ns = 0;
  s.window_error_ns = 0;
  s.window_checks = 0;
  s.window_mismatches = 0;
  s.diag_anchor_sequence_used = 0;
  s.diag_anchor_age_slots = 0;
  s.diag_anchor_selection_kind = 0;
  s.diag_anchor_dwt_at_edge = 0;
  s.diag_anchor_gnss_ns_at_edge = -1;
  s.diag_anchor_cps = 0;
  s.diag_anchor_ns_delta = 0;
  s.diag_anchor_failure_mask = 0;
  s.diag_service_class = 0;
  s.diag_service_offset_signed_ticks = 0;
  s.diag_service_offset_abs_ticks = 0;
  s.diag_interpreted_late_ticks = 0;
  s.diag_early_ticks = 0;
  s.diag_target_delta_mod65536_ticks = 0;
  s.diag_arm_remaining_ticks = 0;
  s.diag_arm_to_isr_ticks = 0;
  s.diag_arm_to_isr_dwt_cycles = 0;
  s.diag_perishable_fact_sequence = 0;
  s.diag_service_correction_cycles = 0;
  s.diag_service_corrected_dwt_at_event = 0;
  s.diag_fact_ring_overflow_count = 0;
  s.diag_counter_delta_violation_count = 0;
  s.diag_last_bad_counter_delta = 0;
  s.diag_last_counter_delta_ticks = 0;
  s.diag_sample_phase_valid = false;
  s.diag_sample_phase_ticks = 0;
  s.diag_sample_phase_ns = 0;
  s.diag_sample_phase_us = 0;
  s.diag_sample_period_ticks = 0;
  s.diag_sample_dwt_at_event = 0;
  s.diag_sample_counter32_at_event = 0;
  s.diag_boundary_dwt_at_event = 0;
  s.diag_boundary_counter32_at_event = 0;
  s.diag_boundary_correction_cycles = 0;
  s.spinidle_shadow_valid = false;
  s.spinidle_shadow_dwt = 0;
  s.spinidle_shadow_to_isr_entry_cycles = 0;
  s.spinidle_shadow_valid_threshold_cycles = 0;
  s.regression_valid = false;
  s.regression_sequence = 0;
  s.regression_sample_count = 0;
  s.regression_observed_dwt_at_event = 0;
  s.regression_inferred_dwt_at_event = 0;
  s.regression_inferred_minus_observed_cycles = 0;
  s.regression_target_counter32_at_event = 0;
  s.regression_target_hardware16_at_event = 0;
  s.regression_observed_hardware16_at_event = 0;
  s.regression_slope_q16_cycles_per_sample = 0;
  s.regression_slope_delta_q16_cycles_per_sample = 0;
  s.regression_fit_error_mean_q16_cycles = 0;
  s.regression_fit_error_stddev_q16_cycles = 0;
  s.regression_fit_error_min_cycles = 0;
  s.regression_fit_error_max_cycles = 0;
  s.regression_fit_error_gt_plus4_count = 0;
  s.regression_fit_error_lt_minus4_count = 0;
  s.regression_fit_error_abs_gt4_count = 0;

  clocks_alpha_dmb();
  s.seq++;
}

static void alpha_forensics_reset_all(void) {
  alpha_forensics_reset_store(g_vclock_forensics);
  alpha_event_flow_note_forensics_reset(time_clock_id_t::VCLOCK);
  alpha_forensics_reset_store(g_ocxo1_forensics);
  alpha_event_flow_note_forensics_reset(time_clock_id_t::OCXO1);
  alpha_forensics_reset_store(g_ocxo2_forensics);
  alpha_event_flow_note_forensics_reset(time_clock_id_t::OCXO2);
}

static void alpha_forensics_publish(time_clock_id_t clock_id,
                                    const clock_state_t& clock,
                                    const clock_measurement_t& meas,
                                    const interrupt_event_t& event,
                                    const interrupt_capture_diag_t* diag,
                                    bool zero_offset_valid,
                                    uint32_t zero_offset_counter32,
                                    uint32_t counter32_delta_since_zero_offset,
                                    uint32_t counter32_delta_since_previous_event,
                                    uint64_t logical_ticks64_since_zero,
                                    uint64_t counter_ns_now,
                                    uint64_t ns_now,
                                    uint64_t physical_measured_ns_at_edge,
                                    uint64_t visible_ns_at_edge,
                                    bool visible_origin_phase_valid,
                                    uint32_t visible_origin_phase_offset_ns) {
  alpha_lane_forensics_store_t* s = alpha_forensics_store(clock_id);
  if (!s) {
    alpha_event_flow_note_forensics_missing_store(clock_id);
    return;
  }

  const bool had_prior_counter_event = s->valid;
  const uint64_t previous_counter_ns = s->nominal_ns_from_counter32_epoch;
  const uint64_t counter_nominal_ns_between_edges =
      (had_prior_counter_event && counter_ns_now >= previous_counter_ns)
          ? (counter_ns_now - previous_counter_ns)
          : 0ULL;

  const uint64_t previous_event_gnss_ns = s->event_gnss_ns;
  const bool sample_gnss_ns_at_event_available =
      (event.gnss_ns_at_event != 0);
  const uint64_t previous_sample_gnss_ns_at_event =
      s->sample_gnss_ns_at_event;
  const bool previous_sample_gnss_ns_at_event_available =
      s->sample_gnss_ns_at_event_available;
  const uint64_t sample_gnss_ns_at_event = sample_gnss_ns_at_event_available
      ? event.gnss_ns_at_event
      : 0ULL;
  const uint64_t event_gnss_ns = sample_gnss_ns_at_event_available
      ? event.gnss_ns_at_event
      : ns_now;

  // For OCXO lanes Alpha's authoritative measured interval remains the
  // existing Alpha-authored measurement surface in this step.  The
  // process_interrupt GNSS timestamp is stored as a passive witness only.
  const bool bridge_interval_valid = (meas.gnss_ns_between_edges != 0);
  const uint64_t bridge_gnss_ns_between_edges = bridge_interval_valid
      ? meas.gnss_ns_between_edges
      : 0ULL;
  // Signed as interval-minus-ideal: negative means the lane fired early / fast.
  const int64_t bridge_residual_ns = bridge_interval_valid
      ? -meas.second_residual_ns
      : 0;

  s->seq++;
  clocks_alpha_dmb();

  s->valid = true;
  s->update_count++;
  // Alpha-facing event DWT.  For OCXO lanes this may be the promoted
  // service-corrected endpoint; the original ISR-serviced endpoint remains
  // reconstructable from diag_service_corrected_dwt_at_event plus
  // diag_service_correction_cycles.
  s->last_event_dwt = event.dwt_at_event;
  s->last_event_counter32 = event.counter32_at_event;
  s->zero_offset_valid = zero_offset_valid;
  s->zero_offset_counter32 = zero_offset_counter32;
  s->counter32_delta_since_zero_offset = counter32_delta_since_zero_offset;
  s->counter32_delta_since_previous_event = counter32_delta_since_previous_event;
  s->logical_ticks64_since_zero = logical_ticks64_since_zero;
  s->nominal_ns64_since_zero = ns_now;
  s->epoch_counter32 = zero_offset_counter32;
  s->counter32_delta_since_epoch = counter32_delta_since_zero_offset;
  s->nominal_ns_from_counter32_epoch = counter_ns_now;
  s->event_gnss_ns = event_gnss_ns;
  s->previous_event_gnss_ns = previous_event_gnss_ns;
  s->sample_gnss_ns_at_event_available = sample_gnss_ns_at_event_available;
  s->previous_sample_gnss_ns_at_event_available =
      previous_sample_gnss_ns_at_event_available;
  s->sample_gnss_ns_at_event = sample_gnss_ns_at_event;
  s->previous_sample_gnss_ns_at_event = previous_sample_gnss_ns_at_event;
  s->phase_offset_ns = clock.phase_offset_ns;
  s->physical_measured_ns_at_edge = physical_measured_ns_at_edge;
  s->visible_ns_at_edge = visible_ns_at_edge;
  s->visible_origin_phase_valid = visible_origin_phase_valid;
  s->visible_origin_phase_offset_ns = visible_origin_phase_offset_ns;
  s->counter_nominal_ns_between_edges = counter_nominal_ns_between_edges;
  s->bridge_gnss_ns_between_edges = bridge_gnss_ns_between_edges;
  s->bridge_residual_ns = bridge_residual_ns;
  s->bridge_interval_valid = bridge_interval_valid;
  s->bridge_anchored = meas.bridge_anchored;
  s->bridge_phi_cycles = meas.bridge_phi_cycles;
  s->bridge_span_cycles = meas.bridge_span_cycles;
  s->bridge_resolved_count = meas.bridge_resolved_count;
  s->bridge_fallback_count = meas.bridge_fallback_count;
  s->ns_between_edges = meas.gnss_ns_between_edges;
  s->dwt_cycles_between_edges = meas.dwt_cycles_between_edges;
  s->second_residual_ns = meas.second_residual_ns;
  s->window_error_ns = clock.window_error_ns;
  s->window_checks = clock.window_checks;
  s->window_mismatches = clock.window_mismatches;

  if (diag) {
    s->dwt_synthetic = diag->dwt_synthetic;
    s->dwt_repair_candidate = diag->dwt_repair_candidate;
    s->dwt_original_at_event = diag->dwt_original_at_event;
    s->dwt_predicted_at_event = diag->dwt_predicted_at_event;
    s->dwt_used_at_event = diag->dwt_used_at_event;
    s->dwt_isr_entry_raw = diag->dwt_isr_entry_raw;
    s->dwt_event_from_isr_entry_raw = diag->dwt_event_from_isr_entry_raw;
    s->dwt_isr_entry_to_event_correction_cycles =
        diag->dwt_isr_entry_to_event_correction_cycles;
    s->dwt_published_minus_event_cycles =
        diag->dwt_published_minus_event_cycles;
    s->dwt_used_minus_event_cycles = diag->dwt_used_minus_event_cycles;
    s->dwt_synthetic_error_cycles = diag->dwt_synthetic_error_cycles;
    s->dwt_synthetic_threshold_cycles = diag->dwt_synthetic_threshold_cycles;
    s->dwt_interval_gate_valid = diag->dwt_interval_gate_valid;
    s->dwt_interval_sample_accepted = diag->dwt_interval_sample_accepted;
    s->dwt_interval_sample_rejected = diag->dwt_interval_sample_rejected;
    s->dwt_interval_ema_updated = diag->dwt_interval_ema_updated;
    s->dwt_interval_observed_cycles = diag->dwt_interval_observed_cycles;
    s->dwt_interval_prediction_cycles = diag->dwt_interval_prediction_cycles;
    s->dwt_interval_effective_cycles = diag->dwt_interval_effective_cycles;
    s->dwt_interval_residual_cycles = diag->dwt_interval_residual_cycles;
    s->dwt_interval_gate_threshold_cycles = diag->dwt_interval_gate_threshold_cycles;
    s->dwt_interval_accept_count = diag->dwt_interval_accept_count;
    s->dwt_interval_reject_count = diag->dwt_interval_reject_count;
    s->dwt_interval_resync_applied = diag->dwt_interval_resync_applied;
    s->dwt_interval_resync_count = diag->dwt_interval_resync_count;
    s->dwt_interval_reject_streak = diag->dwt_interval_reject_streak;
    s->dwt_interval_adjacency_gate_valid =
        diag->dwt_interval_adjacency_gate_valid;
    s->dwt_interval_adjacency_ok = diag->dwt_interval_adjacency_ok;
    s->dwt_interval_adjacency_rejected =
        diag->dwt_interval_adjacency_rejected;
    s->dwt_interval_counter_delta_ticks =
        diag->dwt_interval_counter_delta_ticks;
    s->dwt_interval_expected_counter_delta_ticks =
        diag->dwt_interval_expected_counter_delta_ticks;
    s->dwt_interval_adjacency_reject_count =
        diag->dwt_interval_adjacency_reject_count;
    s->dwt_yardstick_valid = diag->dwt_yardstick_valid;
    s->dwt_yardstick_stale = diag->dwt_yardstick_stale;
    s->dwt_yardstick_seeded = diag->dwt_yardstick_seeded;
    s->dwt_yardstick_excursion = diag->dwt_yardstick_excursion;
    s->dwt_yardstick_pps_sequence = diag->dwt_yardstick_pps_sequence;
    s->dwt_yardstick_pps_seq_delta = diag->dwt_yardstick_pps_seq_delta;
    s->dwt_yardstick_g_now_cycles = diag->dwt_yardstick_g_now_cycles;
    s->dwt_yardstick_g_prev_cycles = diag->dwt_yardstick_g_prev_cycles;
    s->dwt_yardstick_inferred_interval_cycles = diag->dwt_yardstick_inferred_interval_cycles;
    s->dwt_yardstick_observed_interval_cycles = diag->dwt_yardstick_observed_interval_cycles;
    s->dwt_yardstick_inferred_minus_observed_cycles = diag->dwt_yardstick_inferred_minus_observed_cycles;
    s->dwt_yardstick_inferred_endpoint_dwt = diag->dwt_yardstick_inferred_endpoint_dwt;
    s->dwt_yardstick_inferred_endpoint_frac_q16 = diag->dwt_yardstick_inferred_endpoint_frac_q16;
    s->dwt_yardstick_endpoint_minus_observed_cycles = diag->dwt_yardstick_endpoint_minus_observed_cycles;
    s->dwt_yardstick_gate_threshold_cycles = diag->dwt_yardstick_gate_threshold_cycles;
    s->dwt_yardstick_gate_agree_count = diag->dwt_yardstick_gate_agree_count;
    s->dwt_yardstick_gate_excursion_count = diag->dwt_yardstick_gate_excursion_count;
    s->dwt_yardstick_authority = diag->dwt_yardstick_authority;
    s->dwt_ema_dwt_at_event = diag->dwt_ema_dwt_at_event;
    s->dwt_yardstick_auth_endpoint_dwt = diag->dwt_yardstick_auth_endpoint_dwt;
    s->dwt_yardstick_auth_endpoint_frac_q16 = diag->dwt_yardstick_auth_endpoint_frac_q16;
    s->dwt_yardstick_auth_error_cycles = diag->dwt_yardstick_auth_error_cycles;
    s->dwt_yardstick_auth_anchor_applied = diag->dwt_yardstick_auth_anchor_applied;

    s->slipledger_active = diag->slipledger_active;
    s->slipledger_event_corrected = diag->slipledger_event_corrected;
    s->slipledger_event_violation = diag->slipledger_event_violation;
    s->slipledger_ticks = diag->slipledger_ticks;
    s->slipledger_event_ticks = diag->slipledger_event_ticks;
    s->slipledger_generation = diag->slipledger_generation;
    s->slipledger_observe_count = diag->slipledger_observe_count;
    s->slipledger_ok_count = diag->slipledger_ok_count;
    s->slipledger_violation_count = diag->slipledger_violation_count;
    s->slipledger_correction_count = diag->slipledger_correction_count;
    s->slipledger_noop_violation_count = diag->slipledger_noop_violation_count;
    s->slipledger_early_count = diag->slipledger_early_count;
    s->slipledger_late_count = diag->slipledger_late_count;
    s->slipledger_one_second_observe_count = diag->slipledger_one_second_observe_count;
    s->slipledger_one_second_ok_count = diag->slipledger_one_second_ok_count;
    s->slipledger_one_second_violation_count = diag->slipledger_one_second_violation_count;
    s->slipledger_one_second_correction_count = diag->slipledger_one_second_correction_count;
    s->slipledger_last_expected_dwt = diag->slipledger_last_expected_dwt;
    s->slipledger_last_observed_dwt = diag->slipledger_last_observed_dwt;
    s->slipledger_last_authored_dwt = diag->slipledger_last_authored_dwt;
    s->slipledger_last_expected_interval_cycles = diag->slipledger_last_expected_interval_cycles;
    s->slipledger_last_observed_interval_cycles = diag->slipledger_last_observed_interval_cycles;
    s->slipledger_last_dwt_error_cycles = diag->slipledger_last_dwt_error_cycles;
    s->slipledger_last_target_counter32 = diag->slipledger_last_target_counter32;
    s->slipledger_last_hardware_target_low16 = diag->slipledger_last_hardware_target_low16;
    s->slipledger_last_ambient_low16 = diag->slipledger_last_ambient_low16;
    s->slipledger_last_tick_mod = diag->slipledger_last_tick_mod;
    s->slipledger_reason_code = diag->slipledger_reason_code;
    s->slipledger_last_correction_reason_code = diag->slipledger_last_correction_reason_code;
    s->slipledger_last_correction_ticks = diag->slipledger_last_correction_ticks;
    s->slipledger_last_correction_dwt_error_cycles = diag->slipledger_last_correction_dwt_error_cycles;

    s->diag_anchor_sequence_used = diag->anchor_sequence_used;
    s->diag_anchor_age_slots = diag->anchor_age_slots;
    s->diag_anchor_selection_kind = diag->anchor_selection_kind;
    s->diag_anchor_dwt_at_edge = diag->anchor_dwt_at_edge;
    s->diag_anchor_gnss_ns_at_edge = diag->anchor_gnss_ns_at_edge;
    s->diag_anchor_cps = diag->anchor_cps;
    s->diag_anchor_ns_delta = diag->anchor_ns_delta;
    s->diag_anchor_failure_mask = diag->anchor_failure_mask;

    s->diag_service_class = diag->ocxo_service_class;
    s->diag_service_offset_signed_ticks =
        diag->ocxo_service_offset_signed_ticks;
    s->diag_service_offset_abs_ticks = diag->ocxo_service_offset_abs_ticks;
    s->diag_interpreted_late_ticks = diag->ocxo_interpreted_late_ticks;
    s->diag_early_ticks = diag->ocxo_early_ticks;
    s->diag_target_delta_mod65536_ticks =
        diag->ocxo_target_delta_mod65536_ticks;
    s->diag_arm_remaining_ticks = diag->ocxo_arm_remaining_ticks;
    s->diag_arm_to_isr_ticks = diag->ocxo_arm_to_isr_ticks;
    s->diag_arm_to_isr_dwt_cycles = diag->ocxo_arm_to_isr_dwt_cycles;
    s->diag_perishable_fact_sequence = diag->ocxo_perishable_fact_sequence;
    s->diag_service_correction_cycles = diag->ocxo_service_correction_cycles;
    s->diag_service_corrected_dwt_at_event =
        diag->ocxo_service_corrected_dwt_at_event;
    s->diag_fact_ring_overflow_count = diag->ocxo_fact_ring_overflow_count;
    s->diag_counter_delta_violation_count =
        diag->ocxo_counter_delta_violation_count;
    s->diag_last_bad_counter_delta = diag->ocxo_last_bad_counter_delta;
    s->diag_last_counter_delta_ticks = diag->ocxo_last_counter_delta_ticks;
    s->diag_sample_phase_valid = diag->ocxo_sample_phase_valid;
    s->diag_sample_phase_ticks = diag->ocxo_sample_phase_ticks;
    s->diag_sample_phase_ns = diag->ocxo_sample_phase_ns;
    s->diag_sample_phase_us = diag->ocxo_sample_phase_us;
    s->diag_sample_period_ticks = diag->ocxo_sample_period_ticks;
    s->diag_sample_dwt_at_event = diag->ocxo_sample_dwt_at_event;
    s->diag_sample_counter32_at_event = diag->ocxo_sample_counter32_at_event;
    s->diag_boundary_dwt_at_event = diag->ocxo_boundary_dwt_at_event;
    s->diag_boundary_counter32_at_event = diag->ocxo_boundary_counter32_at_event;
    s->diag_boundary_correction_cycles = diag->ocxo_boundary_correction_cycles;

    s->spinidle_shadow_valid = diag->spinidle_shadow_valid;
    s->spinidle_shadow_dwt = diag->spinidle_shadow_dwt;
    s->spinidle_shadow_to_isr_entry_cycles =
        diag->spinidle_shadow_to_isr_entry_cycles;
    s->spinidle_shadow_valid_threshold_cycles =
        diag->spinidle_shadow_valid_threshold_cycles;

    s->regression_valid = diag->regression_valid;
    s->regression_sequence = diag->regression_sequence;
    s->regression_sample_count = diag->regression_sample_count;
    s->regression_observed_dwt_at_event =
        diag->regression_observed_dwt_at_event;
    s->regression_inferred_dwt_at_event =
        diag->regression_inferred_dwt_at_event;
    s->regression_inferred_minus_observed_cycles =
        diag->regression_inferred_minus_observed_cycles;
    s->regression_target_counter32_at_event =
        diag->regression_target_counter32_at_event;
    s->regression_target_hardware16_at_event =
        diag->regression_target_hardware16_at_event;
    s->regression_observed_hardware16_at_event =
        diag->regression_observed_hardware16_at_event;
    s->regression_slope_q16_cycles_per_sample =
        diag->regression_slope_q16_cycles_per_sample;
    s->regression_slope_delta_q16_cycles_per_sample =
        diag->regression_slope_delta_q16_cycles_per_sample;
    s->regression_fit_error_mean_q16_cycles =
        diag->regression_fit_error_mean_q16_cycles;
    s->regression_fit_error_stddev_q16_cycles =
        diag->regression_fit_error_stddev_q16_cycles;
    s->regression_fit_error_min_cycles =
        diag->regression_fit_error_min_cycles;
    s->regression_fit_error_max_cycles =
        diag->regression_fit_error_max_cycles;
    s->regression_fit_error_gt_plus4_count =
        diag->regression_fit_error_gt_plus4_count;
    s->regression_fit_error_lt_minus4_count =
        diag->regression_fit_error_lt_minus4_count;
    s->regression_fit_error_abs_gt4_count =
        diag->regression_fit_error_abs_gt4_count;
  } else {
    s->dwt_synthetic = false;
    s->dwt_repair_candidate = false;
    s->dwt_original_at_event = 0;
    s->dwt_predicted_at_event = 0;
    s->dwt_used_at_event = 0;
    s->dwt_isr_entry_raw = 0;
    s->dwt_event_from_isr_entry_raw = 0;
    s->dwt_isr_entry_to_event_correction_cycles = 0;
    s->dwt_published_minus_event_cycles = 0;
    s->dwt_used_minus_event_cycles = 0;
    s->dwt_synthetic_error_cycles = 0;
    s->dwt_synthetic_threshold_cycles = 0;
    s->dwt_interval_gate_valid = false;
    s->dwt_interval_sample_accepted = false;
    s->dwt_interval_sample_rejected = false;
    s->dwt_interval_ema_updated = false;
    s->dwt_interval_observed_cycles = 0;
    s->dwt_interval_prediction_cycles = 0;
    s->dwt_interval_effective_cycles = 0;
    s->dwt_interval_residual_cycles = 0;
    s->dwt_interval_gate_threshold_cycles = 0;
    s->dwt_interval_accept_count = 0;
    s->dwt_interval_reject_count = 0;
    s->dwt_interval_resync_applied = false;
    s->dwt_interval_resync_count = 0;
    s->dwt_interval_reject_streak = 0;
    s->dwt_interval_adjacency_gate_valid = false;
    s->dwt_interval_adjacency_ok = false;
    s->dwt_interval_adjacency_rejected = false;
    s->dwt_interval_counter_delta_ticks = 0;
    s->dwt_interval_expected_counter_delta_ticks = 0;
    s->dwt_interval_adjacency_reject_count = 0;
    s->dwt_yardstick_valid = false;
    s->dwt_yardstick_stale = false;
    s->dwt_yardstick_seeded = false;
    s->dwt_yardstick_excursion = false;
    s->dwt_yardstick_pps_sequence = 0;
    s->dwt_yardstick_pps_seq_delta = 0;
    s->dwt_yardstick_g_now_cycles = 0;
    s->dwt_yardstick_g_prev_cycles = 0;
    s->dwt_yardstick_inferred_interval_cycles = 0;
    s->dwt_yardstick_observed_interval_cycles = 0;
    s->dwt_yardstick_inferred_minus_observed_cycles = 0;
    s->dwt_yardstick_inferred_endpoint_dwt = 0;
    s->dwt_yardstick_inferred_endpoint_frac_q16 = 0;
    s->dwt_yardstick_endpoint_minus_observed_cycles = 0;
    s->dwt_yardstick_gate_threshold_cycles = 0;
    s->dwt_yardstick_gate_agree_count = 0;
    s->dwt_yardstick_gate_excursion_count = 0;
    s->dwt_yardstick_authority = false;
    s->dwt_ema_dwt_at_event = 0;
    s->dwt_yardstick_auth_endpoint_dwt = 0;
    s->dwt_yardstick_auth_endpoint_frac_q16 = 0;
    s->dwt_yardstick_auth_error_cycles = 0;
    s->dwt_yardstick_auth_anchor_applied = false;
    s->slipledger_active = false;
    s->slipledger_event_corrected = false;
    s->slipledger_event_violation = false;
    s->slipledger_ticks = 0;
    s->slipledger_event_ticks = 0;
    s->slipledger_generation = 0;
    s->slipledger_observe_count = 0;
    s->slipledger_ok_count = 0;
    s->slipledger_violation_count = 0;
    s->slipledger_correction_count = 0;
    s->slipledger_noop_violation_count = 0;
    s->slipledger_early_count = 0;
    s->slipledger_late_count = 0;
    s->slipledger_one_second_observe_count = 0;
    s->slipledger_one_second_ok_count = 0;
    s->slipledger_one_second_violation_count = 0;
    s->slipledger_one_second_correction_count = 0;
    s->slipledger_last_expected_dwt = 0;
    s->slipledger_last_observed_dwt = 0;
    s->slipledger_last_authored_dwt = 0;
    s->slipledger_last_expected_interval_cycles = 0;
    s->slipledger_last_observed_interval_cycles = 0;
    s->slipledger_last_dwt_error_cycles = 0;
    s->slipledger_last_target_counter32 = 0;
    s->slipledger_last_hardware_target_low16 = 0;
    s->slipledger_last_ambient_low16 = 0;
    s->slipledger_last_tick_mod = 0;
    s->slipledger_reason_code = 0;
    s->slipledger_last_correction_reason_code = 0;
    s->slipledger_last_correction_ticks = 0;
    s->slipledger_last_correction_dwt_error_cycles = 0;
    s->diag_anchor_sequence_used = 0;
    s->diag_anchor_age_slots = 0;
    s->diag_anchor_selection_kind = 0;
    s->diag_anchor_dwt_at_edge = 0;
    s->diag_anchor_gnss_ns_at_edge = -1;
    s->diag_anchor_cps = 0;
    s->diag_anchor_ns_delta = 0;
    s->diag_anchor_failure_mask = 0;
    s->diag_service_class = 0;
    s->diag_service_offset_signed_ticks = 0;
    s->diag_service_offset_abs_ticks = 0;
    s->diag_interpreted_late_ticks = 0;
    s->diag_early_ticks = 0;
    s->diag_target_delta_mod65536_ticks = 0;
    s->diag_arm_remaining_ticks = 0;
    s->diag_arm_to_isr_ticks = 0;
    s->diag_arm_to_isr_dwt_cycles = 0;
    s->diag_perishable_fact_sequence = 0;
    s->diag_service_correction_cycles = 0;
    s->diag_service_corrected_dwt_at_event = 0;
    s->diag_fact_ring_overflow_count = 0;
    s->diag_counter_delta_violation_count = 0;
    s->diag_last_bad_counter_delta = 0;
    s->diag_last_counter_delta_ticks = 0;
    s->diag_sample_phase_valid = false;
    s->diag_sample_phase_ticks = 0;
    s->diag_sample_phase_ns = 0;
    s->diag_sample_phase_us = 0;
    s->diag_sample_period_ticks = 0;
    s->diag_sample_dwt_at_event = 0;
    s->diag_sample_counter32_at_event = 0;
    s->diag_boundary_dwt_at_event = 0;
    s->diag_boundary_counter32_at_event = 0;
    s->diag_boundary_correction_cycles = 0;
    s->spinidle_shadow_valid = false;
    s->spinidle_shadow_dwt = 0;
    s->spinidle_shadow_to_isr_entry_cycles = 0;
    s->spinidle_shadow_valid_threshold_cycles = 0;
    s->regression_valid = false;
    s->regression_sequence = 0;
    s->regression_sample_count = 0;
    s->regression_observed_dwt_at_event = 0;
    s->regression_inferred_dwt_at_event = 0;
    s->regression_inferred_minus_observed_cycles = 0;
    s->regression_target_counter32_at_event = 0;
    s->regression_target_hardware16_at_event = 0;
    s->regression_observed_hardware16_at_event = 0;
    s->regression_slope_q16_cycles_per_sample = 0;
    s->regression_slope_delta_q16_cycles_per_sample = 0;
    s->regression_fit_error_mean_q16_cycles = 0;
    s->regression_fit_error_stddev_q16_cycles = 0;
    s->regression_fit_error_min_cycles = 0;
    s->regression_fit_error_max_cycles = 0;
    s->regression_fit_error_gt_plus4_count = 0;
    s->regression_fit_error_lt_minus4_count = 0;
    s->regression_fit_error_abs_gt4_count = 0;
  }

  clocks_alpha_dmb();
  s->seq++;
  alpha_event_flow_note_forensics_publish(clock_id, *s);
}

bool clocks_alpha_lane_forensics(time_clock_id_t clock,
                                 clocks_alpha_lane_forensics_t* out) {
  if (!out) return false;
  alpha_event_flow_note_snapshot_request(clock);
  alpha_lane_forensics_store_t* s = alpha_forensics_store(clock);
  if (!s) {
    alpha_event_flow_note_snapshot_missing_store(clock);
    return false;
  }

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = s->seq;
    clocks_alpha_dmb();

    out->valid = s->valid;
    out->update_count = s->update_count;
    out->last_event_dwt = s->last_event_dwt;
    out->last_event_counter32 = s->last_event_counter32;
    out->zero_offset_valid = s->zero_offset_valid;
    out->zero_offset_counter32 = s->zero_offset_counter32;
    out->counter32_delta_since_zero_offset = s->counter32_delta_since_zero_offset;
    out->counter32_delta_since_previous_event = s->counter32_delta_since_previous_event;
    out->logical_ticks64_since_zero = s->logical_ticks64_since_zero;
    out->nominal_ns64_since_zero = s->nominal_ns64_since_zero;
    out->epoch_counter32 = s->epoch_counter32;
    out->counter32_delta_since_epoch = s->counter32_delta_since_epoch;
    out->nominal_ns_from_counter32_epoch = s->nominal_ns_from_counter32_epoch;
    out->event_gnss_ns = s->event_gnss_ns;
    out->previous_event_gnss_ns = s->previous_event_gnss_ns;
    out->sample_gnss_ns_at_event_available =
        s->sample_gnss_ns_at_event_available;
    out->previous_sample_gnss_ns_at_event_available =
        s->previous_sample_gnss_ns_at_event_available;
    out->sample_gnss_ns_at_event = s->sample_gnss_ns_at_event;
    out->previous_sample_gnss_ns_at_event =
        s->previous_sample_gnss_ns_at_event;
    out->phase_offset_ns = s->phase_offset_ns;
    out->physical_measured_ns_at_edge = s->physical_measured_ns_at_edge;
    out->visible_ns_at_edge = s->visible_ns_at_edge;
    out->visible_origin_phase_valid = s->visible_origin_phase_valid;
    out->visible_origin_phase_offset_ns = s->visible_origin_phase_offset_ns;
    out->counter_nominal_ns_between_edges = s->counter_nominal_ns_between_edges;
    out->bridge_gnss_ns_between_edges = s->bridge_gnss_ns_between_edges;
    out->bridge_residual_ns = s->bridge_residual_ns;
    out->bridge_interval_valid = s->bridge_interval_valid;
    out->bridge_anchored = s->bridge_anchored;
    out->bridge_phi_cycles = s->bridge_phi_cycles;
    out->bridge_span_cycles = s->bridge_span_cycles;
    out->bridge_resolved_count = s->bridge_resolved_count;
    out->bridge_fallback_count = s->bridge_fallback_count;
    out->ns_between_edges = s->ns_between_edges;
    out->dwt_cycles_between_edges = s->dwt_cycles_between_edges;
    out->dwt_synthetic = s->dwt_synthetic;
    out->dwt_repair_candidate = s->dwt_repair_candidate;
    out->dwt_original_at_event = s->dwt_original_at_event;
    out->dwt_predicted_at_event = s->dwt_predicted_at_event;
    out->dwt_used_at_event = s->dwt_used_at_event;
    out->dwt_isr_entry_raw = s->dwt_isr_entry_raw;
    out->dwt_event_from_isr_entry_raw = s->dwt_event_from_isr_entry_raw;
    out->dwt_isr_entry_to_event_correction_cycles =
        s->dwt_isr_entry_to_event_correction_cycles;
    out->dwt_published_minus_event_cycles =
        s->dwt_published_minus_event_cycles;
    out->dwt_used_minus_event_cycles = s->dwt_used_minus_event_cycles;
    out->dwt_synthetic_error_cycles = s->dwt_synthetic_error_cycles;
    out->dwt_synthetic_threshold_cycles = s->dwt_synthetic_threshold_cycles;
    out->dwt_interval_gate_valid = s->dwt_interval_gate_valid;
    out->dwt_interval_sample_accepted = s->dwt_interval_sample_accepted;
    out->dwt_interval_sample_rejected = s->dwt_interval_sample_rejected;
    out->dwt_interval_ema_updated = s->dwt_interval_ema_updated;
    out->dwt_interval_observed_cycles = s->dwt_interval_observed_cycles;
    out->dwt_interval_prediction_cycles = s->dwt_interval_prediction_cycles;
    out->dwt_interval_effective_cycles = s->dwt_interval_effective_cycles;
    out->dwt_interval_residual_cycles = s->dwt_interval_residual_cycles;
    out->dwt_interval_gate_threshold_cycles = s->dwt_interval_gate_threshold_cycles;
    out->dwt_interval_accept_count = s->dwt_interval_accept_count;
    out->dwt_interval_reject_count = s->dwt_interval_reject_count;
    out->dwt_interval_resync_applied = s->dwt_interval_resync_applied;
    out->dwt_interval_resync_count = s->dwt_interval_resync_count;
    out->dwt_interval_reject_streak = s->dwt_interval_reject_streak;
    out->dwt_interval_adjacency_gate_valid =
        s->dwt_interval_adjacency_gate_valid;
    out->dwt_interval_adjacency_ok = s->dwt_interval_adjacency_ok;
    out->dwt_interval_adjacency_rejected =
        s->dwt_interval_adjacency_rejected;
    out->dwt_interval_counter_delta_ticks =
        s->dwt_interval_counter_delta_ticks;
    out->dwt_interval_expected_counter_delta_ticks =
        s->dwt_interval_expected_counter_delta_ticks;
    out->dwt_interval_adjacency_reject_count =
        s->dwt_interval_adjacency_reject_count;
    out->dwt_yardstick_valid = s->dwt_yardstick_valid;
    out->dwt_yardstick_stale = s->dwt_yardstick_stale;
    out->dwt_yardstick_seeded = s->dwt_yardstick_seeded;
    out->dwt_yardstick_excursion = s->dwt_yardstick_excursion;
    out->dwt_yardstick_pps_sequence = s->dwt_yardstick_pps_sequence;
    out->dwt_yardstick_pps_seq_delta = s->dwt_yardstick_pps_seq_delta;
    out->dwt_yardstick_g_now_cycles = s->dwt_yardstick_g_now_cycles;
    out->dwt_yardstick_g_prev_cycles = s->dwt_yardstick_g_prev_cycles;
    out->dwt_yardstick_inferred_interval_cycles = s->dwt_yardstick_inferred_interval_cycles;
    out->dwt_yardstick_observed_interval_cycles = s->dwt_yardstick_observed_interval_cycles;
    out->dwt_yardstick_inferred_minus_observed_cycles = s->dwt_yardstick_inferred_minus_observed_cycles;
    out->dwt_yardstick_inferred_endpoint_dwt = s->dwt_yardstick_inferred_endpoint_dwt;
    out->dwt_yardstick_inferred_endpoint_frac_q16 = s->dwt_yardstick_inferred_endpoint_frac_q16;
    out->dwt_yardstick_endpoint_minus_observed_cycles = s->dwt_yardstick_endpoint_minus_observed_cycles;
    out->dwt_yardstick_gate_threshold_cycles = s->dwt_yardstick_gate_threshold_cycles;
    out->dwt_yardstick_gate_agree_count = s->dwt_yardstick_gate_agree_count;
    out->dwt_yardstick_gate_excursion_count = s->dwt_yardstick_gate_excursion_count;
    out->dwt_yardstick_authority = s->dwt_yardstick_authority;
    out->dwt_ema_dwt_at_event = s->dwt_ema_dwt_at_event;
    out->dwt_yardstick_auth_endpoint_dwt = s->dwt_yardstick_auth_endpoint_dwt;
    out->dwt_yardstick_auth_endpoint_frac_q16 = s->dwt_yardstick_auth_endpoint_frac_q16;
    out->dwt_yardstick_auth_error_cycles = s->dwt_yardstick_auth_error_cycles;
    out->dwt_yardstick_auth_anchor_applied = s->dwt_yardstick_auth_anchor_applied;

    out->slipledger_active = s->slipledger_active;
    out->slipledger_event_corrected = s->slipledger_event_corrected;
    out->slipledger_event_violation = s->slipledger_event_violation;
    out->slipledger_ticks = s->slipledger_ticks;
    out->slipledger_event_ticks = s->slipledger_event_ticks;
    out->slipledger_generation = s->slipledger_generation;
    out->slipledger_observe_count = s->slipledger_observe_count;
    out->slipledger_ok_count = s->slipledger_ok_count;
    out->slipledger_violation_count = s->slipledger_violation_count;
    out->slipledger_correction_count = s->slipledger_correction_count;
    out->slipledger_noop_violation_count = s->slipledger_noop_violation_count;
    out->slipledger_early_count = s->slipledger_early_count;
    out->slipledger_late_count = s->slipledger_late_count;
    out->slipledger_one_second_observe_count = s->slipledger_one_second_observe_count;
    out->slipledger_one_second_ok_count = s->slipledger_one_second_ok_count;
    out->slipledger_one_second_violation_count = s->slipledger_one_second_violation_count;
    out->slipledger_one_second_correction_count = s->slipledger_one_second_correction_count;
    out->slipledger_last_expected_dwt = s->slipledger_last_expected_dwt;
    out->slipledger_last_observed_dwt = s->slipledger_last_observed_dwt;
    out->slipledger_last_authored_dwt = s->slipledger_last_authored_dwt;
    out->slipledger_last_expected_interval_cycles = s->slipledger_last_expected_interval_cycles;
    out->slipledger_last_observed_interval_cycles = s->slipledger_last_observed_interval_cycles;
    out->slipledger_last_dwt_error_cycles = s->slipledger_last_dwt_error_cycles;
    out->slipledger_last_target_counter32 = s->slipledger_last_target_counter32;
    out->slipledger_last_hardware_target_low16 = s->slipledger_last_hardware_target_low16;
    out->slipledger_last_ambient_low16 = s->slipledger_last_ambient_low16;
    out->slipledger_last_tick_mod = s->slipledger_last_tick_mod;
    out->slipledger_reason_code = s->slipledger_reason_code;
    out->slipledger_last_correction_reason_code = s->slipledger_last_correction_reason_code;
    out->slipledger_last_correction_ticks = s->slipledger_last_correction_ticks;
    out->slipledger_last_correction_dwt_error_cycles = s->slipledger_last_correction_dwt_error_cycles;
    out->second_residual_ns = s->second_residual_ns;
    out->window_error_ns = s->window_error_ns;
    out->window_checks = s->window_checks;
    out->window_mismatches = s->window_mismatches;
    out->diag_anchor_sequence_used = s->diag_anchor_sequence_used;
    out->diag_anchor_age_slots = s->diag_anchor_age_slots;
    out->diag_anchor_selection_kind = s->diag_anchor_selection_kind;
    out->diag_anchor_dwt_at_edge = s->diag_anchor_dwt_at_edge;
    out->diag_anchor_gnss_ns_at_edge = s->diag_anchor_gnss_ns_at_edge;
    out->diag_anchor_cps = s->diag_anchor_cps;
    out->diag_anchor_ns_delta = s->diag_anchor_ns_delta;
    out->diag_anchor_failure_mask = s->diag_anchor_failure_mask;
    out->diag_service_class = s->diag_service_class;
    out->diag_service_offset_signed_ticks =
        s->diag_service_offset_signed_ticks;
    out->diag_service_offset_abs_ticks = s->diag_service_offset_abs_ticks;
    out->diag_interpreted_late_ticks = s->diag_interpreted_late_ticks;
    out->diag_early_ticks = s->diag_early_ticks;
    out->diag_target_delta_mod65536_ticks =
        s->diag_target_delta_mod65536_ticks;
    out->diag_arm_remaining_ticks = s->diag_arm_remaining_ticks;
    out->diag_arm_to_isr_ticks = s->diag_arm_to_isr_ticks;
    out->diag_arm_to_isr_dwt_cycles = s->diag_arm_to_isr_dwt_cycles;
    out->diag_perishable_fact_sequence = s->diag_perishable_fact_sequence;
    out->diag_service_correction_cycles = s->diag_service_correction_cycles;
    out->diag_service_corrected_dwt_at_event =
        s->diag_service_corrected_dwt_at_event;
    out->diag_fact_ring_overflow_count = s->diag_fact_ring_overflow_count;
    out->diag_counter_delta_violation_count =
        s->diag_counter_delta_violation_count;
    out->diag_last_bad_counter_delta = s->diag_last_bad_counter_delta;
    out->diag_last_counter_delta_ticks = s->diag_last_counter_delta_ticks;
    out->diag_sample_phase_valid = s->diag_sample_phase_valid;
    out->diag_sample_phase_ticks = s->diag_sample_phase_ticks;
    out->diag_sample_phase_ns = s->diag_sample_phase_ns;
    out->diag_sample_phase_us = s->diag_sample_phase_us;
    out->diag_sample_period_ticks = s->diag_sample_period_ticks;
    out->diag_sample_dwt_at_event = s->diag_sample_dwt_at_event;
    out->diag_sample_counter32_at_event = s->diag_sample_counter32_at_event;
    out->diag_boundary_dwt_at_event = s->diag_boundary_dwt_at_event;
    out->diag_boundary_counter32_at_event = s->diag_boundary_counter32_at_event;
    out->diag_boundary_correction_cycles = s->diag_boundary_correction_cycles;
    out->spinidle_shadow_valid = s->spinidle_shadow_valid;
    out->spinidle_shadow_dwt = s->spinidle_shadow_dwt;
    out->spinidle_shadow_to_isr_entry_cycles =
        s->spinidle_shadow_to_isr_entry_cycles;
    out->spinidle_shadow_valid_threshold_cycles =
        s->spinidle_shadow_valid_threshold_cycles;
    out->regression_valid = s->regression_valid;
    out->regression_sequence = s->regression_sequence;
    out->regression_sample_count = s->regression_sample_count;
    out->regression_observed_dwt_at_event =
        s->regression_observed_dwt_at_event;
    out->regression_inferred_dwt_at_event =
        s->regression_inferred_dwt_at_event;
    out->regression_inferred_minus_observed_cycles =
        s->regression_inferred_minus_observed_cycles;
    out->regression_target_counter32_at_event =
        s->regression_target_counter32_at_event;
    out->regression_target_hardware16_at_event =
        s->regression_target_hardware16_at_event;
    out->regression_observed_hardware16_at_event =
        s->regression_observed_hardware16_at_event;
    out->regression_slope_q16_cycles_per_sample =
        s->regression_slope_q16_cycles_per_sample;
    out->regression_slope_delta_q16_cycles_per_sample =
        s->regression_slope_delta_q16_cycles_per_sample;
    out->regression_fit_error_mean_q16_cycles =
        s->regression_fit_error_mean_q16_cycles;
    out->regression_fit_error_stddev_q16_cycles =
        s->regression_fit_error_stddev_q16_cycles;
    out->regression_fit_error_min_cycles =
        s->regression_fit_error_min_cycles;
    out->regression_fit_error_max_cycles =
        s->regression_fit_error_max_cycles;
    out->regression_fit_error_gt_plus4_count =
        s->regression_fit_error_gt_plus4_count;
    out->regression_fit_error_lt_minus4_count =
        s->regression_fit_error_lt_minus4_count;
    out->regression_fit_error_abs_gt4_count =
        s->regression_fit_error_abs_gt4_count;

    clocks_alpha_dmb();
    const uint32_t seq2 = s->seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      alpha_event_flow_note_snapshot(clock, true, out->valid, out->valid,
                                     out->update_count, seq2);
      return out->valid;
    }
  }

  alpha_event_flow_note_snapshot(clock, false, false, false, 0, 0);
  return false;
}

static void clock_mark_epoch_zero(clock_state_t& clock) {
  clock.ledger_ns_count_at_edge = 0;
  clock.gnss_ns_at_edge = 0;
  clock.ledger_ns_count_at_pps_vclock = 0;
  clock.phase_offset_ns = 0;
  clock.zero_established = true;  // legacy/report mirror; epoch install is a hard invariant
  clock.window_checks = 0;
  clock.window_mismatches = 0;
  clock.window_error_ns = 0;
}

// ============================================================================
// Local epoch install / ZERO
// ============================================================================
//
// ZERO is no longer delegated to process_epoch.  process_interrupt continuously
// authors a zero-offset-ready PPS capture packet in ISR context.  Alpha selects
// the latest valid packet, stores per-lane synthetic counter32 zero offsets,
// installs local canonical nanosecond/cycle ledgers, and notifies
// process_time/process_timepop that the VCLOCK coordinate generation has
// changed.  Raw hardware counters are not written here.
// ============================================================================

static volatile uint32_t g_alpha_epoch_install_count = 0;
static volatile uint32_t g_alpha_epoch_install_failures = 0;
static volatile uint32_t g_alpha_epoch_last_capture_sequence = 0;
static volatile uint32_t g_alpha_epoch_last_capture_window_cycles = 0;
static volatile bool     g_alpha_epoch_last_vclock_capture_valid = false;
static volatile bool     g_alpha_epoch_last_all_lanes_valid = false;
static volatile uint32_t g_alpha_epoch_last_dwt_at_edge = 0;
static volatile uint32_t g_alpha_epoch_last_vclock_counter32 = 0;
static volatile uint32_t g_alpha_epoch_last_ocxo1_counter32 = 0;
static volatile uint32_t g_alpha_epoch_last_ocxo2_counter32 = 0;
static volatile bool     g_alpha_epoch_last_vclock_zero_valid = false;
static volatile bool     g_alpha_epoch_last_ocxo1_zero_valid = false;
static volatile bool     g_alpha_epoch_last_ocxo2_zero_valid = false;
static volatile uint16_t g_alpha_epoch_last_vclock_hardware16_observed = 0;
static volatile uint16_t g_alpha_epoch_last_vclock_hardware16_selected = 0;
static volatile uint16_t g_alpha_epoch_last_ocxo1_hardware16 = 0;
static volatile uint16_t g_alpha_epoch_last_ocxo2_hardware16 = 0;
static volatile uint32_t g_alpha_epoch_sequence = 0;
static char              g_alpha_epoch_last_reason[32] = {0};

static interrupt_smartzero_snapshot_t g_alpha_epoch_last_smartzero = {};
static volatile uint32_t g_alpha_smartzero_begin_count = 0;
static volatile uint32_t g_alpha_smartzero_begin_failures = 0;

// Non-destructive SmartZero begin forensics. These describe the live
// acquisition request, not the installed epoch proof.
static volatile bool     g_alpha_smartzero_last_begin_preserved_epoch = false;
static volatile uint32_t g_alpha_smartzero_last_begin_preserved_epoch_sequence = 0;
static volatile uint32_t g_alpha_smartzero_begin_preserved_epoch_count = 0;
static volatile uint32_t g_alpha_smartzero_begin_cold_count = 0;
static char              g_alpha_smartzero_last_begin_reason[32] = {0};
static volatile bool     g_alpha_smartzero_pending_active = false;
static char              g_alpha_smartzero_pending_reason[32] = {0};

// Atomic SmartZero install transaction forensics.  Beginning SmartZero is a
// live acquisition event; installing SmartZero is the only epoch event.  These
// fields bracket that commit so reports can distinguish a fully committed epoch
// from an in-progress rebase.
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_IDLE             = 0;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_SNAPSHOT         = 1;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_VALIDATE         = 2;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_RESET_STATE      = 3;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_INSTALL_ANCHORS  = 4;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_DWT64            = 5;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_TIME_ANCHORS     = 6;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_TIMEPOP_REAUTHOR = 7;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_COMMIT           = 8;
static constexpr uint32_t SMARTZERO_INSTALL_STAGE_FAIL             = 9;

static constexpr uint32_t SMARTZERO_INSTALL_FAIL_NONE              = 0;
static constexpr uint32_t SMARTZERO_INSTALL_FAIL_REENTRANT         = 1;
static constexpr uint32_t SMARTZERO_INSTALL_FAIL_NO_COMPLETE_PROOF = 2;
static constexpr uint32_t SMARTZERO_INSTALL_FAIL_BAD_LANE_PROOF    = 3;

static volatile uint32_t g_alpha_smartzero_install_attempt_count = 0;
static volatile uint32_t g_alpha_smartzero_install_commit_count = 0;
static volatile uint32_t g_alpha_smartzero_install_failure_count = 0;
static volatile uint32_t g_alpha_smartzero_install_last_stage = SMARTZERO_INSTALL_STAGE_IDLE;
static volatile uint32_t g_alpha_smartzero_install_last_failure_stage = SMARTZERO_INSTALL_STAGE_IDLE;
static volatile uint32_t g_alpha_smartzero_install_last_failure_code = SMARTZERO_INSTALL_FAIL_NONE;
static volatile uint32_t g_alpha_smartzero_install_last_live_sequence = 0;
static volatile uint32_t g_alpha_smartzero_install_last_prior_epoch_sequence = 0;
static volatile uint32_t g_alpha_smartzero_install_last_committed_epoch_sequence = 0;
static volatile uint32_t g_alpha_smartzero_install_last_committed_smartzero_sequence = 0;
static volatile bool     g_alpha_smartzero_install_last_success = false;
static volatile bool     g_alpha_smartzero_install_last_atomic = false;
static char              g_alpha_smartzero_install_last_reason[32] = {0};

static const char* smartzero_install_stage_name(uint32_t stage) {
  switch (stage) {
    case SMARTZERO_INSTALL_STAGE_IDLE:             return "IDLE";
    case SMARTZERO_INSTALL_STAGE_SNAPSHOT:         return "SNAPSHOT";
    case SMARTZERO_INSTALL_STAGE_VALIDATE:         return "VALIDATE";
    case SMARTZERO_INSTALL_STAGE_RESET_STATE:      return "RESET_STATE";
    case SMARTZERO_INSTALL_STAGE_INSTALL_ANCHORS:  return "INSTALL_ANCHORS";
    case SMARTZERO_INSTALL_STAGE_DWT64:            return "DWT64";
    case SMARTZERO_INSTALL_STAGE_TIME_ANCHORS:     return "TIME_ANCHORS";
    case SMARTZERO_INSTALL_STAGE_TIMEPOP_REAUTHOR: return "TIMEPOP_REAUTHOR";
    case SMARTZERO_INSTALL_STAGE_COMMIT:           return "COMMIT";
    case SMARTZERO_INSTALL_STAGE_FAIL:             return "FAIL";
    default:                                       return "UNKNOWN";
  }
}

bool clocks_alpha_epoch_last_smartzero(interrupt_smartzero_snapshot_t* out) {
  if (!out) return false;
  *out = g_alpha_epoch_last_smartzero;
  return out->complete;
}

bool clocks_alpha_installed_smartzero_valid(void) {
  return g_alpha_epoch_last_smartzero.complete;
}

uint32_t clocks_alpha_installed_smartzero_sequence(void) {
  return g_alpha_epoch_last_smartzero.sequence;
}


bool clocks_alpha_installed_smartzero_backing_epoch(void) {
  return g_epoch_initialized &&
         !g_alpha_epoch_install_in_progress &&
         g_alpha_epoch_last_smartzero.complete &&
         g_alpha_epoch_last_smartzero.sequence ==
             g_alpha_smartzero_install_last_committed_smartzero_sequence;
}

bool clocks_alpha_epoch_install_in_progress(void) {
  return g_alpha_epoch_install_in_progress;
}
uint32_t clocks_alpha_smartzero_install_attempt_count(void) {
  return g_alpha_smartzero_install_attempt_count;
}
uint32_t clocks_alpha_smartzero_install_commit_count(void) {
  return g_alpha_smartzero_install_commit_count;
}
uint32_t clocks_alpha_smartzero_install_failure_count(void) {
  return g_alpha_smartzero_install_failure_count;
}
uint32_t clocks_alpha_smartzero_install_last_stage(void) {
  return g_alpha_smartzero_install_last_stage;
}
const char* clocks_alpha_smartzero_install_last_stage_name(void) {
  return smartzero_install_stage_name(g_alpha_smartzero_install_last_stage);
}
uint32_t clocks_alpha_smartzero_install_last_failure_stage(void) {
  return g_alpha_smartzero_install_last_failure_stage;
}
const char* clocks_alpha_smartzero_install_last_failure_stage_name(void) {
  return smartzero_install_stage_name(g_alpha_smartzero_install_last_failure_stage);
}
uint32_t clocks_alpha_smartzero_install_last_failure_code(void) {
  return g_alpha_smartzero_install_last_failure_code;
}
uint32_t clocks_alpha_smartzero_install_last_live_sequence(void) {
  return g_alpha_smartzero_install_last_live_sequence;
}
uint32_t clocks_alpha_smartzero_install_last_prior_epoch_sequence(void) {
  return g_alpha_smartzero_install_last_prior_epoch_sequence;
}
uint32_t clocks_alpha_smartzero_install_last_committed_epoch_sequence(void) {
  return g_alpha_smartzero_install_last_committed_epoch_sequence;
}
uint32_t clocks_alpha_smartzero_install_last_committed_smartzero_sequence(void) {
  return g_alpha_smartzero_install_last_committed_smartzero_sequence;
}
bool clocks_alpha_smartzero_install_last_success(void) {
  return g_alpha_smartzero_install_last_success;
}
bool clocks_alpha_smartzero_install_last_atomic(void) {
  return g_alpha_smartzero_install_last_atomic;
}
const char* clocks_alpha_smartzero_install_last_reason(void) {
  return g_alpha_smartzero_install_last_reason;
}

uint32_t clocks_alpha_smartzero_begin_count(void) { return g_alpha_smartzero_begin_count; }
uint32_t clocks_alpha_smartzero_begin_failures(void) { return g_alpha_smartzero_begin_failures; }
bool clocks_alpha_smartzero_pending_active(void) { return g_alpha_smartzero_pending_active; }
const char* clocks_alpha_smartzero_pending_reason(void) {
  return g_alpha_smartzero_pending_active ? g_alpha_smartzero_pending_reason : "";
}
void clocks_alpha_smartzero_pending_clear(void) {
  g_alpha_smartzero_pending_active = false;
  g_alpha_smartzero_pending_reason[0] = '\0';
}
bool clocks_alpha_smartzero_last_begin_preserved_epoch(void) {
  return g_alpha_smartzero_last_begin_preserved_epoch;
}
uint32_t clocks_alpha_smartzero_last_begin_preserved_epoch_sequence(void) {
  return g_alpha_smartzero_last_begin_preserved_epoch_sequence;
}
uint32_t clocks_alpha_smartzero_begin_preserved_epoch_count(void) {
  return g_alpha_smartzero_begin_preserved_epoch_count;
}
uint32_t clocks_alpha_smartzero_begin_cold_count(void) {
  return g_alpha_smartzero_begin_cold_count;
}
const char* clocks_alpha_smartzero_last_begin_reason(void) {
  return g_alpha_smartzero_last_begin_reason;
}

static void alpha_smartzero_install_mark_stage(uint32_t stage) {
  g_alpha_smartzero_install_last_stage = stage;
  clocks_alpha_dmb();
}

static void alpha_smartzero_install_begin_transaction(const char* reason) {
  safeCopy(g_alpha_smartzero_install_last_reason,
           sizeof(g_alpha_smartzero_install_last_reason),
           (reason && *reason) ? reason : "smartzero");
  g_alpha_epoch_install_in_progress = true;
  g_alpha_smartzero_install_attempt_count++;
  g_alpha_smartzero_install_last_success = false;
  g_alpha_smartzero_install_last_atomic = false;
  g_alpha_smartzero_install_last_failure_code = SMARTZERO_INSTALL_FAIL_NONE;
  g_alpha_smartzero_install_last_failure_stage = SMARTZERO_INSTALL_STAGE_IDLE;
  g_alpha_smartzero_install_last_live_sequence = 0;
  g_alpha_smartzero_install_last_prior_epoch_sequence = g_alpha_epoch_sequence;
  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_SNAPSHOT);
}

static void alpha_smartzero_install_fail(uint32_t stage,
                                         uint32_t failure_code) {
  g_alpha_smartzero_install_last_failure_stage = stage;
  g_alpha_smartzero_install_last_failure_code = failure_code;
  g_alpha_smartzero_install_last_success = false;
  g_alpha_smartzero_install_last_atomic = false;
  g_alpha_smartzero_install_failure_count++;
  g_alpha_epoch_install_failures++;
  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_FAIL);
  g_alpha_epoch_install_in_progress = false;
  clocks_alpha_smartzero_pending_clear();
}

static void alpha_smartzero_install_commit(uint32_t committed_epoch_sequence,
                                           uint32_t committed_smartzero_sequence) {
  g_alpha_smartzero_install_last_committed_epoch_sequence = committed_epoch_sequence;
  g_alpha_smartzero_install_last_committed_smartzero_sequence = committed_smartzero_sequence;
  g_alpha_smartzero_install_last_success = true;
  g_alpha_smartzero_install_last_atomic = true;
  g_alpha_smartzero_install_commit_count++;
  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_COMMIT);
  g_alpha_epoch_install_in_progress = false;
}

static bool smartzero_completed_proof_valid(const interrupt_smartzero_snapshot_t& z) {
  if (!z.complete || z.running || z.aborted) return false;

  const interrupt_smartzero_lane_snapshot_t& vclock = z.lanes[0];
  const interrupt_smartzero_lane_snapshot_t& ocxo1  = z.lanes[1];
  const interrupt_smartzero_lane_snapshot_t& ocxo2  = z.lanes[2];

  return vclock.kind == interrupt_subscriber_kind_t::VCLOCK &&
         ocxo1.kind  == interrupt_subscriber_kind_t::OCXO1 &&
         ocxo2.kind  == interrupt_subscriber_kind_t::OCXO2 &&
         vclock.state == interrupt_smartzero_lane_state_t::LOCKED &&
         ocxo1.state  == interrupt_smartzero_lane_state_t::LOCKED &&
         ocxo2.state  == interrupt_smartzero_lane_state_t::LOCKED &&
         vclock.accepted_count != 0 && ocxo1.accepted_count != 0 &&
         ocxo2.accepted_count != 0 &&
         vclock.anchor_dwt != 0 && ocxo1.anchor_dwt != 0 &&
         ocxo2.anchor_dwt != 0 &&
         vclock.anchor_counter32 != 0 && ocxo1.anchor_counter32 != 0 &&
         ocxo2.anchor_counter32 != 0;
}

// Runtime PPS/VCLOCK sampling no longer requires the epoch-capture packet.
// START/ZERO still require a valid capture packet, but ordinary per-second
// sampling only needs the authored PPS/VCLOCK snapshot and Alpha's measured
// clock state.  These counters audit capture availability/sequence agreement
// without killing an otherwise coherent campaign.
volatile uint32_t g_alpha_runtime_epoch_capture_missing_count = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_sequence_mismatch_count = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_snap_sequence = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_cap_sequence = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_snap_counter32 = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_cap_counter32 = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_snap_dwt = 0;
volatile uint32_t g_alpha_runtime_epoch_capture_last_cap_dwt = 0;
volatile bool     g_alpha_runtime_epoch_capture_last_cap_valid = false;
volatile bool     g_alpha_runtime_epoch_capture_last_cap_vclock_valid = false;
volatile bool     g_alpha_runtime_epoch_capture_last_cap_all_lanes_valid = false;

static void alpha_reset_canonical_clock_state_for_new_epoch(void) {
  const bool saved_dwt_calibration_valid = g_dwt_calibration_valid;
  const uint32_t saved_dwt_cycles_per_second = g_dwt_cycles_between_pps_vclock;

  g_gnss_ns_at_pps_vclock = 0;
  g_ocxo1_measured_gnss_ns_at_pps_vclock = 0;
  g_ocxo2_measured_gnss_ns_at_pps_vclock = 0;
  g_ocxo1_physical_measured_gnss_ns_at_pps_vclock = 0;
  g_ocxo2_physical_measured_gnss_ns_at_pps_vclock = 0;
  g_dwt_at_pps_vclock = 0;
  g_dwt_cycle_count_total = 0;
  g_dwt_cycles_between_pps_vclock = saved_dwt_calibration_valid
      ? saved_dwt_cycles_per_second
      : (uint32_t)DWT_EXPECTED_PER_PPS;
  g_dwt_calibration_valid = saved_dwt_calibration_valid;
  g_counter32_at_pps_vclock = 0;
  g_prev_dwt_at_vclock_event = 0;
  g_prev_observed_dwt_at_vclock_event_valid = false;
  g_prev_observed_dwt_at_vclock_event = 0;
  g_prev_pps_vclock_dwt_at_edge = 0;
  g_prev_pps_vclock_dwt_at_edge_valid = false;
  g_pps_dwt_at_edge = 0;
  g_pps_dwt_cycles_between_edges = 0;
  g_pps_dwt_cycles_between_edges_valid = false;
  g_pps_vclock_phase_cycles = 0;
  alpha_pps_vclock_edge_forensics_reset();
  g_prev_pps_dwt_at_edge = 0;
  g_prev_pps_dwt_at_edge_valid = false;
  g_vclock_event_count = 0;

  g_alpha_runtime_epoch_capture_missing_count = 0;
  g_alpha_runtime_epoch_capture_sequence_mismatch_count = 0;
  g_alpha_runtime_epoch_capture_last_snap_sequence = 0;
  g_alpha_runtime_epoch_capture_last_cap_sequence = 0;
  g_alpha_runtime_epoch_capture_last_snap_counter32 = 0;
  g_alpha_runtime_epoch_capture_last_cap_counter32 = 0;
  g_alpha_runtime_epoch_capture_last_snap_dwt = 0;
  g_alpha_runtime_epoch_capture_last_cap_dwt = 0;
  g_alpha_runtime_epoch_capture_last_cap_valid = false;
  g_alpha_runtime_epoch_capture_last_cap_vclock_valid = false;
  g_alpha_runtime_epoch_capture_last_cap_all_lanes_valid = false;

  g_vclock_clock = {};
  g_vclock_measurement = {};
  g_ocxo1_clock = {};
  g_ocxo2_clock = {};
  g_ocxo1_measurement = {};
  g_ocxo2_measurement = {};
  g_pps_witness_diag = {};
  g_ocxo1_interrupt_diag = {};
  g_ocxo2_interrupt_diag = {};
  alpha_forensics_reset_all();
  alpha_ticks64_reset_all();
  alpha_measured_ns_reset_all();
  alpha_ocxo_pps_projection_reset_all();
  alpha_ocxo_visible_origin_reset_all();
  clocks_static_prediction_reset_all();

  dwt_cycle_count_total = 0;
  gnss_raw_64           = 0;
  ocxo1_measured_gnss_ticks_64        = 0;
  ocxo2_measured_gnss_ticks_64        = 0;

  welford_reset(welford_dwt);
  welford_reset(welford_vclock);
  welford_reset(welford_ocxo1);
  welford_reset(welford_ocxo2);
}

bool clocks_alpha_begin_smartzero_epoch(const char* reason) {
  const char* begin_reason = (reason && *reason) ? reason : "smartzero";
  const bool preserved_epoch = g_epoch_initialized;

  // Beginning SmartZero is not an epoch event. If a service/science epoch is
  // already installed, keep it alive while the replacement proof is acquired.
  // The destructive rebase remains in clocks_alpha_zero_from_smartzero(), where
  // a complete three-lane proof exists and can be installed atomically.
  safeCopy(g_alpha_smartzero_last_begin_reason,
           sizeof(g_alpha_smartzero_last_begin_reason),
           begin_reason);
  safeCopy(g_alpha_smartzero_pending_reason,
           sizeof(g_alpha_smartzero_pending_reason),
           begin_reason);
  g_alpha_smartzero_pending_active = true;
  g_alpha_smartzero_last_begin_preserved_epoch = preserved_epoch;
  g_alpha_smartzero_last_begin_preserved_epoch_sequence =
      preserved_epoch ? g_alpha_epoch_sequence : 0;

  if (preserved_epoch) {
    g_alpha_smartzero_begin_preserved_epoch_count++;
  } else {
    // Cold-start path: no epoch exists yet, so there is no service coordinate
    // system to preserve. Retain the old cleanup only for first-ever acquisition.
    alpha_reset_canonical_clock_state_for_new_epoch();
    g_epoch_initialized = false;
    timebase_invalidate();
    time_pps_vclock_epoch_reset(0, 0);
    time_clock_reset_all();
    g_alpha_smartzero_begin_cold_count++;
    safeCopy(g_alpha_epoch_last_reason, sizeof(g_alpha_epoch_last_reason),
             begin_reason);
  }

  const bool ok = interrupt_smartzero_begin();
  if (ok) {
    g_alpha_smartzero_begin_count++;
  } else {
    g_alpha_smartzero_pending_active = false;
    g_alpha_smartzero_pending_reason[0] = '\0';
    g_alpha_smartzero_begin_failures++;
    if (!preserved_epoch) {
      g_alpha_epoch_install_failures++;
    }
  }
  return ok;
}

bool clocks_alpha_zero_from_smartzero(const char* reason) {
  const char* install_reason = (reason && *reason) ? reason : "smartzero";

  if (g_alpha_epoch_install_in_progress) {
    g_alpha_smartzero_install_attempt_count++;
    safeCopy(g_alpha_smartzero_install_last_reason,
             sizeof(g_alpha_smartzero_install_last_reason),
             install_reason);
    g_alpha_smartzero_install_last_failure_stage = SMARTZERO_INSTALL_STAGE_SNAPSHOT;
    g_alpha_smartzero_install_last_failure_code = SMARTZERO_INSTALL_FAIL_REENTRANT;
    g_alpha_smartzero_install_last_success = false;
    g_alpha_smartzero_install_last_atomic = false;
    g_alpha_smartzero_install_failure_count++;
    g_alpha_epoch_install_failures++;
    g_alpha_smartzero_install_last_stage = SMARTZERO_INSTALL_STAGE_FAIL;
    return false;
  }

  alpha_smartzero_install_begin_transaction(install_reason);

  interrupt_smartzero_snapshot_t z{};
  if (!interrupt_smartzero_snapshot(&z) || !z.complete) {
    alpha_smartzero_install_fail(SMARTZERO_INSTALL_STAGE_SNAPSHOT,
                                 SMARTZERO_INSTALL_FAIL_NO_COMPLETE_PROOF);
    return false;
  }
  g_alpha_smartzero_install_last_live_sequence = z.sequence;

  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_VALIDATE);
  if (!smartzero_completed_proof_valid(z)) {
    alpha_smartzero_install_fail(SMARTZERO_INSTALL_STAGE_VALIDATE,
                                 SMARTZERO_INSTALL_FAIL_BAD_LANE_PROOF);
    return false;
  }

  const interrupt_smartzero_lane_snapshot_t& vclock = z.lanes[0];
  const interrupt_smartzero_lane_snapshot_t& ocxo1  = z.lanes[1];
  const interrupt_smartzero_lane_snapshot_t& ocxo2  = z.lanes[2];
  const uint32_t next_epoch_sequence = g_alpha_epoch_sequence + 1U;

  // Commit window begins.  A complete proof exists, so it is now lawful to
  // replace the installed science epoch.  epoch_ready() is false while this
  // transaction runs, so Alpha event consumers cannot apply events against
  // reset ledgers or half-installed zero offsets.
  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_RESET_STATE);
  alpha_reset_canonical_clock_state_for_new_epoch();

  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_INSTALL_ANCHORS);
  g_gnss_ns_at_pps_vclock = 0;
  g_dwt_at_pps_vclock = vclock.anchor_dwt;
  g_counter32_at_pps_vclock = vclock.anchor_counter32;
  g_prev_dwt_at_vclock_event = vclock.anchor_dwt;
  g_prev_pps_vclock_dwt_at_edge = vclock.anchor_dwt;
  g_prev_pps_vclock_dwt_at_edge_valid = true;

  alpha_ocxo_visible_origin_capture_from_smartzero(
      time_clock_id_t::OCXO1,
      next_epoch_sequence,
      z.sequence,
      vclock.anchor_dwt,
      ocxo1.anchor_dwt,
      alpha_ocxo_visible_origin_select_cps(vclock, ocxo1));
  alpha_ocxo_visible_origin_capture_from_smartzero(
      time_clock_id_t::OCXO2,
      next_epoch_sequence,
      z.sequence,
      vclock.anchor_dwt,
      ocxo2.anchor_dwt,
      alpha_ocxo_visible_origin_select_cps(vclock, ocxo2));

  const uint64_t ocxo1_visible_epoch_ns =
      alpha_ocxo_visible_ns_from_physical(time_clock_id_t::OCXO1, 0ULL);
  const uint64_t ocxo2_visible_epoch_ns =
      alpha_ocxo_visible_ns_from_physical(time_clock_id_t::OCXO2, 0ULL);

  alpha_ticks64_install_zero(g_vclock_ticks64, vclock.anchor_counter32);
  alpha_ticks64_install_zero(g_ocxo1_ticks64, ocxo1.anchor_counter32);
  alpha_ticks64_install_zero(g_ocxo2_ticks64, ocxo2.anchor_counter32);

  // OCXO measured ledgers have their own mathematically-qualified zero
  // anchors. The first post-zero OCXO edge measures from that OCXO anchor,
  // not from the VCLOCK anchor.
  g_ocxo1_measured_ns.initialized = true;
  g_ocxo1_measured_ns.ns_at_edge = 0;
  g_ocxo1_measured_ns.dwt_at_edge = ocxo1.anchor_dwt;
  g_ocxo1_measured_ns.dwt64_at_edge = clocks_dwt_cycles_at_dwt(ocxo1.anchor_dwt);
  g_ocxo2_measured_ns.initialized = true;
  g_ocxo2_measured_ns.ns_at_edge = 0;
  g_ocxo2_measured_ns.dwt_at_edge = ocxo2.anchor_dwt;
  g_ocxo2_measured_ns.dwt64_at_edge = clocks_dwt_cycles_at_dwt(ocxo2.anchor_dwt);

  alpha_ocxo_edge_history_install_zero(time_clock_id_t::OCXO1,
                                       ocxo1.anchor_dwt,
                                       ocxo1.anchor_counter32);
  alpha_ocxo_edge_history_install_zero(time_clock_id_t::OCXO2,
                                       ocxo2.anchor_dwt,
                                       ocxo2.anchor_counter32);

  clock_mark_epoch_zero(g_vclock_clock);
  clock_mark_epoch_zero(g_ocxo1_clock);
  clock_mark_epoch_zero(g_ocxo2_clock);

  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_DWT64);
  (void)clocks_dwt64_epoch_reset_at_dwt32(vclock.anchor_dwt, nullptr);
  dwt64_anchor_reset_to_dwt32(vclock.anchor_dwt, g_dwt64_epoch_raw_cycles);

  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_TIME_ANCHORS);
  time_pps_vclock_epoch_reset(vclock.anchor_dwt, vclock.anchor_counter32);
  time_clock_epoch_reset(time_clock_id_t::VCLOCK, vclock.anchor_dwt, 0);
  time_clock_epoch_reset(time_clock_id_t::OCXO1,  ocxo1.anchor_dwt, ocxo1_visible_epoch_ns);
  time_clock_epoch_reset(time_clock_id_t::OCXO2,  ocxo2.anchor_dwt, ocxo2_visible_epoch_ns);

  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_TIMEPOP_REAUTHOR);
  // CLOCKS has now selected and installed the OCXO logical zero-offset
  // counter32 values. Explicitly hand those anchors back to process_interrupt
  // so the OCXO local compare ladders are re-authored from the installed
  // SmartZero proof instead of relying on acquisition-side residue.
  interrupt_ocxo_logical_grid_epoch(ocxo1.anchor_counter32,
                                    ocxo2.anchor_counter32);

  // VCLOCK synthetic coordinate generation changed. Re-author recurring
  // TimePop timers and cancel unsafe old-coordinate one-shots before normal
  // scheduling resumes.
  timepop_epoch_changed(next_epoch_sequence);

  // Public commit point. From here on, all epoch-report fields, installed
  // SmartZero proof, and epoch sequence refer to the newly installed proof.
  alpha_smartzero_install_mark_stage(SMARTZERO_INSTALL_STAGE_COMMIT);
  g_alpha_epoch_sequence = next_epoch_sequence;
  g_alpha_epoch_last_capture_sequence = z.sequence;
  g_alpha_epoch_last_capture_window_cycles = 0;
  g_alpha_epoch_last_vclock_capture_valid = true;
  g_alpha_epoch_last_all_lanes_valid = true;
  g_alpha_epoch_last_dwt_at_edge = vclock.anchor_dwt;
  g_alpha_epoch_last_vclock_counter32 = vclock.anchor_counter32;
  g_alpha_epoch_last_ocxo1_counter32 = ocxo1.anchor_counter32;
  g_alpha_epoch_last_ocxo2_counter32 = ocxo2.anchor_counter32;
  g_alpha_epoch_last_vclock_zero_valid = true;
  g_alpha_epoch_last_ocxo1_zero_valid = true;
  g_alpha_epoch_last_ocxo2_zero_valid = true;
  g_alpha_epoch_last_vclock_hardware16_observed = vclock.anchor_hardware16;
  g_alpha_epoch_last_vclock_hardware16_selected = vclock.anchor_hardware16;
  g_alpha_epoch_last_ocxo1_hardware16 = ocxo1.anchor_hardware16;
  g_alpha_epoch_last_ocxo2_hardware16 = ocxo2.anchor_hardware16;
  g_alpha_epoch_last_smartzero = z;

  safeCopy(g_alpha_epoch_last_reason, sizeof(g_alpha_epoch_last_reason),
           install_reason);
  g_alpha_epoch_install_count++;
  g_epoch_initialized = true;
  clocks_alpha_smartzero_pending_clear();

  alpha_smartzero_install_commit(next_epoch_sequence, z.sequence);
  return true;
}

bool clocks_alpha_zero_from_interrupt_capture(const char* reason) {
  // Compatibility wrapper: the legacy capture-packet ZERO path is retired.
  // All callers now install from the completed SmartZero proof surface.
  return clocks_alpha_zero_from_smartzero(reason);
}

uint32_t clocks_alpha_epoch_sequence(void) { return g_alpha_epoch_sequence; }
uint32_t clocks_alpha_epoch_install_count(void) { return g_alpha_epoch_install_count; }
uint32_t clocks_alpha_epoch_install_failures(void) { return g_alpha_epoch_install_failures; }
uint32_t clocks_alpha_epoch_last_capture_sequence(void) { return g_alpha_epoch_last_capture_sequence; }
uint32_t clocks_alpha_epoch_last_capture_window_cycles(void) { return g_alpha_epoch_last_capture_window_cycles; }
bool clocks_alpha_epoch_last_vclock_capture_valid(void) { return g_alpha_epoch_last_vclock_capture_valid; }
bool clocks_alpha_epoch_last_all_lanes_valid(void) { return g_alpha_epoch_last_all_lanes_valid; }
uint32_t clocks_alpha_epoch_last_dwt_at_edge(void) { return g_alpha_epoch_last_dwt_at_edge; }
uint32_t clocks_alpha_epoch_last_vclock_counter32(void) { return g_alpha_epoch_last_vclock_counter32; }
uint32_t clocks_alpha_epoch_last_ocxo1_counter32(void) { return g_alpha_epoch_last_ocxo1_counter32; }
uint32_t clocks_alpha_epoch_last_ocxo2_counter32(void) { return g_alpha_epoch_last_ocxo2_counter32; }
bool clocks_alpha_epoch_last_vclock_zero_valid(void) { return g_alpha_epoch_last_vclock_zero_valid; }
bool clocks_alpha_epoch_last_ocxo1_zero_valid(void) { return g_alpha_epoch_last_ocxo1_zero_valid; }
bool clocks_alpha_epoch_last_ocxo2_zero_valid(void) { return g_alpha_epoch_last_ocxo2_zero_valid; }
uint16_t clocks_alpha_epoch_last_vclock_hardware16_observed(void) { return g_alpha_epoch_last_vclock_hardware16_observed; }
uint16_t clocks_alpha_epoch_last_vclock_hardware16_selected(void) { return g_alpha_epoch_last_vclock_hardware16_selected; }
uint16_t clocks_alpha_epoch_last_ocxo1_hardware16(void) { return g_alpha_epoch_last_ocxo1_hardware16; }
uint16_t clocks_alpha_epoch_last_ocxo2_hardware16(void) { return g_alpha_epoch_last_ocxo2_hardware16; }
const char* clocks_alpha_epoch_last_reason(void) { return g_alpha_epoch_last_reason; }
bool clocks_alpha_epoch_initialized(void) { return g_epoch_initialized && !g_alpha_epoch_install_in_progress; }

// ============================================================================
// Direct event application
// ============================================================================

static inline bool alpha_clock_is_ocxo(time_clock_id_t clock) {
  return clock == time_clock_id_t::OCXO1 ||
         clock == time_clock_id_t::OCXO2;
}

static uint32_t alpha_physics_dwt_at_event(time_clock_id_t clock,
                                           const interrupt_event_t& event,
                                           const interrupt_capture_diag_t* diag) {
  if (alpha_clock_is_ocxo(clock) && diag &&
      diag->ocxo_service_corrected_dwt_at_event != 0) {
    return diag->ocxo_service_corrected_dwt_at_event;
  }

  return event.dwt_at_event;
}

static void clocks_apply_epoch_counter_edge(clock_state_t& clock,
                                            clock_measurement_t& meas,
                                            const interrupt_event_t& event,
                                            const interrupt_capture_diag_t* diag,
                                            uint32_t zero_offset_counter32,
                                            time_clock_id_t time_clock) {
  alpha_event_flow_note_apply_entry(time_clock);
  const bool is_ocxo = alpha_clock_is_ocxo(time_clock);
  const bool had_previous = (meas.prev_dwt_at_edge != 0);

  interrupt_event_t applied_event = event;
  const uint32_t physics_dwt =
      alpha_physics_dwt_at_event(time_clock, event, diag);

  // Retired quiet-zone/sample-offset doctrine.  OCXO subscriber events are now
  // ordinary edge facts.  Do not subtract the historical +250 us / +750 us
  // sample phase from either DWT or counter32; doing so creates an OCXO-only
  // boundary transform in the raw custody chain.
  applied_event.dwt_at_event = physics_dwt;

  interrupt_capture_diag_t local_diag{};
  const interrupt_capture_diag_t* applied_diag = diag;
  if (diag) {
    local_diag = *diag;
    if (is_ocxo) {
      // Preserve the old field names as passive compatibility surfaces, but
      // publish the new truth: sample == boundary == applied event, with zero
      // phase correction.  process_interrupt may still populate legacy
      // ocxo_sample_phase_* diagnostics; Alpha deliberately retires them here.
      local_diag.ocxo_sample_phase_valid = false;
      local_diag.ocxo_sample_phase_ticks = 0;
      local_diag.ocxo_sample_phase_ns = 0;
      local_diag.ocxo_sample_phase_us = 0;
      local_diag.ocxo_sample_period_ticks = 0;
      local_diag.ocxo_sample_dwt_at_event = applied_event.dwt_at_event;
      local_diag.ocxo_sample_counter32_at_event = applied_event.counter32_at_event;
      local_diag.ocxo_boundary_dwt_at_event = applied_event.dwt_at_event;
      local_diag.ocxo_boundary_counter32_at_event = applied_event.counter32_at_event;
      local_diag.ocxo_boundary_correction_cycles = 0;
      local_diag.dwt_at_event = applied_event.dwt_at_event;
      local_diag.counter32_at_event = applied_event.counter32_at_event;
    }
    applied_diag = &local_diag;
  }

  uint64_t logical_ticks64 = 0;
  uint64_t counter_ns_now = 0;
  uint32_t counter32_delta_since_zero_offset = 0;
  uint32_t counter32_delta_since_previous_event = 0;

  const bool ticks64_ok = alpha_ticks64_apply_event(time_clock,
                                 applied_event.counter32_at_event,
                                 zero_offset_counter32,
                                 &logical_ticks64,
                                 &counter_ns_now,
                                 &counter32_delta_since_zero_offset,
                                 &counter32_delta_since_previous_event);
  alpha_event_flow_note_ticks64(time_clock, ticks64_ok, applied_event,
                                counter32_delta_since_previous_event);
  if (!ticks64_ok) {
    clocks_watchdog_anomaly("alpha_clock_apply_failed",
                            (uint32_t)((uint8_t)time_clock),
                            applied_event.counter32_at_event,
                            applied_event.dwt_at_event,
                            0);
    return;
  }

  interrupt_integrity_note_counter32(
      alpha_integrity_interrupt_kind(time_clock),
      (uint32_t)(logical_ticks64 / (uint64_t)VCLOCK_COUNTS_PER_SECOND),
      had_previous,
      counter32_delta_since_previous_event,
      (uint32_t)VCLOCK_COUNTS_PER_SECOND,
      applied_event.counter32_at_event);

  uint64_t ns_now = counter_ns_now;
  uint32_t dwt_cycles_between_edges = had_previous
      ? (applied_event.dwt_at_event - meas.prev_dwt_at_edge)
      : 0U;
  uint64_t gnss_ns_between_edges = 0;
  int64_t second_residual_ns = 0;
  int64_t window_error_ns = 0;
  uint64_t physical_measured_ns_at_edge = 0;
  uint64_t visible_ns_at_edge = ns_now;
  bool visible_origin_phase_valid = false;
  uint32_t visible_origin_phase_offset_ns = 0;

  if (is_ocxo) {
    uint32_t measured_dwt_cycles = 0;
    uint64_t real_interval_ns = 0;
    int64_t residual_fast_ns = 0;
    // Science edge species: the observed ISR coordinate feeds the bookends.
    // Its lattice/latency noise is white and unbiased, which the Welford
    // surfaces average honestly; the published anchored endpoint stays the
    // subscriber surface.  Long-term ruling (2026-06-12): the lower-envelope
    // projection replaces this as the science edge once the slope engine
    // exists.
    const uint32_t science_edge_dwt =
        (applied_diag != nullptr && applied_diag->dwt_original_at_event != 0U)
            ? applied_diag->dwt_original_at_event
            : applied_event.dwt_at_event;
    const uint64_t physical_ns_now =
        alpha_ocxo_apply_measured_second(time_clock,
                                         science_edge_dwt,
                                         &measured_dwt_cycles,
                                         &real_interval_ns,
                                         &residual_fast_ns);
    physical_measured_ns_at_edge = physical_ns_now;
    visible_origin_phase_valid =
        alpha_ocxo_visible_origin_phase_offset_ns(time_clock,
                                                  &visible_origin_phase_offset_ns);
    const uint64_t measured_visible_ns =
        alpha_ocxo_visible_ns_from_physical(time_clock, physical_ns_now);
    ns_now = measured_visible_ns;

    // Split the two OCXO edge species explicitly:
    //
    //   measured_visible_ns
    //     Conservative bridge-resolved science ledger.  It intentionally has a
    //     one-edge lag because the closing PPS/GNSS anchor arrives after the
    //     OCXO edge.  Welfords/servo/forensics may consume this.
    //
    //   public_edge_ns_for_history
    //     The OCXO clock-domain edge ledger: synthetic 10 MHz ticks since the
    //     installed SmartZero zero-offset.  This is NOT bridge-lagged.  The
    //     PPS-row public projection must consume this species, then apply the
    //     fixed visible/public origin offset.  Feeding it measured_visible_ns is
    //     exactly the one-second public branch leak.
    const uint64_t public_edge_ns_for_history = counter_ns_now;
    visible_ns_at_edge = public_edge_ns_for_history;
    alpha_integrity_note_ocxo_projected_gnss_second(
        time_clock,
        (uint32_t)(logical_ticks64 / (uint64_t)VCLOCK_COUNTS_PER_SECOND),
        applied_event.dwt_at_event);
    alpha_event_flow_note_measured(time_clock, measured_dwt_cycles,
                                   real_interval_ns, residual_fast_ns,
                                   measured_visible_ns);

    const bool sample_gnss_available = (event.gnss_ns_at_event != 0);
    const uint64_t sample_gnss_ns_at_event = sample_gnss_available
        ? event.gnss_ns_at_event
        : 0ULL;
    const uint64_t boundary_gnss_ns_at_edge = sample_gnss_ns_at_event;
    const bool boundary_gnss_available = sample_gnss_available;

    alpha_ocxo_edge_history_record(time_clock,
                                   applied_event.dwt_at_event,
                                   applied_event.counter32_at_event,
                                   public_edge_ns_for_history,
                                   measured_visible_ns,
                                   sample_gnss_available,
                                   sample_gnss_ns_at_event,
                                   boundary_gnss_available,
                                   boundary_gnss_ns_at_edge,
                                   0U,
                                   0U);

    dwt_cycles_between_edges = measured_dwt_cycles;
    gnss_ns_between_edges = real_interval_ns;
    second_residual_ns = residual_fast_ns;
    window_error_ns = -residual_fast_ns;

    const alpha_measured_ns_clock_t* mstore = alpha_measured_ns_store(time_clock);
    if (mstore) {
      meas.bridge_anchored =
          (real_interval_ns != 0) && mstore->last_resolved_via_bridge;
      meas.bridge_phi_cycles = mstore->bridge_last_phi_cycles;
      meas.bridge_span_cycles = mstore->bridge_last_span_cycles;
      meas.bridge_resolved_count = mstore->bridge_resolved_count;
      meas.bridge_fallback_count = mstore->bridge_fallback_count;
    }
  } else if (had_previous) {
    const uint64_t previous_bridge_gnss_ns = meas.prev_gnss_ns_at_edge;
    const bool bridge_interval_available =
        (event.gnss_ns_at_event != 0 && previous_bridge_gnss_ns != 0 &&
         event.gnss_ns_at_event >= previous_bridge_gnss_ns);
    if (bridge_interval_available) {
      gnss_ns_between_edges = event.gnss_ns_at_event - previous_bridge_gnss_ns;
      second_residual_ns =
          (int64_t)NS_PER_SECOND_U64 - (int64_t)gnss_ns_between_edges;
      window_error_ns =
          (int64_t)gnss_ns_between_edges - (int64_t)NS_PER_SECOND_U64;
    }
  }

  // Public process_time/time.h coordinates for PPS-founded clocks are authored
  // at the canonical PPS/VCLOCK row.  OCXO callbacks already stay out of
  // time.h because their science ledger intentionally has a one-edge lag.
  // VCLOCK subscriber callbacks are now treated the same way for public time:
  // they remain measurement/forensic peers, but the user-visible VCLOCK/GNSS
  // basis is the exact PPS/VCLOCK ledger authored in
  // alpha_sample_all_clocks_at_pps_vclock().  This keeps REPORT_SUMMARY from
  // comparing OCXO public PPS-row values against a VCLOCK basis from a different
  // rail/phase.
  if (!is_ocxo && time_clock != time_clock_id_t::VCLOCK) {
    (void)time_clock_update(time_clock, applied_event.dwt_at_event, ns_now);
    alpha_event_flow_note_time_update(time_clock, ns_now);
  }

  clock.ledger_ns_count_at_edge = ns_now;
  clock.ledger_ns_count_at_pps_vclock = ns_now;
  const uint64_t reference_gnss_ns_at_edge =
      is_ocxo
          ? ns_now
          : ((event.gnss_ns_at_event != 0) ? event.gnss_ns_at_event : ns_now);
  clock.gnss_ns_at_edge = reference_gnss_ns_at_edge;
  clock.phase_offset_ns =
      (int64_t)reference_gnss_ns_at_edge - (int64_t)ns_now;
  clock.zero_established = true;
  clock.window_error_ns = window_error_ns;

  meas.dwt_at_edge = applied_event.dwt_at_event;
  if (!had_previous) {
    meas.gnss_ns_between_edges = 0;
    meas.dwt_cycles_between_edges = 0;
    meas.second_residual_ns = 0;
  } else {
    meas.gnss_ns_between_edges = gnss_ns_between_edges;
    meas.dwt_cycles_between_edges = dwt_cycles_between_edges;
    meas.second_residual_ns = second_residual_ns;
    clock.window_checks++;
    if (llabs(clock.window_error_ns) > CLOCK_WINDOW_TOLERANCE_NS) {
      clock.window_mismatches++;
    }
  }

  // Previous-edge bridge truth. For OCXO lanes the reference coordinate is the
  // measured GNSS-elapsed ledger because process_interrupt may not carry a
  // direct GNSS timestamp in the event.
  meas.prev_gnss_ns_at_edge = reference_gnss_ns_at_edge;
  meas.prev_dwt_at_edge = applied_event.dwt_at_event;

  if (had_previous && dwt_cycles_between_edges != 0) {
    alpha_static_prediction_record(time_clock, dwt_cycles_between_edges);
    alpha_event_flow_note_static_prediction(time_clock, dwt_cycles_between_edges);
  }

  alpha_forensics_publish(time_clock, clock, meas, applied_event, applied_diag,
                          alpha_zero_offset_valid(time_clock),
                          zero_offset_counter32,
                          counter32_delta_since_zero_offset,
                          counter32_delta_since_previous_event,
                          logical_ticks64,
                          counter_ns_now,
                          ns_now,
                          physical_measured_ns_at_edge,
                          visible_ns_at_edge,
                          visible_origin_phase_valid,
                          visible_origin_phase_offset_ns);
  alpha_event_flow_note_complete(time_clock);
}

static inline bool usable_clock_event(const interrupt_event_t&) {
  return epoch_ready();
}

static void apply_ocxo_event(clock_state_t& clock,
                             clock_measurement_t& meas,
                             interrupt_capture_diag_t& diag_dst,
                             const interrupt_event_t& event,
                             const interrupt_capture_diag_t* diag,
                             uint32_t epoch_counter32,
                             time_clock_id_t time_clock) {
  alpha_event_flow_note_callback(time_clock, event, diag);
  clocks_capture_interrupt_diag(diag_dst, diag);
  if (!usable_clock_event(event)) {
    alpha_event_flow_note_reject_epoch(time_clock, event);
    return;
  }
  alpha_event_flow_note_callback_accepted(time_clock);
  clocks_apply_epoch_counter_edge(clock, meas, event, diag,
                                  epoch_counter32, time_clock);
}

static void vclock_callback(const interrupt_event_t& event,
                            const interrupt_capture_diag_t* diag,
                            void*) {
  alpha_event_flow_note_callback(time_clock_id_t::VCLOCK, event, diag);
  clocks_capture_interrupt_diag(g_pps_witness_diag, diag);
  g_last_vclock_event_counter32_at_event = event.counter32_at_event;

  if (!epoch_ready()) {
    alpha_event_flow_note_reject_epoch(time_clock_id_t::VCLOCK, event);
    return;
  }
  alpha_event_flow_note_callback_accepted(time_clock_id_t::VCLOCK);

  g_vclock_event_count++;
  g_prev_dwt_at_vclock_event = event.dwt_at_event;
  const uint32_t observed_dwt_at_vclock_event =
      (diag && diag->dwt_original_at_event != 0)
          ? diag->dwt_original_at_event
          : event.dwt_at_event;

  // Species-pure VCLOCK physical interval for PPS↔VCLOCK integrity reporting.
  //
  // event.dwt_at_event is the subscriber/public DWT species and may be
  // EMA-published.  The PPS interval comparator must instead use the observed
  // latency-corrected VCLOCK edge DWT on both endpoints, matching the raw_cycles
  // v_raw surface rather than the v_ema surface.
  if (g_prev_observed_dwt_at_vclock_event_valid) {
    g_observed_vclock_dwt_cycles_between_edges =
        observed_dwt_at_vclock_event - g_prev_observed_dwt_at_vclock_event;
    g_observed_vclock_dwt_cycles_between_edges_valid = true;
  } else {
    g_observed_vclock_dwt_cycles_between_edges = 0;
    g_observed_vclock_dwt_cycles_between_edges_valid = false;
  }
  g_prev_observed_dwt_at_vclock_event = observed_dwt_at_vclock_event;
  g_prev_observed_dwt_at_vclock_event_valid = true;

  clocks_apply_epoch_counter_edge(g_vclock_clock,
                                  g_vclock_measurement,
                                  event,
                                  diag,
                                  g_alpha_epoch_last_vclock_counter32,
                                  time_clock_id_t::VCLOCK);

}

static void ocxo1_callback(const interrupt_event_t& event,
                           const interrupt_capture_diag_t* diag,
                           void*) {
  apply_ocxo_event(g_ocxo1_clock, g_ocxo1_measurement,
                   g_ocxo1_interrupt_diag, event, diag,
                   g_alpha_epoch_last_ocxo1_counter32,
                   time_clock_id_t::OCXO1);
}

static void ocxo2_callback(const interrupt_event_t& event,
                           const interrupt_capture_diag_t* diag,
                           void*) {
  apply_ocxo_event(g_ocxo2_clock, g_ocxo2_measurement,
                   g_ocxo2_interrupt_diag, event, diag,
                   g_alpha_epoch_last_ocxo2_counter32,
                   time_clock_id_t::OCXO2);
}

// ============================================================================
// PPS selector callback: PPS/VCLOCK epoch anchor, bridge update, beta handoff
// ============================================================================

static void publish_pps_witness_diag(const pps_edge_snapshot_t& snap) {
  const uint32_t physical_pps_dwt = snap.physical_pps_dwt_normalized_at_edge;

  g_pps_dwt_at_edge = physical_pps_dwt;
  if (g_prev_pps_dwt_at_edge_valid) {
    g_pps_dwt_cycles_between_edges = physical_pps_dwt - g_prev_pps_dwt_at_edge;
    g_pps_dwt_cycles_between_edges_valid = true;

    // SmartZero needs a live PPS/GPIO-derived one-second DWT ruler before
    // any campaign epoch exists.  The physical PPS witness is therefore the
    // continuous calibration source; it is not zeroed and does not wait for
    // the PPS/VCLOCK bridge to become valid.
    g_dwt_cycles_between_pps_vclock = (uint32_t)g_pps_dwt_cycles_between_edges;
    g_dwt_calibration_valid = true;

    alpha_static_prediction_record_pps((uint32_t)g_pps_dwt_cycles_between_edges);
  } else {
    g_pps_dwt_cycles_between_edges = 0;
    g_pps_dwt_cycles_between_edges_valid = false;
  }

  // Phase is a physical diagnostic, not timing authority.  After VCLOCK
  // EMA authority, snap.dwt_at_edge is the authored/smoothed PPS_VCLOCK DWT.
  // Use the observed/original VCLOCK DWT from the interrupt diagnostic path
  // when available so phase remains a real PPS↔VCLOCK hardware measurement.
  const uint32_t observed_vclock_dwt_for_phase =
      g_prev_observed_dwt_at_vclock_event_valid
          ? g_prev_observed_dwt_at_vclock_event
          : snap.dwt_at_edge;

  if (snap.vclock_edge_authority.learned_phase_valid) {
    g_pps_vclock_phase_cycles =
        (int32_t)snap.vclock_edge_authority.learned_phase_cycles;
  } else {
    g_pps_vclock_phase_cycles =
        (int32_t)alpha_pps_vclock_phase_cycles_from_edges(
            physical_pps_dwt,
            observed_vclock_dwt_for_phase);
  }
  g_prev_pps_dwt_at_edge = physical_pps_dwt;
  g_prev_pps_dwt_at_edge_valid = true;

  g_pps_witness_diag.pps_edge_sequence = snap.sequence;
  g_pps_witness_diag.pps_edge_dwt_isr_entry_raw = physical_pps_dwt;
  g_pps_witness_diag.pps_edge_gnss_ns = snap.gnss_ns_at_edge;
  g_pps_witness_diag.pps_edge_minus_event_ns =
      (snap.gnss_ns_at_edge >= 0)
          ? (snap.gnss_ns_at_edge - (int64_t)g_gnss_ns_at_pps_vclock)
          : 0;

  const uint32_t dwt_cycles_from_vclock =
      g_prev_observed_dwt_at_vclock_event_valid
          ? (uint32_t)(observed_vclock_dwt_for_phase - physical_pps_dwt)
          : (uint32_t)(snap.dwt_at_edge - g_prev_dwt_at_vclock_event);
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

static bool alpha_sample_all_clocks_at_pps_vclock(const pps_edge_snapshot_t& snap,
                                                   uint64_t vclock_ns) {
  interrupt_epoch_capture_t cap{};
  const bool cap_available = interrupt_last_epoch_capture(&cap);

  g_alpha_runtime_epoch_capture_last_snap_sequence = snap.sequence;
  g_alpha_runtime_epoch_capture_last_cap_sequence = cap.sequence;
  g_alpha_runtime_epoch_capture_last_snap_counter32 = snap.counter32_at_edge;
  g_alpha_runtime_epoch_capture_last_cap_counter32 = cap.vclock_counter32;
  g_alpha_runtime_epoch_capture_last_snap_dwt = snap.dwt_at_edge;
  g_alpha_runtime_epoch_capture_last_cap_dwt = cap.vclock_dwt_at_edge;
  g_alpha_runtime_epoch_capture_last_cap_valid = cap.valid;
  g_alpha_runtime_epoch_capture_last_cap_vclock_valid = cap.vclock_capture_valid;
  g_alpha_runtime_epoch_capture_last_cap_all_lanes_valid = cap.all_lanes_capture_valid;

  if (!cap_available) {
    g_alpha_runtime_epoch_capture_missing_count++;
  } else if (cap.sequence != snap.sequence) {
    g_alpha_runtime_epoch_capture_sequence_mismatch_count++;
  }

  // Record this second's bridge anchor before sampling OCXO measured ledgers.
  // The physical PPS witness DWT is paired with its GNSS nanosecond coordinate.
  // vclock_ns is the canonical PPS/VCLOCK ledger value; physical PPS precedes
  // the canonical edge by the measured phase.  Recording before projection lets
  // the display/sample surface use the OCXO edge that this PPS anchor just
  // bracketed, without consuming that edge from the OCXO callback path.
  {
    const int32_t phase_cycles = g_pps_vclock_phase_cycles;
    const uint64_t phase_ns = phase_cycles > 0
        ? alpha_dwt_cycles_to_gnss_ns((uint32_t)phase_cycles)
        : 0ULL;
    const uint64_t pps_gnss_ns =
        vclock_ns > phase_ns ? vclock_ns - phase_ns : vclock_ns;
    alpha_bridge_anchor_record(snap.physical_pps_dwt_normalized_at_edge,
                               pps_gnss_ns);
  }

  const uint64_t ocxo1_physical_ns =
      alpha_ocxo_project_measured_ns_to_dwt_live(time_clock_id_t::OCXO1,
                                                 snap.dwt_at_edge);
  const uint64_t ocxo2_physical_ns =
      alpha_ocxo_project_measured_ns_to_dwt_live(time_clock_id_t::OCXO2,
                                                 snap.dwt_at_edge);
  const uint64_t ocxo1_measured_visible_ns =
      alpha_ocxo_visible_ns_from_physical(time_clock_id_t::OCXO1,
                                          ocxo1_physical_ns);
  const uint64_t ocxo2_measured_visible_ns =
      alpha_ocxo_visible_ns_from_physical(time_clock_id_t::OCXO2,
                                          ocxo2_physical_ns);

  uint64_t ocxo1_visible_ns = ocxo1_measured_visible_ns;
  uint64_t ocxo2_visible_ns = ocxo2_measured_visible_ns;

  // The projection guard compares the clock-domain PPS projection against the
  // conservative measured-visible surface.  That comparison is forensic only;
  // the public surface below is authored from the projection when available.
  alpha_ocxo_pps_projection_compute(time_clock_id_t::OCXO1,
                                    snap.sequence,
                                    snap.dwt_at_edge,
                                    vclock_ns,
                                    ocxo1_measured_visible_ns);
  alpha_ocxo_pps_projection_compute(time_clock_id_t::OCXO2,
                                    snap.sequence,
                                    snap.dwt_at_edge,
                                    vclock_ns,
                                    ocxo2_measured_visible_ns);

  uint64_t ocxo1_projected_visible_ns = 0;
  uint64_t ocxo2_projected_visible_ns = 0;
  const bool ocxo1_projected_visible_valid =
      alpha_ocxo_pps_projection_visible_ns(time_clock_id_t::OCXO1,
                                           &ocxo1_projected_visible_ns);
  const bool ocxo2_projected_visible_valid =
      alpha_ocxo_pps_projection_visible_ns(time_clock_id_t::OCXO2,
                                           &ocxo2_projected_visible_ns);

  if (ocxo1_projected_visible_valid) {
    ocxo1_visible_ns = ocxo1_projected_visible_ns;
  }
  if (ocxo2_projected_visible_valid) {
    ocxo2_visible_ns = ocxo2_projected_visible_ns;
  }

  // Capture the large campaign public-origin correction only from PPS-row
  // clock-domain projections, never from the conservative measured-edge
  // fallback.  The fallback is allowed to preserve display continuity while the
  // projection warms up, but it must not install the epoch's public branch.
  if (ocxo1_projected_visible_valid && ocxo2_projected_visible_valid) {
    alpha_ocxo_visible_origin_maybe_capture_public_origin(
        time_clock_id_t::OCXO1, snap.sequence, vclock_ns, ocxo1_visible_ns);
    alpha_ocxo_visible_origin_maybe_capture_public_origin(
        time_clock_id_t::OCXO2, snap.sequence, vclock_ns, ocxo2_visible_ns);
  }

  const uint64_t ocxo1_ns =
      alpha_ocxo_public_ns_from_visible_origin(time_clock_id_t::OCXO1,
                                               ocxo1_visible_ns);
  const uint64_t ocxo2_ns =
      alpha_ocxo_public_ns_from_visible_origin(time_clock_id_t::OCXO2,
                                               ocxo2_visible_ns);


  g_gnss_ns_at_pps_vclock = vclock_ns;
  g_ocxo1_physical_measured_gnss_ns_at_pps_vclock = ocxo1_physical_ns;
  g_ocxo2_physical_measured_gnss_ns_at_pps_vclock = ocxo2_physical_ns;
  g_ocxo1_measured_gnss_ns_at_pps_vclock = ocxo1_ns;
  g_ocxo2_measured_gnss_ns_at_pps_vclock = ocxo2_ns;

  // Compatibility mirrors for existing report/Beta surfaces.  The canonical
  // values are the explicit alpha-owned globals above.
  g_vclock_clock.ledger_ns_count_at_pps_vclock = vclock_ns;
  g_ocxo1_clock.ledger_ns_count_at_pps_vclock = ocxo1_ns;
  g_ocxo2_clock.ledger_ns_count_at_pps_vclock = ocxo2_ns;

  // Public VCLOCK/GNSS time-domain authority also lives here.  The selected
  // PPS/VCLOCK edge is the exact canonical public row: vclock_ns is authored by
  // Alpha as the tautological +1e9 ledger, not discovered from subscriber-event
  // DWT projection.  Updating process_time here makes report-time VCLOCK/GNSS
  // use the same PPS-row coordinate species as the OCXO public projections.
  (void)time_clock_update(time_clock_id_t::VCLOCK, snap.dwt_at_edge, vclock_ns);
  alpha_event_flow_note_time_update(time_clock_id_t::VCLOCK, vclock_ns);
  alpha_integrity_note_vclock_gnss_self_map(snap.sequence,
                                            snap.dwt_at_edge,
                                            vclock_ns);

  // Public OCXO time-domain authority lives here too, not in the OCXO callback.
  // The callback's measured-edge resolver intentionally returns the previous
  // bridge-resolved OCXO edge, because the current edge remains pending until
  // the next PPS/GNSS anchor brackets it.  At the PPS/VCLOCK row, however, the
  // bridge anchor has just been recorded and Alpha can project the OCXO public
  // clock-domain evidence to this canonical public DWT.
  // Feed process_time with that PPS-row visible coordinate so time.h clients
  // no longer inherit the one-edge/one-second measured-ledger lag.
  (void)time_clock_update(time_clock_id_t::OCXO1, snap.dwt_at_edge, ocxo1_ns);
  (void)time_clock_update(time_clock_id_t::OCXO2, snap.dwt_at_edge, ocxo2_ns);

  g_vclock_clock.phase_offset_ns = 0;
  g_ocxo1_clock.phase_offset_ns = (int64_t)vclock_ns - (int64_t)ocxo1_ns;
  g_ocxo2_clock.phase_offset_ns = (int64_t)vclock_ns - (int64_t)ocxo2_ns;

  g_vclock_clock.zero_established = true;
  g_ocxo1_clock.zero_established = true;
  g_ocxo2_clock.zero_established = true;
  return true;
}

static void update_pps_vclock_bridge_anchor(const pps_edge_snapshot_t& snap) {
  if (!epoch_ready() || !g_prev_pps_vclock_dwt_at_edge_valid) return;

  const uint32_t dwt_between_pps = snap.dwt_at_edge - g_prev_pps_vclock_dwt_at_edge;

  uint64_t counter_identity_ns = 0;
  const bool counter_identity_valid =
      alpha_ticks64_project_counter32(time_clock_id_t::VCLOCK,
                                      snap.counter32_at_edge,
                                      nullptr,
                                      &counter_identity_ns);

  // PPS/VCLOCK GNSS identity is tautological: each selected PPS/VCLOCK edge is
  // the next integer GNSS second.  The counter-derived value is retained as a
  // courtroom witness, but the canonical GNSS label is the exact +1e9 ledger.
  const uint64_t previous_vclock_ns = g_gnss_ns_at_pps_vclock;
  const uint64_t vclock_ns = previous_vclock_ns + NS_PER_SECOND_U64;

  const uint32_t effective_dwt_cycles_per_second =
      g_pps_dwt_cycles_between_edges_valid
          ? (uint32_t)g_pps_dwt_cycles_between_edges
          : dwt_between_pps;

  interrupt_integrity_note_vclock_pps_interval(
      snap.sequence,
      g_pps_dwt_cycles_between_edges_valid,
      (uint32_t)g_pps_dwt_cycles_between_edges,
      g_observed_vclock_dwt_cycles_between_edges_valid &&
          g_observed_vclock_dwt_cycles_between_edges != 0U,
      (uint32_t)g_observed_vclock_dwt_cycles_between_edges);

  const bool gnss_self_map_valid =
      g_prev_pps_vclock_dwt_at_edge_valid &&
      effective_dwt_cycles_per_second != 0U;
  const uint64_t mapped_gnss_ns = gnss_self_map_valid
      ? (previous_vclock_ns +
         ((uint64_t)dwt_between_pps * NS_PER_SECOND_U64 +
          (uint64_t)effective_dwt_cycles_per_second / 2ULL) /
             (uint64_t)effective_dwt_cycles_per_second)
      : 0ULL;
  const int64_t gnss_self_error_ns = gnss_self_map_valid
      ? ((mapped_gnss_ns >= vclock_ns)
             ? (int64_t)(mapped_gnss_ns - vclock_ns)
             : -(int64_t)(vclock_ns - mapped_gnss_ns))
      : 0LL;

  if (!alpha_sample_all_clocks_at_pps_vclock(snap, vclock_ns)) {
    return;
  }

  // Attach Alpha's campaign GNSS label and PPS/GPIO-derived static CPS to
  // the already-authored process_interrupt PPS_VCLOCK anchor. This makes the
  // interrupt bridge able to normalize later OCXO/TimePop event DWT facts
  // without letting process_interrupt invent campaign GNSS labels locally.
  interrupt_pps_vclock_label_anchor(snap.sequence,
                                    snap.counter32_at_edge,
                                    vclock_ns,
                                    effective_dwt_cycles_per_second);

  alpha_pps_vclock_edge_forensics_publish(snap,
                                           gnss_self_map_valid,
                                           vclock_ns,
                                           mapped_gnss_ns,
                                           gnss_self_error_ns,
                                           dwt_between_pps,
                                           effective_dwt_cycles_per_second,
                                           counter_identity_valid,
                                           counter_identity_ns);

  g_dwt_cycles_between_pps_vclock = effective_dwt_cycles_per_second;
  g_dwt_cycle_count_total += (uint64_t)effective_dwt_cycles_per_second;
  // VCLOCK prediction is recorded from the VCLOCK event rail itself in
  // clocks_apply_epoch_counter_edge().  Do not feed it from the smoother PPS
  // witness interval here; PPS now owns its own first-class prediction lane.
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
  // Startup epoch install is now SmartZero-gated.  Explicit START/ZERO
  // requests are finished by Beta once the same SmartZero proof surface is
  // complete; startup uses it only when no command-owned acquisition is active.
  bool installed_epoch = false;
  if (!g_epoch_initialized && !request_start && !request_zero) {
    if (!interrupt_smartzero_running() && !interrupt_smartzero_complete()) {
      (void)clocks_alpha_begin_smartzero_epoch("startup");
    }
    if (interrupt_smartzero_complete()) {
      installed_epoch = clocks_alpha_zero_from_smartzero("startup");
    }
  }

  publish_pps_witness_diag(snap);
  if (!installed_epoch) {
    update_pps_vclock_bridge_anchor(snap);
  }

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
  timebase_init();

  // Startup epoch is installed locally by alpha from the first valid
  // process_interrupt PPS/VCLOCK epoch capture packet.  Request a PPS
  // rebootstrap so process_interrupt authors the first canonical PPS_VCLOCK
  // edge and its epoch-ready capture packet.
  interrupt_request_pps_rebootstrap();

  g_ad5693r_init_ok = OCXO_DAC_HARDWARE_TOUCH_ENABLED
      ? ad5693r_init()
      : false;

  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);
  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);

  if (OCXO_DAC_HARDWARE_TOUCH_ENABLED) {
    (void)ocxo_dac_set(ocxo1_dac, (double)AD5693R_DAC_DEFAULT);
    (void)ocxo_dac_set(ocxo2_dac, (double)AD5693R_DAC_DEFAULT);
  } else {
    // Memory-only startup intent.  Do not touch DAC hardware and do not
    // report an I/O fault merely because this diagnostic build refuses all
    // DAC transactions.
    ocxo1_dac.dac_fractional = (double)AD5693R_DAC_DEFAULT;
    ocxo2_dac.dac_fractional = (double)AD5693R_DAC_DEFAULT;
    ocxo1_dac.io_last_write_ok = true;
    ocxo2_dac.io_last_write_ok = true;
    ocxo1_dac.io_last_failure_stage = 0;
    ocxo2_dac.io_last_failure_stage = 0;
  }
  pinMode(GNSS_LOCK_PIN, INPUT);

  subscribe_clock(interrupt_subscriber_kind_t::VCLOCK, vclock_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO1, ocxo1_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO2, ocxo2_callback);

  interrupt_pps_edge_register_dispatch(pps_selector_callback);
  (void)clocks_alpha_begin_smartzero_epoch("startup");
}