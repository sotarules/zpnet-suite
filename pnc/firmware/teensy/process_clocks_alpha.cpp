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
// OCXO events are now delivered as rollover-only one-second events whose DWT
// coordinate is EMA-predicted by process_interrupt from completed one-second
// intervals.  Alpha still accepts the older sample-phase diagnostic surface for
// compatibility, but the normal EMA path arrives already at the logical edge.
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

// VCLOCK event counter — increments on every vclock_callback invocation
// AFTER epoch install.  Read by pps_selector_callback to cross-check that no
// VCLOCK event slipped in between the GPIO ISR and the foreground
// pps_selector_callback.
static volatile uint32_t g_vclock_event_count = 0;

// ============================================================================
// OCXO DAC defaults
// ============================================================================

static constexpr uint32_t OCXO_DAC_Q16_SHIFT = 16U;
static constexpr uint32_t OCXO_DAC_Q16_ONE   = (1UL << OCXO_DAC_Q16_SHIFT);
static constexpr uint32_t OCXO_DAC_Q16_MAX   = ((uint32_t)65535U << OCXO_DAC_Q16_SHIFT);

static uint32_t ocxo_dac_value_to_q16(double value) {
  if (value < 0.0) value = 0.0;
  if (value > 65535.0) value = 65535.0;
  const double scaled = value * (double)OCXO_DAC_Q16_ONE;
  const double rounded = scaled + 0.5;
  if (rounded >= (double)OCXO_DAC_Q16_MAX) return OCXO_DAC_Q16_MAX;
  return (uint32_t)rounded;
}

static double ocxo_dac_q16_to_value(uint32_t q16) {
  return (double)q16 / (double)OCXO_DAC_Q16_ONE;
}

static ocxo_dac_state_t make_default_ocxo_dac_state() {
  ocxo_dac_state_t s = {};
  s.dac_fractional = (double)AD5693R_DAC_DEFAULT;
  s.dac_hw_code = AD5693R_DAC_DEFAULT;
  s.dac_min = 0;
  s.dac_max = 65535;
  s.dac_desired_q16 = ((uint32_t)AD5693R_DAC_DEFAULT << OCXO_DAC_Q16_SHIFT);
  s.dither_enabled = true;
  s.dither_rate_hz = 1000U;
  s.dither_period_ns = 1000000U;
  s.dither_effective_window_ticks = 1000U;
  s.dither_low_code = AD5693R_DAC_DEFAULT;
  s.dither_high_code = AD5693R_DAC_DEFAULT;
  s.dither_last_selected_hw_code = AD5693R_DAC_DEFAULT;
  s.dither_last_window_low_code = AD5693R_DAC_DEFAULT;
  s.dither_last_window_high_code = AD5693R_DAC_DEFAULT;
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

void ocxo_dac_dither_reset(ocxo_dac_state_t& s) {
  s.dither_rate_hz = 1000U;
  s.dither_period_ns = 1000000U;
  s.dither_effective_window_ticks = 1000U;
  s.dither_accumulator_q16 = 0;
  s.dither_fraction_q16 = s.dac_desired_q16 & 0xFFFFUL;
  s.dither_low_code = (uint16_t)(s.dac_desired_q16 >> OCXO_DAC_Q16_SHIFT);
  s.dither_high_code = (s.dither_low_code >= 65535U || s.dither_fraction_q16 == 0)
      ? s.dither_low_code
      : (uint16_t)(s.dither_low_code + 1U);
  s.dither_last_selected_hw_code = s.dac_hw_code;
  s.dither_window_tick_count = 0;
  s.dither_window_low_count = 0;
  s.dither_window_high_count = 0;
  s.syncdac_frame_tick = 0;
  s.syncdac_cell_index = 0;
  s.syncdac_cell_tick = 0;
  s.syncdac_high_ms_per_second = 0;
  s.syncdac_low_ms_per_second = 1000U;
  s.syncdac_high_ms_this_cell = 0;
  s.syncdac_low_ms_this_cell = 100U;
  s.syncdac_write_attempts_this_frame = 0;
  s.syncdac_write_successes_this_frame = 0;
  s.syncdac_write_failures_this_frame = 0;
  s.syncdac_write_suppressed_this_frame = 0;
}

bool ocxo_dac_set_desired(ocxo_dac_state_t& s, double value) {
  const uint32_t old_q16 = s.dac_desired_q16;
  const uint16_t old_low = (uint16_t)(old_q16 >> OCXO_DAC_Q16_SHIFT);

  const uint32_t q16 = ocxo_dac_value_to_q16(value);
  s.dac_desired_q16 = q16;
  s.dac_fractional = ocxo_dac_q16_to_value(q16);

  const uint16_t new_low = (uint16_t)(q16 >> OCXO_DAC_Q16_SHIFT);
  if (new_low != old_low) {
    s.dither_accumulator_q16 = 0;
  }
  return true;
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
  if ((uint32_t)hw_code < s.dac_min) hw_code = (uint16_t)s.dac_min;
  if ((uint32_t)hw_code > s.dac_max) hw_code = (uint16_t)s.dac_max;
  s.io_last_attempted_hw_code = hw_code;

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

  uint64_t counter_nominal_ns_between_edges = 0;
  uint64_t bridge_gnss_ns_between_edges = 0;
  int64_t  bridge_residual_ns = 0;
  bool     bridge_interval_valid = false;

  uint64_t ns_between_edges = 0;
  uint32_t dwt_cycles_between_edges = 0;

  bool     dwt_synthetic = false;
  bool     dwt_repair_candidate = false;
  uint32_t dwt_original_at_event = 0;
  uint32_t dwt_predicted_at_event = 0;
  uint32_t dwt_used_at_event = 0;
  uint32_t dwt_isr_entry_raw = 0;
  int32_t  dwt_synthetic_error_cycles = 0;
  uint32_t dwt_synthetic_threshold_cycles = 0;

  interrupt_qtimer_capture_diag_t qtimer_capture{};

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
    f->last_callback_sample_phase_valid = diag->ocxo_sample_phase_valid;
    f->last_callback_sample_phase_ticks = diag->ocxo_sample_phase_ticks;
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

static void alpha_event_flow_note_phase(time_clock_id_t clock,
                                        uint32_t phase_ticks,
                                        uint32_t phase_cycles,
                                        const interrupt_event_t& applied_event) {
  alpha_event_flow_store_t* f = alpha_event_flow_store(clock);
  if (!f) return;
  f->apply_phase_projected_count++;
  f->last_stage = ALPHA_FLOW_STAGE_PHASE_PROJECTED;
  f->last_applied_phase_ticks = phase_ticks;
  f->last_applied_phase_cycles = phase_cycles;
  f->last_applied_dwt_at_event = applied_event.dwt_at_event;
  f->last_applied_counter32_at_event = applied_event.counter32_at_event;
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
struct alpha_measured_ns_clock_t {
  bool     initialized = false;
  uint64_t ns_at_edge = 0;
  uint32_t dwt_at_edge = 0;
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
//   OCXO1  — OCXO1 EMA-predicted edge-to-edge DWT interval
//   OCXO2  — OCXO2 EMA-predicted edge-to-edge DWT interval

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
// boundary edge facts and, once per PPS/VCLOCK row, computes what the OCXO
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

static alpha_ocxo_edge_history_t g_ocxo1_edge_history = {};
static alpha_ocxo_edge_history_t g_ocxo2_edge_history = {};
static alpha_ocxo_pps_projection_store_t g_ocxo1_pps_projection = {};
static alpha_ocxo_pps_projection_store_t g_ocxo2_pps_projection = {};

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
  local.projected_minus_existing_pps_ns =
      alpha_signed_delta_u64(projected_ns, existing_pps_ns);
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

  // Prefer an actual bracketing pair when the latest OCXO event has already
  // arrived after this PPS edge.  Otherwise use the latest edge plus the
  // static next-edge DWT projection.
  if (h->previous_valid && h->previous.valid && h->current.valid) {
    const uint32_t actual_interval =
        h->current.dwt_at_edge - h->previous.dwt_at_edge;
    const uint32_t target_delta = pps_dwt_at_edge - h->previous.dwt_at_edge;
    if (actual_interval != 0 && target_delta <= actual_interval) {
      if (alpha_ocxo_pps_projection_build(
              clock, ALPHA_OCXO_PPS_PROJECTION_SOURCE_ACTUAL_BRACKET,
              h->previous, h->current, actual_interval, target_delta,
              pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
              latest_actual_interval_cycles, completed_interval_count,
              static_prediction_valid,
              0U, target_delta, 0U)) {
        return;
      }
    }
  }

  if (latest_actual_interval_cycles == 0) {
    alpha_ocxo_pps_projection_set_invalid(
        clock, ALPHA_OCXO_PPS_PROJECTION_INVALID_NO_INTERVAL,
        pps_sequence, pps_dwt_at_edge, pps_vclock_ns, existing_pps_ns,
        &h->current, h->previous_valid ? &h->previous : nullptr,
        latest_actual_interval_cycles, completed_interval_count,
        static_prediction_valid,
        pps_dwt_at_edge - h->current.dwt_at_edge, 0U, 0U);
    return;
  }

  const uint32_t raw_target_delta = pps_dwt_at_edge - h->current.dwt_at_edge;
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
    const uint32_t unresolved_overrun = target_delta - latest_actual_interval_cycles;
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
  const uint32_t dwt_per_second = dwt_effective_cycles_per_pps_vclock_second();
  if (dwt_per_second == 0) {
    clocks_watchdog_anomaly("alpha_dwt_cps_zero", dwt_cycles, 0, 0, 0);
    return 0;
  }

  return ((uint64_t)dwt_cycles * NS_PER_SECOND_U64 +
          (uint64_t)dwt_per_second / 2ULL) /
         (uint64_t)dwt_per_second;
}

static uint64_t alpha_ocxo_seed_ns_at_first_edge(uint32_t raw_edge_dwt) {
  const uint32_t epoch_dwt = clocks_alpha_epoch_last_dwt_at_edge();
  if (epoch_dwt == 0) return 0;
  return alpha_dwt_cycles_to_gnss_ns(raw_edge_dwt - epoch_dwt);
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

  if (!m->initialized) {
    m->initialized = true;
    m->dwt_at_edge = raw_edge_dwt;
    m->ns_at_edge = alpha_ocxo_seed_ns_at_first_edge(raw_edge_dwt);
    if (out_dwt_cycles) *out_dwt_cycles = 0;
    if (out_real_interval_ns) *out_real_interval_ns = 0;
    if (out_residual_fast_ns) *out_residual_fast_ns = 0;
    return m->ns_at_edge;
  }

  const uint32_t dwt_cycles = raw_edge_dwt - m->dwt_at_edge;
  const uint64_t real_interval_ns = alpha_dwt_cycles_to_gnss_ns(dwt_cycles);
  const int64_t residual_fast_ns =
      (int64_t)NS_PER_SECOND_U64 - (int64_t)real_interval_ns;

  // The measured OCXO ledger is the accumulated GNSS duration between OCXO
  // one-second edges.  This is intentionally not counter ticks × 100 ns and
  // not an ideal +1e9-per-edge nominal ledger.
  m->ns_at_edge += real_interval_ns;
  m->dwt_at_edge = raw_edge_dwt;

  if (out_dwt_cycles) *out_dwt_cycles = dwt_cycles;
  if (out_real_interval_ns) *out_real_interval_ns = real_interval_ns;
  if (out_residual_fast_ns) *out_residual_fast_ns = residual_fast_ns;
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

  const uint32_t delta_cycles = target_dwt - m->dwt_at_edge;
  return m->ns_at_edge + alpha_dwt_cycles_to_gnss_ns(delta_cycles);
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
  s.counter_nominal_ns_between_edges = 0;
  s.bridge_gnss_ns_between_edges = 0;
  s.bridge_residual_ns = 0;
  s.bridge_interval_valid = false;
  s.ns_between_edges = 0;
  s.dwt_cycles_between_edges = 0;
  s.dwt_synthetic = false;
  s.dwt_repair_candidate = false;
  s.dwt_original_at_event = 0;
  s.dwt_predicted_at_event = 0;
  s.dwt_used_at_event = 0;
  s.dwt_isr_entry_raw = 0;
  s.dwt_synthetic_error_cycles = 0;
  s.dwt_synthetic_threshold_cycles = 0;
  s.qtimer_capture = interrupt_qtimer_capture_diag_t{};
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
                                    uint64_t ns_now) {
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
  s->counter_nominal_ns_between_edges = counter_nominal_ns_between_edges;
  s->bridge_gnss_ns_between_edges = bridge_gnss_ns_between_edges;
  s->bridge_residual_ns = bridge_residual_ns;
  s->bridge_interval_valid = bridge_interval_valid;
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
    s->dwt_synthetic_error_cycles = diag->dwt_synthetic_error_cycles;
    s->dwt_synthetic_threshold_cycles = diag->dwt_synthetic_threshold_cycles;
    s->qtimer_capture = diag->qtimer_capture;
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
    s->dwt_synthetic_error_cycles = 0;
    s->dwt_synthetic_threshold_cycles = 0;
    s->qtimer_capture = interrupt_qtimer_capture_diag_t{};
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
    out->counter_nominal_ns_between_edges = s->counter_nominal_ns_between_edges;
    out->bridge_gnss_ns_between_edges = s->bridge_gnss_ns_between_edges;
    out->bridge_residual_ns = s->bridge_residual_ns;
    out->bridge_interval_valid = s->bridge_interval_valid;
    out->ns_between_edges = s->ns_between_edges;
    out->dwt_cycles_between_edges = s->dwt_cycles_between_edges;
    out->dwt_synthetic = s->dwt_synthetic;
    out->dwt_repair_candidate = s->dwt_repair_candidate;
    out->dwt_original_at_event = s->dwt_original_at_event;
    out->dwt_predicted_at_event = s->dwt_predicted_at_event;
    out->dwt_used_at_event = s->dwt_used_at_event;
    out->dwt_isr_entry_raw = s->dwt_isr_entry_raw;
    out->dwt_synthetic_error_cycles = s->dwt_synthetic_error_cycles;
    out->dwt_synthetic_threshold_cycles = s->dwt_synthetic_threshold_cycles;
    out->qtimer_capture = s->qtimer_capture;
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

  alpha_ticks64_install_zero(g_vclock_ticks64, vclock.anchor_counter32);
  alpha_ticks64_install_zero(g_ocxo1_ticks64, ocxo1.anchor_counter32);
  alpha_ticks64_install_zero(g_ocxo2_ticks64, ocxo2.anchor_counter32);

  // OCXO measured ledgers have their own mathematically-qualified zero
  // anchors. The first post-zero OCXO edge measures from that OCXO anchor,
  // not from the VCLOCK anchor.
  g_ocxo1_measured_ns.initialized = true;
  g_ocxo1_measured_ns.ns_at_edge = 0;
  g_ocxo1_measured_ns.dwt_at_edge = ocxo1.anchor_dwt;
  g_ocxo2_measured_ns.initialized = true;
  g_ocxo2_measured_ns.ns_at_edge = 0;
  g_ocxo2_measured_ns.dwt_at_edge = ocxo2.anchor_dwt;

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
  time_clock_epoch_reset(time_clock_id_t::OCXO1,  ocxo1.anchor_dwt, 0);
  time_clock_epoch_reset(time_clock_id_t::OCXO2,  ocxo2.anchor_dwt, 0);

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

static uint32_t alpha_dwt_cycles_for_10mhz_ticks(uint32_t ticks,
                                                 uint32_t cycles_per_second) {
  if (ticks == 0 || cycles_per_second == 0) return 0;
  return (uint32_t)(((uint64_t)cycles_per_second * (uint64_t)ticks +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static uint32_t alpha_phase_backprojection_cps(const clock_measurement_t& meas) {
  if (meas.dwt_cycles_between_edges != 0) {
    return meas.dwt_cycles_between_edges;
  }
  const uint32_t effective = dwt_effective_cycles_per_pps_vclock_second();
  return effective ? effective : (uint32_t)DWT_EXPECTED_PER_PPS;
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
  const uint32_t sample_physics_dwt =
      alpha_physics_dwt_at_event(time_clock, event, diag);
  uint32_t phase_ticks = 0;
  uint32_t phase_cycles = 0;

  if (is_ocxo && diag && diag->ocxo_sample_phase_valid) {
    phase_ticks = diag->ocxo_sample_phase_ticks;
    phase_cycles = alpha_dwt_cycles_for_10mhz_ticks(
        phase_ticks, alpha_phase_backprojection_cps(meas));
    applied_event.dwt_at_event = sample_physics_dwt - phase_cycles;
    applied_event.counter32_at_event = event.counter32_at_event - phase_ticks;
    alpha_event_flow_note_phase(time_clock, phase_ticks, phase_cycles, applied_event);
  } else {
    applied_event.dwt_at_event = sample_physics_dwt;
  }

  interrupt_capture_diag_t local_diag{};
  const interrupt_capture_diag_t* applied_diag = diag;
  if (diag) {
    local_diag = *diag;
    if (is_ocxo) {
      local_diag.ocxo_sample_dwt_at_event =
          diag->ocxo_sample_dwt_at_event ? diag->ocxo_sample_dwt_at_event : event.dwt_at_event;
      local_diag.ocxo_sample_counter32_at_event =
          diag->ocxo_sample_counter32_at_event ? diag->ocxo_sample_counter32_at_event : event.counter32_at_event;
      local_diag.ocxo_boundary_dwt_at_event = applied_event.dwt_at_event;
      local_diag.ocxo_boundary_counter32_at_event = applied_event.counter32_at_event;
      local_diag.ocxo_boundary_correction_cycles = (int32_t)phase_cycles;
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

  uint64_t ns_now = counter_ns_now;
  uint32_t dwt_cycles_between_edges = had_previous
      ? (applied_event.dwt_at_event - meas.prev_dwt_at_edge)
      : 0U;
  uint64_t gnss_ns_between_edges = 0;
  int64_t second_residual_ns = 0;
  int64_t window_error_ns = 0;

  if (is_ocxo) {
    uint32_t measured_dwt_cycles = 0;
    uint64_t real_interval_ns = 0;
    int64_t residual_fast_ns = 0;
    ns_now = alpha_ocxo_apply_measured_second(time_clock,
                                              applied_event.dwt_at_event,
                                              &measured_dwt_cycles,
                                              &real_interval_ns,
                                              &residual_fast_ns);
    alpha_event_flow_note_measured(time_clock, measured_dwt_cycles,
                                   real_interval_ns, residual_fast_ns, ns_now);

    const bool sample_gnss_available = (event.gnss_ns_at_event != 0);
    const uint64_t sample_gnss_ns_at_event = sample_gnss_available
        ? event.gnss_ns_at_event
        : 0ULL;
    uint64_t boundary_gnss_ns_at_edge = 0;
    bool boundary_gnss_available = false;
    if (sample_gnss_available) {
      const uint32_t dwt_per_second =
          dwt_effective_cycles_per_pps_vclock_second();
      if (dwt_per_second != 0) {
        const uint64_t phase_ns =
            ((uint64_t)phase_cycles * NS_PER_SECOND_U64 +
             (uint64_t)dwt_per_second / 2ULL) /
            (uint64_t)dwt_per_second;
        if (sample_gnss_ns_at_event >= phase_ns) {
          boundary_gnss_ns_at_edge = sample_gnss_ns_at_event - phase_ns;
          boundary_gnss_available = true;
        }
      }
    }

    alpha_ocxo_edge_history_record(time_clock,
                                   applied_event.dwt_at_event,
                                   applied_event.counter32_at_event,
                                   counter_ns_now,
                                   ns_now,
                                   sample_gnss_available,
                                   sample_gnss_ns_at_event,
                                   boundary_gnss_available,
                                   boundary_gnss_ns_at_edge,
                                   phase_ticks,
                                   phase_cycles);

    dwt_cycles_between_edges = measured_dwt_cycles;
    gnss_ns_between_edges = real_interval_ns;
    second_residual_ns = residual_fast_ns;
    window_error_ns = -residual_fast_ns;
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

  (void)time_clock_update(time_clock, applied_event.dwt_at_event, ns_now);
  alpha_event_flow_note_time_update(time_clock, ns_now);

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
                          ns_now);
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
  g_prev_observed_dwt_at_vclock_event =
      (diag && diag->dwt_original_at_event != 0)
          ? diag->dwt_original_at_event
          : event.dwt_at_event;
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

  g_pps_vclock_phase_cycles =
      (int32_t)alpha_pps_vclock_phase_cycles_from_edges(
          physical_pps_dwt,
          observed_vclock_dwt_for_phase);
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

  const uint64_t ocxo1_ns =
      alpha_ocxo_project_measured_ns_to_dwt(time_clock_id_t::OCXO1,
                                            snap.dwt_at_edge);
  const uint64_t ocxo2_ns =
      alpha_ocxo_project_measured_ns_to_dwt(time_clock_id_t::OCXO2,
                                            snap.dwt_at_edge);

  alpha_ocxo_pps_projection_compute(time_clock_id_t::OCXO1,
                                    snap.sequence,
                                    snap.dwt_at_edge,
                                    vclock_ns,
                                    ocxo1_ns);
  alpha_ocxo_pps_projection_compute(time_clock_id_t::OCXO2,
                                    snap.sequence,
                                    snap.dwt_at_edge,
                                    vclock_ns,
                                    ocxo2_ns);

  g_gnss_ns_at_pps_vclock = vclock_ns;
  g_ocxo1_measured_gnss_ns_at_pps_vclock = ocxo1_ns;
  g_ocxo2_measured_gnss_ns_at_pps_vclock = ocxo2_ns;

  // Compatibility mirrors for existing report/Beta surfaces.  The canonical
  // values are the explicit alpha-owned globals above.
  g_vclock_clock.ledger_ns_count_at_pps_vclock = vclock_ns;
  g_ocxo1_clock.ledger_ns_count_at_pps_vclock = ocxo1_ns;
  g_ocxo2_clock.ledger_ns_count_at_pps_vclock = ocxo2_ns;

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

  uint64_t vclock_ns = 0;
  if (!alpha_ticks64_project_counter32(time_clock_id_t::VCLOCK,
                                       snap.counter32_at_edge,
                                       nullptr,
                                       &vclock_ns)) {
    return;
  }
  if (!alpha_sample_all_clocks_at_pps_vclock(snap, vclock_ns)) {
    return;
  }

  const uint32_t effective_dwt_cycles_per_second =
      g_pps_dwt_cycles_between_edges_valid
          ? (uint32_t)g_pps_dwt_cycles_between_edges
          : dwt_between_pps;

  // Attach Alpha's campaign GNSS label and PPS/GPIO-derived static CPS to
  // the already-authored process_interrupt PPS_VCLOCK anchor. This makes the
  // interrupt bridge able to normalize later OCXO/TimePop event DWT facts
  // without letting process_interrupt invent campaign GNSS labels locally.
  interrupt_pps_vclock_label_anchor(snap.sequence,
                                    snap.counter32_at_edge,
                                    vclock_ns,
                                    effective_dwt_cycles_per_second);

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

  g_ad5693r_init_ok = ad5693r_init();

  ocxo_dac_io_reset(ocxo1_dac);
  ocxo_dac_io_reset(ocxo2_dac);
  ocxo_dac_predictor_reset(ocxo1_dac);
  ocxo_dac_predictor_reset(ocxo2_dac);

  (void)ocxo_dac_set(ocxo1_dac, (double)AD5693R_DAC_DEFAULT);
  (void)ocxo_dac_set(ocxo2_dac, (double)AD5693R_DAC_DEFAULT);
  ocxo_dac_dither_reset(ocxo1_dac);
  ocxo_dac_dither_reset(ocxo2_dac);

  pinMode(GNSS_LOCK_PIN, INPUT);

  subscribe_clock(interrupt_subscriber_kind_t::VCLOCK, vclock_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO1, ocxo1_callback);
  subscribe_clock(interrupt_subscriber_kind_t::OCXO2, ocxo2_callback);

  interrupt_pps_edge_register_dispatch(pps_selector_callback);
  clocks_dac_dither_begin();
  (void)clocks_alpha_begin_smartzero_epoch("startup");
}