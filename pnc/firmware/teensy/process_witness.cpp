// ============================================================================
// process_witness.cpp
// ============================================================================
//
// process_witness owns the local hardware latency witnesses:
//
//   • Stimulus source pin  — foreground TimePop callback drives HIGH/LOW.
//   • GPIO sink            — GPIO interrupt sees the source rising edge.
//   • QTimer2 CH0 sink     — external-count compare sees stimulus edges.
//
// This version preserves the old process_interrupt witness_latency
// QTimer lifecycle while using a deterministic four-edge measurement cycle:
//
//   • QTimer2 CH0 is initialized once and left live.
//   • QTimer compare is armed as COMP1 = current CNTR + 1.
//   • The QTimer ISR clears TCF1, records the sample, then immediately
//     re-arms COMP1 = current CNTR + 1.
//   • No per-slot QTimer teardown.
//   • No SCTRL.TCFIE path.
//   • GPIO and QTimer measurements occur in separated windows.
//
// The scheduler is a single 1 Hz supervisor that arms four one-shot edges
// at 200, 400, 600, and 800 ms.  This avoids top-of-second PPS activity
// and makes the stimulus visually regular.
//
// Current command surface:
//   START         — enable full witness instrumentation/circus.
//   STOP          — cancel witness timers/hooks and return to quiet state.
//   REPORT        — lifecycle/hardware/timer state.
//   EDGE          — PPS/PPS_VCLOCK heartbeat + last edge facts.
//   BRIDGE        — DWT/GNSS bridge check using interrupt-owned QTimer1 CH1.
//   GPIO_DELAY    — simple digitalWriteFast HIGH stimulus cost.
//   QTIMER_READ   — GPIO ISR-to-VCLOCK counter observation delay/cost.
//   DWT_READ      — consecutive DWT read-cost measurement.
//   ENTRY_LATENCY — spin-shadow ISR entry latency for PPS and QTimer.
//   PPS_PHASE     — PPS notification vs selected VCLOCK edge phase.
//   ROUND_TRIP    — source, GPIO sink, and QTimer sink latency stats.
//
// Test cadence:
//   • 200 ms: GPIO HIGH
//   • 400 ms: GPIO LOW
//   • 600 ms: QTIMER HIGH
//   • 800 ms: QTIMER LOW
//
// ============================================================================

#include "process_witness.h"

#include "process_interrupt.h"

#include "config.h"
#include "process.h"
#include "payload.h"
#include "timepop.h"
#include "time.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>

// ============================================================================
// Hardware constants
// ============================================================================

static constexpr int WITNESS_STIMULUS_PIN = 24;
static constexpr int WITNESS_GPIO_PIN     = 26;
static constexpr int WITNESS_QTIMER_PIN   = 13;   // QTimer2 TIMER0 input

static constexpr uint64_t GPIO_HIGH_OFFSET_NS   = 200000000ULL;
static constexpr uint64_t GPIO_LOW_OFFSET_NS    = 400000000ULL;
static constexpr uint64_t QTIMER_HIGH_OFFSET_NS = 600000000ULL;
static constexpr uint64_t QTIMER_LOW_OFFSET_NS  = 800000000ULL;
static constexpr uint64_t WITNESS_CYCLE_PERIOD_NS = 1000000000ULL;

static constexpr const char* GPIO_HIGH_NAME   = "WITNESS_GPIO_HIGH";
static constexpr const char* GPIO_LOW_NAME    = "WITNESS_GPIO_LOW";
static constexpr const char* QTIMER_HIGH_NAME = "WITNESS_QTIMER_HIGH";
static constexpr const char* QTIMER_LOW_NAME  = "WITNESS_QTIMER_LOW";
static constexpr const char* WITNESS_CYCLE_NAME = "WITNESS_CYCLE";

static constexpr uint64_t ENTRY_LATENCY_LEAD_NS = 5000ULL;
static constexpr uint32_t ENTRY_LATENCY_TIMEOUT_CYCLES = 100000U;  // ~99 us @ 1.008 GHz
static constexpr const char* ENTRY_PPS_SPIN_NAME = "ENTRY_LATENCY_PPS_SPIN";
static constexpr const char* ENTRY_QTIMER_SPIN_NAME = "ENTRY_LATENCY_QTIMER_SPIN";

static constexpr uint32_t VCLOCK_TICKS_PER_SECOND_U32 = 10000000U;

// Best-guess physical pin-arrival latency model for PPS_PHASE.
// ROUND_TRIP totals include software stimulus launch cost; subtract it when
// estimating the delay from a physical pin transition to ISR entry.
static constexpr uint32_t PPS_PIN_TO_ISR_CYCLES =
    GPIO_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY;
static constexpr uint32_t QTIMER_PIN_TO_ISR_CYCLES =
    QTIMER_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY;

// ============================================================================
// Welford
// ============================================================================

struct welford_t {
  uint64_t n = 0;
  double   mean = 0.0;
  double   m2 = 0.0;
  int32_t  min_val = 0;
  int32_t  max_val = 0;
};

static inline void welford_reset(welford_t& w) {
  w.n = 0;
  w.mean = 0.0;
  w.m2 = 0.0;
  w.min_val = 0;
  w.max_val = 0;
}

static inline void welford_update(welford_t& w, int32_t sample) {
  w.n++;
  if (w.n == 1) {
    w.min_val = sample;
    w.max_val = sample;
  } else {
    if (sample < w.min_val) w.min_val = sample;
    if (sample > w.max_val) w.max_val = sample;
  }

  const double delta = (double)sample - w.mean;
  w.mean += delta / (double)w.n;
  const double delta2 = (double)sample - w.mean;
  w.m2 += delta * delta2;
}

static inline double welford_stddev(const welford_t& w) {
  return (w.n < 2) ? 0.0 : sqrt(w.m2 / (double)(w.n - 1));
}

// 1 cycle = 125/126 ns at 1008 MHz.
static inline double cycles_to_ns(double cycles) {
  return cycles * (125.0 / 126.0);
}

// ============================================================================
// Runtime state
// ============================================================================

static volatile bool g_witness_hw_ready = false;
static volatile bool g_witness_runtime_ready = false;
static volatile bool g_witness_irqs_enabled = false;
static volatile bool g_witness_running = false;
static volatile bool g_witness_schedule_armed = false;
static volatile uint32_t g_witness_start_count = 0;
static volatile uint32_t g_witness_stop_count = 0;
static volatile uint32_t g_witness_last_start_dwt = 0;
static volatile uint32_t g_witness_last_stop_dwt = 0;
static volatile uint32_t g_witness_cycle_count = 0;
static volatile uint32_t g_witness_cycle_reschedules = 0;

enum class witness_window_t : uint8_t { NONE = 0, GPIO, QTIMER };
static volatile witness_window_t g_active_window = witness_window_t::NONE;

static volatile bool g_source_high = false;

static volatile uint32_t g_source_emits = 0;
static volatile uint32_t g_source_lows = 0;
static volatile uint32_t g_source_dwt_at_emit = 0;
static volatile uint32_t g_source_dwt_before = 0;
static volatile uint32_t g_source_dwt_after = 0;
static volatile uint32_t g_source_stim_cycles = 0;

static volatile uint32_t g_gpio_hits = 0;
static volatile uint32_t g_gpio_dwt_at_isr = 0;
static volatile uint32_t g_gpio_delta_cycles = 0;
static volatile uint32_t g_gpio_wrong_window = 0;

static volatile uint32_t g_qtimer_hits = 0;
static volatile uint32_t g_qtimer_irq_entries = 0;
static volatile uint32_t g_qtimer_no_flag = 0;
static volatile uint32_t g_qtimer_outside_window = 0;
static volatile uint32_t g_qtimer_arms = 0;
static volatile uint32_t g_qtimer_dwt_at_isr = 0;
static volatile uint32_t g_qtimer_delta_cycles = 0;
static volatile uint16_t g_qtimer_compare_target = 0;

static welford_t g_source_welford = {};
static welford_t g_gpio_welford = {};
static welford_t g_qtimer_welford = {};

// ============================================================================
// QTIMER_READ — cost of obtaining a VCLOCK counter observation inside GPIO ISR
// ============================================================================
//
// This report measures the timing separation between a GPIO ISR's first-
// instruction DWT capture and an immediate process_interrupt-owned ambient
// VCLOCK counter observation.  It is a local, repeatable analog of the PPS
// GPIO-path question: a counter read performed after ISR entry is not the
// same instant as the edge capture, so the delay and the read cost must be
// explicit facts.
//
// process_witness does not touch QTimer1 registers.  The counter observation
// goes through interrupt_vclock_counter32_observe_ambient(), preserving
// process_interrupt as the hardware authority.

static volatile uint32_t g_qtimer_read_samples = 0;
static volatile uint32_t g_qtimer_read_dwt_at_isr = 0;
static volatile uint32_t g_qtimer_read_dwt_before_counter_read = 0;
static volatile uint32_t g_qtimer_read_dwt_after_counter_read = 0;
static volatile uint32_t g_qtimer_read_counter32_at_read = 0;
static volatile uint32_t g_qtimer_read_entry_to_read_start_cycles = 0;
static volatile uint32_t g_qtimer_read_counter_read_cost_cycles = 0;
static volatile uint32_t g_qtimer_read_entry_to_read_end_cycles = 0;

static welford_t g_qtimer_read_entry_to_read_start_welford = {};
static welford_t g_qtimer_read_counter_read_cost_welford = {};
static welford_t g_qtimer_read_entry_to_read_end_welford = {};


// ============================================================================
// GPIO_DELAY — simple digitalWriteFast HIGH stimulus cost
// ============================================================================
//
// DWT before digitalWriteFast(HIGH), DWT immediately after.  This treats the
// GPIO write as synchronous/blocking and measures the stimulus instruction
// path itself.

static volatile uint32_t g_gpio_delay_samples = 0;
static volatile uint32_t g_gpio_delay_dwt_before = 0;
static volatile uint32_t g_gpio_delay_dwt_after = 0;
static volatile uint32_t g_gpio_delay_cycles = 0;
static welford_t g_gpio_delay_welford = {};

// ============================================================================
// DWT_READ — consecutive ARM_DWT_CYCCNT read cost
// ============================================================================

static volatile uint32_t g_dwt_read_samples = 0;
static volatile uint32_t g_dwt_read_before = 0;
static volatile uint32_t g_dwt_read_after = 0;
static volatile uint32_t g_dwt_read_cycles = 0;
static welford_t g_dwt_read_welford = {};

// ============================================================================
// ENTRY_LATENCY — foreground spin-shadow to ISR first-instruction DWT
// ============================================================================
//
// A TimePop callback lands shortly before a known interrupt, then spins while
// continuously copying ARM_DWT_CYCCNT into a shadow.  The target ISR records
// its first-instruction raw DWT and the shadow value it interrupted.  The
// minimum of (isr_entry_dwt_raw - shadow_at_isr) is the historical ISR entry
// latency measurement.

struct entry_latency_state_t {
  volatile bool     armed = false;
  volatile bool     spin_active = false;
  volatile bool     last_timeout = false;
  volatile uint32_t arm_count = 0;
  volatile uint32_t arm_failures = 0;
  volatile uint32_t timeout_count = 0;
  volatile uint32_t sample_count = 0;
  volatile uint32_t sequence = 0;
  volatile uint32_t landing_dwt = 0;
  volatile uint32_t shadow_dwt = 0;
  volatile uint32_t shadow_at_isr = 0;
  volatile uint32_t approach_cycles = 0;
  volatile uint32_t isr_entry_dwt_raw = 0;
  volatile uint32_t entry_latency_cycles = 0;
  welford_t welford = {};
};

static entry_latency_state_t g_entry_pps = {};
static entry_latency_state_t g_entry_qtimer = {};

// ============================================================================
// PPS_PHASE — raw notification and inferred physical PPS/VCLOCK phase
// ============================================================================
//
// PPS_PHASE piggybacks on the interrupt-owned CH1 BRIDGE compare event.  It
// compares the PPS GPIO ISR first-instruction raw DWT against the CH1 compare
// ISR first-instruction raw DWT, then subtracts the deliberately requested
// VCLOCK tick offset to infer the physical phase between PPS and the selected
// VCLOCK edge.
//
// The inferred physical phase uses the best current pin-arrival model from
// ROUND_TRIP: GPIO_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY for PPS, and
// QTIMER_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY for QTimer.
//
// The result is intentionally NOT reduced modulo one VCLOCK tick: if the
// selected edge is more than one tick away from PPS, we want to see the full
// offset.

struct pps_phase_capture_t {
  uint32_t seq = 0;
  bool     valid = false;

  uint32_t pps_sequence = 0;
  uint32_t qtimer_sequence = 0;

  uint32_t pps_isr_entry_dwt_raw = 0;
  uint32_t vclock_isr_entry_dwt_raw = 0;
  uint32_t raw_notification_delta_cycles = 0;

  uint32_t pps_counter32_at_edge = 0;
  uint32_t pps_vclock_counter32_at_edge = 0;
  uint32_t target_counter32 = 0;
  uint32_t counter32_at_fire = 0;
  uint32_t selected_to_target_ticks = 0;
  uint64_t selected_to_target_ns = 0;

  uint32_t dynamic_cps = 0;
  uint32_t expected_offset_cycles = 0;

  int32_t  pps_pin_to_isr_cycles = 0;
  int32_t  qtimer_pin_to_isr_cycles = 0;
  int64_t  selected_physical_phase_cycles = 0;
  double   selected_physical_phase_ticks = 0.0;
  bool     greater_than_one_vclock_tick = false;
};

static pps_phase_capture_t g_phase_last = {};
static volatile uint32_t g_phase_capture_seq = 0;
static volatile uint32_t g_phase_pps_sequence = 0;
static volatile uint32_t g_phase_pps_isr_entry_dwt_raw = 0;
static volatile uint32_t g_phase_samples = 0;
static volatile uint32_t g_phase_skipped_no_pps_raw = 0;
static volatile uint32_t g_phase_skipped_sequence_mismatch = 0;
static volatile uint32_t g_phase_skipped_no_dynamic_cps = 0;
static welford_t g_phase_notification_welford = {};
static welford_t g_phase_selected_physical_welford = {};

// ============================================================================
// BRIDGE — DWT/GNSS bridge validation against VCLOCK truth
// ============================================================================
//
// One fixed sub-second sample per PPS_VCLOCK second.  VCLOCK is the ground
// truth: gnss_from_vclock_ns is authored from PPS_VCLOCK + counter ticks.
// The test asks whether time_dwt_to_gnss_ns(dwt_at_fire) returns the same
// value for the DWT captured at the compare ISR.

static constexpr uint8_t  BRIDGE_SLOT_INDEX = 5;
// Deliberately not exactly 500 ms: avoiding exact millisecond phase prevents
// CH1 BRIDGE from colliding with the production CH3 1 kHz cadence in the
// shared QTimer1 vector.
static constexpr uint32_t BRIDGE_SLOT_OFFSET_TICKS = 5000123U;
static constexpr uint64_t BRIDGE_SLOT_OFFSET_NS =
    (uint64_t)BRIDGE_SLOT_OFFSET_TICKS * 100ULL;

struct bridge_capture_t {
  uint32_t seq = 0;
  bool     valid = false;

  uint32_t slot_index = 0;
  uint64_t offset_ns = 0;

  uint32_t pvc_sequence = 0;
  uint32_t pvc_dwt_at_edge = 0;
  uint32_t pvc_counter32_at_edge = 0;
  int64_t  pvc_gnss_ns_at_edge = -1;

  uint32_t dynamic_cps = 0;

  uint32_t target_counter32 = 0;
  uint32_t counter32_at_fire = 0;
  int32_t  counter32_residual_ticks = 0;

  uint32_t dwt_at_fire = 0;

  int64_t  gnss_from_vclock_ns = -1;
  int64_t  gnss_from_time_bridge_ns = -1;
  int64_t  residual_ns = 0;

  int64_t  gnss_from_dynamic_dwt_ns = -1;
  int64_t  dynamic_residual_ns = 0;
};

static bridge_capture_t g_bridge_last = {};
static volatile uint32_t g_bridge_capture_seq = 0;
static volatile uint64_t g_bridge_fires_total = 0;
static volatile uint64_t g_bridge_arm_count = 0;
static volatile uint64_t g_bridge_arm_failures = 0;
static volatile uint64_t g_bridge_skipped_no_anchor = 0;
static volatile uint64_t g_bridge_skipped_no_dynamic_cps = 0;
static volatile uint32_t g_bridge_last_armed_target_counter32 = 0;
static volatile bool g_bridge_armed = false;
static welford_t g_bridge_residual_welford = {};
static welford_t g_bridge_dynamic_residual_welford = {};

// ============================================================================
// Forward declarations
// ============================================================================

void witness_gpio_isr(void);

static void qtimer2_isr(void);
static void gpio_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_cycle_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_qtimer_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_isr_handler(uint32_t sequence, uint32_t isr_entry_dwt_raw);
static bool entry_latency_arm_pps_spin(void);
static bool entry_latency_arm_qtimer_spin(uint32_t target_counter32);
static void bridge_ch1_handler(const interrupt_qtimer1_ch1_compare_event_t& event);
static bool bridge_arm_next_sample(void);
static void pps_phase_capture(const interrupt_qtimer1_ch1_compare_event_t& event,
                              const pps_t& pps,
                              const pps_vclock_t& pvc,
                              uint32_t dynamic_cps);
static void qtimer2_arm_next_edge(void);
static bool witness_start_activity(void);
static void witness_stop_activity(void);
static Payload witness_state_payload(void);

// ============================================================================
// QTimer2 CH0 helpers — intentionally old process_interrupt style
// ============================================================================

static inline void qtimer2_ch0_clear_compare_flag(void) {
  IMXRT_TMR2.CH[0].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer2_ch0_program_compare(uint16_t target_low16) {
  qtimer2_ch0_clear_compare_flag();

  IMXRT_TMR2.CH[0].COMP1  = target_low16;
  IMXRT_TMR2.CH[0].CMPLD1 = target_low16;

  qtimer2_ch0_clear_compare_flag();
  IMXRT_TMR2.CH[0].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer2_ch0_disable_compare(void) {
  IMXRT_TMR2.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer2_ch0_clear_compare_flag();
}

static void qtimer2_arm_next_edge(void) {
  if (!g_witness_running) return;

  const uint16_t cntr = IMXRT_TMR2.CH[0].CNTR;
  const uint16_t target = (uint16_t)(cntr + 1);

  g_qtimer_compare_target = target;
  g_qtimer_arms++;

  qtimer2_ch0_program_compare(target);
}

// ============================================================================
// Source drive
// ============================================================================

static void witness_drive_high(void) {
  const uint32_t dwt_before = ARM_DWT_CYCCNT;
  digitalWriteFast(WITNESS_STIMULUS_PIN, HIGH);
  const uint32_t dwt_after = ARM_DWT_CYCCNT;

  g_source_dwt_before = dwt_before;
  g_source_dwt_after = dwt_after;
  g_source_stim_cycles = dwt_after - dwt_before;
  g_source_dwt_at_emit = dwt_before;
  g_source_emits++;
  g_source_high = true;

  welford_update(g_source_welford, (int32_t)g_source_stim_cycles);
}

static void witness_drive_low(void) {
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);
  g_source_lows++;
  g_source_high = false;
}


static void capture_gpio_delay_sample(void) {
  const uint32_t dwt_before = ARM_DWT_CYCCNT;
  digitalWriteFast(WITNESS_STIMULUS_PIN, HIGH);
  const uint32_t dwt_after = ARM_DWT_CYCCNT;
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  const uint32_t sample = dwt_after - dwt_before;
  g_gpio_delay_dwt_before = dwt_before;
  g_gpio_delay_dwt_after = dwt_after;
  g_gpio_delay_cycles = sample;
  g_gpio_delay_samples++;
  g_source_high = false;

  welford_update(g_gpio_delay_welford, (int32_t)sample);
}

static void capture_dwt_read_sample(void) {
  const uint32_t dwt_before = ARM_DWT_CYCCNT;
  const uint32_t dwt_after = ARM_DWT_CYCCNT;

  const uint32_t sample = dwt_after - dwt_before;
  g_dwt_read_before = dwt_before;
  g_dwt_read_after = dwt_after;
  g_dwt_read_cycles = sample;
  g_dwt_read_samples++;

  welford_update(g_dwt_read_welford, (int32_t)sample);
}

// ============================================================================
// ENTRY_LATENCY helpers
// ============================================================================

static void entry_latency_reset(entry_latency_state_t& s) {
  s.armed = false;
  s.spin_active = false;
  s.last_timeout = false;
  s.arm_count = 0;
  s.arm_failures = 0;
  s.timeout_count = 0;
  s.sample_count = 0;
  s.sequence = 0;
  s.landing_dwt = 0;
  s.shadow_dwt = 0;
  s.shadow_at_isr = 0;
  s.approach_cycles = 0;
  s.isr_entry_dwt_raw = 0;
  s.entry_latency_cycles = 0;
  welford_reset(s.welford);
}

static void entry_latency_record_isr(entry_latency_state_t& s,
                                     uint32_t sequence,
                                     uint32_t isr_entry_dwt_raw) {
  if (!s.spin_active) return;

  const uint32_t shadow = s.shadow_dwt;
  const uint32_t landing = s.landing_dwt;
  const uint32_t approach = shadow - landing;
  const uint32_t latency = isr_entry_dwt_raw - shadow;

  s.sequence = sequence;
  s.shadow_at_isr = shadow;
  s.approach_cycles = approach;
  s.isr_entry_dwt_raw = isr_entry_dwt_raw;
  s.entry_latency_cycles = latency;
  s.sample_count++;
  s.last_timeout = false;
  s.spin_active = false;

  welford_update(s.welford, (int32_t)latency);
}

static void entry_latency_spin(entry_latency_state_t& s) {
  const uint32_t landing = ARM_DWT_CYCCNT;
  s.landing_dwt = landing;
  s.shadow_dwt = landing;
  s.shadow_at_isr = 0;
  s.approach_cycles = 0;
  s.isr_entry_dwt_raw = 0;
  s.entry_latency_cycles = 0;
  s.last_timeout = false;
  s.spin_active = true;

  while (s.spin_active) {
    const uint32_t now = ARM_DWT_CYCCNT;
    s.shadow_dwt = now;

    if ((uint32_t)(now - landing) > ENTRY_LATENCY_TIMEOUT_CYCLES) {
      s.spin_active = false;
      s.last_timeout = true;
      s.timeout_count++;
      break;
    }
  }
}

static int64_t entry_latency_target_gnss_for_counter(uint32_t target_counter32) {
  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) return -1;
  const uint32_t ticks_since_anchor = target_counter32 - pvc.counter32_at_edge;
  return pvc.gnss_ns_at_edge + (int64_t)ticks_since_anchor * 100LL;
}

static bool entry_latency_arm_pps_spin(void) {
  if (!g_witness_running) return false;

  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) {
    g_entry_pps.arm_failures++;
    return false;
  }

  int64_t target_gnss_ns = pvc.gnss_ns_at_edge + 1000000000LL -
                           (int64_t)ENTRY_LATENCY_LEAD_NS;
  const int64_t now_gnss_ns = time_gnss_ns_now();
  while (now_gnss_ns >= 0 && target_gnss_ns <= now_gnss_ns) {
    target_gnss_ns += 1000000000LL;
  }

  timepop_cancel_by_name(ENTRY_PPS_SPIN_NAME);
  const timepop_handle_t h =
      timepop_arm_at(target_gnss_ns,
                     false,
                     entry_latency_pps_spin_callback,
                     nullptr,
                     ENTRY_PPS_SPIN_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_entry_pps.armed = false;
    g_entry_pps.arm_failures++;
    return false;
  }

  g_entry_pps.armed = true;
  g_entry_pps.arm_count++;
  return true;
}

static bool entry_latency_arm_qtimer_spin(uint32_t target_counter32) {
  if (!g_witness_running) return false;

  const int64_t target_event_gnss_ns =
      entry_latency_target_gnss_for_counter(target_counter32);
  if (target_event_gnss_ns < 0) {
    g_entry_qtimer.arm_failures++;
    return false;
  }

  const int64_t spin_target_gnss_ns =
      target_event_gnss_ns - (int64_t)ENTRY_LATENCY_LEAD_NS;
  const int64_t now_gnss_ns = time_gnss_ns_now();
  if (now_gnss_ns >= 0 && spin_target_gnss_ns <= now_gnss_ns) {
    g_entry_qtimer.arm_failures++;
    return false;
  }

  timepop_cancel_by_name(ENTRY_QTIMER_SPIN_NAME);
  const timepop_handle_t h =
      timepop_arm_at(spin_target_gnss_ns,
                     false,
                     entry_latency_qtimer_spin_callback,
                     nullptr,
                     ENTRY_QTIMER_SPIN_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_entry_qtimer.armed = false;
    g_entry_qtimer.arm_failures++;
    return false;
  }

  g_entry_qtimer.armed = true;
  g_entry_qtimer.arm_count++;
  return true;
}

static void entry_latency_pps_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  g_entry_pps.armed = false;
  entry_latency_spin(g_entry_pps);
  (void)entry_latency_arm_pps_spin();
}

static void entry_latency_qtimer_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  g_entry_qtimer.armed = false;
  entry_latency_spin(g_entry_qtimer);
}

static void entry_latency_pps_isr_handler(uint32_t sequence,
                                          uint32_t isr_entry_dwt_raw) {
  if (!g_witness_running) return;

  g_phase_pps_sequence = sequence;
  g_phase_pps_isr_entry_dwt_raw = isr_entry_dwt_raw;
  entry_latency_record_isr(g_entry_pps, sequence, isr_entry_dwt_raw);
}

// ============================================================================
// GPIO sink ISR — old-style stable GPIO sink, rising edge only
// ============================================================================

static inline void capture_qtimer_read_sample(uint32_t isr_entry_dwt_raw) {
  const uint32_t before_read = ARM_DWT_CYCCNT;
  const uint32_t counter32 = interrupt_vclock_counter32_observe_ambient();
  const uint32_t after_read = ARM_DWT_CYCCNT;

  const uint32_t entry_to_start = before_read - isr_entry_dwt_raw;
  const uint32_t read_cost = after_read - before_read;
  const uint32_t entry_to_end = after_read - isr_entry_dwt_raw;

  g_qtimer_read_dwt_at_isr = isr_entry_dwt_raw;
  g_qtimer_read_dwt_before_counter_read = before_read;
  g_qtimer_read_dwt_after_counter_read = after_read;
  g_qtimer_read_counter32_at_read = counter32;
  g_qtimer_read_entry_to_read_start_cycles = entry_to_start;
  g_qtimer_read_counter_read_cost_cycles = read_cost;
  g_qtimer_read_entry_to_read_end_cycles = entry_to_end;
  g_qtimer_read_samples++;

  welford_update(g_qtimer_read_entry_to_read_start_welford, (int32_t)entry_to_start);
  welford_update(g_qtimer_read_counter_read_cost_welford, (int32_t)read_cost);
  welford_update(g_qtimer_read_entry_to_read_end_welford, (int32_t)entry_to_end);
}

void witness_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;

  if (!g_witness_running) return;

  capture_qtimer_read_sample(isr_entry_dwt_raw);

  if (g_active_window != witness_window_t::GPIO) {
    g_gpio_wrong_window++;
    return;
  }

  const int32_t sample = (int32_t)(isr_entry_dwt_raw - g_source_dwt_at_emit);

  g_gpio_dwt_at_isr = isr_entry_dwt_raw;
  g_gpio_delta_cycles = (uint32_t)sample;
  g_gpio_hits++;
  welford_update(g_gpio_welford, sample);
}

// ============================================================================
// QTimer2 sink ISR — old-style stable rail, self-rearming
// ============================================================================

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  g_qtimer_irq_entries++;

  if (!(IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1)) {
    g_qtimer_no_flag++;
    return;
  }

  qtimer2_ch0_clear_compare_flag();

  if (!g_witness_running) {
    return;
  }

  if (g_active_window == witness_window_t::QTIMER) {
    const int32_t sample = (int32_t)(isr_entry_dwt_raw - g_source_dwt_at_emit);

    g_qtimer_dwt_at_isr = isr_entry_dwt_raw;
    g_qtimer_delta_cycles = (uint32_t)sample;
    g_qtimer_hits++;
    welford_update(g_qtimer_welford, sample);
  } else {
    g_qtimer_outside_window++;
  }

  // This is the key old-process_interrupt behavior: immediately re-arm
  // against the continuously live counter from inside the ISR.
  qtimer2_arm_next_edge();
}

// ============================================================================
// TimePop waveform callbacks — separated GPIO/QTimer measurement windows
// ============================================================================

static void gpio_high_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  g_active_window = witness_window_t::GPIO;
  pinMode(WITNESS_GPIO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN), witness_gpio_isr, RISING);
  witness_drive_high();
}

static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  witness_drive_low();
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  g_active_window = witness_window_t::NONE;
}

static void qtimer_high_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  g_active_window = witness_window_t::QTIMER;
  witness_drive_high();
}

static void qtimer_low_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  witness_drive_low();
  g_active_window = witness_window_t::NONE;
}

static void witness_cancel_edge_callbacks(void) {
  timepop_cancel_by_name(GPIO_HIGH_NAME);
  timepop_cancel_by_name(GPIO_LOW_NAME);
  timepop_cancel_by_name(QTIMER_HIGH_NAME);
  timepop_cancel_by_name(QTIMER_LOW_NAME);
}

static void witness_arm_cycle_edges(void) {
  witness_cancel_edge_callbacks();

  timepop_arm(GPIO_HIGH_OFFSET_NS,
              false,
              gpio_high_callback,
              nullptr,
              GPIO_HIGH_NAME);

  timepop_arm(GPIO_LOW_OFFSET_NS,
              false,
              gpio_low_callback,
              nullptr,
              GPIO_LOW_NAME);

  timepop_arm(QTIMER_HIGH_OFFSET_NS,
              false,
              qtimer_high_callback,
              nullptr,
              QTIMER_HIGH_NAME);

  timepop_arm(QTIMER_LOW_OFFSET_NS,
              false,
              qtimer_low_callback,
              nullptr,
              QTIMER_LOW_NAME);

  g_witness_cycle_reschedules++;
}

static void witness_cycle_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  g_witness_cycle_count++;

  // Start every cycle from a known, quiet witness state.  The QTimer rail
  // remains continuously armed; only GPIO and the stimulus pin are reset.
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);
  g_source_high = false;
  g_active_window = witness_window_t::NONE;
  g_witness_cycle_count = 0;
  g_witness_cycle_reschedules = 0;

  witness_arm_cycle_edges();
  if (!g_bridge_armed) {
    (void)bridge_arm_next_sample();
  }
  if (!g_entry_pps.armed && !g_entry_pps.spin_active) {
    (void)entry_latency_arm_pps_spin();
  }
}

static void witness_schedule_first_test(void) {
  if (!g_witness_running) return;
  if (g_witness_schedule_armed) return;

  timepop_cancel_by_name(WITNESS_CYCLE_NAME);
  witness_cancel_edge_callbacks();

  timepop_arm(WITNESS_CYCLE_PERIOD_NS,
              true,
              witness_cycle_callback,
              nullptr,
              WITNESS_CYCLE_NAME);

  (void)bridge_arm_next_sample();

  g_witness_schedule_armed = true;

  // Arm the first second immediately; subsequent seconds are regenerated by
  // the recurring supervisor.  This avoids chains of child callbacks.
  g_witness_cycle_count++;
  witness_arm_cycle_edges();
}

// ============================================================================
// START / STOP lifecycle
// ============================================================================

static void witness_cancel_all_timepop_callbacks(void) {
  timepop_cancel_by_name(WITNESS_CYCLE_NAME);
  witness_cancel_edge_callbacks();
  timepop_cancel_by_name(ENTRY_PPS_SPIN_NAME);
  timepop_cancel_by_name(ENTRY_QTIMER_SPIN_NAME);
}

static void witness_quiesce_outputs_and_irqs(void) {
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);
  g_source_high = false;
  g_active_window = witness_window_t::NONE;

  qtimer2_ch0_disable_compare();

  interrupt_qtimer1_ch1_disable_compare();
  interrupt_register_qtimer1_ch1_handler(nullptr);
  interrupt_register_pps_entry_latency_handler(nullptr);
}

static void witness_clear_live_flags(void) {
  g_witness_schedule_armed = false;
  g_bridge_armed = false;

  g_entry_pps.armed = false;
  g_entry_pps.spin_active = false;
  g_entry_qtimer.armed = false;
  g_entry_qtimer.spin_active = false;

  g_active_window = witness_window_t::NONE;
}

static bool witness_start_activity(void) {
  if (!g_witness_runtime_ready) return false;

  if (g_witness_running) {
    return true;
  }

  // Begin from a fully quiet state.  This handles stale callbacks left over
  // from an interrupted prior run and makes START idempotent from the outside.
  witness_cancel_all_timepop_callbacks();
  witness_quiesce_outputs_and_irqs();
  witness_clear_live_flags();

  g_witness_running = true;
  g_witness_start_count++;
  g_witness_last_start_dwt = ARM_DWT_CYCCNT;

  // These hooks are intentionally registered only while witness is running.
  // STOP removes them so the timing hot paths stay quiet during campaign init.
  interrupt_register_qtimer1_ch1_handler(bridge_ch1_handler);
  interrupt_register_pps_entry_latency_handler(entry_latency_pps_isr_handler);

  // Full circus: round-trip supervisor, QTimer2 sink rail, CH1 bridge, and
  // PPS entry-latency spin.  Individual commands may also re-arm their own
  // instruments, but START brings the legacy always-on witness behavior back.
  qtimer2_arm_next_edge();
  witness_schedule_first_test();
  (void)entry_latency_arm_pps_spin();

  return true;
}

static void witness_stop_activity(void) {
  if (!g_witness_runtime_ready && !g_witness_running) return;

  g_witness_running = false;
  g_witness_stop_count++;
  g_witness_last_stop_dwt = ARM_DWT_CYCCNT;

  witness_cancel_all_timepop_callbacks();
  witness_quiesce_outputs_and_irqs();
  witness_clear_live_flags();
}

// ============================================================================
// Payload helpers
// ============================================================================

static Payload welford_payload(const welford_t& w) {
  const double stddev_cycles = welford_stddev(w);
  const double stderr_cycles =
      (w.n >= 2) ? (stddev_cycles / sqrt((double)w.n)) : 0.0;

  Payload p;
  p.add("n",             (uint32_t)w.n);
  p.add("mean_cycles",   w.mean);
  p.add("stddev_cycles", stddev_cycles);
  p.add("stderr_cycles", stderr_cycles);
  p.add("min_cycles",    (int32_t)w.min_val);
  p.add("max_cycles",    (int32_t)w.max_val);
  p.add("mean_ns",       cycles_to_ns(w.mean));
  p.add("stddev_ns",     cycles_to_ns(stddev_cycles));
  p.add("stderr_ns",     cycles_to_ns(stderr_cycles));
  p.add("min_ns",        cycles_to_ns((double)w.min_val));
  p.add("max_ns",        cycles_to_ns((double)w.max_val));
  return p;
}


static Payload ns_welford_payload(const welford_t& w) {
  const double stddev_ns = welford_stddev(w);
  const double stderr_ns =
      (w.n >= 2) ? (stddev_ns / sqrt((double)w.n)) : 0.0;

  Payload p;
  p.add("n",         (uint32_t)w.n);
  p.add("mean_ns",   w.mean);
  p.add("stddev_ns", stddev_ns);
  p.add("stderr_ns", stderr_ns);
  p.add("min_ns",    (w.n > 0) ? (int32_t)w.min_val : 0);
  p.add("max_ns",    (w.n > 0) ? (int32_t)w.max_val : 0);
  return p;
}

static const char* witness_window_str(witness_window_t w) {
  switch (w) {
    case witness_window_t::GPIO:   return "GPIO";
    case witness_window_t::QTIMER: return "QTIMER";
    default:                       return "NONE";
  }
}

static Payload witness_state_payload(void) {
  Payload p;
  p.add("running", g_witness_running);
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);
  p.add("start_count", g_witness_start_count);
  p.add("stop_count", g_witness_stop_count);
  p.add("last_start_dwt", g_witness_last_start_dwt);
  p.add("last_stop_dwt", g_witness_last_stop_dwt);

  Payload timepop_state;
  timepop_state.add("cycle_armed", g_witness_schedule_armed);
  timepop_state.add("cycle_count", g_witness_cycle_count);
  timepop_state.add("cycle_reschedules", g_witness_cycle_reschedules);
  timepop_state.add("bridge_armed", g_bridge_armed);
  timepop_state.add("entry_pps_armed", g_entry_pps.armed);
  timepop_state.add("entry_pps_spin_active", g_entry_pps.spin_active);
  timepop_state.add("entry_qtimer_armed", g_entry_qtimer.armed);
  timepop_state.add("entry_qtimer_spin_active", g_entry_qtimer.spin_active);
  p.add_object("timepop", timepop_state);

  const uint16_t ch1_csctrl = interrupt_qtimer1_ch1_csctrl_now();
  Payload qtimer1_ch1;
  qtimer1_ch1.add("counter", (uint32_t)interrupt_qtimer1_ch1_counter_now());
  qtimer1_ch1.add("comp1", (uint32_t)interrupt_qtimer1_ch1_comp1_now());
  qtimer1_ch1.add("csctrl", (uint32_t)ch1_csctrl);
  qtimer1_ch1.add("compare_enabled", (bool)(ch1_csctrl & TMR_CSCTRL_TCF1EN));
  qtimer1_ch1.add("compare_flag", (bool)(ch1_csctrl & TMR_CSCTRL_TCF1));
  p.add_object("qtimer1_ch1_bridge", qtimer1_ch1);

  Payload qtimer2;
  qtimer2.add("counter", (uint32_t)IMXRT_TMR2.CH[0].CNTR);
  qtimer2.add("comp1", (uint32_t)IMXRT_TMR2.CH[0].COMP1);
  qtimer2.add("csctrl", (uint32_t)IMXRT_TMR2.CH[0].CSCTRL);
  qtimer2.add("compare_enabled", (bool)(IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1EN));
  qtimer2.add("compare_flag", (bool)(IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1));
  qtimer2.add("arms", g_qtimer_arms);
  qtimer2.add("irq_entries", g_qtimer_irq_entries);
  qtimer2.add("hits", g_qtimer_hits);
  qtimer2.add("no_flag", g_qtimer_no_flag);
  qtimer2.add("outside_window", g_qtimer_outside_window);
  qtimer2.add("last_compare_target", (uint32_t)g_qtimer_compare_target);
  p.add_object("qtimer2_ch0_sink", qtimer2);

  Payload gpio;
  gpio.add("active_window", witness_window_str(g_active_window));
  gpio.add("source_high", g_source_high);
  gpio.add("source_emits", g_source_emits);
  gpio.add("source_lows", g_source_lows);
  gpio.add("gpio_hits", g_gpio_hits);
  gpio.add("gpio_wrong_window", g_gpio_wrong_window);
  p.add_object("gpio", gpio);

  Payload bridge;
  bridge.add("armed", g_bridge_armed);
  bridge.add("fires_total", (uint32_t)g_bridge_fires_total);
  bridge.add("arm_count", (uint32_t)g_bridge_arm_count);
  bridge.add("arm_failures", (uint32_t)g_bridge_arm_failures);
  bridge.add("last_armed_target_counter32", g_bridge_last_armed_target_counter32);
  bridge.add("skipped_no_anchor", (uint32_t)g_bridge_skipped_no_anchor);
  bridge.add("skipped_no_dynamic_cps", (uint32_t)g_bridge_skipped_no_dynamic_cps);
  p.add_object("bridge", bridge);

  Payload entry;
  entry.add("pps_armed", g_entry_pps.armed);
  entry.add("pps_spin_active", g_entry_pps.spin_active);
  entry.add("pps_last_timeout", g_entry_pps.last_timeout);
  entry.add("pps_arm_count", g_entry_pps.arm_count);
  entry.add("pps_arm_failures", g_entry_pps.arm_failures);
  entry.add("pps_timeout_count", g_entry_pps.timeout_count);
  entry.add("pps_samples", g_entry_pps.sample_count);
  entry.add("qtimer_armed", g_entry_qtimer.armed);
  entry.add("qtimer_spin_active", g_entry_qtimer.spin_active);
  entry.add("qtimer_last_timeout", g_entry_qtimer.last_timeout);
  entry.add("qtimer_arm_count", g_entry_qtimer.arm_count);
  entry.add("qtimer_arm_failures", g_entry_qtimer.arm_failures);
  entry.add("qtimer_timeout_count", g_entry_qtimer.timeout_count);
  entry.add("qtimer_samples", g_entry_qtimer.sample_count);
  p.add_object("entry_latency", entry);

  return p;
}

static bool bridge_arm_next_sample(void) {
  if (!g_witness_running) return false;

  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) {
    g_bridge_skipped_no_anchor++;
    return false;
  }

  uint32_t target_counter32 = pvc.counter32_at_edge + BRIDGE_SLOT_OFFSET_TICKS;

  // If this second's sample has already passed, target the same offset in the
  // next VCLOCK second.  This uses synthetic counter arithmetic only; no
  // direct QTimer1 register read occurs in process_witness.
  const uint32_t now = interrupt_vclock_counter32_observe_ambient();
  if ((int32_t)(target_counter32 - now) <= 1000) {
    target_counter32 += 10000000U;
  }

  g_bridge_last_armed_target_counter32 = target_counter32;
  g_bridge_arm_count++;

  if (!interrupt_qtimer1_ch1_arm_compare(target_counter32)) {
    g_bridge_armed = false;
    g_bridge_arm_failures++;
    return false;
  }

  g_bridge_armed = true;
  (void)entry_latency_arm_qtimer_spin(target_counter32);
  return true;
}

static void pps_phase_capture(const interrupt_qtimer1_ch1_compare_event_t& event,
                              const pps_t& pps,
                              const pps_vclock_t& pvc,
                              uint32_t dynamic_cps) {
  if (g_phase_pps_sequence == 0 || g_phase_pps_isr_entry_dwt_raw == 0) {
    g_phase_skipped_no_pps_raw++;
    return;
  }

  if (g_phase_pps_sequence != pvc.sequence) {
    g_phase_skipped_sequence_mismatch++;
    return;
  }

  if (dynamic_cps == 0) {
    g_phase_skipped_no_dynamic_cps++;
    return;
  }

  const uint32_t pps_raw = g_phase_pps_isr_entry_dwt_raw;
  const uint32_t vclock_raw = event.isr_entry_dwt_raw;
  const uint32_t raw_delta_cycles = vclock_raw - pps_raw;
  const uint32_t selected_to_target_ticks = event.target_counter32 - pvc.counter32_at_edge;
  const uint64_t selected_to_target_ns = (uint64_t)selected_to_target_ticks * 100ULL;

  const uint32_t expected_offset_cycles =
      (uint32_t)(((uint64_t)selected_to_target_ticks * (uint64_t)dynamic_cps +
                  (uint64_t)VCLOCK_TICKS_PER_SECOND_U32 / 2ULL) /
                 (uint64_t)VCLOCK_TICKS_PER_SECOND_U32);

  // Best estimate of selected-VCLOCK-edge physical phase at the Teensy pins:
  //   qtimer_pin_time - pps_pin_time
  // = (qtimer_isr_raw - qtimer_pin_to_isr)
  //   - (pps_isr_raw - pps_pin_to_isr)
  //   - selected_to_target_offset
  // = raw_delta + pps_pin_to_isr - qtimer_pin_to_isr - expected_offset.
  const int64_t selected_physical_phase_cycles =
      (int64_t)raw_delta_cycles +
      (int64_t)PPS_PIN_TO_ISR_CYCLES -
      (int64_t)QTIMER_PIN_TO_ISR_CYCLES -
      (int64_t)expected_offset_cycles;

  const double selected_physical_phase_ticks =
      ((double)selected_physical_phase_cycles *
       (double)VCLOCK_TICKS_PER_SECOND_U32) /
      (double)dynamic_cps;

  pps_phase_capture_t cap;
  cap.seq = ++g_phase_capture_seq;
  cap.valid = true;
  cap.pps_sequence = pvc.sequence;
  cap.qtimer_sequence = event.sequence;
  cap.pps_isr_entry_dwt_raw = pps_raw;
  cap.vclock_isr_entry_dwt_raw = vclock_raw;
  cap.raw_notification_delta_cycles = raw_delta_cycles;
  cap.pps_counter32_at_edge = pps.counter32_at_edge;
  cap.pps_vclock_counter32_at_edge = pvc.counter32_at_edge;
  cap.target_counter32 = event.target_counter32;
  cap.counter32_at_fire = event.counter32_at_event;
  cap.selected_to_target_ticks = selected_to_target_ticks;
  cap.selected_to_target_ns = selected_to_target_ns;
  cap.dynamic_cps = dynamic_cps;
  cap.expected_offset_cycles = expected_offset_cycles;
  cap.pps_pin_to_isr_cycles = PPS_PIN_TO_ISR_CYCLES;
  cap.qtimer_pin_to_isr_cycles = QTIMER_PIN_TO_ISR_CYCLES;
  cap.selected_physical_phase_cycles = selected_physical_phase_cycles;
  cap.selected_physical_phase_ticks = selected_physical_phase_ticks;
  cap.greater_than_one_vclock_tick = fabs(selected_physical_phase_ticks) > 1.0;

  g_phase_last = cap;
  g_phase_samples++;
  welford_update(g_phase_notification_welford, (int32_t)raw_delta_cycles);
  welford_update(g_phase_selected_physical_welford, (int32_t)selected_physical_phase_cycles);
}

static void bridge_capture(const interrupt_qtimer1_ch1_compare_event_t& event) {
  const pps_t pps = interrupt_last_pps();
  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) {
    g_bridge_skipped_no_anchor++;
    return;
  }

  const uint32_t qtimer_event_dwt = event.dwt_at_event;
  const uint32_t authored_counter32 = event.counter32_at_event;

  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  if (dynamic_cps == 0) {
    g_bridge_skipped_no_dynamic_cps++;
  }

  pps_phase_capture(event, pps, pvc, dynamic_cps);

  const uint32_t target_counter32 = event.target_counter32;
  const int32_t counter32_residual_ticks = event.counter32_residual_ticks;

  const uint32_t vclock_ticks_since_anchor = authored_counter32 - pvc.counter32_at_edge;
  const int64_t gnss_from_vclock_ns =
      pvc.gnss_ns_at_edge + (int64_t)vclock_ticks_since_anchor * 100LL;

  const int64_t gnss_from_time_bridge_ns = time_dwt_to_gnss_ns(qtimer_event_dwt);
  const int64_t residual_ns = gnss_from_time_bridge_ns - gnss_from_vclock_ns;

  int64_t gnss_from_dynamic_dwt_ns = -1;
  int64_t dynamic_residual_ns = 0;
  if (dynamic_cps != 0) {
    const uint32_t dwt_delta = qtimer_event_dwt - pvc.dwt_at_edge;
    const uint64_t ns_delta =
        ((uint64_t)dwt_delta * 1000000000ULL + (uint64_t)dynamic_cps / 2ULL) /
        (uint64_t)dynamic_cps;
    gnss_from_dynamic_dwt_ns = pvc.gnss_ns_at_edge + (int64_t)ns_delta;
    dynamic_residual_ns = gnss_from_dynamic_dwt_ns - gnss_from_vclock_ns;
  }

  bridge_capture_t cap;
  cap.seq = ++g_bridge_capture_seq;
  cap.valid = true;
  cap.slot_index = BRIDGE_SLOT_INDEX;
  cap.offset_ns = BRIDGE_SLOT_OFFSET_NS;
  cap.pvc_sequence = pvc.sequence;
  cap.pvc_dwt_at_edge = pvc.dwt_at_edge;
  cap.pvc_counter32_at_edge = pvc.counter32_at_edge;
  cap.pvc_gnss_ns_at_edge = pvc.gnss_ns_at_edge;
  cap.dynamic_cps = dynamic_cps;
  cap.target_counter32 = target_counter32;
  cap.counter32_at_fire = authored_counter32;
  cap.counter32_residual_ticks = counter32_residual_ticks;
  cap.dwt_at_fire = qtimer_event_dwt;
  cap.gnss_from_vclock_ns = gnss_from_vclock_ns;
  cap.gnss_from_time_bridge_ns = gnss_from_time_bridge_ns;
  cap.residual_ns = residual_ns;
  cap.gnss_from_dynamic_dwt_ns = gnss_from_dynamic_dwt_ns;
  cap.dynamic_residual_ns = dynamic_residual_ns;

  g_bridge_last = cap;
  g_bridge_fires_total++;
  welford_update(g_bridge_residual_welford, (int32_t)residual_ns);
  if (dynamic_cps != 0) {
    welford_update(g_bridge_dynamic_residual_welford, (int32_t)dynamic_residual_ns);
  }
}

static void bridge_ch1_handler(const interrupt_qtimer1_ch1_compare_event_t& event) {
  g_bridge_armed = false;
  if (!g_witness_running) return;
  entry_latency_record_isr(g_entry_qtimer, event.sequence, event.isr_entry_dwt_raw);
  bridge_capture(event);
  (void)bridge_arm_next_sample();
}

static Payload cmd_bridge(const Payload&) {
  if (g_witness_running && !g_bridge_armed && g_witness_runtime_ready) {
    (void)bridge_arm_next_sample();
  }

  Payload p;
  p.add("model", "DWT_GNSS_BRIDGE_VS_VCLOCK_TRUTH");
  p.add("slot_index", (uint32_t)BRIDGE_SLOT_INDEX);
  p.add("offset_ns", BRIDGE_SLOT_OFFSET_NS);
  p.add("fires_total", (uint32_t)g_bridge_fires_total);
  p.add("arm_count", (uint32_t)g_bridge_arm_count);
  p.add("armed", g_bridge_armed);
  p.add("arm_failures", (uint32_t)g_bridge_arm_failures);
  p.add("last_armed_target_counter32", g_bridge_last_armed_target_counter32);
  p.add("skipped_no_anchor", (uint32_t)g_bridge_skipped_no_anchor);
  p.add("skipped_no_dynamic_cps", (uint32_t)g_bridge_skipped_no_dynamic_cps);

  const bridge_capture_t cap = g_bridge_last;
  p.add("valid", cap.valid);
  p.add("seq", cap.seq);

  Payload anchor;
  anchor.add("pvc_sequence", cap.pvc_sequence);
  anchor.add("pvc_dwt_at_edge", cap.pvc_dwt_at_edge);
  anchor.add("pvc_counter32_at_edge", cap.pvc_counter32_at_edge);
  anchor.add("pvc_gnss_ns_at_edge", cap.pvc_gnss_ns_at_edge);
  anchor.add("dynamic_cps", cap.dynamic_cps);
  p.add_object("anchor", anchor);

  Payload event;
  event.add("target_counter32", cap.target_counter32);
  event.add("counter32_at_fire", cap.counter32_at_fire);
  event.add("counter32_residual_ticks", cap.counter32_residual_ticks);
  event.add("dwt_at_fire", cap.dwt_at_fire);
  p.add_object("event", event);

  Payload truth;
  truth.add("gnss_from_vclock_ns", cap.gnss_from_vclock_ns);
  p.add_object("truth", truth);

  Payload bridge;
  bridge.add("gnss_from_time_bridge_ns", cap.gnss_from_time_bridge_ns);
  bridge.add("residual_ns", cap.residual_ns);
  bridge.add_object("welford", ns_welford_payload(g_bridge_residual_welford));
  p.add_object("bridge", bridge);

  Payload dynamic;
  dynamic.add("gnss_from_dynamic_dwt_ns", cap.gnss_from_dynamic_dwt_ns);
  dynamic.add("residual_ns", cap.dynamic_residual_ns);
  dynamic.add_object("welford", ns_welford_payload(g_bridge_dynamic_residual_welford));
  p.add_object("dynamic", dynamic);

  return p;
}

// ============================================================================
// PPS_PHASE command
// ============================================================================

static Payload cmd_pps_phase(const Payload&) {
  if (g_witness_running && !g_bridge_armed && g_witness_runtime_ready) {
    (void)bridge_arm_next_sample();
  }

  const pps_phase_capture_t cap = g_phase_last;

  Payload p;
  p.add("model", "PPS_TO_SELECTED_VCLOCK_PIN_PHASE");
  p.add("valid", cap.valid);
  p.add("seq", cap.seq);
  p.add("samples", g_phase_samples);
  p.add("skipped_no_pps_raw", g_phase_skipped_no_pps_raw);
  p.add("skipped_sequence_mismatch", g_phase_skipped_sequence_mismatch);
  p.add("skipped_no_dynamic_cps", g_phase_skipped_no_dynamic_cps);
  p.add("no_modulo", true);

  Payload raw;
  raw.add("pps_sequence", cap.pps_sequence);
  raw.add("qtimer_sequence", cap.qtimer_sequence);
  raw.add("pps_isr_entry_dwt_raw", cap.pps_isr_entry_dwt_raw);
  raw.add("vclock_isr_entry_dwt_raw", cap.vclock_isr_entry_dwt_raw);
  raw.add("delta_cycles", cap.raw_notification_delta_cycles);
  raw.add("delta_ns", cycles_to_ns((double)cap.raw_notification_delta_cycles));
  raw.add_object("welford", welford_payload(g_phase_notification_welford));
  p.add_object("raw_notification", raw);

  Payload latency_model;
  latency_model.add("basis", "ROUND_TRIP_MINUS_STIMULATE");
  latency_model.add("stimulate_cycles", (uint32_t)WITNESS_STIMULATE_LATENCY);
  latency_model.add("stimulate_ns", cycles_to_ns((double)WITNESS_STIMULATE_LATENCY));
  latency_model.add("gpio_total_cycles", (uint32_t)GPIO_TOTAL_LATENCY);
  latency_model.add("gpio_total_ns", cycles_to_ns((double)GPIO_TOTAL_LATENCY));
  latency_model.add("qtimer_total_cycles", (uint32_t)QTIMER_TOTAL_LATENCY);
  latency_model.add("qtimer_total_ns", cycles_to_ns((double)QTIMER_TOTAL_LATENCY));
  latency_model.add("pps_pin_to_isr_cycles", cap.pps_pin_to_isr_cycles);
  latency_model.add("pps_pin_to_isr_ns", cycles_to_ns((double)cap.pps_pin_to_isr_cycles));
  latency_model.add("qtimer_pin_to_isr_cycles", cap.qtimer_pin_to_isr_cycles);
  latency_model.add("qtimer_pin_to_isr_ns", cycles_to_ns((double)cap.qtimer_pin_to_isr_cycles));
  p.add_object("pin_latency_model", latency_model);

  Payload selection;
  selection.add("pps_counter32_at_edge", cap.pps_counter32_at_edge);
  selection.add("pps_vclock_counter32_at_edge", cap.pps_vclock_counter32_at_edge);
  selection.add("target_counter32", cap.target_counter32);
  selection.add("counter32_at_fire", cap.counter32_at_fire);
  selection.add("selected_to_target_ticks", cap.selected_to_target_ticks);
  selection.add("selected_to_target_ns", cap.selected_to_target_ns);
  selection.add("expected_offset_cycles", cap.expected_offset_cycles);
  selection.add("expected_offset_ns", cycles_to_ns((double)cap.expected_offset_cycles));
  selection.add("dynamic_cps", cap.dynamic_cps);
  p.add_object("selection", selection);

  Payload inferred;
  inferred.add("phase_cycles", cap.selected_physical_phase_cycles);
  inferred.add("phase_ns", cycles_to_ns((double)cap.selected_physical_phase_cycles));
  inferred.add("phase_ticks", cap.selected_physical_phase_ticks);
  inferred.add("greater_than_one_vclock_tick", cap.greater_than_one_vclock_tick);
  inferred.add_object("welford", welford_payload(g_phase_selected_physical_welford));
  p.add_object("inferred_physical", inferred);

  return p;
}

// ============================================================================
// GPIO_DELAY command
// ============================================================================

static Payload cmd_gpio_delay(const Payload&) {
  capture_gpio_delay_sample();

  Payload p;
  p.add("model", "DIGITAL_WRITE_STIMULUS_DELAY");
  p.add("samples", g_gpio_delay_samples);
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);

  Payload last;
  last.add("dwt_before", g_gpio_delay_dwt_before);
  last.add("dwt_after", g_gpio_delay_dwt_after);
  last.add("cycles", g_gpio_delay_cycles);
  last.add("ns", cycles_to_ns((double)g_gpio_delay_cycles));
  p.add_object("last", last);

  p.add_object("welford", welford_payload(g_gpio_delay_welford));
  return p;
}

// ============================================================================
// DWT_READ command
// ============================================================================

static Payload cmd_dwt_read(const Payload&) {
  capture_dwt_read_sample();

  Payload p;
  p.add("model", "DWT_READ_COST");
  p.add("samples", g_dwt_read_samples);

  Payload last;
  last.add("dwt_before", g_dwt_read_before);
  last.add("dwt_after", g_dwt_read_after);
  last.add("cycles", g_dwt_read_cycles);
  last.add("ns", cycles_to_ns((double)g_dwt_read_cycles));
  p.add_object("last", last);

  p.add_object("welford", welford_payload(g_dwt_read_welford));
  return p;
}

// ============================================================================
// QTIMER_READ command
// ============================================================================

static Payload cmd_qtimer_read(const Payload&) {
  if (g_witness_running && !g_witness_schedule_armed && g_witness_runtime_ready) {
    witness_schedule_first_test();
  }

  Payload p;
  p.add("model", "QTIMER_READ_FROM_GPIO_ISR");
  p.add("samples", g_qtimer_read_samples);
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);

  Payload last;
  last.add("dwt_at_isr", g_qtimer_read_dwt_at_isr);
  last.add("dwt_before_counter_read", g_qtimer_read_dwt_before_counter_read);
  last.add("dwt_after_counter_read", g_qtimer_read_dwt_after_counter_read);
  last.add("counter32_at_read", g_qtimer_read_counter32_at_read);
  last.add("entry_to_read_start_cycles", g_qtimer_read_entry_to_read_start_cycles);
  last.add("entry_to_read_start_ns", cycles_to_ns((double)g_qtimer_read_entry_to_read_start_cycles));
  last.add("counter_read_cost_cycles", g_qtimer_read_counter_read_cost_cycles);
  last.add("counter_read_cost_ns", cycles_to_ns((double)g_qtimer_read_counter_read_cost_cycles));
  last.add("entry_to_read_end_cycles", g_qtimer_read_entry_to_read_end_cycles);
  last.add("entry_to_read_end_ns", cycles_to_ns((double)g_qtimer_read_entry_to_read_end_cycles));
  p.add_object("last", last);

  p.add_object("entry_to_read_start", welford_payload(g_qtimer_read_entry_to_read_start_welford));
  p.add_object("counter_read_cost", welford_payload(g_qtimer_read_counter_read_cost_welford));
  p.add_object("entry_to_read_end", welford_payload(g_qtimer_read_entry_to_read_end_welford));

  return p;
}

// ============================================================================
// ============================================================================
// ENTRY_LATENCY command
// ============================================================================

static Payload entry_latency_payload(const entry_latency_state_t& s) {
  Payload p;
  p.add("armed", s.armed);
  p.add("spin_active", s.spin_active);
  p.add("last_timeout", s.last_timeout);
  p.add("arm_count", s.arm_count);
  p.add("arm_failures", s.arm_failures);
  p.add("timeout_count", s.timeout_count);
  p.add("samples", s.sample_count);
  p.add("sequence", s.sequence);

  Payload last;
  last.add("landing_dwt", s.landing_dwt);
  last.add("shadow_dwt", s.shadow_dwt);
  last.add("shadow_at_isr", s.shadow_at_isr);
  last.add("approach_cycles", s.approach_cycles);
  last.add("approach_ns", cycles_to_ns((double)s.approach_cycles));
  last.add("isr_entry_dwt_raw", s.isr_entry_dwt_raw);
  last.add("entry_latency_cycles", s.entry_latency_cycles);
  last.add("entry_latency_ns", cycles_to_ns((double)s.entry_latency_cycles));
  p.add_object("last", last);

  p.add_object("welford", welford_payload(s.welford));
  return p;
}

static Payload cmd_entry_latency(const Payload&) {
  if (g_witness_running && g_witness_runtime_ready) {
    if (!g_entry_pps.armed && !g_entry_pps.spin_active) {
      (void)entry_latency_arm_pps_spin();
    }
    if (!g_bridge_armed) {
      (void)bridge_arm_next_sample();
    }
  }

  Payload p;
  p.add("model", "ISR_ENTRY_LATENCY_FROM_SPIN_SHADOW");
  p.add("lead_ns", ENTRY_LATENCY_LEAD_NS);
  p.add("timeout_cycles", ENTRY_LATENCY_TIMEOUT_CYCLES);
  p.add("timeout_ns", cycles_to_ns((double)ENTRY_LATENCY_TIMEOUT_CYCLES));
  p.add_object("pps", entry_latency_payload(g_entry_pps));
  p.add_object("qtimer", entry_latency_payload(g_entry_qtimer));
  return p;
}

// ============================================================================
// START / STOP / REPORT commands
// ============================================================================

static Payload cmd_start(const Payload&) {
  Payload p;
  if (!g_witness_runtime_ready) {
    p.add("status", "not_ready");
    p.add_object("state", witness_state_payload());
    return p;
  }

  const bool already_running = g_witness_running;
  const bool ok = witness_start_activity();
  p.add("status", ok ? (already_running ? "already_running" : "started") : "start_failed");
  p.add_object("state", witness_state_payload());
  return p;
}

static Payload cmd_stop(const Payload&) {
  const bool was_running = g_witness_running;
  witness_stop_activity();

  Payload p;
  p.add("status", was_running ? "stopped" : "already_stopped");
  p.add_object("state", witness_state_payload());
  return p;
}

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("model", "WITNESS_LIFECYCLE_AND_HARDWARE_STATE");
  p.add_object("state", witness_state_payload());
  return p;
}

// EDGE command
// ============================================================================
//
// EDGE answers: how many PPS edges have been seen, and what were the facts
// of the last edge?  process_interrupt remains the author of PPS/PPS_VCLOCK
// truth; process_witness owns only this report surface.

static Payload cmd_edge(const Payload&) {
  const interrupt_pps_edge_heartbeat_t hb = interrupt_pps_edge_heartbeat();
  const pps_t pps = interrupt_last_pps();
  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  const pps_edge_snapshot_t legacy = interrupt_last_pps_edge();

  Payload p;
  p.add("model", "PPS_EDGE_FACTS_FROM_INTERRUPT_AUTHORITY");
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);

  Payload heartbeat;
  heartbeat.add("edge_count", hb.edge_count);
  heartbeat.add("last_dwt", hb.last_dwt);
  heartbeat.add("last_gnss_ns", hb.last_gnss_ns);
  heartbeat.add("gpio_irq_count", hb.gpio_irq_count);
  heartbeat.add("gpio_miss_count", hb.gpio_miss_count);
  p.add_object("heartbeat", heartbeat);

  Payload pps_obj;
  pps_obj.add("sequence", pps.sequence);
  pps_obj.add("dwt_at_edge", pps.dwt_at_edge);
  pps_obj.add("counter32_at_edge", pps.counter32_at_edge);
  pps_obj.add("ch3_at_edge", (uint32_t)pps.ch3_at_edge);
  p.add_object("pps", pps_obj);

  Payload pvc_obj;
  pvc_obj.add("sequence", pvc.sequence);
  pvc_obj.add("dwt_at_edge", pvc.dwt_at_edge);
  pvc_obj.add("counter32_at_edge", pvc.counter32_at_edge);
  pvc_obj.add("ch3_at_edge", (uint32_t)pvc.ch3_at_edge);
  pvc_obj.add("gnss_ns_at_edge", pvc.gnss_ns_at_edge);
  p.add_object("pps_vclock", pvc_obj);

  Payload identity;
  identity.add("sequence_match", pps.sequence == pvc.sequence);
  identity.add("heartbeat_matches_pps", hb.edge_count == pps.sequence);
  identity.add("heartbeat_matches_pps_vclock", hb.edge_count == pvc.sequence);
  identity.add("legacy_sequence", legacy.sequence);
  identity.add("legacy_matches_pps_vclock", legacy.sequence == pvc.sequence);
  identity.add("vclock_epoch_selected", legacy.vclock_epoch_selected);
  identity.add("vclock_epoch_ticks_after_pps", legacy.vclock_epoch_ticks_after_pps);
  identity.add("vclock_epoch_counter32_offset_ticks", legacy.vclock_epoch_counter32_offset_ticks);
  identity.add("vclock_epoch_dwt_offset_cycles", legacy.vclock_epoch_dwt_offset_cycles);
  p.add_object("identity", identity);

  Payload legacy_obj;
  legacy_obj.add("dwt_at_edge", legacy.dwt_at_edge);
  legacy_obj.add("dwt_raw_at_edge", legacy.dwt_raw_at_edge);
  legacy_obj.add("counter32_at_edge", legacy.counter32_at_edge);
  legacy_obj.add("ch3_at_edge", (uint32_t)legacy.ch3_at_edge);
  legacy_obj.add("gnss_ns_at_edge", legacy.gnss_ns_at_edge);
  legacy_obj.add("physical_pps_dwt_raw_at_edge", legacy.physical_pps_dwt_raw_at_edge);
  legacy_obj.add("physical_pps_dwt_normalized_at_edge", legacy.physical_pps_dwt_normalized_at_edge);
  legacy_obj.add("physical_pps_counter32_at_read", legacy.physical_pps_counter32_at_read);
  legacy_obj.add("physical_pps_ch3_at_read", (uint32_t)legacy.physical_pps_ch3_at_read);
  legacy_obj.add("vclock_epoch_counter32", legacy.vclock_epoch_counter32);
  legacy_obj.add("vclock_epoch_ch3", (uint32_t)legacy.vclock_epoch_ch3);
  p.add_object("legacy_projection", legacy_obj);

  return p;
}

// ============================================================================
// ROUND_TRIP command
// ============================================================================

static Payload cmd_round_trip(const Payload&) {
  if (g_witness_running && !g_witness_schedule_armed && g_witness_runtime_ready) {
    witness_schedule_first_test();
  }

  Payload p;
  p.add("model", "ROUND_TRIP_LATENCY_SUMMARY");

  Payload stimulate;
  stimulate.add("last_cycles", g_source_stim_cycles);
  stimulate.add("last_ns", cycles_to_ns((double)g_source_stim_cycles));
  stimulate.add_object("welford", welford_payload(g_source_welford));
  p.add_object("stimulate", stimulate);

  Payload gpio;
  gpio.add("last_cycles", g_gpio_delta_cycles);
  gpio.add("last_ns", cycles_to_ns((double)g_gpio_delta_cycles));
  gpio.add_object("welford", welford_payload(g_gpio_welford));
  p.add_object("gpio", gpio);

  Payload qtimer;
  qtimer.add("last_cycles", g_qtimer_delta_cycles);
  qtimer.add("last_ns", cycles_to_ns((double)g_qtimer_delta_cycles));
  qtimer.add_object("welford", welford_payload(g_qtimer_welford));
  p.add_object("qtimer", qtimer);

  return p;
}

// ============================================================================
// Init / IRQ enable / registration
// ============================================================================

void process_witness_init_hardware(void) {
  if (g_witness_hw_ready) return;

  pinMode(WITNESS_STIMULUS_PIN, OUTPUT);
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  pinMode(WITNESS_GPIO_PIN, INPUT);

  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);

  *(portConfigRegister(WITNESS_QTIMER_PIN)) = 1;
  IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;

  IMXRT_TMR2.CH[0].CTRL   = 0;
  IMXRT_TMR2.CH[0].SCTRL  = 0;
  IMXRT_TMR2.CH[0].CSCTRL = 0;
  IMXRT_TMR2.CH[0].LOAD   = 0;
  IMXRT_TMR2.CH[0].CNTR   = 0;
  IMXRT_TMR2.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR2.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR2.CH[0].CMPLD2 = 0;
  IMXRT_TMR2.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  qtimer2_ch0_disable_compare();

  // Old process_interrupt asserted all four bits while shaking out the path.
  // Preserve that behavior here rather than trying to be clever with CH0 only.
  IMXRT_TMR2.ENBL |= (uint16_t)0x000F;

  g_witness_hw_ready = true;
}

void process_witness_init(void) {
  if (g_witness_runtime_ready) return;

  g_source_high = false;
  g_active_window = witness_window_t::NONE;
  g_witness_cycle_count = 0;
  g_witness_cycle_reschedules = 0;

  g_source_emits = 0;
  g_source_lows = 0;
  g_source_dwt_at_emit = 0;
  g_source_dwt_before = 0;
  g_source_dwt_after = 0;
  g_source_stim_cycles = 0;

  g_gpio_hits = 0;
  g_gpio_dwt_at_isr = 0;
  g_gpio_delta_cycles = 0;
  g_gpio_wrong_window = 0;

  g_qtimer_hits = 0;
  g_qtimer_irq_entries = 0;
  g_qtimer_no_flag = 0;
  g_qtimer_outside_window = 0;
  g_qtimer_arms = 0;
  g_qtimer_dwt_at_isr = 0;
  g_qtimer_delta_cycles = 0;
  g_qtimer_compare_target = 0;

  welford_reset(g_source_welford);
  welford_reset(g_gpio_welford);
  welford_reset(g_qtimer_welford);

  g_qtimer_read_samples = 0;
  g_qtimer_read_dwt_at_isr = 0;
  g_qtimer_read_dwt_before_counter_read = 0;
  g_qtimer_read_dwt_after_counter_read = 0;
  g_qtimer_read_counter32_at_read = 0;
  g_qtimer_read_entry_to_read_start_cycles = 0;
  g_qtimer_read_counter_read_cost_cycles = 0;
  g_qtimer_read_entry_to_read_end_cycles = 0;
  welford_reset(g_qtimer_read_entry_to_read_start_welford);
  welford_reset(g_qtimer_read_counter_read_cost_welford);
  welford_reset(g_qtimer_read_entry_to_read_end_welford);


  g_gpio_delay_samples = 0;
  g_gpio_delay_dwt_before = 0;
  g_gpio_delay_dwt_after = 0;
  g_gpio_delay_cycles = 0;
  welford_reset(g_gpio_delay_welford);

  g_dwt_read_samples = 0;
  g_dwt_read_before = 0;
  g_dwt_read_after = 0;
  g_dwt_read_cycles = 0;
  welford_reset(g_dwt_read_welford);

  entry_latency_reset(g_entry_pps);
  entry_latency_reset(g_entry_qtimer);

  g_phase_last = pps_phase_capture_t{};
  g_phase_capture_seq = 0;
  g_phase_pps_sequence = 0;
  g_phase_pps_isr_entry_dwt_raw = 0;
  g_phase_samples = 0;
  g_phase_skipped_no_pps_raw = 0;
  g_phase_skipped_sequence_mismatch = 0;
  g_phase_skipped_no_dynamic_cps = 0;
  welford_reset(g_phase_notification_welford);
  welford_reset(g_phase_selected_physical_welford);

  g_bridge_last = bridge_capture_t{};
  g_bridge_capture_seq = 0;
  g_bridge_fires_total = 0;
  g_bridge_arm_count = 0;
  g_bridge_arm_failures = 0;
  g_bridge_skipped_no_anchor = 0;
  g_bridge_skipped_no_dynamic_cps = 0;
  g_bridge_last_armed_target_counter32 = 0;
  g_bridge_armed = false;
  welford_reset(g_bridge_residual_welford);
  welford_reset(g_bridge_dynamic_residual_welford);

  // Witness is registered at boot but remains operationally quiet until
  // WITNESS.START.  The hot-path hooks and timer rails are installed/armed
  // only while running, then removed again by WITNESS.STOP.
  interrupt_register_qtimer1_ch1_handler(nullptr);
  interrupt_register_pps_entry_latency_handler(nullptr);

  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  // Stable QTimer rail: configured once, but compare interrupts remain
  // disabled until START.  GPIO is attached only inside the GPIO measurement
  // window while witness is running.
  pinMode(WITNESS_GPIO_PIN, INPUT);
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  qtimer2_ch0_disable_compare();

  g_witness_running = false;
  g_witness_schedule_armed = false;
  g_witness_runtime_ready = true;
}

void process_witness_enable_irqs(void) {
  if (g_witness_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);

  g_witness_irqs_enabled = true;
}

static const process_command_entry_t WITNESS_COMMANDS[] = {
  { "START",       cmd_start       },
  { "STOP",        cmd_stop        },
  { "REPORT",      cmd_report      },
  { "EDGE",        cmd_edge        },
  { "BRIDGE",      cmd_bridge      },
  { "PPS_PHASE",   cmd_pps_phase   },
  { "GPIO_DELAY",  cmd_gpio_delay  },
  { "QTIMER_READ", cmd_qtimer_read },
  { "DWT_READ",    cmd_dwt_read    },
  { "ENTRY_LATENCY", cmd_entry_latency },
  { "ROUND_TRIP",  cmd_round_trip  },
  { nullptr,      nullptr        }
};

static const process_vtable_t WITNESS_PROCESS = {
  .process_id    = "WITNESS",
  .commands      = WITNESS_COMMANDS,
  .subscriptions = nullptr
};

void process_witness_register(void) {
  process_register("WITNESS", &WITNESS_PROCESS);
}
