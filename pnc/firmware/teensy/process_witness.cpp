// ============================================================================
// process_witness.cpp
// ============================================================================
//
// process_witness owns local timing witness instrumentation.  The local
// hardware ROUND_TRIP rail remains intentionally low-level:
//
//   • Stimulus source pin  — TimePop callback drives HIGH/LOW.
//   • GPIO sink            — GPIO interrupt sees the source rising edge.
//
// Operationally, QTimer2/QTimer3 remain owned by process_interrupt.  This
// file normally preserves only the GPIO round-trip path and TimePop-based
// bridge/phase reports.
//
// Special CLOCK_WITNESS modes are intentionally different:
//
//   • OCXO_START/OCXO_STOP let witness temporarily take over the same OCXO
//     hardware lanes used by process_interrupt after process_interrupt has
//     been explicitly defeated.
//
//   • VCLOCK_START/VCLOCK_STOP use process_interrupt's hosted QTimer1 CH1
//     compare service to ring a side bell every 10,000,000 VCLOCK ticks
//     without taking over the sovereign QTimer1 CH2/TimePop rail.
//
// The shared experiment is minimal edge-to-edge timing: capture DWT at each
// 10,000,000-count clock boundary and publish historical one-second interval
// facts from scheduled context on the CLOCK_WITNESS topic.
//
// The PPS/VCLOCK bridge and phase reports remain ordinary TimePop scheduled
// clients.  TimePop authors one shared fire fact for each scheduled event:
// fire_vclock_raw, fire_dwt_cyccnt, and fire_gnss_ns.  Witness consumes those
// facts exactly like any other TimePop client.
//
// Witness is always-on after initialization.  There are no global START/STOP
// commands; the process is safe to run continuously because the operational
// bridge and phase reports are TimePop clients on the canonical PPS/VCLOCK
// timing rail.  OCXO_START/OCXO_STOP and VCLOCK_START/VCLOCK_STOP are narrow
// test-mode commands only.
//
// Current command surface:
//   REPORT        — lifecycle/hardware/TimePop state.
//   EDGE          — PPS/PPS_VCLOCK heartbeat + last edge facts.
//   BRIDGE        — DWT/GNSS bridge check using TimePop fire facts.
//   GPIO_DELAY    — simple digitalWriteFast HIGH stimulus cost.
//   QTIMER_READ   — GPIO ISR-to-VCLOCK counter observation delay/cost.
//   DWT_READ      — consecutive DWT read-cost measurement.
//   ENTRY_LATENCY — spin-shadow ISR entry latency for PPS.
//   PPS_PHASE     — PPS/VCLOCK phase inferred from TimePop witness events.
//   ROUND_TRIP    — source-to-GPIO ISR latency stats.
//   OCXO_START    — begin minimal OCXO edge-to-edge DWT witness mode.
//   OCXO_STOP     — stop OCXO witness mode and disable its compares.
//   VCLOCK_START  — begin hosted QTimer1 CH1 VCLOCK edge witness mode.
//   VCLOCK_STOP   — stop hosted VCLOCK witness mode.
//
// Test cadence:
//   • 100 ms: GPIO HIGH   (visible 1 Hz heartbeat; away from PPS)
//   • 600 ms: GPIO LOW    (500 ms high duty interval)
//
// The waveform slots are armed as one-shots once per scheduler cycle.  Do not
// make HIGH/LOW independent 1 Hz recurring timers: TimePop intentionally
// re-authors same-period recurring slots onto the same PPS/VCLOCK phase grid
// after epoch changes, which collapses a heartbeat into a zero-width blip.
//
// ============================================================================

#include "process_witness.h"

#include "process_interrupt.h"

#include "config.h"
#include "process.h"
#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "time.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <string.h>
#include <strings.h>

// ============================================================================
// Hardware constants
// ============================================================================

static constexpr int WITNESS_STIMULUS_PIN = 24;
static constexpr int WITNESS_GPIO_PIN     = 26;

static constexpr uint64_t GPIO_HIGH_OFFSET_NS   = 100000000ULL;
static constexpr uint64_t GPIO_LOW_OFFSET_NS    = 600000000ULL;
static constexpr uint64_t WITNESS_CYCLE_PERIOD_NS = 1000000000ULL;

static constexpr const char* GPIO_HIGH_NAME   = "WITNESS_GPIO_HIGH";
static constexpr const char* GPIO_LOW_NAME    = "WITNESS_GPIO_LOW";
static constexpr const char* WITNESS_CYCLE_NAME = "WITNESS_CYCLE";
static constexpr const char* WITNESS_BRIDGE_NAME = "WITNESS_BRIDGE";
static constexpr const char* WITNESS_SCHEDULER_NAME = "WITNESS_SCHEDULER";

static constexpr uint64_t ENTRY_LATENCY_LEAD_NS = 5000ULL;
static constexpr uint32_t ENTRY_LATENCY_TIMEOUT_CYCLES = 100000U;  // ~99 us @ 1.008 GHz
static constexpr const char* ENTRY_PPS_SPIN_NAME = "ENTRY_LATENCY_PPS_SPIN";

static constexpr uint32_t VCLOCK_TICKS_PER_SECOND_U32 = 10000000U;

static pps_t witness_last_physical_pps_diag(void) {
  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();

  pps_t pps{};
  pps.sequence          = snap.sequence;
  pps.dwt_at_edge       = snap.physical_pps_dwt_normalized_at_edge;
  pps.counter32_at_edge = snap.physical_pps_counter32_at_read;
  pps.ch3_at_edge       = snap.physical_pps_ch3_at_read;

  return pps;
}

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

enum class witness_window_t : uint8_t { NONE = 0, GPIO };
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

static welford_t g_source_welford = {};
static welford_t g_gpio_welford = {};

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

// ============================================================================
// PPS_PHASE — TimePop-authored PPS/VCLOCK phase witness
// ============================================================================
//
// PPS_PHASE now rides on the TimePop BRIDGE scheduled client.  There is no
// QTimer1 CH1 compare, no CH1 ISR raw notification, and no private hardware
// rail.  The TimePop callback supplies an authored fire fact; witness walks
// that fact backward by the VCLOCK tick offset to infer the selected
// PPS/VCLOCK DWT coordinate and compares it with the canonical PPS/VCLOCK
// anchor from process_interrupt.

struct pps_phase_capture_t {
  uint32_t seq = 0;
  bool     valid = false;

  uint32_t pps_sequence = 0;
  uint32_t pvc_sequence = 0;

  uint32_t pps_dwt_at_edge = 0;
  uint32_t pvc_dwt_at_edge = 0;
  uint32_t pvc_counter32_at_edge = 0;
  int64_t  pvc_gnss_ns_at_edge = -1;

  uint32_t timepop_deadline = 0;
  uint32_t timepop_fire_vclock_raw = 0;
  uint32_t timepop_fire_dwt = 0;
  int64_t  timepop_fire_gnss_ns = -1;

  uint32_t selected_to_fire_ticks = 0;
  uint64_t selected_to_fire_ns = 0;

  uint32_t dynamic_cps = 0;
  uint32_t expected_offset_cycles = 0;
  uint32_t inferred_pvc_dwt_from_timepop = 0;

  int32_t  timepop_vs_pvc_phase_cycles = 0;
  double   timepop_vs_pvc_phase_ticks = 0.0;
  int32_t  physical_pps_to_selected_vclock_cycles = 0;
  double   physical_pps_to_selected_vclock_ticks = 0.0;
};

static pps_phase_capture_t g_phase_last = {};
static volatile uint32_t g_phase_capture_seq = 0;
static volatile uint32_t g_phase_pps_sequence = 0;
static volatile uint32_t g_phase_pps_isr_entry_dwt_raw = 0;
static volatile uint32_t g_phase_samples = 0;
static volatile uint32_t g_phase_skipped_no_pps_raw = 0;
static volatile uint32_t g_phase_skipped_sequence_mismatch = 0;
static volatile uint32_t g_phase_skipped_no_dynamic_cps = 0;
static welford_t g_phase_timepop_vs_pvc_welford = {};
static welford_t g_phase_physical_pps_to_selected_welford = {};

// ============================================================================
// BRIDGE — DWT/GNSS bridge validation against TimePop/VCLOCK truth
// ============================================================================
//
// One fixed sub-second TimePop client per PPS/VCLOCK second.  TimePop supplies
// the truth tuple: fire_vclock_raw, fire_dwt_cyccnt, fire_gnss_ns.  The test
// asks whether time_gnss_ns_at_dwt(fire_dwt_cyccnt) returns the same GNSS
// coordinate.

static constexpr uint8_t  BRIDGE_SLOT_INDEX = 5;
static constexpr uint64_t BRIDGE_SLOT_OFFSET_NS = 500000000ULL;

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
// CLOCK_WITNESS — minimal edge-to-edge clock/DWT experiment
// ============================================================================
//
// This mode is off by default and must only be used after process_interrupt has
// relinquished the OCXO rails.  It mirrors the current process_interrupt OCXO
// hardware mapping:
//
//   OCXO1: QTimer2 CH0, pin OCXO1_PIN, PCS(0)
//   OCXO2: QTimer3 CH3, pin OCXO2_PIN, PCS(3)
//
// QTimer compare ISR work is deliberately tiny: clear/disable compare, latch
// first-instruction DWT + counter low16, request a safe TimePop ASAP callback.
// The ASAP callback authors edge facts.  A 1 kHz VCLOCK/TimePop cadence minder
// extends the 16-bit counters, arms one-shot compares only when the 10,000,000
// count boundary is near, and requests a scheduled-context publication once per
// VCLOCK second.  Published CLOCK_WITNESS records are historical snapshots of
// already-captured clock intervals.

static constexpr uint32_t OCXO_WITNESS_COUNTS_PER_SECOND = 10000000U;
static constexpr uint32_t OCXO_WITNESS_MINDER_PERIOD_NS = 1000000U;  // 1 ms
static constexpr uint32_t OCXO_WITNESS_MINDER_PER_REPORT = 1000U;
static constexpr uint32_t OCXO_WITNESS_ARM_WINDOW_TICKS = 50000U;
static constexpr uint16_t OCXO_WITNESS_LARGE_DELTA16_TICKS = 50000U;
static constexpr const char* OCXO_WITNESS_MINDER_NAME = "CLOCK_WITNESS_MINDER";
static constexpr const char* OCXO_WITNESS_EDGE_ASAP_NAME = "CLOCK_WITNESS_EDGE_ASAP";
static constexpr const char* OCXO_WITNESS_PUBLISH_ASAP_NAME = "CLOCK_WITNESS_PUBLISH_ASAP";

static constexpr uint8_t OCXO_WITNESS_LANE1_MASK = 0x01;
static constexpr uint8_t OCXO_WITNESS_LANE2_MASK = 0x02;
static constexpr uint8_t OCXO_WITNESS_BOTH_MASK =
    OCXO_WITNESS_LANE1_MASK | OCXO_WITNESS_LANE2_MASK;

static constexpr const char* CLOCK_WITNESS_TOPIC = "CLOCK_WITNESS";

// VCLOCK side-bell target starts at a non-1ms phase so CH1 does not
// intentionally collide with the TimePop/CH2 1ms cadence grid.
static constexpr uint32_t VCLOCK_WITNESS_FIRST_TARGET_DELAY_TICKS = 5005000U;
static constexpr uint32_t VCLOCK_WITNESS_ARM_WINDOW_TICKS = 45000U;
static constexpr uint32_t VCLOCK_WITNESS_MIN_ARM_LEAD_TICKS = 64U;

struct ocxo_witness_edge_record_t {
  bool     valid = false;
  bool     delta_valid = false;
  bool     residual_valid = false;

  uint32_t sequence = 0;
  uint32_t irq_count = 0;

  uint64_t ocxo_total_at_edge = 0;
  uint64_t ocxo_total_at_isr = 0;
  uint64_t dwt_total_at_edge = 0;

  uint32_t dwt_at_edge = 0;
  uint32_t dwt_delta_cycles = 0;
  uint32_t dwt_pred_cycles = 0;
  int32_t  dwt_residual_cycles = 0;

  uint16_t target_low16 = 0;
  uint16_t counter_low16_at_isr = 0;
  int32_t  service_offset_ticks = 0;
};

struct ocxo_witness_lane_t {
  const char*  name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;
  uint8_t      pcs = 0;
  int          input_pin = -1;
  uint8_t      mask = 0;

  volatile bool enabled = false;
  volatile bool initialized = false;
  volatile bool compare_armed = false;
  volatile bool irq_pending = false;

  volatile uint32_t irq_count = 0;
  volatile uint32_t false_irq_count = 0;
  volatile uint32_t pending_overwrite_count = 0;
  volatile uint32_t asap_arm_failures = 0;
  volatile uint32_t pending_dwt_at_isr = 0;
  volatile uint16_t pending_counter_low16_at_isr = 0;
  volatile uint16_t pending_target_low16 = 0;

  uint16_t origin_low16 = 0;
  uint16_t last_hw16 = 0;
  uint64_t total_cycles = 0;
  uint64_t next_target_total = OCXO_WITNESS_COUNTS_PER_SECOND;
  uint16_t next_target_low16 = 0;
  uint64_t dwt_total_cycles = 0;

  bool     seen_first_edge = false;
  bool     have_last_delta = false;
  uint32_t last_edge_dwt32 = 0;
  uint64_t last_edge_dwt64 = 0;
  uint32_t last_dwt_delta_cycles = 0;

  ocxo_witness_edge_record_t previous = {};
  ocxo_witness_edge_record_t last = {};

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t edge_count = 0;
  uint32_t arm_count = 0;
  uint32_t arm_late_count = 0;
  uint32_t missed_target_count = 0;
  uint32_t rollover_update_count = 0;
  uint32_t large_delta_count = 0;
  uint16_t largest_delta16 = 0;
  uint32_t last_published_edge_count = 0;

  welford_t dwt_delta_welford = {};
  welford_t residual_welford = {};
};

static ocxo_witness_lane_t g_ocxo_witness_1 = {};
static ocxo_witness_lane_t g_ocxo_witness_2 = {};

static volatile bool g_ocxo_witness_running = false;
static volatile uint8_t g_ocxo_witness_active_mask = 0;
static volatile bool g_ocxo_witness_minder_armed = false;
static volatile bool g_ocxo_witness_edge_asap_requested = false;
static volatile bool g_ocxo_witness_publish_asap_requested = false;
static volatile uint8_t g_ocxo_witness_vector_mask = 0;
static volatile uint32_t g_ocxo_witness_start_count = 0;
static volatile uint32_t g_ocxo_witness_stop_count = 0;
static volatile uint32_t g_ocxo_witness_minder_fire_count = 0;
static volatile uint32_t g_ocxo_witness_minder_arm_count = 0;
static volatile uint32_t g_ocxo_witness_minder_arm_failures = 0;
static volatile uint32_t g_ocxo_witness_minder_divider = 0;
static volatile uint32_t g_ocxo_witness_publish_request_count = 0;
static volatile uint32_t g_ocxo_witness_publish_count = 0;
static volatile uint32_t g_ocxo_witness_publish_asap_failures = 0;
static volatile uint32_t g_ocxo_witness_edge_asap_count = 0;

// ============================================================================
// VCLOCK witness — hosted QTimer1 CH1 edge-to-edge VCLOCK/DWT experiment
// ============================================================================
//
// This does not take ownership of VCLOCK and does not touch QTimer1 registers.
// process_interrupt owns QTimer1 CH1 as a hosted VCLOCK-domain compare rail.
// Witness registers a tiny IRQ-context handler, arms a near one-shot target
// only when the next 10,000,000-count boundary is close, and folds the
// historical interval facts into the same CLOCK_WITNESS stream used by the
// OCXO witness lanes.

struct vclock_witness_edge_record_t {
  bool     valid = false;
  bool     delta_valid = false;
  bool     residual_valid = false;

  uint32_t sequence = 0;
  uint32_t irq_count = 0;

  uint64_t clock_total_at_edge = 0;
  uint64_t clock_total_at_event = 0;
  uint64_t dwt_total_at_edge = 0;

  uint32_t target_counter32 = 0;
  uint32_t counter32_at_event = 0;
  int32_t  counter32_residual_ticks = 0;

  uint32_t isr_entry_dwt_raw = 0;
  uint32_t dwt_at_edge = 0;
  int64_t  gnss_ns_at_event = -1;

  uint32_t dwt_delta_cycles = 0;
  uint32_t dwt_pred_cycles = 0;
  int32_t  dwt_residual_cycles = 0;

  int32_t  service_offset_ticks = 0;
};

struct vclock_witness_lane_t {
  volatile bool enabled = false;
  volatile bool initialized = false;
  volatile bool compare_armed = false;
  volatile bool irq_pending = false;
  volatile bool handler_registered = false;

  volatile uint32_t irq_count = 0;
  volatile uint32_t false_irq_count = 0;
  volatile uint32_t pending_overwrite_count = 0;
  volatile uint32_t asap_arm_failures = 0;

  volatile uint32_t pending_sequence = 0;
  volatile uint32_t pending_target_counter32 = 0;
  volatile uint32_t pending_counter32_at_event = 0;
  volatile int32_t  pending_counter32_residual_ticks = 0;
  volatile uint32_t pending_isr_entry_dwt_raw = 0;
  volatile uint32_t pending_dwt_at_event = 0;
  volatile int64_t  pending_gnss_ns_at_event = -1;

  uint32_t origin_counter32 = 0;
  uint32_t next_target_counter32 = 0;
  uint64_t next_target_total = OCXO_WITNESS_COUNTS_PER_SECOND;
  uint32_t first_target_delay_ticks = VCLOCK_WITNESS_FIRST_TARGET_DELAY_TICKS;
  uint64_t dwt_total_cycles = 0;

  bool     seen_first_edge = false;
  bool     have_last_delta = false;
  uint32_t last_edge_dwt32 = 0;
  uint64_t last_edge_dwt64 = 0;
  uint32_t last_dwt_delta_cycles = 0;

  vclock_witness_edge_record_t previous = {};
  vclock_witness_edge_record_t last = {};

  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t edge_count = 0;
  uint32_t arm_count = 0;
  uint32_t arm_failures = 0;
  uint32_t arm_late_count = 0;
  uint32_t missed_target_count = 0;
  uint32_t tend_count = 0;
  uint32_t last_arm_remaining_ticks = 0;
  uint32_t last_arm_now_counter32 = 0;
  uint32_t last_published_edge_count = 0;

  welford_t dwt_delta_welford = {};
  welford_t residual_welford = {};
};

static vclock_witness_lane_t g_vclock_witness = {};

// ============================================================================
// Forward declarations
// ============================================================================

void witness_gpio_isr(void);

static void gpio_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_scheduler_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_bridge_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_isr_handler(uint32_t sequence, uint32_t isr_entry_dwt_raw);
static bool entry_latency_arm_pps_spin(void);
static bool witness_ensure_scheduled(void);
static Payload witness_state_payload(void);
static Payload welford_payload(const welford_t& w);
static void ocxo_witness_minder_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void ocxo_witness_edge_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void ocxo_witness_publish_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void vclock_witness_ch1_handler(const interrupt_qtimer1_ch1_compare_event_t& event);
static void vclock_witness_process_pending(void);
static Payload cmd_ocxo_start(const Payload& args);
static Payload cmd_ocxo_stop(const Payload& args);
static Payload cmd_vclock_start(const Payload& args);
static Payload cmd_vclock_stop(const Payload& args);

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
// OCXO_WITNESS helpers
// ============================================================================

static inline int32_t clamp_i64_to_i32(int64_t v) {
  if (v > INT32_MAX) return INT32_MAX;
  if (v < INT32_MIN) return INT32_MIN;
  return (int32_t)v;
}

static bool clock_witness_any_active(void) {
  return g_ocxo_witness_1.enabled || g_ocxo_witness_2.enabled ||
         g_vclock_witness.enabled;
}

static void clock_witness_cancel_deferred_if_idle(void) {
  if (clock_witness_any_active()) return;

  timepop_cancel_by_name(OCXO_WITNESS_MINDER_NAME);
  timepop_cancel_by_name(OCXO_WITNESS_EDGE_ASAP_NAME);
  timepop_cancel_by_name(OCXO_WITNESS_PUBLISH_ASAP_NAME);
  g_ocxo_witness_running = false;
  g_ocxo_witness_minder_armed = false;
  g_ocxo_witness_edge_asap_requested = false;
  g_ocxo_witness_publish_asap_requested = false;
}

static void vclock_witness_reset_runtime(void) {
  const uint32_t prior_start_count = g_vclock_witness.start_count;
  const uint32_t prior_stop_count = g_vclock_witness.stop_count;

  g_vclock_witness = vclock_witness_lane_t{};
  g_vclock_witness.start_count = prior_start_count;
  g_vclock_witness.stop_count = prior_stop_count;
  welford_reset(g_vclock_witness.dwt_delta_welford);
  welford_reset(g_vclock_witness.residual_welford);
}

static void vclock_witness_advance_target_to_future(uint32_t now_counter32) {
  while ((uint32_t)(now_counter32 - g_vclock_witness.next_target_counter32) <
         0x80000000UL) {
    g_vclock_witness.next_target_counter32 += OCXO_WITNESS_COUNTS_PER_SECOND;
    g_vclock_witness.next_target_total += OCXO_WITNESS_COUNTS_PER_SECOND;
    g_vclock_witness.missed_target_count++;
  }
}

static void vclock_witness_tend(void) {
  if (!g_vclock_witness.enabled || !g_vclock_witness.initialized) return;
  if (g_vclock_witness.irq_pending || g_vclock_witness.compare_armed) return;

  const uint32_t now = interrupt_vclock_counter32_observe_ambient();
  g_vclock_witness.tend_count++;

  uint32_t remaining = g_vclock_witness.next_target_counter32 - now;
  if (remaining > 0x7FFFFFFFUL) {
    vclock_witness_advance_target_to_future(now);
    return;
  }

  if (remaining <= VCLOCK_WITNESS_MIN_ARM_LEAD_TICKS) {
    g_vclock_witness.arm_late_count++;
    return;
  }

  if (remaining > VCLOCK_WITNESS_ARM_WINDOW_TICKS) {
    return;
  }

  g_vclock_witness.last_arm_now_counter32 = now;
  g_vclock_witness.last_arm_remaining_ticks = remaining;
  if (interrupt_qtimer1_ch1_arm_compare(g_vclock_witness.next_target_counter32)) {
    g_vclock_witness.compare_armed = true;
    g_vclock_witness.arm_count++;
  } else {
    g_vclock_witness.arm_failures++;
  }
}

static void vclock_witness_complete_edge(uint32_t sequence,
                                         uint32_t target_counter32,
                                         uint32_t counter32_at_event,
                                         int32_t counter32_residual_ticks,
                                         uint32_t isr_entry_dwt_raw,
                                         uint32_t dwt_at_event,
                                         int64_t gnss_ns_at_event) {
  if (!g_vclock_witness.enabled) return;

  const uint64_t target_total = g_vclock_witness.next_target_total;
  const int32_t service_offset = counter32_residual_ticks;

  vclock_witness_edge_record_t rec{};
  rec.valid = true;
  rec.sequence = g_vclock_witness.edge_count + 1;
  rec.irq_count = g_vclock_witness.irq_count;
  rec.clock_total_at_edge = target_total;
  rec.clock_total_at_event =
      (service_offset >= 0)
          ? (target_total + (uint64_t)service_offset)
          : (target_total - (uint64_t)(-service_offset));
  rec.target_counter32 = target_counter32;
  rec.counter32_at_event = counter32_at_event;
  rec.counter32_residual_ticks = counter32_residual_ticks;
  rec.isr_entry_dwt_raw = isr_entry_dwt_raw;
  rec.dwt_at_edge = dwt_at_event;
  rec.gnss_ns_at_event = gnss_ns_at_event;
  rec.service_offset_ticks = service_offset;

  if (g_vclock_witness.seen_first_edge) {
    const uint32_t dwt_delta = (uint32_t)(dwt_at_event - g_vclock_witness.last_edge_dwt32);
    g_vclock_witness.dwt_total_cycles += dwt_delta;
    g_vclock_witness.last_edge_dwt64 = g_vclock_witness.dwt_total_cycles;

    rec.delta_valid = true;
    rec.dwt_total_at_edge = g_vclock_witness.dwt_total_cycles;
    rec.dwt_delta_cycles = dwt_delta;
    rec.dwt_pred_cycles = g_vclock_witness.have_last_delta
        ? g_vclock_witness.last_dwt_delta_cycles
        : 0;

    welford_update(g_vclock_witness.dwt_delta_welford, (int32_t)dwt_delta);

    if (g_vclock_witness.have_last_delta) {
      rec.residual_valid = true;
      rec.dwt_residual_cycles =
          (int32_t)((int64_t)dwt_delta -
                    (int64_t)g_vclock_witness.last_dwt_delta_cycles);
      welford_update(g_vclock_witness.residual_welford,
                     rec.dwt_residual_cycles);
    }

    g_vclock_witness.last_dwt_delta_cycles = dwt_delta;
    g_vclock_witness.have_last_delta = true;
  } else {
    g_vclock_witness.seen_first_edge = true;
    g_vclock_witness.dwt_total_cycles = 0;
    g_vclock_witness.last_edge_dwt64 = 0;
    rec.dwt_total_at_edge = 0;
  }

  g_vclock_witness.last_edge_dwt32 = dwt_at_event;
  g_vclock_witness.previous = g_vclock_witness.last;
  g_vclock_witness.last = rec;
  g_vclock_witness.edge_count++;

  g_vclock_witness.next_target_counter32 =
      target_counter32 + OCXO_WITNESS_COUNTS_PER_SECOND;
  g_vclock_witness.next_target_total =
      target_total + OCXO_WITNESS_COUNTS_PER_SECOND;
  g_vclock_witness.compare_armed = false;
}

static void vclock_witness_process_pending(void) {
  bool pending;
  uint32_t sequence;
  uint32_t target_counter32;
  uint32_t counter32_at_event;
  int32_t counter32_residual_ticks;
  uint32_t isr_entry_dwt_raw;
  uint32_t dwt_at_event;
  int64_t gnss_ns_at_event;

  noInterrupts();
  pending = g_vclock_witness.irq_pending;
  sequence = g_vclock_witness.pending_sequence;
  target_counter32 = g_vclock_witness.pending_target_counter32;
  counter32_at_event = g_vclock_witness.pending_counter32_at_event;
  counter32_residual_ticks = g_vclock_witness.pending_counter32_residual_ticks;
  isr_entry_dwt_raw = g_vclock_witness.pending_isr_entry_dwt_raw;
  dwt_at_event = g_vclock_witness.pending_dwt_at_event;
  gnss_ns_at_event = g_vclock_witness.pending_gnss_ns_at_event;
  g_vclock_witness.irq_pending = false;
  interrupts();

  if (!pending) return;
  vclock_witness_complete_edge(sequence,
                               target_counter32,
                               counter32_at_event,
                               counter32_residual_ticks,
                               isr_entry_dwt_raw,
                               dwt_at_event,
                               gnss_ns_at_event);
}

static void vclock_witness_ch1_handler(const interrupt_qtimer1_ch1_compare_event_t& event) {
  if (!g_vclock_witness.enabled || !g_vclock_witness.compare_armed) {
    g_vclock_witness.false_irq_count++;
    return;
  }

  if (g_vclock_witness.irq_pending) g_vclock_witness.pending_overwrite_count++;

  g_vclock_witness.pending_sequence = event.sequence;
  g_vclock_witness.pending_target_counter32 = event.target_counter32;
  g_vclock_witness.pending_counter32_at_event = event.counter32_at_event;
  g_vclock_witness.pending_counter32_residual_ticks = event.counter32_residual_ticks;
  g_vclock_witness.pending_isr_entry_dwt_raw = event.isr_entry_dwt_raw;
  g_vclock_witness.pending_dwt_at_event = event.dwt_at_event;
  g_vclock_witness.pending_gnss_ns_at_event = event.gnss_ns_at_event;
  g_vclock_witness.irq_pending = true;
  g_vclock_witness.irq_count++;
  g_vclock_witness.compare_armed = false;

  if (!g_ocxo_witness_edge_asap_requested) {
    g_ocxo_witness_edge_asap_requested = true;
    const timepop_handle_t h = timepop_arm_asap(ocxo_witness_edge_asap_callback,
                                                nullptr,
                                                OCXO_WITNESS_EDGE_ASAP_NAME);
    if (h == TIMEPOP_INVALID_HANDLE) {
      g_vclock_witness.asap_arm_failures++;
      g_ocxo_witness_edge_asap_requested = false;
    }
  }
}

static const char* ocxo_witness_module_name(const ocxo_witness_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2) return "QTIMER2";
  if (lane.module == &IMXRT_TMR3) return "QTIMER3";
  return "UNKNOWN";
}

static void ocxo_witness_bind_lanes(void) {
  g_ocxo_witness_1.name = "OCXO1";
  g_ocxo_witness_1.module = &IMXRT_TMR2;
  g_ocxo_witness_1.channel = 0;
  g_ocxo_witness_1.pcs = 0;
  g_ocxo_witness_1.input_pin = OCXO1_PIN;
  g_ocxo_witness_1.mask = OCXO_WITNESS_LANE1_MASK;

  g_ocxo_witness_2.name = "OCXO2";
  g_ocxo_witness_2.module = &IMXRT_TMR3;
  g_ocxo_witness_2.channel = 3;
  g_ocxo_witness_2.pcs = 3;
  g_ocxo_witness_2.input_pin = OCXO2_PIN;
  g_ocxo_witness_2.mask = OCXO_WITNESS_LANE2_MASK;
}

static inline void ocxo_witness_clear_compare_flag(ocxo_witness_lane_t& lane) {
  lane.module->CH[lane.channel].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void ocxo_witness_disable_compare(ocxo_witness_lane_t& lane) {
  lane.module->CH[lane.channel].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  ocxo_witness_clear_compare_flag(lane);
  lane.compare_armed = false;
}

static inline void ocxo_witness_program_compare(ocxo_witness_lane_t& lane,
                                                uint16_t target_low16) {
  ocxo_witness_clear_compare_flag(lane);
  lane.module->CH[lane.channel].COMP1  = target_low16;
  lane.module->CH[lane.channel].CMPLD1 = target_low16;
  ocxo_witness_clear_compare_flag(lane);
  lane.module->CH[lane.channel].CSCTRL |= TMR_CSCTRL_TCF1EN;
  lane.compare_armed = true;
  lane.arm_count++;
}

static inline uint16_t ocxo_witness_counter_now(const ocxo_witness_lane_t& lane) {
  return lane.module->CH[lane.channel].CNTR;
}

static inline uint16_t ocxo_witness_target_low16(const ocxo_witness_lane_t& lane,
                                                 uint64_t target_total) {
  return (uint16_t)(lane.origin_low16 + (uint16_t)(target_total & 0xFFFFULL));
}

static void ocxo_witness_lane_reset_runtime(ocxo_witness_lane_t& lane) {
  lane.enabled = false;
  lane.initialized = false;
  lane.compare_armed = false;
  lane.irq_pending = false;

  lane.irq_count = 0;
  lane.false_irq_count = 0;
  lane.pending_overwrite_count = 0;
  lane.asap_arm_failures = 0;
  lane.pending_dwt_at_isr = 0;
  lane.pending_counter_low16_at_isr = 0;
  lane.pending_target_low16 = 0;

  lane.origin_low16 = 0;
  lane.last_hw16 = 0;
  lane.total_cycles = 0;
  lane.next_target_total = OCXO_WITNESS_COUNTS_PER_SECOND;
  lane.next_target_low16 = 0;
  lane.dwt_total_cycles = 0;

  lane.seen_first_edge = false;
  lane.have_last_delta = false;
  lane.last_edge_dwt32 = 0;
  lane.last_edge_dwt64 = 0;
  lane.last_dwt_delta_cycles = 0;

  lane.previous = ocxo_witness_edge_record_t{};
  lane.last = ocxo_witness_edge_record_t{};

  lane.edge_count = 0;
  lane.arm_count = 0;
  lane.arm_late_count = 0;
  lane.missed_target_count = 0;
  lane.rollover_update_count = 0;
  lane.large_delta_count = 0;
  lane.largest_delta16 = 0;
  lane.last_published_edge_count = 0;

  welford_reset(lane.dwt_delta_welford);
  welford_reset(lane.residual_welford);
}

static void ocxo_witness_configure_hw_lane(ocxo_witness_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
    *(portConfigRegister(lane.input_pin)) = 1;
    IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;
  } else if (lane.module == &IMXRT_TMR3) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
    *(portConfigRegister(lane.input_pin)) = 1;
    IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;
  }

  lane.module->CH[lane.channel].CTRL   = 0;
  lane.module->CH[lane.channel].SCTRL  = 0;
  lane.module->CH[lane.channel].CSCTRL = 0;
  lane.module->CH[lane.channel].LOAD   = 0;
  lane.module->CH[lane.channel].CNTR   = 0;
  lane.module->CH[lane.channel].COMP1  = 0xFFFF;
  lane.module->CH[lane.channel].CMPLD1 = 0xFFFF;
  lane.module->CH[lane.channel].CMPLD2 = 0;
  lane.module->CH[lane.channel].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(lane.pcs);
  ocxo_witness_disable_compare(lane);

  if (lane.module == &IMXRT_TMR2) {
    IMXRT_TMR2.ENBL |= (uint16_t)(1U << lane.channel);
  } else if (lane.module == &IMXRT_TMR3) {
    IMXRT_TMR3.ENBL |= (uint16_t)(1U << lane.channel);
  }

  lane.initialized = true;
}

static void ocxo_witness_qtimer2_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (!(IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1)) return;

  ocxo_witness_lane_t& lane = g_ocxo_witness_1;
  const uint16_t counter_low16 = IMXRT_TMR2.CH[0].CNTR;
  ocxo_witness_clear_compare_flag(lane);

  if (!lane.enabled || !lane.compare_armed) {
    lane.false_irq_count++;
    ocxo_witness_disable_compare(lane);
    return;
  }

  if (lane.irq_pending) lane.pending_overwrite_count++;
  lane.pending_dwt_at_isr = dwt_raw;
  lane.pending_counter_low16_at_isr = counter_low16;
  lane.pending_target_low16 = lane.next_target_low16;
  lane.irq_pending = true;
  lane.irq_count++;
  ocxo_witness_disable_compare(lane);

  if (!g_ocxo_witness_edge_asap_requested) {
    g_ocxo_witness_edge_asap_requested = true;
    const timepop_handle_t h = timepop_arm_asap(ocxo_witness_edge_asap_callback,
                                                nullptr,
                                                OCXO_WITNESS_EDGE_ASAP_NAME);
    if (h == TIMEPOP_INVALID_HANDLE) {
      lane.asap_arm_failures++;
      g_ocxo_witness_edge_asap_requested = false;
    }
  }
}

static void ocxo_witness_qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (!(IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1)) return;

  ocxo_witness_lane_t& lane = g_ocxo_witness_2;
  const uint16_t counter_low16 = IMXRT_TMR3.CH[3].CNTR;
  ocxo_witness_clear_compare_flag(lane);

  if (!lane.enabled || !lane.compare_armed) {
    lane.false_irq_count++;
    ocxo_witness_disable_compare(lane);
    return;
  }

  if (lane.irq_pending) lane.pending_overwrite_count++;
  lane.pending_dwt_at_isr = dwt_raw;
  lane.pending_counter_low16_at_isr = counter_low16;
  lane.pending_target_low16 = lane.next_target_low16;
  lane.irq_pending = true;
  lane.irq_count++;
  ocxo_witness_disable_compare(lane);

  if (!g_ocxo_witness_edge_asap_requested) {
    g_ocxo_witness_edge_asap_requested = true;
    const timepop_handle_t h = timepop_arm_asap(ocxo_witness_edge_asap_callback,
                                                nullptr,
                                                OCXO_WITNESS_EDGE_ASAP_NAME);
    if (h == TIMEPOP_INVALID_HANDLE) {
      lane.asap_arm_failures++;
      g_ocxo_witness_edge_asap_requested = false;
    }
  }
}

static void ocxo_witness_install_vectors(uint8_t mask) {
  if (mask & OCXO_WITNESS_LANE1_MASK) {
    attachInterruptVector(IRQ_QTIMER2, ocxo_witness_qtimer2_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
    NVIC_ENABLE_IRQ(IRQ_QTIMER2);
    g_ocxo_witness_vector_mask |= OCXO_WITNESS_LANE1_MASK;
  }

  if (mask & OCXO_WITNESS_LANE2_MASK) {
    attachInterruptVector(IRQ_QTIMER3, ocxo_witness_qtimer3_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
    NVIC_ENABLE_IRQ(IRQ_QTIMER3);
    g_ocxo_witness_vector_mask |= OCXO_WITNESS_LANE2_MASK;
  }
}

static void ocxo_witness_release_vectors(uint8_t mask) {
  if ((mask & OCXO_WITNESS_LANE1_MASK) &&
      (g_ocxo_witness_vector_mask & OCXO_WITNESS_LANE1_MASK) &&
      !g_ocxo_witness_1.enabled) {
    NVIC_DISABLE_IRQ(IRQ_QTIMER2);
    g_ocxo_witness_vector_mask &= (uint8_t)~OCXO_WITNESS_LANE1_MASK;
  }

  if ((mask & OCXO_WITNESS_LANE2_MASK) &&
      (g_ocxo_witness_vector_mask & OCXO_WITNESS_LANE2_MASK) &&
      !g_ocxo_witness_2.enabled) {
    NVIC_DISABLE_IRQ(IRQ_QTIMER3);
    g_ocxo_witness_vector_mask &= (uint8_t)~OCXO_WITNESS_LANE2_MASK;
  }
}

static void ocxo_witness_start_lane(ocxo_witness_lane_t& lane) {
  const uint32_t prior_start_count = lane.start_count;
  ocxo_witness_lane_reset_runtime(lane);
  lane.start_count = prior_start_count + 1;
  ocxo_witness_configure_hw_lane(lane);

  const uint16_t now16 = ocxo_witness_counter_now(lane);
  lane.origin_low16 = now16;
  lane.last_hw16 = now16;
  lane.total_cycles = 0;
  lane.next_target_total = OCXO_WITNESS_COUNTS_PER_SECOND;
  lane.next_target_low16 = ocxo_witness_target_low16(lane, lane.next_target_total);
  lane.enabled = true;
}

static void ocxo_witness_stop_lane(ocxo_witness_lane_t& lane) {
  const bool was_active = lane.enabled || lane.compare_armed || lane.irq_pending;
  if (was_active && lane.initialized) {
    ocxo_witness_disable_compare(lane);
  }
  lane.enabled = false;
  lane.irq_pending = false;
  if (was_active) lane.stop_count++;
}

static bool ocxo_witness_any_lane_enabled(void) {
  return g_ocxo_witness_1.enabled || g_ocxo_witness_2.enabled;
}

static void ocxo_witness_update_total_from_hw16(ocxo_witness_lane_t& lane,
                                                uint16_t now16) {
  const uint16_t delta16 = (uint16_t)(now16 - lane.last_hw16);
  if (delta16 == 0) return;

  lane.total_cycles += (uint32_t)delta16;
  lane.last_hw16 = now16;
  lane.rollover_update_count++;
  if (delta16 > lane.largest_delta16) lane.largest_delta16 = delta16;
  if (delta16 >= OCXO_WITNESS_LARGE_DELTA16_TICKS) lane.large_delta_count++;
}

static void ocxo_witness_advance_target_to_future(ocxo_witness_lane_t& lane) {
  while (lane.total_cycles >= lane.next_target_total) {
    lane.missed_target_count++;
    lane.next_target_total += OCXO_WITNESS_COUNTS_PER_SECOND;
  }
  lane.next_target_low16 = ocxo_witness_target_low16(lane, lane.next_target_total);
}

static void ocxo_witness_tend_lane(ocxo_witness_lane_t& lane) {
  if (!lane.enabled || !lane.initialized) return;
  if (lane.irq_pending) return;

  // Once the one-shot compare is armed, stop tending this lane until the
  // compare ISR/ASAP pair authors the edge.  The arm window is < one 16-bit
  // rollover, so pausing here prevents the cadence minder from moving the
  // software total past the target before the edge fact is consumed.
  if (lane.compare_armed) return;

  const uint16_t now16 = ocxo_witness_counter_now(lane);
  ocxo_witness_update_total_from_hw16(lane, now16);

  if (lane.total_cycles >= lane.next_target_total) {
    lane.arm_late_count++;
    ocxo_witness_advance_target_to_future(lane);
    return;
  }

  const uint64_t remaining = lane.next_target_total - lane.total_cycles;
  if (remaining <= OCXO_WITNESS_ARM_WINDOW_TICKS) {
    lane.next_target_low16 = ocxo_witness_target_low16(lane, lane.next_target_total);
    ocxo_witness_program_compare(lane, lane.next_target_low16);
  }
}

static void ocxo_witness_complete_edge(ocxo_witness_lane_t& lane,
                                       uint32_t dwt_at_isr,
                                       uint16_t counter_low16_at_isr,
                                       uint16_t target_low16) {
  if (!lane.enabled) return;

  ocxo_witness_update_total_from_hw16(lane, counter_low16_at_isr);

  const uint64_t target_total = lane.next_target_total;
  const uint64_t total_at_isr = lane.total_cycles;
  const int32_t service_offset =
      clamp_i64_to_i32((int64_t)total_at_isr - (int64_t)target_total);

  ocxo_witness_edge_record_t rec{};
  rec.valid = true;
  rec.sequence = lane.edge_count + 1;
  rec.irq_count = lane.irq_count;
  rec.ocxo_total_at_edge = target_total;
  rec.ocxo_total_at_isr = total_at_isr;
  rec.dwt_at_edge = dwt_at_isr;
  rec.target_low16 = target_low16;
  rec.counter_low16_at_isr = counter_low16_at_isr;
  rec.service_offset_ticks = service_offset;

  if (lane.seen_first_edge) {
    const uint32_t dwt_delta = (uint32_t)(dwt_at_isr - lane.last_edge_dwt32);
    lane.dwt_total_cycles += dwt_delta;
    lane.last_edge_dwt64 = lane.dwt_total_cycles;

    rec.delta_valid = true;
    rec.dwt_total_at_edge = lane.dwt_total_cycles;
    rec.dwt_delta_cycles = dwt_delta;
    rec.dwt_pred_cycles = lane.have_last_delta ? lane.last_dwt_delta_cycles : 0;

    welford_update(lane.dwt_delta_welford, (int32_t)dwt_delta);

    if (lane.have_last_delta) {
      rec.residual_valid = true;
      rec.dwt_residual_cycles =
          (int32_t)((int64_t)dwt_delta - (int64_t)lane.last_dwt_delta_cycles);
      welford_update(lane.residual_welford, rec.dwt_residual_cycles);
    }

    lane.last_dwt_delta_cycles = dwt_delta;
    lane.have_last_delta = true;
  } else {
    lane.seen_first_edge = true;
    lane.dwt_total_cycles = 0;
    lane.last_edge_dwt64 = 0;
    rec.dwt_total_at_edge = 0;
  }

  lane.last_edge_dwt32 = dwt_at_isr;
  lane.previous = lane.last;
  lane.last = rec;
  lane.edge_count++;

  lane.next_target_total = target_total + OCXO_WITNESS_COUNTS_PER_SECOND;
  lane.next_target_low16 = ocxo_witness_target_low16(lane, lane.next_target_total);
  lane.compare_armed = false;
}

static void ocxo_witness_process_pending_lane(ocxo_witness_lane_t& lane) {
  bool pending;
  uint32_t dwt_at_isr;
  uint16_t counter_low16_at_isr;
  uint16_t target_low16;

  noInterrupts();
  pending = lane.irq_pending;
  dwt_at_isr = lane.pending_dwt_at_isr;
  counter_low16_at_isr = lane.pending_counter_low16_at_isr;
  target_low16 = lane.pending_target_low16;
  lane.irq_pending = false;
  interrupts();

  if (!pending) return;
  ocxo_witness_complete_edge(lane, dwt_at_isr, counter_low16_at_isr, target_low16);
}

static void ocxo_witness_edge_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  g_ocxo_witness_edge_asap_requested = false;
  g_ocxo_witness_edge_asap_count++;
  vclock_witness_process_pending();
  ocxo_witness_process_pending_lane(g_ocxo_witness_1);
  ocxo_witness_process_pending_lane(g_ocxo_witness_2);
}

static void ocxo_witness_minder_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_ocxo_witness_running) return;

  g_ocxo_witness_minder_fire_count++;
  vclock_witness_tend();
  ocxo_witness_tend_lane(g_ocxo_witness_1);
  ocxo_witness_tend_lane(g_ocxo_witness_2);

  if (++g_ocxo_witness_minder_divider >= OCXO_WITNESS_MINDER_PER_REPORT) {
    g_ocxo_witness_minder_divider = 0;
    g_ocxo_witness_publish_request_count++;

    if (!g_ocxo_witness_publish_asap_requested) {
      g_ocxo_witness_publish_asap_requested = true;
      const timepop_handle_t h = timepop_arm_asap(ocxo_witness_publish_asap_callback,
                                                  nullptr,
                                                  OCXO_WITNESS_PUBLISH_ASAP_NAME);
      if (h == TIMEPOP_INVALID_HANDLE) {
        g_ocxo_witness_publish_asap_requested = false;
        g_ocxo_witness_publish_asap_failures++;
      }
    }
  }
}

static bool ocxo_witness_arm_minder(void) {
  timepop_cancel_by_name(OCXO_WITNESS_MINDER_NAME);
  const timepop_handle_t h =
      timepop_arm_recurring_isr(OCXO_WITNESS_MINDER_PERIOD_NS,
                                ocxo_witness_minder_callback,
                                nullptr,
                                OCXO_WITNESS_MINDER_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_ocxo_witness_minder_armed = false;
    g_ocxo_witness_minder_arm_failures++;
    return false;
  }

  g_ocxo_witness_minder_armed = true;
  g_ocxo_witness_minder_arm_count++;
  return true;
}

static uint8_t ocxo_witness_parse_lane_mask(const Payload& args) {
  const char* lane = args.getString("lane");
  if (lane && *lane) {
    if (strcasecmp(lane, "1") == 0 || strcasecmp(lane, "ocxo1") == 0) {
      return OCXO_WITNESS_LANE1_MASK;
    }
    if (strcasecmp(lane, "2") == 0 || strcasecmp(lane, "ocxo2") == 0) {
      return OCXO_WITNESS_LANE2_MASK;
    }
    if (strcasecmp(lane, "both") == 0 || strcasecmp(lane, "all") == 0 ||
        strcasecmp(lane, "12") == 0) {
      return OCXO_WITNESS_BOTH_MASK;
    }
    return 0;
  }

  uint32_t lane_u32 = 0;
  if (args.tryGetUInt("lane", lane_u32)) {
    if (lane_u32 == 1) return OCXO_WITNESS_LANE1_MASK;
    if (lane_u32 == 2) return OCXO_WITNESS_LANE2_MASK;
    if (lane_u32 == 12 || lane_u32 == 0) return OCXO_WITNESS_BOTH_MASK;
    return 0;
  }

  return OCXO_WITNESS_BOTH_MASK;
}

static Payload ocxo_witness_edge_payload(const ocxo_witness_edge_record_t& e) {
  Payload p;
  p.add("valid", e.valid);
  p.add("delta_valid", e.delta_valid);
  p.add("residual_valid", e.residual_valid);
  p.add("sequence", e.sequence);
  p.add("irq_count", e.irq_count);
  p.add("clock_total_at_edge", e.ocxo_total_at_edge);
  p.add("clock_total_at_event", e.ocxo_total_at_isr);
  p.add("ocxo_total_at_edge", e.ocxo_total_at_edge);
  p.add("ocxo_total_at_isr", e.ocxo_total_at_isr);
  p.add("dwt_total_at_edge", e.dwt_total_at_edge);
  p.add("dwt_at_edge", e.dwt_at_edge);
  p.add("dwt_delta_cycles", e.dwt_delta_cycles);
  p.add("dwt_pred_cycles", e.dwt_pred_cycles);
  p.add("dwt_residual_cycles", e.dwt_residual_cycles);
  p.add("target_low16", (uint32_t)e.target_low16);
  p.add("counter_low16_at_isr", (uint32_t)e.counter_low16_at_isr);
  p.add("service_offset_ticks", e.service_offset_ticks);
  return p;
}

static Payload ocxo_witness_lane_payload(ocxo_witness_lane_t& lane,
                                        bool consume_publish_marker = false) {
  ocxo_witness_edge_record_t last;
  ocxo_witness_edge_record_t previous;
  welford_t delta_w;
  welford_t residual_w;

  noInterrupts();
  last = lane.last;
  previous = lane.previous;
  delta_w = lane.dwt_delta_welford;
  residual_w = lane.residual_welford;
  const bool enabled = lane.enabled;
  const bool initialized = lane.initialized;
  const bool compare_armed = lane.compare_armed;
  const bool irq_pending = lane.irq_pending;
  const uint16_t hw16 = initialized ? ocxo_witness_counter_now(lane) : 0;
  const uint16_t origin_low16 = lane.origin_low16;
  const uint16_t last_hw16 = lane.last_hw16;
  const uint64_t total_cycles = lane.total_cycles;
  const uint64_t next_target_total = lane.next_target_total;
  const uint16_t next_target_low16 = lane.next_target_low16;
  const uint64_t dwt_total_cycles = lane.dwt_total_cycles;
  const uint32_t start_count = lane.start_count;
  const uint32_t stop_count = lane.stop_count;
  const uint32_t edge_count = lane.edge_count;
  const uint32_t irq_count = lane.irq_count;
  const uint32_t false_irq_count = lane.false_irq_count;
  const uint32_t pending_overwrite_count = lane.pending_overwrite_count;
  const uint32_t asap_arm_failures = lane.asap_arm_failures;
  const uint32_t arm_count = lane.arm_count;
  const uint32_t arm_late_count = lane.arm_late_count;
  const uint32_t missed_target_count = lane.missed_target_count;
  const uint32_t rollover_update_count = lane.rollover_update_count;
  const uint32_t large_delta_count = lane.large_delta_count;
  const uint16_t largest_delta16 = lane.largest_delta16;
  const bool new_since_last_publish = lane.edge_count != lane.last_published_edge_count;
  if (consume_publish_marker) {
    lane.last_published_edge_count = lane.edge_count;
  }
  interrupts();

  Payload p;
  p.add("name", lane.name ? lane.name : "");
  p.add("enabled", enabled);
  p.add("initialized", initialized);
  p.add("module", ocxo_witness_module_name(lane));
  p.add("channel", (uint32_t)lane.channel);
  p.add("pcs", (uint32_t)lane.pcs);
  p.add("pin", (uint32_t)lane.input_pin);
  p.add("compare_armed", compare_armed);
  p.add("irq_pending", irq_pending);
  p.add("new_since_last_publish", new_since_last_publish);

  p.add("origin_low16", (uint32_t)origin_low16);
  p.add("hw16_now", (uint32_t)hw16);
  p.add("last_hw16", (uint32_t)last_hw16);
  p.add("total_cycles", total_cycles);
  p.add("next_target_total", next_target_total);
  p.add("next_target_low16", (uint32_t)next_target_low16);
  p.add("dwt_total_cycles", dwt_total_cycles);

  p.add("start_count", start_count);
  p.add("stop_count", stop_count);
  p.add("edge_count", edge_count);
  p.add("irq_count", irq_count);
  p.add("false_irq_count", false_irq_count);
  p.add("pending_overwrite_count", pending_overwrite_count);
  p.add("asap_arm_failures", asap_arm_failures);
  p.add("arm_count", arm_count);
  p.add("arm_late_count", arm_late_count);
  p.add("missed_target_count", missed_target_count);
  p.add("rollover_update_count", rollover_update_count);
  p.add("large_delta_count", large_delta_count);
  p.add("largest_delta16", (uint32_t)largest_delta16);

  p.add_object("last", ocxo_witness_edge_payload(last));
  p.add_object("previous", ocxo_witness_edge_payload(previous));
  p.add_object("dwt_delta_welford", welford_payload(delta_w));
  p.add_object("residual_welford", welford_payload(residual_w));
  return p;
}

static Payload vclock_witness_edge_payload(const vclock_witness_edge_record_t& e) {
  Payload p;
  p.add("valid", e.valid);
  p.add("delta_valid", e.delta_valid);
  p.add("residual_valid", e.residual_valid);
  p.add("sequence", e.sequence);
  p.add("irq_count", e.irq_count);
  p.add("clock_total_at_edge", e.clock_total_at_edge);
  p.add("clock_total_at_event", e.clock_total_at_event);
  p.add("vclock_total_at_edge", e.clock_total_at_edge);
  p.add("vclock_total_at_event", e.clock_total_at_event);
  p.add("dwt_total_at_edge", e.dwt_total_at_edge);
  p.add("target_counter32", e.target_counter32);
  p.add("counter32_at_event", e.counter32_at_event);
  p.add("counter32_residual_ticks", e.counter32_residual_ticks);
  p.add("isr_entry_dwt_raw", e.isr_entry_dwt_raw);
  p.add("dwt_at_edge", e.dwt_at_edge);
  p.add("gnss_ns_at_event", e.gnss_ns_at_event);
  p.add("dwt_delta_cycles", e.dwt_delta_cycles);
  p.add("dwt_pred_cycles", e.dwt_pred_cycles);
  p.add("dwt_residual_cycles", e.dwt_residual_cycles);
  p.add("service_offset_ticks", e.service_offset_ticks);
  return p;
}

static Payload vclock_witness_lane_payload(bool consume_publish_marker = false) {
  vclock_witness_edge_record_t last;
  vclock_witness_edge_record_t previous;
  welford_t delta_w;
  welford_t residual_w;

  noInterrupts();
  last = g_vclock_witness.last;
  previous = g_vclock_witness.previous;
  delta_w = g_vclock_witness.dwt_delta_welford;
  residual_w = g_vclock_witness.residual_welford;
  const bool enabled = g_vclock_witness.enabled;
  const bool initialized = g_vclock_witness.initialized;
  const bool compare_armed = g_vclock_witness.compare_armed;
  const bool irq_pending = g_vclock_witness.irq_pending;
  const bool handler_registered = g_vclock_witness.handler_registered;
  const uint32_t origin_counter32 = g_vclock_witness.origin_counter32;
  const uint32_t next_target_counter32 = g_vclock_witness.next_target_counter32;
  const uint64_t next_target_total = g_vclock_witness.next_target_total;
  const uint32_t first_delay_ticks = g_vclock_witness.first_target_delay_ticks;
  const uint64_t dwt_total_cycles = g_vclock_witness.dwt_total_cycles;
  const uint32_t start_count = g_vclock_witness.start_count;
  const uint32_t stop_count = g_vclock_witness.stop_count;
  const uint32_t edge_count = g_vclock_witness.edge_count;
  const uint32_t irq_count = g_vclock_witness.irq_count;
  const uint32_t false_irq_count = g_vclock_witness.false_irq_count;
  const uint32_t pending_overwrite_count = g_vclock_witness.pending_overwrite_count;
  const uint32_t asap_arm_failures = g_vclock_witness.asap_arm_failures;
  const uint32_t arm_count = g_vclock_witness.arm_count;
  const uint32_t arm_failures = g_vclock_witness.arm_failures;
  const uint32_t arm_late_count = g_vclock_witness.arm_late_count;
  const uint32_t missed_target_count = g_vclock_witness.missed_target_count;
  const uint32_t tend_count = g_vclock_witness.tend_count;
  const uint32_t last_arm_remaining_ticks = g_vclock_witness.last_arm_remaining_ticks;
  const uint32_t last_arm_now_counter32 = g_vclock_witness.last_arm_now_counter32;
  const bool new_since_last_publish =
      g_vclock_witness.edge_count != g_vclock_witness.last_published_edge_count;
  if (consume_publish_marker) {
    g_vclock_witness.last_published_edge_count = g_vclock_witness.edge_count;
  }
  interrupts();

  const uint32_t counter32_now = initialized
      ? interrupt_vclock_counter32_observe_ambient()
      : 0;

  Payload p;
  p.add("name", "VCLOCK");
  p.add("enabled", enabled);
  p.add("initialized", initialized);
  p.add("module", "QTIMER1");
  p.add("channel", (uint32_t)1);
  p.add("hardware_lane", "QTIMER1_CH1_COMP_HOSTED");
  p.add("counter_source", "QTIMER1_CH0_SYNTHETIC_COUNTER32");
  p.add("compare_armed", compare_armed);
  p.add("irq_pending", irq_pending);
  p.add("handler_registered", handler_registered);
  p.add("new_since_last_publish", new_since_last_publish);

  p.add("origin_counter32", origin_counter32);
  p.add("counter32_now", counter32_now);
  p.add("next_target_counter32", next_target_counter32);
  p.add("next_target_total", next_target_total);
  p.add("first_delay_ticks", first_delay_ticks);
  p.add("dwt_total_cycles", dwt_total_cycles);

  p.add("start_count", start_count);
  p.add("stop_count", stop_count);
  p.add("edge_count", edge_count);
  p.add("irq_count", irq_count);
  p.add("false_irq_count", false_irq_count);
  p.add("pending_overwrite_count", pending_overwrite_count);
  p.add("asap_arm_failures", asap_arm_failures);
  p.add("arm_count", arm_count);
  p.add("arm_failures", arm_failures);
  p.add("arm_late_count", arm_late_count);
  p.add("missed_target_count", missed_target_count);
  p.add("tend_count", tend_count);
  p.add("last_arm_remaining_ticks", last_arm_remaining_ticks);
  p.add("last_arm_now_counter32", last_arm_now_counter32);

  p.add_object("last", vclock_witness_edge_payload(last));
  p.add_object("previous", vclock_witness_edge_payload(previous));
  p.add_object("dwt_delta_welford", welford_payload(delta_w));
  p.add_object("residual_welford", welford_payload(residual_w));
  return p;
}

static Payload ocxo_witness_state_payload(bool consume_publish_marker = false) {
  Payload p;
  p.add("running", g_ocxo_witness_running);
  p.add("active_mask", (uint32_t)g_ocxo_witness_active_mask);
  p.add("vclock_active", g_vclock_witness.enabled);
  p.add("vector_mask", (uint32_t)g_ocxo_witness_vector_mask);
  p.add("vectors_installed", g_ocxo_witness_vector_mask != 0);
  p.add("minder_armed", g_ocxo_witness_minder_armed);
  p.add("minder_period_ns", OCXO_WITNESS_MINDER_PERIOD_NS);
  p.add("minder_per_report", OCXO_WITNESS_MINDER_PER_REPORT);
  p.add("arm_window_ticks", OCXO_WITNESS_ARM_WINDOW_TICKS);
  p.add("start_count", g_ocxo_witness_start_count);
  p.add("stop_count", g_ocxo_witness_stop_count);
  p.add("minder_fire_count", g_ocxo_witness_minder_fire_count);
  p.add("minder_arm_count", g_ocxo_witness_minder_arm_count);
  p.add("minder_arm_failures", g_ocxo_witness_minder_arm_failures);
  p.add("publish_request_count", g_ocxo_witness_publish_request_count);
  p.add("publish_count", g_ocxo_witness_publish_count);
  p.add("publish_asap_failures", g_ocxo_witness_publish_asap_failures);
  p.add("edge_asap_count", g_ocxo_witness_edge_asap_count);
  p.add_object("vclock", vclock_witness_lane_payload(consume_publish_marker));
  p.add_object("ocxo1", ocxo_witness_lane_payload(g_ocxo_witness_1, consume_publish_marker));
  p.add_object("ocxo2", ocxo_witness_lane_payload(g_ocxo_witness_2, consume_publish_marker));
  return p;
}

static void ocxo_witness_publish_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  g_ocxo_witness_publish_asap_requested = false;
  g_ocxo_witness_publish_count++;

  Payload p;
  p.add("model", "CLOCK_EDGE_TO_EDGE_DWT_CYCLES");
  p.add("historical", true);
  p.add("counts_per_second", OCXO_WITNESS_COUNTS_PER_SECOND);
  p.add("topic_replaces", "OCXO_WITNESS");
  p.add_object("state", ocxo_witness_state_payload(true));
  publish(CLOCK_WITNESS_TOPIC, p);
}

static void ocxo_witness_stop_mask(uint8_t mask) {
  if (mask & OCXO_WITNESS_LANE1_MASK) ocxo_witness_stop_lane(g_ocxo_witness_1);
  if (mask & OCXO_WITNESS_LANE2_MASK) ocxo_witness_stop_lane(g_ocxo_witness_2);

  uint8_t active = 0;
  if (g_ocxo_witness_1.enabled) active |= OCXO_WITNESS_LANE1_MASK;
  if (g_ocxo_witness_2.enabled) active |= OCXO_WITNESS_LANE2_MASK;
  g_ocxo_witness_active_mask = active;
  g_ocxo_witness_running = clock_witness_any_active();

  ocxo_witness_release_vectors(mask);
  clock_witness_cancel_deferred_if_idle();
}

static Payload cmd_ocxo_start(const Payload& args) {
  const uint8_t mask = ocxo_witness_parse_lane_mask(args);
  Payload p;
  p.add("model", "OCXO_EDGE_TO_EDGE_DWT_CYCLES");
  p.add("publish_topic", CLOCK_WITNESS_TOPIC);
  p.add("requires_process_interrupt_defeated", true);

  if (mask == 0) {
    p.add("status", "error");
    p.add("error", "invalid lane; use lane=1, lane=2, or lane=both");
    return p;
  }

  ocxo_witness_bind_lanes();
  ocxo_witness_stop_mask(g_ocxo_witness_active_mask);
  ocxo_witness_install_vectors(mask);

  g_ocxo_witness_start_count++;
  g_ocxo_witness_active_mask = mask;
  g_ocxo_witness_minder_fire_count = 0;
  g_ocxo_witness_minder_divider = 0;
  g_ocxo_witness_publish_request_count = 0;
  g_ocxo_witness_publish_count = 0;
  g_ocxo_witness_publish_asap_failures = 0;
  g_ocxo_witness_edge_asap_count = 0;

  if (mask & OCXO_WITNESS_LANE1_MASK) ocxo_witness_start_lane(g_ocxo_witness_1);
  if (mask & OCXO_WITNESS_LANE2_MASK) ocxo_witness_start_lane(g_ocxo_witness_2);

  g_ocxo_witness_running = clock_witness_any_active();
  const bool minder_ok = g_ocxo_witness_minder_armed || ocxo_witness_arm_minder();

  p.add("status", g_ocxo_witness_running && minder_ok ? "started" : "error");
  p.add("lane_mask", (uint32_t)mask);
  p.add("minder_ok", minder_ok);
  p.add_object("state", ocxo_witness_state_payload());
  return p;
}

static Payload cmd_ocxo_stop(const Payload& args) {
  uint8_t mask = ocxo_witness_parse_lane_mask(args);
  if (mask == 0) mask = OCXO_WITNESS_BOTH_MASK;

  g_ocxo_witness_stop_count++;
  ocxo_witness_stop_mask(mask);

  Payload p;
  p.add("model", "OCXO_EDGE_TO_EDGE_DWT_CYCLES");
  p.add("publish_topic", CLOCK_WITNESS_TOPIC);
  p.add("status", "stopped");
  p.add("lane_mask", (uint32_t)mask);
  p.add_object("state", ocxo_witness_state_payload());
  return p;
}

static uint32_t vclock_witness_first_delay_from_args(const Payload& args) {
  uint32_t first_delay = VCLOCK_WITNESS_FIRST_TARGET_DELAY_TICKS;
  (void)args.tryGetUInt("first_delay_ticks", first_delay);
  (void)args.tryGetUInt("delay_ticks", first_delay);
  (void)args.tryGetUInt("offset_ticks", first_delay);

  if (first_delay == 0 || first_delay > 0x7FFFFFFFUL) {
    first_delay = VCLOCK_WITNESS_FIRST_TARGET_DELAY_TICKS;
  }
  return first_delay;
}

static Payload cmd_vclock_start(const Payload& args) {
  Payload p;
  p.add("model", "VCLOCK_CH1_EDGE_TO_EDGE_DWT_CYCLES");
  p.add("publish_topic", CLOCK_WITNESS_TOPIC);
  p.add("uses_process_interrupt_hosted_ch1", true);
  p.add("vclock_ch2_untouched", true);

  interrupt_qtimer1_ch1_disable_compare();
  interrupt_register_qtimer1_ch1_handler(nullptr);
  vclock_witness_reset_runtime();
  g_vclock_witness.start_count++;

  const uint32_t first_delay = vclock_witness_first_delay_from_args(args);
  const uint32_t now = interrupt_vclock_counter32_observe_ambient();

  g_vclock_witness.enabled = true;
  g_vclock_witness.initialized = true;
  g_vclock_witness.handler_registered = true;
  g_vclock_witness.origin_counter32 = now;
  g_vclock_witness.first_target_delay_ticks = first_delay;
  g_vclock_witness.next_target_total = first_delay;
  g_vclock_witness.next_target_counter32 = now + first_delay;

  interrupt_register_qtimer1_ch1_handler(vclock_witness_ch1_handler);

  g_ocxo_witness_running = clock_witness_any_active();
  const bool minder_ok = g_ocxo_witness_minder_armed || ocxo_witness_arm_minder();

  p.add("status", minder_ok ? "started" : "error");
  p.add("first_delay_ticks", first_delay);
  p.add("origin_counter32", now);
  p.add("first_target_counter32", g_vclock_witness.next_target_counter32);
  p.add("minder_ok", minder_ok);
  p.add_object("state", ocxo_witness_state_payload());
  return p;
}

static Payload cmd_vclock_stop(const Payload&) {
  const bool was_active = g_vclock_witness.enabled ||
                          g_vclock_witness.compare_armed ||
                          g_vclock_witness.irq_pending ||
                          g_vclock_witness.handler_registered;

  interrupt_qtimer1_ch1_disable_compare();
  interrupt_register_qtimer1_ch1_handler(nullptr);

  g_vclock_witness.enabled = false;
  g_vclock_witness.compare_armed = false;
  g_vclock_witness.irq_pending = false;
  g_vclock_witness.handler_registered = false;
  if (was_active) g_vclock_witness.stop_count++;

  g_ocxo_witness_running = clock_witness_any_active();
  clock_witness_cancel_deferred_if_idle();

  Payload p;
  p.add("model", "VCLOCK_CH1_EDGE_TO_EDGE_DWT_CYCLES");
  p.add("publish_topic", CLOCK_WITNESS_TOPIC);
  p.add("status", "stopped");
  p.add_object("state", ocxo_witness_state_payload());
  return p;
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

static void entry_latency_pps_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;

  g_entry_pps.armed = false;
  entry_latency_spin(g_entry_pps);
  (void)entry_latency_arm_pps_spin();
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
// TimePop waveform callbacks — GPIO measurement window
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


static void witness_cancel_periodic_callbacks(void) {
  timepop_cancel_by_name(GPIO_HIGH_NAME);
  timepop_cancel_by_name(GPIO_LOW_NAME);
  timepop_cancel_by_name(WITNESS_BRIDGE_NAME);
}

static bool witness_arm_one_shot_at_offset(const char* name,
                                            uint64_t offset_ns,
                                            timepop_callback_t callback) {
  const time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return false;

  int64_t anchor_ns = (int64_t)(snap.pps_count - 1) * (int64_t)NS_PER_SECOND;
  int64_t target_ns = anchor_ns + (int64_t)offset_ns;
  const int64_t now_ns = time_gnss_ns_now();
  while (now_ns >= 0 && target_ns <= now_ns) {
    anchor_ns += (int64_t)NS_PER_SECOND;
    target_ns = anchor_ns + (int64_t)offset_ns;
  }

  // These are one-shot phase-offset events.  The 1 Hz WITNESS_SCHEDULER
  // re-arms the next cycle.  This preserves explicit offsets across TimePop
  // epoch changes instead of letting same-period recurring slots collapse onto
  // the same PPS/VCLOCK boundary.
  timepop_cancel_by_name(name);
  const timepop_handle_t h =
      timepop_arm_from_anchor(anchor_ns,
                              (int64_t)offset_ns,
                              false,
                              callback,
                              nullptr,
                              name);
  return h != TIMEPOP_INVALID_HANDLE;
}

static bool witness_arm_periodic_clients(void) {
  if (!g_witness_runtime_ready || !g_witness_running) return false;

  witness_cancel_periodic_callbacks();

  bool ok = true;
  ok = witness_arm_one_shot_at_offset(GPIO_HIGH_NAME,
                                       GPIO_HIGH_OFFSET_NS,
                                       gpio_high_callback) && ok;
  ok = witness_arm_one_shot_at_offset(GPIO_LOW_NAME,
                                       GPIO_LOW_OFFSET_NS,
                                       gpio_low_callback) && ok;
  ok = witness_arm_one_shot_at_offset(WITNESS_BRIDGE_NAME,
                                       BRIDGE_SLOT_OFFSET_NS,
                                       witness_bridge_callback) && ok;

  if (ok) {
    g_witness_schedule_armed = true;
    g_bridge_armed = true;
    g_bridge_arm_count++;
    const time_anchor_snapshot_t snap = time_anchor_snapshot();
    g_bridge_last_armed_target_counter32 =
        snap.counter32_at_pps_vclock + (uint32_t)(BRIDGE_SLOT_OFFSET_NS / 100ULL);
    g_witness_cycle_reschedules++;
  } else {
    g_witness_schedule_armed = false;
    g_bridge_armed = false;
    g_bridge_arm_failures++;
  }

  return ok;
}

static bool witness_ensure_scheduled(void) {
  if (!g_witness_runtime_ready || !g_witness_running) return false;

  if (!g_witness_schedule_armed) {
    return witness_arm_periodic_clients();
  }

  if (!g_entry_pps.armed && !g_entry_pps.spin_active) {
    (void)entry_latency_arm_pps_spin();
  }

  return true;
}

static void witness_scheduler_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_witness_running) return;
  g_witness_cycle_count++;

  // The scheduler is the single recurring witness metronome.  It arms the
  // next cycle's phase-offset one-shots, so HIGH/LOW retain their intended
  // 100 ms / 600 ms positions instead of becoming independent recurring
  // slots that TimePop may legitimately re-phase onto the same boundary.
  (void)witness_arm_periodic_clients();
  if (!g_entry_pps.armed && !g_entry_pps.spin_active) {
    (void)entry_latency_arm_pps_spin();
  }
}

static void witness_arm_scheduler(void) {
  timepop_cancel_by_name(WITNESS_SCHEDULER_NAME);
  (void)timepop_arm(WITNESS_CYCLE_PERIOD_NS,
                    true,
                    witness_scheduler_callback,
                    nullptr,
                    WITNESS_SCHEDULER_NAME);
}

static void witness_start_always_on(void) {
  if (!g_witness_runtime_ready) return;

  g_witness_running = true;
  g_witness_start_count++;
  g_witness_last_start_dwt = ARM_DWT_CYCCNT;

  interrupt_register_pps_entry_latency_handler(entry_latency_pps_isr_handler);

  witness_arm_scheduler();
  (void)witness_ensure_scheduled();
  (void)entry_latency_arm_pps_spin();
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
  p.add_object("timepop", timepop_state);

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
  p.add_object("entry_latency", entry);

  p.add_object("clock_witness", ocxo_witness_state_payload());

  return p;
}

static void pps_phase_capture_from_timepop(const timepop_ctx_t& ctx) {
  const pps_t pps = witness_last_physical_pps_diag();
  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) {
    g_phase_skipped_sequence_mismatch++;
    return;
  }

  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  if (dynamic_cps == 0) {
    g_phase_skipped_no_dynamic_cps++;
    return;
  }

  const uint32_t selected_to_fire_ticks = ctx.fire_vclock_raw - pvc.counter32_at_edge;
  const uint64_t selected_to_fire_ns = (uint64_t)selected_to_fire_ticks * 100ULL;

  const uint32_t expected_offset_cycles =
      (uint32_t)(((uint64_t)selected_to_fire_ticks * (uint64_t)dynamic_cps +
                  (uint64_t)VCLOCK_TICKS_PER_SECOND_U32 / 2ULL) /
                 (uint64_t)VCLOCK_TICKS_PER_SECOND_U32);

  const uint32_t inferred_pvc_dwt = ctx.fire_dwt_cyccnt - expected_offset_cycles;
  const int32_t timepop_vs_pvc_phase =
      (int32_t)((int64_t)inferred_pvc_dwt - (int64_t)pvc.dwt_at_edge);

  const int32_t physical_pps_to_selected =
      (int32_t)((int64_t)pvc.dwt_at_edge - (int64_t)pps.dwt_at_edge);

  const double timepop_vs_pvc_ticks =
      ((double)timepop_vs_pvc_phase * (double)VCLOCK_TICKS_PER_SECOND_U32) /
      (double)dynamic_cps;
  const double physical_phase_ticks =
      ((double)physical_pps_to_selected * (double)VCLOCK_TICKS_PER_SECOND_U32) /
      (double)dynamic_cps;

  pps_phase_capture_t cap;
  cap.seq = ++g_phase_capture_seq;
  cap.valid = true;
  cap.pps_sequence = pps.sequence;
  cap.pvc_sequence = pvc.sequence;
  cap.pps_dwt_at_edge = pps.dwt_at_edge;
  cap.pvc_dwt_at_edge = pvc.dwt_at_edge;
  cap.pvc_counter32_at_edge = pvc.counter32_at_edge;
  cap.pvc_gnss_ns_at_edge = pvc.gnss_ns_at_edge;
  cap.timepop_deadline = ctx.deadline;
  cap.timepop_fire_vclock_raw = ctx.fire_vclock_raw;
  cap.timepop_fire_dwt = ctx.fire_dwt_cyccnt;
  cap.timepop_fire_gnss_ns = ctx.fire_gnss_ns;
  cap.selected_to_fire_ticks = selected_to_fire_ticks;
  cap.selected_to_fire_ns = selected_to_fire_ns;
  cap.dynamic_cps = dynamic_cps;
  cap.expected_offset_cycles = expected_offset_cycles;
  cap.inferred_pvc_dwt_from_timepop = inferred_pvc_dwt;
  cap.timepop_vs_pvc_phase_cycles = timepop_vs_pvc_phase;
  cap.timepop_vs_pvc_phase_ticks = timepop_vs_pvc_ticks;
  cap.physical_pps_to_selected_vclock_cycles = physical_pps_to_selected;
  cap.physical_pps_to_selected_vclock_ticks = physical_phase_ticks;

  g_phase_last = cap;
  g_phase_samples++;
  welford_update(g_phase_timepop_vs_pvc_welford, timepop_vs_pvc_phase);
  welford_update(g_phase_physical_pps_to_selected_welford, physical_pps_to_selected);
}

static void bridge_capture_from_timepop(const timepop_ctx_t& ctx) {
  const pps_vclock_t pvc = interrupt_last_pps_vclock();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) {
    g_bridge_skipped_no_anchor++;
    return;
  }

  const uint32_t qtimer_event_dwt = ctx.fire_dwt_cyccnt;
  const uint32_t authored_counter32 = ctx.fire_vclock_raw;

  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  if (dynamic_cps == 0) {
    g_bridge_skipped_no_dynamic_cps++;
  }

  pps_phase_capture_from_timepop(ctx);

  const uint32_t target_counter32 = ctx.deadline;
  const int32_t counter32_residual_ticks =
      (int32_t)((int64_t)authored_counter32 - (int64_t)target_counter32);

  const int64_t gnss_from_vclock_ns = ctx.fire_gnss_ns;

  const int64_t gnss_from_time_bridge_ns = (int64_t)time_gnss_ns_at_dwt(qtimer_event_dwt);
  const int64_t residual_ns = gnss_from_time_bridge_ns - gnss_from_vclock_ns;

  int64_t gnss_from_dynamic_dwt_ns = gnss_from_time_bridge_ns;
  int64_t dynamic_residual_ns = residual_ns;

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

static void witness_bridge_callback(timepop_ctx_t* ctx, timepop_diag_t*, void*) {
  if (!g_witness_running || !ctx) return;
  g_bridge_armed = true;
  bridge_capture_from_timepop(*ctx);
}

static Payload cmd_bridge(const Payload&) {
  (void)witness_ensure_scheduled();

  Payload p;
  p.add("model", "DWT_GNSS_BRIDGE_VS_TIMEPOP_VCLOCK_TRUTH");
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
  event.add("source", "TIMEPOP");
  event.add("target_counter32", cap.target_counter32);
  event.add("counter32_at_fire", cap.counter32_at_fire);
  event.add("counter32_residual_ticks", cap.counter32_residual_ticks);
  event.add("dwt_at_fire", cap.dwt_at_fire);
  p.add_object("event", event);

  Payload truth;
  truth.add("gnss_from_timepop_vclock_ns", cap.gnss_from_vclock_ns);
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
  (void)witness_ensure_scheduled();

  const pps_phase_capture_t cap = g_phase_last;

  Payload p;
  p.add("model", "PPS_TO_SELECTED_VCLOCK_TIMEPOP_PHASE");
  p.add("valid", cap.valid);
  p.add("seq", cap.seq);
  p.add("samples", g_phase_samples);
  p.add("skipped_no_pps_raw", g_phase_skipped_no_pps_raw);
  p.add("skipped_sequence_mismatch", g_phase_skipped_sequence_mismatch);
  p.add("skipped_no_dynamic_cps", g_phase_skipped_no_dynamic_cps);
  p.add("no_ch1", true);

  Payload anchor;
  anchor.add("pps_sequence", cap.pps_sequence);
  anchor.add("pvc_sequence", cap.pvc_sequence);
  anchor.add("pps_dwt_at_edge", cap.pps_dwt_at_edge);
  anchor.add("pvc_dwt_at_edge", cap.pvc_dwt_at_edge);
  anchor.add("pvc_counter32_at_edge", cap.pvc_counter32_at_edge);
  anchor.add("pvc_gnss_ns_at_edge", cap.pvc_gnss_ns_at_edge);
  p.add_object("anchor", anchor);

  Payload event;
  event.add("source", "TIMEPOP");
  event.add("deadline", cap.timepop_deadline);
  event.add("fire_vclock_raw", cap.timepop_fire_vclock_raw);
  event.add("fire_dwt", cap.timepop_fire_dwt);
  event.add("fire_gnss_ns", cap.timepop_fire_gnss_ns);
  p.add_object("event", event);

  Payload selection;
  selection.add("selected_to_fire_ticks", cap.selected_to_fire_ticks);
  selection.add("selected_to_fire_ns", cap.selected_to_fire_ns);
  selection.add("expected_offset_cycles", cap.expected_offset_cycles);
  selection.add("expected_offset_ns", cycles_to_ns((double)cap.expected_offset_cycles));
  selection.add("dynamic_cps", cap.dynamic_cps);
  selection.add("inferred_pvc_dwt_from_timepop", cap.inferred_pvc_dwt_from_timepop);
  p.add_object("selection", selection);

  Payload inferred;
  inferred.add("timepop_vs_pvc_phase_cycles", cap.timepop_vs_pvc_phase_cycles);
  inferred.add("timepop_vs_pvc_phase_ns", cycles_to_ns((double)cap.timepop_vs_pvc_phase_cycles));
  inferred.add("timepop_vs_pvc_phase_ticks", cap.timepop_vs_pvc_phase_ticks);
  inferred.add_object("timepop_vs_pvc_welford", welford_payload(g_phase_timepop_vs_pvc_welford));
  inferred.add("physical_pps_to_selected_vclock_cycles", cap.physical_pps_to_selected_vclock_cycles);
  inferred.add("physical_pps_to_selected_vclock_ns", cycles_to_ns((double)cap.physical_pps_to_selected_vclock_cycles));
  inferred.add("physical_pps_to_selected_vclock_ticks", cap.physical_pps_to_selected_vclock_ticks);
  inferred.add_object("physical_welford", welford_payload(g_phase_physical_pps_to_selected_welford));
  p.add_object("inferred", inferred);

  return p;
}

// ============================================================================
// GPIO_DELAY command
// ============================================================================
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
  (void)witness_ensure_scheduled();

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
  (void)witness_ensure_scheduled();
  if (g_witness_running && g_witness_runtime_ready &&
      !g_entry_pps.armed && !g_entry_pps.spin_active) {
    (void)entry_latency_arm_pps_spin();
  }

  Payload p;
  p.add("model", "ISR_ENTRY_LATENCY_FROM_SPIN_SHADOW");
  p.add("lead_ns", ENTRY_LATENCY_LEAD_NS);
  p.add("timeout_cycles", ENTRY_LATENCY_TIMEOUT_CYCLES);
  p.add("timeout_ns", cycles_to_ns((double)ENTRY_LATENCY_TIMEOUT_CYCLES));
  p.add_object("pps", entry_latency_payload(g_entry_pps));
  return p;
}

// ============================================================================
// REPORT command
// ============================================================================

static Payload cmd_report(const Payload&) {
  (void)witness_ensure_scheduled();

  Payload p;
  p.add("model", "WITNESS_ALWAYS_ON_HARDWARE_AND_TIMEPOP_STATE");
  p.add_object("state", witness_state_payload());
  return p;
}

// EDGE command
// ============================================================================
// EDGE command
// ============================================================================
//
// EDGE answers: how many PPS edges have been seen, and what were the facts
// of the last edge?  process_interrupt remains the author of PPS/PPS_VCLOCK
// truth; process_witness owns only this report surface.

static Payload cmd_edge(const Payload&) {
  const interrupt_pps_edge_heartbeat_t hb = interrupt_pps_edge_heartbeat();
  const pps_t pps = witness_last_physical_pps_diag();
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
  (void)witness_ensure_scheduled();

  Payload p;
  p.add("model", "GPIO_ROUND_TRIP_LATENCY_SUMMARY");

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

  welford_reset(g_source_welford);
  welford_reset(g_gpio_welford);
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

  g_phase_last = pps_phase_capture_t{};
  g_phase_capture_seq = 0;
  g_phase_pps_sequence = 0;
  g_phase_pps_isr_entry_dwt_raw = 0;
  g_phase_samples = 0;
  g_phase_skipped_no_pps_raw = 0;
  g_phase_skipped_sequence_mismatch = 0;
  g_phase_skipped_no_dynamic_cps = 0;
  welford_reset(g_phase_timepop_vs_pvc_welford);
  welford_reset(g_phase_physical_pps_to_selected_welford);

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

  ocxo_witness_bind_lanes();
  ocxo_witness_lane_reset_runtime(g_ocxo_witness_1);
  ocxo_witness_lane_reset_runtime(g_ocxo_witness_2);
  g_ocxo_witness_running = false;
  g_ocxo_witness_active_mask = 0;
  g_ocxo_witness_minder_armed = false;
  g_ocxo_witness_edge_asap_requested = false;
  g_ocxo_witness_publish_asap_requested = false;
  g_ocxo_witness_vector_mask = 0;
  g_ocxo_witness_start_count = 0;
  g_ocxo_witness_stop_count = 0;
  g_ocxo_witness_minder_fire_count = 0;
  g_ocxo_witness_minder_arm_count = 0;
  g_ocxo_witness_minder_arm_failures = 0;
  g_ocxo_witness_minder_divider = 0;
  g_ocxo_witness_publish_request_count = 0;
  g_ocxo_witness_publish_count = 0;
  g_ocxo_witness_publish_asap_failures = 0;
  g_ocxo_witness_edge_asap_count = 0;
  vclock_witness_reset_runtime();

  // Witness is always-on.  PPS entry latency remains a true low-level
  // witness hook; BRIDGE/PPS_PHASE are ordinary TimePop clients.  The
  // optional VCLOCK witness registers QTimer1 CH1 only while VCLOCK_START
  // is active.
  interrupt_register_pps_entry_latency_handler(entry_latency_pps_isr_handler);

  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  // GPIO is attached only inside the GPIO measurement window.
  pinMode(WITNESS_GPIO_PIN, INPUT);
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));

  g_witness_running = true;
  g_witness_schedule_armed = false;
  g_witness_runtime_ready = true;

  witness_start_always_on();
}

void process_witness_enable_irqs(void) {
  if (g_witness_irqs_enabled) return;

  g_witness_irqs_enabled = true;
}

static const process_command_entry_t WITNESS_COMMANDS[] = {
  { "REPORT",      cmd_report      },
  { "EDGE",        cmd_edge        },
  { "BRIDGE",      cmd_bridge      },
  { "PPS_PHASE",   cmd_pps_phase   },
  { "GPIO_DELAY",  cmd_gpio_delay  },
  { "QTIMER_READ", cmd_qtimer_read },
  { "DWT_READ",    cmd_dwt_read    },
  { "ENTRY_LATENCY", cmd_entry_latency },
  { "ROUND_TRIP",  cmd_round_trip  },
  { "OCXO_START",  cmd_ocxo_start  },
  { "OCXO_STOP",   cmd_ocxo_stop   },
  { "VCLOCK_START", cmd_vclock_start },
  { "VCLOCK_STOP",  cmd_vclock_stop  },
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
