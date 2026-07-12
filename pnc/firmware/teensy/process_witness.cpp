// ============================================================================
// process_witness.cpp
// ============================================================================
//
// process_witness owns local timing witness instrumentation.  The local
// hardware ROUND_TRIP rail remains intentionally low-level:
//
//   • Stimulus source pin  — TimePop callback drives HIGH/LOW.
//   • GPIO sink            — GPIO interrupt sees the source rising edge.
//   • GPT witness sink     — GPT2 external-clock/OCR1 compare ISR
//                            sees the same stimulus edge through timer fabric.
//
// QTimer2 is no longer used by witness.  OCXO1 now owns QTimer2 CH0 through
// process_interrupt, so witness preserves the GPIO round-trip path plus a
// narrow GPT external-clock compare witness for timer-fabric latency tests.
//
// The PPS/VCLOCK bridge and phase witness no longer use QTimer1 CH1 or any
// process_interrupt-hosted compare plumbing.  They are ordinary TimePop
// scheduled clients.  TimePop authors one shared fire fact for each scheduled
// event: fire_vclock_raw, fire_dwt_cyccnt, and fire_gnss_ns.  Witness consumes
// those facts exactly like any other TimePop client.
//
// Witness is always-on after initialization.  There are no START/STOP commands;
// the process is safe to run continuously because the operational bridge and
// phase reports are TimePop clients on the canonical PPS/VCLOCK timing rail.
//
// Current command surface:
//   REPORT        — lifecycle/hardware/TimePop state.
//   EDGE          — PPS/PPS_VCLOCK heartbeat + last edge facts.
//   BRIDGE        — DWT/GNSS bridge check using TimePop fire facts.
//   GPIO_DELAY    — simple digitalWriteFast HIGH stimulus cost.
//   QTIMER_READ   — GPIO ISR-to-VCLOCK counter observation delay/cost.
//   GPT_STIM      — stimulus-to-GPT external-clock/OCR compare ISR latency.
//   DWT_READ      — consecutive DWT read-cost measurement.
//   ENTRY_LATENCY — spin-shadow ISR entry latency for PPS.
//   PPS_PHASE     — PPS/VCLOCK phase inferred from TimePop witness events.
//   ROUND_TRIP    — source-to-GPIO ISR latency stats.
//
// Test cadence:
//   • 100 ms: GPIO HIGH   (visible 1 Hz heartbeat; away from PPS)
//              The same rising edge may be wired to GPIO and GPT witness input.
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

// GPT external-clock witness.
//
// Wire WITNESS_STIMULUS_PIN to WITNESS_GPIO_PIN and WITNESS_GPT_PIN.  The
// witness uses GPT2 OCR1 for CNT+1 immediately before driving the stimulus
// HIGH.  The rising edge should increment GPT2's external-clock counter and
// fire the OCR1 compare interrupt.
//
// Pin 25 is intentionally adjacent to the existing A10/A12 witness pair.  It
// must be muxed to the GPT2 external clock input on the target Teensy build.
// If the board revision uses a different exposed GPTx_CLK pad, only these
// constants and witness_gpt_configure_pin() should change.
static constexpr int  WITNESS_GPT_PIN = 14;
static constexpr const char* WITNESS_GPT_PROVIDER = "GPT2_OCR1_EXTCLK";
static constexpr uint32_t WITNESS_GPT_IRQ_PRIORITY = 32;

// GPT register-bit definitions are supplied by the Teensy core's imxrt.h.

static constexpr uint64_t GPIO_HIGH_OFFSET_NS   = 500500000ULL;  // PPS + 500.500 ms quiet-phase stimulus
static constexpr uint64_t GPIO_LOW_OFFSET_NS    = 600500000ULL;  // 100 ms high interval after quiet-phase stimulus
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
static constexpr uint32_t WITNESS_GPT_COMPARE_LEAD_TICKS = 1U;

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

enum class witness_window_t : uint8_t { NONE = 0, GPIO, GPIO_GPT };
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
// GPT_STIM — stimulus-to-GPT external-clock compare ISR latency
// ============================================================================
//
// This is the first GPT custody probe.  A TimePop callback arms GPT1 OCR1 for
// CNT+1, then drives WITNESS_STIMULUS_PIN HIGH.  If the stimulus pin is wired
// to WITNESS_GPT_PIN and that pin is muxed to GPT2 external clock, the rising
// edge increments GPT2 CNT and should immediately satisfy OCR1.
//
// The GPT ISR captures ARM_DWT_CYCCNT as its first instruction.  The report
// compares that timestamp to the same stimulus DWT used by the GPIO round-trip
// report.  This is intentionally parallel to the OCXO question: can a timer
// compare event deliver a stable ISR-entry DWT when the timer source is a
// stimulus edge?

static volatile bool     g_gpt_configured = false;
static volatile bool     g_gpt_armed = false;
static volatile uint32_t g_gpt_arm_count = 0;
static volatile uint32_t g_gpt_arm_fail_count = 0;
static volatile uint32_t g_gpt_hits = 0;
static volatile uint32_t g_gpt_false_irq = 0;
static volatile uint32_t g_gpt_wrong_window = 0;
static volatile uint32_t g_gpt_sr_at_isr = 0;
static volatile uint32_t g_gpt_cnt_at_arm = 0;
static volatile uint32_t g_gpt_expected_counter = 0;
static volatile uint32_t g_gpt_cnt_at_isr = 0;
static volatile int32_t  g_gpt_counter_residual_ticks = 0;
static volatile uint32_t g_gpt_dwt_at_arm = 0;
static volatile uint32_t g_gpt_dwt_at_isr = 0;
static volatile uint32_t g_gpt_delta_cycles = 0;
static volatile uint32_t g_gpt_entry_to_counter_read_cycles = 0;
static welford_t g_gpt_welford = {};
static welford_t g_gpt_counter_residual_welford = {};
static welford_t g_gpt_entry_to_counter_read_welford = {};


// GPT OCR1 forensics.  These are deliberately redundant register snapshots so
// GPT_STIM can tell whether the counter, compare register, status flag, and
// interrupt mask agree.
static volatile uint32_t g_gpt_live_cnt = 0;
static volatile uint32_t g_gpt_live_sr = 0;
static volatile uint32_t g_gpt_live_ir = 0;
static volatile uint32_t g_gpt_live_cr = 0;
static volatile uint32_t g_gpt_live_pr = 0;
static volatile uint32_t g_gpt_live_ocr1 = 0;
static volatile int32_t  g_gpt_live_ocr1_delta = 0;
static volatile bool     g_gpt_live_of1_pending = false;
static volatile bool     g_gpt_live_of1_irq_enabled = false;
static volatile uint32_t g_gpt_compare_flag_seen_poll_count = 0;
static volatile uint32_t g_gpt_compare_flag_clear_poll_count = 0;
static volatile uint32_t g_gpt_compare_missed_poll_count = 0;
static volatile bool     g_gpt_ladder_mode = true;
static volatile uint32_t g_gpt_ladder_step_ticks = 1;
static volatile uint32_t g_gpt_ladder_rearm_count = 0;
static volatile uint32_t g_gpt_ladder_last_prev_ocr1 = 0;
static volatile uint32_t g_gpt_ladder_last_next_ocr1 = 0;
static volatile int32_t  g_gpt_ladder_last_cnt_minus_prev_ocr1 = 0;
static volatile uint32_t g_gpt_arm_too_close_count = 0;
static volatile uint32_t g_gpt_arm_cnt_after_ocr_count = 0;
static volatile uint32_t g_gpt_arm_cnt_equal_ocr_count = 0;
static volatile uint32_t g_gpt_arm_live_sr_after_arm = 0;
static volatile uint32_t g_gpt_arm_live_ir_after_arm = 0;
static volatile uint32_t g_gpt_arm_live_cr_after_arm = 0;
static volatile uint32_t g_gpt_arm_live_cnt_after_arm = 0;



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
// Forward declarations
// ============================================================================

void witness_gpio_isr(void);
void witness_gpt2_isr(void);

static void gpio_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_scheduler_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_bridge_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_spin_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void entry_latency_pps_isr_handler(uint32_t sequence, uint32_t isr_entry_dwt_raw);
static bool entry_latency_arm_pps_spin(void);
static bool witness_ensure_scheduled(void);
static Payload witness_state_payload(void);


// ============================================================================
// GPT_STIM helpers
// ============================================================================


static inline void witness_gpt_refresh_live_registers(void) {
  const uint32_t cnt = GPT2_CNT;
  const uint32_t sr = GPT2_SR;
  const uint32_t ir = GPT2_IR;
  const uint32_t cr = GPT2_CR;
  const uint32_t pr = GPT2_PR;
  const uint32_t ocr1 = GPT2_OCR1;

  g_gpt_live_cnt = cnt;
  g_gpt_live_sr = sr;
  g_gpt_live_ir = ir;
  g_gpt_live_cr = cr;
  g_gpt_live_pr = pr;
  g_gpt_live_ocr1 = ocr1;
  g_gpt_live_ocr1_delta = (int32_t)((int64_t)ocr1 - (int64_t)cnt);
  g_gpt_live_of1_pending = (sr & GPT_SR_OF1) != 0;
  g_gpt_live_of1_irq_enabled = (ir & GPT_IR_OF1IE) != 0;

  if (g_gpt_live_of1_pending) {
    g_gpt_compare_flag_seen_poll_count++;
  }
}

static void witness_gpt_configure_pin(void) {
  // Known-good POC route:
  //   Teensy pin 14
  //   Pad GPIO_AD_B1_02
  //   ALT8 -> GPT2_CLK
  //
  // The SELECT_INPUT daisy is mandatory; without it GPT2_CNT does not see
  // the external edge even though the pad mux appears configured.
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;

  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;
}

static void witness_gpt_init_hardware(void) {
  if (g_gpt_configured) return;

  witness_gpt_configure_pin();

  // Old proven POC pattern:
  //   GPT2 external clock via pin 14
  //   FRR set
  //   OCR1 initialized before EN
  //   IRQ enabled continuously
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

  GPT2_CR = 0;
  GPT2_SR = 0x3Fu;
  GPT2_PR = 0;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;

  const uint32_t cnt = GPT2_CNT;
  const uint32_t target = cnt + WITNESS_GPT_COMPARE_LEAD_TICKS;
  GPT2_OCR1 = target;
  GPT2_SR = GPT_SR_OF1;
  GPT2_IR |= GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT2, witness_gpt2_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, WITNESS_GPT_IRQ_PRIORITY);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;

  g_gpt_cnt_at_arm = cnt;
  g_gpt_expected_counter = target;
  g_gpt_dwt_at_arm = ARM_DWT_CYCCNT;
  g_gpt_armed = true;
  g_gpt_arm_count++;
  g_gpt_configured = true;

  witness_gpt_refresh_live_registers();
}

void witness_gpt2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  const uint32_t sr = GPT2_SR;

  GPT2_SR = GPT_SR_OF1;

  if ((sr & GPT_SR_OF1) == 0) {
    g_gpt_false_irq++;
    witness_gpt_refresh_live_registers();
    return;
  }

  const uint32_t before_counter = ARM_DWT_CYCCNT;
  const uint32_t cnt = GPT2_CNT;
  const uint32_t after_counter = ARM_DWT_CYCCNT;

  const uint32_t prev_ocr1 = GPT2_OCR1;
  uint32_t next_ocr1 = prev_ocr1;

  // Keep OCR1 strictly in the future.  The old POC used OCR1 += 1, which is
  // fine when the ISR lands before CNT reaches the next rung.  Our witness has
  // shown CNT can already equal/pass that rung, so advance until the target is
  // ahead of the observed GPT count.
  do {
    next_ocr1 += g_gpt_ladder_step_ticks;
  } while ((int32_t)((int64_t)next_ocr1 - (int64_t)cnt) <= 0);

  GPT2_OCR1 = next_ocr1;

  const int32_t counter_residual =
      (int32_t)((int64_t)cnt - (int64_t)prev_ocr1);
  const uint32_t entry_to_counter_read = after_counter - isr_entry_dwt_raw;

  g_gpt_sr_at_isr = sr;
  g_gpt_dwt_at_isr = isr_entry_dwt_raw;
  g_gpt_cnt_at_isr = cnt;
  g_gpt_counter_residual_ticks = counter_residual;
  g_gpt_entry_to_counter_read_cycles = entry_to_counter_read;
  g_gpt_armed = true;

  g_gpt_ladder_rearm_count++;
  g_gpt_ladder_last_prev_ocr1 = prev_ocr1;
  g_gpt_ladder_last_next_ocr1 = next_ocr1;
  g_gpt_ladder_last_cnt_minus_prev_ocr1 = counter_residual;
  g_gpt_expected_counter = next_ocr1;

  welford_update(g_gpt_entry_to_counter_read_welford,
                 (int32_t)entry_to_counter_read);
  welford_update(g_gpt_counter_residual_welford, counter_residual);

  if (!g_witness_running ||
      (g_active_window != witness_window_t::GPIO_GPT &&
       g_active_window != witness_window_t::GPIO)) {
    g_gpt_wrong_window++;
    witness_gpt_refresh_live_registers();
    return;
  }

  const uint32_t delta_cycles = isr_entry_dwt_raw - g_source_dwt_at_emit;
  g_gpt_delta_cycles = delta_cycles;
  g_gpt_hits++;
  welford_update(g_gpt_welford, (int32_t)delta_cycles);

  (void)before_counter;
  witness_gpt_refresh_live_registers();
}

// ============================================================================
// Source drive
// ============================================================================

static void witness_drive_high(void) {
  const uint32_t dwt_before = ARM_DWT_CYCCNT;

  // Publish the common stimulus timestamp before driving the pin.  GPIO/GPT
  // ISRs can then subtract against the correct edge attempt even if they
  // arrive before this function completes.
  g_source_dwt_before = dwt_before;
  g_source_dwt_at_emit = dwt_before;

  digitalWriteFast(WITNESS_STIMULUS_PIN, HIGH);
  const uint32_t dwt_after = ARM_DWT_CYCCNT;

  g_source_dwt_after = dwt_after;
  g_source_stim_cycles = dwt_after - dwt_before;
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

  if (g_active_window != witness_window_t::GPIO &&
      g_active_window != witness_window_t::GPIO_GPT) {
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

  g_active_window = witness_window_t::GPIO_GPT;
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
    case witness_window_t::GPIO:     return "GPIO";
    case witness_window_t::GPIO_GPT: return "GPIO_GPT";
    default:                         return "NONE";
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
  timepop_state.add("gpio_high_offset_ns", GPIO_HIGH_OFFSET_NS);
  timepop_state.add("gpio_low_offset_ns", GPIO_LOW_OFFSET_NS);
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

  Payload gpt;
  gpt.add("enabled", true);
  gpt.add("configured", g_gpt_configured);
  gpt.add("provider", WITNESS_GPT_PROVIDER);
  gpt.add("stimulus_pin", (uint32_t)WITNESS_STIMULUS_PIN);
  gpt.add("gpt_pin", (uint32_t)WITNESS_GPT_PIN);
  gpt.add("armed", g_gpt_armed);
  gpt.add("arm_count", g_gpt_arm_count);
  gpt.add("arm_fail_count", g_gpt_arm_fail_count);
  gpt.add("hits", g_gpt_hits);
  gpt.add("false_irq", g_gpt_false_irq);
  gpt.add("wrong_window", g_gpt_wrong_window);
  witness_gpt_refresh_live_registers();
  gpt.add("live_cnt", g_gpt_live_cnt);
  gpt.add("live_sr", g_gpt_live_sr);
  gpt.add("live_ir", g_gpt_live_ir);
  gpt.add("live_ocr1", g_gpt_live_ocr1);
  gpt.add("of1_pending", g_gpt_live_of1_pending);
  gpt.add("of1_irq_enabled", g_gpt_live_of1_irq_enabled);
  gpt.add("compare_missed_poll_count", g_gpt_compare_missed_poll_count);
  p.add_object("gpt", gpt);

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
// GPT_STIM command
// ============================================================================

static Payload cmd_gpt_stim(const Payload&) {
  witness_gpt_refresh_live_registers();
  (void)witness_ensure_scheduled();

  Payload p;
  p.add("model", "GPT_EXTERNAL_CLOCK_COMPARE_STIMULUS");
  p.add("enabled", true);
  p.add("provider", WITNESS_GPT_PROVIDER);
  p.add("stimulus_pin", (uint32_t)WITNESS_STIMULUS_PIN);
  p.add("gpio_pin", (uint32_t)WITNESS_GPIO_PIN);
  p.add("gpt_pin", (uint32_t)WITNESS_GPT_PIN);
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);
  p.add("configured", g_gpt_configured);
  p.add("armed", g_gpt_armed);
  p.add("arm_count", g_gpt_arm_count);
  p.add("arm_fail_count", g_gpt_arm_fail_count);
  p.add("hits", g_gpt_hits);
  p.add("false_irq", g_gpt_false_irq);
  p.add("wrong_window", g_gpt_wrong_window);

  Payload last;
  last.add("source_dwt_at_emit", g_source_dwt_at_emit);
  last.add("dwt_at_arm", g_gpt_dwt_at_arm);
  last.add("dwt_at_isr", g_gpt_dwt_at_isr);
  last.add("delta_cycles", g_gpt_delta_cycles);
  last.add("delta_ns", cycles_to_ns((double)g_gpt_delta_cycles));
  last.add("cnt_at_arm", g_gpt_cnt_at_arm);
  last.add("expected_counter", g_gpt_expected_counter);
  last.add("cnt_at_isr", g_gpt_cnt_at_isr);
  last.add("counter_residual_ticks", g_gpt_counter_residual_ticks);
  last.add("sr_at_isr", g_gpt_sr_at_isr);
  last.add("entry_to_counter_read_cycles", g_gpt_entry_to_counter_read_cycles);
  last.add("entry_to_counter_read_ns", cycles_to_ns((double)g_gpt_entry_to_counter_read_cycles));
  p.add_object("last", last);

  p.add_object("latency", welford_payload(g_gpt_welford));
  p.add_object("counter_residual", welford_payload(g_gpt_counter_residual_welford));
  p.add_object("entry_to_counter_read", welford_payload(g_gpt_entry_to_counter_read_welford));

  Payload live;
  live.add("cnt", g_gpt_live_cnt);
  live.add("sr", g_gpt_live_sr);
  live.add("ir", g_gpt_live_ir);
  live.add("cr", g_gpt_live_cr);
  live.add("pr", g_gpt_live_pr);
  live.add("ocr1", g_gpt_live_ocr1);
  live.add("ocr1_delta", g_gpt_live_ocr1_delta);
  live.add("of1_pending", g_gpt_live_of1_pending);
  live.add("of1_irq_enabled", g_gpt_live_of1_irq_enabled);
  live.add("compare_flag_seen_poll_count", g_gpt_compare_flag_seen_poll_count);
  live.add("compare_flag_clear_poll_count", g_gpt_compare_flag_clear_poll_count);
  live.add("compare_missed_poll_count", g_gpt_compare_missed_poll_count);
  p.add_object("live", live);

  Payload arm;
  arm.add("too_close_count", g_gpt_arm_too_close_count);
  arm.add("cnt_equal_ocr_count", g_gpt_arm_cnt_equal_ocr_count);
  arm.add("cnt_after_ocr_count", g_gpt_arm_cnt_after_ocr_count);
  arm.add("cnt_after_arm", g_gpt_arm_live_cnt_after_arm);
  arm.add("sr_after_arm", g_gpt_arm_live_sr_after_arm);
  arm.add("ir_after_arm", g_gpt_arm_live_ir_after_arm);
  arm.add("cr_after_arm", g_gpt_arm_live_cr_after_arm);
  p.add_object("arm_forensics", arm);


  Payload ladder;
  ladder.add("mode", g_gpt_ladder_mode);
  ladder.add("step_ticks", g_gpt_ladder_step_ticks);
  ladder.add("rearm_count", g_gpt_ladder_rearm_count);
  ladder.add("last_prev_ocr1", g_gpt_ladder_last_prev_ocr1);
  ladder.add("last_next_ocr1", g_gpt_ladder_last_next_ocr1);
  ladder.add("last_cnt_minus_prev_ocr1", g_gpt_ladder_last_cnt_minus_prev_ocr1);
  p.add_object("ladder", ladder);

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
  p.add("gpio_high_offset_ns", GPIO_HIGH_OFFSET_NS);
  p.add("gpio_low_offset_ns", GPIO_LOW_OFFSET_NS);

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



  Payload gpt;
  gpt.add("enabled", true);
  gpt.add("configured", g_gpt_configured);
  gpt.add("armed", g_gpt_armed);
  gpt.add("provider", WITNESS_GPT_PROVIDER);
  gpt.add("stimulus_pin", (uint32_t)WITNESS_STIMULUS_PIN);
  gpt.add("gpt_pin", (uint32_t)WITNESS_GPT_PIN);
  gpt.add("hits", g_gpt_hits);
  gpt.add("false_irq", g_gpt_false_irq);
  gpt.add("wrong_window", g_gpt_wrong_window);
  gpt.add("last_cycles", g_gpt_delta_cycles);
  gpt.add("last_ns", cycles_to_ns((double)g_gpt_delta_cycles));
  gpt.add("counter_residual_ticks", g_gpt_counter_residual_ticks);
  gpt.add("entry_to_counter_read_cycles", g_gpt_entry_to_counter_read_cycles);
  gpt.add("entry_to_counter_read_ns",
          cycles_to_ns((double)g_gpt_entry_to_counter_read_cycles));
  gpt.add("cnt_at_arm", g_gpt_cnt_at_arm);
  gpt.add("expected_counter", g_gpt_expected_counter);
  gpt.add("cnt_at_isr", g_gpt_cnt_at_isr);
  gpt.add("sr_at_isr", g_gpt_sr_at_isr);
  gpt.add_object("welford", welford_payload(g_gpt_welford));
  gpt.add_object("counter_residual_welford",
                 welford_payload(g_gpt_counter_residual_welford));
  gpt.add_object("entry_to_counter_read_welford",
                 welford_payload(g_gpt_entry_to_counter_read_welford));
  p.add_object("gpt", gpt);

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

  witness_gpt_init_hardware();

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

  g_gpt_armed = g_gpt_configured;
  g_gpt_arm_count = 0;
  g_gpt_arm_fail_count = 0;
  g_gpt_hits = 0;
  g_gpt_false_irq = 0;
  g_gpt_wrong_window = 0;
  g_gpt_sr_at_isr = 0;
  g_gpt_cnt_at_arm = 0;
  g_gpt_expected_counter = 0;
  g_gpt_cnt_at_isr = 0;
  g_gpt_counter_residual_ticks = 0;
  g_gpt_dwt_at_arm = 0;
  g_gpt_dwt_at_isr = 0;
  g_gpt_delta_cycles = 0;
  g_gpt_entry_to_counter_read_cycles = 0;
  welford_reset(g_gpt_welford);
  welford_reset(g_gpt_counter_residual_welford);
  welford_reset(g_gpt_entry_to_counter_read_welford);
  witness_gpt_refresh_live_registers();

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

  // Witness is always-on.  PPS entry latency remains a true low-level
  // witness hook; BRIDGE/PPS_PHASE are ordinary TimePop clients and do not
  // register any QTimer1 CH1 handler.
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
  { "GPT_STIM",    cmd_gpt_stim    },
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
