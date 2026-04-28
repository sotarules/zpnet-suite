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
// This version intentionally mimics the old process_interrupt witness_latency
// architecture as closely as possible:
//
//   • QTimer2 CH0 is initialized once and left live.
//   • QTimer compare is armed as COMP1 = current CNTR + 1.
//   • The QTimer ISR clears TCF1, records the sample, then immediately
//     re-arms COMP1 = current CNTR + 1.
//   • No per-slot QTimer teardown.
//   • No SCTRL.TCFIE path.
//   • No reciprocal GPIO/QTimer ownership state.
//
// This is deliberately less elegant than the two-slot architecture, but it
// recreates the old known-good QTimer lifecycle: stable rail, self-rearm.
//
// Current command surface:
//   ROUND_TRIP — reports source, GPIO sink, and QTimer sink latency stats.
//
// Test cadence:
//   • Source HIGH starts ~250 ms after witness init, repeats every 1 s.
//   • Source LOW occurs 500 ms after HIGH.
//
// ============================================================================

#include "process_witness.h"

#include "process.h"
#include "payload.h"
#include "timepop.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>

// ============================================================================
// Hardware constants
// ============================================================================

static constexpr int WITNESS_STIMULUS_PIN = 24;
static constexpr int WITNESS_GPIO_PIN     = 26;
static constexpr int WITNESS_QTIMER_PIN   = 13;   // QTimer2 TIMER0 input

static constexpr uint64_t SOURCE_FIRST_HIGH_DELAY_NS = 250000000ULL;
static constexpr uint64_t SOURCE_HIGH_PERIOD_NS      = 1000000000ULL;
static constexpr uint64_t SOURCE_PULSE_WIDTH_NS      = 500000000ULL;

static constexpr const char* SOURCE_HIGH_NAME = "WITNESS_SOURCE_HIGH";
static constexpr const char* SOURCE_LOW_NAME  = "WITNESS_SOURCE_LOW";

static constexpr uint16_t QTIMER2_CH0_ENBL_MASK = 0x0001;

// These are report-only decodes.  The old working path did not enable or
// clear SCTRL.TCF; it relied on CSCTRL.TCF1 / CSCTRL.TCF1EN.
static constexpr uint16_t QTIMER_SCTRL_INPUT_MASK = 0x0100;
static constexpr uint16_t QTIMER_SCTRL_IEF_MASK   = 0x0800;
static constexpr uint16_t QTIMER_SCTRL_TCFIE_MASK = 0x4000;
static constexpr uint16_t QTIMER_SCTRL_TCF_MASK   = 0x8000;

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
static volatile bool g_witness_schedule_armed = false;

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

static volatile uint32_t g_qtimer_hits = 0;
static volatile uint32_t g_qtimer_irq_entries = 0;
static volatile uint32_t g_qtimer_no_flag = 0;
static volatile uint32_t g_qtimer_arms = 0;
static volatile uint32_t g_qtimer_dwt_at_isr = 0;
static volatile uint32_t g_qtimer_delta_cycles = 0;
static volatile uint16_t g_qtimer_compare_target = 0;

static welford_t g_source_welford = {};
static welford_t g_gpio_welford = {};
static welford_t g_qtimer_welford = {};

// Selected QTimer2 CH0 register snapshots.
struct qtimer2_snapshot_t {
  uint16_t cntr   = 0;
  uint16_t comp1  = 0;
  uint16_t cmpld1 = 0;
  uint16_t cmpld2 = 0;
  uint16_t csctrl = 0;
  uint16_t sctrl  = 0;
  uint16_t ctrl   = 0;
  uint16_t load   = 0;
  uint16_t enbl   = 0;
  uint16_t mux    = 0;
  uint16_t select_input = 0;
};

static qtimer2_snapshot_t g_qtimer_last_before_arm = {};
static qtimer2_snapshot_t g_qtimer_last_after_arm = {};
static qtimer2_snapshot_t g_qtimer_last_source_high = {};
static qtimer2_snapshot_t g_qtimer_last_source_low = {};
static qtimer2_snapshot_t g_qtimer_last_irq_entry = {};

// ============================================================================
// Forward declarations
// ============================================================================

void witness_gpio_isr(void);

static void qtimer2_isr(void);
static void source_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void source_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer2_arm_next_edge(void);

// ============================================================================
// Snapshot / payload helpers
// ============================================================================

static void qtimer2_capture_snapshot(qtimer2_snapshot_t& out) {
  out.cntr   = IMXRT_TMR2.CH[0].CNTR;
  out.comp1  = IMXRT_TMR2.CH[0].COMP1;
  out.cmpld1 = IMXRT_TMR2.CH[0].CMPLD1;
  out.cmpld2 = IMXRT_TMR2.CH[0].CMPLD2;
  out.csctrl = IMXRT_TMR2.CH[0].CSCTRL;
  out.sctrl  = IMXRT_TMR2.CH[0].SCTRL;
  out.ctrl   = IMXRT_TMR2.CH[0].CTRL;
  out.load   = IMXRT_TMR2.CH[0].LOAD;
  out.enbl   = IMXRT_TMR2.ENBL;
  out.mux    = (uint16_t)(*portConfigRegister(WITNESS_QTIMER_PIN));
  out.select_input = (uint16_t)IOMUXC_QTIMER2_TIMER0_SELECT_INPUT;
}

static void qtimer2_snapshot_payload(Payload& p, const qtimer2_snapshot_t& s) {
  p.add("cntr",   (uint32_t)s.cntr);
  p.add("comp1",  (uint32_t)s.comp1);
  p.add("cmpld1", (uint32_t)s.cmpld1);
  p.add("cmpld2", (uint32_t)s.cmpld2);
  p.add("csctrl", (uint32_t)s.csctrl);
  p.add("csctrl_tcf1",   (s.csctrl & TMR_CSCTRL_TCF1) != 0);
  p.add("csctrl_tcf1en", (s.csctrl & TMR_CSCTRL_TCF1EN) != 0);
  p.add("sctrl",  (uint32_t)s.sctrl);
  p.add("sctrl_input", (s.sctrl & QTIMER_SCTRL_INPUT_MASK) != 0);
  p.add("sctrl_tcf",   (s.sctrl & QTIMER_SCTRL_TCF_MASK) != 0);
  p.add("sctrl_tcfie", (s.sctrl & QTIMER_SCTRL_TCFIE_MASK) != 0);
  p.add("sctrl_ief",   (s.sctrl & QTIMER_SCTRL_IEF_MASK) != 0);
  p.add("ctrl",   (uint32_t)s.ctrl);
  p.add("load",   (uint32_t)s.load);
  p.add("enbl",   (uint32_t)s.enbl);
  p.add("mux",    (uint32_t)s.mux);
  p.add("select_input", (uint32_t)s.select_input);
}

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
  qtimer2_capture_snapshot(g_qtimer_last_before_arm);

  const uint16_t cntr = IMXRT_TMR2.CH[0].CNTR;
  const uint16_t target = (uint16_t)(cntr + 1);

  g_qtimer_compare_target = target;
  g_qtimer_arms++;

  qtimer2_ch0_program_compare(target);
  qtimer2_capture_snapshot(g_qtimer_last_after_arm);
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

  qtimer2_capture_snapshot(g_qtimer_last_source_high);
}

static void witness_drive_low(void) {
  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);
  g_source_lows++;
  g_source_high = false;

  qtimer2_capture_snapshot(g_qtimer_last_source_low);
}

// ============================================================================
// GPIO sink ISR — old-style stable GPIO sink, rising edge only
// ============================================================================

void witness_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
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

  qtimer2_capture_snapshot(g_qtimer_last_irq_entry);

  if (!(IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1)) {
    g_qtimer_no_flag++;
    return;
  }

  qtimer2_ch0_clear_compare_flag();

  const int32_t sample = (int32_t)(isr_entry_dwt_raw - g_source_dwt_at_emit);

  g_qtimer_dwt_at_isr = isr_entry_dwt_raw;
  g_qtimer_delta_cycles = (uint32_t)sample;
  g_qtimer_hits++;
  welford_update(g_qtimer_welford, sample);

  // This is the key old-process_interrupt behavior: immediately re-arm
  // against the continuously live counter from inside the ISR.
  qtimer2_arm_next_edge();
}

// ============================================================================
// TimePop waveform callbacks — one repeating source, not per-sink slots
// ============================================================================

static void source_high_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  witness_drive_high();
}

static void source_low_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  witness_drive_low();
}

static void witness_schedule_first_test(void) {
  if (g_witness_schedule_armed) return;

  timepop_cancel_by_name(SOURCE_HIGH_NAME);
  timepop_cancel_by_name(SOURCE_LOW_NAME);

  // Use two independent recurring timers instead of self-rescheduling from
  // inside an active callback. This makes the source waveform truly stable:
  // HIGH every second, LOW 500 ms later, forever.
  timepop_arm(SOURCE_FIRST_HIGH_DELAY_NS,
              true,
              source_high_callback,
              nullptr,
              SOURCE_HIGH_NAME);

  timepop_arm(SOURCE_FIRST_HIGH_DELAY_NS + SOURCE_PULSE_WIDTH_NS,
              true,
              source_low_callback,
              nullptr,
              SOURCE_LOW_NAME);

  g_witness_schedule_armed = true;
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

// ============================================================================
// ROUND_TRIP command
// ============================================================================

static Payload cmd_round_trip(const Payload&) {
  if (!g_witness_schedule_armed && g_witness_runtime_ready) {
    witness_schedule_first_test();
  }

  const uint32_t csctrl = (uint32_t)IMXRT_TMR2.CH[0].CSCTRL;
  const uint32_t sctrl  = (uint32_t)IMXRT_TMR2.CH[0].SCTRL;

  Payload p;
  p.add("model", "OLD_PROCESS_INTERRUPT_STABLE_RAIL_RECURRING_SOURCE");
  p.add("hardware_ready", g_witness_hw_ready);
  p.add("runtime_ready", g_witness_runtime_ready);
  p.add("irqs_enabled", g_witness_irqs_enabled);
  p.add("schedule_armed", g_witness_schedule_armed);
  p.add("source_high", g_source_high);

  Payload source;
  source.add("emits", g_source_emits);
  source.add("lows", g_source_lows);
  source.add("dwt_at_emit", g_source_dwt_at_emit);
  source.add("dwt_before", g_source_dwt_before);
  source.add("dwt_after", g_source_dwt_after);
  source.add("last_stim_cycles", g_source_stim_cycles);
  source.add("last_stim_ns", cycles_to_ns((double)g_source_stim_cycles));
  source.add_object("welford", welford_payload(g_source_welford));
  p.add_object("source", source);

  Payload gpio;
  gpio.add("hits", g_gpio_hits);
  gpio.add("last_dwt_at_isr", g_gpio_dwt_at_isr);
  gpio.add("last_delta_cycles", g_gpio_delta_cycles);
  gpio.add("last_delta_ns", cycles_to_ns((double)g_gpio_delta_cycles));
  gpio.add_object("welford", welford_payload(g_gpio_welford));
  p.add_object("gpio", gpio);

  Payload qtimer;
  qtimer.add("hits", g_qtimer_hits);
  qtimer.add("irq_entries", g_qtimer_irq_entries);
  qtimer.add("no_flag", g_qtimer_no_flag);
  qtimer.add("arms", g_qtimer_arms);
  qtimer.add("last_dwt_at_isr", g_qtimer_dwt_at_isr);
  qtimer.add("last_delta_cycles", g_qtimer_delta_cycles);
  qtimer.add("last_delta_ns", cycles_to_ns((double)g_qtimer_delta_cycles));
  qtimer.add("compare_target", (uint32_t)g_qtimer_compare_target);
  qtimer.add_object("welford", welford_payload(g_qtimer_welford));

  Payload registers;
  registers.add("cntr", (uint32_t)IMXRT_TMR2.CH[0].CNTR);
  registers.add("comp1", (uint32_t)IMXRT_TMR2.CH[0].COMP1);
  registers.add("cmpld1", (uint32_t)IMXRT_TMR2.CH[0].CMPLD1);
  registers.add("cmpld2", (uint32_t)IMXRT_TMR2.CH[0].CMPLD2);
  registers.add("csctrl", csctrl);
  registers.add("csctrl_tcf1", (csctrl & TMR_CSCTRL_TCF1) != 0);
  registers.add("csctrl_tcf1en", (csctrl & TMR_CSCTRL_TCF1EN) != 0);
  registers.add("ctrl", (uint32_t)IMXRT_TMR2.CH[0].CTRL);
  registers.add("sctrl", sctrl);
  registers.add("sctrl_input", (sctrl & QTIMER_SCTRL_INPUT_MASK) != 0);
  registers.add("sctrl_tcf",   (sctrl & QTIMER_SCTRL_TCF_MASK) != 0);
  registers.add("sctrl_tcfie", (sctrl & QTIMER_SCTRL_TCFIE_MASK) != 0);
  registers.add("sctrl_ief",   (sctrl & QTIMER_SCTRL_IEF_MASK) != 0);
  registers.add("enbl", (uint32_t)IMXRT_TMR2.ENBL);
  registers.add("select_input", (uint32_t)IOMUXC_QTIMER2_TIMER0_SELECT_INPUT);
  registers.add("mux", (uint32_t)*portConfigRegister(WITNESS_QTIMER_PIN));
  qtimer.add_object("registers", registers);

  Payload before_arm;
  qtimer2_snapshot_payload(before_arm, g_qtimer_last_before_arm);
  qtimer.add_object("before_arm", before_arm);

  Payload after_arm;
  qtimer2_snapshot_payload(after_arm, g_qtimer_last_after_arm);
  qtimer.add_object("after_arm", after_arm);

  Payload source_high_snap;
  qtimer2_snapshot_payload(source_high_snap, g_qtimer_last_source_high);
  qtimer.add_object("source_high", source_high_snap);

  Payload source_low_snap;
  qtimer2_snapshot_payload(source_low_snap, g_qtimer_last_source_low);
  qtimer.add_object("source_low", source_low_snap);

  Payload irq_entry;
  qtimer2_snapshot_payload(irq_entry, g_qtimer_last_irq_entry);
  qtimer.add_object("irq_entry", irq_entry);

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

  g_source_emits = 0;
  g_source_lows = 0;
  g_source_dwt_at_emit = 0;
  g_source_dwt_before = 0;
  g_source_dwt_after = 0;
  g_source_stim_cycles = 0;

  g_gpio_hits = 0;
  g_gpio_dwt_at_isr = 0;
  g_gpio_delta_cycles = 0;

  g_qtimer_hits = 0;
  g_qtimer_irq_entries = 0;
  g_qtimer_no_flag = 0;
  g_qtimer_arms = 0;
  g_qtimer_dwt_at_isr = 0;
  g_qtimer_delta_cycles = 0;
  g_qtimer_compare_target = 0;

  welford_reset(g_source_welford);
  welford_reset(g_gpio_welford);
  welford_reset(g_qtimer_welford);

  g_qtimer_last_before_arm = qtimer2_snapshot_t{};
  g_qtimer_last_after_arm = qtimer2_snapshot_t{};
  g_qtimer_last_source_high = qtimer2_snapshot_t{};
  g_qtimer_last_source_low = qtimer2_snapshot_t{};
  g_qtimer_last_irq_entry = qtimer2_snapshot_t{};

  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  // Old-style stable sinks: both are configured and left in place.
  pinMode(WITNESS_GPIO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN), witness_gpio_isr, RISING);

  qtimer2_arm_next_edge();

  g_witness_schedule_armed = false;
  g_witness_runtime_ready = true;

  witness_schedule_first_test();
}

void process_witness_enable_irqs(void) {
  if (g_witness_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);

  g_witness_irqs_enabled = true;
}

static const process_command_entry_t WITNESS_COMMANDS[] = {
  { "ROUND_TRIP", cmd_round_trip },
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
