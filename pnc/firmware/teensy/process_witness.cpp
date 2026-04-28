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
//   ROUND_TRIP — reports source, GPIO sink, and QTimer sink latency stats.
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
// Forward declarations
// ============================================================================

void witness_gpio_isr(void);

static void qtimer2_isr(void);
static void gpio_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer_high_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer_low_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void witness_cycle_callback(timepop_ctx_t*, timepop_diag_t*, void*);
static void qtimer2_arm_next_edge(void);

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

// ============================================================================
// GPIO sink ISR — old-style stable GPIO sink, rising edge only
// ============================================================================

void witness_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;

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
  g_active_window = witness_window_t::GPIO;
  pinMode(WITNESS_GPIO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN), witness_gpio_isr, RISING);
  witness_drive_high();
}

static void gpio_low_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  witness_drive_low();
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  g_active_window = witness_window_t::NONE;
}

static void qtimer_high_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));
  g_active_window = witness_window_t::QTIMER;
  witness_drive_high();
}

static void qtimer_low_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
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
}

static void witness_schedule_first_test(void) {
  if (g_witness_schedule_armed) return;

  timepop_cancel_by_name(WITNESS_CYCLE_NAME);
  witness_cancel_edge_callbacks();

  timepop_arm(WITNESS_CYCLE_PERIOD_NS,
              true,
              witness_cycle_callback,
              nullptr,
              WITNESS_CYCLE_NAME);

  g_witness_schedule_armed = true;

  // Arm the first second immediately; subsequent seconds are regenerated by
  // the recurring supervisor.  This avoids chains of child callbacks.
  g_witness_cycle_count++;
  witness_arm_cycle_edges();
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
  if (!g_witness_schedule_armed && g_witness_runtime_ready) {
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


  digitalWriteFast(WITNESS_STIMULUS_PIN, LOW);

  // Stable QTimer rail: configured once and left continuously armed.
  // GPIO is attached only inside the GPIO measurement window so it cannot
  // preempt the QTimer measurement window.
  pinMode(WITNESS_GPIO_PIN, INPUT);
  detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_PIN));

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
  { "EDGE",       cmd_edge       },
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
