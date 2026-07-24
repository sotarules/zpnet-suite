// ============================================================================
// process_interrupt.cpp
// ============================================================================
//
// process_interrupt owns only executable custody and observed edge facts.
//
// Four execution tiers are deliberate and runtime-reportable:
//
// Priority 0 is the sovereign science-capture tier:
//   * PPS GPIO, OCXO1, and OCXO2 capture ARM_DWT_CYCCNT before handler work;
//   * these sparse one-second sources may preempt every lower execution tier;
//   * each source is minimally defused and a bounded scalar packet is enqueued.
//
// Priority 16 owns the shared QTimer1 capture vector:
//   * native VCLOCK CH0 and TimePop CH2 necessarily share one NVIC priority;
//   * Priority 0 science captures may preempt this vector;
//   * QTimer1 captures/defuses only and pends Priority 32 continuation.
//
// Priority 32 owns continuation:
//   * the native VCLOCK heartbeat extends all three 16-bit timer domains;
//   * each OCXO owns exactly one authored compare target per 10,000,000 ticks;
//   * VCLOCK tends OCXO low-word lineage until the one-second target enters the
//     safe 16-bit compare-programming window;
//   * raw ISR-entry DWT is converted once to the calibrated event coordinate;
//   * every lawful OCXO compare authors one one-second event;
//   * immutable CH2 and subscriber work is transferred to foreground-owned
//     service without entering TimePop or invoking application behavior.
//
// Foreground owns TimePop scheduling policy and all application callbacks.
//
// There is no alternative endpoint estimator, repair candidate, FloorLine, or
// alternative publication court in this module.  OCXO compare custody is 1 Hz;
// ambient counter reads are rollover witnesses and never event truth.
//
// SpinIdle/predecessor/QTimer evidence is integrated into a parallel causal
// verdict: whether an ISR endpoint was delayed, who delayed it, and the signed
// delay contribution to the current interval.  The observed DWT endpoint is
// never overwritten by that verdict.
// ============================================================================

#include "process_interrupt.h"
#include "process_clocks.h"
#include "process_system.h"
#include "crash_forensics.h"

#include "config.h"
#include "debug.h"
#include "process.h"
#include "publish.h"
#include "payload.h"
#include "timepop.h"
#include "process_timepop.h"

#include <Arduino.h>
#include "imxrt.h"
#include <stdio.h>
#include <string.h>
#include <climits>

// process_clocks owns campaign surrender/science-reject semantics.
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3);
bool clocks_watchdog_campaign_armed(void);

void clocks_watchdog_anomaly_payload(const char* reason,
                                     const Payload& payload,
                                     uint32_t detail0,
                                     uint32_t detail1,
                                     uint32_t detail2,
                                     uint32_t detail3)
    __attribute__((weak));

__attribute__((weak)) void clocks_watchdog_anomaly_payload(
    const char* reason,
    const Payload& payload,
    uint32_t detail0,
    uint32_t detail1,
    uint32_t detail2,
    uint32_t detail3) {
  (void)reason;
  (void)detail0;
  (void)detail1;
  (void)detail2;
  (void)detail3;
  publish("WATCHDOG_ANOMALY", payload);
}

// ============================================================================
// Fixed doctrine
// ============================================================================

static constexpr uint32_t INTERRUPT_PRIORITY_SCIENCE = 0U;
static constexpr uint32_t INTERRUPT_PRIORITY_VCLOCK_TIMEPOP = 16U;
static constexpr uint32_t INTERRUPT_PRIORITY_CONTINUATION = 32U;
static constexpr uint32_t INTERRUPT_PRIORITY0_PRESERVING_BASEPRI =
    INTERRUPT_PRIORITY_VCLOCK_TIMEPOP;
static constexpr uint32_t INTERRUPT_HANDOFF_IRQ_NUMBER = 71U;

static_assert(INTERRUPT_PRIORITY_SCIENCE <
                  INTERRUPT_PRIORITY_VCLOCK_TIMEPOP &&
              INTERRUPT_PRIORITY_VCLOCK_TIMEPOP <
                  INTERRUPT_PRIORITY_CONTINUATION,
              "interrupt tiers must remain ordered 0 < 16 < 32");
static_assert((INTERRUPT_PRIORITY_SCIENCE & 0x0FU) == 0U &&
              (INTERRUPT_PRIORITY_VCLOCK_TIMEPOP & 0x0FU) == 0U &&
              (INTERRUPT_PRIORITY_CONTINUATION & 0x0FU) == 0U,
              "i.MX RT1062 priorities must use implemented upper-nibble steps");
static constexpr IRQ_NUMBER_t INTERRUPT_HANDOFF_IRQ =
    (IRQ_NUMBER_t)INTERRUPT_HANDOFF_IRQ_NUMBER;
static constexpr uint32_t INTERRUPT_HANDOFF_DRAIN_BUDGET = 32U;

static constexpr uint32_t HANDOFF_QTIMER1_RING_SIZE = 16U;
static constexpr uint32_t HANDOFF_OCXO_RING_SIZE = 4U;
static constexpr uint32_t HANDOFF_PPS_RING_SIZE = 4U;

// Foreground black-box recorder.  Two milliseconds is deliberately above
// the observed normal callback/idle scale while remaining tiny beside the
// suspected 50 ms event.  CH2 event age retains a tighter 100 us trigger.
static constexpr uint32_t INTERRUPT_FORENSIC_RING_SIZE = 16U;
static constexpr uint32_t INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES =
    (uint32_t)DWT_EXPECTED_PER_PPS / 500U;
static constexpr uint32_t INTERRUPT_FORENSIC_GAP_THRESHOLD_CYCLES =
    (uint32_t)DWT_EXPECTED_PER_PPS / 500U;
static constexpr uint32_t INTERRUPT_FORENSIC_GAP_THRESHOLD_TICKS = 20000U;
static constexpr uint32_t INTERRUPT_FORENSIC_CH2_AGE_THRESHOLD_TICKS = 1000U;
static_assert(INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES != 0U,
              "foreground forensic service threshold must be nonzero");
static_assert(INTERRUPT_FORENSIC_GAP_THRESHOLD_CYCLES != 0U,
              "foreground forensic gap threshold must be nonzero");

static constexpr uint8_t QTIMER1_VCLOCK_CH = 0U;
static constexpr uint8_t QTIMER1_RETIRED_AUX_CH = 1U;
static constexpr uint8_t QTIMER1_TIMEPOP_CH = 2U;
static constexpr uint8_t QTIMER1_VCLOCK_PCS = 0U;
static constexpr uint8_t QTIMER1_TIMEPOP_PCS = 0U;
static constexpr uint8_t QTIMER2_OCXO1_CH = 0U;
static constexpr uint8_t QTIMER2_OCXO1_PCS = 0U;
static constexpr uint8_t QTIMER3_OCXO2_CH = 3U;
static constexpr uint8_t QTIMER3_OCXO2_PCS = 3U;

static constexpr uint32_t STIMULUS_LAUNCH_LATENCY_CYCLES = 5U;
static constexpr uint32_t VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS = 1U;
static constexpr int32_t VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS = 0;
static constexpr uint32_t OCXO_ONE_SECOND_TICKS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static constexpr uint32_t OCXO_COMPARE_RATE_HZ = 1U;
static constexpr uint32_t OCXO_COMPARE_PERIOD_TICKS = OCXO_ONE_SECOND_TICKS;
static constexpr uint32_t OCXO_ARM_WINDOW_TICKS = 49152U;
static constexpr uint32_t OCXO_MIN_ARM_LEAD_TICKS = 64U;
static constexpr uint32_t COUNTER32_LINEAGE_LOCK_STREAK = 8U;
static constexpr uint32_t QTIMER_DWT_1S_LOCK_STREAK = 8U;
static constexpr uint32_t QTIMER_DWT_MATCH_GATE_CYCLES = 24U;

// SmartZero is a repeatability vote, not a reference-frequency estimator.
// Ten recent lawful one-second intervals are retained.  A lane locks when any
// three occupy a total four-cycle band; the newest member of that band is the
// accepted current cycle count.
static constexpr uint32_t SMARTZERO_HISTORY_SIZE = 10U;
static constexpr uint32_t SMARTZERO_QUORUM_REQUIRED = 3U;
static constexpr uint32_t SMARTZERO_VOTE_SPAN_CYCLES = 4U;

static_assert(OCXO_ONE_SECOND_TICKS == 10000000U,
              "OCXO one-second target must be 10,000,000 ticks");
static_assert(OCXO_COMPARE_RATE_HZ == 1U,
              "OCXO compare service must run exactly once per second");
static_assert(OCXO_COMPARE_PERIOD_TICKS == 10000000U,
              "OCXO compare period must be 10,000,000 ticks");
static_assert(OCXO_MIN_ARM_LEAD_TICKS < OCXO_ARM_WINDOW_TICKS,
              "OCXO arm lead must be inside arm window");
static_assert(OCXO_ARM_WINDOW_TICKS < 65536U,
              "OCXO arm window must stay inside one low-word revolution");
static_assert(SMARTZERO_QUORUM_REQUIRED <= SMARTZERO_HISTORY_SIZE,
              "SmartZero quorum must fit inside its history");

static constexpr bool OCXO1_DISABLED = false;
static constexpr bool OCXO2_DISABLED = false;

static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;
volatile bool g_process_interrupt_foreground_pending = false;

struct interrupt_isr_runtime_diag_t {
  volatile bool active = false;
  volatile uint32_t entry_count = 0U;
  volatile uint32_t exit_count = 0U;
  volatile uint32_t last_entry_dwt = 0U;
  volatile uint32_t last_wall_cycles = 0U;
  volatile uint32_t min_wall_cycles = 0U;
  volatile uint32_t max_wall_cycles = 0U;
  volatile uint32_t last_entry_basepri = 0U;
  volatile uint32_t last_entry_primask = 0U;
  volatile uint32_t last_entry_ipsr = 0U;
  volatile bool preempted_during_current_entry = false;
  volatile uint32_t entry_with_basepri_nonzero_count = 0U;
  volatile uint32_t entry_with_primask_nonzero_count = 0U;
  volatile uint32_t preempted_by_higher_tier_count = 0U;
};

struct interrupt_priority_runtime_t {
  interrupt_isr_runtime_diag_t qtimer1{};
  interrupt_isr_runtime_diag_t qtimer2{};
  interrupt_isr_runtime_diag_t qtimer3{};
  interrupt_isr_runtime_diag_t pps{};
  interrupt_isr_runtime_diag_t continuation{};

  volatile uint32_t qtimer1_preempted_continuation_count = 0U;
  volatile uint32_t ocxo1_preempted_qtimer1_count = 0U;
  volatile uint32_t ocxo2_preempted_qtimer1_count = 0U;
  volatile uint32_t pps_preempted_qtimer1_count = 0U;
  volatile uint32_t science_preempted_continuation_count = 0U;

  volatile uint32_t verify_count = 0U;
  volatile uint32_t verify_mismatch_count = 0U;
  volatile bool last_verify_all_match = false;
};

static interrupt_priority_runtime_t g_interrupt_priority_runtime{};

// ============================================================================
// CPU / ordering helpers
// ============================================================================

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static inline void interrupt_handoff_barrier(void) {
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");
}

static inline uint32_t interrupt_ipsr(void) {
  uint32_t value = 0;
  __asm__ volatile ("mrs %0, ipsr" : "=r" (value) :: "memory");
  return value;
}

static inline uint32_t interrupt_basepri(void) {
  uint32_t value = 0U;
  __asm__ volatile ("mrs %0, basepri" : "=r" (value) :: "memory");
  return value;
}

static inline uint32_t interrupt_primask(void) {
  uint32_t value = 0U;
  __asm__ volatile ("mrs %0, primask" : "=r" (value) :: "memory");
  return value;
}

static inline void interrupt_isr_diag_enter(
    interrupt_isr_runtime_diag_t& diag,
    uint32_t entry_dwt) {
  diag.preempted_during_current_entry = false;
  diag.active = true;
  dmb_barrier();
  diag.entry_count++;
  diag.last_entry_dwt = entry_dwt;
  diag.last_entry_basepri = interrupt_basepri();
  diag.last_entry_primask = interrupt_primask();
  diag.last_entry_ipsr = interrupt_ipsr();
  if (diag.last_entry_basepri != 0U) {
    diag.entry_with_basepri_nonzero_count++;
  }
  if (diag.last_entry_primask != 0U) {
    diag.entry_with_primask_nonzero_count++;
  }
}

static inline void interrupt_isr_diag_exit(
    interrupt_isr_runtime_diag_t& diag,
    uint32_t entry_dwt) {
  const uint32_t wall_cycles = ARM_DWT_CYCCNT - entry_dwt;
  diag.last_wall_cycles = wall_cycles;
  if (diag.exit_count == 0U || wall_cycles < diag.min_wall_cycles) {
    diag.min_wall_cycles = wall_cycles;
  }
  if (wall_cycles > diag.max_wall_cycles) {
    diag.max_wall_cycles = wall_cycles;
  }
  diag.exit_count++;
  dmb_barrier();
  diag.active = false;
}

static inline void interrupt_note_science_preemption(
    volatile uint32_t& qtimer1_counter) {
  if (g_interrupt_priority_runtime.qtimer1.active) {
    g_interrupt_priority_runtime.qtimer1.preempted_during_current_entry = true;
    qtimer1_counter++;
    g_interrupt_priority_runtime.qtimer1.preempted_by_higher_tier_count++;
  }
  if (g_interrupt_priority_runtime.continuation.active) {
    g_interrupt_priority_runtime.science_preempted_continuation_count++;
    g_interrupt_priority_runtime.continuation.preempted_by_higher_tier_count++;
  }
}

static inline uint32_t interrupt_priority0_guard_enter(void) {
  uint32_t prior_basepri = 0;
  __asm__ volatile ("mrs %0, basepri" : "=r" (prior_basepri) :: "memory");
  if (prior_basepri == 0U ||
      prior_basepri > INTERRUPT_PRIORITY0_PRESERVING_BASEPRI) {
    __asm__ volatile ("msr basepri, %0"
                      :: "r" (INTERRUPT_PRIORITY0_PRESERVING_BASEPRI)
                      : "memory");
  }
  dmb_barrier();
  return prior_basepri;
}

static inline void interrupt_priority0_guard_exit(uint32_t prior_basepri) {
  dmb_barrier();
  __asm__ volatile ("msr basepri, %0" :: "r" (prior_basepri) : "memory");
}

static bool interrupt_callback_address_executable(uintptr_t bits) {
  if (bits == 0U) return false;
  const uintptr_t address = bits & ~uintptr_t(1U);
  return address < 0x00100000UL ||
         (address >= 0x60000000UL && address < 0x61000000UL);
}

static char interrupt_ascii_fold(char c) {
  return (c >= 'a' && c <= 'z') ? (char)(c - ('a' - 'A')) : c;
}

static bool interrupt_cstr_equal_ci(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    if (interrupt_ascii_fold(*a) != interrupt_ascii_fold(*b)) return false;
    ++a;
    ++b;
  }
  return *a == *b;
}

static uint32_t interrupt_abs_i32(int32_t value) {
  return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static inline uint32_t qtimer_event_dwt_from_isr_entry_raw(
    uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw -
      (QTIMER_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

static inline uint32_t pps_dwt_from_isr_entry_raw(
    uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw -
      (GPIO_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

// ============================================================================
// Snapshot stores
// ============================================================================

struct snapshot_store_t {
  volatile uint32_t seq = 0;
  volatile uint32_t pps_sequence = 0;
  volatile uint32_t pps_dwt_at_edge = 0;
  volatile uint32_t pps_counter32_at_edge = 0;
  volatile uint16_t pps_ch3_at_edge = 0;
  volatile uint32_t pvc_sequence = 0;
  volatile uint32_t pvc_dwt_at_edge = 0;
  volatile uint32_t pvc_counter32_at_edge = 0;
  volatile uint16_t pvc_ch3_at_edge = 0;
};

static snapshot_store_t g_store{};

static void store_publish(const pps_t& pps, const pps_vclock_t& pvc) {
  g_store.seq++;
  dmb_barrier();
  g_store.pps_sequence = pps.sequence;
  g_store.pps_dwt_at_edge = pps.dwt_at_edge;
  g_store.pps_counter32_at_edge = pps.counter32_at_edge;
  g_store.pps_ch3_at_edge = pps.ch3_at_edge;
  g_store.pvc_sequence = pvc.sequence;
  g_store.pvc_dwt_at_edge = pvc.dwt_at_edge;
  g_store.pvc_counter32_at_edge = pvc.counter32_at_edge;
  g_store.pvc_ch3_at_edge = pvc.ch3_at_edge;
  dmb_barrier();
  g_store.seq++;
}

static bool store_load(pps_t& pps, pps_vclock_t& pvc) {
  for (uint32_t attempt = 0; attempt < 4U; ++attempt) {
    const uint32_t seq1 = g_store.seq;
    if (seq1 & 1U) continue;
    dmb_barrier();
    pps.sequence = g_store.pps_sequence;
    pps.dwt_at_edge = g_store.pps_dwt_at_edge;
    pps.counter32_at_edge = g_store.pps_counter32_at_edge;
    pps.ch3_at_edge = g_store.pps_ch3_at_edge;
    pvc.sequence = g_store.pvc_sequence;
    pvc.dwt_at_edge = g_store.pvc_dwt_at_edge;
    pvc.counter32_at_edge = g_store.pvc_counter32_at_edge;
    pvc.ch3_at_edge = g_store.pvc_ch3_at_edge;
    pvc.gnss_ns_at_edge = -1;
    dmb_barrier();
    const uint32_t seq2 = g_store.seq;
    if (seq1 == seq2 && (seq2 & 1U) == 0U) return true;
  }
  pps = pps_t{};
  pvc = pps_vclock_t{};
  return false;
}

static pps_vclock_t store_load_pvc(void) {
  pps_t pps{};
  pps_vclock_t pvc{};
  (void)store_load(pps, pvc);
  return pvc;
}

struct epoch_capture_store_t {
  volatile uint32_t seq = 0;
  interrupt_epoch_capture_t value{};
};

static epoch_capture_store_t g_epoch_capture{};

static void epoch_capture_publish(const interrupt_epoch_capture_t& value) {
  g_epoch_capture.seq++;
  dmb_barrier();
  g_epoch_capture.value = value;
  dmb_barrier();
  g_epoch_capture.seq++;
}

bool interrupt_last_epoch_capture(interrupt_epoch_capture_t* out) {
  if (!out) return false;
  for (uint32_t attempt = 0; attempt < 4U; ++attempt) {
    const uint32_t seq1 = g_epoch_capture.seq;
    if (seq1 & 1U) continue;
    dmb_barrier();
    *out = g_epoch_capture.value;
    dmb_barrier();
    const uint32_t seq2 = g_epoch_capture.seq;
    if (seq1 == seq2 && (seq2 & 1U) == 0U) return out->valid;
  }
  *out = interrupt_epoch_capture_t{};
  return false;
}

// ============================================================================
// Physical PPS / selected VCLOCK state
// ============================================================================

struct pps_gpio_heartbeat_t {
  uint32_t edge_count = 0;
  uint32_t last_dwt = 0;
  int64_t last_gnss_ns = -1;
};

static pps_gpio_heartbeat_t g_pps_gpio_heartbeat{};
static uint32_t g_gpio_irq_count = 0;
static uint32_t g_gpio_miss_count = 0;
static pps_t g_last_pps_witness{};
static bool g_last_pps_witness_valid = false;
static volatile bool g_pps_rebootstrap_pending = false;
static uint32_t g_pps_rebootstrap_count = 0;
static pps_vclock_edge_authority_t g_pps_vclock_edge_authority{};
static pps_edge_dispatch_fn g_pps_edge_dispatch = nullptr;
static volatile interrupt_pps_entry_latency_handler_fn
    g_pps_entry_latency_handler = nullptr;

// CLOCKS may continue labeling observed anchors.  process_interrupt no longer
// projects lower-priority events into GNSS; it retains only the latest label as
// report evidence and a cycles-per-second fallback.
struct observed_anchor_t {
  uint32_t sequence = 0;
  uint32_t counter32 = 0;
  uint64_t gnss_ns = 0;
  uint32_t cps = 0;
  bool valid = false;
  uint32_t update_count = 0;
};
static observed_anchor_t g_observed_anchor{};

void interrupt_pps_vclock_label_anchor(uint32_t sequence,
                                       uint32_t counter32_at_edge,
                                       uint64_t gnss_ns_at_edge,
                                       uint32_t dwt_cycles_per_second) {
  if (sequence == 0U || dwt_cycles_per_second == 0U) return;
  const uint32_t prior = interrupt_priority0_guard_enter();
  g_observed_anchor.sequence = sequence;
  g_observed_anchor.counter32 = counter32_at_edge;
  g_observed_anchor.gnss_ns = gnss_ns_at_edge;
  g_observed_anchor.cps = dwt_cycles_per_second;
  g_observed_anchor.valid = true;
  g_observed_anchor.update_count++;
  interrupt_priority0_guard_exit(prior);
}

static uint32_t interrupt_vclock_cycles_per_second(void) {
  const uint32_t calibrated = clocks_dwt_cycles_per_gnss_second();
  if (calibrated != 0U) return calibrated;
  if (g_observed_anchor.valid && g_observed_anchor.cps != 0U) {
    return g_observed_anchor.cps;
  }
  return (uint32_t)DWT_EXPECTED_PER_PPS;
}

uint32_t interrupt_dynamic_cps(void) {
  return interrupt_vclock_cycles_per_second();
}

// ============================================================================
// Synthetic 32-bit clock identities
// ============================================================================

struct synthetic_clock32_t {
  bool zeroed = false;
  uint64_t zero_ns = 0;
  uint32_t zero_counter32 = 0;
  uint32_t current_counter32 = 0;
  uint64_t current_ns = 0;
  uint16_t hardware16 = 0;
  uint32_t zero_count = 0;
  uint32_t minder_update_count = 0;
  bool pending_zero = false;
  uint64_t pending_zero_ns = 0;
  uint32_t pending_zero_counter32 = 0;
  uint32_t pending_zero_count = 0;
};

struct vclock_synthetic_clock32_t : synthetic_clock32_t {
  uint16_t hardware_low16_at_zero = 0;
  uint16_t hardware_low16_at_current = 0;
  bool hardware_anchor_valid = false;
  uint32_t hardware_anchor_update_count = 0;
};

static vclock_synthetic_clock32_t g_vclock_clock32{};
static synthetic_clock32_t g_ocxo1_clock32{};
static synthetic_clock32_t g_ocxo2_clock32{};

static inline uint32_t clock32_from_ns(uint64_t ns) {
  return (uint32_t)((ns / 100ULL) & 0xFFFFFFFFULL);
}

uint32_t interrupt_clock32_from_ns(uint64_t ns) {
  return clock32_from_ns(ns);
}

static void synthetic_clock_birth(synthetic_clock32_t& clock,
                                  uint16_t hardware16) {
  clock.zeroed = true;
  clock.zero_ns = (uint64_t)hardware16 * 100ULL;
  clock.zero_counter32 = (uint32_t)hardware16;
  clock.current_counter32 = (uint32_t)hardware16;
  clock.current_ns = (uint64_t)hardware16 * 100ULL;
  clock.hardware16 = hardware16;
  clock.pending_zero = false;
}

static void synthetic_clock_zero(synthetic_clock32_t& clock, uint64_t ns) {
  clock.zeroed = true;
  clock.zero_ns = ns;
  clock.zero_counter32 = clock32_from_ns(ns);
  clock.current_counter32 = clock.zero_counter32;
  clock.current_ns = ns;
  clock.hardware16 = (uint16_t)clock.current_counter32;
  clock.zero_count++;
  clock.pending_zero = false;
}

static void vclock_anchor_hardware(uint32_t counter32, uint16_t hardware16) {
  g_vclock_clock32.current_counter32 = counter32;
  g_vclock_clock32.current_ns = (uint64_t)counter32 * 100ULL;
  g_vclock_clock32.hardware16 = hardware16;
  g_vclock_clock32.hardware_low16_at_current = hardware16;
  g_vclock_clock32.hardware_anchor_valid = true;
  g_vclock_clock32.hardware_anchor_update_count++;
}

static uint32_t vclock_synthetic_from_hardware_low16(uint16_t hardware16) {
  if (!g_vclock_clock32.zeroed) return (uint32_t)hardware16;
  if (g_vclock_clock32.hardware_anchor_valid) {
    return g_vclock_clock32.current_counter32 +
        (uint32_t)((uint16_t)(hardware16 -
                             g_vclock_clock32.hardware_low16_at_current));
  }
  return g_vclock_clock32.zero_counter32 +
      (uint32_t)((uint16_t)(hardware16 -
                           g_vclock_clock32.hardware_low16_at_zero));
}

static uint16_t vclock_hardware_low16_from_synthetic(uint32_t counter32) {
  if (!g_vclock_clock32.zeroed) return (uint16_t)counter32;
  return (uint16_t)(g_vclock_clock32.hardware_low16_at_zero +
                    (uint16_t)(counter32 -
                               g_vclock_clock32.zero_counter32));
}

static void vclock_tend_from_hardware(uint16_t hardware16) {
  if (!g_vclock_clock32.zeroed) {
    synthetic_clock_birth(g_vclock_clock32, hardware16);
    g_vclock_clock32.hardware_low16_at_zero = hardware16;
    vclock_anchor_hardware((uint32_t)hardware16, hardware16);
    return;
  }
  const uint32_t counter32 =
      vclock_synthetic_from_hardware_low16(hardware16);
  vclock_anchor_hardware(counter32, hardware16);
  g_vclock_clock32.minder_update_count++;
}

static void synthetic_clock_tend_from_hardware(synthetic_clock32_t& clock,
                                                uint16_t hardware16) {
  if (!clock.zeroed) {
    synthetic_clock_birth(clock, hardware16);
    return;
  }
  const uint32_t delta = (uint32_t)((uint16_t)(hardware16 - clock.hardware16));
  clock.current_counter32 += delta;
  clock.current_ns += (uint64_t)delta * 100ULL;
  clock.hardware16 = hardware16;
  clock.minder_update_count++;
}

// ============================================================================
// Subscriber runtime and foreground custody
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
};

struct interrupt_deferred_dispatch_t {
  bool pending = false;
  uint32_t binding_generation = 0;
  interrupt_subscriber_event_fn callback = nullptr;
  void* user_data = nullptr;
  interrupt_event_t event{};
  bool pps_continuation_valid = false;
  pps_edge_snapshot_t pps_continuation{};
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub{};
  bool subscribed = false;
  bool active = false;
  interrupt_event_t last_event{};
  interrupt_capture_diag_t last_diag{};

  // Previous endpoint verdict retained solely to render the signed delay
  // contamination of the next one-second interval.
  bool previous_delay_valid = false;
  bool previous_endpoint_delayed = false;
  interrupt_delay_cause_t previous_delayed_by =
      interrupt_delay_cause_t::NONE;
  uint32_t previous_delay_cycles = 0U;
  bool previous_interval_delay_valid = false;
  int32_t previous_interval_delay_cycles = 0;
  interrupt_delay_cause_t previous_interval_delayed_by =
      interrupt_delay_cause_t::NONE;

  bool has_fired = false;
  uint32_t binding_generation = 1U;
  interrupt_deferred_dispatch_t deferred{};
  bool dispatch_running = false;
  uint32_t dispatch_busy_drop_count = 0;
  uint32_t dispatch_stale_drop_count = 0;
  uint32_t dispatch_invalid_callback_count = 0;
  uint32_t dispatch_arm_fail_count = 0;
  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;
};

static constexpr uint32_t INTERRUPT_SUBSCRIBER_INDEX_VCLOCK = 0U;
static constexpr uint32_t INTERRUPT_SUBSCRIBER_INDEX_OCXO1 = 1U;
static constexpr uint32_t INTERRUPT_SUBSCRIBER_INDEX_OCXO2 = 2U;

static constexpr interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK",
    interrupt_provider_kind_t::QTIMER1,
    interrupt_lane_t::QTIMER1_CH0_COMP },
  { interrupt_subscriber_kind_t::OCXO1, "OCXO1",
    interrupt_provider_kind_t::QTIMER2,
    interrupt_lane_t::QTIMER2_CH0_COMP },
  { interrupt_subscriber_kind_t::OCXO2, "OCXO2",
    interrupt_provider_kind_t::QTIMER3,
    interrupt_lane_t::QTIMER3_CH3_COMP },
};

static constexpr uint32_t INTERRUPT_SUBSCRIBER_COUNT =
    sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0]);

static_assert(INTERRUPT_SUBSCRIBER_COUNT == 3U,
              "subscriber descriptor indexes require exactly three lanes");
static_assert(DESCRIPTORS[INTERRUPT_SUBSCRIBER_INDEX_VCLOCK].kind ==
                  interrupt_subscriber_kind_t::VCLOCK,
              "VCLOCK descriptor index changed");
static_assert(DESCRIPTORS[INTERRUPT_SUBSCRIBER_INDEX_OCXO1].kind ==
                  interrupt_subscriber_kind_t::OCXO1,
              "OCXO1 descriptor index changed");
static_assert(DESCRIPTORS[INTERRUPT_SUBSCRIBER_INDEX_OCXO2].kind ==
                  interrupt_subscriber_kind_t::OCXO2,
              "OCXO2 descriptor index changed");

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS]{};
static_assert(INTERRUPT_SUBSCRIBER_INDEX_OCXO2 < MAX_INTERRUPT_SUBSCRIBERS,
              "subscriber runtime array cannot hold the static OCXO bindings");
static uint32_t g_subscriber_count = 0;

static interrupt_subscriber_runtime_t* runtime_at_exact(
    uint32_t index,
    interrupt_subscriber_kind_t kind) {
  if (g_subscriber_count != INTERRUPT_SUBSCRIBER_COUNT ||
      index >= INTERRUPT_SUBSCRIBER_COUNT) {
    return nullptr;
  }
  interrupt_subscriber_runtime_t& rt = g_subscribers[index];
  return rt.desc == &DESCRIPTORS[index] && rt.desc->kind == kind
      ? &rt
      : nullptr;
}

static interrupt_subscriber_runtime_t* runtime_for(
    interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:
      return runtime_at_exact(INTERRUPT_SUBSCRIBER_INDEX_VCLOCK, kind);
    case interrupt_subscriber_kind_t::OCXO1:
      return runtime_at_exact(INTERRUPT_SUBSCRIBER_INDEX_OCXO1, kind);
    case interrupt_subscriber_kind_t::OCXO2:
      return runtime_at_exact(INTERRUPT_SUBSCRIBER_INDEX_OCXO2, kind);
    default:
      return nullptr;
  }
}

static bool interrupt_subscriber_callback_executable(
    interrupt_subscriber_event_fn callback) {
  return interrupt_callback_address_executable((uintptr_t)callback);
}

static timepop_dispatch_trace_kind_t interrupt_execution_trace_subscriber_kind(
    interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:
      return timepop_dispatch_trace_kind_t::SUBSCRIBER_VCLOCK;
    case interrupt_subscriber_kind_t::OCXO1:
      return timepop_dispatch_trace_kind_t::SUBSCRIBER_OCXO1;
    case interrupt_subscriber_kind_t::OCXO2:
      return timepop_dispatch_trace_kind_t::SUBSCRIBER_OCXO2;
    default:
      return timepop_dispatch_trace_kind_t::NONE;
  }
}

static void interrupt_dispatch_invalidate_locked(
    interrupt_subscriber_runtime_t& rt) {
  rt.binding_generation++;
  if (rt.binding_generation == 0U) rt.binding_generation = 1U;
  if (!rt.dispatch_running) rt.deferred = interrupt_deferred_dispatch_t{};
}

static bool interrupt_dispatch_begin(interrupt_subscriber_runtime_t& rt,
                                     const interrupt_event_t& event) {
  interrupt_subscriber_event_fn callback = nullptr;
  void* callback_user_data = nullptr;
  uint32_t binding_generation = 0;

  const uint32_t prior_basepri = interrupt_priority0_guard_enter();
  if (!rt.subscribed || !rt.active || !rt.sub.on_event ||
      rt.dispatch_running || rt.deferred.pending) {
    if (rt.dispatch_running || rt.deferred.pending) {
      rt.dispatch_busy_drop_count++;
    }
    interrupt_priority0_guard_exit(prior_basepri);
    return false;
  }

  callback = rt.sub.on_event;
  callback_user_data = rt.sub.user_data;
  binding_generation = rt.binding_generation;
  if (!interrupt_subscriber_callback_executable(callback)) {
    rt.dispatch_invalid_callback_count++;
    interrupt_priority0_guard_exit(prior_basepri);
    return false;
  }

  // Priority 32 authors only an immutable foreground fact.  It does not call
  // TimePop, mutate a TimePop mailbox, or execute application behavior.
  rt.deferred.pending = true;
  rt.deferred.binding_generation = binding_generation;
  rt.deferred.callback = callback;
  rt.deferred.user_data = callback_user_data;
  rt.deferred.event = event;
  g_process_interrupt_foreground_pending = true;

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    rt.deferred.pps_continuation_valid = true;
    rt.deferred.pps_continuation = interrupt_last_pps_edge();
  }
  interrupt_priority0_guard_exit(prior_basepri);

  const timepop_dispatch_trace_kind_t trace_kind =
      interrupt_execution_trace_subscriber_kind(rt.desc->kind);
  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::SUBSCRIBER_SELECTED,
      trace_kind,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback,
      (uint32_t)(uintptr_t)rt.sub.on_event,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)rt.desc->name,
      1U | (binding_generation << 16));
  return true;
}

static bool interrupt_dispatch_foreground_one(
    interrupt_subscriber_runtime_t& rt) {
  interrupt_subscriber_event_fn callback = nullptr;
  void* callback_user_data = nullptr;
  interrupt_event_t event{};
  bool pps_continuation_valid = false;
  pps_edge_snapshot_t pps_continuation{};

  const uint32_t prior_basepri = interrupt_priority0_guard_enter();
  if (!rt.deferred.pending || rt.dispatch_running) {
    interrupt_priority0_guard_exit(prior_basepri);
    return false;
  }
  if (!rt.subscribed || !rt.active ||
      rt.deferred.binding_generation != rt.binding_generation) {
    rt.dispatch_stale_drop_count++;
    rt.deferred = interrupt_deferred_dispatch_t{};
    interrupt_priority0_guard_exit(prior_basepri);
    return false;
  }

  callback = rt.deferred.callback;
  callback_user_data = rt.deferred.user_data;
  if (!interrupt_subscriber_callback_executable(callback)) {
    rt.dispatch_invalid_callback_count++;
    rt.deferred = interrupt_deferred_dispatch_t{};
    interrupt_priority0_guard_exit(prior_basepri);
    return false;
  }

  event = rt.deferred.event;
  pps_continuation_valid = rt.deferred.pps_continuation_valid;
  pps_continuation = rt.deferred.pps_continuation;
  rt.dispatch_running = true;
  interrupt_priority0_guard_exit(prior_basepri);

  const timepop_dispatch_trace_kind_t trace_kind =
      interrupt_execution_trace_subscriber_kind(rt.desc->kind);
  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::SUBSCRIBER_ENTER,
      trace_kind,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback,
      0U,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)rt.desc->name,
      event.counter32_at_event);
  rt.dispatch_count++;
  crash_dispatch_breadcrumb_note(
      CRASH_DISPATCH_BREADCRUMB_SUBSCRIBER_ENTER,
      (uint32_t)(uintptr_t)callback,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)&rt);
  callback(event, &rt.last_diag, callback_user_data);
  crash_dispatch_breadcrumb_note(
      CRASH_DISPATCH_BREADCRUMB_SUBSCRIBER_RETURN,
      (uint32_t)(uintptr_t)callback,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)&rt);
  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::SUBSCRIBER_RETURN,
      trace_kind,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback,
      0U,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)rt.desc->name,
      event.counter32_at_event);

  crash_dispatch_breadcrumb_note(
      CRASH_DISPATCH_BREADCRUMB_CLEANUP_ENTER,
      (uint32_t)(uintptr_t)callback,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)&rt);
  const uint32_t finish_basepri = interrupt_priority0_guard_enter();
  rt.dispatch_running = false;
  rt.deferred = interrupt_deferred_dispatch_t{};
  interrupt_priority0_guard_exit(finish_basepri);
  crash_dispatch_breadcrumb_note(
      CRASH_DISPATCH_BREADCRUMB_CLEANUP_COMPLETE,
      (uint32_t)(uintptr_t)callback,
      (uint32_t)rt.desc->kind,
      event.counter32_at_event,
      (uint32_t)(uintptr_t)callback_user_data,
      (uint32_t)(uintptr_t)&rt);

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK &&
      pps_continuation_valid) {
    const pps_edge_dispatch_fn callback_pps = g_pps_edge_dispatch;
    if (interrupt_callback_address_executable((uintptr_t)callback_pps)) {
      callback_pps(pps_continuation);
    }
  }
  return true;
}

static void interrupt_dispatch_foreground_service(void) {
  for (uint32_t i = 0U; i < INTERRUPT_SUBSCRIBER_COUNT; ++i) {
    (void)interrupt_dispatch_foreground_one(g_subscribers[i]);
  }
}

// ============================================================================
// Lane state
// ============================================================================

struct vclock_lane_t {
  bool initialized = false;
  bool active = false;
  bool phase_bootstrapped = false;
  uint32_t next_one_second_counter32 = 0;
  bool one_second_grid_valid = false;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t heartbeat_count = 0;
  uint32_t one_second_count = 0;
  uint32_t last_target_counter32 = 0;
  uint32_t last_dwt_at_edge = 0;
  uint32_t last_isr_entry_dwt_raw = 0;
};

struct ocxo_lane_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t channel = 0;
  uint8_t pcs = 0;
  int input_pin = -1;

  bool initialized = false;
  bool active = false;
  bool target_grid_valid = false;
  volatile bool compare_armed = false;

  // Priority 0 defuses an authored compare before Priority 32 can commit it.
  // While capture_pending is true, VCLOCK must not tend this synthetic clock or
  // interpret the disabled compare as a missed target.
  volatile bool capture_pending = false;
  uint32_t capture_pending_target_counter32 = 0;

  uint32_t grid_epoch_counter32 = 0;
  uint32_t next_target_counter32 = 0;

  // One authored target equals one physical OCXO second.  VCLOCK extends the
  // low-word lineage between compares, so no intermediate OCXO cadence teeth
  // are needed to preserve the 32-bit target identity.
  uint32_t target_grid_reset_count = 0U;

  uint32_t armed_target_counter32 = 0;
  uint16_t armed_target_low16 = 0;

  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t arm_count = 0;
  uint32_t fire_count = 0;
  uint32_t false_irq_count = 0;
  uint32_t missed_target_count = 0;
  uint32_t rollover_tend_count = 0;
  uint32_t arm_window_wait_count = 0;
  uint32_t arm_too_close_count = 0;
  uint32_t rebootstrap_count = 0;
  uint32_t recover_count = 0;
  uint32_t binding_identity_check_count = 0;
  uint32_t binding_identity_failure_count = 0;

  uint32_t capture_pending_set_count = 0;
  uint32_t capture_pending_clear_count = 0;
  uint32_t capture_pending_tend_skip_count = 0;
  uint32_t capture_pending_enqueue_fail_count = 0;
  uint32_t capture_pending_missing_count = 0;
  uint32_t capture_pending_target_mismatch_count = 0;
  uint32_t capture_pending_recovery_discard_count = 0;

  uint32_t target_commit_rollback_count = 0;
  uint32_t target_commit_ns_underflow_count = 0;
  int32_t last_target_commit_delta_ticks = 0;
  uint32_t last_target_commit_rollback_ticks = 0;

  uint32_t last_arm_dwt = 0;
  uint32_t last_arm_counter32 = 0;
  uint16_t last_arm_hardware16 = 0;
  uint32_t last_arm_remaining_ticks = 0;
  uint32_t last_isr_entry_dwt_raw = 0;
  uint32_t last_dwt_at_edge = 0;
  uint32_t last_event_counter32 = 0;
  uint16_t last_ambient_low16 = 0;
  uint16_t last_compare_low16 = 0;
  int32_t last_ambient_minus_target_ticks = 0;

  bool previous_event_valid = false;
  uint32_t previous_event_counter32 = 0;
  uint32_t last_counter_delta_ticks = 0;
  uint32_t counter_delta_violation_count = 0;
  uint32_t dispatch_sequence = 0;
};

struct ocxo_runtime_context_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane_id = interrupt_lane_t::NONE;
  const char* name = nullptr;
  ocxo_lane_t* lane = nullptr;
  synthetic_clock32_t* clock32 = nullptr;
};

static vclock_lane_t g_vclock_lane{};
static ocxo_lane_t g_ocxo1_lane{};
static ocxo_lane_t g_ocxo2_lane{};
// These bindings describe permanent hardware identity.  Keep the descriptors
// immutable so no runtime write can turn one lane into another object species.
static constexpr ocxo_runtime_context_t g_ocxo1_ctx{
  interrupt_subscriber_kind_t::OCXO1,
  interrupt_provider_kind_t::QTIMER2,
  interrupt_lane_t::QTIMER2_CH0_COMP,
  "OCXO1",
  &g_ocxo1_lane,
  &g_ocxo1_clock32,
};
static constexpr ocxo_runtime_context_t g_ocxo2_ctx{
  interrupt_subscriber_kind_t::OCXO2,
  interrupt_provider_kind_t::QTIMER3,
  interrupt_lane_t::QTIMER3_CH3_COMP,
  "OCXO2",
  &g_ocxo2_lane,
  &g_ocxo2_clock32,
};

static_assert(g_ocxo1_ctx.kind == interrupt_subscriber_kind_t::OCXO1 &&
                  g_ocxo1_ctx.provider == interrupt_provider_kind_t::QTIMER2 &&
                  g_ocxo1_ctx.lane_id == interrupt_lane_t::QTIMER2_CH0_COMP &&
                  g_ocxo1_ctx.lane == &g_ocxo1_lane &&
                  g_ocxo1_ctx.clock32 == &g_ocxo1_clock32,
              "OCXO1 immutable binding changed");
static_assert(g_ocxo2_ctx.kind == interrupt_subscriber_kind_t::OCXO2 &&
                  g_ocxo2_ctx.provider == interrupt_provider_kind_t::QTIMER3 &&
                  g_ocxo2_ctx.lane_id == interrupt_lane_t::QTIMER3_CH3_COMP &&
                  g_ocxo2_ctx.lane == &g_ocxo2_lane &&
                  g_ocxo2_ctx.clock32 == &g_ocxo2_clock32,
              "OCXO2 immutable binding changed");

static const ocxo_runtime_context_t* ocxo_context_for(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_ctx;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_ctx;
  return nullptr;
}

static bool ocxo_kind_disabled(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return OCXO1_DISABLED;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return OCXO2_DISABLED;
  return false;
}

static interrupt_subscriber_runtime_t* ocxo_runtime_for(
    const ocxo_runtime_context_t& ctx) {
  return runtime_for(ctx.kind);
}

static bool ocxo1_static_binding_exact(void) {
  return g_subscriber_count == INTERRUPT_SUBSCRIBER_COUNT &&
      runtime_for(interrupt_subscriber_kind_t::OCXO1) ==
          &g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO1] &&
      g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO1].desc ==
          &DESCRIPTORS[INTERRUPT_SUBSCRIBER_INDEX_OCXO1] &&
      g_ocxo1_ctx.kind == interrupt_subscriber_kind_t::OCXO1 &&
      g_ocxo1_ctx.provider == interrupt_provider_kind_t::QTIMER2 &&
      g_ocxo1_ctx.lane_id == interrupt_lane_t::QTIMER2_CH0_COMP &&
      g_ocxo1_ctx.lane == &g_ocxo1_lane &&
      g_ocxo1_ctx.clock32 == &g_ocxo1_clock32 &&
      g_ocxo1_lane.kind == interrupt_subscriber_kind_t::OCXO1 &&
      g_ocxo1_lane.module == &IMXRT_TMR2 &&
      g_ocxo1_lane.channel == QTIMER2_OCXO1_CH &&
      g_ocxo1_lane.pcs == QTIMER2_OCXO1_PCS;
}

static bool ocxo2_static_binding_exact(void) {
  return g_subscriber_count == INTERRUPT_SUBSCRIBER_COUNT &&
      runtime_for(interrupt_subscriber_kind_t::OCXO2) ==
          &g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO2] &&
      g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO2].desc ==
          &DESCRIPTORS[INTERRUPT_SUBSCRIBER_INDEX_OCXO2] &&
      g_ocxo2_ctx.kind == interrupt_subscriber_kind_t::OCXO2 &&
      g_ocxo2_ctx.provider == interrupt_provider_kind_t::QTIMER3 &&
      g_ocxo2_ctx.lane_id == interrupt_lane_t::QTIMER3_CH3_COMP &&
      g_ocxo2_ctx.lane == &g_ocxo2_lane &&
      g_ocxo2_ctx.clock32 == &g_ocxo2_clock32 &&
      g_ocxo2_lane.kind == interrupt_subscriber_kind_t::OCXO2 &&
      g_ocxo2_lane.module == &IMXRT_TMR3 &&
      g_ocxo2_lane.channel == QTIMER3_OCXO2_CH &&
      g_ocxo2_lane.pcs == QTIMER3_OCXO2_PCS;
}

static bool ocxo_static_binding_exact(
    interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    return ocxo1_static_binding_exact();
  }
  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    return ocxo2_static_binding_exact();
  }
  return false;
}

static __attribute__((noinline, noclone))
bool ocxo1_binding_identity_check(void) {
  g_ocxo1_lane.binding_identity_check_count++;
  const bool exact = ocxo1_static_binding_exact();
  if (!exact) g_ocxo1_lane.binding_identity_failure_count++;
  return exact;
}

static __attribute__((noinline, noclone))
bool ocxo2_binding_identity_check(void) {
  g_ocxo2_lane.binding_identity_check_count++;
  const bool exact = ocxo2_static_binding_exact();
  if (!exact) g_ocxo2_lane.binding_identity_failure_count++;
  return exact;
}

static void interrupt_features_note_observed_edge(void);
static void interrupt_features_note_ocxo_custody(void);
static void interrupt_features_note_lineage(void);
static void interrupt_features_foreground_flush(void);
static bool interrupt_foreground_work_pending_locked(void);

// ============================================================================
// Passive integrity evidence
// ============================================================================

static interrupt_integrity_snapshot_t g_interrupt_integrity{};
static interrupt_integrity_snapshot_t g_interrupt_integrity_report_scratch
    DMAMEM{};

static interrupt_integrity_counter_check_t* integrity_counter_for(
    interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:
      return &g_interrupt_integrity.vclock_counter;
    case interrupt_subscriber_kind_t::OCXO1:
      return &g_interrupt_integrity.ocxo1_counter;
    case interrupt_subscriber_kind_t::OCXO2:
      return &g_interrupt_integrity.ocxo2_counter;
    default:
      return nullptr;
  }
}

static interrupt_integrity_qtimer_dwt_match_check_t* integrity_dwt_for(
    interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:
      return &g_interrupt_integrity.vclock_qtimer_dwt;
    case interrupt_subscriber_kind_t::OCXO1:
      return &g_interrupt_integrity.ocxo1_qtimer_dwt;
    case interrupt_subscriber_kind_t::OCXO2:
      return &g_interrupt_integrity.ocxo2_qtimer_dwt;
    default:
      return nullptr;
  }
}

void interrupt_integrity_note_counter32(interrupt_subscriber_kind_t kind,
                                        uint32_t sequence,
                                        bool interval_valid,
                                        uint32_t observed_delta_ticks,
                                        uint32_t expected_delta_ticks,
                                        uint32_t current_counter32) {
  interrupt_integrity_counter_check_t* state = integrity_counter_for(kind);
  if (!state) return;

  state->valid = true;
  state->sequence = sequence;
  state->expected_delta_ticks = expected_delta_ticks;
  state->observed_delta_ticks = interval_valid ? observed_delta_ticks : 0U;
  state->current_counter32 = current_counter32;
  state->previous_counter32 = interval_valid
      ? current_counter32 - observed_delta_ticks
      : 0U;
  state->lock_streak_required = COUNTER32_LINEAGE_LOCK_STREAK;

  if (!interval_valid || expected_delta_ticks == 0U) {
    state->skipped_count++;
    state->last_sample_counted = false;
    state->last_ok = false;
    state->observed_minus_expected_ticks = 0;
    return;
  }

  const int32_t error = observed_delta_ticks >= expected_delta_ticks
      ? (int32_t)(observed_delta_ticks - expected_delta_ticks)
      : -(int32_t)(expected_delta_ticks - observed_delta_ticks);
  state->observed_minus_expected_ticks = error;
  state->test_count++;
  state->last_sample_counted = true;
  const bool was_locked = state->locked;
  const bool ok = error == 0;
  state->last_ok = ok;

  if (ok) {
    state->ok_count++;
    if (state->consecutive_ok_count != UINT32_MAX) {
      state->consecutive_ok_count++;
    }
    if (was_locked) state->post_lock_ok_count++;
    else state->prelock_ok_count++;
  } else {
    state->bad_count++;
    state->consecutive_ok_count = 0;
    if (state->first_bad_sequence == 0U) {
      state->first_bad_sequence = sequence;
    }
    state->last_bad_sequence = sequence;
    state->last_bad_observed_delta_ticks = observed_delta_ticks;
    state->last_bad_expected_delta_ticks = expected_delta_ticks;
    state->last_bad_observed_minus_expected_ticks = error;
    if (was_locked) state->post_lock_bad_count++;
    else state->prelock_bad_count++;
  }

  if (!was_locked && ok &&
      state->consecutive_ok_count >= COUNTER32_LINEAGE_LOCK_STREAK) {
    state->locked = true;
    state->lock_sequence = sequence;
    state->lock_count++;
  }
}

void interrupt_integrity_note_vclock_pps_interval(
    uint32_t sequence,
    bool pps_interval_valid,
    uint32_t pps_interval_cycles,
    bool vclock_interval_valid,
    uint32_t vclock_interval_cycles) {
  interrupt_integrity_interval_check_t& state =
      g_interrupt_integrity.vclock_pps_interval;
  state.valid = true;
  state.sequence = sequence;
  state.gate_cycles = 10U;
  state.pps_interval_valid = pps_interval_valid;
  state.vclock_interval_valid = vclock_interval_valid;
  state.pps_interval_cycles = pps_interval_valid ? pps_interval_cycles : 0U;
  state.vclock_interval_cycles =
      vclock_interval_valid ? vclock_interval_cycles : 0U;
  if (!pps_interval_valid || !vclock_interval_valid ||
      pps_interval_cycles == 0U || vclock_interval_cycles == 0U) {
    state.skipped_count++;
    state.last_ok = false;
    state.vclock_minus_pps_cycles = 0;
    return;
  }
  const int32_t delta = vclock_interval_cycles >= pps_interval_cycles
      ? (int32_t)(vclock_interval_cycles - pps_interval_cycles)
      : -(int32_t)(pps_interval_cycles - vclock_interval_cycles);
  state.vclock_minus_pps_cycles = delta;
  state.test_count++;
  state.last_ok = interrupt_abs_i32(delta) <= state.gate_cycles;
  if (state.last_ok) state.ok_count++;
  else state.bad_count++;
}

static void integrity_note_dwt_interval(interrupt_subscriber_kind_t kind,
                                        uint32_t sequence,
                                        uint32_t target_counter32,
                                        uint32_t dwt_at_match) {
  interrupt_integrity_qtimer_dwt_match_check_t* match =
      integrity_dwt_for(kind);
  if (!match) return;
  match->valid = true;
  match->sequence = sequence;
  match->target_counter32 = target_counter32;
  match->dwt_at_match = dwt_at_match;
  match->one_second_boundary = true;

  // The historical 1 kHz member remains an ABI shell and is never authored.
  interrupt_integrity_qtimer_dwt_interval_check_t& state = match->one_second;
  state.valid = true;
  state.sequence = sequence;
  state.target_counter32 = target_counter32;
  state.dwt_at_match = dwt_at_match;
  state.gate_cycles = QTIMER_DWT_MATCH_GATE_CYCLES;
  state.lock_streak_required = QTIMER_DWT_1S_LOCK_STREAK;
  state.ruler_qualified = true;

  if (!state.previous_valid) {
    state.previous_valid = true;
    state.previous_sequence = sequence;
    state.previous_target_counter32 = target_counter32;
    state.previous_dwt_at_match = dwt_at_match;
    state.skipped_count++;
    state.last_ok = false;
    return;
  }

  const uint32_t prior_sequence = state.previous_sequence;
  const uint32_t prior_target = state.previous_target_counter32;
  const uint32_t prior_dwt = state.previous_dwt_at_match;
  const uint32_t target_delta = target_counter32 - prior_target;
  const uint32_t observed_cycles = dwt_at_match - prior_dwt;
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  const uint32_t expected_cycles = target_delta == OCXO_ONE_SECOND_TICKS
      ? cps
      : (uint32_t)(((uint64_t)cps * (uint64_t)target_delta +
                    (uint64_t)OCXO_ONE_SECOND_TICKS / 2ULL) /
                   (uint64_t)OCXO_ONE_SECOND_TICKS);

  state.previous_sequence = sequence;
  state.previous_target_counter32 = target_counter32;
  state.previous_dwt_at_match = dwt_at_match;
  state.interval_previous_sequence = prior_sequence;
  state.interval_previous_target_counter32 = prior_target;
  state.interval_previous_dwt_at_match = prior_dwt;
  state.target_delta_ticks = target_delta;
  state.observed_cycles = observed_cycles;
  state.expected_cycles = expected_cycles;

  if (target_delta == 0U || expected_cycles == 0U) {
    state.skipped_count++;
    state.last_ok = false;
    return;
  }

  const int64_t error64 = (int64_t)(uint64_t)observed_cycles -
                          (int64_t)(uint64_t)expected_cycles;
  const int32_t error = error64 > INT32_MAX
      ? INT32_MAX
      : (error64 < INT32_MIN ? INT32_MIN : (int32_t)error64);
  const uint32_t abs_error = interrupt_abs_i32(error);
  const bool was_locked = state.locked;
  const bool ok = abs_error <= state.gate_cycles;
  state.error_cycles = error;
  state.abs_error_cycles = abs_error;
  state.test_count++;
  state.last_ok = ok;

  if (ok) {
    state.match_count++;
    if (state.consecutive_ok_count != UINT32_MAX) {
      state.consecutive_ok_count++;
    }
    if (was_locked) state.post_lock_match_count++;
    else state.prelock_match_count++;
  } else {
    state.mismatch_count++;
    state.consecutive_ok_count = 0;
    if (error < -(int32_t)state.gate_cycles) state.too_short_count++;
    if (error > (int32_t)state.gate_cycles) state.too_long_count++;
    if (state.first_mismatch_sequence == 0U) {
      state.first_mismatch_sequence = sequence;
    }
    state.last_mismatch_sequence = sequence;
    state.last_mismatch_error_cycles = error;
    state.last_mismatch_observed_cycles = observed_cycles;
    state.last_mismatch_expected_cycles = expected_cycles;
    if (was_locked) state.post_lock_mismatch_count++;
    else state.prelock_mismatch_count++;
  }

  if (!was_locked && ok &&
      state.consecutive_ok_count >= QTIMER_DWT_1S_LOCK_STREAK) {
    state.locked = true;
    state.lock_sequence = sequence;
    state.lock_count++;
  }
}

bool interrupt_integrity_snapshot(interrupt_integrity_snapshot_t* out) {
  if (!out) return false;
  g_interrupt_integrity.snapshot_count++;
  g_interrupt_integrity.valid =
      g_interrupt_integrity.vclock_counter.valid ||
      g_interrupt_integrity.ocxo1_counter.valid ||
      g_interrupt_integrity.ocxo2_counter.valid ||
      g_interrupt_integrity.vclock_qtimer_dwt.valid ||
      g_interrupt_integrity.ocxo1_qtimer_dwt.valid ||
      g_interrupt_integrity.ocxo2_qtimer_dwt.valid ||
      g_interrupt_integrity.vclock_pps_interval.valid;
  g_interrupt_integrity_report_scratch = g_interrupt_integrity;
  *out = g_interrupt_integrity_report_scratch;
  return out->valid;
}

// ============================================================================
// One-second SmartZero proof
// ============================================================================

struct smartzero_interval_sample_t {
  uint32_t interval_cycles = 0;
  uint32_t previous_dwt = 0;
  uint32_t current_dwt = 0;
  uint32_t previous_counter32 = 0;
  uint32_t current_counter32 = 0;
  uint16_t current_hardware16 = 0;
  uint32_t ordinal = 0;
};

struct smartzero_vote_result_t {
  bool found = false;
  uint32_t count = 0;
  uint32_t min_cycles = 0;
  uint32_t max_cycles = 0;
  uint32_t span_cycles = 0;
  uint32_t closest_three_span_cycles = 0;
  smartzero_interval_sample_t selected{};
};

struct smartzero_lane_vote_report_t {
  uint32_t history_count = 0;
  uint32_t valid_interval_count = 0;
  uint32_t quorum_count = 0;
  uint32_t quorum_min_cycles = 0;
  uint32_t quorum_max_cycles = 0;
  uint32_t quorum_span_cycles = 0;
  uint32_t closest_three_span_cycles = 0;
  uint32_t accepted_interval_cycles = 0;
  uint32_t accepted_history_age = 0;
};

struct smartzero_lane_runtime_t {
  interrupt_smartzero_lane_snapshot_t pub{};
  bool previous_present = false;
  uint32_t previous_dwt = 0;
  uint32_t previous_counter32 = 0;
  uint16_t previous_hardware16 = 0;
  smartzero_interval_sample_t history[SMARTZERO_HISTORY_SIZE]{};
  uint32_t history_next = 0;
  uint32_t history_count = 0;
  uint32_t interval_ordinal = 0;
  uint32_t quorum_count = 0;
  uint32_t quorum_min_cycles = 0;
  uint32_t quorum_max_cycles = 0;
  uint32_t quorum_span_cycles = 0;
  uint32_t closest_three_span_cycles = 0;
  uint32_t accepted_interval_cycles = 0;
  uint32_t accepted_history_age = 0;
};

struct smartzero_runtime_t {
  volatile uint32_t seq = 0;
  interrupt_smartzero_phase_t phase = interrupt_smartzero_phase_t::IDLE;
  bool running = false;
  bool complete = false;
  uint32_t sequence = 0;
  uint32_t begin_count = 0;
  uint32_t complete_count = 0;
  uint32_t abort_count = 0;
  uint32_t current_lane_index = 0;
  smartzero_lane_runtime_t lanes[SMARTZERO_LANE_COUNT]{};
};

static smartzero_runtime_t g_smartzero{};

static interrupt_subscriber_kind_t smartzero_kind_for_index(uint32_t index) {
  switch (index) {
    case 0: return interrupt_subscriber_kind_t::VCLOCK;
    case 1: return interrupt_subscriber_kind_t::OCXO1;
    case 2: return interrupt_subscriber_kind_t::OCXO2;
    default: return interrupt_subscriber_kind_t::NONE;
  }
}

static int smartzero_index_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) return 0;
  if (kind == interrupt_subscriber_kind_t::OCXO1) return 1;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return 2;
  return -1;
}

static inline void smartzero_write_begin(void) {
  g_smartzero.seq++;
  dmb_barrier();
}

static inline void smartzero_write_end(void) {
  dmb_barrier();
  g_smartzero.seq++;
}

static void smartzero_reset_lane(uint32_t index) {
  smartzero_lane_runtime_t& runtime = g_smartzero.lanes[index];
  runtime = smartzero_lane_runtime_t{};
  runtime.pub.kind = smartzero_kind_for_index(index);
  runtime.pub.state = interrupt_smartzero_lane_state_t::IDLE;
  runtime.pub.last_decision = interrupt_smartzero_decision_t::NONE;
  runtime.pub.tolerance_cycles = SMARTZERO_VOTE_SPAN_CYCLES;
  runtime.pub.required_counter_delta_ticks = OCXO_ONE_SECOND_TICKS;
}

static bool smartzero_is_current_lane(interrupt_subscriber_kind_t kind) {
  for (uint32_t attempt = 0; attempt < 2U; ++attempt) {
    const uint32_t seq1 = g_smartzero.seq;
    if (seq1 & 1U) continue;
    dmb_barrier();
    const bool running = g_smartzero.running;
    const bool complete = g_smartzero.complete;
    const uint32_t lane_index = g_smartzero.current_lane_index;
    dmb_barrier();
    const uint32_t seq2 = g_smartzero.seq;
    if (seq1 == seq2 && (seq2 & 1U) == 0U) {
      return running && !complete &&
          smartzero_kind_for_index(lane_index) == kind;
    }
  }
  return false;
}

static uint32_t smartzero_history_slot(
    const smartzero_lane_runtime_t& runtime,
    uint32_t chronological_index) {
  const uint32_t oldest =
      (runtime.history_next + SMARTZERO_HISTORY_SIZE - runtime.history_count) %
      SMARTZERO_HISTORY_SIZE;
  return (oldest + chronological_index) % SMARTZERO_HISTORY_SIZE;
}

static void smartzero_history_push(
    smartzero_lane_runtime_t& runtime,
    const smartzero_interval_sample_t& sample) {
  runtime.history[runtime.history_next] = sample;
  runtime.history_next =
      (runtime.history_next + 1U) % SMARTZERO_HISTORY_SIZE;
  if (runtime.history_count < SMARTZERO_HISTORY_SIZE) {
    runtime.history_count++;
  }
}

static smartzero_vote_result_t smartzero_find_vote(
    const smartzero_lane_runtime_t& runtime) {
  smartzero_vote_result_t result{};
  const uint32_t count = runtime.history_count;
  if (count < SMARTZERO_QUORUM_REQUIRED) return result;

  const smartzero_interval_sample_t* ordered[SMARTZERO_HISTORY_SIZE]{};
  for (uint32_t i = 0; i < count; ++i) {
    ordered[i] =
        &runtime.history[smartzero_history_slot(runtime, i)];
  }

  // Sort a bounded pointer list by interval value.  No averaging or smoothing
  // occurs: this merely makes the closest-three and quorum spans explicit.
  for (uint32_t i = 1; i < count; ++i) {
    const smartzero_interval_sample_t* value = ordered[i];
    uint32_t j = i;
    while (j > 0U &&
           (ordered[j - 1U]->interval_cycles > value->interval_cycles ||
            (ordered[j - 1U]->interval_cycles == value->interval_cycles &&
             ordered[j - 1U]->ordinal > value->ordinal))) {
      ordered[j] = ordered[j - 1U];
      --j;
    }
    ordered[j] = value;
  }

  uint32_t closest_three = UINT32_MAX;
  for (uint32_t i = 0;
       i + SMARTZERO_QUORUM_REQUIRED <= count;
       ++i) {
    const uint32_t high =
        ordered[i + SMARTZERO_QUORUM_REQUIRED - 1U]->interval_cycles;
    const uint32_t span = high - ordered[i]->interval_cycles;
    if (span < closest_three) closest_three = span;
  }
  result.closest_three_span_cycles =
      closest_three == UINT32_MAX ? 0U : closest_three;

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t min_cycles = ordered[i]->interval_cycles;
    uint32_t end = i;
    while (end < count &&
           ordered[end]->interval_cycles - min_cycles <=
               SMARTZERO_VOTE_SPAN_CYCLES) {
      ++end;
    }
    const uint32_t candidate_count = end - i;
    if (candidate_count < SMARTZERO_QUORUM_REQUIRED) continue;

    const smartzero_interval_sample_t* newest = ordered[i];
    uint32_t max_cycles = min_cycles;
    for (uint32_t j = i; j < end; ++j) {
      if (ordered[j]->ordinal > newest->ordinal) newest = ordered[j];
      if (ordered[j]->interval_cycles > max_cycles) {
        max_cycles = ordered[j]->interval_cycles;
      }
    }
    const uint32_t span = max_cycles - min_cycles;

    const bool better =
        !result.found ||
        newest->ordinal > result.selected.ordinal ||
        (newest->ordinal == result.selected.ordinal &&
         span < result.span_cycles) ||
        (newest->ordinal == result.selected.ordinal &&
         span == result.span_cycles &&
         candidate_count > result.count);
    if (!better) continue;

    result.found = true;
    result.count = candidate_count;
    result.min_cycles = min_cycles;
    result.max_cycles = max_cycles;
    result.span_cycles = span;
    result.selected = *newest;
  }

  return result;
}

static void smartzero_advance_or_complete(void) {
  uint32_t next = g_smartzero.current_lane_index + 1U;
  while (next < SMARTZERO_LANE_COUNT) {
    const interrupt_subscriber_kind_t kind = smartzero_kind_for_index(next);
    if (!ocxo_kind_disabled(kind)) break;
    ++next;
  }
  if (next >= SMARTZERO_LANE_COUNT) {
    g_smartzero.running = false;
    g_smartzero.complete = true;
    g_smartzero.phase = interrupt_smartzero_phase_t::COMPLETE;
    g_smartzero.complete_count++;
    return;
  }
  g_smartzero.current_lane_index = next;
  smartzero_lane_runtime_t& runtime = g_smartzero.lanes[next];
  runtime.previous_present = false;
  runtime.pub.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  runtime.pub.last_decision = interrupt_smartzero_decision_t::NONE;
}

static void smartzero_feed_one_second(interrupt_subscriber_kind_t kind,
                                      uint32_t dwt_at_event,
                                      uint32_t counter32_at_event,
                                      uint16_t target_low16) {
  if (!smartzero_is_current_lane(kind)) return;
  const int index = smartzero_index_for_kind(kind);
  if (index < 0) return;

  // The reference CPS remains report evidence only.  Acceptance is decided by
  // repeatability within this lane, never by agreement with another oscillator.
  const uint32_t expected_cycles = interrupt_vclock_cycles_per_second();
  smartzero_write_begin();
  smartzero_lane_runtime_t& runtime = g_smartzero.lanes[index];
  interrupt_smartzero_lane_snapshot_t& lane = runtime.pub;
  lane.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  lane.sample_count++;
  lane.cps_used = expected_cycles;
  lane.expected_interval_cycles = expected_cycles;
  lane.last_sample_dwt = dwt_at_event;
  lane.last_sample_counter32 = counter32_at_event;
  lane.last_sample_hardware16 = target_low16;

  if (!runtime.previous_present) {
    runtime.previous_dwt = dwt_at_event;
    runtime.previous_counter32 = counter32_at_event;
    runtime.previous_hardware16 = target_low16;
    lane.previous_sample_dwt = dwt_at_event;
    lane.previous_sample_counter32 = counter32_at_event;
    lane.previous_sample_hardware16 = target_low16;
    lane.last_decision = interrupt_smartzero_decision_t::FIRST_SAMPLE;
    runtime.previous_present = true;
    smartzero_write_end();
    return;
  }

  const uint32_t previous_dwt = runtime.previous_dwt;
  const uint32_t previous_counter32 = runtime.previous_counter32;
  const uint16_t previous_hardware16 = runtime.previous_hardware16;
  const uint32_t interval_cycles = dwt_at_event - previous_dwt;
  const uint32_t counter_delta =
      counter32_at_event - previous_counter32;

  lane.previous_sample_dwt = previous_dwt;
  lane.previous_sample_counter32 = previous_counter32;
  lane.previous_sample_hardware16 = previous_hardware16;
  lane.interval_attempt_count++;
  lane.last_interval_cycles = interval_cycles;
  lane.last_counter_delta_ticks = counter_delta;

  if (expected_cycles != 0U) {
    const int64_t error64 =
        (int64_t)(uint64_t)interval_cycles -
        (int64_t)(uint64_t)expected_cycles;
    const int32_t error = error64 > INT32_MAX
        ? INT32_MAX
        : (error64 < INT32_MIN ? INT32_MIN : (int32_t)error64);
    const uint32_t abs_error = interrupt_abs_i32(error);
    lane.last_interval_error_cycles = error;
    if (abs_error > lane.max_abs_interval_error_cycles) {
      lane.max_abs_interval_error_cycles = abs_error;
    }
  } else {
    lane.waiting_for_cps_count++;
    lane.last_interval_error_cycles = 0;
  }

  // Every observed endpoint becomes the next interval's left endpoint.  This
  // naturally exposes a late-ISR positive spike followed by its negative
  // rebound, while retaining both actual endpoints of the interval just tested.
  runtime.previous_dwt = dwt_at_event;
  runtime.previous_counter32 = counter32_at_event;
  runtime.previous_hardware16 = target_low16;

  if (counter_delta != OCXO_ONE_SECOND_TICKS) {
    lane.rejected_count++;
    lane.last_decision = interrupt_smartzero_decision_t::REJECTED_COUNTER;
    smartzero_write_end();
    return;
  }

  smartzero_interval_sample_t sample{};
  sample.interval_cycles = interval_cycles;
  sample.previous_dwt = previous_dwt;
  sample.current_dwt = dwt_at_event;
  sample.previous_counter32 = previous_counter32;
  sample.current_counter32 = counter32_at_event;
  sample.current_hardware16 = target_low16;
  sample.ordinal = ++runtime.interval_ordinal;
  if (sample.ordinal == 0U) sample.ordinal = ++runtime.interval_ordinal;
  smartzero_history_push(runtime, sample);

  const smartzero_vote_result_t vote = smartzero_find_vote(runtime);
  runtime.closest_three_span_cycles = vote.closest_three_span_cycles;
  runtime.quorum_count = vote.found ? vote.count : 0U;
  runtime.quorum_min_cycles = vote.found ? vote.min_cycles : 0U;
  runtime.quorum_max_cycles = vote.found ? vote.max_cycles : 0U;
  runtime.quorum_span_cycles = vote.found ? vote.span_cycles : 0U;

  if (!vote.found) {
    lane.rejected_count++;
    // REJECTED_DWT is retained as the public ABI value; reports name this
    // condition NO_QUORUM because no individual DWT sample is condemned.
    lane.last_decision = interrupt_smartzero_decision_t::REJECTED_DWT;
    smartzero_write_end();
    return;
  }

  runtime.accepted_interval_cycles = vote.selected.interval_cycles;
  runtime.accepted_history_age =
      runtime.interval_ordinal - vote.selected.ordinal;
  lane.accepted_count++;
  lane.state = interrupt_smartzero_lane_state_t::LOCKED;
  lane.last_decision = interrupt_smartzero_decision_t::ACCEPTED;
  lane.anchor_dwt = vote.selected.current_dwt;
  lane.anchor_counter32 = vote.selected.current_counter32;
  lane.anchor_hardware16 = vote.selected.current_hardware16;
  lane.anchor_pair_previous_dwt = vote.selected.previous_dwt;
  lane.anchor_pair_previous_counter32 = vote.selected.previous_counter32;
  smartzero_advance_or_complete();
  smartzero_write_end();
}

bool interrupt_smartzero_begin(void) {
  if (!g_interrupt_runtime_ready || !g_interrupt_hw_ready) return false;
  const uint32_t prior = interrupt_priority0_guard_enter();
  smartzero_write_begin();
  g_smartzero.running = true;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::RUNNING;
  g_smartzero.sequence++;
  g_smartzero.begin_count++;
  g_smartzero.current_lane_index = 0U;
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
    smartzero_reset_lane(i);
  }
  g_smartzero.lanes[0].pub.state =
      interrupt_smartzero_lane_state_t::ACQUIRING;
  smartzero_write_end();
  interrupt_priority0_guard_exit(prior);
  return true;
}

void interrupt_smartzero_abort(void) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  smartzero_write_begin();
  g_smartzero.running = false;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::ABORTED;
  g_smartzero.abort_count++;
  g_smartzero.current_lane_index = SMARTZERO_LANE_COUNT;
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
    smartzero_reset_lane(i);
  }
  smartzero_write_end();
  interrupt_priority0_guard_exit(prior);
}

bool interrupt_smartzero_running(void) {
  return g_smartzero.running && !g_smartzero.complete;
}

bool interrupt_smartzero_complete(void) {
  return g_smartzero.complete;
}

static bool smartzero_snapshot_load(
    interrupt_smartzero_snapshot_t* out,
    smartzero_lane_vote_report_t* vote_reports = nullptr) {
  if (!out) return false;
  for (uint32_t attempt = 0; attempt < 4U; ++attempt) {
    const uint32_t seq1 = g_smartzero.seq;
    if (seq1 & 1U) continue;
    dmb_barrier();
    interrupt_smartzero_snapshot_t local{};
    smartzero_lane_vote_report_t local_votes[SMARTZERO_LANE_COUNT]{};
    local.phase = g_smartzero.phase;
    local.running = g_smartzero.running;
    local.complete = g_smartzero.complete;
    local.aborted = g_smartzero.phase == interrupt_smartzero_phase_t::ABORTED;
    local.sequence = g_smartzero.sequence;
    local.begin_count = g_smartzero.begin_count;
    local.complete_count = g_smartzero.complete_count;
    local.abort_count = g_smartzero.abort_count;
    local.current_lane_index = g_smartzero.current_lane_index;
    local.current_lane = g_smartzero.running
        ? smartzero_kind_for_index(g_smartzero.current_lane_index)
        : interrupt_subscriber_kind_t::NONE;
    local.tolerance_cycles = SMARTZERO_VOTE_SPAN_CYCLES;
    local.sample_rate_hz = 1U;
    local.counter_delta_ticks = OCXO_ONE_SECOND_TICKS;
    for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
      const smartzero_lane_runtime_t& runtime = g_smartzero.lanes[i];
      local.lanes[i] = runtime.pub;
      local_votes[i].history_count = runtime.history_count;
      local_votes[i].valid_interval_count = runtime.interval_ordinal;
      local_votes[i].quorum_count = runtime.quorum_count;
      local_votes[i].quorum_min_cycles = runtime.quorum_min_cycles;
      local_votes[i].quorum_max_cycles = runtime.quorum_max_cycles;
      local_votes[i].quorum_span_cycles = runtime.quorum_span_cycles;
      local_votes[i].closest_three_span_cycles =
          runtime.closest_three_span_cycles;
      local_votes[i].accepted_interval_cycles =
          runtime.accepted_interval_cycles;
      local_votes[i].accepted_history_age = runtime.accepted_history_age;
    }
    dmb_barrier();
    const uint32_t seq2 = g_smartzero.seq;
    if (seq1 == seq2 && (seq2 & 1U) == 0U) {
      *out = local;
      if (vote_reports) {
        for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
          vote_reports[i] = local_votes[i];
        }
      }
      return true;
    }
  }
  *out = interrupt_smartzero_snapshot_t{};
  if (vote_reports) {
    for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
      vote_reports[i] = smartzero_lane_vote_report_t{};
    }
  }
  return false;
}

bool interrupt_smartzero_live_snapshot(interrupt_smartzero_snapshot_t* out) {
  return smartzero_snapshot_load(out) && out->complete;
}

bool interrupt_smartzero_snapshot(interrupt_smartzero_snapshot_t* out) {
  return interrupt_smartzero_live_snapshot(out);
}

// ============================================================================
// QTimer helpers and authored one-second target custody
// ============================================================================

static inline void qtimer_clear_compare_flag(IMXRT_TMR_t& module,
                                              uint8_t channel) {
  module.CH[channel].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)module.CH[channel].CSCTRL;
}

static inline void qtimer_program_compare(IMXRT_TMR_t& module,
                                          uint8_t channel,
                                          uint16_t target_low16) {
  qtimer_clear_compare_flag(module, channel);
  module.CH[channel].COMP1 = target_low16;
  module.CH[channel].CMPLD1 = target_low16;
  qtimer_clear_compare_flag(module, channel);
  module.CH[channel].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer_disable_compare(IMXRT_TMR_t& module,
                                          uint8_t channel) {
  module.CH[channel].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer_clear_compare_flag(module, channel);
  qtimer_clear_compare_flag(module, channel);
}

static inline uint16_t qtimer1_ch0_counter_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_VCLOCK_CH].CNTR;
}

static inline void qtimer1_vclock_program_compare(uint16_t target_low16) {
  qtimer_program_compare(IMXRT_TMR1, QTIMER1_VCLOCK_CH, target_low16);
}

static inline void qtimer1_vclock_clear_compare_flag(void) {
  qtimer_clear_compare_flag(IMXRT_TMR1, QTIMER1_VCLOCK_CH);
}

static inline void qtimer1_ch2_clear_compare_flag(void) {
  qtimer_clear_compare_flag(IMXRT_TMR1, QTIMER1_TIMEPOP_CH);
}

static inline void qtimer1_ch2_disable_compare_hardware(void) {
  qtimer_disable_compare(IMXRT_TMR1, QTIMER1_TIMEPOP_CH);
}

static inline uint16_t ocxo_counter_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].CNTR;
}

static void ocxo_disable_compare(ocxo_lane_t& lane) {
  qtimer_disable_compare(*lane.module, lane.channel);
  lane.compare_armed = false;
  lane.armed_target_counter32 = 0U;
  lane.armed_target_low16 = 0U;
}

static uint16_t ocxo_hardware_low16_for_target(
    const synthetic_clock32_t& clock,
    uint32_t target_counter32) {
  return (uint16_t)(clock.hardware16 +
                    (uint16_t)(target_counter32 -
                               clock.current_counter32));
}

static bool ocxo_target_is_due_or_behind(uint32_t current_counter32,
                                         uint32_t target_counter32) {
  const uint32_t remaining = target_counter32 - current_counter32;
  return remaining == 0U || remaining > 0x7FFFFFFFUL;
}

static void ocxo_advance_target(ocxo_lane_t& lane) {
  lane.next_target_counter32 += OCXO_COMPARE_PERIOD_TICKS;
}

static void ocxo_reset_target_grid(ocxo_lane_t& lane,
                                   synthetic_clock32_t& clock) {
  lane.grid_epoch_counter32 = clock.current_counter32;
  lane.next_target_counter32 =
      clock.current_counter32 + OCXO_COMPARE_PERIOD_TICKS;
  lane.target_grid_valid = true;
  lane.target_grid_reset_count++;
}

static void ocxo_tend_and_arm(const ocxo_runtime_context_t& ctx) {
  ocxo_lane_t& lane = *ctx.lane;
  synthetic_clock32_t& clock = *ctx.clock32;
  if (!lane.initialized) return;

  // Read compare_armed first.  If the compare has not fired, it owns the lane
  // and the final < 65,536-tick interval needs no rollover tending.  If
  // priority 0 fired before this read, compare_armed is false and its subsequent
  // capture_pending write is the second half of the custody test.
  if (lane.compare_armed) return;
  if (lane.capture_pending) {
    lane.capture_pending_tend_skip_count++;
    return;
  }

  const uint16_t ambient_low16 = ocxo_counter_now(lane);
  synthetic_clock_tend_from_hardware(clock, ambient_low16);
  lane.rollover_tend_count++;
  lane.last_ambient_low16 = ambient_low16;

  if (!lane.active || !lane.target_grid_valid) return;

  while (ocxo_target_is_due_or_behind(clock.current_counter32,
                                      lane.next_target_counter32)) {
    ocxo_advance_target(lane);
    lane.missed_target_count++;
  }

  const uint32_t remaining =
      lane.next_target_counter32 - clock.current_counter32;
  if (remaining > OCXO_ARM_WINDOW_TICKS) {
    lane.arm_window_wait_count++;
    return;
  }
  if (remaining < OCXO_MIN_ARM_LEAD_TICKS) {
    lane.arm_too_close_count++;
    lane.missed_target_count++;
    ocxo_advance_target(lane);
    return;
  }

  const uint16_t target_low16 =
      ocxo_hardware_low16_for_target(clock, lane.next_target_counter32);
  lane.last_arm_dwt = ARM_DWT_CYCCNT;
  lane.last_arm_counter32 = clock.current_counter32;
  lane.last_arm_hardware16 = clock.hardware16;
  lane.last_arm_remaining_ticks = remaining;
  lane.armed_target_counter32 = lane.next_target_counter32;
  lane.armed_target_low16 = target_low16;

  // Publish software custody before hardware can assert the compare IRQ.
  dmb_barrier();
  lane.compare_armed = true;
  dmb_barrier();
  qtimer_program_compare(*lane.module, lane.channel, target_low16);
  lane.arm_count++;
}

// Lane-bound service boundaries deliberately take no context argument.  Each
// function reloads its permanent globals after entry, so a caller cannot carry
// a substituted ring/context pointer into the service-start dereference.
static __attribute__((noinline, noclone))
bool ocxo1_start_one_second_service_bound(void) {
  if (!ocxo1_binding_identity_check()) return false;
  ocxo_lane_t& lane = g_ocxo1_lane;
  synthetic_clock32_t& clock = g_ocxo1_clock32;
  if (!lane.initialized) return false;
  lane.active = true;
  if (!clock.zeroed) synthetic_clock_birth(clock, ocxo_counter_now(lane));
  if (!lane.target_grid_valid) {
    ocxo_reset_target_grid(lane, clock);
  }
  ocxo_tend_and_arm(g_ocxo1_ctx);
  interrupt_features_note_ocxo_custody();
  // A target one OCXO second away is intentionally not armed yet; VCLOCK
  // extends low-word lineage and tends it into the safe 16-bit arm window.
  return true;
}

static __attribute__((noinline, noclone))
bool ocxo2_start_one_second_service_bound(void) {
  if (!ocxo2_binding_identity_check()) return false;
  ocxo_lane_t& lane = g_ocxo2_lane;
  synthetic_clock32_t& clock = g_ocxo2_clock32;
  if (!lane.initialized) return false;
  lane.active = true;
  if (!clock.zeroed) synthetic_clock_birth(clock, ocxo_counter_now(lane));
  if (!lane.target_grid_valid) {
    ocxo_reset_target_grid(lane, clock);
  }
  ocxo_tend_and_arm(g_ocxo2_ctx);
  interrupt_features_note_ocxo_custody();
  return true;
}

static __attribute__((noinline, noclone))
void ocxo1_stop_one_second_service_bound(void) {
  g_ocxo1_lane.active = false;
  if (g_ocxo1_lane.initialized) ocxo_disable_compare(g_ocxo1_lane);
}

static __attribute__((noinline, noclone))
void ocxo2_stop_one_second_service_bound(void) {
  g_ocxo2_lane.active = false;
  if (g_ocxo2_lane.initialized) ocxo_disable_compare(g_ocxo2_lane);
}

// ============================================================================
// Clock API
// ============================================================================

bool interrupt_clock32_zero_from_ns(interrupt_subscriber_kind_t kind,
                                    uint64_t ns) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    const uint16_t hardware16 = qtimer1_ch0_counter_now();
    synthetic_clock_zero(g_vclock_clock32, ns);
    g_vclock_clock32.hardware_low16_at_zero = hardware16;
    vclock_anchor_hardware(g_vclock_clock32.zero_counter32, hardware16);
    g_vclock_lane.one_second_grid_valid = false;
    return true;
  }

  const ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  if (!ctx || !ctx->lane->initialized) return false;
  const uint16_t hardware16 = ocxo_counter_now(*ctx->lane);
  synthetic_clock_zero(*ctx->clock32, ns);
  ctx->clock32->hardware16 = hardware16;
  ocxo_reset_target_grid(*ctx->lane, *ctx->clock32);
  if (ctx->lane->compare_armed) ocxo_disable_compare(*ctx->lane);
  return true;
}

bool interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t kind,
                                            uint64_t ns) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_clock32.pending_zero = true;
    g_vclock_clock32.pending_zero_ns = ns;
    g_vclock_clock32.pending_zero_counter32 = clock32_from_ns(ns);
    g_vclock_clock32.pending_zero_count++;
    return true;
  }
  // OCXO epoch changes are coherent foreground transactions, never deferred
  // mutations.  CLOCKS uses interrupt_ocxo_logical_grid_epoch() for them.
  if (kind == interrupt_subscriber_kind_t::OCXO1 ||
      kind == interrupt_subscriber_kind_t::OCXO2) {
    return true;
  }
  return false;
}

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32) {
  struct install_t {
    const ocxo_runtime_context_t* ctx;
    uint32_t epoch;
  } installs[] = {
    { &g_ocxo1_ctx, ocxo1_epoch_counter32 },
    { &g_ocxo2_ctx, ocxo2_epoch_counter32 },
  };

  const uint32_t prior = interrupt_priority0_guard_enter();
  for (install_t& install : installs) {
    ocxo_lane_t& lane = *install.ctx->lane;
    synthetic_clock32_t& clock = *install.ctx->clock32;
    if (!lane.initialized) continue;
    if (lane.compare_armed) ocxo_disable_compare(lane);
    const uint16_t hardware16 = ocxo_counter_now(lane);
    clock.zeroed = true;
    clock.zero_counter32 = install.epoch;
    clock.zero_ns = (uint64_t)install.epoch * 100ULL;
    clock.current_counter32 = install.epoch;
    clock.current_ns = (uint64_t)install.epoch * 100ULL;
    clock.hardware16 = hardware16;
    clock.pending_zero = false;
    ocxo_reset_target_grid(lane, clock);
    lane.previous_event_valid = false;
    lane.previous_event_counter32 = 0U;
    lane.last_counter_delta_ticks = 0U;
  }
  interrupt_priority0_guard_exit(prior);
}

bool interrupt_clock_snapshot(interrupt_subscriber_kind_t kind,
                              interrupt_clock_snapshot_t* out) {
  if (!out) return false;
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    const uint16_t hardware16 = qtimer1_ch0_counter_now();
    out->hardware16 = hardware16;
    out->counter32 = vclock_synthetic_from_hardware_low16(hardware16);
    out->ns64 = (uint64_t)out->counter32 * 100ULL;
    return true;
  }
  const ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  if (!ctx || !ctx->lane->initialized || !ctx->clock32->zeroed) return false;
  const uint16_t hardware16 = ocxo_counter_now(*ctx->lane);
  const uint32_t delta =
      (uint32_t)((uint16_t)(hardware16 - ctx->clock32->hardware16));
  out->hardware16 = hardware16;
  out->counter32 = ctx->clock32->current_counter32 + delta;
  out->ns64 = ctx->clock32->current_ns + (uint64_t)delta * 100ULL;
  return true;
}

uint32_t interrupt_qtimer1_counter32_now(void) {
  return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
}

uint16_t interrupt_qtimer2_ch0_counter_now(void) {
  return OCXO1_DISABLED ? 0U : IMXRT_TMR2.CH[QTIMER2_OCXO1_CH].CNTR;
}

uint16_t interrupt_qtimer3_ch0_counter_now(void) {
  // Legacy accessor: OCXO2 authority is physically on QTimer3 CH3.
  return OCXO2_DISABLED ? 0U : IMXRT_TMR3.CH[0].CNTR;
}

uint16_t interrupt_qtimer3_ch3_counter_now(void) {
  return OCXO2_DISABLED ? 0U : IMXRT_TMR3.CH[QTIMER3_OCXO2_CH].CNTR;
}

// ============================================================================
// Native VCLOCK / TimePop hardware state
// ============================================================================

static volatile bool g_vclock_heartbeat_armed = false;
static volatile uint32_t g_vclock_heartbeat_next_counter32 = 0U;
static volatile uint16_t g_vclock_heartbeat_target_low16 = 0U;
static volatile uint32_t g_vclock_heartbeat_arm_count = 0U;
static volatile uint32_t g_vclock_capture_overrun_rearm_request_count = 0U;
static uint32_t g_vclock_capture_overrun_rearm_complete_count = 0U;
static uint32_t g_ocxo1_capture_overrun_recovery_count = 0U;
static uint32_t g_ocxo2_capture_overrun_recovery_count = 0U;
// CH2 has three distinct identities.  Never collapse them:
//
//   requested target — newest target authored by TimePop policy
//   deferred target  — newest request retained while one fire fact is in custody
//   armed target     — exact target physically committed to COMP1/CMPLD1
//
// Only armed_target may become event identity in the shared QTimer1 ISR.
static volatile uint32_t g_qtimer1_ch2_last_requested_target_counter32 = 0U;
static volatile uint32_t g_qtimer1_ch2_request_count = 0U;
static volatile bool g_qtimer1_ch2_compare_armed = false;
static volatile uint32_t g_qtimer1_ch2_armed_target_counter32 = 0U;
static volatile uint16_t g_qtimer1_ch2_armed_target_low16 = 0U;
static volatile uint32_t g_qtimer1_ch2_physical_arm_count = 0U;
static volatile uint32_t g_qtimer1_ch2_physical_reprogram_count = 0U;
static volatile bool g_qtimer1_ch2_fact_outstanding = false;
static volatile bool g_qtimer1_ch2_deferred_arm_valid = false;
static volatile uint32_t g_qtimer1_ch2_deferred_arm_target_counter32 = 0U;
static volatile uint32_t g_qtimer1_ch2_deferred_arm_count = 0U;
static volatile uint32_t g_qtimer1_ch2_deferred_arm_replace_count = 0U;
static volatile uint32_t g_qtimer1_ch2_release_arm_count = 0U;
static volatile uint32_t g_qtimer1_ch2_release_without_arm_count = 0U;
static volatile uint32_t g_qtimer1_ch2_valid_capture_count = 0U;
static volatile uint32_t g_qtimer1_ch2_rejected_flag_count = 0U;
static volatile uint32_t g_qtimer1_ch2_flag_while_disabled_count = 0U;
static volatile uint32_t g_qtimer1_ch2_flag_while_unarmed_count = 0U;
static volatile uint32_t g_qtimer1_ch2_flag_while_outstanding_count = 0U;
static volatile uint32_t g_qtimer1_ch2_target_mismatch_count = 0U;
// Compatibility aggregate retained for existing reports.  It now counts rejected
// shared-vector CH2 flags; no rejected flag is allowed to become an event fact.
static volatile uint32_t g_qtimer1_ch2_unexpected_capture_count = 0U;
static volatile interrupt_qtimer1_ch1_handler_fn g_qtimer1_ch1_handler = nullptr;

static volatile bool g_pps_relay_active = false;
static volatile uint32_t g_pps_relay_countdown_cells = 0U;
static volatile uint32_t g_pps_relay_assert_count = 0U;
static volatile uint32_t g_pps_relay_deassert_count = 0U;
static constexpr uint32_t PPS_RELAY_OFF_CELLS = 500U;

void interrupt_register_pps_entry_latency_handler(
    interrupt_pps_entry_latency_handler_fn callback) {
  g_pps_entry_latency_handler =
      interrupt_callback_address_executable((uintptr_t)callback)
          ? callback
          : nullptr;
}

void interrupt_register_qtimer1_ch1_handler(
    interrupt_qtimer1_ch1_handler_fn callback) {
  g_qtimer1_ch1_handler =
      interrupt_callback_address_executable((uintptr_t)callback)
          ? callback
          : nullptr;
}

bool interrupt_qtimer1_ch1_arm_compare(uint32_t) {
  return false;
}

void interrupt_qtimer1_ch1_disable_compare(void) {
  qtimer_disable_compare(IMXRT_TMR1, QTIMER1_RETIRED_AUX_CH);
}

uint16_t interrupt_qtimer1_ch1_counter_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CNTR;
}

uint16_t interrupt_qtimer1_ch1_comp1_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].COMP1;
}

uint16_t interrupt_qtimer1_ch1_csctrl_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CSCTRL;
}

uint32_t interrupt_vclock_counter32_observe_ambient(void) {
  return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
}

static void qtimer1_ch2_defer_arm_locked(uint32_t target_counter32) {
  if (g_qtimer1_ch2_deferred_arm_valid) {
    g_qtimer1_ch2_deferred_arm_replace_count++;
  }
  g_qtimer1_ch2_deferred_arm_target_counter32 = target_counter32;
  g_qtimer1_ch2_deferred_arm_valid = true;
  g_qtimer1_ch2_deferred_arm_count++;
}

// Commit one physical CH2 target.  Caller holds the Priority-0-preserving
// guard, but Priority 0 is deliberately still allowed to preempt.  The ordering
// therefore makes every intermediate state harmless to the shared-vector ISR:
// hardware remains disabled until armed identity and custody state are complete.
//
// For an ordinary reprogram, a legitimate old-target fire may preempt just
// before hardware disable.  After disabling, recheck fact custody; if Priority 0
// captured that old edge, preserve the new request as deferred work instead of
// erasing the newly captured fact.  release_owned_fact is true only when the
// caller is completing custody of the fact that intentionally holds the rail.
static bool qtimer1_ch2_commit_physical_arm_locked(
    uint32_t target_counter32,
    bool release_owned_fact) {
  auto& ch = IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH];
  const bool replacing = g_qtimer1_ch2_compare_armed ||
      (ch.CSCTRL & TMR_CSCTRL_TCF1EN) != 0U;
  const uint16_t target_low16 =
      vclock_hardware_low16_from_synthetic(target_counter32);

  qtimer1_ch2_disable_compare_hardware();
  dmb_barrier();

  if (!release_owned_fact && g_qtimer1_ch2_fact_outstanding) {
    qtimer1_ch2_defer_arm_locked(target_counter32);
    return false;
  }

  ch.COMP1 = target_low16;
  ch.CMPLD1 = target_low16;
  qtimer1_ch2_clear_compare_flag();

  // Publish actual hardware identity before enabling the interrupt source.
  g_qtimer1_ch2_armed_target_counter32 = target_counter32;
  g_qtimer1_ch2_armed_target_low16 = target_low16;
  g_qtimer1_ch2_compare_armed = true;
  g_qtimer1_ch2_fact_outstanding = false;
  dmb_barrier();

  // A match may have occurred while programming a disabled channel.  Clear once
  // more immediately before enabling; TCF1EN is always the final write.
  qtimer1_ch2_clear_compare_flag();
  ch.CSCTRL |= TMR_CSCTRL_TCF1EN;
  (void)ch.CSCTRL;

  g_qtimer1_ch2_physical_arm_count++;
  if (replacing) g_qtimer1_ch2_physical_reprogram_count++;
  return true;
}

void interrupt_qtimer1_ch2_arm_compare(uint32_t target_counter32) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  g_qtimer1_ch2_last_requested_target_counter32 = target_counter32;
  g_qtimer1_ch2_request_count++;

  // A captured CH2 edge owns the scheduler rail until foreground consumes it.
  // Scheduler policy may continue to author a desired next target while that
  // fact is in flight, but hardware remains disabled.  Retain only the newest
  // desired target; release commits that target as one atomic physical arm.
  if (g_qtimer1_ch2_fact_outstanding) {
    qtimer1_ch2_defer_arm_locked(target_counter32);
    interrupt_priority0_guard_exit(prior);
    return;
  }

  (void)qtimer1_ch2_commit_physical_arm_locked(target_counter32, false);
  interrupt_priority0_guard_exit(prior);
}

uint16_t interrupt_qtimer1_ch2_counter_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].CNTR;
}

uint16_t interrupt_qtimer1_ch2_comp1_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].COMP1;
}

uint16_t interrupt_qtimer1_ch2_csctrl_now(void) {
  return IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].CSCTRL;
}

static bool vclock_heartbeat_arm(void) {
  const uint16_t hardware16 = qtimer1_ch0_counter_now();
  vclock_tend_from_hardware(hardware16);
  const uint32_t target =
      g_vclock_clock32.current_counter32 + VCLOCK_INTERVAL_COUNTS;
  const uint16_t target_low16 =
      vclock_hardware_low16_from_synthetic(target);
  g_vclock_heartbeat_next_counter32 = target;
  g_vclock_heartbeat_target_low16 = target_low16;
  qtimer1_vclock_program_compare(target_low16);
  g_vclock_heartbeat_armed = true;
  g_vclock_heartbeat_arm_count++;
  return true;
}

// ============================================================================
// ISR capture rings / Priority-32 continuation
// ============================================================================

struct interrupt_handoff_source_diag_t {
  volatile uint32_t capture_count = 0;
  volatile uint32_t enqueue_count = 0;
  volatile uint32_t dequeue_count = 0;
  volatile uint32_t overrun_count = 0;
  volatile uint32_t high_water = 0;
  volatile uint32_t pending_count = 0;
  volatile uint32_t last_sequence = 0;
  volatile uint32_t last_capture_dwt = 0;
  volatile uint32_t last_capture_to_handoff_cycles = 0;
  volatile uint32_t max_capture_to_handoff_cycles = 0;
  volatile uint32_t latency_invalid_count = 0;
};

struct interrupt_handoff_diag_t {
  volatile bool configured = false;
  volatile bool running = false;
  volatile bool pending = false;
  volatile uint32_t configure_count = 0;
  volatile uint32_t request_count = 0;
  volatile uint32_t entry_count = 0;
  volatile uint32_t exit_count = 0;
  volatile uint32_t reentry_count = 0;
  volatile uint32_t repend_count = 0;
  volatile uint32_t drain_budget_exhausted_count = 0;
  volatile uint32_t last_request_dwt = 0;
  volatile uint32_t last_entry_dwt = 0;
  volatile uint32_t last_latency_cycles = 0;
  volatile uint32_t min_latency_cycles = UINT32_MAX;
  volatile uint32_t max_latency_cycles = 0;
  volatile uint32_t latency_invalid_count = 0;
};

static interrupt_handoff_diag_t g_interrupt_handoff{};
static interrupt_handoff_source_diag_t g_handoff_vclock{};
static interrupt_handoff_source_diag_t g_handoff_ch2{};
static interrupt_handoff_source_diag_t g_handoff_ocxo1{};
static interrupt_handoff_source_diag_t g_handoff_ocxo2{};
static interrupt_handoff_source_diag_t g_handoff_pps{};
static volatile uint32_t g_interrupt_capture_sequence = 0U;

// Minimal entry witness copied before handler work.  Classification is deferred
// to Priority 32 so sovereign capture remains short and the diagnostic itself
// cannot create the latency it is trying to measure.
enum class interrupt_execution_source_t : uint8_t {
  NONE = 0,
  QTIMER1 = 1,
  OCXO1 = 2,
  OCXO2 = 3,
  PPS = 4,
  CONTINUATION = 5,
};

struct interrupt_arrival_capture_t {
  bool spinidle_running = false;
  uint32_t spinidle_shadow_dwt = 0U;
  uint32_t pending_mask_at_entry = 0U;
  interrupt_execution_source_t blocker =
      interrupt_execution_source_t::NONE;
  uint32_t blocker_exit_dwt = 0U;
  uint32_t blocker_wall_cycles = 0U;
  bool target_pending_at_blocker_entry = false;
  uint32_t lower_context_active_mask = 0U;
};

struct interrupt_arrival_observation_t {
  interrupt_arrival_capture_t capture{};
  bool spinidle_shadow_valid = false;
  uint32_t spinidle_age_cycles = 0U;
  uint32_t spinidle_valid_threshold_cycles = 0U;
  uint32_t spinidle_excess_cycles = 0U;
  interrupt_delay_forensics_t delay{};
};

struct vclock_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  interrupt_arrival_capture_t arrival{};
  bool preempted_after_entry = false;
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t service_low16 = 0;
};

struct ch2_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t target_counter32 = 0;
};

struct ocxo_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  interrupt_arrival_capture_t arrival{};
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t ambient_low16 = 0;
  uint16_t compare_low16 = 0;
  bool active_at_capture = false;
};

struct pps_capture_packet_t {
  uint32_t sequence = 0;
  uint32_t isr_entry_dwt_raw = 0;
  interrupt_arrival_capture_t arrival{};
  uint16_t vclock_hardware16 = 0;
  uint16_t ocxo1_hardware16 = 0;
  uint16_t ocxo2_hardware16 = 0;
  bool ocxo1_valid = false;
  bool ocxo2_valid = false;
};

template <typename T, uint32_t N>
struct interrupt_capture_ring_t {
  T items[N]{};
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
};

static interrupt_capture_ring_t<vclock_capture_packet_t,
                                HANDOFF_QTIMER1_RING_SIZE>
    g_vclock_capture_ring{};
static interrupt_capture_ring_t<ch2_capture_packet_t,
                                HANDOFF_QTIMER1_RING_SIZE>
    g_ch2_capture_ring{};
static interrupt_capture_ring_t<ocxo_capture_packet_t,
                                HANDOFF_OCXO_RING_SIZE>
    g_ocxo1_capture_ring{};
static interrupt_capture_ring_t<ocxo_capture_packet_t,
                                HANDOFF_OCXO_RING_SIZE>
    g_ocxo2_capture_ring{};
static interrupt_capture_ring_t<pps_capture_packet_t,
                                HANDOFF_PPS_RING_SIZE>
    g_pps_capture_ring{};


struct timepop_foreground_packet_t {
  uint32_t sequence = 0U;
  uint32_t isr_entry_dwt_raw = 0U;
  uint32_t dwt_at_event = 0U;
  uint32_t target_counter32 = 0U;
};

struct timepop_foreground_diag_t {
  volatile uint32_t enqueue_count = 0U;
  volatile uint32_t dequeue_count = 0U;
  volatile uint32_t overrun_count = 0U;
  volatile uint32_t high_water = 0U;
  volatile uint32_t service_count = 0U;
  volatile uint32_t handler_reject_count = 0U;
  volatile uint32_t handler_last_ipsr = 0U;
  volatile uint32_t scheduler_recover_count = 0U;
};

static timepop_foreground_packet_t g_timepop_foreground_mailbox{};
static volatile bool g_timepop_foreground_mailbox_pending = false;
static timepop_foreground_diag_t g_timepop_foreground_diag{};
static volatile bool g_timepop_foreground_rearm_requested = false;
static bool g_interrupt_foreground_service_running = false;

// ============================================================================
// Foreground forensic black box
// ============================================================================

// A record is retained only when a threshold is crossed or TimePop explicitly
// attaches a scheduler-pressure marker.  Routine loop passes update maxima but do
// not consume ring space.  All writes are foreground-only; the sequence word makes
// report snapshots coherent without masking sacred priority 0.
enum : uint32_t {
  INTERRUPT_FORENSIC_TRIGGER_SERVICE_LONG = 1UL << 0,
  INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_LONG = 1UL << 1,
  INTERRUPT_FORENSIC_TRIGGER_SUBSCRIBER_LONG = 1UL << 2,
  INTERRUPT_FORENSIC_TRIGGER_FEATURE_LONG = 1UL << 3,
  INTERRUPT_FORENSIC_TRIGGER_FINALIZE_LONG = 1UL << 4,
  INTERRUPT_FORENSIC_TRIGGER_LOOP_GAP_LONG = 1UL << 5,
  INTERRUPT_FORENSIC_TRIGGER_CH2_AGE_LONG = 1UL << 6,
  INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_PRESSURE = 1UL << 7,
};

enum : uint32_t {
  INTERRUPT_FORENSIC_STATE_MAILBOX_PENDING = 1UL << 0,
  INTERRUPT_FORENSIC_STATE_CH2_FACT_OUTSTANDING = 1UL << 1,
  INTERRUPT_FORENSIC_STATE_CH2_COMPARE_ARMED = 1UL << 2,
  INTERRUPT_FORENSIC_STATE_CH2_DEFERRED_VALID = 1UL << 3,
  INTERRUPT_FORENSIC_STATE_FOREGROUND_PENDING = 1UL << 4,
  INTERRUPT_FORENSIC_STATE_REARM_REQUESTED = 1UL << 5,
};

struct interrupt_foreground_forensic_record_t {
  uint32_t sequence = 0U;
  uint32_t trigger_flags = 0U;

  uint32_t prior_service_exit_dwt = 0U;
  uint32_t prior_service_exit_counter32 = 0U;
  uint32_t service_start_dwt = 0U;
  uint32_t service_end_dwt = 0U;
  uint32_t service_gap_cycles = 0U;
  uint32_t service_gap_ticks = 0U;
  uint32_t service_total_cycles = 0U;
  uint32_t timepop_cycles = 0U;
  uint32_t subscriber_cycles = 0U;
  uint32_t feature_cycles = 0U;
  uint32_t finalize_cycles = 0U;

  uint32_t counter32_entry = 0U;
  uint32_t counter32_exit = 0U;
  uint32_t counter32_delta_ticks = 0U;
  uint32_t pps_count_entry = 0U;
  uint32_t pps_count_exit = 0U;
  uint32_t state_flags_entry = 0U;
  uint32_t state_flags_exit = 0U;

  uint32_t ch2_packet_sequence = 0U;
  uint32_t ch2_event_target_counter32 = 0U;
  uint32_t ch2_service_counter32 = 0U;
  int32_t ch2_event_age_ticks = 0;
  uint32_t ch2_event_age_abs_ticks = 0U;
  uint32_t ch2_capture_to_foreground_cycles = 0U;

  uint32_t timepop_service_count_before = 0U;
  uint32_t timepop_service_count_after = 0U;

  uint32_t pressure_marker_count = 0U;
  interrupt_timepop_forensic_source_t pressure_source =
      interrupt_timepop_forensic_source_t::NONE;
  uint32_t pressure_now_counter32 = 0U;
  uint32_t pressure_deadline_counter32 = 0U;
  int32_t pressure_signed_delta_ticks = 0;
  uint32_t pressure_slot = UINT32_MAX;
  uint32_t pressure_handle = 0U;
  uint32_t pressure_phase = 0U;
  uint32_t pressure_dwt = 0U;
};

struct interrupt_foreground_forensic_runtime_t {
  volatile uint32_t write_seq = 0U;
  interrupt_foreground_forensic_record_t
      records[INTERRUPT_FORENSIC_RING_SIZE]{};

  uint32_t service_pass_count = 0U;
  uint32_t total_recorded = 0U;
  uint32_t overwrite_count = 0U;
  uint32_t next_record_sequence = 0U;
  uint32_t previous_service_exit_dwt = 0U;
  uint32_t previous_service_exit_counter32 = 0U;

  uint32_t pressure_marker_count = 0U;
  uint32_t pressure_marker_outside_service_count = 0U;
  uint32_t pressure_marker_isr_reject_count = 0U;

  uint32_t max_service_gap_cycles = 0U;
  uint32_t max_service_gap_ticks = 0U;
  uint32_t max_service_total_cycles = 0U;
  uint32_t max_timepop_cycles = 0U;
  uint32_t max_subscriber_cycles = 0U;
  uint32_t max_feature_cycles = 0U;
  uint32_t max_finalize_cycles = 0U;
  uint32_t max_ch2_event_age_abs_ticks = 0U;
  uint32_t max_ch2_capture_to_foreground_cycles = 0U;
};

static interrupt_foreground_forensic_runtime_t g_interrupt_foreground_forensics{};
static interrupt_foreground_forensic_record_t g_interrupt_foreground_forensic_live{};
static bool g_interrupt_foreground_forensic_live_active = false;

static uint32_t interrupt_forensic_state_flags(void) {
  uint32_t flags = 0U;
  if (g_timepop_foreground_mailbox_pending) {
    flags |= INTERRUPT_FORENSIC_STATE_MAILBOX_PENDING;
  }
  if (g_qtimer1_ch2_fact_outstanding) {
    flags |= INTERRUPT_FORENSIC_STATE_CH2_FACT_OUTSTANDING;
  }
  if (g_qtimer1_ch2_compare_armed) {
    flags |= INTERRUPT_FORENSIC_STATE_CH2_COMPARE_ARMED;
  }
  if (g_qtimer1_ch2_deferred_arm_valid) {
    flags |= INTERRUPT_FORENSIC_STATE_CH2_DEFERRED_VALID;
  }
  if (g_process_interrupt_foreground_pending) {
    flags |= INTERRUPT_FORENSIC_STATE_FOREGROUND_PENDING;
  }
  if (g_timepop_foreground_rearm_requested) {
    flags |= INTERRUPT_FORENSIC_STATE_REARM_REQUESTED;
  }
  return flags;
}

static void interrupt_forensics_write_begin(void) {
  g_interrupt_foreground_forensics.write_seq++;
  dmb_barrier();
}

static void interrupt_forensics_write_end(void) {
  dmb_barrier();
  g_interrupt_foreground_forensics.write_seq++;
}

static void interrupt_forensics_commit_record(
    interrupt_foreground_forensic_record_t record) {
  interrupt_forensics_write_begin();
  record.sequence = ++g_interrupt_foreground_forensics.next_record_sequence;
  if (record.sequence == 0U) {
    record.sequence = ++g_interrupt_foreground_forensics.next_record_sequence;
  }
  const uint32_t slot =
      g_interrupt_foreground_forensics.total_recorded %
      INTERRUPT_FORENSIC_RING_SIZE;
  g_interrupt_foreground_forensics.records[slot] = record;
  if (g_interrupt_foreground_forensics.total_recorded >=
      INTERRUPT_FORENSIC_RING_SIZE) {
    g_interrupt_foreground_forensics.overwrite_count++;
  }
  g_interrupt_foreground_forensics.total_recorded++;
  interrupt_forensics_write_end();
}

static void interrupt_forensics_begin_service(void) {
  interrupt_foreground_forensic_runtime_t& runtime =
      g_interrupt_foreground_forensics;
  interrupt_foreground_forensic_record_t& record =
      g_interrupt_foreground_forensic_live;
  record = interrupt_foreground_forensic_record_t{};
  record.prior_service_exit_dwt = runtime.previous_service_exit_dwt;
  record.prior_service_exit_counter32 =
      runtime.previous_service_exit_counter32;
  record.service_start_dwt = ARM_DWT_CYCCNT;
  record.counter32_entry = interrupt_qtimer1_counter32_now();
  if (record.prior_service_exit_dwt != 0U) {
    record.service_gap_cycles =
        record.service_start_dwt - record.prior_service_exit_dwt;
    record.service_gap_ticks =
        record.counter32_entry - record.prior_service_exit_counter32;
  }
  record.pps_count_entry = g_pps_gpio_heartbeat.edge_count;
  record.state_flags_entry = interrupt_forensic_state_flags();
  record.timepop_service_count_before =
      (uint32_t)g_timepop_foreground_diag.service_count;
  g_interrupt_foreground_forensic_live_active = true;
}

static void interrupt_forensics_note_ch2_foreground(
    const timepop_foreground_packet_t& packet,
    uint32_t service_dwt,
    uint32_t service_counter32) {
  if (!g_interrupt_foreground_forensic_live_active) return;
  interrupt_foreground_forensic_record_t& record =
      g_interrupt_foreground_forensic_live;
  record.ch2_packet_sequence = packet.sequence;
  record.ch2_event_target_counter32 = packet.target_counter32;
  record.ch2_service_counter32 = service_counter32;
  record.ch2_event_age_ticks =
      (int32_t)(service_counter32 - packet.target_counter32);
  record.ch2_event_age_abs_ticks =
      interrupt_abs_i32(record.ch2_event_age_ticks);
  record.ch2_capture_to_foreground_cycles =
      service_dwt - packet.isr_entry_dwt_raw;
  if (record.ch2_event_age_abs_ticks >=
      INTERRUPT_FORENSIC_CH2_AGE_THRESHOLD_TICKS) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_CH2_AGE_LONG;
  }
}

static void interrupt_forensics_finish_service(void) {
  interrupt_foreground_forensic_runtime_t& runtime =
      g_interrupt_foreground_forensics;
  interrupt_foreground_forensic_record_t& record =
      g_interrupt_foreground_forensic_live;
  record.service_end_dwt = ARM_DWT_CYCCNT;
  record.service_total_cycles =
      record.service_end_dwt - record.service_start_dwt;
  record.counter32_exit = interrupt_qtimer1_counter32_now();
  record.counter32_delta_ticks =
      record.counter32_exit - record.counter32_entry;
  record.pps_count_exit = g_pps_gpio_heartbeat.edge_count;
  record.state_flags_exit = interrupt_forensic_state_flags();
  record.timepop_service_count_after =
      (uint32_t)g_timepop_foreground_diag.service_count;

  runtime.service_pass_count++;
  if (record.service_gap_cycles > runtime.max_service_gap_cycles) {
    runtime.max_service_gap_cycles = record.service_gap_cycles;
  }
  if (record.service_gap_ticks > runtime.max_service_gap_ticks) {
    runtime.max_service_gap_ticks = record.service_gap_ticks;
  }
  if (record.service_total_cycles > runtime.max_service_total_cycles) {
    runtime.max_service_total_cycles = record.service_total_cycles;
  }
  if (record.timepop_cycles > runtime.max_timepop_cycles) {
    runtime.max_timepop_cycles = record.timepop_cycles;
  }
  if (record.subscriber_cycles > runtime.max_subscriber_cycles) {
    runtime.max_subscriber_cycles = record.subscriber_cycles;
  }
  if (record.feature_cycles > runtime.max_feature_cycles) {
    runtime.max_feature_cycles = record.feature_cycles;
  }
  if (record.finalize_cycles > runtime.max_finalize_cycles) {
    runtime.max_finalize_cycles = record.finalize_cycles;
  }
  if (record.ch2_event_age_abs_ticks >
      runtime.max_ch2_event_age_abs_ticks) {
    runtime.max_ch2_event_age_abs_ticks = record.ch2_event_age_abs_ticks;
  }
  if (record.ch2_capture_to_foreground_cycles >
      runtime.max_ch2_capture_to_foreground_cycles) {
    runtime.max_ch2_capture_to_foreground_cycles =
        record.ch2_capture_to_foreground_cycles;
  }

  if (record.service_gap_cycles >=
          INTERRUPT_FORENSIC_GAP_THRESHOLD_CYCLES ||
      record.service_gap_ticks >=
          INTERRUPT_FORENSIC_GAP_THRESHOLD_TICKS) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_LOOP_GAP_LONG;
  }
  if (record.service_total_cycles >=
      INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_SERVICE_LONG;
  }
  if (record.timepop_cycles >=
      INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_LONG;
  }
  if (record.subscriber_cycles >=
      INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_SUBSCRIBER_LONG;
  }
  if (record.feature_cycles >=
      INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_FEATURE_LONG;
  }
  if (record.finalize_cycles >=
      INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES) {
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_FINALIZE_LONG;
  }

  runtime.previous_service_exit_dwt = record.service_end_dwt;
  runtime.previous_service_exit_counter32 = record.counter32_exit;
  g_interrupt_foreground_forensic_live_active = false;
  if (record.trigger_flags != 0U) {
    interrupt_forensics_commit_record(record);
  }
}

void interrupt_forensics_note_timepop_pressure(
    interrupt_timepop_forensic_source_t source,
    uint32_t now_counter32,
    uint32_t deadline_counter32,
    int32_t signed_now_minus_deadline_ticks,
    uint32_t slot,
    uint32_t handle,
    uint32_t phase) {
  if (interrupt_ipsr() != 0U) {
    g_interrupt_foreground_forensics.pressure_marker_isr_reject_count++;
    return;
  }

  g_interrupt_foreground_forensics.pressure_marker_count++;
  const uint32_t marker_dwt = ARM_DWT_CYCCNT;
  if (g_interrupt_foreground_forensic_live_active) {
    interrupt_foreground_forensic_record_t& record =
        g_interrupt_foreground_forensic_live;
    record.trigger_flags |= INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_PRESSURE;
    record.pressure_marker_count++;
    if (record.pressure_marker_count == 1U ||
        interrupt_abs_i32(signed_now_minus_deadline_ticks) >=
            interrupt_abs_i32(record.pressure_signed_delta_ticks)) {
      record.pressure_source = source;
      record.pressure_now_counter32 = now_counter32;
      record.pressure_deadline_counter32 = deadline_counter32;
      record.pressure_signed_delta_ticks =
          signed_now_minus_deadline_ticks;
      record.pressure_slot = slot;
      record.pressure_handle = handle;
      record.pressure_phase = phase;
      record.pressure_dwt = marker_dwt;
    }
    return;
  }

  // A TimePop pressure decision may occur outside the ordinary interrupt bridge
  // (for example, a direct foreground arm API).  Preserve it as a stand-alone
  // record rather than silently losing the evidence.
  g_interrupt_foreground_forensics.pressure_marker_outside_service_count++;
  interrupt_foreground_forensic_record_t record{};
  record.trigger_flags = INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_PRESSURE;
  record.service_start_dwt = marker_dwt;
  record.service_end_dwt = marker_dwt;
  record.counter32_entry = interrupt_qtimer1_counter32_now();
  record.counter32_exit = record.counter32_entry;
  record.state_flags_entry = interrupt_forensic_state_flags();
  record.state_flags_exit = record.state_flags_entry;
  record.pressure_marker_count = 1U;
  record.pressure_source = source;
  record.pressure_now_counter32 = now_counter32;
  record.pressure_deadline_counter32 = deadline_counter32;
  record.pressure_signed_delta_ticks = signed_now_minus_deadline_ticks;
  record.pressure_slot = slot;
  record.pressure_handle = handle;
  record.pressure_phase = phase;
  record.pressure_dwt = marker_dwt;
  interrupt_forensics_commit_record(record);
}

static inline volatile uint32_t& interrupt_nvic_iser_word(uint32_t irq) {
  volatile uint32_t* const base = (volatile uint32_t*)0xE000E100UL;
  return base[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_ispr_word(uint32_t irq) {
  volatile uint32_t* const base = (volatile uint32_t*)0xE000E200UL;
  return base[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_iabr_word(uint32_t irq) {
  volatile uint32_t* const base = (volatile uint32_t*)0xE000E300UL;
  return base[irq >> 5];
}

static inline volatile uint32_t& interrupt_nvic_icpr_word(uint32_t irq) {
  volatile uint32_t* const base = (volatile uint32_t*)0xE000E280UL;
  return base[irq >> 5];
}

static inline uint32_t interrupt_nvic_irq_mask(uint32_t irq) {
  return 1UL << (irq & 31U);
}

static inline uint32_t interrupt_nvic_priority_read(uint32_t irq) {
  volatile const uint8_t* const priorities =
      (volatile const uint8_t*)0xE000E400UL;
  return (uint32_t)priorities[irq];
}

static inline bool interrupt_nvic_enabled(uint32_t irq) {
  return (interrupt_nvic_iser_word(irq) &
          interrupt_nvic_irq_mask(irq)) != 0U;
}

static inline bool interrupt_nvic_pending(uint32_t irq) {
  return (interrupt_nvic_ispr_word(irq) &
          interrupt_nvic_irq_mask(irq)) != 0U;
}

static inline bool interrupt_nvic_active(uint32_t irq) {
  return (interrupt_nvic_iabr_word(irq) &
          interrupt_nvic_irq_mask(irq)) != 0U;
}

// ============================================================================
// ISR entry-delay courtroom
// ============================================================================
//
// SpinIdle supplies the last thread-mode DWT breadcrumb.  A small per-source
// baseline learns the normal shadow-to-first-instruction age.  Per-target
// pending-at-blocker-exit latches distinguish ordinary preemption from actual
// equal/higher-priority serialization.  QTimer counter-vs-target lateness
// is used as a second, quantized witness.  No result mutates the captured edge.

static constexpr uint32_t INTERRUPT_DELAY_SPIN_BASELINE_MIN_SAMPLES = 3U;
static constexpr uint32_t INTERRUPT_DELAY_SPIN_CANDIDATE_MAX_CYCLES = 256U;
static constexpr uint32_t INTERRUPT_DELAY_SPIN_GUARD_CYCLES = 12U;
static constexpr uint32_t INTERRUPT_DELAY_TAILCHAIN_MAX_GAP_CYCLES = 512U;
static constexpr uint32_t INTERRUPT_DELAY_SERVICE_BASELINE_MIN_SAMPLES = 3U;
static constexpr int32_t INTERRUPT_DELAY_SERVICE_NORMAL_JITTER_TICKS = 1;
static constexpr int32_t INTERRUPT_DELAY_SERVICE_CANDIDATE_MIN_TICKS = -2;
static constexpr int32_t INTERRUPT_DELAY_SERVICE_CANDIDATE_MAX_TICKS = 8;

static constexpr uint32_t INTERRUPT_DELAY_SOURCE_BIT_QTIMER1 = 1U << 0;
static constexpr uint32_t INTERRUPT_DELAY_SOURCE_BIT_OCXO1 = 1U << 1;
static constexpr uint32_t INTERRUPT_DELAY_SOURCE_BIT_OCXO2 = 1U << 2;
static constexpr uint32_t INTERRUPT_DELAY_SOURCE_BIT_PPS = 1U << 3;
static constexpr uint32_t INTERRUPT_DELAY_SOURCE_BIT_CONTINUATION = 1U << 4;

struct interrupt_delay_baseline_runtime_t {
  bool spin_valid = false;
  uint32_t spin_sample_count = 0U;
  uint32_t spin_baseline_cycles = 0U;

  bool service_valid = false;
  uint32_t service_sample_count = 0U;
  int32_t service_baseline_ticks = 0;
};

struct interrupt_delay_blocker_latch_t {
  volatile uint32_t seq = 0U;
  volatile uint32_t publish_token = 0U;
  volatile interrupt_execution_source_t blocker =
      interrupt_execution_source_t::NONE;
  volatile uint32_t blocker_exit_dwt = 0U;
  volatile uint32_t blocker_wall_cycles = 0U;
  volatile bool target_pending_at_blocker_entry = false;
};

static interrupt_delay_baseline_runtime_t g_delay_qtimer1{};
static interrupt_delay_baseline_runtime_t g_delay_ocxo1{};
static interrupt_delay_baseline_runtime_t g_delay_ocxo2{};
static interrupt_delay_baseline_runtime_t g_delay_pps{};
static interrupt_delay_blocker_latch_t g_delay_latch_qtimer1{};
static interrupt_delay_blocker_latch_t g_delay_latch_ocxo1{};
static interrupt_delay_blocker_latch_t g_delay_latch_ocxo2{};
static interrupt_delay_blocker_latch_t g_delay_latch_pps{};
static uint32_t g_delay_seen_qtimer1_token = 0U;
static uint32_t g_delay_seen_ocxo1_token = 0U;
static uint32_t g_delay_seen_ocxo2_token = 0U;
static uint32_t g_delay_seen_pps_token = 0U;

static interrupt_delay_forensics_t g_pps_interrupt_delay{};
static interrupt_arrival_capture_t g_pps_arrival_capture{};
static uint32_t g_pps_arrival_entry_dwt = 0U;
static bool g_pps_previous_delay_valid = false;
static bool g_pps_previous_endpoint_delayed = false;
static interrupt_delay_cause_t g_pps_previous_delayed_by =
    interrupt_delay_cause_t::NONE;
static uint32_t g_pps_previous_delay_cycles = 0U;
static bool g_pps_previous_interval_delay_valid = false;
static int32_t g_pps_previous_interval_delay_cycles = 0;
static interrupt_delay_cause_t g_pps_previous_interval_delayed_by =
    interrupt_delay_cause_t::NONE;

static void interrupt_delay_runtime_reset(void) {
  g_delay_qtimer1 = interrupt_delay_baseline_runtime_t{};
  g_delay_ocxo1 = interrupt_delay_baseline_runtime_t{};
  g_delay_ocxo2 = interrupt_delay_baseline_runtime_t{};
  g_delay_pps = interrupt_delay_baseline_runtime_t{};
  g_delay_latch_qtimer1 = interrupt_delay_blocker_latch_t{};
  g_delay_latch_ocxo1 = interrupt_delay_blocker_latch_t{};
  g_delay_latch_ocxo2 = interrupt_delay_blocker_latch_t{};
  g_delay_latch_pps = interrupt_delay_blocker_latch_t{};
  g_delay_seen_qtimer1_token = 0U;
  g_delay_seen_ocxo1_token = 0U;
  g_delay_seen_ocxo2_token = 0U;
  g_delay_seen_pps_token = 0U;
  g_pps_interrupt_delay = interrupt_delay_forensics_t{};
  g_pps_arrival_capture = interrupt_arrival_capture_t{};
  g_pps_arrival_entry_dwt = 0U;
  g_pps_previous_delay_valid = false;
  g_pps_previous_endpoint_delayed = false;
  g_pps_previous_delayed_by = interrupt_delay_cause_t::NONE;
  g_pps_previous_delay_cycles = 0U;
  g_pps_previous_interval_delay_valid = false;
  g_pps_previous_interval_delay_cycles = 0;
  g_pps_previous_interval_delayed_by = interrupt_delay_cause_t::NONE;
}

static uint32_t interrupt_delay_source_bit(interrupt_execution_source_t source) {
  switch (source) {
    case interrupt_execution_source_t::QTIMER1:
      return INTERRUPT_DELAY_SOURCE_BIT_QTIMER1;
    case interrupt_execution_source_t::OCXO1:
      return INTERRUPT_DELAY_SOURCE_BIT_OCXO1;
    case interrupt_execution_source_t::OCXO2:
      return INTERRUPT_DELAY_SOURCE_BIT_OCXO2;
    case interrupt_execution_source_t::PPS:
      return INTERRUPT_DELAY_SOURCE_BIT_PPS;
    case interrupt_execution_source_t::CONTINUATION:
      return INTERRUPT_DELAY_SOURCE_BIT_CONTINUATION;
    default:
      return 0U;
  }
}

static uint32_t interrupt_delay_source_priority(
    interrupt_execution_source_t source) {
  switch (source) {
    case interrupt_execution_source_t::OCXO1:
    case interrupt_execution_source_t::OCXO2:
    case interrupt_execution_source_t::PPS:
      return INTERRUPT_PRIORITY_SCIENCE;
    case interrupt_execution_source_t::QTIMER1:
      return INTERRUPT_PRIORITY_VCLOCK_TIMEPOP;
    case interrupt_execution_source_t::CONTINUATION:
      return INTERRUPT_PRIORITY_CONTINUATION;
    default:
      return UINT32_MAX;
  }
}

static interrupt_delay_cause_t interrupt_delay_cause_from_source(
    interrupt_execution_source_t source) {
  switch (source) {
    case interrupt_execution_source_t::QTIMER1:
      return interrupt_delay_cause_t::VCLOCK_TIMEPOP;
    case interrupt_execution_source_t::OCXO1:
      return interrupt_delay_cause_t::OCXO1;
    case interrupt_execution_source_t::OCXO2:
      return interrupt_delay_cause_t::OCXO2;
    case interrupt_execution_source_t::PPS:
      return interrupt_delay_cause_t::PPS;
    case interrupt_execution_source_t::CONTINUATION:
      return interrupt_delay_cause_t::CONTINUATION;
    default:
      return interrupt_delay_cause_t::UNKNOWN;
  }
}

static interrupt_delay_baseline_runtime_t* interrupt_delay_baseline_for(
    interrupt_execution_source_t source) {
  switch (source) {
    case interrupt_execution_source_t::QTIMER1: return &g_delay_qtimer1;
    case interrupt_execution_source_t::OCXO1: return &g_delay_ocxo1;
    case interrupt_execution_source_t::OCXO2: return &g_delay_ocxo2;
    case interrupt_execution_source_t::PPS: return &g_delay_pps;
    default: return nullptr;
  }
}

static uint32_t interrupt_delay_pending_mask(void) {
  static_assert(((uint32_t)IRQ_QTIMER1 >> 5U) ==
                    ((uint32_t)IRQ_QTIMER2 >> 5U) &&
                ((uint32_t)IRQ_QTIMER2 >> 5U) ==
                    ((uint32_t)IRQ_QTIMER3 >> 5U),
                "QTimer delay witnesses must share one NVIC pending word");

  // This executes only at sparse Priority-0 exits.  Read the three QTimer IRQs
  // from one NVIC word so the witness minimally extends the collision it records.
  const uint32_t qtimer_pending =
      interrupt_nvic_ispr_word((uint32_t)IRQ_QTIMER1);
  const uint32_t gpio_pending =
      interrupt_nvic_ispr_word((uint32_t)IRQ_GPIO6789);

  uint32_t mask = 0U;
  if ((qtimer_pending & interrupt_nvic_irq_mask((uint32_t)IRQ_QTIMER1)) != 0U) {
    mask |= INTERRUPT_DELAY_SOURCE_BIT_QTIMER1;
  }
  if (!OCXO1_DISABLED &&
      (qtimer_pending & interrupt_nvic_irq_mask((uint32_t)IRQ_QTIMER2)) != 0U) {
    mask |= INTERRUPT_DELAY_SOURCE_BIT_OCXO1;
  }
  if (!OCXO2_DISABLED &&
      (qtimer_pending & interrupt_nvic_irq_mask((uint32_t)IRQ_QTIMER3)) != 0U) {
    mask |= INTERRUPT_DELAY_SOURCE_BIT_OCXO2;
  }
  if ((gpio_pending & interrupt_nvic_irq_mask((uint32_t)IRQ_GPIO6789)) != 0U) {
    mask |= INTERRUPT_DELAY_SOURCE_BIT_PPS;
  }
  return mask;
}

static uint32_t interrupt_delay_lower_active_mask(
    interrupt_execution_source_t source) {
  uint32_t mask = 0U;
  if (source == interrupt_execution_source_t::OCXO1 ||
      source == interrupt_execution_source_t::OCXO2 ||
      source == interrupt_execution_source_t::PPS) {
    if (g_interrupt_priority_runtime.qtimer1.active) {
      mask |= INTERRUPT_DELAY_SOURCE_BIT_QTIMER1;
    }
    if (g_interrupt_priority_runtime.continuation.active) {
      mask |= INTERRUPT_DELAY_SOURCE_BIT_CONTINUATION;
    }
  } else if (source == interrupt_execution_source_t::QTIMER1) {
    if (g_interrupt_priority_runtime.continuation.active) {
      mask |= INTERRUPT_DELAY_SOURCE_BIT_CONTINUATION;
    }
  }
  return mask;
}

static interrupt_delay_blocker_latch_t* interrupt_delay_latch_for(
    interrupt_execution_source_t target) {
  switch (target) {
    case interrupt_execution_source_t::QTIMER1:
      return &g_delay_latch_qtimer1;
    case interrupt_execution_source_t::OCXO1:
      return &g_delay_latch_ocxo1;
    case interrupt_execution_source_t::OCXO2:
      return &g_delay_latch_ocxo2;
    case interrupt_execution_source_t::PPS:
      return &g_delay_latch_pps;
    default:
      return nullptr;
  }
}

static uint32_t* interrupt_delay_seen_token_for(
    interrupt_execution_source_t target) {
  switch (target) {
    case interrupt_execution_source_t::QTIMER1:
      return &g_delay_seen_qtimer1_token;
    case interrupt_execution_source_t::OCXO1:
      return &g_delay_seen_ocxo1_token;
    case interrupt_execution_source_t::OCXO2:
      return &g_delay_seen_ocxo2_token;
    case interrupt_execution_source_t::PPS:
      return &g_delay_seen_pps_token;
    default:
      return nullptr;
  }
}

static bool interrupt_delay_source_active(
    interrupt_execution_source_t source) {
  switch (source) {
    case interrupt_execution_source_t::QTIMER1:
      return g_interrupt_priority_runtime.qtimer1.active;
    case interrupt_execution_source_t::OCXO1:
      return g_interrupt_priority_runtime.qtimer2.active;
    case interrupt_execution_source_t::OCXO2:
      return g_interrupt_priority_runtime.qtimer3.active;
    case interrupt_execution_source_t::PPS:
      return g_interrupt_priority_runtime.pps.active;
    case interrupt_execution_source_t::CONTINUATION:
      return g_interrupt_priority_runtime.continuation.active;
    default:
      return false;
  }
}

// Publish a predecessor only when the target was still pending at the
// predecessor's exit and the predecessor's priority could actually have blocked
// it.  Sequence-guarded publication keeps the lower-priority QTimer1 reader
// coherent if another science ISR arrives while it is copying the latch.
static void interrupt_delay_publish_blocker(
    interrupt_execution_source_t blocker,
    interrupt_execution_source_t target,
    uint32_t exit_dwt,
    uint32_t blocker_wall_cycles,
    uint32_t pending_mask_at_blocker_entry,
    uint32_t pending_mask_at_blocker_exit) {
  const uint32_t target_bit = interrupt_delay_source_bit(target);
  if (target_bit == 0U ||
      (pending_mask_at_blocker_exit & target_bit) == 0U) {
    return;
  }
  if (blocker == target) return;
  if (interrupt_delay_source_priority(blocker) >
      interrupt_delay_source_priority(target)) {
    return;
  }

  // A target already active beneath a higher-priority blocker was preempted,
  // not delayed before entry.  Its pending bit may represent another event on
  // the same vector; do not let that future event consume a stale predecessor
  // latch and claim that the current active invocation was serialized.
  if (interrupt_delay_source_active(target)) return;

  interrupt_delay_blocker_latch_t* latch = interrupt_delay_latch_for(target);
  if (!latch) return;
  uint32_t token = latch->publish_token + 1U;
  if (token == 0U) token = 1U;
  latch->seq++;
  dmb_barrier();
  latch->blocker_exit_dwt = exit_dwt;
  latch->blocker_wall_cycles = blocker_wall_cycles;
  latch->target_pending_at_blocker_entry =
      (pending_mask_at_blocker_entry & target_bit) != 0U;
  latch->blocker = blocker;
  latch->publish_token = token;
  dmb_barrier();
  latch->seq++;
}

static interrupt_arrival_capture_t interrupt_delay_capture_entry_fast(
    interrupt_execution_source_t source) {
  // Assign every scalar explicitly: this path is the first handful of
  // instructions in the ISR and must not invite compiler-generated memset.
  interrupt_arrival_capture_t capture;
  capture.spinidle_running = g_timepop_idle_witness_running;
  capture.spinidle_shadow_dwt = g_timepop_idle_witness_shadow_dwt;
  capture.pending_mask_at_entry = interrupt_delay_pending_mask();
  capture.blocker = interrupt_execution_source_t::NONE;
  capture.blocker_exit_dwt = 0U;
  capture.blocker_wall_cycles = 0U;
  capture.target_pending_at_blocker_entry = false;
  capture.lower_context_active_mask =
      interrupt_delay_lower_active_mask(source);

  interrupt_delay_blocker_latch_t* latch = interrupt_delay_latch_for(source);
  uint32_t* seen_token = interrupt_delay_seen_token_for(source);
  if (latch && seen_token) {
    for (uint32_t attempt = 0U; attempt < 3U; ++attempt) {
      const uint32_t seq1 = latch->seq;
      if ((seq1 & 1U) != 0U) continue;
      dmb_barrier();
      const uint32_t token = latch->publish_token;
      const interrupt_execution_source_t blocker = latch->blocker;
      const uint32_t blocker_exit_dwt = latch->blocker_exit_dwt;
      const uint32_t blocker_wall_cycles = latch->blocker_wall_cycles;
      const bool target_pending_at_blocker_entry =
          latch->target_pending_at_blocker_entry;
      dmb_barrier();
      const uint32_t seq2 = latch->seq;
      if (seq1 == seq2 && (seq2 & 1U) == 0U) {
        // A token is consumed exactly once.  This prevents a years-old latch
        // from becoming spuriously "recent" when 32-bit DWT later wraps.
        if (token != 0U && token != *seen_token) {
          *seen_token = token;
          capture.blocker = blocker;
          capture.blocker_exit_dwt =
              blocker == interrupt_execution_source_t::NONE
                  ? 0U
                  : blocker_exit_dwt;
          capture.blocker_wall_cycles = blocker_wall_cycles;
          capture.target_pending_at_blocker_entry =
              target_pending_at_blocker_entry;
        }
        break;
      }
    }
  }
  return capture;
}

static void interrupt_delay_note_isr_exit(
    interrupt_execution_source_t source,
    uint32_t entry_dwt,
    const interrupt_arrival_capture_t& arrival) {
  // Read pending as late as practical.  A target already pending when this ISR
  // entered waited for the full blocker and may use SpinIdle as a fine ruler.
  // A target that appeared during the blocker is still attributed correctly,
  // but compare-service evidence remains the duration authority.
  const uint32_t pending_mask_at_exit = interrupt_delay_pending_mask();
  const uint32_t exit_dwt = ARM_DWT_CYCCNT;
  const uint32_t blocker_wall_cycles = exit_dwt - entry_dwt;

  interrupt_delay_publish_blocker(
      source, interrupt_execution_source_t::QTIMER1,
      exit_dwt, blocker_wall_cycles,
      arrival.pending_mask_at_entry, pending_mask_at_exit);
  interrupt_delay_publish_blocker(
      source, interrupt_execution_source_t::OCXO1,
      exit_dwt, blocker_wall_cycles,
      arrival.pending_mask_at_entry, pending_mask_at_exit);
  interrupt_delay_publish_blocker(
      source, interrupt_execution_source_t::OCXO2,
      exit_dwt, blocker_wall_cycles,
      arrival.pending_mask_at_entry, pending_mask_at_exit);
  interrupt_delay_publish_blocker(
      source, interrupt_execution_source_t::PPS,
      exit_dwt, blocker_wall_cycles,
      arrival.pending_mask_at_entry, pending_mask_at_exit);
}

static void interrupt_delay_learn_spin_baseline(
    interrupt_delay_baseline_runtime_t& runtime,
    bool shadow_valid,
    uint32_t spinidle_age_cycles) {
  if (!shadow_valid ||
      spinidle_age_cycles > INTERRUPT_DELAY_SPIN_CANDIDATE_MAX_CYCLES) {
    return;
  }

  if (runtime.spin_sample_count == 0U ||
      spinidle_age_cycles < runtime.spin_baseline_cycles) {
    runtime.spin_baseline_cycles = spinidle_age_cycles;
  }
  if (runtime.spin_sample_count != UINT32_MAX) runtime.spin_sample_count++;
  runtime.spin_valid = runtime.spin_sample_count >=
      INTERRUPT_DELAY_SPIN_BASELINE_MIN_SAMPLES;
}

static void interrupt_delay_learn_service_baseline(
    interrupt_delay_baseline_runtime_t& runtime,
    int32_t service_offset_ticks) {
  if (service_offset_ticks < INTERRUPT_DELAY_SERVICE_CANDIDATE_MIN_TICKS ||
      service_offset_ticks > INTERRUPT_DELAY_SERVICE_CANDIDATE_MAX_TICKS) {
    return;
  }

  if (runtime.service_sample_count == 0U ||
      service_offset_ticks < runtime.service_baseline_ticks) {
    runtime.service_baseline_ticks = service_offset_ticks;
  }
  if (runtime.service_sample_count != UINT32_MAX) runtime.service_sample_count++;
  runtime.service_valid = runtime.service_sample_count >=
      INTERRUPT_DELAY_SERVICE_BASELINE_MIN_SAMPLES;
}

static interrupt_arrival_observation_t interrupt_delay_classify_entry(
    interrupt_execution_source_t source,
    uint32_t entry_dwt,
    const interrupt_arrival_capture_t& capture) {
  interrupt_arrival_observation_t out{};
  out.capture = capture;

  // A nonzero shadow is meaningful only when the interrupted thread was still
  // inside SpinIdle.  The running bit deliberately remains true across ISR
  // execution and exception tail-chaining, then falls only after thread mode
  // resumes.  This prevents an old foreground shadow from masquerading as an
  // exact delay ruler.
  out.spinidle_shadow_valid = capture.spinidle_running &&
      capture.spinidle_shadow_dwt != 0U;
  out.spinidle_age_cycles = out.spinidle_shadow_valid
      ? (uint32_t)(entry_dwt - capture.spinidle_shadow_dwt)
      : 0U;

  interrupt_delay_baseline_runtime_t* runtime =
      interrupt_delay_baseline_for(source);
  if (!runtime) return out;

  const uint32_t tail_gap =
      capture.blocker != interrupt_execution_source_t::NONE
          ? (uint32_t)(entry_dwt - capture.blocker_exit_dwt)
          : UINT32_MAX;
  const bool blocker_recent =
      capture.blocker != interrupt_execution_source_t::NONE &&
      tail_gap <= INTERRUPT_DELAY_TAILCHAIN_MAX_GAP_CYCLES;
  const bool blocker_could_block =
      capture.blocker != interrupt_execution_source_t::NONE &&
      interrupt_delay_source_priority(capture.blocker) <=
          interrupt_delay_source_priority(source);

  interrupt_delay_learn_spin_baseline(
      *runtime, out.spinidle_shadow_valid, out.spinidle_age_cycles);

  if (runtime->spin_valid && out.spinidle_shadow_valid) {
    out.spinidle_valid_threshold_cycles =
        runtime->spin_baseline_cycles + INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
    out.spinidle_excess_cycles = out.spinidle_age_cycles >
            runtime->spin_baseline_cycles
        ? out.spinidle_age_cycles - runtime->spin_baseline_cycles
        : 0U;

    if (out.spinidle_excess_cycles <= INTERRUPT_DELAY_SPIN_GUARD_CYCLES) {
      out.delay.valid = true;
      out.delay.verdict = interrupt_delay_verdict_t::ON_TIME;
      out.delay.delayed = false;
      out.delay.delayed_by = interrupt_delay_cause_t::NONE;
      out.delay.confidence = interrupt_delay_confidence_t::HIGH_CONFIDENCE;
      out.delay.delay_cycles_valid = true;
      out.delay.delay_cycles = 0U;
      out.delay.uncertainty_cycles = INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
      return out;
    }

    // Only a target that was already pending when the blocker entered waited
    // for the blocker's full residence.  In that case the SpinIdle excess is a
    // fine-grained estimate of the endpoint displacement.  If the target became
    // pending later, SpinIdle still proves non-idle ancestry but overstates its
    // actual wait; the QTimer classifier below must quantify that case.
    if (blocker_recent && blocker_could_block &&
        capture.target_pending_at_blocker_entry) {
      out.delay.valid = true;
      out.delay.verdict = interrupt_delay_verdict_t::DELAYED;
      out.delay.delayed = true;
      out.delay.delayed_by =
          interrupt_delay_cause_from_source(capture.blocker);
      out.delay.confidence = interrupt_delay_confidence_t::HIGH_CONFIDENCE;
      out.delay.delay_cycles_valid = true;
      out.delay.delay_cycles = out.spinidle_excess_cycles;
      out.delay.uncertainty_cycles = INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
      return out;
    }
  }

  // Pending-at-exit proves this endpoint was serialized behind the named ISR,
  // even when the request arrived partway through the blocker and SpinIdle
  // cannot tell us the exact wait.  Preserve that verdict and cause; compare-
  // driven lanes will add a quantified duration below.
  if (blocker_recent && blocker_could_block) {
    out.delay.valid = true;
    out.delay.verdict = interrupt_delay_verdict_t::DELAYED;
    out.delay.delayed = true;
    out.delay.delayed_by =
        interrupt_delay_cause_from_source(capture.blocker);
    out.delay.confidence = interrupt_delay_confidence_t::LOW_CONFIDENCE;
    out.delay.delay_cycles_valid = false;
    out.delay.delay_cycles = 0U;
    out.delay.uncertainty_cycles = capture.blocker_wall_cycles;
    return out;
  }

  // A higher-priority source entering while a lower tier remains active did not
  // wait for that lower tier; it preempted it.  A stale/missing SpinIdle witness
  // in this case describes prior CPU occupancy, not service delay.
  if (capture.lower_context_active_mask != 0U) {
    out.delay.valid = true;
    out.delay.verdict = interrupt_delay_verdict_t::ON_TIME;
    out.delay.delayed = false;
    out.delay.delayed_by = interrupt_delay_cause_t::NONE;
    out.delay.confidence = interrupt_delay_confidence_t::HIGH_CONFIDENCE;
    out.delay.delay_cycles_valid = true;
    out.delay.delay_cycles = 0U;
    out.delay.uncertainty_cycles = INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
  }

  return out;
}

static uint32_t interrupt_delay_ticks_to_cycles(uint32_t ticks) {
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  if (ticks == 0U || cps == 0U) return 0U;
  return (uint32_t)(((uint64_t)ticks * (uint64_t)cps +
                     (uint64_t)OCXO_ONE_SECOND_TICKS / 2ULL) /
                    (uint64_t)OCXO_ONE_SECOND_TICKS);
}

static interrupt_arrival_observation_t interrupt_delay_classify_qtimer(
    interrupt_execution_source_t source,
    uint32_t entry_dwt,
    const interrupt_arrival_capture_t& capture,
    int32_t service_offset_ticks,
    bool service_witness_contaminated = false) {
  interrupt_arrival_observation_t out =
      interrupt_delay_classify_entry(source, entry_dwt, capture);
  interrupt_delay_baseline_runtime_t* runtime =
      interrupt_delay_baseline_for(source);
  if (!runtime) return out;

  // A higher-priority ISR may preempt QTimer1 after the first-instruction DWT
  // was already captured but before CNTR is read.  That makes the service-offset
  // witness late without making the endpoint late.  Preserve the entry verdict
  // and refuse both learning and adjudication from that contaminated witness.
  if (service_witness_contaminated) return out;

  // Learn the ordinary counter-vs-compare offset only from an independently
  // clean SpinIdle verdict.  A delayed sample must never teach the delay ruler.
  if (out.delay.valid &&
      out.delay.verdict == interrupt_delay_verdict_t::ON_TIME &&
      out.spinidle_shadow_valid &&
      out.spinidle_excess_cycles <= INTERRUPT_DELAY_SPIN_GUARD_CYCLES) {
    interrupt_delay_learn_service_baseline(*runtime, service_offset_ticks);
  }
  if (!runtime->service_valid) return out;

  const int32_t service_extra_ticks =
      service_offset_ticks - runtime->service_baseline_ticks;
  const uint32_t one_tick_cycles = interrupt_delay_ticks_to_cycles(1U);

  if (service_extra_ticks <= INTERRUPT_DELAY_SERVICE_NORMAL_JITTER_TICKS) {
    // The hardware compare was serviced at its learned ordinary offset.  This
    // outranks a long SpinIdle ancestry interval: the target may have become
    // pending only at the blocker's final instruction and did not suffer a
    // measurable endpoint displacement.
    out.delay.valid = true;
    out.delay.verdict = interrupt_delay_verdict_t::ON_TIME;
    out.delay.delayed = false;
    out.delay.delayed_by = interrupt_delay_cause_t::NONE;
    out.delay.confidence = interrupt_delay_confidence_t::MEDIUM_CONFIDENCE;
    out.delay.delay_cycles_valid = true;
    out.delay.delay_cycles = 0U;
    out.delay.uncertainty_cycles = one_tick_cycles;
    return out;
  }

  const uint32_t service_delay_cycles =
      interrupt_delay_ticks_to_cycles((uint32_t)service_extra_ticks);
  const uint32_t agreement_gate = one_tick_cycles +
      INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
  uint32_t selected_delay = service_delay_cycles;
  uint32_t selected_uncertainty = agreement_gate;

  // SpinIdle supplies sub-QTimer-tick precision only when the target was already
  // pending at blocker entry (so the full blocker residence belongs to its wait)
  // and the independent timer ruler agrees within one tick.
  if (capture.target_pending_at_blocker_entry &&
      out.spinidle_excess_cycles > INTERRUPT_DELAY_SPIN_GUARD_CYCLES) {
    const uint32_t difference =
        out.spinidle_excess_cycles >= service_delay_cycles
            ? out.spinidle_excess_cycles - service_delay_cycles
            : service_delay_cycles - out.spinidle_excess_cycles;
    if (difference <= agreement_gate) {
      selected_delay = out.spinidle_excess_cycles;
      selected_uncertainty = INTERRUPT_DELAY_SPIN_GUARD_CYCLES;
    }
  }

  out.delay.valid = true;
  out.delay.verdict = interrupt_delay_verdict_t::DELAYED;
  out.delay.delayed = true;
  if (out.delay.delayed_by == interrupt_delay_cause_t::NONE ||
      out.delay.delayed_by == interrupt_delay_cause_t::UNKNOWN) {
    out.delay.delayed_by = interrupt_delay_cause_t::MASKING_OR_UNKNOWN_CPU;
  }
  if ((uint8_t)out.delay.confidence <
      (uint8_t)interrupt_delay_confidence_t::MEDIUM_CONFIDENCE) {
    out.delay.confidence = interrupt_delay_confidence_t::MEDIUM_CONFIDENCE;
  }
  out.delay.delay_cycles_valid = true;
  out.delay.delay_cycles = selected_delay;
  out.delay.uncertainty_cycles = selected_uncertainty;
  return out;
}

static int32_t interrupt_delay_signed_difference(uint32_t current,
                                                 uint32_t previous) {
  const int64_t value = (int64_t)(uint64_t)current -
                        (int64_t)(uint64_t)previous;
  if (value > INT32_MAX) return INT32_MAX;
  if (value < INT32_MIN) return INT32_MIN;
  return (int32_t)value;
}

static interrupt_delay_cause_t interrupt_delay_combine_causes(
    interrupt_delay_cause_t a,
    interrupt_delay_cause_t b) {
  if (a == interrupt_delay_cause_t::NONE) return b;
  if (b == interrupt_delay_cause_t::NONE) return a;
  if (a == b) return a;
  if (a == interrupt_delay_cause_t::UNKNOWN ||
      b == interrupt_delay_cause_t::UNKNOWN) {
    return interrupt_delay_cause_t::UNKNOWN;
  }
  return interrupt_delay_cause_t::MULTIPLE_ISR;
}

static void interrupt_delay_attach_interval_history(
    bool& previous_valid,
    bool& previous_delayed,
    interrupt_delay_cause_t& previous_cause,
    uint32_t& previous_cycles,
    bool& previous_interval_valid,
    int32_t& previous_interval_cycles,
    interrupt_delay_cause_t& previous_interval_cause,
    const interrupt_delay_forensics_t& endpoint,
    interrupt_delay_forensics_t& out) {
  out = endpoint;
  out.previous_endpoint_valid = previous_valid;
  out.previous_endpoint_delayed = previous_valid && previous_delayed;
  out.previous_delayed_by = previous_valid
      ? previous_cause
      : interrupt_delay_cause_t::NONE;
  out.previous_delay_cycles = previous_valid ? previous_cycles : 0U;

  out.interval_delay_valid = endpoint.valid &&
      endpoint.delay_cycles_valid && previous_valid;
  out.interval_delay_cycles = out.interval_delay_valid
      ? interrupt_delay_signed_difference(endpoint.delay_cycles,
                                          previous_cycles)
      : 0;
  if (!out.interval_delay_valid || out.interval_delay_cycles == 0) {
    out.interval_delayed_by = interrupt_delay_cause_t::NONE;
  } else if (out.interval_delay_cycles > 0) {
    out.interval_delayed_by = endpoint.delayed_by;
  } else {
    out.interval_delayed_by = previous_cause;
  }

  out.previous_interval_delay_valid = previous_interval_valid;
  out.previous_interval_delay_cycles = previous_interval_valid
      ? previous_interval_cycles
      : 0;
  out.previous_interval_delayed_by = previous_interval_valid
      ? previous_interval_cause
      : interrupt_delay_cause_t::NONE;
  out.residual_delay_valid = out.interval_delay_valid &&
      previous_interval_valid;
  out.residual_delay_cycles = 0;
  if (out.residual_delay_valid) {
    const int64_t residual64 = (int64_t)out.interval_delay_cycles -
                               (int64_t)previous_interval_cycles;
    out.residual_delay_cycles = residual64 > INT32_MAX
        ? INT32_MAX
        : (residual64 < INT32_MIN ? INT32_MIN : (int32_t)residual64);
  }

  if (!out.residual_delay_valid || out.residual_delay_cycles == 0) {
    out.residual_delayed_by = interrupt_delay_cause_t::NONE;
  } else if (out.interval_delay_cycles == 0) {
    out.residual_delayed_by = previous_interval_cause;
  } else if (previous_interval_cycles == 0) {
    out.residual_delayed_by = out.interval_delayed_by;
  } else {
    out.residual_delayed_by = interrupt_delay_combine_causes(
        out.interval_delayed_by, previous_interval_cause);
  }

  previous_valid = endpoint.valid && endpoint.delay_cycles_valid;
  previous_delayed = endpoint.valid && endpoint.delayed;
  previous_cause = endpoint.valid
      ? endpoint.delayed_by
      : interrupt_delay_cause_t::UNKNOWN;
  previous_cycles = endpoint.valid && endpoint.delay_cycles_valid
      ? endpoint.delay_cycles
      : 0U;

  previous_interval_valid = out.interval_delay_valid;
  previous_interval_cycles = out.interval_delay_valid
      ? out.interval_delay_cycles
      : 0;
  previous_interval_cause = out.interval_delay_valid
      ? out.interval_delayed_by
      : interrupt_delay_cause_t::UNKNOWN;
}

static bool interrupt_priority_readback_all_match(void) {
  bool match =
      interrupt_nvic_priority_read((uint32_t)IRQ_QTIMER1) ==
          INTERRUPT_PRIORITY_VCLOCK_TIMEPOP &&
      interrupt_nvic_priority_read((uint32_t)IRQ_GPIO6789) ==
          INTERRUPT_PRIORITY_SCIENCE &&
      interrupt_nvic_priority_read(INTERRUPT_HANDOFF_IRQ_NUMBER) ==
          INTERRUPT_PRIORITY_CONTINUATION;
  if (!OCXO1_DISABLED) {
    match = match &&
        interrupt_nvic_priority_read((uint32_t)IRQ_QTIMER2) ==
            INTERRUPT_PRIORITY_SCIENCE;
  }
  if (!OCXO2_DISABLED) {
    match = match &&
        interrupt_nvic_priority_read((uint32_t)IRQ_QTIMER3) ==
            INTERRUPT_PRIORITY_SCIENCE;
  }
  return match;
}

static void interrupt_priority_verify_live(void) {
  g_interrupt_priority_runtime.verify_count++;
  const bool match = interrupt_priority_readback_all_match();
  g_interrupt_priority_runtime.last_verify_all_match = match;
  if (!match) {
    g_interrupt_priority_runtime.verify_mismatch_count++;
  }
}

template <typename T, uint32_t N>
static uint32_t capture_ring_pending(
    const interrupt_capture_ring_t<T, N>& ring) {
  const uint32_t tail = ring.tail;
  dmb_barrier();
  return ring.head - tail;
}

template <typename T, uint32_t N>
static bool capture_ring_push_isr(
    interrupt_capture_ring_t<T, N>& ring,
    interrupt_handoff_source_diag_t& diag,
    T packet) {
  const uint32_t head = ring.head;
  dmb_barrier();
  if (head - ring.tail >= N) {
    // The overrun count is the one anomaly fact that cannot be reconstructed
    // after a packet fails to enter custody.  Recovery itself runs at Priority 32.
    diag.overrun_count++;
    return false;
  }
  packet.sequence = ++g_interrupt_capture_sequence;
  ring.items[head % N] = packet;
  dmb_barrier();
  ring.head = head + 1U;
  return true;
}

template <typename T, uint32_t N>
static bool capture_ring_pop_handoff(
    interrupt_capture_ring_t<T, N>& ring,
    interrupt_handoff_source_diag_t& diag,
    T& packet) {
  const uint32_t tail = ring.tail;
  const uint32_t head = ring.head;
  if (tail == head) return false;
  const uint32_t pending_before = head - tail;
  dmb_barrier();
  packet = ring.items[tail % N];
  dmb_barrier();
  ring.tail = tail + 1U;

  // Routine queue accounting belongs to continuation, never sacred capture.
  diag.capture_count++;
  diag.enqueue_count++;
  diag.dequeue_count++;
  diag.pending_count = pending_before - 1U;
  diag.last_sequence = packet.sequence;
  diag.last_capture_dwt = packet.isr_entry_dwt_raw;
  if (pending_before > diag.high_water) diag.high_water = pending_before;
  return true;
}

template <typename T, uint32_t N>
static bool capture_ring_peek_sequence(
    const interrupt_capture_ring_t<T, N>& ring,
    uint32_t& sequence) {
  const uint32_t tail = ring.tail;
  const uint32_t head = ring.head;
  if (tail == head) return false;
  dmb_barrier();
  sequence = ring.items[tail % N].sequence;
  return true;
}

static bool timepop_foreground_mailbox_store_handoff(
    const timepop_foreground_packet_t& packet) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  if (g_timepop_foreground_mailbox_pending) {
    // CH2 is one-shot while a fire fact is under custody.  A second fact is not
    // queue pressure; it is a broken hardware-custody invariant.
    g_timepop_foreground_diag.overrun_count++;
    g_timepop_foreground_rearm_requested = true;
    g_process_interrupt_foreground_pending = true;
    interrupt_priority0_guard_exit(prior);
    return false;
  }
  g_timepop_foreground_mailbox = packet;
  dmb_barrier();
  g_timepop_foreground_mailbox_pending = true;
  g_timepop_foreground_diag.enqueue_count++;
  g_timepop_foreground_diag.high_water = 1U;
  g_process_interrupt_foreground_pending = true;
  interrupt_priority0_guard_exit(prior);
  return true;
}

static bool timepop_foreground_mailbox_take(
    timepop_foreground_packet_t& packet) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  if (!g_timepop_foreground_mailbox_pending) {
    interrupt_priority0_guard_exit(prior);
    return false;
  }
  dmb_barrier();
  packet = g_timepop_foreground_mailbox;
  g_timepop_foreground_mailbox = timepop_foreground_packet_t{};
  dmb_barrier();
  g_timepop_foreground_mailbox_pending = false;
  g_timepop_foreground_diag.dequeue_count++;
  interrupt_priority0_guard_exit(prior);
  return true;
}

static bool handoff_any_pending(void) {
  return capture_ring_pending(g_vclock_capture_ring) != 0U ||
         capture_ring_pending(g_ch2_capture_ring) != 0U ||
         capture_ring_pending(g_ocxo1_capture_ring) != 0U ||
         capture_ring_pending(g_ocxo2_capture_ring) != 0U ||
         capture_ring_pending(g_pps_capture_ring) != 0U;
}

static void interrupt_handoff_request_isr(uint32_t request_dwt) {
  // Preserve the first capture that created this pending continuation.  A
  // same-vector VCLOCK/TimePop pair therefore performs exactly one NVIC pend.
  if (g_interrupt_handoff.pending) return;
  g_interrupt_handoff.last_request_dwt = request_dwt;
  g_interrupt_handoff.pending = true;
  g_interrupt_handoff.request_count++;
  dmb_barrier();
  interrupt_nvic_ispr_word(INTERRUPT_HANDOFF_IRQ_NUMBER) =
      interrupt_nvic_irq_mask(INTERRUPT_HANDOFF_IRQ_NUMBER);
}

static void update_handoff_latency(interrupt_handoff_source_diag_t& diag,
                                   uint32_t capture_dwt) {
  // A Priority 0 science or Priority 16 QTimer1 packet may arrive after the
  // current continuation ISR entered but before it reaches this source.  Measure
  // to the actual dequeue point rather than the older entry timestamp.
  const uint32_t dequeue_dwt = ARM_DWT_CYCCNT;
  const uint32_t latency = dequeue_dwt - capture_dwt;
  if ((int32_t)latency < 0) {
    diag.latency_invalid_count++;
    return;
  }
  diag.last_capture_to_handoff_cycles = latency;
  if (latency > diag.max_capture_to_handoff_cycles) {
    diag.max_capture_to_handoff_cycles = latency;
  }
}

// ============================================================================
// Observed event authorship
// ============================================================================

static void fill_observed_diag(interrupt_capture_diag_t& diag,
                               const interrupt_event_t& event,
                               uint32_t isr_entry_dwt_raw,
                               uint16_t ambient_low16,
                               uint16_t target_low16,
                               uint32_t sequence,
                               const interrupt_arrival_observation_t* arrival =
                                   nullptr) {
  diag.enabled = true;
  diag.provider = event.provider;
  diag.lane = event.lane;
  diag.kind = event.kind;
  diag.dwt_at_event = event.dwt_at_event;
  diag.gnss_ns_at_event = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;
  if (arrival) {
    diag.spinidle_shadow_valid =
        arrival->spinidle_shadow_valid;
    diag.spinidle_shadow_dwt = arrival->capture.spinidle_shadow_dwt;
    diag.spinidle_shadow_to_isr_entry_cycles =
        arrival->spinidle_age_cycles;
    diag.spinidle_shadow_valid_threshold_cycles =
        arrival->spinidle_valid_threshold_cycles;
    diag.interrupt_delay = arrival->delay;
  } else {
    diag.spinidle_shadow_valid = false;
    diag.spinidle_shadow_dwt = 0U;
    diag.spinidle_shadow_to_isr_entry_cycles = 0U;
    diag.spinidle_shadow_valid_threshold_cycles = 0U;
    diag.interrupt_delay = interrupt_delay_forensics_t{};
  }
  diag.dwt_synthetic = false;
  diag.dwt_repair_candidate = false;
  diag.dwt_original_at_event = event.dwt_at_event;
  diag.dwt_predicted_at_event = 0U;
  diag.dwt_used_at_event = event.dwt_at_event;
  diag.dwt_isr_entry_raw = isr_entry_dwt_raw;
  diag.dwt_event_from_isr_entry_raw = event.dwt_at_event;
  diag.dwt_isr_entry_to_event_correction_cycles =
      (int32_t)(event.dwt_at_event - isr_entry_dwt_raw);
  diag.dwt_published_minus_event_cycles = 0;
  diag.dwt_used_minus_event_cycles = 0;
  diag.dwt_synthetic_error_cycles = 0;
  diag.dwt_synthetic_threshold_cycles = 0U;
  diag.dwt_synthetic_reason = "observed";
  diag.dwt_publication_verdict_mask =
      INTERRUPT_DWT_PUBLICATION_VERDICT_OK;
  diag.dwt_publication_verdict_reason_id =
      INTERRUPT_DWT_PUBLICATION_REASON_OK;

  // Compatibility ABI fields below carry observed/target facts only.  No
  // alternative estimator is computed or selected.
  diag.ocxo_perishable_fact_sequence = sequence;
  diag.ocxo_service_corrected_dwt_at_event = event.dwt_at_event;
  diag.ocxo_boundary_dwt_at_event = event.dwt_at_event;
  diag.ocxo_boundary_counter32_at_event = event.counter32_at_event;
  diag.ocxo_boundary_correction_cycles = 0;
  diag.ocxo_isr_counter_low16 = ambient_low16;
  diag.ocxo_isr_compare_low16 = target_low16;
  diag.ocxo_compare_delta_mod65536_ticks =
      (uint32_t)((uint16_t)(ambient_low16 - target_low16));
  diag.ocxo_compare_service_offset_signed_ticks =
      (int32_t)((int16_t)(uint16_t)(ambient_low16 - target_low16));
  diag.ocxo_compare_interpreted_late_ticks =
      diag.ocxo_compare_service_offset_signed_ticks >= 0
          ? (uint32_t)diag.ocxo_compare_service_offset_signed_ticks
          : 0U;
  diag.ocxo_compare_early_ticks =
      diag.ocxo_compare_service_offset_signed_ticks < 0
          ? (uint32_t)(-(int64_t)diag.ocxo_compare_service_offset_signed_ticks)
          : 0U;
  diag.ocxo_service_offset_signed_ticks =
      diag.ocxo_compare_service_offset_signed_ticks;
  diag.ocxo_service_offset_abs_ticks =
      interrupt_abs_i32(diag.ocxo_service_offset_signed_ticks);
  diag.ocxo_interpreted_late_ticks =
      diag.ocxo_compare_interpreted_late_ticks;
  diag.ocxo_early_ticks = diag.ocxo_compare_early_ticks;
  diag.ocxo_target_delta_mod65536_ticks =
      diag.ocxo_compare_delta_mod65536_ticks;
  diag.ocxo_sample_phase_valid = false;
  diag.ocxo_sample_period_ticks = OCXO_COMPARE_PERIOD_TICKS;
  diag.ocxo_sample_dwt_at_event = event.dwt_at_event;
  diag.ocxo_sample_counter32_at_event = event.counter32_at_event;
}

static bool emit_observed_event(interrupt_subscriber_runtime_t& rt,
                                uint32_t dwt_at_event,
                                uint32_t counter32_at_event,
                                uint32_t isr_entry_dwt_raw,
                                uint16_t ambient_low16,
                                uint16_t target_low16,
                                uint32_t sequence,
                                uint32_t pps_sequence,
                                const interrupt_arrival_observation_t& arrival) {
  if (!rt.active || !rt.desc || dwt_at_event == 0U) return false;

  const uint32_t prior = interrupt_priority0_guard_enter();
  const bool busy = rt.dispatch_running || rt.deferred.pending;
  if (busy) rt.dispatch_busy_drop_count++;
  interrupt_priority0_guard_exit(prior);
  if (busy) return false;

  rt.last_event = interrupt_event_t{};
  rt.last_event.kind = rt.desc->kind;
  rt.last_event.provider = rt.desc->provider;
  rt.last_event.lane = rt.desc->lane;
  rt.last_event.status = interrupt_event_status_t::OK;
  rt.last_event.dwt_at_event = dwt_at_event;
  rt.last_event.counter32_at_event = counter32_at_event;
  rt.last_event.pps_sequence = pps_sequence;
  rt.last_event.gnss_ns_at_event = 0U;

  fill_observed_diag(rt.last_diag,
                     rt.last_event,
                     isr_entry_dwt_raw,
                     ambient_low16,
                     target_low16,
                     sequence,
                     &arrival);
  interrupt_delay_attach_interval_history(
      rt.previous_delay_valid,
      rt.previous_endpoint_delayed,
      rt.previous_delayed_by,
      rt.previous_delay_cycles,
      rt.previous_interval_delay_valid,
      rt.previous_interval_delay_cycles,
      rt.previous_interval_delayed_by,
      arrival.delay,
      rt.last_diag.interrupt_delay);
  rt.has_fired = true;
  rt.event_count++;
  return interrupt_dispatch_begin(rt, rt.last_event);
}

static void publish_observed_pps_vclock(uint32_t sequence,
                                        uint32_t dwt_at_edge,
                                        uint32_t counter32_at_edge,
                                        uint16_t target_low16) {
  pps_t pps = g_last_pps_witness_valid
      ? g_last_pps_witness
      : pps_t{};
  if (pps.sequence == 0U) {
    pps.sequence = sequence;
    pps.dwt_at_edge = dwt_at_edge;
    pps.counter32_at_edge = counter32_at_edge;
    pps.ch3_at_edge = target_low16;
  }

  pps_vclock_t pvc{};
  pvc.sequence = sequence;
  pvc.dwt_at_edge = dwt_at_edge;
  pvc.counter32_at_edge = counter32_at_edge;
  pvc.ch3_at_edge = target_low16;
  pvc.gnss_ns_at_edge = -1;

  pps_vclock_edge_authority_t authority{};
  authority.valid = dwt_at_edge != 0U;
  authority.sequence = sequence;
  authority.update_count = g_pps_vclock_edge_authority.update_count + 1U;
  authority.authority_dwt_at_edge = dwt_at_edge;
  authority.pps_dwt_at_edge = pps.dwt_at_edge;
  authority.vclock_observed_dwt_at_edge = dwt_at_edge;
  authority.counter32_at_edge = counter32_at_edge;
  authority.ch3_at_edge = target_low16;
  authority.dwt_cycles_per_second = interrupt_vclock_cycles_per_second();
  authority.decision = PPS_VCLOCK_EDGE_DECISION_OBSERVED_FALLBACK;
  authority.invalid_mask = authority.valid
      ? PPS_VCLOCK_EDGE_INVALID_NONE
      : PPS_VCLOCK_EDGE_INVALID_NO_OBSERVED;
  authority.authority_minus_pps_cycles = pps.dwt_at_edge != 0U
      ? (int32_t)(dwt_at_edge - pps.dwt_at_edge)
      : 0;
  g_pps_vclock_edge_authority = authority;

  store_publish(pps, pvc);
  g_pps_gpio_heartbeat.last_dwt = dwt_at_edge;
  g_pps_gpio_heartbeat.last_gnss_ns = -1;
}

static void pps_relay_tick_from_vclock(void) {
  if (!g_pps_relay_active || g_pps_relay_countdown_cells == 0U) return;
  g_pps_relay_countdown_cells--;
  if (g_pps_relay_countdown_cells == 0U) {
    digitalWriteFast(GNSS_PPS_RELAY, LOW);
    g_pps_relay_active = false;
    g_pps_relay_deassert_count++;
  }
}

// ============================================================================
// Priority-32 continuation packet processing
// ============================================================================

static void process_vclock_packet(const vclock_capture_packet_t& packet) {
  crash_stack_tripwire_isr_check(CRASH_TRIPWIRE_SITE_QTIMER1);
  const uint32_t captured_target = packet.target_counter32;
  const uint16_t target_low16 = packet.target_low16;
  const uint32_t service_counter32 =
      vclock_synthetic_from_hardware_low16(packet.service_low16);
  const int32_t service_offset_ticks =
      (int32_t)((int16_t)(uint16_t)(packet.service_low16 - target_low16));
  const interrupt_arrival_observation_t arrival =
      interrupt_delay_classify_qtimer(
          interrupt_execution_source_t::QTIMER1,
          packet.isr_entry_dwt_raw,
          packet.arrival,
          service_offset_ticks,
          packet.preempted_after_entry);
  uint32_t event_target = captured_target;

  vclock_anchor_hardware(captured_target, target_low16);
  g_vclock_lane.heartbeat_count++;
  g_vclock_lane.irq_count++;
  g_vclock_lane.last_isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  const uint32_t dwt_at_edge =
      qtimer_event_dwt_from_isr_entry_raw(packet.isr_entry_dwt_raw);
  g_vclock_lane.last_dwt_at_edge = dwt_at_edge;

  uint32_t next = captured_target + VCLOCK_INTERVAL_COUNTS;
  if (g_vclock_clock32.pending_zero) {
    const uint64_t ns = g_vclock_clock32.pending_zero_ns;
    g_vclock_clock32.pending_zero = false;
    synthetic_clock_zero(g_vclock_clock32, ns);
    g_vclock_clock32.hardware_low16_at_zero = target_low16;
    vclock_anchor_hardware(g_vclock_clock32.zero_counter32, target_low16);
    event_target = g_vclock_clock32.zero_counter32;
    next = event_target + VCLOCK_INTERVAL_COUNTS;
    g_vclock_lane.one_second_grid_valid = false;
  } else {
    while ((int32_t)(service_counter32 - next) >= 0) {
      next += VCLOCK_INTERVAL_COUNTS;
      g_vclock_lane.miss_count++;
    }
  }

  const uint16_t next_low16 =
      vclock_hardware_low16_from_synthetic(next);
  g_vclock_heartbeat_next_counter32 = next;
  g_vclock_heartbeat_target_low16 = next_low16;
  qtimer1_vclock_program_compare(next_low16);
  g_vclock_lane.last_target_counter32 = event_target;

  // VCLOCK owns all periodic low-word extension and OCXO arm tending.
  if (!OCXO1_DISABLED) ocxo_tend_and_arm(g_ocxo1_ctx);
  if (!OCXO2_DISABLED) ocxo_tend_and_arm(g_ocxo2_ctx);
  pps_relay_tick_from_vclock();

  if (!g_vclock_lane.one_second_grid_valid &&
      g_last_pps_witness_valid) {
    g_vclock_lane.next_one_second_counter32 = event_target;
    g_vclock_lane.one_second_grid_valid = true;
  }

  if (g_vclock_lane.one_second_grid_valid &&
      event_target == g_vclock_lane.next_one_second_counter32) {
    const uint32_t sequence = g_last_pps_witness_valid
        ? g_last_pps_witness.sequence
        : (g_vclock_lane.one_second_count + 1U);
    g_vclock_lane.one_second_count++;
    g_vclock_lane.next_one_second_counter32 += OCXO_ONE_SECOND_TICKS;
    publish_observed_pps_vclock(sequence,
                                dwt_at_edge,
                                event_target,
                                target_low16);
    interrupt_features_note_observed_edge();
    integrity_note_dwt_interval(interrupt_subscriber_kind_t::VCLOCK,
                                sequence,
                                event_target,
                                dwt_at_edge);
    smartzero_feed_one_second(interrupt_subscriber_kind_t::VCLOCK,
                              dwt_at_edge,
                              event_target,
                              target_low16);
    interrupt_features_note_lineage();
    interrupt_subscriber_runtime_t* rt_vclock =
        runtime_for(interrupt_subscriber_kind_t::VCLOCK);
    if (rt_vclock && rt_vclock->active) {
      rt_vclock->irq_count++;
      (void)emit_observed_event(*rt_vclock,
                                dwt_at_edge,
                                event_target,
                                packet.isr_entry_dwt_raw,
                                packet.service_low16,
                                target_low16,
                                sequence,
                                sequence,
                                arrival);
    }
  }
}

static void process_ch2_packet(const ch2_capture_packet_t& packet) {
  crash_stack_tripwire_isr_check(CRASH_TRIPWIRE_SITE_QTIMER1);

  timepop_foreground_packet_t foreground{};
  foreground.sequence = packet.sequence;
  foreground.isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  foreground.dwt_at_event =
      qtimer_event_dwt_from_isr_entry_raw(packet.isr_entry_dwt_raw);
  foreground.target_counter32 = packet.target_counter32;

  // Priority 32 ends at immutable custody transfer.  TimePop scheduler state,
  // callback selection, recurring rearm, and compare selection are foreground.
  (void)timepop_foreground_mailbox_store_handoff(foreground);
  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::HANDOFF_DEQUEUE,
      timepop_dispatch_trace_kind_t::ISR_TIMED,
      QTIMER1_TIMEPOP_CH,
      packet.sequence,
      0U,
      0U,
      packet.target_counter32,
      (uint32_t)(uintptr_t)"QTIMER1_CH2_FOREGROUND",
      foreground.dwt_at_event);
}

static void ocxo_complete_capture_custody(
    ocxo_lane_t& lane,
    const ocxo_capture_packet_t& packet) {
  if (!packet.active_at_capture) return;

  if (!lane.capture_pending) {
    lane.capture_pending_missing_count++;
  } else {
    lane.capture_pending_clear_count++;
  }
  if (lane.capture_pending_target_counter32 != packet.target_counter32) {
    lane.capture_pending_target_mismatch_count++;
  }
  lane.capture_pending_target_counter32 = 0U;
  dmb_barrier();
  lane.capture_pending = false;
  dmb_barrier();
}

static void process_ocxo_packet(const ocxo_runtime_context_t& ctx,
                                const ocxo_capture_packet_t& packet) {
  ocxo_lane_t& lane = *ctx.lane;
  synthetic_clock32_t& clock = *ctx.clock32;
  lane.irq_count++;
  lane.last_isr_entry_dwt_raw = packet.isr_entry_dwt_raw;
  lane.last_ambient_low16 = packet.ambient_low16;
  lane.last_compare_low16 = packet.compare_low16;
  lane.last_ambient_minus_target_ticks =
      (int32_t)((int16_t)(uint16_t)(packet.ambient_low16 -
                                    packet.target_low16));
  const interrupt_execution_source_t delay_source =
      ctx.kind == interrupt_subscriber_kind_t::OCXO1
          ? interrupt_execution_source_t::OCXO1
          : interrupt_execution_source_t::OCXO2;
  const interrupt_arrival_observation_t arrival =
      interrupt_delay_classify_qtimer(
          delay_source,
          packet.isr_entry_dwt_raw,
          packet.arrival,
          lane.last_ambient_minus_target_ticks);

  // The semantic event identity is the target authored before the match.  The
  // ambient read is witness-only and cannot advance, reject, or replace it.
  const uint32_t target = packet.target_counter32;
  if (!packet.active_at_capture || target == 0U) {
    lane.false_irq_count++;
    lane.miss_count++;
    lane.compare_armed = false;
    lane.armed_target_counter32 = 0U;
    lane.armed_target_low16 = 0U;
    ocxo_complete_capture_custody(lane, packet);
    return;
  }

  const uint32_t dwt_at_edge =
      qtimer_event_dwt_from_isr_entry_raw(packet.isr_entry_dwt_raw);
  lane.capture_pending_set_count++;
  lane.last_dwt_at_edge = dwt_at_edge;
  lane.last_event_counter32 = target;
  lane.compare_armed = false;
  lane.armed_target_counter32 = 0U;
  lane.armed_target_low16 = 0U;

  const int32_t target_delta =
      (int32_t)(target - clock.current_counter32);
  lane.last_target_commit_delta_ticks = target_delta;
  lane.last_target_commit_rollback_ticks = 0U;
  if (target_delta >= 0) {
    clock.current_ns += (uint64_t)(uint32_t)target_delta * 100ULL;
  } else {
    // This should remain zero after armed/pending custody is enforced.  Keep the
    // 64-bit clock coherent if another lifecycle path ever advances past a
    // captured authored target, and leave an explicit diagnostic footprint.
    const uint32_t rollback_ticks =
        (uint32_t)(-(int64_t)target_delta);
    const uint64_t rollback_ns = (uint64_t)rollback_ticks * 100ULL;
    lane.target_commit_rollback_count++;
    lane.last_target_commit_rollback_ticks = rollback_ticks;
    if (clock.current_ns >= rollback_ns) {
      clock.current_ns -= rollback_ns;
    } else {
      clock.current_ns = 0U;
      lane.target_commit_ns_underflow_count++;
    }
  }
  clock.current_counter32 = target;
  clock.hardware16 = packet.target_low16;

  lane.next_target_counter32 = target + OCXO_COMPARE_PERIOD_TICKS;
  lane.target_grid_valid = true;

  // The one-second target is now committed in counter and nanosecond custody.
  // Clear in-flight ownership; VCLOCK will tend the next target through the
  // intervening low-word rollovers and arm it only inside the safe window.
  ocxo_complete_capture_custody(lane, packet);
  ocxo_tend_and_arm(ctx);

  crash_stack_tripwire_isr_check(
      ctx.kind == interrupt_subscriber_kind_t::OCXO1
          ? CRASH_TRIPWIRE_SITE_QTIMER2
          : CRASH_TRIPWIRE_SITE_QTIMER3);
  lane.fire_count++;
  lane.dispatch_sequence++;

  const bool interval_valid = lane.previous_event_valid;
  const uint32_t delta = interval_valid
      ? target - lane.previous_event_counter32
      : 0U;
  lane.last_counter_delta_ticks = delta;
  if (interval_valid && delta != OCXO_ONE_SECOND_TICKS) {
    lane.counter_delta_violation_count++;
  }
  lane.previous_event_valid = true;
  lane.previous_event_counter32 = target;

  interrupt_integrity_note_counter32(ctx.kind,
                                     lane.dispatch_sequence,
                                     interval_valid,
                                     delta,
                                     OCXO_ONE_SECOND_TICKS,
                                     target);
  integrity_note_dwt_interval(ctx.kind,
                              lane.dispatch_sequence,
                              target,
                              dwt_at_edge);
  smartzero_feed_one_second(ctx.kind,
                            dwt_at_edge,
                            target,
                            packet.target_low16);
  interrupt_features_note_ocxo_custody();
  interrupt_features_note_lineage();

  interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  if (rt && rt->active && packet.active_at_capture) {
    rt->irq_count++;
    (void)emit_observed_event(*rt,
                              dwt_at_edge,
                              target,
                              packet.isr_entry_dwt_raw,
                              packet.ambient_low16,
                              packet.target_low16,
                              lane.dispatch_sequence,
                              g_last_pps_witness_valid
                                  ? g_last_pps_witness.sequence
                                  : 0U,
                              arrival);
  } else {
    lane.miss_count++;
  }
}

static void process_pps_packet(const pps_capture_packet_t& packet) {
  crash_stack_tripwire_isr_check(CRASH_TRIPWIRE_SITE_PPS_GPIO);
  const interrupt_arrival_observation_t arrival =
      interrupt_delay_classify_entry(
          interrupt_execution_source_t::PPS,
          packet.isr_entry_dwt_raw,
          packet.arrival);
  g_pps_arrival_capture = packet.arrival;
  g_pps_arrival_entry_dwt = packet.isr_entry_dwt_raw;
  interrupt_delay_attach_interval_history(
      g_pps_previous_delay_valid,
      g_pps_previous_endpoint_delayed,
      g_pps_previous_delayed_by,
      g_pps_previous_delay_cycles,
      g_pps_previous_interval_delay_valid,
      g_pps_previous_interval_delay_cycles,
      g_pps_previous_interval_delayed_by,
      arrival.delay,
      g_pps_interrupt_delay);

  pps_t pps{};
  pps.sequence = ++g_pps_gpio_heartbeat.edge_count;
  pps.dwt_at_edge = pps_dwt_from_isr_entry_raw(packet.isr_entry_dwt_raw);
  pps.counter32_at_edge =
      vclock_synthetic_from_hardware_low16(packet.vclock_hardware16);
  pps.ch3_at_edge = packet.vclock_hardware16;
  g_last_pps_witness = pps;
  g_last_pps_witness_valid = true;
  g_gpio_irq_count++;

  interrupt_epoch_capture_t capture{};
  capture.valid = true;
  capture.sequence = pps.sequence;
  capture.capture_dwt_start_raw = packet.isr_entry_dwt_raw;
  capture.capture_dwt_after_vclock_raw = packet.isr_entry_dwt_raw;
  capture.capture_dwt_end_raw = packet.isr_entry_dwt_raw;
  capture.capture_window_cycles = 0U;
  capture.vclock_read_offset_cycles = 0U;
  capture.vclock_dwt_at_edge = pps.dwt_at_edge;
  capture.vclock_capture_valid = true;
  capture.ocxo1_capture_valid = packet.ocxo1_valid;
  capture.ocxo2_capture_valid = packet.ocxo2_valid;
  capture.all_lanes_capture_valid =
      capture.vclock_capture_valid &&
      capture.ocxo1_capture_valid &&
      capture.ocxo2_capture_valid;
  capture.vclock_hardware16_observed = packet.vclock_hardware16;
  capture.vclock_hardware16_selected = packet.vclock_hardware16;
  capture.ocxo1_hardware16 = packet.ocxo1_hardware16;
  capture.ocxo2_hardware16 = packet.ocxo2_hardware16;
  capture.vclock_counter32 = pps.counter32_at_edge;

  // CounterLedger is now a witness only.  Project its ambient reads through the
  // VCLOCK-tended synthetic anchors without mutating event identity.
  if (packet.ocxo1_valid) {
    const uint32_t delta = (uint32_t)((uint16_t)(
        packet.ocxo1_hardware16 - g_ocxo1_clock32.hardware16));
    capture.ocxo1_counter32 = g_ocxo1_clock32.current_counter32 + delta;
  }
  if (packet.ocxo2_valid) {
    const uint32_t delta = (uint32_t)((uint16_t)(
        packet.ocxo2_hardware16 - g_ocxo2_clock32.hardware16));
    capture.ocxo2_counter32 = g_ocxo2_clock32.current_counter32 + delta;
  }
  epoch_capture_publish(capture);

  // The next observed native VCLOCK heartbeat becomes the initial one-second grid
  // after boot/rebootstrap.  No DWT back-projection is performed.
  if (!g_vclock_lane.one_second_grid_valid || g_pps_rebootstrap_pending) {
    g_vclock_lane.next_one_second_counter32 =
        g_vclock_heartbeat_next_counter32;
    g_vclock_lane.one_second_grid_valid = true;
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;
  }

  const interrupt_pps_entry_latency_handler_fn callback =
      g_pps_entry_latency_handler;
  if (interrupt_callback_address_executable((uintptr_t)callback)) {
    callback(pps.sequence, packet.isr_entry_dwt_raw);
  }
}

static void recover_capture_overruns(void) {
  const uint32_t vclock_requests =
      g_vclock_capture_overrun_rearm_request_count;
  if (g_vclock_capture_overrun_rearm_complete_count != vclock_requests) {
    g_vclock_capture_overrun_rearm_complete_count = vclock_requests;
    g_vclock_lane.miss_count++;
    (void)vclock_heartbeat_arm();
  }

  const uint32_t ocxo1_requests =
      g_ocxo1_lane.capture_pending_enqueue_fail_count;
  if (g_ocxo1_capture_overrun_recovery_count != ocxo1_requests) {
    g_ocxo1_capture_overrun_recovery_count = ocxo1_requests;
    ocxo_tend_and_arm(g_ocxo1_ctx);
  }

  const uint32_t ocxo2_requests =
      g_ocxo2_lane.capture_pending_enqueue_fail_count;
  if (g_ocxo2_capture_overrun_recovery_count != ocxo2_requests) {
    g_ocxo2_capture_overrun_recovery_count = ocxo2_requests;
    ocxo_tend_and_arm(g_ocxo2_ctx);
  }
}

static bool handoff_drain_one_oldest(void) {
  enum source_t : uint8_t {
    SRC_NONE,
    SRC_VCLOCK,
    SRC_CH2,
    SRC_OCXO1,
    SRC_OCXO2,
    SRC_PPS,
  };
  source_t source = SRC_NONE;
  uint32_t best = UINT32_MAX;
  uint32_t sequence = 0U;

  if (capture_ring_peek_sequence(g_vclock_capture_ring, sequence) &&
      sequence < best) {
    best = sequence;
    source = SRC_VCLOCK;
  }
  if (capture_ring_peek_sequence(g_ch2_capture_ring, sequence) &&
      sequence < best) {
    best = sequence;
    source = SRC_CH2;
  }
  if (capture_ring_peek_sequence(g_ocxo1_capture_ring, sequence) &&
      sequence < best) {
    best = sequence;
    source = SRC_OCXO1;
  }
  if (capture_ring_peek_sequence(g_ocxo2_capture_ring, sequence) &&
      sequence < best) {
    best = sequence;
    source = SRC_OCXO2;
  }
  if (capture_ring_peek_sequence(g_pps_capture_ring, sequence) &&
      sequence < best) {
    best = sequence;
    source = SRC_PPS;
  }
  if (source == SRC_NONE) return false;

  switch (source) {
    case SRC_VCLOCK: {
      vclock_capture_packet_t packet{};
      if (capture_ring_pop_handoff(g_vclock_capture_ring,
                                   g_handoff_vclock,
                                   packet)) {
        update_handoff_latency(g_handoff_vclock,
                               packet.isr_entry_dwt_raw);
        process_vclock_packet(packet);
      }
      break;
    }
    case SRC_CH2: {
      ch2_capture_packet_t packet{};
      if (capture_ring_pop_handoff(g_ch2_capture_ring,
                                   g_handoff_ch2,
                                   packet)) {
        update_handoff_latency(g_handoff_ch2,
                               packet.isr_entry_dwt_raw);
        process_ch2_packet(packet);
      }
      break;
    }
    case SRC_OCXO1: {
      ocxo_capture_packet_t packet{};
      if (capture_ring_pop_handoff(g_ocxo1_capture_ring,
                                   g_handoff_ocxo1,
                                   packet)) {
        update_handoff_latency(g_handoff_ocxo1,
                               packet.isr_entry_dwt_raw);
        process_ocxo_packet(g_ocxo1_ctx, packet);
      }
      break;
    }
    case SRC_OCXO2: {
      ocxo_capture_packet_t packet{};
      if (capture_ring_pop_handoff(g_ocxo2_capture_ring,
                                   g_handoff_ocxo2,
                                   packet)) {
        update_handoff_latency(g_handoff_ocxo2,
                               packet.isr_entry_dwt_raw);
        process_ocxo_packet(g_ocxo2_ctx, packet);
      }
      break;
    }
    case SRC_PPS: {
      pps_capture_packet_t packet{};
      if (capture_ring_pop_handoff(g_pps_capture_ring,
                                   g_handoff_pps,
                                   packet)) {
        update_handoff_latency(g_handoff_pps,
                               packet.isr_entry_dwt_raw);
        process_pps_packet(packet);
      }
      break;
    }
    default:
      return false;
  }
  return true;
}

static void interrupt_handoff_service_isr(void) {
  const uint32_t entry_dwt = ARM_DWT_CYCCNT;
  interrupt_isr_diag_enter(
      g_interrupt_priority_runtime.continuation, entry_dwt);
  if (g_interrupt_handoff.running) {
    g_interrupt_handoff.reentry_count++;
    interrupt_handoff_request_isr(entry_dwt);
    interrupt_isr_diag_exit(
        g_interrupt_priority_runtime.continuation, entry_dwt);
    return;
  }
  // Snapshot the request while pending is still true.  Requests from either
  // Priority 0 science or Priority 16 QTimer1 coalesce without replacing the
  // first request timestamp.
  const uint32_t request_dwt = g_interrupt_handoff.last_request_dwt;
  g_interrupt_handoff.running = true;
  g_interrupt_handoff.pending = false;
  g_interrupt_handoff.entry_count++;
  g_interrupt_handoff.last_entry_dwt = entry_dwt;

  const uint32_t latency = entry_dwt - request_dwt;
  if ((int32_t)latency >= 0) {
    g_interrupt_handoff.last_latency_cycles = latency;
    if (latency < g_interrupt_handoff.min_latency_cycles) {
      g_interrupt_handoff.min_latency_cycles = latency;
    }
    if (latency > g_interrupt_handoff.max_latency_cycles) {
      g_interrupt_handoff.max_latency_cycles = latency;
    }
  } else {
    g_interrupt_handoff.last_latency_cycles = 0U;
    g_interrupt_handoff.latency_invalid_count++;
  }

  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::HANDOFF_ENTER,
      timepop_dispatch_trace_kind_t::INTERRUPT_HANDOFF,
      TIMEPOP_DISPATCH_TRACE_NO_SLOT,
      g_interrupt_handoff.entry_count,
      0U,
      0U,
      0U,
      (uint32_t)(uintptr_t)"INTERRUPT_HANDOFF",
      entry_dwt);

  uint32_t drained = 0U;
  while (drained < INTERRUPT_HANDOFF_DRAIN_BUDGET &&
         handoff_drain_one_oldest()) {
    ++drained;
  }
  recover_capture_overruns();
  if (handoff_any_pending()) {
    g_interrupt_handoff.drain_budget_exhausted_count++;
    g_interrupt_handoff.repend_count++;
    interrupt_handoff_request_isr(ARM_DWT_CYCCNT);
  }

  ZPNET_EXECUTION_TRACE(
      timepop_dispatch_trace_stage_t::HANDOFF_EXIT,
      timepop_dispatch_trace_kind_t::INTERRUPT_HANDOFF,
      TIMEPOP_DISPATCH_TRACE_NO_SLOT,
      g_interrupt_handoff.entry_count,
      0U,
      0U,
      0U,
      (uint32_t)(uintptr_t)"INTERRUPT_HANDOFF",
      drained);

  g_interrupt_handoff.exit_count++;
  g_interrupt_handoff.running = false;
  interrupt_isr_diag_exit(
      g_interrupt_priority_runtime.continuation, entry_dwt);
}

static void interrupt_handoff_configure(void) {
  if (g_interrupt_handoff.configured) return;
  attachInterruptVector(INTERRUPT_HANDOFF_IRQ, interrupt_handoff_service_isr);
  NVIC_SET_PRIORITY(INTERRUPT_HANDOFF_IRQ,
                    INTERRUPT_PRIORITY_CONTINUATION);
  interrupt_nvic_icpr_word(INTERRUPT_HANDOFF_IRQ_NUMBER) =
      interrupt_nvic_irq_mask(INTERRUPT_HANDOFF_IRQ_NUMBER);
  interrupt_handoff_barrier();
  NVIC_ENABLE_IRQ(INTERRUPT_HANDOFF_IRQ);
  g_interrupt_handoff.configured = true;
  g_interrupt_handoff.configure_count++;
}

// ============================================================================
// Priority-16 shared QTimer1 capture and Priority-0 science capture
// ============================================================================

static __attribute__((always_inline)) inline void
qtimer1_defuse_vclock_priority16(void) {
  // Preserve the established QTimer1 disable/clear/readback sequence exactly;
  // source defusing is Priority-16 work, but compare rearm is not.
  IMXRT_TMR1.CH[QTIMER1_VCLOCK_CH].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_vclock_clear_compare_flag();
}

static __attribute__((always_inline)) inline void
qtimer1_defuse_timepop_priority16(void) {
  // CH2 is one-shot across the execution-class boundary.  Disable and double-
  // clear the source; foreground TimePop may choose the next target only after
  // the captured fact has completed custody.
  qtimer1_ch2_disable_compare_hardware();
}

static __attribute__((always_inline)) inline void
qtimer_defuse_ocxo_priority0(IMXRT_TMR_t& module, uint8_t channel) {
  // Preserve the existing OCXO double-clear/readback hardware sequence.
  qtimer_disable_compare(module, channel);
}

static __attribute__((always_inline)) inline bool
qtimer1_capture_vclock_priority16(
    uint32_t isr_entry_dwt_raw,
    uint16_t service_low16,
    const interrupt_arrival_capture_t& arrival) {
  const uint32_t target = g_vclock_heartbeat_next_counter32;
  const uint16_t target_low16 = g_vclock_heartbeat_target_low16;
  qtimer1_defuse_vclock_priority16();

  // Every member is assigned explicitly; avoid a compiler-generated packet
  // memset in the Priority-16 capture path.
  vclock_capture_packet_t packet;
  packet.sequence = 0U;
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.arrival = arrival;
  packet.preempted_after_entry =
      g_interrupt_priority_runtime.qtimer1.preempted_during_current_entry;
  packet.target_counter32 = target;
  packet.target_low16 = target_low16;
  packet.service_low16 = service_low16;
  if (capture_ring_push_isr(g_vclock_capture_ring,
                                  g_handoff_vclock,
                                  packet)) {
    return true;
  }

  // Keepalive recovery is requested at Priority 16 and executed at Priority 32.
  g_vclock_capture_overrun_rearm_request_count++;
  return true;
}

static __attribute__((always_inline)) inline bool
qtimer1_service_ch2_flag_priority16(uint32_t isr_entry_dwt_raw,
                                   uint32_t ch2_csctrl) {
  const bool interrupt_enabled =
      (ch2_csctrl & TMR_CSCTRL_TCF1EN) != 0U;
  const bool software_armed = g_qtimer1_ch2_compare_armed;
  const bool fact_outstanding = g_qtimer1_ch2_fact_outstanding;
  const uint16_t hardware_target_low16 =
      IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].COMP1;
  const bool target_matches = software_armed &&
      hardware_target_low16 == g_qtimer1_ch2_armed_target_low16;

  // QTimer1 has one vector for all channels.  A disabled CH2 can still carry a
  // stale/reasserted TCF1 status while CH0 legitimately enters the vector.  A
  // naked flag is not an event: enabled hardware, software arm custody, exact
  // programmed-target identity, and no prior fact must all agree.
  if (!interrupt_enabled || !software_armed || fact_outstanding ||
      !target_matches) {
    g_qtimer1_ch2_rejected_flag_count++;
    g_qtimer1_ch2_unexpected_capture_count++;
    if (!interrupt_enabled) g_qtimer1_ch2_flag_while_disabled_count++;
    if (!software_armed) g_qtimer1_ch2_flag_while_unarmed_count++;
    if (fact_outstanding) g_qtimer1_ch2_flag_while_outstanding_count++;
    if (software_armed && !target_matches) {
      g_qtimer1_ch2_target_mismatch_count++;
    }

    // If hardware claimed to be enabled without lawful software custody, seize
    // and invalidate that rail.  During the legitimate arm-before-enable window
    // interrupt_enabled is false, so the published arm token remains intact and
    // the interrupted foreground arm transaction can safely finish.
    if (interrupt_enabled) {
      g_qtimer1_ch2_compare_armed = false;
      g_qtimer1_ch2_armed_target_counter32 = 0U;
      g_qtimer1_ch2_armed_target_low16 = 0U;
    }
    qtimer1_defuse_timepop_priority16();
    return false;
  }

  const uint32_t target_counter32 =
      g_qtimer1_ch2_armed_target_counter32;

  // Transfer the physically armed identity into one-shot fact custody before
  // any lower execution class can author another target.
  g_qtimer1_ch2_compare_armed = false;
  g_qtimer1_ch2_armed_target_counter32 = 0U;
  g_qtimer1_ch2_armed_target_low16 = 0U;
  g_qtimer1_ch2_fact_outstanding = true;
  dmb_barrier();
  qtimer1_defuse_timepop_priority16();
  g_qtimer1_ch2_valid_capture_count++;

  ch2_capture_packet_t packet{};
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.target_counter32 = target_counter32;
  if (!capture_ring_push_isr(g_ch2_capture_ring,
                                   g_handoff_ch2,
                                   packet)) {
    // The hardware rail remains defused.  Priority 32/foreground will recover
    // scheduler custody rather than inventing or repeating the lost edge.
    g_timepop_foreground_rearm_requested = true;
    g_process_interrupt_foreground_pending = true;
  }
  return true;
}

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  interrupt_isr_diag_enter(
      g_interrupt_priority_runtime.qtimer1, isr_entry_dwt_raw);
  const interrupt_arrival_capture_t arrival =
      interrupt_delay_capture_entry_fast(
          interrupt_execution_source_t::QTIMER1);
  if (g_interrupt_priority_runtime.continuation.active) {
    g_interrupt_priority_runtime.qtimer1_preempted_continuation_count++;
    g_interrupt_priority_runtime.continuation
        .preempted_by_higher_tier_count++;
  }

  const uint32_t vclock_csctrl =
      IMXRT_TMR1.CH[QTIMER1_VCLOCK_CH].CSCTRL;
  const uint32_t ch2_csctrl =
      IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].CSCTRL;
  bool handoff_needed = false;

  // Same-vector matches share one entry coordinate and one Priority-32 pend.
  if ((vclock_csctrl & TMR_CSCTRL_TCF1) != 0U) {
    const uint16_t service_low16 =
        IMXRT_TMR1.CH[QTIMER1_VCLOCK_CH].CNTR;
    handoff_needed |=
        qtimer1_capture_vclock_priority16(
            isr_entry_dwt_raw, service_low16, arrival);
  }
  if ((ch2_csctrl & TMR_CSCTRL_TCF1) != 0U) {
    handoff_needed |=
        qtimer1_service_ch2_flag_priority16(isr_entry_dwt_raw, ch2_csctrl);
  }
  if (handoff_needed) {
    interrupt_handoff_request_isr(isr_entry_dwt_raw);
  }
  interrupt_isr_diag_exit(
      g_interrupt_priority_runtime.qtimer1, isr_entry_dwt_raw);
}

template <uint8_t CHANNEL>
static __attribute__((always_inline)) inline bool ocxo_capture_priority0(
    ocxo_lane_t& lane,
    IMXRT_TMR_t& module,
    interrupt_capture_ring_t<ocxo_capture_packet_t,
                             HANDOFF_OCXO_RING_SIZE>& ring,
    interrupt_handoff_source_diag_t& handoff,
    uint32_t isr_entry_dwt_raw,
    const interrupt_arrival_capture_t& arrival) {
  const uint32_t csctrl = module.CH[CHANNEL].CSCTRL;
  if ((csctrl & TMR_CSCTRL_TCF1) == 0U) {
    lane.false_irq_count++;
    return false;
  }

  // These reads are contemporaneous witnesses.  The authored target remains
  // semantic identity and no later hardware read may replace it.
  const uint16_t ambient_low16 = module.CH[CHANNEL].CNTR;
  const uint16_t compare_low16 = module.CH[CHANNEL].COMP1;
  const bool compare_owned = lane.compare_armed;
  dmb_barrier();
  const uint32_t target_counter32 = lane.armed_target_counter32;
  const uint16_t target_low16 = lane.armed_target_low16;
  const bool active = lane.active && compare_owned &&
      target_counter32 != 0U;

  qtimer_defuse_ocxo_priority0(module, CHANNEL);
  lane.compare_armed = false;

  // Every member is assigned explicitly; avoid a compiler-generated packet
  // memset in the sovereign Priority-0 capture path.
  ocxo_capture_packet_t packet;
  packet.sequence = 0U;
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.arrival = arrival;
  packet.target_counter32 = target_counter32;
  packet.target_low16 = target_low16;
  packet.ambient_low16 = ambient_low16;
  packet.compare_low16 = compare_low16;
  packet.active_at_capture = active;
  if (capture_ring_push_isr(ring, handoff, packet)) {
    if (active) {
      lane.capture_pending_target_counter32 = target_counter32;
      dmb_barrier();
      lane.capture_pending = true;
    }
    return true;
  }

  if (active) {
    // The compare was observed but could not enter custody.  Priority 32 will
    // tend the lane past the lost target and arm the next lawful target.
    lane.capture_pending = false;
    lane.capture_pending_target_counter32 = 0U;
    lane.capture_pending_enqueue_fail_count++;
  }
  return true;
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  const interrupt_arrival_capture_t arrival =
      interrupt_delay_capture_entry_fast(
          interrupt_execution_source_t::OCXO1);
  interrupt_isr_diag_enter(
      g_interrupt_priority_runtime.qtimer2, isr_entry_dwt_raw);
  interrupt_note_science_preemption(
      g_interrupt_priority_runtime.ocxo1_preempted_qtimer1_count);
  if (ocxo_capture_priority0<QTIMER2_OCXO1_CH>(
          g_ocxo1_lane,
          IMXRT_TMR2,
          g_ocxo1_capture_ring,
          g_handoff_ocxo1,
          isr_entry_dwt_raw,
          arrival)) {
    interrupt_handoff_request_isr(isr_entry_dwt_raw);
  }
  interrupt_isr_diag_exit(
      g_interrupt_priority_runtime.qtimer2, isr_entry_dwt_raw);
  // Pending-at-exit attribution must be the final ISR action so a peer edge
  // arriving late in this handler is still named as the blocker.
  interrupt_delay_note_isr_exit(
      interrupt_execution_source_t::OCXO1, isr_entry_dwt_raw, arrival);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  const interrupt_arrival_capture_t arrival =
      interrupt_delay_capture_entry_fast(
          interrupt_execution_source_t::OCXO2);
  interrupt_isr_diag_enter(
      g_interrupt_priority_runtime.qtimer3, isr_entry_dwt_raw);
  interrupt_note_science_preemption(
      g_interrupt_priority_runtime.ocxo2_preempted_qtimer1_count);
  if (ocxo_capture_priority0<QTIMER3_OCXO2_CH>(
          g_ocxo2_lane,
          IMXRT_TMR3,
          g_ocxo2_capture_ring,
          g_handoff_ocxo2,
          isr_entry_dwt_raw,
          arrival)) {
    interrupt_handoff_request_isr(isr_entry_dwt_raw);
  }
  interrupt_isr_diag_exit(
      g_interrupt_priority_runtime.qtimer3, isr_entry_dwt_raw);
  // Pending-at-exit attribution must be the final ISR action so a peer edge
  // arriving late in this handler is still named as the blocker.
  interrupt_delay_note_isr_exit(
      interrupt_execution_source_t::OCXO2, isr_entry_dwt_raw, arrival);
}

static void process_interrupt_gpio6789_irq(
    uint32_t isr_entry_dwt_raw,
    const interrupt_arrival_capture_t& arrival) {
  const uint16_t vclock_hardware16 = qtimer1_ch0_counter_now();
  const bool ocxo1_valid = !OCXO1_DISABLED && g_ocxo1_lane.initialized;
  const bool ocxo2_valid = !OCXO2_DISABLED && g_ocxo2_lane.initialized;
  const uint16_t ocxo1_hardware16 = ocxo1_valid
      ? IMXRT_TMR2.CH[QTIMER2_OCXO1_CH].CNTR
      : 0U;
  const uint16_t ocxo2_hardware16 = ocxo2_valid
      ? IMXRT_TMR3.CH[QTIMER3_OCXO2_CH].CNTR
      : 0U;

  // The edge-coincident relay transition is the sole non-capture side effect.
  // Its three scalar state writes are inseparable from guaranteeing deassertion
  // even if the PPS packet cannot enter the handoff ring.
  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  g_pps_relay_active = true;
  g_pps_relay_countdown_cells = PPS_RELAY_OFF_CELLS;
  g_pps_relay_assert_count++;

  // Every member is assigned explicitly; avoid a compiler-generated packet
  // memset in the sovereign Priority-0 capture path.
  pps_capture_packet_t packet;
  packet.sequence = 0U;
  packet.isr_entry_dwt_raw = isr_entry_dwt_raw;
  packet.arrival = arrival;
  packet.vclock_hardware16 = vclock_hardware16;
  packet.ocxo1_hardware16 = ocxo1_hardware16;
  packet.ocxo2_hardware16 = ocxo2_hardware16;
  packet.ocxo1_valid = ocxo1_valid;
  packet.ocxo2_valid = ocxo2_valid;
  if (capture_ring_push_isr(g_pps_capture_ring,
                                  g_handoff_pps,
                                  packet)) {
    interrupt_handoff_request_isr(isr_entry_dwt_raw);
  } else {
    g_gpio_miss_count++;
  }
}

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  const interrupt_arrival_capture_t arrival =
      interrupt_delay_capture_entry_fast(
          interrupt_execution_source_t::PPS);
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw, arrival);
  interrupt_delay_note_isr_exit(
      interrupt_execution_source_t::PPS, isr_entry_dwt_raw, arrival);
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  const interrupt_arrival_capture_t arrival =
      interrupt_delay_capture_entry_fast(
          interrupt_execution_source_t::PPS);
  interrupt_isr_diag_enter(
      g_interrupt_priority_runtime.pps, isr_entry_dwt_raw);
  interrupt_note_science_preemption(
      g_interrupt_priority_runtime.pps_preempted_qtimer1_count);
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw, arrival);
  interrupt_isr_diag_exit(
      g_interrupt_priority_runtime.pps, isr_entry_dwt_raw);
  // Pending-at-exit attribution must be the final ISR action so a peer edge
  // arriving late in this handler is still named as the blocker.
  interrupt_delay_note_isr_exit(
      interrupt_execution_source_t::PPS, isr_entry_dwt_raw, arrival);
}

// ============================================================================
// Public snapshots / compatibility control boundaries
// ============================================================================

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn callback) {
  g_pps_edge_dispatch =
      interrupt_callback_address_executable((uintptr_t)callback)
          ? callback
          : nullptr;
}

pps_vclock_t interrupt_last_pps_vclock(void) {
  return store_load_pvc();
}

bool interrupt_last_pps_vclock_phase_estimate(
    pps_vclock_phase_estimate_t* out) {
  if (!out) return false;
  const pps_vclock_edge_authority_t authority =
      g_pps_vclock_edge_authority;
  pps_vclock_phase_estimate_t estimate{};
  estimate.valid = authority.valid;
  estimate.lattice_dwt_at_edge = authority.authority_dwt_at_edge;
  estimate.estimated_dwt_at_edge = authority.authority_dwt_at_edge;
  estimate.correction_cycles = 0;
  estimate.phase_mod_scaled_cycles = 0U;
  estimate.tick_scaled_cycles = 0U;
  estimate.scale = 1U;
  estimate.dwt_cycles_per_second = authority.dwt_cycles_per_second;
  estimate.pps_sequence = authority.sequence;
  estimate.pvc_sequence = authority.sequence;
  estimate.pps_dwt_at_edge = authority.pps_dwt_at_edge;
  estimate.pps_counter32_at_edge = authority.counter32_at_edge;
  estimate.pvc_counter32_at_edge = authority.counter32_at_edge;
  *out = estimate;
  return estimate.valid;
}

pps_edge_snapshot_t interrupt_last_pps_edge(void) {
  pps_t pps{};
  pps_vclock_t pvc{};
  (void)store_load(pps, pvc);

  pps_edge_snapshot_t out{};
  out.sequence = pvc.sequence;
  out.dwt_at_edge = pvc.dwt_at_edge;
  out.dwt_raw_at_edge = pvc.dwt_at_edge;
  out.counter32_at_edge = pvc.counter32_at_edge;
  out.ch3_at_edge = pvc.ch3_at_edge;
  out.gnss_ns_at_edge = -1;
  out.physical_pps_dwt_raw_at_edge = pps.dwt_at_edge;
  out.physical_pps_dwt_normalized_at_edge = pps.dwt_at_edge;
  out.physical_pps_counter32_at_read = pps.counter32_at_edge;
  out.physical_pps_ch3_at_read = pps.ch3_at_edge;
  out.spinidle_shadow_valid =
      g_pps_arrival_capture.spinidle_running &&
      g_pps_arrival_capture.spinidle_shadow_dwt != 0U;
  out.spinidle_shadow_dwt = g_pps_arrival_capture.spinidle_shadow_dwt;
  out.spinidle_shadow_to_isr_entry_cycles = out.spinidle_shadow_valid
      ? (uint32_t)(g_pps_arrival_entry_dwt -
                   g_pps_arrival_capture.spinidle_shadow_dwt)
      : 0U;
  const interrupt_delay_baseline_runtime_t* pps_delay_runtime =
      interrupt_delay_baseline_for(interrupt_execution_source_t::PPS);
  out.spinidle_shadow_valid_threshold_cycles =
      pps_delay_runtime && pps_delay_runtime->spin_valid
          ? pps_delay_runtime->spin_baseline_cycles +
                INTERRUPT_DELAY_SPIN_GUARD_CYCLES
          : 0U;
  out.interrupt_delay = g_pps_interrupt_delay;
  out.vclock_epoch_counter32 = pvc.counter32_at_edge;
  out.vclock_epoch_ch3 = pvc.ch3_at_edge;
  out.vclock_epoch_ticks_after_pps =
      VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS;
  out.vclock_epoch_counter32_offset_ticks =
      VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  out.vclock_epoch_dwt_offset_cycles = 0;
  out.vclock_epoch_selected = pvc.sequence != 0U;
  out.vclock_edge_authority = g_pps_vclock_edge_authority;
  return out;
}

interrupt_pps_edge_heartbeat_t interrupt_pps_edge_heartbeat(void) {
  interrupt_pps_edge_heartbeat_t out{};
  out.edge_count = g_pps_gpio_heartbeat.edge_count;
  out.last_dwt = g_pps_gpio_heartbeat.last_dwt;
  out.last_gnss_ns = g_pps_gpio_heartbeat.last_gnss_ns;
  out.gpio_irq_count = g_gpio_irq_count;
  out.gpio_miss_count = g_gpio_miss_count;
  return out;
}

static bool g_publication_launch_acquisition = false;
static uint32_t g_recover_publication_custody_reset_count = 0U;

void interrupt_dwt_publication_launch_acquisition_begin(void) {
  g_publication_launch_acquisition = true;
}

void interrupt_dwt_publication_launch_acquisition_end(void) {
  g_publication_launch_acquisition = false;
}

bool interrupt_dwt_publication_launch_acquisition_active(void) {
  return g_publication_launch_acquisition;
}

void interrupt_recover_reset_publication_custody(void) {
  g_recover_publication_custody_reset_count++;
  g_ocxo1_lane.previous_event_valid = false;
  g_ocxo1_lane.previous_event_counter32 = 0U;
  g_ocxo1_lane.last_counter_delta_ticks = 0U;
  g_ocxo2_lane.previous_event_valid = false;
  g_ocxo2_lane.previous_event_counter32 = 0U;
  g_ocxo2_lane.last_counter_delta_ticks = 0U;
  g_interrupt_integrity.ocxo1_counter =
      interrupt_integrity_counter_check_t{};
  g_interrupt_integrity.ocxo2_counter =
      interrupt_integrity_counter_check_t{};
  g_interrupt_integrity.ocxo1_qtimer_dwt =
      interrupt_integrity_qtimer_dwt_match_check_t{};
  g_interrupt_integrity.ocxo2_qtimer_dwt =
      interrupt_integrity_qtimer_dwt_match_check_t{};
}

uint32_t interrupt_recover_publication_custody_reset_count(void) {
  return g_recover_publication_custody_reset_count;
}

// ============================================================================
// Subscription / service lifecycle
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& subscription) {
  if (!g_interrupt_runtime_ready) return false;
  if (subscription.kind == interrupt_subscriber_kind_t::NONE ||
      !subscription.on_event) {
    return false;
  }
  interrupt_subscriber_runtime_t* rt = runtime_for(subscription.kind);
  if (!rt || !rt->desc) return false;
  const uint32_t prior = interrupt_priority0_guard_enter();
  interrupt_dispatch_invalidate_locked(*rt);
  rt->sub = subscription;
  rt->subscribed = true;
  interrupt_priority0_guard_exit(prior);
  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  const uint32_t prior = interrupt_priority0_guard_enter();
  interrupt_dispatch_invalidate_locked(*rt);
  rt->active = true;
  rt->start_count++;
  interrupt_priority0_guard_exit(prior);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = true;
    return g_vclock_heartbeat_armed || vclock_heartbeat_arm();
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    return OCXO1_DISABLED || ocxo1_start_one_second_service_bound();
  }
  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    return OCXO2_DISABLED || ocxo2_start_one_second_service_bound();
  }
  return false;
}

bool interrupt_ensure_service(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->subscribed || !rt->sub.on_event) return false;
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    if (!rt->active) rt->active = true;
    g_vclock_lane.active = true;
    return g_vclock_heartbeat_armed || vclock_heartbeat_arm();
  }
  if (!rt->active) rt->active = true;
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    return OCXO1_DISABLED || ocxo1_start_one_second_service_bound();
  }
  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    return OCXO2_DISABLED || ocxo2_start_one_second_service_bound();
  }
  return false;
}

template <typename T, uint32_t N>
static uint32_t capture_ring_discard_pending(
    interrupt_capture_ring_t<T, N>& ring) {
  const uint32_t tail = ring.tail;
  dmb_barrier();
  const uint32_t head = ring.head;
  ring.tail = head;
  dmb_barrier();
  return head - tail;
}

// Recovery is intentionally duplicated by lane.  Every phase accepts no
// pointers or references, reloads only literal permanent objects, and has an
// explicit compiler boundary.  No context or runtime address survives across
// the large diagnostic clears or across the final service-start boundary.
static __attribute__((noinline, noclone))
void ocxo1_recovery_runtime_reset_bound(void) {
  interrupt_subscriber_runtime_t& rt =
      g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO1];
  interrupt_dispatch_invalidate_locked(rt);
  rt.last_event = interrupt_event_t{};
  rt.last_diag = interrupt_capture_diag_t{};
  rt.previous_delay_valid = false;
  rt.previous_endpoint_delayed = false;
  rt.previous_delayed_by = interrupt_delay_cause_t::NONE;
  rt.previous_delay_cycles = 0U;
  rt.previous_interval_delay_valid = false;
  rt.previous_interval_delay_cycles = 0;
  rt.previous_interval_delayed_by = interrupt_delay_cause_t::NONE;
  g_delay_seen_ocxo1_token = g_delay_latch_ocxo1.publish_token;
  rt.has_fired = false;
  rt.active = true;
}

static __attribute__((noinline, noclone))
void ocxo2_recovery_runtime_reset_bound(void) {
  interrupt_subscriber_runtime_t& rt =
      g_subscribers[INTERRUPT_SUBSCRIBER_INDEX_OCXO2];
  interrupt_dispatch_invalidate_locked(rt);
  rt.last_event = interrupt_event_t{};
  rt.last_diag = interrupt_capture_diag_t{};
  rt.previous_delay_valid = false;
  rt.previous_endpoint_delayed = false;
  rt.previous_delayed_by = interrupt_delay_cause_t::NONE;
  rt.previous_delay_cycles = 0U;
  rt.previous_interval_delay_valid = false;
  rt.previous_interval_delay_cycles = 0;
  rt.previous_interval_delayed_by = interrupt_delay_cause_t::NONE;
  g_delay_seen_ocxo2_token = g_delay_latch_ocxo2.publish_token;
  rt.has_fired = false;
  rt.active = true;
}

static __attribute__((noinline, noclone))
void ocxo1_recovery_lane_reset_bound(void) {
  g_ocxo1_lane.active = true;
  g_ocxo1_lane.previous_event_valid = false;
  if (g_ocxo1_lane.capture_pending) {
    g_ocxo1_lane.capture_pending_recovery_discard_count++;
    g_ocxo1_lane.capture_pending_clear_count++;
  }
  g_ocxo1_lane.capture_pending_target_counter32 = 0U;
  dmb_barrier();
  g_ocxo1_lane.capture_pending = false;
  dmb_barrier();
  if (g_ocxo1_lane.compare_armed) {
    ocxo_disable_compare(g_ocxo1_lane);
  }
  // RECOVER rebuilds one provable one-second target from the live hardware
  // coordinate; VCLOCK resumes rollover tending after this reset.
  synthetic_clock_tend_from_hardware(g_ocxo1_clock32, ocxo_counter_now(g_ocxo1_lane));
  ocxo_reset_target_grid(g_ocxo1_lane, g_ocxo1_clock32);
  g_ocxo1_lane.rebootstrap_count++;
}

static __attribute__((noinline, noclone))
void ocxo2_recovery_lane_reset_bound(void) {
  g_ocxo2_lane.active = true;
  g_ocxo2_lane.previous_event_valid = false;
  if (g_ocxo2_lane.capture_pending) {
    g_ocxo2_lane.capture_pending_recovery_discard_count++;
    g_ocxo2_lane.capture_pending_clear_count++;
  }
  g_ocxo2_lane.capture_pending_target_counter32 = 0U;
  dmb_barrier();
  g_ocxo2_lane.capture_pending = false;
  dmb_barrier();
  if (g_ocxo2_lane.compare_armed) {
    ocxo_disable_compare(g_ocxo2_lane);
  }
  // RECOVER rebuilds one provable one-second target from the live hardware
  // coordinate; VCLOCK resumes rollover tending after this reset.
  synthetic_clock_tend_from_hardware(g_ocxo2_clock32, ocxo_counter_now(g_ocxo2_lane));
  ocxo_reset_target_grid(g_ocxo2_lane, g_ocxo2_clock32);
  g_ocxo2_lane.rebootstrap_count++;
}

static __attribute__((noinline, noclone))
void ocxo1_recovery_prepare_bound(void) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  (void)capture_ring_discard_pending(g_ocxo1_capture_ring);
  ocxo1_recovery_runtime_reset_bound();
  ocxo1_recovery_lane_reset_bound();
  interrupt_priority0_guard_exit(prior);
}

static __attribute__((noinline, noclone))
void ocxo2_recovery_prepare_bound(void) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  (void)capture_ring_discard_pending(g_ocxo2_capture_ring);
  ocxo2_recovery_runtime_reset_bound();
  ocxo2_recovery_lane_reset_bound();
  interrupt_priority0_guard_exit(prior);
}

static __attribute__((noinline, noclone))
bool ocxo1_recovery_ready_bound(void) {
  return ocxo1_binding_identity_check() && g_ocxo1_lane.initialized;
}

static __attribute__((noinline, noclone))
bool ocxo2_recovery_ready_bound(void) {
  return ocxo2_binding_identity_check() && g_ocxo2_lane.initialized;
}

static __attribute__((noinline, noclone))
void ocxo1_recovery_complete_bound(void) {
  g_ocxo1_lane.recover_count++;
}

static __attribute__((noinline, noclone))
void ocxo2_recovery_complete_bound(void) {
  g_ocxo2_lane.recover_count++;
}

static __attribute__((noinline, noclone))
bool ocxo1_recover_rebootstrap_bound(void) {
  if (OCXO1_DISABLED) return true;
  if (!ocxo1_recovery_ready_bound()) return false;

  NVIC_DISABLE_IRQ(IRQ_QTIMER2);
  ocxo1_recovery_prepare_bound();
  const bool started = ocxo1_start_one_second_service_bound();
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);
  ocxo1_recovery_complete_bound();
  return started;
}

static __attribute__((noinline, noclone))
bool ocxo2_recover_rebootstrap_bound(void) {
  if (OCXO2_DISABLED) return true;
  if (!ocxo2_recovery_ready_bound()) return false;

  NVIC_DISABLE_IRQ(IRQ_QTIMER3);
  ocxo2_recovery_prepare_bound();
  const bool started = ocxo2_start_one_second_service_bound();
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  ocxo2_recovery_complete_bound();
  return started;
}

__attribute__((noinline, noclone))
bool interrupt_recover_rebootstrap_ocxo_service(
    interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::OCXO1:
      return ocxo1_recover_rebootstrap_bound();
    case interrupt_subscriber_kind_t::OCXO2:
      return ocxo2_recover_rebootstrap_bound();
    default:
      return false;
  }
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  const uint32_t prior = interrupt_priority0_guard_enter();
  rt->active = false;
  rt->stop_count++;
  interrupt_dispatch_invalidate_locked(*rt);
  interrupt_priority0_guard_exit(prior);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    return true;
  }
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    ocxo1_stop_one_second_service_bound();
    return true;
  }
  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    ocxo2_stop_one_second_service_bound();
    return true;
  }
  return false;
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_lane.one_second_grid_valid = false;
}

bool interrupt_pps_rebootstrap_pending(void) {
  return g_pps_rebootstrap_pending;
}

const interrupt_event_t* interrupt_last_event(
    interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(
    interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_diag;
}

static void interrupt_timepop_release_ch2_custody_foreground(void) {
  const uint32_t prior = interrupt_priority0_guard_enter();
  if (g_qtimer1_ch2_deferred_arm_valid) {
    const uint32_t target =
        g_qtimer1_ch2_deferred_arm_target_counter32;
    g_qtimer1_ch2_deferred_arm_valid = false;
    g_qtimer1_ch2_deferred_arm_target_counter32 = 0U;

    // Keep fact_outstanding true until armed identity and compare registers are
    // complete.  commit_physical_arm clears custody immediately before enabling
    // TCF1EN, eliminating the former release-before-programming window.
    if (qtimer1_ch2_commit_physical_arm_locked(target, true)) {
      g_qtimer1_ch2_release_arm_count++;
    }
  } else {
    // A normal TimePop pass always authors a heartbeat or a real deadline.  If
    // it did not, release custody into an explicitly disabled/unarmed state and
    // leave a court record rather than silently exposing stale hardware.
    qtimer1_ch2_disable_compare_hardware();
    g_qtimer1_ch2_compare_armed = false;
    g_qtimer1_ch2_armed_target_counter32 = 0U;
    g_qtimer1_ch2_armed_target_low16 = 0U;
    g_qtimer1_ch2_fact_outstanding = false;
    g_qtimer1_ch2_release_without_arm_count++;
  }
  interrupt_priority0_guard_exit(prior);
}

static void interrupt_timepop_foreground_service(void) {
  timepop_foreground_packet_t packet{};
  if (timepop_foreground_mailbox_take(packet)) {
    const uint32_t forensic_service_dwt = ARM_DWT_CYCCNT;
    const uint32_t forensic_service_counter32 =
        interrupt_qtimer1_counter32_now();
    interrupt_forensics_note_ch2_foreground(packet,
                                               forensic_service_dwt,
                                               forensic_service_counter32);

    interrupt_event_t event{};
    event.kind = interrupt_subscriber_kind_t::TIMEPOP;
    event.provider = interrupt_provider_kind_t::QTIMER1;
    event.lane = interrupt_lane_t::QTIMER1_CH2_COMP;
    event.status = interrupt_event_status_t::OK;
    event.dwt_at_event = packet.dwt_at_event;
    event.counter32_at_event = packet.target_counter32;
    event.gnss_ns_at_event = 0U;

    interrupt_capture_diag_t diag{};
    fill_observed_diag(diag,
                       event,
                       packet.isr_entry_dwt_raw,
                       (uint16_t)packet.target_counter32,
                       (uint16_t)packet.target_counter32,
                       packet.sequence);

    ZPNET_EXECUTION_TRACE(
        timepop_dispatch_trace_stage_t::SUBSCRIBER_ENTER,
        timepop_dispatch_trace_kind_t::ISR_TIMED,
        QTIMER1_TIMEPOP_CH,
        packet.sequence,
        (uint32_t)(uintptr_t)timepop_accept_ch2_event_foreground,
        0U,
        packet.target_counter32,
        (uint32_t)(uintptr_t)"QTIMER1_CH2_FOREGROUND",
        event.dwt_at_event);
    timepop_accept_ch2_event_foreground(event, diag);
    ZPNET_EXECUTION_TRACE(
        timepop_dispatch_trace_stage_t::SUBSCRIBER_RETURN,
        timepop_dispatch_trace_kind_t::ISR_TIMED,
        QTIMER1_TIMEPOP_CH,
        packet.sequence,
        (uint32_t)(uintptr_t)timepop_accept_ch2_event_foreground,
        0U,
        packet.target_counter32,
        (uint32_t)(uintptr_t)"QTIMER1_CH2_FOREGROUND",
        event.dwt_at_event);
    g_timepop_foreground_diag.service_count++;
    interrupt_timepop_release_ch2_custody_foreground();
  }

  bool recover = false;
  const uint32_t prior = interrupt_priority0_guard_enter();
  if (g_timepop_foreground_rearm_requested) {
    g_timepop_foreground_rearm_requested = false;
    recover = true;
  }
  interrupt_priority0_guard_exit(prior);
  if (recover) {
    timepop_ch2_capture_lost_foreground();
    g_timepop_foreground_diag.scheduler_recover_count++;
    interrupt_timepop_release_ch2_custody_foreground();
  }
}

void process_interrupt_foreground_service(void) {
  const uint32_t ipsr = interrupt_ipsr();
  if (ipsr != 0U) {
    g_timepop_foreground_diag.handler_reject_count++;
    g_timepop_foreground_diag.handler_last_ipsr = ipsr;
    return;
  }
  if (g_interrupt_foreground_service_running) return;
  g_interrupt_foreground_service_running = true;
  interrupt_forensics_begin_service();

  // Called once per ordinary loop() pass.  This is the sole bridge from
  // interrupt-owned immutable facts into TimePop, application subscribers,
  // and SYSTEM's foreground registry.  Phase timing is passive black-box
  // evidence and cannot change the order or behavior of the bridge.
  uint32_t phase_start = ARM_DWT_CYCCNT;
  interrupt_timepop_foreground_service();
  g_interrupt_foreground_forensic_live.timepop_cycles =
      ARM_DWT_CYCCNT - phase_start;

  phase_start = ARM_DWT_CYCCNT;
  interrupt_dispatch_foreground_service();
  g_interrupt_foreground_forensic_live.subscriber_cycles =
      ARM_DWT_CYCCNT - phase_start;

  phase_start = ARM_DWT_CYCCNT;
  interrupt_features_foreground_flush();
  g_interrupt_foreground_forensic_live.feature_cycles =
      ARM_DWT_CYCCNT - phase_start;

  phase_start = ARM_DWT_CYCCNT;
  const uint32_t prior = interrupt_priority0_guard_enter();
  g_process_interrupt_foreground_pending = interrupt_foreground_work_pending_locked();
  interrupt_priority0_guard_exit(prior);
  g_interrupt_foreground_service_running = false;
  g_interrupt_foreground_forensic_live.finalize_cycles =
      ARM_DWT_CYCCNT - phase_start;
  interrupt_forensics_finish_service();
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {
  // Compatibility callback may run inside TimePop dispatch.  It must never
  // re-enter TimePop's scheduler through the full foreground bridge.
  interrupt_features_foreground_flush();
}

// ============================================================================
// Feature registry handoff (interrupt scalars -> foreground SYSTEM registry)
// ============================================================================

struct interrupt_feature_pending_t {
  volatile bool dirty = false;
  volatile uint32_t generation = 0U;
  volatile uint8_t pps_vclock_authority =
      (uint8_t)system_feature_status_t::INITIALIZING;
  volatile uint8_t qtimer_counter_custody =
      (uint8_t)system_feature_status_t::INITIALIZING;
  volatile uint8_t qtimer_dwt_ruler =
      (uint8_t)system_feature_status_t::INITIALIZING;
  volatile uint8_t counter32_lineage =
      (uint8_t)system_feature_status_t::INITIALIZING;
  volatile uint8_t observed_edge_authority =
      (uint8_t)system_feature_status_t::INITIALIZING;
};

static interrupt_feature_pending_t g_interrupt_feature_pending{};

static inline uint8_t interrupt_feature_status_scalar(
    system_feature_status_t status) {
  return (uint8_t)status;
}

static void interrupt_features_mark_dirty(void) {
  dmb_barrier();
  g_interrupt_feature_pending.generation++;
  g_interrupt_feature_pending.dirty = true;
  g_process_interrupt_foreground_pending = true;
  dmb_barrier();
}

static void interrupt_features_mark_initializing(void) {
  g_interrupt_feature_pending.pps_vclock_authority =
      interrupt_feature_status_scalar(system_feature_status_t::INITIALIZING);
  g_interrupt_feature_pending.qtimer_counter_custody =
      interrupt_feature_status_scalar(system_feature_status_t::INITIALIZING);
  g_interrupt_feature_pending.qtimer_dwt_ruler =
      interrupt_feature_status_scalar(system_feature_status_t::INITIALIZING);
  g_interrupt_feature_pending.counter32_lineage =
      interrupt_feature_status_scalar(system_feature_status_t::INITIALIZING);
  g_interrupt_feature_pending.observed_edge_authority =
      interrupt_feature_status_scalar(system_feature_status_t::INITIALIZING);
  interrupt_features_mark_dirty();
}

static void interrupt_features_note_observed_edge(void) {
  g_interrupt_feature_pending.pps_vclock_authority =
      interrupt_feature_status_scalar(system_feature_status_t::NOMINAL);
  g_interrupt_feature_pending.observed_edge_authority =
      interrupt_feature_status_scalar(system_feature_status_t::NOMINAL);
  g_interrupt_feature_pending.qtimer_dwt_ruler =
      interrupt_feature_status_scalar(system_feature_status_t::NOMINAL);
  interrupt_features_mark_dirty();
}

static void interrupt_features_note_ocxo_custody(void) {
  const bool ocxo1_ok = OCXO1_DISABLED ||
      (g_ocxo1_lane.target_grid_valid && g_ocxo1_lane.active);
  const bool ocxo2_ok = OCXO2_DISABLED ||
      (g_ocxo2_lane.target_grid_valid && g_ocxo2_lane.active);
  g_interrupt_feature_pending.qtimer_counter_custody =
      interrupt_feature_status_scalar(
          ocxo1_ok && ocxo2_ok
              ? system_feature_status_t::NOMINAL
              : system_feature_status_t::INITIALIZING);
  interrupt_features_mark_dirty();
}

static void interrupt_features_note_lineage(void) {
  const bool vclock_ok = g_interrupt_integrity.vclock_qtimer_dwt.one_second.valid;
  const bool ocxo1_ok = OCXO1_DISABLED ||
      g_interrupt_integrity.ocxo1_counter.valid;
  const bool ocxo2_ok = OCXO2_DISABLED ||
      g_interrupt_integrity.ocxo2_counter.valid;
  g_interrupt_feature_pending.counter32_lineage =
      interrupt_feature_status_scalar(
          (vclock_ok && ocxo1_ok && ocxo2_ok)
              ? system_feature_status_t::NOMINAL
              : system_feature_status_t::INITIALIZING);
  interrupt_features_mark_dirty();
}

static void interrupt_features_foreground_flush(void) {
  if (!g_interrupt_feature_pending.dirty) return;

  uint8_t pps_vclock_authority = 0U;
  uint8_t qtimer_counter_custody = 0U;
  uint8_t qtimer_dwt_ruler = 0U;
  uint8_t counter32_lineage = 0U;
  uint8_t observed_edge_authority = 0U;
  uint32_t generation = 0U;

  // BASEPRI=16 masks QTimer1 and continuation while preserving Priority 0 science.
  const uint32_t prior = interrupt_priority0_guard_enter();
  if (!g_interrupt_feature_pending.dirty) {
    interrupt_priority0_guard_exit(prior);
    return;
  }
  pps_vclock_authority = g_interrupt_feature_pending.pps_vclock_authority;
  qtimer_counter_custody =
      g_interrupt_feature_pending.qtimer_counter_custody;
  qtimer_dwt_ruler = g_interrupt_feature_pending.qtimer_dwt_ruler;
  counter32_lineage = g_interrupt_feature_pending.counter32_lineage;
  observed_edge_authority =
      g_interrupt_feature_pending.observed_edge_authority;
  generation = g_interrupt_feature_pending.generation;
  g_interrupt_feature_pending.dirty = false;
  interrupt_priority0_guard_exit(prior);

  (void)system_feature_set(
      "INTERRUPT", "PPS_VCLOCK_AUTHORITY",
      (system_feature_status_t)pps_vclock_authority, nullptr);
  (void)system_feature_set(
      "INTERRUPT", "QTIMER_COUNTER_CUSTODY",
      (system_feature_status_t)qtimer_counter_custody, nullptr);
  (void)system_feature_set(
      "INTERRUPT", "QTIMER_DWT_RULER",
      (system_feature_status_t)qtimer_dwt_ruler, nullptr);
  (void)system_feature_set(
      "INTERRUPT", "COUNTER32_LINEAGE",
      (system_feature_status_t)counter32_lineage, nullptr);
  (void)system_feature_set(
      "INTERRUPT", "OBSERVED_EDGE_AUTHORITY",
      (system_feature_status_t)observed_edge_authority, nullptr);

  // Preserve an update that arrived after the snapshot but before flush ended.
  const uint32_t check = interrupt_priority0_guard_enter();
  if (g_interrupt_feature_pending.generation != generation) {
    g_interrupt_feature_pending.dirty = true;
  }
  interrupt_priority0_guard_exit(check);
}

static bool interrupt_foreground_work_pending_locked(void) {
  if (g_timepop_foreground_mailbox_pending ||
      g_timepop_foreground_rearm_requested ||
      g_interrupt_feature_pending.dirty) {
    return true;
  }
  for (uint32_t i = 0U; i < INTERRUPT_SUBSCRIBER_COUNT; ++i) {
    if (g_subscribers[i].deferred.pending) return true;
  }
  return false;
}

// ============================================================================
// Cortex-M7 floating-point exception-context policy
// ============================================================================

static constexpr uintptr_t INTERRUPT_FPU_FPCCR_ADDRESS = 0xE000EF34UL;
static constexpr uintptr_t INTERRUPT_SCB_CPACR_ADDRESS = 0xE000ED88UL;
static constexpr uint32_t INTERRUPT_FPU_FPCCR_ASPEN = 1UL << 31;
static constexpr uint32_t INTERRUPT_FPU_FPCCR_LSPEN = 1UL << 30;
static constexpr uint32_t INTERRUPT_SCB_CPACR_CP10_CP11_FULL = 0x00F00000UL;

struct interrupt_fpu_context_policy_state_t {
  bool attempted = false;
  bool installed = false;
  uint32_t attempt_count = 0;
  uint32_t verify_count = 0;
  uint32_t irq_enable_block_count = 0;
  uint32_t fpccr_before = 0;
  uint32_t fpccr_after = 0;
  uint32_t last_verify_fpccr = 0;
  uint32_t last_verify_cpacr = 0;
};

static interrupt_fpu_context_policy_state_t g_interrupt_fpu_context_policy{};

static inline volatile uint32_t& interrupt_fpu_fpccr_register(void) {
  return *reinterpret_cast<volatile uint32_t*>(
      INTERRUPT_FPU_FPCCR_ADDRESS);
}

static inline volatile uint32_t& interrupt_scb_cpacr_register(void) {
  return *reinterpret_cast<volatile uint32_t*>(
      INTERRUPT_SCB_CPACR_ADDRESS);
}

static bool interrupt_fpu_context_policy_verify_current(void) {
  interrupt_fpu_context_policy_state_t& state =
      g_interrupt_fpu_context_policy;
  state.verify_count++;
  state.last_verify_fpccr = interrupt_fpu_fpccr_register();
  state.last_verify_cpacr = interrupt_scb_cpacr_register();
  return state.attempted && state.installed &&
      (state.last_verify_fpccr & INTERRUPT_FPU_FPCCR_ASPEN) != 0U &&
      (state.last_verify_fpccr & INTERRUPT_FPU_FPCCR_LSPEN) == 0U &&
      (state.last_verify_cpacr & INTERRUPT_SCB_CPACR_CP10_CP11_FULL) ==
          INTERRUPT_SCB_CPACR_CP10_CP11_FULL;
}

static bool interrupt_fpu_context_policy_install_once(void) {
  interrupt_fpu_context_policy_state_t& state =
      g_interrupt_fpu_context_policy;
  if (state.attempted) return interrupt_fpu_context_policy_verify_current();
  state.attempted = true;
  state.attempt_count++;
  if (interrupt_ipsr() != 0U) return false;

  uint32_t prior_primask = 0U;
  __asm__ volatile ("mrs %0, primask" : "=r" (prior_primask) :: "memory");
  __asm__ volatile ("cpsid i" ::: "memory");
  __asm__ volatile ("dsb" ::: "memory");
  state.fpccr_before = interrupt_fpu_fpccr_register();
  interrupt_fpu_fpccr_register() =
      (state.fpccr_before | INTERRUPT_FPU_FPCCR_ASPEN) &
      ~INTERRUPT_FPU_FPCCR_LSPEN;
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");
  state.fpccr_after = interrupt_fpu_fpccr_register();
  state.installed =
      (state.fpccr_after & INTERRUPT_FPU_FPCCR_ASPEN) != 0U &&
      (state.fpccr_after & INTERRUPT_FPU_FPCCR_LSPEN) == 0U &&
      (interrupt_scb_cpacr_register() &
       INTERRUPT_SCB_CPACR_CP10_CP11_FULL) ==
          INTERRUPT_SCB_CPACR_CP10_CP11_FULL;
  __asm__ volatile ("msr primask, %0" :: "r" (prior_primask) : "memory");
  __asm__ volatile ("isb" ::: "memory");
  return interrupt_fpu_context_policy_verify_current();
}

// ============================================================================
// Hardware and lifecycle initialization
// ============================================================================

static void qtimer_init_count_compare_channel(IMXRT_TMR_t& module,
                                               uint8_t channel,
                                               uint8_t pcs) {
  module.CH[channel].CTRL = 0;
  module.CH[channel].SCTRL = 0;
  module.CH[channel].CSCTRL = 0;
  module.CH[channel].LOAD = 0;
  module.CH[channel].CNTR = 0;
  module.CH[channel].COMP1 = 0xFFFF;
  module.CH[channel].CMPLD1 = 0xFFFF;
  module.CH[channel].CMPLD2 = 0;
  module.CH[channel].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(pcs);
}

static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);
  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);
  qtimer_init_count_compare_channel(IMXRT_TMR1,
                                    QTIMER1_VCLOCK_CH,
                                    QTIMER1_VCLOCK_PCS);
  qtimer_disable_compare(IMXRT_TMR1, QTIMER1_VCLOCK_CH);
}

static void qtimer1_init_ch2_scheduler(void) {
  qtimer_init_count_compare_channel(IMXRT_TMR1,
                                    QTIMER1_TIMEPOP_CH,
                                    QTIMER1_TIMEPOP_PCS);
  qtimer_disable_compare(IMXRT_TMR1, QTIMER1_TIMEPOP_CH);
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].SCTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CSCTRL = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].LOAD = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].COMP1 = 0xFFFF;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CMPLD2 = 0;
}

void process_interrupt_init_hardware(void) {
  if (!interrupt_fpu_context_policy_install_once()) return;
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.kind = interrupt_subscriber_kind_t::OCXO1;
  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR2;
  g_ocxo1_lane.channel = QTIMER2_OCXO1_CH;
  g_ocxo1_lane.pcs = QTIMER2_OCXO1_PCS;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.kind = interrupt_subscriber_kind_t::OCXO2;
  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = QTIMER3_OCXO2_CH;
  g_ocxo2_lane.pcs = QTIMER3_OCXO2_PCS;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  if (!OCXO1_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
    *(portConfigRegister(OCXO1_PIN)) = 1;
#if defined(IOMUXC_QTIMER2_TIMER0_SELECT_INPUT)
    IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;
#endif
    IMXRT_TMR2.ENBL &= ~(uint16_t)(1U << QTIMER2_OCXO1_CH);
    qtimer_init_count_compare_channel(IMXRT_TMR2,
                                      QTIMER2_OCXO1_CH,
                                      QTIMER2_OCXO1_PCS);
    qtimer_disable_compare(IMXRT_TMR2, QTIMER2_OCXO1_CH);
    IMXRT_TMR2.CH[QTIMER2_OCXO1_CH].CNTR = 0;
    IMXRT_TMR2.ENBL |= (uint16_t)(1U << QTIMER2_OCXO1_CH);
    g_ocxo1_lane.initialized = true;
  }

  if (!OCXO2_DISABLED) {
    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
    *(portConfigRegister(OCXO2_PIN)) = 1;
#if defined(IOMUXC_QTIMER3_TIMER3_SELECT_INPUT)
    IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;
#endif
    IMXRT_TMR3.ENBL &= ~(uint16_t)((1U << QTIMER3_OCXO2_CH) |
                                   (1U << 0) | (1U << 1) | (1U << 2));
    for (uint8_t channel = 0; channel < 3U; ++channel) {
      IMXRT_TMR3.CH[channel].CTRL = 0;
      IMXRT_TMR3.CH[channel].SCTRL = 0;
      IMXRT_TMR3.CH[channel].CSCTRL = 0;
    }
    qtimer_init_count_compare_channel(IMXRT_TMR3,
                                      QTIMER3_OCXO2_CH,
                                      QTIMER3_OCXO2_PCS);
    qtimer_disable_compare(IMXRT_TMR3, QTIMER3_OCXO2_CH);
    IMXRT_TMR3.CH[QTIMER3_OCXO2_CH].CNTR = 0;
    IMXRT_TMR3.ENBL |= (uint16_t)(1U << QTIMER3_OCXO2_CH);
    g_ocxo2_lane.initialized = true;
  }

  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();
  IMXRT_TMR1.CH[QTIMER1_VCLOCK_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].CNTR = 0;
  IMXRT_TMR1.CH[QTIMER1_RETIRED_AUX_CH].CNTR = 0;
  IMXRT_TMR1.ENBL = (uint16_t)((1U << QTIMER1_VCLOCK_CH) |
                               (1U << QTIMER1_TIMEPOP_CH));
  g_vclock_lane.initialized = true;
  g_interrupt_hw_ready = true;
}

static void runtime_init_subscribers(void) {
  g_subscriber_count = 0U;
  for (interrupt_subscriber_runtime_t& rt : g_subscribers) {
    rt = interrupt_subscriber_runtime_t{};
  }
  for (uint32_t i = 0U; i < INTERRUPT_SUBSCRIBER_COUNT; ++i) {
    g_subscribers[i].desc = &DESCRIPTORS[i];
  }
  dmb_barrier();
  g_subscriber_count = INTERRUPT_SUBSCRIBER_COUNT;
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;
  runtime_init_subscribers();
  interrupt_delay_runtime_reset();
  g_interrupt_integrity = interrupt_integrity_snapshot_t{};
  g_store = snapshot_store_t{};
  g_epoch_capture = epoch_capture_store_t{};
  g_pps_gpio_heartbeat = pps_gpio_heartbeat_t{};
  g_last_pps_witness = pps_t{};
  g_last_pps_witness_valid = false;
  g_pps_vclock_edge_authority = pps_vclock_edge_authority_t{};
  g_smartzero = smartzero_runtime_t{};
  g_timepop_foreground_mailbox = timepop_foreground_packet_t{};
  g_timepop_foreground_mailbox_pending = false;
  g_timepop_foreground_diag = timepop_foreground_diag_t{};
  g_timepop_foreground_rearm_requested = false;
  g_process_interrupt_foreground_pending = false;
  g_interrupt_foreground_service_running = false;
  g_interrupt_foreground_forensics =
      interrupt_foreground_forensic_runtime_t{};
  g_interrupt_foreground_forensic_live =
      interrupt_foreground_forensic_record_t{};
  g_interrupt_foreground_forensic_live_active = false;
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
    smartzero_reset_lane(i);
  }
  g_interrupt_runtime_ready = true;
  interrupt_features_mark_initializing();
  interrupt_handoff_configure();
  (void)vclock_heartbeat_arm();
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;
  if (!g_interrupt_hw_ready ||
      !interrupt_fpu_context_policy_verify_current()) {
    g_interrupt_fpu_context_policy.irq_enable_block_count++;
    return;
  }

  interrupt_handoff_configure();

  // Priority 16: shared native VCLOCK CH0 + TimePop CH2 vector.
  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, INTERRUPT_PRIORITY_VCLOCK_TIMEPOP);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  // Priority 0: sparse sovereign science captures.
  if (!OCXO1_DISABLED) {
    attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER2, INTERRUPT_PRIORITY_SCIENCE);
    NVIC_ENABLE_IRQ(IRQ_QTIMER2);
  }
  if (!OCXO2_DISABLED) {
    attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER3, INTERRUPT_PRIORITY_SCIENCE);
    NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  }

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, INTERRUPT_PRIORITY_SCIENCE);
  g_interrupt_irqs_enabled = true;
  interrupt_priority_verify_live();
}

// ============================================================================
// Lean observed-only reports
// ============================================================================

static FLASHMEM void add_runtime_summary(Payload& payload,
                                         const char* prefix,
                                         const interrupt_subscriber_runtime_t* rt) {
  char key[64];
  snprintf(key, sizeof(key), "%s_subscribed", prefix);
  payload.add(key, rt ? rt->subscribed : false);
  snprintf(key, sizeof(key), "%s_active", prefix);
  payload.add(key, rt ? rt->active : false);
  snprintf(key, sizeof(key), "%s_event_count", prefix);
  payload.add(key, rt ? rt->event_count : 0U);
  snprintf(key, sizeof(key), "%s_dispatch_count", prefix);
  payload.add(key, rt ? rt->dispatch_count : 0U);
  snprintf(key, sizeof(key), "%s_dispatch_busy_drop_count", prefix);
  payload.add(key, rt ? rt->dispatch_busy_drop_count : 0U);
  snprintf(key, sizeof(key), "%s_dispatch_arm_fail_count", prefix);
  payload.add(key, rt ? rt->dispatch_arm_fail_count : 0U);
}

static FLASHMEM void add_ocxo_lane_report(Payload& payload,
                                          const char* prefix,
                                          const ocxo_runtime_context_t& ctx) {
  const ocxo_lane_t& lane = *ctx.lane;
  const synthetic_clock32_t& clock = *ctx.clock32;
  char key[80];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };

  add_bool("initialized", lane.initialized);
  add_bool("active", lane.active);
  add_bool("target_grid_valid", lane.target_grid_valid);
  add_bool("compare_armed", lane.compare_armed);
  add_bool("capture_pending", (bool)lane.capture_pending);
  add_u32("capture_pending_target_counter32",
          lane.capture_pending_target_counter32);
  add_u32("clock32", clock.current_counter32);
  add_u32("hardware16", (uint32_t)clock.hardware16);
  add_u32("grid_epoch_counter32", lane.grid_epoch_counter32);
  add_u32("next_target_counter32", lane.next_target_counter32);
  add_u32("armed_target_counter32", lane.armed_target_counter32);
  add_u32("armed_target_low16", (uint32_t)lane.armed_target_low16);
  add_bool("one_second_compare_only", true);
  add_bool("one_second_boundary_reachable",
           lane.target_grid_valid && lane.next_target_counter32 != 0U);
  add_u32("one_second_ticks", OCXO_ONE_SECOND_TICKS);
  add_u32("compare_period_ticks", OCXO_COMPARE_PERIOD_TICKS);
  add_u32("compare_rate_hz", OCXO_COMPARE_RATE_HZ);
  add_u32("target_grid_reset_count", lane.target_grid_reset_count);
  add_u32("arm_count", lane.arm_count);
  add_u32("fire_count", lane.fire_count);
  // Legacy cadence keys remain explicit zeroes for report compatibility.
  add_bool("cadence_enabled", false);
  add_u32("next_target_tooth", 0U);
  add_u32("armed_target_tooth", 0U);
  add_u32("capture_pending_target_tooth", 0U);
  add_u32("last_fired_target_tooth", 0U);
  add_u32("cadence_phase_reset_count", 0U);
  add_u32("cadence_phase_invalid_count", 0U);
  add_u32("cadence_fire_count", 0U);
  add_u32("cadence_boundary_count", 0U);
  add_u32("cadence_ticks", 0U);
  add_u32("cadence_hz", 0U);
  add_u32("missed_target_count", lane.missed_target_count);
  add_u32("rollover_tend_count", lane.rollover_tend_count);
  add_u32("arm_window_wait_count", lane.arm_window_wait_count);
  add_u32("arm_too_close_count", lane.arm_too_close_count);
  add_u32("rebootstrap_count", lane.rebootstrap_count);
  add_u32("recover_count", lane.recover_count);
  add_u32("binding_identity_check_count",
          lane.binding_identity_check_count);
  add_u32("binding_identity_failure_count",
          lane.binding_identity_failure_count);
  add_bool("binding_identity_exact",
           ocxo_static_binding_exact(ctx.kind));
  add_u32("capture_pending_set_count", lane.capture_pending_set_count);
  add_u32("capture_pending_clear_count", lane.capture_pending_clear_count);
  add_u32("capture_pending_tend_skip_count",
          lane.capture_pending_tend_skip_count);
  add_u32("capture_pending_enqueue_fail_count",
          lane.capture_pending_enqueue_fail_count);
  add_u32("capture_pending_missing_count",
          lane.capture_pending_missing_count);
  add_u32("capture_pending_target_mismatch_count",
          lane.capture_pending_target_mismatch_count);
  add_u32("capture_pending_recovery_discard_count",
          lane.capture_pending_recovery_discard_count);
  add_u32("target_commit_rollback_count",
          lane.target_commit_rollback_count);
  add_u32("target_commit_ns_underflow_count",
          lane.target_commit_ns_underflow_count);
  add_i32("last_target_commit_delta_ticks",
          lane.last_target_commit_delta_ticks);
  add_u32("last_target_commit_rollback_ticks",
          lane.last_target_commit_rollback_ticks);
  add_u32("last_arm_remaining_ticks", lane.last_arm_remaining_ticks);
  add_u32("last_dwt_at_edge", lane.last_dwt_at_edge);
  add_u32("last_event_counter32", lane.last_event_counter32);
  add_u32("last_ambient_low16", (uint32_t)lane.last_ambient_low16);
  add_u32("last_compare_low16", (uint32_t)lane.last_compare_low16);
  add_i32("ambient_minus_target_ticks",
          lane.last_ambient_minus_target_ticks);
  add_u32("cadence_delta_ticks", 0U);
  add_u32("cadence_delta_violation_count", 0U);
  add_u32("counter_delta_ticks", lane.last_counter_delta_ticks);
  add_u32("counter_delta_violation_count",
          lane.counter_delta_violation_count);
}

static FLASHMEM void add_irq_priority_report(
    Payload& payload,
    const char* prefix,
    uint32_t irq,
    uint32_t requested_priority,
    const interrupt_isr_runtime_diag_t& diag) {
  char key[80];
  const uint32_t observed_priority = interrupt_nvic_priority_read(irq);

  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };

  add_u32("irq", irq);
  add_u32("requested_priority", requested_priority);
  add_u32("observed_priority", observed_priority);
  add_bool("priority_match", observed_priority == requested_priority);
  add_bool("enabled", interrupt_nvic_enabled(irq));
  add_bool("pending", interrupt_nvic_pending(irq));
  add_bool("active", interrupt_nvic_active(irq));
  add_u32("entry_count", (uint32_t)diag.entry_count);
  add_u32("last_wall_cycles", (uint32_t)diag.last_wall_cycles);
  add_u32("max_wall_cycles", (uint32_t)diag.max_wall_cycles);
  add_u32("last_entry_basepri", (uint32_t)diag.last_entry_basepri);
  add_u32("entry_with_basepri_nonzero_count",
          (uint32_t)diag.entry_with_basepri_nonzero_count);
  add_u32("last_entry_primask", (uint32_t)diag.last_entry_primask);
  add_u32("entry_with_primask_nonzero_count",
          (uint32_t)diag.entry_with_primask_nonzero_count);
  add_u32("last_entry_ipsr", (uint32_t)diag.last_entry_ipsr);
  add_u32("preempted_by_higher_tier_count",
          (uint32_t)diag.preempted_by_higher_tier_count);
}

static FLASHMEM Payload cmd_report_priorities(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_PRIORITIES");
  payload.add("topology", "P0_SCIENCE__P16_QTIMER1__P32_CONTINUATION__FOREGROUND");
  payload.add("basepri_mask_threshold",
              INTERRUPT_PRIORITY0_PRESERVING_BASEPRI);
  payload.add("current_ipsr", interrupt_ipsr());
  payload.add("current_basepri", interrupt_basepri());
  payload.add("current_primask", interrupt_primask());
  payload.add("verify_count",
              (uint32_t)g_interrupt_priority_runtime.verify_count);
  payload.add("verify_mismatch_count",
              (uint32_t)g_interrupt_priority_runtime.verify_mismatch_count);
  payload.add("last_verify_all_match",
              (bool)g_interrupt_priority_runtime.last_verify_all_match);
  payload.add("live_readback_all_match",
              interrupt_priority_readback_all_match());

  add_irq_priority_report(
      payload, "qtimer1", (uint32_t)IRQ_QTIMER1,
      INTERRUPT_PRIORITY_VCLOCK_TIMEPOP,
      g_interrupt_priority_runtime.qtimer1);
  add_irq_priority_report(
      payload, "ocxo1", (uint32_t)IRQ_QTIMER2,
      INTERRUPT_PRIORITY_SCIENCE,
      g_interrupt_priority_runtime.qtimer2);
  add_irq_priority_report(
      payload, "ocxo2", (uint32_t)IRQ_QTIMER3,
      INTERRUPT_PRIORITY_SCIENCE,
      g_interrupt_priority_runtime.qtimer3);
  add_irq_priority_report(
      payload, "pps", (uint32_t)IRQ_GPIO6789,
      INTERRUPT_PRIORITY_SCIENCE,
      g_interrupt_priority_runtime.pps);
  add_irq_priority_report(
      payload, "continuation", INTERRUPT_HANDOFF_IRQ_NUMBER,
      INTERRUPT_PRIORITY_CONTINUATION,
      g_interrupt_priority_runtime.continuation);

  payload.add("qtimer1_preempted_continuation_count",
              (uint32_t)g_interrupt_priority_runtime
                  .qtimer1_preempted_continuation_count);
  payload.add("ocxo1_preempted_qtimer1_count",
              (uint32_t)g_interrupt_priority_runtime
                  .ocxo1_preempted_qtimer1_count);
  payload.add("ocxo2_preempted_qtimer1_count",
              (uint32_t)g_interrupt_priority_runtime
                  .ocxo2_preempted_qtimer1_count);
  payload.add("pps_preempted_qtimer1_count",
              (uint32_t)g_interrupt_priority_runtime
                  .pps_preempted_qtimer1_count);
  payload.add("science_preempted_continuation_count",
              (uint32_t)g_interrupt_priority_runtime
                  .science_preempted_continuation_count);
  payload.add("vclock_dwt_capture_class", "PRIORITY16_OBSERVED");
  payload.add("ocxo_dwt_capture_class", "PRIORITY0_SOVEREIGN");
  return payload;
}

static FLASHMEM Payload cmd_report_status(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_STATUS");
  payload.add("architecture", "OBSERVED_EDGE_ONLY");
  payload.add("ema_present", false);
  payload.add("yardstick_dwt_candidate_present", false);
  payload.add("ocxo_1khz_cadence_present", false);
  payload.add("ocxo_1khz_capture_hook", "DISABLED");
  payload.add("ocxo_match_period_ticks", OCXO_COMPARE_PERIOD_TICKS);
  payload.add("ocxo_compare_rate_hz", OCXO_COMPARE_RATE_HZ);
  payload.add("ocxo_rollover_owner", "VCLOCK_HEARTBEAT");
  payload.add("ocxo_boundary_period_ticks", OCXO_ONE_SECOND_TICKS);
  payload.add("priority_topology", "SCIENCE_0_QTIMER1_16_CONTINUATION_32");
  payload.add("priority0_science", INTERRUPT_PRIORITY_SCIENCE);
  payload.add("priority16_vclock_timepop",
              INTERRUPT_PRIORITY_VCLOCK_TIMEPOP);
  payload.add("priority32_continuation",
              INTERRUPT_PRIORITY_CONTINUATION);
  payload.add("basepri_science_guard",
              INTERRUPT_PRIORITY0_PRESERVING_BASEPRI);
  payload.add("priority_readback_all_match",
              interrupt_priority_readback_all_match());
  payload.add("hardware_ready", g_interrupt_hw_ready);
  payload.add("runtime_ready", g_interrupt_runtime_ready);
  payload.add("irqs_enabled", g_interrupt_irqs_enabled);
  payload.add("vclock_heartbeat_armed", (bool)g_vclock_heartbeat_armed);
  payload.add("vclock_heartbeat_count", g_vclock_lane.heartbeat_count);
  payload.add("vclock_one_second_count", g_vclock_lane.one_second_count);
  add_runtime_summary(
      payload, "vclock", runtime_for(interrupt_subscriber_kind_t::VCLOCK));
  add_runtime_summary(
      payload, "ocxo1", runtime_for(interrupt_subscriber_kind_t::OCXO1));
  add_runtime_summary(
      payload, "ocxo2", runtime_for(interrupt_subscriber_kind_t::OCXO2));
  return payload;
}

static FLASHMEM Payload cmd_report(const Payload& args) {
  return cmd_report_status(args);
}

static FLASHMEM Payload cmd_report_fpu_context(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_FPU_CONTEXT");
  payload.add("attempted", g_interrupt_fpu_context_policy.attempted);
  payload.add("installed", g_interrupt_fpu_context_policy.installed);
  payload.add("attempt_count", g_interrupt_fpu_context_policy.attempt_count);
  payload.add("verify_count", g_interrupt_fpu_context_policy.verify_count);
  payload.add("irq_enable_block_count",
              g_interrupt_fpu_context_policy.irq_enable_block_count);
  payload.add("fpccr_before", g_interrupt_fpu_context_policy.fpccr_before);
  payload.add("fpccr_after", g_interrupt_fpu_context_policy.fpccr_after);
  payload.add("last_verify_fpccr",
              g_interrupt_fpu_context_policy.last_verify_fpccr);
  payload.add("last_verify_cpacr",
              g_interrupt_fpu_context_policy.last_verify_cpacr);
  return payload;
}

static FLASHMEM Payload cmd_report_pps(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_PPS");
  const pps_edge_snapshot_t snapshot = interrupt_last_pps_edge();
  payload.add("sequence", snapshot.sequence);
  payload.add("pps_dwt_at_edge",
              snapshot.physical_pps_dwt_normalized_at_edge);
  payload.add("vclock_dwt_at_edge", snapshot.dwt_at_edge);
  payload.add("vclock_counter32_at_edge", snapshot.counter32_at_edge);
  payload.add("vclock_target_low16", (uint32_t)snapshot.ch3_at_edge);
  payload.add("authority", "OBSERVED_QTIMER_EDGE");
  payload.add("gpio_irq_count", g_gpio_irq_count);
  payload.add("gpio_miss_count", g_gpio_miss_count);
  payload.add("relay_active", (bool)g_pps_relay_active);
  payload.add("relay_assert_count", (uint32_t)g_pps_relay_assert_count);
  payload.add("relay_deassert_count", (uint32_t)g_pps_relay_deassert_count);
  payload.add("rebootstrap_pending", (bool)g_pps_rebootstrap_pending);
  payload.add("rebootstrap_count", g_pps_rebootstrap_count);
  return payload;
}

static FLASHMEM Payload cmd_report_cadence(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_CADENCE");
  payload.add("vclock_period_ticks", (uint32_t)VCLOCK_INTERVAL_COUNTS);
  payload.add("ocxo_period_ticks", OCXO_COMPARE_PERIOD_TICKS);
  payload.add("ocxo_compare_rate_hz", OCXO_COMPARE_RATE_HZ);
  payload.add("ocxo_1khz_cadence_present", false);
  payload.add("ocxo_cadence_hz", 0U);
  payload.add("ocxo_teeth_per_second", 0U);
  payload.add("ocxo_boundary_period_ticks", OCXO_ONE_SECOND_TICKS);
  payload.add("ocxo_arm_window_ticks", OCXO_ARM_WINDOW_TICKS);
  payload.add("ocxo_min_arm_lead_ticks", OCXO_MIN_ARM_LEAD_TICKS);
  payload.add("rollover_owner", "VCLOCK_PRIORITY16_HEARTBEAT");
  payload.add("vclock_rearm_context", "PRIORITY16_HANDOFF");
  payload.add("counter_authority", "AUTHORED_COMPARE_TARGET");
  payload.add("ambient_counter_role", "WITNESS_ONLY");
  add_ocxo_lane_report(payload, "ocxo1", g_ocxo1_ctx);
  add_ocxo_lane_report(payload, "ocxo2", g_ocxo2_ctx);
  return payload;
}

static const char* smartzero_phase_name(interrupt_smartzero_phase_t phase) {
  switch (phase) {
    case interrupt_smartzero_phase_t::RUNNING: return "RUNNING";
    case interrupt_smartzero_phase_t::COMPLETE: return "COMPLETE";
    case interrupt_smartzero_phase_t::ABORTED: return "ABORTED";
    default: return "IDLE";
  }
}

static const char* smartzero_lane_state_name(
    interrupt_smartzero_lane_state_t state) {
  switch (state) {
    case interrupt_smartzero_lane_state_t::ACQUIRING: return "ACQUIRING";
    case interrupt_smartzero_lane_state_t::LOCKED: return "LOCKED";
    default: return "IDLE";
  }
}

static const char* smartzero_decision_name(
    interrupt_smartzero_decision_t decision) {
  switch (decision) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:
      return "WAITING_FOR_CPS";
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:
      return "FIRST_SAMPLE";
    case interrupt_smartzero_decision_t::ACCEPTED:
      return "ACCEPTED";
    case interrupt_smartzero_decision_t::REJECTED_DWT:
      return "NO_QUORUM";
    case interrupt_smartzero_decision_t::REJECTED_COUNTER:
      return "REJECTED_COUNTER";
    default:
      return "NONE";
  }
}

static const char* smartzero_lane_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1: return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2: return "OCXO2";
    default: return "NONE";
  }
}

static FLASHMEM void add_smartzero_lane_report(
    Payload& payload,
    const char* prefix,
    const interrupt_smartzero_lane_snapshot_t& lane,
    const smartzero_lane_vote_report_t& vote) {
  char key[64];

  auto add_string = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    payload.add(key, value);
  };

  add_string("state", smartzero_lane_state_name(lane.state));
  add_string("last_decision", smartzero_decision_name(lane.last_decision));
  add_u32("sample_count", lane.sample_count);
  add_u32("interval_attempt_count", lane.interval_attempt_count);
  add_u32("accepted_count", lane.accepted_count);
  add_u32("no_quorum_count", lane.rejected_count);
  add_u32("waiting_for_cps_count", lane.waiting_for_cps_count);
  add_u32("cps_used", lane.cps_used);
  add_u32("expected_interval_cycles", lane.expected_interval_cycles);
  add_u32("last_interval_cycles", lane.last_interval_cycles);
  add_i32("last_interval_error_cycles", lane.last_interval_error_cycles);
  add_u32("max_abs_interval_error_cycles",
          lane.max_abs_interval_error_cycles);
  add_u32("last_counter_delta_ticks", lane.last_counter_delta_ticks);
  add_u32("required_counter_delta_ticks",
          lane.required_counter_delta_ticks);
  add_u32("previous_sample_dwt", lane.previous_sample_dwt);
  add_u32("last_sample_dwt", lane.last_sample_dwt);
  add_u32("previous_sample_counter32", lane.previous_sample_counter32);
  add_u32("last_sample_counter32", lane.last_sample_counter32);
  add_u32("anchor_dwt", lane.anchor_dwt);
  add_u32("anchor_counter32", lane.anchor_counter32);
  add_u32("history_count", vote.history_count);
  add_u32("valid_interval_count", vote.valid_interval_count);
  add_u32("quorum_count", vote.quorum_count);
  add_u32("quorum_min_cycles", vote.quorum_min_cycles);
  add_u32("quorum_max_cycles", vote.quorum_max_cycles);
  add_u32("quorum_span_cycles", vote.quorum_span_cycles);
  add_u32("closest_three_span_cycles", vote.closest_three_span_cycles);
  add_u32("accepted_interval_cycles", vote.accepted_interval_cycles);
  add_u32("accepted_history_age", vote.accepted_history_age);
}

static FLASHMEM Payload cmd_report_smartzero(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_SMARTZERO");
  interrupt_smartzero_snapshot_t snapshot{};
  smartzero_lane_vote_report_t votes[SMARTZERO_LANE_COUNT]{};
  (void)smartzero_snapshot_load(&snapshot, votes);
  payload.add("phase", smartzero_phase_name(snapshot.phase));
  payload.add("running", snapshot.running);
  payload.add("complete", snapshot.complete);
  payload.add("aborted", snapshot.aborted);
  payload.add("sequence", snapshot.sequence);
  payload.add("begin_count", snapshot.begin_count);
  payload.add("complete_count", snapshot.complete_count);
  payload.add("abort_count", snapshot.abort_count);
  payload.add("current_lane_index", snapshot.current_lane_index);
  payload.add("current_lane", smartzero_lane_name(snapshot.current_lane));
  payload.add("algorithm", "RECENT_THREE_VOTE");
  payload.add("sample_rate_hz", snapshot.sample_rate_hz);
  payload.add("counter_delta_ticks", snapshot.counter_delta_ticks);
  payload.add("history_size", SMARTZERO_HISTORY_SIZE);
  payload.add("quorum_required", SMARTZERO_QUORUM_REQUIRED);
  payload.add("vote_span_cycles", SMARTZERO_VOTE_SPAN_CYCLES);
  payload.add("tolerance_cycles", snapshot.tolerance_cycles);

  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; ++i) {
    const char* prefix = i == 0U ? "vclock" : (i == 1U ? "ocxo1" : "ocxo2");
    add_smartzero_lane_report(payload,
                              prefix,
                              snapshot.lanes[i],
                              votes[i]);
  }
  return payload;
}

static FLASHMEM void add_handoff_source(
    Payload& payload,
    const char* prefix,
    const interrupt_handoff_source_diag_t& source) {
  char key[64];
  snprintf(key, sizeof(key), "%s_capture", prefix);
  payload.add(key, (uint32_t)source.capture_count);
  snprintf(key, sizeof(key), "%s_dequeue", prefix);
  payload.add(key, (uint32_t)source.dequeue_count);
  snprintf(key, sizeof(key), "%s_overrun", prefix);
  payload.add(key, (uint32_t)source.overrun_count);
  snprintf(key, sizeof(key), "%s_pending", prefix);
  payload.add(key, (uint32_t)source.pending_count);
  snprintf(key, sizeof(key), "%s_high_water", prefix);
  payload.add(key, (uint32_t)source.high_water);
  snprintf(key, sizeof(key), "%s_last_latency_cycles", prefix);
  payload.add(key, (uint32_t)source.last_capture_to_handoff_cycles);
  snprintf(key, sizeof(key), "%s_max_latency_cycles", prefix);
  payload.add(key, (uint32_t)source.max_capture_to_handoff_cycles);
  snprintf(key, sizeof(key), "%s_latency_invalid_count", prefix);
  payload.add(key, (uint32_t)source.latency_invalid_count);
}

static FLASHMEM Payload cmd_report_handoff(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_HANDOFF");
  payload.add("entry_latency_measurement", "FIRST_REQUEST_TO_ENTRY");
  payload.add("source_latency_measurement", "CAPTURE_TO_DEQUEUE");
  payload.add("execution_priority", INTERRUPT_PRIORITY_CONTINUATION);
  payload.add("observed_priority",
              interrupt_nvic_priority_read(INTERRUPT_HANDOFF_IRQ_NUMBER));
  payload.add("priority_match",
              interrupt_nvic_priority_read(INTERRUPT_HANDOFF_IRQ_NUMBER) ==
                  INTERRUPT_PRIORITY_CONTINUATION);
  payload.add("configured", (bool)g_interrupt_handoff.configured);
  payload.add("request_count", (uint32_t)g_interrupt_handoff.request_count);
  payload.add("entry_count", (uint32_t)g_interrupt_handoff.entry_count);
  payload.add("exit_count", (uint32_t)g_interrupt_handoff.exit_count);
  payload.add("reentry_count", (uint32_t)g_interrupt_handoff.reentry_count);
  payload.add("repend_count", (uint32_t)g_interrupt_handoff.repend_count);
  payload.add("drain_budget_exhausted_count",
              (uint32_t)g_interrupt_handoff.drain_budget_exhausted_count);
  payload.add("latency_invalid_count",
              (uint32_t)g_interrupt_handoff.latency_invalid_count);
  payload.add("last_latency_cycles",
              (uint32_t)g_interrupt_handoff.last_latency_cycles);
  payload.add("min_latency_cycles",
              g_interrupt_handoff.min_latency_cycles != UINT32_MAX
                  ? (uint32_t)g_interrupt_handoff.min_latency_cycles
                  : 0U);
  payload.add("max_latency_cycles",
              (uint32_t)g_interrupt_handoff.max_latency_cycles);
  payload.add("vclock_overrun_rearm_requests",
              (uint32_t)g_vclock_capture_overrun_rearm_request_count);
  payload.add("vclock_overrun_rearm_completed",
              g_vclock_capture_overrun_rearm_complete_count);
  payload.add("ocxo1_overrun_recovery_completed",
              g_ocxo1_capture_overrun_recovery_count);
  payload.add("ocxo2_overrun_recovery_completed",
              g_ocxo2_capture_overrun_recovery_count);
  payload.add("timepop_foreground_enqueue_count",
              (uint32_t)g_timepop_foreground_diag.enqueue_count);
  payload.add("timepop_foreground_dequeue_count",
              (uint32_t)g_timepop_foreground_diag.dequeue_count);
  payload.add("timepop_foreground_service_count",
              (uint32_t)g_timepop_foreground_diag.service_count);
  payload.add("timepop_foreground_overrun_count",
              (uint32_t)g_timepop_foreground_diag.overrun_count);
  payload.add("timepop_foreground_high_water",
              (uint32_t)g_timepop_foreground_diag.high_water);
  payload.add("timepop_foreground_handler_reject_count",
              (uint32_t)g_timepop_foreground_diag.handler_reject_count);
  payload.add("timepop_foreground_handler_last_ipsr",
              (uint32_t)g_timepop_foreground_diag.handler_last_ipsr);
  payload.add("timepop_foreground_scheduler_recover_count",
              (uint32_t)g_timepop_foreground_diag.scheduler_recover_count);
  const uint32_t ch2_csctrl_snapshot =
      IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].CSCTRL;
  payload.add("ch2_hw_tcf1",
              (bool)((ch2_csctrl_snapshot & TMR_CSCTRL_TCF1) != 0U));
  payload.add("ch2_hw_tcf1en",
              (bool)((ch2_csctrl_snapshot & TMR_CSCTRL_TCF1EN) != 0U));
  payload.add("ch2_hw_comp1",
              (uint32_t)IMXRT_TMR1.CH[QTIMER1_TIMEPOP_CH].COMP1);
  payload.add("ch2_compare_armed",
              (bool)g_qtimer1_ch2_compare_armed);
  payload.add("ch2_armed_target_counter32",
              (uint32_t)g_qtimer1_ch2_armed_target_counter32);
  payload.add("ch2_armed_target_low16",
              (uint32_t)g_qtimer1_ch2_armed_target_low16);
  payload.add("ch2_last_requested_target_counter32",
              (uint32_t)g_qtimer1_ch2_last_requested_target_counter32);
  payload.add("ch2_request_count",
              (uint32_t)g_qtimer1_ch2_request_count);
  payload.add("ch2_physical_arm_count",
              (uint32_t)g_qtimer1_ch2_physical_arm_count);
  payload.add("ch2_physical_reprogram_count",
              (uint32_t)g_qtimer1_ch2_physical_reprogram_count);
  payload.add("ch2_fact_outstanding",
              (bool)g_qtimer1_ch2_fact_outstanding);
  payload.add("ch2_deferred_arm_valid",
              (bool)g_qtimer1_ch2_deferred_arm_valid);
  payload.add("ch2_deferred_arm_target_counter32",
              (uint32_t)g_qtimer1_ch2_deferred_arm_target_counter32);
  payload.add("ch2_deferred_arm_count",
              (uint32_t)g_qtimer1_ch2_deferred_arm_count);
  payload.add("ch2_deferred_arm_replace_count",
              (uint32_t)g_qtimer1_ch2_deferred_arm_replace_count);
  payload.add("ch2_release_arm_count",
              (uint32_t)g_qtimer1_ch2_release_arm_count);
  payload.add("ch2_release_without_arm_count",
              (uint32_t)g_qtimer1_ch2_release_without_arm_count);
  payload.add("ch2_valid_capture_count",
              (uint32_t)g_qtimer1_ch2_valid_capture_count);
  payload.add("ch2_rejected_flag_count",
              (uint32_t)g_qtimer1_ch2_rejected_flag_count);
  payload.add("ch2_flag_while_disabled_count",
              (uint32_t)g_qtimer1_ch2_flag_while_disabled_count);
  payload.add("ch2_flag_while_unarmed_count",
              (uint32_t)g_qtimer1_ch2_flag_while_unarmed_count);
  payload.add("ch2_flag_while_outstanding_count",
              (uint32_t)g_qtimer1_ch2_flag_while_outstanding_count);
  payload.add("ch2_target_mismatch_count",
              (uint32_t)g_qtimer1_ch2_target_mismatch_count);
  payload.add("ch2_unexpected_capture_count",
              (uint32_t)g_qtimer1_ch2_unexpected_capture_count);
  add_handoff_source(payload, "vclock", g_handoff_vclock);
  add_handoff_source(payload, "ch2", g_handoff_ch2);
  add_handoff_source(payload, "ocxo1", g_handoff_ocxo1);
  add_handoff_source(payload, "ocxo2", g_handoff_ocxo2);
  add_handoff_source(payload, "pps", g_handoff_pps);
  return payload;
}

static const char* interrupt_timepop_forensic_source_name(
    interrupt_timepop_forensic_source_t source) {
  switch (source) {
    case interrupt_timepop_forensic_source_t::SELECTED_PAST_AT_ENTRY:
      return "SELECTED_PAST_AT_ENTRY";
    case interrupt_timepop_forensic_source_t::BECAME_PAST_IN_SCHEDULE_NEXT:
      return "BECAME_PAST_IN_SCHEDULE_NEXT";
    case interrupt_timepop_forensic_source_t::ARM_REQUEST_ALREADY_PAST:
      return "ARM_REQUEST_ALREADY_PAST";
    case interrupt_timepop_forensic_source_t::SCHEDULE_NEXT_PASSED:
      return "SCHEDULE_NEXT_PASSED";
    case interrupt_timepop_forensic_source_t::SCHEDULE_NEXT_TOO_CLOSE:
      return "SCHEDULE_NEXT_TOO_CLOSE";
    case interrupt_timepop_forensic_source_t::SCHEDULE_NEXT_EXPIRED:
      return "SCHEDULE_NEXT_EXPIRED";
    case interrupt_timepop_forensic_source_t::OTHER:
      return "OTHER";
    default:
      return "NONE";
  }
}

static FLASHMEM void add_forensic_record_field(
    Payload& payload,
    uint32_t index,
    const char* suffix,
    uint32_t value) {
  char key[64];
  snprintf(key, sizeof(key), "r%02lu_%s", (unsigned long)index, suffix);
  payload.add(key, value);
}

static FLASHMEM void add_forensic_record_field_i32(
    Payload& payload,
    uint32_t index,
    const char* suffix,
    int32_t value) {
  char key[64];
  snprintf(key, sizeof(key), "r%02lu_%s", (unsigned long)index, suffix);
  payload.add(key, value);
}

static FLASHMEM void add_forensic_record_field_string(
    Payload& payload,
    uint32_t index,
    const char* suffix,
    const char* value) {
  char key[64];
  snprintf(key, sizeof(key), "r%02lu_%s", (unsigned long)index, suffix);
  payload.add(key, value);
}

static FLASHMEM Payload cmd_report_forensics(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_FORENSICS");
  payload.add("architecture", "FOREGROUND_BLACK_BOX");
  payload.add("record_order", "OLDEST_TO_NEWEST");
  payload.add("gap_measurement", "DWT_AND_QTIMER10MHZ");
  payload.add("timepop_pressure_marker_api_present", true);
  payload.add("ring_size", INTERRUPT_FORENSIC_RING_SIZE);
  payload.add("service_threshold_cycles",
              INTERRUPT_FORENSIC_SERVICE_THRESHOLD_CYCLES);
  payload.add("gap_threshold_cycles",
              INTERRUPT_FORENSIC_GAP_THRESHOLD_CYCLES);
  payload.add("gap_threshold_ticks",
              INTERRUPT_FORENSIC_GAP_THRESHOLD_TICKS);
  payload.add("ch2_age_threshold_ticks",
              INTERRUPT_FORENSIC_CH2_AGE_THRESHOLD_TICKS);
  payload.add("trigger_service_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_SERVICE_LONG);
  payload.add("trigger_timepop_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_LONG);
  payload.add("trigger_subscriber_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_SUBSCRIBER_LONG);
  payload.add("trigger_feature_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_FEATURE_LONG);
  payload.add("trigger_finalize_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_FINALIZE_LONG);
  payload.add("trigger_loop_gap_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_LOOP_GAP_LONG);
  payload.add("trigger_ch2_age_long_mask",
              INTERRUPT_FORENSIC_TRIGGER_CH2_AGE_LONG);
  payload.add("trigger_timepop_pressure_mask",
              INTERRUPT_FORENSIC_TRIGGER_TIMEPOP_PRESSURE);

  payload.add("state_mailbox_pending_mask",
              INTERRUPT_FORENSIC_STATE_MAILBOX_PENDING);
  payload.add("state_ch2_fact_outstanding_mask",
              INTERRUPT_FORENSIC_STATE_CH2_FACT_OUTSTANDING);
  payload.add("state_ch2_compare_armed_mask",
              INTERRUPT_FORENSIC_STATE_CH2_COMPARE_ARMED);
  payload.add("state_ch2_deferred_valid_mask",
              INTERRUPT_FORENSIC_STATE_CH2_DEFERRED_VALID);
  payload.add("state_foreground_pending_mask",
              INTERRUPT_FORENSIC_STATE_FOREGROUND_PENDING);
  payload.add("state_rearm_requested_mask",
              INTERRUPT_FORENSIC_STATE_REARM_REQUESTED);

  uint32_t service_pass_count = 0U;
  uint32_t total_recorded = 0U;
  uint32_t overwrite_count = 0U;
  uint32_t pressure_marker_count = 0U;
  uint32_t pressure_marker_outside_service_count = 0U;
  uint32_t pressure_marker_isr_reject_count = 0U;
  uint32_t max_service_gap_cycles = 0U;
  uint32_t max_service_gap_ticks = 0U;
  uint32_t max_service_total_cycles = 0U;
  uint32_t max_timepop_cycles = 0U;
  uint32_t max_subscriber_cycles = 0U;
  uint32_t max_feature_cycles = 0U;
  uint32_t max_finalize_cycles = 0U;
  uint32_t max_ch2_event_age_abs_ticks = 0U;
  uint32_t max_ch2_capture_to_foreground_cycles = 0U;
  bool snapshot_ok = false;

  for (uint32_t attempt = 0U; attempt < 4U; ++attempt) {
    const uint32_t seq1 = g_interrupt_foreground_forensics.write_seq;
    if (seq1 & 1U) continue;
    dmb_barrier();
    service_pass_count = g_interrupt_foreground_forensics.service_pass_count;
    total_recorded = g_interrupt_foreground_forensics.total_recorded;
    overwrite_count = g_interrupt_foreground_forensics.overwrite_count;
    pressure_marker_count =
        g_interrupt_foreground_forensics.pressure_marker_count;
    pressure_marker_outside_service_count =
        g_interrupt_foreground_forensics.pressure_marker_outside_service_count;
    pressure_marker_isr_reject_count =
        g_interrupt_foreground_forensics.pressure_marker_isr_reject_count;
    max_service_gap_cycles =
        g_interrupt_foreground_forensics.max_service_gap_cycles;
    max_service_gap_ticks =
        g_interrupt_foreground_forensics.max_service_gap_ticks;
    max_service_total_cycles =
        g_interrupt_foreground_forensics.max_service_total_cycles;
    max_timepop_cycles = g_interrupt_foreground_forensics.max_timepop_cycles;
    max_subscriber_cycles =
        g_interrupt_foreground_forensics.max_subscriber_cycles;
    max_feature_cycles = g_interrupt_foreground_forensics.max_feature_cycles;
    max_finalize_cycles = g_interrupt_foreground_forensics.max_finalize_cycles;
    max_ch2_event_age_abs_ticks =
        g_interrupt_foreground_forensics.max_ch2_event_age_abs_ticks;
    max_ch2_capture_to_foreground_cycles =
        g_interrupt_foreground_forensics.max_ch2_capture_to_foreground_cycles;
    dmb_barrier();
    const uint32_t seq2 = g_interrupt_foreground_forensics.write_seq;
    if (seq1 == seq2 && (seq2 & 1U) == 0U) {
      snapshot_ok = true;
      break;
    }
  }

  const uint32_t stored = total_recorded < INTERRUPT_FORENSIC_RING_SIZE
      ? total_recorded
      : INTERRUPT_FORENSIC_RING_SIZE;
  payload.add("snapshot_ok", snapshot_ok);
  payload.add("service_pass_count", service_pass_count);
  payload.add("total_recorded", total_recorded);
  payload.add("records_stored", stored);
  payload.add("overwrite_count", overwrite_count);
  payload.add("pressure_marker_count", pressure_marker_count);
  payload.add("timepop_pressure_marker_wiring_observed",
              pressure_marker_count != 0U);
  payload.add("pressure_marker_outside_service_count",
              pressure_marker_outside_service_count);
  payload.add("pressure_marker_isr_reject_count",
              pressure_marker_isr_reject_count);
  payload.add("max_service_gap_cycles", max_service_gap_cycles);
  payload.add("max_service_gap_ticks", max_service_gap_ticks);
  payload.add("max_service_total_cycles", max_service_total_cycles);
  payload.add("max_timepop_cycles", max_timepop_cycles);
  payload.add("max_subscriber_cycles", max_subscriber_cycles);
  payload.add("max_feature_cycles", max_feature_cycles);
  payload.add("max_finalize_cycles", max_finalize_cycles);
  payload.add("max_ch2_event_age_abs_ticks", max_ch2_event_age_abs_ticks);
  payload.add("max_ch2_capture_to_foreground_cycles",
              max_ch2_capture_to_foreground_cycles);
  payload.add("live_record_active", g_interrupt_foreground_forensic_live_active);

  if (!snapshot_ok) return payload;

  const uint32_t oldest = total_recorded >= INTERRUPT_FORENSIC_RING_SIZE
      ? total_recorded % INTERRUPT_FORENSIC_RING_SIZE
      : 0U;
  for (uint32_t i = 0U; i < stored; ++i) {
    const interrupt_foreground_forensic_record_t record =
        g_interrupt_foreground_forensics.records[
            (oldest + i) % INTERRUPT_FORENSIC_RING_SIZE];
    add_forensic_record_field(payload, i, "sequence", record.sequence);
    add_forensic_record_field(payload, i, "trigger_flags",
                               record.trigger_flags);
    add_forensic_record_field(payload, i, "prior_service_exit_dwt",
                               record.prior_service_exit_dwt);
    add_forensic_record_field(payload, i,
                               "prior_service_exit_counter32",
                               record.prior_service_exit_counter32);
    add_forensic_record_field(payload, i, "service_start_dwt",
                               record.service_start_dwt);
    add_forensic_record_field(payload, i, "service_end_dwt",
                               record.service_end_dwt);
    add_forensic_record_field(payload, i, "service_gap_cycles",
                               record.service_gap_cycles);
    add_forensic_record_field(payload, i, "service_gap_ticks",
                               record.service_gap_ticks);
    add_forensic_record_field(payload, i, "service_total_cycles",
                               record.service_total_cycles);
    add_forensic_record_field(payload, i, "timepop_cycles",
                               record.timepop_cycles);
    add_forensic_record_field(payload, i, "subscriber_cycles",
                               record.subscriber_cycles);
    add_forensic_record_field(payload, i, "feature_cycles",
                               record.feature_cycles);
    add_forensic_record_field(payload, i, "finalize_cycles",
                               record.finalize_cycles);
    add_forensic_record_field(payload, i, "counter32_entry",
                               record.counter32_entry);
    add_forensic_record_field(payload, i, "counter32_exit",
                               record.counter32_exit);
    add_forensic_record_field(payload, i, "counter32_delta_ticks",
                               record.counter32_delta_ticks);
    add_forensic_record_field(payload, i, "pps_count_entry",
                               record.pps_count_entry);
    add_forensic_record_field(payload, i, "pps_count_exit",
                               record.pps_count_exit);
    add_forensic_record_field(payload, i, "state_flags_entry",
                               record.state_flags_entry);
    add_forensic_record_field(payload, i, "state_flags_exit",
                               record.state_flags_exit);
    add_forensic_record_field(payload, i, "ch2_packet_sequence",
                               record.ch2_packet_sequence);
    add_forensic_record_field(payload, i, "ch2_event_target_counter32",
                               record.ch2_event_target_counter32);
    add_forensic_record_field(payload, i, "ch2_service_counter32",
                               record.ch2_service_counter32);
    add_forensic_record_field_i32(payload, i, "ch2_event_age_ticks",
                                   record.ch2_event_age_ticks);
    add_forensic_record_field(payload, i, "ch2_event_age_abs_ticks",
                               record.ch2_event_age_abs_ticks);
    add_forensic_record_field(payload, i,
                               "ch2_capture_to_foreground_cycles",
                               record.ch2_capture_to_foreground_cycles);
    add_forensic_record_field(payload, i, "timepop_service_count_before",
                               record.timepop_service_count_before);
    add_forensic_record_field(payload, i, "timepop_service_count_after",
                               record.timepop_service_count_after);
    add_forensic_record_field(payload, i, "pressure_marker_count",
                               record.pressure_marker_count);
    add_forensic_record_field_string(
        payload, i, "pressure_source",
        interrupt_timepop_forensic_source_name(record.pressure_source));
    add_forensic_record_field(payload, i, "pressure_now_counter32",
                               record.pressure_now_counter32);
    add_forensic_record_field(payload, i, "pressure_deadline_counter32",
                               record.pressure_deadline_counter32);
    add_forensic_record_field_i32(payload, i, "pressure_signed_delta_ticks",
                                   record.pressure_signed_delta_ticks);
    add_forensic_record_field(payload, i, "pressure_slot",
                               record.pressure_slot);
    add_forensic_record_field(payload, i, "pressure_handle",
                               record.pressure_handle);
    add_forensic_record_field(payload, i, "pressure_phase",
                               record.pressure_phase);
    add_forensic_record_field(payload, i, "pressure_dwt",
                               record.pressure_dwt);
  }
  return payload;
}

static FLASHMEM Payload cmd_report_integrity(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_INTEGRITY");
  interrupt_integrity_snapshot_t snapshot{};
  (void)interrupt_integrity_snapshot(&snapshot);
  payload.add("valid", snapshot.valid);
  payload.add("vclock_counter_valid", snapshot.vclock_counter.valid);
  payload.add("vclock_counter_last_ok", snapshot.vclock_counter.last_ok);
  payload.add("ocxo1_counter_valid", snapshot.ocxo1_counter.valid);
  payload.add("ocxo1_counter_last_ok", snapshot.ocxo1_counter.last_ok);
  payload.add("ocxo1_counter_bad_count", snapshot.ocxo1_counter.bad_count);
  payload.add("ocxo2_counter_valid", snapshot.ocxo2_counter.valid);
  payload.add("ocxo2_counter_last_ok", snapshot.ocxo2_counter.last_ok);
  payload.add("ocxo2_counter_bad_count", snapshot.ocxo2_counter.bad_count);
  payload.add("vclock_dwt_valid", snapshot.vclock_qtimer_dwt.one_second.valid);
  payload.add("vclock_dwt_error_cycles",
              snapshot.vclock_qtimer_dwt.one_second.error_cycles);
  payload.add("ocxo1_dwt_valid", snapshot.ocxo1_qtimer_dwt.one_second.valid);
  payload.add("ocxo1_dwt_error_cycles",
              snapshot.ocxo1_qtimer_dwt.one_second.error_cycles);
  payload.add("ocxo2_dwt_valid", snapshot.ocxo2_qtimer_dwt.one_second.valid);
  payload.add("ocxo2_dwt_error_cycles",
              snapshot.ocxo2_qtimer_dwt.one_second.error_cycles);
  payload.add("hz1k_cadence_active", false);
  payload.add("hz1k_capture_hook", "DISABLED");
  payload.add("hz1k_rails_authored", false);
  return payload;
}

static FLASHMEM Payload cmd_report_lanes(const Payload&) {
  Payload payload;
  payload.add("report", "INTERRUPT_LANES");
  add_runtime_summary(
      payload, "vclock", runtime_for(interrupt_subscriber_kind_t::VCLOCK));
  add_ocxo_lane_report(payload, "ocxo1", g_ocxo1_ctx);
  add_ocxo_lane_report(payload, "ocxo2", g_ocxo2_ctx);
  return payload;
}

static FLASHMEM Payload cmd_report_lane(const Payload& args) {
  Payload payload;
  payload.add("report", "INTERRUPT_LANE");
  const char* lane = args.getString("lane");
  if (!lane || !*lane) lane = args.getString("name");
  if (!lane || !*lane) {
    payload.add("error", "missing lane parameter");
    return payload;
  }
  payload.add("lane", lane);
  if (interrupt_cstr_equal_ci(lane, "VCLOCK") ||
      interrupt_cstr_equal_ci(lane, "VCLK")) {
    add_runtime_summary(
        payload, "vclock", runtime_for(interrupt_subscriber_kind_t::VCLOCK));
    payload.add("heartbeat_count", g_vclock_lane.heartbeat_count);
    payload.add("one_second_count", g_vclock_lane.one_second_count);
    payload.add("last_target_counter32", g_vclock_lane.last_target_counter32);
    payload.add("last_dwt_at_edge", g_vclock_lane.last_dwt_at_edge);
    return payload;
  }
  if (interrupt_cstr_equal_ci(lane, "OCXO1") ||
      interrupt_cstr_equal_ci(lane, "O1")) {
    add_ocxo_lane_report(payload, "ocxo1", g_ocxo1_ctx);
    return payload;
  }
  if (interrupt_cstr_equal_ci(lane, "OCXO2") ||
      interrupt_cstr_equal_ci(lane, "O2")) {
    add_ocxo_lane_report(payload, "ocxo2", g_ocxo2_ctx);
    return payload;
  }
  payload.add("error", "unknown lane");
  return payload;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT", cmd_report },
  { "REPORT_STATUS", cmd_report_status },
  { "REPORT_FPU_CONTEXT", cmd_report_fpu_context },
  { "REPORT_PRIORITIES", cmd_report_priorities },
  { "REPORT_PPS", cmd_report_pps },
  { "REPORT_CADENCE", cmd_report_cadence },
  { "REPORT_SMARTZERO", cmd_report_smartzero },
  { "REPORT_HANDOFF", cmd_report_handoff },
  { "REPORT_FORENSICS", cmd_report_forensics },
  { "REPORT_INTEGRITY", cmd_report_integrity },
  { "REPORT_LANES", cmd_report_lanes },
  { "REPORT_LANE", cmd_report_lane },
  { nullptr, nullptr },
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id = "INTERRUPT",
  .commands = INTERRUPT_COMMANDS,
  .subscriptions = nullptr,
};

FLASHMEM void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1: return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2: return "OCXO2";
    case interrupt_subscriber_kind_t::TIMEPOP: return "TIMEPOP";
    default: return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1: return "QTIMER1";
    case interrupt_provider_kind_t::QTIMER2: return "QTIMER2";
    case interrupt_provider_kind_t::QTIMER3: return "QTIMER3";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default: return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH0_COMP: return "QTIMER1_CH0_COMP";
    case interrupt_lane_t::QTIMER1_CH1_COMP: return "QTIMER1_CH1_COMP";
    case interrupt_lane_t::QTIMER1_CH2_COMP: return "QTIMER1_CH2_COMP";
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER2_CH0_COMP: return "QTIMER2_CH0_COMP";
    case interrupt_lane_t::QTIMER2_CH1_COMP: return "QTIMER2_CH1_COMP";
    case interrupt_lane_t::QTIMER3_CH0_COMP: return "QTIMER3_CH0_COMP";
    case interrupt_lane_t::QTIMER3_CH1_COMP: return "QTIMER3_CH1_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE: return "GPIO_EDGE";
    default: return "NONE";
  }
}

const char* interrupt_delay_verdict_str(interrupt_delay_verdict_t verdict) {
  switch (verdict) {
    case interrupt_delay_verdict_t::ON_TIME: return "ON_TIME";
    case interrupt_delay_verdict_t::DELAYED: return "DELAYED";
    default: return "UNKNOWN";
  }
}

const char* interrupt_delay_cause_str(interrupt_delay_cause_t cause) {
  switch (cause) {
    case interrupt_delay_cause_t::NONE: return "NONE";
    case interrupt_delay_cause_t::VCLOCK_TIMEPOP: return "VCLOCK_TIMEPOP";
    case interrupt_delay_cause_t::OCXO1: return "OCXO1";
    case interrupt_delay_cause_t::OCXO2: return "OCXO2";
    case interrupt_delay_cause_t::PPS: return "PPS";
    case interrupt_delay_cause_t::CONTINUATION: return "CONTINUATION";
    case interrupt_delay_cause_t::MASKING_OR_UNKNOWN_CPU:
      return "MASKING_OR_UNKNOWN_CPU";
    case interrupt_delay_cause_t::MULTIPLE_ISR: return "MULTIPLE_ISR";
    default: return "UNKNOWN";
  }
}

const char* interrupt_delay_confidence_str(
    interrupt_delay_confidence_t confidence) {
  switch (confidence) {
    case interrupt_delay_confidence_t::LOW_CONFIDENCE: return "LOW";
    case interrupt_delay_confidence_t::MEDIUM_CONFIDENCE: return "MEDIUM";
    case interrupt_delay_confidence_t::HIGH_CONFIDENCE: return "HIGH";
    case interrupt_delay_confidence_t::EXACT_CONFIDENCE: return "EXACT";
    default: return "NONE";
  }
}
