// ============================================================================
// process_interrupt.cpp
// ============================================================================
//
// PPS and PPS_VCLOCK doctrine (this file authors both):
//
//   • PPS         — physical GPIO edge facts.  The DWT/counter32/ch3 read in
//                   the PPS GPIO ISR, latency-adjusted into event coordinates
//                   that represent the actual electrical PPS moment.  Used for
//                   diagnostics, audit, and any rail that needs the truth
//                   about the pulse itself.
//
//   • PPS_VCLOCK  — the VCLOCK edge selected by the physical PPS pulse.  This
//                   is the canonical timing authority of the system.  Its
//                   counter32/ch3 are the VCLOCK-counter identity of the
//                   chosen edge; its dwt is that same edge's DWT coordinate;
//                   its gnss_ns is VCLOCK-authored (advances by exactly 1e9
//                   per PPS) and is therefore quantized to 100 ns — the ns
//                   value always ends in "00".  PPS_VCLOCK ns is computed
//                   from VCLOCK counter ticks × 100, never from DWT.
//
//   • _raw rule   — _raw is reserved for one thing: ARM_DWT_CYCCNT captured
//                   as the first instruction of an ISR.  The moment a value
//                   is latency-adjusted, the _raw is gone.  _raw values do
//                   NOT propagate.  They live in the ISR stack frame and die
//                   there.  Nothing in a data structure, subscription payload,
//                   or TIMEBASE fragment carries _raw.
//
// OCXO subscribers (and TimePop) are the principled exception to the
// "no DWT conversion in PPS_VCLOCK" rule: they must translate their own
// ISR captures onto the PPS_VCLOCK timeline via interrupt_dwt_to_vclock_gnss_ns.
//
// Lanes:
//   VCLOCK : QTimer1 CH3, PCS=0 (10 MHz GNSS-disciplined VCLOCK)
//   OCXO1  : QTimer3 CH2, PCS=2
//   OCXO2  : QTimer3 CH3, PCS=3
//   TimePop: QTimer1 CH2, varied compare intervals (foreground scheduler)
//
// The PPS GPIO ISR captures DWT as its first instruction, then reads the
// QTimer1 cascaded counter and CH3.  When a rebootstrap is pending, it
// reprograms CH3's compare to phase-lock the VCLOCK cadence to the PPS
// moment.  After that, vclock_callback fires N seconds after PPS modulo
// consistent ISR latency.
// ============================================================================

#include "process_interrupt.h"

#include "config.h"
#include "debug.h"
#include "process.h"
#include "payload.h"
#include "time.h"
#include "timepop.h"
#include "process_timepop.h"

#include <Arduino.h>
#include "imxrt.h"
#include <math.h>
#include <strings.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr int WITNESS_SQUARE_OUT_PIN = 24;
static constexpr int WITNESS_GPIO_IN_PIN    = 26;
static constexpr int WITNESS_QTIMER_PIN     = 13;

static constexpr uint64_t HW_WITNESS_HIGH_OFFSET_NS = 250000000ULL;
static constexpr uint64_t HW_WITNESS_LOW_OFFSET_NS  = 750000000ULL;
static constexpr const char* HW_WITNESS_HIGH_NAME   = "HW_WITNESS_HIGH";
static constexpr const char* HW_WITNESS_LOW_NAME    = "HW_WITNESS_LOW";

// PPS_VCLOCK epoch selection: the canonical edge is the first VCLOCK edge
// after the physical PPS pulse.  Counter32 offset is currently 0 because
// the GPIO ISR's bus-delayed counter read is already that selected edge on
// this hardware.  Tunable named constants so live measurements can adjust.
static constexpr uint32_t VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS = 1;
static constexpr int32_t  VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS   = 0;
static constexpr int64_t  GNSS_NS_PER_SECOND                    = 1000000000LL;

// ============================================================================
// PPS / PPS_VCLOCK doctrine
// ============================================================================
//
// pps_t and pps_vclock_t are defined publicly in process_interrupt.h.
// Both structs carry only event-coordinate (latency-adjusted) values; no
// _raw is published.  They share a sequence number — they describe the
// same physical edge from two perspectives (physical truth vs canonical
// VCLOCK-selected edge).

// ============================================================================
// Snapshot store (seqlock)
// ============================================================================

struct snapshot_store_t {
  volatile uint32_t seq = 0;

  volatile uint32_t pps_sequence              = 0;
  volatile uint32_t pps_dwt_at_edge           = 0;
  volatile uint32_t pps_counter32_at_edge     = 0;
  volatile uint16_t pps_ch3_at_edge           = 0;

  volatile uint32_t pvc_sequence              = 0;
  volatile uint32_t pvc_dwt_at_edge           = 0;
  volatile uint32_t pvc_counter32_at_edge     = 0;
  volatile uint16_t pvc_ch3_at_edge           = 0;
  volatile int64_t  pvc_gnss_ns_at_edge       = -1;
};

static snapshot_store_t g_store;

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static void store_publish(const pps_t& pps, const pps_vclock_t& pvc) {
  g_store.seq++;
  dmb_barrier();
  g_store.pps_sequence          = pps.sequence;
  g_store.pps_dwt_at_edge       = pps.dwt_at_edge;
  g_store.pps_counter32_at_edge = pps.counter32_at_edge;
  g_store.pps_ch3_at_edge       = pps.ch3_at_edge;
  g_store.pvc_sequence          = pvc.sequence;
  g_store.pvc_dwt_at_edge       = pvc.dwt_at_edge;
  g_store.pvc_counter32_at_edge = pvc.counter32_at_edge;
  g_store.pvc_ch3_at_edge       = pvc.ch3_at_edge;
  g_store.pvc_gnss_ns_at_edge   = pvc.gnss_ns_at_edge;
  dmb_barrier();
  g_store.seq++;
}

static bool store_load(pps_t& pps, pps_vclock_t& pvc) {
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_store.seq;
    dmb_barrier();
    pps.sequence          = g_store.pps_sequence;
    pps.dwt_at_edge       = g_store.pps_dwt_at_edge;
    pps.counter32_at_edge = g_store.pps_counter32_at_edge;
    pps.ch3_at_edge       = g_store.pps_ch3_at_edge;
    pvc.sequence          = g_store.pvc_sequence;
    pvc.dwt_at_edge       = g_store.pvc_dwt_at_edge;
    pvc.counter32_at_edge = g_store.pvc_counter32_at_edge;
    pvc.ch3_at_edge       = g_store.pvc_ch3_at_edge;
    pvc.gnss_ns_at_edge   = g_store.pvc_gnss_ns_at_edge;
    dmb_barrier();
    const uint32_t s2 = g_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return true;
  }
  return false;
}

static pps_vclock_t store_load_pvc(void) {
  pps_t pps;
  pps_vclock_t pvc;
  store_load(pps, pvc);
  return pvc;
}

// ============================================================================
// VCLOCK GNSS epoch authoring
// ============================================================================
//
// Once the first absolute GNSS second is identified by DWT-bridge seed,
// subsequent PPS epochs advance by exactly 1e9 ns.  No DWT interpolation
// touches PPS_VCLOCK ns — the ruler IS VCLOCK.

static int64_t g_vclock_epoch_gnss_ns = -1;

static int64_t vclock_epoch_gnss_from_dwt_seed(int64_t dwt_gnss_estimate_ns) {
  if (dwt_gnss_estimate_ns < 0) return -1;
  return ((dwt_gnss_estimate_ns + GNSS_NS_PER_SECOND / 2LL) /
          GNSS_NS_PER_SECOND) * GNSS_NS_PER_SECOND;
}

static int64_t vclock_next_epoch_gnss_ns(int64_t dwt_gnss_estimate_ns) {
  if (g_vclock_epoch_gnss_ns >= 0) {
    g_vclock_epoch_gnss_ns += GNSS_NS_PER_SECOND;
    return g_vclock_epoch_gnss_ns;
  }
  g_vclock_epoch_gnss_ns = vclock_epoch_gnss_from_dwt_seed(dwt_gnss_estimate_ns);
  return g_vclock_epoch_gnss_ns;
}

static int64_t vclock_gnss_from_counter32(uint32_t authored_counter32) {
  const pps_vclock_t pvc = store_load_pvc();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) return -1;
  const uint32_t delta_ticks = authored_counter32 - pvc.counter32_at_edge;
  return pvc.gnss_ns_at_edge + (int64_t)delta_ticks * 100LL;
}

// ============================================================================
// Latency adjusters — convert raw ISR-entry DWT to event coordinates
// ============================================================================
//
// These are the ONLY producers of event-coordinate DWT values, and they
// are the ONLY place in the codebase that applies hardware latency math.
// Called exactly once per ISR, on the first-instruction _raw capture.
// All downstream code consumes the returned value as event-coordinate
// truth and applies no further adjustment.

static inline uint32_t pps_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (GPIO_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY);
}

static inline uint32_t pps_vclock_dwt_from_pps_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw + (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
}

static inline uint32_t qtimer_event_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (QTIMER_TOTAL_LATENCY - WITNESS_STIMULATE_LATENCY);
}

// ============================================================================
// Dynamic CPS — DWT cycles per GNSS second, refined on the VCLOCK cadence
// ============================================================================
//
// Anchor:    pps_vclock.dwt_at_edge  (priority-0 PPS GPIO capture, then
//                                     latency-adjusted to PPS_VCLOCK frame)
// Bookend reseed at PPS edge:  cps = pvc.dwt_at_edge - prev pvc.dwt_at_edge
// Refinement at each 1 kHz cadence tick:
//   observed_cycles = qtimer_event_dwt - pvc.dwt_at_edge
//   inferred_cps    = observed_cycles * 1e9 / gnss_offset_ns
//   blend a slew-limited fraction of (inferred_cps - cps) into cps

static volatile uint32_t g_dynamic_cps                          = 0;
static volatile bool     g_dynamic_cps_valid                    = false;
static volatile uint32_t g_dynamic_cps_pps_sequence             = 0;
static volatile uint32_t g_dynamic_cps_last_pvc_dwt_at_edge     = 0;
static volatile uint32_t g_dynamic_cps_last_reseed_value        = 0;
static volatile bool     g_dynamic_cps_last_reseed_was_computed = false;

static constexpr int64_t  DYNAMIC_CPS_MIN_REFINE_OFFSET_NS = 10000000LL;
static constexpr int64_t  DYNAMIC_CPS_MAX_REFINE_OFFSET_NS = 990000000LL;
static constexpr int32_t  DYNAMIC_CPS_MAX_STEP_CYCLES      = 128;
static constexpr uint32_t DYNAMIC_CPS_BLEND_SHIFT          = 4;

static void dynamic_cps_reset_model(void) {
  g_dynamic_cps                          = 0;
  g_dynamic_cps_valid                    = false;
  g_dynamic_cps_pps_sequence             = 0;
  g_dynamic_cps_last_pvc_dwt_at_edge     = 0;
  g_dynamic_cps_last_reseed_value        = 0;
  g_dynamic_cps_last_reseed_was_computed = false;
}

static void dynamic_cps_cadence_update(uint32_t qtimer_event_dwt,
                                       uint32_t cadence_tick_mod_1000) {
  pps_t pps; pps_vclock_t pvc;
  if (!store_load(pps, pvc)) return;
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) return;

  // PPS edge boundary: reseed cps from the two-bookend measurement.
  if (pvc.sequence != g_dynamic_cps_pps_sequence) {
    if (g_dynamic_cps_pps_sequence != 0) {
      g_dynamic_cps = pvc.dwt_at_edge - g_dynamic_cps_last_pvc_dwt_at_edge;
      g_dynamic_cps_valid = true;
      g_dynamic_cps_last_reseed_value = g_dynamic_cps;
      g_dynamic_cps_last_reseed_was_computed = true;
    } else {
      g_dynamic_cps_valid = false;
      g_dynamic_cps_last_reseed_was_computed = false;
    }
    g_dynamic_cps_last_pvc_dwt_at_edge = pvc.dwt_at_edge;
    g_dynamic_cps_pps_sequence         = pvc.sequence;
  }

  if (!g_dynamic_cps_valid || g_dynamic_cps == 0) return;

  // PPS_VCLOCK math: gnss_offset_ns derived from cadence_tick_mod_1000 × 1 ms.
  // cadence_tick_mod_1000 is the post-increment value; tick N is N ms past PPS.
  const int64_t gnss_offset_ns = (int64_t)cadence_tick_mod_1000 * 1000000LL;

  // Exact 1-second boundary belongs to the next epoch — skip it.
  if (gnss_offset_ns >= GNSS_NS_PER_SECOND) return;
  if (gnss_offset_ns <  DYNAMIC_CPS_MIN_REFINE_OFFSET_NS) return;
  if (gnss_offset_ns >  DYNAMIC_CPS_MAX_REFINE_OFFSET_NS) return;

  const uint32_t observed_cycles = qtimer_event_dwt - pvc.dwt_at_edge;
  const uint32_t inferred_cps =
      (uint32_t)(((uint64_t)observed_cycles * (uint64_t)GNSS_NS_PER_SECOND +
                  (uint64_t)gnss_offset_ns / 2ULL) /
                 (uint64_t)gnss_offset_ns);

  int32_t cps_error = (int32_t)((int64_t)inferred_cps - (int64_t)g_dynamic_cps);
  int32_t cps_step  = cps_error >> DYNAMIC_CPS_BLEND_SHIFT;
  if (cps_step == 0 && cps_error != 0) {
    cps_step = (cps_error > 0) ? 1 : -1;
  }
  if (cps_step >  DYNAMIC_CPS_MAX_STEP_CYCLES) cps_step =  DYNAMIC_CPS_MAX_STEP_CYCLES;
  if (cps_step < -DYNAMIC_CPS_MAX_STEP_CYCLES) cps_step = -DYNAMIC_CPS_MAX_STEP_CYCLES;

  g_dynamic_cps = (uint32_t)((int32_t)g_dynamic_cps + cps_step);
}

uint32_t interrupt_dynamic_cps(void) {
  return g_dynamic_cps;
}

// ============================================================================
// DWT → GNSS conversion (the OCXO/TimePop exception)
// ============================================================================

static int64_t interrupt_dwt_to_vclock_gnss_ns(uint32_t dwt_at_event) {
  const pps_vclock_t pvc = store_load_pvc();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) return -1;
  if (!g_dynamic_cps_valid) return -1;
  const uint32_t cps = g_dynamic_cps;
  if (cps == 0) return -1;

  const uint32_t dwt_delta = dwt_at_event - pvc.dwt_at_edge;
  const uint64_t ns_delta =
      ((uint64_t)dwt_delta * (uint64_t)GNSS_NS_PER_SECOND + (uint64_t)cps / 2ULL) /
      (uint64_t)cps;

  if (ns_delta > (uint64_t)GNSS_NS_PER_SECOND * 3ULL) return -1;
  return pvc.gnss_ns_at_edge + (int64_t)ns_delta;
}

// ============================================================================
// Subscriber runtime
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};
  bool subscribed = false;
  bool active = false;
  interrupt_event_t last_event {};
  interrupt_capture_diag_t last_diag {};
  bool has_fired = false;
  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t event_count = 0;
};

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK", interrupt_provider_kind_t::QTIMER1, interrupt_lane_t::QTIMER1_CH3_COMP },
  { interrupt_subscriber_kind_t::OCXO1,  "OCXO1",  interrupt_provider_kind_t::QTIMER3, interrupt_lane_t::QTIMER3_CH2_COMP },
  { interrupt_subscriber_kind_t::OCXO2,  "OCXO2",  interrupt_provider_kind_t::QTIMER3, interrupt_lane_t::QTIMER3_CH3_COMP },
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;
static bool g_interrupt_hw_ready = false;
static bool g_interrupt_runtime_ready = false;
static bool g_interrupt_irqs_enabled = false;

static volatile bool     g_pps_rebootstrap_pending = false;
static volatile uint32_t g_pps_rebootstrap_count   = 0;

static interrupt_subscriber_runtime_t* g_rt_vclock = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1  = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2  = nullptr;

static pps_edge_dispatch_fn g_pps_edge_dispatch = nullptr;

// PPS GPIO witness (for cmd_report fields).
struct pps_gpio_witness_t {
  uint32_t edge_count   = 0;
  uint32_t last_dwt     = 0;     // pps_vclock.dwt_at_edge of most recent edge
  int64_t  last_gnss_ns = 0;     // pvc.gnss_ns_at_edge of most recent edge
};
static pps_gpio_witness_t g_pps_gpio_witness;

static uint32_t g_gpio_irq_count  = 0;
static uint32_t g_gpio_miss_count = 0;

// ============================================================================
// Welford
// ============================================================================

struct welford_t {
  uint64_t n;
  double   mean;
  double   m2;
  int32_t  min_val;
  int32_t  max_val;
};

static inline void welford_reset(welford_t& w) {
  w.n = 0; w.mean = 0.0; w.m2 = 0.0; w.min_val = 0; w.max_val = 0;
}

static inline void welford_update(welford_t& w, int32_t sample) {
  w.n++;
  if (w.n == 1) { w.min_val = sample; w.max_val = sample; }
  else {
    if (sample < w.min_val) w.min_val = sample;
    if (sample > w.max_val) w.max_val = sample;
  }
  const double delta  = (double)sample - w.mean;
  w.mean += delta / (double)w.n;
  const double delta2 = (double)sample - w.mean;
  w.m2 += delta * delta2;
}

static inline double welford_stddev(const welford_t& w) {
  return (w.n < 2) ? 0.0 : sqrt(w.m2 / (double)(w.n - 1));
}

// 1 cycle = 125/126 ns at 1008 MHz (exact).
static inline double cycles_to_ns(double cycles) {
  return cycles * (125.0 / 126.0);
}

// ============================================================================
// Per-lane state
// ============================================================================

struct vclock_lane_t {
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};
static vclock_lane_t g_vclock_lane;

struct ocxo_lane_t {
  const char* name = nullptr;
  IMXRT_TMR_t* module = nullptr;
  uint8_t      channel = 0;
  uint8_t      pcs = 0;
  int          input_pin = -1;
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};
static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

// ============================================================================
// QTimer1 32-bit counter (diagnostic read with torn-read protection)
// ============================================================================

static inline uint32_t qtimer1_read_32_for_diag(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;
  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;
  return (hi1 != hi2) ? (((uint32_t)hi2 << 16) | (uint32_t)lo2)
                      : (((uint32_t)hi1 << 16) | (uint32_t)lo1);
}

uint32_t interrupt_qtimer1_counter32_now(void)   { return qtimer1_read_32_for_diag(); }
uint16_t interrupt_qtimer3_ch2_counter_now(void) { return IMXRT_TMR3.CH[2].CNTR; }
uint16_t interrupt_qtimer3_ch3_counter_now(void) { return IMXRT_TMR3.CH[3].CNTR; }

// ============================================================================
// Compare channel helpers
// ============================================================================

static inline void qtimer1_ch3_clear_compare_flag(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}
static inline void qtimer1_ch3_program_compare(uint16_t target_low16) {
  qtimer1_ch3_clear_compare_flag();
  IMXRT_TMR1.CH[3].COMP1  = target_low16;
  IMXRT_TMR1.CH[3].CMPLD1 = target_low16;
  qtimer1_ch3_clear_compare_flag();
  IMXRT_TMR1.CH[3].CSCTRL |= TMR_CSCTRL_TCF1EN;
}
static inline void qtimer1_ch3_disable_compare(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_ch3_clear_compare_flag();
}

static inline void qtimer1_ch2_clear_compare_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

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

static inline void qtimer3_clear_compare_flag(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}
static inline void qtimer3_program_compare(uint8_t ch, uint16_t target_low16) {
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].COMP1  = target_low16;
  IMXRT_TMR3.CH[ch].CMPLD1 = target_low16;
  qtimer3_clear_compare_flag(ch);
  IMXRT_TMR3.CH[ch].CSCTRL |= TMR_CSCTRL_TCF1EN;
}
static inline void qtimer3_disable_compare(uint8_t ch) {
  IMXRT_TMR3.CH[ch].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer3_clear_compare_flag(ch);
}

// ============================================================================
// Hardware witness — square-wave source + GPIO/QTimer sinks (latency probe)
// ============================================================================

static volatile bool     g_witness_square_high             = false;
static volatile uint32_t g_hw_witness_source_dwt_at_emit   = 0;
static volatile uint32_t g_hw_witness_source_emits         = 0;
static volatile uint32_t g_hw_witness_source_dwt_before    = 0;
static volatile uint32_t g_hw_witness_source_dwt_after     = 0;
static volatile uint32_t g_hw_witness_source_stim_cycles   = 0;

static volatile uint32_t g_hw_witness_gpio_dwt_at_isr      = 0;
static volatile uint32_t g_hw_witness_gpio_delta_cycles    = 0;
static volatile uint32_t g_hw_witness_gpio_hits            = 0;

static volatile uint32_t g_hw_witness_qtimer_dwt_at_isr    = 0;
static volatile uint32_t g_hw_witness_qtimer_delta_cycles  = 0;
static volatile uint32_t g_hw_witness_qtimer_hits          = 0;
static volatile uint16_t g_hw_witness_qtimer_compare_target= 0;

static welford_t g_hw_witness_gpio_welford                  = {};
static welford_t g_hw_witness_qtimer_welford                = {};
static welford_t g_hw_witness_source_stim_welford           = {};

static uint32_t g_hw_witness_gpio_last_reported_hits   = 0;
static uint32_t g_hw_witness_qtimer_last_reported_hits = 0;

enum class hw_witness_mode_t : uint8_t { BOTH = 0, GPIO, QTIMER };
static volatile hw_witness_mode_t g_hw_witness_mode = hw_witness_mode_t::BOTH;

static inline bool hw_witness_gpio_enabled(void) {
  return g_hw_witness_mode == hw_witness_mode_t::BOTH ||
         g_hw_witness_mode == hw_witness_mode_t::GPIO;
}
static inline bool hw_witness_qtimer_enabled(void) {
  return g_hw_witness_mode == hw_witness_mode_t::BOTH ||
         g_hw_witness_mode == hw_witness_mode_t::QTIMER;
}
static const char* hw_witness_mode_str(hw_witness_mode_t mode) {
  switch (mode) {
    case hw_witness_mode_t::GPIO:   return "GPIO";
    case hw_witness_mode_t::QTIMER: return "QTIMER";
    default:                        return "BOTH";
  }
}
static bool hw_witness_mode_parse(const char* s, hw_witness_mode_t& out) {
  if (!s || !*s) return false;
  if (!strcasecmp(s, "BOTH"))   { out = hw_witness_mode_t::BOTH;   return true; }
  if (!strcasecmp(s, "GPIO"))   { out = hw_witness_mode_t::GPIO;   return true; }
  if (!strcasecmp(s, "QTIMER")) { out = hw_witness_mode_t::QTIMER; return true; }
  return false;
}

static void hw_witness_apply_mode(void);

static void hw_witness_drive_high(void) {
  // Critical latency-measurement window: source DWT captured immediately
  // before the pin write.  This is the source-side _raw moment for the
  // round-trip latency measurement, and it stays inside this function.
  const uint32_t dwt_before = ARM_DWT_CYCCNT;
  digitalWriteFast(WITNESS_SQUARE_OUT_PIN, HIGH);
  const uint32_t dwt_after = ARM_DWT_CYCCNT;

  g_hw_witness_source_dwt_before  = dwt_before;
  g_hw_witness_source_dwt_after   = dwt_after;
  g_hw_witness_source_stim_cycles = dwt_after - dwt_before;
  g_hw_witness_source_dwt_at_emit = dwt_before;
  g_witness_square_high = true;
  g_hw_witness_source_emits++;
  welford_update(g_hw_witness_source_stim_welford,
                 (int32_t)g_hw_witness_source_stim_cycles);
}

static void hw_witness_drive_low(void) {
  digitalWriteFast(WITNESS_SQUARE_OUT_PIN, LOW);
  g_witness_square_high = false;
}

static void hw_witness_high_callback(timepop_ctx_t*, timepop_diag_t*, void*) { hw_witness_drive_high(); }
static void hw_witness_low_callback (timepop_ctx_t*, timepop_diag_t*, void*) { hw_witness_drive_low(); }

static void witness_gpio_isr(void) {
  if (!hw_witness_gpio_enabled()) return;
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  const int32_t sample = (int32_t)(isr_entry_dwt_raw - g_hw_witness_source_dwt_at_emit);
  g_hw_witness_gpio_dwt_at_isr   = isr_entry_dwt_raw;
  g_hw_witness_gpio_delta_cycles = (uint32_t)sample;
  g_hw_witness_gpio_hits++;
  welford_update(g_hw_witness_gpio_welford, sample);
}

static void qtimer3_witness_arm_next(void) {
  const uint16_t cntr = IMXRT_TMR2.CH[0].CNTR;
  g_hw_witness_qtimer_compare_target = (uint16_t)(cntr + 1);
  qtimer2_ch0_program_compare(g_hw_witness_qtimer_compare_target);
}

static void handle_qtimer2_witness_irq(uint32_t isr_entry_dwt_raw) {
  qtimer2_ch0_clear_compare_flag();
  const int32_t sample = (int32_t)(isr_entry_dwt_raw - g_hw_witness_source_dwt_at_emit);
  g_hw_witness_qtimer_dwt_at_isr   = isr_entry_dwt_raw;
  g_hw_witness_qtimer_delta_cycles = (uint32_t)sample;
  g_hw_witness_qtimer_hits++;
  welford_update(g_hw_witness_qtimer_welford, sample);
  qtimer3_witness_arm_next();
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR2.CH[0].CSCTRL & TMR_CSCTRL_TCF1) {
    if (!hw_witness_qtimer_enabled()) {
      qtimer2_ch0_disable_compare();
      return;
    }
    handle_qtimer2_witness_irq(isr_entry_dwt_raw);
  }
}

static void hw_witness_apply_mode(void) {
  if (hw_witness_gpio_enabled()) {
    pinMode(WITNESS_GPIO_IN_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_IN_PIN), witness_gpio_isr, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(WITNESS_GPIO_IN_PIN));
  }
  if (hw_witness_qtimer_enabled()) {
    qtimer3_witness_arm_next();
  } else {
    qtimer2_ch0_disable_compare();
  }
}

// ============================================================================
// Subscriber lookup helpers
// ============================================================================

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].desc && g_subscribers[i].desc->kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_lane;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_lane;
  return nullptr;
}

static const char* dispatch_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1:  return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2:  return "OCXO2_DISPATCH";
    default: return "";
  }
}

// ============================================================================
// Diag fill + event dispatch
// ============================================================================

static void fill_diag(interrupt_capture_diag_t& diag,
                      const interrupt_subscriber_runtime_t& rt,
                      const interrupt_event_t& event) {
  diag.enabled  = true;
  diag.provider = rt.desc->provider;
  diag.lane     = rt.desc->lane;
  diag.kind     = rt.desc->kind;
  diag.dwt_at_event       = event.dwt_at_event;
  diag.gnss_ns_at_event   = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    diag.pps_edge_sequence          = g_pps_gpio_witness.edge_count;
    diag.pps_edge_dwt_isr_entry_raw = g_pps_gpio_witness.last_dwt;  // legacy field; carries pvc.dwt_at_edge
    diag.pps_edge_gnss_ns           = g_pps_gpio_witness.last_gnss_ns;
    diag.pps_edge_minus_event_ns =
        (g_pps_gpio_witness.last_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
            ? (g_pps_gpio_witness.last_gnss_ns - (int64_t)event.gnss_ns_at_event)
            : 0;
    diag.pps_coincidence_cycles = event.pps_coincidence_cycles;
    diag.pps_coincidence_valid  = event.pps_coincidence_valid;
  }
}

static void maybe_dispatch_event(interrupt_subscriber_runtime_t& rt) {
  if (!rt.subscribed || !rt.sub.on_event) return;
  rt.sub.on_event(rt.last_event, &rt.last_diag, rt.sub.user_data);
}

static void deferred_dispatch_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;
  maybe_dispatch_event(*rt);
}

static void emit_one_second_event(interrupt_subscriber_runtime_t& rt,
                                  uint32_t dwt_at_event,
                                  uint32_t authored_counter32,
                                  uint32_t pps_coincidence_cycles = 0,
                                  bool     pps_coincidence_valid  = false) {
  if (!rt.active) return;

  interrupt_event_t event {};
  event.kind         = rt.desc->kind;
  event.provider     = rt.desc->provider;
  event.lane         = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.counter32_at_event     = authored_counter32;
  event.pps_coincidence_cycles = pps_coincidence_cycles;
  event.pps_coincidence_valid  = pps_coincidence_valid;

  // VCLOCK lane authors GNSS time directly from VCLOCK ticks (no DWT).
  // OCXO lanes use the DWT bridge — the principled exception.
  int64_t gnss_ns = -1;
  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    gnss_ns = vclock_gnss_from_counter32(authored_counter32);
  } else {
    gnss_ns = interrupt_dwt_to_vclock_gnss_ns(dwt_at_event);
  }
  event.gnss_ns_at_event = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
  event.status = interrupt_event_status_t::OK;

  interrupt_capture_diag_t diag {};
  fill_diag(diag, rt, event);

  rt.last_event = event;
  rt.last_diag  = diag;
  rt.has_fired  = true;
  rt.event_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
}

// ============================================================================
// Witness — passive observer of the production VCLOCK cadence
// ============================================================================
//
// Fires nine slots per PPS_VCLOCK second (100 ms .. 900 ms past the anchor).
// At each slot it asks: "what time does the bridge say it is, vs what time
// does VCLOCK say it is?"  The VCLOCK answer (gnss_from_vclock_ns) is ground
// truth — quantized to 100 ns, ends in "00".  The two bridge rails (time.cpp
// global bridge, and process_interrupt's local dynamic bridge) are evaluated
// against it.

static constexpr uint32_t WITNESS_N_SLOTS               = 9;
static constexpr uint32_t WITNESS_VCLOCK_TICKS_PER_SLOT = 1000000;
static constexpr int32_t  WITNESS_REJECT_CYCLES         = 100000;
static constexpr int32_t  WITNESS_ANOMALY_CYCLES        = 500;
static constexpr uint8_t  WITNESS_ANOMALY_BUFFER_SIZE   = 16;

static volatile interrupt_witness_mode_t g_witness_mode = interrupt_witness_mode_t::ON;

struct witness_capture_t {
  uint32_t seq;
  uint8_t  slot_index;
  bool     valid;

  // PPS_VCLOCK anchor (basis of the prediction).
  uint32_t pvc_dwt_at_edge;
  uint32_t pvc_counter32_at_edge;
  int64_t  pvc_gnss_ns_at_edge;
  uint32_t cps;                  // dynamic CPS used at this fire

  // Targets (predicted).
  uint32_t target_dwt;
  uint32_t target_counter32;

  // Fire (actual).
  uint32_t dwt_at_fire;
  uint32_t counter32_at_fire;

  // Residuals.
  int32_t  residual_cycles;      // dwt_at_fire - target_dwt
  int32_t  vclock_delta_ticks;   // counter32_at_fire - target_counter32

  // GNSS-domain comparison — VCLOCK ground truth ends in "00".
  int64_t  gnss_from_vclock_ns;
  int64_t  gnss_from_time_bridge_ns;
  int64_t  gnss_from_dynamic_dwt_ns;
  int64_t  gnss_time_bridge_residual_ns;
  int64_t  gnss_dynamic_residual_ns;
};

static witness_capture_t g_witness_captures[WITNESS_N_SLOTS + 1] = {};
static volatile uint32_t g_witness_capture_seq = 0;

struct witness_anomaly_t {
  witness_capture_t cap;
  uint64_t fires_total_at_capture;
};
static witness_anomaly_t g_witness_anomalies[WITNESS_ANOMALY_BUFFER_SIZE] = {};
static volatile uint8_t  g_witness_anomaly_slot_next   = 0;
static volatile uint32_t g_witness_anomaly_count_total = 0;

static welford_t g_witness_welford                       = {};
static welford_t g_witness_lag_welford                   = {};
static welford_t g_witness_gpio_counter_delay_welford    = {};
static volatile uint64_t g_witness_fires_total           = 0;
static volatile uint64_t g_witness_fires_rejected        = 0;

static inline void witness_captures_reset(void) {
  for (uint8_t i = 0; i <= WITNESS_N_SLOTS; i++) g_witness_captures[i] = {};
  g_witness_capture_seq = 0;
}

static inline void witness_anomalies_reset(void) {
  for (uint8_t i = 0; i < WITNESS_ANOMALY_BUFFER_SIZE; i++) g_witness_anomalies[i] = {};
  g_witness_anomaly_slot_next   = 0;
  g_witness_anomaly_count_total = 0;
}

static inline void witness_anomaly_maybe_capture(const witness_capture_t& cap,
                                                  uint64_t fires_total) {
  if (cap.residual_cycles >  WITNESS_ANOMALY_CYCLES ||
      cap.residual_cycles < -WITNESS_ANOMALY_CYCLES) {
    g_witness_anomaly_count_total++;
    if (g_witness_anomaly_slot_next < WITNESS_ANOMALY_BUFFER_SIZE) {
      g_witness_anomalies[g_witness_anomaly_slot_next].cap = cap;
      g_witness_anomalies[g_witness_anomaly_slot_next].fires_total_at_capture = fires_total;
      g_witness_anomaly_slot_next++;
    }
  }
}

// target_dwt[N] = anchor_dwt + round(N * cps / 10)   (slot N is at N*100ms)
static inline uint32_t witness_compute_target_dwt(uint32_t pvc_dwt_at_edge,
                                                   uint32_t cps,
                                                   uint32_t slot_index) {
  const uint64_t delta = ((uint64_t)slot_index * (uint64_t)cps + 5ULL) / 10ULL;
  return pvc_dwt_at_edge + (uint32_t)delta;
}

static inline uint32_t witness_compute_target_counter32(uint32_t pvc_counter32_at_edge,
                                                         uint32_t slot_index) {
  return pvc_counter32_at_edge + slot_index * WITNESS_VCLOCK_TICKS_PER_SLOT;
}

// Called from vclock_cadence_isr on the production cadence lane.
static void witness_cadence_observe(uint32_t qtimer_event_dwt,
                                    uint32_t authored_counter32,
                                    uint32_t cadence_tick_mod_1000) {
  if (cadence_tick_mod_1000 == 0 ||
      cadence_tick_mod_1000 >= TICKS_PER_SECOND_EVENT ||
      (cadence_tick_mod_1000 % 100) != 0) {
    return;
  }
  const uint8_t slot_index = (uint8_t)(cadence_tick_mod_1000 / 100);
  if (slot_index == 0 || slot_index > WITNESS_N_SLOTS) return;

  const pps_vclock_t pvc = store_load_pvc();
  if (pvc.sequence == 0 || pvc.gnss_ns_at_edge < 0) return;
  if (!g_dynamic_cps_valid || g_dynamic_cps == 0) return;

  const uint32_t cps              = g_dynamic_cps;
  const uint32_t target_dwt       = witness_compute_target_dwt(pvc.dwt_at_edge, cps, slot_index);
  const uint32_t target_counter32 = witness_compute_target_counter32(pvc.counter32_at_edge, slot_index);

  const int32_t  residual_cycles    = (int32_t)(qtimer_event_dwt - target_dwt);
  const int32_t  vclock_delta_ticks = (int32_t)(authored_counter32 - target_counter32);
  welford_update(g_witness_lag_welford, 0);

  // GNSS-domain comparison.  VCLOCK answer is ground truth.
  const uint32_t vclock_ticks_since_anchor = authored_counter32 - pvc.counter32_at_edge;
  const int64_t  gnss_from_vclock_ns       = pvc.gnss_ns_at_edge + (int64_t)vclock_ticks_since_anchor * 100LL;
  const int64_t  gnss_from_time_bridge_ns  = time_dwt_to_gnss_ns(qtimer_event_dwt);
  const int64_t  gnss_from_dynamic_dwt_ns  = interrupt_dwt_to_vclock_gnss_ns(qtimer_event_dwt);
  const int64_t  gnss_time_bridge_residual_ns = gnss_from_time_bridge_ns - gnss_from_vclock_ns;
  const int64_t  gnss_dynamic_residual_ns     = gnss_from_dynamic_dwt_ns - gnss_from_vclock_ns;

  g_witness_fires_total++;
  witness_capture_t& cap = g_witness_captures[slot_index];
  cap.seq                          = ++g_witness_capture_seq;
  cap.slot_index                   = slot_index;
  cap.valid                        = true;
  cap.pvc_dwt_at_edge              = pvc.dwt_at_edge;
  cap.pvc_counter32_at_edge        = pvc.counter32_at_edge;
  cap.pvc_gnss_ns_at_edge          = pvc.gnss_ns_at_edge;
  cap.cps                          = cps;
  cap.target_dwt                   = target_dwt;
  cap.target_counter32             = target_counter32;
  cap.dwt_at_fire                  = qtimer_event_dwt;
  cap.counter32_at_fire            = authored_counter32;
  cap.residual_cycles              = residual_cycles;
  cap.vclock_delta_ticks           = vclock_delta_ticks;
  cap.gnss_from_vclock_ns          = gnss_from_vclock_ns;
  cap.gnss_from_time_bridge_ns     = gnss_from_time_bridge_ns;
  cap.gnss_from_dynamic_dwt_ns     = gnss_from_dynamic_dwt_ns;
  cap.gnss_time_bridge_residual_ns = gnss_time_bridge_residual_ns;
  cap.gnss_dynamic_residual_ns     = gnss_dynamic_residual_ns;

  if (residual_cycles < -WITNESS_REJECT_CYCLES || residual_cycles > WITNESS_REJECT_CYCLES) {
    g_witness_fires_rejected++;
    witness_anomaly_maybe_capture(cap, g_witness_fires_total);
  } else {
    welford_update(g_witness_welford, residual_cycles);
  }
}

void interrupt_witness_set_mode(interrupt_witness_mode_t mode) { g_witness_mode = mode; }
interrupt_witness_mode_t interrupt_witness_get_mode(void)      { return g_witness_mode; }
void interrupt_witness_arm_first_slot(uint32_t, uint32_t, uint32_t, uint32_t) {}

interrupt_witness_stats_t interrupt_witness_stats(void) {
  interrupt_witness_stats_t s{};
  s.mode = g_witness_mode;
  s.n = g_witness_welford.n;
  s.mean_ns   = cycles_to_ns(g_witness_welford.mean);
  const double stddev_cycles = welford_stddev(g_witness_welford);
  s.stddev_ns = cycles_to_ns(stddev_cycles);
  s.stderr_ns = (s.n >= 2) ? (s.stddev_ns / sqrt((double)s.n)) : 0.0;
  s.min_ns = (int64_t)cycles_to_ns((double)g_witness_welford.min_val);
  s.max_ns = (int64_t)cycles_to_ns((double)g_witness_welford.max_val);
  s.fires_total    = g_witness_fires_total;
  s.fires_rejected = g_witness_fires_rejected;
  s.lag_n            = g_witness_lag_welford.n;
  s.lag_mean_ticks   = g_witness_lag_welford.mean;
  s.lag_stddev_ticks = welford_stddev(g_witness_lag_welford);
  s.lag_min_ticks    = g_witness_lag_welford.min_val;
  s.lag_max_ticks    = g_witness_lag_welford.max_val;
  s.gpio_counter_delay_n             = g_witness_gpio_counter_delay_welford.n;
  s.gpio_counter_delay_mean_cycles   = g_witness_gpio_counter_delay_welford.mean;
  s.gpio_counter_delay_stddev_cycles = welford_stddev(g_witness_gpio_counter_delay_welford);
  s.gpio_counter_delay_min_cycles    = g_witness_gpio_counter_delay_welford.min_val;
  s.gpio_counter_delay_max_cycles    = g_witness_gpio_counter_delay_welford.max_val;
  return s;
}

void interrupt_witness_reset_stats(void) {
  welford_reset(g_witness_welford);
  welford_reset(g_witness_lag_welford);
  welford_reset(g_witness_gpio_counter_delay_welford);
  witness_captures_reset();
  witness_anomalies_reset();
  g_witness_fires_total    = 0;
  g_witness_fires_rejected = 0;
}

uint32_t interrupt_hw_witness_gpio_delta_cycles(void)            { return g_hw_witness_gpio_delta_cycles; }
uint32_t interrupt_hw_witness_qtimer_delta_cycles(void)          { return g_hw_witness_qtimer_delta_cycles; }
uint32_t interrupt_hw_witness_qtimer_cntr(void)                  { return (uint32_t)IMXRT_TMR2.CH[0].CNTR; }
uint32_t interrupt_hw_witness_qtimer_comp1(void)                 { return (uint32_t)IMXRT_TMR2.CH[0].COMP1; }
uint32_t interrupt_hw_witness_qtimer_csctrl(void)                { return (uint32_t)IMXRT_TMR2.CH[0].CSCTRL; }
uint32_t interrupt_hw_witness_qtimer_ctrl(void)                  { return (uint32_t)IMXRT_TMR2.CH[0].CTRL; }
uint32_t interrupt_hw_witness_qtimer_enbl(void)                  { return (uint32_t)IMXRT_TMR2.ENBL; }
uint32_t interrupt_hw_witness_qtimer_mux(void)                   { return (uint32_t)*portConfigRegister(WITNESS_QTIMER_PIN); }
uint32_t interrupt_hw_witness_qtimer_select_input(void)          { return (uint32_t)IOMUXC_QTIMER2_TIMER0_SELECT_INPUT; }
uint32_t interrupt_hw_witness_qtimer_source_reads(void)          { return 0; }
uint32_t interrupt_hw_witness_qtimer_nonzero_cntr_observations(void) { return 0; }
uint32_t interrupt_hw_witness_qtimer_cntr_change_hits(void)      { return 0; }
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_at_source(void)   { return 0; }
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_in_irq(void)      { return g_hw_witness_qtimer_hits; }

// ============================================================================
// VCLOCK lane — QTimer1 CH3 cadence (1 kHz)
// ============================================================================

static void vclock_lane_arm_bootstrap(void) {
  g_vclock_lane.phase_bootstrapped = true;
  g_vclock_lane.bootstrap_count++;
  g_vclock_lane.tick_mod_1000 = 0;
  g_vclock_lane.compare_target =
      (uint16_t)(g_vclock_lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_lane_advance_compare(void) {
  g_vclock_lane.compare_target =
      (uint16_t)(g_vclock_lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_cadence_isr(uint32_t isr_entry_dwt_raw) {
  const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  g_vclock_lane.irq_count++;
  if (g_rt_vclock) g_rt_vclock->irq_count++;

  qtimer1_ch3_clear_compare_flag();

  if (!g_vclock_lane.active || !g_rt_vclock || !g_rt_vclock->active) {
    g_vclock_lane.miss_count++;
    return;
  }
  if (!g_vclock_lane.phase_bootstrapped) {
    vclock_lane_arm_bootstrap();
    g_vclock_lane.miss_count++;
    return;
  }

  vclock_lane_advance_compare();
  g_vclock_lane.cadence_hits_total++;

  const uint32_t cadence_tick_mod_1000 = g_vclock_lane.tick_mod_1000 + 1;
  const uint32_t cadence_counter32 =
      g_vclock_lane.logical_count32_at_last_second +
      cadence_tick_mod_1000 * VCLOCK_INTERVAL_COUNTS;

  dynamic_cps_cadence_update(qtimer_event_dwt, cadence_tick_mod_1000);
  witness_cadence_observe(qtimer_event_dwt, cadence_counter32, cadence_tick_mod_1000);

  if (++g_vclock_lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.logical_count32_at_last_second += VCLOCK_COUNTS_PER_SECOND;

    // PPS_VCLOCK / PPS coincidence diagnostic.
    uint32_t coincidence_cycles = 0;
    bool     coincidence_valid  = false;
    {
      const pps_vclock_t pvc = store_load_pvc();
      if (pvc.sequence > 0) {
        const int32_t cycles_since_pvc = (int32_t)(qtimer_event_dwt - pvc.dwt_at_edge);
        if (llabs((long long)cycles_since_pvc) < DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
          coincidence_cycles = (uint32_t)(cycles_since_pvc >= 0 ? cycles_since_pvc : -cycles_since_pvc);
          coincidence_valid  = true;
        }
      }
    }

    emit_one_second_event(*g_rt_vclock, qtimer_event_dwt,
                          g_vclock_lane.logical_count32_at_last_second,
                          coincidence_cycles, coincidence_valid);
  }
}

// ============================================================================
// OCXO lanes — QTimer3 CH2 / CH3
// ============================================================================

static void ocxo_lane_arm_bootstrap(ocxo_lane_t& lane) {
  lane.phase_bootstrapped = true;
  lane.bootstrap_count++;
  lane.tick_mod_1000 = 0;
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void ocxo_lane_advance_compare(ocxo_lane_t& lane) {
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void handle_ocxo_qtimer16_irq(ocxo_lane_t& lane,
                                     interrupt_subscriber_runtime_t& rt,
                                     uint32_t isr_entry_dwt_raw) {
  const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  lane.irq_count++;
  rt.irq_count++;

  qtimer3_clear_compare_flag(lane.channel);

  if (!lane.active || !rt.active) { lane.miss_count++; return; }
  if (!lane.phase_bootstrapped)   { ocxo_lane_arm_bootstrap(lane); lane.miss_count++; return; }

  ocxo_lane_advance_compare(lane);
  lane.cadence_hits_total++;

  if (++lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    lane.tick_mod_1000 = 0;
    lane.logical_count32_at_last_second += OCXO_COUNTS_PER_SECOND;
    emit_one_second_event(rt, qtimer_event_dwt, lane.logical_count32_at_last_second);
  }
}

void process_interrupt_qtimer3_ch2_irq(uint32_t isr_entry_dwt_raw) {
  if (g_rt_ocxo1) handle_ocxo_qtimer16_irq(g_ocxo1_lane, *g_rt_ocxo1, isr_entry_dwt_raw);
}

void process_interrupt_qtimer3_ch3_irq(uint32_t isr_entry_dwt_raw) {
  if (g_rt_ocxo2) handle_ocxo_qtimer16_irq(g_ocxo2_lane, *g_rt_ocxo2, isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(isr_entry_dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(isr_entry_dwt_raw);
}

// ============================================================================
// QTimer1 vector — dispatches CH2 (TimePop) and CH3 (VCLOCK cadence)
// ============================================================================

static volatile interrupt_qtimer1_ch2_handler_fn g_qtimer1_ch2_handler = nullptr;

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb) {
  g_qtimer1_ch2_handler = cb;
}

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_clear_compare_flag();
    if (g_qtimer1_ch2_handler) {
      const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
      const int64_t  gnss_ns = interrupt_dwt_to_vclock_gnss_ns(qtimer_event_dwt);

      interrupt_event_t event{};
      event.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      event.provider = interrupt_provider_kind_t::QTIMER1;
      event.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      event.status   = interrupt_event_status_t::OK;
      event.dwt_at_event       = qtimer_event_dwt;
      event.gnss_ns_at_event   = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
      event.counter32_at_event = 0;

      interrupt_capture_diag_t diag{};
      diag.enabled  = true;
      diag.provider = interrupt_provider_kind_t::QTIMER1;
      diag.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      diag.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      diag.dwt_at_event       = qtimer_event_dwt;
      diag.gnss_ns_at_event   = event.gnss_ns_at_event;
      diag.counter32_at_event = 0;

      g_qtimer1_ch2_handler(event, diag);
    }
  }
  // CH2 and CH3 are mutually exclusive per ISR invocation (NVIC re-dispatches
  // for the other channel with a fresh first-instruction _raw capture).
  else if (IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) {
    vclock_cadence_isr(isr_entry_dwt_raw);
  }
}

// ============================================================================
// PPS GPIO ISR — authors PPS and PPS_VCLOCK from the same _raw capture
// ============================================================================
//
// The _raw is the first-instruction capture.  It is converted IMMEDIATELY
// into PPS and PPS_VCLOCK event-coordinate values and never propagated.

static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_pps_edge_dispatch) return;
  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  g_pps_edge_dispatch(snap);
  if (snap.gnss_ns_at_edge >= 0) {
    timepop_cancel_by_name(HW_WITNESS_HIGH_NAME);
    timepop_cancel_by_name(HW_WITNESS_LOW_NAME);
    timepop_arm_from_anchor(snap.gnss_ns_at_edge,
                            (int64_t)HW_WITNESS_HIGH_OFFSET_NS, false,
                            hw_witness_high_callback, nullptr,
                            HW_WITNESS_HIGH_NAME);
    timepop_arm_from_anchor(snap.gnss_ns_at_edge,
                            (int64_t)HW_WITNESS_LOW_OFFSET_NS, false,
                            hw_witness_low_callback, nullptr,
                            HW_WITNESS_LOW_NAME);
  }
}

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  // Read the VCLOCK counter and CH3 right after _raw capture.  Measure the
  // peripheral-bus delay so we know the cost of the read.
  const uint32_t counter32 = qtimer1_read_32_for_diag();
  const uint16_t ch3_now   = (uint16_t)(counter32 & 0xFFFF);
  const uint32_t dwt_after_counter_reads = ARM_DWT_CYCCNT;

  welford_update(g_witness_gpio_counter_delay_welford,
                 (int32_t)(dwt_after_counter_reads - isr_entry_dwt_raw));

  g_gpio_irq_count++;
  g_pps_gpio_witness.edge_count++;

  // Author PPS facts (physical edge truth).
  pps_t pps;
  pps.sequence          = g_pps_gpio_witness.edge_count;
  pps.dwt_at_edge       = pps_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  pps.counter32_at_edge = counter32;
  pps.ch3_at_edge       = ch3_now;

  // Author PPS_VCLOCK facts (canonical VCLOCK-selected edge).
  pps_vclock_t pvc;
  pvc.sequence          = g_pps_gpio_witness.edge_count;
  pvc.dwt_at_edge       = pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);
  pvc.counter32_at_edge = counter32 + (uint32_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  pvc.ch3_at_edge       = (uint16_t)(ch3_now + (int16_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS);

  // GNSS ns at the canonical edge, VCLOCK-authored.  Use DWT only as the
  // bootstrap seed to identify the first absolute second; subsequent edges
  // advance by exactly 1e9 ns.
  const int64_t dwt_gnss_estimate_ns = time_dwt_to_gnss_ns(pvc.dwt_at_edge);
  pvc.gnss_ns_at_edge = vclock_next_epoch_gnss_ns(dwt_gnss_estimate_ns);

  g_pps_gpio_witness.last_dwt     = pvc.dwt_at_edge;
  g_pps_gpio_witness.last_gnss_ns = pvc.gnss_ns_at_edge;

  // Rebootstrap if armed by alpha.  Phase-locks the VCLOCK CH3 cadence to
  // the canonical edge.  Runs in GPIO ISR priority (0) — preempts CH3.
  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;
    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.compare_target =
        (uint16_t)(pvc.ch3_at_edge + (uint16_t)VCLOCK_INTERVAL_COUNTS);
    g_vclock_lane.logical_count32_at_last_second = pvc.counter32_at_edge;
    qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
  }

  store_publish(pps, pvc);

  if (g_pps_edge_dispatch) {
    timepop_arm_asap(pps_edge_dispatch_trampoline, nullptr, "PPS_EDGE_DISPATCH");
  }
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(isr_entry_dwt_raw);
}

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn) {
  g_pps_edge_dispatch = fn;
}

// ============================================================================
// PPS / PPS_VCLOCK accessors
// ============================================================================
//
// interrupt_last_pps()        — physical PPS GPIO edge facts.
// interrupt_last_pps_vclock() — canonical PPS_VCLOCK epoch.
// interrupt_last_pps_edge()   — legacy projection, for back-compat with
//                               consumers that still take pps_edge_snapshot_t.
//                               Field map documented inline below.

pps_t interrupt_last_pps(void) {
  pps_t pps;
  pps_vclock_t pvc;
  store_load(pps, pvc);
  return pps;
}

pps_vclock_t interrupt_last_pps_vclock(void) {
  return store_load_pvc();
}

// Legacy projection.  Field map:
//   snapshot.dwt_at_edge       <- pvc.dwt_at_edge       (PPS_VCLOCK)
//   snapshot.counter32_at_edge <- pvc.counter32_at_edge (PPS_VCLOCK)
//   snapshot.ch3_at_edge       <- pvc.ch3_at_edge       (PPS_VCLOCK)
//   snapshot.gnss_ns_at_edge   <- pvc.gnss_ns_at_edge   (PPS_VCLOCK)
//   snapshot.physical_pps_*    <- pps.*                 (physical facts)
//   snapshot.dwt_raw_at_edge   <- pvc.dwt_at_edge       (legacy alias;
//                                                        misnamed, held
//                                                        for API continuity)

pps_edge_snapshot_t interrupt_last_pps_edge(void) {
  pps_t pps;
  pps_vclock_t pvc;
  store_load(pps, pvc);

  pps_edge_snapshot_t out{};
  out.sequence          = pvc.sequence;
  out.dwt_at_edge       = pvc.dwt_at_edge;
  out.dwt_raw_at_edge   = pvc.dwt_at_edge;     // legacy alias, NOT raw
  out.counter32_at_edge = pvc.counter32_at_edge;
  out.ch3_at_edge       = pvc.ch3_at_edge;
  out.gnss_ns_at_edge   = pvc.gnss_ns_at_edge;

  out.physical_pps_dwt_raw_at_edge        = pps.dwt_at_edge;  // legacy field; carries event coord
  out.physical_pps_dwt_normalized_at_edge = pps.dwt_at_edge;
  out.physical_pps_counter32_at_read      = pps.counter32_at_edge;
  out.physical_pps_ch3_at_read            = pps.ch3_at_edge;

  out.vclock_epoch_counter32              = pvc.counter32_at_edge;
  out.vclock_epoch_ch3                    = pvc.ch3_at_edge;
  out.vclock_epoch_ticks_after_pps        = VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS;
  out.vclock_epoch_counter32_offset_ticks = VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  out.vclock_epoch_dwt_offset_cycles      = CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
  out.vclock_epoch_selected               = true;
  return out;
}

// ============================================================================
// Subscribe / start / stop / etc.
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_runtime_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE || !sub.on_event) return false;
  interrupt_subscriber_runtime_t* rt = runtime_for(sub.kind);
  if (!rt || !rt->desc) return false;
  rt->sub = sub;
  rt->subscribed = true;
  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = true;
  rt->start_count++;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = true;
    qtimer1_ch3_program_compare((uint16_t)(g_vclock_lane.compare_target + 1));
    return true;
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized) return false;
  lane->active = true;
  lane->phase_bootstrapped = false;
  lane->tick_mod_1000 = 0;
  qtimer3_program_compare(lane->channel, (uint16_t)(lane->compare_target + 1));
  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = false;
  rt->stop_count++;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    qtimer1_ch3_disable_compare();
    return true;
  }
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return false;
  lane->active = false;
  qtimer3_disable_compare(lane->channel);
  return true;
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_epoch_gnss_ns = -1;
  dynamic_cps_reset_model();
}

bool interrupt_pps_rebootstrap_pending(void) { return g_pps_rebootstrap_pending; }

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_diag;
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {}

// ============================================================================
// Hardware + runtime init
// ============================================================================

static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IMXRT_TMR1.CH[0].CTRL = 0;
  IMXRT_TMR1.CH[1].CTRL = 0;

  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[0].LOAD   = 0;
  IMXRT_TMR1.CH[0].CNTR   = 0;
  IMXRT_TMR1.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(7);

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  (void)IMXRT_TMR1.CH[0].CNTR;
  (void)IMXRT_TMR1.CH[1].HOLD;
}

static void qtimer1_init_ch2_scheduler(void) {
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
}

static void qtimer1_ch3_init_vclock_cadence(void) {
  IMXRT_TMR1.CH[3].CTRL   = 0;
  IMXRT_TMR1.CH[3].SCTRL  = 0;
  IMXRT_TMR1.CH[3].CSCTRL = 0;
  IMXRT_TMR1.CH[3].LOAD   = 0;
  IMXRT_TMR1.CH[3].CNTR   = 0;
  IMXRT_TMR1.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[3].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer1_ch3_disable_compare();
}

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR3;
  g_ocxo1_lane.channel = 2;
  g_ocxo1_lane.pcs = 2;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = 3;
  g_ocxo2_lane.pcs = 3;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  *(portConfigRegister(WITNESS_QTIMER_PIN)) = 1;
  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR2.CH[0].CTRL = 0; IMXRT_TMR2.CH[0].SCTRL = 0;
  IMXRT_TMR2.CH[0].CSCTRL = 0; IMXRT_TMR2.CH[0].LOAD = 0;
  IMXRT_TMR2.CH[0].CNTR = 0; IMXRT_TMR2.CH[0].COMP1 = 0xFFFF;
  IMXRT_TMR2.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR2.CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer2_ch0_disable_compare();

  IMXRT_TMR3.CH[2].CTRL = 0; IMXRT_TMR3.CH[2].SCTRL = 0;
  IMXRT_TMR3.CH[2].CSCTRL = 0; IMXRT_TMR3.CH[2].LOAD = 0;
  IMXRT_TMR3.CH[2].CNTR = 0; IMXRT_TMR3.CH[2].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[2].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo1_lane.pcs);
  qtimer3_disable_compare(2);

  IMXRT_TMR3.CH[3].CTRL = 0; IMXRT_TMR3.CH[3].SCTRL = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0; IMXRT_TMR3.CH[3].LOAD = 0;
  IMXRT_TMR3.CH[3].CNTR = 0; IMXRT_TMR3.CH[3].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo2_lane.pcs);
  qtimer3_disable_compare(3);

  IMXRT_TMR2.ENBL |= (uint16_t)0x000F;

  g_ocxo1_lane.initialized = true;
  g_ocxo2_lane.initialized = true;

  // QTimer1 synchronized channel start: ENBL=0 first, configure CH0/CH1/CH2/CH3
  // fully, then a single ENBL=0xF write starts all four on the same VCLOCK
  // edge — zero phase offset by construction.
  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();
  qtimer1_ch3_init_vclock_cadence();
  g_vclock_lane.initialized = true;
  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;
  IMXRT_TMR1.CH[2].CNTR = 0;
  IMXRT_TMR1.CH[3].CNTR = 0;
  IMXRT_TMR1.ENBL = 0x0F;

  g_interrupt_hw_ready = true;
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  g_subscriber_count = 0;
  for (auto& rt : g_subscribers) rt = interrupt_subscriber_runtime_t{};

  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
    rt = interrupt_subscriber_runtime_t{};
    rt.desc = &DESCRIPTORS[i];
    if      (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) g_rt_vclock = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1)  g_rt_ocxo1  = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO2)  g_rt_ocxo2  = &rt;
  }

  g_pps_gpio_witness = pps_gpio_witness_t{};
  g_gpio_irq_count = 0;
  g_gpio_miss_count = 0;
  g_vclock_epoch_gnss_ns = -1;
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;

  g_store = snapshot_store_t{};
  g_pps_edge_dispatch = nullptr;

  g_witness_square_high = false;
  g_hw_witness_source_dwt_at_emit = 0;
  g_hw_witness_source_emits = 0;
  g_hw_witness_source_dwt_before = 0;
  g_hw_witness_source_dwt_after = 0;
  g_hw_witness_source_stim_cycles = 0;
  g_hw_witness_gpio_dwt_at_isr = 0;
  g_hw_witness_gpio_delta_cycles = 0;
  g_hw_witness_gpio_hits = 0;
  g_hw_witness_qtimer_dwt_at_isr = 0;
  g_hw_witness_qtimer_delta_cycles = 0;
  g_hw_witness_qtimer_hits = 0;
  g_hw_witness_qtimer_compare_target = 0;
  welford_reset(g_hw_witness_gpio_welford);
  welford_reset(g_hw_witness_qtimer_welford);
  welford_reset(g_hw_witness_source_stim_welford);
  g_hw_witness_gpio_last_reported_hits = 0;
  g_hw_witness_qtimer_last_reported_hits = 0;
  g_hw_witness_mode = hw_witness_mode_t::BOTH;

  pinMode(WITNESS_SQUARE_OUT_PIN, OUTPUT);
  digitalWriteFast(WITNESS_SQUARE_OUT_PIN, LOW);
  hw_witness_apply_mode();

  interrupt_witness_reset_stats();
  dynamic_cps_reset_model();

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// Commands: REPORT, WITNESS_*, etc.
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count",   g_pps_rebootstrap_count);
  p.add("hw_witness_mode",            hw_witness_mode_str(g_hw_witness_mode));
  p.add("hw_witness_gpio_enabled",    hw_witness_gpio_enabled());
  p.add("hw_witness_qtimer_enabled",  hw_witness_qtimer_enabled());

  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_witness.edge_count);

  // PPS (physical edge) and PPS_VCLOCK (canonical) in symmetric blocks.
  pps_t pps; pps_vclock_t pvc;
  store_load(pps, pvc);

  p.add("pps_sequence",          pps.sequence);
  p.add("pps_dwt_at_edge",       pps.dwt_at_edge);
  p.add("pps_counter32_at_edge", pps.counter32_at_edge);
  p.add("pps_ch3_at_edge",       (uint32_t)pps.ch3_at_edge);

  p.add("pps_vclock_sequence",          pvc.sequence);
  p.add("pps_vclock_dwt_at_edge",       pvc.dwt_at_edge);
  p.add("pps_vclock_counter32_at_edge", pvc.counter32_at_edge);
  p.add("pps_vclock_ch3_at_edge",       (uint32_t)pvc.ch3_at_edge);
  p.add("pps_vclock_gnss_ns_at_edge",   pvc.gnss_ns_at_edge);

  p.add("pps_edge_dispatch_registered", g_pps_edge_dispatch != nullptr);

  p.add("vclock_irq_count",         g_vclock_lane.irq_count);
  p.add("vclock_miss_count",        g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count",   g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_compare_target",    (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_tick_mod_1000",     g_vclock_lane.tick_mod_1000);
  p.add("vclock_logical_count32",   g_vclock_lane.logical_count32_at_last_second);

  p.add("ocxo1_irq_count",          g_ocxo1_lane.irq_count);
  p.add("ocxo1_miss_count",         g_ocxo1_lane.miss_count);
  p.add("ocxo1_bootstrap_count",    g_ocxo1_lane.bootstrap_count);
  p.add("ocxo1_cadence_hits_total", g_ocxo1_lane.cadence_hits_total);
  p.add("ocxo1_compare_target",     (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_tick_mod_1000",      g_ocxo1_lane.tick_mod_1000);
  p.add("ocxo1_logical_count32",    g_ocxo1_lane.logical_count32_at_last_second);

  p.add("ocxo2_irq_count",          g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count",         g_ocxo2_lane.miss_count);
  p.add("ocxo2_bootstrap_count",    g_ocxo2_lane.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_ocxo2_lane.cadence_hits_total);
  p.add("ocxo2_compare_target",     (uint32_t)g_ocxo2_lane.compare_target);
  p.add("ocxo2_tick_mod_1000",      g_ocxo2_lane.tick_mod_1000);
  p.add("ocxo2_logical_count32",    g_ocxo2_lane.logical_count32_at_last_second);

  // Dynamic CPS — bookend reseed value + live refined value.
  p.add("dynamic_cps",                         g_dynamic_cps);
  p.add("dynamic_cps_valid",                   g_dynamic_cps_valid);
  p.add("dynamic_cps_pps_sequence",            g_dynamic_cps_pps_sequence);
  p.add("dynamic_cps_last_pvc_dwt_at_edge",    g_dynamic_cps_last_pvc_dwt_at_edge);
  p.add("dynamic_cps_last_reseed_value",       g_dynamic_cps_last_reseed_value);
  p.add("dynamic_cps_last_reseed_was_computed", g_dynamic_cps_last_reseed_was_computed);

  // Witness summary.
  const interrupt_witness_stats_t ws = interrupt_witness_stats();
  p.add("witness_active",         ws.mode == interrupt_witness_mode_t::ON);
  p.add("witness_n",              (uint32_t)ws.n);
  p.add("witness_mean_ns",        ws.mean_ns);
  p.add("witness_stddev_ns",      ws.stddev_ns);
  p.add("witness_stderr_ns",      ws.stderr_ns);
  p.add("witness_min_ns",         ws.min_ns);
  p.add("witness_max_ns",         ws.max_ns);
  p.add("witness_fires_total",    (uint32_t)ws.fires_total);
  p.add("witness_fires_rejected", (uint32_t)ws.fires_rejected);
  p.add("witness_gpio_counter_delay_n",            (uint32_t)ws.gpio_counter_delay_n);
  p.add("witness_gpio_counter_delay_mean_cycles",  ws.gpio_counter_delay_mean_cycles);
  p.add("witness_gpio_counter_delay_stddev_cycles", ws.gpio_counter_delay_stddev_cycles);
  p.add("witness_gpio_counter_delay_min_cycles",   ws.gpio_counter_delay_min_cycles);
  p.add("witness_gpio_counter_delay_max_cycles",   ws.gpio_counter_delay_max_cycles);

  return p;
}

static Payload cmd_witness_mode(const Payload& args) {
  hw_witness_mode_t mode = g_hw_witness_mode;
  const char* mode_s = args.getString("mode");
  if (!mode_s || !*mode_s) mode_s = args.getString("MODE");
  if (!hw_witness_mode_parse(mode_s, mode)) {
    Payload err;
    err.add("error", "invalid or missing mode (expected GPIO, QTIMER, or BOTH)");
    err.add("hw_witness_mode", hw_witness_mode_str(g_hw_witness_mode));
    return err;
  }
  g_hw_witness_mode = mode;
  hw_witness_apply_mode();

  Payload p;
  p.add("status", "WITNESS_MODE_SET");
  p.add("hw_witness_mode", hw_witness_mode_str(g_hw_witness_mode));
  p.add("hw_witness_gpio_enabled",   hw_witness_gpio_enabled());
  p.add("hw_witness_qtimer_enabled", hw_witness_qtimer_enabled());
  return p;
}

static Payload cmd_witness_reset(const Payload&) {
  interrupt_witness_reset_stats();
  welford_reset(g_hw_witness_gpio_welford);
  welford_reset(g_hw_witness_qtimer_welford);
  welford_reset(g_hw_witness_source_stim_welford);
  g_hw_witness_gpio_last_reported_hits = 0;
  g_hw_witness_qtimer_last_reported_hits = 0;
  hw_witness_apply_mode();
  Payload p;
  p.add("status", "WITNESS_RESET");
  return p;
}

// Serialize a witness_capture_t into a Payload.  Symmetric naming: pvc_*
// for the PPS_VCLOCK anchor.  Ground truth is gnss_from_vclock_ns (ends in 00).
static void witness_capture_to_payload(const witness_capture_t& cap,
                                       Payload& entry) {
  entry.add("slot_index", (uint32_t)cap.slot_index);
  entry.add("valid",      cap.valid);
  entry.add("seq",        cap.seq);

  entry.add("pvc_dwt_at_edge",         cap.pvc_dwt_at_edge);
  entry.add("pvc_counter32_at_edge",   cap.pvc_counter32_at_edge);
  entry.add("pvc_gnss_ns_at_edge",     cap.pvc_gnss_ns_at_edge);
  entry.add("cps",                     cap.cps);

  entry.add("target_dwt",              cap.target_dwt);
  entry.add("target_counter32",        cap.target_counter32);
  entry.add("dwt_at_fire",             cap.dwt_at_fire);
  entry.add("counter32_at_fire",       cap.counter32_at_fire);

  entry.add("residual_cycles",         cap.residual_cycles);
  entry.add("vclock_delta_ticks",      cap.vclock_delta_ticks);

  entry.add("gnss_from_vclock_ns",         cap.gnss_from_vclock_ns);
  entry.add("gnss_from_time_bridge_ns",    cap.gnss_from_time_bridge_ns);
  entry.add("gnss_from_dynamic_dwt_ns",    cap.gnss_from_dynamic_dwt_ns);
  entry.add("gnss_time_bridge_residual_ns", cap.gnss_time_bridge_residual_ns);
  entry.add("gnss_dynamic_residual_ns",     cap.gnss_dynamic_residual_ns);
}

static Payload cmd_witness_dump(const Payload&) {
  PayloadArray captures_arr;
  for (uint8_t i = 1; i <= WITNESS_N_SLOTS; i++) {
    const witness_capture_t& cap = g_witness_captures[i];
    Payload entry;
    entry.add("array_index", (uint32_t)i);
    witness_capture_to_payload(cap, entry);
    captures_arr.add(entry);
  }

  PayloadArray anomalies_arr;
  const uint8_t anomaly_count_captured = g_witness_anomaly_slot_next;
  for (uint8_t i = 0; i < anomaly_count_captured; i++) {
    const witness_anomaly_t& a = g_witness_anomalies[i];
    Payload entry;
    entry.add("buffer_index",           (uint32_t)i);
    entry.add("fires_total_at_capture", a.fires_total_at_capture);
    witness_capture_to_payload(a.cap, entry);
    anomalies_arr.add(entry);
  }

  Payload p;
  p.add("witness_active",                   g_witness_mode == interrupt_witness_mode_t::ON);
  p.add("witness_anomaly_threshold_cycles", (uint32_t)WITNESS_ANOMALY_CYCLES);
  p.add("witness_anomaly_buffer_size",      (uint32_t)WITNESS_ANOMALY_BUFFER_SIZE);
  p.add("witness_anomaly_count_total",      g_witness_anomaly_count_total);
  p.add("witness_anomaly_count_captured",   (uint32_t)anomaly_count_captured);
  p.add_array("witness_captures",  captures_arr);
  p.add_array("witness_anomalies", anomalies_arr);
  return p;
}

static Payload cmd_witness_latency(const Payload&) {
  const uint32_t gpio_hits   = g_hw_witness_gpio_hits;
  const uint32_t qtimer_hits = g_hw_witness_qtimer_hits;

  const uint32_t gpio_hits_delta   = gpio_hits   - g_hw_witness_gpio_last_reported_hits;
  const uint32_t qtimer_hits_delta = qtimer_hits - g_hw_witness_qtimer_last_reported_hits;

  const double gpio_stddev_cycles   = welford_stddev(g_hw_witness_gpio_welford);
  const double qtimer_stddev_cycles = welford_stddev(g_hw_witness_qtimer_welford);
  const double source_stim_stddev_cycles = welford_stddev(g_hw_witness_source_stim_welford);

  Payload p;
  p.add("hw_witness_mode", hw_witness_mode_str(g_hw_witness_mode));
  p.add("hw_witness_gpio_enabled",   hw_witness_gpio_enabled());
  p.add("hw_witness_qtimer_enabled", hw_witness_qtimer_enabled());

  p.add("hw_witness_source_emits",          g_hw_witness_source_emits);
  p.add("hw_witness_source_dwt_at_emit",    g_hw_witness_source_dwt_at_emit);
  p.add("hw_witness_source_dwt_before",     g_hw_witness_source_dwt_before);
  p.add("hw_witness_source_dwt_after",      g_hw_witness_source_dwt_after);
  p.add("hw_witness_source_stim_cycles",    g_hw_witness_source_stim_cycles);
  p.add("hw_witness_source_stim_ns",        cycles_to_ns((double)g_hw_witness_source_stim_cycles));
  p.add("hw_witness_source_stim_n",         (uint32_t)g_hw_witness_source_stim_welford.n);
  p.add("hw_witness_source_stim_mean_cycles",   g_hw_witness_source_stim_welford.mean);
  p.add("hw_witness_source_stim_stddev_cycles", source_stim_stddev_cycles);
  p.add("hw_witness_source_stim_min_cycles",    g_hw_witness_source_stim_welford.min_val);
  p.add("hw_witness_source_stim_max_cycles",    g_hw_witness_source_stim_welford.max_val);

  p.add("hw_witness_gpio_hits", gpio_hits);
  p.add("hw_witness_gpio_hits_delta", gpio_hits_delta);
  p.add("hw_witness_gpio_dwt_at_isr",   g_hw_witness_gpio_dwt_at_isr);
  p.add("hw_witness_gpio_delta_cycles", g_hw_witness_gpio_delta_cycles);
  p.add("hw_witness_gpio_delta_ns",     cycles_to_ns((double)g_hw_witness_gpio_delta_cycles));
  p.add("hw_witness_gpio_n",            (uint32_t)g_hw_witness_gpio_welford.n);
  p.add("hw_witness_gpio_mean_cycles",   g_hw_witness_gpio_welford.mean);
  p.add("hw_witness_gpio_stddev_cycles", gpio_stddev_cycles);
  p.add("hw_witness_gpio_min_cycles",    g_hw_witness_gpio_welford.min_val);
  p.add("hw_witness_gpio_max_cycles",    g_hw_witness_gpio_welford.max_val);
  p.add("hw_witness_gpio_mean_ns",       cycles_to_ns(g_hw_witness_gpio_welford.mean));
  p.add("hw_witness_gpio_stddev_ns",     cycles_to_ns(gpio_stddev_cycles));

  p.add("hw_witness_qtimer_hits", qtimer_hits);
  p.add("hw_witness_qtimer_hits_delta", qtimer_hits_delta);
  p.add("hw_witness_qtimer_dwt_at_isr",   g_hw_witness_qtimer_dwt_at_isr);
  p.add("hw_witness_qtimer_delta_cycles", g_hw_witness_qtimer_delta_cycles);
  p.add("hw_witness_qtimer_delta_ns",     cycles_to_ns((double)g_hw_witness_qtimer_delta_cycles));
  p.add("hw_witness_qtimer_n",            (uint32_t)g_hw_witness_qtimer_welford.n);
  p.add("hw_witness_qtimer_mean_cycles",   g_hw_witness_qtimer_welford.mean);
  p.add("hw_witness_qtimer_stddev_cycles", qtimer_stddev_cycles);
  p.add("hw_witness_qtimer_min_cycles",    g_hw_witness_qtimer_welford.min_val);
  p.add("hw_witness_qtimer_max_cycles",    g_hw_witness_qtimer_welford.max_val);
  p.add("hw_witness_qtimer_mean_ns",       cycles_to_ns(g_hw_witness_qtimer_welford.mean));
  p.add("hw_witness_qtimer_stddev_ns",     cycles_to_ns(qtimer_stddev_cycles));

  g_hw_witness_gpio_last_reported_hits   = gpio_hits;
  g_hw_witness_qtimer_last_reported_hits = qtimer_hits;
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
  { "WITNESS_RESET",   cmd_witness_reset   },
  { "WITNESS_MODE",    cmd_witness_mode    },
  { "WITNESS_DUMP",    cmd_witness_dump    },
  { "WITNESS_LATENCY", cmd_witness_latency },
  { nullptr,           nullptr             }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:  return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:   return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:   return "OCXO2";
    case interrupt_subscriber_kind_t::TIMEPOP: return "TIMEPOP";
    default:                                   return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1:  return "QTIMER1";
    case interrupt_provider_kind_t::QTIMER3:  return "QTIMER3";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH2_COMP: return "QTIMER1_CH2_COMP";
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}