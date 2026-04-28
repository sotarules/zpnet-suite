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

// ============================================================================
// Constants
// ============================================================================

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

// ============================================================================
// EDGE — PPS GPIO heartbeat
// ============================================================================
//
// State owned by the EDGE command:
//   • g_pps_gpio_heartbeat — running tally of PPS GPIO edges plus the last
//     edge's PPS_VCLOCK DWT and gnss_ns.  Updated in the PPS GPIO ISR.
//   • g_gpio_irq_count   — total PPS GPIO ISR entries.
//   • g_gpio_miss_count  — incremented when a GPIO ISR ran but did not
//     identify a PPS edge to process.
//
// The snapshot store (g_store, declared elsewhere in this file) holds the
// last pps_t and pps_vclock_t.  cmd_edge reads it via store_load() and
// publishes both views alongside the heartbeat tally.
//
// EDGE is a heartbeat plus a snapshot of "the last edge from two perspectives."

struct pps_gpio_heartbeat_t {
  uint32_t edge_count   = 0;
  uint32_t last_dwt     = 0;     // pps_vclock.dwt_at_edge of most recent edge
  int64_t  last_gnss_ns = 0;     // pvc.gnss_ns_at_edge of most recent edge
};
static pps_gpio_heartbeat_t g_pps_gpio_heartbeat;

static uint32_t g_gpio_irq_count  = 0;
static uint32_t g_gpio_miss_count = 0;

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
    diag.pps_edge_sequence          = g_pps_gpio_heartbeat.edge_count;
    diag.pps_edge_dwt_isr_entry_raw = g_pps_gpio_heartbeat.last_dwt;  // legacy field; carries pvc.dwt_at_edge
    diag.pps_edge_gnss_ns           = g_pps_gpio_heartbeat.last_gnss_ns;
    diag.pps_edge_minus_event_ns =
        (g_pps_gpio_heartbeat.last_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
            ? (g_pps_gpio_heartbeat.last_gnss_ns - (int64_t)event.gnss_ns_at_event)
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

  dynamic_cps_cadence_update(qtimer_event_dwt, cadence_tick_mod_1000);

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
}

void process_interrupt_gpio6789_irq(uint32_t isr_entry_dwt_raw) {
  // Read the VCLOCK counter and CH3 right after _raw capture.
  const uint32_t counter32 = qtimer1_read_32_for_diag();
  const uint16_t ch3_now   = (uint16_t)(counter32 & 0xFFFF);

  g_gpio_irq_count++;
  g_pps_gpio_heartbeat.edge_count++;

  // Author PPS facts (physical edge truth).
  pps_t pps;
  pps.sequence          = g_pps_gpio_heartbeat.edge_count;
  pps.dwt_at_edge       = pps_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  pps.counter32_at_edge = counter32;
  pps.ch3_at_edge       = ch3_now;

  // Author PPS_VCLOCK facts (canonical VCLOCK-selected edge).
  pps_vclock_t pvc;
  pvc.sequence          = g_pps_gpio_heartbeat.edge_count;
  pvc.dwt_at_edge       = pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);
  pvc.counter32_at_edge = counter32 + (uint32_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  pvc.ch3_at_edge       = (uint16_t)(ch3_now + (int16_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS);

  // GNSS ns at the canonical edge, VCLOCK-authored.  Use DWT only as the
  // bootstrap seed to identify the first absolute second; subsequent edges
  // advance by exactly 1e9 ns.
  const int64_t dwt_gnss_estimate_ns = time_dwt_to_gnss_ns(pvc.dwt_at_edge);
  pvc.gnss_ns_at_edge = vclock_next_epoch_gnss_ns(dwt_gnss_estimate_ns);

  g_pps_gpio_heartbeat.last_dwt     = pvc.dwt_at_edge;
  g_pps_gpio_heartbeat.last_gnss_ns = pvc.gnss_ns_at_edge;

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

  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER3_TIMER2_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

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

  g_pps_gpio_heartbeat = pps_gpio_heartbeat_t{};
  g_gpio_irq_count = 0;
  g_gpio_miss_count = 0;
  g_vclock_epoch_gnss_ns = -1;
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;

  g_store = snapshot_store_t{};
  g_pps_edge_dispatch = nullptr;

  dynamic_cps_reset_model();

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);


  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  g_interrupt_irqs_enabled = true;
}

// ============================================================================
// EDGE command
// ============================================================================
//
// Returns the PPS GPIO heartbeat plus the most recent pps_t and pps_vclock_t
// from the snapshot store.  Both views describe the same physical edge:
//   • pps        — physical electrical edge facts
//   • pps_vclock — canonical VCLOCK-selected edge (gnss_ns ends in "00")
//
// Payload shape:
//   heartbeat: { edge_count, last_dwt, last_gnss_ns,
//                gpio_irq_count, gpio_miss_count }
//   pps:        { sequence, dwt_at_edge, counter32_at_edge, ch3_at_edge }
//   pps_vclock: { sequence, dwt_at_edge, counter32_at_edge, ch3_at_edge,
//                 gnss_ns_at_edge }
//   snapshot_load_ok: bool — false if the seqlock retry loop gave up

static Payload cmd_edge(const Payload&) {
  Payload p;

  // Heartbeat sub-object.
  Payload heartbeat;
  heartbeat.add("edge_count",      g_pps_gpio_heartbeat.edge_count);
  heartbeat.add("last_dwt",        g_pps_gpio_heartbeat.last_dwt);
  heartbeat.add("last_gnss_ns",    g_pps_gpio_heartbeat.last_gnss_ns);
  heartbeat.add("gpio_irq_count",  g_gpio_irq_count);
  heartbeat.add("gpio_miss_count", g_gpio_miss_count);
  p.add_object("heartbeat", heartbeat);

  // Snapshot store: load both views in one consistent read.
  pps_t        pps_view;
  pps_vclock_t pvc_view;
  const bool snapshot_ok = store_load(pps_view, pvc_view);
  p.add("snapshot_load_ok", snapshot_ok);

  // pps sub-object — physical electrical edge facts.
  Payload pps_obj;
  pps_obj.add("sequence",          pps_view.sequence);
  pps_obj.add("dwt_at_edge",       pps_view.dwt_at_edge);
  pps_obj.add("counter32_at_edge", pps_view.counter32_at_edge);
  pps_obj.add("ch3_at_edge",       (uint32_t)pps_view.ch3_at_edge);
  p.add_object("pps", pps_obj);

  // pps_vclock sub-object — canonical VCLOCK-selected edge.
  Payload pvc_obj;
  pvc_obj.add("sequence",          pvc_view.sequence);
  pvc_obj.add("dwt_at_edge",       pvc_view.dwt_at_edge);
  pvc_obj.add("counter32_at_edge", pvc_view.counter32_at_edge);
  pvc_obj.add("ch3_at_edge",       (uint32_t)pvc_view.ch3_at_edge);
  pvc_obj.add("gnss_ns_at_edge",   pvc_view.gnss_ns_at_edge);
  p.add_object("pps_vclock", pvc_obj);

  return p;
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count",   g_pps_rebootstrap_count);
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);

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

  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
  { "EDGE",            cmd_edge            },
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