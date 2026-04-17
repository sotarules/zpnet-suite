// ============================================================================
// process_interrupt.cpp — rolling DWT integration + physical PPS machinery
// ============================================================================
#
// Three rolling integrators, one per lane, each cadenced by its own
// dedicated QuadTimer compare channel at 1 kHz:
#
//   VCLOCK lane:  QTimer1 CH3 compare, +10000 ticks per interval
//                 (clocked by GNSS 10 MHz VCLOCK; PCS=0)
//                 ISR is registered with TimePop via
//                 timepop_register_qtimer1_ch3_isr() — TimePop's
//                 qtimer1_irq_isr captures DWT as the first instruction
//                 and dispatches here when CH3's flag is set.
#
//   OCXO1 lane:   QTimer3 CH2 compare, +10000 ticks per interval
//                 (clocked by OCXO1 10 MHz; PCS=2)
//                 ISR is owned directly by process_interrupt.
#
//   OCXO2 lane:   QTimer3 CH3 compare, +10000 ticks per interval
//                 (clocked by OCXO2 10 MHz; PCS=3)
//                 ISR is owned directly by process_interrupt.
#
// All three lanes have the SAME hardware-to-software latency profile:
// QuadTimer compare match → NVIC → ISR → DWT capture as first instruction.
#
// VCLOCK is the authoritative GNSS clock — it drives the bridge anchor
// via alpha's vclock_callback.  Its subscribers receive boundary events
// through the interrupt_subscribe / interrupt_start machinery.
#
// Physical PPS GPIO — witness and dispatch authority:
#
//   The physical PPS GPIO edge plays two narrow roles:
#
//     1. WITNESS.  The GPIO ISR captures DWT as its first instruction,
//        translates to GNSS ns via the bridge, stores a seqlock-
//        protected pps_edge_snapshot_t.  Callers read the snapshot via
//        interrupt_last_pps_edge().
#
//     2. DISPATCH.  The GPIO ISR arms timepop_arm_asap to invoke a
//        single registered dispatch callback (alpha's pps_edge_callback)
//        in foreground context.  That callback publishes TIMEBASE_FRAGMENT.
#
//   The physical edge does NOT drive the GNSS clock.  GNSS clock state
//   is maintained by VCLOCK boundary events via alpha's vclock_callback,
//   as before.
#
// Per-interval instrumentation:
#
//   Each integrator also maintains:
//     • interval_min_ever / interval_max_ever — all-time extremes across
//       the ring (updated in the ISR on every tick; catches rare
//       excursions the ring slides past).
//     • compute_ring_interval_stats() — walks the ring to produce
//       window min/max/mean/stddev.  Called from fill_diag() at boundary
//       emission (so the stats ride in TIMEBASE_FRAGMENT diag fields)
//       and from add_integrator_report() for the REPORT command.
#
//   Why: integ_diff and raw_diff both telescope to the same pair of
//   endpoints, so neither can see the 999 intermediate ticks' jitter.
//   The per-interval stats ARE NOT blind to that jitter — they reveal
//   what the integrator's SMA has been smoothing all along.
#
// See process_interrupt.h for the boundary event and snapshot contracts.
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

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;
static constexpr uint64_t NS_PER_SECOND_U64 = 1000000000ULL;

static constexpr uint32_t VCLOCK_INTERVAL_COUNTS = 10000U;  // 1 ms at 10 MHz

static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000U;
static constexpr uint32_t OCXO_INTERVAL_COUNTS = 10000U;    // 1 ms at 10 MHz

static constexpr uint32_t ROLLING_WINDOW_SIZE = 1000U;      // 1000 × 1ms = 1 second

static constexpr int OCXO1_PIN = 14;
static constexpr int OCXO2_PIN = 15;

// ============================================================================
// Rolling DWT integrator — per-lane state
// ============================================================================

struct rolling_dwt_integrator_t {
  const char* name = nullptr;

  bool     baseline_valid = false;
  uint32_t baseline_dwt = 0;

  uint32_t prev_dwt = 0;
  uint64_t total_ticks_seen = 0;
  uint32_t last_interval_cycles = 0;

  uint32_t ring[ROLLING_WINDOW_SIZE] = {};
  uint32_t ring_head = 0;
  uint32_t ring_fill = 0;
  uint64_t ring_sum = 0;

  uint64_t avg_cycles_per_interval_q32 = 0;

  bool     synthetic_valid = false;
  uint32_t last_synthetic_dwt = 0;
  uint32_t boundary_emissions = 0;

  uint32_t interval_min_ever = UINT32_MAX;
  uint32_t interval_max_ever = 0;

  bool boundary_emitted_this_tick = false;
};

static rolling_dwt_integrator_t g_vclock_integrator;
static rolling_dwt_integrator_t g_ocxo1_integrator;
static rolling_dwt_integrator_t g_ocxo2_integrator;

static void rolling_integrator_tick(rolling_dwt_integrator_t& r, uint32_t dwt_raw) {
  r.boundary_emitted_this_tick = false;

  if (!r.baseline_valid) {
    r.baseline_dwt = dwt_raw;
    r.prev_dwt = dwt_raw;
    r.last_synthetic_dwt = dwt_raw;
    r.total_ticks_seen = 0;
    r.interval_min_ever = UINT32_MAX;
    r.interval_max_ever = 0;
    r.baseline_valid = true;
    return;
  }

  const uint32_t interval = dwt_raw - r.prev_dwt;
  r.prev_dwt = dwt_raw;
  r.last_interval_cycles = interval;
  r.total_ticks_seen++;

  if (interval < r.interval_min_ever) r.interval_min_ever = interval;
  if (interval > r.interval_max_ever) r.interval_max_ever = interval;

  if (r.ring_fill == ROLLING_WINDOW_SIZE) {
    r.ring_sum -= r.ring[r.ring_head];
  } else {
    r.ring_fill++;
  }
  r.ring[r.ring_head] = interval;
  r.ring_sum += interval;
  r.ring_head = (r.ring_head + 1u) % ROLLING_WINDOW_SIZE;

  if (r.ring_fill > 0) {
    r.avg_cycles_per_interval_q32 = (r.ring_sum << 32) / r.ring_fill;
  }

  if ((r.total_ticks_seen % ROLLING_WINDOW_SIZE) == 0u &&
      r.ring_fill == ROLLING_WINDOW_SIZE) {
    const uint64_t boundary_number = r.total_ticks_seen / ROLLING_WINDOW_SIZE;
    const uint64_t cumulative_cycles = boundary_number * r.ring_sum;

    r.last_synthetic_dwt = r.baseline_dwt + (uint32_t)cumulative_cycles;
    r.boundary_emissions++;
    r.synthetic_valid = true;
    r.boundary_emitted_this_tick = true;
  }
}

static uint64_t integrator_cycles_per_second(const rolling_dwt_integrator_t& r) {
  if (!r.baseline_valid || r.ring_fill == 0) return 0;
  return (r.avg_cycles_per_interval_q32 * (uint64_t)ROLLING_WINDOW_SIZE) >> 32;
}

uint64_t interrupt_vclock_cycles_per_second(void) {
  return integrator_cycles_per_second(g_vclock_integrator);
}

static void compute_ring_interval_stats(const rolling_dwt_integrator_t& r,
                                        uint32_t& out_min,
                                        uint32_t& out_max,
                                        double&   out_mean,
                                        double&   out_stddev) {
  const uint32_t fill = r.ring_fill;
  if (fill == 0) {
    out_min = 0;
    out_max = 0;
    out_mean = 0.0;
    out_stddev = 0.0;
    return;
  }

  uint32_t mn = UINT32_MAX;
  uint32_t mx = 0;
  uint64_t sum = 0;
  for (uint32_t i = 0; i < fill; i++) {
    const uint32_t v = r.ring[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }

  const double mean = (double)sum / (double)fill;
  double sq_sum = 0.0;
  for (uint32_t i = 0; i < fill; i++) {
    const double d = (double)r.ring[i] - mean;
    sq_sum += d * d;
  }

  out_min = mn;
  out_max = mx;
  out_mean = mean;
  out_stddev = (fill >= 2) ? sqrt(sq_sum / (double)(fill - 1)) : 0.0;
}

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
static volatile bool g_pps_zero_pending = false;

static interrupt_subscriber_runtime_t* g_rt_vclock = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo1  = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2  = nullptr;

struct vclock_lane_t {
  bool     initialized = false;
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;
  uint32_t logical_count32_at_last_boundary = 0;
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
  uint32_t logical_count32_at_last_boundary = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
};

static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

struct pps_gpio_witness_t {
  uint32_t edge_count = 0;
  uint32_t last_dwt = 0;
  int64_t  last_gnss_ns = 0;
};

static pps_gpio_witness_t g_pps_gpio_witness;

static uint32_t g_gpio_irq_count = 0;
static uint32_t g_gpio_miss_count = 0;

struct pps_edge_snapshot_store_t {
  volatile uint32_t seq             = 0;
  volatile uint32_t sequence        = 0;
  volatile uint32_t dwt_at_edge     = 0;
  volatile int64_t  gnss_ns_at_edge = -1;
};

static pps_edge_snapshot_store_t g_pps_edge_store;
static pps_edge_dispatch_fn      g_pps_edge_dispatch = nullptr;

static inline void dmb_barrier(void) {
  __asm__ volatile ("dmb" ::: "memory");
}

static uint32_t qtimer1_read_32_for_diag(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;
  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;
  return (hi1 != hi2) ? (((uint32_t)hi2 << 16) | (uint32_t)lo2)
                      : (((uint32_t)hi1 << 16) | (uint32_t)lo1);
}

uint32_t interrupt_qtimer1_counter32_now(void) { return qtimer1_read_32_for_diag(); }
uint16_t interrupt_qtimer3_ch2_counter_now(void) { return IMXRT_TMR3.CH[2].CNTR; }
uint16_t interrupt_qtimer3_ch3_counter_now(void) { return IMXRT_TMR3.CH[3].CNTR; }

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

static rolling_dwt_integrator_t* integrator_for(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return &g_vclock_integrator;
    case interrupt_subscriber_kind_t::OCXO1:  return &g_ocxo1_integrator;
    case interrupt_subscriber_kind_t::OCXO2:  return &g_ocxo2_integrator;
    default: return nullptr;
  }
}

static const char* dispatch_timer_name(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK: return "VCLOCK_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO1:  return "OCXO1_DISPATCH";
    case interrupt_subscriber_kind_t::OCXO2:  return "OCXO2_DISPATCH";
    default: return "";
  }
}

static void fill_diag(interrupt_capture_diag_t& diag,
                      const interrupt_subscriber_runtime_t& rt,
                      const interrupt_event_t& event,
                      uint32_t dwt_isr_entry_raw,
                      const rolling_dwt_integrator_t& r) {
  diag.enabled = true;
  diag.provider = rt.desc->provider;
  diag.lane = rt.desc->lane;
  diag.kind = rt.desc->kind;

  diag.boundary_dwt_isr_entry_raw = dwt_isr_entry_raw;
  const int64_t isr_gnss_ns = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
  diag.boundary_dwt_isr_entry_gnss_ns = isr_gnss_ns;
  diag.boundary_dwt_isr_entry_minus_event_ns =
      (isr_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
          ? (isr_gnss_ns - (int64_t)event.gnss_ns_at_event)
          : 0;

  diag.dwt_at_event = event.dwt_at_event;
  diag.gnss_ns_at_event = event.gnss_ns_at_event;
  diag.counter32_at_event = event.counter32_at_event;

  diag.integrator_baseline_valid = r.baseline_valid;
  diag.integrator_baseline_dwt = r.baseline_dwt;
  diag.integrator_total_ticks = r.total_ticks_seen;
  diag.integrator_ring_fill = r.ring_fill;
  diag.integrator_ring_sum = r.ring_sum;
  diag.integrator_avg_cycles_per_sec = integrator_cycles_per_second(r);
  diag.integrator_last_interval_cycles = r.last_interval_cycles;
  diag.integrator_boundary_emissions = r.boundary_emissions;

  {
    uint32_t window_min = 0, window_max = 0;
    double   window_mean = 0.0, window_stddev = 0.0;
    compute_ring_interval_stats(r, window_min, window_max, window_mean, window_stddev);
    diag.integrator_interval_window_min_cycles = window_min;
    diag.integrator_interval_window_max_cycles = window_max;
    diag.integrator_interval_window_mean_cycles = window_mean;
    diag.integrator_interval_window_stddev_cycles = window_stddev;
    diag.integrator_interval_min_ever_cycles =
        (r.interval_min_ever == UINT32_MAX) ? 0u : r.interval_min_ever;
    diag.integrator_interval_max_ever_cycles = r.interval_max_ever;
  }

  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    diag.pps_edge_sequence = g_pps_gpio_witness.edge_count;
    diag.pps_edge_dwt_isr_entry_raw = g_pps_gpio_witness.last_dwt;
    diag.pps_edge_gnss_ns = g_pps_gpio_witness.last_gnss_ns;
    diag.pps_edge_minus_event_ns =
        (g_pps_gpio_witness.last_gnss_ns >= 0 && event.gnss_ns_at_event != 0)
            ? (g_pps_gpio_witness.last_gnss_ns - (int64_t)event.gnss_ns_at_event)
            : 0;
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
  if (rt->desc->kind == interrupt_subscriber_kind_t::VCLOCK && g_pps_zero_pending) {
    g_pps_zero_pending = false;
  }
}

static void emit_boundary_event(interrupt_subscriber_runtime_t& rt,
                                uint32_t dwt_isr_entry_raw,
                                uint32_t authored_counter32) {
  if (!rt.active) return;

  rolling_dwt_integrator_t* r = integrator_for(rt.desc->kind);
  if (!r) return;

  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.dwt_at_event = r->last_synthetic_dwt;
  event.counter32_at_event = authored_counter32;

  const int64_t gnss_ns = time_dwt_to_gnss_ns(r->last_synthetic_dwt);
  if (gnss_ns >= 0) {
    event.gnss_ns_at_event = (uint64_t)gnss_ns;
    event.status = interrupt_event_status_t::OK;
  } else {
    event.gnss_ns_at_event = 0;
    event.status = interrupt_event_status_t::OK;
  }

  interrupt_capture_diag_t diag {};
  fill_diag(diag, rt, event, dwt_isr_entry_raw, *r);

  rt.last_event = event;
  rt.last_diag = diag;
  rt.has_fired = true;
  rt.event_count++;
  rt.dispatch_count++;

  timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
}

static void vclock_lane_arm_bootstrap(void) {
  const uint16_t now16 = IMXRT_TMR1.CH[3].CNTR;
  g_vclock_lane.phase_bootstrapped = true;
  g_vclock_lane.bootstrap_count++;
  g_vclock_lane.compare_target = (uint16_t)(now16 + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_lane_advance_compare(void) {
  g_vclock_lane.compare_target =
      (uint16_t)(g_vclock_lane.compare_target + (uint16_t)VCLOCK_INTERVAL_COUNTS);
  qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
}

static void vclock_integrator_isr(uint32_t dwt_raw) {
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

  rolling_integrator_tick(g_vclock_integrator, dwt_raw);

  if (g_vclock_integrator.boundary_emitted_this_tick) {
    g_vclock_lane.logical_count32_at_last_boundary += VCLOCK_COUNTS_PER_SECOND;
    emit_boundary_event(*g_rt_vclock, dwt_raw,
                        g_vclock_lane.logical_count32_at_last_boundary);
  }
}

static void ocxo_lane_arm_bootstrap(ocxo_lane_t& lane) {
  const uint16_t now16 = lane.module->CH[lane.channel].CNTR;
  lane.phase_bootstrapped = true;
  lane.bootstrap_count++;
  lane.compare_target = (uint16_t)(now16 + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void ocxo_lane_advance_compare(ocxo_lane_t& lane) {
  lane.compare_target = (uint16_t)(lane.compare_target + (uint16_t)OCXO_INTERVAL_COUNTS);
  qtimer3_program_compare(lane.channel, lane.compare_target);
}

static void handle_ocxo_qtimer16_irq(ocxo_lane_t& lane,
                                     interrupt_subscriber_runtime_t& rt,
                                     rolling_dwt_integrator_t& integrator,
                                     uint32_t dwt_raw) {
  lane.irq_count++;
  rt.irq_count++;

  qtimer3_clear_compare_flag(lane.channel);

  if (!lane.active || !rt.active) {
    lane.miss_count++;
    return;
  }

  if (!lane.phase_bootstrapped) {
    ocxo_lane_arm_bootstrap(lane);
    lane.miss_count++;
    return;
  }

  ocxo_lane_advance_compare(lane);
  lane.cadence_hits_total++;

  rolling_integrator_tick(integrator, dwt_raw);

  if (integrator.boundary_emitted_this_tick) {
    lane.logical_count32_at_last_boundary += OCXO_COUNTS_PER_SECOND;
    emit_boundary_event(rt, dwt_raw, lane.logical_count32_at_last_boundary);
  }
}

void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_raw) {
  if (g_rt_ocxo1) {
    handle_ocxo_qtimer16_irq(g_ocxo1_lane, *g_rt_ocxo1, g_ocxo1_integrator, dwt_raw);
  }
}

void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_raw) {
  if (g_rt_ocxo2) {
    handle_ocxo_qtimer16_irq(g_ocxo2_lane, *g_rt_ocxo2, g_ocxo2_integrator, dwt_raw);
  }
}

static void qtimer3_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  if (IMXRT_TMR3.CH[2].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch2_irq(dwt_raw);
  if (IMXRT_TMR3.CH[3].CSCTRL & TMR_CSCTRL_TCF1) process_interrupt_qtimer3_ch3_irq(dwt_raw);
}

static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*) {
  if (!g_pps_edge_dispatch) return;
  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  g_pps_edge_dispatch(snap);
}

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw) {
  g_gpio_irq_count++;
  g_pps_gpio_witness.edge_count++;
  g_pps_gpio_witness.last_dwt = dwt_isr_entry_raw;

  const int64_t gnss_ns = time_dwt_to_gnss_ns(dwt_isr_entry_raw);
  g_pps_gpio_witness.last_gnss_ns = gnss_ns;

  g_pps_edge_store.seq++;
  dmb_barrier();
  g_pps_edge_store.sequence        = g_pps_gpio_witness.edge_count;
  g_pps_edge_store.dwt_at_edge     = dwt_isr_entry_raw;
  g_pps_edge_store.gnss_ns_at_edge = gnss_ns;
  dmb_barrier();
  g_pps_edge_store.seq++;

  if (g_pps_edge_dispatch) {
    timepop_arm_asap(pps_edge_dispatch_trampoline, nullptr, "PPS_EDGE_DISPATCH");
  }
}

static void pps_gpio_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  process_interrupt_gpio6789_irq(dwt_raw);
}

void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn) {
  g_pps_edge_dispatch = fn;
}

pps_edge_snapshot_t interrupt_last_pps_edge(void) {
  pps_edge_snapshot_t out {};
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_pps_edge_store.seq;
    dmb_barrier();
    out.sequence        = g_pps_edge_store.sequence;
    out.dwt_at_edge     = g_pps_edge_store.dwt_at_edge;
    out.gnss_ns_at_edge = g_pps_edge_store.gnss_ns_at_edge;
    dmb_barrier();
    const uint32_t s2 = g_pps_edge_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out;
  }
  return out;
}

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
    g_vclock_lane.phase_bootstrapped = false;
    qtimer1_ch3_program_compare((uint16_t)(IMXRT_TMR1.CH[3].CNTR + 1));
    return true;
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized) return false;
  lane->active = true;
  lane->phase_bootstrapped = false;
  qtimer3_program_compare(lane->channel,
                          (uint16_t)(lane->module->CH[lane->channel].CNTR + 1));
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

void interrupt_request_pps_zero(void) { g_pps_zero_pending = true; }
bool interrupt_pps_zero_pending(void) { return g_pps_zero_pending; }

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_event;
}

const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  return (!rt || !rt->has_fired) ? nullptr : &rt->last_diag;
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {}

static void qtimer1_ch3_init_vclock_integrator(void) {
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

  qtimer1_ch3_init_vclock_integrator();
  g_vclock_lane.initialized = true;

  g_interrupt_hw_ready = true;
}

void process_interrupt_init(void) {
  if (g_interrupt_runtime_ready) return;

  g_vclock_integrator = rolling_dwt_integrator_t{};
  g_vclock_integrator.name = "VCLOCK";
  g_ocxo1_integrator = rolling_dwt_integrator_t{};
  g_ocxo1_integrator.name = "OCXO1";
  g_ocxo2_integrator = rolling_dwt_integrator_t{};
  g_ocxo2_integrator.name = "OCXO2";

  g_subscriber_count = 0;
  for (auto& rt : g_subscribers) rt = interrupt_subscriber_runtime_t{};

  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
    rt = interrupt_subscriber_runtime_t{};
    rt.desc = &DESCRIPTORS[i];
    if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) g_rt_vclock = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1) g_rt_ocxo1 = &rt;
    else if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) g_rt_ocxo2 = &rt;
  }

  g_pps_gpio_witness = pps_gpio_witness_t{};
  g_gpio_irq_count = 0;
  g_gpio_miss_count = 0;
  g_pps_zero_pending = false;

  g_pps_edge_store = pps_edge_snapshot_store_t{};
  g_pps_edge_dispatch = nullptr;

  g_interrupt_runtime_ready = true;
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  timepop_register_qtimer1_ch3_isr(vclock_integrator_isr);

  g_interrupt_irqs_enabled = true;
}

static void add_integrator_report(Payload& p, const char* prefix,
                                  const rolling_dwt_integrator_t& r) {
  char key[96];
  auto keyf = [&](const char* suffix) -> const char* {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    return key;
  };
  p.add(keyf("baseline_valid"), r.baseline_valid);
  p.add(keyf("baseline_dwt"), r.baseline_dwt);
  p.add(keyf("total_ticks_seen"), r.total_ticks_seen);
  p.add(keyf("ring_fill"), r.ring_fill);
  p.add(keyf("ring_sum"), r.ring_sum);
  p.add(keyf("avg_cycles_per_sec"), integrator_cycles_per_second(r));
  p.add(keyf("last_interval_cycles"), r.last_interval_cycles);
  p.add(keyf("boundary_emissions"), r.boundary_emissions);
  p.add(keyf("synthetic_valid"), r.synthetic_valid);
  p.add(keyf("last_synthetic_dwt"), r.last_synthetic_dwt);

  uint32_t window_min, window_max;
  double   window_mean, window_stddev;
  compute_ring_interval_stats(r, window_min, window_max, window_mean, window_stddev);
  p.add(keyf("interval_window_min_cycles"), window_min);
  p.add(keyf("interval_window_max_cycles"), window_max);
  p.add(keyf("interval_window_range_cycles"),
        (window_max >= window_min) ? (window_max - window_min) : 0u);
  p.add(keyf("interval_window_mean_cycles"), window_mean);
  p.add(keyf("interval_window_stddev_cycles"), window_stddev);
  p.add(keyf("interval_min_ever_cycles"),
        (r.interval_min_ever == UINT32_MAX) ? 0u : r.interval_min_ever);
  p.add(keyf("interval_max_ever_cycles"), r.interval_max_ever);
}

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready", g_interrupt_runtime_ready);
  p.add("irqs_enabled", g_interrupt_irqs_enabled);
  p.add("subscriber_count", g_subscriber_count);
  p.add("pps_zero_pending", g_pps_zero_pending);

  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_witness.edge_count);
  p.add("gpio_last_dwt", g_pps_gpio_witness.last_dwt);
  p.add("pps_edge_gnss_ns", g_pps_gpio_witness.last_gnss_ns);

  const pps_edge_snapshot_t snap = interrupt_last_pps_edge();
  p.add("pps_edge_sequence", snap.sequence);
  p.add("pps_edge_dwt", snap.dwt_at_edge);
  p.add("pps_edge_gnss_ns_snapshot", snap.gnss_ns_at_edge);
  p.add("pps_edge_dispatch_registered", g_pps_edge_dispatch != nullptr);

  add_integrator_report(p, "vclock", g_vclock_integrator);
  add_integrator_report(p, "ocxo1", g_ocxo1_integrator);
  add_integrator_report(p, "ocxo2", g_ocxo2_integrator);

  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count", g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_compare_target", (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_logical_count32", g_vclock_lane.logical_count32_at_last_boundary);

  p.add("ocxo1_irq_count", g_ocxo1_lane.irq_count);
  p.add("ocxo1_miss_count", g_ocxo1_lane.miss_count);
  p.add("ocxo1_bootstrap_count", g_ocxo1_lane.bootstrap_count);
  p.add("ocxo1_cadence_hits_total", g_ocxo1_lane.cadence_hits_total);
  p.add("ocxo1_compare_target", (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_logical_count32", g_ocxo1_lane.logical_count32_at_last_boundary);

  p.add("ocxo2_irq_count", g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count", g_ocxo2_lane.miss_count);
  p.add("ocxo2_bootstrap_count", g_ocxo2_lane.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_ocxo2_lane.cadence_hits_total);
  p.add("ocxo2_compare_target", (uint32_t)g_ocxo2_lane.compare_target);
  p.add("ocxo2_logical_count32", g_ocxo2_lane.logical_count32_at_last_boundary);

  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::VCLOCK:    return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:     return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:     return "OCXO2";
    case interrupt_subscriber_kind_t::TIME_TEST: return "TIME_TEST";
    default:                                     return "NONE";
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
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}
