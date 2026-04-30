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
// QTimer1 CH0 low-word counter.  process_interrupt maps that low-word
// hardware observation into the private synthetic 32-bit VCLOCK identity.
// When a rebootstrap is pending, it
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
#include <stdio.h>

// process_clocks owns watchdog publication/stop semantics.  process_interrupt
// only raises hard timing-identity faults through this narrow boundary.
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0,
                             uint32_t detail1,
                             uint32_t detail2,
                             uint32_t detail3);

// process_time owns the dynamic CPS report surface.  This internal hook lets
// process_interrupt publish the every-PPS local VCLOCK phase-probe diagnostic
// without exposing raw ISR-entry DWT as a general timing API.
void time_dynamic_cps_phase_probe_update(uint32_t pps_sequence,
                                         uint32_t pps_isr_entry_dwt_raw,
                                         uint32_t pps_dwt_at_edge,
                                         uint32_t selected_counter32,
                                         uint32_t target_counter32,
                                         uint32_t counter32_at_event,
                                         int32_t counter32_residual_ticks,
                                         uint32_t arm_dwt_raw,
                                         uint32_t vclock_isr_entry_dwt_raw,
                                         uint32_t vclock_dwt_at_event,
                                         uint32_t pps_to_arm_raw_cycles,
                                         uint32_t arm_to_vclock_raw_cycles,
                                         uint32_t raw_delta_cycles,
                                         uint32_t ticks_from_sacred_to_probe,
                                         uint32_t cps_used,
                                         uint32_t cps_source,
                                         uint32_t expected_probe_offset_cycles,
                                         int64_t expected_raw_delta_cycles,
                                         int32_t residual_cycles,
                                         int64_t residual_scaled_numerator,
                                         uint32_t residual_scaled_denominator,
                                         int32_t residual_millicycles,
                                         uint32_t sacred_dwt_from_probe_raw,
                                         uint32_t sacred_dwt_from_pps_raw,
                                         uint32_t arm_count,
                                         uint32_t fire_count,
                                         uint32_t miss_count);

// ============================================================================
// Constants
// ============================================================================

// The GPIO/QTimer latency constants are measured by the latency calibration
// process as full software-stimulus-to-ISR paths.  process_interrupt needs the
// event-coordinate ISR-entry correction, so subtract the known software pin
// launch overhead from those totals.
static constexpr uint32_t STIMULUS_LAUNCH_LATENCY_CYCLES = 5;

// PPS_VCLOCK epoch selection: the canonical edge is the first VCLOCK edge
// after the physical PPS pulse.  Counter32 offset is currently 0 because
// the GPIO ISR's bus-delayed counter read is already that selected edge on
// this hardware.  Tunable named constants so live measurements can adjust.
static constexpr uint32_t VCLOCK_EPOCH_TICKS_AFTER_PHYSICAL_PPS = 1;
static constexpr int32_t  VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS   = 0;
// The PPS GPIO ISR reads the VCLOCK low word after the selected edge has
// already matured.  Empirically this observation is two 10 MHz ticks later
// than the sacred first-after-PPS VCLOCK edge.
static constexpr uint32_t VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED = 2;
static constexpr int64_t  GNSS_NS_PER_SECOND                    = 1000000000LL;

// DWT→PPS_VCLOCK anchor selection for OCXO/TimePop event projection.
// A DWT event near a PPS boundary may be serviced after the PPS ISR has
// published the next anchor.  The latest anchor can therefore be coherent but
// semantically wrong for that event.  Keep a tiny immutable history and choose
// the newest anchor that makes the event land in a lawful post-anchor window.
static constexpr uint32_t PVC_ANCHOR_RING_SIZE = 4;
static constexpr uint64_t PVC_ANCHOR_MAX_EVENT_OFFSET_NS = 1250000000ULL;

static constexpr uint32_t ANCHOR_SELECT_NONE     = 0;
static constexpr uint32_t ANCHOR_SELECT_LATEST   = 1;
static constexpr uint32_t ANCHOR_SELECT_PREVIOUS = 2;
static constexpr uint32_t ANCHOR_SELECT_OLDER    = 3;
static constexpr uint32_t ANCHOR_SELECT_FAILED   = 4;

static constexpr uint32_t ANCHOR_FAIL_RING_UNSTABLE = 1u << 0;
static constexpr uint32_t ANCHOR_FAIL_NO_ANCHORS    = 1u << 1;
static constexpr uint32_t ANCHOR_FAIL_BAD_ANCHOR    = 1u << 2;
static constexpr uint32_t ANCHOR_FAIL_NO_CPS        = 1u << 3;
static constexpr uint32_t ANCHOR_FAIL_DELTA_RANGE   = 1u << 4;
static constexpr uint32_t ANCHOR_FAIL_NO_PLAUSIBLE  = 1u << 5;

// VCLOCK endpoint repair is currently disabled / witness-only.
//
// The previous active diagnostic path proved useful but could destabilize the
// foreground path while we are still validating the new VCLOCK-domain anchor
// architecture.  Leave the constants and report surface in place, but keep the
// ISR hot path passive until we explicitly re-enable the experiment.
static constexpr bool     VCLOCK_DWT_REPAIR_DIAG_ENABLED = false;
static constexpr uint32_t VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES = 10;
static constexpr uint32_t VCLOCK_DWT_REPAIR_MIN_HISTORY_COUNT = 8;
static constexpr uint32_t VCLOCK_DWT_REPAIR_MAX_PREDICTION_RESIDUAL_CYCLES = 25;

// PPS/VCLOCK identity watchdog.  Paired PPS and selected VCLOCK events should
// remain close to the empirically calibrated raw-PPS-to-selected-VCLOCK offset.
// Fault publication is temporarily disabled; the report still records phase
// checks and faults so we can verify stability without stopping campaigns or
// flooding the foreground queue.
static constexpr bool     PPS_VCLOCK_PHASE_WATCHDOG_ENABLED = false;
static constexpr uint32_t PPS_VCLOCK_PHASE_TOLERANCE_CYCLES = 10;

// Every PPS edge arms QTimer1 CH1 as a local VCLOCK phase probe.  The target
// is intentionally only a few 10 MHz ticks in the future so tau and DWT-vs-ns
// conversion error vanish below the diagnostic resolution.  If this lead is
// too aggressive, the missed-probe counter will tell us immediately.
static constexpr uint32_t PPS_VCLOCK_PHASE_PROBE_LEAD_TICKS = 8;

static constexpr uint32_t PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_PREDICTION = 1;
static constexpr uint32_t PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_DYNAMIC_CPS = 2;
static constexpr uint32_t PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_NOMINAL = 3;

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

static pps_edge_dispatch_fn g_pps_edge_dispatch = nullptr;
static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// Dynamic CPS compatibility
// ============================================================================
//
// The intra-second DWT cycles-per-second refinement model is owned by
// process_time.  process_interrupt still provides this legacy accessor so
// existing diagnostics and callers continue to compile.

uint32_t interrupt_dynamic_cps(void) {
  return time_dynamic_cps_current();
}

// ============================================================================
// PPS_VCLOCK anchor history (for DWT→GNSS projection)
// ============================================================================
//
// The latest PPS_VCLOCK anchor is not always the correct anchor for a lower-
// priority OCXO/TimePop event.  If an event occurs just before a PPS boundary
// but is serviced after the priority-0 PPS ISR publishes the next anchor,
// projecting against the latest anchor turns "slightly before" into a huge
// unsigned DWT delta.  The ring preserves recent immutable anchors so the
// projector can choose the newest anchor that makes the event physically
// plausible.

struct pvc_anchor_record_t {
  uint32_t sequence = 0;
  uint32_t dwt_at_edge = 0;
  uint32_t counter32_at_edge = 0;
  int64_t  gnss_ns_at_edge = -1;
  uint32_t cps = 0;
  bool     cps_valid = false;
};

struct bridge_projection_t {
  int64_t  gnss_ns = -1;
  uint32_t anchor_sequence_used = 0;
  uint32_t anchor_age_slots = 0;
  uint32_t anchor_selection_kind = ANCHOR_SELECT_FAILED;
  uint32_t anchor_dwt_at_edge = 0;
  int64_t  anchor_gnss_ns_at_edge = -1;
  uint32_t anchor_cps = 0;
  uint64_t anchor_ns_delta = 0;
  uint32_t anchor_failure_mask = 0;
};

struct bridge_anchor_stats_t {
  uint32_t latest_count = 0;
  uint32_t previous_count = 0;
  uint32_t older_count = 0;
  uint32_t failed_count = 0;
  uint32_t last_selection_kind = ANCHOR_SELECT_NONE;
  uint32_t last_anchor_age_slots = 0;
  uint32_t last_anchor_sequence_used = 0;
  uint64_t last_anchor_ns_delta = 0;
  uint32_t last_anchor_failure_mask = 0;
};

static volatile uint32_t g_pvc_anchor_seq = 0;
static volatile uint32_t g_pvc_anchor_head = 0;
static volatile uint32_t g_pvc_anchor_count = 0;
static volatile bool     g_pvc_anchor_reset_pending = false;
static pvc_anchor_record_t g_pvc_anchor_ring[PVC_ANCHOR_RING_SIZE];

static bridge_anchor_stats_t g_bridge_stats_timepop = {};
static bridge_anchor_stats_t g_bridge_stats_ocxo1 = {};
static bridge_anchor_stats_t g_bridge_stats_ocxo2 = {};

static void pvc_anchor_ring_reset(void) {
  g_pvc_anchor_seq++;
  dmb_barrier();

  g_pvc_anchor_head = 0;
  g_pvc_anchor_count = 0;
  g_pvc_anchor_reset_pending = false;
  for (uint32_t i = 0; i < PVC_ANCHOR_RING_SIZE; i++) {
    g_pvc_anchor_ring[i] = pvc_anchor_record_t{};
  }

  dmb_barrier();
  g_pvc_anchor_seq++;
}

static void pvc_anchor_publish(const pps_vclock_t& pvc) {
  const uint32_t next = (g_pvc_anchor_count == 0)
      ? 0
      : ((g_pvc_anchor_head + 1u) % PVC_ANCHOR_RING_SIZE);

  g_pvc_anchor_seq++;
  dmb_barrier();

  g_pvc_anchor_ring[next].sequence = pvc.sequence;
  g_pvc_anchor_ring[next].dwt_at_edge = pvc.dwt_at_edge;
  g_pvc_anchor_ring[next].counter32_at_edge = pvc.counter32_at_edge;
  g_pvc_anchor_ring[next].gnss_ns_at_edge = pvc.gnss_ns_at_edge;
  uint32_t cps = 0;
  const bool cps_valid = time_dynamic_cps_for_pvc_sequence(pvc.sequence, &cps);
  g_pvc_anchor_ring[next].cps = cps;
  g_pvc_anchor_ring[next].cps_valid = cps_valid && cps != 0;

  g_pvc_anchor_head = next;
  if (g_pvc_anchor_count < PVC_ANCHOR_RING_SIZE) {
    g_pvc_anchor_count++;
  }

  dmb_barrier();
  g_pvc_anchor_seq++;
}

static bool pvc_anchor_snapshot(pvc_anchor_record_t* out, uint32_t& out_count) {
  out_count = 0;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_pvc_anchor_seq;
    dmb_barrier();

    const uint32_t count = g_pvc_anchor_count;
    const uint32_t head = g_pvc_anchor_head;
    const uint32_t bounded = (count > PVC_ANCHOR_RING_SIZE) ? PVC_ANCHOR_RING_SIZE : count;

    for (uint32_t age = 0; age < bounded; age++) {
      const uint32_t idx = (head + PVC_ANCHOR_RING_SIZE - age) % PVC_ANCHOR_RING_SIZE;
      out[age].sequence = g_pvc_anchor_ring[idx].sequence;
      out[age].dwt_at_edge = g_pvc_anchor_ring[idx].dwt_at_edge;
      out[age].counter32_at_edge = g_pvc_anchor_ring[idx].counter32_at_edge;
      out[age].gnss_ns_at_edge = g_pvc_anchor_ring[idx].gnss_ns_at_edge;
      out[age].cps = g_pvc_anchor_ring[idx].cps;
      out[age].cps_valid = g_pvc_anchor_ring[idx].cps_valid;
    }

    dmb_barrier();
    const uint32_t s2 = g_pvc_anchor_seq;
    if (s1 == s2 && (s1 & 1u) == 0u) {
      out_count = bounded;
      return true;
    }
  }

  return false;
}

static void bridge_projection_copy_to_diag(interrupt_capture_diag_t& diag,
                                           const bridge_projection_t& proj) {
  diag.anchor_sequence_used = proj.anchor_sequence_used;
  diag.anchor_age_slots = proj.anchor_age_slots;
  diag.anchor_selection_kind = proj.anchor_selection_kind;
  diag.anchor_dwt_at_edge = proj.anchor_dwt_at_edge;
  diag.anchor_gnss_ns_at_edge = proj.anchor_gnss_ns_at_edge;
  diag.anchor_cps = proj.anchor_cps;
  diag.anchor_ns_delta = proj.anchor_ns_delta;
  diag.anchor_failure_mask = proj.anchor_failure_mask;
}

static bridge_anchor_stats_t* bridge_stats_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::TIMEPOP) return &g_bridge_stats_timepop;
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_bridge_stats_ocxo1;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_bridge_stats_ocxo2;
  return nullptr;
}

static void bridge_projection_record_stats(interrupt_subscriber_kind_t kind,
                                           const bridge_projection_t& proj) {
  bridge_anchor_stats_t* s = bridge_stats_for(kind);
  if (!s) return;

  if (proj.anchor_selection_kind == ANCHOR_SELECT_LATEST) {
    s->latest_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_PREVIOUS) {
    s->previous_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_OLDER) {
    s->older_count++;
  } else if (proj.anchor_selection_kind == ANCHOR_SELECT_FAILED) {
    s->failed_count++;
  }

  s->last_selection_kind = proj.anchor_selection_kind;
  s->last_anchor_age_slots = proj.anchor_age_slots;
  s->last_anchor_sequence_used = proj.anchor_sequence_used;
  s->last_anchor_ns_delta = proj.anchor_ns_delta;
  s->last_anchor_failure_mask = proj.anchor_failure_mask;
}

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
// PPS witness + VCLOCK-domain canonical epoch latch
// ============================================================================
//
// PPS GPIO is a witness/selector only.  It never authors the public
// PPS/VCLOCK DWT coordinate.  The super-canonical PPS/VCLOCK DWT coordinate
// is authored by the VCLOCK/QTimer1 CH3 path, so every public DWT timing fact
// lives in the same QTimer event-coordinate species.
//
// During rebootstrap, the PPS ISR selects the VCLOCK counter identity of the
// sacred edge and arms CH3.  The first CH3 cadence event then backdates by
// one 1 ms VCLOCK cadence interval to author the selected edge's DWT in the
// VCLOCK coordinate domain.  After bootstrap, each 1000th CH3 cadence event
// authors the next canonical PPS/VCLOCK bookend directly.

struct vclock_epoch_latch_t {
  bool     pending = false;
  pps_t    pps{};

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge = 0;
  int64_t  gnss_ns_at_edge = -1;

  uint32_t observed_counter32 = 0;
  uint16_t observed_ch3 = 0;
  uint32_t observed_ticks_after_selected = 0;
  uint32_t first_cadence_counter32 = 0;
  uint32_t first_cadence_dwt = 0;
  uint32_t backdate_ticks = 0;
  uint32_t backdate_cycles = 0;
  uint32_t sacred_dwt = 0;
};

static pps_t g_last_pps_witness = {};
static bool  g_last_pps_witness_valid = false;
static vclock_epoch_latch_t g_vclock_epoch_latch = {};

static inline uint32_t abs_i32_to_u32_local(int32_t v) {
  return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

struct dwt_repair_diag_t {
  bool     valid = false;
  bool     candidate = false;
  bool     synthetic = false;   // operational replacement applied; currently false
  uint32_t original_dwt = 0;
  uint32_t predicted_dwt = 0;
  uint32_t used_dwt = 0;
  int32_t  error_cycles = 0;
  const char* reason = "none";
};

struct vclock_repair_stats_t {
  uint32_t candidate_count = 0;
  uint32_t applied_count = 0;
  uint32_t consecutive_candidate_count = 0;
  uint32_t last_original_dwt = 0;
  uint32_t last_predicted_dwt = 0;
  uint32_t last_used_dwt = 0;
  int32_t  last_error_cycles = 0;
  uint32_t max_abs_error_cycles = 0;
  bool     last_candidate = false;
  bool     last_synthetic = false;

  uint32_t phase_check_count = 0;
  uint32_t phase_fault_count = 0;
  uint32_t phase_unpaired_count = 0;
  int32_t  last_phase_cycles = 0;
  int32_t  last_phase_error_cycles = 0;
};

static vclock_repair_stats_t g_vclock_repair_stats = {};

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

static uint32_t vclock_cycles_for_ticks(uint32_t vclock_ticks) {
  uint32_t cycles = time_dwt_next_prediction_cycles();
  if (cycles == 0) cycles = interrupt_dynamic_cps();
  if (cycles == 0) cycles = DWT_EXPECTED_PER_PPS;

  return (uint32_t)(((uint64_t)cycles * (uint64_t)vclock_ticks +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static dwt_repair_diag_t vclock_endpoint_repair_diagnostic(uint32_t observed_dwt) {
  dwt_repair_diag_t r{};
  r.valid = false;
  r.candidate = false;
  r.synthetic = false;
  r.original_dwt = observed_dwt;
  r.predicted_dwt = 0;
  r.used_dwt = observed_dwt;
  r.error_cycles = 0;
  r.reason = "disabled";

  g_vclock_repair_stats.last_candidate = false;
  g_vclock_repair_stats.last_synthetic = false;
  g_vclock_repair_stats.last_original_dwt = observed_dwt;
  g_vclock_repair_stats.last_predicted_dwt = 0;
  g_vclock_repair_stats.last_used_dwt = observed_dwt;
  g_vclock_repair_stats.last_error_cycles = 0;
  g_vclock_repair_stats.consecutive_candidate_count = 0;

  if (!VCLOCK_DWT_REPAIR_DIAG_ENABLED) {
    return r;
  }

  return r;
}

static void vclock_check_pps_phase_or_watchdog(const pps_t& pps,
                                               uint32_t sequence,
                                               uint32_t counter32_at_edge,
                                               uint32_t dwt_at_edge) {
  if (pps.sequence != sequence || pps.counter32_at_edge == 0) {
    g_vclock_repair_stats.phase_unpaired_count++;
    return;
  }

  const uint32_t observed_delta_ticks = pps.counter32_at_edge - counter32_at_edge;
  if (observed_delta_ticks != VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED) {
    g_vclock_repair_stats.phase_unpaired_count++;
    return;
  }

  const uint32_t pps_raw_entry_dwt =
      pps.dwt_at_edge + (GPIO_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
  const int32_t phase_cycles = (int32_t)(dwt_at_edge - pps_raw_entry_dwt);
  const int32_t phase_error =
      phase_cycles - (int32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
  const uint32_t abs_error = abs_i32_to_u32_local(phase_error);

  g_vclock_repair_stats.phase_check_count++;
  g_vclock_repair_stats.last_phase_cycles = phase_cycles;
  g_vclock_repair_stats.last_phase_error_cycles = phase_error;

  if (abs_error > PPS_VCLOCK_PHASE_TOLERANCE_CYCLES) {
    g_vclock_repair_stats.phase_fault_count++;
    if (PPS_VCLOCK_PHASE_WATCHDOG_ENABLED) {
      clocks_watchdog_anomaly("pps_vclock_phase_error",
                              (uint32_t)phase_cycles,
                              (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES,
                              (uint32_t)phase_error,
                              sequence);
    }
  }
}

static void publish_vclock_domain_pps_vclock(const pps_t& pps,
                                             uint32_t sequence,
                                             uint32_t dwt_at_edge,
                                             uint32_t counter32_at_edge,
                                             uint16_t ch3_at_edge,
                                             int64_t gnss_ns_at_edge) {
  pps_vclock_t pvc;
  pvc.sequence = sequence;
  pvc.dwt_at_edge = dwt_at_edge;
  pvc.counter32_at_edge = counter32_at_edge;
  pvc.ch3_at_edge = ch3_at_edge;
  pvc.gnss_ns_at_edge = gnss_ns_at_edge;

  vclock_check_pps_phase_or_watchdog(pps, sequence, counter32_at_edge, dwt_at_edge);

  g_pps_gpio_heartbeat.last_dwt = pvc.dwt_at_edge;
  g_pps_gpio_heartbeat.last_gnss_ns = pvc.gnss_ns_at_edge;

  if (g_pvc_anchor_reset_pending) {
    pvc_anchor_ring_reset();
  }

  time_dynamic_cps_pps_vclock_edge(pvc.sequence, pvc.dwt_at_edge);

  store_publish(pps, pvc);
  pvc_anchor_publish(pvc);
}

static void publish_selected_epoch_from_first_vclock_tick(uint32_t qtimer_event_dwt) {
  if (!g_vclock_epoch_latch.pending) return;

  const uint32_t backdate_ticks = VCLOCK_INTERVAL_COUNTS;
  const uint32_t backdate_cycles = vclock_cycles_for_ticks(backdate_ticks);
  const uint32_t anchor_dwt = qtimer_event_dwt - backdate_cycles;

  g_vclock_epoch_latch.first_cadence_counter32 =
      g_vclock_epoch_latch.counter32_at_edge + backdate_ticks;
  g_vclock_epoch_latch.first_cadence_dwt = qtimer_event_dwt;
  g_vclock_epoch_latch.backdate_ticks = backdate_ticks;
  g_vclock_epoch_latch.backdate_cycles = backdate_cycles;
  g_vclock_epoch_latch.sacred_dwt = anchor_dwt;

  publish_vclock_domain_pps_vclock(g_vclock_epoch_latch.pps,
                                    g_vclock_epoch_latch.pps.sequence,
                                    anchor_dwt,
                                    g_vclock_epoch_latch.counter32_at_edge,
                                    g_vclock_epoch_latch.ch3_at_edge,
                                    g_vclock_epoch_latch.gnss_ns_at_edge);

  g_vclock_epoch_latch.pending = false;

  if (g_pps_edge_dispatch) {
    timepop_arm_asap(pps_edge_dispatch_trampoline, nullptr, "PPS_VCLOCK_EPOCH_DISPATCH");
  }
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
  return isr_entry_dwt_raw - (GPIO_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

static inline uint32_t pps_vclock_dwt_from_pps_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw + (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;
}

static inline uint32_t qtimer_event_dwt_from_isr_entry_raw(uint32_t isr_entry_dwt_raw) {
  return isr_entry_dwt_raw - (QTIMER_TOTAL_LATENCY - STIMULUS_LAUNCH_LATENCY_CYCLES);
}

// ============================================================================
// DWT → GNSS conversion (the OCXO/TimePop exception)
// ============================================================================

static bridge_projection_t interrupt_dwt_to_vclock_gnss_projection(uint32_t dwt_at_event) {
  bridge_projection_t out{};
  out.gnss_ns = -1;
  out.anchor_selection_kind = ANCHOR_SELECT_FAILED;

  pvc_anchor_record_t anchors[PVC_ANCHOR_RING_SIZE];
  uint32_t count = 0;
  if (!pvc_anchor_snapshot(anchors, count)) {
    out.anchor_failure_mask |= ANCHOR_FAIL_RING_UNSTABLE;
    return out;
  }

  if (count == 0) {
    out.anchor_failure_mask |= ANCHOR_FAIL_NO_ANCHORS;
    return out;
  }

  for (uint32_t age = 0; age < count; age++) {
    const pvc_anchor_record_t& a = anchors[age];

    if (a.sequence == 0 || a.gnss_ns_at_edge < 0) {
      out.anchor_failure_mask |= ANCHOR_FAIL_BAD_ANCHOR;
      continue;
    }

    uint32_t cps = 0;
    bool cps_valid = time_dynamic_cps_for_pvc_sequence(a.sequence, &cps);
    if (!cps_valid && a.cps_valid && a.cps != 0) {
      cps = a.cps;
      cps_valid = true;
    }

    if (!cps_valid || cps == 0) {
      out.anchor_failure_mask |= ANCHOR_FAIL_NO_CPS;
      continue;
    }

    const uint32_t dwt_delta = dwt_at_event - a.dwt_at_edge;
    const uint64_t ns_delta =
        ((uint64_t)dwt_delta * (uint64_t)GNSS_NS_PER_SECOND + (uint64_t)cps / 2ULL) /
        (uint64_t)cps;

    if (ns_delta > PVC_ANCHOR_MAX_EVENT_OFFSET_NS) {
      out.anchor_failure_mask |= ANCHOR_FAIL_DELTA_RANGE;
      continue;
    }

    out.gnss_ns = a.gnss_ns_at_edge + (int64_t)ns_delta;
    out.anchor_sequence_used = a.sequence;
    out.anchor_age_slots = age;
    out.anchor_selection_kind = (age == 0)
        ? ANCHOR_SELECT_LATEST
        : ((age == 1) ? ANCHOR_SELECT_PREVIOUS : ANCHOR_SELECT_OLDER);
    out.anchor_dwt_at_edge = a.dwt_at_edge;
    out.anchor_gnss_ns_at_edge = a.gnss_ns_at_edge;
    out.anchor_cps = cps;
    out.anchor_ns_delta = ns_delta;
    return out;
  }

  out.anchor_failure_mask |= ANCHOR_FAIL_NO_PLAUSIBLE;
  return out;
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

static volatile interrupt_pps_entry_latency_handler_fn
    g_pps_entry_latency_handler = nullptr;



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

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind);

// ============================================================================
// QTimer1 CH0 low-word counter
// ============================================================================
//
// CH0 is the only VCLOCK hardware counter used by process_interrupt now.
// CH1 is deliberately not cascaded; it remains free for a future dedicated
// VCLOCK-domain compare rail.  Any public 32-bit VCLOCK identity is synthetic
// and authored below.

static inline uint16_t qtimer1_ch0_counter_now(void) {
  return IMXRT_TMR1.CH[0].CNTR;
}

// ============================================================================
// Private synthetic 32-bit clock identities
// ============================================================================
//
// Each QTimer-derived clock domain has a private process_interrupt-owned
// synthetic 32-bit identity.  The public contract is simple: subscribers get
// counter32_at_event only in the interrupt event payload.  They do not read
// timer hardware, and they do not author counter identities themselves.
//
// For VCLOCK, the synthetic identity is mapped to QTimer1 CH0's 16-bit
// hardware counter by a low-word anchor established at an exact edge.  CH1 is
// no longer cascaded.  TimePop schedules by synthetic deadlines while
// process_interrupt alone translates those deadlines into low-word hardware
// compare values.  For OCXO lanes, the synthetic identity is advanced by the
// exact 10 MHz tick interval of the process_interrupt-owned cadence.

struct synthetic_clock32_t {
  bool     zeroed = false;
  uint64_t zero_ns = 0;
  uint32_t zero_counter32 = 0;
  uint32_t current_counter32 = 0;
  uint32_t zero_count = 0;

  bool     pending_zero = false;
  uint64_t pending_zero_ns = 0;
  uint32_t pending_zero_counter32 = 0;
  uint32_t pending_zero_count = 0;
};

struct vclock_synthetic_clock32_t : synthetic_clock32_t {
  uint16_t hardware_low16_at_zero = 0;
  uint16_t hardware_low16_at_current = 0;
  bool     hardware_anchor_valid = false;
  uint32_t hardware_anchor_update_count = 0;
};

static vclock_synthetic_clock32_t g_vclock_clock32;
static synthetic_clock32_t        g_ocxo1_clock32;
static synthetic_clock32_t        g_ocxo2_clock32;

static volatile uint32_t g_qtimer1_ch1_sequence = 0;
static volatile uint32_t g_qtimer1_ch1_target_counter32 = 0;
static volatile uint32_t g_qtimer1_ch1_next_compare_counter32 = 0;
static volatile uint32_t g_qtimer1_ch1_arm_count = 0;
static volatile uint32_t g_qtimer1_ch1_fire_count = 0;
static volatile uint32_t g_qtimer1_ch1_hop_count = 0;
static volatile bool     g_qtimer1_ch1_active = false;
static volatile interrupt_qtimer1_ch1_handler_fn g_qtimer1_ch1_handler = nullptr;

static constexpr uint8_t QTIMER1_CH1_OWNER_NONE = 0;
static constexpr uint8_t QTIMER1_CH1_OWNER_HOSTED = 1;
static constexpr uint8_t QTIMER1_CH1_OWNER_PPS_PHASE_PROBE = 2;

static volatile uint8_t g_qtimer1_ch1_next_owner = QTIMER1_CH1_OWNER_NONE;

struct pps_vclock_phase_probe_t {
  bool     active = false;
  uint32_t pps_sequence = 0;
  uint32_t pps_isr_entry_dwt_raw = 0;
  uint32_t pps_dwt_at_edge = 0;
  uint32_t selected_counter32 = 0;
  uint32_t target_counter32 = 0;
  uint32_t arm_dwt_raw = 0;
  uint32_t arm_count = 0;
  uint32_t fire_count = 0;
  uint32_t miss_count = 0;
};

static pps_vclock_phase_probe_t g_pps_vclock_phase_probe = {};

static volatile uint32_t g_qtimer1_ch2_last_target_counter32 = 0;
static volatile uint32_t g_qtimer1_ch2_arm_count = 0;

static inline uint32_t clock32_from_ns(uint64_t ns) {
  return (uint32_t)((ns / 100ULL) & 0xFFFFFFFFULL);
}

uint32_t interrupt_clock32_from_ns(uint64_t ns) {
  return clock32_from_ns(ns);
}

static synthetic_clock32_t* synthetic_clock_for_kind(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_clock32;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_clock32;
  return nullptr;
}

static void synthetic_clock_zero(synthetic_clock32_t& c, uint64_t ns) {
  c.zeroed = true;
  c.zero_ns = ns;
  c.zero_counter32 = clock32_from_ns(ns);
  c.current_counter32 = c.zero_counter32;
  c.zero_count++;
  c.pending_zero = false;
}

static void vclock_clock_anchor_hardware_low16(uint32_t synthetic_counter32,
                                                 uint16_t hardware_low16) {
  g_vclock_clock32.current_counter32 = synthetic_counter32;
  g_vclock_clock32.hardware_low16_at_current = hardware_low16;
  g_vclock_clock32.hardware_anchor_valid = true;
  g_vclock_clock32.hardware_anchor_update_count++;
}

static void vclock_clock_zero_at_hardware_low16(uint64_t ns,
                                                uint16_t hardware_low16) {
  g_vclock_clock32.zeroed = true;
  g_vclock_clock32.zero_ns = ns;
  g_vclock_clock32.zero_counter32 = clock32_from_ns(ns);
  g_vclock_clock32.hardware_low16_at_zero = hardware_low16;
  g_vclock_clock32.zero_count++;
  g_vclock_clock32.pending_zero = false;
  vclock_clock_anchor_hardware_low16(g_vclock_clock32.zero_counter32,
                                     hardware_low16);
}

static uint32_t vclock_synthetic_from_hardware_low16(uint16_t hardware_low16) {
  if (!g_vclock_clock32.zeroed) return (uint32_t)hardware_low16;

  if (g_vclock_clock32.hardware_anchor_valid) {
    return g_vclock_clock32.current_counter32 +
           (uint32_t)((uint16_t)(hardware_low16 -
                                g_vclock_clock32.hardware_low16_at_current));
  }

  return g_vclock_clock32.zero_counter32 +
         (uint32_t)((uint16_t)(hardware_low16 -
                              g_vclock_clock32.hardware_low16_at_zero));
}

static uint16_t vclock_hardware_low16_from_synthetic(uint32_t synthetic_counter32) {
  if (!g_vclock_clock32.zeroed) return (uint16_t)(synthetic_counter32 & 0xFFFFU);

  return (uint16_t)(g_vclock_clock32.hardware_low16_at_zero +
                   (uint16_t)(synthetic_counter32 -
                              g_vclock_clock32.zero_counter32));
}

bool interrupt_clock32_zero_from_ns(interrupt_subscriber_kind_t kind, uint64_t ns) {
  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    vclock_clock_zero_at_hardware_low16(ns, qtimer1_ch0_counter_now());
    g_vclock_lane.logical_count32_at_last_second = g_vclock_clock32.current_counter32;
    return true;
  }

  synthetic_clock32_t* c = synthetic_clock_for_kind(kind);
  if (!c) return false;
  synthetic_clock_zero(*c, ns);

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (lane) {
    lane->logical_count32_at_last_second = c->current_counter32;
    lane->tick_mod_1000 = 0;
  }
  return true;
}

bool interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t kind, uint64_t ns) {
  const uint32_t counter32 = clock32_from_ns(ns);

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_clock32.pending_zero = true;
    g_vclock_clock32.pending_zero_ns = ns;
    g_vclock_clock32.pending_zero_counter32 = counter32;
    g_vclock_clock32.pending_zero_count++;
    return true;
  }

  synthetic_clock32_t* c = synthetic_clock_for_kind(kind);
  if (!c) return false;
  c->pending_zero = true;
  c->pending_zero_ns = ns;
  c->pending_zero_counter32 = counter32;
  c->pending_zero_count++;
  return true;
}

static inline void synthetic_clock_consume_pending_zero_if_any(synthetic_clock32_t& c) {
  if (!c.pending_zero) return;
  synthetic_clock_zero(c, c.pending_zero_ns);
}

static inline uint32_t synthetic_clock_advance(synthetic_clock32_t& c,
                                               uint32_t ticks) {
  synthetic_clock_consume_pending_zero_if_any(c);
  c.current_counter32 += ticks;
  return c.current_counter32;
}

uint32_t interrupt_qtimer1_counter32_now(void)   { return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now()); }
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

static inline void qtimer1_ch1_clear_compare_flag(void) {
  IMXRT_TMR1.CH[1].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer1_ch1_program_compare(uint16_t target_low16) {
  qtimer1_ch1_clear_compare_flag();
  IMXRT_TMR1.CH[1].COMP1  = target_low16;
  IMXRT_TMR1.CH[1].CMPLD1 = target_low16;
  qtimer1_ch1_clear_compare_flag();
  IMXRT_TMR1.CH[1].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static inline void qtimer1_ch1_disable_compare_hw(void) {
  IMXRT_TMR1.CH[1].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  qtimer1_ch1_clear_compare_flag();
}

static inline void qtimer1_ch2_clear_compare_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void qtimer1_ch2_program_compare(uint16_t target_low16) {
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].COMP1  = target_low16;
  IMXRT_TMR1.CH[2].CMPLD1 = target_low16;
  qtimer1_ch2_clear_compare_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
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
                                  bool     pps_coincidence_valid  = false,
                                  const dwt_repair_diag_t* repair = nullptr) {
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
  bridge_projection_t bridge{};
  int64_t gnss_ns = -1;
  if (rt.desc->kind == interrupt_subscriber_kind_t::VCLOCK) {
    gnss_ns = vclock_gnss_from_counter32(authored_counter32);
    bridge.anchor_selection_kind = ANCHOR_SELECT_NONE;
  } else {
    bridge = interrupt_dwt_to_vclock_gnss_projection(dwt_at_event);
    gnss_ns = bridge.gnss_ns;
    bridge_projection_record_stats(rt.desc->kind, bridge);
  }
  event.gnss_ns_at_event = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
  event.status = interrupt_event_status_t::OK;

  interrupt_capture_diag_t diag {};
  fill_diag(diag, rt, event);
  if (repair) {
    diag.dwt_synthetic = repair->synthetic;
    diag.dwt_repair_candidate = repair->candidate;
    diag.dwt_original_at_event = repair->original_dwt;
    diag.dwt_predicted_at_event = repair->predicted_dwt;
    diag.dwt_used_at_event = repair->used_dwt;
    diag.dwt_synthetic_error_cycles = repair->error_cycles;
    diag.dwt_synthetic_threshold_cycles = VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES;
    diag.dwt_synthetic_reason = repair->reason;
  }
  if (rt.desc->kind != interrupt_subscriber_kind_t::VCLOCK) {
    bridge_projection_copy_to_diag(diag, bridge);
  }

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

  const uint16_t fired_low16 = g_vclock_lane.compare_target;
  vclock_lane_advance_compare();
  g_vclock_lane.cadence_hits_total++;

  // Synthetic VCLOCK identity advances in exact 10 MHz lockstep with the
  // process_interrupt-owned CH3 cadence.  The fired CH3 compare target is
  // also the CH0 low-word identity at this cadence edge, so it becomes the
  // latest low-word anchor for ambient synthetic observations.
  g_vclock_lane.logical_count32_at_last_second += VCLOCK_INTERVAL_COUNTS;
  vclock_clock_anchor_hardware_low16(g_vclock_lane.logical_count32_at_last_second,
                                     fired_low16);

  const uint32_t cadence_tick_mod_1000 = g_vclock_lane.tick_mod_1000 + 1;

  if (cadence_tick_mod_1000 == 1 && g_vclock_epoch_latch.pending) {
    publish_selected_epoch_from_first_vclock_tick(qtimer_event_dwt);
  }

  time_dynamic_cps_cadence_update(qtimer_event_dwt, cadence_tick_mod_1000);

  if (++g_vclock_lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    g_vclock_lane.tick_mod_1000 = 0;

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

    const pps_t witness_pps = g_last_pps_witness_valid
        ? g_last_pps_witness
        : pps_t{};
    const dwt_repair_diag_t repair =
        vclock_endpoint_repair_diagnostic(qtimer_event_dwt);
    const int64_t pvc_gnss_ns =
        vclock_next_epoch_gnss_ns(time_dwt_to_gnss_ns(qtimer_event_dwt));

    // Repair is diagnostic-only for now: the observed QTimer event DWT remains
    // the canonical PPS/VCLOCK bookend.  The predicted DWT and would-repair
    // classification are carried in the VCLOCK diagnostic payload.
    publish_vclock_domain_pps_vclock(witness_pps,
                                      (witness_pps.sequence != 0)
                                          ? witness_pps.sequence
                                          : (g_pps_gpio_heartbeat.edge_count + 1),
                                      qtimer_event_dwt,
                                      g_vclock_lane.logical_count32_at_last_second,
                                      fired_low16,
                                      pvc_gnss_ns);

    emit_one_second_event(*g_rt_vclock, qtimer_event_dwt,
                          g_vclock_lane.logical_count32_at_last_second,
                          coincidence_cycles, coincidence_valid, &repair);

    if (g_pps_edge_dispatch) {
      timepop_arm_asap(pps_edge_dispatch_trampoline, nullptr, "PPS_VCLOCK_DISPATCH");
    }
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

  synthetic_clock32_t* c = synthetic_clock_for_kind(rt.desc->kind);
  if (c) {
    lane.logical_count32_at_last_second = synthetic_clock_advance(*c, OCXO_INTERVAL_COUNTS);
  } else {
    lane.logical_count32_at_last_second += OCXO_INTERVAL_COUNTS;
  }

  if (++lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    lane.tick_mod_1000 = 0;
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

// ============================================================================
// PPS GPIO ISR-entry listener — hosted diagnostics hook for process_witness
// ============================================================================

void interrupt_register_pps_entry_latency_handler(
    interrupt_pps_entry_latency_handler_fn cb) {
  g_pps_entry_latency_handler = cb;
}

// ============================================================================
// QTimer1 CH1 compare service — hosted rail for process_witness
// ============================================================================

void interrupt_register_qtimer1_ch1_handler(interrupt_qtimer1_ch1_handler_fn cb) {
  g_qtimer1_ch1_handler = cb;
}

static bool qtimer1_ch1_future_remaining(uint32_t now,
                                             uint32_t target,
                                             uint32_t& remaining) {
  remaining = target - now;
  return remaining != 0 && remaining <= 0x7FFFFFFFUL;
}

static void qtimer1_ch1_note_phase_probe_missed(void) {
  if (!g_pps_vclock_phase_probe.active) return;
  g_pps_vclock_phase_probe.active = false;
  g_pps_vclock_phase_probe.miss_count++;
}

static void qtimer1_ch1_schedule_next_hop(void) {
  const uint32_t now = vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());

  uint32_t best_remaining = 0xFFFFFFFFUL;
  uint32_t best_target = 0;
  uint8_t best_owner = QTIMER1_CH1_OWNER_NONE;

  if (g_pps_vclock_phase_probe.active) {
    uint32_t remaining = 0;
    if (qtimer1_ch1_future_remaining(now,
                                     g_pps_vclock_phase_probe.target_counter32,
                                     remaining)) {
      best_remaining = remaining;
      best_target = g_pps_vclock_phase_probe.target_counter32;
      best_owner = QTIMER1_CH1_OWNER_PPS_PHASE_PROBE;
    } else {
      qtimer1_ch1_note_phase_probe_missed();
    }
  }

  if (g_qtimer1_ch1_active) {
    uint32_t remaining = 0;
    if (qtimer1_ch1_future_remaining(now, g_qtimer1_ch1_target_counter32, remaining)) {
      if (remaining < best_remaining) {
        best_remaining = remaining;
        best_target = g_qtimer1_ch1_target_counter32;
        best_owner = QTIMER1_CH1_OWNER_HOSTED;
      }
    } else {
      // If the hosted target is behind us in modular time, stop rather than
      // generating misleading event facts.  The caller can re-arm fresh.
      g_qtimer1_ch1_active = false;
    }
  }

  if (best_owner == QTIMER1_CH1_OWNER_NONE) {
    g_qtimer1_ch1_next_owner = QTIMER1_CH1_OWNER_NONE;
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  static constexpr uint32_t CH1_MAX_COMPARE_STEP_TICKS = 49123U;
  const uint32_t step =
      (best_remaining > CH1_MAX_COMPARE_STEP_TICKS)
          ? CH1_MAX_COMPARE_STEP_TICKS
          : best_remaining;

  g_qtimer1_ch1_next_compare_counter32 = now + step;
  g_qtimer1_ch1_next_owner = best_owner;

  const uint16_t hardware_target =
      vclock_hardware_low16_from_synthetic(g_qtimer1_ch1_next_compare_counter32);
  qtimer1_ch1_program_compare(hardware_target);
}

bool interrupt_qtimer1_ch1_arm_compare(uint32_t target_counter32) {
  if (!g_interrupt_hw_ready) return false;

  g_qtimer1_ch1_target_counter32 = target_counter32;
  g_qtimer1_ch1_active = true;
  g_qtimer1_ch1_arm_count++;

  qtimer1_ch1_schedule_next_hop();
  return g_qtimer1_ch1_active;
}

void interrupt_qtimer1_ch1_disable_compare(void) {
  g_qtimer1_ch1_active = false;
  qtimer1_ch1_schedule_next_hop();
}

static uint32_t pps_vclock_phase_probe_cps(uint32_t& source) {
  uint32_t cps = time_dwt_next_prediction_cycles();
  if (cps != 0) {
    source = PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_PREDICTION;
    return cps;
  }

  cps = interrupt_dynamic_cps();
  if (cps != 0) {
    source = PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_DYNAMIC_CPS;
    return cps;
  }

  source = PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_NOMINAL;
  return DWT_EXPECTED_PER_PPS;
}

static uint32_t rounded_cycles_for_vclock_ticks(uint32_t ticks,
                                                uint32_t cps) {
  return (uint32_t)(((uint64_t)ticks * (uint64_t)cps +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static int32_t scaled_residual_to_millicycles(int64_t numerator,
                                              uint32_t denominator) {
  if (denominator == 0) return 0;

  const bool neg = numerator < 0;
  uint64_t mag = neg ? (uint64_t)(-numerator) : (uint64_t)numerator;
  const uint64_t rounded =
      (mag * 1000ULL + (uint64_t)denominator / 2ULL) /
      (uint64_t)denominator;

  if (rounded > 2147483647ULL) {
    return neg ? INT32_MIN : INT32_MAX;
  }

  return neg ? -(int32_t)rounded : (int32_t)rounded;
}

static void pps_vclock_phase_probe_arm(uint32_t pps_sequence,
                                       uint32_t pps_isr_entry_dwt_raw,
                                       uint32_t pps_dwt_at_edge,
                                       uint32_t selected_counter32) {
  if (!g_interrupt_hw_ready) return;

  if (g_pps_vclock_phase_probe.active) {
    g_pps_vclock_phase_probe.miss_count++;
  }

  const uint32_t now = vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());

  g_pps_vclock_phase_probe.active = true;
  g_pps_vclock_phase_probe.pps_sequence = pps_sequence;
  g_pps_vclock_phase_probe.pps_isr_entry_dwt_raw = pps_isr_entry_dwt_raw;
  g_pps_vclock_phase_probe.pps_dwt_at_edge = pps_dwt_at_edge;
  g_pps_vclock_phase_probe.selected_counter32 = selected_counter32;
  g_pps_vclock_phase_probe.target_counter32 =
      now + PPS_VCLOCK_PHASE_PROBE_LEAD_TICKS;
  g_pps_vclock_phase_probe.arm_dwt_raw = 0;
  g_pps_vclock_phase_probe.arm_count++;

  qtimer1_ch1_schedule_next_hop();

  // Captured immediately after the CH1 compare has been programmed (or the
  // arbiter decided it cannot be programmed).  This lets the report separate
  // "time spent arming the trap" from "time waiting for the trap to fire."
  g_pps_vclock_phase_probe.arm_dwt_raw = ARM_DWT_CYCCNT;
}

static void pps_vclock_phase_probe_fire(uint32_t fired_counter32,
                                        uint32_t isr_entry_dwt_raw) {
  if (!g_pps_vclock_phase_probe.active) return;

  const pps_vclock_phase_probe_t probe = g_pps_vclock_phase_probe;
  g_pps_vclock_phase_probe.active = false;
  g_pps_vclock_phase_probe.fire_count++;

  const uint32_t qtimer_event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const uint32_t raw_delta_cycles =
      isr_entry_dwt_raw - probe.pps_isr_entry_dwt_raw;
  const uint32_t pps_to_arm_raw_cycles =
      (probe.arm_dwt_raw != 0)
          ? (uint32_t)(probe.arm_dwt_raw - probe.pps_isr_entry_dwt_raw)
          : 0;
  const uint32_t arm_to_vclock_raw_cycles =
      (probe.arm_dwt_raw != 0)
          ? (uint32_t)(isr_entry_dwt_raw - probe.arm_dwt_raw)
          : 0;

  const uint32_t ticks_from_sacred_to_probe =
      probe.target_counter32 - probe.selected_counter32;

  uint32_t cps_source = 0;
  const uint32_t cps_used = pps_vclock_phase_probe_cps(cps_source);
  const uint32_t expected_probe_offset_cycles =
      rounded_cycles_for_vclock_ticks(ticks_from_sacred_to_probe, cps_used);

  const int64_t expected_raw_delta_cycles =
      (int64_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES +
      (int64_t)expected_probe_offset_cycles;
  const int32_t residual_cycles =
      (int32_t)((int64_t)raw_delta_cycles - expected_raw_delta_cycles);

  // Exact fixed-point residual before final presentation rounding:
  //   residual = raw_delta - canonical_offset - ticks*cps/10MHz
  // represented as numerator / VCLOCK_COUNTS_PER_SECOND cycles.
  const int64_t residual_scaled_numerator =
      ((int64_t)raw_delta_cycles -
       (int64_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES) *
          (int64_t)VCLOCK_COUNTS_PER_SECOND -
      (int64_t)((uint64_t)ticks_from_sacred_to_probe * (uint64_t)cps_used);
  const uint32_t residual_scaled_denominator = VCLOCK_COUNTS_PER_SECOND;
  const int32_t residual_millicycles =
      scaled_residual_to_millicycles(residual_scaled_numerator,
                                     residual_scaled_denominator);

  const uint32_t sacred_dwt_from_probe_raw =
      isr_entry_dwt_raw - expected_probe_offset_cycles;
  const uint32_t sacred_dwt_from_pps_raw =
      probe.pps_isr_entry_dwt_raw +
      (uint32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES;

  time_dynamic_cps_phase_probe_update(probe.pps_sequence,
                                      probe.pps_isr_entry_dwt_raw,
                                      probe.pps_dwt_at_edge,
                                      probe.selected_counter32,
                                      probe.target_counter32,
                                      fired_counter32,
                                      (int32_t)(fired_counter32 - probe.target_counter32),
                                      probe.arm_dwt_raw,
                                      isr_entry_dwt_raw,
                                      qtimer_event_dwt,
                                      pps_to_arm_raw_cycles,
                                      arm_to_vclock_raw_cycles,
                                      raw_delta_cycles,
                                      ticks_from_sacred_to_probe,
                                      cps_used,
                                      cps_source,
                                      expected_probe_offset_cycles,
                                      expected_raw_delta_cycles,
                                      residual_cycles,
                                      residual_scaled_numerator,
                                      residual_scaled_denominator,
                                      residual_millicycles,
                                      sacred_dwt_from_probe_raw,
                                      sacred_dwt_from_pps_raw,
                                      g_pps_vclock_phase_probe.arm_count,
                                      g_pps_vclock_phase_probe.fire_count,
                                      g_pps_vclock_phase_probe.miss_count);
}

uint16_t interrupt_qtimer1_ch1_counter_now(void) { return IMXRT_TMR1.CH[1].CNTR; }
uint16_t interrupt_qtimer1_ch1_comp1_now(void)   { return IMXRT_TMR1.CH[1].COMP1; }
uint16_t interrupt_qtimer1_ch1_csctrl_now(void)  { return IMXRT_TMR1.CH[1].CSCTRL; }

// ============================================================================
// TimePop hardware service API
// ============================================================================

uint32_t interrupt_vclock_counter32_observe_ambient(void) {
  return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
}

void interrupt_qtimer1_ch2_arm_compare(uint32_t target_counter32) {
  g_qtimer1_ch2_last_target_counter32 = target_counter32;
  g_qtimer1_ch2_arm_count++;
  const uint16_t hardware_target = vclock_hardware_low16_from_synthetic(target_counter32);
  qtimer1_ch2_program_compare(hardware_target);
}

uint16_t interrupt_qtimer1_ch2_counter_now(void) { return IMXRT_TMR1.CH[2].CNTR; }
uint16_t interrupt_qtimer1_ch2_comp1_now(void)   { return IMXRT_TMR1.CH[2].COMP1; }
uint16_t interrupt_qtimer1_ch2_csctrl_now(void)  { return IMXRT_TMR1.CH[2].CSCTRL; }

static void qtimer1_ch1_bridge_isr(uint32_t isr_entry_dwt_raw) {
  qtimer1_ch1_clear_compare_flag();

  if (!g_qtimer1_ch1_active && !g_pps_vclock_phase_probe.active) {
    g_qtimer1_ch1_next_owner = QTIMER1_CH1_OWNER_NONE;
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  const uint32_t fired_counter32 = g_qtimer1_ch1_next_compare_counter32;
  const uint8_t owner = g_qtimer1_ch1_next_owner;

  if (owner == QTIMER1_CH1_OWNER_PPS_PHASE_PROBE) {
    if (fired_counter32 != g_pps_vclock_phase_probe.target_counter32) {
      g_qtimer1_ch1_hop_count++;
      qtimer1_ch1_schedule_next_hop();
      return;
    }

    pps_vclock_phase_probe_fire(fired_counter32, isr_entry_dwt_raw);
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  if (owner != QTIMER1_CH1_OWNER_HOSTED || !g_qtimer1_ch1_active) {
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  if (fired_counter32 != g_qtimer1_ch1_target_counter32) {
    g_qtimer1_ch1_hop_count++;
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  g_qtimer1_ch1_active = false;
  g_qtimer1_ch1_fire_count++;

  const uint32_t qtimer_event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const int64_t gnss_ns = vclock_gnss_from_counter32(fired_counter32);

  interrupt_qtimer1_ch1_compare_event_t event{};
  event.sequence = ++g_qtimer1_ch1_sequence;
  event.target_counter32 = g_qtimer1_ch1_target_counter32;
  event.counter32_at_event = fired_counter32;
  event.counter32_residual_ticks =
      (int32_t)(fired_counter32 - g_qtimer1_ch1_target_counter32);
  event.isr_entry_dwt_raw = isr_entry_dwt_raw;
  event.dwt_at_event = qtimer_event_dwt;
  event.gnss_ns_at_event = gnss_ns;

  if (g_qtimer1_ch1_handler) {
    g_qtimer1_ch1_handler(event);
  }

  qtimer1_ch1_schedule_next_hop();
}

static void qtimer1_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.

  if (IMXRT_TMR1.CH[1].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch1_bridge_isr(isr_entry_dwt_raw);
  }
  else if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_clear_compare_flag();
    if (g_qtimer1_ch2_handler) {
      const uint32_t qtimer_event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
      const bridge_projection_t bridge = interrupt_dwt_to_vclock_gnss_projection(qtimer_event_dwt);
      const int64_t  gnss_ns = bridge.gnss_ns;
      bridge_projection_record_stats(interrupt_subscriber_kind_t::TIMEPOP, bridge);

      interrupt_event_t event{};
      event.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      event.provider = interrupt_provider_kind_t::QTIMER1;
      event.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      event.status   = interrupt_event_status_t::OK;
      event.dwt_at_event       = qtimer_event_dwt;
      event.gnss_ns_at_event   = (gnss_ns >= 0) ? (uint64_t)gnss_ns : 0;
      // CH2 compare event identity is the synthetic deadline that
      // process_interrupt armed in hardware on TimePop's behalf.  This is an
      // event fact, not a post-event ambient read.
      event.counter32_at_event = g_qtimer1_ch2_last_target_counter32;

      interrupt_capture_diag_t diag{};
      diag.enabled  = true;
      diag.provider = interrupt_provider_kind_t::QTIMER1;
      diag.lane     = interrupt_lane_t::QTIMER1_CH2_COMP;
      diag.kind     = interrupt_subscriber_kind_t::TIMEPOP;
      diag.dwt_at_event       = qtimer_event_dwt;
      diag.gnss_ns_at_event   = event.gnss_ns_at_event;
      diag.counter32_at_event = event.counter32_at_event;
      bridge_projection_copy_to_diag(diag, bridge);

      g_qtimer1_ch2_handler(event, diag);
    }
  }
  // CH1/CH2/CH3 share one vector.  If multiple flags are pending, the
  // unhandled flag remains set and NVIC re-dispatches with a fresh
  // first-instruction _raw capture.
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
  // PPS GPIO is a witness/selector only.  It captures the physical PPS facts
  // and, during rebootstrap, selects the VCLOCK-domain edge identity.  It does
  // NOT author the public PPS/VCLOCK DWT coordinate; that coordinate is later
  // authored by the VCLOCK/QTimer1 CH3 path so all public DWT captures live in
  // a single VCLOCK event-coordinate system.
  const uint16_t hardware_low16 = qtimer1_ch0_counter_now();
  const uint16_t ch3_now = hardware_low16;

  const uint16_t selected_low16 =
      (uint16_t)(hardware_low16 -
                 (uint16_t)VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED);

  uint64_t zero_ns_consumed = 0;
  bool zero_consumed = false;
  if (g_vclock_clock32.pending_zero) {
    zero_ns_consumed = g_vclock_clock32.pending_zero_ns;
    vclock_clock_zero_at_hardware_low16(zero_ns_consumed, selected_low16);
    zero_consumed = true;
    g_vclock_epoch_gnss_ns = (int64_t)zero_ns_consumed;
  }

  const uint32_t observed_counter32 =
      vclock_synthetic_from_hardware_low16(hardware_low16) +
      (uint32_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  const uint32_t selected_counter32 =
      observed_counter32 - VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED;

  g_gpio_irq_count++;
  g_pps_gpio_heartbeat.edge_count++;

  if (g_pps_entry_latency_handler) {
    g_pps_entry_latency_handler(g_pps_gpio_heartbeat.edge_count,
                                isr_entry_dwt_raw);
  }

  pps_t pps;
  pps.sequence          = g_pps_gpio_heartbeat.edge_count;
  pps.dwt_at_edge       = pps_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  pps.counter32_at_edge = observed_counter32;
  pps.ch3_at_edge       = ch3_now;

  g_last_pps_witness = pps;
  g_last_pps_witness_valid = true;

  // Every PPS gets a near-future VCLOCK-domain CH1 probe.  This is a local
  // phase measurement, not a timing authority.  The probe records how long
  // it took to arm CH1, where the compare actually landed, and a tick-aware
  // fixed-point residual against the empirical raw PPS→VCLOCK offset.
  pps_vclock_phase_probe_arm(pps.sequence,
                             isr_entry_dwt_raw,
                             pps.dwt_at_edge,
                             selected_counter32);

  // During rebootstrap, PPS selects the sacred VCLOCK edge identity and arms
  // the CH3 cadence.  The first CH3 cadence event backdates one millisecond in
  // the VCLOCK coordinate system and publishes the canonical epoch.
  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;

    const int64_t selected_gnss_ns = zero_consumed
        ? (int64_t)zero_ns_consumed
        : vclock_next_epoch_gnss_ns(time_dwt_to_gnss_ns(pps.dwt_at_edge));

    g_vclock_epoch_latch.pending = true;
    g_vclock_epoch_latch.pps = pps;
    g_vclock_epoch_latch.counter32_at_edge = selected_counter32;
    g_vclock_epoch_latch.ch3_at_edge = selected_low16;
    g_vclock_epoch_latch.gnss_ns_at_edge = selected_gnss_ns;
    g_vclock_epoch_latch.observed_counter32 = observed_counter32;
    g_vclock_epoch_latch.observed_ch3 = ch3_now;
    g_vclock_epoch_latch.observed_ticks_after_selected =
        VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED;
    g_vclock_epoch_latch.first_cadence_counter32 = 0;
    g_vclock_epoch_latch.first_cadence_dwt = 0;
    g_vclock_epoch_latch.backdate_ticks = 0;
    g_vclock_epoch_latch.backdate_cycles = 0;
    g_vclock_epoch_latch.sacred_dwt = 0;

    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    g_vclock_lane.compare_target =
        (uint16_t)(selected_low16 + (uint16_t)VCLOCK_INTERVAL_COUNTS);
    g_vclock_lane.logical_count32_at_last_second = selected_counter32;
    vclock_clock_anchor_hardware_low16(selected_counter32, selected_low16);
    qtimer1_ch3_program_compare(g_vclock_lane.compare_target);
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
// interrupt_last_pps_vclock() — canonical PPS_VCLOCK epoch.
// interrupt_last_pps_edge()   — legacy projection, for back-compat with
//                               consumers that still take pps_edge_snapshot_t.
//                               Field map documented inline below.
//
// There is intentionally no public interrupt_last_pps() accessor.  Physical
// PPS GPIO DWT is a private witness/audit fact, not a timing primitive.

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


interrupt_pps_edge_heartbeat_t interrupt_pps_edge_heartbeat(void) {
  interrupt_pps_edge_heartbeat_t out{};
  out.edge_count = g_pps_gpio_heartbeat.edge_count;
  out.last_dwt = g_pps_gpio_heartbeat.last_dwt;
  out.last_gnss_ns = g_pps_gpio_heartbeat.last_gnss_ns;
  out.gpio_irq_count = g_gpio_irq_count;
  out.gpio_miss_count = g_gpio_miss_count;
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
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
  g_vclock_epoch_gnss_ns = -1;
  g_pvc_anchor_reset_pending = true;
  time_dynamic_cps_reset();
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

  // CH1 is an independent VCLOCK-domain compare rail hosted by
  // process_interrupt for clients such as process_witness.  It is NOT a
  // cascade high word.
  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  qtimer1_ch1_disable_compare_hw();

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;

  (void)IMXRT_TMR1.CH[0].CNTR;
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
  // fully, then a single ENBL=0x0F write starts the active VCLOCK-domain
  // channels on the same edge.  CH1 is an interrupt-owned hosted compare rail.
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
  g_last_pps_witness = pps_t{};
  g_last_pps_witness_valid = false;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
  g_vclock_repair_stats = vclock_repair_stats_t{};
  g_vclock_epoch_gnss_ns = -1;
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;

  g_store = snapshot_store_t{};
  pvc_anchor_ring_reset();
  g_bridge_stats_timepop = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo1 = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo2 = bridge_anchor_stats_t{};
  g_pps_edge_dispatch = nullptr;
  g_pps_entry_latency_handler = nullptr;

  g_vclock_clock32 = vclock_synthetic_clock32_t{};
  g_ocxo1_clock32 = synthetic_clock32_t{};
  g_ocxo2_clock32 = synthetic_clock32_t{};
  g_qtimer1_ch1_sequence = 0;
  g_qtimer1_ch1_target_counter32 = 0;
  g_qtimer1_ch1_next_compare_counter32 = 0;
  g_qtimer1_ch1_arm_count = 0;
  g_qtimer1_ch1_fire_count = 0;
  g_qtimer1_ch1_hop_count = 0;
  g_qtimer1_ch1_active = false;
  g_qtimer1_ch1_handler = nullptr;
  g_qtimer1_ch1_next_owner = QTIMER1_CH1_OWNER_NONE;
  g_pps_vclock_phase_probe = pps_vclock_phase_probe_t{};
  g_qtimer1_ch2_last_target_counter32 = 0;
  g_qtimer1_ch2_arm_count = 0;

  time_dynamic_cps_reset();

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


static void add_bridge_stats_payload(Payload& p,
                                     const char* prefix,
                                     const bridge_anchor_stats_t& s) {
  char key[96];
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_u32("anchor_latest_count", s.latest_count);
  add_u32("anchor_previous_count", s.previous_count);
  add_u32("anchor_older_count", s.older_count);
  add_u32("anchor_failed_count", s.failed_count);
  add_u32("anchor_last_selection_kind", s.last_selection_kind);
  add_u32("anchor_last_age_slots", s.last_anchor_age_slots);
  add_u32("anchor_last_sequence_used", s.last_anchor_sequence_used);
  add_u64("anchor_last_ns_delta", s.last_anchor_ns_delta);
  add_u32("anchor_last_failure_mask", s.last_anchor_failure_mask);
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
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);
  p.add("vclock_epoch_latch_counter32", g_vclock_epoch_latch.counter32_at_edge);
  p.add("vclock_epoch_latch_gnss_ns", g_vclock_epoch_latch.gnss_ns_at_edge);
  p.add("vclock_epoch_latch_observed_counter32", g_vclock_epoch_latch.observed_counter32);
  p.add("vclock_epoch_latch_observed_ch3", (uint32_t)g_vclock_epoch_latch.observed_ch3);
  p.add("vclock_epoch_latch_observed_ticks_after_selected", g_vclock_epoch_latch.observed_ticks_after_selected);
  p.add("vclock_epoch_latch_first_cadence_counter32", g_vclock_epoch_latch.first_cadence_counter32);
  p.add("vclock_epoch_latch_first_cadence_dwt", g_vclock_epoch_latch.first_cadence_dwt);
  p.add("vclock_epoch_latch_backdate_ticks", g_vclock_epoch_latch.backdate_ticks);
  p.add("vclock_epoch_latch_backdate_cycles", g_vclock_epoch_latch.backdate_cycles);
  p.add("vclock_epoch_latch_sacred_dwt", g_vclock_epoch_latch.sacred_dwt);
  p.add("vclock_dwt_repair_enabled", false);
  p.add("vclock_dwt_repair_threshold_cycles", VCLOCK_DWT_REPAIR_THRESHOLD_CYCLES);
  p.add("vclock_dwt_repair_min_history_count", VCLOCK_DWT_REPAIR_MIN_HISTORY_COUNT);
  p.add("vclock_dwt_repair_max_prediction_residual_cycles", VCLOCK_DWT_REPAIR_MAX_PREDICTION_RESIDUAL_CYCLES);
  p.add("vclock_dwt_repair_candidate_count", g_vclock_repair_stats.candidate_count);
  p.add("vclock_dwt_repair_applied_count", g_vclock_repair_stats.applied_count);
  p.add("vclock_dwt_repair_consecutive_candidate_count", g_vclock_repair_stats.consecutive_candidate_count);
  p.add("vclock_dwt_repair_last_candidate", g_vclock_repair_stats.last_candidate);
  p.add("vclock_dwt_repair_last_synthetic", g_vclock_repair_stats.last_synthetic);
  p.add("vclock_dwt_repair_last_original_dwt", g_vclock_repair_stats.last_original_dwt);
  p.add("vclock_dwt_repair_last_predicted_dwt", g_vclock_repair_stats.last_predicted_dwt);
  p.add("vclock_dwt_repair_last_used_dwt", g_vclock_repair_stats.last_used_dwt);
  p.add("vclock_dwt_repair_last_error_cycles", g_vclock_repair_stats.last_error_cycles);
  p.add("vclock_dwt_repair_max_abs_error_cycles", g_vclock_repair_stats.max_abs_error_cycles);
  p.add("pps_vclock_phase_expected_cycles", (int32_t)CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES);
  p.add("pps_vclock_phase_tolerance_cycles", PPS_VCLOCK_PHASE_TOLERANCE_CYCLES);
  p.add("pps_vclock_phase_watchdog_enabled", PPS_VCLOCK_PHASE_WATCHDOG_ENABLED);
  p.add("pps_vclock_phase_check_count", g_vclock_repair_stats.phase_check_count);
  p.add("pps_vclock_phase_fault_count", g_vclock_repair_stats.phase_fault_count);
  p.add("pps_vclock_phase_unpaired_count", g_vclock_repair_stats.phase_unpaired_count);
  p.add("pps_vclock_phase_last_cycles", g_vclock_repair_stats.last_phase_cycles);
  p.add("pps_vclock_phase_last_error_cycles", g_vclock_repair_stats.last_phase_error_cycles);
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

  // Dynamic CPS — now owned by process_time; mirrored here for compatibility.
  const time_dynamic_cps_snapshot_t dcps = time_dynamic_cps_snapshot();
  p.add("dynamic_cps_owner", "TIME");
  p.add("dynamic_cps",                         dcps.current_cycles);
  p.add("dynamic_cps_valid",                   dcps.valid);
  p.add("dynamic_cps_pps_sequence",            dcps.pvc_sequence);
  p.add("dynamic_cps_last_pvc_dwt_at_edge",    dcps.pvc_dwt_at_edge);
  p.add("dynamic_cps_last_reseed_value",       dcps.base_cycles);
  p.add("dynamic_cps_last_reseed_was_computed", dcps.valid);
  p.add("dynamic_cps_net_adjustment_cycles",   dcps.net_adjustment_cycles);
  p.add("dynamic_cps_refine_ticks_this_second", dcps.refine_ticks_this_second);
  p.add("dynamic_cps_adjustments_this_second", dcps.adjustments_this_second);
  p.add("dynamic_cps_total_refine_ticks",      dcps.total_refine_ticks);
  p.add("dynamic_cps_total_adjustments",       dcps.total_adjustments);

  p.add("pvc_anchor_ring_count", g_pvc_anchor_count);
  p.add("pvc_anchor_ring_head", g_pvc_anchor_head);
  p.add("pvc_anchor_ring_seq", g_pvc_anchor_seq);
  p.add("pvc_anchor_reset_pending", g_pvc_anchor_reset_pending);
  add_bridge_stats_payload(p, "timepop", g_bridge_stats_timepop);
  add_bridge_stats_payload(p, "ocxo1", g_bridge_stats_ocxo1);
  add_bridge_stats_payload(p, "ocxo2", g_bridge_stats_ocxo2);

  p.add("vclock_clock32_zeroed", g_vclock_clock32.zeroed);
  p.add("vclock_clock32_zero_ns", g_vclock_clock32.zero_ns);
  p.add("vclock_clock32_zero_counter32", g_vclock_clock32.zero_counter32);
  p.add("vclock_clock32_current", g_vclock_clock32.current_counter32);
  p.add("vclock_clock32_hardware_low16_at_zero", (uint32_t)g_vclock_clock32.hardware_low16_at_zero);
  p.add("vclock_clock32_hardware_low16_at_current", (uint32_t)g_vclock_clock32.hardware_low16_at_current);
  p.add("vclock_clock32_hardware_anchor_valid", g_vclock_clock32.hardware_anchor_valid);
  p.add("vclock_clock32_hardware_anchor_update_count", g_vclock_clock32.hardware_anchor_update_count);
  p.add("vclock_clock32_pending_zero", g_vclock_clock32.pending_zero);
  p.add("vclock_clock32_zero_count", g_vclock_clock32.zero_count);
  p.add("vclock_clock32_pending_zero_count", g_vclock_clock32.pending_zero_count);

  p.add("ocxo1_clock32_zeroed", g_ocxo1_clock32.zeroed);
  p.add("ocxo1_clock32_zero_ns", g_ocxo1_clock32.zero_ns);
  p.add("ocxo1_clock32_zero_counter32", g_ocxo1_clock32.zero_counter32);
  p.add("ocxo1_clock32_current", g_ocxo1_clock32.current_counter32);
  p.add("ocxo1_clock32_pending_zero", g_ocxo1_clock32.pending_zero);
  p.add("ocxo1_clock32_zero_count", g_ocxo1_clock32.zero_count);

  p.add("ocxo2_clock32_zeroed", g_ocxo2_clock32.zeroed);
  p.add("ocxo2_clock32_zero_ns", g_ocxo2_clock32.zero_ns);
  p.add("ocxo2_clock32_zero_counter32", g_ocxo2_clock32.zero_counter32);
  p.add("ocxo2_clock32_current", g_ocxo2_clock32.current_counter32);
  p.add("ocxo2_clock32_pending_zero", g_ocxo2_clock32.pending_zero);
  p.add("ocxo2_clock32_zero_count", g_ocxo2_clock32.zero_count);

  p.add("qtimer1_ch1_active", g_qtimer1_ch1_active);
  p.add("qtimer1_ch1_sequence", g_qtimer1_ch1_sequence);
  p.add("qtimer1_ch1_target_counter32", g_qtimer1_ch1_target_counter32);
  p.add("qtimer1_ch1_next_compare_counter32", g_qtimer1_ch1_next_compare_counter32);
  p.add("qtimer1_ch1_next_owner", (uint32_t)g_qtimer1_ch1_next_owner);
  p.add("qtimer1_ch1_arm_count", g_qtimer1_ch1_arm_count);
  p.add("qtimer1_ch1_fire_count", g_qtimer1_ch1_fire_count);
  p.add("qtimer1_ch1_hop_count", g_qtimer1_ch1_hop_count);
  p.add("pps_vclock_phase_probe_active", g_pps_vclock_phase_probe.active);
  p.add("pps_vclock_phase_probe_lead_ticks", PPS_VCLOCK_PHASE_PROBE_LEAD_TICKS);
  p.add("pps_vclock_phase_probe_cps_source_prediction", PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_PREDICTION);
  p.add("pps_vclock_phase_probe_cps_source_dynamic_cps", PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_DYNAMIC_CPS);
  p.add("pps_vclock_phase_probe_cps_source_nominal", PPS_VCLOCK_PHASE_PROBE_CPS_SOURCE_NOMINAL);
  p.add("pps_vclock_phase_probe_pps_sequence", g_pps_vclock_phase_probe.pps_sequence);
  p.add("pps_vclock_phase_probe_selected_counter32", g_pps_vclock_phase_probe.selected_counter32);
  p.add("pps_vclock_phase_probe_target_counter32", g_pps_vclock_phase_probe.target_counter32);
  p.add("pps_vclock_phase_probe_arm_dwt_raw", g_pps_vclock_phase_probe.arm_dwt_raw);
  p.add("pps_vclock_phase_probe_arm_count", g_pps_vclock_phase_probe.arm_count);
  p.add("pps_vclock_phase_probe_fire_count", g_pps_vclock_phase_probe.fire_count);
  p.add("pps_vclock_phase_probe_miss_count", g_pps_vclock_phase_probe.miss_count);
  p.add("qtimer1_ch2_last_target_counter32", g_qtimer1_ch2_last_target_counter32);
  p.add("qtimer1_ch2_arm_count", g_qtimer1_ch2_arm_count);

  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
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
    case interrupt_lane_t::QTIMER1_CH1_COMP: return "QTIMER1_CH1_COMP";
    case interrupt_lane_t::QTIMER1_CH2_COMP: return "QTIMER1_CH2_COMP";
    case interrupt_lane_t::QTIMER1_CH3_COMP: return "QTIMER1_CH3_COMP";
    case interrupt_lane_t::QTIMER3_CH2_COMP: return "QTIMER3_CH2_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}