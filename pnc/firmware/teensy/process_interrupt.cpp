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
// Scheduling architecture:
//
//   VCLOCK remains asymmetric by design: TimePop/CADENCE_MINDER tends the
//   sovereign VCLOCK rail on QTimer1 CH2.  OCXO1 and OCXO2 do not use TimePop
//   for cadence, compare arming, or post-ISR dispatch.  Each OCXO lane owns a
//   private QTimer compare mini-scheduler that advances by 10,000 lane-local
//   OCXO ticks per fire.  Most fires maintain 16-bit-to-32-bit custody; every
//   1000th epoch-aligned fire authors the OCXO one-second edge and dispatches
//   it directly to the subscriber callback.
//
// Lanes:
//   VCLOCK : TimePop recurring client on QTimer1 CH2 (10 MHz VCLOCK domain)
//   OCXO1  : QTimer2 CH0 private 1 kHz scheduler, PCS=0
//   OCXO2  : QTimer3 CH3 private 1 kHz scheduler, PCS=3
//   TimePop: QTimer1 CH2, varied compare intervals (foreground scheduler)
//
// The PPS GPIO ISR captures DWT as its first instruction, then reads the
// QTimer1 CH0 low-word counter.  process_interrupt maps that low-word
// hardware observation into the private synthetic 32-bit VCLOCK identity.
// During rebootstrap, PPS selects the sacred VCLOCK edge identity; the
// already-running TimePop VCLOCK cadence consumes the next CH2 fire fact,
// back-projects to the selected edge, refreshes the synthetic VCLOCK low-word
// anchor before TimePop schedules the next compare, and thereafter emits the
// one-second VCLOCK events on the TimePop rail.
// ============================================================================

#include "process_interrupt.h"
#include "process_clocks.h"

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

// Epoch-ready PPS capture packet.  The ISR captures the first-instruction DWT
// and the three lane hardware counters in one tiny custody window.  DWT runs
// at ~1.008 GHz, so 101 cycles is roughly one 10 MHz tick.  The packet remains
// available for the entire following second; ZERO can select it asynchronously.
static constexpr uint32_t EPOCH_CAPTURE_MAX_WINDOW_CYCLES =
    (DWT_EXPECTED_PER_PPS + (VCLOCK_COUNTS_PER_SECOND - 1U)) /
    VCLOCK_COUNTS_PER_SECOND;

// Single TimePop custody agent for the VCLOCK rail only.
//
// CADENCE_MINDER remains a TimePop critical recurring client because VCLOCK is
// the sovereign GNSS-disciplined domain and TimePop already lives there.
// OCXO1 and OCXO2 no longer participate in TimePop cadence, arming, or
// post-ISR dispatch.  Each OCXO lane owns a private QTimer compare mini-
// scheduler that fires every 1 ms in that lane's own 10 MHz domain.
static constexpr uint64_t CADENCE_MINDER_PERIOD_NS = 1000000ULL;
static constexpr const char* CADENCE_MINDER_NAME = "CADENCE_MINDER";

// OCXO lane-local mini-scheduler.
//
// The hardware compare fires every 10,000 OCXO ticks (1 ms).  Most fires
// simply extend the lane's 16-bit counter into process_interrupt's synthetic
// 32-bit counter.  Every 1000th epoch-aligned fire is the OCXO one-second
// edge delivered to CLOCKS/Alpha.
static constexpr uint32_t OCXO_CADENCE_INTERVAL_COUNTS = 10000U;
static constexpr uint32_t OCXO_EDGE_INTERVAL_COUNTS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static constexpr uint32_t OCXO_CADENCE_EVENTS_PER_SECOND =
    OCXO_EDGE_INTERVAL_COUNTS / OCXO_CADENCE_INTERVAL_COUNTS;
static_assert(OCXO_EDGE_INTERVAL_COUNTS == 10000000U,
              "OCXO edge interval must be one 10 MHz second");
static_assert(OCXO_CADENCE_EVENTS_PER_SECOND == 1000U,
              "OCXO mini-scheduler must run at 1 kHz");
static_assert((OCXO_EDGE_INTERVAL_COUNTS % OCXO_CADENCE_INTERVAL_COUNTS) == 0U,
              "OCXO one-second grid must be an integer number of cadence fires");

// OCXO DWT authorship.  OCXO1 and OCXO2 now live on separate QuadTimer
// modules/vectors.  Each OCXO ISR captures ARM_DWT_CYCCNT as its first
// instruction and applies the calibrated QTimer ISR-entry latency correction.
static constexpr uint32_t OCXO_DWT_SOURCE_NONE = 0;
static constexpr uint32_t OCXO_DWT_SOURCE_ISR_ENTRY = 1;

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
    pvc.gnss_ns_at_edge   = -1;  // GNSS labels are CLOCKS-owned.
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
// Epoch-ready PPS capture packet (seqlock)
// ============================================================================
//
// This packet is authored on every PPS GPIO ISR from the ISR's opening custody
// window.  The 16-bit hardware reads are retained only as forensic evidence;
// runtime consumers use the translated process_interrupt-owned synthetic
// 32-bit lane coordinates.  ZERO consumers select this already-authored packet
// later.

struct epoch_capture_store_t {
  volatile uint32_t seq = 0;

  volatile bool     valid = false;
  volatile uint32_t sequence = 0;
  volatile uint32_t capture_dwt_start_raw = 0;
  volatile uint32_t capture_dwt_after_vclock_raw = 0;
  volatile uint32_t capture_dwt_end_raw = 0;
  volatile uint32_t capture_window_cycles = 0;
  volatile uint32_t vclock_read_offset_cycles = 0;

  // Latency-adjusted DWT coordinate of the selected PPS_VCLOCK edge,
  // derived directly from the same first-instruction PPS ISR raw capture
  // that authored this epoch-ready packet.  CLOCKS ZERO consumes this field
  // instead of requiring the most recent published PPS_VCLOCK snapshot to
  // have caught up to this GPIO edge.
  volatile uint32_t vclock_dwt_at_edge = 0;

  volatile bool     vclock_capture_valid = false;
  volatile bool     all_lanes_capture_valid = false;

  volatile uint16_t vclock_hardware16_observed = 0;
  volatile uint16_t vclock_hardware16_selected = 0;
  volatile uint16_t ocxo1_hardware16 = 0;
  volatile uint16_t ocxo2_hardware16 = 0;

  volatile uint32_t vclock_counter32 = 0;
  volatile uint32_t ocxo1_counter32 = 0;
  volatile uint32_t ocxo2_counter32 = 0;
};

static epoch_capture_store_t g_epoch_capture_store;

static void epoch_capture_publish(const interrupt_epoch_capture_t& cap) {
  g_epoch_capture_store.seq++;
  dmb_barrier();

  g_epoch_capture_store.valid = cap.valid;
  g_epoch_capture_store.sequence = cap.sequence;
  g_epoch_capture_store.capture_dwt_start_raw = cap.capture_dwt_start_raw;
  g_epoch_capture_store.capture_dwt_after_vclock_raw = cap.capture_dwt_after_vclock_raw;
  g_epoch_capture_store.capture_dwt_end_raw = cap.capture_dwt_end_raw;
  g_epoch_capture_store.capture_window_cycles = cap.capture_window_cycles;
  g_epoch_capture_store.vclock_read_offset_cycles = cap.vclock_read_offset_cycles;
  g_epoch_capture_store.vclock_dwt_at_edge = cap.vclock_dwt_at_edge;
  g_epoch_capture_store.vclock_capture_valid = cap.vclock_capture_valid;
  g_epoch_capture_store.all_lanes_capture_valid = cap.all_lanes_capture_valid;
  g_epoch_capture_store.vclock_hardware16_observed = cap.vclock_hardware16_observed;
  g_epoch_capture_store.vclock_hardware16_selected = cap.vclock_hardware16_selected;
  g_epoch_capture_store.ocxo1_hardware16 = cap.ocxo1_hardware16;
  g_epoch_capture_store.ocxo2_hardware16 = cap.ocxo2_hardware16;
  g_epoch_capture_store.vclock_counter32 = cap.vclock_counter32;
  g_epoch_capture_store.ocxo1_counter32 = cap.ocxo1_counter32;
  g_epoch_capture_store.ocxo2_counter32 = cap.ocxo2_counter32;

  dmb_barrier();
  g_epoch_capture_store.seq++;
}

bool interrupt_last_epoch_capture(interrupt_epoch_capture_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = g_epoch_capture_store.seq;
    dmb_barrier();

    out->valid = g_epoch_capture_store.valid;
    out->sequence = g_epoch_capture_store.sequence;
    out->capture_dwt_start_raw = g_epoch_capture_store.capture_dwt_start_raw;
    out->capture_dwt_after_vclock_raw = g_epoch_capture_store.capture_dwt_after_vclock_raw;
    out->capture_dwt_end_raw = g_epoch_capture_store.capture_dwt_end_raw;
    out->capture_window_cycles = g_epoch_capture_store.capture_window_cycles;
    out->vclock_read_offset_cycles = g_epoch_capture_store.vclock_read_offset_cycles;
    out->vclock_dwt_at_edge = g_epoch_capture_store.vclock_dwt_at_edge;
    out->vclock_capture_valid = g_epoch_capture_store.vclock_capture_valid;
    out->all_lanes_capture_valid = g_epoch_capture_store.all_lanes_capture_valid;
    out->vclock_hardware16_observed = g_epoch_capture_store.vclock_hardware16_observed;
    out->vclock_hardware16_selected = g_epoch_capture_store.vclock_hardware16_selected;
    out->ocxo1_hardware16 = g_epoch_capture_store.ocxo1_hardware16;
    out->ocxo2_hardware16 = g_epoch_capture_store.ocxo2_hardware16;
    out->vclock_counter32 = g_epoch_capture_store.vclock_counter32;
    out->ocxo1_counter32 = g_epoch_capture_store.ocxo1_counter32;
    out->ocxo2_counter32 = g_epoch_capture_store.ocxo2_counter32;

    dmb_barrier();
    const uint32_t s2 = g_epoch_capture_store.seq;
    if (s1 == s2 && (s1 & 1u) == 0u) return out->valid;
  }

  return false;
}

static pps_edge_dispatch_fn g_pps_edge_dispatch = nullptr;
static void pps_edge_dispatch_trampoline(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// DWT cycles-per-second compatibility
// ============================================================================
//
// process_interrupt no longer asks process_time for dynamic CPS.  The legacy
// interrupt_dynamic_cps() now mirrors CLOCKS static PPS/GPIO-derived
// DWT cycles-per-GNSS-second.  Gamma/dynamic prediction has been retired.

static uint32_t interrupt_vclock_cycles_per_second(void);

uint32_t interrupt_dynamic_cps(void) {
  return interrupt_vclock_cycles_per_second();
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

  // Store a reconstructive CPS with each anchor without consulting process_time.
  // The newest anchor uses the just-measured previous PPS_VCLOCK interval as
  // its forward projection denominator, matching the old random-walk policy.
  uint32_t cps = 0;
  bool cps_valid = false;
  if (g_pvc_anchor_count > 0) {
    const pvc_anchor_record_t& prev = g_pvc_anchor_ring[g_pvc_anchor_head];
    if (prev.sequence != 0) {
      cps = pvc.dwt_at_edge - prev.dwt_at_edge;
      cps_valid = (cps != 0);
    }
  }
  g_pvc_anchor_ring[next].cps = cps;
  g_pvc_anchor_ring[next].cps_valid = cps_valid;

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
  int64_t  last_gnss_ns = -1;    // GNSS labels are CLOCKS-owned; unavailable here.
};
static pps_gpio_heartbeat_t g_pps_gpio_heartbeat;

static uint32_t g_gpio_irq_count  = 0;
static uint32_t g_gpio_miss_count = 0;

// ============================================================================
// PPS witness + VCLOCK-domain canonical epoch latch
// ============================================================================
//
// PPS GPIO is a witness/selector only.  It never authors the public
// PPS/VCLOCK DWT coordinate.  The super-canonical PPS/VCLOCK DWT coordinate is now authored from the
// TimePop CH2 fire fact of the VCLOCK cadence client.  Every public DWT timing
// fact therefore lives on the same TimePop/CH2 event-coordinate rail.
//
// During rebootstrap, the PPS ISR selects the VCLOCK counter identity of the
// sacred edge.  The next TimePop VCLOCK cadence event back-projects from its
// own shared CH2 fire fact to author the selected edge's DWT.  After bootstrap,
// every 1000th phase-locked TimePop cadence event authors the next canonical
// PPS/VCLOCK bookend directly.

struct vclock_epoch_latch_t {
  bool     pending = false;
  pps_t    pps{};

  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge = 0;
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

};

static vclock_repair_stats_t g_vclock_repair_stats = {};

// ============================================================================
// VCLOCK GNSS labels
// ============================================================================
//
// process_interrupt intentionally does not own campaign GNSS labels. It owns
// event custody: DWT coordinates, VCLOCK/OCXO counter identities, low-word
// phases, and phase diagnostics. CLOCKS/Alpha/Beta own the campaign GNSS
// timeline and publish pps_ns / pps_vclock_ns / vclock_ns in TIMEBASE_FRAGMENT.
//
// Returning -1 here is deliberate. A local VCLOCK-relative nanosecond value
// would be worse than unavailable because it could be mistaken for CLOCKS'
// campaign GNSS ledger.

static int64_t vclock_gnss_from_counter32(uint32_t) {
  return -1;
}

static uint32_t pvc_anchor_latest_cps(void) {
  if (g_pvc_anchor_count == 0) return 0;
  const pvc_anchor_record_t& a = g_pvc_anchor_ring[g_pvc_anchor_head];
  return (a.cps_valid && a.cps != 0) ? a.cps : 0;
}

static uint32_t interrupt_vclock_cycles_per_second(void) {
  const uint32_t calibrated = clocks_dwt_cycles_per_gnss_second();
  if (calibrated != 0) return calibrated;

  const uint32_t anchor_cps = pvc_anchor_latest_cps();
  if (anchor_cps != 0) return anchor_cps;

  return DWT_EXPECTED_PER_PPS;
}

static uint32_t vclock_cycles_for_ticks(uint32_t vclock_ticks) {
  const uint32_t cycles = interrupt_vclock_cycles_per_second();

  return (uint32_t)(((uint64_t)cycles * (uint64_t)vclock_ticks +
                     (uint64_t)VCLOCK_COUNTS_PER_SECOND / 2ULL) /
                    (uint64_t)VCLOCK_COUNTS_PER_SECOND);
}

static uint32_t pps_vclock_phase_cycles_from_edges(const pps_t& pps,
                                                   const pps_vclock_t& pvc) {
  // Scalar PPS→VCLOCK phase, by definition less than one 10 MHz tick.
  //
  // pvc.dwt_at_edge may be any later VCLOCK edge authored on the TimePop rail.
  // Since all VCLOCK edges are separated by exactly one 10 MHz tick, the
  // phase from physical PPS to the selected first-after-PPS VCLOCK edge is the
  // DWT delta to any VCLOCK edge, modulo the DWT cycles in one 100 ns tick.
  //
  // Use scaled integer arithmetic rather than truncating cycles_per_tick:
  //   phase_scaled = (delta_dwt * 10,000,000) mod dwt_cycles_per_second
  //   phase_cycles = round(phase_scaled / 10,000,000)
  const uint32_t cycles = interrupt_vclock_cycles_per_second();
  const uint32_t delta_dwt = pvc.dwt_at_edge - pps.dwt_at_edge;
  const uint64_t phase_scaled =
      ((uint64_t)delta_dwt * (uint64_t)VCLOCK_COUNTS_PER_SECOND) %
      (uint64_t)cycles;

  return (uint32_t)((phase_scaled +
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

static void publish_vclock_domain_pps_vclock(const pps_t& pps,
                                             uint32_t sequence,
                                             uint32_t dwt_at_edge,
                                             uint32_t counter32_at_edge,
                                             uint16_t ch3_at_edge) {
  pps_vclock_t pvc;
  pvc.sequence = sequence;
  pvc.dwt_at_edge = dwt_at_edge;
  pvc.counter32_at_edge = counter32_at_edge;
  pvc.ch3_at_edge = ch3_at_edge;
  pvc.gnss_ns_at_edge = -1;  // GNSS labels are CLOCKS-owned.

  g_pps_gpio_heartbeat.last_dwt = pvc.dwt_at_edge;
  g_pps_gpio_heartbeat.last_gnss_ns = -1;

  if (g_pvc_anchor_reset_pending) {
    pvc_anchor_ring_reset();
  }

  store_publish(pps, pvc);
  pvc_anchor_publish(pvc);
}

static void publish_selected_epoch_from_vclock_cadence(uint32_t cadence_counter32,
                                                       uint32_t cadence_event_dwt) {
  if (!g_vclock_epoch_latch.pending) return;

  const uint32_t backdate_ticks =
      cadence_counter32 - g_vclock_epoch_latch.counter32_at_edge;
  if (backdate_ticks == 0 || backdate_ticks > VCLOCK_COUNTS_PER_SECOND) {
    return;
  }

  const uint32_t backdate_cycles = vclock_cycles_for_ticks(backdate_ticks);
  const uint32_t anchor_dwt = cadence_event_dwt - backdate_cycles;

  g_vclock_epoch_latch.first_cadence_counter32 = cadence_counter32;
  g_vclock_epoch_latch.first_cadence_dwt = cadence_event_dwt;
  g_vclock_epoch_latch.backdate_ticks = backdate_ticks;
  g_vclock_epoch_latch.backdate_cycles = backdate_cycles;
  g_vclock_epoch_latch.sacred_dwt = anchor_dwt;

  publish_vclock_domain_pps_vclock(g_vclock_epoch_latch.pps,
                                    g_vclock_epoch_latch.pps.sequence,
                                    anchor_dwt,
                                    g_vclock_epoch_latch.counter32_at_edge,
                                    g_vclock_epoch_latch.ch3_at_edge);

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

    if (!a.cps_valid || a.cps == 0) {
      out.anchor_failure_mask |= ANCHOR_FAIL_NO_CPS;
      continue;
    }
    const uint32_t cps = a.cps;

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

static int64_t interrupt_project_dwt_to_vclock_gnss_ns(uint32_t dwt_at_event) {
  return interrupt_dwt_to_vclock_gnss_projection(dwt_at_event).gnss_ns;
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
  { interrupt_subscriber_kind_t::VCLOCK, "VCLOCK", interrupt_provider_kind_t::QTIMER1, interrupt_lane_t::QTIMER1_CH2_COMP },
  { interrupt_subscriber_kind_t::OCXO1,  "OCXO1",  interrupt_provider_kind_t::QTIMER2, interrupt_lane_t::QTIMER2_CH0_COMP },
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
  bool     active = false;             // subscriber/campaign interest
  bool     scheduler_active = false;   // lane-local 1 kHz custody running
  bool     second_grid_valid = false;  // CLOCKS-authored epoch grid installed
  bool     phase_bootstrapped = false;

  // Hardware compare target for the next 1 kHz custody fire.  The low word is
  // written to COMP1/CMPLD1; next_cadence_counter32 is the full synthetic
  // target identity authored by this lane's private scheduler.
  uint16_t compare_target = 0;
  uint32_t next_cadence_counter32 = 0;

  // CLOCKS-authored OCXO one-second grid.  Every 1000th cadence target that
  // equals next_edge_counter32 is emitted to Alpha as an OCXO second edge.
  uint32_t grid_epoch_counter32 = 0;
  uint32_t next_edge_counter32 = 0;

  uint32_t tick_mod_1000 = 0;
  uint32_t logical_count32_at_last_second = 0;
  uint32_t irq_count = 0;
  uint32_t miss_count = 0;
  uint32_t bootstrap_count = 0;
  uint32_t cadence_hits_total = 0;
  uint32_t edge_hits_total = 0;
  uint32_t second_event_count = 0;
  uint32_t deferred_edge_count = 0;
  uint32_t deferred_overwrite_count = 0;
  uint32_t late_max_ticks = 0;
  uint32_t last_late_ticks = 0;
  uint32_t skipped_cadence_count = 0;
  uint32_t missed_second_count = 0;
  uint32_t compare_rearm_count = 0;
  uint32_t compare_rearm_failure_count = 0;
  uint32_t last_fired_counter32 = 0;
  uint16_t last_fired_low16 = 0;
};
static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind);
static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane);

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
  uint64_t current_ns = 0;
  uint16_t hardware16 = 0;
  uint32_t zero_count = 0;
  uint32_t minder_update_count = 0;

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

static volatile uint32_t g_qtimer1_ch2_last_target_counter32 = 0;
static volatile uint32_t g_qtimer1_ch2_arm_count = 0;

static volatile uint32_t g_qtimer2_ch0_count = 0;
static volatile uint32_t g_qtimer2_last_ch0_late_ticks = 0;
static volatile uint32_t g_qtimer2_last_ch0_dwt_raw = 0;
static volatile uint32_t g_qtimer2_last_ch0_event_dwt = 0;
static volatile uint32_t g_qtimer2_last_ch0_dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;

static volatile uint32_t g_qtimer3_ch3_count = 0;
static volatile uint32_t g_qtimer3_last_ch3_late_ticks = 0;
static volatile uint32_t g_qtimer3_last_ch3_dwt_raw = 0;
static volatile uint32_t g_qtimer3_last_ch3_event_dwt = 0;
static volatile uint32_t g_qtimer3_last_ch3_dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;

static volatile uint32_t g_ocxo_deferred_asap_arm_count = 0;
static volatile uint32_t g_ocxo_deferred_asap_overwrite_count = 0;
static volatile uint32_t g_ocxo_deferred_sample_count = 0;
static volatile uint32_t g_ocxo_deferred_edge_count = 0;

static volatile bool     g_cadence_minder_armed = false;
static volatile uint32_t g_cadence_minder_arm_count = 0;
static volatile uint32_t g_cadence_minder_arm_failures = 0;
static volatile uint32_t g_cadence_minder_fire_count = 0;
static volatile uint32_t g_cadence_minder_vclock_updates = 0;
static volatile uint32_t g_cadence_minder_ocxo1_updates = 0;
static volatile uint32_t g_cadence_minder_ocxo2_updates = 0;
static volatile uint16_t g_cadence_minder_last_vclock_hw16 = 0;
static volatile uint16_t g_cadence_minder_last_ocxo1_hw16 = 0;
static volatile uint16_t g_cadence_minder_last_ocxo2_hw16 = 0;
static volatile uint32_t g_cadence_minder_last_vclock_counter32 = 0;
static volatile uint32_t g_cadence_minder_last_ocxo1_counter32 = 0;
static volatile uint32_t g_cadence_minder_last_ocxo2_counter32 = 0;

static volatile uint32_t g_ocxo1_edge_fire_count = 0;
static volatile uint32_t g_ocxo2_edge_fire_count = 0;

// CLOCKS-owned OCXO logical grid.  The grid is consumed by the private OCXO
// mini-schedulers; no TimePop one-shot cancellation or arm window exists here.
static volatile bool     g_ocxo_grid_epoch_valid = false;
static volatile uint32_t g_ocxo1_grid_epoch_counter32 = 0;
static volatile uint32_t g_ocxo2_grid_epoch_counter32 = 0;
static volatile uint32_t g_ocxo1_grid_epoch_start_count = 0;
static volatile uint32_t g_ocxo2_grid_epoch_start_count = 0;
static volatile uint32_t g_ocxo1_grid_epoch_wait_count = 0;
static volatile uint32_t g_ocxo2_grid_epoch_wait_count = 0;

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
  c.current_ns = ns;
  c.hardware16 = (uint16_t)c.current_counter32;
  c.zero_count++;
  c.pending_zero = false;
}

static void vclock_clock_anchor_hardware_low16(uint32_t synthetic_counter32,
                                                 uint16_t hardware_low16) {
  g_vclock_clock32.current_counter32 = synthetic_counter32;
  g_vclock_clock32.current_ns = (uint64_t)synthetic_counter32 * 100ULL;
  g_vclock_clock32.hardware16 = hardware_low16;
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

static void synthetic_clock_bootstrap_from_hw16(synthetic_clock32_t& c, uint16_t hardware16);
static void vclock_clock_bootstrap_from_hw16(uint16_t hardware16);
static uint32_t project_counter32_from_hw16(const synthetic_clock32_t& c, uint16_t hardware16);
static inline void synthetic_clock_consume_pending_zero_if_any(synthetic_clock32_t& c);

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

static inline uint32_t synthetic_clock_advance_at_hardware(synthetic_clock32_t& c,
                                                           uint32_t ticks,
                                                           uint16_t hardware16_at_event) {
  synthetic_clock_consume_pending_zero_if_any(c);
  c.current_counter32 += ticks;
  c.current_ns += (uint64_t)ticks * 100ULL;
  c.hardware16 = hardware16_at_event;
  return c.current_counter32;
}

uint32_t interrupt_qtimer1_counter32_now(void)   { return vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now()); }
uint16_t interrupt_qtimer2_ch0_counter_now(void) { return IMXRT_TMR2.CH[0].CNTR; }
uint16_t interrupt_qtimer3_ch3_counter_now(void) { return IMXRT_TMR3.CH[3].CNTR; }

// Forward declarations for compare helpers defined below.
static void qtimer2_ch0_program_compare(uint16_t target_low16);
static void qtimer3_program_compare(uint8_t ch, uint16_t target_low16);

static inline uint32_t cadence_mod_for_ticks(uint32_t ticks, uint32_t interval) {
  if (interval == 0) return 0;
  return (ticks / interval) % TICKS_PER_SECOND_EVENT;
}

static uint32_t project_counter32_from_hw16(const synthetic_clock32_t& c,
                                            uint16_t hardware16) {
  // Boot-safety rule: before a lane has been explicitly born/zeroed, raw
  // hardware low-word observations are still safe diagnostic coordinates.
  // Do not project from an uninitialized synthetic anchor.  This preserves
  // the old pre-epoch behavior and keeps early PPS/REPORT paths harmless.
  if (!c.zeroed) return (uint32_t)hardware16;

  return c.current_counter32 + (uint32_t)((uint16_t)(hardware16 - c.hardware16));
}

static uint64_t project_ns64_from_hw16(const synthetic_clock32_t& c,
                                       uint16_t hardware16) {
  // Same boot-safety rule as COUNT32 projection.  This value is not global
  // truth before ZERO; it is only a harmless local low-word-derived surface.
  if (!c.zeroed) return (uint64_t)hardware16 * 100ULL;

  const uint32_t delta = (uint32_t)((uint16_t)(hardware16 - c.hardware16));
  return c.current_ns + (uint64_t)delta * 100ULL;
}

static void synthetic_clock_bootstrap_from_hw16(synthetic_clock32_t& c,
                                                uint16_t hardware16) {
  // This is not a user ZERO.  It merely gives the synthetic extender a
  // coherent birth anchor so early ISR/report paths never project from an
  // all-zero default object.
  c.zeroed = true;
  c.zero_ns = (uint64_t)hardware16 * 100ULL;
  c.zero_counter32 = (uint32_t)hardware16;
  c.current_counter32 = (uint32_t)hardware16;
  c.current_ns = (uint64_t)hardware16 * 100ULL;
  c.hardware16 = hardware16;
  c.pending_zero = false;
}

static void vclock_clock_bootstrap_from_hw16(uint16_t hardware16) {
  // Birth the VCLOCK synthetic layer without declaring a logical ZERO.  The
  // PPS/VCLOCK rebootstrap and CLOCKS.ZERO paths may later install the real
  // coordinate origin, but the interrupt layer is safe immediately after IRQs
  // go live.
  g_vclock_clock32.zeroed = true;
  g_vclock_clock32.zero_ns = (uint64_t)hardware16 * 100ULL;
  g_vclock_clock32.zero_counter32 = (uint32_t)hardware16;
  g_vclock_clock32.hardware_low16_at_zero = hardware16;
  g_vclock_clock32.pending_zero = false;
  vclock_clock_anchor_hardware_low16((uint32_t)hardware16, hardware16);
  g_vclock_lane.logical_count32_at_last_second = (uint32_t)hardware16;
}

static void vclock_clock_tend_from_hardware_low16(uint16_t hardware_low16) {
  if (!g_vclock_clock32.zeroed) {
    vclock_clock_bootstrap_from_hw16(hardware_low16);
    return;
  }

  const uint32_t counter32 = vclock_synthetic_from_hardware_low16(hardware_low16);
  vclock_clock_anchor_hardware_low16(counter32, hardware_low16);
  g_vclock_clock32.minder_update_count++;
}

static void synthetic_clock_tend_from_hw16(synthetic_clock32_t& c,
                                           uint16_t hardware16) {
  if (!c.zeroed) {
    synthetic_clock_bootstrap_from_hw16(c, hardware16);
    return;
  }

  synthetic_clock_consume_pending_zero_if_any(c);
  const uint32_t counter32 = project_counter32_from_hw16(c, hardware16);
  const uint32_t delta = counter32 - c.current_counter32;
  c.current_counter32 = counter32;
  c.current_ns += (uint64_t)delta * 100ULL;
  c.hardware16 = hardware16;
  c.minder_update_count++;
}

bool interrupt_clock_snapshot(interrupt_subscriber_kind_t kind, interrupt_clock_snapshot_t* out) {
  if (!out) return false;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    const uint16_t hw = qtimer1_ch0_counter_now();
    out->hardware16 = hw;
    out->counter32 = vclock_synthetic_from_hardware_low16(hw);
    out->ns64 = project_ns64_from_hw16(g_vclock_clock32, hw);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    const uint16_t hw = IMXRT_TMR2.CH[0].CNTR;
    out->hardware16 = hw;
    out->counter32 = project_counter32_from_hw16(g_ocxo1_clock32, hw);
    out->ns64 = project_ns64_from_hw16(g_ocxo1_clock32, hw);
    return true;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    const uint16_t hw = IMXRT_TMR3.CH[3].CNTR;
    out->hardware16 = hw;
    out->counter32 = project_counter32_from_hw16(g_ocxo2_clock32, hw);
    out->ns64 = project_ns64_from_hw16(g_ocxo2_clock32, hw);
    return true;
  }

  return false;
}

// ============================================================================
// Compare channel helpers
// ============================================================================

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

static inline void qtimer2_ch0_clear_compare_flag(void) {
  // Empirical ZPNet result: OCXO QuadTimer compare flags clear by writing the
  // TCF bits LOW in CSCTRL.  The former write-one-to-clear assumption leaves
  // TCF1 latched and can create an interrupt storm when TCF1EN is enabled.
  IMXRT_TMR2.CH[0].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)IMXRT_TMR2.CH[0].CSCTRL;
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
  qtimer2_ch0_clear_compare_flag();
}

static inline void qtimer3_clear_compare_flag(uint8_t ch) {
  // Empirical ZPNet result: OCXO QuadTimer compare flags clear by writing the
  // TCF bits LOW in CSCTRL.  The former write-one-to-clear assumption leaves
  // TCF1 latched and can create an interrupt storm when TCF1EN is enabled.
  IMXRT_TMR3.CH[ch].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)IMXRT_TMR3.CH[ch].CSCTRL;
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
  rt.dispatch_count++;
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
                                  const dwt_repair_diag_t* repair = nullptr,
                                  bool dispatch_immediately = false) {
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
  // OCXO lanes try the DWT bridge for diagnostics, but dispatch does not
  // depend on projection success. Alpha constructs OCXO measured-GNSS
  // intervals from consecutive OCXO edge DWT facts.
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

  if (dispatch_immediately) {
    maybe_dispatch_event(rt);
  } else {
    timepop_arm_asap(deferred_dispatch_callback, &rt, dispatch_timer_name(rt.desc->kind));
  }
}


// ============================================================================
// Cadence minder — single TimePop custody agent for VCLOCK only
// ============================================================================
//
// OCXO1 and OCXO2 are deliberately absent here.  Their 16-bit rollover custody
// is handled by their own QTimer2/QTimer3 1 kHz mini-schedulers, so no OCXO
// ISR path ever mutates TimePop slots.

static void cadence_minder_timepop_callback(timepop_ctx_t* ctx,
                                            timepop_diag_t*,
                                            void*) {
  if (!ctx || !g_interrupt_hw_ready) return;

  const uint32_t qtimer_event_dwt = ctx->fire_dwt_cyccnt;
  const uint32_t cadence_counter32 = ctx->fire_vclock_raw;
  const uint16_t fired_low16 = (uint16_t)(cadence_counter32 & 0xFFFFU);

  g_cadence_minder_fire_count++;

  // VCLOCK: CADENCE_MINDER is now the only 1 ms substrate cadence.  It owns
  // the synthetic VCLOCK low-word anchor refresh and the former separate
  // VCLOCK one-second/bookend duties.
  vclock_clock_anchor_hardware_low16(cadence_counter32, fired_low16);
  g_vclock_clock32.minder_update_count++;
  g_vclock_lane.logical_count32_at_last_second = cadence_counter32;
  g_cadence_minder_vclock_updates++;
  g_cadence_minder_last_vclock_hw16 = fired_low16;
  g_cadence_minder_last_vclock_counter32 = cadence_counter32;

  if (g_rt_vclock) {
    g_rt_vclock->irq_count++;
  }
  g_vclock_lane.irq_count++;

  if (g_vclock_lane.active && g_rt_vclock && g_rt_vclock->active) {
    if (!g_vclock_lane.phase_bootstrapped) {
      g_vclock_lane.phase_bootstrapped = true;
      g_vclock_lane.bootstrap_count++;
      g_vclock_lane.tick_mod_1000 = 0;
    }

    g_vclock_lane.cadence_hits_total++;

    bool vclock_epoch_consumed_this_fire = false;
    if (g_vclock_epoch_latch.pending) {
      const uint32_t selected_to_cadence_ticks =
          cadence_counter32 - g_vclock_epoch_latch.counter32_at_edge;
      publish_selected_epoch_from_vclock_cadence(cadence_counter32,
                                                 qtimer_event_dwt);
      if (!g_vclock_epoch_latch.pending) {
        g_vclock_lane.tick_mod_1000 =
            (selected_to_cadence_ticks / VCLOCK_INTERVAL_COUNTS) %
            TICKS_PER_SECOND_EVENT;
        vclock_epoch_consumed_this_fire = true;
      }
    }

    if (!vclock_epoch_consumed_this_fire &&
        ++g_vclock_lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
      g_vclock_lane.tick_mod_1000 = 0;

      // PPS_VCLOCK / PPS coincidence diagnostic.
      uint32_t coincidence_cycles = 0;
      bool     coincidence_valid  = false;
      {
        const pps_vclock_t pvc = store_load_pvc();
        if (pvc.sequence > 0) {
          const int32_t cycles_since_pvc =
              (int32_t)(qtimer_event_dwt - pvc.dwt_at_edge);
          if (llabs((long long)cycles_since_pvc) <
              DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) {
            coincidence_cycles =
                (uint32_t)(cycles_since_pvc >= 0 ? cycles_since_pvc
                                                 : -cycles_since_pvc);
            coincidence_valid = true;
          }
        }
      }

      const pps_t witness_pps = g_last_pps_witness_valid
          ? g_last_pps_witness
          : pps_t{};
      const dwt_repair_diag_t repair =
          vclock_endpoint_repair_diagnostic(qtimer_event_dwt);
      publish_vclock_domain_pps_vclock(witness_pps,
                                        (witness_pps.sequence != 0)
                                            ? witness_pps.sequence
                                            : (g_pps_gpio_heartbeat.edge_count + 1),
                                        qtimer_event_dwt,
                                        cadence_counter32,
                                        fired_low16);

      emit_one_second_event(*g_rt_vclock, qtimer_event_dwt,
                            cadence_counter32,
                            coincidence_cycles, coincidence_valid, &repair);

      if (g_pps_edge_dispatch) {
        timepop_arm_asap(pps_edge_dispatch_trampoline,
                         nullptr,
                         "PPS_VCLOCK_DISPATCH");
      }
    }
  } else {
    g_vclock_lane.miss_count++;
  }

  // OCXO lanes intentionally not touched.
}

static bool cadence_minder_arm_timepop(void) {
  if (!g_interrupt_hw_ready || !g_interrupt_runtime_ready) return false;

  timepop_cancel_by_name(CADENCE_MINDER_NAME);
  const timepop_handle_t h =
      timepop_arm_recurring_isr(CADENCE_MINDER_PERIOD_NS,
                                cadence_minder_timepop_callback,
                                nullptr,
                                CADENCE_MINDER_NAME);
  if (h == TIMEPOP_INVALID_HANDLE) {
    g_cadence_minder_armed = false;
    g_cadence_minder_arm_failures++;
    return false;
  }

  g_cadence_minder_armed = true;
  g_cadence_minder_arm_count++;
  g_vclock_lane.phase_bootstrapped = true;
  g_vclock_lane.bootstrap_count++;
  return true;
}

// ============================================================================
// OCXO lanes — private QTimer2/QTimer3 1 kHz mini-schedulers
// ============================================================================
//
// OCXO1 and OCXO2 are now self-sufficient timing domains.  TimePop does not
// arm OCXO compares, does not drain OCXO post-ISR work, and does not tend OCXO
// low-word rollover.  Each lane owns one recurring hardware compare chain:
//
//   target[n + 1] = target[n] + 10,000 OCXO ticks
//
// Most cadence fires are custody-only.  When the full 32-bit cadence target
// equals the CLOCKS-authored one-second grid target, the lane captures the
// latency-adjusted DWT at that OCXO edge and dispatches the subscriber callback
// directly.  This first-cut path intentionally passes the quantized QuadTimer
// DWT-at-edge through to CLOCKS/Alpha without SpinCatch or dequantization.

static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].CNTR;
}

static inline bool ocxo_lane_compare_flag_pending(const ocxo_lane_t& lane) {
  return (lane.module->CH[lane.channel].CSCTRL & TMR_CSCTRL_TCF1) != 0;
}

static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_clear_compare_flag();
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_clear_compare_flag(lane.channel);
  }
}

static void ocxo_lane_clear_nvic_pending(const ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    NVIC_CLEAR_PENDING(IRQ_QTIMER2);
  } else if (lane.module == &IMXRT_TMR3) {
    NVIC_CLEAR_PENDING(IRQ_QTIMER3);
  }
}

static void ocxo_lane_disable_compare_gate_only(ocxo_lane_t& lane) {
  lane.module->CH[lane.channel].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  (void)lane.module->CH[lane.channel].CSCTRL;
}

static void ocxo_lane_disable_compare(ocxo_lane_t& lane) {
  ocxo_lane_disable_compare_gate_only(lane);
  ocxo_lane_clear_compare_flag(lane);
  ocxo_lane_clear_compare_flag(lane);
  ocxo_lane_clear_nvic_pending(lane);
}

static bool ocxo_lane_program_cadence_compare(ocxo_lane_t& lane,
                                              uint16_t target_low16) {
  // The compare target is always the next lane-local 1 kHz grid point.  Since
  // the interval is 10,000 ticks (< 2^16), a low-word COMP1 match uniquely
  // refers to the next scheduled target as long as this ISR is serviced within
  // one 16-bit wrap, which is the same custody assumption the 1 kHz cadence is
  // designed to enforce.
  ocxo_lane_disable_compare_gate_only(lane);
  ocxo_lane_clear_compare_flag(lane);

  lane.module->CH[lane.channel].COMP1  = target_low16;
  lane.module->CH[lane.channel].CMPLD1 = target_low16;

  ocxo_lane_clear_compare_flag(lane);
  if (ocxo_lane_compare_flag_pending(lane)) {
    ocxo_lane_disable_compare(lane);
    return false;
  }

  lane.module->CH[lane.channel].CSCTRL |= TMR_CSCTRL_TCF1EN;
  return true;
}

static void ocxo_lane_record_isr_diag(interrupt_subscriber_kind_t kind,
                                      uint32_t isr_entry_dwt_raw,
                                      uint32_t event_dwt,
                                      uint32_t late_ticks) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_qtimer2_ch0_count++;
    g_qtimer2_last_ch0_dwt_raw = isr_entry_dwt_raw;
    g_qtimer2_last_ch0_event_dwt = event_dwt;
    g_qtimer2_last_ch0_dwt_coordinate_source = OCXO_DWT_SOURCE_ISR_ENTRY;
    g_qtimer2_last_ch0_late_ticks = late_ticks;
    return;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    g_qtimer3_ch3_count++;
    g_qtimer3_last_ch3_dwt_raw = isr_entry_dwt_raw;
    g_qtimer3_last_ch3_event_dwt = event_dwt;
    g_qtimer3_last_ch3_dwt_coordinate_source = OCXO_DWT_SOURCE_ISR_ENTRY;
    g_qtimer3_last_ch3_late_ticks = late_ticks;
  }
}

static void ocxo_edge_fire_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_fire_count++;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    g_ocxo2_edge_fire_count++;
  }
}

struct ocxo_deferred_work_t {
  volatile bool     edge_pending = false;
  volatile uint32_t edge_dwt = 0;
  volatile uint32_t edge_counter32 = 0;
  volatile uint32_t sequence = 0;
};

static ocxo_deferred_work_t g_ocxo1_deferred = {};
static ocxo_deferred_work_t g_ocxo2_deferred = {};

static ocxo_deferred_work_t& ocxo_deferred_for(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_deferred
      : g_ocxo2_deferred;
}

static void ocxo_defer_one_second_edge(ocxo_lane_t& lane,
                                       ocxo_deferred_work_t& work,
                                       uint32_t event_dwt,
                                       uint32_t event_counter32) {
  if (work.edge_pending) {
    lane.deferred_overwrite_count++;
    g_ocxo_deferred_asap_overwrite_count++;  // legacy counter name
  }

  work.edge_dwt = event_dwt;
  work.edge_counter32 = event_counter32;
  work.sequence++;
  work.edge_pending = true;

  lane.deferred_edge_count++;
  g_ocxo_deferred_edge_count++;
}

static inline void ocxo_lane_update_late_diag(ocxo_lane_t& lane,
                                              uint32_t late_ticks) {
  lane.last_late_ticks = late_ticks;
  if (late_ticks > lane.late_max_ticks) {
    lane.late_max_ticks = late_ticks;
  }
}

static inline bool counter32_future(uint32_t target, uint32_t now) {
  const uint32_t delta = target - now;
  return delta != 0 && delta < 0x80000000UL;
}

static uint32_t next_ocxo_grid_target_after(uint32_t epoch_counter32,
                                            uint32_t current_counter32,
                                            uint32_t interval_counts) {
  const uint32_t delta = current_counter32 - epoch_counter32;
  const uint32_t intervals_completed = (delta / interval_counts) + 1U;
  return epoch_counter32 + intervals_completed * interval_counts;
}

static uint32_t next_ocxo_second_target_after(uint32_t epoch_counter32,
                                              uint32_t current_counter32) {
  return next_ocxo_grid_target_after(epoch_counter32,
                                     current_counter32,
                                     OCXO_EDGE_INTERVAL_COUNTS);
}

static uint32_t next_ocxo_cadence_target_after(uint32_t epoch_counter32,
                                               uint32_t current_counter32) {
  return next_ocxo_grid_target_after(epoch_counter32,
                                     current_counter32,
                                     OCXO_CADENCE_INTERVAL_COUNTS);
}

static void ocxo_lane_anchor_clock_to_event(synthetic_clock32_t& clock32,
                                            uint32_t event_counter32,
                                            uint16_t event_low16) {
  synthetic_clock_consume_pending_zero_if_any(clock32);
  if (!clock32.zeroed) {
    synthetic_clock_bootstrap_from_hw16(clock32, event_low16);
  }

  const uint32_t delta = event_counter32 - clock32.current_counter32;
  clock32.current_counter32 = event_counter32;
  clock32.current_ns += (uint64_t)delta * 100ULL;
  clock32.hardware16 = event_low16;
  clock32.minder_update_count++;  // retained name: now lane-local custody count
}

static bool ocxo_lane_arm_next_compare(ocxo_lane_t& lane,
                                       uint32_t next_counter32) {
  lane.next_cadence_counter32 = next_counter32;
  lane.compare_target = (uint16_t)(next_counter32 & 0xFFFFU);

  if (!ocxo_lane_program_cadence_compare(lane, lane.compare_target)) {
    lane.compare_rearm_failure_count++;
    return false;
  }

  lane.compare_rearm_count++;
  lane.scheduler_active = true;
  return true;
}

static bool ocxo_lane_start_custody_scheduler(ocxo_lane_t& lane,
                                              synthetic_clock32_t& clock32) {
  if (!lane.initialized) return false;

  ocxo_lane_disable_compare(lane);

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);
  lane.logical_count32_at_last_second = clock32.current_counter32;

  const uint32_t next_counter32 =
      clock32.current_counter32 + OCXO_CADENCE_INTERVAL_COUNTS;

  lane.phase_bootstrapped = true;
  lane.bootstrap_count++;
  return ocxo_lane_arm_next_compare(lane, next_counter32);
}

static void ocxo_grid_epoch_wait_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_grid_epoch_wait_count++;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    g_ocxo2_grid_epoch_wait_count++;
  }
}

static void ocxo_grid_epoch_start_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_grid_epoch_start_count++;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    g_ocxo2_grid_epoch_start_count++;
  }
}

static bool ocxo_lane_install_epoch_grid(interrupt_subscriber_kind_t kind,
                                         ocxo_lane_t& lane,
                                         synthetic_clock32_t& clock32,
                                         uint32_t epoch_counter32) {
  if (!lane.initialized) return false;

  ocxo_lane_disable_compare(lane);

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);
  lane.logical_count32_at_last_second = clock32.current_counter32;

  lane.grid_epoch_counter32 = epoch_counter32;
  lane.next_edge_counter32 = next_ocxo_second_target_after(epoch_counter32,
                                                           clock32.current_counter32);
  const uint32_t next_cadence = next_ocxo_cadence_target_after(epoch_counter32,
                                                               clock32.current_counter32);
  lane.second_grid_valid = true;
  lane.tick_mod_1000 =
      ((next_cadence - epoch_counter32) / OCXO_CADENCE_INTERVAL_COUNTS) %
      OCXO_CADENCE_EVENTS_PER_SECOND;

  ocxo_grid_epoch_start_inc(kind);
  return ocxo_lane_arm_next_compare(lane, next_cadence);
}

static void ocxo_schedule_epoch_grid_if_ready(interrupt_subscriber_runtime_t* rt) {
  if (!rt || !rt->desc) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized) return;

  if (!g_ocxo_grid_epoch_valid) {
    ocxo_grid_epoch_wait_inc(kind);
    return;
  }

  const uint32_t epoch_counter32 =
      (kind == interrupt_subscriber_kind_t::OCXO1)
          ? g_ocxo1_grid_epoch_counter32
          : g_ocxo2_grid_epoch_counter32;

  (void)ocxo_lane_install_epoch_grid(kind, *lane, *clock32, epoch_counter32);
}

static void ocxo_lane_reschedule_after_fire(ocxo_lane_t& lane,
                                            uint32_t fired_counter32,
                                            uint16_t fired_low16) {
  const uint16_t now_low16 = ocxo_lane_counter_now(lane);
  const uint32_t now_counter32 =
      fired_counter32 + (uint32_t)((uint16_t)(now_low16 - fired_low16));

  uint32_t next = fired_counter32 + OCXO_CADENCE_INTERVAL_COUNTS;
  uint32_t skipped = 0;
  while (!counter32_future(next, now_counter32) && skipped < 2048U) {
    next += OCXO_CADENCE_INTERVAL_COUNTS;
    skipped++;
  }

  if (!counter32_future(next, now_counter32)) {
    // Extreme catch-up fallback: if the ISR was blocked for longer than the
    // bounded skip loop, restart the custody cadence one interval ahead of the
    // best lane-local now estimate.  This is a visible integrity event, not a
    // silent correction.
    next = now_counter32 + OCXO_CADENCE_INTERVAL_COUNTS;
    skipped++;
  }

  if (skipped > 0) {
    lane.skipped_cadence_count += skipped;
  }

  if (lane.second_grid_valid) {
    // If catch-up skipped over an epoch-aligned second target, we cannot emit
    // it honestly because no ISR-entry DWT was captured at that target.  Advance
    // the second grid and make the miss visible.
    while (lane.next_edge_counter32 != 0) {
      const uint32_t edge_from_fired = lane.next_edge_counter32 - fired_counter32;
      const uint32_t next_from_fired = next - fired_counter32;
      if (edge_from_fired != 0 && edge_from_fired < next_from_fired) {
        lane.next_edge_counter32 += OCXO_EDGE_INTERVAL_COUNTS;
        lane.missed_second_count++;
        continue;
      }
      break;
    }

    lane.tick_mod_1000 =
        ((next - lane.grid_epoch_counter32) / OCXO_CADENCE_INTERVAL_COUNTS) %
        OCXO_CADENCE_EVENTS_PER_SECOND;
  }

  (void)ocxo_lane_arm_next_compare(lane, next);
}

static void ocxo_lane_cadence_isr(ocxo_lane_t& lane,
                                  interrupt_subscriber_runtime_t* rt,
                                  interrupt_subscriber_kind_t kind,
                                  synthetic_clock32_t& clock32,
                                  ocxo_deferred_work_t& work,
                                  uint32_t isr_entry_dwt_raw) {
  (void)work;

  if (!ocxo_lane_compare_flag_pending(lane)) {
    lane.miss_count++;
    return;
  }

  if (!lane.scheduler_active) {
    ocxo_lane_disable_compare(lane);
    lane.miss_count++;
    return;
  }

  const uint32_t fired_counter32 = lane.next_cadence_counter32;
  const uint16_t fired_low16 = lane.compare_target;
  const uint32_t event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);

  ocxo_lane_clear_compare_flag(lane);

  lane.irq_count++;
  lane.cadence_hits_total++;
  lane.last_fired_counter32 = fired_counter32;
  lane.last_fired_low16 = fired_low16;
  if (rt) rt->irq_count++;

  const uint32_t late_ticks =
      (uint32_t)((uint16_t)(ocxo_lane_counter_now(lane) - fired_low16));
  ocxo_lane_update_late_diag(lane, late_ticks);
  ocxo_lane_record_isr_diag(kind, isr_entry_dwt_raw, event_dwt, late_ticks);

  ocxo_lane_anchor_clock_to_event(clock32, fired_counter32, fired_low16);
  lane.logical_count32_at_last_second = fired_counter32;

  const bool emit_second =
      lane.second_grid_valid &&
      lane.active && rt && rt->active &&
      fired_counter32 == lane.next_edge_counter32;

  if (emit_second) {
    lane.edge_hits_total++;
    lane.second_event_count++;
    ocxo_edge_fire_inc(kind);

    // First-cut OCXO delivery: pass the honest, latency-adjusted, still-
    // quantized QTimer DWT-at-edge directly to CLOCKS/Alpha.  The former
    // SpinCatch/prespin mailbox path is retained only as a compatibility
    // drain for any stale pending work; normal OCXO events no longer depend
    // on it.
    emit_one_second_event(*rt, event_dwt, fired_counter32,
                          0, false, nullptr, true);

    lane.next_edge_counter32 += OCXO_EDGE_INTERVAL_COUNTS;
  }

  ocxo_lane_reschedule_after_fire(lane, fired_counter32, fired_low16);
}

static void ocxo_drain_deferred_edge(interrupt_subscriber_runtime_t* rt,
                                     ocxo_lane_t& lane,
                                     ocxo_deferred_work_t& work) {
  uint32_t event_dwt = 0;
  uint32_t event_counter32 = 0;
  bool had_edge = false;

  __disable_irq();
  if (work.edge_pending) {
    event_dwt = work.edge_dwt;
    event_counter32 = work.edge_counter32;
    work.edge_pending = false;
    had_edge = true;
  }
  __enable_irq();

  if (!had_edge || !rt || !rt->active) return;

  // Foreground direct dispatch: no TimePop slot mutation.  Alpha receives the
  // same interrupt_event_t / diag surface, with the OCXO DWT coordinate already
  // latency-adjusted at ISR entry.
  emit_one_second_event(*rt, event_dwt, event_counter32, 0, false, nullptr, true);
  (void)lane;
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  ocxo_lane_cadence_isr(g_ocxo1_lane,
                        g_rt_ocxo1,
                        interrupt_subscriber_kind_t::OCXO1,
                        g_ocxo1_clock32,
                        g_ocxo1_deferred,
                        isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  ocxo_lane_cadence_isr(g_ocxo2_lane,
                        g_rt_ocxo2,
                        interrupt_subscriber_kind_t::OCXO2,
                        g_ocxo2_clock32,
                        g_ocxo2_deferred,
                        isr_entry_dwt_raw);
}

// ============================================================================
// QTimer1 vector — dispatches CH2 (TimePop)
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

static void qtimer1_ch1_schedule_next_hop(void) {
  if (!g_qtimer1_ch1_active) return;

  const uint32_t now = vclock_synthetic_from_hardware_low16(qtimer1_ch0_counter_now());
  const uint32_t remaining = g_qtimer1_ch1_target_counter32 - now;

  // If the target is behind us in modular time, stop rather than generating
  // misleading event facts.  The caller can re-arm from a fresh PPS anchor.
  if (remaining == 0 || remaining > 0x7FFFFFFFUL) {
    g_qtimer1_ch1_active = false;
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  static constexpr uint32_t CH1_MAX_COMPARE_STEP_TICKS = 49123U;

  const uint32_t step =
      (remaining > CH1_MAX_COMPARE_STEP_TICKS)
          ? CH1_MAX_COMPARE_STEP_TICKS
          : remaining;

  g_qtimer1_ch1_next_compare_counter32 = now + step;

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
  qtimer1_ch1_disable_compare_hw();
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

  if (!g_qtimer1_ch1_active) {
    qtimer1_ch1_disable_compare_hw();
    return;
  }

  const uint32_t fired_counter32 = g_qtimer1_ch1_next_compare_counter32;

  if (fired_counter32 != g_qtimer1_ch1_target_counter32) {
    g_qtimer1_ch1_hop_count++;
    qtimer1_ch1_schedule_next_hop();
    return;
  }

  g_qtimer1_ch1_active = false;
  qtimer1_ch1_disable_compare_hw();
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
  // CH1/CH2 share one vector.  If multiple flags are pending, the
  // unhandled flag remains set and NVIC re-dispatches with a fresh
  // first-instruction _raw capture.  VCLOCK cadence no longer owns CH3; it
  // is a normal TimePop client on CH2.
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
  // authored by the TimePop VCLOCK cadence client on QTimer1 CH2 so all public
  // DWT captures live in one TimePop event-coordinate system.
  //
  // Zero-offset capture doctrine: immediately after the first-instruction
  // DWT raw capture, read the three 10 MHz lane counters in one custody
  // window.  The raw 16-bit reads are retained as forensic-only evidence;
  // runtime timing math consumes the synthetic 32-bit lane coordinates below.
  const uint32_t epoch_capture_start_raw = isr_entry_dwt_raw;
  const uint16_t hardware_low16 = qtimer1_ch0_counter_now();
  const uint32_t epoch_capture_after_vclock_raw = ARM_DWT_CYCCNT;

  // Defensive boot rule: VCLOCK is mandatory and QTimer1 CH0 is initialized
  // before IRQs are enabled.  OCXO raw reads are desirable, but they are not
  // allowed to become a boot-time dependency unless QTimer3 lane hardware is
  // known initialized.  This keeps PPS witness/selector work alive even if an
  // OCXO lane is not ready yet.
  const bool ocxo_capture_hw_ready =
      g_interrupt_hw_ready && g_ocxo1_lane.initialized && g_ocxo2_lane.initialized;
  const uint16_t ocxo1_hardware16 = ocxo_capture_hw_ready ? IMXRT_TMR2.CH[0].CNTR : 0;
  const uint16_t ocxo2_hardware16 = ocxo_capture_hw_ready ? IMXRT_TMR3.CH[3].CNTR : 0;
  const uint32_t epoch_capture_end_raw = ARM_DWT_CYCCNT;

  const uint16_t ch3_now = hardware_low16;

  const uint16_t selected_low16 =
      (uint16_t)(hardware_low16 -
                 (uint16_t)VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED);

  if (g_vclock_clock32.pending_zero) {
    vclock_clock_zero_at_hardware_low16(g_vclock_clock32.pending_zero_ns,
                                        selected_low16);
  }

  const uint32_t observed_counter32 =
      vclock_synthetic_from_hardware_low16(hardware_low16) +
      (uint32_t)VCLOCK_EPOCH_COUNTER32_OFFSET_TICKS;
  const uint32_t selected_counter32 =
      observed_counter32 - VCLOCK_EPOCH_OBSERVED_TICKS_AFTER_SELECTED;

  const uint32_t ocxo1_counter32 = ocxo_capture_hw_ready
      ? project_counter32_from_hw16(g_ocxo1_clock32, ocxo1_hardware16)
      : 0;
  const uint32_t ocxo2_counter32 = ocxo_capture_hw_ready
      ? project_counter32_from_hw16(g_ocxo2_clock32, ocxo2_hardware16)
      : 0;

  g_gpio_irq_count++;
  g_pps_gpio_heartbeat.edge_count++;

  interrupt_epoch_capture_t epoch_cap{};
  epoch_cap.sequence = g_pps_gpio_heartbeat.edge_count;
  epoch_cap.capture_dwt_start_raw = epoch_capture_start_raw;
  epoch_cap.capture_dwt_after_vclock_raw = epoch_capture_after_vclock_raw;
  epoch_cap.capture_dwt_end_raw = epoch_capture_end_raw;
  epoch_cap.capture_window_cycles = epoch_capture_end_raw - epoch_capture_start_raw;
  epoch_cap.vclock_read_offset_cycles =
      epoch_capture_after_vclock_raw - epoch_capture_start_raw;
  epoch_cap.vclock_dwt_at_edge =
      pps_vclock_dwt_from_pps_isr_entry_raw(isr_entry_dwt_raw);
  epoch_cap.vclock_capture_valid =
      epoch_cap.vclock_read_offset_cycles <= EPOCH_CAPTURE_MAX_WINDOW_CYCLES;
  epoch_cap.all_lanes_capture_valid =
      ocxo_capture_hw_ready &&
      epoch_cap.capture_window_cycles <= EPOCH_CAPTURE_MAX_WINDOW_CYCLES;
  // A packet is operationally selectable only after interrupt runtime has
  // finished initialization.  This prevents a boot-time partial capture from
  // being mistaken for a CLOCKS.ZERO epoch packet.
  epoch_cap.valid = g_interrupt_runtime_ready && epoch_cap.vclock_capture_valid;
  epoch_cap.vclock_hardware16_observed = hardware_low16;
  epoch_cap.vclock_hardware16_selected = selected_low16;
  epoch_cap.ocxo1_hardware16 = ocxo1_hardware16;
  epoch_cap.ocxo2_hardware16 = ocxo2_hardware16;
  epoch_cap.vclock_counter32 = selected_counter32;
  epoch_cap.ocxo1_counter32 = ocxo1_counter32;
  epoch_cap.ocxo2_counter32 = ocxo2_counter32;
  epoch_capture_publish(epoch_cap);

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

  // During rebootstrap, PPS selects the sacred VCLOCK edge identity.  The
  // already-running critical TimePop VCLOCK cadence client consumes the next CH2
  // fire fact, back-projects to this selected edge, refreshes the synthetic
  // VCLOCK anchor in ISR context, and publishes the canonical epoch.
  if (g_pps_rebootstrap_pending) {
    g_pps_rebootstrap_pending = false;
    g_pps_rebootstrap_count++;

    g_vclock_epoch_latch.pending = true;
    g_vclock_epoch_latch.pps = pps;
    g_vclock_epoch_latch.counter32_at_edge = selected_counter32;
    g_vclock_epoch_latch.ch3_at_edge = selected_low16;
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
//   snapshot.gnss_ns_at_edge   <- -1 (GNSS labels are CLOCKS-owned)
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
  out.gnss_ns_at_edge   = -1;  // GNSS labels are CLOCKS-owned.

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
    g_vclock_lane.phase_bootstrapped = true;
    g_vclock_lane.tick_mod_1000 = 0;
    // CADENCE_MINDER is armed once during process_interrupt_init() and tends
    // only VCLOCK.  Starting VCLOCK must not arm a second cadence slot.
    return g_cadence_minder_armed || cadence_minder_arm_timepop();
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized) return false;

  lane->active = true;
  lane->phase_bootstrapped = true;
  lane->tick_mod_1000 = 0;

  // OCXO custody is lane-local and independent of TimePop.  The scheduler is
  // normally started during process_interrupt_init(); this start path is kept
  // as a self-healing hook in case the lane was not yet armed.
  if (!lane->scheduler_active) {
    (void)ocxo_lane_start_custody_scheduler(*lane, *clock32);
  }

  // If CLOCKS has already installed the OCXO logical grid, align the private
  // scheduler to it now.  Otherwise the 1 kHz scheduler keeps rollover custody
  // only; second-edge emission begins when interrupt_ocxo_logical_grid_epoch()
  // arrives.
  ocxo_schedule_epoch_grid_if_ready(rt);
  return lane->scheduler_active;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = false;
  rt->stop_count++;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    g_vclock_lane.phase_bootstrapped = false;
    // Do not cancel CADENCE_MINDER here; it is the system-wide VCLOCK rail.
    return true;
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return false;
  lane->active = false;
  // Keep the private OCXO scheduler alive for rollover custody.  Stopping a
  // subscriber suppresses events; it does not shut down the lane's synthetic
  // 32-bit counter maintenance.
  return true;
}

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32) {
  g_ocxo1_grid_epoch_counter32 = ocxo1_epoch_counter32;
  g_ocxo2_grid_epoch_counter32 = ocxo2_epoch_counter32;
  g_ocxo_grid_epoch_valid = true;

  if (g_ocxo1_lane.initialized) {
    (void)ocxo_lane_install_epoch_grid(interrupt_subscriber_kind_t::OCXO1,
                                       g_ocxo1_lane,
                                       g_ocxo1_clock32,
                                       ocxo1_epoch_counter32);
  }

  if (g_ocxo2_lane.initialized) {
    (void)ocxo_lane_install_epoch_grid(interrupt_subscriber_kind_t::OCXO2,
                                       g_ocxo2_lane,
                                       g_ocxo2_clock32,
                                       ocxo2_epoch_counter32);
  }
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};

  // OCXO custody keeps running, but the one-second emission grid belongs to
  // CLOCKS and must be reinstalled after the new logical zero is selected.
  g_ocxo_grid_epoch_valid = false;
  g_ocxo1_lane.second_grid_valid = false;
  g_ocxo2_lane.second_grid_valid = false;
  g_ocxo1_lane.next_edge_counter32 = 0;
  g_ocxo2_lane.next_edge_counter32 = 0;

  // Rebootstrap selects a fresh PPS_VCLOCK edge identity.
  g_pvc_anchor_reset_pending = true;
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

void interrupt_service_deferred_events(void) {
  // Compatibility drain for stale pre-direct-dispatch OCXO mailbox entries.
  // Normal OCXO one-second events now dispatch directly from the lane-local
  // QTimer ISR path and do not depend on this service being called.
  ocxo_drain_deferred_edge(g_rt_ocxo1, g_ocxo1_lane, g_ocxo1_deferred);
  ocxo_drain_deferred_edge(g_rt_ocxo2, g_ocxo2_lane, g_ocxo2_deferred);
}

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*) {
  interrupt_service_deferred_events();
}

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

void process_interrupt_init_hardware(void) {
  if (g_interrupt_hw_ready) return;

  g_ocxo1_lane.name = "OCXO1";
  g_ocxo1_lane.module = &IMXRT_TMR2;
  g_ocxo1_lane.channel = 0;
  g_ocxo1_lane.pcs = 0;
  g_ocxo1_lane.input_pin = OCXO1_PIN;

  g_ocxo2_lane.name = "OCXO2";
  g_ocxo2_lane.module = &IMXRT_TMR3;
  g_ocxo2_lane.channel = 3;
  g_ocxo2_lane.pcs = 3;
  g_ocxo2_lane.input_pin = OCXO2_PIN;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
  *(portConfigRegister(OCXO1_PIN)) = 1;
  *(portConfigRegister(OCXO2_PIN)) = 1;
  IOMUXC_QTIMER2_TIMER0_SELECT_INPUT = 1;
  IOMUXC_QTIMER3_TIMER3_SELECT_INPUT = 1;

  IMXRT_TMR2.CH[0].CTRL = 0; IMXRT_TMR2.CH[0].SCTRL = 0;
  IMXRT_TMR2.CH[0].CSCTRL = 0; IMXRT_TMR2.CH[0].LOAD = 0;
  IMXRT_TMR2.CH[0].CNTR = 0; IMXRT_TMR2.CH[0].COMP1 = 0xFFFF;
  IMXRT_TMR2.CH[0].CMPLD1 = 0xFFFF; IMXRT_TMR2.CH[0].CMPLD2 = 0;
  IMXRT_TMR2.CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo1_lane.pcs);
  qtimer2_ch0_disable_compare();
  IMXRT_TMR2.ENBL |= 0x0001;

  IMXRT_TMR3.CH[3].CTRL = 0; IMXRT_TMR3.CH[3].SCTRL = 0;
  IMXRT_TMR3.CH[3].CSCTRL = 0; IMXRT_TMR3.CH[3].LOAD = 0;
  IMXRT_TMR3.CH[3].CNTR = 0; IMXRT_TMR3.CH[3].COMP1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR3.CH[3].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(g_ocxo2_lane.pcs);
  qtimer3_disable_compare(3);
  IMXRT_TMR3.ENBL |= (uint16_t)(1U << 3);

  g_ocxo1_lane.initialized = true;
  g_ocxo2_lane.initialized = true;

  // QTimer1 synchronized channel start: ENBL=0 first, configure CH0/CH1/CH2
  // fully, then a single ENBL write starts the active VCLOCK-domain channels
  // on the same edge.  CH2 is the only enabled compare channel; VCLOCK cadence
  // is expressed as a normal TimePop client.
  IMXRT_TMR1.ENBL = 0;
  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();
  g_vclock_lane.initialized = true;
  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;
  IMXRT_TMR1.CH[2].CNTR = 0;
  // Only CH0 (passive VCLOCK counter) and CH2 (TimePop scheduler compare)
  // are enabled.  VCLOCK cadence is now a TimePop client; CH3 remains off.
  IMXRT_TMR1.ENBL = 0x05;

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
  g_pps_rebootstrap_pending = false;
  g_pps_rebootstrap_count = 0;

  g_store = snapshot_store_t{};
  g_epoch_capture_store = epoch_capture_store_t{};
  pvc_anchor_ring_reset();
  g_bridge_stats_timepop = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo1 = bridge_anchor_stats_t{};
  g_bridge_stats_ocxo2 = bridge_anchor_stats_t{};
  g_pps_edge_dispatch = nullptr;
  g_pps_entry_latency_handler = nullptr;

  g_vclock_clock32 = vclock_synthetic_clock32_t{};
  g_ocxo1_clock32 = synthetic_clock32_t{};
  g_ocxo2_clock32 = synthetic_clock32_t{};

  // Defensive birth anchors.  These are not logical ZERO operations; they
  // simply make process_interrupt's synthetic coordinate extenders safe before
  // CLOCKS has installed a user/campaign epoch.
  if (g_interrupt_hw_ready) {
    vclock_clock_bootstrap_from_hw16(qtimer1_ch0_counter_now());
    if (g_ocxo1_lane.initialized) {
      synthetic_clock_bootstrap_from_hw16(g_ocxo1_clock32, IMXRT_TMR2.CH[0].CNTR);
      g_ocxo1_lane.logical_count32_at_last_second = g_ocxo1_clock32.current_counter32;
    }
    if (g_ocxo2_lane.initialized) {
      synthetic_clock_bootstrap_from_hw16(g_ocxo2_clock32, IMXRT_TMR3.CH[3].CNTR);
      g_ocxo2_lane.logical_count32_at_last_second = g_ocxo2_clock32.current_counter32;
    }
  }
  g_qtimer1_ch1_sequence = 0;
  g_qtimer1_ch1_target_counter32 = 0;
  g_qtimer1_ch1_next_compare_counter32 = 0;
  g_qtimer1_ch1_arm_count = 0;
  g_qtimer1_ch1_fire_count = 0;
  g_qtimer1_ch1_hop_count = 0;
  g_qtimer1_ch1_active = false;
  g_qtimer1_ch1_handler = nullptr;
  g_qtimer1_ch2_last_target_counter32 = 0;
  g_qtimer1_ch2_arm_count = 0;
  g_qtimer2_ch0_count = 0;
  g_qtimer2_last_ch0_late_ticks = 0;
  g_qtimer2_last_ch0_dwt_raw = 0;
  g_qtimer2_last_ch0_event_dwt = 0;
  g_qtimer2_last_ch0_dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;
  g_qtimer3_ch3_count = 0;
  g_qtimer3_last_ch3_late_ticks = 0;
  g_qtimer3_last_ch3_dwt_raw = 0;
  g_qtimer3_last_ch3_event_dwt = 0;
  g_qtimer3_last_ch3_dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;
  g_ocxo_deferred_asap_arm_count = 0;
  g_ocxo_deferred_asap_overwrite_count = 0;
  g_ocxo_deferred_sample_count = 0;
  g_ocxo_deferred_edge_count = 0;
  g_cadence_minder_armed = false;
  g_cadence_minder_arm_count = 0;
  g_cadence_minder_arm_failures = 0;
  g_cadence_minder_fire_count = 0;
  g_cadence_minder_vclock_updates = 0;
  g_cadence_minder_ocxo1_updates = 0;
  g_cadence_minder_ocxo2_updates = 0;
  g_cadence_minder_last_vclock_hw16 = 0;
  g_cadence_minder_last_ocxo1_hw16 = 0;
  g_cadence_minder_last_ocxo2_hw16 = 0;
  g_cadence_minder_last_vclock_counter32 = 0;
  g_cadence_minder_last_ocxo1_counter32 = 0;
  g_cadence_minder_last_ocxo2_counter32 = 0;
  g_ocxo1_edge_fire_count = 0;
  g_ocxo2_edge_fire_count = 0;
  g_ocxo1_lane.active = false;
  g_ocxo1_lane.scheduler_active = false;
  g_ocxo1_lane.second_grid_valid = false;
  g_ocxo1_lane.phase_bootstrapped = false;
  g_ocxo1_lane.compare_target = 0;
  g_ocxo1_lane.next_cadence_counter32 = 0;
  g_ocxo1_lane.grid_epoch_counter32 = 0;
  g_ocxo1_lane.next_edge_counter32 = 0;
  g_ocxo1_lane.tick_mod_1000 = 0;
  g_ocxo1_lane.irq_count = 0;
  g_ocxo1_lane.miss_count = 0;
  g_ocxo1_lane.bootstrap_count = 0;
  g_ocxo1_lane.cadence_hits_total = 0;
  g_ocxo1_lane.edge_hits_total = 0;
  g_ocxo1_lane.second_event_count = 0;
  g_ocxo1_lane.deferred_edge_count = 0;
  g_ocxo1_lane.deferred_overwrite_count = 0;
  g_ocxo1_lane.late_max_ticks = 0;
  g_ocxo1_lane.last_late_ticks = 0;
  g_ocxo1_lane.skipped_cadence_count = 0;
  g_ocxo1_lane.missed_second_count = 0;
  g_ocxo1_lane.compare_rearm_count = 0;
  g_ocxo1_lane.compare_rearm_failure_count = 0;
  g_ocxo1_lane.last_fired_counter32 = 0;
  g_ocxo1_lane.last_fired_low16 = 0;

  g_ocxo2_lane.active = false;
  g_ocxo2_lane.scheduler_active = false;
  g_ocxo2_lane.second_grid_valid = false;
  g_ocxo2_lane.phase_bootstrapped = false;
  g_ocxo2_lane.compare_target = 0;
  g_ocxo2_lane.next_cadence_counter32 = 0;
  g_ocxo2_lane.grid_epoch_counter32 = 0;
  g_ocxo2_lane.next_edge_counter32 = 0;
  g_ocxo2_lane.tick_mod_1000 = 0;
  g_ocxo2_lane.irq_count = 0;
  g_ocxo2_lane.miss_count = 0;
  g_ocxo2_lane.bootstrap_count = 0;
  g_ocxo2_lane.cadence_hits_total = 0;
  g_ocxo2_lane.edge_hits_total = 0;
  g_ocxo2_lane.second_event_count = 0;
  g_ocxo2_lane.deferred_edge_count = 0;
  g_ocxo2_lane.deferred_overwrite_count = 0;
  g_ocxo2_lane.late_max_ticks = 0;
  g_ocxo2_lane.last_late_ticks = 0;
  g_ocxo2_lane.skipped_cadence_count = 0;
  g_ocxo2_lane.missed_second_count = 0;
  g_ocxo2_lane.compare_rearm_count = 0;
  g_ocxo2_lane.compare_rearm_failure_count = 0;
  g_ocxo2_lane.last_fired_counter32 = 0;
  g_ocxo2_lane.last_fired_low16 = 0;

  g_ocxo_grid_epoch_valid = false;
  g_ocxo1_grid_epoch_counter32 = 0;
  g_ocxo2_grid_epoch_counter32 = 0;
  g_ocxo1_grid_epoch_start_count = 0;
  g_ocxo2_grid_epoch_start_count = 0;
  g_ocxo1_grid_epoch_wait_count = 0;
  g_ocxo2_grid_epoch_wait_count = 0;
  g_ocxo1_deferred = ocxo_deferred_work_t{};
  g_ocxo2_deferred = ocxo_deferred_work_t{};

  g_interrupt_runtime_ready = true;
  (void)cadence_minder_arm_timepop();
}

void process_interrupt_enable_irqs(void) {
  if (g_interrupt_irqs_enabled) return;

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  // VCLOCK/TimePop QTimer1 is the sovereign timing rail.  Keep the entire
  // shared QTimer1 vector at highest priority; same-vector logical ordering
  // is handled inside TimePop/process_interrupt.
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 0);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);

  // Start the OCXO private 1 kHz custody schedulers immediately before their
  // NVIC lines go live.  They do not emit second events until CLOCKS installs
  // the OCXO logical grid, but they do maintain 16-bit rollover custody.
  if (g_interrupt_hw_ready) {
    if (g_ocxo1_lane.initialized) {
      if (g_ocxo_grid_epoch_valid) {
        (void)ocxo_lane_install_epoch_grid(interrupt_subscriber_kind_t::OCXO1,
                                           g_ocxo1_lane,
                                           g_ocxo1_clock32,
                                           g_ocxo1_grid_epoch_counter32);
      } else {
        (void)ocxo_lane_start_custody_scheduler(g_ocxo1_lane, g_ocxo1_clock32);
      }
    }
    if (g_ocxo2_lane.initialized) {
      if (g_ocxo_grid_epoch_valid) {
        (void)ocxo_lane_install_epoch_grid(interrupt_subscriber_kind_t::OCXO2,
                                           g_ocxo2_lane,
                                           g_ocxo2_clock32,
                                           g_ocxo2_grid_epoch_counter32);
      } else {
        (void)ocxo_lane_start_custody_scheduler(g_ocxo2_lane, g_ocxo2_clock32);
      }
    }
  }

  NVIC_ENABLE_IRQ(IRQ_QTIMER2);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  // PPS GPIO is now a witness/selector, not the highest-priority timing
  // interpolation rail.  Keep it below QTimer1 so VCLOCK/TimePop event facts
  // are not delayed by PPS witness work.
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
  p.add("vclock_timepop_cadence_minder", true);
  p.add("cadence_minder_tends_vclock_only", true);
  p.add("ocxo_timepop_arm_retired", true);
  p.add("ocxo_timepop_post_isr_retired", true);
  p.add("ocxo_edges_authored_by_qtimer_isr", true);
  p.add("ocxo_edges_armed_by_timepop", false);
  p.add("ocxo_scheduler_model", "LANE_LOCAL_1KHZ_QTIMER_COMPARE");
  p.add("ocxo_cadence_interval_counts", (uint32_t)OCXO_CADENCE_INTERVAL_COUNTS);
  p.add("ocxo_cadence_events_per_second", (uint32_t)OCXO_CADENCE_EVENTS_PER_SECOND);
  p.add("ocxo_edge_interval_counts", (uint32_t)OCXO_EDGE_INTERVAL_COUNTS);
  p.add("ocxo_edge_tick_to_gnss_ns_estimate", 100U);
  p.add("ocxo_deferred_dispatch_uses_timepop", false);
  p.add("ocxo_deferred_dispatch_uses_prespin", false);
  p.add("ocxo_direct_dispatch_from_qtimer_isr", true);
  p.add("ocxo_prespin_service_retired", true);
  p.add("ocxo_quad_timer_dwt_quantization_uncompensated", true);
  p.add("ocxo_grid_epoch_valid", g_ocxo_grid_epoch_valid);
  p.add("ocxo1_grid_epoch_counter32", g_ocxo1_grid_epoch_counter32);
  p.add("ocxo2_grid_epoch_counter32", g_ocxo2_grid_epoch_counter32);
  p.add("ocxo1_grid_epoch_start_count", g_ocxo1_grid_epoch_start_count);
  p.add("ocxo2_grid_epoch_start_count", g_ocxo2_grid_epoch_start_count);
  p.add("ocxo1_grid_epoch_wait_count", g_ocxo1_grid_epoch_wait_count);
  p.add("ocxo2_grid_epoch_wait_count", g_ocxo2_grid_epoch_wait_count);
  p.add("cadence_minder_period_ns", (uint64_t)CADENCE_MINDER_PERIOD_NS);
  p.add("cadence_minder_armed", g_cadence_minder_armed);
  p.add("cadence_minder_arm_count", g_cadence_minder_arm_count);
  p.add("cadence_minder_arm_failures", g_cadence_minder_arm_failures);
  p.add("cadence_minder_fire_count", g_cadence_minder_fire_count);
  p.add("cadence_minder_vclock_updates", g_cadence_minder_vclock_updates);
  p.add("cadence_minder_ocxo1_updates", g_cadence_minder_ocxo1_updates);
  p.add("cadence_minder_ocxo2_updates", g_cadence_minder_ocxo2_updates);
  p.add("cadence_minder_last_vclock_hw16", (uint32_t)g_cadence_minder_last_vclock_hw16);
  p.add("cadence_minder_last_ocxo1_hw16", (uint32_t)g_cadence_minder_last_ocxo1_hw16);
  p.add("cadence_minder_last_ocxo2_hw16", (uint32_t)g_cadence_minder_last_ocxo2_hw16);
  p.add("cadence_minder_last_vclock_counter32", g_cadence_minder_last_vclock_counter32);
  p.add("cadence_minder_last_ocxo1_counter32", g_cadence_minder_last_ocxo1_counter32);
  p.add("cadence_minder_last_ocxo2_counter32", g_cadence_minder_last_ocxo2_counter32);
  p.add("vclock_subscribed", g_rt_vclock ? g_rt_vclock->subscribed : false);
  p.add("vclock_active", g_rt_vclock ? g_rt_vclock->active : false);
  p.add("vclock_event_count", g_rt_vclock ? g_rt_vclock->event_count : 0U);
  p.add("vclock_dispatch_count", g_rt_vclock ? g_rt_vclock->dispatch_count : 0U);
  p.add("ocxo1_subscribed", g_rt_ocxo1 ? g_rt_ocxo1->subscribed : false);
  p.add("ocxo1_active", g_rt_ocxo1 ? g_rt_ocxo1->active : false);
  p.add("ocxo1_event_count", g_rt_ocxo1 ? g_rt_ocxo1->event_count : 0U);
  p.add("ocxo1_dispatch_count", g_rt_ocxo1 ? g_rt_ocxo1->dispatch_count : 0U);

  p.add("ocxo2_subscribed", g_rt_ocxo2 ? g_rt_ocxo2->subscribed : false);
  p.add("ocxo2_active", g_rt_ocxo2 ? g_rt_ocxo2->active : false);
  p.add("ocxo2_event_count", g_rt_ocxo2 ? g_rt_ocxo2->event_count : 0U);
  p.add("ocxo2_dispatch_count", g_rt_ocxo2 ? g_rt_ocxo2->dispatch_count : 0U);
  p.add("ocxo1_scheduler_active", g_ocxo1_lane.scheduler_active);
  p.add("ocxo1_second_grid_valid", g_ocxo1_lane.second_grid_valid);
  p.add("ocxo1_next_cadence_counter32", g_ocxo1_lane.next_cadence_counter32);
  p.add("ocxo1_next_edge_counter32", g_ocxo1_lane.next_edge_counter32);
  p.add("ocxo1_grid_epoch_counter32_lane", g_ocxo1_lane.grid_epoch_counter32);
  p.add("ocxo1_edge_fire_count", g_ocxo1_edge_fire_count);
  p.add("ocxo1_second_event_count", g_ocxo1_lane.second_event_count);
  p.add("ocxo1_deferred_edge_count", g_ocxo1_lane.deferred_edge_count);
  p.add("ocxo1_deferred_overwrite_count", g_ocxo1_lane.deferred_overwrite_count);
  p.add("ocxo1_late_max_ticks", g_ocxo1_lane.late_max_ticks);
  p.add("ocxo1_last_late_ticks", g_ocxo1_lane.last_late_ticks);
  p.add("ocxo1_skipped_cadence_count", g_ocxo1_lane.skipped_cadence_count);
  p.add("ocxo1_missed_second_count", g_ocxo1_lane.missed_second_count);
  p.add("ocxo1_compare_rearm_count", g_ocxo1_lane.compare_rearm_count);
  p.add("ocxo1_compare_rearm_failure_count", g_ocxo1_lane.compare_rearm_failure_count);
  p.add("ocxo1_last_fired_counter32", g_ocxo1_lane.last_fired_counter32);
  p.add("ocxo1_last_fired_low16", (uint32_t)g_ocxo1_lane.last_fired_low16);

  p.add("ocxo2_scheduler_active", g_ocxo2_lane.scheduler_active);
  p.add("ocxo2_second_grid_valid", g_ocxo2_lane.second_grid_valid);
  p.add("ocxo2_next_cadence_counter32", g_ocxo2_lane.next_cadence_counter32);
  p.add("ocxo2_next_edge_counter32", g_ocxo2_lane.next_edge_counter32);
  p.add("ocxo2_grid_epoch_counter32_lane", g_ocxo2_lane.grid_epoch_counter32);
  p.add("ocxo2_edge_fire_count", g_ocxo2_edge_fire_count);
  p.add("ocxo2_second_event_count", g_ocxo2_lane.second_event_count);
  p.add("ocxo2_deferred_edge_count", g_ocxo2_lane.deferred_edge_count);
  p.add("ocxo2_deferred_overwrite_count", g_ocxo2_lane.deferred_overwrite_count);
  p.add("ocxo2_late_max_ticks", g_ocxo2_lane.late_max_ticks);
  p.add("ocxo2_last_late_ticks", g_ocxo2_lane.last_late_ticks);
  p.add("ocxo2_skipped_cadence_count", g_ocxo2_lane.skipped_cadence_count);
  p.add("ocxo2_missed_second_count", g_ocxo2_lane.missed_second_count);
  p.add("ocxo2_compare_rearm_count", g_ocxo2_lane.compare_rearm_count);
  p.add("ocxo2_compare_rearm_failure_count", g_ocxo2_lane.compare_rearm_failure_count);
  p.add("ocxo2_last_fired_counter32", g_ocxo2_lane.last_fired_counter32);
  p.add("ocxo2_last_fired_low16", (uint32_t)g_ocxo2_lane.last_fired_low16);

  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count",   g_pps_rebootstrap_count);
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);
  p.add("vclock_epoch_latch_counter32", g_vclock_epoch_latch.counter32_at_edge);
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

  p.add("pps_vclock_phase_cycles", pps_vclock_phase_cycles_from_edges(pps, pvc));

  interrupt_epoch_capture_t epoch_cap{};
  const bool epoch_cap_ok = interrupt_last_epoch_capture(&epoch_cap);
  p.add("epoch_capture_available", epoch_cap_ok);
  p.add("epoch_capture_valid", epoch_cap.valid);
  p.add("epoch_capture_sequence", epoch_cap.sequence);
  p.add("epoch_capture_window_cycles", epoch_cap.capture_window_cycles);
  p.add("epoch_capture_max_window_cycles", EPOCH_CAPTURE_MAX_WINDOW_CYCLES);
  p.add("epoch_capture_vclock_read_offset_cycles", epoch_cap.vclock_read_offset_cycles);
  p.add("epoch_capture_vclock_dwt_at_edge", epoch_cap.vclock_dwt_at_edge);
  p.add("epoch_capture_vclock_valid", epoch_cap.vclock_capture_valid);
  p.add("epoch_capture_all_lanes_valid", epoch_cap.all_lanes_capture_valid);
  p.add("epoch_capture_vclock_hardware16_observed", (uint32_t)epoch_cap.vclock_hardware16_observed);
  p.add("epoch_capture_vclock_hardware16_selected", (uint32_t)epoch_cap.vclock_hardware16_selected);
  p.add("epoch_capture_ocxo1_hardware16", (uint32_t)epoch_cap.ocxo1_hardware16);
  p.add("epoch_capture_ocxo2_hardware16", (uint32_t)epoch_cap.ocxo2_hardware16);
  p.add("epoch_capture_vclock_counter32", epoch_cap.vclock_counter32);
  p.add("epoch_capture_ocxo1_counter32", epoch_cap.ocxo1_counter32);
  p.add("epoch_capture_ocxo2_counter32", epoch_cap.ocxo2_counter32);

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
  p.add("ocxo1_edge_hits_total", g_ocxo1_lane.edge_hits_total);
  p.add("ocxo1_compare_target",     (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_next_edge_counter32", g_ocxo1_lane.next_edge_counter32);
  p.add("ocxo1_tick_mod_1000",      g_ocxo1_lane.tick_mod_1000);
  p.add("ocxo1_logical_count32",    g_ocxo1_lane.logical_count32_at_last_second);

  p.add("ocxo2_irq_count",          g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count",         g_ocxo2_lane.miss_count);
  p.add("ocxo2_bootstrap_count",    g_ocxo2_lane.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_ocxo2_lane.cadence_hits_total);
  p.add("ocxo2_edge_hits_total", g_ocxo2_lane.edge_hits_total);
  p.add("ocxo2_compare_target",     (uint32_t)g_ocxo2_lane.compare_target);
  p.add("ocxo2_next_edge_counter32", g_ocxo2_lane.next_edge_counter32);
  p.add("ocxo2_tick_mod_1000",      g_ocxo2_lane.tick_mod_1000);
  p.add("ocxo2_logical_count32",    g_ocxo2_lane.logical_count32_at_last_second);

  // Static CPS compatibility — process_interrupt now mirrors the CLOCKS
  // PPS/GPIO-derived one-second DWT slope.  Dynamic Gamma prediction has been
  // retired.
  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  p.add("dynamic_cps_owner", "CLOCKS_STATIC_PPS");
  p.add("dynamic_cps",                         dynamic_cps);
  p.add("dynamic_cps_valid",                   dynamic_cps != 0);
  p.add("dynamic_cps_pps_sequence",            g_pps_gpio_heartbeat.edge_count);
  p.add("dynamic_cps_last_pvc_dwt_at_edge",    g_pps_gpio_heartbeat.last_dwt);
  p.add("dynamic_cps_last_reseed_value",       dynamic_cps);
  p.add("dynamic_cps_last_reseed_was_computed", dynamic_cps != 0);
  p.add("dynamic_cps_net_adjustment_cycles",   0);
  p.add("dynamic_cps_refine_ticks_this_second", 0);
  p.add("dynamic_cps_adjustments_this_second", 0);
  p.add("dynamic_cps_total_refine_ticks",      0);
  p.add("dynamic_cps_total_adjustments",       0);

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
  p.add("vclock_clock32_minder_update_count", g_vclock_clock32.minder_update_count);
  p.add("vclock_clock32_pending_zero_count", g_vclock_clock32.pending_zero_count);

  p.add("ocxo1_clock32_zeroed", g_ocxo1_clock32.zeroed);
  p.add("ocxo1_clock32_zero_ns", g_ocxo1_clock32.zero_ns);
  p.add("ocxo1_clock32_zero_counter32", g_ocxo1_clock32.zero_counter32);
  p.add("ocxo1_clock32_current", g_ocxo1_clock32.current_counter32);
  p.add("ocxo1_clock32_pending_zero", g_ocxo1_clock32.pending_zero);
  p.add("ocxo1_clock32_zero_count", g_ocxo1_clock32.zero_count);
  p.add("ocxo1_clock32_minder_update_count", g_ocxo1_clock32.minder_update_count);

  p.add("ocxo2_clock32_zeroed", g_ocxo2_clock32.zeroed);
  p.add("ocxo2_clock32_zero_ns", g_ocxo2_clock32.zero_ns);
  p.add("ocxo2_clock32_zero_counter32", g_ocxo2_clock32.zero_counter32);
  p.add("ocxo2_clock32_current", g_ocxo2_clock32.current_counter32);
  p.add("ocxo2_clock32_pending_zero", g_ocxo2_clock32.pending_zero);
  p.add("ocxo2_clock32_zero_count", g_ocxo2_clock32.zero_count);
  p.add("ocxo2_clock32_minder_update_count", g_ocxo2_clock32.minder_update_count);

  p.add("qtimer1_ch1_active", g_qtimer1_ch1_active);
  p.add("qtimer1_ch1_sequence", g_qtimer1_ch1_sequence);
  p.add("qtimer1_ch1_target_counter32", g_qtimer1_ch1_target_counter32);
  p.add("qtimer1_ch1_next_compare_counter32", g_qtimer1_ch1_next_compare_counter32);
  p.add("qtimer1_ch1_arm_count", g_qtimer1_ch1_arm_count);
  p.add("qtimer1_ch1_fire_count", g_qtimer1_ch1_fire_count);
  p.add("qtimer1_ch1_hop_count", g_qtimer1_ch1_hop_count);
  p.add("qtimer1_ch2_last_target_counter32", g_qtimer1_ch2_last_target_counter32);
  p.add("qtimer1_ch2_arm_count", g_qtimer1_ch2_arm_count);

  p.add("qtimer2_ch0_count", g_qtimer2_ch0_count);
  p.add("qtimer2_ch0_counter", (uint32_t)IMXRT_TMR2.CH[0].CNTR);
  p.add("qtimer2_ch0_comp1", (uint32_t)IMXRT_TMR2.CH[0].COMP1);
  const uint16_t qtimer2_ch0_csctrl = IMXRT_TMR2.CH[0].CSCTRL;
  const bool qtimer2_ch0_compare_enabled =
      (qtimer2_ch0_csctrl & TMR_CSCTRL_TCF1EN) != 0;
  const bool qtimer2_ch0_compare_flag =
      (qtimer2_ch0_csctrl & TMR_CSCTRL_TCF1) != 0;
  p.add("qtimer2_ch0_csctrl", (uint32_t)qtimer2_ch0_csctrl);
  p.add("qtimer2_ch0_compare_enabled", qtimer2_ch0_compare_enabled);
  p.add("qtimer2_ch0_compare_flag", qtimer2_ch0_compare_flag);
  p.add("qtimer2_ch0_compare_live",
        qtimer2_ch0_compare_enabled && qtimer2_ch0_compare_flag);
  p.add("qtimer2_last_ch0_late_ticks", g_qtimer2_last_ch0_late_ticks);
  p.add("qtimer2_last_ch0_dwt_raw", g_qtimer2_last_ch0_dwt_raw);
  p.add("qtimer2_last_ch0_event_dwt", g_qtimer2_last_ch0_event_dwt);
  p.add("qtimer2_last_ch0_dwt_coordinate_source", g_qtimer2_last_ch0_dwt_coordinate_source);

  p.add("qtimer3_ch3_count", g_qtimer3_ch3_count);
  p.add("qtimer3_ch3_counter", (uint32_t)IMXRT_TMR3.CH[3].CNTR);
  p.add("qtimer3_ch3_comp1", (uint32_t)IMXRT_TMR3.CH[3].COMP1);
  const uint16_t qtimer3_ch3_csctrl = IMXRT_TMR3.CH[3].CSCTRL;
  const bool qtimer3_ch3_compare_enabled =
      (qtimer3_ch3_csctrl & TMR_CSCTRL_TCF1EN) != 0;
  const bool qtimer3_ch3_compare_flag =
      (qtimer3_ch3_csctrl & TMR_CSCTRL_TCF1) != 0;
  p.add("qtimer3_ch3_csctrl", (uint32_t)qtimer3_ch3_csctrl);
  p.add("qtimer3_ch3_compare_enabled", qtimer3_ch3_compare_enabled);
  p.add("qtimer3_ch3_compare_flag", qtimer3_ch3_compare_flag);
  p.add("qtimer3_ch3_compare_live",
        qtimer3_ch3_compare_enabled && qtimer3_ch3_compare_flag);
  p.add("qtimer3_last_ch3_late_ticks", g_qtimer3_last_ch3_late_ticks);
  p.add("qtimer3_last_ch3_dwt_raw", g_qtimer3_last_ch3_dwt_raw);
  p.add("qtimer3_last_ch3_event_dwt", g_qtimer3_last_ch3_event_dwt);
  p.add("qtimer3_last_ch3_dwt_coordinate_source", g_qtimer3_last_ch3_dwt_coordinate_source);

  p.add("ocxo_deferred_asap_arm_count", g_ocxo_deferred_asap_arm_count);
  p.add("ocxo_deferred_asap_overwrite_count", g_ocxo_deferred_asap_overwrite_count);
  p.add("ocxo_deferred_sample_count", g_ocxo_deferred_sample_count);
  p.add("ocxo_deferred_edge_count", g_ocxo_deferred_edge_count);
  p.add("ocxo1_deferred_sample_pending", false);
  p.add("ocxo1_deferred_edge_pending", g_ocxo1_deferred.edge_pending);
  p.add("ocxo2_deferred_sample_pending", false);
  p.add("ocxo2_deferred_edge_pending", g_ocxo2_deferred.edge_pending);

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
    case interrupt_provider_kind_t::QTIMER2:  return "QTIMER2";
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
    case interrupt_lane_t::QTIMER2_CH0_COMP: return "QTIMER2_CH0_COMP";
    case interrupt_lane_t::QTIMER3_CH3_COMP: return "QTIMER3_CH3_COMP";
    case interrupt_lane_t::GPIO_EDGE:        return "GPIO_EDGE";
    default:                                 return "NONE";
  }
}
