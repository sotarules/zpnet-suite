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
// TimePop is the principled exception to the "no DWT conversion in
// PPS_VCLOCK" rule: it projects CH2 fire facts onto the PPS_VCLOCK timeline
// for scheduling diagnostics. OCXO one-second subscribers receive authored
// OCXO edge facts even when that projection is unavailable; CLOCKS/Alpha owns
// OCXO measured-GNSS interval construction from consecutive OCXO edge DWTs.
//
// Lanes:
//   VCLOCK : TimePop recurring client on QTimer1 CH2 (10 MHz VCLOCK domain)
//   OCXO1  : QTimer2 CH0, PCS=0
//   OCXO2  : QTimer3 CH3, PCS=3
//   TimePop: QTimer1 CH2, varied compare intervals (foreground scheduler)
//
// The PPS GPIO ISR captures DWT as its first instruction, then reads the
// QTimer1 CH0 low-word counter.  process_interrupt maps that low-word
// hardware observation into the private synthetic 32-bit VCLOCK identity.
// When a rebootstrap is pending, it
// records the selected VCLOCK edge.  The recurring TimePop VCLOCK cadence client is armed as a critical recurring
// ISR slot.  It consumes the next CH2 fire fact, back-projects to the selected
// edge, refreshes the synthetic VCLOCK low-word anchor before TimePop schedules
// the next compare, and thereafter emits the one-second VCLOCK events on the
// TimePop rail.
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

// Single TimePop custody agent for all 16-bit hardware counters and all
// one-second lane events.
//
// CADENCE_MINDER replaces the former separate VCLOCK cadence slot.  For OCXO
// lanes it performs low-word custody only; OCXO one-second edge facts are
// authored by their own QTimer compare ISRs after TimePop opens the compare
// gate near the predicted edge.
static constexpr uint64_t CADENCE_MINDER_PERIOD_NS = 1000000ULL;
static constexpr const char* CADENCE_MINDER_NAME = "CADENCE_MINDER";

// OCXO edge capture is lane-local.  TimePop only opens the compare gate
// shortly before the expected edge; the QTimer2/QTimer3 ISR remains the sole
// author of OCXO edge DWT/counter facts.
static constexpr uint64_t OCXO_EDGE_ARM_LEAD_NS  = 1000000ULL;
static constexpr uint32_t OCXO_EDGE_ARM_LEAD_TICKS =
    (uint32_t)(OCXO_EDGE_ARM_LEAD_NS / 100ULL);

// OCXO edge target is OCXO-domain: one OCXO second is 10,000,000 ticks.
// TimePop arming is GNSS-domain: convert the estimated ticks-until-arm to
// GNSS nanoseconds using the nominal 100 ns/tick mapping. Tau correction is
// deliberately not used for this initial scheduler because the OCXOs are close
// enough that a 1 ms lead leaves ample margin.
static constexpr uint32_t OCXO_EDGE_INTERVAL_COUNTS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static_assert(OCXO_EDGE_INTERVAL_COUNTS == 10000000U,
              "OCXO edge interval must be one 10 MHz second");

static inline uint64_t ocxo_ticks_to_estimated_gnss_ns(uint32_t ticks) {
  return (uint64_t)ticks * 100ULL;
}

static constexpr const char* OCXO1_EDGE_ARM_NAME = "OCXO1_EDGE_ARM";
static constexpr const char* OCXO2_EDGE_ARM_NAME = "OCXO2_EDGE_ARM";
static constexpr const char* OCXO1_EDGE_ARM_REARM_NAME = "OCXO1_EDGE_ARM_REARM";
static constexpr const char* OCXO2_EDGE_ARM_REARM_NAME = "OCXO2_EDGE_ARM_REARM";

// Live-gate staging: TimePop opens the OCXO compare gate only after the
// zero-clear preflight proves the stale compare flag is clear.  If the
// preflight fails, the gate remains closed and the dry scheduler re-arms for
// the next OCXO edge.
static constexpr bool OCXO_EDGE_ARM_DRY_RUN = false;

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
  bool     active = false;
  bool     phase_bootstrapped = false;
  uint16_t compare_target = 0;

  // Full 32-bit OCXO edge identity currently targeted by compare_target.
  // compare_target is only the hardware low word used when the QTimer gate is
  // opened; next_edge_counter32 is the lane-local one-second edge identity
  // used to project the next TimePop arming window without drifting early.
  uint32_t next_edge_counter32 = 0;

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

static volatile uint32_t g_ocxo1_edge_arm_schedule_count = 0;
static volatile uint32_t g_ocxo1_edge_arm_schedule_failures = 0;
static volatile uint32_t g_ocxo1_edge_arm_callback_count = 0;
static volatile uint32_t g_ocxo1_edge_arm_enable_count = 0;
static volatile uint32_t g_ocxo1_edge_fire_count = 0;
static volatile uint32_t g_ocxo1_edge_arm_last_target_counter32 = 0;
static volatile uint32_t g_ocxo1_edge_arm_last_now_counter32 = 0;
static volatile uint32_t g_ocxo1_edge_arm_last_ticks_to_target = 0;
static volatile uint64_t g_ocxo1_edge_arm_last_delay_ns = 0;
static volatile uint32_t g_ocxo2_edge_arm_schedule_count = 0;
static volatile uint32_t g_ocxo2_edge_arm_schedule_failures = 0;
static volatile uint32_t g_ocxo2_edge_arm_callback_count = 0;
static volatile uint32_t g_ocxo2_edge_arm_enable_count = 0;
static volatile uint32_t g_ocxo2_edge_fire_count = 0;
static volatile uint32_t g_ocxo2_edge_arm_last_target_counter32 = 0;
static volatile uint32_t g_ocxo2_edge_arm_last_now_counter32 = 0;
static volatile uint32_t g_ocxo2_edge_arm_last_ticks_to_target = 0;
static volatile uint64_t g_ocxo2_edge_arm_last_delay_ns = 0;
static volatile uint32_t g_ocxo1_edge_arm_rearm_asap_count = 0;
static volatile uint32_t g_ocxo2_edge_arm_rearm_asap_count = 0;

// Patch 1 — advance-iteration and arm-delay diagnostics for
// ocxo_schedule_edge_arm_for_target.  The while loop that walks
// next_edge_counter32 forward by OCXO_EDGE_INTERVAL_COUNTS is bounded in theory
// (~430 iterations covers a full 32-bit wrap), but a synthetic-clock race can
// poison now_counter32 and either drive the loop far higher than it should be
// or compute an arm_delay_ns that lands the new TimePop slot already past-due.
// Both conditions are precursors to the freeze pattern we are chasing.
//
// Under the Patch 8 priority architecture (QTimer1=32, OCXO=16, PPS=0) the
// underlying race is structurally prevented — BASEPRI-16 critical sections
// inside timepop_arm and timepop_cancel_by_name now lawfully exclude
// CADENCE_MINDER.  These diagnostics remain in place as sanity checks: under
// Patch 8, advance_iter_count_max should stay at 1 (the legitimate single
// catch-up case) and short_delay_count should stay at zero forever.  Any
// growth in these counters after Patch 8 would indicate either the priority
// change did not take effect or a different race we have not yet identified.
static volatile uint32_t g_ocxo1_edge_arm_advance_iter_count_total = 0;
static volatile uint32_t g_ocxo1_edge_arm_advance_iter_count_max = 0;
static volatile uint32_t g_ocxo1_edge_arm_advance_iter_count_last = 0;
static volatile uint32_t g_ocxo1_edge_arm_advance_iter_break_count = 0;
static volatile uint32_t g_ocxo1_edge_arm_last_arm_delay_ns_lo = 0;
static volatile uint32_t g_ocxo1_edge_arm_short_delay_count = 0;

static volatile uint32_t g_ocxo2_edge_arm_advance_iter_count_total = 0;
static volatile uint32_t g_ocxo2_edge_arm_advance_iter_count_max = 0;
static volatile uint32_t g_ocxo2_edge_arm_advance_iter_count_last = 0;
static volatile uint32_t g_ocxo2_edge_arm_advance_iter_break_count = 0;
static volatile uint32_t g_ocxo2_edge_arm_last_arm_delay_ns_lo = 0;
static volatile uint32_t g_ocxo2_edge_arm_short_delay_count = 0;

// Patch 2 — synthetic clock regression detector.  Synthetic ticks must never
// run backward.  A regression here is almost always evidence of an unguarded
// read-modify-write race against another execution context tending the same
// clock.  These counters are written from inside synthetic_clock_tend_from_hw16
// itself, so they witness every tending pass regardless of caller.
//
// Under the Patch 8 priority architecture these counters should stay at zero
// forever.  Retained as sanity check: a non-zero count after Patch 8 means
// either the priority change did not take effect or there is a different race
// we have not yet identified.
static volatile uint32_t g_ocxo1_clock32_regression_count = 0;
static volatile uint32_t g_ocxo1_clock32_last_regression_delta = 0;
static volatile uint32_t g_ocxo1_clock32_max_regression_delta = 0;
static volatile uint32_t g_ocxo2_clock32_regression_count = 0;
static volatile uint32_t g_ocxo2_clock32_last_regression_delta = 0;
static volatile uint32_t g_ocxo2_clock32_max_regression_delta = 0;
static volatile uint32_t g_vclock_clock32_regression_count = 0;
static volatile uint32_t g_vclock_clock32_last_regression_delta = 0;
static volatile uint32_t g_vclock_clock32_max_regression_delta = 0;

// ============================================================================
// Patch 9 — aliveness probes (freeze diagnosis)
// ============================================================================
//
// Three independent aliveness probes that emit one debug line per second from
// three distinct execution contexts.  Each probe maintains its own millis()
// gate so emissions stay at ~1 Hz regardless of the underlying tick rate.
//
// When the freeze hits:
//   • If "pps_alive" stops, the PPS GPIO ISR itself is dead (hard fault or
//     deeper hardware issue).
//   • If "pps_alive" continues but "cadence_alive" stops, CADENCE_MINDER
//     specifically has been starved — its slot has been mis-scheduled, its
//     IRQ is being blocked, or its callback has hung.
//   • If both ISR probes continue but the foreground loop-counter doesn't
//     advance in the next loop emission, foreground is starved while ISRs
//     remain alive.
//
// These probes use debug_log directly to Serial, bypassing TimePop and the
// transport queue.  They are safe from any context.

static volatile uint32_t g_patch9_pps_emit_count = 0;
static volatile uint32_t g_patch9_pps_last_emit_ms = 0;
static volatile uint32_t g_patch9_cadence_emit_count = 0;
static volatile uint32_t g_patch9_cadence_last_emit_ms = 0;

// ============================================================================
// Patch 10 — CH2 forensic capture
// ============================================================================
//
// When CADENCE_MINDER dies, the freeze pattern observed in the field is
// "happily doing nothing" — PPS GPIO ISR continues firing, foreground loop
// continues spinning, but QTimer1 CH2 IRQ never fires again.  The mechanism
// that produces this state is one of:
//
//   (a) CH2 COMP1 was programmed to a deadline that the counter has already
//       passed; CH2 won't fire until the counter wraps (~6.5 ms wrap for
//       16-bit at 10 MHz, but the IRQ won't dispatch until the IRQ
//       infrastructure thinks it can — see below).
//   (b) CSCTRL's TCF1EN bit got cleared, disabling the compare interrupt
//       enable.  TCF1 may still set on compare match but no IRQ fires.
//   (c) NVIC IRQ_QTIMER1 got disabled or stuck pending.
//   (d) BASEPRI is held at a value that masks priority 32 (the Patch 8
//       priority for QTimer1).  If foreground enters a long critical_enter
//       block and never exits, CH2 IRQ is masked indefinitely.
//
// Each PPS edge captures BASEPRI at ISR entry into the global below.  The
// enhanced PPS probe (under Patch 10, replaces the simpler Patch 9 emission)
// reads QTimer1 CH2 register state and the captured BASEPRI, and emits
// everything in one line.  Once CADENCE_MINDER stops, this probe keeps
// reporting the CH2 hardware state every second, so we can see exactly
// which mechanism produced the freeze.

static volatile uint32_t g_patch10_basepri_at_pps_entry = 0;

// OCXO compare-flag preflight diagnostics. These dry-run hardware probes run
// from the TimePop OCXO arm callback before compare enable is allowed. They
// answer whether the stale TCF flag can be cleared reliably at the intended
// 1 ms pre-edge arm point.
static volatile uint32_t g_ocxo1_arm_preflight_count = 0;
static volatile uint32_t g_ocxo1_arm_flag_before_clear = 0;
static volatile uint32_t g_ocxo1_arm_flag_after_clear = 0;
static volatile uint32_t g_ocxo1_arm_stale_flag_survived_count = 0;
static volatile uint32_t g_ocxo1_arm_last_csctrl_before_clear = 0;
static volatile uint32_t g_ocxo1_arm_last_csctrl_after_clear = 0;
static volatile uint32_t g_ocxo1_arm_compare_enabled_before_clear = 0;
static volatile uint32_t g_ocxo1_arm_compare_enabled_after_clear = 0;
static volatile uint32_t g_ocxo2_arm_preflight_count = 0;
static volatile uint32_t g_ocxo2_arm_flag_before_clear = 0;
static volatile uint32_t g_ocxo2_arm_flag_after_clear = 0;
static volatile uint32_t g_ocxo2_arm_stale_flag_survived_count = 0;
static volatile uint32_t g_ocxo2_arm_last_csctrl_before_clear = 0;
static volatile uint32_t g_ocxo2_arm_last_csctrl_after_clear = 0;
static volatile uint32_t g_ocxo2_arm_compare_enabled_before_clear = 0;
static volatile uint32_t g_ocxo2_arm_compare_enabled_after_clear = 0;

// Defensive compare-clear / live-gate diagnostics.  Preflight proved the
// canonical OCXO clear path is CSCTRL zero-clear, not write-one-to-clear.  The
// live gate still records zero-clear success/failure before compare enable.
static volatile uint32_t g_ocxo1_arm_last_sctrl_before_clear = 0;
static volatile uint32_t g_ocxo1_arm_last_sctrl_after_clear = 0;
static volatile uint32_t g_ocxo1_arm_csctrl_after_w1c = 0;
static volatile uint32_t g_ocxo1_arm_csctrl_after_zero_clear = 0;
static volatile uint32_t g_ocxo1_arm_csctrl_after_far_retarget = 0;
static volatile uint32_t g_ocxo1_arm_flag_after_w1c = 0;
static volatile uint32_t g_ocxo1_arm_flag_after_zero_clear = 0;
static volatile uint32_t g_ocxo1_arm_flag_after_far_retarget = 0;
static volatile uint32_t g_ocxo1_arm_clear_success_stage = 0;
static volatile uint32_t g_ocxo1_arm_clear_success_count = 0;
static volatile uint32_t g_ocxo1_arm_clear_failed_count = 0;
static volatile uint32_t g_ocxo1_arm_far_target_low16 = 0;
static volatile uint32_t g_ocxo1_arm_nvic_clear_count = 0;

static volatile uint32_t g_ocxo2_arm_last_sctrl_before_clear = 0;
static volatile uint32_t g_ocxo2_arm_last_sctrl_after_clear = 0;
static volatile uint32_t g_ocxo2_arm_csctrl_after_w1c = 0;
static volatile uint32_t g_ocxo2_arm_csctrl_after_zero_clear = 0;
static volatile uint32_t g_ocxo2_arm_csctrl_after_far_retarget = 0;
static volatile uint32_t g_ocxo2_arm_flag_after_w1c = 0;
static volatile uint32_t g_ocxo2_arm_flag_after_zero_clear = 0;
static volatile uint32_t g_ocxo2_arm_flag_after_far_retarget = 0;
static volatile uint32_t g_ocxo2_arm_clear_success_stage = 0;
static volatile uint32_t g_ocxo2_arm_clear_success_count = 0;
static volatile uint32_t g_ocxo2_arm_clear_failed_count = 0;
static volatile uint32_t g_ocxo2_arm_far_target_low16 = 0;
static volatile uint32_t g_ocxo2_arm_nvic_clear_count = 0;

// CLOCKS-owned OCXO logical grid, installed only after TimePop epoch-change
// cancellation has completed.  interrupt_start() must not create pre-epoch
// OCXO one-shots; it may only consume this grid once it exists.
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

  // Patch 2 — regression detector.  Snapshot the prior counter32 before any
  // write so we can compare against the new projection.  A synthetic tick must
  // never run backward; if delta wraps into the negative half-range (i.e. the
  // projected counter32 lands before current_counter32), record the regression
  // for diagnostic readout.  This is the smoking gun for an unguarded
  // read-modify-write race between CADENCE_MINDER (ISR) and a foreground
  // writer.  Note: we still apply the new value either way — the goal is to
  // observe the lie, not to suppress it.
  const uint32_t prior_counter32 = c.current_counter32;
  const uint32_t counter32 = project_counter32_from_hw16(c, hardware16);
  const uint32_t delta = counter32 - prior_counter32;

  if (delta > 0x80000000UL) {
    const uint32_t regression = prior_counter32 - counter32;
    if (&c == &g_ocxo1_clock32) {
      g_ocxo1_clock32_regression_count++;
      g_ocxo1_clock32_last_regression_delta = regression;
      if (regression > g_ocxo1_clock32_max_regression_delta) {
        g_ocxo1_clock32_max_regression_delta = regression;
      }
    } else if (&c == &g_ocxo2_clock32) {
      g_ocxo2_clock32_regression_count++;
      g_ocxo2_clock32_last_regression_delta = regression;
      if (regression > g_ocxo2_clock32_max_regression_delta) {
        g_ocxo2_clock32_max_regression_delta = regression;
      }
    } else {
      g_vclock_clock32_regression_count++;
      g_vclock_clock32_last_regression_delta = regression;
      if (regression > g_vclock_clock32_max_regression_delta) {
        g_vclock_clock32_max_regression_delta = regression;
      }
    }
  }

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
// Cadence minder — single TimePop custody agent for all low-word counters
// ============================================================================

static void cadence_minder_tend_ocxo(interrupt_subscriber_kind_t kind,
                                     ocxo_lane_t& lane,
                                     synthetic_clock32_t& clock32) {
  if (!lane.initialized) return;

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);
  lane.logical_count32_at_last_second = clock32.current_counter32;

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_cadence_minder_ocxo1_updates++;
    g_cadence_minder_last_ocxo1_hw16 = hw16;
    g_cadence_minder_last_ocxo1_counter32 = clock32.current_counter32;
  } else {
    g_cadence_minder_ocxo2_updates++;
    g_cadence_minder_last_ocxo2_hw16 = hw16;
    g_cadence_minder_last_ocxo2_counter32 = clock32.current_counter32;
  }

  // CADENCE_MINDER is custody only for OCXO lanes.  It keeps the synthetic
  // 32-bit extension warm, but it never authors OCXO one-second edge events.
  // OCXO edge facts come only from the QTimer2/QTimer3 compare ISR path.
}

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

  // OCXO lanes: custody only.  The minder tends low-word extensions;
  // it does not author OCXO one-second edge events.
  cadence_minder_tend_ocxo(interrupt_subscriber_kind_t::OCXO1,
                           g_ocxo1_lane,
                           g_ocxo1_clock32);
  cadence_minder_tend_ocxo(interrupt_subscriber_kind_t::OCXO2,
                           g_ocxo2_lane,
                           g_ocxo2_clock32);
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
// OCXO lanes — QTimer2 CH0 / QTimer3 CH3
// ============================================================================
//
static void ocxo_lane_clear_nvic_pending(const ocxo_lane_t& lane);
static void ocxo_lane_disable_compare_gate_only(ocxo_lane_t& lane);
static uint16_t ocxo_lane_zero_clear(ocxo_lane_t& lane);
static inline bool ocxo_csctrl_flag_set(uint16_t csctrl);

static bool ocxo_lane_program_compare_if_clear(ocxo_lane_t& lane,
                                                uint16_t target_low16) {
  // Live-gate safety: the compare interrupt may only be enabled after the
  // target is programmed and the compare flag is observed clear.  If the flag
  // is already asserted after programming, leave the gate closed and let the
  // arm callback reschedule the next opportunity.
  ocxo_lane_disable_compare_gate_only(lane);
  ocxo_lane_zero_clear(lane);

  lane.module->CH[lane.channel].COMP1  = target_low16;
  lane.module->CH[lane.channel].CMPLD1 = target_low16;

  const uint16_t cs_after_clear = ocxo_lane_zero_clear(lane);
  if (ocxo_csctrl_flag_set(cs_after_clear)) {
    ocxo_lane_disable_compare_gate_only(lane);
    ocxo_lane_clear_nvic_pending(lane);
    return false;
  }

  lane.module->CH[lane.channel].CSCTRL |= TMR_CSCTRL_TCF1EN;
  return true;
}

static void ocxo_lane_disable_compare(ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_disable_compare();
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_disable_compare(lane.channel);
  }
}

// ISR discipline for OCXO lanes is intentionally flat and tiny:
//   1. capture first-instruction DWT,
//   2. clear the compare flag,
//   3. compute the latency-adjusted event DWT,
//   4. re-arm the next 1 kHz compare,
//   5. refresh the synthetic 32-bit lane anchor,
//   6. defer only 1 Hz edge publication.
//
// Dynamic 100 Hz prediction has been retired; OCXO 100 Hz samples are no
// longer queued from the ISR.
// ============================================================================

struct ocxo_deferred_work_t {
  volatile bool edge_pending = false;
  volatile uint32_t edge_dwt = 0;
  volatile uint32_t edge_counter32 = 0;
};

static ocxo_deferred_work_t g_ocxo1_deferred = {};
static ocxo_deferred_work_t g_ocxo2_deferred = {};

// ----------------------------------------------------------------------------
// OCXO post-ISR mailbox — single slot per lane, drained by ASAP foreground.
// ----------------------------------------------------------------------------
//
// The QTimer2/QTimer3 ISR captures the minimum facts needed for hardware
// rearm and 1 Hz cadence accounting, then writes this mailbox and arms a
// TimePop ASAP slot.  All non-essential bookkeeping (synthetic clock advance,
// diag stores, late-ticks accounting, 1 Hz edge defer) runs in the ASAP
// callback in foreground context.
//
// Mailbox semantics:
//   • Single slot.  If a new ISR fires while the previous sample has not been
//     consumed, the new sample overwrites and overwrite_count is incremented.
//   • The lossy bookkeeping is acceptable because cadence-critical state
//     (lane.tick_mod_1000) is updated inside the ISR itself.
//   • Hardware re-arm and cadence counting cannot be lost.

struct ocxo_post_isr_mailbox_t {
  volatile bool     pending = false;
  volatile uint32_t isr_entry_dwt_raw = 0;
  volatile uint32_t event_dwt = 0;
  volatile uint16_t fired_low16 = 0;
  volatile uint32_t event_counter32 = 0;
  volatile bool     emit_one_second = false;
  volatile uint32_t sequence = 0;
};

static ocxo_post_isr_mailbox_t g_ocxo1_post_isr = {};
static ocxo_post_isr_mailbox_t g_ocxo2_post_isr = {};

static volatile uint32_t g_ocxo1_post_isr_overwrite_count = 0;
static volatile uint32_t g_ocxo2_post_isr_overwrite_count = 0;
static volatile uint32_t g_ocxo1_post_isr_arm_count = 0;
static volatile uint32_t g_ocxo2_post_isr_arm_count = 0;
static volatile uint32_t g_ocxo1_post_isr_drain_count = 0;
static volatile uint32_t g_ocxo2_post_isr_drain_count = 0;

static const char* ocxo_post_isr_name(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? "OCXO1_POST_ISR"
      : "OCXO2_POST_ISR";
}

static ocxo_post_isr_mailbox_t&
ocxo_post_isr_mailbox_for(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_post_isr
      : g_ocxo2_post_isr;
}

static void ocxo_post_isr_overwrite_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_post_isr_overwrite_count++;
  } else {
    g_ocxo2_post_isr_overwrite_count++;
  }
}

static void ocxo_post_isr_arm_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_post_isr_arm_count++;
  } else {
    g_ocxo2_post_isr_arm_count++;
  }
}

static void ocxo_post_isr_drain_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_post_isr_drain_count++;
  } else {
    g_ocxo2_post_isr_drain_count++;
  }
}

// Forward decl so the ISR can reference it.
static void ocxo_post_isr_asap_callback(timepop_ctx_t*,
                                        timepop_diag_t*,
                                        void* user_data);

static void ocxo_deferred_asap_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;

  ocxo_deferred_work_t& work =
      (rt->desc->kind == interrupt_subscriber_kind_t::OCXO1)
          ? g_ocxo1_deferred
          : g_ocxo2_deferred;
  if (work.edge_pending) {
    const uint32_t dwt = work.edge_dwt;
    const uint32_t counter32 = work.edge_counter32;
    work.edge_pending = false;
    g_ocxo_deferred_edge_count++;

    // We are already in foreground ASAP context.  Dispatch the authored OCXO
    // edge directly instead of arming a second named ASAP slot that can be
    // replaced by the next 100 Hz deferred sample before Alpha sees the edge.
    emit_one_second_event(*rt, dwt, counter32, 0, false, nullptr, true);
  }
}


static inline bool ocxo_lane_compare_flag_pending(const ocxo_lane_t& lane) {
  return (lane.module->CH[lane.channel].CSCTRL & TMR_CSCTRL_TCF1) != 0;
}

static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].CNTR;
}

static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_clear_compare_flag();
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_clear_compare_flag(lane.channel);
  }
}

static uint16_t ocxo_lane_csctrl_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].CSCTRL;
}

static uint16_t ocxo_lane_sctrl_now(const ocxo_lane_t& lane) {
  return lane.module->CH[lane.channel].SCTRL;
}

static inline bool ocxo_csctrl_flag_set(uint16_t csctrl) {
  return (csctrl & TMR_CSCTRL_TCF1) != 0;
}

static inline bool ocxo_csctrl_enabled(uint16_t csctrl) {
  return (csctrl & TMR_CSCTRL_TCF1EN) != 0;
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

static uint16_t ocxo_lane_zero_clear(ocxo_lane_t& lane) {
  // Proven clear path for OCXO QuadTimer compare flags: write TCF1/TCF2 low.
  // This matches the QTimer1 clear style and the OCXO preflight evidence.
  lane.module->CH[lane.channel].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  (void)lane.module->CH[lane.channel].CSCTRL;
  return lane.module->CH[lane.channel].CSCTRL;
}

static uint16_t ocxo_lane_retarget_far_then_zero_clear(ocxo_lane_t& lane,
                                                       uint16_t far_target_low16) {
  // Fallback only: move COMP/CMPLD away from the current low-word neighborhood
  // while the interrupt gate is disabled, then apply the proven zero-clear.
  lane.module->CH[lane.channel].COMP1  = far_target_low16;
  lane.module->CH[lane.channel].CMPLD1 = far_target_low16;
  return ocxo_lane_zero_clear(lane);
}

static bool ocxo_edge_arm_preflight_compare_flag(interrupt_subscriber_kind_t kind,
                                                 ocxo_lane_t& lane) {
  const uint16_t cs_before = ocxo_lane_csctrl_now(lane);
  const uint16_t s_before  = ocxo_lane_sctrl_now(lane);
  const bool flag_before = ocxo_csctrl_flag_set(cs_before);
  const bool enabled_before = ocxo_csctrl_enabled(cs_before);

  // Keep the compare interrupt gate closed during all clearing and target
  // programming preflight.  Enabling is performed only after the final observed
  // flag state is clear.
  ocxo_lane_disable_compare_gate_only(lane);

  const uint16_t cs_after_zero = ocxo_lane_zero_clear(lane);
  const bool flag_after_zero = ocxo_csctrl_flag_set(cs_after_zero);

  const uint16_t now_low16 = ocxo_lane_counter_now(lane);
  const uint16_t far_target_low16 = (uint16_t)(now_low16 + (uint16_t)0x4000U);

  uint16_t cs_after_far = cs_after_zero;
  bool flag_after_far = flag_after_zero;
  uint32_t success_stage = 0;

  if (!flag_after_zero) {
    success_stage = 1;
  } else {
    cs_after_far = ocxo_lane_retarget_far_then_zero_clear(lane, far_target_low16);
    flag_after_far = ocxo_csctrl_flag_set(cs_after_far);
    if (!flag_after_far) {
      success_stage = 2;
    }
  }

  // Clear any NVIC pending state after peripheral-level clear attempts.  This
  // is harmless if no IRQ is pending and gives the live-gate path a clean NVIC
  // slate immediately before COMP1/CMPLD1 programming.
  ocxo_lane_clear_nvic_pending(lane);

  const uint16_t cs_after = ocxo_lane_csctrl_now(lane);
  const uint16_t s_after  = ocxo_lane_sctrl_now(lane);
  const bool flag_after = ocxo_csctrl_flag_set(cs_after);
  const bool enabled_after = ocxo_csctrl_enabled(cs_after);
  const bool clear_ok = !flag_after && !enabled_after;

  if (clear_ok && success_stage == 0) {
    success_stage = 3;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_arm_preflight_count++;
    g_ocxo1_arm_flag_before_clear = flag_before ? 1U : 0U;
    g_ocxo1_arm_flag_after_clear = flag_after ? 1U : 0U;
    g_ocxo1_arm_last_csctrl_before_clear = (uint32_t)cs_before;
    g_ocxo1_arm_last_csctrl_after_clear = (uint32_t)cs_after;
    g_ocxo1_arm_compare_enabled_before_clear = enabled_before ? 1U : 0U;
    g_ocxo1_arm_compare_enabled_after_clear = enabled_after ? 1U : 0U;
    g_ocxo1_arm_last_sctrl_before_clear = (uint32_t)s_before;
    g_ocxo1_arm_last_sctrl_after_clear = (uint32_t)s_after;
    g_ocxo1_arm_csctrl_after_w1c = 0;
    g_ocxo1_arm_csctrl_after_zero_clear = (uint32_t)cs_after_zero;
    g_ocxo1_arm_csctrl_after_far_retarget = (uint32_t)cs_after_far;
    g_ocxo1_arm_flag_after_w1c = 0;
    g_ocxo1_arm_flag_after_zero_clear = flag_after_zero ? 1U : 0U;
    g_ocxo1_arm_flag_after_far_retarget = flag_after_far ? 1U : 0U;
    g_ocxo1_arm_clear_success_stage = success_stage;
    g_ocxo1_arm_far_target_low16 = (uint32_t)far_target_low16;
    g_ocxo1_arm_nvic_clear_count++;
    if (clear_ok) {
      g_ocxo1_arm_clear_success_count++;
    } else {
      g_ocxo1_arm_stale_flag_survived_count++;
      g_ocxo1_arm_clear_failed_count++;
    }
    return clear_ok;
  }

  if (kind == interrupt_subscriber_kind_t::OCXO2) {
    g_ocxo2_arm_preflight_count++;
    g_ocxo2_arm_flag_before_clear = flag_before ? 1U : 0U;
    g_ocxo2_arm_flag_after_clear = flag_after ? 1U : 0U;
    g_ocxo2_arm_last_csctrl_before_clear = (uint32_t)cs_before;
    g_ocxo2_arm_last_csctrl_after_clear = (uint32_t)cs_after;
    g_ocxo2_arm_compare_enabled_before_clear = enabled_before ? 1U : 0U;
    g_ocxo2_arm_compare_enabled_after_clear = enabled_after ? 1U : 0U;
    g_ocxo2_arm_last_sctrl_before_clear = (uint32_t)s_before;
    g_ocxo2_arm_last_sctrl_after_clear = (uint32_t)s_after;
    g_ocxo2_arm_csctrl_after_w1c = 0;
    g_ocxo2_arm_csctrl_after_zero_clear = (uint32_t)cs_after_zero;
    g_ocxo2_arm_csctrl_after_far_retarget = (uint32_t)cs_after_far;
    g_ocxo2_arm_flag_after_w1c = 0;
    g_ocxo2_arm_flag_after_zero_clear = flag_after_zero ? 1U : 0U;
    g_ocxo2_arm_flag_after_far_retarget = flag_after_far ? 1U : 0U;
    g_ocxo2_arm_clear_success_stage = success_stage;
    g_ocxo2_arm_far_target_low16 = (uint32_t)far_target_low16;
    g_ocxo2_arm_nvic_clear_count++;
    if (clear_ok) {
      g_ocxo2_arm_clear_success_count++;
    } else {
      g_ocxo2_arm_stale_flag_survived_count++;
      g_ocxo2_arm_clear_failed_count++;
    }
    return clear_ok;
  }

  return clear_ok;
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

static const char* ocxo_deferred_name(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? "OCXO1_DEFERRED"
      : "OCXO2_DEFERRED";
}

static void ocxo_defer_one_second_edge(ocxo_deferred_work_t& work,
                                       interrupt_subscriber_runtime_t* rt,
                                       interrupt_subscriber_kind_t kind,
                                       uint32_t event_dwt,
                                       uint32_t event_counter32) {
  if (work.edge_pending) g_ocxo_deferred_asap_overwrite_count++;

  work.edge_dwt = event_dwt;
  work.edge_counter32 = event_counter32;
  work.edge_pending = true;
  g_ocxo_deferred_asap_arm_count++;

  (void)timepop_arm_asap(ocxo_deferred_asap_callback,
                         rt,
                         ocxo_deferred_name(kind));
}

static const char* ocxo_edge_arm_name(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? OCXO1_EDGE_ARM_NAME
      : OCXO2_EDGE_ARM_NAME;
}

static const char* ocxo_edge_arm_rearm_name(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? OCXO1_EDGE_ARM_REARM_NAME
      : OCXO2_EDGE_ARM_REARM_NAME;
}

static void ocxo_edge_arm_rearm_asap_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_rearm_asap_count++;
  } else {
    g_ocxo2_edge_arm_rearm_asap_count++;
  }
}

static void ocxo_edge_arm_schedule_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_schedule_count++;
  } else {
    g_ocxo2_edge_arm_schedule_count++;
  }
}

static void ocxo_edge_arm_schedule_failure_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_schedule_failures++;
  } else {
    g_ocxo2_edge_arm_schedule_failures++;
  }
}

static void ocxo_edge_arm_callback_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_callback_count++;
  } else {
    g_ocxo2_edge_arm_callback_count++;
  }
}

static void ocxo_edge_arm_enable_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_enable_count++;
  } else {
    g_ocxo2_edge_arm_enable_count++;
  }
}

static void ocxo_edge_arm_record_schedule_diag(interrupt_subscriber_kind_t kind,
                                               uint32_t target_counter32,
                                               uint32_t now_counter32,
                                               uint32_t ticks_to_target,
                                               uint64_t delay_ns) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_last_target_counter32 = target_counter32;
    g_ocxo1_edge_arm_last_now_counter32 = now_counter32;
    g_ocxo1_edge_arm_last_ticks_to_target = ticks_to_target;
    g_ocxo1_edge_arm_last_delay_ns = delay_ns;
  } else {
    g_ocxo2_edge_arm_last_target_counter32 = target_counter32;
    g_ocxo2_edge_arm_last_now_counter32 = now_counter32;
    g_ocxo2_edge_arm_last_ticks_to_target = ticks_to_target;
    g_ocxo2_edge_arm_last_delay_ns = delay_ns;
  }
}

static void ocxo_edge_fire_inc(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_fire_count++;
  } else {
    g_ocxo2_edge_fire_count++;
  }
}

static void ocxo_edge_arm_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data);
static void ocxo_edge_arm_reschedule_asap_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data);
static void ocxo_schedule_epoch_grid_if_ready(interrupt_subscriber_runtime_t* rt);

static void ocxo_schedule_edge_arm_for_target(interrupt_subscriber_runtime_t* rt,
                                              uint32_t next_edge_counter32) {
  if (!rt || !rt->desc || !rt->active) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized || !lane->active) return;

  // Compute the arming delay from the OCXO lane itself.  The initial arm after
  // START/ZERO may be anywhere inside the lane's one-second grid, so a fixed
  // 999 ms delay can easily miss the first target.  CADENCE_MINDER may have
  // tended the synthetic counter recently, but take one fresh low-word sample
  // here so this calculation is lane-local and current.
  const uint16_t hw16 = ocxo_lane_counter_now(*lane);
  synthetic_clock_tend_from_hw16(*clock32, hw16);
  lane->logical_count32_at_last_second = clock32->current_counter32;

  uint32_t now_counter32 = clock32->current_counter32;
  uint32_t ticks_to_target = next_edge_counter32 - now_counter32;

  // Patch 1 — bounded iteration with diagnostic counters.  In normal operation
  // this loop iterates 0 or 1 times.  Anything over a couple iterations means
  // now_counter32 was either far ahead of or far behind the intended target,
  // which is consistent with a synthetic-clock race poisoning current_counter32
  // between the tend call above and the read below.  The hard cap at 600 is
  // well above the theoretical maximum of 2^32 / OCXO_EDGE_INTERVAL_COUNTS
  // (~430); reaching it means now_counter32 is unrecoverable for this pass and
  // we break out loudly rather than spin forever.
  uint32_t advance_iters = 0;
  bool advance_broke_out = false;
  while (ticks_to_target > 0x80000000UL ||
         ticks_to_target <= OCXO_EDGE_ARM_LEAD_TICKS) {
    next_edge_counter32 += OCXO_EDGE_INTERVAL_COUNTS;
    ticks_to_target = next_edge_counter32 - now_counter32;
    advance_iters++;
    if (advance_iters > 600U) {
      advance_broke_out = true;
      break;
    }
  }

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    g_ocxo1_edge_arm_advance_iter_count_total += advance_iters;
    g_ocxo1_edge_arm_advance_iter_count_last = advance_iters;
    if (advance_iters > g_ocxo1_edge_arm_advance_iter_count_max) {
      g_ocxo1_edge_arm_advance_iter_count_max = advance_iters;
    }
    if (advance_broke_out) g_ocxo1_edge_arm_advance_iter_break_count++;
  } else {
    g_ocxo2_edge_arm_advance_iter_count_total += advance_iters;
    g_ocxo2_edge_arm_advance_iter_count_last = advance_iters;
    if (advance_iters > g_ocxo2_edge_arm_advance_iter_count_max) {
      g_ocxo2_edge_arm_advance_iter_count_max = advance_iters;
    }
    if (advance_broke_out) g_ocxo2_edge_arm_advance_iter_break_count++;
  }

  if (advance_broke_out) {
    // now_counter32 is poisoned for this pass.  Do not arm — the lane will be
    // scheduled again at the next CADENCE_MINDER pulse via the normal rearm
    // path, and the iter_break_count above is the durable evidence that this
    // pass was abandoned.
    ocxo_edge_arm_schedule_failure_inc(kind);
    return;
  }

  lane->next_edge_counter32 = next_edge_counter32;
  lane->compare_target = (uint16_t)(next_edge_counter32 & 0xFFFFU);

  const uint32_t ticks_until_arm =
      ticks_to_target - OCXO_EDGE_ARM_LEAD_TICKS;
  const uint64_t arm_delay_ns = ocxo_ticks_to_estimated_gnss_ns(ticks_until_arm);

  // Patch 1 — short-delay witness.  If arm_delay_ns lands below the lead
  // duration itself, the new TimePop slot will be born essentially at-or-past
  // its deadline.  This is a precursor to a same-pass recall loop because
  // schedule_next will immediately rediscover the slot as expired.
  if (arm_delay_ns < (uint64_t)OCXO_EDGE_ARM_LEAD_NS) {
    if (kind == interrupt_subscriber_kind_t::OCXO1) {
      g_ocxo1_edge_arm_short_delay_count++;
      g_ocxo1_edge_arm_last_arm_delay_ns_lo = (uint32_t)arm_delay_ns;
    } else {
      g_ocxo2_edge_arm_short_delay_count++;
      g_ocxo2_edge_arm_last_arm_delay_ns_lo = (uint32_t)arm_delay_ns;
    }
  }

  ocxo_edge_arm_record_schedule_diag(kind,
                                     next_edge_counter32,
                                     now_counter32,
                                     ticks_to_target,
                                     arm_delay_ns);

  timepop_cancel_by_name(ocxo_edge_arm_name(kind));
  const timepop_handle_t h =
      timepop_arm(arm_delay_ns,
                  false,
                  ocxo_edge_arm_callback,
                  rt,
                  ocxo_edge_arm_name(kind));

  if (h == TIMEPOP_INVALID_HANDLE) {
    ocxo_edge_arm_schedule_failure_inc(kind);
    return;
  }

  ocxo_edge_arm_schedule_inc(kind);
}

static void ocxo_schedule_next_edge_arm(interrupt_subscriber_runtime_t* rt) {
  if (!rt || !rt->desc || !rt->active) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized || !lane->active) return;

  // Use the current OCXO hardware low word only to keep the synthetic extender
  // warm.  The *grid* is not derived from this read.  Once a lane has a full
  // 32-bit edge target, every dry-run rearm advances from that prior target by
  // exactly one OCXO second.  This prevents the scheduler from drifting early
  // by re-targeting from the callback time (which is intentionally 1 ms before
  // the edge).
  const uint16_t hw16 = ocxo_lane_counter_now(*lane);
  synthetic_clock_tend_from_hw16(*clock32, hw16);
  lane->logical_count32_at_last_second = clock32->current_counter32;

  if (lane->next_edge_counter32 == 0) {
    // No lane-local target has been born yet.  Do not synthesize a target from
    // "now + one second" here; the first target must come from the
    // CLOCKS-selected OCXO epoch grid so the lane remains phase-consistent
    // with ZERO/START.  If the grid is already valid this will schedule now;
    // otherwise it records a wait count and returns quietly.
    ocxo_schedule_epoch_grid_if_ready(rt);
    return;
  }

  ocxo_schedule_edge_arm_for_target(
      rt,
      lane->next_edge_counter32 + OCXO_EDGE_INTERVAL_COUNTS);
}

static void ocxo_edge_arm_reschedule_asap_callback(timepop_ctx_t*,
                                                   timepop_diag_t*,
                                                   void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc || !rt->active) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_edge_arm_rearm_asap_inc(kind);
  ocxo_schedule_next_edge_arm(rt);
}

static void ocxo_edge_arm_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc || !rt->active) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_edge_arm_callback_inc(kind);

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized || !lane->active) return;

  // Prove the stale compare flag is clear before opening the hardware gate.
  // If the preflight fails, do not enable; simply re-arm for the next OCXO edge
  // so the system remains alive and the failure counter remains visible.
  const bool preflight_ok = ocxo_edge_arm_preflight_compare_flag(kind, *lane);

  if (OCXO_EDGE_ARM_DRY_RUN || !preflight_ok) {
    (void)timepop_arm_asap(ocxo_edge_arm_reschedule_asap_callback,
                           rt,
                           ocxo_edge_arm_rearm_name(kind));
    return;
  }

  // LIVE GATE PATH: after successful zero-clear preflight, program the OCXO
  // low-word compare target and enable the compare interrupt only if the flag
  // remains clear after the target is installed.
  if (!ocxo_lane_program_compare_if_clear(*lane, lane->compare_target)) {
    (void)timepop_arm_asap(ocxo_edge_arm_reschedule_asap_callback,
                           rt,
                           ocxo_edge_arm_rearm_name(kind));
    return;
  }
  ocxo_edge_arm_enable_inc(kind);
}

// Minimal OCXO edge ISR.
//
// TimePop opens the compare gate roughly 1 ms before the predicted OCXO
// one-second edge.  The QTimer ISR is therefore quiet for almost the entire
// second, but remains the sole author of the OCXO edge fact when the compare
// match actually occurs.
//
// ISR discipline:
//   1. capture first-instruction DWT,
//   2. confirm/clear the compare flag,
//   3. disable the compare gate immediately,
//   4. latency-adjust DWT into event coordinates,
//   5. mailbox the edge fact for foreground publication.
//
// Synthetic clock custody, diagnostics, event dispatch, and scheduling the
// next TimePop compare-enable callback happen in ocxo_post_isr_asap_callback().

static void ocxo_lane_compare_isr(ocxo_lane_t& lane,
                                  interrupt_subscriber_runtime_t* rt,
                                  interrupt_subscriber_kind_t kind,
                                  synthetic_clock32_t& clock32,
                                  ocxo_deferred_work_t&,
                                  uint32_t isr_entry_dwt_raw) {
  if (!ocxo_lane_compare_flag_pending(lane)) return;

  const uint16_t fired_low16 = lane.compare_target;
  ocxo_lane_clear_compare_flag(lane);
  ocxo_lane_disable_compare(lane);

  lane.irq_count++;
  lane.cadence_hits_total++;
  ocxo_edge_fire_inc(kind);

  const uint32_t event_dwt = qtimer_event_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  const uint32_t event_counter32 = project_counter32_from_hw16(clock32, fired_low16);

  ocxo_post_isr_mailbox_t& mb = ocxo_post_isr_mailbox_for(kind);
  if (mb.pending) {
    ocxo_post_isr_overwrite_inc(kind);
  }

  mb.isr_entry_dwt_raw = isr_entry_dwt_raw;
  mb.event_dwt = event_dwt;
  mb.fired_low16 = fired_low16;
  mb.event_counter32 = event_counter32;
  mb.emit_one_second = true;
  mb.sequence++;
  mb.pending = true;

  ocxo_post_isr_arm_inc(kind);
  (void)timepop_arm_asap(ocxo_post_isr_asap_callback,
                         rt,
                         ocxo_post_isr_name(kind));
}

// Foreground drain — runs at TimePop ASAP priority, no ISR pressure.
//
// Snapshots the mailbox under critical-section guard, then performs all
// synthetic clock and diagnostic work outside the critical section.

static void ocxo_post_isr_asap_callback(timepop_ctx_t*,
                                        timepop_diag_t*,
                                        void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_post_isr_mailbox_t& mb = ocxo_post_isr_mailbox_for(kind);

  // Snapshot under PRIMASK so an ISR that fires mid-drain cannot tear the
  // sample we are about to consume.  The ISR may overwrite *after* we exit
  // this section; that simply queues a fresh ASAP for the next sample.
  uint32_t isr_entry_dwt_raw;
  uint32_t event_dwt;
  uint16_t fired_low16;
  uint32_t event_counter32;
  bool     emit_one_second;
  bool     had_sample;

  __disable_irq();
  had_sample = mb.pending;
  if (had_sample) {
    isr_entry_dwt_raw = mb.isr_entry_dwt_raw;
    event_dwt         = mb.event_dwt;
    fired_low16       = mb.fired_low16;
    event_counter32   = mb.event_counter32;
    emit_one_second   = mb.emit_one_second;
    mb.pending = false;
  } else {
    isr_entry_dwt_raw = 0;
    event_dwt         = 0;
    fired_low16       = 0;
    event_counter32   = 0;
    emit_one_second   = false;
  }
  __enable_irq();

  if (!had_sample) return;

  ocxo_post_isr_drain_inc(kind);

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return;

  // Late-ticks moral equivalent: we are running in foreground, so any
  // CNTR read here is "drain latency in OCXO ticks", not "ISR-entry-to-
  // counter ticks".  The diag store retains its existing field shape so
  // downstream consumers do not see a structural change; the value's
  // semantics shift from ISR-entry-late to drain-late.
  const uint32_t drain_ticks =
      (uint32_t)((uint16_t)(ocxo_lane_counter_now(*lane) - fired_low16));

  ocxo_lane_record_isr_diag(kind, isr_entry_dwt_raw, event_dwt, drain_ticks);

  // The ISR already authored the exact synthetic counter32 identity for the
  // fired low-word.  CADENCE_MINDER may have advanced the ambient synthetic
  // state before this foreground drain runs, so do not recompute the edge
  // identity here from a newer anchor.
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (clock32) {
    const uint32_t delta_to_edge = event_counter32 - clock32->current_counter32;
    if (delta_to_edge < 0x80000000UL) {
      clock32->current_counter32 = event_counter32;
      clock32->current_ns += (uint64_t)delta_to_edge * 100ULL;
      clock32->hardware16 = fired_low16;
    }
    lane->logical_count32_at_last_second = event_counter32;
  }

  // 1 Hz edge defer — chain into the existing publication path so all
  // downstream behavior is preserved exactly.
  if (emit_one_second) {
    ocxo_deferred_work_t& work =
        (kind == interrupt_subscriber_kind_t::OCXO1)
            ? g_ocxo1_deferred
            : g_ocxo2_deferred;
    ocxo_defer_one_second_edge(work,
                               rt,
                               kind,
                               event_dwt,
                               event_counter32);
    ocxo_schedule_edge_arm_for_target(
        rt, event_counter32 + OCXO_EDGE_INTERVAL_COUNTS);
  }
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  ocxo_lane_compare_isr(g_ocxo1_lane,
                        g_rt_ocxo1,
                        interrupt_subscriber_kind_t::OCXO1,
                        g_ocxo1_clock32,
                        g_ocxo1_deferred,
                        isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  ocxo_lane_compare_isr(g_ocxo2_lane,
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

  // ── Patch 9 + Patch 10 + Patch 11 — PPS aliveness + CH2 forensics + TimePop state ──
  //
  // Emit one debug line per second from PPS GPIO ISR context (priority 0).
  // Carries:
  //   • Patch 9/10: PPS counters, CH2 register forensics, NVIC enable/pending,
  //     and BASEPRI at PPS ISR entry.
  //   • Patch 11: CADENCE_MINDER slot state + TimePop scheduler counters,
  //     read via timepop_get_freeze_diag().  This tells us what state
  //     CADENCE_MINDER is sitting in at the freeze moment and whether
  //     TimePop's scheduler is busy-looping or quiet.
  //
  // Existing fields (Patch 9/10):
  //   emit=         — PPS probe emission count
  //   gpio=         — total PPS GPIO ISR entries
  //   cad_fires=    — g_cadence_minder_fire_count (frozen at death)
  //   ch2_cntr=     — current CH2 counter (low 16 bits)
  //   ch2_comp1=    — current CH2 compare target
  //   ch2_csctrl=   — CH2 CSCTRL register
  //   nvic_en=      — IRQ_QTIMER1 enabled in NVIC
  //   nvic_pn=      — IRQ_QTIMER1 pending in NVIC
  //   bpri=         — BASEPRI at PPS ISR entry
  //
  // New fields (Patch 11):
  //   cm_a=         — CADENCE_MINDER slot.active
  //   cm_e=         — CADENCE_MINDER slot.expired (true means waiting for dispatch)
  //   cm_icf=       — CADENCE_MINDER slot.isr_callback_fired
  //   cm_dl=        — CADENCE_MINDER slot.deadline (32-bit VCLOCK)
  //   cm_tgt32=     — low 32 bits of slot.target_gnss_ns (hex)
  //   cm_rar=       — recurring_rearmed_count (total rearms across all paths)
  //   cm_irq=       — irq_expired_by_irq_count (IRQ caught the deadline)
  //   cm_sxe=       — schedule_next_expired_count (foreground caught past-deadline)
  //   cm_sk=        — recurring_total_skipped_intervals
  //   sn=           — TimePop diag_schedule_next_calls_total
  //   rearm=        — TimePop diag_rearm_count (CH2 arms)
  //   hb=           — TimePop diag_heartbeat_rearms (no slot had near deadline)
  //   disp=         — TimePop diag_dispatch_calls (dispatch ran)
  //   isr=          — TimePop diag_isr_count (CH2 IRQ entries)
  //
  // Interpretation patterns:
  //   • cm_e=1 stuck across many samples → CADENCE_MINDER is expired but
  //     dispatch isn't processing it (or isr_callback_fired=true keeps
  //     skipping the callback path).
  //   • cm_e=0 stuck → schedule_next never marks it past-deadline, meaning
  //     cm_dl always appears in the future.  cm_dl should then be advancing.
  //   • cm_rar growing but cad_fires NOT growing → rearm is happening but
  //     callback is being skipped (e.g., isr_callback_fired stuck true).
  //   • sn growing by millions per second → schedule_next runaway loop.
  //   • hb growing fast → schedule_next can't find any near deadline and
  //     keeps arming the 1 ms heartbeat.
  //   • disp growing slowly → dispatch is rarely running, timepop_pending
  //     is rarely set.
  const uint32_t pps_now_ms = millis();
  if ((uint32_t)(pps_now_ms - g_patch9_pps_last_emit_ms) >= 1000) {
    g_patch9_pps_last_emit_ms = pps_now_ms;
    g_patch9_pps_emit_count++;

    const uint16_t ch2_cntr   = IMXRT_TMR1.CH[2].CNTR;
    const uint16_t ch2_comp1  = IMXRT_TMR1.CH[2].COMP1;
    const uint16_t ch2_csctrl = IMXRT_TMR1.CH[2].CSCTRL;

    // NVIC introspection — read the ISER and ISPR bits for IRQ_QTIMER1.
    // IRQ_QTIMER1 numeric value > 32, so we use the appropriate ISER/ISPR
    // bank.  Access via CMSIS-style pattern available through imxrt.h.
    const uint32_t irq_num    = (uint32_t)IRQ_QTIMER1;
    const uint32_t irq_bank   = irq_num >> 5;
    const uint32_t irq_bit    = irq_num & 31U;
    const uint32_t iser_word  = ((volatile uint32_t*)0xE000E100)[irq_bank];
    const uint32_t ispr_word  = ((volatile uint32_t*)0xE000E200)[irq_bank];
    const unsigned nvic_en    = (iser_word >> irq_bit) & 1U;
    const unsigned nvic_pn    = (ispr_word >> irq_bit) & 1U;

    // Patch 11: snapshot TimePop's view of CADENCE_MINDER and scheduler.
    timepop_freeze_diag_t td;
    timepop_get_freeze_diag(td);
    const uint32_t cm_tgt32 = (uint32_t)((uint64_t)td.cm_target_gnss_ns & 0xFFFFFFFFULL);

    char buf[512];
    snprintf(buf, sizeof(buf),
             "emit=%lu gpio=%lu cad_fires=%lu "
             "ch2_cntr=%u ch2_comp1=%u ch2_csctrl=0x%04x "
             "nvic_en=%u nvic_pn=%u bpri=%lu "
             "cm_a=%u cm_e=%u cm_icf=%u cm_dl=%lu cm_tgt32=%08lx "
             "cm_rar=%lu cm_irq=%lu cm_sxe=%lu cm_sk=%lu "
             "sn=%lu rearm=%lu hb=%lu disp=%lu isr=%lu",
             (unsigned long)g_patch9_pps_emit_count,
             (unsigned long)g_gpio_irq_count,
             (unsigned long)g_cadence_minder_fire_count,
             (unsigned)ch2_cntr,
             (unsigned)ch2_comp1,
             (unsigned)ch2_csctrl,
             nvic_en,
             nvic_pn,
             (unsigned long)g_patch10_basepri_at_pps_entry,
             (unsigned)(td.cm_found && td.cm_active ? 1 : 0),
             (unsigned)(td.cm_found && td.cm_expired ? 1 : 0),
             (unsigned)(td.cm_found && td.cm_isr_callback_fired ? 1 : 0),
             (unsigned long)td.cm_deadline,
             (unsigned long)cm_tgt32,
             (unsigned long)td.cm_recurring_rearmed_count,
             (unsigned long)td.cm_irq_expired_by_irq_count,
             (unsigned long)td.cm_schedule_next_expired_count,
             (unsigned long)td.cm_recurring_total_skipped_intervals,
             (unsigned long)td.schedule_next_calls_total,
             (unsigned long)td.rearm_count,
             (unsigned long)td.heartbeat_rearms,
             (unsigned long)td.dispatch_calls,
             (unsigned long)td.isr_count);
    debug_log("pps_alive", buf);
  }
}

static void pps_gpio_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;
  // Patch 10: capture BASEPRI immediately on ISR entry.  This reflects what
  // BASEPRI was in the preempted context (foreground or another ISR).  If
  // CADENCE_MINDER is being masked by an overlong critical_enter, this will
  // show BASEPRI=16 (or whatever mask is in effect) at the moment PPS fired.
  uint32_t basepri_at_entry;
  __asm volatile("mrs %0, basepri" : "=r" (basepri_at_entry));
  g_patch10_basepri_at_pps_entry = basepri_at_entry;

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
    // CADENCE_MINDER is armed once during process_interrupt_init() and is the
    // only 1 ms TimePop cadence source.  Starting VCLOCK must not arm a second
    // cadence slot.
    return g_cadence_minder_armed || cadence_minder_arm_timepop();
  }

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane || !lane->initialized) return false;
  lane->active = true;
  lane->phase_bootstrapped = true;
  lane->tick_mod_1000 = 0;

  // OCXO lanes use QTimer compare only as an edge gate.  Keep the compare
  // disabled until TimePop opens it 1 ms before the predicted next edge.
  ocxo_lane_disable_compare(*lane);
  lane->compare_target = 0;
  lane->next_edge_counter32 = 0;

  const bool minder_ok = g_cadence_minder_armed || cadence_minder_arm_timepop();
  if (minder_ok) {
    // Do not create pre-epoch OCXO TimePop one-shots here.  They are unsafe
    // because timepop_epoch_changed() cancels one-shots during CLOCKS.ZERO /
    // START.  If CLOCKS has already installed the OCXO grid, consume it;
    // otherwise wait for interrupt_ocxo_logical_grid_epoch().
    ocxo_schedule_epoch_grid_if_ready(rt);
  }
  return minder_ok;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;
  rt->active = false;
  rt->stop_count++;

  if (kind == interrupt_subscriber_kind_t::VCLOCK) {
    g_vclock_lane.active = false;
    g_vclock_lane.phase_bootstrapped = false;
    // Do not cancel CADENCE_MINDER here; it is the system-wide substrate
    // cadence for VCLOCK and the OCXO passive counter domains.
    return true;
  }
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  if (!lane) return false;
  lane->active = false;
  timepop_cancel_by_name(ocxo_edge_arm_name(kind));
  timepop_cancel_by_name(ocxo_edge_arm_rearm_name(kind));
  ocxo_lane_disable_compare(*lane);
  lane->next_edge_counter32 = 0;
  return true;
}

static uint32_t next_ocxo_grid_target_after(uint32_t epoch_counter32,
                                            uint32_t current_counter32) {
  const uint32_t delta = current_counter32 - epoch_counter32;
  const uint32_t interval = OCXO_EDGE_INTERVAL_COUNTS;
  const uint32_t intervals_completed = (delta / interval) + 1U;
  return epoch_counter32 + intervals_completed * interval;
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

static void ocxo_schedule_epoch_grid_if_ready(interrupt_subscriber_runtime_t* rt) {
  if (!rt || !rt->desc || !rt->active) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized || !lane->active) return;

  if (!g_ocxo_grid_epoch_valid) {
    ocxo_grid_epoch_wait_inc(kind);
    return;
  }

  const uint32_t epoch_counter32 =
      (kind == interrupt_subscriber_kind_t::OCXO1)
          ? g_ocxo1_grid_epoch_counter32
          : g_ocxo2_grid_epoch_counter32;

  // The grid is CLOCKS-authored.  Take a fresh low-word sample only to locate
  // the next grid point from the current OCXO lane position.
  const uint16_t hw16 = ocxo_lane_counter_now(*lane);
  synthetic_clock_tend_from_hw16(*clock32, hw16);
  lane->logical_count32_at_last_second = clock32->current_counter32;

  ocxo_grid_epoch_start_inc(kind);
  ocxo_schedule_edge_arm_for_target(
      rt,
      next_ocxo_grid_target_after(epoch_counter32, clock32->current_counter32));
}

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32) {
  g_ocxo1_grid_epoch_counter32 = ocxo1_epoch_counter32;
  g_ocxo2_grid_epoch_counter32 = ocxo2_epoch_counter32;
  g_ocxo_grid_epoch_valid = true;

  if (g_rt_ocxo1 && g_ocxo1_lane.initialized) {
    ocxo_lane_disable_compare(g_ocxo1_lane);
    timepop_cancel_by_name(OCXO1_EDGE_ARM_NAME);
    timepop_cancel_by_name(OCXO1_EDGE_ARM_REARM_NAME);
    ocxo_schedule_epoch_grid_if_ready(g_rt_ocxo1);
  }

  if (g_rt_ocxo2 && g_ocxo2_lane.initialized) {
    ocxo_lane_disable_compare(g_ocxo2_lane);
    timepop_cancel_by_name(OCXO2_EDGE_ARM_NAME);
    timepop_cancel_by_name(OCXO2_EDGE_ARM_REARM_NAME);
    ocxo_schedule_epoch_grid_if_ready(g_rt_ocxo2);
  }
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
  // OCXO arm windows are tied to the CLOCKS-selected logical grid.  Invalidate
  // the old grid until Alpha installs the next epoch and calls
  // interrupt_ocxo_logical_grid_epoch() after timepop_epoch_changed().
  g_ocxo_grid_epoch_valid = false;
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
  g_ocxo1_edge_arm_schedule_count = 0;
  g_ocxo1_edge_arm_schedule_failures = 0;
  g_ocxo1_edge_arm_callback_count = 0;
  g_ocxo1_edge_arm_enable_count = 0;
  g_ocxo1_edge_fire_count = 0;
  g_ocxo1_edge_arm_last_target_counter32 = 0;
  g_ocxo1_edge_arm_last_now_counter32 = 0;
  g_ocxo1_edge_arm_last_ticks_to_target = 0;
  g_ocxo1_edge_arm_last_delay_ns = 0;
  g_ocxo2_edge_arm_schedule_count = 0;
  g_ocxo2_edge_arm_schedule_failures = 0;
  g_ocxo2_edge_arm_callback_count = 0;
  g_ocxo2_edge_arm_enable_count = 0;
  g_ocxo2_edge_fire_count = 0;
  g_ocxo2_edge_arm_last_target_counter32 = 0;
  g_ocxo2_edge_arm_last_now_counter32 = 0;
  g_ocxo2_edge_arm_last_ticks_to_target = 0;
  g_ocxo2_edge_arm_last_delay_ns = 0;
  g_ocxo1_edge_arm_rearm_asap_count = 0;
  g_ocxo2_edge_arm_rearm_asap_count = 0;

  // Patch 1 — zero advance-iter and short-delay diagnostics.
  g_ocxo1_edge_arm_advance_iter_count_total = 0;
  g_ocxo1_edge_arm_advance_iter_count_max = 0;
  g_ocxo1_edge_arm_advance_iter_count_last = 0;
  g_ocxo1_edge_arm_advance_iter_break_count = 0;
  g_ocxo1_edge_arm_last_arm_delay_ns_lo = 0;
  g_ocxo1_edge_arm_short_delay_count = 0;
  g_ocxo2_edge_arm_advance_iter_count_total = 0;
  g_ocxo2_edge_arm_advance_iter_count_max = 0;
  g_ocxo2_edge_arm_advance_iter_count_last = 0;
  g_ocxo2_edge_arm_advance_iter_break_count = 0;
  g_ocxo2_edge_arm_last_arm_delay_ns_lo = 0;
  g_ocxo2_edge_arm_short_delay_count = 0;

  // Patch 2 — zero synthetic-clock regression counters.
  g_ocxo1_clock32_regression_count = 0;
  g_ocxo1_clock32_last_regression_delta = 0;
  g_ocxo1_clock32_max_regression_delta = 0;
  g_ocxo2_clock32_regression_count = 0;
  g_ocxo2_clock32_last_regression_delta = 0;
  g_ocxo2_clock32_max_regression_delta = 0;
  g_vclock_clock32_regression_count = 0;
  g_vclock_clock32_last_regression_delta = 0;
  g_vclock_clock32_max_regression_delta = 0;

  g_ocxo1_arm_preflight_count = 0;
  g_ocxo1_arm_flag_before_clear = 0;
  g_ocxo1_arm_flag_after_clear = 0;
  g_ocxo1_arm_stale_flag_survived_count = 0;
  g_ocxo1_arm_last_csctrl_before_clear = 0;
  g_ocxo1_arm_last_csctrl_after_clear = 0;
  g_ocxo1_arm_compare_enabled_before_clear = 0;
  g_ocxo1_arm_compare_enabled_after_clear = 0;
  g_ocxo1_arm_last_sctrl_before_clear = 0;
  g_ocxo1_arm_last_sctrl_after_clear = 0;
  g_ocxo1_arm_csctrl_after_w1c = 0;
  g_ocxo1_arm_csctrl_after_zero_clear = 0;
  g_ocxo1_arm_csctrl_after_far_retarget = 0;
  g_ocxo1_arm_flag_after_w1c = 0;
  g_ocxo1_arm_flag_after_zero_clear = 0;
  g_ocxo1_arm_flag_after_far_retarget = 0;
  g_ocxo1_arm_clear_success_stage = 0;
  g_ocxo1_arm_clear_success_count = 0;
  g_ocxo1_arm_clear_failed_count = 0;
  g_ocxo1_arm_far_target_low16 = 0;
  g_ocxo1_arm_nvic_clear_count = 0;
  g_ocxo2_arm_preflight_count = 0;
  g_ocxo2_arm_flag_before_clear = 0;
  g_ocxo2_arm_flag_after_clear = 0;
  g_ocxo2_arm_stale_flag_survived_count = 0;
  g_ocxo2_arm_last_csctrl_before_clear = 0;
  g_ocxo2_arm_last_csctrl_after_clear = 0;
  g_ocxo2_arm_compare_enabled_before_clear = 0;
  g_ocxo2_arm_compare_enabled_after_clear = 0;
  g_ocxo2_arm_last_sctrl_before_clear = 0;
  g_ocxo2_arm_last_sctrl_after_clear = 0;
  g_ocxo2_arm_csctrl_after_w1c = 0;
  g_ocxo2_arm_csctrl_after_zero_clear = 0;
  g_ocxo2_arm_csctrl_after_far_retarget = 0;
  g_ocxo2_arm_flag_after_w1c = 0;
  g_ocxo2_arm_flag_after_zero_clear = 0;
  g_ocxo2_arm_flag_after_far_retarget = 0;
  g_ocxo2_arm_clear_success_stage = 0;
  g_ocxo2_arm_clear_success_count = 0;
  g_ocxo2_arm_clear_failed_count = 0;
  g_ocxo2_arm_far_target_low16 = 0;
  g_ocxo2_arm_nvic_clear_count = 0;
  g_ocxo1_lane.next_edge_counter32 = 0;
  g_ocxo2_lane.next_edge_counter32 = 0;
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

  // ── Patch 8 — priority architecture ────────────────────────────────────
  //
  // Priority ordering (lower number = higher priority on ARM Cortex-M):
  //
  //   PPS GPIO  : 0   — physical edge from the GF-8802; labeling witness for
  //                     the canonical second.  Sovereign and uncontested.
  //   OCXO 1/2  : 16  — physical hardware compare events.  Their ISR-entry
  //                     DWT is the OCXO edge fact; they must not be delayed
  //                     by the TimePop bookkeeping substrate.
  //   QTimer1   : 32  — TimePop's CH2 scheduler and CADENCE_MINDER.  This is
  //                     software bookkeeping over the slot tables; it sits
  //                     below the physical-event observers.
  //
  // The doctrinal alignment: events that observe physical reality (PPS, OCXO
  // edges) outrank the substrate that maintains software bookkeeping
  // (TimePop slot tables, CADENCE_MINDER counter tending).  This is the
  // priority architecture demanded by "interrupts are the sole lawful timing
  // truth" — the physical observers must always run promptly, while the
  // bookkeeping can be paused under critical_enter() without compromising
  // measurement integrity.
  //
  // BASEPRI invariant: BASEPRI_MASK_BELOW_PPS = 16 in util.h.  A foreground
  // critical_enter() call sets BASEPRI to 16, which masks all interrupts at
  // priority ≥ 16.  Under this scheme that includes QTimer1 (32) and OCXO
  // (16) — meaning the TimePop slot tables can be safely mutated under
  // critical_enter() without racing against CADENCE_MINDER or any other
  // CH2-IRQ-context slot-table writer.  PPS GPIO at priority 0 remains
  // unmasked and sovereign, as it must.
  //
  // Consequences of this scheme worth being explicit about:
  //
  //   • PPS can now preempt QTimer1 mid-flight.  This is correct — PPS is
  //     the labeling witness for the canonical second; nothing should delay
  //     it.  QTimer1 ISRs do not touch the synchronization surfaces that
  //     PPS reads, so the preemption is harmless.
  //
  //   • OCXO can now preempt QTimer1 mid-flight.  Also correct — OCXO edge
  //     facts capture ARM_DWT_CYCCNT as their first instruction, and any
  //     QTimer1-induced delay would corrupt that capture.
  //
  //   • CADENCE_MINDER (in QTimer1) can be delayed by a foreground
  //     critical_enter() critical section.  Critical sections in TimePop
  //     are short (slot-table walks, microseconds at most), so the delay
  //     to CADENCE_MINDER's 1 ms cadence is negligible.  Keep critical
  //     sections short.
  //
  //   • The PPS ISR is now genuinely uncontested at priority 0.  Previously
  //     PPS and QTimer1 were peers at priority 0, so neither could preempt
  //     the other; now PPS is sovereign.  ISR-entry latency calibration
  //     constants (CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES,
  //     *_ISR_FIXED_OVERHEAD) were empirically near-invariant to the old
  //     equal-priority arrangement, but spot-check the post-flash
  //     measurements against prior baselines and re-measure if they shift.

  attachInterruptVector(IRQ_QTIMER1, qtimer1_isr);
  // QTimer1 hosts TimePop's CH2 scheduler and CADENCE_MINDER.  This is
  // bookkeeping, not physical-event observation; it sits below PPS and OCXO.
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 32);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  // OCXO1 compare match — physical hardware event.  Above QTimer1 so its
  // ISR-entry DWT capture is not delayed by bookkeeping ISRs.
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  // OCXO2 compare match — same rationale as QTimer2.
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  // PPS GPIO — labeling witness for the canonical second.  Sovereign at
  // priority 0; nothing in the system may delay it.
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
  p.add("single_cadence_agent", true);
  p.add("cadence_minder_isr_mode", true);
  p.add("vclock_cadence_retired", true);
  p.add("ocxo_legacy_compare_cadence_retired", true);
  p.add("ocxo_edges_authored_by_qtimer_isr", true);
  p.add("ocxo_edges_armed_by_timepop", true);
  p.add("ocxo_edge_arm_dry_run", OCXO_EDGE_ARM_DRY_RUN);
  p.add("ocxo_compare_hw_enable_suppressed", OCXO_EDGE_ARM_DRY_RUN);
  p.add("ocxo_rearm_uses_prior_edge_target", true);
  p.add("ocxo_rearm_current_plus_interval_disabled", true);
  p.add("ocxo_edge_arm_lead_ns", (uint64_t)OCXO_EDGE_ARM_LEAD_NS);
  p.add("ocxo_edge_arm_lead_ticks", (uint32_t)OCXO_EDGE_ARM_LEAD_TICKS);
  p.add("ocxo_edge_interval_counts", (uint32_t)OCXO_EDGE_INTERVAL_COUNTS);
  p.add("ocxo_edge_tick_to_gnss_ns_estimate", 100U);
  p.add("ocxo_compare_live_requires_enabled_and_flag", true);
  p.add("ocxo_grid_epoch_valid", g_ocxo_grid_epoch_valid);
  p.add("ocxo1_grid_epoch_counter32", g_ocxo1_grid_epoch_counter32);
  p.add("ocxo2_grid_epoch_counter32", g_ocxo2_grid_epoch_counter32);
  p.add("ocxo1_grid_epoch_start_count", g_ocxo1_grid_epoch_start_count);
  p.add("ocxo2_grid_epoch_start_count", g_ocxo2_grid_epoch_start_count);
  p.add("ocxo1_grid_epoch_wait_count", g_ocxo1_grid_epoch_wait_count);
  p.add("ocxo2_grid_epoch_wait_count", g_ocxo2_grid_epoch_wait_count);
  p.add("ocxo_arm_preflight_enabled", true);
  p.add("ocxo1_arm_preflight_count", g_ocxo1_arm_preflight_count);
  p.add("ocxo1_arm_flag_before_clear", g_ocxo1_arm_flag_before_clear);
  p.add("ocxo1_arm_flag_after_clear", g_ocxo1_arm_flag_after_clear);
  p.add("ocxo1_arm_stale_flag_survived_count", g_ocxo1_arm_stale_flag_survived_count);
  p.add("ocxo1_arm_last_csctrl_before_clear", g_ocxo1_arm_last_csctrl_before_clear);
  p.add("ocxo1_arm_last_csctrl_after_clear", g_ocxo1_arm_last_csctrl_after_clear);
  p.add("ocxo1_arm_compare_enabled_before_clear", g_ocxo1_arm_compare_enabled_before_clear);
  p.add("ocxo1_arm_compare_enabled_after_clear", g_ocxo1_arm_compare_enabled_after_clear);
  p.add("ocxo1_arm_last_sctrl_before_clear", g_ocxo1_arm_last_sctrl_before_clear);
  p.add("ocxo1_arm_last_sctrl_after_clear", g_ocxo1_arm_last_sctrl_after_clear);
  p.add("ocxo1_arm_csctrl_after_zero_clear", g_ocxo1_arm_csctrl_after_zero_clear);
  p.add("ocxo1_arm_csctrl_after_far_retarget", g_ocxo1_arm_csctrl_after_far_retarget);
  p.add("ocxo1_arm_flag_after_zero_clear", g_ocxo1_arm_flag_after_zero_clear);
  p.add("ocxo1_arm_flag_after_far_retarget", g_ocxo1_arm_flag_after_far_retarget);
  p.add("ocxo1_arm_clear_success_stage", g_ocxo1_arm_clear_success_stage);
  p.add("ocxo1_arm_clear_success_count", g_ocxo1_arm_clear_success_count);
  p.add("ocxo1_arm_clear_failed_count", g_ocxo1_arm_clear_failed_count);
  p.add("ocxo1_arm_far_target_low16", g_ocxo1_arm_far_target_low16);
  p.add("ocxo1_arm_nvic_clear_count", g_ocxo1_arm_nvic_clear_count);
  p.add("ocxo2_arm_preflight_count", g_ocxo2_arm_preflight_count);
  p.add("ocxo2_arm_flag_before_clear", g_ocxo2_arm_flag_before_clear);
  p.add("ocxo2_arm_flag_after_clear", g_ocxo2_arm_flag_after_clear);
  p.add("ocxo2_arm_stale_flag_survived_count", g_ocxo2_arm_stale_flag_survived_count);
  p.add("ocxo2_arm_last_csctrl_before_clear", g_ocxo2_arm_last_csctrl_before_clear);
  p.add("ocxo2_arm_last_csctrl_after_clear", g_ocxo2_arm_last_csctrl_after_clear);
  p.add("ocxo2_arm_compare_enabled_before_clear", g_ocxo2_arm_compare_enabled_before_clear);
  p.add("ocxo2_arm_compare_enabled_after_clear", g_ocxo2_arm_compare_enabled_after_clear);
  p.add("ocxo2_arm_last_sctrl_before_clear", g_ocxo2_arm_last_sctrl_before_clear);
  p.add("ocxo2_arm_last_sctrl_after_clear", g_ocxo2_arm_last_sctrl_after_clear);
  p.add("ocxo2_arm_csctrl_after_zero_clear", g_ocxo2_arm_csctrl_after_zero_clear);
  p.add("ocxo2_arm_csctrl_after_far_retarget", g_ocxo2_arm_csctrl_after_far_retarget);
  p.add("ocxo2_arm_flag_after_zero_clear", g_ocxo2_arm_flag_after_zero_clear);
  p.add("ocxo2_arm_flag_after_far_retarget", g_ocxo2_arm_flag_after_far_retarget);
  p.add("ocxo2_arm_clear_success_stage", g_ocxo2_arm_clear_success_stage);
  p.add("ocxo2_arm_clear_success_count", g_ocxo2_arm_clear_success_count);
  p.add("ocxo2_arm_clear_failed_count", g_ocxo2_arm_clear_failed_count);
  p.add("ocxo2_arm_far_target_low16", g_ocxo2_arm_far_target_low16);
  p.add("ocxo2_arm_nvic_clear_count", g_ocxo2_arm_nvic_clear_count);
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
  p.add("ocxo1_post_isr_arm_count",       g_ocxo1_post_isr_arm_count);
  p.add("ocxo1_post_isr_drain_count",     g_ocxo1_post_isr_drain_count);
  p.add("ocxo1_post_isr_overwrite_count", g_ocxo1_post_isr_overwrite_count);
  p.add("ocxo1_edge_arm_schedule_count", g_ocxo1_edge_arm_schedule_count);
  p.add("ocxo1_edge_arm_schedule_failures", g_ocxo1_edge_arm_schedule_failures);
  p.add("ocxo1_edge_arm_callback_count", g_ocxo1_edge_arm_callback_count);
  p.add("ocxo1_edge_arm_enable_count", g_ocxo1_edge_arm_enable_count);
  p.add("ocxo1_edge_fire_count", g_ocxo1_edge_fire_count);
  p.add("ocxo1_edge_arm_last_target_counter32", g_ocxo1_edge_arm_last_target_counter32);
  p.add("ocxo1_edge_arm_last_now_counter32", g_ocxo1_edge_arm_last_now_counter32);
  p.add("ocxo1_edge_arm_last_ticks_to_target", g_ocxo1_edge_arm_last_ticks_to_target);
  p.add("ocxo1_edge_arm_last_delay_ns", g_ocxo1_edge_arm_last_delay_ns);
  p.add("ocxo1_edge_arm_rearm_asap_count", g_ocxo1_edge_arm_rearm_asap_count);
  p.add("ocxo2_post_isr_arm_count",       g_ocxo2_post_isr_arm_count);
  p.add("ocxo2_post_isr_drain_count",     g_ocxo2_post_isr_drain_count);
  p.add("ocxo2_post_isr_overwrite_count", g_ocxo2_post_isr_overwrite_count);
  p.add("ocxo2_edge_arm_schedule_count", g_ocxo2_edge_arm_schedule_count);
  p.add("ocxo2_edge_arm_schedule_failures", g_ocxo2_edge_arm_schedule_failures);
  p.add("ocxo2_edge_arm_callback_count", g_ocxo2_edge_arm_callback_count);
  p.add("ocxo2_edge_arm_enable_count", g_ocxo2_edge_arm_enable_count);
  p.add("ocxo2_edge_fire_count", g_ocxo2_edge_fire_count);
  p.add("ocxo2_edge_arm_last_target_counter32", g_ocxo2_edge_arm_last_target_counter32);
  p.add("ocxo2_edge_arm_last_now_counter32", g_ocxo2_edge_arm_last_now_counter32);
  p.add("ocxo2_edge_arm_last_ticks_to_target", g_ocxo2_edge_arm_last_ticks_to_target);
  p.add("ocxo2_edge_arm_last_delay_ns", g_ocxo2_edge_arm_last_delay_ns);
  p.add("ocxo2_edge_arm_rearm_asap_count", g_ocxo2_edge_arm_rearm_asap_count);

  // Patch 1 — advance-iter and short-delay diagnostics.  These are the
  // primary witnesses for the synthetic-clock race hypothesis: a healthy
  // system produces near-zero iter counts and zero short-delay events.
  p.add("ocxo1_edge_arm_advance_iter_count_total", g_ocxo1_edge_arm_advance_iter_count_total);
  p.add("ocxo1_edge_arm_advance_iter_count_max",   g_ocxo1_edge_arm_advance_iter_count_max);
  p.add("ocxo1_edge_arm_advance_iter_count_last",  g_ocxo1_edge_arm_advance_iter_count_last);
  p.add("ocxo1_edge_arm_advance_iter_break_count", g_ocxo1_edge_arm_advance_iter_break_count);
  p.add("ocxo1_edge_arm_short_delay_count",        g_ocxo1_edge_arm_short_delay_count);
  p.add("ocxo1_edge_arm_last_arm_delay_ns_lo",     g_ocxo1_edge_arm_last_arm_delay_ns_lo);
  p.add("ocxo2_edge_arm_advance_iter_count_total", g_ocxo2_edge_arm_advance_iter_count_total);
  p.add("ocxo2_edge_arm_advance_iter_count_max",   g_ocxo2_edge_arm_advance_iter_count_max);
  p.add("ocxo2_edge_arm_advance_iter_count_last",  g_ocxo2_edge_arm_advance_iter_count_last);
  p.add("ocxo2_edge_arm_advance_iter_break_count", g_ocxo2_edge_arm_advance_iter_break_count);
  p.add("ocxo2_edge_arm_short_delay_count",        g_ocxo2_edge_arm_short_delay_count);
  p.add("ocxo2_edge_arm_last_arm_delay_ns_lo",     g_ocxo2_edge_arm_last_arm_delay_ns_lo);

  // Patch 2 — synthetic-clock regression witnesses.  Any non-zero count here
  // is the smoking gun for an unguarded read-modify-write race between
  // CADENCE_MINDER (IRQ context, QTimer1) and a foreground caller of
  // synthetic_clock_tend_from_hw16 on the same clock instance.  Under the
  // Patch 8 priority architecture (QTimer1=32, OCXO=16, PPS=0) these
  // counters should remain at zero forever — BASEPRI-16 critical sections
  // now lawfully mask CADENCE_MINDER, so the race they witnessed is
  // structurally prevented.  A non-zero count after Patch 8 would mean
  // either the priority change did not take effect or there is a different
  // race we have not yet identified.
  p.add("ocxo1_clock32_regression_count",        g_ocxo1_clock32_regression_count);
  p.add("ocxo1_clock32_last_regression_delta",   g_ocxo1_clock32_last_regression_delta);
  p.add("ocxo1_clock32_max_regression_delta",    g_ocxo1_clock32_max_regression_delta);
  p.add("ocxo2_clock32_regression_count",        g_ocxo2_clock32_regression_count);
  p.add("ocxo2_clock32_last_regression_delta",   g_ocxo2_clock32_last_regression_delta);
  p.add("ocxo2_clock32_max_regression_delta",    g_ocxo2_clock32_max_regression_delta);
  p.add("vclock_clock32_regression_count",       g_vclock_clock32_regression_count);
  p.add("vclock_clock32_last_regression_delta",  g_vclock_clock32_last_regression_delta);
  p.add("vclock_clock32_max_regression_delta",   g_vclock_clock32_max_regression_delta);

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
  p.add("ocxo1_cadence_hits_total", g_ocxo1_lane.cadence_hits_total);  // legacy alias; now edge hits
  p.add("ocxo1_edge_hits_total", g_ocxo1_lane.cadence_hits_total);
  p.add("ocxo1_compare_target",     (uint32_t)g_ocxo1_lane.compare_target);
  p.add("ocxo1_next_edge_counter32", g_ocxo1_lane.next_edge_counter32);
  p.add("ocxo1_tick_mod_1000",      g_ocxo1_lane.tick_mod_1000);
  p.add("ocxo1_logical_count32",    g_ocxo1_lane.logical_count32_at_last_second);

  p.add("ocxo2_irq_count",          g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count",         g_ocxo2_lane.miss_count);
  p.add("ocxo2_bootstrap_count",    g_ocxo2_lane.bootstrap_count);
  p.add("ocxo2_cadence_hits_total", g_ocxo2_lane.cadence_hits_total);  // legacy alias; now edge hits
  p.add("ocxo2_edge_hits_total", g_ocxo2_lane.cadence_hits_total);
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