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
#include "timepop.h"
#include "process_timepop.h"

#include <Arduino.h>
#include "imxrt.h"
#include <math.h>
#include <stdio.h>
#include <strings.h>

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

// VCLOCK/relay TimePop heartbeat.
//
// CADENCE_MINDER remains a critical recurring TimePop ISR client on the
// sovereign QTimer1 CH2 rail, but after the lane-local cadence migration it no
// longer owns OCXO rollover custody.  VCLOCK still uses this rail deliberately:
// TimePop's same-deadline coalescing keeps VCLOCK facts on one perishable ISR
// surface and avoids introducing a competing preemptive VCLOCK compare.
static constexpr uint64_t CADENCE_MINDER_PERIOD_NS = 1000000ULL;
static constexpr const char* CADENCE_MINDER_NAME = "CADENCE_MINDER";

// PPS relay ownership lives in process_interrupt because the visible relay edge
// is a physical PPS witness.  The rising edge is asserted directly inside the
// PPS GPIO ISR; the 500 ms deassert is driven by CADENCE_MINDER's existing
// 1 ms heartbeat so relay-off does not allocate/cancel/mutate TimePop slots.
static constexpr uint64_t PPS_RELAY_OFF_NS = 500000000ULL;
static constexpr uint32_t PPS_RELAY_OFF_CADENCE_TICKS =
    (uint32_t)(PPS_RELAY_OFF_NS / CADENCE_MINDER_PERIOD_NS);
static_assert((PPS_RELAY_OFF_NS % CADENCE_MINDER_PERIOD_NS) == 0ULL,
              "PPS relay off interval must be an integer CADENCE_MINDER period");
static_assert(PPS_RELAY_OFF_CADENCE_TICKS == 500U,
              "PPS relay off interval should be 500 cadence ticks");

// OCXO DWT authorship.  OCXO1 and OCXO2 now live on separate QuadTimer
// modules/vectors.  Each OCXO ISR captures ARM_DWT_CYCCNT as its first
// instruction and applies the calibrated QTimer ISR-entry latency correction.
static constexpr uint32_t OCXO_DWT_SOURCE_NONE = 0;
static constexpr uint32_t OCXO_DWT_SOURCE_ISR_ENTRY = 1;

// OCXO lane-local cadence.  OCXO1/OCXO2 now own their own 1 kHz rollover
// custody by programming their local QTimer compare channel to the next
// 10,000-tick target in the lane's own 10 MHz domain.  Every compare ISR
// immediately schedules the next target and captures the perishable service
// facts; foreground ASAP drains those facts.  Every 1000th local cadence fact
// remains the public one-second OCXO event consumed by Alpha.
static constexpr uint32_t OCXO_CADENCE_INTERVAL_TICKS = 10000U;
static constexpr uint32_t OCXO_WITNESS_ONE_SECOND_COUNTS =
    (uint32_t)VCLOCK_COUNTS_PER_SECOND;
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS == 10000000U,
              "OCXO witness edge interval must be one 10 MHz second");
static_assert(OCXO_WITNESS_ONE_SECOND_COUNTS ==
              (OCXO_CADENCE_INTERVAL_TICKS * TICKS_PER_SECOND_EVENT),
              "OCXO one-second event must be exactly 1000 local cadence samples");
static constexpr uint32_t OCXO_WITNESS_ARM_WINDOW_TICKS = OCXO_CADENCE_INTERVAL_TICKS;
static constexpr uint32_t OCXO_WITNESS_MIN_ARM_LEAD_TICKS = 64U;

static constexpr uint32_t OCXO_CADENCE_REASON_NONE          = 0;
static constexpr uint32_t OCXO_CADENCE_REASON_BOOTSTRAP     = 1;
static constexpr uint32_t OCXO_CADENCE_REASON_START         = 2;
static constexpr uint32_t OCXO_CADENCE_REASON_SMARTZERO     = 3;
static constexpr uint32_t OCXO_CADENCE_REASON_LOGICAL_GRID  = 4;
static constexpr uint32_t OCXO_CADENCE_REASON_LOGICAL_ZERO  = 5;
static constexpr uint32_t OCXO_CADENCE_REASON_REARM         = 6;
static constexpr uint32_t OCXO_CADENCE_REASON_STOP          = 7;

// OCXO witness scheduling/report classifications.  These are instrumentation
// only: they make CADENCE_MINDER arm decisions and OCXO compare-service timing
// visible without changing whether an event is published.
static constexpr uint32_t OCXO_SCHEDULE_DECISION_NONE               = 0;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_INACTIVE           = 1;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_ALREADY_ARMED      = 2;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TARGET_INITIALIZED = 3;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TARGET_ADVANCED    = 4;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_TOO_CLOSE          = 5;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW     = 6;
static constexpr uint32_t OCXO_SCHEDULE_DECISION_ARMED              = 7;

static constexpr uint32_t OCXO_SERVICE_CLASS_NONE                   = 0;
static constexpr uint32_t OCXO_SERVICE_CLASS_FALSE_IRQ              = 1;
static constexpr uint32_t OCXO_SERVICE_CLASS_EARLY_PRETARGET        = 2;
static constexpr uint32_t OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET     = 3;

// ============================================================================
// Step 0 migration baseline — report-only safety rails
// ============================================================================
//
// These constants and mirrors do not change interrupt behavior. They simply
// make the current timing architecture and applied NVIC priorities visible in
// INTERRUPT.REPORT before any future migration step touches cadence ownership
// or OCXO event authorship.

static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY  = 0;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY  = 16;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY  = 16;
static constexpr uint32_t INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY = 0;


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

// PPS post-ISR drain.  The GPIO ISR still captures the physical PPS witness
// facts and the small three-counter custody window immediately, because those
// are the timing facts.  Publication of the epoch packet and diagnostic entry
// callback are deferred to a TimePop ASAP slot so the priority-0 PPS ISR stays
// as short as possible.
struct pps_post_isr_mailbox_t {
  volatile bool pending = false;
  volatile uint32_t arm_count = 0;
  volatile uint32_t drain_count = 0;
  volatile uint32_t overwrite_count = 0;
  volatile uint32_t asap_fail_count = 0;
  volatile uint32_t last_sequence = 0;
  volatile uint32_t last_capture_window_cycles = 0;
  interrupt_epoch_capture_t cap{};
  uint32_t isr_entry_dwt_raw = 0;
};

static pps_post_isr_mailbox_t g_pps_post_isr = {};
static void pps_post_isr_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*);

static volatile bool     g_pps_relay_timer_active = false;
static volatile bool     g_pps_relay_deassert_arm_pending = false;
static volatile uint32_t g_pps_relay_deassert_countdown_ticks = 0;
static volatile uint32_t g_pps_relay_assert_count = 0;
static volatile uint32_t g_pps_relay_deassert_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_fail_count = 0;
static volatile uint32_t g_pps_relay_deassert_arm_skip_count = 0;
static volatile uint32_t g_pps_relay_last_assert_sequence = 0;
static volatile bool     g_pps_relay_pin_initialized = false;

static inline void pps_relay_assert_from_isr(uint32_t sequence);
static inline void pps_relay_cadence_minder_tick(void);

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

static volatile uint32_t g_step0_qtimer1_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_qtimer2_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_qtimer3_priority_applied  = 0xFFFFFFFFUL;
static volatile uint32_t g_step0_gpio6789_priority_applied = 0xFFFFFFFFUL;

static volatile bool g_step0_qtimer1_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_qtimer2_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_qtimer3_irq_enabled_by_interrupt  = false;
static volatile bool g_step0_gpio6789_configured_by_interrupt = false;


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

struct synthetic_clock32_t;
struct ocxo_runtime_context_t;

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

  // Lane-local 1 kHz rollover cadence.  This is the device-centric substrate
  // that will later carry the regression window.  The compare target is a
  // process_interrupt-authored synthetic counter32 identity; the low 16 bits
  // are merely the hardware projection used to arm the local QTimer channel.
  bool     cadence_enabled = false;
  bool     cadence_armed = false;
  bool     cadence_epoch_valid = false;
  uint32_t cadence_epoch_counter32 = 0;
  uint32_t cadence_next_counter32 = 0;
  uint16_t cadence_next_low16 = 0;
  uint32_t cadence_last_target_counter32 = 0;
  uint16_t cadence_last_target_low16 = 0;
  uint16_t cadence_last_service_low16 = 0;
  uint32_t cadence_arm_count = 0;
  uint32_t cadence_rearm_count = 0;
  uint32_t cadence_fire_count = 0;
  uint32_t cadence_false_irq_count = 0;
  uint32_t cadence_one_second_due_count = 0;
  uint32_t cadence_last_fire_dwt = 0;
  uint32_t cadence_last_isr_entry_dwt_raw = 0;
  int32_t  cadence_last_service_offset_signed_ticks = 0;
  uint32_t cadence_last_service_offset_abs_ticks = 0;
  uint32_t cadence_last_interpreted_late_ticks = 0;
  uint32_t cadence_last_early_ticks = 0;
  bool     cadence_last_was_early = false;
  bool     cadence_last_one_second_due = false;
  uint32_t cadence_last_program_csctrl_before = 0;
  uint32_t cadence_last_program_csctrl_after = 0;
  bool     cadence_last_program_flag_before = false;
  bool     cadence_last_program_flag_after = false;
  bool     cadence_last_program_enabled_before = false;
  bool     cadence_last_program_enabled_after = false;
  uint32_t cadence_last_reason = OCXO_CADENCE_REASON_NONE;
  uint32_t cadence_user_callback_count = 0;

  // Once-per-second OCXO witness surface.  This is now derived from the 1000th
  // lane-local cadence sample rather than armed as a separate compare target.
  bool     witness_target_initialized = false;
  bool     witness_armed = false;
  uint32_t witness_target_counter32 = 0;
  uint16_t witness_target_low16 = 0;
  uint32_t witness_arm_count = 0;
  uint32_t witness_fire_count = 0;
  uint32_t witness_false_irq_count = 0;
  uint32_t witness_missed_target_count = 0;
  uint32_t witness_late_arm_count = 0;
  uint32_t witness_last_arm_remaining_ticks = 0;
  uint32_t witness_last_event_dwt = 0;
  uint32_t witness_last_event_counter32 = 0;

  // Legacy ambiguous surface retained for continuity.  This is the raw
  // 16-bit modular delta: isr_counter_low16 - target_low16 (mod 65536).
  // Values > 32767 mean early service, not very-late service.
  uint32_t witness_last_late_ticks = 0;

  // CADENCE_MINDER arm-decision diagnostics.
  uint32_t witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_NONE;
  uint32_t witness_schedule_last_current_counter32 = 0;
  uint32_t witness_schedule_last_target_counter32 = 0;
  uint32_t witness_schedule_last_remaining_ticks = 0;
  uint32_t witness_schedule_last_phase_ticks = 0;
  uint32_t witness_schedule_last_ticks_until_arm_window = 0;
  uint16_t witness_schedule_last_current_low16 = 0;
  uint16_t witness_schedule_last_target_low16 = 0;

  // Arming snapshots.
  uint32_t witness_last_arm_dwt_raw = 0;
  uint32_t witness_last_arm_counter32 = 0;
  uint16_t witness_last_arm_low16 = 0;
  uint32_t witness_last_arm_target_counter32 = 0;
  uint16_t witness_last_arm_target_low16 = 0;
  uint32_t witness_last_arm_to_isr_ticks = 0;
  uint32_t witness_last_arm_to_isr_dwt_cycles = 0;
  uint32_t witness_last_program_csctrl_before = 0;
  uint32_t witness_last_program_csctrl_after = 0;
  bool     witness_last_program_flag_before = false;
  bool     witness_last_program_flag_after = false;
  bool     witness_last_program_enabled_before = false;
  bool     witness_last_program_enabled_after = false;

  // Compare-service diagnostics.  These are report-only and do not change
  // publish/compare behavior.
  uint32_t witness_last_isr_csctrl_entry = 0;
  uint32_t witness_last_isr_csctrl_after_disable = 0;
  bool     witness_last_isr_compare_flag_entry = false;
  bool     witness_last_isr_compare_enabled_entry = false;
  bool     witness_last_isr_compare_flag_after_disable = false;
  bool     witness_last_isr_compare_enabled_after_disable = false;
  bool     witness_last_irq_had_armed = false;
  bool     witness_last_irq_had_active_rt = false;

  uint16_t witness_last_target_low16 = 0;
  uint16_t witness_last_isr_counter_low16 = 0;
  uint32_t witness_last_target_delta_mod65536_ticks = 0;
  uint32_t witness_last_interpreted_late_ticks = 0;
  uint32_t witness_last_early_ticks = 0;
  int32_t  witness_last_service_offset_signed_ticks = 0;
  uint32_t witness_last_service_offset_abs_ticks = 0;
  bool     witness_last_service_was_early = false;
  bool     witness_last_service_was_on_or_after_target = false;
  uint32_t witness_last_service_class = OCXO_SERVICE_CLASS_NONE;

  // Deferred perishable-fact diagnostics.  The ISR captures these facts, but
  // foreground ASAP interprets them and updates this report surface.
  uint32_t witness_last_fact_sequence = 0;
  int32_t  witness_last_service_correction_cycles = 0;
  uint32_t witness_last_service_corrected_dwt = 0;
  bool     witness_last_fact_one_second_due = false;
  uint32_t witness_fact_enqueue_count = 0;
  uint32_t witness_fact_drain_count = 0;
  uint32_t witness_fact_overflow_count = 0;
  uint32_t witness_fact_high_water = 0;
  uint32_t witness_fact_asap_arm_count = 0;
  uint32_t witness_fact_asap_fail_count = 0;

  // Canonical one-second custody audit.  A normal OCXO event stream advances
  // exactly 10,000,000 ticks per published edge.
  bool     witness_previous_event_counter32_valid = false;
  uint32_t witness_previous_event_counter32 = 0;
  uint32_t witness_last_counter_delta_ticks = 0;
  uint32_t witness_counter_delta_violation_count = 0;
  uint32_t witness_last_bad_counter_delta = 0;

  uint32_t witness_service_count = 0;
  uint32_t witness_early_service_count = 0;
  uint32_t witness_on_or_after_service_count = 0;
  uint32_t witness_valid_publish_count = 0;
  uint32_t witness_early_service_published_count = 0;
  bool     witness_last_event_published = false;
};
static ocxo_lane_t g_ocxo1_lane;
static ocxo_lane_t g_ocxo2_lane;

static ocxo_lane_t* ocxo_lane_for(interrupt_subscriber_kind_t kind);
static inline uint16_t ocxo_lane_counter_now(const ocxo_lane_t& lane);
static void ocxo_lane_program_compare(ocxo_lane_t& lane, uint16_t target_low16);
static void ocxo_lane_disable_compare(ocxo_lane_t& lane);
static void ocxo_lane_clear_compare_flag(ocxo_lane_t& lane);
static inline bool ocxo_lane_compare_flag_pending(const ocxo_lane_t& lane);
static void ocxo_qtimer_diag_record(ocxo_runtime_context_t& ctx,
                                    uint32_t isr_entry_dwt_raw,
                                    uint32_t event_dwt,
                                    uint32_t legacy_late_ticks,
                                    uint32_t interpreted_late_ticks,
                                    uint32_t early_ticks,
                                    int32_t service_offset_signed_ticks,
                                    uint32_t service_offset_abs_ticks,
                                    uint32_t target_delta_mod65536_ticks,
                                    uint16_t target_low16,
                                    uint16_t isr_counter_low16);
static bool ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t kind,
                                          ocxo_lane_t& lane,
                                          synthetic_clock32_t& clock32,
                                          uint32_t reason);
static bool ocxo_lane_start_local_cadence_at_target(interrupt_subscriber_kind_t kind,
                                                    ocxo_lane_t& lane,
                                                    synthetic_clock32_t& clock32,
                                                    uint32_t target_counter32,
                                                    uint32_t reason);
static void ocxo_lane_stop_local_cadence(ocxo_lane_t& lane, uint32_t reason);
static void ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t kind,
                                           ocxo_lane_t& lane,
                                           synthetic_clock32_t& clock32,
                                           uint32_t epoch_counter32,
                                           uint32_t reason);

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

struct ocxo_qtimer_diag_t {
  volatile uint32_t count = 0;
  volatile uint32_t late_ticks = 0;  // legacy raw modular delta
  volatile uint32_t interpreted_late_ticks = 0;
  volatile uint32_t early_ticks = 0;
  volatile int32_t  service_offset_signed_ticks = 0;
  volatile uint32_t service_offset_abs_ticks = 0;
  volatile uint32_t target_delta_mod65536_ticks = 0;
  volatile uint16_t target_low16 = 0;
  volatile uint16_t isr_counter_low16 = 0;
  volatile uint32_t dwt_raw = 0;
  volatile uint32_t event_dwt = 0;
  volatile uint32_t dwt_coordinate_source = OCXO_DWT_SOURCE_NONE;
};

static ocxo_qtimer_diag_t g_ocxo1_qtimer_diag = {};
static ocxo_qtimer_diag_t g_ocxo2_qtimer_diag = {};

struct ocxo_runtime_context_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane_id = interrupt_lane_t::NONE;
  const char* name = nullptr;
  const char* prefix = nullptr;
  const char* cadence_source = nullptr;
  const char* counter_source = nullptr;
  const char* dwt_authority = nullptr;
  ocxo_lane_t* lane = nullptr;
  synthetic_clock32_t* clock32 = nullptr;
  interrupt_subscriber_runtime_t** rt_slot = nullptr;
  ocxo_qtimer_diag_t* qtimer_diag = nullptr;
};

static ocxo_runtime_context_t g_ocxo1_ctx = {
  interrupt_subscriber_kind_t::OCXO1,
  interrupt_provider_kind_t::QTIMER2,
  interrupt_lane_t::QTIMER2_CH0_COMP,
  "OCXO1",
  "ocxo1",
  "QTIMER2_CH0_LOCAL_1KHZ_COMPARE",
  "QTIMER2_CH0_LOCAL_SYNTHETIC_COUNTER32",
  "QTIMER2_CH0_ISR_ENTRY_DWT",
  &g_ocxo1_lane,
  &g_ocxo1_clock32,
  &g_rt_ocxo1,
  &g_ocxo1_qtimer_diag,
};

static ocxo_runtime_context_t g_ocxo2_ctx = {
  interrupt_subscriber_kind_t::OCXO2,
  interrupt_provider_kind_t::QTIMER3,
  interrupt_lane_t::QTIMER3_CH3_COMP,
  "OCXO2",
  "ocxo2",
  "QTIMER3_CH3_LOCAL_1KHZ_COMPARE",
  "QTIMER3_CH3_LOCAL_SYNTHETIC_COUNTER32",
  "QTIMER3_CH3_ISR_ENTRY_DWT",
  &g_ocxo2_lane,
  &g_ocxo2_clock32,
  &g_rt_ocxo2,
  &g_ocxo2_qtimer_diag,
};

static ocxo_runtime_context_t* ocxo_context_for(interrupt_subscriber_kind_t kind) {
  if (kind == interrupt_subscriber_kind_t::OCXO1) return &g_ocxo1_ctx;
  if (kind == interrupt_subscriber_kind_t::OCXO2) return &g_ocxo2_ctx;
  return nullptr;
}

static interrupt_subscriber_runtime_t* ocxo_runtime_for(const ocxo_runtime_context_t& ctx) {
  return ctx.rt_slot ? *ctx.rt_slot : nullptr;
}

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

static inline uint32_t clock32_from_ns(uint64_t ns) {
  return (uint32_t)((ns / 100ULL) & 0xFFFFFFFFULL);
}

uint32_t interrupt_clock32_from_ns(uint64_t ns) {
  return clock32_from_ns(ns);
}

static synthetic_clock32_t* synthetic_clock_for_kind(interrupt_subscriber_kind_t kind) {
  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  return ctx ? ctx->clock32 : nullptr;
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
    ocxo_lane_install_logical_grid(kind, *lane, *c, c->zero_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_ZERO);
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

void interrupt_ocxo_logical_grid_epoch(uint32_t ocxo1_epoch_counter32,
                                       uint32_t ocxo2_epoch_counter32) {
  if (g_ocxo1_lane.initialized) {
    ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t::OCXO1,
                                   g_ocxo1_lane,
                                   g_ocxo1_clock32,
                                   ocxo1_epoch_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_GRID);
  }

  if (g_ocxo2_lane.initialized) {
    ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t::OCXO2,
                                   g_ocxo2_lane,
                                   g_ocxo2_clock32,
                                   ocxo2_epoch_counter32,
                                   OCXO_CADENCE_REASON_LOGICAL_GRID);
  }
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
// SmartZero(R) — mathematically-qualified logical zero acquisition
// ============================================================================
//
// The old epoch packet trusted a latency-adjusted ISR capture. SmartZero does
// not.  It observes consecutive 1 kHz edge captures and accepts a lane only
// when the DWT interval matches the current PPS/GPIO-derived cycles-per-second
// ruler within a tight cycle window.  The later edge of the accepted pair is
// the lane's canonical zero anchor.  Acquisition is strictly serial:
// VCLOCK -> OCXO1 -> OCXO2.

static constexpr uint32_t SMARTZERO_INTERVAL_TICKS = SMARTZERO_COUNTER_DELTA_TICKS;
static constexpr int32_t  SMARTZERO_INTERVAL_TOLERANCE_CYCLES =
    SMARTZERO_DEFAULT_TOLERANCE_CYCLES;

struct smartzero_lane_runtime_t {
  interrupt_smartzero_lane_snapshot_t pub{};
  bool previous_present = false;
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
  smartzero_lane_runtime_t lanes[SMARTZERO_LANE_COUNT];
};

static smartzero_runtime_t g_smartzero = {};

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
  if (kind == interrupt_subscriber_kind_t::OCXO1)  return 1;
  if (kind == interrupt_subscriber_kind_t::OCXO2)  return 2;
  return -1;
}

static const char* smartzero_decision_name(interrupt_smartzero_decision_t d) {
  switch (d) {
    case interrupt_smartzero_decision_t::WAITING_FOR_CPS:  return "WAITING_FOR_CPS";
    case interrupt_smartzero_decision_t::FIRST_SAMPLE:     return "FIRST_SAMPLE";
    case interrupt_smartzero_decision_t::ACCEPTED:         return "ACCEPTED";
    case interrupt_smartzero_decision_t::REJECTED_DWT:     return "REJECTED_DWT";
    case interrupt_smartzero_decision_t::REJECTED_COUNTER: return "REJECTED_COUNTER";
    default:                                               return "NONE";
  }
}

static const char* smartzero_lane_state_name(interrupt_smartzero_lane_state_t s) {
  switch (s) {
    case interrupt_smartzero_lane_state_t::ACQUIRING: return "ACQUIRING";
    case interrupt_smartzero_lane_state_t::LOCKED:    return "LOCKED";
    default:                                         return "IDLE";
  }
}

static const char* smartzero_phase_name(interrupt_smartzero_phase_t p) {
  switch (p) {
    case interrupt_smartzero_phase_t::RUNNING:  return "RUNNING";
    case interrupt_smartzero_phase_t::COMPLETE: return "COMPLETE";
    case interrupt_smartzero_phase_t::ABORTED:  return "ABORTED";
    default:                                    return "IDLE";
  }
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
  smartzero_lane_runtime_t& r = g_smartzero.lanes[index];
  r = smartzero_lane_runtime_t{};
  r.pub.kind = smartzero_kind_for_index(index);
  r.pub.state = interrupt_smartzero_lane_state_t::IDLE;
  r.pub.last_decision = interrupt_smartzero_decision_t::NONE;
  r.pub.tolerance_cycles = SMARTZERO_INTERVAL_TOLERANCE_CYCLES;
  r.pub.required_counter_delta_ticks = SMARTZERO_INTERVAL_TICKS;
}

static bool smartzero_is_current_lane(interrupt_subscriber_kind_t kind) {
  if (!g_smartzero.running || g_smartzero.complete) return false;
  return smartzero_kind_for_index(g_smartzero.current_lane_index) == kind;
}

static uint32_t smartzero_expected_interval_cycles(uint32_t* out_cps) {
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  if (out_cps) *out_cps = cps;

  // SmartZero's admissibility ruler is intentionally PPS/GPIO-derived.  The
  // fallback DWT_EXPECTED_PER_PPS is close, but ZERO authority should wait for
  // the physical PPS witness to give us a live one-second ruler.
  if (!clocks_dwt_calibration_valid() || cps == 0) return 0;

  return (uint32_t)(((uint64_t)cps +
                     (uint64_t)SMARTZERO_SAMPLE_RATE_HZ / 2ULL) /
                    (uint64_t)SMARTZERO_SAMPLE_RATE_HZ);
}

static void smartzero_arm_ocxo_lane(interrupt_subscriber_kind_t kind) {
  const int idx = smartzero_index_for_kind(kind);
  if (idx < 0) return;

  ocxo_lane_t* lane = ocxo_lane_for(kind);
  synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
  if (!lane || !clock32 || !lane->initialized) return;

  // SmartZero now rides the same lane-local 1 kHz compare cadence that will
  // later feed regression.  The compare target is the authored sample
  // identity; service-time low-word reads remain diagnostics only.
  ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_SMARTZERO);

  const uint16_t hw16 = ocxo_lane_counter_now(*lane);
  synthetic_clock_tend_from_hw16(*clock32, hw16);
  const uint32_t target_counter32 =
      clock32->current_counter32 + SMARTZERO_INTERVAL_TICKS;

  smartzero_lane_runtime_t& r = g_smartzero.lanes[idx];
  r.pub.next_target_counter32 = target_counter32;
  r.pub.arm_count++;

  (void)ocxo_lane_start_local_cadence_at_target(kind, *lane, *clock32,
                                                target_counter32,
                                                OCXO_CADENCE_REASON_SMARTZERO);
}

static void smartzero_arm_current_lane(void) {
  const interrupt_subscriber_kind_t kind =
      smartzero_kind_for_index(g_smartzero.current_lane_index);
  if (kind == interrupt_subscriber_kind_t::OCXO1 ||
      kind == interrupt_subscriber_kind_t::OCXO2) {
    smartzero_arm_ocxo_lane(kind);
  }
}

static void smartzero_advance_or_complete(void) {
  if (g_smartzero.current_lane_index + 1U >= SMARTZERO_LANE_COUNT) {
    g_smartzero.running = false;
    g_smartzero.complete = true;
    g_smartzero.phase = interrupt_smartzero_phase_t::COMPLETE;
    g_smartzero.complete_count++;
    return;
  }

  g_smartzero.current_lane_index++;
  smartzero_lane_runtime_t& next = g_smartzero.lanes[g_smartzero.current_lane_index];
  next.pub.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  next.pub.last_decision = interrupt_smartzero_decision_t::NONE;
  next.previous_present = false;
  smartzero_arm_current_lane();
}

static void smartzero_feed_sample(interrupt_subscriber_kind_t kind,
                                  uint32_t event_dwt,
                                  uint32_t counter32,
                                  uint16_t hardware16) {
  if (!smartzero_is_current_lane(kind)) return;

  const int idx = smartzero_index_for_kind(kind);
  if (idx < 0) return;

  uint32_t cps = 0;
  const uint32_t expected = smartzero_expected_interval_cycles(&cps);

  smartzero_write_begin();

  smartzero_lane_runtime_t& r = g_smartzero.lanes[idx];
  interrupt_smartzero_lane_snapshot_t& z = r.pub;

  z.state = interrupt_smartzero_lane_state_t::ACQUIRING;
  z.sample_count++;
  z.cps_used = cps;
  z.expected_interval_cycles = expected;
  z.last_sample_dwt = event_dwt;
  z.last_sample_counter32 = counter32;
  z.last_sample_hardware16 = hardware16;

  if (expected == 0) {
    z.waiting_for_cps_count++;
    z.last_decision = interrupt_smartzero_decision_t::WAITING_FOR_CPS;
    r.previous_present = false;
    smartzero_write_end();
    return;
  }

  if (!r.previous_present) {
    z.previous_sample_dwt = event_dwt;
    z.previous_sample_counter32 = counter32;
    z.previous_sample_hardware16 = hardware16;
    z.last_decision = interrupt_smartzero_decision_t::FIRST_SAMPLE;
    r.previous_present = true;
    smartzero_write_end();
    return;
  }

  const uint32_t previous_dwt = z.previous_sample_dwt;
  const uint32_t previous_counter32 = z.previous_sample_counter32;
  const uint16_t previous_hw16 = z.previous_sample_hardware16;

  const uint32_t interval_cycles = event_dwt - previous_dwt;
  const uint32_t counter_delta = counter32 - previous_counter32;
  const int32_t error_cycles =
      (int32_t)((int64_t)interval_cycles - (int64_t)expected);
  const uint32_t abs_error = (uint32_t)(error_cycles < 0
      ? -(int64_t)error_cycles
      :  (int64_t)error_cycles);

  z.interval_attempt_count++;
  z.last_interval_cycles = interval_cycles;
  z.last_interval_error_cycles = error_cycles;
  z.last_counter_delta_ticks = counter_delta;
  if (abs_error > z.max_abs_interval_error_cycles) {
    z.max_abs_interval_error_cycles = abs_error;
  }

  const bool counter_ok = (counter_delta == SMARTZERO_INTERVAL_TICKS);
  const bool dwt_ok = ((int32_t)abs_error <= SMARTZERO_INTERVAL_TOLERANCE_CYCLES);

  if (counter_ok && dwt_ok) {
    z.accepted_count++;
    z.state = interrupt_smartzero_lane_state_t::LOCKED;
    z.last_decision = interrupt_smartzero_decision_t::ACCEPTED;
    z.anchor_dwt = event_dwt;
    z.anchor_counter32 = counter32;
    z.anchor_hardware16 = hardware16;
    z.anchor_pair_previous_dwt = previous_dwt;
    z.anchor_pair_previous_counter32 = previous_counter32;

    if (kind == interrupt_subscriber_kind_t::OCXO1 ||
        kind == interrupt_subscriber_kind_t::OCXO2) {
      ocxo_lane_t* lane = ocxo_lane_for(kind);
      synthetic_clock32_t* clock32 = synthetic_clock_for_kind(kind);
      if (lane && clock32) {
        ocxo_lane_install_logical_grid(kind, *lane, *clock32, counter32,
                                       OCXO_CADENCE_REASON_SMARTZERO);
        if (!lane->active) {
          ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_SMARTZERO);
        }
      }
    }

    smartzero_advance_or_complete();
    smartzero_write_end();
    return;
  }

  z.rejected_count++;
  z.last_decision = counter_ok
      ? interrupt_smartzero_decision_t::REJECTED_DWT
      : interrupt_smartzero_decision_t::REJECTED_COUNTER;

  // Roll the observation window forward.  If this sample was the bad one, the
  // next interval will reject; if it was good, the next good edge can lock.
  z.previous_sample_dwt = event_dwt;
  z.previous_sample_counter32 = counter32;
  z.previous_sample_hardware16 = hardware16;
  (void)previous_hw16;

  smartzero_write_end();
}

// OCXO SmartZero samples are now served by the same lane-local 1 kHz
// compare cadence used for normal rollover custody.  There is intentionally
// no separate SmartZero ISR path here; the unified OCXO cadence ISR feeds
// smartzero_feed_sample() with the authored compare-target identity.

bool interrupt_smartzero_begin(void) {
  if (!g_interrupt_runtime_ready || !g_interrupt_hw_ready) return false;

  smartzero_write_begin();

  g_smartzero.running = true;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::RUNNING;
  g_smartzero.sequence++;
  g_smartzero.begin_count++;
  g_smartzero.current_lane_index = 0;
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }
  g_smartzero.lanes[0].pub.state = interrupt_smartzero_lane_state_t::ACQUIRING;

  // SmartZero takes temporary custody of the OCXO local compare cadence only
  // when each OCXO lane becomes current.  Stop any running OCXO cadence now so
  // the acquisition sequence can re-author clean +10,000-tick sample ladders.
  ocxo_lane_stop_local_cadence(g_ocxo1_lane, OCXO_CADENCE_REASON_SMARTZERO);
  ocxo_lane_stop_local_cadence(g_ocxo2_lane, OCXO_CADENCE_REASON_SMARTZERO);

  smartzero_write_end();
  return true;
}

void interrupt_smartzero_abort(void) {
  smartzero_write_begin();
  g_smartzero.running = false;
  g_smartzero.complete = false;
  g_smartzero.phase = interrupt_smartzero_phase_t::ABORTED;
  g_smartzero.abort_count++;
  g_smartzero.current_lane_index = SMARTZERO_LANE_COUNT;

  // Live acquisition state is not the installed proof.  When a live attempt
  // is aborted, clear its lane proof surface so reports do not show stale
  // LOCKED anchors under smartzero_complete=false.  CLOCKS/Alpha retains the
  // installed SmartZero proof separately.
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }

  ocxo_lane_stop_local_cadence(g_ocxo1_lane, OCXO_CADENCE_REASON_STOP);
  ocxo_lane_stop_local_cadence(g_ocxo2_lane, OCXO_CADENCE_REASON_STOP);
  smartzero_write_end();
}

bool interrupt_smartzero_running(void) {
  return g_smartzero.running && !g_smartzero.complete;
}

bool interrupt_smartzero_complete(void) {
  return g_smartzero.complete;
}

bool interrupt_smartzero_live_snapshot(interrupt_smartzero_snapshot_t* out) {
  if (!out) return false;

  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t seq1 = g_smartzero.seq;
    dmb_barrier();

    interrupt_smartzero_snapshot_t local{};
    local.phase = g_smartzero.phase;
    local.running = g_smartzero.running;
    local.complete = g_smartzero.complete;
    local.aborted = (g_smartzero.phase == interrupt_smartzero_phase_t::ABORTED);
    local.sequence = g_smartzero.sequence;
    local.begin_count = g_smartzero.begin_count;
    local.complete_count = g_smartzero.complete_count;
    local.abort_count = g_smartzero.abort_count;
    local.current_lane_index = g_smartzero.current_lane_index;
    local.current_lane = g_smartzero.running
        ? smartzero_kind_for_index(g_smartzero.current_lane_index)
        : interrupt_subscriber_kind_t::NONE;
    local.tolerance_cycles = SMARTZERO_INTERVAL_TOLERANCE_CYCLES;
    local.sample_rate_hz = SMARTZERO_SAMPLE_RATE_HZ;
    local.counter_delta_ticks = SMARTZERO_INTERVAL_TICKS;
    for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
      local.lanes[i] = g_smartzero.lanes[i].pub;
    }

    dmb_barrier();
    const uint32_t seq2 = g_smartzero.seq;
    if (seq1 == seq2 && (seq1 & 1u) == 0u) {
      *out = local;
      return local.complete;
    }
  }

  *out = interrupt_smartzero_snapshot_t{};
  return false;
}

bool interrupt_smartzero_snapshot(interrupt_smartzero_snapshot_t* out) {
  return interrupt_smartzero_live_snapshot(out);
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

// QuadTimer compare flags are cleared by writing the flag bits low in CSCTRL.
// Match the QTimer1 CH1/CH2 clear idiom.
static inline void qtimer2_ch0_clear_compare_flag(void) {
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

// QuadTimer compare flags are cleared by writing the flag bits low in CSCTRL.
// Match the QTimer1 CH1/CH2 clear idiom.
static inline void qtimer3_clear_compare_flag(uint8_t ch) {
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
  ocxo_runtime_context_t* ctx = ocxo_context_for(kind);
  return ctx ? ctx->lane : nullptr;
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

  if (rt.desc->kind == interrupt_subscriber_kind_t::OCXO1 ||
      rt.desc->kind == interrupt_subscriber_kind_t::OCXO2) {
    const ocxo_lane_t* lane = ocxo_lane_for(rt.desc->kind);
    if (lane) {
      diag.ocxo_service_class = lane->witness_last_service_class;
      diag.ocxo_service_offset_signed_ticks =
          lane->witness_last_service_offset_signed_ticks;
      diag.ocxo_service_offset_abs_ticks =
          lane->witness_last_service_offset_abs_ticks;
      diag.ocxo_interpreted_late_ticks =
          lane->witness_last_interpreted_late_ticks;
      diag.ocxo_early_ticks = lane->witness_last_early_ticks;
      diag.ocxo_target_delta_mod65536_ticks =
          lane->witness_last_target_delta_mod65536_ticks;
      diag.ocxo_arm_remaining_ticks = lane->witness_last_arm_remaining_ticks;
      diag.ocxo_arm_to_isr_ticks = lane->witness_last_arm_to_isr_ticks;
      diag.ocxo_arm_to_isr_dwt_cycles = lane->witness_last_arm_to_isr_dwt_cycles;
      diag.ocxo_perishable_fact_sequence = lane->witness_last_fact_sequence;
      diag.ocxo_service_correction_cycles =
          lane->witness_last_service_correction_cycles;
      diag.ocxo_service_corrected_dwt_at_event =
          lane->witness_last_service_corrected_dwt;
      diag.ocxo_fact_ring_overflow_count = lane->witness_fact_overflow_count;
      diag.ocxo_counter_delta_violation_count =
          lane->witness_counter_delta_violation_count;
      diag.ocxo_last_bad_counter_delta = lane->witness_last_bad_counter_delta;
      diag.ocxo_last_counter_delta_ticks = lane->witness_last_counter_delta_ticks;
    }
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
// Cadence surfaces — VCLOCK TimePop heartbeat + OCXO lane-local compare custody
// ============================================================================

static const char* ocxo_schedule_decision_name(uint32_t decision) {
  switch (decision) {
    case OCXO_SCHEDULE_DECISION_INACTIVE: return "INACTIVE";
    case OCXO_SCHEDULE_DECISION_ALREADY_ARMED: return "ALREADY_ARMED";
    case OCXO_SCHEDULE_DECISION_TARGET_INITIALIZED: return "TARGET_INITIALIZED";
    case OCXO_SCHEDULE_DECISION_TARGET_ADVANCED: return "TARGET_ADVANCED";
    case OCXO_SCHEDULE_DECISION_TOO_CLOSE: return "TOO_CLOSE";
    case OCXO_SCHEDULE_DECISION_OUTSIDE_WINDOW: return "OUTSIDE_WINDOW";
    case OCXO_SCHEDULE_DECISION_ARMED: return "ARMED";
    default: return "NONE";
  }
}

static const char* ocxo_service_class_name(uint32_t service_class) {
  switch (service_class) {
    case OCXO_SERVICE_CLASS_FALSE_IRQ: return "FALSE_IRQ";
    case OCXO_SERVICE_CLASS_EARLY_PRETARGET: return "EARLY_PRETARGET";
    case OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET: return "ON_OR_AFTER_TARGET";
    default: return "NONE";
  }
}

static const char* ocxo_cadence_reason_name(uint32_t reason) {
  switch (reason) {
    case OCXO_CADENCE_REASON_BOOTSTRAP:    return "BOOTSTRAP";
    case OCXO_CADENCE_REASON_START:        return "START";
    case OCXO_CADENCE_REASON_SMARTZERO:    return "SMARTZERO";
    case OCXO_CADENCE_REASON_LOGICAL_GRID: return "LOGICAL_GRID";
    case OCXO_CADENCE_REASON_LOGICAL_ZERO: return "LOGICAL_ZERO";
    case OCXO_CADENCE_REASON_REARM:        return "REARM";
    case OCXO_CADENCE_REASON_STOP:         return "STOP";
    default:                               return "NONE";
  }
}

static uint32_t ocxo_grid_next_target_after(uint32_t epoch_counter32,
                                            uint32_t current_counter32) {
  const uint32_t delta = current_counter32 - epoch_counter32;

  // If the epoch appears to be in the modular future, use the first cadence
  // target after the epoch.  This can only happen across a 32-bit wrap or when
  // a caller deliberately re-authors a future logical grid.
  if (delta > 0x7FFFFFFFUL) {
    return epoch_counter32 + OCXO_CADENCE_INTERVAL_TICKS;
  }

  const uint32_t steps_completed = delta / OCXO_CADENCE_INTERVAL_TICKS;
  return epoch_counter32 + ((steps_completed + 1U) * OCXO_CADENCE_INTERVAL_TICKS);
}

static void ocxo_lane_program_local_cadence_compare(ocxo_lane_t& lane,
                                                    uint32_t target_counter32,
                                                    uint32_t reason) {
  const uint16_t target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);

  lane.cadence_last_program_csctrl_before = lane.module->CH[lane.channel].CSCTRL;
  lane.cadence_last_program_flag_before =
      (lane.cadence_last_program_csctrl_before & TMR_CSCTRL_TCF1) != 0;
  lane.cadence_last_program_enabled_before =
      (lane.cadence_last_program_csctrl_before & TMR_CSCTRL_TCF1EN) != 0;

  lane.cadence_next_counter32 = target_counter32;
  lane.cadence_next_low16 = target_low16;
  lane.compare_target = target_low16;
  lane.witness_armed = true;
  lane.witness_target_initialized = true;
  lane.witness_target_counter32 = target_counter32;
  lane.witness_target_low16 = target_low16;

  ocxo_lane_program_compare(lane, target_low16);

  lane.cadence_last_program_csctrl_after = lane.module->CH[lane.channel].CSCTRL;
  lane.cadence_last_program_flag_after =
      (lane.cadence_last_program_csctrl_after & TMR_CSCTRL_TCF1) != 0;
  lane.cadence_last_program_enabled_after =
      (lane.cadence_last_program_csctrl_after & TMR_CSCTRL_TCF1EN) != 0;
  lane.cadence_armed = true;
  lane.cadence_arm_count++;
  if (reason == OCXO_CADENCE_REASON_REARM) {
    lane.cadence_rearm_count++;
  }
  lane.cadence_last_reason = reason;
}

static bool ocxo_lane_start_local_cadence_at_target(interrupt_subscriber_kind_t,
                                                    ocxo_lane_t& lane,
                                                    synthetic_clock32_t& clock32,
                                                    uint32_t target_counter32,
                                                    uint32_t reason) {
  if (!lane.initialized) return false;

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);

  lane.cadence_enabled = true;
  lane.cadence_last_reason = reason;
  lane.witness_last_arm_dwt_raw = ARM_DWT_CYCCNT;
  lane.witness_last_arm_counter32 = clock32.current_counter32;
  lane.witness_last_arm_low16 = hw16;
  lane.witness_last_arm_target_counter32 = target_counter32;
  lane.witness_last_arm_target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);
  lane.witness_last_arm_remaining_ticks = target_counter32 - clock32.current_counter32;
  lane.witness_schedule_last_decision = OCXO_SCHEDULE_DECISION_ARMED;
  lane.witness_schedule_last_current_counter32 = clock32.current_counter32;
  lane.witness_schedule_last_target_counter32 = target_counter32;
  lane.witness_schedule_last_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  lane.witness_schedule_last_current_low16 = hw16;
  lane.witness_schedule_last_target_low16 = (uint16_t)(target_counter32 & 0xFFFFU);
  lane.witness_schedule_last_phase_ticks =
      lane.cadence_epoch_valid ? (clock32.current_counter32 - lane.cadence_epoch_counter32) : 0U;
  lane.witness_schedule_last_ticks_until_arm_window = 0;

  ocxo_lane_program_local_cadence_compare(lane, target_counter32, reason);
  return true;
}

static bool ocxo_lane_start_local_cadence(interrupt_subscriber_kind_t kind,
                                          ocxo_lane_t& lane,
                                          synthetic_clock32_t& clock32,
                                          uint32_t reason) {
  if (!lane.initialized) return false;

  const uint16_t hw16 = ocxo_lane_counter_now(lane);
  synthetic_clock_tend_from_hw16(clock32, hw16);

  const uint32_t target = lane.cadence_epoch_valid
      ? ocxo_grid_next_target_after(lane.cadence_epoch_counter32,
                                    clock32.current_counter32)
      : (clock32.current_counter32 + OCXO_CADENCE_INTERVAL_TICKS);

  if (!lane.phase_bootstrapped) {
    lane.phase_bootstrapped = true;
    lane.bootstrap_count++;
  }
  lane.tick_mod_1000 = lane.cadence_epoch_valid
      ? ((target - lane.cadence_epoch_counter32) / OCXO_CADENCE_INTERVAL_TICKS - 1U) %
            TICKS_PER_SECOND_EVENT
      : 0U;

  return ocxo_lane_start_local_cadence_at_target(kind, lane, clock32, target, reason);
}

static void ocxo_lane_stop_local_cadence(ocxo_lane_t& lane, uint32_t reason) {
  lane.cadence_enabled = false;
  lane.cadence_armed = false;
  lane.witness_armed = false;
  lane.cadence_last_reason = reason;
  ocxo_lane_disable_compare(lane);
}

static void ocxo_lane_install_logical_grid(interrupt_subscriber_kind_t kind,
                                           ocxo_lane_t& lane,
                                           synthetic_clock32_t& clock32,
                                           uint32_t epoch_counter32,
                                           uint32_t reason) {
  lane.cadence_epoch_valid = true;
  lane.cadence_epoch_counter32 = epoch_counter32;
  lane.tick_mod_1000 = 0;
  lane.witness_previous_event_counter32_valid = false;
  lane.witness_previous_event_counter32 = 0;
  lane.witness_last_counter_delta_ticks = 0;

  if (lane.active || smartzero_is_current_lane(kind)) {
    (void)ocxo_lane_start_local_cadence(kind, lane, clock32, reason);
  }
}

// The old CADENCE_MINDER-driven OCXO arming path has been retired.  OCXO1 and
// OCXO2 now maintain their own 1 kHz compare cadence on their local QTimer
// channels; CADENCE_MINDER remains only the VCLOCK/relay TimePop heartbeat.

static void cadence_minder_timepop_callback(timepop_ctx_t* ctx,
                                            timepop_diag_t*,
                                            void*) {
  if (!ctx || !g_interrupt_hw_ready) return;

  const uint32_t qtimer_event_dwt = ctx->fire_dwt_cyccnt;
  const uint32_t cadence_counter32 = ctx->fire_vclock_raw;
  const uint16_t fired_low16 = (uint16_t)(cadence_counter32 & 0xFFFFU);

  g_cadence_minder_fire_count++;
  pps_relay_cadence_minder_tick();

  // VCLOCK: CADENCE_MINDER is now the only 1 ms substrate cadence.  It owns
  // the synthetic VCLOCK low-word anchor refresh and the former separate
  // VCLOCK one-second/bookend duties.
  vclock_clock_anchor_hardware_low16(cadence_counter32, fired_low16);
  g_vclock_clock32.minder_update_count++;
  g_vclock_lane.logical_count32_at_last_second = cadence_counter32;
  g_cadence_minder_vclock_updates++;
  g_cadence_minder_last_vclock_hw16 = fired_low16;
  g_cadence_minder_last_vclock_counter32 = cadence_counter32;

  // SmartZero VCLOCK samples are the same 1 kHz TimePop fire facts that will
  // later feed regression.  During zero acquisition they are observations; the
  // accepted pair decides the canonical VCLOCK zero anchor.
  smartzero_feed_sample(interrupt_subscriber_kind_t::VCLOCK,
                        qtimer_event_dwt,
                        cadence_counter32,
                        fired_low16);

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

  // OCXO lanes are intentionally absent here.  Their 16-bit rollover cadence is
  // now device-local on QTimer2/QTimer3; this TimePop callback is the VCLOCK
  // cadence surface plus the PPS relay-off heartbeat.
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
static void ocxo_lane_program_compare(ocxo_lane_t& lane, uint16_t target_low16) {
  if (lane.module == &IMXRT_TMR2 && lane.channel == 0) {
    qtimer2_ch0_program_compare(target_low16);
  } else if (lane.module == &IMXRT_TMR3) {
    qtimer3_program_compare(lane.channel, target_low16);
  }
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
// Standardized OCXO perishable-fact ring — ISR capture, ASAP interpretation.
// ----------------------------------------------------------------------------
//
// This is the conservative first step toward the universal interrupt capture
// pipeline.  The ISR captures only facts that would be lost if delayed:
// first-instruction DWT, authored compare target identity, service-time
// hardware low word, service offset, compare flag state, and minimal class.
// Foreground ASAP drains the ring, updates diagnostics, audits custody, and
// dispatches the one-second event to CLOCKS/Alpha.
//
// The ring is intentionally per-lane and loss-visible.  The cadence rail is
// now 1 kHz, so the buffer must absorb short foreground/ASAP stalls without
// ever silently overwriting a perishable sample.  Size 32 spans 32 ms at the
// new cadence while remaining small enough for deterministic ISR writes.

static constexpr uint32_t OCXO_PERISHABLE_FACT_RING_SIZE = 32;

struct interrupt_perishable_fact_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  uint32_t sequence = 0;

  // Captured as first ISR instruction.  This remains private custody evidence;
  // published event DWT remains latency-adjusted service_dwt_at_event.
  uint32_t isr_entry_dwt_raw = 0;
  uint32_t service_dwt_at_event = 0;

  // Authored target identity.  This is the event identity, not an ambient read.
  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;

  // Hardware observation at ISR service time.
  uint16_t service_counter_low16 = 0;
  int16_t service_offset_ticks = 0;
  uint32_t service_offset_abs_ticks = 0;
  uint32_t interpreted_late_ticks = 0;
  uint32_t early_ticks = 0;
  uint32_t target_delta_mod65536_ticks = 0;

  // Arming/service forensics captured before interpretation.
  uint32_t arm_remaining_ticks = 0;
  uint32_t arm_to_isr_ticks = 0;
  uint32_t arm_to_isr_dwt_cycles = 0;
  uint32_t csctrl_entry = 0;
  uint32_t csctrl_after_disable = 0;
  bool compare_flag_seen = false;
  bool compare_enabled_entry = false;
  bool compare_flag_after_disable = false;
  bool compare_enabled_after_disable = false;
  bool had_armed = false;
  bool had_active_rt = false;

  uint32_t service_class = OCXO_SERVICE_CLASS_NONE;
  bool one_second_due = false;
  bool exact_target_identity = true;
};

struct ocxo_perishable_ring_t {
  interrupt_perishable_fact_t facts[OCXO_PERISHABLE_FACT_RING_SIZE]{};
  volatile uint32_t head = 0;
  volatile uint32_t tail = 0;
  volatile uint32_t count = 0;
  volatile bool drain_armed = false;
  volatile uint32_t sequence = 0;
  volatile uint32_t enqueue_count = 0;
  volatile uint32_t drain_count = 0;
  volatile uint32_t overflow_count = 0;
  volatile uint32_t high_water = 0;
  volatile uint32_t asap_arm_count = 0;
  volatile uint32_t asap_fail_count = 0;
};

static ocxo_perishable_ring_t g_ocxo1_fact_ring = {};
static ocxo_perishable_ring_t g_ocxo2_fact_ring = {};

static ocxo_perishable_ring_t&
ocxo_fact_ring_for(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_fact_ring
      : g_ocxo2_fact_ring;
}

static const char* ocxo_fact_drain_name(interrupt_subscriber_kind_t kind) {
  return (kind == interrupt_subscriber_kind_t::OCXO1)
      ? "OCXO1_FACT_DRAIN"
      : "OCXO2_FACT_DRAIN";
}

static void ocxo_perishable_fact_ring_reset(ocxo_perishable_ring_t& r) {
  for (uint32_t i = 0; i < OCXO_PERISHABLE_FACT_RING_SIZE; i++) {
    r.facts[i] = interrupt_perishable_fact_t{};
  }
  r.head = 0;
  r.tail = 0;
  r.count = 0;
  r.drain_armed = false;
  r.sequence = 0;
  r.enqueue_count = 0;
  r.drain_count = 0;
  r.overflow_count = 0;
  r.high_water = 0;
  r.asap_arm_count = 0;
  r.asap_fail_count = 0;
}

static int32_t dwt_cycles_for_ocxo_ticks_signed(int32_t ticks) {
  const uint32_t cps = interrupt_vclock_cycles_per_second();
  if (cps == 0 || ticks == 0) return 0;

  const int64_t numerator = (int64_t)ticks * (int64_t)cps;
  const int64_t denom = (int64_t)OCXO_WITNESS_ONE_SECOND_COUNTS;
  if (numerator >= 0) {
    return (int32_t)((numerator + denom / 2) / denom);
  }
  return (int32_t)(-(((-numerator) + denom / 2) / denom));
}

static bool ocxo_fact_ring_push_from_isr(interrupt_subscriber_kind_t kind,
                                         const interrupt_perishable_fact_t& fact,
                                         interrupt_subscriber_runtime_t* rt);
static void ocxo_fact_drain_callback(timepop_ctx_t*, timepop_diag_t*, void* user_data);

static bool ocxo_fact_ring_pop(interrupt_subscriber_kind_t kind,
                               interrupt_perishable_fact_t& out) {
  ocxo_perishable_ring_t& r = ocxo_fact_ring_for(kind);

  __disable_irq();
  if (r.count == 0) {
    r.drain_armed = false;
    __enable_irq();
    return false;
  }

  out = r.facts[r.tail];
  r.tail = (r.tail + 1U) % OCXO_PERISHABLE_FACT_RING_SIZE;
  r.count--;
  r.drain_count++;
  if (r.count == 0) {
    r.drain_armed = false;
  }
  __enable_irq();
  return true;
}

static bool ocxo_fact_ring_push_from_isr(interrupt_subscriber_kind_t kind,
                                         const interrupt_perishable_fact_t& fact,
                                         interrupt_subscriber_runtime_t* rt) {
  ocxo_perishable_ring_t& r = ocxo_fact_ring_for(kind);
  ocxo_lane_t* lane = ocxo_lane_for(kind);

  if (r.count >= OCXO_PERISHABLE_FACT_RING_SIZE) {
    r.overflow_count++;
    if (lane) {
      lane->witness_fact_overflow_count = r.overflow_count;
    }
    return false;
  }

  r.facts[r.head] = fact;
  r.head = (r.head + 1U) % OCXO_PERISHABLE_FACT_RING_SIZE;
  r.count++;
  r.enqueue_count++;
  if (r.count > r.high_water) r.high_water = r.count;

  if (lane) {
    lane->witness_fact_enqueue_count = r.enqueue_count;
    lane->witness_fact_overflow_count = r.overflow_count;
    lane->witness_fact_high_water = r.high_water;
  }

  if (!r.drain_armed) {
    r.drain_armed = true;
    r.asap_arm_count++;
    if (lane) lane->witness_fact_asap_arm_count = r.asap_arm_count;

    const timepop_handle_t h =
        timepop_arm_asap(ocxo_fact_drain_callback, rt, ocxo_fact_drain_name(kind));
    if (h == TIMEPOP_INVALID_HANDLE) {
      r.drain_armed = false;
      r.asap_fail_count++;
      if (lane) lane->witness_fact_asap_fail_count = r.asap_fail_count;
      return false;
    }
  }

  return true;
}

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

static void ocxo_qtimer_diag_record(ocxo_runtime_context_t& ctx,
                                    uint32_t isr_entry_dwt_raw,
                                    uint32_t event_dwt,
                                    uint32_t legacy_late_ticks,
                                    uint32_t interpreted_late_ticks = 0,
                                    uint32_t early_ticks = 0,
                                    int32_t service_offset_signed_ticks = 0,
                                    uint32_t service_offset_abs_ticks = 0,
                                    uint32_t target_delta_mod65536_ticks = 0,
                                    uint16_t target_low16 = 0,
                                    uint16_t isr_counter_low16 = 0) {
  if (!ctx.qtimer_diag) return;

  ocxo_qtimer_diag_t& d = *ctx.qtimer_diag;
  d.count++;
  d.dwt_raw = isr_entry_dwt_raw;
  d.event_dwt = event_dwt;
  d.dwt_coordinate_source = OCXO_DWT_SOURCE_ISR_ENTRY;
  d.late_ticks = legacy_late_ticks;
  d.interpreted_late_ticks = interpreted_late_ticks;
  d.early_ticks = early_ticks;
  d.service_offset_signed_ticks = service_offset_signed_ticks;
  d.service_offset_abs_ticks = service_offset_abs_ticks;
  d.target_delta_mod65536_ticks = target_delta_mod65536_ticks;
  d.target_low16 = target_low16;
  d.isr_counter_low16 = isr_counter_low16;
}

static void ocxo_apply_perishable_fact_deferred(
    const interrupt_perishable_fact_t& fact,
    interrupt_subscriber_runtime_t* rt) {
  ocxo_runtime_context_t* ctx = ocxo_context_for(fact.kind);
  if (!ctx || !ctx->lane) return;

  ocxo_lane_t* lane = ctx->lane;

  // First-pass lane-local cadence user callback.  Regression will attach here;
  // for this migration pass it is intentionally a no-op with a visible count.
  lane->cadence_user_callback_count++;

  ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(fact.kind);
  lane->witness_fact_drain_count = ring.drain_count;
  lane->witness_fact_overflow_count = ring.overflow_count;
  lane->witness_fact_high_water = ring.high_water;
  lane->witness_fact_asap_arm_count = ring.asap_arm_count;
  lane->witness_fact_asap_fail_count = ring.asap_fail_count;

  const int32_t correction_cycles =
      dwt_cycles_for_ocxo_ticks_signed((int32_t)fact.service_offset_ticks);
  const uint32_t corrected_dwt =
      (uint32_t)((int64_t)fact.service_dwt_at_event -
                 (int64_t)correction_cycles);

  lane->witness_last_fact_sequence = fact.sequence;
  lane->witness_last_fact_one_second_due = fact.one_second_due;
  lane->witness_last_isr_csctrl_entry = fact.csctrl_entry;
  lane->witness_last_isr_compare_flag_entry = fact.compare_flag_seen;
  lane->witness_last_isr_compare_enabled_entry = fact.compare_enabled_entry;
  lane->witness_last_irq_had_armed = fact.had_armed;
  lane->witness_last_irq_had_active_rt = fact.had_active_rt;
  lane->witness_last_target_low16 = fact.target_low16;
  lane->witness_last_isr_counter_low16 = fact.service_counter_low16;
  lane->witness_last_target_delta_mod65536_ticks =
      fact.target_delta_mod65536_ticks;
  lane->witness_last_interpreted_late_ticks = fact.interpreted_late_ticks;
  lane->witness_last_early_ticks = fact.early_ticks;
  lane->witness_last_service_offset_signed_ticks =
      (int32_t)fact.service_offset_ticks;
  lane->witness_last_service_offset_abs_ticks = fact.service_offset_abs_ticks;
  lane->witness_last_service_was_early = (fact.service_offset_ticks < 0);
  lane->witness_last_service_was_on_or_after_target =
      (fact.service_offset_ticks >= 0);
  lane->witness_last_arm_to_isr_ticks = fact.arm_to_isr_ticks;
  lane->witness_last_arm_to_isr_dwt_cycles = fact.arm_to_isr_dwt_cycles;
  lane->witness_last_service_class = fact.service_class;
  lane->witness_last_isr_csctrl_after_disable = fact.csctrl_after_disable;
  lane->witness_last_isr_compare_flag_after_disable =
      fact.compare_flag_after_disable;
  lane->witness_last_isr_compare_enabled_after_disable =
      fact.compare_enabled_after_disable;
  lane->witness_last_service_correction_cycles = correction_cycles;
  lane->witness_last_service_corrected_dwt = corrected_dwt;
  lane->witness_last_late_ticks = fact.target_delta_mod65536_ticks;
  lane->witness_last_event_published = false;

  lane->witness_service_count++;
  if (fact.service_class == OCXO_SERVICE_CLASS_FALSE_IRQ) {
    lane->witness_false_irq_count++;
  } else if (fact.service_offset_ticks < 0) {
    lane->witness_early_service_count++;
  } else {
    lane->witness_on_or_after_service_count++;
  }

  ocxo_qtimer_diag_record(*ctx,
                          fact.isr_entry_dwt_raw,
                          fact.service_dwt_at_event,
                          fact.target_delta_mod65536_ticks,
                          fact.interpreted_late_ticks,
                          fact.early_ticks,
                          (int32_t)fact.service_offset_ticks,
                          fact.service_offset_abs_ticks,
                          fact.target_delta_mod65536_ticks,
                          fact.target_low16,
                          fact.service_counter_low16);

  if (!fact.one_second_due || !rt || !rt->active) {
    return;
  }

  lane->witness_fire_count++;
  lane->irq_count++;
  lane->logical_count32_at_last_second = fact.target_counter32;
  lane->witness_last_event_dwt = fact.service_dwt_at_event;
  lane->witness_last_event_counter32 = fact.target_counter32;
  lane->witness_last_event_published = true;
  lane->witness_valid_publish_count++;
  if (fact.service_offset_ticks < 0) {
    lane->witness_early_service_published_count++;
  }

  if (lane->witness_previous_event_counter32_valid) {
    const uint32_t delta =
        fact.target_counter32 - lane->witness_previous_event_counter32;
    lane->witness_last_counter_delta_ticks = delta;
    if (delta != OCXO_WITNESS_ONE_SECOND_COUNTS) {
      lane->witness_counter_delta_violation_count++;
      lane->witness_last_bad_counter_delta = delta;
    }
  } else {
    lane->witness_last_counter_delta_ticks = 0;
  }
  lane->witness_previous_event_counter32 = fact.target_counter32;
  lane->witness_previous_event_counter32_valid = true;

  // We are already in TimePop ASAP/foreground context.  Dispatch the authored
  // OCXO edge directly so a second single-slot deferred mailbox cannot drop a
  // canonical one-second event.  Canonical DWT remains the raw service DWT for
  // this pass; corrected DWT is diagnostic-only.
  g_ocxo_deferred_edge_count++;
  emit_one_second_event(*rt,
                        fact.service_dwt_at_event,
                        fact.target_counter32,
                        0,
                        false,
                        nullptr,
                        true);
}

static void ocxo_fact_drain_callback(timepop_ctx_t*,
                                     timepop_diag_t*,
                                     void* user_data) {
  auto* rt = static_cast<interrupt_subscriber_runtime_t*>(user_data);
  if (!rt || !rt->desc) return;

  const interrupt_subscriber_kind_t kind = rt->desc->kind;
  for (;;) {
    interrupt_perishable_fact_t fact{};
    if (!ocxo_fact_ring_pop(kind, fact)) break;
    ocxo_apply_perishable_fact_deferred(fact, rt);
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

struct ocxo_cadence_isr_sample_t {
  interrupt_subscriber_runtime_t* rt = nullptr;

  uint32_t isr_entry_dwt_raw = 0;
  uint32_t isr_csctrl_entry = 0;
  uint32_t isr_csctrl_after_program = 0;
  uint32_t event_dwt = 0;

  bool had_armed = false;
  bool had_active_rt = false;
  bool was_smartzero_current = false;

  uint32_t target_counter32 = 0;
  uint16_t target_low16 = 0;
  uint16_t service_counter_low16 = 0;
  uint32_t target_delta_mod65536_ticks = 0;

  int16_t service_offset_signed_ticks = 0;
  bool service_was_early = false;
  uint32_t early_ticks = 0;
  uint32_t interpreted_late_ticks = 0;
  uint32_t service_offset_abs_ticks = 0;

  uint32_t arm_remaining_ticks = 0;
  uint32_t arm_to_isr_ticks = 0;
  uint32_t arm_to_isr_dwt_cycles = 0;

  bool one_second_due = false;
};

static bool ocxo_cadence_classify_irq(ocxo_runtime_context_t& ctx,
                                      uint32_t isr_csctrl_entry) {
  ocxo_lane_t& lane = *ctx.lane;

  if ((isr_csctrl_entry & TMR_CSCTRL_TCF1) == 0) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    return false;
  }

  if (!lane.cadence_enabled || !lane.cadence_armed) {
    lane.cadence_false_irq_count++;
    lane.witness_false_irq_count++;
    ocxo_lane_clear_compare_flag(lane);
    ocxo_lane_stop_local_cadence(lane, OCXO_CADENCE_REASON_STOP);
    return false;
  }

  return true;
}

static ocxo_cadence_isr_sample_t ocxo_cadence_capture_perishable_facts(
    ocxo_runtime_context_t& ctx,
    uint32_t isr_entry_dwt_raw,
    uint32_t isr_csctrl_entry) {
  ocxo_lane_t& lane = *ctx.lane;

  ocxo_cadence_isr_sample_t sample{};
  sample.rt = ocxo_runtime_for(ctx);
  sample.isr_entry_dwt_raw = isr_entry_dwt_raw;
  sample.isr_csctrl_entry = isr_csctrl_entry;
  sample.had_armed = lane.cadence_armed;
  sample.had_active_rt = (sample.rt && sample.rt->active && lane.active);
  sample.was_smartzero_current = smartzero_is_current_lane(ctx.kind);

  if (sample.rt) sample.rt->irq_count++;

  sample.target_counter32 = lane.cadence_next_counter32;
  sample.target_low16 = (uint16_t)(sample.target_counter32 & 0xFFFFU);
  sample.service_counter_low16 = ocxo_lane_counter_now(lane);
  sample.target_delta_mod65536_ticks =
      (uint32_t)((uint16_t)(sample.service_counter_low16 - sample.target_low16));

  sample.service_offset_signed_ticks =
      (int16_t)((uint16_t)sample.target_delta_mod65536_ticks);
  sample.service_was_early = (sample.service_offset_signed_ticks < 0);
  sample.early_ticks = sample.service_was_early
      ? (uint32_t)(-(int32_t)sample.service_offset_signed_ticks)
      : 0U;
  sample.interpreted_late_ticks = sample.service_was_early
      ? 0U
      : (uint32_t)sample.service_offset_signed_ticks;
  sample.service_offset_abs_ticks = sample.service_was_early
      ? sample.early_ticks
      : sample.interpreted_late_ticks;

  sample.arm_remaining_ticks = lane.witness_last_arm_remaining_ticks;
  sample.arm_to_isr_ticks =
      (uint32_t)((uint16_t)(sample.service_counter_low16 -
                            lane.witness_last_arm_low16));
  sample.arm_to_isr_dwt_cycles =
      isr_entry_dwt_raw - lane.witness_last_arm_dwt_raw;

  return sample;
}

static void ocxo_cadence_acknowledge_and_timestamp(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  // Acknowledge hardware immediately; the next cadence target is programmed
  // before any foreground work is requested.
  ocxo_lane_clear_compare_flag(*ctx.lane);
  sample.event_dwt =
      qtimer_event_dwt_from_isr_entry_raw(sample.isr_entry_dwt_raw);
}

static void ocxo_cadence_update_synthetic_identity(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  ocxo_lane_t& lane = *ctx.lane;

  // Authored sample identity.  The compare target, not the service-time low16
  // read, is the synthetic counter32 coordinate of this cadence sample.
  if (ctx.clock32) {
    ctx.clock32->current_counter32 = sample.target_counter32;
    ctx.clock32->current_ns = (uint64_t)sample.target_counter32 * 100ULL;
    ctx.clock32->hardware16 = sample.target_low16;
    ctx.clock32->minder_update_count++;
  }

  lane.logical_count32_at_last_second = sample.target_counter32;
  lane.compare_target = sample.target_low16;
  lane.cadence_hits_total++;
  lane.cadence_fire_count++;
  lane.cadence_last_target_counter32 = sample.target_counter32;
  lane.cadence_last_target_low16 = sample.target_low16;
  lane.cadence_last_service_low16 = sample.service_counter_low16;
  lane.cadence_last_fire_dwt = sample.event_dwt;
  lane.cadence_last_isr_entry_dwt_raw = sample.isr_entry_dwt_raw;
  lane.cadence_last_service_offset_signed_ticks =
      (int32_t)sample.service_offset_signed_ticks;
  lane.cadence_last_service_offset_abs_ticks = sample.service_offset_abs_ticks;
  lane.cadence_last_interpreted_late_ticks = sample.interpreted_late_ticks;
  lane.cadence_last_early_ticks = sample.early_ticks;
  lane.cadence_last_was_early = sample.service_was_early;
  lane.witness_last_arm_to_isr_ticks = sample.arm_to_isr_ticks;
  lane.witness_last_arm_to_isr_dwt_cycles = sample.arm_to_isr_dwt_cycles;
}

static void ocxo_cadence_update_sample_phase(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  ocxo_lane_t& lane = *ctx.lane;

  if (!lane.phase_bootstrapped) {
    lane.phase_bootstrapped = true;
    lane.bootstrap_count++;
    lane.tick_mod_1000 = 0;
  }

  if (++lane.tick_mod_1000 >= TICKS_PER_SECOND_EVENT) {
    lane.tick_mod_1000 = 0;
    sample.one_second_due = true;
    lane.cadence_one_second_due_count++;
  }
  lane.cadence_last_one_second_due = sample.one_second_due;
}

static void ocxo_cadence_feed_smartzero(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  if (sample.was_smartzero_current) {
    const int idx = smartzero_index_for_kind(ctx.kind);
    if (idx >= 0) {
      smartzero_write_begin();
      g_smartzero.lanes[idx].pub.fire_count++;
      smartzero_write_end();
    }
  }

  smartzero_feed_sample(ctx.kind,
                        sample.event_dwt,
                        sample.target_counter32,
                        sample.target_low16);
}

static void ocxo_cadence_rearm_or_stop(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  ocxo_lane_t& lane = *ctx.lane;

  const bool keep_for_smartzero = smartzero_is_current_lane(ctx.kind);
  const bool keep_running = lane.active || keep_for_smartzero;
  if (keep_running) {
    const uint32_t next_target =
        sample.target_counter32 + OCXO_CADENCE_INTERVAL_TICKS;
    if (keep_for_smartzero) {
      const int idx = smartzero_index_for_kind(ctx.kind);
      if (idx >= 0) {
        smartzero_write_begin();
        g_smartzero.lanes[idx].pub.next_target_counter32 = next_target;
        g_smartzero.lanes[idx].pub.arm_count++;
        smartzero_write_end();
      }
    }

    lane.witness_last_arm_dwt_raw = ARM_DWT_CYCCNT;
    lane.witness_last_arm_counter32 = sample.target_counter32;
    lane.witness_last_arm_low16 = sample.target_low16;
    lane.witness_last_arm_target_counter32 = next_target;
    lane.witness_last_arm_target_low16 = (uint16_t)(next_target & 0xFFFFU);
    lane.witness_last_arm_remaining_ticks = OCXO_CADENCE_INTERVAL_TICKS;
    ocxo_lane_program_local_cadence_compare(lane, next_target,
                                            OCXO_CADENCE_REASON_REARM);
  } else {
    ocxo_lane_stop_local_cadence(lane, OCXO_CADENCE_REASON_STOP);
  }

  sample.isr_csctrl_after_program = lane.module->CH[lane.channel].CSCTRL;
}

static interrupt_perishable_fact_t ocxo_cadence_build_perishable_fact(
    ocxo_runtime_context_t& ctx,
    ocxo_cadence_isr_sample_t& sample) {
  ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx.kind);

  interrupt_perishable_fact_t fact{};
  fact.kind = ctx.kind;
  fact.provider = ctx.provider;
  fact.lane = ctx.lane_id;
  fact.sequence = ++ring.sequence;
  fact.isr_entry_dwt_raw = sample.isr_entry_dwt_raw;
  fact.service_dwt_at_event = sample.event_dwt;
  fact.target_counter32 = sample.target_counter32;
  fact.target_low16 = sample.target_low16;
  fact.service_counter_low16 = sample.service_counter_low16;
  fact.service_offset_ticks = sample.service_offset_signed_ticks;
  fact.service_offset_abs_ticks = sample.service_offset_abs_ticks;
  fact.interpreted_late_ticks = sample.interpreted_late_ticks;
  fact.early_ticks = sample.early_ticks;
  fact.target_delta_mod65536_ticks = sample.target_delta_mod65536_ticks;
  fact.arm_remaining_ticks = sample.arm_remaining_ticks;
  fact.arm_to_isr_ticks = sample.arm_to_isr_ticks;
  fact.arm_to_isr_dwt_cycles = sample.arm_to_isr_dwt_cycles;
  fact.csctrl_entry = sample.isr_csctrl_entry;
  fact.csctrl_after_disable = sample.isr_csctrl_after_program;
  fact.compare_flag_seen = true;
  fact.compare_enabled_entry =
      (sample.isr_csctrl_entry & TMR_CSCTRL_TCF1EN) != 0;
  fact.compare_flag_after_disable =
      (sample.isr_csctrl_after_program & TMR_CSCTRL_TCF1) != 0;
  fact.compare_enabled_after_disable =
      (sample.isr_csctrl_after_program & TMR_CSCTRL_TCF1EN) != 0;
  fact.had_armed = sample.had_armed;
  fact.had_active_rt = sample.had_active_rt;
  fact.one_second_due = sample.one_second_due && sample.had_active_rt;
  fact.service_class = sample.had_armed
      ? (sample.service_was_early ? OCXO_SERVICE_CLASS_EARLY_PRETARGET
                                  : OCXO_SERVICE_CLASS_ON_OR_AFTER_TARGET)
      : OCXO_SERVICE_CLASS_FALSE_IRQ;
  fact.exact_target_identity = true;

  return fact;
}

static void ocxo_cadence_update_one_second_witness(
    ocxo_runtime_context_t& ctx,
    const ocxo_cadence_isr_sample_t& sample) {
  if (!sample.one_second_due) return;
  ctx.lane->witness_target_counter32 = sample.target_counter32;
  ctx.lane->witness_target_low16 = sample.target_low16;
}

static void ocxo_cadence_enqueue_fact(
    ocxo_runtime_context_t& ctx,
    const interrupt_perishable_fact_t& fact,
    interrupt_subscriber_runtime_t* rt) {
  ocxo_lane_t& lane = *ctx.lane;

  if (!ocxo_fact_ring_push_from_isr(ctx.kind, fact, rt)) {
    // The ring reports the overflow.  Count the lane miss as well so the
    // existing lane report shows that a hardware event was not handed off.
    lane.miss_count++;
  }
}

static void ocxo_cadence_compare_isr(ocxo_runtime_context_t& ctx,
                                     uint32_t isr_entry_dwt_raw) {
  ocxo_lane_t& lane = *ctx.lane;

  const uint32_t isr_csctrl_entry = lane.module->CH[lane.channel].CSCTRL;
  if (!ocxo_cadence_classify_irq(ctx, isr_csctrl_entry)) return;

  ocxo_cadence_isr_sample_t sample =
      ocxo_cadence_capture_perishable_facts(ctx,
                                            isr_entry_dwt_raw,
                                            isr_csctrl_entry);
  ocxo_cadence_acknowledge_and_timestamp(ctx, sample);
  ocxo_cadence_update_synthetic_identity(ctx, sample);
  ocxo_cadence_update_sample_phase(ctx, sample);

  // Preserve the existing SmartZero custody rule: feed the sample before
  // deciding whether the local cadence remains owned by SmartZero or by the
  // normal active lane.
  ocxo_cadence_feed_smartzero(ctx, sample);
  ocxo_cadence_rearm_or_stop(ctx, sample);
  ocxo_cadence_update_one_second_witness(ctx, sample);

  interrupt_perishable_fact_t fact =
      ocxo_cadence_build_perishable_fact(ctx, sample);
  ocxo_cadence_enqueue_fact(ctx, fact, sample.rt);
}

static void qtimer2_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_compare_isr(g_ocxo1_ctx, isr_entry_dwt_raw);
}

static void qtimer3_isr(void) {
  const uint32_t isr_entry_dwt_raw = ARM_DWT_CYCCNT;  // SACRED: first instruction.
  ocxo_cadence_compare_isr(g_ocxo2_ctx, isr_entry_dwt_raw);
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


static inline void pps_relay_assert_from_isr(uint32_t sequence) {
  // PPS relay HIGH is a physical PPS witness and stays in ISR context.
  // Relay LOW is intentionally *not* scheduled through TimePop.  Instead,
  // CADENCE_MINDER's existing 1 ms heartbeat owns the countdown and deasserts
  // the relay when the counter reaches zero.  This removes relay-off from
  // TimePop ASAP/one-shot slot mutation entirely.
  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  g_pps_relay_assert_count++;
  g_pps_relay_last_assert_sequence = sequence;
  g_pps_relay_deassert_countdown_ticks = PPS_RELAY_OFF_CADENCE_TICKS;
  g_pps_relay_deassert_arm_pending = false;
  g_pps_relay_timer_active = true;
  g_pps_relay_deassert_arm_count++;
}

static inline void pps_relay_cadence_minder_tick(void) {
  uint32_t ticks = g_pps_relay_deassert_countdown_ticks;
  if (ticks == 0) return;

  ticks--;
  g_pps_relay_deassert_countdown_ticks = ticks;
  if (ticks != 0) return;

  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  g_pps_relay_timer_active = false;
  g_pps_relay_deassert_count++;
}

static void pps_post_isr_publish_inline(const interrupt_epoch_capture_t& cap,
                                        uint32_t isr_entry_dwt_raw) {
  epoch_capture_publish(cap);

  if (g_pps_entry_latency_handler) {
    g_pps_entry_latency_handler(cap.sequence, isr_entry_dwt_raw);
  }
}

static void pps_post_isr_asap_callback(timepop_ctx_t*, timepop_diag_t*, void*) {
  interrupt_epoch_capture_t cap{};
  uint32_t isr_entry_dwt_raw = 0;
  bool had_sample = false;

  __disable_irq();
  had_sample = g_pps_post_isr.pending;
  if (had_sample) {
    cap = g_pps_post_isr.cap;
    isr_entry_dwt_raw = g_pps_post_isr.isr_entry_dwt_raw;
    g_pps_post_isr.pending = false;
  }
  __enable_irq();

  if (!had_sample) return;

  g_pps_post_isr.drain_count++;
  pps_post_isr_publish_inline(cap, isr_entry_dwt_raw);
}

static void pps_post_isr_defer(const interrupt_epoch_capture_t& cap,
                               uint32_t isr_entry_dwt_raw) {
  if (!g_interrupt_runtime_ready) {
    pps_post_isr_publish_inline(cap, isr_entry_dwt_raw);
    return;
  }

  __disable_irq();
  if (g_pps_post_isr.pending) {
    g_pps_post_isr.overwrite_count++;
  }
  g_pps_post_isr.cap = cap;
  g_pps_post_isr.isr_entry_dwt_raw = isr_entry_dwt_raw;
  g_pps_post_isr.pending = true;
  g_pps_post_isr.arm_count++;
  g_pps_post_isr.last_sequence = cap.sequence;
  g_pps_post_isr.last_capture_window_cycles = cap.capture_window_cycles;
  __enable_irq();

  const timepop_handle_t h =
      timepop_arm_asap(pps_post_isr_asap_callback, nullptr, "PPS_POST_ISR");
  if (h == TIMEPOP_INVALID_HANDLE) {
    interrupt_epoch_capture_t fallback{};
    uint32_t fallback_raw = 0;
    bool had_sample = false;

    __disable_irq();
    had_sample = g_pps_post_isr.pending;
    if (had_sample) {
      fallback = g_pps_post_isr.cap;
      fallback_raw = g_pps_post_isr.isr_entry_dwt_raw;
      g_pps_post_isr.pending = false;
      g_pps_post_isr.asap_fail_count++;
    }
    __enable_irq();

    if (had_sample) {
      pps_post_isr_publish_inline(fallback, fallback_raw);
    }
  }
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
  pps_relay_assert_from_isr(g_pps_gpio_heartbeat.edge_count);

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

  pps_t pps;
  pps.sequence          = g_pps_gpio_heartbeat.edge_count;
  pps.dwt_at_edge       = pps_dwt_from_isr_entry_raw(isr_entry_dwt_raw);
  pps.counter32_at_edge = observed_counter32;
  pps.ch3_at_edge       = ch3_now;

  g_last_pps_witness = pps;
  g_last_pps_witness_valid = true;

  // Defer epoch-packet publication and PPS entry-latency diagnostic callback.
  // The witness facts above remain immediate because VCLOCK steady-state and
  // rebootstrap consume them.
  pps_post_isr_defer(epoch_cap, isr_entry_dwt_raw);

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

  // OCXO lanes now own their local 1 kHz rollover cadence.  Start the
  // compare ladder from the installed logical grid when available; otherwise
  // birth from the current low-word observation.
  return ocxo_lane_start_local_cadence(kind, *lane,
                                       *synthetic_clock_for_kind(kind),
                                       OCXO_CADENCE_REASON_START);
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
  ocxo_lane_stop_local_cadence(*lane, OCXO_CADENCE_REASON_STOP);
  return true;
}

void interrupt_request_pps_rebootstrap(void) {
  g_pps_rebootstrap_pending = true;
  g_vclock_epoch_latch = vclock_epoch_latch_t{};
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

  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  g_pps_relay_pin_initialized = true;

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
  g_pps_post_isr = pps_post_isr_mailbox_t{};
  g_pps_relay_timer_active = false;
  g_pps_relay_deassert_arm_pending = false;
  g_pps_relay_deassert_countdown_ticks = 0;
  g_pps_relay_assert_count = 0;
  g_pps_relay_deassert_count = 0;
  g_pps_relay_deassert_arm_count = 0;
  g_pps_relay_deassert_arm_fail_count = 0;
  g_pps_relay_deassert_arm_skip_count = 0;
  g_pps_relay_last_assert_sequence = 0;
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
  g_smartzero = smartzero_runtime_t{};
  for (uint32_t i = 0; i < SMARTZERO_LANE_COUNT; i++) {
    smartzero_reset_lane(i);
  }

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
  g_ocxo1_qtimer_diag = ocxo_qtimer_diag_t{};
  g_ocxo2_qtimer_diag = ocxo_qtimer_diag_t{};
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
  g_ocxo1_deferred = ocxo_deferred_work_t{};
  g_ocxo2_deferred = ocxo_deferred_work_t{};
  ocxo_perishable_fact_ring_reset(g_ocxo1_fact_ring);
  ocxo_perishable_fact_ring_reset(g_ocxo2_fact_ring);

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
  g_step0_qtimer1_priority_applied = 0;
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);
  g_step0_qtimer1_irq_enabled_by_interrupt = true;

  attachInterruptVector(IRQ_QTIMER2, qtimer2_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER2, 16);
  g_step0_qtimer2_priority_applied = 16;
  NVIC_ENABLE_IRQ(IRQ_QTIMER2);
  g_step0_qtimer2_irq_enabled_by_interrupt = true;

  attachInterruptVector(IRQ_QTIMER3, qtimer3_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER3, 16);
  g_step0_qtimer3_priority_applied = 16;
  NVIC_ENABLE_IRQ(IRQ_QTIMER3);
  g_step0_qtimer3_irq_enabled_by_interrupt = true;

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_gpio_isr, RISING);
  // PPS GPIO is now a witness/selector, not the highest-priority timing
  // interpolation rail.  Keep it below QTimer1 so VCLOCK/TimePop event facts
  // are not delayed by PPS witness work.
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);
  g_step0_gpio6789_priority_applied = 0;
  g_step0_gpio6789_configured_by_interrupt = true;

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

static void add_priority_payload(Payload& p) {
  p.add("qtimer1_sovereign_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);
  p.add("qtimer1_sovereign_priority_applied", g_step0_qtimer1_priority_applied);
  p.add("qtimer1_sovereign_priority_ok",
        g_step0_qtimer1_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);

  p.add("qtimer2_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);
  p.add("qtimer2_priority_applied", g_step0_qtimer2_priority_applied);
  p.add("qtimer2_priority_ok",
        g_step0_qtimer2_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);

  p.add("qtimer3_priority_expected", INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);
  p.add("qtimer3_priority_applied", g_step0_qtimer3_priority_applied);
  p.add("qtimer3_priority_ok",
        g_step0_qtimer3_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);

  p.add("gpio6789_priority_expected", INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);
  p.add("gpio6789_priority_applied", g_step0_gpio6789_priority_applied);
  p.add("gpio6789_priority_ok",
        g_step0_gpio6789_priority_applied == INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);

  p.add("irq_qtimer1_enabled_by_interrupt", g_step0_qtimer1_irq_enabled_by_interrupt);
  p.add("irq_qtimer2_enabled_by_interrupt", g_step0_qtimer2_irq_enabled_by_interrupt);
  p.add("irq_qtimer3_enabled_by_interrupt", g_step0_qtimer3_irq_enabled_by_interrupt);
  p.add("irq_gpio6789_configured_by_interrupt", g_step0_gpio6789_configured_by_interrupt);
}

static void add_runtime_payload(Payload& p) {
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("timing_arch_step", "LANE_LOCAL_OCXO_1KHZ_CADENCE");
  p.add("timing_arch_behavior_changed", true);
  p.add("timing_arch_vclock_authority", "QTIMER1_CH2_TIMEPOP_CADENCE");
  p.add("timing_arch_ocxo_model", "LOCAL_QTIMER_1KHZ_ROLLOVER_CADENCE");
  p.add("timing_arch_ocxo_migration_stage", "LANE_LOCAL_CADENCE_PRE_REGRESSION");
  p.add("subscriber_count", g_subscriber_count);
  p.add("single_cadence_agent", false);
  p.add("cadence_minder_isr_mode", true);
  p.add("vclock_cadence_retired", true);
  p.add("ocxo_legacy_compare_cadence_retired", true);
  p.add("ocxo_compare_live_requires_enabled_and_flag", true);
  p.add("lane_report_command", "INTERRUPT.REPORT_LANES");
  p.add("single_lane_report_command", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
}

static void add_cadence_minder_payload(Payload& p) {
  p.add("cadence_minder_period_ns", (uint64_t)CADENCE_MINDER_PERIOD_NS);
  p.add("cadence_minder_armed", g_cadence_minder_armed);
  p.add("cadence_minder_arm_count", g_cadence_minder_arm_count);
  p.add("cadence_minder_arm_failures", g_cadence_minder_arm_failures);
  p.add("cadence_minder_fire_count", g_cadence_minder_fire_count);
  p.add("cadence_minder_vclock_updates", g_cadence_minder_vclock_updates);
  p.add("cadence_minder_ocxo1_updates", g_cadence_minder_ocxo1_updates);
  p.add("cadence_minder_ocxo2_updates", g_cadence_minder_ocxo2_updates);
  p.add("cadence_minder_last_vclock_counter32", g_cadence_minder_last_vclock_counter32);
  p.add("cadence_minder_ocxo_rollover_retired", true);

  p.add("ocxo_cadence_interval_ticks", OCXO_CADENCE_INTERVAL_TICKS);
  p.add("ocxo_cadence_samples_per_second", TICKS_PER_SECOND_EVENT);
  p.add("ocxo1_cadence_enabled", g_ocxo1_lane.cadence_enabled);
  p.add("ocxo1_cadence_armed", g_ocxo1_lane.cadence_armed);
  p.add("ocxo1_cadence_epoch_valid", g_ocxo1_lane.cadence_epoch_valid);
  p.add("ocxo1_cadence_epoch_counter32", g_ocxo1_lane.cadence_epoch_counter32);
  p.add("ocxo1_cadence_next_counter32", g_ocxo1_lane.cadence_next_counter32);
  p.add("ocxo1_cadence_fire_count", g_ocxo1_lane.cadence_fire_count);
  p.add("ocxo1_cadence_one_second_due_count", g_ocxo1_lane.cadence_one_second_due_count);
  p.add("ocxo1_cadence_false_irq_count", g_ocxo1_lane.cadence_false_irq_count);
  p.add("ocxo1_cadence_last_reason", g_ocxo1_lane.cadence_last_reason);
  p.add("ocxo1_cadence_last_reason_name",
        ocxo_cadence_reason_name(g_ocxo1_lane.cadence_last_reason));

  p.add("ocxo2_cadence_enabled", g_ocxo2_lane.cadence_enabled);
  p.add("ocxo2_cadence_armed", g_ocxo2_lane.cadence_armed);
  p.add("ocxo2_cadence_epoch_valid", g_ocxo2_lane.cadence_epoch_valid);
  p.add("ocxo2_cadence_epoch_counter32", g_ocxo2_lane.cadence_epoch_counter32);
  p.add("ocxo2_cadence_next_counter32", g_ocxo2_lane.cadence_next_counter32);
  p.add("ocxo2_cadence_fire_count", g_ocxo2_lane.cadence_fire_count);
  p.add("ocxo2_cadence_one_second_due_count", g_ocxo2_lane.cadence_one_second_due_count);
  p.add("ocxo2_cadence_false_irq_count", g_ocxo2_lane.cadence_false_irq_count);
  p.add("ocxo2_cadence_last_reason", g_ocxo2_lane.cadence_last_reason);
  p.add("ocxo2_cadence_last_reason_name",
        ocxo_cadence_reason_name(g_ocxo2_lane.cadence_last_reason));
}

static void add_pps_payload(Payload& p) {
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);

  p.add("pps_post_isr_deferred", true);
  p.add("pps_post_isr_pending", g_pps_post_isr.pending);
  p.add("pps_post_isr_arm_count", g_pps_post_isr.arm_count);
  p.add("pps_post_isr_drain_count", g_pps_post_isr.drain_count);
  p.add("pps_post_isr_overwrite_count", g_pps_post_isr.overwrite_count);
  p.add("pps_post_isr_asap_fail_count", g_pps_post_isr.asap_fail_count);
  p.add("pps_post_isr_last_sequence", g_pps_post_isr.last_sequence);
  p.add("pps_post_isr_last_capture_window_cycles",
        g_pps_post_isr.last_capture_window_cycles);

  p.add("pps_relay_owner", "process_interrupt");
  p.add("pps_relay_assert_in_isr", true);
  p.add("pps_relay_deassert_in_timepop", false);
  p.add("pps_relay_deassert_in_cadence_minder", true);
  p.add("pps_relay_rearm_deassert_every_pps", true);
  p.add("pps_relay_off_ns", (uint64_t)PPS_RELAY_OFF_NS);
  p.add("pps_relay_off_cadence_ticks", PPS_RELAY_OFF_CADENCE_TICKS);
  p.add("pps_relay_pin_initialized", g_pps_relay_pin_initialized);
  p.add("pps_relay_timer_active", g_pps_relay_timer_active);
  p.add("pps_relay_deassert_arm_pending", g_pps_relay_deassert_arm_pending);
  p.add("pps_relay_deassert_countdown_ticks", g_pps_relay_deassert_countdown_ticks);
  p.add("pps_relay_assert_count", g_pps_relay_assert_count);
  p.add("pps_relay_deassert_count", g_pps_relay_deassert_count);
  p.add("pps_relay_deassert_arm_count", g_pps_relay_deassert_arm_count);
  p.add("pps_relay_deassert_arm_fail_count", g_pps_relay_deassert_arm_fail_count);
  p.add("pps_relay_deassert_arm_skip_count", g_pps_relay_deassert_arm_skip_count);
  p.add("pps_relay_last_assert_sequence", g_pps_relay_last_assert_sequence);

  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("pps_rebootstrap_count",   g_pps_rebootstrap_count);
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);
  p.add("vclock_epoch_latch_counter32", g_vclock_epoch_latch.counter32_at_edge);
  p.add("vclock_epoch_latch_observed_counter32", g_vclock_epoch_latch.observed_counter32);
  p.add("vclock_epoch_latch_first_cadence_counter32", g_vclock_epoch_latch.first_cadence_counter32);
  p.add("vclock_epoch_latch_first_cadence_dwt", g_vclock_epoch_latch.first_cadence_dwt);
  p.add("vclock_epoch_latch_backdate_ticks", g_vclock_epoch_latch.backdate_ticks);
  p.add("vclock_epoch_latch_backdate_cycles", g_vclock_epoch_latch.backdate_cycles);
  p.add("vclock_epoch_latch_sacred_dwt", g_vclock_epoch_latch.sacred_dwt);

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

  p.add("pps_edge_dispatch_registered", g_pps_edge_dispatch != nullptr);
}

static void add_epoch_capture_payload(Payload& p) {
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
  p.add("epoch_capture_vclock_counter32", epoch_cap.vclock_counter32);
  p.add("epoch_capture_ocxo1_counter32", epoch_cap.ocxo1_counter32);
  p.add("epoch_capture_ocxo2_counter32", epoch_cap.ocxo2_counter32);
}

static void add_dynamic_cps_payload(Payload& p) {
  const uint32_t dynamic_cps = interrupt_dynamic_cps();
  p.add("dynamic_cps_owner", "CLOCKS_STATIC_PPS");
  p.add("dynamic_cps", dynamic_cps);
  p.add("dynamic_cps_valid", dynamic_cps != 0);
  p.add("dynamic_cps_pps_sequence", g_pps_gpio_heartbeat.edge_count);
  p.add("dynamic_cps_last_pvc_dwt_at_edge", g_pps_gpio_heartbeat.last_dwt);
  p.add("dynamic_cps_last_reseed_value", dynamic_cps);
  p.add("dynamic_cps_last_reseed_was_computed", dynamic_cps != 0);
}


static void add_smartzero_lane_payload(Payload& parent,
                                       const char* key,
                                       const interrupt_smartzero_lane_snapshot_t& z) {
  Payload p;
  p.add("kind", interrupt_subscriber_kind_str(z.kind));
  p.add("state", smartzero_lane_state_name(z.state));
  p.add("last_decision", smartzero_decision_name(z.last_decision));
  p.add("sample_count", z.sample_count);
  p.add("interval_attempt_count", z.interval_attempt_count);
  p.add("accepted_count", z.accepted_count);
  p.add("rejected_count", z.rejected_count);
  p.add("waiting_for_cps_count", z.waiting_for_cps_count);
  p.add("cps_used", z.cps_used);
  p.add("expected_interval_cycles", z.expected_interval_cycles);
  p.add("tolerance_cycles", z.tolerance_cycles);
  p.add("required_counter_delta_ticks", z.required_counter_delta_ticks);
  p.add("last_sample_dwt", z.last_sample_dwt);
  p.add("last_sample_counter32", z.last_sample_counter32);
  p.add("last_sample_hardware16", (uint32_t)z.last_sample_hardware16);
  p.add("previous_sample_dwt", z.previous_sample_dwt);
  p.add("previous_sample_counter32", z.previous_sample_counter32);
  p.add("previous_sample_hardware16", (uint32_t)z.previous_sample_hardware16);
  p.add("last_interval_cycles", z.last_interval_cycles);
  p.add("last_interval_error_cycles", z.last_interval_error_cycles);
  p.add("max_abs_interval_error_cycles", z.max_abs_interval_error_cycles);
  p.add("last_counter_delta_ticks", z.last_counter_delta_ticks);
  p.add("anchor_dwt", z.anchor_dwt);
  p.add("anchor_counter32", z.anchor_counter32);
  p.add("anchor_hardware16", (uint32_t)z.anchor_hardware16);
  p.add("anchor_pair_previous_dwt", z.anchor_pair_previous_dwt);
  p.add("anchor_pair_previous_counter32", z.anchor_pair_previous_counter32);
  p.add("arm_count", z.arm_count);
  p.add("fire_count", z.fire_count);
  p.add("next_target_counter32", z.next_target_counter32);
  parent.add_object(key, p);
}

static void add_smartzero_payload(Payload& p) {
  interrupt_smartzero_snapshot_t z{};
  (void)interrupt_smartzero_live_snapshot(&z);

  // Explicit live-acquisition surface.  This is NOT the installed epoch proof.
  p.add("live_smartzero_phase", smartzero_phase_name(z.phase));
  p.add("live_smartzero_running", z.running);
  p.add("live_smartzero_complete", z.complete);
  p.add("live_smartzero_aborted", z.aborted);
  p.add("live_smartzero_sequence", z.sequence);
  p.add("live_smartzero_begin_count", z.begin_count);
  p.add("live_smartzero_complete_count", z.complete_count);
  p.add("live_smartzero_abort_count", z.abort_count);
  p.add("live_smartzero_current_lane_index", z.current_lane_index);
  p.add("live_smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));
  p.add("live_smartzero_sample_rate_hz", z.sample_rate_hz);
  p.add("live_smartzero_counter_delta_ticks", z.counter_delta_ticks);
  p.add("live_smartzero_tolerance_cycles", z.tolerance_cycles);

  // Legacy aliases retained for existing tooling.  These names refer to the
  // live acquisition attempt only; CLOCKS reports the installed proof.
  p.add("smartzero_phase", smartzero_phase_name(z.phase));
  p.add("smartzero_running", z.running);
  p.add("smartzero_complete", z.complete);
  p.add("smartzero_aborted", z.aborted);
  p.add("smartzero_sequence", z.sequence);
  p.add("smartzero_begin_count", z.begin_count);
  p.add("smartzero_complete_count", z.complete_count);
  p.add("smartzero_abort_count", z.abort_count);
  p.add("smartzero_current_lane_index", z.current_lane_index);
  p.add("smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));
  p.add("smartzero_sample_rate_hz", z.sample_rate_hz);
  p.add("smartzero_counter_delta_ticks", z.counter_delta_ticks);
  p.add("smartzero_tolerance_cycles", z.tolerance_cycles);

  Payload live_lanes;
  add_smartzero_lane_payload(live_lanes, "vclock", z.lanes[0]);
  add_smartzero_lane_payload(live_lanes, "ocxo1", z.lanes[1]);
  add_smartzero_lane_payload(live_lanes, "ocxo2", z.lanes[2]);
  p.add_object("live_smartzero", live_lanes);

  Payload legacy_lanes;
  add_smartzero_lane_payload(legacy_lanes, "vclock", z.lanes[0]);
  add_smartzero_lane_payload(legacy_lanes, "ocxo1", z.lanes[1]);
  add_smartzero_lane_payload(legacy_lanes, "ocxo2", z.lanes[2]);
  p.add_object("smartzero", legacy_lanes);  // legacy live alias
}

static void add_vclock_clock32_payload(Payload& p, const char* prefix) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_bool("clock32_zeroed", g_vclock_clock32.zeroed);
  add_u64("clock32_zero_ns", g_vclock_clock32.zero_ns);
  add_u32("clock32_zero_counter32", g_vclock_clock32.zero_counter32);
  add_u32("clock32_current", g_vclock_clock32.current_counter32);
  add_u32("clock32_hardware_low16_at_zero", (uint32_t)g_vclock_clock32.hardware_low16_at_zero);
  add_u32("clock32_hardware_low16_at_current", (uint32_t)g_vclock_clock32.hardware_low16_at_current);
  add_bool("clock32_hardware_anchor_valid", g_vclock_clock32.hardware_anchor_valid);
  add_u32("clock32_hardware_anchor_update_count", g_vclock_clock32.hardware_anchor_update_count);
  add_bool("clock32_pending_zero", g_vclock_clock32.pending_zero);
  add_u32("clock32_zero_count", g_vclock_clock32.zero_count);
  add_u32("clock32_minder_update_count", g_vclock_clock32.minder_update_count);
  add_u32("clock32_pending_zero_count", g_vclock_clock32.pending_zero_count);
}

static void add_ocxo_clock32_payload(Payload& p,
                                     const char* prefix,
                                     const synthetic_clock32_t& clock32) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_bool("clock32_zeroed", clock32.zeroed);
  add_u64("clock32_zero_ns", clock32.zero_ns);
  add_u32("clock32_zero_counter32", clock32.zero_counter32);
  add_u32("clock32_current", clock32.current_counter32);
  add_bool("clock32_pending_zero", clock32.pending_zero);
  add_u32("clock32_zero_count", clock32.zero_count);
  add_u32("clock32_minder_update_count", clock32.minder_update_count);
}

static void add_runtime_lane_summary(Payload& p,
                                     const char* prefix,
                                     const interrupt_subscriber_runtime_t* rt) {
  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_bool("subscribed", rt ? rt->subscribed : false);
  add_bool("active", rt ? rt->active : false);
  add_bool("has_fired", rt ? rt->has_fired : false);
  add_u32("start_count", rt ? rt->start_count : 0U);
  add_u32("stop_count", rt ? rt->stop_count : 0U);
  add_u32("irq_count", rt ? rt->irq_count : 0U);
  add_u32("dispatch_count", rt ? rt->dispatch_count : 0U);
  add_u32("event_count", rt ? rt->event_count : 0U);
  if (rt && rt->has_fired) {
    add_u32("last_event_dwt", rt->last_event.dwt_at_event);
    add_u32("last_event_counter32", rt->last_event.counter32_at_event);
    add_u32("last_event_status", (uint32_t)((uint8_t)rt->last_event.status));
    add_u32("last_diag_anchor_selection_kind", rt->last_diag.anchor_selection_kind);
    add_u32("last_diag_anchor_failure_mask", rt->last_diag.anchor_failure_mask);
  } else {
    add_u32("last_event_dwt", 0);
    add_u32("last_event_counter32", 0);
    add_u32("last_event_status", 0);
    add_u32("last_diag_anchor_selection_kind", 0);
    add_u32("last_diag_anchor_failure_mask", 0);
  }
}

static void add_vclock_lane_payload(Payload& p, bool detailed) {
  p.add("lane", "VCLOCK");
  p.add("kind", "VCLOCK");
  p.add("provider", "QTIMER1");
  p.add("hardware_lane", "QTIMER1_CH2_COMP");
  p.add("cadence_source", "QTIMER1_CH2_TIMEPOP_CADENCE_MINDER");
  p.add("counter_source", "QTIMER1_CH0_SYNTHETIC_COUNTER32");
  p.add("event_source", "CADENCE_MINDER_1000TH_FIRE");
  p.add("dwt_authority", "QTIMER1_CH2_TIMEPOP_EVENT_DWT");

  add_runtime_lane_summary(p, "vclock", g_rt_vclock);
  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);
  p.add("vclock_bootstrap_count", g_vclock_lane.bootstrap_count);
  p.add("vclock_cadence_hits_total", g_vclock_lane.cadence_hits_total);
  p.add("vclock_compare_target", (uint32_t)g_vclock_lane.compare_target);
  p.add("vclock_tick_mod_1000", g_vclock_lane.tick_mod_1000);
  p.add("vclock_logical_count32", g_vclock_lane.logical_count32_at_last_second);

  if (!detailed) return;

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

  add_vclock_clock32_payload(p, "vclock");
  p.add("qtimer1_ch1_active", g_qtimer1_ch1_active);
  p.add("qtimer1_ch1_sequence", g_qtimer1_ch1_sequence);
  p.add("qtimer1_ch1_target_counter32", g_qtimer1_ch1_target_counter32);
  p.add("qtimer1_ch1_next_compare_counter32", g_qtimer1_ch1_next_compare_counter32);
  p.add("qtimer1_ch1_arm_count", g_qtimer1_ch1_arm_count);
  p.add("qtimer1_ch1_fire_count", g_qtimer1_ch1_fire_count);
  p.add("qtimer1_ch1_hop_count", g_qtimer1_ch1_hop_count);
  p.add("qtimer1_ch2_last_target_counter32", g_qtimer1_ch2_last_target_counter32);
  p.add("qtimer1_ch2_arm_count", g_qtimer1_ch2_arm_count);
}

static void add_ocxo_lane_payload(Payload& p,
                                  const ocxo_runtime_context_t& ctx,
                                  bool detailed) {
  ocxo_lane_t& lane = *ctx.lane;
  const synthetic_clock32_t& clock32 = *ctx.clock32;
  const interrupt_subscriber_runtime_t* rt = ocxo_runtime_for(ctx);
  const ocxo_qtimer_diag_t& qdiag = *ctx.qtimer_diag;
  const char* prefix = ctx.prefix;

  p.add("lane", ctx.name);
  p.add("kind", ctx.name);
  p.add("provider", interrupt_provider_kind_str(ctx.provider));
  p.add("hardware_lane", interrupt_lane_str(ctx.lane_id));
  p.add("cadence_source", ctx.cadence_source);
  p.add("counter_source", ctx.counter_source);
  p.add("event_source", "LOCAL_CADENCE_1000TH_SAMPLE");
  p.add("dwt_authority", ctx.dwt_authority);

  add_runtime_lane_summary(p, prefix, rt);

  char key[96];
  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_i32 = [&](const char* suffix, int32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };
  auto add_str = [&](const char* suffix, const char* value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value ? value : "");
  };

  add_bool("initialized", lane.initialized);
  add_bool("active", lane.active);
  add_u32("irq_count", lane.irq_count);
  add_u32("miss_count", lane.miss_count);
  add_u32("bootstrap_count", lane.bootstrap_count);
  add_u32("cadence_hits_total", lane.cadence_hits_total);
  add_bool("cadence_enabled", lane.cadence_enabled);
  add_bool("cadence_armed", lane.cadence_armed);
  add_bool("cadence_epoch_valid", lane.cadence_epoch_valid);
  add_u32("cadence_epoch_counter32", lane.cadence_epoch_counter32);
  add_u32("cadence_interval_ticks", OCXO_CADENCE_INTERVAL_TICKS);
  add_u32("cadence_next_counter32", lane.cadence_next_counter32);
  add_u32("cadence_next_low16", (uint32_t)lane.cadence_next_low16);
  add_u32("cadence_fire_count", lane.cadence_fire_count);
  add_u32("cadence_arm_count", lane.cadence_arm_count);
  add_u32("cadence_rearm_count", lane.cadence_rearm_count);
  add_u32("cadence_false_irq_count", lane.cadence_false_irq_count);
  add_u32("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  add_u32("cadence_user_callback_count", lane.cadence_user_callback_count);
  add_u32("cadence_last_reason", lane.cadence_last_reason);
  add_str("cadence_last_reason_name", ocxo_cadence_reason_name(lane.cadence_last_reason));
  add_u32("compare_target", (uint32_t)lane.compare_target);
  add_u32("tick_mod_1000", lane.tick_mod_1000);
  add_u32("logical_count32", lane.logical_count32_at_last_second);
  add_bool("witness_target_initialized", lane.witness_target_initialized);
  add_bool("witness_armed", lane.witness_armed);
  add_u32("witness_arm_count", lane.witness_arm_count);
  add_u32("witness_fire_count", lane.witness_fire_count);
  add_u32("witness_false_irq_count", lane.witness_false_irq_count);
  add_u32("witness_missed_target_count", lane.witness_missed_target_count);
  add_u32("witness_late_arm_count", lane.witness_late_arm_count);
  add_u32("witness_target_counter32", lane.witness_target_counter32);
  add_u32("witness_last_event_dwt", lane.witness_last_event_dwt);
  add_u32("witness_last_event_counter32", lane.witness_last_event_counter32);
  add_u32("witness_last_late_ticks", lane.witness_last_late_ticks);
  add_u32("witness_last_arm_remaining_ticks", lane.witness_last_arm_remaining_ticks);

  // Compare-service interpretation.  These make early/stale service visible
  // without changing the existing publication path.
  add_u32("witness_last_target_delta_mod65536_ticks",
          lane.witness_last_target_delta_mod65536_ticks);
  add_i32("witness_last_service_offset_signed_ticks",
          lane.witness_last_service_offset_signed_ticks);
  add_u32("witness_last_early_ticks", lane.witness_last_early_ticks);
  add_u32("witness_last_interpreted_late_ticks",
          lane.witness_last_interpreted_late_ticks);
  add_u32("witness_last_service_offset_abs_ticks",
          lane.witness_last_service_offset_abs_ticks);
  add_bool("witness_last_service_was_early",
           lane.witness_last_service_was_early);
  add_bool("witness_last_service_was_on_or_after_target",
           lane.witness_last_service_was_on_or_after_target);
  add_bool("witness_last_event_published", lane.witness_last_event_published);
  add_u32("witness_last_service_class", lane.witness_last_service_class);
  add_str("witness_last_service_class_name",
          ocxo_service_class_name(lane.witness_last_service_class));
  add_u32("witness_service_count", lane.witness_service_count);
  add_u32("witness_early_service_count", lane.witness_early_service_count);
  add_u32("witness_on_or_after_service_count",
          lane.witness_on_or_after_service_count);
  add_u32("witness_valid_publish_count", lane.witness_valid_publish_count);
  add_u32("witness_early_service_published_count",
          lane.witness_early_service_published_count);
  add_u32("witness_last_fact_sequence", lane.witness_last_fact_sequence);
  add_bool("witness_last_fact_one_second_due",
           lane.witness_last_fact_one_second_due);
  add_i32("witness_last_service_correction_cycles",
          lane.witness_last_service_correction_cycles);
  add_u32("witness_last_service_corrected_dwt",
          lane.witness_last_service_corrected_dwt);
  add_u32("witness_last_counter_delta_ticks",
          lane.witness_last_counter_delta_ticks);
  add_u32("witness_counter_delta_violation_count",
          lane.witness_counter_delta_violation_count);
  add_u32("witness_last_bad_counter_delta",
          lane.witness_last_bad_counter_delta);
  add_u32("witness_fact_enqueue_count", lane.witness_fact_enqueue_count);
  add_u32("witness_fact_drain_count", lane.witness_fact_drain_count);
  add_u32("witness_fact_overflow_count", lane.witness_fact_overflow_count);
  add_u32("witness_fact_high_water", lane.witness_fact_high_water);
  add_u32("witness_fact_asap_arm_count", lane.witness_fact_asap_arm_count);
  add_u32("witness_fact_asap_fail_count", lane.witness_fact_asap_fail_count);
  add_u32("witness_schedule_last_decision",
          lane.witness_schedule_last_decision);
  add_str("witness_schedule_last_decision_name",
          ocxo_schedule_decision_name(lane.witness_schedule_last_decision));

  add_u32("cadence_last_target_counter32", lane.cadence_last_target_counter32);
  add_u32("cadence_last_target_low16", (uint32_t)lane.cadence_last_target_low16);
  add_u32("cadence_last_service_low16", (uint32_t)lane.cadence_last_service_low16);
  add_u32("cadence_last_fire_dwt", lane.cadence_last_fire_dwt);
  add_u32("cadence_last_isr_entry_dwt_raw", lane.cadence_last_isr_entry_dwt_raw);
  add_i32("cadence_last_service_offset_signed_ticks",
          lane.cadence_last_service_offset_signed_ticks);
  add_u32("cadence_last_service_offset_abs_ticks",
          lane.cadence_last_service_offset_abs_ticks);
  add_u32("cadence_last_interpreted_late_ticks",
          lane.cadence_last_interpreted_late_ticks);
  add_u32("cadence_last_early_ticks", lane.cadence_last_early_ticks);
  add_bool("cadence_last_was_early", lane.cadence_last_was_early);
  add_bool("cadence_last_one_second_due", lane.cadence_last_one_second_due);

  if (!detailed) return;

  add_u32("witness_schedule_last_current_counter32",
          lane.witness_schedule_last_current_counter32);
  add_u32("witness_schedule_last_target_counter32",
          lane.witness_schedule_last_target_counter32);
  add_u32("witness_schedule_last_remaining_ticks",
          lane.witness_schedule_last_remaining_ticks);
  add_u32("witness_schedule_last_phase_ticks",
          lane.witness_schedule_last_phase_ticks);
  add_u32("witness_schedule_last_ticks_until_arm_window",
          lane.witness_schedule_last_ticks_until_arm_window);
  add_u32("witness_schedule_last_current_low16",
          (uint32_t)lane.witness_schedule_last_current_low16);
  add_u32("witness_schedule_last_target_low16",
          (uint32_t)lane.witness_schedule_last_target_low16);

  add_u32("witness_last_arm_dwt_raw", lane.witness_last_arm_dwt_raw);
  add_u32("witness_last_arm_counter32", lane.witness_last_arm_counter32);
  add_u32("witness_last_arm_low16", (uint32_t)lane.witness_last_arm_low16);
  add_u32("witness_last_arm_target_counter32",
          lane.witness_last_arm_target_counter32);
  add_u32("witness_last_arm_target_low16",
          (uint32_t)lane.witness_last_arm_target_low16);
  add_u32("witness_last_arm_to_isr_ticks",
          lane.witness_last_arm_to_isr_ticks);
  add_u32("witness_last_arm_to_isr_dwt_cycles",
          lane.witness_last_arm_to_isr_dwt_cycles);
  add_i32("witness_last_arm_remaining_minus_early_ticks",
          (int32_t)lane.witness_last_arm_remaining_ticks -
          (int32_t)lane.witness_last_early_ticks);

  add_u32("witness_last_program_csctrl_before",
          lane.witness_last_program_csctrl_before);
  add_u32("witness_last_program_csctrl_after",
          lane.witness_last_program_csctrl_after);
  add_bool("witness_last_program_flag_before",
           lane.witness_last_program_flag_before);
  add_bool("witness_last_program_flag_after",
           lane.witness_last_program_flag_after);
  add_bool("witness_last_program_enabled_before",
           lane.witness_last_program_enabled_before);
  add_bool("witness_last_program_enabled_after",
           lane.witness_last_program_enabled_after);

  add_u32("cadence_last_program_csctrl_before",
          lane.cadence_last_program_csctrl_before);
  add_u32("cadence_last_program_csctrl_after",
          lane.cadence_last_program_csctrl_after);
  add_bool("cadence_last_program_flag_before",
           lane.cadence_last_program_flag_before);
  add_bool("cadence_last_program_flag_after",
           lane.cadence_last_program_flag_after);
  add_bool("cadence_last_program_enabled_before",
           lane.cadence_last_program_enabled_before);
  add_bool("cadence_last_program_enabled_after",
           lane.cadence_last_program_enabled_after);

  add_u32("witness_last_isr_csctrl_entry",
          lane.witness_last_isr_csctrl_entry);
  add_u32("witness_last_isr_csctrl_after_disable",
          lane.witness_last_isr_csctrl_after_disable);
  add_bool("witness_last_isr_compare_flag_entry",
           lane.witness_last_isr_compare_flag_entry);
  add_bool("witness_last_isr_compare_enabled_entry",
           lane.witness_last_isr_compare_enabled_entry);
  add_bool("witness_last_isr_compare_flag_after_disable",
           lane.witness_last_isr_compare_flag_after_disable);
  add_bool("witness_last_isr_compare_enabled_after_disable",
           lane.witness_last_isr_compare_enabled_after_disable);
  add_bool("witness_last_irq_had_armed", lane.witness_last_irq_had_armed);
  add_bool("witness_last_irq_had_active_rt",
           lane.witness_last_irq_had_active_rt);

  add_u32("witness_last_target_low16",
          (uint32_t)lane.witness_last_target_low16);
  add_u32("witness_last_isr_counter_low16",
          (uint32_t)lane.witness_last_isr_counter_low16);
  add_bool("witness_last_service_early_immediate_after_arm",
           lane.witness_last_service_was_early &&
           lane.witness_last_arm_to_isr_ticks <= 256U);

  add_ocxo_clock32_payload(p, prefix, clock32);

  const uint16_t csctrl = lane.module->CH[lane.channel].CSCTRL;
  const bool compare_enabled = (csctrl & TMR_CSCTRL_TCF1EN) != 0;
  const bool compare_flag = (csctrl & TMR_CSCTRL_TCF1) != 0;
  ocxo_perishable_ring_t& ring = ocxo_fact_ring_for(ctx.kind);
  ocxo_deferred_work_t& deferred = (ctx.kind == interrupt_subscriber_kind_t::OCXO1)
      ? g_ocxo1_deferred
      : g_ocxo2_deferred;

  add_u32("qtimer_count", qdiag.count);
  add_u32("qtimer_counter", (uint32_t)lane.module->CH[lane.channel].CNTR);
  add_u32("qtimer_comp1", (uint32_t)lane.module->CH[lane.channel].COMP1);
  add_u32("qtimer_csctrl", (uint32_t)csctrl);
  add_bool("qtimer_compare_enabled", compare_enabled);
  add_bool("qtimer_compare_flag", compare_flag);
  add_bool("qtimer_compare_live", compare_enabled && compare_flag);
  add_u32("qtimer_last_late_ticks", qdiag.late_ticks);
  add_u32("qtimer_last_interpreted_late_ticks", qdiag.interpreted_late_ticks);
  add_u32("qtimer_last_early_ticks", qdiag.early_ticks);
  add_i32("qtimer_last_service_offset_signed_ticks",
          qdiag.service_offset_signed_ticks);
  add_u32("qtimer_last_service_offset_abs_ticks",
          qdiag.service_offset_abs_ticks);
  add_u32("qtimer_last_target_delta_mod65536_ticks",
          qdiag.target_delta_mod65536_ticks);
  add_u32("qtimer_last_target_low16", (uint32_t)qdiag.target_low16);
  add_u32("qtimer_last_isr_counter_low16",
          (uint32_t)qdiag.isr_counter_low16);
  add_u32("qtimer_last_dwt_raw", qdiag.dwt_raw);
  add_u32("qtimer_last_event_dwt", qdiag.event_dwt);
  add_u32("qtimer_last_dwt_coordinate_source",
          qdiag.dwt_coordinate_source);

  add_u32("perishable_fact_ring_size", OCXO_PERISHABLE_FACT_RING_SIZE);
  add_u32("perishable_fact_ring_count", ring.count);
  add_u32("perishable_fact_enqueue_count", ring.enqueue_count);
  add_u32("perishable_fact_drain_count", ring.drain_count);
  add_u32("perishable_fact_overflow_count", ring.overflow_count);
  add_u32("perishable_fact_high_water", ring.high_water);
  add_u32("perishable_fact_asap_arm_count", ring.asap_arm_count);
  add_u32("perishable_fact_asap_fail_count", ring.asap_fail_count);
  add_bool("perishable_fact_drain_armed", ring.drain_armed);
  add_bool("deferred_edge_pending", deferred.edge_pending);
}

static Payload cmd_report(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_COMPACT");
  p.add("subreports",
        "REPORT_STATUS REPORT_PPS REPORT_CADENCE REPORT_SMARTZERO "
        "REPORT_BRIDGE REPORT_LANES REPORT_LANE");

  // Keep the default INTERRUPT.REPORT deliberately small.  The former
  // monolithic report carried PPS, epoch capture, SmartZero lane proof,
  // bridge, cadence, and lane forensics all at once; after the perishable
  // fact pass it can exceed the transport/payload budget.  This compact
  // report is a health dashboard only.  Heavy surfaces are explicit opt-in
  // subreports below.
  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("runtime_ready",  g_interrupt_runtime_ready);
  p.add("irqs_enabled",   g_interrupt_irqs_enabled);
  p.add("timing_arch_step", "LANE_LOCAL_OCXO_1KHZ_CADENCE");
  p.add("timing_arch_behavior_changed", true);
  p.add("single_cadence_agent", false);
  p.add("cadence_minder_isr_mode", true);
  p.add("ocxo_lane_local_cadence", true);
  p.add("ocxo_perishable_fact_capture", true);
  p.add("ocxo_fact_ring_size", OCXO_PERISHABLE_FACT_RING_SIZE);

  p.add("qtimer1_priority_ok",
        g_step0_qtimer1_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER1_PRIORITY);
  p.add("qtimer2_priority_ok",
        g_step0_qtimer2_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER2_PRIORITY);
  p.add("qtimer3_priority_ok",
        g_step0_qtimer3_priority_applied == INTERRUPT_STEP0_EXPECTED_QTIMER3_PRIORITY);
  p.add("gpio6789_priority_ok",
        g_step0_gpio6789_priority_applied == INTERRUPT_STEP0_EXPECTED_GPIO6789_PRIORITY);

  p.add("gpio_edge_count", g_pps_gpio_heartbeat.edge_count);
  p.add("gpio_irq_count", g_gpio_irq_count);
  p.add("gpio_miss_count", g_gpio_miss_count);
  p.add("pps_post_isr_pending", g_pps_post_isr.pending);
  p.add("pps_post_isr_overwrite_count", g_pps_post_isr.overwrite_count);
  p.add("pps_post_isr_asap_fail_count", g_pps_post_isr.asap_fail_count);
  p.add("pps_rebootstrap_pending", g_pps_rebootstrap_pending);
  p.add("vclock_epoch_latch_pending", g_vclock_epoch_latch.pending);

  p.add("cadence_minder_armed", g_cadence_minder_armed);
  p.add("cadence_minder_fire_count", g_cadence_minder_fire_count);
  p.add("cadence_minder_arm_failures", g_cadence_minder_arm_failures);
  p.add("cadence_minder_vclock_updates", g_cadence_minder_vclock_updates);
  p.add("cadence_minder_ocxo1_updates", g_cadence_minder_ocxo1_updates);
  p.add("cadence_minder_ocxo2_updates", g_cadence_minder_ocxo2_updates);

  interrupt_smartzero_snapshot_t z{};
  (void)interrupt_smartzero_live_snapshot(&z);
  p.add("live_smartzero_phase", smartzero_phase_name(z.phase));
  p.add("live_smartzero_running", z.running);
  p.add("live_smartzero_complete", z.complete);
  p.add("live_smartzero_aborted", z.aborted);
  p.add("live_smartzero_sequence", z.sequence);
  p.add("live_smartzero_current_lane", interrupt_subscriber_kind_str(z.current_lane));

  p.add("vclock_event_count", g_rt_vclock ? g_rt_vclock->event_count : 0U);
  p.add("vclock_dispatch_count", g_rt_vclock ? g_rt_vclock->dispatch_count : 0U);
  p.add("vclock_irq_count", g_vclock_lane.irq_count);
  p.add("vclock_miss_count", g_vclock_lane.miss_count);

  p.add("ocxo1_event_count", g_rt_ocxo1 ? g_rt_ocxo1->event_count : 0U);
  p.add("ocxo1_dispatch_count", g_rt_ocxo1 ? g_rt_ocxo1->dispatch_count : 0U);
  p.add("ocxo1_irq_count", g_ocxo1_lane.irq_count);
  p.add("ocxo1_miss_count", g_ocxo1_lane.miss_count);
  p.add("ocxo1_cadence_enabled", g_ocxo1_lane.cadence_enabled);
  p.add("ocxo1_cadence_armed", g_ocxo1_lane.cadence_armed);
  p.add("ocxo1_cadence_fire_count", g_ocxo1_lane.cadence_fire_count);
  p.add("ocxo1_cadence_one_second_due_count", g_ocxo1_lane.cadence_one_second_due_count);
  p.add("ocxo1_cadence_false_irq_count", g_ocxo1_lane.cadence_false_irq_count);
  p.add("ocxo1_cadence_next_counter32", g_ocxo1_lane.cadence_next_counter32);
  p.add("ocxo1_cadence_service_offset_ticks",
        g_ocxo1_lane.cadence_last_service_offset_signed_ticks);
  p.add("ocxo1_service_offset_ticks", g_ocxo1_lane.witness_last_service_offset_signed_ticks);
  p.add("ocxo1_counter_delta_violation_count", g_ocxo1_lane.witness_counter_delta_violation_count);
  p.add("ocxo1_missed_target_count", g_ocxo1_lane.witness_missed_target_count);
  p.add("ocxo1_false_irq_count", g_ocxo1_lane.witness_false_irq_count);
  p.add("ocxo1_late_arm_count", g_ocxo1_lane.witness_late_arm_count);
  p.add("ocxo1_fact_enqueue_count", g_ocxo1_fact_ring.enqueue_count);
  p.add("ocxo1_fact_drain_count", g_ocxo1_fact_ring.drain_count);
  p.add("ocxo1_fact_overflow_count", g_ocxo1_fact_ring.overflow_count);
  p.add("ocxo1_fact_high_water", g_ocxo1_fact_ring.high_water);
  p.add("ocxo1_fact_asap_fail_count", g_ocxo1_fact_ring.asap_fail_count);

  p.add("ocxo2_event_count", g_rt_ocxo2 ? g_rt_ocxo2->event_count : 0U);
  p.add("ocxo2_dispatch_count", g_rt_ocxo2 ? g_rt_ocxo2->dispatch_count : 0U);
  p.add("ocxo2_irq_count", g_ocxo2_lane.irq_count);
  p.add("ocxo2_miss_count", g_ocxo2_lane.miss_count);
  p.add("ocxo2_cadence_enabled", g_ocxo2_lane.cadence_enabled);
  p.add("ocxo2_cadence_armed", g_ocxo2_lane.cadence_armed);
  p.add("ocxo2_cadence_fire_count", g_ocxo2_lane.cadence_fire_count);
  p.add("ocxo2_cadence_one_second_due_count", g_ocxo2_lane.cadence_one_second_due_count);
  p.add("ocxo2_cadence_false_irq_count", g_ocxo2_lane.cadence_false_irq_count);
  p.add("ocxo2_cadence_next_counter32", g_ocxo2_lane.cadence_next_counter32);
  p.add("ocxo2_cadence_service_offset_ticks",
        g_ocxo2_lane.cadence_last_service_offset_signed_ticks);
  p.add("ocxo2_service_offset_ticks", g_ocxo2_lane.witness_last_service_offset_signed_ticks);
  p.add("ocxo2_counter_delta_violation_count", g_ocxo2_lane.witness_counter_delta_violation_count);
  p.add("ocxo2_missed_target_count", g_ocxo2_lane.witness_missed_target_count);
  p.add("ocxo2_false_irq_count", g_ocxo2_lane.witness_false_irq_count);
  p.add("ocxo2_late_arm_count", g_ocxo2_lane.witness_late_arm_count);
  p.add("ocxo2_fact_enqueue_count", g_ocxo2_fact_ring.enqueue_count);
  p.add("ocxo2_fact_drain_count", g_ocxo2_fact_ring.drain_count);
  p.add("ocxo2_fact_overflow_count", g_ocxo2_fact_ring.overflow_count);
  p.add("ocxo2_fact_high_water", g_ocxo2_fact_ring.high_water);
  p.add("ocxo2_fact_asap_fail_count", g_ocxo2_fact_ring.asap_fail_count);

  p.add("ocxo_deferred_asap_overwrite_count", g_ocxo_deferred_asap_overwrite_count);
  p.add("ocxo_deferred_edge_count", g_ocxo_deferred_edge_count);
  return p;
}

static Payload cmd_report_status(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_STATUS");
  add_runtime_payload(p);
  add_priority_payload(p);
  return p;
}

static Payload cmd_report_pps(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_PPS");
  add_pps_payload(p);
  add_epoch_capture_payload(p);
  return p;
}

static Payload cmd_report_cadence(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_CADENCE");
  add_cadence_minder_payload(p);
  add_dynamic_cps_payload(p);
  return p;
}

static Payload cmd_report_smartzero(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_SMARTZERO");
  add_smartzero_payload(p);
  return p;
}

static void add_bridge_payload(Payload& p) {
  p.add("pvc_anchor_ring_count", g_pvc_anchor_count);
  p.add("pvc_anchor_ring_head", g_pvc_anchor_head);
  p.add("pvc_anchor_ring_seq", g_pvc_anchor_seq);
  p.add("pvc_anchor_reset_pending", g_pvc_anchor_reset_pending);
  add_bridge_stats_payload(p, "timepop", g_bridge_stats_timepop);
  add_bridge_stats_payload(p, "ocxo1", g_bridge_stats_ocxo1);
  add_bridge_stats_payload(p, "ocxo2", g_bridge_stats_ocxo2);
}

static Payload cmd_report_bridge(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_BRIDGE");
  add_bridge_payload(p);
  return p;
}

static void add_lane_summary_object(Payload& parent,
                                    const char* key,
                                    const char* name,
                                    const interrupt_subscriber_runtime_t* rt,
                                    uint32_t irq_count,
                                    uint32_t miss_count,
                                    uint32_t tick_mod_1000,
                                    uint32_t logical_count32) {
  Payload lane;
  lane.add("lane", name);
  lane.add("subscribed", rt ? rt->subscribed : false);
  lane.add("active", rt ? rt->active : false);
  lane.add("has_fired", rt ? rt->has_fired : false);
  lane.add("event_count", rt ? rt->event_count : 0U);
  lane.add("dispatch_count", rt ? rt->dispatch_count : 0U);
  lane.add("irq_count", irq_count);
  lane.add("miss_count", miss_count);
  lane.add("tick_mod_1000", tick_mod_1000);
  lane.add("logical_count32", logical_count32);
  if (rt && rt->has_fired) {
    lane.add("last_event_dwt", rt->last_event.dwt_at_event);
    lane.add("last_event_counter32", rt->last_event.counter32_at_event);
  } else {
    lane.add("last_event_dwt", 0U);
    lane.add("last_event_counter32", 0U);
  }
  parent.add_object(key, lane);
}

static void add_ocxo_lane_summary_object(Payload& parent,
                                         const char* key,
                                         const char* name,
                                         const interrupt_subscriber_runtime_t* rt,
                                         const ocxo_lane_t& lane,
                                         const ocxo_perishable_ring_t& ring) {
  Payload o;
  o.add("lane", name);
  o.add("subscribed", rt ? rt->subscribed : false);
  o.add("active", rt ? rt->active : false);
  o.add("has_fired", rt ? rt->has_fired : false);
  o.add("event_count", rt ? rt->event_count : 0U);
  o.add("dispatch_count", rt ? rt->dispatch_count : 0U);
  o.add("irq_count", lane.irq_count);
  o.add("miss_count", lane.miss_count);
  o.add("tick_mod_1000", lane.tick_mod_1000);
  o.add("logical_count32", lane.logical_count32_at_last_second);
  o.add("cadence_enabled", lane.cadence_enabled);
  o.add("cadence_armed", lane.cadence_armed);
  o.add("cadence_epoch_valid", lane.cadence_epoch_valid);
  o.add("cadence_epoch_counter32", lane.cadence_epoch_counter32);
  o.add("cadence_next_counter32", lane.cadence_next_counter32);
  o.add("cadence_fire_count", lane.cadence_fire_count);
  o.add("cadence_arm_count", lane.cadence_arm_count);
  o.add("cadence_rearm_count", lane.cadence_rearm_count);
  o.add("cadence_false_irq_count", lane.cadence_false_irq_count);
  o.add("cadence_one_second_due_count", lane.cadence_one_second_due_count);
  o.add("cadence_last_service_offset_ticks",
        lane.cadence_last_service_offset_signed_ticks);
  o.add("cadence_last_reason", lane.cadence_last_reason);
  o.add("cadence_last_reason_name",
        ocxo_cadence_reason_name(lane.cadence_last_reason));
  o.add("witness_arm_count", lane.witness_arm_count);
  o.add("witness_fire_count", lane.witness_fire_count);
  o.add("witness_last_counter_delta_ticks", lane.witness_last_counter_delta_ticks);
  o.add("witness_counter_delta_violation_count", lane.witness_counter_delta_violation_count);
  o.add("witness_missed_target_count", lane.witness_missed_target_count);
  o.add("witness_false_irq_count", lane.witness_false_irq_count);
  o.add("witness_late_arm_count", lane.witness_late_arm_count);
  o.add("witness_service_offset_ticks", lane.witness_last_service_offset_signed_ticks);
  o.add("witness_service_class", lane.witness_last_service_class);
  o.add("witness_service_class_name", ocxo_service_class_name(lane.witness_last_service_class));
  o.add("fact_enqueue_count", ring.enqueue_count);
  o.add("fact_drain_count", ring.drain_count);
  o.add("fact_overflow_count", ring.overflow_count);
  o.add("fact_high_water", ring.high_water);
  o.add("fact_asap_fail_count", ring.asap_fail_count);
  if (rt && rt->has_fired) {
    o.add("last_event_dwt", rt->last_event.dwt_at_event);
    o.add("last_event_counter32", rt->last_event.counter32_at_event);
  } else {
    o.add("last_event_dwt", 0U);
    o.add("last_event_counter32", 0U);
  }
  parent.add_object(key, o);
}

static Payload cmd_report_lanes(const Payload&) {
  Payload p;
  p.add("report", "INTERRUPT_LANES");
  p.add("lane_count", 3);
  p.add("detail_command", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");

  add_lane_summary_object(p,
                          "vclock",
                          "VCLOCK",
                          g_rt_vclock,
                          g_vclock_lane.irq_count,
                          g_vclock_lane.miss_count,
                          g_vclock_lane.tick_mod_1000,
                          g_vclock_lane.logical_count32_at_last_second);

  add_ocxo_lane_summary_object(p,
                               "ocxo1",
                               "OCXO1",
                               g_rt_ocxo1,
                               g_ocxo1_lane,
                               g_ocxo1_fact_ring);

  add_ocxo_lane_summary_object(p,
                               "ocxo2",
                               "OCXO2",
                               g_rt_ocxo2,
                               g_ocxo2_lane,
                               g_ocxo2_fact_ring);

  return p;
}

static Payload cmd_report_lane(const Payload& args) {
  const char* lane = args.getString("lane");
  if (!lane || !*lane) lane = args.getString("name");

  Payload p;
  p.add("report", "lane_detail");

  if (!lane || !*lane) {
    p.add("error", "missing lane parameter");
    p.add("usage", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
    return p;
  }

  if (!strcasecmp(lane, "VCLOCK") || !strcasecmp(lane, "VCLK")) {
    add_vclock_lane_payload(p, true);
    return p;
  }

  if (!strcasecmp(lane, "OCXO1") || !strcasecmp(lane, "O1")) {
    add_ocxo_lane_payload(p, g_ocxo1_ctx, true);
    return p;
  }

  if (!strcasecmp(lane, "OCXO2") || !strcasecmp(lane, "O2")) {
    add_ocxo_lane_payload(p, g_ocxo2_ctx, true);
    return p;
  }

  p.add("error", "unknown lane");
  p.add("lane", lane);
  p.add("usage", "INTERRUPT.REPORT_LANE lane=VCLOCK|OCXO1|OCXO2");
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT",          cmd_report          },
  { "REPORT_STATUS",   cmd_report_status   },
  { "REPORT_PPS",      cmd_report_pps      },
  { "REPORT_CADENCE",  cmd_report_cadence  },
  { "REPORT_SMARTZERO", cmd_report_smartzero },
  { "REPORT_BRIDGE",   cmd_report_bridge   },
  { "REPORT_LANES",    cmd_report_lanes    },
  { "REPORT_LANE",     cmd_report_lane     },
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
