// ============================================================================
// process_interrupt.h
// ============================================================================
//
// Interrupt custody and low-word counter cadence:
//
//   • Cadence minder (single TimePop client tending all 16-bit counters)
//   • OCXO lanes    (QTimer2 CH0 for OCXO1, QTimer3 CH3 for OCXO2; currently still event rails)
//   • VCLOCK lane   (critical recurring TimePop client on QTimer1 CH2)
//   • TimePop       (QTimer1 CH2, hosted scheduler/client rail)
//   • PPS GPIO edge (diagnostics + dispatch authority + epoch anchor)
//
// QTimer1 vector custody:
//
//   QTimer1 has a single shared IRQ vector across all four channels.
//   process_interrupt owns the vector and dispatches in IRQ context:
//     • CH1 flag set → legacy hosted compare rail, if explicitly armed
//     • CH2 flag set → registered TimePop scheduler handler
//   VCLOCK cadence no longer owns a private QTimer1 compare channel.  It is
//   a critical recurring TimePop slot that runs from the CH2 shared fire facts
//   and rearms inside the CH2 ISR pass before TimePop chooses the next compare.
//
//   TimePop owns scheduler policy only.  process_interrupt owns the CH2
//   compare-register programming; TimePop requests target updates through
//   interrupt_qtimer1_ch2_arm_compare().  TimePop registers its CH2 handler
//   via interrupt_register_qtimer1_ch2_handler at init time.
//
// ─── PPS / PPS_VCLOCK doctrine ──────────────────────────────────────────────
//
//   Two distinct subscriptions describe the same physical PPS event:
//
//     • PPS         — physical GPIO edge facts.  The DWT/counter32/ch3
//                     read in the PPS GPIO ISR, latency-adjusted into
//                     event coordinates that represent the actual
//                     electrical PPS moment.  Used for diagnostics,
//                     audit, and any rail that needs the truth about
//                     the pulse itself.
//
//     • PPS_VCLOCK  — the VCLOCK edge selected by the physical PPS
//                     pulse.  This is the canonical timing authority
//                     of the system.  Its counter32/ch3 are the
//                     VCLOCK-counter identity of the chosen edge; its
//                     dwt is that same edge's DWT coordinate; its
//                     gnss_ns is VCLOCK-authored (advances by exactly
//                     1e9 per PPS) and is therefore quantized to 100
//                     ns — the ns value always ends in "00".
//
//   PPS_VCLOCK ns is computed from VCLOCK counter ticks × 100 ns,
//   never from DWT.  TimePop uses the DWT bridge for CH2 fire diagnostics.
//   OCXO subscribers receive authored OCXO edge DWT/counter facts even when
//   bridge projection is unavailable; CLOCKS/Alpha owns OCXO measured-GNSS
//   interval construction from consecutive OCXO edge DWTs.
//
//   The _raw rule is absolute: _raw is reserved for one thing only —
//   ARM_DWT_CYCCNT captured as the first instruction of an ISR.  The
//   moment a value is latency-adjusted, the _raw is gone.  _raw values
//   do NOT propagate.  They live in the ISR stack frame and die there.
//   Nothing in any data structure, subscription payload, or TIMEBASE
//   fragment carries _raw.  ISR entry parameters retain the suffix as
//   the visible mark of where the discipline begins.
//
// ─── PPS-anchored VCLOCK cadence ────────────────────────────────────────────
//
//   The VCLOCK cadence is PPS-anchored but no longer uses QTimer1 CH3.
//   On the first physical PPS edge after a rebootstrap request, the GPIO ISR
//   captures the QTimer1 CH0 low-word count and selects the sacred VCLOCK edge.
//   The already-running VCLOCK_CADENCE TimePop client consumes a CH2 shared
//   fire fact, back-projects from that cadence event to the selected edge, and
//   publishes the canonical PPS_VCLOCK epoch.
//
//   VCLOCK_CADENCE is armed with TimePop's critical recurring ISR mode.  During
//   PPS rebootstrap, process_interrupt re-arms that cadence from the selected
//   PPS_VCLOCK GNSS base using TimePop's anchored recurring API, so the cadence
//   grid is:
//
//       base_gnss_ns + k * 1 ms
//
//   rather than an inherited stale scheduler phase.  Its callback refreshes the
//   process_interrupt-owned synthetic VCLOCK low-word anchor inside the CH2 ISR
//   pass, before TimePop calls schedule_next().  This prevents TimePop from
//   scheduling future compares from a stale synthetic VCLOCK anchor while
//   preserving the single-compare-rail architecture.
//
// ─── PPS GPIO edge — three roles ────────────────────────────────────────────
//
//     1. DIAGNOSTIC SNAPSHOT.  Every edge populates a seqlock-protected
//        store with PPS facts and PPS_VCLOCK facts.  Consumers read
//        via interrupt_last_pps_vclock() or the legacy
//        interrupt_last_pps_edge().
//
//     2. DISPATCH AUTHORITY FOR TIMEBASE_FRAGMENT.  On every edge, the
//        GPIO ISR arms timepop_arm_asap to invoke a single registered
//        dispatch callback in foreground context.  That callback
//        publishes TIMEBASE_FRAGMENT.
//
//     3. EPOCH ANCHOR.  When a PPS rebootstrap is pending (armed by
//        alpha), the next PPS edge re-aligns the VCLOCK cadence to
//        the PPS moment and seeds the lane's logical counter from
//        the PPS_VCLOCK counter32.
//
// ─── Per-lane cadence mechanics ─────────────────────────────────────────────
//
//   Three lanes (VCLOCK, OCXO1, OCXO2) produce one-second subscriber events.
//   A single TimePop cadence minder now refreshes all process_interrupt-owned
//   synthetic 32-bit counter anchors so 16-bit low-word rollover custody is
//   centralized on the sovereign VCLOCK/TimePop rail.  OCXO compare channels
//   are still present as event rails in this step, but rollover custody no
//   longer depends on frequent OCXO interrupt service.  VCLOCK cadence is a
//   TimePop critical recurring ISR slot on QTimer1 CH2, sharing fire facts
//   with other same-deadline TimePop clients.
//
//   Cadence sources:
//     VCLOCK : TimePop critical recurring ISR slot, +10000 counts/interval
//              in the GNSS-disciplined 10 MHz VCLOCK domain.  QTimer1 CH0 is
//              the passive low-word VCLOCK counter; QTimer1 CH2 is the only
//              active TimePop compare rail.
//     MINDER : TimePop recurring 1 ms client, tends VCLOCK/OCXO low-word counters
//     OCXO1  : QTimer2 CH0 compare, event rail retained for current one-second events
//     OCXO2  : QTimer3 CH3 compare, event rail retained for current one-second events
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

// Compatibility shim — retired but still referenced by loop() plumbing.
void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// Subscriber identities
// ============================================================================
//
// A subscriber is a one-second-event recipient, not a hardware event source.
// VCLOCK, OCXO1, OCXO2 are the three one-second-event-emitting subscribers.
// The PPS GPIO edge is handled separately — see pps_edge_dispatch_fn and the
// PPS / PPS_VCLOCK accessors below.

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  VCLOCK,
  OCXO1,
  OCXO2,
  TIMEPOP,
};

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER1,
  QTIMER2,
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER1_CH1_COMP,
  QTIMER1_CH2_COMP,
  QTIMER1_CH3_COMP,
  QTIMER2_CH0_COMP,
  QTIMER3_CH3_COMP,
  GPIO_EDGE,
};

enum class interrupt_event_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

// ============================================================================
// One-second event — VCLOCK / OCXO subscribers (every 1000 ticks)
// CH2 cadence event — TimePop (every QTimer1 CH2 compare-match)
// ============================================================================
//
// VCLOCK and OCXO lanes deliver one event per second to their foreground
// subscribers (via timepop_arm_asap deferral).  TIMEPOP receives an event
// on every CH2 compare-match — IRQ-context delivery, no foreground hop.

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind     = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t   provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t            lane     = interrupt_lane_t::NONE;
  interrupt_event_status_t    status   = interrupt_event_status_t::OK;

  // Latency-adjusted DWT coordinate of the lane's 1 kHz compare event on the
  // 1000th tick.  Derived from the ISR's first-instruction _raw capture.
  uint32_t dwt_at_event = 0;

  // GNSS ns at the event.  For VCLOCK, authored from the VCLOCK counter
  // (PPS_VCLOCK ruler — ends in "00").  For TimePop, derived via the
  // DWT→PPS_VCLOCK bridge.  For OCXO, this is diagnostic-only and may be 0;
  // Alpha measures OCXO intervals from consecutive OCXO edge DWT facts.
  uint64_t gnss_ns_at_event = 0;

  // process_interrupt-authored private synthetic 32-bit clock identity at
  // this event.  This is an event fact, delivered only through the interrupt
  // callback path.  It is derived from the corresponding 64-bit nanosecond
  // ledger as (ns / 100) mod 2^32 and advanced in exact 10 MHz lockstep.
  uint32_t counter32_at_event = 0;

  // ── PPS / VCLOCK coincidence measurement ──
  //
  // Populated only for VCLOCK-kind one-second events.  Measures the
  // raw DWT-cycles difference between THIS ISR's event coordinate and
  // the most recent PPS_VCLOCK edge:
  //
  //     pps_coincidence_cycles = dwt_at_event − pvc.dwt_at_edge
  //
  // Under PPS-anchored operation the CH3 one-second compare-match and
  // the PPS GPIO edge are the same physical moment at the receiver,
  // modulo sub-nanosecond propagation.  Steady-state cycles is a small
  // positive number dominated by GPIO ISR body duration plus CH3 entry
  // latency.
  //
  // pps_coincidence_valid is true iff:
  //   • At least one PPS edge has been captured (pvc.sequence > 0)
  //   • cycles is non-negative
  //   • cycles < DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES
  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;
};

// ============================================================================
// Diagnostic surface — carried alongside every one-second event
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t   provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t            lane     = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind     = interrupt_subscriber_kind_t::NONE;

  // Event truth surface — mirrors interrupt_event_t for consumers that
  // only hold a diag pointer.
  uint32_t dwt_at_event       = 0;
  uint64_t gnss_ns_at_event   = 0;
  uint32_t counter32_at_event = 0;

  // DWT admissibility / repair audit.  dwt_at_event remains the authoritative
  // coordinate.  While repair is diagnostic-only, dwt_synthetic is false and
  // dwt_repair_candidate indicates that the observed endpoint would have been
  // replaced if operational repair were enabled.
  bool     dwt_synthetic = false;
  bool     dwt_repair_candidate = false;
  uint32_t dwt_original_at_event = 0;
  uint32_t dwt_predicted_at_event = 0;
  uint32_t dwt_used_at_event = 0;
  int32_t  dwt_synthetic_error_cycles = 0;
  uint32_t dwt_synthetic_threshold_cycles = 0;
  const char* dwt_synthetic_reason = nullptr;

  // DWT→PPS_VCLOCK anchor-selection diagnostics.
  // Populated for OCXO and TIMEPOP events whose GNSS coordinate is derived
  // by projecting an event DWT coordinate onto the PPS_VCLOCK timeline.
  // anchor_age_slots: 0=latest anchor, 1=previous, 2+=older ring entry.
  // anchor_selection_kind: 0=NONE, 1=LATEST, 2=PREVIOUS, 3=OLDER, 4=FAILED.
  uint32_t anchor_sequence_used = 0;
  uint32_t anchor_age_slots = 0;
  uint32_t anchor_selection_kind = 0;
  uint32_t anchor_dwt_at_edge = 0;
  int64_t  anchor_gnss_ns_at_edge = -1;
  uint32_t anchor_cps = 0;
  uint64_t anchor_ns_delta = 0;
  uint32_t anchor_failure_mask = 0;

  // PPS GPIO audit fields, populated only on the VCLOCK diag.
  //
  // pps_edge_dwt_isr_entry_raw is a transitional field name that
  // carries pvc.dwt_at_edge (event-coordinate, NOT _raw).  The name is
  // legacy; do not interpret it as raw ISR-entry DWT.  Will be renamed
  // when alpha migrates to the PPS_VCLOCK subscription.
  uint32_t pps_edge_sequence          = 0;
  uint32_t pps_edge_dwt_isr_entry_raw = 0;   // carries pvc.dwt_at_edge
  int64_t  pps_edge_gnss_ns           = -1;
  int64_t  pps_edge_minus_event_ns    = 0;

  // Bridge-independent VCLOCK-to-edge phase diagnostics — pure ISR-
  // captured facts plus integer arithmetic.  Under PPS-anchored
  // operation, pps_edge_ns_from_vclock should stay near zero.
  uint32_t pps_edge_dwt_cycles_from_vclock = 0;
  int64_t  pps_edge_ns_from_vclock         = 0;
  uint32_t pps_edge_vclock_event_count     = 0;

  // PPS / VCLOCK coincidence mirror (see interrupt_event_t).
  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;

  // OCXO compare-service diagnostics.
  // Populated only for OCXO1/OCXO2 one-second witness events.  These are the
  // compact fields needed to correlate TIMEBASE residuals with QTimer compare
  // service timing without carrying the entire lane-report surface in every
  // fragment.
  uint32_t ocxo_service_class = 0;
  int32_t  ocxo_service_offset_signed_ticks = 0;
  uint32_t ocxo_service_offset_abs_ticks = 0;
  uint32_t ocxo_interpreted_late_ticks = 0;
  uint32_t ocxo_early_ticks = 0;
  uint32_t ocxo_target_delta_mod65536_ticks = 0;
  uint32_t ocxo_arm_remaining_ticks = 0;
  uint32_t ocxo_arm_to_isr_ticks = 0;
  uint32_t ocxo_arm_to_isr_dwt_cycles = 0;

  uint32_t anomaly_count = 0;
};

// ============================================================================
// PPS / VCLOCK coincidence threshold
// ============================================================================
//
// When the VCLOCK one-second compare-match fires, the ISR reads the most
// recent PPS_VCLOCK snapshot and computes the DWT-cycles difference.  If
// the difference is in [0, DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES) the two
// events are declared coincident and the measurement is published.
//
// 10,000 cycles at 1.008 GHz ≈ 10 µs.  Steady state is ~300–600 cycles
// (GPIO ISR body + entry latencies); this threshold cleanly excludes the
// "PPS was seconds ago" case while leaving generous headroom.
static constexpr int32_t DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES = 10000;

// ============================================================================
// VCLOCK epoch tick offset (alpha-side correction)
// ============================================================================
//
// Retired compatibility constant from the former hardware-cascade epoch
// correction path.  process_interrupt now publishes a private synthetic
// VCLOCK counter32 identity; alpha consumes that identity verbatim.
static constexpr int32_t VCLOCK_EPOCH_TICK_OFFSET = -2;

// ============================================================================
// PPS — physical GPIO edge facts
// ============================================================================
//
// The latency-adjusted truth about the physical PPS pulse.  All values
// are event coordinates; no _raw values are carried here.

struct pps_t {
  uint32_t sequence          = 0;

  // Physical PPS DWT, latency-adjusted from the GPIO ISR's first-
  // instruction _raw capture.  Represents the electrical PPS moment.
  uint32_t dwt_at_edge       = 0;

  // process_interrupt-authored synthetic VCLOCK counter32 identity and
  // QTimer1 low-word phase observed in the PPS GPIO ISR.
  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;
};

// ============================================================================
// PPS_VCLOCK — canonical VCLOCK-selected edge
// ============================================================================
//
// The canonical timing authority.  The VCLOCK edge selected by the physical
// PPS pulse.  All values are event coordinates; gnss_ns_at_edge is VCLOCK-
// authored and quantized to 100 ns (ends in "00").
//
// PPS_VCLOCK math discipline: ns derived from counter ticks × 100, never
// from DWT.  The DWT bridge is permitted only for OCXO and TimePop, which
// must translate their own ISR captures onto this timeline.

struct pps_vclock_t {
  uint32_t sequence          = 0;

  // Canonical VCLOCK-selected edge expressed in DWT.  During rebootstrap this
  // may be back-projected from the VCLOCK cadence rail; steady-state bookends
  // are authored directly by the VCLOCK/TimePop cadence event path.
  uint32_t dwt_at_edge       = 0;

  // process_interrupt-authored synthetic VCLOCK counter identity of the
  // canonical edge.
  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;

  // GNSS ns at the canonical edge.  VCLOCK-authored: seeded from the
  // DWT bridge on first acquisition, then advances by exactly 1e9 per
  // PPS edge.  Always ends in "00".
  int64_t  gnss_ns_at_edge   = -1;
};

// ============================================================================
// Experimental PPS-witness-derived PPS_VCLOCK DWT phase estimate
// ============================================================================
//
// The canonical PPS_VCLOCK identity remains VCLOCK/counter32-authored.  This
// report surface estimates the selected VCLOCK edge's DWT coordinate using the
// physical PPS DWT witness as the smoother phase ruler and the VCLOCK lattice
// DWT as identity evidence.  All math is integer/fixed-point DWT-cycle math;
// this surface never asks the DWT-GNSS bridge for a timestamp.

struct pps_vclock_phase_estimate_t {
  bool     valid = false;

  uint32_t lattice_dwt_at_edge = 0;
  uint32_t estimated_dwt_at_edge = 0;
  int32_t  correction_cycles = 0;

  uint32_t phase_mod_scaled_cycles = 0;
  uint32_t tick_scaled_cycles = 0;
  uint32_t scale = 0;
  uint32_t dwt_cycles_per_second = 0;

  uint32_t pps_sequence = 0;
  uint32_t pvc_sequence = 0;
  uint32_t pps_dwt_at_edge = 0;
  uint32_t pps_counter32_at_edge = 0;
  uint32_t pvc_counter32_at_edge = 0;
};

// ============================================================================
// Legacy: pps_edge_snapshot_t — transitional projection
// ============================================================================
//
// Carried for backward compatibility while alpha continues to consume the
// flat snapshot.  Will be retired when alpha migrates to the PPS_VCLOCK
// subscription.
//
// Field map (populated by process_interrupt.cpp):
//
//   sequence           ← pvc.sequence
//   dwt_at_edge        ← pvc.dwt_at_edge          (PPS_VCLOCK)
//   dwt_raw_at_edge    ← pvc.dwt_at_edge          (legacy alias; NOT _raw
//                                                  — name is misleading,
//                                                  retained for API)
//   counter32_at_edge  ← pvc.counter32_at_edge    (PPS_VCLOCK)
//   ch3_at_edge        ← pvc.ch3_at_edge          (PPS_VCLOCK)
//   gnss_ns_at_edge    ← pvc.gnss_ns_at_edge      (PPS_VCLOCK; ends "00")
//
//   physical_pps_dwt_raw_at_edge        ← pps.dwt_at_edge
//                                         (legacy field name; carries
//                                          event-coordinate pps DWT)
//   physical_pps_dwt_normalized_at_edge ← pps.dwt_at_edge
//   physical_pps_counter32_at_read      ← pps.counter32_at_edge
//   physical_pps_ch3_at_read            ← pps.ch3_at_edge
//
//   vclock_epoch_*  — audit metadata (constants describing the offsets
//                     used when deriving PPS_VCLOCK from PPS).
//
// New code should consume pps_vclock_t via interrupt_last_pps_vclock().  The misleadingly-named "_raw" fields
// here will be renamed when alpha migrates.

struct pps_edge_snapshot_t {
  uint32_t sequence          = 0;

  // Canonical (PPS_VCLOCK) values.
  uint32_t dwt_at_edge       = 0;
  uint32_t dwt_raw_at_edge   = 0;   // legacy alias; carries pvc.dwt_at_edge
  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;
  int64_t  gnss_ns_at_edge   = -1;

  // Physical PPS GPIO audit values (legacy field naming).
  uint32_t physical_pps_dwt_raw_at_edge        = 0;   // carries pps.dwt_at_edge
  uint32_t physical_pps_dwt_normalized_at_edge = 0;   // carries pps.dwt_at_edge
  uint32_t physical_pps_counter32_at_read      = 0;
  uint16_t physical_pps_ch3_at_read            = 0;

  // VCLOCK epoch derivation audit.
  uint32_t vclock_epoch_counter32              = 0;
  uint16_t vclock_epoch_ch3                    = 0;
  uint32_t vclock_epoch_ticks_after_pps        = 0;
  int32_t  vclock_epoch_counter32_offset_ticks = 0;
  int32_t  vclock_epoch_dwt_offset_cycles      = 0;
  bool     vclock_epoch_selected               = false;
};

// ============================================================================
// Epoch-ready PPS capture packet
// ============================================================================
//
// Authored on every PPS GPIO ISR from the ISR opening custody window.  Raw
// 16-bit hardware counter reads are retained here only as forensic evidence.
// Runtime math consumes the process_interrupt-authored synthetic 32-bit lane
// coordinates below; ZERO can select this packet asynchronously without waiting
// for the next PPS edge.

struct interrupt_epoch_capture_t {
  bool     valid = false;
  uint32_t sequence = 0;

  uint32_t capture_dwt_start_raw = 0;
  uint32_t capture_dwt_after_vclock_raw = 0;
  uint32_t capture_dwt_end_raw = 0;
  uint32_t capture_window_cycles = 0;
  uint32_t vclock_read_offset_cycles = 0;

  // Latency-adjusted DWT coordinate of the selected PPS_VCLOCK edge authored
  // from this same PPS ISR capture.  CLOCKS ZERO consumes this field directly;
  // it does not require the previous PPS_VCLOCK snapshot store to have already
  // advanced to this sequence.
  uint32_t vclock_dwt_at_edge = 0;

  // vclock_capture_valid is the operational requirement.  If VCLOCK is wrong,
  // the packet cannot be used as a clock epoch source.  all_lanes_capture_valid
  // is the quality gate for installing OCXO zero offsets from this packet.
  bool     vclock_capture_valid = false;
  bool     all_lanes_capture_valid = false;

  // Forensic-only raw/selected low-word captures from the PPS custody window.
  // These are never timing authority; they let reports prove how the synthetic
  // zero-offset coordinates below were born.
  uint16_t vclock_hardware16_observed = 0;
  uint16_t vclock_hardware16_selected = 0;
  uint16_t ocxo1_hardware16 = 0;
  uint16_t ocxo2_hardware16 = 0;

  // Synthetic 32-bit coordinates selected as per-lane zero-offset ticks by
  // CLOCKS.  CLOCKS subtracts these from later event counter32 values and then
  // extends the deltas into 64-bit logical tick ledgers.
  uint32_t vclock_counter32 = 0;
  uint32_t ocxo1_counter32 = 0;
  uint32_t ocxo2_counter32 = 0;
};

bool interrupt_last_epoch_capture(interrupt_epoch_capture_t* out);

// Re-author OCXO QuadTimer witness targets from CLOCKS phase-authored zero.
//
// OCXO lanes are not tandem-zeroed with VCLOCK.  CLOCKS/Alpha chooses crafted
// OCXO zero-offset ticks such that the OCXO local coordinates are already
// non-zero at PPS/VCLOCK zero.  It then passes the first physical witness
// targets to process_interrupt:
//
//   OCXO1: PPS/VCLOCK zero + 250,500 us
//   OCXO2: PPS/VCLOCK zero + 750,500 us
//
// If the requested first target is already behind the current passive OCXO
// observation, the cadence minder advances the target by exact 10 MHz seconds
// until the same phase is future-reachable.  After each witnessed OCXO edge,
// the ISR advances the lane target by exactly 10,000,000 ticks.
void interrupt_ocxo_phase_grid_epoch(uint32_t ocxo1_first_target_counter32,
                                     uint32_t ocxo2_first_target_counter32);


// ============================================================================
// PPS GPIO heartbeat accessor
// ============================================================================
//
// Report-only heartbeat counters owned by the PPS GPIO ISR.  These are not
// timing authorities; they let the EDGE report publish without owning
// or duplicating PPS/PPS_VCLOCK authorship.

struct interrupt_pps_edge_heartbeat_t {
  uint32_t edge_count = 0;
  uint32_t last_dwt = 0;       // pps_vclock.dwt_at_edge of most recent edge
  int64_t  last_gnss_ns = -1;  // pps_vclock.gnss_ns_at_edge of most recent edge
  uint32_t gpio_irq_count = 0;
  uint32_t gpio_miss_count = 0;
};

interrupt_pps_edge_heartbeat_t interrupt_pps_edge_heartbeat(void);

// ============================================================================
// Subscription and event callback (VCLOCK + OCXO)
// ============================================================================

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t* diag,
             void* user_data);

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t  kind     = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event = nullptr;
  void*                         user_data = nullptr;
};

// ============================================================================
// PPS edge dispatch callback (legacy)
// ============================================================================
//
// Invoked from foreground context (via timepop_arm_asap) exactly once per
// physical PPS edge.  The snapshot is a stable copy of the state captured
// in the GPIO ISR.  The registered callback's responsibilities:
//
//   1. Stash audit fields into the TIMEBASE_FRAGMENT-bound diag.
//   2. Call clocks_beta_pps() to publish the fragment.
//
// Epoch ZERO no longer depends on this dispatch path; clocks code selects the
// already-authored interrupt_epoch_capture_t when it needs an asynchronous
// logical epoch.  This callback will be replaced by a PPS_VCLOCK-typed dispatch
// when alpha migrates.

using pps_edge_dispatch_fn = void (*)(const pps_edge_snapshot_t& snap);

// ============================================================================
// Public API
// ============================================================================

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

// ── PPS-anchored epoch installation ──
//
// Alpha calls interrupt_request_pps_rebootstrap() when it needs a fresh
// PPS-anchored epoch (at boot, ZERO, START).  The next PPS GPIO edge will:
//   • Capture the VCLOCK counter at ISR entry.
//   • Select the sacred VCLOCK edge associated with PPS.
//   • Re-arm the critical TimePop VCLOCK_CADENCE client from the selected
//     PPS_VCLOCK GNSS base.
//   • Let that anchored cadence client back-project from its next CH2 shared
//     fire fact to the selected edge.
//   • Reset tick_mod_1000 so the first post-anchor one-second event lands on
//     the PPS/VCLOCK boundary.
//   • Seed the private synthetic VCLOCK counter32 from pvc.counter32_at_edge.
//   • Clear the rebootstrap pending flag.
void interrupt_request_pps_rebootstrap(void);
bool interrupt_pps_rebootstrap_pending(void);

const interrupt_event_t*        interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag (interrupt_subscriber_kind_t kind);

// Register the single dispatch callback for PPS GPIO edges.  Only one
// callback may be registered; later registrations replace the prior.
// Called once at init by process_clocks_alpha.
void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn);

// ============================================================================
// PPS / PPS_VCLOCK accessors
// ============================================================================
//
// Seqlock-safe snapshot of the most recent canonical PPS/VCLOCK edge.
// Physical PPS GPIO DWT is deliberately not exposed as a timing accessor;
// it remains a witness/audit fact inside process_interrupt and the legacy
// diagnostic snapshot only.

pps_vclock_t interrupt_last_pps_vclock(void);

bool interrupt_last_pps_vclock_phase_estimate(
    pps_vclock_phase_estimate_t* out);

// Legacy accessor — populates pps_edge_snapshot_t from the same internal
// store.  Kept for back-compat with alpha; new code should prefer the
// typed accessors above.
pps_edge_snapshot_t interrupt_last_pps_edge(void);

// ============================================================================
// PPS GPIO ISR-entry listener — hosted diagnostics hook for process_witness
// ============================================================================
//
// Called from the PPS GPIO ISR with the first-instruction raw DWT capture and
// the PPS edge sequence being authored.  This hook is for entry-latency
// diagnostics only; it must remain tiny and must not touch hardware.

using interrupt_pps_entry_latency_handler_fn =
    void (*)(uint32_t sequence, uint32_t isr_entry_dwt_raw);

void interrupt_register_pps_entry_latency_handler(
    interrupt_pps_entry_latency_handler_fn cb);

// ============================================================================
// QTimer1 CH1 compare service — hosted hardware rail for process_witness
// ============================================================================
//
// CH1 is a VCLOCK-domain compare rail owned by process_interrupt. Clients do
// not touch QTimer1 registers. A client registers a single IRQ-context handler
// and asks process_interrupt to arm a synthetic 32-bit VCLOCK target.
// process_interrupt advances any required 16-bit compare hops internally and
// invokes the handler only at the requested target event.
//
// The same CH1 rail is also used internally once per PPS as a near-future
// VCLOCK phase probe. process_interrupt arbitrates these targets so hosted
// callers do not directly fight the PPS probe.

struct interrupt_qtimer1_ch1_compare_event_t {
  uint32_t sequence = 0;
  uint32_t target_counter32 = 0;
  uint32_t counter32_at_event = 0;
  int32_t  counter32_residual_ticks = 0;

  // First-instruction ISR-entry DWT, before latency normalization.
  // Exposed so process_witness can measure true interrupt entry latency
  // from a foreground spin-shadow without guessing the correction constant.
  uint32_t isr_entry_dwt_raw = 0;

  // Latency-adjusted QTimer event coordinate.  This remains the normal
  // event fact used by BRIDGE and other timing reports.
  uint32_t dwt_at_event = 0;
  int64_t  gnss_ns_at_event = -1;
};

using interrupt_qtimer1_ch1_handler_fn =
    void (*)(const interrupt_qtimer1_ch1_compare_event_t& event);

void interrupt_register_qtimer1_ch1_handler(interrupt_qtimer1_ch1_handler_fn cb);
bool interrupt_qtimer1_ch1_arm_compare(uint32_t target_counter32);
void interrupt_qtimer1_ch1_disable_compare(void);

uint16_t interrupt_qtimer1_ch1_counter_now(void);
uint16_t interrupt_qtimer1_ch1_comp1_now(void);
uint16_t interrupt_qtimer1_ch1_csctrl_now(void);

// ============================================================================
// QTimer1 CH2 IRQ-context handler — TimePop scheduler heartbeat
// ============================================================================
//
// TimePop registers a single CH2 handler at init.  process_interrupt's
// QTimer1 ISR captures DWT as the first instruction, then on every CH2
// compare-match assembles a standard interrupt_event_t and
// interrupt_capture_diag_t (kind = TIMEPOP) and invokes this handler in
// IRQ context.  Unlike VCLOCK/OCXO subscriptions, CH2 delivery is NOT
// foreground-deferred — TimePop IS the foreground dispatcher.
//
// Pass nullptr to unregister.  Only one CH2 handler may be registered.

using interrupt_qtimer1_ch2_handler_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t& diag);

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb);

// ============================================================================
// TimePop hardware service API — process_interrupt owns QTimer1 CH2 hardware
// ============================================================================
//
// TimePop remains the scheduling policy engine, but it no longer programs
// QTimer1 CH2 or reads the VCLOCK counter directly.  It asks process_interrupt
// to perform hardware actions and to provide explicitly ambient scheduling
// observations.  These observations are NOT event facts.

uint32_t interrupt_vclock_counter32_observe_ambient(void);
void     interrupt_qtimer1_ch2_arm_compare(uint32_t target_counter32);

// ============================================================================
// Private synthetic clock32 API
// ============================================================================
//
// The synthetic 32-bit QTimer identity for each clock domain is owned by
// process_interrupt because ISR callbacks need counter32_at_event immediately.
// Higher layers interpret those event coordinates; they do not write raw
// hardware counters.  The former process_epoch setter API has been retired.

struct interrupt_clock_snapshot_t {
  uint16_t hardware16 = 0;
  uint32_t counter32 = 0;
  uint64_t ns64 = 0;
};

bool interrupt_clock_snapshot(interrupt_subscriber_kind_t kind,
                              interrupt_clock_snapshot_t* out);

uint32_t interrupt_clock32_from_ns(uint64_t ns);
bool     interrupt_clock32_zero_from_ns(interrupt_subscriber_kind_t kind,
                                        uint64_t ns);
bool     interrupt_clock32_request_zero_from_ns(interrupt_subscriber_kind_t kind,
                                                uint64_t ns);

uint16_t interrupt_qtimer1_ch2_counter_now(void);
uint16_t interrupt_qtimer1_ch2_comp1_now(void);
uint16_t interrupt_qtimer1_ch2_csctrl_now(void);

// ============================================================================
// ISR entry points (invoked by vector shims)
// ============================================================================
//
// Parameter is the first-instruction ARM_DWT_CYCCNT capture.  The _raw
// suffix is the visible mark of the discipline's only legitimate hold:
// this value is converted into event coordinates inside the function and
// never propagated.

void process_interrupt_gpio6789_irq  (uint32_t isr_entry_dwt_raw);

// ============================================================================
// Counter accessors (diagnostic, free-running)
// ============================================================================

uint16_t interrupt_qtimer2_ch0_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now (void);

// ============================================================================
// Dynamic DWT cycles per GNSS second — compatibility accessor
// ============================================================================
//
// Compatibility accessor for the current DWT cycles-per-GNSS-second estimate.
// process_interrupt no longer asks process_time for this value; it mirrors the
// best CLOCKS/Gamma-owned VCLOCK prediction available to the interrupt layer.

uint32_t interrupt_dynamic_cps(void);

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str  (interrupt_provider_kind_t   provider);
const char* interrupt_lane_str           (interrupt_lane_t            lane);