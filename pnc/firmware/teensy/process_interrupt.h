// ============================================================================
// process_interrupt.h
// ============================================================================
//
// Interrupt custody and per-lane 1 kHz cadence:
//
//   • OCXO lanes    (QTimer3 vector — CH2, CH3)
//   • VCLOCK lane   (QTimer1 vector — CH3)
//   • TimePop       (QTimer1 vector — CH2, hosted client)
//   • PPS GPIO edge (witness + dispatch authority + epoch anchor)
//
// QTimer1 vector custody:
//
//   QTimer1 has a single shared IRQ vector across all four channels.
//   process_interrupt owns the vector and dispatches in IRQ context:
//     • CH2 flag set → registered TimePop scheduler handler
//     • CH3 flag set → vclock_cadence_isr (process_interrupt internal)
//   Both handlers receive the same first-instruction DWT capture, so
//   they get identical latency profiles.
//
//   TimePop owns the CH2 compare-register programming (advancing the
//   compare target as it schedules slots).  TimePop registers its CH2
//   handler via interrupt_register_qtimer1_ch2_handler at init time.
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
//   never from DWT.  OCXO subscribers and TimePop are the principled
//   exception: they must use the DWT bridge to translate their own
//   ISR captures onto the PPS_VCLOCK timeline.
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
//   The VCLOCK cadence is PPS-anchored.  On the first physical PPS
//   edge after a rebootstrap request, the GPIO ISR captures the
//   QTimer1 CH0+CH1 32-bit count and the QTimer1 CH3 16-bit count
//   immediately after the DWT capture, then re-programs CH3's
//   compare target to fire exactly VCLOCK_INTERVAL_COUNTS ticks later
//   — aligning the VCLOCK cadence with the PPS moment.  From that
//   moment on, every VCLOCK one-second event fires exactly 1 second
//   after the PPS moment, modulo consistent ISR latency.
//
// ─── PPS GPIO edge — three roles ────────────────────────────────────────────
//
//     1. DIAGNOSTIC WITNESS.  Every edge populates a seqlock-protected
//        store with PPS facts and PPS_VCLOCK facts.  Consumers read
//        via interrupt_last_pps(), interrupt_last_pps_vclock(), or
//        the legacy interrupt_last_pps_edge().
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
//   Three lanes (VCLOCK, OCXO1, OCXO2), each cadenced by its own
//   QuadTimer compare channel at 1 kHz.  Identical hardware-to-software
//   latency profile: compare match → NVIC → ISR → DWT capture as first
//   instruction.  The 1 kHz cadence exists because the QuadTimer
//   compare registers are 16-bit — advancing 10,000,000 counts in one
//   shot would overflow many times over.
//
//   Cadence sources:
//     VCLOCK : QTimer1 CH3 compare, +10000 counts/interval (GNSS 10 MHz)
//     OCXO1  : QTimer3 CH2 compare, +10000 counts/interval (OCXO1 10 MHz)
//     OCXO2  : QTimer3 CH3 compare, +10000 counts/interval (OCXO2 10 MHz)
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
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER1_CH2_COMP,
  QTIMER1_CH3_COMP,
  QTIMER3_CH2_COMP,
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
  // (PPS_VCLOCK ruler — ends in "00").  For OCXO/TIMEPOP, derived via the
  // DWT→PPS_VCLOCK bridge (the principled exception to the no-DWT rule).
  // May be 0 for events emitted before the first PPS edge anchors GNSS.
  uint64_t gnss_ns_at_event = 0;

  // Software-extended logical 32-bit count, advances by exactly
  // 10,000,000 per event.  For the VCLOCK lane, re-seeded at PPS
  // rebootstrap to match the PPS_VCLOCK counter32_at_edge — so
  // counter32_at_event - pvc.counter32_at_edge equals exactly
  // N * VCLOCK_COUNTS_PER_SECOND for the Nth event after anchor.
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

  // PPS GPIO witness fields, populated only on the VCLOCK diag.
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
// Compensation for peripheral-bus-read quantization during epoch
// establishment.  When the PPS GPIO ISR captures the 32-bit VCLOCK
// cascade counter, the read completes ~140 CPU cycles (~138 ns) after
// the true PPS edge.  During that window the VCLOCK counter (10 MHz)
// has ticked once, so the counter value we read is the tick AFTER the
// PPS-corresponding tick.
//
// The correction is applied by alpha when it installs its epoch from
// the PPS_VCLOCK snapshot.  process_interrupt's PPS_VCLOCK store
// publishes the counter as read; alpha adjusts.  This split keeps the
// canonical authority for "which counter tick corresponds to PPS"
// inside alpha while leaving the ISR-side store as a record of facts.
//
// Tunable: if live measurements show residuals shifting by more than
// expected after a code change, adjust this value (plausible range:
// -2 to 0).
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

  // VCLOCK 32-bit count and CH3 16-bit count read in the PPS GPIO ISR.
  // These are the physical-edge facts as the QTimer1 cascade saw them.
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

  // Canonical VCLOCK-selected edge expressed in DWT.  Derived from the
  // PPS GPIO ISR's first-instruction _raw capture plus a fixed offset
  // (CANONICAL_VCLOCK_EPOCH_MINUS_RAW_PPS_ISR_CYCLES).
  uint32_t dwt_at_edge       = 0;

  // VCLOCK counter identity of the canonical edge.  In the current
  // build this equals counter32 as read by the ISR (with offset 0).
  uint32_t counter32_at_edge = 0;
  uint16_t ch3_at_edge       = 0;

  // GNSS ns at the canonical edge.  VCLOCK-authored: seeded from the
  // DWT bridge on first acquisition, then advances by exactly 1e9 per
  // PPS edge.  Always ends in "00".
  int64_t  gnss_ns_at_edge   = -1;
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
// New code should consume pps_t / pps_vclock_t via interrupt_last_pps()
// and interrupt_last_pps_vclock().  The misleadingly-named "_raw" fields
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
// PPS GPIO heartbeat accessor
// ============================================================================
//
// Report-only heartbeat counters owned by the PPS GPIO ISR.  These are not
// timing authorities; they let process_witness publish EDGE without owning
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
//   1. If g_epoch_pending is true, install the epoch from snap.
//   2. Stash witness fields into the TIMEBASE_FRAGMENT-bound diag.
//   3. Call clocks_beta_pps() to publish the fragment.
//
// Will be replaced by a PPS_VCLOCK-typed dispatch when alpha migrates.

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
//   • Reprogram CH3's compare to fire VCLOCK_INTERVAL_COUNTS ticks later,
//     aligning the VCLOCK cadence with the PPS moment.
//   • Reset tick_mod_1000 to 0 so the first post-anchor one-second event
//     fires exactly VCLOCK_COUNTS_PER_SECOND ticks after PPS.
//   • Seed logical_count32 from pvc.counter32_at_edge.
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
// Seqlock-safe snapshots of the most recent physical PPS edge.  Either
// accessor returns a consistent view of its own subscription's facts;
// because both are populated from the same GPIO ISR with the same
// sequence number, two reads (one of each) on the same sequence describe
// the same edge.

pps_t        interrupt_last_pps       (void);
pps_vclock_t interrupt_last_pps_vclock(void);

// Legacy accessor — populates pps_edge_snapshot_t from the same internal
// store.  Kept for back-compat with alpha; new code should prefer the
// typed accessors above.
pps_edge_snapshot_t interrupt_last_pps_edge(void);

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

uint16_t interrupt_qtimer1_ch2_counter_now(void);
uint16_t interrupt_qtimer1_ch2_comp1_now(void);
uint16_t interrupt_qtimer1_ch2_csctrl_now(void);

// ============================================================================
// QTimer1 CH3 witness — torture-sanity test of timekeeping math
// ============================================================================
//
// The witness fires on the production VCLOCK CH3 cadence at nine deliberately
// PPS-avoiding sub-second slots (100 ms through 900 ms past the PPS_VCLOCK
// anchor).  At each fire it asks: "what time does the bridge say it is, vs
// what time does VCLOCK say it is?"  The VCLOCK answer is ground truth —
// quantized to 100 ns, ends in "00".  The two bridge rails (time.cpp global
// bridge, and process_interrupt's local dynamic bridge) are evaluated
// against it.
//
// Witness is passive in the current build: production VCLOCK cadence is not
// suspended.  interrupt_witness_arm_first_slot is retained as a stub for
// API compatibility.

enum class interrupt_witness_mode_t : uint8_t {
  OFF = 0,
  ON,
};

struct interrupt_witness_stats_t {
  interrupt_witness_mode_t mode;
  uint64_t n;
  double   mean_ns;
  double   stddev_ns;
  double   stderr_ns;
  int64_t  min_ns;
  int64_t  max_ns;
  uint64_t fires_total;
  uint64_t fires_rejected;

  // CH3-vs-CH0 phase-lag measurement (diagnostic).  Each tick = 100 ns.
  uint64_t lag_n;
  double   lag_mean_ticks;
  double   lag_stddev_ticks;
  int32_t  lag_min_ticks;
  int32_t  lag_max_ticks;

  // GPIO ISR peripheral-bus delay: DWT cycles between the ISR's first-
  // instruction _raw capture and the counter read that populates
  // pps.counter32_at_edge.
  uint64_t gpio_counter_delay_n;
  double   gpio_counter_delay_mean_cycles;
  double   gpio_counter_delay_stddev_cycles;
  int32_t  gpio_counter_delay_min_cycles;
  int32_t  gpio_counter_delay_max_cycles;
};

void                     interrupt_witness_set_mode(interrupt_witness_mode_t mode);
interrupt_witness_mode_t interrupt_witness_get_mode(void);

// Stub retained for API compatibility.  Passive witness no longer arms CH3.
void interrupt_witness_arm_first_slot(uint32_t anchor_counter32,
                                      uint32_t anchor_dwt,
                                      uint32_t anchor_dwt_raw,
                                      uint32_t dwt_cycles_per_second);

interrupt_witness_stats_t interrupt_witness_stats     (void);
void                      interrupt_witness_reset_stats(void);

// HW witness accessors (square-wave source + GPIO/QTimer sinks).
uint32_t interrupt_hw_witness_gpio_delta_cycles            (void);
uint32_t interrupt_hw_witness_qtimer_delta_cycles          (void);
uint32_t interrupt_hw_witness_qtimer_cntr                  (void);
uint32_t interrupt_hw_witness_qtimer_comp1                 (void);
uint32_t interrupt_hw_witness_qtimer_csctrl                (void);
uint32_t interrupt_hw_witness_qtimer_ctrl                  (void);
uint32_t interrupt_hw_witness_qtimer_enbl                  (void);
uint32_t interrupt_hw_witness_qtimer_mux                   (void);
uint32_t interrupt_hw_witness_qtimer_select_input          (void);
uint32_t interrupt_hw_witness_qtimer_source_reads          (void);
uint32_t interrupt_hw_witness_qtimer_nonzero_cntr_observations(void);
uint32_t interrupt_hw_witness_qtimer_cntr_change_hits      (void);
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_at_source   (void);
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_in_irq      (void);

// ============================================================================
// ISR entry points (invoked by vector shims)
// ============================================================================
//
// Parameter is the first-instruction ARM_DWT_CYCCNT capture.  The _raw
// suffix is the visible mark of the discipline's only legitimate hold:
// this value is converted into event coordinates inside the function and
// never propagated.

void process_interrupt_gpio6789_irq  (uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t isr_entry_dwt_raw);

// ============================================================================
// Counter accessors (diagnostic, free-running)
// ============================================================================

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now (void);

// ============================================================================
// Dynamic DWT cycles per GNSS second
// ============================================================================
//
// The PPS_VCLOCK-phased VCLOCK cadence ISR observes ARM_DWT_CYCCNT at each
// 1 ms tick and refines an estimate of DWT cycles per GNSS second.  Anchor
// is pvc.dwt_at_edge — the canonical PPS_VCLOCK DWT.  At each PPS edge the
// estimate is reseeded from the two-bookend measurement
// (pvc.dwt_at_edge[N] - pvc.dwt_at_edge[N-1]).  Between edges it is
// refined per tick by blending toward the inferred CPS implied by the
// observed mid-second DWT displacement.

uint32_t interrupt_dynamic_cps(void);

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str  (interrupt_provider_kind_t   provider);
const char* interrupt_lane_str           (interrupt_lane_t            lane);