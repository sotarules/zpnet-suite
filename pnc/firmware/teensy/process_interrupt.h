// ============================================================================
// process_interrupt.h — interrupt custody + per-lane 1 kHz cadence
// ============================================================================
//
// process_interrupt owns interrupt custody for:
//   • OCXO lanes (QTimer3 vector — CH2, CH3)
//   • VCLOCK lane (QTimer1 vector — CH3)
//   • TimePop scheduler (QTimer1 vector — CH2, hosted client)
//   • Physical PPS GPIO edge (witness + dispatch authority + epoch anchor)
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
//   compare target as it schedules slots).  TimePop registers its
//   CH2 handler via interrupt_register_qtimer1_ch2_handler at init
//   time; the registration is a pure function-pointer assignment with
//   no hardware interaction.
//
// Doctrine — canonical GNSS clock authority:
//
//   The GNSS ns timeline is VCLOCK-driven.  The VCLOCK lane delivers one
//   event per second to alpha's vclock_callback, which advances the
//   bridge anchor and all canonical clock state.
//
//   The VCLOCK cadence itself is PPS-anchored.  On the first physical
//   PPS edge after a rebootstrap request, the GPIO ISR captures the
//   QTimer1 CH0+CH1 32-bit count AND the QTimer1 CH3 16-bit count as
//   its first instructions (after the DWT capture), then re-programs
//   CH3's compare target to fire exactly VCLOCK_INTERVAL_COUNTS ticks
//   later — aligning the VCLOCK cadence with the physical PPS moment.
//
//   From that moment on, every VCLOCK one-second event fires exactly
//   1 second after the PPS moment, and every Nth one-second event
//   fires at QTimer1 count C0 + N*VCLOCK_COUNTS_PER_SECOND where C0 is
//   the QTimer1 count captured at the PPS ISR.
//
//   This is the mechanism that gives "pulse identity" to VCLOCK cycles:
//   before PPS anchor, "pulse N" is whatever tick the counter happened
//   to label N after boot — arbitrary.  After PPS anchor, "pulse N" is
//   the tick that coincided (to GPIO-ISR latency) with the physical
//   PPS second boundary.
//
// Doctrine — physical PPS edge:
//
//   The physical PPS GPIO edge plays three narrow roles:
//
//     1. DIAGNOSTIC WITNESS.  Every edge populates a seqlock-protected
//        pps_edge_snapshot_t with ISR-captured facts: sequence, DWT,
//        QTimer1 CH0+CH1 (counter32), QTimer1 CH3, and the bridge-
//        derived gnss_ns_at_edge.  Consumers read via
//        interrupt_last_pps_edge().
//
//     2. DISPATCH AUTHORITY FOR TIMEBASE_FRAGMENT.  On every edge, the
//        GPIO ISR arms timepop_arm_asap to invoke a single registered
//        dispatch callback in foreground context.  That callback
//        publishes TIMEBASE_FRAGMENT.
//
//     3. EPOCH ANCHOR.  When a PPS rebootstrap is pending (armed by
//        alpha), the next PPS edge re-aligns the VCLOCK cadence to the
//        PPS moment.  The snapshot from that edge carries the counter
//        values that define the epoch.
//
//   The physical PPS edge thus establishes phase identity for VCLOCK
//   counting but does NOT drive per-second clock state advance.  Those
//   advances remain the responsibility of vclock_callback, fed by the
//   CH3 cadence now phase-locked to PPS.
//
// Per-lane cadence mechanics:
//
//   Three lanes (VCLOCK, OCXO1, OCXO2), each cadenced by its own
//   QuadTimer compare channel at 1 kHz, with identical hardware-to-
//   software latency profile: compare match → NVIC → ISR → DWT capture
//   as first instruction.
//
//   The 1 kHz cadence exists for one reason only: the QuadTimer compare
//   registers are 16-bit.  Advancing by 10,000,000 counts per second in
//   a single shot would overflow the compare target many times over, so
//   the compare is advanced every 10,000 counts (1 ms) and re-armed.
//
// Cadence sources:
//   VCLOCK: QTimer1 CH3 compare, +10000 counts per interval (GNSS 10 MHz)
//           ISR hosted by TimePop.  PPS-anchored via rebootstrap flow.
//   OCXO1:  QTimer3 CH2 compare, +10000 counts per interval (OCXO1 10 MHz)
//   OCXO2:  QTimer3 CH3 compare, +10000 counts per interval (OCXO2 10 MHz)
//   PPS:    Physical GPIO edge from GNSS receiver (witness + dispatch +
//           epoch anchor).
//
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
// A subscriber is a one-second-event recipient, not a hardware event
// source.  VCLOCK, OCXO1, OCXO2 are the three one-second-event-emitting
// subscribers.  The physical PPS GPIO edge is handled separately — see
// pps_edge_dispatch_fn and interrupt_last_pps_edge.
//

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
// One-second event — delivered to VCLOCK / OCXO subscribers every 1000 ticks
// CH2 cadence event — delivered to TimePop on every QTimer1 CH2 compare-match
// ============================================================================
//
// VCLOCK and OCXO lanes deliver one event per second to their foreground
// subscribers (via timepop_arm_asap deferral).  TIMEPOP receives an event
// on every CH2 compare-match — IRQ-context delivery, no foreground hop —
// because TimePop IS the foreground dispatcher and cannot defer its own
// scheduler heartbeat.
//
// The event payload is uniform across all kinds: dwt_at_event,
// gnss_ns_at_event, counter32_at_event, plus the optional
// pps_coincidence fields populated only on VCLOCK kind.

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  // DWT cycle count captured as the first instruction of the lane's
  // 1 kHz ISR on the 1000th tick.  Honest ISR-entry capture — no
  // projection, no synthesis.
  uint32_t dwt_at_event = 0;

  // GNSS ns at the one-second event, computed by time_dwt_to_gnss_ns()
  // from dwt_at_event.  May be 0 for events emitted before the bridge
  // has been anchored by alpha.
  uint64_t gnss_ns_at_event = 0;

  // Software-extended logical 32-bit count, advances by exactly
  // 10,000,000 per event.  For the VCLOCK lane, this is re-seeded at
  // PPS rebootstrap to match the QTimer1 CH0+CH1 count captured at the
  // PPS edge — so counter32_at_event - snapshot.counter32_at_edge
  // equals exactly N * VCLOCK_COUNTS_PER_SECOND for the Nth event
  // after anchor.
  uint32_t counter32_at_event = 0;

  // ── PPS/VCLOCK coincidence measurement ──
  //
  // Populated only for VCLOCK-kind one-second events.  Measures the
  // raw DWT-cycles difference between THIS ISR's entry (dwt_at_event)
  // and the most recent physical PPS edge's ISR entry:
  //
  //     pps_coincidence_cycles = dwt_at_event − snap.dwt_at_edge
  //
  // Under PPS-anchored operation the CH3 one-second compare-match and
  // the PPS GPIO edge are the same physical moment at the receiver,
  // modulo sub-nanosecond propagation.  With GPIO at priority 0 and
  // QTimer1 at priority 1, GPIO preempts — so CH3's ISR entry always
  // lands strictly AFTER the GPIO ISR finishes.  Steady-state cycles
  // is therefore expected to be a small positive number dominated by
  // GPIO ISR body duration plus CH3 entry latency — a constant, with
  // tiny variance determined only by pipeline state.
  //
  // `pps_coincidence_valid` is true iff:
  //   • At least one PPS edge has been captured (snap.sequence > 0)
  //   • cycles is non-negative
  //   • cycles < DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES
  //
  // A false value indicates this one-second event was NOT coincident
  // with a recent PPS edge (e.g. pre-anchor operation, or a VCLOCK
  // event that drifted out of PPS phase — which would itself be a
  // falsified-phase-lock signal).
  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;
};

// ============================================================================
// Diagnostic surface — carried alongside every one-second event
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  // Event truth surface — mirrors interrupt_event_t so consumers that
  // only hold a diag pointer still see the authoritative per-event facts.
  uint32_t dwt_at_event = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;

  // PPS GPIO witness fields.
  //
  // Populated only on the VCLOCK diag.  Refreshed in alpha's
  // pps_edge_callback from the authoritative pps_edge_snapshot_t
  // captured in the GPIO ISR.
  //
  // Under PPS-anchored operation, pps_edge_minus_event_ns is the
  // per-edge measurement that answers "how consistent is the GPIO-ISR
  // latency between the physical PPS edge and its coincident VCLOCK
  // one-second event."  Its mean is the DC latency; its stddev is
  // the jitter.  In steady state the mean is tens of nanoseconds
  // (GPIO ISR prologue cost) and the stddev is very small.
  uint32_t pps_edge_sequence = 0;
  uint32_t pps_edge_dwt_isr_entry_raw = 0;
  int64_t  pps_edge_gnss_ns = -1;
  int64_t  pps_edge_minus_event_ns = 0;

  // ── Bridge-independent VCLOCK-to-edge phase diagnostics ──
  //
  // Same information in a form that does NOT depend on the DWT↔GNSS
  // bridge.  Pure ISR-captured facts plus integer arithmetic.  Under
  // PPS-anchored operation, pps_edge_ns_from_vclock should stay very
  // close to zero — the GPIO edge and the VCLOCK one-second moment
  // are the same event up to GPIO-ISR latency.
  uint32_t pps_edge_dwt_cycles_from_vclock = 0;
  int64_t  pps_edge_ns_from_vclock = 0;
  uint32_t pps_edge_vclock_event_count = 0;

  // ── PPS/VCLOCK coincidence mirror ──
  //
  // Mirrors event.pps_coincidence_{cycles,valid} for consumers that
  // only hold a diag pointer.  See interrupt_event_t for semantics.
  uint32_t pps_coincidence_cycles = 0;
  bool     pps_coincidence_valid  = false;

  uint32_t anomaly_count = 0;
};

// ============================================================================
// PPS/VCLOCK coincidence threshold
// ============================================================================
//
// When the VCLOCK one-second compare-match fires, the ISR reads the
// most recent PPS edge snapshot and computes the DWT-cycles difference.
// If the difference is in [0, DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES),
// the two events are declared coincident and the measurement is
// published.  Otherwise the measurement is marked invalid (the PPS
// edge happened too long ago — either pre-anchor, or a phase-lock
// failure).
//
// 10,000 cycles at 1.008 GHz ≈ 10 µs.  The expected steady-state
// value is ~300–600 cycles (GPIO ISR body + entry latencies), so
// this threshold has three orders of magnitude of margin on the
// "normal" side and cleanly excludes the "PPS was seconds ago" case.
//
static constexpr int32_t DWT_PPS_COINCIDENCE_THRESHOLD_CYCLES = 10000;

// ── VCLOCK epoch tick offset ──
//
// Compensation for peripheral-bus-read quantization during epoch
// establishment.  When the PPS GPIO ISR captures the 32-bit VCLOCK
// cascade counter, the read completes ~140 CPU cycles (~138 ns) after
// the true PPS edge.  During that window the VCLOCK counter (10 MHz)
// has ticked once, so the counter value we read is the tick AFTER the
// one that corresponds to the PPS moment — not the PPS-corresponding
// tick itself.  If we store the read value as the anchor unmodified,
// we are asserting that the PPS edge happened at a moment that is
// physically 100 ns past the true edge, and every downstream target
// computed from that anchor lands one tick (100 ns) late.
//
// The correction is an identity adjustment.  We asked "which VCLOCK
// tick corresponds to the PPS moment?" and the hardware answered with
// the tick that came AFTER the one that really did.  We subtract 1
// to recover the tick that actually corresponds to the PPS edge.
//
// (The sign here is for READING an existing labeling.  If this code
// ever changed to WRITE a new labeling — setting the counter to a
// specific value at PPS — the sign would flip: we'd write +1 instead
// of 0, to honestly report that one tick has already passed since PPS
// by the time we got our write in.  Reading late vs writing late are
// opposite sides of the same coin.)
//
// This is a software label change — nothing about VCLOCK hardware,
// its 10 MHz tick stream, or PPS arrival is modified.  We are simply
// correcting the identity we assign to the PPS-corresponding tick.
//
// APPLICATION BOUNDARIES:
//   • Alpha applies this offset when installing its epoch from the
//     snapshot (process_clocks_alpha.cpp).  Alpha is the canonical
//     authority for the PPS-corresponding counter identity.
//   • process_interrupt's VCLOCK lane rebootstrap applies it locally,
//     because the VCLOCK lane establishes its logical count directly
//     from counter32 without going through alpha.
//   • The snapshot's counter32_at_edge is published RAW.  Consumers
//     that need the corrected identity get it from alpha, not the
//     snapshot.
//
// Deterministic because ISR entry and bus-read latencies have SD of
// 1-2 cycles across hundreds of fires.  Tunable constant: if live
// measurements show residuals shifting by more than expected after a
// code change, adjust this value (plausible range: -2 to 0).
static constexpr int32_t VCLOCK_EPOCH_TICK_OFFSET = -2;

// ============================================================================
// Physical PPS edge snapshot — captured in the GPIO ISR
// ============================================================================
//
// On every physical PPS rising edge, the GPIO ISR captures — in this
// order, as its first instructions after the DWT capture —
//
//   • QTimer1 CH0+CH1 cascaded 32-bit counter (the VCLOCK count)
//   • QTimer1 CH3 16-bit counter (the VCLOCK cadence channel)
//
// These captures define the PPS moment in terms of the VCLOCK counter
// hardware.  When a rebootstrap request is pending, the ISR additionally
// reprograms CH3's compare target so the next 1 ms cadence tick fires
// exactly VCLOCK_INTERVAL_COUNTS ticks after the PPS moment — phase-
// locking the VCLOCK one-second cadence to the physical PPS edge.
//
// Access is seqlock-protected; callers use interrupt_last_pps_edge()
// for a torn-read-safe copy.
//

struct pps_edge_snapshot_t {
  uint32_t sequence          = 0;
  uint32_t dwt_at_edge       = 0;   // NORMALIZED: raw PPS ISR-entry DWT minus
  //   (GPIO_TOTAL_LATENCY − WITNESS_STIMULATE_LATENCY)
  uint32_t dwt_raw_at_edge   = 0;   // RAW PPS ISR-entry DWT, unmodified
  //   (audit field — invariant is
  //    dwt_raw_at_edge − dwt_at_edge == 67)
  uint32_t counter32_at_edge = 0;   // QTimer1 CH0+CH1 32-bit count at ISR entry
  uint16_t ch3_at_edge       = 0;   // QTimer1 CH3 count at ISR entry
  int64_t  gnss_ns_at_edge   = -1;
};

// ============================================================================
// Subscription and event callback (VCLOCK + OCXO)
// ============================================================================

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t* diag,
             void* user_data);

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event = nullptr;
  void* user_data = nullptr;
};

// ============================================================================
// PPS edge dispatch callback
// ============================================================================
//
// Invoked from foreground context (via timepop_arm_asap) exactly once
// per physical PPS edge.  The snapshot is a stable copy of the state
// captured in the GPIO ISR at first instruction.  The registered
// callback's responsibilities:
//
//   1. If g_epoch_pending is true (i.e. alpha is waiting for a PPS-
//      anchored epoch install), install the epoch from snap.
//   2. Stash witness fields into the TIMEBASE_FRAGMENT-bound diag.
//   3. Call clocks_beta_pps() to publish the fragment.
//

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
// PPS-anchored epoch (at boot, at ZERO, at START).  The next physical
// PPS edge will:
//   • Capture the VCLOCK counter at ISR entry.
//   • Reprogram CH3's compare target to fire VCLOCK_INTERVAL_COUNTS
//     ticks later, aligning the VCLOCK cadence with the PPS moment.
//   • Reset the VCLOCK lane's tick_mod_1000 to 0 so the first post-
//     anchor one-second event fires exactly VCLOCK_COUNTS_PER_SECOND
//     ticks after PPS.
//   • Seed logical_count32 from snap.counter32_at_edge so subsequent
//     counter32_at_event values maintain the hardware-count identity.
//   • Clear the rebootstrap pending flag.
//
// interrupt_pps_rebootstrap_pending() returns true if the flag is set
// (i.e. a rebootstrap is armed but the next PPS edge has not yet
// consumed it).
//
void interrupt_request_pps_rebootstrap(void);
bool interrupt_pps_rebootstrap_pending(void);

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind);

// Register the single dispatch callback for physical PPS edges.  Only
// one callback may be registered; later registrations replace the prior.
// Called once at init by process_clocks_alpha.
void interrupt_pps_edge_register_dispatch(pps_edge_dispatch_fn fn);

// Seqlock-safe snapshot of the most recent physical PPS edge.
pps_edge_snapshot_t interrupt_last_pps_edge(void);

// ============================================================================
// QTimer1 CH2 IRQ-context handler — TimePop scheduler heartbeat
// ============================================================================
//
// TimePop registers a single CH2 handler at init.  process_interrupt's
// QTimer1 ISR captures DWT as the first instruction, then on every
// QTimer1 CH2 compare-match assembles a standard interrupt_event_t and
// interrupt_capture_diag_t (kind = TIMEPOP) and invokes this handler in
// IRQ context.  The handler runs to completion before the ISR returns.
//
// Unlike VCLOCK/OCXO subscriptions, this delivery is NOT foreground-
// deferred — TimePop IS the foreground dispatcher and cannot defer its
// own scheduler heartbeat.
//
// Pass nullptr to unregister.  Only one CH2 handler may be registered;
// subsequent calls replace the previous registration.
//
using interrupt_qtimer1_ch2_handler_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t& diag);

void interrupt_register_qtimer1_ch2_handler(interrupt_qtimer1_ch2_handler_fn cb);

// ============================================================================
// QTimer1 CH3 witness — torture-sanity test of timekeeping math
// ============================================================================
//
// The witness fires QTimer1 CH3 at nine deliberately PPS-avoiding sub-second
// slots (100 ms through 900 ms offsets from the most recent PPS edge).  At
// each fire, CH3's ISR captures DWT as the first instruction and compares
// against the caller-supplied prediction of what DWT should read at that
// VCLOCK moment.  The residual — if everything is right — is dominated by
// NVIC vectoring + ISR entry latency: a deterministic small constant,
// probably under 100 ns.
//
// If the residual is larger, it accuses one of the math assumptions in the
// timekeeping stack: the DWT rate prediction, the PPS anchor, the VCLOCK
// counter arithmetic, a TDC constant, or an off-by-one in the cascade read.
// Every component that contributes to time_dwt_to_gnss_ns() is implicated.
//
// Life cycle:
//   1. Alpha calls interrupt_witness_set_mode(ON).  Normal VCLOCK cadence
//      is suspended while the witness holds CH3.  Foreground traffic
//      continues normally because CH2 (TimePop scheduler) is untouched.
//   2. On every PPS edge, alpha's pps_edge_callback calls
//      interrupt_witness_arm_first_slot(anchor_counter32, anchor_dwt,
//      dwt_cycles_per_second).  This schedules the first of nine slots.
//   3. Each witness CH3 fire self-rotates to the next slot (100ms → 200ms
//      → ... → 900ms).  After the 900 ms slot, CH3 is left idle until the
//      next PPS edge re-arms slot 0.
//   4. Alpha calls interrupt_witness_set_mode(OFF) to stop.  Next PPS edge
//      performs a normal rebootstrap and VCLOCK cadence resumes.
//
// The Welford accumulates residual-in-nanoseconds.  Alpha (or any
// consumer) may read via interrupt_witness_stats().

enum class interrupt_witness_mode_t : uint8_t {
  OFF = 0,  // normal CH3 cadence operation
  ON,       // witness owns CH3
};

struct interrupt_witness_stats_t {
  interrupt_witness_mode_t mode;
  uint64_t n;                    // combined Welford sample count
  double   mean_ns;              // combined mean residual in nanoseconds
  double   stddev_ns;            // combined stddev in nanoseconds
  double   stderr_ns;            // combined standard error in nanoseconds
  int64_t  min_ns;               // worst-case negative residual observed
  int64_t  max_ns;               // worst-case positive residual observed
  uint64_t fires_total;          // CH3 fires while in witness mode
  uint64_t fires_rejected;       // fires rejected as out-of-window garbage

  // CH3-vs-CH0 phase-lag measurement (diagnostic).  At witness ISR
  // entry we read both the CH0+CH1 cascade counter AND the CH3
  // counter.  CH0 and CH3 both clock from the 10 MHz VCLOCK (PCS=0)
  // but were enabled in separate register writes at init time, so
  // CH3 typically lags CH0 by a small, fixed number of ticks.  Each
  // tick = 100 ns.  This lag directly accounts for why compare
  // events fire late relative to predictions derived from CH0's
  // counter — which is the root cause of any non-physical offset
  // in mean_ns.
  uint64_t lag_n;
  double   lag_mean_ticks;       // mean lag in 100-ns VCLOCK ticks
  double   lag_stddev_ticks;
  int32_t  lag_min_ticks;
  int32_t  lag_max_ticks;

  // GPIO ISR peripheral-bus delay: DWT cycles between the ISR's
  // first-instruction DWT capture and the counter read that populates
  // snap.counter32_at_edge.  This measures how much real time elapses
  // before the counter reflects the post-PPS state.  A residual that's
  // off by this many cycles would be attributable to using
  // snap.dwt_at_edge as the anchor while snap.counter32_at_edge was
  // read later.
  uint64_t gpio_counter_delay_n;
  double   gpio_counter_delay_mean_cycles;
  double   gpio_counter_delay_stddev_cycles;
  int32_t  gpio_counter_delay_min_cycles;
  int32_t  gpio_counter_delay_max_cycles;
};

void interrupt_witness_set_mode(interrupt_witness_mode_t mode);
interrupt_witness_mode_t interrupt_witness_get_mode(void);

// Arm the first witness slot (100 ms after anchor_counter32 VCLOCK tick).
// Called by alpha from pps_edge_callback, exactly once per PPS edge, when
// witness mode is ON.  Caller supplies the PPS-edge counter32, PPS-edge
// DWT, and the most recent DWT-cycles-per-second prediction.  Subsequent
// slots (200ms, 300ms, ..., 900ms) are armed self-recursively by the
// witness ISR path.
void interrupt_witness_arm_first_slot(uint32_t anchor_counter32,
                                      uint32_t anchor_dwt,
                                      uint32_t anchor_dwt_raw,
                                      uint32_t dwt_cycles_per_second);

interrupt_witness_stats_t interrupt_witness_stats(void);
void interrupt_witness_reset_stats(void);

uint32_t interrupt_hw_witness_gpio_delta_cycles(void);
uint32_t interrupt_hw_witness_qtimer_delta_cycles(void);
uint32_t interrupt_hw_witness_qtimer_cntr(void);
uint32_t interrupt_hw_witness_qtimer_comp1(void);
uint32_t interrupt_hw_witness_qtimer_csctrl(void);
uint32_t interrupt_hw_witness_qtimer_ctrl(void);
uint32_t interrupt_hw_witness_qtimer_enbl(void);
uint32_t interrupt_hw_witness_qtimer_mux(void);
uint32_t interrupt_hw_witness_qtimer_select_input(void);
uint32_t interrupt_hw_witness_qtimer_source_reads(void);
uint32_t interrupt_hw_witness_qtimer_nonzero_cntr_observations(void);
uint32_t interrupt_hw_witness_qtimer_cntr_change_hits(void);
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_at_source(void);
uint32_t interrupt_hw_witness_qtimer_tcf1_seen_in_irq(void);

// ISR entry points (invoked by vector shims).
void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_isr_entry_raw);

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now(void);

// ── Dynamic DWT cycles per GNSS second ──
//
// Home of the enhanced DWT-cycles-per-second prediction.  A 1 kHz
// TimePop recurring series observes ARM_DWT_CYCCNT at each tick and
// refines an estimate of DWT cycles per GNSS second by absorbing the
// delta between actual DWT and DWT predicted from the PPS anchor.
//
// Bookends are the pure-gears, ISR-grade anchors:
//   anchor_dwt     = priority-0 DWT at the most recent PPS GPIO edge
//   anchor_gnss_ns = GNSS ns at that same PPS edge
//
// At each PPS edge the estimate is reseeded from alpha's
// g_dwt_cycle_count_between_pps (the honest two-bookend measurement).
// Between edges it is refined permanently by every tick observation:
// the delta between actual and predicted DWT is added to the estimate.
// Self-correcting per tick, with PPS-edge re-anchor as ground truth.
//
// Clients #include process_interrupt.h to consume.
uint32_t interrupt_dynamic_cps(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);