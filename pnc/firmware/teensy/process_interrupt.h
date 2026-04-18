// ============================================================================
// process_interrupt.h — interrupt custody + per-lane 1 kHz cadence
// ============================================================================
//
// process_interrupt owns interrupt custody for:
//   • OCXO lanes (QTimer3 vector — CH2, CH3)
//   • VCLOCK lane (QTimer1 CH3, via TimePop's hosted dispatch)
//   • Physical PPS GPIO edge (witness + dispatch authority + epoch anchor)
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
  TIME_TEST,
};

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER1,
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
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
// ============================================================================

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
  uint32_t dwt_at_edge       = 0;
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

// ISR entry points (invoked by vector shims).
void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_isr_entry_raw);

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);