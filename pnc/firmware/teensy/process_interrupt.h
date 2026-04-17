// ============================================================================
// process_interrupt.h — interrupt custody + per-lane 1 kHz cadence
// ============================================================================
//
// process_interrupt owns interrupt custody for:
//   • OCXO lanes (QTimer3 vector — CH2, CH3)
//   • VCLOCK lane (QTimer1 CH3, via TimePop's hosted dispatch)
//   • Physical PPS GPIO edge (witness + dispatch authority)
//
// Doctrine — canonical GNSS clock authority:
//
//   The GNSS ns timeline is VCLOCK-driven.  The VCLOCK lane delivers one
//   event per second to alpha's vclock_callback, which advances the
//   bridge anchor and all canonical clock state.  Each event carries
//   dwt_at_event (the ISR's first-instruction DWT capture),
//   gnss_ns_at_event, and counter32_at_event — all ISR-captured facts
//   or simple authored counts.  Alpha computes DWT cycles-between-events
//   by one-second subtraction of consecutive dwt_at_event values.
//
//   This arrangement exists to escape the latency and jitter of the
//   physical PPS GPIO edge.  The physical edge is a real-world event;
//   VCLOCK is the gear we set running by the first one and read ever
//   after.
//
// Doctrine — physical PPS edge:
//
//   The physical PPS GPIO edge plays two narrow roles:
//
//     1. DIAGNOSTIC.  The GPIO ISR captures DWT as its first
//        instruction, translates to GNSS ns via the bridge, and stores
//        a seqlock-protected pps_edge_snapshot_t (sequence, dwt, gnss_ns).
//        Consumers read this via interrupt_last_pps_edge().  Under this
//        doctrine, gnss_ns_at_edge IS the canonical "when did the
//        physical PPS edge occur in GNSS ns" diagnostic.
//
//     2. DISPATCH AUTHORITY FOR TIMEBASE_FRAGMENT.  On every edge, the
//        GPIO ISR arms timepop_arm_asap to invoke a single registered
//        dispatch callback in foreground context.  That callback
//        assembles and publishes TIMEBASE_FRAGMENT.
//
//   The physical edge does NOT drive GNSS clock state.  VCLOCK remains
//   the authority for all of that.
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
//   On every 1 kHz tick the ISR does three things:
//     1. Capture ARM_DWT_CYCCNT as the first instruction.
//     2. Clear the compare flag.
//     3. Advance the compare target by +10000 counts (16-bit wrapping).
//
//   Every 1000th tick additionally emits a one-second event to the
//   lane's subscriber, carrying the ISR-entry DWT capture, the
//   DWT-to-GNSS-ns translation, and an authored 32-bit count that
//   advances by exactly 10,000,000 per event.
//
// Cadence sources:
//   VCLOCK: QTimer1 CH3 compare, +10000 counts per interval (GNSS 10 MHz)
//           ISR hosted by TimePop.
//   OCXO1:  QTimer3 CH2 compare, +10000 counts per interval (OCXO1 10 MHz)
//   OCXO2:  QTimer3 CH3 compare, +10000 counts per interval (OCXO2 10 MHz)
//   PPS:    Physical GPIO edge from GNSS receiver (witness + dispatch only).
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
  // from dwt_at_event.  May be 0 for the bootstrap VCLOCK event if the
  // bridge is not yet valid (alpha treats that as epoch install).
  uint64_t gnss_ns_at_event = 0;

  // Software-extended logical 32-bit count, advances by exactly
  // 10,000,000 per event.
  uint32_t counter32_at_event = 0;
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
  // These describe the physical PPS GPIO ISR path and are the ONLY fields
  // in this struct that should be used for PPS-latency / PPS-jitter
  // analysis.  Populated only on the VCLOCK diag.  They are refreshed in
  // alpha's pps_edge_callback from the authoritative pps_edge_snapshot_t
  // captured in the GPIO ISR.
  uint32_t pps_edge_sequence = 0;
  uint32_t pps_edge_dwt_isr_entry_raw = 0;
  int64_t  pps_edge_gnss_ns = -1;
  int64_t  pps_edge_minus_event_ns = 0;

  // ── Bridge-independent VCLOCK-to-edge phase diagnostics ──
  //
  // These three fields expose the phase relationship between the physical
  // PPS edge and the most recent VCLOCK one-second event, WITHOUT going
  // through the DWT↔GNSS bridge.  They are pure ISR-captured facts and
  // arithmetic, and reveal the boot-determined phase offset between the
  // VCLOCK cadence and the physical PPS cadence.
  //
  //   pps_edge_dwt_cycles_from_vclock
  //     = snap.dwt_at_edge − g_prev_dwt_at_vclock_event
  //     (32-bit subtraction; wraps correctly across DWT_CYCCNT rollover).
  //     In steady state this is the DWT distance from the most recent
  //     VCLOCK one-second event to the physical PPS edge that followed it.
  //
  //   pps_edge_ns_from_vclock
  //     = pps_edge_dwt_cycles_from_vclock × 1e9 / dwt_effective_cycles_per_second()
  //     Same quantity in nanoseconds, using the same divisor the bridge
  //     uses for interpolation (apples-to-apples with pps_edge_gnss_ns).
  //
  //   pps_edge_vclock_event_count
  //     = number of times vclock_callback has been entered, as observed
  //     from pps_edge_callback.  In steady state this should equal
  //     (g_gnss_ns_count_at_pps / 1e9) at publish time, confirming that
  //     no VCLOCK event slipped in between the GPIO ISR and the
  //     foreground pps_edge_callback.
  uint32_t pps_edge_dwt_cycles_from_vclock = 0;
  int64_t  pps_edge_ns_from_vclock = 0;
  uint32_t pps_edge_vclock_event_count = 0;

  uint32_t anomaly_count = 0;
};

// ============================================================================
// Physical PPS edge snapshot — captured in the GPIO ISR
// ============================================================================
//
// On every physical PPS rising edge, the GPIO ISR captures DWT as its
// first instruction, translates to GNSS ns via the bridge (if valid),
// and stores this snapshot.  Access is seqlock-protected; callers use
// interrupt_last_pps_edge() for a torn-read-safe copy.
//
// gnss_ns_at_edge is -1 on the first edges after boot if the bridge has
// not yet been validated by VCLOCK.  In steady state — VCLOCK has been
// disciplined, bridge is valid — gnss_ns_at_edge is the canonical
// answer to "when did the physical PPS edge occur, expressed in GNSS
// nanoseconds."
//
// The sequence field advances monotonically and is used by consumers
// to detect missed edges.
//

struct pps_edge_snapshot_t {
  uint32_t sequence        = 0;
  uint32_t dwt_at_edge     = 0;
  int64_t  gnss_ns_at_edge = -1;
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
// callback's responsibility is narrow: stash gnss_ns_at_edge into the
// TIMEBASE_FRAGMENT-bound diag, and call clocks_beta_pps() to publish
// the fragment.
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

void interrupt_request_pps_zero(void);
bool interrupt_pps_zero_pending(void);

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