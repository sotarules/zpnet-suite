#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// process_interrupt.h — ISR custodian, sole owner of QTimer hardware
// ============================================================================
//
// Responsibilities:
//   • SOLE OWNER of QTimer1 and QTimer3.  No other module touches a
//     QTimer register.  Any module that needs hardware-precise timing
//     calls our public API.
//   • ISR custody: GPIO6789 (PPS), QTimer1 (CH3 cadence + CH2 schedule
//     fire), QTimer3 (CH2 OCXO1, CH3 OCXO2).
//   • Latency adjustment of first-instruction _raw DWT captures into
//     event-coordinate DWT.
//   • Three internal synthetic 32-bit counters (vclock, ocxo1, ocxo2)
//     advanced by gear arithmetic on each cadence ISR.  Never read from
//     foreground; the only way to observe their values is through the
//     dispatched event.
//   • TICK dispatch every cadence interval (1 ms) for foreground
//     scheduling consumers (TimePop's asap/alap drain + pre-bridge
//     anchoring).
//   • Per-lane 1 Hz subscriber events on the 1000th cadence tick (VCLOCK,
//     OCXO1, OCXO2).
//   • PPS-anchored VCLOCK rebootstrap on demand.
//   • Schedule-fire mechanism: foreground modules program a future
//     counter32 deadline via interrupt_schedule_fire_at(); when the
//     QTimer1 CH2 compare matches, an ISR-context callback is invoked.
//
// Non-responsibilities:
//   • State that anyone else owns.  The clocks are kept by alpha; the
//     synthetic counters above are zeroed by alpha at epoch install via
//     interrupt_synthetic_counters_zero().
//
// Event contract (subscriber dispatch):
//   The dispatched `interrupt_event_t` carries:
//     gnss_ns_at_edge    — canonical GNSS nanoseconds at the edge
//     dwt_at_edge        — event-coordinate DWT at the edge
//     counter32_at_edge  — event-coordinate synthetic counter32 at the
//                          edge (in the lane's tick domain — VCLOCK
//                          ticks for PPS_VCLOCK / VCLOCK / TICK,
//                          OCXO1 ticks for OCXO1, OCXO2 ticks for OCXO2)
//     kind               — which subscription
//     sequence           — monotone per-kind counter
//
//   For PPS_VCLOCK: gnss_ns_at_edge = sequence × 1e9 (VCLOCK-authored;
//     ends in "00000000").
//   For all other kinds: gnss_ns_at_edge = time_dwt_to_gnss_ns(dwt_at_edge),
//     evaluated inside the ISR against alpha's PPS_VCLOCK slot.
//
// _raw rule:
//   _raw is reserved for the values an ISR captures directly from
//   hardware (ARM_DWT_CYCCNT first-instruction reads).  Converted to
//   event-coordinate immediately and never propagated.  No data structure
//   carries _raw.
//
// Counter read doctrine:
//   Live synthetic-counter reads from foreground are FORBIDDEN.
//   Synthetic counters are advanced by gear arithmetic at known moments
//   (cadence ISR fires) and observed only through the dispatched event.
//   The schedule-fire callback receives counter32_at_edge as part of its
//   ISR-context parameters — that is the only legitimate window into the
//   synthetic counter at the moment of fire.
//
// NVIC priority structure:
//   PPS GPIO (IRQ_GPIO6789)  : priority 0  — sovereign; preempts all
//   QTimer1  (IRQ_QTIMER1)   : priority 16 — VCLOCK cadence (CH3) +
//                                            schedule fire (CH2);
//                                            preempted by PPS only
//   QTimer3  (IRQ_QTIMER3)   : priority 32 — OCXO1 (CH2) + OCXO2 (CH3);
//                                            preempted by PPS or QTimer1
//
//   This structure ensures PPS_VCLOCK is never delayed by lower-priority
//   ISRs.  PPS_VCLOCK must remain unperturbed because it is the sovereign
//   anchor of GNSS time on the Teensy — any latency added to its capture
//   corrupts the bookend DWT rate that drives the entire bridge.
//
//   Within IRQ_QTIMER1, CH2 (schedule fire / SpinDry) and CH3 (VCLOCK
//   cadence) share the same priority and tail-chain.  This is the
//   correct relationship: a SpinDry approach must not be preempted by
//   the 1 kHz cadence (which would corrupt sub-µs landing precision),
//   and a cadence ISR must not be preempted by a SpinDry approach
//   (which would skew the synthetic counter advance against its own
//   ISR clock).  Tail-chain ordering preserves both.
// ============================================================================

// ============================================================================
// Subscriber identities
// ============================================================================

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  PPS_VCLOCK,    // 1 Hz GNSS PPS edge.
  VCLOCK,        // 1 Hz one-second event on VCLOCK CH3 cadence.
  OCXO1,         // 1 Hz one-second event on OCXO1 cadence.
  OCXO2,         // 1 Hz one-second event on OCXO2 cadence.
  TICK,          // 1 kHz cadence tick (foreground scheduler heartbeat).
};

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  GPIO6789,
  QTIMER1,
  QTIMER3,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  GPIO_EDGE,
  QTIMER1_CH2_SCHED,   // schedule-fire (TimePop-driven CH2 compare)
  QTIMER1_CH3_COMP,    // VCLOCK cadence
  QTIMER3_CH2_COMP,    // OCXO1 cadence
  QTIMER3_CH3_COMP,    // OCXO2 cadence
};

// ============================================================================
// Event — the entire payload of a subscription dispatch
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  int64_t  gnss_ns_at_edge   = -1;  // canonical GNSS ns at the edge
  uint32_t dwt_at_edge       = 0;   // event-coordinate DWT
  uint32_t counter32_at_edge = 0;   // synthetic counter32 in the lane's tick domain
  uint32_t sequence          = 0;   // monotone per-kind counter
};

// ============================================================================
// Subscription
// ============================================================================

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event, void* user_data);

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t   kind      = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event  = nullptr;
  void*                         user_data = nullptr;
};

// ============================================================================
// Public API — lifecycle
// ============================================================================

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

// ============================================================================
// Public API — subscriber registry
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start  (interrupt_subscriber_kind_t kind);
bool interrupt_stop   (interrupt_subscriber_kind_t kind);

// ============================================================================
// Public API — PPS-anchored epoch installation
// ============================================================================
//
// Alpha calls this at INIT/ZERO/START.  The next physical PPS GPIO edge
// will:
//   • Reprogram CH3's compare to fire VCLOCK_INTERVAL_COUNTS ticks later
//     (phase-locking the VCLOCK cadence to the PPS moment).
//   • Reset tick_mod_1000 to 0 so the first post-anchor one-second event
//     fires exactly VCLOCK_COUNTS_PER_SECOND ticks after PPS.
//   • Reset PPS_VCLOCK sequence to 0 (so gnss_ns_at_edge restarts at 0).
//   • Clear the rebootstrap pending flag.
void interrupt_request_pps_rebootstrap(void);
bool interrupt_pps_rebootstrap_pending(void);

// Synthetic-counter zeroing — alpha's only privilege over the internal
// counters.  Called from alpha_install_new_epoch_from_pps_event in tandem
// with the ns-counter zeroing.  Sets all three (vclock, ocxo1, ocxo2)
// synthetic counters to 0.  The next event from each lane will carry
// counter32_at_edge advanced by exactly the cadence step that fires it.
void interrupt_synthetic_counters_zero(void);

// ============================================================================
// Public API — schedule-fire mechanism (CH2 hardware-driven)
// ============================================================================
//
// TimePop is the sole consumer of this API.  No other module should
// register or arm CH2 fires.
//
// Contract:
//   1. The consumer registers a callback ONCE via
//      interrupt_schedule_register().
//   2. The consumer arms a fire by calling interrupt_schedule_fire_at()
//      with a target counter32 value (in VCLOCK tick domain).  Idempotent:
//      replaces any previously armed fire.  Returns false if the target
//      has already passed (consumer should fire via foreground asap).
//   3. When the QTimer1 CH2 compare matches the target, the registered
//      callback is invoked IN ISR CONTEXT with the event-coordinate DWT
//      and synthetic counter32 captured at the fire moment.  After the
//      callback returns, the compare is left disabled until the consumer
//      arms a new fire.
//   4. The consumer cancels via interrupt_schedule_fire_cancel() to
//      disarm without firing.
//
// ISR-context callback contract:
//   • Runs at IRQ_QTIMER1 priority — preempted only by PPS GPIO.
//   • Must not block, must not wait on foreground state, must return
//     promptly.  Consumers that need foreground delivery should arm an
//     asap slot from inside the callback and let foreground drain it.
//   • May call back into interrupt_schedule_fire_at() to arm the next
//     fire (the standard head-of-queue rearm pattern).
//
// Counter32 latency:
//   The captured counter32_at_edge is the synthetic VCLOCK counter snapped
//   from g_vclock_count32 at the moment the CH2 ISR runs.  It is in the
//   SAME coordinate system as cadence/PPS event counter32_at_edge values
//   — uniform tick-axis truth.  It may differ from the requested target
//   by a small number of ticks (the ISR latency budget); the consumer
//   should treat this as the authoritative "now in counter32 space" for
//   subsequent scheduling.

typedef void (*interrupt_schedule_fire_isr_fn)(
    uint32_t dwt_at_edge,
    uint32_t counter32_at_edge,
    void*    user_data
);

// One-time registration.  Replaces any prior registration.  Safe to call
// before process_interrupt_enable_irqs() — the callback will not be
// invoked until both registration AND a fire are armed AND the IRQs are
// live.
void interrupt_schedule_register(
    interrupt_schedule_fire_isr_fn callback,
    void*                          user_data
);

// Program CH2 to fire when g_vclock_count32 reaches target_counter32.
// Returns:
//   • true  — fire armed; callback will be invoked in ISR context when
//             the compare matches
//   • false — target is already in the past (caller should fire via
//             foreground asap rather than wait for the 32-bit cascade
//             to wrap around in ~7 minutes), OR no callback registered,
//             OR hardware not yet initialized
//
// Idempotent: replaces any previously armed fire.
bool interrupt_schedule_fire_at(uint32_t target_counter32);

// Disarm CH2 — no scheduled fire pending.
void interrupt_schedule_fire_cancel(void);

// True if a CH2 fire is currently armed.
bool interrupt_schedule_fire_armed(void);

// Returns the most-recently-observed counter32_at_edge across all
// VCLOCK-domain events (TICK, VCLOCK, PPS_VCLOCK, schedule-fire).
// Foreground consumers may use this as their best estimate of "now in
// counter32 space" for scheduling decisions.  Stale by at most 1 ms in
// steady state (a TICK fires every ms).  Returns 0 if no event has yet
// been dispatched.
uint32_t interrupt_last_known_counter32(void);

// True iff at least one event has been dispatched since boot (i.e.
// interrupt_last_known_counter32() returns a meaningful value).
bool interrupt_last_known_counter32_valid(void);

// Diagnostic counters — exposed for REPORT.
uint32_t interrupt_schedule_fires_armed_total     (void);
uint32_t interrupt_schedule_fires_dispatched_total(void);
uint32_t interrupt_schedule_fires_late_arm_total  (void);
uint32_t interrupt_schedule_fires_cancelled_total (void);

// ============================================================================
// ISR entry points (invoked by vector shims)
// ============================================================================

void process_interrupt_gpio6789_irq   (uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer1_ch2_irq(uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t isr_entry_dwt_raw);

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str  (interrupt_provider_kind_t   provider);
const char* interrupt_lane_str           (interrupt_lane_t            lane);