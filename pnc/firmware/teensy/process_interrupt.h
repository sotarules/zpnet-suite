#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// process_interrupt.h — ISR custodian, scorched
// ============================================================================
//
// Responsibilities:
//   • ISR custody (QTimer1, QTimer3, GPIO6789).  All QTimer1 channels
//     (CH0/CH1 cascade, CH3 cadence) are owned here end-to-end.  CH2 is
//     unused.
//   • Latency adjustment of first-instruction _raw DWT captures into
//     event-coordinate DWT.
//   • Three internal synthetic 32-bit counters (vclock, ocxo1, ocxo2)
//     advanced by gear arithmetic on each cadence ISR.  Never read from
//     foreground; the only way to observe their values is through the
//     dispatched event.
//   • TICK dispatch every cadence interval (1 ms) for foreground
//     scheduling consumers (TimePop).
//   • Per-lane 1 Hz subscriber events on the 1000th cadence tick (VCLOCK,
//     OCXO1, OCXO2).
//   • PPS-anchored VCLOCK rebootstrap on demand.
//
// Non-responsibilities:
//   • State that anyone else owns.  The clocks are kept by alpha; the
//     synthetic counters above are zeroed by alpha at epoch install via
//     interrupt_synthetic_counters_zero().
//
// Event contract:
//   The dispatched `interrupt_event_t` carries:
//     gnss_ns_at_edge    — canonical GNSS nanoseconds at the edge
//     dwt_at_edge        — event-coordinate DWT at the edge
//     counter32_at_edge  — event-coordinate synthetic counter32 at the edge
//                          (in the lane's tick domain — VCLOCK ticks for
//                           PPS_VCLOCK / VCLOCK / TICK / TIMEPOP, OCXO1
//                           ticks for OCXO1, OCXO2 ticks for OCXO2)
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
//   Live counter reads are FORBIDDEN — even in ISR context.  Synthetic
//   counters are advanced by gear arithmetic at known moments (cadence
//   ISR fires), and observed only through the dispatched event.  No
//   accessor exposes them.  No foreground caller can ever read them.
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
  TIMEPOP,       // Reserved for future scheduled-fire scheme.
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
  QTIMER1_CH3_COMP,
  QTIMER3_CH2_COMP,
  QTIMER3_CH3_COMP,
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
// Public API
// ============================================================================

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

// PPS-anchored epoch installation.  Alpha calls this at INIT/ZERO/START.
// The next physical PPS GPIO edge will:
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
// ISR entry points (invoked by vector shims)
// ============================================================================

void process_interrupt_gpio6789_irq  (uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t isr_entry_dwt_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t isr_entry_dwt_raw);

// ============================================================================
// Compatibility shim (loop() plumbing)
// ============================================================================

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// Stringifiers
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str  (interrupt_provider_kind_t   provider);
const char* interrupt_lane_str           (interrupt_lane_t            lane);