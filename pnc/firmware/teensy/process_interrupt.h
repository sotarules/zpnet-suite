#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// process_interrupt.h - ISR custodian, sole owner of QTimer hardware
// ============================================================================
//
// process_interrupt is the firmware timing boundary.  It owns QTimer1,
// QTimer3, PPS GPIO interrupt custody, first-instruction DWT capture, event
// coordinate conversion, synthetic event-ledger counters, and the CH2
// schedule-fire actuator used by TimePop's explicit precision mode.
//
// Responsibilities:
//   - SOLE OWNER of QTimer1 and QTimer3 registers.
//   - ISR custody for PPS GPIO, QTimer1 CH2/CH3, and QTimer3 CH2/CH3.
//   - Conversion of _raw first-instruction DWT captures into canonical
//     event-coordinate DWT values.
//   - Maintenance of three internal synthetic 32-bit event ledgers:
//       vclock, ocxo1, ocxo2.
//   - Publication of small interrupt_event_t payloads to subscribers.
//   - PPS-anchored VCLOCK rebootstrap on demand.
//   - CH2 schedule-fire actuator for TimePop precision/SpinDry work.
//   - PPS↔VCLOCK irreducible-offset measurement (NVIC-serialized; the
//     PPS GPIO ISR stashes its raw entry DWT and the VCLOCK cadence
//     ISR computes the cycle delta on every 1-second boundary).
//
// Non-responsibilities:
//   - Foreground timing policy.  TimePop decides what to schedule.
//   - Clock ownership.  Alpha owns canonical clock state and bridge state.
//   - General foreground register reads.  No module should ask QTimer
//     hardware, "what time is it now?"
//
// Synthetic counter doctrine:
//   The synthetic counters are event ledgers, not public live clocks.
//   They are exact at authored cadence events.  They are advanced by gear
//   arithmetic in ISR context and observed only through interrupt_event_t
//   or the schedule-fire callback.  There are no foreground read accessors
//   for the counters themselves.
//
// Last-known counter32:
//   interrupt_last_known_counter32() returns the most recent event-authored
//   VCLOCK-domain coordinate observed by dispatch.  It is a fallback and
//   diagnostic reference, not a live clock.  It is invalidated when Alpha
//   zeros the synthetic counters.
//
// Event contract:
//   interrupt_event_t carries:
//     gnss_ns_at_edge    - canonical GNSS nanoseconds at the edge
//     dwt_at_edge        - event-coordinate DWT at the edge
//     counter32_at_edge  - event-coordinate synthetic counter32 in the
//                          event lane's tick domain
//     kind               - subscriber kind
//     sequence           - monotone per-kind sequence
//
//   PPS_VCLOCK gnss_ns_at_edge is sequence * 1e9.
//   Other lanes project GNSS time from event-coordinate DWT via Alpha's
//   bridge.
//
// _raw rule:
//   _raw values are hardware-only ISR-entry captures.  They are converted
//   immediately and never propagated in public structures.
//
// TimePop relationship:
//   process_interrupt provides two TimePop services:
//     1. TICK events from QTimer1 CH3 for stable foreground scheduling.
//     2. QTimer1 CH2 schedule-fire for explicit precision/SpinDry work.
//
//   process_interrupt also uses TimePop ASAP to defer subscriber callbacks
//   out of ISR context.  This mutual dependency is intentional and relies
//   on ordinary recurring services staying on TimePop's TICK mode.
//
// Schedule-fire contract:
//   CH2 is a precision actuator, not the general foreground scheduler.
//   The caller supplies a target synthetic VCLOCK counter32 coordinate.
//   CH2's 16-bit compare is programmed for the corresponding low-word
//   candidate fire.  On each CH2 interrupt, process_interrupt projects the
//   current 32-bit VCLOCK coordinate from event-coordinate DWT and Alpha's
//   bridge.  The callback is invoked only when the projected coordinate is
//   within a small tolerance of the requested target.  Premature low-word
//   matches remain armed and are counted diagnostically.
//
// PPS↔VCLOCK irreducible offset (NVIC-serialized):
//   PPS GPIO and the GF-8802's 10 MHz VCLOCK are physically synchronous —
//   the same emission moment.  process_interrupt measures the irreducible
//   path-difference DWT cycles between them empirically.  The PPS GPIO
//   ISR (priority 0) captures its first-instruction raw DWT and stashes
//   it.  The VCLOCK CH3 cadence ISR (priority 16), on every 1-second
//   boundary, captures its own first-instruction raw DWT and computes
//   delta = vclock_dwt - pps_dwt.  Both timestamps live in the same raw
//   ISR-entry frame; no latency math is applied (it would distort what
//   the measurement is capturing).  NVIC strictly serializes the two
//   ISRs, so the cadence boundary always lands cleanly after the PPS
//   GPIO ISR has returned, and common-mode jitter partially cancels in
//   the subtraction.  A sanity ceiling filters skewed/missed-edge
//   samples.  No constants are consulted; the truth is whatever the
//   hardware reports.
//
// NVIC priority structure:
//   PPS GPIO (IRQ_GPIO6789)  : priority 0  - sovereign; preempts all
//   QTimer1  (IRQ_QTIMER1)   : priority 16 - VCLOCK cadence + schedule-fire
//   QTimer3  (IRQ_QTIMER3)   : priority 32 - OCXO cadences
//
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
  QTIMER1_CH2_SCHED,
  QTIMER1_CH3_COMP,
  QTIMER3_CH2_COMP,
  QTIMER3_CH3_COMP,
};

// ============================================================================
// Event payload
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  int64_t  gnss_ns_at_edge   = -1;
  uint32_t dwt_at_edge       = 0;
  uint32_t counter32_at_edge = 0;
  uint32_t sequence          = 0;
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
// Lifecycle
// ============================================================================

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

// ============================================================================
// Subscriber registry
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start  (interrupt_subscriber_kind_t kind);
bool interrupt_stop   (interrupt_subscriber_kind_t kind);

// ============================================================================
// PPS-anchored epoch installation
// ============================================================================
//
// Alpha calls this at INIT/ZERO/START.  The next physical PPS edge
// re-phases VCLOCK CH3, resets the PPS_VCLOCK sequence, and lets Alpha
// install a physical-edge epoch from the resulting PPS_VCLOCK event.

void interrupt_request_pps_rebootstrap(void);
bool interrupt_pps_rebootstrap_pending(void);

// Alpha's only privilege over the internal event-ledger counters.  Sets
// vclock/ocxo1/ocxo2 synthetic counters to zero and invalidates the
// last-known VCLOCK-domain event coordinate.
void interrupt_synthetic_counters_zero(void);

// ============================================================================
// Schedule-fire mechanism (QTimer1 CH2 precision actuator)
// ============================================================================
//
// TimePop is the sole consumer of this API.
//
// Contract:
//   1. Register one ISR callback via interrupt_schedule_register().
//   2. Arm a fire using a target VCLOCK-domain counter32 coordinate.
//   3. CH2 may interrupt on premature low-word matches; these are absorbed
//      internally and counted.  The callback fires only when the projected
//      full 32-bit VCLOCK coordinate is within tolerance of the target.
//   4. Callback runs in IRQ_QTIMER1 context and must be tiny.

typedef void (*interrupt_schedule_fire_isr_fn)(
    uint32_t dwt_at_edge,
    uint32_t counter32_at_edge,
    void*    user_data
);

void interrupt_schedule_register(
    interrupt_schedule_fire_isr_fn callback,
    void*                          user_data
);

bool interrupt_schedule_fire_at(uint32_t target_counter32);
void interrupt_schedule_fire_cancel(void);
bool interrupt_schedule_fire_armed(void);

// Last event-authored VCLOCK-domain coordinate.  Fallback/diagnostic only;
// not a live clock.
uint32_t interrupt_last_known_counter32(void);
bool interrupt_last_known_counter32_valid(void);

// Diagnostic counters exposed for REPORT / external inspection.
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