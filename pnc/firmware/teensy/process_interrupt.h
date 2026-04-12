// ============================================================================
// process_interrupt.h (QTimer version — v24)
// ============================================================================
//
// Shared interrupt authority + subscriber runtime.
//
// process_interrupt is the sole authority for all precision interrupt hardware.
//
// Core laws:
//
//   • Clients subscribe by logical identity (kind), never by provider details.
//   • process_interrupt owns provider custody and ISR vectors.
//   • ARM_DWT_CYCCNT is captured as the first instruction in every ISR.
//   • The callback contract is expressed in canonical event truth.
//   • GNSS nanoseconds are the public timing language of the subsystem.
//   • DWT remains the precision substrate for capture and diagnostics.
//   • Pre-spin is an integral part of interrupt handling for PPS / OCXO1 / OCXO2.
//   • process_interrupt owns the pre-spin shadow-write loop and scheduling.
//   • TimePop owns QTimer1. process_interrupt owns QTimer3 CH2+CH3.
//   • Diagnostics are always available to clients through the callback contract.
//
// OCXO next-second prediction:
//
//   process_interrupt maintains a per-OCXO prediction expressed in GNSS
//   nanoseconds. The predictor is intentionally simple: last lawful measured
//   OCXO second is used as the next-second prediction.
//
//   This prediction exists ONLY to place the OCXO prespin window. It is not
//   canonical timing truth and must not be used for OCXO residual math.
//
//   Prespin for OCXO1 / OCXO2 is gated on prediction validity. Until a lawful
//   measured OCXO second is available, OCXO events use only the fixed ISR
//   overhead correction path with no prespin anomaly penalty.
//
// Corrected provider model:
//
//   • QTimer3 CH2  — OCXO1 10 MHz external clock on pin 14
//   • QTimer3 CH3  — OCXO2 10 MHz external clock on pin 15
//   • GPIO6789     — PPS rising edge on pin 1
//
//   QTimer1 remains owned by TimePop (GNSS VCLOCK + scheduler).
//   QTimer2 is untouched.
//   Pin 13 (LED_BUILTIN) is preserved for fault Morse annunciator.
//
// Important architectural correction:
//
//   QTimer3 CH2 and CH3 are treated as 16-bit cadence lanes, NOT as free-running
//   32-bit OCXO clocks.  No canonical OCXO time is obtained by reading those
//   counters live.  Instead:
//
//     • lawful compare interrupts define event truth
//     • DWT captures the event in ISR context
//     • GNSS nanoseconds are reconstructed mathematically from the DWT bridge
//     • counter32_at_event is a software-reconstructed logical OCXO count at the
//       lawful one-second edge, not a raw hardware 32-bit counter read
//
// Pin-to-QTimer mapping:
//
//   • Pin 14 → QTIMER3_TIMER2 (CH2)
//   • Pin 15 → QTIMER3_TIMER3 (CH3)
//
// ISR architecture:
//
//   Both OCXO channels share a single NVIC vector (IRQ_QTIMER3).
//   The ISR captures DWT_CYCCNT as its first instruction, then checks
//   status flags to determine which channel(s) fired.  This mirrors the
//   shared-dispatch pattern used elsewhere in the timing subsystem.
//
// Lifecycle:
//
//   1. process_interrupt_init_hardware() — QTimer3 clock gate, pin mux, bring-up
//   2. process_interrupt_init()          — runtime slot creation, descriptor wiring
//   3. process_interrupt_enable_irqs()   — ISR vector installation + NVIC enable
//   4. interrupt_subscribe() / start()   — client wiring + activation
//
// ============================================================================

#pragma once

#include "timepop.h"
#include <stdint.h>
#include <stdbool.h>

void interrupt_prespin_service(timepop_ctx_t*, timepop_diag_t*, void*);

// ============================================================================
// Subscriber identities
// ============================================================================

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  PPS,
  OCXO1,
  OCXO2,
  TIME_TEST,
};

// ============================================================================
// Provider identities
// ============================================================================

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER3,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER3_CH2_COMP,    // OCXO1 on QTimer3 channel 2
  QTIMER3_CH3_COMP,    // OCXO2 on QTimer3 channel 3
  GPIO_EDGE,
};

// ============================================================================
// Event + diagnostics
// ============================================================================

enum class interrupt_event_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  // DWT cycle count reconstructed to the actual event boundary by applying the
  // fixed ISR overhead correction to the raw ISR-entry capture.
  uint32_t dwt_at_event = 0;

  // GNSS nanosecond corresponding to dwt_at_event via the canonical DWT↔GNSS bridge.
  uint64_t gnss_ns_at_event = 0;

  // For PPS:
  //   Raw 32-bit QTimer1 VCLOCK value captured at the lawful PPS edge.
  //
  // For OCXO1 / OCXO2:
  //   Software-reconstructed logical OCXO count at the lawful one-second edge.
  //   This is NOT a live 32-bit hardware counter read from QTimer3.
  uint32_t counter32_at_event = 0;

  // Fixed cycle correction subtracted from dwt_isr_entry_raw to obtain dwt_at_event.
  uint32_t dwt_event_correction_cycles = 0;
};

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  // Scheduled prespin target for this event in GNSS nanoseconds.
  uint64_t prespin_target_gnss_ns = 0;

  // Intended event boundary for this event in GNSS nanoseconds.
  uint64_t event_target_gnss_ns = 0;

  bool prespin_active = false;
  bool prespin_fired = false;
  bool prespin_timed_out = false;

  // Last DWT value written by the prespin shadow loop.
  uint32_t shadow_dwt = 0;

  // First DWT capture taken on ISR entry.
  uint32_t dwt_isr_entry_raw = 0;

  uint32_t approach_cycles = 0;
  uint32_t shadow_to_isr_cycles = 0;

  uint32_t dwt_at_event_adjusted = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;
  uint32_t dwt_event_correction_cycles = 0;

  uint32_t prespin_arm_count = 0;
  uint32_t prespin_complete_count = 0;
  uint32_t prespin_timeout_count = 0;
  uint32_t anomaly_count = 0;
};

// ============================================================================
// Subscriber callback signature
// ============================================================================

using interrupt_subscriber_event_fn =
    void (*)(const interrupt_event_t& event,
             const interrupt_capture_diag_t* diag,
             void* user_data);

// ============================================================================
// Public subscription surface
// ============================================================================

struct interrupt_subscription_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_subscriber_event_fn on_event = nullptr;
  void* user_data = nullptr;
};

// ============================================================================
// Subsystem lifecycle
// ============================================================================

void process_interrupt_init_hardware(void);
void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

// ============================================================================
// Subscriber control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

// ============================================================================
// PPS drumbeat control
// ============================================================================

void interrupt_request_pps_zero(void);
bool interrupt_pps_zero_pending(void);

// ============================================================================
// Last event / diag access
// ============================================================================

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind);

// ============================================================================
// Shared IRQ authority entry points
// ============================================================================

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_isr_entry_raw);

// ============================================================================
// Counter access — 16-bit raw reads (diagnostics only)
// ============================================================================
//
// These are raw hardware reads from QTimer3 CH2 / CH3 and are never canonical
// timing truth. They exist only for instrumentation, reporting, and sanity checks.
//

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);

// ============================================================================
// Helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);

uint32_t interrupt_qtimer1_counter32_now(void);