// ============================================================================
// process_interrupt.h (QTimer version — v18)
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
// Provider hardware (v18 — QTimer migration):
//
//   • QTimer3 CH2  — OCXO1 10 MHz external clock on pin 14 (GPIO_AD_B1_02)
//   • QTimer3 CH3  — OCXO2 10 MHz external clock on pin 15 (GPIO_AD_B1_03)
//   • GPIO6789     — PPS rising edge on pin 1
//
//   QTimer1 remains owned by TimePop (GNSS VCLOCK + scheduler).
//   QTimer2 is untouched.
//   Pin 13 (LED_BUILTIN) is preserved for fault Morse annunciator.
//
// Pin-to-QTimer mapping (verified from core_pins.h / i.MX RT1062 RM):
//
//   Pin 14 = GPIO_AD_B1_02 → ALT1 = QTIMER3_TIMER2 (CH2)  → PCS(2)
//   Pin 15 = GPIO_AD_B1_03 → ALT1 = QTIMER3_TIMER3 (CH3)  → PCS(3)
//
//   Pin 11 = GPIO_B0_02    → ALT1 = QTIMER1_TIMER2 (CH2) — TimePop compare.
//   DO NOT use pin 11 for OCXO input.
//
// ISR architecture:
//
//   Both OCXO channels share a single NVIC vector (IRQ_QTIMER3).
//   The ISR captures DWT_CYCCNT as its first instruction, then checks
//   status flags to determine which channel(s) fired.  This is the same
//   dispatcher pattern used by TimePop's qtimer1_irq_isr.
//
// Lifecycle:
//
//   1. process_interrupt_init_hardware() — QTimer3 clock gate, pin mux, enable
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

  uint32_t dwt_at_event = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;
  uint32_t dwt_event_correction_cycles = 0;
};

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  uint64_t prespin_target_gnss_ns = 0;
  uint64_t event_target_gnss_ns = 0;

  bool prespin_active = false;
  bool prespin_fired = false;
  bool prespin_timed_out = false;

  uint32_t shadow_dwt = 0;
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
// Counter access — 16-bit raw reads (for diagnostics only)
// ============================================================================

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);

// ============================================================================
// Helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);