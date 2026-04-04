// ============================================================================
// process_interrupt.h
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
//   • TimePop owns QTimer1. process_interrupt owns GPT1/GPT2 provider custody.
//   • Diagnostics are always available to clients through the callback contract.
//   • There is no best-effort reconstruction path if sacred capture facts are absent.
//
// Lifecycle:
//
//   1. process_interrupt_init_hardware() — GPT clock gates, pin mux, enable
//   2. process_interrupt_init()          — runtime slot creation, descriptor wiring
//   3. process_interrupt_enable_irqs()   — ISR vector installation + NVIC enable
//   4. interrupt_subscribe() / start()   — client wiring + activation
//
// ============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Subscriber identities (logical clients)
// ============================================================================

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  PPS,
  OCXO1,
  OCXO2,
  TIME_TEST,
};

// ============================================================================
// Internal provider / lane identities
// ============================================================================

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  GPT1,
  GPT2,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  GPT1_COMPARE3,
  GPT2_COMPARE3,
  GPIO_EDGE,
};

// ============================================================================
// Canonical event status
// ============================================================================

enum class interrupt_event_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

// ============================================================================
// Canonical interrupt event
// ============================================================================
//
// This is the authoritative event truth handed forward from the interrupt
// boundary. It describes what happened, when it happened, and what provider
// produced it.
//
// Fields:
//   dwt_at_event:
//     DWT cycle count at the event boundary after fixed-overhead correction.
//
//   gnss_ns_at_event:
//     Absolute GNSS nanosecond at the event boundary when known.
//
//   counter32_at_event:
//     Logical event count or provider-side second-count index.
//
//   dwt_event_correction_cycles:
//     Fixed correction applied to ISR-entry DWT to derive dwt_at_event.
//
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  uint32_t dwt_at_event = 0;
  uint64_t gnss_ns_at_event = 0;

  // For PPS this is the PPS count since runtime start.
  // For OCXO second-boundary subscribers this is the second-count index
  // since runtime start.
  uint32_t counter32_at_event = 0;

  uint32_t dwt_event_correction_cycles = 0;
};

// ============================================================================
// Optional capture diagnostics
// ============================================================================
//
// This block provides capture-path diagnostics for clients that need to inspect
// pre-spin behavior or ISR-side timing detail.
//
// Terminology:
//   approach_cycles:
//     Total cycles from pre-spin start until interrupt fire.
//
//   shadow_to_isr_cycles:
//     Cycles from the final shadow DWT sample to ISR entry.
//
// These are not interchangeable. approach_cycles is the schedule-tuning metric.
// shadow_to_isr_cycles is the tail-latency metric.
//
// ============================================================================

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

  // Total prespin runtime until interrupt fire.
  uint32_t approach_cycles = 0;

  // Tail distance from last shadow-write sample to ISR entry.
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
//
// Callbacks receive canonical event truth plus optional diagnostics.
//
// The event is the authoritative fact.
// The diagnostic block is supplemental.
//
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

/// Phase 1: GPT clock gates, pin mux, timer enable. Call early in boot.
void process_interrupt_init_hardware(void);

/// Phase 2: Runtime slot creation and descriptor wiring.
void process_interrupt_init(void);

/// Phase 3: ISR vector installation + NVIC enable. Call only after TimePop
/// and other core infrastructure are alive. This is the moment interrupts
/// become live.
void process_interrupt_enable_irqs(void);

/// Process-framework registration (REPORT command).
void process_interrupt_register(void);

// ============================================================================
// Subscriber control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

// ============================================================================
// Last event / diag access
// ============================================================================

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind);

// ============================================================================
// Shared IRQ authority entry points
// ============================================================================
//
// These are called only from process_interrupt-owned ISR vectors.
//
// ============================================================================

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpt1_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpt2_irq(uint32_t dwt_isr_entry_raw);

// ============================================================================
// Provider ownership helpers exposed to other subsystems when needed
// ============================================================================

uint32_t interrupt_gpt1_counter_now(void);
uint32_t interrupt_gpt2_counter_now(void);

// ============================================================================
// Helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);