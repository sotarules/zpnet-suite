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
//   • Clients subscribe by logical identity (kind), never by lane details.
//   • The interrupt subsystem owns provider/lane custody.
//   • The interrupt subsystem owns all ISR vectors (GPT1, GPT2, GPIO6789).
//   • ARM_DWT_CYCCNT is captured as the first instruction in every ISR.
//   • The callback contract is steeped in GNSS nanoseconds.
//   • DWT remains available as the precision substrate for last-mile timing.
//   • The callback always receives canonical event truth plus diagnostics.
//   • Diagnostics are always available; clients never need private capture
//     globals just to test the interrupt path.
//   • TimePop owns QTimer1. process_interrupt owns GPT1/GPT2 provider custody.
//   • Pre-spin is an integral part of interrupt handling for PPS / OCXO1 / OCXO2.
//   • process_interrupt owns the pre-spin shadow-write loop and scheduling.
//   • There is no best-effort fallback if pre-spin is missing or times out.
//
// Lifecycle:
//
//   1. process_interrupt_init_hardware() — GPT clock gates, pin mux, enable
//   2. process_interrupt_init()          — runtime slots, descriptor wiring
//   3. process_interrupt_enable_irqs()   — ISR vectors + NVIC enable
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
// Event + diag
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  uint32_t dwt_at_event = 0;
  uint64_t gnss_ns_at_event = 0;

  // For PPS this is the PPS count since runtime start.
  // For OCXO second-boundary subscribers this is the second-count index since start.
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
// Subscriber callback signature (ISR context)
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

/// Phase 2: Runtime slot creation, descriptor wiring.
void process_interrupt_init(void);

/// Phase 3: ISR vector installation + NVIC enable. Call after TimePop
/// and transport are alive. This is the moment hardware interrupts go live.
void process_interrupt_enable_irqs(void);

/// Process framework registration (REPORT command).
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
// Shared IRQ authority entry points (internal — called from owned ISR vectors)
// ============================================================================

void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpt1_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpt2_irq(uint32_t dwt_isr_entry_raw);

// ============================================================================
// Provider ownership helpers exposed for clocks when needed
// ============================================================================

uint32_t interrupt_gpt1_counter_now(void);
uint32_t interrupt_gpt2_counter_now(void);

// ============================================================================
// Helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);