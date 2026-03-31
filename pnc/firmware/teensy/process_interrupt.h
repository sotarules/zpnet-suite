#pragma once

// ============================================================================
// process_interrupt.h
// ============================================================================
//
// Shared interrupt authority + subscriber runtime.
//
// Philosophy:
//   • Clients subscribe by known identity (kind), not by lane details.
//   • The interrupt subsystem owns provider/lane custody.
//   • Clients supply one ISR-context callback.
//   • Callback always receives canonical event + diag.
//   • Callback returns the next absolute GNSS target time, or no-follow-on.
//   • TimePop is used by the interrupt subsystem for pre-spin scheduling.
//   • Required event fields are guaranteed by contract.
//   • If the subsystem cannot produce a required field, that is an integrity
//     failure and the design must be fixed.
//   • For QTIMER compare subscribers, the authoritative counter value at event
//     time is the full 32-bit target count that we armed.
//   • Interrupt-time register reads are used only for verification and
//     diagnostics, never to invent event time after the fact.
//
// ============================================================================

#include <stdint.h>
#include <stdbool.h>

#include "process_clocks_internal.h"
#include "timepop.h"

// ============================================================================
// Subscriber identities (logical clients)
// ============================================================================

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  TIMEPOP,
  TIME_TEST,
  PPS,
  VCLOCK,
  OCXO1,
  OCXO2,
  PHOTODIODE,
};

// ============================================================================
// Internal provider / lane identities (hardware custody)
// ============================================================================

enum class interrupt_provider_kind_t : uint8_t {
  NONE = 0,
  QTIMER1,
  GPT1,
  GPT2,
  GPIO6789,
};

enum class interrupt_lane_t : uint8_t {
  NONE = 0,
  QTIMER1_CH2,
  QTIMER1_CH3,
  GPT1_COMPARE1,
  GPT2_COMPARE1,
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
// Profiled latency for QTimer interrupts
// ============================================================================
static constexpr uint32_t TDC_ISR_LATENCY_CYCLES_QTIMER = 54;

// ============================================================================
// Raw capture — source-local hardware truth harvested in ISR context
// ============================================================================

struct interrupt_capture_raw_t {
  // First-line ISR DWT captured at the shared IRQ entry point, before any other
  // work in the software-controlled ISR path.
  uint32_t dwt_isr_entry_raw = 0;

  // DWT after raw-capture harvesting, for instrumentation only.
  uint32_t dwt_after_capture = 0;

  // Compare/event facts harvested in ISR context.
  uint16_t compare16 = 0;

  // Verification registers harvested in ISR context.
  uint16_t verify_low16 = 0;
  uint16_t verify_high16 = 0;

  uint32_t irq_status = 0;
  uint32_t irq_flags_cleared = 0;
};

// ============================================================================
// Live latency telemetry — carried alongside normalized event truth
// ============================================================================

struct interrupt_capture_latency_t {
  uint32_t last_spin_dwt = 0;
  uint32_t prev_spin_dwt = 0;

  uint32_t dwt_event_correction_cycles = 0;
  uint32_t dwt_event_correction_cycles_default = 0;
  uint32_t dwt_event_correction_cycles_min = 0;
  uint32_t dwt_event_correction_cycles_max = 0;

  uint64_t sample_count = 0;
  double   mean_cycles = 0.0;
  double   m2_cycles = 0.0;

  bool deterministic_spin_active = false;
  bool live_profile_valid = false;
};

// ============================================================================
// Diagnostics — always available to subscribers
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  uint32_t shadow_dwt = 0;
  bool     shadow_valid = false;

  // Delta between loop shadow DWT and first-line ISR DWT. This should always be
  // tracked, even if later it converges to a tight constant.
  uint32_t shadow_to_isr_entry_cycles = 0;

  uint32_t dwt_isr_entry_raw = 0;
  uint32_t dwt_after_capture = 0;

  uint32_t spin_last_dwt = 0;
  uint32_t spin_prev_dwt = 0;

  // Formal DWT event correction.
  uint32_t dwt_event_correction_cycles = 0;
  uint32_t candidate_correction_default_cycles = 0;
  uint32_t candidate_correction_live_cycles = 0;
  bool     used_live_profile = false;
  bool     used_default_profile = false;

  uint32_t dwt_at_event_adjusted = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;

  // Raw verification facts.
  uint16_t raw_compare16 = 0;
  uint16_t raw_verify_low16 = 0;
  uint16_t raw_verify_high16 = 0;

  // QTIMER alias / neighborhood verification.
  uint16_t expected_low16 = 0;
  uint16_t expected_high16 = 0;
  bool     verify_high16_matches = false;
  bool     verify_high16_is_previous = false;
};

// ============================================================================
// Canonical event — what subscribers receive in ISR context
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  // Fully adjusted event-time DWT, formally corrected before the callback sees
  // it.
  uint32_t dwt_at_event = 0;

  // Best corresponding GNSS nanoseconds at event time.
  uint64_t gnss_ns_at_event = 0;

  // Authoritative source counter value at event time.
  // For QTIMER compare subscribers this is the full armed 32-bit target count.
  uint32_t counter32_at_event = 0;

  // Formal DWT correction applied.
  uint32_t dwt_event_correction_cycles = 0;

  interrupt_capture_raw_t raw {};
  interrupt_capture_latency_t latency {};
};

// ============================================================================
// Subscriber callback return
// ============================================================================

struct interrupt_next_target_t {
  bool schedule_next = false;
  uint64_t target_gnss_ns = 0;
};

// ============================================================================
// Subscriber callback signature (ISR context)
// ============================================================================

using interrupt_subscriber_event_fn =
    interrupt_next_target_t (*)(const interrupt_event_t& event,
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
// Subscriber control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub);
bool interrupt_start(interrupt_subscriber_kind_t kind);
bool interrupt_stop(interrupt_subscriber_kind_t kind);

// Used by known clients (e.g. TIME_TEST / TIMEPOP) to request a fresh cycle.
// The target is the absolute GNSS time of the desired event.
// The interrupt subsystem applies prespin lead internally.
bool interrupt_schedule_target(interrupt_subscriber_kind_t kind,
                               uint64_t target_gnss_ns);

// ============================================================================
// Shared IRQ authority entry points
// ============================================================================

void process_interrupt_qtimer1_irq(void);
void process_interrupt_gpt1_irq(void);
void process_interrupt_gpt2_irq(void);
void process_interrupt_gpio6789_irq(void);

// ============================================================================
// Subsystem lifecycle / reporting
// ============================================================================

void process_interrupt_init(void);
void process_interrupt_register(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);

// ============================================================================
// Helper functions / defaults used internally and by known subscribers
// ============================================================================

uint64_t interrupt_capture_default_gnss_projection(uint32_t dwt_at_event_adjusted);

// Formal DWT adjustment hook.
// For now this may simply subtract a constant per provider/lane, but it is an
// explicit, first-class step in the event reconstruction flow.
uint32_t interrupt_adjust_dwt_at_event(interrupt_subscriber_kind_t kind,
                                       interrupt_provider_kind_t provider,
                                       interrupt_lane_t lane,
                                       uint32_t dwt_isr_entry_raw,
                                       interrupt_capture_diag_t* diag);