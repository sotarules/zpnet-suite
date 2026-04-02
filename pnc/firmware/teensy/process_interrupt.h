// ============================================================================
// process_interrupt.h
// ============================================================================
//
// Shared interrupt authority + subscriber runtime.
//
// Core laws:
//   • Clients subscribe by logical identity (kind), never by lane details.
//   • The interrupt subsystem owns provider/lane custody.
//   • Every shared IRQ path captures DWT first. No exceptions.
//   • Every deterministic prespin loop does only one thing:
//       continuously shadow DWT until interrupted.
//   • The callback contract is steeped in GNSS nanoseconds.
//   • DWT remains available as the precision substrate for last-mile timing.
//   • The callback always receives canonical event truth plus diagnostics.
//   • Diagnostics are always available; clients never need private capture
//     globals just to test the interrupt path.
//   • Clients return the next exact interrupt time in absolute GNSS ns, or
//     no-follow-on.
//   • process_interrupt owns prespin lead policy. Clients do not compute it.
//   • QTimer / GPT counters are authoritative only under interrupt custody;
//     their event-time values are handed to clients on a silver platter.
//   • Direct live register reads are verification / diagnostics only; they are
//     never used to invent event time after the fact.
//   • DWT and PPS are explicit special cases outside the general prohibition.
//   • process_interrupt owns the prespin shadow-write loop.  Clients never
//     maintain their own shadow DWT globals.
//
// ============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>

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
// Default correction constants (per-provider / lane defaults)
// ============================================================================

static constexpr uint32_t TDC_ISR_LATENCY_CYCLES_QTIMER = 54;

// process_interrupt owns prespin lead. Clients specify exact target time only.
static constexpr uint64_t INTERRUPT_PRESPIN_LEAD_NS_DEFAULT = 5000ULL;

// ============================================================================
// Raw capture — source-local hardware truth harvested in ISR context
// ============================================================================

struct interrupt_capture_raw_t {
  // First-line ISR DWT captured at the shared IRQ entry point, before any
  // other work in the software-controlled ISR path.
  uint32_t dwt_isr_entry_raw = 0;

  // DWT after raw-capture harvesting, for instrumentation only.
  uint32_t dwt_after_capture = 0;

  // Compare/event facts harvested in ISR context.
  uint16_t compare16 = 0;

  // Verification registers harvested in ISR context only.
  uint16_t verify_low16 = 0;
  uint16_t verify_high16 = 0;

  uint32_t irq_status = 0;
  uint32_t irq_flags_cleared = 0;
};

// ============================================================================
// Live latency telemetry — carried alongside normalized event truth
// ============================================================================

struct interrupt_capture_latency_t {
  // Deterministic prespin loop state.
  uint32_t last_spin_dwt = 0;
  uint32_t prev_spin_dwt = 0;

  // Formal DWT event correction applied for this event.
  uint32_t dwt_event_correction_cycles = 0;

  // Static default correction from the descriptor.
  uint32_t dwt_event_correction_cycles_default = 0;

  // Welford tracking of shadow_to_isr_entry_cycles (ISR latency profile).
  // This tracks the actual observed ISR entry latency across events,
  // providing a live empirical profile of the interrupt path.
  uint64_t sample_count = 0;
  double   mean_cycles = 0.0;
  double   m2_cycles = 0.0;
  uint32_t min_cycles = 0;
  uint32_t max_cycles = 0;

  bool deterministic_spin_active = false;
};

// ============================================================================
// Diagnostics — always available to subscribers
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  // Deterministic prespin / shadow facts.
  uint32_t shadow_dwt = 0;
  bool     shadow_valid = false;

  // Delta between prespin shadow DWT and first-line ISR DWT.
  uint32_t shadow_to_isr_entry_cycles = 0;

  // Shared ISR prologue facts.
  uint32_t dwt_isr_entry_raw = 0;
  uint32_t dwt_after_capture = 0;

  // Prespin loop continuity.
  uint32_t spin_last_dwt = 0;
  uint32_t spin_prev_dwt = 0;

  // Formal DWT event correction.
  uint32_t dwt_event_correction_cycles = 0;

  // Canonical event outputs.
  uint32_t dwt_at_event_adjusted = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;

  // Raw verification facts.
  uint16_t raw_compare16 = 0;
  uint16_t raw_verify_low16 = 0;
  uint16_t raw_verify_high16 = 0;

  // Expected target neighborhood verification.
  uint16_t expected_low16 = 0;
  uint16_t expected_high16 = 0;
  bool     verify_high16_matches = false;
  bool     verify_high16_is_previous = false;

  // Integrity / provenance flags.
  bool counter32_authoritative = false;
  bool gnss_projection_valid = false;
  bool prespin_established = false;

  // Verification outcome — true only when live register values are
  // consistent with the armed target.  False indicates a mismatch
  // (event status will be HOLD).
  bool verification_ok = false;
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
  // it. This is the precision substrate.
  uint32_t dwt_at_event = 0;

  // Best corresponding canonical GNSS nanoseconds at event time.
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

// The client supplies the exact desired event time in absolute GNSS ns.
// process_interrupt owns prespin lead policy internally.
bool interrupt_schedule_target(interrupt_subscriber_kind_t kind,
                               uint64_t target_gnss_ns);

// Optional explicit sacred target injection for known subscribers that already
// know the authoritative provider counter value at event time.
// This keeps migration impact low while allowing process_interrupt to become
// the canonical event-reconstruction layer.
bool interrupt_set_sacred_counter32_target(interrupt_subscriber_kind_t kind,
                                           uint32_t counter32_target);

// ============================================================================
// Last event / diag access — read the persisted per-subscriber state
//
// Clients (e.g., beta reporting) can read the last event and diag for any
// subscriber without maintaining their own parallel capture structs.
// Returns nullptr if the subscriber has not yet fired.
// ============================================================================

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind);
const interrupt_capture_diag_t* interrupt_last_diag(interrupt_subscriber_kind_t kind);

// ============================================================================
// Shared IRQ authority entry points
// ============================================================================

void process_interrupt_qtimer1_irq(void);
void process_interrupt_gpt1_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpt2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_gpio6789_irq(void);

// ============================================================================
// Subsystem lifecycle / reporting
// ============================================================================

void process_interrupt_init(void);
void process_interrupt_enable_irqs(void);
void process_interrupt_register(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);