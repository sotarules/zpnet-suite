// ============================================================================
// process_interrupt.h — interrupt custody and rolling DWT integration
// ============================================================================
//
// process_interrupt is the sole authority for hardware interrupt custody
// for OCXO lanes (QTimer3 vector) and the PPS GPIO witness.  For the
// VCLOCK lane on QTimer1 CH3, process_interrupt is a hosted client of
// TimePop's QTimer1 vector — see process_timepop.h for the dispatch
// contract.
//
// Doctrine:
//
//   Three rolling DWT integrators, one per lane (VCLOCK, OCXO1, OCXO2),
//   each cadenced by its own dedicated QuadTimer compare channel at 1 kHz.
//   All three lanes have identical hardware-to-software latency profiles:
//   QuadTimer compare match → NVIC → ISR → DWT capture as first instruction.
//
//   Each integrator:
//     • Captures DWT as the FIRST INSTRUCTION of its 1 kHz ISR.
//     • Holds an immutable baseline captured once at the first tick.
//     • Accumulates cycles_since_baseline as an honest running tally
//       (sum of all per-tick intervals).
//     • Maintains a 1000-slot SMA ring of inter-tick DWT cycle deltas,
//       used solely for the smoothed cycles-per-second rate output.
//     • Every 1000 ticks emits a synthetic 1-second boundary:
//          synthetic_dwt = baseline + cumulative_cycles_since_baseline
//       which by construction equals the DWT captured at that boundary's
//       ISR entry (telescoping sum of intervals).  No projection; no
//       N × rate multiplication; no retroactive history rewriting.
//     • The boundary handler translates synthetic_dwt to GNSS ns via the
//       DWT↔GNSS bridge.  Bootstrap escape: the very first VCLOCK
//       boundary fires with gnss_ns_at_event = 0 (alpha installs the
//       epoch on receipt; from then on the bridge is valid for all lanes).
//     • The authored counter32 is a software-extended logical 32-bit count
//       that advances by exactly 10,000,000 per boundary.
//     • Per-interval distribution (window min/max/mean/stddev plus
//       all-time min/max) rides along in the capture diag on every
//       boundary.  Reveals per-tick ISR-entry jitter that endpoint-only
//       metrics (integ_diff, raw_diff) telescope away by algebraic
//       identity.
//
//   Subscribers receive boundary events.  No counter reads in canonical
//   paths.  No compare-latency corrections.  No raw/final duality.
//
//   The PPS GPIO edge is WITNESS-ONLY — it records its offset from the
//   most recent VCLOCK synthetic boundary as a diagnostic, and dispatches
//   nothing.
//
// Cadence sources:
//   VCLOCK: QTimer1 CH3 compare, +10000 ticks per interval (GNSS 10 MHz)
//           ISR hosted by TimePop (see timepop_register_qtimer1_ch3_isr).
//   OCXO1:  QTimer3 CH2 compare, +10000 ticks per interval (OCXO1 10 MHz)
//   OCXO2:  QTimer3 CH3 compare, +10000 ticks per interval (OCXO2 10 MHz)
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

enum class interrupt_subscriber_kind_t : uint8_t {
  NONE = 0,
  PPS,
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
// Boundary event — delivered to subscribers every 1000 ticks
// ============================================================================

struct interrupt_event_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_event_status_t status = interrupt_event_status_t::OK;

  // Synthetic DWT at the 1-second boundary — baseline + cumulative
  // cycles, equivalent to the DWT captured at this boundary's ISR
  // entry.
  uint32_t dwt_at_event = 0;

  // GNSS ns at the boundary, computed via the DWT↔GNSS bridge.  May be 0
  // for the bootstrap VCLOCK boundary (alpha treats that as epoch install).
  uint64_t gnss_ns_at_event = 0;

  // Software-extended logical 32-bit count, advances by exactly
  // 10,000,000 per boundary.
  uint32_t counter32_at_event = 0;
};

// ============================================================================
// Diagnostic surface — carried alongside every event
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = true;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;

  uint32_t dwt_isr_entry_raw = 0;
  int64_t  dwt_isr_entry_gnss_ns = -1;
  int64_t  dwt_isr_entry_minus_event_ns = 0;

  uint32_t dwt_at_event = 0;
  uint64_t gnss_ns_at_event = 0;
  uint32_t counter32_at_event = 0;

  bool     integrator_baseline_valid = false;
  uint32_t integrator_baseline_dwt = 0;
  uint64_t integrator_total_ticks = 0;
  uint32_t integrator_ring_fill = 0;
  uint64_t integrator_ring_sum = 0;
  uint64_t integrator_avg_cycles_per_sec = 0;
  uint32_t integrator_last_interval_cycles = 0;
  uint32_t integrator_boundary_emissions = 0;

  // Per-interval distribution, computed at boundary emission over the
  // 1000 intervals that summed to this boundary's endpoint delta.
  // Reveals the per-tick ISR-entry jitter that endpoint-level metrics
  // (integ_diff, raw_diff) telescope away by algebraic identity.
  // Window stats describe the most recent 1-second window; ever-min/max
  // are cumulative since baseline and catch rare excursions the ring
  // may have slid past.
  uint32_t integrator_interval_window_min_cycles = 0;
  uint32_t integrator_interval_window_max_cycles = 0;
  double   integrator_interval_window_mean_cycles = 0.0;
  double   integrator_interval_window_stddev_cycles = 0.0;
  uint32_t integrator_interval_min_ever_cycles = 0;
  uint32_t integrator_interval_max_ever_cycles = 0;

  // PPS GPIO witness (populated on the PPS subscriber's diag only).
  uint32_t gpio_edge_count = 0;
  uint32_t gpio_last_dwt = 0;
  int64_t  gpio_last_gnss_ns = 0;
  int64_t  gpio_minus_synthetic_ns = 0;

  uint32_t anomaly_count = 0;
};

// ============================================================================
// Subscription and event callback
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

// ISR entry points (invoked by vector shims).
void process_interrupt_gpio6789_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch2_irq(uint32_t dwt_isr_entry_raw);
void process_interrupt_qtimer3_ch3_irq(uint32_t dwt_isr_entry_raw);

uint64_t interrupt_vclock_cycles_per_second(void);

uint16_t interrupt_qtimer3_ch2_counter_now(void);
uint16_t interrupt_qtimer3_ch3_counter_now(void);
uint32_t interrupt_qtimer1_counter32_now(void);

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind);
const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider);
const char* interrupt_lane_str(interrupt_lane_t lane);