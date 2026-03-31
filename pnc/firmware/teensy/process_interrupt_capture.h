#pragma once

// ============================================================================
// process_interrupt_capture.h
// ============================================================================
//
// Generalized interrupt-time capture engine for timing-sensitive paths.
//
// This module normalizes timing truth in ISR context so callers do not need to
// reconstruct it later from scattered snapshots and post-hoc correction.
//
// Canonical outputs per event:
//   - dwt_at_event        : best reconstructed DWT cycle count at the event edge
//   - gnss_ns_at_event    : best reconstructed GNSS nanosecond time at the event
//   - counter32_at_event  : source-domain 32-bit event-aligned counter when valid
//   - optional diagnostics: derivation/provenance used to produce the above
//
// Public abstraction boundary:
//   Callers configure a capture class (GPIO edge, QTimer compare, GPT1 compare,
//   GPT2 compare), arm targets where applicable, run the deterministic pre-spin,
//   and consume normalized ISR-context callbacks.
//
// The caller does NOT perform:
//   - low/high-word reconstruction for event counters
//   - ISR-latency correction
//   - DWT-at-edge reconstruction
//   - GNSS-at-edge projection
//
// Those all live here.
// ============================================================================

#include <stdint.h>
#include <stdbool.h>

#include "process_clocks_internal.h"

// ============================================================================
// Public enums
// ============================================================================

enum class interrupt_capture_class_t : uint8_t {
  GPIO_EDGE = 0,
  QTIMER_COMPARE,
  GPT1_COMPARE,
  GPT2_COMPARE,
};

enum class interrupt_capture_status_t : uint8_t {
  OK = 0,
  HOLD,
  FAULT,
};

enum class interrupt_capture_rearm_mode_t : uint8_t {
  NONE = 0,
  ONE_SHOT,
  PERIODIC,
  CALLBACK_DRIVEN,
};

// ============================================================================
// Raw capture — source-local hardware truth harvested in ISR context
// ============================================================================

struct interrupt_capture_raw_t {
  uint32_t dwt_isr_entry = 0;
  uint32_t dwt_after_capture = 0;

  uint16_t counter16_at_event = 0;   // e.g. COMP1 low word or equivalent
  uint16_t compare16 = 0;

  uint32_t counter32_live = 0;       // live 32-bit counter read in ISR if available
  uint32_t qtimer32_live = 0;
  uint32_t gpt_cnt_live = 0;

  uint32_t irq_status = 0;
  uint32_t irq_flags_cleared = 0;
};

// ============================================================================
// Live latency telemetry — carried alongside normalized event truth
// ============================================================================

struct interrupt_capture_latency_t {
  uint32_t last_spin_dwt = 0;
  uint32_t prev_spin_dwt = 0;

  uint32_t entry_latency_cycles = 0;
  uint32_t entry_latency_cycles_default = 0;
  uint32_t entry_latency_cycles_min = 0;
  uint32_t entry_latency_cycles_max = 0;

  uint64_t sample_count = 0;
  double   mean_cycles = 0.0;
  double   m2_cycles = 0.0;

  bool deterministic_spin_active = false;
  bool live_profile_valid = false;
};

// ============================================================================
// Optional diagnostics — derivation trail used to produce canonical values
// ============================================================================

struct interrupt_capture_diag_t {
  bool enabled = false;

  uint32_t dwt_isr_entry = 0;
  uint32_t dwt_before_capture = 0;
  uint32_t dwt_after_capture = 0;

  uint32_t spin_last_dwt = 0;
  uint32_t spin_prev_dwt = 0;

  uint32_t chosen_entry_latency_cycles = 0;
  uint32_t candidate_latency_default_cycles = 0;
  uint32_t candidate_latency_live_cycles = 0;

  uint32_t chosen_dwt_at_event = 0;
  uint64_t chosen_gnss_ns_at_event = 0;
  uint32_t chosen_counter32_at_event = 0;

  uint16_t raw_counter16_at_event = 0;
  uint16_t raw_compare16 = 0;
  uint32_t raw_counter32_live = 0;

  uint32_t candidate_counter32_a = 0;
  uint32_t candidate_counter32_b = 0;
  bool used_counter32_candidate_b = false;

  bool used_live_profile = false;
  bool used_default_profile = false;
};

// ============================================================================
// Canonical normalized event — this is what the ISR-context callback receives
// ============================================================================

struct interrupt_capture_event_t {
  interrupt_capture_class_t cls = interrupt_capture_class_t::GPIO_EDGE;
  interrupt_capture_status_t status = interrupt_capture_status_t::OK;

  uint32_t dwt_at_event = 0;
  bool dwt_valid = false;

  uint64_t gnss_ns_at_event = 0;
  bool gnss_valid = false;

  uint32_t counter32_at_event = 0;
  bool counter32_valid = false;

  uint32_t latency_cycles_used = 0;

  interrupt_capture_raw_t raw {};
  interrupt_capture_latency_t latency {};
};

struct interrupt_capture_rearm_decision_t {
  bool rearm = false;
  uint32_t next_counter32_target = 0;
  uint64_t next_target_gnss_ns = 0;
};

// ============================================================================
// Source-specific function hooks hidden behind capture class abstraction
// ============================================================================

using interrupt_capture_raw_fn =
    void (*)(interrupt_capture_raw_t& out_raw);

using interrupt_capture_counter32_fn =
    uint32_t (*)(const interrupt_capture_raw_t& raw,
                 interrupt_capture_diag_t* diag,
                 bool* out_valid);

using interrupt_capture_gnss_fn =
    uint64_t (*)(const interrupt_capture_raw_t& raw,
                 uint32_t dwt_at_event,
                 uint32_t counter32_at_event,
                 bool counter32_valid,
                 const interrupt_capture_latency_t& latency,
                 interrupt_capture_diag_t* diag,
                 bool* out_valid);

using interrupt_capture_callback_fn =
    void (*)(const interrupt_capture_event_t& event,
             const interrupt_capture_diag_t* diag);

using interrupt_capture_rearm_fn =
    interrupt_capture_rearm_decision_t (*)(const interrupt_capture_event_t& event,
                                           const interrupt_capture_diag_t* diag);

using interrupt_capture_spin_window_fn =
    bool (*)(uint64_t now_gnss_ns,
             uint64_t target_gnss_ns);

// ============================================================================
// User config
// ============================================================================

struct interrupt_capture_config_t {
  const char* name = nullptr;
  interrupt_capture_class_t cls = interrupt_capture_class_t::GPIO_EDGE;
  interrupt_capture_rearm_mode_t rearm_mode = interrupt_capture_rearm_mode_t::NONE;

  uint32_t default_entry_latency_cycles = 0;
  bool diagnostics_enabled = false;

  interrupt_capture_raw_fn raw_capture_fn = nullptr;
  interrupt_capture_counter32_fn counter32_at_event_fn = nullptr;
  interrupt_capture_gnss_fn gnss_at_event_fn = nullptr;
  interrupt_capture_callback_fn callback_fn = nullptr;
  interrupt_capture_rearm_fn rearm_fn = nullptr;
  interrupt_capture_spin_window_fn spin_window_start_fn = nullptr;
};

// ============================================================================
// Engine class
// ============================================================================

class interrupt_capture_engine_t {
public:
  explicit interrupt_capture_engine_t(const interrupt_capture_config_t& cfg);

  void arm(uint64_t target_gnss_ns, uint32_t counter32_target = 0);
  void disarm();
  bool armed() const;

  void spin_step();
  void handle_interrupt();

  const interrupt_capture_config_t& config() const;
  const interrupt_capture_latency_t& latency() const;

private:
  uint32_t choose_entry_latency_cycles(const interrupt_capture_raw_t& raw,
                                       interrupt_capture_diag_t& diag) const;
  void update_live_latency(uint32_t entry_latency_cycles);
  void rearm_from_isr(const interrupt_capture_event_t& event,
                      const interrupt_capture_diag_t* diag);

private:
  interrupt_capture_config_t cfg_ {};
  interrupt_capture_latency_t latency_ {};

  bool armed_ = false;
  uint64_t next_target_gnss_ns_ = 0;
  uint32_t next_counter32_target_ = 0;
};

// ============================================================================
// Helper functions / default policies
// ============================================================================

bool interrupt_capture_default_spin_window(uint64_t now_gnss_ns,
                                           uint64_t target_gnss_ns);

uint64_t interrupt_capture_default_gnss_projection(const interrupt_capture_raw_t& raw,
                                                   uint32_t dwt_at_event,
                                                   uint32_t counter32_at_event,
                                                   bool counter32_valid,
                                                   const interrupt_capture_latency_t& latency,
                                                   interrupt_capture_diag_t* diag,
                                                   bool* out_valid);

interrupt_capture_rearm_decision_t interrupt_capture_no_rearm(
    const interrupt_capture_event_t& event,
    const interrupt_capture_diag_t* diag);

// ============================================================================
// Class helpers — used by callers to pick the right backend semantics
// ============================================================================

const char* interrupt_capture_class_str(interrupt_capture_class_t cls);

// QTimer compare helpers
uint32_t interrupt_capture_qtimer_counter32_at_event(const interrupt_capture_raw_t& raw,
                                                     interrupt_capture_diag_t* diag,
                                                     bool* out_valid);

// GPT compare helpers
uint32_t interrupt_capture_gpt_counter32_at_event(const interrupt_capture_raw_t& raw,
                                                  interrupt_capture_diag_t* diag,
                                                  bool* out_valid);

// GPIO edge helpers — no canonical event counter by default
uint32_t interrupt_capture_gpio_no_counter32(const interrupt_capture_raw_t& raw,
                                             interrupt_capture_diag_t* diag,
                                             bool* out_valid);