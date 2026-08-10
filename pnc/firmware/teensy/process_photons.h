#pragma once

#include <stdint.h>

// ============================================================================
// PHOTONS — skeletal optical timing subsystem
// ============================================================================
//
// This is intentionally a scaffold, not the final PHOTONS scientific contract.
// Its immediate purpose is to validate:
//
//   PD200T comparator edge
//       -> process_interrupt PHOTODIODE subscription
//       -> process_photons ISR-safe capture
//       -> 1 Hz foreground aggregation
//       -> PHOTONS_FRAGMENT publication
//
// The toy structures below are deliberately small and disposable.  They give
// REPORT and PHOTONS_FRAGMENT enough shape to exercise custody and transport
// before the real pulse-train / lap / statistics model is implemented.
// ============================================================================

// Cumulative ISR-authored optical-edge state.
struct photons_toy_capture_t {
  uint32_t edge_count = 0;

  uint32_t last_edge_sequence = 0;
  uint32_t last_pps_sequence = 0;
  uint32_t last_dwt_at_edge = 0;
  uint32_t last_isr_entry_dwt_raw = 0;
  int32_t  isr_entry_to_edge_correction_cycles = 0;

  bool     interval_valid = false;
  uint32_t last_interval_cycles = 0;
  uint32_t min_interval_cycles = 0;
  uint32_t max_interval_cycles = 0;

  // Toy static carry-forward predictor:
  // prediction(n) = actual_interval(n - 1)
  bool     prediction_valid = false;
  uint32_t prediction_cycles = 0;
  int32_t  residual_cycles = 0;
};

// Once-per-second foreground snapshot published as PHOTONS_FRAGMENT.
struct photons_toy_fragment_t {
  uint32_t sequence = 0;
  uint32_t publish_count = 0;

  uint32_t edge_count_total = 0;
  uint32_t edges_this_second = 0;

  photons_toy_capture_t capture{};

  // Snapshot of process_interrupt's PHOTODIODE lane testimony.
  uint32_t interrupt_irq_count = 0;
  uint32_t interrupt_callback_count = 0;
  uint32_t interrupt_callback_missing_count = 0;
  uint32_t interrupt_inactive_edge_count = 0;
  uint32_t interrupt_source_pin = 0;
  uint32_t interrupt_last_callback_wall_cycles = 0;
  uint32_t interrupt_max_callback_wall_cycles = 0;
};

// Initialize PHOTONS runtime state, subscribe to the process_interrupt
// PHOTODIODE lane, start that lane, and arm the 1 Hz toy fragment publisher.
// Must run after process_interrupt_init() and timepop_init().
void process_photons_init(void);

// Register the PHOTONS process command surface.  The scaffold intentionally
// exposes only PHOTONS.REPORT.
void process_photons_register(void);

// Foreground/test accessors.  They return a coherent snapshot of the current
// toy structures without changing PHOTONS state.
bool photons_toy_capture_snapshot(photons_toy_capture_t* out);
bool photons_toy_fragment_snapshot(photons_toy_fragment_t* out);
