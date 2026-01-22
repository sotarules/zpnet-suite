#pragma once

// ============================================================================
// TimePop — Process Interface (Control Plane)
// ============================================================================
//
// This header exposes TimePop as a managed system process.
//
// It is intended ONLY for:
//   • system initialization (setup())
//   • process registration
//   • diagnostics / reporting (e.g., REPORT command)
//
// It must NOT be included by general timer consumers.
// ============================================================================

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

// Initialize TimePop hardware and internal state.
// Must be called once during system startup *before* timers are used.
void timepop_init(void);

// -----------------------------------------------------------------------------
// Process Registration
// -----------------------------------------------------------------------------

// Register the TimePop process with the system process registry.
// Enables commands such as REPORT and future diagnostics.
void process_timepop_register(void);

// -----------------------------------------------------------------------------
// TimePop diagnostics (read-only)
// -----------------------------------------------------------------------------

uint32_t timepop_get_pit_tick_count(void);
uint32_t timepop_get_last_remaining(void);
uint32_t timepop_get_zero_hits(void);

