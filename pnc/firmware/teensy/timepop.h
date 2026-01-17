/**
 * -----------------------------------------------------------------------------
 *  TimePop Timer Module Interface (timepop.h)
 * -----------------------------------------------------------------------------
 *
 *  The TimePop module provides a software-emulated timer facility built on
 *  a single hardware PIT (Periodic Interrupt Timer).
 *
 *  It creates the illusion of an arbitrarily large number of independent
 *  timers while preserving strict interrupt-driven causality.
 *
 *  TimePop is a core ZPNet runtime service.
 *
 *  DESIGN PRINCIPLES:
 *  ------------------
 *  - Uses exactly one hardware PIT as a global heartbeat
 *  - Supports many concurrent timers without hardware allocation
 *  - All callbacks execute in scheduled (non-ISR) context
 *  - The PIT ISR only records timer expiration; it never runs callbacks
 *  - Timer dispatch is explicitly delegated by the runtime harness
 *
 *  EXECUTION CONTEXT RULES:
 *  ------------------------
 *  - timepop_init() must be called during system setup
 *  - timepop_dispatch() must be called from the main runtime loop
 *  - No TimePop callback will ever execute inside an ISR
 *
 *  TIME RESOLUTION:
 *  ----------------
 *  The base resolution is determined by the PIT tick interval configured
 *  internally by the module.
 *
 *  Requested delays are rounded up to the nearest supported tick.
 *
 *  TimePop guarantees *ordering* and *causality*, not sub-tick accuracy.
 *
 *  TERMINOLOGY:
 *  ------------
 *  - "Timer pop" refers to the scheduled expiration of a timer
 *  - "Callback" refers to the function invoked when a timer pop occurs
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Public Types
// -----------------------------------------------------------------------------

/**
 * Opaque handle identifying a scheduled timer.
 *
 * Handles are stable for the lifetime of the timer and may be used to
 * cancel or query timers. A handle becomes invalid once the timer fires
 * or is canceled.
 */
typedef uint32_t timepop_handle_t;

/**
 * Time units supported when scheduling timers.
 */
typedef enum {
  TIMEPOP_UNITS_MICROSECONDS,
  TIMEPOP_UNITS_MILLISECONDS
} timepop_units_t;

/**
 * Callback invoked when a timer expires.
 *
 * Callbacks execute in scheduled (non-ISR) context and may:
 *  - Schedule additional timers
 *  - Interact with other runtime services
 *  - Emit events or log output
 *
 * Callbacks must not block indefinitely.
 */
typedef void (*timepop_callback_t)(void* context);

// -----------------------------------------------------------------------------
// Module Lifecycle
// -----------------------------------------------------------------------------

/**
 * Initialize the TimePop module.
 *
 * This function:
 *  - Configures the hardware PIT used as the global heartbeat
 *  - Initializes all internal timer state
 *
 * It must be called exactly once during system setup.
 */
void timepop_init();

// -----------------------------------------------------------------------------
// Timer Scheduling
// -----------------------------------------------------------------------------

/**
 * Schedule a one-shot timer.
 *
 * @param delay        Number of time units to wait before firing
 * @param units        Units of the delay (microseconds or milliseconds)
 * @param callback     Function to invoke when the timer expires
 * @param context      Opaque pointer passed to the callback
 * @param name         Optional logical name (for debugging / visibility)
 *
 * @return A handle identifying the scheduled timer, or 0 on failure.
 *
 * A delay of zero is legal and schedules the callback for the next
 * dispatch opportunity.
 */
timepop_handle_t timepop_schedule(
  uint32_t delay,
  timepop_units_t units,
  timepop_callback_t callback,
  void* context,
  const char* name
);

// -----------------------------------------------------------------------------
// Timer Cancellation
// -----------------------------------------------------------------------------

/**
 * Cancel a previously scheduled timer.
 *
 * @param handle  Handle returned by timepop_schedule()
 *
 * @return true if the timer was canceled, false if the handle was invalid
 *         or the timer had already fired.
 *
 * Cancellation is idempotent; canceling the same handle multiple times
 * is safe.
 */
bool timepop_cancel(timepop_handle_t handle);

// -----------------------------------------------------------------------------
// Timer Dispatch
// -----------------------------------------------------------------------------

/**
 * Dispatch expired timers.
 *
 * This function:
 *  - Identifies timers whose delay has elapsed
 *  - Invokes their callbacks in scheduled context
 *  - Cleans up internal timer state
 *
 * It must be called from the main runtime loop.
 *
 * If no timers are pending, this function returns immediately.
 */
void timepop_dispatch();

// -----------------------------------------------------------------------------
// Optional Introspection (Debug / Visibility)
// -----------------------------------------------------------------------------

/**
 * Return the number of currently active timers.
 *
 * Intended for debugging and visibility only.
 */
uint32_t timepop_active_count();
