#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// TimePop — Public Timer Interface (Consumer API)
// ============================================================================
//
// TimePop provides a declarative, class-based timer facility.
// Consumers declare *roles* (classes), not durations.
//
// • All callbacks execute in scheduled (non-ISR) context
// • The PIT ISR only marks expiration
// • TimePop owns all wall-clock semantics
//
// Dual-PIT Architecture (Symmetric):
//
//   Every timer class has an interval type: milliseconds or microseconds.
//   The interval type determines which PIT channel drives the countdown.
//   All other semantics — arming, dispatch, recurring re-arm, cancel —
//   are identical regardless of interval type.
//
//   PIT0 — Millisecond channel (1 kHz tick, always running)
//     Drives all ms-class timers.  Always active.
//
//   PIT1 — Microsecond channel (100 kHz = 10 µs tick, on-demand)
//     Drives all µs-class timers.  Enabled automatically when the
//     first µs client arms, disabled when the last µs client cancels
//     or completes.  Zero overhead when no µs timers are active.
//
//   The 10 µs tick gives ample precision for pre-PPS camp-on timing
//   while keeping PIT1 ISR overhead under 0.5% CPU when active.
//
// This header is SAFE for inclusion by any subsystem.
// No process or control-plane semantics are exposed here.
// ============================================================================


// -----------------------------------------------------------------------------
// Timer Classes (semantic roles)
// -----------------------------------------------------------------------------
//
// Each class has a fixed interval type (ms or µs) and a default period.
// The interval type is an intrinsic property of the class — it cannot
// be changed at runtime.
//

typedef enum {
  // === Millisecond classes (PIT0) ===
  TIMEPOP_CLASS_ASAP = 0,  // special semantics, no PIT involvement
  TIMEPOP_CLASS_RX_POLL,
  TIMEPOP_CLASS_EVENTBUS,
  TIMEPOP_CLASS_CPU_SAMPLE,
  TIMEPOP_CLASS_CLOCKS,
  TIMEPOP_CLASS_GUARD,
  TIMEPOP_CLASS_PPS_RELAY,
  TIMEPOP_CLASS_FLASH,
  TIMEPOP_CLASS_DEBUG_BEACON,
  TIMEPOP_CLASS_USER_1,
  TIMEPOP_CLASS_USER_2,
  TIMEPOP_CLASS_TX_PUMP,
  TIMEPOP_CLASS_OCXO_DITHER,
  TIMEPOP_CLASS_PRE_PPS_COARSE,
  TIMEPOP_CLASS_COUNT,
} timepop_class_t;

// -----------------------------------------------------------------------------
// Interval Type (intrinsic per class, not caller-selectable)
// -----------------------------------------------------------------------------

typedef enum {
  TIMEPOP_INTERVAL_MS,   // Period expressed in milliseconds, driven by PIT0
  TIMEPOP_INTERVAL_US,   // Period expressed in microseconds, driven by PIT1
} timepop_interval_t;

// -----------------------------------------------------------------------------
// Timer Control Interface (passed to callbacks)
// -----------------------------------------------------------------------------

typedef struct timepop_ctx_t {
  timepop_class_t klass;

  // Cancel this timer (one-shot or recurring)
  void (*cancel)(struct timepop_ctx_t* self);
} timepop_ctx_t;


// -----------------------------------------------------------------------------
// Callback Signature
// -----------------------------------------------------------------------------

typedef void (*timepop_callback_t)(
  timepop_ctx_t* timer,
  void*          user_ctx
);


// -----------------------------------------------------------------------------
// Arm / Cancel
// -----------------------------------------------------------------------------

// Arm a timer by semantic class, using the default period for that class.
//
// If recurring=true, the timer automatically re-arms after each fire
// until explicitly canceled.
//
// The interval type (ms or µs) is determined by the class — callers
// do not specify it.
//
// Returns true on success.
bool timepop_arm(
  timepop_class_t     klass,
  bool               recurring,
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
);

// Arm a timer by semantic class with a custom period override.
//
// The period is interpreted in the class's native interval type
// (ms for ms-classes, µs for µs-classes).  This allows dynamic
// period computation without changing the class definition.
//
// Returns true on success.
bool timepop_arm_with_period(
  timepop_class_t     klass,
  bool               recurring,
  uint32_t           period,     // in the class's native units (ms or µs)
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
);

// Cancel a timer by class.
// Safe to call multiple times.
bool timepop_cancel(timepop_class_t klass);


// -----------------------------------------------------------------------------
// Dispatch
// -----------------------------------------------------------------------------

// Dispatch expired timers.
// Must be called from scheduled (non-ISR) runtime context.
void timepop_dispatch(void);


// -----------------------------------------------------------------------------
// Introspection (lightweight, consumer-safe)
// -----------------------------------------------------------------------------

// Return number of active timers.
// Intended for debugging and visibility only.
uint32_t timepop_active_count(void);