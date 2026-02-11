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
// This header is SAFE for inclusion by any subsystem.
// No process or control-plane semantics are exposed here.
// ============================================================================


// -----------------------------------------------------------------------------
// Timer Classes (semantic roles)
// -----------------------------------------------------------------------------

typedef enum {
  TIMEPOP_CLASS_ASAP = 0,  // special semantics, no PIT involvement
  TIMEPOP_CLASS_RX_POLL,
  TIMEPOP_CLASS_EVENTBUS,
  TIMEPOP_CLASS_CPU_SAMPLE,
  TIMEPOP_CLASS_CLOCKS,
  TIMEPOP_CLASS_GUARD,
  TIMEPOP_CLASS_FLASH,
  TIMEPOP_CLASS_DEBUG_BEACON,
  TIMEPOP_CLASS_USER_1,
  TIMEPOP_CLASS_USER_2,
  TIMEPOP_CLASS_TX_PUMP,
  TIMEPOP_CLASS_COUNT,
} timepop_class_t;

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

// Arm a timer by semantic class.
//
// If recurring=true, the timer automatically re-arms after each fire
// until explicitly canceled.
//
// Returns true on success.
bool timepop_arm(
  timepop_class_t     klass,
  bool               recurring,
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
