/**
 * -----------------------------------------------------------------------------
 *  TimePop Timer Module Implementation (timepop.cpp)
 * -----------------------------------------------------------------------------
 *
 *  Software-emulated timer fabric built on a single PIT channel.
 *
 *  - One PIT provides a global heartbeat (fixed tick)
 *  - Many logical timers are multiplexed in software
 *  - PIT ISR records expiration only (no callbacks in ISR)
 *  - Callbacks are dispatched from scheduled context via timepop_dispatch()
 *
 * -----------------------------------------------------------------------------
 */

#include "timepop.h"
#include <Arduino.h>
#include "imxrt.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// Base tick for the PIT heartbeat.
// Change this to 100 for 100us resolution if needed.
#define TIMEPOP_TICK_US        1000u   // 1 ms

// Maximum number of concurrent timers.
// "Practically unlimited" without dynamic allocation.
#define TIMEPOP_MAX_TIMERS    64

// Use PIT channel 0
#define TIMEPOP_PIT_CHANNEL   0

// -----------------------------------------------------------------------------
// Internal Types
// -----------------------------------------------------------------------------

typedef struct {
  bool                 active;
  bool                 expired;
  uint32_t             remaining_ticks;
  timepop_callback_t   callback;
  void*                context;
  const char*          name;
  uint32_t             generation;
} timepop_slot_t;

// -----------------------------------------------------------------------------
// Internal State
// -----------------------------------------------------------------------------

static volatile bool timepop_pending = false;
static timepop_slot_t slots[TIMEPOP_MAX_TIMERS];
static uint32_t next_generation = 1;

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------

static inline uint32_t units_to_ticks(uint32_t delay, timepop_units_t units) {
  if (delay == 0) {
    return 0;
  }

  uint64_t us =
    (units == TIMEPOP_UNITS_MILLISECONDS)
      ? (uint64_t)delay * 1000ULL
      : (uint64_t)delay;

  // Round up to the next tick
  return (uint32_t)((us + TIMEPOP_TICK_US - 1) / TIMEPOP_TICK_US);
}

static inline timepop_handle_t make_handle(uint16_t index, uint16_t gen) {
  return ((timepop_handle_t)gen << 16) | index;
}

static inline bool decode_handle(timepop_handle_t handle,
                                 uint16_t* index,
                                 uint16_t* gen) {
  if (handle == 0) return false;
  *index = (uint16_t)(handle & 0xFFFF);
  *gen   = (uint16_t)(handle >> 16);
  return (*index < TIMEPOP_MAX_TIMERS);
}

// -----------------------------------------------------------------------------
// PIT ISR
// -----------------------------------------------------------------------------

void pit0_isr(void) {
  // Clear interrupt flag
  PIT_TFLG0 = PIT_TFLG_TIF;

  bool any_expired = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_TIMERS; i++) {
    if (!slots[i].active) continue;

    if (slots[i].remaining_ticks > 0) {
      slots[i].remaining_ticks--;
    }

    if (slots[i].remaining_ticks == 0 && !slots[i].expired) {
      slots[i].expired = true;
      any_expired = true;
    }
  }

  if (any_expired) {
    timepop_pending = true;
  }
}

// -----------------------------------------------------------------------------
// Module Lifecycle
// -----------------------------------------------------------------------------

void timepop_init() {
  // Clear slots
  for (uint32_t i = 0; i < TIMEPOP_MAX_TIMERS; i++) {
    slots[i].active  = false;
    slots[i].expired = false;
    slots[i].generation = 0;
  }

  timepop_pending = false;

  // Enable PIT clock
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

  // Enable PIT module
  PIT_MCR = 0;

  // Configure PIT channel
  uint32_t pit_cycles =
    (uint32_t)((F_BUS_ACTUAL / 1000000) * TIMEPOP_TICK_US) - 1;

  PIT_LDVAL0 = pit_cycles;
  PIT_TCTRL0 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;

  // Attach ISR
  attachInterruptVector(IRQ_PIT, pit0_isr);
  NVIC_ENABLE_IRQ(IRQ_PIT);
}

// -----------------------------------------------------------------------------
// Scheduling
// -----------------------------------------------------------------------------

timepop_handle_t timepop_schedule(
  uint32_t delay,
  timepop_units_t units,
  timepop_callback_t callback,
  void* context,
  const char* name
) {
  if (!callback) return 0;

  uint32_t ticks = units_to_ticks(delay, units);

  noInterrupts();
  for (uint16_t i = 0; i < TIMEPOP_MAX_TIMERS; i++) {
    if (!slots[i].active) {
      slots[i].active           = true;
      slots[i].expired          = (ticks == 0);
      slots[i].remaining_ticks  = ticks;
      slots[i].callback         = callback;
      slots[i].context          = context;
      slots[i].name             = name;
      slots[i].generation       = (uint16_t)(next_generation++);

      if (slots[i].expired) {
        timepop_pending = true;
      }

      interrupts();
      return make_handle(i, slots[i].generation);
    }
  }
  interrupts();

  return 0; // no free slot
}

// -----------------------------------------------------------------------------
// Cancellation
// -----------------------------------------------------------------------------

bool timepop_cancel(timepop_handle_t handle) {
  uint16_t index, gen;
  if (!decode_handle(handle, &index, &gen)) {
    return false;
  }

  noInterrupts();
  if (slots[index].active && slots[index].generation == gen) {
    slots[index].active  = false;
    slots[index].expired = false;
    interrupts();
    return true;
  }
  interrupts();

  return false;
}

// -----------------------------------------------------------------------------
// Dispatch (Scheduled Context)
// -----------------------------------------------------------------------------

void timepop_dispatch() {
  if (!timepop_pending) return;

  timepop_pending = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_TIMERS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;

    // Capture callback and context
    timepop_callback_t cb = slots[i].callback;
    void* ctx             = slots[i].context;

    // Invalidate slot before callback (one-shot semantics)
    slots[i].active  = false;
    slots[i].expired = false;

    // Invoke callback in scheduled context
    cb(ctx);
  }
}

// -----------------------------------------------------------------------------
// Introspection
// -----------------------------------------------------------------------------

uint32_t timepop_active_count() {
  uint32_t count = 0;
  noInterrupts();
  for (uint32_t i = 0; i < TIMEPOP_MAX_TIMERS; i++) {
    if (slots[i].active) count++;
  }
  interrupts();
  return count;
}
