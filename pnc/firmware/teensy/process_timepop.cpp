#include "debug.h"
#include "timepop.h"

#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"

#include <Arduino.h>
#include "imxrt.h"

// ================================================================
// Configuration
// ================================================================

#define TIMEPOP_TICK_US      1000u   // 1 ms PIT
#define TIMEPOP_PIT_CHANNEL 0

#define TIMEPOP_MAX_SLOTS   16       // hard upper bound (tune later)

// Fixed cadence per class (in ticks)
//
// NOTE:
//   remaining_ticks == 0 means:
//     • expire immediately
//     • dispatch on next timepop_dispatch()
//     • no PIT involvement
//
static uint32_t CLASS_PERIOD_TICKS[TIMEPOP_CLASS_COUNT];

static void init_timepop_class_periods(void) {
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_ASAP]         = 0;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_RX_POLL]      = 5;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_EVENTBUS]     = 1000;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_CPU_SAMPLE]   = 1000;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_CLOCKS]       = 200;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_GUARD]        = 500;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_FLASH]        = 20;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_DEBUG_BEACON] = 1000;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_USER_1]       = 10;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_USER_2]       = 100;
  CLASS_PERIOD_TICKS[TIMEPOP_CLASS_DEBUG_TX]     = 5;
}

// ================================================================
// Internal Slot
// ================================================================

typedef struct {
  bool                active;
  bool                expired;
  bool                recurring;

  timepop_class_t     klass;
  uint32_t            remaining_ticks;

  timepop_callback_t  callback;
  void*               user_ctx;
  const char*         name;

  uint32_t            id;      // monotonic instance id (debugging)
} timepop_slot_t;

// ================================================================
// State
// ================================================================

static timepop_slot_t slots[TIMEPOP_MAX_SLOTS];
static volatile bool timepop_pending = false;

static uint32_t next_slot_id = 1;

volatile uint32_t pit_tick_count     = 0;
volatile uint32_t pit_last_remaining = 0;
volatile uint32_t pit_zero_hits      = 0;

// ================================================================
// PIT ISR
// ================================================================

void pit0_isr(void) {

  PIT_TFLG0 = PIT_TFLG_TIF;
  pit_tick_count++;

  bool any_expired = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active) continue;

    // Skip ASAP slots (remaining_ticks == 0)
    if (slots[i].remaining_ticks > 0) {

      slots[i].remaining_ticks--;

      pit_last_remaining = slots[i].remaining_ticks;

      if (slots[i].remaining_ticks == 0 && !slots[i].expired) {
        slots[i].expired = true;
        pit_zero_hits++;
        any_expired = true;
      }
    }
  }

  if (any_expired) {
    timepop_pending = true;
  }
}

// ================================================================
// Explicit initialization (physical, required)
// ================================================================

void timepop_init(void) {

  // Initialize class cadence table (must precede any arming)
  init_timepop_class_periods();

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {
    slots[i] = {};
  }

  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 0;

  uint32_t pit_cycles =
    (uint32_t)((F_BUS_ACTUAL / 1000000) * TIMEPOP_TICK_US) - 1;

  PIT_LDVAL0 = pit_cycles;
  PIT_TCTRL0 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;

  attachInterruptVector(IRQ_PIT, pit0_isr);
  NVIC_ENABLE_IRQ(IRQ_PIT);
}

// ================================================================
// Arm / Cancel
// ================================================================

bool timepop_arm(
  timepop_class_t     klass,
  bool               recurring,
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
) {
  if (klass >= TIMEPOP_CLASS_COUNT || !callback) return false;

  noInterrupts();

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (slots[i].active) continue;

    slots[i].active    = true;
    slots[i].recurring = recurring;
    slots[i].klass     = klass;
    slots[i].callback  = callback;
    slots[i].user_ctx  = user_ctx;
    slots[i].name      = name;
    slots[i].id        = next_slot_id++;

    uint32_t period = CLASS_PERIOD_TICKS[klass];

    if (period == 0) {
      // ----------------------------------------------------------
      // ASAP continuation semantics
      // ----------------------------------------------------------
      slots[i].remaining_ticks = 0;
      slots[i].expired         = true;

      // Force dispatch on next loop()
      timepop_pending = true;
    } else {
      slots[i].remaining_ticks = period;
      slots[i].expired         = false;
    }

    interrupts();
    return true;
  }

  interrupts();
  return false;
}

// ================================================================
// Dispatch (scheduled context)
// ================================================================

void timepop_dispatch(void) {

  if (!timepop_pending) return;
  timepop_pending = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active || !slots[i].expired) continue;

    slots[i].expired = false;

    timepop_ctx_t ctx;
    ctx.klass  = slots[i].klass;
    ctx.cancel = (void (*)(timepop_ctx_t*))((uintptr_t)i); // encode slot index

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, slots[i].user_ctx);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);

    if (slots[i].active && slots[i].recurring) {

      uint32_t period = CLASS_PERIOD_TICKS[slots[i].klass];

      if (period == 0) {
        // Recurring ASAP is forbidden by construction
        slots[i].active = false;
      } else {
        slots[i].remaining_ticks = period;
      }

    } else {
      slots[i].active = false;
    }
  }
}

// ================================================================
// Introspection
// ================================================================

uint32_t timepop_active_count(void) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {
    if (slots[i].active) n++;
  }
  return n;
}

uint32_t timepop_get_pit_tick_count(void) {
  return pit_tick_count;
}

uint32_t timepop_get_last_remaining(void) {
  return pit_last_remaining;
}

uint32_t timepop_get_zero_hits(void) {
  return pit_zero_hits;
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — active TimePop timer snapshot
// ------------------------------------------------------------
static Payload cmd_report(const Payload& /*args*/) {

  Payload out;

  String timers;
  timers += "[";

  bool first = true;
  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active) continue;

    if (!first) timers += ",";
    first = false;

    timers += "{";
    timers += "\"slot\":"; timers += i;
    timers += ",\"id\":"; timers += slots[i].id;
    timers += ",\"class\":"; timers += slots[i].klass;
    timers += ",\"name\":\"";
    timers += (slots[i].name ? slots[i].name : "unnamed");
    timers += "\"";
    timers += ",\"remaining_ticks\":"; timers += slots[i].remaining_ticks;
    timers += "}";

  }

  timers += "]";

  out.add("timers", timers.c_str());
  return out;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr }   // sentinel
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}
