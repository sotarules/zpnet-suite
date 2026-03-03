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

#define TIMEPOP_MAX_SLOTS    16      // hard upper bound (tune later)

// PIT0: 1 ms tick = 24 MHz / 24000
static constexpr uint32_t PIT0_LOAD = 23999u;

// PIT1: 10 µs tick = 24 MHz / 240
// 10 µs gives ample precision for pre-PPS camp-on.
// ISR overhead at 100 kHz: ~0.3-0.5% CPU when active.
static constexpr uint32_t PIT1_LOAD     = 239u;
static constexpr uint32_t PIT1_TICK_US  = 10u;

// ================================================================
// Dual-PIT Architecture (Symmetric)
// ================================================================
//
// PIT0 and PIT1 implement IDENTICAL countdown logic.  Each channel:
//
//   1. Fires its ISR at a fixed tick rate
//   2. Scans all active slots belonging to its interval type
//   3. Decrements remaining_ticks
//   4. Marks slots as expired when remaining_ticks reaches 0
//   5. Sets timepop_pending to wake the dispatch loop
//
// The ONLY differences between the two channels:
//
//   PIT0                          PIT1
//   ----                          ----
//   Tick rate:  1 ms              Tick rate:  10 µs
//   Interval:   TIMEPOP_INTERVAL_MS    TIMEPOP_INTERVAL_US
//   Lifecycle:  Always running    Lifecycle:  Active only when
//                                             µs clients > 0
//
// Period units follow the interval type:
//   ms class with period 500  → fires every 500 ms (500 PIT0 ticks)
//   µs class with period 999000 → fires every 999 ms (99900 PIT1 ticks)
//

// ================================================================
// Class Definition Table
// ================================================================

struct timepop_class_def_t {
  timepop_interval_t interval;   // ms or µs
  uint32_t           period;     // default period in native units
};

static timepop_class_def_t CLASS_DEF[TIMEPOP_CLASS_COUNT];

static void init_class_definitions(void) {

  // --- Millisecond classes (PIT0) ---
  CLASS_DEF[TIMEPOP_CLASS_ASAP]         = { TIMEPOP_INTERVAL_MS,      0 };
  CLASS_DEF[TIMEPOP_CLASS_RX_POLL]      = { TIMEPOP_INTERVAL_MS,      1 };
  CLASS_DEF[TIMEPOP_CLASS_EVENTBUS]     = { TIMEPOP_INTERVAL_MS,   1000 };
  CLASS_DEF[TIMEPOP_CLASS_CPU_SAMPLE]   = { TIMEPOP_INTERVAL_MS,   1000 };
  CLASS_DEF[TIMEPOP_CLASS_CLOCKS]       = { TIMEPOP_INTERVAL_MS,    200 };
  CLASS_DEF[TIMEPOP_CLASS_GUARD]        = { TIMEPOP_INTERVAL_MS,    500 };
  CLASS_DEF[TIMEPOP_CLASS_PPS_RELAY]    = { TIMEPOP_INTERVAL_MS,    500 };
  CLASS_DEF[TIMEPOP_CLASS_FLASH]        = { TIMEPOP_INTERVAL_MS,   5000 };
  CLASS_DEF[TIMEPOP_CLASS_DEBUG_BEACON] = { TIMEPOP_INTERVAL_MS,   1000 };
  CLASS_DEF[TIMEPOP_CLASS_USER_1]       = { TIMEPOP_INTERVAL_MS,     10 };
  CLASS_DEF[TIMEPOP_CLASS_USER_2]       = { TIMEPOP_INTERVAL_MS,    100 };
  CLASS_DEF[TIMEPOP_CLASS_TX_PUMP]      = { TIMEPOP_INTERVAL_MS,      1 };
  CLASS_DEF[TIMEPOP_CLASS_PRE_PPS_COARSE] = { TIMEPOP_INTERVAL_MS,  999 };

  // --- Microsecond classes (PIT1) ---
  CLASS_DEF[TIMEPOP_CLASS_PRE_PPS]      = { TIMEPOP_INTERVAL_US, 998 };
}

// ================================================================
// Internal Slot (identical for both interval types)
// ================================================================

typedef struct {
  bool                active;
  bool                expired;
  bool                recurring;

  timepop_class_t     klass;
  uint32_t            remaining_ticks;   // PIT ticks until expiration
  uint32_t            period_ticks;      // reload value for recurring

  timepop_callback_t  callback;
  void*               user_ctx;
  const char*         name;

  uint32_t            id;                // monotonic instance id
} timepop_slot_t;

// ================================================================
// State
// ================================================================

static timepop_slot_t slots[TIMEPOP_MAX_SLOTS];
static volatile bool timepop_pending = false;

static uint32_t next_slot_id = 1;

// ================================================================
// PIT0 Diagnostics
// ================================================================

static volatile uint32_t pit0_tick_count = 0;
static volatile uint32_t pit0_zero_hits  = 0;

// ================================================================
// PIT1 State and Diagnostics
// ================================================================

// Active µs-class client count.
// PIT1 is enabled when this transitions 0 → 1,
// disabled when it transitions 1 → 0.
static volatile uint8_t pit1_client_count = 0;

static volatile uint32_t pit1_tick_count = 0;
static volatile uint32_t pit1_zero_hits  = 0;

// ================================================================
// PIT1 Enable / Disable
// ================================================================
//
// Called with interrupts disabled.
// These manage PIT1's physical state based on client count.
//

static void pit1_enable(void) {
  PIT_TFLG1  = PIT_TFLG_TIF;   // clear any stale flag
  PIT_LDVAL1 = PIT1_LOAD;       // 10 µs tick
  PIT_TCTRL1 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;
}

static void pit1_disable(void) {
  PIT_TCTRL1 = 0;
  PIT_TFLG1  = PIT_TFLG_TIF;   // clear flag
}

// Track a µs client being added.  Enable PIT1 on first client.
// Must be called with interrupts disabled.
static void pit1_client_add(void) {
  pit1_client_count++;
  if (pit1_client_count == 1) {
    pit1_enable();
  }
}

// Track a µs client being removed.  Disable PIT1 on last client.
// Must be called with interrupts disabled.
static void pit1_client_remove(void) {
  if (pit1_client_count > 0) {
    pit1_client_count--;
  }
  if (pit1_client_count == 0) {
    pit1_disable();
  }
}

// ================================================================
// Period Conversion
// ================================================================
//
// Convert a period in native units (ms or µs) to PIT ticks.
//
//   ms class: 1 period unit = 1 PIT0 tick (1:1)
//   µs class: 1 period unit = 1/PIT1_TICK_US PIT1 ticks
//             e.g. 999000 µs / 10 µs = 99900 ticks
//

static inline uint32_t period_to_ticks(timepop_class_t klass, uint32_t period) {
  if (CLASS_DEF[klass].interval == TIMEPOP_INTERVAL_US) {
    return period / PIT1_TICK_US;
  }
  return period;  // ms classes: 1:1
}

// ================================================================
// PIT0 ISR — Millisecond channel
// ================================================================
//
// Scans all active ms-class slots.  Identical logic to PIT1.
//

static void pit0_isr(void) {

  PIT_TFLG0 = PIT_TFLG_TIF;
  pit0_tick_count++;

  bool any_expired = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active) continue;
    if (CLASS_DEF[slots[i].klass].interval != TIMEPOP_INTERVAL_MS) continue;

    if (slots[i].remaining_ticks > 0) {
      slots[i].remaining_ticks--;

      if (slots[i].remaining_ticks == 0 && !slots[i].expired) {
        slots[i].expired = true;
        pit0_zero_hits++;
        any_expired = true;
      }
    }
  }

  if (any_expired) {
    timepop_pending = true;
  }
}

// ================================================================
// PIT1 ISR — Microsecond channel
// ================================================================
//
// Scans all active µs-class slots.  Identical logic to PIT0.
//

static void pit1_isr(void) {

  PIT_TFLG1 = PIT_TFLG_TIF;
  pit1_tick_count++;

  bool any_expired = false;

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active) continue;
    if (CLASS_DEF[slots[i].klass].interval != TIMEPOP_INTERVAL_US) continue;

    if (slots[i].remaining_ticks > 0) {
      slots[i].remaining_ticks--;

      if (slots[i].remaining_ticks == 0 && !slots[i].expired) {
        slots[i].expired = true;
        pit1_zero_hits++;
        any_expired = true;
      }
    }
  }

  if (any_expired) {
    timepop_pending = true;
  }
}

// ================================================================
// Combined PIT ISR dispatcher
// ================================================================
//
// All PIT channels on the i.MX RT1062 share IRQ_PIT.
// Check both flag registers and dispatch accordingly.
//

static void pit_combined_isr(void) {

  if (PIT_TFLG0 & PIT_TFLG_TIF) {
    pit0_isr();
  }

  if (PIT_TFLG1 & PIT_TFLG_TIF) {
    pit1_isr();
  }
}

// ================================================================
// Initialization
// ================================================================

void timepop_init(void) {

  init_class_definitions();

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {
    slots[i] = {};
  }

  // --------------------------------------------------------
  // PIT module clock gate (shared by all channels)
  // --------------------------------------------------------
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 0;

  // --------------------------------------------------------
  // PIT0 — Millisecond channel (always running)
  // --------------------------------------------------------
  PIT_LDVAL0 = PIT0_LOAD;   // 24 MHz / 24000 = 1 kHz
  PIT_TCTRL0 = PIT_TCTRL_TEN | PIT_TCTRL_TIE;

  // --------------------------------------------------------
  // PIT1 — Microsecond channel (initially disabled)
  // Enabled on-demand by pit1_client_add().
  // --------------------------------------------------------
  PIT_TCTRL1 = 0;
  PIT_TFLG1  = PIT_TFLG_TIF;
  pit1_client_count = 0;

  // --------------------------------------------------------
  // Install combined ISR
  // --------------------------------------------------------
  attachInterruptVector(IRQ_PIT, pit_combined_isr);
  NVIC_ENABLE_IRQ(IRQ_PIT);
}

// ================================================================
// Arm (internal, shared by both public entry points)
// ================================================================

static bool timepop_arm_internal(
  timepop_class_t     klass,
  bool               recurring,
  uint32_t           period_native,   // in ms or µs depending on class
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
) {
  if (klass >= TIMEPOP_CLASS_COUNT || !callback) return false;

  const timepop_class_def_t& def = CLASS_DEF[klass];
  bool is_us = (def.interval == TIMEPOP_INTERVAL_US);

  noInterrupts();

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (slots[i].active) continue;

    uint32_t ticks = period_to_ticks(klass, period_native);

    slots[i].active          = true;
    slots[i].recurring       = recurring;
    slots[i].klass           = klass;
    slots[i].callback        = callback;
    slots[i].user_ctx        = user_ctx;
    slots[i].name            = name;
    slots[i].id              = next_slot_id++;
    slots[i].period_ticks    = ticks;

    if (klass == TIMEPOP_CLASS_ASAP) {
      // ----------------------------------------------------------
      // ASAP: expire immediately, no PIT involvement
      // ----------------------------------------------------------
      slots[i].remaining_ticks = 0;
      slots[i].expired         = true;
      timepop_pending          = true;
    } else {
      slots[i].remaining_ticks = ticks;
      slots[i].expired         = false;

      // Enable PIT1 if this is its first client
      if (is_us) {
        pit1_client_add();
      }
    }

    interrupts();
    return true;
  }

  interrupts();
  return false;
}

// ================================================================
// Arm (public API — default period)
// ================================================================

bool timepop_arm(
  timepop_class_t     klass,
  bool               recurring,
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
) {
  return timepop_arm_internal(
    klass, recurring,
    CLASS_DEF[klass].period,
    callback, user_ctx, name
  );
}

// ================================================================
// Arm with custom period (public API)
// ================================================================

bool timepop_arm_with_period(
  timepop_class_t     klass,
  bool               recurring,
  uint32_t           period,
  timepop_callback_t callback,
  void*              user_ctx,
  const char*        name
) {
  return timepop_arm_internal(
    klass, recurring,
    period,
    callback, user_ctx, name
  );
}

// ================================================================
// Cancel
// ================================================================

bool timepop_cancel(timepop_class_t klass) {

  bool is_us = (CLASS_DEF[klass].interval == TIMEPOP_INTERVAL_US);

  noInterrupts();

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].klass == klass) {

      slots[i].active  = false;
      slots[i].expired = false;

      // Disable PIT1 if this was its last client
      if (is_us) {
        pit1_client_remove();
      }
    }
  }

  interrupts();
  return true;
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
    ctx.cancel = (void (*)(timepop_ctx_t*))((uintptr_t)i);

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, slots[i].user_ctx);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);

    if (slots[i].active && slots[i].recurring) {

      uint32_t reload = slots[i].period_ticks;

      if (reload == 0) {
        // Recurring ASAP is forbidden by construction
        bool is_us = (CLASS_DEF[slots[i].klass].interval == TIMEPOP_INTERVAL_US);
        slots[i].active = false;
        if (is_us) {
          noInterrupts();
          pit1_client_remove();
          interrupts();
        }
      } else {
        slots[i].remaining_ticks = reload;
      }

    } else {
      // One-shot complete, or canceled during callback.
      //
      // Guard: if the callback already canceled itself via
      // timepop_cancel(), active is already false and the
      // client count was already decremented.  Only remove
      // the client if WE are the ones deactivating the slot.
      bool was_active = slots[i].active;
      bool is_us = (CLASS_DEF[slots[i].klass].interval == TIMEPOP_INTERVAL_US);

      slots[i].active = false;

      // Release PIT1 client only if we just deactivated it
      if (was_active && is_us) {
        noInterrupts();
        pit1_client_remove();
        interrupts();
      }
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
  return pit0_tick_count;
}

uint32_t timepop_get_last_remaining(void) {
  // Legacy: return 0.  This diagnostic was PIT0-specific
  // and of limited value.  The REPORT command provides
  // per-slot remaining_ticks for full visibility.
  return 0;
}

uint32_t timepop_get_zero_hits(void) {
  return pit0_zero_hits;
}

// ================================================================
// Commands
// ================================================================

static const char* interval_str(timepop_interval_t iv) {
  return (iv == TIMEPOP_INTERVAL_US) ? "us" : "ms";
}

// ------------------------------------------------------------
// REPORT — active TimePop timer snapshot
// ------------------------------------------------------------
static Payload cmd_report(const Payload& /*args*/) {

  Payload out;

  // Channel health
  out.add("pit0_tick_count",    pit0_tick_count);
  out.add("pit0_zero_hits",     pit0_zero_hits);
  out.add("pit1_tick_count",    pit1_tick_count);
  out.add("pit1_zero_hits",     pit1_zero_hits);
  out.add("pit1_client_count",  (uint32_t)pit1_client_count);
  out.add("pit1_active",        pit1_client_count > 0);

  PayloadArray timers;

  for (uint32_t i = 0; i < TIMEPOP_MAX_SLOTS; i++) {

    if (!slots[i].active) continue;

    const timepop_class_def_t& def = CLASS_DEF[slots[i].klass];

    Payload entry;
    entry.add("slot",            i);
    entry.add("id",              slots[i].id);
    entry.add("class",           (uint32_t)slots[i].klass);
    entry.add("name",            slots[i].name ? slots[i].name : "unnamed");
    entry.add("interval",        interval_str(def.interval));
    entry.add("period_ticks",    slots[i].period_ticks);
    entry.add("remaining_ticks", slots[i].remaining_ticks);
    entry.add("recurring",       slots[i].recurring);

    timers.add(entry);
  }

  out.add_array("timers", timers);
  return out;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr }
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}