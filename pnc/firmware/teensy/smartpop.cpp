// =============================================================
// FILE: smartpop.cpp
// =============================================================
//
// SmartPOP — Prescaled Edge–Aligned Scheduling
//
// Software scheduler built on prescaled hardware edge ticks
// (10 kHz GNSS / OCXO domains).
//
// Design:
//   • Hardware ISR increments tick counters only
//   • ISR marks expirations, never runs callbacks
//   • Callbacks are dispatched from scheduled context
//
// Author: The Mule + GPT
//

#include "smartpop.h"
#include "cpu_usage.h"
#include "clock.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Configuration
// --------------------------------------------------------------

#define SMARTPOP_MAX_EVENTS  32

// --------------------------------------------------------------
// Internal Types
// --------------------------------------------------------------

typedef struct {
  bool                   active;
  bool                   expired;
  uint64_t               target_tick;
  smartpop_callback_t    callback;
  void*                  context;
} smartpop_slot_t;

// --------------------------------------------------------------
// Internal State
// --------------------------------------------------------------

static smartpop_slot_t gnss_slots[SMARTPOP_MAX_EVENTS];
static smartpop_slot_t ocxo_slots[SMARTPOP_MAX_EVENTS];

static volatile bool gnss_pending = false;
static volatile bool ocxo_pending = false;

// --------------------------------------------------------------
// Lifecycle
// --------------------------------------------------------------

void smartpop_init(void) {
  for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
    gnss_slots[i].active = false;
    gnss_slots[i].expired = false;
    ocxo_slots[i].active = false;
    ocxo_slots[i].expired = false;
  }

  gnss_pending = false;
  ocxo_pending = false;
}

// --------------------------------------------------------------
// Utility
// --------------------------------------------------------------

static smartpop_slot_t* allocate_slot(smartpop_slot_t* slots) {
  for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
    if (!slots[i].active) {
      slots[i].active = true;
      slots[i].expired = false;
      return &slots[i];
    }
  }
  return nullptr;
}

// --------------------------------------------------------------
// ISR entry points (called from clock.cpp)
// --------------------------------------------------------------

void smartpop_gnss_tick(void) {
  uint64_t now = clock_gnss_10khz_ticks();

  bool any_expired = false;

  for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
    if (!gnss_slots[i].active || gnss_slots[i].expired) continue;

    if (now >= gnss_slots[i].target_tick) {
      gnss_slots[i].expired = true;
      any_expired = true;
    }
  }

  if (any_expired) {
    gnss_pending = true;
  }
}

void smartpop_ocxo_tick(void) {
  uint64_t now = clock_ocxo_10khz_ticks();

  bool any_expired = false;

  for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
    if (!ocxo_slots[i].active || ocxo_slots[i].expired) continue;

    if (now >= ocxo_slots[i].target_tick) {
      ocxo_slots[i].expired = true;
      any_expired = true;
    }
  }

  if (any_expired) {
    ocxo_pending = true;
  }
}

// --------------------------------------------------------------
// Scheduling API
// --------------------------------------------------------------

void smartpop_start(
  smartpop_callback_t callback,
  void* context
) {
  if (!callback) return;

  noInterrupts();

  smartpop_slot_t* slot = allocate_slot(gnss_slots);
  if (!slot) {
    interrupts();
    return;
  }

  uint64_t now = clock_gnss_10khz_ticks();

  slot->target_tick = now + 1;  // next prescaled tick
  slot->callback    = callback;
  slot->context     = context;

  interrupts();
}

void smartpop_end(
  uint32_t ticks,
  smartpop_callback_t callback,
  void* context
) {
  if (!callback) return;

  noInterrupts();

  smartpop_slot_t* slot = allocate_slot(gnss_slots);
  if (!slot) {
    interrupts();
    return;
  }

  uint64_t now = clock_gnss_10khz_ticks();

  slot->target_tick = now + ticks;
  slot->callback    = callback;
  slot->context     = context;

  interrupts();
}

// --------------------------------------------------------------
// Dispatch (scheduled context)
// --------------------------------------------------------------

void smartpop_dispatch(void) {

  // ---------------- GNSS ----------------
  if (gnss_pending) {
    gnss_pending = false;

    for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
      if (!gnss_slots[i].active || !gnss_slots[i].expired) continue;

      smartpop_callback_t cb = gnss_slots[i].callback;
      void* ctx              = gnss_slots[i].context;

      gnss_slots[i].active  = false;
      gnss_slots[i].expired = false;

      uint32_t start_cycles = ARM_DWT_CYCCNT;
      cb(ctx);
      uint32_t end_cycles   = ARM_DWT_CYCCNT;

      cpu_usage_account_busy(end_cycles - start_cycles);
    }
  }

  // ---------------- OCXO ----------------
  if (ocxo_pending) {
    ocxo_pending = false;

    for (uint32_t i = 0; i < SMARTPOP_MAX_EVENTS; i++) {
      if (!ocxo_slots[i].active || !ocxo_slots[i].expired) continue;

      smartpop_callback_t cb = ocxo_slots[i].callback;
      void* ctx              = ocxo_slots[i].context;

      ocxo_slots[i].active  = false;
      ocxo_slots[i].expired = false;

      uint32_t start_cycles = ARM_DWT_CYCCNT;
      cb(ctx);
      uint32_t end_cycles   = ARM_DWT_CYCCNT;

      cpu_usage_account_busy(end_cycles - start_cycles);
    }
  }
}
