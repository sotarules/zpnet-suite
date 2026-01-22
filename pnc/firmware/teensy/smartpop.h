// =============================================================
// FILE: smartpop.h
// =============================================================
//
// SmartPOP — Prescaled Edge–Aligned Scheduling
//
// Provides deterministic, edge-aligned scheduling on top of
// 10 kHz prescaled GNSS / OCXO hardware clocks.
//
// Author: The Mule + GPT
//

#pragma once

#include <stdint.h>

// --------------------------------------------------------------
// Lifecycle
// --------------------------------------------------------------

void smartpop_init(void);
void smartpop_dispatch(void);

// --------------------------------------------------------------
// ISR entry points (called from clock.cpp)
// --------------------------------------------------------------

void smartpop_gnss_tick(void);
void smartpop_ocxo_tick(void);

// --------------------------------------------------------------
// Scheduling API
// --------------------------------------------------------------

typedef void (*smartpop_callback_t)(void* context);

// Fire callback on the next prescaled GNSS tick.
void smartpop_start(
  smartpop_callback_t callback,
  void* context
);

// Fire callback after N prescaled GNSS ticks.
void smartpop_end(
  uint32_t ticks,
  smartpop_callback_t callback,
  void* context
);
