// =============================================================
// FILE: smartpop.h
// =============================================================
//
// SmartPOP — Prescaled Edge–Aligned Scheduling
//
// SmartPOP provides a causally aligned scheduling facility built
// on prescaled hardware clock edges (GNSS / OCXO).
//
// This header defines the public contract only.
// Implementation lives in smartpop.cpp.
//
// NOTE:
//   At this stage, SmartPOP is intentionally unimplemented.
//   These declarations exist to anchor clock.cpp delegation
//   and allow incremental bring-up.
//
// Author: The Mule + GPT
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------
// Prescaled edge entry points (called from clock.cpp ISRs)
//
// These functions are invoked exactly once per prescaled
// hardware tick (e.g., 10 kHz).
//
// They MUST be ISR-safe and minimal.
// --------------------------------------------------------------

void smartpop_gnss_tick(void);
void smartpop_ocxo_tick(void);

// --------------------------------------------------------------
// Lifecycle (optional; no-op for now)
// --------------------------------------------------------------

void smartpop_init(void);

// --------------------------------------------------------------
// Scheduling API (to be implemented later)
// --------------------------------------------------------------

typedef void (*smartpop_callback_t)(void* context);

// Schedule a smart-aligned start on the next valid prescaled tick.
void smartpop_start(
  smartpop_callback_t callback,
  void* context
);

// Schedule a smart-aligned end after N prescaled ticks.
void smartpop_end(
  uint32_t ticks,
  smartpop_callback_t callback,
  void* context
);

// Cancel a previously scheduled smart operation.
void smartpop_cancel(void* token);

#ifdef __cplusplus
}
#endif
