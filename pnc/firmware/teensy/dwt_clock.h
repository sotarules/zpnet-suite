#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// DWT cycle counter (CPU-core local)
//
// Stepwise bring-up:
//   STEP 1: compile + link only (no enable)
//   STEP 2: enable CYCCNT and read-only access
//
// This module MUST:
//   • never touch CCM
//   • never touch peripheral clocks
//   • never affect USB
// --------------------------------------------------------------

void dwt_clock_init();
uint32_t dwt_clock_read();
