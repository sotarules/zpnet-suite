#pragma once

#include <stdint.h>

// ============================================================================
// Fixed ISR-overhead edge timing helpers
// ============================================================================
//
// Current model:
//
//   The shadow-write loops remain in the system to place the Cortex-M7
//   into a consistent, well-characterized execution state before a key
//   interrupt arrives.  Under that conditioned state, the authoritative
//   estimate of the physical edge is a fixed subtraction from the first
//   DWT_CYCCNT captured in the ISR.
//
//   PPS edge:   edge_dwt = isr_dwt - 48
//   GPT edge:   edge_dwt = isr_dwt - 17
//
// The loop-position delta between shadow_dwt and isr_dwt is retained only
// as optional forensics.  It is not part of the production timing API.
// All production code should consume the canonical helpers below and never
// re-derive edge time locally.
//
// ============================================================================

static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD = 48;
static constexpr uint32_t GPT_ISR_FIXED_OVERHEAD = 17;

// Backward-compatible names used elsewhere in the firmware.
static constexpr uint32_t TDC_FIXED_OVERHEAD     = PPS_ISR_FIXED_OVERHEAD;
static constexpr uint32_t GPT_TDC_FIXED_OVERHEAD = GPT_ISR_FIXED_OVERHEAD;

static inline uint32_t pps_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - PPS_ISR_FIXED_OVERHEAD;
}

static inline uint32_t gpt_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - GPT_ISR_FIXED_OVERHEAD;
}
