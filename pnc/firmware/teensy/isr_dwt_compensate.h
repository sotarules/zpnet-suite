#pragma once

#include <stdint.h>

// ============================================================================
// isr_dwt_compensate.h — ISR Entry Latency Compensation
// ============================================================================
//
// Reciprocal of tdc_correction.h.
//
// tdc_correct adjusts a shadow DWT forward to the true PPS edge:
//   corrected = shadow + (delta - TDC_FIXED_OVERHEAD)
//
// This header does the opposite: given a DWT_CYCCNT read as the
// FIRST instruction inside an ISR, subtract the ISR entry latency
// to recover the DWT value at the moment the hardware event fired.
//
// The ISR entry latency on Cortex-M7 at 1008 MHz consists of:
//
//   - NVIC priority check + tail-chain decision:    ~4 cycles
//   - Register stacking (8 regs, 64-bit bus):       ~6 cycles
//   - Vector table fetch + pipeline flush:           ~6 cycles
//   - Branch to ISR entry point:                     ~2 cycles
//   - Prologue instructions before DWT read:        variable
//
// Total: ~18-20 cycles for the hardware portion, plus whatever
// instructions precede the ARM_DWT_CYCCNT read in the ISR body.
//
// For the GPT2 compare ISR (gpt2_compare_isr), the DWT read is
// the very first operation.  The compiler may emit a function
// prologue (push, frame pointer) before the first user instruction.
// At 1008 MHz with -O2, the measured latency is approximately
// 49 cycles (48.6 ns) from the GPT2 compare match to the DWT read.
//
// This constant should be validated empirically using the DWT
// prediction Welford accumulator: the mean DWT prediction residual
// (actual - predicted) at steady state reveals the true ISR latency.
// Adjust GPT2_ISR_ENTRY_DWT_CYCLES until the mean residual
// converges to zero.
//
// ============================================================================

// ISR entry latency for the GPT2 output compare ISR.
// This is the number of DWT cycles between the GPT2 compare match
// (the physical event) and the first ARM_DWT_CYCCNT read inside
// gpt2_compare_isr().
//
// Initial estimate: 49 cycles (~48.6 ns at 1008 MHz).
// Calibrate using TIMEPOP DIAG: adjust until dwt_pred_mean ≈ 0.
static constexpr uint32_t GPT2_ISR_ENTRY_DWT_CYCLES = 49;

// ============================================================================
// dwt_at_event — recover the DWT_CYCCNT at the actual hardware event
//
// Given a DWT_CYCCNT captured as the first instruction inside an ISR,
// subtracts the ISR entry latency to estimate the DWT value at the
// moment the hardware event (e.g., GPT2 compare match) actually fired.
//
// This is the inverse of ISR latency: the event happened BEFORE the
// ISR read the counter, so the true DWT is earlier (smaller).
//
// Parameters:
//   isr_dwt_cyccnt   — DWT_CYCCNT captured at ISR entry
//   isr_entry_cycles — ISR entry latency in DWT cycles
//
// Returns:
//   Estimated DWT_CYCCNT at the hardware event moment.
// ============================================================================

static inline uint32_t dwt_at_event(
  uint32_t isr_dwt_cyccnt,
  uint32_t isr_entry_cycles
) {
  return isr_dwt_cyccnt - isr_entry_cycles;
}

// ============================================================================
// Convenience: GPT2 compare ISR specific
// ============================================================================

static inline uint32_t dwt_at_gpt2_compare(uint32_t isr_dwt_cyccnt) {
  return dwt_at_event(isr_dwt_cyccnt, GPT2_ISR_ENTRY_DWT_CYCLES);
}