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
// ── QTimer1 CH2 compare path (TimePop v8.0 priority queue) ──
//
// The DWT read is the first operation inside qtimer1_ch2_isr(),
// but the NVIC vector points to qtimer1_irq_isr() which checks the
// CH2 TCF1 flag and then calls qtimer1_ch2_isr().  This adds
// one flag read + conditional branch before the DWT capture compared
// to a direct-vector ISR like the PPS GPIO path.
//
// Initial estimate: 54 cycles (~53.6 ns at 1008 MHz).
//
//   Base NVIC + stacking:        ~20 cycles
//   qtimer1_irq_isr prologue:    ~3 cycles  (push/frame)
//   CH2 flag check + branch:     ~4 cycles  (load CSCTRL, test, branch)
//   ch2_isr prologue:            ~2 cycles  (inlined or minimal)
//   Compiler prologue variance:  ~2 cycles
//   ─────────────────────────────────────
//   Estimated total:              ~54 cycles
//
// This constant should be validated empirically using the DWT
// prediction Welford accumulator: the mean DWT prediction residual
// (actual - predicted) at steady state reveals the true ISR latency.
// Adjust QTIMER1_CH2_ISR_ENTRY_DWT_CYCLES until the mean residual
// converges to zero.
//
// NOTE: The v8.0 CH2 configuration (CM=1, no LENGTH, dynamic compare)
// differs from v7.0 (CM=2, LENGTH, periodic heartbeat).  The ISR
// dispatch path is the same (flag check + call), so the latency
// constant should be close.  Recalibrate via TIMEPOP DIAG after
// the migration.
//
// ============================================================================

// ISR entry latency for the QTimer1 CH2 compare ISR (TimePop production).
// This is the number of DWT cycles between the QTimer1 CH2 compare match
// (the physical event) and the first ARM_DWT_CYCCNT read inside
// qtimer1_ch2_isr().
//
// Initial estimate: 54 cycles (~53.6 ns at 1008 MHz).
// Calibrate using TIMEPOP DIAG: adjust until dwt_pred_mean ≈ 0.
static constexpr uint32_t QTIMER1_CH2_ISR_ENTRY_DWT_CYCLES = 54;

// GPT2 ISR entry latency — retained for PPS-path reference.
// GPT2 is now dedicated to OCXO2 counting; no longer used for
// TimePop compare.
static constexpr uint32_t GPT2_ISR_ENTRY_DWT_CYCLES = 49;

// ============================================================================
// dwt_at_event — recover the DWT_CYCCNT at the actual hardware event
//
// Given a DWT_CYCCNT captured as the first instruction inside an ISR,
// subtracts the ISR entry latency to estimate the DWT value at the
// moment the hardware event (e.g., compare match) actually fired.
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
// Convenience: QTimer1 CH2 compare ISR (TimePop production)
// ============================================================================

static inline uint32_t dwt_at_qtimer1_ch2_compare(uint32_t isr_dwt_cyccnt) {
  return dwt_at_event(isr_dwt_cyccnt, QTIMER1_CH2_ISR_ENTRY_DWT_CYCLES);
}

// ============================================================================
// Convenience: GPT2 ISR (retained for reference)
// ============================================================================

static inline uint32_t dwt_at_gpt2_compare(uint32_t isr_dwt_cyccnt) {
  return dwt_at_event(isr_dwt_cyccnt, GPT2_ISR_ENTRY_DWT_CYCLES);
}