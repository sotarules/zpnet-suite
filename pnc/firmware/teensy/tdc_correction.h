#pragma once

#include <stdint.h>

// ============================================================================
// tdc_correction.h — ISR Edge Timing Constants and Helpers
// ============================================================================
//
// Every shadow-write loop in ZPNet exists to place the Cortex-M7 into
// a consistent, well-characterized execution state before a key
// interrupt arrives.  Under that conditioned state, the authoritative
// estimate of the physical edge is a fixed subtraction from the first
// DWT_CYCCNT captured in the ISR.
//
// The constants below are the empirically measured minimum ISR entry
// latencies (isr_dwt - shadow_dwt) across thousands of samples.  The
// minimum represents the case where the shadow write executed
// immediately before preemption — maximally fresh.  Any delta above
// the minimum is loop-position jitter (the ISR caught the CPU at a
// different point in the loop body).
//
// These constants are sacred: changing the shadow-write loop body
// changes the ISR entry latency distribution.  The loop must remain
// unchanged, and the constants must be re-derived from the loop via
// histogram analysis (tdc_analyzer, time_test_analyzer) whenever the
// loop is modified.
//
// Architecture:
//
//   PPS edge:        pps_isr (priority 0, GPIO direct vector)
//                    shadow loop in pps_spin_callback (ISR context, priority 16)
//                    edge_dwt = isr_dwt - PPS_ISR_FIXED_OVERHEAD
//
//   GPT1/GPT2 edge:  gpt1/2_phase_isr (priority 0, direct vector)
//                    shadow loop in pps_asap_callback (scheduled context)
//                    edge_dwt = isr_dwt - GPT_ISR_FIXED_OVERHEAD
//
//   QTimer1 CH2:     qtimer1_ch2_isr (priority 16, via qtimer1_irq_isr)
//                    no shadow loop — used by TimePop DWT prediction
//                    edge_dwt = isr_dwt - QTIMER1_CH2_ISR_FIXED_OVERHEAD
//
//   QTimer1 CH3:     time_test_ch3_isr (priority 16, via qtimer1_irq_isr)
//                    shadow loop in time_test_callback (scheduled context)
//                    edge_dwt = isr_dwt - TIME_TEST_ISR_FIXED_OVERHEAD
//
// Calibration:
//
//   PPS:       tdc_analyzer (spin_delta_cycles histogram)
//   GPT1/GPT2: gpt_phase_tdc / edge_tdc_analyzer
//   CH2:       TIMEPOP DIAG (dwt_pred_mean → adjust until ≈ 0)
//   CH3:       time_test_analyzer (isr_delta_cycles histogram)
//
// ============================================================================

// ── PPS ISR — GPIO direct vector, priority 0 ──
// Shadow loop: pps_spin_callback (ISR context via timepop_arm_ns)
// Empirical minimum: 48 cycles across >10,000 samples.
static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD = 48;

// ── GPT1/GPT2 ISR — direct vector, priority 0 ──
// Shadow loop: pps_asap_callback OCXO phase spin (scheduled context)
// Empirical minimum: 17 cycles.
static constexpr uint32_t GPT_ISR_FIXED_OVERHEAD = 17;

// ── QTimer1 CH2 ISR — via qtimer1_irq_isr dispatcher, priority 16 ──
// No shadow loop.  Used by TimePop for DWT prediction.
// Calibrated via TIMEPOP DIAG: adjust until dwt_pred_mean ≈ 0.
//
// Dispatch path:
//   NVIC + stacking (~20) + dispatcher prologue (~3) +
//   CH3 flag check (~4, since CH3 is checked first) +
//   CH2 flag check + branch (~4) + ch2_isr entry (~2) +
//   compiler variance (~2) = ~54 cycles.
//
// NOTE: The CH3 flag check added for TIME_TEST adds ~4 cycles to
// the CH2 path.  Re-calibrate via TIMEPOP DIAG after any change
// to the qtimer1_irq_isr dispatcher.
static constexpr uint32_t QTIMER1_CH2_ISR_FIXED_OVERHEAD = 54;

// ── QTimer1 CH3 ISR — via qtimer1_irq_isr dispatcher, priority 16 ──
// Shadow loop: time_test_callback (scheduled context)
// Empirical minimum: 54 cycles across 477 samples (time_test_analyzer).
//
// Dispatch path:
//   NVIC + stacking (~20) + dispatcher prologue (~3) +
//   CH3 flag check + branch (~4) + ch3 body entry (~2) +
//   compiler variance (~2) = ~54 cycles.
//
// CH3 is checked first in qtimer1_irq_isr, so this path is
// slightly shorter than CH2.  The empirical value matches CH2
// because NVIC overhead dominates.
static constexpr uint32_t TIME_TEST_ISR_FIXED_OVERHEAD = 54;

// ============================================================================
// Edge DWT helpers — subtract ISR overhead to recover DWT at event
//
// Given a DWT_CYCCNT captured as the first instruction inside an ISR,
// subtracts the fixed ISR entry latency to estimate the DWT value at
// the moment the hardware event (edge, compare match) actually fired.
// ============================================================================

static inline uint32_t pps_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - PPS_ISR_FIXED_OVERHEAD;
}

static inline uint32_t gpt_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - GPT_ISR_FIXED_OVERHEAD;
}

static inline uint32_t qtimer1_ch2_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - QTIMER1_CH2_ISR_FIXED_OVERHEAD;
}

static inline uint32_t time_test_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - TIME_TEST_ISR_FIXED_OVERHEAD;
}