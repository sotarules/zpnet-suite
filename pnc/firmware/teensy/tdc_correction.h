#pragma once

#include <Arduino.h>
#include <imxrt.h>
#include <stdint.h>

// ============================================================================
// tdc_correction.h — ISR Edge Timing Constants and Helpers (v18 QTimer)
// ============================================================================
//
// Every shadow-write loop in ZPNet exists to place the Cortex-M7 into
// a consistent, well-characterized execution state before a key
// interrupt arrives.  Under that conditioned state, the authoritative
// estimate of the physical edge is a fixed subtraction from the first
// DWT_CYCCNT captured in the ISR.
//
// Architecture (v18):
//
//   PPS edge:         pps_isr (priority 0, GPIO direct vector)
//                     shadow loop via prespin_service (scheduled context)
//                     edge_dwt = isr_dwt - PPS_ISR_FIXED_OVERHEAD
//
//   QTimer3 CH2/CH3:  qtimer3_isr (priority 0, shared vector)
//                     shadow loop via prespin_service (scheduled context)
//                     edge_dwt = isr_dwt - QTIMER_ISR_FIXED_OVERHEAD
//
//   QTimer1 CH2:      qtimer1_ch2_isr (priority 16, via qtimer1_irq_isr)
//                     no shadow loop — used by TimePop DWT prediction
//                     edge_dwt = isr_dwt - QTIMER1_CH2_ISR_FIXED_OVERHEAD
//
//   QTimer1 CH3:      time_test_ch3_isr (priority 16, via qtimer1_irq_isr)
//                     shadow loop in time_test_callback (scheduled context)
//                     edge_dwt = isr_dwt - TIME_TEST_ISR_FIXED_OVERHEAD
//
// Calibration:
//
//   PPS:           tdc_analyzer (spin_delta_cycles histogram)
//   QTimer3 OCXO:  edge_tdc_analyzer — NEEDS RECALIBRATION after migration
//   CH2:           TIMEPOP DIAG (dwt_pred_mean → adjust until ≈ 0)
//   CH3:           time_test_analyzer (isr_delta_cycles histogram)
//
// ============================================================================

// ── PPS ISR — GPIO direct vector, priority 0 ──
// Shadow loop: prespin_service (scheduled context)
// Empirical minimum: 48 cycles across >10,000 samples.
static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD = 48;

// ── QTimer3 OCXO ISR — direct vector, priority 0 ──
// Shadow loop: prespin_service (scheduled context)
//
// INITIAL ESTIMATE: 50 cycles.
//
// This is a starting value for bring-up.  The QTimer3 ISR path is:
//   NVIC + stacking (~12 cycles at priority 0, no tail-chaining)
//   + DWT_CYCCNT capture (~1 cycle, first instruction)
//   + SCTRL flag check + branch (~4 cycles)
//   + function call to ch2/ch3 handler (~3 cycles)
//   ... total entry overhead before handle_event: ~20 cycles
//
// However, QTIMER_ISR_FIXED_OVERHEAD represents the full path from
// the physical edge (compare match) to the first DWT capture in the
// ISR, which includes NVIC latency.  The empirical minimum from
// shadow_to_isr_cycles histograms will determine the true value.
//
// MUST BE RECALIBRATED from production histogram data.
//
static constexpr uint32_t QTIMER_ISR_FIXED_OVERHEAD = 50;

// ── QTimer1 CH2 ISR — via qtimer1_irq_isr dispatcher, priority 16 ──
// No shadow loop.  Used by TimePop for DWT prediction.
static constexpr uint32_t QTIMER1_CH2_ISR_FIXED_OVERHEAD = 54;

// ── QTimer1 CH3 ISR — via qtimer1_irq_isr dispatcher, priority 16 ──
// Shadow loop: time_test_callback (scheduled context)
static constexpr uint32_t TIME_TEST_ISR_FIXED_OVERHEAD = 54;

// ── Legacy GPT constant — retained for reference only ──
// static constexpr uint32_t GPT_ISR_FIXED_OVERHEAD = 17;

// ============================================================================
// Edge DWT helpers
// ============================================================================

static inline uint32_t pps_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - PPS_ISR_FIXED_OVERHEAD;
}

static inline uint32_t qtimer_ocxo_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - QTIMER_ISR_FIXED_OVERHEAD;
}

static inline uint32_t qtimer1_ch2_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - QTIMER1_CH2_ISR_FIXED_OVERHEAD;
}

static inline uint32_t time_test_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - TIME_TEST_ISR_FIXED_OVERHEAD;
}

// ── PPS peripheral correction — recover 10 MHz tick at true PPS edge ──

static inline uint32_t pps_correct_10mhz(uint32_t raw_value, uint32_t base_dwt, volatile uint32_t* diag_dwt) {
  const uint32_t now = ARM_DWT_CYCCNT;
  if (diag_dwt) *diag_dwt = now;
  const uint32_t elapsed = now - base_dwt + PPS_ISR_FIXED_OVERHEAD;
  return raw_value - (elapsed / 101);
}