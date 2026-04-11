#pragma once

#include <Arduino.h>
#include <imxrt.h>
#include <stdint.h>

// ============================================================================
// tdc_correction.h — ISR Edge Timing Constants and Helpers (v18 QTimer)
// ============================================================================

static constexpr uint32_t PPS_ISR_FIXED_OVERHEAD = 47;
static constexpr uint32_t QTIMER_ISR_FIXED_OVERHEAD = 16;

// ============================================================================
// Edge DWT helpers
// ============================================================================

static inline uint32_t pps_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - PPS_ISR_FIXED_OVERHEAD;
}

static inline uint32_t qtimer_ocxo_dwt_at_edge(uint32_t isr_dwt) {
  return isr_dwt - QTIMER_ISR_FIXED_OVERHEAD;
}
