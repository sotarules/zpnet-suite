#pragma once

// ============================================================================
// TimePop v6.0 — Process Interface
// ============================================================================
//
// System initialization and process registration only.
// General consumers include timepop.h, not this file.
//
// Commands:
//   REPORT — active timer diagnostics (includes per-slot predicted_dwt)
//   DIAG   — DWT prediction + GNSS-now Welford statistics
//   TEST   — timer accuracy verification { "ns": <uint64> }
//   NS_TEST — nano-precise timer accuracy test
//   INTERP_TEST — dual-path validation (legacy, pending removal)
//   VCLOCK_TEST — historical direct compare validation (retained)
//
// v5.3a: Added IOMUXC_QTIMER3_TIMER3_SELECT_INPUT daisy chain register.
//
//   v5.3 had correct IOMUX mux/pad setup for pin 15 (GPIO_AD_B1_03,
//   ALT1 = QTIMER3_TIMER3) but omitted the input select register that
//   routes the physical pad signal to the QTimer3 CH3 peripheral input.
//   Without the daisy chain register, the counter stayed at zero.
//
//   Pin 15 (GPIO_AD_B1_03, IOMUX ALT1 = QTIMER3_TIMER3) receives the
//   same GNSS 10 MHz signal as pin 10 via a Kynar 26 AWG bridge.
//   QTimer3 CH3 counts GNSS edges independently.  When its COMP1
//   fires, the ISR reads QTimer1 CH0+CH1 (the undisturbed 32-bit
//   passive counter) for alias rejection against the full 32-bit
//   target.
//
//   IRQ_QTIMER3 is a completely separate NVIC vector from
//   IRQ_QTIMER1.  No shared-IRQ interactions.
//
//   Pin 15 is normally PHOTODIODE_ADC — temporarily reassigned
//   for VCLOCK_TEST POC validation.
//
// v5.0: Continuous DWT prediction and GNSS-now validation.
//
//   Every standard timer fire produces two measurements:
//     1. DWT prediction residual: actual DWT_CYCCNT vs predicted
//     2. GNSS-now residual: time_dwt_to_gnss_ns(captured_dwt) vs
//        the GNSS nanosecond derived from the VCLOCK position
//
//   Welford accumulators maintain running statistics across all
//   timer fires.  DIAG dumps the full statistical state.
//
// ============================================================================

#include "timepop.h"
#include <stdint.h>

/// Initialize TimePop hardware (QTimer1 CH2 production compare + CH3
/// historical VCLOCK_TEST doorbell). QTimer1 must already be running
/// (process_clocks_init_hardware). Must be called once during setup().
void timepop_init(void);

/// Register the TIMEPOP process with the command framework.
void process_timepop_register(void);

//   VCLOCK_TEST — direct QTimer3 CH3 compare validation { "ns": <uint64> }
//                 arm_ns range: 1,000,000–999,000,000
//                 bypasses timepop_arm(), measures direct VCLOCK IRQ timing
//                 Pin 15 must be bridged to GNSS 10 MHz (same signal as pin 10)