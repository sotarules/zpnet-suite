#pragma once

#include <stdint.h>

// ============================================================================
// Software TDC (Time-to-Digital Converter) — 1008 MHz Correction
// ============================================================================
//
// v8.1 recalibration: 2026-03-20, Shakeout4 campaign (138 samples).
//
// Architecture:
//
//   A nano-precise TimePop ISR callback (priority 16) lands ~50 µs
//   before the PPS edge and enters a tight shadow-write loop.  The
//   PPS ISR (priority 0) preempts the callback via NVIC nesting,
//   captures the shadow value, and breaks the loop.
//
//   The delta (isr_snap_dwt - shadow_dwt) reveals which instruction
//   in the loop was executing when the PPS interrupt fired.  This
//   delta is deterministic and quantized by the loop's instruction
//   timing, plus variable NVIC nesting overhead.
//
// Disassembly of the shadow-write loop at 1008 MHz (Cortex-M7):
//
//   2370:  684b      ldr   r3, [r1, #4]     ; load DWT_CYCCNT
//   2372:  6023      str   r3, [r4, #0]     ; store to dispatch_shadow_dwt
//   2374:  7803      ldrb  r3, [r0, #0]     ; load pps_fired
//   2376:  2b00      cmp   r3, #0           ; compare to zero
//   2378:  d0fa      beq.n 2370             ; branch back if not fired
//
//   5 instructions per iteration, ~1 DWT cycle per instruction
//   on the Cortex-M7 dual-issue pipeline at 1008 MHz.
//
// v8.1 empirical histogram (138 samples, Shakeout4 campaign):
//
//   delta 48:  10 hits (  7.2%)
//   delta 49:  18 hits ( 13.0%)
//   delta 50:  11 hits (  8.0%)
//   delta 51:  14 hits ( 10.1%)
//   delta 52:  16 hits ( 11.6%)
//   delta 53:  19 hits ( 13.8%)
//   delta 54:  17 hits ( 12.3%)
//   delta 55:  19 hits ( 13.8%)
//   delta 56:  14 hits ( 10.1%)
//
//   100% of samples fall within [48, 56].  Zero outliers.
//   mean = 52.26 cycles (51.8 ns), stddev = 2.49 cycles (2.5 ns).
//
// Comparison with v7.0 (non-nested ISR, Baseline1 campaign, 109 samples):
//
//   v7.0: delta 47-51, 5 clusters, stddev 1.3 cycles
//   v8.1: delta 48-56, 9 values,   stddev 2.5 cycles
//
//   The wider spread in v8.1 is due to NVIC nested interrupt overhead:
//   the PPS ISR (priority 0) preempts the TimePop callback running
//   inside the CH2 ISR (priority 16).  The tail-chain latency varies
//   by ~3-4 cycles depending on Cortex-M7 pipeline state at the
//   moment of preemption.  This adds to the base 5-cycle loop
//   quantization, producing a 9-cycle spread instead of 5.
//
// Correction:
//
//   The shadow_dwt value was captured by the `str` instruction,
//   using the DWT_CYCCNT loaded by the preceding `ldr`.  The shadow
//   is the DWT value from the *start* of the loop iteration in which
//   the ISR fired.
//
//   The true PPS-edge DWT is:
//
//     pps_edge_dwt = shadow_dwt + (delta - TDC_FIXED_OVERHEAD)
//
//   Where the correction (delta - 48) accounts for how far into
//   the loop iteration (plus NVIC nesting jitter) the ISR caught us.
//
//   The corrected value is the DWT_CYCCNT at the PPS rising edge,
//   accurate to ±0.5 DWT cycles (±0.5 ns) within the correction
//   window, with ~2.5 ns of additional jitter from the NVIC nesting.
//
// Constants:
//
//   TDC_FIXED_OVERHEAD:  The minimum observed delta.  This is the
//                        irreducible ISR entry latency: NVIC nesting
//                        overhead (stacking outer frame + tail-chain),
//                        register stacking for PPS ISR, vector fetch,
//                        plus the instructions in pps_isr() before
//                        the first DWT_CYCCNT read.
//
//   TDC_LOOP_CYCLES:     DWT cycles per loop iteration.  At 1008 MHz
//                        on Cortex-M7, the 5-instruction loop executes
//                        in effectively 1 cycle per instruction due to
//                        dual-issue pipelining.  The histogram spacing
//                        confirms 1 cycle between values.
//
//   TDC_MAX_CORRECTION:  Maximum valid correction value.  Deltas
//                        outside [OVERHEAD, OVERHEAD + MAX_CORRECTION]
//                        indicate the spin loop was not running when
//                        the PPS fired (miss, timeout, or not armed).
//
// ============================================================================

static constexpr uint32_t TDC_FIXED_OVERHEAD  = 48;   // min delta (nested ISR entry latency)
static constexpr uint32_t TDC_LOOP_CYCLES     = 1;    // cycles per loop iteration
static constexpr uint32_t TDC_MAX_CORRECTION  = 16;

// ============================================================================
// tdc_correct — apply TDC correction to a spin capture
//
// Given the shadow DWT and the delta (isr_snap_dwt - shadow_dwt),
// returns the corrected DWT value at the true PPS edge.
//
// Returns the corrected DWT, or shadow_dwt unchanged if the delta
// is outside the valid range (indicating a miss or anomaly).
//
// The out parameter `correction_applied` receives the correction
// in DWT cycles (0-16), or -1 if no correction was applied.
// ============================================================================

static inline uint32_t tdc_correct(
  uint32_t shadow_dwt,
  int32_t  delta_cycles,
  int32_t& correction_applied
) {
  const int32_t correction = delta_cycles - (int32_t)TDC_FIXED_OVERHEAD;

  if (correction >= 0 && correction <= (int32_t)TDC_MAX_CORRECTION) {
    // Delta is within the valid TDC window — apply correction.
    correction_applied = correction;
    return shadow_dwt + (uint32_t)correction;
  }

  // Delta outside valid range — spin loop was not running at PPS,
  // or an anomalous condition occurred.  Return uncorrected.
  correction_applied = -1;
  return shadow_dwt;
}