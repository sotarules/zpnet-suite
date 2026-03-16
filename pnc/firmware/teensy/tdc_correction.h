#pragma once

#include <stdint.h>

// ============================================================================
// Software TDC (Time-to-Digital Converter) — 1008 MHz Correction
// ============================================================================
//
// Derived 2026-03-15 from spin capture + disassembly at 1008 MHz.
//
// Architecture:
//
//   A nano-precise TimePop callback lands ~50 µs before the PPS edge
//   and enters a tight shadow-write loop.  The PPS ISR (priority 0)
//   preempts the loop (priority 16), captures the shadow value, and
//   breaks the loop.
//
//   The delta (isr_snap_dwt - shadow_dwt) reveals which instruction
//   in the loop was executing when the PPS interrupt fired.  This
//   delta is deterministic and quantized by the loop's instruction
//   timing.
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
// Empirical histogram (109 samples, Baseline1 campaign):
//
//   delta 47:  16 hits (14.7%)   ← ISR caught at ldr  (instruction 0)
//   delta 48:  25 hits (22.9%)   ← ISR caught at str  (instruction 1)
//   delta 49:  32 hits (29.4%)   ← ISR caught at ldrb (instruction 2)
//   delta 50:  16 hits (14.7%)   ← ISR caught at cmp  (instruction 3)
//   delta 51:  20 hits (18.3%)   ← ISR caught at beq  (instruction 4)
//
//   100% of samples fall within [47, 51].  Zero outliers.
//   stddev = 1.3 cycles (1.3 ns).
//
// Correction:
//
//   The shadow_dwt value was captured by the `str` instruction at
//   address 0x2372, using the DWT_CYCCNT loaded by the `ldr` at
//   address 0x2370.  The shadow is therefore the DWT value from
//   the *start* of the loop iteration in which the ISR fired.
//
//   The true PPS-edge DWT is:
//
//     pps_edge_dwt = shadow_dwt + (delta - TDC_FIXED_OVERHEAD)
//
//   Where the correction (delta - 47) accounts for how far into
//   the loop iteration the ISR caught us:
//
//     correction 0: ISR caught at the ldr (shadow is current)
//     correction 1: ISR caught at the str (shadow is 1 cycle old)
//     correction 2: ISR caught at the ldrb (shadow is 2 cycles old)
//     correction 3: ISR caught at the cmp (shadow is 3 cycles old)
//     correction 4: ISR caught at the beq (shadow is 4 cycles old)
//
//   The corrected value is the DWT_CYCCNT at the PPS rising edge,
//   accurate to ±0.5 DWT cycles (±0.5 ns).
//
// Constants:
//
//   TDC_FIXED_OVERHEAD:  The minimum observed delta.  This is the
//                        irreducible ISR entry latency: register
//                        stacking (~12 cycles), vector fetch, plus
//                        the instructions in pps_isr() before the
//                        first DWT_CYCCNT read.
//
//   TDC_LOOP_CYCLES:     DWT cycles per loop iteration.  At 1008 MHz
//                        on Cortex-M7, the 5-instruction loop executes
//                        in effectively 1 cycle per instruction due to
//                        dual-issue pipelining.  The histogram spacing
//                        confirms 1 cycle between clusters.
//
//   TDC_MAX_CORRECTION:  Maximum valid correction value.  Deltas
//                        outside [OVERHEAD, OVERHEAD + MAX_CORRECTION]
//                        indicate the spin loop was not running when
//                        the PPS fired (miss, timeout, or not armed).
//
// ============================================================================

static constexpr uint32_t TDC_FIXED_OVERHEAD  = 47;   // min delta (ISR entry latency)
static constexpr uint32_t TDC_LOOP_CYCLES     = 1;    // cycles per loop iteration
static constexpr uint32_t TDC_MAX_CORRECTION  = 4;    // max correction (delta range 47-51)

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
// in DWT cycles (0-4), or -1 if no correction was applied.
// ============================================================================

static inline uint32_t tdc_correct(
  uint32_t shadow_dwt,
  int32_t  delta_cycles,
  int32_t& correction_applied
) {
  // Delta must be within the expected range [47, 51].
  // Values outside this range mean the spin loop was not active
  // when the PPS fired — fall back to uncorrected shadow.
  if (delta_cycles < (int32_t)TDC_FIXED_OVERHEAD) {
    correction_applied = -1;
    return shadow_dwt;
  }

  uint32_t correction = (uint32_t)(delta_cycles - (int32_t)TDC_FIXED_OVERHEAD);

  if (correction > TDC_MAX_CORRECTION) {
    correction_applied = -1;
    return shadow_dwt;
  }

  correction_applied = (int32_t)correction;
  return shadow_dwt + correction;
}