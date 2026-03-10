#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// TimePop v2 — GPT2 Output Compare Timer System
// ============================================================================
//
// TimePop provides nanosecond-resolution software timers backed by the
// GF-8802 GNSS 10 MHz clock on GPT2.
//
// The time experience in ZPNet is the GNSS clock.  Period.
//
// Architecture:
//
//   GPT2 free-runs at 10 MHz (100 ns/tick), clocked directly by the
//   GF-8802 GNSS VCLOCK.  This signal is present from power-on —
//   it is hardware, not firmware-dependent.
//
//   TimePop maintains a priority queue of deadlines expressed as
//   GPT2 count values.  OCR1 is always loaded with the nearest
//   deadline.  When the compare matches, an ISR fires and captures
//   GPT2_CNT at that instant.  This timestamp is carried through
//   to the callback via the context struct, giving every callback
//   the exact GNSS tick at which it was triggered — independent
//   of dispatch latency.
//
// API:
//
//   Everything speaks nanoseconds.  The conversion is trivial:
//
//     gpt2_ticks = nanoseconds / 100
//
//   No clock domains.  No drift correction.  No conversion ratios.
//   The timer hardware IS the GNSS clock.
//
// Callback Context:
//
//   Every callback receives a timepop_ctx_t with:
//
//     handle       — for cancellation
//     fire_gpt2    — GPT2_CNT captured in the ISR at the moment
//                    the deadline was detected as expired
//     deadline     — the GPT2 count the timer was targeting
//
//   fire_gpt2 - deadline = OCR hardware precision (in ticks)
//   Multiply by 100 for nanoseconds.
//
//   This allows callbacks to know exactly when they were triggered
//   in GNSS time, regardless of how long dispatch took.
//
// Resolution:
//
//   100 ns (one GPT2 tick at 10 MHz).
//   Hardware-precise triggering via output compare.
//   Zero ISR overhead between timer fires.
//
// Wrap handling:
//
//   GPT2 is 32-bit.  At 10 MHz it wraps every ~429 seconds.
//   All deadline comparisons use unsigned delta arithmetic.
//   Maximum single-shot delay: ~214 seconds (half the wrap range).
//
// Concurrency:
//
//   Up to 16 timers may be active simultaneously.
//   Callbacks execute in scheduled (non-ISR) context.
//
// ============================================================================

// ============================================================================
// Timer handle (opaque, for cancellation)
// ============================================================================

typedef uint32_t timepop_handle_t;

static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Timer context (passed to callbacks)
// ============================================================================
//
// fire_gpt2:  GPT2_CNT captured in the ISR at the moment the timer
//             was detected as expired.  This is the authoritative
//             "when did this fire?" timestamp in GNSS ticks.
//
// deadline:   The GPT2 count value the timer was targeting.
//
// fire_ns:    fire_gpt2 converted to nanoseconds offset from deadline.
//             (fire_gpt2 - deadline) * 100.  Positive = ISR arrived
//             after the ideal moment (normal).
//
// handle:     Opaque handle for cancellation.
//

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t         fire_gpt2;    // GPT2_CNT at ISR entry
  uint32_t         deadline;     // target GPT2 count
  int32_t          fire_ns;      // (fire_gpt2 - deadline) * 100

  // ── DWT + GNSS snapshot (v3) ──
  //
  // Captured in the ISR at the same instant as fire_gpt2.
  // Together with frag_gnss_ns and frag_dwt_cycles, these allow
  // the callback to reconstruct the exact GNSS nanosecond at the
  // fire moment via timebase_now_gnss_ns_from_dwt().
  //
  // fire_dwt_cyccnt:  raw ARM_DWT_CYCCNT register at ISR entry.
  // frag_gnss_ns:     fragment gnss_ns at PPS (base for interpolation).
  // frag_dwt_cycles:  fragment dwt_cycles at PPS (64-bit accumulator).
  // frag_dwt_cyccnt_at_pps:  raw DWT_CYCCNT at PPS edge (32-bit anchor).
  // frag_dwt_cycles_per_pps: DWT cycles in the prior second (denominator).
  // frag_valid:       true if a valid fragment was available at ISR time.

  uint32_t         fire_dwt_cyccnt;         // DWT_CYCCNT at ISR entry
  uint64_t         frag_gnss_ns;            // fragment.gnss_ns
  uint64_t         frag_dwt_cycles;         // fragment.dwt_cycles
  uint32_t         frag_dwt_cyccnt_at_pps;  // fragment.dwt_cyccnt_at_pps
  uint32_t         frag_dwt_cycles_per_pps; // fragment.dwt_cycles_per_pps
  uint32_t         frag_gpt2_at_pps;        // fragment.gpt2_at_pps (VCLOCK anchor)
  bool             frag_valid;              // was fragment available?
} timepop_ctx_t;

// ============================================================================
// Callback signature
// ============================================================================

typedef void (*timepop_callback_t)(
  timepop_ctx_t* ctx,
  void*          user_data
);

// ============================================================================
// Arm a timer
// ============================================================================
//
// Schedule a callback to fire after `delay_ns` nanoseconds of GNSS time.
//
// Resolution: 100 ns.  Values are rounded down to the nearest 100 ns.
//
// If recurring=true, the timer re-arms automatically after each fire
// with the same delay.
//
// Special case: delay_ns=0 means "fire as soon as possible" (ASAP).
// The callback will execute on the next timepop_dispatch() call.
// For ASAP timers, fire_gpt2 and deadline will both be 0.
//
// Returns a handle for cancellation, or TIMEPOP_INVALID_HANDLE on
// failure (all slots in use, or callback is null).
//

timepop_handle_t timepop_arm(
  uint64_t            delay_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Cancel a timer by handle
// ============================================================================
//
// Returns true if a timer was found and cancelled.
//

bool timepop_cancel(timepop_handle_t handle);

// ============================================================================
// Cancel all timers with a given name
// ============================================================================
//
// Returns the number of timers cancelled.
//

uint32_t timepop_cancel_by_name(const char* name);

// ============================================================================
// Dispatch
// ============================================================================
//
// Process expired timers.  Must be called from the main loop.
// All callbacks execute in this context (non-ISR).
//

void timepop_dispatch(void);

// ============================================================================
// Introspection
// ============================================================================

/// Number of currently active timers.
uint32_t timepop_active_count(void);