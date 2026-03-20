#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// TimePop v5.0 — GPT2 Output Compare Timer System + Nano-Precise DWT Spin
// ============================================================================
//
// Two delivery modes:
//
//   Standard (timepop_arm):
//     Resolution: 100 ns (one GPT2 tick).
//     Callbacks execute in scheduled (non-ISR) context.
//
//   Nano-precise (timepop_arm_ns):
//     Resolution: ~5 ns (DWT cycle-level).
//     Caller provides the target GNSS nanosecond and DWT cycle count.
//     The ISR fires early, spins on DWT_CYCCNT to the exact target,
//     then fires the callback immediately in ISR context.
//     fire_gnss_ns contains the GNSS nanosecond at the landed DWT value.
//
// v5.0: Campaign-independent.  All time references use time.h, not
//       the timebase fragment.  No campaign dependency whatsoever.
//
// ============================================================================

// ============================================================================
// Timer handle
// ============================================================================

typedef uint32_t timepop_handle_t;

static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Timer context (passed to callbacks)
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t         fire_gpt2;       // GPT2_CNT at ISR entry
  uint32_t         deadline;        // target GPT2 count
  int32_t          fire_ns;         // (fire_gpt2 - deadline) * 100

  // ── Nano-precise fields ──
  int64_t          fire_gnss_ns;    // GNSS ns at spin-landed DWT (-1 for standard)
  bool             nano_precise;    // true if nano-precise callback
  bool             nano_timeout;    // true if DWT spin timed out

  // ── DWT capture ──
  uint32_t         fire_dwt_cyccnt; // DWT_CYCCNT at fire (compensated for ISR latency)
} timepop_ctx_t;

// ============================================================================
// Callback signature
// ============================================================================

typedef void (*timepop_callback_t)(
  timepop_ctx_t* ctx,
  void*          user_data
);

// ============================================================================
// Arm a standard timer (50 ns raw resolution, scheduled context)
// ============================================================================

timepop_handle_t timepop_arm(
  uint64_t            delay_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Arm a nano-precise timer (~5 ns resolution, ISR context)
// ============================================================================
//
// The caller owns the target computation:
//
//   int64_t  now_ns     = time_gnss_ns_now();
//   int64_t  target_ns  = now_ns + delay_ns;
//   uint32_t target_dwt = time_gnss_ns_to_dwt(target_ns);
//   timepop_arm_ns(target_ns, target_dwt, cb, ud, "name");
//
// This eliminates arm-path bias — the target in the slot is exactly
// the target the caller computed, with no recomputation delay.
//
// The GPT2 deadline is derived from the delay (target_ns minus the
// current GNSS time estimate), backed off by NANO_EARLY_TICKS so
// the ISR fires early and the DWT spin covers the final stretch.
//
// The callback fires in ISR context.  Keep it fast.
// Not available for recurring timers.
//
// Returns TIMEPOP_INVALID_HANDLE on failure (slots full, etc.).
//

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Cancel
// ============================================================================

bool timepop_cancel(timepop_handle_t handle);

uint32_t timepop_cancel_by_name(const char* name);

// ============================================================================
// Dispatch (scheduled context — standard timers only)
// ============================================================================

void timepop_dispatch(void);

// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void);