#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — TimePop v8.0 Public Interface
// ============================================================================
//
// TimePop is ZPNet's phase-locked timer subsystem.  All timers run at
// 10 MHz GNSS resolution (100 ns per tick) with DWT nano-spin delivery
// for cycle-level precision at the fire moment.
//
// Architecture:
//
//   QTimer1 CH0+CH1 form a passive 32-bit counter at 10 MHz (CM=1,
//   rising edges of GNSS VCLOCK).  CH2 is a dynamic compare channel
//   implementing a priority queue scheduler — armed to the low 16 bits
//   of the next soonest deadline, with 32-bit alias rejection in the ISR.
//
//   Every timer — recurring or one-shot — is nano-precise.  The ISR
//   DWT-spins to the predicted cycle count (≤100 ns spin) and captures
//   the fire state.  Callbacks execute in scheduled (non-ISR) context
//   via timepop_dispatch().
//
//   A 1 ms ceiling heartbeat ensures the system never goes silent.
//
// Two arming modes:
//
//   timepop_arm (delay in nanoseconds):
//     Deadline = vclock_count() + ns_to_ticks(delay_ns).
//     DWT prediction computed at arm time.
//     Supports recurring timers.
//
//   timepop_arm_ns (target GNSS nanosecond + DWT):
//     Caller owns the target computation.  The QTimer deadline is
//     derived from the delay.  The caller-provided DWT target is
//     used directly for the spin.  One-shot only.
//
// Timer context:
//
//   Every callback receives a timepop_ctx_t with:
//     fire_vclock_raw  — QTimer1 32-bit value at ISR entry
//     deadline         — the target QTimer1 tick
//     fire_ns          — (fire_vclock_raw - deadline) * 100 (lateness in ns)
//     fire_gnss_ns     — GNSS nanosecond at the spin-landed DWT moment
//     fire_dwt_cyccnt  — DWT_CYCCNT at spin landing
//
// v8.0: Priority queue with single hardware comparator, 10 MHz.
// v5.0: Campaign-independent.  All time references use time.h.
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
  uint32_t         fire_vclock_raw; // qtimer1_read_32() at ISR entry
  uint32_t         deadline;        // target QTimer1 10 MHz tick
  int32_t          fire_ns;         // (fire_vclock_raw - deadline) * 100

  // ── GNSS nanosecond at fire moment ──
  int64_t          fire_gnss_ns;    // GNSS ns at spin-landed DWT

  // ── Compatibility fields (always true / always false in v8.0) ──
  bool             nano_precise;    // always true in v8.0
  bool             nano_timeout;    // always false in v8.0

  // ── DWT capture ──
  uint32_t         fire_dwt_cyccnt; // DWT_CYCCNT at spin landing
} timepop_ctx_t;

// ============================================================================
// Callback signature
// ============================================================================

typedef void (*timepop_callback_t)(
  timepop_ctx_t* ctx,
  void*          user_data
);

// ============================================================================
// Arm a timer (100 ns resolution, scheduled context callback)
// ============================================================================
//
// delay_ns:
//   Nanoseconds until the timer fires.  Converted to 10 MHz ticks
//   internally (1 tick = 100 ns).
//
//   delay_ns == 0 is ASAP: the callback fires on the next
//   timepop_dispatch() call without entering the ISR path.
//
// recurring:
//   If true, the timer rearms automatically after each fire.
//   The deadline advances by period_ticks to maintain phase lock.
//
// The callback receives a timepop_ctx_t with fire_dwt_cyccnt set
// to the predicted DWT at the deadline moment.
//
// Returns TIMEPOP_INVALID_HANDLE on failure (slots full, etc.).
//

timepop_handle_t timepop_arm(
  uint64_t            delay_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Arm a nano-precise timer (caller-owned target, one-shot)
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
// The QTimer deadline is derived from the delay.  The ISR DWT-spins
// to the caller-provided target_dwt.
//
// The callback fires in scheduled context via timepop_dispatch().
// fire_gnss_ns contains the GNSS nanosecond at the landed DWT value.
//
// One-shot only.  Not available for recurring timers.
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
// Dispatch (scheduled context — all callbacks fire here)
// ============================================================================

void timepop_dispatch(void);

// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void);