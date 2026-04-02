#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — TimePop v9 Public Interface
// ============================================================================
//
// TimePop is ZPNet's phase-locked timer subsystem.  All timers run at
// 10 MHz GNSS resolution (100 ns per tick).
//
// Architecture:
//
//   QTimer1 CH0+CH1 form a passive 32-bit counter at 10 MHz (CM=1,
//   rising edges of GNSS VCLOCK).  CH2 is a dynamic compare channel
//   implementing a priority queue scheduler — armed to the low 16 bits
//   of the next soonest deadline, with 32-bit alias rejection in the ISR.
//
//   A 1 ms ceiling heartbeat ensures the system never goes silent.
//
// v9: VCLOCK-exact GNSS nanoseconds.
//
//   The VCLOCK is phase-locked to GNSS at 10 MHz.  Every tick is
//   exactly 100 ns.  The GNSS nanosecond at any VCLOCK tick is pure
//   arithmetic:
//
//     gnss_ns = (pps_count - 1) * 1,000,000,000
//             + (vclock_tick - qtimer_at_pps) * 100
//
//   No DWT involved.  No prediction.  No spin.  No correction.
//   The VCLOCK tick IS the time.
//
//   DWT information is available in the diagnostic block for clients
//   that need it (e.g., cross-checking DWT prediction accuracy).
//
// Two arming modes:
//
//   timepop_arm (delay in nanoseconds):
//     Deadline = vclock_count() + ns_to_ticks(delay_ns).
//     Supports recurring timers.
//     Callbacks fire in scheduled context via timepop_dispatch().
//
//   timepop_arm_ns (target GNSS nanosecond + DWT):
//     Caller owns the target computation.  The QTimer deadline is
//     derived from the delay.  One-shot only.
//     Optionally fires the callback in ISR context (isr_callback=true).
//
// Callback interface:
//
//   Every callback receives:
//     timepop_ctx_t*  — authoritative event identity and GNSS time
//     timepop_diag_t* — nullable diagnostic block (DWT, anchor, spin)
//     void*           — user data
//
// ============================================================================

// ============================================================================
// Timer handle
// ============================================================================

typedef uint32_t timepop_handle_t;

static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Timer context — authoritative event truth
//
// All fields are derived from the VCLOCK domain.  fire_gnss_ns is
// computed from fire_vclock_raw via GNSS arithmetic — not from DWT.
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;

  uint32_t         fire_vclock_raw; // qtimer1_read_32() at ISR entry
  uint32_t         deadline;        // target QTimer1 10 MHz tick

  int32_t          fire_ns;         // lateness: (fire_vclock_raw - deadline) * 100

  // GNSS nanosecond at the fire moment.
  // Computed from VCLOCK arithmetic when anchor is valid.
  // Falls back to DWT projection when anchor is not yet established.
  int64_t          fire_gnss_ns;
} timepop_ctx_t;

// ============================================================================
// Diagnostic block — optional, nullable
//
// Provides DWT capture, anchor snapshot, and spin diagnostics for
// clients that need cross-domain analysis.  Populated for ISR callbacks
// and timed slots.  Null for ASAP dispatch.
// ============================================================================

typedef struct timepop_diag_t {
  // DWT at ISR entry — raw, before any spin.
  uint32_t  dwt_at_isr_entry;

  // DWT after spin landing (or same as entry if no spin).
  uint32_t  dwt_at_fire;

  // DWT prediction for this slot's deadline.
  uint32_t  predicted_dwt;
  bool      prediction_valid;

  // Spin outcome.
  int32_t   spin_error_cycles;   // dwt_at_fire - predicted_dwt

  // Anchor snapshot at ISR entry.
  uint32_t  anchor_pps_count;
  uint32_t  anchor_qtimer_at_pps;
  uint32_t  anchor_dwt_at_pps;
  uint32_t  anchor_dwt_cycles_per_s;
  bool      anchor_valid;
} timepop_diag_t;

// ============================================================================
// Callback signature
// ============================================================================

typedef void (*timepop_callback_t)(
  timepop_ctx_t*  ctx,
  timepop_diag_t* diag,       // nullable — null for ASAP dispatch
  void*           user_data
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
// isr_callback:
//   If false (default), the callback fires in scheduled context via
//   timepop_dispatch(), like all other timers.
//
//   If true, the callback fires immediately in ISR context after the
//   DWT spin lands.  ISR callbacks must be fast and must not call
//   timepop_arm or any function that acquires the slot lock.
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
  const char*         name,
  bool                isr_callback = false
);

// ============================================================================
// Cancel
// ============================================================================

bool timepop_cancel(timepop_handle_t handle);

uint32_t timepop_cancel_by_name(const char* name);

// ============================================================================
// Dispatch (scheduled context — all non-ISR callbacks fire here)
// ============================================================================

void timepop_dispatch(void);

// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void);