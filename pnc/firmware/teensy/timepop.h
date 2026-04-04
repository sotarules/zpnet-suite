#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — Public Interface
// ============================================================================
//
// TimePop is ZPNet's phase-locked timer subsystem.
//
// Core model:
//   • QTimer1 CH0+CH1 form a passive 32-bit VCLOCK counter at 10 MHz.
//   • One VCLOCK tick = 100 ns.
//   • QTimer1 CH2 is the dynamic compare scheduler.
//   • All public timing semantics are expressed in GNSS nanoseconds.
//
// Scheduling modes:
//
//   1. Relative scheduling
//        timepop_arm(delay_gnss_ns, ...)
//
//      Use this when the caller only cares about "fire after this delay."
//      Typical examples: serial RX/TX pacing, generic housekeeping timers.
//
//   2. Absolute GNSS scheduling
//        timepop_arm_at(target_gnss_ns, ...)
//
//      Use this when the caller already knows the exact GNSS nanosecond
//      boundary it wants. This avoids "whenever I happened to ask" timing
//      leakage in the caller.
//
//   3. Anchor-relative GNSS scheduling
//        timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//
//      Use this when the caller has a sacred recurrence anchor and wants the
//      target to be derived from that anchor plus a relative offset, rather
//      than from dispatch-time "now". This is the correct path for recurring
//      phase-locked work such as PPS pre-spin.
//
//   4. Caller-owned exact target scheduling
//        timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//
//      Use this when the caller owns both the GNSS target and the DWT target
//      and wants the full nano-precise path, optionally with ISR callback.
//
// TimePop owns:
//   • slot scheduling
//   • next-deadline selection
//   • CH2 compare arming
//   • deferred scheduled-context callback dispatch
//   • timer diagnostics
//
// TimePop does not own:
//   • interrupt-latency canonicalization
//   • last-mile pre-spin semantics in other subsystems
//   • provider-specific interrupt custody
//
// Those belong elsewhere.
//
// ============================================================================

// ============================================================================
// Handle
// ============================================================================

typedef uint32_t timepop_handle_t;

static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Callback context — authoritative fire facts
// ============================================================================
//
// These values describe when the timer actually fired.
//
// All public time values are expressed in the GNSS nanosecond domain.
// fire_vclock_raw and deadline expose the underlying 10 MHz scheduler facts.
//
// ============================================================================

typedef struct timepop_ctx_t {
  // Opaque timer identity returned from arm().
  timepop_handle_t handle;

  // 10 MHz VCLOCK value captured at fire time.
  uint32_t fire_vclock_raw;

  // 10 MHz VCLOCK deadline that this slot was armed against.
  uint32_t deadline;

  // Scheduler lateness expressed in GNSS nanoseconds:
  //   (fire_vclock_raw - deadline) * 100
  //
  // Negative means early, positive means late.
  int32_t fire_gnss_error_ns;

  // Absolute GNSS nanosecond at the fire moment.
  int64_t fire_gnss_ns;
} timepop_ctx_t;

// ============================================================================
// Optional diagnostics
// ============================================================================
//
// Populated for timed callbacks and ISR callbacks.
// Null for ASAP scheduled-context dispatch.
//
// This block is diagnostic only. It is not the authoritative timer result.
// The authoritative result is timepop_ctx_t.
//
// ============================================================================

typedef struct timepop_diag_t {
  // DWT at ISR entry — raw capture before any optional spin.
  uint32_t dwt_at_isr_entry;

  // DWT at actual callback fire point after optional spin landing.
  uint32_t dwt_at_fire;

  // Predicted DWT corresponding to the slot's target, if available.
  uint32_t predicted_dwt;
  bool     prediction_valid;

  // Spin landing error in cycles:
  //   dwt_at_fire - predicted_dwt
  int32_t  spin_error_cycles;

  // Anchor snapshot used for cross-domain interpretation.
  uint32_t anchor_pps_count;
  uint32_t anchor_qtimer_at_pps;
  uint32_t anchor_dwt_at_pps;
  uint32_t anchor_dwt_cycles_per_s;
  bool     anchor_valid;
} timepop_diag_t;

// ============================================================================
// Callback signature
// ============================================================================

typedef void (*timepop_callback_t)(
  timepop_ctx_t*  ctx,
  timepop_diag_t* diag,   // nullable
  void*           user_data
);

// ============================================================================
// Relative scheduling
// ============================================================================
//
// Arms a timer relative to the current GNSS/VCLOCK-aligned scheduler state.
//
// delay_gnss_ns:
//   Delay in GNSS nanoseconds from "now".
//
//   delay_gnss_ns == 0 means ASAP scheduled-context dispatch.
//   ASAP does not enter the timed ISR path.
//
// recurring:
//   If true, the timer rearms automatically using the same relative period.
//
// Returns TIMEPOP_INVALID_HANDLE on failure.
//
// ============================================================================

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Absolute GNSS scheduling
// ============================================================================
//
// Arms a timer for an exact GNSS nanosecond target.
//
// target_gnss_ns:
//   Absolute GNSS nanosecond at which the timer should fire.
//
// recurring:
//   Reserved for future use. In the current design, absolute timers are
//   expected to be one-shot unless the implementation explicitly documents
//   recurring absolute semantics.
//
// This API is intended for callers that already know the exact target
// boundary and want to avoid caller-side relative scheduling jitter.
//
// Returns TIMEPOP_INVALID_HANDLE on failure.
//
// ============================================================================

timepop_handle_t timepop_arm_at(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Anchor-relative GNSS scheduling
// ============================================================================
//
// Arms a timer using:
//
//   target_gnss_ns = anchor_gnss_ns + offset_gnss_ns
//
// This preserves sacred recurrence phase because the caller supplies the
// recurrence anchor explicitly instead of deriving the next target from "now".
//
// Typical use:
//   PPS prespin for the next second:
//     anchor_gnss_ns = last PPS edge
//     offset_gnss_ns = 1_000_000_000 - 5_000
//
// recurring:
//   Reserved for future use. Anchor-relative timers are currently expected to
//   be one-shot unless the implementation explicitly documents recurrence.
//
// Returns TIMEPOP_INVALID_HANDLE on failure.
//
// ============================================================================

timepop_handle_t timepop_arm_from_anchor(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Caller-owned exact target scheduling
// ============================================================================
//
// Arms a one-shot timer using caller-owned GNSS and DWT targets.
//
// This is the most exact path. The caller supplies both:
//
//   target_gnss_ns:
//     Exact GNSS nanosecond target
//
//   target_dwt:
//     DWT target corresponding to that GNSS target
//
// isr_callback:
//   false:
//     callback fires later in scheduled context
//
//   true:
//     callback fires immediately in ISR context after any spin landing
//
// ISR callbacks must be extremely small and must not perform operations
// that depend on scheduled-context-only mechanisms.
//
// Returns TIMEPOP_INVALID_HANDLE on failure.
//
// ============================================================================

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback = false
);

// ============================================================================
// Cancellation
// ============================================================================

bool timepop_cancel(timepop_handle_t handle);

uint32_t timepop_cancel_by_name(const char* name);

// ============================================================================
// Scheduled-context dispatch
// ============================================================================
//
// All non-ISR callbacks fire here.
// Call from the main runtime loop.
//
// ============================================================================

void timepop_dispatch(void);

// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void);