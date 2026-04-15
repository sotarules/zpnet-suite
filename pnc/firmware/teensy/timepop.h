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
//      If recurring=true, TimePop will phase-lock the recurrence to the
//      current lawful GNSS/VCLOCK anchor when one is available.
//
//   2. Scheduled-context ASAP dispatch
//        timepop_arm_asap(...)
//
//   3. Scheduled-context ALAP dispatch
//        timepop_arm_alap(...)
//
//   4. Absolute GNSS scheduling
//        timepop_arm_at(target_gnss_ns, ...)
//
//   5. Anchor-relative GNSS scheduling
//        timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//
//   6. Caller-owned exact target scheduling
//        timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//
// TimePop owns:
//   • timed slot scheduling
//   • deferred ASAP/ALAP scheduled-context dispatch
//   • next-deadline selection
//   • CH2 compare arming
//   • absolute recurring timer series
//   • Spin-Dry early-wake + deterministic landing for VCLOCK scheduling
//   • timer diagnostics
//   • always-on internal VCLOCK monitor
//
// ============================================================================

typedef uint32_t timepop_handle_t;
static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Callback context — authoritative fire facts
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t fire_vclock_raw;
  uint32_t deadline;
  int32_t  fire_gnss_error_ns;
  int64_t  fire_gnss_ns;
} timepop_ctx_t;

// ============================================================================
// Optional diagnostics
// ============================================================================
//
// Populated for timed callbacks and ISR callbacks.
// Null for ASAP/ALAP scheduled-context dispatch.
//
// The authoritative result is still timepop_ctx_t. These diagnostics expose
// Spin-Dry / landing facts that are useful for instrumentation and later
// TIMEBASE_FRAGMENT analysis.
//

typedef struct timepop_diag_t {
  uint32_t dwt_at_isr_entry;
  uint32_t dwt_at_fire;

  uint32_t predicted_dwt;
  bool     prediction_valid;

  bool     spin_dry_used;
  uint32_t wake_target_dwt;
  int32_t  wake_error_cycles;

  int32_t  spin_error_cycles;

  uint32_t anchor_pps_count;
  uint32_t anchor_qtimer_at_pps;
  uint32_t anchor_dwt_at_pps;
  uint32_t anchor_dwt_cycles_per_s;
  bool     anchor_valid;
} timepop_diag_t;

typedef void (*timepop_callback_t)(
  timepop_ctx_t*  ctx,
  timepop_diag_t* diag,
  void*           user_data
);

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_asap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_alap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_at(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_from_anchor(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback = false
);

bool timepop_cancel(timepop_handle_t handle);
uint32_t timepop_cancel_by_name(const char* name);
void timepop_dispatch(void);
uint32_t timepop_active_count(void);
