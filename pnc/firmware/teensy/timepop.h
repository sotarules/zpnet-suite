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
//   • QTimer1 CH0 is the passive VCLOCK counter at 10 MHz.
//   • One VCLOCK tick = 100 ns.
//   • QTimer1 CH2 is the dynamic compare scheduler.
//   • All public timing semantics are expressed in GNSS nanoseconds.
//
// SpinCatch:
//   TimePop can arm a pre-interrupt spin window at target - lead.  The spin
//   loop records raw DWT shadow facts until a different, higher-priority ISR
//   calls timepop_spincatch_finish_from_isr().  The user callback is then
//   invoked synchronously in that target ISR context with raw, unadjusted
//   SpinCatch diagnostics in timepop_diag_t.
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
  uint32_t fire_dwt_cyccnt;
  uint32_t deadline;
  int32_t  fire_gnss_error_ns;
  int64_t  fire_gnss_ns;
} timepop_ctx_t;

// ============================================================================
// Optional diagnostics
// ============================================================================

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

  // SpinCatch raw forensics. These fields are deliberately NOT
  // latency-adjusted.  They describe the target ISR's encounter with the
  // TimePop spin shadow.
  bool     spin_catch_used;
  bool     spin_catch_timeout;
  uint32_t spin_catch_handle;
  uint32_t spin_catch_target_deadline;
  int64_t  spin_catch_target_gnss_ns;
  uint64_t spin_catch_lead_ns;
  uint32_t spin_catch_lead_ticks;
  uint32_t spin_catch_timeout_cycles;
  uint32_t spin_catch_landing_dwt;
  uint32_t spin_catch_final_shadow_dwt;
  uint32_t spin_catch_shadow_seq;
  uint32_t spin_catch_approach_cycles;
  uint32_t spin_catch_isr_entry_dwt_raw;
  uint32_t spin_catch_isr_entry_latency_cycles;
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

timepop_handle_t timepop_arm_recurring_isr(
  uint64_t            period_gnss_ns,
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
