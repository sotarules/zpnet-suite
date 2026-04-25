#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — Public Interface
// ============================================================================
//
// TimePop is ZPNet's GNSS-nanosecond scheduler.  It is foreground-driven
// from process_interrupt's TICK subscription (every cadence, ~1 ms).
// On every TICK, TimePop scans its slot list and fires any expired
// callbacks in foreground context.
//
// Resolution is 1 ms — scheduling honor is "fire at or after the
// requested deadline, on the next TICK boundary."  Callers requesting
// sub-ms scheduling will get up to 1 ms of late-fire jitter.  This is
// adequate for all current ZPNet consumers.  A future scheduled-fire
// extension may reintroduce sub-ms precision via process_interrupt.
//
// Scheduling modes:
//   timepop_arm(delay_gnss_ns, ...)            — relative ("from now")
//   timepop_arm_at(target_gnss_ns, ...)         — absolute GNSS ns
//   timepop_arm_from_anchor(anchor, offset, ...) — anchor-relative
//   timepop_arm_asap(...)                       — fire on next dispatch
//   timepop_arm_alap(...)                       — fire after asap phase
//
// All "now" semantics use time_gnss_ns_now() — the gear-locked answer
// derived from alpha's slot.  No counter peeks.  No live reads.
// ============================================================================

typedef uint32_t timepop_handle_t;
static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// Callback context
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t fire_dwt_cyccnt;       // event-coordinate DWT at the firing TICK
  uint32_t fire_counter32;        // synthetic VCLOCK counter at the firing TICK
  int64_t  fire_gnss_ns;          // GNSS ns at the firing TICK
  int64_t  target_gnss_ns;        // requested deadline (absolute paths)
  int64_t  fire_gnss_error_ns;    // fire_gnss_ns − target_gnss_ns
} timepop_ctx_t;

// ============================================================================
// Optional diagnostics (unused under tick-driven model — preserved for
// callers that pass a nullptr-tolerant pointer)
// ============================================================================

typedef struct timepop_diag_t {
  uint32_t reserved;
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

bool timepop_cancel(timepop_handle_t handle);
uint32_t timepop_cancel_by_name(const char* name);
void timepop_dispatch(void);
uint32_t timepop_active_count(void);

// ============================================================================
// Synthetic-counter zero handler
// ============================================================================
//
// Called by alpha after interrupt_synthetic_counters_zero() to re-anchor
// every active slot to the new counter timeline.  Without this, tick-based
// recurring slots (e.g. transport's RX/TX pumps) keep their pre-zero
// deadlines and stay frozen until the synthetic counter wraps back around
// to the old deadline value — which can take tens of seconds to minutes
// depending on how long the system was running before the zero.
//
// After this call:
//   • Tick recurring slots:   pending_anchor=true, delay_ticks=period_ticks.
//                             They re-anchor on the next TICK and resume
//                             firing at their normal cadence.
//   • Absolute slots:         deadline recomputed from current alpha slot
//                             via target_gnss_ns.  If the bridge is not
//                             yet authoritative (sequence==0), the slot
//                             falls back to pending_anchor with delay=0
//                             so it fires on the next TICK.
//   • Tick one-shot slots:    set pending_anchor=true.  Will fire after
//                             delay_ticks on the new timeline.
void timepop_handle_synthetic_counter_zero(void);