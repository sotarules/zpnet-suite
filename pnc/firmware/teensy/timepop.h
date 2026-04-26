#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — Public Interface
// ============================================================================
//
// TimePop is ZPNet's scheduler.  It owns no hardware: process_interrupt is
// the sole owner of QTimer1/QTimer3 registers.  TimePop has two timing
// modes:
//
//   TICK mode (default, foreground, 1 kHz):
//     timepop_arm / timepop_arm_at / timepop_arm_from_anchor /
//     timepop_arm_asap / timepop_arm_alap
//
//     The user callback runs in foreground callback context.  Timed slots
//     are scanned from TimePop's 1 kHz TICK subscriber.  Resolution is one
//     TICK (normally 1 ms), which is exactly right for normal process
//     plumbing: transport pumps, event publishing, CPU usage, watchdogs,
//     DAC pacing, PPS relay off, and anything that may allocate payloads,
//     publish messages, print debug, or call foreground APIs.
//
//   SPINDRY mode (opt-in, precision, ISR context):
//     timepop_arm_at_spindry / timepop_arm_from_anchor_spindry
//
//     The user callback runs in ISR context, on the QTimer1 stack, at the
//     requested deadline modulo SpinDry approach and DWT spin behavior.
//     TimePop arms process_interrupt's QTimer1 CH2 schedule-fire lane to
//     fire SPINDRY_APPROACH_NS before the deadline; on entry, it spins on
//     ARM_DWT_CYCCNT until the target is reached, then invokes the user
//     callback synchronously.
//
//     Use only for tiny hardware-stimulus callbacks.  The callback MUST be
//     brief, MUST NOT block, and MUST NOT call foreground APIs.
//
// All normal foreground timed services intentionally use TICK mode.  CH2
// precision scheduling is reserved for explicit SpinDry calls.
//
// ============================================================================

typedef uint32_t timepop_handle_t;
static constexpr timepop_handle_t TIMEPOP_INVALID_HANDLE = 0;

// ============================================================================
// SpinDry approach window
// ============================================================================
//
// Time between the CH2 hardware-compare fire (approach ISR entry) and
// the SpinDry target deadline.  This is the "runway" — the spin loop's
// budget to absorb ISR entry latency, potential PPS preemption, and
// spin-loop iteration overhead.

static constexpr uint64_t SPINDRY_APPROACH_NS = 5000ULL;

// ============================================================================
// Callback context (TICK foreground or SpinDry — same shape)
// ============================================================================

typedef struct timepop_ctx_t {
  timepop_handle_t handle;
  uint32_t fire_dwt_cyccnt;       // event-coordinate DWT at the firing edge
  uint32_t fire_counter32;        // synthetic VCLOCK counter at the firing edge
  int64_t  fire_gnss_ns;          // GNSS ns at the firing edge
  int64_t  target_gnss_ns;        // requested deadline (absolute paths)
  int64_t  fire_gnss_error_ns;    // fire_gnss_ns − target_gnss_ns
} timepop_ctx_t;

// ============================================================================
// SpinDry diagnostics
// ============================================================================
//
// Populated by TimePop for SpinDry fires.  For TICK/foreground fires,
// is_spindry is false and the remaining fields are zero.

typedef struct timepop_diag_t {
  bool     is_spindry;
  bool     preempted;
  bool     deadline_overshot;
  uint32_t approach_isr_dwt;
  uint32_t approach_target_dwt;
  uint32_t target_dwt;
  uint32_t spin_landed_dwt;
  int32_t  spin_error_cycles;
  uint32_t preempt_cycles;
} timepop_diag_t;

typedef void (*timepop_callback_t)(
  timepop_ctx_t*  ctx,
  timepop_diag_t* diag,
  void*           user_data
);

// ============================================================================
// Foreground / TICK arming API
// ============================================================================
//
// These arms produce TICK-mode slots.  User callbacks run in foreground
// callback context from the 1 kHz TICK scan or from timepop_dispatch()
// for ASAP/ALAP work.

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

// ============================================================================
// SpinDry arming API (single-shot, ISR-context delivery)
// ============================================================================
//
// SpinDry arms produce precision slots whose user callback runs IN ISR
// CONTEXT at the requested deadline.  Both functions return
// TIMEPOP_INVALID_HANDLE if the bridge is not valid or the target is not
// usable.

timepop_handle_t timepop_arm_at_spindry(
  int64_t             target_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

timepop_handle_t timepop_arm_from_anchor_spindry(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
);

// ============================================================================
// Cancellation and inspection
// ============================================================================

bool     timepop_cancel        (timepop_handle_t handle);
uint32_t timepop_cancel_by_name(const char* name);
void     timepop_dispatch      (void);
uint32_t timepop_active_count  (void);

// ============================================================================
// Synthetic-counter zero handler
// ============================================================================
//
// Called by alpha after interrupt_synthetic_counters_zero() to re-anchor
// every active slot to the new counter timeline.
//
// After this call:
//   • TICK recurring slots: pending_anchor=true, delay_ticks=period_ticks;
//     they re-anchor on the next TICK and resume their normal cadence.
//   • TICK absolute slots: deadline recomputed from target_gnss_ns if the
//     bridge is valid; otherwise they fall back to pending_anchor.
//   • TICK one-shots: pending_anchor=true and fire on the new timeline.
//   • SpinDry slots: deadline/target_dwt recomputed; invalid bridge
//     deactivates the slot.
//
// After re-anchoring, only the SpinDry/precision head is recomputed and
// CH2 is re-armed if needed.
void timepop_handle_synthetic_counter_zero(void);
