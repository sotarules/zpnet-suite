#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// timepop.h — Public Interface
// ============================================================================
//
// TimePop is ZPNet's GNSS-nanosecond scheduler.  It owns no hardware:
// process_interrupt is the sole owner of QTimer1/QTimer3 registers, and
// TimePop schedules its head-of-queue deadline via the public schedule-
// fire API (interrupt_schedule_fire_at).
//
// Two delivery modes:
//
//   FOREGROUND (default):
//     timepop_arm / timepop_arm_at / timepop_arm_from_anchor /
//     timepop_arm_asap / timepop_arm_alap
//
//     The user callback runs in foreground (loop()) context.  The hardware
//     compare fires at the deadline; TimePop's CH2 ISR queues the slot
//     for foreground dispatch via the asap mechanism.  Latency from
//     hardware fire to user callback is dominated by foreground loop
//     turnaround — typically tens of microseconds.
//
//     Use for: anything that calls back into foreground APIs, allocates
//     payloads, sends transport frames, prints debug, etc.  This is the
//     right mode for ~99% of consumers.
//
//   SPINDRY (opt-in, sub-microsecond):
//     timepop_arm_at_spindry / timepop_arm_from_anchor_spindry
//
//     The user callback runs in ISR context, on the QTimer1 stack, at
//     the EXACT requested deadline (modulo a few DWT cycles).  TimePop
//     arms the CH2 compare to fire SPINDRY_APPROACH_NS before the
//     deadline; on entry, it spins on ARM_DWT_CYCCNT until the target
//     is reached, then invokes the user callback synchronously.
//
//     Use for: stimulating hardware at a precise moment (LANTERN photon
//     loop trigger, TIMEPULSE mid-second anchoring witness pulse, etc.).
//     The user callback MUST be brief, MUST NOT block, MUST NOT call
//     foreground APIs.
//
//     SpinDry slots:
//       • Are SINGLE-SHOT.  Re-arm from inside the callback if you want
//         a recurring pattern.
//       • REQUIRE the bridge to be valid at arm time (reverse bridge:
//         time_gnss_ns_to_dwt must succeed).  Returns INVALID_HANDLE
//         otherwise.
//       • May be preempted by PPS GPIO (priority 0).  When that happens,
//         the spin loop pauses, PPS runs, spin resumes.  Two outcomes:
//           — PPS finishes within the runway: spin still lands on time.
//             diag.preempted = true, diag.deadline_overshot = false.
//           — PPS finishes past the deadline: spin lands late.
//             diag.preempted = true, diag.deadline_overshot = true.
//         Consumers that are sensitive to overshoot (LANTERN photon
//         capture, e.g.) should inspect diag.deadline_overshot in their
//         callback and skip the sample if true.
//
// All "now" semantics use time_gnss_ns_now() — the gear-locked answer
// derived from alpha's slot.  No counter peeks.  No live reads.
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
//
// Empirical tuning target (initial value, expected to be adjusted after
// flash-and-observe):
//
//   • ISR entry latency      : ~50 cycles ≈ 50 ns
//   • PPS ISR worst-case     : ~250 ns (single-pass; may chain on
//                              rebootstrap edge)
//   • Spin loop minimum      : ~10 cycles ≈ 10 ns
//   • Safety margin          : remainder
//
// 5 µs gives ~4.5 µs of margin above worst-case observed latencies.
// Reduce after empirical confirmation that approach landing is stable.

static constexpr uint64_t SPINDRY_APPROACH_NS = 5000ULL;

// ============================================================================
// Callback context (foreground or SpinDry — same shape)
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
// Populated by TimePop for SpinDry fires.  For foreground fires, only
// `is_spindry` is set (false), and the rest are zero.
//
// Field semantics:
//
//   is_spindry             True iff this fire is the SpinDry path.
//
//   approach_isr_dwt       Event-coordinate DWT captured at CH2 ISR entry
//                          (the approach ISR — i.e., the hardware-compare
//                          fire that opens the spin runway).
//
//   approach_target_dwt    Predicted DWT at which the approach ISR was
//                          expected to fire.  Computed at arm time as
//                          target_dwt - APPROACH_DWT_CYCLES.  Useful for
//                          diagnosing ISR latency drift over time.
//
//   target_dwt             Predicted DWT at which the user callback should
//                          fire.  Recomputed inside the approach ISR using
//                          the freshest bridge (alpha's PPS_VCLOCK slot)
//                          rather than the stale value cached at arm time.
//
//   spin_landed_dwt        Actual DWT at which the spin loop exited and
//                          the user callback was invoked.
//
//   spin_error_cycles      spin_landed_dwt - target_dwt (signed).
//                          • Near-zero in normal operation (a few cycles,
//                            limited by spin loop iteration time).
//                          • Positive value indicates the spin landed
//                            past the target — see deadline_overshot.
//
//   preempt_cycles         Excess time spent in the approach ISR beyond
//                          the expected approach duration.  Computed as
//                          (spin_landed_dwt - approach_isr_dwt) -
//                          (target_dwt - approach_isr_dwt).  In normal
//                          operation this is bounded by spin loop noise
//                          (a few cycles).  A spike (≥ ~100 cycles)
//                          indicates that a higher-priority ISR (PPS)
//                          ran during the approach.
//
//   preempted              True iff preempt_cycles exceeded a small
//                          tolerance (16 cycles).  Indicates PPS or
//                          another higher-priority ISR landed during the
//                          approach window.
//
//   deadline_overshot      True iff spin_error_cycles exceeded a small
//                          tolerance (16 cycles).  Indicates the target
//                          was missed — preemption pushed the spin past
//                          the deadline rather than being absorbed by
//                          the runway.  The user callback STILL FIRES
//                          (we don't skip on overshoot), but consumers
//                          sensitive to timing should inspect this flag
//                          and decide whether to use or discard the
//                          sample.
//
// Invariants:
//   • For foreground fires: all fields are zero except is_spindry=false.
//   • For SpinDry fires: is_spindry=true, other fields populated.
//   • preempted implies preempt_cycles > 16.
//   • deadline_overshot implies spin_error_cycles > 16.
//   • deadline_overshot implies preempted (PPS preemption is the only
//     known cause of overshoot under normal operating conditions).

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
// Foreground arming API
// ============================================================================
//
// All foreground arms produce slots whose user callback runs in foreground
// (loop()) context.  The hardware compare fires at the deadline; the CH2
// ISR pushes the slot into the asap queue, which is drained by the next
// loop() iteration via timepop_dispatch().

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
// SpinDry arms produce slots whose user callback runs IN ISR CONTEXT
// at the requested deadline (sub-microsecond precision).  See the file
// header for the contract.  Both functions return TIMEPOP_INVALID_HANDLE
// if the bridge is not yet valid (no PPS edge seen) or if the target is
// not in the future.

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
//   • SpinDry slots:          deadline recomputed.  If bridge is invalid,
//                             slot is deactivated (SpinDry requires a
//                             valid bridge to compute target_dwt).
//
// After re-anchoring, the head-of-queue is recomputed and CH2 is re-armed.
void timepop_handle_synthetic_counter_zero(void);