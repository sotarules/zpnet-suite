// ============================================================================
// process_timepop.cpp — TimePop v8.0 (Priority Queue, 10 MHz Phase-Locked)
// ============================================================================
//
// v8.0: Priority queue with a single hardware comparator.
//
//   Architecture:
//
//     All four QTimer1 channels now count at 10 MHz (CM=1, rising edge
//     only).  CH0+CH1 form the passive 32-bit GNSS counter.  CH2 is
//     the scheduling comparator.  CH3 is unallocated.
//
//     Every timer — recurring or one-shot — is nano-precise.  There is
//     no "standard" vs "nano" distinction.  Every callback receives a
//     predicted DWT_CYCCNT at its deadline moment.
//
//     Scheduling:
//       After each ISR (and after each arm/cancel), the scheduler scans
//       all active slots, finds the soonest deadline, and arms CH2's
//       COMP1 with the low 16 bits of that deadline.  CH2 free-runs at
//       10 MHz (no LENGTH reset), wrapping every 65,536 ticks = 6.5536 ms.
//
//       When CH2's compare fires, the ISR reads the full 32-bit CH0+CH1
//       counter and qualifies each expired slot against its 32-bit
//       deadline.  Phantom firings (16-bit alias matches where no 32-bit
//       deadline is actually due) are detected and dismissed — the ISR
//       simply rearms CH2 for the next deadline and exits.
//
//       A 1 ms ceiling ensures the system never goes silent: if the
//       next deadline is more than 10,000 ticks away (or no slots are
//       active), CH2 is armed 10,000 ticks ahead as a housekeeping
//       heartbeat.
//
//     DWT spin:
//       Every slot has a predicted_dwt computed at arm time.  When the
//       ISR fires within one QTimer tick (100 ns) of the deadline, it
//       DWT-spins to the predicted cycle count.  Maximum spin is ~100 ns
//       ≈ 100 DWT cycles — negligible on a 1008 MHz core.
//
//     Phase locking:
//       Deadlines are expressed in 10 MHz GNSS ticks — the native unit
//       of every clock domain in ZPNet.  A recurring 1 ms timer fires
//       at exact GNSS-aligned boundaries: PPS + 10000, PPS + 20000, ...
//       One tick = 100 ns = one GNSS cycle.  No translation.
//
//     ASAP timers:
//       delay_ns == 0 timers are marked immediately expired and
//       dispatched on the next timepop_dispatch() call in scheduled
//       context, bypassing the ISR entirely.
//
//   CH2 compare race avoidance:
//
//     After writing COMP1, the ISR reads CH2's CNTR.  If the counter
//     has already passed the compare value (i.e., (COMP1 - CNTR) as
//     unsigned 16-bit > 0x8000), the compare was missed.  In that case
//     the ISR handles expired slots immediately rather than waiting for
//     the next wrap.
//
//   Pin assignment:
//     Pin 10 (GPIO_B0_00, ALT1) — QTimer1 CH0+CH1 — passive 32-bit (10 MHz)
//     QTimer1 CH2              — dynamic compare (priority queue scheduler)
//     QTimer1 CH3              — unallocated
//
// v7.1: VCLOCK_TEST removed.
// v7.0: CH2 as periodic 1 ms heartbeat (superseded by v8.0).
// v5.0: Continuous DWT prediction and GNSS-now validation.
// v4.3: Comprehensive forensic diagnostics.
// v4: DWT nano-spin ISR delivery.
// v3: TEST and INTERP_TEST use time.h.  Campaign-independent.
//
// ============================================================================

#include "timepop.h"
#include "process_timepop.h"
#include "process_clocks_internal.h"

#include "publish.h"

#include "debug.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "time.h"
#include "isr_dwt_compensate.h"

#include <Arduino.h>
#include "imxrt.h"
#include <string.h>
#include <math.h>
#include <climits>

// ============================================================================
// Constants — 10 MHz domain
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;

// One QTimer tick at 10 MHz = 100 ns.
static constexpr uint64_t NS_PER_TICK = 100ULL;

// 32-bit unsigned deadline math: any (deadline - now) above this is "expired".
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;

// Minimum arm distance — prevents degenerate compare races.
static constexpr uint32_t MIN_DELAY_TICKS = 2;

// 1 ms ceiling heartbeat: 10,000 ticks at 10 MHz.
// If the next deadline is farther away (or no slots active), arm this far ahead.
static constexpr uint32_t HEARTBEAT_TICKS = 10000;

// DWT spin budget: 110 DWT cycles ≈ 109 ns at 1008 MHz.
// One QTimer tick is 100 ns ≈ ~101 DWT cycles, so 110 gives margin.
static constexpr uint32_t DWT_SPIN_MAX_CYCLES = 110;

// Staleness limit for DWT prediction: 15,000,000 ticks = 1.5 seconds at 10 MHz.
static constexpr uint32_t PREDICT_MAX_QTIMER_ELAPSED = 15000000U;

// ============================================================================
// Welford accumulator (ISR-safe: only written from dispatch context)
// ============================================================================

struct welford_t {
  int64_t  n;
  double   mean;
  double   m2;
  int64_t  min_val;
  int64_t  max_val;
  int32_t  outside_100ns;
  int64_t  last_residual;

  void reset(void) {
    n = 0; mean = 0.0; m2 = 0.0;
    min_val = INT64_MAX; max_val = INT64_MIN;
    outside_100ns = 0; last_residual = 0;
  }

  void update(int64_t residual) {
    n++;
    const double x = (double)residual;
    const double d1 = x - mean;
    mean += d1 / (double)n;
    m2 += (x - mean) * d1;
    if (residual < min_val) min_val = residual;
    if (residual > max_val) max_val = residual;
    if (residual > 100 || residual < -100) outside_100ns++;
    last_residual = residual;
  }

  double stddev(void) const {
    return (n >= 2) ? sqrt(m2 / (double)(n - 1)) : 0.0;
  }

  double stderr_val(void) const {
    return (n >= 2) ? stddev() / sqrt((double)n) : 0.0;
  }
};

// ============================================================================
// Slot
// ============================================================================

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;

  timepop_handle_t    handle;
  uint32_t            deadline;       // 32-bit QTimer1 10 MHz tick
  uint32_t            fire_vclock_raw;
  uint32_t            period_ticks;
  uint64_t            period_ns;

  int64_t             target_gnss_ns; // for arm_ns callers
  int64_t             fire_gnss_ns;

  uint32_t            fire_dwt_cyccnt;

  // ── time.h anchor snapshot at ISR fire ──
  uint32_t            anchor_dwt_at_pps;
  uint32_t            anchor_dwt_cycles_per_s;
  uint32_t            anchor_qtimer_at_pps;
  uint32_t            anchor_pps_count;
  bool                anchor_valid;

  // ── DWT prediction ──
  uint32_t            predicted_dwt;
  bool                prediction_valid;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
};

// ============================================================================
// State
// ============================================================================

static timepop_slot_t slots[MAX_SLOTS];
static volatile bool  timepop_pending = false;
static uint32_t       next_handle = 1;

// ============================================================================
// Diagnostics
// ============================================================================

static volatile uint32_t isr_fire_count     = 0;
static volatile uint32_t expired_count      = 0;
static volatile uint32_t phantom_count      = 0;

// ── Slot pressure ──
static volatile uint32_t diag_slots_high_water  = 0;
static volatile uint32_t diag_arm_failures      = 0;

// ── ASAP tracking ──
static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

// ── Dispatch health ──
static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

// ── Continuous validation Welford accumulators ──
static welford_t welford_dwt_prediction = {};
static welford_t welford_gnss_now       = {};

// ── Dispatch-level sample counts ──
static volatile uint32_t diag_prediction_samples = 0;
static volatile uint32_t diag_prediction_skipped = 0;

// ── v8.0: Scheduler diagnostics ──
static volatile uint32_t diag_isr_count          = 0;
static volatile uint32_t diag_rearm_count        = 0;
static volatile uint32_t diag_heartbeat_rearms   = 0;
static volatile uint32_t diag_race_recoveries    = 0;

// ============================================================================
// Forward declarations
// ============================================================================

static void qtimer1_irq_isr(void);
static void schedule_next(void);

// ============================================================================
// Slot counting helper (called with interrupts disabled)
// ============================================================================

static inline void update_slot_high_water(void) {
  uint32_t active = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) active++;
  }
  if (active > diag_slots_high_water) {
    diag_slots_high_water = active;
  }
}

// ============================================================================
// QTimer1 raw VCLOCK helpers — 10 MHz domain
// ============================================================================

static inline uint32_t vclock_count(void) { return qtimer1_read_32(); }

static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

static inline uint32_t ns_to_ticks(uint64_t ns) {
  uint64_t ticks = ns / NS_PER_TICK;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  return (uint32_t)ticks;
}

// ============================================================================
// DWT prediction helper — 10 MHz domain
// ============================================================================

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) return 0;
  if (snap.dwt_cycles_per_s == 0) return 0;

  const uint32_t qtimer_elapsed = deadline - snap.qtimer_at_pps;

  // Staleness guard: reject predictions spanning > 1.5 seconds.
  if (qtimer_elapsed > PREDICT_MAX_QTIMER_ELAPSED) return 0;

  // 10 MHz: 1 tick = 100 ns.
  const uint64_t ns_elapsed = (uint64_t)qtimer_elapsed * NS_PER_TICK;

  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;

  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

// ============================================================================
// CH2 compare management — dynamic priority queue scheduler
//
// CH2 counts at 10 MHz (CM=1, PCS=0, no LENGTH).  Its 16-bit counter
// free-runs in phase with CH0 (they count the same edges).  COMP1 is
// written to the low 16 bits of the next soonest deadline.  When CNTR
// matches COMP1, the ISR fires.
//
// 16-bit alias period: 65,536 ticks = 6.5536 ms at 10 MHz.
// Phantom firings are resolved by 32-bit qualification in the ISR.
//
// Race avoidance: after writing COMP1, read CNTR.  If the compare
// point has already passed, handle it immediately.
// ============================================================================

static inline void ch2_clear_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static inline void ch2_arm_compare(uint16_t target_low16) {
  ch2_clear_flag();
  IMXRT_TMR1.CH[2].COMP1  = target_low16;
  IMXRT_TMR1.CH[2].CMPLD1 = target_low16;
  ch2_clear_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

// ============================================================================
// schedule_next — scan all slots, arm CH2 for the soonest deadline.
//
// Called from:
//   - The ISR after processing expired slots
//   - timepop_arm / timepop_arm_ns after inserting a new slot
//   - timepop_cancel after removing a slot
//
// Must be called with interrupts disabled.
// ============================================================================

static void schedule_next(void) {
  const uint32_t now = vclock_count();
  uint32_t soonest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    // Already past due — mark pending and let dispatch handle it.
    if (deadline_expired(slots[i].deadline, now)) {
      slots[i].expired = true;
      expired_count++;
      timepop_pending = true;
      continue;
    }

    if (!found || ((slots[i].deadline - now) < (soonest - now))) {
      soonest = slots[i].deadline;
      found = true;
    }
  }

  uint32_t target;
  if (!found) {
    // No active non-expired slots — arm 1 ms heartbeat ceiling.
    target = now + HEARTBEAT_TICKS;
    diag_heartbeat_rearms++;
  } else {
    uint32_t distance = soonest - now;
    if (distance > HEARTBEAT_TICKS) {
      // Deadline is far away — arm ceiling heartbeat to stay alive.
      target = now + HEARTBEAT_TICKS;
      diag_heartbeat_rearms++;
    } else {
      target = soonest;
    }
  }

  ch2_arm_compare((uint16_t)(target & 0xFFFF));
  diag_rearm_count++;

  // ── Race check: did the counter already pass our compare point? ──
  const uint16_t cntr = IMXRT_TMR1.CH[2].CNTR;
  const uint16_t comp = (uint16_t)(target & 0xFFFF);
  const uint16_t distance_16 = comp - cntr;

  // If distance_16 > 0x8000, the counter has already passed the compare
  // value — the compare won't fire until the next 16-bit wrap (~6.5 ms).
  // Check if the 32-bit deadline is actually past due now.
  if (distance_16 > 0x8000u) {
    const uint32_t now2 = vclock_count();
    if (found && deadline_expired(soonest, now2)) {
      // The soonest deadline has passed.  Flag it and let the main
      // loop handle it via timepop_dispatch().  The ISR will fire
      // on the next 16-bit wrap (~6.5 ms) or sooner if schedule_next
      // is called again — but timepop_dispatch() will process it first.
      for (uint32_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots[i].active || slots[i].expired) continue;
        if (deadline_expired(slots[i].deadline, now2)) {
          slots[i].expired = true;
          expired_count++;
          timepop_pending = true;
        }
      }
      diag_race_recoveries++;
    }
  }
}

// ============================================================================
// ISR helper — snapshot pre-read anchor into slot at fire time
// ============================================================================

static inline void slot_capture(
  timepop_slot_t& slot,
  uint32_t fire_vclock_raw,
  uint32_t fire_dwt_cyccnt,
  const time_anchor_snapshot_t& snap
) {
  slot.fire_vclock_raw         = fire_vclock_raw;
  slot.fire_dwt_cyccnt         = fire_dwt_cyccnt;
  slot.anchor_dwt_at_pps       = snap.dwt_at_pps;
  slot.anchor_dwt_cycles_per_s = snap.dwt_cycles_per_s;
  slot.anchor_qtimer_at_pps    = snap.qtimer_at_pps;
  slot.anchor_pps_count        = snap.pps_count;
  slot.anchor_valid            = snap.ok && snap.valid;
}

// ============================================================================
// ISR helper — build ctx from slot
// ============================================================================

static inline void slot_build_ctx(
  const timepop_slot_t& slot,
  timepop_ctx_t& ctx
) {
  ctx.handle          = slot.handle;
  ctx.fire_vclock_raw = slot.fire_vclock_raw;
  ctx.deadline        = slot.deadline;
  ctx.fire_ns         = (int32_t)(slot.fire_vclock_raw - slot.deadline) * (int32_t)NS_PER_TICK;
  ctx.fire_gnss_ns    = slot.fire_gnss_ns;
  ctx.nano_precise    = true;
  ctx.nano_timeout    = false;
  ctx.fire_dwt_cyccnt = slot.fire_dwt_cyccnt;
}

// ============================================================================
// QTimer1 CH2 ISR — priority queue scheduler
//
// Fires when CH2's 16-bit CNTR matches COMP1.  May be a real deadline
// or a phantom (16-bit alias).  The ISR qualifies against the full
// 32-bit counter and DWT-spins to the predicted cycle count for each
// expired slot.
// ============================================================================

static void qtimer1_ch2_isr(void) {

  const uint32_t dwt_entry = ARM_DWT_CYCCNT;
  const uint32_t now       = qtimer1_read_32();
  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  ch2_clear_flag();
  diag_isr_count++;

  bool any_expired = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    // ── DWT fire moment determination ──
    //
    // The compare event fired at the QTimer tick corresponding to
    // this slot's deadline.  The predicted_dwt is our best estimate
    // of the DWT_CYCCNT at that exact tick.
    //
    // Three cases:
    //
    //   1. predicted_dwt is in the future relative to dwt_entry:
    //      The ISR arrived early (rare, possible with cascaded slots).
    //      Spin forward to the target.  Maximum spin: ~100 DWT cycles.
    //
    //   2. predicted_dwt is in the past relative to dwt_entry:
    //      The ISR entry latency + processing overhead consumed the
    //      window.  The compare DID fire at the right QTimer tick.
    //      The predicted DWT at that tick is our best truth — use it
    //      directly.  No spin needed.
    //
    //   3. No valid prediction: use dwt_entry as-is.

    uint32_t landed_dwt;

    if (slots[i].prediction_valid) {
      const uint32_t target_dwt = slots[i].predicted_dwt;

      if ((int32_t)(target_dwt - dwt_entry) > 0) {
        // Case 1: target is still ahead — spin to it.
        const uint32_t spin_start = ARM_DWT_CYCCNT;

        while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
          if ((ARM_DWT_CYCCNT - spin_start) > DWT_SPIN_MAX_CYCLES) {
            break;
          }
        }

        landed_dwt = ARM_DWT_CYCCNT;
      } else {
        // Case 2: target already passed — the compare fired on time,
        // ISR overhead consumed the window.  Trust the prediction.
        landed_dwt = target_dwt;
      }
    } else {
      // Case 3: no prediction available.
      landed_dwt = dwt_entry;
    }

    slots[i].fire_gnss_ns = time_dwt_to_gnss_ns(landed_dwt);
    slot_capture(slots[i], now, landed_dwt, anchor);

    slots[i].expired = true;
    expired_count++;
    any_expired = true;
  }

  if (any_expired) {
    isr_fire_count++;
    timepop_pending = true;
  } else {
    phantom_count++;
  }

  // Rearm CH2 for the next soonest deadline (or heartbeat ceiling).
  schedule_next();
}

// ============================================================================
// Initialization
// ============================================================================

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};

  welford_dwt_prediction.reset();
  welford_gnss_now.reset();

  // ── QTimer1 CH2 — dynamic compare scheduler ──
  //
  // CH0+CH1 are already running as the passive 32-bit GNSS counter
  // (10 MHz, CM=1) in process_clocks.  CH2 is configured to count
  // the same 10 MHz input, free-running (no LENGTH reset):
  //
  //   CM=1    — count rising edges only (10 MHz)
  //   PCS=0   — primary clock source = Counter 0's input pin (GNSS 10 MHz)
  //   No LENGTH — CNTR free-runs, wraps at 65535 → 0 every 6.5536 ms
  //
  // COMP1 is dynamically set to the low 16 bits of the next deadline.
  // CMPLD1 auto-reloads COMP1 on match (keeps value stable).
  // TCF1EN enables the compare interrupt.
  //
  // Initial compare: 1 ms heartbeat ceiling.
  //
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;   // will be set by schedule_next()
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  attachInterruptVector(IRQ_QTIMER1, qtimer1_irq_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  // Arm initial 1 ms heartbeat.
  noInterrupts();
  schedule_next();
  interrupts();
}

// ============================================================================
// Arm — standard (delay in nanoseconds)
//
// All timers are nano-precise.  The deadline is placed in the 10 MHz
// QTimer domain, and a DWT prediction is computed at arm time.
// ============================================================================

timepop_handle_t timepop_arm(
  uint64_t            delay_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  bool is_asap = (delay_ns == 0);
  uint32_t ticks = is_asap ? 0 : ns_to_ticks(delay_ns);

  noInterrupts();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active       = true;
    slots[i].expired      = is_asap;
    slots[i].recurring    = recurring;
    slots[i].handle       = h;
    slots[i].period_ns    = delay_ns;
    slots[i].period_ticks = ticks;
    slots[i].callback     = callback;
    slots[i].user_data    = user_data;
    slots[i].name         = name;
    slots[i].fire_gnss_ns = -1;
    slots[i].target_gnss_ns = -1;

    if (is_asap) {
      slots[i].deadline = 0;
      slots[i].prediction_valid = false;
      slots[i].predicted_dwt    = 0;
      timepop_pending = true;
      diag_asap_armed++;
      diag_asap_last_armed_dwt  = ARM_DWT_CYCCNT;
      diag_asap_last_armed_name = name;
    } else {
      slots[i].deadline = vclock_count() + ticks;

      slots[i].predicted_dwt = predict_dwt_at_deadline(
        slots[i].deadline, slots[i].prediction_valid);

      // Compute target GNSS nanosecond from QTimer deadline via PPS anchor.
      if (slots[i].prediction_valid) {
        slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
      }

      // New slot may be sooner than current CH2 target — reschedule.
      schedule_next();
    }

    update_slot_high_water();

    interrupts();
    return h;
  }

  diag_arm_failures++;
  if (is_asap) diag_asap_arm_failures++;

  interrupts();
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Arm — nano-precise (target GNSS nanosecond + DWT)
//
// The caller specifies a target GNSS nanosecond and the corresponding
// DWT cycle count.  The QTimer deadline is derived from the delay.
// ============================================================================

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  int64_t now_ns = time_gnss_ns_now();
  if (now_ns < 0) return TIMEPOP_INVALID_HANDLE;

  int64_t delay_ns = target_gnss_ns - now_ns;
  if (delay_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t delay_ticks = ns_to_ticks((uint64_t)delay_ns);
  if (delay_ticks < MIN_DELAY_TICKS) delay_ticks = MIN_DELAY_TICKS;

  uint32_t vclock_deadline = vclock_count() + delay_ticks;

  noInterrupts();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].expired        = false;
    slots[i].recurring      = false;
    slots[i].handle         = h;
    slots[i].period_ns      = 0;
    slots[i].period_ticks   = 0;
    slots[i].deadline       = vclock_deadline;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;
    slots[i].fire_gnss_ns   = -1;

    slots[i].predicted_dwt    = target_dwt;
    slots[i].prediction_valid = true;

    // New slot may be sooner than current CH2 target — reschedule.
    schedule_next();

    update_slot_high_water();

    interrupts();
    return h;
  }

  diag_arm_failures++;

  interrupts();
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Cancel
// ============================================================================

bool timepop_cancel(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;

  noInterrupts();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      slots[i].active = false;
      schedule_next();
      interrupts();
      return true;
    }
  }
  interrupts();
  return false;
}

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name) return 0;

  uint32_t cancelled = 0;
  noInterrupts();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name && strcmp(slots[i].name, name) == 0) {
      slots[i].active = false;
      cancelled++;
    }
  }
  if (cancelled > 0) schedule_next();
  interrupts();
  return cancelled;
}

// ============================================================================
// NS_TEST — deferred event emission
// ============================================================================

static volatile bool ns_test_result_ready = false;
static Payload       ns_test_result_ev;

static void emit_deferred_ns_test(void) {
  if (!ns_test_result_ready) return;
  ns_test_result_ready = false;
  enqueueEvent("TIMEPOP_NS_TEST", ns_test_result_ev);
  ns_test_result_ev.clear();
}

// ============================================================================
// Slot-level Welford update — 10 MHz domain
// ============================================================================

static void slot_welford_update(const timepop_slot_t& slot) {

  // Gate: need valid prediction and a target GNSS nanosecond.
  if (!slot.prediction_valid || slot.target_gnss_ns <= 0) {
    diag_prediction_skipped++;
    return;
  }

  if (slot.fire_gnss_ns <= 0) {
    diag_prediction_skipped++;
    return;
  }

  // ── Primary residual: GNSS delivery accuracy ──
  //
  // How close did we land to the intended GNSS nanosecond?
  // This is the authoritative measure of TimePop precision.
  // Computed entirely in the GNSS domain — no DWT prediction
  // error, no ISR latency, no multi-slot sequential bias.
  const int64_t gnss_residual = slot.fire_gnss_ns - slot.target_gnss_ns;
  welford_dwt_prediction.update(gnss_residual);

  // ── Cross-check: DWT-interpolated ns vs QTimer-derived ns ──
  //
  // Validates that time_dwt_to_gnss_ns() and the QTimer position
  // agree at the fire moment.  Independent of scheduling accuracy.
  if (slot.anchor_valid && slot.anchor_dwt_cycles_per_s > 0 &&
      slot.anchor_pps_count >= 2) {

    const int64_t epoch_ns = (int64_t)(slot.anchor_pps_count - 1) * (int64_t)1000000000LL;

    const uint32_t qtimer_since_pps = slot.fire_vclock_raw - slot.anchor_qtimer_at_pps;
    const int64_t vclock_gnss_ns = epoch_ns + (int64_t)((uint64_t)qtimer_since_pps * NS_PER_TICK);

    const uint32_t dwt_elapsed = slot.fire_dwt_cyccnt - slot.anchor_dwt_at_pps;
    const uint64_t dwt_ns_into_second =
      ((uint64_t)dwt_elapsed * 1000000000ULL + (uint64_t)slot.anchor_dwt_cycles_per_s / 2)
      / (uint64_t)slot.anchor_dwt_cycles_per_s;
    const int64_t dwt_gnss_ns = epoch_ns + (int64_t)dwt_ns_into_second;

    const int64_t crosscheck_residual = dwt_gnss_ns - vclock_gnss_ns;
    welford_gnss_now.update(crosscheck_residual);
      }

  diag_prediction_samples++;
}

// ============================================================================
// Dispatch — scheduled context
//
// All callbacks fire here (not in ISR).  The ISR marks slots expired
// and captures fire state; dispatch invokes the actual callbacks.
// ============================================================================

void timepop_dispatch(void) {

  emit_deferred_ns_test();

  if (!timepop_pending) return;
  timepop_pending = false;

  diag_dispatch_calls++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;

    slots[i].expired = false;

    // ── ASAP ──
    if (slots[i].period_ns == 0 && !slots[i].recurring) {

      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);

      uint32_t start = ARM_DWT_CYCCNT;
      slots[i].callback(&ctx, slots[i].user_data);
      uint32_t end   = ARM_DWT_CYCCNT;

      cpu_usage_account_busy(end - start);
      diag_dispatch_callbacks++;
      diag_asap_dispatched++;
      diag_asap_last_dispatch_dwt = end;
      diag_prediction_skipped++;

      slots[i].active = false;
      continue;
    }

    // ── Timed (all are nano-precise) ──

    timepop_ctx_t ctx;
    slot_build_ctx(slots[i], ctx);

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, slots[i].user_data);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;

    slot_welford_update(slots[i]);

    if (slots[i].recurring) {
      if (slots[i].period_ticks == 0) {
        slots[i].active = false;
      } else {
        slots[i].deadline += slots[i].period_ticks;
        uint32_t now = vclock_count();
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].deadline = now + slots[i].period_ticks;
        }

        slots[i].predicted_dwt = predict_dwt_at_deadline(
                  slots[i].deadline, slots[i].prediction_valid);

        if (slots[i].prediction_valid) {
          slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
        }

        slots[i].expired = false;

        // Rearmed slot may affect the schedule.
        noInterrupts();
        schedule_next();
        interrupts();
      }
    } else {
      slots[i].active = false;
    }
  }
}

// ============================================================================
// Introspection
// ============================================================================

uint32_t timepop_active_count(void) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) n++;
  }
  return n;
}

// ============================================================================
// TEST command — standard timer accuracy
// ============================================================================

struct test_context_t {
  int64_t  arm_gnss_ns;
  uint64_t requested_ns;
  uint32_t arm_pps_count;
};

static test_context_t test_ctx = {};
static volatile bool  test_in_flight = false;

static void test_timer_callback(timepop_ctx_t* ctx, void*) {
  uint32_t cb_vclock = qtimer1_read_32();
  int64_t end_gnss_ns = time_gnss_ns_now();

  int64_t measured_ns = -1;
  if (test_ctx.arm_gnss_ns >= 0 && end_gnss_ns >= 0)
    measured_ns = end_gnss_ns - test_ctx.arm_gnss_ns;

  int64_t error_ns = -1;
  if (measured_ns >= 0)
    error_ns = measured_ns - (int64_t)test_ctx.requested_ns;

  int32_t dispatch_ns = (int32_t)(cb_vclock - ctx->fire_vclock_raw) * (int32_t)NS_PER_TICK;
  uint32_t end_pps = time_pps_count();

  Payload ev;
  ev.add("err",         error_ns);
  ev.add("meas",        measured_ns);
  ev.add("req",         test_ctx.requested_ns);
  ev.add("fire_ns",     ctx->fire_ns);
  ev.add("dispatch_ns", dispatch_ns);
  ev.add("arm_pps",     test_ctx.arm_pps_count);
  ev.add("end_pps",     end_pps);
  ev.add("pps_x",       (int32_t)(end_pps - test_ctx.arm_pps_count));
  ev.add("time_valid",  time_valid());

  enqueueEvent("TIMEPOP_TEST", ev);
  test_in_flight = false;
}

static Payload cmd_test(const Payload& args) {
  Payload resp;

  if (test_in_flight) { resp.add("error", "test already in flight"); return resp; }
  if (!args.has("ns")) { resp.add("error", "specify ns=<nanoseconds>"); return resp; }

  uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val == 0) { resp.add("error", "ns must be > 0"); return resp; }

  int64_t arm_gnss = time_gnss_ns_now();
  if (arm_gnss < 0) { resp.add("error", "time not valid (need PPS lock)"); return resp; }

  test_ctx = {};
  test_ctx.arm_gnss_ns   = arm_gnss;
  test_ctx.requested_ns  = ns_val;
  test_ctx.arm_pps_count = time_pps_count();

  timepop_handle_t h = timepop_arm(ns_val, false, test_timer_callback, nullptr, "test");
  if (h == TIMEPOP_INVALID_HANDLE) { resp.add("error", "arm failed"); return resp; }

  test_in_flight = true;
  resp.add("status", "armed");
  resp.add("req",    ns_val);
  return resp;
}

// ============================================================================
// NS_TEST — nano-precise timer accuracy test
// ============================================================================

struct ns_test_context_t {
  int64_t  arm_gnss_ns;
  uint64_t requested_ns;
  int64_t  target_gnss_ns;
  uint32_t target_dwt;
  uint32_t arm_dwt;
  uint32_t arm_vclock_raw;
};

static ns_test_context_t ns_test_ctx = {};
static volatile bool     ns_test_in_flight = false;

static void ns_test_timer_callback(timepop_ctx_t* ctx, void*) {
  int64_t expected_ns = ns_test_ctx.target_gnss_ns;
  int64_t error_ns = (ctx->fire_gnss_ns >= 0)
    ? ctx->fire_gnss_ns - expected_ns
    : -1;

  uint32_t dwt_elapsed = ctx->fire_dwt_cyccnt - ns_test_ctx.arm_dwt;
  uint32_t qtimer_elapsed = ctx->fire_vclock_raw - ns_test_ctx.arm_vclock_raw;

  int32_t spin_error = (int32_t)(ctx->fire_dwt_cyccnt - ns_test_ctx.target_dwt);

  Payload ev;
  ev.add("err",       error_ns);
  ev.add("expected",  expected_ns);
  ev.add("actual",    ctx->fire_gnss_ns);
  ev.add("spin_err",  spin_error);
  ev.add("timeout",   ctx->nano_timeout);
  ev.add("dwt_elapsed",  dwt_elapsed);
  ev.add("qtimer_elapsed", qtimer_elapsed);
  ev.add("req",       ns_test_ctx.requested_ns);
  ev.add("fire_ns",   ctx->fire_ns);

  ns_test_result_ev = ev;
  ns_test_result_ready = true;

  ns_test_in_flight = false;
}

static Payload cmd_ns_test(const Payload& args) {
  Payload resp;

  if (ns_test_in_flight) { resp.add("error", "test already in flight"); return resp; }
  if (!args.has("ns")) { resp.add("error", "specify ns=<nanoseconds>"); return resp; }

  uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val == 0) { resp.add("error", "ns must be > 0"); return resp; }
  if (!time_valid()) { resp.add("error", "time not valid"); return resp; }

  uint32_t arm_dwt  = ARM_DWT_CYCCNT;
  uint32_t arm_vclock_raw = qtimer1_read_32();
  int64_t  arm_gnss = time_gnss_ns_now();
  if (arm_gnss < 0) { resp.add("error", "time_gnss_ns_now failed"); return resp; }

  int64_t  target_gnss_ns = arm_gnss + (int64_t)ns_val;
  uint32_t target_dwt     = time_gnss_ns_to_dwt(target_gnss_ns);

  ns_test_ctx = {};
  ns_test_ctx.arm_gnss_ns    = arm_gnss;
  ns_test_ctx.requested_ns   = ns_val;
  ns_test_ctx.target_gnss_ns = target_gnss_ns;
  ns_test_ctx.target_dwt     = target_dwt;
  ns_test_ctx.arm_dwt        = arm_dwt;
  ns_test_ctx.arm_vclock_raw = arm_vclock_raw;

  timepop_handle_t h = timepop_arm_ns(
    target_gnss_ns, target_dwt,
    ns_test_timer_callback, nullptr, "ns-test"
  );
  if (h == TIMEPOP_INVALID_HANDLE) { resp.add("error", "arm_ns failed"); return resp; }

  ns_test_in_flight = true;
  resp.add("status", "armed");
  resp.add("req",    ns_val);
  resp.add("target_gnss_ns", target_gnss_ns);
  resp.add("target_dwt",     target_dwt);
  return resp;
}

// ============================================================================
// INTERP_TEST — dual-path interpolation validation
// ============================================================================

struct interp_test_context_t {
  uint32_t tests_remaining;
  uint64_t delay_ns;

  int64_t  arm_gnss_ns;
  int64_t  arm_time_ns;
  uint32_t arm_vclock_raw;

  int64_t  w_n;
  double   w_mean;
  double   w_m2;
  int64_t  err_min;
  int64_t  err_max;
  int32_t  outside_100ns;

  int64_t  t_n;
  double   t_mean;
  double   t_m2;
  int64_t  t_err_min;
  int64_t  t_err_max;
};

static interp_test_context_t itest_ctx = {};
static volatile bool         itest_in_flight = false;

static void interp_test_arm_next(void);

static void interp_test_callback(timepop_ctx_t* ctx, void*) {

  bool path1_valid = false;
  int64_t path1_error_ns = 0;
  uint64_t vclock_ns_into_second = 0;

  {
    time_anchor_snapshot_t snap = time_anchor_snapshot();
    if (snap.ok && snap.valid && snap.pps_count >= 2 && snap.dwt_cycles_per_s > 0) {
      const int64_t epoch_ns = (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL;

      const uint32_t vclock_since_pps = ctx->fire_vclock_raw - snap.qtimer_at_pps;
      vclock_ns_into_second = (uint64_t)vclock_since_pps * NS_PER_TICK;
      const int64_t vclock_gnss_ns = epoch_ns + (int64_t)vclock_ns_into_second;

      const uint32_t dwt_elapsed = ctx->fire_dwt_cyccnt - snap.dwt_at_pps;
      const uint64_t dwt_ns_into_second =
        ((uint64_t)dwt_elapsed * 1000000000ULL + (uint64_t)snap.dwt_cycles_per_s / 2)
        / (uint64_t)snap.dwt_cycles_per_s;
      const int64_t dwt_gnss_ns = epoch_ns + (int64_t)dwt_ns_into_second;

      path1_error_ns = dwt_gnss_ns - vclock_gnss_ns;
      path1_valid = true;
    }
  }

  bool path2_valid = false;
  int64_t path2_error_ns = 0;
  {
    const int64_t fire_time_ns = time_gnss_ns_now();
    const uint32_t fire_vclock_raw   = qtimer1_read_32();
    if (itest_ctx.arm_time_ns >= 0 && fire_time_ns >= 0) {
      const int64_t time_delta_ns   = fire_time_ns - itest_ctx.arm_time_ns;
      const int64_t vclock_delta_ns = (int64_t)((uint32_t)(fire_vclock_raw - itest_ctx.arm_vclock_raw)) * (int64_t)NS_PER_TICK;
      path2_error_ns = time_delta_ns - vclock_delta_ns;
      path2_valid = true;
    }
  }

  if (path1_valid) {
    itest_ctx.w_n++;
    const double x = (double)path1_error_ns, d1 = x - itest_ctx.w_mean;
    itest_ctx.w_mean += d1 / (double)itest_ctx.w_n;
    itest_ctx.w_m2 += (x - itest_ctx.w_mean) * d1;
    if (path1_error_ns < itest_ctx.err_min) itest_ctx.err_min = path1_error_ns;
    if (path1_error_ns > itest_ctx.err_max) itest_ctx.err_max = path1_error_ns;
    if (path1_error_ns > 100 || path1_error_ns < -100) itest_ctx.outside_100ns++;
  }

  if (path2_valid) {
    itest_ctx.t_n++;
    const double x = (double)path2_error_ns, d1 = x - itest_ctx.t_mean;
    itest_ctx.t_mean += d1 / (double)itest_ctx.t_n;
    itest_ctx.t_m2 += (x - itest_ctx.t_mean) * d1;
    if (path2_error_ns < itest_ctx.t_err_min) itest_ctx.t_err_min = path2_error_ns;
    if (path2_error_ns > itest_ctx.t_err_max) itest_ctx.t_err_max = path2_error_ns;
  }

  itest_ctx.tests_remaining--;
  if (itest_ctx.tests_remaining > 0) { interp_test_arm_next(); return; }

  const double w_stddev = (itest_ctx.w_n >= 2) ? sqrt(itest_ctx.w_m2 / (double)(itest_ctx.w_n - 1)) : 0.0;
  const double t_stddev = (itest_ctx.t_n >= 2) ? sqrt(itest_ctx.t_m2 / (double)(itest_ctx.t_n - 1)) : 0.0;

  Payload ev;
  ev.add("m_n",   (int32_t)itest_ctx.w_n);
  ev.add("m_mean", itest_ctx.w_mean);
  ev.add("m_sd",   w_stddev);
  ev.add("m_min",  (itest_ctx.w_n > 0) ? itest_ctx.err_min : (int64_t)0);
  ev.add("m_max",  (itest_ctx.w_n > 0) ? itest_ctx.err_max : (int64_t)0);
  ev.add("m_out",  itest_ctx.outside_100ns);
  ev.add("t_n",   (int32_t)itest_ctx.t_n);
  ev.add("t_mean", itest_ctx.t_mean);
  ev.add("t_sd",   t_stddev);
  ev.add("t_min",  (itest_ctx.t_n > 0) ? itest_ctx.t_err_min : (int64_t)0);
  ev.add("t_max",  (itest_ctx.t_n > 0) ? itest_ctx.t_err_max : (int64_t)0);
  ev.add("s0_p1",  path1_error_ns);
  ev.add("s0_p2",  path2_error_ns);
  ev.add("s0_pos", (double)vclock_ns_into_second / 1000000000.0);
  ev.add("s0_fns", ctx->fire_ns);

  enqueueEvent("TIMEPOP_INTERP_TEST", ev);
  itest_in_flight = false;
}

static void interp_test_arm_next(void) {
  itest_ctx.arm_gnss_ns    = time_gnss_ns_now();
  itest_ctx.arm_time_ns    = time_gnss_ns_now();
  itest_ctx.arm_vclock_raw = qtimer1_read_32();

  timepop_arm(itest_ctx.delay_ns, false, interp_test_callback, nullptr, "itest");
}

static Payload cmd_interp_test(const Payload& args) {
  Payload resp;

  if (itest_in_flight) { resp.add("error", "test already in flight"); return resp; }
  if (!time_valid()) { resp.add("error", "time not valid (need PPS lock)"); return resp; }

  const uint64_t delay_ns = args.getUInt64("delay_ns", 200000000ULL);
  const int32_t count_raw = args.getInt("count", 10);
  const int32_t count = (count_raw < 1) ? 1 : (count_raw > 100) ? 100 : count_raw;

  itest_ctx = {};
  itest_ctx.tests_remaining = count;
  itest_ctx.delay_ns        = delay_ns;
  itest_ctx.err_min         = INT64_MAX;
  itest_ctx.err_max         = INT64_MIN;
  itest_ctx.t_err_min       = INT64_MAX;
  itest_ctx.t_err_max       = INT64_MIN;

  itest_in_flight = true;
  interp_test_arm_next();

  resp.add("status", "armed");
  resp.add("count",  count);
  resp.add("delay_ns", delay_ns);
  return resp;
}

// ============================================================================
// REPORT command
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;

  out.add("isr_fires",         isr_fire_count);
  out.add("isr_count",         diag_isr_count);
  out.add("phantom_count",     phantom_count);
  out.add("rearm_count",       diag_rearm_count);
  out.add("heartbeat_rearms",  diag_heartbeat_rearms);
  out.add("race_recoveries",   diag_race_recoveries);
  out.add("expired",           expired_count);
  out.add("vclock_raw_now",    vclock_count());
  out.add("test_in_flight",    test_in_flight);
  out.add("ns_test_in_flight", ns_test_in_flight);
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());

  out.add("slots_active_now",  timepop_active_count());
  out.add("slots_high_water",  diag_slots_high_water);
  out.add("slots_max",         (uint32_t)MAX_SLOTS);
  out.add("arm_failures",      diag_arm_failures);

  out.add("asap_armed",             diag_asap_armed);
  out.add("asap_dispatched",        diag_asap_dispatched);
  out.add("asap_arm_failures",      diag_asap_arm_failures);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name", (const char*)(diag_asap_last_armed_name ?
    diag_asap_last_armed_name : ""));

  out.add("dispatch_calls",     diag_dispatch_calls);
  out.add("dispatch_callbacks", diag_dispatch_callbacks);

  out.add("dwt_pred_n",       (int32_t)welford_dwt_prediction.n);
  out.add("dwt_pred_stddev",  welford_dwt_prediction.stddev());
  out.add("gnss_now_n",       (int32_t)welford_gnss_now.n);
  out.add("gnss_now_stddev",  welford_gnss_now.stddev());

  // QTimer1 CH2 state
  out.add("qtmr1_ch2_cntr",   (uint32_t)IMXRT_TMR1.CH[2].CNTR);
  out.add("qtmr1_ch2_comp1",  (uint32_t)IMXRT_TMR1.CH[2].COMP1);
  out.add("qtmr1_ch2_csctrl", (uint32_t)IMXRT_TMR1.CH[2].CSCTRL);

  PayloadArray timers;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;
    Payload entry;
    entry.add("slot",      i);
    entry.add("handle",    slots[i].handle);
    entry.add("name",      slots[i].name ? slots[i].name : "");
    entry.add("period_ns", slots[i].period_ns);
    entry.add("ticks",     slots[i].period_ticks);
    entry.add("deadline",  slots[i].deadline);
    entry.add("recurring", slots[i].recurring);
    entry.add("expired",   slots[i].expired);
    entry.add("predicted_dwt",    slots[i].predicted_dwt);
    entry.add("prediction_valid", slots[i].prediction_valid);
    timers.add(entry);
  }
  out.add_array("timers", timers);
  return out;
}

// ============================================================================
// DIAG command
// ============================================================================

static void report_welford(Payload& out, const char* prefix, const welford_t& w) {
  char key[64];

  snprintf(key, sizeof(key), "%s_n", prefix);
  out.add(key, (int32_t)w.n);

  snprintf(key, sizeof(key), "%s_mean", prefix);
  out.add(key, w.mean);

  snprintf(key, sizeof(key), "%s_stddev", prefix);
  out.add(key, w.stddev());

  snprintf(key, sizeof(key), "%s_stderr", prefix);
  out.add(key, w.stderr_val());

  snprintf(key, sizeof(key), "%s_min", prefix);
  out.add(key, (w.n > 0) ? w.min_val : (int64_t)0);

  snprintf(key, sizeof(key), "%s_max", prefix);
  out.add(key, (w.n > 0) ? w.max_val : (int64_t)0);

  snprintf(key, sizeof(key), "%s_out100", prefix);
  out.add(key, w.outside_100ns);

  snprintf(key, sizeof(key), "%s_last", prefix);
  out.add(key, w.last_residual);
}

static Payload cmd_diag(const Payload&) {
  Payload out;

  report_welford(out, "dwt_pred",  welford_dwt_prediction);
  report_welford(out, "gnss_now",  welford_gnss_now);

  out.add("prediction_samples", diag_prediction_samples);
  out.add("prediction_skipped", diag_prediction_skipped);

  out.add("time_valid",     time_valid());
  out.add("time_pps_count", time_pps_count());

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  out.add("anchor_ok",               snap.ok);
  out.add("anchor_valid",            snap.valid);
  out.add("anchor_dwt_at_pps",       snap.dwt_at_pps);
  out.add("anchor_dwt_cycles_per_s", snap.dwt_cycles_per_s);
  out.add("anchor_qtimer_at_pps",    snap.qtimer_at_pps);
  out.add("anchor_pps_count",        snap.pps_count);

  return out;
}

// ============================================================================
// DIAG_RESET command
// ============================================================================

static Payload cmd_diag_reset(const Payload&) {
  welford_dwt_prediction.reset();
  welford_gnss_now.reset();
  diag_prediction_samples = 0;
  diag_prediction_skipped = 0;

  Payload out;
  out.add("status", "reset");
  return out;
}

// ============================================================================
// QTimer1 ISR — CH2 only
// ============================================================================

static void qtimer1_irq_isr(void) {
  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_isr();
  }
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT",      cmd_report      },
  { "DIAG",        cmd_diag        },
  { "DIAG_RESET",  cmd_diag_reset  },
  { "TEST",        cmd_test        },
  { "NS_TEST",     cmd_ns_test     },
  { "INTERP_TEST", cmd_interp_test },
  { nullptr,       nullptr }
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}