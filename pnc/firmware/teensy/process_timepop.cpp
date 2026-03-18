// ============================================================================
// process_timepop.cpp — TimePop v5.0 (Continuous DWT + GNSS Validation)
// ============================================================================
//
// The time experience in ZPNet is the GNSS clock.
//
// v5.0: Continuous DWT prediction and GNSS-now validation.
//
//   Every standard timer dispatch now produces two residual measurements:
//
//   1. DWT Prediction Residual:
//      At arm time, each slot computes a predicted DWT_CYCCNT for its
//      deadline using the time.h anchor.  When the ISR fires,
//      the very first instruction captures the actual DWT_CYCCNT.
//      The residual (actual - predicted) feeds into a global Welford
//      accumulator.  This directly measures how well we predict the
//      DWT cycle count at interrupt time.
//
//   2. GNSS-now Residual:
//      At ISR fire time, we know the exact GNSS nanosecond from the
//      VCLOCK position: (pps-1)*1e9 + (fire_gpt2 - gpt2_at_pps) * 100.
//      We also compute the GNSS nanosecond by feeding the captured
//      DWT_CYCCNT into time_dwt_to_gnss_ns().  The residual between
//      these two values feeds into a second Welford accumulator.
//      This is the end-to-end test: can we derive GNSS time from a
//      DWT cycle count with < 3ns stddev?
//
//   Both accumulators are global (not per-slot) and accumulate across
//   all standard timer dispatches — thousands per second.
//
//   New command: DIAG — dumps the full Welford state for both
//   accumulators plus recent sample forensics.
//
//   The standard REPORT now includes predicted_dwt per timer slot.
//
// v4.3: Comprehensive forensic diagnostics.
// v4.2: Caller-owns-target for nano-precise timers.
// v4.1: NS_TEST full forensics.
// v4: DWT nano-spin ISR delivery.
// v3.1: INTERP_TEST Path 2 epoch fix.
// v3: TEST and INTERP_TEST use time.h.  Campaign-independent.
//
// ============================================================================

#include "timepop.h"
#include "process_timepop.h"
#include "process_clocks_internal.h"

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
// Constants
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint64_t NS_PER_TICK = 100ULL;
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;
static constexpr uint32_t MIN_DELAY_TICKS = 2;
static constexpr uint32_t NANO_EARLY_TICKS = 100;         // 10 us early arrival
static constexpr uint32_t NANO_SPIN_MAX_CYCLES = 11000;   // ~10.9 us timeout

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
  bool                nano_precise;

  timepop_handle_t    handle;
  uint32_t            deadline;
  uint32_t            fire_gpt2;
  uint32_t            period_ticks;
  uint64_t            period_ns;

  int64_t             target_gnss_ns;
  uint32_t            target_dwt;
  int64_t             fire_gnss_ns;
  bool                nano_timeout;

  uint32_t            fire_dwt_cyccnt;

  // ── v5.0: time.h anchor snapshot at ISR fire ──
  uint32_t            anchor_dwt_at_pps;
  uint32_t            anchor_dwt_cycles_per_s;
  uint32_t            anchor_gpt2_at_pps;
  uint32_t            anchor_pps_count;
  bool                anchor_valid;

  // ── v5.0: DWT prediction ──
  uint32_t            predicted_dwt;
  bool                prediction_valid;

  // ── v5.0: Deferred Welford ──
  bool                nano_callback_fired;
  uint32_t            fire_pps_count;

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
static volatile uint32_t nano_spin_count    = 0;
static volatile uint32_t nano_timeout_count = 0;

// ── v4.3: Slot pressure ──
static volatile uint32_t diag_slots_high_water  = 0;
static volatile uint32_t diag_arm_failures      = 0;

// ── v4.3: ASAP tracking ──
static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

// ── v4.3: Dispatch health ──
static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

// ── v5.0: Continuous validation Welford accumulators ──
static welford_t welford_dwt_prediction = {};
static welford_t welford_gnss_now       = {};

// ── v5.0: Dispatch-level sample counts ──
static volatile uint32_t diag_prediction_samples = 0;
static volatile uint32_t diag_prediction_skipped = 0;

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
// GPT2 helpers
// ============================================================================

static inline uint32_t gpt2_count(void) { return GPT2_CNT; }

static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

static inline bool deadline_sooner(uint32_t a, uint32_t b, uint32_t now) {
  return (a - now) < (b - now);
}

static inline uint32_t ns_to_ticks(uint64_t ns) {
  uint64_t ticks = ns / NS_PER_TICK;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  return (uint32_t)ticks;
}

// ============================================================================
// DWT prediction helper
//
// Given a GPT2 deadline and a time.h anchor snapshot, predict the
// DWT_CYCCNT value that will be observed when GPT2_CNT reaches
// the deadline.
//
// The math:
//   gpt2_elapsed = deadline - anchor.gpt2_at_pps   (GPT2 ticks since PPS)
//   ns_elapsed   = gpt2_elapsed * 100              (each GPT2 tick = 100ns)
//   dwt_elapsed  = ns_elapsed * dwt_cycles_per_s / 1,000,000,000
//   predicted    = anchor.dwt_at_pps + dwt_elapsed
//
// Returns 0 with valid=false if anchor is not available.
// ============================================================================

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) return 0;
  if (snap.dwt_cycles_per_s == 0) return 0;

  const uint32_t gpt2_elapsed = deadline - snap.gpt2_at_pps;
  const uint64_t ns_elapsed = (uint64_t)gpt2_elapsed * 100ULL;

  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;

  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

// ============================================================================
// OCR1 management
// ============================================================================

static void ocr1_set(uint32_t value) {
  GPT2_OCR1 = value;
  GPT2_SR   = GPT_SR_OF1;
  GPT2_IR  |= GPT_IR_OF1IE;
}

static void ocr1_disable(void) {
  GPT2_IR &= ~GPT_IR_OF1IE;
  GPT2_SR  = GPT_SR_OF1;
}

static void reload_ocr(void) {
  uint32_t now = gpt2_count();
  uint32_t nearest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!found || deadline_sooner(slots[i].deadline, nearest, now)) {
      nearest = slots[i].deadline;
      found = true;
    }
  }

  if (found) {
    if (deadline_expired(nearest, now)) nearest = now + MIN_DELAY_TICKS;
    ocr1_set(nearest);
  } else {
    ocr1_disable();
  }
}

// ============================================================================
// ISR helper — snapshot pre-read anchor into slot at fire time
// ============================================================================

static inline void slot_capture(
  timepop_slot_t& slot,
  uint32_t fire_gpt2,
  uint32_t fire_dwt_cyccnt,
  const time_anchor_snapshot_t& snap
) {
  slot.fire_gpt2               = fire_gpt2;
  slot.fire_dwt_cyccnt         = fire_dwt_cyccnt;
  slot.anchor_dwt_at_pps       = snap.dwt_at_pps;
  slot.anchor_dwt_cycles_per_s = snap.dwt_cycles_per_s;
  slot.anchor_gpt2_at_pps      = snap.gpt2_at_pps;
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
  ctx.fire_gpt2       = slot.fire_gpt2;
  ctx.deadline        = slot.deadline;
  ctx.fire_ns         = (int32_t)(slot.fire_gpt2 - slot.deadline) * 100;
  ctx.fire_gnss_ns    = slot.fire_gnss_ns;
  ctx.nano_precise    = slot.nano_precise;
  ctx.nano_timeout    = slot.nano_timeout;
  ctx.fire_dwt_cyccnt = slot.fire_dwt_cyccnt;
}

// ============================================================================
// GPT2 Output Compare ISR
// ============================================================================

static void gpt2_compare_isr(void) {

  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  const uint32_t dwt_now = dwt_at_gpt2_compare(dwt_raw);
  const uint32_t now     = GPT2_CNT;

  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  GPT2_SR = GPT_SR_OF1;
  isr_fire_count++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    // ── Nano-precise: spin to exact DWT target, fire callback in ISR ──

    if (slots[i].nano_precise) {
      const uint32_t target_dwt = slots[i].target_dwt;
      uint32_t spin_start = ARM_DWT_CYCCNT;
      bool timed_out = false;

      while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
        if ((ARM_DWT_CYCCNT - spin_start) > NANO_SPIN_MAX_CYCLES) {
          timed_out = true;
          break;
        }
      }

      const uint32_t landed_dwt = ARM_DWT_CYCCNT;
      nano_spin_count++;

      slots[i].nano_timeout = timed_out;
      if (timed_out) {
        nano_timeout_count++;
        slots[i].fire_gnss_ns = -1;
      } else {
        slots[i].fire_gnss_ns = time_dwt_to_gnss_ns(landed_dwt);
      }

      slot_capture(slots[i], now, landed_dwt, anchor);

      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);
      slots[i].callback(&ctx, slots[i].user_data);

      slots[i].nano_callback_fired = true;
      slots[i].expired = true;
      expired_count++;
      continue;
    }

    // ── Standard: snapshot and defer to dispatch ──

    slot_capture(slots[i], now, dwt_now, anchor);
    slots[i].expired = true;
    expired_count++;
  }

  reload_ocr();
  timepop_pending = true;
}

// ============================================================================
// Initialization
// ============================================================================

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};

  welford_dwt_prediction.reset();
  welford_gnss_now.reset();

  GPT2_IR &= ~(GPT_IR_OF1IE | GPT_IR_OF2IE | GPT_IR_OF3IE);
  GPT2_SR  = GPT_SR_OF1 | GPT_SR_OF2 | GPT_SR_OF3;

  attachInterruptVector(IRQ_GPT2, gpt2_compare_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, 16);
  NVIC_ENABLE_IRQ(IRQ_GPT2);
}

// ============================================================================
// Arm — standard (100 ns resolution, scheduled context)
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
    slots[i].nano_precise = false;
    slots[i].handle       = h;
    slots[i].period_ns    = delay_ns;
    slots[i].period_ticks = ticks;
    slots[i].callback     = callback;
    slots[i].user_data    = user_data;
    slots[i].name         = name;
    slots[i].fire_gnss_ns = -1;

    if (is_asap) {
      slots[i].deadline = 0;
      slots[i].prediction_valid = false;
      slots[i].predicted_dwt    = 0;
      timepop_pending = true;
      diag_asap_armed++;
      diag_asap_last_armed_dwt  = ARM_DWT_CYCCNT;
      diag_asap_last_armed_name = name;
    } else {
      slots[i].deadline = gpt2_count() + ticks;

      // ── v5.0: Compute DWT prediction at arm time ──
      slots[i].predicted_dwt = predict_dwt_at_deadline(
        slots[i].deadline, slots[i].prediction_valid);

      reload_ocr();
    }

    update_slot_high_water();

    interrupts();
    return h;
  }

  // All slots full
  diag_arm_failures++;
  if (is_asap) diag_asap_arm_failures++;

  interrupts();
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Arm — nano-precise (~5 ns resolution, ISR context)
//
// Caller owns the target computation.  No internal time_gnss_ns_now()
// or time_gnss_ns_to_dwt() — the target goes straight into the slot.
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
  uint32_t gpt2_deadline;

  if (delay_ticks > NANO_EARLY_TICKS) {
    gpt2_deadline = gpt2_count() + delay_ticks - NANO_EARLY_TICKS;
  } else {
    gpt2_deadline = gpt2_count() + MIN_DELAY_TICKS;
  }

  noInterrupts();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].expired        = false;
    slots[i].recurring      = false;
    slots[i].nano_precise   = true;
    slots[i].handle         = h;
    slots[i].period_ns      = (uint64_t)delay_ns;
    slots[i].period_ticks   = 0;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].target_dwt     = target_dwt;
    slots[i].fire_gnss_ns   = -1;
    slots[i].nano_timeout   = false;
    slots[i].deadline       = gpt2_deadline;

    // Nano-precise slots use target_dwt as the prediction.
    slots[i].predicted_dwt    = target_dwt;
    slots[i].prediction_valid = true;

    update_slot_high_water();

    reload_ocr();
    interrupts();
    return h;
  }

  diag_arm_failures++;

  interrupts();
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Cancel by handle
// ============================================================================

bool timepop_cancel(timepop_handle_t handle) {
  if (handle == TIMEPOP_INVALID_HANDLE) return false;
  noInterrupts();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].handle == handle) {
      slots[i].active  = false;
      slots[i].expired = false;
      reload_ocr();
      interrupts();
      return true;
    }
  }
  interrupts();
  return false;
}

// ============================================================================
// Cancel by name
// ============================================================================

uint32_t timepop_cancel_by_name(const char* name) {
  if (!name) return 0;
  uint32_t cancelled = 0;
  noInterrupts();
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active && slots[i].name &&
        strcmp(slots[i].name, name) == 0) {
      slots[i].active  = false;
      slots[i].expired = false;
      cancelled++;
    }
  }
  if (cancelled > 0) reload_ocr();
  interrupts();
  return cancelled;
}

// ============================================================================
// Deferred event emission for nano-precise tests
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
// Slot-level Welford update — shared by all dispatch cases
//
// Given a slot with a captured DWT and fragment data, computes:
//   1. DWT prediction residual (captured vs predicted)
//   2. GNSS-now residual (time_dwt_to_gnss_ns vs VCLOCK ground truth)
// and feeds both into the global Welford accumulators.
//
// Both sides of the GNSS residual use the time.h coordinate system:
//   local GNSS ns = (pps_count - 1) * 1e9 + ns_into_second
// where pps_count comes from time_pps_count() and ns_into_second
// comes from the VCLOCK (GPT2) position relative to the PPS anchor.
// ============================================================================

static void slot_welford_update(const timepop_slot_t& slot) {

  if (!slot.prediction_valid || !slot.anchor_valid ||
      slot.anchor_dwt_cycles_per_s == 0) {
    diag_prediction_skipped++;
    return;
  }

  const uint32_t pps = slot.anchor_pps_count;
  if (pps < 2) {
    diag_prediction_skipped++;
    return;
  }

  // DWT residual: captured DWT vs predicted DWT
  const int32_t dwt_residual =
    (int32_t)(slot.fire_dwt_cyccnt - slot.predicted_dwt);
  welford_dwt_prediction.update((int64_t)dwt_residual);

  // GNSS residual: DWT-derived GNSS ns vs VCLOCK-derived GNSS ns
  //
  // Both sides use the SAME anchor captured in the ISR.
  // This eliminates PPS-boundary skew between ISR and dispatch.
  //
  // VCLOCK path (ground truth):
  //   gpt2_since_pps = fire_gpt2 - anchor_gpt2_at_pps
  //   vclock_gnss_ns = (pps - 1) * 1e9 + gpt2_since_pps * 100
  //
  // DWT path (test):
  //   dwt_elapsed = fire_dwt_cyccnt - anchor_dwt_at_pps
  //   dwt_gnss_ns = (pps - 1) * 1e9 + dwt_elapsed * 1e9 / dwt_cycles_per_s

  const int64_t epoch_ns = (int64_t)(pps - 1) * (int64_t)1000000000LL;

  // VCLOCK ground truth
  const uint32_t gpt2_since_pps = slot.fire_gpt2 - slot.anchor_gpt2_at_pps;
  const int64_t vclock_gnss_ns = epoch_ns + (int64_t)((uint64_t)gpt2_since_pps * 100ULL);

  // DWT interpolation (same math as time_dwt_to_gnss_ns, but using slot anchor)
  const uint32_t dwt_elapsed = slot.fire_dwt_cyccnt - slot.anchor_dwt_at_pps;
  const uint64_t dwt_ns_into_second =
    ((uint64_t)dwt_elapsed * 1000000000ULL + (uint64_t)slot.anchor_dwt_cycles_per_s / 2)
    / (uint64_t)slot.anchor_dwt_cycles_per_s;
  const int64_t dwt_gnss_ns = epoch_ns + (int64_t)dwt_ns_into_second;

  const int64_t gnss_residual = dwt_gnss_ns - vclock_gnss_ns;
  welford_gnss_now.update(gnss_residual);
  diag_prediction_samples++;
}

// ============================================================================
// Dispatch (scheduled context)
//
// v5.0: Processes ALL expired slots in three monolithic cases.
//
// Each case is self-contained: callback, Welford, lifecycle.
// No shared conditional branches.  Read any one case top-to-bottom.
//
//   ASAP:         fire callback, skip Welford, deactivate.
//   Nano-precise: callback already fired in ISR, Welford, deactivate.
//   Standard:     fire callback, Welford, re-arm or deactivate.
// ============================================================================

void timepop_dispatch(void) {

  emit_deferred_ns_test();

  if (!timepop_pending) return;
  timepop_pending = false;

  diag_dispatch_calls++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;

    slots[i].expired = false;

    // ================================================================
    // Case 1: ASAP — one-shot, no deadline, no prediction
    // ================================================================

    if (!slots[i].nano_precise &&
        slots[i].period_ns == 0 && !slots[i].recurring) {

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
      noInterrupts();
      reload_ocr();
      interrupts();
      continue;
    }

    // ================================================================
    // Case 2: Nano-precise — callback already fired in ISR
    // ================================================================

    if (slots[i].nano_precise) {

      if (!slots[i].nano_timeout) {
        slot_welford_update(slots[i]);
      } else {
        diag_prediction_skipped++;
      }

      slots[i].active = false;
      noInterrupts();
      reload_ocr();
      interrupts();
      continue;
    }

    // ================================================================
    // Case 3: Standard — fire callback, Welford, re-arm or deactivate
    // ================================================================

    // ── Callback ──

    timepop_ctx_t ctx;
    slot_build_ctx(slots[i], ctx);

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, slots[i].user_data);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;

    // ── Welford: DWT prediction + GNSS-now ──

    slot_welford_update(slots[i]);

    // ── Lifecycle: re-arm recurring or deactivate one-shot ──

    if (slots[i].recurring) {
      if (slots[i].period_ticks == 0) {
        slots[i].active = false;
      } else {
        slots[i].deadline += slots[i].period_ticks;
        uint32_t now = gpt2_count();
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].deadline = now + slots[i].period_ticks;
        }

        slots[i].predicted_dwt = predict_dwt_at_deadline(
          slots[i].deadline, slots[i].prediction_valid);

        noInterrupts();
        reload_ocr();
        interrupts();
      }
    } else {
      slots[i].active = false;
      noInterrupts();
      reload_ocr();
      interrupts();
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
// TEST command — standard timer accuracy via time.h
// ============================================================================

struct test_context_t {
  int64_t  arm_gnss_ns;
  uint64_t requested_ns;
  uint32_t arm_pps_count;
};

static test_context_t test_ctx = {};
static volatile bool  test_in_flight = false;

static void test_timer_callback(timepop_ctx_t* ctx, void*) {
  uint32_t cb_gpt2 = GPT2_CNT;
  int64_t end_gnss_ns = time_gnss_ns_now();

  int64_t measured_ns = -1;
  if (test_ctx.arm_gnss_ns >= 0 && end_gnss_ns >= 0)
    measured_ns = end_gnss_ns - test_ctx.arm_gnss_ns;

  int64_t error_ns = -1;
  if (measured_ns >= 0)
    error_ns = measured_ns - (int64_t)test_ctx.requested_ns;

  int32_t dispatch_ns = (int32_t)(cb_gpt2 - ctx->fire_gpt2) * 100;
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
// NS_TEST — nano-precise timer accuracy test (full forensics)
// ============================================================================

struct ns_test_context_t {
  int64_t  arm_gnss_ns;
  uint64_t requested_ns;
  int64_t  target_gnss_ns;
  uint32_t target_dwt;
  uint32_t arm_dwt;
  uint32_t arm_gpt2;
};

static ns_test_context_t ns_test_ctx = {};
static volatile bool     ns_test_in_flight = false;

static void ns_test_timer_callback(timepop_ctx_t* ctx, void*) {
  int64_t expected_ns = ns_test_ctx.target_gnss_ns;
  int64_t error_ns = (ctx->fire_gnss_ns >= 0)
    ? ctx->fire_gnss_ns - expected_ns
    : -1;

  uint32_t dwt_elapsed = ctx->fire_dwt_cyccnt - ns_test_ctx.arm_dwt;
  uint32_t gpt2_elapsed = ctx->fire_gpt2 - ns_test_ctx.arm_gpt2;

  int32_t spin_error = (int32_t)(ctx->fire_dwt_cyccnt - ns_test_ctx.target_dwt);

  Payload ev;
  ev.add("err",       error_ns);
  ev.add("expected",  expected_ns);
  ev.add("actual",    ctx->fire_gnss_ns);
  ev.add("spin_err",  spin_error);
  ev.add("timeout",   ctx->nano_timeout);
  ev.add("dwt_elapsed",  dwt_elapsed);
  ev.add("gpt2_elapsed", gpt2_elapsed);
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
  uint32_t arm_gpt2 = GPT2_CNT;
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
  ns_test_ctx.arm_gpt2       = arm_gpt2;

  timepop_handle_t h = timepop_arm_ns(
    target_gnss_ns, target_dwt,
    ns_test_timer_callback, nullptr, "ns-test"
  );
  if (h == TIMEPOP_INVALID_HANDLE) {
    resp.add("error", "arm_ns failed");
    return resp;
  }

  ns_test_in_flight = true;
  resp.add("status", "armed");
  resp.add("req",    ns_val);
  return resp;
}

// ============================================================================
// INTERP_TEST — dual-path validation (unchanged from v3.1, pending removal)
// ============================================================================

struct interp_test_ctx_t {
  uint32_t tests_remaining;
  uint64_t delay_ns;
  int64_t  arm_time_ns;
  uint32_t arm_gpt2;
  int64_t  w_n;   double w_mean, w_m2;
  int64_t  err_min, err_max;
  int32_t  outside_100ns;
  int64_t  t_n;   double t_mean, t_m2;
  int64_t  t_err_min, t_err_max;
};

static interp_test_ctx_t itest_ctx = {};
static volatile bool     itest_in_flight = false;

static void interp_test_callback(timepop_ctx_t* ctx, void*);

static void interp_test_arm_next(void) {
  itest_ctx.arm_time_ns = time_gnss_ns_now();
  itest_ctx.arm_gpt2    = GPT2_CNT;
  timepop_handle_t h = timepop_arm(
    itest_ctx.delay_ns, false, interp_test_callback, nullptr, "itest");
  if (h == TIMEPOP_INVALID_HANDLE) itest_in_flight = false;
}

static void interp_test_callback(timepop_ctx_t* ctx, void*) {

  // Path 1 disabled — fragment fields removed from timepop_ctx_t.
  // Pending full removal of INTERP_TEST.
  bool path1_valid = false;
  int64_t path1_error_ns = 0;
  uint64_t vclock_ns_into_second = 0;

  bool path2_valid = false;
  int64_t path2_error_ns = 0;
  {
    const int64_t fire_time_ns = time_gnss_ns_now();
    const uint32_t fire_gpt2   = GPT2_CNT;
    if (itest_ctx.arm_time_ns >= 0 && fire_time_ns >= 0) {
      const int64_t time_delta_ns   = fire_time_ns - itest_ctx.arm_time_ns;
      const int64_t vclock_delta_ns = (int64_t)((uint32_t)(fire_gpt2 - itest_ctx.arm_gpt2)) * 100LL;
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
  out.add("expired",           expired_count);
  out.add("gpt2_cnt",          gpt2_count());
  out.add("test_in_flight",    test_in_flight);
  out.add("ns_test_in_flight", ns_test_in_flight);
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());
  out.add("nano_spins",        nano_spin_count);
  out.add("nano_timeouts",     nano_timeout_count);

  // ── v4.3: Slot pressure ──
  out.add("slots_active_now",  timepop_active_count());
  out.add("slots_high_water",  diag_slots_high_water);
  out.add("slots_max",         (uint32_t)MAX_SLOTS);
  out.add("arm_failures",      diag_arm_failures);

  // ── v4.3: ASAP tracking ──
  out.add("asap_armed",             diag_asap_armed);
  out.add("asap_dispatched",        diag_asap_dispatched);
  out.add("asap_arm_failures",      diag_asap_arm_failures);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name", (const char*)(diag_asap_last_armed_name ? diag_asap_last_armed_name : ""));

  // ── v4.3: Dispatch health ──
  out.add("dispatch_calls",     diag_dispatch_calls);
  out.add("dispatch_callbacks", diag_dispatch_callbacks);

  // ── v5.0: Validation summary (top-level) ──
  out.add("dwt_pred_n",       (int32_t)welford_dwt_prediction.n);
  out.add("dwt_pred_stddev",  welford_dwt_prediction.stddev());
  out.add("gnss_now_n",       (int32_t)welford_gnss_now.n);
  out.add("gnss_now_stddev",  welford_gnss_now.stddev());

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
    entry.add("nano",      slots[i].nano_precise);
    entry.add("expired",   slots[i].expired);

    // ── v5.0: Per-slot prediction ──
    entry.add("predicted_dwt",    slots[i].predicted_dwt);
    entry.add("prediction_valid", slots[i].prediction_valid);

    timers.add(entry);
  }
  out.add_array("timers", timers);
  return out;
}

// ============================================================================
// DIAG command — full Welford statistics for DWT prediction + GNSS-now
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

  // Current time.h anchor for context
  time_anchor_snapshot_t snap = time_anchor_snapshot();
  out.add("anchor_ok",               snap.ok);
  out.add("anchor_valid",            snap.valid);
  out.add("anchor_dwt_at_pps",       snap.dwt_at_pps);
  out.add("anchor_dwt_cycles_per_s", snap.dwt_cycles_per_s);
  out.add("anchor_gpt2_at_pps",      snap.gpt2_at_pps);
  out.add("anchor_pps_count",        snap.pps_count);

  return out;
}

// ============================================================================
// DIAG_RESET command — zero the Welford accumulators
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
// VCLOCK_TEST — QTimer1 interval measurement via GPT2-scheduled callback
//
// Temporary diagnostic to characterize QTimer1 (GNSS VCLOCK) counting
// accuracy and interrupt-readability independent of the scheduling engine.
//
// Protocol:
//   1. Spin-wait for the next PPS edge (isr_snap_gnss changes).
//      This aligns the arm point to the VCLOCK/PPS boundary — the same
//      moment the system considers "time zero" for the new second.
//   2. Capture qtimer1_read_32() immediately after the PPS edge.
//   3. Arm a standard timepop_arm() timer for the requested duration.
//   4. In the callback, capture qtimer1_read_32() again.
//   5. Compute:
//        vclock_raw_delta  = end_raw - start_raw   (dual-edge counts)
//        vclock_ticks      = vclock_raw_delta / 2  (10 MHz ticks)
//        expected_ticks    = requested_ns / 100     (100 ns per tick)
//        residual_ticks    = vclock_ticks - expected_ticks
//        residual_ns       = residual_ticks * 100
//
// The callback also captures DWT elapsed and time_gnss_ns_now() elapsed
// for cross-reference against the VCLOCK measurement.
//
// Command: { "ns": <uint64> }
//   Minimum: 1,000,000 (1 ms) to give GPT2 scheduling time.
//   Maximum: 999,000,000 (999 ms) to stay within one PPS second.
//
// Event: TIMEPOP_VCLOCK_TEST
// ============================================================================

struct vclock_test_context_t {
  uint32_t arm_vclock_raw;   // qtimer1_read_32() at arm point (post-PPS)
  uint32_t arm_dwt;          // DWT_CYCCNT at arm point
  int64_t  arm_gnss_ns;      // time_gnss_ns_now() at arm point
  uint64_t requested_ns;     // requested duration
  uint32_t arm_pps_count;    // time_pps_count() at arm
};

static vclock_test_context_t vclock_test_ctx = {};
static volatile bool         vclock_test_in_flight = false;

static void vclock_test_callback(timepop_ctx_t* ctx, void*) {

  // Capture end state immediately
  const uint32_t end_vclock_raw = qtimer1_read_32();
  const uint32_t end_dwt        = ARM_DWT_CYCCNT;
  const int64_t  end_gnss_ns    = time_gnss_ns_now();

  // VCLOCK measurement
  const uint32_t vclock_raw_delta  = end_vclock_raw - vclock_test_ctx.arm_vclock_raw;
  const uint32_t vclock_ticks      = vclock_raw_delta / 2;
  const uint64_t expected_ticks    = vclock_test_ctx.requested_ns / 100ULL;
  const int32_t  residual_ticks    = (int32_t)vclock_ticks - (int32_t)expected_ticks;
  const int32_t  residual_ns       = residual_ticks * 100;

  // DWT cross-reference
  const uint32_t dwt_elapsed = end_dwt - vclock_test_ctx.arm_dwt;

  // GNSS cross-reference
  int64_t gnss_elapsed_ns = -1;
  if (vclock_test_ctx.arm_gnss_ns >= 0 && end_gnss_ns >= 0)
    gnss_elapsed_ns = end_gnss_ns - vclock_test_ctx.arm_gnss_ns;

  Payload ev;
  ev.add("req_ns",           vclock_test_ctx.requested_ns);
  ev.add("vclock_raw_delta", (uint64_t)vclock_raw_delta);  // dual-edge count
  ev.add("vclock_ticks",     (uint64_t)vclock_ticks);      // /2 = 10 MHz ticks
  ev.add("expected_ticks",   (uint64_t)expected_ticks);
  ev.add("residual_ticks",   (int32_t)residual_ticks);
  ev.add("residual_ns",      (int32_t)residual_ns);
  ev.add("dwt_elapsed",      (uint64_t)dwt_elapsed);
  ev.add("gnss_elapsed_ns",  gnss_elapsed_ns);
  ev.add("arm_pps",          vclock_test_ctx.arm_pps_count);
  ev.add("end_pps",          time_pps_count());
  ev.add("fire_ns",          ctx->fire_ns);    // GPT2 lateness vs deadline

  enqueueEvent("TIMEPOP_VCLOCK_TEST", ev);
  vclock_test_in_flight = false;
}

static Payload cmd_vclock_test(const Payload& args) {
  Payload resp;

  if (vclock_test_in_flight) {
    resp.add("error", "test already in flight");
    return resp;
  }
  if (!time_valid()) {
    resp.add("error", "time not valid (need PPS lock)");
    return resp;
  }

  uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val < 1000000ULL) {
    resp.add("error", "ns must be >= 1000000 (1 ms)");
    return resp;
  }
  if (ns_val > 999000000ULL) {
    resp.add("error", "ns must be <= 999000000 (999 ms)");
    return resp;
  }

  // PPS just fired. Capture VCLOCK position immediately.
  const uint32_t arm_vclock_raw = qtimer1_read_32();
  const uint32_t arm_dwt        = ARM_DWT_CYCCNT;
  const int64_t  arm_gnss_ns    = time_gnss_ns_now();

  vclock_test_ctx = {};
  vclock_test_ctx.arm_vclock_raw = arm_vclock_raw;
  vclock_test_ctx.arm_dwt        = arm_dwt;
  vclock_test_ctx.arm_gnss_ns    = arm_gnss_ns;
  vclock_test_ctx.requested_ns   = ns_val;
  vclock_test_ctx.arm_pps_count  = time_pps_count();

  timepop_handle_t h = timepop_arm(
    ns_val, false, vclock_test_callback, nullptr, "vclock-test"
  );
  if (h == TIMEPOP_INVALID_HANDLE) {
    resp.add("error", "timepop_arm failed");
    return resp;
  }

  vclock_test_in_flight = true;
  resp.add("status",    "armed");
  resp.add("req_ns",    ns_val);
  resp.add("arm_vclock_raw", (uint64_t)arm_vclock_raw);
  return resp;
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
  { "VCLOCK_TEST",  cmd_vclock_test  },
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