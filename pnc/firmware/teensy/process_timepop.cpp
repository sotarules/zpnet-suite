// ============================================================================
// process_timepop.cpp — TimePop v5.4 (QTimer1 CH3 VCLOCK_TEST — internal routing)
// ============================================================================
//
// v5.4: VCLOCK_TEST doorbell migrated to QTimer1 CH3 via internal PCS routing.
//
//   No external wire, no second pin, no IOMUX.  QTimer1 CH3 counts both
//   edges of Counter 0's input pin (GNSS 10 MHz on pin 10) using:
//     CM=2 (count rising AND falling edges of primary source)
//     PCS=0 (primary clock source = Counter 0's input pin)
//
//   This was validated in v5.3a by running QTimer1 CH3 and QTimer3 CH3
//   (on pin 15 via Kynar bridge) simultaneously.  Both counters tracked
//   identically, confirming the internal PCS routing delivers the same
//   GNSS signal without a physical connection.
//
//   IRQ_QTIMER1 is shared across all four channels.  The ISR checks
//   only CH3's TCF1 flag and returns immediately otherwise.  CH0 and
//   CH1 CSCTRL registers are NEVER touched by the ISR — they have no
//   TCF1EN set and generate no compare interrupts.
//
//   Pin assignment:
//     Pin 10 (GPIO_B0_00, ALT1) — QTimer1 CH0+CH1 — passive 32-bit counting
//     QTimer1 CH3              — compare doorbell (internal PCS from CH0 pin)
//
// v5.3a: QTimer3 CH3 on pin 15 + daisy chain (validated, then superseded).
// v5.3: QTimer3 CH3 on pin 15 (missing daisy chain — counter stayed at zero).
// v5.2: QTimer1 CH3 via SCS (wrong clock source — CM=1/SCS=0 got IPBus).
// v5.1: QTimer1 CH2 via XBAR (superseded — XBAR has no QTimer1 inputs).
// v5.0: Continuous DWT prediction and GNSS-now validation.
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
// Constants
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint64_t NS_PER_TICK = 50ULL;
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
  uint32_t            fire_vclock_raw;
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
  uint32_t            anchor_qtimer_at_pps;
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

static volatile uint32_t diag_vclock_candidate_irqs = 0;
static volatile uint32_t diag_vclock_early_rejects   = 0;

// ============================================================================
// VCLOCK_TEST forward declarations
// ============================================================================

static inline void vclock_test_disable_compare(void);
static void emit_deferred_vclock_test(void);
static void qtimer1_irq_isr(void);
static void qtimer1_ch3_doorbell_isr(void);

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
// QTimer1 raw VCLOCK helpers
// ============================================================================

static inline uint32_t vclock_count(void) { return qtimer1_read_32(); }

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
// ============================================================================

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) return 0;
  if (snap.dwt_cycles_per_s == 0) return 0;

  const uint32_t qtimer_elapsed = deadline - snap.qtimer_at_pps;
  const uint64_t ns_elapsed = (uint64_t)qtimer_elapsed * 50ULL;

  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;

  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

// ============================================================================
// QTimer1 CH2 compare management
// ============================================================================

static inline void ch2_clear_flag(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
}

static void ocr1_set(uint32_t value) {
  const uint16_t low16 = (uint16_t)(value & 0xFFFFu);
  IMXRT_TMR1.CH[2].COMP1  = low16;
  IMXRT_TMR1.CH[2].CMPLD1 = low16;
  ch2_clear_flag();
  IMXRT_TMR1.CH[2].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static void ocr1_disable(void) {
  IMXRT_TMR1.CH[2].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  ch2_clear_flag();
}

static void reload_ocr(void) {
  uint32_t now = vclock_count();
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
  uint32_t fire_vclock_raw,
  uint32_t fire_dwt_cyccnt,
  const time_anchor_snapshot_t& snap
) {
  slot.fire_vclock_raw               = fire_vclock_raw;
  slot.fire_dwt_cyccnt         = fire_dwt_cyccnt;
  slot.anchor_dwt_at_pps       = snap.dwt_at_pps;
  slot.anchor_dwt_cycles_per_s = snap.dwt_cycles_per_s;
  slot.anchor_qtimer_at_pps      = snap.qtimer_at_pps;
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
  ctx.fire_vclock_raw       = slot.fire_vclock_raw;
  ctx.deadline        = slot.deadline;
  ctx.fire_ns         = (int32_t)(slot.fire_vclock_raw - slot.deadline) * 50;
  ctx.fire_gnss_ns    = slot.fire_gnss_ns;
  ctx.nano_precise    = slot.nano_precise;
  ctx.nano_timeout    = slot.nano_timeout;
  ctx.fire_dwt_cyccnt = slot.fire_dwt_cyccnt;
}

// ============================================================================
// QTimer1 CH2 compare ISR
// ============================================================================

static void qtimer1_ch2_compare_isr(void) {

  const uint32_t dwt_raw = ARM_DWT_CYCCNT;
  const uint32_t dwt_now = dwt_at_gpt2_compare(dwt_raw);
  const uint32_t now     = qtimer1_read_32();

  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  ch2_clear_flag();
  isr_fire_count++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

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

static volatile bool vclock_test_in_flight    = false;
static volatile bool vclock_test_irq_armed    = false;
static volatile bool vclock_test_result_ready = false;
static Payload       vclock_test_result_ev;

static void vclock_test_init_qtimer1_ch3(void);

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};

  welford_dwt_prediction.reset();
  welford_gnss_now.reset();

  // Production TimePop compare on QTimer1 CH2.  CH0+CH1 are already
  // running as the passive 32-bit GNSS counter in process_clocks.
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = 0;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(2) | TMR_CTRL_PCS(0);

  vclock_test_in_flight    = false;
  vclock_test_irq_armed    = false;
  vclock_test_result_ready = false;
  ocr1_disable();
  vclock_test_disable_compare();

  attachInterruptVector(IRQ_QTIMER1, qtimer1_irq_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  vclock_test_init_qtimer1_ch3();
}

// ============================================================================
// Arm — standard
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
      slots[i].deadline = vclock_count() + ticks;

      slots[i].predicted_dwt = predict_dwt_at_deadline(
        slots[i].deadline, slots[i].prediction_valid);

      reload_ocr();
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
// Arm — nano-precise
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
    gpt2_deadline = vclock_count() + delay_ticks - NANO_EARLY_TICKS;
  } else {
    gpt2_deadline = vclock_count() + MIN_DELAY_TICKS;
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
// Slot-level Welford update
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

  const int32_t dwt_residual =
    (int32_t)(slot.fire_dwt_cyccnt - slot.predicted_dwt);
  welford_dwt_prediction.update((int64_t)dwt_residual);

  const int64_t epoch_ns = (int64_t)(pps - 1) * (int64_t)1000000000LL;

  const uint32_t qtimer_since_pps = slot.fire_vclock_raw - slot.anchor_qtimer_at_pps;
  const int64_t vclock_gnss_ns = epoch_ns + (int64_t)((uint64_t)qtimer_since_pps * 50ULL);

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
// Dispatch
// ============================================================================

void timepop_dispatch(void) {

  emit_deferred_ns_test();
  emit_deferred_vclock_test();

  if (!timepop_pending) return;
  timepop_pending = false;

  diag_dispatch_calls++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;

    slots[i].expired = false;

    // ── ASAP ──
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

    // ── Nano-precise ──
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

    // ── Standard ──

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
// TEST command
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

  int32_t dispatch_ns = (int32_t)(cb_vclock - ctx->fire_vclock_raw) * 50;
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
// INTERP_TEST
// ============================================================================

struct interp_test_ctx_t {
  uint32_t tests_remaining;
  uint64_t delay_ns;
  int64_t  arm_time_ns;
  uint32_t arm_vclock_raw;
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
  itest_ctx.arm_vclock_raw    = qtimer1_read_32();
  timepop_handle_t h = timepop_arm(
    itest_ctx.delay_ns, false, interp_test_callback, nullptr, "itest");
  if (h == TIMEPOP_INVALID_HANDLE) itest_in_flight = false;
}

static void interp_test_callback(timepop_ctx_t* ctx, void*) {

  bool path1_valid = false;
  int64_t path1_error_ns = 0;
  uint64_t vclock_ns_into_second = 0;

  bool path2_valid = false;
  int64_t path2_error_ns = 0;
  {
    const int64_t fire_time_ns = time_gnss_ns_now();
    const uint32_t fire_vclock_raw   = qtimer1_read_32();
    if (itest_ctx.arm_time_ns >= 0 && fire_time_ns >= 0) {
      const int64_t time_delta_ns   = fire_time_ns - itest_ctx.arm_time_ns;
      const int64_t vclock_delta_ns = (int64_t)((uint32_t)(fire_vclock_raw - itest_ctx.arm_vclock_raw)) * 50LL;
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
  out.add("vclock_raw_now",    vclock_count());
  out.add("test_in_flight",    test_in_flight);
  out.add("ns_test_in_flight", ns_test_in_flight);
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());
  out.add("nano_spins",        nano_spin_count);
  out.add("nano_timeouts",     nano_timeout_count);

  out.add("slots_active_now",  timepop_active_count());
  out.add("slots_high_water",  diag_slots_high_water);
  out.add("slots_max",         (uint32_t)MAX_SLOTS);
  out.add("arm_failures",      diag_arm_failures);

  out.add("asap_armed",             diag_asap_armed);
  out.add("asap_dispatched",        diag_asap_dispatched);
  out.add("asap_arm_failures",      diag_asap_arm_failures);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name", (const char*)(diag_asap_last_armed_name ? diag_asap_last_armed_name : ""));

  out.add("dispatch_calls",     diag_dispatch_calls);
  out.add("dispatch_callbacks", diag_dispatch_callbacks);

  out.add("dwt_pred_n",       (int32_t)welford_dwt_prediction.n);
  out.add("dwt_pred_stddev",  welford_dwt_prediction.stddev());
  out.add("gnss_now_n",       (int32_t)welford_gnss_now.n);
  out.add("gnss_now_stddev",  welford_gnss_now.stddev());

  out.add("vclock_test_in_flight",    vclock_test_in_flight);
  out.add("vclock_test_result_ready", vclock_test_result_ready);
  out.add("vclock_candidate_irqs", diag_vclock_candidate_irqs);
  out.add("vclock_early_rejects",  diag_vclock_early_rejects);

  // QTimer1 CH3 doorbell diagnostics (internal PCS routing from CH0 pin)
  out.add("qtmr1_ch2_cntr",   (uint32_t)IMXRT_TMR1.CH[2].CNTR);
  out.add("qtmr1_ch2_csctrl", (uint32_t)IMXRT_TMR1.CH[2].CSCTRL);
  out.add("qtmr1_ch3_cntr",   (uint32_t)IMXRT_TMR1.CH[3].CNTR);
  out.add("qtmr1_ch3_csctrl", (uint32_t)IMXRT_TMR1.CH[3].CSCTRL);

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
  out.add("anchor_qtimer_at_pps",      snap.qtimer_at_pps);
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
// VCLOCK_TEST v5.4 — QTimer1 CH3 compare doorbell (internal PCS routing)
//
// Architecture:
//   QTimer1 CH0 + CH1 = passive 32-bit counter.  Never touched by compare.
//   QTimer1 CH3       = independent 16-bit compare-only doorbell channel,
//                       counting both edges of CH0's input pin via PCS=0.
//                       No external wire, no IOMUX, no daisy chain.
//
// When QTimer1 CH3's compare fires, the ISR reads QTimer1 CH0+CH1 via
// qtimer1_read_32() — completely undisturbed.  IRQ_QTIMER1 is shared
// across all four channels; the ISR checks only CH3's TCF1 flag.
// CH0 and CH1 CSCTRL registers are NEVER modified by the ISR.
//
// QTimer1 CH3 target computation:
//   At arm time, read QTimer1 CH3's current 16-bit count.
//   Add the same raw_delta (requested_ticks * 2) used for the QTimer1 target.
//   The low 16 bits of that sum become QTimer1 CH3's COMP1 value.
//   The full 32-bit QTimer1 CH0+CH1 target is still computed for alias rejection.
// ============================================================================

struct vclock_test_context_t {
  uint32_t arm_vclock_raw;      // qtimer1_read_32() at arm (CH0+CH1)
  uint16_t arm_qtmr3_count;     // QTimer1 CH3 raw count at arm
  uint32_t arm_dwt;             // DWT_CYCCNT at arm
  int64_t  arm_gnss_ns;         // time_gnss_ns_now() at arm
  uint64_t requested_ns;        // requested interval
  uint32_t requested_ticks;     // requested_ns / 100
  uint32_t target_vclock_raw;   // QTimer1 full 32-bit target (alias rejection)
  uint16_t target_qtmr3_low16;  // QTimer1 CH3 COMP1 value
  uint32_t arm_pps_count;       // time_pps_count() at arm

  uint32_t end_vclock_raw;      // qtimer1_read_32() in ISR (undisturbed)
  uint32_t end_dwt;             // DWT_CYCCNT in ISR
  int64_t  end_gnss_ns;         // time_gnss_ns_now() in ISR
  uint32_t end_pps_count;       // time_pps_count() in ISR

  int32_t  fire_raw;            // end_vclock_raw - target_vclock_raw
  bool     fired;

  // ── Forensics ──
  uint32_t candidate_irq_count;
  uint32_t early_reject_count;
  uint32_t first_candidate_raw;
  uint32_t last_candidate_raw;
  uint32_t last_candidate_gap_raw;
  uint32_t min_candidate_gap_raw;
  uint32_t max_candidate_gap_raw;
};

static vclock_test_context_t vclock_test_ctx = {};

// ── QTimer1 CH3 register helpers ──

static inline void vclock_test_clear_flag(void) {
  uint16_t csctrl = IMXRT_TMR1.CH[3].CSCTRL;
  csctrl &= ~(TMR_CSCTRL_TCF1 | TMR_CSCTRL_TCF2);
  IMXRT_TMR1.CH[3].CSCTRL = csctrl;
}

static inline void vclock_test_disable_compare(void) {
  IMXRT_TMR1.CH[3].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  vclock_test_clear_flag();
}

static inline void vclock_test_arm_compare(uint16_t target_low16) {
  IMXRT_TMR1.CH[3].COMP1  = target_low16;
  IMXRT_TMR1.CH[3].CMPLD1 = target_low16;
  vclock_test_clear_flag();
  IMXRT_TMR1.CH[3].CSCTRL |= TMR_CSCTRL_TCF1EN;
}

static void vclock_test_init_qtimer1_ch3(void) {
  // ── Architecture ──────────────────────────────────────────────────────────
  // QTimer1 CH3 is configured as an independent compare-only doorbell,
  // counting both edges of Counter 0's input pin (GNSS 10 MHz on pin 10)
  // via the Primary Count Source (PCS) internal routing.
  //
  // No external wire, no IOMUX, no daisy chain.  PCS=0 within QTimer1
  // selects Counter 0's input pin as the primary clock source for CH3.
  // CM=2 counts both rising and falling edges, giving 20 MHz raw — same
  // rate as QTimer1 CH0.
  //
  // QTimer1 CH0+CH1 remain the passive 32-bit counting pair, untouched.
  // When CH3's compare fires, the ISR reads CH0+CH1 via qtimer1_read_32().
  //
  // IRQ_QTIMER1 is shared across all four channels.  The ISR checks only
  // CH3's TCF1 flag.  CH0 and CH1 CSCTRL are NEVER modified by the ISR.
  // ─────────────────────────────────────────────────────────────────────────

  IMXRT_TMR1.CH[3].CTRL   = 0;
  IMXRT_TMR1.CH[3].CNTR   = 0;
  IMXRT_TMR1.CH[3].LOAD   = 0;
  IMXRT_TMR1.CH[3].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[3].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[3].SCTRL  = 0;
  IMXRT_TMR1.CH[3].CSCTRL = 0;

  // CM=2 (count rising AND falling edges of primary source)
  // PCS=0 (primary clock source = Counter 0's input pin = GNSS 10 MHz)
  IMXRT_TMR1.CH[3].CTRL = TMR_CTRL_CM(2) | TMR_CTRL_PCS(0);
}

static void emit_deferred_vclock_test(void) {
  if (!vclock_test_result_ready) return;
  vclock_test_result_ready = false;
  publish("TIMEPOP_VCLOCK_TEST", vclock_test_result_ev);
  vclock_test_result_ev.clear();
}

static void qtimer1_irq_isr(void) {
  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_compare_isr();
  }
  if (IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch3_doorbell_isr();
  }
}

// ── ISR ──
// IRQ_QTIMER1 is shared across all four channels.
// We service only QTimer1 CH3's TCF1 flag.
// CH0 and CH1 CSCTRL are NEVER touched — they have no TCF1EN set.

static void qtimer1_ch3_doorbell_isr(void) {
  // Check QTimer1 CH3 flag — only channel we service.
  if ((IMXRT_TMR1.CH[3].CSCTRL & TMR_CSCTRL_TCF1) == 0) return;

  // Clear flag before any other work.
  vclock_test_clear_flag();

  if (!vclock_test_irq_armed) return;

  // ── Storm guard ──
  vclock_test_ctx.candidate_irq_count++;
  if (vclock_test_ctx.candidate_irq_count > 100) {
    vclock_test_disable_compare();
    vclock_test_irq_armed = false;
    vclock_test_in_flight = false;
    return;
  }

  // after candidate_irq_count++:
  diag_vclock_candidate_irqs = vclock_test_ctx.candidate_irq_count;

  // after early_reject_count++:
  diag_vclock_early_rejects = vclock_test_ctx.early_reject_count;

  // Read QTimer1 CH0+CH1 — completely undisturbed.
  const uint32_t now_raw = qtimer1_read_32();

  // ── Candidate forensics ──
  const uint32_t irq_n = vclock_test_ctx.candidate_irq_count;
  if (irq_n == 1) {
    vclock_test_ctx.first_candidate_raw    = now_raw;
    vclock_test_ctx.last_candidate_raw     = now_raw;
    vclock_test_ctx.last_candidate_gap_raw = 0;
    vclock_test_ctx.min_candidate_gap_raw  = 0;
    vclock_test_ctx.max_candidate_gap_raw  = 0;
  } else {
    const uint32_t gap = now_raw - vclock_test_ctx.last_candidate_raw;
    vclock_test_ctx.last_candidate_gap_raw = gap;
    if (irq_n == 2) {
      vclock_test_ctx.min_candidate_gap_raw = gap;
      vclock_test_ctx.max_candidate_gap_raw = gap;
    } else {
      if (gap < vclock_test_ctx.min_candidate_gap_raw) vclock_test_ctx.min_candidate_gap_raw = gap;
      if (gap > vclock_test_ctx.max_candidate_gap_raw) vclock_test_ctx.max_candidate_gap_raw = gap;
    }
    vclock_test_ctx.last_candidate_raw = now_raw;
  }

  // Alias rejection: QTimer1 CH3 fires every 65,536 raw counts (16-bit wrap).
  // At 20 MHz raw (dual-edge 10 MHz) that is ~3.28 ms per alias.
  // Reject any fire where the QTimer1 32-bit target has not yet arrived.
  if ((int32_t)(now_raw - vclock_test_ctx.target_vclock_raw) < 0) {
    vclock_test_ctx.early_reject_count++;
    return;
  }

  const uint32_t end_dwt     = ARM_DWT_CYCCNT;
  const int64_t  end_gnss_ns = time_gnss_ns_now();
  const uint32_t end_pps     = time_pps_count();

  vclock_test_disable_compare();
  vclock_test_irq_armed = false;

  vclock_test_ctx.end_vclock_raw = now_raw;
  vclock_test_ctx.end_dwt        = end_dwt;
  vclock_test_ctx.end_gnss_ns    = end_gnss_ns;
  vclock_test_ctx.end_pps_count  = end_pps;
  vclock_test_ctx.fire_raw       = (int32_t)(now_raw - vclock_test_ctx.target_vclock_raw);
  vclock_test_ctx.fired          = true;

  const uint32_t vclock_raw_delta =
    vclock_test_ctx.end_vclock_raw - vclock_test_ctx.arm_vclock_raw;

  const uint32_t vclock_ticks = vclock_raw_delta / 2U;

  const int32_t residual_ticks =
    (int32_t)vclock_ticks - (int32_t)vclock_test_ctx.requested_ticks;
  const int32_t residual_ns = residual_ticks * 50;

  const uint32_t dwt_elapsed =
    vclock_test_ctx.end_dwt - vclock_test_ctx.arm_dwt;

  int64_t gnss_elapsed_ns = -1;
  if (vclock_test_ctx.arm_gnss_ns >= 0 && vclock_test_ctx.end_gnss_ns >= 0) {
    gnss_elapsed_ns = vclock_test_ctx.end_gnss_ns - vclock_test_ctx.arm_gnss_ns;
  }

  // Published directly via publish() — no EVT_BODY_MAX constraint.
  Payload ev;
  ev.add("req_ns",              vclock_test_ctx.requested_ns);
  ev.add("requested_ticks",     (uint64_t)vclock_test_ctx.requested_ticks);
  ev.add("arm_vclock_raw",      (uint64_t)vclock_test_ctx.arm_vclock_raw);
  ev.add("target_vclock_raw",   (uint64_t)vclock_test_ctx.target_vclock_raw);
  ev.add("end_vclock_raw",      (uint64_t)vclock_test_ctx.end_vclock_raw);
  ev.add("vclock_raw_delta",    (uint64_t)vclock_raw_delta);
  ev.add("vclock_ticks",        (uint64_t)vclock_ticks);
  ev.add("residual_ticks",      residual_ticks);
  ev.add("residual_ns",         residual_ns);
  ev.add("fire_raw",            vclock_test_ctx.fire_raw);
  ev.add("fire_ns",             vclock_test_ctx.fire_raw * 50);
  ev.add("dwt_elapsed",         (uint64_t)dwt_elapsed);
  ev.add("gnss_elapsed_ns",     gnss_elapsed_ns);
  ev.add("arm_pps",             vclock_test_ctx.arm_pps_count);
  ev.add("end_pps",             vclock_test_ctx.end_pps_count);
  ev.add("candidate_irq_count",    vclock_test_ctx.candidate_irq_count);
  ev.add("early_reject_count",     vclock_test_ctx.early_reject_count);
  ev.add("first_candidate_raw",    (uint64_t)vclock_test_ctx.first_candidate_raw);
  ev.add("last_candidate_raw",     (uint64_t)vclock_test_ctx.last_candidate_raw);
  ev.add("last_candidate_gap_raw", (uint64_t)vclock_test_ctx.last_candidate_gap_raw);
  ev.add("min_candidate_gap_raw",  (uint64_t)vclock_test_ctx.min_candidate_gap_raw);
  ev.add("max_candidate_gap_raw",  (uint64_t)vclock_test_ctx.max_candidate_gap_raw);

  vclock_test_result_ev    = ev;
  vclock_test_result_ready = true;
  vclock_test_in_flight    = false;
  timepop_pending          = true;   // wake dispatch to emit the deferred event
}

static Payload cmd_vclock_test(const Payload& args) {
  Payload resp;

  if (vclock_test_in_flight || vclock_test_irq_armed) {
    resp.add("error", "test already in flight");
    return resp;
  }
  if (!time_valid()) {
    resp.add("error", "time not valid (need PPS lock)");
    return resp;
  }

  const uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val < 1000000ULL) {
    resp.add("error", "ns must be >= 1000000 (1 ms)");
    return resp;
  }
  if (ns_val > 999000000ULL) {
    resp.add("error", "ns must be <= 999000000 (999 ms)");
    return resp;
  }

  const uint32_t requested_ticks = (uint32_t)(ns_val / 100ULL);
  const uint32_t raw_delta       = requested_ticks * 2U;

  // Snapshot QTimer1 32-bit truth and QTimer1 CH3 doorbell at arm time.
  const uint32_t arm_vclock_raw  = qtimer1_read_32();
  const uint16_t arm_qtmr3_count = IMXRT_TMR1.CH[3].CNTR;
  const uint32_t arm_dwt         = ARM_DWT_CYCCNT;
  const int64_t  arm_gnss_ns     = time_gnss_ns_now();
  const uint32_t arm_pps_count   = time_pps_count();

  // QTimer1 32-bit target — used only for alias rejection in the ISR.
  const uint32_t target_vclock_raw = arm_vclock_raw + raw_delta;

  // QTimer1 CH3 target — the actual compare value that fires the doorbell.
  // QTimer1 CH3 is 16-bit only; use low 16 bits of (arm_qtmr3 + raw_delta).
  const uint16_t target_qtmr3_low16 =
    (uint16_t)((arm_qtmr3_count + (uint16_t)(raw_delta & 0xFFFFu)) & 0xFFFFu);

  vclock_test_ctx = {};
  vclock_test_ctx.arm_vclock_raw     = arm_vclock_raw;
  vclock_test_ctx.arm_qtmr3_count    = arm_qtmr3_count;
  vclock_test_ctx.arm_dwt            = arm_dwt;
  vclock_test_ctx.arm_gnss_ns        = arm_gnss_ns;
  vclock_test_ctx.requested_ns       = ns_val;
  vclock_test_ctx.requested_ticks    = requested_ticks;
  vclock_test_ctx.target_vclock_raw  = target_vclock_raw;
  vclock_test_ctx.target_qtmr3_low16 = target_qtmr3_low16;
  vclock_test_ctx.arm_pps_count      = arm_pps_count;

  noInterrupts();
  vclock_test_in_flight = true;
  vclock_test_irq_armed = true;
  vclock_test_arm_compare(target_qtmr3_low16);
  interrupts();

  resp.add("status",             "armed");
  resp.add("req_ns",             ns_val);
  resp.add("requested_ticks",    (uint64_t)requested_ticks);
  resp.add("arm_vclock_raw",     (uint64_t)arm_vclock_raw);
  resp.add("arm_qtmr3_count",    (uint32_t)arm_qtmr3_count);
  resp.add("target_vclock_raw",  (uint64_t)target_vclock_raw);
  resp.add("target_qtmr3_low16", (uint32_t)target_qtmr3_low16);
  resp.add("arm_pps",            arm_pps_count);
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
  { "VCLOCK_TEST", cmd_vclock_test },
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