// ============================================================================
// process_timepop.cpp — TimePop v9 (VCLOCK-exact GNSS, diag block)
// ============================================================================
//
// v9: VCLOCK-exact GNSS nanoseconds.
//
//   fire_gnss_ns is computed from VCLOCK arithmetic:
//     gnss_ns = (pps_count - 1) * 1e9 + (fire_vclock_raw - qtimer_at_pps) * 100
//
//   DWT is removed from timepop_ctx_t.  Clients that need DWT can
//   compute it from fire_gnss_ns via time_gnss_ns_to_dwt(), or read
//   it from the diagnostic block (timepop_diag_t).
//
//   The callback signature is now:
//     void (*)(timepop_ctx_t* ctx, timepop_diag_t* diag, void* user_data)
//
//   diag is nullable — null for ASAP dispatch, populated for ISR and
//   timed callbacks.
//
//   DWT spin is retained in the ISR for ISR callbacks that need to be
//   actively executing at a specific DWT moment (PPS shadow-write loop).
//   The spin result is captured in diag->dwt_at_fire.
//
// v8.1: ISR callback facility for timepop_arm_ns.
// v8.0: Priority queue with a single hardware comparator.
//
//   Pin assignment:
//     Pin 10 (GPIO_B0_00, ALT1) — QTimer1 CH0+CH1 — passive 32-bit (10 MHz)
//     QTimer1 CH2              — dynamic compare (priority queue scheduler)
//     QTimer1 CH3              — unallocated
//
// ============================================================================

#include "timepop.h"
#include "process_timepop.h"

#include "publish.h"

#include "debug.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "time.h"

#include <Arduino.h>
#include "imxrt.h"
#include <string.h>
#include <math.h>
#include <climits>

// ============================================================================
// Constants — 10 MHz domain
// ============================================================================

static constexpr uint32_t MAX_SLOTS = 16;
static constexpr uint64_t NS_PER_TICK = 100ULL;
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;
static constexpr uint32_t MIN_DELAY_TICKS = 2;
static constexpr uint32_t HEARTBEAT_TICKS = 10000;
static constexpr uint32_t DWT_SPIN_MAX_CYCLES = 110;
static constexpr uint32_t PREDICT_MAX_QTIMER_ELAPSED = 15000000U;

// ============================================================================
// Welford accumulator
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
  uint32_t            deadline;
  uint32_t            fire_vclock_raw;
  uint32_t            period_ticks;
  uint64_t            period_ns;

  int64_t             target_gnss_ns;
  int64_t             fire_gnss_ns;

  uint32_t            fire_dwt_cyccnt;

  uint32_t            anchor_dwt_at_pps;
  uint32_t            anchor_dwt_cycles_per_s;
  uint32_t            anchor_qtimer_at_pps;
  uint32_t            anchor_pps_count;
  bool                anchor_valid;

  uint32_t            predicted_dwt;
  bool                prediction_valid;

  bool                isr_callback;
  bool                isr_callback_fired;

  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;

  uint32_t            recurring_rearmed_count;
  uint32_t            recurring_immediate_expire_count;
  uint32_t            recurring_catchup_count;

  uint32_t            arm_vclock_raw;
  uint32_t            arm_delta_ticks;
  uint32_t            first_expire_now;
  bool                first_expire_recorded;
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

static volatile uint32_t diag_slots_high_water  = 0;
static volatile uint32_t diag_arm_failures      = 0;

static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

static welford_t welford_gnss_residual = {};
static welford_t welford_dwt_crosscheck = {};

static volatile uint32_t diag_prediction_samples = 0;
static volatile uint32_t diag_prediction_skipped = 0;

static volatile uint32_t diag_isr_count          = 0;
static volatile uint32_t diag_rearm_count        = 0;
static volatile uint32_t diag_heartbeat_rearms   = 0;
static volatile uint32_t diag_race_recoveries    = 0;
static volatile uint32_t diag_isr_callbacks      = 0;

static volatile uint32_t diag_schedule_next_calls_total = 0;
static volatile uint32_t diag_schedule_next_calls_from_dispatch = 0;
static volatile uint32_t diag_schedule_next_calls_from_other = 0;

// ============================================================================
// Forward declarations
// ============================================================================

static void qtimer1_irq_isr(void);
static void schedule_next(void);

// ============================================================================
// Helpers
// ============================================================================

static inline void update_slot_high_water(void) {
  uint32_t active = 0;
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) active++;
  }
  if (active > diag_slots_high_water) diag_slots_high_water = active;
}

// QTimer1 CH0+CH1 32-bit read, instrumented locally in TimePop.
// CH0 is low 16 bits, CH1 HOLD is high 16 bits of the cascaded counter.
static volatile uint32_t diag_qread_total = 0;
static volatile uint32_t diag_qread_same_hi = 0;
static volatile uint32_t diag_qread_retry_hi_changed = 0;
static volatile uint32_t diag_qread_monotonic_backsteps = 0;
static volatile uint32_t diag_qread_large_forward_jumps = 0;
static volatile uint16_t diag_qread_last_lo1 = 0;
static volatile uint16_t diag_qread_last_hi1 = 0;
static volatile uint16_t diag_qread_last_lo2 = 0;
static volatile uint16_t diag_qread_last_hi2 = 0;
static volatile uint32_t diag_qread_last_value = 0;

static inline uint32_t qtimer1_read_32_local(void) {
  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;

  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;

  diag_qread_total++;
  diag_qread_last_lo1 = lo1;
  diag_qread_last_hi1 = hi1;
  diag_qread_last_lo2 = lo2;
  diag_qread_last_hi2 = hi2;

  uint32_t value;
  if (hi1 != hi2) {
    diag_qread_retry_hi_changed++;
    value = ((uint32_t)hi2 << 16) | (uint32_t)lo2;
  } else {
    diag_qread_same_hi++;
    value = ((uint32_t)hi1 << 16) | (uint32_t)lo1;
  }

  const uint32_t prev = diag_qread_last_value;
  if (diag_qread_total > 1) {
    const int32_t signed_delta = (int32_t)(value - prev);
    if (signed_delta < 0) {
      diag_qread_monotonic_backsteps++;
    } else if ((uint32_t)signed_delta > 1000000U) {
      diag_qread_large_forward_jumps++;
    }
  }
  diag_qread_last_value = value;
  return value;
}

static inline uint32_t vclock_count(void) { return qtimer1_read_32_local(); }

static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

static inline uint32_t ns_to_ticks(uint64_t ns) {
  uint64_t ticks = ns / NS_PER_TICK;
  if (ticks < MIN_DELAY_TICKS) ticks = MIN_DELAY_TICKS;
  if (ticks > MAX_DELAY_TICKS) ticks = MAX_DELAY_TICKS;
  return (uint32_t)ticks;
}

static uint32_t predict_dwt_at_deadline(uint32_t deadline, bool& valid) {
  valid = false;
  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) return 0;
  if (snap.dwt_cycles_per_s == 0) return 0;
  const uint32_t qtimer_elapsed = deadline - snap.qtimer_at_pps;
  if (qtimer_elapsed > PREDICT_MAX_QTIMER_ELAPSED) return 0;
  const uint64_t ns_elapsed = (uint64_t)qtimer_elapsed * NS_PER_TICK;
  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;
  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

// ============================================================================
// VCLOCK-exact GNSS nanosecond computation
// ============================================================================

static int64_t vclock_to_gnss_ns(uint32_t vclock_raw,
                                 const time_anchor_snapshot_t& snap) {
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return -1;

  const uint32_t ticks_since_pps = vclock_raw - snap.qtimer_at_pps;
  return (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL
       + (int64_t)((uint64_t)ticks_since_pps * NS_PER_TICK);
}

// ============================================================================
// CH2 compare management
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
// schedule_next
// ============================================================================

static void schedule_next(void) {
  diag_schedule_next_calls_total++;

  const uint32_t now = vclock_count();
  uint32_t soonest = 0;
  bool found = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (deadline_expired(slots[i].deadline, now)) {
      if (!slots[i].first_expire_recorded) {
        slots[i].first_expire_now = now;
        slots[i].first_expire_recorded = true;
      }
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
    target = now + HEARTBEAT_TICKS;
    diag_heartbeat_rearms++;
  } else {
    uint32_t distance = soonest - now;
    if (distance > HEARTBEAT_TICKS) {
      target = now + HEARTBEAT_TICKS;
      diag_heartbeat_rearms++;
    } else {
      target = soonest;
    }
  }

  ch2_arm_compare((uint16_t)(target & 0xFFFF));
  diag_rearm_count++;

  const uint16_t cntr = IMXRT_TMR1.CH[2].CNTR;
  const uint16_t comp = (uint16_t)(target & 0xFFFF);
  const uint16_t distance_16 = comp - cntr;
  (void)distance_16;
}

// ============================================================================
// ISR helpers
// ============================================================================

static inline void slot_capture(
  timepop_slot_t& slot,
  uint32_t fire_vclock_raw,
  uint32_t fire_dwt_cyccnt,
  int64_t fire_gnss_ns,
  const time_anchor_snapshot_t& snap
) {
  slot.fire_vclock_raw         = fire_vclock_raw;
  slot.fire_dwt_cyccnt         = fire_dwt_cyccnt;
  slot.fire_gnss_ns            = fire_gnss_ns;
  slot.anchor_dwt_at_pps       = snap.dwt_at_pps;
  slot.anchor_dwt_cycles_per_s = snap.dwt_cycles_per_s;
  slot.anchor_qtimer_at_pps    = snap.qtimer_at_pps;
  slot.anchor_pps_count        = snap.pps_count;
  slot.anchor_valid            = snap.ok && snap.valid;
}

static inline void slot_build_ctx(
  const timepop_slot_t& slot,
  timepop_ctx_t& ctx
) {
  ctx.handle          = slot.handle;
  ctx.fire_vclock_raw = slot.fire_vclock_raw;
  ctx.deadline        = slot.deadline;
  ctx.fire_ns         = (int32_t)(slot.fire_vclock_raw - slot.deadline) * (int32_t)NS_PER_TICK;
  ctx.fire_gnss_ns    = slot.fire_gnss_ns;
}

static inline void slot_build_diag(
  const timepop_slot_t& slot,
  uint32_t dwt_at_isr_entry,
  timepop_diag_t& diag
) {
  diag.dwt_at_isr_entry      = dwt_at_isr_entry;
  diag.dwt_at_fire           = slot.fire_dwt_cyccnt;
  diag.predicted_dwt         = slot.predicted_dwt;
  diag.prediction_valid      = slot.prediction_valid;
  diag.spin_error_cycles     = (int32_t)(slot.fire_dwt_cyccnt - slot.predicted_dwt);
  diag.anchor_pps_count      = slot.anchor_pps_count;
  diag.anchor_qtimer_at_pps  = slot.anchor_qtimer_at_pps;
  diag.anchor_dwt_at_pps     = slot.anchor_dwt_at_pps;
  diag.anchor_dwt_cycles_per_s = slot.anchor_dwt_cycles_per_s;
  diag.anchor_valid          = slot.anchor_valid;
}

// ============================================================================
// QTimer1 CH2 ISR — priority queue scheduler
// ============================================================================

static void qtimer1_ch2_isr(void) {

  const uint32_t dwt_entry = ARM_DWT_CYCCNT;
  const uint32_t now       = qtimer1_read_32_local();
  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  ch2_clear_flag();
  diag_isr_count++;

  const int64_t gnss_ns_at_isr = vclock_to_gnss_ns(now, anchor);

  bool any_expired = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    uint32_t landed_dwt;

    if (slots[i].prediction_valid) {
      const uint32_t target_dwt = slots[i].predicted_dwt;

      if ((int32_t)(target_dwt - dwt_entry) > 0) {
        const uint32_t spin_start = ARM_DWT_CYCCNT;
        while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
          if ((ARM_DWT_CYCCNT - spin_start) > DWT_SPIN_MAX_CYCLES) break;
        }
        landed_dwt = ARM_DWT_CYCCNT;
      } else {
        landed_dwt = target_dwt;
      }
    } else {
      landed_dwt = dwt_entry;
    }

    slot_capture(slots[i], now, landed_dwt, gnss_ns_at_isr, anchor);

    slots[i].isr_callback_fired = false;

    if (slots[i].isr_callback) {
      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);

      timepop_diag_t diag;
      slot_build_diag(slots[i], dwt_entry, diag);

      slots[i].callback(&ctx, &diag, slots[i].user_data);
      slots[i].isr_callback_fired = true;
      diag_isr_callbacks++;
    }

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

  diag_schedule_next_calls_from_other++;
  schedule_next();
}

// ============================================================================
// QTimer1 initialization — TimePop owns CH0/CH1/CH2
// ============================================================================

static void qtimer1_init_vclock_base(void) {
  // QTimer1 CH0+CH1 — passive 32-bit GNSS counter (10 MHz on pin 10 / GPIO_B0_00 ALT1)
  // CH0 is the low word driven directly from the external 10 MHz clock.
  // CH1 is the cascade companion — counts CH0 overflows via CM(7).
  // Together they form the 32-bit VCLOCK that is the sovereign time substrate.

  // Enable QTimer1 clock gate.
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  // Pin 10 mux to ALT1 (QTimer1 CH0 external input) with explicit pad control.
  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  // Disable both channels before configuration.
  IMXRT_TMR1.CH[0].CTRL   = 0;
  IMXRT_TMR1.CH[1].CTRL   = 0;

  // CH0 — primary external count, CM(1) PCS(0) = count rising edges on pin input.
  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[0].LOAD   = 0;
  IMXRT_TMR1.CH[0].CNTR   = 0;
  IMXRT_TMR1.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  // CH1 — cascade from CH0 overflow via CM(7).
  // This is the high word of the 32-bit counter.
  // No LENGTH, no PCS — pure cascade.
  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(7);

  // Suppress any spurious interrupts from CH0/CH1.
  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  // Prime the HOLD path so the first qtimer1_read_32() has a defined baseline.
  (void)IMXRT_TMR1.CH[0].CNTR;
  (void)IMXRT_TMR1.CH[1].HOLD;
}

static void qtimer1_init_ch2_scheduler(void) {
  // QTimer1 CH2 — dynamic compare scheduler riding on the same 10 MHz source.
  IMXRT_TMR1.CH[2].CTRL   = 0;
  IMXRT_TMR1.CH[2].CNTR   = 0;
  IMXRT_TMR1.CH[2].LOAD   = 0;
  IMXRT_TMR1.CH[2].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[2].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[2].SCTRL  = 0;
  IMXRT_TMR1.CH[2].CSCTRL = TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[2].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
}

// ============================================================================
// Initialization
// ============================================================================

void timepop_init(void) {
  for (uint32_t i = 0; i < MAX_SLOTS; i++) slots[i] = {};

  welford_gnss_residual.reset();
  welford_dwt_crosscheck.reset();

  qtimer1_init_vclock_base();
  qtimer1_init_ch2_scheduler();

  attachInterruptVector(IRQ_QTIMER1, qtimer1_irq_isr);
  NVIC_SET_PRIORITY(IRQ_QTIMER1, 16);
  NVIC_ENABLE_IRQ(IRQ_QTIMER1);

  noInterrupts();
  diag_schedule_next_calls_from_other++;
  schedule_next();
  interrupts();
}

// ============================================================================
// Arm — standard
// ============================================================================

timepop_handle_t timepop_arm(
  uint64_t delay_ns, bool recurring,
  timepop_callback_t callback, void* user_data, const char* name
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
    slots[i].isr_callback = false;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].arm_vclock_raw = 0;
    slots[i].arm_delta_ticks = 0;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    if (is_asap) {
      slots[i].deadline = 0;
      slots[i].prediction_valid = false;
      slots[i].predicted_dwt    = 0;
      timepop_pending = true;
      diag_asap_armed++;
      diag_asap_last_armed_dwt  = ARM_DWT_CYCCNT;
      diag_asap_last_armed_name = name;
    } else {
      const uint32_t now = vclock_count();
      slots[i].deadline = now + ticks;
      slots[i].predicted_dwt = predict_dwt_at_deadline(
        slots[i].deadline, slots[i].prediction_valid);
      if (slots[i].prediction_valid) {
        slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
      }
      slots[i].arm_vclock_raw = now;
      slots[i].arm_delta_ticks = ticks;

      diag_schedule_next_calls_from_other++;
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
// Arm — nano-precise (with optional ISR callback)
// ============================================================================

timepop_handle_t timepop_arm_ns(
  int64_t target_gnss_ns, uint32_t target_dwt,
  timepop_callback_t callback, void* user_data,
  const char* name, bool isr_callback
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  int64_t now_ns = time_gnss_ns_now();
  if (now_ns < 0) return TIMEPOP_INVALID_HANDLE;

  int64_t delay_ns = target_gnss_ns - now_ns;
  if (delay_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  uint32_t delay_ticks = ns_to_ticks((uint64_t)delay_ns);
  if (delay_ticks < MIN_DELAY_TICKS) delay_ticks = MIN_DELAY_TICKS;

  const uint32_t now = vclock_count();
  const uint32_t vclock_deadline = now + delay_ticks;

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
    slots[i].isr_callback   = isr_callback;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].arm_vclock_raw = now;
    slots[i].arm_delta_ticks = delay_ticks;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    slots[i].predicted_dwt    = target_dwt;
    slots[i].prediction_valid = true;

    diag_schedule_next_calls_from_other++;
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
      diag_schedule_next_calls_from_other++;
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
  if (cancelled > 0) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }
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
// Slot-level Welford update — GNSS domain
// ============================================================================

static void slot_welford_update(const timepop_slot_t& slot) {

  if (!slot.prediction_valid || slot.target_gnss_ns <= 0) {
    diag_prediction_skipped++;
    return;
  }

  if (slot.fire_gnss_ns <= 0) {
    diag_prediction_skipped++;
    return;
  }

  const int64_t gnss_residual = slot.fire_gnss_ns - slot.target_gnss_ns;
  welford_gnss_residual.update(gnss_residual);

  if (slot.anchor_valid && slot.anchor_dwt_cycles_per_s > 0 &&
      slot.anchor_pps_count >= 2) {

    const uint32_t dwt_elapsed = slot.fire_dwt_cyccnt - slot.anchor_dwt_at_pps;
    const uint64_t dwt_ns_into_second =
      ((uint64_t)dwt_elapsed * 1000000000ULL + (uint64_t)slot.anchor_dwt_cycles_per_s / 2)
      / (uint64_t)slot.anchor_dwt_cycles_per_s;

    const int64_t epoch_ns = (int64_t)(slot.anchor_pps_count - 1) * (int64_t)1000000000LL;
    const int64_t dwt_gnss_ns = epoch_ns + (int64_t)dwt_ns_into_second;

    const int64_t crosscheck_residual = dwt_gnss_ns - slot.fire_gnss_ns;
    welford_dwt_crosscheck.update(crosscheck_residual);
  }

  diag_prediction_samples++;
}

// ============================================================================
// Dispatch
// ============================================================================

void timepop_dispatch(void) {

  emit_deferred_ns_test();

  if (!timepop_pending) return;
  timepop_pending = false;
  diag_dispatch_calls++;

  bool need_reschedule = false;
  const uint32_t dispatch_now = vclock_count();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;
    slots[i].expired = false;

    // ── ASAP (zero-delay, one-shot, non-ISR) ──
    if (slots[i].period_ns == 0 && !slots[i].recurring &&
        !slots[i].isr_callback_fired) {

      timepop_ctx_t ctx;
      slot_build_ctx(slots[i], ctx);

      uint32_t start = ARM_DWT_CYCCNT;
      slots[i].callback(&ctx, nullptr, slots[i].user_data);
      uint32_t end   = ARM_DWT_CYCCNT;

      cpu_usage_account_busy(end - start);
      diag_dispatch_callbacks++;
      diag_asap_dispatched++;
      diag_asap_last_dispatch_dwt = end;
      diag_prediction_skipped++;

      slots[i].active = false;
      continue;
    }

    // ── ISR-callback slot: callback already ran in ISR context ──
    if (slots[i].isr_callback_fired) {
      slot_welford_update(slots[i]);
      slots[i].active = false;
      continue;
    }

    // ── Timed (scheduled context callback) ──
    timepop_ctx_t ctx;
    slot_build_ctx(slots[i], ctx);

    timepop_diag_t diag;
    slot_build_diag(slots[i], 0, diag);

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, &diag, slots[i].user_data);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;
    slot_welford_update(slots[i]);

    // ── Recurring rearm ──
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
        noInterrupts();
        schedule_next();
        interrupts();
      }
    } else {
      slots[i].active = false;
    }
  }

  if (need_reschedule) {
    diag_schedule_next_calls_from_dispatch++;
    noInterrupts();
    schedule_next();
    interrupts();
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

static void test_timer_callback(timepop_ctx_t* ctx, timepop_diag_t*, void*) {
  uint32_t cb_vclock = qtimer1_read_32_local();
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
// NS_TEST
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

static void ns_test_timer_callback(timepop_ctx_t* ctx, timepop_diag_t* diag, void*) {
  int64_t expected_ns = ns_test_ctx.target_gnss_ns;
  int64_t error_ns = (ctx->fire_gnss_ns >= 0)
    ? ctx->fire_gnss_ns - expected_ns : -1;

  uint32_t qtimer_elapsed = ctx->fire_vclock_raw - ns_test_ctx.arm_vclock_raw;

  Payload ev;
  ev.add("err",       error_ns);
  ev.add("expected",  expected_ns);
  ev.add("actual",    ctx->fire_gnss_ns);
  ev.add("qtimer_elapsed", qtimer_elapsed);
  ev.add("req",       ns_test_ctx.requested_ns);
  ev.add("fire_ns",   ctx->fire_ns);

  if (diag) {
    ev.add("dwt_at_fire",       diag->dwt_at_fire);
    ev.add("spin_error_cycles", diag->spin_error_cycles);
    ev.add("prediction_valid",  diag->prediction_valid);
  }

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
  uint32_t arm_vclock_raw = qtimer1_read_32_local();
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
// REPORT
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload out;

  out.add("isr_fires",         isr_fire_count);
  out.add("isr_count",         diag_isr_count);
  out.add("isr_callbacks",     diag_isr_callbacks);
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

  out.add("gnss_residual_n",       (int32_t)welford_gnss_residual.n);
  out.add("gnss_residual_stddev",  welford_gnss_residual.stddev());
  out.add("dwt_crosscheck_n",      (int32_t)welford_dwt_crosscheck.n);
  out.add("dwt_crosscheck_stddev", welford_dwt_crosscheck.stddev());

  out.add("prediction_samples", diag_prediction_samples);
  out.add("prediction_skipped", diag_prediction_skipped);

  out.add("schedule_next_calls_total",         diag_schedule_next_calls_total);
  out.add("schedule_next_calls_from_dispatch", diag_schedule_next_calls_from_dispatch);
  out.add("schedule_next_calls_from_other",    diag_schedule_next_calls_from_other);

  out.add("qread_total",                  diag_qread_total);
  out.add("qread_same_hi",                diag_qread_same_hi);
  out.add("qread_retry_hi_changed",       diag_qread_retry_hi_changed);
  out.add("qread_monotonic_backsteps",    diag_qread_monotonic_backsteps);
  out.add("qread_large_forward_jumps",    diag_qread_large_forward_jumps);
  out.add("qread_last_lo1",               (uint32_t)diag_qread_last_lo1);
  out.add("qread_last_hi1",               (uint32_t)diag_qread_last_hi1);
  out.add("qread_last_lo2",               (uint32_t)diag_qread_last_lo2);
  out.add("qread_last_hi2",               (uint32_t)diag_qread_last_hi2);
  out.add("qread_last_value",             diag_qread_last_value);

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
    entry.add("isr_cb",    slots[i].isr_callback);
    entry.add("predicted_dwt",    slots[i].predicted_dwt);
    entry.add("prediction_valid", slots[i].prediction_valid);

    entry.add("recurring_rearmed_count",          slots[i].recurring_rearmed_count);
    entry.add("recurring_immediate_expire_count", slots[i].recurring_immediate_expire_count);
    entry.add("recurring_catchup_count",          slots[i].recurring_catchup_count);

    entry.add("arm_vclock_raw",        slots[i].arm_vclock_raw);
    entry.add("arm_delta_ticks",       slots[i].arm_delta_ticks);
    entry.add("first_expire_now",      slots[i].first_expire_now);
    entry.add("first_expire_recorded", slots[i].first_expire_recorded);

    timers.add(entry);
  }
  out.add_array("timers", timers);
  return out;
}

// ============================================================================
// DIAG
// ============================================================================

static void report_welford(Payload& out, const char* prefix, const welford_t& w) {
  char key[64];
  snprintf(key, sizeof(key), "%s_n", prefix);      out.add(key, (int32_t)w.n);
  snprintf(key, sizeof(key), "%s_mean", prefix);   out.add(key, w.mean);
  snprintf(key, sizeof(key), "%s_stddev", prefix); out.add(key, w.stddev());
  snprintf(key, sizeof(key), "%s_stderr", prefix); out.add(key, w.stderr_val());
  snprintf(key, sizeof(key), "%s_min", prefix);    out.add(key, (w.n > 0) ? w.min_val : (int64_t)0);
  snprintf(key, sizeof(key), "%s_max", prefix);    out.add(key, (w.n > 0) ? w.max_val : (int64_t)0);
  snprintf(key, sizeof(key), "%s_out100", prefix); out.add(key, w.outside_100ns);
  snprintf(key, sizeof(key), "%s_last", prefix);   out.add(key, w.last_residual);
}

static Payload cmd_diag(const Payload&) {
  Payload out;

  report_welford(out, "gnss_residual",    welford_gnss_residual);
  report_welford(out, "dwt_crosscheck",   welford_dwt_crosscheck);

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
// DIAG_RESET
// ============================================================================

static Payload cmd_diag_reset(const Payload&) {
  welford_gnss_residual.reset();
  welford_dwt_crosscheck.reset();
  diag_prediction_samples = 0;
  diag_prediction_skipped = 0;
  Payload out;
  out.add("status", "reset");
  return out;
}

// ============================================================================
// QTimer1 ISR
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
  { nullptr,       nullptr         }
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}