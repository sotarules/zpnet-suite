// ============================================================================
// process_timepop.cpp — TimePop (VCLOCK-exact GNSS scheduling)
// ============================================================================
//
// TimePop is the GNSS-domain timer subsystem.
//
// Core timing substrate:
//   • QTimer1 CH0+CH1 form a passive 32-bit VCLOCK counter at 10 MHz
//   • one VCLOCK tick = 100 ns
//   • QTimer1 CH2 is the dynamic compare scheduler
//
// Scheduling modes:
//   • relative scheduling        — timepop_arm(delay_gnss_ns, ...)
//   • scheduled ASAP dispatch    — timepop_arm_asap(...)
//   • scheduled ALAP dispatch    — timepop_arm_alap(...)
//   • absolute scheduling        — timepop_arm_at(target_gnss_ns, ...)
//   • anchor-relative scheduling — timepop_arm_from_anchor(anchor_gnss_ns, offset_gnss_ns, ...)
//   • caller-owned exact path    — timepop_arm_ns(target_gnss_ns, target_dwt, ...)
//
// fire_gnss_ns is computed from VCLOCK arithmetic:
//   gnss_ns = (pps_count - 1) * 1e9 + (fire_vclock_raw - qtimer_at_pps) * 100
//
// DWT is not the authoritative time base here.
// DWT is diagnostic / cross-check material.
//
// Scheduling philosophy — gears, not rubber bands:
//
//   Absolute and anchor-relative scheduling NEVER ask "what time is it now?"
//   Instead, deadlines are computed from the time anchor via pure arithmetic:
//
//     anchor_pps_gnss_ns = (pps_count - 1) * 1,000,000,000
//     ticks_from_anchor  = (target_gnss_ns - anchor_pps_gnss_ns) / 100
//     deadline           = qtimer_at_pps + ticks_from_anchor
//
//   This is possible because VCLOCK is phase-locked to GNSS.  The counter
//   IS the clock — reading it to compute a deadline introduces jitter that
//   pure arithmetic avoids.
//
//   Only relative scheduling ("fire N ns from now") reads the ambient counter,
//   because "from now" inherently requires knowing "now."
//
// Dispatch timing audit:
//
//   TimePop validates actual dispatch timing in the ISR using two independent
//   domains:
//
//   1. DWT-domain: captures ARM_DWT_CYCCNT, subtracts fixed ISR overhead,
//      converts to GNSS ns via the DWT bridge, compares against target.
//
//   2. VCLOCK-domain: reads the VCLOCK counter (CH0+CH1), converts to GNSS ns
//      via VCLOCK arithmetic, compares against target.  This is self-consistent
//      with how the deadline was scheduled and reveals any CH2/CH0 phase offset.
//
//   If the DWT domain shows ~585 ns error but the VCLOCK domain shows ~600 ns,
//   the cause is a boot-time initialization offset between CH0 and CH2 — both
//   count the same 10 MHz clock but CH0 was enabled first.
//
// CH2/CH0 initialization offset:
//
//   CH0 and CH2 both count the GNSS 10 MHz clock (CM=1, PCS=0) but are enabled
//   in separate init functions.  Between CH0's enable (in qtimer1_init_vclock_base)
//   and CH2's enable (in qtimer1_init_ch2_scheduler), peripheral register writes
//   for CH1 setup consume several VCLOCK ticks.  This creates a permanent phase
//   offset: CH2 lags CH0 by a small number of ticks.
//
//   Since deadlines are computed from CH0+CH1 but the compare fires on CH2, every
//   compare fires late by the offset.  The fix (once confirmed) is to synchronize
//   CH2 to CH0 at init time.
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
static constexpr uint32_t MAX_ASAP_SLOTS = 8;
static constexpr uint32_t MAX_ALAP_SLOTS = 4;
static constexpr uint64_t NS_PER_TICK = 100ULL;
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;
static constexpr uint32_t MIN_DELAY_TICKS = 2;
static constexpr uint32_t HEARTBEAT_TICKS = 10000;
static constexpr uint32_t DWT_SPIN_MAX_CYCLES = 110;
static constexpr uint32_t PREDICT_MAX_QTIMER_ELAPSED = 15000000U;
static constexpr int64_t  TIMEPOP_DISPATCH_GNSS_TOLERANCE_NS = 100LL;
static constexpr uint64_t SMARTPOP_EARLY_WAKE_NS = 1000ULL;
static constexpr uint32_t SMARTPOP_SPIN_MAX_CYCLES = 3000U;
static constexpr uint32_t SMARTPOP_MIN_MARGIN_CYCLES = 64U;

static inline uint32_t ns_to_dwt_cycles_runtime(uint64_t ns) {
  const uint32_t f = F_CPU_ACTUAL ? F_CPU_ACTUAL : 1008000000U;
  return (uint32_t)(((uint64_t)f * ns + 500000000ULL) / 1000000000ULL);
}


enum class monitored_client_t : uint8_t {
  NONE = 0,
  PPS,
  OCXO1,
  OCXO2,
};

static constexpr const char* TIMEPOP_MONITORED_NAMES[] = {
  "PPS_PRESPIN",
  "OCXO1_PRESPIN",
  "OCXO2_PRESPIN",
  nullptr
};

static inline monitored_client_t monitored_client_for_name(const char* name) {
  if (!name || !*name) return monitored_client_t::NONE;
  if (strcmp(name, TIMEPOP_MONITORED_NAMES[0]) == 0) return monitored_client_t::PPS;
  if (strcmp(name, TIMEPOP_MONITORED_NAMES[1]) == 0) return monitored_client_t::OCXO1;
  if (strcmp(name, TIMEPOP_MONITORED_NAMES[2]) == 0) return monitored_client_t::OCXO2;
  return monitored_client_t::NONE;
}

static inline bool is_monitored_name(const char* name) {
  return monitored_client_for_name(name) != monitored_client_t::NONE;
}

// ============================================================================
// Slot
// ============================================================================

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;
  bool                is_absolute;

  timepop_handle_t    handle;
  uint32_t            deadline;
  uint32_t            target_deadline;
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

  bool                smartpop_enabled;
  uint32_t            smartpop_early_ticks;

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


struct deferred_slot_t {
  bool                active;
  timepop_handle_t    handle;
  timepop_callback_t  callback;
  void*               user_data;
  const char*         name;
};

// ============================================================================
// State
// ============================================================================

static timepop_slot_t slots[MAX_SLOTS];
static deferred_slot_t asap_slots[MAX_ASAP_SLOTS];
static deferred_slot_t alap_slots[MAX_ALAP_SLOTS];
static volatile bool   timepop_pending = false;
static uint32_t        next_handle = 1;

// ============================================================================
// Diagnostics
// ============================================================================

static volatile uint32_t isr_fire_count     = 0;
static volatile uint32_t expired_count      = 0;
static volatile uint32_t phantom_count      = 0;

static volatile uint32_t diag_slots_high_water        = 0;
static volatile uint32_t diag_asap_slots_high_water   = 0;
static volatile uint32_t diag_alap_slots_high_water   = 0;
static volatile uint32_t diag_arm_failures            = 0;
static volatile uint32_t diag_named_replacements      = 0;
static volatile uint32_t diag_named_replacements_pps_prespin = 0;
static volatile uint32_t diag_named_replacements_ocxo1_prespin = 0;
static volatile uint32_t diag_named_replacements_ocxo2_prespin = 0;
static volatile uint32_t diag_named_replacements_pps_dispatch = 0;
static volatile uint32_t diag_named_replacements_ocxo1_dispatch = 0;
static volatile uint32_t diag_named_replacements_ocxo2_dispatch = 0;

static volatile uint32_t diag_asap_armed              = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

static volatile uint32_t diag_alap_armed              = 0;
static volatile uint32_t diag_alap_dispatched         = 0;
static volatile uint32_t diag_alap_arm_failures       = 0;
static volatile uint32_t diag_alap_last_armed_dwt     = 0;
static volatile uint32_t diag_alap_last_dispatch_dwt  = 0;
static volatile const char* diag_alap_last_armed_name = nullptr;

static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

static volatile uint32_t diag_isr_count          = 0;
static volatile uint32_t diag_rearm_count        = 0;
static volatile uint32_t diag_heartbeat_rearms   = 0;
static volatile uint32_t diag_race_recoveries    = 0;
static volatile uint32_t diag_isr_callbacks      = 0;

static volatile uint32_t diag_schedule_next_calls_total = 0;
static volatile uint32_t diag_schedule_next_calls_from_dispatch = 0;
static volatile uint32_t diag_schedule_next_calls_from_other = 0;

// SmartPop / dispatch timing diagnostics
static volatile uint32_t diag_dispatch_gnss_checks = 0;
static volatile uint32_t diag_dispatch_gnss_mismatches = 0;
static volatile uint32_t diag_dispatch_gnss_skipped_no_target = 0;
static volatile uint32_t diag_dispatch_gnss_skipped_invalid_time = 0;
static volatile uint32_t diag_dispatch_gnss_skipped_unmonitored = 0;
static volatile int64_t  diag_dispatch_gnss_last_target_ns = -1;
static volatile int64_t  diag_dispatch_gnss_last_actual_ns = -1;
static volatile int64_t  diag_dispatch_gnss_last_error_ns = 0;
static volatile uint64_t diag_dispatch_gnss_max_abs_error_ns = 0;

static volatile uint32_t diag_smartpop_checks = 0;
static volatile uint32_t diag_smartpop_armed = 0;
static volatile uint32_t diag_smartpop_fallback_no_prediction = 0;
static volatile uint32_t diag_smartpop_fallback_late_arm = 0;
static volatile uint32_t diag_smartpop_woke_early = 0;
static volatile uint32_t diag_smartpop_woke_late = 0;
static volatile uint32_t diag_smartpop_spin_attempts = 0;
static volatile uint32_t diag_smartpop_spin_success = 0;
static volatile uint32_t diag_smartpop_spin_timeouts = 0;
static volatile uint32_t diag_smartpop_last_target_dwt = 0;
static volatile uint32_t diag_smartpop_last_entry_dwt = 0;
static volatile uint32_t diag_smartpop_last_land_dwt = 0;
static volatile int32_t  diag_smartpop_last_cycles_early = 0;
static volatile uint32_t diag_smartpop_last_spin_cycles = 0;
static volatile uint32_t diag_smartpop_max_spin_cycles = 0;
static volatile uint32_t diag_smartpop_max_cycles_early = 0;

static volatile uint32_t diag_smartpop_pps_checks = 0;
static volatile uint32_t diag_smartpop_ocxo1_checks = 0;
static volatile uint32_t diag_smartpop_ocxo2_checks = 0;
static volatile uint32_t diag_smartpop_dwt_adjust_checks = 0;

static volatile uint32_t diag_smartpop_pps_spin_success = 0;
static volatile uint32_t diag_smartpop_ocxo1_spin_success = 0;
static volatile uint32_t diag_smartpop_ocxo2_spin_success = 0;
static volatile uint32_t diag_smartpop_dwt_adjust_spin_success = 0;

static volatile uint32_t diag_smartpop_pps_spin_timeouts = 0;
static volatile uint32_t diag_smartpop_ocxo1_spin_timeouts = 0;
static volatile uint32_t diag_smartpop_ocxo2_spin_timeouts = 0;
static volatile uint32_t diag_smartpop_dwt_adjust_spin_timeouts = 0;

static volatile uint32_t diag_smartpop_pps_fallback_no_prediction = 0;
static volatile uint32_t diag_smartpop_ocxo1_fallback_no_prediction = 0;
static volatile uint32_t diag_smartpop_ocxo2_fallback_no_prediction = 0;
static volatile uint32_t diag_smartpop_dwt_adjust_fallback_no_prediction = 0;

static volatile uint32_t diag_smartpop_pps_fallback_late_arm = 0;
static volatile uint32_t diag_smartpop_ocxo1_fallback_late_arm = 0;
static volatile uint32_t diag_smartpop_ocxo2_fallback_late_arm = 0;
static volatile uint32_t diag_smartpop_dwt_adjust_fallback_late_arm = 0;

static volatile int64_t  diag_dispatch_gnss_worst_target_ns = -1;
static volatile int64_t  diag_dispatch_gnss_worst_actual_ns = -1;
static volatile int64_t  diag_dispatch_gnss_worst_error_ns = 0;
static volatile uint32_t diag_dispatch_gnss_worst_handle = 0;
static volatile bool     diag_dispatch_gnss_worst_prediction_valid = false;
static volatile bool     diag_dispatch_gnss_worst_smartpop_enabled = false;
static volatile const char* diag_dispatch_gnss_worst_name = nullptr;


static volatile uint32_t diag_deadline_negative_offset = 0;
static volatile int64_t  diag_deadline_last_target_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_anchor_pps_gnss_ns = -1;
static volatile int64_t  diag_deadline_last_ns_from_anchor = 0;

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


static inline void update_deferred_high_water(deferred_slot_t* slots_buf,
                                              uint32_t max_slots,
                                              volatile uint32_t& high_water) {
  uint32_t active = 0;
  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].active) active++;
  }
  if (active > high_water) high_water = active;
}

// QTimer1 CH0+CH1 32-bit read, instrumented locally in TimePop.
// CH0 is low 16 bits, CH1 HOLD is high 16 bits of the cascaded counter.
//
// This read is used ONLY in contexts where ambient time is genuinely needed:
//   • schedule_next() — must compare deadlines against the current counter
//   • relative scheduling — "from now" inherently requires knowing "now"
//   • recurring timer catch-up — relative timers that fell behind
// //
// Absolute and anchor-relative scheduling NEVER call this function.
// Their deadlines are computed from the time anchor via pure arithmetic.
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

static inline uint32_t vclock_count(void) {
  return qtimer1_read_32_local();
}

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

  const uint64_t ns_elapsed =
    (uint64_t)qtimer_elapsed * NS_PER_TICK;

  const uint64_t dwt_elapsed =
    (ns_elapsed * (uint64_t)snap.dwt_cycles_per_s + 500000000ULL) / 1000000000ULL;

  valid = true;
  return snap.dwt_at_pps + (uint32_t)dwt_elapsed;
}

static inline void smartpop_note_check_for_name(const char* name) {
  switch (monitored_client_for_name(name)) {
    case monitored_client_t::PPS:   diag_smartpop_pps_checks++; break;
    case monitored_client_t::OCXO1: diag_smartpop_ocxo1_checks++; break;
    case monitored_client_t::OCXO2: diag_smartpop_ocxo2_checks++; break;
    default: break;
  }
}

static inline void smartpop_note_spin_success_for_name(const char* name) {
  switch (monitored_client_for_name(name)) {
    case monitored_client_t::PPS:   diag_smartpop_pps_spin_success++; break;
    case monitored_client_t::OCXO1: diag_smartpop_ocxo1_spin_success++; break;
    case monitored_client_t::OCXO2: diag_smartpop_ocxo2_spin_success++; break;
    default: break;
  }
}

static inline void smartpop_note_spin_timeout_for_name(const char* name) {
  switch (monitored_client_for_name(name)) {
    case monitored_client_t::PPS:   diag_smartpop_pps_spin_timeouts++; break;
    case monitored_client_t::OCXO1: diag_smartpop_ocxo1_spin_timeouts++; break;
    case monitored_client_t::OCXO2: diag_smartpop_ocxo2_spin_timeouts++; break;
    default: break;
  }
}

static inline void smartpop_note_fallback_late_arm_for_name(const char* name) {
  switch (monitored_client_for_name(name)) {
    case monitored_client_t::PPS:   diag_smartpop_pps_fallback_late_arm++; break;
    case monitored_client_t::OCXO1: diag_smartpop_ocxo1_fallback_late_arm++; break;
    case monitored_client_t::OCXO2: diag_smartpop_ocxo2_fallback_late_arm++; break;
    default: break;
  }
}

static inline void smartpop_note_fallback_no_prediction_for_name(const char* name) {
  switch (monitored_client_for_name(name)) {
    case monitored_client_t::PPS:   diag_smartpop_pps_fallback_no_prediction++; break;
    case monitored_client_t::OCXO1: diag_smartpop_ocxo1_fallback_no_prediction++; break;
    case monitored_client_t::OCXO2: diag_smartpop_ocxo2_fallback_no_prediction++; break;
    default: break;
  }
}

static inline void smartpop_prepare_slot(timepop_slot_t& slot) {
  slot.target_deadline = slot.deadline;
  slot.smartpop_enabled = false;
  slot.smartpop_early_ticks = 0;

  if (!slot.prediction_valid || slot.predicted_dwt == 0 || slot.target_gnss_ns < 0) {
    diag_smartpop_fallback_no_prediction++;
    smartpop_note_fallback_no_prediction_for_name(slot.name);
    return;
  }

  const uint32_t early_ticks = (uint32_t)(SMARTPOP_EARLY_WAKE_NS / NS_PER_TICK);
  if (early_ticks == 0) {
    diag_smartpop_fallback_no_prediction++;
    smartpop_note_fallback_no_prediction_for_name(slot.name);
    return;
  }

  const uint32_t early_cycles = ns_to_dwt_cycles_runtime(SMARTPOP_EARLY_WAKE_NS);
  const uint32_t now_dwt = ARM_DWT_CYCCNT;
  const int32_t margin_cycles = (int32_t)(slot.predicted_dwt - now_dwt);

  if (margin_cycles <= (int32_t)(early_cycles + SMARTPOP_MIN_MARGIN_CYCLES)) {
    diag_smartpop_fallback_late_arm++;
    smartpop_note_fallback_late_arm_for_name(slot.name);
    return;
  }

  slot.deadline = slot.target_deadline - early_ticks;
  slot.smartpop_enabled = true;
  slot.smartpop_early_ticks = early_ticks;
  diag_smartpop_armed++;
}

// ============================================================================
// Absolute GNSS nanosecond → VCLOCK deadline (pure arithmetic)
// ============================================================================
//
// Converts a target GNSS nanosecond to a VCLOCK deadline using the time
// anchor.  No ambient counter read.  No "what time is it now?"
//
// The arithmetic:
//   anchor_pps_gnss_ns = (pps_count - 1) * 1,000,000,000
//   ns_from_anchor     = target_gnss_ns - anchor_pps_gnss_ns
//   ticks_from_anchor  = ns_from_anchor / 100
//   deadline           = qtimer_at_pps + ticks_from_anchor
//
// Returns true if the conversion succeeded (anchor valid, target in range).
// On success, writes the VCLOCK deadline to *out_deadline.
static bool gnss_ns_to_vclock_deadline(int64_t target_gnss_ns,
                                       const time_anchor_snapshot_t& snap,
                                       uint32_t& out_deadline) {
  if (!snap.ok || !snap.valid || snap.pps_count < 1) return false;

  const int64_t anchor_pps_gnss_ns =
    (int64_t)(snap.pps_count - 1) * (int64_t)1000000000LL;

  const int64_t ns_from_anchor = target_gnss_ns - anchor_pps_gnss_ns;

  diag_deadline_last_target_gnss_ns = target_gnss_ns;
  diag_deadline_last_anchor_pps_gnss_ns = anchor_pps_gnss_ns;
  diag_deadline_last_ns_from_anchor = ns_from_anchor;

  if (ns_from_anchor < 0) {
    diag_deadline_negative_offset++;
    return false;
  }

  const uint64_t ticks64 = (uint64_t)ns_from_anchor / NS_PER_TICK;
  if (ticks64 > 0xFFFFFFFFULL) {
    return false;
  }

  out_deadline = snap.qtimer_at_pps + (uint32_t)ticks64;
  return true;
}

// ============================================================================
// Named singleton replacement
// ============================================================================

static inline void note_named_replacement(const char* name) {
  if (!name || !*name) return;
  if (strcmp(name, "PPS_PRESPIN") == 0) {
    diag_named_replacements_pps_prespin++;
  } else if (strcmp(name, "OCXO1_PRESPIN") == 0) {
    diag_named_replacements_ocxo1_prespin++;
  } else if (strcmp(name, "OCXO2_PRESPIN") == 0) {
    diag_named_replacements_ocxo2_prespin++;
  } else if (strcmp(name, "PPS_DISPATCH") == 0) {
    diag_named_replacements_pps_dispatch++;
  } else if (strcmp(name, "OCXO1_DISPATCH") == 0) {
    diag_named_replacements_ocxo1_dispatch++;
  } else if (strcmp(name, "OCXO2_DISPATCH") == 0) {
    diag_named_replacements_ocxo2_dispatch++;
  }
}

static bool retire_existing_named_slots(const char* name) {
  if (!name || !*name) return false;

  bool retired_any = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active) continue;
    if (!slots[i].name) continue;
    if (strcmp(slots[i].name, name) != 0) continue;

    slots[i].active = false;
    slots[i].expired = false;
    retired_any = true;
    diag_named_replacements++;
    note_named_replacement(name);
  }

  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (!asap_slots[i].active) continue;
    if (!asap_slots[i].name) continue;
    if (strcmp(asap_slots[i].name, name) != 0) continue;

    asap_slots[i].active = false;
    retired_any = true;
    diag_named_replacements++;
    note_named_replacement(name);
  }

  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (!alap_slots[i].active) continue;
    if (!alap_slots[i].name) continue;
    if (strcmp(alap_slots[i].name, name) != 0) continue;

    alap_slots[i].active = false;
    retired_any = true;
    diag_named_replacements++;
    note_named_replacement(name);
  }

  return retired_any;
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
// Dispatch timing validation — landed DWT domain
// ============================================================================

static inline void validate_dispatch_against_dwt(const timepop_slot_t& slot,
                                                 uint32_t landed_dwt) {
  if (!is_monitored_name(slot.name)) {
    diag_dispatch_gnss_skipped_unmonitored++;
    return;
  }

  if (slot.target_gnss_ns < 0) {
    diag_dispatch_gnss_skipped_no_target++;
    return;
  }

  const int64_t actual_gnss_ns = time_dwt_to_gnss_ns(landed_dwt);
  if (actual_gnss_ns < 0) {
    diag_dispatch_gnss_skipped_invalid_time++;
    return;
  }

  diag_dispatch_gnss_checks++;
  diag_dispatch_gnss_last_target_ns = slot.target_gnss_ns;
  diag_dispatch_gnss_last_actual_ns = actual_gnss_ns;

  const int64_t error_ns = actual_gnss_ns - slot.target_gnss_ns;
  diag_dispatch_gnss_last_error_ns = error_ns;

  const uint64_t abs_error_ns = (uint64_t)llabs(error_ns);
  if (abs_error_ns > diag_dispatch_gnss_max_abs_error_ns) {
    diag_dispatch_gnss_max_abs_error_ns = abs_error_ns;
    diag_dispatch_gnss_worst_target_ns = slot.target_gnss_ns;
    diag_dispatch_gnss_worst_actual_ns = actual_gnss_ns;
    diag_dispatch_gnss_worst_error_ns = error_ns;
    diag_dispatch_gnss_worst_handle = slot.handle;
    diag_dispatch_gnss_worst_prediction_valid = slot.prediction_valid;
    diag_dispatch_gnss_worst_smartpop_enabled = slot.smartpop_enabled;
    diag_dispatch_gnss_worst_name = slot.name;
  }

  if (abs_error_ns > (uint64_t)TIMEPOP_DISPATCH_GNSS_TOLERANCE_NS) {
    diag_dispatch_gnss_mismatches++;
  }
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
//
// This is the ONE place in TimePop that legitimately reads the ambient
// VCLOCK counter.  It must compare deadlines against the current counter
// to determine which slots have expired and how far away the nearest
// deadline is.  This is the heartbeat polling loop.

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
  ctx.handle              = slot.handle;
  ctx.fire_vclock_raw     = slot.fire_vclock_raw;
  ctx.deadline            = slot.target_deadline;
  ctx.fire_gnss_error_ns  = (int32_t)(slot.fire_vclock_raw - slot.target_deadline) * (int32_t)NS_PER_TICK;
  ctx.fire_gnss_ns        = slot.fire_gnss_ns;
}

static inline void slot_build_diag(
  const timepop_slot_t& slot,
  uint32_t dwt_at_isr_entry,
  timepop_diag_t& diag
) {
  diag.dwt_at_isr_entry        = dwt_at_isr_entry;
  diag.dwt_at_fire             = slot.fire_dwt_cyccnt;
  diag.predicted_dwt           = slot.predicted_dwt;
  diag.prediction_valid        = slot.prediction_valid;
  diag.spin_error_cycles       = (int32_t)(slot.fire_dwt_cyccnt - slot.predicted_dwt);
  diag.anchor_pps_count        = slot.anchor_pps_count;
  diag.anchor_qtimer_at_pps    = slot.anchor_qtimer_at_pps;
  diag.anchor_dwt_at_pps       = slot.anchor_dwt_at_pps;
  diag.anchor_dwt_cycles_per_s = slot.anchor_dwt_cycles_per_s;
  diag.anchor_valid            = slot.anchor_valid;
}

// ============================================================================
// QTimer1 CH2 ISR — priority queue scheduler
// ============================================================================

static void qtimer1_ch2_isr(uint32_t dwt_entry) {

  const uint32_t now = qtimer1_read_32_local();
  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  ch2_clear_flag();
  diag_isr_count++;

  const int64_t gnss_ns_at_isr = vclock_to_gnss_ns(now, anchor);

  bool any_expired = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    uint32_t landed_dwt = dwt_entry;

    if (slots[i].smartpop_enabled && slots[i].prediction_valid) {
      if (is_monitored_name(slots[i].name)) {
        diag_smartpop_checks++;
        smartpop_note_check_for_name(slots[i].name);
      }
      diag_smartpop_last_target_dwt = slots[i].predicted_dwt;
      diag_smartpop_last_entry_dwt = dwt_entry;

      const int32_t cycles_early = (int32_t)(slots[i].predicted_dwt - dwt_entry);
      diag_smartpop_last_cycles_early = cycles_early;

      if (cycles_early > 0) {
        diag_smartpop_woke_early++;
        if ((uint32_t)cycles_early > diag_smartpop_max_cycles_early) {
          diag_smartpop_max_cycles_early = (uint32_t)cycles_early;
        }

        diag_smartpop_spin_attempts++;
        const uint32_t spin_start = ARM_DWT_CYCCNT;
        while ((int32_t)(slots[i].predicted_dwt - ARM_DWT_CYCCNT) > 0) {
          if ((ARM_DWT_CYCCNT - spin_start) > SMARTPOP_SPIN_MAX_CYCLES) {
            break;
          }
        }
        landed_dwt = ARM_DWT_CYCCNT;
        const uint32_t spin_cycles = landed_dwt - spin_start;
        diag_smartpop_last_spin_cycles = spin_cycles;
        if (spin_cycles > diag_smartpop_max_spin_cycles) {
          diag_smartpop_max_spin_cycles = spin_cycles;
        }

        if ((int32_t)(slots[i].predicted_dwt - landed_dwt) <= 0) {
          diag_smartpop_spin_success++;
          smartpop_note_spin_success_for_name(slots[i].name);
        } else {
          diag_smartpop_spin_timeouts++;
          smartpop_note_spin_timeout_for_name(slots[i].name);
        }
      } else {
        diag_smartpop_woke_late++;
        diag_smartpop_last_spin_cycles = 0;
        landed_dwt = dwt_entry;
      }

      diag_smartpop_last_land_dwt = landed_dwt;
    } else if (slots[i].prediction_valid) {
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
    }

    validate_dispatch_against_dwt(slots[i], landed_dwt);

    int64_t fire_gnss_ns = gnss_ns_at_isr;
    if (slots[i].target_gnss_ns >= 0 && slots[i].smartpop_enabled && slots[i].prediction_valid) {
      fire_gnss_ns = slots[i].target_gnss_ns;
    } else {
      const int64_t dwt_gnss_ns = time_dwt_to_gnss_ns(landed_dwt);
      if (dwt_gnss_ns >= 0) fire_gnss_ns = dwt_gnss_ns;
    }

    slot_capture(slots[i], now, landed_dwt, fire_gnss_ns, anchor);

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


static inline void build_deferred_ctx(timepop_handle_t handle, timepop_ctx_t& ctx) {
  ctx.handle = handle;
  ctx.fire_vclock_raw = 0;
  ctx.deadline = 0;
  ctx.fire_gnss_error_ns = 0;
  ctx.fire_gnss_ns = -1;
}

static void dispatch_deferred_phase(deferred_slot_t* slots_buf,
                                    uint32_t max_slots,
                                    volatile uint32_t& dispatched_count,
                                    volatile uint32_t& last_dispatch_dwt) {
  bool pending_this_pass[MAX_ASAP_SLOTS > MAX_ALAP_SLOTS ? MAX_ASAP_SLOTS : MAX_ALAP_SLOTS] = {};
  for (uint32_t i = 0; i < max_slots; i++) {
    pending_this_pass[i] = slots_buf[i].active;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (!pending_this_pass[i]) continue;
    if (!slots_buf[i].active) continue;

    timepop_ctx_t ctx;
    build_deferred_ctx(slots_buf[i].handle, ctx);

    uint32_t start = ARM_DWT_CYCCNT;
    slots_buf[i].callback(&ctx, nullptr, slots_buf[i].user_data);
    uint32_t end = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;
    dispatched_count++;
    last_dispatch_dwt = end;

    slots_buf[i].active = false;
  }
}

static timepop_handle_t arm_deferred(deferred_slot_t* slots_buf,
                                     uint32_t max_slots,
                                     volatile uint32_t& arm_failures,
                                     volatile uint32_t& armed_count,
                                     volatile uint32_t& last_armed_dwt,
                                     volatile const char*& last_armed_name,
                                     volatile uint32_t& high_water,
                                     timepop_callback_t callback,
                                     void* user_data,
                                     const char* name) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  noInterrupts();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    timepop_pending = true;
  }

  for (uint32_t i = 0; i < max_slots; i++) {
    if (slots_buf[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots_buf[i] = {};
    slots_buf[i].active = true;
    slots_buf[i].handle = h;
    slots_buf[i].callback = callback;
    slots_buf[i].user_data = user_data;
    slots_buf[i].name = name;

    timepop_pending = true;
    armed_count++;
    last_armed_dwt = ARM_DWT_CYCCNT;
    last_armed_name = name;

    update_deferred_high_water(slots_buf, max_slots, high_water);

    interrupts();
    return h;
  }

  arm_failures++;
  interrupts();
  return TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// QTimer1 initialization — TimePop owns CH0/CH1/CH2
// ============================================================================

static void qtimer1_init_vclock_base(void) {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  *(portConfigRegister(10)) = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IMXRT_TMR1.CH[0].CTRL = 0;
  IMXRT_TMR1.CH[1].CTRL = 0;

  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[0].LOAD   = 0;
  IMXRT_TMR1.CH[0].CNTR   = 0;
  IMXRT_TMR1.CH[0].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].LOAD   = 0;
  IMXRT_TMR1.CH[1].CNTR   = 0;
  IMXRT_TMR1.CH[1].COMP1  = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CTRL   = TMR_CTRL_CM(7);

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  (void)IMXRT_TMR1.CH[0].CNTR;
  (void)IMXRT_TMR1.CH[1].HOLD;
}

static void qtimer1_init_ch2_scheduler(void) {
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
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) asap_slots[i] = {};
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) alap_slots[i] = {};

  diag_dispatch_gnss_checks = 0;
  diag_dispatch_gnss_mismatches = 0;
  diag_dispatch_gnss_skipped_no_target = 0;
  diag_dispatch_gnss_skipped_invalid_time = 0;
  diag_dispatch_gnss_skipped_unmonitored = 0;
  diag_dispatch_gnss_last_target_ns = -1;
  diag_dispatch_gnss_last_actual_ns = -1;
  diag_dispatch_gnss_last_error_ns = 0;
  diag_dispatch_gnss_max_abs_error_ns = 0;

  diag_smartpop_checks = 0;
  diag_smartpop_armed = 0;
  diag_smartpop_fallback_no_prediction = 0;
  diag_smartpop_fallback_late_arm = 0;
  diag_smartpop_woke_early = 0;
  diag_smartpop_woke_late = 0;
  diag_smartpop_spin_attempts = 0;
  diag_smartpop_spin_success = 0;
  diag_smartpop_spin_timeouts = 0;
  diag_smartpop_last_target_dwt = 0;
  diag_smartpop_last_entry_dwt = 0;
  diag_smartpop_last_land_dwt = 0;
  diag_smartpop_last_cycles_early = 0;
  diag_smartpop_last_spin_cycles = 0;
  diag_smartpop_max_spin_cycles = 0;
  diag_smartpop_max_cycles_early = 0;

  diag_smartpop_pps_checks = 0;
  diag_smartpop_ocxo1_checks = 0;
  diag_smartpop_ocxo2_checks = 0;
  diag_smartpop_dwt_adjust_checks = 0;
  diag_smartpop_pps_spin_success = 0;
  diag_smartpop_ocxo1_spin_success = 0;
  diag_smartpop_ocxo2_spin_success = 0;
  diag_smartpop_dwt_adjust_spin_success = 0;
  diag_smartpop_pps_spin_timeouts = 0;
  diag_smartpop_ocxo1_spin_timeouts = 0;
  diag_smartpop_ocxo2_spin_timeouts = 0;
  diag_smartpop_dwt_adjust_spin_timeouts = 0;
  diag_smartpop_pps_fallback_no_prediction = 0;
  diag_smartpop_ocxo1_fallback_no_prediction = 0;
  diag_smartpop_ocxo2_fallback_no_prediction = 0;
  diag_smartpop_dwt_adjust_fallback_no_prediction = 0;
  diag_smartpop_pps_fallback_late_arm = 0;
  diag_smartpop_ocxo1_fallback_late_arm = 0;
  diag_smartpop_ocxo2_fallback_late_arm = 0;
  diag_smartpop_dwt_adjust_fallback_late_arm = 0;
  diag_dispatch_gnss_worst_target_ns = -1;
  diag_dispatch_gnss_worst_actual_ns = -1;
  diag_dispatch_gnss_worst_error_ns = 0;
  diag_dispatch_gnss_worst_handle = 0;
  diag_dispatch_gnss_worst_prediction_valid = false;
  diag_dispatch_gnss_worst_smartpop_enabled = false;
  diag_dispatch_gnss_worst_name = nullptr;


  diag_deadline_negative_offset = 0;
  diag_deadline_last_target_gnss_ns = -1;
  diag_deadline_last_anchor_pps_gnss_ns = -1;
  diag_deadline_last_ns_from_anchor = 0;
  diag_named_replacements_pps_prespin = 0;
  diag_named_replacements_ocxo1_prespin = 0;
  diag_named_replacements_ocxo2_prespin = 0;
  diag_named_replacements_pps_dispatch = 0;
  diag_named_replacements_ocxo1_dispatch = 0;
  diag_named_replacements_ocxo2_dispatch = 0;

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
// Arm — relative scheduling ("from now")
// ============================================================================
//
// This is the ONLY arming path that reads the ambient VCLOCK counter,
// because "from now" inherently requires knowing "now."

timepop_handle_t timepop_arm(
  uint64_t            delay_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;

  noInterrupts();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active       = true;
    slots[i].expired      = false;
    slots[i].recurring    = recurring;
    slots[i].is_absolute  = false;
    slots[i].handle       = h;
    slots[i].period_ns    = delay_gnss_ns;
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

    const uint32_t ticks = ns_to_ticks(delay_gnss_ns);
    slots[i].period_ticks = ticks;

    // Relative scheduling: "from now" requires reading the counter.
    const uint32_t now = vclock_count();
    slots[i].deadline = now + ticks;
    slots[i].arm_vclock_raw = now;
    slots[i].arm_delta_ticks = ticks;

    slots[i].predicted_dwt = predict_dwt_at_deadline(
      slots[i].deadline, slots[i].prediction_valid);

    if (slots[i].prediction_valid) {
      slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
    }

    smartpop_prepare_slot(slots[i]);

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

timepop_handle_t timepop_arm_asap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_deferred(asap_slots,
                      MAX_ASAP_SLOTS,
                      diag_asap_arm_failures,
                      diag_asap_armed,
                      diag_asap_last_armed_dwt,
                      diag_asap_last_armed_name,
                      diag_asap_slots_high_water,
                      callback,
                      user_data,
                      name);
}

timepop_handle_t timepop_arm_alap(
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  return arm_deferred(alap_slots,
                      MAX_ALAP_SLOTS,
                      diag_alap_arm_failures,
                      diag_alap_armed,
                      diag_alap_last_armed_dwt,
                      diag_alap_last_armed_name,
                      diag_alap_slots_high_water,
                      callback,
                      user_data,
                      name);
}

// ============================================================================
// Arm — absolute scheduling (deterministic, no ambient "now")
// ============================================================================
//
// Converts the target GNSS nanosecond to a VCLOCK deadline using the time
// anchor via pure arithmetic.  No call to vclock_count().  No call to
// time_gnss_ns_now().  The deadline is geared from the anchor, not measured
// from ambient time.

timepop_handle_t timepop_arm_at(
  int64_t             target_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const time_anchor_snapshot_t snap = time_anchor_snapshot();

  uint32_t deadline = 0;
  if (!gnss_ns_to_vclock_deadline(target_gnss_ns, snap, deadline)) {
    return TIMEPOP_INVALID_HANDLE;
  }

  noInterrupts();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active       = true;
    slots[i].expired      = false;
    slots[i].recurring    = recurring;
    slots[i].is_absolute  = true;
    slots[i].handle       = h;
    slots[i].period_ns    = 0;
    slots[i].period_ticks = 0;
    slots[i].deadline     = deadline;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].callback     = callback;
    slots[i].user_data    = user_data;
    slots[i].name         = name;
    slots[i].fire_gnss_ns = -1;
    slots[i].isr_callback = false;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].arm_vclock_raw = deadline;
    slots[i].arm_delta_ticks = 0;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    slots[i].predicted_dwt = predict_dwt_at_deadline(
      deadline, slots[i].prediction_valid);

    smartpop_prepare_slot(slots[i]);

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
// Arm — anchor-relative GNSS scheduling
// ============================================================================

timepop_handle_t timepop_arm_from_anchor(
  int64_t             anchor_gnss_ns,
  int64_t             offset_gnss_ns,
  bool                recurring,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (anchor_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const int64_t target_gnss_ns = anchor_gnss_ns + offset_gnss_ns;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= anchor_gnss_ns && offset_gnss_ns > 0) {
    return TIMEPOP_INVALID_HANDLE;
  }

  return timepop_arm_at(
    target_gnss_ns,
    recurring,
    callback,
    user_data,
    name
  );
}

// ============================================================================
// Arm — caller-owned exact target path (deterministic, no ambient "now")
// ============================================================================
//
// The caller provides both the GNSS nanosecond target and the DWT prediction.
// The VCLOCK deadline is computed from the time anchor — no ambient read.

timepop_handle_t timepop_arm_ns(
  int64_t             target_gnss_ns,
  uint32_t            target_dwt,
  timepop_callback_t  callback,
  void*               user_data,
  const char*         name,
  bool                isr_callback
) {
  if (!callback) return TIMEPOP_INVALID_HANDLE;
  if (target_gnss_ns <= 0) return TIMEPOP_INVALID_HANDLE;

  const time_anchor_snapshot_t snap = time_anchor_snapshot();

  uint32_t deadline = 0;
  if (!gnss_ns_to_vclock_deadline(target_gnss_ns, snap, deadline)) {
    return TIMEPOP_INVALID_HANDLE;
  }

  noInterrupts();

  const bool retired_named = retire_existing_named_slots(name);
  if (retired_named) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
  }

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i] = {};
    slots[i].active         = true;
    slots[i].expired        = false;
    slots[i].recurring      = false;
    slots[i].is_absolute    = true;
    slots[i].handle         = h;
    slots[i].period_ns      = 0;
    slots[i].period_ticks   = 0;
    slots[i].deadline       = deadline;
    slots[i].target_gnss_ns = target_gnss_ns;
    slots[i].callback       = callback;
    slots[i].user_data      = user_data;
    slots[i].name           = name;
    slots[i].fire_gnss_ns   = -1;
    slots[i].isr_callback   = isr_callback;
    slots[i].recurring_rearmed_count = 0;
    slots[i].recurring_immediate_expire_count = 0;
    slots[i].recurring_catchup_count = 0;
    slots[i].arm_vclock_raw = deadline;
    slots[i].arm_delta_ticks = 0;
    slots[i].first_expire_now = 0;
    slots[i].first_expire_recorded = false;

    if (target_dwt != 0) {
      slots[i].predicted_dwt    = target_dwt;
      slots[i].prediction_valid = true;
    } else {
      slots[i].predicted_dwt = predict_dwt_at_deadline(
        deadline, slots[i].prediction_valid);
      if (slots[i].prediction_valid) {
        slots[i].target_gnss_ns = target_gnss_ns;
      }
    }

    smartpop_prepare_slot(slots[i]);

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
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].handle == handle) {
      asap_slots[i].active = false;
      interrupts();
      return true;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].handle == handle) {
      alap_slots[i].active = false;
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
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active && asap_slots[i].name && strcmp(asap_slots[i].name, name) == 0) {
      asap_slots[i].active = false;
      cancelled++;
    }
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active && alap_slots[i].name && strcmp(alap_slots[i].name, name) == 0) {
      alap_slots[i].active = false;
      cancelled++;
    }
  }
  if (cancelled > 0) {
    diag_schedule_next_calls_from_other++;
    schedule_next();
    timepop_pending = true;
  }
  interrupts();
  return cancelled;
}

// ============================================================================
// Dispatch
// ============================================================================

void timepop_dispatch(void) {
  if (!timepop_pending) return;
  timepop_pending = false;
  diag_dispatch_calls++;

  dispatch_deferred_phase(asap_slots,
                          MAX_ASAP_SLOTS,
                          diag_asap_dispatched,
                          diag_asap_last_dispatch_dwt);

  const uint32_t dispatch_now = vclock_count();
  (void)dispatch_now;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;
    slots[i].expired = false;

    // ── ISR-callback slot: callback already ran in ISR context ──
    if (slots[i].isr_callback_fired) {
      slots[i].active = false;
      continue;
    }

    // ── Timed scheduled-context callback ──
    timepop_ctx_t ctx;
    slot_build_ctx(slots[i], ctx);

    timepop_diag_t diag;
    slot_build_diag(slots[i], 0, diag);

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, &diag, slots[i].user_data);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);
    diag_dispatch_callbacks++;

    // ── Recurring rearm ──
    if (slots[i].recurring) {
      if (slots[i].is_absolute) {
        // Absolute timers are one-shot by nature — the caller reschedules
        // from the just-consummated event with fresh anchor arithmetic.
        slots[i].active = false;
      } else if (slots[i].period_ticks == 0) {
        slots[i].active = false;
      } else {
        // Relative recurring: advance deadline by one period.
        slots[i].deadline += slots[i].period_ticks;
        slots[i].recurring_rearmed_count++;

        // Catch-up: if we fell behind, skip forward.
        uint32_t now = vclock_count();
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].recurring_immediate_expire_count++;
          slots[i].deadline = now + slots[i].period_ticks;
          slots[i].recurring_catchup_count++;
        }

        slots[i].predicted_dwt = predict_dwt_at_deadline(
          slots[i].deadline, slots[i].prediction_valid);

        if (slots[i].prediction_valid) {
          slots[i].target_gnss_ns = time_dwt_to_gnss_ns(slots[i].predicted_dwt);
        }

        smartpop_prepare_slot(slots[i]);

        slots[i].expired = false;
        noInterrupts();
        schedule_next();
        interrupts();
      }
    } else {
      slots[i].active = false;
    }
  }

  dispatch_deferred_phase(alap_slots,
                          MAX_ALAP_SLOTS,
                          diag_alap_dispatched,
                          diag_alap_last_dispatch_dwt);

  bool more_pending = false;
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active) { more_pending = true; break; }
  }
  if (!more_pending) {
    for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
      if (alap_slots[i].active) { more_pending = true; break; }
    }
  }
  if (more_pending) {
    timepop_pending = true;
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
  for (uint32_t i = 0; i < MAX_ASAP_SLOTS; i++) {
    if (asap_slots[i].active) n++;
  }
  for (uint32_t i = 0; i < MAX_ALAP_SLOTS; i++) {
    if (alap_slots[i].active) n++;
  }
  return n;
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
  out.add("time_valid",        time_valid());
  out.add("time_pps_count",    time_pps_count());

  // ── DWT-domain dispatch validation ──
  out.add("dispatch_gnss_tolerance_ns",         (int64_t)TIMEPOP_DISPATCH_GNSS_TOLERANCE_NS);
  out.add("dispatch_gnss_checks",               diag_dispatch_gnss_checks);
  out.add("dispatch_gnss_mismatches",           diag_dispatch_gnss_mismatches);
  out.add("dispatch_gnss_skipped_no_target",    diag_dispatch_gnss_skipped_no_target);
  out.add("dispatch_gnss_skipped_invalid_time", diag_dispatch_gnss_skipped_invalid_time);
  out.add("dispatch_gnss_skipped_unmonitored", diag_dispatch_gnss_skipped_unmonitored);
  out.add("dispatch_gnss_last_target_ns",       diag_dispatch_gnss_last_target_ns);
  out.add("dispatch_gnss_last_actual_ns",       diag_dispatch_gnss_last_actual_ns);
  out.add("dispatch_gnss_last_error_ns",        diag_dispatch_gnss_last_error_ns);
  out.add("dispatch_gnss_max_abs_error_ns",     diag_dispatch_gnss_max_abs_error_ns);

  out.add("smartpop_early_wake_ns",             (uint64_t)SMARTPOP_EARLY_WAKE_NS);
  out.add("smartpop_spin_max_cycles",           (uint32_t)SMARTPOP_SPIN_MAX_CYCLES);
  out.add("smartpop_min_margin_cycles",         (uint32_t)SMARTPOP_MIN_MARGIN_CYCLES);
  out.add("smartpop_checks",                    diag_smartpop_checks);
  out.add("smartpop_armed",                     diag_smartpop_armed);
  out.add("smartpop_fallback_no_prediction",    diag_smartpop_fallback_no_prediction);
  out.add("smartpop_fallback_late_arm",         diag_smartpop_fallback_late_arm);
  out.add("smartpop_woke_early",                diag_smartpop_woke_early);
  out.add("smartpop_woke_late",                 diag_smartpop_woke_late);
  out.add("smartpop_spin_attempts",             diag_smartpop_spin_attempts);
  out.add("smartpop_spin_success",              diag_smartpop_spin_success);
  out.add("smartpop_spin_timeouts",             diag_smartpop_spin_timeouts);
  out.add("smartpop_last_target_dwt",           diag_smartpop_last_target_dwt);
  out.add("smartpop_last_entry_dwt",            diag_smartpop_last_entry_dwt);
  out.add("smartpop_last_land_dwt",             diag_smartpop_last_land_dwt);
  out.add("smartpop_last_cycles_early",         diag_smartpop_last_cycles_early);
  out.add("smartpop_last_spin_cycles",          diag_smartpop_last_spin_cycles);
  out.add("smartpop_max_spin_cycles",           diag_smartpop_max_spin_cycles);
  out.add("smartpop_max_cycles_early",          diag_smartpop_max_cycles_early);

  out.add("smartpop_pps_checks",                    diag_smartpop_pps_checks);
  out.add("smartpop_ocxo1_checks",                  diag_smartpop_ocxo1_checks);
  out.add("smartpop_ocxo2_checks",                  diag_smartpop_ocxo2_checks);
  out.add("smartpop_pps_spin_success",              diag_smartpop_pps_spin_success);
  out.add("smartpop_ocxo1_spin_success",            diag_smartpop_ocxo1_spin_success);
  out.add("smartpop_ocxo2_spin_success",            diag_smartpop_ocxo2_spin_success);
  out.add("smartpop_pps_spin_timeouts",             diag_smartpop_pps_spin_timeouts);
  out.add("smartpop_ocxo1_spin_timeouts",           diag_smartpop_ocxo1_spin_timeouts);
  out.add("smartpop_ocxo2_spin_timeouts",           diag_smartpop_ocxo2_spin_timeouts);
  out.add("smartpop_pps_fallback_no_prediction",    diag_smartpop_pps_fallback_no_prediction);
  out.add("smartpop_ocxo1_fallback_no_prediction",  diag_smartpop_ocxo1_fallback_no_prediction);
  out.add("smartpop_ocxo2_fallback_no_prediction",  diag_smartpop_ocxo2_fallback_no_prediction);
  out.add("smartpop_pps_fallback_late_arm",         diag_smartpop_pps_fallback_late_arm);
  out.add("smartpop_ocxo1_fallback_late_arm",       diag_smartpop_ocxo1_fallback_late_arm);
  out.add("smartpop_ocxo2_fallback_late_arm",       diag_smartpop_ocxo2_fallback_late_arm);
  out.add("dispatch_gnss_worst_target_ns",          diag_dispatch_gnss_worst_target_ns);
  out.add("dispatch_gnss_worst_actual_ns",          diag_dispatch_gnss_worst_actual_ns);
  out.add("dispatch_gnss_worst_error_ns",           diag_dispatch_gnss_worst_error_ns);
  out.add("dispatch_gnss_worst_handle",             diag_dispatch_gnss_worst_handle);
  out.add("dispatch_gnss_worst_prediction_valid",   diag_dispatch_gnss_worst_prediction_valid);
  out.add("dispatch_gnss_worst_smartpop_enabled",   diag_dispatch_gnss_worst_smartpop_enabled);
  out.add("dispatch_gnss_worst_name",               (const char*)(diag_dispatch_gnss_worst_name ? diag_dispatch_gnss_worst_name : ""));


  out.add("slots_active_now",    timepop_active_count());
  out.add("slots_high_water",     diag_slots_high_water);
  out.add("slots_max",            (uint32_t)MAX_SLOTS);
  out.add("asap_slots_high_water", diag_asap_slots_high_water);
  out.add("asap_slots_max",       (uint32_t)MAX_ASAP_SLOTS);
  out.add("alap_slots_high_water", diag_alap_slots_high_water);
  out.add("alap_slots_max",       (uint32_t)MAX_ALAP_SLOTS);
  out.add("arm_failures",         diag_arm_failures);
  out.add("named_replacements",   diag_named_replacements);
  out.add("named_replacements_pps_prespin", diag_named_replacements_pps_prespin);
  out.add("named_replacements_ocxo1_prespin", diag_named_replacements_ocxo1_prespin);
  out.add("named_replacements_ocxo2_prespin", diag_named_replacements_ocxo2_prespin);
  out.add("named_replacements_pps_dispatch", diag_named_replacements_pps_dispatch);
  out.add("named_replacements_ocxo1_dispatch", diag_named_replacements_ocxo1_dispatch);
  out.add("named_replacements_ocxo2_dispatch", diag_named_replacements_ocxo2_dispatch);

  out.add("deadline_negative_offset", diag_deadline_negative_offset);
  out.add("deadline_last_target_gnss_ns", diag_deadline_last_target_gnss_ns);
  out.add("deadline_last_anchor_pps_gnss_ns", diag_deadline_last_anchor_pps_gnss_ns);
  out.add("deadline_last_ns_from_anchor", diag_deadline_last_ns_from_anchor);

  out.add("asap_armed",             diag_asap_armed);
  out.add("asap_dispatched",        diag_asap_dispatched);
  out.add("asap_arm_failures",      diag_asap_arm_failures);
  out.add("asap_last_armed_dwt",    diag_asap_last_armed_dwt);
  out.add("asap_last_dispatch_dwt", diag_asap_last_dispatch_dwt);
  out.add("asap_last_armed_name", (const char*)(diag_asap_last_armed_name ?
    diag_asap_last_armed_name : ""));
  out.add("alap_armed",             diag_alap_armed);
  out.add("alap_dispatched",        diag_alap_dispatched);
  out.add("alap_arm_failures",      diag_alap_arm_failures);
  out.add("alap_last_armed_dwt",    diag_alap_last_armed_dwt);
  out.add("alap_last_dispatch_dwt", diag_alap_last_dispatch_dwt);
  out.add("alap_last_armed_name", (const char*)(diag_alap_last_armed_name ?
    diag_alap_last_armed_name : ""));

  out.add("dispatch_calls",     diag_dispatch_calls);
  out.add("dispatch_callbacks", diag_dispatch_callbacks);

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
    entry.add("target_deadline",  slots[i].target_deadline);
    entry.add("smartpop_enabled", slots[i].smartpop_enabled);
    entry.add("smartpop_early_ticks", slots[i].smartpop_early_ticks);
    entry.add("recurring", slots[i].recurring);
    entry.add("expired",   slots[i].expired);
    entry.add("isr_cb",    slots[i].isr_callback);

    entry.add("is_absolute",      slots[i].is_absolute);
    entry.add("target_gnss_ns",   slots[i].target_gnss_ns);
    entry.add("predicted_dwt",    slots[i].predicted_dwt);
    entry.add("prediction_valid", slots[i].prediction_valid);

    entry.add("recurring_rearmed_count",          slots[i].recurring_rearmed_count);
    entry.add("recurring_immediate_expire_count", slots[i].recurring_immediate_expire_count);
    entry.add("recurring_catchup_count",          slots[i].recurring_catchup_count);

    timers.add(entry);
  }
  out.add_array("timers", timers);

  return out;
}

// ============================================================================
// QTimer1 ISR
// ============================================================================

static void qtimer1_irq_isr(void) {
  const uint32_t dwt_raw = ARM_DWT_CYCCNT;           // FIRST instruction
  if (IMXRT_TMR1.CH[2].CSCTRL & TMR_CSCTRL_TCF1) {
    qtimer1_ch2_isr(dwt_raw);                          // pass it down
  }
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT",      cmd_report      },
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