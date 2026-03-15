// ============================================================================
// process_timepop.cpp — TimePop v4.3 (Forensic Diagnostics)
// ============================================================================
//
// The time experience in ZPNet is the GNSS clock.
//
// v4.3: Comprehensive forensic diagnostics.
//
//   Following a production stall where a PPS ASAP callback was lost
//   with no trace, v4.3 adds slot-level and ASAP-specific telemetry
//   to make such failures visible and diagnosable.
//
//   New diagnostics:
//
//   Slot pressure:
//     slots_active_now       — current active slot count (snapshot)
//     slots_high_water       — lifetime high-water mark of active slots
//     slots_max              — compile-time MAX_SLOTS (for context)
//     arm_failures           — total timepop_arm/arm_ns INVALID_HANDLE
//
//   ASAP tracking:
//     asap_armed             — ASAP callbacks successfully armed
//     asap_dispatched        — ASAP callbacks successfully dispatched
//     asap_arm_failures      — ASAP arm attempts that returned
//                              INVALID_HANDLE (slot exhaustion)
//     asap_last_armed_dwt    — DWT_CYCCNT at last successful ASAP arm
//     asap_last_dispatch_dwt — DWT_CYCCNT at last successful ASAP dispatch
//
//   Dispatch health:
//     dispatch_calls         — total timepop_dispatch() invocations
//     dispatch_callbacks     — total callbacks fired from dispatch
//
//   The ASAP armed-vs-dispatched delta is the critical signal:
//   if armed > dispatched, an ASAP callback was lost between arm
//   and dispatch.
//
// v4.2: Caller-owns-target for nano-precise timers.
//
//   timepop_arm_ns() now takes the pre-computed target GNSS nanosecond
//   and DWT cycle count directly from the caller.  No internal
//   time_gnss_ns_now() or time_gnss_ns_to_dwt() — the target in the
//   slot is exactly what the caller computed.  This eliminates the
//   ~470 ns arm-path bias caused by the double computation in v4/v4.1.
//
// v4.1: NS_TEST full forensics.
// v4: DWT nano-spin ISR delivery.
// v3.1: INTERP_TEST Path 2 epoch fix.
// v3: TEST and INTERP_TEST use time.h.  Campaign-independent.
//
// ============================================================================

#include "timepop.h"
#include "process_timepop.h"

#include "debug.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "timebase.h"
#include "time.h"

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
  uint64_t            frag_gnss_ns;
  uint64_t            frag_dwt_cycles;
  uint32_t            frag_dwt_cyccnt_at_pps;
  uint32_t            frag_dwt_cycles_per_pps;
  uint32_t            frag_gpt2_at_pps;
  bool                frag_valid;

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
//
// ASAP callbacks are one-shot timers armed with delay_ns == 0.
// They are immediately marked expired and dispatched on the next
// timepop_dispatch() call.  The armed-vs-dispatched delta reveals
// lost callbacks.
static volatile uint32_t diag_asap_armed             = 0;
static volatile uint32_t diag_asap_dispatched         = 0;
static volatile uint32_t diag_asap_arm_failures       = 0;
static volatile uint32_t diag_asap_last_armed_dwt     = 0;
static volatile uint32_t diag_asap_last_dispatch_dwt  = 0;
static volatile const char* diag_asap_last_armed_name = nullptr;

// ── v4.3: Dispatch health ──
static volatile uint32_t diag_dispatch_calls     = 0;
static volatile uint32_t diag_dispatch_callbacks = 0;

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
// GPT2 Output Compare ISR
// ============================================================================

static void gpt2_compare_isr(void) {

  const uint32_t now     = GPT2_CNT;
  const uint32_t dwt_now = ARM_DWT_CYCCNT;

  const timebase_fragment_t* frag = timebase_last_fragment();
  const bool frag_ok = frag && frag->valid;

  GPT2_SR = GPT_SR_OF1;
  isr_fire_count++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;
    if (!deadline_expired(slots[i].deadline, now)) continue;

    // ── Nano-precise: DWT spin to exact target cycle ──
    if (slots[i].nano_precise) {
      const uint32_t target_dwt = slots[i].target_dwt;
      uint32_t spin_start = ARM_DWT_CYCCNT;
      bool timed_out = false;

      while ((int32_t)(target_dwt - ARM_DWT_CYCCNT) > 0) {
        if ((ARM_DWT_CYCCNT - spin_start) > NANO_SPIN_MAX_CYCLES) {
          timed_out = true;
          nano_timeout_count++;
          break;
        }
      }

      const uint32_t landed_dwt = ARM_DWT_CYCCNT;

      slots[i].fire_gnss_ns   = time_dwt_to_gnss_ns(landed_dwt);
      slots[i].fire_dwt_cyccnt = landed_dwt;
      slots[i].fire_gpt2      = GPT2_CNT;
      slots[i].nano_timeout   = timed_out;

      if (!timed_out) nano_spin_count++;

      if (frag_ok) {
        slots[i].frag_gnss_ns            = frag->gnss_ns;
        slots[i].frag_dwt_cycles         = frag->dwt_cycles;
        slots[i].frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
        slots[i].frag_dwt_cycles_per_pps = frag->dwt_cycles_per_pps;
        slots[i].frag_gpt2_at_pps        = frag->gpt2_at_pps;
        slots[i].frag_valid              = true;
      } else {
        slots[i].frag_valid = false;
      }

      timepop_ctx_t ctx;
      ctx.handle       = slots[i].handle;
      ctx.fire_gpt2    = slots[i].fire_gpt2;
      ctx.deadline     = slots[i].deadline;
      ctx.fire_ns      = (int32_t)(slots[i].fire_gpt2 - slots[i].deadline) * 100;
      ctx.fire_gnss_ns = slots[i].fire_gnss_ns;
      ctx.nano_precise = true;
      ctx.nano_timeout = slots[i].nano_timeout;

      ctx.fire_dwt_cyccnt         = slots[i].fire_dwt_cyccnt;
      ctx.frag_gnss_ns            = slots[i].frag_gnss_ns;
      ctx.frag_dwt_cycles         = slots[i].frag_dwt_cycles;
      ctx.frag_dwt_cyccnt_at_pps  = slots[i].frag_dwt_cyccnt_at_pps;
      ctx.frag_dwt_cycles_per_pps = slots[i].frag_dwt_cycles_per_pps;
      ctx.frag_gpt2_at_pps        = slots[i].frag_gpt2_at_pps;
      ctx.frag_valid              = slots[i].frag_valid;

      slots[i].callback(&ctx, slots[i].user_data);

      slots[i].active  = false;
      slots[i].expired = false;
      expired_count++;
      continue;
    }

    // ── Standard: mark expired, defer to dispatch ──
    slots[i].expired        = true;
    slots[i].fire_gpt2      = now;
    slots[i].fire_dwt_cyccnt = dwt_now;

    if (frag_ok) {
      slots[i].frag_gnss_ns            = frag->gnss_ns;
      slots[i].frag_dwt_cycles         = frag->dwt_cycles;
      slots[i].frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
      slots[i].frag_dwt_cycles_per_pps = frag->dwt_cycles_per_pps;
      slots[i].frag_gpt2_at_pps        = frag->gpt2_at_pps;
      slots[i].frag_valid              = true;
    } else {
      slots[i].frag_valid = false;
    }

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
      timepop_pending = true;
      diag_asap_armed++;
      diag_asap_last_armed_dwt  = ARM_DWT_CYCCNT;
      diag_asap_last_armed_name = name;
    } else {
      slots[i].deadline = gpt2_count() + ticks;
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

  // Compute GPT2 deadline from the delay between now and target.
  // time_gnss_ns_now() is called here ONLY to derive the GPT2 offset —
  // it does NOT affect the DWT target (which the caller already computed).
  int64_t now_ns = time_gnss_ns_now();
  if (now_ns < 0) return TIMEPOP_INVALID_HANDLE;

  int64_t delay_ns = target_gnss_ns - now_ns;
  if (delay_ns <= 0) return TIMEPOP_INVALID_HANDLE;  // target already passed

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
// Dispatch (scheduled context — standard timers only)
// ============================================================================

void timepop_dispatch(void) {

  emit_deferred_ns_test();

  if (!timepop_pending) return;
  timepop_pending = false;

  diag_dispatch_calls++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || !slots[i].expired) continue;
    if (slots[i].nano_precise) continue;

    slots[i].expired = false;

    // Track ASAP dispatch: ASAP callbacks have period_ns == 0 and
    // are non-recurring one-shots.  Their deadline is 0 (set at arm).
    const bool is_asap = (slots[i].period_ns == 0 && !slots[i].recurring);

    timepop_ctx_t ctx;
    ctx.handle       = slots[i].handle;
    ctx.fire_gpt2    = slots[i].fire_gpt2;
    ctx.deadline     = slots[i].deadline;
    ctx.fire_ns      = (int32_t)(slots[i].fire_gpt2 - slots[i].deadline) * 100;
    ctx.fire_gnss_ns = -1;
    ctx.nano_precise = false;
    ctx.nano_timeout = false;

    ctx.fire_dwt_cyccnt         = slots[i].fire_dwt_cyccnt;
    ctx.frag_gnss_ns            = slots[i].frag_gnss_ns;
    ctx.frag_dwt_cycles         = slots[i].frag_dwt_cycles;
    ctx.frag_dwt_cyccnt_at_pps  = slots[i].frag_dwt_cyccnt_at_pps;
    ctx.frag_dwt_cycles_per_pps = slots[i].frag_dwt_cycles_per_pps;
    ctx.frag_gpt2_at_pps        = slots[i].frag_gpt2_at_pps;
    ctx.frag_valid              = slots[i].frag_valid;

    uint32_t start = ARM_DWT_CYCCNT;
    slots[i].callback(&ctx, slots[i].user_data);
    uint32_t end   = ARM_DWT_CYCCNT;

    cpu_usage_account_busy(end - start);

    diag_dispatch_callbacks++;

    if (is_asap) {
      diag_asap_dispatched++;
      diag_asap_last_dispatch_dwt = end;
    }

    if (slots[i].active && slots[i].recurring) {
      if (slots[i].period_ticks == 0) {
        slots[i].active = false;
      } else {
        slots[i].deadline += slots[i].period_ticks;
        uint32_t now = gpt2_count();
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].deadline = now + slots[i].period_ticks;
        }
        noInterrupts();
        reload_ocr();
        interrupts();
      }
    } else if (slots[i].active) {
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

  // DWT spin precision: how close did the spin land to the target?
  int32_t spin_err = (int32_t)(ctx->fire_dwt_cyccnt - ns_test_ctx.target_dwt);

  ns_test_result_ev.clear();

  // Arm-side
  ns_test_result_ev.add("arm_ns",    ns_test_ctx.arm_gnss_ns);
  ns_test_result_ev.add("tgt_ns",    ns_test_ctx.target_gnss_ns);
  ns_test_result_ev.add("tgt_dwt",   ns_test_ctx.target_dwt);
  ns_test_result_ev.add("arm_dwt",   ns_test_ctx.arm_dwt);
  ns_test_result_ev.add("arm_gpt2",  ns_test_ctx.arm_gpt2);

  // Fire-side
  ns_test_result_ev.add("f_ns",      ctx->fire_gnss_ns);
  ns_test_result_ev.add("f_dwt",     ctx->fire_dwt_cyccnt);
  ns_test_result_ev.add("f_gpt2",    ctx->fire_gpt2);
  ns_test_result_ev.add("ocr_ns",    ctx->fire_ns);
  ns_test_result_ev.add("timeout",   ctx->nano_timeout);

  // Deltas and precision
  ns_test_result_ev.add("d_dwt",     dwt_elapsed);
  ns_test_result_ev.add("d_gpt2",    gpt2_elapsed);
  ns_test_result_ev.add("spin_err",  spin_err);

  // Derived
  ns_test_result_ev.add("err",       error_ns);
  ns_test_result_ev.add("req",       ns_test_ctx.requested_ns);

  ns_test_result_ready = true;
  ns_test_in_flight = false;
}

static Payload cmd_ns_test(const Payload& args) {
  Payload resp;

  if (ns_test_in_flight) { resp.add("error", "ns_test already in flight"); return resp; }
  if (!args.has("ns")) { resp.add("error", "specify ns=<nanoseconds>"); return resp; }

  uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val == 0) { resp.add("error", "ns must be > 0"); return resp; }

  if (!time_valid()) { resp.add("error", "time not valid (need PPS lock)"); return resp; }

  // ── Single-point capture: arm time, target, and DWT target ──
  // All computed from one time_gnss_ns_now() call — no recomputation.
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

  // ── Arm with pre-computed target — no recomputation inside arm_ns ──
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
// INTERP_TEST — dual-path validation (unchanged from v3.1)
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

  bool path1_valid = false;
  int64_t path1_error_ns = 0;
  uint64_t vclock_ns_into_second = 0;

  if (ctx->frag_valid && ctx->frag_dwt_cycles_per_pps > 0) {
    const uint32_t gpt2_since_pps = ctx->deadline - ctx->frag_gpt2_at_pps;
    vclock_ns_into_second = (uint64_t)gpt2_since_pps * 100ULL;
    const int64_t vclock_gnss_ns = (int64_t)(ctx->frag_gnss_ns + vclock_ns_into_second);
    const uint32_t synth_dwt_elapsed =
      (uint32_t)((vclock_ns_into_second * (uint64_t)ctx->frag_dwt_cycles_per_pps + 500000000ULL) / 1000000000ULL);
    const uint32_t synth_dwt = ctx->frag_dwt_cyccnt_at_pps + synth_dwt_elapsed;
    const int64_t interp_gnss_ns = timebase_gnss_ns_from_dwt(
      synth_dwt, ctx->frag_gnss_ns, ctx->frag_dwt_cyccnt_at_pps, ctx->frag_dwt_cycles_per_pps);
    path1_error_ns = (interp_gnss_ns >= 0) ? interp_gnss_ns - vclock_gnss_ns : 0;
    path1_valid = true;
  }

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
    timers.add(entry);
  }
  out.add_array("timers", timers);
  return out;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t TIMEPOP_COMMANDS[] = {
  { "REPORT",      cmd_report      },
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