// ============================================================================
// process_timepop.cpp — TimePop v3.1 (GPT2 Output Compare + time.h)
// ============================================================================
//
// The time experience in ZPNet is the GNSS clock.
//
// GPT2 free-runs at 10 MHz clocked by the GF-8802 GNSS VCLOCK.
// TimePop uses Output Compare channel 1 (OCR1) to fire an interrupt
// at precise GPT2 count values.
//
// When the ISR fires, it captures GPT2_CNT immediately.  This
// timestamp is stored in the slot and passed to the callback via
// timepop_ctx_t.  Every callback knows the exact GNSS tick at which
// it was triggered — independent of dispatch latency.
//
// No PIT.  No clock domains.  No drift correction.
//
// v3.1: Fixed INTERP_TEST Path 2 epoch mismatch.
//
//   time_gnss_ns_now() counts from first-PPS-after-boot.
//   TIMEBASE fragment gnss_ns counts from campaign start.
//   These are different epochs — comparing them directly produces
//   a systematic offset equal to the boot-to-campaign-start gap.
//
//   Fix: Path 2 now captures time_gnss_ns_now() at arm time AND
//   at callback dispatch time.  The delta is compared against the
//   VCLOCK delta (GPT2 ticks * 100 ns).  This is a pure rate
//   comparison with no epoch dependency.
//
// v3: TEST and INTERP_TEST commands use time_gnss_ns_now() from
//     time.h instead of timebase_now_gnss_ns().  Campaign-independent.
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

// 1 GPT2 tick = 100 ns
static constexpr uint64_t NS_PER_TICK = 100ULL;

// Maximum delay: half the 32-bit range (~214 seconds)
static constexpr uint32_t MAX_DELAY_TICKS = 0x7FFFFFFFU;

// Minimum delay: avoid setting OCR in the immediate past
static constexpr uint32_t MIN_DELAY_TICKS = 2;

// ============================================================================
// Slot
// ============================================================================

struct timepop_slot_t {
  bool                active;
  bool                expired;
  bool                recurring;

  timepop_handle_t    handle;
  uint32_t            deadline;       // GPT2 count at intended expiry
  uint32_t            fire_gpt2;      // GPT2_CNT captured in ISR at expiry
  uint32_t            period_ticks;   // reload value for recurring
  uint64_t            period_ns;      // original delay (for diagnostics)

  // DWT + fragment snapshot (captured in ISR alongside fire_gpt2)
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

static volatile uint32_t isr_fire_count  = 0;
static volatile uint32_t expired_count   = 0;

// ============================================================================
// GPT2 helpers
// ============================================================================

static inline uint32_t gpt2_count(void) {
  return GPT2_CNT;
}

// Is deadline in the "past" half of the 32-bit range?
static inline bool deadline_expired(uint32_t deadline, uint32_t now) {
  return (deadline - now) > MAX_DELAY_TICKS;
}

// Is `a` sooner than `b` relative to `now`?
static inline bool deadline_sooner(uint32_t a, uint32_t b, uint32_t now) {
  return (a - now) < (b - now);
}

// ============================================================================
// Nanoseconds to GPT2 ticks
// ============================================================================

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
  GPT2_SR   = GPT_SR_OF1;          // clear any pending flag
  GPT2_IR  |= GPT_IR_OF1IE;        // enable compare interrupt
}

static void ocr1_disable(void) {
  GPT2_IR &= ~GPT_IR_OF1IE;        // disable compare interrupt
  GPT2_SR  = GPT_SR_OF1;           // clear flag
}

// Reload OCR1 with the nearest active, non-expired deadline.
// Call with interrupts disabled.
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
    // If already past, nudge forward to fire ASAP
    if (deadline_expired(nearest, now)) {
      nearest = now + MIN_DELAY_TICKS;
    }
    ocr1_set(nearest);
  } else {
    ocr1_disable();
  }
}

// ============================================================================
// GPT2 Output Compare ISR
// ============================================================================

static void gpt2_compare_isr(void) {

  // Capture GPT2_CNT and DWT_CYCCNT simultaneously — these are the
  // authoritative "when did the ISR fire?" timestamps.  GPT2 gives
  // GNSS ticks, DWT gives CPU cycles.  Both captured within 1-2
  // instructions of each other.
  const uint32_t now     = GPT2_CNT;
  const uint32_t dwt_now = ARM_DWT_CYCCNT;

  // Snapshot the current timebase fragment (read once, use for all slots)
  const timebase_fragment_t* frag = timebase_last_fragment();
  const bool frag_ok = frag && frag->valid;

  GPT2_SR = GPT_SR_OF1;
  isr_fire_count++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    if (deadline_expired(slots[i].deadline, now)) {
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
  }

  reload_ocr();

  timepop_pending = true;
}

// ============================================================================
// Initialization
// ============================================================================

void timepop_init(void) {

  // Clear all slots
  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    slots[i] = {};
  }

  // Disable output compare 1 and clear stale flags
  GPT2_IR &= ~(GPT_IR_OF1IE | GPT_IR_OF2IE | GPT_IR_OF3IE);
  GPT2_SR  = GPT_SR_OF1 | GPT_SR_OF2 | GPT_SR_OF3;

  // Install ISR — priority 16 (below PPS at 0)
  attachInterruptVector(IRQ_GPT2, gpt2_compare_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, 16);
  NVIC_ENABLE_IRQ(IRQ_GPT2);
}

// ============================================================================
// Arm
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

  uint32_t ticks = 0;
  if (!is_asap) {
    ticks = ns_to_ticks(delay_ns);
  }

  noInterrupts();

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (slots[i].active) continue;

    timepop_handle_t h = next_handle++;
    if (next_handle == TIMEPOP_INVALID_HANDLE) next_handle = 1;

    slots[i].active       = true;
    slots[i].expired      = is_asap;
    slots[i].recurring    = recurring;
    slots[i].handle       = h;
    slots[i].period_ns    = delay_ns;
    slots[i].period_ticks = ticks;
    slots[i].callback     = callback;
    slots[i].user_data    = user_data;
    slots[i].name         = name;
    slots[i].fire_gpt2    = 0;

    if (is_asap) {
      slots[i].deadline = 0;
      timepop_pending = true;
    } else {
      slots[i].deadline = gpt2_count() + ticks;
      reload_ocr();
    }

    interrupts();
    return h;
  }

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
// Dispatch (scheduled context)
// ============================================================================

void timepop_dispatch(void) {

  if (!timepop_pending) return;
  timepop_pending = false;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {

    if (!slots[i].active || !slots[i].expired) continue;

    slots[i].expired = false;

    // Build context with ISR-captured fire time + DWT/GNSS snapshot
    timepop_ctx_t ctx;
    ctx.handle    = slots[i].handle;
    ctx.fire_gpt2 = slots[i].fire_gpt2;
    ctx.deadline  = slots[i].deadline;
    ctx.fire_ns   = (int32_t)(slots[i].fire_gpt2 - slots[i].deadline) * 100;

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

    if (slots[i].active && slots[i].recurring) {

      if (slots[i].period_ticks == 0) {
        // Recurring ASAP is nonsensical — retire
        slots[i].active = false;
      } else {
        // Re-arm from previous deadline to avoid cumulative
        // dispatch latency drift
        slots[i].deadline += slots[i].period_ticks;

        // If the new deadline is already past (callback was slow),
        // reset to now + period
        uint32_t now = gpt2_count();
        if (deadline_expired(slots[i].deadline, now)) {
          slots[i].deadline = now + slots[i].period_ticks;
        }

        noInterrupts();
        reload_ocr();
        interrupts();
      }

    } else if (slots[i].active) {
      // One-shot: retire
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
// TEST command — validates TimePop delay accuracy via time.h
// ============================================================================
//
// v3: Uses time_gnss_ns_now() instead of timebase_now_gnss_ns().
//     Works without a running campaign — only needs PPS lock.
//

struct test_context_t {
  int64_t  arm_gnss_ns;       // time_gnss_ns_now() at arm
  uint64_t requested_ns;      // requested delay
  uint32_t arm_pps_count;     // time_pps_count() at arm
};

static test_context_t test_ctx = {};
static volatile bool  test_in_flight = false;

static void test_timer_callback(timepop_ctx_t* ctx, void*) {

  // Capture GPT2 at callback entry (for dispatch latency measurement)
  uint32_t cb_gpt2 = GPT2_CNT;

  // v3: Use time.h for GNSS nanosecond at callback entry
  int64_t end_gnss_ns = time_gnss_ns_now();

  int64_t measured_ns = -1;
  if (test_ctx.arm_gnss_ns >= 0 && end_gnss_ns >= 0) {
    measured_ns = end_gnss_ns - test_ctx.arm_gnss_ns;
  }

  int64_t error_ns = -1;
  if (measured_ns >= 0) {
    error_ns = measured_ns - (int64_t)test_ctx.requested_ns;
  }

  // Dispatch latency: how long between ISR fire and callback entry
  int32_t dispatch_ns = (int32_t)(cb_gpt2 - ctx->fire_gpt2) * 100;

  // PPS crossings during the delay
  uint32_t end_pps = time_pps_count();

  Payload ev;
  ev.add("err",         error_ns);
  ev.add("meas",        measured_ns);
  ev.add("req",         test_ctx.requested_ns);
  ev.add("fire_ns",     ctx->fire_ns);       // OCR precision: ISR vs deadline
  ev.add("dispatch_ns", dispatch_ns);         // dispatch latency: callback vs ISR
  ev.add("arm_pps",     test_ctx.arm_pps_count);
  ev.add("end_pps",     end_pps);
  ev.add("pps_x",       (int32_t)(end_pps - test_ctx.arm_pps_count));
  ev.add("time_valid",  time_valid());

  enqueueEvent("TIMEPOP_TEST", ev);

  test_in_flight = false;
}

static Payload cmd_test(const Payload& args) {

  Payload resp;

  if (test_in_flight) {
    resp.add("error", "test already in flight");
    return resp;
  }

  if (!args.has("ns")) {
    resp.add("error", "specify ns=<nanoseconds>");
    return resp;
  }

  uint64_t ns_val = args.getUInt64("ns", 0);
  if (ns_val == 0) {
    resp.add("error", "ns must be > 0");
    return resp;
  }

  // v3: Use time.h instead of timebase
  int64_t arm_gnss = time_gnss_ns_now();
  if (arm_gnss < 0) {
    resp.add("error", "time not valid (need PPS lock)");
    return resp;
  }

  test_ctx = {};
  test_ctx.arm_gnss_ns   = arm_gnss;
  test_ctx.requested_ns  = ns_val;
  test_ctx.arm_pps_count = time_pps_count();

  timepop_handle_t h = timepop_arm(
    ns_val, false,
    test_timer_callback, nullptr, "test"
  );

  if (h == TIMEPOP_INVALID_HANDLE) {
    resp.add("error", "arm failed");
    return resp;
  }

  test_in_flight = true;

  resp.add("status", "armed");
  resp.add("req",    ns_val);

  return resp;
}

// ============================================================================
// INTERP_TEST — Dual-path interpolation validation via TimePop
// ============================================================================
//
// Path 1 (math round-trip, requires campaign):
//
//   VCLOCK truth → synthesized DWT → timebase_gnss_ns_from_dwt()
//   Validates the interpolation MATH with zero real-world noise.
//   Round-trip error should be exactly 0 ns.
//
// Path 2 (time.h rate accuracy, campaign-independent):
//
//   Captures time_gnss_ns_now() at arm time and at callback dispatch
//   time.  The delta is compared against the VCLOCK delta (GPT2 ticks
//   between arm and fire, multiplied by 100 ns/tick).
//
//   This is a pure rate comparison — no epoch dependency.  The error
//   reflects real DWT interpolation accuracy plus dispatch latency.
//
// v3.1: Path 2 fixed.  Previous version compared time_gnss_ns_now()
//   (boot-relative) against frag_gnss_ns + offset (campaign-relative),
//   producing a systematic offset equal to the boot-to-campaign gap.
//
// ============================================================================

struct interp_test_ctx_t {
  uint32_t tests_remaining;
  uint64_t delay_ns;

  // Path 2 arm-time captures (for delta comparison)
  int64_t  arm_time_ns;       // time_gnss_ns_now() at arm
  uint32_t arm_gpt2;          // GPT2_CNT at arm

  // Path 1: math round-trip (Welford)
  int64_t  w_n;
  double   w_mean;
  double   w_m2;
  int64_t  err_min;
  int64_t  err_max;
  int32_t  outside_100ns;

  // Path 2: time.h rate accuracy (Welford)
  int64_t  t_n;
  double   t_mean;
  double   t_m2;
  int64_t  t_err_min;
  int64_t  t_err_max;
};

static interp_test_ctx_t itest_ctx = {};
static volatile bool     itest_in_flight = false;

static void interp_test_callback(timepop_ctx_t* ctx, void*);

static void interp_test_arm_next(void) {
  // Capture arm-time references for Path 2 delta comparison.
  // These are taken at the moment the timer is armed, so the
  // delta between arm and fire reflects the actual elapsed time.
  itest_ctx.arm_time_ns = time_gnss_ns_now();
  itest_ctx.arm_gpt2    = GPT2_CNT;

  timepop_handle_t h = timepop_arm(
    itest_ctx.delay_ns, false,
    interp_test_callback, nullptr, "itest"
  );
  if (h == TIMEPOP_INVALID_HANDLE) {
    itest_in_flight = false;
  }
}

static void interp_test_callback(timepop_ctx_t* ctx, void*) {

  // ── Path 1: math round-trip (requires campaign fragment) ──

  bool path1_valid = false;
  int64_t path1_error_ns = 0;
  uint64_t vclock_ns_into_second = 0;

  if (ctx->frag_valid && ctx->frag_dwt_cycles_per_pps > 0) {
    const uint32_t gpt2_since_pps = ctx->deadline - ctx->frag_gpt2_at_pps;
    vclock_ns_into_second = (uint64_t)gpt2_since_pps * 100ULL;
    const int64_t vclock_gnss_ns = (int64_t)(ctx->frag_gnss_ns + vclock_ns_into_second);

    // Synthesize DWT cycle count at the deadline moment (round-to-nearest)
    const uint32_t synth_dwt_elapsed =
      (uint32_t)((vclock_ns_into_second * (uint64_t)ctx->frag_dwt_cycles_per_pps + 500000000ULL) / 1000000000ULL);
    const uint32_t synth_dwt = ctx->frag_dwt_cyccnt_at_pps + synth_dwt_elapsed;

    // Forward interpolation: DWT cycles → GNSS nanoseconds
    const int64_t interp_gnss_ns = timebase_gnss_ns_from_dwt(
      synth_dwt,
      ctx->frag_gnss_ns,
      ctx->frag_dwt_cyccnt_at_pps,
      ctx->frag_dwt_cycles_per_pps
    );

    path1_error_ns = (interp_gnss_ns >= 0)
      ? interp_gnss_ns - vclock_gnss_ns
      : 0;
    path1_valid = true;
  }

  // ── Path 2: time.h rate accuracy (epoch-independent) ──
  //
  // Compare elapsed time as measured by time.h vs VCLOCK.
  //
  //   time_delta  = time_gnss_ns_now()  -  arm_time_ns
  //   vclock_delta = (GPT2_CNT - arm_gpt2) * 100
  //   error = time_delta - vclock_delta
  //
  // Both deltas measure the same physical interval.  The difference
  // is the DWT interpolation error accumulated over the delay.

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

  // ── Path 1: Welford update ──
  if (path1_valid) {
    itest_ctx.w_n++;
    const double x  = (double)path1_error_ns;
    const double d1 = x - itest_ctx.w_mean;
    itest_ctx.w_mean += d1 / (double)itest_ctx.w_n;
    const double d2 = x - itest_ctx.w_mean;
    itest_ctx.w_m2 += d1 * d2;

    if (path1_error_ns < itest_ctx.err_min) itest_ctx.err_min = path1_error_ns;
    if (path1_error_ns > itest_ctx.err_max) itest_ctx.err_max = path1_error_ns;
    if (path1_error_ns > 100 || path1_error_ns < -100) itest_ctx.outside_100ns++;
  }

  // ── Path 2: Welford update ──
  if (path2_valid) {
    itest_ctx.t_n++;
    const double x  = (double)path2_error_ns;
    const double d1 = x - itest_ctx.t_mean;
    itest_ctx.t_mean += d1 / (double)itest_ctx.t_n;
    const double d2 = x - itest_ctx.t_mean;
    itest_ctx.t_m2 += d1 * d2;

    if (path2_error_ns < itest_ctx.t_err_min) itest_ctx.t_err_min = path2_error_ns;
    if (path2_error_ns > itest_ctx.t_err_max) itest_ctx.t_err_max = path2_error_ns;
  }

  // ── Arm next or emit results ──
  itest_ctx.tests_remaining--;

  if (itest_ctx.tests_remaining > 0) {
    interp_test_arm_next();
    return;
  }

  // ── All samples collected — emit event ──
  const double w_stddev = (itest_ctx.w_n >= 2)
    ? sqrt(itest_ctx.w_m2 / (double)(itest_ctx.w_n - 1)) : 0.0;
  const double w_stderr = (itest_ctx.w_n >= 2)
    ? w_stddev / sqrt((double)itest_ctx.w_n) : 0.0;

  const double t_stddev = (itest_ctx.t_n >= 2)
    ? sqrt(itest_ctx.t_m2 / (double)(itest_ctx.t_n - 1)) : 0.0;
  const double t_stderr = (itest_ctx.t_n >= 2)
    ? t_stddev / sqrt((double)itest_ctx.t_n) : 0.0;

  Payload ev;

  // Path 1: math round-trip
  ev.add("m_n",    (int32_t)itest_ctx.w_n);
  ev.add("m_mean", itest_ctx.w_mean);
  ev.add("m_sd",   w_stddev);
  ev.add("m_se",   w_stderr);
  ev.add("m_min",  (itest_ctx.w_n > 0) ? itest_ctx.err_min : (int64_t)0);
  ev.add("m_max",  (itest_ctx.w_n > 0) ? itest_ctx.err_max : (int64_t)0);
  ev.add("m_out",  itest_ctx.outside_100ns);

  // Path 2: time.h rate accuracy
  ev.add("t_n",    (int32_t)itest_ctx.t_n);
  ev.add("t_mean", itest_ctx.t_mean);
  ev.add("t_sd",   t_stddev);
  ev.add("t_se",   t_stderr);
  ev.add("t_min",  (itest_ctx.t_n > 0) ? itest_ctx.t_err_min : (int64_t)0);
  ev.add("t_max",  (itest_ctx.t_n > 0) ? itest_ctx.t_err_max : (int64_t)0);

  // Forensics (last sample only)
  ev.add("s0_p1",  path1_error_ns);
  ev.add("s0_p2",  path2_error_ns);
  ev.add("s0_pos", (double)vclock_ns_into_second / 1000000000.0);
  ev.add("s0_fns", ctx->fire_ns);

  enqueueEvent("TIMEPOP_INTERP_TEST", ev);

  itest_in_flight = false;
}

static Payload cmd_interp_test(const Payload& args) {

  Payload resp;

  if (itest_in_flight) {
    resp.add("error", "test already in flight");
    return resp;
  }

  // time.h must be valid (PPS lock) for path 2.
  // Path 1 additionally requires a campaign for fragment data.
  if (!time_valid()) {
    resp.add("error", "time not valid (need PPS lock)");
    return resp;
  }

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

  out.add("isr_fires",      isr_fire_count);
  out.add("expired",        expired_count);
  out.add("gpt2_cnt",       gpt2_count());
  out.add("test_in_flight", test_in_flight);
  out.add("time_valid",     time_valid());
  out.add("time_pps_count", time_pps_count());

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