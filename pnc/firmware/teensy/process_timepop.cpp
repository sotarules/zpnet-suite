// ============================================================================
// process_timepop.cpp — TimePop v2 (GPT2 Output Compare)
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
// ============================================================================

#include "timepop.h"
#include "process_timepop.h"

#include "debug.h"
#include "process.h"
#include "events.h"
#include "payload.h"
#include "cpu_usage.h"
#include "timebase.h"

#include <Arduino.h>
#include "imxrt.h"
#include <string.h>

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

  // Capture GPT2_CNT immediately — this is the authoritative
  // "when did the ISR fire?" timestamp in GNSS ticks.
  const uint32_t now = GPT2_CNT;

  GPT2_SR = GPT_SR_OF1;
  isr_fire_count++;

  for (uint32_t i = 0; i < MAX_SLOTS; i++) {
    if (!slots[i].active || slots[i].expired) continue;

    if (deadline_expired(slots[i].deadline, now)) {
      slots[i].expired   = true;
      slots[i].fire_gpt2 = now;
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

    // Build context with ISR-captured fire time
    timepop_ctx_t ctx;
    ctx.handle    = slots[i].handle;
    ctx.fire_gpt2 = slots[i].fire_gpt2;
    ctx.deadline  = slots[i].deadline;
    ctx.fire_ns   = (int32_t)(slots[i].fire_gpt2 - slots[i].deadline) * 100;

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
// TEST command
// ============================================================================

struct test_context_t {
  int64_t  arm_gnss_ns;
  uint64_t requested_ns;
  uint32_t arm_frag_pps;
};

static test_context_t test_ctx = {};
static volatile bool  test_in_flight = false;

static void test_timer_callback(timepop_ctx_t* ctx, void*) {

  // Capture GPT2 at callback entry (for dispatch latency measurement)
  uint32_t cb_gpt2 = GPT2_CNT;

  int64_t end_gnss_ns = timebase_now_gnss_ns();
  const timebase_fragment_t* end_frag = timebase_last_fragment();

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

  Payload ev;
  ev.add("err",         error_ns);
  ev.add("meas",        measured_ns);
  ev.add("req",         test_ctx.requested_ns);
  ev.add("fire_ns",     ctx->fire_ns);       // OCR precision: ISR vs deadline
  ev.add("dispatch_ns", dispatch_ns);         // dispatch latency: callback vs ISR
  ev.add("arm_pps",     test_ctx.arm_frag_pps);
  ev.add("end_pps",     end_frag ? (uint32_t)end_frag->pps_count : (uint32_t)0);
  ev.add("pps_x",       end_frag ? (int32_t)((uint32_t)end_frag->pps_count - test_ctx.arm_frag_pps) : -1);

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

  int64_t arm_gnss = timebase_now_gnss_ns();
  const timebase_fragment_t* arm_frag = timebase_last_fragment();

  test_ctx = {};
  test_ctx.arm_gnss_ns  = arm_gnss;
  test_ctx.requested_ns = ns_val;
  test_ctx.arm_frag_pps = arm_frag ? arm_frag->pps_count : 0;

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
// REPORT command
// ============================================================================

static Payload cmd_report(const Payload&) {

  Payload out;

  out.add("isr_fires",      isr_fire_count);
  out.add("expired",        expired_count);
  out.add("gpt2_cnt",       gpt2_count());
  out.add("test_in_flight", test_in_flight);

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
  { "REPORT", cmd_report },
  { "TEST",   cmd_test   },
  { nullptr,  nullptr }
};

static const process_vtable_t TIMEPOP_PROCESS = {
  .process_id    = "TIMEPOP",
  .commands      = TIMEPOP_COMMANDS,
  .subscriptions = nullptr,
};

void process_timepop_register(void) {
  process_register("TIMEPOP", &TIMEPOP_PROCESS);
}
