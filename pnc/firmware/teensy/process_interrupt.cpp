// ============================================================================
// process_interrupt.cpp — v6.1 GPT OCXO second-boundary (OCR3)
// ============================================================================
//
// GPT1/GPT2 for OCXO second-boundary events using output compare 3.
//
// OCR1 is reserved for the existing PPS phase capture system (one-shot,
// armed by pps_isr, consumed by gpt1/2_phase_isr).  We use OCR3 to
// avoid any conflict.
//
// GPT runs in free-running mode (FRR=1), so the counter never resets.
// OCR3 fires when counter == OCR3 value.  The ISR advances OCR3 by
// OCXO_COUNTS_PER_SECOND for the next second boundary.
//
// The existing GPT ISR vectors (gpt1_phase_isr, gpt2_phase_isr) are
// owned by process_clocks_alpha.  They check the OF3 flag and call
// process_interrupt_gpt1_irq / gpt2_irq, passing the DWT they already
// captured as their first ISR instruction.
//
// ============================================================================

#include "process_interrupt.h"
#include "debug.h"

#include "time.h"
#include "process.h"
#include "payload.h"
#include "tdc_correction.h"

#include <Arduino.h>
#include "imxrt.h"

#include <string.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_SUBSCRIBERS = 8;

// OCXO second boundary: 10,000,000 counts at ~10 MHz.
static constexpr uint32_t OCXO_COUNTS_PER_SECOND = 10000000;

// ============================================================================
// Hard-fail integrity trap
// ============================================================================

[[noreturn]] static void interrupt_integrity_trap(uint32_t code) {
  while (true) {
    switch (code) {
    case 1:  debug_blink("1"); break;
    case 2:  debug_blink("11"); break;
    case 3:  debug_blink("111"); break;
    case 4:  debug_blink("1111"); break;
    case 5:  debug_blink("11111"); break;
    case 6:  debug_blink("111111"); break;
    case 7:  debug_blink("1111111"); break;
    case 8:  debug_blink("11111111"); break;
    case 9:  debug_blink("111111111"); break;

    case 10: debug_blink("41"); break;
    case 11: debug_blink("411"); break;
    case 12: debug_blink("4111"); break;
    case 13: debug_blink("41111"); break;
    case 14: debug_blink("411111"); break;

    case 15: debug_blink("91"); break;
    case 16: debug_blink("911"); break;
    case 17: debug_blink("9111"); break;

    default: debug_blink("911911"); break;
    }
    debug_sleep_ms(1500);
  }
}

// ============================================================================
// Subscriber descriptor + runtime state
// ============================================================================

struct interrupt_subscriber_descriptor_t {
  interrupt_subscriber_kind_t kind = interrupt_subscriber_kind_t::NONE;
  const char* name = nullptr;

  interrupt_provider_kind_t provider = interrupt_provider_kind_t::NONE;
  interrupt_lane_t lane = interrupt_lane_t::NONE;

  uint32_t isr_overhead_cycles = 0;
};

struct interrupt_subscriber_runtime_t {
  const interrupt_subscriber_descriptor_t* desc = nullptr;
  interrupt_subscription_t sub {};

  bool subscribed = false;
  bool active = false;

  // OCR3 tracking — the next compare value for continuous fire.
  uint32_t next_ocr3 = 0;

  // Last event delivered to the subscriber.
  interrupt_event_t last_event {};
  bool has_fired = false;

  // Counters.
  uint32_t start_count = 0;
  uint32_t stop_count = 0;
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t ocxo_second_count = 0;
};

// ============================================================================
// Provider diagnostics
// ============================================================================

struct interrupt_provider_diag_t {
  uint32_t irq_count = 0;
  uint32_t dispatch_count = 0;
  uint32_t miss_count = 0;
};

static interrupt_subscriber_runtime_t g_subscribers[MAX_INTERRUPT_SUBSCRIBERS] = {};
static uint32_t g_subscriber_count = 0;

static bool g_interrupt_hw_ready = false;

static interrupt_provider_diag_t g_gpt1_diag = {};
static interrupt_provider_diag_t g_gpt2_diag = {};

// Direct runtime pointers for ISR dispatch — cached at subscribe time.
static interrupt_subscriber_runtime_t* g_rt_ocxo1 = nullptr;
static interrupt_subscriber_runtime_t* g_rt_ocxo2 = nullptr;

// ============================================================================
// Subscriber descriptors
// ============================================================================

static const interrupt_subscriber_descriptor_t DESCRIPTORS[] = {
  {
    interrupt_subscriber_kind_t::OCXO1,
    "OCXO1",
    interrupt_provider_kind_t::GPT1,
    interrupt_lane_t::GPT1_COMPARE1,
    GPT_ISR_FIXED_OVERHEAD,
  },
  {
    interrupt_subscriber_kind_t::OCXO2,
    "OCXO2",
    interrupt_provider_kind_t::GPT2,
    interrupt_lane_t::GPT2_COMPARE1,
    GPT_ISR_FIXED_OVERHEAD,
  },
};

// ============================================================================
// Descriptor / runtime lookup
// ============================================================================

static const interrupt_subscriber_descriptor_t* descriptor_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < (sizeof(DESCRIPTORS) / sizeof(DESCRIPTORS[0])); i++) {
    if (DESCRIPTORS[i].kind == kind) return &DESCRIPTORS[i];
  }
  return nullptr;
}

static interrupt_subscriber_runtime_t* runtime_for(interrupt_subscriber_kind_t kind) {
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    if (g_subscribers[i].subscribed &&
        g_subscribers[i].sub.kind == kind) {
      return &g_subscribers[i];
    }
  }
  return nullptr;
}

// ============================================================================
// Public helper strings
// ============================================================================

const char* interrupt_subscriber_kind_str(interrupt_subscriber_kind_t kind) {
  switch (kind) {
    case interrupt_subscriber_kind_t::TIMEPOP:     return "TIMEPOP";
    case interrupt_subscriber_kind_t::TIME_TEST:   return "TIME_TEST";
    case interrupt_subscriber_kind_t::PPS:         return "PPS";
    case interrupt_subscriber_kind_t::VCLOCK:      return "VCLOCK";
    case interrupt_subscriber_kind_t::OCXO1:       return "OCXO1";
    case interrupt_subscriber_kind_t::OCXO2:       return "OCXO2";
    case interrupt_subscriber_kind_t::PHOTODIODE:  return "PHOTODIODE";
    default:                                       return "NONE";
  }
}

const char* interrupt_provider_kind_str(interrupt_provider_kind_t provider) {
  switch (provider) {
    case interrupt_provider_kind_t::QTIMER1:  return "QTIMER1";
    case interrupt_provider_kind_t::GPT1:     return "GPT1";
    case interrupt_provider_kind_t::GPT2:     return "GPT2";
    case interrupt_provider_kind_t::GPIO6789: return "GPIO6789";
    default:                                  return "NONE";
  }
}

const char* interrupt_lane_str(interrupt_lane_t lane) {
  switch (lane) {
    case interrupt_lane_t::QTIMER1_CH2:   return "QTIMER1_CH2";
    case interrupt_lane_t::QTIMER1_CH3:   return "QTIMER1_CH3";
    case interrupt_lane_t::GPT1_COMPARE1: return "GPT1_COMPARE1";
    case interrupt_lane_t::GPT2_COMPARE1: return "GPT2_COMPARE1";
    case interrupt_lane_t::GPIO_EDGE:     return "GPIO_EDGE";
    default:                              return "NONE";
  }
}

// ============================================================================
// Event reconstruction — GPT OCXO second-boundary
// ============================================================================

static void gpt_handle_ocxo_event(interrupt_subscriber_runtime_t& rt,
                                  uint32_t dwt_isr_entry_raw) {
  if (!rt.desc || !rt.sub.on_event) return;

  const uint32_t correction = rt.desc->isr_overhead_cycles;
  const uint32_t dwt_at_event = dwt_isr_entry_raw - correction;

  const int64_t gnss_ns_signed = time_dwt_to_gnss_ns(dwt_at_event);

  rt.ocxo_second_count++;

  interrupt_event_t event {};
  event.kind = rt.desc->kind;
  event.provider = rt.desc->provider;
  event.lane = rt.desc->lane;
  event.dwt_at_event = dwt_at_event;
  event.dwt_event_correction_cycles = correction;
  event.counter32_at_event = rt.ocxo_second_count;

  if (gnss_ns_signed >= 0) {
    event.gnss_ns_at_event = (uint64_t)gnss_ns_signed;
    event.status = interrupt_event_status_t::OK;
  } else {
    event.gnss_ns_at_event = 0;
    event.status = interrupt_event_status_t::HOLD;
  }

  rt.last_event = event;
  rt.has_fired = true;
  rt.irq_count++;
  rt.dispatch_count++;

  rt.sub.on_event(event, nullptr, rt.sub.user_data);
}

// ============================================================================
// GPT1 ISR entry — called from gpt1_phase_isr when OF3 is set
// ============================================================================

void process_interrupt_gpt1_irq(uint32_t dwt_isr_entry_raw) {
  g_gpt1_diag.irq_count++;

  if (!(GPT1_SR & GPT_SR_OF3)) {
    g_gpt1_diag.miss_count++;
    return;
  }

  // Clear OF3 flag.
  GPT1_SR = GPT_SR_OF3;

  g_gpt1_diag.dispatch_count++;

  interrupt_subscriber_runtime_t* rt = g_rt_ocxo1;
  if (!rt || !rt->active) return;

  // Advance OCR3 for next second boundary.
  rt->next_ocr3 += OCXO_COUNTS_PER_SECOND;
  GPT1_OCR3 = rt->next_ocr3;

  gpt_handle_ocxo_event(*rt, dwt_isr_entry_raw);
}

// ============================================================================
// GPT2 ISR entry — called from gpt2_phase_isr when OF3 is set
// ============================================================================

void process_interrupt_gpt2_irq(uint32_t dwt_isr_entry_raw) {
  g_gpt2_diag.irq_count++;

  if (!(GPT2_SR & GPT_SR_OF3)) {
    g_gpt2_diag.miss_count++;
    return;
  }

  GPT2_SR = GPT_SR_OF3;

  g_gpt2_diag.dispatch_count++;

  interrupt_subscriber_runtime_t* rt = g_rt_ocxo2;
  if (!rt || !rt->active) return;

  rt->next_ocr3 += OCXO_COUNTS_PER_SECOND;
  GPT2_OCR3 = rt->next_ocr3;

  gpt_handle_ocxo_event(*rt, dwt_isr_entry_raw);
}

// ============================================================================
// Public control
// ============================================================================

bool interrupt_subscribe(const interrupt_subscription_t& sub) {
  if (!g_interrupt_hw_ready) return false;
  if (sub.kind == interrupt_subscriber_kind_t::NONE) return false;
  if (!sub.on_event) return false;

  const interrupt_subscriber_descriptor_t* desc = descriptor_for(sub.kind);
  if (!desc) return false;

  interrupt_subscriber_runtime_t* existing = runtime_for(sub.kind);
  if (existing) {
    existing->sub = sub;
    existing->desc = desc;
    existing->subscribed = true;
    return true;
  }

  if (g_subscriber_count >= MAX_INTERRUPT_SUBSCRIBERS) return false;

  interrupt_subscriber_runtime_t& rt = g_subscribers[g_subscriber_count++];
  rt = {};
  rt.desc = desc;
  rt.sub = sub;
  rt.subscribed = true;

  if (sub.kind == interrupt_subscriber_kind_t::OCXO1) {
    g_rt_ocxo1 = &rt;
  } else if (sub.kind == interrupt_subscriber_kind_t::OCXO2) {
    g_rt_ocxo2 = &rt;
  }

  return true;
}

bool interrupt_start(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = true;
  rt->start_count++;

  // Read the current GPT counter and set OCR3 to the next second boundary.
  // This ensures the first fire happens within one OCXO-second of start.
  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    uint32_t now = GPT1_CNT;
    rt->next_ocr3 = now + OCXO_COUNTS_PER_SECOND;
    GPT1_OCR3 = rt->next_ocr3;
    GPT1_SR = GPT_SR_OF3;                     // clear any stale flag
    GPT1_IR |= GPT_IR_OF3IE;                  // enable output compare 3 interrupt
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    uint32_t now = GPT2_CNT;
    rt->next_ocr3 = now + OCXO_COUNTS_PER_SECOND;
    GPT2_OCR3 = rt->next_ocr3;
    GPT2_SR = GPT_SR_OF3;
    GPT2_IR |= GPT_IR_OF3IE;
  }

  return true;
}

bool interrupt_stop(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->desc) return false;

  rt->active = false;
  rt->stop_count++;

  if (kind == interrupt_subscriber_kind_t::OCXO1) {
    GPT1_IR &= ~GPT_IR_OF3IE;
  } else if (kind == interrupt_subscriber_kind_t::OCXO2) {
    GPT2_IR &= ~GPT_IR_OF3IE;
  }

  return true;
}

// ============================================================================
// Last event accessor
// ============================================================================

const interrupt_event_t* interrupt_last_event(interrupt_subscriber_kind_t kind) {
  interrupt_subscriber_runtime_t* rt = runtime_for(kind);
  if (!rt || !rt->has_fired) return nullptr;
  return &rt->last_event;
}

// ============================================================================
// Init / process registration
// ============================================================================

void process_interrupt_init(void) {
  if (g_interrupt_hw_ready) return;
  g_interrupt_hw_ready = true;
}

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_hw_ready);
  p.add("subscriber_count", g_subscriber_count);

  p.add("gpt1_irq_count", g_gpt1_diag.irq_count);
  p.add("gpt1_dispatch_count", g_gpt1_diag.dispatch_count);
  p.add("gpt1_miss_count", g_gpt1_diag.miss_count);

  p.add("gpt2_irq_count", g_gpt2_diag.irq_count);
  p.add("gpt2_dispatch_count", g_gpt2_diag.dispatch_count);
  p.add("gpt2_miss_count", g_gpt2_diag.miss_count);

  PayloadArray subscribers;
  for (uint32_t i = 0; i < g_subscriber_count; i++) {
    const interrupt_subscriber_runtime_t& rt = g_subscribers[i];
    if (!rt.subscribed || !rt.desc) continue;

    Payload s;
    s.add("slot", i);
    s.add("kind", interrupt_subscriber_kind_str(rt.sub.kind));
    s.add("provider", interrupt_provider_kind_str(rt.desc->provider));
    s.add("lane", interrupt_lane_str(rt.desc->lane));
    s.add("active", rt.active);

    s.add("start_count", rt.start_count);
    s.add("stop_count", rt.stop_count);
    s.add("irq_count", rt.irq_count);
    s.add("dispatch_count", rt.dispatch_count);
    s.add("ocxo_second_count", rt.ocxo_second_count);
    s.add("next_ocr3", rt.next_ocr3);

    s.add("last_dwt_at_event", rt.last_event.dwt_at_event);
    s.add("last_gnss_ns_at_event", rt.last_event.gnss_ns_at_event);
    s.add("last_correction_cycles", rt.last_event.dwt_event_correction_cycles);
    s.add("last_status", (uint32_t)rt.last_event.status);

    subscribers.add(s);
  }

  p.add_array("subscribers", subscribers);
  return p;
}

static const process_command_entry_t INTERRUPT_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr,  nullptr    }
};

static const process_vtable_t INTERRUPT_PROCESS = {
  .process_id    = "INTERRUPT",
  .commands      = INTERRUPT_COMMANDS,
  .subscriptions = nullptr
};

void process_interrupt_register(void) {
  process_register("INTERRUPT", &INTERRUPT_PROCESS);
}