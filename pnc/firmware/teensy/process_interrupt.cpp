// ============================================================================
// process_interrupt_capture.cpp
// ============================================================================

#include "process_interrupt.h"

#include "time.h"
#include "process.h"
#include "payload.h"
#include "process_timepop.h"
#include "timepop.h"

#include <Arduino.h>
#include "imxrt.h"

#include <string.h>

// ============================================================================
// Constants / shared authority state
// ============================================================================

static constexpr uint32_t MAX_INTERRUPT_CAPTURE_ENGINES = 8;

static interrupt_capture_engine_t* g_engines[MAX_INTERRUPT_CAPTURE_ENGINES] = {};
static uint32_t g_engine_count = 0;
static bool g_interrupt_capture_hw_ready = false;

static volatile uint32_t g_qtimer1_irq_count = 0;
static volatile uint32_t g_gpt1_irq_count = 0;
static volatile uint32_t g_gpt2_irq_count = 0;
static volatile uint32_t g_gpio6789_irq_count = 0;
static volatile uint32_t g_dispatch_misses = 0;

// ============================================================================
// Local helpers
// ============================================================================

static inline uint32_t ic_read_dwt() {
  return DWT_CYCCNT;
}

static inline void ic_welford_add(interrupt_capture_latency_t& t, double x) {
  t.sample_count += 1;
  const double delta = x - t.mean_cycles;
  t.mean_cycles += delta / static_cast<double>(t.sample_count);
  const double delta2 = x - t.mean_cycles;
  t.m2_cycles += delta * delta2;
  t.live_profile_valid = true;
}

static inline uint32_t ic_welford_mean_u32(const interrupt_capture_latency_t& t) {
  return static_cast<uint32_t>(t.mean_cycles + 0.5);
}

static interrupt_capture_hw_domain_t hw_domain_for(interrupt_capture_class_t cls) {
  switch (cls) {
    case interrupt_capture_class_t::QTIMER_COMPARE: return interrupt_capture_hw_domain_t::QTIMER1;
    case interrupt_capture_class_t::GPT1_COMPARE:   return interrupt_capture_hw_domain_t::GPT1;
    case interrupt_capture_class_t::GPT2_COMPARE:   return interrupt_capture_hw_domain_t::GPT2;
    case interrupt_capture_class_t::GPIO_EDGE:      return interrupt_capture_hw_domain_t::GPIO6789;
    default:                                        return interrupt_capture_hw_domain_t::NONE;
  }
}

// ============================================================================
// Public helpers
// ============================================================================

const char* interrupt_capture_class_str(interrupt_capture_class_t cls) {
  switch (cls) {
    case interrupt_capture_class_t::GPIO_EDGE:      return "GPIO_EDGE";
    case interrupt_capture_class_t::QTIMER_COMPARE: return "QTIMER_COMPARE";
    case interrupt_capture_class_t::GPT1_COMPARE:   return "GPT1_COMPARE";
    case interrupt_capture_class_t::GPT2_COMPARE:   return "GPT2_COMPARE";
    default:                                        return "UNKNOWN";
  }
}

uint64_t interrupt_capture_default_gnss_projection(const interrupt_capture_raw_t&,
                                                   uint32_t dwt_at_event,
                                                   uint32_t,
                                                   bool,
                                                   const interrupt_capture_latency_t&,
                                                   interrupt_capture_diag_t*,
                                                   bool* out_valid) {
  const int64_t gnss = time_dwt_to_gnss_ns(dwt_at_event);
  if (gnss < 0) {
    if (out_valid) *out_valid = false;
    return 0;
  }
  if (out_valid) *out_valid = true;
  return static_cast<uint64_t>(gnss);
}

interrupt_capture_rearm_decision_t interrupt_capture_no_rearm(
    const interrupt_capture_event_t&,
    const interrupt_capture_diag_t*) {
  return {};
}

uint32_t interrupt_capture_gpio_no_counter32(const interrupt_capture_raw_t&,
                                             interrupt_capture_diag_t*,
                                             bool* out_valid) {
  if (out_valid) *out_valid = false;
  return 0;
}

uint32_t interrupt_capture_qtimer_counter32_at_event(const interrupt_capture_raw_t& raw,
                                                     interrupt_capture_diag_t* diag,
                                                     bool* out_valid) {
  const uint32_t live = raw.counter32_live;
  const uint32_t a = (live & 0xFFFF0000u) | static_cast<uint32_t>(raw.counter16_at_event);

  uint32_t b = a;
  bool use_b = false;

  if ((live & 0xFFFFu) < raw.counter16_at_event) {
    b = a - 0x00010000u;
    use_b = true;
  }

  if (diag) {
    diag->candidate_counter32_a = a;
    diag->candidate_counter32_b = b;
    diag->used_counter32_candidate_b = use_b;
  }

  if (out_valid) *out_valid = true;
  return use_b ? b : a;
}

uint32_t interrupt_capture_gpt_counter32_at_event(const interrupt_capture_raw_t& raw,
                                                  interrupt_capture_diag_t* diag,
                                                  bool* out_valid) {
  const uint32_t live = raw.counter32_live;
  const uint32_t a = (live & 0xFFFF0000u) | static_cast<uint32_t>(raw.counter16_at_event);

  uint32_t b = a;
  bool use_b = false;

  if ((live & 0xFFFFu) < raw.counter16_at_event) {
    b = a - 0x00010000u;
    use_b = true;
  }

  if (diag) {
    diag->candidate_counter32_a = a;
    diag->candidate_counter32_b = b;
    diag->used_counter32_candidate_b = use_b;
  }

  if (out_valid) *out_valid = true;
  return use_b ? b : a;
}

// ============================================================================
// Engine
// ============================================================================

interrupt_capture_engine_t::interrupt_capture_engine_t(
    const interrupt_capture_config_t& cfg)
    : cfg_(cfg) {
  latency_ = {};
  latency_.entry_latency_cycles_default = cfg.default_entry_latency_cycles;
  latency_.entry_latency_cycles_min = cfg.default_entry_latency_cycles;
  latency_.entry_latency_cycles_max = cfg.default_entry_latency_cycles;
}

const interrupt_capture_config_t& interrupt_capture_engine_t::config() const {
  return cfg_;
}

const interrupt_capture_latency_t& interrupt_capture_engine_t::latency() const {
  return latency_;
}

bool interrupt_capture_engine_t::active() const {
  return active_;
}

bool interrupt_capture_engine_t::waiting_for_interrupt() const {
  return waiting_for_interrupt_;
}

void interrupt_capture_engine_t::start() {
  active_ = true;
  schedule_next_spin(nullptr, nullptr);
}

void interrupt_capture_engine_t::stop() {
  active_ = false;
  waiting_for_interrupt_ = false;
  if (spin_handle_ != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(spin_handle_);
    spin_handle_ = TIMEPOP_INVALID_HANDLE;
  }
}

void interrupt_capture_engine_t::spin_timepop_callback(timepop_ctx_t*, void* user_data) {
  auto* self = static_cast<interrupt_capture_engine_t*>(user_data);
  if (!self) return;
  self->on_spin_timepop();
}

void interrupt_capture_engine_t::on_spin_timepop() {
  if (!active_) return;

  uint32_t target = 0;
  if (cfg_.arm_target_fn) {
    if (!cfg_.arm_target_fn(cfg_.user_data, &target)) {
      waiting_for_interrupt_ = false;
      schedule_next_spin(nullptr, nullptr);
      return;
    }
  }

  armed_counter32_target_ = target;
  waiting_for_interrupt_ = true;
}

void interrupt_capture_engine_t::schedule_next_spin(const interrupt_capture_event_t* last_event,
                                                    const interrupt_capture_diag_t* last_diag) {
  if (!active_) return;
  if (!cfg_.next_spin_delay_fn) return;

  const int64_t delay_ns = cfg_.next_spin_delay_fn(cfg_.user_data, last_event, last_diag);
  if (delay_ns < 0) return;

  if (spin_handle_ != TIMEPOP_INVALID_HANDLE) {
    timepop_cancel(spin_handle_);
    spin_handle_ = TIMEPOP_INVALID_HANDLE;
  }

  spin_handle_ = timepop_arm((uint64_t)delay_ns,
                             false,
                             interrupt_capture_engine_t::spin_timepop_callback,
                             this,
                             cfg_.name ? cfg_.name : "interrupt-spin");
}

void interrupt_capture_engine_t::run_spin_step() {
  if (!active_ || !waiting_for_interrupt_) {
    latency_.deterministic_spin_active = false;
    return;
  }

  latency_.deterministic_spin_active = true;
  latency_.prev_spin_dwt = latency_.last_spin_dwt;
  latency_.last_spin_dwt = ic_read_dwt();
}

bool interrupt_capture_engine_t::matches_qtimer1_irq() const {
  return active_ &&
         waiting_for_interrupt_ &&
         cfg_.cls == interrupt_capture_class_t::QTIMER_COMPARE;
}

bool interrupt_capture_engine_t::matches_gpt1_irq() const {
  return active_ &&
         waiting_for_interrupt_ &&
         cfg_.cls == interrupt_capture_class_t::GPT1_COMPARE;
}

bool interrupt_capture_engine_t::matches_gpt2_irq() const {
  return active_ &&
         waiting_for_interrupt_ &&
         cfg_.cls == interrupt_capture_class_t::GPT2_COMPARE;
}

bool interrupt_capture_engine_t::matches_gpio6789_irq() const {
  return active_ &&
         waiting_for_interrupt_ &&
         cfg_.cls == interrupt_capture_class_t::GPIO_EDGE;
}

uint32_t interrupt_capture_engine_t::choose_entry_latency_cycles(
    const interrupt_capture_raw_t&,
    interrupt_capture_diag_t& diag) const {
  const uint32_t default_cycles = latency_.entry_latency_cycles_default;
  const uint32_t live_cycles =
      latency_.live_profile_valid ? ic_welford_mean_u32(latency_) : default_cycles;

  diag.candidate_latency_default_cycles = default_cycles;
  diag.candidate_latency_live_cycles = live_cycles;

  if (latency_.live_profile_valid) {
    diag.used_live_profile = true;
    return live_cycles;
  }

  diag.used_default_profile = true;
  return default_cycles;
}

void interrupt_capture_engine_t::update_live_latency(uint32_t entry_latency_cycles) {
  latency_.entry_latency_cycles = entry_latency_cycles;

  if (latency_.sample_count == 0) {
    latency_.entry_latency_cycles_min = entry_latency_cycles;
    latency_.entry_latency_cycles_max = entry_latency_cycles;
  } else {
    if (entry_latency_cycles < latency_.entry_latency_cycles_min) {
      latency_.entry_latency_cycles_min = entry_latency_cycles;
    }
    if (entry_latency_cycles > latency_.entry_latency_cycles_max) {
      latency_.entry_latency_cycles_max = entry_latency_cycles;
    }
  }

  ic_welford_add(latency_, static_cast<double>(entry_latency_cycles));
}

void interrupt_capture_engine_t::rearm_from_isr(const interrupt_capture_event_t& event,
                                                const interrupt_capture_diag_t* diag) {
  waiting_for_interrupt_ = false;

  if (cfg_.rearm_mode == interrupt_capture_rearm_mode_t::NONE) {
    return;
  }

  if (cfg_.rearm_mode == interrupt_capture_rearm_mode_t::CALLBACK_DRIVEN && cfg_.rearm_fn) {
    const interrupt_capture_rearm_decision_t decision = cfg_.rearm_fn(event, diag);
    if (!decision.rearm) {
      return;
    }
  }

  schedule_next_spin(&event, diag);
}

void interrupt_capture_engine_t::handle_interrupt() {
  interrupt_capture_diag_t diag {};
  diag.enabled = cfg_.diagnostics_enabled;
  diag.spin_last_dwt = latency_.last_spin_dwt;
  diag.spin_prev_dwt = latency_.prev_spin_dwt;

  interrupt_capture_raw_t raw {};
  raw.dwt_isr_entry = ic_read_dwt();
  diag.dwt_isr_entry = raw.dwt_isr_entry;

  if (!cfg_.raw_capture_fn) {
    waiting_for_interrupt_ = false;
    return;
  }

  cfg_.raw_capture_fn(raw, cfg_.diagnostics_enabled ? &diag : nullptr);
  raw.dwt_after_capture = ic_read_dwt();

  diag.dwt_before_capture = raw.dwt_isr_entry;
  diag.dwt_after_capture = raw.dwt_after_capture;
  diag.raw_counter16_at_event = raw.counter16_at_event;
  diag.raw_compare16 = raw.compare16;
  diag.raw_counter32_live = raw.counter32_live;

  if (diag.shadow_valid) {
    diag.delta_cycles = raw.dwt_isr_entry - diag.shadow_dwt;
  } else {
    diag.delta_cycles = 0;
  }

  const uint32_t latency_cycles = choose_entry_latency_cycles(raw, diag);
  const uint32_t dwt_at_event = raw.dwt_isr_entry - latency_cycles;

  bool counter32_valid = false;
  uint32_t counter32_at_event = 0;
  if (cfg_.counter32_at_event_fn) {
    counter32_at_event = cfg_.counter32_at_event_fn(
        raw, cfg_.diagnostics_enabled ? &diag : nullptr, &counter32_valid);
  }

  bool gnss_valid = false;
  uint64_t gnss_ns_at_event = 0;
  if (cfg_.gnss_at_event_fn) {
    gnss_ns_at_event = cfg_.gnss_at_event_fn(
        raw,
        dwt_at_event,
        counter32_at_event,
        counter32_valid,
        latency_,
        cfg_.diagnostics_enabled ? &diag : nullptr,
        &gnss_valid
    );
  }

  diag.chosen_entry_latency_cycles = latency_cycles;
  diag.chosen_dwt_at_event = dwt_at_event;
  diag.chosen_gnss_ns_at_event = gnss_ns_at_event;
  diag.chosen_counter32_at_event = counter32_at_event;

  update_live_latency(latency_cycles);

  interrupt_capture_event_t event {};
  event.cls = cfg_.cls;
  event.status = interrupt_capture_status_t::OK;
  event.dwt_at_event = dwt_at_event;
  event.dwt_valid = true;
  event.gnss_ns_at_event = gnss_ns_at_event;
  event.gnss_valid = gnss_valid;
  event.counter32_at_event = counter32_at_event;
  event.counter32_valid = counter32_valid;
  event.latency_cycles_used = latency_cycles;
  event.raw = raw;
  event.latency = latency_;

  if (cfg_.callback_fn) {
    cfg_.callback_fn(event, cfg_.diagnostics_enabled ? &diag : nullptr);
  }

  rearm_from_isr(event, cfg_.diagnostics_enabled ? &diag : nullptr);
}

// ============================================================================
// Shared hardware authority init
// ============================================================================

void process_interrupt_init(void) {
  if (g_interrupt_capture_hw_ready) return;

  // Shared QTimer1 authority: do NOT install a new vector here.
  // process_timepop.cpp will forward shared IRQ ownership to us through
  // process_interrupt_qtimer1_irq().
  //
  // GPT1/GPT2/GPIO domains may be added here later if we centralize those
  // vectors too. For now this process owns the registry/dispatch logic and
  // TimePop owns the physical QTIMER1 vector.
  g_interrupt_capture_hw_ready = true;
}

// ============================================================================
// Declarative handler registration
// ============================================================================

bool declareInterruptHandler(const interrupt_capture_handler_registration_t& reg) {
  if (!g_interrupt_capture_hw_ready) return false;
  if (g_engine_count >= MAX_INTERRUPT_CAPTURE_ENGINES) return false;

  auto* engine = new interrupt_capture_engine_t(reg.cfg);
  if (!engine) return false;

  g_engines[g_engine_count++] = engine;
  engine->start();
  return true;
}

// ============================================================================
// Shared IRQ authority dispatch
// ============================================================================

void process_interrupt_qtimer1_irq(void) {
  g_qtimer1_irq_count++;

  for (uint32_t i = 0; i < g_engine_count; i++) {
    if (!g_engines[i]) continue;
    if (!g_engines[i]->matches_qtimer1_irq()) continue;
    g_engines[i]->handle_interrupt();
    return;
  }

  g_dispatch_misses++;
}

void process_interrupt_gpt1_irq(void) {
  g_gpt1_irq_count++;

  for (uint32_t i = 0; i < g_engine_count; i++) {
    if (!g_engines[i]) continue;
    if (!g_engines[i]->matches_gpt1_irq()) continue;
    g_engines[i]->handle_interrupt();
    return;
  }

  g_dispatch_misses++;
}

void process_interrupt_gpt2_irq(void) {
  g_gpt2_irq_count++;

  for (uint32_t i = 0; i < g_engine_count; i++) {
    if (!g_engines[i]) continue;
    if (!g_engines[i]->matches_gpt2_irq()) continue;
    g_engines[i]->handle_interrupt();
    return;
  }

  g_dispatch_misses++;
}

void process_interrupt_gpio6789_irq(void) {
  g_gpio6789_irq_count++;

  for (uint32_t i = 0; i < g_engine_count; i++) {
    if (!g_engines[i]) continue;
    if (!g_engines[i]->matches_gpio6789_irq()) continue;
    g_engines[i]->handle_interrupt();
    return;
  }

  g_dispatch_misses++;
}

// ============================================================================
// Process command surface
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("hardware_ready", g_interrupt_capture_hw_ready);
  p.add("engine_count", g_engine_count);
  p.add("qtimer1_irq_count", g_qtimer1_irq_count);
  p.add("gpt1_irq_count", g_gpt1_irq_count);
  p.add("gpt2_irq_count", g_gpt2_irq_count);
  p.add("gpio6789_irq_count", g_gpio6789_irq_count);
  p.add("dispatch_misses", g_dispatch_misses);

  PayloadArray handlers;
  for (uint32_t i = 0; i < g_engine_count; i++) {
    if (!g_engines[i]) continue;
    Payload h;
    h.add("slot", i);
    h.add("name", g_engines[i]->config().name ? g_engines[i]->config().name : "");
    h.add("class", interrupt_capture_class_str(g_engines[i]->config().cls));
    h.add("active", g_engines[i]->active());
    h.add("waiting_for_interrupt", g_engines[i]->waiting_for_interrupt());
    h.add("latency_default_cycles", g_engines[i]->latency().entry_latency_cycles_default);
    h.add("latency_mean_cycles", g_engines[i]->latency().mean_cycles);
    h.add("latency_samples", (int32_t)g_engines[i]->latency().sample_count);
    handlers.add(h);
  }
  p.add_array("handlers", handlers);

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