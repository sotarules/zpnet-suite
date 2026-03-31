// ============================================================================
// process_interrupt_capture.cpp
// ============================================================================

#include "process_interrupt_capture.h"

#include "time.h"
#include "imxrt.h"

#include <string.h>

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

bool interrupt_capture_default_spin_window(uint64_t now_gnss_ns,
                                           uint64_t target_gnss_ns) {
  constexpr uint64_t PRESPIN_NS = 10000ULL;
  return (target_gnss_ns > now_gnss_ns) &&
         ((target_gnss_ns - now_gnss_ns) <= PRESPIN_NS);
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

// ============================================================================
// Counter32 reconstruction helpers
// ============================================================================

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

void interrupt_capture_engine_t::arm(uint64_t target_gnss_ns,
                                     uint32_t counter32_target) {
  next_target_gnss_ns_ = target_gnss_ns;
  next_counter32_target_ = counter32_target;
  armed_ = true;
}

void interrupt_capture_engine_t::disarm() {
  armed_ = false;
  next_target_gnss_ns_ = 0;
  next_counter32_target_ = 0;
}

bool interrupt_capture_engine_t::armed() const {
  return armed_;
}

const interrupt_capture_config_t& interrupt_capture_engine_t::config() const {
  return cfg_;
}

const interrupt_capture_latency_t& interrupt_capture_engine_t::latency() const {
  return latency_;
}

void interrupt_capture_engine_t::spin_step() {
  if (!armed_) {
    latency_.deterministic_spin_active = false;
    return;
  }

  const int64_t now_gnss_signed = time_gnss_ns_now();
  if (now_gnss_signed < 0) {
    latency_.deterministic_spin_active = false;
    return;
  }

  const uint64_t now_gnss_ns = static_cast<uint64_t>(now_gnss_signed);

  if (cfg_.spin_window_start_fn &&
      !cfg_.spin_window_start_fn(now_gnss_ns, next_target_gnss_ns_)) {
    latency_.deterministic_spin_active = false;
    return;
  }

  latency_.deterministic_spin_active = true;
  latency_.prev_spin_dwt = latency_.last_spin_dwt;
  latency_.last_spin_dwt = ic_read_dwt();
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
  if (cfg_.rearm_mode == interrupt_capture_rearm_mode_t::NONE || !cfg_.rearm_fn) {
    armed_ = false;
    return;
  }

  const interrupt_capture_rearm_decision_t decision = cfg_.rearm_fn(event, diag);
  if (!decision.rearm) {
    armed_ = false;
    return;
  }

  next_counter32_target_ = decision.next_counter32_target;
  next_target_gnss_ns_ = decision.next_target_gnss_ns;
  armed_ = true;
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
    return;
  }

  cfg_.raw_capture_fn(raw);
  raw.dwt_after_capture = ic_read_dwt();

  diag.dwt_before_capture = raw.dwt_isr_entry;
  diag.dwt_after_capture = raw.dwt_after_capture;
  diag.raw_counter16_at_event = raw.counter16_at_event;
  diag.raw_compare16 = raw.compare16;
  diag.raw_counter32_live = raw.counter32_live;

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