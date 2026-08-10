#include "process_photons.h"

#include "payload.h"
#include "process.h"
#include "process_interrupt.h"
#include "publish.h"
#include "timepop.h"

#include <Arduino.h>

// ============================================================================
// PHOTONS scaffold doctrine
// ============================================================================
//
// process_interrupt owns the physical PD200T comparator interrupt and the
// first-instruction DWT coordinate.
//
// PHOTONS consumes that immutable edge fact through the specialized high-rate
// PHOTODIODE subscription.  The ISR callback below is intentionally tiny:
// scalar updates only, no Payload, no publication, no CLOCKS call, no TimePop
// mutation.
//
// Once per second a normal foreground TimePop callback snapshots the toy state
// and publishes PHOTONS_FRAGMENT.  This is scaffolding for validating custody,
// not the final optical science implementation.
// ============================================================================

static constexpr uint64_t PHOTONS_FRAGMENT_PERIOD_NS = 1000000000ULL;

// -----------------------------------------------------------------------------
// ISR-authored live state
// -----------------------------------------------------------------------------
//
// Single writer: PHOTODIODE callback.
// Foreground readers use generation as a tiny seqlock.  No interrupt masking is
// required, so PHOTONS reporting/publication never delays sovereign CLOCKS IRQs.
//
struct photons_live_state_t {
  volatile uint32_t generation = 0;
  photons_toy_capture_t capture{};

  bool     previous_edge_valid = false;
  uint32_t previous_dwt_at_edge = 0;

  bool     previous_interval_valid = false;
  uint32_t previous_interval_cycles = 0;
};

static photons_live_state_t g_photons_live{};

// -----------------------------------------------------------------------------
// Foreground-owned publication state
// -----------------------------------------------------------------------------

static photons_toy_fragment_t g_last_fragment{};
static uint32_t g_fragment_sequence = 0;
static uint32_t g_publish_count = 0;
static uint32_t g_publish_reject_count = 0;
static uint32_t g_last_published_edge_count = 0;

static bool g_initialized = false;
static bool g_subscription_ok = false;
static bool g_interrupt_started = false;

static timepop_handle_t g_fragment_timer = TIMEPOP_INVALID_HANDLE;

static inline void photons_compiler_barrier(void) {
  __asm__ volatile("" ::: "memory");
}

// ============================================================================
// High-rate PHOTODIODE subscriber
// ============================================================================

static void photons_on_photodiode_edge(
    const interrupt_photodiode_edge_t& edge,
    const interrupt_photodiode_diag_t& /*diag*/,
    void* /*user_data*/) {

  // Begin seqlock write: odd generation means foreground must retry.
  g_photons_live.generation++;
  photons_compiler_barrier();

  photons_toy_capture_t& c = g_photons_live.capture;

  c.edge_count++;
  c.last_edge_sequence = edge.sequence;
  c.last_pps_sequence = edge.pps_sequence;
  c.last_dwt_at_edge = edge.dwt_at_edge;
  c.last_isr_entry_dwt_raw = edge.isr_entry_dwt_raw;
  c.isr_entry_to_edge_correction_cycles =
      edge.isr_entry_to_edge_correction_cycles;

  if (g_photons_live.previous_edge_valid) {
    const uint32_t interval =
        edge.dwt_at_edge - g_photons_live.previous_dwt_at_edge;

    c.interval_valid = true;
    c.last_interval_cycles = interval;

    if (c.min_interval_cycles == 0U || interval < c.min_interval_cycles) {
      c.min_interval_cycles = interval;
    }
    if (interval > c.max_interval_cycles) {
      c.max_interval_cycles = interval;
    }

    // Toy predictor only.  The final PHOTONS engine will decide its own
    // acceptance/exclusion semantics.
    if (g_photons_live.previous_interval_valid) {
      c.prediction_valid = true;
      c.prediction_cycles = g_photons_live.previous_interval_cycles;
      c.residual_cycles =
          (int32_t)(interval - g_photons_live.previous_interval_cycles);
    } else {
      c.prediction_valid = false;
      c.prediction_cycles = 0U;
      c.residual_cycles = 0;
    }

    g_photons_live.previous_interval_cycles = interval;
    g_photons_live.previous_interval_valid = true;
  }

  g_photons_live.previous_dwt_at_edge = edge.dwt_at_edge;
  g_photons_live.previous_edge_valid = true;

  photons_compiler_barrier();
  g_photons_live.generation++;  // commit: even generation
}

// ============================================================================
// Coherent toy snapshots
// ============================================================================

bool photons_toy_capture_snapshot(photons_toy_capture_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_photons_live.generation;
    if (before & 1U) continue;

    photons_compiler_barrier();
    const photons_toy_capture_t snapshot = g_photons_live.capture;
    photons_compiler_barrier();

    const uint32_t after = g_photons_live.generation;
    if (before == after && !(after & 1U)) {
      *out = snapshot;
      return true;
    }
  }
}

bool photons_toy_fragment_snapshot(photons_toy_fragment_t* out) {
  if (!out) return false;
  *out = g_last_fragment;
  return true;
}

// ============================================================================
// PHOTONS_FRAGMENT toy publication
// ============================================================================

static Payload photons_fragment_payload(const photons_toy_fragment_t& f) {
  Payload p;

  p.add("schema", "PHOTONS_FRAGMENT_TOY_V0");
  p.add("sequence", f.sequence);
  p.add("publish_count", f.publish_count);

  p.add("edge_count_total", f.edge_count_total);
  p.add("edges_this_second", f.edges_this_second);

  p.add("last_edge_sequence", f.capture.last_edge_sequence);
  p.add("last_pps_sequence", f.capture.last_pps_sequence);
  p.add("last_dwt_at_edge", f.capture.last_dwt_at_edge);
  p.add("last_isr_entry_dwt_raw", f.capture.last_isr_entry_dwt_raw);
  p.add("isr_entry_to_edge_correction_cycles",
        f.capture.isr_entry_to_edge_correction_cycles);

  p.add("interval_valid", f.capture.interval_valid);
  p.add("last_interval_cycles", f.capture.last_interval_cycles);
  p.add("min_interval_cycles", f.capture.min_interval_cycles);
  p.add("max_interval_cycles", f.capture.max_interval_cycles);

  p.add("prediction_valid", f.capture.prediction_valid);
  p.add("prediction_cycles", f.capture.prediction_cycles);
  p.add("residual_cycles", f.capture.residual_cycles);

  p.add("interrupt_irq_count", f.interrupt_irq_count);
  p.add("interrupt_callback_count", f.interrupt_callback_count);
  p.add("interrupt_callback_missing_count",
        f.interrupt_callback_missing_count);
  p.add("interrupt_inactive_edge_count", f.interrupt_inactive_edge_count);
  p.add("interrupt_source_pin", f.interrupt_source_pin);
  p.add("interrupt_last_callback_wall_cycles",
        f.interrupt_last_callback_wall_cycles);
  p.add("interrupt_max_callback_wall_cycles",
        f.interrupt_max_callback_wall_cycles);

  return p;
}

static void photons_fragment_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) return;

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  photons_toy_fragment_t fragment{};
  fragment.sequence = ++g_fragment_sequence;
  fragment.publish_count = g_publish_count + 1U;

  fragment.edge_count_total = capture.edge_count;
  fragment.edges_this_second =
      capture.edge_count - g_last_published_edge_count;

  fragment.capture = capture;

  fragment.interrupt_irq_count = interrupt_diag.irq_count;
  fragment.interrupt_callback_count = interrupt_diag.callback_count;
  fragment.interrupt_callback_missing_count =
      interrupt_diag.callback_missing_count;
  fragment.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  fragment.interrupt_source_pin = interrupt_diag.source_pin;
  fragment.interrupt_last_callback_wall_cycles =
      interrupt_diag.last_callback_wall_cycles;
  fragment.interrupt_max_callback_wall_cycles =
      interrupt_diag.max_callback_wall_cycles;

  // Author the foreground snapshot before transport admission so REPORT can
  // show exactly what PHOTONS attempted to publish.
  g_last_fragment = fragment;
  g_last_published_edge_count = capture.edge_count;

  Payload payload = photons_fragment_payload(fragment);
  if (publish("PHOTONS_FRAGMENT", payload)) {
    g_publish_count++;
    g_last_fragment.publish_count = g_publish_count;
  } else {
    g_publish_reject_count++;
  }
}

// ============================================================================
// Initialization
// ============================================================================

void process_photons_init(void) {
  if (g_initialized) return;

  g_photons_live = photons_live_state_t{};
  g_last_fragment = photons_toy_fragment_t{};
  g_fragment_sequence = 0U;
  g_publish_count = 0U;
  g_publish_reject_count = 0U;
  g_last_published_edge_count = 0U;

  interrupt_photodiode_subscription_t subscription{};
  subscription.on_edge = photons_on_photodiode_edge;
  subscription.user_data = nullptr;

  g_subscription_ok = interrupt_photodiode_subscribe(subscription);
  g_interrupt_started =
      g_subscription_ok &&
      interrupt_start(interrupt_subscriber_kind_t::PHOTODIODE);

  g_fragment_timer = timepop_arm(
      PHOTONS_FRAGMENT_PERIOD_NS,
      true,
      photons_fragment_tick,
      nullptr,
      "PHOTONS_FRAGMENT");

  g_initialized = true;
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_report(const Payload& /*args*/) {
  photons_toy_capture_t capture{};
  (void)photons_toy_capture_snapshot(&capture);

  photons_toy_fragment_t fragment{};
  (void)photons_toy_fragment_snapshot(&fragment);

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  Payload p;

  p.add("initialized", g_initialized);
  p.add("subscription_ok", g_subscription_ok);
  p.add("interrupt_started", g_interrupt_started);
  p.add("fragment_timer_armed",
        g_fragment_timer != TIMEPOP_INVALID_HANDLE);

  p.add("capture_edge_count", capture.edge_count);
  p.add("capture_last_edge_sequence", capture.last_edge_sequence);
  p.add("capture_last_pps_sequence", capture.last_pps_sequence);
  p.add("capture_last_dwt_at_edge", capture.last_dwt_at_edge);
  p.add("capture_last_isr_entry_dwt_raw", capture.last_isr_entry_dwt_raw);
  p.add("capture_isr_entry_to_edge_correction_cycles",
        capture.isr_entry_to_edge_correction_cycles);

  p.add("capture_interval_valid", capture.interval_valid);
  p.add("capture_last_interval_cycles", capture.last_interval_cycles);
  p.add("capture_min_interval_cycles", capture.min_interval_cycles);
  p.add("capture_max_interval_cycles", capture.max_interval_cycles);

  p.add("capture_prediction_valid", capture.prediction_valid);
  p.add("capture_prediction_cycles", capture.prediction_cycles);
  p.add("capture_residual_cycles", capture.residual_cycles);

  p.add("fragment_sequence", fragment.sequence);
  p.add("fragment_publish_count", g_publish_count);
  p.add("fragment_publish_reject_count", g_publish_reject_count);
  p.add("fragment_edge_count_total", fragment.edge_count_total);
  p.add("fragment_edges_this_second", fragment.edges_this_second);

  p.add("interrupt_subscribed", interrupt_diag.subscribed);
  p.add("interrupt_active", interrupt_diag.active);
  p.add("interrupt_irq_count", interrupt_diag.irq_count);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("interrupt_inactive_edge_count",
        interrupt_diag.inactive_edge_count);
  p.add("interrupt_source_pin", interrupt_diag.source_pin);
  p.add("interrupt_last_callback_wall_cycles",
        interrupt_diag.last_callback_wall_cycles);
  p.add("interrupt_max_callback_wall_cycles",
        interrupt_diag.max_callback_wall_cycles);

  return p;
}

// ============================================================================
// Registration
// ============================================================================

static const process_command_entry_t PHOTONS_COMMANDS[] = {
  { "REPORT", cmd_report },
  { nullptr, nullptr }
};

static const process_vtable_t PHOTONS_PROCESS = {
  .process_id = "PHOTONS",
  .commands = PHOTONS_COMMANDS,
  .subscriptions = nullptr,
};

void process_photons_register(void) {
  process_register("PHOTONS", &PHOTONS_PROCESS);
}
