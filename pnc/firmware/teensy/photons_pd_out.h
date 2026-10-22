#pragma once

#include <stdint.h>

// Physical PD OUT HIGH samples, independent of flight-time acceptance. A sample
// is triggered in the photodiode ISR and collected by Priority-16 software handoff.
// Counts are raw 12-bit ADC codes; delay measurements bracket software triggers,
// not the ADC aperture.
static constexpr uint32_t PHOTONS_PD_OUT_HIGH_WINDOW_MS = 60000U;

// LOW is an ADC1 witness; the existing ADC2 HIGH population/correction is intact.
// Outcomes describe observations, not voltage-validity votes. No delta filter.
enum class photons_low_high_outcome_t : uint32_t {
  PAIRED, NO_PREDICTION, ARM_TOO_LATE, SCHEDULE_MISSED, CANCELLED,
  NO_HIGH_AT_CLOSE, ADC_TIMEOUT, LOW_NOT_BEFORE_HIGH, HIGH_OUTSIDE_WINDOW,
  MULTIPLE_HIGH, COUNT
};
static constexpr uint32_t PHOTONS_LOW_HIGH_MOMENTS = 10U;
struct photons_low_high_moment_t {
  int64_t sum = 0;
  uint64_t squares = 0;
  int32_t minimum = 0, maximum = 0;
};
struct photons_low_high_row_t {
  uint64_t launch_id = 0;
  uint32_t launch_dwt = 0, predicted_cycles = 0, target_dwt = 0;
  uint32_t low_trigger_dwt = 0, low_read_dwt = 0;
  uint32_t action_isr_entry_dwt = 0, action_dispatch_dwt = 0;
  uint32_t edge_dwt = 0, high_trigger_dwt = 0, high_read_dwt = 0;
  uint32_t low_triggers = 0, low_samples = 0, high_samples = 0;
  uint32_t low_counts = 0, high_counts = 0, launch_bracket_cycles = 0;
  uint32_t lead_ns = 0, close_reason = 0;
  photons_low_high_outcome_t outcome = photons_low_high_outcome_t::NO_PREDICTION;
};
// One foreground-owned rolling population of paired laps. No ISR work, heap,
// voltage filtering, or independent LOW/HIGH membership. Partial windows are
// published with their actual count; missing readings never become zero volts.
static constexpr uint32_t PHOTONS_LOW_HIGH_WINDOW_DEFAULT = 1000U;
static constexpr uint32_t PHOTONS_LOW_HIGH_WINDOW_MAX = 2048U;
struct photons_low_high_sum_t {
  int64_t sum = 0;
  uint64_t squares = 0;
};
struct photons_low_high_window_t {
  uint32_t target_laps = PHOTONS_LOW_HIGH_WINDOW_DEFAULT;
  uint32_t count = 0, generation = 0;
  uint64_t paired_total = 0, last_launch_id = 0;
  uint32_t last_pair_at_ms = 0, observed_at_ms = 0, low_lead_ns = 0;
  photons_low_high_sum_t moments[3]; // LOW, HIGH, HIGH-LOW
};
void photons_pd_out_low_high_set_window(uint32_t laps);
photons_low_high_window_t photons_pd_out_low_high_window();

struct photons_low_high_batch_t {
  uint32_t attempts = 0, low_samples = 0;
  uint32_t lead_min_ns = 0, lead_max_ns = 0;
  uint64_t low_sum = 0, low_squares = 0;
  uint32_t outcomes[(uint32_t)photons_low_high_outcome_t::COUNT] = {};
  // All moments and both cross-products use exactly the PAIRED population:
  // LOW, HIGH, HIGH-LOW, raw flight, LOW trigger lead, prediction error,
  // HIGH trigger delay, LOW trigger-to-read, LOW trigger lateness,
  // HIGH trigger-to-read. All timing moments are DWT cycles.
  photons_low_high_moment_t paired[PHOTONS_LOW_HIGH_MOMENTS];
  int64_t low_raw_cross = 0, high_raw_cross = 0;
  photons_low_high_row_t last, last_unpaired;
  photons_low_high_window_t window; // frozen at the fragment cut
};
enum class photons_low_high_close_t : uint32_t {
  NEXT_LAUNCH, FRAGMENT, ADC1_USER, STOP
};
// Foreground only. Prepare with detector capture withdrawn, before MOD HIGH.
void photons_pd_out_low_prepare(uint64_t launch_id, uint32_t predicted_cycles,
                               uint32_t receive_max_cycles);
void photons_pd_out_low_launch(uint32_t launch_dwt, uint32_t bracket_cycles);
void photons_pd_out_low_finish(photons_low_high_close_t reason);
uint32_t photons_pd_out_low_lead_ns();
void photons_pd_out_low_set_lead_ns(uint32_t ns);

// Foreground-owned population, frozen into each fragment. Whole-fragment suffix
// covering at least 60 s after warm-up; exact moments weight every physical sample.
struct photons_pd_out_window_t {
  uint64_t sample_count = 0, high_sum = 0, high_sum_squares = 0;
  uint64_t elapsed_ms = 0;
  uint32_t fragments = 0;
  uint32_t completed_at_ms = 0;
};

struct photons_pd_out_batch_t {
  uint64_t sample_count = 0;
  uint64_t high_sum = 0;
  uint64_t high_sum_squares = 0; // exact raw-code second moment for HIGH witness
  uint64_t high_delay_cycles_sum = 0;
  uint32_t busy_edges = 0;
  // Lifetime stage counters and live foreground register observations.
  // These are diagnostic observations, not an atomic hardware snapshot.
  uint32_t high_start_count_total = 0;
  uint32_t high_complete_count_total = 0;
  uint32_t phase = 0; // 0=IDLE, 1=SAMPLING_HIGH
  uint32_t adc_hc0 = 0, adc_hs = 0, adc_cfg = 0, adc_gc = 0;
  uint32_t adc_irq_enabled = 0;
  photons_pd_out_window_t high_window; // populated only at foreground batch cut
  photons_low_high_batch_t low_high;
};

static constexpr uint32_t PHOTONS_PD_OUT_CAPTURE_ROWS = 1000U;
static constexpr uint32_t PHOTONS_PD_OUT_CAPTURE_SAMPLES = 1U;

// Two instances live in RAM2: Priority-16-owned current second and foreground-owned
// completed second. Raw codes are retained in acquisition order, without voting.
struct photons_pd_out_capture_t {
  uint32_t capture_id;
  uint32_t started_at_ms, completed_at_ms;
  uint32_t stored_rows, rows_seen;
  uint32_t first_entry_dwt;
  uint32_t start_cycles[PHOTONS_PD_OUT_CAPTURE_SAMPLES];
  uint32_t read_cycles[PHOTONS_PD_OUT_CAPTURE_SAMPLES];
  uint16_t samples[PHOTONS_PD_OUT_CAPTURE_ROWS][PHOTONS_PD_OUT_CAPTURE_SAMPLES];
};

// Boot, before activation. ADC2 hardware interrupt generation stays disabled.
void photons_pd_out_init();
// Physical GPIO ISR only, after its immutable timestamp and entry diagnostics.
void photons_pd_out_high_isr(uint32_t edge_entry_dwt);
// Software-pended Priority 16 only. Clock captures can preempt this wait.
void photons_pd_out_complete_isr();
// process_interrupt owns vector installation and interrupt priority.
void interrupt_adc_high_configure();
// Foreground, after the detector is stopped and its software handoff drained.
void photons_pd_out_stop();
// Foreground publisher boundaries; no masking of CLOCKS/PHOTODIODE interrupts.
// Rebase starts a fresh HIGH window; acquisition pauses/recovery time are excluded.
// This voltage witness is not part of the deterministic lap-recovery accumulators.
void photons_pd_out_rebase(uint32_t cut_ms);
photons_pd_out_batch_t photons_pd_out_take_batch(uint32_t cut_ms);
// Borrow only within one foreground callback. No latch across report requests.
const photons_pd_out_capture_t& photons_pd_out_capture_latest();
// Extra residence reserved after the receive horizon by quiet launch admission.
uint32_t photons_pd_out_isr_budget_cycles();
