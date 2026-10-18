#pragma once

#include <stdint.h>

// Physical PD OUT HIGH samples, independent of flight-time acceptance. A sample
// is triggered in the photodiode ISR and collected by Priority-16 software handoff.
// Counts are raw 12-bit ADC codes; delay measurements bracket software triggers,
// not the ADC aperture.
static constexpr uint32_t PHOTONS_PD_OUT_HIGH_WINDOW_MS = 60000U;

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
