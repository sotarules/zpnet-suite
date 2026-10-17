#pragma once

#include <stdint.h>

// Physical PD OUT HIGH samples, independent of flight-time acceptance. A sample
// belongs to the fragment in which its ADC2 completion ISR records it. Counts are raw 12-bit
// ADC codes; delay measurements bracket software triggers, not the ADC aperture.
struct photons_pd_out_batch_t {
  uint64_t sample_count = 0;
  uint64_t high_sum = 0;
  uint64_t high_sum_squares = 0; // exact raw-code second moment for HIGH witness
  uint64_t high_delay_cycles_sum = 0;
  uint32_t busy_edges = 0;
  // Lifetime stage counters and live foreground register observations.
  // These are diagnostic observations, not an atomic hardware snapshot.
  uint32_t high_start_count_total = 0;
  uint32_t high_irq_count_total = 0;
  uint32_t phase = 0; // 0=IDLE, 1=SAMPLING_HIGH
  uint32_t adc_hc0 = 0, adc_hs = 0, adc_cfg = 0, adc_gc = 0;
  uint32_t adc_irq_enabled = 0;
};

// Boot, before the detector is activated. Reserves ADC2 channel 9 and its completion IRQ.
void photons_pd_out_init();
// Physical GPIO ISR only, immediately after its immutable DWT timestamp.
void photons_pd_out_high_isr(uint32_t edge_entry_dwt);
// Foreground, after the detector is stopped. Cancels an unfinished HIGH sample.
void photons_pd_out_stop();
// Foreground publisher boundaries; no masking of CLOCKS/PHOTODIODE interrupts.
void photons_pd_out_rebase();
photons_pd_out_batch_t photons_pd_out_take_batch();
