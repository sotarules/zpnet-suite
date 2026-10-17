#pragma once

#include <stdint.h>

// Physical PD OUT pairs, independent of flight-time acceptance. A pair belongs
// to the fragment in which its LOW conversion completes. Counts are raw 12-bit
// ADC codes; delay measurements bracket software triggers, not the ADC aperture.
struct photons_pd_out_batch_t {
  uint64_t pair_count = 0;
  uint64_t high_sum = 0;
  uint64_t high_sum_squares = 0; // exact raw-code second moment for HIGH witness
  uint64_t low_sum = 0;
  uint64_t high_delay_cycles_sum = 0;
  uint64_t low_delay_cycles_sum = 0;
  uint32_t busy_edges = 0;
  uint32_t high_not_ready = 0;
  // Lifetime stage counters and live foreground register observations.
  // These are diagnostic observations, not an atomic hardware snapshot.
  uint32_t high_start_count_total = 0;
  uint32_t timer_irq_count_total = 0;
  uint32_t low_irq_count_total = 0;
  uint32_t phase = 0; // 0=IDLE, 1=SAMPLING_HIGH, 2=SAMPLING_LOW
  uint32_t gpt_cr = 0, gpt_cnt = 0, gpt_sr = 0, gpt_ir = 0;
  uint32_t adc_hc0 = 0, adc_hs = 0, adc_cfg = 0, adc_gc = 0;
  uint32_t irq_enabled_mask = 0; // bit 0=GPT2, bit 1=ADC2
};

// Boot, before the detector is activated. Reserves ADC2 channel 9 and GPT2.
void photons_pd_out_init();
// Physical GPIO ISR only, immediately after its immutable DWT timestamp.
void photons_pd_out_high_isr(uint32_t edge_entry_dwt);
// Foreground, after the detector is stopped. Cancels an unfinished pair.
void photons_pd_out_stop();
// Foreground publisher boundaries; no masking of CLOCKS/PHOTODIODE interrupts.
void photons_pd_out_rebase();
photons_pd_out_batch_t photons_pd_out_take_batch();
