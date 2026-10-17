#include "integer_only.h"
#include "photons_pd_out.h"
#include "config.h"
#include <Arduino.h>

// Pin 40 / AD_B1_04 is channel 9 on either ADC. Own ADC2 so the existing
// foreground analogRead(20) (ADC1) cannot overwrite a pulse conversion.
static_assert(PHOTODIODE_ANALOG_PIN == 40, "PD OUT ADC2 channel requires pin 40");
static constexpr uint32_t PD_CHANNEL = 9U;
static constexpr uint32_t PD_IRQ_PRIORITY = 64U; // CLOCKS and capture preempt us

enum class pd_phase_t : uint32_t { IDLE, SAMPLING_HIGH };
static volatile pd_phase_t g_pd_phase = pd_phase_t::IDLE;
// Computed at init, before acquisition, using the configured CPU clock.
static uint32_t g_pd_high_delay_cycles;
static uint32_t g_pd_edge_dwt;
static uint32_t g_pd_high_dwt;
static volatile uint32_t g_pd_busy_edges;
static volatile uint32_t g_pd_high_start_count;
static volatile uint32_t g_pd_high_irq_count;
// Only the HIGH completion ISR writes totals; foreground takes a seqlock copy.
static volatile uint32_t g_pd_generation;
static volatile photons_pd_out_batch_t g_pd_totals;
static photons_pd_out_batch_t g_pd_previous;

static void pd_high_complete_isr() {
  if (g_pd_phase != pd_phase_t::SAMPLING_HIGH || !(ADC2_HS & ADC_HS_COCO0))
    __builtin_trap();
  const uint32_t high = ADC2_R0; // reading the result clears completion
  g_pd_high_irq_count++;
  g_pd_generation++;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_totals.sample_count++;
  g_pd_totals.high_sum += high;
  // A 12-bit code squared fits uint32_t. No floating point in this ISR.
  g_pd_totals.high_sum_squares += high * high;
  g_pd_totals.high_delay_cycles_sum += (uint32_t)(g_pd_high_dwt - g_pd_edge_dwt);
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_generation++;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_phase = pd_phase_t::IDLE; // publish availability after the complete sample
  __asm__ volatile ("dsb" ::: "memory");
}

void photons_pd_out_high_isr(uint32_t edge_entry_dwt) {
  if (g_pd_phase != pd_phase_t::IDLE) {
    g_pd_busy_edges++;
    return;
  }
  g_pd_phase = pd_phase_t::SAMPLING_HIGH;
  g_pd_edge_dwt = edge_entry_dwt;
  // Count work already elapsed since entry; do not add a fresh 200 ns here.
  // Unsigned subtraction also handles the DWT counter wrapping in this wait.
  // IRQ priorities/masks and the immutable lap timestamp remain unchanged.
  while ((uint32_t)(ARM_DWT_CYCCNT - edge_entry_dwt) < g_pd_high_delay_cycles) {}
  g_pd_high_dwt = ARM_DWT_CYCCNT;
  ADC2_HC0 = ADC_HC_AIEN | ADC_HC_ADCH(PD_CHANNEL); // existing ADC2 IRQ completes HIGH
  g_pd_high_start_count++;
}

void photons_pd_out_init() {
  if (F_CPU_ACTUAL == 0U) __builtin_trap();
  g_pd_high_delay_cycles = (uint32_t)(
      ((uint64_t)F_CPU_ACTUAL * PHOTONS_PD_OUT_HIGH_DELAY_NS + 999999999ULL) /
      1000000000ULL); // round up: the target is a minimum
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT_DISABLE);
  // Core startup has calibrated ADC2. Use a single 12-bit conversion, shortest
  // acquisition, no hardware averaging. IPG/8 = 18.75 MHz at the 600 MHz build.
  // analogReadResolution/Averaging affect BOTH ADCs and must precede this setup.
  ADC2_HC0 = ADC_HC_ADCH(31);
  ADC2_CFG = ADC_CFG_ADICLK(1) | ADC_CFG_ADIV(2) |
             ADC_CFG_MODE(2) | ADC_CFG_ADHSC;
  ADC2_GC = 0U;
  (void)ADC2_R0;

  // Reuse the existing ADC2 completion IRQ for HIGH. No delayed LOW sample,
  // timer peripheral, or additional interrupt source is involved.
  attachInterruptVector(IRQ_ADC2, pd_high_complete_isr);
  NVIC_SET_PRIORITY(IRQ_ADC2, PD_IRQ_PRIORITY);
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  NVIC_ENABLE_IRQ(IRQ_ADC2);
}

void photons_pd_out_stop() {
  // Caller has stopped PHOTODIODE. Mask only ADC2 while cancelling any
  // unfinished HIGH; clock-source interrupts remain enabled.
  NVIC_DISABLE_IRQ(IRQ_ADC2);
  __asm__ volatile ("dsb" ::: "memory");
  ADC2_HC0 = ADC_HC_ADCH(31);
  (void)ADC2_R0;
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  g_pd_phase = pd_phase_t::IDLE;
  __asm__ volatile ("dsb" ::: "memory");
  NVIC_ENABLE_IRQ(IRQ_ADC2);
}

static photons_pd_out_batch_t pd_snapshot() {
  photons_pd_out_batch_t out;
  for (;;) {
    const uint32_t before = g_pd_generation;
    if (before & 1U) continue;
    __asm__ volatile ("dmb" ::: "memory");
    out.sample_count = g_pd_totals.sample_count;
    out.high_sum = g_pd_totals.high_sum;
    out.high_sum_squares = g_pd_totals.high_sum_squares;
    out.high_delay_cycles_sum = g_pd_totals.high_delay_cycles_sum;
    __asm__ volatile ("dmb" ::: "memory");
    if (before == g_pd_generation) break;
  }
  out.busy_edges = g_pd_busy_edges;
  return out;
}

void photons_pd_out_rebase() {
  g_pd_previous = pd_snapshot();
}

photons_pd_out_batch_t photons_pd_out_take_batch() {
  const auto now = pd_snapshot();
  photons_pd_out_batch_t out;
  out.sample_count = now.sample_count - g_pd_previous.sample_count;
  out.high_sum = now.high_sum - g_pd_previous.high_sum;
  out.high_sum_squares = now.high_sum_squares - g_pd_previous.high_sum_squares;
  out.high_delay_cycles_sum = now.high_delay_cycles_sum - g_pd_previous.high_delay_cycles_sum;
  out.busy_edges = now.busy_edges - g_pd_previous.busy_edges;
  out.high_start_count_total = g_pd_high_start_count;
  out.high_irq_count_total = g_pd_high_irq_count;
  out.phase = (uint32_t)g_pd_phase;
  out.adc_hc0 = ADC2_HC0;
  out.adc_hs = ADC2_HS;
  out.adc_cfg = ADC2_CFG;
  out.adc_gc = ADC2_GC;
  out.adc_irq_enabled = NVIC_IS_ENABLED(IRQ_ADC2) ? 1U : 0U;
  g_pd_previous = now;
  return out;
}
