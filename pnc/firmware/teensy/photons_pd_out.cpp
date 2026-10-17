#include "integer_only.h"
#include "photons_pd_out.h"
#include "config.h"
#include <Arduino.h>

// Pin 40 / AD_B1_04 is channel 9 on either ADC. Own ADC2 so the existing
// foreground analogRead(20) (ADC1) cannot overwrite a pulse conversion.
static_assert(PHOTODIODE_ANALOG_PIN == 40, "PD OUT ADC2 channel requires pin 40");
static constexpr uint32_t PD_CHANNEL = 9U;
static constexpr uint32_t PD_IRQ_PRIORITY = 64U; // CLOCKS and capture preempt us
// Teensy 1.59 startup routes the 24 MHz oscillator through PERCLK with /1.
// Select that peripheral clock (1); direct oscillator input (5) did not count.
// Keep the shared CCM clock selection/divider unchanged.
static constexpr uint32_t PD_TIMER_CONFIG =
    GPT_CR_CLKSRC(1) | GPT_CR_ENMOD | GPT_CR_FRR;
static_assert(PHOTONS_PD_OUT_LOW_DELAY_NS % 125U == 0U,
              "PD OUT delay must be an exact number of 24 MHz ticks");
static constexpr uint32_t PD_TIMER_TICKS =
    PHOTONS_PD_OUT_LOW_DELAY_NS / 125U * 3U;

enum class pd_phase_t : uint32_t { IDLE, SAMPLING_HIGH, SAMPLING_LOW };
static volatile pd_phase_t g_pd_phase = pd_phase_t::IDLE;
static uint32_t g_pd_edge_dwt;
static uint32_t g_pd_high_dwt;
static uint32_t g_pd_low_dwt;
static uint32_t g_pd_high;
static volatile uint32_t g_pd_busy_edges;
static volatile uint32_t g_pd_high_not_ready;
static volatile uint32_t g_pd_high_start_count;
static volatile uint32_t g_pd_timer_irq_count;
static volatile uint32_t g_pd_low_irq_count;
// Only the LOW completion ISR writes totals; foreground takes a seqlock copy.
static volatile uint32_t g_pd_generation;
static volatile photons_pd_out_batch_t g_pd_totals;
static photons_pd_out_batch_t g_pd_previous;

static void pd_low_timer_isr() {
  GPT2_CR = PD_TIMER_CONFIG; // one-shot: stop before acknowledging the compare
  GPT2_SR = GPT_SR_OF1;
  g_pd_timer_irq_count++;
  if (g_pd_phase != pd_phase_t::SAMPLING_HIGH) __builtin_trap();
  if (!(ADC2_HS & ADC_HS_COCO0)) {
    // A hardware conversion missed the requested deadline. Do not invent a
    // HIGH value or include half a pair in the means; expose the observation.
    ADC2_HC0 = ADC_HC_ADCH(31);
    g_pd_high_not_ready++;
    g_pd_phase = pd_phase_t::IDLE;
    __asm__ volatile ("dsb" ::: "memory");
    return;
  }
  g_pd_high = ADC2_R0; // reading the result clears HIGH's completion flag
  g_pd_phase = pd_phase_t::SAMPLING_LOW;
  g_pd_low_dwt = ARM_DWT_CYCCNT;
  ADC2_HC0 = ADC_HC_AIEN | ADC_HC_ADCH(PD_CHANNEL);
  __asm__ volatile ("dsb" ::: "memory");
}

static void pd_low_complete_isr() {
  if (g_pd_phase != pd_phase_t::SAMPLING_LOW || !(ADC2_HS & ADC_HS_COCO0))
    __builtin_trap();
  const uint32_t low = ADC2_R0;
  g_pd_low_irq_count++;
  g_pd_generation++;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_totals.pair_count++;
  g_pd_totals.high_sum += g_pd_high;
  g_pd_totals.low_sum += low;
  g_pd_totals.high_delay_cycles_sum += (uint32_t)(g_pd_high_dwt - g_pd_edge_dwt);
  g_pd_totals.low_delay_cycles_sum += (uint32_t)(g_pd_low_dwt - g_pd_high_dwt);
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_generation++;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_phase = pd_phase_t::IDLE; // publish availability after the complete pair
  __asm__ volatile ("dsb" ::: "memory");
}

void photons_pd_out_high_isr(uint32_t edge_entry_dwt) {
  if (g_pd_phase != pd_phase_t::IDLE) {
    g_pd_busy_edges++;
    return;
  }
  g_pd_phase = pd_phase_t::SAMPLING_HIGH;
  g_pd_edge_dwt = edge_entry_dwt;
  g_pd_high_dwt = ARM_DWT_CYCCNT;
  ADC2_HC0 = ADC_HC_ADCH(PD_CHANNEL); // start HIGH; no conversion wait, no IRQ
  GPT2_CR = PD_TIMER_CONFIG | GPT_CR_EN; // ENMOD restarts the counter at zero
  g_pd_high_start_count++;
}

void photons_pd_out_init() {
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT_DISABLE);
  // Core startup has calibrated ADC2. Use a single 12-bit conversion, shortest
  // acquisition, no hardware averaging. IPG/8 = 18.75 MHz at the 600 MHz build.
  // analogReadResolution/Averaging affect BOTH ADCs and must precede this setup.
  ADC2_HC0 = ADC_HC_ADCH(31);
  ADC2_CFG = ADC_CFG_ADICLK(1) | ADC_CFG_ADIV(2) |
             ADC_CFG_MODE(2) | ADC_CFG_ADHSC;
  ADC2_GC = 0U;
  (void)ADC2_R0;

  // Ordinary TimePop slots are foreground-only and require >=6.4 us lead.
  // GPT2 provides this separate 2 us diagnostic one-shot without changing
  // TimePop, QTimer1, or any CLOCKS timer. Its IRQ latency is measured below.
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) |
               CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);
  GPT2_CR = GPT_CR_SWR;
  while (GPT2_CR & GPT_CR_SWR) {} // boot-only peripheral reset
  GPT2_PR = 0U;
  GPT2_CR = PD_TIMER_CONFIG;
  GPT2_OCR1 = PD_TIMER_TICKS;
  GPT2_SR = 0x3fU;
  GPT2_IR = GPT_IR_OF1IE;
  attachInterruptVector(IRQ_GPT2, pd_low_timer_isr);
  attachInterruptVector(IRQ_ADC2, pd_low_complete_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, PD_IRQ_PRIORITY);
  NVIC_SET_PRIORITY(IRQ_ADC2, PD_IRQ_PRIORITY);
  NVIC_CLEAR_PENDING(IRQ_GPT2);
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  NVIC_ENABLE_IRQ(IRQ_GPT2);
  NVIC_ENABLE_IRQ(IRQ_ADC2);
}

void photons_pd_out_stop() {
  // Caller has stopped PHOTODIODE. Mask only the two diagnostic IRQs, never
  // the clock sources; no new HIGH can interleave this cancellation.
  NVIC_DISABLE_IRQ(IRQ_GPT2);
  NVIC_DISABLE_IRQ(IRQ_ADC2);
  __asm__ volatile ("dsb" ::: "memory");
  GPT2_CR = PD_TIMER_CONFIG;
  GPT2_SR = 0x3fU;
  ADC2_HC0 = ADC_HC_ADCH(31);
  (void)ADC2_R0;
  NVIC_CLEAR_PENDING(IRQ_GPT2);
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  g_pd_phase = pd_phase_t::IDLE;
  __asm__ volatile ("dsb" ::: "memory");
  NVIC_ENABLE_IRQ(IRQ_GPT2);
  NVIC_ENABLE_IRQ(IRQ_ADC2);
}

static photons_pd_out_batch_t pd_snapshot() {
  photons_pd_out_batch_t out;
  for (;;) {
    const uint32_t before = g_pd_generation;
    if (before & 1U) continue;
    __asm__ volatile ("dmb" ::: "memory");
    out.pair_count = g_pd_totals.pair_count;
    out.high_sum = g_pd_totals.high_sum;
    out.low_sum = g_pd_totals.low_sum;
    out.high_delay_cycles_sum = g_pd_totals.high_delay_cycles_sum;
    out.low_delay_cycles_sum = g_pd_totals.low_delay_cycles_sum;
    __asm__ volatile ("dmb" ::: "memory");
    if (before == g_pd_generation) break;
  }
  out.busy_edges = g_pd_busy_edges;
  out.high_not_ready = g_pd_high_not_ready;
  return out;
}

void photons_pd_out_rebase() {
  g_pd_previous = pd_snapshot();
}

photons_pd_out_batch_t photons_pd_out_take_batch() {
  const auto now = pd_snapshot();
  photons_pd_out_batch_t out;
  out.pair_count = now.pair_count - g_pd_previous.pair_count;
  out.high_sum = now.high_sum - g_pd_previous.high_sum;
  out.low_sum = now.low_sum - g_pd_previous.low_sum;
  out.high_delay_cycles_sum = now.high_delay_cycles_sum - g_pd_previous.high_delay_cycles_sum;
  out.low_delay_cycles_sum = now.low_delay_cycles_sum - g_pd_previous.low_delay_cycles_sum;
  out.busy_edges = now.busy_edges - g_pd_previous.busy_edges;
  out.high_not_ready = now.high_not_ready - g_pd_previous.high_not_ready;
  out.high_start_count_total = g_pd_high_start_count;
  out.timer_irq_count_total = g_pd_timer_irq_count;
  out.low_irq_count_total = g_pd_low_irq_count;
  out.phase = (uint32_t)g_pd_phase;
  out.gpt_cr = GPT2_CR;
  out.gpt_cnt = GPT2_CNT;
  out.gpt_sr = GPT2_SR;
  out.gpt_ir = GPT2_IR;
  out.adc_hc0 = ADC2_HC0;
  out.adc_hs = ADC2_HS;
  out.adc_cfg = ADC2_CFG;
  out.adc_gc = ADC2_GC;
  out.irq_enabled_mask = (NVIC_IS_ENABLED(IRQ_GPT2) ? 1U : 0U) |
                         (NVIC_IS_ENABLED(IRQ_ADC2) ? 2U : 0U);
  g_pd_previous = now;
  return out;
}
