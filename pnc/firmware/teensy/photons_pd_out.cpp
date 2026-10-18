#include "integer_only.h"
#include "photons_pd_out.h"
#include "config.h"
#include <Arduino.h>

// Pin 40 / AD_B1_04 is channel 9 on either ADC. Own ADC2 so the existing
// foreground analogRead(20) (ADC1) cannot overwrite a pulse conversion.
static_assert(PHOTODIODE_ANALOG_PIN == 40, "PD OUT ADC2 channel requires pin 40");
static constexpr uint32_t PD_CHANNEL = 9U;

enum class pd_phase_t : uint32_t { IDLE, SAMPLING_HIGH };
static volatile pd_phase_t g_pd_phase = pd_phase_t::IDLE;
// Computed before acquisition, using the configured CPU clock.
static uint32_t g_pd_high_delay_cycles;
static uint32_t g_pd_conversion_timeout_cycles;
static uint32_t g_pd_isr_budget_cycles;
static volatile uint32_t g_pd_busy_edges;
static volatile uint32_t g_pd_high_start_count;
static volatile uint32_t g_pd_high_complete_count;
// Priority 0 owns one pending conversion; Priority 16 alone writes totals.
// The GPIO first-edge gate closes before the handoff can run. Foreground is
// required to run with Priority 16 unmasked at every batch/report boundary.
static uint32_t g_pd_pending_entry_dwt, g_pd_pending_start_dwt;
static uint32_t g_pd_pending_bank;
// Foreground takes a seqlock copy of completed totals.
static volatile uint32_t g_pd_generation;
static volatile photons_pd_out_batch_t g_pd_totals;
static photons_pd_out_batch_t g_pd_previous;

// RAM2 is not startup-zeroed. Init explicitly establishes metadata; only the
// stored_rows prefix is ever read. Neither rotation nor reporting copies rows.
alignas(32) static DMAMEM photons_pd_out_capture_t g_pd_capture[2];
static uint32_t g_pd_write_bank;
static uint32_t g_pd_completed_bank = 1U;
static uint32_t g_pd_capture_sequence;

// Foreground only: fixed 2 KiB RAM2 ring, no ISR work and no heap allocation.
// Publication is a 1 Hz recurring service that coalesces missed appointments.
// 64 cells allow whole-fragment rounding and ordinary boundary jitter. Exhaustion
// is a cadence contract failure; never silently shorten the requested population.
static constexpr uint32_t PD_WINDOW_CAPACITY = 64U;
struct pd_window_cell_t {
  uint64_t n, sum, squares;
  uint32_t elapsed_ms;
};
static_assert(sizeof(pd_window_cell_t) == 32U, "HIGH window RAM2 geometry changed");
alignas(32) static DMAMEM pd_window_cell_t g_pd_window_cells[PD_WINDOW_CAPACITY];
static uint32_t g_pd_window_head, g_pd_window_cut_ms;
static photons_pd_out_window_t g_pd_window;

static void pd_reset_window(uint32_t cut_ms) {
  // RAM2 need not be zeroed: only the live prefix, written below, is ever read.
  g_pd_window = photons_pd_out_window_t{};
  g_pd_window_head = 0U;
  g_pd_window_cut_ms = cut_ms;
}

static photons_pd_out_window_t pd_update_window(
    const photons_pd_out_batch_t& batch, uint32_t cut_ms) {
  const uint32_t elapsed = cut_ms - g_pd_window_cut_ms; // millis rollover
  if (elapsed == 0U) __builtin_trap();
  g_pd_window_cut_ms = cut_ms;
  g_pd_window.elapsed_ms += elapsed;
  g_pd_window.sample_count += batch.sample_count;
  g_pd_window.high_sum += batch.high_sum;
  g_pd_window.high_sum_squares += batch.high_sum_squares;
  // Trim before appending: a long delayed fragment may supersede the whole ring.
  while (g_pd_window.fragments != 0U) {
    const auto& oldest = g_pd_window_cells[g_pd_window_head];
    if (g_pd_window.elapsed_ms - oldest.elapsed_ms < PHOTONS_PD_OUT_HIGH_WINDOW_MS)
      break;
    g_pd_window.elapsed_ms -= oldest.elapsed_ms;
    g_pd_window.sample_count -= oldest.n;
    g_pd_window.high_sum -= oldest.sum;
    g_pd_window.high_sum_squares -= oldest.squares;
    g_pd_window_head = (g_pd_window_head + 1U) % PD_WINDOW_CAPACITY;
    --g_pd_window.fragments;
  }
  if (g_pd_window.fragments == PD_WINDOW_CAPACITY) __builtin_trap();
  const uint32_t tail = (g_pd_window_head + g_pd_window.fragments) % PD_WINDOW_CAPACITY;
  g_pd_window_cells[tail] = {batch.sample_count, batch.high_sum,
                             batch.high_sum_squares, elapsed};
  ++g_pd_window.fragments;
  g_pd_window.completed_at_ms = cut_ms;
  return g_pd_window;
}

static void pd_reset_capture(photons_pd_out_capture_t& capture, uint32_t cut_ms) {
  capture.capture_id = 0U;
  capture.started_at_ms = cut_ms;
  capture.completed_at_ms = cut_ms;
  capture.stored_rows = 0U;
  capture.rows_seen = 0U;
  capture.first_entry_dwt = 0U;
  for (uint32_t i = 0U; i < PHOTONS_PD_OUT_CAPTURE_SAMPLES; ++i) {
    capture.start_cycles[i] = 0U;
    capture.read_cycles[i] = 0U;
  }
}

static void pd_rotate_capture(uint32_t cut_ms, bool publish) {
  // Foreground is serialized with the report. Priority 16 must be serviceable:
  // a Priority-0 trigger and its Priority-16 completion then both finish before
  // foreground resumes. No in-flight conversion can cross this bank cut.
  uint32_t primask, basepri;
  __asm__ volatile ("mrs %0, primask" : "=r" (primask));
  __asm__ volatile ("mrs %0, basepri" : "=r" (basepri));
  if (primask != 0U || basepri != 0U || g_pd_phase != pd_phase_t::IDLE)
    __builtin_trap();
  const uint32_t old = __atomic_load_n(&g_pd_write_bank, __ATOMIC_RELAXED);
  const uint32_t next = old ^ 1U;
  pd_reset_capture(g_pd_capture[next], cut_ms);
  __atomic_store_n(&g_pd_write_bank, next, __ATOMIC_RELEASE);
  auto& completed = g_pd_capture[old];
  completed.completed_at_ms = cut_ms;
  if (publish) {
    if (++g_pd_capture_sequence == 0U) __builtin_trap();
    completed.capture_id = g_pd_capture_sequence;
  } else {
    completed.capture_id = 0U; // recovery/start discards the preceding partial second
  }
  g_pd_completed_bank = old;
}

const photons_pd_out_capture_t& photons_pd_out_capture_latest() {
  return g_pd_capture[g_pd_completed_bank];
}

uint32_t photons_pd_out_isr_budget_cycles() {
  return g_pd_isr_budget_cycles;
}

void photons_pd_out_high_isr(uint32_t edge_entry_dwt) {
  if (g_pd_phase != pd_phase_t::IDLE) {
    g_pd_busy_edges++;
    __builtin_trap(); // never overwrite an unread physical measurement
  }
  g_pd_phase = pd_phase_t::SAMPLING_HIGH;
  g_pd_high_start_count++;
  // Preserve the trigger's minimum delay from the immutable lap timestamp.
  while ((uint32_t)(ARM_DWT_CYCCNT - edge_entry_dwt) < g_pd_high_delay_cycles) {}
  const uint32_t start = ARM_DWT_CYCCNT;
  ADC2_HC0 = ADC_HC_ADCH(PD_CHANNEL); // AIEN clear: no hardware completion IRQ
  g_pd_pending_entry_dwt = edge_entry_dwt;
  g_pd_pending_start_dwt = start;
  g_pd_pending_bank = __atomic_load_n(&g_pd_write_bank, __ATOMIC_ACQUIRE);
  // process_interrupt publishes the software pend after closing the edge gate.
  // No completion polling, RAM2 row storage or moment arithmetic at Priority 0.
}

void photons_pd_out_complete_isr() {
  if (g_pd_phase != pd_phase_t::SAMPLING_HIGH) __builtin_trap();
  const uint32_t entry = g_pd_pending_entry_dwt;
  const uint32_t start = g_pd_pending_start_dwt;
  while (!(ADC2_HS & ADC_HS_COCO0)) {
    // Recheck completion after reading the deadline: a clock ISR may have
    // preempted either read. A ready result is legal even after a long preemption.
    if ((uint32_t)(ARM_DWT_CYCCNT - start) >= g_pd_conversion_timeout_cycles &&
        !(ADC2_HS & ADC_HS_COCO0)) __builtin_trap();
  }
  const uint32_t high = ADC2_R0;
  const uint32_t end = ARM_DWT_CYCCNT;
  const uint32_t delay = start - entry;
  auto& capture = g_pd_capture[g_pd_pending_bank];
  const uint32_t row = capture.stored_rows;
  capture.rows_seen++;
  if (row < PHOTONS_PD_OUT_CAPTURE_ROWS) {
    capture.samples[row][0] = (uint16_t)high;
    if (row == 0U) {
      capture.first_entry_dwt = entry;
      capture.start_cycles[0] = delay;
      // Includes handoff latency and higher-priority preemption, NOT aperture.
      capture.read_cycles[0] = end - start;
    }
    capture.stored_rows = row + 1U;
  }
  g_pd_generation++;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_totals.sample_count++;
  g_pd_totals.high_sum += high;
  g_pd_totals.high_sum_squares += high * high;
  g_pd_totals.high_delay_cycles_sum += delay;
  __asm__ volatile ("dmb" ::: "memory");
  g_pd_generation++;
  g_pd_high_complete_count++;
  g_pd_phase = pd_phase_t::IDLE;
  __asm__ volatile ("dsb" ::: "memory");
}

void photons_pd_out_init() {
  if (F_CPU_ACTUAL == 0U) __builtin_trap();
  g_pd_high_delay_cycles = (uint32_t)(
      ((uint64_t)F_CPU_ACTUAL * PHOTONS_PD_OUT_HIGH_DELAY_NS + 999999999ULL) /
      1000000000ULL); // round up: the target is a minimum
  g_pd_conversion_timeout_cycles = (uint32_t)(
      ((uint64_t)F_CPU_ACTUAL * PHOTONS_PD_OUT_CONVERSION_TIMEOUT_NS + 999999999ULL) /
      1000000000ULL);
  g_pd_isr_budget_cycles = (uint32_t)(
      ((uint64_t)F_CPU_ACTUAL * PHOTONS_PD_OUT_ISR_BUDGET_NS + 999999999ULL) /
      1000000000ULL);
  const uint32_t now_ms = millis();
  pd_reset_capture(g_pd_capture[0], now_ms);
  pd_reset_capture(g_pd_capture[1], now_ms);
  g_pd_write_bank = 0U;
  g_pd_completed_bank = 1U;
  g_pd_capture_sequence = 0U;
  pd_reset_window(now_ms);

  pinMode(PHOTODIODE_ANALOG_PIN, INPUT_DISABLE);
  // Core startup has calibrated ADC2. Use single 12-bit conversions, shortest
  // acquisition, no hardware averaging. IPG/8 = 18.75 MHz at the 600 MHz build.
  // analogReadResolution/Averaging affect BOTH ADCs and must precede this setup.
  NVIC_DISABLE_IRQ(IRQ_ADC2);
  __asm__ volatile ("dsb" ::: "memory");
  ADC2_HC0 = ADC_HC_ADCH(31);
  ADC2_CFG = ADC_CFG_ADICLK(1) | ADC_CFG_ADIV(2) |
             ADC_CFG_MODE(2) | ADC_CFG_ADHSC;
  ADC2_GC = 0U;
  (void)ADC2_R0;
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  interrupt_adc_high_configure(); // software-pended vector, priority 16
}

void photons_pd_out_stop() {
  // Caller has stopped PHOTODIODE. Pending completion must run before this
  // foreground callback; never cancel an outstanding measurement.
  if (g_pd_phase != pd_phase_t::IDLE) __builtin_trap();
  __asm__ volatile ("dsb" ::: "memory");
  ADC2_HC0 = ADC_HC_ADCH(31);
  (void)ADC2_R0;
  NVIC_CLEAR_PENDING(IRQ_ADC2);
  __asm__ volatile ("dsb" ::: "memory");
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

void photons_pd_out_rebase(uint32_t cut_ms) {
  pd_rotate_capture(cut_ms, false);
  g_pd_previous = pd_snapshot();
  pd_reset_window(cut_ms);
}

photons_pd_out_batch_t photons_pd_out_take_batch(uint32_t cut_ms) {
  pd_rotate_capture(cut_ms, true);
  // The raw bank cut and cumulative scalar snapshot are separate foreground
  // operations. An intervening ISR may put its row in the new raw bank while
  // its first-sample scalar enters this fragment; neither population loses data.
  const auto now = pd_snapshot();
  photons_pd_out_batch_t out;
  out.sample_count = now.sample_count - g_pd_previous.sample_count;
  out.high_sum = now.high_sum - g_pd_previous.high_sum;
  out.high_sum_squares = now.high_sum_squares - g_pd_previous.high_sum_squares;
  out.high_delay_cycles_sum = now.high_delay_cycles_sum - g_pd_previous.high_delay_cycles_sum;
  out.busy_edges = now.busy_edges - g_pd_previous.busy_edges;
  out.high_start_count_total = g_pd_high_start_count;
  out.high_complete_count_total = g_pd_high_complete_count;
  out.phase = (uint32_t)g_pd_phase;
  out.adc_hc0 = ADC2_HC0;
  out.adc_hs = ADC2_HS;
  out.adc_cfg = ADC2_CFG;
  out.adc_gc = ADC2_GC;
  out.adc_irq_enabled = NVIC_IS_ENABLED(IRQ_ADC2) ? 1U : 0U;
  out.high_window = pd_update_window(out, cut_ms);
  g_pd_previous = now;
  return out;
}
