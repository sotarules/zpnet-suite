#include <Arduino.h>
#include <stddef.h>

#include "imxrt.h"
#include "config.h"
#include "gpt_count.h"
#include "dwt_clock.h"
#include "event_bus.h"

// =============================================================
// EXISTING INTERNAL STATE (UNCHANGED)
// =============================================================

// GNSS edge gates
static volatile bool start_edge_seen = false;
static volatile bool end_edge_seen   = false;

// Synthetic DWT state (64-bit)
static uint32_t dwt_last = 0;
static uint64_t dwt_acc  = 0;

// =============================================================
// SYNTHETIC DWT — MAINTENANCE PUMP (UNCHANGED)
// =============================================================
// Must be called periodically (<< 7 seconds)

void dwt_pump(void) {
  uint32_t now = dwt_clock_read();
  uint32_t delta = now - dwt_last;   // wrap-safe
  dwt_acc += (uint64_t)delta;
  dwt_last = now;
}

// =============================================================
// SYNTHETIC DWT — READ (NO SIDE EFFECTS)
// =============================================================

static inline uint64_t dwt_read_64(void) {
  return dwt_acc;
}

// =============================================================
// ISR: START edge (phase gate only) — UNCHANGED
// =============================================================

static void gpt_confirm_start_isr() {
  start_edge_seen = true;
  detachInterrupt(GNSS_VCLK_PIN);
}

// =============================================================
// ISR: END edge (phase gate only) — UNCHANGED
// =============================================================

static void gpt_confirm_end_isr() {
  end_edge_seen = true;
  detachInterrupt(GNSS_VCLK_PIN);
}

// =============================================================
// ORIGINAL CONFIRM — SINGLE MONTE CARLO SAMPLE (UNCHANGED CORE)
// =============================================================

uint64_t gpt_count_confirm(
    uint64_t  target_ext_ticks,
    uint64_t* cpu_cycles_out,
    double*   ratio_out,
    int64_t*  error_cycles_out
) {
  // Defensive defaults
  if (cpu_cycles_out)   *cpu_cycles_out   = 0;
  if (ratio_out)        *ratio_out        = 0.0;
  if (error_cycles_out) *error_cycles_out = 0;

  start_edge_seen = false;
  end_edge_seen   = false;

  // Ensure DWT is enabled and primed
  dwt_clock_init();

  dwt_last = dwt_clock_read();
  dwt_acc  = 0;

  // -----------------------------------------------------------
  // PHASE 0 — wait for START GNSS edge
  // -----------------------------------------------------------
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_start_isr, RISING);
  while (!start_edge_seen) {
    dwt_pump();
  }

  uint64_t start_cycles = dwt_read_64();

  // -----------------------------------------------------------
  // PHASE 1 — GNSS pin → GPT2 external clock
  // -----------------------------------------------------------

  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;   // ALT8 = GPT2_CLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  delayMicroseconds(1);

  // -----------------------------------------------------------
  // PHASE 2 — GPT2 external counting
  // -----------------------------------------------------------

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_CR = GPT_CR_CLKSRC(3);
  GPT2_CR |= GPT_CR_EN;

  uint32_t gpt_start = GPT2_CNT;

  while ((uint64_t)(GPT2_CNT - gpt_start) < target_ext_ticks) {
    dwt_pump();
  }

  // -----------------------------------------------------------
  // PHASE 3 — restore GPIO + wait for END GNSS edge
  // -----------------------------------------------------------

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 5;   // ALT5 = GPIO
  delayMicroseconds(1);

  GPT2_CR &= ~GPT_CR_EN;

  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_end_isr, RISING);
  while (!end_edge_seen) {
    dwt_pump();
  }

  uint64_t end_cycles = dwt_read_64();

  // -----------------------------------------------------------
  // PHASE 4 — compute results
  // -----------------------------------------------------------

  uint64_t delta_cpu_cycles = end_cycles - start_cycles;

  if (cpu_cycles_out) {
    *cpu_cycles_out = delta_cpu_cycles;
  }

  if (ratio_out && delta_cpu_cycles > 0) {
    *ratio_out =
        (double)target_ext_ticks /
        (double)delta_cpu_cycles;
  }

  int64_t ideal_cpu_cycles =
      (int64_t)target_ext_ticks * 60;

  if (error_cycles_out) {
    *error_cycles_out =
        (int64_t)delta_cpu_cycles - ideal_cpu_cycles;
  }

  return target_ext_ticks;
}

// =============================================================
// MONTE CARLO τ ACCUMULATION LAYER (NEW, ADDITIVE)
// =============================================================

// ---- histogram configuration limits ----
#define MAX_TAU_BINS 2048

typedef struct {
  int32_t   min_delta;
  int32_t   max_delta;
  uint32_t  bucket_width;
  uint32_t  bin_count;

  uint32_t  total_samples;
  uint32_t  bins[MAX_TAU_BINS];
} tau_histogram_t;

// ------------------------------------------------------------
// Histogram helpers
// ------------------------------------------------------------

static void tau_hist_init(
    tau_histogram_t* h,
    int32_t span_cycles,
    uint32_t bucket_width
) {
  h->min_delta    = -span_cycles;
  h->max_delta    = +span_cycles;
  h->bucket_width = bucket_width;

  h->bin_count =
      (uint32_t)((h->max_delta - h->min_delta + 1) / bucket_width);

  if (h->bin_count > MAX_TAU_BINS) {
    h->bin_count = MAX_TAU_BINS;
  }

  h->total_samples = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    h->bins[i] = 0;
  }
}

static inline void tau_hist_add(
    tau_histogram_t* h,
    int32_t delta_cycles
) {
  if (delta_cycles < h->min_delta ||
      delta_cycles > h->max_delta) {
    return;
  }

  uint32_t idx =
      (uint32_t)((delta_cycles - h->min_delta) / h->bucket_width);

  if (idx >= h->bin_count) return;

  h->bins[idx]++;
  h->total_samples++;
}

static void tau_hist_reduce(
    const tau_histogram_t* h,
    uint64_t ideal_cycles,
    double* tau_out,
    double* mean_delta_out,
    uint32_t* samples_out
) {
  int64_t weighted_sum = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    if (h->bins[i] == 0) continue;

    int32_t center =
        h->min_delta +
        (int32_t)(i * h->bucket_width) +
        (int32_t)(h->bucket_width / 2);

    weighted_sum +=
        (int64_t)center * (int64_t)h->bins[i];
  }

  double mean_delta =
      (h->total_samples > 0)
          ? (double)weighted_sum / (double)h->total_samples
          : 0.0;

  if (mean_delta_out) *mean_delta_out = mean_delta;
  if (samples_out)    *samples_out    = h->total_samples;

  if (tau_out) {
    *tau_out =
        (double)(ideal_cycles + mean_delta) /
        (double)ideal_cycles;
  }
}

// =============================================================
// τ PROFILING WRAPPER (INTERRUPT-ANCHORED)
// =============================================================

bool gpt_tau_profile(
    uint32_t total_seconds,
    uint32_t sample_seconds,
    int32_t  span_cycles,
    uint32_t bucket_width
) {
  if (sample_seconds == 0) return false;
  if (total_seconds < sample_seconds) return false;
  if (bucket_width == 0) return false;

  tau_histogram_t hist;
  tau_hist_init(&hist, span_cycles, bucket_width);

  uint64_t ideal_cycles =
      (uint64_t)sample_seconds *
      10000000ULL *
      60ULL;

  uint32_t elapsed = 0;

  while (elapsed < total_seconds) {

    uint64_t cpu_cycles = 0;
    int64_t  error_cycles = 0;

    // One GNSS-anchored Monte Carlo sample
    gpt_count_confirm(
        (uint64_t)sample_seconds * 10000000ULL,
        &cpu_cycles,
        nullptr,
        &error_cycles
    );

    tau_hist_add(&hist, (int32_t)error_cycles);

    elapsed += sample_seconds;

    // Periodic reduction → τ
    double tau = 0.0;
    double mean_delta = 0.0;
    uint32_t samples = 0;

    tau_hist_reduce(
        &hist,
        ideal_cycles,
        &tau,
        &mean_delta,
        &samples
    );

    // Emit durable event
    String body;
    body += "\"elapsed\":";
    body += elapsed;
    body += ",\"tau\":";

    char buf[32];
    snprintf(buf, sizeof(buf), "%.12f", tau);
    body += buf;

    body += ",\"samples\":";
    body += samples;

    enqueueEvent("TAU_RESULT", body);
  }

  return true;
}
