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
// CONFIRM — SINGLE GNSS-ANCHORED SAMPLE (SECONDS-BASED API)
// =============================================================
//
// Semantics:
//   • Measures CPU state transitions over an exact GNSS-defined
//     interval expressed in seconds
//   • GNSS VCLOCK (10 MHz) defines the authoritative window
//   • CPU acts strictly as observer
//   • Jitter is preserved intentionally
//
// This function is a single Monte Carlo draw.
// =============================================================

uint64_t gpt_count_confirm(
    uint32_t seconds,
    uint64_t* cpu_cycles_out,
    double*   ratio_out,
    int64_t*  error_cycles_out
) {
  // -----------------------------------------------------------
  // Defensive defaults
  // -----------------------------------------------------------
  if (cpu_cycles_out)   *cpu_cycles_out   = 0;
  if (ratio_out)        *ratio_out        = 0.0;
  if (error_cycles_out) *error_cycles_out = 0;

  if (seconds == 0) return 0;

  // GNSS VCLOCK = 10 MHz
  uint64_t target_ext_ticks =
      (uint64_t)seconds * 10000000ULL;

  start_edge_seen = false;
  end_edge_seen   = false;

  // -----------------------------------------------------------
  // Ensure DWT is enabled and primed
  // -----------------------------------------------------------
  dwt_clock_init();

  dwt_last = dwt_clock_read();
  dwt_acc  = 0;

  // ===========================================================
  // PHASE 0 — wait for START GNSS edge
  // ===========================================================
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_start_isr, RISING);
  while (!start_edge_seen) {
    dwt_pump();
  }

  uint64_t start_cycles = dwt_read_64();

  // ===========================================================
  // PHASE 1 — GNSS pin → GPT2 external clock
  // ===========================================================

  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;   // ALT8 = GPT2_CLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  delayMicroseconds(1);

  // ===========================================================
  // PHASE 2 — GPT2 external counting
  // ===========================================================

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_CR = GPT_CR_CLKSRC(3);
  GPT2_CR |= GPT_CR_EN;

  uint32_t gpt_start = GPT2_CNT;

  while ((uint64_t)(GPT2_CNT - gpt_start) < target_ext_ticks) {
    dwt_pump();
  }

  // ===========================================================
  // PHASE 3 — restore GPIO + wait for END GNSS edge
  // ===========================================================

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 5;   // ALT5 = GPIO
  delayMicroseconds(1);

  GPT2_CR &= ~GPT_CR_EN;

  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_end_isr, RISING);
  while (!end_edge_seen) {
    dwt_pump();
  }

  uint64_t end_cycles = dwt_read_64();

  // ===========================================================
  // PHASE 4 — compute results
  // ===========================================================

  uint64_t delta_cpu_cycles =
      end_cycles - start_cycles;

  if (cpu_cycles_out) {
    *cpu_cycles_out = delta_cpu_cycles;
  }

  if (ratio_out && delta_cpu_cycles > 0) {
    *ratio_out =
        (double)target_ext_ticks /
        (double)delta_cpu_cycles;
  }

  int64_t ideal_cpu_cycles =
      (int64_t)target_ext_ticks * 60LL;

  if (error_cycles_out) {
    *error_cycles_out =
        (int64_t)delta_cpu_cycles - ideal_cpu_cycles;
  }

  return target_ext_ticks;
}

// =============================================================
// STANDARD ERROR DISCOVERY (SELF-CALIBRATING)
// =============================================================

#define BASELINE_WINDOW     64
#define CONVERGENCE_EPSILON 1     // cycles
#define CONVERGENCE_RUNS    5

static int cmp_int32(const void* a, const void* b) {
  int32_t x = *(const int32_t*)a;
  int32_t y = *(const int32_t*)b;
  return (x > y) - (x < y);
}

bool gpt_discover_standard_error(
    uint32_t* samples_out,
    int32_t*  min_error_out,
    int32_t*  max_error_out,
    int32_t*  med_error_out,
    int32_t*  std_error_out,
    double*   stddev_out
) {
  int32_t window[BASELINE_WINDOW];
  uint32_t count = 0;

  int32_t global_min = 0;
  int32_t global_max = 0;

  bool have_minmax = false;

  int32_t last_median = 0;
  uint32_t stable_runs = 0;

  while (true) {

    int64_t error_cycles = 0;

    // One GNSS-anchored Monte Carlo draw
    gpt_count_confirm(
        1,        // 1 second samples are fine
        nullptr,
        nullptr,
        &error_cycles
    );

    int32_t e = (int32_t)error_cycles;

    window[count % BASELINE_WINDOW] = e;
    count++;

    if (!have_minmax) {
      global_min = global_max = e;
      have_minmax = true;
    } else {
      if (e < global_min) global_min = e;
      if (e > global_max) global_max = e;
    }

    // Only test convergence once window is full
    if (count < BASELINE_WINDOW) continue;

    // Copy + sort window
    int32_t sorted[BASELINE_WINDOW];
    for (uint32_t i = 0; i < BASELINE_WINDOW; i++) {
      sorted[i] = window[i];
    }

    qsort(sorted, BASELINE_WINDOW, sizeof(int32_t), cmp_int32);

    int32_t median = sorted[BASELINE_WINDOW / 2];

    if (count > BASELINE_WINDOW) {
      if (abs(median - last_median) <= CONVERGENCE_EPSILON) {
        stable_runs++;
        if (stable_runs >= CONVERGENCE_RUNS) {

          // ---- final statistics ----
          double mean = 0.0;
          for (uint32_t i = 0; i < BASELINE_WINDOW; i++) {
            mean += (double)sorted[i];
          }
          mean /= (double)BASELINE_WINDOW;

          double var = 0.0;
          for (uint32_t i = 0; i < BASELINE_WINDOW; i++) {
            double d = (double)sorted[i] - mean;
            var += d * d;
          }
          var /= (double)BASELINE_WINDOW;

          if (samples_out)   *samples_out   = count;
          if (min_error_out) *min_error_out = global_min;
          if (max_error_out) *max_error_out = global_max;
          if (med_error_out) *med_error_out = median;
          if (std_error_out) *std_error_out = (int32_t)median;
          if (stddev_out)    *stddev_out    = sqrt(var);

          return true;
        }
      } else {
        stable_runs = 0;
      }
    }

    last_median = median;
  }
}


// =============================================================
// MONTE CARLO τ ACCUMULATION LAYER (1-CYCLE BINS, SELF-CALIBRATING)
// =============================================================

// ---- histogram configuration limits ----
#define MAX_TAU_BINS 2048

// Envelope policy (baseline-derived)
#define HIST_SIGMA_MULT       4.0   // histogram span = ±4σ
#define GATE_SIGMA_MULT       3.0   // hard outlier gate = ±3σ
#define MIN_SPAN_CYCLES       64    // defensive minimum span
#define MAX_SPAN_CYCLES       1000  // defensive cap (fits in MAX_TAU_BINS)

// Telemetry: avoid spamming heavy stats every sample if desired
// (Set to 1 to emit every sample. Set to >1 to decimate.)
#define TAU_EMIT_INTERVAL_SECONDS 10

static uint32_t last_emit_elapsed = 0;

typedef struct {
  // Histogram domain: residual cycles in [min_residual, max_residual]
  int32_t  min_residual;
  int32_t  max_residual;
  uint32_t bin_count;    // == (max_residual - min_residual + 1)

  // Fixed gate (absolute residual)
  int32_t  gate_cycles;

  // Counts
  uint32_t samples_total;   // windows attempted
  uint32_t samples_kept;    // accepted into hist
  uint32_t samples_dropped; // gated out

  // Histogram bins (1 cycle per bin)
  uint32_t bins[MAX_TAU_BINS];
} tau_histogram_t;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

static void tau_hist_init_from_baseline(
    tau_histogram_t* h,
    double baseline_stddev
) {
  // ---- choose span (±4σ) ----
  int32_t span = (int32_t)(HIST_SIGMA_MULT * baseline_stddev + 0.5);
  if (span < MIN_SPAN_CYCLES) span = MIN_SPAN_CYCLES;
  if (span > MAX_SPAN_CYCLES) span = MAX_SPAN_CYCLES;

  // Ensure bin_count fits in MAX_TAU_BINS
  // bin_count = 2*span + 1
  if ((uint32_t)(2 * span + 1) > MAX_TAU_BINS) {
    span = (MAX_TAU_BINS - 1) / 2;
  }

  h->min_residual = -span;
  h->max_residual = +span;
  h->bin_count    = (uint32_t)(h->max_residual - h->min_residual + 1);

  // ---- choose hard gate (±3σ) ----
  int32_t gate = (int32_t)(GATE_SIGMA_MULT * baseline_stddev + 0.5);
  if (gate < MIN_SPAN_CYCLES / 2) gate = MIN_SPAN_CYCLES / 2;
  if (gate > span) gate = span; // never exceed hist domain
  h->gate_cycles = gate;

  h->samples_total   = 0;
  h->samples_kept    = 0;
  h->samples_dropped = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    h->bins[i] = 0;
  }
}

static inline bool tau_hist_add_residual(
    tau_histogram_t* h,
    int32_t residual_cycles
) {
  h->samples_total++;

  // Hard outlier gate
  if (residual_cycles > h->gate_cycles ||
      residual_cycles < -h->gate_cycles) {
    h->samples_dropped++;
    return false;
  }

  // Must also fit histogram span
  if (residual_cycles < h->min_residual ||
      residual_cycles > h->max_residual) {
    h->samples_dropped++;
    return false;
  }

  uint32_t idx = (uint32_t)(residual_cycles - h->min_residual);
  if (idx >= h->bin_count) {
    h->samples_dropped++;
    return false;
  }

  h->bins[idx]++;
  h->samples_kept++;
  return true;
}

static void tau_hist_reduce(
    const tau_histogram_t* h,
    int32_t baseline_error,     // absolute (cycles)
    uint32_t sample_seconds,
    // outputs:
    double*  tau_out,
    double*  mean_residual_out,
    int32_t* med_residual_out,
    int32_t* min_residual_out,
    int32_t* max_residual_out,
    double*  stddev_residual_out
) {
  if (!h || h->samples_kept == 0) {
    if (tau_out) *tau_out = 1.0;
    if (mean_residual_out) *mean_residual_out = 0.0;
    if (med_residual_out) *med_residual_out = 0;
    if (min_residual_out) *min_residual_out = 0;
    if (max_residual_out) *max_residual_out = 0;
    if (stddev_residual_out) *stddev_residual_out = 0.0;
    return;
  }

  // ---- mean residual ----
  int64_t sum = 0;
  for (uint32_t i = 0; i < h->bin_count; i++) {
    uint32_t c = h->bins[i];
    if (!c) continue;
    int32_t residual = h->min_residual + (int32_t)i; // 1 cycle per bin
    sum += (int64_t)residual * (int64_t)c;
  }

  double mean = (double)sum / (double)h->samples_kept;

  // ---- min/max residual (occupied bins) ----
  int32_t min_r = 0;
  int32_t max_r = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    if (h->bins[i]) {
      min_r = h->min_residual + (int32_t)i;
      break;
    }
  }
  for (int32_t i = (int32_t)h->bin_count - 1; i >= 0; i--) {
    if (h->bins[i]) {
      max_r = h->min_residual + i;
      break;
    }
  }

  // ---- median residual ----
  uint32_t target = (h->samples_kept + 1) / 2;
  uint32_t cum = 0;
  int32_t med_r = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    cum += h->bins[i];
    if (cum >= target) {
      med_r = h->min_residual + (int32_t)i;
      break;
    }
  }

  // ---- residual stddev (about mean) ----
  double var = 0.0;
  for (uint32_t i = 0; i < h->bin_count; i++) {
    uint32_t c = h->bins[i];
    if (!c) continue;
    double r = (double)(h->min_residual + (int32_t)i);
    double d = r - mean;
    var += d * d * (double)c;
  }
  var /= (double)h->samples_kept;

  double stddev = sqrt(var);

  // ---- τ from absolute mean error ----
  // absolute mean error = baseline_error + mean_residual
  double mean_error = (double)baseline_error + mean;

  // ideal CPU cycles per sample window:
  //   sample_seconds * 10 MHz * 60
  int64_t ideal_cycles =
      (int64_t)sample_seconds * 10000000LL * 60LL;

  double tau = 1.0;
  if (ideal_cycles > 0) {
    tau = (double)ideal_cycles / (double)((double)ideal_cycles + mean_error);
  }

  if (tau_out) *tau_out = tau;
  if (mean_residual_out) *mean_residual_out = mean;
  if (med_residual_out) *med_residual_out = med_r;
  if (min_residual_out) *min_residual_out = min_r;
  if (max_residual_out) *max_residual_out = max_r;
  if (stddev_residual_out) *stddev_residual_out = stddev;
}

// =============================================================
// τ PROFILING WRAPPER (SELF-CALIBRATING, 1-CYCLE HISTOGRAM)
// =============================================================

bool gpt_tau_profile(
    uint32_t total_seconds,
    uint32_t sample_seconds
) {
  if (sample_seconds == 0) return false;
  if (total_seconds < sample_seconds) return false;

  // ------------------------------------------------------------
  // Phase 0: discover baseline (standard error + σ) once
  // ------------------------------------------------------------
  uint32_t baseline_samples = 0;
  int32_t  baseline_min = 0;
  int32_t  baseline_max = 0;
  int32_t  baseline_med = 0;
  int32_t  baseline_std_error = 0;
  double   baseline_stddev = 0.0;

  bool ok = gpt_discover_standard_error(
      &baseline_samples,
      &baseline_min,
      &baseline_max,
      &baseline_med,
      &baseline_std_error,
      &baseline_stddev
  );

  if (!ok) return false;

  // Emit baseline used for this profile (durable trace)
  {
    String b;
    b += "\"samples\":";
    b += baseline_samples;
    b += ",\"min_error\":";
    b += baseline_min;
    b += ",\"max_error\":";
    b += baseline_max;
    b += ",\"med_error\":";
    b += baseline_med;
    b += ",\"std_error\":";
    b += baseline_std_error;
    b += ",\"stddev\":";
    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f", baseline_stddev);
    b += buf;

    enqueueEvent("TAU_PROFILE_BASELINE", b);
  }

  // ------------------------------------------------------------
  // Phase 1: initialize 1-cycle histogram using baseline σ
  // ------------------------------------------------------------
  tau_histogram_t hist;
  tau_hist_init_from_baseline(&hist, baseline_stddev);

  // ------------------------------------------------------------
  // Phase 2: profiling loop
  // ------------------------------------------------------------
  uint32_t elapsed = 0;
  uint32_t iter = 0;

  while (elapsed < total_seconds) {
    iter++;

    int64_t error_cycles_64 = 0;

    // One GNSS-anchored window
    gpt_count_confirm(
        sample_seconds,
        nullptr,
        nullptr,
        &error_cycles_64
    );

    int32_t error_cycles = (int32_t)error_cycles_64;

    // Residual around locked baseline
    int32_t residual = (int32_t)(error_cycles - baseline_std_error);

    bool kept = tau_hist_add_residual(&hist, residual);

    elapsed += sample_seconds;

    // ----------------------------------------------------------
    // Reduce + emit (rich telemetry)
    // ----------------------------------------------------------
    if ((elapsed - last_emit_elapsed) >= TAU_EMIT_INTERVAL_SECONDS) {
      last_emit_elapsed = elapsed;

      double tau = 1.0;
      double mean_residual = 0.0;
      int32_t med_residual = 0;
      int32_t min_residual = 0;
      int32_t max_residual = 0;
      double stddev_residual = 0.0;

      tau_hist_reduce(
          &hist,
          baseline_std_error,
          sample_seconds,
          &tau,
          &mean_residual,
          &med_residual,
          &min_residual,
          &max_residual,
          &stddev_residual
      );

      // Derived absolute stats
      double mean_error = (double)baseline_std_error + mean_residual;
      int32_t med_error = baseline_std_error + med_residual;
      int32_t min_error = baseline_std_error + min_residual;
      int32_t max_error = baseline_std_error + max_residual;

      // Standard error of mean (cycles) for mean_residual
      double sem_cycles = 0.0;
      if (hist.samples_kept > 0) {
        sem_cycles = stddev_residual / sqrt((double)hist.samples_kept);
      }

      // Convert SEM to seconds (optional; useful)
      // 1 cycle @ 600 MHz ≈ 1/600e6 s
      double sem_seconds = sem_cycles / 600000000.0;

      String body;
      body += "\"elapsed\":";
      body += elapsed;

      body += ",\"sample_seconds\":";
      body += sample_seconds;

      // Raw last sample visibility
      body += ",\"last_error\":";
      body += error_cycles;
      body += ",\"last_residual\":";
      body += residual;
      body += ",\"last_kept\":";
      body += kept ? "true" : "false";

      // Baseline used
      body += ",\"baseline_error\":";
      body += baseline_std_error;
      body += ",\"baseline_stddev\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6f", baseline_stddev);
        body += buf;
      }

      // Histogram domain + gating
      body += ",\"hist_min_residual\":";
      body += hist.min_residual;
      body += ",\"hist_max_residual\":";
      body += hist.max_residual;
      body += ",\"gate_cycles\":";
      body += hist.gate_cycles;
      body += ",\"bin_count\":";
      body += hist.bin_count;

      // Counters
      body += ",\"windows_total\":";
      body += hist.samples_total;
      body += ",\"samples_kept\":";
      body += hist.samples_kept;
      body += ",\"samples_dropped\":";
      body += hist.samples_dropped;

      // Residual distribution stats
      body += ",\"mean_residual\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6f", mean_residual);
        body += buf;
      }
      body += ",\"med_residual\":";
      body += med_residual;
      body += ",\"min_residual\":";
      body += min_residual;
      body += ",\"max_residual\":";
      body += max_residual;
      body += ",\"stddev_residual\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6f", stddev_residual);
        body += buf;
      }

      // Absolute error stats (baseline + residual stats)
      body += ",\"mean_error\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6f", mean_error);
        body += buf;
      }
      body += ",\"med_error\":";
      body += med_error;
      body += ",\"min_error\":";
      body += min_error;
      body += ",\"max_error\":";
      body += max_error;

      // τ + uncertainty
      body += ",\"tau\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.12f", tau);
        body += buf;
      }

      body += ",\"sem_cycles\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6f", sem_cycles);
        body += buf;
      }

      body += ",\"sem_seconds\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.12e", sem_seconds);
        body += buf;
      }

      enqueueEvent("TAU_RESULT", body);
    }
  }

  return true;
}



