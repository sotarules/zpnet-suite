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

// Emit cadence (seconds)
#define TAU_EMIT_INTERVAL_SECONDS 10

static uint32_t last_emit_elapsed = 0;

// ------------------------------------------------------------
// Histogram structure
// ------------------------------------------------------------

typedef struct {
  int32_t  min_residual;
  int32_t  max_residual;
  uint32_t bin_count;

  int32_t  gate_cycles;

  uint32_t samples_total;
  uint32_t samples_kept;
  uint32_t samples_dropped;

  uint32_t bins[MAX_TAU_BINS];
} tau_histogram_t;

// ------------------------------------------------------------
// Histogram helpers
// ------------------------------------------------------------

static void tau_hist_init_from_baseline(
    tau_histogram_t* h,
    double baseline_stddev
) {
  int32_t span = (int32_t)(HIST_SIGMA_MULT * baseline_stddev + 0.5);
  if (span < MIN_SPAN_CYCLES) span = MIN_SPAN_CYCLES;
  if (span > MAX_SPAN_CYCLES) span = MAX_SPAN_CYCLES;

  if ((uint32_t)(2 * span + 1) > MAX_TAU_BINS) {
    span = (MAX_TAU_BINS - 1) / 2;
  }

  h->min_residual = -span;
  h->max_residual = +span;
  h->bin_count =
      (uint32_t)(h->max_residual - h->min_residual + 1);

  int32_t gate = (int32_t)(GATE_SIGMA_MULT * baseline_stddev + 0.5);
  if (gate < MIN_SPAN_CYCLES / 2) gate = MIN_SPAN_CYCLES / 2;
  if (gate > span) gate = span;
  h->gate_cycles = gate;

  h->samples_total   = 0;
  h->samples_kept    = 0;
  h->samples_dropped = 0;

  for (uint32_t i = 0; i < h->bin_count; i++) {
    h->bins[i] = 0;
  }

  // ---- diagnostic ----
  {
    String d;
    d += "\"baseline_stddev\":";
    d += baseline_stddev;
    d += ",\"span\":";
    d += span;
    d += ",\"gate\":";
    d += gate;
    d += ",\"bin_count\":";
    d += h->bin_count;
    enqueueEvent("TAU_DIAG_HIST_INIT", d);
  }
}

static inline bool tau_hist_add_residual(
    tau_histogram_t* h,
    int32_t residual
) {
  h->samples_total++;

  {
    String d;
    d += "\"residual\":";
    d += residual;
    d += ",\"gate\":";
    d += h->gate_cycles;
    enqueueEvent("TAU_FOO_RESIDUAL", d);
  }

  if (residual > h->gate_cycles || residual < -h->gate_cycles) {
    h->samples_dropped++;
    enqueueEvent("TAU_FOO_DROP", "\"reason\":\"gate_violation\"");
    return false;
  }

  if (residual < h->min_residual || residual > h->max_residual) {
    h->samples_dropped++;
    enqueueEvent("TAU_FOO_DROP", "\"reason\":\"span_violation\"");
    return false;
  }

  uint32_t idx = (uint32_t)(residual - h->min_residual);
  h->bins[idx]++;
  h->samples_kept++;

  enqueueEvent("TAU_FOO_KEEP", "\"status\":\"accepted\"");
  return true;
}

// ------------------------------------------------------------
// Histogram reduction
// ------------------------------------------------------------

static void tau_hist_reduce(
    const tau_histogram_t* h,
    int32_t baseline_error,
    uint32_t sample_seconds,
    double*  tau_out,
    double*  mean_error_out,
    double*  sem_seconds_out
) {
  if (!h || h->samples_kept == 0) {
    enqueueEvent("TAU_DIAG_REDUCE_EMPTY", "\"note\":\"no_kept_samples\"");
    if (tau_out) *tau_out = 1.0;
    if (mean_error_out) *mean_error_out = 0.0;
    if (sem_seconds_out) *sem_seconds_out = 0.0;
    return;
  }

  int64_t sum = 0;
  for (uint32_t i = 0; i < h->bin_count; i++) {
    if (!h->bins[i]) continue;
    sum += (int64_t)(h->min_residual + (int32_t)i) * h->bins[i];
  }

  double mean_residual =
      (double)sum / (double)h->samples_kept;

  double mean_error =
      (double)baseline_error + mean_residual;

  double var = 0.0;
  for (uint32_t i = 0; i < h->bin_count; i++) {
    if (!h->bins[i]) continue;
    double d =
        (double)(h->min_residual + (int32_t)i) - mean_residual;
    var += d * d * (double)h->bins[i];
  }
  var /= (double)h->samples_kept;

  double stddev_cycles = sqrt(var);
  double sem_cycles =
      stddev_cycles / sqrt((double)h->samples_kept);
  double sem_seconds =
      sem_cycles / 600000000.0;

  int64_t ideal_cycles =
      (int64_t)sample_seconds * 10000000LL * 60LL;

  // -------------------------------
  // CHANGE #2 (τ scaling)
  // -------------------------------
  double tau =
      (double)ideal_cycles /
      (double)((double)ideal_cycles + mean_error * sample_seconds);

  if (tau_out) *tau_out = tau;
  if (mean_error_out) *mean_error_out = mean_error;
  if (sem_seconds_out) *sem_seconds_out = sem_seconds;
}

// ------------------------------------------------------------
// Unified result emitter
// ------------------------------------------------------------

static void emit_tau_result(
    const char* event_type,
    uint32_t elapsed,
    uint32_t sample_seconds,
    int32_t baseline_error,
    double baseline_stddev,
    const tau_histogram_t* hist
) {
  enqueueEvent("TAU_DIAG_EMIT_ENTER", "\"stage\":\"emit_tau_result\"");

  double tau = 1.0;
  double mean_error = 0.0;
  double sem_seconds = 0.0;

  tau_hist_reduce(
      hist,
      baseline_error,
      sample_seconds,
      &tau,
      &mean_error,
      &sem_seconds
  );

  String body;
  body += "\"elapsed\":";
  body += elapsed;
  body += ",\"sample_seconds\":";
  body += sample_seconds;
  body += ",\"baseline_error\":";
  body += baseline_error;
  body += ",\"baseline_stddev\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f", baseline_stddev);
    body += buf;
  }
  body += ",\"windows_total\":";
  body += hist->samples_total;
  body += ",\"samples_kept\":";
  body += hist->samples_kept;
  body += ",\"samples_dropped\":";
  body += hist->samples_dropped;
  body += ",\"mean_error\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f", mean_error);
    body += buf;
  }
  body += ",\"tau\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.12f", tau);
    body += buf;
  }
  body += ",\"sem_seconds\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.12e", sem_seconds);
    body += buf;
  }

  enqueueEvent(event_type, body);
}

// ------------------------------------------------------------
// τ PROFILING WRAPPER
// ------------------------------------------------------------

bool gpt_tau_profile(
    uint32_t total_seconds,
    uint32_t sample_seconds
) {
  enqueueEvent("TAU_DIAG_ENTER", "\"stage\":\"gpt_tau_profile\"");

  if (sample_seconds == 0) return false;
  if (total_seconds < sample_seconds) return false;

  last_emit_elapsed = 0;

  // ---- baseline discovery (1-second units) ----
  uint32_t baseline_samples = 0;
  int32_t  baseline_error = 0;
  double   baseline_stddev = 0.0;

  if (!gpt_discover_standard_error(
        &baseline_samples,
        nullptr, nullptr, nullptr,
        &baseline_error,
        &baseline_stddev)) {
    return false;
  }

  {
    String b;
    b += "\"samples\":";
    b += baseline_samples;
    b += ",\"baseline_error\":";
    b += baseline_error;
    b += ",\"baseline_stddev\":";
    b += baseline_stddev;
    enqueueEvent("TAU_PROFILE_BASELINE", b);
  }

  tau_histogram_t hist;
  tau_hist_init_from_baseline(&hist, baseline_stddev);

  uint32_t elapsed = 0;

  while (elapsed < total_seconds) {
    int64_t err64 = 0;

    enqueueEvent("TAU_DIAG_BEFORE_CONFIRM", "\"note\":\"calling_confirm\"");
    gpt_count_confirm(sample_seconds, nullptr, nullptr, &err64);
    enqueueEvent("TAU_DIAG_AFTER_CONFIRM", "\"note\":\"confirm_returned\"");

    // -------------------------------
    // CHANGE #1 (normalization)
    // -------------------------------
    double err_per_second =
        (double)err64 / (double)sample_seconds;

    int32_t residual =
        (int32_t)(err_per_second - (double)baseline_error);

    {
      String d;
      d += "\"err64\":";
      d += err64;
      d += ",\"err_per_second\":";
      d += err_per_second;
      d += ",\"baseline\":";
      d += baseline_error;
      d += ",\"residual\":";
      d += residual;
      enqueueEvent("TAU_DIAG_RESIDUAL_NORM", d);
    }

    tau_hist_add_residual(&hist, residual);

    elapsed += sample_seconds;

    if ((elapsed - last_emit_elapsed) >= TAU_EMIT_INTERVAL_SECONDS) {
      last_emit_elapsed = elapsed;
      emit_tau_result(
          "TAU_RESULT",
          elapsed,
          sample_seconds,
          baseline_error,
          baseline_stddev,
          &hist
      );
    }
  }

  emit_tau_result(
      "TAU_FINAL_RESULT",
      elapsed,
      sample_seconds,
      baseline_error,
      baseline_stddev,
      &hist
  );

  enqueueEvent("TAU_DIAG_EXIT", "\"stage\":\"complete\"");
  return true;
}

