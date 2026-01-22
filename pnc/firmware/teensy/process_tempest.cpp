#include <Arduino.h>
#include <stddef.h>

#include "process_tempest.h"
#include "imxrt.h"
#include "clock.h"
#include "config.h"
#include "event_bus.h"
#include "process.h"
#include "smartpop.h"

// ------------------------------------------------------------
// Forward declarations (lifecycle)
// ------------------------------------------------------------
static bool tempest_start(void);
static void tempest_stop(void);

static void tempest_confirm_start_cb(void* context);
static void tempest_confirm_end_cb(void* context);

// =============================================================
// INTERNAL STATE (UNCHANGED)
// =============================================================

// GNSS edge gates
static volatile bool start_edge_seen = false;
static volatile bool end_edge_seen   = false;

// =============================================================
// INTERNAL STATE (SmartPOP-based)
// =============================================================

static volatile bool confirm_done = false;
static volatile bool confirm_active = false;

// Parameters
static uint32_t confirm_seconds = 0;

// Captured times (ns)
static uint64_t confirm_start_ns = 0;
static uint64_t confirm_end_ns   = 0;


// -------------------------------------------------------------
// SmartPOP-aligned START callback
// -------------------------------------------------------------

static void tempest_confirm_start_cb(void* context) {

  uint32_t seconds = (uint32_t)(uintptr_t)context;

  // Capture aligned start time
  confirm_start_ns = clock_dwt_ns_now();

  // Schedule aligned end
  uint32_t ticks = seconds * SMARTPOP_HZ;

  smartpop_end(
    ticks,
    tempest_confirm_end_cb,
    nullptr
  );
}

// -------------------------------------------------------------
// SmartPOP-aligned END callback
// -------------------------------------------------------------
static void tempest_confirm_end_cb(void* /*context*/) {

  confirm_end_ns = clock_dwt_ns_now();

  uint64_t measured_ns = confirm_end_ns - confirm_start_ns;
  uint64_t ideal_ns    = (uint64_t)confirm_seconds * NS_PER_SECOND;
  int64_t  error_ns    = (int64_t)measured_ns - (int64_t)ideal_ns;

  double ratio = 0.0;
  if (measured_ns > 0) {
    ratio = (double)ideal_ns / (double)measured_ns;
  }

  // ---------------------------------------------------------
  // Emit authoritative result
  // ---------------------------------------------------------
  String body;

  body += "\"seconds\":";
  body += confirm_seconds;

  body += ",\"ideal_ns\":";
  body += ideal_ns;

  body += ",\"measured_ns\":";
  body += measured_ns;

  body += ",\"error_ns\":";
  body += error_ns;

  body += ",\"ratio\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.12f", ratio);
    body += buf;
  }

  enqueueEvent("TEMPEST_CONFIRM_RESULT", body);

  // ---------------------------------------------------------
  // Reset state
  // ---------------------------------------------------------
  confirm_active = false;
}

// -------------------------------------------------------------
// Forward declarations (SmartPOP callbacks)
// -------------------------------------------------------------

static void tempest_confirm_start_cb(void* context);
static void tempest_confirm_end_cb(void* context);

static String cmd_confirm(const char* args_json) {

  if (!args_json) {
    return "{\"error\":\"missing args\"}";
  }

  const char* p = strstr(args_json, "\"seconds\"");
  if (!p) {
    return "{\"error\":\"missing seconds\"}";
  }

  p = strchr(p, ':');
  if (!p) {
    return "{\"error\":\"malformed seconds\"}";
  }
  p++;

  uint32_t seconds = (uint32_t)strtoul(p, nullptr, 10);
  if (seconds == 0) {
    return "{\"error\":\"seconds must be > 0\"}";
  }

  // ---------------------------------------------------------
  // Enforce single in-flight confirm
  // ---------------------------------------------------------
  if (confirm_active) {
    return "{\"error\":\"confirm already in progress\"}";
  }

  // ---------------------------------------------------------
  // Arm confirm
  // ---------------------------------------------------------
  confirm_active    = true;
  confirm_seconds   = seconds;
  confirm_start_ns  = 0;
  confirm_end_ns    = 0;

  // Schedule aligned start on next GNSS tick
  smartpop_start(
    tempest_confirm_start_cb,
    (void*)(uintptr_t)seconds
  );

  // Return immediately
  return "{\"status\":\"armed\"}";
}

static String cmd_baseline(const char*) {
  uint32_t samples = 0;
  int32_t  min_e = 0, max_e = 0, med_e = 0, std_e = 0;
  double   stddev = 0.0;

  tempest_discover_standard_error(
      &samples,
      &min_e,
      &max_e,
      &med_e,
      &std_e,
      &stddev
  );

  String r = "{";

  r += "\"samples\":";
  r += samples;

  r += ",\"baseline_error\":";
  r += std_e;

  r += ",\"stddev\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f", stddev);
    r += buf;
  }

  r += "}";

  return r;
}

static String cmd_tau(const char* args) {
  uint32_t seconds = 0;

  const char* p = strstr(args, "seconds=");
  if (!p) {
    return "{\"error\":\"missing seconds\"}";
  }

  seconds = (uint32_t)strtoul(p + 8, nullptr, 10);

  bool ok = tempest_tau_profile(seconds);

  return ok ? "{\"status\":\"ok\"}" : "{\"status\":\"fail\"}";
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t TEMPEST_COMMANDS[] = {
  { "CONFIRM",  cmd_confirm  },
  { "BASELINE", cmd_baseline },
  { "TAU",      cmd_tau      },
};

static const process_vtable_t TEMPEST_PROCESS = {
  .name = "TEMPEST",
  .start = tempest_start,
  .stop  = tempest_stop,
  .query = nullptr,
  .commands = TEMPEST_COMMANDS,
  .command_count = 3,
};

void process_tempest_register(void) {
  process_register(PROCESS_TYPE_TEMPEST, &TEMPEST_PROCESS);
}

static bool tempest_start(void) {
  enqueueEvent("TEMPEST_INIT_ENTER", "\"stage\":\"process_start\"");
  return true;
}

static void tempest_stop(void) {
  enqueueEvent("TEMPEST_STOP", "\"stage\":\"process_stop\"");
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

bool tempest_discover_standard_error(
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
    /*
    tempest_confirm(
        1,        // 1 second samples
        nullptr,
        nullptr,
        &error_cycles
    );
    */

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

#define MAX_TAU_BINS 2048

#define HIST_SIGMA_MULT       4.0
#define GATE_SIGMA_MULT       3.0
#define MIN_SPAN_CYCLES       64
#define MAX_SPAN_CYCLES       1000

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
  h->bin_count = (uint32_t)(h->max_residual - h->min_residual + 1);

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

  double mean_residual = (double)sum / (double)h->samples_kept;
  double mean_error = (double)baseline_error + mean_residual;

  double var = 0.0;
  for (uint32_t i = 0; i < h->bin_count; i++) {
    if (!h->bins[i]) continue;
    double d = (double)(h->min_residual + (int32_t)i) - mean_residual;
    var += d * d * (double)h->bins[i];
  }
  var /= (double)h->samples_kept;

  double stddev_cycles = sqrt(var);
  double sem_cycles = stddev_cycles / sqrt((double)h->samples_kept);
  double sem_seconds = sem_cycles / 600000000.0;

  int64_t ideal_cycles = 600000000LL;  // 1 second * 60 MHz

  double tau =
      ((double)ideal_cycles + mean_error) / (double)ideal_cycles;

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
    int32_t baseline_error,
    double baseline_stddev,
    const tau_histogram_t* hist
) {
  enqueueEvent("TAU_DIAG_EMIT_ENTER", "\"stage\":\"emit_tau_result\"");

  double tau = 1.0;
  double mean_error = 0.0;
  double sem_seconds = 0.0;

  tau_hist_reduce(hist, baseline_error, &tau, &mean_error, &sem_seconds);

  String body;
  body += "\"elapsed\":";
  body += elapsed;
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

// =============================================================
// τ PROFILING WRAPPER
// =============================================================

bool tempest_tau_profile(uint32_t total_seconds) {
  enqueueEvent("TAU_DIAG_ENTER", "\"stage\":\"tempest_tau_profile\"");

  if (total_seconds < 1) return false;

  last_emit_elapsed = 0;

  // ---- baseline discovery (1-second units) ----
  uint32_t baseline_samples = 0;
  int32_t  baseline_error   = 0;
  double   baseline_stddev  = 0.0;

  if (!tempest_discover_standard_error(
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
    uint64_t cpu_cycles  = 0;
    double   ratio       = 0.0;
    int64_t  error_cycles = 0;

    //tempest_confirm(1, &cpu_cycles, &ratio, &error_cycles);

    int32_t residual = (int32_t)(error_cycles - baseline_error);

    {
      String d;
      d += "\"cpu_cycles\":";
      d += cpu_cycles;
      d += ",\"error_cycles\":";
      d += error_cycles;
      d += ",\"baseline_error\":";
      d += baseline_error;
      d += ",\"residual\":";
      d += residual;
      d += ",\"ratio\":";
      {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.12f", ratio);
        d += buf;
      }
      enqueueEvent("TAU_DIAG_SAMPLE", d);
    }

    tau_hist_add_residual(&hist, residual);

    elapsed += 1;

    if ((elapsed - last_emit_elapsed) >= TAU_EMIT_INTERVAL_SECONDS) {
      last_emit_elapsed = elapsed;
      emit_tau_result(
          "TAU_RESULT",
          elapsed,
          baseline_error,
          baseline_stddev,
          &hist
      );
    }
  }

  emit_tau_result(
      "TAU_FINAL_RESULT",
      elapsed,
      baseline_error,
      baseline_stddev,
      &hist
  );

  enqueueEvent("TAU_DIAG_EXIT", "\"stage\":\"complete\"");
  return true;
}
