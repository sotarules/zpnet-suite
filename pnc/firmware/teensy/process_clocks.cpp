#include "process_clocks.h"

#include "clock.h"
#include "events.h"
#include "payload.h"
#include "process.h"

#include <Arduino.h>

// ================================================================
// Helpers
// ================================================================

static void format_hms(uint64_t seconds, char* out, size_t out_sz) {
  uint64_t h = seconds / 3600;
  uint64_t m = (seconds % 3600) / 60;
  uint64_t s = seconds % 60;

  snprintf(out, out_sz, "%02llu:%02llu:%02llu",
           (unsigned long long)h,
           (unsigned long long)m,
           (unsigned long long)s);
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// REPORT — extended synthetic clock introspection
// ------------------------------------------------------------
static Payload cmd_report(const Payload& /*args*/) {

  Payload p;

  // ----------------------------------------------------------
  // Raw authoritative clocks
  // ----------------------------------------------------------
  const uint64_t dwt_ns  = clock_dwt_ns_now();
  const uint64_t gnss_ns = clock_gnss_ns_now();
  const uint64_t ocxo_ns = clock_ocxo_ns_now();

  p.add("dwt_ns", dwt_ns);
  p.add("gnss_ns", gnss_ns);
  p.add("ocxo_ns", ocxo_ns);

  // ----------------------------------------------------------
  // Elapsed wall time since last zero (GNSS-anchored)
  // ----------------------------------------------------------
  uint64_t zero_ns    = clock_gnss_zero_ns();
  uint64_t elapsed_ns = (gnss_ns >= zero_ns) ? (gnss_ns - zero_ns) : 0;
  uint64_t elapsed_s  = elapsed_ns / 1000000000ULL;

  char hms[16];
  format_hms(elapsed_s, hms, sizeof(hms));

  p.add("elapsed_hms", hms);
  p.add("elapsed_seconds", elapsed_s);

  // ----------------------------------------------------------
  // Drift + tempo metrics (relative to GNSS)
  // ----------------------------------------------------------
  if (gnss_ns > 0) {

    // Absolute drift (signed)
    int64_t drift_dwt_ns  = (int64_t)dwt_ns  - (int64_t)gnss_ns;
    int64_t drift_ocxo_ns = (int64_t)ocxo_ns - (int64_t)gnss_ns;

    p.add("dwt_drift_ns",  drift_dwt_ns);
    p.add("ocxo_drift_ns", drift_ocxo_ns);

    // Tau (dimensionless)
    double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;

    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);

    // Parts per billion (tempo offset)
    double ppb_dwt  = ((double)drift_dwt_ns  / (double)gnss_ns) * 1e9;
    double ppb_ocxo = ((double)drift_ocxo_ns / (double)gnss_ns) * 1e9;

    p.add_fmt("dwt_ppb",  "%.6f", ppb_dwt);
    p.add_fmt("ocxo_ppb", "%.6f", ppb_ocxo);

  } else {
    // GNSS not yet valid
    p.add("dwt_drift_ns",  0);
    p.add("ocxo_drift_ns", 0);
    p.add("tau_dwt",       0.0);
    p.add("tau_ocxo",      0.0);
    p.add("dwt_ppb",       0.0);
    p.add("ocxo_ppb",      0.0);
  }

  return p;
}

// ------------------------------------------------------------
// CLEAR — zero all synthetic clocks
// ------------------------------------------------------------
static Payload cmd_clear(const Payload&) {

  clock_zero_all();

  // Durable fact
  {
    Payload ev;
    ev.add("action", "all_zeroed");
    enqueueEvent("CLOCKS_CLEAR", ev);
  }

  // Command acknowledgment
  return ok_payload();
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "REPORT", cmd_report },
  { "CLEAR",  cmd_clear  },
  { nullptr,  nullptr }   // sentinel
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = nullptr,
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}
