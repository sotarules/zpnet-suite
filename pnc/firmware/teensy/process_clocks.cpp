#include "process_clocks.h"

#include "clock.h"
#include "events.h"
#include "payload.h"
#include "process.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"

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
// PPS CAPTURE — CLEAN, HARDWARE-TRUSTING
// ================================================================
//
// Semantics:
//   • Hardware has already decided truth (pull-up / conditioning)
//   • ISR captures authoritative DWT timestamp
//   • No filtering, no plausibility checks
//   • Publishing is deferred to non-ISR context
//
static volatile bool     pps_enabled     = false;
static volatile bool     pps_followon_scheduled = false;
static volatile uint64_t pps_count       = 0;
static volatile uint64_t last_pps_dwt_ns = 0;

// ------------------------------------------------------------
// Forward declarations (ISR and follow-on callback)
// ------------------------------------------------------------

static void pps_isr(void);
static void pps_followon_cb(timepop_ctx_t*, void*);


// ------------------------------------------------------------
// PPS ISR — MINIMAL, NON-BLOCKING
// ------------------------------------------------------------
static void pps_isr(void) {

  if (!pps_enabled) return;

  // --- capture authoritative truth ---
  last_pps_dwt_ns = clock_dwt_ns_now();
  pps_count++;

  // --- schedule continuation exactly once ---
  if (!pps_followon_scheduled) {
    pps_followon_scheduled = true;

    timepop_arm(
      TIMEPOP_CLASS_ASAP,   // <<< key change
      false,                // one-shot
      pps_followon_cb,
      nullptr,
      "pps-followon"
    );
  }
}

static void pps_followon_cb(timepop_ctx_t*, void*) {

  Payload ev;
  ev.add("pps_count", pps_count);
  ev.add("dwt_ns",    last_pps_dwt_ns);

  publish("PPS", ev);

  // Allow next PPS to schedule again
  pps_followon_scheduled = false;
}


// ================================================================
// Canonical clock report builder
// ================================================================
static Payload build_clock_report_payload(void) {

  Payload p;

  // Raw authoritative clocks
  const uint64_t dwt_ns  = clock_dwt_ns_now();
  const uint64_t gnss_ns = clock_gnss_ns_now();
  const uint64_t ocxo_ns = clock_ocxo_ns_now();

  p.add("dwt_ns",  dwt_ns);
  p.add("gnss_ns", gnss_ns);
  p.add("ocxo_ns", ocxo_ns);

  // Elapsed wall time since last GNSS zero
  uint64_t zero_ns    = clock_gnss_zero_ns();
  uint64_t elapsed_ns = (gnss_ns >= zero_ns) ? (gnss_ns - zero_ns) : 0;
  uint64_t elapsed_s  = elapsed_ns / 1000000000ULL;

  char hms[16];
  format_hms(elapsed_s, hms, sizeof(hms));

  p.add("elapsed_hms",     hms);
  p.add("elapsed_seconds", elapsed_s);

  // Drift + tempo metrics
  if (gnss_ns > 0) {

    int64_t drift_dwt_ns  = (int64_t)dwt_ns  - (int64_t)gnss_ns;
    int64_t drift_ocxo_ns = (int64_t)ocxo_ns - (int64_t)gnss_ns;

    p.add("dwt_drift_ns",  drift_dwt_ns);
    p.add("ocxo_drift_ns", drift_ocxo_ns);

    double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;

    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);

    double ppb_dwt  = ((double)drift_dwt_ns  / (double)gnss_ns) * 1e9;
    double ppb_ocxo = ((double)drift_ocxo_ns / (double)gnss_ns) * 1e9;

    p.add_fmt("dwt_ppb",  "%.6f", ppb_dwt);
    p.add_fmt("ocxo_ppb", "%.6f", ppb_ocxo);

  } else {
    p.add("dwt_drift_ns",  0);
    p.add("ocxo_drift_ns", 0);
    p.add("tau_dwt",       0.0);
    p.add("tau_ocxo",      0.0);
    p.add("dwt_ppb",       0.0);
    p.add("ocxo_ppb",      0.0);
  }

  p.add("gnss_lock", digitalRead(GNSS_LOCK_PIN));

  return p;
}

// ================================================================
// Commands
// ================================================================

// REPORT — pull-based introspection
static Payload cmd_report(const Payload&) {
  return build_clock_report_payload();
}

// CLEAR — zero all synthetic clocks
static Payload cmd_clear(const Payload&) {

  clock_zero_all();

  Payload ev;
  ev.add("action", "all_zeroed");
  publish("CLOCKS_CLEAR", ev);

  return ok_payload();
}

static Payload cmd_start_pps(const Payload&) {

  pps_count = 0;
  pps_enabled = true;
  pps_followon_scheduled = false;

  pinMode(GNSS_PPS_PIN, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    pps_isr,
    RISING
  );

  pinMode(GNSS_LOCK_PIN, INPUT);

  Payload ev;
  ev.add("status", "started");
  publish("PPS_STARTED", ev);

  return ok_payload();
}

static Payload cmd_stop_pps(const Payload&) {

  pps_enabled = false;
  detachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN));

  Payload ev;
  ev.add("status", "stopped");
  publish("PPS_STOPPED", ev);

  return ok_payload();
}

// ================================================================
// 1 Hz push publication
// ================================================================
static void clocks_1hz_tick(timepop_ctx_t*, void*) {

  // Publish clock state
  Payload p = build_clock_report_payload();
  publish("CLOCKS/STATE", p);
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "REPORT",    cmd_report    },
  { "CLEAR",     cmd_clear     },
  { "START_PPS", cmd_start_pps },
  { "STOP_PPS",  cmd_stop_pps  },
  { nullptr,     nullptr       }
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = nullptr,
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}

// --------------------------------------------------------------
// Explicit initialization
// --------------------------------------------------------------
void process_clocks_init(void) {
  timepop_arm(
    TIMEPOP_CLASS_CLOCKS,
    true,
    clocks_1hz_tick,
    nullptr,
    "clocks"
  );
}
