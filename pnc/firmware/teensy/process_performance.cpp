#include "process_performance.h"

#include "payload.h"
#include "process.h"
#include <Arduino.h>

// ============================================================================
// Configuration
// ============================================================================

static constexpr uint32_t DEFAULT_BIN_WIDTH_MS = 100;   // 100 ms
static constexpr size_t   BIN_COUNT            = 100;  // fixed, per spec

// ============================================================================
// RX Starvation Histogram State
// ============================================================================

static uint32_t bin_width_ms = DEFAULT_BIN_WIDTH_MS;

static uint32_t bins[BIN_COUNT];
static uint32_t last_rx_ms = 0;
static bool     rx_seen = false;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static inline void histogram_clear(void) {
  for (size_t i = 0; i < BIN_COUNT; i++) {
    bins[i] = 0;
  }
}

static inline void record_delta_ms(uint32_t delta_ms) {

  uint32_t idx = delta_ms / bin_width_ms;

  if (idx >= BIN_COUNT) {
    idx = BIN_COUNT - 1;   // saturating tail bin
  }

  bins[idx]++;
}

// ============================================================================
// RX Entry Hook (PUBLIC API)
// ============================================================================

void transport_rx_entered(void) {

  uint32_t now_ms = millis();

  if (rx_seen) {
    uint32_t delta = now_ms - last_rx_ms;
    record_delta_ms(delta);
  } else {
    // First observation establishes baseline only
    rx_seen = true;
  }

  last_rx_ms = now_ms;
}

// ============================================================================
// Commands
// ============================================================================

// ------------------------------------------------------------
// SET_BINWIDTH — configure histogram resolution
//
// Args:
//   { "ms": <uint32> }
//
// Semantics:
//   • Clears existing histogram
//   • Applies new bin width immediately
// ------------------------------------------------------------
static Payload cmd_set_binwidth(const Payload& args) {

  Payload resp;

  uint32_t ms;
  if (!args.tryGetUInt("ms", ms) || ms == 0) {
    resp.add("error", "missing or invalid ms");
    return resp;
  }

  bin_width_ms = ms;
  histogram_clear();
  rx_seen = false;

  resp.add("bin_width_ms", bin_width_ms);
  resp.add("bins", (uint32_t)BIN_COUNT);
  resp.add("status", "ok");

  return resp;
}

// ------------------------------------------------------------
// CLEAR — reset histogram state
// ------------------------------------------------------------
static Payload cmd_clear(const Payload&) {

  histogram_clear();
  rx_seen = false;
  last_rx_ms = 0;

  Payload resp;
  resp.add("status", "cleared");
  return resp;
}

// ------------------------------------------------------------
// REPORT — dump histogram snapshot
//
// Output shape:
// {
//   "bin_width_ms": <n>,
//   "bins": [
//     { "range_ms": "0-100",   "count": 123 },
//     { "range_ms": "100-200", "count": 45  },
//     ...
//   ]
// }
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;
  resp.add("bin_width_ms", bin_width_ms);

  PayloadArray arr;

  for (size_t i = 0; i < BIN_COUNT; i++) {

    uint32_t count = bins[i];
    if (count == 0) {
      continue;   // suppress empty bins
    }

    uint32_t start_ms = i * bin_width_ms;
    uint32_t end_ms   = start_ms + bin_width_ms;

    char range[32];
    snprintf(
      range,
      sizeof(range),
      "%" PRIu32 "-%" PRIu32,
      start_ms,
      end_ms
    );

    Payload e;
    e.add("range_ms", range);
    e.add("count", count);

    arr.add(e);
  }

  resp.add_array("bins", arr);
  return resp;
}


// ============================================================================
// Process Registration
// ============================================================================

static const process_command_entry_t PERFORMANCE_COMMANDS[] = {
  { "SET_BINWIDTH", cmd_set_binwidth },
  { "CLEAR",        cmd_clear        },
  { "REPORT",       cmd_report       },
  { nullptr,        nullptr          }
};

static const process_vtable_t PERFORMANCE_PROCESS = {
  .process_id    = "PERFORMANCE",
  .commands      = PERFORMANCE_COMMANDS,
  .subscriptions = nullptr
};

void process_performance_register(void) {
  process_register("PERFORMANCE", &PERFORMANCE_PROCESS);
}
