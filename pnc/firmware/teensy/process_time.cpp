// ================================================================
// FILE: process_time.cpp
// ================================================================
//
// ZPNet TIME Subsystem — Greenfield Clock Playground
//
// First live experiment:
//   • Instantiate a single DwtClock on START
//   • Expose its synthetic nanoseconds via REPORT
//
// ================================================================

#include "process_time.h"

#include "payload.h"
#include "process.h"

#include "dwt_clock.h"

#include <Arduino.h>
#include <string.h>

// ---------------------------------------------------------------
// Experimental state
// ---------------------------------------------------------------

static bool time_running = false;
static char current_campaign[64] = {0};

// Single experimental clock instance (for now)
static DwtClock* dwt_clock = nullptr;

// Teensy 4.1: ~600 MHz CPU clock
static constexpr uint32_t DWT_CYCLES_PER_SECOND = 600000000u;

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------

static Payload build_time_report_payload(void) {
  Payload p;

  p.add("running", time_running);
  p.add("campaign", current_campaign[0] ? current_campaign : "(none)");

  if (dwt_clock) {
    p.add("dwt_ns", dwt_clock->nanoseconds_now());
  } else {
    p.add("dwt_ns", 0);
  }

  return p;
}

// ---------------------------------------------------------------
// Commands
// ---------------------------------------------------------------

// REPORT — pull-based introspection
static Payload cmd_report(const Payload&) {
  return build_time_report_payload();
}

// START — instantiate DwtClock
static Payload cmd_start(const Payload& req) {

  const char* campaign = req.getString("campaign");
  if (campaign) {
    strncpy(current_campaign, campaign, sizeof(current_campaign) - 1);
    current_campaign[sizeof(current_campaign) - 1] = '\0';
  } else {
    current_campaign[0] = '\0';
  }

  // Instantiate clock if not already running
  if (!dwt_clock) {
    dwt_clock = new DwtClock(DWT_CYCLES_PER_SECOND);
  }

  time_running = true;

  Payload p;
  p.add("status", "ok");
  p.add("action", "start");
  p.add("campaign", current_campaign[0] ? current_campaign : "(none)");

  return p;
}

// STOP — freeze time (no destruction yet)
static Payload cmd_stop(const Payload&) {

  time_running = false;

  Payload p;
  p.add("status", "ok");
  p.add("action", "stop");

  return p;
}

// CLEAR — reset synthetic nanoseconds (future policy)
static Payload cmd_clear(const Payload&) {

  // NOTE:
  //   Clearing behavior will later call AbstractClock::request_clear()
  //   For now, we simply discard the clock.

  if (dwt_clock) {
    delete dwt_clock;
    dwt_clock = nullptr;
  }

  time_running = false;
  current_campaign[0] = '\0';

  Payload p;
  p.add("status", "ok");
  p.add("action", "clear");

  return p;
}

// RECOVER — stub (future PPS-aligned rebinding)
static Payload cmd_recover(const Payload& req) {

  const char* campaign = req.getString("campaign");
  if (campaign) {
    strncpy(current_campaign, campaign, sizeof(current_campaign) - 1);
    current_campaign[sizeof(current_campaign) - 1] = '\0';
  }

  // Recovery semantics will be added later.
  // For now, treat as START without clearing.

  if (!dwt_clock) {
    dwt_clock = new DwtClock(DWT_CYCLES_PER_SECOND);
  }

  time_running = true;

  Payload p;
  p.add("status", "ok");
  p.add("action", "recover");
  p.add("campaign", current_campaign[0] ? current_campaign : "(none)");

  return p;
}

// ---------------------------------------------------------------
// Command table
// ---------------------------------------------------------------

static const process_command_entry_t TIME_COMMANDS[] = {
  { "REPORT",  cmd_report  },
  { "START",   cmd_start   },
  { "STOP",    cmd_stop    },
  { "CLEAR",   cmd_clear   },
  { "RECOVER", cmd_recover },
  { nullptr,   nullptr     }
};

// ---------------------------------------------------------------
// Process vtable
// ---------------------------------------------------------------

static const process_vtable_t TIME_PROCESS = {
  .process_id    = "TIME",
  .commands      = TIME_COMMANDS,
  .subscriptions = nullptr,
};

// ---------------------------------------------------------------
// Registration
// ---------------------------------------------------------------

void process_time_register(void) {
  process_register("TIME", &TIME_PROCESS);
}

// ---------------------------------------------------------------
// Explicit initialization
// ---------------------------------------------------------------

void process_time_init(void) {
  // Intentionally empty.
  // TIME is inert until START is issued.
}
