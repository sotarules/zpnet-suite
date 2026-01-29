// ============================================================================
// FILE: process_events.cpp
// ============================================================================
//
// ZPNet EVENTS Process
//
// Role:
//   • Maintain a bounded FIFO of durable events
//   • Expose queued events via EVENTS.GET
//
// Semantics:
//   • enqueueEvent() records immutable facts (type + Payload)
//   • EVENTS.GET returns queued events and clears the queue
//   • Event draining requires explicit scheduling via TimePop
//   • No streaming, framing, or hidden lifecycle
//
// ============================================================================

#include "config.h"
#include "debug.h"
#include "process_events.h"
#include "process.h"
#include "payload.h"
#include "transport.h"
#include "util.h"
#include "timepop.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Event storage
// --------------------------------------------------------------
//
// NOTE:
//   • Payload is serialized at enqueue time
//   • EVENTS stores only complete, valid JSON objects
//   • No fragments, no syntax splicing
//

struct EventItem {
  char type[EVT_TYPE_MAX];
  char payload[EVT_BODY_MAX];  // full JSON object or empty string
};

static EventItem evtq[EVT_MAX];
static size_t evt_head  = 0;
static size_t evt_tail  = 0;
static size_t evt_count = 0;

// --------------------------------------------------------------
// Explicit initialization
// --------------------------------------------------------------
//
// EVENTS depends on TimePop to drain expired events.
// This function must be called exactly once after timepop_init().
//

void process_events_init(void) {
  // No operation
}

// --------------------------------------------------------------
// Public API — enqueue durable event
// --------------------------------------------------------------
//
// Contract:
//   • Payload is a complete JSON object
//   • Empty payload means "no payload"
//   • Serialization happens here, once
//

void enqueueEvent(const char* type, const Payload& payload) {

  if (evt_count >= EVT_MAX) {
    // Drop silently: overflow is an observable condition downstream
    return;
  }

  EventItem& e = evtq[evt_head];
  safeCopy(e.type, sizeof(e.type), type);

  if (!payload.empty()) {
    String json = payload.to_json();
    safeCopy(e.payload, sizeof(e.payload), json.c_str());
  } else {
    e.payload[0] = '\0';
  }

  evt_head = (evt_head + 1) % EVT_MAX;
  evt_count++;
}

void emit_system_error(
  const char* subsystem,
  const char* file,
  const char* function,
  const char* condition
) {
  Payload ev;

  if (subsystem) ev.add("subsystem", subsystem);
  if (file)      ev.add("file", file);
  if (function)  ev.add("function", function);
  if (condition) ev.add("condition", condition);

  enqueueEvent("SYSTEM_ERROR", ev);
}


// --------------------------------------------------------------
// Command: GET — return and clear all queued events
// --------------------------------------------------------------
//
// Returns Payload containing:
//   {
//     "events": [
//       { "event_type": "...", "payload": { ... } },
//       ...
//     ]
//   }
//
// Args are ignored (EVENTS.GET takes no arguments).
//

static const Payload* cmd_get(const Payload& /*args*/) {

  static Payload out;
  out.clear();

  PayloadArray events;

  while (evt_count > 0) {

    const EventItem& e = evtq[evt_tail];

    Payload item;
    item.add("event_type", e.type);

    if (e.payload[0] != '\0') {
      // Adopt trusted JSON object as a real sub-node
      item.add_raw_object("payload", e.payload);
    }

    events.add(item);

    evt_tail = (evt_tail + 1) % EVT_MAX;
    evt_count--;
  }

  out.add_array("events", events);

  return &out;
}

// --------------------------------------------------------------
// Registration
// --------------------------------------------------------------

static const process_command_entry_t EVENTS_COMMANDS[] = {
  { "GET", cmd_get },
};

static const process_vtable_t EVENTS_PROCESS = {
  .name = "EVENTS",
  .query = nullptr,
  .commands = EVENTS_COMMANDS,
  .command_count = 1,
};

void process_events_register(void) {
  process_register("EVENTS", &EVENTS_PROCESS);
}
