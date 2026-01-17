// ============================================================================
// FILE: event_bus.cpp
// ============================================================================
#include "event_bus.h"
#include "transport.h"
#include "util.h"
#include "timepop.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal ring buffer state
// --------------------------------------------------------------
static EventItem evtq[EVT_MAX];
static size_t evt_head = 0;
static size_t evt_tail = 0;
static size_t evt_count = 0;
static uint32_t evt_dropped = 0;

// Uncomment for imperative diagnostics
// #define EVENTBUS_IMMEDIATE_DIAGNOSTICS

// --------------------------------------------------------------
// TimePop-driven drain scheduling (new)
// --------------------------------------------------------------
//
// Design constraints:
//   • event_bus owns its own cadence (self-rescheduling TimePop callback)
//   • no work in ISR
//   • no polling in zpnet_loop()
//   • NO unsolicited serial output:
//       - the tick emits nothing unless a drain has been explicitly requested
//
// Behavioral contract:
//   • command plane requests a drain via event_bus_request_drain()
//   • event_bus tick performs drain in scheduled context
//
static constexpr uint32_t EVENTBUS_TICK_INTERVAL_MS = 5;

static volatile bool g_drain_requested  = false;
static volatile bool g_drain_in_progress = false;

static void eventbus_tick(void*);

// --------------------------------------------------------------
// Internal helpers
// --------------------------------------------------------------
//
// Emit a protocol-control message (NOT an event).
// If body == nullptr, only the control field is emitted.
//
static inline void emitControlMessage(
    const char* control,
    const String* body
) {
  String out;

  out += "{\"control\":\"";
  out += control;
  out += "\"";

  if (body && body->length() > 0) {
    out += ",";
    out += *body;
  }

  out += "}";

  transport_send_frame(out.c_str(), out.length());
}

// Overload: control-only message (no body)
static inline void emitControlMessage(const char* control) {
  emitControlMessage(control, nullptr);
}

//
// Emit a factual event message (durable truth).
//
static inline void emitEventMessage(
    const char* type,
    const String& body
) {
  String out;

  out += "{\"event_type\":\"";
  out += type;
  out += "\"";

  if (body.length() > 0) {
    out += ",";
    out += body;
  }

  out += "}";

  transport_send_frame(out.c_str(), out.length());
}

static bool dequeueEvent(EventItem& out) {
  if (!evt_count) return false;

  out = evtq[evt_tail];
  evt_tail = (evt_tail + 1) % EVT_MAX;
  evt_count--;
  return true;
}

// --------------------------------------------------------------
// Public API (new)
// --------------------------------------------------------------
void event_bus_init() {
  // Reset scheduler state
  g_drain_requested = false;
  g_drain_in_progress = false;

  // Arm first tick (self-scheduling thereafter)
  timepop_schedule(
      EVENTBUS_TICK_INTERVAL_MS,
      TIMEPOP_UNITS_MILLISECONDS,
      eventbus_tick,
      nullptr,
      "event-bus"
  );
}

void event_bus_request_drain() {
  // Idempotent: multiple requests collapse to one drain opportunity.
  g_drain_requested = true;
}

// --------------------------------------------------------------
// Public API (existing)
// --------------------------------------------------------------
void enqueueEvent(const char* type, const String& body) {

#ifdef EVENTBUS_IMMEDIATE_DIAGNOSTICS
  // ------------------------------------------------------------
  // Diagnostic mode:
  // Emit immediately, bypassing queue and durability semantics.
  // ------------------------------------------------------------
  emitEventMessage(type, body);
  return;
#endif

  // ------------------------------------------------------------
  // Normal durable mode (unchanged)
  // ------------------------------------------------------------
  if (evt_count >= EVT_MAX) {
    evt_dropped++;
    return;
  }

  EventItem& e = evtq[evt_head];
  safeCopy(e.type, sizeof(e.type), type);
  safeCopy(e.body, sizeof(e.body), body.c_str());

  evt_head = (evt_head + 1) % EVT_MAX;
  evt_count++;
}

void drainEventsNow() {
  // ------------------------------------------------------------
  // Protocol control: EVENTS_BEGIN
  // ------------------------------------------------------------
  {
    String meta;
    meta += "\"count\":";
    meta += evt_count;
    meta += ",\"dropped\":";
    meta += evt_dropped;

    emitControlMessage("EVENTS_BEGIN", &meta);
  }

  // ------------------------------------------------------------
  // Drain queued events (pure facts)
  // ------------------------------------------------------------
  EventItem e;
  while (dequeueEvent(e)) {
    String body;

    // Body is already a valid JSON fragment (no braces)
    if (e.body[0]) {
      body += e.body;
    }

    emitEventMessage(e.type, body);
  }

  // ------------------------------------------------------------
  // Protocol control: EVENTS_END
  // ------------------------------------------------------------
  emitControlMessage("EVENTS_END");
}

void enqueueAckEvent(const char* cmd) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\"";
  b += ",\"ok\":true";

  enqueueEvent("ACK", b);
}

void enqueueErrEvent(const char* cmd, const char* msg) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\"";
  b += ",\"ok\":false";

  if (msg) {
    b += ",\"error\":\"";
    b += jsonEscape(msg);
    b += "\"";
  }

  enqueueEvent("ERR", b);
}

uint32_t eventQueueDepth() {
  return (uint32_t)evt_count;
}

uint32_t eventDroppedCount() {
  return evt_dropped;
}

// --------------------------------------------------------------
// TimePop tick (scheduled context)
// --------------------------------------------------------------
static void eventbus_tick(void*) {
  // ------------------------------------------------------------
  // No unsolicited output:
  // Only drain if an explicit request has been made.
  // ------------------------------------------------------------
  if (g_drain_requested && !g_drain_in_progress) {

    // Claim the drain (collapse multiple requests)
    g_drain_requested = false;
    g_drain_in_progress = true;

    // Perform full drain in scheduled context (NOT ISR)
    drainEventsNow();

    g_drain_in_progress = false;
  }

  // Self-reschedule (module owns its cadence)
  timepop_schedule(
      EVENTBUS_TICK_INTERVAL_MS,
      TIMEPOP_UNITS_MILLISECONDS,
      eventbus_tick,
      nullptr,
      "event-bus"
  );
}
