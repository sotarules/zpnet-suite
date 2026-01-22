// ============================================================================
// FILE: event_bus.cpp
// ============================================================================
//
// ZPNet Event Bus (Durable Truth Queue)
//
// Core contract:
//   • Enqueue facts freely.
//   • Emit nothing unless explicitly asked.
//   • Drain ONLY in scheduled (non-ISR) context.
//   • No polling in loop().
//   • TimePop owns cadence; event_bus does NOT self-schedule.
//
// ============================================================================

#include "event_bus.h"

#include "debug.h"
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

// Uncomment for imperative diagnostics (bypasses durability semantics)
// #define EVENTBUS_IMMEDIATE_DIAGNOSTICS

// --------------------------------------------------------------
// Drain request state (cross-context flags)
// --------------------------------------------------------------
static volatile bool g_drain_requested   = false;
static volatile bool g_drain_in_progress = false;

// --------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------
static void eventbus_tick(timepop_ctx_t* timer, void* user);

// --------------------------------------------------------------
// Optional debug tick (rate-limited to 1 Hz)
// --------------------------------------------------------------
static uint32_t last_tick_ms = 0;

static void maybe_debug_tick(const char* tag) {
  uint32_t now = millis();
  if (now - last_tick_ms >= 1000) {
    last_tick_ms = now;
  }
}

// --------------------------------------------------------------
// Internal helpers — protocol/control vs. fact emission
// --------------------------------------------------------------

// Emit a protocol-control message (NOT a durable event).
// If body == nullptr, only {"control":"X"} is emitted.
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
    out += *body;   // already a JSON fragment
  }

  out += "}";

  transport_send_frame(out.c_str(), out.length());
}

// Overload: control-only message (no body)
static inline void emitControlMessage(const char* control) {
  emitControlMessage(control, nullptr);
}

// Emit a factual event message (durable truth).
// Body must already be a valid JSON fragment WITHOUT braces.
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
// Public API (TimePop-owned lifecycle)
// --------------------------------------------------------------
void event_bus_init(void) {
  g_drain_requested   = false;
  g_drain_in_progress = false;

  // Arm recurring event bus tick.
  // TimePop owns cadence; this module does NOT self-reschedule.
  timepop_arm(
    TIMEPOP_CLASS_EVENTBUS,
    true,                 // recurring
    eventbus_tick,
    nullptr,
    "event-bus"
  );
}

void event_bus_request_drain(void) {
  // Idempotent: multiple requests collapse to one opportunity.
  g_drain_requested = true;
}

// --------------------------------------------------------------
// Public API (durable truth queue)
// --------------------------------------------------------------
void enqueueEvent(const char* type, const String& body) {

#ifdef EVENTBUS_IMMEDIATE_DIAGNOSTICS
  // Diagnostic mode: emit immediately (NOT durable).
  emitEventMessage(type, body);
  return;
#endif

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

// --------------------------------------------------------------
// Drain (protocol framing + factual events)
// --------------------------------------------------------------
void drainEventsNow(void) {

  // Protocol control: EVENTS_BEGIN
  {
    String meta;
    meta += "\"count\":";
    meta += evt_count;
    meta += ",\"dropped\":";
    meta += evt_dropped;

    emitControlMessage("EVENTS_BEGIN", &meta);
  }

  // Drain queued events (pure facts)
  EventItem e;
  while (dequeueEvent(e)) {
    String body;
    if (e.body[0]) {
      body += e.body;  // already JSON fragment
    }
    emitEventMessage(e.type, body);
  }

  // Protocol control: EVENTS_END
  emitControlMessage("EVENTS_END");
}

// --------------------------------------------------------------
// Convenience helpers (queue-only)
// --------------------------------------------------------------
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

// --------------------------------------------------------------
// Diagnostics
// --------------------------------------------------------------
uint32_t eventQueueDepth(void) {
  return (uint32_t)evt_count;
}

uint32_t eventDroppedCount(void) {
  return evt_dropped;
}

// --------------------------------------------------------------
// TimePop tick (scheduled context)
// --------------------------------------------------------------
static void eventbus_tick(timepop_ctx_t* /*timer*/, void* /*user*/) {

  // Optional visibility (rate-limited)
  maybe_debug_tick("eventbus");

  // No unsolicited output:
  // only drain when explicitly requested.
  if (g_drain_requested && !g_drain_in_progress) {

    // Claim the drain (collapse concurrent requests)
    g_drain_requested   = false;
    g_drain_in_progress = true;

    // Drain in scheduled context (NOT ISR)
    drainEventsNow();

    g_drain_in_progress = false;
  }
}
