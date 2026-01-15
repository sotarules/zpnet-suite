#include "event/event_bus.h"

#include "transport/transport.h"
#include "util/util.h"

// --------------------------------------------------------------
// Internal ring buffer state
// --------------------------------------------------------------
static EventItem evtq[EVT_MAX];
static size_t evt_head = 0;
static size_t evt_tail = 0;
static size_t evt_count = 0;
static uint32_t evt_dropped = 0;

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
// Public API
// --------------------------------------------------------------
void enqueueEvent(const char* type, const String& body) {
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
  return evt_count;
}

uint32_t eventDroppedCount() {
  return evt_dropped;
}
