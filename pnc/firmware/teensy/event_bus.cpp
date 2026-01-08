#include "event_bus.h"
#include "util.h"

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
static inline void emitJson(const char* type, const String& body) {
  Serial.print("{\"event_type\":\"");
  Serial.print(type);
  Serial.print("\"");
  if (body.length()) {
    Serial.print(",");
    Serial.print(body);
  }
  Serial.println("}");
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
  e.ms = millis();
  safeCopy(e.type, sizeof(e.type), type);
  safeCopy(e.body, sizeof(e.body), body.c_str());

  evt_head = (evt_head + 1) % EVT_MAX;
  evt_count++;
}

void drainEventsNow() {
  // Frame: BEGIN
  {
    String b;
    b += "\"count\":";
    b += evt_count;
    b += ",\"dropped\":";
    b += evt_dropped;
    emitJson("EVENTS_BEGIN", b);
  }

  // Drain queue
  EventItem e;
  while (dequeueEvent(e)) {
    String b;
    b += "\"millis\":";
    b += e.ms;

    if (e.body[0]) {
      b += ",";
      b += e.body;
    }

    emitJson(e.type, b);
  }

  // Frame: END
  emitJson("EVENTS_END", "\"count\":0");
}

void enqueueAckEvent(const char* cmd) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\"";
  b += ",\"ok\":true";
  b += ",\"millis\":";
  b += millis();

  enqueueEvent("ACK", b);
}

void enqueueErrEvent(const char* cmd, const char* msg) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\"";
  b += ",\"ok\":false";
  b += ",\"millis\":";
  b += millis();

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
