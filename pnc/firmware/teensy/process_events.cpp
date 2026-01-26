// ============================================================================
// FILE: process_events.cpp (Self-contained Version)
// ============================================================================
//
// ZPNet EVENTS Process — Durable Event Drain via Structured Command
//
// Responsibilities:
//   • Supersedes event_bus.cpp by inlining all functionality
//   • Provides command interface for GET
//   • Implements internal event queue, enqueue, and drain logic
//   • Conforms to process registration contract
//
// Author: The Mule + GPT
// ============================================================================

#include "process_events.h"
#include "process.h"
#include "timepop.h"
#include "debug.h"
#include "transport.h"
#include "util.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal config + storage
// --------------------------------------------------------------
#define EVT_MAX 64
#define EVT_TYPE_MAX 32
#define EVT_BODY_MAX 192

struct EventItem {
  char type[EVT_TYPE_MAX];
  char body[EVT_BODY_MAX];
};

static EventItem evtq[EVT_MAX];
static size_t evt_head = 0;
static size_t evt_tail = 0;
static size_t evt_count = 0;
static uint32_t evt_dropped = 0;

static volatile bool g_drain_requested   = false;
static volatile bool g_drain_in_progress = false;

static uint32_t last_tick_ms = 0;

static void maybe_debug_tick(const char* tag) {
  uint32_t now = millis();
  if (now - last_tick_ms >= 1000) {
    last_tick_ms = now;
  }
}

// --------------------------------------------------------------
// Protocol emitters
// --------------------------------------------------------------
static inline void emitControlMessage(const char* control, const String* body = nullptr) {
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

static inline void emitEventMessage(const char* type, const String& body) {
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

// --------------------------------------------------------------
// Queue helpers
// --------------------------------------------------------------
static bool dequeueEvent(EventItem& out) {
  if (!evt_count) return false;
  out = evtq[evt_tail];
  evt_tail = (evt_tail + 1) % EVT_MAX;
  evt_count--;
  return true;
}

// --------------------------------------------------------------
// Public API for enqueue
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

// --------------------------------------------------------------
// Drain logic (immediate or tick)
// --------------------------------------------------------------
void drainEventsNow(void) {
  String meta;
  meta += "\"count\":";
  meta += evt_count;
  meta += ",\"dropped\":";
  meta += evt_dropped;
  emitControlMessage("EVENTS_BEGIN", &meta);

  EventItem e;
  while (dequeueEvent(e)) {
    String body;
    if (e.body[0]) body += e.body;
    emitEventMessage(e.type, body);
  }

  emitControlMessage("EVENTS_END");
}

// --------------------------------------------------------------
// TimePop tick
// --------------------------------------------------------------
static void eventbus_tick(timepop_ctx_t* /*timer*/, void* /*user*/) {
  maybe_debug_tick("eventbus");
  if (g_drain_requested && !g_drain_in_progress) {
    g_drain_requested = false;
    g_drain_in_progress = true;
    drainEventsNow();
    g_drain_in_progress = false;
  }
}

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void event_bus_init(void) {
  g_drain_requested = false;
  g_drain_in_progress = false;
  timepop_arm(TIMEPOP_CLASS_EVENTBUS, true, eventbus_tick, nullptr, "event-bus");
}

void event_bus_request_drain(void) {
  g_drain_requested = true;
}

void enqueueAckEvent(const char* cmd) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\",\"ok\":true";
  enqueueEvent("ACK", b);
}

void enqueueErrEvent(const char* cmd, const char* msg) {
  String b;
  b += "\"cmd\":\"";
  b += cmd;
  b += "\",\"ok\":false";
  if (msg) {
    b += ",\"error\":\"";
    b += jsonEscape(msg);
    b += "\"";
  }
  enqueueEvent("ERR", b);
}

uint32_t eventQueueDepth(void) {
  return (uint32_t)evt_count;
}

uint32_t eventDroppedCount(void) {
  return evt_dropped;
}

// ------------------------------------------------------------------
// Lifecycle (process)
// ------------------------------------------------------------------
static bool events_start(void) {
  event_bus_init();
  return true;
}

static void events_stop(void) {}

// ------------------------------------------------------------------
// Commands
// ------------------------------------------------------------------
static const String* cmd_get(const char* /*args_json*/) {
  static String payload;
  payload = "{}";
  drainEventsNow();
  return &payload;
}

// ------------------------------------------------------------------
// Registration
// ------------------------------------------------------------------
static const process_command_entry_t EVENTS_COMMANDS[] = {
  { "GET", cmd_get },
};

static const process_vtable_t EVENTS_PROCESS = {
  .name = "EVENTS",
  .start = events_start,
  .stop  = events_stop,
  .query = nullptr,
  .commands = EVENTS_COMMANDS,
  .command_count = 1,
};

void process_events_register(void) {
  process_register(PROCESS_TYPE_EVENTS, &EVENTS_PROCESS);
}
