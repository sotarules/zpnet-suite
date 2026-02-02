// ============================================================================
// process.cpp — ZPNet Process Framework
// ============================================================================
//
// Subscriptions:
//   • NO persistence
//   • NO local storage
//   • Subscription truth is owned by PUBSUB (volatile, RAM-only)
//
// Handlers:
//   • Handler functions live in process vtables (code truth)
//
// ============================================================================

#include "process.h"
#include "transport.h"
#include "debug.h"
#include "process_pubsub.h"   // <-- subscription access

#include <Arduino.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

static constexpr size_t MAX_PROCESSES = 30;

// ============================================================================
// Registry Entry
// ============================================================================

typedef struct {
  const char*             id;
  const process_vtable_t* vtable;
} process_entry_t;

// ============================================================================
// Registry State (commands only)
// ============================================================================

static process_entry_t registry[MAX_PROCESSES];
static size_t registry_count = 0;

// ============================================================================
// Helpers
// ============================================================================

static process_entry_t* find_process(const char* process_id) {
  if (!process_id) return nullptr;

  for (size_t i = 0; i < registry_count; i++) {
    if (strcmp(registry[i].id, process_id) == 0) {
      return &registry[i];
    }
  }
  return nullptr;
}

static const process_command_entry_t*
find_command(const process_vtable_t* vtable, const char* name) {
  if (!vtable || !vtable->commands || !name) return nullptr;

  for (const process_command_entry_t* e = vtable->commands;
       e->name;
       ++e) {
    if (strcmp(e->name, name) == 0) {
      return e;
    }
  }

  return nullptr;
}

static const process_subscription_entry_t*
find_subscription_handler(
  const process_vtable_t* vtable,
  const char*             topic
) {
  if (!vtable || !vtable->subscriptions || !topic) return nullptr;

  for (const process_subscription_entry_t* e = vtable->subscriptions;
       e->topic;
       ++e) {
    if (strcmp(e->topic, topic) == 0) {
      return e;
    }
  }

  return nullptr;
}

// ============================================================================
// Canonical helpers
// ============================================================================

Payload ok_payload() {
  Payload p;
  p.add("status", "ok");
  return p;
}

static Payload make_error_payload(const char* msg) {
  Payload p;
  p.add("error", msg ? msg : "unknown error");
  return p;
}

// ============================================================================
// Lifecycle
// ============================================================================

void process_init(void) {
  registry_count = 0;
}

// ============================================================================
// Registration (commands + handler capability only)
// ============================================================================

bool process_register(
  const char*             process_id,
  const process_vtable_t* vtable
) {
  if (!process_id || !vtable || !vtable->process_id) {
    return false;
  }

  if (strcmp(process_id, vtable->process_id) != 0) {
    return false;
  }

  if (registry_count >= MAX_PROCESSES) {
    return false;
  }

  if (find_process(process_id)) {
    return false;
  }

  registry[registry_count].id     = process_id;
  registry[registry_count].vtable = vtable;
  registry_count++;

  return true;
}

// ============================================================================
// Registry Introspection (READ-ONLY)
// ============================================================================

size_t process_get_count(void) {
  return registry_count;
}

const char* process_get_name(size_t idx) {
  if (idx >= registry_count) return nullptr;
  return registry[idx].id;
}

const process_vtable_t* process_get_vtable(size_t idx) {
  if (idx >= registry_count) return nullptr;
  return registry[idx].vtable;
}

// ============================================================================
// REQUEST / RESPONSE Command Processor (UNCHANGED)
// ============================================================================

void process_command(const Payload& request) {

  Payload response;

  if (request.has("req_id")) {
    response.add("req_id", request.getUInt("req_id"));
  }

  const char* subsystem_c = request.getString("subsystem");
  const char* command_c   = request.getString("command");

  if (!subsystem_c || !command_c) {
    response.add("success", false);
    response.add("message", "BAD");
    response.add_object("payload",
                        make_error_payload("missing routing fields"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  process_entry_t* proc = find_process(subsystem_c);
  if (!proc) {
    response.add("success", false);
    response.add("message", "BAD");
    response.add_object("payload",
                        make_error_payload("unknown subsystem"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  const process_command_entry_t* entry =
    find_command(proc->vtable, command_c);

  if (!entry || !entry->handler) {
    response.add("success", false);
    response.add("message", "BAD");
    response.add_object("payload",
                        make_error_payload("unknown command"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  Payload args;
  if (request.has("args")) {
    args = request.getPayload("args");
  }

  Payload payload = entry->handler(args);

  response.add("success", true);
  response.add("message", "OK");
  response.add_object("payload", payload);

  transport_send(
    TRAFFIC_REQUEST_RESPONSE,
    response
  );
}

// ============================================================================
// PUB / SUB Dispatch (volatile, PUBSUB-owned truth)
// ============================================================================

void process_publish_dispatch(
  const char* topic,
  const Payload& payload
) {
  if (!topic || !*topic) return;

  // ------------------------------------------------------------
  // Obtain current volatile subscription truth
  // ------------------------------------------------------------

  const Payload* subs = pubsub_get_subscriptions();
  if (!subs || subs->empty()) {
    return;   // no active subscriptions in this runtime
  }

  // ------------------------------------------------------------
  // Deliver to matching process handlers
  // ------------------------------------------------------------

  for (size_t i = 0; i < registry_count; i++) {

    const process_vtable_t* v = registry[i].vtable;
    if (!v) continue;

    if (!subs->hasArray(v->process_id)) continue;

    PayloadArrayView topics = subs->getArrayView(v->process_id);

    for (size_t j = 0; j < topics.size(); j++) {

      Payload entry = topics.get(j);

      const char* subscribed_topic = entry.getString("topic");
      if (!subscribed_topic) continue;
      if (strcmp(subscribed_topic, topic) != 0) continue;

      const process_subscription_entry_t* handler =
        find_subscription_handler(v, topic);

      if (!handler || !handler->handler) continue;

      handler->handler(payload);
    }
  }
}