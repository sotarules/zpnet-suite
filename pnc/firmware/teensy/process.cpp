#include "process.h"
#include "transport.h"
#include "debug.h"

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
// Registry State
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
find_subscription(
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
// Registration
// ============================================================================

bool process_register(
  const char*             process_id,
  const process_vtable_t* vtable
) {
  if (!process_id || !vtable || !vtable->process_id) {
    return false;
  }

  // Enforce identity invariant
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
// REQUEST / RESPONSE Command Processor
// ============================================================================
//
// Expects a fully parsed semantic request Payload:
//
//   {
//     "subsystem": "...",
//     "command":   "...",
//     "args":      { ... },
//     "req_id":    N
//   }
//
// Always responds exactly once.
//

void process_command(const Payload& request) {

  Payload response;

  // ----------------------------------------------------------
  // Preserve req_id if present
  // ----------------------------------------------------------

  if (request.has("req_id")) {
    response.add("req_id", request.getUInt("req_id"));
  }

  // ----------------------------------------------------------
  // Extract routing fields (COPY IMMEDIATELY)
  // ----------------------------------------------------------

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

  String subsystem(subsystem_c);
  String command(command_c);

  // ----------------------------------------------------------
  // Resolve process
  // ----------------------------------------------------------

  process_entry_t* proc = find_process(subsystem.c_str());
  if (!proc) {
    response.add("success", false);
    response.add("message", "BAD");
    response.add_object("payload",
                        make_error_payload("unknown subsystem"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  // ----------------------------------------------------------
  // Resolve command
  // ----------------------------------------------------------

  const process_command_entry_t* entry =
    find_command(proc->vtable, command.c_str());

  if (!entry || !entry->handler) {
    response.add("success", false);
    response.add("message", "BAD");
    response.add_object("payload",
                        make_error_payload("unknown command"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  // ----------------------------------------------------------
  // Extract args (if present)
  // ----------------------------------------------------------

  Payload args;
  if (request.has("args")) {
    args = request.getPayload("args");
  }

  // ----------------------------------------------------------
  // Invoke handler
  // ----------------------------------------------------------

  Payload payload = entry->handler(args);

  // ----------------------------------------------------------
  // Construct canonical success envelope
  // ----------------------------------------------------------

  response.add("success", true);
  response.add("message", "OK");
  response.add_object("payload", payload);

  transport_send(
    TRAFFIC_REQUEST_RESPONSE,
    response
  );
}

// ============================================================================
// PUB / SUB Dispatch (local delivery only)
// ============================================================================
//
// Called by pub/sub ingress (transport or local publish).
// Exactly one handler per (process, topic).
//

void process_publish_dispatch(
  const char* topic,
  const Payload& payload
) {
  if (!topic || !*topic) return;

  for (size_t i = 0; i < registry_count; i++) {

    const process_vtable_t* v = registry[i].vtable;
    if (!v || !v->subscriptions) continue;

    const process_subscription_entry_t* sub =
      find_subscription(v, topic);

    if (!sub || !sub->handler) continue;

    // Single, direct delivery
    sub->handler(payload);
  }
}
