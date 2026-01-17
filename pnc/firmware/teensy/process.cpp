#include "process.h"
#include "event_bus.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const size_t MAX_PROCESSES = 8;

// -----------------------------------------------------------------------------
// Internal registry entry
// -----------------------------------------------------------------------------

typedef struct {
  process_type_t            type;
  process_state_t           state;
  const process_vtable_t*   vtable;
} process_entry_t;

// -----------------------------------------------------------------------------
// Internal registry state
// -----------------------------------------------------------------------------

static process_entry_t registry[MAX_PROCESSES];
static size_t registry_count = 0;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static process_entry_t* find_process(process_type_t type) {
  for (size_t i = 0; i < registry_count; ++i) {
    if (registry[i].type == type) {
      return &registry[i];
    }
  }
  return nullptr;
}

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

void process_init(void) {
  registry_count = 0;
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

bool process_register(
  process_type_t type,
  const process_vtable_t* vtable
) {
  if (!vtable || !vtable->start || !vtable->stop || !vtable->query) {
    return false;
  }

  if (registry_count >= MAX_PROCESSES) {
    return false;
  }

  if (find_process(type)) {
    return false; // duplicate registration
  }

  registry[registry_count].type   = type;
  registry[registry_count].state  = PROCESS_STATE_STOPPED;
  registry[registry_count].vtable = vtable;
  registry_count++;

  return true;
}

// -----------------------------------------------------------------------------
// Start / Stop
// -----------------------------------------------------------------------------

bool process_start(process_type_t type) {
  process_entry_t* p = find_process(type);
  if (!p) return false;

  if (p->state == PROCESS_STATE_RUNNING) {
    return true; // idempotent
  }

  bool ok = p->vtable->start();
  if (ok) {
    p->state = PROCESS_STATE_RUNNING;

    String body;
    body += "\"name\":\"";
    body += p->vtable->name;
    body += "\"";

    enqueueEvent("PROCESS_STARTED", body);
  } else {
    p->state = PROCESS_STATE_ERROR;

    String body;
    body += "\"name\":\"";
    body += p->vtable->name;
    body += "\"";

    enqueueEvent("PROCESS_ERROR", body);
  }

  return ok;
}

bool process_stop(process_type_t type) {
  process_entry_t* p = find_process(type);
  if (!p) return false;

  if (p->state != PROCESS_STATE_RUNNING) {
    return true; // idempotent
  }

  p->vtable->stop();
  p->state = PROCESS_STATE_STOPPED;

  String body;
  body += "\"name\":\"";
  body += p->vtable->name;
  body += "\"";

  enqueueEvent("PROCESS_STOPPED", body);
  return true;
}

// -----------------------------------------------------------------------------
// Query
// -----------------------------------------------------------------------------

bool process_query(
  process_type_t type,
  String& out_body
) {
  process_entry_t* p = find_process(type);
  if (!p) return false;

  out_body = "\"name\":\"";
  out_body += p->vtable->name;
  out_body += "\",\"state\":\"";

  switch (p->state) {
    case PROCESS_STATE_RUNNING:
      out_body += "RUNNING";
      break;
    case PROCESS_STATE_STOPPED:
      out_body += "STOPPED";
      break;
    case PROCESS_STATE_ERROR:
      out_body += "ERROR";
      break;
  }

  out_body += "\"";

  String inner = p->vtable->query();
  if (inner.length()) {
    out_body += ",";
    out_body += inner;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Command
// -----------------------------------------------------------------------------

bool process_command(
  process_type_t type,
  const char* cmd,
  const char* args,
  String& out_body
) {
  process_entry_t* p = find_process(type);
  if (!p || !p->vtable->command) {
    return false;
  }
  out_body = p->vtable->command(cmd, args);
  return true;
}

// -----------------------------------------------------------------------------
// Registry introspection
// -----------------------------------------------------------------------------

String process_list_json(void) {
  String b;
  b += "\"processes\":[";

  for (size_t i = 0; i < registry_count; ++i) {
    if (i) b += ",";

    b += "{";
    b += "\"name\":\"";
    b += registry[i].vtable->name;
    b += "\",\"state\":\"";

    switch (registry[i].state) {
      case PROCESS_STATE_RUNNING:
        b += "RUNNING";
        break;
      case PROCESS_STATE_STOPPED:
        b += "STOPPED";
        break;
      case PROCESS_STATE_ERROR:
        b += "ERROR";
        break;
    }

    b += "\"}";
  }

  b += "]";
  return b;
}
