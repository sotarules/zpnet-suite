// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Volatile Subscription Control Plane (Teensy)
//
// Responsibilities:
//   • Maintain volatile subscription truth (RAM-only)
//   • Apply authoritative subscription sets from Pi
//   • Derive and expose Cartesian routing edges
//   • Report routing truth in execution form
//   • Expose declared subscription intent (ALLSUBSCRIPTIONS)
//
// Semantics:
//   • Pi is authoritative
//   • Teensy never infers or reshapes intent
//   • Subscription state is clobber-and-go
//   • No persistence
//
// =============================================================

#include "process_pubsub.h"

#include "process.h"
#include "publish.h"
#include "payload.h"
#include "debug.h"

#include <string.h>

// ------------------------------------------------------------
// Local helper: find subscription handler in a vtable
// ------------------------------------------------------------
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

// ================================================================
// Authoritative subscription payload (verbatim from Pi)
//
// Shape:
//   {
//     "subscriptions": [ ... ]
//   }
// ================================================================

static Payload g_union_payload;

// ================================================================
// Derived Cartesian routing edges (execution truth)
//
// Each row represents ONE atomic routing edge:
//   (machine, subsystem, topic)
//
// Stored as an array of objects:
//   { "machine": "...", "subsystem": "...", "topic": "..." }
// ================================================================

static Payload g_routes;  // { "routes": [ {m,s,t}, ... ] }

// ================================================================
// Helpers
// ================================================================

static void clear_state() {
  g_union_payload.clear();
  g_routes.clear();
}

// ================================================================
// Accessors
// ================================================================

const Payload* pubsub_get_subscriptions() {
  return &g_routes;
}

// ================================================================
// Ingress: publications arriving from Pi (DATA PLANE ONLY)
// ================================================================

void process_publish_dispatch(const Payload& message) {

  if (!message.has("topic")) return;

  const char* topic = message.getString("topic");
  if (!topic || !*topic) return;

  Payload payload;
  if (message.has("payload")) {
    payload = message.getPayload("payload");
  }

  // DATA PLANE ONLY
  process_publish_dispatch(topic, payload);
}

// ================================================================
// Fan-out (execution truth, Cartesian routing)
// ================================================================

void process_pubsub_fanout(
  const char* topic,
  const Payload& payload
) {
  if (!topic || !*topic) return;
  if (!g_routes.hasArray("routes")) return;

  PayloadArrayView routes = g_routes.getArrayView("routes");

  for (size_t i = 0; i < routes.size(); i++) {

    Payload edge = routes.get(i);

    const char* machine   = edge.getString("machine");
    const char* subsystem = edge.getString("subsystem");
    const char* t         = edge.getString("topic");

    if (!machine || !subsystem || !t) continue;
    if (strcmp(t, topic) != 0) continue;

    // ------------------------------------------------------------
    // Only deliver locally for TEENSY targets
    // ------------------------------------------------------------
    if (strcmp(machine, "TEENSY") != 0) continue;

    const process_vtable_t* vt = process_get_vtable_by_name(subsystem);
    if (!vt || !vt->subscriptions) continue;

    const process_subscription_entry_t* handler =
      find_subscription_handler(vt, topic);

    if (!handler || !handler->handler) continue;

    handler->handler(payload);
  }
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// PUBLISH — invoke local + Pi-forward publish path
// ------------------------------------------------------------
static Payload cmd_publish(const Payload& args) {

  if (!args.has("topic")) {
    Payload err;
    err.add("error", "missing topic");
    return err;
  }

  const char* topic = args.getString("topic");
  if (!topic || !*topic) {
    Payload err;
    err.add("error", "invalid topic");
    return err;
  }

  Payload payload;
  if (args.has("payload")) {
    payload = args.getPayload("payload");
  }

  publish(topic, payload);
  return ok_payload();
}

// ------------------------------------------------------------
// SETSUBSCRIPTIONS — authoritative apply (clobber-and-go)
//
// Payload shape:
// {
//   "subscriptions": [
//     {
//       "machine": "...",
//       "subsystem": "...",
//       "subscriptions": [ { "name": "TOPIC" } ]
//     }
//   ]
// }
// ------------------------------------------------------------
static Payload cmd_setsubscriptions(const Payload& args) {

  debug_log("cmd_setsubscriptions", args);

  if (!args.hasArray("subscriptions")) {
    Payload err;
    err.add("error", "missing subscriptions");
    return err;
  }

  clear_state();

  // Preserve union payload verbatim
  g_union_payload = args;

  // ------------------------------------------------------------
  // Derive Cartesian routing edges
  // ------------------------------------------------------------

  PayloadArray routes;

  PayloadArrayView rows = args.getArrayView("subscriptions");

  for (size_t i = 0; i < rows.size(); i++) {

    Payload row = rows.get(i);

    const char* machine   = row.getString("machine");
    const char* subsystem = row.getString("subsystem");

    if (!machine || !*machine) continue;
    if (!subsystem || !*subsystem) continue;
    if (!row.hasArray("subscriptions")) continue;

    PayloadArrayView topics = row.getArrayView("subscriptions");

    for (size_t j = 0; j < topics.size(); j++) {

      Payload t = topics.get(j);
      const char* topic = t.getString("name");
      if (!topic || !*topic) continue;

      Payload edge;
      edge.add("machine", machine);
      edge.add("subsystem", subsystem);
      edge.add("topic", topic);

      routes.add(edge);
    }
  }

  g_routes.add_array("routes", routes);

  Payload resp;
  resp.add("status", "applied");
  return resp;
}

// ------------------------------------------------------------
// REPORT — expose execution truth (Cartesian edge list)
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  if (!g_routes.hasArray("routes")) {
    Payload resp;
    resp.add("routes", "none");
    return resp;
  }

  // Return execution truth verbatim
  return g_routes;
}

// ------------------------------------------------------------
// ALLSUBSCRIPTIONS — declared consumption truth (vtable-owned)
// ------------------------------------------------------------
static Payload cmd_allsubscriptions(const Payload&) {

  Payload resp;
  PayloadArray out;

  const size_t n = process_get_count();

  for (size_t i = 0; i < n; i++) {

    const process_vtable_t* vt = process_get_vtable(i);
    if (!vt || !vt->process_id) continue;
    if (!vt->subscriptions) continue;

    Payload row;
    row.add("machine", "TEENSY");
    row.add("subsystem", vt->process_id);

    PayloadArray subs;

    for (const process_subscription_entry_t* s = vt->subscriptions;
         s && s->topic;
         ++s) {

      Payload e;
      e.add("name", s->topic);
      subs.add(e);
    }

    row.add_array("subscriptions", subs);
    out.add(row);
  }

  resp.add_array("subscriptions", out);
  return resp;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PUBSUB_COMMANDS[] = {
  { "PUBLISH",          cmd_publish          },
  { "SETSUBSCRIPTIONS", cmd_setsubscriptions },
  { "REPORT",           cmd_report           },
  { "ALLSUBSCRIPTIONS", cmd_allsubscriptions },
  { nullptr,            nullptr              }
};

static const process_vtable_t PUBSUB_PROCESS = {
  .process_id    = "PUBSUB",
  .commands      = PUBSUB_COMMANDS,
  .subscriptions = nullptr
};

void process_pubsub_register(void) {
  process_register("PUBSUB", &PUBSUB_PROCESS);
}