// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Static Formal Routing Topology (Teensy)
//
// Responsibilities:
//   • Own the complete formal route graph as code truth
//   • Expose that exact Cartesian route graph through REPORT
//   • Receive the data plane from Pi
//
// Semantics:
//   • No subsystem declares subscriptions
//   • No runtime command may mutate topology
//   • Pi and Teensy carry the same six formal route edges at boot
//   • The current formal topology has no TEENSY delivery targets
//
// =============================================================

#include "process_pubsub.h"

#include "process.h"
#include "publish.h"
#include "payload.h"

// ================================================================
// Canonical formal route graph
// ================================================================
//
// This is intentionally duplicated in Pi pubsub/core.py.  It is small,
// reviewable code truth rather than discovered runtime state.  The current
// union has PI recipients only; adding a TEENSY recipient requires adding the
// explicit local handler dispatch in this file in the same change.

struct pubsub_static_route_t {
  const char* machine;
  const char* subsystem;
  const char* topic;
};

static constexpr pubsub_static_route_t STATIC_ROUTES[] = {
  { "PI", "CLOCKS",  "CLOCKS_FRAGMENT" },
  { "PI", "CLOCKS",  "CLOCKS_RECOVERY_STALLED" },
  { "PI", "CLOCKS",  "WATCHDOG_ANOMALY" },
  { "PI", "EVENTS",  "EVENTS" },
  { "PI", "PHOTONS", "PHOTONS_FRAGMENT" },
  { "PI", "SYSTEM",  "GNSS_ANNOUNCEMENT" },
};

static constexpr size_t STATIC_ROUTE_COUNT =
    sizeof(STATIC_ROUTES) / sizeof(STATIC_ROUTES[0]);

static Payload g_routes;
static bool g_routes_ready = false;

static void ensure_static_routes(void) {
  if (g_routes_ready) return;

  PayloadArray routes;
  for (size_t i = 0; i < STATIC_ROUTE_COUNT; i++) {
    Payload edge;
    edge.add("machine", STATIC_ROUTES[i].machine);
    edge.add("subsystem", STATIC_ROUTES[i].subsystem);
    edge.add("topic", STATIC_ROUTES[i].topic);
    routes.add(edge);
  }

  g_routes.clear();
  g_routes.add_array("routes", routes);
  g_routes_ready = true;
}

bool pubsub_routes_ready(void) {
  ensure_static_routes();
  return true;
}

// ================================================================
// Accessors
// ================================================================

const Payload* pubsub_get_subscriptions() {
  ensure_static_routes();
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
  ensure_static_routes();

  // The canonical route graph above currently contains no TEENSY recipients.
  // Keep this entry point because publish() and Pi-origin data-plane ingress are
  // symmetric, but there is deliberately no generic process-level subscription
  // handler contract anymore.  A future TEENSY route must wire its handler here
  // explicitly beside the static route addition.
  (void)payload;
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
// REPORT — expose execution truth (Cartesian edge list)
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {
  ensure_static_routes();
  return g_routes;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PUBSUB_COMMANDS[] = {
  { "PUBLISH", cmd_publish },
  { "REPORT",  cmd_report  },
  { nullptr,    nullptr     }
};

static const process_vtable_t PUBSUB_PROCESS = {
  .process_id = "PUBSUB",
  .commands   = PUBSUB_COMMANDS,
};

void process_pubsub_register(void) {
  ensure_static_routes();
  process_register("PUBSUB", &PUBSUB_PROCESS);
}