// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Volatile Subscription Control Plane (Teensy)
//
// Responsibilities:
//   • Maintain volatile subscription truth (RAM-only, authoritative)
//   • Apply subscription sets exactly as delivered by Pi
//   • Report applied subscription truth verbatim
//   • Expose declared subscription intent (ALLSUBSCRIPTIONS)
//
// Semantics:
//   • Pi is authoritative
//   • Teensy never infers or reshapes intent
//   • Subscription state is clobber-and-go
//   • No persistence
//
// Non-responsibilities:
//   • Routing
//   • Fan-out
//   • Negotiation
//   • Announcement
//   • Transport policy
//
// =============================================================

#include "process_pubsub.h"

#include "process.h"
#include "publish.h"
#include "payload.h"
#include "debug.h"

#include <string.h>

// ================================================================
// Subscription state (authoritative, opaque)
//
// Stored exactly as received from Pi.
// Shape:
//   {
//     "subscriptions": [ ... ]
//   }
//
// Interpreted later via PayloadArrayView.
// ================================================================

static Payload g_subscriptions;

// ================================================================
// Accessor
// ================================================================

const Payload* pubsub_get_subscriptions() {
  return &g_subscriptions;
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
// Payload shape (canonical, opaque):
// {
//   "subscriptions": [
//     {
//       "machine": "PI" | "TEENSY",
//       "subsystem": "NAME",
//       "subscriptions": [
//         { "name": "TOPIC" }
//       ]
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

  // Clobber-and-go: store exactly what was received
  g_subscriptions.clear();
  g_subscriptions = args;

  Payload resp;
  resp.add("status", "applied");
  return resp;
}

// ------------------------------------------------------------
// REPORT — reflect applied subscription truth verbatim
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;

  if (!g_subscriptions.has("subscriptions")) {
    resp.add("subscriptions", "none");
    return resp;
  }

  // Return exactly what we are holding
  resp = g_subscriptions;
  return resp;
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