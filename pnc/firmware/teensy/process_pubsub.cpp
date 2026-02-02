// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Volatile Subscription Control Plane
//
// Responsibilities:
//   • Ingress for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
//   • Control-plane mutation of subscription state (RAM-only)
//   • Delegation to process_publish_dispatch(topic, payload)
//
// Semantics:
//   • Subscription state is VOLATILE
//   • Teensy does not persist subscriptions
//   • Pi is authoritative and replays subscriptions after reboot
//
// Non-responsibilities:
//   • Routing
//   • Fan-out
//   • Handler ownership
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
// Volatile subscription state (authoritative for this runtime only)
//
// Shape:
//   {
//     "PROCESS_ID": [
//       { "topic": "TOPIC_NAME" },
//       ...
//     ],
//     ...
//   }
//
// This state is cleared on reboot and must be restored by Pi replay.
// ================================================================

static Payload g_subscriptions;

// ================================================================
// Helpers (volatile subscription mutation)
// ================================================================

static bool topic_exists(const PayloadArray& arr, const char* topic) {
  if (!topic) return false;

  for (size_t i = 0; i < arr.size(); i++) {
    Payload e = arr.get(i);
    const char* t = e.getString("topic");
    if (t && strcmp(t, topic) == 0) {
      return true;
    }
  }
  return false;
}

static void apply_subscribe(const Payload& args) {

  // Accept either "process" or legacy "subsystem"
  const char* process = nullptr;

  if (args.has("process")) {
    process = args.getString("process");
  } else if (args.has("subsystem")) {
    process = args.getString("subsystem");
  }

  if (!process || !*process) {
    return;
  }

  // Accept either single "topic" or array "topics"
  if (args.has("topic")) {

    const char* topic = args.getString("topic");
    if (!topic || !*topic) return;

    PayloadArray arr;
    if (g_subscriptions.has(process)) {
      arr = g_subscriptions.getArray(process);
    }

    if (!topic_exists(arr, topic)) {
      Payload entry;
      entry.add("topic", topic);
      arr.add(entry);
    }

    g_subscriptions.add_array(process, arr);

    debug_log("pubsub.subscribe",
              String(process) + " -> " + topic);
  }

  else if (args.has("topics")) {

    PayloadArray topics = args.getArray("topics");

    for (size_t i = 0; i < topics.size(); i++) {
      Payload t = topics.get(i);
      const char* topic = t.getString(nullptr); // array of primitives
      if (!topic || !*topic) continue;

      PayloadArray arr;
      if (g_subscriptions.has(process)) {
        arr = g_subscriptions.getArray(process);
      }

      if (!topic_exists(arr, topic)) {
        Payload entry;
        entry.add("topic", topic);
        arr.add(entry);
      }

      g_subscriptions.add_array(process, arr);

      debug_log("pubsub.subscribe",
                String(process) + " -> " + topic);
    }
  }
}

static void apply_unsubscribe(const Payload& args) {

  if (!args.has("subsystem") || !args.has("topic")) {
    return;
  }

  const char* subsystem = args.getString("subsystem");
  const char* topic   = args.getString("topic");

  if (!subsystem || !*subsystem || !topic || !*topic) {
    return;
  }

  if (!g_subscriptions.has(subsystem)) {
    return;
  }

  PayloadArray old_arr = g_subscriptions.getArray(subsystem);
  PayloadArray new_arr;

  for (size_t i = 0; i < old_arr.size(); i++) {
    Payload e = old_arr.get(i);
    const char* t = e.getString("topic");
    if (t && strcmp(t, topic) != 0) {
      new_arr.add(e);
    }
  }

  g_subscriptions.add_array(subsystem, new_arr);

  debug_log("pubsub.unsubscribe",
            String(subsystem) + " -> " + topic);
}

const Payload* pubsub_get_subscriptions() {
  return &g_subscriptions;
}

// ================================================================
// Ingress: publications arriving from Pi (TRAFFIC_PUBLISH_SUBSCRIBE)
// ================================================================
//
// Payload shape (authoritative):
//   {
//     "topic":   "...",
//     "payload": { ... }
//   }
//
// Control-plane topics:
//   • SUBSCRIBE
//   • UNSUBSCRIBE
//
// All other topics are treated as data-plane publications.
// ================================================================

void process_publish_dispatch(const Payload& message) {

  if (!message.has("topic")) {
    return;
  }

  const char* topic = message.getString("topic");
  if (!topic || !*topic) {
    return;
  }

  Payload payload;
  if (message.has("payload")) {
    payload = message.getPayload("payload");
  }

  // ------------------------------------------------------------
  // Control-plane interception
  // ------------------------------------------------------------

  if (strcmp(topic, "SUBSCRIBE") == 0) {
    apply_subscribe(payload);
    return;
  }

  if (strcmp(topic, "UNSUBSCRIBE") == 0) {
    apply_unsubscribe(payload);
    return;
  }

  // ------------------------------------------------------------
  // Data-plane local delivery only
  // ------------------------------------------------------------

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
// SUBSCRIBE — mutate volatile subscription state
// ------------------------------------------------------------
static Payload cmd_subscribe(const Payload& args) {
  apply_subscribe(args);

  Payload resp;
  resp.add("status", "subscribed");
  return resp;
}

// ------------------------------------------------------------
// UNSUBSCRIBE — mutate volatile subscription state
// ------------------------------------------------------------
static Payload cmd_unsubscribe(const Payload& args) {
  apply_unsubscribe(args);

  Payload resp;
  resp.add("status", "unsubscribed");
  return resp;
}

// ------------------------------------------------------------
// REPORT — reflect volatile subscription truth
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;

  if (g_subscriptions.empty()) {
    resp.add("subscriptions", "none");
    return resp;
  }

  resp.add_object("subscriptions", g_subscriptions);
  return resp;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PUBSUB_COMMANDS[] = {
  { "PUBLISH",     cmd_publish     },
  { "SUBSCRIBE",   cmd_subscribe   },
  { "UNSUBSCRIBE", cmd_unsubscribe },
  { "REPORT",      cmd_report      },
  { nullptr,       nullptr         }
};

static const process_vtable_t PUBSUB_PROCESS = {
  .process_id    = "PUBSUB",
  .commands      = PUBSUB_COMMANDS,
  .subscriptions = nullptr
};

void process_pubsub_register(void) {
  process_register("PUBSUB", &PUBSUB_PROCESS);
}