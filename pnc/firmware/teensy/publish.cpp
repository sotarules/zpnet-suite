// ============================================================================
// FILE: publish.cpp
// ============================================================================
//
// ZPNet Publish API — Semantic Pub/Sub Egress
//
// Responsibilities:
//   • Provide a simple publish(topic, payload) API
//   • Deliver publications locally via process framework
//   • Forward publications to the Pi for authoritative routing
//
// Non-responsibilities:
//   • Subscription storage
//   • Topic matching
//   • Fan-out logic
//   • Process introspection
//
// All local routing is owned by the process framework.
//
// ============================================================================

#include "publish.h"

#include "process.h"
#include "transport.h"

// -----------------------------------------------------------------------------
// Local delivery (delegated)
// -----------------------------------------------------------------------------
//
// All local pub/sub delivery is handled by the process framework.
// This function exists only as a semantic hook.
//

void publish_local(const char* topic, const Payload& payload) {
  if (!topic || !*topic) return;
  process_publish_dispatch(topic, payload);
}

// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------
//
// Semantics:
//   • Synchronous local delivery
//   • Unconditional forward to Pi
//   • No inference, no filtering
//

void publish(const char* topic, const Payload& payload) {

  if (!topic || !*topic) return;

  // 1) Local synchronous delivery
  publish_local(topic, payload);

  // 2) Forward to Pi (authoritative routing)
  Payload envelope;
  envelope.add("topic", topic);
  envelope.add_object("payload", payload);

  transport_send(
    TRAFFIC_PUBLISH_SUBSCRIBE,
    envelope
  );
}