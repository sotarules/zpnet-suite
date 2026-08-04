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
//   • Best-effort forward to Pi with an explicit custody verdict
//   • true means the complete Pi-bound wire image was enqueued
//   • No inference, no filtering
//

bool publish_to_pi(const char* topic, const Payload& payload) {

  if (!topic || !*topic) return false;

  // Transport authors the pub/sub envelope directly into its final wire image.
  // This avoids holding a second full Payload copy while allocating the queued
  // transport frame.
  return transport_send_publish(topic, payload);
}

bool publish(const char* topic, const Payload& payload) {

  if (!topic || !*topic) return false;

  // 1) Local synchronous delivery
  publish_local(topic, payload);

  // 2) Forward to Pi (authoritative routing)
  return publish_to_pi(topic, payload);
}