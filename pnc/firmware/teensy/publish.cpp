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

#include <stdint.h>

// -----------------------------------------------------------------------------
// Publish diagnostics and safety rails
// -----------------------------------------------------------------------------

static volatile bool     g_publish_active = false;
static volatile uint32_t g_publish_calls = 0;
static volatile uint32_t g_publish_invalid_topic = 0;
static volatile uint32_t g_publish_unsafe_context_drop = 0;
static volatile uint32_t g_publish_reentrant_drop = 0;
static volatile uint32_t g_publish_depth_high_water = 0;
static volatile uint32_t g_publish_local_dispatch = 0;
static volatile uint32_t g_publish_forward_attempt = 0;
static volatile uint32_t g_publish_envelope_build = 0;
static volatile uint32_t g_publish_transport_send = 0;

static inline bool publish_in_exception_context(void) {
#if defined(__arm__) || defined(__thumb__)
  uint32_t ipsr = 0;
  __asm__ volatile ("mrs %0, ipsr" : "=r" (ipsr));
  return ipsr != 0;
#else
  return false;
#endif
}

void publish_get_info(publish_info_t* out) {
  if (!out) return;

  out->publish_calls = g_publish_calls;
  out->publish_invalid_topic = g_publish_invalid_topic;
  out->publish_unsafe_context_drop = g_publish_unsafe_context_drop;
  out->publish_reentrant_drop = g_publish_reentrant_drop;
  out->publish_depth_high_water = g_publish_depth_high_water;
  out->publish_active = g_publish_active;
  out->publish_local_dispatch = g_publish_local_dispatch;
  out->publish_forward_attempt = g_publish_forward_attempt;
  out->publish_envelope_build = g_publish_envelope_build;
  out->publish_transport_send = g_publish_transport_send;
}

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

  g_publish_calls++;

  if (!topic || !*topic) {
    g_publish_invalid_topic++;
    return;
  }

  // Payload construction, local dispatch, transport enqueue, and malloc-backed
  // serialization are foreground activities.  If a future caller attempts to
  // publish from an ISR/exception context, drop the publication and make the
  // violation visible rather than corrupting shared Payload/transport state.
  if (publish_in_exception_context()) {
    g_publish_unsafe_context_drop++;
    return;
  }

  // publish() is not reentrant.  Local pub/sub delivery may call arbitrary
  // subscriber code before the outbound envelope is constructed.  A nested
  // publish would overlap Payload/envelope/transport work with the active
  // publication; drop it and count it.
  if (g_publish_active) {
    g_publish_reentrant_drop++;
    return;
  }

  g_publish_active = true;
  if (g_publish_depth_high_water < 1) g_publish_depth_high_water = 1;

  // 1) Local synchronous delivery
  g_publish_local_dispatch++;
  publish_local(topic, payload);

  // 2) Forward to Pi (authoritative routing)
  g_publish_forward_attempt++;
  Payload envelope;
  envelope.add("topic", topic);
  envelope.add_object("payload", payload);
  g_publish_envelope_build++;

  transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, envelope);
  g_publish_transport_send++;

  g_publish_active = false;
}