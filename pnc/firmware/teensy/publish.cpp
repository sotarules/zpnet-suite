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
#include <Arduino.h>

static publish_trace_info_t g_publish_trace DMAMEM;

static void publish_trace_note(uint32_t stage, const char* topic,
                               const Payload* payload, const Payload* envelope) {
  uint32_t sequence = g_publish_trace.sequence + 1U;
  if (sequence == 0U) sequence = 1U;
  g_publish_trace.valid = 0U;
  g_publish_trace.sequence = sequence;
  g_publish_trace.stage = stage;
  g_publish_trace.dwt = ARM_DWT_CYCCNT;
  g_publish_trace.topic_ptr = (uint32_t)(uintptr_t)topic;
  g_publish_trace.payload_ptr = (uint32_t)(uintptr_t)payload;
  g_publish_trace.envelope_ptr = (uint32_t)(uintptr_t)envelope;
  __asm__ volatile("dmb" ::: "memory");
  g_publish_trace.valid = 1U;
  arm_dcache_flush(&g_publish_trace, sizeof(g_publish_trace));
}

void publish_trace_snapshot(publish_trace_info_t* out) { if (out) *out = g_publish_trace; }

const char* publish_trace_stage_name(uint32_t stage) {
  switch (stage) {
    case PUBLISH_TRACE_ENTER: return "ENTER";
    case PUBLISH_TRACE_LOCAL_ENTER: return "LOCAL_ENTER";
    case PUBLISH_TRACE_LOCAL_RETURN: return "LOCAL_RETURN";
    case PUBLISH_TRACE_ENVELOPE_READY: return "ENVELOPE_READY";
    case PUBLISH_TRACE_TOPIC_ADDED: return "TOPIC_ADDED";
    case PUBLISH_TRACE_PAYLOAD_ADDED: return "PAYLOAD_ADDED";
    case PUBLISH_TRACE_TRANSPORT_ENTER: return "TRANSPORT_ENTER";
    case PUBLISH_TRACE_TRANSPORT_RETURN: return "TRANSPORT_RETURN";
    case PUBLISH_TRACE_COMPLETE: return "COMPLETE";
    default: return "NONE";
  }
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
  if (!topic || !*topic) return;
  publish_trace_note(PUBLISH_TRACE_ENTER, topic, &payload, nullptr);
  publish_trace_note(PUBLISH_TRACE_LOCAL_ENTER, topic, &payload, nullptr);
  publish_local(topic, payload);
  publish_trace_note(PUBLISH_TRACE_LOCAL_RETURN, topic, &payload, nullptr);

  Payload envelope;
  publish_trace_note(PUBLISH_TRACE_ENVELOPE_READY, topic, &payload, &envelope);
  if (!envelope.add("topic", topic)) return;
  publish_trace_note(PUBLISH_TRACE_TOPIC_ADDED, topic, &payload, &envelope);
  if (!envelope.add_object("payload", payload)) return;
  publish_trace_note(PUBLISH_TRACE_PAYLOAD_ADDED, topic, &payload, &envelope);
  publish_trace_note(PUBLISH_TRACE_TRANSPORT_ENTER, topic, &payload, &envelope);
  transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, envelope);
  publish_trace_note(PUBLISH_TRACE_TRANSPORT_RETURN, topic, &payload, &envelope);
  publish_trace_note(PUBLISH_TRACE_COMPLETE, topic, &payload, &envelope);
}
