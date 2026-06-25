#pragma once

#include "payload.h"

#include <stdint.h>
// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------

void publish(const char* topic, const Payload& payload);

void publish_local(const char* topic, const Payload& payload);

// -----------------------------------------------------------------------------
// Publish diagnostics
// -----------------------------------------------------------------------------
//
// publish() is the semantic egress boundary shared by TIMEBASE, EVENTS,
// FEATURE_STATUS, dashboard feeds, and watchdog publications.  These counters
// expose whether anything is attempting to publish from unsafe exception
// context or reenter publish() while a prior publish is still building its
// local dispatch/envelope/transport path.

typedef struct {
  uint32_t publish_calls;
  uint32_t publish_invalid_topic;

  uint32_t publish_unsafe_context_drop;
  uint32_t publish_reentrant_drop;
  uint32_t publish_depth_high_water;
  bool     publish_active;

  uint32_t publish_local_dispatch;
  uint32_t publish_forward_attempt;
  uint32_t publish_envelope_build;
  uint32_t publish_transport_send;
} publish_info_t;

void publish_get_info(publish_info_t* out);

// -----------------------------------------------------------------------------
// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
// -----------------------------------------------------------------------------
//
// Called by transport when a publication arrives from the Pi.
// This MUST NOT forward traffic back to the Pi.
//

void process_publish_dispatch(const Payload& message);