#pragma once

#include "payload.h"

// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------

// Returns true only when the Pi-bound envelope enters transport custody.
// Local synchronous delivery still occurs before the forwarding attempt.
bool publish(const char* topic, const Payload& payload);

// Forward without repeating local delivery. Transport authors the pub/sub
// envelope directly into its final wire image, so no second full Payload copy
// is held while the queued transport allocation is acquired. This is reserved
// for retrying a publication that already completed local synchronous dispatch.
bool publish_to_pi(const char* topic, const Payload& payload);

void publish_local(const char* topic, const Payload& payload);

// -----------------------------------------------------------------------------
// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
// -----------------------------------------------------------------------------
//
// Called by transport when a publication arrives from the Pi.
// This MUST NOT forward traffic back to the Pi.
//

void process_publish_dispatch(const Payload& message);