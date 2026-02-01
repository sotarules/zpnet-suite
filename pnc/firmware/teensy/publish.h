#pragma once

#include "payload.h"

// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------

void publish(const char* topic, const Payload& payload);

void publish_local(const char* topic, const Payload& payload);

// -----------------------------------------------------------------------------
// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
// -----------------------------------------------------------------------------
//
// Called by transport when a publication arrives from the Pi.
// This MUST NOT forward traffic back to the Pi.
//

void process_publish_dispatch(const Payload& message);