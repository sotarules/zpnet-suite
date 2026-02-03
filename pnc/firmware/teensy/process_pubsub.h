#pragma once

#include "payload.h"

const Payload* pubsub_get_subscriptions();

// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE
void process_publish_dispatch(const Payload& message);

// Register the PUBSUB process command surface
void process_pubsub_register(void);

// Fan-out a published message using Cartesian routing truth
void process_pubsub_fanout(
  const char* topic,
  const Payload& payload
);