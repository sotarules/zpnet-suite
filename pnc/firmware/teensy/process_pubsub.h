#pragma once

#include "payload.h"

// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE
void process_publish_dispatch(const Payload& message);
