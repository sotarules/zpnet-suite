#pragma once

#include "payload.h"

// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------

enum publish_trace_stage_t : uint32_t {
  PUBLISH_TRACE_NONE = 0U,
  PUBLISH_TRACE_ENTER = 1U,
  PUBLISH_TRACE_LOCAL_ENTER = 2U,
  PUBLISH_TRACE_LOCAL_RETURN = 3U,
  PUBLISH_TRACE_ENVELOPE_READY = 4U,
  PUBLISH_TRACE_TOPIC_ADDED = 5U,
  PUBLISH_TRACE_PAYLOAD_ADDED = 6U,
  PUBLISH_TRACE_TRANSPORT_ENTER = 7U,
  PUBLISH_TRACE_TRANSPORT_RETURN = 8U,
  PUBLISH_TRACE_COMPLETE = 9U,
};

struct publish_trace_info_t {
  uint32_t valid;
  uint32_t sequence;
  uint32_t stage;
  uint32_t dwt;
  uint32_t topic_ptr;
  uint32_t payload_ptr;
  uint32_t envelope_ptr;
};

void publish(const char* topic, const Payload& payload);
void publish_trace_snapshot(publish_trace_info_t* out);
const char* publish_trace_stage_name(uint32_t stage);

void publish_local(const char* topic, const Payload& payload);

// -----------------------------------------------------------------------------
// Entry point for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
// -----------------------------------------------------------------------------
//
// Called by transport when a publication arrives from the Pi.
// This MUST NOT forward traffic back to the Pi.
//

void process_publish_dispatch(const Payload& message);