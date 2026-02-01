#include "publish.h"

#include "process.h"
#include "transport.h"

#include <string.h>

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static inline bool topic_matches(const char* a, const char* b) {
  return a && b && strcmp(a, b) == 0;
}

// -----------------------------------------------------------------------------
// Local fan-out (shared by all pub/sub paths)
// -----------------------------------------------------------------------------

void publish_local(const char* topic, const Payload& payload) {
  if (!topic || !*topic) return;

  size_t count = process_get_count();

  for (size_t i = 0; i < count; i++) {

    const process_vtable_t* v = process_get_vtable(i);
    if (!v || !v->subscriptions || !v->on_message) {
      continue;
    }

    for (size_t s = 0; s < v->subscription_count; s++) {
      const char* sub = v->subscriptions[s].topic;
      if (topic_matches(sub, topic)) {
        v->on_message(topic, payload);
        break; // one delivery per process
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Public publish()
// -----------------------------------------------------------------------------

void publish(const char* topic, const Payload& payload) {

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