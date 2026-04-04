// ============================================================================
// process.cpp — ZPNet Process Framework
// ============================================================================
//
// Subscriptions:
//   • NO persistence
//   • NO local storage
//   • Subscription truth is owned by PUBSUB (volatile, RAM-only)
//
// Handlers:
//   • Handler functions live in process vtables (code truth)
//
// Instrumentation:
//   • RPC command pipeline is fully counted
//   • process_get_rpc_info() provides snapshot for PROCESS_INFO report
//
// ============================================================================

#include "process.h"
#include "transport.h"
#include "debug.h"
#include "process_pubsub.h"

#include <Arduino.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

static constexpr size_t MAX_PROCESSES = 30;
static constexpr size_t RESPONSE_PREFLIGHT_JSON_MAX = 6144;

// ============================================================================
// Registry Entry
// ============================================================================

typedef struct {
    const char*             id;
    const process_vtable_t* vtable;
} process_entry_t;

// ============================================================================
// Registry State (commands only)
// ============================================================================

static process_entry_t registry[MAX_PROCESSES];
static size_t registry_count = 0;

// ============================================================================
// RPC Pipeline Counters
// ============================================================================

static volatile uint32_t rpc_received              = 0;
static volatile uint32_t rpc_routed                = 0;
static volatile uint32_t rpc_handler_invoked       = 0;
static volatile uint32_t rpc_handler_completed     = 0;
static volatile uint32_t rpc_response_sent         = 0;
static volatile uint32_t rpc_error_missing_fields  = 0;
static volatile uint32_t rpc_error_unknown_subsys  = 0;
static volatile uint32_t rpc_error_unknown_command = 0;
static volatile uint32_t rpc_error_response_sent   = 0;

static volatile uint32_t ps_dispatched             = 0;

// ============================================================================
// Helpers
// ============================================================================

static process_entry_t* find_process(const char* process_id) {
    if (!process_id) return nullptr;

    for (size_t i = 0; i < registry_count; i++) {
        if (strcmp(registry[i].id, process_id) == 0) {
            return &registry[i];
        }
    }
    return nullptr;
}

static const process_command_entry_t*
find_command(const process_vtable_t* vtable, const char* name) {
    if (!vtable || !vtable->commands || !name) return nullptr;

    for (const process_command_entry_t* e = vtable->commands;
         e->name;
         ++e) {
        if (strcmp(e->name, name) == 0) {
            return e;
        }
    }

    return nullptr;
}

// ============================================================================
// Canonical helpers
// ============================================================================

Payload ok_payload() {
    Payload p;
    p.add("status", "ok");
    return p;
}

static Payload make_error_payload(const char* msg) {
    Payload p;
    p.add("error", msg ? msg : "unknown error");
    return p;
}

static void copy_request_metadata_into_response(const Payload& request, Payload& response) {
    if (request.has("req_id")) {
        response.add("req_id", request.getUInt("req_id"));
    }

    if (request.has("req_ts_ms")) {
        response.add("req_ts_ms", request.getUInt("req_ts_ms"));
    }
}

static void send_overflow_response(const Payload& request) {
    Payload response;
    copy_request_metadata_into_response(request, response);
    response.add("success", false);
    response.add("message", "OVERFLOW");
    response.add_object("payload", make_error_payload("payload_overflow"));
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    rpc_error_response_sent++;
}

static bool response_serializes(const Payload& response) {
    char local_buf[RESPONSE_PREFLIGHT_JSON_MAX];
    return response.write_json(local_buf, sizeof(local_buf)) > 0;
}

static void send_response_or_overflow(const Payload& request, const Payload& response) {
    if (!response_serializes(response)) {
        send_overflow_response(request);
        return;
    }

    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
}

// ============================================================================
// Lifecycle
// ============================================================================

void process_init(void) {
    registry_count = 0;
}

// ============================================================================
// Registration (commands + handler capability only)
// ============================================================================

bool process_register(
    const char*             process_id,
    const process_vtable_t* vtable
) {
    if (!process_id || !vtable || !vtable->process_id) {
        return false;
    }

    if (strcmp(process_id, vtable->process_id) != 0) {
        return false;
    }

    if (registry_count >= MAX_PROCESSES) {
        return false;
    }

    if (find_process(process_id)) {
        return false;
    }

    registry[registry_count].id     = process_id;
    registry[registry_count].vtable = vtable;
    registry_count++;

    return true;
}

// ============================================================================
// Registry Introspection (READ-ONLY)
// ============================================================================

size_t process_get_count(void) {
    return registry_count;
}

const char* process_get_name(size_t idx) {
    if (idx >= registry_count) return nullptr;
    return registry[idx].id;
}

const process_vtable_t* process_get_vtable(size_t idx) {
    if (idx >= registry_count) return nullptr;
    return registry[idx].vtable;
}

const process_vtable_t* process_get_vtable_by_name(const char* name) {
    if (!name) return nullptr;

    for (size_t i = 0; i < registry_count; i++) {
        if (strcmp(registry[i].id, name) == 0) {
            return registry[i].vtable;
        }
    }
    return nullptr;
}

// ============================================================================
// REQUEST / RESPONSE Command Processor (INSTRUMENTED)
// ============================================================================

void process_command(const Payload& request) {

    rpc_received++;

    Payload response;
    copy_request_metadata_into_response(request, response);

    const char* subsystem_c = request.getString("subsystem");
    const char* command_c   = request.getString("command");

    // ---- Error path: missing routing fields ----

    if (!subsystem_c || !command_c) {
        rpc_error_missing_fields++;
        response.add("success", false);
        response.add("message", "BAD");
        response.add_object("payload",
                            make_error_payload("missing routing fields"));
        send_response_or_overflow(request, response);
        return;
    }

    // ---- Error path: unknown subsystem ----

    process_entry_t* proc = find_process(subsystem_c);
    if (!proc) {
        rpc_error_unknown_subsys++;
        response.add("success", false);
        response.add("message", "BAD");
        response.add_object("payload",
                            make_error_payload("unknown subsystem"));
        send_response_or_overflow(request, response);
        return;
    }

    // ---- Error path: unknown command ----

    const process_command_entry_t* entry =
        find_command(proc->vtable, command_c);

    if (!entry || !entry->handler) {
        rpc_error_unknown_command++;
        response.add("success", false);
        response.add("message", "BAD");
        response.add_object("payload",
                            make_error_payload("unknown command"));
        send_response_or_overflow(request, response);
        return;
    }

    // ---- Happy path ----

    rpc_routed++;

    Payload args;
    if (request.has("args")) {
        args = request.getPayload("args");
    }

    rpc_handler_invoked++;

    Payload payload = entry->handler(args);

    rpc_handler_completed++;

    response.add("success", true);
    response.add("message", "OK");
    response.add_object("payload", payload);

    send_response_or_overflow(request, response);

    rpc_response_sent++;
}

// ============================================================================
// RPC Info Snapshot
// ============================================================================

void process_get_rpc_info(process_rpc_info_t* out) {
    if (!out) return;

    out->received              = rpc_received;
    out->routed                = rpc_routed;
    out->handler_invoked       = rpc_handler_invoked;
    out->handler_completed     = rpc_handler_completed;
    out->response_sent         = rpc_response_sent;
    out->error_missing_fields  = rpc_error_missing_fields;
    out->error_unknown_subsys  = rpc_error_unknown_subsys;
    out->error_unknown_command = rpc_error_unknown_command;
    out->error_response_sent   = rpc_error_response_sent;
    out->ps_dispatched         = ps_dispatched;
}

// ============================================================================
// PUB / SUB Dispatch — delegated to PUBSUB
// ============================================================================

void process_publish_dispatch(
    const char* topic,
    const Payload& payload
) {
    if (!topic || !*topic) return;

    ps_dispatched++;

    process_pubsub_fanout(topic, payload);
}