#include "bootloader.h"
#include "debug.h"
#include "command.h"
#include "event_bus.h"
#include "util.h"
#include "system.h"
#include "process.h"

#include "laser.h"
#include "photodiode.h"
#include "teensy_status.h"
#include "transport.h"

#include <string.h>
#include <stdlib.h>

// =============================================================
// Helpers — req_id extraction (optional)
// =============================================================

static bool extractReqId(const char* line, uint32_t& out) {
    const char* p = strstr(line, "\"req_id\"");
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;

    out = (uint32_t)strtoul(p, nullptr, 10);
    return true;
}

// =============================================================
// Helpers — Immediate JSON Response Emission (SOLE PATH)
// =============================================================

static inline void emitImmediateResponse(
    bool success,
    const char* message,
    const String* payload,   // optional JSON body fragment
    bool has_req_id,
    uint32_t req_id
) {
    String out;
    out += "{";

    if (has_req_id) {
        out += "\"req_id\":";
        out += req_id;
        out += ",";
    }

    out += "\"success\":";
    out += success ? "true" : "false";

    out += ",\"message\":\"";
    out += message ? message : "";
    out += "\"";

    if (payload && payload->length() > 0) {
        out += ",\"payload\":";
        out += *payload;
    }

    out += "}";

    transport_send_frame(out.c_str(), out.length());
}

static inline void emitOK(
    const String* payload,
    bool has_req_id,
    uint32_t req_id
) {
    emitImmediateResponse(true, "OK", payload, has_req_id, req_id);
}

static inline void emitError(
    const char* message,
    bool has_req_id,
    uint32_t req_id
) {
    // Wrap the error text as JSON in a payload object
    String errPayload;
    errPayload += "{";
    errPayload += "\"message\":\"";
    errPayload += message ? message : "";
    errPayload += "\"}";
    emitImmediateResponse(false, message, &errPayload, has_req_id, req_id);
}

// =============================================================
// Parsing helpers
// =============================================================

static bool extractCmd(const char* line, char* out, size_t out_sz) {
    const char* p = strstr(line, "\"cmd\"");
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;

    while (*p == ' ') p++;
    if (*p != '\"') return false;
    p++;

    const char* q = strchr(p, '\"');
    if (!q) return false;

    size_t n = q - p;
    if (n >= out_sz) n = out_sz - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static bool parseProcessType(const char* line, process_type_t& out) {\
    if (strstr(line, "\"type\":\"CLOCKS\"")) { out = PROCESS_TYPE_CLOCKS; return true; }
    if (strstr(line, "\"type\":\"LASER\"")) { out = PROCESS_TYPE_LASER; return true; }
    if (strstr(line, "\"type\":\"PHOTODIODE\"")) { out = PROCESS_TYPE_PHOTODIODE; return true; }
    if (strstr(line, "\"type\":\"TEMPEST\"")) { out = PROCESS_TYPE_TEMPEST; return true; }
    if (strstr(line, "\"type\":\"LANTERN\"")) { out = PROCESS_TYPE_LANTERN; return true; }
    return false;
}

// =============================================================
// Command Execution — NORMALIZED
// =============================================================

void command_exec(const char* line) {

    uint32_t req_id = 0;
    bool has_req_id = extractReqId(line, req_id);

    char cmd[64];
    if (!extractCmd(line, cmd, sizeof(cmd))) {
        emitError("missing cmd", has_req_id, req_id);
        return;
    }

    // ---------------------------------------------------------
    // EVENT BUS CONTROL
    // ---------------------------------------------------------
    if (strcmp(cmd, "EVENTS.GET") == 0) {
        drainEventsNow();
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    // ---------------------------------------------------------
    // PROCESS COMMANDS (ALL NORMALIZED)
    // ---------------------------------------------------------

    if (strcmp(cmd, "PROCESS.LIST") == 0) {
        String payload = process_list_json();
        emitOK(&payload, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PROCESS.START") == 0) {
        process_type_t type;
        if (!parseProcessType(line, type)) {
            emitError("missing or invalid process type", has_req_id, req_id);
            return;
        }
        if (!process_start(type)) {
            emitError("process start failed", has_req_id, req_id);
            return;
        }
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PROCESS.STOP") == 0) {
        process_type_t type;
        if (!parseProcessType(line, type)) {
            emitError("missing or invalid process type", has_req_id, req_id);
            return;
        }
        if (!process_stop(type)) {
            emitError("process stop failed", has_req_id, req_id);
            return;
        }
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PROCESS.QUERY") == 0) {
        process_type_t type;
        if (!parseProcessType(line, type)) {
            emitError("missing or invalid process type", has_req_id, req_id);
            return;
        }

        String payload;
        if (!process_query(type, payload)) {
            emitError("process not found", has_req_id, req_id);
            return;
        }

        emitOK(&payload, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PROCESS.COMMAND") == 0) {
        process_type_t type;
        if (!parseProcessType(line, type)) {
            emitError("missing or invalid process type", has_req_id, req_id);
            return;
        }

        const char* p = strstr(line, "\"proc_cmd\"");
        if (!p) {
            emitError("missing proc_cmd", has_req_id, req_id);
            return;
        }

        p = strchr(p, ':');
        if (!p) {
            emitError("malformed proc_cmd", has_req_id, req_id);
            return;
        }

        p++;
        while (*p == ' ') p++;
        if (*p != '\"') {
            emitError("malformed proc_cmd", has_req_id, req_id);
            return;
        }
        p++;

        char proc_cmd[64];
        const char* q = strchr(p, '\"');
        if (!q) {
            emitError("malformed proc_cmd", has_req_id, req_id);
            return;
        }

        size_t n = q - p;
        if (n >= sizeof(proc_cmd)) n = sizeof(proc_cmd) - 1;
        memcpy(proc_cmd, p, n);
        proc_cmd[n] = '\0';

        const char* args_json = nullptr;

        const char* a = strstr(line, "\"args\"");
        if (a) {
            a = strchr(a, ':');
            if (a) {
                a++;
                while (*a == ' ') a++;
                if (*a == '{') {
                    args_json = a;
                }
            }
        }

        String payload;
        if (!process_command(type, proc_cmd, args_json, payload)) {
            emitError("process command failed", has_req_id, req_id);
            return;
        }

        emitOK(payload.length() ? &payload : nullptr, has_req_id, req_id);
        return;
    }

    // ---------------------------------------------------------
    // LEGACY / DIRECT COMMANDS
    // ---------------------------------------------------------

    if (strcmp(cmd, "TEENSY.STATUS") == 0) {
        String payload = buildTeensyStatusBody();
        emitOK(&payload, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PHOTODIODE.STATUS") == 0) {
        String payload = buildPhotodiodeStatusBody();
        emitOK(&payload, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PHOTODIODE.COUNT") == 0) {
        String payload = buildPhotodiodeCountBody();
        emitOK(&payload, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "PHOTODIODE.CLEAR") == 0) {
        photodiode_clear();
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "LASER.ON") == 0) {
        laser_on();
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "LASER.OFF") == 0) {
        laser_off();
        emitOK(nullptr, has_req_id, req_id);
        return;
    }

    if (strcmp(cmd, "SYSTEM.SHUTDOWN") == 0) {
        emitOK(nullptr, has_req_id, req_id);
        system_request_shutdown();
        return;
    }

    // ---------------------------------------------------------
    // UNKNOWN
    // ---------------------------------------------------------

    emitError("unknown command", has_req_id, req_id);
}
