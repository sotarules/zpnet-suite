// debug.cpp
//
// ZPNet Debug Facility (Teensy side)
//
// Routes all debug logs through transport_send() using traffic byte 0xD0.
//

#include "debug.h"

#include "transport.h"
#include "timepop.h"
#include "payload.h"

#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// =============================================================
// Uptime prefix helper
// =============================================================

static void debug_prefix(char* out, size_t out_sz, const char* name) {
    if (!out || out_sz == 0) return;

    unsigned long t = millis();

    if (name && *name) {
        snprintf(out, out_sz, "[%lu] %s: ", t, name);
    } else {
        snprintf(out, out_sz, "[%lu] ", t);
    }
}

// =============================================================
// Beacon
// =============================================================

static void debug_tick(timepop_ctx_t*, void*) {
    char msg[128];

    unsigned long ms = millis();
    unsigned long s  = ms / 1000;
    unsigned long m  = s / 60;
    unsigned long h  = m / 60;

    snprintf(
        msg,
        sizeof(msg),
        "uptime=%luh:%02lum:%02lus",
        h,
        m % 60,
        s % 60
    );

    debug_log("beacon", msg);
}

// =============================================================
// Initialization
// =============================================================

void debug_init(void) {
    debug_log("init", "=== ZPNet Debug Online ===");
}

void debug_beacon(void) {
    timepop_arm(
        TIMEPOP_CLASS_DEBUG_BEACON,
        true,
        debug_tick,
        nullptr,
        "debug_beacon"
    );
}

// =============================================================
// Core logger
// =============================================================

static void debug_emit(const char* full_msg) {
    if (!full_msg || !*full_msg) return;

    Payload p;
    p.add("debug", full_msg);
    transport_send(TRAFFIC_DEBUG, p);
}

void debug_log(const char* name, const char* msg) {
    if (!msg) return;

    char line[512];
    char prefix[64];

    debug_prefix(prefix, sizeof(prefix), name);
    snprintf(line, sizeof(line), "%s%s", prefix, msg);

    debug_emit(line);
}

void debug_log(const char* name, const String& value) {
    debug_log(name, value.c_str());
}

// =============================================================
// Payload overload (native)
// =============================================================
//
// Logs the serialized JSON form of a Payload.
// Uses json_view() (zero-alloc) and emits as one debug line.
//

void debug_log(const char* name, const Payload& p) {

    char line[512];
    char prefix[64];

    debug_prefix(prefix, sizeof(prefix), name);

    JsonView v = p.json_view();

    // Render prefix + JSON into bounded line buffer.
    // If it doesn't fit, we truncate (debug is observability, not protocol).
    size_t pos = 0;

    int n0 = snprintf(line, sizeof(line), "%s", prefix);
    if (n0 < 0) return;
    pos = (size_t)n0;
    if (pos >= sizeof(line)) {
        line[sizeof(line) - 1] = '\0';
        debug_emit(line);
        return;
    }

    size_t remaining = sizeof(line) - pos - 1; // leave room for '\0'
    size_t take = (v.len < remaining) ? v.len : remaining;

    if (take > 0) {
        memcpy(line + pos, v.data, take);
        pos += take;
    }

    line[pos] = '\0';

    debug_emit(line);
}

// =============================================================
// Scalar overloads
// =============================================================

#define DEBUG_SCALAR_FMT(fmt, value)                    \
    do {                                                \
        char line[256];                                 \
        char prefix[64];                                \
        debug_prefix(prefix, sizeof(prefix), name);     \
        snprintf(line, sizeof(line), "%s" fmt,          \
                 prefix, value);                        \
        debug_emit(line);                               \
    } while (0)

void debug_log(const char* name, int value)            { DEBUG_SCALAR_FMT("%d", value); }
void debug_log(const char* name, unsigned int value)   { DEBUG_SCALAR_FMT("%u", value); }
void debug_log(const char* name, int32_t value)        { DEBUG_SCALAR_FMT("%ld", (long)value); }
void debug_log(const char* name, uint32_t value)       { DEBUG_SCALAR_FMT("%lu", (unsigned long)value); }
void debug_log(const char* name, int64_t value)        { DEBUG_SCALAR_FMT("%lld", (long long)value); }
void debug_log(const char* name, uint64_t value)       { DEBUG_SCALAR_FMT("%llu", (unsigned long long)value); }
void debug_log(const char* name, float value)          { DEBUG_SCALAR_FMT("%.6f", value); }
void debug_log(const char* name, double value)         { DEBUG_SCALAR_FMT("%.9f", value); }
void debug_log(const char* name, bool value)           { DEBUG_SCALAR_FMT("%s", value ? "true" : "false"); }
void debug_log(const char* name, const void* ptr)      { DEBUG_SCALAR_FMT("0x%lx", (unsigned long)ptr); }

// =============================================================
// Buffer dump
// =============================================================

void debug_log(const char* name, const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return;

    char line[512];
    char prefix[64];

    debug_prefix(prefix, sizeof(prefix), name);

    size_t pos = snprintf(
        line,
        sizeof(line),
        "%slen=%u [",
        prefix,
        (unsigned)len
    );

    for (size_t i = 0; i < len && pos + 3 < sizeof(line); i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X", buf[i]);
        if (i + 1 < len && pos + 1 < sizeof(line)) {
            line[pos++] = ' ';
            line[pos] = '\0';
        }
    }

    if (pos + 1 < sizeof(line)) {
        line[pos++] = ']';
        line[pos] = '\0';
    }

    debug_emit(line);
}

// =============================================================
// Blink primitive
// =============================================================

static constexpr uint32_t BLINK_UNIT_MS  = 150;
static constexpr uint32_t BLINK_GAP_MS   = 150;
static constexpr uint32_t PATTERN_GAP_MS = 600;

void debug_blink(const char* pattern) {
    if (!pattern || !*pattern) return;

    pinMode(LED_BUILTIN, OUTPUT);

    for (const char* p = pattern; *p; ++p) {
        char c = *p;
        if (c < '0' || c > '9') continue;

        uint8_t units = (uint8_t)(c - '0');
        if (units == 0) continue;

        digitalWrite(LED_BUILTIN, HIGH);
        delay(units * BLINK_UNIT_MS);
        digitalWrite(LED_BUILTIN, LOW);
        delay(BLINK_GAP_MS);
    }

    delay(PATTERN_GAP_MS);
}