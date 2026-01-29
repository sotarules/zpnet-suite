#include "debug.h"
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

#include <usb_rawhid.h>
#include "timepop.h"

// -----------------------------------------------------------------------------
// RawHID Debug Transport (software-multiplexed)
// -----------------------------------------------------------------------------

static constexpr uint8_t  DEBUG_MARKER    = 0xD0;
static constexpr size_t   HID_PACKET_SIZE = 64;
static constexpr size_t   HID_PAYLOAD_SZ  = HID_PACKET_SIZE - 1;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static inline void debug_send(const char* msg) {

    // Debug is best-effort and non-semantic
    if (!msg) return;

    const size_t msg_len = strlen(msg);
    if (msg_len == 0) return;

    size_t offset = 0;
    bool first_packet = true;

    while (offset < msg_len) {

        // One raw HID packet (always zero-padded)
        uint8_t pkt[HID_PACKET_SIZE];
        memset(pkt, 0, sizeof(pkt));

        size_t payload_offset = 0;

        // ---------------------------------------------------------
        // First packet only: emit traffic marker
        // ---------------------------------------------------------
        if (first_packet) {
            pkt[0] = DEBUG_MARKER;
            payload_offset = 1;   // payload starts after traffic byte
            first_packet = false;
        }

        // ---------------------------------------------------------
        // Determine how much message data fits in this packet
        // ---------------------------------------------------------
        size_t available = HID_PACKET_SIZE - payload_offset;
        size_t chunk = msg_len - offset;

        if (chunk > available) {
            chunk = available;
        }

        // ---------------------------------------------------------
        // Copy raw debug bytes into packet
        // ---------------------------------------------------------
        memcpy(pkt + payload_offset, msg + offset, chunk);

        // ---------------------------------------------------------
        // Physical egress (no framing, no retries, no recovery)
        // ---------------------------------------------------------
        RawHID.send(pkt, 0);

        offset += chunk;
    }
}

static void debug_prefix(char* out, size_t out_sz, const char* name) {
    if (!out || out_sz == 0) return;

    unsigned long t = millis();

    if (name && *name) {
        snprintf(out, out_sz, "[%lu] %s: ", t, name);
    } else {
        snprintf(out, out_sz, "[%lu] ", t);
    }
}

// -----------------------------------------------------------------------------
// Periodic beacon
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void debug_init(void) {
    debug_log("init", "=== ZPNet HID Debug Online ===");
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

// -----------------------------------------------------------------------------
// Core string logger (newline-framed)
// -----------------------------------------------------------------------------

void debug_log(const char* name, const char* msg) {
    char line[512];
    char prefix[64];

    debug_prefix(prefix, sizeof(prefix), name);

    if (msg) {
        snprintf(line, sizeof(line), "%s%s", prefix, msg);
    } else {
        snprintf(line, sizeof(line), "%s", prefix);
    }

    debug_send(line);
}

void debug_log(const char* name, const String& value) {
    debug_log(name, value.c_str());
}

// -----------------------------------------------------------------------------
// Scalar overloads (newline-framed)
// -----------------------------------------------------------------------------

#define DEBUG_SCALAR_FMT(fmt, value)                    \
    do {                                                \
        char line[256];                                 \
        char prefix[64];                                \
        debug_prefix(prefix, sizeof(prefix), name);     \
        snprintf(line, sizeof(line), "%s" fmt,          \
                 prefix, value);                        \
        debug_send(line);                               \
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

// -----------------------------------------------------------------------------
// Buffer logger (newline-framed hex dump)
// -----------------------------------------------------------------------------

void debug_log(const char* name, const uint8_t* buf, size_t len) {
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

    debug_send(line);
}

// -----------------------------------------------------------------------------
// Visible LED debug (unchanged)
// -----------------------------------------------------------------------------

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
