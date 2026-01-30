#include "debug.h"
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

#include <usb_rawhid.h>
#include "timepop.h"

// -----------------------------------------------------------------------------
// RawHID Debug Transport (framed, proprietary, transport-independent)
// -----------------------------------------------------------------------------

static constexpr uint8_t  DEBUG_MARKER    = 0xD0;
static constexpr size_t   HID_PACKET_SIZE = 64;

// Framing literals (authoritative)
static const char STX_PREFIX[] = "<STX=";
static const char ETX_SEQ[]    = "<ETX>";

// -----------------------------------------------------------------------------
// Low-level framed send
// -----------------------------------------------------------------------------
//
// Sends:
//   [DEBUG_MARKER]<STX=n>payload<ETX>
//
// Fragmented into 64-byte RawHID packets.
// Padding is physical only (zeros), never semantic.
//

// -----------------------------------------------------------------------------
// Low-level framed send
// -----------------------------------------------------------------------------
//
// Sends:
//   [DEBUG_MARKER]<STX=n>payload<ETX>
//
// Fragmented into 64-byte RawHID packets.
// Padding is physical only (zeros), never semantic.
//
void debug_send_framed(const char* payload, size_t payload_len) {

    if (!payload || payload_len == 0) return;

    // ---------------------------------------------------------
    // Build header: "<STX=n>"
    // ---------------------------------------------------------
    char header[12];   // exact fit for <STX=10240>
    int header_len = snprintf(
        header,
        sizeof(header),
        "%s%u>",
        STX_PREFIX,
        (unsigned)payload_len
    );

    // Hard invariants
    if (header_len <= 0 || header_len >= (int)sizeof(header)) {
        return;
    }

    // ---------------------------------------------------------
    // Compute total stream length
    //   [traffic][header][payload][ETX]
    // ---------------------------------------------------------
    const size_t etx_len = sizeof(ETX_SEQ) - 1;  // exclude '\0'
    const size_t total   = 1 + (size_t)header_len + payload_len + etx_len;

    // ---------------------------------------------------------
    // Build contiguous stream (single pass)
    // ---------------------------------------------------------
    uint8_t stream[total];
    size_t off = 0;

    // Traffic byte
    stream[off++] = DEBUG_MARKER;

    // Header
    memcpy(stream + off, header, (size_t)header_len);
    off += (size_t)header_len;

    // Payload
    memcpy(stream + off, payload, payload_len);
    off += payload_len;

    // ETX
    memcpy(stream + off, ETX_SEQ, etx_len);
    off += etx_len;

    // Final sanity check (debug invariant)
    if (off != total) return;

    // ---------------------------------------------------------
    // Fragment into HID packets
    // ---------------------------------------------------------
    uint8_t pkt[HID_PACKET_SIZE];
    size_t pos = 0;

    while (pos < total) {

        size_t n = total - pos;
        if (n > HID_PACKET_SIZE) n = HID_PACKET_SIZE;

        memset(pkt, 0, HID_PACKET_SIZE);
        memcpy(pkt, stream + pos, n);

        RawHID.send(pkt, 0);
        pos += n;
    }
}

// -----------------------------------------------------------------------------
// Prefix helper
// -----------------------------------------------------------------------------

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
// Core string logger
// -----------------------------------------------------------------------------

void debug_log(const char* name, const char* msg) {

    if (!msg) return;

    char line[512];
    char prefix[64];

    debug_prefix(prefix, sizeof(prefix), name);
    snprintf(line, sizeof(line), "%s%s", prefix, msg);

    debug_send_framed(line, strlen(line));
}

void debug_log(const char* name, const String& value) {
    debug_log(name, value.c_str());
}

// -----------------------------------------------------------------------------
// Scalar overloads
// -----------------------------------------------------------------------------

#define DEBUG_SCALAR_FMT(fmt, value)                    \
    do {                                                \
        char line[256];                                 \
        char prefix[64];                                \
        debug_prefix(prefix, sizeof(prefix), name);     \
        snprintf(line, sizeof(line), "%s" fmt,          \
                 prefix, value);                        \
        debug_send_framed(line, strlen(line));          \
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
// Buffer logger (hex dump)
// -----------------------------------------------------------------------------

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

    debug_send_framed(line, strlen(line));
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