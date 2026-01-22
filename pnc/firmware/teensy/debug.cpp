#include "debug.h"
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static constexpr uint32_t DEBUG_BAUD = 115200;
static constexpr uint32_t DEBUG_ENUM_TIMEOUT_MS = 20000;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static inline bool debug_ready() {
    return SerialUSB1;
}

static void debug_prefix(const char* name) {
    SerialUSB1.print("[");
    SerialUSB1.print(millis());
    SerialUSB1.print("] ");
    if (name && *name) {
        SerialUSB1.print(name);
        SerialUSB1.print(": ");
    }
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void debug_init(void) {

    SerialUSB1.begin(DEBUG_BAUD);

    uint32_t start = millis();
    while (!SerialUSB1 && (millis() - start) < DEBUG_ENUM_TIMEOUT_MS) {
        // wait briefly for host enumeration
    }

    if (SerialUSB1) {
        SerialUSB1.println();
        SerialUSB1.println("=== ZPNet USB Debug Online ===");
        SerialUSB1.flush();
    }
}

// -----------------------------------------------------------------------------
// Core string logger
// -----------------------------------------------------------------------------

void debug_log(const char* name, const char* msg) {

    if (!debug_ready()) return;

    debug_prefix(name);
    if (msg) {
        SerialUSB1.print(msg);
    }
    SerialUSB1.println();
    SerialUSB1.flush();
}

// -----------------------------------------------------------------------------
// Scalar overloads
// -----------------------------------------------------------------------------

void debug_log(const char* name, int value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value);
    SerialUSB1.println();
}

void debug_log(const char* name, unsigned int value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value);
    SerialUSB1.println();
}

void debug_log(const char* name, int32_t value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value);
    SerialUSB1.println();
}

void debug_log(const char* name, uint32_t value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value);
    SerialUSB1.println();
}

void debug_log(const char* name, int64_t value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print((long long)value);
    SerialUSB1.println();
}

void debug_log(const char* name, uint64_t value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print((unsigned long long)value);
    SerialUSB1.println();
}

void debug_log(const char* name, float value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value, 6);
    SerialUSB1.println();
}

void debug_log(const char* name, double value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value, 9);
    SerialUSB1.println();
}

void debug_log(const char* name, bool value) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print(value ? "true" : "false");
    SerialUSB1.println();
}

void debug_log(const char* name, const void* ptr) {
    if (!debug_ready()) return;
    debug_prefix(name);
    SerialUSB1.print("0x");
    SerialUSB1.print((uintptr_t)ptr, HEX);
    SerialUSB1.println();
}

// -----------------------------------------------------------------------------
// Buffer logger (hex dump, compact)
// -----------------------------------------------------------------------------

void debug_log(const char* name, const uint8_t* buf, size_t len) {

    if (!debug_ready()) return;

    debug_prefix(name);
    SerialUSB1.print("len=");
    SerialUSB1.print(len);
    SerialUSB1.print(" [");

    for (size_t i = 0; i < len; i++) {
        if (i) SerialUSB1.print(' ');
        if (buf[i] < 16) SerialUSB1.print('0');
        SerialUSB1.print(buf[i], HEX);
    }

    SerialUSB1.print("]");
    SerialUSB1.println();
}
