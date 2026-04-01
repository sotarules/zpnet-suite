#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// Forward declaration to avoid circular include
class Payload;

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

void debug_init(void);

// -----------------------------------------------------------------------------
// Core logging entry points
// -----------------------------------------------------------------------------

void debug_log(const char* name, const String& value);
void debug_log(const char* name, const char* msg);

// -----------------------------------------------------------------------------
// Overloads for common scalar types
// -----------------------------------------------------------------------------

void debug_log(const char* name, int value);
void debug_log(const char* name, unsigned int value);
void debug_log(const char* name, int32_t value);
void debug_log(const char* name, uint32_t value);
void debug_log(const char* name, int64_t value);
void debug_log(const char* name, uint64_t value);
void debug_log(const char* name, float value);
void debug_log(const char* name, double value);
void debug_log(const char* name, bool value);
void debug_log(const char* name, const void* ptr);

// -----------------------------------------------------------------------------
// Buffer / structured logging
// -----------------------------------------------------------------------------

void debug_log(const char* name, const uint8_t* buf, size_t len);
void debug_log(const char* name, const Payload& p);

// -----------------------------------------------------------------------------
// Debugging the debugger
// -----------------------------------------------------------------------------

void debug_send_framed(const char* payload, size_t payload_len);

// Blink a visible debug pattern on LED_BUILTIN.
// Pattern digits define blink duration units.
// Example: "111511"
void debug_blink(const char* pattern);

void debug_sleep_ms(uint32_t ms);

// -----------------------------------------------------------------------------
// Optional convenience macro
// -----------------------------------------------------------------------------

#define DEBUG(name, value) debug_log((name), (value))