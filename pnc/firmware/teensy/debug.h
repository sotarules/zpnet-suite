#pragma once

#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

void debug_init(void);
void debug_beacon(void);

// -----------------------------------------------------------------------------
// Core logging entry points
// -----------------------------------------------------------------------------

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
// Buffer / blob logging
// -----------------------------------------------------------------------------

void debug_log(const char* name, const uint8_t* buf, size_t len);

// Blink a visible debug pattern on LED_BUILTIN.
// Pattern digits define blink duration units.
// Example: "111511"
void debug_blink(const char* pattern);

// -----------------------------------------------------------------------------
// Optional convenience macro (safe, minimal)
// -----------------------------------------------------------------------------

#define DEBUG(name, value) debug_log((name), (value))
