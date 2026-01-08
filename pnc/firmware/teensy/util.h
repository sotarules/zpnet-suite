#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Generic utility helpers
// --------------------------------------------------------------
//
// These functions have no domain meaning.
// They exist to support other modules cleanly.
//

// Safe bounded string copy (always null-terminated)
void safeCopy(char* dst, size_t dst_sz, const char* src);

// Escape a C string for JSON inclusion
String jsonEscape(const char* s);

// Append a float key/value pair to a JSON body fragment
void appendFloatKV(
  String& b,
  const char* key,
  float value,
  int decimals = 5
);

// CPU temperature in Celsius (best-effort)
float cpuTempC();

// Internal voltage reference estimate (volts)
float readVrefVolts();

// Free heap memory (bytes)
uint32_t freeHeapBytes();
