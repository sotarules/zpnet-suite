#include "util.h"
#include "debug.h"
#include <malloc.h>
#include <string.h>
#include <math.h>

#if defined(ARDUINO_TEENSY41)
#include <ADC.h>
#endif

// ============================================================================
// Fixed-decimal publication boundary
// ============================================================================

static constexpr uint64_t FIXED_DECIMAL_MAX_WHOLE =
    9000000000000000000ULL;

fixed_decimal_t toFixedDecimal(double value, int decimal_places) {
  fixed_decimal_t out;
  out.whole = 0ULL;
  out.fractional = 0ULL;
  out.source_bits = 0ULL;
  memcpy(&out.source_bits, &value, sizeof(value));
  out.decimal_places = 0U;
  out.negative = 0U;
  out.status = fixed_decimal_status_t::VALID;

  if (decimal_places < 0) decimal_places = 0;
  if (decimal_places > (int)FIXED_DECIMAL_MAX_PLACES) {
    decimal_places = (int)FIXED_DECIMAL_MAX_PLACES;
  }
  out.decimal_places = (uint8_t)decimal_places;

  if (value != value) {
    out.status = fixed_decimal_status_t::NAN_VALUE;
    return out;
  }
  if (isinf(value)) {
    out.status = signbit(value)
        ? fixed_decimal_status_t::NEGATIVE_INFINITY
        : fixed_decimal_status_t::POSITIVE_INFINITY;
    return out;
  }

  const bool negative = value < 0.0;
  const double magnitude = negative ? -value : value;
  if (magnitude > (double)FIXED_DECIMAL_MAX_WHOLE) {
    out.status = fixed_decimal_status_t::OUT_OF_RANGE;
    return out;
  }

  uint64_t whole = (uint64_t)magnitude;
  double fraction = magnitude - (double)whole;
  if (fraction < 0.0) fraction = 0.0;
  if (fraction >= 1.0) fraction = 0.9999999999999999;

  // Every digit that will be read is written by the loop.  Deliberately avoid
  // aggregate zero-initialization so this conversion helper cannot grow a
  // compiler-generated memset of its own.
  uint8_t fractional_digits[FIXED_DECIMAL_MAX_PLACES];
  uint8_t guard_digit = 0U;

  for (uint32_t i = 0U; i <= out.decimal_places; ++i) {
    const double scaled = fraction * 10.0;
    int digit = (int)scaled;
    if (digit < 0) digit = 0;
    if (digit > 9) digit = 9;

    fraction = scaled - (double)digit;
    if (fraction < 0.0) fraction = 0.0;
    if (fraction > 1.0) fraction = 1.0;

    if (i < out.decimal_places) {
      fractional_digits[i] = (uint8_t)digit;
    } else {
      guard_digit = (uint8_t)digit;
    }
  }

  if (guard_digit >= 5U) {
    bool carry = true;
    for (uint32_t i = out.decimal_places; i != 0U && carry; --i) {
      uint8_t& digit = fractional_digits[i - 1U];
      if (digit == 9U) {
        digit = 0U;
      } else {
        ++digit;
        carry = false;
      }
    }
    if (carry) {
      ++whole;
    }
  }

  if (whole > FIXED_DECIMAL_MAX_WHOLE) {
    out.status = fixed_decimal_status_t::OUT_OF_RANGE;
    return out;
  }

  uint64_t fractional = 0ULL;
  bool rounded_nonzero = whole != 0ULL;
  for (uint32_t i = 0U; i < out.decimal_places; ++i) {
    fractional = fractional * 10ULL + fractional_digits[i];
    if (fractional_digits[i] != 0U) rounded_nonzero = true;
  }

  out.whole = whole;
  out.fractional = fractional;
  out.negative = (negative && rounded_nonzero) ? 1U : 0U;
  return out;
}

const char* fixedDecimalStatusName(fixed_decimal_status_t status) {
  switch (status) {
    case fixed_decimal_status_t::VALID: return "VALID";
    case fixed_decimal_status_t::NAN_VALUE: return "NAN";
    case fixed_decimal_status_t::POSITIVE_INFINITY: return "POSITIVE_INFINITY";
    case fixed_decimal_status_t::NEGATIVE_INFINITY: return "NEGATIVE_INFINITY";
    case fixed_decimal_status_t::OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------------------
// Safe string copy
// --------------------------------------------------------------
void safeCopy(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;

  if (!src) {
    dst[0] = '\0';
    return;
  }

  size_t n = 0;
  while (n < dst_sz - 1 && src[n] != '\0') {
    n++;
  }

  memcpy(dst, src, n);
  dst[n] = '\0';
}

// --------------------------------------------------------------
// JSON escape helper
// --------------------------------------------------------------
String jsonEscape(const char* s) {
  String out;
  if (!s) return out;

  while (*s) {
    char c = *s++;

    if (c == '\\')       out += "\\\\";
    else if (c == '\"')  out += "\\\"";
    else if (c == '\n')  out += "\\n";
    else if (c == '\r')  out += "\\r";
    else if ((uint8_t)c < 0x20)
                         out += " ";
    else                 out += c;
  }

  return out;
}

// --------------------------------------------------------------
// CPU temperature
// --------------------------------------------------------------
float cpuTempC() {
#if defined(ARDUINO_TEENSY41)
  return tempmonGetTemp();
#else
  return 0.0f;
#endif
}

// --------------------------------------------------------------
// Internal voltage reference
// --------------------------------------------------------------
float readVrefVolts() {
#if defined(ARDUINO_TEENSY41)
  static ADC* adc = new ADC();

  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);

  uint16_t raw = adc->adc0->analogRead(ADC_INTERNAL_SOURCE::VREFSH);
  if (raw == 0) return 0.0f;

  const float VREF_INTERNAL = 1.2f;
  const float ADC_MAX = 4095.0f;

  return VREF_INTERNAL / (raw / ADC_MAX);
#else
  return 0.0f;
#endif
}

// --------------------------------------------------------------
// Heap availability
// --------------------------------------------------------------
uint32_t freeHeapBytes() {
  struct mallinfo mi = mallinfo();
  return (uint32_t)mi.fordblks;
}

uint32_t maxAllocBytes() {
  uint32_t lo = 0, hi = 256 * 1024; // Teensy 4.1 has plenty; clamp as needed
  while (lo + 1 < hi) {
    uint32_t mid = (lo + hi) / 2;
    void* p = malloc(mid);
    if (p) { free(p); lo = mid; }
    else { hi = mid; }
  }
  return lo;
}

