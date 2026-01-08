#include "util.h"

#include <malloc.h>
#include <string.h>

#if defined(ARDUINO_TEENSY41)
#include <ADC.h>
#endif

// --------------------------------------------------------------
// Safe string copy
// --------------------------------------------------------------
void safeCopy(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;

  if (!src) {
    dst[0] = '\0';
    return;
  }

  size_t n = strnlen(src, dst_sz - 1);
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
// Float key/value append
// --------------------------------------------------------------
void appendFloatKV(
  String& b,
  const char* key,
  float value,
  int decimals
) {
  char buf[48];
  snprintf(buf, sizeof(buf), "\"%s\":%.*f", key, decimals, value);
  b += buf;
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
