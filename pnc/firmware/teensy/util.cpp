#include "integer_only.h"
#include "double.h"
#include "util.h"
#include "debug.h"
#include "payload.h"
#include <malloc.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>

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
Double cpuTempC() {
#if defined(ARDUINO_TEENSY41)
  // Read the same OTP calibration and sensor code as the Teensy core, but
  // perform the conversion here without entering its FP implementation.
  const uint32_t calibration = HW_OCOTP_ANA1;
  const int32_t hot_c = calibration & 0xffU;
  const int32_t hot_count = (calibration >> 8) & 0xfffU;
  const int32_t room_count = (calibration >> 20) & 0xfffU;
  while (!(TEMPMON_TEMPSENSE0 & 4U)) {}
  const int32_t measured = (TEMPMON_TEMPSENSE0 >> 8) & 0xfffU;
  return Double(hot_c) - Double(measured - hot_count) *
      Double(hot_c - 25) / Double(room_count - hot_count);
#else
  return 0_D;
#endif
}

// --------------------------------------------------------------
// Internal voltage reference
// --------------------------------------------------------------
Double readVrefVolts() {
#if defined(ARDUINO_TEENSY41)
  static ADC* adc = new ADC();

  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);

  uint16_t raw = adc->adc0->analogRead(ADC_INTERNAL_SOURCE::VREFSH);
  if (raw == 0) return 0_D;

  const Double VREF_INTERNAL = 1.2_D;
  const Double ADC_MAX = 4095_D;

  return VREF_INTERNAL / (raw / ADC_MAX);
#else
  return 0_D;
#endif
}

// --------------------------------------------------------------
// Reserve Teensy CrashReport's retained RAM from heap growth
// --------------------------------------------------------------
extern "C" {

extern char *__brkval;
extern unsigned long _heap_end;

void *_sbrk(int incr) {
  char *prev = __brkval;
  char *limit = reinterpret_cast<char *>(
      reinterpret_cast<uintptr_t>(&_heap_end) - uintptr_t{128});

  if (incr != 0) {
    if (prev + incr > limit) {
      errno = ENOMEM;
      return reinterpret_cast<void *>(-1);
    }
    __brkval = prev + incr;
  }

  return prev;
}

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
    void* p = payload_shared_heap_malloc(mid);
    if (p) {
      if (!payload_shared_heap_free(p)) __builtin_trap();
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

