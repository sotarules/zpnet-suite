/*
  ZPNet Teensy Telemetry Firmware --- Slave Command Appliance + Event Bus
  Event-only edition (no imperative replies)

  Invariants:
    • Teensy emits NOTHING unless explicitly commanded.
    • Serial output occurs ONLY during EVENTS.GET (explicit drain).
    • Routine query commands enqueue data events only (no ACK noise).
    • State-changing commands enqueue ACK events via the event queue.
    • All truth leaves via the event queue.
    • Exactly one serial consumer is assumed.

  Photodiode/TDM integration notes (2025-12-28):
    The new UK photodiode path is a Tiny Device Module (TDM) that presents
    two distinct signals to the Teensy:

      1) Pin 14 --- fast digital edge suitable for interrupts (pulse counting)
      2) Pin 15 --- analog voltage that remains present while the laser is on

    The existing interrupt counter remains valuable and stays in place.

    This revision adds:
      • A dedicated analog pin (15) for photodiode amplitude / presence signal
      • PHOTODIODE.STATUS payload now includes:
          - edge_level (digital read of pin 14, best-effort snapshot)
          - edge_pulse_count Monotonic count of light-present episodes since last PHOTODIODE.CLEAR.
          - analog_raw (ADC counts from pin 15)
          - analog_v (derived voltage estimate using 3.3 V assumption)
          - millis

    Out of scope on purpose:
      • Any inference of laser state from pin 15.
        That belongs in later aggregation logic (LASER_STATUS / health layers).

  Author: The Mule + GPT
*/

#include <Arduino.h>
#include <Wire.h>
#include <malloc.h>
#include <string.h>
#include <ADC.h>

// --------------------------------------------------------------
// Version
// --------------------------------------------------------------
static const char* FW_VERSION = "teensy-telemetry-4.3.5";

// --------------------------------------------------------------
// System state
// --------------------------------------------------------------
static bool system_shutdown = false;

// --------------------------------------------------------------
// GNSS serial config
// --------------------------------------------------------------
static const unsigned long GNSSDO_BAUD = 38400;
static const size_t GNSS_LINE_MAX = 192;
static const unsigned long GNSS_SILENCE_FLUSH_MS = 50;

// --------------------------------------------------------------
// Laser pins
// --------------------------------------------------------------
static const int EN_PIN    = 20;
static const int LD_ON_PIN = 21;
static bool ldOn = false;

// --------------------------------------------------------------
// Photodiode pins (TDM: edge + analog)
// --------------------------------------------------------------
//
// Pin 14: fast digital edge (interrupt-capable). This is the pulse counter path.
// Pin 15: analog voltage. This is the continuous ``laser present'' amplitude path.
//
static const int PHOTODIODE_EDGE_PIN   = 14;
static const int PHOTODIODE_ANALOG_PIN = 15;

// --------------------------------------------------------------
// Photodiode episode latch (state-driven)
// --------------------------------------------------------------
static volatile bool photodiode_edge_seen = false;
static volatile bool photodiode_episode_latched = false;
static volatile uint32_t photodiode_episode_count = 0;

static const uint8_t PHOTODIODE_OFF_CONFIRM_SAMPLES = 5;

// Analog authority threshold (volts)
static const float PHOTODIODE_ON_THRESHOLD_V  = 0.20f;  // rising threshold
static const float PHOTODIODE_OFF_THRESHOLD_V = 0.05f;  // falling threshold

static const uint32_t PHOTODIODE_OFF_STABLE_MS = 20;

void photodiodeISR() {
  photodiode_edge_seen = true;
}

// --------------------------------------------------------------
// GNSS rolling state
// --------------------------------------------------------------
static unsigned long lastGnssByteMs = 0;

// Latest raw sentence
static char last_sentence[GNSS_LINE_MAX] = "";

// Canonical retained sentences
static char last_crw[GNSS_LINE_MAX] = "";
static char last_crx[GNSS_LINE_MAX] = "";
static char last_crz[GNSS_LINE_MAX] = "";
static char last_zda[GNSS_LINE_MAX] = "";
static char last_rmc[GNSS_LINE_MAX] = "";
static char last_gga[GNSS_LINE_MAX] = "";

// Parsed authoritative fields
static float latitude_deg  = NAN;
static float longitude_deg = NAN;

// --------------------------------------------------------------
// GNSS PPS (Teensy pin 33)
// --------------------------------------------------------------
static const int GNSS_PPS_PIN = 33;

// --------------------------------------------------------------
// GNSS VCLOCK (10 MHz) — Teensy pin 9
// --------------------------------------------------------------
static const int GNSS_VCLK_PIN = 9;

// PPS capture state (ISR writes, loop reads)
static volatile bool     pps_seen = false;
static volatile uint32_t pps_cycles = 0;
static volatile uint32_t pps_count = 0;

// Optional: allow PPS telemetry to be toggled (e.g., via command)
static bool pps_telemetry_enabled = true;

// --------------------------------------------------------------
// PPS telemetry rate limiting
// --------------------------------------------------------------
static const uint32_t PPS_EMIT_INTERVAL_MS = 10000; // 10 seconds
static uint32_t last_pps_emit_ms = 0;

// --------------------------------------------------------------
// Utilities
// --------------------------------------------------------------
static inline float cpuTempC() {
#if defined(ARDUINO_TEENSY41)
  return tempmonGetTemp();
#else
  return 0.0f;
#endif
}

static ADC* adc = new ADC();

static inline float readVrefVolts() {
  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);
  uint16_t raw = adc->adc0->analogRead(ADC_INTERNAL_SOURCE::VREFSH);
  if (raw == 0) return 0.0f;
  const float VREF_INTERNAL = 1.2f;
  const float ADC_MAX = 4095.0f;
  return VREF_INTERNAL / (raw / ADC_MAX);
}

static inline uint32_t freeHeapBytes() {
  struct mallinfo mi = mallinfo();
  return (uint32_t)mi.fordblks;
}

static inline void safeCopy(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;
  if (!src) { dst[0] = '\0'; return; }
  size_t n = strnlen(src, dst_sz - 1);
  memcpy(dst, src, n);
  dst[n] = '\0';
}

static String jsonEscape(const char* s) {
  String out;
  if (!s) return out;
  while (*s) {
    char c = *s++;
    if (c == '\\') out += "\\\\";
    else if (c == '\"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if ((uint8_t)c < 0x20) out += " ";
    else out += c;
  }
  return out;
}

static inline void appendFloatKV(
    String& b,
    const char* key,
    float value,
    int decimals = 5
) {
    char buf[48];
    snprintf(buf, sizeof(buf), "\"%s\":%.*f", key, decimals, value);
    b += buf;
}

// --------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------
static void enqueueEvent(const char* type, const String& body);

// --------------------------------------------------------------
// JSON emit helper (ONLY used during explicit queue drain)
// --------------------------------------------------------------
static inline void emitJson(const char* type, const String& body) {
  Serial.print("{\"event_type\":\"");
  Serial.print(type);
  Serial.print("\"");
  if (body.length()) {
    Serial.print(",");
    Serial.print(body);
  }
  Serial.println("}");
}

// --------------------------------------------------------------
// Immediate JSON emit helper (QUERY plane only)
// --------------------------------------------------------------
//
// Used ONLY for non-destructive CMD? queries.
// This bypasses the event queue entirely.
//
static inline void emitImmediateJson(const char* type, const String& body) {
  Serial.print("{\"type\":\"");
  Serial.print(type);
  Serial.print("\"");
  if (body.length()) {
    Serial.print(",");
    Serial.print(body);
  }
  Serial.println("}");
}

// --------------------------------------------------------------
// ACK/ERR are now EVENT-QUEUE ONLY (no direct Serial output)
// --------------------------------------------------------------
static inline void enqueueAckEvent(const char* cmd) {
  String b;
  b += "\"cmd\":\""; b += cmd; b += "\"";
  b += ",\"ok\":true";
  b += ",\"millis\":"; b += millis();
  enqueueEvent("ACK", b);
}

static inline void enqueueErrEvent(const char* cmd, const char* msg) {
  String b;
  b += "\"cmd\":\""; b += cmd; b += "\"";
  b += ",\"ok\":false";
  b += ",\"millis\":"; b += millis();
  if (msg) {
    b += ",\"error\":\""; b += jsonEscape(msg); b += "\"";
  }
  enqueueEvent("ERR", b);
}

// --------------------------------------------------------------
// Terminal quiescence (no return)
// --------------------------------------------------------------
static void enterPermanentQuiescence() {
  // Laser outputs must already be disabled by caller.
  // This function never returns.

  // Optional: detach interrupts to reduce noise
  detachInterrupt(digitalPinToInterrupt(PHOTODIODE_EDGE_PIN));

  // Enter an inert loop
  while (true) {
    delay(1000);
  }
}

// --------------------------------------------------------------
// GNSS helpers
// --------------------------------------------------------------
static inline bool startsWith(const char* s, const char* p) {
  return strncmp(s, p, strlen(p)) == 0;
}

static float nmeaLatLonToDeg(const char* ddmm, const char* hemi) {
  if (!ddmm || !hemi) return NAN;
  double v = atof(ddmm);
  if (v <= 0.0) return NAN;
  double deg = floor(v / 100.0);
  double min = v - deg * 100.0;
  double out = deg + min / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') out = -out;
  return (float)out;
}

static void parseRMC(const char* l) {
  char lat[16], latH[4], lon[16], lonH[4];
  if (sscanf(l, "%*[^,],%*[^,],A,%15[^,],%3[^,],%15[^,],%3[^,]",
             lat, latH, lon, lonH) == 4) {
    latitude_deg  = nmeaLatLonToDeg(lat, latH);
    longitude_deg = nmeaLatLonToDeg(lon, lonH);
  }
}

// --------------------------------------------------------------
// GNSS ingestion
// --------------------------------------------------------------
static char lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;

static void ingestGnssLine(const char* line) {
  if (!line || !*line) return;

  safeCopy(last_sentence, sizeof(last_sentence), line);

  if (startsWith(line, "$PERDCRW,")) safeCopy(last_crw, sizeof(last_crw), line);
  else if (startsWith(line, "$PERDCRX,")) safeCopy(last_crx, sizeof(last_crx), line);
  else if (startsWith(line, "$PERDCRZ,")) safeCopy(last_crz, sizeof(last_crz), line);
  else if (startsWith(line, "$GPZDA,"))  safeCopy(last_zda, sizeof(last_zda), line);
  else if (startsWith(line, "$GNRMC,") || startsWith(line, "$GPRMC,")) {
    safeCopy(last_rmc, sizeof(last_rmc), line);
    parseRMC(line);
  }
  else if (startsWith(line, "$GNGGA,") || startsWith(line, "$GPGGA,")) {
    safeCopy(last_gga, sizeof(last_gga), line);
  }
}

static void pollGnssSerial() {
  const unsigned long start_ms = millis();

  // Budget: process GNSS for at most this many milliseconds per loop().
  // This ensures USB CDC commands remain responsive even if GNSS is chatty.
  const unsigned long BUDGET_MS = 2;

  // Optional additional cap (belt-and-suspenders)
  const uint32_t MAX_BYTES = 512;
  uint32_t bytes = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    lastGnssByteMs = millis();  // update with real time, not a stale snapshot

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) ingestGnssLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && lineLen < GNSS_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    }

    bytes++;
    if (bytes >= MAX_BYTES) break;
    if (millis() - start_ms >= BUDGET_MS) break;
  }

  // Flush partial line only when we have been silent long enough.
  if (lineLen && (millis() - lastGnssByteMs) > GNSS_SILENCE_FLUSH_MS) {
    lineBuf[lineLen] = '\0';
    ingestGnssLine(lineBuf);
    lineLen = 0;
  }
}


// --------------------------------------------------------------
// Event Bus
// --------------------------------------------------------------
static const size_t EVT_MAX = 32;
static const size_t EVT_TYPE_MAX = 32;
static const size_t EVT_BODY_MAX = 512;

struct EventItem {
  uint32_t ms;
  char type[EVT_TYPE_MAX];
  char body[EVT_BODY_MAX];
};

static EventItem evtq[EVT_MAX];
static size_t evt_head = 0;
static size_t evt_tail = 0;
static size_t evt_count = 0;
static uint32_t evt_dropped = 0;

static void enqueueEvent(const char* type, const String& body) {
  if (evt_count >= EVT_MAX) { evt_dropped++; return; }
  EventItem& e = evtq[evt_head];
  e.ms = millis();
  safeCopy(e.type, sizeof(e.type), type);
  safeCopy(e.body, sizeof(e.body), body.c_str());
  evt_head = (evt_head + 1) % EVT_MAX;
  evt_count++;
}

static bool dequeueEvent(EventItem& out) {
  if (!evt_count) return false;
  out = evtq[evt_tail];
  evt_tail = (evt_tail + 1) % EVT_MAX;
  evt_count--;
  return true;
}

static void drainEventsNow() {
  {
    String b;
    b += "\"count\":"; b += evt_count;
    b += ",\"dropped\":"; b += evt_dropped;
    emitJson("EVENTS_BEGIN", b);
  }

  EventItem e;
  while (dequeueEvent(e)) {
    String b;
    b += "\"millis\":"; b += e.ms;
    if (e.body[0]) { b += ","; b += e.body; }
    emitJson(e.type, b);
  }

  emitJson("EVENTS_END", "\"count\":0");
}

// --------------------------------------------------------------
// Status builders
// --------------------------------------------------------------
static String buildTeensyStatusBody() {
  String b;
  b += "\"fw_version\":\""; b += FW_VERSION; b += "\"";
  b += ",\"millis\":"; b += millis();
  b += ",\"cpu_temp_c\":"; b += cpuTempC();
  b += ",\"vref_v\":"; b += readVrefVolts();
  b += ",\"free_heap_bytes\":"; b += freeHeapBytes();
  b += ",\"laser_enabled\":"; b += (ldOn ? "true" : "false");
  return b;
}

static String buildGnssStatusBody() {
  String b;
  if (last_sentence[0]) {
    b += "\"raw_sentence\":\"";
    b += jsonEscape(last_sentence);
    b += "\"";
  }
  return b;
}

static String buildGnssDataBody() {
  String b;

  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
    b += "\"latitude_deg\":"; b += latitude_deg;
    b += ",\"longitude_deg\":"; b += longitude_deg;
  }

  if (last_zda[0]) {
    if (b.length()) b += ",";
    b += "\"raw_zda\":\"";
    b += jsonEscape(last_zda);
    b += "\"";
  }

  if (last_rmc[0]) {
    if (b.length()) b += ",";
    b += "\"raw_rmc\":\"";
    b += jsonEscape(last_rmc);
    b += "\"";
  }

  return b;
}

static String buildGnssPpsBody(
    uint32_t cycles,
    uint32_t count,
    uint32_t vclk_cycles,
    int vclk_level
) {
  String b;
  b += "\"millis\":"; b += millis();
  b += ",\"pps_count\":"; b += count;
  b += ",\"cycles\":"; b += cycles;

  // VCLOCK instrumentation
  b += ",\"vclk_cycles\":"; b += vclk_cycles;
  b += ",\"vclk_level\":"; b += vclk_level;

  b += ",\"emit_interval_ms\":"; b += PPS_EMIT_INTERVAL_MS;
  return b;
}

static String buildPhotodiodeStatusBody() {
  // ------------------------------------------------------------------
  // This payload is meant to be raw and non-judgmental.
  // No laser inference occurs here.
  //
  // edge_level:
  //   A best-effort snapshot of the digital state on the edge pin.
  //   Useful for wiring sanity checks and basic liveness inspection.
  //
  // edge_pulse_count:
  //   Monotonic count since last PHOTODIODE.CLEAR.
  //
  // analog_raw:
  //   ADC counts for the continuous analog channel.
  //
  // analog_v:
  //   A derived voltage using 12-bit scaling and a 3.3 V reference.
  //   This is a convenience field; downstream can prefer analog_raw.
  // ------------------------------------------------------------------
  int edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  // Latch the ISR-updated counter atomically.
  uint32_t count;
  noInterrupts();
  count = photodiode_episode_count;
  interrupts();

  // Read analog channel.
  // Note: Teensy analogRead returns 0..4095 when resolution is 12 bits.
  // The default resolution on Teensy is often 10 bits unless changed.
  // For determinism, enforce 12-bit resolution here for the read.
  analogReadResolution(12);
  int analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);

  // Convert to volts using a simple 3.3 V full-scale assumption.
  // This is not a promise of absolute accuracy, only a friendly scale.
  const float ADC_FS_COUNTS = 4095.0f;
  const float ADC_FS_VOLTS  = 3.3f;
  float analog_v = ((float)analog_raw / ADC_FS_COUNTS) * ADC_FS_VOLTS;

  String b;
  b += "\"edge_level\":"; b += edge_level;
  b += ",\"edge_pulse_count\":"; b += count;
  b += ",\"analog_raw\":"; b += analog_raw;
  b += ",\"analog_v\":";
  {
    // Keep float formatting stable and compact.
    // 5 decimals is consistent with other telemetry float fields.
    char buf[32];
    snprintf(buf, sizeof(buf), "%.5f", analog_v);
    b += buf;
  }
  b += ",\"millis\":"; b += millis();
  return b;
}

static String buildPhotodiodeCountBody() {
  uint32_t count;
  noInterrupts();
  count = photodiode_episode_count;
  interrupts();

  String b;
  b += "\"count\":"; b += count;
  b += ",\"millis\":"; b += millis();
  return b;
}

// --------------------------------------------------------------
// IMON averaging helper
// --------------------------------------------------------------
//
// This helper historically sampled PHOTODIODE_PIN for long-duration
// averaging. With the TDM split-pin design, the intended analog path
// is pin 15. The averaging helper is kept for the existing LASER.VOLTAGES
// command, but migrated to the dedicated analog pin.
//
// This avoids overloading the edge pin with analog sampling semantics.
//
static float averageImonSeconds(uint32_t seconds) {
  const uint32_t duration_ms = seconds * 1000UL;
  uint32_t start = millis();
  uint64_t sum = 0;
  uint32_t count = 0;

  // Force a stable resolution for averaging.
  analogReadResolution(12);

  // Use default Arduino analogRead (Teensy ADC).
  // Sampling cadence is intentionally modest; the intent is a 10s mean.
  while (millis() - start < duration_ms) {
    sum += (uint16_t)analogRead(PHOTODIODE_ANALOG_PIN);
    count++;
    delay(1);  // ~1 kHz sample cadence (adequate for long averaging)
  }

  if (count == 0) return 0.0f;

  float adc_avg = (float)sum / (float)count;
  return (adc_avg / 4095.0f) * 3.3f;
}

// --------------------------------------------------------------
// Photodiode episode state machine (with temporal off-stability)
// --------------------------------------------------------------
static void updatePhotodiodeEpisodeLatch() {
  // Sample analog channel (authoritative)
  analogReadResolution(12);
  int analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);
  float analog_v = (analog_raw / 4095.0f) * 3.3f;

  // Hysteresis thresholds
  static const float PHOTODIODE_ON_THRESHOLD_V  = 0.20f;
  static const float PHOTODIODE_OFF_THRESHOLD_V = 0.05f;

  // Temporal off-stability
  static uint32_t photodiode_dark_since_ms = 0;
  static const uint32_t PHOTODIODE_OFF_STABLE_MS = 20;

  static bool light_present = false;
  uint32_t now = millis();

  // --- Hysteresis-based light state ---
  if (!light_present) {
    if (analog_v >= PHOTODIODE_ON_THRESHOLD_V) {
      light_present = true;
    }
  } else {
    if (analog_v <= PHOTODIODE_OFF_THRESHOLD_V) {
      light_present = false;
    }
  }

  noInterrupts();

  // --- Dark tracking / latch reset ---
  if (!light_present) {
    if (photodiode_dark_since_ms == 0) {
      photodiode_dark_since_ms = now;
    }

    if (now - photodiode_dark_since_ms >= PHOTODIODE_OFF_STABLE_MS) {
      photodiode_episode_latched = false;
    }
  } else {
    photodiode_dark_since_ms = 0;

    // --- Episode detection ---
    if (photodiode_edge_seen && !photodiode_episode_latched) {
      photodiode_episode_latched = true;
      photodiode_episode_count++;
    }
  }

  photodiode_edge_seen = false;
  interrupts();
}


// --------------------------------------------------------------
// Command parsing and execution (event-only)
// --------------------------------------------------------------
static bool extractCmd(const char* line, char* out, size_t out_sz) {
  const char* p = strstr(line, "\"cmd\"");
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ') p++;
  if (*p != '\"') return false;
  p++;
  const char* q = strchr(p, '\"');
  if (!q) return false;
  size_t n = q - p;
  if (n >= out_sz) n = out_sz - 1;
  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}

// Command semantics:
//   CMD        → enqueue durable event
//   CMD?       → immediate reply, no queue interaction
//   TERMINAL   → irreversible state change, no return (e.g. SYSTEM.SHUTDOWN)
static void execCommand(const char* line) {
  char cmd[64];
  if (!extractCmd(line, cmd, sizeof(cmd))) {
    enqueueErrEvent("UNKNOWN", "missing cmd");
    return;
  }

  if (strcmp(cmd, "TEENSY.STATUS") == 0) {
    enqueueEvent("TEENSY_STATUS", buildTeensyStatusBody());
    return;
  }

  if (strcmp(cmd, "GNSS.STATUS") == 0) {
    enqueueEvent("GNSS_STATUS", buildGnssStatusBody());
    return;
  }

  if (strcmp(cmd, "GNSS.DATA") == 0) {
    enqueueEvent("GNSS_DATA", buildGnssDataBody());
    return;
  }

  // --------------------------------------------------------------
  // QUERY COMMANDS (NON-DESTRUCTIVE, IMMEDIATE REPLY)
  // --------------------------------------------------------------
  if (strcmp(cmd, "PHOTODIODE.STATUS?") == 0) {
    emitImmediateJson(
      "PHOTODIODE_STATUS",
      buildPhotodiodeStatusBody()
    );
    return;
  }

  if (strcmp(cmd, "PHOTODIODE.STATUS") == 0) {
    // PHOTODIODE_STATUS is now richer:
    //   edge_level, edge_pulse_count, analog_raw, analog_v, millis
    enqueueEvent("PHOTODIODE_STATUS", buildPhotodiodeStatusBody());
    return;
  }

  if (strcmp(cmd, "PHOTODIODE.COUNT") == 0) {
    enqueueEvent("PHOTODIODE_COUNT", buildPhotodiodeCountBody());
    return;
  }

  if (strcmp(cmd, "PHOTODIODE.CLEAR") == 0) {
    noInterrupts();
    photodiode_episode_count = 0;
    photodiode_episode_latched = false;
    photodiode_edge_seen = false;
    interrupts();
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.ON") == 0) {
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(LD_ON_PIN, HIGH);
    ldOn = true;

    enqueueEvent("LASER_STATE", "\"enabled\":true");
    enqueueAckEvent(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.OFF") == 0) {
    digitalWrite(LD_ON_PIN, LOW);
    ldOn = false;

    enqueueEvent("LASER_STATE", "\"enabled\":false");
    enqueueAckEvent(cmd);
    return;
  }

  // --------------------------------------------------------------
  // SYSTEM.SHUTDOWN (terminal command)
  // --------------------------------------------------------------
  if (strcmp(cmd, "SYSTEM.SHUTDOWN") == 0) {
    digitalWrite(LD_ON_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
    ldOn = false;

    system_shutdown = true;
    enqueueAckEvent(cmd);
    enterPermanentQuiescence();
  }

  // ----------------------------------------------------------
  // LASER.VOLTAGES
  // ----------------------------------------------------------
  //
  // Historical diagnostic:
  //   Measure long-duration averaged analog voltage with laser off/on.
  //
  // With the TDM split-pin design, the averaging now samples the
  // dedicated analog channel (pin 15).
  //
  if (strcmp(cmd, "LASER.VOLTAGES") == 0) {
    // (1) Laser OFF
    digitalWrite(LD_ON_PIN, LOW);
    ldOn = false;
    delay(50);

    float off_v = averageImonSeconds(10);

    // (2) Laser ON
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(LD_ON_PIN, HIGH);
    ldOn = true;
    delay(50);

    float on_v = averageImonSeconds(10);

    // (3) Laser OFF again (leave EN as-is; only turn off LD_ON)
    digitalWrite(LD_ON_PIN, LOW);
    ldOn = false;

    String b;
    appendFloatKV(b, "off_v", off_v, 5);
    b += ",";
    appendFloatKV(b, "on_v", on_v, 5);
    b += ",";
    b += "\"millis\":";
    b += millis();

    enqueueEvent("LASER_VOLTAGES", b);
    enqueueAckEvent(cmd); // state-changing
    return;
  }

  if (strcmp(cmd, "EVENTS.GET") == 0) {
    drainEventsNow();
    return;
  }

  enqueueErrEvent(cmd, "unknown command");
}

// --------------------------------------------------------------
// Setup / Loop
// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial1.begin(GNSSDO_BAUD);

  // ------------------------------------------------------------
  // Enable ARM DWT cycle counter (Teensy 4.1)
  // ------------------------------------------------------------
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  ARM_DWT_CYCCNT = 0;

  // ------------------------------------------------------------
  // GNSS PPS pin setup
  // ------------------------------------------------------------
  pinMode(GNSS_PPS_PIN, INPUT);

  // --------------------------------------------------------------
  // GNSS VCLOCK (10 MHz) — Teensy pin 9
  // --------------------------------------------------------------
  static const int GNSS_VCLK_PIN = 9;

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    []() {
      pps_cycles = ARM_DWT_CYCCNT;
      pps_count++;
      pps_seen = true;
    },
    RISING
  );

  pinMode(EN_PIN, OUTPUT);
  pinMode(LD_ON_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(LD_ON_PIN, LOW);

  // ------------------------------------------------------------
  // Photodiode pin configuration
  // ------------------------------------------------------------
  //
  // Edge pin (14):
  //   Treated as a digital input for interrupt pulse counting.
  //   No internal pullups enabled. The TDM should drive this line.
  //
  // Analog pin (15):
  //   Treated as ADC input. No pullups. The TDM should drive a
  //   stable analog level during laser-on.
  //
  pinMode(PHOTODIODE_EDGE_PIN, INPUT);
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT);

  // Attach interrupt to edge pin. Mode CHANGE counts both edges,
  // maximizing visibility into pulse activity without interpretation.
  attachInterrupt(
    digitalPinToInterrupt(PHOTODIODE_EDGE_PIN),
    photodiodeISR,
    CHANGE
  );

  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

void loop() {
  // ------------------------------------------------------------
  // PRIORITY: USB CDC commands (queries, control)
  // ------------------------------------------------------------
  static char buf[256];
  static size_t len = 0;

  if (system_shutdown) {
    enterPermanentQuiescence();
  }

  // Maintain photodiode episode latch (state-driven)
  updatePhotodiodeEpisodeLatch();

  // ------------------------------------------------------------
  // GNSS PPS capture → rate-limited GNSS_PPS event
  // ------------------------------------------------------------
  if (pps_telemetry_enabled) {
    bool seen = false;
    uint32_t cyc = 0;
    uint32_t cnt = 0;

    noInterrupts();
    if (pps_seen) {
      seen = true;
      cyc = pps_cycles;
      cnt = pps_count;
      pps_seen = false;
    }
    interrupts();

    if (seen) {
      uint32_t now_ms = millis();

      // Emit at most once every PPS_EMIT_INTERVAL_MS
      if (now_ms - last_pps_emit_ms >= PPS_EMIT_INTERVAL_MS) {
        last_pps_emit_ms = now_ms;

        // Snapshot VCLOCK observables
        uint32_t vclk_cyc = ARM_DWT_CYCCNT;
        int vclk_lvl = digitalRead(GNSS_VCLK_PIN);

        // Emit event
        enqueueEvent(
          "GNSS_PPS",
          buildGnssPpsBody(cyc, cnt, vclk_cyc, vclk_lvl)
        );

        // Reset PPS counter for next window
        noInterrupts();
        pps_count = 0;
        interrupts();
      }
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      if (len) execCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  // ------------------------------------------------------------
  // BACKGROUND: GNSS ingestion (opportunistic)
  // ------------------------------------------------------------
  pollGnssSerial();
}

