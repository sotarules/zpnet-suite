/*
  ZPNet Teensy Telemetry Firmware — Slave Command Appliance + Event Bus

  Invariants:
    • Teensy emits NOTHING unless explicitly commanded.
    • ACTION commands return ACK/ERR only; truth is delivered via EventBus.
    • QUERY commands (with ?) return immediate snapshots.
    • Routine status commands enqueue events and return ACK.
    • Device ingestion is eager; queries are cheap; events are rare.

  Author: The Mule + GPT
*/

#include <Arduino.h>
#include <Wire.h>
#include <malloc.h>
#include <string.h>
#include <ADC.h>

// --------------------------------------------------------------
// Version (identity only; surfaced via TEENSY_STATUS)
// --------------------------------------------------------------
static const char* FW_VERSION = "teensy-telemetry-4.2.0";

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
  // Configure ADC0 (Teensy 4.x)
  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);

  // Read internal 1.2V reference (VREF Sense High)
  uint16_t raw = adc->adc0->analogRead(ADC_INTERNAL_SOURCE::VREFSH);

  if (raw == 0) {
    return 0.0f;
  }

  const float VREF_INTERNAL = 1.2f;   // volts (nominal)
  const float ADC_MAX = 4095.0f;       // 12-bit ADC

  float vref = VREF_INTERNAL / (raw / ADC_MAX);
  return vref;
}

static inline uint32_t freeHeapBytes() {
  struct mallinfo mi = mallinfo();
  return (uint32_t)mi.fordblks;
}

static inline void safeCopy(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;
  if (!src) {
    dst[0] = '\0';
    return;
  }
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

// --------------------------------------------------------------
// JSON emit helpers
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

static inline void emitAck(const char* cmd) {
  String b;
  b += "\"cmd\":\""; b += cmd; b += "\"";
  b += ",\"ok\":true";
  b += ",\"millis\":"; b += millis();
  emitJson("ACK", b);
}

static inline void emitErr(const char* cmd, const char* msg) {
  String b;
  b += "\"cmd\":\""; b += cmd; b += "\"";
  b += ",\"ok\":false";
  b += ",\"millis\":"; b += millis();
  if (msg) {
    b += ",\"error\":\""; b += jsonEscape(msg); b += "\"";
  }
  emitJson("ERR", b);
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
// GNSS ingestion (eager, silent)
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
  unsigned long now = millis();

  while (Serial1.available()) {
    char c = Serial1.read();
    lastGnssByteMs = now;

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) ingestGnssLine(lineBuf);
      lineLen = 0;
      continue;
    }

    if (c != '\r' && lineLen < GNSS_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  if (lineLen && (now - lastGnssByteMs) > GNSS_SILENCE_FLUSH_MS) {
    lineBuf[lineLen] = '\0';
    ingestGnssLine(lineBuf);
    lineLen = 0;
  }
}

// --------------------------------------------------------------
// Event Bus (bounded ring buffer)
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
  if (evt_count >= EVT_MAX) {
    evt_dropped++;
    return;
  }
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

// --------------------------------------------------------------
// Command parsing and execution
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

static void execCommand(const char* line) {
  char cmd[64];
  if (!extractCmd(line, cmd, sizeof(cmd))) {
    emitErr("UNKNOWN", "missing cmd");
    return;
  }

  // -------- Immediate QUERY responses --------
  if (strcmp(cmd, "TEENSY.STATUS?") == 0) {
    emitJson("TEENSY_STATUS", buildTeensyStatusBody());
    return;
  }
  if (strcmp(cmd, "GNSS.STATUS?") == 0) {
    emitJson("GNSS_STATUS", buildGnssStatusBody());
    return;
  }
  if (strcmp(cmd, "GNSS.DATA?") == 0) {
    emitJson("GNSS_DATA", buildGnssDataBody());
    return;
  }

  // -------- Event-generating commands --------
  if (strcmp(cmd, "TEENSY.STATUS") == 0) {
    enqueueEvent("TEENSY_STATUS", buildTeensyStatusBody());
    emitAck(cmd);
    return;
  }
  if (strcmp(cmd, "GNSS.STATUS") == 0) {
    enqueueEvent("GNSS_STATUS", buildGnssStatusBody());
    emitAck(cmd);
    return;
  }
  if (strcmp(cmd, "GNSS.DATA") == 0) {
    enqueueEvent("GNSS_DATA", buildGnssDataBody());
    emitAck(cmd);
    return;
  }

  // -------- Laser actions --------
  if (strcmp(cmd, "LASER.ON") == 0) {
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(LD_ON_PIN, HIGH);
    ldOn = true;
    enqueueEvent("LASER_STATE", "\"enabled\":true");
    emitAck(cmd);
    return;
  }

  if (strcmp(cmd, "LASER.OFF") == 0) {
    digitalWrite(LD_ON_PIN, LOW);
    ldOn = false;
    enqueueEvent("LASER_STATE", "\"enabled\":false");
    emitAck(cmd);
    return;
  }

  if (strcmp(cmd, "EVENTS.GET") == 0) {
    drainEventsNow();
    return;
  }

  emitErr(cmd, "unknown command");
}

// --------------------------------------------------------------
// Setup / Loop
// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial1.begin(GNSSDO_BAUD);

  pinMode(EN_PIN, OUTPUT);
  pinMode(LD_ON_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(LD_ON_PIN, LOW);

  enqueueEvent("BOOT", "\"status\":\"READY\"");
}

void loop() {
  pollGnssSerial();

  static char buf[256];
  static size_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      if (len) execCommand(buf);
      len = 0;
    } else if (c != '\r' && len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
