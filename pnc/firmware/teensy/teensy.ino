/*
  ZPNet Teensy Telemetry Firmware — Pure Command-Driven GNSS Appliance

  Philosophy:
    • Teensy asserts NO health, NO cadence, NO policy.
    • Teensy emits truth only when explicitly asked.
    • GNSS_STATUS is a raw witness (latest sentence only).
    • GNSS_DATA is authoritative snapshot.

  Emits (on demand only):
    • GNSS_STATUS  (latest raw GNSS sentence)
    • GNSS_DATA    (authoritative snapshot)
    • TEENSY_STATUS
    • LASER_STATE

  Author: The Mule + GPT
*/

#include <Arduino.h>
#include <Wire.h>
#include <malloc.h>

// --------------------------------------------------------------
// Version
// --------------------------------------------------------------
static const char* FW_VERSION = "teensy-telemetry-3.1.0";

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

static bool enEnabled = false;
static bool ldOn = false;

// --------------------------------------------------------------
// GNSS rolling state
// --------------------------------------------------------------
static unsigned long lastGnssByteMs = 0;

// Latest raw sentence (any type)
static char last_sentence[GNSS_LINE_MAX] = "";

// Canonical retained sentences
static char last_crw[GNSS_LINE_MAX] = "";
static char last_crx[GNSS_LINE_MAX] = "";
static char last_crz[GNSS_LINE_MAX] = "";
static char last_zda[GNSS_LINE_MAX] = "";
static char last_rmc[GNSS_LINE_MAX] = "";
static char last_gga[GNSS_LINE_MAX] = "";

// Parsed authoritative fields (best-effort)
static char  utc_datetime[20] = "";
static int   time_status = -1;
static int   leap_seconds = -9999;
static float clock_drift_ppb = NAN;
static float temperature_c = NAN;

static int   freq_mode = -1;
static float pps_timing_error_ns = NAN;
static float pps_accuracy_ns = NAN;

static float latitude_deg = NAN;
static float longitude_deg = NAN;
static float altitude_m = NAN;
static int   fix_quality = -1;
static int   num_sats = -1;

// --------------------------------------------------------------
// JSON helper
// --------------------------------------------------------------
static inline void emitJson(const char* eventType, const String& body) {
  Serial.print("{\"event_type\":\"");
  Serial.print(eventType);
  Serial.print("\"");
  if (body.length()) {
    Serial.print(",");
    Serial.print(body);
  }
  Serial.println("}");
}

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

static inline uint32_t freeHeapBytes() {
  struct mallinfo mi = mallinfo();
  return (uint32_t)mi.fordblks;
}

static inline bool startsWith(const char* s, const char* p) {
  return strncmp(s, p, strlen(p)) == 0;
}

static inline void safeCopy(char* dst, size_t sz, const char* src) {
  if (!dst || !src || sz == 0) return;
  memset(dst, 0, sz);
  strncpy(dst, src, sz - 1);
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
// Minimal NMEA helpers
// --------------------------------------------------------------
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
    latitude_deg = nmeaLatLonToDeg(lat, latH);
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
  else if (startsWith(line, "$GPZDA,")) safeCopy(last_zda, sizeof(last_zda), line);
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
// Emissions (on demand only)
// --------------------------------------------------------------
static void emitGnssStatus() {
  String b;
  b += "\"fw_version\":\""; b += FW_VERSION; b += "\"";
  b += ",\"millis\":"; b += millis();

  if (last_sentence[0]) {
    b += ",\"raw_sentence\":\"";
    b += jsonEscape(last_sentence);
    b += "\"";
  }

  emitJson("GNSS_STATUS", b);
}

static void emitGnssData() {
  String b;
  b.reserve(1400);

  b += "\"fw_version\":\""; b += FW_VERSION; b += "\"";

  if (last_crw[0]) { b += ",\"raw_crw\":\""; b += jsonEscape(last_crw); b += "\""; }
  if (last_crx[0]) { b += ",\"raw_crx\":\""; b += jsonEscape(last_crx); b += "\""; }
  if (last_crz[0]) { b += ",\"raw_crz\":\""; b += jsonEscape(last_crz); b += "\""; }
  if (last_zda[0]) { b += ",\"raw_zda\":\""; b += jsonEscape(last_zda); b += "\""; }
  if (last_rmc[0]) { b += ",\"raw_rmc\":\""; b += jsonEscape(last_rmc); b += "\""; }
  if (last_gga[0]) { b += ",\"raw_gga\":\""; b += jsonEscape(last_gga); b += "\""; }

  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
    b += ",\"latitude_deg\":"; b += latitude_deg;
    b += ",\"longitude_deg\":"; b += longitude_deg;
  }
  if (!isnan(altitude_m)) {
    b += ",\"altitude_m\":"; b += altitude_m;
  }

  emitJson("GNSS_DATA", b);
}

static void emitTeensyStatus() {
  String b;
  b += "\"fw_version\":\""; b += FW_VERSION; b += "\"";
  b += ",\"cpu_temp_c\":"; b += cpuTempC();
  b += ",\"free_heap_bytes\":"; b += freeHeapBytes();
  b += ",\"en_enabled\":"; b += (enEnabled ? "true" : "false");
  b += ",\"laser_enabled\":"; b += (ldOn ? "true" : "false");
  b += ",\"status\":\"NOMINAL\"";
  emitJson("TEENSY_STATUS", b);
}

// --------------------------------------------------------------
// USB command handling
// --------------------------------------------------------------
static void handleUsbCommand(const String& cmd) {
  if (cmd == "{\"cmd\":\"GNSS_STATUS_NOW\"}") emitGnssStatus();
  else if (cmd == "{\"cmd\":\"GNSS_DATA_NOW\"}") emitGnssData();
  else if (cmd == "{\"cmd\":\"TEENSY_STATUS_NOW\"}") emitTeensyStatus();
  else if (cmd == "{\"cmd\":\"LASER_ON\"}") {
    digitalWrite(EN_PIN, HIGH); enEnabled = true;
    digitalWrite(LD_ON_PIN, HIGH); ldOn = true;
    emitJson("LASER_STATE", "\"enabled\":true");
  }
  else if (cmd == "{\"cmd\":\"LASER_OFF\"}") {
    digitalWrite(LD_ON_PIN, LOW); ldOn = false;
    emitJson("LASER_STATE", "\"enabled\":false");
  }
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
}

void loop() {
  pollGnssSerial();

  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) handleUsbCommand(buf);
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
}
