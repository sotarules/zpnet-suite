/*
  ZPNet Teensy Telemetry Firmware — GNSS Slave Architecture + GNSS_DATA Snapshot (GF-8802 / CF-8802)

  Design goals:
    • Teensy acts as a stable "device appliance" for GNSS + laser I/O.
    • Policy (cadence, verbosity, querying) lives on the Raspberry Pi in Python.
    • Teensy emits:
        - GNSS_STATUS: boot reassurance burst + occasional / on-demand status
        - GNSS_DATA: authoritative snapshot (time + discipline + PPS + optional position) on request
        - TEENSY_STATUS: periodic health
        - LASER_STATE: when laser commands are received
    • Teensy accepts JSON commands over USB serial to:
        - request GNSS_STATUS now
        - request GNSS_DATA now
        - set verbosity (make GNSS_STATUS chatty when locked)
        - send arbitrary line(s) to GNSSDO (passthrough)

  Version: teensy-telemetry-2.0.0
  Author: The Mule + GPT
*/

#include <Arduino.h>
#include <Wire.h>
#include <malloc.h>

// --------------------------------------------------------------
// Configuration
// --------------------------------------------------------------
static const char* FW_VERSION = "teensy-telemetry-2.0.0";

// --- Timing ---
static const unsigned long TELEMETRY_INTERVAL_MS = 60000UL;

// GNSS status emission policy:
//   - During boot: emit GNSS_STATUS frequently for reassurance (boot burst window)
//   - After boot: emit infrequently, unless verbose_when_locked is enabled
static const unsigned long GNSS_BOOT_BURST_MS = 30000UL;        // 30s reassurance window
static const unsigned long GNSS_STATUS_SLOW_MS = 10000UL;       // 10s default
static const unsigned long GNSS_STATUS_FAST_MS = 1000UL;        // 1s when "verbose" and locked

// GNSS read / frame behavior
static const unsigned long GNSSDO_BAUD = 38400;
static const size_t GNSS_LINE_MAX = 192;                        // enough for CR* sentences (truncated safely)
static const unsigned long GNSS_SILENCE_FLUSH_MS = 50;          // treat silence as boundary

// --------------------------------------------------------------
// Laser gates
// --------------------------------------------------------------
static const int EN_PIN    = 20;   // driver enable / arming gate
static const int LD_ON_PIN = 21;   // emission gate

static bool enEnabled = false;
static bool ldOn = false;

// --------------------------------------------------------------
// GNSS state (latest-known truth)
// --------------------------------------------------------------
static bool verbose_when_locked = false;     // controlled by Pi

static unsigned long boot_ms = 0;
static unsigned long lastTelemetryMs = 0;
static unsigned long lastGnssStatusEmitMs = 0;

// Rolling liveness
static bool serial1_saw_data_since_last_status = false;
static unsigned long lastGnssByteMs = 0;

// Latest key sentences (authoritative-ish)
static char last_crw[GNSS_LINE_MAX] = "";    // $PERDCRW,TPS1,...
static char last_crx[GNSS_LINE_MAX] = "";    // $PERDCRX,TPS2,...
static char last_crz[GNSS_LINE_MAX] = "";    // $PERDCRZ,TPS4,...
static char last_zda[GNSS_LINE_MAX] = "";    // $GPZDA,...
static char last_rmc[GNSS_LINE_MAX] = "";    // $GNRMC,...
static char last_gga[GNSS_LINE_MAX] = "";    // $GNGGA / $GPGGA,...

// Parsed fields for GNSS_DATA (only emit when known)
static char utc_datetime_yyyymmddhhmmss[20] = "";   // from CRW TPS1
static int  time_status = -1;                        // from CRW TPS1 (do not emit if unknown)
static char leap_event_yyyymmddhhmmss[20] = "";      // from CRW TPS1 (if present)
static int  leap_seconds = -9999;                    // from CRW TPS1 (do not emit if unknown)
static float clock_drift_ppb = NAN;                  // from CRW TPS1
static float temperature_c = NAN;                    // from CRW TPS1

// Discipline snapshot (best-effort)
static int freq_mode = -1;                           // from CRZ if we can parse; else unknown
static float pps_timing_error_ns = NAN;              // from CRZ if parseable
static float pps_accuracy_ns = NAN;                  // from CRX if parseable

// Position (optional)
static float latitude_deg = NAN;
static float longitude_deg = NAN;
static float altitude_m = NAN;
static int fix_quality = -1;
static int num_sats = -1;

// --------------------------------------------------------------
// JSON emit utility (newline-delimited JSON objects)
// --------------------------------------------------------------
static inline void emitJson(const char* eventType, const String& bodyNoBraces) {
  Serial.print("{\"event_type\":\"");
  Serial.print(eventType);
  Serial.print("\"");
  if (bodyNoBraces.length() > 0) {
    Serial.print(",");
    Serial.print(bodyNoBraces);
  }
  Serial.println("}");
}

// --------------------------------------------------------------
// Helpers
// --------------------------------------------------------------
static inline float cpuTempC() {
#if defined(ARDUINO_TEENSY41)
  return tempmonGetTemp();
#else
  return 0.0f;
#endif
}

static inline float vccVolts() { return 3.30f; }

static inline uint32_t freeHeapBytes() {
  struct mallinfo mi = mallinfo();
  return (uint32_t)mi.fordblks;
}

static inline bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

static inline void safeCopy(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  memset(dst, 0, dstSize);
  strncpy(dst, src, dstSize - 1);
}

static String jsonEscape(const char* s) {
  // Minimal JSON string escaping for quotes/backslashes/control chars
  String out;
  if (!s) return out;
  while (*s) {
    char c = *s++;
    if (c == '\\') out += "\\\\";
    else if (c == '\"') out += "\\\"";
    else if (c == '\r') out += "\\r";
    else if (c == '\n') out += "\\n";
    else if ((uint8_t)c < 0x20) out += " ";  // replace other controls with space
    else out += c;
  }
  return out;
}

// --------------------------------------------------------------
// NMEA parsing helpers (best-effort, no sentinels emitted)
// --------------------------------------------------------------
static bool nmeaField(const char* line, int fieldIndex, char* out, size_t outSize) {
  // fieldIndex: 0="$GNRMC", 1=time, 2=status, ...
  if (!line || !out || outSize == 0) return false;

  int idx = 0;
  const char* p = line;
  const char* start = p;

  while (*p && *p != '*') {
    if (*p == ',') {
      if (idx == fieldIndex) {
        size_t n = (size_t)(p - start);
        if (n >= outSize) n = outSize - 1;
        memcpy(out, start, n);
        out[n] = '\0';
        return true;
      }
      idx++;
      start = p + 1;
    }
    p++;
  }

  // last field before '*'
  if (idx == fieldIndex && p > start) {
    size_t n = (size_t)(p - start);
    if (n >= outSize) n = outSize - 1;
    memcpy(out, start, n);
    out[n] = '\0';
    return true;
  }

  return false;
}

static float nmeaToFloat(const char* s) {
  if (!s || !*s) return NAN;
  return (float)atof(s);
}

static float nmeaLatLonToDeg(const char* ddmm, const char* hemi) {
  if (!ddmm || !*ddmm || !hemi || !*hemi) return NAN;
  // ddmm.mmmm (lat) or dddmm.mmmm (lon)
  double v = atof(ddmm);
  if (v <= 0.0) return NAN;

  double deg = floor(v / 100.0);
  double min = v - deg * 100.0;
  double out = deg + (min / 60.0);

  char h = hemi[0];
  if (h == 'S' || h == 'W') out = -out;
  return (float)out;
}

static void parseRMC(const char* line) {
  // $GNRMC,hhmmss.sss,A,lat,N,lon,W,sog,cog,ddmmyy,...
  char t[16] = {0};
  char status[4] = {0};
  char lat[16] = {0}, latH[4] = {0};
  char lon[16] = {0}, lonH[4] = {0};

  if (!nmeaField(line, 1, t, sizeof(t))) return;
  if (!nmeaField(line, 2, status, sizeof(status))) return;

  // Only proceed if active
  if (status[0] != 'A') return;

  if (nmeaField(line, 3, lat, sizeof(lat)) && nmeaField(line, 4, latH, sizeof(latH)) &&
      nmeaField(line, 5, lon, sizeof(lon)) && nmeaField(line, 6, lonH, sizeof(lonH))) {
    latitude_deg = nmeaLatLonToDeg(lat, latH);
    longitude_deg = nmeaLatLonToDeg(lon, lonH);
  }
}

static void parseGGA(const char* line) {
  // $GNGGA,hhmmss,lat,N,lon,W,fix,num,hdop,alt,M,...
  char fix[8] = {0};
  char nsats[8] = {0};
  char alt[16] = {0};
  char lat[16] = {0}, latH[4] = {0};
  char lon[16] = {0}, lonH[4] = {0};

  if (nmeaField(line, 6, fix, sizeof(fix))) {
    int q = atoi(fix);
    if (q >= 0) fix_quality = q;
  }
  if (nmeaField(line, 7, nsats, sizeof(nsats))) {
    int n = atoi(nsats);
    if (n >= 0) num_sats = n;
  }
  if (nmeaField(line, 9, alt, sizeof(alt))) {
    altitude_m = nmeaToFloat(alt);
  }

  // also update lat/lon if present
  if (nmeaField(line, 2, lat, sizeof(lat)) && nmeaField(line, 3, latH, sizeof(latH)) &&
      nmeaField(line, 4, lon, sizeof(lon)) && nmeaField(line, 5, lonH, sizeof(lonH))) {
    latitude_deg = nmeaLatLonToDeg(lat, latH);
    longitude_deg = nmeaLatLonToDeg(lon, lonH);
  }
}

// --------------------------------------------------------------
// Proprietary CR* parsing (best-effort)
// We always retain the full raw sentences; parsing extracts only what we’re confident about.
// --------------------------------------------------------------
static void parseCRW_TPS1(const char* line) {
  // Example (from protocol):
  // $PERDCRW,TPS1,YYYYMMDDhhmmss,TimeStatus,YYYYMMDDhhmmss,LeapSec,LeapEvent,ClockDriftPpb,Temp*CS
  //
  // Real fields vary slightly by firmware; we parse conservatively:
  //   field2: datetime
  //   field3: time status
  //   field4: leap event datetime (if present)
  //   then look for leap seconds (small int) and drift/temp near end
  //
  // We do not emit anything we cannot parse plausibly.

  // tokenize by commas into small array
  const int MAXF = 16;
  const char* f[MAXF] = {0};
  int n = 0;

  // copy to scratch (safe)
  static char buf[GNSS_LINE_MAX];
  safeCopy(buf, sizeof(buf), line);

  // strip checksum part
  char* star = strchr(buf, '*');
  if (star) *star = '\0';

  char* p = buf;
  while (p && *p && n < MAXF) {
    f[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = '\0';
    p = c + 1;
  }

  // Need at least: $PERDCRW, TPS1, datetime, timeStatus
  if (n >= 4) {
    if (f[2] && strlen(f[2]) >= 14 && strlen(f[2]) <= 16) {
      safeCopy(utc_datetime_yyyymmddhhmmss, sizeof(utc_datetime_yyyymmddhhmmss), f[2]);
    }
    if (f[3] && strlen(f[3]) > 0) {
      int ts = atoi(f[3]);
      if (ts >= 0 && ts <= 9) time_status = ts;
    }
  }

  // Optional: leap event datetime at f[4]
  if (n >= 5 && f[4] && strlen(f[4]) >= 14) {
    safeCopy(leap_event_yyyymmddhhmmss, sizeof(leap_event_yyyymmddhhmmss), f[4]);
  }

  // Heuristic: find leap seconds (small-ish int) and drift/temp near end
  // This is intentionally conservative; raw sentence remains authoritative.
  for (int i = 5; i < n; i++) {
    if (!f[i] || !*f[i]) continue;
    // leap seconds often looks like +15 / +18 / etc
    if ((f[i][0] == '+' || f[i][0] == '-') && strlen(f[i]) <= 4) {
      int ls = atoi(f[i]);
      if (ls > -30 && ls < 30) leap_seconds = ls;
    }
  }
  // last two numeric-ish fields sometimes drift (ppb) and temp (scaled)
  // try parse floats if present
  if (n >= 2) {
    // scan from end backwards
    for (int i = n - 1; i >= 0; i--) {
      if (!f[i] || !*f[i]) continue;
      if (strchr(f[i], '.')) {
        float v = (float)atof(f[i]);
        // plausible temperature range
        if (v > -50.0f && v < 120.0f && isnan(temperature_c)) {
          temperature_c = v;
          continue;
        }
        // plausible drift range in ppb
        if (v > -100000.0f && v < 100000.0f && isnan(clock_drift_ppb)) {
          clock_drift_ppb = v;
          continue;
        }
      }
    }
  }
}

static void parseCRZ_TPS4(const char* line) {
  // TPS4 is "VCLK Frequency and Control". Field layout depends on firmware.
  // We parse only what we can plausibly identify:
  //   - frequency mode (small int 0..9) often appears early
  //   - PPS timing error often appears as signed small integer-ish or float near middle
  //
  // Raw sentence remains the real truth.
  const int MAXF = 20;
  const char* f[MAXF] = {0};
  int n = 0;

  static char buf[GNSS_LINE_MAX];
  safeCopy(buf, sizeof(buf), line);
  char* star = strchr(buf, '*');
  if (star) *star = '\0';

  char* p = buf;
  while (p && *p && n < MAXF) {
    f[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = '\0';
    p = c + 1;
  }

  // Heuristic: look for a single-digit mode value near the front
  for (int i = 2; i < n; i++) {
    if (!f[i] || !*f[i]) continue;
    if (strlen(f[i]) == 1 && isdigit(f[i][0])) {
      int m = atoi(f[i]);
      if (m >= 0 && m <= 9) {
        freq_mode = m;
        break;
      }
    }
  }

  // Heuristic: look for a plausible PPS error (ns) as signed integer-ish
  // (we only set if magnitude is not insane)
  for (int i = 2; i < n; i++) {
    if (!f[i] || !*f[i]) continue;
    // Many firmwares report signed integers like +000000020
    if ((f[i][0] == '+' || f[i][0] == '-') && strlen(f[i]) >= 2 && strlen(f[i]) <= 12) {
      long v = atol(f[i]);
      if (v > -100000000 && v < 100000000) {
        // Interpret as nanoseconds if it looks like ns-scale
        pps_timing_error_ns = (float)v;
        break;
      }
    }
  }
}

static void parseCRX_TPS2(const char* line) {
  // TPS2 often includes PPS mode + estimated accuracy.
  // Firmware-dependent; we store raw and attempt to parse a small accuracy field.
  const int MAXF = 20;
  const char* f[MAXF] = {0};
  int n = 0;

  static char buf[GNSS_LINE_MAX];
  safeCopy(buf, sizeof(buf), line);
  char* star = strchr(buf, '*');
  if (star) *star = '\0';

  char* p = buf;
  while (p && *p && n < MAXF) {
    f[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = '\0';
    p = c + 1;
  }

  // Try parse a plausible accuracy field (ns) as float
  for (int i = 2; i < n; i++) {
    if (!f[i] || !*f[i]) continue;
    if (strchr(f[i], '.')) {
      float v = (float)atof(f[i]);
      if (v >= 0.0f && v < 1000000.0f) {
        pps_accuracy_ns = v;
        break;
      }
    }
  }
}

// --------------------------------------------------------------
// GNSS line ingestion
// --------------------------------------------------------------
static char lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;

static void ingestGnssLine(const char* line) {
  if (!line || !*line) return;

  // Identify and retain latest key sentences
  if (startsWith(line, "$PERDCRW,")) {
    safeCopy(last_crw, sizeof(last_crw), line);
    parseCRW_TPS1(line);
    return;
  }
  if (startsWith(line, "$PERDCRX,")) {
    safeCopy(last_crx, sizeof(last_crx), line);
    parseCRX_TPS2(line);
    return;
  }
  if (startsWith(line, "$PERDCRZ,")) {
    safeCopy(last_crz, sizeof(last_crz), line);
    parseCRZ_TPS4(line);
    return;
  }

  // Standard NMEA we may want for optional position/time cross-checks
  if (startsWith(line, "$GPZDA,")) {
    safeCopy(last_zda, sizeof(last_zda), line);
    return;
  }
  if (startsWith(line, "$GNRMC,") || startsWith(line, "$GPRMC,")) {
    safeCopy(last_rmc, sizeof(last_rmc), line);
    parseRMC(line);
    return;
  }
  if (startsWith(line, "$GNGGA,") || startsWith(line, "$GPGGA,")) {
    safeCopy(last_gga, sizeof(last_gga), line);
    parseGGA(line);
    return;
  }

  // otherwise ignore (we can add more later without changing architecture)
}

static void pollGnssSerial() {
  unsigned long now = millis();

  while (Serial1.available()) {
    char c = (char)Serial1.read();
    serial1_saw_data_since_last_status = true;
    lastGnssByteMs = now;

    // accumulate into line buffer
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) ingestGnssLine(lineBuf);
      lineLen = 0;
      continue;
    }

    if (lineLen < (GNSS_LINE_MAX - 1)) {
      lineBuf[lineLen++] = c;
    }
  }

  // silence-based flush (some firmwares chunk)
  if (lineLen > 0 && (now - lastGnssByteMs) >= GNSS_SILENCE_FLUSH_MS) {
    lineBuf[lineLen] = '\0';
    ingestGnssLine(lineBuf);
    lineLen = 0;
  }
}

// --------------------------------------------------------------
// GNSS_STATUS and GNSS_DATA emissions
// --------------------------------------------------------------
static const char* gnssHealthState() {
  // Minimal, present-tense:
  //   NOMINAL: we have authoritative UTC datetime from CRW (timing engine is alive)
  //   HOLD: any GNSS data seen recently but not yet time-authoritative
  //   DOWN: no data seen since last status cycle
  if (utc_datetime_yyyymmddhhmmss[0] != '\0') return "NOMINAL";
  if (serial1_saw_data_since_last_status) return "HOLD";
  return "DOWN";
}

static bool lockedEnoughForFastStatus() {
  // "locked" in spirit: we have CRW time OR we have good fix/position.
  if (utc_datetime_yyyymmddhhmmss[0] != '\0') return true;
  if (fix_quality >= 1 && num_sats >= 4) return true;
  return false;
}

static void emitGnssStatusNow(const char* source) {
  String body;
  body.reserve(700);

  body += "\"millis\":";
  body += millis();

  body += ",\"fw_version\":\"";
  body += FW_VERSION;
  body += "\"";

  body += ",\"source\":\"";
  body += source ? source : "unknown";
  body += "\"";

  body += ",\"serial_saw_data\":";
  body += (serial1_saw_data_since_last_status ? "true" : "false");

  // Include a small set of reassuring breadcrumbs during boot / lock
  if (last_crw[0] != '\0') {
    body += ",\"crw_present\":true";
  }
  if (last_crz[0] != '\0') {
    body += ",\"crz_present\":true";
  }

  // Expose time-authoritative witness without exploding payload
  if (utc_datetime_yyyymmddhhmmss[0] != '\0') {
    body += ",\"utc_datetime\":\"";
    body += utc_datetime_yyyymmddhhmmss;
    body += "\"";
  }
  if (time_status >= 0) {
    body += ",\"time_status\":";
    body += time_status;
  }

  // Optional: discipline hints if parsed
  if (!isnan(pps_timing_error_ns)) {
    body += ",\"pps_timing_error_ns\":";
    body += String(pps_timing_error_ns, 0);
  }
  if (freq_mode >= 0) {
    body += ",\"freq_mode\":";
    body += freq_mode;
  }

  body += ",\"health_state\":\"";
  body += gnssHealthState();
  body += "\"";

  emitJson("GNSS_STATUS", body);

  // reset transient witness for next status interval
  serial1_saw_data_since_last_status = false;
}

static void emitGnssDataNow(const char* source) {
  // GNSS_DATA = authoritative snapshot, not a stream.
  // We include raw sentences for TPS1/TPS2/TPS4 as ground truth (auditable),
  // plus parsed convenience fields when known.

  String body;
  body.reserve(1600);

  body += "\"millis\":";
  body += millis();

  body += ",\"fw_version\":\"";
  body += FW_VERSION;
  body += "\"";

  body += ",\"source\":\"";
  body += source ? source : "unknown";
  body += "\"";

  // Health for GNSS_DATA is stronger: do we have time authority?
  body += ",\"health_state\":\"";
  body += (utc_datetime_yyyymmddhhmmss[0] != '\0' ? "NOMINAL" : "HOLD");
  body += "\"";

  // Canonical time (from CRW/TPS1)
  if (utc_datetime_yyyymmddhhmmss[0] != '\0') {
    body += ",\"utc_datetime\":\"";
    body += utc_datetime_yyyymmddhhmmss;
    body += "\"";
  }
  if (time_status >= 0) {
    body += ",\"time_status\":";
    body += time_status;
  }
  if (leap_seconds > -9999) {
    body += ",\"leap_seconds\":";
    body += leap_seconds;
  }
  if (leap_event_yyyymmddhhmmss[0] != '\0') {
    body += ",\"leap_event\":\"";
    body += leap_event_yyyymmddhhmmss;
    body += "\"";
  }
  if (!isnan(clock_drift_ppb)) {
    body += ",\"clock_drift_ppb\":";
    body += String(clock_drift_ppb, 3);
  }
  if (!isnan(temperature_c)) {
    body += ",\"temperature_c\":";
    body += String(temperature_c, 2);
  }

  // Discipline / PPS
  if (freq_mode >= 0) {
    body += ",\"freq_mode\":";
    body += freq_mode;
  }
  if (!isnan(pps_timing_error_ns)) {
    body += ",\"pps_timing_error_ns\":";
    body += String(pps_timing_error_ns, 0);
  }
  if (!isnan(pps_accuracy_ns)) {
    body += ",\"pps_accuracy_ns\":";
    body += String(pps_accuracy_ns, 2);
  }

  // Optional position (only if known)
  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
    body += ",\"latitude_deg\":";
    body += String(latitude_deg, 7);
    body += ",\"longitude_deg\":";
    body += String(longitude_deg, 7);
  }
  if (!isnan(altitude_m)) {
    body += ",\"altitude_m\":";
    body += String(altitude_m, 2);
  }
  if (fix_quality >= 0) {
    body += ",\"fix_quality\":";
    body += fix_quality;
  }
  if (num_sats >= 0) {
    body += ",\"num_satellites\":";
    body += num_sats;
  }

  // Raw auditable sentences (escaped)
  if (last_crw[0] != '\0') {
    body += ",\"raw_crw\":\"";
    body += jsonEscape(last_crw);
    body += "\"";
  }
  if (last_crx[0] != '\0') {
    body += ",\"raw_crx\":\"";
    body += jsonEscape(last_crx);
    body += "\"";
  }
  if (last_crz[0] != '\0') {
    body += ",\"raw_crz\":\"";
    body += jsonEscape(last_crz);
    body += "\"";
  }
  if (last_zda[0] != '\0') {
    body += ",\"raw_zda\":\"";
    body += jsonEscape(last_zda);
    body += "\"";
  }
  if (last_rmc[0] != '\0') {
    body += ",\"raw_rmc\":\"";
    body += jsonEscape(last_rmc);
    body += "\"";
  }
  if (last_gga[0] != '\0') {
    body += ",\"raw_gga\":\"";
    body += jsonEscape(last_gga);
    body += "\"";
  }

  emitJson("GNSS_DATA", body);
}

// --------------------------------------------------------------
// GNSSDO commands (passthrough)
// --------------------------------------------------------------
static void gnssSendLine(const char* line) {
  if (!line || !*line) return;
  Serial1.print(line);
  // Ensure CRLF termination if caller forgot
  size_t n = strlen(line);
  if (n >= 1 && line[n - 1] != '\n') Serial1.print("\r\n");
}

// Conservative “hello” set — safe and non-disruptive.
// (We do not assume hidden modes; this is just ASCII command interface.)
static void gnssHello() {
  // You can replace these with the exact commands you prefer from the protocol.
  // Keeping it minimal: the device will already stream NMEA/CR* once active.
  // If firmware requires explicit enabling, Pi can send CROUT commands later.
  gnssSendLine("$PFEC,GPQ,VERSION*27");
  gnssSendLine("$PFEC,GPQ,INFO*2A");
}

// --------------------------------------------------------------
// USB command protocol (newline-delimited JSON)
// Commands are *requests* from the Pi; Teensy responds by emitting events.
// --------------------------------------------------------------
//
// Supported command types (examples):
//
// {"cmd":"GNSS_STATUS_NOW"}
// {"cmd":"GNSS_DATA_NOW"}
// {"cmd":"SET_VERBOSE","enabled":true}
// {"cmd":"GNSS_SEND","line":"$PERDSYS,VERSION*XX"}      // raw passthrough; Pi should send correct checksum
// {"cmd":"LASER_ON"} / {"cmd":"LASER_OFF"}             // optional JSON form in addition to plain text
//
static void handleLaserOn() {
  digitalWrite(EN_PIN, HIGH);
  enEnabled = true;

  digitalWrite(LD_ON_PIN, HIGH);
  ldOn = true;

  emitJson("LASER_STATE", "\"enabled\":true,\"source\":\"serial_command\"");
}

static void handleLaserOff() {
  digitalWrite(LD_ON_PIN, LOW);
  ldOn = false;

  emitJson("LASER_STATE", "\"enabled\":false,\"source\":\"serial_command\"");
}

static bool jsonBoolField(const char* json, const char* key, bool* out) {
  // Extremely small parser: looks for "key":true/false
  if (!json || !key || !out) return false;

  String pat = String("\"") + key + "\":";
  const char* p = strstr(json, pat.c_str());
  if (!p) return false;
  p += pat.length();

  while (*p == ' ' || *p == '\t') p++;

  if (strncmp(p, "true", 4) == 0) { *out = true; return true; }
  if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
  return false;
}

static bool jsonStringField(const char* json, const char* key, char* out, size_t outSize) {
  // Looks for "key":"value"
  if (!json || !key || !out || outSize == 0) return false;

  String pat = String("\"") + key + "\":\"";
  const char* p = strstr(json, pat.c_str());
  if (!p) return false;
  p += pat.length();

  const char* q = strchr(p, '\"');
  if (!q) return false;

  size_t n = (size_t)(q - p);
  if (n >= outSize) n = outSize - 1;
  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}

static bool jsonCmdIs(const char* json, const char* cmd) {
  if (!json || !cmd) return false;
  String pat = String("\"cmd\":\"") + cmd + "\"";
  return strstr(json, pat.c_str()) != nullptr;
}

static void handleUsbCommandJson(const char* jsonLine) {
  if (jsonCmdIs(jsonLine, "GNSS_STATUS_NOW")) {
    emitGnssStatusNow("usb_request");
    emitJson("COMMAND_ACK", "\"cmd\":\"GNSS_STATUS_NOW\",\"ok\":true");
    return;
  }

  if (jsonCmdIs(jsonLine, "GNSS_DATA_NOW")) {
    emitGnssDataNow("usb_request");
    emitJson("COMMAND_ACK", "\"cmd\":\"GNSS_DATA_NOW\",\"ok\":true");
    return;
  }

  if (jsonCmdIs(jsonLine, "SET_VERBOSE")) {
    bool en = false;
    if (jsonBoolField(jsonLine, "enabled", &en)) {
      verbose_when_locked = en;
      emitJson("COMMAND_ACK", String("\"cmd\":\"SET_VERBOSE\",\"ok\":true,\"enabled\":") + (en ? "true" : "false"));
      return;
    }
    emitJson("COMMAND_ACK", "\"cmd\":\"SET_VERBOSE\",\"ok\":false,\"error\":\"missing enabled\"");
    return;
  }

  if (jsonCmdIs(jsonLine, "GNSS_SEND")) {
    static char line[GNSS_LINE_MAX];
    if (jsonStringField(jsonLine, "line", line, sizeof(line))) {
      gnssSendLine(line);
      emitJson("COMMAND_ACK", "\"cmd\":\"GNSS_SEND\",\"ok\":true");
      return;
    }
    emitJson("COMMAND_ACK", "\"cmd\":\"GNSS_SEND\",\"ok\":false,\"error\":\"missing line\"");
    return;
  }

  if (jsonCmdIs(jsonLine, "LASER_ON")) {
    handleLaserOn();
    emitJson("COMMAND_ACK", "\"cmd\":\"LASER_ON\",\"ok\":true");
    return;
  }
  if (jsonCmdIs(jsonLine, "LASER_OFF")) {
    handleLaserOff();
    emitJson("COMMAND_ACK", "\"cmd\":\"LASER_OFF\",\"ok\":true");
    return;
  }

  emitJson("COMMAND_ACK", "\"cmd\":\"UNKNOWN\",\"ok\":false");
}

// Also support your old plain-text commands for convenience
static void handleUsbPlainText(const String& cmd) {
  if (cmd == "LASER_ON") { handleLaserOn(); return; }
  if (cmd == "LASER_OFF") { handleLaserOff(); return; }

  // If you type raw GNSS line manually, allow prefix "GNSS:"
  if (cmd.startsWith("GNSS:")) {
    String line = cmd.substring(5);
    line.trim();
    gnssSendLine(line.c_str());
    emitJson("COMMAND_ACK", "\"cmd\":\"GNSS_SEND\",\"ok\":true");
    return;
  }
}

static void pollUsbCommands() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf.trim();
      if (buf.length() > 0) {
        if (buf.startsWith("{")) {
          handleUsbCommandJson(buf.c_str());
        } else {
          handleUsbPlainText(buf);
        }
      }
      buf = "";
      continue;
    }
    if (buf.length() < 2048) buf += c;
  }
}

// --------------------------------------------------------------
// Periodic TEENSY_STATUS
// --------------------------------------------------------------
static void emitTeensyStatus() {
  String payload;
  payload.reserve(512);

  payload += "\"fw_version\":\""; payload += FW_VERSION; payload += "\"";
  payload += ",\"millis\":"; payload += millis();
  payload += ",\"cpu_temp_c\":"; payload += cpuTempC();
  payload += ",\"vcc_v\":"; payload += vccVolts();
  payload += ",\"free_heap_bytes\":"; payload += freeHeapBytes();
  payload += ",\"en_enabled\":"; payload += (enEnabled ? "true" : "false");
  payload += ",\"laser_enabled\":"; payload += (ldOn ? "true" : "false");
  payload += ",\"status\":\"NOMINAL\"";

  emitJson("TEENSY_STATUS", payload);
}

// --------------------------------------------------------------
// Setup / Loop
// --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Serial1.begin(GNSSDO_BAUD);

  pinMode(EN_PIN, OUTPUT);
  pinMode(LD_ON_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(LD_ON_PIN, LOW);

  boot_ms = millis();
  lastTelemetryMs = 0;
  lastGnssStatusEmitMs = 0;

  // Clear state
  lineLen = 0;
  serial1_saw_data_since_last_status = false;
  lastGnssByteMs = 0;

  safeCopy(last_crw, sizeof(last_crw), "");
  safeCopy(last_crx, sizeof(last_crx), "");
  safeCopy(last_crz, sizeof(last_crz), "");
  safeCopy(last_zda, sizeof(last_zda), "");
  safeCopy(last_rmc, sizeof(last_rmc), "");
  safeCopy(last_gga, sizeof(last_gga), "");

  safeCopy(utc_datetime_yyyymmddhhmmss, sizeof(utc_datetime_yyyymmddhhmmss), "");
  safeCopy(leap_event_yyyymmddhhmmss, sizeof(leap_event_yyyymmddhhmmss), "");
  time_status = -1;
  leap_seconds = -9999;
  clock_drift_ppb = NAN;
  temperature_c = NAN;
  freq_mode = -1;
  pps_timing_error_ns = NAN;
  pps_accuracy_ns = NAN;

  latitude_deg = NAN;
  longitude_deg = NAN;
  altitude_m = NAN;
  fix_quality = -1;
  num_sats = -1;

  emitJson(
    "ZPNET_BOOT",
    String("\"fw_version\":\"") + FW_VERSION +
    "\",\"millis\":" + String(millis()) +
    ",\"en_enabled\":false" +
    ",\"laser_enabled\":false"
  );

  // Gentle hello (does not assume “mode”; just queries)
  delay(200);
  gnssHello();

  // Immediate first status so the operator sees life
  emitGnssStatusNow("boot");
}

void loop() {
  pollUsbCommands();
  pollGnssSerial();

  unsigned long now = millis();

  // GNSS_STATUS emission policy:
  //  - Always chatty during boot burst window
  //  - After that: slow, unless verbose_when_locked && lockedEnoughForFastStatus()
  unsigned long desiredInterval = GNSS_STATUS_SLOW_MS;

  if ((now - boot_ms) < GNSS_BOOT_BURST_MS) {
    desiredInterval = GNSS_STATUS_FAST_MS;
  } else if (verbose_when_locked && lockedEnoughForFastStatus()) {
    desiredInterval = GNSS_STATUS_FAST_MS;
  } else {
    desiredInterval = GNSS_STATUS_SLOW_MS;
  }

  if (now - lastGnssStatusEmitMs >= desiredInterval) {
    emitGnssStatusNow("periodic");
    lastGnssStatusEmitMs = now;
  }

  // TEENSY_STATUS
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    emitTeensyStatus();
    lastTelemetryMs = now;
  }

  delay(2);
}
