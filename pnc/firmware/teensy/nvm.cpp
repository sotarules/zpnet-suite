// ============================================================================
// FILE: nvm.cpp
// ============================================================================

#include "nvm.h"
#include "debug.h"

#include <Arduino.h>
#include <LittleFS.h>

// Use external QSPI flash on Teensy 4.1
static LittleFS_QSPIFlash FS;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

static String key_to_filename(const char* key) {
  // Very conservative sanitization:
  // key: "subscriptions:TEMPEST"
  // file: "/subscriptions_TEMPEST.json"

  String name = "/";
  while (*key) {
    char c = *key++;
    if (c == ':' || c == '/' || c == '\\') {
      name += '_';
    } else {
      name += c;
    }
  }
  name += ".json";
  return name;
}

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

bool nvm_init(void) {

  if (FS.begin()) {
    debug_log("nvm", "LittleFS mounted");
    return true;
  }

  // First boot or corrupted FS — do NOT silently format
  debug_log("nvm", "LittleFS mount failed");

  // For now: explicit format on failure (first incarnation)
  // Later we may gate this behind a compile-time flag
  debug_log("nvm", "formatting filesystem");
  if (!FS.format()) {
    debug_log("nvm", "format failed");
    return false;
  }

  if (!FS.begin()) {
    debug_log("nvm", "mount failed after format");
    return false;
  }

  debug_log("nvm", "filesystem formatted and mounted");
  return true;
}

// ------------------------------------------------------------
// CRUD
// ------------------------------------------------------------

bool nvm_read(
  const char* key,
  Payload&    out
) {
  if (!key || !*key) return false;

  String path = key_to_filename(key);
  File f = FS.open(path.c_str(), FILE_READ);
  if (!f) return false;

  size_t len = f.size();
  if (len == 0 || len > 4096) {
    f.close();
    return false;
  }

  static uint8_t buf[4096];
  size_t n = f.read(buf, len);
  f.close();

  if (n != len) return false;

  return out.parseJSON(buf, len);
}

bool nvm_write(
  const char* key,
  const Payload& value
) {
  if (!key || !*key) return false;

  String path = key_to_filename(key);
  File f = FS.open(path.c_str(), FILE_WRITE);
  if (!f) return false;

  String json = value.to_json();
  size_t n = f.write((const uint8_t*)json.c_str(), json.length());
  f.close();

  return n == json.length();
}

bool nvm_delete(
  const char* key
) {
  if (!key || !*key) return false;

  String path = key_to_filename(key);
  if (!FS.exists(path.c_str())) return false;

  return FS.remove(path.c_str());
}

// ------------------------------------------------------------
// Introspection
// ------------------------------------------------------------

bool nvm_exists(const char* key) {
  if (!key || !*key) return false;
  return FS.exists(key_to_filename(key).c_str());
}

void nvm_debug_dump(void) {
  File root = FS.open("/");
  if (!root) return;

  File f = root.openNextFile();
  while (f) {
    debug_log("nvm.file", f.name());
    f = root.openNextFile();
  }
}