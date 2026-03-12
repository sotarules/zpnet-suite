#include "payload.h"
#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ============================================================================
// Global Payload Instrumentation (monotonic, centralized)
// ============================================================================

static volatile uint32_t g_payload_instances_constructed = 0;
static volatile uint32_t g_payload_instances_destroyed   = 0;

static volatile uint32_t g_payload_alive_now             = 0;
static volatile uint32_t g_payload_alive_high_water      = 0;

static volatile uint32_t g_payload_arena_alloc_fail      = 0;
static volatile uint32_t g_payload_entry_overflow        = 0;

static volatile uint32_t g_payload_arena_high_water_global = 0;
static volatile uint32_t g_payload_entry_high_water_global = 0;

// ----------------------------------------------------------------------------
// Internal helper — mark construction
// ----------------------------------------------------------------------------

static inline void payload_mark_constructed() {
    g_payload_instances_constructed++;
    g_payload_alive_now++;

    if (g_payload_alive_now > g_payload_alive_high_water) {
        g_payload_alive_high_water = g_payload_alive_now;
    }
}

// ----------------------------------------------------------------------------
// Internal helper — mark destruction
// ----------------------------------------------------------------------------

static inline void payload_mark_destroyed() {
    g_payload_instances_destroyed++;

    // Guard against underflow (should never happen)
    if (g_payload_alive_now > 0) {
        g_payload_alive_now--;
    }
}

// ============================================================================
// Construction / Destruction
// ============================================================================

Payload::Payload()
    : _count(0)
    , _arena(nullptr)
    , _arena_used(0)
    , _arena_cap(0)
{
    payload_mark_constructed();
}

Payload::~Payload() {
    free(_arena);
    payload_mark_destroyed();
}

// ============================================================================
// Move semantics (zero-copy arena transfer)
// ============================================================================

Payload::Payload(Payload&& other) noexcept
    : _count(other._count)
    , _arena(other._arena)
    , _arena_used(other._arena_used)
    , _arena_cap(other._arena_cap)
{
    payload_mark_constructed();

    memcpy(_entries, other._entries, _count * sizeof(Entry));

    // Donor is now empty — no dangling free
    other._arena      = nullptr;
    other._arena_used = 0;
    other._arena_cap  = 0;
    other._count      = 0;
}

Payload& Payload::operator=(Payload&& other) noexcept {
    if (this == &other) return *this;

    free(_arena);

    _count      = other._count;
    _arena      = other._arena;
    _arena_used = other._arena_used;
    _arena_cap  = other._arena_cap;

    memcpy(_entries, other._entries, _count * sizeof(Entry));

    other._arena      = nullptr;
    other._arena_used = 0;
    other._arena_cap  = 0;
    other._count      = 0;

    return *this;
}

// ============================================================================
// Copy semantics (deep copy of arena)
// ============================================================================

Payload::Payload(const Payload& other)
    : _count(other._count)
    , _arena(nullptr)
    , _arena_used(other._arena_used)
    , _arena_cap(other._arena_used)  // trim to used size
{
    payload_mark_constructed();

    memcpy(_entries, other._entries, _count * sizeof(Entry));

    if (_arena_used > 0) {
        _arena = (char*)malloc(_arena_cap);
        if (_arena) {
            memcpy(_arena, other._arena, _arena_used);
        } else {
            // Allocation failure: reset to empty
            _count      = 0;
            _arena_used = 0;
            _arena_cap  = 0;
        }
    }
}

Payload& Payload::operator=(const Payload& other) {
    if (this == &other) return *this;

    free(_arena);
    _arena = nullptr;

    _count      = other._count;
    _arena_used = other._arena_used;
    _arena_cap  = other._arena_used;  // trim

    memcpy(_entries, other._entries, _count * sizeof(Entry));

    if (_arena_used > 0) {
        _arena = (char*)malloc(_arena_cap);
        if (_arena) {
            memcpy(_arena, other._arena, _arena_used);
        } else {
            _count      = 0;
            _arena_used = 0;
            _arena_cap  = 0;
        }
    }

    return *this;
}

// ============================================================================
// Lifecycle
// ============================================================================

void Payload::clear() {
    _count      = 0;
    _arena_used = 0;
    // Keep arena allocation for reuse — don't free
}

bool Payload::empty() const {
    return _count == 0;
}

Payload Payload::clone() const {
    return Payload(*this);  // uses copy constructor
}

// ============================================================================
// Arena internals
// ============================================================================

bool Payload::_ensure(size_t additional) {
    size_t needed = _arena_used + additional;

    if (needed <= _arena_cap) return true;

    if (needed > ARENA_MAX) {
        g_payload_arena_alloc_fail++;
        return false;
    }

    size_t new_cap = _arena_cap == 0 ? ARENA_INITIAL : _arena_cap;
    while (new_cap < needed) {
        new_cap *= 2;
    }
    if (new_cap > ARENA_MAX) new_cap = ARENA_MAX;

    char* new_arena = (char*)realloc(_arena, new_cap);
    if (!new_arena) {
        g_payload_arena_alloc_fail++;
        return false;
    }

    _arena     = new_arena;
    _arena_cap = new_cap;
    return true;
}

uint16_t Payload::_put(const char* str, size_t len) {

    if (!_ensure(len + 1)) return UINT16_MAX;

    if (_arena_used + len + 1 > UINT16_MAX) {
        g_payload_arena_alloc_fail++;
        return UINT16_MAX;
    }

    uint16_t offset = (uint16_t)_arena_used;
    memcpy(_arena + _arena_used, str, len);
    _arena[_arena_used + len] = '\0';
    _arena_used += len + 1;

    // Global high-water update
    if (_arena_used > g_payload_arena_high_water_global) {
        g_payload_arena_high_water_global = _arena_used;
    }

    return offset;
}


uint16_t Payload::_put(const char* str) {
    if (!str) str = "";
    return _put(str, strlen(str));
}

const char* Payload::_at(uint16_t offset) const {
    if (!_arena || offset >= _arena_used) return "";
    return _arena + offset;
}

const Payload::Entry* Payload::_find(const char* key) const {
    for (size_t i = 0; i < _count; i++) {
        if (strcmp(_at(_entries[i].key_off), key) == 0) {
            return &_entries[i];
        }
    }
    return nullptr;
}

// ============================================================================
// Internal: add an entry with pre-formatted value
// ============================================================================

void Payload::_add_entry(const char* key, const char* value, size_t value_len, char kind) {

    if (_count >= MAX_ENTRIES) {
        g_payload_entry_overflow++;
        return;
    }

    uint16_t k_off = _put(key);
    if (k_off == UINT16_MAX) return;

    uint16_t v_off = _put(value, value_len);
    if (v_off == UINT16_MAX) return;

    _entries[_count++] = {
        k_off,
        v_off,
        (uint16_t)value_len,
        kind,
        0
    };

    if (_count > g_payload_entry_high_water_global) {
        g_payload_entry_high_water_global = _count;
    }
}


// ============================================================================
// Semantic construction
// ============================================================================

void Payload::add(const char* key, int32_t value) {
    char tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%ld", (long)value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, uint32_t value) {
    char tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, int64_t value) {
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%lld", (long long)value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, uint64_t value) {
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%llu", (unsigned long long)value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, const char* value) {
    if (!value) value = "";
    _add_entry(key, value, strlen(value), 'p');
}

void Payload::add(const char* key, const String& value) {
    add(key, value.c_str());
}

void Payload::add(const char* key, bool value) {
    const char* s = value ? "true" : "false";
    _add_entry(key, s, strlen(s), 'p');
}

void Payload::add(const char* key, float value) {
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%.6f", (double)value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, double value) {
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%.6f", value);
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add_fmt(const char* key, const char* fmt, ...) {
    if (_count >= MAX_ENTRIES) return;

    char tmp[96];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(tmp)) n = (int)(sizeof(tmp) - 1);

    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add_object(const char* key, const Payload& obj) {
    if (_count >= MAX_ENTRIES) return;

    // Serialize the nested object into scratch, then copy to arena
    char local_buf[ARENA_MAX];
    size_t json_len = obj.write_json(local_buf, sizeof(local_buf));

    if (json_len == 0) {
        _add_entry(key, "{}", 2, 'o');
    } else {
        _add_entry(key, local_buf, json_len, 'o');
    }
}

void Payload::add_array(const char* key, const PayloadArray& arr) {
    if (_count >= MAX_ENTRIES) return;

    String json = arr.to_json();
    _add_entry(key, json.c_str(), json.length(), 'a');
}

void Payload::add_raw_object(const char* key, const char* raw_json_object) {
    if (!raw_json_object) raw_json_object = "{}";
    _add_entry(key, raw_json_object, strlen(raw_json_object), 'o');
}

// ============================================================================
// JSON Serialization
// ============================================================================

size_t Payload::write_json(char* buf, size_t buf_size) const {
    if (!buf || buf_size < 3) {
        // Minimum: "{}\0"
        if (buf && buf_size > 0) buf[0] = '\0';
        return 0;
    }

    size_t pos = 0;
    bool overflow = false;

    // -----------------------------------------------------------
    // Local append helpers
    // -----------------------------------------------------------

    auto remaining = [&]() -> size_t {
        return buf_size - pos - 1;  // reserve 1 for NUL
    };

    auto append_char = [&](char c) {
        if (overflow) return;
        if (remaining() < 1) { overflow = true; return; }
        buf[pos++] = c;
    };

    auto append_str = [&](const char* s, size_t n) {
        if (overflow || !s) return;
        if (n > remaining()) { overflow = true; return; }
        memcpy(buf + pos, s, n);
        pos += n;
    };

    auto append = [&](const char* s) {
        if (s) append_str(s, strlen(s));
    };

    auto append_escaped = [&](const char* s) {
        if (overflow || !s) return;
        while (*s) {
            char c = *s++;
            switch (c) {
                case '"': append_str("\\\"", 2); break;
                case '\\': append_str("\\\\", 2); break;
                case '\n': append_str("\\n", 2);  break;
                case '\r': append_str("\\r", 2);  break;
                case '\t': append_str("\\t", 2);  break;
                default:   append_char(c);         break;
            }
            if (overflow) return;
        }
    };

    // -----------------------------------------------------------
    // Serialize
    // -----------------------------------------------------------

    append_char('{');

    for (size_t i = 0; i < _count && !overflow; i++) {
        if (i) append_char(',');

        const Entry& e = _entries[i];
        const char* key = _at(e.key_off);
        const char* val = _at(e.val_off);

        // Key
        append_char('"');
        append_escaped(key);
        append_str("\":", 2);

        if (overflow) break;

        // Value
        switch (e.kind) {
            case 'p': {
                // Detect booleans and numbers to emit unquoted
                if (strcmp(val, "true") == 0 || strcmp(val, "false") == 0) {
                    append(val);
                } else {
                    char* end = nullptr;
                    strtod(val, &end);
                    if (val[0] != '\0' && end && *end == '\0') {
                        // Numeric — emit raw
                        append(val);
                    } else {
                        // String — emit quoted
                        append_char('"');
                        append_escaped(val);
                        append_char('"');
                    }
                }
                break;
            }

            case 'o':
            case 'a':
                // Already valid JSON — emit verbatim
                append_str(val, e.val_len);
                break;

            default:
                append_str("null", 4);
                break;
        }
    }

    append_char('}');

    if (overflow) {
        // Overflow fallback: emit error object
        const char* err = "{\"error\":\"payload_overflow\"}";
        size_t err_len = strlen(err);
        if (err_len < buf_size) {
            memcpy(buf, err, err_len);
            buf[err_len] = '\0';
            return err_len;
        }
        buf[0] = '\0';
        return 0;
    }

    buf[pos] = '\0';
    return pos;
}

String Payload::to_json() const {
    char local_buf[4096];
    write_json(local_buf, sizeof(local_buf));
    return String(local_buf);
}

// ============================================================================
// Parsing — canonicalizes JSON into entries[] + arena
// ============================================================================

bool Payload::parseJSON(const uint8_t* data, size_t len) {
    clear();

    if (!data || len < 2 || data[0] != '{') {
        return false;
    }

    size_t i = 1;

    while (i < len && _count < MAX_ENTRIES) {

        // Find opening quote of key
        while (i < len && data[i] != '"') i++;
        if (i >= len) break;

        size_t key_start = ++i;
        while (i < len && data[i] != '"') i++;
        if (i >= len) break;

        size_t key_len = i - key_start;
        i++;  // skip closing quote

        // Find colon
        while (i < len && data[i] != ':') i++;
        if (i >= len) break;
        i++;  // skip colon

        // Skip whitespace
        while (i < len && (data[i] == ' ' || data[i] == '\t' ||
                           data[i] == '\n' || data[i] == '\r')) i++;
        if (i >= len) break;

        // Determine value kind
        char kind =
            data[i] == '{' ? 'o' :
            data[i] == '[' ? 'a' :
            'p';

        size_t value_start = i;

        if (kind == 'p') {
            // Primitive: scan to comma or closing brace
            while (i < len && data[i] != ',' && data[i] != '}') i++;
        } else {
            // Object or array: balanced brace/bracket scan
            int depth = 0;
            do {
                if (data[i] == '{' || data[i] == '[') depth++;
                if (data[i] == '}' || data[i] == ']') depth--;
                i++;
            } while (i < len && depth > 0);
        }

        size_t value_len = i - value_start;

        // Trim trailing whitespace from primitive values
        if (kind == 'p') {
            while (value_len > 0 &&
                   (data[value_start + value_len - 1] == ' ' ||
                    data[value_start + value_len - 1] == '\t' ||
                    data[value_start + value_len - 1] == '\n' ||
                    data[value_start + value_len - 1] == '\r')) {
                value_len--;
            }
        }

        // Store key in arena
        uint16_t k_off = _put((const char*)(data + key_start), key_len);
        if (k_off == UINT16_MAX) break;

        // For primitive strings, strip surrounding quotes
        const char* val_ptr = (const char*)(data + value_start);
        size_t      val_len = value_len;

        if (kind == 'p' && val_len >= 2 &&
            val_ptr[0] == '"' && val_ptr[val_len - 1] == '"') {
            val_ptr++;
            val_len -= 2;
        }

        // Store value in arena
        uint16_t v_off = _put(val_ptr, val_len);
        if (v_off == UINT16_MAX) break;

        _entries[_count++] = {
            k_off,
            v_off,
            (uint16_t)val_len,
            kind,
            0
        };

        // Skip comma
        if (i < len && data[i] == ',') i++;
    }

    return true;
}

// ============================================================================
// Lookup / Accessors
// ============================================================================

bool Payload::has(const char* key) const {
    return _find(key) != nullptr;
}

const char* Payload::getString(const char* key) const {
    const Entry* e = _find(key);
    if (!e || e->kind != 'p') return nullptr;
    return _at(e->val_off);
}

// ============================================================================
// Strict accessors
// ============================================================================

bool Payload::tryGetBool(const char* key, bool& out) const {
    const char* s = getString(key);
    if (!s) return false;
    if (strcmp(s, "true") == 0)  { out = true;  return true; }
    if (strcmp(s, "false") == 0) { out = false; return true; }
    return false;
}

bool Payload::tryGetInt(const char* key, int32_t& out) const {
    const char* s = getString(key);
    if (!s) return false;
    char* end = nullptr;
    long v = strtol(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = (int32_t)v;
    return true;
}

bool Payload::tryGetUInt(const char* key, uint32_t& out) const {
    const char* s = getString(key);
    if (!s) return false;
    char* end = nullptr;
    unsigned long v = strtoul(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = (uint32_t)v;
    return true;
}

bool Payload::tryGetUInt64(const char* key, uint64_t& out) const {
    const char* s = getString(key);
    if (!s) return false;
    char* end = nullptr;
    unsigned long long v = strtoull(s, &end, 10);
    if (!end || *end != '\0') return false;
    out = (uint64_t)v;
    return true;
}

bool Payload::tryGetFloat(const char* key, float& out) const {
    const char* s = getString(key);
    if (!s) return false;
    char* end = nullptr;
    float v = strtof(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

bool Payload::tryGetDouble(const char* key, double& out) const {
    const char* s = getString(key);
    if (!s) return false;
    char* end = nullptr;
    double v = strtod(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

// ============================================================================
// Convenience accessors (with defaults)
// ============================================================================

bool Payload::getBool(const char* key, bool default_value) const {
    bool v;
    return tryGetBool(key, v) ? v : default_value;
}

int32_t Payload::getInt(const char* key, int32_t default_value) const {
    int32_t v;
    return tryGetInt(key, v) ? v : default_value;
}

uint32_t Payload::getUInt(const char* key, uint32_t default_value) const {
    uint32_t v;
    return tryGetUInt(key, v) ? v : default_value;
}

uint64_t Payload::getUInt64(const char* key, uint64_t default_value) const {
    uint64_t v;
    return tryGetUInt64(key, v) ? v : default_value;
}

float Payload::getFloat(const char* key, float default_value) const {
    float v;
    return tryGetFloat(key, v) ? v : default_value;
}

double Payload::getDouble(const char* key, double default_value) const {
    double v;
    return tryGetDouble(key, v) ? v : default_value;
}

// ============================================================================
// Structured accessors
// ============================================================================

Payload Payload::getPayload(const char* key) const {
    Payload p;
    const Entry* e = _find(key);
    if (!e || e->kind != 'o') return p;
    const char* val = _at(e->val_off);
    p.parseJSON((const uint8_t*)val, e->val_len);
    return p;
}

PayloadArray Payload::getArray(const char* key) const {
    PayloadArray arr;
    const Entry* e = _find(key);
    if (!e || e->kind != 'a') return arr;
    arr.parseJSON(_at(e->val_off));
    return arr;
}

bool Payload::hasArray(const char* key) const {
    const Entry* e = _find(key);
    return e && e->kind == 'a';
}

PayloadArrayView Payload::getArrayView(const char* key) const {
    const Entry* e = _find(key);
    if (!e || e->kind != 'a') {
        return PayloadArrayView();
    }
    return PayloadArrayView(_at(e->val_off), e->val_len);
}

// ============================================================================
// Diagnostics
// ============================================================================

void Payload::debug_dump(const char* tag) const {
    char line[128];

    snprintf(
        line, sizeof(line),
        "Payload dump (%s): %u entries, arena %u/%u bytes",
        tag ? tag : "",
        (unsigned)_count,
        (unsigned)_arena_used,
        (unsigned)_arena_cap
    );
    debug_log("payload", line);

    for (size_t i = 0; i < _count; i++) {
        const Entry& e = _entries[i];

        const char* kind_str =
            e.kind == 'p' ? "primitive" :
            e.kind == 'o' ? "object" :
            e.kind == 'a' ? "array" :
            "unknown";

        debug_log("payload.key", _at(e.key_off));
        debug_log("payload.kind", kind_str);

        const char* v = _at(e.val_off);
        char preview[96];
        size_t n = strlen(v);
        if (n > sizeof(preview) - 1) n = sizeof(preview) - 1;
        memcpy(preview, v, n);
        preview[n] = '\0';

        debug_log("payload.value", preview);
    }
}

// ============================================================================
// PayloadArrayView
// ============================================================================

PayloadArrayView::PayloadArrayView()
    : _json(nullptr), _len(0) {}

PayloadArrayView::PayloadArrayView(const char* json, size_t len)
    : _json(json), _len(len) {}

bool PayloadArrayView::valid() const {
    return _json && _len >= 2 && _json[0] == '[';
}

size_t PayloadArrayView::size() const {
    if (!valid()) return 0;

    size_t count = 0;
    int depth = 0;
    bool in_string = false;

    for (size_t i = 0; i < _len; i++) {
        char c = _json[i];

        if (c == '"' && (i == 0 || _json[i - 1] != '\\')) {
            in_string = !in_string;
        }
        if (in_string) continue;

        if (c == '{' || c == '[') depth++;
        if (c == '}' || c == ']') depth--;
        if (c == ',' && depth == 1) count++;
    }

    return count + 1;
}

Payload PayloadArrayView::get(size_t index) const {
    Payload out;
    if (!valid()) return out;

    size_t current = 0;
    int depth = 0;
    bool in_string = false;
    size_t start = 0;

    for (size_t i = 1; i < _len; i++) {
        char c = _json[i];

        if (c == '"' && _json[i - 1] != '\\') {
            in_string = !in_string;
        }
        if (in_string) continue;

        if (c == '{' || c == '[') {
            if (depth == 0 && current == index) start = i;
            depth++;
        }

        if (c == '}' || c == ']') {
            depth--;
            if (depth == 0 && current == index) {
                out.parseJSON(
                    (const uint8_t*)(_json + start),
                    i - start + 1
                );
                return out;
            }
        }

        if (c == ',' && depth == 0) current++;
    }

    return Payload();
}

// ============================================================================
// PayloadArray
// ============================================================================

PayloadArray::PayloadArray()
    : _buf("")
    , _first(true)
    , _item_count(0) {}

void PayloadArray::clear() {
    _buf = "";
    _first = true;
    _item_count = 0;
}

bool PayloadArray::empty() const {
    return _item_count == 0 && _buf.length() == 0;
}

String PayloadArray::to_json() const {
    if (_item_count > 0) {
        String out = "[";
        for (size_t i = 0; i < _item_count; i++) {
            if (i) out += ",";
            out += _items[i].to_json();
        }
        out += "]";
        return out;
    }

    return String("[") + _buf + "]";
}

void PayloadArray::add(const Payload& obj) {
    if (!_first) _buf += ",";
    _first = false;
    _buf += obj.to_json();
}

bool PayloadArray::parseJSON(const char* json) {
    _item_count = 0;
    if (!json || json[0] != '[') return false;

    int i = 1;
    while (json[i] && _item_count < MAX_ITEMS) {
        while (json[i] && json[i] != '{') i++;
        if (!json[i]) break;

        int v0 = i;
        int depth = 0;

        do {
            if (json[i] == '{') depth++;
            if (json[i] == '}') depth--;
            i++;
        } while (depth > 0 && json[i]);

        _items[_item_count].parseJSON(
            (const uint8_t*)(json + v0),
            (size_t)(i - v0)
        );

        _item_count++;
    }

    return true;
}

size_t PayloadArray::size() const {
    return _item_count;
}

Payload PayloadArray::get(size_t idx) const {
    if (idx >= _item_count) return Payload();
    return _items[idx];
}

// ============================================================================
// Payload Instrumentation Access (snapshot-only, monotonic)
// ============================================================================

void payload_get_info(payload_info_t* out)
{
    if (!out) return;

    // ----------------------------------------------------------
    // Lifetime totals
    // ----------------------------------------------------------

    out->instances_constructed = g_payload_instances_constructed;
    out->instances_destroyed   = g_payload_instances_destroyed;

    // ----------------------------------------------------------
    // Live instance tracking (invariant support)
    // ----------------------------------------------------------

    out->alive_now             = g_payload_alive_now;
    out->alive_high_water      = g_payload_alive_high_water;

    // ----------------------------------------------------------
    // Arena behavior
    // ----------------------------------------------------------

    out->arena_alloc_fail      = g_payload_arena_alloc_fail;
    out->arena_high_water      = g_payload_arena_high_water_global;

    // ----------------------------------------------------------
    // Entry behavior
    // ----------------------------------------------------------

    out->entry_overflow        = g_payload_entry_overflow;
    out->entry_high_water      = g_payload_entry_high_water_global;
}