#include "payload.h"
#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// ============================================================================
// Global Payload Instrumentation (monotonic, centralized)
// ============================================================================

static volatile uint32_t g_payload_instances_constructed = 0;
static volatile uint32_t g_payload_instances_destroyed   = 0;

static volatile uint32_t g_payload_alive_now             = 0;
static volatile uint32_t g_payload_alive_high_water      = 0;

static volatile uint32_t g_payload_entry_alloc_fail      = 0;
static volatile uint32_t g_payload_entry_realloc_count   = 0;
static volatile uint32_t g_payload_entry_heap_bytes_alive = 0;
static volatile uint32_t g_payload_entry_heap_bytes_high_water = 0;
static volatile uint32_t g_payload_entry_overflow        = 0;
static volatile uint32_t g_payload_entry_high_water_global = 0;
static volatile uint32_t g_payload_max_entry_capacity_seen = 0;

static volatile uint32_t g_payload_arena_alloc_fail      = 0;
static volatile uint32_t g_payload_arena_realloc_count   = 0;
static volatile uint32_t g_payload_arena_heap_bytes_alive = 0;
static volatile uint32_t g_payload_arena_heap_bytes_high_water = 0;
static volatile uint32_t g_payload_arena_high_water_global = 0;
static volatile uint32_t g_payload_max_arena_capacity_seen = 0;

static volatile uint32_t g_payload_serialize_overflow    = 0;
static volatile uint32_t g_payload_to_json_fail          = 0;
static volatile uint32_t g_payload_string_truncation     = 0;
static volatile uint32_t g_payload_parse_error           = 0;
static volatile uint32_t g_payload_integrity_fail        = 0;
static volatile uint32_t g_payload_invalid_kind          = 0;

static volatile uint32_t g_payload_last_error_code       = 0;
static volatile uint32_t g_payload_last_error_count      = 0;
static volatile uint32_t g_payload_last_error_this       = 0;
static char g_payload_last_error_op[32] = {0};

// ============================================================================
// Error codes
// ============================================================================

enum payload_error_code_t : uint32_t {
    PAYLOAD_ERR_NONE = 0,
    PAYLOAD_ERR_ENTRY_OVERFLOW = 1,
    PAYLOAD_ERR_ENTRY_ALLOC_FAIL = 2,
    PAYLOAD_ERR_ARENA_ALLOC_FAIL = 3,
    PAYLOAD_ERR_ARENA_LIMIT = 4,
    PAYLOAD_ERR_STRING_TOO_LONG = 5,
    PAYLOAD_ERR_SERIALIZE_OVERFLOW = 6,
    PAYLOAD_ERR_TO_JSON_FAIL = 7,
    PAYLOAD_ERR_STRING_TRUNCATION = 8,
    PAYLOAD_ERR_PARSE_ERROR = 9,
    PAYLOAD_ERR_INTEGRITY = 10,
    PAYLOAD_ERR_INVALID_KIND = 11,
    PAYLOAD_ERR_COPY_ALLOC_FAIL = 12,
};

const char* payload_error_code_name(uint32_t code) {
    switch (code) {
        case PAYLOAD_ERR_NONE:               return "NONE";
        case PAYLOAD_ERR_ENTRY_OVERFLOW:     return "ENTRY_OVERFLOW";
        case PAYLOAD_ERR_ENTRY_ALLOC_FAIL:   return "ENTRY_ALLOC_FAIL";
        case PAYLOAD_ERR_ARENA_ALLOC_FAIL:   return "ARENA_ALLOC_FAIL";
        case PAYLOAD_ERR_ARENA_LIMIT:        return "ARENA_LIMIT";
        case PAYLOAD_ERR_STRING_TOO_LONG:    return "STRING_TOO_LONG";
        case PAYLOAD_ERR_SERIALIZE_OVERFLOW: return "SERIALIZE_OVERFLOW";
        case PAYLOAD_ERR_TO_JSON_FAIL:       return "TO_JSON_FAIL";
        case PAYLOAD_ERR_STRING_TRUNCATION:  return "STRING_TRUNCATION";
        case PAYLOAD_ERR_PARSE_ERROR:        return "PARSE_ERROR";
        case PAYLOAD_ERR_INTEGRITY:          return "INTEGRITY";
        case PAYLOAD_ERR_INVALID_KIND:       return "INVALID_KIND";
        case PAYLOAD_ERR_COPY_ALLOC_FAIL:    return "COPY_ALLOC_FAIL";
        default:                             return "UNKNOWN";
    }
}

static void payload_note_error(uint32_t code, const char* op, const void* self) {
    g_payload_last_error_code = code;
    g_payload_last_error_count++;
    g_payload_last_error_this = (uint32_t)(uintptr_t)self;

    if (!op) op = "?";
    size_t n = strlen(op);
    if (n >= sizeof(g_payload_last_error_op)) {
        n = sizeof(g_payload_last_error_op) - 1;
    }
    memcpy(g_payload_last_error_op, op, n);
    g_payload_last_error_op[n] = '\0';
}

static inline void payload_mark_constructed() {
    g_payload_instances_constructed++;
    g_payload_alive_now++;

    if (g_payload_alive_now > g_payload_alive_high_water) {
        g_payload_alive_high_water = g_payload_alive_now;
    }
}

static inline void payload_mark_destroyed() {
    g_payload_instances_destroyed++;

    if (g_payload_alive_now > 0) {
        g_payload_alive_now--;
    }
}

static inline void payload_note_entry_heap_delta(int32_t delta) {
    if (delta >= 0) {
        g_payload_entry_heap_bytes_alive += (uint32_t)delta;
    } else {
        const uint32_t dec = (uint32_t)(-delta);
        g_payload_entry_heap_bytes_alive =
            (g_payload_entry_heap_bytes_alive >= dec)
                ? (g_payload_entry_heap_bytes_alive - dec)
                : 0;
    }

    if (g_payload_entry_heap_bytes_alive > g_payload_entry_heap_bytes_high_water) {
        g_payload_entry_heap_bytes_high_water = g_payload_entry_heap_bytes_alive;
    }
}

static inline void payload_note_arena_heap_delta(int32_t delta) {
    if (delta >= 0) {
        g_payload_arena_heap_bytes_alive += (uint32_t)delta;
    } else {
        const uint32_t dec = (uint32_t)(-delta);
        g_payload_arena_heap_bytes_alive =
            (g_payload_arena_heap_bytes_alive >= dec)
                ? (g_payload_arena_heap_bytes_alive - dec)
                : 0;
    }

    if (g_payload_arena_heap_bytes_alive > g_payload_arena_heap_bytes_high_water) {
        g_payload_arena_heap_bytes_high_water = g_payload_arena_heap_bytes_alive;
    }
}

static size_t grow_power_of_two(size_t current, size_t minimum, size_t maximum) {
    size_t cap = current ? current : 1;
    while (cap < minimum && cap < maximum) {
        cap *= 2;
    }
    if (cap > maximum) cap = maximum;
    return cap;
}

// ============================================================================
// Construction / Destruction
// ============================================================================

Payload::Payload()
    : _magic(MAGIC_LIVE)
    , _entries(_inline_entries)
    , _count(0)
    , _entry_cap(INLINE_ENTRIES)
    , _arena(nullptr)
    , _arena_used(0)
    , _arena_cap(0)
{
    payload_mark_constructed();
    if (INLINE_ENTRIES > g_payload_max_entry_capacity_seen) {
        g_payload_max_entry_capacity_seen = INLINE_ENTRIES;
    }
}

Payload::~Payload() {
    _release_storage();
    _magic = MAGIC_DEAD;
    payload_mark_destroyed();
}

void Payload::_release_storage() {
    if (_entries && _entries != _inline_entries) {
        payload_note_entry_heap_delta(-(int32_t)(_entry_cap * sizeof(Entry)));
        free(_entries);
    }

    if (_arena) {
        payload_note_arena_heap_delta(-(int32_t)_arena_cap);
        free(_arena);
    }

    _entries = _inline_entries;
    _count = 0;
    _entry_cap = INLINE_ENTRIES;
    _arena = nullptr;
    _arena_used = 0;
    _arena_cap = 0;
}

// ============================================================================
// Move semantics
// ============================================================================

Payload::Payload(Payload&& other) noexcept
    : _magic(MAGIC_LIVE)
    , _entries(_inline_entries)
    , _count(0)
    , _entry_cap(INLINE_ENTRIES)
    , _arena(nullptr)
    , _arena_used(0)
    , _arena_cap(0)
{
    payload_mark_constructed();
    _move_from(other);
}

Payload& Payload::operator=(Payload&& other) noexcept {
    if (this == &other) return *this;

    _release_storage();
    _magic = MAGIC_LIVE;
    _move_from(other);
    return *this;
}

void Payload::_move_from(Payload& other) {
    if (!other._self_ok("move_from")) {
        return;
    }

    _count = other._count;

    if (other._entries == other._inline_entries) {
        _entries = _inline_entries;
        _entry_cap = INLINE_ENTRIES;
        if (_count > 0) {
            memcpy(_inline_entries, other._inline_entries, _count * sizeof(Entry));
        }
    } else {
        _entries = other._entries;
        _entry_cap = other._entry_cap;
        other._entries = other._inline_entries;
        other._entry_cap = INLINE_ENTRIES;
    }

    _arena = other._arena;
    _arena_used = other._arena_used;
    _arena_cap = other._arena_cap;

    other._count = 0;
    other._arena = nullptr;
    other._arena_used = 0;
    other._arena_cap = 0;
}

// ============================================================================
// Copy semantics
// ============================================================================

Payload::Payload(const Payload& other)
    : _magic(MAGIC_LIVE)
    , _entries(_inline_entries)
    , _count(0)
    , _entry_cap(INLINE_ENTRIES)
    , _arena(nullptr)
    , _arena_used(0)
    , _arena_cap(0)
{
    payload_mark_constructed();
    _copy_from(other);
}

Payload& Payload::operator=(const Payload& other) {
    if (this == &other) return *this;

    _release_storage();
    _magic = MAGIC_LIVE;
    _copy_from(other);
    return *this;
}

bool Payload::_copy_from(const Payload& other) {
    if (!other._self_ok("copy_from")) {
        return false;
    }

    if (other._count > 0) {
        if (!_ensure_entries(other._count)) {
            g_payload_entry_alloc_fail++;
            payload_note_error(PAYLOAD_ERR_COPY_ALLOC_FAIL, "copy_entries", this);
            return false;
        }
        memcpy(_entries, other._entries, other._count * sizeof(Entry));
        _count = other._count;
    }

    if (other._arena_used > 0) {
        if (!_ensure_arena(other._arena_used)) {
            payload_note_error(PAYLOAD_ERR_COPY_ALLOC_FAIL, "copy_arena", this);
            _count = 0;
            _arena_used = 0;
            return false;
        }
        memcpy(_arena, other._arena, other._arena_used);
        _arena_used = other._arena_used;
    }

    return true;
}

// ============================================================================
// Lifecycle
// ============================================================================

void Payload::clear() {
    if (!_self_ok("clear")) return;
    _count = 0;
    _arena_used = 0;
}

bool Payload::empty() const {
    return !_self_ok("empty") || _count == 0;
}

Payload Payload::clone() const {
    return Payload(*this);
}

// ============================================================================
// Internal integrity guard
// ============================================================================

bool Payload::_self_ok(const char* op) const {
    if (_magic != MAGIC_LIVE) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    if (!_entries || _entry_cap < INLINE_ENTRIES || _entry_cap > MAX_ENTRIES) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    const uintptr_t entries_addr = (uintptr_t)_entries;
    const uintptr_t arena_addr = (uintptr_t)_arena;
    if (entries_addr == (uintptr_t)MAGIC_LIVE ||
        entries_addr == (uintptr_t)MAGIC_DEAD ||
        arena_addr == (uintptr_t)MAGIC_LIVE ||
        arena_addr == (uintptr_t)MAGIC_DEAD) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    const bool entries_inline = (_entries == _inline_entries);
    if ((entries_inline && _entry_cap != INLINE_ENTRIES) ||
        (!entries_inline && _entry_cap <= INLINE_ENTRIES)) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    if (_count > _entry_cap || _count > MAX_ENTRIES) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    if (_arena_used > _arena_cap || _arena_cap > ARENA_MAX) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    if ((_arena_cap == 0 && _arena != nullptr) ||
        (_arena_cap != 0 && _arena == nullptr)) {
        g_payload_integrity_fail++;
        payload_note_error(PAYLOAD_ERR_INTEGRITY, op, this);
        return false;
    }

    return true;
}

// ============================================================================
// Entry + arena internals
// ============================================================================

bool Payload::_ensure_entries(size_t needed_count) {
    if (!_self_ok("ensure_entries")) return false;

    if (needed_count <= _entry_cap) return true;

    if (needed_count > MAX_ENTRIES) {
        g_payload_entry_overflow++;
        payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, "ensure_entries", this);
        return false;
    }

    size_t new_cap = grow_power_of_two(_entry_cap, needed_count, MAX_ENTRIES);
    if (new_cap <= _entry_cap) {
        g_payload_entry_overflow++;
        payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, "ensure_entries_cap", this);
        return false;
    }

    // Deliberately avoid realloc(): growth is an explicit allocate/copy/free
    // transaction so allocator-side memmove never interprets Payload storage
    // state while expanding the entry table.
    const bool had_heap_entries = (_entries != _inline_entries);
    Entry* const old_entries = had_heap_entries ? _entries : nullptr;
    const size_t old_bytes = had_heap_entries ? (_entry_cap * sizeof(Entry)) : 0U;
    const size_t new_bytes = new_cap * sizeof(Entry);

    Entry* new_entries = (Entry*)malloc(new_bytes);
    if (!new_entries) {
        g_payload_entry_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ENTRY_ALLOC_FAIL, "ensure_entries_alloc", this);
        return false;
    }

    if (_count > 0) {
        memcpy(new_entries, _entries, _count * sizeof(Entry));
    }
    memset(new_entries + _count, 0, (new_cap - _count) * sizeof(Entry));

    _entries = new_entries;
    _entry_cap = new_cap;
    g_payload_entry_realloc_count++;
    payload_note_entry_heap_delta((int32_t)(new_bytes - old_bytes));

    if (old_entries) {
        free(old_entries);
    }

    if (_entry_cap > g_payload_max_entry_capacity_seen) {
        g_payload_max_entry_capacity_seen = _entry_cap;
    }
    return true;
}

bool Payload::_ensure_arena(size_t additional) {
    if (!_self_ok("ensure_arena")) return false;

    if (additional > ARENA_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, "ensure_arena_add", this);
        return false;
    }

    if (_arena_used > ARENA_MAX || additional > (ARENA_MAX - _arena_used)) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, "ensure_arena_limit", this);
        return false;
    }

    const size_t needed = _arena_used + additional;
    if (needed <= _arena_cap) return true;

    size_t new_cap = _arena_cap == 0 ? ARENA_INITIAL : _arena_cap;
    new_cap = grow_power_of_two(new_cap, needed, ARENA_MAX);

    if (new_cap < needed || new_cap > ARENA_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, "ensure_arena_cap", this);
        return false;
    }

    // Deliberately avoid realloc(): growth is an explicit allocate/copy/free
    // transaction so allocator-side memmove never receives Payload arena state.
    char* const old_arena = _arena;
    const size_t old_cap = _arena_cap;
    char* new_arena = (char*)malloc(new_cap);
    if (!new_arena) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, "ensure_arena_alloc", this);
        return false;
    }

    if (_arena_used > 0) {
        memcpy(new_arena, _arena, _arena_used);
    }

    _arena = new_arena;
    _arena_cap = new_cap;
    g_payload_arena_realloc_count++;
    payload_note_arena_heap_delta((int32_t)(new_cap - old_cap));

    if (old_arena) {
        free(old_arena);
    }

    if (_arena_cap > g_payload_max_arena_capacity_seen) {
        g_payload_max_arena_capacity_seen = _arena_cap;
    }
    return true;
}

uint16_t Payload::_put(const char* str, size_t len) {
    if (!_self_ok("put")) return UINT16_MAX;

    if (!str) {
        if (len != 0) {
            payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, "put_null", this);
            return UINT16_MAX;
        }
        str = "";
    }

    if (len > UINT16_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, "put_len", this);
        return UINT16_MAX;
    }

    if (len + 1U > ARENA_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, "put_arena_limit", this);
        return UINT16_MAX;
    }

    uintptr_t arena_alias_offset = UINTPTR_MAX;
    if (_arena) {
        const uintptr_t arena_begin = (uintptr_t)_arena;
        const uintptr_t arena_end = arena_begin + _arena_used;
        const uintptr_t str_addr = (uintptr_t)str;
        if (str_addr >= arena_begin && str_addr < arena_end) {
            arena_alias_offset = str_addr - arena_begin;
        }
    }

    if (!_ensure_arena(len + 1U)) return UINT16_MAX;

    if (arena_alias_offset != UINTPTR_MAX) {
        str = _arena + arena_alias_offset;
    }

    if (_arena_used + len + 1U > UINT16_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_ARENA_LIMIT, "put_offset_limit", this);
        return UINT16_MAX;
    }

    const uint16_t offset = (uint16_t)_arena_used;
    if (len > 0) {
        memcpy(_arena + _arena_used, str, len);
    }
    _arena[_arena_used + len] = '\0';
    _arena_used += len + 1U;

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
    if (!_self_ok("at")) return "";
    if (!_arena || offset >= _arena_used) return "";
    return _arena + offset;
}

const Payload::Entry* Payload::_find(const char* key) const {
    if (!_self_ok("find") || !key) return nullptr;
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
    if (!_self_ok("add_entry")) return;

    if (!key) key = "";
    if (!value) {
        value = "";
        value_len = 0;
    }

    if (kind != 'p' && kind != 'o' && kind != 'a') {
        g_payload_invalid_kind++;
        payload_note_error(PAYLOAD_ERR_INVALID_KIND, "add_entry_kind", this);
        kind = 'p';
    }

    if (_count >= MAX_ENTRIES) {
        g_payload_entry_overflow++;
        payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, "add_entry_count", this);
        return;
    }

    if (value_len > UINT16_MAX) {
        g_payload_arena_alloc_fail++;
        payload_note_error(PAYLOAD_ERR_STRING_TOO_LONG, "add_entry_value_len", this);
        return;
    }

    if (!_ensure_entries(_count + 1U)) {
        return;
    }

    const size_t arena_mark = _arena_used;

    const uint16_t k_off = _put(key);
    if (k_off == UINT16_MAX) {
        _arena_used = arena_mark;
        return;
    }

    const uint16_t v_off = _put(value, value_len);
    if (v_off == UINT16_MAX) {
        _arena_used = arena_mark;
        return;
    }

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
    if (n < 0) n = 0;
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, uint32_t value) {
    char tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)value);
    if (n < 0) n = 0;
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, int64_t value) {
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%lld", (long long)value);
    if (n < 0) n = 0;
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, uint64_t value) {
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%llu", (unsigned long long)value);
    if (n < 0) n = 0;
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
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(tmp)) {
        n = (int)sizeof(tmp) - 1;
        g_payload_string_truncation++;
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, "add_float", this);
    }
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, double value) {
    char tmp[32];
    int n = snprintf(tmp, sizeof(tmp), "%.6f", value);
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(tmp)) {
        n = (int)sizeof(tmp) - 1;
        g_payload_string_truncation++;
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, "add_double", this);
    }
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add(const char* key, double value, int precision) {
    if (precision < 0) precision = 0;
    if (precision > 12) precision = 12;

    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%.%df", precision);
    char tmp[48];
    int n = snprintf(tmp, sizeof(tmp), fmt, value);
    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(tmp)) {
        n = (int)sizeof(tmp) - 1;
        g_payload_string_truncation++;
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, "add_double_prec", this);
    }
    _add_entry(key, tmp, (size_t)n, 'p');
}

void Payload::add_fmt(const char* key, const char* fmt, ...) {
    if (!_self_ok("add_fmt")) return;
    if (!fmt) fmt = "";

    char tmp[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    if (n < 0) n = 0;
    if ((size_t)n >= sizeof(tmp)) {
        n = (int)(sizeof(tmp) - 1);
        g_payload_string_truncation++;
        payload_note_error(PAYLOAD_ERR_STRING_TRUNCATION, "add_fmt", this);
    }

    _add_entry(key, tmp, (size_t)n, 'p');
}

// Estimate a bounded starting buffer for serializing a nested Payload.  The
// exact size depends on escaping, so callers retry upward when needed.
static size_t payload_initial_json_buffer_size(const Payload& obj) {
    size_t buf_size = (obj.arena_used() * 2U) + (obj.count() * 32U) + 64U;
    if (buf_size < 256U) buf_size = 256U;
    if (buf_size > Payload::ARENA_MAX) buf_size = Payload::ARENA_MAX;
    return buf_size;
}

void Payload::add_object(const char* key, const Payload& obj) {
    if (!_self_ok("add_object")) return;
    if (_count >= MAX_ENTRIES) {
        g_payload_entry_overflow++;
        payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, "add_object", this);
        return;
    }

    size_t buf_size = payload_initial_json_buffer_size(obj);
    bool added = false;

    while (buf_size <= ARENA_MAX) {
        char* json_buf = (char*)malloc(buf_size);
        if (!json_buf) {
            g_payload_arena_alloc_fail++;
            payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, "add_object_tmp", this);
            break;
        }

        size_t json_len = obj.write_json(json_buf, buf_size);
        if (json_len != 0) {
            _add_entry(key, json_buf, json_len, 'o');
            added = true;
            free(json_buf);
            break;
        }

        free(json_buf);
        if (buf_size == ARENA_MAX) break;
        buf_size *= 2U;
        if (buf_size > ARENA_MAX) buf_size = ARENA_MAX;
    }

    if (!added) {
        g_payload_serialize_overflow++;
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, "add_object_json", this);
        _add_entry(key, "{}", 2, 'o');
    }
}

void Payload::add_array(const char* key, const PayloadArray& arr) {
    if (!_self_ok("add_array")) return;
    String json = arr.to_json();
    _add_entry(key, json.c_str(), json.length(), 'a');
}

void Payload::add_raw_object(const char* key, const char* raw_json_object) {
    if (!raw_json_object || raw_json_object[0] != '{') {
        g_payload_parse_error++;
        payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "add_raw_object", this);
        raw_json_object = "{}";
    }
    _add_entry(key, raw_json_object, strlen(raw_json_object), 'o');
}

// ============================================================================
// JSON Serialization
// ============================================================================

static bool is_json_number_literal(const char* s) {
    if (!s || !*s) return false;

    const char* p = s;
    if (*p == '-') p++;

    if (*p == '0') {
        p++;
    } else if (*p >= '1' && *p <= '9') {
        while (isdigit((unsigned char)*p)) p++;
    } else {
        return false;
    }

    if (*p == '.') {
        p++;
        if (!isdigit((unsigned char)*p)) return false;
        while (isdigit((unsigned char)*p)) p++;
    }

    if (*p == 'e' || *p == 'E') {
        p++;
        if (*p == '+' || *p == '-') p++;
        if (!isdigit((unsigned char)*p)) return false;
        while (isdigit((unsigned char)*p)) p++;
    }

    return *p == '\0';
}

size_t Payload::write_json(char* buf, size_t buf_size) const {
    if (!buf || buf_size < 3) {
        if (buf && buf_size > 0) buf[0] = '\0';
        g_payload_serialize_overflow++;
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, "write_json_buf", this);
        return 0;
    }

    if (!_self_ok("write_json")) {
        buf[0] = '\0';
        return 0;
    }

    size_t pos = 0;
    bool overflow = false;

    auto remaining = [&]() -> size_t {
        return buf_size - pos - 1U;
    };

    auto append_char = [&](char c) {
        if (overflow) return;
        if (remaining() < 1U) { overflow = true; return; }
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
            const unsigned char c = (unsigned char)*s++;
            switch (c) {
                case '"': append_str("\\\"", 2); break;
                case '\\': append_str("\\\\", 2); break;
                case '\n': append_str("\\n", 2);  break;
                case '\r': append_str("\\r", 2);  break;
                case '\t': append_str("\\t", 2);  break;
                default:
                    if (c < 0x20U) {
                        char esc[7];
                        snprintf(esc, sizeof(esc), "\\u%04X", (unsigned)c);
                        append_str(esc, 6);
                    } else {
                        append_char((char)c);
                    }
                    break;
            }
            if (overflow) return;
        }
    };

    append_char('{');

    for (size_t i = 0; i < _count && !overflow; i++) {
        if (i) append_char(',');

        const Entry& e = _entries[i];
        const char* key = _at(e.key_off);
        const char* val = _at(e.val_off);

        append_char('"');
        append_escaped(key);
        append_str("\":", 2);

        if (overflow) break;

        switch (e.kind) {
            case 'p': {
                if (strcmp(val, "true") == 0 ||
                    strcmp(val, "false") == 0 ||
                    strcmp(val, "null") == 0 ||
                    is_json_number_literal(val)) {
                    append(val);
                } else {
                    append_char('"');
                    append_escaped(val);
                    append_char('"');
                }
                break;
            }

            case 'o':
            case 'a':
                append_str(val, e.val_len);
                break;

            default:
                g_payload_invalid_kind++;
                payload_note_error(PAYLOAD_ERR_INVALID_KIND, "write_json_kind", this);
                append_str("null", 4);
                break;
        }
    }

    append_char('}');

    if (overflow) {
        buf[0] = '\0';
        g_payload_serialize_overflow++;
        payload_note_error(PAYLOAD_ERR_SERIALIZE_OVERFLOW, "write_json_overflow", this);
        return 0;
    }

    buf[pos] = '\0';
    return pos;
}

String Payload::to_json() const {
    if (!_self_ok("to_json")) {
        g_payload_to_json_fail++;
        return String("{}");
    }

    size_t buf_size = payload_initial_json_buffer_size(*this);

    while (buf_size <= ARENA_MAX) {
        char* json_buf = (char*)malloc(buf_size);
        if (!json_buf) {
            g_payload_arena_alloc_fail++;
            payload_note_error(PAYLOAD_ERR_ARENA_ALLOC_FAIL, "to_json_alloc", this);
            break;
        }

        const size_t json_len = write_json(json_buf, buf_size);
        if (json_len != 0) {
            String out(json_buf);
            free(json_buf);
            return out;
        }

        free(json_buf);
        if (buf_size == ARENA_MAX) break;
        buf_size *= 2U;
        if (buf_size > ARENA_MAX) buf_size = ARENA_MAX;
    }

    g_payload_to_json_fail++;
    payload_note_error(PAYLOAD_ERR_TO_JSON_FAIL, "to_json", this);
    return String("{}");
}

// ============================================================================
// Parsing — canonicalizes JSON into entries[] + arena
// ============================================================================

static void skip_ws(const uint8_t* data, size_t len, size_t& i) {
    while (i < len && (data[i] == ' ' || data[i] == '\t' ||
                       data[i] == '\n' || data[i] == '\r')) {
        i++;
    }
}

static bool scan_json_string(const uint8_t* data, size_t len, size_t& i,
                             size_t& content_start, size_t& content_len) {
    if (i >= len || data[i] != '"') return false;
    content_start = ++i;

    bool escape = false;
    while (i < len) {
        const uint8_t c = data[i];
        if (escape) {
            escape = false;
            i++;
            continue;
        }
        if (c == '\\') {
            escape = true;
            i++;
            continue;
        }
        if (c == '"') {
            content_len = i - content_start;
            i++;
            return true;
        }
        i++;
    }
    return false;
}

static bool scan_json_balanced(const uint8_t* data, size_t len, size_t& i,
                               uint8_t open_c, uint8_t close_c,
                               size_t& value_start, size_t& value_len) {
    (void)open_c;
    (void)close_c;
    if (i >= len || (data[i] != '{' && data[i] != '[')) return false;
    value_start = i;

    int brace_depth = 0;
    int bracket_depth = 0;
    bool in_string = false;
    bool escape = false;

    while (i < len) {
        const uint8_t c = data[i++];

        if (in_string) {
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"') {
            in_string = true;
            continue;
        }
        if (c == '{') brace_depth++;
        else if (c == '}') brace_depth--;
        else if (c == '[') bracket_depth++;
        else if (c == ']') bracket_depth--;

        if (brace_depth < 0 || bracket_depth < 0) {
            return false;
        }
        if (brace_depth == 0 && bracket_depth == 0) {
            value_len = i - value_start;
            return true;
        }
    }
    return false;
}

static bool scan_json_primitive(const uint8_t* data, size_t len, size_t& i,
                                size_t& value_start, size_t& value_len) {
    value_start = i;
    while (i < len && data[i] != ',' && data[i] != '}') {
        i++;
    }
    value_len = i - value_start;
    while (value_len > 0) {
        const uint8_t c = data[value_start + value_len - 1U];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            value_len--;
        } else {
            break;
        }
    }
    return value_len > 0;
}

bool Payload::parseJSON(const uint8_t* data, size_t len) {
    clear();

    if (!data || len < 2) {
        g_payload_parse_error++;
        payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_null", this);
        return false;
    }

    size_t i = 0;
    skip_ws(data, len, i);
    if (i >= len || data[i] != '{') {
        g_payload_parse_error++;
        payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_open", this);
        return false;
    }
    i++;

    skip_ws(data, len, i);
    if (i < len && data[i] == '}') {
        return true;
    }

    while (i < len) {
        skip_ws(data, len, i);

        size_t key_start = 0;
        size_t key_len = 0;
        if (!scan_json_string(data, len, i, key_start, key_len)) {
            g_payload_parse_error++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_key", this);
            return false;
        }

        skip_ws(data, len, i);
        if (i >= len || data[i] != ':') {
            g_payload_parse_error++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_colon", this);
            return false;
        }
        i++;
        skip_ws(data, len, i);
        if (i >= len) {
            g_payload_parse_error++;
            payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_value_missing", this);
            return false;
        }

        char kind = 'p';
        size_t value_start = i;
        size_t value_len = 0;

        if (data[i] == '"') {
            if (!scan_json_string(data, len, i, value_start, value_len)) {
                g_payload_parse_error++;
                payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_string", this);
                return false;
            }
            kind = 'p';
        } else if (data[i] == '{') {
            if (!scan_json_balanced(data, len, i, '{', '}', value_start, value_len)) {
                g_payload_parse_error++;
                payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_object", this);
                return false;
            }
            kind = 'o';
        } else if (data[i] == '[') {
            if (!scan_json_balanced(data, len, i, '[', ']', value_start, value_len)) {
                g_payload_parse_error++;
                payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_array", this);
                return false;
            }
            kind = 'a';
        } else {
            if (!scan_json_primitive(data, len, i, value_start, value_len)) {
                g_payload_parse_error++;
                payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_primitive", this);
                return false;
            }
            kind = 'p';
        }

        if (_count >= MAX_ENTRIES) {
            g_payload_entry_overflow++;
            payload_note_error(PAYLOAD_ERR_ENTRY_OVERFLOW, "parse_entries", this);
            return false;
        }

        if (!_ensure_entries(_count + 1U)) return false;

        const size_t arena_mark = _arena_used;
        const uint16_t k_off = _put((const char*)(data + key_start), key_len);
        if (k_off == UINT16_MAX) {
            _arena_used = arena_mark;
            return false;
        }

        const uint16_t v_off = _put((const char*)(data + value_start), value_len);
        if (v_off == UINT16_MAX) {
            _arena_used = arena_mark;
            return false;
        }

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

        skip_ws(data, len, i);
        if (i < len && data[i] == ',') {
            i++;
            continue;
        }
        if (i < len && data[i] == '}') {
            return true;
        }

        // Permit trailing whitespace after the closing object only.
        if (i >= len) break;
        g_payload_parse_error++;
        payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_separator", this);
        return false;
    }

    g_payload_parse_error++;
    payload_note_error(PAYLOAD_ERR_PARSE_ERROR, "parse_eof", this);
    return false;
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
    char line[160];

    snprintf(
        line, sizeof(line),
        "Payload dump (%s): %u entries cap=%u arena %u/%u bytes heap_entries=%u sizeof=%u",
        tag ? tag : "",
        (unsigned)_count,
        (unsigned)_entry_cap,
        (unsigned)_arena_used,
        (unsigned)_arena_cap,
        (unsigned)(_entries != _inline_entries),
        (unsigned)sizeof(Payload)
    );
    debug_log("payload", line);

    if (!_self_ok("debug_dump")) return;

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

    size_t first = 1;
    while (first < _len && (_json[first] == ' ' || _json[first] == '\t' ||
                            _json[first] == '\n' || _json[first] == '\r')) {
        first++;
    }
    if (first < _len && _json[first] == ']') {
        return 0;
    }

    size_t count = 0;
    int depth = 0;
    bool in_string = false;
    bool escape = false;

    for (size_t i = 0; i < _len; i++) {
        const char c = _json[i];

        if (in_string) {
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"') {
            in_string = true;
            continue;
        }
        if (c == '{' || c == '[') depth++;
        if (c == '}' || c == ']') depth--;
        if (c == ',' && depth == 1) count++;
    }

    return count + 1U;
}

Payload PayloadArrayView::get(size_t index) const {
    Payload out;
    if (!valid()) return out;

    size_t current = 0;
    int depth = 0;
    bool in_string = false;
    bool escape = false;
    size_t start = 0;

    for (size_t i = 1; i < _len; i++) {
        const char c = _json[i];

        if (in_string) {
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"') {
            in_string = true;
            continue;
        }

        if (c == '{') {
            if (depth == 0 && current == index) start = i;
            depth++;
        } else if (c == '}') {
            depth--;
            if (depth == 0 && current == index) {
                out.parseJSON(
                    (const uint8_t*)(_json + start),
                    i - start + 1U
                );
                return out;
            }
        } else if (c == ',' && depth == 0) {
            current++;
        }
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
        bool in_string = false;
        bool escape = false;

        do {
            const char c = json[i++];
            if (in_string) {
                if (escape) {
                    escape = false;
                } else if (c == '\\') {
                    escape = true;
                } else if (c == '"') {
                    in_string = false;
                }
                continue;
            }
            if (c == '"') {
                in_string = true;
            } else if (c == '{') {
                depth++;
            } else if (c == '}') {
                depth--;
            }
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

    memset(out, 0, sizeof(*out));

    out->payload_object_size       = (uint32_t)sizeof(Payload);
    out->payload_entry_size        = (uint32_t)sizeof(Payload::Entry);
    out->payload_array_object_size = (uint32_t)sizeof(PayloadArray);
    out->payload_inline_entries    = (uint32_t)Payload::INLINE_ENTRIES;
    out->payload_max_entries       = (uint32_t)Payload::MAX_ENTRIES;
    out->payload_arena_initial     = (uint32_t)Payload::ARENA_INITIAL;
    out->payload_arena_max         = (uint32_t)Payload::ARENA_MAX;

    out->instances_constructed = g_payload_instances_constructed;
    out->instances_destroyed   = g_payload_instances_destroyed;

    out->alive_now             = g_payload_alive_now;
    out->alive_high_water      = g_payload_alive_high_water;

    out->entry_alloc_fail      = g_payload_entry_alloc_fail;
    out->entry_realloc_count   = g_payload_entry_realloc_count;
    out->entry_heap_bytes_alive = g_payload_entry_heap_bytes_alive;
    out->entry_heap_bytes_high_water = g_payload_entry_heap_bytes_high_water;
    out->entry_overflow        = g_payload_entry_overflow;
    out->entry_high_water      = g_payload_entry_high_water_global;
    out->max_entry_capacity_seen = g_payload_max_entry_capacity_seen;

    out->arena_alloc_fail      = g_payload_arena_alloc_fail;
    out->arena_realloc_count   = g_payload_arena_realloc_count;
    out->arena_heap_bytes_alive = g_payload_arena_heap_bytes_alive;
    out->arena_heap_bytes_high_water = g_payload_arena_heap_bytes_high_water;
    out->arena_high_water      = g_payload_arena_high_water_global;
    out->max_arena_capacity_seen = g_payload_max_arena_capacity_seen;

    out->serialize_overflow    = g_payload_serialize_overflow;
    out->to_json_fail          = g_payload_to_json_fail;
    out->string_truncation     = g_payload_string_truncation;
    out->parse_error           = g_payload_parse_error;
    out->integrity_fail        = g_payload_integrity_fail;
    out->invalid_kind          = g_payload_invalid_kind;

    out->last_error_code       = g_payload_last_error_code;
    out->last_error_count      = g_payload_last_error_count;
    out->last_error_this       = g_payload_last_error_this;
    do {
        size_t n = strlen(g_payload_last_error_op);
        if (n >= sizeof(out->last_error_op)) n = sizeof(out->last_error_op) - 1;
        memcpy(out->last_error_op, g_payload_last_error_op, n);
        out->last_error_op[n] = '\0';
    } while (0);
}
