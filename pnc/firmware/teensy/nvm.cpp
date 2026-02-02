#include "nvm.h"

struct RamEntry {
  bool    used;
  String  key;
  Payload value;
};

static constexpr size_t MAX_RAM_NVM_ENTRIES = 8;
static RamEntry ram_nvm[MAX_RAM_NVM_ENTRIES];

bool nvm_init(void) {
  for (auto& e : ram_nvm) {
    e.used = false;
  }
  return true;
}

static RamEntry* find_entry(const char* key) {
  for (auto& e : ram_nvm) {
    if (e.used && e.key == key) return &e;
  }
  return nullptr;
}

static RamEntry* alloc_entry(const char* key) {
  for (auto& e : ram_nvm) {
    if (!e.used) {
      e.used = true;
      e.key = key;
      e.value.clear();
      return &e;
    }
  }
  return nullptr;
}

bool nvm_exists(const char* key) {
  return find_entry(key) != nullptr;
}

bool nvm_read(const char* key, Payload& out) {
  auto* e = find_entry(key);
  if (!e) return false;
  out = e->value;   // value semantics
  return true;
}

bool nvm_write(const char* key, const Payload& value) {
  auto* e = find_entry(key);
  if (!e) {
    e = alloc_entry(key);
    if (!e) return false;
  }
  e->value = value; // clobber semantics
  return true;
}

bool nvm_delete(const char* key) {
  auto* e = find_entry(key);
  if (!e) return false;
  e->used = false;
  e->key = "";
  e->value.clear();
  return true;
}

void nvm_debug_dump(void) {
  for (auto& e : ram_nvm) {
    if (e.used) {
      debug_log("nvm.ram.key", e.key.c_str());
      debug_log("nvm.ram.value", e.value);
    }
  }
}
