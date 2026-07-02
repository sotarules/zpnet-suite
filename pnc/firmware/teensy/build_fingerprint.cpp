// build_fingerprint.cpp
//
// Immutable build fingerprint.
// This translation unit exists solely to make the compiled
// firmware artifact self-describing and auditable.
//
// The fingerprint is:
//   • compile-time only
//   • allocator-free
//   • linker-visible
//   • flash-resident (.rodata)
//   • verifiable via HEX / ELF inspection
//

#include "config.h"
#include <stdint.h>

extern "C" {

// -----------------------------------------------------------------------------
// Immutable build fingerprint string
// -----------------------------------------------------------------------------

__attribute__((used, section(".rodata")))
const char ZPNET_BUILD_FINGERPRINT[] =
  "ZPNET_BUILD{"
  "TRANSPORT=SERIAL;"
  "USB=CDC;"
  "FW_VERSION=" FW_VERSION_STR ";"
  "}";

} // extern "C"
