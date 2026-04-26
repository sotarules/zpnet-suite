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

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  "TRANSPORT=SERIAL;"
#elif defined(ZPNET_TRANSPORT_SELECTED_HID)
  "TRANSPORT=HID;"
#else
  "TRANSPORT=UNKNOWN;"
#endif

#if defined(USB_SERIAL)
  "USB=CDC;"
#elif defined(USB_RAWHID)
  "USB=RAWHID;"
#elif defined(USB_SERIAL_HID)
  "USB=CDC+HID;"
#elif defined(USB_RAWHID_SERIAL)
  "USB=RAWHID+CDC;"
#else
  "USB=UNKNOWN;"
#endif

  "}";

} // extern "C"
