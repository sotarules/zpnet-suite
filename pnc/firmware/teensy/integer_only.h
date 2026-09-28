#pragma once

// Load the vendor declarations (including their native FP overloads) before
// constraining our definitions. GCC otherwise diagnoses the unused overloads
// inside <cmath> even though the application never calls them.
#if defined(__IMXRT1062__) && defined(__GNUC__)
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

// Integer code can otherwise borrow VFP registers for copies or spills.
// Keep all application definitions, including inline helpers, on core registers.
// The installed Teensy core and libraries retain their own build settings.
#pragma GCC target("general-regs-only")
#endif
