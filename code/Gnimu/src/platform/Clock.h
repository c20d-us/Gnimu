#pragma once

#include <stdint.h>

namespace gnimu {

// Implemented once per platform in platform/<mcu>/Clock.cpp; only the file
// matching the selected board compiles.
struct PlatformClock {
  // Monotonic microseconds since boot, 64-bit.
  static uint64_t nowUs();
  // Sleep until the deadline without busy-waiting. Granularity is one RTOS
  // tick (1 ms), and it sleeps at least one tick.
  static void sleepUntilUs(uint64_t deadlineUs);
};

}  // namespace gnimu
