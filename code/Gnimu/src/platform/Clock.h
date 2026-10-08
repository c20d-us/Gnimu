#pragma once

#include <stdint.h>

namespace gnimu {

// Implemented once per platform in platform/<mcu>/Clock.cpp; only the file
// matching the selected board compiles.
struct PlatformClock {
  // Starts the timebase; call once before the first nowUs().
  static void begin();
  // Runs the timebase (and on the nRF the UARTs too) from the crystal until
  // the next reset. Call once, after BLE has started. False if the request
  // was refused.
  static bool useCrystal();
  // Monotonic microseconds since boot, 64-bit. Loop task only: on the nRF it
  // isn't safe against another task calling it at the same moment. Every
  // caller today runs on the loop task - the scheduler and its jobs,
  // Diagnostics, Console, BLE send() and the GNSS PVT callback (which runs
  // inside Gnss::poll()); no BLE stack callback uses it.
  static uint64_t nowUs();
  // Sleep until the deadline without busy-waiting. Granularity is one RTOS
  // tick (1 ms), and it sleeps at least one tick.
  static void sleepUntilUs(uint64_t deadlineUs);
};

}  // namespace gnimu
