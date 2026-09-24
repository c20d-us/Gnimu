#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>

#include "../Clock.h"

namespace gnimu {

// micros() is 32-bit and wraps every ~71 minutes; extending it relies on
// nowUs() being called more often than that, which the scheduler guarantees.
uint64_t PlatformClock::nowUs() {
  static uint32_t last = 0;
  static uint32_t wraps = 0;
  const uint32_t now = micros();
  if (now < last) wraps++;
  last = now;
  return (static_cast<uint64_t>(wraps) << 32) | now;
}

void PlatformClock::sleepUntilUs(uint64_t deadlineUs) {
  const uint64_t now = nowUs();
  if (deadlineUs <= now) return;
  const uint64_t ms = (deadlineUs - now) / 1000;
  delay(ms < 1 ? 1 : static_cast<uint32_t>(ms));
}

}  // namespace gnimu

#endif
