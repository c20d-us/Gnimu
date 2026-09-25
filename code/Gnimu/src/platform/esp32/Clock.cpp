#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <esp_timer.h>

#include "../Clock.h"

namespace gnimu {

void PlatformClock::begin() {}

uint64_t PlatformClock::nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

void PlatformClock::sleepUntilUs(uint64_t deadlineUs) {
  const uint64_t now = nowUs();
  if (deadlineUs <= now) return;
  const uint64_t ms = (deadlineUs - now) / 1000;
  delay(ms < 1 ? 1 : static_cast<uint32_t>(ms));
}

}  // namespace gnimu

#endif
