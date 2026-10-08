// Two-layer guard. ARDUINO_ARCH_ESP32 is a raw compiler -D flag: safe to
// check before any #include, but shared by the whole family (S3, C3, C6,
// ...). CONFIG_IDF_TARGET_ESP32 is the actual chip-specific macro, but it
// only exists once sdkconfig.h is included - it is NOT a compiler flag, so
// unlike ARDUINO_ARCH_ESP32 it can't gate anything before that #include runs.
// A future variant's platform folder repeats this same two-layer pattern
// with its own chip macro (e.g. CONFIG_IDF_TARGET_ESP32C6).
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>
#include <esp_timer.h>

#include "../Clock.h"

namespace gnimu {

void PlatformClock::begin() {}

// esp_timer already runs from the 40 MHz crystal.
bool PlatformClock::useCrystal() { return true; }

uint64_t PlatformClock::nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

void PlatformClock::sleepUntilUs(uint64_t deadlineUs) {
  const uint64_t now = nowUs();
  if (deadlineUs <= now) return;
  const uint64_t ms = (deadlineUs - now) / 1000;
  delay(ms < 1 ? 1 : static_cast<uint32_t>(ms));
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
