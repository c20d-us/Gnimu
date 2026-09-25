#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>

#include "../Clock.h"

namespace gnimu {

// The core's micros() has only 1 ms resolution (RTOS tick), so TIMER4 runs as
// a free 32-bit 1 MHz counter instead. It wraps every ~71 minutes; nowUs()
// extends it to 64 bits and must be called more often than that, which the
// scheduler guarantees.
void PlatformClock::begin() {
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER4->PRESCALER = 4;  // 16 MHz / 2^4 = 1 MHz
  NRF_TIMER4->TASKS_CLEAR = 1;
  NRF_TIMER4->TASKS_START = 1;
}

uint64_t PlatformClock::nowUs() {
  static uint32_t last = 0;
  static uint32_t wraps = 0;
  NRF_TIMER4->TASKS_CAPTURE[0] = 1;
  const uint32_t now = NRF_TIMER4->CC[0];
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
