// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: the latter is shared by every board
// this core (and other nRF52 cores, e.g. Adafruit's) supports, including
// nRF52832 boards that lack peripherals this file uses directly (TIMER4 here
// does not exist on an nRF52832). NRF52840_XXAA is Nordic's own chip macro,
// defined by every board in this core that is actually an nRF52840.
#if defined(NRF52840_XXAA)

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

// TIMER4 and the UART baud generators run from HFCLK, which is the internal RC
// oscillator (about +-1.5%) unless something asks for the crystal. USB does,
// so on USB it's already running; on battery nothing else would. With the
// SoftDevice enabled, CLOCK is its peripheral and the request goes through
// it; it then holds the crystal until a reset or System OFF.
bool PlatformClock::useCrystal() {
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);
  if (sdEnabled) return sd_clock_hfclk_request() == NRF_SUCCESS;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  return true;
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
