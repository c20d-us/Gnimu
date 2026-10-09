// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: same reasoning as Clock.cpp.
#if defined(NRF52840_XXAA)

#include <Arduino.h>

#include "../../app/SystemConfig.h"
#include "../Watchdog.h"

namespace gnimu {

// Keeps counting while the CPU sleeps between loop passes, pauses while a
// debugger halts it. Its registers lock once it's running, so after a soft
// reset these writes are ignored and the earlier boot's identical settings
// stay in force.
bool Watchdog::start() {
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                    (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
  NRF_WDT->CRV = config::kWatchdogMs * 32768ULL / 1000 - 1;  // 32.768 kHz ticks
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
  feed();
  return NRF_WDT->RUNSTATUS != 0;
}

void Watchdog::feed() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }

}  // namespace gnimu

#endif
