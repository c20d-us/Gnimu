// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: this is native-USB-CDC behavior
// tied to the chip, not to any one board's pinout, and every board in this
// core that is an nRF52840 uses the same USB peripheral setup.
#if defined(NRF52840_XXAA)

#include <Arduino.h>

#include "../ConsolePort.h"
#include "../Watchdog.h"

namespace gnimu {

// Native USB CDC: wait for the host to enumerate the port, but only with USB
// power present - on battery there's no host, and waiting
// would only delay every power-on.
void consoleOpen() {
  Serial.begin(115200);
  if ((NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) == 0) return;
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {
    Watchdog::feed();
  }
}

}  // namespace gnimu

#endif
