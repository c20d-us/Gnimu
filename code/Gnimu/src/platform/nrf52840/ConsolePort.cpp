#if defined(ARDUINO_ARCH_NRF52)

#include <Arduino.h>

#include "../ConsolePort.h"

namespace gnimu {

// Native USB CDC: wait for the host to enumerate the port. Classic waited only
// when USB power was present so a battery power-on wasn't delayed; there is no
// power module here yet, so for now this waits the full 3 s with no host.
void consoleOpen() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {
  }
}

}  // namespace gnimu

#endif
