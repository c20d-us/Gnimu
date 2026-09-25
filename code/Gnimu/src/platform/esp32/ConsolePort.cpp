#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "../ConsolePort.h"

namespace gnimu {

// Through the USB-UART bridge Serial is true as soon as begin() returns, and
// anything sent before the host reopens the port after a reset is lost. A fixed
// pause is the only option; waiting on Serial doesn't help.
void consoleOpen() {
  Serial.begin(115200);
  delay(750);
}

}  // namespace gnimu

#endif
