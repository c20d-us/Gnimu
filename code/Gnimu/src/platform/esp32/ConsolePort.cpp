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

#include "driver/uart.h"

#include "../ConsolePort.h"

namespace gnimu {

// Through the USB-UART bridge Serial is true as soon as begin() returns, and
// anything sent before the host reopens the port after a reset is lost. A fixed
// pause is the only option; waiting on Serial doesn't help.
void consoleOpen() {
  // The core's default is no TX buffer, only the UART's 128-byte FIFO, so any
  // longer print blocks the loop until it's on the wire (~15 ms for a status
  // line, ~95 ms for D at 115200). With a buffer, writes return at once and
  // the UART interrupt drains it. Must be set before begin().
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(750);
}

// Serial is UART0. Not Serial.flush(false): that also spins until the
// transmitter is idle before clearing input.
void consoleDiscardInput() { uart_flush_input(UART_NUM_0); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
