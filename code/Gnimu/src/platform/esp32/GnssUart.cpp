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

#include "../GnssUart.h"
#include "BoardConfig.h"

namespace gnimu {

static HardwareSerial uart(2);

Stream& GnssUart::stream() { return uart; }

void GnssUart::begin(uint32_t baud) {
  uart.setRxBufferSize(board::kGnssRxBufferBytes);
  uart.begin(baud, SERIAL_8N1, board::kGnssRxPin, board::kGnssTxPin);
}

void GnssUart::end() { uart.end(); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
