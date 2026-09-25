#if defined(ARDUINO_ARCH_ESP32)

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

#endif
