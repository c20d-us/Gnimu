#if defined(ARDUINO_ARCH_NRF52)

#include "../GnssUart.h"

namespace gnimu {

// Serial1 is fixed on the XIAO nRF52840: D6 = TX, D7 = RX. Its RX ring is only
// 63 bytes, less than one NAV-PVT, so the drain period matters more here.
Stream& GnssUart::stream() { return Serial1; }

void GnssUart::begin(uint32_t baud) { Serial1.begin(baud); }

void GnssUart::end() { Serial1.end(); }

}  // namespace gnimu

#endif
