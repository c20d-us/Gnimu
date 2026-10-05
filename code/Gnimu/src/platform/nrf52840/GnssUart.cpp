// The four Seeed XIAO nRF52840 boards (base, Sense, Plus, Sense Plus) share
// one pinout: Serial1 is D6 (TX) / D7 (RX) on all of them, confirmed against
// each board's variant.h. The Wio Tracker 1110 and Tracker T1000 E, built on
// the same chip but a different form factor, use different Serial1 pins
// entirely and are deliberately excluded. This core's own Wire_nRF52.cpp
// groups the same four boards together for the same reason.
#if defined(ARDUINO_Seeed_XIAO_nRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Plus) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)

#include "../GnssUart.h"

namespace gnimu {

// Serial1 is fixed on the XIAO nRF52840: D6 = TX, D7 = RX. Its RX ring is only
// 63 bytes, less than one NAV-PVT, so the drain period matters more here.
Stream& GnssUart::stream() { return Serial1; }

namespace {
// The core's Uart::end() spins until the UARTE reports TX and RX stopped -
// events a never-started (or already-ended) UARTE never raises - so a second
// or unstarted end() would hang the boot. Only end what's open.
bool uartOpen = false;
}  // namespace

void GnssUart::begin(uint32_t baud) {
  Serial1.begin(baud);
  uartOpen = true;
}

void GnssUart::end() {
  if (!uartOpen) return;
  Serial1.end();
  uartOpen = false;
}

}  // namespace gnimu

#endif
