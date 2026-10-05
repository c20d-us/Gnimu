// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: this is native-USB-CDC and TinyUSB
// behavior tied to the chip, not to any one board's pinout, and every board
// in this core that is an nRF52840 uses the same USB peripheral setup.
#if defined(NRF52840_XXAA)

// Serial (USB CDC) on this core needs the TinyUSB library linked, and nothing
// else in the sketch includes it, so library discovery would otherwise skip it.
#include <Adafruit_TinyUSB.h>

#endif
