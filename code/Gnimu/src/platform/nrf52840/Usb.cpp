#if defined(ARDUINO_ARCH_NRF52)

// Serial (USB CDC) on this core needs the TinyUSB library linked, and nothing
// else in the sketch includes it, so library discovery would otherwise skip it.
#include <Adafruit_TinyUSB.h>

#endif
