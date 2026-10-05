// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: the latter is shared by every board
// this core (and other nRF52 cores, e.g. Adafruit's) supports, including
// nRF52832 boards. NRF52840_XXAA is Nordic's own chip macro, defined by every
// board in this core that is actually an nRF52840.
#if defined(NRF52840_XXAA)

#include "../Target.h"

namespace gnimu {

const char* targetName() { return "nrf52840"; }

}  // namespace gnimu

#endif
