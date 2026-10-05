// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: see Clock.cpp for why. FICR.DEVICEID
// is a chip register (present on any nRF52840, regardless of board), not a
// board-specific pinout, so this belongs with the chip-generic files.
#if defined(NRF52840_XXAA)

#include <Arduino.h>

#include "../HardwareId.h"

namespace gnimu {

uint64_t hardwareId() {
  return (static_cast<uint64_t>(NRF_FICR->DEVICEID[1]) << 32) | NRF_FICR->DEVICEID[0];
}

}  // namespace gnimu

#endif
