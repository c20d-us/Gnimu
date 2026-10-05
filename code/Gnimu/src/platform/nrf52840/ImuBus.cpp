// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: see Clock.cpp for why.
//
// No board in hand wires an IMU here yet - the XIAO Sense's onboard
// LSM6DS3TR-C needs its own driver, not this MPU-6050 one - but Wire.begin()
// is harmless with nothing answering, and this is the same bus bring-up that
// driver will need, so it's written once now rather than as a throwaway stub.
#if defined(NRF52840_XXAA)

#include <Wire.h>

#include "../ImuBus.h"

namespace gnimu {

void imuBusBegin(uint32_t clockHz) {
  Wire.begin();
  Wire.setClock(clockHz);
}

}  // namespace gnimu

#endif
