// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: see Clock.cpp for why.
//
// The I2C bring-up the portable MPU-6050 driver needs on an nRF board with an
// external MPU-6050. The XIAO Sense's onboard LSM6DS3TR-C doesn't use it: its
// driver runs on Wire1 through the vendor library.
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
