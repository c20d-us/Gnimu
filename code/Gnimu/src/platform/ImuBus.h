#pragma once

#include <stdint.h>

namespace gnimu {

// Brings up the I2C bus the IMU is on, at the given clock. The register-level
// driver (peripherals/imu/Mpu6050.cpp) talks to it through Arduino's global
// Wire, which is portable as-is; only pin selection differs per board.
void imuBusBegin(uint32_t clockHz);

}  // namespace gnimu
