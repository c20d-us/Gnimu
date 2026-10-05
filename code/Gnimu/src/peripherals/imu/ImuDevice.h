#pragma once

// Which concrete IMU class App wires up, resolved at compile time - no
// virtual dispatch. ARDUINO_ARCH_ESP32 here (not the two-layer chip-specific
// guard some platform/esp32 files need): Mpu6050 is a builder-wired external
// sensor over the portable Wire API, not tied to one exact chip in the ESP32
// family. NRF52840_XXAA: the Sense's onboard chip is a fixed hardware fact of
// that chip, not a pinout choice, matching Clock/ConsolePort/Target's guard.
#if defined(ARDUINO_ARCH_ESP32)

#include "Mpu6050.h"
namespace gnimu {
using ImuDevice = Mpu6050;
}

#elif defined(NRF52840_XXAA)

#include "Lsm6ds3.h"
namespace gnimu {
using ImuDevice = Lsm6ds3;
}

#endif
