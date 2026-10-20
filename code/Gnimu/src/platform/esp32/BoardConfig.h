// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>

#include "../../app/ConfigTypes.h"

// The pins and sensor mounting that depend on how the board is wired
// (kHardwareName, kGnssRxPin/TxPin, kImuSdaPin/SclPin, kStatusLedPin,
// kMpu6050AxisSrc/Sign) come from the build's hardware profile in
// variants/esp32/hardware/.
#define GNIMU_HARDWARE_DEFAULT "esp32/hardware/reference.h"
#include "../../variants/hardware.h"

namespace gnimu {
namespace board {

// What this board has, fixed at compile time: USB-powered, peripherals wired
// straight to 3.3 V.
constexpr bool kHasBattery = false;
constexpr bool kHasSwitchedPower = false;

static_assert(config::isAxisPermutation(kMpu6050AxisSrc),
              "Mpu6050 axis remap must be a permutation of 0,1,2");

// The stock 256-byte ring fills in ~22 ms at 115200 baud. 512 doubles that.
constexpr size_t kGnssRxBufferBytes = 512;

}  // namespace board
}  // namespace gnimu
