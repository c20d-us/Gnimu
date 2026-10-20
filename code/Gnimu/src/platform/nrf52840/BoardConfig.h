// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Arduino.h>

#include "../../app/ConfigTypes.h"

// The pins and sensor mounting that differ between hardware revisions
// (kHardwareName, kGnssEnablePin, kSwitchSensePin, kLsm6ds3AxisSrc/Sign) come
// from the build's hardware profile in variants/nrf52840/hardware/.
#define GNIMU_HARDWARE_DEFAULT "nrf52840/hardware/v2.h"
#include "../../variants/hardware.h"

namespace gnimu {
namespace board {

// What this board has, fixed at compile time: a LiPo with charger and slide
// switch, and switched power for the GNSS rail and the IMU.
constexpr bool kHasBattery = true;
constexpr bool kHasSwitchedPower = true;

// Drives HIGH to power the onboard LSM6DS3TR-C. Defined identically in every
// XIAO nRF52840 variant.h (even ones without the chip populated), so this
// compiles the same everywhere; only Sense/Sense Plus actually have the chip.
constexpr int kImuPowerPin = PIN_LSM6DS3TR_C_POWER;

static_assert(config::isAxisPermutation(kLsm6ds3AxisSrc),
              "Lsm6ds3 axis remap must be a permutation of 0,1,2");

constexpr float kSwitchOffThresholdMv = 800.0f; // above this = switch OFF

// Serial1 TX, the GNSS receiver's RX: idled low as a GPIO while the GNSS is
// unpowered (see PeripheralPower.cpp).
constexpr int kGnssTxPin = PIN_SERIAL1_TX;

} // namespace board
} // namespace gnimu
