// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace gnimu {
namespace board {

// V2: the current hardware revision, used by the carrier PCB and the build
// guide. As the console's V command shows it.
constexpr const char* kHardwareName = "V2";

// EN of the regulator that powers only the GNSS module (TPS63070). LOW
// disables it; released to an input, the regulator's own pull-up enables it.
constexpr int kGnssEnablePin = D1;

// The slide switch's spare throw through a 510k/510k divider: ~2 V switch
// OFF, ~0 V switch ON, read through the ADC.
constexpr int kSwitchSensePin = A0;

// Sense IMU (LSM6DS3TR-C) sensor frame -> vehicle frame. The chip is fixed to
// the XIAO, so the map follows which way the XIAO faces: here its USB
// connector toward the rear of the vehicle, verified on hardware. Output
// axis i is input axis src[i] times sign[i]; src must be a permutation of
// 0, 1, 2.
constexpr uint8_t kLsm6ds3AxisSrc[3] = {0, 1, 2};
constexpr float kLsm6ds3AxisSign[3] = {+1.0f, +1.0f, +1.0f};

}  // namespace board
}  // namespace gnimu
