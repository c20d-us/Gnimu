// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace gnimu {
namespace board {

// Classic: a device built from the Classic nRF52840 instructions. Same
// circuits as V2; two signals sit on different XIAO pins, and the XIAO faces
// the other way in the case.
constexpr const char* kHardwareName = "Classic";

// EN of the regulator that powers only the GNSS module (TPS63020). Same
// behaviour as on V2.
constexpr int kGnssEnablePin = D9;

// The slide switch's spare throw through a 510k/510k divider, read through
// the ADC. Same circuit as V2.
constexpr int kSwitchSensePin = A4;

// Sense IMU sensor frame -> vehicle frame: the XIAO faces the opposite way
// to V2, which flips X and Y. Same meaning as in v2.h.
constexpr uint8_t kLsm6ds3AxisSrc[3] = {0, 1, 2};
constexpr float kLsm6ds3AxisSign[3] = {-1.0f, -1.0f, +1.0f};

}  // namespace board
}  // namespace gnimu
