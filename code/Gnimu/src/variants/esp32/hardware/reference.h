// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {
namespace board {

// The reference build: the ESP32 build guide's wiring and mounting. As the
// console's V command shows it.
constexpr const char* kHardwareName = "Reference";

// GNSS receiver UART.
constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;

// The core's I2C defaults; an external MPU-6050 is wired here.
constexpr int kImuSdaPin = 21;
constexpr int kImuSclPin = 22;

// The dev board's onboard LED: single color, blue.
constexpr int kStatusLedPin = 2;

// MPU-6050 sensor frame -> vehicle frame. The module is separate from the
// ESP32, so this follows how it is mounted. The reference build is a GY-521
// aligned to the axis marks on its silkscreen: mounting holes toward the
// front of the vehicle, header pins toward the rear, PCB face up. Output axis
// i is input axis src[i] times sign[i]; src must be a permutation of 0, 1, 2.
constexpr uint8_t kMpu6050AxisSrc[3] = {0, 1, 2};
constexpr float kMpu6050AxisSign[3] = {+1.0f, +1.0f, +1.0f};

}  // namespace board
}  // namespace gnimu
