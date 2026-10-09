// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Gnimu settings: the values chosen for a build. Each one is checked when the
// firmware compiles (src/app/SystemConfig.h), so a value out of range fails
// the build with a message saying why. The constants the firmware's timing
// and safety depend on are in src/app/SystemConfig.h, not here.

#include <stdint.h>

#include "src/app/ConfigTypes.h"

namespace gnimu {
namespace config {

// Build

// Which BLE protocol this build speaks: one binary per protocol. The
// protocol, not the board, decides whether the IMU is sampled at all; a
// RaceChrono build ignores even an onboard IMU. Choices: RaceBox.
constexpr Protocol kProtocol = Protocol::RaceBox;

// GNSS

// The baud rate for GNSS communications. Higher than 115200 is not needed.
// Must be one of the standard rates the detection sweep tries.
constexpr uint32_t kGnssBaud = 115200;
// Navigation rate, 1-25 Hz. 20 Hz is the max with 2 constellations.
constexpr uint8_t kGnssNavRateHz = 20;
// Satellites below this elevation are ignored, 0-90 degrees.
constexpr uint8_t kGnssMinElevDeg = 5;
// Constellations, most preferred first, each listed once (Gps, Galileo,
// Beidou, Glonass); the first kConstellationCount are enabled.
constexpr GnssSystem kConstellationPriority[] = {
    GnssSystem::Gps, GnssSystem::Galileo, GnssSystem::Beidou,
    GnssSystem::Glonass};
// How many constellations to enable, 1-4. 20 Hz needs at most 2.
constexpr unsigned kConstellationCount = 2;

// BLE

// BLE Tx power, low on purpose: less RF near the GNSS antenna, and the client
// is usually close. Raise it if connections drop. Each board validates the
// value against the levels its own radio supports.
constexpr int kBleTxPowerDbm = -12;

// IMU

// EMA smoothing, 1.0 = raw. Its delay is about (1 - alpha) / alpha samples:
// 0.33 is about 20 ms at 100 Hz, so a packet's IMU value lands near the epoch
// it's sent with, while still damping vibration that would alias into the
// 20 Hz packets. Heavier smoothing trades lag for calm: 0.18 is about 45 ms,
// and 0.09 about 100 ms, which shifts braking and turn-in by about 3 m at
// 30 m/s.
constexpr float kImuAccelAlpha = 0.33f;
constexpr float kImuGyroAlpha = 0.33f;

// Sensor frame -> vehicle frame, one per chip: the external MPU-6050's mount
// is a builder/BOM choice (currently identity), while the Sense's onboard
// LSM6DS3TR-C has a fixed orientation on that PCB (signs -1,-1,+1). Output
// axis i is input axis src[i] times sign[i]; src must be a permutation of
// 0, 1, 2.
constexpr uint8_t kMpu6050AxisSrc[3] = {0, 1, 2};
constexpr float kMpu6050AxisSign[3] = {+1.0f, +1.0f, +1.0f};
constexpr uint8_t kLsm6ds3AxisSrc[3] = {0, 1, 2};
constexpr float kLsm6ds3AxisSign[3] = {-1.0f, -1.0f, +1.0f};

// Battery, for builds that have one

// Cell voltages, ordered cutoff < critical < warning < full scale <= 4.2 V.
// Percent is a straight line from cutoff (0%) to full scale (100%).
constexpr float kBatteryCutoffV = 3.30f;     // System OFF below this, on battery
constexpr float kBatteryCriticalV = 3.35f;   // red blink
constexpr float kBatteryWarningV = 3.40f;    // amber blink
constexpr float kBatteryFullScaleV = 4.10f;  // 100%
constexpr float kBatteryChargeFullV = 4.10f; // Rough value for "full"

}  // namespace config
}  // namespace gnimu
