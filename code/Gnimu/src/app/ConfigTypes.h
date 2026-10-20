// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {
namespace config {

// The choices the settings in Config.h (next to Gnimu.ino) are written in.

// Which BLE protocol a build speaks; app/protocol/Protocol.h picks the class.
enum class Protocol : uint8_t { RaceBox };

// GNSS constellations. QZSS and SBAS are never enabled.
enum class GnssSystem : uint8_t { Gps, Galileo, Beidou, Glonass };
constexpr const char* gnssSystemName(GnssSystem s) {
  return s == GnssSystem::Gps       ? "GPS"
         : s == GnssSystem::Galileo ? "Galileo"
         : s == GnssSystem::Beidou  ? "BeiDou"
                                    : "GLONASS";
}

// True if src holds each of 0, 1, 2 once: an axis remap that is a rotation or
// reflection of the sensor, not a collapse of two axes onto one.
constexpr bool isAxisPermutation(const uint8_t src[3]) {
  return src[0] < 3 && src[1] < 3 && src[2] < 3 && src[0] != src[1] &&
         src[0] != src[2] && src[1] != src[2];
}

}  // namespace config
}  // namespace gnimu
