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

}  // namespace config
}  // namespace gnimu
