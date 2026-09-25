#pragma once

#include <stdint.h>

namespace gnimu {
namespace config {

constexpr uint32_t kGnssBaud = 115200;
constexpr uint8_t kGnssNavRateHz = 20;  // 20 is the max with 2 constellations enabled
constexpr uint8_t kGnssMinElevDeg = 5;  // ignore SVs below this angle (anti-multipath)
constexpr uint8_t kGnssDynamicModel = 4;  // u-blox dynamic model 4 = automotive

// Constellations, most preferred first; the first kConstellationCount are
// enabled. 20 Hz needs at most 2. QZSS and SBAS are never enabled.
enum class GnssSystem : uint8_t { Gps, Galileo, Beidou, Glonass };
constexpr GnssSystem kConstellationPriority[] = {GnssSystem::Gps, GnssSystem::Galileo,
                                                 GnssSystem::Beidou, GnssSystem::Glonass};
constexpr unsigned kConstellationCount = 2;

// The drain period is also the longest the loop ever sleeps. The UART RX ring
// must hold more than one period of data, with plenty of margin.
constexpr uint32_t kGnssDrainPeriodUs = 1000;
constexpr uint32_t kGnssDrainBudgetUs = 3000;
constexpr uint32_t kConsolePeriodUs = 10000;
constexpr uint32_t kConsoleBudgetUs = 2000;

static_assert(kGnssBaud == 9600 || kGnssBaud == 38400 || kGnssBaud == 57600 ||
                  kGnssBaud == 115200 || kGnssBaud == 230400 || kGnssBaud == 460800,
              "kGnssBaud must be a rate the baud sweep can find");
static_assert(kGnssNavRateHz > 0 && kGnssNavRateHz <= 25, "nav rate must be 1-25 Hz");
static_assert(kGnssMinElevDeg <= 90, "min elevation must be 0-90 degrees");
static_assert(kConstellationCount >= 1 && kConstellationCount <= 4,
              "constellation count must be 1-4");
static_assert(kConstellationPriority[0] != kConstellationPriority[1] &&
                  kConstellationPriority[0] != kConstellationPriority[2] &&
                  kConstellationPriority[0] != kConstellationPriority[3] &&
                  kConstellationPriority[1] != kConstellationPriority[2] &&
                  kConstellationPriority[1] != kConstellationPriority[3] &&
                  kConstellationPriority[2] != kConstellationPriority[3],
              "constellation priority must list each system exactly once");
static_assert(kGnssDrainPeriodUs > 0, "drain period must be positive");

}  // namespace config
}  // namespace gnimu
