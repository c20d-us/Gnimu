#pragma once

#include <stdint.h>

namespace gnimu {

// u-blox NAV-PVT fixType in words, for the D report.
inline const char* fixTypeName(uint8_t fixType) {
  switch (fixType) {
    case 0:
      return "no fix";
    case 1:
      return "dead reckoning";
    case 2:
      return "2D fix";
    case 3:
      return "3D fix";
    case 4:
      return "GNSS + dead reckoning";
    case 5:
      return "time only";
    default:
      return "unknown fix";
  }
}

// The same, short, for the status line's Fix cell.
inline const char* fixTypeShortName(uint8_t fixType) {
  switch (fixType) {
    case 0:
      return "none";
    case 1:
      return "DR";
    case 2:
      return "2D";
    case 3:
      return "3D";
    case 4:
      return "GNSS+DR";
    case 5:
      return "time";
    default:
      return "?";
  }
}

// u-blox CFG-NAVSPG-DYNMODEL values in words. 1 is reserved.
inline const char* dynamicModelName(uint8_t model) {
  switch (model) {
    case 0:
      return "portable";
    case 2:
      return "stationary";
    case 3:
      return "pedestrian";
    case 4:
      return "automotive";
    case 5:
      return "sea";
    case 6:
      return "airborne <1g";
    case 7:
      return "airborne <2g";
    case 8:
      return "airborne <4g";
    case 9:
      return "wrist";
    case 10:
      return "bike";
    case 11:
      return "mower";
    case 12:
      return "e-scooter";
    default:
      return "unknown model";
  }
}

}  // namespace gnimu
