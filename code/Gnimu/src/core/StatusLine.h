#pragma once

#include <stdint.h>

#include "BatteryMonitor.h"
#include "GnssEpoch.h"
#include "TextOut.h"

namespace gnimu {

// The periodic (1 Hz) status line: one terse row for watching the console
// scroll by, as opposed to the on-request `D` report's full table. Pipe-
// delimited "Label: value" cells, matching Classic's console line style
// rather than a comma key=value format.
struct StatusView {
  uint64_t uptimeUs;
  // Hz * 100 internally (see RateTracker) - packets sent / epochs received
  // since the last status line, not cumulative counts - but displayed rounded
  // to a whole Hz. Integer throughout, no float format specifier, same
  // reasoning as Report.h.
  uint32_t bleCentiHz;
  uint32_t gnssCentiHz;
  const BatteryStatus *battery; // nullptr on boards without one (the ESP32)
  bool usbPresent;
  const GnssEpoch
      *latest; // nullptr until the first epoch, same convention as ReportView
  // Pre-converted by the caller (milli-g, centi-deg/s), same reasoning as
  // ReportView: Report/StatusLine stay free of float format specifiers.
  int32_t imuAccelMilliG[3];
  int32_t imuGyroCentiDegS[3];
  bool trimConverged;
  int32_t trimTiltCentiDeg; // pre-converted by the caller, same as Report.h
};

namespace detail {
// Degrees from a PVT-native deg*1e-7 fixed-point value, full native
// precision (no rounding - this isn't a made-up decimal count, it's exactly
// what the receiver reports). Handles the sign itself since %07lu can't.
inline void putDegrees1e7(Out &o, const char *label, int32_t v1e7) {
  const bool neg = v1e7 < 0;
  // Negated as unsigned: -v1e7 would overflow (undefined) for INT32_MIN.
  const uint32_t mag = neg ? 0u - static_cast<uint32_t>(v1e7) : static_cast<uint32_t>(v1e7);
  o.put("%s: %s%lu.%07lu | ", label, neg ? "-" : "",
        static_cast<unsigned long>(mag / 10000000UL),
        static_cast<unsigned long>(mag % 10000000UL));
}
} // namespace detail

inline size_t formatStatus(char *buf, size_t cap, const StatusView &v) {
  detail::Out o{buf, cap, 0};
  if (cap)
    buf[0] = '\0';

  const unsigned long uptimeS =
      static_cast<unsigned long>(v.uptimeUs / 1000000ULL);
  const unsigned long bleHz =
      (static_cast<unsigned long>(v.bleCentiHz) + 50) / 100;
  const unsigned long gnssHz =
      (static_cast<unsigned long>(v.gnssCentiHz) + 50) / 100;
  o.put("RT: %lus | BLE: %luHz | GNSS: %luHz | ", uptimeS, bleHz, gnssHz);
  if (v.battery && v.battery->valid) {
    // Volts to hundredths, integer-only like everything else here.
    const unsigned long centiV =
        static_cast<unsigned long>(v.battery->voltage * 100.0f + 0.5f);
    // The charger also reads active only with USB present, but the explicit
    // check keeps a floating status line from ever claiming a charge.
    const bool charging = v.usbPresent && v.battery->chargerActive;
    o.put("Batt: %lu.%02luV %u%%%s | ", centiV / 100, centiV % 100,
          static_cast<unsigned>(v.battery->percent),
          charging ? " \xE2\x9A\xA1" : "");
  }
  if (v.latest) {
    o.put("Fix: %u | SV: %u | tAcc: %luns | hAcc: %lumm | ",
          static_cast<unsigned>(v.latest->fixType),
          static_cast<unsigned>(v.latest->numSv),
          static_cast<unsigned long>(v.latest->tAccNs),
          static_cast<unsigned long>(v.latest->hAccMm));
    detail::putDegrees1e7(o, "Lat", v.latest->latDeg1e7);
    detail::putDegrees1e7(o, "Long", v.latest->lonDeg1e7);
  } else {
    o.put("Fix: none | ");
  }
  o.put("Accel mG: %ld %ld %ld | Gyro c°/s: %ld %ld %ld | ",
        static_cast<long>(v.imuAccelMilliG[0]),
        static_cast<long>(v.imuAccelMilliG[1]),
        static_cast<long>(v.imuAccelMilliG[2]),
        static_cast<long>(v.imuGyroCentiDegS[0]),
        static_cast<long>(v.imuGyroCentiDegS[1]),
        static_cast<long>(v.imuGyroCentiDegS[2]));
  // Same rounded-to-tenths, integer-only tilt conversion as Report.h.
  const unsigned long tiltTenths =
      (static_cast<unsigned long>(v.trimTiltCentiDeg) + 5) / 10;
  o.put("Trim: %lu.%lu° %s\n", tiltTenths / 10, tiltTenths % 10,
        v.trimConverged ? "🔒" : "⏳");
  return o.len;
}

} // namespace gnimu
