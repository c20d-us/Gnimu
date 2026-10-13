// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "BatteryMonitor.h"
#include "DeviceStatus.h"
#include "GnssEpoch.h"
#include "GnssNames.h"
#include "TextOut.h"

namespace gnimu {

// The periodic (1 Hz) status line: one terse row for watching the console
// scroll by, as opposed to the on-request `D` report's full table. Pipe-
// delimited "Label: value" cells.
// The status line's per-line rates: events since the previous line, as
// RateTracker measures them. Hz * 100, displayed rounded to a whole Hz.
struct StatusRates {
  uint32_t bleCentiHz;   // packets sent
  uint32_t gnssCentiHz;  // epochs received
};

// The longest status line, newline included, plus its terminator. The console
// sends a line whole or not at all, with the newline as CR+LF, and the nRF's
// USB transmit FIFO is 256 bytes: 254 bytes of text plus CR+LF fills it
// exactly, and a longer line could never be sent there. test_statusline
// checks the worst case fits.
constexpr size_t kStatusLineMaxBytes = 256;

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

inline size_t formatStatus(char *buf, size_t cap, const DeviceStatus &d, const StatusRates &r) {
  detail::Out o{buf, cap, 0};
  if (cap)
    buf[0] = '\0';

  const unsigned long uptimeS =
      static_cast<unsigned long>(d.uptimeUs / 1000000ULL);
  const unsigned long bleHz =
      (static_cast<unsigned long>(r.bleCentiHz) + 50) / 100;
  const unsigned long gnssHz =
      (static_cast<unsigned long>(r.gnssCentiHz) + 50) / 100;
  o.put("RT: %lus | BLE: %luHz | GNSS: %luHz | ", uptimeS, bleHz, gnssHz);
  if (d.power.battery && d.power.battery->valid) {
    // Volts to hundredths, integer-only like everything else here.
    const unsigned long centiV =
        static_cast<unsigned long>(d.power.battery->voltage * 100.0f + 0.5f);
    const bool charging = isCharging(*d.power.battery, d.power.usbPresent);
    o.put("Batt: %lu.%02luV %u%%%s | ", centiV / 100, centiV % 100,
          static_cast<unsigned>(d.power.battery->percent),
          charging ? " \xE2\x9A\xA1" : "");
  }
  if (d.gnss.latest) {
    char tAcc[16];
    char hAcc[16];
    formatAccuracy(tAcc, sizeof(tAcc), d.gnss.latest->tAccNs, "ns");
    formatAccuracy(hAcc, sizeof(hAcc), d.gnss.latest->hAccMm, "mm");
    o.put("Fix: %s | SV: %u | tAcc: %s | hAcc: %s | ",
          fixTypeShortName(d.gnss.latest->fixType),
          static_cast<unsigned>(d.gnss.latest->numSv), tAcc, hAcc);
    detail::putDegrees1e7(o, "Lat", d.gnss.latest->latDeg1e7);
    detail::putDegrees1e7(o, "Long", d.gnss.latest->lonDeg1e7);
  } else {
    o.put("Fix: none | ");
  }
  const ImuDisplay imu = toDisplayUnits(d.imu.sample);
  o.put("Accel mG: %ld %ld %ld | Gyro c°/s: %ld %ld %ld | ",
        static_cast<long>(imu.accelMilliG[0]),
        static_cast<long>(imu.accelMilliG[1]),
        static_cast<long>(imu.accelMilliG[2]),
        static_cast<long>(imu.gyroCentiDegS[0]),
        static_cast<long>(imu.gyroCentiDegS[1]),
        static_cast<long>(imu.gyroCentiDegS[2]));
  // Same rounded-to-tenths, integer-only tilt conversion as Report.h.
  const unsigned long tiltTenths =
      (static_cast<unsigned long>(tiltCentiDeg(d.imu.tiltDegrees)) + 5) / 10;
  o.put("Trim: %lu.%lu° %s\n", tiltTenths / 10, tiltTenths % 10,
        d.imu.trimLocked ? "🔒" : "⏳");
  return o.len;
}

} // namespace gnimu
