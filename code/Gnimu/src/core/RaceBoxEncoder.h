#pragma once

#include <stddef.h>
#include <stdint.h>

#include "GnssEpoch.h"
#include "ImuSample.h"

namespace gnimu {

// RaceBox Data Message: UBX-style framing (B5 62, class FF, id 01, length 80),
// an 80-byte little-endian payload, and the UBX Fletcher checksum, laid out as
// the RaceBox BLE protocol's data message.
constexpr size_t kRaceBoxPacketBytes = 88;

namespace detail {

inline void putLe(uint8_t *p, uint32_t v, size_t bytes) {
  for (size_t i = 0; i < bytes; i++) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

// Saturates rather than wrapping: a wrapped overflow flips sign, reporting a
// hard left spin as a right one. The gyro can exceed int16 at +/-500 deg/s.
inline int16_t toProtocolInt16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return static_cast<int16_t>(v);
}

}  // namespace detail

// `imu` is the vehicle-frame, filtered sample; `battery` is the protocol's
// battery byte (bit 7 charging, bits 0-6 percent).
inline void encodeRaceBoxData(const GnssEpoch &e, const ImuSample &imu, uint8_t battery,
                              uint8_t (&out)[kRaceBoxPacketBytes]) {
  using detail::putLe;
  for (size_t i = 0; i < kRaceBoxPacketBytes; i++) out[i] = 0;
  uint8_t *p = out + 6;

  putLe(p + 0, e.iTowMs, 4);
  putLe(p + 4, e.year, 2);
  p[6] = e.month;
  p[7] = e.day;
  p[8] = e.hour;
  p[9] = e.min;
  p[10] = e.sec;
  p[11] = static_cast<uint8_t>((e.validDate ? 1 : 0) | (e.validTime ? 2 : 0) |
                               (e.fullyResolved ? 4 : 0) | (e.validMag ? 8 : 0));
  putLe(p + 12, e.tAccNs, 4);
  putLe(p + 16, static_cast<uint32_t>(e.nanoNs), 4);

  // The protocol only defines 0 (none), 2 (2D) and 3 (3D); other u-blox fix
  // types (dead reckoning, time-only, ...) are reported as no fix.
  const uint8_t fix = (e.fixType == 2 || e.fixType == 3) ? e.fixType : 0;
  p[20] = fix;
  p[21] = static_cast<uint8_t>(((e.fixType == 3 && e.gnssFixOk) ? 0x01 : 0) |
                               (e.headVehValid ? 0x20 : 0));
  p[22] = static_cast<uint8_t>((e.validTime ? 0x20 : 0) | (e.validDate ? 0x40 : 0) |
                               ((e.validTime && e.fullyResolved) ? 0x80 : 0));
  p[23] = e.numSv;

  putLe(p + 24, static_cast<uint32_t>(e.lonDeg1e7), 4);
  putLe(p + 28, static_cast<uint32_t>(e.latDeg1e7), 4);
  putLe(p + 32, static_cast<uint32_t>(e.heightMm), 4);
  putLe(p + 36, static_cast<uint32_t>(e.hMslMm), 4);
  putLe(p + 40, e.hAccMm, 4);
  putLe(p + 44, e.vAccMm, 4);
  putLe(p + 48, e.gSpeedMmPerS, 4);
  putLe(p + 52, static_cast<uint32_t>(e.headMotDeg1e5), 4);
  putLe(p + 56, e.sAccMmPerS, 4);
  putLe(p + 60, e.headAccDeg1e5, 4);
  putLe(p + 64, e.pDop, 2);
  p[66] = fix == 0 ? 0x01 : 0;  // invalid lat/lon/heights, agrees with offset 20
  p[67] = battery;

  for (int i = 0; i < 3; i++) {
    putLe(p + 68 + 2 * i,
          static_cast<uint16_t>(detail::toProtocolInt16(imu.accel[i] * kMps2ToMilliG)), 2);
    putLe(p + 74 + 2 * i,
          static_cast<uint16_t>(detail::toProtocolInt16(imu.gyro[i] * kRadpsToCentiDeg)), 2);
  }

  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = 0xFF;  // class: RaceBox data message
  out[3] = 0x01;  // id
  out[4] = 80;    // payload length, little-endian
  out[5] = 0;

  // UBX Fletcher-8 over class, id, length and payload - i.e. bytes 2..85.
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + 80; i++) {
    a = static_cast<uint8_t>(a + out[i]);
    b = static_cast<uint8_t>(b + a);
  }
  out[86] = a;
  out[87] = b;
}

}  // namespace gnimu
