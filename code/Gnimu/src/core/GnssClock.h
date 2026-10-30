// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gnimu {

// The M10's CPU clock: the four CFG-SPG clock keys, in the order the OTP poll
// requests them, and the values the high-performance setting holds.
constexpr uint32_t kGnssClockKeys[4] = {0x40A40001UL, 0x40A40003UL, 0x40A40005UL, 0x40A4000AUL};
constexpr uint32_t kGnssHighClockHz[4] = {192000000UL, 192000000UL, 192000000UL, 96000000UL};

enum class GnssClockState : uint8_t {
  Unchecked,  // not read yet this boot
  Unknown,    // the receiver could not be read
  Stock,      // nothing in OTP, running the stock clock
  Pending,    // high clock in OTP, not what the receiver is running yet
  Active,     // running the high clock
};

// Little-endian, as UBX carries every field.
inline uint32_t clockLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 |
         static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24;
}

// True if a UBX-CFG-VALGET reply payload from the OTP layer holds all four
// clock keys at their high values. The payload is a 4-byte header (version,
// layer, position) then a key and a 4-byte value per entry, in request order.
inline bool otpHoldsHighClock(const uint8_t* payload, size_t len) {
  constexpr size_t kHeader = 4, kEntry = 8;
  if (len < kHeader + 4 * kEntry)
    return false;
  for (size_t i = 0; i < 4; i++) {
    const uint8_t* entry = payload + kHeader + i * kEntry;
    if (clockLe32(entry) != kGnssClockKeys[i] || clockLe32(entry + 4) != kGnssHighClockHz[i])
      return false;
  }
  return true;
}

// `ramRead`: the RAM-layer read of the first clock key got an answer, which
// shows the receiver is talking at all, so an OTP layer that stays silent
// means a blank OTP rather than a lost reply. A receiver running the high
// clock is Active however it got there.
inline GnssClockState classifyClock(bool ramRead, bool ramHigh, bool otpHigh) {
  if (!ramRead)
    return GnssClockState::Unknown;
  if (ramHigh)
    return GnssClockState::Active;
  return otpHigh ? GnssClockState::Pending : GnssClockState::Stock;
}

inline const char* gnssClockText(GnssClockState s) {
  switch (s) {
    case GnssClockState::Stock:
      return "stock 128 MHz";
    case GnssClockState::Pending:
      return "high clock programmed, not active yet";
    case GnssClockState::Active:
      return "192 MHz";
    case GnssClockState::Unchecked:
      return "not checked";
    case GnssClockState::Unknown:
      break;
  }
  return "unknown";
}

}  // namespace gnimu
