// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../core/BatteryMonitor.h"
#include "../../core/BleProfile.h"
#include "../../core/GnssEpoch.h"
#include "../../core/ImuSample.h"
#include "../../core/RaceBoxEncoder.h"

namespace gnimu {

// The RaceBox Mini protocol: its BLE layout, its 88-byte data packet, and its
// inbound commands (none handled yet). Pure - no hardware - so it's
// host-testable; App hands the encoded packet to BLE.
class RaceBoxProtocol {
 public:
  static constexpr bool kUsesImu = true;
  static constexpr size_t kPacketBytes = kRaceBoxPacketBytes;
  using Packet = uint8_t[kPacketBytes];

  static const BleProfile& profile() { return kRaceBoxProfile; }
  static const char* name() { return "RaceBox"; }

  // `battery` is nullptr on boards without one.
  void encode(const GnssEpoch& e, const ImuSample& imu, const BatteryStatus* battery, bool usbPresent,
              Packet& out) const {
    encodeRaceBoxData(e, imu, batteryByte(battery, usbPresent), out);
  }

  // Bit 7 charging, bits 0-6 percent. A fixed 100 (not charging) without a
  // battery or before the first reading.
  static uint8_t batteryByte(const BatteryStatus* b, bool usbPresent) {
    if (!b || !b->valid) return kNoBattery;
    return static_cast<uint8_t>((b->percent & 0x7F) | (isCharging(*b, usbPresent) ? 0x80 : 0));
  }

  // Bytes the client wrote. No RaceBox commands are handled yet, so they're
  // dropped here; the command decoder goes in this class.
  void onRx(const uint8_t*, size_t) {}

 private:
  static constexpr uint8_t kNoBattery = 100;
};

}  // namespace gnimu
