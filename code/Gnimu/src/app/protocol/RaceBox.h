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

constexpr char kNusServiceUuid[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
// The Gnimu service: the device's configuration as text, read on demand
// (docs/device-commands.md). Its own service, so RaceBox clients never see
// anything unfamiliar in theirs, and not advertised: a client finds it by
// discovering services once connected.
constexpr char kGnimuServiceUuid[] = "20D21CDE-F59E-41F1-9BCA-5BDE69876C8B";

// The RaceBox data protocol's BLE layout. The model and manufacturer are left
// empty: App fills them from Config.h, the one place the device's identity is
// set.
constexpr BleProfile kRaceBoxProfile = {
    "",
    "",
    "3.3",
    "1",
    kNusServiceUuid,
    3,
    {
        {kNusServiceUuid, "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", kBleNotify, BleText::None, nullptr},
        {kNusServiceUuid, "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", kBleWrite, BleText::None, nullptr},
        {kGnimuServiceUuid, "689E54CD-60B0-473A-B049-A5E02A2716E0", kBleRead, BleText::ConfigReport,
         nullptr},
    },
};

// The RaceBox data protocol: its BLE layout, its 88-byte data packet, and its
// inbound commands (none handled yet). Pure - no hardware - so it's
// host-testable; App hands the encoded packet to BLE.
class RaceBoxProtocol {
 public:
  static constexpr bool kUsesImu = true;
  static constexpr size_t kPacketBytes = kRaceBoxPacketBytes;
  using Packet = uint8_t[kPacketBytes];

  // Positions in kRaceBoxProfile's list.
  enum : uint8_t { kTx, kRx, kConfig, kCharCount };
  static constexpr uint8_t kDataChar = kTx;  // packets go out here
  static constexpr uint8_t kRxChar = kRx;    // the client's writes arrive here

  static const BleProfile& profile() { return kRaceBoxProfile; }
  static const char* name() { return "RaceBox"; }

  // `battery` is nullptr on boards without one. RaceBox carries no device
  // time, so `nowUs` goes unused; it keeps App's call the same for every
  // protocol.
  void encode(const GnssEpoch& e, const ImuSample& imu, const BatteryStatus* battery, bool usbPresent,
              uint64_t /*nowUs*/, Packet& out) const {
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

static_assert(bleProfileValid(kRaceBoxProfile), "RaceBox BLE profile is malformed");
static_assert(kRaceBoxProfile.charCount == RaceBoxProtocol::kCharCount,
              "RaceBox's characteristic names don't match its list");
static_assert(bleNotifies(kRaceBoxProfile, RaceBoxProtocol::kDataChar),
              "RaceBox sends on a notify characteristic");
static_assert(bleWritableOrNone(kRaceBoxProfile, RaceBoxProtocol::kRxChar),
              "RaceBox reads from a writable characteristic");

}  // namespace gnimu
