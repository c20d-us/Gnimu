// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/Config.h"
#include "../code/Gnimu/src/app/protocol/RaceBox.h"

using namespace gnimu;

static void testBatteryByte() {
  // No battery, or no reading yet: a fixed 100, not charging.
  assert(RaceBoxProtocol::batteryByte(nullptr, true) == 100);
  BatteryStatus b;
  assert(RaceBoxProtocol::batteryByte(&b, true) == 100);

  b.valid = true;
  b.percent = 76;
  assert(RaceBoxProtocol::batteryByte(&b, true) == 76);
  b.chargerActive = true;
  assert(RaceBoxProtocol::batteryByte(&b, true) == (76 | 0x80));
  // The charging bit needs USB too, the same rule as the status line's bolt.
  assert(RaceBoxProtocol::batteryByte(&b, false) == 76);
  b.percent = 0xFF;  // out of range: masked to 7 bits, never into the flag
  b.chargerActive = false;
  assert(RaceBoxProtocol::batteryByte(&b, true) == 0x7F);
}

static void testEncodeMatchesTheEncoder() {
  GnssEpoch e;
  e.iTowMs = 123456;
  e.fixType = 3;
  ImuSample imu = {};
  BatteryStatus b;
  b.valid = true;
  b.percent = 50;
  b.chargerActive = true;

  RaceBoxProtocol p;
  RaceBoxProtocol::Packet viaProtocol;
  p.encode(e, imu, &b, true, 0, viaProtocol);
  uint8_t direct[kRaceBoxPacketBytes];
  encodeRaceBoxData(e, imu, 50 | 0x80, direct);
  assert(memcmp(viaProtocol, direct, sizeof(direct)) == 0);
}

static void testDefaultIdentityIsNotRaceBox() {
  // The shipped identity is Gnimu's own; a build that needs another name
  // edits Config.h.
  assert(strncmp(config::kBleNamePrefix, "RaceBox", 7) != 0);
  assert(strncmp(config::kBleManufacturer, "RaceBox", 7) != 0);
  // The profile leaves identity to Config.h, so it can't carry one of its own.
  assert(RaceBoxProtocol::profile().model[0] == '\0');
  assert(RaceBoxProtocol::profile().manufacturer[0] == '\0');
}

static void testProfileLayout() {
  // NUS TX and RX, then the config text in the Gnimu service: the same GATT
  // layout RaceBox clients have always seen.
  const BleProfile& p = RaceBoxProtocol::profile();
  assert(p.charCount == 3);
  assert(strcmp(p.advertisedServiceUuid, kNusServiceUuid) == 0);
  assert(strcmp(p.chars[RaceBoxProtocol::kTx].uuid, "6E400003-B5A3-F393-E0A9-E50E24DCCA9E") == 0);
  assert(p.chars[RaceBoxProtocol::kTx].props == kBleNotify);
  assert(strcmp(p.chars[RaceBoxProtocol::kRx].uuid, "6E400002-B5A3-F393-E0A9-E50E24DCCA9E") == 0);
  assert(p.chars[RaceBoxProtocol::kRx].props == kBleWrite);
  const BleCharacteristic& config = p.chars[RaceBoxProtocol::kConfig];
  assert(strcmp(config.serviceUuid, kGnimuServiceUuid) == 0);
  assert(strcmp(config.uuid, "689E54CD-60B0-473A-B049-A5E02A2716E0") == 0);
  assert(config.props == kBleRead && config.text == BleText::ConfigReport);
}

int main() {
  testDefaultIdentityIsNotRaceBox();
  testProfileLayout();
  testBatteryByte();
  testEncodeMatchesTheEncoder();
  puts("raceboxprotocol: all tests passed");
  return 0;
}
