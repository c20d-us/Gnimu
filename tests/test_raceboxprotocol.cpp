#include <assert.h>
#include <stdio.h>
#include <string.h>

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
  p.encode(e, imu, &b, true, viaProtocol);
  uint8_t direct[kRaceBoxPacketBytes];
  encodeRaceBoxData(e, imu, 50 | 0x80, direct);
  assert(memcmp(viaProtocol, direct, sizeof(direct)) == 0);
}

int main() {
  testBatteryByte();
  testEncodeMatchesTheEncoder();
  puts("raceboxprotocol: all tests passed");
  return 0;
}
