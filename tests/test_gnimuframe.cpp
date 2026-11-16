// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/app/protocol/GnimuFrame.h"

using namespace gnimu;
using gnimu_frame::v1::Decoder;
using gnimu_frame::v1::Field;

// Gnimu-protocol's vectors/v1.json, "gnimu-parked".
static const char kParkedHex[] =
    "32ef000011010b02ef3f22bb1cc06ccf4740d88e988f5a965ec0d5dd00000000"
    "00009c01dd02030c031100c0e5dcc7187a4215000c0000003f00000000e80300"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "000000000007920157000000";

static void fromHex(const char* hex, uint8_t* out, size_t n) {
  for (size_t i = 0; i < n; i++) {
    unsigned v;
    sscanf(hex + 2 * i, "%2x", &v);
    out[i] = static_cast<uint8_t>(v);
  }
}

// The parked vector's situation, in the receiver's own units. Frames are sent
// at 61.234 s, 12 ms after the epoch arrived.
constexpr uint64_t kNowUs = 61234000;

static GnssEpoch parkedEpoch() {
  GnssEpoch e;
  e.arrivalUs = kNowUs - 12000;
  e.latDeg1e7 = 476205063;
  e.lonDeg1e7 = -1223492774;
  e.hMslMm = 56789;
  e.hAccMm = 412;
  e.vAccMm = 733;
  e.sAccMmPerS = 70;  // 0.252 km/h, sent as 0.3
  e.pDop = 121;
  e.fixType = 3;
  e.gnssFixOk = true;
  e.numSv = 17;
  e.year = 2026;
  e.month = 10;
  e.day = 30;
  e.hour = 11;
  e.min = 6;
  e.sec = 57;
  e.nanoNs = 500000000;
  e.validDate = e.validTime = e.fullyResolved = true;
  e.tAccNs = 21;
  return e;
}

static ImuSample parkedImu() {
  ImuSample s;
  s.accel[2] = 9.80665f;
  return s;
}

static BatteryStatus parkedBattery() {
  BatteryStatus b;
  b.valid = true;
  b.voltage = 4.02f;
  b.percent = 87;
  return b;
}

// Encodes with a fresh protocol whose sequence is advanced to `sequence`.
static void encodeAt(uint8_t sequence, const GnssEpoch& e, const ImuSample& imu, const BatteryStatus* b,
                     bool usb, GnimuFrameProtocol::Packet& out) {
  GnimuFrameProtocol p;
  for (unsigned i = 0; i < sequence; i++) p.encode(e, imu, b, usb, kNowUs, out);
  p.encode(e, imu, b, usb, kNowUs, out);
}

static void testParkedMatchesTheVector() {
  const GnssEpoch e = parkedEpoch();
  const BatteryStatus b = parkedBattery();
  GnimuFrameProtocol::Packet frame;
  encodeAt(17, e, parkedImu(), &b, true, frame);
  uint8_t expected[GnimuFrameProtocol::kPacketBytes];
  fromHex(kParkedHex, expected, sizeof(expected));
  assert(memcmp(frame, expected, sizeof(expected)) == 0);
}

static void testNoFix() {
  GnssEpoch e = parkedEpoch();
  e.fixType = 0;
  e.gnssFixOk = false;
  GnimuFrameProtocol::Packet frame;
  encodeAt(0, e, parkedImu(), nullptr, false, frame);
  const Decoder d(frame);
  // No position, but the solution itself is described.
  assert(!d.valid(Field::Latitude) && !d.valid(Field::Longitude) && !d.valid(Field::AltitudeMsl));
  assert(!d.valid(Field::GroundSpeed) && !d.valid(Field::SpeedAccuracy) && !d.valid(Field::Heading));
  assert(d.valid(Field::HorizontalAccuracy) && d.valid(Field::VerticalAccuracy) && d.valid(Field::Pdop));
  assert(d.valid(Field::FixType) && d.value(Field::FixType) == 0);
  assert(d.valid(Field::Satellites) && d.valid(Field::GnssEpochAge));
  assert(d.value(Field::Latitude) == 0);
}

static void testFixTypeNeedsGnssFixOk() {
  GnssEpoch e = parkedEpoch();
  e.gnssFixOk = false;
  GnimuFrameProtocol::Packet frame;
  encodeAt(0, e, parkedImu(), nullptr, false, frame);
  assert(!Decoder(frame).valid(Field::Latitude));

  e.gnssFixOk = true;
  e.fixType = 5;  // time only
  encodeAt(0, e, parkedImu(), nullptr, false, frame);
  assert(!Decoder(frame).valid(Field::Latitude));
}

static void testHeadingSpeedGate() {
  GnssEpoch e = parkedEpoch();
  e.headMotDeg1e5 = 27150000;
  // 5 km/h is 1388.9 mm/s.
  e.gSpeedMmPerS = 1388;
  GnimuFrameProtocol::Packet frame;
  encodeAt(0, e, parkedImu(), nullptr, false, frame);
  assert(!Decoder(frame).valid(Field::Heading));

  e.gSpeedMmPerS = 1389;
  encodeAt(0, e, parkedImu(), nullptr, false, frame);
  const Decoder d(frame);
  assert(d.valid(Field::Heading) && d.value(Field::Heading) == 271.5);
  assert(d.value(Field::GroundSpeed) == 5.0);
}

static void testUtcNeedsAllThreeFlags() {
  for (int clear = 0; clear < 3; clear++) {
    GnssEpoch e = parkedEpoch();
    if (clear == 0) e.validDate = false;
    if (clear == 1) e.validTime = false;
    if (clear == 2) e.fullyResolved = false;
    GnimuFrameProtocol::Packet frame;
    encodeAt(0, e, parkedImu(), nullptr, false, frame);
    const Decoder d(frame);
    assert(!d.valid(Field::UtcTimestamp) && !d.valid(Field::TimeAccuracy));
    assert(d.valid(Field::Latitude));
  }
}

static void testNegativeNanoBorrows() {
  GnssEpoch e = parkedEpoch();
  e.sec = 58;
  e.nanoNs = -250000000;
  assert(GnimuFrameProtocol::utcMs(e) == 1793358417750.0);
}

static void testEpochAgeSaturates() {
  GnssEpoch e = parkedEpoch();
  const uint64_t nowUs = 100000000;
  e.arrivalUs = nowUs - 70000000;
  GnimuFrameProtocol p;
  GnimuFrameProtocol::Packet frame;
  p.encode(e, parkedImu(), nullptr, false, nowUs, frame);
  assert(Decoder(frame).value(Field::GnssEpochAge) == 65535);
}

static void testPower() {
  const GnssEpoch e = parkedEpoch();
  GnimuFrameProtocol::Packet frame;
  // No battery on the board: no power capability and an empty block.
  encodeAt(0, e, parkedImu(), nullptr, true, frame);
  assert(Decoder(frame).capabilities() == 3);
  assert(frame[gnimu_frame::v1::kPowerMaskOffset] == 0);

  // A battery with no reading yet: the capability, but nothing valid.
  BatteryStatus b;
  encodeAt(0, e, parkedImu(), &b, true, frame);
  assert(Decoder(frame).capabilities() == 11);
  assert(frame[gnimu_frame::v1::kPowerMaskOffset] == 0);

  // Charging needs USB as well as the charger pin.
  b = parkedBattery();
  b.chargerActive = true;
  encodeAt(0, e, parkedImu(), &b, false, frame);
  assert(Decoder(frame).valid(Field::Charging) && Decoder(frame).value(Field::Charging) == 0);
  encodeAt(0, e, parkedImu(), &b, true, frame);
  assert(Decoder(frame).value(Field::Charging) == 1);
}

static void testImu() {
  const GnssEpoch e = parkedEpoch();
  GnimuFrameProtocol::Packet frame;
  // Down: all zeros, so the block is empty.
  encodeAt(0, e, ImuSample{}, nullptr, false, frame);
  assert(frame[gnimu_frame::v1::kImuMaskOffset] == 0);

  // A left turn: the signs pass straight through, in g and deg/s.
  ImuSample s;
  s.accel[1] = 0.85f * 9.80665f;
  s.accel[2] = 9.80665f;
  s.gyro[0] = -0.0366519f;  // -2.1 deg/s
  s.gyro[2] = 0.4415683f;   // 25.3 deg/s
  encodeAt(0, e, s, nullptr, false, frame);
  const Decoder d(frame);
  assert(frame[gnimu_frame::v1::kImuMaskOffset] == 0x3F);
  assert(d.value(Field::AccelY) == 0.85 && d.value(Field::AccelZ) == 1.0);
  assert(d.value(Field::RollRate) == -2.1 && d.value(Field::YawRate) == 25.3);
}

static void testSequenceWraps() {
  const GnssEpoch e = parkedEpoch();
  GnimuFrameProtocol::Packet frame;
  encodeAt(255, e, parkedImu(), nullptr, false, frame);
  assert(Decoder(frame).sequence() == 255);
  GnimuFrameProtocol p;
  for (int i = 0; i < 257; i++) p.encode(e, parkedImu(), nullptr, false, kNowUs, frame);
  assert(Decoder(frame).sequence() == 0);
}

static void testEveryFieldValidMatchesTheFillMasks() {
  // Moving with a fix and UTC time, IMU up, battery read: every field Gnimu
  // can provide is valid, and those are exactly what the Schema promises.
  GnssEpoch e = parkedEpoch();
  e.gSpeedMmPerS = 20000;
  e.headMotDeg1e5 = 9000000;
  const BatteryStatus b = parkedBattery();
  GnimuFrameProtocol::Packet frame;
  encodeAt(0, e, parkedImu(), &b, true, frame);
  using namespace gnimu_frame::v1;
  assert((frame[kGnssMaskOffset] | frame[kGnssMaskOffset + 1] << 8) == GnimuFrameProtocol::kGnssFill);
  assert(frame[kImuMaskOffset] == GnimuFrameProtocol::kImuFill);
  assert(frame[kPowerMaskOffset] == GnimuFrameProtocol::kPowerFill);
  for (size_t i = 0; i < 4; i++) assert(frame[kVehicleMaskOffset + i] == 0);
}

static void testSchema() {
  char buf[kBleTextMaxBytes];
  const size_t n = GnimuFrameProtocol::formatSchema(buf, sizeof(buf), true);
  char expected[96];
  snprintf(expected, sizeof(expected), "{\"v\":1,\"d\":\"Gnimu\",\"r\":%u,\"fill\":{\"gnss\":16383,\"imu\":63,\"power\":7}}",
           static_cast<unsigned>(config::kGnssNavRateHz));
  assert(strcmp(buf, expected) == 0 && n == strlen(expected));

  GnimuFrameProtocol::formatSchema(buf, sizeof(buf), false);
  snprintf(expected, sizeof(expected), "{\"v\":1,\"d\":\"Gnimu\",\"r\":%u,\"fill\":{\"gnss\":16383,\"imu\":63}}",
           static_cast<unsigned>(config::kGnssNavRateHz));
  assert(strcmp(buf, expected) == 0);

  // Cut off, never overrun.
  char tiny[8];
  assert(GnimuFrameProtocol::formatSchema(tiny, sizeof(tiny), true) == 7 && tiny[7] == '\0');
}

static void testProfile() {
  // The family UUIDs from the spec, the same on every Gnimu device.
  const BleProfile& p = GnimuFrameProtocol::profile();
  assert(p.charCount == 3);
  assert(strcmp(p.advertisedServiceUuid, "9074beb4-2f1a-4b5f-9818-fe2d2277ef05") == 0);
  const BleCharacteristic& f = p.chars[GnimuFrameProtocol::kFrame];
  assert(strcmp(f.uuid, "0752d3c3-ceaf-462f-a944-bcc06c57284b") == 0 && f.props == (kBleNotify | kBleRead));
  const BleCharacteristic& s = p.chars[GnimuFrameProtocol::kSchema];
  assert(strcmp(s.uuid, "1c9b3945-10eb-4fd0-a5a4-b9d80842380d") == 0 && s.text == BleText::Schema);
  const BleCharacteristic& d = p.chars[GnimuFrameProtocol::kDeviceInfo];
  assert(strcmp(d.uuid, "6d0adfef-8218-488a-a5f9-1ba05f422c22") == 0 && d.text == BleText::DeviceInfo);
  // The DIS revisions are the build's own, filled in by App.
  assert(p.firmwareRevision == nullptr && p.hardwareRevision == nullptr);
  assert(GnimuFrameProtocol::kRxChar == kBleNoChar);
}

int main() {
  testParkedMatchesTheVector();
  testEveryFieldValidMatchesTheFillMasks();
  testSchema();
  testProfile();
  testNoFix();
  testFixTypeNeedsGnssFixOk();
  testHeadingSpeedGate();
  testUtcNeedsAllThreeFlags();
  testNegativeNanoBorrows();
  testEpochAgeSaturates();
  testPower();
  testImu();
  testSequenceWraps();
  puts("gnimuframe: all tests passed");
  return 0;
}
