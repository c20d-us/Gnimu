// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/RaceBoxEncoder.h"

using namespace gnimu;

// Golden packets computed independently (Python struct.pack of the RaceBox
// layout + UBX Fletcher checksum), not derived from the encoder under test.
// A: 3D fix with gnssFixOK, heading valid, date/time valid and resolved.
static const uint8_t kGoldenA[88] = {
    0xB5, 0x62, 0xFF, 0x01, 0x50, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0xEA, 0x07,
    0x0A, 0x02, 0x0E, 0x23, 0x3B, 0x07, 0x16, 0x00, 0x00, 0x00, 0xC0, 0x1D,
    0xFE, 0xFF, 0x03, 0x21, 0xE0, 0x11, 0xC0, 0x64, 0x46, 0xB7, 0xD2, 0x54,
    0x49, 0x1C, 0x40, 0xE2, 0x01, 0x00, 0xB0, 0xAD, 0x01, 0x00, 0x11, 0x02,
    0x00, 0x00, 0x2C, 0x03, 0x00, 0x00, 0xC6, 0x68, 0x00, 0x00, 0xF9, 0x2C,
    0x9C, 0x01, 0x96, 0x00, 0x00, 0x00, 0x87, 0xD6, 0x12, 0x00, 0xB1, 0x00,
    0x00, 0x64, 0x32, 0x00, 0x86, 0xFF, 0xE8, 0x03, 0x3C, 0x02, 0x68, 0xFA,
    0x61, 0x16, 0x3B, 0xA6,
};
// B: same data with fix type 4 (GNSS+DR), which the protocol reports as no
// fix: fix 0, no valid-fix bit despite gnssFixOK, lat/lon-invalid set; no
// time validity; battery 57% charging.
static const uint8_t kGoldenB[88] = {
    0xB5, 0x62, 0xFF, 0x01, 0x50, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0xEA, 0x07,
    0x0A, 0x02, 0x0E, 0x23, 0x3B, 0x00, 0x16, 0x00, 0x00, 0x00, 0xC0, 0x1D,
    0xFE, 0xFF, 0x00, 0x00, 0x00, 0x11, 0xC0, 0x64, 0x46, 0xB7, 0xD2, 0x54,
    0x49, 0x1C, 0x40, 0xE2, 0x01, 0x00, 0xB0, 0xAD, 0x01, 0x00, 0x11, 0x02,
    0x00, 0x00, 0x2C, 0x03, 0x00, 0x00, 0xC6, 0x68, 0x00, 0x00, 0xF9, 0x2C,
    0x9C, 0x01, 0x96, 0x00, 0x00, 0x00, 0x87, 0xD6, 0x12, 0x00, 0xB1, 0x00,
    0x01, 0xB9, 0x32, 0x00, 0x86, 0xFF, 0xE8, 0x03, 0x3C, 0x02, 0x68, 0xFA,
    0x61, 0x16, 0x86, 0x13,
};

static GnssEpoch fixtureA() {
  GnssEpoch e;
  e.iTowMs = 123456789;
  e.year = 2026;
  e.month = 10;
  e.day = 2;
  e.hour = 14;
  e.min = 35;
  e.sec = 59;
  e.validDate = e.validTime = e.fullyResolved = true;
  e.validMag = false;
  e.tAccNs = 22;
  e.nanoNs = -123456;
  e.fixType = 3;
  e.gnssFixOk = true;
  e.headVehValid = true;
  e.numSv = 17;
  e.lonDeg1e7 = -1220123456;
  e.latDeg1e7 = 474567890;
  e.heightMm = 123456;
  e.hMslMm = 110000;
  e.hAccMm = 529;
  e.vAccMm = 812;
  e.gSpeedMmPerS = 26822;
  e.headMotDeg1e5 = 27012345;
  e.sAccMmPerS = 150;
  e.headAccDeg1e5 = 1234567;
  e.pDop = 177;
  return e;
}

static ImuSample imuFixture() {
  ImuSample s;
  const float a[3] = {0.5f, -1.2f, 9.81f};
  const float g[3] = {0.1f, -0.25f, 1.0f};
  for (int i = 0; i < 3; i++) {
    s.accel[i] = a[i];
    s.gyro[i] = g[i];
  }
  return s;
}

static void testGoldenA() {
  uint8_t out[kRaceBoxPacketBytes];
  encodeRaceBoxData(fixtureA(), imuFixture(), 100, out);
  assert(memcmp(out, kGoldenA, sizeof(out)) == 0);
}

static void testGoldenBFixTypeClampedToNoFix() {
  GnssEpoch e = fixtureA();
  e.fixType = 4;
  e.headVehValid = false;
  e.validDate = e.validTime = e.fullyResolved = false;
  uint8_t out[kRaceBoxPacketBytes];
  encodeRaceBoxData(e, imuFixture(), 0x80 | 57, out);
  assert(memcmp(out, kGoldenB, sizeof(out)) == 0);
}

static void testImuSaturatesInsteadOfWrapping() {
  ImuSample s;
  s.accel[0] = 400.0f;   // ~40788 mG -> clamps to +32767
  s.gyro[0] = 10.0f;     // ~57296 c-deg/s -> clamps to +32767
  s.gyro[1] = -10.0f;    // -> clamps to -32768, not a wrapped positive
  uint8_t out[kRaceBoxPacketBytes];
  encodeRaceBoxData(fixtureA(), s, 100, out);
  const uint8_t *p = out + 6;
  assert(p[68] == 0xFF && p[69] == 0x7F);  // accel X = 32767
  assert(p[74] == 0xFF && p[75] == 0x7F);  // gyro X = 32767
  assert(p[76] == 0x00 && p[77] == 0x80);  // gyro Y = -32768
}

int main() {
  testGoldenA();
  testGoldenBFixTypeClampedToNoFix();
  testImuSaturatesInsteadOfWrapping();
  puts("raceboxencoder: all tests passed");
  return 0;
}
