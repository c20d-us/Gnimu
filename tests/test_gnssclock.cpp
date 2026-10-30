// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <string.h>

#include "../code/Gnimu/src/core/GnssClock.h"

using namespace gnimu;

// The payload of the manual's expected OTP reply (UBX-20053088 sec 2.1.7),
// without the frame's sync, class/id/length and checksum.
static const uint8_t kProgrammed[] = {
    0x01, 0x04, 0x00, 0x00,                          // version, layer 4, position
    0x01, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B,  // 0x40A40001 = 192 MHz
    0x03, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B,  // 0x40A40003 = 192 MHz
    0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B,  // 0x40A40005 = 192 MHz
    0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05,  // 0x40A4000A = 96 MHz
};

static void testManualsReplyIsHigh() {
  assert(otpHoldsHighClock(kProgrammed, sizeof(kProgrammed)));
}

// The stock values (128/128/128/64 MHz) in the same shape are not the high clock.
static void testStockValuesAreNotHigh() {
  uint8_t stock[sizeof(kProgrammed)];
  memcpy(stock, kProgrammed, sizeof(stock));
  const uint32_t hz[4] = {128000000UL, 128000000UL, 128000000UL, 64000000UL};
  for (size_t i = 0; i < 4; i++)
    for (size_t b = 0; b < 4; b++)
      stock[8 + i * 8 + b] = static_cast<uint8_t>(hz[i] >> (8 * b));
  assert(!otpHoldsHighClock(stock, sizeof(stock)));
}

// One wrong value out of four is not enough.
static void testOneKeyWrong() {
  uint8_t p[sizeof(kProgrammed)];
  memcpy(p, kProgrammed, sizeof(p));
  p[sizeof(p) - 3] ^= 0x01;
  assert(!otpHoldsHighClock(p, sizeof(p)));
}

// The keys must come back in the order they were asked for.
static void testKeysOutOfOrder() {
  uint8_t p[sizeof(kProgrammed)];
  memcpy(p, kProgrammed, sizeof(p));
  p[4] = 0x03;
  p[12] = 0x01;
  assert(!otpHoldsHighClock(p, sizeof(p)));
}

// A short reply never reads past its end.
static void testTruncatedReply() {
  for (size_t n = 0; n < sizeof(kProgrammed); n++)
    assert(!otpHoldsHighClock(kProgrammed, n));
}

static void testClassify() {
  assert(classifyClock(false, false, false) == GnssClockState::Unknown);
  assert(classifyClock(false, true, true) == GnssClockState::Unknown);
  assert(classifyClock(true, false, false) == GnssClockState::Stock);
  assert(classifyClock(true, false, true) == GnssClockState::Pending);
  assert(classifyClock(true, true, true) == GnssClockState::Active);
  assert(classifyClock(true, true, false) == GnssClockState::Active);
}

static void testText() {
  assert(!strcmp(gnssClockText(GnssClockState::Active), "192 MHz"));
  assert(!strcmp(gnssClockText(GnssClockState::Stock), "stock 128 MHz"));
  assert(!strcmp(gnssClockText(GnssClockState::Pending), "high clock programmed, not active yet"));
  assert(!strcmp(gnssClockText(GnssClockState::Unknown), "unknown"));
  assert(!strcmp(gnssClockText(GnssClockState::Unchecked), "not checked"));
}

int main() {
  testManualsReplyIsHigh();
  testStockValuesAreNotHigh();
  testOneKeyWrong();
  testKeysOutOfOrder();
  testTruncatedReply();
  testClassify();
  testText();
  return 0;
}
