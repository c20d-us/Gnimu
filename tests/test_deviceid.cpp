// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/DeviceId.h"

using namespace gnimu;

// Cross-checked against an independent Python implementation of the same
// FNV-1a-over-"GN1"-then-hardwareId-bytes, mod 4e9 algorithm.
static void expect(uint64_t hardwareId, const char* golden) {
  char out[11];
  decimalDeviceId(hardwareId, out);
  assert(strlen(out) == 10);
  assert(strcmp(out, golden) == 0);
}

int main() {
  expect(0x0, "3119116107");
  expect(0x0123456789ABCDEF, "3371058667");
  expect(0xFFFFFFFFFFFFFFFF, "3059863491");
  expect(0x1, "0151526698");  // exercises leading-zero padding
  puts("deviceid: all tests passed");
  return 0;
}
