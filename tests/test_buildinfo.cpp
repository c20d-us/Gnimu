// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/BuildInfo.h"

using namespace gnimu;

int main() {
  const BuildInfo b = {"v1.2-3-gabc123-dirty", "abc123def456", "1", "2026-09-24T18:00:00Z", "esp32"};
  char buf[192];
  const size_t n = formatBuildLine(buf, sizeof(buf), b);
  assert(n == strlen(buf));
  assert(strcmp(buf,
                "BUILD,version=v1.2-3-gabc123-dirty,revision=abc123def456,dirty=1,"
                "utc=2026-09-24T18:00:00Z,target=esp32") == 0);

  char small[16];
  const size_t m = formatBuildLine(small, sizeof(small), b);
  assert(m == sizeof(small) - 1 && small[m] == '\0');
  puts("buildinfo: all tests passed");
  return 0;
}
