// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

// ============================================================================
// GC1 golden vectors - the file format, and the loader both host harnesses use.
//
// test/harness.cpp runs each vector through the encoder alone;
// test/telemetry/telemetry_harness.cpp runs the same vectors through the real
// g_telemetry.cpp. One parser, so the two cannot disagree about what a vector
// says. Moved here unchanged from harness.cpp.
//
// Header-only and in an anonymous namespace: include it from exactly one
// translation unit per program.
// ============================================================================

#include "g_proto_racebox.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Field order on a GC1 line, matching GC1_ORDER in test/capture.py. A vector
// file carries 35 input values then the expected packet as hex.
constexpr int GC1_FIELDS = 35;

struct Vector {
  std::string source; // file:line, for reporting
  std::string label;  // preceding "# ..." comment, if any
  TelemetrySample sample;
  uint8_t expected[RACEBOX_PACKET_LEN];
};

bool parseHex(const std::string &hex, uint8_t *out, size_t outLen) {
  if (hex.size() != outLen * 2) {
    return false;
  }
  for (size_t i = 0; i < outLen; i++) {
    unsigned byte = 0;
    if (sscanf(hex.c_str() + i * 2, "%2x", &byte) != 1) {
      return false;
    }
    out[i] = (uint8_t)byte;
  }
  return true;
}

// Split on commas without trimming - the writers never emit spaces.
std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> parts;
  size_t start = 0;
  for (size_t i = 0; i <= line.size(); i++) {
    if (i == line.size() || line[i] == ',') {
      parts.push_back(line.substr(start, i - start));
      start = i + 1;
    }
  }
  return parts;
}

// Field values are written by Python as plain decimal, including values that
// overflow int32 when signed (e.g. lon at -2147483648). strtoll then a cast
// keeps every width honest.
long long num(const std::string &s) { return strtoll(s.c_str(), nullptr, 10); }

bool parseVector(const std::vector<std::string> &f, Vector &v) {
  // "GC1" + 35 inputs + hex
  if (f.size() != (size_t)GC1_FIELDS + 2) {
    return false;
  }
  TelemetrySample &s = v.sample;
  int i = 1;
  s.iTOW = (uint32_t)num(f[i++]);
  s.year = (uint16_t)num(f[i++]);
  s.month = (uint8_t)num(f[i++]);
  s.day = (uint8_t)num(f[i++]);
  s.hour = (uint8_t)num(f[i++]);
  s.min = (uint8_t)num(f[i++]);
  s.sec = (uint8_t)num(f[i++]);
  s.validDate = num(f[i++]) != 0;
  s.validTime = num(f[i++]) != 0;
  s.fullyResolved = num(f[i++]) != 0;
  s.validMag = num(f[i++]) != 0;
  s.tAcc = (uint32_t)num(f[i++]);
  s.nano = (int32_t)num(f[i++]);
  s.fixType = (uint8_t)num(f[i++]);
  s.gnssFixOK = num(f[i++]) != 0;
  s.headVehValid = num(f[i++]) != 0;
  s.numSV = (uint8_t)num(f[i++]);
  s.lon = (int32_t)num(f[i++]);
  s.lat = (int32_t)num(f[i++]);
  s.height = (int32_t)num(f[i++]);
  s.hMSL = (int32_t)num(f[i++]);
  s.hAcc = (uint32_t)num(f[i++]);
  s.vAcc = (uint32_t)num(f[i++]);
  s.gSpeed = (int32_t)num(f[i++]);
  s.headMot = (int32_t)num(f[i++]);
  s.sAcc = (uint32_t)num(f[i++]);
  s.headAcc = (uint32_t)num(f[i++]);
  s.pDOP = (uint16_t)num(f[i++]);
  s.accelX = (int16_t)num(f[i++]);
  s.accelY = (int16_t)num(f[i++]);
  s.accelZ = (int16_t)num(f[i++]);
  s.gyroX = (int16_t)num(f[i++]);
  s.gyroY = (int16_t)num(f[i++]);
  s.gyroZ = (int16_t)num(f[i++]);

  // A GC1 line carries the PACKED battery byte, because that is what the
  // firmware puts on the wire. The sample splits it, so the harness must too -
  // and the encoder re-packing it is exactly what this round trip tests.
  const uint8_t packed = (uint8_t)num(f[i++]);
  s.batteryCharging = (packed & 0x80) != 0;
  s.batteryPercent = packed & 0x7F;

  return parseHex(f[i], v.expected, RACEBOX_PACKET_LEN);
}

int loadFile(const char *path, std::vector<Vector> &out, bool required) {
  FILE *fp = fopen(path, "r");
  if (!fp) {
    if (required) {
      fprintf(stderr, "cannot open %s\n", path);
      return -1;
    }
    printf("  %-28s (absent - skipped)\n", path);
    return 0;
  }
  char buf[8192];
  int lineno = 0, loaded = 0, malformed = 0;
  std::string pendingLabel;
  while (fgets(buf, sizeof(buf), fp)) {
    lineno++;
    std::string line(buf);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      // Synthetic vectors carry a "# label" line naming what they test, so a
      // failure can say what broke rather than just which line number.
      pendingLabel = line.substr(1);
      while (!pendingLabel.empty() && pendingLabel[0] == ' ') {
        pendingLabel.erase(0, 1);
      }
      continue;
    }
    if (line.compare(0, 4, "GC1,") != 0) {
      continue;
    }
    Vector v;
    v.source = std::string(path) + ":" + std::to_string(lineno);
    v.label = pendingLabel;
    pendingLabel.clear();
    if (!parseVector(split(line), v)) {
      malformed++;
      continue;
    }
    out.push_back(v);
    loaded++;
  }
  fclose(fp);
  printf("  %-28s %5d vectors", path, loaded);
  if (malformed) {
    printf("   ⚠️  %d malformed lines", malformed);
  }
  printf("\n");
  return malformed ? -1 : loaded;
}

// Load the vector files named on the command line (from argv[first] on), or
// the defaults when there are none. Returns false after printing why, if
// anything failed to load or nothing loaded at all.
bool loadVectors(int argc, char **argv, int first, std::vector<Vector> &vectors) {
  printf("loading vectors:\n");

  bool bad = false;
  if (argc > first) {
    for (int i = first; i < argc; i++) {
      if (loadFile(argv[i], vectors, true) < 0) {
        bad = true;
      }
    }
  } else {
    // Synthetic vectors are committed and always present; captured vectors are
    // gitignored (they hold real positions), so their absence is not an error.
    if (loadFile("test/synthetic.gc1", vectors, true) < 0) {
      bad = true;
    }
    if (loadFile("test/vectors.gc1", vectors, false) < 0) {
      bad = true;
    }
  }
  if (bad) {
    return false;
  }
  if (vectors.empty()) {
    fprintf(stderr, "no vectors loaded\n");
    return false;
  }
  return true;
}

} // namespace
