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

#include "gc_ubx_helpers.h"

// Calculate the payload's checksum using the UBX checksum algorithm (an 8-bit
// Fletcher checksum). Return the checksum as a UbxChecksum struct.
UbxChecksum calculateChecksum(const uint8_t *payload, uint16_t len, uint8_t cls,
                              uint8_t id) {
  uint8_t a = 0;
  uint8_t b = 0;

  a += cls;
  b += a;
  a += id;
  b += a;
  a += (len & 0xFF);
  b += a;
  a += (len >> 8);
  b += a;
  for (uint16_t i = 0; i < len; i++) {
    a += payload[i];
    b += a;
  }
  return {a, b};
}
