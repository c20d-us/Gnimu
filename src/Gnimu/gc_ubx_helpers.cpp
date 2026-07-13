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

// The byte writes below are explicit (least-significant byte first) rather than
// a memcpy of the raw value, so the output is little-endian on ANY host CPU
// regardless of the machine's native byte order. The signed overloads
// reinterpret the value as their unsigned twin (a well-defined 2's-complement
// bit copy) and reuse the same byte logic, which also avoids
// implementation-defined right-shifts of negative numbers.

void writeLittleEndian(uint8_t *buffer, int offset, uint32_t value) {
  buffer[offset + 0] = (uint8_t)(value);
  buffer[offset + 1] = (uint8_t)(value >> 8);
  buffer[offset + 2] = (uint8_t)(value >> 16);
  buffer[offset + 3] = (uint8_t)(value >> 24);
}

void writeLittleEndian(uint8_t *buffer, int offset, int32_t value) {
  writeLittleEndian(buffer, offset, (uint32_t)value);
}

void writeLittleEndian(uint8_t *buffer, int offset, uint16_t value) {
  buffer[offset + 0] = (uint8_t)(value);
  buffer[offset + 1] = (uint8_t)(value >> 8);
}

void writeLittleEndian(uint8_t *buffer, int offset, int16_t value) {
  writeLittleEndian(buffer, offset, (uint16_t)value);
}

void writeLittleEndian(uint8_t *buffer, int offset, uint8_t value) {
  buffer[offset] = value;
}

void writeLittleEndian(uint8_t *buffer, int offset, int8_t value) {
  buffer[offset] = (uint8_t)value;
}

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
