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
#include <Arduino.h>
#include <type_traits>

// ============================================================================
// UBX Packet Construction Helpers
//
// Stateless helpers for building u-blox UBX-format binary messages.
// ============================================================================

// Write a little-endian integer into buffer at the given offset. Templated
// over the six fixed-width integer types used by the RaceBox payload; the
// static_assert acts as a whitelist so a call with an unsanctioned type (e.g.
// a bare uint64_t or size_t) fails to compile instead of silently writing the
// wrong number of bytes into a buffer sized for a narrower field.
//
// The value is reinterpreted as its same-width unsigned twin (a well-defined
// 2's-complement bit copy) before shifting, which avoids the
// implementation-defined behavior of right-shifting a negative signed value
// directly. Bytes are written explicitly (least-significant first) rather
// than via memcpy, so the output is little-endian on ANY host CPU regardless
// of the machine's native byte order.
template <typename T>
void writeLittleEndian(uint8_t *buffer, int offset, T value) {
  static_assert(
      std::is_same<T, uint32_t>::value || std::is_same<T, int32_t>::value ||
          std::is_same<T, uint16_t>::value || std::is_same<T, int16_t>::value ||
          std::is_same<T, uint8_t>::value || std::is_same<T, int8_t>::value,
      "writeLittleEndian only supports uint8/16/32_t and "
      "int8/16/32_t.");
  using UnsignedT = typename std::make_unsigned<T>::type;
  UnsignedT bits = static_cast<UnsignedT>(value);
  for (size_t i = 0; i < sizeof(T); i++) {
    buffer[offset + i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}

// A struct to hold the UBX checksum values (ckA and ckB).
struct UbxChecksum {
  uint8_t ckA;
  uint8_t ckB;
};

// Compute the UBX 8-bit Fletcher checksum over class + id + length + payload.
// Results are returned as a UbxChecksum struct.
UbxChecksum calculateChecksum(const uint8_t *payload, uint16_t len, uint8_t cls,
                              uint8_t id);
