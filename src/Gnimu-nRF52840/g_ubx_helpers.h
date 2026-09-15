// Gnimu - GNSS+IMU streaming telemetry
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
// Standard headers only, so the encoders build on a host for test/harness.cpp.
#include <stddef.h>
#include <stdint.h>
#include <type_traits>

// UBX helpers: stateless builders for u-blox UBX binary messages.

// Write a little-endian integer into buffer at offset. Only the six 8/16/32-bit
// fixed-width types compile. Byte order is independent of the host CPU.
template <typename T>
void writeLittleEndian(uint8_t *buffer, int offset, T value) {
  static_assert(
      std::is_same<T, uint32_t>::value || std::is_same<T, int32_t>::value ||
          std::is_same<T, uint16_t>::value || std::is_same<T, int16_t>::value ||
          std::is_same<T, uint8_t>::value || std::is_same<T, int8_t>::value,
      "writeLittleEndian only supports uint8/16/32_t and "
      "int8/16/32_t.");
  // Shift the unsigned twin: right-shifting a negative value is
  // implementation-defined.
  using UnsignedT = typename std::make_unsigned<T>::type;
  UnsignedT bits = static_cast<UnsignedT>(value);
  for (size_t i = 0; i < sizeof(T); i++) {
    buffer[offset + i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}

struct UbxChecksum {
  uint8_t ckA;
  uint8_t ckB;
};

// UBX 8-bit Fletcher checksum over class, id, length, and payload.
UbxChecksum calculateChecksum(const uint8_t *payload, uint16_t len, uint8_t cls,
                              uint8_t id);
