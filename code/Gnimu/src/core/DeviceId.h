#pragma once

#include <stdint.h>

namespace gnimu {

// Stable rendering of a hardware ID as a 10-digit decimal string, zero-padded.
// FNV-1a 64-bit over a fixed "GN1" domain prefix (this scheme's version) then
// the hardware ID's 8 bytes, little end first, folded into the 10-digit
// namespace by taking it mod 4e9. The namespace is smaller than the ID space,
// so collisions are possible in principle; there is no persistence or
// override, so a migrated or replaced chip gets a new ID.
inline void decimalDeviceId(uint64_t hardwareId, char (&out)[11]) {
  uint64_t hash = UINT64_C(14695981039346656037);
  const uint8_t prefix[] = {'G', 'N', '1'};
  for (uint8_t byte : prefix) hash = (hash ^ byte) * UINT64_C(1099511628211);
  for (unsigned i = 0; i < 8; i++)
    hash = (hash ^ static_cast<uint8_t>(hardwareId >> (8 * i))) * UINT64_C(1099511628211);

  uint32_t decimal = static_cast<uint32_t>(hash % UINT64_C(4000000000));
  for (int i = 9; i >= 0; i--) {
    out[i] = static_cast<char>('0' + decimal % 10);
    decimal /= 10;
  }
  out[10] = '\0';
}

}  // namespace gnimu
