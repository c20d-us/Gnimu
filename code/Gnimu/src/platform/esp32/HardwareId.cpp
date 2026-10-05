// ARDUINO_ARCH_ESP32, not the chip-specific two-layer guard Clock/GnssUart
// need: ESP.getEfuseMac() is implemented unconditionally in the core's own
// Esp.cpp (no CONFIG_IDF_TARGET_* guard around it, unlike its neighboring
// functions that do need one), so it's genuinely portable across the whole
// family and this file can be reused unchanged by a future S3/C3/C6 variant.
#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "../HardwareId.h"

namespace gnimu {

uint64_t hardwareId() { return ESP.getEfuseMac(); }

}  // namespace gnimu

#endif
