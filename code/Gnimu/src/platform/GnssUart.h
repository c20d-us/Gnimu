#pragma once

#include <Arduino.h>

namespace gnimu {

// The UART wired to the GNSS receiver. Implemented once per platform.
struct GnssUart {
  static Stream& stream();
  static void begin(uint32_t baud);
  // Safe when the port isn't open (never started, or already ended).
  static void end();
};

}  // namespace gnimu
