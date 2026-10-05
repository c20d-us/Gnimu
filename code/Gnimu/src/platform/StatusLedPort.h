#pragma once

#include "../core/StatusLed.h"

namespace gnimu {

// The board's status LED. Implemented once per platform; core/StatusLed
// decides what to show, this only renders it. A single-color LED ignores
// `color`.
struct StatusLedPort {
  static void begin();  // configures the pin(s), LED off
  static void write(bool on, LedColor color);
};

}  // namespace gnimu
