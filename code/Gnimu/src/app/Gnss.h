#pragma once

#include "../core/GnssEpoch.h"
#include "Console.h"

namespace gnimu {

// u-blox receiver: baud detection, startup configuration, and epoch capture.
// Configuration goes to RAM+BBR only; the baud rate is written to flash, and
// only when it had to change.
class Gnss {
 public:
  // Detects and configures the receiver, logging each step to `log`. Returns
  // false if the receiver was not found at any baud rate.
  bool begin(Console& log);
  // Drains the UART and parses; call at least every drain period.
  void poll();
  // Returns true and fills `out` once per new epoch.
  bool consumeEpoch(GnssEpoch& out);

 private:
  bool tryBaud(uint32_t baud);
  bool switchToTargetBaud();
  bool connectAtBaud();
  void configure();
  void configureConstellations();
  void step(bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

  Console* log_ = nullptr;
};

}  // namespace gnimu
