#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../app/Console.h"
#include "../../core/BleProfile.h"
#include "../../core/BleStats.h"

namespace gnimu {

// Stand-in for boards whose BLE transport doesn't exist yet - the nRF's
// Bluefruit port arrives with the nRF parity work. It never connects, so
// nothing is ever sent, and the boot log says so plainly.
class BleStub {
 public:
  // True: there's no stack to fail, and a board without BLE still runs.
  bool begin(const BleProfile&, const char*, const char*, Console& log) {
    log.log("BLE: not implemented on this board yet - no telemetry will be sent");
    return true;
  }
  void end() {}
  void update() {}
  bool connected() const { return false; }
  bool send(const uint8_t*, size_t) { return false; }
  size_t read(uint8_t*, size_t) { return 0; }
  void setBatteryPercent(uint8_t) {}
  BleStats stats() const { return {}; }
  void clearMaxSend() {}
};

}  // namespace gnimu
