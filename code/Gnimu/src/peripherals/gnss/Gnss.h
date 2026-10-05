#pragma once

#include "../../app/Console.h"
#include "../../core/GnssEpoch.h"

namespace gnimu {

// u-blox receiver: baud detection, startup configuration, and epoch capture.
// Every setting is read first and only written (to flash, via ensureVal8/16)
// if it doesn't already match - so a normal boot writes nothing, though every
// setting still logs whether it was already correct or had to be written.
// The risky write-and-ACK path only has to succeed once per receiver, ever.
class Gnss {
public:
  // Detects and configures the receiver, logging each step to `log`. Returns
  // false if the receiver was not found at any baud rate.
  bool begin(Console &log);
  // Drains the UART and parses; call at least every drain period.
  void poll();
  // Returns true and fills `out` once per new epoch.
  bool consumeEpoch(GnssEpoch &out);
  // Closes the UART, so its TX pin can be idled low before the rail goes off.
  // Leaving a stop goes through a reset, so there's no restart path.
  void end();

private:
  bool tryBaud(uint32_t baud);
  bool switchToTargetBaud();
  bool connectAtBaud();
  void configure();
  // Reads the key from the FLASH layer and logs one line either way: that it
  // already matched `desired`, or that it had to be written (to VAL_LAYER_ALL)
  // and read back from flash to confirm. Flash persistence means the write
  // only has to succeed once per receiver, ever. ensureVal8/16 are thin
  // wrappers so call sites don't need to name the template argument; the
  // shared logic lives once in ensureVal<T>, defined in Gnss.cpp and used
  // only there, so it never needs to appear in this header.
  template <typename T>
  bool ensureVal(uint32_t key, T desired, const char *name);
  bool ensureVal8(uint32_t key, uint8_t desired, const char *name);
  bool ensureVal16(uint32_t key, uint16_t desired, const char *name);

  Console *log_ = nullptr;
};

} // namespace gnimu
