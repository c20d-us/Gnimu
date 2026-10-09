#pragma once

#include "../../app/Console.h"
#include "../../core/GnssEpoch.h"

namespace gnimu {

// u-blox receiver: baud detection, startup configuration, and epoch capture.
// Every setting is read first (from RAM, the layer the receiver runs on) and
// only written, to every layer, if it doesn't already match - so a normal
// boot writes nothing, though every setting still logs whether it was
// already correct or had to be written. Because the write includes flash, it
// only has to succeed once per receiver.
class Gnss {
public:
  // Detects and configures the receiver, logging each step to `log`. Returns
  // false if the receiver was not found at any baud rate; a found receiver
  // returns true even if configuration failed (see configured()).
  bool begin(Console &log);
  // False if the last begin() left a setting wrong in RAM: the receiver
  // isn't running the configuration, which is fatal (GnssConfigFailed).
  // Settings that are right but not saved to flash are only warnings.
  bool configured() const { return configured_; }
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
  // False when a setting is left wrong in RAM (see configured()).
  bool configure();
  // Which layer a key is checked in: RAM, or flash for the one key
  // configure() deliberately holds off in RAM while it works.
  enum class Check : uint8_t { Ram, FlashOnly };
  // Checks one key and logs one line: already right, written, set but not
  // saved to flash (a warning), or could not be set (returns false). The
  // rules are in Gnss.cpp, the only place it's used. T is the key's value
  // width: uint8_t or uint16_t.
  template <typename T>
  bool ensureVal(uint32_t key, T desired, const char *name, Check check = Check::Ram);

  Console *log_ = nullptr;
  bool configured_ = false;
  unsigned flashWarnings_ = 0;  // this configure()'s "not saved to flash" keys
};

} // namespace gnimu
