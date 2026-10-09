#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../app/Console.h"
#include "../../core/BleProfile.h"
#include "../../core/BleStats.h"
#include "BleLink.h"

namespace gnimu {

// Protocol-agnostic BLE peripheral on the nRF52's Bluefruit stack, laid out
// from a BleProfile exactly like BleEsp32: one service with a notify (TX) and
// a write (RX) characteristic, plus Device Information. Same contract too -
// a frame the peer's MTU can't carry whole is refused, never split. Bluefruit
// runs its callbacks in its own tasks, so they only update atomics and queue
// inbound bytes; logging happens on the loop.
class BleNrf {
 public:
  // Starts advertising as `name`. The first call brings up the stack and GATT
  // table; a call after end() just resumes advertising. Idempotent.
  // True once advertising; false if the stack or advertising failed to
  // start, which leaves nothing for a client to connect to.
  bool begin(const BleProfile& profile, const char* name, const char* serial, Console& log);
  // Stops advertising and drops any connection; begin() can follow. Idempotent.
  void end();
  // Loop housekeeping: logs connection edges. Bluefruit restarts advertising
  // after a disconnect on its own.
  void update();
  bool connected() const;
  // Notifies one frame. False (not sent) when there's no connection, the peer
  // hasn't enabled notifications, its MTU can't carry the whole frame, or the
  // send queue stayed full for 100 ms.
  bool send(const uint8_t* data, size_t len);
  // Bytes the peer wrote to RX, in arrival order.
  size_t read(uint8_t* out, size_t max);
  // The standard Battery Service (0x180F), as Classic and NG publish it.
  // Updated, and notified to a subscribed client, only when the value changes.
  void setBatteryPercent(uint8_t percent);
  BleStats stats() const;
  void clearMaxSend() { link_.clearMaxSend(); }

 private:
  BleLink link_;  // MTU refusal, send counters, connection-edge logging
  bool built_ = false;
  bool started_ = false;
  uint8_t batteryPercent_ = 0xFF;  // forces the first write
};

}  // namespace gnimu
