// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../app/Console.h"
#include "../../core/BleProfile.h"
#include "../../core/BleStats.h"
#include "BleLink.h"

namespace gnimu {

// Protocol-agnostic BLE peripheral on the ESP32's Bluedroid stack: the
// services and characteristics a BleProfile lists, plus Device Information.
// Stack callbacks run on Bluedroid's task on the other core, so they only
// update atomics and queue inbound bytes; everything else, including logging,
// happens on the loop.
class BleEsp32 {
 public:
  // Starts advertising as `name`. The first call builds the stack and GATT
  // table; a call after end() just resumes advertising. Idempotent.
  // True once advertising; false if the stack or advertising failed to
  // start, which leaves nothing for a client to connect to.
  bool begin(const BleProfile& profile, const char* name, const char* serial, Console& log);
  // Stops advertising and drops any connection. Deliberately not a stack
  // teardown - on the ESP32 that can't be undone without a reboot - so begin()
  // can follow. Idempotent.
  void end();
  // Loop housekeeping: logs connection edges. A connection shorter than one
  // pass goes unlogged; re-advertising doesn't depend on it.
  void update();
  bool connected() const;
  // Notifies one frame on characteristic `index` of the profile. False (not
  // sent) when there's no connection, the peer's MTU can't carry the whole
  // frame, or the stack didn't take it (notifications not enabled, for one).
  bool send(uint8_t index, const uint8_t* data, size_t len);
  // Bytes the peer wrote to characteristic `index`, in arrival order. Always 0
  // for a characteristic that isn't writable, or kBleNoChar.
  size_t read(uint8_t index, uint8_t* out, size_t max);
  // No Battery Service on the ESP32: it has no battery.
  void setBatteryPercent(uint8_t) {}
  BleStats stats() const;
  // Console C: every counter in stats() starts again from zero.
  void clearStats();

 private:
  BleLink link_;  // MTU refusal, send counters, connection-edge logging
  // The inbound totals at the last clearStats(). The queues' own counters are
  // written by the stack's task, so they're never reset, only measured from.
  uint32_t rxBytesBase_ = 0;
  uint32_t rxDroppedBase_ = 0;
  bool built_ = false;
  bool started_ = false;
};

}  // namespace gnimu
