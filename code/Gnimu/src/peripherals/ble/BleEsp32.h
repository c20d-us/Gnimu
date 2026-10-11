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

// Protocol-agnostic BLE peripheral on the ESP32's Bluedroid stack: one service
// with a notify (TX) and a write (RX) characteristic, plus Device Information,
// laid out from a BleProfile. Stack callbacks run on Bluedroid's task on the
// other core, so they only update atomics and queue inbound bytes; everything
// else, including logging, happens on the loop.
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
  // Notifies one frame. False (not sent) when there's no connection, the
  // peer's MTU can't carry the whole frame, or the stack didn't take it
  // (notifications not enabled, for one).
  bool send(const uint8_t* data, size_t len);
  // Bytes the peer wrote to RX, in arrival order.
  size_t read(uint8_t* out, size_t max);
  // No Battery Service on the ESP32: it has no battery.
  void setBatteryPercent(uint8_t) {}
  BleStats stats() const;
  void clearMaxSend() { link_.clearMaxSend(); }

 private:
  BleLink link_;  // MTU refusal, send counters, connection-edge logging
  bool built_ = false;
  bool started_ = false;
};

}  // namespace gnimu
