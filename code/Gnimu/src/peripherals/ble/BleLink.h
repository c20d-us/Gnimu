// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../app/Console.h"
#include "../../core/BleStats.h"
#include "../../platform/Clock.h"

namespace gnimu {

// The BLE rules both transports share, written once: refusing a frame the
// peer's MTU can't carry whole, counting what was sent, and logging connection
// edges. Each transport keeps only its stack calls and calls these around
// them. Loop task only: send() and update() are only called from jobs.
class BleLink {
 public:
  void setLog(Console& log) { log_ = &log; }

  // False, and counted as refused, when a `len`-byte frame won't fit in one
  // notification (MTU - 3 bytes): refused rather than split. A few refusals
  // right after connecting are normal - the phone hasn't finished the MTU
  // exchange yet - so only a refusal lasting kRefusalLogUs is logged, once
  // per connection.
  bool admit(size_t len, uint16_t mtu) {
    if (len + 3 <= mtu) {
      refusing_ = false;
      return true;
    }
    const uint64_t now = PlatformClock::nowUs();
    if (!refusing_) {
      refusing_ = true;
      refusingSinceUs_ = now;
    } else if (!refusalLogged_ && now - refusingSinceUs_ >= kRefusalLogUs) {
      refusalLogged_ = true;
      log_->log("BLE: peer MTU %u too small for %u-byte frames - not sending \xE2\x9D\x8C",
                static_cast<unsigned>(mtu), static_cast<unsigned>(len));
    }
    stats_.refused++;
    return false;
  }

  // Records one notify that started at `startUs`. Returns `ok`.
  bool sent(bool ok, uint64_t startUs) {
    const uint64_t us = PlatformClock::nowUs() - startUs;
    if (us > stats_.maxSendUs) stats_.maxSendUs = us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(us);
    if (ok) {
      stats_.sent++;
    } else {
      stats_.failed++;
    }
    return ok;
  }

  // Logs connection edges, and re-arms the refusal log on each connect.
  void edges(bool up) {
    if (up && !wasUp_) {
      log_->log("BLE: client connected");
      refusing_ = false;
      refusalLogged_ = false;
    } else if (!up && wasUp_) {
      log_->log("BLE: client disconnected");
    }
    wasUp_ = up;
  }

  // The send counters; the transport fills in its own RX fields.
  BleStats stats() const { return stats_; }
  void clearMaxSend() { stats_.maxSendUs = 0; }

 private:
  static constexpr uint64_t kRefusalLogUs = 1000000;

  Console* log_ = nullptr;
  bool wasUp_ = false;
  bool refusing_ = false;
  bool refusalLogged_ = false;
  uint64_t refusingSinceUs_ = 0;
  BleStats stats_;
};

}  // namespace gnimu
