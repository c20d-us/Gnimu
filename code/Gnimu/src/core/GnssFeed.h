// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "GnssEpoch.h"
#include "GnssSpeedGate.h"

namespace gnimu {

// Whether epochs are arriving. NeverStarted: none at all within the silence
// window of the first check - the receiver answers but sends nothing.
// Stopped: epochs arrived, then none for the window - it stopped mid-run.
enum class GnssLiveness : uint8_t { Ok, NeverStarted, Stopped };

// Everything kept about the epochs that arrive: the gap/step statistics, the
// latest epoch, and the ground speed the IMU trim gates on. Fed epochs, not
// the receiver, so it stays free of hardware and host-testable.
class GnssFeed {
 public:
  GnssFeed(uint32_t navPeriodMs, uint64_t speedStaleAfterUs, uint64_t silenceUs)
      : epochs_(navPeriodMs), speedGate_(speedStaleAfterUs), silenceUs_(silenceUs) {}

  void add(const GnssEpoch& e) {
    epochs_.feed(e);
    speedGate_.feed(e);
    latest_ = e;
    haveEpoch_ = true;
  }

  // Called from the GNSS job. The first call starts the "never started"
  // window - the first moment epochs can be read at all - so boot time spent
  // before the scheduler runs doesn't count against it.
  GnssLiveness liveness(uint64_t nowUs) {
    if (!armed_) {
      armed_ = true;
      armedUs_ = nowUs;
    }
    if (!haveEpoch_) return nowUs - armedUs_ >= silenceUs_ ? GnssLiveness::NeverStarted : GnssLiveness::Ok;
    const uint64_t last = latest_.arrivalUs;
    return nowUs > last && nowUs - last >= silenceUs_ ? GnssLiveness::Stopped : GnssLiveness::Ok;
  }

  const EpochTracker& epochs() const { return epochs_; }
  // nullptr until the first epoch arrives.
  const GnssEpoch* latest() const { return haveEpoch_ ? &latest_ : nullptr; }
  float speedMps() const { return speedGate_.speedMps(); }
  bool speedValid(uint64_t nowUs) const { return speedGate_.valid(nowUs); }
  // Clears the statistics (console C); the latest epoch and speed stay.
  void clear() { epochs_.clear(); }

 private:
  EpochTracker epochs_;
  GnssSpeedGate speedGate_;
  GnssEpoch latest_;
  bool haveEpoch_ = false;
  uint64_t silenceUs_;
  bool armed_ = false;
  uint64_t armedUs_ = 0;
};

}  // namespace gnimu
