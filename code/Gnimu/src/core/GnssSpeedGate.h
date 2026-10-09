// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "GnssEpoch.h"

namespace gnimu {

// Ground speed for the trim motion gate, with staleness tracking so a
// receiver that stops updating doesn't leave a stale "stationary" reading
// trusted forever. Staleness is judged by time since the last epoch arrived,
// on the platform clock, so it doesn't depend on how often anyone asks.
class GnssSpeedGate {
 public:
  // staleAfterUs: how long after the last epoch's arrival its speed stops
  // counting as valid.
  explicit GnssSpeedGate(uint64_t staleAfterUs) : staleAfterUs_(staleAfterUs) {}

  // Call once per fresh epoch.
  void feed(const GnssEpoch& e) {
    speedMps_ = static_cast<float>(e.gSpeedMmPerS) * 0.001f;
    // The same strict "valid" GNSS reserves for lat/lon: fixType 3 alone
    // isn't enough, since gnssFixOK is a distinct validity bit.
    fixOk_ = (e.fixType == 3) && e.gnssFixOk;
    lastArrivalUs_ = e.arrivalUs;
    haveEpoch_ = true;
  }

  bool valid(uint64_t nowUs) const {
    return haveEpoch_ && fixOk_ && nowUs - lastArrivalUs_ < staleAfterUs_;
  }
  float speedMps() const { return speedMps_; }

 private:
  uint64_t staleAfterUs_;
  uint64_t lastArrivalUs_ = 0;
  float speedMps_ = 0.0f;
  bool fixOk_ = false;
  bool haveEpoch_ = false;
};

}  // namespace gnimu
