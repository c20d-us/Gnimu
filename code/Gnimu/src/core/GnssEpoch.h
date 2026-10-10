// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// The parts of a GNSS epoch the firmware acts on so far. The u-blox library's
// types stay out of core/ so this builds on a host.
struct GnssEpoch {
  uint64_t arrivalUs = 0;  // when the epoch was parsed, on the platform clock
  uint32_t iTowMs = 0;     // GPS time of week
  uint32_t hAccMm = 0;
  uint32_t tAccNs = 0;     // time accuracy estimate (UTC)
  int32_t latDeg1e7 = 0;   // degrees * 1e-7, same native scale as the PVT field
  int32_t lonDeg1e7 = 0;
  uint16_t pDop = 0;       // 0.01 units
  uint8_t fixType = 0;
  uint8_t numSv = 0;
  uint32_t gSpeedMmPerS = 0;  // ground speed
  bool gnssFixOk = false;      // PVT flags.gnssFixOK; distinct from fixType

  // The rest of NAV-PVT that the RaceBox packet carries, in PVT-native units.
  uint16_t year = 0;  // UTC
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  bool validDate = false;  // PVT valid.* bits
  bool validTime = false;
  bool fullyResolved = false;
  bool validMag = false;
  int32_t nanoNs = 0;  // fraction of second, -1e9..1e9
  int32_t heightMm = 0;  // above ellipsoid
  int32_t hMslMm = 0;    // above mean sea level
  uint32_t vAccMm = 0;
  int32_t headMotDeg1e5 = 0;
  uint32_t sAccMmPerS = 0;
  uint32_t headAccDeg1e5 = 0;
  bool headVehValid = false;  // PVT flags.headVehValid
};

// Counts epochs and spots the ones that never arrived, from gaps in iTOW.
class EpochTracker {
 public:
  explicit EpochTracker(uint32_t navPeriodMs) : periodMs_(navPeriodMs) {}

  void feed(const GnssEpoch& e) {
    epochs_++;
    if (have_) {
      const uint32_t dt = e.iTowMs >= lastITowMs_ ? e.iTowMs - lastITowMs_
                                                  : e.iTowMs + kWeekMs - lastITowMs_;
      if (dt > kMaxGapMs) {
        // A receiver with no time yet counts iTOW from zero, then jumps when it
        // gets time. That is a clock step, not epochs that failed to arrive.
        timeSteps_++;
      } else {
        const uint32_t slots = (dt + periodMs_ / 2) / periodMs_;
        if (slots > 1) missed_ += slots - 1;
      }
      const uint64_t gap = e.arrivalUs - lastArrivalUs_;
      const uint32_t gapUs = gap > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(gap);
      if (gapUs > maxGapUs_) maxGapUs_ = gapUs;
    }
    have_ = true;
    lastITowMs_ = e.iTowMs;
    lastArrivalUs_ = e.arrivalUs;
  }

  void clear() {
    epochs_ = missed_ = maxGapUs_ = timeSteps_ = 0;
    have_ = false;
  }

  uint32_t epochs() const { return epochs_; }
  uint32_t missed() const { return missed_; }
  uint32_t maxGapUs() const { return maxGapUs_; }
  uint32_t timeSteps() const { return timeSteps_; }

 private:
  static constexpr uint32_t kWeekMs = 604800000;
  static constexpr uint32_t kMaxGapMs = 5000;
  uint32_t periodMs_;
  uint32_t epochs_ = 0;
  uint32_t missed_ = 0;
  uint32_t maxGapUs_ = 0;
  uint32_t timeSteps_ = 0;
  uint32_t lastITowMs_ = 0;
  uint64_t lastArrivalUs_ = 0;
  bool have_ = false;
};

}  // namespace gnimu
