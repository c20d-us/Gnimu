#pragma once

#include <stdint.h>

namespace gnimu {

// The parts of a GNSS epoch the firmware acts on so far. The u-blox library's
// types stay out of core/ so this builds on a host.
struct GnssEpoch {
  uint64_t arrivalUs = 0;  // when the epoch was parsed, on the platform clock
  uint32_t iTowMs = 0;     // GPS time of week
  uint32_t hAccMm = 0;
  uint16_t pDop = 0;       // 0.01 units
  uint8_t fixType = 0;
  uint8_t numSv = 0;
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
