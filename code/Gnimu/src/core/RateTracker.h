#pragma once

#include <stdint.h>

namespace gnimu {

// Events-per-second of a running counter (GNSS epochs, BLE packets sent)
// between successive samples, not the cumulative count: a live rate shows a
// stall the instant it happens, rather than waiting for a cumulative counter
// to catch up over several seconds.
class RateTracker {
 public:
  // Call when periodic reporting (re)starts, so the first sample() measures
  // the interval from a clean baseline instead of from zero, and whenever the
  // counter is cleared (console C), which would otherwise read as a wrap.
  void reset(uint32_t count, uint64_t nowUs) {
    lastCount_ = count;
    lastUs_ = nowUs;
  }

  // Centihertz (Hz * 100) of events since the last reset()/sample() call.
  // Integer-only - no float format specifier, same reasoning as Report.h.
  uint32_t sampleCentiHz(uint32_t count, uint64_t nowUs) {
    const uint64_t elapsedUs = nowUs - lastUs_;
    const uint32_t delta = count - lastCount_;
    reset(count, nowUs);
    if (elapsedUs == 0) return 0;
    // events/s * 100 == delta * 100 * 1,000,000 / elapsedUs
    return static_cast<uint32_t>((static_cast<uint64_t>(delta) * 100000000ULL) / elapsedUs);
  }

 private:
  uint32_t lastCount_ = 0;
  uint64_t lastUs_ = 0;
};

}  // namespace gnimu
