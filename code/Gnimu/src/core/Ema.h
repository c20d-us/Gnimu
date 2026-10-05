#pragma once

namespace gnimu {

// A single-value exponential moving average. No transient/peak tracking: the
// parked blend logic in Classic's ImuAxis never fired in practice (its
// thresholds were sentinels), so this keeps only the part that did something.
class Ema {
 public:
  explicit Ema(float alpha) : alpha_(alpha) {}

  // Seeds the filter with a real reading rather than leaving it at zero, so
  // the first update() doesn't see an artificial jump (e.g. gravity on Z).
  void reset(float value) { value_ = value; }
  void update(float raw) { value_ = alpha_ * raw + (1.0f - alpha_) * value_; }
  float read() const { return value_; }

 private:
  float alpha_;
  float value_ = 0.0f;
};

}  // namespace gnimu
