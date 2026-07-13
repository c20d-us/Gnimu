#include "ImuAxis.h"
#include <math.h> // For fabs()

// Constructor for the IMU Axis object.
ImuAxis::ImuAxis(float alpha, float transientThreshold) {
  alpha_ = alpha;
  transientThreshold_ = transientThreshold;
  reset(0.0f);
}

// Update the filter.
void ImuAxis::update(float rawValue) {
  // Check deviation against the current baselinebefore folding in thissample.
  float currentDeviation = fabs(rawValue - smoothedValue_);
  if (currentDeviation > maxDeviation_) {
    maxDeviation_ = currentDeviation;
    peakDeviationValue_ = rawValue; // Capture the true raw peak
  }

  // Advance the Exponential Moving Average baseline.
  smoothedValue_ = (alpha_ * rawValue) + ((1.0f - alpha_) * smoothedValue_);
}

float ImuAxis::read() {
  // Default to the smoothed baseline.
  float valueToSend = smoothedValue_;

  // Blend the captured raw peak into the baseline in proportion to how far the
  // largest in-window deviation exceeded the transient threshold, rather than
  // hard-switching between the two. Blend weight w:
  //   w = 0          at maxDeviation == threshold        -> pure smoothed
  //   w ramps 0 -> 1 between threshold and 2*threshold   -> partial peak
  //   w = 1          at maxDeviation >= 2*threshold       -> full raw peak
  // The proportional ramp removes the threshold-boundary flicker of a hard
  // switch (a marginal event no longer snaps the output between baseline and
  // full peak frame to frame) while still surfacing genuine transients that a
  // low-alpha EMA would otherwise wash out. Strong events (>= 2x threshold)
  // still yield the full raw peak.
  //
  // The threshold guard also protects the division from a zero/negative
  // (misconfigured) threshold, in which case no blending is applied.
  if (transientThreshold_ > 0.0f && maxDeviation_ > transientThreshold_) {
    float w = (maxDeviation_ / transientThreshold_) - 1.0f;
    if (w > 1.0f) {
      w = 1.0f;
    }
    valueToSend = smoothedValue_ + (w * (peakDeviationValue_ - smoothedValue_));
  }

  // Reset window metrics for the next 40ms cycle
  maxDeviation_ = 0.0f;
  peakDeviationValue_ = smoothedValue_;

  return valueToSend;
}

void ImuAxis::reset(float baselineValue) {
  smoothedValue_ = baselineValue;
  peakDeviationValue_ = baselineValue;
  maxDeviation_ = 0.0f;
}
