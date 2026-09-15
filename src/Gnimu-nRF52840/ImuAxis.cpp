// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "ImuAxis.h"
#include <math.h>

ImuAxis::ImuAxis(float alpha, float transientThreshold) {
  alpha_ = alpha;
  transientThreshold_ = transientThreshold;
  reset(0.0f);
}

void ImuAxis::update(float rawValue) {
  // Measure deviation against the baseline before folding the sample in.
  float currentDeviation = fabs(rawValue - smoothedValue_);
  if (currentDeviation > maxDeviation_) {
    maxDeviation_ = currentDeviation;
    peakDeviationValue_ = rawValue;
  }

  // EMA baseline.
  smoothedValue_ = (alpha_ * rawValue) + ((1.0f - alpha_) * smoothedValue_);
}

float ImuAxis::read() {
  float valueToSend = smoothedValue_;

  // Blend weight w ramps 0 -> 1 as the peak deviation goes from 1x to 2x the
  // threshold. A threshold <= 0 disables blending.
  if (transientThreshold_ > 0.0f && maxDeviation_ > transientThreshold_) {
    float w = (maxDeviation_ / transientThreshold_) - 1.0f;
    if (w > 1.0f) {
      w = 1.0f;
    }
    valueToSend = smoothedValue_ + (w * (peakDeviationValue_ - smoothedValue_));
  }

  // Start the next window.
  maxDeviation_ = 0.0f;
  peakDeviationValue_ = smoothedValue_;

  return valueToSend;
}

void ImuAxis::reset(float baselineValue) {
  smoothedValue_ = baselineValue;
  peakDeviationValue_ = baselineValue;
  maxDeviation_ = 0.0f;
}
