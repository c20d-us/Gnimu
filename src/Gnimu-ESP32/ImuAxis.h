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

#pragma once
#include <Arduino.h>

// IMU axis: one sensor axis with low-pass filtering and transient peak
// detection.
class ImuAxis {
private:
  float smoothedValue_;
  float peakDeviationValue_;
  float maxDeviation_;
  float alpha_;
  float transientThreshold_;

public:
  // alpha: low-pass weight, 0.0-1.0 (lower is smoother).
  // transientThreshold: deviation from baseline at which the raw peak blends in.
  ImuAxis(float alpha, float transientThreshold);

  // Fold in one raw sample. Called at the sample rate.
  void update(float rawValue);

  // Return this window's value and start a new window. Called at the send rate.
  // The result is the baseline blended toward the window's raw peak: pure
  // baseline at or below the threshold, full peak at 2x or beyond.
  float read();

  // Reset the filter and peak tracker to baselineValue.
  void reset(float baselineValue = 0.0f);
};
