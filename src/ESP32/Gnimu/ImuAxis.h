#pragma once
#include <Arduino.h>

// ============================================================================
// IMU Axis module - accelerometer + gyroscope data with integrated filtering
// ============================================================================

// The IMU Axis class represents a single axis of an IMU sensor, providing
// low-pass filtering and transient peak detection for telemetry payloads.
class ImuAxis {
private:
  float smoothedValue_;
  float peakDeviationValue_;
  float maxDeviation_;
  float alpha_;
  float transientThreshold_;

public:
  // IMU Axis object constructor.
  // alpha: Low-pass filter weight (0.0 to 1.0).Lower values mean more
  //    smoothing.
  // transientThreshold: The deviation delta from baseline required to
  //    trigger raw peak transmission.
  ImuAxis(float alpha, float transientThreshold);

  // Update the filter. Called at the fast sampling rate.
  // rawValue: The raw sensor reading from the IMU.
  void update(float rawValue);

  // Read the downsampled value for this window. Called at your transmission
  // rate.
  //
  // Returns the smoothed baseline blended toward the window's raw peak in
  // proportion to how far the largest deviation exceeded the transient
  // threshold: pure baseline at/below the threshold, full raw peak at or beyond
  // 2x the threshold, and a proportional mix in between.
  float read();

  // Resets the internal filters and peak trackers to a baseline value.
  //
  // baselineValue: The value to reset the filters and peak trackers to.
  void reset(float baselineValue = 0.0f);
};
