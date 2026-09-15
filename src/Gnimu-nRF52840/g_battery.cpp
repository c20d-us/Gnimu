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

#include "g_battery.h"
#include "config.h"
#include "g_log.h"
#include "g_power.h"

// Discharge curve from BATTERY_DISCHARGE_CURVE.
struct CurvePoint {
  float voltage;
  uint8_t percent;
};
static const CurvePoint kCurve[] = BATTERY_DISCHARGE_CURVE;
static const size_t kCurveLen = sizeof(kCurve) / sizeof(kCurve[0]);

static BatteryStatus status = {0.0f, 0, false, false, false, false};

// Smoothed voltage for display. The cutoff uses the unsmoothed peak.
static float smoothedVoltage = 0.0f;
static bool voltagePrimed = false;

// millis() when the peak first dropped below BATTERY_CUTOFF_V, or 0.
static unsigned long belowCutoffSinceMs = 0;

// Non-blocking sampler. A run starts every BATTERY_POLL_INTERVAL_MS and takes
// BATTERY_SAMPLE_COUNT paced reads. The max feeds ingestPeak(); the min is
// unused.
enum SamplerState { S_IDLE, S_SAMPLING };
static SamplerState sState = S_IDLE;
static unsigned long sRunStartMs = 0;
static unsigned long sLastSampleUs = 0;
static int sMinAdc = 0;
static int sMaxAdc = 0;
static int sSampleCount = 0;

// The VBAT divider is enabled only while sampling.
static void dividerEnable(bool on) {
  digitalWrite(BATTERY_ADC_ENABLE_PIN, on ? LOW : HIGH); // active-low
}

// Convert a raw ADC count to cell volts.
static float rawToCellVolts(int raw) {
  const float adcMax = (float)((1UL << SAADC_RESOLUTION_BITS) - 1);
  const float adcVolts = ((float)raw / adcMax) * (SAADC_REFERENCE_MV / 1000.0f);
  return adcVolts * BATTERY_DIVIDER_RATIO;
}

// Interpolate percent from the curve (sorted high to low), clamping at the
// ends.
static uint8_t voltageToPercent(float v) {
  if (v >= kCurve[0].voltage)
    return kCurve[0].percent;
  if (v <= kCurve[kCurveLen - 1].voltage)
    return kCurve[kCurveLen - 1].percent;
  for (size_t i = 1; i < kCurveLen; i++) {
    if (v >= kCurve[i].voltage) {
      const CurvePoint &hi = kCurve[i - 1];
      const CurvePoint &lo = kCurve[i];
      const float span = hi.voltage - lo.voltage;
      const float frac = span > 0.0f ? (v - lo.voltage) / span : 0.0f;
      return (uint8_t)(lo.percent + frac * (hi.percent - lo.percent) + 0.5f);
    }
  }
  return kCurve[kCurveLen - 1].percent; // unreachable
}

// Fold a run's peak into the smoothed voltage, refresh the status, and update
// the cutoff debounce from the unsmoothed peak.
static void ingestPeak(float freshVoltage, unsigned long nowMs) {
  if (!voltagePrimed) {
    smoothedVoltage = freshVoltage;
    voltagePrimed = true;
  } else {
    smoothedVoltage = BATTERY_EMA_ALPHA * freshVoltage +
                      (1.0f - BATTERY_EMA_ALPHA) * smoothedVoltage;
  }

  status.voltage = smoothedVoltage;
  status.percent = voltageToPercent(smoothedVoltage);
  status.charging = powerUsbPresent() && powerSwitchOn();
  status.warn = smoothedVoltage <= BATTERY_WARN_V;
  status.critical = smoothedVoltage <= BATTERY_CRITICAL_V;
  status.full = status.charging && smoothedVoltage >= BATTERY_FULL_V;

  if (freshVoltage < BATTERY_CUTOFF_V) {
    if (belowCutoffSinceMs == 0)
      belowCutoffSinceMs = nowMs;
  } else {
    belowCutoffSinceMs = 0;
  }
}

static void samplerBeginRun(unsigned long nowMs) {
  sRunStartMs = nowMs;
  sMinAdc = (int)((1UL << SAADC_RESOLUTION_BITS) - 1);
  sMaxAdc = 0;
  sSampleCount = 0;
  dividerEnable(true);
  // The first read waits one spacing, which also lets the divider settle.
  sLastSampleUs = micros();
  sState = S_SAMPLING;
}

static void samplerFinishRun(unsigned long nowMs) {
  dividerEnable(false);
  sState = S_IDLE;
  ingestPeak(rawToCellVolts(sMaxAdc), nowMs);
}

// Blocking run for boot, ~BATTERY_SAMPLE_COUNT * BATTERY_SAMPLE_SPACING_US.
static void samplerBlockingPrime(unsigned long nowMs) {
  samplerBeginRun(nowMs);
  while (sSampleCount < BATTERY_SAMPLE_COUNT) {
    while ((int32_t)(micros() - sLastSampleUs) <
           (int32_t)BATTERY_SAMPLE_SPACING_US) {
    }
    sLastSampleUs = micros();
    const int v = analogRead(BATTERY_ADC_PIN);
    if (v < sMinAdc)
      sMinAdc = v;
    if (v > sMaxAdc)
      sMaxAdc = v;
    sSampleCount++;
  }
  samplerFinishRun(millis());
}

void batteryBegin() {
  pinMode(BATTERY_ADC_ENABLE_PIN, OUTPUT);
  dividerEnable(false);

#ifdef BATTERY_FAST_CHARGE
  // ~100mA charge current (HICHG, active-low).
  pinMode(BATTERY_CHARGE_CURRENT_PIN, OUTPUT);
  digitalWrite(BATTERY_CHARGE_CURRENT_PIN, LOW);
#endif

  samplerBlockingPrime(millis());

  LOG_PRINTF("🔋 Battery: %.2f V %s\n", status.voltage,
             status.charging ? " [charging]" : "");
}

void batteryPoll() {
  const unsigned long nowMs = millis();
  if (sState == S_IDLE) {
    if (nowMs - sRunStartMs >= BATTERY_POLL_INTERVAL_MS)
      samplerBeginRun(nowMs);
    return;
  }
  const unsigned long nowUs = micros();
  if ((int32_t)(nowUs - sLastSampleUs) < (int32_t)BATTERY_SAMPLE_SPACING_US)
    return;
  sLastSampleUs = nowUs;
  const int v = analogRead(BATTERY_ADC_PIN);
  if (v < sMinAdc)
    sMinAdc = v;
  if (v > sMaxAdc)
    sMaxAdc = v;
  sSampleCount++;
  if (sSampleCount >= BATTERY_SAMPLE_COUNT)
    samplerFinishRun(nowMs);
}

BatteryStatus batteryGetStatus() { return status; }

bool batteryCutoffRequested() {
  if (belowCutoffSinceMs == 0)
    return false;
  return (millis() - belowCutoffSinceMs) >= BATTERY_CUTOFF_DEBOUNCE_MS;
}
