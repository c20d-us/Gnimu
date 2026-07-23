// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
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

#include "gp_battery.h"
#include "config.h"
#include "gp_log.h"
#include "gp_power.h"

// Cached battery state
static BatteryStatus status = {0.0f, 0, false, false, false, false};

// EMA-smoothed cell voltage that feeds the DISPLAYED voltage/percent/LED.
// The cutoff-request debounce anchors on the FRESH per-run peak instead, so
// smoothing can never hide a genuine end-of-charge drop from the safety path.
static float smoothedVoltage = 0.0f;
static bool voltagePrimed = false;

// Debounce anchor for the cutoff request: millis() when the fresh peak first
// went below BATTERY_CUTOFF_V, or 0 when not currently below.
static unsigned long belowCutoffSinceMs = 0;

// Discharge curve, instantiated once from the config.h macro
namespace {
struct CurvePoint {
  float voltage;
  uint8_t percent;
};
const CurvePoint kCurve[] = BATTERY_DISCHARGE_CURVE;
const size_t kCurveLen = sizeof(kCurve) / sizeof(kCurve[0]);
} // namespace

// Enable/disable the XIAO's VBAT divider. Only connected while sampling so it
// doesn't waste current continuously.
static void dividerEnable(bool on) {
  digitalWrite(BATTERY_ADC_ENABLE_PIN, on ? LOW : HIGH); // active-low enable
}

// Recover cell voltage (volts) from a raw ADC count. Undoes the ADC scale,
// applies the internal 3.0 V reference, then the divider ratio.
static float rawToCellVolts(int raw) {
  const float adcMax = (float)((1UL << SAADC_RESOLUTION_BITS) - 1);
  const float adcVolts = ((float)raw / adcMax) * (SAADC_REFERENCE_MV / 1000.0f);
  return adcVolts * BATTERY_DIVIDER_RATIO;
}

// Map a resting cell voltage to 0-100% using the config.h curve (sorted
// high->low). Endpoints clamp; interior points linearly interpolate.
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

// Fold a fresh per-run peak into the smoothed voltage, refresh the cached
// status, and update the cutoff-request debounce anchor from the FRESH peak.
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
    belowCutoffSinceMs = 0; // any recovery resets the debounce
  }
}

// Non-blocking sampler: IDLE / SAMPLING state machine
// Every BATTERY_POLL_INTERVAL_MS a new run starts. During SAMPLING, at most
// one paced analogRead() per call, tracking min/max. Terminates on count.
// Peak (max) feeds ingestPeak(); min is only kept for symmetry / diagnostics.
enum SamplerState { S_IDLE, S_SAMPLING };
static SamplerState sState = S_IDLE;
static unsigned long sRunStartMs = 0;
static unsigned long sLastSampleUs = 0;
static int sMinAdc = 0;
static int sMaxAdc = 0;
static int sSampleCount = 0;

static void samplerBeginRun(unsigned long nowMs) {
  sRunStartMs = nowMs;
  sMinAdc = (int)((1UL << SAADC_RESOLUTION_BITS) - 1);
  sMaxAdc = 0;
  sSampleCount = 0;
  dividerEnable(true);
  // First read fires one full spacing from now, which also covers the divider
  // settle (spacing >> the ~200 us the XIAO divider needs).
  sLastSampleUs = micros();
  sState = S_SAMPLING;
}

static void samplerFinishRun(unsigned long nowMs) {
  dividerEnable(false);
  sState = S_IDLE;
  ingestPeak(rawToCellVolts(sMaxAdc), nowMs);
}

// Blocking one-shot for boot priming. Same algorithm as the async sampler,
// but busy-waits through the pacing. Blocks for
// ~BATTERY_SAMPLE_COUNT * BATTERY_SAMPLE_SPACING_US (~50 ms at defaults).
static void samplerBlockingPrime(unsigned long nowMs) {
  samplerBeginRun(nowMs);
  while (sSampleCount < BATTERY_SAMPLE_COUNT) {
    while ((int32_t)(micros() - sLastSampleUs) <
           (int32_t)BATTERY_SAMPLE_SPACING_US) {
      // busy-wait
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
  // powerBegin() has already configured the ADC (resolution + reference +
  // TACQ); we just own the divider-enable pin.
  pinMode(BATTERY_ADC_ENABLE_PIN, OUTPUT);
  dividerEnable(false);

#ifdef BATTERY_FAST_CHARGE
  // Select the ~100 mA charge pad (active-low per the XIAO's HICHG pin).
  pinMode(BATTERY_CHARGE_CURRENT_PIN, OUTPUT);
  digitalWrite(BATTERY_CHARGE_CURRENT_PIN, LOW);
#endif

  // Prime the smoothed voltage + cached status so the state-machine
  // classifier and telemetry consumers have a valid initial peak on return.
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
  // SAMPLING: at most one paced read per call, terminate on count.
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

uint8_t batteryProtocolByte() {
  const uint8_t percent = status.percent > 100 ? 100 : status.percent;
  return (status.charging ? 0x80 : 0x00) | (percent & 0x7F);
}
