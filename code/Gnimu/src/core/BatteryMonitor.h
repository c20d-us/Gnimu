#pragma once

#include <stdint.h>

namespace gnimu {

struct BatteryConfig {
  uint8_t samplesPerRun;
  uint32_t runIntervalMs;  // between sampling-run starts
  float emaAlpha;          // display smoothing across runs: 1.0 = raw
  float cutoffV;           // also 0% on the straight-line charge curve
  float criticalV;
  float warningV;
  float fullScaleV;  // 100% on the curve
  uint32_t cutoffDebounceMs;
  float chargeFullV;  // plausibility floor before "full" can show
  uint32_t chargeFullDebounceMs;
};

struct BatteryStatus {
  bool valid = false;  // false until the first run completes
  float voltage = 0.0f;       // smoothed: display only (percent, warning, critical)
  float freshVoltage = 0.0f;  // latest run's peak, unsmoothed: the cutoff's input
  uint8_t percent = 0;
  bool warning = false;
  bool critical = false;
  bool cutoffRequested = false;
  bool chargerActive = false;  // the charger's CHG pin, as last read
  bool chargeFull = false;
};

// The one definition of "charging" for everything shown or sent: the CHG pin
// says the charger is charging, and USB is present. The charger can't charge
// without USB, so the USB check only guards against a floating or noisy pin
// on battery ever claiming a charge.
inline bool isCharging(const BatteryStatus& b, bool usbPresent) { return usbPresent && b.chargerActive; }

// Classic's g_battery sampler, as NG ported it. Each run keeps the PEAK of
// its readings: load sag (GNSS acquisition current, the TPS) only ever pulls
// a reading down, so the peak is the best estimate of the resting voltage.
// Smoothing is display-only; the cutoff watches each run's unsmoothed peak,
// so smoothing can never hide a genuinely low cell from the safety path.
class BatteryMonitor {
 public:
  explicit BatteryMonitor(const BatteryConfig& c) : c_(c) {}

  // True while a sampling run is open, opening one immediately if no run has
  // ever been taken or the run interval has elapsed. Call once per sample
  // period and take exactly one reading each time it returns true.
  bool sampling(uint32_t nowMs) {
    if (!running_ && (!everStarted_ || nowMs - runStartMs_ >= c_.runIntervalMs)) {
      running_ = true;
      everStarted_ = true;
      runStartMs_ = nowMs;
      count_ = 0;
      peakMv_ = 0;
    }
    return running_;
  }

  void addSample(uint16_t cellMv, uint32_t nowMs) {
    if (cellMv > peakMv_) peakMv_ = cellMv;
    if (++count_ >= c_.samplesPerRun) finishRun(nowMs);
  }

  // NG's rule: the charger's status line alone can't say "full" - outside a
  // charge it reads inactive too - so it must stay inactive for a while AND
  // the cell must plausibly be near the top.
  void updateCharger(bool chargerActive, uint32_t nowMs) {
    s_.chargerActive = chargerActive;
    if (chargerActive) {
      chargeInactivePending_ = false;
      s_.chargeFull = false;
      return;
    }
    if (!chargeInactivePending_) {
      chargeInactivePending_ = true;
      chargeInactiveSinceMs_ = nowMs;
    }
    if (!s_.chargeFull && s_.valid && s_.freshVoltage >= c_.chargeFullV &&
        nowMs - chargeInactiveSinceMs_ >= c_.chargeFullDebounceMs)
      s_.chargeFull = true;
  }

  const BatteryStatus& status() const { return s_; }

 private:
  void finishRun(uint32_t nowMs) {
    running_ = false;
    const float fresh = peakMv_ / 1000.0f;
    s_.freshVoltage = fresh;
    s_.voltage = s_.valid ? s_.voltage + c_.emaAlpha * (fresh - s_.voltage) : fresh;
    s_.valid = true;
    s_.percent = percentFor(s_.voltage);
    s_.warning = s_.voltage <= c_.warningV;
    s_.critical = s_.voltage <= c_.criticalV;
    if (fresh < c_.cutoffV) {
      if (!belowCutoff_) {
        belowCutoff_ = true;
        belowCutoffSinceMs_ = nowMs;
      }
      if (nowMs - belowCutoffSinceMs_ >= c_.cutoffDebounceMs) s_.cutoffRequested = true;
    } else {
      belowCutoff_ = false;
      s_.cutoffRequested = false;
    }
  }

  // A deliberate straight line between cutoff and full-scale, not a real
  // LiPo discharge curve (Classic's own choice).
  uint8_t percentFor(float v) const {
    if (v <= c_.cutoffV) return 0;
    if (v >= c_.fullScaleV) return 100;
    return static_cast<uint8_t>((v - c_.cutoffV) * (100.0f / (c_.fullScaleV - c_.cutoffV)) + 0.5f);
  }

  BatteryConfig c_;
  BatteryStatus s_;
  bool running_ = false;
  bool everStarted_ = false;
  uint32_t runStartMs_ = 0;
  uint8_t count_ = 0;
  uint16_t peakMv_ = 0;
  bool belowCutoff_ = false;
  uint32_t belowCutoffSinceMs_ = 0;
  bool chargeInactivePending_ = false;
  uint32_t chargeInactiveSinceMs_ = 0;
};

}  // namespace gnimu
