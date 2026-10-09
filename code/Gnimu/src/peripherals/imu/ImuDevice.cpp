#include "ImuDevice.h"

#include "../../app/SystemConfig.h"

namespace gnimu {

void ImuDevice::begin(Console& log) {
  log_ = &log;
  filter_ = ImuFilter(config::kImuAccelAlpha, config::kImuGyroAlpha);
  health_ = ImuHealth(config::kImuDownAfterFailures);
  trim_ = ImuTrim(ImuTrimConfig{
      config::kImuTrimGravityNative,
      static_cast<float>(config::kImuSamplePeriodUs) / 1000.0f,
      config::kImuTrimQualifyMs,
      config::kImuTrimBlockMs,
      config::kImuTrimLockBlocks,
      config::kImuTrimSpeedMaxMps,
      config::kImuTrimAccelSanityTol,
      config::kImuTrimAccelVarMax,
      config::kImuTrimGyroVarMax,
      config::kImuTrimMaxTiltDeg,
      config::kImuTrimRequireFix,
  });

  // The driver logs its own steps, and which one failed.
  ImuSample first;
  if (!chip_.begin(log, first)) {
    log.log("\xE2\x9D\x8C IMU: continuing without it - IMU fields will read zero");
    return;
  }
  health_.markUp();
  // Seeded with a real first reading rather than left at zero, so the first
  // update() doesn't see an artificial jump (e.g. gravity on Z) latched in as
  // a spurious transient.
  filter_.reset(first);
}

void ImuDevice::poll(float speedMps, bool speedValid) {
  if (!health_.up()) return;

  ImuSample sample;
  const bool ok = chip_.read(sample);
  if (ok) {
    // Trim runs on the raw, uncorrected, remapped sample and BEFORE the EMA
    // filter - feeding it filtered data would smear the stillness gate's
    // variance test, and feeding it its own output would close the loop and
    // make the reported tilt decay toward zero as the correction converged.
    trim_.update(sample.accel, sample.gyro, speedMps, speedValid);
    trim_.apply(sample.accel, sample.gyro);
    filter_.update(sample);
  }
  if (health_.feed(ok)) {
    log_->log("\xE2\x9D\x8C IMU: stopped answering - IMU fields will read zero until reboot");
  }
}

void ImuDevice::end() {
  health_.markDown();
  chip_.end();
}

}  // namespace gnimu
