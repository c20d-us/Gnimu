#include "Mpu6050.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../../app/Config.h"
#include "../../platform/ImuBus.h"

namespace gnimu {

namespace {

// MPU-6050 address and the registers this driver touches.
constexpr uint8_t kAddr = 0x68; // AD0 low
constexpr uint8_t kWhoAmIValue = 0x68;
constexpr uint8_t kRegSmplrtDiv = 0x19; // 0x19-0x1C are consecutive
constexpr uint8_t kRegAccelOut = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

// FS_SEL / AFS_SEL code for a full-scale range; the code lands in bits 4:3 of
// GYRO_CONFIG / ACCEL_CONFIG.
constexpr uint8_t accelRangeCode(int g) {
  return g == 2 ? 0 : g == 4 ? 1 : g == 8 ? 2 : 3;
}
constexpr uint8_t gyroRangeCode(int dps) {
  return dps == 250 ? 0 : dps == 500 ? 1 : dps == 1000 ? 2 : 3;
}
constexpr uint8_t kAccelRangeCode = accelRangeCode(config::kImuAccelRangeG);
constexpr uint8_t kGyroRangeCode = gyroRangeCode(config::kImuGyroRangeDps);

// SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG, written in one burst and read
// back to verify. Divider 0 samples at the DLPF's 1kHz output rate.
constexpr uint8_t kImuConfig[4] = {
    0x00,
    config::kImuDlpfCfg,
    static_cast<uint8_t>(kGyroRangeCode << 3),
    static_cast<uint8_t>(kAccelRangeCode << 3),
};

constexpr float kAccelCountsPerG = 16384.0f / (1 << kAccelRangeCode);
constexpr float kGyroCountsPerDps = kGyroRangeCode == 0   ? 131.0f
                                    : kGyroRangeCode == 1 ? 65.5f
                                    : kGyroRangeCode == 2 ? 32.8f
                                                          : 16.4f;
constexpr float kGravity = 9.80665f;      // m/s^2 per g
constexpr float kDegToRad = 0.017453293f; // rad/s per deg/s

const AxisMap kAxisMap = {
    {config::kMpu6050AxisSrc[0], config::kMpu6050AxisSrc[1],
     config::kMpu6050AxisSrc[2]},
    {config::kMpu6050AxisSign[0], config::kMpu6050AxisSign[1],
     config::kMpu6050AxisSign[2]}};

void writeRegs(uint8_t reg, const uint8_t *data, size_t len) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(data, len);
  (void)Wire.endTransmission();
}

// False on a short read. endTransmission(false) only queues the write on this
// core; the transaction completes inside requestFrom().
bool readRegs(uint8_t reg, uint8_t *out, size_t len) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  (void)Wire.endTransmission(false);
  if (Wire.requestFrom(kAddr, static_cast<uint8_t>(len),
                       static_cast<uint8_t>(true)) != len)
    return false;
  for (size_t i = 0; i < len; i++)
    out[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

int16_t rawPair(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) |
                              static_cast<uint16_t>(p[1]));
}

// One burst of the 14 data registers from ACCEL_XOUT_H: accel X/Y/Z,
// temperature (skipped), gyro X/Y/Z. False on a short read, or if all three
// accel axes are exactly zero - the chip's power-on state after a supply
// glitch, which reads back cleanly but is not a measurement.
bool readSample(ImuSample &out) {
  uint8_t raw[14];
  if (!readRegs(kRegAccelOut, raw, sizeof(raw)))
    return false;

  const int16_t ax = rawPair(&raw[0]);
  const int16_t ay = rawPair(&raw[2]);
  const int16_t az = rawPair(&raw[4]);
  if (ax == 0 && ay == 0 && az == 0)
    return false;

  out.accel[0] = (ax / kAccelCountsPerG) * kGravity;
  out.accel[1] = (ay / kAccelCountsPerG) * kGravity;
  out.accel[2] = (az / kAccelCountsPerG) * kGravity;
  out.gyro[0] = (rawPair(&raw[8]) / kGyroCountsPerDps) * kDegToRad;
  out.gyro[1] = (rawPair(&raw[10]) / kGyroCountsPerDps) * kDegToRad;
  out.gyro[2] = (rawPair(&raw[12]) / kGyroCountsPerDps) * kDegToRad;

  remapAxes(out.accel, kAxisMap);
  remapAxes(out.gyro, kAxisMap);
  return true;
}

} // namespace

void Mpu6050::step(bool ok, const char *fmt, ...) {
  char what[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(what, sizeof(what), fmt, ap);
  va_end(ap);
  log_->log("IMU: %s %s", what, ok ? "✅" : "❌");
}

// Wired straight to the supply, so nothing to power off; just stop reading.
void Mpu6050::end() { health_.markDown(); }

void Mpu6050::begin(Console &log) {
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

  imuBusBegin(config::kImuI2cClockHz);

  uint8_t whoAmI = 0;
  const bool found = readRegs(kRegWhoAmI, &whoAmI, 1) && whoAmI == kWhoAmIValue;
  step(found, "MPU-6050 detected");
  if (!found) {
    log_->log("❌ IMU: no IMU found, continuing without it - IMU fields will "
              "read zero");
    return;
  }

  // Reset to a known state, then wake on the X gyro's PLL, a steadier clock
  // than the internal oscillator the chip wakes on.
  const uint8_t reset = 0x80;
  const uint8_t wake = 0x01;
  writeRegs(kRegPwrMgmt1, &reset, 1);
  delay(100);
  writeRegs(kRegPwrMgmt1, &wake, 1);
  writeRegs(kRegSmplrtDiv, kImuConfig, sizeof(kImuConfig));

  // readSample() scales from these ranges at compile time - a write that
  // didn't land would mis-scale every sample.
  uint8_t pwr = 0;
  uint8_t cfg[sizeof(kImuConfig)] = {};
  const bool configured = readRegs(kRegPwrMgmt1, &pwr, 1) && pwr == wake &&
                          readRegs(kRegSmplrtDiv, cfg, sizeof(cfg)) &&
                          memcmp(cfg, kImuConfig, sizeof(cfg)) == 0;
  step(configured, "configured, %dg %ddeg/s", config::kImuAccelRangeG,
       config::kImuGyroRangeDps);
  if (!configured) {
    log_->log("❌ IMU: configuration failed, continuing without it - IMU "
              "fields will read zero");
    return;
  }

  // Let the gyro and filter settle before the seed read below.
  delay(100);
  health_.markUp();

  // Seed the filter with a real first reading rather than leaving it at zero,
  // so the first update() doesn't see an artificial jump (e.g. gravity on Z)
  // latched in as a spurious transient.
  ImuSample seed;
  if (readSample(seed))
    filter_.reset(seed);
}

void Mpu6050::poll(float speedMps, bool speedValid) {
  if (!health_.up())
    return;

  ImuSample sample;
  const bool ok = readSample(sample);
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
    log_->log(
        "❌ IMU: stopped answering - IMU fields will read zero until reboot");
  }
}

} // namespace gnimu
