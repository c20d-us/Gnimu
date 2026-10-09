#include "Mpu6050.h"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "../../app/Config.h"
#include "../../platform/ImuBus.h"
#include "ImuLog.h"

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

} // namespace

// One burst of the 14 data registers from ACCEL_XOUT_H: accel X/Y/Z,
// temperature (skipped), gyro X/Y/Z. False on a short or all-zero read.
bool Mpu6050::read(ImuSample &out) {
  uint8_t raw[14];
  if (!readRegs(kRegAccelOut, raw, sizeof(raw)))
    return false;

  out.accel[0] = (rawPair(&raw[0]) / kAccelCountsPerG) * kGravity;
  out.accel[1] = (rawPair(&raw[2]) / kAccelCountsPerG) * kGravity;
  out.accel[2] = (rawPair(&raw[4]) / kAccelCountsPerG) * kGravity;
  out.gyro[0] = (rawPair(&raw[8]) / kGyroCountsPerDps) * kDegToRad;
  out.gyro[1] = (rawPair(&raw[10]) / kGyroCountsPerDps) * kDegToRad;
  out.gyro[2] = (rawPair(&raw[12]) / kGyroCountsPerDps) * kDegToRad;

  remapAxes(out.accel, kAxisMap);
  remapAxes(out.gyro, kAxisMap);
  return !isZeroAccel(out);
}

bool Mpu6050::begin(Console &log, ImuSample &first) {
  imuBusBegin(config::kImuI2cClockHz);

  uint8_t whoAmI = 0;
  const bool found = readRegs(kRegWhoAmI, &whoAmI, 1) && whoAmI == kWhoAmIValue;
  imuStep(log, found, "MPU-6050 detected");
  if (!found)
    return false;

  // Reset to a known state, then wake on the X gyro's PLL, a steadier clock
  // than the internal oscillator the chip wakes on.
  const uint8_t reset = 0x80;
  const uint8_t wake = 0x01;
  writeRegs(kRegPwrMgmt1, &reset, 1);
  delay(100);
  writeRegs(kRegPwrMgmt1, &wake, 1);
  writeRegs(kRegSmplrtDiv, kImuConfig, sizeof(kImuConfig));

  // read() scales from these ranges at compile time - a write that didn't
  // land would mis-scale every sample.
  uint8_t pwr = 0;
  uint8_t cfg[sizeof(kImuConfig)] = {};
  const bool configured = readRegs(kRegPwrMgmt1, &pwr, 1) && pwr == wake &&
                          readRegs(kRegSmplrtDiv, cfg, sizeof(cfg)) &&
                          memcmp(cfg, kImuConfig, sizeof(cfg)) == 0;
  imuStep(log, configured, "configured, %dg %ddeg/s", config::kImuAccelRangeG,
          config::kImuGyroRangeDps);
  if (!configured)
    return false;

  // Let the gyro settle before the first sample, which seeds the filter.
  delay(100);
  const bool sampled = read(first);
  if (!sampled)
    imuStep(log, false, "first sample read failed");
  return sampled;
}

} // namespace gnimu
