// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: see Clock.cpp for why. This chip is
// only actually populated on Sense/Sense Plus, but the vendor library itself
// degrades gracefully (falls back to the primary Wire, finds nothing, and
// this class reports "not found") on a board that lacks it, matching the
// same never-halt policy that already covers a completely absent IMU.
#if defined(NRF52840_XXAA)

#include "Lsm6ds3.h"

#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <stdio.h>

#include "../../app/Config.h"
#include "../../platform/nrf52840/BoardConfig.h"
#include "ImuLog.h"

namespace gnimu {

namespace {

constexpr uint8_t kAddr = 0x6A; // SA0 tied high on the XIAO Sense
constexpr uint16_t kAccelOdrHz =
    104; // output data rate; >= the 100 Hz poll rate
constexpr uint16_t kGyroOdrHz = 104;
constexpr uint16_t kAccelBandwidthHz = 50; // anti-alias filter

// The control-register bytes begin() plus our CTRL3_C write must leave behind,
// built from the library's own field constants. The library maps an
// unsupported range to its default (16 g / 2000 deg/s) while calcAccel() and
// calcGyro() still scale by the range asked for, so an unsupported range has
// to fail the build here instead.
constexpr bool isLsm6ds3AccelRange(int g) { return g == 2 || g == 4 || g == 8 || g == 16; }
constexpr bool isLsm6ds3GyroRange(int dps) {
  return dps == 125 || dps == 245 || dps == 500 || dps == 1000 || dps == 2000;
}
static_assert(isLsm6ds3AccelRange(config::kImuAccelRangeG), "not an LSM6DS3 accelerometer range");
static_assert(isLsm6ds3GyroRange(config::kImuGyroRangeDps), "not an LSM6DS3 gyro range");
static_assert(kAccelOdrHz == 104 && kGyroOdrHz == 104 && kAccelBandwidthHz == 50,
              "kCtrl1Xl/kCtrl2G below encode 104 Hz and a 50 Hz bandwidth");

// Each field is cast to uint8_t: they come from different library enums, and
// mixing those in one expression draws enum-mismatch warnings.
constexpr uint8_t accelFsBits(int g) {
  return g == 2   ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_2g)
         : g == 4 ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_4g)
         : g == 8 ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_8g)
                  : static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_16g);
}
constexpr uint8_t gyroFsBits(int dps) {
  return dps == 125    ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_125_ENABLED)
         : dps == 245  ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_G_245dps)
         : dps == 500  ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_G_500dps)
         : dps == 1000 ? static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_G_1000dps)
                       : static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_G_2000dps);
}
constexpr uint8_t kCtrl1Xl = static_cast<uint8_t>(LSM6DS3_ACC_GYRO_ODR_XL_104Hz) |
                             accelFsBits(config::kImuAccelRangeG) |
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_BW_XL_50Hz);
constexpr uint8_t kCtrl2G =
    static_cast<uint8_t>(LSM6DS3_ACC_GYRO_ODR_G_104Hz) | gyroFsBits(config::kImuGyroRangeDps);
// Block Data Update: freezes each 16-bit output register between its low-
// and high-byte reads, so a two-byte fetch can never straddle a sample
// rollover (torn read). IF_INC keeps multi-byte reads auto-incrementing.
constexpr uint8_t kCtrl3C = static_cast<uint8_t>(LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE) |
                            static_cast<uint8_t>(LSM6DS3_ACC_GYRO_IF_INC_ENABLED);

constexpr float kGravity = 9.80665f;      // m/s^2 per g
constexpr float kDegToRad = 0.017453293f; // rad/s per deg/s

LSM6DS3 sensor(I2C_MODE, kAddr);

const AxisMap kAxisMap = {
    {config::kLsm6ds3AxisSrc[0], config::kLsm6ds3AxisSrc[1],
     config::kLsm6ds3AxisSrc[2]},
    {config::kLsm6ds3AxisSign[0], config::kLsm6ds3AxisSign[1],
     config::kLsm6ds3AxisSign[2]}};

// Little-endian, matching the library's own readRegisterInt16().
int16_t rawPair(const uint8_t *p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

} // namespace

// One 12-byte burst, gyro X/Y/Z then accel X/Y/Z (OUTX_L_G..OUTZ_H_XL), rather
// than six separate two-transaction reads. BDU (enabled in begin()) freezes
// each 16-bit pair against a mid-read sample rollover, but only within that
// one pair; six separate transactions could still straddle a new sample
// landing between them, mixing an old X with a new Z. One burst is one
// coherent six-axis set.
bool Lsm6ds3::read(ImuSample &out) {
  uint8_t raw[12];
  if (sensor.readRegisterRegion(raw, LSM6DS3_ACC_GYRO_OUTX_L_G, sizeof(raw)) !=
      IMU_SUCCESS)
    return false;

  out.gyro[0] = sensor.calcGyro(rawPair(&raw[0])) * kDegToRad;
  out.gyro[1] = sensor.calcGyro(rawPair(&raw[2])) * kDegToRad;
  out.gyro[2] = sensor.calcGyro(rawPair(&raw[4])) * kDegToRad;
  out.accel[0] = sensor.calcAccel(rawPair(&raw[6])) * kGravity;
  out.accel[1] = sensor.calcAccel(rawPair(&raw[8])) * kGravity;
  out.accel[2] = sensor.calcAccel(rawPair(&raw[10])) * kGravity;

  remapAxes(out.accel, kAxisMap);
  remapAxes(out.gyro, kAxisMap);
  return !isZeroAccel(out);
}

// Powers the chip back off, as at boot. No restart path: leaving a stop goes
// through a reset, and begin() powers it up again from scratch.
void Lsm6ds3::end() { digitalWrite(board::kImuPowerPin, LOW); }

bool Lsm6ds3::begin(Console &log, ImuSample &first) {
  // Power the onboard chip and let it settle before talking to it over I2C.
  pinMode(board::kImuPowerPin, OUTPUT);
  digitalWrite(board::kImuPowerPin, HIGH);
  delay(300);

  sensor.settings.accelEnabled = 1;
  sensor.settings.accelRange = config::kImuAccelRangeG;
  sensor.settings.accelSampleRate = kAccelOdrHz;
  sensor.settings.accelBandWidth = kAccelBandwidthHz;
  sensor.settings.gyroEnabled = 1;
  sensor.settings.gyroRange = config::kImuGyroRangeDps;
  sensor.settings.gyroSampleRate = kGyroOdrHz;
  sensor.settings.tempEnabled = 0; // unused by this firmware

  // begin() fails only on a WHO_AM_I mismatch; its register writes go
  // unchecked, so "configured" is verified separately below.
  const bool found = sensor.begin() == IMU_SUCCESS;
  imuStep(log, found, "LSM6DS3TR-C detected");
  if (!found)
    return false;

  // Must come after begin(): the library's begin() calls Wire1.begin()
  // internally, which resets the TWI frequency register, silently overwriting
  // anything set earlier. Everything below then runs at the real bus speed.
  Wire1.setClock(config::kImuI2cClockHz);

  // CTRL1_XL, CTRL2_G and CTRL3_C are consecutive (0x10-0x12), so one burst
  // reads back everything sample scaling depends on, plus BDU.
  uint8_t ctrl[3] = {};
  const bool regsAccessible =
      sensor.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, kCtrl3C) == IMU_SUCCESS &&
      sensor.readRegisterRegion(ctrl, LSM6DS3_ACC_GYRO_CTRL1_XL, sizeof(ctrl)) == IMU_SUCCESS;
  const bool regsMatch =
      regsAccessible && ctrl[0] == kCtrl1Xl && ctrl[1] == kCtrl2G && ctrl[2] == kCtrl3C;
  // The first sample also proves a full sample read works at this speed.
  const bool sampled = regsMatch && read(first);

  char why[64] = "";
  if (!regsAccessible) {
    snprintf(why, sizeof(why), " (register access failed)");
  } else if (!regsMatch) {
    snprintf(why, sizeof(why), " (read %02X %02X %02X, expected %02X %02X %02X)", ctrl[0], ctrl[1],
             ctrl[2], kCtrl1Xl, kCtrl2G, kCtrl3C);
  } else if (!sampled) {
    snprintf(why, sizeof(why), " (first sample read failed)");
  }
  imuStep(log, sampled, "configured, %dg %ddeg/s%s", config::kImuAccelRangeG,
          config::kImuGyroRangeDps, why);
  return sampled;
}

} // namespace gnimu

#endif
