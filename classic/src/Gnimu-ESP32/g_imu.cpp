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

#include "g_imu.h"
#include "ImuAxis.h"
#include "config.h"
#include "g_gnss.h"
#include "g_imu_trim.h"
#include "g_log.h"
#include <Wire.h>

// MPU-6050 address and the registers this build touches.
static constexpr uint8_t kImuAddr = 0x68; // AD0 low
static constexpr uint8_t kImuWhoAmIValue = 0x68;
static constexpr uint8_t kRegSmplrtDiv = 0x19; // 0x19-0x1C are consecutive
static constexpr uint8_t kRegAccelOut = 0x3B;
static constexpr uint8_t kRegPwrMgmt1 = 0x6B;
static constexpr uint8_t kRegWhoAmI = 0x75;

// Three axes each for accelerometer and gyroscope, indexed [0]=X, [1]=Y,
// [2]=Z. Arrays let imuBegin()/imuPoll() drive all three axes of a sensor
// with one loop instead of one line per axis.
static ImuAxis accelAxes[3] = {
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
};
static ImuAxis gyroAxes[3] = {
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
};

// Latest values decimated to the transmission rate, refreshed by imuPoll().
// Kept as a plain cache so imuReadProtocolUnits() stays a cheap, side-effect
// free accessor safe to call from multiple places (BLE packet send and serial
// debug reporting both read it today).
static ImuProtocolUnits latestUnits = {0, 0, 0, 0, 0, 0};

// A single raw IMU sample, split into per-axis arrays indexed [0]=X, [1]=Y,
// [2]=Z, matching accelAxes/gyroAxes.
struct ImuRawSample {
  float accel[3]; // m/s^2
  float gyro[3];  // rad/s
};

// False until bring-up succeeds, and latched false for the rest of the session
// if the IMU stops answering. While false every IMU field reads zero and GNSS,
// BLE and telemetry carry on without it.
static bool imuUp = false;

// Consecutive failed reads that mark the IMU down: about 100ms at 100Hz.
static constexpr uint8_t kImuDownAfterFailures = 10;

// FS_SEL / AFS_SEL code for each full-scale range, 0xFF if the chip has no
// such range. The code lands in bits 4:3 of GYRO_CONFIG / ACCEL_CONFIG.
static constexpr uint8_t accelRangeCode(int g) {
  return g == 2 ? 0 : g == 4 ? 1 : g == 8 ? 2 : g == 16 ? 3 : 0xFF;
}
static constexpr uint8_t gyroRangeCode(int dps) {
  return dps == 250    ? 0
         : dps == 500  ? 1
         : dps == 1000 ? 2
         : dps == 2000 ? 3
                       : 0xFF;
}
static constexpr uint8_t kAccelRangeCode = accelRangeCode(IMU_ACCEL_RANGE_G);
static constexpr uint8_t kGyroRangeCode = gyroRangeCode(IMU_GYRO_RANGE_DPS);
static_assert(kAccelRangeCode != 0xFF && kGyroRangeCode != 0xFF,
              "ERROR: IMU_ACCEL_RANGE_G / IMU_GYRO_RANGE_DPS is not an "
              "MPU-6050 range.");

// SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG, written in one burst and read
// back to verify. Divider 0 samples at the DLPF's 1kHz output rate.
static constexpr uint8_t kImuConfig[4] = {
    0x00,
    (uint8_t)IMU_DLPF_CFG,
    (uint8_t)(kGyroRangeCode << 3),
    (uint8_t)(kAccelRangeCode << 3),
};

// Raw counts per g and per deg/s for the configured ranges, from the register
// map. Fixed at compile time; imuBegin() verifies the chip took the ranges.
static constexpr float kAccelCountsPerG = 16384.0f / (1 << kAccelRangeCode);
static constexpr float kGyroCountsPerDps =
    kGyroRangeCode == 0   ? 131.0f
    : kGyroRangeCode == 1 ? 65.5f
    : kGyroRangeCode == 2 ? 32.8f
                          : 16.4f;
static constexpr float kGravity = 9.80665f;       // m/s^2 per g
static constexpr float kDegToRad = 0.017453293f;  // rad/s per deg/s

// ============================================================================
// Axis remap - installed orientation. Maps the sensor frame into the vehicle
// frame. The mounting correction itself (IMU_AXIS_*_SRC / _SIGN) lives in
// config.h, along with the order table, the derivation procedure, and the
// static_asserts that reject a physically impossible map.
//
// A general permutation cannot be done in place - writing triple[0] would
// clobber a value a later axis still needs to read - so work from a copy. Three
// floats of stack, and it keeps the mapping a plain declarative read of the
// config rather than a sequence of conditional swaps.
// ============================================================================
static void remapAxes(float triple[3]) {
  const float in[3] = {triple[0], triple[1], triple[2]};
  triple[0] = in[IMU_AXIS_X_SRC] * IMU_AXIS_X_SIGN;
  triple[1] = in[IMU_AXIS_Y_SRC] * IMU_AXIS_Y_SIGN;
  triple[2] = in[IMU_AXIS_Z_SRC] * IMU_AXIS_Z_SIGN;
}

// Convert a scaled sensor value (gyro in centi-deg/sec, accel in milli-g) to
// the protocol's int16_t, saturating at the representable ±32767 limit rather
// than overflowing. A wrapped overflow would flip sign (i.e., reporting a hard
// left spin as a right one), so we clamp to the extreme. The gyro can exceed
// range at ±500 °/s; the accelerometer stays well within it even at the max ±g
// but goes through here too so all six fields follow one consistent, safe
// pattern.
static int16_t toProtocolInt16(float value) {
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  return (int16_t)value;
}

// Fetch GNSS ground speed for the trim gate, in m/s.
//
// gnssLatestPvt() rather than gnssConsumePvt(): the latter is consume-once and
// belongs to g_telemetry, so a second consumer here would race it and drop BLE
// packets. See g_gnss.h.
//
// Staleness is this caller's problem - gnssLatestPvt() hands back the last
// epoch however old, so a receiver that died mid-drive would otherwise freeze
// at a stale 0 m/s and let the stationary gate pass while moving. Watch iTOW
// and stop trusting the speed once it has stalled for IMU_TRIM_PVT_STALE_MS.
static bool trimSpeedMps(float *speedMps) {
  static uint32_t lastITOW = 0;
  static uint16_t staleSamples = 0;
  static const uint16_t kStaleMax =
      (uint16_t)(IMU_TRIM_PVT_STALE_MS / IMU_SAMPLE_INTERVAL_MS);

  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  if (pvt == nullptr)
    return false;

  if (pvt->iTOW != lastITOW) {
    lastITOW = pvt->iTOW;
    staleSamples = 0;
  } else if (staleSamples < kStaleMax) {
    staleSamples++;
  }
  if (staleSamples >= kStaleMax)
    return false;

  // 3D fix only - the same strict read of "valid" that g_telemetry reserves for
  // lat/lon. This was briefly relaxed to accept a 2D fix, on the grounds that
  // requiring 3D suspended trim learning under marginal sky. Locking the
  // orientation after one window removed that argument: there is no ongoing
  // accelerometer learning left to protect, and the single window that matters
  // happens in the paddock under clear sky, where a 3D fix is a given. Strict
  // costs nothing there and buys a stronger validity guarantee.
  //
  // One gate serves both halves, so this also pauses GYRO refinement under a
  // poor fix. Accepted deliberately: the gyro only needs to land occasionally
  // to track thermal drift, and a second gate is not worth the extra constant
  // or the longer explanation.
  if (pvt->fixType != 3 || !pvt->flags.bits.gnssFixOK)
    return false;

  *speedMps = (float)pvt->gSpeed * 0.001f; // mm/s -> m/s
  return true;
}

// Mark the IMU down for the rest of the session. Logs once.
static void markImuDown() {
  if (!imuUp)
    return;
  imuUp = false;
  latestUnits = {0, 0, 0, 0, 0, 0};
  LOG_PRINTLN("❌ IMU stopped answering - IMU fields will read zero until "
              "reboot.");
}

// Write consecutive registers from reg. Unchecked: imuBegin() reads the
// configuration back.
static void writeRegs(uint8_t reg, const uint8_t *data, size_t len) {
  Wire.beginTransmission(kImuAddr);
  Wire.write(reg);
  Wire.write(data, len);
  (void)Wire.endTransmission();
}

// Read consecutive registers from reg. False on a short read.
//
// requestFrom()'s count is the check: on this core endTransmission(false) only
// queues the write, and the whole transaction happens inside requestFrom().
static bool readRegs(uint8_t reg, uint8_t *out, size_t len) {
  Wire.beginTransmission(kImuAddr);
  Wire.write(reg);
  (void)Wire.endTransmission(false);
  if (Wire.requestFrom(kImuAddr, len, true) != len)
    return false;
  for (size_t i = 0; i < len; i++)
    out[i] = (uint8_t)Wire.read();
  return true;
}

// Reassemble a big-endian register pair.
static inline int16_t rawPair(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

// One burst of the 14 data registers from ACCEL_XOUT_H: accel X/Y/Z,
// temperature, gyro X/Y/Z. False on a short read, or if all three accel axes
// are exactly zero - the chip's power-on state after a supply glitch, which
// reads back cleanly but is not a measurement.
static bool readImuRegisters(ImuRawSample *out) {
  uint8_t raw[14];
  if (!readRegs(kRegAccelOut, raw, sizeof(raw)))
    return false;

  const int16_t ax = rawPair(&raw[0]);
  const int16_t ay = rawPair(&raw[2]);
  const int16_t az = rawPair(&raw[4]);
  if (ax == 0 && ay == 0 && az == 0)
    return false;

  out->accel[0] = (ax / kAccelCountsPerG) * kGravity;
  out->accel[1] = (ay / kAccelCountsPerG) * kGravity;
  out->accel[2] = (az / kAccelCountsPerG) * kGravity;
  // raw[6..7] is the die temperature, read only because it sits between the
  // accel and gyro registers.
  out->gyro[0] = (rawPair(&raw[8]) / kGyroCountsPerDps) * kDegToRad;
  out->gyro[1] = (rawPair(&raw[10]) / kGyroCountsPerDps) * kDegToRad;
  out->gyro[2] = (rawPair(&raw[12]) / kGyroCountsPerDps) * kDegToRad;
  return true;
}

// Read one sample, remapped into the vehicle frame and runtime-trimmed. A
// failed read returns the last good sample without touching the trim;
// kImuDownAfterFailures in a row marks the IMU down.
//
// There is no build-time zero correction: the runtime trim measures the total
// resting error, chip bias and mounting tilt together, so the firmware image
// is identical across boards.
static ImuRawSample readImuRaw() {
  static ImuRawSample lastGood = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
  static uint8_t failedInARow = 0;

  ImuRawSample s;
  if (!readImuRegisters(&s)) {
    if (++failedInARow >= kImuDownAfterFailures)
      markImuDown();
    return lastGood;
  }
  failedInARow = 0;

  remapAxes(s.accel);
  remapAxes(s.gyro);

  // Runtime trim, in the vehicle frame. Update BEFORE apply and on the
  // uncorrected values: feeding the estimator its own output would close the
  // loop and make imuTrimTiltDegrees() decay toward zero as it converged,
  // which is exactly wrong for a mounting guard that needs the absolute tilt.
  float speedMps = 0.0f;
  const bool speedValid = trimSpeedMps(&speedMps);
  imuTrimUpdate(s.accel, s.gyro, speedMps, speedValid);
  imuTrimApply(s.accel, s.gyro);

  lastGood = s;
  return s;
}

// Initialize the IMU, including setting up the sensor ranges and seed values.
// Never halts: a missing or misconfigured IMU leaves it down, and the rest of
// the device runs without it.
void imuBegin() {
  // Configured even if the IMU turns out to be absent, so the trim's accessors
  // report sane values either way.
  const ImuTrimConfig trimCfg = {
      IMU_GRAVITY_NATIVE,      (float)IMU_SAMPLE_INTERVAL_MS,
      IMU_TRIM_QUALIFY_MS,     IMU_TRIM_BLOCK_MS,
      IMU_TRIM_LOCK_BLOCKS,    IMU_TRIM_SPEED_MAX_MPS,
      IMU_TRIM_ACCEL_SANITY_TOL, IMU_TRIM_ACCEL_VAR_MAX,
      IMU_TRIM_GYRO_VAR_MAX,   IMU_TRIM_MAX_TILT_DEG,
      (bool)IMU_TRIM_REQUIRE_FIX,
  };
  imuTrimBegin(trimCfg);

  // 400kHz rather than the core's 100kHz default; see IMU_I2C_CLOCK_HZ.
  Wire.begin();
  Wire.setClock(IMU_I2C_CLOCK_HZ);

  uint8_t whoAmI = 0;
  if (!readRegs(kRegWhoAmI, &whoAmI, 1) || whoAmI != kImuWhoAmIValue) {
    LOG_PRINTLN("❌ IMU not found - continuing without it: IMU fields will "
                "read zero.");
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

  // readImuRegisters() scales from these ranges at compile time - a write that
  // did not land would mis-scale every sample.
  uint8_t pwr = 0;
  uint8_t cfg[sizeof(kImuConfig)] = {};
  if (!readRegs(kRegPwrMgmt1, &pwr, 1) || pwr != wake ||
      !readRegs(kRegSmplrtDiv, cfg, sizeof(cfg)) ||
      memcmp(cfg, kImuConfig, sizeof(cfg)) != 0) {
    LOG_PRINTLN("❌ IMU did not take its configuration - continuing without "
                "it: IMU fields will read zero.");
    return;
  }
  // Let the gyro and filter settle before the seed read below.
  delay(100);
  imuUp = true;
  LOG_PRINTLN("✅ IMU Accelerometer/Gyro enabled.");

  // Seed each axis with a real first reading rather than leaving it at its
  // zero-baseline default. Otherwise the first update() would see a huge
  // artificial jump (e.g. gravity on the Z axis) that gets latched into the
  // window's peak deviation and misreported as a genuine transient in the
  // very first transmitted frame.
  ImuRawSample seed = readImuRaw();
  for (int i = 0; i < 3; i++) {
    accelAxes[i].reset(seed.accel[i]);
    gyroAxes[i].reset(seed.gyro[i]);
  }
}

// Poll the IMU and update the axis filters at our configured sample rate.
// Both cadences below are deadline-anchored (the anchor advances by the
// interval, not to "now"), so per-loop latency doesn't stretch every period
// and quietly drop the real rates below their configured values. If the loop
// ever falls more than one full interval behind, the anchor resyncs to now
// rather than firing a rapid catch-up burst.
void imuPoll() {
  static unsigned long lastImuReadMs = 0;
  static unsigned long lastTransmitReadMs = 0;
  const unsigned long nowMs = millis();

  if (!imuUp)
    return; // latestUnits stays zero

  // Update all six axis filters if it's time to sample.
  if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuReadMs += IMU_SAMPLE_INTERVAL_MS;
    if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
      lastImuReadMs = nowMs; // fell > 1 interval behind - resync
    }
    ImuRawSample raw = readImuRaw();
    for (int i = 0; i < 3; i++) {
      accelAxes[i].update(raw.accel[i]);
      gyroAxes[i].update(raw.gyro[i]);
    }
  }

  // Decimate to the transmission rate on a fixed cadence, regardless of BLE
  // connection state. This is what drains each axis's transient window; if it
  // only ran while connected, the window would silently accumulate deviations
  // for as long as the device stayed disconnected and dump a stale "peak"
  // into the first packet after reconnecting.
  if (nowMs - lastTransmitReadMs >= IMU_TRANSMIT_INTERVAL_MS) {
    lastTransmitReadMs += IMU_TRANSMIT_INTERVAL_MS;
    if (nowMs - lastTransmitReadMs >= IMU_TRANSMIT_INTERVAL_MS) {
      lastTransmitReadMs = nowMs; // fell > 1 interval behind - resync
    }

    float accelFiltered[3], gyroFiltered[3];
    for (int i = 0; i < 3; i++) {
      accelFiltered[i] = accelAxes[i].read();
      gyroFiltered[i] = gyroAxes[i].read();
    }

    // Convert accelerometer to milli-g (1g = 9.80665 m/s^2) and gyro to
    // centi-deg/sec. All-float math: the constant factors fold at compile
    // time and the ESP32's FPU is single-precision only, so double math here
    // would fall back to (slow) software emulation.
    const float mps2ToMilliG = 1000.0f / 9.80665f;
    const float radpsToCentiDeg = (180.0f / (float)M_PI) * 100.0f;
    latestUnits.gX = toProtocolInt16(accelFiltered[0] * mps2ToMilliG);
    latestUnits.gY = toProtocolInt16(accelFiltered[1] * mps2ToMilliG);
    latestUnits.gZ = toProtocolInt16(accelFiltered[2] * mps2ToMilliG);
    latestUnits.rX = toProtocolInt16(gyroFiltered[0] * radpsToCentiDeg);
    latestUnits.rY = toProtocolInt16(gyroFiltered[1] * radpsToCentiDeg);
    latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * radpsToCentiDeg);
  }
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
