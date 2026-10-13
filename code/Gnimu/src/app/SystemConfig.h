// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// System constants: the values the firmware's timing, safety and hardware
// handling depend on, as opposed to the settings chosen for a build. Those
// are in Config.h next to Gnimu.ino, included here first so code includes
// only this file. The checks at the end cover both.

#include <stddef.h>
#include <stdint.h>

#include "../../Config.h"

namespace gnimu {
namespace config {

// GNSS

constexpr uint8_t kGnssDynamicModel = 4; // u-blox dynamic model 4 = automotive

// The baud rate the receiver is switched to and run at. Higher than 115200
// isn't needed, and lower leads to poor performance. Must be one of the rates
// the detection sweep tries.
constexpr uint32_t kGnssBaud = 115200;

// Every rate a receiver could be saved at, in the order the sweep tries them.
// kGnssBaud is tried first and skipped here.
constexpr uint32_t kGnssBaudRates[] = {4800,   9600,   19200,  38400, 57600,
                                       115200, 230400, 460800, 921600};

// Recursive rather than a loop: the nRF core builds as C++11, where a constexpr
// function is a single return.
constexpr bool baudSweepIncludes(uint32_t baud, size_t i = 0) {
  return i < (sizeof(kGnssBaudRates) / sizeof(kGnssBaudRates[0])) &&
         (kGnssBaudRates[i] == baud || baudSweepIncludes(baud, i + 1));
}
static_assert(
    baudSweepIncludes(kGnssBaud),
    "kGnssBaud must be a rate the sweep can detect and switch between");

// The drain period is also the longest the loop ever sleeps. The UART RX ring
// must hold more than one period of data, with plenty of margin.
constexpr uint32_t kGnssDrainPeriodUs = 1000;
// Liveness: no epoch for this long once the GNSS job is running faults the
// device if none ever arrived, or restarts it if they stopped mid-run. Even a
// receiver that fell back to 1 Hz is caught well inside it.
constexpr uint32_t kGnssSilenceMs = 2000;
// Includes the per-epoch BLE send, which occasionally takes the pass just past
// 3 ms while streaming (measured max 3.2 ms); harmless against the RX ring.
constexpr uint32_t kGnssDrainBudgetUs = 4000;
// Hardware watchdog: fed every loop pass, and between the boot steps and the
// GNSS library's round trips. The longest of those, a detection attempt on a
// silent port (three 1.1 s tries), is checked against it in Gnss.cpp.
constexpr uint32_t kWatchdogMs = 5000;
constexpr uint32_t kConsolePeriodUs = 10000;
constexpr uint32_t kConsoleBudgetUs = 2000;
// Longest one console line waits for room in the output buffer before it's
// dropped. Native USB blocks while a host holds the port open without
// reading; this keeps every write well inside the time the GNSS RX ring
// takes to fill (~5 ms on the nRF's 64-byte ring).
constexpr uint32_t kConsoleWaitUs = 2000;
// BLE housekeeping: connection-edge logging. Sends
// happen from the GNSS job as each epoch lands, not from this job.
constexpr uint32_t kBlePeriodUs = 10000;
constexpr uint32_t kBleBudgetUs = 2000;

// Status LED. core/StatusLed.h decides the patterns; these set their timing.
constexpr uint32_t kLedPeriodUs =
    25000; // fine enough to render the 5 Hz fault blink cleanly
constexpr uint32_t kLedBudgetUs = 500;
// The standard blink - advertising, battery warning/critical, charging -
// 1 s on, 1 s off.
constexpr uint32_t kLedBlinkHalfMs = 1000;
// Fault: 5 Hz. Urgent, and twice BATTERY_WAIT's rate so the two can't be
// confused.
constexpr uint32_t kLedFaultHalfMs = 100;
// BATTERY_WAIT: amber, fast enough to read as "not running" next to the 1 s
// battery-warning amber. Only ever on USB power, so it costs nothing.
constexpr uint32_t kLedBatteryWaitHalfMs = 200;

// Battery settings, for builds that have one.
// The job period is also the spacing between readings within a run.
constexpr uint32_t kBatteryPeriodUs = 10000;
constexpr uint32_t kBatteryBudgetUs = 500;
constexpr uint8_t kBatterySamplesPerRun = 10; // 100 ms window at 10 ms spacing
constexpr uint32_t kBatteryRunIntervalMs = 1000; // one run a second
constexpr float kBatteryEmaAlpha = 0.1f;         // display only
// Low long enough to be a depleted cell, not acquisition-current sag: ~5 runs.
constexpr uint32_t kBatteryCutoffDebounceMs = 5000;
constexpr uint32_t kBatteryChargeFullDebounceMs = 5000;

// Power state machine for battery builds. The job also paces the switch-sense
// ADC read.
constexpr uint32_t kPowerPeriodUs = 50000;
constexpr uint32_t kPowerBudgetUs = 1000;
// A switch change, OFF or ON, must last this long to count (noise, EMI, a
// brushed switch).
constexpr uint32_t kSwitchDebounceMs = 500;

// Periodic status line cadence ('L' toggle or an N+Enter report count); fired
// from the console job's own tick, not a separate scheduler job.
constexpr uint64_t kStatusPeriodUs = 1000000;
// Largest status-line count accepted from the console (3600 = an hour of 1 s
// lines), and the length of an L run, so status reporting always ends by
// itself. The console's entry buffer and per-pass read limit follow from its
// digit count, so changing this resizes both.
constexpr uint32_t kStatusMaxCount = 3600;

// IMU sample rate, shared by both MPU-6050 and LSM6DS3.
// 100 Hz, undecimated - a future encoder step picks its own transmit cadence
// rather than this job deciding one. The LSM6DS3 outputs at 104 Hz (100 isn't
// one of its rates, and its clock isn't synced to ours), so about one sample
// in 26 is never read; accepted, as the EMA smoothing (Config.h) covers it.
constexpr uint32_t kImuSamplePeriodUs = 10000;
constexpr uint32_t kImuSampleBudgetUs = 1000;
constexpr int kImuAccelRangeG = 4;    // 2, 4, 8 or 16 g; 4g is ample for auto-x
constexpr int kImuGyroRangeDps = 500; // 500, 1000 or 2000 deg/s suit both IMUs
                                      // (MPU-6050 also 250, LSM6DS3 125/245)
constexpr uint32_t kImuI2cClockHz = 400000;
// ESP32 only: how long one I2C transaction may wait before giving up (the
// core's default is 50 ms). A 14-byte read at 400 kHz takes about 0.4 ms, so
// 2 ms is ample, and an IMU that fails costs the GNSS ring (about 44 ms of
// data on the ESP32) about 2 ms per read instead of 50.
constexpr uint16_t kImuI2cTimeoutMs = 2;
// Consecutive failed reads that mark the IMU down: ~100 ms at 100 Hz. With
// the I2C timeout above, the failures cost the loop about 20 ms in all.
constexpr uint8_t kImuDownAfterFailures = 10;

// MPU-6050 only: DLPF_CFG has no LSM6DS3 equivalent (it uses ODR + bandwidth
// instead, set directly in Lsm6ds3.cpp).
constexpr uint8_t kImuDlpfCfg = 4; // DLPF_CFG 0-6; 4 = 21Hz accel, 20Hz gyro

// Trim (runtime levelling + gyro de-biasing).
// Shared by both chips: both drivers already convert to SI
// (m/s^2, rad/s) before this point, so one set of thresholds applies to both.
constexpr float kImuTrimGravityNative =
    9.80665f; // m/s^2; matches ImuSample's units
// 10s: not a noise requirement (half a second of averaging is already 2
// orders of magnitude finer than anything that matters), and not what keeps a
// sloped staging lane out of the calibration - locking after the FIRST
// qualifying window does that, since the first stop of a session is the
// paddock. This is only a backstop for the device being switched on already
// leaving the paddock, so the first qualifying stop is a staging lane instead.
constexpr uint32_t kImuTrimQualifyMs = 10000;
constexpr uint32_t kImuTrimBlockMs = 1000;
// 5 blocks = 5s on top of the qualify wait. Margin against a sub-threshold
// disturbance (someone leaning on the car) biasing a measurement that's never
// revisited - not a precision requirement.
constexpr uint32_t kImuTrimLockBlocks = 5;
constexpr float kImuTrimSpeedMaxMps = 0.5f;
// |a| plausibility band, deliberately wide: a tight "|a| == 1g" test is
// circular, since the chip's own zero-g bias is exactly what trim exists to
// remove (an uncalibrated MPU-6050 reading 0.925g failed a 4% band on every
// sample and could never converge). This only catches something gross - a
// misconfigured gravityNative, a dead axis, a failed read.
constexpr float kImuTrimAccelSanityTol = 0.25f;
// Per-axis accel std-dev ceiling. Measured: stationary engine-idling gives
// 6-10 mg per axis; driving gives 70-118 mg. This sits ~4x above idle and ~2x
// below driving; it's the secondary motion check, GNSS speed is the primary.
constexpr float kImuTrimAccelVarMax = 0.392f; // m/s^2, == 0.04g
// Per-axis gyro std-dev ceiling. Variance, not magnitude - gating on |gyro|
// is circular, since a chip whose resting bias exceeds the threshold would
// hold the gate shut against the very measurement that would correct it.
// Measured: cold idle 0.62 deg/s, warm idle 0.37 deg/s, driving 3.6-4.2 deg/s.
// 1.0 clears cold idle by 1.6x and stays 4.2x under driving; a rougher engine
// than the one this was tuned against may need this raised.
constexpr float kImuTrimGyroVarMax = 0.017453f; // rad/s, == 1.0 deg/s
// Largest tilt trim will correct; past this the user has most likely made a
// mistake rather than a mounting choice, so the rotation is refused rather
// than clamped.
constexpr float kImuTrimMaxTiltDeg = 15.0f;
// Demand a valid 3D fix before trimming: closes the one real gap in the rest
// of the gate (constant-velocity cruise on smooth pavement reads ~1g with
// near-zero gyro variance, indistinguishable from parked). Set false for bench
// testing indoors, which never gets a fix.
constexpr bool kImuTrimRequireFix = true;
// How long the last epoch may go without a fresh one before its speed stops
// counting as valid - a receiver that dies mid-drive must not freeze the gate
// open on a stale "stationary" reading.
constexpr uint32_t kImuTrimPvtStaleMs = 1000;

// Checks: the settings in Config.h first, then the system constants.

static_assert(kGnssNavRateHz > 0 && kGnssNavRateHz <= 25,
              "nav rate must be 1-25 Hz");
static_assert(kGnssMinElevDeg <= 90, "min elevation must be 0-90 degrees");
static_assert(kConstellationCount >= 1 && kConstellationCount <= 4,
              "constellation count must be 1-4");
static_assert(kConstellationPriority[0] != kConstellationPriority[1] &&
                  kConstellationPriority[0] != kConstellationPriority[2] &&
                  kConstellationPriority[0] != kConstellationPriority[3] &&
                  kConstellationPriority[1] != kConstellationPriority[2] &&
                  kConstellationPriority[1] != kConstellationPriority[3] &&
                  kConstellationPriority[2] != kConstellationPriority[3],
              "constellation priority must list each system exactly once");
static_assert(sizeof(kBleNamePrefix) >= 2 && sizeof(kBleNamePrefix) <= 21,
              "BLE name prefix must be 1-20 characters so the advertised name "
              "fits in 31 bytes");
static_assert(sizeof(kBleManufacturer) >= 2,
              "BLE manufacturer string must not be empty");
static_assert(kStatusPeriodUs > 0, "status period must be positive");
static_assert(kGnssSilenceMs > 1000 &&
                  kGnssSilenceMs >= 10 * (1000 / kGnssNavRateHz),
              "silence window must span a 1 Hz period and many nav periods");
static_assert(kLedBlinkHalfMs > 0 && kLedFaultHalfMs > 0 &&
                  kLedBatteryWaitHalfMs > 0,
              "LED half-periods must be positive");
static_assert(kLedPeriodUs / 1000 <= kLedBatteryWaitHalfMs,
              "LED job must run at least once per BATTERY_WAIT half-period");
static_assert(kBatteryCutoffV > 3.0f && kBatteryCutoffV < kBatteryCriticalV &&
                  kBatteryCriticalV < kBatteryWarningV &&
                  kBatteryWarningV < kBatteryFullScaleV &&
                  kBatteryFullScaleV <= 4.2f,
              "battery thresholds must be ordered cutoff < critical < warning "
              "< full <= 4.2 V");
static_assert(kBatterySamplesPerRun >= 1 &&
                  kBatterySamplesPerRun * (kBatteryPeriodUs / 1000.0f) <
                      kBatteryRunIntervalMs,
              "a battery sampling run must finish within its run interval");
static_assert(kBatteryCutoffDebounceMs >= kBatteryRunIntervalMs,
              "cutoff debounce shorter than one sampling run");
static_assert(kBatteryEmaAlpha > 0.0f && kBatteryEmaAlpha <= 1.0f,
              "alpha must be in (0, 1]");
static_assert(kSwitchDebounceMs >= 2 * (kPowerPeriodUs / 1000),
              "switch debounce must span more than one switch reading");
static_assert(kLedPeriodUs / 1000 <= kLedFaultHalfMs,
              "LED job must run at least once per fault half-period or the "
              "blink aliases");

constexpr bool isMpu6050AccelRange(int g) {
  return g == 2 || g == 4 || g == 8 || g == 16;
}
constexpr bool isMpu6050GyroRange(int dps) {
  return dps == 250 || dps == 500 || dps == 1000 || dps == 2000;
}
static_assert(isMpu6050AccelRange(kImuAccelRangeG),
              "not an MPU-6050 accelerometer range");
static_assert(isMpu6050GyroRange(kImuGyroRangeDps),
              "not an MPU-6050 gyro range");
static_assert(kImuDlpfCfg <= 6, "DLPF_CFG is a 0-6 register field");
static_assert(kImuAccelAlpha > 0.0f && kImuAccelAlpha <= 1.0f,
              "alpha must be in (0, 1]");
static_assert(kImuGyroAlpha > 0.0f && kImuGyroAlpha <= 1.0f,
              "alpha must be in (0, 1]");
constexpr bool isAxisPermutation(const uint8_t src[3]) {
  return src[0] < 3 && src[1] < 3 && src[2] < 3 && src[0] != src[1] &&
         src[0] != src[2] && src[1] != src[2];
}
static_assert(isAxisPermutation(kMpu6050AxisSrc),
              "Mpu6050 axis remap must be a permutation of 0,1,2");
static_assert(isAxisPermutation(kLsm6ds3AxisSrc),
              "Lsm6ds3 axis remap must be a permutation of 0,1,2");

static_assert(kImuTrimGravityNative > 0.0f, "gravity must be positive");
static_assert(kImuTrimQualifyMs >= kImuSamplePeriodUs / 1000,
              "qualify window shorter than one sample");
static_assert(kImuTrimBlockMs >= kImuSamplePeriodUs / 1000,
              "block shorter than one sample");
static_assert(kImuTrimLockBlocks >= 1, "must lock on at least one block");
static_assert(kImuTrimSpeedMaxMps > 0.0f, "speed gate must be positive");
static_assert(kImuTrimAccelSanityTol > 0.0f && kImuTrimAccelSanityTol < 1.0f,
              "sanity tolerance must be a fraction of gravity in (0, 1)");
static_assert(kImuTrimAccelVarMax > 0.0f,
              "accel variance ceiling must be positive");
static_assert(kImuTrimGyroVarMax > 0.0f,
              "gyro variance ceiling must be positive");
static_assert(kImuTrimMaxTiltDeg > 0.0f && kImuTrimMaxTiltDeg < 60.0f,
              "max tilt must be a small, physically sane angle");
static_assert(kImuTrimPvtStaleMs > 0, "stale window must be positive");

} // namespace config
} // namespace gnimu
