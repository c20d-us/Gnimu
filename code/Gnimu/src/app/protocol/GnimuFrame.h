// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../core/BatteryMonitor.h"
#include "../../core/BleProfile.h"
#include "../../core/GnssEpoch.h"
#include "../../core/ImuSample.h"
#include "../../core/vendor/gnimu_frame/frame_v1.h"
#include "../SystemConfig.h"

namespace gnimu {

// The Gnimu family's BLE layout, the same on every Gnimu device. The model and
// manufacturer are left empty for App to fill from Config.h, and the DIS
// revisions null for App to fill with this build's own.
constexpr char kGnimuFamilyServiceUuid[] = "9074beb4-2f1a-4b5f-9818-fe2d2277ef05";
constexpr BleProfile kGnimuFrameProfile = {
    "",
    "",
    nullptr,
    nullptr,
    kGnimuFamilyServiceUuid,
    3,
    {
        {kGnimuFamilyServiceUuid, "0752d3c3-ceaf-462f-a944-bcc06c57284b", kBleNotify | kBleRead,
         BleText::None, nullptr},
        {kGnimuFamilyServiceUuid, "1c9b3945-10eb-4fd0-a5a4-b9d80842380d", kBleRead, BleText::Schema,
         nullptr},
        {kGnimuFamilyServiceUuid, "6d0adfef-8218-488a-a5f9-1ba05f422c22", kBleRead, BleText::DeviceInfo,
         nullptr},
    },
};

// The Gnimu frame, v1 (Gnimu-protocol, docs/frame.md): one 140-byte frame per
// GNSS epoch, filling the GNSS, IMU and power blocks. This class converts
// Gnimu's units and decides the valid bits; the vendored reference encoder
// does the layout, scaling, rounding and saturation. Pure - no hardware - so
// it's host-testable; App hands the encoded frame to BLE.
class GnimuFrameProtocol {
 public:
  static constexpr bool kUsesImu = true;
  static constexpr size_t kPacketBytes = gnimu_frame::v1::kFrameBytes;
  using Packet = uint8_t[kPacketBytes];

  // Positions in kGnimuFrameProfile's list.
  enum : uint8_t { kFrame, kSchema, kDeviceInfo, kCharCount };
  static constexpr uint8_t kDataChar = kFrame;
  // No Control characteristic yet, so nothing for a client to write.
  static constexpr uint8_t kRxChar = kBleNoChar;

  // The fields each block fills, as the Schema reports them: every field of
  // the GNSS and IMU blocks, and of the power block on a board with a battery.
  static constexpr uint16_t kGnssFill = 0x3FFF;
  static constexpr uint8_t kImuFill = 0x3F;
  static constexpr uint8_t kPowerFill = 0x07;

  static const BleProfile& profile() { return kGnimuFrameProfile; }
  static const char* name() { return "Gnimu"; }

  // The Schema characteristic's JSON. `r` is the nav rate, since a frame goes
  // out with each epoch. Returns the length; truncated, never overrun.
  static size_t formatSchema(char* buf, size_t cap, bool hasBattery) {
    char power[16] = "";
    if (hasBattery) snprintf(power, sizeof(power), ",\"power\":%u", static_cast<unsigned>(kPowerFill));
    const int n = snprintf(buf, cap, "{\"v\":1,\"d\":\"%s\",\"r\":%u,\"fill\":{\"gnss\":%u,\"imu\":%u%s}}",
                           name(), static_cast<unsigned>(config::kGnssNavRateHz),
                           static_cast<unsigned>(kGnssFill), static_cast<unsigned>(kImuFill), power);
    if (n < 0 || cap == 0) return 0;
    return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
  }

  // Bytes the client wrote: none can arrive without a writable characteristic.
  void onRx(const uint8_t*, size_t) {}

  // `battery` is nullptr on boards without one, which also leaves the power
  // block out of the capabilities. `nowUs` is the platform clock at sending.
  void encode(const GnssEpoch& e, const ImuSample& imu, const BatteryStatus* battery, bool usbPresent,
              uint64_t nowUs, Packet& out) {
    using namespace gnimu_frame::v1;
    const uint8_t caps = kCapGnss | kCapImu | (battery ? kCapPower : 0);
    Encoder f;
    // Device time wraps at 2^31 ms, as the spec says; the cast is that wrap.
    f.begin(static_cast<int32_t>(static_cast<uint32_t>(nowUs / 1000)), sequence_++, caps, kStateRunning);

    // A solution, fix or not: these describe it either way.
    f.set(Field::HorizontalAccuracy, e.hAccMm / 1000.0);
    f.set(Field::VerticalAccuracy, e.vAccMm / 1000.0);
    f.set(Field::Pdop, e.pDop / 100.0);
    f.set(Field::FixType, e.fixType);
    f.set(Field::Satellites, e.numSv);
    f.set(Field::GnssEpochAge, (static_cast<double>(nowUs) - static_cast<double>(e.arrivalUs)) / 1000.0);

    // gnssFixOK is a separate bit from the fix type, and both must agree.
    if (e.fixType >= 2 && e.fixType <= 4 && e.gnssFixOk) {
      // Divided, not multiplied by 1e-7: division gives the nearest double.
      f.set(Field::Latitude, e.latDeg1e7 / 1e7);
      f.set(Field::Longitude, e.lonDeg1e7 / 1e7);
      f.set(Field::AltitudeMsl, e.hMslMm / 1000.0);
      const double speedKmh = e.gSpeedMmPerS * kMmPerSToKmh;
      f.set(Field::GroundSpeed, speedKmh);
      f.set(Field::SpeedAccuracy, e.sAccMmPerS * kMmPerSToKmh);
      // The receiver's own heading-valid flag is only set in sensor-fusion
      // mode, so a speed gate stands in for it.
      if (speedKmh >= config::kGnssHeadingMinKmh) f.set(Field::Heading, e.headMotDeg1e5 / 1e5);
    }

    if (e.validDate && e.validTime && e.fullyResolved) {
      f.set(Field::UtcTimestamp, utcMs(e));
      f.set(Field::TimeAccuracy, e.tAccNs);
    }

    // A down IMU reads all zeros, which is never a real measurement.
    if (!isZeroAccel(imu)) {
      f.set(Field::AccelX, imu.accel[0] / kStandardGravity);
      f.set(Field::AccelY, imu.accel[1] / kStandardGravity);
      f.set(Field::AccelZ, imu.accel[2] / kStandardGravity);
      f.set(Field::RollRate, imu.gyro[0] * kRadToDeg);
      f.set(Field::PitchRate, imu.gyro[1] * kRadToDeg);
      f.set(Field::YawRate, imu.gyro[2] * kRadToDeg);
    }

    if (battery && battery->valid) {
      f.set(Field::SupplyVoltage, battery->voltage);
      f.set(Field::BatteryCharge, battery->percent);
      f.set(Field::Charging, isCharging(*battery, usbPresent) ? 1 : 0);
    }

    memcpy(out, f.finish(), kPacketBytes);
  }

  // The epoch's UTC time as ms since the Unix epoch, nano included (it can be
  // negative, borrowing from the second).
  static double utcMs(const GnssEpoch& e) {
    const int64_t days = daysFromCivil(e.year, e.month, e.day);
    const int64_t secs = days * 86400 + e.hour * 3600 + e.min * 60 + e.sec;
    return static_cast<double>(secs) * 1000.0 + e.nanoNs / 1e6;
  }

 private:
  // BLE only runs in the Running state, so every frame says so.
  static constexpr uint8_t kStateRunning = 2;
  static constexpr double kMmPerSToKmh = 0.0036;
  static constexpr double kStandardGravity = 9.80665;
  static constexpr double kRadToDeg = 57.29577951308232;

  // Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's
  // days_from_civil). The receiver's years are all well after 1970.
  static int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = y / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
  }

  uint8_t sequence_ = 0;  // wraps at 255, as the spec's sequence does
};

static_assert(bleProfileValid(kGnimuFrameProfile), "Gnimu BLE profile is malformed");
static_assert(kGnimuFrameProfile.charCount == GnimuFrameProtocol::kCharCount,
              "Gnimu's characteristic names don't match its list");
static_assert(bleNotifies(kGnimuFrameProfile, GnimuFrameProtocol::kDataChar),
              "frames go on a notify characteristic");
static_assert(bleWritableOrNone(kGnimuFrameProfile, GnimuFrameProtocol::kRxChar),
              "Gnimu reads from a writable characteristic, or none");

}  // namespace gnimu
