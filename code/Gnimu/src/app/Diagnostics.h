// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/BatteryMonitor.h"
#include "../core/GnssFeed.h"
#include "../core/RateTracker.h"
#include "../core/ResetReason.h"
#include "../core/RunStateMachine.h"
#include "../core/Scheduler.h"
#include "../peripherals/ble/BleTransport.h"
#include "../peripherals/imu/ImuDevice.h"
#include "Console.h"

namespace gnimu {

// The console's reporting side: the D report, the 1 s status line, and the
// build and device ID lines. It only reads the modules it reports on; App
// routes each command, clears its own state on Clear, and passes every command
// here too, so each owner resets only what it owns.
class Diagnostics {
 public:
  // Everything is referenced, not copied, and must outlive this object.
  // `battery` is nullptr on boards without one.
  Diagnostics(Console& console, const Job* jobs, const JobStats* stats, size_t jobCount,
              const uint32_t& loops, const GnssFeed& feed, const ImuDevice& imu,
              const BleTransport& ble, const BatteryStatus* battery,
              const RunStateMachine& machine, const char* deviceId, const ResetRecord& reset)
      : console_(console),
        jobs_(jobs),
        stats_(stats),
        jobCount_(jobCount),
        loops_(loops),
        feed_(feed),
        imu_(imu),
        ble_(ble),
        battery_(battery),
        machine_(machine),
        deviceId_(deviceId),
        reset_(reset) {
    formatGnssSettings();
  }

  // One poll's result: Report, Version, ToggleStatus and a count act here;
  // Clear re-baselines the status line's rate trackers (App clears the
  // counters themselves first).
  void handle(const ConsoleCommand& cmd);
  // Prints the status line when one is due.
  void tick(uint64_t nowUs);

  void printBuild();
  void printDeviceId();

 private:
  // Arms (or disarms, with count == 0) periodic status reporting and resets
  // the rate trackers' baseline so the first line's Hz reflects only the
  // interval after arming, not however long events have been accumulating.
  void armStatus(uint32_t count);
  // The configured GNSS settings line: fixed for the build.
  void formatGnssSettings();
  // The device's state now, for the D report and the status line alike.
  DeviceStatus snapshot(uint64_t nowUs) const;
  void printReport();
  void printStatus(uint64_t nowUs);

  Console& console_;
  const Job* jobs_;
  const JobStats* stats_;
  size_t jobCount_;
  const uint32_t& loops_;
  const GnssFeed& feed_;
  const ImuDevice& imu_;
  const BleTransport& ble_;
  const BatteryStatus* battery_;
  const RunStateMachine& machine_;
  const char* deviceId_;
  const ResetRecord& reset_;
  char gnssSettings_[64] = {};  // e.g. "GPS+Galileo, 20 Hz, automotive"

  RateTracker gnssRate_;  // status line: epochs/s
  RateTracker bleRate_;   // status line: packets sent/s
  // Status lines still to print: 0 = off. Both L and a typed count arm it,
  // never beyond kStatusMaxCount.
  uint32_t statusRemaining_ = 0;
  uint64_t nextStatusDueUs_ = 0;
};

}  // namespace gnimu
