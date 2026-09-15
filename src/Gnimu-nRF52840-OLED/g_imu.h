// Gnimu - GNSS+IMU streaming telemetry
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

#pragma once
#include "g_imu_sensor.h"
#include <Arduino.h>

// IMU: the accelerometer and gyro pipeline, shared by every board.
// The sensor driver is behind g_imu_sensor.h.

// Start trim, bring up the sensor, and seed the filters. If the sensor does not
// answer, the IMU is marked down (fields read zero) and nothing halts.
void imuBegin();

// True while the IMU delivers samples. Goes false if bring-up failed or a run
// of reads fails, and stays false until reboot.
bool imuIsUp();

// Failed sensor reads since boot. A single failure reuses the last good sample
// and is not logged; only a run of them takes the IMU down.
uint32_t imuFailedReads();

// Sample the sensor and advance the filters, throttled to
// IMU_SAMPLE_INTERVAL_MS. Free-running, not tied to GNSS epochs.
// Safe to call every loop(); no-op while the IMU is down.
void imuPoll();

// Close the transient window, latch a sample for this GNSS epoch, and return
// it. Locking to the epoch gives each packet a constant sample age, and each
// window spans exactly one packet interval.
//
// Call once per consumed epoch from g_telemetry, whether or not a client is
// connected. With transient thresholds live, the first epoch after a GNSS stall
// would carry a stale peak; see g_imu_tuning.h.
ImuProtocolUnits imuLatchForEpoch();

// Return the last latched values without side effects.
ImuProtocolUnits imuReadProtocolUnits();
