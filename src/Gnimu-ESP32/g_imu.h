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

#pragma once
#include "g_imu_sensor.h" // ImuProtocolUnits
#include <Arduino.h>

// ============================================================================
// IMU module - accelerometer + gyroscope, whatever part is fitted.
//
// The pipeline behind this interface is identical in every tree; only the
// sensor driver differs (see g_imu_sensor.h). Owns all IMU state internally.
// ============================================================================

// Start the runtime trim, bring up the sensor, and seed the filters with a
// first reading.
//
// DOES NOT HALT if the sensor does not answer. The IMU is marked down instead:
// every IMU field reads zero, trim never runs, and the rest of the device -
// GNSS, BLE, and on the battery builds the low-voltage cutoff - carries on. A
// halt here was the IMU twin of the GNSS defect ROB-1 fixed.
void imuBegin();

// True while the IMU is delivering samples. False if it never answered at
// bring-up, or has since stopped answering (a run of failed reads). While
// false, imuLatchForEpoch() and imuReadProtocolUnits() return zeros, and it
// stays false until the next boot.
bool imuIsUp();

// Failed sensor reads since boot, the count g_telemetry reports when it moves.
//
// Isolated failures are held (the last good sample is reused) and deliberately
// not logged one by one - at IMU_SAMPLE_INTERVAL_MS that would be its own
// latency problem. Only a RUN of them is loud, as the IMU going down. This
// counter covers the gap between: a flaky bus degrading the data while never
// failing enough times in a row to be declared dead.
uint32_t imuFailedReads();

// Poll the IMU and advance its filters. Self-throttles on
// IMU_SAMPLE_INTERVAL_MS, so it is safe to call every loop(). Does nothing
// while the IMU is down.
//
// Sampling is free-running and deliberately NOT tied to the GNSS epoch: the
// EMA and the transient tracker both want a uniform rate. Producing the value
// that actually gets transmitted is a separate step - see imuLatchForEpoch().
void imuPoll();

// Close the current transient window, latch a decimated sample for this GNSS
// epoch, and return it.
//
// PHASE-LOCKED TO THE EPOCH rather than to a timer, which is the whole point.
// Two timers sharing a period do not share a phase, so a free-running
// decimation left the sample riding each packet anywhere from 0 to one full
// interval old, with that age walking as the two clocks beat against each
// other. Driving it from epoch arrival makes the offset constant instead.
//
// A constant offset remains - the epoch describes an instant already past by
// the receiver's output latency, the UART transit of the message, and up to
// one poll interval. That is characterisable in a way a wandering one is not,
// which is the property being bought here. It is not zero, and no protocol
// field expresses it.
//
// RETURNS the latched values rather than leaving the caller to fetch them
// separately, so "latch before you encode" is a data dependency the compiler
// enforces rather than an ordering comment someone can reorder past.
//
// Each axis's transient window therefore spans exactly the interval between
// two transmitted samples - the peak reported in a packet is the peak over the
// interval that packet represents.
//
// Call exactly once per consumed GNSS epoch, from g_telemetry, and outside any
// BLE-connected test: draining the window must not depend on a client being
// attached. There is no timer fallback and none is needed - no epoch means no
// packet, so there is nothing to drain. If the receiver stops delivering
// entirely the cached value freezes and the window widens until epochs resume,
// a state in which the device is not producing telemetry anyway.
ImuProtocolUnits imuLatchForEpoch();

// Retrieve the most recently latched IMU values in RaceBox protocol units.
//
// The OBSERVER half of the pair: cheap, const, no side effects, safe to call
// any number of times per frame. It does not advance anything - the serial
// report uses it to show whatever was last transmitted. Producers want
// imuLatchForEpoch() instead.
ImuProtocolUnits imuReadProtocolUnits();
