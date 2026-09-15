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
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// GNSS: the u-blox receiver, its configuration, and the epoch cache. The UART
// is behind g_gnss_port.h.
//
// The UART ring overflows silently. A lost byte fails an epoch's checksum and
// shows as a GNSS rate below GNSS_NAV_RATE_HZ on the stats line. Each port file
// documents its drain deadline; check it before adding work to loop() or a
// callback.

// Bring up and configure the receiver. Returns false if it never answered the
// baud sweep. Does not halt, so battery protection keeps running; there is no
// retry, and a power cycle recovers.
bool gnssBegin();

// True once configured, until gnssEnd(). With no receiver, GNSS fields are
// sentinels that look like a search for a fix, so callers check this first.
bool gnssIsUp();

// True if the receiver is up but no epoch has arrived for the larger of 1s and
// three epoch periods, counting from bring-up. Clears on the next epoch. Always
// false while gnssIsUp() is false.
//
// A brief power loss recovers on its own (configuration is held in BBR). An
// outage long enough to drain the backup supply needs a power cycle.
bool gnssStalled();

// Release the UART and clear gnssIsUp(). On nRF this must be called before
// BATTERY_WAIT or DEEP_SLEEP: an open UART holds TX high and back-powers the
// receiver through its RX pin.
void gnssEnd();

// Parse incoming bytes and fire the PVT callback. No-op while gnssIsUp() is
// false. The nav rate is fixed at GNSS_NAV_RATE_HZ. Call every loop().
void gnssPoll();

// Return the newest PVT if a new epoch arrived since the last call, else
// nullptr. Only g_telemetry may call this.
const UBX_NAV_PVT_data_t *gnssConsumePvt();

// Return the newest PVT without consuming it, or nullptr before the first
// epoch. For read-only observers. Says nothing about staleness.
const UBX_NAV_PVT_data_t *gnssLatestPvt();
