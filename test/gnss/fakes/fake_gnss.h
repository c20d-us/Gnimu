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

// Scenario controls for the GNSS harness, and the call log the goldens are
// made of. Everything the fake receiver does is decided here.
#include <stdint.h>

// The baud the fake receiver is listening at. The library's begin() answers
// only when the port is open at this rate. 0 = a receiver that never answers
// (unwired, unpowered, dead).
void fakeGnssReceiverBaud(unsigned long baud);

// After a baud switch, fail the verification begin() - the "something went
// deeply wrong" branch, which no hardware test has ever reached.
void fakeGnssFailVerify(bool fail);

// Make the named configuration call report failure. Matched on the call name
// as it appears in the log ("setAopCfg", "setDynamicModel", "enableGNSS",
// "setVal8", ...); every later call of that name fails too. One at a time.
void fakeGnssRejectCall(const char *name);

// Hand the driver one NAV-PVT epoch, exactly as the library's
// checkCallbacks() would - only reaches the driver if it registered a callback.
void fakeGnssDeliverEpoch(uint32_t iTOW, uint8_t fixType, int32_t gSpeedMmS);

// The call log: every library and port call in order, with its arguments.
void fakeGnssLogReset();
void fakeGnssLogPrint(); // to stdout, one call per line
