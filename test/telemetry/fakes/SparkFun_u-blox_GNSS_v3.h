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

// Stand-in for the SparkFun u-blox GNSS v3 library header, which needs the
// Arduino core. g_gnss.h includes it only for UBX_NAV_PVT_data_t, and the
// library's own struct header compiles standalone (see ROB-6) - so this pulls
// in exactly that. The harness tests against the REAL shipping struct, not a
// hand-written copy that could only prove it matches itself.
//
// Found ahead of the real header because test/telemetry/fakes is first on the
// include path (see test/run_telemetry_harness.sh).
#include <stddef.h>
#include <stdint.h>

#include "u-blox_structs.h"
