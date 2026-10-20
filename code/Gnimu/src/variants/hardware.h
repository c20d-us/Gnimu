// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pulls in the hardware profile for this build: the pins and sensor mounting
// that differ between builds of the same board. A board's BoardConfig.h
// defines GNIMU_HARDWARE_DEFAULT, a path relative to this folder, and includes
// this file. scripts/build.sh overrides the choice with --hardware, which
// defines GNIMU_HARDWARE_FILE to the matching file; an Arduino IDE build gets
// the default.
//
// A profile is a header in <board>/hardware/ that defines, in gnimu::board,
// every constant the other profiles for that board define. To make your own,
// copy the nearest one, change its values, and build with --hardware <name>.

#ifndef GNIMU_HARDWARE_FILE
#define GNIMU_HARDWARE_FILE GNIMU_HARDWARE_DEFAULT
#endif
#include GNIMU_HARDWARE_FILE
