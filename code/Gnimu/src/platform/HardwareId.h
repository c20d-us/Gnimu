// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// A factory-programmed, per-chip identifier: not the BLE MAC, and not
// something firmware can set. Implemented once per platform.
uint64_t hardwareId();

}  // namespace gnimu
