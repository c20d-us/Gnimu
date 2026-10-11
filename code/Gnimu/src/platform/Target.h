// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gnimu {

// "esp32" or "nrf52840". Implemented once per platform.
const char* targetName();

}  // namespace gnimu
