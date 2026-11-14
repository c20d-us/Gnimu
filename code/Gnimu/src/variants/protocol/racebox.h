// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../../app/ConfigTypes.h"

namespace gnimu {
namespace config {

// The RaceBox data protocol.
constexpr Protocol kProtocol = Protocol::RaceBox;
// The default advertised-name prefix: Gnimu, marked as speaking RaceBox.
constexpr char kProtocolNamePrefix[] = "Gnimu-RB";

}  // namespace config
}  // namespace gnimu
