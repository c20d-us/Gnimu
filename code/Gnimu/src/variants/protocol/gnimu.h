// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../../app/ConfigTypes.h"

namespace gnimu {
namespace config {

// The Gnimu telemetry frame (Gnimu-protocol, frame v1).
constexpr Protocol kProtocol = Protocol::Gnimu;
// The default advertised-name prefix: the device type, as the spec names it.
constexpr char kProtocolNamePrefix[] = "Gnimu";

}  // namespace config
}  // namespace gnimu
