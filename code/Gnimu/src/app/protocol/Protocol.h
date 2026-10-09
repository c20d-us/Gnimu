// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../SystemConfig.h"
#include "RaceBox.h"

namespace gnimu {

// The protocol class for config::kProtocol, chosen at compile time with no
// virtual dispatch. A Protocol value without a specialisation here fails the
// build.
template <config::Protocol P>
struct ProtocolFor;
template <>
struct ProtocolFor<config::Protocol::RaceBox> {
  using type = RaceBoxProtocol;
};

using Protocol = ProtocolFor<config::kProtocol>::type;

}  // namespace gnimu
