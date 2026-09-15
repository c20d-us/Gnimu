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
#include "config.h"
#include "g_protocol.h"

// Active protocol: maps TELEMETRY_PROTOCOL (config.h) to its descriptor.
//
// To add a protocol: write g_proto_<name>.*, give it an ID in g_protocol.h, and
// add a branch below. Selection is compile-time, so unselected encoders are not
// linked. Kept out of g_protocol.h so encoders stay free of config.h and build
// on a host.

#ifndef TELEMETRY_PROTOCOL
#error "TELEMETRY_PROTOCOL is not defined. It belongs in config.h, Section 1."
#endif

#if TELEMETRY_PROTOCOL == PROTO_RACEBOX
#include "g_proto_racebox.h"
static const ProtocolDescriptor *const ACTIVE_PROTOCOL = &RACEBOX_PROTOCOL;

#else
#error "TELEMETRY_PROTOCOL names a protocol this build does not implement. \
Valid values are the PROTO_* ids in g_protocol.h that have a branch here."
#endif

// PROTOCOL_MAX_FRAME_LEN is constexpr, so it is checked here rather than with
// #ifndef.
static_assert(PROTOCOL_MAX_FRAME_LEN > 0,
              "The selected protocol header must define a positive "
              "PROTOCOL_MAX_FRAME_LEN - see g_protocol.h.");
