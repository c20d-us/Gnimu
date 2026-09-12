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
#include "config.h"
#include "g_protocol.h"

// ============================================================================
// Resolves TELEMETRY_PROTOCOL (config.h) to the descriptor the firmware runs.
//
// THIS IS THE ONE PLACE A NEW PROTOCOL IS WIRED IN. Adding one is: write
// g_proto_<name>.*, give it an ID in g_protocol.h, add a branch below. Nothing
// in g_telemetry or g_ble changes - they consume ACTIVE_PROTOCOL and never
// name a concrete protocol.
//
// Selection is COMPILE-TIME, so the unselected protocols' encoders are not
// linked and cost no flash. That is the reason it is a #if rather than a
// runtime table; see docs/multiprotocol-design.md section 8.1 for why runtime
// switching was rejected.
//
// Kept out of g_protocol.h on purpose: this header includes config.h, and
// g_protocol.h must not, or the encoders stop being host-compilable. The
// harness includes g_protocol.h and g_proto_<name>.h but never this file.
// ============================================================================

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

// Confirms the selected protocol defined PROTOCOL_MAX_FRAME_LEN, and that the
// value is sane.
//
// NOT #ifndef: that constant is a constexpr, and the preprocessor cannot see
// C++ declarations - an #ifndef on it is always true and would reject every
// build. (It did. TELEMETRY_PROTOCOL above is a real macro, so its guard is
// fine.) Referencing the constant here instead puts the "not declared" error in
// this file, which is where the requirement is documented, rather than leaving
// it to surface in whichever transport happens to use it first.
static_assert(PROTOCOL_MAX_FRAME_LEN > 0,
              "The selected protocol header must define a positive "
              "PROTOCOL_MAX_FRAME_LEN - see g_protocol.h.");
