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

// The BLE harness's stand-in for src/*/g_protocol_active.h. The runner copies
// the driver into a scratch directory beside THIS file, so g_ble.cpp's
// #include "g_protocol_active.h" finds it instead of the real one.
//
// A test protocol rather than RaceBox, because the driver's policy has cases
// RaceBox cannot reach: two NOTIFY channels (the multi-channel "sent" rule), a
// write-only channel sitting between them, and a frame limit small enough to
// exercise the MTU refusal. Shape: 0 notify, 1 write, 2 notify+read.

constexpr size_t PROTOCOL_MAX_FRAME_LEN = 20;
constexpr uint8_t PROTOCOL_CHANNEL_COUNT = 3;
constexpr TransportKind PROTOCOL_TRANSPORT = TRANSPORT_GATT_CHANNELS;

void harnessEncode(const TelemetrySample &, TelemetryEmit);
void harnessOnWrite(uint8_t channel, const uint8_t *data, size_t len);

constexpr ProtocolChannel harnessChannels[] = {
    {0x0001, nullptr, PROP_NOTIFY},
    {0x0002, nullptr, PROP_WRITE},
    {0x0003, nullptr, PROP_NOTIFY | PROP_READ},
};

constexpr ProtocolDescriptor HARNESS_PROTOCOL = {
    "Test Model",           "Test Maker", "HW2", "FW9", 0x1FF8, nullptr,
    TRANSPORT_GATT_CHANNELS, harnessChannels,
    (uint8_t)(sizeof(harnessChannels) / sizeof(harnessChannels[0])),
    harnessEncode,          harnessOnWrite,
};

static const ProtocolDescriptor *const ACTIVE_PROTOCOL = &HARNESS_PROTOCOL;
