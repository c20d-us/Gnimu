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
#include <Arduino.h>

// ============================================================================
// BLE module - RaceBox-compatible Bluetooth Low Energy server
//
// Owns the BLE server, characteristics, and connection state internally.
// Callers interact only through the small interface below; the live server
// objects are never exposed.
// ============================================================================

// Initialize the BLE device: configure the status LED pin, set TX power, create
// the RaceBox service and its Tx/Rx characteristics, publish the Device
// Information Service, and start advertising. Call once in setup().
void bleBegin();

// True while a client is connected.
bool bleIsConnected();

// Send one encoded frame to the connected client. Signature matches
// TelemetryEmit (g_protocol.h), so it can be handed straight to an encoder as
// its frame sink with no adapter.
//
// `channel` indexes the active protocol's channel table.
//
// Returns whether the transport accepted the frame - see TelemetryEmit in
// g_protocol.h for what that does and does not guarantee, which differs by
// stack. Never a delivery receipt.
//
// Caller is responsible for checking bleIsConnected() first if it cares.
bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len);

// Frames this transport refused or truncated, cumulative since boot.
//
// Counts FRAMES, not epochs - a protocol emitting several frames per sample
// can lose one and keep the rest. g_telemetry uses the delta across an encode
// call to decide whether that epoch went out whole, and reports the
// per-window delta on the stats output.
uint32_t bleDroppedFrames();

// Inbound writes this transport could not deliver, cumulative since boot.
//
// A write is dropped when the queue is full (the loop has not drained a burst
// yet) or, on a discrete transport, when it exceeds TELEMETRY_MAX_WRITE_LEN.
// A dropped write means a command the protocol never saw, so this is reported
// alongside the outbound drop count rather than left to accumulate quietly.
uint32_t bleDroppedWrites();

// Service the connection lifecycle.
// Re-advertise after a disconnect, track connect/disconnect edges, and drive
// the status LED (solid while connected, blinking while disconnected).
// Call every loop().
void bleUpdate();
