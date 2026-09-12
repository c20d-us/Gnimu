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
// BLE module - Bluetooth Low Energy peripheral
//
// Owns the BLE stack, its services, and the connection state internally.
// Callers interact only through the small interface below; the live Bluefruit
// objects are never exposed.
//
// Knows nothing about any particular protocol: identity, service topology and
// characteristic UUIDs all come from the active ProtocolDescriptor
// (g_protocol.h). What stays here is stack MECHANICS - MTU, TX power,
// connection lifecycle, advertising - which is platform-specific and protocol
// independent.
// ============================================================================

// Initialize the BLE peripheral: raise the MTU ceiling, set TX power, expose
// the Nordic UART (RaceBox) service and the Device Information service, then
// start advertising. Call once in setup().
void bleBegin();

// True while a client is connected.
bool bleIsConnected();

// True while a client is connected AND subscribed to notifications - that is,
// actually receiving the stream. g_state keys its idle cutoff on this rather
// than bleIsConnected(): a client that connects and never subscribes gets
// nothing, and must not hold the device at full power until the battery
// cutoff. Same check bleEmitFrame() makes before every send.
bool bleIsSubscribed();

// Send one encoded frame to the connected client. Signature matches
// TelemetryEmit (g_protocol.h), so it can be handed straight to an encoder as
// its frame sink with no adapter.
//
// `channel` indexes the active protocol's channel table. The Nordic UART
// transport has only one stream and ignores it; the GATT-channels transport
// routes to the matching characteristic.
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
void bleUpdate();

// Disconnect any active peripheral connection and stop advertising, so the
// device disappears from BLE scans and any connected client cleanly sees the
// link end. Does NOT tear down the Bluefruit stack.
void bleStop();
