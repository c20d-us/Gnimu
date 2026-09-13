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
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// BLE module - the Bluetooth Low Energy peripheral, identical on every board.
//
// Knows nothing about any particular protocol: identity, service topology and
// characteristic UUIDs all come from the active ProtocolDescriptor
// (g_protocol.h). The decisions live in g_ble.cpp; the stack's mechanism lives
// in a per-core port behind g_ble_port.h. Callers see only this interface.
// ============================================================================

// Bring up the peripheral: identity, TX power, the protocol's service, the
// Device Information Service, advertising. Call once in setup().
void bleBegin();

// True while a central is connected.
bool bleIsConnected();

// True while a central is connected AND subscribed to notifications on at
// least one of the protocol's notify channels - that is, actually receiving
// the stream. The nRF state machine keys its idle cutoff on this rather than
// bleIsConnected(): a client that connects and never subscribes gets nothing,
// and must not hold the device at full power until the battery cutoff.
bool bleIsSubscribed();

// Send one encoded frame to the connected central. Signature matches
// TelemetryEmit (g_protocol.h), so it is handed straight to an encoder as its
// frame sink.
//
// `channel` indexes the active protocol's channel table and must be a notify
// channel. Returns whether the transport accepted the frame - see
// TelemetryEmit for what that does and does not guarantee. Never a delivery
// receipt.
//
// Caller is responsible for checking bleIsConnected() first if it cares.
bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len);

// Frames the transport accepted, cumulative since boot.
uint32_t bleSentFrames();

// Frames the transport did not send, for any reason, cumulative since boot.
//
// Counts FRAMES, not epochs - a protocol emitting several frames per sample can
// lose one and keep the rest.
uint32_t bleDroppedFrames();

// The part of bleDroppedFrames() refused because the central had not enabled
// notifications on that channel - always <= it, cumulative since boot.
//
// That refusal is BLE working as specified (NEW-1), not a fault: nRF Connect
// connects without subscribing, and apps take a moment to subscribe. So a
// failure is bleDroppedFrames() - bleUnsubscribedFrames(), and that is what
// g_telemetry reports and what decides whether an epoch counts as sent: at
// least one frame accepted (bleSentFrames() moved) and none failed.
uint32_t bleUnsubscribedFrames();

// Inbound writes this transport could not deliver, cumulative since boot.
//
// A write is dropped when the queue is full (the loop has not drained a burst
// yet) or, on a discrete transport, when it exceeds TELEMETRY_MAX_WRITE_LEN.
// A dropped write means a command the protocol never saw, so this is reported
// alongside the outbound drop count rather than left to accumulate quietly.
uint32_t bleDroppedWrites();

// Service the connection: dispatch one queued inbound write, follow session
// changes and the MTU, and run the stack's own housekeeping. Call every loop().
void bleUpdate();

// Disconnect any central and stop advertising. A no-op if bleBegin() never
// brought the stack up. Every board has it so the interface does not fork; the
// nRF state machine is what calls it.
void bleStop();
