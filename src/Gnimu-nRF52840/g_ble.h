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
#include <stddef.h>
#include <stdint.h>

// BLE: the protocol-agnostic peripheral. Identity, services, and UUIDs come
// from the active ProtocolDescriptor (g_protocol.h); the stack is behind
// g_ble_port.h.

// Bring up the peripheral: identity, TX power, the protocol's service, Device
// Information Service, and advertising. Call once in setup().
void bleBegin();

// True while a central is connected.
bool bleIsConnected();

// True while a central is connected and subscribed to at least one of the
// protocol's notify channels. The nRF idle timeout keys on this.
bool bleIsSubscribed();

// Send one frame on a notify channel. Matches TelemetryEmit (g_protocol.h).
// Returns whether the transport accepted it, not whether it was delivered.
// Does not check bleIsConnected().
bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len);

// Frames accepted by the transport since boot.
uint32_t bleSentFrames();

// Frames not sent, for any reason, since boot.
uint32_t bleDroppedFrames();

// The subset of bleDroppedFrames() refused because the channel was not
// subscribed. This is normal, not a fault: real failures are
// ( bleDroppedFrames() - bleUnsubscribedFrames() ).
uint32_t bleUnsubscribedFrames();

// Inbound writes handed to the protocol since boot. Counted rather than logged
// per write: a central sets that rate.
uint32_t bleDispatchedWrites();

// Inbound writes dropped since boot: queue full, or on a discrete transport
// longer than TELEMETRY_MAX_WRITE_LEN.
uint32_t bleDroppedWrites();

// Dispatch one queued inbound write, track session and MTU changes, and run
// stack housekeeping. Call every loop().
void bleUpdate();

// Disconnect any central and stop advertising. No-op if bleBegin() failed.
void bleStop();
