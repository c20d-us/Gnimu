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

// Send a packet to the connected client via a notify on the Tx characteristic.
// Caller is responsible for checking bleIsConnected() first if it cares.
void bleSendPacket(uint8_t *data, size_t len);

// Service the connection lifecycle.
// Re-advertise after a disconnect, track connect/disconnect edges, and drive
// the status LED (solid while connected, blinking while disconnected).
// Call every loop().
void bleUpdate();
