// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// Based on the Open-Source RaceBox Mini Emulator by Anchit Chandra Sekhar
// (https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator)
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
#include <Arduino.h> // for uint8_t / uint16_t / uint32_t / etc.

// ============================================================================
// UBX Packet Construction Helpers
//
// Stateless helpers for building u-blox UBX-format binary messages (the same
// envelope the RaceBox Data Message rides on). These touch no globals - they
// only act on the buffers/values passed in - so they live cleanly on their own.
// ============================================================================

// Write a little-endian integer into buffer at the given offset.
// Overloaded for each width/signedness used by the RaceBox payload.
void writeLittleEndian(uint8_t *buffer, int offset, uint32_t value);
void writeLittleEndian(uint8_t *buffer, int offset, int32_t value);
void writeLittleEndian(uint8_t *buffer, int offset, uint16_t value);
void writeLittleEndian(uint8_t *buffer, int offset, int16_t value);
void writeLittleEndian(uint8_t *buffer, int offset, uint8_t value);
void writeLittleEndian(uint8_t *buffer, int offset, int8_t value);

// Compute the UBX 8-bit Fletcher checksum over class + id + length + payload.
// Results are written to *ckA and *ckB.
void calculateChecksum(uint8_t *payload, uint16_t len, uint8_t cls, uint8_t id,
                       uint8_t *ckA, uint8_t *ckB);
