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
// The seam between the GNSS DRIVER (g_gnss.cpp - identical in every tree) and
// the MCU's UART (g_gnss_port_<mcu>.cpp - one per core).
//
// The driver owns everything about the RECEIVER: the baud sweep, the whole
// configuration sequence, the PVT callback and the epoch cache. The port owns
// everything about the MCU's serial peripheral: which object it is, which pins
// it uses, how its receive ring is sized, and the real-time constraint that
// ring imposes - which differs in KIND between the two cores, and is written
// out in each port file rather than in the shared driver.
//
//   g_gnss_port_esp32.cpp  - HardwareSerial(2), pins from config.h, 512-byte
//                            ring (a duration budget)
//   g_gnss_port_nrf52.cpp  - Serial1 on the XIAO's fixed D6/D7, 63 usable
//                            bytes (a phase-sensitive deadline)
//
// Ports are named after the CORE, not the board: what varies is the serial
// peripheral's API, and every board on a core shares it.
// ============================================================================

// Open the receiver's UART at `baud` and return the stream to talk to it on,
// or nullptr if the port could not be opened. Called repeatedly by the driver's
// baud sweep, so it must be safe to call after gnssPortEnd() - and any one-time
// setup a core needs BEFORE its first begin() (the ESP32's ring sizing) belongs
// here, done once, not in the driver.
//
// The returned pointer stays valid until the next gnssPortEnd(); the port owns
// the object.
Stream *gnssPortBegin(uint32_t baud);

// Release the UART. After this the pins revert to plain GPIO, which is what
// lets a board idle them low (the nRF's rail cutoff depends on it - see that
// port file). Safe to call when nothing is open.
void gnssPortEnd();
