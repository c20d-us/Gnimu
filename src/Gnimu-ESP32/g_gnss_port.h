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
#include <Arduino.h>

// GNSS port: the interface between the shared driver (g_gnss.cpp) and each
// core's UART. The driver owns the receiver; the port owns the serial
// peripheral, its pins and ring, and documents the drain deadline.
//
//   g_gnss_port_esp32.cpp  HardwareSerial(2), 512-byte ring (duration budget)
//   g_gnss_port_nrf52.cpp  Serial1 on D6/D7, 63-byte ring (phase-sensitive
//                          deadline)

// Open the UART at `baud` and return its stream, or nullptr on failure. Called
// repeatedly by the baud sweep, so it must be safe after gnssPortEnd(). The
// port owns the stream; it is valid until gnssPortEnd().
Stream *gnssPortBegin(uint32_t baud);

// Release the UART and return the pins to GPIO. Safe when nothing is open.
void gnssPortEnd();
