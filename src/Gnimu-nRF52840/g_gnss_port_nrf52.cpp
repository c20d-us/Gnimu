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

#include "config.h"
#include "g_gnss_port.h"

// GNSS UART port for nRF52840: Serial1 on the XIAO's D6 (TX) / D7 (RX).
//
// Drain deadline: Serial1's ring holds SERIAL_BUFFER_SIZE - 1 = 63 bytes, 5.5ms
// to fill at 115200 baud. A 100-byte NAV-PVT takes 8.7ms, so gnssPoll() must
// run repeatedly while each message arrives. Overflow drops bytes silently (see
// g_gnss.h).
//
// SERIAL_BUFFER_SIZE is set in the prebuilt core. Overriding it from the sketch
// would mismatch the Uart object layout.

static Uart &gnssSerial = Serial1;

static_assert(SERIAL_BUFFER_SIZE < 100,
              "Serial1's ring now holds a whole NAV-PVT, so the drain deadline "
              "documented above no longer applies. Update this comment (and "
              "g_protocol.h's onWrite contract, g_ble.cpp's ring sizing, and "
              "the OLED config.h slice rationale) before removing this.");
Stream *gnssPortBegin(uint32_t baud) {
  gnssSerial.begin(baud); // pins are fixed by the core
  return &gnssSerial;
}

// Frees D6 so powerHoldPeripheralsOff() can hold it low. Safe if never opened.
void gnssPortEnd() { gnssSerial.end(); }
