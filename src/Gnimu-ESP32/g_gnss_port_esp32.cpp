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
#include "g_log.h"

// GNSS UART port for ESP32: HardwareSerial(2) on GNSS_RX_PIN / GNSS_TX_PIN.
//
// Drain budget: the 512-byte RX ring holds 2.5 NAV-PVTs, so timing relative to
// messages doesn't matter, only total stall time. At 2000 bytes/s (20Hz) the
// loop may stall about 256ms before bytes are lost (see g_gnss.h).

static HardwareSerial gnssSerial(2);

static constexpr size_t kGnssRxRingBytes = 512;

// setRxBufferSize() must run once, before the first begin(), and fails
// silently otherwise, so the result is checked. It survives end().
static bool ringSized = false;

Stream *gnssPortBegin(uint32_t baud) {
  if (!ringSized) {
    ringSized = true;
    if (gnssSerial.setRxBufferSize(kGnssRxRingBytes) == 0) {
      LOG_PRINTLN("⚠️ GNSS RX ring not resized - setRxBufferSize() must be "
                  "called BEFORE begin(). Falling back to the 256-byte "
                  "default.");
    }
  }
  gnssSerial.begin(baud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  return &gnssSerial;
}

void gnssPortEnd() { gnssSerial.end(); }
