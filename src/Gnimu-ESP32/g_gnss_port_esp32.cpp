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
// ============================================================================
// GNSS UART PORT - ESP32. HardwareSerial(2) on the GPIOs config.h names.
//
// The MCU half of the GNSS module; the receiver half is the shared driver in
// g_gnss.cpp, and the seam is g_gnss_port.h.
// ============================================================================

#include "g_gnss_port.h"
#include "config.h"
#include "g_log.h"

static HardwareSerial gnssSerial(2);

// Driver RX ring for the GNSS UART, in bytes. An implementation budget rather
// than a device tunable, so it lives here and not in config.h - nothing about
// a board or a wiring choice changes it.
static constexpr size_t kGnssRxRingBytes = 512;

// THE UART DRAIN CONSTRAINT - a DIFFERENT KIND of constraint from the nRF's,
// not the same one with more headroom.
//
// On the nRF trees the ring is 63 usable bytes against a 100-byte NAV-PVT: it
// cannot hold one message, so the MCU must be draining WHILE each message is on
// the wire. That is a PHASE-SENSITIVE deadline - a 30ms stall in the quiet gap
// between messages costs nothing, a 6ms stall overlapping a message costs an
// epoch.
//
// Here the ring holds 2.5 messages, so phase stops mattering and what remains
// is a DURATION BUDGET: how long may the loop stall before bytes are lost, in
// wall-clock time, whenever it happens. A 20Hz stream of 100-byte messages is
// 2000 bytes/sec, so the ring size IS the budget:
//
//     256 bytes (stock)   -> ~128ms of stall
//     512 bytes (ours)    -> ~256ms   see kGnssRxRingBytes in g_gnss.cpp
//
// Do NOT reach for "N bytes at 115200 is N*10/115200 seconds" here. That wire
// time answers a question nobody is asking, because the bytes do not arrive
// back-to-back at line rate - they arrive as a 8.68ms burst every 50ms. It is
// the right frame for the nRF, where the deadline really is set by how fast a
// single message fills the ring, and the wrong one for this variant.
//
// Both framings still make LAT-1's point: a ~20ms blocking console write is
// 20ms of not draining. Against 128ms that was survivable and against 256ms it
// is comfortable, but it was never headroom worth spending - see the
// setTxBufferSize() note in the .ino.
//
// No static_assert to match the nRF one: SERIAL_BUFFER_SIZE is an nRF core
// macro with no ESP32 equivalent. _rxBufferSize is a private runtime member,
// not a constant expression, so there is nothing to check at compile time. If
// more room is ever wanted it is available honestly - Serial2.setRxBufferSize()
// before begin(), as tools/common/gnss_otp_clock already does at 1024.
//
// The authoritative statement of the constraint, including the silent-overflow
// behaviour and the consequences for BLE handlers, is g_protocol.h's onWrite
// contract - a file in the checked common set, byte-identical in every tree.
// ---------------------------------------------------------------------------
// Sized ONCE, before the first begin(), and the return is captured because the
// failure is SILENT: setRxBufferSize() only log_e()s and returns 0 when the
// driver is already running. The driver's sweep calls this function repeatedly,
// so "once, and first" is enforced here with a flag rather than left to the
// order of calls somewhere else.
//
// end() clears the driver's UART but leaves _rxBufferSize alone, so one call
// covers the whole sweep and every re-begin after a baud switch.
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
