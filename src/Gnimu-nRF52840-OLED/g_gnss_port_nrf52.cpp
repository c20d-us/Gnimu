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
// GNSS UART PORT - nRF52840. Serial1 on the XIAO's fixed D6 (TX) / D7 (RX).
//
// The MCU half of the GNSS module; the receiver half is the shared driver in
// g_gnss.cpp, and the seam is g_gnss_port.h.
// ============================================================================

#include "g_gnss_port.h"
#include "config.h"

static Uart &gnssSerial = Serial1;

// THE HARD REAL-TIME CONSTRAINT ON THIS MODULE - read before adding work to
// loop(), and before making any BLE or display callback do more.
//
// Serial1's receive path is a one-byte EasyDMA buffer feeding a software ring
// one ENDRX interrupt at a time (core: Uart.cpp IrqHandler, RingBuffer.cpp).
// That ring is SERIAL_BUFFER_SIZE bytes, of which only SERIAL_BUFFER_SIZE - 1
// are usable - RingBuffer detects "full" as head+1 == tail, so one slot is
// permanently reserved. 63 usable bytes at 115200 8N1 is 5.47ms of wire time.
//
// A NAV-PVT is 100 bytes (2 sync + class + id + 2 length + 92 payload + 2
// checksum) = 8.68ms on the wire. THE RING CANNOT HOLD ONE MESSAGE. gnssPoll()
// must therefore be reached repeatedly WHILE a message is arriving, not merely
// once between messages. Outside that window a 20Hz stream leaves ~41ms of
// clear air in which the loop may do as it likes.
//
// OVERFLOW IS SILENT. RingBuffer::store_char() drops the byte and does not
// advance the head - no flag, no counter, no error. Nothing in this firmware
// can observe it directly. It surfaces one step removed, as a checksum failure
// that costs a whole epoch, and one step further removed as a GNSS rate below
// GNSS_NAV_RATE_HZ on the 1Hz stats line. A sagging rate with no other
// explanation is the signature.
//
// This is why the OLED variant pushes display slices phase-locked to epoch
// arrival rather than on a free-running timer (see DISPLAY_SLICES_PER_EPOCH in
// its config.h), and why the onWrite contract in g_protocol.h forbids blocking
// work in a BLE handler. Both are consequences of this paragraph.
//
// The number is not ours to choose: SERIAL_BUFFER_SIZE lives in the core
// (RingBuffer.h). It is #ifndef-guarded, so -DSERIAL_BUFFER_SIZE=<n> would
// raise it - deliberately NOT done here. The macro sizes a CLASS MEMBER
// (RingBuffer::_aucBuffer, hence sizeof(Uart)), so a flag that reached the
// sketch's translation units but not the prebuilt core archive would leave the
// two disagreeing about object layout: memory corruption, not a build error.
// Buying margin on a constraint the code already meets is not worth that risk.
//
// The static_assert keeps the arithmetic above honest against a core upgrade.
// It permits a bigger ring; it just refuses to let one arrive unnoticed and
// leave four files' worth of comments quietly wrong.
// ---------------------------------------------------------------------------
static_assert(SERIAL_BUFFER_SIZE < 100,
              "Serial1's ring now holds a whole NAV-PVT, so the drain deadline "
              "documented above no longer applies. Update this comment (and "
              "g_protocol.h's onWrite contract, g_ble.cpp's ring sizing, and "
              "the OLED config.h slice rationale) before removing this.");
Stream *gnssPortBegin(uint32_t baud) {
  // Fixed pins: the XIAO wires Serial1 to D6/D7 and the core owns the mapping.
  gnssSerial.begin(baud);
  return &gnssSerial;
}

// Releasing the UART is what lets powerHoldPeripheralsOff() drive D6 LOW: while
// the peripheral owns the pin it keeps idling HIGH, phantom-powering the
// receiver through its RX ESD diode even with the TPS rail cut. Deliberately
// unguarded - releasing a port that was never opened is harmless, and the
// state machine calls it on paths where the receiver may never have answered.
void gnssPortEnd() { gnssSerial.end(); }
