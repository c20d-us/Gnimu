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

// Settings live in config.h
// Hardware & protocol logic lives in the ble, gnss, imu, and telemetry modules
#include "config.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_log.h"
#include "g_telemetry.h"

void setup() {
#if LOG_ENABLED
  // Hold off briefly so the startup banner and init logs aren't dropped while
  // the terminal is still attaching. Skipped entirely in silent builds. See
  // the delay() below for why this is a flat wait and not a poll on Serial.
  //
  // Give Serial a real TX ring FIRST. HardwareSerial constructs with
  // _txBufferSize(0), and with no ring uart_write_bytes() blocks the loop task
  // until the 128-byte hardware FIFO drains - about 20ms for the once-per-second
  // stats line at 115200. That is 20ms of not calling gnssPoll(), against a
  // 256-byte driver RX ring that a 20Hz NAV-PVT stream fills in roughly 22ms.
  // It fits today; the margin is the only thing between an attached console and
  // a corrupted epoch. 512 makes the write queue instead, and costs RAM only in
  // LOG_ENABLED builds.
  //
  // MUST precede begin(): setTxBufferSize() refuses once the driver is running
  // ("TX Buffer can't be resized when Serial is already running") and returns 0
  // rather than failing loudly, so the wrong order silently keeps the stall.
  //
  // ESP32-only by necessity, not oversight. On nRF, Serial is USB CDC: write()
  // returns immediately when no host is attached, and its 256-byte FIFO
  // (CFG_TUD_CDC_TX_BUFSIZE, an unguarded #define - not overridable) swallows a
  // whole line in one call when one is. Nothing there to set.
  //
  // The return is captured rather than discarded because the failure above is
  // SILENT: setTxBufferSize() only log_e()s and returns 0, so a reordering
  // would restore the stall with nothing on the console to say so. Checking it
  // turns the ordering rule from a comment someone can edit past into a fact
  // the running device reports.
  const size_t txRing = Serial.setTxBufferSize(512);
  Serial.begin(115200);
  // A flat delay, NOT `while (!Serial ...)`. On a UART-bridge board that guard
  // is a no-op: HardwareSerial::operator bool() is uartIsDriverInstalled(),
  // true the instant begin() returns, so the loop exits on its first check and
  // waits zero. Meanwhile the bridge discards anything written before the host
  // opens the port, which silently ate the banner and both GNSS
  // baud-detection lines - the console picked up mid-stream around the first
  // config command. There is no way to observe the host from here, so pay a
  // fixed cost instead of testing a condition that cannot be false.
  //
  // 700ms against a measured loss window of ~200-400ms (module already at
  // GNSS_BAUD, so connectAndConfigureBaud() succeeds on its first iteration).
  // It does NOT cover the Arduino IDE's post-upload monitor reconnect; if the
  // banner survives a plain reset but not an upload, that is the IDE.
  //
  // The nRF trees keep the `while (!Serial ...)` form deliberately: Serial is
  // USB CDC there and operator bool() tracks real host attachment, so the poll
  // both works and returns as soon as the monitor is up.
  //
  // This also cleared a block of garbage that used to precede the banner. That
  // is the same fault, not a separate one: bytes sent while the host is still
  // configuring the port arrive with framing errors rather than being dropped,
  // so the early output was partly lost and partly mangled. A delay fixes both
  // because both are the same window. (An earlier reading blamed the ESP32 ROM
  // bootloader's crystal-derived baud; that was disproved - ROM output precedes
  // this code and no delay here could have changed it.)
  delay(700);
  if (txRing == 0) {
    LOG_PRINTLN("⚠️ Serial TX ring not installed - setTxBufferSize() must be "
                "called BEFORE Serial.begin(). The 1Hz stats line will block "
                "the loop for ~20ms and can cost GNSS bytes.");
  }
#endif
  LOG_PRINTF("🚀 Gnimu [%s] starting up...\n", GNIMU_VARIANT);

  // See the nRF note: continue regardless so the loop keeps running.
  (void)gnssBegin();
  imuBegin();
  bleBegin();
  telemetryBegin();
}

void loop() {
  gnssPoll();
  imuPoll();
  telemetrySendIfReady();
  bleUpdate();
}
