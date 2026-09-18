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
  // A fixed pause, not a wait on Serial: through the USB-UART bridge Serial is
  // true as soon as begin() returns, and anything sent before the host reopens
  // the port after a reset is lost - including the GNSS bring-up lines. Skipped
  // entirely in silent builds.
  Serial.begin(115200);
  delay(750);
#endif
  LOG_PRINTF("🚀 Gnimu [%s] starting up...\n", GNIMU_VARIANT);

  gnssBegin();
  imuBegin();
  bleBegin();
  telemetryBegin();
}

void loop() {
  gnssPoll();
  imuPoll();
  telemetrySendIfReady();
  bleUpdate();

  // Idle the core for one tick. Code runs from external flash through an
  // instruction cache, so a free-running loop whose working set outgrows the
  // cache fetches continuously, and the constant SPI bursts desense the GNSS
  // front end: the receiver loses satellites and then its fix while a client
  // streams. delay() parks the core until the next tick, which stops the
  // fetching whatever the loop's size. It is not a tuning knob.
  //
  // A tick is far inside every deadline: the default 256-byte GNSS RX ring
  // holds ~22ms at 115200, a PVT is sent in the same pass that completes it,
  // and the IMU cadence resyncs past a tick of jitter.
  delay(1);
}
