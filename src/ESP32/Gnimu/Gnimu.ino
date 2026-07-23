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
  // Wait up to 3s for the serial port so the startup banner and init logs
  // aren't dropped while the terminal is still attaching. Harmless on
  // UART-bridge boards (Serial is always truthy there); gives a native-USB
  // monitor a brief chance to attach. Skipped entirely in silent builds.
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
#endif
  LOG_PRINTLN("🚀 Gnimu starting up...");

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
}
