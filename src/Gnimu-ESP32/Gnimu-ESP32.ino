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

// Settings are in config.h.
#include "config.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_led.h"
#include "g_log.h"
#include "g_telemetry.h"

void setup() {
#if LOG_ENABLED
  // Install a TX ring so console writes don't block the loop and starve
  // gnssPoll() while the 128-byte hardware FIFO drains. Must precede begin();
  // it fails silently otherwise, so the result is checked below.
  const size_t txRing = Serial.setTxBufferSize(512);
  Serial.begin(115200);
  // A fixed wait, not `while (!Serial)`: on a UART bridge Serial is true as
  // soon as begin() returns, and output before the host opens the port is lost
  // or garbled.
  delay(700);
  if (txRing == 0) {
    LOG_PRINTLN("⚠️ Serial TX ring not installed - setTxBufferSize() must be "
                "called BEFORE Serial.begin(). The 1Hz stats line will block "
                "the loop for ~20ms and can cost GNSS bytes.");
  }
#endif
  LOG_PRINTF("🚀 Gnimu [%s] starting up...\n", GNIMU_VARIANT);

  // Continue even if GNSS is absent.
  (void)gnssBegin();
  imuBegin();
  ledBegin();
  bleBegin();
  telemetryBegin();
}

void loop() {
  gnssPoll();
  imuPoll();
  telemetrySendIfReady();
  bleUpdate();
  ledUpdate();
}
