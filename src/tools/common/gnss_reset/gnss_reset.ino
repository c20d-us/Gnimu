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
// TOOL: full GNSS factory reset
//
// Connects to the u-blox M10 (sweeping common baud rates, since a receiver
// that's been reconfigured by a prior firmware run may not be at its factory
// baud) and issues myGNSS.factoryReset() - a full wipe of BBR + Flash +
// EEPROM followed by a cold-start reset, per the SparkFun v3 library. This is
// the "nuke it from orbit" recovery step: heavier than clearing config alone,
// it also discards almanac/ephemeris, so the next fix will pay a cold-start
// TTFF penalty.
//
// Use this when you want a guaranteed-clean receiver with no leftover state
// from prior runs - e.g. after debugging a config-persistence issue, or
// before handing off a board. It is NOT part of the normal boot flow:
// g_gnss.cpp's gnssBegin() already sets every runtime option with
// VAL_LAYER_RAM (RAM-only, never persisted), so a factory reset should not be
// needed in ordinary operation. See the gnss-first-command-ack-bug memory /
// DESIGN.md for the persistence bug this tool was built to recover from.
//
// After this sketch reports success, power-cycle the board and flash/run the
// main firmware - gnssBegin()'s baud sweep will find the receiver back at its
// factory baud (typically 38400) and reconfigure it from config.h as usual.
//
// Requires: on the nRF52840 variants, a XIAO nRF52840 Sense wired per
// DESIGN.md (GNSS TX -> D7, GNSS RX <- D6); on the ESP32 variant, the receiver
// on the UART2 pins set below. Plus USB for serial and the
// SparkFun_u-blox_GNSS_v3 library installed.
// ============================================================================

#include <Arduino.h>

// --- Platform wiring -------------------------------------------------------
// The only architecture-dependent part of this sketch: which UART object the
// receiver hangs off, and whether opening it needs pin arguments.
#if defined(ARDUINO_ARCH_ESP32)

// ESP32 variant: UART2 on the GPIOs from Gnimu-ESP32/config.h. Change these to
// match your wiring if you routed the receiver elsewhere.
static const int GNSS_RX_PIN = 16; // ESP32 RX <- GNSS TX
static const int GNSS_TX_PIN = 17; // ESP32 TX -> GNSS RX
static HardwareSerial gnssSerial(2);
#define GNSS_SERIAL_BEGIN(baud)                                                \
  gnssSerial.begin((baud), SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN)

#else

// nRF52840 (XIAO) variants: Serial (USB CDC) needs the TinyUSB library linked,
// and this sketch pulls in nothing else that would include it transitively.
#include <Adafruit_TinyUSB.h>

// Serial1 = D6 (TX) / D7 (RX), fixed on the nRF52 - same wiring as g_gnss.cpp.
static Uart &gnssSerial = Serial1;
#define GNSS_SERIAL_BEGIN(baud) gnssSerial.begin(baud)

#endif

#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS_SERIAL myGNSS;

// Common u-blox baud rates.
static const uint32_t BAUD_RATES[] = {9600,   19200,  38400, 57600,
                                      115200, 230400, 460800};
static const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// Sweep common baud rates until the receiver responds. Returns true and
// leaves gnssSerial/myGNSS connected at the working baud on success.
static bool connectAtAnyBaud() {
  for (int i = 0; i < NUM_BAUD_RATES; i++) {
    uint32_t baud = BAUD_RATES[i];
    Serial.printf("Trying GNSS at %lu baud...\n", (unsigned long)baud);

    GNSS_SERIAL_BEGIN(baud);
    delay(100); // let the serial port stabilize

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("Connected at %lu baud.\n", (unsigned long)baud);
      return true;
    }

    gnssSerial.end();
    delay(100);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("\n=== GNSS full factory reset ===");
  Serial.println("This wipes BBR + Flash + EEPROM config and cold-resets the");
  Serial.println("receiver. Almanac/ephemeris is discarded, so the next fix");
  Serial.println("will take longer (cold-start TTFF).\n");

  if (!connectAtAnyBaud()) {
    Serial.println("\nFAILED: u-blox GNSS not detected at any standard baud "
                   "rate. Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // Give the receiver a moment to fully wake up, then drain any backlog
  // before issuing the reset - same lesson as g_gnss.cpp's gnssBegin().
  delay(500);
  while (gnssSerial.available()) {
    gnssSerial.read();
  }

  Serial.println("\nIssuing factory reset (CFG-CFG wipe + cold-start "
                 "reset)...");
  myGNSS.factoryReset(); // void - does not wait for/expect an ACK

  Serial.println("Reset command sent. The receiver is now cold-resetting.");
  Serial.println("Power-cycle the board, then flash/run the main firmware -");
  Serial.println("its baud sweep will find the receiver at its factory baud");
  Serial.println("(typically 38400) and reconfigure it from config.h.");
  Serial.println("\nDone. Halting.");
}

void loop() {
  delay(1000); // Nothing to do - setup() already halted logically.
}
