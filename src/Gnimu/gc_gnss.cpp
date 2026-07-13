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

#include "gc_gnss.h"
#include "config.h"

// --- GNSS state ---
static SFE_UBLOX_GNSS myGNSS;
static HardwareSerial gnssSerial(2);

// --- PVT data and state ---
static UBX_NAV_PVT_data_t cachedPVT;
static bool newEpochAvailable = false;

// --- Struct to hold the constellation configuration from config.h---
struct Constellations {
  const char *name;
  sfe_ublox_gnss_ids_e id;
  bool enabled;
};

// The callback function triggered automatically by checkCallbacks()
// when a new UBX-NAV-PVT packet has been constructed.
static void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // Copy the new PVT data our local cache
  memcpy(&cachedPVT, ubxDataStruct, sizeof(UBX_NAV_PVT_data_t));
  newEpochAvailable = true;
}

// Try connecting to the GNSS at the target baud rate, and if that fails, sweep
// through all common u-blox baud rates to find the module and reconfigure it.
static bool connectAndConfigureBaud(uint32_t targetBaud) {
  // Array of baud rates to test. We test the target rate first for the fastest
  // normal boot, followed by common u-blox rates.
  const uint32_t baudRates[] = {targetBaud, 9600,   38400, 57600,
                                115200,     230400, 460800};
  const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);

  for (int i = 0; i < numRates; i++) {
    uint32_t testBaud = baudRates[i];
    Serial.printf("🔎 Trying GNSS at %d baud...\n", testBaud);

    gnssSerial.begin(testBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    delay(100); // Give the serial port a moment to stabilize

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("✅ GNSS detected at %d baud.\n", testBaud);

      // If we found it, but it's at the wrong speed, switch it.
      if (testBaud != targetBaud) {
        Serial.printf("🔀 Switching GNSS to target %d baud...\n", targetBaud);
        myGNSS.setSerialRate(targetBaud);
        delay(100);

        // Cycle the microcontroller's UART to match the new module speed
        gnssSerial.end();
        delay(100);
        gnssSerial.begin(targetBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
        delay(100);

        if (myGNSS.begin(gnssSerial)) {
          Serial.println("⚡ Baud rate switched. Saving to flash...");
          myGNSS.saveConfiguration(); // Lock it in for the next boot
          return true;
        } else {
          Serial.println("❌ Failed to verify new baud rate.");
          return false; // Something went deeply wrong
        }
      }
      return true; // We connected successfully at the target baud rate
    }

    // Clean up and prepare for the next loop iteration if this baud failed
    gnssSerial.end();
    delay(100);
  }
}

static void enableConstellations() {
  // Instantiate the array directly from the config macro
  const Constellations targetConstellations[] = GNSS_CONSTELLATIONS;

  Serial.println("🛰️ Enabling GNSS constellations...");

  for (const auto &target : targetConstellations) {
    if (myGNSS.enableGNSS(target.enabled, target.id)) {
      if (target.enabled) {
        Serial.printf("✅ %s enabled.\n", target.name);
      } else {
        Serial.printf("🚫 %s disabled.\n", target.name);
      }
    } else {
      if (target.enabled) {
        // We wanted it ON, but the chip rejected it. Real error.
        Serial.printf("❌ Failed to enable %s.\n", target.name);
      } else {
        // We wanted it OFF, and the chip rejected it. It's unsupported.
        Serial.printf("⚪ %s unsupported.\n", target.name);
      }
    }
  }
}

void gnssBegin() {
  // Make sure we can connect to the GNSS module at the target baud rate.
  // If we can't connect, halt with an error message.
  if (!connectAndConfigureBaud(GNSS_BAUD)) {
    Serial.println("❌ u-blox GNSS not detected at any standard baud rate.");
    Serial.println("X Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // --- Turn off NMEA messages - we want UBX only ---
  myGNSS.setUART1Output(COM_TYPE_UBX);

  // --- Set the GNSS dynamic model ---
  myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL);

  // --- Set the minimum elevation of satellites to track (anti-multipath) ---
  myGNSS.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, SV_MINELEV);

  // --- Turn on automatic PVT output ---
  myGNSS.setAutoPVT(true);

  // --- Register the callback to handle new PVT data ---
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback);

  // --- Set the GNSS update rate to MAX_NAVIGATION_RATE Hz ---
  if (myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE)) {
    Serial.printf("✅ GNSS update rate set to %d Hz.\n", MAX_NAVIGATION_RATE);
  } else {
    Serial.println("❌ Failed to set GNSS update rate.");
  }

  // --- Constellation Setup ---
  enableConstellations();
}

const UBX_NAV_PVT_data_t *gnssConsumePvt() {
  if (!newEpochAvailable) {
    return nullptr; // No new data since last time
  } else {
    newEpochAvailable = false; // "Consume" the flag
    return &cachedPVT;         // Return the fresh data
  }
}

bool gnssHeadingValid() {
  // Convenience wrapper around getHeadVehValid()
  return myGNSS.getHeadVehValid();
}

void gnssPoll() {
  // Pump the UART and parse incoming bytes into complete packets
  myGNSS.checkUblox();
  // Fire the registered callbacks for any completed packets
  myGNSS.checkCallbacks();
}
