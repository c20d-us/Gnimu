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

#include "gc_gnss.h"
#include "config.h"

// --- GNSS state - private to this file ---
static SFE_UBLOX_GNSS myGNSS;
static HardwareSerial gnssSerial(2);

// --- Callback State ---
static UBX_NAV_PVT_data_t cachedPVT;
static volatile bool newEpochAvailable = false;

// The callback function triggered automatically by checkCallbacks()
// when a completely valid UBX-NAV-PVT packet is received.
static void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // Copy the cleanly parsed data into our local cache
  memcpy(&cachedPVT, ubxDataStruct, sizeof(UBX_NAV_PVT_data_t));
  newEpochAvailable = true;
}

// Try connecting at the target baud rate, and if that fails, sweep through
// all common u-blox baud rates to find the module and reconfigure it.
bool connectAndConfigureBaud(uint32_t targetBaud) {
  // Array of baud rates to test. We test the target rate first for the fastest
  // boot on normal runs, followed by common u-blox rates.
  const uint32_t baudRates[] = {targetBaud, 9600,   38400, 57600,
                                115200,     230400, 460800};
  const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);

  for (int i = 0; i < numRates; i++) {
    uint32_t testBaud = baudRates[i];
    Serial.printf("Trying GNSS at %d baud...\n", testBaud);

    gnssSerial.begin(testBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    delay(100); // Give the serial port a moment to stabilize

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("✅ GNSS detected at %d baud.\n", testBaud);

      // If we found it, but it's at the wrong speed, switch it.
      if (testBaud != targetBaud) {
        Serial.printf("Switching GNSS to target %d baud...\n", targetBaud);
        myGNSS.setSerialRate(targetBaud);
        delay(100);

        // Cycle the microcontroller's UART to match the new module speed
        gnssSerial.end();
        delay(100);
        gnssSerial.begin(targetBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
        delay(100);

        if (myGNSS.begin(gnssSerial)) {
          Serial.println(
              "✅ Baud rate switched successfully. Saving to flash...");
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

void gnssBegin() {
  if (!connectAndConfigureBaud(GNSS_BAUD)) {
    Serial.println("❌ u-blox GNSS not detected at any standard baud rate.");
    Serial.println("X Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // --- Set GNSS output to PVT only ---
  myGNSS.setAutoPVT(true);

  // --- Register the callback to handle the PVT data automatically ---
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback);

  // --- Set the GNSS dynamic model ---
  myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL);

  // --- Turn off NMEA messages - we want UBX only ---
  myGNSS.setUART1Output(COM_TYPE_UBX);

  // --- Set minimum elevation of satellites to track ---
  myGNSS.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, SV_MIN_ELEVAION);

  // --- Configure GNSS update rate to MAX_NAVIGATION_RATE Hz ---
  if (myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE)) {
    Serial.printf("✅ GNSS update rate set to %d Hz.\n", MAX_NAVIGATION_RATE);
  } else {
    Serial.println("❌ Failed to set GNSS update rate.");
  }

// --- GNSS Constellation Setup ---

// GPS
#ifdef ENABLE_GNSS_GPS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)) {
    Serial.println("✅ GPS enabled.");
  } else {
    Serial.println("❌ Failed to enable GPS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
  Serial.println("🚫 GPS disabled.");
#endif

// Galileo
#ifdef ENABLE_GNSS_GALILEO
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO)) {
    Serial.println("✅ Galileo enabled.");
  } else {
    Serial.println("❌ Failed to enable Galileo.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
  Serial.println("🚫 Galileo disabled.");
#endif

// GLONASS
#ifdef ENABLE_GNSS_GLONASS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS)) {
    Serial.println("✅ GLONASS enabled.");
  } else {
    Serial.println("❌ Failed to enable GLONASS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
  Serial.println("🚫 GLONASS disabled.");
#endif

// BeiDou
#ifdef ENABLE_GNSS_BEIDOU
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU)) {
    Serial.println("✅ BEIDOU enabled.");
  } else {
    Serial.println("❌ Failed to enable BEIDOU.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
  Serial.println("🚫 BEIDOU disabled.");
#endif

// QZSS
#ifdef ENABLE_GNSS_QZSS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS)) {
    Serial.println("✅ QZSS enabled.");
  } else {
    Serial.println("❌ Failed to enable QZSS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
  Serial.println("🚫 QZSS disabled.");
#endif

// SBAS (satellite-based augmentation)
#ifdef ENABLE_GNSS_SBAS
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS)) {
    Serial.println("✅ SBAS enabled.");
  } else {
    Serial.println("❌ Failed to enable SBAS.");
  }
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
  Serial.println("🚫 SBAS disabled.");
#endif
}

void gnssPoll() {
  // Pump the UART and parse incoming bytes into complete packets
  myGNSS.checkUblox();
  // Fire the registered callbacks for any completed packets
  myGNSS.checkCallbacks();
}

bool gnssHasNewEpoch() {
  if (newEpochAvailable) {
    newEpochAvailable = false; // Reset the flag for the next epoch
    return true;
  }
  return false;
}

const UBX_NAV_PVT_data_t *gnssLatestPvt() {
  // Return a pointer to our safely cached struct, not the live volatile one
  return &cachedPVT;
}

bool gnssHeadingValid() {
  // Since checkUblox() updates the library's internal state on successful
  // parse, we can still safely call this, or extract it from the cachedPVT if
  // desired.
  return myGNSS.getHeadVehValid();
}
