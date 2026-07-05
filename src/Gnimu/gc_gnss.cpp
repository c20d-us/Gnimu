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

// Report a detection failure at the given baud rate and halt forever - the
// device cannot function without the GNSS module.
static void haltGnssNotDetected(uint32_t baud) {
  Serial.print("u-blox GNSS not detected at ");
  Serial.print(baud);
  Serial.println(" baud.");
  Serial.println("Check documentation for the factory baud rate and/or check "
                 "your wiring.");
  while (1)
    delay(100);
}

static void resetGnssBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  gnssSerial.begin(FACTORY_GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(gnssSerial)) {
    haltGnssNotDetected(FACTORY_GNSS_BAUD);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(FACTORY_GNSS_BAUD);
    Serial.println(" baud!");
  }
  delay(500);

  // Now switch baud rate
  Serial.print("Setting baud rate to ");
  Serial.print(GNSS_BAUD);
  Serial.println("...");
  myGNSS.setSerialRate(GNSS_BAUD);
  Serial.print("Baud rate changed to ");
  Serial.println(GNSS_BAUD);

  gnssSerial.end();
  delay(100);
  // Re-initialize the serial port at the new baud rate
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(gnssSerial)) {
    haltGnssNotDetected(GNSS_BAUD);
  }
  Serial.print("GNSS detected at ");
  Serial.print(GNSS_BAUD);
  Serial.println(" baud! Saving to Flash");
  myGNSS.saveConfiguration(); // Save to flash
  gnssSerial.end();
}

void gnssBegin() {
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  if (!myGNSS.begin(gnssSerial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    gnssSerial.end();
    resetGnssBaudRate();
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL);
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

void gnssPoll() { myGNSS.checkUblox(); }

bool gnssHasNewEpoch() {
  if (!(myGNSS.getPVT() && myGNSS.packetUBXNAVPVT != nullptr))
    return false;

  static uint32_t lastITOW = 0;
  uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;
  if (currentITOW == lastITOW)
    return false; // Same epoch - nothing new
  lastITOW = currentITOW;
  return true;
}

const UBX_NAV_PVT_data_t *gnssLatestPvt() {
  return myGNSS.packetUBXNAVPVT ? &myGNSS.packetUBXNAVPVT->data : nullptr;
}

bool gnssHeadingValid() { return myGNSS.getHeadVehValid(); }
