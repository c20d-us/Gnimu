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

#include "g_gnss.h"
#include "config.h"
#include "g_log.h"

// GNSS state
static SFE_UBLOX_GNSS_SERIAL myGNSS;
static HardwareSerial gnssSerial(2);

// PVT data and state
static UBX_NAV_PVT_data_t latestPVT;
static bool newEpochAvailable = false;
// Latches true on the first epoch ever received and never clears. Distinct
// from newEpochAvailable, which is per-epoch and consumed: this one lets
// gnssLatestPvt() tell "no data yet" apart from "data, but stale".
static bool everReceivedPvt = false;

// The callback function triggered automatically by checkCallbacks()
// when a new UBX-NAV-PVT packet has been constructed.
static void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // Copy the new PVT data our local copy
  memcpy(&latestPVT, ubxDataStruct, sizeof(UBX_NAV_PVT_data_t));
  newEpochAvailable = true;
  everReceivedPvt = true;
}

// Try connecting to the GNSS at the baud rate from config.h, and if that fails,
// sweep through all common u-blox baud rates to find the module and reconfigure
// it.
static bool connectAndConfigureBaud() {
  // Array of baud rates to test. We test the config.h rate first for the
  // fastest normal boot, followed by common u-blox rates.
  const uint32_t baudRates[] = {GNSS_BAUD, 9600,   38400, 57600,
                                115200,    230400, 460800};
  const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);

  for (int i = 0; i < numRates; i++) {
    uint32_t testBaud = baudRates[i];
    LOG_PRINTF("🔎 Trying GNSS at %d baud...\n", testBaud);

    gnssSerial.begin(testBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    delay(100); // Give the serial port a moment to stabilize

    if (myGNSS.begin(gnssSerial)) {
      LOG_PRINTF("✅ GNSS detected at %d baud.\n", testBaud);

      // If we found it, but it's at the wrong speed, switch it.
      if (testBaud != GNSS_BAUD) {
        LOG_PRINTF("🔀 Switching GNSS to target %d baud...\n", GNSS_BAUD);
        myGNSS.setSerialRate(GNSS_BAUD);
        delay(100);

        // Cycle the microcontroller's UART to match the new module speed
        gnssSerial.end();
        delay(100);
        gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
        delay(100);

        if (myGNSS.begin(gnssSerial)) {
          LOG_PRINTLN("⚡ Baud rate switched. Saving to flash...");
          // Save ONLY the I/O-port (baud) subsection. The rest of the
          // config isn't applied until gnssBegin() below, so a full save
          // here would persist a partial config and wear flash needlessly.
          myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
          return true;
        } else {
          LOG_PRINTLN("❌ Failed to verify new baud rate.");
          return false; // Something went deeply wrong
        }
      }
      return true; // We connected successfully at the target baud rate
    }

    // Clean up and prepare for the next loop iteration if this baud failed
    gnssSerial.end();
    delay(100);
  }
  return false; // Swept everything and never connected
}

// Set up the constellations defined in config.h
// Explicitly disable unwanted constellations to ensure that only the desired
// constellations are active.
static void setConstellations() {
  // Struct to hold the constellation configuration from config.h
  struct Constellations {
    const char *name;
    sfe_ublox_gnss_ids_e id;
    bool enabled;
  };

  // Instantiate the array directly from the config macro
  const Constellations targetConstellations[] = GNSS_CONSTELLATIONS;

  LOG_PRINTLN("🛰️ Enabling GNSS constellations...");

  for (const auto &target : targetConstellations) {
    if (myGNSS.enableGNSS(target.enabled, target.id)) {
      if (target.enabled) {
        LOG_PRINTF("✅ %s enabled.\n", target.name);
      } else {
        LOG_PRINTF("🚫 %s disabled.\n", target.name);
      }
    } else {
      if (target.enabled) {
        // We wanted it ON, but the chip rejected it. Real error.
        LOG_PRINTF("❌ Failed to enable %s.\n", target.name);
      } else {
        // We wanted it OFF, and the chip rejected it. It's unsupported.
        LOG_PRINTF("⚪ %s unsupported.\n", target.name);
      }
    }
  }
}

// Drain the GNSS serial buffer to clear any pending data.
static void drainSerial() {
  while (gnssSerial.available()) {
    gnssSerial.read();
  }
}

// Initialize the GNSS module.
void gnssBegin() {
  // Make sure we can connect to the GNSS module at the target baud rate.
  // If we can't connect, halt with an error message.
  if (!connectAndConfigureBaud()) {
    LOG_PRINTLN("❌ u-blox GNSS not detected at any standard baud rate.");
    LOG_PRINTLN("❌ Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // Let the GNSS settle before pushing config writes at it, and clear any
  // bytes buffered during the baud handshake so the first CFG ACKs parse
  // cleanly.
  delay(500);
  drainSerial();

  // Enable AssistNow Autonomous, letting the receiver predict satellite
  // orbits for faster reacquisition after signal loss.
  if (myGNSS.setAopCfg(1, 0)) {
    LOG_PRINTLN("✅ AssistNow Autonomous enabled.");
  } else {
    LOG_PRINTLN("❌ Failed to enable AssistNow Autonomous.");
  }

  // Set the GNSS dynamic model
  if (myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL)) {
    LOG_PRINTF("✅ GNSS dynamic model set to %d.\n", GNSS_DYNAMIC_MODEL);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS dynamic model.");
  }

  // Turn off NMEA messages - we want UBX only
  if (myGNSS.setUART1Output(COM_TYPE_UBX)) {
    LOG_PRINTLN("✅ NMEA messages disabled.");
  } else {
    LOG_PRINTLN("❌ Failed to disable NMEA messages.");
  }

  // Set the minimum elevation of satellites to track (anti-multipath)
  if (myGNSS.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, GNSS_SV_MINELEV_DEG)) {
    LOG_PRINTF("✅ GNSS minimum SV elevation set to %d deg.\n",
               GNSS_SV_MINELEV_DEG);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS minimum elevation.");
  }

  // Constellation setup
  setConstellations();

  // Set the GNSS PVT update frequency.
  if (myGNSS.setNavigationFrequency(GNSS_NAV_RATE_HZ)) {
    LOG_PRINTF("✅ GNSS update rate set to %dHz.\n", GNSS_NAV_RATE_HZ);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS update rate.");
  }

  // Register the PVT callback and enable automatic PVT output LAST, once
  // the module is fully configured. setAutoPVTcallbackPtr() implicitly enables
  // AutoPVT, so no separate setAutoPVT(true) call is needed.
  if (myGNSS.setAutoPVTcallbackPtr(&pvtCallback)) {
    LOG_PRINTLN("✅ PVT callback registered; auto PVT output enabled.");
  } else {
    LOG_PRINTLN("❌ Failed to register PVT callback / enable auto PVT.");
  }
}

// Consume the cached PVT data if available, returning nullptr if no new data.
const UBX_NAV_PVT_data_t *gnssConsumePvt() {
  if (!newEpochAvailable) {
    return nullptr; // No new data since last time
  } else {
    newEpochAvailable = false; // "Consume" the flag
    return &latestPVT;         // Return the fresh data
  }
}

// Non-consuming peek - see the header for why read-only observers must use
// this rather than gnssConsumePvt(). Guarded by a latching "have we ever
// received one" flag so callers never see the zero-initialised struct as if it
// were a real fix.
const UBX_NAV_PVT_data_t *gnssLatestPvt() {
  return everReceivedPvt ? &latestPVT : nullptr;
}

// GNSS module poller - called every loop().
// Prompts firing of registered callback when a new PVT epoch is available.
void gnssPoll() {
  // Pump the UART and parse incoming bytes into complete packets
  myGNSS.checkUblox();
  // Fire the registered callbacks for any completed packets
  myGNSS.checkCallbacks();
}
