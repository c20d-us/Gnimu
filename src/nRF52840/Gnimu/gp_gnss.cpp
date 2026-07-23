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

#include "gp_gnss.h"
#include "config.h"
#include "gp_ble.h"
#include "gp_log.h"

// GNSS state
static SFE_UBLOX_GNSS_SERIAL myGNSS;
static Uart &gnssSerial = Serial1;

// PVT data cache and eopch state
static UBX_NAV_PVT_data_t latestPVT;
static bool newEpochAvailable = false;

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

    gnssSerial.begin(testBaud);
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
        gnssSerial.begin(GNSS_BAUD);
        delay(100);

        if (myGNSS.begin(gnssSerial)) {
          LOG_PRINTLN("⚡ Baud rate switched. Saving to flash...");
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

// Drain the GNSS serial buffer to clear any pending data.
static void drainSerial() {
  while (gnssSerial.available()) {
    gnssSerial.read();
  }
}

// The callback function triggered automatically by checkCallbacks()
// when a new UBX-NAV-PVT packet has been constructed.
static void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // Copy the new PVT data our local copy
  memcpy(&latestPVT, ubxDataStruct, sizeof(UBX_NAV_PVT_data_t));
  newEpochAvailable = true;
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
    if (myGNSS.enableGNSS(target.enabled, target.id, VAL_LAYER_RAM_BBR)) {
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

// Set the navigation frequency based on BLE client connection state.
static void setNavFreq() {
  // Navigation frequency to use when Gnimu is idle (no BLE clients connected).
  const uint8_t idleNavFreq = 1;
  // The current navigation frequency, updated as BLE clients
  // connect/disconnect.
  static uint8_t currentNavFreq = 0;

  // Check to see if there is a BLE client connected, and adjust the navigation
  // frequency accordingly.
  const uint8_t desiredNavFreq =
      bleIsConnected() ? GNSS_NAV_RATE_HZ : idleNavFreq;
  if (currentNavFreq != desiredNavFreq) {
    if (myGNSS.setNavigationFrequency(desiredNavFreq, VAL_LAYER_RAM_BBR)) {
      currentNavFreq = desiredNavFreq;
      LOG_PRINTF("✅ GNSS update rate set to %dHz.\n", desiredNavFreq);
    } else {
      LOG_PRINTLN("❌ Failed to reset GNSS update rate.");
    }
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

// Initialize the GNSS module.
void gnssBegin() {
  // Make sure we can connect to the GNSS module at the target baud rate.
  // If we can't connect, halt with an error message.
  if (!connectAndConfigureBaud()) {
    LOG_PRINTLN("❌ u-blox GNSS not detected at any standard baud rate.");
    LOG_PRINTLN("X Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // Let the GNSS settle before pushing CFG-VALSET writes at it.
  delay(500);
  drainSerial();

  // Every setter below deliberately targets VAL_LAYER_RAM_BBR: config.h is
  // still the single source of truth, but BBR is what lets these settings
  // survive a LIGHT_SLEEP backup-mode cycle intact

  // Enable AssistNow Autonomous
  if (myGNSS.setAopCfg(1, 0, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("✅ AssistNow Autonomous enabled.");
  } else {
    LOG_PRINTLN("X Failed to enable AssistNow Autonomous.");
  }

  // Set the GNSS dynamic model
  if (myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS dynamic model set to %d.\n", GNSS_DYNAMIC_MODEL);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS dynamic model.");
  }

  // Turn off NMEA messages - we want UBX only
  if (myGNSS.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("✅ NMEA messages disabled.");
  } else {
    LOG_PRINTLN("❌ Failed to disable NMEA messages.");
  }

  // Set the minimum elevation of satellites to track (anti-multipath)
  if (myGNSS.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, GNSS_SV_MINELEV_DEG,
                     VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS minimum SV elevation set to %dº.\n",
               GNSS_SV_MINELEV_DEG);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS minimum elevation.");
  }

  // Constellation setup
  setConstellations();

  // Set the GNSS PVT update frequency
  setNavFreq();

  // Register our callback and enable automatic PVT output.
  // setAutoPVTcallbackPtr() implicitly enables AutoPVT.
  if (myGNSS.setAutoPVTcallbackPtr(&pvtCallback, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("✅ PVT callback registered; automatic PVT output enabled.");
  } else {
    LOG_PRINTLN("❌ Failed to register PVT callback / enable automatic PVT.");
  }
}

// Release the UART peripheral. After this call D6/D7 are plain GPIO again,
// so powerHoldPeripheralsOff() can drive D6 LOW to cut the RX back-feed path.
void gnssEnd() { gnssSerial.end(); }

// Convenience wrapper around getHeadVehValid()
bool gnssHeadingValid() { return myGNSS.getHeadVehValid(); }

// GNSS module poller - called every loop().
// Prompts firing of registered callback when a new PVT epoch is available.
// Updates the Navigation Frequency as needed depending on BLE connection state.
void gnssPoll() {
  // Pump the UART and parse incoming bytes into complete packets
  myGNSS.checkUblox();
  // Fire registered callbacks for any completed packets
  myGNSS.checkCallbacks();
  // Set to the desired navigation frequency for our current state
  setNavFreq();
}

// LIGHT_SLEEP entry: RXM-PMREQ backup mode, infinite duration, UART-RX wake
// armed. Serial1 is deliberately left running so that gnssWake() can release
// it later for wake pulse.
void gnssSleep() {
  myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX);
  LOG_PRINTLN("💤 GNSS backup mode (UART-RX wake armed).");
}

// Wake from gnssSleep() via a manual GPIO pulse on the shared UART TX line.
void gnssWake() {
  gnssSerial.end();
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
  delay(GNSS_WAKE_PULSE_MS);
  digitalWrite(GNSS_TX_PIN, HIGH);
  delay(GNSS_WAKE_PULSE_MS);
  digitalWrite(GNSS_TX_PIN, LOW);
  gnssSerial.begin(GNSS_BAUD);

  // Re-sync with the receiver
  drainSerial();
  if (myGNSS.begin(gnssSerial)) {
    LOG_PRINTLN("⏰ GNSS wake pulse sent and Serial re-synced.");
  } else {
    LOG_PRINTLN("⚠️ GNSS wake pulse sent, but re-sync failed - receiver may "
                "not have woken.");
  }
}
