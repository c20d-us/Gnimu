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
static Uart &gnssSerial = Serial1;

// PVT data cache and epoch state
static UBX_NAV_PVT_data_t latestPVT;
static bool newEpochAvailable = false;
// Latches true on the first epoch ever received and never clears. Distinct
// from newEpochAvailable, which is per-epoch and consumed: this one lets
// gnssLatestPvt() tell "no data yet" apart from "data, but stale".
static bool everReceivedPvt = false;

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
  everReceivedPvt = true;
}

// Every NMEA sentence the u-blox M10 interface description defines for UART1.
//
// setUART1Output(COM_TYPE_UBX) below clears the port's NMEA protocol bit, which
// suppresses NMEA at the output stage but leaves each sentence's CFG-MSGOUT
// rate untouched - a factory M10 keeps GGA, GLL, GSA, GSV, RMC and VTG at rate
// 1 underneath the filter, which tools/common/gnss_ver phase 5 will show you.
// Whether the receiver still composes a sentence it is going to discard is not
// documented either way, so the rates are zeroed as well rather than relying on
// the filter alone: it makes the intent explicit, and removes any composition
// cost if one exists.
//
// The full set is listed rather than only the six a factory module enables, so
// a receiver carrying some other stored config is covered too. A rejected key
// means that sentence does not exist on this firmware - there is nothing to
// disable, so it is not an error.
static const uint32_t NMEA_MSGOUT_KEYS[] = {
    UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1,
};
static const int NUM_NMEA_MSGOUT_KEYS =
    sizeof(NMEA_MSGOUT_KEYS) / sizeof(NMEA_MSGOUT_KEYS[0]);

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

  // Let the GNSS settle before pushing CFG-VALSET writes at it.
  delay(500);
  drainSerial();

  // Every setter below deliberately targets VAL_LAYER_RAM_BBR: config.h is
  // still the single source of truth, but BBR is what lets these settings
  // survive a LIGHT_SLEEP backup-mode cycle intact

  // AssistNow Autonomous is explicitly DISABLED.
  //
  // It works by computing predicted satellite orbits and holding them in the
  // receiver's backup RAM, to shorten TTFF on a later start where the broadcast
  // ephemeris has expired but the prediction is still good. That payoff needs
  // the stored data to survive the off period, and on this design it cannot:
  // g_power's peripheral-off paths cut the GNSS rail, which takes the module's
  // backup supply with it. LIGHT_SLEEP doesn't need it either - RXM-PMREQ keeps
  // the rail up, so ephemeris survives and the wake is already a hot start. The
  // window where AOP could help is therefore about as long as the rail stays up
  // after we stop using it, which is seconds.
  //
  // This is a write rather than a deleted call on purpose: earlier firmware
  // enabled AOP into the BBR config layer, so simply not asking for it would
  // leave it on wherever that layer survived.
  if (myGNSS.setAopCfg(0, 0, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("🚫 AssistNow Autonomous disabled.");
  } else {
    LOG_PRINTLN("❌ Failed to disable AssistNow Autonomous.");
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

  // Zero the NMEA sentence rates sitting behind that protocol filter. See
  // NMEA_MSGOUT_KEYS above for why the filter alone isn't the whole job.
  {
    int zeroed = 0;
    for (const auto &key : NMEA_MSGOUT_KEYS) {
      if (myGNSS.setVal8(key, 0, VAL_LAYER_RAM_BBR)) {
        zeroed++;
      }
    }
    LOG_PRINTF("✅ NMEA sentence rates zeroed (%d of %d; any remainder is "
               "unsupported by this firmware).\n",
               zeroed, NUM_NMEA_MSGOUT_KEYS);
    (void)zeroed; // only read by the log line, which silent builds compile out
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

  // Set the GNSS PVT update frequency.
  if (myGNSS.setNavigationFrequency(GNSS_NAV_RATE_HZ, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS update rate set to %dHz.\n", GNSS_NAV_RATE_HZ);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS update rate.");
  }

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

// GNSS module poller - called every loop().
// Prompts firing of registered callback when a new PVT epoch is available.
void gnssPoll() {
  // Pump the UART and parse incoming bytes into complete packets
  myGNSS.checkUblox();
  // Fire registered callbacks for any completed packets
  myGNSS.checkCallbacks();
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
