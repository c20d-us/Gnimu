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

#include "g_gnss.h"
#include "config.h"
#include "g_gnss_port.h"
#include "g_log.h"

static SFE_UBLOX_GNSS_SERIAL myGNSS;
static Stream *gnssStream = nullptr;

// PVT cache and epoch state
static UBX_NAV_PVT_data_t latestPVT;
static bool newEpochAvailable = false;
// Set on the first epoch and never cleared.
static bool everReceivedPvt = false;

// millis() of the latest epoch, or of bring-up before the first one.
static unsigned long lastEpochMs = 0;

// Stall threshold: the larger of 1s and three epoch periods.
static constexpr unsigned long kStallMs = (3000UL / GNSS_NAV_RATE_HZ) > 1000UL
                                              ? (3000UL / GNSS_NAV_RATE_HZ)
                                              : 1000UL;

// Find the receiver, trying GNSS_BAUD first and then common u-blox rates. If
// found at another rate, switch it to GNSS_BAUD and save that to flash.
static bool connectAndConfigureBaud() {
  const uint32_t baudRates[] = {GNSS_BAUD, 4800,   9600,   19200,  38400,
                                57600,     115200, 230400, 460800, 921600};
  const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);

  for (int i = 0; i < numRates; i++) {
    uint32_t testBaud = baudRates[i];
    LOG_PRINTF("🔎 Trying GNSS at %u baud...\n", (unsigned int)testBaud);

    gnssStream = gnssPortBegin(testBaud);
    delay(100); // let the port settle

    if (gnssStream != nullptr && myGNSS.begin(*gnssStream)) {
      LOG_PRINTF("✅ GNSS detected at %u baud.\n", (unsigned int)testBaud);

      if (testBaud != GNSS_BAUD) {
        LOG_PRINTF("🔀 Switching GNSS to target %u baud...\n",
                   (unsigned int)GNSS_BAUD);
        myGNSS.setSerialRate(GNSS_BAUD);
        delay(100);

        // Reopen the UART at the new rate.
        gnssPortEnd();
        delay(100);
        gnssStream = gnssPortBegin(GNSS_BAUD);
        delay(100);

        if (gnssStream != nullptr && myGNSS.begin(*gnssStream)) {
          LOG_PRINTLN("⚡ Baud rate switched. Saving to flash...");
          myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
          return true;
        } else {
          LOG_PRINTLN("❌ Failed to verify new baud rate.");
          return false;
        }
      }
      return true;
    }

    gnssPortEnd();
    delay(100);
  }
  return false;
}

// Discard any pending UART bytes.
static void drainSerial() {
  while (gnssStream != nullptr && gnssStream->available()) {
    gnssStream->read();
  }
}

// Called by checkCallbacks() for each new UBX-NAV-PVT.
static void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
  memcpy(&latestPVT, ubxDataStruct, sizeof(UBX_NAV_PVT_data_t));
  newEpochAvailable = true;
  everReceivedPvt = true;
  lastEpochMs = millis();
}

// Every M10 UART1 NMEA sentence. setUART1Output(COM_TYPE_UBX) filters NMEA but
// leaves each sentence's rate set, so the rates are zeroed too. A rejected key
// is a sentence this firmware lacks, not an error.
static const uint32_t NMEA_MSGOUT_KEYS[] = {
    UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1,
};

// Enable or disable each constellation listed in GNSS_CONSTELLATIONS.
static void setConstellations() {
  struct Constellations {
    const char *name;
    sfe_ublox_gnss_ids_e id;
    bool enabled;
  };

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
        LOG_PRINTF("❌ Failed to enable %s.\n", target.name);
      } else {
        // A rejected disable means the receiver doesn't support it.
        LOG_PRINTF("⚪ %s unsupported.\n", target.name);
      }
    }
  }
}

const UBX_NAV_PVT_data_t *gnssConsumePvt() {
  if (!newEpochAvailable) {
    return nullptr;
  } else {
    newEpochAvailable = false;
    return &latestPVT;
  }
}

const UBX_NAV_PVT_data_t *gnssLatestPvt() {
  return everReceivedPvt ? &latestPVT : nullptr;
}

static bool gnssUp = false;

bool gnssIsUp() { return gnssUp; }

bool gnssStalled() { return gnssUp && (millis() - lastEpochMs) > kStallMs; }

bool gnssBegin() {
  // No receiver: report and continue without telemetry. No retry; the library
  // already polls each baud rate three times.
  if (!connectAndConfigureBaud()) {
    LOG_PRINTLN("❌ u-blox GNSS not detected at any standard baud rate.");
    LOG_PRINTLN("❌ Check your wiring. Continuing WITHOUT GNSS: no telemetry "
                "will be produced; everything else keeps running.");
    LOG_PRINTLN("❌ Power-cycle the device to try again.");
    gnssUp = false;
    return false;
  }

  // Let the receiver settle before configuring it.
  delay(500);
  drainSerial();

  // All settings go to RAM/BBR and are reapplied every boot. The only flash
  // write is the baud rate, saved by connectAndConfigureBaud() after a switch.

  // if (myGNSS.setAopCfg(0, 0, VAL_LAYER_RAM_BBR)) {
  //   LOG_PRINTLN("🚫 AssistNow Autonomous disabled.");
  // } else {
  //   LOG_PRINTLN("❌ Failed to disable AssistNow Autonomous.");
  // }

  if (myGNSS.setDynamicModel(GNSS_DYNAMIC_MODEL, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS dynamic model set to %d.\n", GNSS_DYNAMIC_MODEL);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS dynamic model.");
  }

  // UBX only.
  if (myGNSS.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("✅ NMEA messages disabled.");
  } else {
    LOG_PRINTLN("❌ Failed to disable NMEA messages.");
  }

  // Zero the NMEA sentence rates (see NMEA_MSGOUT_KEYS).
  {
    int zeroed = 0;
    for (const auto &key : NMEA_MSGOUT_KEYS) {
      if (myGNSS.setVal8(key, 0, VAL_LAYER_RAM_BBR)) {
        zeroed++;
      }
    }
    LOG_PRINTF("✅ NMEA sentence rates zeroed (%d of %d).\n", zeroed,
               (int)(sizeof(NMEA_MSGOUT_KEYS) / sizeof(NMEA_MSGOUT_KEYS[0])));
    (void)zeroed; // unused when logging is compiled out
  }

  if (myGNSS.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, GNSS_SV_MINELEV_DEG,
                     VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS minimum SV elevation set to %d°.\n",
               GNSS_SV_MINELEV_DEG);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS minimum elevation.");
  }

  setConstellations();

  if (myGNSS.setNavigationFrequency(GNSS_NAV_RATE_HZ, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTF("✅ GNSS update rate set to %dHz.\n", GNSS_NAV_RATE_HZ);
  } else {
    LOG_PRINTLN("❌ Failed to set GNSS update rate.");
  }

  // Registering the callback also enables automatic PVT output.
  if (myGNSS.setAutoPVTcallbackPtr(&pvtCallback, VAL_LAYER_RAM_BBR)) {
    LOG_PRINTLN("✅ PVT callback registered; automatic PVT output enabled.");
  } else {
    LOG_PRINTLN("❌ Failed to register PVT callback / enable automatic PVT.");
  }

  lastEpochMs = millis(); // start the stall clock
  gnssUp = true;
  return true;
}

// Not guarded on gnssUp: power-cut callers need the UART released regardless.
void gnssEnd() {
  gnssPortEnd();
  gnssStream = nullptr;
  gnssUp = false;
}

void gnssPoll() {
  if (!gnssUp) {
    return;
  }
  myGNSS.checkUblox();     // parse incoming bytes
  myGNSS.checkCallbacks(); // fire callbacks for completed packets
}
