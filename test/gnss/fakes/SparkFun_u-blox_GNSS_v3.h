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
#pragma once

// Stand-in for the SparkFun u-blox GNSS v3 library: the surface g_gnss.cpp
// actually uses, and nothing else. Every call is RECORDED with its arguments -
// the recording is the point, because what must not change across the planned
// shared-core split is the sequence of calls the receiver sees.
//
// The data structures are the library's REAL ones (u-blox_structs.h compiles
// standalone - see ROB-6), so the PVT the callback receives is the shipping
// struct, not a copy of it.
#include <stddef.h>
#include <stdint.h>

#include "u-blox_structs.h"

#include "Arduino.h"     // FakeSerialPort
#include "fake_gnss.h"

// --- Constants g_gnss.cpp names ---------------------------------------------
#define COM_TYPE_UBX 0x01
#define VAL_CFG_SUBSEC_IOPORT 0x01
#define VAL_LAYER_RAM_BBR 0x03

// Configuration keys. The values are the real ones where g_gnss.cpp's log
// prints them; for the NMEA set only identity matters, so they are numbered in
// the interface description's order.
#define UBLOX_CFG_NAVSPG_INFIL_MINELEV 0x201100a4
#define UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1 0x2091a087
#define UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1 0x2091a088
#define UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1 0x2091a089
#define UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1 0x2091a08a
#define UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1 0x2091a08b
#define UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1 0x2091a08c
#define UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1 0x2091a08d
#define UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1 0x2091a08e
#define UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1 0x2091a08f
#define UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1 0x2091a090
#define UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1 0x2091a091
#define UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1 0x2091a092
#define UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1 0x2091a093
#define UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1 0x2091a094

enum sfe_ublox_gnss_ids_e {
  SFE_UBLOX_GNSS_ID_GPS = 0,
  SFE_UBLOX_GNSS_ID_SBAS = 1,
  SFE_UBLOX_GNSS_ID_GALILEO = 2,
  SFE_UBLOX_GNSS_ID_BEIDOU = 3,
  SFE_UBLOX_GNSS_ID_QZSS = 5,
  SFE_UBLOX_GNSS_ID_GLONASS = 6,
};

enum dynModel { DYN_MODEL_PORTABLE = 0, DYN_MODEL_AUTOMOTIVE = 4 };

// --- The library ------------------------------------------------------------
class SFE_UBLOX_GNSS_SERIAL {
public:
  // Answers only if the port is open at the receiver's baud - which is what
  // makes the baud sweep in connectAndConfigureBaud() a real test.
  bool begin(Stream &port);

  bool setSerialRate(unsigned long baud, uint8_t layer = VAL_LAYER_RAM_BBR);
  bool saveConfigSelective(uint32_t subsection);
  bool enableGNSS(bool enable, sfe_ublox_gnss_ids_e id,
                  uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setAopCfg(uint8_t aopCfg, uint16_t maxWait = 0,
                 uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setDynamicModel(dynModel model, uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setUART1Output(uint8_t comType, uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setVal8(uint32_t key, uint8_t value, uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setNavigationFrequency(uint8_t hz, uint8_t layer = VAL_LAYER_RAM_BBR);
  bool setAutoPVTcallbackPtr(void (*cb)(UBX_NAV_PVT_data_t *),
                             uint8_t layer = VAL_LAYER_RAM_BBR);
  void checkUblox();
  void checkCallbacks();
};
