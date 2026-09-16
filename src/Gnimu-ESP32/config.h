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

#pragma once

// Build configuration.
//
// Section 1 (Tunables) holds values expected to change.
// Section 2 (Supporting constants) describes fixed hardware and derived values.
// Both sections list subsystems in the same order.
// Names are prefixed by subsystem and suffixed by unit.
// Compile-time checks are at the end.

// IMU sample rate, filtering, and trim values shared by all boards.
#include "g_imu_tuning.h"

// ----------------------------------------------------------------------------
// Section 1: Tunables
// ----------------------------------------------------------------------------

// Device identity

// Exactly 10 digits, quoted so leading zeros survive.
// First digit 0-3: the RaceBox app rejects IDs of 4000000000 or higher.
#define DEVICE_ID "1000000001"

// Build name shown in the boot banner.
#define GNIMU_VARIANT "ESP32"

// IMU (MPU-6050)

// 1 if an MPU-6050 is fitted. With 0 the driver is a stub (no Adafruit library
// needed) and IMU fields read zero.
#define IMU_ENABLED 1

// 0x68 with AD0 low, 0x69 with AD0 high.
#define IMU_I2C_ADDRESS 0x68

// Ranges and built-in low-pass filter, as Adafruit MPU6050 enum values.
#define IMU_ACCEL_RANGE_G MPU6050_RANGE_4_G
#define IMU_GYRO_RANGE_DPS MPU6050_RANGE_500_DEG
#define IMU_FILTER_BANDWIDTH_HZ MPU6050_BAND_21_HZ

// I2C clock. The bus defaults to 100kHz; faster transfers block gnssPoll() for
// less time. Applied after myIMU.begin().
#define IMU_I2C_CLOCK_HZ 400000

// Axis orientation
//
// Maps sensor axes to the vehicle frame: X forward, Y left, Z up (ISO 8855).
// Each vehicle axis takes a sensor axis (0=X, 1=Y, 2=Z) and a sign.
//
// A valid map is a rotation: the number of sign flips must match the
// permutation's parity (checked below).
//
//   Order   _SRC triple   Parity   Sign flips
//   XYZ     0, 1, 2       even     0 or 2
//   XZY     0, 2, 1       odd      1 or 3
//   YXZ     1, 0, 2       odd      1 or 3
//   YZX     1, 2, 0       even     0 or 2
//   ZXY     2, 0, 1       even     0 or 2
//   ZYX     2, 1, 0       odd      1 or 3
//
// To derive a map, hold the unit as installed and watch the serial mG values:
//   1. At rest, the axis reading about +/-1000 is vertical; the sign gives up.
//   2. Raise the front: the axis going positive is X.
//   3. Raise the left side: the axis going positive is Y.
#define IMU_AXIS_X_SRC 0 // vehicle forward <- sensor X
#define IMU_AXIS_X_SIGN +1.0f
#define IMU_AXIS_Y_SRC 1 // vehicle left    <- sensor Y
#define IMU_AXIS_Y_SIGN +1.0f
#define IMU_AXIS_Z_SRC 2 // vehicle up      <- sensor Z
#define IMU_AXIS_Z_SIGN +1.0f

// GNSS (u-blox)

#define GNSS_BAUD 115200      // higher rates can lower the PVT rate
#define GNSS_NAV_RATE_HZ 20   // max 20 with 2 constellations
#define GNSS_SV_MINELEV_DEG 5 // elevation mask
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

// GNSS UART pins.
#define GNSS_RX_PIN 16
#define GNSS_TX_PIN 17

// Constellations. More constellations lower the maximum PVT rate: up to 2 for
// 20Hz, 1 for 25Hz. Coverage: https://app.qzss.go.jp/GNSSView/gnssview.html
#define GNSS_CONSTELLATIONS                                                    \
  {                                                                            \
      {"GPS", SFE_UBLOX_GNSS_ID_GPS, true},                                    \
      {"Galileo", SFE_UBLOX_GNSS_ID_GALILEO, true},                            \
      {"GLONASS", SFE_UBLOX_GNSS_ID_GLONASS, false},                           \
      {"BeiDou", SFE_UBLOX_GNSS_ID_BEIDOU, false},                             \
      {"QZSS", SFE_UBLOX_GNSS_ID_QZSS, false},                                 \
      {"SBAS", SFE_UBLOX_GNSS_ID_SBAS, false},                                 \
  }

// BLE

// TX power. Lower power reduces interference with GNSS; raise it if
// connections drop. Valid levels: -12, -9, -6, -3, 0, 3, 6, 9.
#define BLE_TX_POWER_ADV_DBM -12  // advertising
#define BLE_TX_POWER_CONN_DBM -12 // connected

#define BLE_READVERTISE_DELAY_MS 500 // delay before re-advertising

// LED (onboard)

#define LED_ONBOARD_PIN 2
#define LED_BLINK_INTERVAL_MS 1000 // blink while disconnected

// Logging

// 1: serial logging enabled.
// 0: silent.
#define LOG_ENABLED 1

#define LOG_STATS_INTERVAL_MS 1000 // stats line interval

// Protocol

// Telemetry protocol, one of the PROTO_* IDs in g_protocol.h. Compile-time;
// protocol constants live in g_proto_<name>.h.
#define TELEMETRY_PROTOCOL PROTO_RACEBOX

// ----------------------------------------------------------------------------
// Section 2: Supporting constants
// ----------------------------------------------------------------------------

// Battery

// No battery on this board; the RaceBox battery byte reports this value.
#define BATTERY_REPORT_PERCENT 100

// 0: no battery sensing (g_battery is a stub).
#define BATTERY_HAS_GAUGE 0

// ----------------------------------------------------------------------------
// Compile-time validation
// ----------------------------------------------------------------------------

// DEVICE_ID: 10 digits, first digit 0-3.
namespace device_id {
constexpr int length(const char *s) { return *s ? 1 + length(s + 1) : 0; }
constexpr bool allDigits(const char *s) {
  return *s == '\0'                 ? true
         : (*s >= '0' && *s <= '9') ? allDigits(s + 1)
                                    : false;
}
} // namespace device_id

static_assert(
    device_id::length(DEVICE_ID) == 10,
    "ERROR: DEVICE_ID must be exactly 10 digits, quoted as a string.");
static_assert(device_id::allDigits(DEVICE_ID),
              "ERROR: DEVICE_ID must contain only digits 0-9.");
static_assert(DEVICE_ID[0] >= '0' && DEVICE_ID[0] <= '3',
              "ERROR: DEVICE_ID's first digit must be 0-3 (value below "
              "4000000000).");

static_assert(GNSS_RX_PIN >= 0 && GNSS_RX_PIN <= 39 && GNSS_TX_PIN >= 0 &&
                  GNSS_TX_PIN <= 39 && LED_ONBOARD_PIN >= 0 &&
                  LED_ONBOARD_PIN <= 39,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and LED_ONBOARD_PIN must be "
              "valid ESP32 GPIO numbers (0-39).");
static_assert(GNSS_RX_PIN != GNSS_TX_PIN && GNSS_RX_PIN != LED_ONBOARD_PIN &&
                  GNSS_TX_PIN != LED_ONBOARD_PIN,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and LED_ONBOARD_PIN must all "
              "be different pins.");

static_assert(GNSS_NAV_RATE_HZ > 0 && GNSS_NAV_RATE_HZ <= 25,
              "ERROR: GNSS_NAV_RATE_HZ must be between 1 and 25.");

static_assert(GNSS_SV_MINELEV_DEG >= 0 && GNSS_SV_MINELEV_DEG <= 90,
              "ERROR: GNSS_SV_MINELEV_DEG must be between 0 and 90 degrees.");

// Each epoch must span at least one IMU sample.
static_assert((1000 / GNSS_NAV_RATE_HZ) >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: the GNSS epoch interval (1000 / GNSS_NAV_RATE_HZ) must "
              "be >= IMU_SAMPLE_INTERVAL_MS, so each transmitted sample's "
              "window contains at least one fresh IMU reading for ImuAxis's "
              "transient peak tracking to work.");

static_assert(BLE_READVERTISE_DELAY_MS > 0,
              "ERROR: BLE_READVERTISE_DELAY_MS must be greater than 0.");
// Checked in dBm: the enum's compatibility aliases would accept -14 or +7.
static_assert(BLE_TX_POWER_ADV_DBM >= -12 && BLE_TX_POWER_ADV_DBM <= 9 &&
                  (BLE_TX_POWER_ADV_DBM + 12) % 3 == 0,
              "ERROR: BLE_TX_POWER_ADV_DBM must be -12, -9, -6, -3, 0, 3, 6 or "
              "9 dBm.");
static_assert(
    BLE_TX_POWER_CONN_DBM >= -12 && BLE_TX_POWER_CONN_DBM <= 9 &&
        (BLE_TX_POWER_CONN_DBM + 12) % 3 == 0,
    "ERROR: BLE_TX_POWER_CONN_DBM must be -12, -9, -6, -3, 0, 3, 6 or "
    "9 dBm.");
static_assert(LED_BLINK_INTERVAL_MS > 0,
              "ERROR: LED_BLINK_INTERVAL_MS must be greater than 0.");
static_assert(LOG_STATS_INTERVAL_MS > 0,
              "ERROR: LOG_STATS_INTERVAL_MS must be greater than 0.");

static_assert(LOG_ENABLED == 0 || LOG_ENABLED == 1,
              "ERROR: LOG_ENABLED must be 0 or 1.");

static_assert(IMU_ENABLED == 0 || IMU_ENABLED == 1,
              "ERROR: IMU_ENABLED must be 0 or 1.");
static_assert(IMU_I2C_ADDRESS == 0x68 || IMU_I2C_ADDRESS == 0x69,
              "ERROR: IMU_I2C_ADDRESS must be 0x68 (AD0 low) or 0x69 (AD0 "
              "high) - the only two addresses an MPU-6050 can have.");

static_assert(IMU_AXIS_X_SIGN == 1.0f || IMU_AXIS_X_SIGN == -1.0f,
              "ERROR: IMU_AXIS_X_SIGN must be exactly +1.0f or -1.0f.");
static_assert(IMU_AXIS_Y_SIGN == 1.0f || IMU_AXIS_Y_SIGN == -1.0f,
              "ERROR: IMU_AXIS_Y_SIGN must be exactly +1.0f or -1.0f.");
static_assert(IMU_AXIS_Z_SIGN == 1.0f || IMU_AXIS_Z_SIGN == -1.0f,
              "ERROR: IMU_AXIS_Z_SIGN must be exactly +1.0f or -1.0f.");

static_assert(IMU_AXIS_X_SRC >= 0 && IMU_AXIS_X_SRC <= 2,
              "ERROR: IMU_AXIS_X_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");
static_assert(IMU_AXIS_Y_SRC >= 0 && IMU_AXIS_Y_SRC <= 2,
              "ERROR: IMU_AXIS_Y_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");
static_assert(IMU_AXIS_Z_SRC >= 0 && IMU_AXIS_Z_SRC <= 2,
              "ERROR: IMU_AXIS_Z_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");

// Permutation parity: +1 even, -1 odd, 0 if two _SRC values match.
#define IMU_AXIS_PARITY                                                        \
  ((((IMU_AXIS_Y_SRC) - (IMU_AXIS_X_SRC)) *                                    \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_X_SRC)) *                                    \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_Y_SRC))) /                                   \
   2)

static_assert(IMU_AXIS_PARITY != 0,
              "ERROR: IMU_AXIS_X/Y/Z_SRC must be a permutation - each of "
              "sensor 0, 1 and 2 used exactly once. Two vehicle axes are "
              "currently fed by the same sensor axis.");

// Determinant must be +1 (parity x signs). -1 is a mirror, which no mounting
// can produce.
static_assert(IMU_AXIS_PARITY * IMU_AXIS_X_SIGN * IMU_AXIS_Y_SIGN *
                      IMU_AXIS_Z_SIGN ==
                  1.0f,
              "ERROR: axis map is a mirror, not a rotation (determinant -1). "
              "An even permutation needs an even number of sign flips, an odd "
              "permutation an odd number. See the order table in the Axis "
              "orientation section above.");

static_assert(BATTERY_REPORT_PERCENT >= 0 && BATTERY_REPORT_PERCENT <= 100,
              "ERROR: BATTERY_REPORT_PERCENT must be between 0 and 100.");

static_assert(BATTERY_HAS_GAUGE == 0 || BATTERY_HAS_GAUGE == 1,
              "ERROR: BATTERY_HAS_GAUGE must be 0 or 1.");
