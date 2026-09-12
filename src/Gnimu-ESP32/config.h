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

// This file has two top-level sections:
//   1. TUNABLES - values you should expect to change: thresholds, timing,
//      smoothing, sensor/feature choices, per-build calibration, and the
//      wiring choices you made yourself (which GPIO you routed a signal to).
//   2. SUPPORTING CONSTANTS - values fixed by the RaceBox protocol or derived
//      from other constants. Changing them without changing the matching part
//      of the system (or the protocol) will break compatibility.
// Within each section, entries are grouped by subsystem (Device Identity, IMU,
// GNSS, BLE, LED, Logging, Battery, Protocol), in the same order in both
// sections. Every define is prefixed with the subsystem it belongs to, and
// every value that carries a unit is suffixed with it (_MS, _HZ, _G, _DPS,
// _MPS, _DEG, _BYTES, _PERCENT, _PIN).

// IMU TUNING IS NOT IN THIS FILE. The sample interval, filter smoothing,
// transient thresholds and runtime-trim values are the same on every board, so
// they live in g_imu_tuning.h, which check_common.sh keeps identical across all
// three trees. What stays here is what really differs per board: whether an
// IMU is fitted, its address, pins, ranges and rates, and how it is mounted.
#include "g_imu_tuning.h"

// ============================================================================
// ============================================================================
// SECTION 1: TUNABLES
// ============================================================================
// ============================================================================

// ----------------------------------------------------------------------------
// --- Device Identity ---
// ----------------------------------------------------------------------------

// Change DEVICE_ID to personalize your device.
// It is a STRING of exactly 10 digits. Quote it, so that leading zeros are kept
// (e.g. "0123456789"). Do NOT use a bare number: a leading zero would be read
// as an octal literal and an unquoted ID loses its leading zeros.
// First digit must be 0-3, so the value stays below 4000000000. The RaceBox
// app will not connect to IDs of 4000000000 or higher. See compile-time
// validation at the bottom of this file.
#define DEVICE_ID "1000000001"

// Identifies which build this binary is, printed as the first line of the
// startup banner. Purely diagnostic - nothing branches on it.
//
// Less critical here than on the nRF52840 variants, which share an MCU and so
// can be cross-flashed: this one needs a different core and board selection
// entirely. It still tells you which source tree a running binary came from
// when several IDE windows are open.
#define GNIMU_VARIANT "ESP32"

// ----------------------------------------------------------------------------
// --- IMU (MPU-6050) ---
// ----------------------------------------------------------------------------

// Is an IMU fitted at all? It is optional: RaceChrono never uses IMU data, and
// the RaceBox protocol works without it (it reports zero g), so an ESP32 +
// GNSS build with no MPU-6050 is a legitimate, cheaper device. With 0, the
// MPU-6050 driver compiles to a stub - so the Adafruit MPU6050 library is not
// needed to build at all - the IMU fields read zero exactly as if the IMU had
// failed, trim never runs, and the log says "not fitted" rather than "not
// found". Set by hand: the MPU-6050 is an external part, so nothing about the
// board says whether it is wired in.
#define IMU_ENABLED 1

// I2C address of the MPU-6050: 0x68 with its AD0 pin low (the common breakout
// default), 0x69 with AD0 high. Also a clean way to rehearse a missing IMU on
// a board where it is soldered in: point this at the address nothing answers.
#define IMU_I2C_ADDRESS 0x68

// Sensor full-scale ranges and the built-in low-pass bandwidth
// (Uses Adafruit MPU6050 enum tokens)
#define IMU_ACCEL_RANGE_G MPU6050_RANGE_4_G        // 4g, ample for auto-x
#define IMU_GYRO_RANGE_DPS MPU6050_RANGE_500_DEG   // 500 deg/s for auto-x
#define IMU_FILTER_BANDWIDTH_HZ MPU6050_BAND_21_HZ // built-in low-pass filter

// I2C bus speed for the MPU-6050.
//
// This is a LATENCY setting: imuPoll() reads the sensor every
// IMU_SAMPLE_INTERVAL_MS, and every millisecond spent blocked in that transfer
// is a millisecond gnssPoll() is not draining the GNSS UART.
//
// The default is NOT 400kHz and has to be set explicitly. Wire.begin() leaves
// the ESP32 bus at 100kHz; Adafruit_BusIO exposes setSpeed() but the MPU-6050
// library never calls it, so nothing raises it on its own. The MPU-6050
// driver (g_imu_mpu6050.cpp) applies this AFTER myIMU.begin(). begin() brings the bus up and would overwrite any
// earlier setting.
#define IMU_I2C_CLOCK_HZ 400000

// Sample interval, smoothing, transient thresholds and runtime trim: shared by
// every board, in g_imu_tuning.h (included at the top of this file).

// Axis orientation (installed mounting)
//
// Corrects the sensor's raw axes into the vehicle frame. What varies per BUILD
// is how the MPU-6050 module sits in your enclosure. Each VEHICLE axis below
// names which SENSOR axis feeds it (0=X, 1=Y, 2=Z) plus a sign. This covers all
// 24 physically-realizable orientations.
//
// TARGET OUTPUT FRAME: X forward+, Y left+, Z up+ (ISO 8855, right-handed).
//
// ORDER NAMES read as "which sensor axis feeds vehicle X, Y, Z". Because the
// target frame is right-handed, a valid map is always a proper rotation, so
// the permutation's parity must match the number of sign flips - odd needs
// odd, even needs even. The static_assert at the bottom of this file enforces
// it; a map that fails is a mirror, which no physical mounting can produce.
//
//   Order   _SRC triple   Parity   Sign flips required
//   XYZ     0, 1, 2       even     even (0 or 2)
//   XZY     0, 2, 1       odd      odd  (1 or 3)
//   YXZ     1, 0, 2       odd      odd  (1 or 3)
//   YZX     1, 2, 0       even     even (0 or 2)
//   ZXY     2, 0, 1       even     even (0 or 2)
//   ZYX     2, 1, 0       odd      odd  (1 or 3)
//
// TO DERIVE A NEW MAP: hold the assembled unit in its installed orientation and
// read the 1 Hz serial mG line (LOG must be enabled):
//   1. At rest, the axis reading ~+/-1000 is vehicle-vertical; sign gives
//      up vs down.
//   2. Raise the forward end - the axis going positive is vehicle X.
//   3. Raise the left side  - the axis going positive is vehicle Y.
#define IMU_AXIS_X_SRC 0 // vehicle forward <- sensor X
#define IMU_AXIS_X_SIGN +1.0f
#define IMU_AXIS_Y_SRC 1 // vehicle left    <- sensor Y
#define IMU_AXIS_Y_SIGN +1.0f
#define IMU_AXIS_Z_SRC 2 // vehicle up      <- sensor Z
#define IMU_AXIS_Z_SIGN +1.0f

// ----------------------------------------------------------------------------
// --- GNSS (u-blox) ---
// ----------------------------------------------------------------------------

// No need for greater than 115200; higher can reduce PVT rate.
#define GNSS_BAUD 115200
#define GNSS_NAV_RATE_HZ 20   // 20 is max for 2 enabled constellations
#define GNSS_SV_MINELEV_DEG 5 // ignore SVs below this angle (anti-multipath)
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

// GNSS UART wiring - which ESP32 GPIOs you routed the receiver's TX/RX to.
#define GNSS_RX_PIN 16 // change to match your board/wiring
#define GNSS_TX_PIN 17 // change to match your board/wiring

// --- GNSS Constellation Toggles ---
// Enable only the constellations your module supports and your region benefits
// from. Enabling too many can reduce the PVT rate.
// For 20Hz PVT rate, enable up to 2 constellaations.
// For 25Hz PVT rate, enable only 1 constellation.
// For North American use you should always include GPS.
// Reference: https://app.qzss.go.jp/GNSSView/gnssview.html
#define GNSS_CONSTELLATIONS                                                    \
  {                                                                            \
      {"GPS", SFE_UBLOX_GNSS_ID_GPS, true},                                    \
      {"Galileo", SFE_UBLOX_GNSS_ID_GALILEO, true},                            \
      {"GLONASS", SFE_UBLOX_GNSS_ID_GLONASS, false},                           \
      {"BeiDou", SFE_UBLOX_GNSS_ID_BEIDOU, false},                             \
      {"QZSS", SFE_UBLOX_GNSS_ID_QZSS, false},                                 \
      {"SBAS", SFE_UBLOX_GNSS_ID_SBAS, false},                                 \
  }

// ----------------------------------------------------------------------------
// --- BLE ---
// ----------------------------------------------------------------------------

// BLE Transmit Power
// Select one of the following levels by assigning it to BLE_TX_POWER.
// Lower power reduces potential RF interference with the GNSS module.
// The receiver will usually be close, so high power is not really needed.
// If you have connection drop issues, try increasing the power level.
//   ESP_PWR_LVL_N12  =  -12 dBm (minimum power)
//   ESP_PWR_LVL_N9   =   -9 dBm
//   ESP_PWR_LVL_N6   =   -6 dBm
//   ESP_PWR_LVL_N3   =   -3 dBm
//   ESP_PWR_LVL_N0   =    0 dBm
//   ESP_PWR_LVL_P3   =   +3 dBm (default)
//   ESP_PWR_LVL_P6   =   +6 dBm
//   ESP_PWR_LVL_P9   =   +9 dBm (maximum power)
#define BLE_TX_POWER ESP_PWR_LVL_N12

#define BLE_READVERTISE_DELAY_MS 500 // delay before re-advertising

// How long after a client connects before bleIsConnected() reports true.
// Gives the MTU negotiation a moment to finish so the first notify isn't
// sent against the default 23-byte MTU and chunked.
#define BLE_CONNECT_SETTLE_MS 100

// ----------------------------------------------------------------------------
// --- LED (onboard status LED) ---
// ----------------------------------------------------------------------------

#define LED_ONBOARD_PIN 2 // onboard status LED; change to match your board
#define LED_BLINK_INTERVAL_MS 1000 // blink rate while disconnected

// ----------------------------------------------------------------------------
// --- Logging ---
// ----------------------------------------------------------------------------

// Master switch for all Serial diagnostic output.
// Logging OFF reduced loop latency.
// 1 = normal verbose output
// 0 = silent
#define LOG_ENABLED 1

#define LOG_STATS_INTERVAL_MS 1000 // serial stats reporting interval

// ----------------------------------------------------------------------------
// --- Protocol ---
// ----------------------------------------------------------------------------

// Which telemetry protocol this build emits. Values are the PROTO_* ids in
// g_protocol.h; g_protocol_active.h resolves the choice and is the only place
// that has to know about a new protocol.
//
// Compile-time by design: the unselected protocols are not linked, so they
// cost no flash, and there is no persistence or switching UI to build. The
// trade is that changing protocol needs a reflash. See
// docs/multiprotocol-design.md section 8.1.
//
// The protocol's own constants - identity strings, UUIDs, and the asserts that
// validate them - live in g_proto_<name>.h, not here. See section 7.
#define TELEMETRY_PROTOCOL PROTO_RACEBOX

// ============================================================================
// ============================================================================
// SECTION 2: SUPPORTING CONSTANTS
// These are protocol requirements or values derived from other constants.
// Changing them without also changing the matching part of the system (or the
// RaceBox protocol) will break the firmware.
// ============================================================================
// ============================================================================

// ----------------------------------------------------------------------------
// --- IMU (MPU-6050) ---
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// --- Battery ---
// ----------------------------------------------------------------------------

// No battery circuit on this build. The RaceBox protocol still carries a
// battery byte, so we report a constant full charge.
#define BATTERY_REPORT_PERCENT 100

// This build has no battery gauge: the shared telemetry module omits the
// battery segment from the serial stats line, and g_battery is a constant
// stub.
#define BATTERY_HAS_GAUGE 0

// ----------------------------------------------------------------------------
// --- Protocol ---
// ----------------------------------------------------------------------------
//
// The protocol's own constants - identity strings, service and characteristic
// UUIDs, and the asserts validating them - live with the protocol, in
// g_proto_<name>.h, NOT here. Two reasons:
//
//   1. They were byte-identical across all three variants but UNCHECKED,
//      because config.h cannot join check_common.sh's common set (DEVICE_ID,
//      pins and IMU ranges legitimately differ). Nothing prevented them
//      drifting and silently breaking app compatibility.
//   2. Validation that fires unconditionally here would force a build using a
//      different protocol to keep dead RaceBox constants alive just to satisfy
//      it.
//
// See docs/multiprotocol-design.md section 7.

// ============================================================================
// --- COMPILE-TIME VALIDATION ---
// ============================================================================

// Enforce device ID format: a 10-digit string with first digit 0-3.
//
// These rules are RaceBox APP constraints, so by rights they belong with the
// protocol. They stay here because DEVICE_ID itself is per-variant config, and
// g_proto_racebox.h must not include config.h - that dependency-freedom is what
// makes the encoder host-testable. Validating the value where the value lives
// is the lesser compromise.
// DEVICE_ID is a string so leading zeros survive; these constexpr helpers let
// us validate that string at compile time (C++ has no compile-time regex).
namespace device_id {
// Length of a C-string literal, counted at compile time.
constexpr int length(const char *s) { return *s ? 1 + length(s + 1) : 0; }
// True only if every character is a digit 0-9.
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

// Enforce valid, distinct ESP32 GPIO numbers for the three assigned pins.
static_assert(GNSS_RX_PIN >= 0 && GNSS_RX_PIN <= 39 && GNSS_TX_PIN >= 0 &&
                  GNSS_TX_PIN <= 39 && LED_ONBOARD_PIN >= 0 &&
                  LED_ONBOARD_PIN <= 39,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and LED_ONBOARD_PIN must be "
              "valid ESP32 GPIO numbers (0-39).");
static_assert(GNSS_RX_PIN != GNSS_TX_PIN && GNSS_RX_PIN != LED_ONBOARD_PIN &&
                  GNSS_TX_PIN != LED_ONBOARD_PIN,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and LED_ONBOARD_PIN must all "
              "be different pins.");

// Enforce a GNSS_BAUD the firmware actually knows how to detect and switch
// to. Keep this list in sync with the fallback baud rates in
// connectAndConfigureBaud() (g_gnss.cpp) - an unswept value here risks
// telling the module to save an unrecoverable baud rate to flash.
static_assert(GNSS_BAUD == 9600 || GNSS_BAUD == 38400 || GNSS_BAUD == 57600 ||
                  GNSS_BAUD == 115200 || GNSS_BAUD == 230400 ||
                  GNSS_BAUD == 460800,
              "ERROR: GNSS_BAUD must be one of the baud rates "
              "connectAndConfigureBaud() knows how to detect/switch between "
              "(9600, 38400, 57600, 115200, 230400, 460800).");

// Enforce navigation rate limit
static_assert(GNSS_NAV_RATE_HZ > 0 && GNSS_NAV_RATE_HZ <= 25,
              "ERROR: GNSS_NAV_RATE_HZ must be between 1 and 25.");

// Enforce a sane satellite elevation mask (a real angle above the horizon)
static_assert(GNSS_SV_MINELEV_DEG >= 0 && GNSS_SV_MINELEV_DEG <= 90,
              "ERROR: GNSS_SV_MINELEV_DEG must be between 0 and 90 degrees.");

// Enforce a GNSS epoch interval at least as long as the IMU sample interval.
// Otherwise a transmit window could contain zero fresh samples, silently
// degrading ImuAxis's transient peak tracking to a plain EMA with no warning.
// Checked here rather than in g_imu_tuning.h because GNSS_NAV_RATE_HZ is a
// per-board setting, and that file must not depend on one.
//
// The relationship used to be expressed against IMU_TRANSMIT_INTERVAL_MS, back
// when decimation ran on its own timer. It is now driven directly by the GNSS
// epoch (see imuLatchForEpoch()), so the real constraint is between the nav
// rate and the sample rate - which is what this checks. At 20Hz nav and a 10ms
// sample interval that is 5 samples per window.
static_assert((1000 / GNSS_NAV_RATE_HZ) >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: the GNSS epoch interval (1000 / GNSS_NAV_RATE_HZ) must "
              "be >= IMU_SAMPLE_INTERVAL_MS, so each transmitted sample's "
              "window contains at least one fresh IMU reading for ImuAxis's "
              "transient peak tracking to work.");

// Enforce positive timing intervals (a zero or negative value here would
// either fire every loop() or, once implicitly converted to the unsigned
// long millis() uses, wrap around to a value so large the action would
// effectively never fire).
static_assert(BLE_READVERTISE_DELAY_MS > 0,
              "ERROR: BLE_READVERTISE_DELAY_MS must be greater than 0.");
static_assert(BLE_CONNECT_SETTLE_MS > 0,
              "ERROR: BLE_CONNECT_SETTLE_MS must be greater than 0.");
static_assert(LED_BLINK_INTERVAL_MS > 0,
              "ERROR: LED_BLINK_INTERVAL_MS must be greater than 0.");
static_assert(LOG_STATS_INTERVAL_MS > 0,
              "ERROR: LOG_STATS_INTERVAL_MS must be greater than 0.");

// Logging feature flag: strictly 0 or 1.
static_assert(LOG_ENABLED == 0 || LOG_ENABLED == 1,
              "ERROR: LOG_ENABLED must be 0 or 1.");

static_assert(IMU_ENABLED == 0 || IMU_ENABLED == 1,
              "ERROR: IMU_ENABLED must be 0 or 1.");
static_assert(IMU_I2C_ADDRESS == 0x68 || IMU_I2C_ADDRESS == 0x69,
              "ERROR: IMU_I2C_ADDRESS must be 0x68 (AD0 low) or 0x69 (AD0 "
              "high) - the only two addresses an MPU-6050 can have.");

// Enforce each axis sign is a true sign, not a scale factor
static_assert(IMU_AXIS_X_SIGN == 1.0f || IMU_AXIS_X_SIGN == -1.0f,
              "ERROR: IMU_AXIS_X_SIGN must be exactly +1.0f or -1.0f.");
static_assert(IMU_AXIS_Y_SIGN == 1.0f || IMU_AXIS_Y_SIGN == -1.0f,
              "ERROR: IMU_AXIS_Y_SIGN must be exactly +1.0f or -1.0f.");
static_assert(IMU_AXIS_Z_SIGN == 1.0f || IMU_AXIS_Z_SIGN == -1.0f,
              "ERROR: IMU_AXIS_Z_SIGN must be exactly +1.0f or -1.0f.");

// Enforce each source index names a real sensor axis.
static_assert(IMU_AXIS_X_SRC >= 0 && IMU_AXIS_X_SRC <= 2,
              "ERROR: IMU_AXIS_X_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");
static_assert(IMU_AXIS_Y_SRC >= 0 && IMU_AXIS_Y_SRC <= 2,
              "ERROR: IMU_AXIS_Y_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");
static_assert(IMU_AXIS_Z_SRC >= 0 && IMU_AXIS_Z_SRC <= 2,
              "ERROR: IMU_AXIS_Z_SRC must be 0 (sensor X), 1 (Y) or 2 (Z).");

// Permutation parity: +1 if even, -1 if odd, 0 if any two _SRC collide.
// Integer arithmetic only, so it is usable in a static_assert.
#define IMU_AXIS_PARITY                                                        \
  ((((IMU_AXIS_Y_SRC) - (IMU_AXIS_X_SRC)) *                                    \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_X_SRC)) *                                    \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_Y_SRC))) /                                   \
   2)

// Catch a duplicated source index with a message that names the real problem,
// rather than letting it fall through to the mirror check below as a 0.
static_assert(IMU_AXIS_PARITY != 0,
              "ERROR: IMU_AXIS_X/Y/Z_SRC must be a permutation - each of "
              "sensor 0, 1 and 2 used exactly once. Two vehicle axes are "
              "currently fed by the same sensor axis.");

// The determinant check, and the reason this scheme is worth having.
//
// The target output frame is right-handed, so any valid mounting map is a
// proper rotation: determinant +1. Determinant = permutation parity x the
// product of the signs, so an even permutation needs an even number of sign
// flips and an odd permutation an odd number. Anything else is a MIRROR - an
// orientation no physical mounting can produce.
//
// This matters because a mirror fails silently and slowly. The data still
// looks entirely plausible; whichever axes are mis-signed are simply wrong,
// for accel and gyro alike, since remapAxes() applies the same matrix to both.
// The nRF52840 variant's config carried exactly that bug for months - X had
// been flipped without Y - and it survived a drive test and an app-side
// investigation before the arithmetic caught it. This variant is MORE exposed
// to that class of error, not less: its sensor is a separate module you
// oriented by hand rather than one fixed to a known board.
static_assert(IMU_AXIS_PARITY * IMU_AXIS_X_SIGN * IMU_AXIS_Y_SIGN *
                      IMU_AXIS_Z_SIGN ==
                  1.0f,
              "ERROR: axis map is a mirror, not a rotation (determinant -1). "
              "An even permutation needs an even number of sign flips, an odd "
              "permutation an odd number. See the order table in the Axis "
              "orientation section above.");

// Enforce a valid reported battery percentage (transmitted as a raw byte)
static_assert(BATTERY_REPORT_PERCENT >= 0 && BATTERY_REPORT_PERCENT <= 100,
              "ERROR: BATTERY_REPORT_PERCENT must be between 0 and 100.");

// Battery-gauge feature flag: strictly 0 or 1.
static_assert(BATTERY_HAS_GAUGE == 0 || BATTERY_HAS_GAUGE == 1,
              "ERROR: BATTERY_HAS_GAUGE must be 0 or 1.");
