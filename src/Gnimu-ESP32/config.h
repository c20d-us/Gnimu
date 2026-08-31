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
// _MPS2, _RADPS, _DEG, _BYTES, _PERCENT, _PIN).

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

#define IMU_SAMPLE_INTERVAL_MS 10 // 10 == 100Hz sample rate

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
// library never calls it, so nothing raises it on its own. g_imu.cpp applies
// this AFTER myIMU.begin(). begin() brings the bus up and would overwrite any
// earlier setting.
#define IMU_I2C_CLOCK_HZ 400000

// ImuAxis smoothing rates and transient thresholds.
// The deviation (in raw sensor units - m/s^2 for accel, rad/s for gyro) a
// window's peak must exceed before it gets blended into the transmitted value
// instead of the plain EMA baseline.
#define IMU_ACCEL_ALPHA 0.2f // EMA smoothing: 1.0 = raw, 0.1 = heavy
#define IMU_GYRO_ALPHA 0.2f  // EMA smoothing: 1.0 = raw, 0.1 = heavy
#define IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2 2.0f // ~0.2g
#define IMU_GYRO_TRANSIENT_THRESHOLD_RADPS 0.5f // ~28.6 deg/s

// Per-axis zero-point offsets (raw sensor frame)
//
// Subtracted from each raw axis inside g_imu's readImuRaw(), correcting the
// chip's intrinsic bias before any smoothing or protocol conversion. Units
// match the Adafruit MPU6050 driver's native output: m/s^2 for accel, rad/s
// for gyro. The accel Z offset is a bias ONLY. Gravity is not included (the
// calibration sketch removes 9.80665 m/s^2 before reporting), so a level board
// still reads ~1g on Z after correction.
#define IMU_ACCEL_OFFSET_X_MPS2 +0.367034f
#define IMU_ACCEL_OFFSET_Y_MPS2 +0.055432f
#define IMU_ACCEL_OFFSET_Z_MPS2 -0.676274f
#define IMU_GYRO_OFFSET_X_RADPS -0.080341f
#define IMU_GYRO_OFFSET_Y_RADPS +0.003458f
#define IMU_GYRO_OFFSET_Z_RADPS -0.009113f

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
// read the 1 Hz serial milliG line (LOG must be enabled):
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

#define BLE_MTU_BYTES 128            // must be >= 91 to carry an 88-byte notify
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

// Decimate the IMU stream down to the transmission rate. Derived from
// GNSS_NAV_RATE_HZ so the two can't drift out of sync.
#define IMU_TRANSMIT_INTERVAL_MS (1000 / GNSS_NAV_RATE_HZ)

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
// --- Protocol (RaceBox BLE protocol) ---
// These match the RaceBox BLE protocol and should not be changed.
// ----------------------------------------------------------------------------

#define RACEBOX_MODEL "RaceBox Mini"   // Compatibility requirement
#define RACEBOX_MANUFACTURER "RaceBox" // Compatibility requirement
#define RACEBOX_HARDWARE_VERSION "1"   // Compatibility requirement
#define RACEBOX_FIRMWARE_VERSION "3.3" // Compatibility requirement
#define RACEBOX_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ============================================================================
// --- COMPILE-TIME VALIDATION ---
// ============================================================================

// Enforce device ID format: a 10-digit string with first digit 0-3.
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

// Validate the three RaceBox protocol UUIDs are well-formed 8-4-4-4-12 hex
// strings, the same way DEVICE_ID's format is checked above.
namespace uuid_format {
constexpr bool isHexDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}
// Hyphens are required at positions 8, 13, 18, and 23; every other position
// (of the required 36 total) must be a hex digit.
constexpr bool isValid(const char *s, int i = 0) {
  return i == 36 ? s[i] == '\0'
         : (i == 8 || i == 13 || i == 18 || i == 23)
             ? (s[i] == '-' && isValid(s, i + 1))
             : (isHexDigit(s[i]) && isValid(s, i + 1));
}
} // namespace uuid_format

static_assert(uuid_format::isValid(RACEBOX_SERVICE_UUID),
              "ERROR: RACEBOX_SERVICE_UUID must be a standard 8-4-4-4-12 hex "
              "UUID string.");
static_assert(uuid_format::isValid(RACEBOX_CHARACTERISTIC_TX_UUID),
              "ERROR: RACEBOX_CHARACTERISTIC_TX_UUID must be a standard "
              "8-4-4-4-12 hex UUID string.");
static_assert(uuid_format::isValid(RACEBOX_CHARACTERISTIC_RX_UUID),
              "ERROR: RACEBOX_CHARACTERISTIC_RX_UUID must be a standard "
              "8-4-4-4-12 hex UUID string.");

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

// Enforce sane EMA alpha range
static_assert(IMU_ACCEL_ALPHA > 0.0f && IMU_ACCEL_ALPHA <= 1.0f,
              "ERROR: IMU_ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(IMU_GYRO_ALPHA > 0.0f && IMU_GYRO_ALPHA <= 1.0f,
              "ERROR: IMU_GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce positive transient thresholds (a zero/negative threshold would
// disable transient blending entirely; see ImuAxis::read()).
static_assert(
    IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2 > 0.0f,
    "ERROR: IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2 must be greater than 0.");
static_assert(
    IMU_GYRO_TRANSIENT_THRESHOLD_RADPS > 0.0f,
    "ERROR: IMU_GYRO_TRANSIENT_THRESHOLD_RADPS must be greater than 0.");

// Enforce a positive sample interval, and a transmit interval that's at
// least as long as it. Otherwise a transmit window could contain zero fresh
// samples, silently degrading ImuAxis's transient peak tracking to a plain
// EMA with no warning.
static_assert(IMU_SAMPLE_INTERVAL_MS > 0,
              "ERROR: IMU_SAMPLE_INTERVAL_MS must be greater than 0.");
static_assert(IMU_TRANSMIT_INTERVAL_MS >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: IMU_TRANSMIT_INTERVAL_MS must be >= "
              "IMU_SAMPLE_INTERVAL_MS so each transmit window contains at "
              "least one fast sample for ImuAxis's transient peak tracking "
              "to work.");

// Enforce MTU large enough for an 88-byte notify plus the 3-byte ATT header,
// and no larger than the maximum ATT MTU defined by the BLE spec.
static_assert(BLE_MTU_BYTES >= 91,
              "ERROR: BLE_MTU_BYTES must be >= 91 to carry an 88-byte notify.");
static_assert(BLE_MTU_BYTES <= 517,
              "ERROR: BLE_MTU_BYTES must be <= 517, the maximum ATT MTU "
              "defined by the Bluetooth Low Energy spec.");

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
