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

// ============================================================================
// --- DEVICE IDENTITY ---
// ============================================================================

// Change DEVICE_ID to personalize your device.
// It is a STRING of exactly 10 digits. Quote it, so that leading zeros are kept
// (e.g. "0123456789"). Do NOT use a bare number: a leading zero would be read
// as an octal literal and an unquoted ID loses its leading zeros.
// First digit must be 0-3, so the value stays below 4000000000. The RaceBox
// app will not connect to IDs of 4000000000 or higher. See compile-time
// validation at the bottom of this file.
#define DEVICE_ID "0008675309" // Customize for uniqueness
#define FIRMWARE_VERSION "3.3" // Compatibility requirement - don't change
#define HARDWARE_VERSION "1"   // Compatibility requirement - don't change
#define MANUFACTURER "RaceBox" // Compatibility requirement - don't change
#define MODEL "RaceBox Mini"   // Compatibility requirement - don't change

// ============================================================================
// --- HARDWARE PINS ---
// ============================================================================

#define GNSS_RX_PIN 16    // Change if your specific ESP32 board differs
#define GNSS_TX_PIN 17    // Change if your specific ESP32 board differs
#define ONBOARD_LED_PIN 2 // Change if your specific ESP32 board differs

// ============================================================================
// --- GNSS SETTINGS ---
// ============================================================================

#define SV_MINELEV 15    // in deg; ignore SVs below this angle (anti-multipath)
#define GNSS_BAUD 460800 // in bps; 9600, 38400, 57600, 115200, 230400, 460800
#define MAX_NAVIGATION_RATE 25 // in Hz; max supported by RaceBox Mini protocol
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

// --- GNSS Constellation Toggles ---
// Enable only the constellations your module supports and your region benefits
// from. Enabling too many can reduce the update rate below 25Hz.
// For North American use you should include GPS and Galileo.
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

// ============================================================================
// --- IMU SETTINGS ---
// ============================================================================

#define IMU_SAMPLE_INTERVAL_MS 10           // in ms; 10 == 100Hz sample rate
#define ACCEL_RANGE MPU6050_RANGE_4_G       // 4g range is sufficient for auto-x
#define GYRO_RANGE MPU6050_RANGE_500_DEG    // 500deg/s is sufficient for auto-x
#define FILTER_BANDWIDTH MPU6050_BAND_21_HZ // built-in low-pass filter setting

// Decimate the IMU stream down to the transmission rate.
// Derived from MAX_NAVIGATION_RATE so the two can't drift out of sync.
#define IMU_TRANSMIT_INTERVAL_MS (1000 / MAX_NAVIGATION_RATE)

// ImuAxis smoothing rates and transient thresholds.
// The deviation (in raw sensor units - m/s^2 for accel, rad/s for gyro) a
// window's peak must exceed before it gets blended into the transmitted value
// instead of the plain EMA baseline. These are PLACEHOLDER starting points only
// - the right value depends on this specific car's vibration floor
// (engine/tire/kerb noise) versus genuine events, and must be tuned empirically
// against real track data per axis.
#define ACCEL_ALPHA 0.2f               // EMA smoothing: 1.0 = raw, 0.1 = heavy
#define GYRO_ALPHA 0.2f                // EMA smoothing: 1.0 = raw, 0.1 = heavy
#define ACCEL_TRANSIENT_THRESHOLD 2.0f // in m/s^2 (~0.2g)
#define GYRO_TRANSIENT_THRESHOLD 0.5f  // in rad/s (~28.6 deg/s)

// ============================================================================
// --- BLE SETTINGS ---
// ============================================================================

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

#define BLE_MTU_SIZE 128 // in bytes; must be >= 91 to carry an 88-byte notify
#define BLE_READVERTISE_DELAY_MS 500 // in ms; delay before BLE re-advertising
#define LED_BLINK_INTERVAL_MS 1000   // in ms; LED blink rate when disconnected

// ============================================================================
// --- SERIAL REPORTING TIMING ---
// ============================================================================

#define STATS_REPORT_INTERVAL_MS 1000 // in ms; serial stats reporting interval

// ============================================================================
// --- PROTOCOL CONSTANTS ---
// These match the RaceBox BLE protocol and should not be changed.
// ============================================================================

#define RACEBOX_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BATTERY_REPORT_PERCENT 100 // No battery circuit - always report 100%

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
                  GNSS_TX_PIN <= 39 && ONBOARD_LED_PIN >= 0 &&
                  ONBOARD_LED_PIN <= 39,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and ONBOARD_LED_PIN must be "
              "valid ESP32 GPIO numbers (0-39).");
static_assert(GNSS_RX_PIN != GNSS_TX_PIN && GNSS_RX_PIN != ONBOARD_LED_PIN &&
                  GNSS_TX_PIN != ONBOARD_LED_PIN,
              "ERROR: GNSS_RX_PIN, GNSS_TX_PIN, and ONBOARD_LED_PIN must all "
              "be different pins.");

// Enforce a GNSS_BAUD the firmware actually knows how to detect and switch
// to. Keep this list in sync with the fallback baud rates in
// connectAndConfigureBaud() (gc_gnss.cpp) - an unswept value here risks
// telling the module to save an unrecoverable baud rate to flash.
static_assert(GNSS_BAUD == 9600 || GNSS_BAUD == 38400 || GNSS_BAUD == 57600 ||
                  GNSS_BAUD == 115200 || GNSS_BAUD == 230400 ||
                  GNSS_BAUD == 460800,
              "ERROR: GNSS_BAUD must be one of the baud rates "
              "connectAndConfigureBaud() knows how to detect/switch between "
              "(9600, 38400, 57600, 115200, 230400, 460800).");

// Enforce navigation rate limit
static_assert(MAX_NAVIGATION_RATE > 0 && MAX_NAVIGATION_RATE <= 25,
              "ERROR: MAX_NAVIGATION_RATE must be between 1 and 25 Hz.");

// Enforce a sane satellite elevation mask (a real angle above the horizon)
static_assert(SV_MINELEV >= 0 && SV_MINELEV <= 90,
              "ERROR: SV_MINELEV must be between 0 and 90 degrees.");

// Enforce sane EMA alpha range
static_assert(ACCEL_ALPHA > 0.0f && ACCEL_ALPHA <= 1.0f,
              "ERROR: ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(GYRO_ALPHA > 0.0f && GYRO_ALPHA <= 1.0f,
              "ERROR: GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce positive transient thresholds (a zero/negative threshold would
// disable transient blending entirely; see ImuAxis::read()).
static_assert(ACCEL_TRANSIENT_THRESHOLD > 0.0f,
              "ERROR: ACCEL_TRANSIENT_THRESHOLD must be greater than 0.");
static_assert(GYRO_TRANSIENT_THRESHOLD > 0.0f,
              "ERROR: GYRO_TRANSIENT_THRESHOLD must be greater than 0.");

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
static_assert(BLE_MTU_SIZE >= 91,
              "ERROR: BLE_MTU_SIZE must be >= 91 to carry an 88-byte notify.");
static_assert(BLE_MTU_SIZE <= 517,
              "ERROR: BLE_MTU_SIZE must be <= 517, the maximum ATT MTU "
              "defined by the Bluetooth Low Energy spec.");

// Enforce positive timing intervals (a zero or negative value here would
// either fire every loop() or, once implicitly converted to the unsigned
// long millis() uses, wrap around to a value so large the action would
// effectively never fire).
static_assert(BLE_READVERTISE_DELAY_MS > 0,
              "ERROR: BLE_READVERTISE_DELAY_MS must be greater than 0.");
static_assert(LED_BLINK_INTERVAL_MS > 0,
              "ERROR: LED_BLINK_INTERVAL_MS must be greater than 0.");
static_assert(STATS_REPORT_INTERVAL_MS > 0,
              "ERROR: STATS_REPORT_INTERVAL_MS must be greater than 0.");

// Enforce a valid reported battery percentage (transmitted as a raw byte)
static_assert(BATTERY_REPORT_PERCENT >= 0 && BATTERY_REPORT_PERCENT <= 100,
              "ERROR: BATTERY_REPORT_PERCENT must be between 0 and 100.");
