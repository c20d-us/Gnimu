#pragma once

// ============================================================================
// --- DEVICE IDENTITY ---
// ============================================================================

// Change DEVICE_ID to personalize your device.
// It is a STRING of exactly 10 digits — quote it, so leading zeros are kept
// (e.g. "0123456789"). Do NOT use a bare number: a leading zero would be read
// as an octal literal and an unquoted ID loses its leading zeros.
// First digit must be 0-3, so the value stays below 4000000000 — the RaceBox
// app will not connect to IDs of 4000000000 or higher. See compile-time
// validation at the bottom of this file.
#define DEVICE_ID "3608675309"
#define FIRMWARE_VERSION "3.3"
#define HARDWARE_VERSION "1"
#define MANUFACTURER "RaceBox"
#define MODEL "RaceBox Mini"

// ============================================================================
// --- HARDWARE PINS ---
// ============================================================================

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define ONBOARD_LED_PIN 2

// ============================================================================
// --- GPS / GNSS SETTINGS ---
// ============================================================================

#define GPS_BAUD 115200
#define FACTORY_GPS_BAUD 9600
#define MAX_NAVIGATION_RATE 25 // Hz — max supported by RaceBox Mini protocol
#define GPS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

// --- GNSS Constellation Toggles ---
// Enable only the constellations your module supports and your region benefits
// from. Enabling too many can reduce the update rate below 25Hz on some
// modules. Reference: https://app.qzss.go.jp/GNSSView/gnssview.html

#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS  // Not supported by M10 hardware
// #define ENABLE_GNSS_BEIDOU   // Not supported by M10 hardware
// #define ENABLE_GNSS_SBAS     // Not supported by M10 hardware
// #define ENABLE_GNSS_QZSS     // Not supported by M10 hardware

// ============================================================================
// --- IMU SETTINGS ---
// ============================================================================

// Comment out to disable gyro bias calibration at startup.
// When enabled, the device must be stationary during the first second of boot.
#define GYRO_CALIBRATION_ENABLED

// Number of calibration samples to average (10ms each) - 100 seems good
#define GYRO_CALIBRATION_SAMPLES 100

#define ACCEL_SAMPLE_INTERVAL_MS 10 // 10ms = 100Hz sample rate
#define ACCEL_ALPHA 0.8f            // EMA smoothing: 1.0 = raw, 0.5 = moderate
#define GYRO_ALPHA 0.8f             // EMA smoothing: 1.0 = raw, 0.5 = moderate
#define ACCEL_RANGE MPU6050_RANGE_8_G
#define GYRO_RANGE MPU6050_RANGE_500_DEG
#define FILTER_BANDWIDTH MPU6050_BAND_21_HZ

// ============================================================================
// --- BLE SETTINGS ---
// ============================================================================

// Comment out to disable BLE (e.g. to test GPS without RF interference)
#define BLE_ENABLED

// BLE Transmit Power
// Select one of the following levels by assigning it to BLE_TX_POWER.
// Lower power reduces potential RF interference with the GNSS module.
// The receiver will usually be close, so high power is not really needed.
// If you have connection drops, try increasing the power level.
//   ESP_PWR_LVL_N12  =  -12 dBm  (lowest — ~125x less power than default)
//   ESP_PWR_LVL_N9   =   -9 dBm
//   ESP_PWR_LVL_N6   =   -6 dBm
//   ESP_PWR_LVL_N3   =   -3 dBm
//   ESP_PWR_LVL_N0   =    0 dBm
//   ESP_PWR_LVL_P3   =   +3 dBm
//   ESP_PWR_LVL_P6   =   +6 dBm
//   ESP_PWR_LVL_P9   =   +9 dBm  (default ESP32 power)
#define BLE_TX_POWER ESP_PWR_LVL_N12

#define BLE_MTU_SIZE 128 // Bytes — must be >= 91 to carry an 88-byte notify
#define BLE_READVERTISE_DELAY_MS 500 // ms - delay before re-advertising
#define LED_BLINK_INTERVAL_MS 1000   // ms - LED blink rate when disconnected

// ============================================================================
// --- TIMING & REPORTING ---
// ============================================================================

#define GPS_RATE_REPORT_INTERVAL_MS 1000 // ms — serial stats reporting interval

// ============================================================================
// --- PROTOCOL CONSTANTS ---
// These match the RaceBox BLE protocol and should not be changed.
// ============================================================================

#define RACEBOX_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BATTERY_REPORT_PERCENT 100 // No battery circuit — always report 100%

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

// Enforce sane EMA alpha range
static_assert(ACCEL_ALPHA > 0.0f && ACCEL_ALPHA <= 1.0f,
              "ERROR: ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(GYRO_ALPHA > 0.0f && GYRO_ALPHA <= 1.0f,
              "ERROR: GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce navigation rate limit
static_assert(MAX_NAVIGATION_RATE > 0 && MAX_NAVIGATION_RATE <= 25,
              "ERROR: MAX_NAVIGATION_RATE must be between 1 and 25 Hz.");
