#pragma once

// ============================================================================
// --- DEVICE IDENTITY ---
// ============================================================================

// Change DEVICE_ID to personalize your device. Must be 10 digits and less
// than 4000000000 — the RaceBox app will not connect to IDs of 4000000000
// or higher.
#define DEVICE_ID         2064591255UL
#define FIRMWARE_VERSION  "3.3"
#define HARDWARE_VERSION  "1"
#define MANUFACTURER      "RaceBox"
#define MODEL             "RaceBox Mini"

// ============================================================================
// --- HARDWARE PINS ---
// ============================================================================

#define GPS_RX_PIN       16
#define GPS_TX_PIN       17
#define ONBOARD_LED_PIN   2

// ============================================================================
// --- GPS / GNSS SETTINGS ---
// ============================================================================

#define GPS_BAUD             115200
#define FACTORY_GPS_BAUD       9600
#define MAX_NAVIGATION_RATE      25   // Hz — maximum supported by RaceBox Mini protocol
#define GPS_DYNAMIC_MODEL   DYN_MODEL_AUTOMOTIVE

// --- GNSS Constellation Toggles ---
// Enable only the constellations your module supports and your region benefits from.
// Enabling too many can reduce the update rate below 25Hz on some modules.
// Reference: https://app.qzss.go.jp/GNSSView/gnssview.html
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS
// #define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

// ============================================================================
// --- IMU SETTINGS ---
// ============================================================================

#define ACCEL_SAMPLE_INTERVAL_MS  10              // 10ms = 100Hz sample rate
#define ACCEL_ALPHA               0.8f            // EMA smoothing: 1.0 = raw, 0.5 = moderate
#define GYRO_ALPHA                0.8f            // EMA smoothing: 1.0 = raw, 0.5 = moderate
#define ACCEL_RANGE               MPU6050_RANGE_8_G
#define GYRO_RANGE                MPU6050_RANGE_500_DEG
#define FILTER_BANDWIDTH          MPU6050_BAND_21_HZ

// ============================================================================
// --- BLE SETTINGS ---
// ============================================================================

#define BLE_MTU_SIZE             128   // Bytes — must be >= 91 to carry an 88-byte notify
#define BLE_READVERTISE_DELAY_MS 500   // ms delay before re-advertising after disconnect
#define LED_BLINK_INTERVAL_MS    500   // ms — LED blink rate while waiting for a BLE connection

// ============================================================================
// --- TIMING & REPORTING ---
// ============================================================================

#define GPS_RATE_REPORT_INTERVAL_MS 5000 // ms — serial stats reporting interval

// ============================================================================
// --- PROTOCOL CONSTANTS ---
// These match the RaceBox BLE protocol and should not be changed.
// ============================================================================

#define RACEBOX_SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BATTERY_REPORT_PERCENT          100   // No battery circuit — always report 100%

// ============================================================================
// --- COMPILE-TIME VALIDATION ---
// ============================================================================

// Enforce device ID range
static_assert(DEVICE_ID >= 1000000000UL && DEVICE_ID <= 3999999999UL,
    "ERROR: DEVICE_ID must be a 10-digit number between 1000000000 and 3999999999.");

// Enforce sane EMA alpha range
static_assert(ACCEL_ALPHA > 0.0f && ACCEL_ALPHA <= 1.0f,
    "ERROR: ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(GYRO_ALPHA > 0.0f && GYRO_ALPHA <= 1.0f,
    "ERROR: GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce navigation rate limit
static_assert(MAX_NAVIGATION_RATE > 0 && MAX_NAVIGATION_RATE <= 25,
    "ERROR: MAX_NAVIGATION_RATE must be between 1 and 25 Hz.");
