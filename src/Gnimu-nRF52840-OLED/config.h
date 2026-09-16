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

// For the board's pin macros.
#include <Arduino.h>

// IMU sample rate, filtering, and trim values shared by all boards.
#include "g_imu_tuning.h"

// ----------------------------------------------------------------------------
// Section 1: Tunables
// ----------------------------------------------------------------------------

// Device identity

// Exactly 10 digits, quoted so leading zeros survive.
// First digit 0-3: the RaceBox app rejects IDs of 4000000000 or higher.
#define DEVICE_ID "1000000003"

// Build name shown in the boot banner.
#define GNIMU_VARIANT "nRF52840-OLED"

// IMU (onboard LSM6DS3TR-C)

// Set from the IDE board selection: the Sense boards have the IMU, the plain
// boards don't. With 0 the driver is a stub (no LSM6DS3 library needed) and
// IMU fields read zero. Replace the block with `#define IMU_ENABLED 0` or `1`
// to override.
#if defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) ||                              \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)
#define IMU_ENABLED 1
#else
#define IMU_ENABLED 0
#endif

#define IMU_ACCEL_RANGE_G 4    // +/- g: 2, 4, 8, 16
#define IMU_GYRO_RANGE_DPS 500 // deg/s: 125, 245, 500, 1000, 2000
#define IMU_ACCEL_ODR_HZ 104   // at least the 100Hz sample rate
#define IMU_GYRO_ODR_HZ 104    // at least the 100Hz sample rate

// Accel LPF1 cutoff as an ODR divider (LPF1_BW_SEL): 2 or 4.
// At 104Hz ODR, 4 gives 26Hz.
#define IMU_ACCEL_LPF1_ODR_DIV 4

#define IMU_ACCEL_LPF1_CUTOFF_HZ (IMU_ACCEL_ODR_HZ / IMU_ACCEL_LPF1_ODR_DIV)

// Axis orientation
//
// Maps sensor axes to the vehicle frame: X forward, Y left, Z up (ISO 8855).
// Each vehicle axis takes a sensor axis (0=X, 1=Y, 2=Z) and a sign.
//
// XIAO Sense sensor axes: +X toward the antenna end (away from USB-C), +Y
// toward the LED edge, +Z out of the SoC face.
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

// GNSS (HGLRC M100 Mini, u-blox M10)

// TPS63020 enable pin for the GNSS rail.
#define GNSS_EN_PIN D9

#define GNSS_BAUD 115200      // higher rates can lower the PVT rate
#define GNSS_NAV_RATE_HZ 20   // max 20 with 2 constellations
#define GNSS_SV_MINELEV_DEG 5 // elevation mask
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

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

// BLE (Bluefruit)

// TX power. Lower power reduces interference with GNSS.
// Valid levels: -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8.
#define BLE_TX_POWER_ADV_DBM -16  // advertising
#define BLE_TX_POWER_CONN_DBM -16 // connected

// Battery

// Low-voltage cutoff: below BATTERY_CUTOFF_V for the debounce period, the
// device enters System OFF until USB plug-in or a switch off -> on.
#define BATTERY_CUTOFF_V 3.35f
#define BATTERY_WARN_V 3.45f     // amber blink
#define BATTERY_CRITICAL_V 3.40f // red blink
#define BATTERY_CUTOFF_DEBOUNCE_MS 5000

// VBAT sampler: every poll interval, take BATTERY_SAMPLE_COUNT paced reads and
// keep the peak, which rejects load sag.
#define BATTERY_POLL_INTERVAL_MS 250   // time between runs
#define BATTERY_SAMPLE_COUNT 20        // reads per run
#define BATTERY_SAMPLE_SPACING_US 2500 // time between reads (~50ms run)

// Voltage shown as full while charging. Below 4.2V so charger and ADC tolerance
// can't keep it from being reached.
#define BATTERY_FULL_V 4.15f

// Select ~100mA charge current (default ~50mA). Comment out for the default.
#define BATTERY_FAST_CHARGE

// Smoothing of the reported voltage across runs (higher is faster). The cutoff
// uses the unsmoothed peak.
#define BATTERY_EMA_ALPHA 0.1f

// Resting voltage -> percent, {voltage, percent} pairs sorted high to low. A
// straight line between two points.
#define BATTERY_DISCHARGE_CURVE {{3.90f, 100}, {3.35f, 0}}

// Power

// Slide-switch sense: the switch's spare pole feeds a 510k/510k divider, ~0V
// when on and >= ~1.7V when off. A1 because A4 is the display's SDA on this
// board.
#define POWER_SWITCH_SENSE_PIN A1
#define POWER_SWITCH_OFF_THRESHOLD_MV 800 // above this = off

// Expected sense levels, for the margin check below. OFF_MV_MIN is at the
// 3.35V cutoff.
#define POWER_SWITCH_ON_MV 0         // on shorts the tap to ground
#define POWER_SWITCH_OFF_MV_MIN 1675 // 510k/510k at 3.35V

// powerSwitchOn() refresh interval.
#define POWER_SWITCH_POLL_INTERVAL_MS 50

// State

// 1: USB with the switch on enters CHARGE_ONLY (peripherals off); unplugging or
// switching off resets.
// 0: USB is ignored and the device runs normally while charging.
#define STATE_CHARGE_ONLY_ON_USB 0

// Idle timeout: with no subscribed BLE client and no USB for this long, enter
// DEEP_SLEEP. Recover with a switch off -> on or USB plug-in.
#define STATE_IDLE_TIMEOUT_MIN 240 // 4h

// A switch-off reading must persist this long before BATTERY_WAIT. Switch-on is
// immediate.
#define STATE_SWITCH_OFF_DEBOUNCE_MS 500

// LED (onboard RGB)

// 0: the LED is used only when no display is detected.
// 1: the LED always runs alongside the display.
// The charge LED is hardware-driven either way.
#define LED_ENABLED 0

#define LED_BLINK_INTERVAL_MS 1000    // normal blink
#define LED_BATTERY_WAIT_BLINK_MS 150 // BATTERY_WAIT blink

// Display (SSD1306 128x64 OLED)

// 0 compiles the display out and the LED fallback applies.
#define DISPLAY_ENABLED 1

// Full-screen render interval.
#define DISPLAY_REFRESH_INTERVAL_MS 1000

// Minimum time between slice pushes when epochs aren't arriving (fallback
// path). Spacing keeps the GNSS UART drained between pushes.
#define DISPLAY_SLICE_INTERVAL_MS 20

// Slices pushed right after each NAV-PVT, while the UART is idle. The
// static_asserts below keep this within the idle window and the refresh
// interval.
#define DISPLAY_SLICES_PER_EPOCH 2

// Time without an epoch before falling back to DISPLAY_SLICE_INTERVAL_MS.
#define DISPLAY_EPOCH_STALE_MS 250

// Burn-in pixel shift interval.
#define DISPLAY_SHIFT_INTERVAL_MS 300000 // 5 minutes

// Slice width in 8x8 tiles (8 bytes per tile).
// Smaller slices block for less time but take more pushes per frame.
#define DISPLAY_CHUNK_TILES_W 4

#define DISPLAY_CONTRAST 255 // 0-255

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

// SAADC (shared nRF52840 ADC)

// Applied by powerBegin() for VBAT and switch-sense reads.
#define SAADC_RESOLUTION_BITS 12
#define SAADC_REFERENCE_MV 3000.0f

// Acquisition time. The dividers are ~338k (VBAT) and ~255k (switch); the 3us
// default undershoots above ~40k, and 40us covers up to ~800k.
#define SAADC_TACQ_US 40

// IMU (onboard LSM6DS3TR-C)

// On Wire1. 0x6B is the part's other address; nothing answers there.
#define IMU_I2C_ADDRESS 0x6A // SA0 high on the XIAO Sense

// Wire1 clock. The core defaults to 100kHz, and each IMU sample is several I2C
// transactions, so 400kHz cuts loop blocking. Applied after myIMU.begin().
#define IMU_I2C_CLOCK_HZ 400000
#define IMU_POWER_PIN PIN_LSM6DS3TR_C_POWER

// GNSS (HGLRC M100 Mini, u-blox M10)

// Serial1: D6 TX -> GNSS RX, D7 RX <- GNSS TX.
#define GNSS_RX_PIN D7
#define GNSS_TX_PIN D6

// Battery

// 1: battery sensing is present (adds battery to the stats line).
#define BATTERY_HAS_GAUGE 1

// XIAO onboard VBAT divider: VBAT = ADC volts * (R_TOP + R_BOTTOM) / R_BOTTOM.
#define BATTERY_DIVIDER_R_TOP_OHM 1000000.0f
#define BATTERY_DIVIDER_R_BOTTOM_OHM 510000.0f
#define BATTERY_DIVIDER_RATIO                                                  \
  ((BATTERY_DIVIDER_R_TOP_OHM + BATTERY_DIVIDER_R_BOTTOM_OHM) /                \
   BATTERY_DIVIDER_R_BOTTOM_OHM)

// VBAT sense and divider enable (active-low). Uses the variant's macros when
// defined, otherwise Seeed's pin indices.
#ifdef PIN_VBAT
#define BATTERY_ADC_PIN PIN_VBAT
#else
#define BATTERY_ADC_PIN 32 // D32 -> P0.31 / AIN7
#endif
#ifdef VBAT_ENABLE
#define BATTERY_ADC_ENABLE_PIN VBAT_ENABLE
#else
#define BATTERY_ADC_ENABLE_PIN 14 // D14 -> P0.14
#endif

// Charge-current select (HICHG): low = ~100mA, high = ~50mA. See
// BATTERY_FAST_CHARGE.
#ifdef PIN_HICHG
#define BATTERY_CHARGE_CURRENT_PIN PIN_HICHG
#else
#define BATTERY_CHARGE_CURRENT_PIN 22 // D22 -> P0.13
#endif

// LED (onboard RGB, active-low)

#define LED_RED_PIN LED_RED
#define LED_GREEN_PIN LED_GREEN
#define LED_BLUE_PIN LED_BLUE

// Display (SSD1306 128x64 OLED, hardware I2C)

// 7-bit address; g_display shifts it for u8g2.
#define DISPLAY_I2C_ADDRESS 0x3C

// Panel size and the layout area inside the pixel-shift margin (right and
// bottom).
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_SHIFT_MAX 2
#define DISPLAY_LAYOUT_W (DISPLAY_WIDTH - DISPLAY_SHIFT_MAX)
#define DISPLAY_LAYOUT_H (DISPLAY_HEIGHT - DISPLAY_SHIFT_MAX)

// 8x8 px tiles: 16x8.
#define DISPLAY_TILES_W (DISPLAY_WIDTH / 8)
#define DISPLAY_TILES_H (DISPLAY_HEIGHT / 8)

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

static_assert(GNSS_RX_PIN != GNSS_TX_PIN,
              "ERROR: GNSS_RX_PIN and GNSS_TX_PIN must be different pins.");

// Fallback path: a full frame of spaced slices must finish within the refresh
// interval, or the display pushes continuously.
static_assert(((DISPLAY_TILES_W / DISPLAY_CHUNK_TILES_W) * DISPLAY_TILES_H) *
                      DISPLAY_SLICE_INTERVAL_MS <
                  DISPLAY_REFRESH_INTERVAL_MS,
              "ERROR: a full display push takes longer than the refresh "
              "interval - the display would push continuously. Raise "
              "DISPLAY_REFRESH_INTERVAL_MS, or raise DISPLAY_CHUNK_TILES_W, or "
              "lower DISPLAY_SLICE_INTERVAL_MS.");

// Epoch-locked display timing, derived from GNSS_NAV_RATE_HZ and GNSS_BAUD.

#define GNSS_EPOCH_PERIOD_MS (1000 / GNSS_NAV_RATE_HZ)

// NAV-PVT wire time: 100 bytes at 10 bits per byte.
#define GNSS_PVT_WIRE_MS ((100UL * 10UL * 1000UL) / GNSS_BAUD)

// Time per epoch when the UART is idle.
#define DISPLAY_CLEAR_AIR_MS (GNSS_EPOCH_PERIOD_MS - GNSS_PVT_WIRE_MS)

// Slice push time, scaled from the measured full-frame time (31ms at 400kHz,
// from tools/nRF52840-OLED/oled_bench). Re-measure if the panel or bus changes.
#define DISPLAY_FRAME_PUSH_MS 31
#define DISPLAY_BUFFER_BYTES ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8)
#define DISPLAY_SLICE_BYTES (DISPLAY_CHUNK_TILES_W * 8)
#define DISPLAY_SLICE_PUSH_MS                                                  \
  (((DISPLAY_FRAME_PUSH_MS * DISPLAY_SLICE_BYTES) + DISPLAY_BUFFER_BYTES -     \
    1) /                                                                       \
   DISPLAY_BUFFER_BYTES)

// The per-epoch burst must use under half the idle window.
static_assert((DISPLAY_SLICES_PER_EPOCH * DISPLAY_SLICE_PUSH_MS) * 2 <
                  DISPLAY_CLEAR_AIR_MS,
              "ERROR: DISPLAY_SLICES_PER_EPOCH pushes more than half the clear "
              "air between NAV-PVT messages. Lower it, lower "
              "DISPLAY_CHUNK_TILES_W, or lower GNSS_NAV_RATE_HZ.");

// Epochs per frame on the epoch-locked path.
#define DISPLAY_FRAME_EPOCHS                                                   \
  (((((DISPLAY_TILES_W / DISPLAY_CHUNK_TILES_W) * DISPLAY_TILES_H)) +          \
    DISPLAY_SLICES_PER_EPOCH - 1) /                                            \
   DISPLAY_SLICES_PER_EPOCH)

static_assert(DISPLAY_FRAME_EPOCHS * GNSS_EPOCH_PERIOD_MS <
                  DISPLAY_REFRESH_INTERVAL_MS,
              "ERROR: a phase-locked frame push takes longer than the refresh "
              "interval. Raise DISPLAY_SLICES_PER_EPOCH or "
              "DISPLAY_REFRESH_INTERVAL_MS, or raise DISPLAY_CHUNK_TILES_W.");

static_assert(GNSS_NAV_RATE_HZ > 0 && GNSS_NAV_RATE_HZ <= 25,
              "ERROR: GNSS_NAV_RATE_HZ must be between 1 and 25.");

static_assert(GNSS_SV_MINELEV_DEG >= 0 && GNSS_SV_MINELEV_DEG <= 90,
              "ERROR: GNSS_SV_MINELEV_DEG must be between 0 and 90.");

// Each epoch must span at least one IMU sample.
static_assert((1000 / GNSS_NAV_RATE_HZ) >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: the GNSS epoch interval (1000 / GNSS_NAV_RATE_HZ) must "
              "be >= IMU_SAMPLE_INTERVAL_MS, so each transmitted sample's "
              "window contains at least one fresh IMU reading for ImuAxis's "
              "transient peak tracking to work.");

static_assert(IMU_ACCEL_RANGE_G == 2 || IMU_ACCEL_RANGE_G == 4 ||
                  IMU_ACCEL_RANGE_G == 8 || IMU_ACCEL_RANGE_G == 16,
              "ERROR: IMU_ACCEL_RANGE_G must be one of 2, 4, 8, 16.");
static_assert(
    IMU_GYRO_RANGE_DPS == 125 || IMU_GYRO_RANGE_DPS == 245 ||
        IMU_GYRO_RANGE_DPS == 500 || IMU_GYRO_RANGE_DPS == 1000 ||
        IMU_GYRO_RANGE_DPS == 2000,
    "ERROR: IMU_GYRO_RANGE_DPS must be one of 125, 245, 500, 1000, 2000.");

// The rates the Seeed library maps; anything else silently becomes 104Hz.
static_assert(IMU_ACCEL_ODR_HZ == 13 || IMU_ACCEL_ODR_HZ == 26 ||
                  IMU_ACCEL_ODR_HZ == 52 || IMU_ACCEL_ODR_HZ == 104 ||
                  IMU_ACCEL_ODR_HZ == 208 || IMU_ACCEL_ODR_HZ == 416 ||
                  IMU_ACCEL_ODR_HZ == 833 || IMU_ACCEL_ODR_HZ == 1660 ||
                  IMU_ACCEL_ODR_HZ == 3330 || IMU_ACCEL_ODR_HZ == 6660,
              "ERROR: IMU_ACCEL_ODR_HZ must be one of 13, 26, 52, 104, 208, "
              "416, 833, 1660, 3330, 6660 - the rates the Seeed LSM6DS3 "
              "library maps. Anything else silently becomes 104 Hz.");
static_assert(IMU_GYRO_ODR_HZ == 13 || IMU_GYRO_ODR_HZ == 26 ||
                  IMU_GYRO_ODR_HZ == 52 || IMU_GYRO_ODR_HZ == 104 ||
                  IMU_GYRO_ODR_HZ == 208 || IMU_GYRO_ODR_HZ == 416 ||
                  IMU_GYRO_ODR_HZ == 833 || IMU_GYRO_ODR_HZ == 1660,
              "ERROR: IMU_GYRO_ODR_HZ must be one of 13, 26, 52, 104, 208, "
              "416, 833, 1660 - the rates the Seeed LSM6DS3 library maps for "
              "the gyro. Anything else silently becomes 104 Hz.");
static_assert(IMU_ACCEL_LPF1_ODR_DIV == 2 || IMU_ACCEL_LPF1_ODR_DIV == 4,
              "ERROR: IMU_ACCEL_LPF1_ODR_DIV must be 2 or 4 (LPF1_BW_SEL).");

// LPF1 is the only anti-alias filter before imuPoll() resamples, so its cutoff
// must be at most half the read rate.
static_assert(IMU_ACCEL_LPF1_CUTOFF_HZ * 2 <= 1000 / IMU_SAMPLE_INTERVAL_MS,
              "ERROR: the accelerometer's LPF1 cutoff (IMU_ACCEL_ODR_HZ / "
              "IMU_ACCEL_LPF1_ODR_DIV) is above half the rate imuPoll() reads "
              "at, so sensor content would alias into the transmitted band. "
              "Raise IMU_ACCEL_LPF1_ODR_DIV, lower IMU_ACCEL_ODR_HZ, or lower "
              "IMU_SAMPLE_INTERVAL_MS to read faster.");

// BW0_XL is left at default, which is only valid below 1.67kHz.
static_assert(
    IMU_ACCEL_ODR_HZ < 1667,
    "ERROR: at this ODR the accelerometer's ANALOG bandwidth (BW0_XL) "
    "is no longer irrelevant. Decide it explicitly in "
    "g_imu_lsm6ds3.cpp before raising IMU_ACCEL_ODR_HZ this far.");

// ODR must be at least the sample rate, or samples repeat.
static_assert(IMU_ACCEL_ODR_HZ * IMU_SAMPLE_INTERVAL_MS >= 1000,
              "ERROR: IMU_ACCEL_ODR_HZ must be at least the sample rate "
              "(1000 / IMU_SAMPLE_INTERVAL_MS, in g_imu_tuning.h).");
static_assert(IMU_GYRO_ODR_HZ * IMU_SAMPLE_INTERVAL_MS >= 1000,
              "ERROR: IMU_GYRO_ODR_HZ must be at least the sample rate "
              "(1000 / IMU_SAMPLE_INTERVAL_MS, in g_imu_tuning.h).");

static_assert(IMU_ENABLED == 0 || IMU_ENABLED == 1,
              "ERROR: IMU_ENABLED must be 0 or 1.");

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

// setTxPower() ignores any other value.
static_assert(
    BLE_TX_POWER_ADV_DBM == -40 || BLE_TX_POWER_ADV_DBM == -20 ||
        BLE_TX_POWER_ADV_DBM == -16 || BLE_TX_POWER_ADV_DBM == -12 ||
        BLE_TX_POWER_ADV_DBM == -8 || BLE_TX_POWER_ADV_DBM == -4 ||
        BLE_TX_POWER_ADV_DBM == 0 || BLE_TX_POWER_ADV_DBM == 2 ||
        BLE_TX_POWER_ADV_DBM == 3 || BLE_TX_POWER_ADV_DBM == 4 ||
        BLE_TX_POWER_ADV_DBM == 5 || BLE_TX_POWER_ADV_DBM == 6 ||
        BLE_TX_POWER_ADV_DBM == 7 || BLE_TX_POWER_ADV_DBM == 8,
    "ERROR: BLE_TX_POWER_ADV_DBM must be an exact nRF52840 level: -40, -20, "
    "-16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, or 8.");
static_assert(
    BLE_TX_POWER_CONN_DBM == -40 || BLE_TX_POWER_CONN_DBM == -20 ||
        BLE_TX_POWER_CONN_DBM == -16 || BLE_TX_POWER_CONN_DBM == -12 ||
        BLE_TX_POWER_CONN_DBM == -8 || BLE_TX_POWER_CONN_DBM == -4 ||
        BLE_TX_POWER_CONN_DBM == 0 || BLE_TX_POWER_CONN_DBM == 2 ||
        BLE_TX_POWER_CONN_DBM == 3 || BLE_TX_POWER_CONN_DBM == 4 ||
        BLE_TX_POWER_CONN_DBM == 5 || BLE_TX_POWER_CONN_DBM == 6 ||
        BLE_TX_POWER_CONN_DBM == 7 || BLE_TX_POWER_CONN_DBM == 8,
    "ERROR: BLE_TX_POWER_CONN_DBM must be an exact nRF52840 level: -40, "
    "-20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, or 8.");

static_assert(LOG_STATS_INTERVAL_MS > 0,
              "ERROR: LOG_STATS_INTERVAL_MS must be greater than 0.");

static_assert(BATTERY_EMA_ALPHA > 0.0f && BATTERY_EMA_ALPHA <= 1.0f,
              "ERROR: BATTERY_EMA_ALPHA must be in between 0.0 and 1.0.");

// Thresholds must be ordered and within a 1S LiPo's resting range.
static_assert(BATTERY_CUTOFF_V > 3.0f && BATTERY_CUTOFF_V < 3.7f,
              "ERROR: BATTERY_CUTOFF_V must be a resting LiPo voltage between "
              "3.0 and 3.7.");
static_assert(BATTERY_CRITICAL_V > BATTERY_CUTOFF_V &&
                  BATTERY_CRITICAL_V < BATTERY_WARN_V,
              "ERROR: BATTERY_CRITICAL_V must sit between the cutoff and "
              "warn thresholds.");
static_assert(BATTERY_WARN_V > BATTERY_CRITICAL_V && BATTERY_WARN_V < 4.2f,
              "ERROR: BATTERY_WARN_V must be > BATTERY_CRITICAL_V and < 4.2.");
static_assert(BATTERY_FULL_V > BATTERY_WARN_V && BATTERY_FULL_V <= 4.2f,
              "ERROR: BATTERY_FULL_V must be > BATTERY_WARN_V and <= 4.2.");

// Sampler: sample count, spacing, run length within the poll interval, and a
// supported TACQ.
static_assert(BATTERY_SAMPLE_COUNT >= 4 && BATTERY_SAMPLE_COUNT <= 200,
              "ERROR: BATTERY_SAMPLE_COUNT must be 4..200.");
static_assert(BATTERY_SAMPLE_SPACING_US >= 100 &&
                  BATTERY_SAMPLE_SPACING_US <= 10000,
              "ERROR: BATTERY_SAMPLE_SPACING_US must be 100..10000.");
static_assert(
    (uint32_t)BATTERY_SAMPLE_COUNT * (uint32_t)BATTERY_SAMPLE_SPACING_US <
        (uint32_t)BATTERY_POLL_INTERVAL_MS * 1000UL,
    "ERROR: sampler window (BATTERY_SAMPLE_COUNT * BATTERY_SAMPLE_SPACING_US) "
    "must fit inside BATTERY_POLL_INTERVAL_MS.");
static_assert(SAADC_TACQ_US == 3 || SAADC_TACQ_US == 5 || SAADC_TACQ_US == 10 ||
                  SAADC_TACQ_US == 15 || SAADC_TACQ_US == 20 ||
                  SAADC_TACQ_US == 40,
              "ERROR: SAADC_TACQ_US must be one of 3, 5, 10, 15, 20, "
              "40 (values the SAADC supports via analogSampleTime()).");

// switchReadOnce() takes a single read, which relies on this margin.
static_assert(POWER_SWITCH_OFF_THRESHOLD_MV - POWER_SWITCH_ON_MV >= 500 &&
                  POWER_SWITCH_OFF_MV_MIN - POWER_SWITCH_OFF_THRESHOLD_MV >=
                      500,
              "ERROR: POWER_SWITCH_OFF_THRESHOLD_MV must keep >=500mV margin "
              "on BOTH sides. The single unaveraged analogRead() in "
              "switchReadOnce() depends on it - revisit powerSwitchOn()'s note "
              "in g_power.h before widening this.");

static_assert(POWER_SWITCH_OFF_THRESHOLD_MV > 0 &&
                  POWER_SWITCH_OFF_THRESHOLD_MV < 3000,
              "ERROR: POWER_SWITCH_OFF_THRESHOLD_MV must be within the ADC "
              "range (0, 3000).");

static_assert(POWER_SWITCH_POLL_INTERVAL_MS > 0,
              "ERROR: POWER_SWITCH_POLL_INTERVAL_MS must be greater than 0.");
static_assert(STATE_SWITCH_OFF_DEBOUNCE_MS >= POWER_SWITCH_POLL_INTERVAL_MS,
              "ERROR: STATE_SWITCH_OFF_DEBOUNCE_MS must be >= "
              "POWER_SWITCH_POLL_INTERVAL_MS.");

static_assert(LED_BLINK_INTERVAL_MS > 0,
              "ERROR: LED_BLINK_INTERVAL_MS must be greater than 0.");
static_assert(LED_BATTERY_WAIT_BLINK_MS >= 50 &&
                  LED_BATTERY_WAIT_BLINK_MS <= 1000,
              "ERROR: LED_BATTERY_WAIT_BLINK_MS must be 50..1000.");

static_assert(STATE_CHARGE_ONLY_ON_USB == 0 || STATE_CHARGE_ONLY_ON_USB == 1,
              "ERROR: STATE_CHARGE_ONLY_ON_USB must be 0 or 1.");

// Zero would sleep at boot, before a client can connect.
static_assert(STATE_IDLE_TIMEOUT_MIN > 0,
              "ERROR: STATE_IDLE_TIMEOUT_MIN must be greater than 0.");

static_assert(LOG_ENABLED == 0 || LOG_ENABLED == 1,
              "ERROR: LOG_ENABLED must be 0 or 1.");

static_assert(BATTERY_HAS_GAUGE == 0 || BATTERY_HAS_GAUGE == 1,
              "ERROR: BATTERY_HAS_GAUGE must be 0 or 1.");
