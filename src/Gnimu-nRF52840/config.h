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
//      alphas, feature flags, per-build calibration, and wiring choices you
//      made yourself (i.e., which GPIO you routed a signal to).
//   2. SUPPORTING CONSTANTS - hardware descriptors that will break the
//      firmware if changed without also changing the physical part: pins/
//      addresses fixed by the board or chip, protocol UUIDs, and values
//      derived from other constants.
// Within each section, entries are grouped by subsystem (Device Identity,
// SAADC, IMU, GNSS, BLE, Battery, Power, State, LED, Logging, Protocol), in
// the same order in both sections. Every define is prefixed with the
// subsystem it belongs to, and every value that carries a unit is suffixed
// with it (_MS, _US, _MV, _V, _HZ, _G, _DPS, _DEG, _DBM, _OHM, _BITS, _PIN).

// Pull in Arduino.h for the board's pin/peripheral macros (D6, D7,
// PIN_LSM6DS3TR_C_POWER, ...). config.h uses these symbolic names instead of
// raw pin numbers, so it must see Arduino.h first.
#include <Arduino.h>

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
// It is a STRING of exactly 10 digits - quote it, so leading zeros are kept
// (e.g. "0123456789"). Do NOT use a bare number: a leading zero would be read
// as an octal literal and an unquoted ID loses its leading zeros.
// First digit must be 0-3, so the value stays below 4000000000 - the RaceBox
// app will not connect to IDs of 4000000000 or higher. See compile-time
// validation at the bottom of this file.
#define DEVICE_ID "1000000002"

// Identifies which build this binary is, printed as the first line of the
// startup banner. Purely diagnostic - nothing branches on it.
#define GNIMU_VARIANT "nRF52840"

// ----------------------------------------------------------------------------
// --- IMU (onboard LSM6DS3TR-C) ---
// ----------------------------------------------------------------------------

// Is an IMU fitted? Follows the board selected in the IDE, so it is right
// without editing anything: the XIAO nRF52840 Sense (and Sense Plus) carries
// an onboard LSM6DS3TR-C; the plain XIAO nRF52840 (and Plus) does not. The
// build defines ARDUINO_<board> from boards.txt, which is what this tests.
//
// With 0, the LSM6DS3 driver compiles to a stub - the Seeed LSM6DS3 library is
// then not needed to build - the IMU fields read zero exactly as if the IMU had
// failed, and trim never runs. RaceChrono never uses IMU data; RaceBox works
// and reports zero g. To override, replace this block with a plain
// `#define IMU_ENABLED 0` or `1`.
#if defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) ||                              \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)
#define IMU_ENABLED 1
#else
#define IMU_ENABLED 0
#endif

#define IMU_ACCEL_RANGE_G 4       // +/- g sensor range: one of 2, 4, 8, 16
#define IMU_GYRO_RANGE_DPS 500    // deg/s sensor range: 125,245,500,1000,2000
#define IMU_ACCEL_ODR_HZ 104      // output data rate; >= the 100Hz poll rate
#define IMU_GYRO_ODR_HZ 104       // output data rate; >= the 100Hz poll rate

// Accelerometer digital low-pass filter (LPF1), as the ODR DIVIDER the part
// implements - 2 or 4, ST's LPF1_BW_SEL in CTRL1_XL. At IMU_ACCEL_ODR_HZ 104
// that is 52 Hz or 26 Hz.
//
// A divider rather than a bandwidth in Hz because that is what the register
// holds, and because Hz cannot express every case: ODR 13 would need 6.5 Hz.
//
// THIS REPLACED `IMU_ACCEL_BANDWIDTH_HZ 50` (2026-09-11), which was wrong in
// name and value. The Seeed library targets the original LSM6DS3, where
// CTRL1_XL bits 1:0 are an analog anti-alias filter (400/200/100/50 Hz). On the
// LSM6DS3TR-C actually fitted, ST splits those two bits: bit 0 BW0_XL (analog
// chain, and per ST's driver "only for accelerometer ODR >= 1.67 kHz" - inert
// here) and bit 1 LPF1_BW_SEL (ODR/2 or ODR/4). So the old "50" selected
// neither 50 Hz nor an anti-alias filter; it happened to land on ODR/4, which
// is what this keeps. Confirmed against ST's lsm6ds3tr-c-pid register driver.
#define IMU_ACCEL_LPF1_ODR_DIV 4

// Derived, for reading and for the checks at the bottom of this file.
#define IMU_ACCEL_LPF1_CUTOFF_HZ (IMU_ACCEL_ODR_HZ / IMU_ACCEL_LPF1_ODR_DIV)

// Sample interval, smoothing, transient thresholds and runtime trim: shared by
// every board, in g_imu_tuning.h (included at the top of this file).

// --- Axis orientation (installed mounting) ---
// Corrects the sensor's raw axes into the vehicle frame.
// The bench-verified orientation of the XIAO Sense:
//      +X -> toward the BLE-antenna end (away from USB-C)
//      +Y -> toward the left edge (LED side)
//      +Z -> up, out of the top of the SoC face
// What may vary per BUILD is how the module sits in your enclosure. Each
// VEHICLE axis below names which SENSOR axis feeds it (0=X, 1=Y, 2=Z) plus a
// sign. This covers all 24 physically-realizable orientations.
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
// TO DERIVE A NEW MAP, no drive test is needed - hold the assembled unit in
// its installed orientation and read the 1 Hz serial mG line:
//   1. At rest, the axis reading ~+/-1000 is vehicle-vertical; sign gives
//      up vs down.
//   2. Raise the forward end - the axis going positive is vehicle X.
//   3. Raise the left side  - the axis going positive is vehicle Y.
#define IMU_AXIS_X_SRC 0 // vehicle forward <- sensor X
#define IMU_AXIS_X_SIGN -1.0f
#define IMU_AXIS_Y_SRC 1 // vehicle left    <- sensor Y
#define IMU_AXIS_Y_SIGN -1.0f
#define IMU_AXIS_Z_SRC 2 // vehicle up      <- sensor Z
#define IMU_AXIS_Z_SIGN +1.0f

// ----------------------------------------------------------------------------
// --- GNSS (HGLRC M100-5883, u-blox M10 chipset) ---
// ----------------------------------------------------------------------------

// --- GNSS power gate pin (TPS63020 buck-boost EN) ---
// Drives the regulator EN pad that powers the GNSS 3.3V rail. Wire this to
// whichever GPIO you use for the gate on your build. Must be set with
// pinMode(OUTPUT) before any digitalWrite. An INPUT-mode pin lets the TPS
// pullup silently win every write, defeating our rail-cutoff.
#define GNSS_EN_PIN D9

// No need for greater than 115200; higher can reduce PVT rate.
#define GNSS_BAUD 115200
#define GNSS_NAV_RATE_HZ 20   // 20 is max for 2 enabled constellations
#define GNSS_SV_MINELEV_DEG 5 // ignore SVs below this angle (anti-multipath)
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

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
// --- BLE (Bluefruit) ---
// ----------------------------------------------------------------------------

// BLE Transmit Power in dBm.
// Two independent levels: the power used while ADVERTISING (idle/discoverable)
// and the power used once a client is CONNECTED. Lower power reduces RF
// interference with the GNSS.
// Valid nRF52840 levels: -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8.
#define BLE_TX_POWER_ADV_DBM 0    // while advertising
#define BLE_TX_POWER_CONN_DBM -16 // while client is connected

// ----------------------------------------------------------------------------
// --- Battery ---
// ----------------------------------------------------------------------------

// --- Low-voltage cutoff ---
// On a sustained VBAT drop below the cutoff voltage, the firmware kills the
// GNSS rail (EN pulled low) then enters System OFF, which remains latched until
// a real power event (USB plug-in or a switch off->on cycle).
// Debounce so acquisition current spikes don't trip it.
#define BATTERY_CUTOFF_V 3.35f          // Cutoff voltage threshold
#define BATTERY_WARN_V 3.60f            // Amber LED blink voltage threshold
#define BATTERY_CRITICAL_V 3.40f        // Red LED blink voltage threshold
#define BATTERY_CUTOFF_DEBOUNCE_MS 5000 // low-V duration before tripping

// --- Non-blocking VBAT sampler ---
// The sampler advances an IDLE/SAMPLING state machine from batteryPoll().
// Every BATTERY_POLL_INTERVAL_MS it collects BATTERY_SAMPLE_COUNT paced ADC
// reads (~BATTERY_SAMPLE_SPACING_US between reads -> ~50ms observation
// window at defaults), tracks min/max, and terminates on count. Peak feeds
// both SoC and the low-voltage cutoff (peak rejects load-sag transients that
// lie about the resting voltage). At 4Hz cadence a ~5s cutoff debounce
// sees ~20 runs of evidence before tripping.
#define BATTERY_POLL_INTERVAL_MS 250   // 4Hz between sampling-run starts
#define BATTERY_SAMPLE_COUNT 20        // paced ADC reads per run
#define BATTERY_SAMPLE_SPACING_US 2500 // paced sample spacing (~50ms window)

// --- Charge-full indication (LED only) ---
// The XIAO exposes no charge-complete signal from its charger IC, so "fully
// charged" is approximated from voltage: while charging (USB present), a cell
// voltage at or above BATTERY_FULL_V shows as full (solid green LED) vs.
// blinking green while still charging.
//
// Deliberately set BELOW the 4.2V CV target, and NOT equal to the discharge
// curve's "4.20V = 100%" point, which is a *resting* voltage, a different
// thing. During charge the charger holds ~4.2V with its own tolerance, and
// the VBAT divider/ADC add theirs, so a 4.2V threshold might never be
// reached and "full" would never latch. A small margin guarantees it triggers
// near the top.
#define BATTERY_FULL_V 4.15f

// --- Fast charge ---
// Define to select the ~100mA charge pad on the XIAO(default ~50mA).
// Comment out to leave the board at its default charge current.
#define BATTERY_FAST_CHARGE

// EMA factor across sampler runs for the DISPLAYED voltage (percent/LED):
// higher = snappier, lower = smoother. The low-voltage cutoff uses the
// un-smoothed fresh peak, so this is display-only and can never hide a
// genuine low voltage cutoff from the safety path.
#define BATTERY_EMA_ALPHA 0.1f

// --- Discharge curve (resting volts -> percent), high to low ---
// A macro (not an array) so the table lives here while g_battery.cpp owns
// the single instance: it expands to an initializer list of
// {voltage, percent} pairs. Must be sorted high voltage -> low.
//
// Deliberately simplified to just two endpoints - a straight line, not a
// real LiPo discharge curve.
#define BATTERY_DISCHARGE_CURVE {{3.90f, 100}, {3.35f, 0}}

// ----------------------------------------------------------------------------
// --- Power ---
// ----------------------------------------------------------------------------

// --- Slide-switch sense via voltage divider ---
// The 2-position, 3-pole slide switch's spare throw is wired through a
// 510k/510k resistor divider to A4: reads ~2V (of the ~4.2V node) when the
// switch is OFF and ~0V when ON. This gives a load-independent, SoC-
// independent battery-present signal. Read as ANALOG (via ADC), not digital.
// Divider sized at 510k for low always-on switch-off standby draw (~4uA), while
// staying inside the SAADC's TACQ=40us source-impedance budget and the GPIO
// input-leakage margin below POWER_SWITCH_OFF_THRESHOLD_MV.
#define POWER_SWITCH_SENSE_PIN A4         // Set to your selected GPIO pin
#define POWER_SWITCH_OFF_THRESHOLD_MV 800 // pin mv above this == switch OFF

// The two levels the threshold sits between. Not used at runtime - they exist
// so the margin that justifies switchReadOnce()'s SINGLE unaveraged read is a
// compile-time fact rather than a sentence in g_power.h that can drift away
// from the code (which is exactly what it did - see ROB-2).
//
// OFF_MV_MIN is deliberately the WORST case, not the nice one: 510k/510k halves
// the cell, and the cell is at its lowest usable point at the 3.35V end of
// BATTERY_DISCHARGE_CURVE, giving ~1675mV. At a full 4.2V the tap reads ~2100mV
// and the margin is wider still.
#define POWER_SWITCH_ON_MV 0         // switch ON shorts the tap to ground
#define POWER_SWITCH_OFF_MV_MIN 1675 // 510k/510k at the 3.35V discharge floor

// How often powerSwitchOn() refreshes its cached switch-sense reading. Reads
// between refreshes return the cache, keeping the per-loop cost to a compare.
#define POWER_SWITCH_POLL_INTERVAL_MS 50

// ----------------------------------------------------------------------------
// --- State ---
// ----------------------------------------------------------------------------

// When 1, plugging in USB with the switch ON auto-enters CHARGE_ONLY (all
// peripherals held off, LiPo cell in circuit for max charge speed). Exit is
// unplug USB or switch off, both of which reset the MCU back through the
// boot classifier.
// When 0, USB presence is ignored and the device stays in RUNNING while plugged
// in. Use this mode for bench development so a plugged-in device continues
// streaming/serving BLE.
#define STATE_CHARGE_ONLY_ON_USB 0

// --- Idle cutoff ---
// A device left running that nobody is using drops to DEEP_SLEEP (System OFF)
// after this long. Recovery is a switch off->on cycle or a USB plug-in, and the
// GNSS cold-starts, as after any other DEEP_SLEEP. This is the backstop for a
// forgotten device: between it and the low-voltage cutoff, the cell is never
// run flat by a unit nobody is watching.
//
// "Nobody is using it" means no BLE client SUBSCRIBED to notifications, not
// merely none connected. A central that connects and never subscribes - a
// stranger, or a forgotten nRF Connect session - receives nothing, and must not
// hold the device at full power until the battery cutoff. The clock also stands
// still on USB power, where there is no cell to protect.
//
// This replaced a LIGHT_SLEEP state (GNSS backup mode, IMU shake-to-wake, BLE
// wake), removed 2026-09-11: the most intricate machinery in the firmware, for
// a saving only a forgotten device ever collected. See "LIGHT_SLEEP removed" in
// docs/code-review-remediation.md.
#define STATE_IDLE_TIMEOUT_MIN 240 // 4 h

// --- Switch-off debounce ---
// A floating/unwired switch-sense pin, and even a wired divider under EMI,
// can produce transient OFF readings; require the OFF to be sustained this
// long before entering BATTERY_WAIT so a noise spike can't reset a
// happily-running device. Switch-ON is instant.
#define STATE_SWITCH_OFF_DEBOUNCE_MS 500

// ----------------------------------------------------------------------------
// --- LED (onboard RGB status LED) ---
// ----------------------------------------------------------------------------

#define LED_BLINK_INTERVAL_MS 1000    // standard interval for blinking states
#define LED_BATTERY_WAIT_BLINK_MS 150 // rapid blink for BATTERY_WAIT

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
// These are hardware descriptors or protocol requirements. Changing them
// without also changing the physical part (or the RaceBox protocol) will
// break the firmware.
// ============================================================================
// ============================================================================

// ----------------------------------------------------------------------------
// --- SAADC (shared nRF52840 ADC peripheral) ---
// ----------------------------------------------------------------------------

// nRF ADC config that g_power applies to the shared SAADC (used by both the
// VBAT sampler and the switch-sense read). The internal 3.0V reference
// gives a clean full-scale for a 1S LiPo.
#define SAADC_RESOLUTION_BITS 12
#define SAADC_REFERENCE_MV 3000.0f

// SAADC acquisition time (TACQ), in microseconds. The core default is 3us,
// valid only up to a ~40k source; the XIAO VBAT divider is ~338k (1M || 510k)
// and the switch-sense divider is ~255k (510k || 510k), so short TACQ leaves
// the sampling cap under-charged and every read undershoots ~100mV. 40us
// covers up to ~800k. Value is dictated by the fixed divider impedance, not a
// free performance knob.
#define SAADC_TACQ_US 40

// ----------------------------------------------------------------------------
// --- IMU (onboard LSM6DS3TR-C) ---
// ----------------------------------------------------------------------------

// Powered by a dedicated enable pin; sits on the internal I2C bus (Wire1).
// Also a clean way to rehearse a missing IMU: 0x6B is the part's only other
// address, and the Sense wires SA0 high, so nothing answers there.
#define IMU_I2C_ADDRESS 0x6A // SA0 tied high on the XIAO Sense

// I2C bus speed for the onboard IMU.
//
// This is a LATENCY setting, not a throughput one. imuPoll() reads six 16-bit
// registers every IMU_SAMPLE_INTERVAL_MS, and the Seeed LSM6DS3 library issues
// each as TWO separate I2C transactions (write register address, then read) -
// twelve transactions per sample tick, no burst read. At the core's default
// bus speed that is the single largest recurring blocking cost in loop().
//
// The default is NOT 400kHz and has to be set explicitly. TwoWire::begin()
// hardcodes FREQUENCY to K100, and the LSM6DS3 library calls begin() but never
// setClock() - so without this the IMU bus runs at 100kHz while the display's
// runs at 400kHz. The LSM6DS3 driver (g_imu_lsm6ds3.cpp) applies it AFTER
// myIMU.begin(), which is required:
// begin() resets the frequency register, so setting it earlier is silently
// undone.
//
// The LSM6DS3TR-C supports I2C fast mode (400kHz) per ST's datasheet, and
// Wire1 is entirely internal to the XIAO - short traces, onboard pull-ups - so
// there is no signal-integrity reason to stay at 100kHz.
#define IMU_I2C_CLOCK_HZ 400000
#define IMU_POWER_PIN PIN_LSM6DS3TR_C_POWER

// ----------------------------------------------------------------------------
// --- GNSS (HGLRC M100-5883, u-blox M10 chipset) ---
// ----------------------------------------------------------------------------

// --- GNSS UART ---
// On the XIAO, Serial1 is D7 (RX) / D6 (TX). The only hardware-UART pins.
// Wiring:
//   XIAO D6 (Serial1 TX) -> GNSS RX
//   XIAO D7 (Serial1 RX) <- GNSS TX
#define GNSS_RX_PIN D7
#define GNSS_TX_PIN D6

// ----------------------------------------------------------------------------
// --- Battery ---
// ----------------------------------------------------------------------------

// This build has a real battery gauge (VBAT sensing + fuel gauge in
// g_battery): the shared telemetry module includes the battery segment in the
// serial stats line. Set to 0 for non-battery builds.
#define BATTERY_HAS_GAUGE 1

// --- Voltage-sense scaling ---
// Recovered VBAT = ADC_volts * (R_TOP + R_BOTTOM) / R_BOTTOM.
// Values are the XIAO nRF52840's onboard divider.
#define BATTERY_DIVIDER_R_TOP_OHM 1000000.0f   // ohms
#define BATTERY_DIVIDER_R_BOTTOM_OHM 510000.0f // ohms
#define BATTERY_DIVIDER_RATIO                                                  \
  ((BATTERY_DIVIDER_R_TOP_OHM + BATTERY_DIVIDER_R_BOTTOM_OHM) /                \
   BATTERY_DIVIDER_R_BOTTOM_OHM)

// --- Battery voltage sense (XIAO internal VBAT divider) ---
// PIN_VBAT / VBAT_ENABLE are normally provided by the variant; fall back to
// the documented Seeed pin numbers if a given core doesn't define them.
// VBAT_ENABLE is driven LOW per-read to connect the divider, then released to
// save power. Confirm these against your installed variant.h.
// Values below are Arduino digital-pin *indices* (what analogRead/digitalWrite
// take), which the variant's g_ADigitalPinMap resolves to the actual nRF port
// pin. Comments note the underlying P0.x pin they land on.
#ifdef PIN_VBAT
#define BATTERY_ADC_PIN PIN_VBAT
#else
#define BATTERY_ADC_PIN 32 // Arduino D32 -> P0.31 / AIN7 (VBAT sense)
#endif
#ifdef VBAT_ENABLE
#define BATTERY_ADC_ENABLE_PIN VBAT_ENABLE
#else
#define BATTERY_ADC_ENABLE_PIN 14 // Arduino D14 -> P0.14 (LOW = divider on)
#endif
// Active-low: driven LOW to connect the divider.

// --- Charge-current select (HICHG pad) ---
// Drive LOW for the ~100mA fast-charge pad, release/HIGH for the ~50mA
// default. See BATTERY_FAST_CHARGE in the Tunables section above.
#ifdef PIN_HICHG
#define BATTERY_CHARGE_CURRENT_PIN PIN_HICHG
#else
#define BATTERY_CHARGE_CURRENT_PIN 22 // Arduino D22 -> P0.13 (HICHG)
#endif

// ----------------------------------------------------------------------------
// --- LED (onboard RGB status LED, active-LOW) ---
// ----------------------------------------------------------------------------

// The XIAO's RGB LED is common-anode: drive a pin LOW to light that color
// (active-low).
#define LED_RED_PIN LED_RED
#define LED_GREEN_PIN LED_GREEN
#define LED_BLUE_PIN LED_BLUE

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

// Enforce GNSS_RX_PIN and GNSS_TX_PIN are distinct pins.
static_assert(GNSS_RX_PIN != GNSS_TX_PIN,
              "ERROR: GNSS_RX_PIN and GNSS_TX_PIN must be different pins.");

// Enforce a GNSS_BAUD the firmware actually knows how to detect and switch
// to. Keep this list in sync with the fallback baud rates in
// connectAndConfigureBaud() (gnss.cpp) - an unswept value here risks telling
// the module to save an unrecoverable baud rate to flash.
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
              "ERROR: GNSS_SV_MINELEV_DEG must be between 0 and 90.");

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

// Enforce IMU full-scale ranges are values the LSM6DS3 supports
static_assert(IMU_ACCEL_RANGE_G == 2 || IMU_ACCEL_RANGE_G == 4 ||
                  IMU_ACCEL_RANGE_G == 8 || IMU_ACCEL_RANGE_G == 16,
              "ERROR: IMU_ACCEL_RANGE_G must be one of 2, 4, 8, 16.");
static_assert(
    IMU_GYRO_RANGE_DPS == 125 || IMU_GYRO_RANGE_DPS == 245 ||
        IMU_GYRO_RANGE_DPS == 500 || IMU_GYRO_RANGE_DPS == 1000 ||
        IMU_GYRO_RANGE_DPS == 2000,
    "ERROR: IMU_GYRO_RANGE_DPS must be one of 125, 245, 500, 1000, 2000.");

// Enforce IMU output data rates the Seeed LSM6DS3 library actually MAPS, which
// is not the same list as the datasheet's. Its begin() switches on these plain
// integers and sends anything it does not recognise to `default:` - 104 Hz,
// silently. The datasheet's 1.66/3.33/6.66 kHz steps are quoted as 1666/3332/
// 6664 in places; the library's cases are 1660/3330/6660, so the rounder
// spellings used to compile here and run at 104 Hz. The gyro list is shorter:
// the library maps no gyro rate above 1660. Found 2026-09-11; the driver's own
// odrCode() static_assert keeps the two lists in step.
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
// The part offers exactly these two.
static_assert(IMU_ACCEL_LPF1_ODR_DIV == 2 || IMU_ACCEL_LPF1_ODR_DIV == 4,
              "ERROR: IMU_ACCEL_LPF1_ODR_DIV must be 2 or 4 (LPF1_BW_SEL).");

// The sensor's filter is the ONLY thing protecting the resample this firmware
// does: the chip updates its output registers at IMU_ACCEL_ODR_HZ and imuPoll()
// reads them at 1000 / IMU_SAMPLE_INTERVAL_MS. Content above half the READ rate
// folds into the band we keep, and nothing downstream can undo it - the EMA in
// ImuAxis attenuates the folded energy but cannot tell it from signal, and the
// transient-peak detector sees the raw sample before any of that.
//
// At 104 Hz ODR read at 100 Hz this passes with the divider at 4 (26 Hz) and
// FAILS at 2 (52 Hz), which is the honest answer: 52 Hz content would alias.
// Raising IMU_ACCEL_ODR_HZ without also polling faster fails for the same
// reason - at 208 Hz every divider aliases against a 100 Hz read.
static_assert(IMU_ACCEL_LPF1_CUTOFF_HZ * 2 <= 1000 / IMU_SAMPLE_INTERVAL_MS,
              "ERROR: the accelerometer's LPF1 cutoff (IMU_ACCEL_ODR_HZ / "
              "IMU_ACCEL_LPF1_ODR_DIV) is above half the rate imuPoll() reads "
              "at, so sensor content would alias into the transmitted band. "
              "Raise IMU_ACCEL_LPF1_ODR_DIV, lower IMU_ACCEL_ODR_HZ, or lower "
              "IMU_SAMPLE_INTERVAL_MS to read faster.");

// BW0_XL, the other half of those two bits, selects the ANALOG chain bandwidth
// and does nothing below 1.67 kHz (ST's driver says so in as many words), which
// is why imuSensorBegin() leaves it at the part's default. Above that rate it
// starts to matter and this config would have to choose it deliberately.
static_assert(IMU_ACCEL_ODR_HZ < 1667,
              "ERROR: at this ODR the accelerometer's ANALOG bandwidth (BW0_XL) "
              "is no longer irrelevant. Decide it explicitly in "
              "g_imu_lsm6ds3.cpp before raising IMU_ACCEL_ODR_HZ this far.");

// Enforce output data rates at least as fast as the sample rate. Polled faster
// than the part produces samples, imuPoll() reads the same sample twice: the
// filters see duplicates, and the rate they effectively run at - the one the
// alphas in g_imu_tuning.h are tuned for - becomes the ODR, not
// IMU_SAMPLE_INTERVAL_MS. Checked here, not there, because the ODR is per part.
static_assert(IMU_ACCEL_ODR_HZ * IMU_SAMPLE_INTERVAL_MS >= 1000,
              "ERROR: IMU_ACCEL_ODR_HZ must be at least the sample rate "
              "(1000 / IMU_SAMPLE_INTERVAL_MS, in g_imu_tuning.h).");
static_assert(IMU_GYRO_ODR_HZ * IMU_SAMPLE_INTERVAL_MS >= 1000,
              "ERROR: IMU_GYRO_ODR_HZ must be at least the sample rate "
              "(1000 / IMU_SAMPLE_INTERVAL_MS, in g_imu_tuning.h).");

static_assert(IMU_ENABLED == 0 || IMU_ENABLED == 1,
              "ERROR: IMU_ENABLED must be 0 or 1.");

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

// The target output frame is right-handed, so any valid mounting map is a
// proper rotation: determinant +1. Determinant = permutation parity x the
// product of the signs, so an even permutation needs an even number of sign
// flips and an odd permutation an odd number. Anything else is a MIRROR - an
// orientation no physical mounting can produce.
//
// This matters because a mirror fails silently and slowly. The data still
// looks entirely plausible; whichever axes are mis-signed are simply wrong,
// for accel and gyro alike, since remapAxes() applies the same matrix to both.
// THIS config carried exactly that bug for months - X had been flipped without
// Y - and it survived a drive test and an app-side investigation before the
// arithmetic caught it.
static_assert(IMU_AXIS_PARITY * IMU_AXIS_X_SIGN * IMU_AXIS_Y_SIGN *
                      IMU_AXIS_Z_SIGN ==
                  1.0f,
              "ERROR: axis map is a mirror, not a rotation (determinant -1). "
              "An even permutation needs an even number of sign flips, an odd "
              "permutation an odd number. See the order table in the Axis "
              "orientation section above.");

// Enforce both BLE TX power levels are exact levels the nRF52840 radio
// supports. Bluefruit.setTxPower() rejects (and ignores) any other value at
// runtime, so membership - not just a range - is checked here.
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

// Enforce a positive stats-reporting interval.
static_assert(LOG_STATS_INTERVAL_MS > 0,
              "ERROR: LOG_STATS_INTERVAL_MS must be greater than 0.");

// Enforce a sane EMA alpha for the displayed voltage smoother.
static_assert(BATTERY_EMA_ALPHA > 0.0f && BATTERY_EMA_ALPHA <= 1.0f,
              "ERROR: BATTERY_EMA_ALPHA must be in between 0.0 and 1.0.");

// Enforce sane, correctly-ordered battery thresholds. The window is bounded
// to a 1S LiPo's usable resting range so a typo can't disable the safety
// cutoff.
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

// Sampler sanity: enough samples per run for min/max to be meaningful; window
// no wider than the poll interval; TACQ one of the values the core supports.
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

// Switch-sense margin. switchReadOnce() takes ONE unaveraged analogRead(), and
// that is only defensible because the threshold sits hundreds of millivolts
// from both levels - far beyond any SAADC channel-switch settling error, which
// is tens of millivolts at most on a channel with TACQ set for the divider's
// ~255k source impedance. If the divider or the threshold moves, that argument
// has to be re-made rather than assumed.
static_assert(POWER_SWITCH_OFF_THRESHOLD_MV - POWER_SWITCH_ON_MV >= 500 &&
                  POWER_SWITCH_OFF_MV_MIN - POWER_SWITCH_OFF_THRESHOLD_MV >= 500,
              "ERROR: POWER_SWITCH_OFF_THRESHOLD_MV must keep >=500mV margin "
              "on BOTH sides. The single unaveraged analogRead() in "
              "switchReadOnce() depends on it - revisit powerSwitchOn()'s note "
              "in g_power.h before widening this.");

// Switch-sense threshold sits inside the ADC's readable range.
static_assert(POWER_SWITCH_OFF_THRESHOLD_MV > 0 &&
                  POWER_SWITCH_OFF_THRESHOLD_MV < 3000,
              "ERROR: POWER_SWITCH_OFF_THRESHOLD_MV must be within the ADC "
              "range (0, 3000).");

// Switch-sense timing sanity: a positive poll cadence, and a debounce long
// enough to span several polls so it actually filters noise.
static_assert(POWER_SWITCH_POLL_INTERVAL_MS > 0,
              "ERROR: POWER_SWITCH_POLL_INTERVAL_MS must be greater than 0.");
static_assert(STATE_SWITCH_OFF_DEBOUNCE_MS >= POWER_SWITCH_POLL_INTERVAL_MS,
              "ERROR: STATE_SWITCH_OFF_DEBOUNCE_MS must be >= "
              "POWER_SWITCH_POLL_INTERVAL_MS.");

// LED blink timing sanity.
static_assert(LED_BLINK_INTERVAL_MS > 0,
              "ERROR: LED_BLINK_INTERVAL_MS must be greater than 0.");
static_assert(LED_BATTERY_WAIT_BLINK_MS >= 50 &&
                  LED_BATTERY_WAIT_BLINK_MS <= 1000,
              "ERROR: LED_BATTERY_WAIT_BLINK_MS must be 50..1000.");

// State-machine feature flag: strictly 0 or 1.
static_assert(STATE_CHARGE_ONLY_ON_USB == 0 || STATE_CHARGE_ONLY_ON_USB == 1,
              "ERROR: STATE_CHARGE_ONLY_ON_USB must be 0 or 1.");

// A positive idle cutoff: zero would drop to DEEP_SLEEP on the first pass with
// nobody subscribed - i.e. at boot, before any app has had a chance to connect.
static_assert(STATE_IDLE_TIMEOUT_MIN > 0,
              "ERROR: STATE_IDLE_TIMEOUT_MIN must be greater than 0.");

// Logging feature flag: strictly 0 or 1.
static_assert(LOG_ENABLED == 0 || LOG_ENABLED == 1,
              "ERROR: LOG_ENABLED must be 0 or 1.");

// Battery-gauge feature flag: strictly 0 or 1.
static_assert(BATTERY_HAS_GAUGE == 0 || BATTERY_HAS_GAUGE == 1,
              "ERROR: BATTERY_HAS_GAUGE must be 0 or 1.");
