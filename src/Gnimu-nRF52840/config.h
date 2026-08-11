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
#define DEVICE_ID "1001001001"

// Identifies which build this binary is, printed as the first line of the
// startup banner. Purely diagnostic - nothing branches on it.
//
// This build and nRF52840-OLED target the SAME MCU, so the wrong binary flashes
// and runs happily. They differ in POWER_SWITCH_SENSE_PIN: A4 here, A1 in the
// OLED build, because on that board A4 IS SDA and its display needs the bus.
// Flash this firmware onto OLED hardware and the switch-sense read lands on a
// pin the display is driving - a misread switch position plus a divider argued
// onto the I2C bus, presenting as flaky state transitions and a glitchy screen
// rather than anything pointing at the real cause. This line makes it visible
// on the first serial output instead.
#define GNIMU_VARIANT "nRF52840"

// ----------------------------------------------------------------------------
// --- IMU (onboard LSM6DS3TR-C) ---
// ----------------------------------------------------------------------------

#define IMU_SAMPLE_INTERVAL_MS 10 // default 100Hz filter/sample rate

// ImuAxis smoothing rates and transient thresholds. The deviation that a
// sample window's peak must exceed before it gets blended into the transmitted
// value instead of the plain EMA baseline. These are PLACEHOLDER starting
// points only. The right value depends on this specific car's vibration floor
// (engine/tire/kerb noise) versus genuine events, and must be tuned empirically
// against real track data per axis.
#define IMU_ACCEL_ALPHA 0.2f // EMA smoothing: 1.0=raw, 0.1=heavy
#define IMU_GYRO_ALPHA 0.2f  // EMA smoothing: 1.0=raw, 0.1=heavy
#define IMU_ACCEL_TRANSIENT_THRESHOLD_G 0.2f   // 0.2g = ~2.0m/s^2
#define IMU_GYRO_TRANSIENT_THRESHOLD_DPS 28.6f // 28.6deg/s = ~0.5rad/s

#define IMU_ACCEL_RANGE_G 4       // +/- g sensor range: one of 2, 4, 8, 16
#define IMU_GYRO_RANGE_DPS 500    // deg/s sensor range: 125,245,500,1000,2000
#define IMU_ACCEL_ODR_HZ 104      // output data rate; >= the 100Hz poll rate
#define IMU_GYRO_ODR_HZ 104       // output data rate; >= the 100Hz poll rate
#define IMU_ACCEL_BANDWIDTH_HZ 50 // anti-alias filter: one of 50, 100, 200, 400

// --- Per-chip zero-point offsets (raw sensor frame) ---
// Subtracted from each raw axis inside g_imu's readImuRaw() BEFORE the
// mounting remap runs, so these values are intrinsic to the chip and do not
// need to change if IMU_SWAP_XY / IMU_SIGN_* change. Units match the LSM6DS3
// native units (g for accel, deg/s for gyro).
//
// Defaults are 0 = no correction. To calibrate a specific board, run the
// src/tools/nRF52840/imu_calibration sketch and paste its printed values
// here. Typical magnitudes on a healthy chip: accel < ~0.1g, gyro < ~5deg/s.
// Bench-calibrated 2026-07-23 (average of 4 imu_calibration runs; per-run
// spread was < 0.001g accel / < 0.03dps gyro). Two further trims on top of
// that, both from in-case full-firmware behavior the bench sketch can't see
// (it only powers the IMU, not GNSS/BLE, so it misses production's
// thermal/electrical load):
//   * Z accel: +0.0075g (rest reading sat at 0.992-0.993g).
//   * Z gyro:  +0.045dps (Gnimu Monitor yaw rate sat at +0.02..0.07dps).
#define IMU_ACCEL_OFFSET_X_G +0.004967f
#define IMU_ACCEL_OFFSET_Y_G +0.003585f
#define IMU_ACCEL_OFFSET_Z_G +0.019085f
#define IMU_GYRO_OFFSET_X_DPS +0.611464f
#define IMU_GYRO_OFFSET_Y_DPS -1.458232f
#define IMU_GYRO_OFFSET_Z_DPS +0.609842f

// --- Axis orientation (installed mounting) ---
// Corrects the sensor's raw axes into the vehicle frame.
// Board-frame axes, to be bench-confirmed via src/tools/nRF52840/imu_tiltmap
// The bench-verified orientation of the XIAO Sense:
//      +X -> toward the BLE-antenna end (away from USB-C)
//      +Y -> toward the left edge (LED side)
//      +Z -> up, out of the top of the SoC face
// What may vary per BUILD is how the module sits in your enclosure. These
// four values correct for different orientations. This model assumes the
// module is mounted FLAT, with sensor Z vehicle-vertical, and covers all 8
// flat-mount variants (any 90-degree yaw rotation, right-side-up or
// upside-down). It does NOT cover mounting the board on any edge.
// Vehicle forward/lateral assignment (which of X/Y is which, and their
// final signs) still needs an in-car drive test once mounted.
#define IMU_SWAP_XY false // true if raw X axis is lateral, not longitudinal
#define IMU_SIGN_X -1.0f
#define IMU_SIGN_Y -1.0f
#define IMU_SIGN_Z +1.0f

// --- LIGHT_SLEEP wake-up detector (omni-directional shake-to-wake) ---
// Configured via direct register writes (the Seeed library has no high-level
// API for this) per ST AN4650. Bench-tuned via src/tools/nRF52840/imu_wake.
// CTRL1_XL: ODR=12.5Hz (low-power mode) | FS=+/-4g - bits [7:4] ODR_XL=0001,
// bits [3:2] FS_XL=10. A low ODR is what puts the accelerometer into the
// chip's automatic low-power mode; full accuracy isn't needed just to detect
// "something moved." FS is deliberately kept matched to IMU_ACCEL_RANGE_G
// (the RUNNING-mode range) rather than dropped to +/-2g: since this is a live
// mode switch on an already-running chip (not a cold power-on), changing
// FS_XL creates a scale discontinuity in the wake-up detector's raw-code
// threshold comparison that persists (not a settling transient - a delay
// before arming does not fix it) and reliably false-triggers immediately.
// Only ODR changes across the RUNNING<->LIGHT_SLEEP transition now.
#define IMU_WAKE_CTRL1_XL 0x18
// 6-bit wake-up threshold (0-63), LSB weight = FS_XL/64 - so this scales with
// the FS_XL chosen above; re-tune with src/tools/nRF52840/imu_wake if FS_XL
// changes. THE key tunable for shake-to-wake: too low false-triggers on small
// vibrations, too high misses a real pickup.
#define IMU_WAKE_THS 4
#define IMU_WAKE_DUR 0 // debounce, in ODR cycles - 0 = fire on first sample

// ----------------------------------------------------------------------------
// --- GNSS (HGLRC M100-5883, u-blox M10 chipset) ---
// ----------------------------------------------------------------------------

// --- GNSS power gate pin (TPS63020 buck-boost EN) ---
// Drives the regulator EN pad that powers the GNSS 3.3V rail. Wire this to
// whichever GPIO you use for the gate on your build. Must be set with
// pinMode(OUTPUT) before any digitalWrite. An INPUT-mode pin lets the TPS
// pullup silently win every write, defeating our rail-cutoff.
#define GNSS_EN_PIN D9

// LOWER is better here, within reason - this setting buys loop-latency
// tolerance, not throughput.
//
// What matters is how long the ~64-byte Serial1 RX buffer takes to fill, since
// that is the deadline by which gnssPoll() must service it or UBX bytes are
// lost and the observed PVT rate sags. Halving the baud doubles that deadline:
//
//   baud     RX fill window   NAV-PVT airtime   link use @25Hz
//   460800   ~1.4 ms          ~2.2 ms           5%
//   115200   ~5.6 ms          ~8.7 ms           22%
//   57600    ~11.1 ms         ~17.4 ms          43%
//   38400    ~16.7 ms         ~26 ms            65%
//
// Throughput is nowhere near the constraint: NAV-PVT is a fixed 100 bytes
// (92 payload + 8 framing) regardless of SV count, so 25Hz is only ~2500
// bytes/sec and link utilisation does not grow as satellites are added.
//
// This project has now walked this down twice for the same reason. 460800 was
// abandoned first - a 1.4 ms window is shorter than worst-case loop latency,
// which craters the rate. 115200 held for a long time, but the GPS+Galileo
// work (2026-08-06) needed more margin than its 5.6 ms window allowed.
//
// 57600 doubles the deadline again while still using under half the link at
// full rate. The costs are that a packet takes ~8.7 ms longer to arrive (still
// well inside one epoch, irrelevant for this use case) and that there is less
// slack if additional UBX messages are ever enabled - only NAV-PVT is today,
// and adding NAV-SAT or similar would change the arithmetic quickly here.
//
// 38400 would still work at 25Hz but the margin starts running the wrong way:
// two-thirds link utilisation, and the packet occupies ~26 ms of a 40 ms epoch.
//
// gnssBegin's connectAndConfigureBaud() sweeps common rates, reconfigures the
// receiver, and saves to flash if it is found at a different rate - so changing
// this value is a one-line edit that survives the next boot on its own.
#define GNSS_BAUD 57600
#define GNSS_NAV_RATE_HZ 25
// PVT rate while no BLE client is connected - keeps the receiver ticking (and
// the fix warm) without the full 25Hz load when nobody is listening.
#define GNSS_IDLE_NAV_RATE_HZ 1
#define GNSS_SV_MINELEV_DEG 5 // ignore SVs below this angle (anti-multipath)
#define GNSS_DYNAMIC_MODEL DYN_MODEL_AUTOMOTIVE

// --- LIGHT_SLEEP wake pulse ---
// gnssWake() rouses the receiver from RXM-PMREQ backup mode by releasing
// Serial1 and driving GNSS_TX_PIN through a manual LOW->HIGH->LOW pulse.
#define GNSS_WAKE_PULSE_MS 10

// --- GNSS Constellation Toggles ---
// Enable only the constellations your module supports and your region
// benefits from. Enabling too many can pull the update rate below 25Hz.
// HGLRC M100-5883 purports to support GPS/Galileo/GLONASS/BeiDou/QZSS/SBAS,
// though emprical testing indicates it only supports GPS + Galileo.
// Only GPS is enabled below for North American use. Testing has shown that the
// M100-5883 can't quite maintain a 25Hz fix rate with both GPS+Galileo enabled.
// Reference: https://app.qzss.go.jp/GNSSView/gnssview.html
#define GNSS_CONSTELLATIONS                                                    \
  {                                                                            \
      {"GPS", SFE_UBLOX_GNSS_ID_GPS, true},                                    \
      {"Galileo", SFE_UBLOX_GNSS_ID_GALILEO, false},                           \
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
#define BLE_TX_POWER_ADV_DBM -16  // while advertising
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
#define BATTERY_EMA_ALPHA 0.2f

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

// --- LIGHT_SLEEP timing (Phase 2) ---
// Defensive power-shedding for a device left running with no BLE client
// connected. Three tiers, all measured from last BLE disconnect in RUNNING:
//   1. STATE_IDLE_TIMEOUT_MIN (RUNNING -> LIGHT_SLEEP). GNSS drops to
//      backup mode (ephemeris retained, wakes in ~1-3s), IMU arms its
//      wake-up detector, LED blinks blue slowly. Reversible without a reset.
//      BLE client connect or IMU shake-to-wake returns to RUNNING instantly.
//   2. STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN (from LIGHT_SLEEP entry). GNSS
//      power save escalates to a full EN-cut. By this point ephemeris would
//      likely be stale anyway, so there's no reacquisition-speed cost to
//      cutting harder. BLE/IMU wake capability is unaffected.
//   3. STATE_LIGHT_SLEEP_TIMEOUT_MIN (from LIGHT_SLEEP entry).Gives up
//      entirely and drops to DEEP_SLEEP (System OFF). This is the ultimate
//      backstop for a truly-forgotten device. Past this point BLE/shake-to-wake
//      no longer work. Recovery requires a switch cycle or USB plug-in,
//      same as any other DEEP_SLEEP entry.
#define STATE_IDLE_TIMEOUT_MIN 30
#define STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN 180
#define STATE_LIGHT_SLEEP_TIMEOUT_MIN 360

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

// LIGHT_SLEEP blink - short on, long off (a "sleeping" pulse), deliberately
// distinct from the advertising blink's even on/off.
#define LED_LIGHT_SLEEP_BLINK_ON_MS 250
#define LED_LIGHT_SLEEP_BLINK_OFF_MS 5000

// ----------------------------------------------------------------------------
// --- Logging ---
// ----------------------------------------------------------------------------

// Master switch for all Serial diagnostic output.
// 1 = normal verbose output
// 0 = silent
// Every LOG_PRINT / LOG_PRINTLN / LOG_PRINTF / LOG_FLUSH call is a
// preprocessor-level no-op, both the call AND its arguments vanish before the
// compiler sees them. Turning this off eliminates a small amount of
// once-per-second stats-printf loop-latency.
#define LOG_ENABLED 0

#define LOG_STATS_INTERVAL_MS 1000 // serial stats reporting interval

// Separate, much slower cadence for LIGHT_SLEEP's own heartbeat (g_state) -
// LOG_STATS_INTERVAL_MS's 1 Hz is right for live telemetry monitoring, but
// LIGHT_SLEEP can last hours (up to STATE_LIGHT_SLEEP_TIMEOUT_MIN), and
// nothing else logs while asleep - without this there's no way to tell it's
// still alive/how long it's been asleep until it wakes back up.
#define LOG_LIGHT_SLEEP_INTERVAL_MS 10000

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
// covers up to ~800k. Bench-validated via src/tools/nRF52840/battery_presence.
// Value is dictated by the fixed divider impedance, not a free performance
// knob.
#define SAADC_TACQ_US 40

// ----------------------------------------------------------------------------
// --- IMU (onboard LSM6DS3TR-C) ---
// ----------------------------------------------------------------------------

// Powered by a dedicated enable pin; sits on the internal I2C bus (Wire1).
#define IMU_I2C_ADDRESS 0x6A // SA0 tied high on the XIAO Sense

// I2C bus speed for the onboard IMU (Wire1 / TWIM1 - a separate peripheral
// from the display's Wire/TWIM0, so the two never contend).
//
// This is a LATENCY setting, not a throughput one. imuPoll() reads six 16-bit
// registers every IMU_SAMPLE_INTERVAL_MS, and the Seeed LSM6DS3 library issues
// each as TWO separate I2C transactions (write register address, then read) -
// twelve transactions per sample tick, no burst read. At the core's default
// bus speed that is the single largest recurring blocking cost in loop(),
// larger than the display's metered slices and 100x/second rather than 32.
//
// The default is NOT 400kHz and has to be set explicitly. TwoWire::begin()
// hardcodes FREQUENCY to K100, and the LSM6DS3 library calls begin() but never
// setClock() - so without this the IMU bus runs at 100kHz while the display's
// runs at 400kHz. g_imu.cpp applies it AFTER myIMU.begin(), which is required:
// begin() resets the frequency register, so setting it earlier is silently
// undone. It is re-applied on every configureNormalMode() for the same reason
// (imuDisarmWake() calls begin() again on each LIGHT_SLEEP exit).
//
// The LSM6DS3TR-C supports I2C fast mode (400kHz) per ST's datasheet, and
// Wire1 is entirely internal to the XIAO - short traces, onboard pull-ups - so
// there is no signal-integrity reason to stay at 100kHz.
#define IMU_I2C_CLOCK_HZ 400000
#define IMU_POWER_PIN PIN_LSM6DS3TR_C_POWER
// Wake-up detector's interrupt output (onboard, no wiring needed).
#define IMU_INT1_PIN PIN_LSM6DS3TR_C_INT1

// Decimate the IMU stream down to the transmission rate. Derived from
// GNSS_NAV_RATE_HZ so the two can't drift out of sync.
#define IMU_TRANSMIT_INTERVAL_MS (1000 / GNSS_NAV_RATE_HZ)

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

// --- GNSS power gate polarity (TPS63020 buck-boost EN) ---
// Firmware pulls this LOW to disable the rail for low-voltage cutoff and idle
// sleep, and releases it hi-Z (TPS63020's own EN pullup takes over) to
// enable (HIGH/hi-Z = rail on, LOW = rail off). Polarity is fixed by the
// TPS63020's own EN behavior, not a wiring choice - see GNSS_EN_PIN in the
// Tunables section above for the pin itself.

// ----------------------------------------------------------------------------
// --- Battery ---
// ----------------------------------------------------------------------------

// This build has a real battery gauge (VBAT sensing + fuel gauge in
// g_battery): the shared telemetry module includes the battery segment in the
// serial stats line. (0 on builds without battery hardware, e.g. the ESP32
// variant, whose g_battery is a constant stub.)
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
// --- Protocol (RaceBox BLE protocol) ---
// These match the RaceBox BLE protocol and should not be changed. The UUIDs
// are the Nordic UART UUIDs, which Bluefruit's BLEUart uses natively.
// ----------------------------------------------------------------------------

#define RACEBOX_MODEL "RaceBox Mini"
#define RACEBOX_MANUFACTURER "RaceBox"
#define RACEBOX_HARDWARE_VERSION "1"
#define RACEBOX_FIRMWARE_VERSION "3.3"
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
static_assert(GNSS_IDLE_NAV_RATE_HZ >= 1 &&
                  GNSS_IDLE_NAV_RATE_HZ <= GNSS_NAV_RATE_HZ,
              "ERROR: GNSS_IDLE_NAV_RATE_HZ must be between 1 and "
              "GNSS_NAV_RATE_HZ.");

// Enforce a sane satellite elevation mask (a real angle above the horizon)
static_assert(GNSS_SV_MINELEV_DEG >= 0 && GNSS_SV_MINELEV_DEG <= 90,
              "ERROR: GNSS_SV_MINELEV_DEG must be between 0 and 90.");

// Enforce sane EMA alpha range
static_assert(IMU_ACCEL_ALPHA > 0.0f && IMU_ACCEL_ALPHA <= 1.0f,
              "ERROR: IMU_ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(IMU_GYRO_ALPHA > 0.0f && IMU_GYRO_ALPHA <= 1.0f,
              "ERROR: IMU_GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce positive transient thresholds (a zero/negative threshold would
// disable transient blending entirely; see ImuAxis::read()).
static_assert(IMU_ACCEL_TRANSIENT_THRESHOLD_G > 0.0f,
              "ERROR: IMU_ACCEL_TRANSIENT_THRESHOLD_G must be greater than 0.");
static_assert(
    IMU_GYRO_TRANSIENT_THRESHOLD_DPS > 0.0f,
    "ERROR: IMU_GYRO_TRANSIENT_THRESHOLD_DPS must be greater than 0.");

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

// Enforce IMU full-scale ranges are values the LSM6DS3 supports
static_assert(IMU_ACCEL_RANGE_G == 2 || IMU_ACCEL_RANGE_G == 4 ||
                  IMU_ACCEL_RANGE_G == 8 || IMU_ACCEL_RANGE_G == 16,
              "ERROR: IMU_ACCEL_RANGE_G must be one of 2, 4, 8, 16.");
static_assert(
    IMU_GYRO_RANGE_DPS == 125 || IMU_GYRO_RANGE_DPS == 245 ||
        IMU_GYRO_RANGE_DPS == 500 || IMU_GYRO_RANGE_DPS == 1000 ||
        IMU_GYRO_RANGE_DPS == 2000,
    "ERROR: IMU_GYRO_RANGE_DPS must be one of 125, 245, 500, 1000, 2000.");

// Enforce IMU output data rates are values the LSM6DS3 supports
static_assert(IMU_ACCEL_ODR_HZ == 13 || IMU_ACCEL_ODR_HZ == 26 ||
                  IMU_ACCEL_ODR_HZ == 52 || IMU_ACCEL_ODR_HZ == 104 ||
                  IMU_ACCEL_ODR_HZ == 208 || IMU_ACCEL_ODR_HZ == 416 ||
                  IMU_ACCEL_ODR_HZ == 833 || IMU_ACCEL_ODR_HZ == 1666 ||
                  IMU_ACCEL_ODR_HZ == 3332 || IMU_ACCEL_ODR_HZ == 6664,
              "ERROR: IMU_ACCEL_ODR_HZ must be a supported LSM6DS3 rate.");
static_assert(IMU_GYRO_ODR_HZ == 13 || IMU_GYRO_ODR_HZ == 26 ||
                  IMU_GYRO_ODR_HZ == 52 || IMU_GYRO_ODR_HZ == 104 ||
                  IMU_GYRO_ODR_HZ == 208 || IMU_GYRO_ODR_HZ == 416 ||
                  IMU_GYRO_ODR_HZ == 833 || IMU_GYRO_ODR_HZ == 1666 ||
                  IMU_GYRO_ODR_HZ == 3332 || IMU_GYRO_ODR_HZ == 6664,
              "ERROR: IMU_GYRO_ODR_HZ must be a supported LSM6DS3 rate.");
static_assert(
    IMU_ACCEL_BANDWIDTH_HZ == 50 || IMU_ACCEL_BANDWIDTH_HZ == 100 ||
        IMU_ACCEL_BANDWIDTH_HZ == 200 || IMU_ACCEL_BANDWIDTH_HZ == 400,
    "ERROR: IMU_ACCEL_BANDWIDTH_HZ must be one of 50, 100, 200, 400.");

// Enforce each axis sign is a true sign, not a scale factor
static_assert(IMU_SIGN_X == 1.0f || IMU_SIGN_X == -1.0f,
              "ERROR: IMU_SIGN_X must be exactly +1.0f or -1.0f.");
static_assert(IMU_SIGN_Y == 1.0f || IMU_SIGN_Y == -1.0f,
              "ERROR: IMU_SIGN_Y must be exactly +1.0f or -1.0f.");
static_assert(IMU_SIGN_Z == 1.0f || IMU_SIGN_Z == -1.0f,
              "ERROR: IMU_SIGN_Z must be exactly +1.0f or -1.0f.");

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
static_assert(LOG_LIGHT_SLEEP_INTERVAL_MS > 0,
              "ERROR: LOG_LIGHT_SLEEP_INTERVAL_MS must be greater than 0.");

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
static_assert(LED_LIGHT_SLEEP_BLINK_ON_MS > 0 &&
                  LED_LIGHT_SLEEP_BLINK_OFF_MS > 0,
              "ERROR: LED_LIGHT_SLEEP_BLINK_*_MS must be greater than 0.");

// State-machine feature flag: strictly 0 or 1.
static_assert(STATE_CHARGE_ONLY_ON_USB == 0 || STATE_CHARGE_ONLY_ON_USB == 1,
              "ERROR: STATE_CHARGE_ONLY_ON_USB must be 0 or 1.");

// LIGHT_SLEEP timing must be strictly increasing - each tier is measured
// from LIGHT_SLEEP entry (or, for the first tier, from losing the last BLE
// client), so a misordered value would mean a "later" escalation fires
// before an "earlier" one.
static_assert(STATE_IDLE_TIMEOUT_MIN > 0, "ERROR: STATE_IDLE_TIMEOUT_MIN must "
                                          "be greater than 0.");
static_assert(STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN > 0 &&
                  STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN <
                      STATE_LIGHT_SLEEP_TIMEOUT_MIN,
              "ERROR: STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN must be > 0 and < "
              "STATE_LIGHT_SLEEP_TIMEOUT_MIN.");
static_assert(STATE_LIGHT_SLEEP_TIMEOUT_MIN > 0,
              "ERROR: STATE_LIGHT_SLEEP_TIMEOUT_MIN must be greater than 0.");

// GNSS wake pulse must be a positive, sane hold time per level.
static_assert(GNSS_WAKE_PULSE_MS > 0 && GNSS_WAKE_PULSE_MS <= 1000,
              "ERROR: GNSS_WAKE_PULSE_MS must be 1..1000 ms.");

// IMU wake-up threshold is a 6-bit field (0-63); duration is a small packed
// field - bounded generously rather than to its exact bit width, since the
// real constraint is "fits in the register", which the write already masks.
static_assert(IMU_WAKE_THS <= 63, "ERROR: IMU_WAKE_THS must be 0..63 (6-bit "
                                  "field).");
static_assert(IMU_WAKE_DUR <= 15, "ERROR: IMU_WAKE_DUR must be 0..15.");

// Logging feature flag: strictly 0 or 1.
static_assert(LOG_ENABLED == 0 || LOG_ENABLED == 1,
              "ERROR: LOG_ENABLED must be 0 or 1.");

// Battery-gauge feature flag: strictly 0 or 1.
static_assert(BATTERY_HAS_GAUGE == 0 || BATTERY_HAS_GAUGE == 1,
              "ERROR: BATTERY_HAS_GAUGE must be 0 or 1.");
