# Migrating from Classic

If you built a device from the [Classic](../classic/README.md) instructions, it can run the current firmware. This page covers what to do and what will be different afterwards.

You don't have to migrate. Classic still works on the hardware it was written for. It just won't receive updates or fixes.

| Classic build | Can it run the current firmware? |
|---|---|
| **ESP32** | Yes, with no hardware changes. |
| **nRF52840** | Yes. Either select the Classic pinout in the firmware, or move two wires. |
| **nRF52840-OLED** | Not with its display. The OLED variant has no successor; stay on Classic if you want the screen. |

---

## Before you flash

**Write down the settings you changed.** If you edited Classic's `config.h`, note the values you changed from the defaults. The settings are in a different file with different names now; the tables below map them.

**Expect a new BLE name.** Classic advertised as `RaceBox Mini` followed by the `DEVICE_ID` you chose. The current firmware derives that number from the chip's hardware ID instead, so your device will show up under a different name. Connect to the new name in your app, and remove the old device from the app if it keeps a list. The new name is stable: it will be the same on every boot from now on. The boot log prints it as `Device ID`.

Then build and flash as described in [Building and flashing](building.md). There is nothing you have to edit first on an ESP32.

---

## ESP32

**Hardware:** nothing changes. The GNSS stays on GPIO16/17, the MPU-6050 on GPIO21/22, and the status LED is the onboard one on GPIO2.

If you moved the GNSS or LED to different pins by editing Classic's `config.h`, set the same pins in [`code/Gnimu/src/platform/esp32/BoardConfig.h`](../code/Gnimu/src/platform/esp32/BoardConfig.h).

---

## nRF52840

**Hardware:** two signals moved between Classic and the current pinout.

| Signal | Classic | Current |
|---|---|---|
| GNSS regulator enable (EN) | D9 | D1 |
| Power-switch sense | A4 | A0 |

Everything else is the same: the GNSS on D6/D7, the onboard IMU and LED, and the battery wiring. The regulator can stay as it is; the firmware drives a TPS63020 and a TPS63070 the same way.

You have two options:

- **Keep your wiring.** In [`code/Gnimu/src/platform/nrf52840/BoardConfig.h`](../code/Gnimu/src/platform/nrf52840/BoardConfig.h), change `kWiring` from `Wiring::V2` to `Wiring::Classic`, then build and flash. You'll need to make that one edit again whenever you update the firmware.
- **Move the two wires** to D1 and A0, and flash the firmware unmodified.

> [!IMPORTANT]
> Don't flash the unmodified firmware onto Classic wiring. With the wrong pinout the firmware can't control the GNSS power or read the switch, and the device won't behave correctly.

---

## Settings

All settings are now in [`code/Gnimu/Config.h`](../code/Gnimu/Config.h), described in [Configuration](configuration.md). The defaults match Classic's where the setting still exists: GPS and Galileo at 20 Hz, and -12 dBm BLE power.

| Classic `config.h` | Now |
|---|---|
| `DEVICE_ID` | Gone. Derived from the chip's hardware ID. |
| `GNSS_NAV_RATE_HZ` | `kGnssNavRateHz` |
| `GNSS_SV_MINELEV_DEG` | `kGnssMinElevDeg` |
| `GNSS_CONSTELLATIONS` | `kConstellationPriority` and `kConstellationCount`: an ordered list and how many of it to enable, instead of an on/off flag per constellation. QZSS and SBAS are always off. |
| `GNSS_BAUD` | No longer a setting. The firmware always runs the receiver at 115200. |
| `IMU_ACCEL_ALPHA`, `IMU_GYRO_ALPHA` | `kImuAccelAlpha`, `kImuGyroAlpha`. **The default changed from 0.09 to 0.33**: lighter smoothing, less lag. |
| `IMU_*_TRANSIENT_THRESHOLD*` | Gone. There is no transient blending; with the lighter smoothing it isn't needed. |
| `IMU_AXIS_X/Y/Z_SRC`, `IMU_AXIS_X/Y/Z_SIGN` | `kMpu6050AxisSrc` / `kMpu6050AxisSign` on the ESP32, `kLsm6ds3AxisSrc` / `kLsm6ds3AxisSign` on the nRF52840. Same meaning, written as `{X, Y, Z}`. |
| `IMU_TRIM_*` | Not settings any more. The self-calibration behaves the same way. |
| `BLE_TX_POWER` (ESP32), `BLE_TX_POWER_ADV` / `_CONN` (nRF52840) | `kBleTxPowerDbm`: one value, in dBm on both boards. |
| `LOW_BATT_CUTOFF_V`, `LOW_BATT_CRITICAL_V`, `LOW_BATT_WARN_V`, `BATTERY_FULL_V` | `kBatteryCutoffV`, `kBatteryCriticalV`, `kBatteryWarningV`, `kBatteryChargeFullV` |
| `BATTERY_DISCHARGE_CURVE` | Gone. The percentage is a straight line from the cutoff voltage to `kBatteryFullScaleV`. |
| `BATTERY_FAST_CHARGE` | Gone. The charger always runs at the XIAO's higher rate, about 100 mA. |
| `STATE_CHARGE_ONLY_ON_USB` | Gone. See [Power and charging](#power-and-charging) below. |
| `STATE_IDLE_TIMEOUT_MIN` and the other `LIGHT_SLEEP` settings, `IMU_WAKE_*`, `GNSS_WAKE_PULSE_MS` | Gone, along with LIGHT_SLEEP itself. |
| `LOG_ENABLED` | Gone. The console is silent after boot unless you ask it for something. |
| Pin assignments | In each board's `BoardConfig.h`, not in `Config.h`. |

If you had the IMU mounted in a non-default orientation, carry your six axis values over.

> [!IMPORTANT]
> **Classic nRF52840 devices need their axis signs set.** The XIAO faces a different way in the current build than it did in the Classic nRF52840 build, so the default changed. For a device built from the Classic nRF52840 instructions, set `kLsm6ds3AxisSign` to `{-1.0f, -1.0f, +1.0f}` in `Config.h`, which is what Classic used. Without it, forward and left read reversed.

If your IMU smoothing was tuned to taste at Classic's default, set both alphas back to `0.09` to keep the feel you had.

Sensor ranges, filter settings, and timing constants that Classic exposed are now in [`SystemConfig.h`](../code/Gnimu/src/app/SystemConfig.h), away from the settings you're expected to touch.

---

## What behaves differently

### The receiver's settings are saved

Classic wrote its GNSS settings to the receiver's RAM at every boot and saved only the baud rate to flash. The current firmware saves all of them to the receiver's flash and afterwards only checks them. The first boot after migrating takes a little longer, with a run of ⏬ lines in the boot log as the settings are written; later boots show ✅ and are quicker.

If you ever go back to Classic, or move the receiver to another project, [`tools/gnss_reset`](../tools/gnss_reset/gnss_reset.ino) returns it to factory settings.

### The console is quiet

Classic printed a status line every second for as long as it ran. The current firmware prints its boot log and then nothing, until you type a command. `L` and Enter turns the 1Hz status line on. See [Console and diagnostics](console.md).

### A missing receiver is a fault, not a hang

If the GNSS receiver isn't found, Classic stopped dead. The current firmware enters a fault state: the LED blinks red rapidly, the console still answers, and the boot log says what went wrong. Power-cycle after fixing the cause.

### Power and charging

This applies to the nRF52840 build.

**There is no LIGHT_SLEEP.** Classic put the device to sleep after 30 minutes without an app connected, and woke it on movement. The current firmware runs until you switch it off. **Turn the switch off when you're done**, or the battery will run down to the low-voltage cutoff.

**Plugging in USB no longer stops the device.** Classic switched to charge-only as soon as USB was connected. Now a running device keeps running when you plug it in, which is what you want on the bench. To charge with everything else off, so the cell can reach full:

1. With USB connected, slide the switch **off**. The LED blinks amber quickly.
2. Slide it back **on**. The LED blinks green while charging and turns solid green when full.

To go back to running, slide the switch off and on again, or unplug USB.

The low-voltage cutoff works as before: on battery, below the cutoff voltage, the device powers itself off to protect the cell.

### The status LED

On the ESP32 the LED means what it did: blinking while waiting for an app, solid when connected. It now also blinks rapidly on a fault.

On the nRF52840 some colors changed:

| | Classic | Current |
|---|---|---|
| Waiting for an app | Blue, blinking | Blue, blinking |
| App connected | Blue, steady | Blue, steady |
| Battery low | Amber, blinking | Amber, blinking |
| Battery critical | Red, blinking | Red, blinking |
| Switch off on USB (BATTERY_WAIT) | Red, rapid blink | **Amber, quick blink** |
| Fault | n/a | **Red, rapid blink** |
| Charging | Green, whenever on USB | Green, **only in charge-only mode** |
| LIGHT_SLEEP | Blue, short pulse | n/a |

Rapid red used to mean "check the switch". It now means something is wrong; look at the boot log.

---

## Diagnostic sketches

The three GNSS utilities moved to the top-level [`tools/`](../tools/) folder and are maintained: `gnss_ver`, `gnss_reset`, and `gnss_otp_clock`. If your receiver's clock was already programmed with `gnss_otp_clock`, that stays done; it lives in the receiver, not the firmware.

Classic's other bench sketches (IMU calibration and probing, battery logging, LED and BLE checks) are still in [`classic/src/tools/`](../classic/src/tools/). They were written against Classic's wiring and settings and aren't needed to build or run the current firmware.
