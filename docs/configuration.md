# Configuration

The firmware works as-is on the supported hardware; there is nothing you have to set before the first build. When you do want to change how it behaves, the settings are in one file: [`code/Gnimu/Config.h`](../code/Gnimu/Config.h). In the Arduino IDE it opens as a tab next to the sketch.

Every setting is checked when the firmware compiles. A value that's out of range, or a combination that can't work, **fails the build with a message saying why**, so a bad setting never reaches the device.

After changing a setting, rebuild and upload. See [Building and flashing](building.md).

To see the settings a device is actually running, type `V` in the console. It prints them as [`Config:` lines](console.md#what-the-device-is), which is the quickest way to confirm that the firmware you flashed is the one you edited.

---

## GNSS

| Setting | Default | What it does |
|---|---|---|
| `kGnssNavRateHz` | `20` | Position updates per second, 1 to 25. |
| `kConstellationCount` | `2` | How many satellite constellations to use, 1 to 4. |
| `kConstellationPriority` | GPS, Galileo, BeiDou, GLONASS | The order of preference. The first `kConstellationCount` in the list are enabled; the rest are turned off. |
| `kGnssMinElevDeg` | `5` | Satellites lower than this above the horizon are ignored, which reduces multipath error. 0 to 90 degrees. |

The firmware saves these to the receiver's own flash the first time it boots with them, and from then on only checks them. Changing one here and re-flashing is all it takes; the next boot rewrites just the settings that differ.

### Fix rate and enabled constellations

The update rate an M10 receiver can sustain depends on how many constellations it is tracking:

| Constellations enabled | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| High CPU clock | 25 Hz | 20 Hz | 16 Hz | 10 Hz |
| Default CPU clock | 18 Hz | 10 Hz | 10 Hz | 5 Hz |

So the two practical choices are **20 Hz with two constellations** (the default, GPS and Galileo) or **25 Hz with one**. For 25 Hz, set `kGnssNavRateHz` to `25` and `kConstellationCount` to `1`.

Those figures are the top row. The HGLRC modules ship at the default clock, the bottom row, and need a one-time, permanent change to reach the top one. The firmware will ask an unmodified module for 20 Hz and the module will accept the setting, then drop fixes once it's tracking more than about 10 to 12 satellites. See the notes on GNSS modules in the [README](../README.md#three-notes-about-gnss-modules), and run [`tools/gnss_ver`](../tools/gnss_ver/gnss_ver.ino) to see which row your module is on.

QZSS and SBAS are always off. They are augmentation systems rather than constellations, and the firmware doesn't use them.

---

## BLE

| Setting | Default | What it does |
|---|---|---|
| `kBleTxPowerDbm` | `-12` | Radio transmit power in dBm. |

The default is low on purpose. The receiver is normally within a couple of meters, and less RF energy next to the GNSS antenna is better for the fix. Raise it if connections drop.

Each board accepts only the levels its radio supports:

- **ESP32:** -12 to +9 dBm, in 3 dB steps.
- **XIAO nRF52840:** -40, -20, -16, -12, -8, -4, 0, or 2 to 8 dBm.

The advertised BLE name is `kBleNamePrefix` followed by a 10-digit ID derived from the chip's hardware ID, so every device has a stable name of its own from the same firmware image. The prefix is also the Device Information model string.

| Setting | Default | What it does |
|---|---|---|
| `kBleNamePrefix` | `"Gnimu-RB"` | The advertised-name prefix, up to 20 characters. Some apps pick devices out of a scan by the name they expect, so change it if yours does. |
| `kBleManufacturer` | `"Gnimu"` | The Device Information manufacturer string. |

---

## IMU

### Smoothing

| Setting | Default | What it does |
|---|---|---|
| `kImuAccelAlpha` | `0.33` | Accelerometer smoothing. |
| `kImuGyroAlpha` | `0.33` | Gyroscope smoothing. |

The IMU is sampled at 100 Hz and smoothed before it's sent. Alpha of `1.0` is the raw signal with no smoothing; smaller values smooth more, and lag more. The lag is about (1 − alpha) / alpha samples:

| Alpha | Lag | Character |
|---|---|---|
| `0.33` | about 20 ms | Close to the GNSS update it's sent with, while still damping engine and road vibration. |
| `0.18` | about 45 ms | Calmer traces. |
| `0.09` | about 100 ms | Very calm, but braking and turn-in points shift by about 3 m at 30 m/s. |

### Mounting orientation

Which way the IMU sits in the vehicle depends on how the device was built, so it belongs to the build's [hardware profile](#hardware-profiles), not to this file:

| Profile | Mounting it describes | Axis map |
|---|---|---|
| ESP32 `reference` | A GY-521 (MPU-6050) aligned to the axis marks on its silkscreen: mounting holes toward the front of the car, header pins toward the rear, PCB face up. | `kMpu6050AxisSrc` `{0, 1, 2}`, `kMpu6050AxisSign` `{+1, +1, +1}` |
| nRF52840 `v2` | The XIAO's USB connector toward the rear of the car, as in the [XIAO nRF52840 build guide](build-nrf52840.md). | `kLsm6ds3AxisSrc` `{0, 1, 2}`, `kLsm6ds3AxisSign` `{+1, +1, +1}` |
| nRF52840 `classic` | The XIAO facing the other way, as in the Classic nRF52840 build. | `kLsm6ds3AxisSrc` `{0, 1, 2}`, `kLsm6ds3AxisSign` `{-1, -1, +1}` |

The map takes the sensor's axes onto the vehicle's. Output axis *i* is sensor axis `Src[i]` multiplied by `Sign[i]`. `Src` must use each of 0, 1 and 2 exactly once, which is checked when the firmware compiles.

If you mounted the sensor differently from the build guide, make [a profile of your own](#adding-your-own-profile) with the right map. If an axis reads backwards in your app, flip its sign; if two are swapped, swap their `Src` entries.

This is for getting the axes pointing the right way. Small tilts, up to about 15°, are measured and corrected automatically while the car is parked; see the mounting note in the [README](../README.md#a-note-about-mounting-and-self-calibration).

---

## Battery

These apply to the XIAO nRF52840, the board with a battery.

| Setting | Default | What it does |
|---|---|---|
| `kBatteryFullScaleV` | `4.10` | The voltage reported as 100%. |
| `kBatteryWarningV` | `3.40` | Below this the status LED blinks amber. |
| `kBatteryCriticalV` | `3.35` | Below this the status LED blinks red. |
| `kBatteryCutoffV` | `3.30` | The voltage reported as 0%. Running on battery below this for five seconds powers the device off to protect the cell. |
| `kBatteryChargeFullV` | `4.10` | With the charger no longer charging, at or above this the cell counts as full. |

The percentage is a straight line from the cutoff (0%) to full scale (100%). The four thresholds must stay in order: cutoff below critical, below warning, below full scale, and nothing above 4.2 V.

---

## Build variants

Some choices are made when the firmware is built, by naming a *profile*: a small header file in [`code/Gnimu/src/variants/`](../code/Gnimu/src/variants/). Choosing a profile modifies no file, so one tagged release can produce every combination and each build reports `dirty=0`.

| Choice | Option | Profiles (default first) |
|---|---|---|
| Hardware | `--hardware <name>` | nRF52840: `v2`, `classic`. ESP32: `reference`. |
| Protocol | `--protocol <name>` | `racebox` |

```bash
scripts/build.sh nrf52840 --hardware classic
```

See [Building and flashing](building.md#command-line). A profile's name is its file name, so adding a profile adds an option value. An unknown name is refused with a list of the ones that exist. `scripts/build.sh --matrix` builds every combination.

### Hardware profiles

A hardware profile holds what depends on how the device was built: the pins you are free to route differently, and which way the IMU is mounted. Pins the board fixes for you (the XIAO's RGB LED, battery sense, charger and IMU power, and the GNSS UART on D6/D7) are not in profiles.

| | nRF52840 `v2` | nRF52840 `classic` | ESP32 `reference` |
|---|---|---|---|
| GNSS regulator enable | D1 | D9 | not switched |
| Power-switch sense | A0 | A4 | not present |
| GNSS UART RX, TX | fixed | fixed | GPIO16, GPIO17 |
| IMU I²C SDA, SCL | fixed | fixed | GPIO21, GPIO22 |
| Status LED | fixed (RGB) | fixed (RGB) | GPIO2 |
| IMU axis map | identity | X and Y reversed | identity |

- **`v2`** is the current pinout, used by the carrier PCB and by the [build guide](build-nrf52840.md).
- **`classic`** matches a device built from the Classic nRF52840 instructions, so existing hardware runs this firmware without rewiring. See [Migrating from Classic](migrating-from-classic.md).
- **`reference`** is the [ESP32 build guide](build-esp32.md)'s wiring and mounting.

The console's `V` command shows which profile a device was built with, as `hardware` on the `Config: board` line.

### Adding your own profile

If your wiring or IMU mounting differs from the build guide, make a profile for it instead of editing a shared file:

1. Copy the closest profile in `code/Gnimu/src/variants/<board>/hardware/`, for example `reference.h` to `mine.h`.
2. Change the values you need, and give `kHardwareName` a name for `V` to show.
3. Build with `scripts/build.sh <board> --hardware mine`.

Keep every constant the copied file defines. A missing one fails the build with an error naming it, and an axis map that isn't a permutation fails with a message saying so. Your new file shows as untracked in `git status`, so a build that includes it reports `dirty=1`, which is accurate: the firmware isn't a clean copy of the repository.

### Protocols

The RaceBox data protocol is the only one available today. [RaceChrono DIY](https://github.com/aollin/racechrono-ble-diy-device) is planned.

### Arduino IDE

The IDE can't pass options, so it builds the defaults. To build another profile there, change the default: the `GNIMU_HARDWARE_DEFAULT` line in the board's `BoardConfig.h` (for example `"nrf52840/hardware/classic.h"`), or `GNIMU_PROTOCOL_FILE` in `Config.h`. Or edit the default profile itself. Edits like these show as `dirty=1` in a command-line build, but an IDE build doesn't record that either way.

---

## Everything else

The constants the firmware's timing and safety depend on are kept apart, in [`code/Gnimu/src/app/SystemConfig.h`](../code/Gnimu/src/app/SystemConfig.h): job rates and time budgets, the watchdog, debounce times, LED blink timing, IMU ranges, and the self-calibration thresholds. They are tuned against each other and against measurements on real hardware, each one has a comment explaining its value, and the same file holds the compile-time checks.

You shouldn't need to change them. One is worth knowing about for bench work: `kImuTrimRequireFix`. The self-calibration normally waits for a 3D fix, which never arrives indoors; setting it to `false` lets you watch the calibration work on a desk. Set it back before using the device in a car.
