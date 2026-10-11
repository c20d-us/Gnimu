# Configuration

The firmware works as-is on the supported hardware; there is nothing you have to set before the first build. When you do want to change how it behaves, the settings are in one file: [`code/Gnimu/Config.h`](../code/Gnimu/Config.h). In the Arduino IDE it opens as a tab next to the sketch.

Every setting is checked when the firmware compiles. A value that's out of range, or a combination that can't work, **fails the build with a message saying why**, so a bad setting never reaches the device.

After changing a setting, rebuild and upload. See [Building and flashing](building.md).

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

The BLE name is not a setting. It is `RaceBox Mini` followed by a 10-digit ID derived from the chip's hardware ID, so every device has a stable name of its own from the same firmware image.

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

| Setting | Default | Applies to |
|---|---|---|
| `kMpu6050AxisSrc`, `kMpu6050AxisSign` | `{0, 1, 2}`, `{+1, +1, +1}` | ESP32, external MPU-6050 |
| `kLsm6ds3AxisSrc`, `kLsm6ds3AxisSign` | `{0, 1, 2}`, `{+1, +1, +1}` | XIAO nRF52840 Sense, onboard LSM6DS3TR-C |

These map the sensor's axes onto the vehicle's. Output axis *i* is sensor axis `Src[i]` multiplied by `Sign[i]`. `Src` must use each of 0, 1 and 2 exactly once.

The XIAO's sensor is soldered to the board, so its mapping follows which way the XIAO faces. The default is for the XIAO's USB connector pointing toward the rear of the car, as in the [XIAO nRF52840 build guide](build-nrf52840.md). On the ESP32 the MPU-6050 is a separate module, and you may have mounted it differently than the [ESP32 build guide](build-esp32.md) shows. If an axis reads backwards in your app, flip its sign; if two are swapped, swap their `Src` entries.

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

## Protocol

| Setting | Default | What it does |
|---|---|---|
| `kProtocol` | `Protocol::RaceBox` | Which BLE protocol the firmware speaks. |

RaceBox is the only protocol available today.

---

## Board wiring (XIAO nRF52840)

One hardware setting lives outside `Config.h`, in [`code/Gnimu/src/platform/nrf52840/BoardConfig.h`](../code/Gnimu/src/platform/nrf52840/BoardConfig.h):

| Setting | Default | What it does |
|---|---|---|
| `kWiring` | `Wiring::V2` | Which pins the GNSS power control and the power-switch sense are wired to. |

| | `Wiring::V2` | `Wiring::Classic` |
|---|---|---|
| GNSS regulator enable | D1 | D9 |
| Power-switch sense | A0 | A4 |

`Wiring::V2` is the current pinout, used by the carrier PCB and by the [build guide](build-nrf52840.md). `Wiring::Classic` matches a device built from the Classic nRF52840 instructions, so existing hardware can run this firmware without rewiring. See [Migrating from Classic](migrating-from-classic.md).

---

## Everything else

The constants the firmware's timing and safety depend on are kept apart, in [`code/Gnimu/src/app/SystemConfig.h`](../code/Gnimu/src/app/SystemConfig.h): job rates and time budgets, the watchdog, debounce times, LED blink timing, IMU ranges, and the self-calibration thresholds. They are tuned against each other and against measurements on real hardware, each one has a comment explaining its value, and the same file holds the compile-time checks.

You shouldn't need to change them. One is worth knowing about for bench work: `kImuTrimRequireFix`. The self-calibration normally waits for a 3D fix, which never arrives indoors; setting it to `false` lets you watch the calibration work on a desk. Set it back before using the device in a car.
