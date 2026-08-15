# XIAO tools

Standalone sketches to validate assumptions the Gnimu nRF52840 firmware makes about
the **Seeed XIAO nRF52840 Sense**, before the full sketch is compiled or the rest
of the hardware is wired. Each is self-contained (no dependency on the main
`config.h`) and lives in its own folder so the Arduino IDE can open it directly.

Board package: **Seeed nRF52 Boards** (the non-mbed, Adafruit-nRF52-based core —
required for Bluefruit; do **not** use "Seeed nRF52 mbed-enabled Boards"). Board:
"Seeed XIAO nRF52840 Sense". Boards Manager URL:
`https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`.
Libraries: **Seeed Arduino LSM6DS3** (for `imu_probe`); Bluefruit ships with the
core. Open the serial monitor at **115200**.

| Sketch | Validates | Extra parts | Pass criteria |
|---|---|---|---|
| [`imu_probe/`](imu_probe/imu_probe.ino) | Onboard LSM6DS3TR-C power pin, library bring-up, units (`g_imu.cpp`, DESIGN §4) | none | `begin() OK`; resting board reads ~+1 g on one accel axis (total ~1 g, not ~9.8) and ~0 dps gyro. *(Confirmed — see DESIGN Open items.)* |
| [`imu_tiltmap/`](imu_tiltmap/imu_tiltmap.ino) | Maps LSM6DS3 sensor axes to the board (fills `config.h`'s `IMU_SIGN_*` / `IMU_SWAP_XY`, or `IMU_AXIS_*_SRC`/`_SIGN` on the OLED tree — see note below) | none | Flat + component-up prints `UP = +Z`; each edge-down pose names the in-plane axis. |
| [`imu_calibration/`](imu_calibration/imu_calibration.ino) | Per-axis IMU zero-point offsets in the raw sensor frame, independent of the axis remap however it's spelled (feeds `config.h`'s `IMU_ACCEL_OFFSET_*`/`IMU_GYRO_OFFSET_*`). **Base tree's copy** — the OLED tree has [its own](../nRF52840-OLED/imu_calibration/imu_calibration.ino) | level bench surface | Unattended, no USB needed: warms up until die temp plateaus (5–20 min), then repeating 10000-sample sessions 1 min apart, each gated on a stability check and appended to internal flash. Press any key over Serial to halt, then `a` to aggregate the run into six paste-ready `IMU_*_OFFSET_*` `#define` lines. |
| [`imu_wake/`](imu_wake/imu_wake.ino) | LSM6DS3TR-C's embedded wake-up (activity) detector register config (`CTRL1_XL`, `WAKE_UP_THS`, `WAKE_UP_DUR`) used by LIGHT_SLEEP's shake-to-wake exit trigger | none | Threshold/debounce tuned so a real pickup/shake reliably fires without false-triggering from bench vibration or handling. *(Bench-tuned — see DESIGN §5.)* |
| [`led_check/`](led_check/led_check.ino) | RGB LED pins + active-LOW polarity + status colors (`g_led.cpp`) | none | The LED color matches each name printed over serial; OFF goes fully dark. *(Confirmed.)* |
| [`ble_mtu/`](ble_mtu/ble_mtu.ino) | Advertising name, TX power, MTU ≥ 91, `BLEUart` 88-byte notify (`g_ble.cpp`) | phone w/ nRF Connect | Advertises as `RaceBox Mini <id>`; "Negotiated MTU" line reports ≥ 91; the 88-byte test notify is received. *(Confirmed — MTU 23→247.)* |
| [`gnss_en/`](gnss_en/gnss_en.ino) | GNSS rail EN-pad cutoff: does D9 LOW actually disconnect the TPS63020 output? (`g_battery.cpp` low-voltage cutoff + planned GNSS idle-cutoff) | TPS63020 + GNSS wired, multimeter | In the LOW state the TPS63020 output drops to ~0 V **and** GNSS UART goes silent (0 bytes). *(Confirmed — with a caveat, see below.)* |
| [`gnss_pmreq/`](gnss_pmreq/gnss_pmreq.ino) | UBX-RXM-PMREQ GNSS backup mode for LIGHT_SLEEP's GNSS power ladder: does UART traffic stop while "asleep", does it wake on UART RX activity, does ephemeris survive the sleep | GNSS wired, clear sky view, optional multimeter | UART goes silent while asleep; wakes on RX activity; post-wake TTFF is a HOT start (~1–3 s), confirming ephemeris survived rather than falling back to a ~20–30 s cold-start-style reacquire. *(Confirmed — validated on hardware, shipped in `g_gnss.cpp`.)* |
| [`battery_presence/`](battery_presence/battery_presence.ino) | Switch-sense divider tap (A1) — the authoritative battery-present signal in `g_power.cpp` — plus the non-blocking VBAT sampler and divider-recovery math that feeds `g_battery`'s state-of-charge fuel gauge | LiPo, multimeter, USB | Self-check mV matches the meter on the cell; switch-sense reads ~2 V OFF / ~0 V ON, powered, matching the meter. |
| [`battery_log/`](battery_log/battery_log.ino) | The LiPo's true resting voltage at full charge, for `BATTERY_DISCHARGE_CURVE`'s 100% anchor — VBAT logged to internal flash through a full plug-in → charge → unplug → settle cycle (survives a Serial disconnect mid-run) | LiPo, USB | Log flags `CHARGE_PLATEAU` near full charge and `SETTLED` after unplug; the settled reading is the value to paste into `BATTERY_DISCHARGE_CURVE`. |
| [`storage_check/`](storage_check/storage_check.ino) | QSPI flash + LittleFS stack in isolation (chip detection, mount, format, read/write/delete) on the XIAO Sense's Puya P25Q16H chip | none | Phase 1 (read-only) reports the chip correctly; Phase 2 formats only after typing `FORMAT`, then a read/write/delete round-trip succeeds. Not currently used by the main firmware — kept as a standalone diagnostic for the flash chip itself. |

> **`imu_calibration` is per-tree.** Since 2026-08-14 the OLED variant carries
> [its own copy](../nRF52840-OLED/imu_calibration/imu_calibration.ino) rather
> than this one taking a build flag. The measurement core is byte-identical, so
> results are comparable; each copy just hardcodes its own variant's settings
> (the panel as thermal load and USB-free readout there, and a different BLE
> advertising power). **A change to the measurement logic must be applied to
> both.** The ESP32 tree has a
> [third copy](../ESP32/imu_calibration/imu_calibration.ino), same approach in
> that sensor's native units.

> **`imu_tiltmap` is single-sourced here on purpose, and is usually not the
> tool you want.** The production firmware already prints the 1 Hz serial
> `milliG` line, and the three static poses documented in `config.h`'s axis
> section fully determine the map from it — which is how the base tree's
> as-built map was actually settled, in preference to a drive test. Reach for
> this sketch when a board's sensor orientation is unknown from scratch;
> otherwise just run the firmware.
>
> ⚠️ **Derive against the raw serial `milliG` numbers, not the Gnimu Monitor
> readout.** Monitor is a display layer that has been wrong about exactly this
> before, masking a mirrored (determinant −1) axis map. It cannot validate
> firmware signs. See [`Gnimu-nRF52840/DESIGN.md`](../../Gnimu-nRF52840/DESIGN.md) §4.
>
> **Axis-macro note.** This sketch prints results in terms of `IMU_SWAP_XY` /
> `IMU_SIGN_*`, which is what the base nRF52840 and ESP32 trees use. The
> **nRF52840-OLED** tree has replaced those with `IMU_AXIS_{X,Y,Z}_SRC` /
> `_SIGN` (all 24 orientations rather than 8). The bench poses are identical
> either way — only the macros you write differ. The old→new migration table is
> in [`Gnimu-nRF52840-OLED/DESIGN.md`](../../Gnimu-nRF52840-OLED/DESIGN.md) §6.

## What each result feeds back into the firmware

- **imu_probe** — RESOLVED: the stock Seeed library reaches the IMU with no
  manual bus setup, `PIN_LSM6DS3TR_C_POWER` (pin 15) HIGH enables it, and it
  returns g / deg/s directly (validating `g_imu.cpp`'s simplified unit
  conversion). `g_imu.cpp` was updated to drop the unneeded `Wire1.begin()`.
- **led_check** — RESOLVED: OFF goes fully dark and every color matches, so
  `LED_ACTIVE_LOW = 1` and the `LED_*_PIN` mapping used by `g_led.cpp`'s
  `setLed()` are correct as-is.
- **imu_calibration** — AVAILABLE: produces per-board `IMU_ACCEL_OFFSET_*` /
  `IMU_GYRO_OFFSET_*` values for `config.h`. Defaults are all `0.0f`, so a
  board that hasn't been run through this sketch behaves identically to
  before — running it is an optional per-board tuning step, not a
  correctness gate. Runs standalone off the slide switch with no USB
  attached: results go to internal flash (`/imu_cal.csv`, ~10–11 h of
  sessions before the ~22 KB budget fills), and each boot appends a new run
  rather than overwriting. GNSS and BLE are deliberately left running so the
  die reaches a temperature representative of production rather than of an
  idle board. **Caveat on the accel offsets:** one degree of bench tilt leaks
  ~17 mg into the horizontal axes — several times the bias being measured —
  so accel X/Y is as much a levelness measurement as a sensor one, and
  averaging cannot separate them (accel Z is hit too, by the smaller cosine
  error). Only the **gyro** offsets are immune to pose. Tilt therefore does
  not block calibration: the sketch records it per session and, if the run's
  median tilt exceeds ~1°, comments the accel `#define`s out of the
  paste-ready block while still emitting the gyro ones. Separating accel X/Y
  properly needs a multi-position tumble calibration, which this hands-off
  tool deliberately does not attempt. **On the nRF52840-OLED board set
  `CAL_DISPLAY_ENABLED` to 1** — that variant runs its panel continuously, so
  it belongs in the thermal load, and it turns the sketch into a standalone
  readout (phase, die temp, latest offsets) needing no USB at all. Leave it 0
  for the plain build, where A4 is `POWER_SWITCH_SENSE_PIN` rather than SDA.
- **imu_wake** — bench-tuned the `IMU_WAKE_THS` / `IMU_WAKE_DUR` values that
  `g_imu.cpp`'s `imuArmWake()` (called from `g_state.cpp` on the RUNNING →
  LIGHT_SLEEP transition) writes to the LSM6DS3TR-C. The accelerometer's
  full-scale bits (`FS_XL`) must stay identical to `IMU_ACCEL_RANGE_G` across
  that transition — only the ODR bits should differ — or the wake-up detector
  false-triggers immediately.
- **ble_mtu** — RESOLVED: `configPrphBandwidth(BANDWIDTH_MAX)` + central-driven
  negotiation reach MTU 247 (23→247 on the central's request), so an 88-byte
  RaceBox Data Message rides in one notify and `BLEUart` (Nordic UART = RaceBox
  UUIDs) transports it. Do **not** peripheral-initiate `requestMtuExchange`.
- **gnss_en** — RESOLVED (2026-07): EN-low truly disconnects the TPS63020
  output — no load switch needed — but the GNSS stays phantom-powered through
  its RX pin if XIAO TX (D6) idles HIGH. `g_power.cpp`'s
  `powerHoldPeripheralsOff()` / `powerEnterDeepSleep()` therefore also
  `Serial1.end()` and drive D6 LOW to fully cut it.
- **gnss_pmreq** — RESOLVED: proved out RXM-PMREQ backup mode and its wake
  mechanism, now shipped in `g_gnss.cpp`'s `gnssSleep()` / `gnssWake()`. The
  actual wake is a deliberate GPIO-level pulse on the shared UART TX line —
  the "send any UART byte" wake suggested by the GNSS library's own docs
  reliably failed on the bench.
- **battery_presence** — RESOLVED (2026-07): bench-confirmed the switch-sense
  divider (A1) reads ~0.00 V ON / ~2.05 V OFF, powered, matching theory — this
  is now the sole battery-presence signal (`powerSwitchOn()`), replacing an
  earlier VBAT-based floor/variance approach that couldn't distinguish a
  charger-in-CV feeding a load from a battery-in-CV feeding a load. Confirmed
  peak-for-SoC and the `analogSampleTime(40)` TACQ fix along the way (both
  shipped in `g_battery.cpp`). Readings taken with the chip unpowered are not
  reliable (ESD-diode loading on a dead pin) — always test powered.
- **battery_log** — a data-collection tool rather than a pass/fail validator:
  its output is the empirical 100% voltage anchor for
  `BATTERY_DISCHARGE_CURVE`, measured with the same sampling method
  `g_battery.cpp` uses so the number is directly comparable.
- **storage_check** — the QSPI/LittleFS feature it provisions for was built
  and bench-validated, then later pruned from the main firmware as not
  worth the added complexity for the warm-start improvement it bought. This
  sketch remains as a standalone diagnostic for the flash chip itself,
  independent of current firmware state.

The constants in `ble_mtu` mirror `config.h` (`BLE_TX_POWER_ADV`, `MODEL` +
`DEVICE_ID`); `gnss_en` and `gnss_pmreq` mirror `GNSS_EN_PIN` (D9) and
`GNSS_BAUD`; `imu_calibration` mirrors the IMU range/ODR/bandwidth block
(`IMU_ACCEL_RANGE_G`, `IMU_GYRO_RANGE_DPS`, `IMU_*_ODR_HZ`,
`IMU_ACCEL_BANDWIDTH_HZ`, `IMU_SAMPLE_INTERVAL_MS`) plus `GNSS_EN_PIN` /
`GNSS_BAUD` / `BLE_TX_POWER_ADV_DBM` for its thermal load; `imu_wake` mirrors
`IMU_WAKE_CTRL1_XL`, `IMU_WAKE_THS`, and
`IMU_WAKE_DUR`; `battery_presence` mirrors the VBAT ADC/divider constants
(`BATTERY_ADC_PIN`, `VBAT_ENABLE`, `ADC_REFERENCE_MV`, the divider ratio) plus
`SWITCH_SENSE_PIN` (A1) / `SWITCH_OFF_THRESHOLD_MV`. Keep them in sync if you
change `config.h`.

> Not covered here (need extra parts): the `Serial1` D6↔D7 UART loopback (needs a
> jumper). See the `[bench-verify]` notes in `g_gnss.cpp` when you get to that.
