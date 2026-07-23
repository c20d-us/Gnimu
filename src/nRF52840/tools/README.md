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
| [`imu_probe/`](imu_probe/imu_probe.ino) | Onboard LSM6DS3TR-C power pin, library bring-up, units (`gp_imu.cpp`, DESIGN §4) | none | `begin() OK`; resting board reads ~+1 g on one accel axis (total ~1 g, not ~9.8) and ~0 dps gyro. *(Confirmed — see DESIGN Open items.)* |
| [`imu_tiltmap/`](imu_tiltmap/imu_tiltmap.ino) | Maps LSM6DS3 sensor axes to the board (fills `gp_imu.cpp` `IMU_SIGN_*` / `IMU_SWAP_XY`) | none | Flat + component-up prints `UP = +Z`; each edge-down pose names the in-plane axis. |
| [`imu_calibration/`](imu_calibration/imu_calibration.ino) | Per-axis IMU zero-point offsets in the raw sensor frame, independent of `IMU_SWAP_XY`/`IMU_SIGN_*` (feeds `config.h`'s `IMU_ACCEL_OFFSET_*`/`IMU_GYRO_OFFSET_*`) | level bench surface | Device held still through a 5-minute warmup + 5000-sample average; prints six paste-ready `IMU_*_OFFSET_*` `#define` lines. |
| [`imu_wake/`](imu_wake/imu_wake.ino) | LSM6DS3TR-C's embedded wake-up (activity) detector register config (`CTRL1_XL`, `WAKE_UP_THS`, `WAKE_UP_DUR`) used by LIGHT_SLEEP's shake-to-wake exit trigger | none | Threshold/debounce tuned so a real pickup/shake reliably fires without false-triggering from bench vibration or handling. *(Bench-tuned — see DESIGN §5.)* |
| [`led_check/`](led_check/led_check.ino) | RGB LED pins + active-LOW polarity + status colors (`gp_led.cpp`) | none | The LED color matches each name printed over serial; OFF goes fully dark. *(Confirmed.)* |
| [`ble_mtu/`](ble_mtu/ble_mtu.ino) | Advertising name, TX power, MTU ≥ 91, `BLEUart` 88-byte notify (`gp_ble.cpp`) | phone w/ nRF Connect | Advertises as `RaceBox Mini <id>`; "Negotiated MTU" line reports ≥ 91; the 88-byte test notify is received. *(Confirmed — MTU 23→247.)* |
| [`gnss_en/`](gnss_en/gnss_en.ino) | GNSS rail EN-pad cutoff: does D8 LOW actually disconnect the TPS63020 output? (`gp_battery.cpp` low-voltage cutoff + planned GNSS idle-cutoff) | TPS63020 + GNSS wired, multimeter | In the LOW state the TPS63020 output drops to ~0 V **and** GNSS UART goes silent (0 bytes). *(Confirmed — with a caveat, see below.)* |
| [`gnss_pmreq/`](gnss_pmreq/gnss_pmreq.ino) | UBX-RXM-PMREQ GNSS backup mode for LIGHT_SLEEP's GNSS power ladder: does UART traffic stop while "asleep", does it wake on UART RX activity, does ephemeris survive the sleep | GNSS wired, clear sky view, optional multimeter | UART goes silent while asleep; wakes on RX activity; post-wake TTFF is a HOT start (~1–3 s), confirming ephemeris survived rather than falling back to a ~20–30 s cold-start-style reacquire. *(Confirmed — validated on hardware, shipped in `gp_gnss.cpp`.)* |
| [`battery_presence/`](battery_presence/battery_presence.ino) | Switch-sense divider tap (A1) — the authoritative battery-present signal in `gp_power.cpp` — plus the non-blocking VBAT sampler and divider-recovery math that feeds `gp_battery`'s state-of-charge fuel gauge | LiPo, multimeter, USB | Self-check mV matches the meter on the cell; switch-sense reads ~2 V OFF / ~0 V ON, powered, matching the meter. |
| [`battery_log/`](battery_log/battery_log.ino) | The LiPo's true resting voltage at full charge, for `BATTERY_DISCHARGE_CURVE`'s 100% anchor — VBAT logged to internal flash through a full plug-in → charge → unplug → settle cycle (survives a Serial disconnect mid-run) | LiPo, USB | Log flags `CHARGE_PLATEAU` near full charge and `SETTLED` after unplug; the settled reading is the value to paste into `BATTERY_DISCHARGE_CURVE`. |
| [`storage_check/`](storage_check/storage_check.ino) | QSPI flash + LittleFS stack in isolation (chip detection, mount, format, read/write/delete) on the XIAO Sense's Puya P25Q16H chip | none | Phase 1 (read-only) reports the chip correctly; Phase 2 formats only after typing `FORMAT`, then a read/write/delete round-trip succeeds. Not currently used by the main firmware — kept as a standalone diagnostic for the flash chip itself. |

## What each result feeds back into the firmware

- **imu_probe** — RESOLVED: the stock Seeed library reaches the IMU with no
  manual bus setup, `PIN_LSM6DS3TR_C_POWER` (pin 15) HIGH enables it, and it
  returns g / deg/s directly (validating `gp_imu.cpp`'s simplified unit
  conversion). `gp_imu.cpp` was updated to drop the unneeded `Wire1.begin()`.
- **led_check** — RESOLVED: OFF goes fully dark and every color matches, so
  `LED_ACTIVE_LOW = 1` and the `LED_*_PIN` mapping used by `gp_led.cpp`'s
  `setLed()` are correct as-is.
- **imu_calibration** — AVAILABLE: prints per-board `IMU_ACCEL_OFFSET_*` /
  `IMU_GYRO_OFFSET_*` values for `config.h`. Defaults are all `0.0f`, so a
  board that hasn't been run through this sketch behaves identically to
  before — running it is an optional per-board tuning step, not a
  correctness gate.
- **imu_wake** — bench-tuned the `IMU_WAKE_THS` / `IMU_WAKE_DUR` values that
  `gp_imu.cpp`'s `imuArmWake()` (called from `gp_state.cpp` on the RUNNING →
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
  its RX pin if XIAO TX (D6) idles HIGH. `gp_power.cpp`'s
  `powerHoldPeripheralsOff()` / `powerEnterDeepSleep()` therefore also
  `Serial1.end()` and drive D6 LOW to fully cut it.
- **gnss_pmreq** — RESOLVED: proved out RXM-PMREQ backup mode and its wake
  mechanism, now shipped in `gp_gnss.cpp`'s `gnssSleep()` / `gnssWake()`. The
  actual wake is a deliberate GPIO-level pulse on the shared UART TX line —
  the "send any UART byte" wake suggested by the GNSS library's own docs
  reliably failed on the bench.
- **battery_presence** — RESOLVED (2026-07): bench-confirmed the switch-sense
  divider (A1) reads ~0.00 V ON / ~2.05 V OFF, powered, matching theory — this
  is now the sole battery-presence signal (`powerSwitchOn()`), replacing an
  earlier VBAT-based floor/variance approach that couldn't distinguish a
  charger-in-CV feeding a load from a battery-in-CV feeding a load. Confirmed
  peak-for-SoC and the `analogSampleTime(40)` TACQ fix along the way (both
  shipped in `gp_battery.cpp`). Readings taken with the chip unpowered are not
  reliable (ESD-diode loading on a dead pin) — always test powered.
- **battery_log** — a data-collection tool rather than a pass/fail validator:
  its output is the empirical 100% voltage anchor for
  `BATTERY_DISCHARGE_CURVE`, measured with the same sampling method
  `gp_battery.cpp` uses so the number is directly comparable.
- **storage_check** — the QSPI/LittleFS feature it provisions for was built
  and bench-validated, then later pruned from the main firmware as not
  worth the added complexity for the warm-start improvement it bought. This
  sketch remains as a standalone diagnostic for the flash chip itself,
  independent of current firmware state.

The constants in `ble_mtu` mirror `config.h` (`BLE_TX_POWER_ADV`, `MODEL` +
`DEVICE_ID`); `gnss_en` and `gnss_pmreq` mirror `GNSS_EN_PIN` (D8) and
`GNSS_BAUD`; `imu_wake` mirrors `IMU_WAKE_CTRL1_XL`, `IMU_WAKE_THS`, and
`IMU_WAKE_DUR`; `battery_presence` mirrors the VBAT ADC/divider constants
(`BATTERY_ADC_PIN`, `VBAT_ENABLE`, `ADC_REFERENCE_MV`, the divider ratio) plus
`SWITCH_SENSE_PIN` (A1) / `SWITCH_OFF_THRESHOLD_MV`. Keep them in sync if you
change `config.h`.

> Not covered here (need extra parts): the `Serial1` D6↔D7 UART loopback (needs a
> jumper). See the `[bench-verify]` notes in `gp_gnss.cpp` when you get to that.
