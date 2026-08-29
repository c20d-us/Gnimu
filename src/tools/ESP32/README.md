# ESP32 tools

Standalone sketches for bringing up or bench-tuning parts of the ESP32 build
outside the main firmware. Each lives in its own folder so the Arduino IDE can
open it directly.

| Sketch | Purpose | Pass criteria |
|---|---|---|
| [`imu_calibration/`](imu_calibration/imu_calibration.ino) | Per-axis MPU-6050 zero-point offsets in the raw sensor frame, independent of the `IMU_AXIS_*` remap (feeds `config.h`'s `IMU_ACCEL_OFFSET_*_MPS2` / `IMU_GYRO_OFFSET_*_RADPS`). Warms up until die temp plateaus (5–20 min), then repeating 10000-sample sessions 1 min apart, each gated on a stability check and appended to LittleFS. Press any key over Serial to halt, then `a` to aggregate the run into six paste-ready `#define` lines. | Gate passes on a still bench; `a` reports a median and 20% trimmed mean that agree within 1 sd, and a median tilt under 1°. |

> **Also usable here:** [`tools/common/gnss_ver`](../common/gnss_ver/gnss_ver.ino)
> — GNSS identity and high-rate capability report (which M10 part, which
> firmware, whether the receiver is running the high CPU clock the 25Hz/20Hz
> figures require, what the current rate/constellation config is rated for, and
> an inventory of every message with a non-zero `CFG-MSGOUT` rate, and a
> measured fix rate with an `iTOW` gap histogram to show whether running past
> that rating is actually dropping epochs). Phases 1–5 are read-only; phase 6
> enables NAV-PVT on the RAM layer, which a power cycle clears. It carries an
> `ARDUINO_ARCH_ESP32` branch that opens UART2 on `GNSS_RX_PIN`/`GNSS_TX_PIN`,
> so it compiles for this tree unchanged — edit those two constants at the top
> if your wiring differs from `config.h`.
>
> [`tools/common/gnss_reset`](../common/gnss_reset/gnss_reset.ino) carries the
> same platform block and is likewise usable here: a full GNSS factory reset
> (BBR + Flash + EEPROM wipe, then a cold-start reset) for recovering a
> receiver left in an unexpected config state.
>
> [`tools/common/gnss_otp_clock`](../common/gnss_otp_clock/gnss_otp_clock.ino)
> — same platform block again. Programs the M10 high-performance CPU clock into
> OTP, gated behind a typed confirmation. **Permanent and irreversible**; read
> its header before running it.

> **Method note.** This sketch mirrors
> [`tools/nRF52840/imu_calibration`](../nRF52840/imu_calibration/imu_calibration.ino)
> deliberately — same plateau-detected warmup, stability gate, session logging
> and aggregation — so results from the two trees are comparable. Read that
> sketch's header for the reasoning; this one's header covers only where the
> hardware genuinely differs:
>
> - **Units.** The MPU-6050 reports **m/s² and rad/s**, not the LSM6DS3's g and
>   deg/s. Every threshold and emitted `#define` is in native units.
> - **No GNSS power gate.** This variant has no `GNSS_EN_PIN`; the receiver runs
>   off the board rail regardless, so the UART is opened and drained to match
>   production's pin state, not to add the receiver's heat.
> - **USB is always present** (the board is USB-powered), so unlike the nRF tool
>   this is not about running without USB. Flash logging is there to survive
>   monitor disconnects and auto-resets, and to aggregate a run nobody watched.
> - **Storage isn't the limit.** LittleFS has megabytes free, so `LOG_BUDGET_BYTES`
>   is sized just under `MAX_RECORDS` rows on purpose — you can never log more
>   than `a` can aggregate. Raise both together or neither.
