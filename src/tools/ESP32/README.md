# ESP32 tools

Standalone sketches for bringing up or bench-tuning parts of the ESP32 build
outside the main firmware. Each lives in its own folder so the Arduino IDE can
open it directly.

| Sketch | Purpose | Pass criteria |
|---|---|---|
| [`imu_calibration/`](imu_calibration/imu_calibration.ino) | Per-axis MPU-6050 zero-point offsets in the raw sensor frame, independent of `IMU_SWAP_XY`/`IMU_SIGN_*` (feeds `config.h`'s `IMU_ACCEL_OFFSET_*_MPS2` / `IMU_GYRO_OFFSET_*_RADPS`). Warms up until die temp plateaus (5–20 min), then repeating 10000-sample sessions 1 min apart, each gated on a stability check and appended to LittleFS. Press any key over Serial to halt, then `a` to aggregate the run into six paste-ready `#define` lines. | Gate passes on a still bench; `a` reports a median and 20% trimmed mean that agree within 1 sd, and a median tilt under 1°. |

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
