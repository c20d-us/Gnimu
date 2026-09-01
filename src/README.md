# `src/` — what lives where

Three firmware variants of the same device, plus a shared bench-tools tree.
Each variant is a self-contained Arduino sketch folder: open the folder's `.ino`
in the Arduino IDE and it compiles on its own, with no cross-folder dependencies.

| Folder | What it is | Start here |
|---|---|---|
| [`Gnimu-ESP32/`](Gnimu-ESP32/) | The original always-on build. ESP32-WROOM-32 dev board, external MPU-6050 IMU, USB-powered, RGB status LED. | [README](Gnimu-ESP32/README.md) |
| [`Gnimu-nRF52840/`](Gnimu-nRF52840/) | The battery-powered evolution. Seeed XIAO nRF52840 Sense with its onboard LSM6DS3TR-C IMU, LiPo charge/state-of-charge subsystem, sleep-state machine, RGB status LED. | [README](Gnimu-nRF52840/README.md) |
| [`Gnimu-nRF52840-OLED/`](Gnimu-nRF52840-OLED/) | The same nRF52840 build with a 128×64 SSD1306 OLED in place of the status LED, showing state, battery, and GNSS fix quality as text. | [README](Gnimu-nRF52840-OLED/README.md) |
| [`tools/`](tools/) | Standalone diagnostic and bench-calibration sketches (not part of any firmware build), plus `check_common.sh`. | [nRF52840 tools](tools/nRF52840/README.md) · [ESP32 tools](tools/ESP32/README.md) |

All three variants advertise the same BLE identity and speak the same RaceBox
Data Message protocol, so any RaceBox-compatible app works with any of them.
Pick the one that matches your hardware; the repo-root [README](../README.md)
has the side-by-side comparison.

---

## Inside a variant folder

Every sketch folder follows the same shape:

```
Gnimu-<variant>.ino     Arduino entry point: setup() / loop()
config.h                All tunables - pins, rates, offsets, BLE identity
g_gnss.*                u-blox receiver: UART, UBX config, fix parsing
g_imu.*                 IMU: sampling, offsets, smoothing
g_telemetry.*           Packs GNSS+IMU into a RaceBox Data Message
g_ble.*                 BLE stack, services, advertising, notifications
g_battery.*             Cell voltage, state of charge, charge detection
g_ubx_helpers.*         UBX framing/checksum helpers
g_log.h                 Serial logging macros
ImuAxis.*               Sensor-frame -> vehicle-frame axis remap
g_led.* / g_display.*   Status readout (LED on two variants, OLED on the third)
g_power.* / g_state.*   Power gating and the sleep-state machine (nRF52840 only)
```

The folder name and the `.ino` name always match, as the Arduino IDE requires -
which conveniently means the IDE's title bar tells you which variant is open.

**If you change a file, check whether it is shared.** Several modules are
deliberately duplicated byte-identical across variants rather than factored into
a library (a shared library doesn't fit the Arduino sketch build model). Change
one copy, apply the same change to the others, then run:

```bash
./src/tools/check_common.sh
```

It lists which files are in the shared set, which are shared only between the two
nRF52840 trees, and which are excluded on purpose.

---

## Inside `tools/`

Diagnostic sketches, one per folder so the IDE can open them directly. Nothing
here is compiled into the firmware; these exist to validate an assumption or
measure a per-board constant that then gets pasted into a variant's `config.h`.

| Folder | Contents |
|---|---|
| [`tools/common/`](tools/common/) | GNSS sketches that build for any variant: `gnss_ver` (identity + high-rate capability report), `gnss_reset` (factory reset), `gnss_otp_clock` (**permanent** M10 high-performance clock burn). |
| [`tools/nRF52840/`](tools/nRF52840/) | The largest set - IMU probe/tiltmap/calibration/wake, LED check, BLE MTU, GNSS power gating and backup mode, battery presence and logging, flash storage. [README](tools/nRF52840/README.md) has a pass-criteria table and what each result fed back into the firmware. |
| [`tools/nRF52840-OLED/`](tools/nRF52840-OLED/) | OLED bring-up (`oled_probe`, `oled_layout`, `oled_bench`) plus this tree's own copy of `imu_calibration`. |
| [`tools/ESP32/`](tools/ESP32/) | `imu_calibration` in the MPU-6050's native units. [README](tools/ESP32/README.md) |
