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
config.h                Per-board settings - pins, rates, sensor setup, protocol choice
g_imu_tuning.h          IMU tuning shared by every board: filters, thresholds, trim
g_gnss.*                u-blox receiver, same in every tree: baud sweep, UBX
                        config, PVT callback and epoch cache
g_gnss_port.h           The seam between that driver and the MCU's UART
g_gnss_port_<mcu>.*     The UART itself: g_gnss_port_esp32 (HardwareSerial(2),
                        512-byte ring), g_gnss_port_nrf52 (Serial1, D6/D7)
g_imu.*                 IMU pipeline, same in every tree: remap, trim, filters, decimation
g_imu_sensor.h          The seam between the pipeline and a sensor driver
g_imu_<part>.*          Sensor driver: g_imu_mpu6050 (ESP32), g_imu_lsm6ds3 (nRF52840)
g_imu_trim.*            Runtime mounting-tilt and gyro-bias correction
ImuAxis.*               Per-axis smoothing filter and transient-peak tracker
g_telemetry.*           Cadence: builds a TelemetrySample per GNSS epoch and
                        hands it to the active protocol encoder
g_protocol.h            The protocol plug-in contract: TelemetrySample, the
                        frame sink, and the transport descriptor
g_protocol_active.h     Resolves TELEMETRY_PROTOCOL to one descriptor - the
                        single place a new protocol is wired in
g_proto_racebox.*       RaceBox wire format: identity, UUIDs, packet encoder
g_ble.*                 BLE stack mechanics; builds services from the descriptor
g_battery.*             Cell voltage, state of charge, charge detection
g_ubx_helpers.*         UBX framing/checksum helpers
g_log.h                 Serial logging macros
g_led.* / g_display.*   Status readout (LED on two variants, OLED on the third)
g_power.* / g_state.*   Power gating and the sleep-state machine (nRF52840 only)
```

The folder name and the `.ino` name always match, as the Arduino IDE requires -
which conveniently means the IDE's title bar tells you which variant is open.

**If you change a file, check whether it is shared.** Several modules are
deliberately duplicated byte-identical across variants rather than factored into
a library. Change one copy, apply the same change to the others, then run:

```bash
./src/tools/check_common.sh
```

It lists which files are in the shared set, which are shared only between the two
nRF52840 trees, and which are excluded on purpose. It also sweeps for what you
forgot: any file present in two or more trees but on no list fails the check, and
so does a sketch folder missing from its `VARIANTS` list. Put a new shared file
in the list it belongs to - or in `EXCLUDED_FILES`, with a reason, if its copies
are meant to differ.

**Why duplication and not a shared library.** Arduino *does* support libraries,
so this is a choice rather than a constraint, and the reason is specific. The
modules with real behaviour - GNSS, BLE, IMU, battery, power, telemetry, logging
- read the sketch's own `config.h` for pins, rates, thresholds and feature flags.
Arduino compiles a library's sources without the sketch folder on the include
path, so a library cannot see `config.h`. One was built anyway (`GnimuCore`) and
made to work, but every config-coupled module needed an implementation header
plus a per-sketch shim to inject its configuration, which was more convoluted
than the duplication it replaced, and it was rejected. Distribution was a
secondary cost: a library has to be installed into the sketchbook before any
variant will build, where a sketch folder just opens.

The modules that do *not* read `config.h` - the protocol encoder, `ImuAxis`, the
trim module, the UBX helpers - are config-free by design (it is what lets
`test/harness.cpp` compile the encoder on a host). They could in principle live
in a library, but two sharing mechanisms would be worse than one, and
`check_common.sh` would still be needed for the rest. Duplication plus the
script is the settled answer; revisit it only with a way around the include-path
problem.

The wire format lives entirely in `g_proto_<name>.*`. Nothing in `g_telemetry`
or `g_ble` knows what protocol is running - they consume the descriptor that
`g_protocol_active.h` selects. See [`../docs/architecture-modules.md`](../docs/architecture-modules.md)
for a diagram of how these files depend on each other and which must stay
byte-identical, and [`../docs/multiprotocol-design.md`](../docs/multiprotocol-design.md)
for the design record.

---

## Inside `tools/`

Diagnostic sketches, one per folder so the IDE can open them directly. Nothing
here is compiled into the firmware; these exist to validate an assumption or
measure a per-board constant that then gets pasted into a variant's `config.h`.

| Folder | Contents |
|---|---|
| [`tools/common/`](tools/common/) | GNSS sketches that build for any variant: `gnss_ver` (identity + high-rate capability report), `gnss_reset` (factory reset), `gnss_otp_clock` (**permanent** M10 high-performance clock burn). |
| [`tools/nRF52840/`](tools/nRF52840/) | The largest set - IMU probe/tiltmap/calibration, LED check, BLE MTU, GNSS power gating, battery presence and logging, flash storage. [README](tools/nRF52840/README.md) has a pass-criteria table and what each result fed back into the firmware. |
| [`tools/nRF52840-OLED/`](tools/nRF52840-OLED/) | OLED bring-up (`oled_probe`, `oled_bench`) plus this tree's own copy of `imu_calibration`. |
| [`tools/ESP32/`](tools/ESP32/) | `imu_calibration` in the MPU-6050's native units. [README](tools/ESP32/README.md) |
