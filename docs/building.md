# Building and flashing

The firmware is one Arduino sketch, [`code/Gnimu/`](../code/Gnimu/), that builds for both supported boards. You can build it in the Arduino IDE or from the command line with `arduino-cli`; both use the same toolchain and produce the same firmware.

For the hardware side of a build (parts, wiring, photos), see the guide for your board: [ESP32](build-esp32.md) or [XIAO nRF52840](build-nrf52840.md).

---

## What you need

| | ESP32 | XIAO nRF52840 |
|---|---|---|
| Board package | **esp32** by Espressif Systems | **Seeed nRF52 Boards** |
| Board to select | **ESP32 Dev Module** | **Seeed XIAO nRF52840 Sense** |
| Libraries | SparkFun u-blox GNSS v3 | SparkFun u-blox GNSS v3, Seeed Arduino LSM6DS3 |

BLE support comes with each board package; there is no BLE library to install.

These are the versions the firmware is built and tested against. Newer ones will probably work, but if something breaks after an update, come back to these:

| Component | Version |
|---|---|
| esp32 board package | 3.3.12 |
| Seeed nRF52 Boards | 1.1.13 |
| SparkFun u-blox GNSS v3 | 3.1.15 |
| Seeed Arduino LSM6DS3 | 2.0.7 |

> [!IMPORTANT]
> **Use "Seeed nRF52 Boards", not "Seeed nRF52 mbed-enabled Boards".** The mbed-enabled package lacks the Bluefruit BLE stack this firmware uses.

---

## Arduino IDE

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x).
2. For the XIAO nRF52840, add this URL under **Settings → Additional boards manager URLs**:
   ```
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   ```
3. Install the board package for your board from the **Boards Manager**, and the libraries from the **Library Manager** (see the table above).
4. Open [`code/Gnimu/Gnimu.ino`](../code/Gnimu/Gnimu.ino).
5. Select your board and its serial port.
6. Click **Upload**.
7. Open the **Serial Monitor** at **115200 baud** to watch the boot log. See [Console and diagnostics](console.md) for what it shows.

There is nothing you have to edit before the first build. The settings you might want to change later are in [`Config.h`](../code/Gnimu/Config.h), which appears as a tab next to the sketch; see [Configuration](configuration.md).

If an upload to the XIAO can't reset the board into its bootloader, **double-tap the reset button on the XIAO** to force it, then upload again.

> [!IMPORTANT]
> If you are building on an Apple Silicon Mac, you can use the native Arduino IDE but you **must** have Rosetta installed in order to compile. Without Rosetta installed you will get a compilation error.

> [!IMPORTANT]
> **macOS build gotcha (XIAO nRF52840):** The Seeed nRF52 core's `platform.txt` invokes bare `python` for its UF2 step, but modern macOS only ships `python3`, so compiling fails with `exec: "python": executable file not found in $PATH`.
>
> **Fix:** in `~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/<version>/platform.txt`, change `python` to `python3` on the `recipe.objcopy.uf2.pattern` line. This reverts on every core reinstall or update, so it must be re-done afterward.

### The build stamp

Every boot prints a `BUILD` line with the firmware's version, git revision, and build time. Those values are filled in by the build script described below. The Arduino IDE doesn't run that script, so an IDE build reports placeholders instead:

```
BUILD,version=0.0.0-dev,revision=IDE-UNSTAMPED,dirty=unknown,utc=unknown,target=...
```

That is expected, and the firmware is otherwise identical.

---

## Command line

[`scripts/build.sh`](../scripts/build.sh) builds the firmware with `arduino-cli`:

```bash
scripts/build.sh            # both boards
scripts/build.sh esp32      # one board
scripts/build.sh nrf52840
```

The script:

- compiles from clean with all warnings on, and **fails if any warning comes from the project's own sources** (warnings from board packages and libraries are shown but don't fail the build);
- stamps the version, git revision, and build time into the firmware;
- writes the finished firmware to `build/esp32/` or `build/nrf52840/`.

It finds `arduino-cli` on your `PATH`, or uses the copy bundled with the Arduino IDE on macOS, or whatever `ARDUINO_CLI` points to. It uses its own build folder, so it can run alongside an IDE build without either disturbing the other.

To set up `arduino-cli` from scratch, install the board packages and libraries at the tested versions:

```bash
arduino-cli core update-index --config-file arduino-cli.yaml
arduino-cli core install esp32:esp32@3.3.12 --config-file arduino-cli.yaml
arduino-cli core install Seeeduino:nrf52@1.1.13 --config-file arduino-cli.yaml
arduino-cli lib install "SparkFun u-blox GNSS v3@3.1.15"
arduino-cli lib install "Seeed Arduino LSM6DS3@2.0.7"
```

[`arduino-cli.yaml`](../arduino-cli.yaml), in the repository root, carries the Seeed boards manager URL. If you already installed everything through the Arduino IDE, `arduino-cli` shares those installs and there is nothing more to do.

The script only builds. To flash from the command line, use `arduino-cli upload` with your board's port, or simply upload from the IDE.

---

## Host tests

The hardware-independent part of the firmware, [`code/Gnimu/src/core/`](../code/Gnimu/src/core/), has unit tests that run on your computer with no board attached:

```bash
tests/run.sh
```

It needs only `g++`. Each test is built and run twice: once plain, and once under AddressSanitizer and UndefinedBehaviorSanitizer, where the first finding fails the run.

The tests compile as C++11 on purpose. The Seeed nRF52 board package builds sketches as C++11, so anything newer in `core/` would fail the nRF52840 build; compiling the tests the same way catches it here first.

---

## Continuous integration

Every push and pull request runs the host tests and builds both boards with `scripts/build.sh`, at the versions listed above. The workflow is [`.github/workflows/ci.yml`](../.github/workflows/ci.yml).
