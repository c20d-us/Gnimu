<p>
  <img src="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/svg/gnimu-icon-default-rounded.svg" alt="Gnimu icon" height="72">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/wordmark/gnimu-wordmark-transparent-white.svg">
    <img src="https://raw.githubusercontent.com/c20d-us/Gnimu-branding/main/wordmark/gnimu-wordmark-transparent-navy.svg" alt="gnimu" height="72">
  </picture>
</p>

# GNSS+IMU motorsports telemetry

[![License: GPL v3+](https://img.shields.io/badge/License-GPLv3%2B-blue.svg)](LICENSE)
[![Platform: ESP32 / nRF52840](https://img.shields.io/badge/platform-ESP32%20%2F%20nRF52840-000000.svg)](#supported-boards)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu turns an MCU (microcontroller), a GNSS (Global Navigation Satellite System) module and an IMU (Inertial Measurement Unit) into a motorsports telemetry source that streams live position, speed, and motion data at up to 25Hz over BLE (Bluetooth Low Energy). It implements the publicly documented RaceBox BLE data protocol for interoperability. Telemetry apps and tools that read that protocol should be able to use it, with a minor caveat regarding the advertised name as outlined below. Only the streaming data protocol is implemented, not the RaceBox command/configuration functions. The firmware ignores BLE packets written back to it except for three Gnimu-specific command structures (re-trim, device restart and system OFF) described in [device commands](docs/device-commands.md). Only a Gnimu-specific client such as [Gnimu Monitor](https://github.com/c20d-us/Gnimu-Monitor) will send these commands.

It's a low-cost, hackable platform for experimenting with microprocessors, GNSS & IMU data capture, BLE telemetry protocols and sensor fusion, built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry source for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.
> [!NOTE]
> **Multi-protocol compatibility.** Right now, Gnimu streams the RaceBox data protocol only. The recent re-write of the firmware was specifically structured in a way that allows other protocol formats to be wired in, and [RaceChrono DIY](https://github.com/aollin/racechrono-ble-diy-device) is the next one I intend to implement. In the future I intend to implement a new, purpose-built, Open Source telemetry protocol that stands on its own and delivers GNSS+IMU data in an extensible format. Keep an eye out for those updates.

> [!NOTE]
> **The advertised name.** The device advertises as `Gnimu-RB` followed by a device-unique 10-digit ID. Some apps pick devices out of a BLE scan by the name they expect; if the app you use does this, you'll need to set `kBleNamePrefix` and `kBleManufacturer` appropriately in [`Config.h`](code/Gnimu/Config.h) or the app will not see the device.

> [!NOTE]
> **Looking for the original firmware?** The first generation of Gnimu, including the OLED build, now lives in [`classic/`](classic/README.md). It still works, but it is deprecated and no longer receives updates or fixes. See [Gnimu Classic](#gnimu-classic) below.

> [!IMPORTANT]
> **Unofficial project.** This is an independent and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner, and are used here only to describe the protocol this firmware is compatible with. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## Supported boards

One firmware (in [`code/Gnimu/`](code/Gnimu/)) builds for both boards. The board you select in the Arduino IDE or with the command-line build script decides which hardware support is compiled in; there is nothing to edit to switch between them. Both advertise the same BLE identity and stream the same data protocol, so the same apps work with either.

| | [**ESP32**](docs/build-esp32.md) | [**XIAO nRF52840**](docs/build-nrf52840.md) |
|---|---|---|
| MCU | ESP32-WROOM-32 dev board | Seeed XIAO nRF52840 Sense |
| Power | USB-powered | 3.7V LiPo battery or USB |
| GNSS | HGLRC M100-5883 | HGLRC M100-5883 |
| IMU | External 6-axis, MPU-6050 | Onboard 6-axis, LSM6DS3TR-C |
| Status readout | Single-color LED | RGB LED |
| Construction | Hand-wired | Hand-wired, or on the [Gnimu carrier PCB](https://github.com/c20d-us/Gnimu-PCB) |
| Best for | A simple, always-plugged-in build | A portable, battery-powered build |

Start with the build guide for whichever hardware you're building (links at the top of the columns). Each has its own bill of materials, wiring, and usage notes. The rest of the documentation is shared:

| Guide | What it covers |
|---|---|
| [Building and flashing](docs/building.md) | Arduino IDE and `arduino-cli` setup, library versions, the build script, and the host tests |
| [Configuration](docs/configuration.md) | The settings in `Config.h`: GNSS rate and constellations, BLE power, IMU smoothing, battery thresholds, and the hardware and protocol build profiles |
| [Console and diagnostics](docs/console.md) | The boot log, the serial commands, the diagnostics report, and the 1Hz status line |
| [Device commands](docs/device-commands.md) | Re-trim, restart and System OFF, from the serial console or over BLE, and reading the configuration over BLE |
| [Architecture](docs/architecture.md) | How the code is organized, the fixed-rate scheduler, and the run and power states |
| [Migrating from Classic](docs/migrating-from-classic.md) | What changes if you already have a Classic build |

---

## What's new since Classic

This firmware is a ground-up rewrite of the original Gnimu. The device does the same job, and the packets on the wire are the same, but nearly everything underneath changed:

- **One codebase for every board.** Classic kept a separate copy of the firmware per variant. Now there is one sketch, with the board-specific parts isolated in their own layer.
- **Fixed-rate scheduling.** Every job (GNSS, IMU, BLE, battery, LED, console) runs on its own timer, with per-job timing statistics you can read back at any time.
- **Receiver settings that stick.** Each GNSS setting is checked at boot and written only if it differs, then saved to the receiver's own flash. A configured receiver boots faster and stays configured.
- **Nothing to set per device.** The BLE name is derived from the chip's hardware ID, so every unit gets a stable, unique name from the same firmware image.
- **Faults instead of hangs.** A missing or silent GNSS receiver puts the device into a fault state with a red LED and a working console, and a watchdog restarts the firmware if it ever stalls.
- **A serial console.** A boot log that reports every step, an on-demand diagnostics report, and an optional 1Hz status line.
- **Device commands.** Re-trim the IMU, restart, or switch the device off, from the serial console or over BLE, and read the device's configuration over BLE. See [Device commands](docs/device-commands.md).
- **Simpler power states on battery.** Running, waiting for the switch, charge-only, and a low-voltage cutoff that protects the cell.
- **Tests.** The hardware-independent logic is unit-tested on the host, and CI builds both boards.

The OLED variant was not carried forward. If you want the display, [Classic](classic/src/Gnimu-nRF52840-OLED/README.md) is where it lives.

---

## Three notes about GNSS modules

1. My builds use the M10-based HGLRC M100-5883, which is ~$20 from Amazon and excellent for the price. I've seen 16+ SVs locked with <0.200m hAcc and 1.2 pDOP with the device sitting on a table in my living room. The smaller M100 Mini also works, but I wouldn't choose it again: its smaller antenna patch (15x15mm vs. 21x21mm) costs some SV-lock performance, and the savings aren't worth it IMO.

2. If you use the HGLRC M100-5883 or M100 Mini modules, be aware that **as-delivered they *cannot* hold a 20Hz+ fix rate** even with just one constellation configured. To enable high fix rates you need to adjust the module's clock speed by writing values to One-Time Programmable (OTP) memory. [`tools/gnss_ver`](tools/gnss_ver/gnss_ver.ino) tells you whether your module needs it, and [`tools/gnss_otp_clock`](tools/gnss_otp_clock/gnss_otp_clock.ino) does the write. **The OTP write is permanent.** Without it you'll never get 20Hz (with GPS **and** Galileo) or 25Hz (with GPS **or** Galileo) fix rates at high SV counts. All will look fine until you reach ~12 SVs, and then the fix rate will start to stumble and sag, getting worse as the SV count climbs. After you write the OTP settings, you should see solid 20Hz/25Hz performance with SV counts into the high teens. Keep in mind that the module documentation only claims ≥98% of fix rate at full SV capacity, so you may still see minor degradation when lots of SVs (20+) are visible. If you regularly use the device in a location with wide sky visibility, you should consider going to one enabled constellation at either 20Hz or 25Hz to avoid high-SV-count fix drops. Testing will tell you if this is a problem.

3. There are other u-blox compatible GNSS modules that should work with this firmware, either as a drop-in replacement or with minor code tweaks. I have not tried any other options, but you can easily find several other M10-based modules on Amazon, DigiKey, and Mouser. Keep in mind that M10-based modules will all have similar performance and constraints as the HGLRC modules. I have not looked into non-UBX (NMEA-only) GNSS modules at all, so if you want to try one you're on your own.

---

## A note about mounting and self-calibration

The firmware calibrates itself to how it's mounted. Once it's powered on, settled, and stationary with a valid 3D fix, it measures its own mounting tilt and gyroscope zero point and applies them. The tilt correction is then held until the device is powered off or you ask it to re-trim; the gyroscope zero point continues refining whenever the device is stationary to adjust for temperature variations that can affect the gyro.

This calibration process means that you don't have to worry about getting the device perfectly level when you mount it in your vehicle, though you should try to get reasonably close. The calibration won't correct for a wildly askew mount.

A few practical notes:

- **Mount the device in your desired location, power it on while parked in a flat area, and leave it on.** The calibration happens during the *first* qualifying stationary period, which is what keeps it from calibrating itself to a sloped staging lane later on.
- **Give it a minute or two.** The calibration window doesn't start until there's a usable 3D fix, and on a cold start that's usually the largest part of the wait. Once the fix is there, it takes about 15 seconds of sitting still to calibrate.
- If you're connected to serial, **watch the `Trim:` field** on the status line. `⏳` means it hasn't locked yet; `🔒` means it has, and the number beside it is the mounting tilt it measured.
- **It'll correct up to about 15° of tilt.** Past that it refuses rather than half-correcting. If `Trim:` shows an angle above 15° and stays at `⏳`, the mount is the problem, not the firmware.
- **It calibrates against the ground it's parked on**, and can't tell mounting tilt from the slope under the car, so try to not have the calibration step happen when parked on any amount of slope.
- **You can redo it on demand without a restart.** `T` on the serial console, or the matching [BLE command](docs/device-commands.md), drops the lock and measures again at the next stop.

Engine vibration doesn't interfere with the calibration process, so it doesn't matter whether you start the car before calibration or calibrate first.

For what it's worth, some commercial devices handle this differently, by having the user run an accelerometer calibration from the app every time the device is mounted. That works, but it's a step you can forget, and forgetting it silently tilts your g-force data for the whole session. Doing it in firmware seemed like the better trade, even though it costs some stationary time up front.

The reasoning behind the design is written up in [`docs/imu-trim.md`](docs/imu-trim.md).

---

## Repo layout notes

```
code/Gnimu/              The firmware sketch
  Gnimu.ino              Open this in the Arduino IDE
  Config.h               The settings you might change
  src/
    core/                Hardware-independent logic (unit-tested on the host)
    platform/            Per-board support: esp32/ and nrf52840/
    peripherals/         GNSS, IMU and BLE drivers
    app/                 Wiring it all together: jobs, states, console
    variants/            Build profiles: hardware (pins, IMU mounting) and protocol
tools/                   GNSS utility sketches: identity report, factory reset, OTP clock
tests/                   Host-side unit tests (tests/run.sh)
scripts/                 build.sh and flash.sh: build and upload with arduino-cli
docs/                    Build guides and reference documentation
images/                  Build photos for the current guides
classic/                 The original firmware, deprecated (see below)
```

---

## Gnimu Classic

[`classic/`](classic/README.md) holds the original Gnimu firmware, final at version 1.2.1, with all of its documentation, build photos, and diagnostic sketches: the ESP32, nRF52840, and nRF52840-OLED variants.

**Classic still works** on the hardware it was written for, and you're welcome to keep using it. It is, however, **deprecated**: it no longer receives updates or fixes, and new builds should use the firmware in this README. If you have a Classic build and want to move it over, see [Migrating from Classic](docs/migrating-from-classic.md).

The repository as it was before the reorganization is preserved at the [`classic-final`](https://github.com/c20d-us/Gnimu/tree/classic-final) tag.

---

## Acknowledgments & Origins

The original Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox Mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator), and that lineage continues in [Classic](classic/README.md). The current firmware is a ground-up rewrite that shares no code with it, but I am grateful to the original author for the implementation that started me down the road on this project.

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License, version 3 or (at your option) any later version** — see [`LICENSE`](LICENSE).

The Gnimu name and logo are not covered by this license. See the [Gnimu branding usage policy](https://github.com/c20d-us/Gnimu-branding/blob/main/LICENSE.md).
