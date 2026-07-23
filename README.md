# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Platform: ESP32 / nRF52840](https://img.shields.io/badge/platform-ESP32%20%2F%20nRF52840-000000.svg)](#variants)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu turns a GNSS (Global Navigation Satellite System) module and an IMU (Inertial Measurement Unit) into a device that emulates the function of a [RaceBox Mini](https://www.racebox.pro/products/racebox-mini) streaming performance telemetry meter. The official RaceBox app and other RaceBox-compatible tools connect to it over BLE (Bluetooth Low Energy) and read live position, speed, and motion data at or near 25Hz.

It's a low-cost, hackable platform for experimenting with GNSS+IMU data logging, the RaceBox BLE protocol, and sensor fusion built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry source for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## Variants

This repo hosts two builds of the same idea, targeting different microcontrollers and use cases. Both advertise the same BLE identity and RaceBox Data Message protocol, so any RaceBox-compatible app works with either one.

| | [**Gnimu ESP32**](src/ESP32/README.md) | [**Gnimu nRF52840**](src/nRF52840/README.md) |
|---|---|---|
| MCU | ESP32 dev board | Seeed XIAO nRF52840 Sense |
| Power | USB, always-on | 3.7V LiPo battery, with charge/idle/sleep power states |
| GNSS | HGLRC M100-5883 | HGLRC M100-5883 |
| IMU | External 6-axis, MPU-6050 | Onboard 6-axis, LSM6DS3TR-C |
| Best for | A simple, always-plugged-in build | A portable, battery-powered build |

Start with [`src/ESP32/README.md`](src/ESP32/README.md) or [`src/nRF52840/README.md`](src/nRF52840/README.md) depending on which hardware you're building — each has its own bill of materials, wiring, build/flash instructions, and configuration reference.

---

## Repo layout

```
images/
  ESP32/          Build photos for the Gnimu ESP32 variant
  nRF52840/       Build photos for the Gnimu nRF52840 variant
src/
  ESP32/          Gnimu ESP32 firmware, README, and diagnostic tools
  nRF52840/       Gnimu nRF52840 firmware, README, and diagnostic tools
  universal/      Sketches that aren't tied to either platform
tools/
  check_common.sh Verifies the modules shared by both variants are identical
```

Several modules are deliberately duplicated between the two variants and kept byte-identical (a shared-library approach doesn't fit the Arduino sketch build model). If you change one of the shared files, apply the same change to the other variant and run `tools/check_common.sh` to confirm they still match.

---

## Acknowledgment & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, both Gnimu variants have been completely overhauled from the original single-file Arduino sketch architecture — modular codebases, externalized configuration, and (for Gnimu nRF52840) a full battery/power subsystem and a from-scratch BLE stack port.

I am grateful to the original author for the initial implementation that made this project possible.

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE).
