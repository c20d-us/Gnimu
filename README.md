# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Platform: ESP32 / nRF52840](https://img.shields.io/badge/platform-ESP32%20%2F%20nRF52840-000000.svg)](#variants)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu turns an MCU (microcontroller), a GNSS (Global Navigation Satellite System) module and an IMU (Inertial Measurement Unit) into a device that emulates the function of a [RaceBox Mini](https://www.racebox.pro/products/racebox-mini) streaming performance telemetry meter. The official RaceBox app and other RaceBox-compatible tools connect to it over BLE (Bluetooth Low Energy) and read live position, speed, and motion data at up to 25Hz.

It's a low-cost, hackable platform for experimenting with microprocessors, GNSS & IMU data capture, the RaceBox BLE protocol, and sensor fusion built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry source for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## Variants

This repo hosts three builds of the same concept, targeting different microcontrollers and use patterns. All advertise the same BLE identity and RaceBox Data Message protocol, so any RaceBox-compatible app works with any of them.

| | [**Gnimu ESP32**](src/Gnimu-ESP32/README.md) | [**Gnimu nRF52840**](src/Gnimu-nRF52840/README.md) | [**Gnimu nRF52840-OLED**](src/Gnimu-nRF52840-OLED/README.md) |
|---|---|---|---|
| MCU | ESP32-WROOM-32 dev board | Seeed XIAO nRF52840 Sense | Seeed XIAO nRF52840 Sense |
| Power | USB-powered | 3.7V LiPo battery or USB | 3.7V LiPo battery or USB |
| GNSS | HGLRC M100-5883 | HGLRC M100-5883 | HGLRC M100-5883 |
| IMU | External 6-axis, MPU-6050 | Onboard 6-axis, LSM6DS3TR-C | Onboard 6-axis, LSM6DS3TR-C |
| Status readout | RGB LED | RGB LED | 0.96" 128×64 OLED |
| Status | Complete | Complete | **In development** |
| Best for | A simple, always-plugged-in build | A portable, battery-powered build | Seeing fix quality and battery state without a phone |

**Gnimu nRF52840-OLED** is an evolution of the nRF52840 build that swaps the RGB status LED for a small OLED. It shows device state, BLE connection and battery level as readable text, plus GNSS quality the LED could never convey — satellites locked, fix type, pDOP, horizontal accuracy and PVT rate. It is **not finished**: the hardware, wiring and screen layout are settled and bench-tested, but the display module itself isn't written yet, so the firmware currently behaves identically to the nRF52840 build. See its [`DESIGN.md`](src/Gnimu-nRF52840-OLED/DESIGN.md) for what's decided and what's outstanding.

Start with the README for whichever hardware you're building. Each has its own bill of materials, wiring, build/flash instructions, and configuration reference.

---

## Repo layout

```
images/
  ESP32/                 Build photos for the Gnimu ESP32 variant
  nRF52840/              Build photos for the Gnimu nRF52840 variant
src/
  Gnimu-ESP32/           ESP32 firmware + README
  Gnimu-nRF52840/        nRF52840 firmware + README + DESIGN notes
  Gnimu-nRF52840-OLED/    nRF52840 + OLED firmware + README + DESIGN notes
  tools/
    check_common.sh      Verifies the modules shared across variants are identical
    common/              Diagnostic sketches not tied to any one platform
    ESP32/               Diagnostic sketches for the ESP32 variant
    nRF52840/            Diagnostic sketches for the nRF52840 variant
    nRF52840-OLED/        Diagnostic sketches for the nRF52840-OLED variant
```

Each sketch folder is named for its variant and contains the `.ino` of the same
name, as the Arduino IDE requires. That also means the IDE's window title and
tab name identify which variant you have open.

Several modules are deliberately duplicated across the variants and kept byte-identical (a shared-library approach doesn't fit the Arduino sketch build model). If you change one of the shared files, apply the same change to the others and run `src/tools/check_common.sh` to confirm they still match. The script covers all three variants; add any new sketch folder to its `VARIANTS` list or that copy goes unchecked.

---

## Acknowledgment & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, both Gnimu variants have been completely overhauled from the original single-file Arduino sketch architecture — modular codebases, externalized configuration, and (for Gnimu nRF52840) a full battery/power subsystem and a from-scratch BLE stack port.

I am grateful to the original author for the initial implementation that made this project possible.

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE).
