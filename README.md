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
| GNSS | HGLRC M100-5883 | HGLRC M100-5883 | HGLRC M100 Mini |
| IMU | External 6-axis, MPU-6050 | Onboard 6-axis, LSM6DS3TR-C | Onboard 6-axis, LSM6DS3TR-C |
| Status readout | RGB LED | RGB LED | 0.96" 128×64 OLED |
| Best for | A simple, always-plugged-in build | A portable, battery-powered build | Seeing fix quality and battery state without a receiver |
|Build notes|Best bang-for-buck option. Easy build, cheap, rock-solid performance at 20Hz (GPS+Gal) or 25Hz (GPS only). Does require USB power source.|Simplest battery-powered option. Long battery life, solid performance. Slightly trickier build, but not hard. Could fit a slightly bigger LiPo.|"Advanced Beginner" mode. Trickiest build, but still not terribly hard. M100 Mini has slightly lower lock performance. If I were to do it again I'd skip the M100 Mini and use another M100-5883.|

Start with the README for whichever hardware you're building. Each has its own bill of materials, wiring, build/flash instructions, and configuration reference.

---

## Two notes about GNSS modules

1. All three of my builds use M10-based GNSS modules from HGLRC. Two use the M100-5883 module, and one uses the M100 Mini. Both modules are ~$20 each from Amazon. The M100-5883 is excellent for the price. I've seen 16+ SVs locked with <0.200m hAcc and 1.2 pDOP with the device sitting on a table in my living room. The M100 Mini is a little bit cheaper, and definitely smaller, but I wouldn't use it on future builds. It works OK, but the performance is not quite as good as the M100-5883's due to the smaller antenna patch (15x15mm vs. 21x21mm) . It's not worth the slight cost savings IMO.

2. If you use the HGLRC M100-5883 or M100 Mini modules, be aware that **as-delivered they *cannot* hold a 20Hz+ fix rate** even with just one constellation configured. To enable high fix rates you need to adjust the module's clock speed by writing values to their One-Time Programmable (OTP) memory (there is a sketch in the `./src/tools/common` folder to do this). Without doing this you'll never get 20Hz (with GPS **and** Galileo) or 25Hz (with GPS **or** Galileo) fix rates at high SV counts. All will look fine until you reach ~10-12 SVs, and then the fix rate will start to stumble and sag, getting worse as the SV count climbs. After you burn the OTP settings, you will see solid 20Hz/25Hz performance into the high teen SV counts and beyond. Keep in mind that the module documentation only claims ≥98% of fix rate at full SV capacity, so you may see minor degradation when lots of SVs are visible.

---

## A note about obscure settings and latency tweaks

I've spent a lot of time researching the ESP32, nRF52840 XIAO, MPU-6050, and M100 modules, in service of squeezing every last bit of performance and latency out of the Gnimu firmware builds. There are several places in the code where bus rates get tweaked, various features get turned on or off, and techniques are used to eliminate as much latency and blocking in the code as possible. I'm sure I've missed some opportunities somewhere, but if you see something odd in the code that makes you scratch your head and wonder, there is a high probably that it was done to ensure that the telemetry data flows as fast and most importantly as consistently as possible. This kind of device is less than helpful if the data flow is inconsistent, so I've focused on consistency as a primary design goal, with performance secondary.

---

## Repo layout

```
images/
  ESP32/                 Build photos for the Gnimu ESP32 variant
  nRF52840/              Build photos for the Gnimu nRF52840 variant
src/
  README.md              Guide to the folders below
  Gnimu-ESP32/           ESP32 firmware + README
  Gnimu-nRF52840/        nRF52840 firmware + README
  Gnimu-nRF52840-OLED/    nRF52840 + OLED firmware + README
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

## Acknowledgments & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, both Gnimu variants have been completely overhauled from the original single-file Arduino sketch architecture — modular codebases, externalized configuration, and (for Gnimu nRF52840) a full battery/power subsystem and a from-scratch BLE stack port.

I am grateful to the original author for the initial implementation that made this project possible.

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE).
