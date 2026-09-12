# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Platform: ESP32 / nRF52840](https://img.shields.io/badge/platform-ESP32%20%2F%20nRF52840-000000.svg)](#variants)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu turns an MCU (microcontroller), a GNSS (Global Navigation Satellite System) module and an IMU (Inertial Measurement Unit) into a device that emulates the function of a [RaceBox Mini](https://www.racebox.pro/products/racebox-mini) streaming performance telemetry device. The official RaceBox app and other RaceBox-compatible tools connect to it over BLE (Bluetooth Low Energy) and read live position, speed, and motion data at up to 25Hz.

It's a low-cost, hackable platform for experimenting with microprocessors, GNSS & IMU data capture, the RaceBox BLE protocol, and sensor fusion built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry source for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## Variants

This repo hosts three builds of the same concept, targeting different microcontrollers and use patterns. All advertise the same BLE identity and RaceBox Data Message protocol, so any RaceBox-compatible app should work with any of them.

| | [**Gnimu ESP32**](src/Gnimu-ESP32/README.md) | [**Gnimu nRF52840**](src/Gnimu-nRF52840/README.md) | [**Gnimu nRF52840-OLED**](src/Gnimu-nRF52840-OLED/README.md) |
|---|---|---|---|
| MCU | ESP32-WROOM-32 dev board | Seeed XIAO nRF52840 Sense | Seeed XIAO nRF52840 Sense |
| Power | USB-powered | 3.7V LiPo battery or USB | 3.7V LiPo battery or USB |
| GNSS | HGLRC M100-5883 | HGLRC M100-5883 | HGLRC M100 Mini |
| IMU | External 6-axis, MPU-6050 | Onboard 6-axis, LSM6DS3TR-C | Onboard 6-axis, LSM6DS3TR-C |
| Status readout | RGB LED | RGB LED | 0.96" 128×64 OLED |
| Best for | A simple, always-plugged-in build | A portable, battery-powered build | Seeing fix quality, rate, and battery state without a receiver |
|Build notes|Best bang-for-the-buck option. Easy build, cheap, rock-solid performance at 20Hz (GPS+Gal) or 25Hz (GPS only). Requires USB power source.|Simplest battery-powered option. Long battery life, solid performance. Slightly trickier build, but not hard. Could fit a slightly bigger LiPo.|"Advanced Beginner" mode. Trickiest build, but still not terribly hard. M100 Mini has slightly lower lock performance. If I were to do it again I'd skip the M100 Mini and use another M100-5883.|

**The IMU is optional.** Position, speed, lap timing and everything else GNSS-based work without one; only g-force data needs it. So an ESP32 and GNSS module with no MPU-6050, or the plain (non-Sense) XIAO nRF52840, makes a cheaper device that is still fully useful. RaceBox-protocol apps simply see zero g. The nRF52840 builds switch the IMU off by themselves when you select the plain XIAO board in the IDE; on the ESP32, set `IMU_ENABLED 0` in `config.h`. Either way no IMU library is needed to build. What you give up: g-force data, the automatic self-levelling, and (on the nRF52840) waking from light sleep by motion — a connection from the app still wakes it.

Start with the README for whichever hardware you're building (links at the top of the columns). Each has its own bill of materials, wiring, build/flash instructions, and configuration reference.

---

## Three notes about GNSS modules

1. All three of my builds use M10-based GNSS modules from HGLRC. Two use the M100-5883 module, and one uses the M100 Mini. Both modules are ~$20 each from Amazon. The M100-5883 is excellent for the price. I've seen 16+ SVs locked with <0.200m hAcc and 1.2 pDOP with the device sitting on a table in my living room. The M100 Mini is a little bit cheaper, and definitely smaller, but I wouldn't use it on future builds. It works OK, but the performance is not quite as good as the M100-5883's due to the smaller antenna patch (15x15mm vs. 21x21mm) . It's not worth the slight cost savings IMO.

2. If you use the HGLRC M100-5883 or M100 Mini modules, be aware that **as-delivered they *cannot* hold a 20Hz+ fix rate** even with just one constellation configured. To enable high fix rates you need to adjust the module's clock speed by writing values to their One-Time Programmable (OTP) memory (there is a sketch in the `./src/tools/common` folder to do this). Without doing this you'll never get 20Hz (with GPS **and** Galileo) or 25Hz (with GPS **or** Galileo) fix rates at high SV counts. All will look fine until you reach ~10-12 SVs, and then the fix rate will start to stumble and sag, getting worse as the SV count climbs. After you burn the OTP settings, you will see solid 20Hz/25Hz performance into the high teen SV counts and beyond. Keep in mind that the module documentation only claims ≥98% of fix rate at full SV capacity, so you may see minor degradation when lots of SVs are visible.

3. There are other u-blox compatible GNSS modules that should work with this firmware, either as a drop-in replacement or with minor code tweaks. I have not tried any other options, but you can easily find several other M10-based modules on Amazon, DigiKey, and Mouser. Keep in mind that M10-based modules will all have similar performance and constraints as the HGLRC modules.

---

## A note about mounting and self-calibration

The firmware calibrates itself to how it's mounted. Once it's powered on, settled, and stationary for 30 seconds with a valid 3D fix, it measures its own mounting tilt and gyroscope zero point, applies them, and holds that calibration until the device is powered off.

That means you don't have to get the mount perfectly level, and there's no per-board calibration step before you flash. Earlier versions of this firmware needed six hand-measured zero-point offsets pasted into `config.h` for every individual board; those are gone, and the firmware image is now identical on every unit.

A few practical notes:

- **Mount the device in your desired location, power it on while parked, and let it sit.** The calibration happens during the *first* qualifying stationary period, which is what keeps it from calibrating itself to a sloped staging lane later on.
- **Give it a minute or two.** The calibration window doesn't start until there's a usable 3D fix, and on a cold start that's usually the largest part of the stabilization period. In my testing it's taken anywhere from 70 to 95 seconds from power-on to get to the calibration window.
- If you're connected to serial, **watch the `Trim:` field** on the log line. `⏳` means it hasn't locked yet; `✅` means it has, and the number beside it is the mounting tilt it measured. On the OLED build the same states show up as an icon in the status bar. You'll see a check once it's locked, an X if it refused, and nothing while it's still deciding.
- **It'll correct up to about 15° of tilt.** Past that it refuses rather than half-correcting, and shows `❌` — so if you see that with a `Trim:` angle above 15°, the mount is the problem, not the firmware. Don't expect the `❌` immediately though: the reported tilt sits at 0° until the first measurement lands, so a badly mounted device shows `⏳` for the first 30 seconds or so and only then switches over.
- **It calibrates against the ground it's parked on**, and can't tell mounting tilt from the slope under the car.

Engine vibration doesn't interfere with this process. I checked, and a calibration captured at cold idle is repeatable to about 0.02°, which is miniscule for our purposes. So it doesn't matter whether you start the car before calibration or calibrate first.

For what it's worth, the real RaceBox Mini handles this differently. The user manual instructs to run an accelerometer calibration from the app, and says to *"perform this procedure every time you mount the device."* That works, but it's a step you can forget, and forgetting it silently tilts your g-force data for the whole session. Doing it in firmware seemed like the better trade, even though it costs some stationary time up front.

---

## A note about privacy: this is an open location beacon

Gnimu has to look exactly like a RaceBox Mini to the apps that talk to it, and a RaceBox Mini accepts connections from anyone. So there's no pairing, no password and no encryption: **any phone within Bluetooth range can connect and read your live position, 20 times a second.** It also advertises under a fixed name, so anyone scanning nearby can see it's there without connecting at all. That can't be locked down without breaking compatibility with the apps.

When it's reachable:

- **ESP32:** whenever it has power.
- **nRF52840 builds:** while it's running, and for up to 4 hours after an app last received its data (4 hours with no app subscribed, running on the battery). After that it drops into deep sleep and stops advertising entirely.

Two side effects of the same openness. On every build, only one phone can be connected at a time, so a stranger who connects first locks your app out until they leave. And on the nRF52840 builds, an app that is actually receiving the data keeps the device awake, so one left open (yours or anyone's) holds it at full power until the battery's low-voltage cutoff. A bare connection that never subscribes, such as a Bluetooth-scanner app, does not.

**The slide switch is the only sure way to make it unreachable.** If you leave it mounted in a car, switch it off — or unplug the ESP32 build — when you park.

---

## A note about obscure settings and latency tweaks

I've spent a lot of time researching the ESP32, nRF52840 XIAO, MPU-6050, and M100 modules, in service of squeezing every last bit of performance and latency out of the Gnimu firmware builds. There are several places in the code where bus rates get tweaked, various features get turned on or off, and techniques are used to eliminate as much latency and blocking in the code as possible. I'm sure I've missed some opportunities somewhere, but if you see something odd in the code that makes you scratch your head and wonder, there is a high probably that it was done to ensure that the telemetry data flows as fast and (most importantly) as consistently as possible. This kind of device is not very useful if the data flow is inconsistent, so I've focused on consistent performance as a primary design goal.

---

## Repo layout

```
docs/
  architecture-modules.md
                         Diagram: module dependencies and sharing scope
  architecture-runtime.md
                         Diagram: the path taken on each GNSS epoch
  architecture-verification.md
                         Diagram: how the encoder is proven correct
  imu-trim-design.md     Design record for the runtime mounting/gyro calibration
  multiprotocol-design.md
                         Design record for the protocol plug-in architecture
  racechrono-ble-mapping.md
                         RaceChrono DIY BLE reference + field mapping (research
                         only - not implemented)
test/
  capture.py             Turns a Gnimu Monitor capture into golden test vectors
  synthetic.py           Vectors for states the hardware cannot reach
  synthetic.gc1          Those vectors, generated
  harness.cpp            Runs the firmware encoder against the vectors
  run_harness.sh         Builds and runs the harness
  telemetry/             Runs the real g_telemetry.cpp on the host: the same
                         vectors through buildSample(), rates, stats line
  run_telemetry_harness.sh
                         Builds and runs it for all three variants
  imu/                   Runs the real IMU pipeline against fake sensors
  run_imu_harness.sh     Builds and runs it for all three variants
  gnss/                  Runs the real GNSS driver against a fake receiver:
                         baud sweep, config sequence, epoch plumbing
  run_gnss_harness.sh    Builds and runs it for all three variants
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

Several modules are deliberately duplicated across the variants and kept byte-identical rather than factored into a shared Arduino library, because those modules read each sketch's own `config.h`, which a library's sources cannot see — a library was built and rejected on exactly that point ([details](src/README.md#inside-a-variant-folder)). If you change one of the shared files, apply the same change to the others and run `src/tools/check_common.sh` to confirm they still match. The script also catches what you forget to tell it about: a file shared by two or more trees but on none of its lists, and a sketch folder missing from its `VARIANTS` list.

The wire format is isolated in `g_proto_<name>.*`, so the packet layout can be changed — or a second protocol added — without touching the GNSS, IMU or BLE code. `test/run_harness.sh` compiles that encoder on the host and checks it against recorded golden vectors, which is how a change to it can be proven byte-identical before flashing anything. See [`docs/multiprotocol-design.md`](docs/multiprotocol-design.md).

---

## Acknowledgments & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, both Gnimu variants have been completely overhauled from the original single-file Arduino sketch architecture — modular codebases, externalized configuration, and (for Gnimu nRF52840) a full battery/power subsystem and a from-scratch BLE stack port. I am grateful to the original author for the initial implementation that started me down the road on this project.

I also want to acknowledge that much of the background research and some of the more complex code in this project wouldn't have been possible without the help of Claude and Claude Code. I've done quite a lot of coding over the course of my career, but not much C++ work and no Arduino projects before this one. Claude Code helped me learn the basics of Arduino development and solved some of the thornier issues that I struggled with along the way. Claude was invaluable as a research assistant, and uncovered documentation that I never would have found otherwise. 

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE).
