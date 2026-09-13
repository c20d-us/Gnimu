# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Platform: ESP32 / nRF52840](https://img.shields.io/badge/platform-ESP32%20%2F%20nRF52840-000000.svg)](#variants)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

Gnimu turns an MCU (microcontroller), a GNSS (Global Navigation Satellite System) module and an IMU (Inertial Measurement Unit) into a device that emulates the function of a [RaceBox Mini](https://www.racebox.pro/products/racebox-mini) streaming telemetry device. The official RaceBox app and other RaceBox-compatible tools connect to it over BLE (Bluetooth Low Energy) and read live position, speed, and motion data at up to 25Hz.

It's a low-cost, hackable platform for experimenting with microprocessors, GNSS & IMU data capture, the RaceBox BLE protocol, and sensor fusion built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry device for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## What it does

- Reads a live [**GNSS fix**](https://en.wikipedia.org/wiki/Satellite_navigation) (position, altitude, speed, heading, accuracy, fix status, satellite count) from a u-blox GNSS receiver.
- Reads **acceleration and rotation** from a 6-axis [**IMU**](https://en.wikipedia.org/wiki/Inertial_measurement_unit) at 100Hz, subtracts zero-point offsets, applies a transient-aware filter, and decimates the data to the BLE transmission rate.
- Packs the GNSS and IMU data into a **RaceBox Data Message** (a u-blox UBX-framed binary packet) and streams it over **BLE** to a RaceBox-compatible client.
- Advertises a BLE **Device Information Service** (model, serial, firmware, hardware, manufacturer) so official apps recognize and pair with it.
- Optionally, prints a human-readable **serial status line** at 1Hz for debugging: packet rate, GNSS data rate, satellite count, fix type, horizontal accuracy, position, and IMU values.

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

**The IMU is optional, but leaving it out has drawbacks.** Position, speed, lap timing and everything else GNSS-based will work without one; only g-force and gyro data need it. An ESP32 and GNSS module with no MPU-6050, or the plain (non-Sense) XIAO nRF52840, makes a slightly cheaper device that is still useful. RaceBox-protocol apps will see zeros for g-force and gyro. How the apps respond to that is an app-specific question that I can't answer. The nRF52840 builds switch the IMU functions off automatically if you build on the plain XIAO board in the IDE; on the ESP32, set `IMU_ENABLED 0` in `config.h` to turn off the IMU code (if you forget, it will still build and run but issue an error on boot that no IMU is present). What you give up: g-force and gyro data that a receiving app might use for sensor fusion.

Start with the README for whichever hardware you're building (links at the top of the columns). Each has its own bill of materials and wiring, plus the build steps and settings specific to that board; the ones shared by every build are in [Building](#building) and [Configuration](#configuration) below.

---

## GNSS module considerations

1. All three of my builds use M10-based GNSS modules from HGLRC. Two use the M100-5883 module, and one uses the M100 Mini. Both modules are ~$20 each from Amazon. The M100-5883 is excellent for the price. I've seen 16+ SVs locked with <0.200m hAcc and 1.2 pDOP with the device sitting on a table in my living room. The M100 Mini is a little bit cheaper, and definitely smaller, but I wouldn't use it on future builds. It works fine, but the performance is not quite as good as the M100-5883's due to the smaller antenna patch (15x15mm vs. 21x21mm). The slightly smaller size is not worth the few dollars of cost savings IMO.

2. There are other u-blox compatible GNSS modules that should work with this firmware, either as a drop-in replacement or with minor code tweaks. I have not tried any other options, but you can easily find several other M10-based modules on Amazon, DigiKey, and Mouser. Keep in mind that M10-based modules will all have similar performance and constraints as the HGLRC modules.

3. GNSS reception is sensitive to nearby RF noise. On compact builds, the BLE radio can desensitize the GNSS receiver. Dialing `BLE_TX_POWER_ADV_DBM` / `BLE_TX_POWER_CONN_DBM` down to the lowest level that works for you is advised. The receiving app is usually close by, so high power isn't generally needed. When the BLE power level was left at the default value of +9dbm on my ESP32 build, the device had significantly worse lock quality, sometimes not getting a fix at all (especially indoors).

## GNSS fix rate and enabled constellations

The maximum PVT rate on the u-blox M10 platform depends on how many constellations you enable and, less obviously, on CPU clock settings. Both rows below are published u-blox specifications from [UBX-23006557][ubx-m10-specs]:

| Concurrent constellations | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| Low CPU clock max nav rate | 18Hz | 10Hz | 10Hz | 5Hz |
| High CPU clock max nav rate | **25Hz** | **20Hz** | 16Hz | 10Hz |

The High CPU clock row is the one every M10 glossy sheet quotes, footnoted as *"Configuration required."* That footnote is load-bearing: u-blox ships (or at least used to ship) M10 silicon at the low CPU clock rates (128/128/128/64MHz) for lower power consumption. The higher rates (192/192/192/96MHz) **need a one-time, permanent write of the faster clock rates set into the receiver's OTP memory.** This is per §2.1.7 of the MAX-M10S integration manual [UBX-20053088][ubx-m10-integration]. Without the CPU clock rate change, you will never get 20Hz or 25Hz sustained nav rates at high SV counts regardless of how many or few constellations you configure.

u-blox permits running past the published ratings ("The navigation update rate can be increased beyond the maximum value stated in the datasheet. However, this may result in a reduced fix rate."). The receiver does not reject the setting, it just silently skips navigation epochs when it cannot keep up. For use as a motorsports telemetry device, a solidly consistent nav rate and high position accuracy are key attributes, so this behavior is particularly bad.

**Check your own module.** [`src/tools/common/gnss_ver`](./src/tools/common/gnss_ver/gnss_ver.ino) reports current configuration. [`src/tools/common/gnss_otp_clock`](./src/tools/common/gnss_otp_clock/gnss_otp_clock.ino) performs the OTP write, behind a typed confirmation. **The OTP write cannot be undone**, and it consumes 18 of the receiver's 64 bytes of OTP space.

I run 20Hz nav rate with GNS+Galileo enabled, to get higher-precision positioning. A valid alternative is to run GPS *-or-* Galileo alone at 25Hz. This costs you the second constellation's geometry, and the accuracy difference can be visible. If you would rather have the higher 25Hz rate at the expense of potentially lower accuracy, set `GNSS_NAV_RATE_HZ 25` and disable Galileo (or GPS, depending on where you are in the world) in `GNSS_CONSTELLATIONS`.

**Why the real RaceBox Mini delivers 25Hz:** it uses a [u-blox NEO-M9N][ubx-m9n-specs] GNSS, which is a different platform that does not derate at higher constellation counts. The M9N datasheet lists 25Hz for *every* configuration, from a single constellation up to GPS+GLO+GAL+BDS concurrently. The drawback is higher power consumption and cost. The M10 is an economical choice for a small battery-powered device, but the 20Hz ceiling for GPS+GAL is the downside. If you want to try and fully emulate a RaceBox Mini, a NEO-M9N module shouldn't be too hard to integrate with this code (it's perhaps even a drop-in), but it will draw more power and deplete your battery faster (not a concern for the ESP32-based build).

---

### M100 GNSS LED indicators

These are the M100 module's own LEDs (not driven by our firmware). They are useful for judging rough fix status without a serial connection.

| LED | Pattern | Meaning |
|---|---|---|
| Red (power) | Solid | GNSS rail powered |
| Blue (PPS) | Fast flicker | Powered, no fix acquired yet |
| Blue (PPS) | Fast flicker w/ 1Hz blink | 3D fix & time lock acquired |

***Note*** - the M100 Mini's LEDs are **OPPOSITE** of the M100-5883's. Swap Red and Blue in the chart above for the Mini.

---

### IMU smoothing

Raw accelerometer and gyroscope samples are read at 100Hz and run through a per-axis filter before being decimated to the configured transmission rate:

- Each axis tracks an EMA (exponential moving average) baseline (`IMU_ACCEL_ALPHA` / `IMU_GYRO_ALPHA`) for a smooth, low-noise signal.
- Within each transmission window, the axis also tracks the largest raw deviation from that baseline.
- If the deviation exceeds `IMU_ACCEL_TRANSIENT_THRESHOLD_G` / `IMU_GYRO_TRANSIENT_THRESHOLD_DPS`, the transmitted value blends toward the raw peak in proportion to how far past the threshold it went — fully at 2× the threshold, partially in between, pure baseline at or under it.

This keeps the transmitted trace smooth during normal driving while still surfacing sharp events (kerb strikes, hard transients) that a plain low-pass filter would otherwise flatten out. The thresholds are tunable per-axis-group in `g_imu_tuning.h` and should be set above your car's vibration floor (engine/tire/kerb noise) but below the magnitude of events you want preserved. They currently ship **parked**, with blending disabled, until the further testing is done to determine appropriate thresholds (see [Configuration](#configuration)).

---

## Mounting and self-calibration

When an IMU is part of the build, the firmware calibrates itself to how it's mounted. Once it's powered on, settled, and stationary for `IMU_TRIM_QUALIFY_MS` milliseconds with a valid 3D fix, it measures its own mounting tilt and gyroscope zero point, applies calculated offsets, and holds that calibration until the device is powered off.

This means you don't have to get the mount perfectly level, and there's no per-board calibration step before you flash. Earlier versions of this firmware needed six hand-measured zero-point offsets pasted into `config.h` for every individual board; those are gone.

A few practical notes about mounting and calibration:

- **Mount the device in your desired location, power it on while parked.** The calibration happens during the *first* qualifying stationary period, which is what keeps it from calibrating itself to a sloped staging lane later on.
- **Give it a few minutes before using.** The calibration window doesn't start until there's a usable 3D fix, and on a cold start that's usually the largest part of the stabilization period. If you're not in an area with a very clear sky, 3D fix can take up to 2-3 minutes. If the sky is relatively clear, calibration should be done in under 2 minutes.
- If you're connected to serial, **watch the `Trim:` field** on the log line. `⏳` means it hasn't locked yet; `✅` means it has, and the number beside it is the mounting tilt it measured. On the OLED build the same states show up as an icon in the status bar. You'll see a check once it's locked, an X if it refused to calibrate (too much tilt), and nothing while it's still deciding.
- **It'll correct up to about 15° of tilt.** Past that it refuses rather than half-correcting, and shows `❌` on the serial line. If you see that with a `Trim:` angle above 15°, the mount is the problem, not the firmware. As of right now, there is no outward indication of the excess tilt error. I will probably come up with an LED flash pattern to indicate it at some point.
- **It will calibrate against the ground you're parked on.** The device can't tell mounting tilt from the slope under your car. If at all possible, park somewhere that is as close to level as possible when calibrating. If your grid area is heavily sloped it won't matter how level the device is mounted relative to your vehicle. You'd be better off mounting and powering on the device somewhere level to calibrate before moving to grid.
- **Engine vibration doesn't interfere with calibration.** I checked, and a calibration captured at cold idle is repeatable to about 0.02°, which is a minuscule error range for our purposes. So it doesn't matter whether you start the car before calibration or calibrate first.

For what it's worth, a real RaceBox Mini handles this differently. The user manual instructs to run an accelerometer calibration from the app, and says to *"perform this procedure every time you mount the device."* That works, but it's a step you can forget, and forgetting it silently pollutes your g-force data for the whole session. Doing it in automatically seems like the better approach, even though it costs some stationary time up front. I may build a re-calibration function into the firmware at some point that can be triggered in a similar way as the RaceBox Mini. In the meantime, a re-calibration can always be accomplished by just power cycling the device.

---

## Privacy considerations

Gnimu has to look exactly like a RaceBox Mini to the apps that talk to it, and a RaceBox Mini accepts connections from anyone. There's no pairing, no password, and no encryption. **Any Bluetooth receiver within range can connect and read your live position.** It also advertises under a fixed name, so anyone scanning nearby can see it's there without connecting at all. These behaviors can't be changed without breaking app compatibility.

Also, only one receiver can be connected at a time, so a stranger who connects first can lock your app out until they disconnect or you power cycle the device (assuming the stranger's app doesn't have auto-reconnect enabled). If you don't see your device as available for connection in your app, this might be the cause.

**The slide switch and/or USB power connection is the only sure way to make the device unreachable.** If you leave your device mounted in a car, switch it off or unplug it when you park if you're concerned about third-parties connecting to it.

---

## A note about obscure settings and latency tweaks

I've spent a lot of time researching the ESP32, nRF52840 XIAO, MPU-6050, and M100 modules, in service of squeezing every last bit of performance and latency out of the Gnimu firmware builds. There are several places in the code where bus rates get tweaked, various features get turned on or off, and techniques are used to eliminate as much latency and blocking in the code as possible. I'm sure I've missed some opportunities somewhere, but if you see something odd in the code that makes you scratch your head and wonder, there is a high probability that it was done to ensure that the telemetry data flows as fast and (most importantly) as consistently as possible. This kind of device is not very useful if the data flow is inconsistent, so I've focused on consistent performance as a primary design goal.

---

## Building

Everything here applies to all three variants. Each variant's README lists its board support package, any extra libraries, and anything specific to flashing that board.

- **[Arduino IDE](https://www.arduino.cc/en/software)** (2.x recommended).
- **SparkFun u-blox GNSS v3**, installed via Library Manager — the one library every variant needs.

1. Install your variant's board support package and extra libraries.
2. Open the variant's `.ino` (for example `src/Gnimu-ESP32/Gnimu-ESP32.ino`).
3. Edit that folder's `config.h` — at minimum, set your `DEVICE_ID` (see [Configuration](#configuration)).
4. Select your board and the correct serial port.
5. Click **Upload**.
6. Open the **Serial Monitor** at **115200 baud** to watch startup and status output.

> [!IMPORTANT]
> If you are building on an Apple Silicon Mac, you can use the AS-native Arduino IDE but you **must** have Rosetta installed in order to correctly compile the binary. Without Rosetta installed you will get a compilation error.

---

## Configuration

Each sketch folder has its own `config.h`, grouped into sections. The one exception is the IMU tuning (smoothing, transient thresholds, `IMU_TRIM_*`), which is the same on every Gnimu board and so lives in `g_imu_tuning.h`, kept identical across all three trees by [`src/tools/check_common.sh`](src/tools/check_common.sh). Many values are checked with `static_assert` at compile time, so an invalid configuration fails the build with a clear message instead of misbehaving on the device.

The settings below mean the same thing on every build. Hardware-specific settings (pins, sensor ranges and filters, BLE power, battery, power states and display) are listed in each variant's README.

| Setting | Purpose |
|---|---|
| `DEVICE_ID` | 10-digit device serial as a **quoted string** (e.g. `"1001001001"`). Validated at compile time: exactly 10 digits, first digit `0`–`3`. |
| `TELEMETRY_PROTOCOL` | Which wire protocol this build emits. `PROTO_RACEBOX` is currently the only implemented value. Chosen at compile time, so unselected protocols are never linked and cost no flash. A protocol's own constants (identity strings, service and characteristic UUIDs) live in `g_proto_<name>.h` rather than `config.h`, so they stay under `check_common.sh`. |
| `GNSS_BAUD` | GNSS serial baud rate (`115200` by default). On boot the firmware finds the module at any standard rate, switches it to `GNSS_BAUD`, and saves that port setting to the receiver, so a change survives the next boot. |
| `GNSS_NAV_RATE_HZ` | GNSS PVT rate in Hz (1–25; `20` by default). Set once at startup and held for the life of the session, connected or not. What the receiver can actually sustain depends on its CPU clock and the constellations enabled — see [GNSS fix rate and enabled constellations](#gnss-fix-rate-and-enabled-constellations). |
| `GNSS_SV_MINELEV_DEG` | Ignore satellites below this elevation angle (anti-multipath). |
| `GNSS_CONSTELLATIONS` | Macro-array of `{name, id, enabled}` entries, one per constellation the M10 supports (GPS, Galileo, GLONASS, BeiDou, QZSS, SBAS); GPS + Galileo by default. Enable only what your module and region support: each one added lowers the maximum nav rate (see the table linked above). |
| `IMU_ACCEL_ALPHA`, `IMU_GYRO_ALPHA`, `IMU_ACCEL_TRANSIENT_THRESHOLD_G`, `IMU_GYRO_TRANSIENT_THRESHOLD_DPS` | Per-axis-group EMA smoothing (lower = smoother, more lag) and the deviation each transmit window's peak must exceed before it's blended into the reported value — see [IMU smoothing](#imu-smoothing). Thresholds are in g and °/s on every build; each IMU driver converts at the read. The accel alpha is tuned against GNSS-referenced track data (2018 BMW M2, nRF52840 build); the gyro values remain untested placeholders. The transient thresholds currently ship **parked** (blending disabled) until further research is done. Both settings are car- and mount-specific: once un-parked, the threshold must sit above your vibration floor, or the blend fires continuously and inflates reported peaks. |
| `IMU_TRIM_*` | Runtime levelling and gyro de-biasing. After `IMU_TRIM_QUALIFY_MS` milliseconds continuously stationary with a valid 3D fix, the firmware measures its own mounting tilt and gyro zero, applies them, and **locks the orientation for the rest of the power cycle** — see [Mounting and self-calibration](#mounting-and-self-calibration). `IMU_TRIM_REQUIRE_FIX 0` can be used for bench testing if you have trouble getting a fix indoors. |
| `IMU_AXIS_X/Y/Z_SRC`, `IMU_AXIS_X/Y/Z_SIGN` | Mounting-orientation remap into the vehicle frame. Each vehicle axis names which sensor axis feeds it (`0`=X, `1`=Y, `2`=Z) plus a sign, covering all **24** physically-realizable orientations. A determinant `static_assert` rejects a mirrored (impossible) map at compile time. The shipped values reflect how each reference build is physically mounted, so expect to change them for your own enclosure; the derivation procedure and order table are in `config.h`. |
| `LOG_ENABLED` | Master switch for all serial diagnostic output. `1` (default) = normal logging. `0` = **silent build**: every `LOG_*` call and its arguments vanish at compile time, and `setup()` no longer waits for a serial port. |

---

## Connecting

These steps are the same on every build; each variant's README adds what its status LED or display shows along the way.

1. Power the device and give the GNSS time to acquire a fix. The module's own LEDs show progress (see [M100 GNSS LED indicators](#m100-gnss-led-indicators)), and if an IMU is fitted, the first stationary period after a fix is when it calibrates (see [Mounting and self-calibration](#mounting-and-self-calibration)).
2. In the **RaceBox app** (or another RaceBox-compatible client), scan for and connect to the device. It advertises as "RaceBox Mini" followed by your `DEVICE_ID`, e.g. "RaceBox Mini 1001001001".
3. On connect, the device begins streaming data packets. Only one client can be connected at a time (see [Privacy considerations](#privacy-considerations)).
4. Optional: connect over USB and open a serial monitor at 115200 baud to watch the 1Hz status line (requires `LOG_ENABLED 1`).

---

## Troubleshooting

Symptoms common to every build. Each variant's README lists the ones specific to its hardware.

| Symptom | Things to check |
|---|---|
| `❌ u-blox GNSS not detected` | UART wiring (note the TX↔RX crossover) and power to the receiver. The firmware tries every standard baud rate on its own, so a receiver left at a different baud is not the cause. |
| Few or no satellites | Move outdoors or near a window; lower the BLE transmit power (see [GNSS module considerations](#gnss-module-considerations)); check which constellations are enabled; give it a cold-start minute. |
| App won't connect | Confirm `DEVICE_ID` is valid (10 digits, first digit 0–3); make sure no other client already holds the (single) connection. |
| Build fails with a `static_assert` message | Read the message — it names the offending `config.h` value and the allowed range. |

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
  ble/                   Runs the real BLE driver against a fake port: emit
                         policy, sessions, the inbound write queue
  run_ble_harness.sh     Builds and runs it for all three variants
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

Several modules are deliberately duplicated across the variants and kept byte-identical rather than factored into a shared Arduino library, because those modules read each sketch's own `config.h`, which a library cannot see. If you change one of the shared files, apply the same change to the others and run `src/tools/check_common.sh` to confirm they still match. The script also catches what you forget to tell it about: a file shared by two or more trees but on none of its lists, and a sketch folder missing from its `VARIANTS` list.

The wire format is isolated in `g_proto_<name>.*`, so the packet layout can be changed or a second protocol added without touching the GNSS, IMU or BLE code. `test/run_harness.sh` compiles that encoder on the host and checks it against recorded golden vectors, which is how a change to it can be proven byte-identical before flashing anything. See [`docs/multiprotocol-design.md`](docs/multiprotocol-design.md).

---

## Acknowledgments & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, both Gnimu variants have been completely overhauled from the original single-file Arduino sketch architecture — modular codebases, externalized configuration, and (for Gnimu nRF52840) a full battery/power subsystem and a from-scratch BLE stack port. I am grateful to the original author for the initial implementation that started me down the road on this project.

I also want to acknowledge that much of the background research and some of the more complex code in this project wouldn't have been possible without the help of Claude and Claude Code. I've done quite a lot of coding over the course of my career, but not much C++ work and no Arduino projects before this one. Claude Code helped me learn the basics of Arduino development and solved some of the thornier issues that I struggled with along the way. Claude was invaluable as a research assistant, and uncovered documentation that I never would have found otherwise. 

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE).

[ubx-m10-specs]: https://content.u-blox.com/sites/default/files/documents/u-bloxM10-with-25Hz-Navigation-UpdateRate_IN_UBX-23006557.pdf
[ubx-m9n-specs]: https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf
[ubx-m10-integration]: https://content.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf
