# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](../../LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/platform-ESP32-000000.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Language: C++ (Arduino)](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg)](https://www.arduino.cc/)

The code in this repo lets you turn an ESP32 development board, a GNSS (Global Navigation Satellite System) module, and an IMU (Inertial Measurement Unit) module into a device that emulates the function of a [RaceBox Mini](https://www.racebox.pro/products/racebox-mini) streaming performance telemetry meter. The official RaceBox app and other RaceBox-compatible tools should be able to connect to it over BLE (Bluetooth Low Energy) and read live position, speed, and motion data at or near 25Hz.

This is a low-cost, hackable platform for experimenting with GNSS+IMU data logging, the RaceBox BLE protocol, and sensor fusion built from inexpensive off-the-shelf parts.

I originally started this project as a streaming GNSS+IMU telemetry source for use with the [AutoX Data Logger for iOS](https://autoxdrivermod.com) app.

I pronounce the project name as "nigh-mew," though I have no strong opinion on how anyone else should pronounce it.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## What it does

- Reads a live [**GNSS fix**](https://en.wikipedia.org/wiki/Satellite_navigation) (position, altitude, speed, heading, accuracy, fix status, satellite count) from a u-blox GNSS receiver at **20 Hz**.
- Reads **acceleration and rotation** from a 6-axis [**IMU**](https://en.wikipedia.org/wiki/Inertial_measurement_unit) at 100Hz, subtracts per-chip zero-point offsets, smooths it with a transient-aware filter, and decimates it to the 25Hz transmission rate — see [IMU smoothing](#imu-smoothing).
- Packs the GNSS and IMU data into a **RaceBox Data Message** (a u-blox UBX-framed binary packet) and streams it over **BLE** to a RaceBox-compatible client at or near 25Hz.
- Advertises a BLE **Device Information Service** (model, serial, firmware, hardware, manufacturer) so official apps recognize and pair with it.
- Prints a human-readable **serial status line** at 1Hz for debugging: packet rate, GNSS data rate, satellite count, fix type, horizontal accuracy, position, and IMU values.

```mermaid
flowchart LR
    GNSS["u-blox GNSS module"] -- "UART · 115200 baud" --> ESP32["ESP32"]
    IMU["accel + gyro"] -- "I²C" --> ESP32
    ESP32 -- "BLE notify · RaceBox UBX packets" --> App["RaceBox-compatible app"]
```

---

## Hardware

<table>
  <tr>
    <th width="28%" align="left">Part</th>
    <th align="left">Notes</th>
  </tr>
  <tr>
    <td><a href="https://www.amazon.com/dp/B0DF2YJSHN"><strong>ESP32 dev board</strong></a></td>
    <td>Developed on an AITRIP ESP32-WROOM-32 Development Board.</td>
  </tr>
  <tr>
    <td><a href="https://www.amazon.com/dp/B0CB5N8RQ8"><strong>u-blox GNSS module</strong></a></td>
    <td>A u-blox <a href="https://www.u-blox.com/en/product/max-m10-series">M10-class</a> GNSS receiver. Reference unit: <a href="https://www.hglrc.com/products/m100-5883-gps">HGLRC M100-5883</a>. Other u-blox modules supported by the SparkFun library should work.</td>
  </tr>
  <tr>
    <td><a href="https://www.amazon.com/dp/B01DK83ZYQ"><strong>IMU module</strong></a></td>
    <td>I²C 6-axis accelerometer + gyroscope breakout. Reference unit: <a href="http://www.hiletgo.com/ProductDetail/2157948.html">HiLetgo GY-521</a> based on the <a href="https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/">InvenSense MPU-6050</a>.</td>
  </tr>
  <tr>
    <td><a href="https://www.amazon.com/dp/B0BQYPKRQS"><strong>Project Box</strong></a></td>
    <td>ABS plastic project case, white, 80x50x26mm. You'll need to cut holes into this box to fit your specific board and component layout (see images below).</td>
  </tr>
  <tr>
    <td><a href="https://www.amazon.com/dp/B0FPMC9917"><strong>Nylon M2.5 hex standoffs</strong></a></td>
    <td>Nylon hex standoffs, washers, nuts, screws, to help with positioning the components within the project box.</td>
  </tr>
</table>

### Wiring

**GNSS module → ESP32 (UART, Serial2)**

| GNSS pin | ESP32 pin |
|----------|-----------|
| TX       | GPIO16 (RX2) |
| RX       | GPIO17 (TX2) |
| VCC      | 3V3 |
| GND      | GND |

**IMU module → ESP32 (I²C)**

| IMU pin | ESP32 pin |
|-------------|-----------|
| SDA         | GPIO21 (default I²C SDA) |
| SCL         | GPIO22 (default I²C SCL) |
| VCC         | VIN (5V pin) |
| GND         | GND |

**Status LED:** the onboard LED (GPIO2) blinks while waiting for a BLE connection and stays solid when a client is connected.

> Pin assignments for the GNSS UART and the LED are configurable in [`config.h`](config.h). The IMU uses the ESP32's default I²C pins.

---

## Build gallery

Photos of the reference build, from loose components to the finished, enclosed unit. Several shots show an **RF shield** fitted over the electronics — a hardware counterpart to the firmware's reduced BLE power that further isolates the GNSS receiver from radio noise (see [A note on BLE power and GNSS lock](#a-note-on-ble-power-and-gnss-lock)).

<div align="center">
  <img src="../../images/ESP32/completed-emulator.jpeg" alt="The finished RaceBox Mini emulator" width="520"><br>
  <sub>The completed emulator, decorated with the stickers that came with the GNSS module<br>and an indicator of which end points forward (for Gyro/Accelerometer).</sub><br>&nbsp;
</div>

<table>
  <tr>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/components-and-box.jpeg" alt="Components laid out with the enclosure" width="340"><br>
      <sub>Components and enclosure prior to assembly.</sub><br>&nbsp;
    </td>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/GNSS-and-lid.jpeg" alt="GNSS module with mounting hole in the lid" width="340"><br>
      <sub>GNSS module with mounting hole in the lid.</sub><br>&nbsp;
    </td>
  </tr>
  <tr>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/components-wired.jpeg" alt="Components wired together" width="340"><br>
      <sub>Components wired together, using header pins<br>underneath the ESP32 board. Extra unused<br>pins were clipped off.</sub><br>&nbsp;
    </td>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/assembled-without-shield.jpeg" alt="Assembly without the RF shield" width="340"><br>
      <sub>Assembled, using hardening epoxy putty<br>to firmly affix the components.</sub>
    </td>
  </tr>
  <tr>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/shield-test-fit.jpeg" alt="RF shield test fit" width="340"><br>
      <sub>RF shield test fit, not yet grounded or affixed.</sub><br>&nbsp;
    </td>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/shield-grounded.jpeg" alt="RF shield grounded" width="340"><br>
      <sub>RF shield grounded; I used two-sided tape to mount the shield.</sub><br>&nbsp;
    </td>
  </tr>
  <tr>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/box-closed.jpeg" alt="Enclosure closed" width="340"><br>
      <sub>Enclosure closed up, ready to test (before stickers!).</sub><br>&nbsp;
    </td>
    <td align="center" valign="top" width="50%">
      <img src="../../images/ESP32/powered-up.jpeg" alt="Connected and working" width="340"><br>
      <sub>Connected to power, LEDs lit up.</sub><br>&nbsp;
    </td>
  </tr>
</table>

---

## Software & dependencies

- **[Arduino IDE](https://www.arduino.cc/en/software)** (2.x recommended).
- **ESP32 board support** — install the `esp32` package by Espressif via the Boards Manager.
- Libraries (install via Library Manager):
  - **Adafruit MPU6050** (pulls in Adafruit Unified Sensor + Adafruit BusIO)
  - **SparkFun u-blox GNSS v3**
  - BLE support is built into the ESP32 Arduino core — no extra install needed.

---

## Build & flash

### Arduino IDE

1. Install the ESP32 board package and the libraries listed above.
2. Open [`Gnimu-ESP32.ino`](Gnimu-ESP32.ino).
3. Edit [`config.h`](config.h) (at minimum, set your `DEVICE_ID`).
4. Select your board (e.g. **ESP32 Dev Module**) and the correct serial port.
5. Click **Upload**.
6. Open the **Serial Monitor** at **115200 baud** to watch the startup and status output.

> [!IMPORTANT]
> If you are building on an Apple Silicon Mac, you can use the AS-native Arduino IDE but you **must** have Rosetta installed in order to correctly compile the ESP32 binary. Without Rosetta installed you will get a compilation error.
---

## Configuration

All user-tunable settings live in [`config.h`](config.h), grouped into sections. Highlights:

| Setting | Purpose |
|---------|---------|
| `DEVICE_ID` | 10-digit device serial as a **quoted string** (e.g. `"3608675309"`). Validated at compile time: exactly 10 digits, first digit `0`–`3`. |
| `GNSS_RX_PIN`, `GNSS_TX_PIN`, `LED_ONBOARD_PIN` | Hardware pin assignments. |
| `GNSS_BAUD` | GNSS serial baud rate. On boot the firmware can detect a module at any valid baud rate, switch it to `GNSS_BAUD`, and save the config to flash. |
| `GNSS_NAV_RATE_HZ` | GNSS PVT rate in Hz (1–25). Set once at startup and held for the life of the session, connected or not. |
| `GNSS_SV_MINELEV_DEG` | Ignore satellites below this elevation angle (anti-multipath). |
| `GNSS_CONSTELLATIONS` | Per-constellation enable/disable list (GPS, Galileo, GLONASS, BeiDou, QZSS, SBAS). Enable only what your module/region supports — too many can drop the update rate below 25Hz. |
| `IMU_ACCEL_RANGE_G`, `IMU_GYRO_RANGE_DPS`, `IMU_FILTER_BANDWIDTH_HZ` | MPU-6050 full-scale ranges and built-in low-pass bandwidth (Adafruit MPU6050 enum tokens). |
| `IMU_ACCEL_ALPHA`, `IMU_GYRO_ALPHA` | EMA baseline smoothing strength per axis group. Lower = smoother, more lag. |
| `IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2`, `IMU_GYRO_TRANSIENT_THRESHOLD_RADPS` | Deviation (native sensor units — m/s² for accel, rad/s for gyro) that triggers blending the raw peak into the transmitted value. See [IMU smoothing](#imu-smoothing). |
| `IMU_ACCEL_OFFSET_*_MPS2`, `IMU_GYRO_OFFSET_*_RADPS` | Per-chip zero-point corrections in the raw sensor frame, subtracted before smoothing. Measure once per board with [`tools/imu_calibration/`](../tools/ESP32/imu_calibration/imu_calibration.ino) and paste the printed values. |
| `IMU_AXIS_X/Y/Z_SRC`, `IMU_AXIS_X/Y/Z_SIGN` | Mounting-orientation remap into the vehicle frame. Each vehicle axis names which sensor axis feeds it (`0`=X, `1`=Y, `2`=Z) plus a sign, covering all **24** physically-realizable orientations. A determinant `static_assert` rejects a mirrored (impossible) map at compile time. Defaults are the identity map, leaving the raw sensor frame untouched. Derivation procedure and the order table are in `config.h`. |
| `BLE_TX_POWER` | BLE transmit power. **Lowering this reduces RF interference with the GNSS front end and can noticeably improve satellite lock** — see notes below. |
| `LOG_ENABLED` | Master switch for all serial diagnostic output. `0` = **silent build**: every `LOG_*` call vanishes at compile time and `setup()` skips the wait for the serial port. |

Several values are checked with `static_assert` at compile time, so an invalid configuration fails the build with a clear message instead of misbehaving on the device.

### IMU smoothing

Raw accelerometer and gyroscope samples are read at 100Hz and run through a per-axis filter (one instance each for accel X/Y/Z and gyro X/Y/Z) before being decimated to the 25Hz transmission rate:

- Each axis tracks an EMA (exponential moving average) baseline (`IMU_ACCEL_ALPHA` / `IMU_GYRO_ALPHA`) for a smooth, low-noise signal.
- Within each transmission window, the axis also tracks the largest raw deviation from that baseline.
- If the deviation exceeds `IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2` / `IMU_GYRO_TRANSIENT_THRESHOLD_RADPS`, the transmitted value blends toward the raw peak in proportion to how far past the threshold it went — fully at 2× the threshold, partially in between, pure baseline at or under it.

This keeps the transmitted trace smooth during normal driving while still surfacing sharp events (kerb strikes, hard transients) that a plain low-pass filter would otherwise flatten out. The thresholds are tunable per-axis-group in `config.h` and should be set above your car's vibration floor (engine/tire/kerb noise) but below the magnitude of events you want preserved.

### A note on BLE power and GNSS lock

GNSS reception is sensitive to nearby RF noise. On compact builds, the ESP32's BLE radio can desensitize the GNSS receiver. Dialing `BLE_TX_POWER` down to a low level (the default is `ESP_PWR_LVL_N12`, the minimum) keeps the radio quiet — the receiver is usually close by, so high power isn't needed — and can dramatically improve fix quality, including indoors.

The **RF shield** shown in the [build gallery](#build-gallery) is the hardware counterpart to this: a grounded metal enclosure over the GNSS module that physically blocks radio noise from reaching the GNSS receiver. The two measures stack — lowering the BLE power quiets the source, while the shield blocks whatever remains. Either helps on its own; together they give the most reliable lock.

When the BLE power level was left at the default value of +9dbm on my ESP32 board, the device had significantly worse lock quality, sometimes not getting a fix at all (especially indoors).

With the BLE power level set to -12db, I have seen simultaneuous lock on as many as 20 satellites with horizontal accuracy (HAcc) as low as 220mm and [pDOP](https://en.wikipedia.org/wiki/Dilution_of_precision) values under 2 (really good for a cheap consumer-grade GNSS module).

### A note on GNSS fix rate and enabled constellations

The maximum PVT rate on the u-blox M10 platform depends on how many constellations you enable. This is a documented platform limit, not a tuning problem — u-blox publishes the figures in [UBX-23006557](https://content.u-blox.com/sites/default/files/documents/u-bloxM10-with-25Hz-Navigation-UpdateRate_IN_UBX-23006557.pdf):

| Concurrent constellations | 1 (GPS only) | 2 (GPS+GAL) | 3 | 4 |
|---|---|---|---|---|
| Max nav update rate | **25 Hz** | **20 Hz** | 16 Hz | 10 Hz |

Those are the rates with the M10's high-rate setting active; without it the same configurations cap at 18 / 10 / 10 / 5 Hz.

Gnimu ships `GNSS_NAV_RATE_HZ 20` with **GPS + Galileo**, which is exactly the ceiling for two constellations. Bench testing on the M100-5883 matches the spec: 20 Hz holds rock-solid at 15–18 SVs, while asking for 25 Hz with both constellations enabled fluctuates between 24 and 25 Hz, because it is above the platform limit. Raising `GNSS_BAUD` does not help — the constraint is in the receiver, not the serial link.

The alternative the table allows is **25 Hz with GPS only**. That costs you the second constellation's geometry, and the accuracy difference is visible: GPS + Galileo at 20 Hz measures **1.3 pDOP and 0.24 m hAcc at 18 SVs**, meaningfully better than GPS alone at 25 Hz. For a device of this type the accuracy is worth more than the extra 5 Hz, which is why 20 Hz is the default. If you would rather have the rate, set `GNSS_NAV_RATE_HZ 25` and disable Galileo in `GNSS_CONSTELLATIONS`.

**Why the real RaceBox Mini does 25 Hz:** it uses a [u-blox NEO-M9N](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf), a different platform that does not derate at all — its datasheet lists 25 Hz for *every* configuration, from a single constellation up to GPS+GLO+GAL+BDS concurrently. It pays for that in power: 36–50 mA acquisition current against 6.5–10.5 mA for the [MAX-M10S](https://cdn.sparkfun.com/assets/7/5/9/a/a/MAX-M10S_DataSheet_UBX-20035208.pdf), and 100 mA peaks against 25 mA. The M10 is the right part for a small battery-powered device; the 20 Hz ceiling is what that choice costs.

---

## Usage

1. Power the assembled device and give the GNSS module time to acquire a fix. The onboard LED blinks while unconnected.
2. In the **RaceBox app** (or another RaceBox-compatible client), scan for and connect to the device — it advertises using the `MODEL` + `DEVICE_ID` name.
3. On connect, the LED goes solid and the device begins streaming data packets.
4. Optional: keep a serial monitor open at 115200 baud to watch live diagnostics.

---

## Troubleshooting

| Symptom | Things to check |
|---------|-----------------|
| `Failed to find IMU module` | I²C wiring (SDA/SCL), 3V3 power, board address. |
| `u-blox GNSS not detected` | UART wiring (note TX↔RX crossover), module power. The sketch will attempt to auto-configure the baud rate. |
| Few or no satellites | Move outdoors / near a window; lower `BLE_TX_POWER`; give it a cold-start minute. |
| App won't connect | Confirm `DEVICE_ID` is valid (10 digits, first digit 0–3); make sure no other client is already connected. |
| Build fails with a `static_assert` message | Read the message — it names the offending `config.h` value and the allowed range. |

---

## Acknowledgment & Origins

Gnimu began as a derivative of [**Anchit Chandra Sekhar's RaceBox mini emulator**](https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator). While that repository provided the foundational logic and initial inspiration, Gnimu has been completely overhauled from its original single-file Arduino sketch architecture.

**Key evolutions include:**
- Modular Architecture: Refactored into a highly modular codebase for improved maintainability.
- Externalized Configuration: Moved away from in-line constants to standard `config.h` approach.
- Transient-aware EMA smoothing: Applies exponential moving average (EMA) smoothing to IMU data, with transient thresholding to capture and integrate high-deviation signals that would otherwise be missed.
- Performance & Structure: Extensive cleanup and optimization of the core logic.

I am grateful to the original author for the initial implementation that made this project possible.

Protocol details follow the *RaceBox BLE Protocol Description (rev 8)*, [available from RaceBox](https://www.racebox.pro/products/mini-micro-protocol-documentation).

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](../../LICENSE).
