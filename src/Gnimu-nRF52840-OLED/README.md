# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3][License-shield]][License-link]
[![Platform: nRF52840][Platform-shield]][Platform-link]
[![Language: C++ (Arduino)][Language-shield]][Language-link]

This section of the repository is a further evolution of [Gnimu nRF52840][0]. The same **Seeed Studio XIAO nRF52840 Sense** [MCU][9], GNSS module, onboard IMU, battery subsystem, and BLE protocol, but it replaces the onboard RGB status LED with a small **SSD1306 OLED display**. Where the LED could only signal state through color and blink patterns, the display can show it directly as text, alongside GNSS quality information the LED never could: **locked satellites, pDOP, horizontal accuracy, PVT rate, and fix status**.

The advertised BLE identity and RaceBox Data Message protocol are unaffected. This variant differs only in its status/telemetry presentation, not in what it streams to a connected app.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

---

## What it does

This variant does everything [Gnimu nRF52840][0] does, plus:

- Shows device state (RUNNING / CHARGE_ONLY / BATTERY_WAIT / DEEP_SLEEP), BLE connection status, and battery charge/charging status on-screen, replacing the RGB LED's color/blink code with readable text.
- Shows GNSS fix quality that was previously only visible over serial: **satellites locked, pDOP, horizontal accuracy (hAcc), current PVT rate, and fix status**.
- Draws its status independent of the GNSS's power schedule. The display stays live and readable through states (like CHARGE_ONLY) where the GNSS is deliberately powered down, so it can always show at least charge/battery status.

See [Gnimu nRF52840's README][0] for everything this variant inherits unchanged: GNSS/IMU pipeline, BLE protocol, battery subsystem, and the RUNNING/CHARGE_ONLY/BATTERY_WAIT/DEEP_SLEEP state machine.

---

## Hardware

Everything from [Gnimu nRF52840's hardware list][0] applies unchanged, plus:

<table>
  <tr>
    <th width="30%" align="left">Part</th>
    <th align="left">Notes</th>
  </tr>
  <tr>
    <td>
        <a href="https://www.amazon.com/dp/B0D91NB1CP"><strong>SSD1306 0.96" 128×64 OLED (I2C)</strong></a>
    </td>
    <td>
        4-pin I2C-only module (VCC/GND/SCL/SDA), sold as a 2-pack with one white and one blue panel. I2C address <strong>0x3C</strong>. Powered from the XIAO's <strong>3V3</strong> pin.
    </td>
  </tr>
</table>

**Two other parts differ from [Gnimu nRF52840][0]**:

- **GNSS: [HGLRC M100 Mini](https://www.amazon.com/dp/B0BX65QZJ8)** instead of the M100-5883. Same u-blox M10 receiver, so the firmware is unchanged, but a smaller board that drops the QMC5883L compass this project never used.
- **Battery: [1000mAh flat LiPo](https://www.amazon.com/dp/B0DPZVBKMY)** instead of 900mAh (the 900mAh was unavailable). Slightly more runtime, and at the top of what the enclosure will take.

Plus one addition: **[JST 1.25mm 4-pin pre-crimped connector pairs](https://www.amazon.com/dp/B0DNTK1S9L)**, giving the GNSS and the display quick-disconnects so either can be lifted out without disturbing the shield wiring.

> [!TIP]
> **Use opposite genders on harnesses that share a connector type.** Several 4-pin runs in this build use the same part, so the wrong plug physically fits the wrong socket. Putting the male half board-side on one harness and the female half board-side on its neighbour makes an incorrect mating impossible to assemble — no labels to read. It matters most on the battery / buck-boost / `BAT±` runs, where a crossed connection carries raw cell voltage and can destroy the XIAO or the regulator.

The rest — XIAO, TPS63020, switch, other JST leads, resistors, USB-C adapter, project box — is identical to [Gnimu nRF52840][0]; see that README for part links and notes. The existing 45×75×20mm project box is a tight fit even before the display, so case fit is snug. Be careful when closing up the case or you might pinch some wires.

---

## Wiring

The display adds four connections on the XIAO's external I2C bus (`Wire`, separate from the IMU's internal `Wire1`):

| From XIAO | To | Notes |
|---|---|---|
| **D4** (SDA) | Display **SDA** | External I2C bus |
| **D5** (SCL) | Display **SCL** | External I2C bus |
| **3V3** | Display **VCC** | Always-on regulated rail |
| **GND** | Display **GND** | |

> [!IMPORTANT]
> **One connection differs from [Gnimu nRF52840][0]: the slide-switch sense divider moves from `A4` to `A1`.**
> On this board `A4` and `SDA` are the same physical pin, so the switch-sense line and the display cannot coexist there. Everything else in the base wiring is unchanged.

| From XIAO | To | Notes |
|---|---|---|
| **A1** | Switch-sense divider tap | 510kΩ/510kΩ from the slide switch's spare pole; **A1 here, not A4** (A1 sits opposite GND on the pad layout) |

---

## Software & dependencies

Same toolchain as [Gnimu nRF52840's Software & dependencies][0-software], including its macOS build gotcha, plus:

- **u8g2** (olikraus), via Library Manager — the display library, and required by the diagnostic sketches below. Chosen over Adafruit SSD1306 + GFX for its `updateDisplayArea()` partial updates. Used by `g_display.cpp`.

---

## Build & flash

Same as [Gnimu nRF52840][0-build], opening [`Gnimu-nRF52840-OLED.ino`][5] instead.

---

## Configuration

Settings shared by every Gnimu build are described in the [main README's Configuration section](../../README.md#configuration). Everything else in `config.h` matches [Gnimu nRF52840][0] — see that README's [Configuration section][0-config]. The settings below are where this variant **differs**; anything not listed behaves as documented in those two places.

| Setting | Purpose |
|---|---|
| `DISPLAY_ENABLED`, `DISPLAY_I2C_ADDRESS`, `DISPLAY_WIDTH/HEIGHT` | Panel presence, I2C address (`0x3C`), and geometry for the SSD1306 128×64. |
| `DISPLAY_REFRESH_INTERVAL_MS`, `DISPLAY_SLICE_INTERVAL_MS`, `DISPLAY_CHUNK_TILES_W` | Redraw cadence (1 Hz) and the metered chunk-at-a-time write that keeps a full frame's I2C cost off any single `loop()` pass. |
| `DISPLAY_SHIFT_INTERVAL_MS`, `DISPLAY_SHIFT_MAX`, `DISPLAY_LAYOUT_W/H` | Burn-in mitigation: the layout is inset by `DISPLAY_SHIFT_MAX` px and walks within that margin every 5 minutes. |
| `DISPLAY_CONTRAST` | 0–255; full scale by default for daylight readability. |
| `LED_ENABLED` | **`0` in this variant**. The display replaces the RGB status LED. `g_led.cpp` still checks `displayIsPresent()` at *runtime*, so the LED comes back automatically if the panel is missing at boot. |
| `POWER_SWITCH_SENSE_PIN` | **`A1` here, not `A4`**. On this board `A4` is `PIN_WIRE_SDA`, which the display needs. |

---

## Battery & power

Unchanged from [Gnimu nRF52840][0]. Same state machine, same low-voltage cutoff, similar estimated runtime. The one addition: both routes into DEEP_SLEEP (at runtime and at boot) blank the panel with the SSD1306's `DISPLAYOFF` command first, because System OFF doesn't cut its 3V3 rail and it would otherwise stay lit on a stale frame. BATTERY_WAIT and CHARGE_ONLY deliberately keep the panel lit.

---

## Usage

Follows [Gnimu nRF52840][0] with one difference: **status comes from the display, not the RGB LED.**

The screen shows what state the device is in, whether BLE is advertising or connected, battery percentage with a charging bolt, and GNSS quality (satellites, fix type, pDOP, horizontal accuracy, PVT rate). `BATTERY_WAIT` takes over the whole screen with a `Switch is OFF` alert.

The **onboard RGB LED stays dark**, since the enclosure puts it where you can't see it and the display says more. It comes back automatically as a fallback if no panel is detected at boot, so a display or wiring failure still leaves you with the LED signalling rather than a device that looks dead. Set `LED_ENABLED` to `1` in [`config.h`][config] to keep the LED active alongside the display for bench work.

The XIAO's own charge LED is wired to the charge controller and can't be driven by firmware, so it still lights while charging regardless of either setting.

---

## Troubleshooting

For everything not display-related, see the [main README's Troubleshooting table](../../README.md#troubleshooting) and [Gnimu nRF52840's][0-troubleshooting].

---

## Diagnostic sketches

- [`tools/oled_probe`](../tools/nRF52840-OLED/oled_probe/oled_probe.ino) — OLED bring-up and power characterization. Scans the I2C bus with raw `Wire` before any display library loads (so a wiring/power fault is distinguishable from a library problem), then holds test patterns on serial command: geometry (frame + corner ticks + crosshair, which catches a wrong panel size or controller variant immediately), font sizes, the Open Iconic Bluetooth glyph alongside drawn battery bars, a high-contrast screen for outdoor readability, and three discrete states for metering — all pixels on, controller active with nothing lit, and `DISPLAYOFF` sleep. Modes hold until the next keypress so a meter can be read without fighting a timer. Requires the **u8g2** library.

- [`tools/oled_bench`](../tools/nRF52840-OLED/oled_bench/oled_bench.ino) — update-cost benchmark and partial-update validation. `b` times full-frame and partial writes at the current bus clock, labelling each against the GNSS UART's ~5.5ms tolerance; `p` animates a counter inside one region against a static backdrop to prove `updateDisplayArea()` doesn't corrupt anything outside it; `f` gives the full-frame cost for comparison. `1`/`4`/`8` switch the I2C clock. This is the sketch that decided the display library.

- [`tools/imu_calibration`](../tools/nRF52840-OLED/imu_calibration/imu_calibration.ino) — per-chip IMU zero-point offsets, this tree's own copy. Warms up until the die temperature plateaus, then runs repeating 10000-sample sessions a minute apart, each gated on a stability check and appended to internal flash; press any key and then `a` to aggregate the run into six `#define`-formatted lines. **Those no longer feed `config.h`** — the six `IMU_*_OFFSET_*` defines were removed when `g_imu_trim` landed, and the firmware now learns the same correction at runtime; the sketch is kept as a bench diagnostic (see [`docs/imu-trim-design.md`](../../docs/imu-trim-design.md)). The measurement core is byte-identical to the [base tree's copy](../tools/nRF52840/imu_calibration/imu_calibration.ino), so results from the two are directly comparable. What differs is this variant's own settings baked in — the panel is brought up as part of the thermal load the die settles against (production keeps it lit), which also makes the run readable with **no USB attached**. Requires **u8g2**.

The remaining IMU/GNSS/battery diagnostic sketches are not duplicated here.

[License-shield]: https://img.shields.io/badge/License-GPLv3-blue.svg
[Platform-shield]: https://img.shields.io/badge/platform-nRF52840-00A9CE.svg
[Language-shield]: https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg
[License-link]: ../../LICENSE
[Platform-link]: https://wiki.seeedstudio.com/XIAO_BLE/
[Language-link]: https://www.arduino.cc/
[config]: ./config.h

[0]: ../Gnimu-nRF52840/README.md
[0-config]: ../Gnimu-nRF52840/README.md#configuration
[0-software]: ../Gnimu-nRF52840/README.md#software--dependencies
[0-build]: ../Gnimu-nRF52840/README.md#build--flash
[0-troubleshooting]: ../Gnimu-nRF52840/README.md#troubleshooting
[0-tools]: ../tools/nRF52840/
[5]: ./Gnimu-nRF52840-OLED.ino
[9]: https://en.wikipedia.org/wiki/Microcontroller
