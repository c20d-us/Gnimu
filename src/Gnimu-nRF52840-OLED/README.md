# Gnimu: GNSS+IMU data over BLE

[![License: GPL v3][License-shield]][License-link]
[![Platform: nRF52840][Platform-shield]][Platform-link]
[![Language: C++ (Arduino)][Language-shield]][Language-link]

This section of the repository is a further evolution of [Gnimu nRF52840][0] — same **Seeed Studio XIAO nRF52840 Sense** [MCU][9], GNSS module, onboard IMU, battery subsystem, and BLE protocol — that replaces the onboard RGB status LED with a small **SSD1306 OLED display**. Where the LED could only signal state through color and blink pattern, the display can show it directly as text, alongside GNSS quality information the LED never could: **locked satellites, pDOP, horizontal accuracy, PVT rate, and fix status**.

The advertised BLE identity and RaceBox Data Message protocol are unaffected — this variant differs only in its status/telemetry presentation, not in what it streams to a connected app.

> [!IMPORTANT]
> **Unofficial project.** This is an independent, educational, and non-commercial implementation. It is **not affiliated with, endorsed by, or supported by RaceBox.** "RaceBox" and related marks belong to their respective owner. Use this code for learning and personal purposes only, and at your own risk. Do not use this code to impersonate a genuine device for any commercial or fraudulent purpose.

> [!NOTE]
> **Status: pre-implementation.** This tree started as a fork of [Gnimu nRF52840][0]'s firmware (its IMU/GNSS/battery `tools/` sketches not duplicated) — the display hardware has been selected and bench-tested, and its power/wiring architecture worked out (see [`DESIGN.md`](DESIGN.md)), but the `g_display` module itself hasn't been written yet. Until it lands, this firmware behaves identically to the base nRF52840 build, RGB LED included. This README describes the intended end state; sections that don't apply yet are marked accordingly.

---

## What it does

Everything [Gnimu nRF52840][0] does — live 25Hz GNSS+IMU streaming over BLE as a RaceBox Mini-compatible device, battery power with a full state machine, low-voltage cutoff — plus, once `g_display` lands:

- Shows device state (RUNNING / CHARGE_ONLY / LIGHT_SLEEP / BATTERY_WAIT / DEEP_SLEEP), BLE connection status, and battery charge/charging status on-screen, replacing the RGB LED's color/blink code with readable text.
- Shows GNSS fix quality that was previously only visible over serial: **satellites locked, pDOP, horizontal accuracy (hAcc), current PVT rate, and fix status**.
- Draws its status independent of the GNSS's power schedule — the display stays live and readable through states (like CHARGE_ONLY) where the GNSS is deliberately powered down, so it can always show at least charge/battery status. See [`DESIGN.md`](DESIGN.md#3-power-architecture).

See [Gnimu nRF52840's README][0] for everything this variant inherits unchanged: GNSS/IMU pipeline, BLE protocol, battery subsystem, and the RUNNING/CHARGE_ONLY/LIGHT_SLEEP/BATTERY_WAIT/DEEP_SLEEP state machine.

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
        4-pin I2C-only module (VCC/GND/SCL/SDA), sold as a 2-pack with one white and one blue panel — <strong>white</strong> is the one used here, bench-tested against the blue. I2C address <strong>0x3C</strong>. Powered from the XIAO's <strong>3V3</strong> pin, not the TPS63020 GNSS rail — see <a href="DESIGN.md#3-power-architecture"><code>DESIGN.md</code></a> for why. No load switch / power-gating hardware — power-down uses the SSD1306's own <code>DISPLAYOFF</code> command instead.
    </td>
  </tr>
</table>

The rest of the bill of materials — XIAO, GNSS module, TPS63020, LiPo, switch, JST leads, resistors, USB-C adapter, project box — is identical to [Gnimu nRF52840][0]; see that README for part links and notes. The existing 45×75×20mm project box is a tight fit even before the display, so case fit is an open item (see [`DESIGN.md`](DESIGN.md#7-open-items)).

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
> On this board `A4` and `SDA` are the same physical pin, so the switch-sense line and the display cannot coexist there. Everything else in the base wiring is unchanged. See [`DESIGN.md`](DESIGN.md#the-switch-sense-pin-has-to-move-a4-is-sda).

| From XIAO | To | Notes |
|---|---|---|
| **A1** | Switch-sense divider tap | 510kΩ/510kΩ from the slide switch's spare pole; **A1 here, not A4** (A1 sits opposite GND on the pad layout) |

See [`DESIGN.md`](DESIGN.md#4-wiring-planned) for the reasoning behind the power source choice.

---

## Software & dependencies

Same toolchain as [Gnimu nRF52840][0], plus the display library:

- **[Arduino IDE][4]** (2.x recommended).
- **Board support — "Seeed nRF52 Boards"** (the **non-mbed**, Adafruit-nRF52-based core; **do not use** "Seeed nRF52 mbed-enabled Boards", which lacks Bluefruit). Add this Boards Manager URL, then install the package and select **Seeed XIAO nRF52840 Sense**:
  ```
  https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
  ```
- Libraries (install via Library Manager):
  - **Seeed Arduino LSM6DS3** (onboard IMU)
  - **SparkFun u-blox GNSS Arduino Library** (GNSS)
  - **u8g2** (olikraus) — the display library, and required by the diagnostic sketches below. Chosen over Adafruit SSD1306 + GFX for its `updateDisplayArea()` partial updates: a full-frame push costs ~31ms on this panel, far past the window the GNSS UART can tolerate, so partial updates aren't an optimization here but the only workable path (see [`DESIGN.md`](DESIGN.md#measured-update-cost)). Not wired into the firmware yet — `g_display` doesn't exist.

> [!IMPORTANT]
> **macOS build gotcha:** The Seeed nRF52 core's `platform.txt` invokes bare `python` for its UF2 step, but modern macOS only ships `python3`, so compiling fails with `exec: "python": executable file not found in $PATH`.
>
> **Fix:** in `~/Library/Arduino15/packages/Seeeduino/hardware/nrf52/<version>/platform.txt`, change `python` to `python3` on the `recipe.objcopy.uf2.pattern` line. (note: this reverts on every core reinstall/update, so it must be re-done afterward)

---

## Build & flash

1. Install the board package and libraries above.
2. Open [`Gnimu-nRF52840-OLED.ino`][5].
3. Edit [`config.h`][config] — at minimum, set your `DEVICE_ID`.
4. Select **Seeed XIAO nRF52840 Sense** as the board and the correct serial port.
5. Click **Upload**. If the upload can't reset into the bootloader (common with BLE/SoftDevice sketches), **double-tap the reset button on the XIAO** to force it, then upload again.
6. Open the **Serial Monitor** at **115200 baud** to watch startup and status output.

> [!IMPORTANT]
> If you are building on an Apple Silicon Mac, you can use the AS-native Arduino IDE but you **must** have Rosetta installed in order to correctly compile the binary. Without Rosetta installed you will get a compilation error.

As of today this produces firmware **identical in behavior** to [Gnimu nRF52840][0] — the display isn't wired into the build yet.

---

## Configuration

`config.h` is currently unmodified from [Gnimu nRF52840][0] — see that README's [Configuration section][0-config] for the full reference. Display-specific settings (I2C pins, refresh cadence, field layout) will be added here once `g_display` is implemented.

---

## Battery & power

Unchanged from [Gnimu nRF52840][0] — same state machine, same low-voltage cutoff, same estimated runtime. The one planned addition: `powerEnterDeepSleep()` and the other peripheral hold-off paths will send the display a `DISPLAYOFF` command before parking, the same way they already call `gnssEnd()` before holding GNSS pins low. See [`DESIGN.md`](DESIGN.md#3-power-architecture) for the reasoning, including the accepted DEEP_SLEEP standby-current tradeoff of not physically gating the display's power.

---

## Usage

Follows [Gnimu nRF52840][0] — charge the cell, slide the switch on, let the GNSS acquire, then connect from a RaceBox-compatible app — with one difference: **status comes from the display, not the RGB LED.**

The screen shows what state the device is in, whether BLE is advertising or connected, battery percentage with a charging bolt, and GNSS quality (satellites, fix type, pDOP, horizontal accuracy, PVT rate). `BATTERY_WAIT` — switch off while USB is plugged in — takes over the whole screen with a `Switch is OFF` alert.

The **onboard RGB LED stays dark**, since the enclosure puts it where you can't see it and the display says more. It comes back automatically as a fallback if no panel is detected at boot, so a display or wiring failure still leaves you with the LED signalling rather than a device that looks dead. Set `LED_ENABLED` to `1` in [`config.h`][config] to keep the LED active alongside the display for bench work.

The XIAO's own charge LED is wired to the charge controller and can't be driven by firmware, so it still lights while charging regardless of either setting.

---

## Troubleshooting

Same as [Gnimu nRF52840's troubleshooting table][0] for everything not display-related. Display-specific troubleshooting will be added once `g_display` exists.

---

## Diagnostic sketches

- [`tools/oled_probe`](../tools/nRF52840-OLED/oled_probe/oled_probe.ino) — OLED bring-up and power characterization. Scans the I2C bus with raw `Wire` before any display library loads (so a wiring/power fault is distinguishable from a library problem), then holds test patterns on serial command: geometry (frame + corner ticks + crosshair, which catches a wrong panel size or controller variant immediately), font sizes, the Open Iconic Bluetooth glyph alongside drawn battery bars, a high-contrast screen for outdoor readability, and three discrete states for metering — all pixels on, controller active with nothing lit, and `DISPLAYOFF` sleep. Modes hold until the next keypress so a meter can be read without fighting a timer. Requires the **u8g2** library.

- [`tools/oled_layout`](../tools/nRF52840-OLED/oled_layout/oled_layout.ino) — screen-layout mockup. Renders all five per-state screens with fake data so the layout can be judged on real glass: `1`/`2` RUNNING connected/advertising, `3` CHARGE_ONLY, `4` LIGHT_SLEEP, `5` BATTERY_WAIT, `6` display off. `w` swaps in worst-case field widths (layouts look fine on typical data and break on the extremes); `j` steps the burn-in pixel-shift offset to confirm nothing clips. This is the reference implementation of the layout spec in [`DESIGN.md`](DESIGN.md#screen-layout). Requires **u8g2**.

- [`tools/oled_bench`](../tools/nRF52840-OLED/oled_bench/oled_bench.ino) — update-cost benchmark and partial-update validation. `b` times full-frame and partial writes at the current bus clock, labelling each against the GNSS UART's ~5.5ms tolerance; `p` animates a counter inside one region against a static backdrop to prove `updateDisplayArea()` doesn't corrupt anything outside it; `f` gives the full-frame cost for comparison. `1`/`4`/`8` switch the I2C clock. This is the sketch that decided the display library — results in [`DESIGN.md`](DESIGN.md#measured-update-cost).

The IMU/GNSS/battery diagnostic sketches are not duplicated here — see [Gnimu nRF52840's `tools/`][0-tools].

---

## Design notes

[`DESIGN.md`](DESIGN.md) is the working engineering log for this variant — hardware selection, power architecture decisions, and open items, kept as a historical record rather than polished documentation. Start there for the reasoning behind the display's power source and wiring choices.

---

## Credits

Gnimu nRF52840-OLED is a further evolution of [**Gnimu nRF52840**][0], which is itself the battery-powered port of the original **Gnimu ESP32** build — a major evolution of the [**Open-Source RaceBox Mini Emulator**][6] by [**Anchit Chandra Sekhar**][7].

Protocol details follow the *RaceBox BLE Protocol Description*, [available from RaceBox][8].

---

## License

Released under the **GNU General Public License v3.0** — see [`LICENSE`](../../LICENSE). As a derivative of the GPL-v3 licensed Gnimu / Open-Source RaceBox Mini Emulator, Gnimu nRF52840-OLED carries the same license.

[License-shield]: https://img.shields.io/badge/License-GPLv3-blue.svg
[Platform-shield]: https://img.shields.io/badge/platform-nRF52840-00A9CE.svg
[Language-shield]: https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C.svg
[License-link]: ../../LICENSE
[Platform-link]: https://wiki.seeedstudio.com/XIAO_BLE/
[Language-link]: https://www.arduino.cc/
[config]: ./config.h

[0]: ../Gnimu-nRF52840/README.md
[0-config]: ../Gnimu-nRF52840/README.md#configuration
[0-tools]: ../tools/nRF52840/
[4]: https://www.arduino.cc/en/software
[5]: ./Gnimu-nRF52840-OLED.ino
[6]: https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator
[7]: https://github.com/anchit92
[8]: https://www.racebox.pro/products/mini-micro-protocol-documentation
[9]: https://en.wikipedia.org/wiki/Microcontroller
