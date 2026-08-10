# Gnimu nRF52840-OLED — Design Notes

Design reference for **Gnimu nRF52840-OLED**, an evolution of the battery-powered [Gnimu nRF52840](../Gnimu-nRF52840/DESIGN.md) build that replaces the RGB status LED with a small OLED display — able to show richer state and GNSS-quality information than a blink pattern can carry.

> **Status: pre-implementation.** This tree is currently a byte-for-byte fork of `nRF52840/Gnimu` (minus the tools sketches). No display code has been written yet — this document records the hardware and architecture decisions made ahead of that work, and the open questions still to resolve. See [Open items](#7-open-items).

---

## 1. Goals

- Add a **128×64 monochrome OLED** as the primary status/telemetry display, driven off state and sensor data the nRF52840 firmware already computes internally but doesn't currently expose.
- Replace what the RGB LED signals today (RUNNING / CHARGE_ONLY / LIGHT_SLEEP / BATTERY_WAIT / DEEP_SLEEP, BLE connection state, charging / charge level) with on-screen equivalents, plus fields the LED never could show: **locked SVs, pDOP, hAcc, PVT rate, and fix status**.
- Everything else — GNSS, IMU, BLE protocol, battery subsystem, state machine — carries over from [Gnimu nRF52840](../Gnimu-nRF52840/DESIGN.md) **unchanged**. This document only covers what's new for the display; see the base doc for everything else.

## 2. Hardware selection

| Item | Choice | Rationale |
|---|---|---|
| Display | **SSD1306 0.96" 128×64 OLED, I2C, 4-pin — blue** ([2-pack, white/blue][amazon-oled]) | Ubiquitous, cheap, mature Arduino library support. I2C-only (confirmed 4-pin, not a solder-jumper SPI/I2C board) keeps wiring to 4 wires. Both panels bench-tested 2026-08-04, white originally selected on appearance and brightness — **but the white unit cracked during lid mounting** (2026-08-06) and the spare blue unit is now the one in the build. See [Cracked display during mounting](#cracked-display-during-mounting-2026-08-06) below. Both enumerate at **0x3C**. |
| GNSS | **HGLRC M100 Mini** ([link][amazon-gnss]) — *differs from the other variants, which use the M100-5883* | Same u-blox **M10** silicon, so `g_gnss.cpp` and the SparkFun UBX library are unchanged; 3.3–5V, UART at 115200. Chosen for **size** — the case is tight once the display is in — and it drops the QMC5883L compass the firmware never used, freeing the SDA/SCL pins that were previously left unconnected. **Rated 10Hz**, same nominal rating the -5883 carries while running at 25Hz in practice; see [Open items](#7-open-items). |
| Battery | **1000mAh flat LiPo** ([link][amazon-lipo]) — *differs from the other variants' 900mAh* | The 900mAh cell was unavailable. ~11% more runtime, but it sits at the **top** of the ~1000mAh the enclosure was judged to accept, and that judgement predates the display. Built-in **PCM** confirmed (overcharge, over-discharge, overcurrent, overheating, short circuit) — the hardware over-discharge backstop the design depends on for DEEP_SLEEP and switch-OFF. |
| Connectors | **JST 1.25mm 4-pin, pre-crimped male/female pairs** ([link][amazon-conn]) | One part number covers both the GNSS (`VCC GND TX RX`) and the display (`GND VCC SCL SDA` — check the silkscreen, the order differs). Pre-crimped deliberately: proper JST crimp tooling costs more than the build, and hand-crimped 1.25mm terminals fail intermittently. These are PicoBlade-style friction fit with **no locking tab**, so the harness must be secured independently and the connector never left carrying load. See [Gender convention](#gender-convention-makes-mis-plugging-impossible). |
| Power source | **XIAO 3V3 pin**, not the TPS63020 GNSS rail | See [§3](#3-power-architecture) — decoupling the display's power schedule from GNSS's was the deciding factor. |
| Power gating | **None — no load switch.** Display stays on the always-on 3V3 rail; power-down is via the SSD1306 `DISPLAYOFF` command (~10µA datasheet sleep current), not a physical rail cut. | Simplest option — avoids an extra load-switch IC, an extra GPIO, and more board space in an enclosure that's already tight before the display's own footprint is even accounted for. Traded off against a small DEEP_SLEEP standby-current increase; see [§3](#3-power-architecture). |

### Display alternatives considered (and not chosen for v1)

- **Sharp Memory LCD** — much better direct-sunlight readability and far lower current (µA vs mA), which matters for a device meant to be glanced at outdoors/in-car. Not chosen for v1: SSD1306 is cheaper and simpler (I2C vs SPI + VCOM drive). **Now a fallback rather than an active candidate** — the SSD1306 was bench-checked in bright sun on 2026-08-04 and judged washed out but legible, which was the condition this alternative was being held against (see [Open items](#7-open-items)).
- **7-pin SPI/I2C-selectable OLED boards** — some cheap "0.96 OLED" listings are actually solder-jumper-selectable between I2C and SPI, which would need 3 extra GPIO (DC/RES/CS) in SPI mode. Confirmed the ordered board is the plain 4-pin, I2C-only variant.

### Cell specifications, and where the design sits inside them

Vendor figures for the 1000mAh pack, against what the firmware and hardware actually do:

| Cell spec | Design's use of it | Margin |
|---|---|---|
| Charge cut-off **4.2V** | `BATTERY_FULL_V` = 4.15V, deliberately below the CV target so ADC and charger tolerances cannot stop "full" ever latching | — |
| Discharge cut-off **2.75V** (PCM floor) | `BATTERY_CUTOFF_V` = 3.37V — firmware halts **0.62V above** the cell's own floor | Large, and intentional |
| Max charge rate **1C** (1000mA) | XIAO charger on the ~100mA pad = **0.1C** | 10× |
| Max discharge rate **1C** (1000mA) | ~35–45mA in RUNNING = **~0.045C** | ~22× |
| Capacity **1000mAh** @0.5C | Rated at a discharge 11× harder than ours, so the usable figure should if anything be optimistic in our favour | — |

The two voltage rows confirm the layering §6 describes: firmware's cutoff protects *cycle life* by stopping early during normal use, while the PCM's 2.75V is the hardware backstop for DEEP_SLEEP and switch-OFF, where no firmware is running. They are not redundant — they act in different regimes. `BATTERY_DISCHARGE_CURVE` bottoms at 3.37V/0%, matching the firmware cutoff exactly, so the reported percentage reaches zero precisely when the device shuts itself down.

Current draw is nowhere near the cell's limits in either direction, so rate is a non-issue.

### ⚠️ Temperature is the one spec this build can actually violate

**Operating 0–45°C, storage −10–45°C.** That is narrower than many LiPos and directly at odds with where this device lives.

- A car parked in summer sun routinely reaches **60–70°C** inside. That is well past both the operating *and* storage ceilings, and heat is what ages LiPo cells fastest.
- Winter takes the other end below 0°C. Discharging cold is merely inefficient — but **charging a LiPo below 0°C causes lithium plating**: permanent capacity loss, and a genuine safety hazard rather than just wear.

**Firmware cannot prevent this.** The XIAO's BQ25101 charges whenever USB is present and the slide switch is closed; the `HICHG` pad selects charge *current*, not enable, and no firmware-controllable enable line exists. So there is no automatic protection to add — the mitigation is procedural: **don't store the device in the car, and don't charge it cold.**

If monitoring is ever wanted, the only temperature signal on board is the LSM6DS3TR-C's own sensor, currently disabled (`tempEnabled = 0` in `g_imu.cpp`). It reads die temperature rather than cell temperature, but inside a sealed 45×75×20mm case the two should track closely enough to drive a warning on the display. That would be advisory only — it still could not stop the charger.

## 3. Power architecture

### Why not share the GNSS's TPS63020 EN gate

Electrically, the display would run fine off the TPS63020's 3.34V output — same voltage as 3V3, and trivial extra current on top of the GNSS's ~30mA. But gating the display with the *same* EN signal as GNSS ties its on/off schedule to GNSS's, and the two peripherals are off for unrelated reasons:

- **CHARGE_ONLY** cuts GNSS deliberately (so the charger gets full current) — but that's exactly the state where the display should keep showing charge/battery status, i.e. the direct replacement for the LED's green-blink behavior today. Sharing GNSS's gate would blank the display in the one state that matters most for it.
- **LIGHT_SLEEP** cuts GNSS on an ephemeris-staleness timer (`STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN`) that has nothing to do with when the display should blank for power saving.

So the display is powered from the **XIAO 3V3 pin** instead — live whenever the board itself has power, independent of the GNSS rail's schedule.

### Why no load switch for the display itself

A GPIO-controlled load switch (e.g. TI TPS22918) was considered, to fully de-power the display in low-power states — mirroring the GNSS EN pattern but on its own gate. Decided against it:

- Extra parts (load-switch IC) and an extra GPIO, in an enclosure that's already tight on space even before the display's own footprint is accounted for.
- The SSD1306's own `DISPLAYOFF` command drops current to ~10µA (datasheet max) without physically removing power — a couple of I2C command bytes, and no re-init needed on wake (unlike a hard power-cut, which would need a cold `begin()` again, the same cost GNSS pays after a full EN-cut).
- RUNNING / CHARGE_ONLY / LIGHT_SLEEP / BATTERY_WAIT all have the MCU actively polling anyway, so a few µA of sleep current is noise next to everything else already running in those states.

**Accepted tradeoff — DEEP_SLEEP standby current.** Unlike GNSS/IMU, the display's 3V3 supply isn't cut by `sd_power_system_off()` — that's a power mode internal to the nRF52840 chip; VDD (and therefore the board's own 3V3-generating regulator) stays live the whole time the slide switch is physically ON. So the display sits in `DISPLAYOFF` sleep — not truly unpowered — for the entire DEEP_SLEEP duration, which by design can be indefinite. The base nRF52840 DESIGN.md's DEEP_SLEEP target was "well under 10µA" for the whole system, benchmarked explicitly against LiPo shelf self-discharge as the "doesn't matter" yardstick — adding another ~10µA+ on top could roughly double that. DEEP_SLEEP is a rare backstop state (low-voltage cutoff / forgotten-device timeout), not everyday behavior, so this was judged an acceptable tradeoff for the space/complexity savings — but it **needs a real bench measurement** once the board is assembled (datasheet ~10µA is a floor; this specific module's onboard regulator may add more). See [Open items](#7-open-items).

`powerEnterDeepSleep()` (and the other hold-off paths — `enterBatteryWait()`, CHARGE_ONLY entry) will need to send `DISPLAYOFF` before parking, the same way they already call `gnssEnd()` before `powerHoldPeripheralsOff()`.

## 4. Wiring (planned)

Display uses the XIAO's **external I2C bus** (`Wire`, pins **D4 = SDA / D5 = SCL**) — completely separate from the IMU's internal `Wire1` bus (pins 17/16, onboard-only), so no bus sharing and no address conflicts. The Arduino core enables internal I2C pull-ups on D4/D5 by default; combined with whatever pull-ups the display board itself carries, this should be fine as parallel resistance, but worth confirming bus signal levels once wired.

| From XIAO | To | Notes |
|---|---|---|
| **D4** (SDA) | Display **SDA** | External I2C bus, unused in the base nRF52840 design |
| **D5** (SCL) | Display **SCL** | External I2C bus, unused in the base nRF52840 design |
| **3V3** | Display **VCC** | Always-on regulated rail — see [§3](#3-power-architecture) for why not the TPS rail |
| **GND** | Display **GND** | |

Bench-confirmed on breadboard 2026-08-04 (both modules, [`oled_probe`](../tools/nRF52840-OLED/oled_probe/oled_probe.ino)): the panel runs correctly on this wiring at **I2C address 0x3C**. That address becomes a `config.h` constant when `g_display` lands.

### Gender convention makes mis-plugging impossible

Several harnesses in this build use the same 4-pin JST 1.25mm connector, which means the wrong plug physically fits the wrong socket. Rather than rely on labels, **each harness pair uses the opposite gender to its neighbour** — male board-side on one, female board-side on the other — so an incorrect mating cannot be assembled at all.

Applied to the two pairings that could otherwise be confused:

| Pairing | Why it matters if crossed |
|---|---|
| GNSS ↔ display | Both are 4-pin, but the pin *orders* differ. Crossing them puts the XIAO's SCL output against the GNSS's TX output — two push-pull drivers fighting. Not instantly fatal at 3.3V, but real contention, and miserable to diagnose inside a sealed case. |
| Battery ↔ buck-boost ↔ XIAO BAT± | **Destructive.** These carry raw cell voltage; a reversed or swapped connection can take out the XIAO or the TPS63020 outright. |

The keying is worth more than the labelling because it survives being forgotten. Anyone reassembling this later — including its author — gets the constraint enforced by the parts rather than by remembering a convention.

### Cracked display during mounting (2026-08-06)

The white panel developed a **vertical crack** while being mounted to the lid and failed in two stages, both traced to the same physical damage rather than two separate faults:

1. **First symptom: total boot hang.** `displayBegin()`'s I2C probe (`Wire.beginTransmission()`/`endTransmission()`) never returned, so nothing after it in `setup()` ever ran — not state classification, not GNSS, not IMU, not BLE. Diagnosed by the *absence* of `g_display`'s own log lines: it always prints either the `⚠️ No OLED` warning or `✅ OLED display enabled.`, and neither appeared, which only happens if the transaction itself is stuck rather than cleanly failing.
2. **After reseating the harness: clean `⚠️ No OLED at 0x3C`.** The hang cleared, but the panel stopped answering at all.

**Root cause: SSD1306 modules are chip-on-glass** — the driver IC is bonded directly to the glass, with its traces running through that bond to the header pins. There is no flex tolerance. A *vertical* crack (across the panel, not at a corner) points to bending stress during mounting rather than an impact. The two-stage symptom is consistent with one crack: it likely first bridged two traces (a marginal short — the hang), then opened further during handling (an open circuit — the clean no-ACK).

**Confirmed by substitution**, the cleanest test available: swapping in the spare (blue) module with no other change fixed it immediately. **The build now uses the blue panel** — the hardware table above and the README are updated to match. The white unit is destroyed and was discarded.

**The mechanical root cause, established once the mounting method was known:** the lid has a rectangular cutout for the display, with holes drilled at the corners to match the PCB's mounting holes, secured with 2.5mm nylon screws and nuts. The white panel's PCB was mounted **flush against the underside of the lid, with nothing between them**. SSD1306 modules have the glass sitting *proud* of the PCB substrate — raised above it by a small lip, since the glass sits over components and traces on the board. Mounted flush, the glass (not the PCB) is the first thing to contact the lid, so tightening the corner screws pulls the PCB's corners down while the glass — now acting as the fulcrum — resists, bowing the board across exactly the axis that produced a *vertical* crack.

**The fix used for the blue panel is a double-nut standoff**, built from parts already on hand rather than sourced hardware: thread a nut partway down each screw first (this becomes the spacer), set the screw into the lid, rest the PCB's corner holes on top of that nut, then add a second nut on top to clamp the PCB down against the standoff. The first nut's height sets a fixed, repeatable gap between the PCB and the lid — large enough that the glass, which sits proud, still clears the lid with margin. Clamping load lands on the PCB at the standoff, never on the glass.

**Open item:** confirm this clearance survives final assembly (harness routed, any gasket/foam added, lid fully seated) — the gap here is deliberately small ("just enough"), so anything that shifts the lid or pulls on the connector even slightly is worth checking doesn't close it back up.

Consequence for the outstanding `g_display` robustness gap: a *marginal* version of this same fault — a joint or trace stressed but not fully broken — would reproduce the original hang, and `g_display.h`'s stated goal ("a missing display should not stop the device streaming telemetry") does not hold for a stuck bus, only a cleanly absent one. Worth a timeout guard around the I2C probe regardless of what caused this specific incident, since the failure mode is real and this is the second time it's been hit this build. **Investigated 2026-08-06, deferred rather than implemented** — see the dedicated open item below for what was found and why it's non-trivial.

### The switch-sense pin has to move: A4 *is* SDA

**Resolved 2026-08-04 — `POWER_SWITCH_SENSE_PIN` is `A1` in this tree, not `A4`.**

On the XIAO nRF52840 the analog aliases and the I2C pins are the same silicon. From the core's `Seeed_XIAO_nRF52840_Sense/variant.h`:

```
#define PIN_A4         (4)      #define PIN_WIRE_SDA   (4)
#define PIN_A5         (5)      #define PIN_WIRE_SCL   (5)
```

The base design puts the slide switch's 510kΩ/510kΩ sense divider on **A4** — the signal [§6 of the base doc](../Gnimu-nRF52840/DESIGN.md#battery-presence-switch-sense) calls the linchpin of load-independent battery-presence detection, polled by `powerSwitchOn()` every loop. This variant puts the OLED on the external I2C bus. Those are the same two pins.

The two cannot share it. The sense read needs the pin in analog-input mode while I2C needs it owned by the TWI peripheral, and the switch is polled continuously — no sequencing fixes that. Independently, the divider is a permanent resistive path to a ~2V node whenever the switch is off, which would fight the bus pull-ups.

Moving the display was rejected: `Wire` on D4/D5 is the only hardware I2C broken out (`Wire1` is internal to the IMU on pins 17/16), and the alternatives — bit-banged I2C or an SPI panel — both give up transfer speed, which [§5's measurements](#measured-update-cost) show is the binding constraint.

So the sense line moves to **A1**, the divider tap being trivially relocatable in a build that doesn't exist yet. A1 sits directly opposite `GND` on the XIAO's pad layout — left-side position 2 against right-side position 2 — which is the divider's other connection, so the two legs land across from each other instead of at a board corner. A0, A2 and A3 are equally free electrically; this is a build-layout preference.

Consequences worth recording:

- **The base nRF52840 tree is unaffected.** No display, so A4 is correct there and the assembled unit needs no rework. This is the first *deliberate* `config.h` divergence between the two trees — they were byte-identical before.
- **A5/SCL is clean.** Nothing in the design used it.
- This was originally logged as a "verify D4/D5 are free of conflicts" item on the reasoning that nothing in the base design uses the `Wire` bus. That was true and irrelevant: the base design doesn't use `Wire`, but it does use A4, and on this board that *is* SDA. **Check pin numbers, not peripheral names.**

## 5. Firmware architecture (planned)

**Implemented 2026-08-04** as `g_display.cpp` / `.h`, wired into the sketch's `setup()`/`loop()` and into `g_state`'s two DEEP_SLEEP paths. This section records the design; the module is the authority on detail.

- New module **`g_display.cpp` / `.h`**, structured like `g_led` today: owns the SSD1306, polled once per loop, reads the same public queries `g_led` already reads (battery %/charging, `bleIsConnected()`, current state) plus GNSS quality fields not currently surfaced outside `g_gnss`.
- **Most GNSS fields needed are already in-hand.** `numSV`, `pDOP`, and `hAcc` are fields on the same `UBX_NAV_PVT_data_t` struct `g_gnss.cpp` already consumes via `setAutoPVTcallbackPtr` — no new UBX polling required, just plumbing already-received fields out to `g_display`. **Fix status** and **PVT rate** still need mapping: fix status is likely `fixType` off the same NAV-PVT struct; PVT rate isn't a struct field, so it'll need to be derived (time between successive callback firings, or read back from the `MAX_NAVIGATION_RATE` config value already used to set `setNavigationFrequency`).
- `g_state` gains one more peripheral to hold off/wake at the existing transition points (LIGHT_SLEEP, BATTERY_WAIT, CHARGE_ONLY, DEEP_SLEEP) — a `DISPLAYOFF`/redraw call at each, not new states.
- **Redraw throttling.** A full-frame push costs ~31ms at 400kHz (measured — see [Measured update cost](#measured-update-cost)) against the ~5.5ms GNSS UART budget, so it can never be one blocking call. `g_display` renders into RAM in one go, then pushes 64-byte slices one per `loop()` iteration — the same non-blocking-state-machine shape as the `g_battery` VBAT sampler. Refresh cadence is `DISPLAY_REFRESH_INTERVAL_MS` (400ms, 2.5Hz), well under the GNSS's up-to-25Hz PVT rate since nothing on screen is worth reading that fast.

### Display library: Adafruit_SSD1306 vs u8g2

**DECIDED 2026-08-04: u8g2**, on measurement rather than preference — see [Measured update cost](#measured-update-cost) below. The dependency lists originally named Adafruit_SSD1306 + Adafruit_GFX, which was the default assumption when the hardware was picked, not a decision.

| | **Adafruit_SSD1306 + GFX** | **u8g2** |
|---|---|---|
| Partial updates | `display()` only — always pushes the full 1024-byte frame | `updateDisplayArea(tx, ty, tw, th)` sends only a tile sub-region (8px units) |
| Buffer modes | Full buffer (1KB RAM) | Full (`F`, 1KB), page (`1`/`2`, 128/256 B) |
| Fonts | One built-in 5×7 face, scaled integer multiples; CP437 extension for box/arrow glyphs | Large bundled set at many sizes, including **Open Iconic** icon fonts |
| Icons | Roll your own `PROGMEM` bitmaps via `drawBitmap()` | `u8g2_font_open_iconic_*` — themed subsets; the `embedded` set has battery-empty/full and Bluetooth (glyph 74 in `..._embedded_2x_t`, 16×16) |
| Footprint | Smaller, simpler API | Larger, but themed font subsets keep flash pay-for-what-you-use |
| Familiarity | Ubiquitous, matches most SSD1306 tutorials | Own API idioms to learn |

**u8g2 is the current favorite, on the partial-update point.** The redraw-throttling problem above is really a *bytes-per-update* problem, and most of this screen is static labels — only the numbers change between frames. `updateDisplayArea()` turns a ~25ms full-frame write into a fraction of that, directly, with no page-addressing code of our own. With Adafruit_SSD1306 the same optimization means driving the SSD1306's page/column addressing by hand around its buffer, which is exactly the sort of thing that works until it doesn't and then costs a day.

The icon fonts are a genuine secondary convenience (Bluetooth, satellite, and similar static marks come free instead of as hand-built bitmaps), but they are not the deciding factor — see the note on the battery indicator below.

Neither library's resource cost matters here. The 1KB framebuffer sits against 256KB of RAM and fonts sit against 1MB of flash; the RAM pressure that dominates SSD1306 design on AVR parts is simply absent on this one. So the choice should be made on API fit, not footprint — which is what points at u8g2.

### Data sources for the displayed fields

**Mapped 2026-08-04.** Most of what the layout shows is already computed somewhere in the firmware; only two items need new plumbing, and both are traps rather than work.

| Display field | Source | Ready? |
|---|---|---|
| SV count | `pvt->numSV` | yes |
| Fix status | `pvt->fixType` — the **raw** value, not the protocol clamp | yes |
| pDOP | `pvt->pDOP` ÷ 100 | yes |
| hAcc | `pvt->hAcc` (mm) | yes |
| PVT rate | `telemetryGnssRateHz()` | yes |
| Battery %, charging, full | `batteryGetStatus()` | yes |
| BLE connected | `bleIsConnected()` | yes |
| Device state | `g_state` | yes |
| Runtime | `millis()` | yes |
| *(all PVT access)* | `gnssLatestPvt()` — non-consuming | yes |

**Fix status: use the raw `fixType`, and reuse the existing position-valid predicate.** `g_telemetry`'s `sendPacket()` clamps `fixType` to `{0, 2, 3}` because the RaceBox protocol defines nothing else — that clamp belongs to the wire format and must not leak into the display. For the `--` rule, the codebase already has a canonical test: `sendPacket()` marks Lat/Lon invalid when `fixType < 2`. `g_display` should use the identical condition so the screen and the packet can never disagree about whether a position exists. Note the consequence: **a 2D fix is position-valid** — hAcc and pDOP are real numbers there, merely worse — so the `--` case is `fixType < 2`, not "anything but 3D".

**PVT rate had been compiled out of production builds — fixed 2026-08-04.** The epoch counter incremented in `telemetrySendIfReady()`, outside any guard, but the rate *computation* and the counter *reset* both sat inside `#if LOG_ENABLED` in `telemetrySerialReport()`. With `LOG_ENABLED = 0` — the configuration the README recommends for shipping, since it sheds the loop-latency spike that forced `GNSS_BAUD` down to 115200 — the rate was never calculated and the counter grew unbounded. The display's rate field would have read zero in exactly the build intended to ship. Rate accounting now lives in an unconditional `updateRates()` behind `telemetryGnssRateHz()` / `telemetryBleRateHz()`, with the serial report as one consumer rather than the owner. Note the window check had to move out of the guard too: hoisting only the arithmetic would have left the window never closing in a silent build. `telemetryBleRateHz()` came along because it had the identical defect.

**`gnssConsumePvt()` is consume-once — `g_display` must not call it.** It returns `nullptr` unless a new epoch has arrived and clears the flag on the way out, and `g_telemetry` already calls it every loop. A second caller would race: whichever ran first that iteration takes the epoch and the other sees `nullptr`. Depending on call order that is either dropped BLE packets or a display updating at half rate — and it would present as an intermittent glitch rather than a design error. **`gnssLatestPvt()` (added 2026-08-04) is the accessor read-only observers must use**: it returns `&latestPVT` without touching the flag, gated by a latching `everReceivedPvt` so callers can tell "no data yet" from "data, but stale". It says nothing about age — in LIGHT_SLEEP or after an EN-cut the last epoch simply stops advancing, and judging that is the caller's job. This also keeps `g_display` independent of `g_telemetry`: both are readers of `g_gnss`.

### Measured update cost

Bench-measured 2026-08-04 at 400kHz via [`oled_bench`](../tools/nRF52840-OLED/oled_bench/oled_bench.ino). The reference budget is the **~5.5ms Serial1 RX window** from the base design — the ~64-byte UART buffer at `GNSS_BAUD` 115200 — since any blocking write longer than that drops NAV-PVT bytes and craters the observed rate.

| Transfer | Bytes | Time | vs. 5.5ms budget |
|---|---|---|---|
| Full frame (`sendBuffer()`) | 1024 | **31ms** | 5.7× over |
| Half screen | 512 | 16ms | 3× over |
| One text row | 256 | 7.8ms | over |
| One 8px page | 128 | 3.9ms | 71% |
| Number field (4×2 tiles) | 64 | 2.0ms | 36% |
| Small field (2×1 tiles) | 16 | <1ms | ~18% |

Cost is **linear at ≈30.5µs/byte** (74% of the theoretical 22.5µs at 400kHz; the rest is I2C overhead). There is **no meaningful per-call penalty** — splitting a redraw into smaller chunks costs nothing in total and only reduces peak blocking time.

> **Measurement caveat:** this core derives `micros()` from a 1024Hz FreeRTOS tick, so timings quantize to ~977µs. Every raw figure was an exact multiple of it. Large transfers are reliable; the 16-byte row sits at one quantum, so its true cost (~490µs by the model) is below what the timer can resolve. An earlier fit of these numbers produced a phantom "~560µs fixed per page" term that was pure quantization artifact — **don't design around a per-call overhead that doesn't exist.**

Three conclusions, the second of which was revised once `g_display` was actually written:

1. **A blocking full redraw cannot ship.** At 5.7× over budget it would cost GNSS data several times a second — the same failure mode the base design already hit once with a 5ms printf. This is what rules out `Adafruit_SSD1306`, whose only path to the panel is `display()`.

2. **Separate rendering from pushing, and meter only the push.** Drawing into u8g2's RAM framebuffer costs no I2C at all, so a full re-render is effectively free and can happen in one go. Only the transfer is expensive, so that is the only part that needs rationing: `g_display` renders a whole frame, then pushes it as fixed-size slices, one per `loop()` iteration.

    `DISPLAY_CHUNK_TILES_W` (8 tile columns = 64 bytes ≈ **2ms**, about a third of the budget) sets the slice size, giving 16 slices per frame. **It is a latency control, not a throughput one** — the cost model is linear with no per-call penalty, so slicing changes nothing about total bytes and everything about peak blocking time.

    > **Revised from "update individual fields, not pages."** The original conclusion here assumed most of the screen is static labels, making field-level dirty tracking a large saving. It isn't: hAcc, pDOP and the PVT rate all change at *every* refresh, so nearly the whole screen is dirty each time and dirty tracking would buy little for real bookkeeping. Uniform slicing is simpler and meets the constraint that actually matters. If a future layout does become mostly static, field-level targeting is the optimization to reach for — the measurements above still support it.

3. **The pixel shift needs no special handling.** A shift dirties the entire frame, which would be a problem for a dirty-tracking design. Under uniform slicing every frame is a full push anyway, so the shift only has to advance *between* frames — never mid-push, or it would tear. `g_display` steps it in the same branch that starts a render, which makes that ordering structural rather than something to remember.

`updateDisplayArea()` was validated visually over 400 frames against a static hatched backdrop with no tearing or corruption outside the updated region.

### Field result: the constraint was density, not volume

The bench numbers establish the *cost* of a transfer. What they could not predict was which property of the traffic actually hurts — and the first two attempts got that wrong.

**Attempt 1 — smaller slices.** At `DISPLAY_CHUNK_TILES_W = 8` (64 bytes, ~2ms per call) the GNSS rate sagged to a wandering 15–25Hz. Isolated 2026-08-05 by raising the refresh to 60s, which restored a solid 25Hz and pinned the cause on the display rather than anything else in the system. Halving to 4 tiles (~1ms) improved it, but the rate still fell to 23–24Hz regularly.

**Attempt 2 — spacing the slices.** Adding `DISPLAY_SLICE_INTERVAL_MS = 20`, so slices go out 20ms apart instead of on consecutive `loop()` iterations, restored a steady 25Hz.

**The lesson: total I2C time was never the constraint.** A whole frame is ~31ms spread across a second — roughly 3% duty, trivially affordable. What broke the GNSS was pushing those slices *back to back*, which left the UART without a clear stretch in which to be drained. The RX buffer fills in ~5.5ms at `GNSS_BAUD`, and a burst of consecutive transfers never gave it that long. Spacing at 20ms leaves about four fill windows between hits, and the same total bytes then cost nothing.

So `DISPLAY_CHUNK_TILES_W` (how big each push is) matters far less than `DISPLAY_SLICE_INTERVAL_MS` (how close together they are). **Reach for the spacing first when tuning.**

**A second benefit: the timing became analyzable.** One slice per iteration made frame duration depend on loop speed, which is unknowable at compile time — so the runaway condition (a push outlasting the refresh interval, leaving the display pushing continuously) could only be a warning in a comment. With explicit spacing the duration is `slice count × interval`, and `config.h` now carries a `static_assert` that catches a bad combination at build time and names the three ways out.

Current settings: 4-tile slices, 20ms apart, 1Hz refresh — a 640ms push window inside each second, ~5% bus duty while pushing, 360ms idle.

**This also removed the need for a reduced connected-mode screen.** A cut-down layout for when a BLE client is attached was designed and costed (full screen = 32 slices; a two-row dynamic region would have been 8), but proved unnecessary once spacing fixed the underlying problem. Worth remembering as the next lever if on-screen content grows enough to reopen this.

### Screen layout

**Settled 2026-08-04** on hardware via [`oled_layout`](../tools/nRF52840-OLED/oled_layout/oled_layout.ino), which renders every screen below with fake data. Layout at this size can't be judged on paper — legibility and density have to be looked at on the panel — so that sketch is the reference implementation, not this prose.

**Structure: a persistent status bar over a per-state body.** The screen splits at y=12. The bar is identical everywhere the cell is in circuit; the body shows only what the current state can actually know. That per-state split is the thing the RGB LED could never do — it shows what's relevant *now* rather than encoding every condition into one blink pattern.

```
[BT] Connected            [USB] [███░]  <- status bar (rows 0..11)
────────────────────────────────────
11 SV            3D                    <- body (rows 14..61)
pDOP 1.42            25.0Hz
hAcc 248mm          1:23:45
```

| State | Status bar | Body |
|---|---|---|
| **RUNNING** | `Connected` / `Advertising` + USB + battery gauge | SV count and fix type large; pDOP, PVT rate, hAcc, runtime below |
| **CHARGE_ONLY** | `Charging` / `Full` + battery, no BLE icon | Cell voltage only — large, centred |
| **LIGHT_SLEEP** | `Advertising` + battery | `IDLE - Sleeping` + `shake or connect to wake` |
| **BATTERY_WAIT** | *(none)* | Full-screen alert: inverted block, `Switch is OFF` + instructions, all centred |
| **DEEP_SLEEP** | — | `DISPLAYOFF` |

Four decisions in that table are load-bearing rather than cosmetic:

- **The RUNNING bar shows the BLE link state, not the state name.** RUNNING is the implicit default; spending scarce bar width restating it is wasteful, while the link state is what actually changes and what the user is checking for. A consequence worth knowing: RUNNING-while-advertising and LIGHT_SLEEP show an identical status bar, since LIGHT_SLEEP genuinely is still advertising. The body distinguishes them.
- **CHARGE_ONLY shows no GNSS data at all.** The receiver is held off in that state, so any fix numbers would be stale — displaying them would be actively misleading. Percentage and charge state already live in the bar, so the body carries only the voltage.
- **`Full` must come from `g_battery`'s voltage-based flag** (`charging && voltage >= BATTERY_FULL_V`), not from percentage hitting 100. Percentage is a lookup through `BATTERY_DISCHARGE_CURVE`, which is itself an open item for tuning against this cell. (The mockup proxies it off percentage only so the label can be demonstrated.)
- **BATTERY_WAIT drops the status bar entirely.** The slide switch has taken the cell out of circuit, so a battery percentage there is meaningless. It's also the one state that is a genuine error condition, which earns the full screen and the inverted block.
- **The battery shows as a gauge only — no percentage.** The bar conveys the level well enough at a glance, and a number implied more precision than a voltage-derived SoC estimate actually has, particularly with `BATTERY_DISCHARGE_CURVE` still untuned for this cell. Dropping it also freed the width the USB icon uses.
- **A USB icon indicates plugged-in, replacing an earlier charging bolt.** The bolt was flickering on the bench, and the change stands on its own merits — `usbPresent` is a hardware register with no threshold to cross, so the indicator no longer depends on any analog comparison. The icon is hand-drawn for the same reason the bolt was: Open Iconic's `embedded` subset has no USB glyph, and its vector downscales lose their shape at 8×8. Note `bat.charging` is `usbPresent && switchOn`, but every state that draws a status bar already has the switch on, so inside the bar it is exactly "USB plugged in".

    > **The flicker's actual cause was a floating input, diagnosed 2026-08-05 — not the `full` flag, as first assumed.** The USB icon flickered too, and it is gated on `bat.charging` alone with no `full` involved. `charging` is `usbPresent && switchOn`; `usbPresent` reads `NRF_POWER->USBREGSTATUS` and cannot chatter, which left `powerSwitchOn()` — an `analogRead()` on the switch-sense divider. **The slide switch and divider simply were not on the breadboard**, so that pin was floating, and a high-impedance input sampled with a 40µs acquisition window drifts unpredictably across the 800mV threshold. A bench-setup gap, not a firmware defect. Worth remembering that this variant's switch-sense moved to A1 and has *never* been verified on hardware, so it is the first thing to suspect whenever `switchOn` misbehaves here.

- **With no fix, pDOP and hAcc must render as `--`.** This is a correctness requirement, not cosmetics. u-blox reports a large sentinel `hAcc` and a meaningless `pDOP` when there is no solution; drawing them raw puts plausible-looking numbers on screen that read as *"the fix is poor"* rather than *"there is no fix"*. **Gate on `fixType`, never on a magnitude threshold over the values themselves.** The PVT rate stays live and is the one field still informative in that state, since NAV-PVT keeps arriving at the configured rate regardless of solution status. `No Fix` is also the widest string the fix field takes — 60px at 10x20, against `11 SV` ending at x=52 — and is checked on hardware as the third `w` dataset in the mockup.

**Wording note:** the alert reads `Switch is OFF` — an observation of state. An imperative like "SWITCH OFF" reads as an instruction to switch something off, the opposite of the required action.

### Layout area and burn-in pixel shift

The device displays a near-static screen for up to 8 hours per charge, which is the OLED differential-aging case. Mitigation is **pixel shifting**: nudge the whole layout on a slow cycle so no element sits on the same pixels indefinitely.

The rule that makes this workable:

- Content is laid out for a **126×62 area** with its origin at the panel's top-left.
- The shift is **positive-only, 0..2 px on each axis**, sliding that area into a 2px gutter along the right and bottom edges.
- Every draw call routes through an offset helper. Full-width elements use the layout width, not 128.

A symmetric ±shift was tried first and is the obvious thing to reach for — it clips anything sitting at x=0 or y=0, notably the status bar's Bluetooth icon, because there is no gutter at the top-left to give. One-directional shift means layout code only reserves slack on two edges instead of four. The mockup walks the perimeter of a 3×3 grid so each element visits 8 distinct positions rather than sliding along a single diagonal.

**Interaction with partial updates:** a pixel shift dirties the entire frame, defeating `updateDisplayArea()`. These coexist by shifting rarely (minutes apart) with one full redraw at each shift, and partial updates in between. Both optimizations are wanted; they just have to know about each other.

Build the offset in from the start — retrofitting it means touching every coordinate in every screen.

### Fonts in use

All from u8g2's bundled set; nothing custom. Right-aligned and centred fields measure with `getStrWidth()` rather than assuming a fixed advance, since not every `_tf` font is fixed-pitch.

| Role | Font |
|---|---|
| Status bar text | `5x7_tf` |
| Body detail lines | `6x12_tf` |
| Body headline (SV / fix) | `10x20_tf` |
| LIGHT_SLEEP headline | `7x14B_tf` |
| BATTERY_WAIT alert | `9x18B_tf` — 10x20 would need 130px for `Switch is OFF` against 126 available |
| Charge voltage | `logisoso24_tn` (numerals-only subset) + `7x14B_tf` for the `V` |
| Bluetooth icon | `open_iconic_embedded_1x_t`, glyph 74 |

Worst-case field widths (2-digit SV, `19.99` pDOP, 4-digit hAcc, `100%`, `23:59:59` runtime) were checked on hardware and all fit; `hAcc 9999mm` against `23:59:59` on one row is the tightest pairing in the design.

## 6. IMU axis remapping — generalized to all 24 orientations

**Status: designed, not yet implemented.** This section is the build plan. Nothing in `config.h` / `g_imu.cpp` has changed yet beyond the `IMU_SIGN_Y` correction described under [Settling the target frame](#settling-the-target-frame-2026-08-04).

### Why the existing model runs out

The base nRF52840 design corrects mounting orientation with four macros — `IMU_SWAP_XY` plus `IMU_SIGN_X/Y/Z` — applied by `remapAxes()` in `g_imu.cpp`. That covers **8 orientations**: any 90° yaw rotation, right-side-up or upside-down, and `config.h` says so explicitly ("It does NOT cover mounting the board on any edge"). It was sufficient because both prior builds mount the XIAO flat, so sensor Z was always vehicle-vertical.

This build breaks that assumption. The display claims the underside of the lid, which is where the XIAO was mounted, so the XIAO moves to the **end wall** — PCB vertical. Sensor Z becomes a *horizontal* vehicle axis (fore/aft), and one of the in-plane axes becomes vehicle-vertical. That's a 3-axis permutation, outside the flat model's reach entirely.

The generalization target is the full set of **24 physically-realizable orientations** (6 axis permutations × 8 sign combinations, halved by the handedness constraint below).

### Settling the target frame (2026-08-04)

A remap is only meaningful against a defined output frame, and the RaceBox protocol documentation turned out to be actively misleading on this point. Resolved empirically:

| Source | X | Y | Z |
|---|---|---|---|
| Upstream emulator (raw sensor, USB-rearward flat mount, no remap at all) | forward+ | left+ | up+ |
| Protocol doc's own worked example packet, decoded by RaceBox (p. 8) | — | — | **up+** (`GForceZ = +0.974 g` on a stationary device, speed 0.126 kph) |
| RaceBox Mini user manual mounting rules | — | — | up+ (logo to sky) |
| Protocol doc **axis figure** (p. 7) | ~~rearward+~~ | ~~right+~~ | ~~down+~~ |

**Target output frame: X forward+, Y left+, Z up+ — ISO 8855, right-handed.** Three independent sources agree; the figure is the sole outlier.

The figure is wrong in a specific and explicable way: **all three arrows are drawn inverted**, as if rendered from inside the device looking out. Established by locating the charging port in the drawing — the product photo on RaceBox's site shows the status LED sits on the same face as the USB-C port, and the figure draws that LED at the edge its +X arrow exits, so per the manual's "charging port towards the back of the vehicle" the figure's +X points rearward. Working the other two axes through the same geometry gives +Y = vehicle right and +Z = down. Negating all three axes of a right-handed frame also flips its handedness, which explains why the drawn frame reads as left-handed while every real implementation is right-handed — one systematic error, not three independent ones.

Practical consequence: **the target frame is right-handed, so no protocol-convention layer is needed.** The sensor→protocol map is a pure mounting rotation, and the determinant assert below is unconditionally correct. A left-handed target would have forced a two-layer design (rotation + explicit handedness flip); it doesn't.

This also surfaced a real bug in the shipped config, since fixed: `IMU_SIGN_X` had been flipped to `-1.0f` without `IMU_SIGN_Y`, giving a determinant of −1 — a mirror, not any physical mounting. Bench-confirmed by static tilt against the 1 Hz serial `milliG` output and corrected to `IMU_SIGN_Y = -1.0f` in both nRF52840 trees on 2026-08-04. See [Open items](#7-open-items) for the follow-ups.

### The scheme

Each **vehicle** axis names which **sensor** axis feeds it, plus a sign. Replaces `IMU_SWAP_XY` + `IMU_SIGN_*` outright — one source of truth, no overlap between two mechanisms describing the same thing.

```c
// Order XYZ (identity) - the "normal" reference mounting:
// flat, SoC face up, USB-C pointing rearward.
#define IMU_AXIS_X_SRC   0      // vehicle forward <- sensor X
#define IMU_AXIS_X_SIGN  +1.0f
#define IMU_AXIS_Y_SRC   1      // vehicle left    <- sensor Y
#define IMU_AXIS_Y_SIGN  +1.0f
#define IMU_AXIS_Z_SRC   2      // vehicle up      <- sensor Z
#define IMU_AXIS_Z_SIGN  +1.0f
```

`remapAxes()` needs a temp copy — a permutation can't be done in place:

```c
static void remapAxes(float t[3]) {
  const float in[3] = {t[0], t[1], t[2]};
  t[0] = in[IMU_AXIS_X_SRC] * IMU_AXIS_X_SIGN;
  t[1] = in[IMU_AXIS_Y_SRC] * IMU_AXIS_Y_SIGN;
  t[2] = in[IMU_AXIS_Z_SRC] * IMU_AXIS_Z_SIGN;
}
```

Everything around it is unchanged: still called on accel and gyro alike, still after the per-chip offset subtraction.

A **bitmap encoding was considered and rejected.** The proposal was one bit per axis indicating "is this axis in its original position," plus three sign bits. It fails because fixed-point count doesn't identify a permutation of three elements: the two 3-cycles `ZXY` and `YZX` both have zero axes in place and collide on the same code — and 3-cycles are exactly what an edge mount produces. It also presents the permutation and sign bits as independent when they're coupled (see the parity rule below), and `0b010110` needs a decoder ring at 7am at a track, where `IMU_AXIS_X_SRC 2` reads on sight. The permutation *names* from that idea are worth keeping, as comments.

### Compile-time validation

`config.h` already validates most settings with `static_assert`; this fits the existing pattern.

- Each `_SRC` in 0–2.
- Each `_SIGN` exactly `+1.0f` or `-1.0f` (the base design already asserts this for `IMU_SIGN_*`).
- **Handedness / distinctness**, the one that earns its keep:

```c
// +1 for an even permutation, -1 for odd, 0 if any two _SRC collide.
#define IMU_AXIS_PARITY                            \
  ((((IMU_AXIS_Y_SRC) - (IMU_AXIS_X_SRC)) *        \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_X_SRC)) *        \
    ((IMU_AXIS_Z_SRC) - (IMU_AXIS_Y_SRC))) / 2)

static_assert(IMU_AXIS_PARITY * IMU_AXIS_X_SIGN * IMU_AXIS_Y_SIGN *
              IMU_AXIS_Z_SIGN == 1.0f,
              "ERROR: axis map is a mirror, not a rotation. Check that the "
              "permutation's parity matches the number of sign flips.");
```

One expression covers duplicate source indices (parity → 0) and mirrored maps (determinant → −1). Worth having because the failure is silent and slow to find: a mirrored map still produces entirely plausible-looking data, and whichever axes are mis-signed are wrong for accel and gyro alike. The shipped `SIGN_X`-only config sat undetected through a drive test and an app-side investigation before the arithmetic caught it — with 24 orientations instead of 8, hand-derivation gets *more* error-prone, so the guard matters more here, not less.

### Order reference

Deriving a map should be a lookup, not a re-derivation. Order names read as "which sensor axis feeds vehicle X, Y, Z."

| Order | `_SRC` triple | Parity | Required sign flips |
|---|---|---|---|
| `XYZ` | 0, 1, 2 | even | even (0 or 2) |
| `XZY` | 0, 2, 1 | odd | odd (1 or 3) |
| `YXZ` | 1, 0, 2 | odd | odd (1 or 3) |
| `YZX` | 1, 2, 0 | even | even (0 or 2) |
| `ZXY` | 2, 0, 1 | even | even (0 or 2) |
| `ZYX` | 2, 1, 0 | odd | odd (1 or 3) |

Six orders × four valid sign combinations each = the 24 real orientations.

### Deriving the map on the bench

No drive test required — static poses against the 1 Hz serial `milliG` output are sufficient (and preferable: the Monitor app is a display layer that has been wrong here before, so it can't validate firmware signs). Hold the assembled unit in its installed orientation and:

1. **At rest** — whichever axis reads ≈ ±1000 is vehicle-vertical; sign gives up vs. down.
2. **Raise the forward end** — the axis that goes positive is vehicle X (forward+).
3. **Raise the left side** — the axis that goes positive is vehicle Y (left+).

Fill in the six macros; the determinant assert catches an inconsistent result at build time.

### Migration

`IMU_SWAP_XY` is retired — the new form subsumes it. Old configs map mechanically:

| Old | New |
|---|---|
| `SWAP_XY false`, signs (sx, sy, sz) | `X_SRC 0`, `Y_SRC 1`, `Z_SRC 2` with the same signs |
| `SWAP_XY true`, signs (sx, sy, sz) | `X_SRC 1`, `Y_SRC 0`, `Z_SRC 2` with the same signs |

The current shipped config (`SWAP_XY false`, `−1, −1, +1`) becomes order `XYZ` with signs `−1, −1, +1` — even permutation, two sign flips, determinant +1. Valid.

### Interactions and scope

- **Per-chip offsets are unaffected in principle.** `IMU_ACCEL_OFFSET_*` / `IMU_GYRO_OFFSET_*` are subtracted in the raw sensor frame *before* the remap, so they stay intrinsic to the chip and don't move when the mounting changes. That ordering is already correct in `g_imu.cpp` and must be preserved.
- **…but one hand-trim in the current config is orientation-dependent.** `config.h` documents a +0.0075g trim added to `IMU_ACCEL_OFFSET_Z_G` because the at-rest reading sat at 0.992–0.993g. That was tuned with sensor Z carrying 1g, so it absorbs scale-factor error, which only appears under load. Edge-mounted, sensor Z carries ≈0g and that trim becomes a small bias in the wrong place, while the axis now carrying gravity has no equivalent. Re-run `imu_calibration` (it wants +Z-up on a level bench — calibrate first, then mount) and re-derive the trim for the new orientation. The gyro trim is genuinely orientation-independent and carries over.
- **The on-chip wake-up detector is unaffected.** `remapAxes()` only reorients data read out over I2C; the LSM6DS3TR-C's embedded wake function runs on raw hardware axes. Our shipped wake is omni-directional shake, which doesn't care. This only becomes relevant if a *directional* tap-wake is ever added — see the base DESIGN.md's note on AN4650 §3.8.
- **Out of scope: non-orthogonal mounting.** The model still assumes the board sits square to the vehicle axes. A skewed mount needs a general rotation matrix, which is a different (and much less checkable) design. Keep mounting the board square.

### Where this should live

The scheme is platform-neutral and equally applicable to the base nRF52840 tree (and, with a different board frame, the ESP32 tree). It lands here first because this build needs it. Note that `g_imu.cpp` and `config.h` are **not** in `tools/check_common.sh`'s byte-identical set, so the trees may diverge on this without tripping the check — a deliberate divergence to record if the base tree isn't migrated at the same time.

## 7. Open items

- [x] **RGB LED decided (2026-08-04): dark by default, automatic fallback when no display is detected.** The enclosure layout for this variant puts the LED where it cannot conveniently be seen, and the OLED carries strictly richer status — so a visible-indicator role for it does not survive. It is *not* removed, though: `displayBegin()` disables the display module if no panel answers, and with the LED also off a loose I2C wire would leave the device with **no user-visible feedback at all**, looking dead while streaming perfectly. BATTERY_WAIT is the sharp case — "you left the switch off" is exactly when a signal matters and serial is least likely to be attached. So `ledUpdate()` gates on `displayIsPresent()` at runtime (not a compile-time `#if`, since panel presence is only knowable at boot), and `LED_ENABLED = 1` in `config.h` forces it always-on for bench work with the board out of the case. Note the XIAO's charge LED is wired to the BQ25101 and cannot be driven by firmware, so the device is never fully dark while charging regardless.
- [x] **Bench bring-up on both modules (2026-08-04)** via [`oled_probe`](../tools/nRF52840-OLED/oled_probe/oled_probe.ino): both the white and blue units enumerate at **I2C address 0x3C** (not 0x3D — no per-board variation), come up on the XIAO's 3V3 rail over D4/D5, and render correctly. Geometry confirmed via the frame / corner-tick / crosshair pattern, so the panel is genuinely 128×64 and the `SSD1306_128X64_NONAME` controller variant is right. Fonts and the Open Iconic Bluetooth glyph both render. **White chosen** over blue — also the better bet for the outstanding sunlight-readability question, being the brighter panel. Note the bus was not separately characterized (pull-up values unmeasured); it simply works, which is sufficient unless more I2C devices are ever added.
- [ ] **Verify the M100 Mini sustains 25Hz.** It is rated 10Hz — the same nominal rating the M100-5883 carries while running at 25Hz in the other variants, and it is the same u-blox M10 silicon, so this should carry over. But everything about this build's timing work assumes 25Hz, so confirm it early rather than late. GPS-only is the configuration that holds 25Hz on the -5883; multi-constellation sags.
- [x] **PCM confirmed on the 1000mAh pack (2026-08-05).** The vendor lists a built-in protection board covering overcharge, over-discharge, overcurrent, overheating and short circuit — at least equivalent to the 900mAh cell it replaces, which listed the same set minus thermal. This matters more than a spec line: the base design leans on the cell's own protection as the **hardware** over-discharge backstop for the two states firmware cannot monitor — DEEP_SLEEP and switch-OFF — where `LOW_BATT_CUTOFF_V` is not being enforced because nothing is running to enforce it. Firmware's 3.5V cutoff protects cycle life during normal use; the PCM is what protects the cell when the MCU is halted or unpowered. Worth eyeballing the pack for the protection board when it arrives, since listing copy is not a datasheet.
- [ ] **Case fit — the whole assembly, against a 45×75×20mm enclosure the base design already called snug.** Four things changed since that judgement was made, in both directions: the **display** and its mounting are new additions; the **M100 Mini** is smaller than the M100-5883 it replaces (which is why it was chosen); the **1000mAh cell** sits at the top of the ~1000mAh the case was judged to accept; and the **XIAO moves to the end wall**, since the display now takes the lid position it used to occupy. Settle this before committing to a mounting layout — the axis map depends on the final XIAO orientation, and re-deriving it is wasted work if the mechanical arrangement moves afterwards.
- [x] **Display library chosen: u8g2 (2026-08-04)** via [`oled_bench`](../tools/nRF52840-OLED/oled_bench/oled_bench.ino). `updateDisplayArea()` validated visually over 400 frames (no tearing or corruption outside the region) and measured affordable, while a full frame costs 31ms against the ~5.5ms GNSS RX budget — which rules out `Adafruit_SSD1306`, whose only path to the panel is `display()`. Numbers, cost model, and the three design consequences in [§5](#measured-update-cost).
- [ ] Bench-measure `DISPLAYOFF` sleep current, plus the all-pixels-on and controller-active baselines (§3). `oled_probe` modes 4/5/6 hold each state for a meter. **Deliberately deferred 2026-08-04** — not blocking any current work; needed only when the load-switch question in §3 is reopened.
- [x] **Bright-light readability judged sufficient (2026-08-04)** — `oled_probe` mode `r` (large, sparse, max contrast) held next to a window in bright direct sun: **washed out but still legible**, and accepted as good enough for this use case. This retires the original risk that made OLED a questionable pick here. Caveat on the test conditions for the record: this was indoors through glass, not the harsher case of the unit sitting on a dashboard in unfiltered sun. If that turns out worse than acceptable in practice, the fallback is the Sharp Memory LCD noted in [§2](#display-alternatives-considered-and-not-chosen-for-v1) — a real but non-trivial change (SPI instead of I2C, plus VCOM drive).
- [x] **On-screen layout designed and validated on hardware (2026-08-04)** — status bar over per-state bodies, five screens, 126×62 layout area with a positive-only pixel-shift gutter. Worst-case field widths and the full shift range both checked clean on the panel. Spec in [§5](#screen-layout); reference implementation in [`oled_layout`](../tools/nRF52840-OLED/oled_layout/oled_layout.ino).
- [x] **`g_display` implemented (2026-08-04)** — all five screens, the positive-only pixel shift, and the render-then-sliced-push state machine (§5). Wired into `setup()`/`loop()` and into both of `g_state`'s DEEP_SLEEP paths, since the panel's 3V3 rail survives System OFF and would otherwise sit lit on a stale frame. A missing panel disables the module rather than halting — unlike `g_imu`, a dead screen does not stop the device doing its actual job. **Bench-validated 2026-08-05: 25 Hz holds steady with the display live**, after spacing the slice pushes 20 ms apart. Smaller slices alone were not enough — see [Field result](#field-result-the-constraint-was-density-not-volume). Status-bar revisions from that session (no percentage, USB icon in place of the charging bolt) are in [§5](#screen-layout).
- [ ] **Consider hysteresis on `g_battery`'s `full` flag — latent, not observed.** It is a bare `voltage >= BATTERY_FULL_V` comparison, so in principle a cell sitting near the threshold could oscillate the flag, which drives CHARGE_ONLY's `Full`/`Charging` label here and the steady-green LED in the non-display variants. **This was originally logged as the cause of a bench flicker; that attribution was wrong** — the flicker came from a floating switch-sense pin (see §5). Nothing has actually been seen to chatter on `full`, so this is a code-reading concern rather than a reproduced bug. Confirm it happens before adding machinery to a shared, safety-adjacent module.
- [x] **Displayed fields mapped to data sources (2026-08-04)** — see [§5](#data-sources-for-the-displayed-fields). Everything the layout shows already exists except two items, both traps rather than work: the PVT rate is currently computed only inside `#if LOG_ENABLED` and so would read zero in a shipping silent build, and `gnssConsumePvt()` is consume-once, so `g_display` needs a non-consuming `gnssLatestPvt()` rather than racing `g_telemetry` for each epoch.
- [x] **Both plumbing changes landed across all three trees (2026-08-04), compiles clean on each.** `gnssLatestPvt()` added to `g_gnss` (applied by hand to ESP32, whose `g_gnss` is platform-specific, with comment text lifted verbatim so the trees read identically), and PVT/BLE rate accounting hoisted into an unconditional `updateRates()` in `g_telemetry` behind getters. `g_telemetry.h`/`.cpp` are in `tools/check_common.sh`'s byte-identical set, so those copies are identical by construction and the check passes; `g_gnss.*` is not in that set, so its parity is deliberate rather than enforced — keep it in mind when either tree's GNSS interface changes again.
- [ ] **Confirm the double-nut standoff clearance survives final assembly.** Fixed the cracked-display incident (2026-08-06, see [§4](#cracked-display-during-mounting-2026-08-06)) — the gap between the PCB and the lid is deliberately small, so re-check it isn't closed up by harness routing, any gasket/foam, or the lid not seating exactly as it did on the bench.
- [ ] **`g_display`'s I2C probe has no timeout — deferred, not forgotten.** Investigated 2026-08-06 after the cracked-display incident hung the whole boot. Confirmed by reading the installed core's `Wire_nRF52.cpp`: `TwoWire::endTransmission()` on this platform is a genuine unbounded register spin (`while(!EVENTS_TXSTARTED && !EVENTS_ERROR);` and similar, two of which don't even check `EVENTS_ERROR`), with no `Wire.setTimeout()` escape available. A timeout *wrapped around* the call cannot work — the call never yields, so surrounding code never gets a chance to check a clock. Two real fixes, neither implemented yet:
  - **Scoped one-shot timer + `NVIC_SystemReset()`**, gated by an `NRF_POWER->GPREGRET` sentinel so a genuine timeout skips the display on the next boot instead of looping forever. Every mechanism here (software reset, GPREGRET retention across a soft reset) is standard, documented Cortex-M/nRF52 behavior — more code than a one-function patch, but nothing in it is speculative.
  - **Hand-rolled bounded poll directly against `NRF_TWIM0`** (confirmed the peripheral `Wire` binds to), reproducing the vendor's TASK/EVENT sequence with `millis()` bounds instead of unconditional spins. Stays local to `g_display.cpp`, but relies on reverse-engineered register timing that can't be verified without hardware, and only covers the initial probe — not any hang inside `oled.begin()`/`sendBuffer()` later, which go through the same unbounded path.

  Left alone for now: today's fault was mechanical (cracked glass) and is fixed by the mounting change above, so the practical odds of hitting this exact hang again are lower than they were. Revisit if it recurs, or before this device goes somewhere unattended for long stretches.
- [x] **D4/D5 conflict found and resolved in config (2026-08-04) — A4 *is* SDA on this board.** The base design's switch-sense divider sits on A4, which `variant.h` defines as the same pin as `PIN_WIRE_SDA`. `POWER_SWITCH_SENSE_PIN` moved to **A1** in this tree only; see [§4](#the-switch-sense-pin-has-to-move-a4-is-sda). **Bench-confirmed 2026-08-05**: a purpose-built 510kΩ/510kΩ divider soldered up for breadboard use reads correctly on A1, and the display's I2C bus is healthy with the divider no longer sharing SDA. Before this, A1 was left floating on the bench and its drifting reads chattered `powerSwitchOn()` — which produced a flickering status icon that was initially and wrongly attributed to `bat.full` hysteresis (see [§5](#screen-layout)). A floating high-impedance input sampled with a 40µs acquisition window is the thing to suspect first whenever `switchOn` misbehaves.
- [ ] **Implement the generalized axis remap** (§6): `_SRC`/`_SIGN` macros, the determinant `static_assert`, retire `IMU_SWAP_XY`, update `remapAxes()`.
- [ ] **Derive this build's axis map** once the end-wall mount is final, via the static-pose procedure in §6.
- [ ] **Re-run `imu_calibration` and re-derive the accel Z trim** for the new mounting (§6 — the existing +0.0075g trim is orientation-dependent).
- [ ] Copy `imu_calibration` / `imu_tiltmap` into this tree's `tools/`, or decide the IMU tools stay single-sourced in the base tree and say so in the README. (`tools/` is no longer empty — it now holds [`oled_probe`](../tools/nRF52840-OLED/oled_probe/oled_probe.ino), which is display-specific and belongs only to this tree.)
- [ ] Decide whether to migrate the **base nRF52840 tree** to the same remap scheme, or accept the divergence (§6, *Where this should live*).
- [x] **Recorded the RaceBox protocol-figure discrepancy and the corrected axis map in the base tree's DESIGN.md (2026-08-04)** — its §4 now carries the as-built orientation, the bench-verified signs, and the output-frame evidence, and the superseded drive-test open item is marked as such.
- [ ] Check the **ESP32 tree's** axis signs independently — it still carries `+1/+1/+1`, correct only if that build mounts USB-rearward; different enclosure and an external MPU-6050, so its board frame is not the XIAO's.

---

This document follows the same open-items/checklist convention as [`nRF52840/DESIGN.md`](../Gnimu-nRF52840/DESIGN.md) — check items off as they're bench-validated.

[amazon-conn]: https://www.amazon.com/dp/B0DNTK1S9L
[amazon-gnss]: https://www.amazon.com/dp/B0BX65QZJ8
[amazon-lipo]: https://www.amazon.com/dp/B0DPZVBKMY
[amazon-oled]: https://www.amazon.com/dp/B0D91NB1CP
