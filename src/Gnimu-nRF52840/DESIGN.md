# Gnimu Pro — Design Notes

Design reference for **Gnimu Pro**, an evolution of the ESP32-based Gnimu RaceBox Mini emulator, re-targeted to a **battery-powered Seeed Studio XIAO nRF52840 Sense**.

---

## 1. Goals

- Port the existing modular ESP32 firmware to the **nRF52840**, reusing as much code as possible.
- Run on a **3.7 V LiPo** instead of assuming an always-present 5 V USB supply — this is the main new engineering.
- Keep the RaceBox Mini BLE protocol behavior identical (app compatibility).
- Use the XIAO Sense's **onboard 6-axis IMU** (no external IMU).

## 2. Hardware selection

| Item | Choice | Rationale |
|---|---|---|
| SoC module | **Seeed XIAO nRF52840 Sense** | Nearly unique combination of tiny footprint (21×18 mm) + onboard LiPo charging + onboard 6-axis IMU (LSM6DS3TR-C) + BLE, with the mature Nordic/Adafruit **Bluefruit** BLE ecosystem. |
| GNSS | **HGLRC M100 (M100-5883, u-blox M10)** | Reused from v1; good real-world performance. Must stay **u-blox/UBX** — the whole `g_gnss.cpp` (SparkFun u-blox lib, NAV-PVT) only works with u-blox silicon. |
| Regulator | **TPS63020 buck-boost @ 3.3 V** | Powers the GNSS. See [Power architecture](#3-power-architecture--wiring). |
| Battery | **Flat pouch LiPo, 900 mAh, built-in PCM** (overcharge/over-discharge/overcurrent/short-circuit protection) | Fits the enclosure comfortably; see [Battery sizing](#7-battery-sizing). PCM is the hardware over-discharge backstop below firmware's reach — see [Standby power draw](#standby-power-draw-and-the-ultimate-over-discharge-floor). |
| Master switch | **Slide switch (maintained), SPST, inline on battery +** | Easy to mount; full battery disconnect for long idle periods. |

### SoC alternatives considered (and rejected)

- **XIAO MG24 Sense** — cheaper, newer (Silicon Labs EFR32MG24, Matter/Thread, AI accelerator), but a *different BLE ecosystem*. Would force redoing the BLE layer on a less-mature stack, plus rework of sleep/bootloader/battery-sense. Its extra features are unused here.
- **Adafruit Feather nRF52840 Sense** — the only board matching all three killer features; same chip, same Bluefruit stack, same LSM6DS3TR-C IMU (code transfers 1:1). Rejected only on size (~51×23 mm vs 21×18 mm). Good bench-prototyping companion.
- Others (SparkFun Pro nRF52840 Mini, Arduino Nano 33 BLE Sense, ESP32 IMU boards) each drop at least one of: onboard IMU, LiPo charging, or the right ecosystem.

### GNSS alternatives (future upgrade paths only)

- **MAX-M10S** — same M10 capability, lower power / better antenna.
- **NEO-F10N / DAN-F10N** — dual-band L1/L5, better multipath rejection at tracks, but *unverified it cab sustain 25 Hz* (F10 is asset-tracking oriented).
- **ZED-F9P** — overkill (power/cost).
- The **M100-Pro** is a *lateral move* (also M10, rated 10 Hz, unused compass) — not an upgrade.

## 3. Power architecture & wiring

**Topology:** XIAO runs directly off the LiPo on its BAT pads (onboard LDO + charging intact). The buck-boost exists only to supply the GNSS a stable **3.3 V** rail. v1 already ran the M100 at 3.3 V, so 3.3 V is confirmed fine — no 5 V needed. Benefits of 3.3 V buck-boost: eliminates the ~50 mW the M100's internal LDO wasted at 5 V; holds a stable rail across the full LiPo range (4.2→3.0 V) so no GNSS brownout near end of charge; GNSS UART and XIAO are both 3.3 V logic, so **no level shifting**.

### Connections

```
             (+) ──[ slide switch ]──┬─────────────► XIAO  BAT+ pad
  LiPo 3.7V                          ├─────────────► TPS63020  VIN
             (–) ───────────────── star GND ─────── XIAO GND / TPS63020 GND / GNSS GND

  TPS63020  VOUT (3.3 V) ───────────────────────────► GNSS  VCC

  XIAO D6 (Serial1 TX) ─────────────────────────────► GNSS  RX
  XIAO D7 (Serial1 RX) ◄───────────────────────────── GNSS  TX
```

- **VIN header holes double as the battery+ junction:** land LiPo+ (post-switch) in one VIN hole and XIAO BAT+ in the other — both are the same node internally (raw battery voltage, which is what BAT+ wants). No splicing.
- **GND header holes = a ground bus:** land battery−, XIAO GND, and GNSS GND each in its own GND hole (all common internally). Treat as a star node, not a daisy chain.
- GNSS PPS / SDA / SCL (the M100-5883 compass pins) are **unconnected** — firmware doesn't use them.

### Regulator configuration (TPS63020 board)

- **Output:** set to 3.3 V via the board's output-select pad. **Measure the output (unloaded, loaded, and while sweeping Vin 4.2→3.3 V) before connecting the GNSS** — pad silkscreen isn't always trustworthy.
- **EN pad → XIAO GPIO:** used as the GNSS power gate (see [cutoff](#low-voltage-cutoff)). Firmware drives EN **low to disable** the GNSS rail, releases hi-Z / high to enable. Verify the board's onboard pullup; **do not add a discrete pulldown** (it would fight the pullup). Do **not** back-feed this rail into the XIAO 3V3 pin.
- **PS pad:** open = power-save mode (best light-load efficiency, default); short = forced-PWM (fixed ~2.4 MHz, lower ripple). **Built shorted (forced-PWM)** for the cleaner rail — the light-load efficiency penalty is negligible since the GNSS is a steady ~30 mA load, and it settles the output to a tight 3.34 V.

### Considerations

- **TX↔RX crossover** is the classic mistake: GNSS TX→XIAO RX, GNSS RX→XIAO TX.
- **Switching noise vs GNSS:** keep the buck-boost + inductor away from the GNSS antenna. Layout matters more than the PS mode setting.
- **Switch must be ON to charge** (charger sits behind it on the BAT+ node).
- The firmware [low-voltage cutoff](#low-voltage-cutoff) is the primary defense, protecting cycle life by stopping well above the cell's own hardware floor during normal use. The chosen cell's built-in PCM is a secondary backstop for the two states firmware can't monitor (DEEP_SLEEP, switch-OFF) — see [Standby power draw](#standby-power-draw-and-the-ultimate-over-discharge-floor). Neither is a substitute for the other.

## 4. Firmware architecture

The module boundaries (platform-neutral `.h` interfaces) are what make the port feasible: keep the interfaces, rewrite the platform-bound `.cpp` internals.

**Carries over ~unchanged:** `g_ubx_helpers.*` (portable byte packing), `g_telemetry.*` (only the offset-67 battery byte source changes), all `.h` interfaces, the SparkFun u-blox GNSS library, `config.h` compile-time `static_assert`s.

| Module | Change |
|---|---|
| `g_ble.cpp` | **Full rewrite** ESP32 `BLEDevice/BLEServer/BLE2902` → nRF52 **Bluefruit**. Same 4 functions. Use `BLEDis`/`BLEBas` helpers; custom 128-bit Nordic UART UUIDs map directly. `BLE_TX_POWER_ADV`/`_CONN` macros: `ESP_PWR_LVL_*` enum → dBm int (`Bluefruit.setTxPower`), split into separate advertising vs connected levels (switched in the connect/disconnect callbacks). Raise the MTU ceiling with `configPrphBandwidth(BANDWIDTH_MAX)` before `begin()` and let the central negotiate (do **not** peripheral-initiate `requestMtuExchange` — it deadlocks iOS/macOS service discovery). `Advertising.restartOnDisconnect(true)` lets most of the re-advertise state machine be deleted. |
| `g_imu.cpp` | MPU6050 → onboard **LSM6DS3TR-C**. Swap `Adafruit_MPU6050` → Seeed LSM6DS3 lib. Enable the IMU's dedicated power pin; init on the internal I2C bus (`Wire1`), not external Grove/Qwiic. LSM6DS3 returns g and deg/s directly → conversion in `imuReadProtocolUnits()` simplifies. Replace `MPU6050_*` range/bandwidth enums. **Remap/sign-flip axes** for the mounting (see below). Keep EMA filter, gyro-bias cal, saturation. |
| `g_gnss.cpp` | UART only. nRF52 core has no 4-arg `begin(baud,8N1,RX,TX)`; use `Serial1` (D6/D7 on the XIAO). Constellation toggles carry over unchanged. **Target baud is 115200, not 460800** — 25 Hz PVT is only ~2500 bytes/sec so either fits easily, but at 460800 the Serial1 RX buffer (~64 bytes) fills in ~1.4 ms, and our worst-case loop latency (~5 ms, dominated by the once-per-second stats printf) drops bytes and craters the observed rate. 115200 gives a ~5.5 ms fill window with room to spare. Exposes `gnssEnd()` for the state machine to call before entering held-off states (see §5). |
| `Gnimu.ino` | Add `batteryBegin()`/`batteryPoll()`. **Never gate startup on `while(!Serial)`** — battery has no USB host. |
| `g_led.cpp` | The RGB status LED is its own module (observes battery + connection state via their public queries). XIAO RGB LED is **active-LOW** (invert polarity). Color/blink signals state: blue-blink = advertising, blue-steady = connected, green-blink = charging, green-steady = full, amber-blink = low-battery warn, red-blink = low-battery critical, short-on/long-off blue pulse = LIGHT_SLEEP. "Full" is voltage-approximated (no charge-complete pin on the XIAO); see `config.h` `BATTERY_FULL_V`. Drives every channel with plain `digitalWrite()` only — an `analogWrite()`-based PWM "breathing" effect was tried for LIGHT_SLEEP and reverted; see [LED signal for LIGHT_SLEEP](#led-signal-for-light_sleep-digital-only-no-pwm) below for why. The XIAO also has a tiny onboard **charge-status LED** (hardware-fixed: green while the BQ25101 charger is active). We deliberately don't rely on it — it's too small to be readable next to the user-facing RGB LED and firmware can't affect its behavior. Any charge-state feedback the user sees comes from the RGB LED's green channel via `g_led`. |

### IMU axis mapping

Mounting update (post-parts fit): the XIAO is mounted **flat and level under the lid, right-side-up** (see [Mechanical](#8-mechanical--enclosure)) — not vertical against the end wall as originally planned. Because the PCB is now horizontal, **Z (perpendicular to the PCB) is the vehicle-vertical axis**: at rest it reads ≈ +1 g and, since the board is right-side-up, **no Z remap is expected beyond confirming that sign**.

That leaves **X and Y in the horizontal plane = vehicle forward/lateral**. The XIAO Sense's in-plane axis directions/signs are **not officially documented** (confirmed via Seeed wiki + forums), so which of X/Y is forward vs. lateral, and their signs, had to be established empirically. In practice this reduced to an **X↔Y swap and/or sign-flip of X and Y** — the full 3-axis permutation the end-wall mount would have required is not needed now that Z falls out naturally. **Apply the identical remap to the gyro.** This simple swap/sign-flip is valid only because the board is mounted **square** (orthogonal) and level in a consistent, known orientation — keep it that way.

**Resolved 2026-08-04 (as-built):** the module sits with **USB-C pointing forward** — i.e. the XIAO's BLE-antenna end (sensor +X) faces **rearward**, and the GNSS is at the forward end of the case. That is a **180° yaw** from the reference orientation, which flips **both** X and Y: `IMU_SWAP_XY = false`, `IMU_SIGN_X = -1.0f`, `IMU_SIGN_Y = -1.0f`, `IMU_SIGN_Z = +1.0f`. Bench-verified by static tilt against the 1 Hz serial `milliG` output, all three axes: standing on the forward (GNSS) end reads X ≈ −1 g and on the rearward (XIAO) end X ≈ +1 g; left-side-down reads Y ≈ −1 g (a right turn) and right-side-down Y ≈ +1 g; flat and level reads Z ≈ +1 g.

This corrects a long-standing error. `IMU_SIGN_X` had been flipped to `-1.0f` at some point without a matching `IMU_SIGN_Y`, which made the remap matrix a **mirror** (determinant −1) rather than a rotation — an orientation no physical mounting can produce. The symptom was inverted lateral g with correct longitudinal g, easy to miss because braking/acceleration is the axis you notice first, and further masked by the Monitor app label bug noted in the open items. Two lessons worth keeping: a signed axis map must always have **determinant +1** (an even permutation needs an even number of sign flips, odd needs odd), and the **Monitor app cannot validate firmware signs** — it is a display layer that has been wrong here before, so verification goes against the raw serial `milliG` numbers.

### Output frame and the RaceBox protocol figure

The target output frame is **X forward+, Y left+, Z up+ — ISO 8855, right-handed**. Established from three independent sources: the upstream emulator's unremapped output on a USB-rearward flat mount; the RaceBox BLE Protocol Documentation's own worked example packet (Rev 8, p. 8), which decodes `GForceZ = +0.974 g` on a device sitting still at 0.126 kph; and the RaceBox Mini user manual's mounting rules (logo to the sky, charging port to the rear).

**The protocol document's axis figure (Rev 8, p. 7) contradicts all three and should be disregarded — its arrows are uniformly inverted.** It draws +X rearward, +Y right, and +Z down, i.e. the exact negation of the frame the hardware actually emits. The tell is the status LED in the drawing: on the real device that LED sits on the same face as the USB-C port, and the figure places it at the edge its +X arrow exits, so by the manual's own mounting rule that arrow points rearward. Negating all three axes also flips handedness, which is why the drawn frame reads as left-handed while every real implementation is right-handed — one systematic error rather than three independent ones. Recorded here because anyone reading the protocol doc cold will conclude the firmware is wrong. Full write-up, including the generalized remap design this fed into, is in [`nRF52840-OLED/DESIGN.md` §6](../Gnimu-nRF52840-OLED/DESIGN.md#6-imu-axis-remapping--generalized-to-all-24-orientations).

## 5. State machine

Gnimu Classic ran on always-present USB — "powered ⇒ everything on at full rate" was the only mode, so `setup()`/`loop()` encoded it implicitly. Pro adds a battery, a hardware switch, and a low-power runtime target, which multiplies the meaningful *operating modes* — not features. To make that behavior auditable rather than emergent, Pro replaces the ad-hoc reflexes with an explicit state machine owned by a top-level orchestrator (`g_state`). Every state change routes through one transition table, and each state's peripheral posture is a single entry action.

### States

| State | GNSS | IMU | BLE | LED | Live? |
|---|---|---|---|---|---|
| **RUNNING** | hot | on, polling | fast adv + conn | normal priority | yes |
| **CHARGE_ONLY** | off (EN low, TX low) | off | off (disconnected + not advertising) | green blink (same as charging) | yes (polls USB + switch) |
| **LIGHT_SLEEP** | backup mode (~µA, later EN-cut) | low-power ODR, wake-detect armed | unchanged (still advertising/connectable) | blue pulse (short on / long off) | yes |
| **BATTERY_WAIT** | off (EN low, TX low) | off | off | rapid red blink | yes (polls switch) |
| **DEEP_SLEEP** | off | off | off (SoftDevice halts) | off | no — System OFF |
| *POWER_OFF* | — | — | — | — | not modeled |

`RUNNING ↔ LIGHT_SLEEP` is the only hot pair — a reversible transition without reset. Everything else is a **one-way door**: BATTERY_WAIT, CHARGE_ONLY, and DEEP_SLEEP all exit via reset (`NVIC_SystemReset()` or a hardware power event), which sends the machine back through `setup()` and re-classifies from scratch. That simplification is deliberate — hot-resuming from DEEP_SLEEP is impossible anyway (System OFF is a chip-level halt), and hot-resuming from BATTERY_WAIT / CHARGE_ONLY would require modeling a mid-boot restart of every peripheral that was skipped. POWER_OFF (switch off + no USB) is not a software state at all: the rail vanishes and no code runs — it's shown here only for completeness.

### Inputs

Five inputs, all queried each tick:

| Input | Source | Meaning |
|---|---|---|
| `switchOn` | `powerSwitchOn()` (A4 divider) | Authoritative battery-in-circuit / switch position. Load- and SoC-independent. |
| `usbPresent` | `powerUsbPresent()` | VBUS present. |
| `cutoffRequested` | `batteryCutoffRequested()` | Fresh VBAT peak below `LOW_BATT_CUTOFF_V`, debounced ≥ `LOW_BATT_DEBOUNCE_MS`. Voltage-only. |
| `bleConnected` | `bleIsConnected()` | Client connected. |
| `idleElapsed` | timer | No BLE client for > `STATE_IDLE_TIMEOUT_MIN` minutes. |

The `switchOn` input replaces the old VBAT-based battery-presence heuristic entirely — see [Battery presence (switch-sense)](#battery-presence-switch-sense) in §6 for why VBAT-floor / sawtooth-variance detection can't survive the "charger-in-CV feeding a load" case.

### Boot classification

Every boot runs the same short prologue, in this order:

1. `powerHoldPeripheralsOff()` — first executable line in `setup()`. Drives GNSS `EN` low, idles GNSS TX (D6) low, drives IMU power low, sets the LED off. Runs *unconditionally*, before `Serial.begin()`. Kills the phantom paths that would otherwise light the GNSS during switch-off boots.
2. Bring up `g_battery` sensor and `g_power` sense pins. Prime the sampler with one blocking run (~50 ms) to establish an initial VBAT peak; read USB presence and switch position.
3. Classify:

    ```
    if (!switchOn)                                             -> BATTERY_WAIT
    else if (!usbPresent && bootVbatPeak < LOW_BATT_CUTOFF_V)  -> DEEP_SLEEP
    else if (CHARGE_ONLY_ON_USB && usbPresent)                 -> CHARGE_ONLY
    else                                                       -> RUNNING
    ```

4. Bring up GNSS, IMU, BLE **only** if entering RUNNING. In BATTERY_WAIT, CHARGE_ONLY, and DEEP_SLEEP those peripherals stay held-off from step 1.

The DEEP_SLEEP boot branch is an optimization — an already-depleted cell on no USB skips the cold-start of GNSS just to be cut ~5 s later. The CHARGE_ONLY boot branch is the same idea for a different reason: user plugged in intending to charge, so don't waste cold-start current on peripherals we're about to hold off.

### Transitions

Guards are evaluated in priority order per source state; first match wins.

**From RUNNING:**

| Priority | Guard | → | Action |
|---|---|---|---|
| 1 | `cutoffRequested && !usbPresent` | DEEP_SLEEP | `bleStop()` + `gnssEnd()` + `powerEnterDeepSleep()` (System OFF) |
| 2 | `!switchOn` | BATTERY_WAIT | `bleStop()` + `gnssEnd()` + `powerHoldPeripheralsOff()` |
| 3 | `CHARGE_ONLY_ON_USB && usbPresent` | CHARGE_ONLY | `bleStop()` + `gnssEnd()` + `powerHoldPeripheralsOff()` |
| 4 | `idleElapsed` | LIGHT_SLEEP | `gnssSleep()` (RXM-PMREQ backup mode) + `imuArmWake()`, BLE/advertising left unchanged, LED → blue pulse |
| — | else | RUNNING | — |

**From CHARGE_ONLY:**

| Priority | Guard | → | Action |
|---|---|---|---|
| 1 | `!usbPresent` | *(reset)* | `NVIC_SystemReset()` → boot → classification (lands in RUNNING) |
| 2 | `!switchOn` | *(reset)* | `NVIC_SystemReset()` → boot → classification (lands in BATTERY_WAIT) |
| — | else | CHARGE_ONLY | poll; cutoff is inapplicable (charger clamps VBAT) |

**From LIGHT_SLEEP:**

| Priority | Guard | → | Action |
|---|---|---|---|
| 1 | `cutoffRequested && !usbPresent` | DEEP_SLEEP | `powerEnterDeepSleep()` |
| 2 | `!switchOn` | BATTERY_WAIT | `enterBatteryWait()` (`powerHoldPeripheralsOff()`) |
| 3 | `bleConnected \|\| imuWakeTriggered()` | RUNNING | `exitLightSleep()`: `gnssWake()` (or cold `gnssBegin()` if already EN-cut) + `imuDisarmWake()`, idle timer reset |
| 4 | `gnssInBackup && (now - lightSleepEnteredMs) >= STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN` | *(stays LIGHT_SLEEP)* | GNSS backup mode → full EN-cut (ephemeris presumed stale by this point; one-shot, guarded by `gnssInBackup`) |
| 5 | `(now - lightSleepEnteredMs) >= STATE_LIGHT_SLEEP_TIMEOUT_MIN` | DEEP_SLEEP | `powerEnterDeepSleep()` — ultimate backstop for a forgotten device; BLE/shake wake no longer work past this point |
| — | else | LIGHT_SLEEP | — |

Priorities 4 and 5 are two independent timers, both measured from LIGHT_SLEEP entry (`lightSleepEnteredMs`), not from each other — priority 4 only fires once (`gnssInBackup` flips false after) and only escalates GNSS; IMU wake-detect and BLE stay exactly as they were the whole time up to priority 5's full DEEP_SLEEP backstop.

**Wake sources:** LIGHT_SLEEP → RUNNING is triggered by BLE connect **or** an IMU wake event (omni-directional shake, not directional tap — see below). Because LIGHT_SLEEP is not System OFF, `loop()` keeps running, so this doesn't need a true hardware interrupt wake — the LSM6DS3TR-C's embedded wake-up detector runs on-chip in low-power accel mode and asserts `IMU_INT1_PIN`, which `g_state` polls each loop via `imuWakeTriggered()`. The IMU is **not** fully powered down in LIGHT_SLEEP (that would kill wake capability) — `imuArmWake()` drops it to a low-power ODR (`IMU_WAKE_CTRL1_XL`, default 12.5 Hz) with the wake interrupt armed instead.

Three hardware/ordering lessons from bringing this up, all in `g_imu.cpp`:
- **`FS_XL` must match the RUNNING-mode accel range, not be dropped further.** Since arming is a *live* mode switch on an already-running chip (not a cold power-on), changing the accelerometer's full-scale range creates a persistent scale discontinuity in the wake-up detector's raw-code threshold comparison that reliably false-triggers immediately — a settle delay does not fix it. Only ODR changes across the RUNNING↔LIGHT_SLEEP transition (`IMU_WAKE_CTRL1_XL` keeps the same FS bits as `IMU_ACCEL_RANGE_G`, only the ODR bits differ).
- **`imuDisarmWake()` must fully disable the detector (`TAP_CFG1 = 0x00`), not just un-route it from INT1 (`MD1_CFG = 0x00`) alone.** Un-routing alone leaves the detector's `INTERRUPTS_ENABLE` bit live, so it keeps running — and latching, since `LIR=1` — for the entire RUNNING period between LIGHT_SLEEP cycles. Real motion during that window (device handled, table bumped) then shows up as an instant false wake on the *next* `imuArmWake()`, and the effect visibly worsens cycle over cycle (the very first arm of a boot is the only one starting from a true power-on-reset `TAP_CFG1=0x00`).
- **`imuArmWake()` must flush `WAKE_UP_SRC` *before* routing to INT1 (`MD1_CFG`), not after.** Routing first and clearing the stale/settling latch 250 ms later leaves a window where anything already latched (or generated by the ODR switch itself) is visible on the pin before it's flushed — this showed up as a second-LIGHT_SLEEP-cycle-only spurious wake, since only the second+ cycle has anything to flush. Order is: reconfigure registers → settle delay → flush `WAKE_UP_SRC` (single read; it's a latch, not a FIFO) → **then** route `MD1_CFG`.

Reference for the wake-up detector's register model: ST **AN4650** (*LSM6DS3 always-on 3D accelerometer and 3D gyroscope*); bench-tuned via [`imu_wake`](../tools/nRF52840/imu_wake/imu_wake.ino).

### LED signal for LIGHT_SLEEP: digital-only, no PWM

LIGHT_SLEEP's LED was originally planned (and briefly implemented) as a non-linear "Apple-style" PWM breathing effect — see git history / the original design rationale below for the model. It was **reverted to a plain short-on/long-off blue blink** (`LED_LIGHT_SLEEP_BLINK_ON_MS` / `_OFF_MS` in `config.h`, both digitalWrite-driven, no PWM at all) after repeated bugs traced to one root cause on this Arduino core: once a pin is handed to a hardware PWM peripheral via `analogWrite()`, plain `digitalWrite()`/`pinMode()` calls never hand it back (confirmed in the Seeeduino core's `wiring_digital.c`) — so any state after the LED had breathed even once would leave the pin stuck at its last PWM duty cycle instead of updating. This surfaced three times before the pattern was recognized: the LED failing to resume blinking after an IMU wake, after a BLE disconnect, and — worst — staying lit indefinitely after DEEP_SLEEP's halt, since `loop()` never runs again afterward to self-correct. Switching `g_led.cpp` fully to `analogWrite()` (so no control-mode switch ever happens) didn't resolve the DEEP_SLEEP case either — by that point the actual value being written stopped mattering, which is consistent with nRF52's System OFF retaining GPIO output through a low-power latch rather than the pin's normal driver, weak enough that a sensitive LED could still glow very faintly regardless of what was driven right before the halt. Dropping PWM for LIGHT_SLEEP entirely — a slow blink needs none — removed the whole bug class at the root instead of working around the retention behavior.

Original breathing model, kept for reference (Recktenwald, *EAS 199: Arduino Programs for a Breathing LED*): each cycle has three phases — **inhale** (rising), **pause** (hold at `Vmax`), **exhale** (falling, deliberately longer than inhale) — each phase an exponential `v = a·e^(b·t)` fit between that phase's start/end brightness and duration. The `led_breathe` validation sketch that proved this model out has since been removed from `tools/` along with the feature; recover it from git history if this is ever revisited on hardware/toolchain that handles the PWM-ownership handoff better.

**From BATTERY_WAIT:**

| Priority | Guard | → | Action |
|---|---|---|---|
| 1 | `switchOn` | *(reset)* | `NVIC_SystemReset()` → boot → classification |
| — | else | BATTERY_WAIT | poll; USB removal simply kills the rail |

**DEEP_SLEEP** is terminal in software. Exit only via reset triggered by a USB plug-in or a switch off→on cycle (both hardware power events that reset the MCU).

Every live state carries the same safety edges: `cutoffRequested → DEEP_SLEEP` and `!switchOn → BATTERY_WAIT` in RUNNING (and LIGHT_SLEEP); `!switchOn → *(reset)* → BATTERY_WAIT` in CHARGE_ONLY. So a switch flip or a critical dip always resolves to the correct next state regardless of where you were. (Cutoff isn't checked in CHARGE_ONLY because USB is definitionally present there — cutoff needs `!usbPresent` — and losing USB immediately resets us into RUNNING, where cutoff resumes its normal watch.)

### Runtime shutdown ordering: release UART before holding pins

Any runtime transition out of RUNNING that ends in `powerHoldPeripheralsOff()` must first call `gnssEnd()` — that is, `enterBatteryWait()` and the RUNNING → DEEP_SLEEP branch, both in `g_state.cpp`. The reason is subtle and load-bearing: while `Serial1` is initialized, the UART peripheral **owns** D6, and any `pinMode(D6, OUTPUT); digitalWrite(D6, LOW)` from GPIO code is silently ignored. D6 keeps idling HIGH (UART TX idle level), which back-feeds through the GNSS RX ESD diode into GNSS VCC even with the TPS rail cut — phantom-powering the receiver. `gnssEnd()` releases Serial1 so D6 reverts to plain GPIO; only then does `powerHoldPeripheralsOff()`'s TX-low drive take effect. Boot-classified BATTERY_WAIT/DEEP_SLEEP entries don't need this — Serial1 was never started on those paths.

### CHARGE_ONLY

Addresses a real usability issue: **switch ON + USB in** would otherwise leave the device in RUNNING (streaming), which parks the GNSS on the shared BAT+ node continuously. The XIAO's charge IC sees that ~30 mA GNSS load in parallel with the cell and will likely never sense "battery full" — the user's "plug in overnight to top up" case gets slow-charged (or never latches full) instead.

The remedy is `CHARGE_ONLY` — cell in circuit (chargeable), peripherals held off (GNSS EN low + TX low, IMU power low, LED continues to reflect state), no streaming. Trigger is **automatic on USB plug-in when the switch is ON**, gated by the `CHARGE_ONLY_ON_USB` compile-time flag in `config.h` (default 1). No gesture is needed — the act of plugging in already expresses the user's intent to charge. Exit is unplug USB (device resets → boot classifier lands in RUNNING) or switch off (resets → BATTERY_WAIT). Both exit paths take the reset route for the same reason BATTERY_WAIT does: no mid-boot peripheral restart to model.

**Bench-development override:** set `CHARGE_ONLY_ON_USB` to `0` in `config.h`. USB presence is then ignored and the device stays in RUNNING while plugged in — streaming and BLE stay live for developer inspection. Ship default is `1`.

**Transitions (added to the priority tables below):**
- **RUNNING** priority 3: `CHARGE_ONLY_ON_USB && usbPresent` → CHARGE_ONLY (entry = `gnssEnd()` + `powerHoldPeripheralsOff()`, same UART-ownership rule as BATTERY_WAIT — release Serial1 before driving D6 low).
- **CHARGE_ONLY**: `!usbPresent` → reset; `!switchOnDebounced` → reset. Voltage cutoff is inapplicable here (charger clamps VBAT) so no explicit guard; if USB later drops out, we reset to RUNNING and the normal cutoff path in RUNNING catches it.
- **Boot classifier** gains a new step just above RUNNING: after `!switchOn` (BATTERY_WAIT) and `!usb && peak < cutoff` (DEEP_SLEEP), `CHARGE_ONLY_ON_USB && usb` → CHARGE_ONLY. Handles both "boot with USB already in" and "unplug + reset" landing paths uniformly.

**LED:** currently shows the same green-blink charging pattern as RUNNING-while-charging — deliberate, g_led just reads `bat.charging` and doesn't need to distinguish the two states yet. Possible future enhancement: a distinct green pulse pattern for CHARGE_ONLY (mirroring LIGHT_SLEEP's distinct blue pulse), so a glance tells you *which* charging mode the device is in — same short-on/long-off digital approach as LIGHT_SLEEP, not PWM (see [LED signal for LIGHT_SLEEP](#led-signal-for-light_sleep-digital-only-no-pwm)).

**BLE in quiet states:** every runtime transition out of RUNNING calls `bleStop()` (g_ble) alongside `gnssEnd()`. `bleStop()` disconnects any active peripheral client and halts advertising, so the device is invisible in BLE scans in CHARGE_ONLY / BATTERY_WAIT / DEEP_SLEEP. It does **not** tear down the Bluefruit stack — every quiet state exits via reset, so `bleBegin()` naturally runs again on the next boot. Boot-classified entries into a quiet state never called `bleBegin()` in the first place; a `bleInitialized` guard in `bleStop()` makes the call safe either way.

Part of Phase 1.5 — landed after the initial state-machine bring-up, before Phase 2.

## 6. Battery + power subsystem

The old single `g_battery` module has been split by concern:

- **`g_battery` (sensor / fuel gauge)** — owns the VBAT ADC + the non-blocking sampler; produces cell voltage, state of charge %, and the debounced `batteryCutoffRequested()` signal. Pure measurement.
- **`g_power` (rail mechanisms)** — owns USB presence, GNSS `EN` / TX drive, the switch-sense pin, `powerHoldPeripheralsOff()`, `powerEnterDeepSleep()`. Pure primitives, no policy.
- **`g_state`** (see [§5](#5-state-machine)) — the only module that turns these signals into actions.

`g_battery` reads USB via `powerUsbPresent()` for the displayed `charging` / `full` flags, so there is exactly one definition of "USB present" — but the *cutoff* path stays voltage-only in `g_battery`, and USB gating is applied by `g_state`.

### VBAT sampler

The XIAO exposes VBAT through a ~338 kΩ divider (1 MΩ / 510 kΩ). Two properties of the source drive the sampler design:

- **Long acquisition required.** The SAADC's default TACQ is 3 µs — valid only up to a ~40 kΩ source; on this divider each conversion undershoots by 100+ mV. `g_battery` calls `analogSampleTime(40)` at startup — 40 µs covers up to ~800 kΩ. Without this the VBAT reading is noisy and biased low, and any variance-based signal is swamped.
- **Non-blocking sampling.** The safety-relevant quantity is the *peak* voltage across a short window (not a single reading, not a mean — a real cell under load-transient dips is momentarily lower than its true SoC voltage). The sampler runs an `IDLE / SAMPLING` state machine advanced from `batteryPoll()`: every `BATTERY_POLL_INTERVAL_MS`, it collects `BATTERY_SAMPLE_COUNT` paced ADC reads (~`BATTERY_SAMPLE_SPACING_US` between reads → ~50 ms window at defaults), tracks `min`/`max`, terminates on count, then disables the divider. One `analogRead()` per `loop()`; never blocks longer than a single conversion.

`peak = maxAdc → mV` feeds both SoC and the cutoff. Peak was chosen (over mean) because load sag is a *downward* transient that lies about the resting voltage — peak rejects it. This also makes SoC more accurate than the old 8-sample averaging path: undershoot got averaged in; peak ignores it.

### Voltage → percent

The `BATTERY_DISCHARGE_CURVE` macro in `config.h` is a resting-voltage LiPo curve at 5 % steps, sorted high→low. `voltageToPercent()` clamps at both endpoints and linearly interpolates in between. Fed by the EMA-smoothed peak. Note the curve is nearly flat 80 %–20 %, so voltage is a weak SoC proxy in the middle regardless of resolution — that's a LiPo fact, not a sampling artifact.

### Charging + full

`charging = usbPresent && switchOn` (a cell can only charge when the switch closes the battery circuit). `full = charging && voltage ≥ BATTERY_FULL_V` — set below the 4.2 V CV target so the ADC/charger tolerances don't prevent it from ever latching. The protocol byte at telemetry offset 67 is `(charging << 7) | percent`, filled by `g_telemetry` from `batteryProtocolByte()`.

### Low-voltage cutoff

Safety requirement — protects the cell if the switch is accidentally left on with no USB. Now a three-part design:

- **`g_battery` computes the request.** `batteryCutoffRequested()` returns true when the fresh peak has been below `LOW_BATT_CUTOFF_V` for `LOW_BATT_DEBOUNCE_MS`. Voltage-only; no knowledge of USB. The debounce anchors on the first below-threshold reading and resets on any reading at or above, so a transient acquisition dip never trips it.
- **`g_state` decides.** In RUNNING (and LIGHT_SLEEP), the transition guard is `cutoffRequested && !usbPresent`. If USB is present, VBAT is being clamped by the charger — the cutoff is inhibited, letting the cell recover.
- **`g_power` acts.** `powerEnterDeepSleep()` cuts the GNSS rail (`EN` low + idle TX low), powers the IMU down, turns the LED off, then calls `sd_power_system_off()`. The MCU enters System OFF (single-digit µA). Latching is inherent — execution stops until a hardware reset. Recovery is any power event (USB plug-in or switch off→on), both confirmed on hardware to reset the MCU back into normal startup.

> **Hardware notes (from 2026-07 validation):** EN-low **truly disconnects** the TPS63020 output — no load switch needed — but the GNSS remains **phantom-powered through its RX pin** if XIAO TX (D6) idles HIGH. `powerHoldPeripheralsOff()` and `powerEnterDeepSleep()` therefore also `Serial1.end()` + drive D6 as GPIO LOW. Also, `GNSS_EN_PIN` must have `pinMode(..., OUTPUT)` called explicitly — an `INPUT`-mode pin lets the TPS63020's own EN pullup win, silently no-op'ing every `digitalWrite()`. Both actions live in `g_power` so they're impossible to forget. (Diagnostic: [`gnss_en`](../tools/nRF52840/gnss_en/gnss_en.ino).)

### Battery presence (switch-sense)

The **slide switch is 2-position, 3-pole**: one pole switches battery+, and one **spare pole** — normally unused — feeds a **510 kΩ / 510 kΩ divider to A4**. With the switch **off** the spare pole reads the ~4.2 V node → tap ~2 V. With the switch **on** the pole floats → the divider's lower resistor pulls the tap to ~0 V. `powerSwitchOn()` reads A4 as analog and compares against `SWITCH_OFF_THRESHOLD_MV` (800 mV — the huge gap makes the exact number irrelevant). Bench-confirmed 2026-07-14 on the breadboard: **0–9 mV ON / ~2083–2089 mV OFF**, clean transitions in both directions.

This gives a **load-independent, SoC-independent** presence signal — which was the linchpin. VBAT-based detection was investigated first (a DC-floor branch and a sawtooth-variance branch, characterized in [`battery_presence`](../tools/nRF52840/battery_presence/battery_presence.ino)), and *neither* branch could distinguish a charger-in-CV feeding a load from a battery-in-CV feeding a load: under GNSS load the charge IC holds the open BAT+ node at ~4.15 V CV, visually identical to a present cell. The switch-sense pin doesn't care what the node voltage is doing; it reports the switch position directly. So VBAT-based presence detection is retired; presence is a *power-module* concept, `powerSwitchOn()`.

Boot handling: switch off + USB in → the classifier lands in BATTERY_WAIT and holds every peripheral down (no phantom GNSS, no misleading "charging" LED — the rapid-red BATTERY_WAIT blink instead signals "flip the switch"). Runtime handling: a switch flip off transitions RUNNING/LIGHT_SLEEP → BATTERY_WAIT via `powerHoldPeripheralsOff()`, immediately, load-independent.

### Standby power draw and the ultimate over-discharge floor

Both non-RUNNING resting states draw well under 10 µA at the battery:

- **DEEP_SLEEP (switch ON):** `sd_power_system_off()` puts the nRF52840 itself in the single-digit-µA range; GNSS and IMU are held off by the same entry action, so their contribution is negligible. Firmware never wakes on its own from here — no voltage monitoring happens in this state.
- **BATTERY_WAIT / switch OFF:** the MCU is fully unpowered (the switch physically disconnects BAT+). The only standing load is the switch-sense divider itself, continuously biased by the cell through the switch's spare pole: ~4.1 µA at 510 kΩ/510 kΩ (vs. ~21 µA at the originally-considered 100 kΩ/100 kΩ — the 510 kΩ pair was chosen specifically to cut this idle draw ~5×; see `config.h`).

Both figures are well below typical LiPo shelf self-discharge, so neither materially shortens storage life on its own.

**Once the MCU is asleep or unpowered, our firmware's `LOW_BATT_CUTOFF_V` (3.50 V) no longer applies** — it only protects while RUNNING/LIGHT_SLEEP are actively polling VBAT. In DEEP_SLEEP or switch-OFF, the cell simply self-discharges (plus the µA-level draws above) until whatever floor the **cell's own protection circuit** enforces. The 900 mAh cell used in this build has a **PCM (protection circuit module) built into the pack** — overcharge, over-discharge, overcurrent, and short-circuit protection — which is the real backstop in this regime: it latches the cell off at its own UVLO threshold (typically lower than our 3.5 V firmware cutoff) and only releases once charging voltage is reapplied. Firmware's 3.5 V cutoff exists to protect *cycle life* by stopping well above that hardware floor during normal use; the PCM is the last line of defense for the two states where firmware isn't watching at all.

## 7. Battery sizing

Target: ~8 h continuous at max (25 Hz, BLE connected, no sleep).

- Estimated average draw **~35–45 mA** at the battery (GNSS ~30 mA @ 3.3 V ≈ 99 mW → ~29 mA @ 3.7 V via buck-boost; XIAO+IMU ~7 mA; misc ~1–2 mA).
- 8 h needs ~280–360 mAh delivered; usable LiPo ~80 % (stop ~3.4 V) → rated minimum ~350–450 mAh.
- **Chosen: 900 mAh flat pouch, built-in PCM** (overcharge/over-discharge/overcurrent/short-circuit protection) — comfortably covers the 8 h target with margin for aging, and the PCM gives a real hardware over-discharge floor for the two states firmware can't monitor (DEEP_SLEEP, switch-OFF) — see [Standby power draw](#standby-power-draw-and-the-ultimate-over-discharge-floor). Fits the enclosure with room to spare (up to ~1000 mAh).
- Bump the XIAO charge current to the ~100 mA pad for faster recharge (default ~50 mA).
- The slide switch additionally eliminates *storage* drain (off-state), separate from active runtime — see standby power draw above (~4.1 µA switch-sense divider draw vs. sub-10 µA DEEP_SLEEP).
- Validate real M100 draw at 3.3 V with an inline power meter once built — biggest unknown.

## 8. Mechanical / enclosure

Target enclosure internal dimensions: **45 W × 75 L × 20 D mm**.

- **XIAO mounting:** it has no mounting holes — it's a plug-in module (pre-soldered male headers point ~6 mm down). Plan: solder 2× 2.54 mm 7-pin **female header strips to a protoboard "shield"** (mirrors the v1 shield), wire GNSS/regulator/switch on the same board, and **mount that shield to the underside of the lid** (standoffs off the lid). XIAO stays removable for USB flashing.
- **Orientation (as built):** XIAO mounted to the **underside of the lid, flat and right-side-up**, header pins pointing **down** into the case, with its **USB-C end at the GNSS (forward) end** of the case. The XIAO's BLE antenna sits on the opposite short edge from USB-C, so it faces **rearward**. Two wins over the original end-wall plan: the board sits level, so **Z is the vertical axis and the IMU remap simplifies** (see [IMU axis mapping](#imu-axis-mapping)); and it puts the **BLE antenna** at the far end from the GNSS antenna — good RF separation. Still mount **square** so the remap stays a simple swap/sign-flip. *(Corrected 2026-08-04: this bullet previously claimed USB-C exits the end wall opposite the GNSS, and described a single "BLE/USB-antenna end" — but those are opposite edges of the XIAO. The RF-separation conclusion survives the correction; the reason is the BLE antenna's position, not USB-C's.)*
- **Case cutouts:** USB-C does **not** exit an end wall. A **right-angle USB-C adapter** on the XIAO turns the plug 90°, and its socket is exposed through the **left side wall** (case oriented GNSS-forward) — size the opening for the adapter housing, not just the port. With the board right-side-up, the RGB status LED, charge LED, and reset button face the **lid** — put viewing/access holes there, accounting for the shield standoff gap.
- **Reset:** the P0.18 "RESET" pad is the reset line, in parallel with the onboard button. Option to wire an external momentary button (pad → GND) instead of a poke-hole. Double-tap reset → UF2 bootloader (firmware-recovery path).
- **LiPo:** flat pouch on the **case floor** (with the shield now on the lid), not sandwiched under the shield's solder side — puncture risk. Secure with foam tape; leave room for slight swelling; keep it clear of the BLE/USB-antenna end and the GNSS antenna.
- **SWD:** keep the SWDIO/SWCLK pads accessible (don't bury) for hardware debugging / un-brick via an SWD probe, even if not wired permanently.

## 9. Protocol identity / compatibility

**Do NOT change these** — RaceBox app compatibility depends on them:

- `MODEL "RaceBox Mini"`, `FIRMWARE_VERSION "3.3"`, `HARDWARE_VERSION "1"`, `MANUFACTURER "RaceBox"`.
- The "Gnimu Pro" name is marketing only (BLE advertised name is `MODEL + DEVICE_ID`); it cannot live in those protocol fields.

## 10. Toolchain

ESP32 core → **Seeed/Adafruit nRF52 board package**. Drop ESP32 BLE (use Bluefruit) and `Adafruit_MPU6050` (use Seeed LSM6DS3); keep SparkFun u-blox.

## Open items

Several module-level assumptions are checked by the standalone sketches in [`tools`](../tools/nRF52840/) (XIAO + USB, no other parts) — referenced inline below.

- [x] TPS63020 **true disconnect** on EN-low **confirmed** via [`gnss_en`](../tools/nRF52840/gnss_en/gnss_en.ino): GNSS goes silent and the M100 power LED goes fully dark once the RX back-feed is removed — **no load switch needed**. Caveat found: a bare EN cut phantom-powers the GNSS through RX (idle TX D6 at 3.3 V via the RX ESD diode), so `gnssEnable(false)` must idle TX low too.
- [x] GNSS rail validated on the bench: **3V3 pad shorted** (4V2/5V open) and **PS pad shorted** (forced-PWM for a cleaner/lower-ripple rail; efficiency penalty negligible at the GNSS's steady ~30 mA load). With PS shorted the output settles to a tight **3.34 V, stable with *and* without the GNSS load** (LiPo at 3.93 V) — the M100 powers up and acquires satellites. 3.34 V is comfortably inside the M100's **3.3–5 V** input range. (PFM/power-save rides high, which is why the five modules read 3.40–3.50 V unloaded before the PS short; forced-PWM regulates to the true setpoint.) Note: because `LOW_BATT_CUTOFF_V` (3.35 V VBAT) ≈ the 3.34 V output, the buck-boost input stays above its output across the usable LiPo range, so it runs essentially as a buck and never boosts hard under load — the end-of-charge sag case is largely designed out. Optional follow-up: spot-check the rail near the cutoff voltage.
- [x] Buck-boost EN **onboard pullup confirmed**: with the EN lead soldered but left floating (XIAO not yet attached), the module enables and the GNSS powers up — so EN defaults high/enabled when un-driven, as assumed. Firmware actively drives EN **low** to gate the rail off, and low-drive **truly disconnects** the output (now confirmed — see the true-disconnect item above).
- [x] IMU **bus / power / address / units confirmed** via [`imu_probe`](../tools/nRF52840/imu_probe/imu_probe.ino): `PIN_LSM6DS3TR_C_POWER` (pin 15) HIGH enables the IMU; the stock Seeed LSM6DS3 library reaches it out of the box (no manual `Wire`/`Wire1` setup); address `0x6A`; library returns g / deg·s⁻¹ directly. `g_imu.cpp` updated to match.
- [x] IMU **X/Y forward-vs-lateral assignment + signs confirmed** via a real in-car accelerate/brake/turn test (and the equivalent static bench pose) on both Gnimu Classic and Gnimu Pro: both check out clean with **no firmware changes** — `IMU_SWAP_XY = false`, `IMU_SIGN_X/Y/Z = +1.0f` (the untouched defaults) are correct. Convention: +X = forward/accelerating, +Y = left (ISO 8855-style). The swap symptom that prompted this investigation turned out to be a **Gnimu Monitor app bug** (Lateral/Longitudinal display labels bound to the wrong packet fields), not a firmware issue — now fixed in the app. **⚠️ Superseded 2026-08-04:** this conclusion was captured under the earlier mounting (USB-C rearward, an identity map). The as-built module has USB-C **forward** — a 180° yaw — so the correct values are `IMU_SIGN_X = IMU_SIGN_Y = -1.0f`. The intervening single-sign edit to `IMU_SIGN_X` left the map a mirror; see [IMU axis mapping](#imu-axis-mapping) and the bench-verification item below.
- [x] RGB LED **active-LOW polarity and status colors confirmed** via [`led_check`](../tools/nRF52840/led_check/led_check.ino): OFF goes fully dark and every color matches its label, so `config.h` `LED_ACTIVE_LOW = 1` and the `LED_*_PIN` mapping / `g_led.cpp` `setLed()` are correct as-is.
- [x] BLE advertising name, TX power, and **MTU ≥ 91** confirmed via [`ble_mtu`](../tools/nRF52840/ble_mtu/ble_mtu.ino) (MTU 23→247) and the full firmware (streams 25 Hz to a connected client) — `g_ble.cpp`'s `configPrphBandwidth(BANDWIDTH_MAX)` + central-driven `BLEUart` notify path validated.
- [x] M100 draw: using the **published ~30 mA** figure rather than bench-measuring it (measurement deferred — needs a fresh meter fuse or a bench PSU). The 3.34 V rail is confirmed load-stable, and DESIGN §7's battery budget already assumes ~30 mA, so this is a curiosity, not a gate.
- [x] VBAT sampler / presence-detection characterization via [`battery_presence`](../tools/nRF52840/battery_presence/battery_presence.ino): confirmed peak-for-SoC + the `analogSampleTime(40)` TACQ fix, and proved that VBAT-based presence (both the DC-floor and sawtooth-variance branches) is masked when the load equals the charger's output — driving the switch to the hardware switch-sense pin design.
- [x] **Switch-sense divider (2026-07-14):** 2× 510 kΩ resistors between the slide switch's spare pole and A4 (upper) / GND (lower). Bench-confirmed via [`battery_presence`](../tools/nRF52840/battery_presence/battery_presence.ino) (since stripped to a minimal switch-sense-only test): 0–9 mV ON / ~2083–2089 mV OFF, clean transitions, comfortably below/above the 800 mV threshold.
- [x] **State machine Phase 1 (2026-07-14):** module split done (`g_battery` sensor / `g_power` mechanisms / `g_state` orchestrator / `g_led` state-aware) and wired end-to-end. Boot classifier RUNNING/BATTERY_WAIT/DEEP_SLEEP, runtime transitions RUNNING → BATTERY_WAIT (switch-off + 500 ms debounce) and RUNNING → DEEP_SLEEP (voltage cutoff + `gnssEnd()` before hold-off), BATTERY_WAIT → reset on switch-on. See [§5](#5-state-machine).
- [x] **Axis map re-verified on the as-built mount (2026-08-04):** settled by static tilt poses against the 1 Hz serial `milliG` output rather than a drive test — the poses fully determine the sign map, and the Monitor app (the usual in-car readout) is a display layer that has been wrong here before, so it can't validate firmware signs. Result: `IMU_SWAP_XY = false`, `IMU_SIGN_X = -1.0f`, `IMU_SIGN_Y = -1.0f`, `IMU_SIGN_Z = +1.0f`, matching the as-built USB-C-forward orientation. Fixed a mirrored (determinant −1) map that had inverted lateral g. See [IMU axis mapping](#imu-axis-mapping) for the poses, the evidence for the output frame, and why the protocol doc's axis figure is wrong. An in-car run under real dynamics would be nice-to-have confirmation, not a gate. Note the gyro's Y (pitch rate) sign flipped with the accel — correct and intended, but not exercised by static poses.
- [x] **Reconciled [§8](#8-mechanical--enclosure)'s USB-C placement with the as-built module (2026-08-04).** §8 had claimed "USB-C exits the end wall opposite the M100 (GNSS)", which contradicted the bench-verified axis map. Confirmed against the physical build: USB-C is at the **GNSS (forward) end**, feeding a **right-angle adapter whose socket exits the left side wall** — so no USB cutout in either end wall, and the XIAO's BLE antenna faces rearward. §8's orientation and cutout bullets corrected; the RF-separation rationale still holds (BLE antenna far from the GNSS antenna), it was just attributed to the wrong edge of the board.
- [x] **Bench IMU calibration sketch (2026-07-14):** landed at [`imu_calibration`](../tools/nRF52840/imu_calibration/imu_calibration.ino). Measures per-axis accel + gyro zero-point offsets in the raw sensor frame (independent of `IMU_SWAP_XY` / `IMU_SIGN_*`), matches `g_imu.cpp`'s operating settings (range/ODR/bandwidth/BDU), averages 5000 samples at 100 Hz after a 5-minute warmup, and prints six paste-ready `#define` lines for the `IMU_ACCEL_OFFSET_*` / `IMU_GYRO_OFFSET_*` block in `config.h`. `g_imu.cpp`'s `readImuRaw()` subtracts them before `remapAxes()`. Defaults are all `0.0f` so untuned devices behave identically to before.
- [x] **Migrated to SparkFun u-blox library v3 (2026-07-17).** `g_gnss.h`/`g_gnss.cpp` now use `SparkFun_u-blox_GNSS_v3.h` and `SFE_UBLOX_GNSS_SERIAL` (v3's transport-specific class for our exclusive UART usage, replacing v2's monolithic `SFE_UBLOX_GNSS`). Verified against the installed v3 (3.1.14) source before editing: `begin(Stream&, ...)`, `setSerialRate`, `saveConfiguration`, `setUART1Output`, `setDynamicModel`, `setVal8`, `setAutoPVT`, `setAutoPVTcallbackPtr`, `setNavigationFrequency`, `enableGNSS`, `setAopCfg`, `getHeadVehValid`, `checkUblox`, `checkCallbacks` all kept their same signatures (only new optional trailing args added); `UBX_NAV_PVT_data_t` and the `sfe_ublox_gnss_ids_e`/`SFE_UBLOX_GNSS_ID_*` enum are unchanged. No call-site renames needed - the only edits were the `#include` and the class type. Done ahead of the original Phase-2 gate since the GNSS DB persistence feature (the other thing that would have needed re-validating) was reverted first (see above).
- [x] **Phase 2: LIGHT_SLEEP, bench-validated end-to-end (2026-07-19).** Three-tier idle power-shedding, fully wired in `g_state`/`g_gnss`/`g_imu`/`g_power`/`g_led`: (1) `STATE_IDLE_TIMEOUT_MIN` after the last BLE disconnect → GNSS backup mode + IMU wake-arm, reversible without a reset; (2) `STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN` into LIGHT_SLEEP → GNSS escalates backup mode to a full EN-cut (ephemeris presumed stale); (3) `STATE_LIGHT_SLEEP_TIMEOUT_MIN` into LIGHT_SLEEP → full DEEP_SLEEP backstop for a forgotten device. Wake triggers (BLE connect or IMU shake) both confirmed working across many consecutive cycles in one boot session. See [§5](#5-state-machine) for the transition table and the three IMU wake-detector ordering bugs found along the way (FS_XL match, disarm must clear `TAP_CFG1` not just `MD1_CFG`, arm must flush `WAKE_UP_SRC` before routing to INT1), and [LED signal for LIGHT_SLEEP](#led-signal-for-light_sleep-digital-only-no-pwm) for why the LED ended up digital-only instead of PWM breathing. Reference for the wake-up detector programming and low-power ODR modes: ST **AN4650** (*LSM6DS3 always-on 3D accelerometer and 3D gyroscope*), https://cdn.sparkfun.com/assets/learn_tutorials/4/1/6/AN4650_DM00157511.pdf. Not pursued: AN4650 §3.8's **hardware axis orientation** (sign-inversion via `CTRL7_G`, orientation via `CTRL4_C`) — our shipped wake-detect is omni-directional shake-to-wake, which doesn't need it; only relevant if a future *directional* tap-wake is added, since `remapAxes()` in `g_imu.cpp` only reorients the data we read out, not the on-chip embedded functions.

  **GNSS warm standby — VALIDATED on hardware and shipped in `g_gnss.cpp`** (originally proved out via [`gnss_pmreq`](../tools/nRF52840/gnss_pmreq/gnss_pmreq.ino), a standalone diagnostic built for this): `myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX)` (UBX-RXM-PMREQ, infinite duration) puts the receiver into ~µA backup mode with BBR/ephemeris retained, rail still on (`gnssSleep()`). The wake mechanism is **not** what the SparkFun library's own example or docs suggest ("send any UART byte") - framed UBX polls and a plain `Serial1.end()`/`begin()` restart both reliably failed to wake it across many bench runs. The actual wake (`gnssWake()`) is a **deliberate GPIO-level pulse on the shared UART TX line** (release Serial1, `pinMode(GNSS_TX_PIN, OUTPUT)`, drive LOW → HIGH → LOW, then `Serial1.begin()` again, followed by a library re-sync) - confirmed working with holds as short as `GNSS_WAKE_PULSE_MS`. The `forceWhileUsb` flag made no difference (tested both ways). Reacquisition after wake was consistently fast outdoors (sub-2 s typical, ~3.3 s worst case across many runs), confirming ephemeris survives the sleep. One more discovery from integration: **RXM-PMREQ backup mode silently wipes any setting applied via `VAL_LAYER_RAM`-only** (constellations, AutoPVT, nav rate all reverted to receiver defaults on wake, even though the wake pulse itself succeeded) — fixed by targeting `VAL_LAYER_RAM_BBR` for every setter in `gnssBegin()`/`gnssPoll()` instead, made safe by `gnssBegin()`'s existing RX-drain (which prevents a stale BBR-persisted AutoPVT backlog from corrupting the first post-boot command's ACK — see the `gnss-first-command-ack-bug` memory) plus the fact every setter unconditionally overwrites whatever's in BBR, so `config.h` stays the single source of truth on a real reboot regardless. Full test history and reasoning in the [[gnss-idle-cutoff-enhancement]] memory.
- [x] **LiPo chosen: 900 mAh flat pouch with built-in PCM** (overcharge/over-discharge/overcurrent/short-circuit protection) — see [§7](#7-battery-sizing).
- [ ] **Tune the voltage → SoC discharge curve for this specific cell.** Symptom observed 2026-07-14: after a full charge (LED went solid green while USB in), unplugging and reading via Gnimu Monitor showed ~95 %. The curve in `config.h`'s `BATTERY_DISCHARGE_CURVE` is a generic single-cell LiPo table, not a measurement of the installed 900 mAh pouch. Fix is config-only, no code changes: characterize the actual cell (or at least shift the top-of-curve anchors) so a freshly-topped-up pack reads 100 %. Related discussion in [§6](#6-battery--power-subsystem).
- [x] **Log shim wired (2026-07-14).** [`Gnimu/g_log.h`](g_log.h) provides `LOG_PRINT` / `LOG_PRINTLN` / `LOG_PRINTF` / `LOG_FLUSH` as preprocessor-level replacements for the same-named `Serial.*` calls. Every module's diagnostic output routes through the macros; `Serial.begin()` and `while (!Serial …)` stay unconditional as USB CDC lifecycle. `GNIMU_SERIAL_LOG_ENABLED` in `config.h` (default `1`) is the master switch — set to `0` to strip every log call at compile time, which also eliminates the once-per-second stats-printf loop-latency spike that motivated `GNSS_BAUD = 115200` (see [§4](#4-firmware-architecture)).
- [ ] **LIGHT_SLEEP has no USB guard — charging while asleep never enters CHARGE_ONLY.** [§5](#5-state-machine)'s LIGHT_SLEEP transition table checks `cutoffRequested`, `!switchOn`, `bleConnected || imuWakeTriggered()`, and the two timers — but nothing for `usbPresent`. So plugging in while the device is asleep leaves it in LIGHT_SLEEP, charging around a live load: GNSS still in backup mode (or EN-cut, depending on how far the tier timers have run) and BLE still advertising. RUNNING diverts to CHARGE_ONLY on USB precisely to avoid this, since the XIAO's charge IC struggles to sense "battery full" against a parallel load — the same problem, reached by a different path. Plausibly deliberate, as LIGHT_SLEEP's draw is far below RUNNING's and the argument is weaker, but it reads more like a gap in the table than a decision, and it isn't recorded either way. Decide and document: either add `CHARGE_ONLY_ON_USB && usbPresent → CHARGE_ONLY` to LIGHT_SLEEP (entry would need `bleStop()` + `gnssEnd()`, and the reset-based exit already handles the return path), or record why LIGHT_SLEEP is deliberately exempt. Noticed 2026-08-04 while designing the OLED variant's charging indicator — see [`nRF52840-OLED/DESIGN.md` §5](../Gnimu-nRF52840-OLED/DESIGN.md#screen-layout), where it is the case that makes the status bar's charging bolt load-bearing rather than decorative.
- [x] **Added a non-consuming `gnssLatestPvt()` to `g_gnss`, and hoisted the PVT-rate accumulator out of `#if LOG_ENABLED` (2026-08-04).** Landed in all three trees, each compiling clean. Both originated in the OLED variant's display work (see [`nRF52840-OLED/DESIGN.md` §5](../Gnimu-nRF52840-OLED/DESIGN.md#data-sources-for-the-displayed-fields)) but neither is display-specific, and the trees should not drift on either:
  - **`gnssLatestPvt()`** — `gnssConsumePvt()` is consume-once: it returns `nullptr` unless a new epoch arrived and clears the flag on the way out, so exactly one caller can ever see each epoch. `g_telemetry` is that caller today. Any second consumer (the OLED tree's `g_display`, or anything added here later) would race it, and the symptom — dropped BLE packets or a half-rate reader, depending on call order — would look like an intermittent glitch rather than a design error. A non-consuming accessor returning `&latestPVT` plus a "have we ever received one" guard removes the hazard for good. `g_gnss.*` is not in `tools/check_common.sh`'s set, so nothing enforces parity; mirror it deliberately.
  - **PVT-rate hoist** — the epoch counter incremented outside any guard, but the rate computation and the counter reset both sat inside `#if LOG_ENABLED`. With `GNIMU_SERIAL_LOG_ENABLED = 0`, the recommended shipping configuration, the rate was never computed and the counter grew unbounded. Harmless while the rate was only ever printed, but it meant the value silently did not exist in production builds — a trap for any future consumer. Now an unconditional `updateRates()` behind `telemetryGnssRateHz()` / `telemetryBleRateHz()`, with the serial report as a consumer rather than the owner. Two notes from doing it: the *window check* had to move out of the guard as well, since hoisting only the arithmetic would have left the window never closing in a silent build; and the BLE rate had the identical defect, so it was fixed alongside. **`g_telemetry.h`/`.cpp` are in the common set**, so the edit landed in the ESP32 tree too and `check_common.sh` passes.
- [ ] **Discharge-curve logging sketch (later).** Rigorous companion to the curve-tuning item above: a `tools/` sketch that boots on a fully-charged pack (USB unplugged), samples VBAT at a fixed interval, and logs each `{elapsed_ms, mV}` pair to internal flash (Adafruit_LittleFS_InternalFlash on the nRF52840's 1 MB, plenty for a whole discharge). Runs until `LOW_BATT_CUTOFF_V` trips and it enters DEEP_SLEEP. User plugs back in later; a companion "dump" mode reads the log out over USB serial (or the same sketch prints on second boot with USB present). Design decisions to nail down when building it: **sample interval** (5 min gives ~100 points over an 8 h run — comfortably fits in a small flash file); **load profile** (minimal MCU + peripherals held off, or the real ~35 mA RUNNING load? — probably the latter, so the recovered curve matches operating conditions); **retrieval UX** (auto-dump on USB detect vs. gated by a serial command). Output: 22 `{voltage, percent}` anchors at 5 % SoC intervals, ready to paste into `BATTERY_DISCHARGE_CURVE`.
