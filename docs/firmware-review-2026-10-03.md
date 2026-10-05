# Gnimu v2 Firmware Review

**Date:** 2026-10-03
**Scope:** `code/Gnimu/**` (app, core, peripherals, platform for ESP32 and XIAO nRF52840), `tests/`, `scripts/build.sh`, `.github/workflows/ci.yml`. Reviewed against the working tree, including uncommitted and untracked files.
**Library behaviour checked against installed sources:** Seeeduino nRF52 core 1.1.13 (Bluefruit52Lib, Wire, Uart), arduino-esp32 3.3.12 (BLE), SparkFun u-blox GNSS v3.

---

## Verdict

The `core/` layer is in good shape. It is small, host-testable, bounds-checked, and mostly free of hidden state. The problems are in how the pieces are put together at runtime. On the nRF52840, the main target, the design depends on a 1 ms cooperative poll loop draining a **63-byte** UART ring. That leaves about **5.5 ms** of headroom at 115200 baud. Meanwhile at least four code paths on the same loop can block for 2–100 ms or, in one case, forever. There is no hardware watchdog. The GNSS receiver's PVT output is enabled only in volatile layers, and nothing at runtime notices when epochs stop arriving. When the BLE link gets weak or the GNSS rail dips, the device will degrade or go silent, and the LED will keep showing a healthy state.

None of this shows up on a bench with a phone 30 cm away, which is probably why it hasn't been seen yet. It will show up in a car.

The comments are extensive and mostly accurate. They are also long enough to make review harder, and several of them (e.g. the nRF `GnssUart.cpp` note about the 63-byte ring) document a hazard without the code addressing it.

---

## Priority summary

| ID | Pri | Category | Finding |
|----|-----|----------|---------|
| PERF-1 | **P0** | Performance | nRF GNSS ingestion has ~5.5 ms headroom; BLE notify (≤100 ms), console output (≤~50 ms) and I2C (unbounded) all block the same loop |
| REL-1 | **P0** | Reliability | No hardware watchdog on either platform |
| REL-2 | **P0** | Reliability | NAV-PVT output enabled in RAM+BBR only; a GNSS reset silently ends the stream for the rest of the session, and nothing detects it |
| BUG-1 | **P0** | Race | ESP32: a connect/disconnect within one 10 ms BLE tick is never seen, so advertising never restarts. The device becomes permanently unconnectable |
| REL-3 | P1 | Reliability | nRF `Wire` has unbounded spin loops, so a stuck I2C bus hangs the device; ESP32 can block 500 ms before giving up |
| BUG-2 | P1 | Latent bug | GNSS config is verified against the FLASH layer only. BBR overrides flash at startup, so the effective config can differ while the log shows ✅ |
| REL-4 | P1 | Reliability | GNSS configure/PVT-enable failures are logged and ignored; the device enters Running with no data source |
| REL-5 | P1 | Reliability | `GnssFault` is terminal on ESP32 (no retry); on nRF battery it goes dark after 30 s and drains the cell |
| REL-6 | P1 | Reliability | IMU latches down for the session after 100 ms of failed reads; no re-init path |
| REL-7 | P1 | Reliability | `Bluefruit.begin()` failure leaves the app in Running with the LED showing "advertising" |
| BUG-3 | P1 | Observability | BLE `sent` counters are wrong on both platforms; ESP32 congestion drops are invisible |
| PERF-2 | P2 | Performance | Console busy-waits with `micros()`; `D` report formats and emits ~1.4 KB on the real-time loop |
| PERF-3 | P2 | Power/perf | 400 Hz battery job, 1 kHz wakeups, TIMER4 holding HFCLK; all of these continue in CHARGE_ONLY / BATTERY_WAIT |
| PERF-4 | P2 | Performance | ESP32 notify path allocates on the heap for every packet |
| DATA-1 | P2 | Data quality | ~100 ms EMA lag on IMU data, no time alignment to the PVT epoch, 104 Hz ODR polled at 100 Hz |
| PWR-1 | P2 | Robustness | Switch-off stop is accepted on battery (physically impossible, so it can only be noise); switch-on is not debounced |
| BAT-1 | P2 | Robustness | Peak-hold battery sampling is biased upward by noise and ADC ghosting, which can mask a real cutoff |
| HW-1 | P2 | Hardware | `VBAT_ENABLE` driven HIGH for System OFF exposes P0.31 to VBAT through 1 MΩ (verify against Seeed guidance) |
| CLK-1 | P2 | Latent bug | nRF `nowUs()` is non-reentrant and runs from HFINT (±~1.5%) unless HFXO is requested |
| SEC-1..4 | P2/P3 | Security | Connection squatting, silent MTU starvation, open RX for a future decoder, stable tracking identity (accepted) |
| CPLX-1..7 | P3 | Complexity | Pseudo-class GNSS singleton, duplicated IMU/BLE logic, tick-based staleness, platform-layer bypasses, comment bloat |
| MIN-1..6 | P3 | Minor bugs | INT32_MIN negation, period-0 division, unchecked BDU write, UTF-8 width, unbounded console read, battery service never notifies |
| TEST-1..3 | P3 | Testing | No coverage of App/Console/GNSS/BLE; no sanitizers; firmware warnings not fatal; no soak/HIL |

P0 means fix before the device is trusted on track. P1 means fix before calling it field-ready. P2 means schedule it. P3 means cleanup.

---

## 1. Real-time performance and determinism

### PERF-1 (P0): GNSS ingestion on nRF52840 has ~5 ms of headroom, and several paths block much longer

**Where:** `platform/nrf52840/GnssUart.cpp:14-16`, `app/Config.h:40-52`, `app/App.cpp:230-253`, `peripherals/ble/BleNrf.cpp:176`, `app/Console.cpp:93-103`, `core/Scheduler.h:53-79`

**Facts:**
- The Seeed core's `Serial1` RX ring is `SERIAL_BUFFER_SIZE = 64` (63 usable), filled one byte per ENDRX interrupt. At 115200 baud that is about **5.5 ms** of buffering.
- A NAV-PVT frame is 100 bytes and takes **8.7 ms** to arrive, which is longer than the ring can hold. Every epoch therefore needs at least two drains during the burst. Any 5.5 ms stall during a burst loses bytes, the frame checksum fails, and the epoch is dropped.
- The scheduler runs to completion and checks each job once per tick. After `gnss` runs, `imu`, `battery`, `power`, `ble`, `led` and `console` can all run before `gnss` is checked again. Their declared budgets alone total 7 ms. Overruns are counted but nothing prevents them.

**Blocking paths on that same loop:**

1. **`BleNrf::send()` → `BLECharacteristic::notify()` → `BLEConnection::getHvnPacket()`**. This does `xSemaphoreTake(_hvn_sem, ms2tick(BLE_GENERIC_TIMEOUT))` with `BLE_GENERIC_TIMEOUT = 100` ms, and the queue is 3 deep (`configPrphConn(247, 100, 3, …)` for `BANDWIDTH_MAX`). Whenever the link stalls (phone CPU busy, RF shadow, the lead-up to a supervision timeout), every send blocks for **up to 100 ms inside the GNSS job**. At 20 Hz that loses about every other epoch for as long as the link is marginal, which is exactly when you can least afford it. The "measured max 3.2 ms" in `Config.h:43-44` was measured on a healthy link.
2. **`Console::emit()`** busy-waits up to 2 ms per line (`kConsoleWaitUs`). A `D` report is about 22 lines and 1.4 KB, while the nRF USB CDC FIFO is 256 bytes. With a host that is reading, the report needs about 20 or more USB frames, so the loop spins for something like 15–20 ms. With a host that has the port open but isn't reading, it spins 2 ms × 22 ≈ 44 ms and then drops everything. In both cases the time is far beyond 5.5 ms. Pressing `D` while streaming will very likely cost an epoch. Confirm with the report's own `missed` counter.
3. **I2C** (see REL-3). On nRF it is unbounded; on ESP32 a failing read takes about 50 ms.
4. **`Console::log()`** calls from state transitions, BLE edges and IMU failure all go through the same `emit()` wait.

**ESP32** is less exposed: a 512-byte ring gives about 44 ms of headroom, and Bluedroid notify does not block. It is still exposed to items 2 and 3.

**Fix (in order of leverage):**
- **Take byte ingestion off the cooperative loop.** Both platforms run FreeRTOS. Options: a high-priority GNSS task that blocks on UART RX and feeds the parser, or on nRF your own UARTE driver with double-buffered EasyDMA (e.g. 2×256 B plus an RX timeout via TIMER/PPI, as in Nordic's `libuarte`). Either one turns the 5.5 ms constraint into hundreds of ms and removes the 1 kHz poll.
- **Take BLE sends off the GNSS path.** Encode into a single-slot "latest packet" mailbox and send from a separate job or task that is allowed to block. Count and drop stale packets instead of queueing them. Alternatively, raise `hvn_qsize` (e.g. `configPrphConn(247, evtLen, 8, …)`) *and* never call `notify()` from the ingestion context.
- **Console**: see PERF-2.
- **Scheduler**: if the cooperative model stays, re-run the highest-priority due job between lower-priority jobs, i.e. restart the scan from index 0 after every job run. That puts a ceiling on the latency between GNSS drains equal to one job's worst case, not the sum of all of them.

### PERF-2 (P2): Console output is a busy-wait on the real-time loop

**Where:** `app/Console.cpp:40-55`, `93-103`, `14-38`

- `emit()` spins on `availableForWrite()` against `micros()`. On nRF, `micros()` has 1 ms resolution per your own `Clock.cpp:14`, so the "2 ms" wait is really somewhere between 1 and 2 ms. Use `PlatformClock::nowUs()`.
- After one drop, every following line also waits the full timeout. Latch a "host stalled" state on the first timeout and drop immediately until `availableForWrite()` recovers.
- `printReport` formats 1.4 KB with `vsnprintf` and emits it all in one job run. Spread it across ticks (one line per console tick from a static buffer) or accept dropping it.
- `poll()` loops `while (Serial.available())` with no bound. A host pasting a large blob keeps the loop there. Cap it, e.g. at 64 bytes per tick.

### PERF-3 (P2): Wakeup rate and clocking waste power and CPU, in every state

**Where:** `app/Config.h:78-81`, `platform/nrf52840/Clock.cpp:18-25`, `platform/*/Clock.cpp:sleepUntilUs`

- The battery job runs every **2.5 ms (400 Hz) forever** to collect 20 samples every 250 ms. That is 380 wasted wakeups per second. It keeps running in CHARGE_ONLY and BATTERY_WAIT, where nothing else runs. Change it to one 250 ms job that starts a burst, or use SAADC oversampling/burst so the hardware averages.
- The GNSS drain forces 1 kHz wakeups (fixed by PERF-1's DMA/task change).
- TIMER4 in 32-bit timer mode keeps the 16 MHz HFCLK running permanently (hundreds of µA). That doesn't matter while streaming, but it does in CHARGE_ONLY and BATTERY_WAIT. Stop TIMER4 when entering those states, or switch the timebase to RTC2 (32.768 kHz) with TIMER4 only for sub-ms deltas.

### PERF-4 (P2): ESP32 notify allocates every packet

**Where:** `peripherals/ble/BleEsp32.cpp:172-173`

`BLECharacteristic::setValue()` stores into a `BLEValue` (an Arduino `String`) and `notify()` walks a `std::map` of peers. That means heap allocation and free 20 times a second for the length of a session. Fragmentation on a long-running Bluedroid heap is a real failure mode. Prefer `esp_ble_gatts_send_indicate()` directly with a static buffer and the cached handle, or move to NimBLE-Arduino, which is lighter and has a non-allocating notify path.

---

## 2. Reliability and robustness

### REL-1 (P0): No watchdog

`grep -ri wdt` finds nothing. There is no hardware watchdog on either platform. With REL-3's unbounded I2C loops, Bluefruit's internal semaphores, a SoftDevice assert, or any future infinite loop, the device stops until someone power-cycles it. In a car that means after the session, when the data is already lost.

**Fix:**
- **nRF:** start the WDT (`NRF_WDT`, e.g. 2 s, `CONFIG.SLEEP=Run` / `HALT=Pause`) after bring-up and feed it from the scheduler loop only when the GNSS job has run recently. That makes the watchdog prove real progress, not just "the loop is spinning". The nRF WDT cannot be stopped once started. That suits System OFF, where it stops, but `BatteryWait`/`ChargeOnly` must keep feeding it, which they will because the scheduler keeps running.
- **ESP32:** `esp_task_wdt_add()` on the loop task, fed under the same rule.
- Record the reset reason (`NRF_POWER->RESETREAS`, `esp_reset_reason()`) at boot and print it. Right now a watchdog reset would be indistinguishable from a power cycle.

### REL-2 (P0): PVT output is volatile, and silence goes undetected

**Where:** `peripherals/gnss/Gnss.cpp:203-245, 267`; `app/App.cpp:230-242`, `315-344`

- Every setting in `settings[]` is persisted to flash **except the one that matters most**: `setAutoPVTcallbackPtr()` defaults to `VAL_LAYER_RAM_BBR` (`u-blox_GNSS.h:795`). If the receiver resets on its own (a brown-out on the GNSS rail during engine cranking, ESD, a firmware watchdog in the module) and V_BCKP isn't independently powered, it comes back from flash with NAV-PVT output **off**. The MCU stays in `Running`, BLE stays connected, the LED shows solid blue, and nothing is streamed for the rest of the session.
- Nothing watches epoch arrival at runtime. `GnssSpeedGate` tracks staleness only to gate trim. `EpochTracker` counts gaps only for the `D` report.

**Fix:**
- Add `UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1` to the flash-ensured list. Register the callback with `setAutoPVTcallbackPtr(&onPvt, VAL_LAYER_RAM)` or skip the write once the key is persisted.
- Add a GNSS liveness check, e.g. no epoch for 3 × nav period (or 2 s). Escalate in steps: log it, re-run `configure()`; if that fails, power-cycle the GNSS rail on nRF (you already have the control) or reboot. Show it on the LED.
- Consider a module-reset detector: UBX-INF/NOTICE boot banners, or the `iTOW` regression that `EpochTracker` already counts as `timeSteps`.

### REL-3 (P1): I2C can hang the nRF forever and stall the ESP32 for 500 ms

**Where:** `peripherals/imu/Lsm6ds3.cpp:52-56`, `peripherals/imu/Mpu6050.cpp:58-77`; core `Wire_nRF52.cpp:171-186, 235-252`

- The Seeed nRF `TwoWire::requestFrom/endTransmission` contain `while(!EVENTS_STOPPED);` and `while(!EVENTS_SUSPENDED);` with **no timeout and no error escape**. A slave holding SDA low after a brown-out mid-transaction, or a cable disturbance on an external IMU, hangs the loop permanently. With no watchdog (REL-1), the device is dead.
- On ESP32 the default Wire timeout is about 50 ms. `ImuHealth` needs 10 consecutive failures, so that's about 500 ms of blocking before the IMU is marked down. Each 50 ms stall is longer than the ESP32's 44 ms GNSS ring, so every failed read during that window can also cost an epoch.

**Fix:** Use a bounded I2C transaction on nRF. Either write a small TWIM wrapper with a µs deadline and a `TASKS_STOP` fallback, or at least run IMU I/O in its own task so that a hang trips the watchdog and not the whole product. Add I2C bus recovery: toggle SCL 9 times as GPIO, then send a STOP, before re-init. On ESP32, `Wire.setTimeOut(2)` (ms) is enough for a 14-byte read at 400 kHz (~0.4 ms).

### BUG-2 (P1): Configuration is "verified" against the wrong layer

**Where:** `peripherals/gnss/Gnss.cpp:162-175`

On a u-blox receiver, the effective (RAM) configuration at startup is *defaults ← flash ← BBR*: **BBR overrides flash**. `ensureVal()` reads only `VAL_LAYER_FLASH`. If BBR holds a different value (left by a previous firmware, u-center, or Classic), the flash read matches, the log prints ✅, and the receiver actually runs the BBR value. That could be the wrong dynamic model, nav rate or constellations, which is exactly the kind of quiet misconfiguration this code is supposed to catch.

**Fix:** Read `VAL_LAYER_RAM` (effective) as well. If RAM ≠ desired, write `VAL_LAYER_ALL` (RAM+BBR+flash) and re-read RAM *and* flash. Better still, batch it: one `newCfgValset`/`addCfgValset`/`sendCfgValset` transaction for all 30 keys instead of 30+ round trips, each with up to three retries at a 1.1 s timeout. The current worst case for a receiver that ACKs nothing is several minutes of boot time, with no battery cutoff monitoring because the scheduler hasn't started.

### REL-4 (P1): Configuration failures don't change behaviour

**Where:** `peripherals/gnss/Gnss.cpp:258-271`, `274-280`

`configure()` returns nothing. `allOk == false` and `setAutoPVTcallbackPtr()` failing are both log-only. `Gnss::begin()` returns `true` regardless, and the app enters `Running` with BLE advertising. A device that cannot produce epochs should not look identical to one that can. Return a status, retry once, and if it still fails go to a degraded/fault state with a distinct LED pattern.

`switchToTargetBaud()` also ignores `setSerialRate()`'s return value. That one happens to be caught by the `begin()` that follows, but only by luck.

### REL-5 (P1): `GnssFault` is a dead end

**Where:** `app/App.cpp:152-155, 168-174`; `core/StatusLed.h:57-58`

- **ESP32:** the battery and power jobs are disabled (`App.cpp:107-110`), so nothing can leave `GnssFault`. A receiver that is slow to boot, momentarily disconnected or browned out at power-on bricks the session until a manual reset.
- **nRF on battery:** the LED goes dark after 30 s (`kLedFaultMs`) while the MCU stays awake with the console, battery, power and LED jobs at 400 Hz. To the user the device looks off, but it keeps draining the cell to cutoff.

**Fix:** Retry detection on a backoff (e.g. 5 s, 10 s, 30 s, with a rail power-cycle on nRF). After N failures on battery, go to System OFF instead of going dark and staying awake.

### REL-6 (P1): IMU is latched down permanently after 100 ms of glitches

**Where:** `core/ImuHealth.h:21-32`

Ten consecutive failures at 100 Hz is 100 ms. Ignition noise, a loose connector on the external MPU-6050, or the I2C recovery case in REL-3 will take the IMU out for the whole session, and it then reports zeros (which a consumer can't tell apart from "level and still"). "Matching Classic" is not a reliability argument. Add a re-init path: after going down, attempt `begin()` with backoff, and keep a counter of recoveries for the `D` report.

### REL-7 (P1): BLE stack failure is invisible

**Where:** `peripherals/ble/BleNrf.cpp:69-72`; `app/App.cpp:315-344`

If `Bluefruit.begin()` fails, `begin()` returns with `started_ = false`. The app stays `Running`, and `ledJob` shows `Advertising` (blinking blue) because `connected()` is false. The one visible status indicator lies. The nRF initial `Advertising.start(0)` failure is also log-only (`BleNrf.cpp:122-123`). Give the transport a `healthy()` signal and map failure to a fault LED state, with a retry or reboot.

### PWR-1 (P2): Power policy trusts the switch-sense line more than physics allows

**Where:** `core/PowerPolicy.h:45-75`; `platform/nrf52840/BatteryPort.cpp:54-62`; `platform/nrf52840/BoardConfig.h:24-27`

- The switch-sense input is a 510k/510k divider read by the ADC: high impedance, in a car. On battery, "switch OFF" physically removes power, so an OFF reading while `!usbPresent` *can only be noise*. Yet `update()` will still debounce it for 500 ms and return `StopForSwitch`. That turns off the GNSS rail mid-session, which loses ephemeris unless the receiver has backup power, and then reboots on the next ON reading. **Require `usbPresent` for `StopForSwitch`.**
- Switch-ON from `BatteryWait` is not debounced: one ghost reading reboots. That is less harmful, but it is the same class of problem. Debounce both edges.
- Add an RC filter on the sense line in hardware if it isn't already there.

### BAT-1 (P2): Peak-hold sampling hides low cells

**Where:** `core/BatteryMonitor.h:55-58, 82-101`

Taking the max of 20 samples maximises upward noise. With ADC noise σ, the peak sits about +2σ above truth, and a single EMI spike sets the run's value outright. The cutoff watches this "fresh" peak, so noise biases the protective path towards *not* cutting off. In addition, VBAT reads that immediately follow a switch-sense read (switch OFF ≈ 2 V at the pin vs ≈ 1.3 V for VBAT) carry ghost charge upward, which is the effect `switchOn()` itself guards against but `readCellMv()` doesn't. Use a high percentile (e.g. second-highest of 20) or a trimmed mean of the top quartile, and discard the first VBAT conversion after any channel switch.

### HW-1 (P2, verify): `VBAT_ENABLE` HIGH during System OFF

**Where:** `platform/nrf52840/BatteryPort.cpp:64`, `app/App.cpp:316`

On the XIAO nRF52840, driving P0.14 HIGH turns off the divider's low-side switch, and P0.31 is then pulled towards VBAT through 1 MΩ. Seeed's own guidance (and several community reports) is to keep it LOW for exactly this reason: VBAT can exceed VDD + 0.3 V. The current is limited to sub-µA, so damage is unlikely, but the device would sit in System OFF for weeks in an out-of-spec condition. Confirm against the schematic and Seeed's current advice. If the concern holds, prefer leaving VBAT_ENABLE LOW (the divider draws ~2.8 µA at 4.2 V) or setting P0.31 to an input with the pin disconnected.

### DATA-1 (P2): IMU data is lagged and not time-aligned

**Where:** `app/Config.h:114-115`; `peripherals/imu/Lsm6ds3.cpp:24-26`; `app/App.cpp:244-249`

- An EMA with α = 0.09 at 100 Hz has a time constant of about 105 ms. Each RaceBox packet pairs a PVT epoch with an accelerometer value that reflects roughly the *previous 100–200 ms*. For braking-point and g-g analysis that is a visible smear and phase lag. If smoothing is wanted, use a short FIR/biquad with known group delay, or send the decimated mean of the samples since the last epoch.
- The packet pairs the PVT with whatever IMU value exists when the PVT finishes parsing. That is after receiver compute latency plus 8.7 ms of UART transfer, so the IMU sample is from *after* the PVT measurement epoch. Timestamp IMU samples and pick, or interpolate, the one nearest the epoch time.
- The LSM6DS3 runs at 104 Hz ODR and is polled at 100 Hz without checking data-ready, so roughly one sample in 26 is skipped. That's harmless for trim, but it is aliasing, and it's free to fix (use DRDY or the FIFO).

---

## 3. Latent bugs, races and unchecked results

### BUG-1 (P0): ESP32 can stop advertising permanently

**Where:** `peripherals/ble/BleEsp32.cpp:38-49, 123-152`

Re-advertising on ESP32 is driven purely by `update()` noticing a falling edge of `linkUp`, sampled every 10 ms. The controller stops advertising when a connection is established. If a central connects and disconnects within one `bleJob` period (an aborted connection, a scanner probing GATT, iOS retrying), `update()` sees `linkUp == false` both times, never sees an edge, never sets `readvertisePending_`, and **the device never advertises again**. `BLEServer::m_advertiseOnDisconnect` defaults to false (`BLEServer.cpp:93`), so the library won't save you.

**Fix:** In `onConnect`/`onDisconnect`, increment atomic connect/disconnect counters, and have `update()` compare counters instead of sampling a level. Or call `server->advertiseOnDisconnect(true)` and keep the delayed path only for logging. The nRF side has the same edge-sampling pattern, but Bluefruit's `restartOnDisconnect(true)` covers it, so there only the log lines are lost.

### BUG-3 (P1): BLE throughput statistics are wrong

**Where:** `peripherals/ble/BleEsp32.cpp:172-175`, `peripherals/ble/BleNrf.cpp:176-178`

- ESP32: `notify()` returns `void`. Congestion (`ESP_GATT_CONGESTED`), CCCD-disabled and no-client cases are reported only through `onStatus` callbacks you haven't registered. `sent_++` runs anyway, so the status line's "BLE: 20Hz" can be fiction.
- nRF: a `false` from `notify()` (CCCD off, or 100 ms timeout) is returned but never counted.

You can't tune a streaming link you can't measure. Count `sent`, `failed`, `refusedMtu` and `blockedUs` (max time spent inside send) on both platforms, and put them in the `D` report.

### CLK-1 (P2): `PlatformClock::nowUs()` on nRF is not reentrant, and its clock source is imprecise

**Where:** `platform/nrf52840/Clock.cpp:27-35`

- `TASKS_CAPTURE[0]` followed by reading `CC[0]` plus the `static last/wraps` read-modify-write is not safe against preemption. Today only the loop task calls it, but Bluefruit callbacks run in preemptive FreeRTOS tasks. The first time someone timestamps an RX write or a connect event with `nowUs()`, wraps will occasionally double-count, a 71-minute jump. Use a dedicated CC channel per context, or disable interrupts around the capture and update, and document it as loop-only with an assert.
- TIMER4 runs from whichever HFCLK source is active. Without `sd_clock_hfclk_request()` that is HFINT, which is only accurate to about ±1.5%. `maxGapUs`, the scheduler grid and the status RT drift accordingly. Request HFXO while running.

### MIN-1..6 (P3): Minor defects

| ID | Where | Issue |
|----|-------|-------|
| MIN-1 | `core/StatusLine.h:41` | `-v1e7` is UB for `INT32_MIN`. Use `0u - static_cast<uint32_t>(v)`. |
| MIN-2 | `core/Scheduler.h:61` | Divides by `periodUs` with no guard. Only some periods are `static_assert`ed positive. Assert in the `Scheduler` constructor. |
| MIN-3 | `peripherals/imu/Lsm6ds3.cpp:137-139` | The `CTRL3_C` (BDU) write result is ignored, unlike the MPU path, which reads its config back. A failed write silently allows torn samples. |
| MIN-4 | `core/Report.h:83-85` | `"max μs"` is 7 bytes for 6 glyphs (`μ` is 2 bytes in UTF-8), so `right()` misaligns the header. |
| MIN-5 | `app/Console.cpp:16` | Unbounded `while (Serial.available())` (see PERF-2). |
| MIN-6 | `peripherals/ble/BleNrf.cpp:181-185` | `BLEBas::write()` updates the value but doesn't notify subscribers, so a client subscribed to Battery Level never sees changes. Use `notify()`. Also, `batteryJob` calls this at 400 Hz. |

`App::powerJob` relies on `[[noreturn]]` instead of `break` for fall-through between cases (`App.cpp:286-291`). That is correct today, but it is one attribute removal away from a real fall-through, and `-Wimplicit-fallthrough` may flag it. Add an explicit `break;` after each call.

---

## 4. Security

The stated threat model accepts an unauthenticated BLE peripheral that broadcasts a stable identity and streams location data. Within that model:

- **SEC-1 (P2): Connection squatting / no idle-client policy.** Only one central can connect. Any device that connects and then never enables notifications, or negotiates MTU < 91, holds the slot indefinitely. The real user's phone can't connect, the LED shows "connected", and nothing streams. Add a policy: if no successful notify within N seconds of connecting (CCCD not enabled, or MTU too small), disconnect and re-advertise.
- **SEC-2 (P2): MTU starvation is silent to the user.** `send()` refuses frames when `len + 3 > mtu` and logs once on the serial console, which nobody has in the car. Combine with SEC-1 so the device disconnects, and consider calling `requestMtuExchange()` from the peripheral side on nRF.
- **SEC-3 (P3): RX path is open and will feed a parser.** The 256-byte SPSC ring is correct and bounded today. When the command decoder lands, it must treat input as hostile: fixed-size frames, a length check before any copy, checksum before action, and no command that writes to the GNSS receiver's flash or changes power state without rate-limiting. Write those rules down now, before the decoder exists.
- **SEC-4 (P3, accepted): Stable identity.** The name suffix and DIS serial are derived from FICR/eFuse, and nRF uses a static random address, so the device is trivially trackable across sessions. That is accepted per the brief; noted only for completeness.
- No OTA/DFU over BLE, no secrets on the device, and console input is parsed with bounded buffers. Good.

---

## 5. Complexity and maintainability

- **CPLX-1: `Gnss` is a class in name only.** All of its state (`receiver`, `latestEpoch`, `epochPending`) lives in file-static globals (`Gnss.cpp:14-16`), and the instance holds only a log pointer. Two instances would silently share everything. Either make it a namespace or move the state into the class.
- **CPLX-2: Duplicated IMU pipelines.** `Mpu6050::begin/poll` and `Lsm6ds3::begin/poll` repeat the same trim-config construction (13 lines each), filter/health setup, the poll body, `step()` and the "not found" logging. Split it into a chip driver (`bool begin()`, `bool read(ImuSample&)`) plus one shared `ImuPipeline` (health, trim, filter, logging). That also makes REL-6's re-init path one change instead of two.
- **CPLX-3: Duplicated BLE policy.** The MTU-refusal logging state machine is copied between `BleEsp32` and `BleNrf`. Move the policy (refusal tracking, counters, idle-client timeout from SEC-1) into a platform-independent wrapper, and leave only the stack calls in the platform files.
- **CPLX-4: Tick-based staleness.** `GnssSpeedGate` counts IMU job invocations as a proxy for time, so scheduler skips stretch it, and it adds a second "tick" concept to the codebase. Store `arrivalUs` and compare against `PlatformClock::nowUs()`. It's simpler and correct.
- **CPLX-5: Platform-layer leaks.** `Console` calls `Serial` and `micros()` directly, while every other hardware touch goes through `platform/`. `Gnss.cpp` uses `delay()` throughout. `BleEsp32` uses `millis()`. Pick one clock.
- **CPLX-6: `ensureVal` machinery** (template + `ensureVal8/16` wrappers + `getFlash/setAll` overload pairs + `readFlash` retry template) is about 50 lines to read and write 30 keys. A single batched VALGET/VALSET (see BUG-2) replaces most of it and is faster.
- **CPLX-7: Comment volume and history narration.** Many comments are longer than the code they describe, and a large share narrate provenance ("Classic's", "NG's choice", "seen: GNSS: 50Hz"), which goes stale as the code evolves. The two-layer ESP32 guard explanation is pasted into about 10 files. Keep the *why* that constrains future edits, and move the history and rationale into `docs/`. Long comments also hide real gaps: the nRF `GnssUart.cpp` comment describes the 63-byte hazard precisely, and the architecture never addresses it.

---

## 6. Testing and tooling

- **TEST-1 (P3):** Host tests cover `core/` well (scheduler, encoder, trim, policy, monitor). Nothing covers `App`'s state machine, `Console::poll` parsing, the GNSS config flow, or BLE edge handling, which is where the P0/P1 bugs above are. Make `App` testable against fakes (the `Clock` template parameter on `Scheduler` already shows the pattern), and add a fake BLE transport that can simulate fast connect/disconnect and blocking sends.
- **TEST-2 (P3):** `tests/run.sh` builds at `-O1` without sanitizers. Add a `-fsanitize=address,undefined` pass. It would have flagged MIN-1.
- **TEST-3 (P3):** Firmware builds use `--warnings all` but don't fail on them. Add `-Werror` for project sources (not the vendor libraries), or at minimum have CI grep for warnings from `code/Gnimu/src`.
- **Missing entirely:** a soak/HIL test. A long run (hours) with a phone at range, a deliberately flaky link (shield the phone), `D` pressed periodically, and the `missed`/`skipped`/`overruns` counters checked at the end would have surfaced PERF-1 directly. Make it a release gate.

---

## 7. What's done well

These matter because they should be kept while the above is fixed:

- `core/` is genuinely host-testable, and the bounded string builder (`TextOut.h`) is used consistently. I found no buffer overruns in any formatter.
- `ByteRing` is a correct SPSC queue with proper acquire/release ordering and counted drops.
- Hardware-safe ordering for power transitions (UART closed → TX idled low → rail off) is deliberate and correct.
- The RaceBox encoder saturates instead of wrapping, and the fix-type and validity flag mapping is careful.
- Config constants carry `static_assert`s for most invariants.
- Library and core versions are pinned in CI.

---

## Recommended order of work

1. **REL-1** watchdog plus reset-reason logging. This is cheap and turns every hang below into a recoverable glitch.
2. **BUG-1** ESP32 re-advertise counters. It's a one-hour fix.
3. **REL-2** persist the NAV-PVT output and add a GNSS liveness check with escalation.
4. **PERF-1** move UART ingestion to DMA or a task, and decouple BLE send from it. This is the structural fix. Validate with a weak-link soak.
5. **REL-3** bounded I2C plus bus recovery; **REL-6** IMU re-init.
6. **BUG-2 / REL-4 / REL-5 / REL-7** config verification against RAM, failure states that the LED reflects, and fault retry.
7. **BUG-3** honest BLE counters, then **SEC-1/2** idle-client policy.
8. P2 items (power, battery sampling, data alignment, clock), then P3 cleanup.
