# Gnimu v2 Firmware Review: Follow-up

**Date:** 2026-10-09
**Scope:** the tracker ([firmware-review-2026-10-03-response.md](firmware-review-2026-10-03-response.md)), then every firmware source as of commit `b8f414f` (6,338 lines across `code/Gnimu/**`), `tests/`, `scripts/build.sh` and CI.
**Method:**
- Read every source file in full.
- Checked library behaviour against the installed Seeeduino nRF52 1.1.13 (Bluefruit, Uart, Wire), arduino-esp32 3.3.12 (BLE, HardwareSerial) and SparkFun u-blox GNSS v3 sources.
- Ran `tests/run.sh`: all 22 suites pass, plain and sanitized.
- Ran scratch host programs to measure the formatters' worst-case output against their buffers.
- No hardware was run.

---

## Verdict

The codebase is in substantially better shape than on 2026-10-03:
- **Hangs** now end in a watchdog reset that the next boot reports.
- **A silent GNSS** now leads to a restart, or a fault if it never sent anything.
- **GNSS configuration** is now verified against the layer the receiver actually runs.
- **Run-state logic** is pure and host-tested.
- **Duplicated driver code** is gone.

The decisions in the tracker are well reasoned, and the declines are defensible with the data behind them. I don't disagree with any of them.

**The one material risk left is the one you already know about:** the nRF BLE send runs inside the GNSS job, and Bluefruit can block it for up to 100 ms. The soak is the right gate, and two cheap knobs below should be tried during it.

The new findings are small. One is a real behaviour problem created by two individually good decisions interacting (N1). Two are buffer margins that have quietly eroded as the reports grew (N2, N3). The rest are minor correctness or clean-up items. I found no new memory-safety or concurrency defects.

---

## Priority summary

| ID | Pri | Category | Finding |
|----|-----|----------|---------|
| N1 | P2 | Behaviour | One noisy OFF reading at boot on USB now lands in CHARGE_ONLY, not a reboot. Telemetry stays off until the switch is cycled. |
| N2 | P2 | Latent bug | Status line: a realistic worst case is 251 of 256 bytes, so a little more content loses the newline. It also prints raw `4294967295` for accuracy the receiver doesn't know. |
| N3 | P3 | Latent bug | The D report's worst case is 1,860 bytes for a 1,536-byte buffer (about 1,570 after 30 days up). The tail (Battery, Console lines) is silently truncated. The comment's "~1420" is stale. |
| N4 | P3 | Correctness | Two console commands that arrive in one pass: the second overwrites the first. |
| N5 | P3 | Robustness/UX | The LED is dark throughout Starting: up to ~31 s on a no-receiver baud sweep, so the device looks dead before the fault blink starts. |
| P1 | P1 | Performance | (Known: PERF-1.) Soak additions: the connection-interval and send-queue-depth knobs, plus the readings to take. |
| P2 | P3 | Build | The ESP32 image fills 91% of its app partition. A larger app partition is a one-word change. |
| C1 | P3 | Complexity | `PowerPolicy` keeps its own run state (`running_`, `chargingOnly_`) alongside `RunStateMachine::state_`. |
| C2 | P3 | Complexity | Dead or test-only API, listed below. |
| C3 | P3 | Layering | `core/ResetReason.h` pulls in the whole state machine for one enum and its text. |
| I1 | Info | Note | The NAV-PVT output key is checked only in flash, so a stale BBR value would survive. It self-heals through liveness. |

P1 means do it with the soak. P2 means fix soon. P3 means clean-up. Info needs no action.

---

## Findings

### N1 (P2): A boot-time ghost OFF on USB now means CHARGE_ONLY

**Where:** `core/PowerPolicy.h:80-89` (`begin`), `:91-100` (`update`); `app/App.cpp:86-90`.

**What happens:**
1. `begin()` classifies the boot on a single switch reading. On USB, OFF → `StopForSwitch` → BATTERY_WAIT, with `onEntersChargeOnly_ = true`.
2. The switch is actually ON, so the debounced reading settles at ON 500 ms later.
3. From BATTERY_WAIT, ON with USB → `EnterChargeOnly`.

So one ghost OFF at power-up, on USB, puts a device that should be running into CHARGE_ONLY. It stays there with GNSS, IMU and BLE off until someone cycles the switch.

**Why it's new:** PWR-1 considered "several readings at boot" and dropped it, because the only remaining case was "one extra reboot on the bench": BATTERY_WAIT followed by switch-on used to mean restart into RUNNING. The later owner change made switch-on from BATTERY_WAIT mean CHARGE_ONLY. That turned the same ghost into a sticky wrong state. Each decision was fine on its own; together they change the consequence.

**Exposure:** USB only. The USB rule makes an OFF reading impossible on battery, so the car is unaffected. It does affect running on the bench, and running while plugged into a charger.

**Fix (costs nothing on a normal boot):** confirm an OFF reading at boot before acting on it, only when USB is present. For example, take 3–5 readings about 20 ms apart, and treat the switch as OFF only if they all agree. A normal boot (switch ON) never takes that path. A real plugged-in-and-off boot pays about 60–100 ms once. Add a `test_powerpolicy.cpp` case for "boot OFF ghost on USB, then steady ON" → Running (or a restart into it), not CHARGE_ONLY.

### N2 (P2): Status-line buffer margin is down to a few bytes

**Where:** `app/Console.cpp:65-71` (256-byte buffer, comment "~185 bytes"); `core/StatusLine.h:59-63`.

**Measured** (scratch host program, real formatter):

| Case | Bytes | Buffer |
|---|---|---|
| Realistic worst | **251** | 256 |
| Extreme values | 299 | 256 |

The realistic worst case is: no fix yet (hAcc and tAcc are `4294967295`), 10-day RT, charging battery, full-scale IMU, lat/long near ±90/±180.

Adding a field, or a longer battery cell, puts the line over the buffer. `Out` then truncates and drops the trailing `\n`, which is exactly the failure this buffer was once sized up to fix.

**Separately:** the status line prints the receiver's "unknown" sentinel raw (`tAcc: 4294967295ns | hAcc: 4294967295mm`), while D already prints `unknown`. That is inconsistent, and it is also what makes the realistic case long.

**Fix:**
- Print `unknown` in the status line, as D does. That alone saves about 14 bytes.
- Raise the buffer to 320.
- Correct the comment. It stays under the nRF's 256-byte CDC FIFO only if the line does, so either assert the formatted length ≤ 255, or note that a longer line can never be sent on the nRF (`emit()` would wait 2 ms and drop it on every tick).

### N3 (P3): The D report can outgrow its buffer

**Where:** `app/Console.cpp:48-63` (`static char buf[1536]`, comment "Worst case ... ~1420 bytes").

**Measured:**

| Case | Bytes | Buffer |
|---|---|---|
| Every counter at 10 digits | **1,860** | 1,536 |
| 30 days of uptime at the real job rates | about **1,570** | 1,536 |

The report gained the `max late μs` column, the `Reset:` line, the GNSS settings line and the BLE/battery wording since the comment was written. Truncation is silent and takes the last lines: Battery, then Console.

Today's sessions are far below the limit, so this is a margin issue, not a field bug.

**Fix:** 2,048 bytes (static RAM, 512 B more), with the comment updated. Optionally add a host test that formats a 10-digit worst case and asserts it fits, so the next added line fails a test rather than truncating on the device.

### N4 (P3): Two commands in one pass, one lost

**Where:** `app/Console.cpp:20-38`; `app/Console.h:13-19`.

`poll()` reads up to 6 bytes per pass, and `ConsoleCommand` holds one `kind`. If `D⏎C⏎` arrives together (a paste, a scripted host, or a slow console pass), the second entry overwrites the first: only `C` runs, and `D` is dropped silently. The header comment says both a command and a count can come through in one pass, which is true, but two commands can't.

**Fix:**
- Simplest: stop reading at the first completed entry, so the rest waits for the next 10 ms pass. That needs no change to `ConsoleCommand`, and any remainder is still bounded by the existing discard rule.
- Alternatively, document the limit.

### N5 (P3): Dark LED during Starting

**Where:** `app/App.cpp:61` (LED off at boot), `:146-167` (Starting); the LED job runs only once the scheduler starts.

Starting blocks before the scheduler runs, so the LED stays off throughout:
- **nRF on USB:** up to 3 s waiting for the host to enumerate, before Starting.
- **Normal GNSS detection:** about 1–2 s.
- **No receiver:** the full 9-rate sweep takes about 9 × 3.5 s ≈ **31 s** before the fault blink starts.

For half a minute the device looks unpowered.

**Fix (owner's call):** at Starting entry, call `StatusLedPort::write(true, <colour>)` directly, e.g. solid blue or amber, as a "booting" indication. The LED job overwrites it as soon as the scheduler runs. It's one line and needs no new state.

### I1 (Info): The NAV-PVT output key is checked in flash only

**Where:** `peripherals/gnss/Gnss.cpp:184-187, 322`.

`Check::FlashOnly` is right while configuring. Its side effect: if BBR holds `MSGOUT_UBX_NAV_PVT_UART1 = 0` (u-center, or an older tool), every receiver self-reset comes back silent, because BBR overrides flash. Liveness then restarts the MCU and the boot fixes RAM. So it self-heals, at the cost of one restart per receiver reset. No action unless a field log shows repeated `GNSS: epochs stopped` restarts. Writing the key with `VAL_LAYER_ALL` when flash is already right but RAM is 0 would close it, but that conflicts with the deliberate RAM pause.

---

## Performance: additions for the planned soak (P1)

PERF-1 is correctly deferred to the weak-link soak. It is still the biggest risk: `BleNrf::send()` → `BLECharacteristic::notify()` → `getHvnPacket()` waits up to `BLE_GENERIC_TIMEOUT` (100 ms) for one of the **3** HVN slots that `BANDWIDTH_MAX` configures. That happens inside the GNSS job, on a 5.5 ms UART ring.

**Two cheap knobs to try in the soak before writing any new driver or task.** Both are configuration only:

1. **Send-queue depth.** Call `Bluefruit.configPrphConn(247, 100, <hvn_qsize>, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT)` (what `BANDWIDTH_MAX` sets, with only the queue depth changed) instead of `configPrphBandwidth(BANDWIDTH_MAX)`, with `hvn_qsize` around 8–10. With 3 slots at 20 Hz, a link stall longer than about 150 ms starts blocking. With 10 it takes about 500 ms. It costs SoftDevice RAM (`Bluefruit.begin()` reports if the RAM start needs moving).
2. **Connection interval.** No preferred interval is requested today, so the phone chooses (iOS typically 30 ms, Android 7.5–50 ms). `Bluefruit.Periph.setConnInterval(12, 24)` (15–30 ms) gives the queue more drain opportunities per second. Check what each phone actually grants.

**Readings to take in each soak phase (close, marginal, lost, close):**
- `gnss` max μs and max late μs
- `missed`
- BLE `max send`, `failed` and `refused`
- any `GNSS: epochs stopped - restarting` lines
- the `Reset:` line after the run

**Checked:** a stalled link does **not** trip the liveness restart. With each send blocking 100 ms, the drain still completes a PVT frame about every 150 ms, well inside the 2 s silence window. A weak link degrades the stream; it doesn't reboot the device. The soak should confirm that `epochs stopped` never appears.

`BleNrf::setBatteryPercent()` → `BLEBas::notify()` uses the same queue from the battery job (accepted under MIN-6). Include it in the soak's attribution: a `battery` job max near 100 ms means it blocked.

## P2 (P3): ESP32 flash headroom

The image fills 91% of the default 1.25 MB app partition. The firmware has no OTA and no SPIFFS use. Switching the FQBN to `esp32:esp32:esp32:PartitionScheme=huge_app` (3 MB app, no OTA) in `scripts/build.sh` triples the headroom at no runtime cost. Do it before the RaceBox command decoder lands, not after a build fails.

---

## Complexity and clean-up

### C1 (P3): Run state kept in two places

`PowerPolicy` tracks `running_`, `chargingOnly_` and `onEntersChargeOnly_`, while `RunStateMachine` owns `state_`. They agree by construction, and the tests prove the paths. But Fault is "running" to the policy only implicitly, and every new state will need its policy equivalent kept in step by hand.

The policy could instead take the machine's current state as an input (`update(state, …)`) and keep only the debouncer, the noise counter and `onEntersChargeOnly_`.

**Not urgent:** it's correct today, and the change touches tested code. Worth doing if another power state is ever added.

### C2 (P3): Dead or test-only API

- `Scheduler::job(i)` and `Scheduler::count()`: no callers. `Scheduler::stats(i)` is used only by tests.
- `PowerPolicy::running()` and `chargingOnly()`: used only by tests. Tests can observe the returned actions instead.
- Already in the tracker as optional: the `ensureVal8`/`ensureVal16` wrappers, and `RateTracker`'s counter-went-backwards check, which is redundant now that Diagnostics re-baselines on `C`.

Removing these is a few lines each and narrows what a reader has to understand.

### C3 (P3): `ResetReason.h` → `RunStateMachine.h`

`core/ResetReason.h` includes the full state machine (and through it `GnssFeed`, `PowerPolicy` and `StatusLed`) only for `RestartReason`, `kRestartReasonCount` and `restartReasonText()`. Moving those three into a small `core/RestartReason.h` that both include keeps the reset formatting independent of run-state logic. Cosmetic.

---

## Checked and found sound

So it's clear what was examined, not just what was found:

- **Watchdog coverage against every blocking boot step.** Longest unfed stretch: detection on a silent port, 100 ms + 3 × 1.1 s + 100 ms ≈ 3.5 s against 5 s, with a compile-time assert. The configure path, IMU bring-up, BLE init, the console's 3 s USB wait and `Console::flush()` all feed or stay well under.
- **`apply()` called from inside jobs** (Fault from `gnssJob`, power transitions from `powerJob`). `setEnabled()` during `Scheduler::tick()` is safe: later jobs in the same pass see the new enable flags.
- **App member order** against Diagnostics' references (`loops_`, `reset_`, `battery_` are constructed before `diagnostics_`). `reset_` is written before any Diagnostics use.
- **ESP32 `onStatus()`:** every call site in the 3.3.12 library is inside `notify()`, on the caller's task, so `lastNotifyStatus` has no cross-task race.
- **nRF `notify()` with notifications off** falls back to `write()` and returns false. It is correctly counted as failed and doesn't block.
- **`HardwareSerial::end()` on ESP32** is idempotent (`_uart == NULL` guard), so Fault after a failed sweep is safe. The nRF side is guarded by `uartOpen`.
- **Millisecond and microsecond arithmetic** is wrap-safe everywhere it's compared (`Debounced`, `StatusLed`, `BatteryMonitor`, `BleLink`, `GnssFeed`, `GnssSpeedGate`).
- **Every formatter is bounded** (`Out` never overruns). N2 and N3 are about truncation, not overflow.
- **`ConsoleEntry`** handles signed `char` input, overflow-by-length and non-printables correctly.
- **Host tests:** all 22 suites pass at `-std=gnu++11`, plain and under ASan/UBSan.

---

## Recommended order

1. **N1:** a small policy change plus one test. It's the only finding that leaves the device in a wrong state.
2. **N2 and N3:** buffer sizes and comments, plus `unknown` on the status line. Together they're a few minutes' work.
3. **The soak, with the P1 knobs.** This decides PERF-1.
4. **P2:** ESP32 partition scheme.
5. **N4, N5, C1–C3:** as clean-up.
