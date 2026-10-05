# Response to the 2026-10-03 Firmware Review

Tracks what is being done about each finding in [firmware-review-2026-10-03.md](firmware-review-2026-10-03.md): the decision, the reasoning, and, once it is built, what changed and how it was verified. Finding IDs match the review.

**Bench status (2026-10-04):** BUG-1, BUG-3, REL-5 (LED), MIN-6, the console item, CPLX-4, MIN-1, MIN-2 and MIN-3 have compiled and passed bench checks (both boards; MIN-3 is nRF only). HW-1 was declined (comments only).

**Process:** each finding is taken one at a time. The finding and a proposed fix are described, the owner confirms or adjusts it, and only then is the code changed. Nothing here is committed until the owner commits it.

**Status values:**
- **Done (unverified):** the code has changed and host tests pass, but it has not yet been compiled or run on hardware.
- **Done:** additionally confirmed by a firmware build and a bench test.
- **Planned:** accepted, with the phase it belongs to.
- **Declined** or **Deferred:** with the reason.

---

## Corrections to the review

These claims were checked against the code, the installed library sources or bench measurements, and don't hold as written.

| Finding | Review claim | What was found |
|---|---|---|
| PERF-1 (console part) | A `D` report blocks the loop for about 15–20 ms on nRF while the host is reading | Measured on nRF with the host reading: the console job's worst run was **3.9 ms**, one overrun per `D`, and **0 missed epochs** over several reports. That is within the 5.5 ms UART ring headroom. |
| CLK-1 (accuracy) | TIMER4 runs from HFINT at about ±1.5% | On USB, a 512 s run's epoch count (20 Hz) and RT agree to within about 0.2%, so the clock is accurate there. USB most likely keeps HFXO running. The concern is still valid on battery, where it is unmeasured. |
| BUG-3 (ESP32 congestion) | ESP32 congestion is reported through `onStatus` callbacks | For notifications, arduino-esp32 3.3.12's `onStatus` covers only the checks before sending: no client, notifications disabled, and a failed hand-off to the stack. Congestion arrives later as a separate GATTS event (`ESP_GATTS_CONGEST_EVT`) and is not visible through `onStatus`. |
| MIN-6 (400 Hz writes) | `batteryJob` writes the Battery Service at 400 Hz | `batteryJob` calls `setBatteryPercent()` at 400 Hz, but `BleNrf::setBatteryPercent()` returns early unless the value changed, so the GATT write only happens on change. The missing *notify* is real. |
| MIN-4 | The `"max μs"` header misaligns | It only misaligns once a column grows past 6 digits (≥ 1 s). Real reports render aligned today. Still worth fixing. |

Confirmed as stated:
- `setAutoPVTcallbackPtr` defaults to `VAL_LAYER_RAM_BBR`.
- The ESP32 samples the link level every 10 ms to detect connects and disconnects.
- The Seeed `Wire_nRF52.cpp` has unbounded `while(!EVENTS_STOPPED)` and `while(!EVENTS_SUSPENDED)` loops.
- `BLEBas::write()` does not notify.
- The nRF core's `micros()` has 1 ms resolution, so the console's 2 ms wait is really 1–2 ms.
- `powerJob` relies on `[[noreturn]]` instead of `break`.

---

## Owner decisions

| Topic | Decision |
|---|---|
| PWR-1 hardware | On battery, switch OFF means the board is fully unpowered. A switch-OFF reading without USB can only be noise. |
| REL-5, GnssFault on battery | **Blink rapid red (10 Hz) indefinitely.** Drop the 30 s window that let the LED go dark. Do not enter System OFF and do not retry. A visibly faulted device prompts the user to power-cycle it, and the low-voltage cutoff still protects the cell. |
| REL-2, GNSS liveness | In RUNNING, **no NAV-PVT epoch for 2 s → log and restart.** Boot re-runs the full config, and on nRF boot also power-cycles the GNSS rail. A receiver that is really gone then lands in GnssFault. |
| BUG-1 approach | Hand re-advertising to the library (`advertiseOnDisconnect(true)`), replacing our delayed re-advertise. |
| BUG-3 | `C` resets `max send`, as it resets the job table's max μs. The other BLE counters are not cleared. |
| Console input | Line-based entries, each ended by Enter. Count capped at `kStatusMaxCount` (3600), with the input limits derived from it. Values above the cap are rejected, not clamped. Excess input is discarded. |
| D report wording | The BLE and IMU lines use words instead of 0/1 (done in BUG-3). The GNSS and Battery lines are to get the same treatment later, as a separate effort. |
| Status line | Keep the trim 🔒/⏳ icons. |

---

## Work completed

### BUG-1 (P0): ESP32 can stop advertising permanently. Done

**Finding confirmed:** `BleEsp32::update()` detected disconnects by sampling `linkUp` every 10 ms. A connect followed by a disconnect inside one pass was never seen, so advertising, which Bluedroid stops when a client connects, was never restarted.

**Background on the old 500 ms delay:** it was inherited from Classic, which turned the arduino-esp32 example sketches' blocking `delay(500)` before restarting advertising into a non-blocking deferral. Neither Classic nor NG documents a failure that required it. The library's own `advertiseOnDisconnect` path restarts immediately inside its `ESP_GATTS_DISCONNECT_EVT` handler (`BLEServer.cpp:586`).

**Fix:**
- `begin()` calls `server->advertiseOnDisconnect(true)`.
- `end()` calls `advertiseOnDisconnect(false)` *before* it disconnects the client, so stopping BLE (BATTERY_WAIT, GnssFault) doesn't restart advertising.
- `update()` now only logs the connect and disconnect edges. The pending flag, timer and retry branch are removed, along with `kBleReadvertiseDelayMs`.

**Trade-off accepted:** the library ignores a failed `start()`, so the old retry-on-failure is gone. A failed re-advertise has never been observed. A connection shorter than one 10 ms pass goes unlogged, as on nRF, but no longer affects advertising.

**nRF:** unchanged. Bluefruit's `restartOnDisconnect(true)` already covers it; only the log lines can be missed.

**Files:** `peripherals/ble/BleEsp32.cpp`, `peripherals/ble/BleEsp32.h`, `app/Config.h`.

**To verify:** on ESP32, connect and disconnect three or four times. Advertising should resume immediately each time.

### BUG-3 (P1): BLE send counters can't be trusted. Done

**Finding confirmed:**
- ESP32 counted every `notify()` as sent. `notify()` returns `void`, and frames the library skips (notifications not enabled, hand-off failed) were still counted.
- nRF counted successes correctly but discarded failures: notifications not enabled, the 100 ms `BLE_GENERIC_TIMEOUT` waiting for a send-queue slot, and stack errors.
- Neither board counted MTU refusals or measured how long a send blocks.

**Fix:**
- New `core/BleStats.h` with `sent`, `failed`, `refused`, `maxSendUs`, `rxBytes` and `rxDropped`. Each transport (ESP32, nRF, stub) returns it from `stats()` and has `clearMaxSend()`. `ReportView`'s loose BLE fields are replaced by one `BleStats`.
- ESP32: a `TxCallbacks::onStatus` records the status that `notify()` reports synchronously on the loop task. Only `SUCCESS_NOTIFY` counts as sent; anything else counts as failed. Congestion is not captured (see Corrections).
- nRF: a `false` from `notify()` counts as failed.
- Both boards count MTU refusals as refused (the existing log line stays) and time each send with `PlatformClock::nowUs()`, keeping the maximum.
- `C` (`App::clearStats`) calls `ble_.clearMaxSend()`.
- The status line's BLE rate reads `stats().sent`, which is now accurate on both boards.
- D report: `BLE: connected|disconnected, sent N, failed N, refused N, max send Nμs, rx N bytes, rx dropped N`.
- The IMU lines were also reworded: `IMU: up|down, Accel …` and `IMU: trim locked|settling, tilt …`.

**Verified 2026-10-04:** compiled and bench-tested on ESP32 and nRF; all checks passed. Field note: at connect, `refused` (≈5) and `failed` (≈17) rise and then freeze. That is about 250 ms before the client raises the MTU above the 23-byte default, then about 850 ms before it enables notifications (ESP32 reports these as `ERROR_NOTIFY_DISABLED`). On a healthy link neither counter moves while streaming. A count that keeps rising while connected is the signal to watch in the soak.

**Files:** `core/BleStats.h` (new), `core/Report.h`, `peripherals/ble/BleEsp32.{h,cpp}`, `peripherals/ble/BleNrf.{h,cpp}`, `peripherals/ble/BleStub.h`, `app/App.cpp`, `tests/test_report.cpp`.

**Tests:** `test_report.cpp` covers the new BLE line (connected and disconnected) and the IMU wording.

**To verify, both boards:**
- Connect and stream: `sent` climbs, `failed` and `refused` stay at 0, and `max send` is small.
- `C` resets `max send`.
- Connecting without subscribing to notifications should raise `failed`, not `sent`.

### REL-5 (P1, LED part): GnssFault goes dark after 30 s. Done

**Finding:** in GnssFault, `StatusLed` blinked red at 10 Hz only for the first 30 s (`kLedFaultMs`), then turned the LED off. That was meant to keep a battery build quiet. A dark device with the switch on looks off, even though it stays awake until the low-voltage cutoff.

**Fix (owner's decision, in place of the review's retry or System OFF suggestion):**
- The Fault pattern blinks at `faultHalfMs` (10 Hz) for as long as the state lasts.
- `faultMs` is removed from `LedTiming`, and `kLedFaultMs` from `Config.h`.
- The ESP32 shows the same pattern in blue (single-colour LED).

**Accepted side effect:** on battery, the LED's 50% duty adds a few mA, so an unattended faulted device reaches the cutoff somewhat sooner. The cutoff still shuts it off safely.

**Files:** `core/StatusLed.h`, `app/Config.h`, `app/App.cpp`, `tests/test_statusled.cpp`.

**Tests:** `testFaultBlinksRedIndefinitely` replaces `testFaultBlinksRedThirtySecondsThenOff`. It checks the phase starts at fault entry, the blink toggles every 50 ms, and it is still blinking at 30 s and at 10 minutes.

**To verify:** boot with the GNSS unplugged and confirm it is still blinking after a minute, red on nRF and blue on ESP32.

### MIN-6 (P3): Battery Service never notifies. Done

**Finding:** `BleNrf::setBatteryPercent()` used `BLEBas::write()`, which only stores the value, so a client subscribed to Battery Level (0x2A19) never saw changes. (The review's 400 Hz claim was wrong; see Corrections.)

**Fix:** call `BLEBas::notify(percent)` instead. Per the Bluefruit source, `BLECharacteristic::notify()` sends a notification when the client has enabled them (`sd_ble_gatts_hvx` also updates the stored value) and falls back to `write()` otherwise, so reads stay current. It is still called only when the percentage changes.

**Accepted:**
- The notification shares the send queue with telemetry, so it can block for up to 100 ms when that queue is full. That only happens on a percentage change during a stalled link, and the Phase 4 soak decides on the send path as a whole.
- No hysteresis (owner's decision). An EMA sitting on a boundary may notify a few times a minute.

**Files:** `peripherals/ble/BleNrf.cpp`, `peripherals/ble/BleNrf.h`.

**To verify:** in nRF Connect or LightBlue, subscribe to Battery Level (0x2A19). Change the level (plug or unplug USB, or let it drift) and confirm notifications arrive.

### PERF-2 (part), MIN-5, CPLX-5 (part): console input and wait timing. Done

**Findings confirmed:**
- `Console::emit()` timed its bounded output wait with `micros()`, which on nRF advances only once per 1 ms RTOS tick, so the "2 ms" wait was really 1–2 ms.
- `Console::poll()` read with an unbounded `while (Serial.available())`.
- Input was handled byte by byte with no notion of an entry:
  - `Love` ran `L`
  - `DVL` ran `L`
  - `1g6`+Enter armed 6 status lines, because `g` abandoned the 1
  - a digit past the 9-digit buffer was silently dropped

**Fix (owner's decisions: line-based entries, a count cap, reject anything above the cap):**
- `emit()` times its wait with `PlatformClock::nowUs()`, so it is exactly `kConsoleWaitUs` on both boards.
- New pure `core/ConsoleEntry.h` (template on the cap):
  - Collects bytes into entries ended by CR or LF.
  - A valid entry is exactly one of `D`, `C`, `V` or `L`, or a number 0..`kStatusMaxCount`.
  - A number over the cap, including one with too many digits, is *too large*. Anything else is *unrecognised* and rejected whole.
  - Empty entries (the LF of CR+LF) are ignored.
  - The buffer is sized from the cap's digit count (`decimalDigits`), with a `static_assert` of at most 9 digits so parsing can't overflow.
- `config::kStatusMaxCount = 3600` (an hour of 1 s lines). Changing it resizes the entry buffer and the per-pass read limit automatically.
- `Console::poll()`:
  - Reads at most `kCapacity + 2` bytes per pass (6 for 3600).
  - If more are waiting, discards all input at once, abandons the entry and logs `Console: input too long, discarded`.
  - Logs `Console: count must be 0-3600` and `Console: unrecognised input '…'` (bounded echo, non-printables as `?`, `…` when truncated).
- New `consoleDiscardInput()` platform function:
  - nRF: TinyUSB `tud_cdc_read_flush()` (via `<Adafruit_TinyUSB.h>` and `<tusb.h>`).
  - ESP32: ESP-IDF `uart_flush_input(UART_NUM_0)`. Not `Serial.flush(false)`, which spins until TX is idle.
- The boot help line says commands end with Enter and gives the count range.

**Accepted trade-off:** command letters now need Enter. A terminal that sends each keystroke no longer runs `D` without Enter, and the Arduino Serial Monitor needs a line-ending setting other than "No line ending".

**Files:** `core/ConsoleEntry.h` (new), `tests/test_consoleentry.cpp` (new), `app/Console.{h,cpp}`, `app/Config.h`, `app/App.cpp`, `platform/ConsolePort.h`, `platform/nrf52840/ConsolePort.cpp`, `platform/esp32/ConsolePort.cpp`.

**Tests:** `test_consoleentry.cpp` covers:
- commands with CR, LF and CR+LF
- counts 0, 3600 and 007
- too large: 3601 and 12 digits
- unrecognised: `Love`, `DVL`, `1g6`, `1g6t`, lowercase, leading space
- echo truncation, non-printables, empty entries and `reset()`
- the capacity tracking the cap (300 → 3, 3600 → 4, 99999 → 5)

**To verify, either board:**
- `D⏎`, `L⏎` and `5⏎` work.
- `Love⏎` and `1g6⏎` give the unrecognised message, and `3601⏎` gives the range message.
- Pasting a long block logs the discard, with no new missed epochs.
- First compile check on nRF: `<tusb.h>` resolves once `<Adafruit_TinyUSB.h>` is included.

### CPLX-4 (P3): speed-gate staleness counted in IMU ticks. Done

**Finding confirmed:**
- `GnssSpeedGate` judged staleness by counting `tick()` calls from `imuJob`. `kImuTrimPvtStaleMs` (1 s) became `kImuTrimPvtStaleTicks` = 100.
- Skipped IMU runs stretched the real window, and the conversion depended on the IMU period.
- `imuJob` had to call `tick()` before `poll()`.
- The practical effect was tiny (a few skips per run). The issue was a second, indirect notion of time.

**Fix:**
- `feed()` records the epoch's own `arrivalUs` (platform clock, stamped at parse).
- `valid(nowUs)` is true while `nowUs - lastArrivalUs < staleAfterUs`.
- `tick()` and the tick counter are removed. The constructor takes `kImuTrimPvtStaleMs * 1000`.
- `kImuTrimPvtStaleTicks` and its assert are removed from `Config.h`.
- `imuJob` passes `valid(PlatformClock::nowUs())`.

**Behaviour change:** none in normal running. During a scheduler stall the gate now goes stale after a true 1 s.

**Files:** `core/GnssSpeedGate.h`, `app/Config.h`, `app/App.cpp`, `tests/test_gnssspeedgate.cpp`.

**Tests:**
- Valid at +999,999 µs, stale at +1,000,000 µs.
- A fresh epoch restarts the window.
- New: staleness is independent of how often `valid()` is called.
- Existing fix-type and `gnssFixOK` checks kept.

**Verified 2026-10-04:** both boards build, boot and reach `IMU: trim locked` while stationary.

### MIN-1 (P3): INT32_MIN negation in the status line. Done

**Finding confirmed:** `putDegrees1e7()` in `core/StatusLine.h` took the magnitude with `neg ? -v1e7 : v1e7`. That is signed overflow, which is undefined behaviour, when `v1e7 == INT32_MIN`. It's the only signed negation of its kind in `core/`. It's unreachable from real data, since ±180° is ±1.8e9 and INT32_MIN is −214.7483648°.

**Fix (owner chose the minimal form):**
- The magnitude is computed in unsigned arithmetic, `0u - static_cast<uint32_t>(v1e7)`, with a one-line comment.
- Output is unchanged for every other value.

**Files:** `core/StatusLine.h`, `tests/test_statusline.cpp`.

**Tests:** `testWithFix` adds `latDeg1e7 = INT32_MIN` → `Lat: -214.7483648`. That pins the output and gives the TEST-2 sanitizer pass a path to check.

**To verify:** none on hardware (status line unchanged).

### MIN-2 (P3): unguarded division by job period. Done

**Finding confirmed:**
- `Scheduler::tick()` divides by `periodUs` to count skipped periods.
- Only two of the seven job periods were asserted positive (GNSS drain, BLE).
- IMU, battery, power, LED and console were not. Two of those sat in other asserts that a 0 would still pass, e.g. `kLedPeriodUs / 1000 <= kLedBatteryWaitHalfMs`.
- A zero period was therefore only an edit mistake away from a crash at the first tick.

**Fix (owner's choice: one combined compile-time assert now, automatic check in the restructure):**
- One `static_assert` in `Config.h` covers all seven job periods. It replaces the two individual job-period asserts, with a comment that a new job's period belongs there too.
- `kStatusPeriodUs` keeps its own assert (not a job).
- `Scheduler.h`'s class comment states the precondition.

**Rejected alternatives:**
- A runtime assert in the `Scheduler` constructor. On these boards a failed `assert()` halts or reboots without a useful message, so the build failing is better.
- Treating 0 as "every tick". The loop would never sleep, swapping a crash for a pegged CPU.

**Files:** `app/Config.h`, `core/Scheduler.h`.

**Tests:** compile-time only. `Config.h` syntax-checked on the host. Host suites pass.

### MIN-3 (P3): LSM6DS3 configuration not verified. Done

**Finding confirmed, and wider than the review stated:**
- The `CTRL3_C` (BDU) write result was ignored.
- The Seeed library's `begin()` doesn't check *any* of its register writes.
- `begin()`'s range `switch` falls back to 16 g / 2000 °/s for an unsupported value, while `calcAccel()`/`calcGyro()` still scale by the range requested. An unsupported Config range, or a write that didn't land, would therefore silently mis-scale every sample.
- The boot log said `detected and configured ✅` before BDU or the I2C clock had been set.

**Fix (owner approved, gravity-plausibility warning skipped):**
- **Compile-time expected bytes** built from the library's own field constants:
  - `CTRL1_XL = ODR_XL_104Hz | FS_XL(range) | BW_XL_50Hz`
  - `CTRL2_G = ODR_G_104Hz | FS_G(range)`
  - `CTRL3_C = BDU | IF_INC`
  - These are `4B 44 44` for 4 g / 500 °/s, host-checked against the library header.
- **Build-time range checks:** `static_assert`s make an unsupported accelerometer or gyro range (or a changed ODR/bandwidth constant) fail the build.
- **Boot sequence:**
  1. `begin()` logs `IMU: LSM6DS3TR-C detected` (it fails only on a WHO_AM_I mismatch).
  2. Set the I2C clock, so verification runs at the real bus speed.
  3. Write `CTRL3_C`, then read back 0x10–0x12 in one burst and compare.
  4. Read the filter-seed sample.
  5. `IMU: configured, 4g 500deg/s ✅` only if all of that succeeds. Otherwise ❌ with the reason: register access failed; `read XX XX XX, expected XX XX XX`; or first sample read failed.
  6. On failure, continue without the IMU, as the MPU-6050 path does.

**Side effect:** `kImuGyroRangeDps` must now suit both chips (500, 1000 or 2000). The MPU's 250 and the LSM's 245 no longer overlap, so 250 now fails the nRF build instead of becoming 2000 °/s.

**Deferred to Phase 5:** retry on failure (with REL-6 re-init), and a shared write/verify/log helper for both IMU drivers (CPLX-2).

**Files:** `peripherals/imu/Lsm6ds3.cpp`.

**Verified 2026-10-04:** nRF boot shows `IMU: LSM6DS3TR-C detected ✅` and `IMU: configured, 4g 500deg/s ✅`, so the readback matched `4B 44 44` on hardware.

### MIN-4 (P3): table header width counted in bytes. Done (unverified)

**Finding confirmed:**
- `Out::left()`/`right()` in `core/TextOut.h` padded text cells by `strlen()`. `μ` is 2 bytes, so `"max μs"` and `"mean μs"` measured one byte too wide.
- That was invisible at the minimum column widths. Once a max reached 7 digits (≥ 1 s) or a mean 8 digits, the header got one space too few, and every `|` after it shifted left.
- The alignment test had excluded the header row to work around it.

**Fix (owner's choice: the general fix plus the test):**
- New `detail::displayWidth()` counts characters by skipping UTF-8 continuation bytes.
- The text overloads of `left()`/`right()` pad by it. Numbers are unchanged.
- The helper's comment notes that wide emoji (drawn two columns wide) don't belong in padded cells.

**Rejected alternatives:**
- ASCII `us`: inconsistent with `μs` elsewhere in the report.
- A one-byte width bump for the two μ headers: hard-codes the label contents.
- Declining: cosmetic, but the test exception was hiding it.

**Files:** `core/TextOut.h`, `tests/test_report.cpp`.

**Tests:**
- `testTableStaysAligned` now compares display widths, including the header row.
- It runs a second time with `maxUs` = 1,234,567 and mean 12,345,678, so both μ headers need padding.
- Confirmed to fail with the old `strlen()` padding and pass with the fix.

**To verify:** none on hardware. The table renders the same until a job takes ≥ 1 s.

### powerJob fall-through (P3, from MIN section). Done (unverified)

**Finding confirmed:**
- `App::powerJob` had no `break` after `restart()` (×2) or `enterSystemOff()`. It relied on their `[[noreturn]]` to avoid falling through.
- The review missed one more instance: `begin()`'s boot switch, where `enterSystemOff();` fell into `default: break;`.
- Correct today, but a future returning path would fall through, e.g. `ResetForSwitchOn` into `restart("USB removed")`.

**Fix:** an explicit `break;` after each of the four calls, with a one-line comment in `powerJob`. No behaviour change. `begin()`'s `default:` stays, since `policy_.begin()` returns only a subset of actions at boot, and the restructure's state machine will replace both switches.

**Files:** `app/App.cpp`.

**To verify:** both boards compile.

### TEST-2 (P3): no sanitizers in host tests. Done (unverified in CI)

**Finding confirmed:** `tests/run.sh` built each test once at `-O1` without sanitizers.

**Trial before the change (scratch builds):**
- All 17 suites passed clean under ASan and UBSan with `-fno-sanitize-recover=all` (Apple clang 21).
- With MIN-1's old negation reinstated in a scratch copy, the plain build still passed, but the sanitized build failed: `negation of -2147483648 cannot be represented in type 'int32_t'`.
- Cost: about 2.8 s plain plus about 6 s sanitized.

**Fix:**
- `run.sh` builds and runs every test twice.
  - **Plain:** unchanged flags, kept as the closest match to firmware codegen.
  - **Sanitized:** plus `-g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer`, written to a separate `-san` binary.
- The first sanitizer finding fails the run.
- CI's `host-tests` job already runs `tests/run.sh`, so it picks up the sanitized pass automatically.

**Files:** `tests/run.sh`.

**To verify:** local run passes (exit 0). The first CI run on the Ubuntu runner (gcc, where ASan also enables leak checking) should pass too.

---

## Remaining plan

### Phase 1: small, low-risk fixes (complete; MIN-4, powerJob and TEST-2 awaiting build/CI confirmation)

| Finding | Status | Planned fix |
|---|---|---|
| BUG-1 | Done | See above |
| BUG-3 | Done | See above |
| REL-5 LED | Done | See above |
| MIN-6 | Done | See above |
| HW-1 | Declined | See Declined (comment rewritten) |
| PERF-2 (part) / MIN-5 / CPLX-5 (part) | Done | See above (now line-based entries with a count cap) |
| CPLX-4 | Done | See above |
| MIN-1 | Done | See above |
| MIN-2 | Done | See above |
| MIN-3 | Done | See above (now verifies all three control registers) |
| MIN-4 | Done (unverified) | See above |
| powerJob | Done (unverified) | See above |
| TEST-2 | Done (unverified in CI) | See above |

### Between Phases 1 and 2: restructure App.cpp (proposed, to be discussed in detail)

**Design write-up for review:** [app-restructure-design.md](app-restructure-design.md). It covers the overall shape, interfaces, transition and job tables, steps R1–R6, and open questions.

Raised by the owner. It relates to TEST-1, which notes the untested glue in `App` is where most P0/P1 findings live.

| Item | Status | Proposal |
|---|---|---|
| Run-state machine | Proposed | Extract run-state decisions from `App.cpp` into a pure, host-tested `core/RunStateMachine.h`, using the same "decide in core, apply in App" seam as `core/PowerPolicy.h`. It takes events (GNSS detected or missing, PVT enable failed, PVT silent 2 s, BLE stack failed, and PowerPolicy's switch, USB and cutoff actions). It returns the new state plus ordered actions (start peripherals, stop peripherals, restart, System OFF). The state → LED pattern mapping becomes a pure function alongside it. `App` keeps only the hardware calls that carry out the actions. Done before Phase 2, so Phase 2's new triggers (REL-2 liveness, REL-4, REL-7, PWR-1) land in tested code instead of being added to `App.cpp` and moved later. |
| Slim `App.cpp` | Proposed | Move job bodies and related state into a few modules by concern, e.g. telemetry (epoch bookkeeping, protocol encode and send) and diagnostics (console commands, `D` report assembly, status line). `App` would keep construction, the job table, `begin()`, applying state-machine actions, and `tick()`. |

Additional parts of the same restructure (all five accepted by the owner for the proposal):

| Item | Status | Proposal |
|---|---|---|
| Per-state job set | Proposed | One table saying which jobs run in each run state, applied as a single state-machine action. It replaces the scattered `setEnabled` calls in `stopPeripherals()`, `begin()` (ESP32 battery and power) and the Running branch (IMU off for protocols without IMU). |
| Board capabilities at compile time | Proposed | `board::kHasBattery` and `board::kHasSwitchedPower` constants in each `BoardConfig`, replacing the runtime `haveBattery_` and `switchedPower_` flags, so boards without them leave that code and those jobs out at compile time. The state machine knows which states each board can reach. |
| One device-status snapshot | Proposed | Gather device status once (GNSS, IMU, BLE, battery, USB, with unit conversions) into a struct with nested parts. Both the `D` report and the status line are formatted from it. Removes the duplicated assembly in `consoleJob` and `printStatus`, and replaces `ReportView`'s long positional list of initialisers, which every added field currently forces callers and tests to update. |
| Member-function jobs | Proposed | A small template wrapper in the scheduler, so jobs are ordinary member functions instead of `static void xJob(void* self)` that cast and use `app->` throughout. Readability only. |
| Protocol module | Proposed | One module per protocol owning its BLE profile, packet encoding (epoch + IMU + battery) and inbound command handling. Replaces the `config::kProtocol` switches in `activeProfile()` and `sendTelemetry()`, and the RX drain in `bleJob`. Home of the future RaceBox command decoder and calibration, and of the SEC-3 input-handling rules. It subsumes the "telemetry" module named above. |
| Job table checked at compile time | Proposed | With member-function jobs, make the job table a compile-time constant so one check in the scheduler covers every entry's period (including future jobs). That replaces MIN-2's hand-maintained list in `Config.h`. |

Considered and left out: templating every module on its hardware types for host testing (the state machine is where tests pay off), and splitting `Config.h` by module (reasonable, separate effort).

### Phase 2: survive faults instead of hanging

| Finding | Planned fix |
|---|---|
| REL-1 | Hardware watchdog on both boards, fed from the loop; reset reason printed at boot. Also converts REL-3's nRF I2C hang into a reboot. |
| REL-2 | Add `UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1` to the flash-ensured list and register the PVT callback with RAM only. 2 s liveness check → log and restart. |
| REL-4 | If enabling PVT fails, go to GnssFault. The liveness check catches any other "configured but silent" case. |
| REL-7 | If `Bluefruit.begin()` fails, go to the fault state, not a blinking "advertising" LED |
| PWR-1 | Accept switch-off only while USB is present; debounce switch-on out of BATTERY_WAIT |
| CLK-1 | `sd_clock_hfclk_request()` once the SoftDevice is up (battery accuracy); document `nowUs()` as loop-only (nothing outside the loop calls it today) |

### Phase 3: GNSS configuration verification

| Finding | Planned fix |
|---|---|
| BUG-2 | Also read the RAM (effective) layer. Write all layers if RAM or flash differs. Keep the per-key ✅ log. |
| CPLX-6 | Deferred. Batch the transactions only if the extra reads make boot noticeably slower. |

### Phase 4: measure, then fix the structure

| Finding | Planned fix |
|---|---|
| PERF-1 (scheduler) | After every job run, rescan from the GNSS job (index 0), so the time between drains is bounded by one job's worst case |
| Soak / HIL | Weak-link BLE soak on nRF: phone at range or shielded, `D` pressed periodically; check `missed`, `skipped` and the new BLE `failed` / `max send` counters |
| PERF-1 (BLE and UART) | Decided by the soak result: a deeper HVN queue, a separate BLE sender task, or EasyDMA UART ingestion |

### Phase 5: IMU robustness

| Finding | Planned fix |
|---|---|
| REL-6 | Re-initialise the IMU with backoff after it is marked down; recovery counter in `D` |
| CPLX-2 | Merge the MPU-6050 and LSM6DS3 pipelines (chip driver plus shared pipeline) while doing REL-6 |
| REL-3 | ESP32 `Wire.setTimeOut()` short. nRF bounded TWIM driver deferred; the REL-1 watchdog covers the hang. |

### Later, as a separate effort

- D report GNSS lines: fix type as words (e.g. `3D fix`); `no epoch yet` instead of `fix none` before the first epoch.
- D report Battery line: 0/1 flags as words, or list only the true ones.

---

## Open checks

Hardware checks that decisions above depend on.

| Check | Why it matters |
|---|---|
| **System OFF test** (temporarily raise `kBatteryCutoffV` and the warning thresholds): confirm entry into System OFF, the low current, and that **plugging in USB wakes the nRF** into a normal boot | HW-1's decline relies on USB wake. If USB does *not* wake it, a charger could raise the cell towards 4.2 V with P0.14 still HIGH. Then HW-1 flips to "leave `VBAT_ENABLE` LOW in System OFF" (~2.3 µA). |

---

## Declined or deferred

| Finding | Decision | Reason |
|---|---|---|
| PERF-1 (console) / PERF-2 "spread `D` across ticks" | Declined for now | Measured at 3.9 ms with 0 missed epochs (see Corrections). Revisit if the soak shows misses. |
| PERF-2 "latch a stalled host" | Declined | The owner chose the simplest console writer (each line sent whole or dropped after a bounded wait); the latch was rejected as unnecessary. |
| PERF-3 | Declined | BATTERY_WAIT and CHARGE_ONLY both run on USB power. While RUNNING, the loop already wakes at 1 kHz for the GNSS drain. |
| PERF-4 | Deferred | ESP32 is the secondary board. Same-size allocations; a long soak would show fragmentation. |
| BAT-1 | Declined | Peak-hold is deliberate, to reject dips under radio and GNSS load. With the switch ON on battery, the only time the cutoff matters, the switch-sense pin sits near 0 V, so ADC carryover biases VBAT readings *down*, and peak-hold discards those. |
| HW-1 | Declined (comment rewritten) | Seeed's wiki says don't drive P0.14 HIGH *while charging*. The divider's 510 kΩ leg returns to P0.14, so HIGH puts P0.31 at about 3.3 V + (VBAT − 3.3 V)/3, which is 3.60 V with a 4.2 V cell. The same page recommends HIGH after reading to save the divider's ~2.3 µA in sleep. The code holds P0.14 LOW whenever it's running or charging, and drives it HIGH only on System OFF entry, below the 3.30 V cutoff, where P0.31 stays around 3.3–3.4 V. USB plug-in should wake the chip into a boot that drives it LOW before the cell can charge, so this rests on USB wake from System OFF (see Open checks). The review's alternative (P0.31 as a disconnected input) wouldn't change the pin voltage. Sources: [Seeed XIAO nRF52840 wiki](https://wiki.seeedstudio.com/XIAO_BLE/), [quote of the wiki note](https://github.com/honvl/Seeed-Xiao-NRF52840-Battery/issues/1). Files: `platform/BatteryPort.h`, `platform/nrf52840/BatteryPort.cpp` (comments only). |
| DATA-1 | Deferred | A real data-quality question (EMA lag, IMU/epoch alignment), but it changes what is sent. To be discussed separately after these phases. |
| SEC-1, SEC-2 | Declined | Outside the stated threat model. The owner previously decided to refuse oversized frames and not request a larger MTU. |
| SEC-3 | Deferred | Input-handling rules will be written when the RaceBox command decoder is built. No code today. |
| SEC-4 | Accepted risk | Already accepted. |
| CPLX-1 | Declined | The SparkFun callback is a plain function pointer, so the state must be static. There is one receiver by design. |
| CPLX-3 | Deferred | Revisit when SEC-1-style policy or the command decoder lands |
| CPLX-7, TEST-1, TEST-3 | Deferred | Later cleanup |
