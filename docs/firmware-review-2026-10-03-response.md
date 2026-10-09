# Response to the 2026-10-03 Firmware Review

Tracks what is being done about each finding in [firmware-review-2026-10-03.md](firmware-review-2026-10-03.md): the decision, the reasoning, and, once it is built, what changed and how it was verified. Finding IDs match the review.

**Bench status (2026-10-04):** BUG-1, BUG-3, REL-5 (LED), MIN-6, the console item, CPLX-4, MIN-1, MIN-2, MIN-3, MIN-4 and the powerJob breaks have compiled and passed bench checks (both boards; MIN-3 is nRF only). TEST-2 passes in CI. **Phase 1 complete.** HW-1 was declined (comments only).

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
| REL-6 (zeros) | A down IMU reports zeros, "which a consumer can't tell apart from level and still" | (Owner, 2026-10-09.) A healthy accelerometer always measures gravity: level and still reads about 1000 mG on Z. An all-zero accel vector can't come from a working IMU, so a client *can* tell a failed IMU from real data. The MPU-6050 driver already treats exactly that reading as a failure (its power-on state after a supply glitch). |
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
| REL-5, GnssFault on battery | **Blink rapid red indefinitely** (10 Hz at first; 5 Hz since 2026-10-08, owner). Drop the 30 s window that let the LED go dark. Do not enter System OFF and do not retry. A visibly faulted device prompts the user to power-cycle it, and the low-voltage cutoff still protects the cell. |
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
- The Fault pattern blinks at `faultHalfMs` (10 Hz then; 5 Hz since 2026-10-08, see REL-7) for as long as the state lasts.
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

### MIN-4 (P3): table header width counted in bytes. Done

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

### powerJob fall-through (P3, from MIN section). Done

**Finding confirmed:**
- `App::powerJob` had no `break` after `restart()` (×2) or `enterSystemOff()`. It relied on their `[[noreturn]]` to avoid falling through.
- The review missed one more instance: `begin()`'s boot switch, where `enterSystemOff();` fell into `default: break;`.
- Correct today, but a future returning path would fall through, e.g. `ResetForSwitchOn` into `restart("USB removed")`.

**Fix:** an explicit `break;` after each of the four calls, with a one-line comment in `powerJob`. No behaviour change. `begin()`'s `default:` stays, since `policy_.begin()` returns only a subset of actions at boot, and the restructure's state machine will replace both switches.

**Files:** `app/App.cpp`.

**Verified 2026-10-04:** both boards build, flash and boot.

### TEST-2 (P3): no sanitizers in host tests. Done

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

**Verified 2026-10-04:** local run passes, and CI's `host-tests` job passes on the Ubuntu runner (gcc, with ASan leak checking).

---

## Remaining plan

### Phase 1: small, low-risk fixes (complete, all verified)

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
| MIN-4 | Done | See above |
| powerJob | Done | See above |
| TEST-2 | Done | See above |

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

### Restructure progress

**Order changed (owner, 2026-10-05), after the back-fed GNSS finding:** R0–R3 done → **R6** (GnssFeed + protocol module) → **Phase 2's GNSS items** (REL-2 liveness with the restart-or-fault refinement, REL-4) → **R4, R5** (Diagnostics move, status snapshot) → the rest of Phase 2. R6 doesn't depend on R4/R5, and this fixes the user-visible "healthy but silent" failure sooner, with the liveness check written once in its final home.

#### R0: host tests build as C++11. Done

**Why:** the Seeed nRF core compiles with `-std=gnu++11`, but host tests used `-std=c++17`, so C++14-and-later code in `core/` would only fail in a firmware build.

**Change:**
- `tests/run.sh` uses `-std=gnu++11` for both passes.
- `test_report.cpp` sets `BleStats` fields by name. A struct with default member values isn't an aggregate in C++11, so the positional `{…}` was the one C++14-only construct found.

**Verified:** all 17 suites pass, plain and sanitized.

#### R1: member-function jobs and a compile-time job table. Done

**Change:**
- `core/Scheduler.h` adds:
  - `JobSpec<T>` (`{id, name, periodUs, budgetUs, void (T::*run)()}`)
  - `jobSpecsValid()`, a recursive `constexpr` check that every entry sits at its id with a name and a positive period
  - `MemberJobs<T, N>`, which builds the `Job[]` table and dispatches each run to its member function; non-copyable
- App's seven jobs are now ordinary member functions (bodies unchanged apart from dropping `app->`), declared in `App::kJobSpecs[kJobs]`, one line per job.
- The check runs as a `static_assert` in App's constructor: GCC rejects it in the class body, where App is incomplete.
- MIN-2's hand-maintained period assert in `Config.h` is removed, since the table check covers it.
- The boot log's job lines read from the same table.

**Toolchain findings:**
- Checked with the boards' own compilers (arm-none-eabi-g++ 9 and xtensa g++, `-std=gnu++11 -Wall -Wextra`).
- A well-formed table compiles; a swapped table and a zero-period table are rejected.
- GCC 9 can't compare a member-function pointer in a constant expression, so `run` isn't checked at compile time.

**Files:** `core/Scheduler.h`, `app/App.h`, `app/App.cpp`, `app/Config.h`, `tests/test_scheduler.cpp`.

**Tests:**
- `testMemberJobsDispatchToTheRightMember` covers names, periods and budgets carried into the built table, and per-member dispatch at different periods.
- `static_assert`s cover a good table and three bad tables: swapped, zero period, no name.

**Verified 2026-10-04:** both boards compile, flash, boot and run as before.

#### R2: board capabilities as compile-time constants. Done

**Change:**
- `board::kHasBattery` and `board::kHasSwitchedPower` in each `BoardConfig.h` (ESP32 false/false, nRF true/true).
- New `platform/Board.h` picks the board's `BoardConfig.h` using the existing guards: the ESP32 two-layer chip check and the four XIAO nRF52840 macros. It's an `#error` otherwise.
- App's runtime `haveBattery_` and `switchedPower_` members are removed. Their uses are ordinary `if (board::k…)` checks, C++11-safe.
- `batteryStatus()` moves to App.cpp.
- `PeripheralPower::holdAllOff()` and `BatteryPort::begin()` become `void`. Their return values were constant per board, and the headers now say "a no-op where the board lacks it".

**Behaviour change:** none.

**Checks:**
- `App.h` syntax-checked with arm-none-eabi-g++ 9 and xtensa g++ (`-std=gnu++11`).
- `Board.h` checked on xtensa against the ESP32's real `sdkconfig.h`, asserting both capabilities false.
- Host suites pass.

**Files:** `platform/Board.h` (new), `platform/esp32/BoardConfig.h`, `platform/nrf52840/BoardConfig.h`, `platform/PeripheralPower.h`, `platform/BatteryPort.h`, `platform/{esp32,nrf52840}/PeripheralPower.cpp`, `platform/{esp32,nrf52840}/BatteryPort.cpp`, `app/App.{h,cpp}`.

**Verified 2026-10-04:** both boards compile, flash and boot as expected.

#### R3: run-state machine. Done

**Owner's choices:** `Fault` with a reason (replacing `GnssFault`); the machine owns `PowerPolicy`; state and reason text lives in core.

**Change:**
- **New pure `core/RunStateMachine.h`:**
  - States `Starting, Running, Fault, BatteryWait, ChargeOnly`; `FaultReason::GnssMissing`; `RestartReason::SwitchOn/UsbRemoved`.
  - Events `boot()`, `gnssStarted()`, `power()`, each returning a `Transition` (Stay, Enter, Restart, SystemOff).
  - It owns `PowerPolicy`, which is updated on every `power()` call so its debounce keeps running. An event that doesn't apply returns Stay.
  - Also: `jobsFor()` (named per-job flags), `ledStateFor()`, and text lookups (`stateName`, `faultMessage`, `entryMessage`, `restartReasonText`).
- **App:**
  - `enterState()` (with its recursive call), `stateName()`, the private `RunState`, both `PowerAction` switches, `ledJob`'s mapping and the scattered `setEnabled()` calls are removed.
  - `apply(Transition)` loops: set the entered state's jobs, then run its entry effects. Only Starting produces a follow-up (the GNSS result), so the loop runs at most twice. Restart and SystemOff go to the existing `restart()` and `enterSystemOff()`.
  - `enter(RunState)` holds the fixed per-state entry effects:
    - Starting: rail on, log, `gnss_.begin()`
    - Running: IMU, then BLE
    - Fault: log, stop, rail-off log
    - BatteryWait / ChargeOnly: stop, then log
  - `stopPeripherals()` no longer touches jobs.
  - `powerJob` and `ledJob` are a few lines each. One `constexpr RunConfig kRunConfig` (board and protocol capabilities plus the policy timings) feeds both the machine and `jobsFor()`.

**Behaviour change:** none intended. Log lines, their order, jobs per state, LED patterns and power transitions are unchanged. Boot now passes through `Starting` internally, which isn't visible because the final boot line prints once the final state is reached.

**Checks:**
- Host suites pass, plain and sanitized, as C++11.
- `test_runstatemachine.cpp` also compiles with arm-none-eabi-g++ 9 and xtensa g++ (`-Werror`).
- `App.h` syntax-checks on both.
- `App.cpp` syntax-checks clean (`-Wall -Wextra`) against the full ESP32 core.

**Files:** `core/RunStateMachine.h` (new), `tests/test_runstatemachine.cpp` (new), `app/App.h`, `app/App.cpp`.

**Tests (12):**
- boot classification, battery and battery-less
- Running only via `gnssStarted(true)`, and a late "detected" after Fault stays
- `gnssStarted()` outside Starting stays
- switch-off on USB → BatteryWait (debounced)
- Fault can still wait or cut off
- cutoff only on battery
- BatteryWait switch-on → Restart(SwitchOn)
- the CHARGE_ONLY gesture and its exits (USB removed → Restart(UsbRemoved); off again → BatteryWait; then on → Restart(SwitchOn))
- no power transitions without a battery
- jobs per state (battery, battery-less, no-IMU protocol)
- LED priorities
- every text lookup

**Verified 2026-10-05:**
- ESP32 normal and GNSS-unplugged boots match pre-R3 output.
- nRF clean boot is identical.
- All nRF power transitions pass: BATTERY_WAIT, switch-on restart, CHARGE_ONLY, USB removal in CHARGE_ONLY.
- The GNSS-missing boot shows the fault line, `Power: GNSS rail off`, only battery, power, LED and console jobs running, and `GNSS fault - console only`.

#### R6: GnssFeed, the RaceBox protocol module and one definition of "charging". Done

**Owner's choices:** an `app/protocol/` subfolder; a shared `isCharging()` helper that uses the charger's CHG pin, guarded by USB presence.

**Change:**
- **New `core/GnssFeed.h`** (pure, host-tested), an improvement on the design doc's `app/` version.
  - Owns the epoch tracker, the latest epoch and the speed gate.
  - It's fed epochs (`add()`) rather than polling the receiver, so the Phase 2 liveness rule can live here with host tests.
  - Accessors `epochs()`, `latest()` (nullptr until the first epoch), `speedMps()`, `speedValid(nowUs)`, `clear()`.
- **New `app/protocol/RaceBox.h`** (pure):
  - `profile()` and `kUsesImu`.
  - `encode()`, which wraps `encodeRaceBoxData`. It absorbs App's battery-byte logic as `batteryByte()` and doesn't send.
  - `onRx()`, empty for now: the home of the future command decoder.
- **New `app/protocol/Protocol.h`:** picks the class from `config::kProtocol` by template specialisation. An unhandled protocol fails the build.
- **`isCharging(const BatteryStatus&, bool usbPresent)`** in `core/BatteryMonitor.h` is the one definition: the CHG pin says the charger is charging, and USB is present. It's used by the boot battery line, the status line's ⚡ and the RaceBox charging bit. The D report's `charging 0/1` stays the raw pin, by the owner's earlier choice.
- **App:**
  - `gnssJob` polls the receiver, calls `feed_.add()`, and if connected encodes with the protocol and sends.
  - `bleJob` passes reads to `protocol_.onRx()`.
  - `imuJob` reads speed from the feed.
  - The console code reads `feed_`.
  - `activeProfile()`, `sendTelemetry()`, `raceBoxBatteryByte()` and the four GNSS members are gone. `kRunConfig` takes `Protocol::kUsesImu`.
  - `App.cpp` is down to 342 lines.
- **`Config.h`:** `protocolUsesImu()` and `kProtocolUsesImu` removed.

**Behaviour change:** none intended. BLE packets, name, D report and status line are unchanged.

**Checks:**
- 19 host suites pass, plain and sanitized, as C++11.
- The new tests also compile with arm-none-eabi-g++ 9 and xtensa g++ (`-Werror`).
- `App.h` syntax-checks on both.
- `App.cpp` is clean against the full ESP32 core.

**Files:** `core/GnssFeed.h` (new), `app/protocol/RaceBox.h` (new), `app/protocol/Protocol.h` (new), `core/BatteryMonitor.h`, `core/StatusLine.h`, `app/App.{h,cpp}`, `app/Config.h`, `tests/test_gnssfeed.cpp` (new), `tests/test_raceboxprotocol.cpp` (new), `tests/test_batterymonitor.cpp`.

**Tests:**
- GnssFeed: empty before the first epoch; `add()` updates the count, latest epoch, speed and staleness; `clear()` resets only the statistics.
- RaceBox battery byte: no battery or no reading → 100; percent; charging bit only with the pin *and* USB; percent masked to 7 bits.
- The protocol's `encode()` is byte-identical to calling the encoder directly.
- `isCharging()` needs both the pin and USB.

**Verified 2026-10-05:** both boards compile, flash and stream to the app. On the nRF, every boot line is ✅, the status line shows ⚡ while charging, trim locks (🔒 at about 25 s), and `D` shows 0 missed epochs.

#### Phase 2, GNSS item 1 (REL-2): persistent PVT output and liveness. Done

**Finding confirmed:**
- NAV-PVT output was the one setting not ensured in flash. `setAutoPVTcallbackPtr()` defaulted to RAM+BBR, so a receiver reset without backup power came back with output off.
- Nothing watched epoch arrival.
- The back-feed finding added a third case: epochs that never start.

**Fix (owner approved after two refinement passes):**
- **Persisted:** `UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1 = 1` is ensured in flash, last in the list (`GNSS: NAV-PVT output rate=1 ✅`). A receiver reset now resumes output by itself. Where V_BCKP holds BBR this already worked; this covers the loss of backup power too.
- **Quiet configuration:** `configure()` first sets PVT output to 0 in **RAM only** (`GNSS: NAV-PVT output paused for configuration ✅`), so the config reads and writes still run on a quiet line even though flash now starts output at power-up. The callback is registered with `VAL_LAYER_RAM`, which turns it back on.
- **Liveness in `core/GnssFeed.h`:**
  - `liveness(nowUs)` returns `Ok`, `NeverStarted` (no epoch within `kGnssSilenceMs` of the first check) or `Stopped` (epochs, then none for `kGnssSilenceMs`).
  - The window is armed on the first check, i.e. the first GNSS job run, so boot time before the scheduler doesn't count.
  - One constant, `kGnssSilenceMs = 2000`, asserted to span a 1 Hz period and at least 10 nav periods.
- **State machine:** `gnssLiveness()`, effective in Running only.
  - Stopped → Restart (`GnssSilent`), logged `GNSS: epochs stopped - restarting into RUNNING`.
  - NeverStarted → Fault (`GnssNoData`), logged `GNSS detected but sending no data. Check its power supply. Everything else stopped.`
  - That's at most one restart per failure: a receiver still silent after the restart faults.
- **Restart log:** the line is now `<reason> - restarting into RUNNING`, with each reason carrying its prefix. The existing power lines are unchanged ("Power: switch on", "Power: USB removed").
- **App:** `gnssJob` checks liveness after each poll (two comparisons per run).

**nRF bench (2026-10-05):**
- A normal boot is good.
- A mid-session GNSS disconnect → one restart, then the GNSS-missing fault.
- A back-fed boot → Running, then about 2 s later the no-data fault with the peripherals stopped. The paused-output write also fails (❌), an early sign of an unhealthy receiver.
- The no-data message was reworded from "Nothing else will start" to "Everything else stopped", since the IMU and BLE had already started by then (owner's choice).
- ESP32: a normal boot with epochs flowing.

**Files:** `peripherals/gnss/Gnss.cpp`, `core/GnssFeed.h`, `core/RunStateMachine.h`, `app/Config.h`, `app/App.cpp`, `tests/test_gnssfeed.cpp`, `tests/test_runstatemachine.cpp`.

**Checks:**
- 19 host suites pass, plain and sanitized.
- The new tests compile with both board compilers (`-Werror`).
- `App.cpp` and `Gnss.cpp` are clean against the full ESP32 core and the SparkFun library.

**Tests:**
- GnssFeed: NeverStarted counts from the first check, not from boot; an epoch keeps it Ok; 2 s of silence → Stopped; a fresh epoch recovers; `clear()` doesn't affect liveness.
- State machine: each liveness result in Running; ignored in Starting, Fault and BatteryWait (a Fault's first reason stands); the new text.

**To verify:**
1. A normal boot shows the two new ✅ lines, every setting ✅, and epochs flowing.
2. Unplug the GNSS mid-session → `GNSS: epochs stopped - restarting into RUNNING` after about 2 s, then the boot faults (receiver missing).
3. The back-fed boot (TPS feed off) → Fault with the "sending no data" line and the red blink, about 2 s after the scheduler starts, instead of a silent Running.

#### Phase 2, GNSS item 2 (REL-4): configuration failures. Partly done (unverified); the fatal/warning decision moves to Phase 3

**Owner's decision (option C):** decide in Phase 3, alongside BUG-2's RAM-layer check.
- **Fatal** (Fault `GnssConfigFailed`, red blink): a setting that is wrong *in RAM*, i.e. actually in effect.
- **Warning only** (⚠️ in the log, a line in `D`): a setting that's right in RAM but couldn't be saved to flash.

**Why not fatal now:**
- REL-2's liveness already faults the user-visible case (PVT enable fails, or the back-fed receiver sends nothing) within 2 s.
- A ❌ today only means "didn't verify in flash". Each write goes to RAM+BBR+flash, so a receiver with worn flash may still run correctly. Making that fatal would brick a working device.

**Done now (plumbing only, no behaviour change):**
- `Gnss::configure()` returns whether every setting verified and PVT output was enabled. `Gnss::begin()` stores it, and `Gnss::configured()` exposes it for Phase 3. `begin()` still returns false only when no receiver is found.

**Review claim corrected:**
- The review suggested `switchToTargetBaud()` should check `setSerialRate()`'s result.
- The SparkFun library documents that it "can't return success/fail" because the receiver's ACK is lost in the baud change, so a check would log false failures on successful switches.
- The existing reconnect-and-verify at the new rate is the correct check. A comment now says so.

**Files:** `peripherals/gnss/Gnss.{h,cpp}`.

**Checks:** host suites pass. `Gnss.cpp` and `App.cpp` are clean against the full ESP32 core and the SparkFun library.

**Verified 2026-10-05:** both boards compile, flash and boot as before.

**Follow-up from that bench run:**
- **Symptom:** on a few consecutive nRF boots the quiet-config pause (REL-2) logged ❌, and on one boot another setting also failed.
- **Cause:** once flash holds PVT output on, the receiver streams 20 Hz PVT from power-up. The pause is the first command into that stream, and its ACK can be lost among PVT frames on the 64-byte RX ring. With the pause missed, later ACKs face the same traffic.
- **Fixes (owner approved, `peripherals/gnss/Gnss.cpp`):**
  1. The pause is retried up to 3 times, draining the UART before each retry. It logs `(attempt N)` when a retry was needed.
  2. `ensureVal()` judges a written setting by its (already retried) flash readback alone, not by the write's ACK, so a lost ACK on a write that landed no longer logs `write to flash failed ❌`.
- `Gnss.cpp` is clean against the full ESP32 core and the SparkFun library.
- **Verified 2026-10-08:** 6 nRF boots in a row with no failures.


#### R4: Diagnostics module, typed console commands, and the C rate glitch. Done

**Owner's choices:**
- App routes every console command and owns `C`'s counters.
- Commands are a typed enum.
- Each owner resets only its own state on `C`.
- A RateTracker safety net is added.

**Change:**
- **New `app/Diagnostics.{h,cpp}`**, a pure move of App's console reporting:
  - the D report assembly, the status-line grid, arming and printing, the GNSS and BLE rate trackers, `toDisplayUnits`, and the build and device-ID lines
  - references are passed at construction; it only reads the modules it reports on
  - `handle(cmd)` acts on Report, Version, ToggleStatus and counts; on Clear it re-baselines its own rate trackers
  - `tick(nowUs)` prints a status line when one is due
- **App:**
  - `consoleJob()` polls, routes with a `switch` (Clear → `clearStats()`, App's own counters), then passes the command to Diagnostics and ticks it.
  - The boot prints the build and device ID through Diagnostics.
  - Four members and six functions are gone. `App.cpp` is 270 lines (437 before the restructure).
- **`CommandKind`** (`None, Report, Clear, Version, ToggleStatus`) in `core/ConsoleEntry.h`:
  - The letters are mapped once, where the parser recognises them.
  - `ConsoleCommand` carries a `CommandKind` instead of a `char`.
  - App and Diagnostics `switch` over it, so an unrouted new command draws a warning.
- **Latent glitch fixed:** `C` while the status line ran left the rate trackers holding the pre-clear epoch count, so the next line's GNSS Hz wrapped to a huge number (owner reproduced it). Diagnostics now re-baselines on Clear, after App clears the counters.
- **Safety net:** `RateTracker::sampleCentiHz()` re-baselines and returns 0 when a counter goes backwards, instead of wrapping.

**Behaviour change:** the D report, status line, V and C output are unchanged, except that the post-`C` status line is now correct.

**Checks:**
- Host suites pass, plain and sanitized.
- `App.h` syntax-checks on arm-none-eabi-g++ 9 and xtensa g++.
- `App.cpp`, `Diagnostics.cpp` and `Console.cpp` are clean against the full ESP32 core.

**Files:** `app/Diagnostics.{h,cpp}` (new), `app/App.{h,cpp}`, `app/Console.{h,cpp}`, `core/ConsoleEntry.h`, `core/RateTracker.h`, `tests/test_consoleentry.cpp`, `tests/test_ratetracker.cpp`.

**Tests:**
- Each letter maps to its `CommandKind`.
- `RateTracker` re-baselines on a counter going backwards, and the next window measures from the reset point.

**Verified 2026-10-08:** both boards compile, flash and boot, and all console commands work.

#### R5: one device-status snapshot. Done

**Owner's choices:** pointers rather than copies (open question 5); the two refinements below.

**Change:**
- **New `core/DeviceStatus.h`:** one snapshot with named nested parts.
  - `uptimeUs`
  - `gnss` (epoch tracker, latest epoch or nullptr)
  - `imu` (up, the raw `ImuSample` in physical units, trim locked, tilt in degrees)
  - `ble` (connected, `BleStats`)
  - `power` (battery or nullptr, USB present)
  - `consoleDropped`

  It has no default member values, so it stays a C++11 aggregate: `DeviceStatus s = {};` zeroes everything, and fields are set by name.
- **The formatters:** `formatReport(buf, cap, JobTable, DeviceStatus)` and `formatStatus(buf, cap, DeviceStatus, StatusRates)`. `ReportView` and `StatusView` are gone.
  - The job table is report-only.
  - The per-line rates are a separate argument, so the snapshot has no fields that are valid only on some paths.
- **The IMU display conversion moved into core and into the formatters:** `toDisplayUnits()` and `tiltCentiDeg()` in `core/ImuSample.h`, truncating exactly as before. It is now host-tested, and the snapshot carries physical units with no "pre-converted" fields.
- **Diagnostics:** `snapshot(nowUs)` is one function of plain assignments, used by both the report and the status line. The status line adds `StatusRates`.

**Behaviour change:** none. **Every expected output string in `test_report.cpp` and `test_statusline.cpp` is unchanged since the Phase 1 commit;** the only diffs in those files are setup lines and an include. That is the byte-for-byte proof.

**Checks:**
- Host suites pass, plain and sanitized.
- Both formatter tests compile with arm-none-eabi-g++ 9 and xtensa g++ (`-Werror`).
- `App.h` syntax-checks.
- `App.cpp`, `Diagnostics.cpp` and `Console.cpp` are clean against the full ESP32 core.

**Files:** `core/DeviceStatus.h` (new), `core/ImuSample.h`, `core/Report.h`, `core/StatusLine.h`, `app/Diagnostics.{h,cpp}`, `app/Console.{h,cpp}`, `tests/display_units.h` (new test helper), `tests/test_report.cpp`, `tests/test_statusline.cpp`.

**Tests:**
- The formatter tests build snapshots by named fields. A test helper turns intended display integers into physical-unit inputs half a unit from zero, so truncation lands exactly.
- New `testDisplayUnits`: 9.81 m/s² → 1000 mG, −9.81 → −1000 (truncation toward zero), 1 rad/s → 5729 c°/s, tilt 12.349° → 1234 centi-deg.

**Verified 2026-10-08:** both boards; `D` and the status line are exactly as before.

#### Console follow-ups after R5 (owner requests, 2026-10-08). Done (unverified)

- **`L` is capped at `kStatusMaxCount`:** `L` now starts a run of `kStatusMaxCount` lines (3600) instead of running forever, so status reporting left on by a host that went away still ends by itself. A second `L` still stops it. `statusRemaining_` became unsigned, with no "continuous" (−1) value.
- **Boot help wording:**
  - `C` is described as "clear statistics counters", so it isn't mistaken for clearing the screen.
  - `L` is described as "1s status on/off (up to 3600 lines)".
- **The help is split into two lines:** the state (`Running.`), then the command list. The longer wording would have passed `Console::log`'s 160-character limit in every state: 161–197 characters combined, against 152 for the command line alone.

- **D report IMU line reformatted:** `IMU: up, Accel XYZ mG 0 0 1000, Gyro XYZ c°/s 0 0 0` (mG, matching the status line), with units once per group instead of after every value (`core/Report.h`, with `tests/test_report.cpp` expectations updated).

**Files:** `app/Diagnostics.{h,cpp}`, `app/App.cpp`, `app/Console.h`, `app/Config.h` (comments), `core/Report.h`, `tests/test_report.cpp`.

**Checks:** host suites pass. `App.cpp` and `Diagnostics.cpp` are clean against the full ESP32 core.

**To verify:**
- The boot ends with the state line, then the command line, in full.
- `L⏎` starts lines, and `L⏎` again stops them.

#### Phase 2 item 3 (REL-1): hardware watchdog and reset reason. Done (boot verified)

**Finding confirmed:**
- Neither board ran a watchdog on the loop. The ESP32 core starts its task watchdog (5 s, panic) but only watches the idle tasks: `loopTaskWDTEnabled` is false and the loop task is never added. Nothing starts `NRF_WDT` on the nRF.
- The boot log never said why the chip started, so a watchdog reset would have looked like a power cycle. The nRF core already latches and clears `RESETREAS` before `setup()` (`readResetReason()`).

**Fix (owner approved after two refinement rounds):**
- **`platform/Watchdog.h`:** `start()` and `feed()`, `config::kWatchdogMs` = **5000**.
  - nRF: `NRF_WDT`, keeps running while the CPU sleeps, pauses under a debugger.
  - ESP32: the existing task watchdog retimed to ours (same idle-task checks, panic kept, so a hang prints the stuck task's backtrace), and the loop task added.
- **Started on every boot, right after `holdAllOff()`,** so a hang during boot is caught too. On the nRF a soft reset doesn't stop the watchdog, and its registers lock once it runs. Starting it again with the same settings is harmless, so there's no "already running" special case.
- **One feed rule: anything that can block for over a second feeds first.**
  - `App::tick()` (every pass, in every state).
  - Between boot steps (console, battery, each state entry in `apply()`) and inside the nRF's 3 s USB wait.
  - In `Gnss.cpp`, before every library call that waits for a reply: detection, baud switch and save, the PVT pause attempts, each flash-read attempt, each write, and the callback registration.
- **Why not "feed only if GNSS ran recently" (the review's rule):** every hang it lists blocks inside a job, which already stops the per-pass feed. "Loop runs but no data" is the REL-2 liveness check's job. The GNSS job is off in Fault, BatteryWait and ChargeOnly, so the rule would need per-state exceptions.
- **Timeout 5 s, not the 4 s first discussed:** the longest unfed stretch is detection on a silent port. The library's `begin()` tries three times at its 1.1 s timeout, between `tryBaud()`'s 100 ms pauses: about 3.5 s. `Gnss.cpp` asserts `3 × kUBLOXGNSSDefaultMaxWait + 1000 ≤ kWatchdogMs`, so lowering the watchdog or a library timeout increase fails the build.
- **Reset reason:** `Reset: <cause>` at boot (after `Gnimu starting`) and as the first line under the D report's RT/Loops table, so it can be read any time later, not only with a terminal open at boot.
  - nRF: watchdog, CPU lockup, software, reset pin, wake from System OFF (USB or pin). No bit set prints `power-on or brownout`: the nRF can't tell them apart.
  - ESP32: from `esp_reset_reason()`, including brownout and crash.
- **Our own restart reason is kept across the reset:** one byte, `RestartReason + 1`. nRF `GPREGRET2` (through the SoftDevice when it's running; `GPREGRET` is the bootloader's). ESP32 an `RTC_NOINIT_ATTR` byte. `restart()` stores it before resetting.
  - Read and cleared at boot. Shown only when the hardware cause is "software", e.g. `Reset: software (GNSS: epochs stopped)`. The byte also survives watchdog and pin resets, and the ESP32's copy is garbage after power-on.
  - A software reset with nothing stored (an upload) stays plain `software`.
- **Boot log also prints** `Watchdog: 5000 ms ✅`, or `❌` if it didn't start.

**Considered and dropped:** a reset counter (one reason plus RT answers "did it, why, when"); saving the hung PC from the nRF's pre-reset interrupt (about 61 µs).

**Files:** `platform/Watchdog.h`, `platform/ResetInfo.h`, `platform/{nrf52840,esp32}/{Watchdog,ResetInfo}.cpp` (all new); `core/ResetReason.h` (new); `core/RunStateMachine.h` (`kRestartReasonCount`); `core/DeviceStatus.h`, `core/Report.h`; `app/App.{h,cpp}` (`restart(RestartReason)`), `app/Diagnostics.{h,cpp}`, `app/Config.h`; `peripherals/gnss/Gnss.cpp`; `platform/nrf52840/ConsolePort.cpp`; `tests/test_resetreason.cpp` (new), `tests/test_report.cpp`.

**Tests:** `test_resetreason.cpp`: each restart reason shown after a software reset; nothing stored → plain `software`; a stale reason ignored after watchdog, pin and power-on resets; out-of-range codes ignored; every in-range code names a real reason; cause texts; truncation. `test_report.cpp` checks the `Reset:` line's position and text.

**Checks:** host suites pass (plain and sanitized). Every changed firmware file is clean against both cores (`-fsyntax-only`, with injected-error probes confirming each check really compiles).

**Verified 2026-10-08:** owner accepted the 5 s timeout; compiled, flashed and booted on both boards, working. Uploads onto the running new firmware (watchdog already running) complete normally on both boards, so the nRF bootloader copes with a running watchdog. Forced hang (a temporary `for (;;)` under `V`, since removed) reset both boards after about 5 s, and the next boot reported `Reset: watchdog`.

**Still to verify, both boards:**
- A power-on prints `Reset: power-on or brownout` (nRF) / `Reset: power-on` (ESP32) and `Watchdog: 5000 ms ✅`. **If every nRF reset shows `power-on or brownout`, the bootloader is clearing `RESETREAS` first.**
- Restart paths still boot cleanly with the watchdog inherited, and say why: switch on from BatteryWait (`software (Power: switch on)`), USB removed in ChargeOnly, GNSS unplugged mid-run (`software (GNSS: epochs stopped)`).
- `D` shows the same `Reset:` line under the RT/Loops table.

#### Phase 2 item 4 (REL-7): BLE start failure → Fault. Done (forced failure verified)

**Finding confirmed, and worse on ESP32:**
- nRF: a failed `Bluefruit.begin()` was logged ❌, and the device entered Running anyway with the LED blinking "advertising". A failed first `Advertising.start(0)` was also log-only.
- ESP32: `BLEDevice::init()` returns `bool` in this core, and it was ignored. The code then built the server and services on a stack that wasn't running, which would likely crash (since REL-1, a crash/reboot loop). A failed `advertising->start()` was log-only too.
- Never seen on the bench. It can only happen at boot.

**Fix (owner approved after one simplification round):**
- **`begin()` returns `bool`** on both transports: true only once advertising. It returns false if the stack fails to start, stopping right there, or if the first advertising start fails. Both mean nothing can connect. `BleStub::begin()` returns true, since there's no stack to fail.
- **All bring-up moved into Starting.** It runs GNSS, then the IMU, then BLE, and ends in one event: `gnssStarted(bool)` became `started(FaultReason failure)`. `None` enters Running; any other reason enters Fault with that reason. Running has no entry effects any more, which is safe because it's only ever entered from Starting. The apply loop's "at most twice" bound is unchanged.
- **`FaultReason::BleFailed`**, with the message `BLE failed to start. Power-cycle to retry. Everything else stopped.` Fault itself is unchanged: peripherals stopped, console only, rapid red blink (fast blue on ESP32). `stateName(Fault)` is now `Fault - console only`, no longer GNSS-specific.
- **No retry,** matching the owner's GNSS-fault decision (a person power-cycles). One automatic retry through the REL-1 retained restart reason was considered and dropped. Both stacks fail to start only for causes a reboot doesn't fix (memory, NVS, SoftDevice configuration).
- **Watchdog:** Starting feeds before the IMU and before BLE, keeping the one-feed-per-boot-step rule.
- **Checked:** a failed advertising start leaves `started_` true, so Fault's `stopPeripherals()` still runs `ble_.end()`. Battery updates, `connected()` and `send()` all check `built_`/`started_`, so nothing touches a half-started stack in Fault.

**Files:** `core/RunStateMachine.h`, `app/App.cpp`, `peripherals/ble/BleNrf.{h,cpp}`, `peripherals/ble/BleEsp32.{h,cpp}`, `peripherals/ble/BleStub.h`, `tests/test_runstatemachine.cpp`, `docs/app-restructure-design.md` (implementation note).

**Tests:** the `gnssStarted` cases moved to `started(GnssMissing)` / `started(None)`. New cases: `started(BleFailed)` faults with `BleFailed` (battery and no-battery boards); `started()` of every kind is ignored outside Starting and leaves no fault reason behind; the new message text and state name.

**Checks:** host suites pass (plain and sanitized). `App.cpp`, `BleNrf.cpp` and `BleEsp32.cpp` are clean against both cores.

**Fault blink slowed (owner, 2026-10-08):** after seeing it on the bench, `kLedFaultHalfMs` went from 50 to 100 ms: 5 Hz instead of 10 Hz. It's still twice BATTERY_WAIT's 2.5 Hz, and a whole multiple of the 25 ms LED job period. Applies to every fault. `test_statusled.cpp` keeps its own 50 ms test timing, since it checks the blink logic, not the configured rate.

**Verified 2026-10-08, both boards:** forced failure (a temporary override of `begin()`'s result after the stack and advertising had started, since removed) gave the Fault message, `Fault - console only.`, the fault blink, D and V still answering, and BLE gone from a scan. That also exercises Fault's `ble_.end()` teardown of a started stack.

**Still to verify, both boards:**
- A normal boot is unchanged: `BLE: advertising as ... ✅`, then `Running.`, and a client connects.
- No GNSS (unplugged) still gives the GnssMissing fault, with no BLE started.

#### Phase 2 item 5 (PWR-1): switch-sense noise. Done (bench verified)

**Finding confirmed (`core/PowerPolicy.h`):**
- A switch-OFF reading was debounced (500 ms) and acted on even without USB. On battery, switching off cuts the MCU's power, so such a reading can only be noise. It stopped the GNSS and IMU mid-session and went to BatteryWait, then the next ON reading rebooted. The class comment claimed every switch-off happens with USB, but nothing enforced it.
- **The same hole at boot (missed by the review):** `begin()` acts on one reading, so a single noisy OFF at power-up on battery meant BatteryWait, then a reboot.
- Switch-ON from BatteryWait was not debounced: one ghost reading rebooted.
- Already fine: the ADC acquisition time is 40 µs (`BatteryPort::begin()`), within the nRF spec for the divider's ~255 kΩ source, and the first conversion after a channel switch is already discarded.

**Fix (owner approved after one simplification round, with the noise counter):**
- **The USB rule applied to the reading, once:** "OFF and no USB" reads as ON, at the top of both `begin()` and `update()`. Nothing downstream needs to know about USB. At boot it means a normal start, or System OFF if the battery is below cutoff.
- **One stable switch state, debounced on both edges:** a small `Debounced` helper in `PowerPolicy.h`. A new reading becomes the stable value only after holding for `kSwitchDebounceMs` (renamed from `kSwitchOffDebounceMs`, still 500 ms). The policy acts only on stable edges, which removed its own pending-off tracking.
- **Gesture timing preserved without extra code:** both edges are delayed alike, so the time between the confirmed OFF and the confirmed ON equals the real OFF time. "Back on within 2 s" means the same as before. Switch-on (reboot, or entering CHARGE_ONLY) now takes effect 500 ms after the switch moves.
- **Noise counter:** `PowerPolicy::switchNoise()` counts glitches (runs of differing readings shorter than the debounce) plus runs of OFF readings without USB, each run once. It's shown as `switch noise N` at the end of the D report's Battery line, and `C` clears it. A car session that ends at 0 says the sense line needs no RC filter.
- **Hardware RC filter:** left to the owner as a PCB decision. The noise counter gives the field data for it.
- **Considered and dropped:** several readings at boot. With the USB rule, the only case left is a ghost OFF at boot on USB: one extra reboot on the bench, never in the car. Waiting out a debounce at boot would add 500 ms to every boot.

**Files:** `core/PowerPolicy.h`, `core/RunStateMachine.h` (`switchNoise()`, `clearSwitchNoise()`), `core/DeviceStatus.h`, `core/Report.h`, `app/Config.h`, `app/App.cpp`, `app/Diagnostics.{h,cpp}`, `tests/test_powerpolicy.cpp`, `tests/test_runstatemachine.cpp`, `tests/test_report.cpp`.

**Tests:**
- `test_powerpolicy.cpp`: boot OFF without USB is a normal boot (or System OFF below cutoff) and counts as noise. OFF without USB is ignored however long, each run counted once, and C clears it. A short flick counts one glitch. A single ON ghost in the wait is ignored, and a held ON reboots 500 ms later. Gesture at 1950 ms is confirmed 500 ms later and enters CHARGE_ONLY; 2000 ms reboots. Leaving CHARGE_ONLY, and USB removal (not debounced, a digital VBUS detect). `Debounced` on its own.
- `test_runstatemachine.cpp`: switch-on transitions held through the debounce, and switch-OFF on battery leaves Running and counts as noise.
- `test_report.cpp`: the Battery line ends with `switch noise 3`.

**Checks:** host suites pass (plain and sanitized). `App.cpp` and `Diagnostics.cpp` are clean against both cores.

**Bench finding, and the owner's change (2026-10-08): CHARGE_ONLY from BATTERY_WAIT.**
- On the bench the gesture worked from Running but not from BATTERY_WAIT, as designed: the gesture was only armed from Running, before and after PWR-1. The owner's use case is a device that's already off, gets plugged in (booting into BATTERY_WAIT), and should then charge.
- In BATTERY_WAIT the switch is off, so the battery is disconnected and any route to charging has to start with switch-on, which meant "run". Options were a quick on → off → on from BATTERY_WAIT (every normal start from there 1.5 s slower), switch-on from BATTERY_WAIT means CHARGE_ONLY, or no change.
- **Owner chose: switch-on from BATTERY_WAIT means CHARGE_ONLY.** The normal state is battery power in the car, where the MCU only ever sees the switch on. A plugged-in, switched-off device is on a charger.
- **New rule (`PowerPolicy`):** on USB, switch off waits (BATTERY_WAIT); switch on from there enters CHARGE_ONLY, except straight after CHARGE_ONLY, where off/on reboots into RUNNING (the way to run while plugged in). Unplugging USB in CHARGE_ONLY still reboots into RUNNING on battery.
- **The 2 s gesture window is gone** (`kChargeOnlyWindowMs`, its `static_assert`, the config field and the timestamp): from Running on USB, off then on enters CHARGE_ONLY however long it was off. Before, an off longer than 2 s rebooted into Running.
- The CHARGE_ONLY entry message is now `Power: switch on with USB - CHARGE_ONLY, charging with everything else off`.
- The temporary switch-voltage logging used for the investigation is removed; `BatteryPort` is back to its committed state.
- **Tests:** off then on from Running enters CHARGE_ONLY for 0.5 s, 1.95 s and 10 s offs; boot into the wait then on enters CHARGE_ONLY; an ON ghost in the wait does nothing; off/on from CHARGE_ONLY reboots into RUNNING (quick or not); USB removal from CHARGE_ONLY; the machine's BatteryWait and CHARGE_ONLY transitions and message text.

**Verified 2026-10-08 (owner):** all the checks below pass on the nRF.

**Checks, nRF:**
- On battery, normal use is unchanged, and D shows `switch noise 0` (or a small number).
- On USB, from Running: switch off goes to BATTERY_WAIT after about 0.5 s; switch on (after any delay) enters CHARGE_ONLY about 0.5 s later.
- Plugged in with the switch off (boot into BATTERY_WAIT), then switch on: CHARGE_ONLY.
- From CHARGE_ONLY: off/on reboots into RUNNING; unplugging USB reboots into RUNNING.
- `C` resets `switch noise` to 0.

#### Phase 2 item 6 (CLK-1): the nRF clock. Done (boot verified; battery run pending)

**Findings:**
- **New, and the most serious: the clock was used before it started.** `App::begin()` logged about six lines before `PlatformClock::begin()`. `Console::emit()` times its "wait at most 2 ms for room" loop with `nowUs()`, which on the nRF stands still until TIMER4 starts. A USB host holding the port open without reading, filling the 256-byte buffer during those lines, would have hung the boot (since REL-1, a watchdog reset loop for as long as the host stayed stuck). ESP32 unaffected: its timer runs from power-up.
- **Accuracy (the review's point):** TIMER4 and the UART baud generators run from HFCLK. Nothing requested the crystal, so on battery they most likely ran on the internal RC oscillator (about ±1.5%). On USB the USB stack requests the crystal, which is why the 512 s bench run agreed to about 0.2%. On battery, RT, status-line Hz, `max gap` and IMU sample spacing drift by that much (harmless), and a 115200-baud UART on a clock 1.5% off is closer to the receiver's tolerance (unmeasured).
- **Reentrancy (the review's other point):** `nowUs()`'s capture and `last`/`wraps` update aren't safe against a second task. Every caller was checked and runs on the loop task: the scheduler and jobs, Diagnostics, Console, BLE `send()`, and the GNSS PVT callback (inside `Gnss::poll()`). No BLE stack callback uses it.

**Fix (owner approved after one simplification round):**
- `PlatformClock::begin()` moved to the top of `App::begin()`, before the console (only register writes). The `Clock started` log line stays where it was.
- New `PlatformClock::useCrystal()`, called once at the end of Starting, right after BLE begins. On the nRF it goes through the SoftDevice (`sd_clock_hfclk_request()`) when that's running, else starts HFXO directly (the `systemOff()` pattern). The SoftDevice then holds the crystal until a reset or System OFF. One call after the SoftDevice is up avoids depending on whether enabling it keeps an earlier app-started crystal. ESP32: returns true (its timer already runs from the 40 MHz crystal). Logs `Clock: crystal ✅/❌` on both boards.
- Boot (GNSS detection and configuration) and the Fault states stay on whatever clock is running, as on battery today. The earlier PVT-pause ACK losses were on USB, where the crystal already ran, so they weren't this.
- `nowUs()` documented as loop-only in `Clock.h`, with the caller list. No runtime check: thousands of calls a second, and making it fully safe would add code for a caller that doesn't exist.
- Cost: about 0.25 mA for the crystal, negligible next to the GNSS.

**Files:** `app/App.cpp`, `platform/Clock.h`, `platform/nrf52840/Clock.cpp`, `platform/esp32/Clock.cpp`.

**Checks:** host suites pass; `App.cpp` and both `Clock.cpp` files are clean against their cores.

**Verified 2026-10-08 (owner):** both boards boot normally with `Clock: crystal ✅`.

**Still to verify:**
- Both boards: the boot shows `Clock: crystal ✅` after the BLE line, and is otherwise unchanged. (Done.) On USB this only shows the request was accepted: USB already runs the crystal.
- **nRF on battery, the real check:** about 10 minutes on battery with a fix, then plug in USB and type `D` straight away. `epochs ÷ 20` should match `RT` to within about a second, with `missed` near 0. (On the internal oscillator, 1.5% would be about 9 s over 10 minutes.) No new code needed: the D counters already compare GNSS time with the MCU clock.

#### Phase 3 (BUG-2 + REL-4): GNSS settings checked in RAM; a setting not applied is fatal. Done (bench verified)

**Finding confirmed (`Gnss.cpp`, `ensureVal()`):** the receiver runs on its RAM layer, built at startup from defaults, then flash, then BBR (which overrides flash). Only flash was read, so a stale BBR value (left by u-center, Classic or an earlier firmware) passed as ✅ while the receiver ran something else. This matters most on the ESP32, where the GNSS stays powered across MCU resets. On the nRF, BBR survives the GNSS rail power-cycle only if the module's backup supply is powered.

**Fix (owner approved after one simplification round):**
- **RAM checked per key; flash read only after a write.** A normal boot reads RAM once per key (the same number of reads as before, so boot time is unchanged and CPLX-6 batching stays unneeded). A mismatch is written to all layers (RAM, BBR and flash, so a stale BBR value is overwritten too), then judged by readback:

  | After the write | Log | Result |
  |---|---|---|
  | RAM still wrong, or no reply | `could not be set ❌` | **fatal** |
  | RAM right, flash right | `written ⏬` | fixed |
  | RAM right, flash wrong | `set, not saved to flash ⚠️` | warning: in effect now, rewritten next boot |

- **Why flash needn't be read on a normal boot:** RAM is what the receiver runs, so checking it catches the BBR case and any other effective mismatch. Flash is written whenever RAM was wrong (e.g. a first-ever configuration), and each later boot's RAM comes from it. Flash wrong while BBR holds the right value is invisible but harmless while BBR lasts; once BBR is lost, that boot's RAM is wrong and gets fixed.
- **NAV-PVT output key:** still checked against flash only (`Check::FlashOnly`), because configure() holds it at 0 in RAM. Failing to persist it is a warning.
- **The PVT callback's missing ACK is now a warning,** not a failure: it can be lost among the PVT traffic it just enabled. REL-2 liveness is the real check (`GnssNoData` after 2 s).
- **REL-4 (owner's option C):** `configure()` returns false only for a setting left wrong in RAM. Starting then returns `started(FaultReason::GnssConfigFailed)`, message `GNSS settings could not be applied. Power-cycle to retry. Everything else stopped.` Warnings are summarised (`N setting(s) not saved to flash - in effect now, rewritten next boot ⚠️`) and the device carries on.
- `kFlashReadAttempts` became `kReadAttempts`; each attempt feeds the watchdog (REL-1), RAM reads included.
- **Considered and dropped:** a separate host-tested `core/GnssKeyCheck.h` (three outcomes after a write; a comment covers it) and a `configuration took N ms` line (no extra reads on a normal boot).

**Risk raised in review, to bench-check:** the `SIGNAL_*` keys have combination rules (BeiDou B1I/B1C exclusive; a system can't be enabled with all its signals off). A single-key write that passes through an invalid combination is NAKed for every layer. That used to be a ❌ line; under option C it leaves RAM wrong and is fatal. The key order avoids it, and the first-ever configuration went through, but it's exercised only when the constellation selection changes. If a NAK shows up, the fix is one transaction for the signal group (CPLX-6 batching limited to that group).

**Files:** `peripherals/gnss/Gnss.{h,cpp}`, `core/RunStateMachine.h`, `app/App.cpp`, `tests/test_runstatemachine.cpp`.

**Tests:** `started(GnssConfigFailed)` faults with that reason; its message text. (`Gnss.cpp` is library-bound and has no host tests.)

**Checks:** host suites pass (plain and sanitized). `Gnss.cpp` and `App.cpp` are clean against both cores.

**Verified 2026-10-08 (owner), both boards:** a normal boot shows every key ✅. Changing `kGnssMinElevDeg` showed `written ⏬`, and changing it back showed `written ⏬` again. **Boot is noticeably quicker** than with flash reads, which suggests the receiver answers RAM reads faster than flash reads. **Fatal path verified:** a temporary hack (since removed) made the dynamic model's RAM checks fail. It gave `could not be set ❌`, the configuration-failed line, the `GnssConfigFailed` message and the fault state, as expected. **Constellation swap verified in both directions:** the signal keys were written with no NAK, so the key order holds under option C and signal-group batching isn't needed.

**Still to verify, both boards:**
- Optional (ESP32, GNSS stays powered): write a different value to BBR only, then reset. Flash still matches but RAM doesn't; the boot should rewrite it.

#### Phase 4, step 1: `max late μs` column and baseline; rescan declined. Done (bench verified)

**Owner's plan:** add the measurement first, run both boards for a while to get a baseline, then decide on the rescan.

**Change:** `JobStats::maxLateUs`, the longest any run started after it was due (counted from the original due time, so skipped periods count). Shown as a `max late μs` column at the right of the D report's job table. `C` clears it with the other stats. No scheduling change.

**Reading it (`gnss` row, nRF):** the UART ring holds about 5.5 ms and `gnss` is due every 1 ms, so the drain gap is about 1 ms plus its lateness. Treat about **4.5 ms** as the limit. Expect a floor of up to about 1 ms even when idle, from the 1 ms RTOS-tick sleep. It's a maximum since the last `C`: to attribute it, press `C`, do one activity, then `D`. A D report's own cost shows on the *next* `D`. ESP32's GNSS ring is about 44 ms, so its numbers are context only.

**Files:** `core/Scheduler.h`, `core/Report.h`, `tests/test_scheduler.cpp` (`testMaxLate`), `tests/test_report.cpp`.

**Checks:** host suites pass (plain and sanitized). Clean against both cores.

**First baseline run (nRF, USB, healthy link, 2026-10-09), about 10 minutes with D presses and status lines:**
- `gnss` max late reached **5677 µs**, while its own max run was 931 µs (BLE send max 452 µs). The stall was the **console job**: max 5229 µs, with one budget overrun per D press (8 overruns for 8 reports). The D report has grown from the 3.9 ms measured earlier (the `Reset:` line, the late column, `switch noise`). 5677 ≈ one D report plus a small pile-up of other jobs or wake jitter.
- The idle floor at the first D (8 s) was 1070 µs, matching the expected ~1 ms RTOS-tick sleep jitter.
- One epoch missed, before 83 s (max gap 97.7 ms), none in the following ~500 s despite stalls past 5.5 ms. The cause is ambiguous: either an early D report overflowing the UART ring, or the receiver while acquiring its first fix (`time steps 1`). A stall only loses bytes when it overlaps a frame's ~8.7 ms arrival, about 17% of the time.
- **Client side (owner):** solid. 20 Hz, more than 18,000 frames received, 0% errors.
- **Reading:** the D-report stall is bench-only. In the car there's no USB host, and `Console::emit()` returns at once with no listener; status lines only run if `L` is typed. The rescan can't remove it (it is the longest single job). The field-relevant number is lateness **without console activity**: next, `C`, then 30–60 min untouched (no D, no L), then one D.
- **Possible bench-only follow-up:** make D non-blocking. First split its ~5 ms into formatting versus waiting for USB; queueing console output and draining it per tick fixes the latter.

**Field-like baseline (nRF, USB, client connected, no console activity, 2026-10-09), about 300 s after `C`:** `gnss` max late **1299 µs**, max run 579 µs, 0 missed of 6017 epochs, BLE max send 216 µs. Without console activity the worst case is bounded by the table itself: one sleep wake-up (about 1 ms) plus every other job's max run in the same tick (imu 433 + battery 196 + power 115 + ble 12 + led 133 + console 50 ≈ 0.94 ms), **about 2 ms whatever the epoch alignment**, so a longer run wasn't needed. Side notes: `max gap` 69 ms with nothing missed was one epoch output about 19 ms late by the receiver (the drain was never more than 1.3 ms late). About 10% of loop wake-ups find nothing due (335,246 loops against about 300,000 GNSS runs), because the nRF's 1024 Hz tick (0.98 ms) is slightly shorter than the 1 ms drain period. That also explains the ~1 ms floor and the `skipped` count. Harmless.

**Rescan: declined (owner, 2026-10-09).** It could save at most about 1 ms off a 2 ms worst case that's already under half the 4.5 ms limit, and it can't touch the only stalls that matter: a D report (bench-only) or a BLE send inside the `gnss` job (the weak-link soak's question). The `max late μs` column stays: the soak needs it.

**Rescan as proposed (kept for the record; declined above):** after every other job, re-check job 0 (`gnss`) and run it if due. Every other job still runs at most once per tick (no starvation, unlike the review's full rescan from 0). Plus `static_assert(kGnssJob == 0)` in the App constructor, so reordering the table can't drop the protection. It bounds the drain gap by the longest *single* job (about 3.9 ms for D), and can't help a stall inside the `gnss` job itself (a BLE send on a weak link), which is the soak's question. If the baseline sits far below 4.5 ms, dropping the rescan is a fair call.

**Possible later item: finer sleep than the RTOS tick.** `sleepUntilUs()` uses `delay()`, one tick of about 1 ms (nRF 1024 Hz, ESP32 1000 Hz), so starts jitter up to about 1 ms. Options: busy-wait the last fraction (costs battery current, and on the nRF bypasses `delay()`'s USB-serial flush), or a timer-compare wake via task notification (µs-accurate, still sleeps; needs an interrupt that fits the SoftDevice's priority rules and must keep the USB flush). Not worth it now: it buys about 1 ms of the 4.5 ms margin, while the real risks are single-job stalls. Revisit if the baseline's idle floor is high, or with DATA-1 (IMU-to-GNSS time alignment).

#### Phase 5: REL-6 declined; REL-3 ESP32 I²C timeout. Done (boot verified)

**REL-6 (IMU latched down until reboot): declined, owner's choice (2026-10-09).**
- The IMU is secondary. Its state changes no run state, jobs or LED, and GNSS keeps streaming without it, so a dropout is a degraded session, not a failed one. Recovery would add code for an unlikely edge case. A down IMU waits for the next power cycle, which the existing log line already says (`IMU fields will read zero until reboot`).
- A down IMU's all-zero accel is recognisably invalid (see Corrections), so declining doesn't leave the data ambiguous.
- Proposals considered on the way: a non-blocking reset/settle state machine (bring-up blocks for 200–300 ms), then the simpler "re-run `configure()` every 1 s" with a `recoveries` counter in D. Both declined with REL-6.
- `core/ImuHealth.h`'s comment now gives the real reason for the latch instead of "matching Classic".

**REL-3 (ESP32 part): done.** `Wire.setTimeOut(config::kImuI2cTimeoutMs)` (2 ms) in the ESP32's `imuBusBegin()`, after `Wire.begin()`. Before, each failed read waited the core's 50 ms default, so the 10 failures before the latch stalled the loop for about 500 ms, and with the ESP32's ~44 ms GNSS ring that would have cost roughly 10 epochs. Now about 2 ms per failed read, about 20 ms in all. A 14-byte read at 400 kHz takes about 0.4 ms. On the nRF, a stuck bus still hangs `Wire`, and the REL-1 watchdog resets the device; the bounded nRF I²C driver stays deferred.

**CPLX-2:** stays a later cleanup (no longer needed by REL-6).

**Files:** `app/Config.h` (`kImuI2cTimeoutMs`, comment on `kImuDownAfterFailures`), `platform/esp32/ImuBus.cpp`, `core/ImuHealth.h` (comment only).

**Checks:** host suites pass; `ImuBus.cpp` is clean against the ESP32 core.

**Verified 2026-10-09 (owner):** both boards boot and work normally. **Still optional (ESP32):** IMU readings unchanged. Optionally pull the MPU-6050's wire mid-run: expect `stopped answering ... until reboot`, D `IMU: down`, and `missed` unchanged (before this, that would have cost epochs).

#### CPLX-5 (rest): one clock; the remaining platform-layer items declined. Done (boot verified)

**What was left after the console part:** four `millis()` calls (the 1 s MTU-refusal log timer in `BleNrf.cpp` and `BleEsp32.cpp`), boot-time `delay()` / `delayMicroseconds()` waits (`Gnss.cpp`, both IMU drivers, `Console` flush, `primeBattery()`), and `Console` using `Serial` directly.

**Done (owner chose option A, "one clock"):** both MTU-refusal timers now use `PlatformClock::nowUs()` (`mtuRefusingSinceUs_`, `kMtuRefusalLogUs`). Everything that *measures* time now uses one timebase. Both run inside `send()`, on the loop task, matching `nowUs()`'s loop-only rule. No behaviour change.

**Declined:** wrapping `delay()` / `delayMicroseconds()` and routing `Console`'s `Serial` through `ConsolePort`. These are portable Arduino calls, the same on every board, like `Wire`. The boundary is: `platform/` holds what differs per board (`ConsolePort` already holds the USB wait and the input flush), and portable Arduino calls stay where they're used. Wrapping them would touch every bench-verified driver for no functional gain. A `Console` without direct `Serial` would help host testing, which belongs with TEST-1 if that's ever done.

**Files:** `peripherals/ble/BleNrf.{h,cpp}`, `peripherals/ble/BleEsp32.{h,cpp}`.

**Checks:** host suites pass; both BLE files are clean against their cores.

**Verified 2026-10-09 (owner):** both boards compile, boot and work normally. The MTU-refusal log only appears with a client that never raises its MTU.

#### TEST-3: project warnings fail the firmware build. Done

**Finding confirmed:** CI and `scripts/build.sh` compiled both boards with `--warnings all` (`-Wall -Wextra` on both cores; the nRF core turns a few back off), but nothing failed on a warning. Host tests already build with `-Werror`.

**Baseline:** with the IDE's bundled arduino-cli, both boards build with **0 warnings**, project and vendor alike, so nothing needed cleaning up first.

**Fix (owner approved after two refinement rounds), all in `scripts/build.sh`:**
- After the compile, the output is searched for `: warning:` lines under the sketch's own path (`$PWD/code/Gnimu/`). Any match prints the lines and fails the script. Core and library warnings are shown but don't fail. Arduino has no sketch-only `-Werror`: `compiler.cpp.extra_flags` reaches the core and every library too.
- `--clean` on every build, so every source is recompiled and every warning printed. An incremental build skips unchanged files, and their warnings with them, so a local re-run could otherwise pass falsely. The core stays precompiled in its own cache.
- No argument builds both boards (ESP32, then nRF, stopping at the first failure). CI's matrix still passes one board each; the workflow is unchanged.
- arduino-cli is `$ARDUINO_CLI` if set, else on the PATH, else the copy bundled with the Arduino IDE on macOS.
- The output goes through `tee` under `pipefail`, so a failed compile still fails, and "no matches" from `grep` is handled as the pass.

**Checks:**
- `scripts/build.sh`: both boards pass (`no warnings in project sources`).
- A scratch copy with a planted unused variable in `App.cpp` fails, naming `App.cpp:221:7`.
- The filter ignores a vendor-library warning line (synthetic log in arduino-cli's format; the installed libraries weren't touched).

**Side note from the build:** the ESP32 image uses **91% of its flash partition** (1,194,083 of 1,310,720 bytes). Not a problem today, but little headroom.

#### D-report wording (the "later, separate effort"). Done (bench verified)

**Owner's decisions (2026-10-09):** all three planned items, plus `fix OK`, `Fix: 3D` on the status line, and the dynamic model on the settings line. `charging` in D now uses `isCharging()`, reversing R6's raw-pin choice for D.

**Changes:**
- **New first GNSS line:** `GNSS: GPS+Galileo, 20 Hz, automotive`. Built once by Diagnostics from `Config.h` (constellations in priority order via the new `config::gnssSystemName()`, `kGnssNavRateHz`, `kGnssDynamicModel`). These are the targets the boot check verified, not a live readback. Carried in `DeviceStatus::gnss.settings`; `nullptr` omits the line.
- **Fix line in words:** `GNSS: 3D fix, 11 SVs, hAcc 597mm, tAcc 22ns, pDOP 2.34`. Fix types from NAV-PVT: no fix, dead reckoning, 2D fix, 3D fix, GNSS + dead reckoning, time only. `(not yet OK)` follows any fix type other than "no fix" while `gnssFixOK` is false. An accuracy the receiver can't estimate (all bits set) shows as `unknown`. Before any epoch: `GNSS: no epoch yet`.
- **Battery line:** only the true flags, as words (`USB`, `charging`, `full`, `warning`, `critical`, `cutoff requested`), e.g. `Battery: 4.16V (last peak 4.18V), 100%, USB, full, switch noise 0`. `charging` is `isCharging()` (USB present and the charger pin), the same as the boot line, status line and RaceBox bit.
- **Status line:** `Fix: 3D` (short names: none, DR, 2D, 3D, GNSS+DR, time). Its pre-epoch `Fix: none` is unchanged.
- **New `core/GnssNames.h`:** `fixTypeName()`, `fixTypeShortName()`, `dynamicModelName()` (u-blox CFG-NAVSPG-DYNMODEL; 1 is reserved).

**Files:** `core/GnssNames.h` (new), `core/Report.h`, `core/StatusLine.h`, `core/DeviceStatus.h`, `app/Diagnostics.{h,cpp}`, `app/Config.h`, `tests/test_gnssnames.cpp` (new), `tests/test_report.cpp`, `tests/test_statusline.cpp`.

**Tests:** every fix type (long and short) and out-of-range values; dynamic models including the reserved 1; the settings line's position and omission; `(not yet OK)`; unknown accuracy; `no epoch yet`; battery flag combinations, including the charger pin without USB not counting as charging.

**Checks:** host suites pass (plain and sanitized); `scripts/build.sh` passes for both boards with no project warnings.

**Verified 2026-10-09 (owner):** both boards compile, boot and work. **Checks were:** `D` after boot shows the settings line and the new fix line (`(not yet OK)` until the fix is good); on battery boards, the battery line's flags; the status line (`L`) shows `Fix: 3D`.

#### CPLX-3: shared BLE rules in one place. Done (bench verified)

**Finding confirmed:** `BleNrf.cpp` and `BleEsp32.cpp` repeated the MTU-refusal rule and its 1 s log, the send accounting (`sent` / `failed` / `maxSendUs`), and the connection-edge logging almost line for line. BUG-3 and CPLX-5 had each needed the same change made twice.

**Fix (owner approved after two simplification rounds):** a small `BleLink` member in `peripherals/ble/BleLink.h`, owned by each transport. It holds the shared state and rules: `admit(len, mtu)` (refuse rather than split, count `refused`, log once after 1 s of refusals, re-armed on each connect), `sent(ok, startUs)` (accounting), `edges(up)` (connect and disconnect log lines), `stats()` and `clearMaxSend()`. It logs through `Console` directly. Each transport keeps its stack calls, and its `send()` stays a short, readable sequence in its own file. No template, no inheritance. Loop task only, as before.
- **Owner's change:** the nRF no longer captures or logs the disconnect reason (the `disconnectReason` atomic is gone), matching the ESP32. Both now log `BLE: client disconnected`.
- `begin()`, `end()`, `read()`, the RX counters and the nRF battery service stay per transport. `BleStub` is unchanged.
- Considered on the way: a pure core helper (host-testable, but needing a "log is due" flag and more code) and a template base class (fewest lines, but a send can't be read in one file).

**Files:** `peripherals/ble/BleLink.h` (new), `peripherals/ble/BleNrf.{h,cpp}`, `peripherals/ble/BleEsp32.{h,cpp}`. Net 93 lines removed and 20 added in the transports, plus the new header.

**Checks:** host suites pass; `scripts/build.sh` passes for both boards with no project warnings. No host tests for `BleLink`, as before for this logic.

**Verified 2026-10-09 (owner):** both boards compile, boot and work. **Checks were:** boot, connect and stream: `failed` / `refused` about 11 / 13 at connect, then frozen; `max send` as before; `BLE: client connected` / `disconnected` logged, with no reason on the nRF.

#### CPLX-2: one IMU pipeline around small chip drivers. Done (bench verified)

**Finding confirmed:** `Mpu6050` and `Lsm6ds3` were two copies of one class: the same public interface, the same filter, health and 13-line trim setup in `begin()`, the same `poll()` body, `step()` helper and failure lines. Only bring-up, the sample read and `end()` differed.

**Fix (owner approved after one simplification round):**
- **`ImuDevice` is now one concrete class** (`peripherals/imu/ImuDevice.{h,cpp}`) holding the shared pipeline, with the board's chip driver as a member (`ImuChip`, chosen in `ImuDevice.h` as before). Plain member object, like `BleLink`: no template, no inheritance. App's interface is unchanged.
- **Each driver has three functions:** `bool begin(Console&, ImuSample& first)` (its own bring-up and ✅/❌ steps, returning the first good sample), `bool read(ImuSample&)` (physical units, remapped), `void end()` (the nRF powers its chip off; the ESP32 does nothing). The shared `imuStep()` log helper is in `ImuLog.h`.
- **The driver returns its first sample** (the LSM6DS3's bring-up already took one as its last check, the MPU-6050's comes after its 100 ms settle), so there's no extra read and no seeding logic in the pipeline. Seeding from the first poll was considered and dropped: it would leave a ~10 ms window "up" but reading zeros.
- **Behaviour:** unchanged except that (1) the two closing failure lines became one, `❌ IMU: continuing without it - IMU fields will read zero`, after the driver's own ❌ step; and (2) on the MPU-6050 a first read that fails right after a verified configuration now fails bring-up (logged `first sample read failed ❌`), matching the LSM6DS3, instead of coming up unseeded.
- `Mpu6050.cpp` stays unguarded: it's a portable `Wire` driver by design, and both linkers use `--gc-sections` (the nRF image contains no `Mpu6050` symbols).

**Files:** `peripherals/imu/ImuDevice.{h,cpp}` (`.cpp` new), `peripherals/imu/ImuLog.h` (new), `peripherals/imu/Mpu6050.{h,cpp}`, `peripherals/imu/Lsm6ds3.{h,cpp}`. 225 lines removed, 108 added in the existing files, plus the two new ones.

**Checks:** host suites pass; `scripts/build.sh` passes for both boards with no project warnings.

**Verified 2026-10-09 (owner):** both boards compile, boot and work. **Checks were:** boot IMU lines as before (`detected ✅`, `configured, 4g 500deg/s ✅`); D's IMU readings normal (about 1000 mG on Z when level); trim reaches `locked`.

### Phase 2: survive faults instead of hanging

| Finding | Planned fix |
|---|---|
| REL-1 | **Done (boot verified)**, see Restructure progress → "Phase 2 item 3". Also converts REL-3's nRF I2C hang into a reboot. |
| REL-2 | **Done**, see Restructure progress → "Phase 2, GNSS item 1" |
| REL-4 | **Done (bench verified)** with BUG-2, see Restructure progress → "Phase 3". A setting left wrong in RAM → Fault `GnssConfigFailed`; a lost PVT-enable ACK is a warning, and liveness catches a receiver that's configured but silent. |
| REL-7 | **Done (forced failure verified)**, see Restructure progress → "Phase 2 item 4". |
| PWR-1 | **Done (bench verified)**, see Restructure progress → "Phase 2 item 5". |
| CLK-1 | **Done (boot verified; battery run pending)**, see Restructure progress → "Phase 2 item 6". Also fixed the clock being used before it started. |

**Field finding (2026-10-05), input to REL-2 and REL-4: a back-fed GNSS receiver.**

With the receiver's primary feed (the TPS rail) disconnected, the nRF's UART TX line idling high back-feeds it through its RX pin. That's enough to run the UART and nothing else. The boot log shows:
- detection ✅ at 115200
- about a third of config reads succeed (✅ where flash already matched)
- every flash write fails ❌, and the configuration summary ❌ changes nothing (REL-4)
- PVT enable ✅ (a RAM/BBR write is still ACKed)
- the device enters Running with the LED showing advertising
- **zero epochs** over two minutes

**Consequence for the agreed liveness rule:** "no epoch for 2 s → restart" would loop forever in this case, because each boot re-detects the half-powered receiver.

**Proposed refinement, to decide in Phase 2:**
- *Epochs flowed, then stopped for 2 s* → Restart (mid-session loss, where a reboot and rail cycle can help).
- *No epoch ever since entering Running*, within a few seconds → Fault (never worked; restarting can't help).

Confirmed on battery (no USB): the LED shows the normal advertising blink, a phone connects, and no data arrives. Nothing tells the user the GNSS isn't working.

**Aftermath:** after these tests the receiver didn't answer at any baud, even with its feed restored. It recovered fully after sitting unpowered for a while. That fits either a back-feed latch-up or state held in BBR/backup decaying; the two can't be told apart from here. Either way it adds weight to removing the back-feed in hardware. It also shows that every boot's baud sweep with the rail down re-feeds a receiver through TX.

That allows at most one restart per failure. REL-4 ("configuration failed → Fault") would catch this case even earlier. The hardware fix for the back-feed itself (a series resistor on TX, or confirming the rail is up before opening the UART) is an owner/PCB decision.

### Phase 3: GNSS configuration verification

| Finding | Planned fix |
|---|---|
| BUG-2 | **Done (bench verified)**, see Restructure progress → "Phase 3". RAM checked per key, flash read only after a write; option C for REL-4. |
| CPLX-6 | Not needed: BUG-2 adds no reads on a normal boot, and the constellation swap went through without a NAK. |

### Phase 4: measure, then fix the structure

| Finding | Planned fix |
|---|---|
| PERF-1 (scheduler) | **Rescan declined** after the baseline: `gnss` max late 1.3 ms with no console activity, bounded at about 2 ms. `max late μs` column kept. See Restructure progress → "Phase 4, step 1". |
| Soak / HIL | Weak-link BLE soak on nRF: phone at range or shielded, `D` pressed periodically; check `missed`, `skipped` and the new BLE `failed` / `max send` counters |
| PERF-1 (BLE and UART) | Decided by the soak result: a deeper HVN queue, a separate BLE sender task, or EasyDMA UART ingestion |

### Phase 5: IMU robustness

| Finding | Planned fix |
|---|---|
| REL-6 | **Declined** (owner): the IMU stays down until reboot; GNSS unaffected. See Restructure progress → "Phase 5". |
| CPLX-2 | **Done (bench verified):** one `ImuDevice` pipeline around small chip drivers. See Restructure progress → "CPLX-2". |
| REL-3 | **ESP32 done (boot verified):** `Wire.setTimeOut(2)`. nRF bounded TWIM driver deferred; the REL-1 watchdog covers the hang. |

### Later, as a separate effort

- ~~D report GNSS lines, Battery line, and the GNSS settings line~~: **done**, see Restructure progress → "D-report wording".

---

## Open checks

Hardware checks that decisions above depend on.

| Check | Why it matters |
|---|---|
| **Battery run, then `D` (CLK-1):** about 10 minutes on battery with a fix, then plug in USB and type `D`. `epochs ÷ 20` should match `RT` to within about a second, `missed` near 0. Can be part of the weak-link BLE soak. | Confirms the crystal request works on battery, where USB isn't running it. |
| **REL-1 restart reasons and the D `Reset:` line:** e.g. unplug the GNSS mid-run → `Reset: software (GNSS: epochs stopped)`. | The retained restart reason is only bench-tested through the forced hang so far. |
| **System OFF test** (temporarily raise `kBatteryCutoffV` and the warning thresholds): confirm entry into System OFF, the low current, and that **plugging in USB wakes the nRF** into a normal boot (since REL-1, the boot's `Reset:` line should read `wake from System OFF (USB)`) | HW-1's decline relies on USB wake. If USB does *not* wake it, a charger could raise the cell towards 4.2 V with P0.14 still HIGH. Then HW-1 flips to "leave `VBAT_ENABLE` LOW in System OFF" (~2.3 µA). |

---

## Declined or deferred

| Finding | Decision | Reason |
|---|---|---|
| PERF-1 (console) / PERF-2 "spread `D` across ticks" | Declined for now | Measured at 3.9 ms with 0 missed epochs (see Corrections); 5.2 ms in the 2026-10-09 baseline, now that the report is longer. Bench-only: with no USB host (the car) `emit()` returns at once. Revisit if it disturbs bench measurements. |
| PERF-1 (scheduler rescan) | Declined (2026-10-09) | Baseline without console activity: `gnss` max late 1.3 ms, bounded at about 2 ms, against a 4.5 ms limit. See Phase 4, step 1. |
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
| CPLX-3 | Done (2026-10-09) | Shared BLE rules in `BleLink`. See Restructure progress → "CPLX-3". |
| CPLX-5 (rest) | Declined (2026-10-09) | The clock part is done (one timebase). Wrapping portable Arduino calls (`delay`, `Serial`) is declined: `platform/` holds what differs per board. See Restructure progress → "CPLX-5 (rest)". |
| CPLX-7, TEST-1 | Deferred | Later cleanup (TEST-3 done, see Restructure progress → "TEST-3") |
