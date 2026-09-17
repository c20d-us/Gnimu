# Gnimu Fourth-Pass Review

**Code scan · branch `gnimu_multiproto` · commit `ba4e497` · 16 Sep 2026**

A check of the R3 dispositions in `docs/code-review-remediation.md` against the
code, plus anything new the changes introduced.

| | |
|---|---|
| Variants compiled | 3 / 3, zero sketch warnings |
| Flash | ESP32 1,183,007 (90%) · nRF52840 191,500 · OLED 213,284 |
| Host harnesses | 5 / 5 pass (encoder, BLE, GNSS, IMU, telemetry) |
| `check_common.sh` | 25 all-variant + 8 nRF-shared files identical |
| New findings | 3 (1 low-medium, 2 low) |

Findings use `R4-*` IDs, distinct from earlier reviews' IDs and the remediation
record's `NEW-*`.

---

## The R3 dispositions

Every disposition is agreed, and the implemented fixes match the record.

- **R3-1** — `powerEnterDeepSleep()` checks `sd_softdevice_is_enabled()` and
  writes `NRF_POWER->SYSTEMOFF` directly otherwise, as the core's `systemOff()`
  does. Bench check 9 is still outstanding, and it is the most important one:
  no harness covers this path, and it is what protects a flat cell.
- **R3-2** — `peerMtu` starts at `kDefaultMtu` (23) with no
  `updatePeerMTU()`/`getPeerMTU()` pair, so the MTU check can now fire. The
  hardware capture shows it refusing the ~5 frames per fast reconnect that used
  to go to a 23-byte link as 88-byte notifies. Marking the three earlier "no
  refusal" results invalid was the right call.
- **R3-3** — the AssistNow Autonomous disable is restored; the GNSS goldens
  confirm it.
- **R3-4** — Low rather than Medium is the fairer severity. The goldens not
  moving showed the harness fake never exercised the library's double math, and
  deleting the fake's `calcAccel()`/`calcGyro()` makes a regression fail to
  build.
- **R3-5 declined** — fair. The display's timing is measured and meets its
  budget; the finding was an optimization, not a defect.
- **R3-6** — gating the boot wait on `powerUsbPresent()` is correct.
- **R3-7** — the rate is tried first and then skipped in the sweep, rather than
  deleted from the list. Deleting it, as the review proposed, would have made a
  receiver still saved at an old `GNSS_BAUD` unfindable after a change. Moving
  the `static_assert` beside the array is cleaner. **The short sweep wait is
  addressed by R4-1 below.**
- **R3-8, R3-9 items 1–5** — clean. Item 1 turned up the round's most valuable
  result: the old staleness check was never exercised, because the stale phase
  ran at 8 m/s where the speed gate already refused. The 35 s dead-receiver
  cruise now tests it.
- **R3-9 item 6 declined** — agreed. A `displayIsPresent()` stub on a board with
  no display, only to satisfy a comparison script, is not worth it.

---

## R4-1 — Low-medium — The 250 ms sweep wait may be too short at low baud rates

`src/Gnimu-nRF52840/g_gnss.cpp:59` (`kSweepMaxWaitMs`), `:128` (all three trees)

### Correction to R3-7

R3-7 recommended "a shorter `maxWait` (around 250 ms)", and said the library's
header calls 250 ms "enough otherwise". The remediation repeats that: "the value
the library's own header says is enough off SerialUSB". The header says less
than that:

```c
// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
```

It vouches for I2C only, and says nothing about a hardware UART.

### Why a UART needs more

`DevUBLOXGNSS::isConnected()` sends a `CFG-VALGET` for `UART1INPROT_UBX` and
waits up to `maxWait` for the ACK and the reply. The library tries three times.
The reply leaves through the receiver's transmit queue, behind whatever NMEA is
already waiting — and a receiver saved at a non-target rate is likely to still
have NMEA enabled.

- **At 4800 baud** the link carries about 480 bytes/s. A full factory NMEA set
  with many satellites in view can exceed that, so the transmit buffer backs up
  and a reply can wait an unbounded time or be dropped.
- **At 9600 baud** a once-a-second NMEA burst takes on the order of half a
  second to drain.

**No timeout covers the true worst case**, the default included: a link carrying
more than its capacity has no bounded reply time. The practical case is better,
because the sweep runs right after power-on. With few satellites tracked, NMEA
output is small — by estimate a few hundred bytes a second — and the three
attempts spread their waits across three NMEA cycles.

Estimated coverage at boot-time NMEA levels (not measured):

| Rate | 250 ms | 500 ms | 750 ms | Default (1,100 ms) |
|---|---|---|---|---|
| 9600 | marginal | comfortable | comfortable | comfortable |
| 4800 | likely misses | marginal | probably fine | best available |

A receiver saved at 4800 or 9600 is exactly the one the sweep exists to find. The
GNSS harness fake answers immediately, so it cannot see this.

### Recommendation: revert the sweep to the library default

Use the default `maxWait` for every attempt. This removes a constant and a
parameter rather than adding logic, and it cannot find fewer receivers than the
code before R3-7, which found them at every rate.

**`maxWait` is a ceiling, not a delay.** `begin()` returns as soon as the reply
arrives, so a longer value costs nothing on a rate where a receiver answers. It
only adds time on rates where nothing answers.

**Boot-time cost** (each failed rate: 3 × `maxWait` plus the sweep's 200 ms):

| Case | Now (250 ms sweep) | Default for all |
|---|---|---|
| Receiver at `GNSS_BAUD` | immediate | immediate |
| Factory M10 at 38400, first boot only | ~6.5 s | ~14 s |
| No receiver | ~11 s | ~32 s |

The factory case fails `GNSS_BAUD` first, then 4800, 9600 and 19200, before
answering at 38400. It happens once: the receiver is then saved at `GNSS_BAUD`.
The no-receiver case is a fault that already ends in "power-cycle to retry".

*These figures correct the answer given in conversation, which put the factory
first boot at ~10 s by leaving out the failed `GNSS_BAUD` attempt.*

The duplicate-rate skip, the membership `static_assert`, and the
`tryBaud()`/`switchToTargetBaud()` split all stay.

### What the reversion touches

- **`g_gnss.cpp`, all three trees:**
  - delete `kSweepMaxWaitMs` and its comment;
  - drop `tryBaud()`'s `maxWait` parameter and call
    `myGNSS.begin(*gnssStream)`;
  - the call sites become `tryBaud(GNSS_BAUD)` and `tryBaud(rate)`;
  - update the comment above `connectAndConfigureBaud()` that describes two
    different waits.
- **`test/gnss/gnss_harness.cpp`:**
  - `at-9600` asserts `maxWait=250` once — change to the default, or drop the
    `maxWait` check;
  - `absent` asserts `maxWait=250` eight times — change to "every attempt at the
    default", or drop it;
  - the four R3-7 mutations (first attempt shortened, sweep lengthened, 250 → 300)
    no longer apply;
  - the duplicate-skip assertions stay.
- **GNSS goldens:** re-save. The diff should be only the `maxWait=` values on
  sweep attempts.
- **`docs/code-review-remediation.md`:** the R3-7 entry, including its
  paraphrase of the library header, and the ~11 s no-receiver figure quoted in
  hardware check 11.

### Optional, data only

The factory first boot can stay quick without any logic: move 38400 to the front
of `kBaudRates`. It is the M10 factory rate, so a new module is found on the
second attempt — about 3.5 s, the failed `GNSS_BAUD` try plus the answer. The
list stays complete, so nothing else changes.

---

## R4-2 — Low — Every fast reconnect on the ESP32 now prints a drop warning

`src/Gnimu-nRF52840/g_ble.cpp:245` → `src/Gnimu-nRF52840/g_telemetry.cpp:244`
(all three trees)

A consequence of R3-2, working as designed. On a fast reconnect the central
subscribes before its MTU exchange, so frames sent in that window go through the
driver's MTU refusal. That path increments `droppedFrames` and not the
`unsubscribedFrames` subset, so the stats report counts them as failures:

```
❌ BLE: peer MTU 23 too small for a 88-byte frame (need 91). …
⚠️  BLE dropped 5 frame(s) this window (5 total)
✅ BLE: peer MTU now 517 - sending resumed.
```

This is the R2-4 pattern again: an expected, unavoidable refusal reported like a
fault, so a real congestion drop in the same window looks identical. R2-4
resolved it for pre-subscription refusals by counting them as a subset and
subtracting them from the drop line.

The remediation record treats counting these as deliberate ("those frames are now
refused and counted, which is what the check is for"), which is defensible — the
refusal line already explains them.

> **Option.** If the drop line starts to read as noise, count MTU refusals as a
> second expected subset and subtract it the same way. The ❌/✅ episode lines
> could report their own count, as the unsubscribed pair already does.

---

## R4-3 — Low — `g_log.h` doesn't state the new logging rule

`src/Gnimu-nRF52840/g_log.h:22-24`, `:50` (all three trees)

R3-9 item 5 made `LOG_PRINTF` a call to `logPrintf()`, so its arguments are now
evaluated even with no console attached. `LOG_PRINT` and `LOG_PRINTLN` are still
macros that skip their arguments when `Serial` is false. One header now holds two
evaluation rules.

- The header comment still says "At runtime every macro first checks Serial, so
  an unattached console skips formatting as well as writing." For `LOG_PRINTF`
  the formatting is still skipped, but the arguments are not.
- The rule R3-9 introduced — **no side effects in `LOG_PRINTF` arguments** — is
  written down only in the remediation record.

Every current call site was checked and complies. The risk is the next one.

> **Fix.** State the rule in `g_log.h`, beside the `LOG_PRINTF` definition, and
> adjust the "every macro" sentence. That is where someone adding a log call will
> look.

---

## What I ran

- **arduino-cli compile ×3** — `--warnings all`, using the IDE's
  `arduino-cli.yaml` for the relocated sketchbook. All succeed with zero sketch
  warnings.
- **Host harnesses** — all five runners, with `SPARKFUN_UBLOX_SRC` pointed at
  the relocated library. All pass, exit code 0.
- **`check_common.sh`** — green; coverage sweep and concurrency tripwire pass.
- **Source read** — the full firmware diff of `ba4e497`, and SparkFun
  `DevUBLOXGNSS::isConnected()` and the `kUBLOXGNSSDefaultMaxWait` comment (R4-1).
- **Not measured** — the NMEA byte rates and coverage table in R4-1 are
  estimates; nothing was flashed.

---

`R4-*` IDs are stable and safe to cite. No source file was modified.
