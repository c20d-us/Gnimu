# Code review remediation — working plan

Tracks how the findings in [`code-review-2026-09-09.md`](code-review-2026-09-09.md)
are being worked through. That document is the review as delivered and is not
edited; this one records what was decided and why.

Worked one change at a time, in batches, each described and agreed before any
code moved. Finding IDs (`IMU-1`, `ARC-8`, …) are the review's and are stable.

## Review closed — 2026-09-11

Every finding and architecture note has a disposition. Details are in the batch
sections below; this table is the index.

| ID | Disposition |
|---|---|
| IMU-1 | ✅ Fixed — batch 1 (decimation latched on the GNSS epoch) |
| IMU-2 | ✅ Fixed — batch 1 (parked sentinel, comments made true) |
| IMU-3 | ✅ Fixed — batch 5, option 2 (checked burst read, config read-back, zero-sample check) |
| IMU-4 | ✅ Fixed — batch 1 (NaN guard) |
| BLE-1, BLE-2, BLE-3 | ✅ Fixed — batch 2 |
| BLE-4 | ✅ As the review asked: recorded in a comment beside `lastLoggedMtu` during the ARC-1 audit - a race costs at most one duplicated or missed log line; deliberately not synchronised |
| LAT-1, LAT-2, LAT-3, LAT-5 | ✅ Fixed — batch 4 |
| LAT-4 | Declined — batch 4 (one adjacent ESP32 nit taken) |
| API-1, API-2, API-5 | ✅ Fixed — batch 3 |
| API-3 | ✅ Fixed — batch 2 |
| API-4 | ✅ Fixed — batch 5 (Nordic UART shape and UUIDs asserted at compile time) |
| API-6 | Deferred to phase H — partition scheme first, NimBLE only if needed |
| ROB-1 … ROB-5 | ✅ Fixed — batch 4 |
| ROB-6 | ✅ Fixed — parts 1 and 3 in batch 4, part 2 in batch 5 (telemetry harness) |
| SEC-1 | ✅ Fixed — batch 5 (documented; `onWrite` contract; "in use" now means subscribed) |
| SEC-2 | ✅ Nothing to fix — memory safety and untrusted input found clean. The forward guard is `onWrite`'s "EVERY WRITE IS UNTRUSTED" contract (SEC-1), which matters from the first protocol with a real command set |
| BLD-1 | ✅ Fixed — batch 4 |
| ARC-1 | ✅ Batch 5 — the concurrency model documented; the audit found and fixed a real ESP32 race |
| ARC-2 | ✅ Nothing to do — the three decisions it asks to protect (POD descriptor, compile-time selection, a dependency-free `g_protocol.h`) all still hold; API-4's check was written to keep the last one |
| ARC-3 | Declined — in favour of API-4's explicit check (see "Where the review was not followed") |
| ARC-4 | ✅ Batch 3 — the inbound contract (API-1, API-2, API-5 were its three symptoms) |
| ARC-5 | Not acted on — recorded in batch 3; revisit at phase F, where RaceChrono's CAN filter needs per-session reset |
| ARC-6 | Acknowledged, not acted on — premature at two protocols |
| ARC-7 | ✅ Batch 5 — phase F's evidence standard, `multiprotocol-design.md` §11.1 |
| ARC-8 | ✅ Batch 5 — `g_imu_tuning.h`, checked |
| ARC-9 | Declined — anchoring is right for exactly one timer, which has it (see "Where the review was not followed") |
| ARC-10 | ✅ A standing pattern, applied throughout: invariants moved out of prose into `static_assert`s, harnesses and `check_common.sh` |
| ARC-11 | ✅ Batch 5 |

**Still open, none of it review work:**

- **Checks that need hardware or the IDE:** a whole-build warnings pass
  (Arduino IDE → Compiler warnings: All) for each variant (batch 4); one real
  write down the inbound path, e.g. from nRF Connect (batch 3); the
  LIGHT_SLEEP-removal bench test. (The OLED SCL-to-GND freeze test was declined
  on 2026-09-11; see that deferred entry for what was recorded instead.)
- **Follow-ons found while remediating**, each described in its own deferred
  entry at the end of batch 5. All are now closed except **API-6** (decide at
  phase H) and **the TR-C bandwidth question** (needs the datasheet). The nRF
  watchdog and ESP32 IMU recovery were declined, with reasons recorded; the
  rest were done.

---

## Batch 1 — Sensor accuracy ✅ COMPLETE

Changes what the device measures, so the host harness cannot verify it — the
encoder is untouched throughout. Verification was by capture comparison
instead.

**Verified 2026-09-10 on all three variants.** nRF52840 and ESP32 were each
flashed and captured; both showed the latch firing per epoch (repeats occurring
only at near-zero motion, 0/13 and 0/2 above median motion level), correct
scale (ESP32 median |accel| exactly 1000 milli-g), and a clean encoder
self-check. The OLED was flashed and confirmed; it shares every changed file
byte-identically with the nRF52840 build already captured, and its one unique
module (`g_display`) does not read the IMU.

The two post-change captures were merged into the vector set, bringing it to
23,910 captured + 42 synthetic. They contributed negative `gZ` and saturated
gyro values no earlier capture held, and coverage now reports no gaps.

| Finding | Status |
|---|---|
| IMU-1 — decimation on its own clock | ✅ done |
| IMU-2 — transient thresholds vs their comments | ✅ done |
| IMU-3 — ESP32 has no failed-read guard | ⚠️ documented here; ✅ fixed in batch 5 (option 2) |
| IMU-4 — `toProtocolInt16()` passes NaN through | ✅ done |

### IMU-1 — as implemented

Decimation is now driven by the GNSS epoch rather than a `millis()` timer.
`imuLatchForEpoch()` holds the body that used to sit in `imuPoll()`, and
`telemetrySendIfReady()` calls it on epoch arrival, outside the
`bleIsConnected()` test.

Three decisions worth recording, because none of them were in the review's
suggested fix:

**No timer fallback.** The review proposed keeping the timer for when epochs
stop. It is not needed: no epoch means no packet, so there is nothing to drain.
The hazard the timer originally existed for — a stale peak accumulating across
a long BLE disconnect — is covered by the latch sitting outside the connected
test, since epochs arrive regardless of connection state.

Dropping it turned the change into a net deletion: the timer, its
`lastTransmitReadMs` static, and `IMU_TRANSMIT_INTERVAL_MS` in all three
`config.h` files are gone.

**The latch returns its value.** `imuLatchForEpoch()` hands back what it
latched and `buildSample()` takes it as a parameter, rather than reaching for
`imuReadProtocolUnits()` internally. That converts "latch before you encode"
from an ordering comment into a data dependency the compiler enforces — the
ARC-10 pattern applied to the change being made. `imuReadProtocolUnits()`
survives as the const observer accessor for the serial report.

**The `static_assert` was rewritten, not deleted.** It guarded against
`read()` being called more often than `update()`, which would silently degrade
transient tracking to a plain EMA. That hazard did not disappear, it moved: the
constraint is now between `GNSS_NAV_RATE_HZ` and `IMU_SAMPLE_INTERVAL_MS`, and
the assert checks that instead.

**What this does not fix.** A constant offset remains — the epoch describes an
instant already past by receiver output latency, UART transit, and up to one
poll interval. The change makes that offset constant and therefore
characterisable, rather than wandering across the full 0–50 ms range as the two
clocks beat. It is not zero. Fixing it properly needs a timestamped IMU ring
buffer selected against `iTOW`/`nano`, which is the wrong trade for a project
this attentive to loop determinism — and no RaceBox field expresses a differing
IMU timestamp anyway, so the app assumes coincidence regardless.

**Mirroring check.** `g_imu.cpp` differs legitimately between trees, so
`check_common.sh` cannot see a botched mirror — which is exactly how IMU-3
happened. After editing, the two `imuLatchForEpoch()` bodies were diffed
directly and confirmed to differ only in the unit-scale block. Worth repeating
for any edit to a legitimately-divergent file.

**Still to verify on hardware.** Neither the harness nor `check_common.sh`
compiles `g_imu.cpp` or `g_telemetry.cpp`. All three variants need building in
the IDE, then one capture compared against the five pre-change captures — not
to prove alignment, which nothing available can, but to catch values frozen or
magnitudes collapsed.

### IMU-2 — as implemented

The review's framing was partly wrong: the prose block above the defines
already said the blend was parked, so a careful reader was not misled about
*that*. But three real defects sat underneath it, and one was worse than
reported.

**The inline comments named values that were not there** — `99.0f // 1.5g` and
`9999.0f // 28.6deg/s`.

**The prose contradicted itself.** It called 1.5g "parked out of reach" eleven
lines after measuring the vibration floor at ~1.55g. 1.5 is *below* 1.55, so
that value never parked anything — which is presumably why it moved to `99.0f`
without the prose following. The document recorded a decision that had since
been found wrong and superseded, without saying so.

**The ESP32's park was fragile and unit-mismatched.** `99.0f` had been copied
across trees without converting units:

| | Constant | Parked at | Max reachable deviation | Margin |
|---|---|---|---|---|
| nRF | `..._THRESHOLD_G` | 99 g | ~8 g at ±4 g | ~12× |
| ESP32 | `..._THRESHOLD_MPS2` | 99 m/s² ≈ 10.1 g | ~78 m/s² at ±4 g | ~1.26× |

Behaviourally identical today, but widening `IMU_ACCEL_RANGE_G` to ±8 g would
have silently re-enabled the blend on ESP32 while leaving nRF parked. Worse,
its comment read `// ~1.5g`, and its prose named 14.7 m/s² as the holding
value — a number that is comfortably *reachable* on that board. Anyone
reconciling the comment with the value in either direction would have switched
the blend back on without noticing.

**Fix.** A named, unit-agnostic sentinel:

```c
#define IMU_TRANSIENT_PARKED 1.0e6f
```

This addresses all three at once: the intent moves from prose into a symbol,
the unit mismatch cannot recur because a sentinel has no units, and
unreachability stops depending on the configured range. The prose now records
*why* 1.5g was superseded rather than silently leaving the old reasoning in
place, and the ESP32 block carries an explicit warning about the 14.7 m/s²
trap. The commented-out `28.6f` line — dead config inviting an uninformed
uncomment — is gone.

**Behaviour: unchanged.** Unreachable before, unreachable after. Unlike IMU-1
this needs no capture of its own; it rides along with the next build. Harness
stayed at 23,028/23,028 throughout, as expected for a `config.h`-only change.

### IMU-3 — why the review's fix was not applied

**The suggested fix cannot work.** The review recommends mirroring the nRF's
`lastGood` guard "keyed off `getEvent()`'s return." `Adafruit_MPU6050.cpp:716`
ends `getEvent()` with a bare `return true;` regardless of what happened on the
bus. A guard keyed off it compiles, reads correctly, closes the finding — and
can never fire. That is worse than no guard, so it was not added.

The error is discarded a level lower too: `_read()` is `void`, and its
`data_reg.read(buffer, 14)` — the call that does report I2C failure — is
unchecked.

**The failure mode also differs from the one described.** `_read()`'s
`uint8_t buffer[14]` is uninitialised stack, so a failed transfer yields
*garbage*, not a stale repeat. The review's rationale (borrowed from the nRF
comment) is about repeats falsely satisfying `g_imu_trim`'s stillness gate;
garbage does the opposite — it inflates variance, rejecting a qualifying block
and delaying trim lock rather than corrupting one. Range-checking cannot catch
it either: raw values are `int16` scaled to the configured range, so nonsense
lands inside ±4 g like anything legitimate.

**Done now (option 1).** A comment at the ESP32 `readImuRaw()` recording all of
the above, so the next reader does not implement the useless guard. The hazard
remains. *(Closed in batch 5 - see "IMU-3 option 2 — as implemented".)*

**Deferred to batch 5 (option 2).** Real parity means doing the burst read here
through `Adafruit_BusIO_Register` with its result checked, plus owning the
raw→physical scaling that implies — roughly 25–30 lines. It belongs beside the
`g_imu.cpp` unification, which is already a deliberate look at this read path,
rather than bolted onto a batch-1 comment fix.

Weighed against waiting: one failed read costs a garbage sample at 9% EMA
weight and a possibly-delayed trim lock. Real, but modest — and option 2
rewrites a working sensor path on the board with the least IMU test history.

### IMU-4 — as implemented

A NaN guard at the top of `toProtocolInt16()`, returning 0.

**Not `isfinite()`, which the review offered as an alternative.** Infinities
are already handled correctly by the existing clamps — `+inf > 32767.0f` is
true, so an infinity saturates to the largest representable value. Routing them
to 0 instead would have been a regression. Only NaN slips through, because
every comparison against it is false.

**Confirmed defensive, not a bug fix.** NaN is not producible today: the sensor
path is integer-derived, and `g_imu_trim` guards both of its divisions
(`mag < 1e-6f`, `mn < 1e-6f`) and clamps `acosf`'s argument. The guard exists
because the function is the last thing between the filters and the wire and is
written as though it were total.

`0` is the least-bad substitute — the protocol has no "invalid" encoding for an
IMU field, so any value is a lie, and a consistent 0 at least reads as a
stuck-sensor fault rather than noise. Silent by design, matching the nRF
failed-read guard's reasoning about per-sample logging.

Verified on the host across NaN, ±inf, normal values and both saturation
directions.

---

## Batch 2 — Transport integrity

Originally scoped as one change; decomposed into four so each is independently
buildable, with API-3 first because the others need a channel to report through.

| Finding | Status |
|---|---|
| API-3 — `TelemetryEmit` cannot report failure | ✅ done |
| BLE-1 — nRF discards `bleuart.write()`'s result | ✅ done |
| BLE-2 — ESP32 never verifies the MTU it asked for | ✅ done |
| BLE-3 — nRF `bleEmitFrame()` ignores `channel` | ✅ done |
| BLE-4 — `lastLoggedMtu` written from two threads | ✅ commented, as asked (during ARC-1) |

### API-3 — as implemented

`TelemetryEmit` and `bleEmitFrame()` now return `bool`.

**The plan changed once, and IMU-3 is why.** The original intent was to land
the signature as pure plumbing, with the stacks returning a placeholder and
BLE-1 wiring up real detection afterwards. A placeholder means `return true;`
unconditionally — exactly the `getEvent()` anti-pattern documented three
changes earlier, where a bool that can never be false reads like a signal and
is worse than no bool at all. So the truthful return landed with the signature,
and BLE-1 reduces to counting and reporting.

**The two stacks detect different things, and the contract says so.** This came
out of reading the cores rather than assuming symmetry:

| | Can it observe a failed send? |
|---|---|
| nRF — `BLEUart::write()` returns `size_t` | **Yes** — returns 0 on an exhausted HVN queue, so `== len` is a real test |
| ESP32 — `BLECharacteristic::notify()` returns `void` | **No** — the stack cannot report it |

`true` therefore means *the transport accepted the frame*, never *the peer
received it* — BLE notify cannot tell anyone that. On nRF, `false` catches a
genuinely truncated frame. On ESP32 it catches what that stack can see: an
invalid channel now, an inadequate MTU once BLE-2 lands, which is that board's
actual failure mode. `g_protocol.h` documents the asymmetry explicitly, because
a protocol author would otherwise assume `false` means the same thing
everywhere.

**Incidental.** `setValue()` takes a `const` pointer as of esp32 core 3.x, so
`bleEmitFrame`'s `const_cast` is a no-op and its comment was wrong. The comment
was corrected and the cast kept — it costs nothing and keeps older cores,
where the parameter really was non-const, building.

**Behaviour: unchanged.** Nothing consumes the result yet.

**Verified.** Harness 23,952/23,952 (it compiles the encoder against the new
contract). Because the harness does *not* compile `g_ble.cpp`, a separate
host check confirmed `TelemetryEmit sink = bleEmitFrame;` compiles against each
tree's headers — the assignment `g_telemetry.cpp` actually performs.

### BLE-1 — as implemented

`bleDroppedFrames()` — a cumulative frame-drop counter owned by the transport,
incremented on `bleEmitFrame()`'s failure path.

**The counter lives in `g_ble`, not `g_telemetry`.** Telemetry hands
`bleEmitFrame` straight to `proto->encode()`, so it never sees individual frame
results and has no interposition point. The transport is also what knows why a
send failed, and this arrangement stays correct for a protocol emitting several
frames per sample.

**`BLE: xx.xxHz` now means complete packets, not attempts.** Telemetry brackets
the encode call with the drop counter:

```c
const uint32_t dropsBefore = bleDroppedFrames();
proto->encode(buildSample(*pvt, imu), bleEmitFrame);
if (bleDroppedFrames() == dropsBefore) bleSentPacketCount++;
```

That gives the familiar field the meaning readers already assume, without
changing `encode()`'s signature — an encoder emitting multiple frames knows to
stop on a `false`, and the transport has already recorded which failed, so a
return value there would add nothing.

**Drops get their own line, and LAT-3 is why.** The first design put a
`| drops: N` field on the stats line. `Print::printf` on the nRF core formats
into a 256-byte stack buffer, and that line already measures 239 bytes normally
and 257 at the no-fix sentinels — it clips silently today. Adding a field would
push the normal case toward the ceiling exactly when frames are being lost,
which is when the line most needs to survive. A separate `LOG_PRINTF`, emitted
only when the count moves, costs nothing in the normal case and avoids the
interaction entirely. LAT-3 remains worth fixing in batch 4 regardless.

Counted silently at the point of failure and reported once per stats window —
per-frame logging on the transmit path is the latency problem the IMU
failed-read guard already documents.

**Expect zero on ESP32 until BLE-2.** `notify()` returns `void` there, so the
only reachable failure is an invalid channel — a programming error. A quiet
counter on that board is not evidence that nothing is being dropped; it is
evidence the stack cannot see drops yet. The code says so at the counter.

**Verified.** Harness 23,952/23,952. The window arithmetic was simulated on the
host across four windows including a multi-frame loss: 76 complete packets from
80 epochs with 5 frames lost, deltas and totals correct, and silent windows
genuinely silent.

**Not demonstrable without inducing congestion.** The failure path has never
fired. Walking the device to the edge of BLE range mid-capture is the realistic
way to see it work. If drops turn out to be common at 20 Hz, that is a finding
in itself rather than a fault in this change.

### BLE-2 — as implemented

The review's claim was verified in the core source: `BLECharacteristic.cpp:877`
warns `"Truncating to N bytes"` at a debug level nobody enables and then hands
the controller the full length anyway, which cuts at MTU−3.

Three departures from the suggested fix, each found by stress-testing rather
than by reading the finding.

**Not `onMtuChanged`.** It fires on the Bluedroid BTC task; storing the MTU
there and reading it from `bleEmitFrame()` on the loop is the same cross-thread
non-atomic write BLE-4 flags as a defect on the nRF side. Following the review
here would have introduced the problem it complains about three findings later.

**No cache either, and this reversed an earlier decision.** The first design
polled `getPeerMTU()` in `bleUpdate()` "to avoid a semaphore call per frame at
20 Hz" — but `bleUpdate()` runs every loop iteration, upwards of 1000 Hz. The
cheaper-looking placement was fifty times more expensive. Polling at the point
of use also removed three problems the cache had created: staleness, a spurious
refusal on the first frame after every connection, and the need to invalidate
on disconnect.

**Threshold is per-frame, not `RACEBOX_PACKET_LEN + 3`.** Hardcoding RaceBox's
size would refuse RaceChrono's 20-byte GPS frames, which fit a default 23-byte
MTU perfectly well. `len + 3 > mtu` keeps the transport ignorant of which
protocol it serves — written in that form rather than `len > mtu - 3` because
both operands are unsigned and the subtraction underflows below mtu 3.

**`BLE_MTU_BYTES` was deleted rather than re-asserted.** The original question
was whether to check `>= 91` (RaceBox-specific) or `> 23` (protocol-agnostic).
Neither: it has exactly one correct class of value — large enough for the
protocol's biggest frame — so it was never a tunable, and having it in
`config.h` only created something to get wrong. It is now derived in
`g_ble.cpp` as `PROTOCOL_MAX_FRAME_LEN + 3`, tracking the protocol
automatically, with a single surviving `static_assert` against the 517-byte ATT
maximum. That removes the last protocol-specific number from `config.h`,
properly closing the phase-C deviation instead of relocating it.

Exact fit, no headroom: a margin would mask an off-by-one rather than expose
it, and an inadequate negotiated MTU is now caught loudly at send time.

**New contract requirement.** Every `g_proto_<name>.h` must define
`constexpr size_t PROTOCOL_MAX_FRAME_LEN`; `g_protocol.h` documents it and
`g_protocol_active.h` checks it. A constexpr rather than a `ProtocolDescriptor`
field because both uses — the `static_assert` and the derived MTU request — are
compile-time.

*Corrected after the first ESP32 build failed.* That check was originally an
`#ifndef PROTOCOL_MAX_FRAME_LEN` / `#error`, which can never work: the
preprocessor cannot see C++ declarations, so an `#ifndef` on a constexpr is
always true and rejected every build. It is now a `static_assert` referencing
the constant, which keeps the error in the file documenting the requirement
while also validating the value is positive. The `#ifndef` on
`TELEMETRY_PROTOCOL` beside it is correct — that one is a real macro.

**Logging is edge-triggered both ways**, once per episode, with the actual
numbers. The refusing→sending line matters: without it the console shows
"refusing" and then nothing, and a reader cannot tell recovery from a dead
device. The latch is cleared in `bleUpdate()` when the link drops — loop-side,
not in the disconnect callback — because without a per-connection reset a
second bad client after a good one would be silent.

**Verified on the host** across refusal, recovery, reconnect-with-a-new-bad-
client, the exact-fit boundary at MTU 91, one byte short at 90, and a
20-byte frame at MTU 23 (sends — the case a hardcoded threshold would break).
Drop accounting reconciled.

**Tested on ESP32 hardware 2026-09-10, and the test corrected a wrong claim.**

Setting `kRequestedMtu` to 23 was documented as simulating a client that
declines the MTU raise. It does not: the central drives the exchange, and an
iOS peer negotiated 517 regardless of what was requested. What the knob
actually exposes is the window *before* that exchange completes, where
`getPeerMTU()` still reports the 23-byte default — about 650 ms, 13 frames at
20 Hz.

That still exercised the full sequence end to end, which was worth having:

```
✅ BLE Client connected & MTU update requested
❌ BLE: peer MTU 23 too small for a 88-byte frame (need 91). Refusing to send
✅ BLE: peer MTU now 517 - sending resumed.
⚠️  BLE dropped 13 frame(s) this window (13 total)
```

Refusal, one-shot warning with real numbers, recovery on a live MTU change,
drop counting, and a clean 20.00 Hz afterwards — the recovery path in
particular being the half hardest to reach deliberately.

**With the derived value there is no refusal at connect** — verified on the
same hardware immediately afterwards. The check costs nothing in normal
operation and needs no grace-period handling. Worth settling with a second run
rather than redesigning on a reading of a test that turned out to measure
something else.

**The path the check exists for remains untested.** A central that genuinely
declines the raise cannot be simulated from firmware; it needs a BLE client
under our control. The comment at `kRequestedMtu` now says so rather than
giving instructions that do not work.

### BLE-3 — as implemented

A guard rejecting any channel other than `TELEMETRY_CHANNEL_PRIMARY` on the
Nordic UART transport, counted as a drop like any other refusal.

Correct before the change, since that transport has one outbound stream — but
a protocol emitting on the wrong index would have had its frame routed out the
Tx characteristic regardless: wrong data on a valid-looking stream, which is
the hardest class of bug to notice. `TELEMETRY_CHANNEL_NORDIC_RX` exists at
index 1, so the mistake is reachable.

The ESP32 path already validated `channel` against `channelCount`, so this only
applies to the nRF.

---

## New findings — discovered while remediating

Not from the review. Recorded with the same discipline so they do not get lost.

### NEW-1 — RESOLVED — Connect-time drops are the pre-subscription window

`g_ble.cpp bleEmitFrame()` · measured on hardware 2026-09-10

Found by BLE-1 on the first nRF run after it landed. Pre-existing: nothing in
the remediation changed `bleuart.write()`'s behaviour, only whether its return
value was read.

| Window | Epochs | Complete | Dropped |
|---|---|---|---|
| connect second | 20 | **0** | 20 |
| next second | 20 | 12 | 8 |
| thereafter | 20 | 20 | 0 |

**First diagnosis, now believed wrong.** Attributed to HVN queue exhaustion:
at the 23-byte default MTU an 88-byte frame needs five notifies, 100/second at
20 Hz, and `BLECharacteristic::notify()` bails *inside* its chunking loop when
the queue is empty.

**Re-diagnosed after an nRF Connect test.** That client connects without ever
writing the CCCD, and **every frame failed for the life of the connection** —
1,471 drops, `BLE: 0.00Hz` throughout. `BLECharacteristic.cpp:717` gates the
entire send loop on `notifyEnabled()` and falls through to `false` otherwise.

A client that has connected but not yet subscribed produces *exactly* the
pattern in the table above: the RaceBox app connects, negotiates MTU, and only
then writes the CCCD. That explains the connect-time losses at least as well as
congestion, and probably better. Both causes look identical as a short write,
so reading the code cannot settle it.

**Resolution: make the two distinguishable, then measure.** Both transports now
check subscription before attempting a send and log it distinctly, using the
same one-shot latch as BLE-2's MTU check:

```
❌ BLE: client connected but has not subscribed to notifications - nothing is being sent.
✅ BLE: notifications enabled - sending resumed.
```

That labelling is what makes the two distinguishable in future; the question
itself was settled by the toggle test below before the labels were flashed.

On ESP32 this is a pre-check rather than an explanation of a failure, since
`notify()` returns `void` and cannot report anything — there, an unsubscribed
client was previously silent in *both* directions.

**Verified on nRF hardware 2026-09-10.** One-shot ❌ on connecting without
subscribing, silent thereafter through 127 counted drops, ✅ on enabling
notifications, then eight seconds of clean 20 Hz with zero drops. Disabling
notifications *without disconnecting* fired the ❌ a second time — the latch
resets on the recovery edge, not only on disconnect, which is the behaviour
wanted and was not something the design explicitly called for. The
`bleStop()` path was exercised by the switch-off at the end of the run.

**Resolved 2026-09-10 by a notification-toggle test.** Decisive evidence, and
it came from the pre-check build rather than needing the new labels:

| Time | Event | Drops |
|---|---|---|
| 396 | client connects (MTU 23) | 10 |
| **397** | **MTU rises 23 → 247** | 20/s |
| 398–417 | still 20/s, `BLE: 0.00Hz` | 20/s |
| **418** | **notifications enabled** | 13, then clean |
| 419–428 | `BLE: 20.00Hz` | **zero** |
| 429 | notifications turned off | 15, then 20/s |

The MTU rose at t=397 and drops continued for twenty more seconds, stopping
only when the CCCD was written. Chunking-at-MTU-23 would have stopped at 397.
The first diagnosis was wrong.

Equally important: **once subscribed, zero drops** across ten clean seconds at
20 Hz. There is no congestion problem to chase, and the earlier RaceBox-app
pattern (20, then 8, then clean) is just that app taking ~1.4 s to subscribe
after connecting.

**No fix needed.** A client that has not subscribed cannot receive; that is BLE
working as specified. The subscription check repairs nothing — it stops the
condition being reported as an anonymous drop count. It also retires the worry
raised when this was logged, that two warning lines per connection would train
the reader to ignore them: they are now a labelled, explicable event.

---

## Batch 3 — Inbound contract ✅ COMPLETE

| Finding | Status |
|---|---|
| API-1 — `onWrite`'s execution context unspecified | ✅ done |
| API-2 — `onWrite`'s buffer lifetime unspecified | ✅ done |
| API-5 — nRF silently truncates writes past 64 bytes | ✅ done |

Three symptoms of one root (ARC-4): outbound got a design, inbound got a
function pointer. One change addressed all three — `g_ble` buffers writes and
dispatches them from `bleUpdate()`, which defines the thread, defines the
buffer, and makes oversize handling possible.

### The contract now states what it cannot promise

**Message boundaries are not guaranteed, and cannot be.** `TRANSPORT_GATT_CHANNELS`
delivers one client write per call; `TRANSPORT_NORDIC_UART` is a byte stream
where two writes can coalesce and one can split. Buffering makes the plumbing
look symmetric while the semantics stay different, so `g_protocol.h` says so
outright. ARC-4 gestures at this; the fix as described would have papered over it.

**`onWrite` runs on the loop — and inherits a deadline.** That is the point of
the change, but it converts a slow handler from a latency problem into a
correctness one: `Serial1`'s 64-byte ring against a 100-byte NAV-PVT means
`gnssPoll()` must be reached about every 5.5 ms or GNSS bytes are lost. The
contract states it at `onWrite`, which also puts LAT-2's constraint somewhere a
protocol author will actually see it — previously it lived only in
`g_display.cpp`, a file two of the three trees do not have.

### `std::atomic`, not `volatile`

The review's fix (buffer, dispatch from the loop) is right but insufficient as
stated. On ESP32 the Bluedroid task and the Arduino loop task run on
**different cores**, so a `volatile` flag gives no visibility ordering and the
consumer could see the index before the bytes it guards. Release/acquire on the
ring indices is the minimum that is actually correct; it costs nothing on the
single-core nRF and needs no FreeRTOS types.

This does not widen ARC-1's cooperative-polled model. BLE callbacks were
already the one breach, and this is the first data crossing it wider than a
single byte — exactly where `volatile` stops being adequate.

*Correction (ARC-1, 2026-09-10): "the first data crossing it" was wrong.* On
ESP32, `deviceConnected` was already publishing `connectTimeMs` from `onConnect`
to the loop — the exact flag-guards-data pattern this paragraph says `volatile`
cannot handle — and in the wrong order besides. It went unseen because nothing
listed what crosses the boundary. Fixed under ARC-1.

### Per-transport oversize policy

Uniformity would have been wrong once boundaries are known to differ:

| Transport | Policy | Why |
|---|---|---|
| Nordic UART (stream) | split across slots | boundaries were never promised |
| GATT channels (discrete) | drop the whole write, count it | a partial message that looks complete is API-5's failure |

### The ring size came from a measurement, after a test caught a real defect

First sized at 4 slots. The host simulation showed a 200-byte stream drain
delivering **192 of 200 bytes** — splitting worked, but three usable slots held
only 192 bytes and the tail was dropped, while the comment claimed "nothing is
dropped for size."

`BLE_UART_DEFAULT_FIFO_DEPTH` is 256, so one drain can yield four 64-byte
chunks. Resized to 8 slots (seven usable, 448 bytes), verified against both a
200- and a full 256-byte drain: all bytes delivered, none dropped. 512 bytes of
RAM on parts with 256 KB.

### Also

Logging moved off both callback threads — the nRF previously `LOG_PRINTF`d
every received byte from the SoftDevice callback. `bleDroppedWrites()` mirrors
`bleDroppedFrames()`, reported on the stats line with what it means: *commands
may have been lost*.

### Not included

**ARC-5** — no `begin`/`end`/`onConnect`/`onDisconnect` on the descriptor.
RaceChrono's CAN filter is per-session state that ought to reset when a client
drops, and there is nowhere to hook it. Real, adjacent, and an architecture
note rather than a finding.

### Build result

ESP32 and nRF52840 both compiled and flashed clean (2026-09-10), which settles
the one real unknown: `<atomic>` is available and lock-free for `uint8_t` on
both the Adafruit nRF52 and esp32 cores. No critical-section fallback needed.
The OLED variant shares every changed file with the nRF52840 build.

### Still unexercised at runtime

The inbound path has no client driving it. `raceboxOnWrite()` is a no-op and
the RaceBox app does not appear to write to the Rx characteristic, so the ring,
the dispatch and the drop counting are structurally correct and specified but
have never carried a real write.

A generic BLE tool (nRF Connect or similar) writing a few bytes to the Nordic
UART Rx characteristic would exercise the whole path and produce
`📨 BLE write: N byte(s) on channel 1` from the loop thread. Worth doing before
phase F depends on it, since RaceChrono's CAN-filter characteristic is the
first real consumer.

## Batch 4 — Timing and hygiene ✅ COMPLETE

LAT-1…5, ROB-1…6, BLD-1. Independent and mostly mechanical. ROB-1 was the one
with teeth: a runtime-reachable `while(1)` on a battery build. **ALL DONE: LAT-1…5, ROB-1…6, BLD-1, plus NEW-2.** LAT-4 declined; ROB-6
part 2 deferred to batch 5. **All three variants built, flashed and ran
(2026-09-10)**, which clears every compile-only risk in the batch: LAT-2's
`SERIAL_BUFFER_SIZE` assert, ROB-2's switch-margin assert, ROB-4's `#if`
restructuring, and the ESP32 `txRing` / `kGnssRxRingBytes` constants.
Still open: whether the builds are warning-free — ROB-4's reason for existing.

### ROB-1 — as implemented

**The defect.** `gnssBegin()` ended a failed probe with `while (1) delay(100);`
— an unconditional, runtime-reachable infinite loop on a battery-powered build.
On nRF that is worse than the review states: the same firmware also drives the
battery cut-off, so a device that halts there sits on a LiPo with nothing left
watching the pack voltage. The review framed it as "a dead device is confusing";
the real objection is that the one job that must survive every other failure was
the job being abandoned.

**What changed.**

- `gnssBegin()` returns `bool` instead of halting. The `while (1)` is gone.
- New `bool gnssIsUp()` — true after a successful begin, false on failure and
  after `gnssEnd()`. Backup mode counts as UP: the receiver is present and
  answers, it is simply asleep.
- `gnssPoll()`, `gnssSleep()`, `gnssWake()` return early when not up.
  `gnssEnd()` is deliberately *not* guarded, so the UART is released on the way
  to deep sleep regardless of how the receiver ended up down.
- All three `.ino` files call `(void)gnssBegin();` with a comment saying why the
  return is discarded: the loop must keep running for battery protection whether
  or not there is a receiver.
- The 1 Hz stats line collapses to `RT: %us | ❌ GNSS not responding` (plus the
  battery field where there is a gauge) when GNSS is down. Every other field on
  that line — fix type, satellites, rates, epoch age — describes a receiver that
  is not answering, so printing them at 1 Hz is noise that hides the one fact
  that matters. Battery is kept precisely to show the cut-off is still armed.

**Departures from the review.**

*No retry, no backoff, no absent-latch.* Three successive drafts of this fix
carried recovery machinery; all three came out. The receiver does not repair
itself, and the one plausible transient — someone unseats the connector on a
running device — is already answered by a power cycle. Cost without a retry is
bounded anyway: the discovery sweep is ~24 s across the standard baud rates
(the SparkFun `isConnected` path probes three times at the 1100 ms default),
paid exactly once at boot, and the state machine's own timers — the 30-minute
idle timeout, then the 180 minutes into LIGHT_SLEEP at which the GNSS rail is
cut (only a wake *after* that re-runs the probe) — mean a wake-probe-fail cycle
could recur at worst about once every 3.5 hours. *(Wording corrected under
SEC-1: this first said "the 180-minute LIGHT_SLEEP window". The window is 360
minutes; 180 is the rail-cut point. The 3.5-hour bound was right.)* Machinery to bound a cost
that existing timers already bound is machinery that can rot.

*`g_state.cpp` untouched.* An earlier draft short-circuited the sleep tiers when
GNSS was absent. It was dropped after asking what the stats line would report
during LIGHT_SLEEP phase 2: the answer is that with backup counted as UP, a
sleeping-but-present receiver reports normally and only a genuinely missing one
reports the fault. Nothing in the state machine needed to know.

**Verification.** Harness re-run: 23952/23952, as expected — the encoder is not
on this path. `check_common.sh` clean (13 all-variant + 11 nRF-shared files
identical). Brace balance and branch nesting in the rewritten stats report
checked by hand. All three variants need reflashing: `g_gnss.h`, `g_gnss.cpp`,
`g_telemetry.cpp` and the `.ino` changed in every tree.

**Open, optional.** The OLED variant could show the fault on-screen using
`gnssIsUp()`; without it, an OLED device with a dead receiver looks like one
that simply has not acquired.

### LAT-1 — as implemented

**The defect.** ESP32's `HardwareSerial` constructs with `_txBufferSize(0)`
(core 3.3.11, `HardwareSerial.cpp:142`). With no TX ring, `uart_write_bytes()`
blocks the loop task until the 128-byte hardware FIFO drains — about 20 ms for
the once-per-second stats line at 115200. Those are 20 ms of not calling
`gnssPoll()`.

**What changed.** One line plus its rationale in `Gnimu-ESP32.ino`, inside the
existing `#if LOG_ENABLED`, before `Serial.begin()`:

```c
Serial.setTxBufferSize(512);
```

**Scope, stated honestly: this is margin, not an active bug.** ESP32 has a
256-byte driver RX ring plus the 128-byte FIFO in front, against ~230 bytes
accumulating during the blocking write. It fits today, ~1.6x. The fix is worth
one line because that margin is the only thing between an attached console and a
corrupted NAV-PVT, but nothing observable should change. `GNSS: 20.00Hz` on the
stats line is the tell either way — a ring overflow would show as a rate dip,
since a corrupted message never becomes an epoch.

**Measured, correcting the review.** The line is **225 bytes normally, 236 at
the no-fix sentinels** — not the 239/257 the review reports. ROB-1 did not touch
the GNSS-up branch; the review simply counted generously. The conclusion is
unchanged.

**Departure: the review's second fix was not applied, because it is backwards.**
The review asks to "defer the report by one iteration so it can never share a
pass with an epoch." Two independent problems:

1. *It moves nothing.* Loop iterations are sub-millisecond. Deferring a 20 ms
   blocking write by one iteration relocates it by ~0.2 ms — 1% of one epoch.
   The harm is 20 ms of not draining the ring; where in the gap those 20 ms
   begin is what matters.
2. *Sharing the epoch's iteration is the SAFE phase.* NAV-PVT is 100 bytes
   (~8.7 ms of wire time) every 50 ms, leaving ~41 ms of silence. A print
   starting right after the epoch is consumed runs t=0…20 ms of that quiet
   window and finishes with 20 ms to spare. One starting at t=30 ms runs
   straight into the next message. The review would push the print away from
   the one moment with maximum slack.

"By construction" does not hold either: the report period is 1000 ms + overshoot
on `millis()`, the epoch period 50 ms on the *GNSS* oscillator. Two clocks
sharing a nominal ratio do not share a phase — the same argument that justified
phase-locking `imuLatchForEpoch()` in IMU-1.

**Why nRF gets nothing.** Not an asymmetry in the fix; the platforms differ.
`Serial` there is USB CDC: `Adafruit_USBD_CDC::write()` returns immediately when
`tud_cdc_n_connected()` is false, and when a host *is* attached the 236-byte line
fits the empty 256-byte FIFO in a single `tud_cdc_n_write()`, so the
`while (remain)` loop never takes a second pass and `yield()` is never reached.
There is nothing to set in any case: `CFG_TUD_CDC_TX_BUFSIZE` is a bare
`#define` in `tusb_config_nrf.h:66`, *not* `#ifndef`-guarded, and the class
exposes no runtime setter.

**Note for LAT-3.** nRF's `Print::printf` truncation at 256 bytes
(`Print.cpp:192`, `char buf[256]`) is currently what *guarantees* the CDC FIFO
can never block. Splitting the line into two `LOG_PRINTF` calls lifts that
guarantee (two ~130-byte writes still fit, so it is safe — but the reason it is
safe changes). Dropping the redundant `tAcc` field keeps the guarantee intact
and is the better of the review's two options.

**Note for LAT-2.** `SERIAL_BUFFER_SIZE 64` (`RingBuffer.h:29`) *is*
`#ifndef`-guarded, so `-DSERIAL_BUFFER_SIZE=256` genuinely works — and that is
the buffer with a real deficit: at 115200 a 64-byte ring fills in 5.55 ms while
a 100-byte NAV-PVT takes 8.68 ms to arrive, so the ring cannot hold one message.
But there is no `boards.local.txt` or `platform.local.txt` anywhere in this repo,
and a correctness-critical flag living outside the sketch builds the fragile way
on every machine that lacks it — a trap for anyone following the README. That
argues for LAT-2's *first* suggestion (document the constraint in `g_gnss.h`)
with the `-D` recorded as optional margin, not as the fix. Also note
`SERIAL_BUFFER_SIZE` sizes `txBuffer[]` too (`Uart.h:60`), so the cost is ~384
bytes per Uart instance, not 192.

**Verification.** `check_common.sh` clean — the `.ino` files are per-variant and
outside the checked set, so only the ESP32 tree changed. No host build available
(`arduino-cli` is not installed); ESP32 needs a compile-and-flash. nRF and OLED
are untouched by this finding.

**Not done, offered.** `setTxBufferSize()` returns 0 and only `log_e`s when
called in the wrong order, so a future edit that moves it after `begin()` would
silently restore the stall. Capturing the return and warning after `begin()`
would make that un-rottable, in the ARC-10 spirit. Left out as beyond the
one-line change that was approved.

### NEW-2 — ESP32 boot log loses its first lines (found during LAT-1)

Reported from hardware while testing LAT-1: garbage at the head of the ESP32
console, then several startup lines missing entirely. **One cause, confirmed by
the fix** — see the correction at the end of this entry.

**The missing lines are ours, and the bug is in the guard written to prevent
them.**

```c
Serial.begin(115200);
uint32_t t0 = millis();
while (!Serial && millis() - t0 < 3000) { }
```

`HardwareSerial::operator bool()` is `return uartIsDriverInstalled(_uart);`
(core 3.3.11, `HardwareSerial.cpp:594`) — true the instant `begin()` returns.
`!Serial` is therefore false on the first check and the loop waits **zero**. The
old comment acknowledged the truthiness but called it harmless; it is not. A
UART bridge discards everything written before the host opens the port, so the
banner and both GNSS baud-detection lines were being thrown away.

The observed cutoff matches: the module is already at `GNSS_BAUD`, which
`connectAndConfigureBaud()` tries first, so it connects on iteration 0 after
~100 ms of `delay()` plus the probe, and the console picked up mid-stream at the
first config command — a loss window of ~200–400 ms.

**Fix.** Replace the no-op poll with `delay(700)`, `LOG_ENABLED` only. Chosen as
a *replacement* rather than an addition: the condition that cannot be false
goes, the comment becomes true, and the line count is unchanged. There is no way
to observe host attachment on a UART bridge, so a flat cost is the honest
option. It does not cover the Arduino IDE's post-upload monitor reconnect —
recorded in the comment so a future report of "still missing after upload" is
not mistaken for a regression.

Also moved the LAT-1 `txRing == 0` warning to after the delay, so the guard
cannot itself be lost to the race it now documents.

**nRF deliberately keeps the poll.** `Adafruit_USBD_CDC::operator bool()`
returns `tud_cdc_n_connected(_instance)` — real host attachment — and yields
explicitly to support `while (!Serial) {}`. There the guard both works and
returns as soon as the monitor is up, so a flat delay would be strictly worse.
The divergence is recorded in the ESP32 comment.

**Correction — the garbage had the same cause, and an earlier reading of it was
wrong.** This entry first attributed the garbage to the ESP32 ROM bootloader
printing at a crystal-derived baud (74880 on a 26 MHz part) before any user code
runs, and called it permanent and cosmetic. Hardware disproved that: after the
`delay(700)` the garbage was gone. A sketch-side delay cannot affect output
emitted before the sketch exists, so the garbage was never ROM output — it was
our own early bytes.

The unified explanation is simpler and better evidenced. Bytes written while the
host is still configuring the port do not all vanish; some arrive with framing
errors instead. So the same few-hundred-millisecond window produced *both*
symptoms — the lost lines and the mush — and moving all output past it fixed
both at once. The `-D`-free 74880 monitor test is no longer needed and has been
withdrawn.

Worth keeping as a method note: the two-causes reading was plausible and wrong,
and the thing that settled it was a fix whose scope did not match the
hypothesis. When a change fixes more than it was aimed at, the surplus is
evidence the model is wrong, not a bonus.

**Verification.** Built, flashed and confirmed on hardware: the banner and both
`🔎`/`✅` detection lines survive, and the garbage is gone. `check_common.sh`
clean; the `.ino` files are per-variant and outside the checked set, so only the
ESP32 tree changed. No orphaned `t0`.

### LAT-2 — as implemented

**The review's premise was false by the time it was acted on.** It claims the
UART deadline "is written down only in the header comment of the OLED variant's
display module." It is not in `g_display.cpp` at all, and it is stated in four
places — most importantly `g_protocol.h:257`, which is in the **checked common
set**, so it is byte-identical in every tree. That is already the strongest
placement available. Also in `g_ble.cpp` (both stacks), the OLED `config.h`
slice rationale, and the nRF README's `GNSS_BAUD` row.

In fairness: the `g_protocol.h` and `g_ble.cpp` statements are ours, added in
batches 2–3 *after* the review was written. The review was accurate when
authored and was overtaken by our own work.

**The real gap, which is narrower and sharper.** `g_gnss.h` — the module that
owns the UART — said nothing, in any of the three trees. Zero hits. That is
exactly where someone modifying GNSS code looks.

**Two mechanism details the review missed**, found by reading the core:

- **Usable capacity is 63 bytes, not 64.** `RingBuffer` detects "full" as
  `head+1 == tail`, permanently reserving one slot. So the window is **5.47ms**,
  not 5.5 — and every prior statement of the number in this repo is very
  slightly optimistic.
- **Overflow is silent.** `RingBuffer::store_char()` drops the byte and does not
  advance the head — no flag, no counter, no error. Nothing in this firmware can
  observe it directly. It surfaces only as a checksum failure costing an epoch,
  and one step further out as a GNSS rate below `GNSS_NAV_RATE_HZ`.

The RX path is also confirmed as a one-byte EasyDMA buffer with one ENDRX
interrupt per byte (`Uart.cpp:195`), not a DMA'd block — so the software ring is
genuinely the whole budget.

**What changed.** Documentation plus one compile-time check; no behaviour change
anywhere.

- Both nRF trees (`g_gnss.h` is nRF-shared, byte-identical): the constraint
  written out with the corrected arithmetic, the silent-overflow fact, the
  observable signature, and the note that the OLED's `DISPLAY_SLICES_PER_EPOCH`
  and `g_protocol.h`'s onWrite contract are both consequences of it.
- `static_assert(SERIAL_BUFFER_SIZE < 100, ...)`. ARC-10: the numbers are
  hostage to a core we do not control, and a Seeeduino bump would silently
  falsify four files' worth of comments. Deliberately `< 100` rather than
  `== 64` — it permits a larger ring, it just refuses to let one arrive
  unnoticed. Verified to fire in both directions on the host (compiles at 64,
  fails at 256).
- ESP32 `g_gnss.h`: the same constraint with its real margin — `_rxBufferSize`
  is 256 plus a 128-byte FIFO, ~22ms against an 8.68ms message, so that ring
  holds a whole NAV-PVT and `gnssPoll()` need only be reached between messages.
  Records why there is no matching assert (`SERIAL_BUFFER_SIZE` is an nRF core
  macro; `_rxBufferSize` is a private runtime member, not a constant
  expression), and that `setRxBufferSize()` is the honest lever if room is ever
  wanted — as `tools/common/gnss_otp_clock` already does at 1024. Also ties the
  22ms figure to LAT-1: a ~20ms blocking console write spends most of it.

**Departure: `-DSERIAL_BUFFER_SIZE=256` declined, on stronger grounds than
reproducibility.** The earlier objection was that a correctness-critical flag
living outside the sketch builds the fragile way on any machine lacking it. The
better objection is that **it is an ODR hazard**: the macro sizes a *class
member* (`RingBuffer::_aucBuffer`, hence `sizeof(Uart)`), so a flag reaching the
sketch's translation units but not the prebuilt core archive leaves the two
disagreeing about object layout — memory corruption, not a build error. Not
proven to happen with `boards.local.txt`, and `build.extra_flags` generally does
reach core compilation; but "generally" is not a basis for a flag whose failure
mode is silent corruption, bought to widen a margin the code already meets.

**Not done, offered.** `gnssPoll()` could timestamp with `micros()` and warn when
the gap since the previous call exceeded the deadline, turning "rate looks low"
into "loop stalled 12.3ms, GNSS bytes may have been lost." Diagnosis rather than
defence, and `LOG_ENABLED`-only. Left out as runtime machinery.

**Verification.** `check_common.sh` clean (13 + 11 identical). The assert
verified on the host in both directions. All three variants need a compile to
confirm — nothing to observe at runtime, since nothing changed.

### LAT-3 — as implemented

**Corrected in both directions.** The review both overstates how often this
happens and badly understates what happens when it does.

*Overstated frequency.* Measured against the real format string, the line was
216 bytes in a normal fix and 226 with the accuracy sentinels — not the 239/257
claimed. The specific case the review calls already-broken had 29 bytes spare.

*Understated severity, and this is the part that mattered.* The review says it
"fails safely (`vsnprintf` is bounded, so it clips rather than overflows)."
It does not clip:

```c
char buf[256];
len = vsnprintf(buf, 256, format, ap);
this->write(buf, len);          // len is the WOULD-BE length
```

`vsnprintf` returns the length it *would* have written, so past 255 bytes
`write()` reads beyond a 256-byte **stack** buffer and transmits adjacent stack
memory to the console — undefined behaviour and a memory disclosure, not a
cosmetic truncation. A second latent path: `vsnprintf` returns negative on
encoding error and `write()` takes `size_t`.

nRF-only — ESP32's `Print::vprintf` checks `len < 0` and reallocates correctly.
The asymmetry bites the wrong way round, though: both nRF variants have
`BATTERY_HAS_GAUGE 1`, so the **longest** line runs on the **only** platform
that over-reads. A reachable combination (no fix + last-known position + hard
cornering + charging) measured 257 bytes.

**Departure: `tAcc` was NOT dropped.** That was the review's fix and my first
proposal. Rejected on use: tAcc converging is how you see the receiver settle on
the available SVs, which no other field reports.

**What replaced it, and it is better.** Render a value as `-` once it is past
the range in which it carries information. A **threshold, not a comparison
against `0xFFFFFFFF`** — the receiver is not contractually bound to any
sentinel, and anything past the bound is uninformative however it got there.
The insight is that the widest values are precisely the least informative ones:
`4294967295` spends ten characters saying "no data".

The design constraint this surfaced is worth keeping: **the threshold sets the
field width, so it is a width budget, not a plausibility check.** Rendering the
sentinel as `-` does nothing for a value of `999999`. Early drafts using a
6-digit cap still measured 256 — one byte over.

**Label shortening bought the margin back.** `tAcc`→`tA`, `hAcc`→`hA`,
`milliG`→`mG`, `centiDeg/s`→`c°/s`, `deg`→`°`, rates to one decimal, and
`" | "`→`"|"`. The separators alone were 24 bytes — twelve at three bytes each,
by far the largest single saving and the one that cost nothing. That paid for a
*more* generous 999999 bound than the tight version, and still left 39 spare.

**Declined shortenings, recorded so they are not re-proposed.**

- *Bare `40,-9,927` triples (12 bytes).* The axis-mapping procedure in six doc
  locations works by reading this line and identifying which axis is which.
  `X=40 Y=-9 Z=927` is self-describing; the bare triple assumes the very
  convention that procedure exists to establish. The OLED README already warns
  that Monitor "has been wrong about exactly this before, masking a mirrored
  axis map" — the ground-truth readout should stay explicit.
- *Merged `Hz: 0.0/21.0` (13 bytes).* BLE below GNSS is the drop indicator;
  making the two rates positional invites the one misreading that matters.
- *`F:`/`T:`/`B:` labels (8 bytes).* `B:` sits four fields from `BLE:`.
- *`c°` instead of `c°/s` (4 bytes).* Drops the time base; a rotation rate
  without a denominator is a different quantity.
- *Emoji for labels.* Non-ASCII costs 2–4 bytes in UTF-8, so `🛰` (4) does not
  beat `SV: ` (4). The emoji already present are the efficient case — 3 bytes
  each replacing words.

**Two independent defences, deliberately not merged.** Bounded field widths make
the maximum arithmetic; a bounded *write* makes the ceiling survivable when that
arithmetic eventually goes stale. `telemetrySerialReport()` now formats with
`snprintf` into its own buffer and prints the result, so `Print::printf` never
sees a long format — an over-long line becomes a **clipped** line, which is what
the review mistakenly believed already happened. The second defence is what
stops the first being load-bearing for someone adding a field.

**Verification.** A host test compiles the helpers and the gauge format string
**extracted verbatim from the shipping file** rather than retyped, so the thing
measured is the thing that ships. Result: **216 bytes at all-fields-type-maxima
against the 255 ceiling, 39 spare** — matching the prediction exactly — clean
under `-Wall`, with the `-` paths confirmed for out-of-bound tAcc/hAcc, invalid
lat/lon, and NaN (the `!(v >= -l && v <= l)` form is chosen over `fabs()` so NaN
takes the `-` road rather than printing "nan"). `check_common.sh` clean, harness
23952/23952 (no packet or protocol change). Four short fixed-width `LOG_PRINTF`
calls remain — the ROB-1 GNSS-down branch and the two drop reports — all far
from the ceiling.

**Doc churn.** `milliG`→`mG` updated in all 8 references across 6 files
(three `config.h`, two READMEs, `imu_tiltmap.ino`); none remain.

**Note for batch 5 / SEC-1.** This was a memory disclosure in a logging path. It
needs physical serial access so practical risk is near zero, but it belongs in
that conversation rather than being closed silently here.

### LAT-4 — DECLINED (with one adjacent nit taken)

The review asks to cache the LED triple and to add a `batteryPercent()`
accessor so `bleUpdate()` stops copying a struct to read one byte. Both
premises are true. Both fixes cost more than they save.

**Scope is narrower than stated.** The LED half is nRF-only — there is no
`g_led.cpp` in the ESP32 tree at all; ESP32 drives one LED from `updateLed()`
inside `g_ble.cpp`, already timer-gated on the blink path.

**The battery half is mostly already done.** `BatteryStatus` is **12 bytes, not
16** (measured on the host: `float` + `uint8_t` + four `bool`, padded).
`batteryGetStatus()` is literally `{ return status; }` — a cached struct, no
sampling. And the notify is *already* change-gated by
`if (pct != lastBasPercent)`, so the review's "needless notify every loop"
concern does not exist. Only the copy remains, and a second accessor for a
datum that already has one is a divergence surface bought with 12 bytes.

**Sizing against the constraint that actually matters.** `ledUpdate()` runs
before `gnssPoll()` in the same iteration, so it does count against LAT-2's
5.47ms drain deadline — at roughly 1µs, about **0.02%** of it. The review's
"Low" rating is right.

**The LED cache would introduce a bug, which is the real reason to decline.**
`g_power.cpp`'s `ledPinsOff()` writes the pins **directly, bypassing
`setLed()`** — documented as used "before g_led has initialized, and as the
final word before DEEP_SLEEP's permanent halt." The DEEP_SLEEP use is harmless.
The boot use is not: with a cache initialised to "all off", `ledBegin()`'s
`setLed(false, false, false)` would **hit the cache and skip the writes**.
`pinMode(OUTPUT)` leaves an nRF pin LOW, and this LED is active-low, so LOW is
*on* — the LED would be stuck on from boot.

Fixable with a sentinel "unknown" initial state or a forced first write, but the
mitigation is larger than the optimisation, guards a hazard that does not exist
today, and lands in two `g_led.cpp` files that already diverge (the OLED tree
adds a `displayIsPresent()` early return).

**Taken instead: the ESP32 LED toggle.** `updateLed()` used
`digitalWrite(pin, !digitalRead(pin))` — reading an output pin back to decide
what to drive it to. That works here (GPIO2 is a plain output and reads back its
driven level), so this is robustness, not a bug fix: read-back does not reflect
the driven value on every pin configuration, and the LED's state belonged in us
rather than in the pin. Replaced with a `static bool`, updated on **both**
paths — a cache written on only one path is precisely the drift that made the
nRF version a bad trade. ESP32 `g_ble.cpp` is in the nRF-shared list only, so
this is a standalone edit; `check_common.sh` clean.

### LAT-5 — as implemented

**What changed.** `Gnimu-ESP32/g_gnss.cpp`: `gnssSerial.setRxBufferSize()` at
`kGnssRxRingBytes` (512), called **once before the sweep loop**, with the return
captured. Plus a framing correction in ESP32's `g_gnss.h`.

**The review's number is better than the one LAT-2 wrote, and that is the real
content of this finding.** LAT-2's ESP32 comment said the 256-byte ring is
"~22ms of wire time." The review says "about 100ms of headroom." Both are
arithmetically correct and they answer different questions:

- **22ms** is how long the ring takes to fill *if bytes arrive back-to-back at
  line rate*. They do not — a NAV-PVT is a 8.68ms burst every 50ms.
- **~128ms** is how long the loop may stall before losing bytes, which is the
  question anyone actually has. 20Hz x 100 bytes = 2000 bytes/sec, so
  256 / 2000 = 128ms. Confirmed 20Hz with NAV-PVT only (NMEA disabled,
  `setAutoPVTcallbackPtr` the sole source).

**The distinction worth keeping: these are different KINDS of constraint.** The
nRF ring cannot hold one message, so the MCU must drain *during* every
transmission — a **phase-sensitive deadline**, where a 30ms stall in the quiet
gap is free and a 6ms stall overlapping a message costs an epoch. The ESP32 ring
holds 2.5 messages, so phase stops mattering and what remains is a **duration
budget**. Quoting "5.47ms vs 22ms" makes them look like one constraint at two
scales. They are not, and the ESP32 comment has been rewritten to say so — it
now explicitly warns against the wire-time framing for this variant while
affirming it for the nRF.

The nRF comment needed no change: it already said "gnssPoll() must be reached
repeatedly WHILE a message is arriving, not merely once between messages,"
which is the phase-sensitive model stated correctly.

**One call covers the whole sweep** — verified in the core: `end()` clears
`_uart` but leaves `_rxBufferSize` untouched, so the size survives every
`end()`/`begin()` cycle in the sweep and the re-`begin()` after a baud switch.

**Return captured, as in LAT-1.** `setRxBufferSize()` has the identical silent
failure — `if (_uart) { log_e(...); return 0; }`. The guard matters *more* here
than for the TX ring: `begin()` sits inside the sweep loop, so an edit that
slid the call downward would quietly leave the stock 256 bytes.

**Constant, not a config knob.** `kGnssRxRingBytes` is file-local `constexpr`,
following the `kStatsMeasMax` precedent from LAT-3: nothing about a board or a
wiring choice changes it, so it is an implementation budget, not a tunable.

**Honest scope: this fixes nothing.** Since LAT-1 removed the ~20ms blocking
console write, no stall in the ESP32 loop is within an order of magnitude of
128ms. Taken anyway because the cost profile is the opposite of LAT-4's: no new
state, nothing to keep in sync, no failure mode beyond the ordering the guard
now catches. That difference — not the performance — is the argument.

**Nothing on the nRF trees, verified rather than assumed.** The core's `Uart`
class exposes no buffer API at all (`begin`/`end`/`available`/`peek`/`read`/
`flush`/`write` and nothing else); `RingBuffer rxBuffer` is a fixed-size member
sized by the compile-time macro. The only route is the `-D` that LAT-2 declined
on ODR grounds. The nRF's answer to this constraint is architectural and already
built: epoch-phase-locked display slices, the onWrite contract, batch 3's SPSC
ring, and LAT-2's documentation.

**Recorded, not proposed: `GNSS_BAUD` is the one real nRF lever.** Lowering it
widens the drain window (57600 gives 10.9ms instead of 5.47ms) but also doubles
the fraction of each epoch the UART is vulnerable — 35% against 17%. A net gain
for a reliably fast loop, a net loss for one with occasional long stalls. Left
alone; revisit only if rate sag is ever observed. The rate table in `config.h`
already covers the tradeoff.

**Verification.** `check_common.sh` clean — `g_gnss.cpp`/`g_gnss.h` are
nRF-shared, so the ESP32 copies are standalone and the nRF trees are untouched.
Needs an ESP32 compile; nothing observable at runtime.

### ROB-2 — as implemented (the header was wrong, not the code)

**Confirmed.** `switchReadOnce()` is one bare `analogRead()`. No dummy read, no
burst, no averaging — only the ~50ms throttle was real. The header promised all
of it.

**Which one changes comes down to margin, and the margin is decisive.** The
510k/510k divider reads ~0mV with the switch ON and >=1675mV OFF (the worst
case: half the cell at the 3.35V end of `BATTERY_DISCHARGE_CURVE`; ~2100mV when
full), against an 800mV threshold. That is **800mV and 875mV of headroom** — a
ghost would have to shift the reading by ~28% of the 3000mV reference to flip
the decision, against real SAADC settling error of tens of millivolts at most.

**The high-Z case the ghost concern is about is already handled properly, just
not the way the comment said.** `powerBegin()` calls
`analogSampleTime(SAADC_TACQ_US)` at 40us explicitly "for the high-Z divider",
`config.h` documents the ~255k source-impedance reasoning, and there is already
a `static_assert` validating TACQ is a core-supported value. Settling is bought
with acquisition time, not with discarded reads.

**One point the review missed, which strengthens its concern before margin kills
it.** `BATTERY_POLL_INTERVAL_MS` (250) and `POWER_SWITCH_POLL_INTERVAL_MS` (50)
are exactly commensurate and both run off `millis()`, so the two channel reads
CAN phase-lock. A systematic offset would not be averaged away by
`STATE_SWITCH_OFF_DEBOUNCE_MS`, which only covers uncorrelated single bad reads.
That is the strongest form of the argument — and it is why **margin, not the
debounce, is the load-bearing part**. The corrected header says so.

**A smaller tell the comment was aspirational:** it described an "averaged
burst", but this firmware's actual ADC idiom — the battery sampler — uses the
*peak* of a paced burst, not an average. The technique it described is not used
anywhere here.

**What changed.**

- `g_power.h` in both nRF trees: the mitigation claim replaced with what is true
  and why it suffices — the three things that actually carry it (margin, TACQ,
  debounce), in that order of importance, plus the phase-locking note.
- `config.h` in both nRF trees: `POWER_SWITCH_ON_MV` / `POWER_SWITCH_OFF_MV_MIN`
  and a `static_assert` requiring >=500mV margin on **both** sides.

**Why the assert, and not just a better comment.** This finding *is* a comment
that drifted from its code. Fixing it with more prose leaves it free to drift
again. The assert makes the justification a build-time fact: move the divider or
the threshold and the build stops, pointing at the note that has to be re-made.
`POWER_SWITCH_OFF_MV_MIN` is deliberately keyed to the discharge floor rather
than the full-charge figure, so it asserts the worst case.

**A divergence closed on the way past.** `g_power.h` is NOT in the checked
nRF-shared set (only `g_power.cpp` is), and the two copies had drifted: the
comment hardcoded `A4` in one tree and `A1` in the other. The replacement refers
to `POWER_SWITCH_SENSE_PIN` instead, so that half is now identical. One
divergence remains at the module-overview line, which still names the pin —
a genuine per-board fact. Worth noting that `g_power.h` is one parameterised
comment away from joining the checked set; not done here.

**nRF-only.** ESP32 has no power-switch module at all (zero `POWER_SWITCH`
references in its `config.h`).

**Verification.** Assert verified on the host in both directions — compiles at
the shipping 800mV threshold, fails at 1300mV. Old claim confirmed gone from
both trees. `check_common.sh` clean. Both nRF variants need a compile; nothing
observable at runtime.

### ROB-3 — as implemented (deleted)

**Confirmed.** `displayWake()` existed twice — the real implementation and a
`DISPLAY_ENABLED == 0` stub — was declared in `g_display.h`, and had no callers
anywhere. Both `displaySleep()` sites (`enterDeepSleepFrom()` and the boot
classifier's low-voltage path) run `powerEnterDeepSleep()` immediately after,
and System OFF does not return, so `asleep` was write-once-true.

**Checked what the review did not: there IS a live wake path in this firmware.**
`LIGHT_SLEEP -> RUNNING` at `g_state.cpp:138` cold-starts the GNSS back up. It
needs no display wake because **the panel is not slept in LIGHT_SLEEP** —
`g_display.cpp:407` records that LIGHT_SLEEP, CHARGE_ONLY and BATTERY_WAIT all
have a screen to draw. Display sleep is exclusively a shutdown action. That
closed the one hole that would have argued for keeping the function.

**Deleted rather than documented-as-unused, and the reason is stronger than
tidiness: this code had never executed.** `oled.setPowerSave(0)` on this panel,
the forced `lastRenderMs = 0` repaint, its interaction with the `pushCursor = -1`
that `displaySleep()` sets — none of it had ever run on hardware. Leaving it
annotated keeps unverified code in a public header looking like working API,
which is a worse trap than its absence. If a wake path appears, it is four lines
written against a real requirement instead of a guess at one.

**`asleep` stays.** It still guards `displayUpdate()` (`g_display.cpp:379`), so
only its *reset* was unreachable, not the flag. Verified still referenced at the
declaration, `displayBegin()`, the `displayUpdate()` guard, and `displaySleep()`.

**`displaySleep()`'s header note now explains the asymmetry** rather than leaving
it looking like an oversight: the operation is terminal, both callers end in
System OFF, and the blank is needed *because* the panel's 3V3 rail survives
System OFF — without it the panel would sit lit on a stale frame until the cell
ran down.

**OLED-only.** No other variant has a display module and `g_display.*` is in no
checked set. Braces balanced (52/52), `check_common.sh` clean, and the stub
region's "Stubs matching g_display.h exactly" claim is true again.

### ROB-4 — as implemented

**Confirmed.** `static void enterChargeOnly()` was defined unguarded while its
only call site sat inside `#if STATE_CHARGE_ONLY_ON_USB`, which is `0` in both
nRF configs — a defined-but-unreferenced `static`, i.e. `-Wunused-function`.

**Fixed as the review asked:** the definition and its comment block wrapped in
the same flag, so all three CHARGE_ONLY guards now match and grep together.
Nesting verified balanced in both trees; they are the only conditionals in the
file.

**`#if` rather than `[[maybe_unused]]`.** Both silence it, but the `#if` ties
the code's existence to the flag that controls it and does not permanently
suppress the signal on the day the function goes unused for some other reason.

**The tempting broader fix is wrong, and that is the useful part of this
entry.** `case STATE_CHARGE_ONLY:` in `stateUpdate()` is equally unreachable at
`0`, so guarding it too looks like the consistent move. It is not: the switch
has **five cases and no `default:`**, so it is exhaustive over `SystemState`,
and that exhaustiveness is what keeps `-Wswitch` quiet. Dropping a case would
trade `-Wunused-function` for `-Wswitch`. Case labels raise no unused warning,
so there was never anything to fix there. Recorded in the source comment beside
the guard so the asymmetry reads as deliberate.

**More of CHARGE_ONLY compiles in than the finding implies, correctly.** The
state handler (`g_state.cpp:239`) and the OLED's `g_display.cpp:314` case are
both unguarded and should stay so — `STATE_CHARGE_ONLY` is a valid enum value
and the whole state is reachable the moment the flag is set to 1. Only the two
*entry points* are gated, and only one of them is a function.

**Guarding orphans nothing** — `enterChargeOnly()` calls `bleStop()`,
`gnssEnd()`, `powerHoldPeripheralsOff()` and `LOG_FLUSH()`, all also used by
`enterDeepSleepFrom()` and `enterBatteryWait()`, so no cascade of new warnings.

**nRF-only.** ESP32 has no CHARGE_ONLY state at all. `g_state.cpp` is
per-variant, so both nRF trees got the same edit at different lines.

**Unverified claim, worth checking on the next build.** The review calls this
"the only warning either nRF build emits". The mechanism is confirmed but the
"only" is not — there is no host compile here. Worth a **verbose** nRF build
after this lands, because the value of this fix is not the warning itself, it is
reaching zero so the next one is visible. If others were hiding behind it, that
is a more useful finding than this one.

### ROB-5 — as implemented (three sites, not two)

**The review found two mismatches; there were three.** It caught the nRF
`gnssPoll()` doc claiming the function "sets the appropriate navigation
frequency based on the current state". ESP32's copy makes a **different** false
claim — "based on **BLE connection state**" — which the review did not mention
and which is the more dangerous of the two: it is specific enough to sound like
a deliberate power optimisation someone might go looking for, or preserve.

**Neither is true.** `setNavigationFrequency()` is called exactly once per tree,
inside `gnssBegin()`, at the fixed `GNSS_NAV_RATE_HZ` (`g_gnss.cpp:272` nRF,
`:280` ESP32). Nothing varies it at runtime. `gnssPoll()` calls `checkUblox()`
and `checkCallbacks()`, plus ROB-1's early return when the receiver is down.

**Fixed by asserting the negative, not just deleting the claim.** The new doc
says explicitly that the rate is fixed in `gnssBegin()` and that neither system
state nor BLE connection varies it — *because two separate versions of this
header have now invented a dynamic rate*. Deleting the sentence would have left
the next person free to invent it a third time.

**The BATTERY_WAIT log now reports what it observed.** `if (!switchOn)` logged
"switch off, USB in" without reading `usb`. The claim is sound — the slide
switch is 3-pole and takes the cell physically out of circuit, so an MCU
executing at all with the switch off must be running on USB — but nothing in the
branch established it. Now:

```c
LOG_PRINTF("Boot -> BATTERY_WAIT (switch off, USB %s).\n",
           usb ? "in" : "absent");
```

Preferred over deleting "USB in", which loses real information, and over a
comment asserting it cannot be absent, which is another unchecked assertion of
exactly the kind this finding is about. Printing the value costs nothing, and an
"absent" would mean something genuinely surprising about the hardware rather
than passing unnoticed.

**Incidental check on ROB-4.** That change wrapped the `if (usb)` CHARGE_ONLY
branch in `#if STATE_CHARGE_ONLY_ON_USB`, so `usb` could have become unused at
flag `0`. It has not — `if (!usb && peak < BATTERY_CUTOFF_V)` sits outside all
three guards. No unused-variable warning was introduced.

**Scope.** `g_gnss.h` in all three trees (nRF-shared pair plus ESP32's own);
`g_state.cpp` in both nRF trees (per-variant — ESP32 has no `g_state` module at
all). `check_common.sh` clean. No behaviour change except the one log line.

### ROB-6 — parts 1 and 3 implemented, part 2 deferred to batch 5

**Part 1 — `check_common.sh` inverted.** The allowlists cannot catch what nobody
adds to them, so a coverage sweep now enumerates every `.h`/`.cpp` present in two
or more trees, subtracts `COMMON_FILES`, `NRF_COMMON_FILES` and a new
`EXCLUDED_FILES`, and fails on the remainder.

**`config.h` is the proof the sweep was needed.** Run by hand first, the
inversion flagged four files: the three documented exclusions **plus `config.h`**,
which sits in all three trees and appeared on no list at all — purely because it
is so obviously per-variant that nobody thought to write it down. That is the
argument in one example: the exclusions you reasoned about are not the risk, the
ones too obvious to mention are. Nothing was unchecked-but-identical, so this
found no live drift; it is a guard against the next shared file someone forgets.

`EXCLUDED_FILES` entries each carry a reason, and the failure text says so
explicitly — adding a file there to silence the sweep, without a reason, defeats
the sweep.

**Verified the check can actually fail.** A check that never fires is worthless,
so the sweep was tested end to end: dropping a `zz_probe.h` into both nRF trees
produced `❌ UNLISTED: zz_probe.h (in 2+ trees, on no list)` and **exit 1**;
removing it returned exit 0. Probe files confirmed gone afterwards.

**Part 3 — `g_power.h` joined the checked set.** nRF-shared coverage goes 11 ->
12 files.

**Order mattered here, and nearly cost a fact.** The exclusion comment in
`check_common.sh` was the repo's ONLY record of *why* the pin differs — "A4 IS
SDA there". The OLED's `config.h` did not say it; its comment was byte-identical
to the base tree's and simply read `A1`. Deleting the exclusion would have
deleted the reason. So: the rationale moved into the OLED `config.h` beside the
`#define` **first**, where someone changing the pin will actually see it — better
placement than a build script regardless — then the last divergent comment line
was reworded to name `POWER_SWITCH_SENSE_PIN`, then the file was added to
`NRF_COMMON_FILES`. (The other half of this was already done in ROB-2.)

**Part 2 — harness coverage of `buildSample()` — DEFERRED to batch 5** (since
done, differently: see "ROB-6 part 2 — as implemented" in batch 5).

*A finding that de-risks it: the real struct works on the host.* The review
suggests "a small host stub" for `UBX_NAV_PVT_data_t`. None is needed —
`u-blox_structs.h` compiles standalone with nothing but `<cstdint>`, verified:
`sizeof(UBX_NAV_PVT_data_t) == 92` (exactly the NAV-PVT payload) and the
`valid.bits.*` / `flags.bits.*` unions work. The harness can therefore test
against the **real shipping struct**. That matters: a hand-written stub would
only prove the stub matches our copy of it.

*Why it is not a tooling tweak.* `buildSample()` is `static` inside
`g_telemetry.cpp`, which cannot compile on the host — it pulls `config.h`,
`g_battery.h`, `g_ble.h`, `g_imu.h` and more. Making it testable needs three
things: extracting it into its own translation unit (a new file pair in all
three trees, and into the checked set); removing its hidden `batteryGetStatus()`
call in favour of a parameter; and deciding where `ImuProtocolUnits` lives, since
it is declared in `g_imu.h`, which includes `<Arduino.h>`.

*The battery parameter is independently right, and the code already argues it.*
`buildSample()`'s own comment explains that the IMU is taken as a parameter so
that ordering is "a data dependency the compiler enforces rather than an ordering
comment a later edit could reorder past". Battery is the one left behind.

*Deferred for sequencing, not avoidance.* The `ImuProtocolUnits` question is the
same one batch 5's `g_imu.cpp` unification and ARC-8 must answer. Doing part 2
first would move that struct, then very likely move it again.

### BLD-1 — as implemented

**Half of it was already fixed, by LAT-3.** The review cites four warnings
across three lines. Two were at `g_telemetry.cpp:189` — `%u` against `uint32_t`
`tAcc` and `hAcc` — and LAT-3 replaced that line entirely: `fmtMeas()` now does
`snprintf(buf, n, "%u%s", (unsigned int)v, unit)`. Not deliberate; it fell out
of the threshold-rendering work.

**Two remained**, both `%d` against `uint32_t testBaud` in ESP32's `g_gnss.cpp`.
Fixed with `%u` plus an explicit `(unsigned int)` cast.

**The nRF copy had the same defect and was fixed too.** There `uint32_t` is
`unsigned int`, so it is only a signedness mismatch and not in `-Wall` — but a
shared file that is correct only because of one ABI is exactly the trap this
finding names.

*Correction (SEC-1): the `unsigned int` claim above — the review's, repeated
here — is wrong.* On the nRF's arm-none-eabi toolchain `uint32_t` is
`long unsigned int`, exactly as on xtensa (verified: `__UINT32_TYPE__`). The nRF
never warned for a different reason: the nRF core's `Print::printf` carries no
format attribute, so no nRF log line had ever been format-checked at all. The
casts were right regardless. Since SEC-1 routes `LOG_PRINTF` through
`snprintf`, the nRF builds are now format-checked too. `g_gnss.cpp` is nRF-shared, so the fix lands in both nRF trees.

**Departure: casts, not `PRIu32`.** The review recommends `PRIu32`. The
codebase had already answered this about eight times with `(unsigned int)`
casts, added during batches 2-3 (`g_ble.cpp:121/432/440`, the `g_telemetry.cpp`
drop reports, the RT field, `fmtMeas`). So the real choice was not
cast-vs-`PRIu32` at these sites, it was **keep one idiom or introduce a second**.
Casts keep format strings readable and leave one way of doing this in the tree.

*The honest weakness,* recorded rather than glossed: `(unsigned int)` would
truncate where `int` is 16-bit, and `PRIu32` would not. Both targets are 32-bit
and such a port would break far more than this, but that is the real argument on
the other side. Converting all ~10 sites to `PRIu32` remains available and is
the only *consistent* way to adopt it.

**A third line made uniform beyond the finding.** `"Switching GNSS to target %d
baud"` passes `GNSS_BAUD`, an `int` literal, so `%d` was correct. It is now
`%u` with a cast anyway: three adjacent lines describing the same quantity,
formatted two ways, invites someone to "fix" the wrong one — and if `GNSS_BAUD`
ever became a `uint32_t` constant instead of a macro, `%d` would silently become
the same bug just removed.

**Left alone, deliberately.** Three sites pass `uint8_t`/`uint16_t` to `%u`
(ESP32 `g_ble.cpp:296`, nRF `g_ble.cpp:169` and `:372`). Those promote to `int`
in varargs and do not warn under `-Wall`, so they are not BLD-1 and churning
shared files for them is not worth it.

**Verification.** `check_common.sh` clean including the new coverage sweep;
harness 23952/23952. No behaviour change — the output was already correct, since
both types are 32 bits wide on both targets.

### NEW-3 — GNSS rate aliased into false 19/21 readings (found on hardware)

Reported from the ESP32 after the ARC-1 build: the 1Hz line cycling
19.0 / 20.0 / 21.0 Hz, alongside a weak fix. Two separate things.

**The rate cycling was a measurement artifact, and it undermined LAT-2.** In a
300s log, excluding the first window, **20 windows read 19.0 and exactly 20
read 21.0** — perfectly balanced, so not one epoch was lost. The window was
timed on the MCU's clock while epochs arrive on the receiver's; an epoch landing
near a boundary counted in whichever window jitter chose. Because the window
restarts from `now`, any late close carries forward, so the boundary sweeps
across the 50ms epoch spacing and the pairs arrive in clusters.

It was probably always there. **LAT-3 made it visible**: at two decimals every
window wobbled in the hundredths and an aliased pair hid among them; at one
decimal a normal window reads exactly 20.0 and every pair stands out. It
mattered because LAT-2 documents "a GNSS rate below 20 with no other
explanation" as the only signature of silent UART overflow — this produced that
signature with nothing wrong.

**Fixed by measuring the rate epoch to epoch.** The window still closes on the
clock, so the report cadence is unchanged; the span divided into runs from the
last epoch of the previous window to the last epoch of this one. Every gap
between epochs is counted exactly once. The epoch that opens a span is its edge
and is not counted (still encoded and sent — only the counting differs); a
window with no epochs reports 0 and lets the next epoch open a fresh span.
BLE is counted on the same epochs over the same span.

**Decided by simulation, which overturned two proposals of mine.** A host model
(50ms epochs on a drifting receiver clock, UART and pickup jitter, loop passes
with occasional 1-2.5ms stalls, five crystal errors x 300s) compared three
designs:

| | clock window (was) | close on epoch, 975ms | **epoch to epoch (shipped)** |
|---|---|---|---|
| false 19.0 / 21.0 in 1500 windows | 40 | 0 | **0** |
| ±0.1 wobble | 2 | 4.4% | 4.4% |
| 81 single lost epochs, every position | 81 found | 81 | **81** |
| false alarms, no loss | 23 | 0 | **0** |
| RT skips / repeats | 0 | 23 | **0** |
| first window | 20.0 | 20.8 | **20.0** |
| 3s outage | 20, 19, 0, 0, 1, 20 | 20, 17.3, 0, 0, 7, 20 | **20, 20, 0, 0, 0, 20** |

- The first proposal, `(count-1)/(tLast-tFirst)` within each window, was
  dropped before simulating: the gap *across* a boundary belongs to neither
  window, so a loss there would be invisible.
- The second, closing the window on the first epoch after 1000ms, fixed the
  rate but lengthened ~half the windows to 21 epochs (RT skips); the 975ms
  refinement fixed that but, by locking reports to epochs, moved the aliasing
  into the RT counter on unlucky boots (23 in one run). Epoch-to-epoch keeps
  reports on the clock, so neither problem can arise, and needs no grace period
  or `gnssIsUp()` special case.

**The shipping code was verified against the model, not just the model.**
`updateRates()` and the epoch bookkeeping were extracted verbatim from
`g_telemetry.cpp`, compiled on the host, and fed the same event streams as the
simulation — no loss, all 81 single losses, a dropped BLE frame, an outage,
GNSS down. **26,371 readings matched, largest difference 3.4e-5 Hz** (float vs
double). Clean under `-Wall -Wextra`.

**Residual, stated honestly:** about 1 window in 25 reads 19.9 or 20.1 — pickup
jitter at each epoch edge. A lost epoch reads 19.0 or lower, so the two cannot
be confused. The receiver's own `iTOW` would make the span exact, but it is not
trustworthy until the receiver has resolved time; not worth it unless the tenth
bothers anyone on hardware.

**Scope:** `g_telemetry.cpp` and `g_telemetry.h`, all-variant shared — all three
trees. The OLED panel's rate (`g_display.cpp:236`) reads the same value and
steadies with it. The simulation script lives outside the repo; the table above
is its record.

**The weak fix: conditions, not firmware.** 3 SVs and a 2D fix after five
minutes. Nothing firmware-side can explain it: no receiver-facing call added or
removed in `g_gnss.cpp` since the last commit, nothing that resets the receiver
anywhere, and the only `config.h` changes are IMU thresholds — BLE TX power, nav
rate, constellations and elevation mask unchanged, every config line in the boot
log succeeding with the same values. The log reads as a healthy receiver on a
cold start under a poor sky: pre-fix positions wandering hundreds of km (receiver
guessing while Fix is 0), first fix at 28s, hAcc converging steadily from ~800m
to ~5.7m, tAcc settling 3.4µs to 1.0µs, no epochs lost. The same unit had a 3D
fix with 5 SVs within a second earlier the same day — a hot start. Suggested
confirmations: reset without unplugging (receiver keeps its state), the nRF unit
in the same spot, or a window/outdoor position.

## Batch 5 — Decisions and structure

ARC-7 (phase F's evidence standard — gates phase F), ARC-8 (migrate the shared
`IMU_TRIM_*` constants into the checked set), ARC-11, SEC-1, ARC-1.

**Done: ARC-11, ARC-1, SEC-1, ARC-7, plus NEW-3; the `g_imu.cpp` unification
(steps A, A2, B), ARC-8, ROB-6 part 2, IMU-3 option 2, the removal of
LIGHT_SLEEP** (which replaced the planned collapse), **and API-4** (decided in
batch 2's review response but never implemented until now). Remaining: the
deferred items at the end of this batch.

### ARC-11 — as implemented

**Right about the sentence, wrong about the cause.** Both READMEs said "a
shared library doesn't fit the Arduino sketch build model" — refutable, since
Arduino supports a sketchbook `libraries/` folder. The review substituted
"distribution friction". But the library was not rejected for friction: it was
**built** (`GnimuCore`) and rejected because Arduino compiles a library's
sources without the sketch folder on the include path, so a library cannot see
the sketch's `config.h`. Every config-coupled module needed an implementation
header plus a per-sketch shim. Friction is real, but secondary.

**Quantified.** 8 of the 25 shared files include `config.h`, and they are
exactly the modules with behaviour — GNSS, BLE, IMU, battery, power, telemetry,
logging, protocol selection. The other 17 are config-free by design (encoder,
`ImuAxis`, trim, UBX helpers), which is what lets the harness compile the
encoder on a host. The new README text pre-empts the obvious follow-up — "then
put those 17 in a library" — with the answer: two sharing mechanisms would be
worse than one, and the script would still be needed for the rest.

**The larger finding: the real reason was recorded nowhere in the repo.**
`GnimuCore` appears in no commit — built and rejected without ever being
committed — and no doc held the reasoning. The only copy was in an assistant's
session memory, invisible to any reader or collaborator. Same shape as ROB-6's
"A4 is SDA": the sole record of a decision sitting where nobody changing the code
would look. And it produced exactly the failure the review demonstrated — read
the vague claim, check it, find `libraries/` works, conclude the duplication is
an unexamined habit.

**What changed.**

- `src/README.md` now carries the durable record: the include-path reason, the
  built-and-rejected library, the secondary friction cost, why the config-free
  modules are not split out, and "revisit only with a way around the
  include-path problem".
- `README.md` states the reason in one accurate clause and links to it.

**Two stale claims ROB-6 should have fixed, and did not.** Both READMEs still
told readers that a new shared file goes unchecked until listed — the root one
as "two things go unchecked if you forget them", `src/README.md` as "a new
shared file is not covered until you add it". ROB-6's coverage sweep made both
false. Corrected here; an earlier grep missed the second because of its
phrasing.

**And the last hole closed: a variant sweep.** The one remaining "you have to
remember" warning was a new sketch folder missing from `VARIANTS`, which would
leave its copies compared against nothing. `check_common.sh` now finds sketch
folders by the Arduino IDE's own rule — a folder holding a same-named `.ino` —
rather than by the `Gnimu-` prefix, so `tools/` is skipped naturally and an
oddly-named variant cannot slip past. Only `VARIANTS` is enforced: whether a new
folder also belongs in `NRF_VARIANTS` depends on its MCU, which the script
cannot judge, and the header now says so.

Verified with real exit codes, three cases: clean tree exits 0; an unlisted
`Zz-Probe/Zz-Probe.ino` fails with `❌ UNLISTED VARIANT` and exit 1; a folder
holding a *differently*-named `.ino` is correctly ignored (exit 0). Probes
confirmed removed.

**No firmware touched; no build needed.**

### ARC-1 — as implemented (and the audit found a race)

**Stale in both directions.** The review counted three `volatile bool`s; there
were four (ESP32 also has `oldDeviceConnected`) plus batch 3's `std::atomic`
ring indices in all three trees. Its sharpest concern — the plug-in contract
routing protocol authors into the off-loop breach via `onWrite` — was already
closed by batch 3's SPSC ring. And LAT-2 had written down the latency half.

**What remained true:** the *concurrency* half was stated nowhere at system
level — `docs/architecture-runtime.md` had no mention of it at all.

**Writing the invariant down meant listing what crosses, and that found a real
race.** ESP32's `onConnect()` did:

```c
deviceConnected = true;                       // loop, on the other core, sees this...
pServer->updatePeerMTU(pServer->getConnId(), kRequestedMtu);
connectTimeMs = millis();                     // ...before this
```

while the loop checks `deviceConnected && (millis() - connectTimeMs >
BLE_CONNECT_SETTLE_MS)`. With the Bluedroid task and the loop on different cores
in parallel, the loop could see "connected" beside the previous connection's
timestamp — or 0 on first connect — and **skip the 100ms settle window
entirely**. Not a memory-model subtlety: plain program order, with a stack call
widening the gap. Consequence was graceful — BLE-2's MTU check would refuse the
early frames — but it defeated exactly what the settle window is for.

**It also contradicted batch 3's own entry**, which called the ring "the first
data crossing it wider than a single byte". Corrected in place, above.

**What changed.**

1. **ESP32 fix.** Timestamp stored *before* the flag; `deviceConnected` is now
   `std::atomic<bool>`, release-stored in the callbacks and acquire-loaded in
   `bleIsConnected()`. The `&&` short-circuits, so `connectTimeMs` is only ever
   read after a load has seen the flag that published it — never while a
   callback could still be writing it. Every usage form host-checked under
   `-Wall -Wextra`; the remaining implicit uses are sequentially-consistent
   loads, stronger than needed and correct. **ESP32 only** — nRF's flag
   publishes nothing else, so a lone `volatile` is adequate there, and the
   source says why rather than changing it for uniformity.
2. **The invariant, written down.** A "Concurrency" section in
   `docs/architecture-runtime.md`: the cooperative-polled model; loop latency as
   a correctness property with each MCU's *kind* of budget; the one exception
   and its execution context per stack; four rules for callback code; and an
   inventory of everything that crosses with a verdict on each. "Adding to a
   callback means adding a row here." Pointer comments above the callbacks in
   every `g_ble.cpp`, where someone adding one will actually be.
3. **Benign crossings recorded, not fixed.** `lastLoggedMtu` (nRF, two writers —
   worst case one duplicated or missed log line); `droppedWrites` (one writer,
   32-bit aligned access atomic on both MCUs); ESP32's `oldDeviceConnected`
   (loop-only, so its `volatile` is vestigial and harmless).
4. **Callback logging verified safe, not assumed.** ESP32's `uartWriteBuf()`
   takes `UART_MUTEX_LOCK`; the nRF port builds TinyUSB with
   `CFG_TUSB_OS = OPT_OS_FREERTOS`, which enables its FIFO mutex. Lines can
   interleave, not corrupt.
5. **Concurrency tripwire.** `check_common.sh` now fails if an interrupt-attach,
   task-creation, semaphore, queue, critical-section, `std::thread` or
   `std::mutex` call appears in any firmware tree — the model's strongest claim
   made a checked fact. Scoped to `VARIANTS`; `tools/` is exempt. Whole-line
   comments are ignored. Verified three ways: clean tree exit 0; a
   comment-only mention exit 0; a real `attachInterrupt` call fails naming the
   file and line, exit 1.

**A mistake of mine, caught.** The tripwire's first draft justified exempting
`tools/` with "imu_wake legitimately uses interrupts". The pre-check I ran
alongside it came back empty — nothing under `tools/` uses any of these. The
comment now says the exemption is because the model describes the shipping
firmware, and that none use them *today* is a fact about now, not a
constraint.

**And a stale doc line fixed in passing.** `architecture-runtime.md` still said
the ESP32 "relies on `BLE_MTU_BYTES` being negotiated up front". That constant
was removed in BLE-2; the ESP32 now derives `kRequestedMtu` from the largest
frame and refuses frames that do not fit.

**Needs an ESP32 build and a connect test.** The nRF trees changed only in
comments. *Verified on hardware 2026-09-10: ESP32 flashed, connecting and
streaming as expected.*

### SEC-1 — as implemented

**Confirmed, and forced.** Neither stack has pairing, encryption or an
allow-list — the RaceBox app expects exactly this GATT, so it is a property, not
a defect. Both are single-connection in practice (nRF: `Bluefruit.begin()`
defaults; ESP32: re-advertises only after a disconnect). The Rx characteristic
takes write-without-response, the flood-capable kind. No README said any of it.

**Three things the review missed.**

1. **On the nRF, a connection is itself a wake trigger.** LIGHT_SLEEP exits on
   "BLE connect or IMU motion" and stays connectable throughout, so anyone
   passing a parked unit can wake the GNSS and receive position — and after the
   3-hour rail cut, that wake is a cold GNSS start, so it costs battery too.
   Reachable for up to ~6.5 h after last use: 30 min idle
   (`STATE_IDLE_TIMEOUT_MIN`) plus up to 6 h of LIGHT_SLEEP
   (`STATE_LIGHT_SLEEP_TIMEOUT_MIN 360`); DEEP_SLEEP calls `bleStop()`.
2. **An idle connection prevents sleep entirely** — see the deferred LIGHT_SLEEP
   entry, where the fix is recorded, because it cannot be made on its own.
3. **LAT-3's stack over-read was still reachable as a class.** `LOG_PRINTF` was
   a direct `Serial.printf`, which on nRF *is* the over-reading
   `Print::printf`. LAT-3 bounded one line; any future line over 255 bytes
   would have reopened it.

Also: it advertises a fixed name (`RaceBox Mini` + `DEVICE_ID`), so it is
recognisable to anyone scanning, without connecting.

**Checked and bounded, no change: a write flood.** Writes land in the 8-slot
SPSC ring; overflow is counted and reported; dispatch is one per loop pass, on
the loop. The per-write debug log throttles the ESP32 loop to the UART's pace
under a flood (~3.5 ms per line) — far inside its ~256 ms receive budget —
and the nRF's USB log does not block. `raceboxOnWrite()` is empty, so writes
are inert today.

**What changed.**

1. **Root README: "A note about privacy: this is an open location beacon".**
   What is exposed, when (per variant, with the real timers), the one-connection
   lockout (every build) and the never-sleeps effect (nRF), and that the slide
   switch is the only sure off. Root README only, so three copies cannot drift.
   A first draft wrongly scoped the lockout to the nRF builds; corrected before
   it left.
2. **`g_protocol.h` onWrite contract: "EVERY WRITE IS UNTRUSTED".** Any central,
   any content, any rate; validate every length, index, count and value; drop
   malformed commands rather than repairing them. Inert today, and aimed at
   phase F, whose CAN-filter channel is the first real command parser.
3. **`LOG_PRINTF` is bounded.** `g_log.h` now formats into its own
   `LOG_LINE_MAX` (256) buffer with `snprintf` and writes only what fits, so no
   log line can reach the core's `printf` — an over-long line is clipped, never
   stack contents, and a negative return writes nothing. Stack cost unchanged on
   nRF (it replaces `Print::printf`'s own 256-byte buffer); on ESP32 the buffer
   only lands on the loop task, since its BLE callbacks log with `LOG_PRINTLN`.

**Item 3 had a consequence that had to be checked first.** `snprintf` carries a
printf format attribute and the nRF core's `Print::printf` does not — so the nRF
builds had never format-checked a single log line, and this switches that on.
It also exposed that `uint32_t` is `long unsigned int` on the nRF toolchain
(corrected in BLD-1 above). Every never-checked nRF call with an integer format
was reproduced with its real argument types, expanded through the new macro,
and compiled with the nRF's own `arm-none-eabi-g++ -Wall -Wextra`: **exit 0,
0 warnings**. A negative control — an uncast `uint32_t` to `%u` — does warn,
so the check can see the class. The stricter `-Wformat-signedness`, in no
Arduino build, flags only the OLED's `0x%02X` with an `int` literal; harmless.

The shipping macro was then extracted from `g_log.h` and exercised: a 300-char
line writes 255 bytes (clipped), a short line and the no-argument form write
exactly. Clean on the host and on the nRF compiler. Harness 23952/23952,
`check_common.sh` clean.

**Recorded, not done.**

- **"In use" = subscribed** — in the deferred LIGHT_SLEEP entry, with why it is
  coupled to the wake condition.
- **Optional LESC pairing as a build flag.** Bluefruit supports it without
  disturbing the GATT layout, but it breaks compatibility with the RaceBox app.
  If it is ever wanted, a build flag, never a default. Not built.

**Two corrections to earlier entries**, made visibly in place: ROB-1 called 180
minutes "the LIGHT_SLEEP window" (it is the rail-cut point; the window is 360 —
conclusion unchanged), and BLD-1 repeated the review's claim that `uint32_t` is
`unsigned int` on nRF.

**Build:** `g_log.h` and `g_protocol.h` are shared by all three variants — all
three need rebuilding. Nothing on the console should look different.

**Hardware (2026-09-11):** all three variants built, flashed and verified
working as expected. That build carries every batch 5 change so far — ARC-1's
connect ordering, NEW-3's epoch-to-epoch rate, and SEC-1's bounded
`LOG_PRINTF` with nRF format checking newly switched on — so the compile-time
risks (the macro across every call site, the new format checks) are cleared on
real toolchains, not only in the host reproductions.

### ARC-7 — as implemented (a decision, written into the design record)

**The standard now exists before the encoder does:**
`docs/multiprotocol-design.md` §11.1, with phases F and G in §10 pointed at it
and a pointer from the head of `racechrono-ble-mapping.md`.

**Right, and stronger than stated.** The golden vectors prove "unchanged from
the previous implementation", and RaceChrono has none. §11 called phase F "the
harness's real payoff" without saying what it would check against. The review
worried hand-written expectations would be weak; the real problem is that they
are not neutral — the protocol README and the author's reference sketch already
disagree twice (byte 19; negative clamping), so writing expectations means
picking a source.

**Overstated:** "the strongest quality asset silently does not extend". It
extends twice over. The captured corpus is usable as *input* — raw u-blox
samples, so §14's preserved RaceBox quirks stay in the RaceBox encoder. And
golden vectors can be *created* for RaceChrono once a first version is
accepted; the weaker instruments are needed exactly once.

**Coverage measured, not assumed.** Over the 23,910 captured samples (no
positions printed): 5 real hour boundaries, 500 negative-`nano` samples, 3,094
negative altitudes within the offset — but a maximum speed of **16 km/h** (the
corpus is bench and walking, not driving), so coarse speed is never exercised;
coarse altitude only via pre-fix garbage; no fix types 4/5, no
`headVehValid = 1`, no clamp below −500 m, no week or day rollover. Each gap is
listed in §11.1 as needing a committed synthetic vector.

**The four gates.**

1. **Round-trip** (F, host, continuous) — an independent spec decoder, full
   corpus plus synthetic boundaries. Weakness stated: shares one reading of the
   spec with the encoder.
2. **Differential against the reference sketch** (F, host, once) — removes that
   shared-misreading weakness; every disagreement explained in writing. Rule:
   where README and reference disagree, follow the reference (known to work
   with the app) and record it. Open question, phase F's first task: whether
   the reference's packing can be isolated at all.
3. **The app decides** (G, hardware) — bench and drive, spanning a real hour
   boundary for the sync counter. Coarse speed cannot be road-tested; §11.1 says
   so rather than implying coverage it does not have.
4. **Freeze** (end of G) — golden vectors from the accepted encoder; captured-
   derived ones gitignored (real positions), synthetic ones committed.

**Deliberately left open** for phase F: generalising the harness beyond
`raceboxEncode()`. And gates 1–2 start from the sample, so `buildSample()` stays
covered only by gate 3 until ROB-6 part 2 lands — which raises that deferred
item's value.

**Documentation only; no build.**

### IMU unification, step A — the split (plus NEW-4, and a new IMU harness)

**Bigger than the batch 1 note said.** That note had the two `imuLatchForEpoch()`
bodies differing "only in unit-scale factors". A function-by-function diff found
the whole pipeline duplicated: `imuPoll()`, `imuReadProtocolUnits()`,
`remapAxes()`, `toProtocolInt16()` and `trimSpeedMps()` were identical in code
across the ESP32 and nRF trees, the filter state differed only in threshold
macro *names*, and `imuLatchForEpoch()` only in its two unit factors — all in
files no script could compare, the blind spot the review blamed for IMU-3. Only
the sensor I/O genuinely differed. The seam also had to cut *inside*
`readImuRaw()`, which did sensor read, remap and trim in one function.

**The new layout.**

- `g_imu.cpp` + `g_imu.h` — the pipeline, **identical in all three trees**:
  remap, trim, filters, epoch-locked decimation, protocol conversion, and the
  failed/missing-sensor policy.
- `g_imu_sensor.h` — the seam, all-variant and Arduino-free (only `<stdint.h>`):
  `ImuProtocolUnits` (moved here from `g_imu.h`), `ImuRawSample`,
  `imuSensorBegin()` / `imuSensorRead()` (driver), the driver's unit factors,
  and `imuSensorRestarted()` (pipeline, called by a driver that reconfigures
  outside `imuBegin()`). Settles where `ImuProtocolUnits` lives — the reason this
  was sequenced first — so ROB-6 part 2 can include it on a host.
- Drivers **named after the part**, not the board — the part is what varies, and
  nothing ties a sensor to an MCU family: `g_imu_mpu6050.cpp` (ESP32 tree only)
  and `g_imu_lsm6ds3.cpp` + `g_imu_lsm6ds3.h` (both nRF trees). The LIGHT_SLEEP
  motion-wake API moved into `g_imu_lsm6ds3.h`, which is what let `g_imu.h`
  become identical everywhere; `g_state.cpp` now includes it, stating honestly
  that motion wake is a feature of that part.
- `check_common.sh`: all-variant 13 → **16** (`g_imu.h`, `g_imu.cpp`,
  `g_imu_sensor.h`); nRF-shared swaps `g_imu.cpp`/`g_imu.h` for the LSM6DS3
  driver pair (12).

**Two refinements to the plan, both within its direction.** Unit factors live in
the *drivers* (`kImuAccelToMilliG`, `kImuGyroToCentiDeg`), not as a new
`config.h` gyro constant that step B would only delete — they describe the part,
not a setting. Written token-for-token as before (`1000.0f / IMU_GRAVITY_NATIVE`,
`(180.0f / (float)M_PI) * 100.0f`, `100.0f`), so the output is bit-identical.
And the threshold rename waits for step B: the pipeline reads temporary
`IMU_{ACCEL,GYRO}_TRANSIENT_THRESHOLD_NATIVE` aliases in each `config.h`, so the
names you edit change exactly once — in step B, together with their units.

**NEW-4 — the IMU halted too, and on the nRF on every wake.** Both drivers ended
a failed bring-up in `while (1) delay(100);`. On the nRF that sat inside
`configureNormalMode()`, which also runs from `imuDisarmWake()` on *every*
LIGHT_SLEEP exit — so one I2C hiccup on any wake would park a battery unit in a
loop where the low-voltage cutoff never runs. ROB-1's defect in the IMU, runtime
reachable, and missed by the review because ROB-1 looked only at the GNSS.
Fixed: drivers return `false`; the pipeline marks the IMU down (zeros, trim
stopped, one log line, `imuIsUp()` false) and everything else carries on.

**The failed-read policy moved to the pipeline.** The nRF driver used to return
its last good sample on a failed read, forever — a dead IMU would transmit a
frozen reading as live for the rest of the session; the ESP32 had no guard at all
(IMU-3). Now `imuSensorRead()` reports failure and the pipeline holds the last
good sample through a glitch, then marks the IMU down after **10 consecutive
failures (100 ms)** — within a few epochs, never per sample. Down lasts until a
successful restart (on the nRF, the next LIGHT_SLEEP exit). IMU-3 option 2 is now
a change to `g_imu_mpu6050.cpp` alone.

**Axis-map permutation assert.** Each `IMU_AXIS_*_SRC` was range-checked but not
required distinct, so `X_SRC 0 / Y_SRC 0` compiled and silently duplicated an
axis. A pairwise-distinct `static_assert` now sits beside `remapAxes()` in the
shared file, one copy for every board.

**The proof: a new IMU pipeline harness** (`test/run_imu_harness.sh`,
`test/imu/`). It compiles the **real** `g_imu*`, `ImuAxis` and `g_imu_trim`
sources and each variant's **real `config.h`** against fake sensor libraries,
`Arduino.h`, `Wire.h`, `g_gnss.h` and `g_log.h`, and drives them through one
deterministic script: tilted, biased sensor at rest; no-fix, 3D and 2D fixes;
trim qualifying and locking (~37 s, the real timing); cornering; 3 g and
410 °/s spikes (past the protocol ceiling); an iTOW stall; a 25 ms loop stall;
single and triple failed reads; and on the nRF a LIGHT_SLEEP arm/wake/re-seed.
Two profiles per variant — `parked` (config as shipped) and `live` (transient
thresholds switched on, ~600-700 epochs differing, so `ImuAxis`'s transient path
is really exercised rather than idle behind IMU-2's sentinel).

- **Baseline captured from the pre-split code first; after the split all six
  runs are byte-identical.** No behaviour change on the normal path.
- **The harness was shown able to fail**, by planting bugs and restoring them:
  zeros on a failed read, no re-seed on wake, 9.80665 → 9.8, trim never applied,
  no stall resync, rounding instead of truncation, a down-threshold of 1, and an
  ignored Y sign. Each failed exactly the runs it should — including the
  correct "misses": the Y-sign mutation is a no-op on the ESP32 and OLED maps,
  where `IMU_AXIS_Y_SIGN` is +1.
- Physically sane: at rest the tilted sensor reads 52/31/998 mG (signs per each
  variant's axis map), 0/0/999 after lock, and the tilt reads **3.47°** — exactly
  `atan(0.0605/0.998)` for the scripted motion.
- A third profile, `faults`, documents the new down state (no baseline exists;
  checked by hand before saving): boot-missing (one line, zero reads, zeros, no
  halt); dies-then-recovers (last good held, down after exactly 10 reads/100 ms,
  one log line for 15 failures, reads stop while down, back up with real values
  after a LIGHT_SLEEP exit); restart-fails (down, and the wake API goes quiet).
- Two harness bugs found and fixed on the way: `diff | head` under
  `pipefail`+`set -e` aborted the run at the first mismatch, hiding the rest;
  and buffered stdout interleaved log lines out of order.
- The new nRF files syntax-checked with the nRF's own `arm-none-eabi-g++
  -Wall -Wextra`: 0 warnings (host clang alone would miss `uint32_t` being
  `long` there).

**Also corrected:** stale layout references in `config.h` (the I2C clock is set
by the drivers now), `g_imu_trim.h`, `src/README.md` (which had also long
mislabelled `ImuAxis.*` as the axis remap — it is the per-axis filter), the tool
sketches and tools README (the wake API lives in `g_imu_lsm6ds3.cpp`), and
pointers in `multiprotocol-design.md` and `imu-trim-design.md`.

**Needs:** all three variants built and flashed. On hardware: `✅ IMU` at boot,
rest values and trim lock as before, and on the nRF a motion wake from
LIGHT_SLEEP with no spike in the first frame. The ESP32 can test NEW-4 directly:
boot with the MPU-6050 unplugged — it used to halt; it should now log
`❌ IMU not found - continuing without it` and stream GNSS normally.

**Hardware (2026-09-11): all three variants built, flashed and working as
expected.** That clears the split on the real toolchains — the new files, the
moved `ImuProtocolUnits`, the `extern const` unit factors and the
`g_imu_lsm6ds3.h` include in `g_state.cpp` all compile and link in the Arduino
builds, not only in the host harness.

### IMU unification, step A2 — `IMU_ENABLED`: builds with no IMU

**Why.** The IMU is optional: everything GNSS-based works without it and only
g-force data needs it, so an ESP32 + GNSS build with no MPU-6050, or the plain
(non-Sense) XIAO nRF52840, is a legitimate cheaper device. After step A such a
board already *survived* — a failed bring-up is the IMU-down state — so the
switch buys three things runtime detection cannot: the right message ("not
fitted", not an error), no IMU library needed to build, and no pointless
probe at boot.

**What changed.**

- `IMU_ENABLED` (0/1, `static_assert`ed) in every `config.h`. **On the nRF it
  sets itself from the board selected in the IDE**: `1` when the build defines
  `ARDUINO_Seeed_XIAO_nRF52840_Sense` or `…_Sense_Plus`, `0` otherwise — so a
  plain-XIAO builder gets it right without editing. Verified first that every
  XIAO variant, Sense or not, defines `PIN_LSM6DS3TR_C_POWER` and `Wire1`, so the
  plain board already compiled; the platform passes `-DARDUINO_{build.board}`.
  On the ESP32 it is set by hand (default `1`) — the MPU-6050 is external, so
  nothing about the board says whether it is wired in.
- **`IMU_I2C_ADDRESS` added to the ESP32 config** (`0x68`, `static_assert`ed to
  `0x68`/`0x69`) and passed to `begin()`, matching the nRF. Helps a breakout with
  AD0 tied high, and makes the missing-IMU rehearsal a config change on both
  boards: point it where nothing answers (`0x69` ESP32, `0x6B` nRF — the
  LSM6DS3's only other address, unused because the Sense wires SA0 high).
- **Drivers:** the real implementation and its library `#include` sit inside
  `#if IMU_ENABLED`; the `#else` is a stub (`imuSensorBegin()` / `imuSensorRead()`
  return false; on the LSM6DS3 the wake API is inert). One file per part still
  covers that part being switched off.
- **Pipeline:** unchanged except the boot message — `⏸️ IMU not fitted
  (IMU_ENABLED 0)` versus `❌ IMU not found`. Both land in the same down state:
  one code path, two causes.
- **Stats line:** while `imuIsUp()` is false, for any reason, it renders
  `mG: -|c°/s: -|Trim: -` instead of zeros and a ⏳ that never resolves. Budget
  re-verified from the shipping code: live form is exactly LAT-3's 216/255 (the
  IMU segment uses 83 of its 95-byte buffer); dashed form 155/255.
- **OLED:** the trim indicator is gated on `imuIsUp()` as well as RUNNING.
- **Docs:** a root README note ("the IMU is optional", phrased around g-force
  data since RaceChrono is not implemented yet) and `IMU_ENABLED` /
  `IMU_I2C_ADDRESS` rows in each variant README's configuration table.

**Verified.**

- Harness, two new profiles. `notfitted`: built with the fake sensor headers
  **deleted** — so a no-IMU build demonstrably needs no sensor library — and on
  the nRF via the plain-XIAO board macro, so the auto-detection is exercised
  too. Result: "not fitted", zero reads, zeros, no power-up delay (up at 1 ms
  rather than 301), wake API inert. `wrongaddr`: `IMU_I2C_ADDRESS` moved to where
  nothing answers — the hardware rehearsal — takes the genuine "not found" path
  through the real driver (the nRF even spends its 300 ms power-up first),
  proving the configured address reaches `begin()`.
- The fakes were corrected to mirror the real signatures
  (`begin(uint8_t addr = 0x68)`; `LSM6DS3(mode, addr)`) and to answer only at the
  part's real address; the first A2 run failed to build precisely because the
  fake `begin()` took no argument.
- All nine pre-existing runs byte-identical — nothing moved for a fitted IMU
  (the nRF harness builds now pass the Sense board macro, as the IDE does).
- nRF ARM `g++ -Wall -Wextra`: 0 warnings for both board selections, including
  the plain XIAO compiling the LSM6DS3 driver with the LSM6DS3 header absent.
- `check_common.sh` clean; encoder harness 23952/23952.

**Needs on hardware:** a normal build should look exactly as before. Then the
rehearsal on each board — wrong `IMU_I2C_ADDRESS`, expect `❌ IMU not found`, the
device carrying on, dashes on the stats line, no trim icon on the OLED — and
restore it. Optionally `IMU_ENABLED 0` on the ESP32 for `⏸️ IMU not fitted`. And
still outstanding from step A: an nRF motion wake from LIGHT_SLEEP.

**Hardware (2026-09-11): all three variants built, flashed and working as
expected, and both rehearsals done on hardware** — `IMU_I2C_ADDRESS` moved to
where nothing answers (the NEW-4 "not found" path, which also exercised the
halt fix for real), and `IMU_ENABLED 0` ("not fitted"). The nRF LIGHT_SLEEP
motion-wake check remains outstanding.

### IMU unification, step B — one unit system, and rounding

**What changed.**

- **Every driver reports g and deg/s.** The MPU-6050 driver converts at the read
  (`÷ 9.80665`, `× 180/π`); the LSM6DS3 library already did. The pipeline's
  protocol factors are fixed `× 1000` / `× 100`; the driver-supplied factors
  (`kImuAccelToMilliG`, `kImuGyroToCentiDeg`) and `IMU_GRAVITY_NATIVE` are gone.
- **The configs converged.** Four ESP32 values changed, each to the number its
  own comment named: `IMU_TRIM_ACCEL_VAR_MAX` 0.392 m/s² → `0.04f` g,
  `IMU_TRIM_GYRO_VAR_MAX` 0.017453 rad/s → `1.0f` °/s, and the two transient
  thresholds renamed `_MPS2`/`_RADPS` → `_G`/`_DPS` (still parked). The
  temporary `_NATIVE` aliases from step A were deleted — so the names you edit
  changed exactly once, together with their units, as planned.
- **Trim module untouched.** `g_imu_trim` is unit-agnostic by design; the
  pipeline passes `1.0f` for gravity rather than the field being removed.
- **`toProtocolInt16()` rounds** half away from zero instead of truncating
  toward zero (by hand, still no `<math.h>`; safe at the clamps).

**Measured in two halves against step A's golden output**, so each effect is
attributable:

- **B1, rounding:** every changed field moved by at most **1 unit** on every
  board and profile; ~98% of epochs changed, matching 1 − 0.5⁶ ≈ 98.4% for six
  independently-rounding fields; trim tilt and lock time **unchanged** (rounding
  sits after the trim); no-IMU runs identical.
- **B2, units:** **nRF byte-identical** (as predicted — already g/°/s).
  **ESP32: zero transmitted values changed** in either profile — better than
  predicted, which allowed for values on a rounding boundary to flip. Only the
  trim module's internal tilt moved, by at most **0.00006°** (float rounding in
  g rather than m/s²; the stats line prints one decimal). Lock time unchanged.
- **Acceptance met on every criterion**: all fields within 1 unit of step A,
  trim lock unchanged, nRF identical across B2. New goldens saved after both.

**What "identical on every board" actually covers — corrected.** The plan said
every IMU constant would be identical after B. Checked against the files: all
**16 tuning constants** are (filter alphas, transient thresholds, the parked
sentinel, sample interval, and every `IMU_TRIM_*`). The 15 that still differ are
per-part or per-mount by nature, not units: sensor configuration (ranges, ODR,
bandwidth — enum tokens on the ESP32), the LSM6DS3's pins and wake registers,
I2C addresses, and the axis signs, which follow each board's mounting. The 16
are exactly ARC-8's scope.

**Also:** the ESP32 README's config row and smoothing section renamed and
re-expressed in g; the ESP32 bench tool's comments corrected; a dated
"superseded" note in `imu-trim-design.md` rather than rewriting its history;
the unit-suffix list at the top of the ESP32 `config.h` corrected; the harness
runner's dead m/s²/rad/s substitutions removed. nRF ARM `-Wall -Wextra`: 0
warnings; `check_common.sh` clean; encoder harness 23952/23952.

**Retires a project rule.** "Unit systems differ per family and this is
intrinsic" was the ESP32 library's units leaking through the whole pipeline, not
a property of the hardware.

**Needs:** all three built and flashed. Expect resting values within ±1 mG /
±1 c°/s of before and trim still locking; the ESP32 is where the real change is.

**Hardware (2026-09-11): all three variants built, flashed and working as
expected.** The ESP32's first build with its pipeline running in g and deg/s.

### ARC-8 — as implemented (step C of the IMU work; wider than the review's version)

**The review's scope was out of date by the time it came up.** It listed the
eight identical `IMU_TRIM_*` values and proposed a shared header with three
unit-dependent ones left behind in `config.h`. Step B removed the unit
dependence, so the real hole was all **16 IMU tuning constants** — identical in
all three trees and checked by nothing: the same "byte-identical but UNCHECKED"
state §7.2 of the multiprotocol design fixed for the RaceBox constants.

**What changed.**

- **New `g_imu_tuning.h`, all-variant, in `check_common.sh`'s common set (now
  17 files).** It holds the 16 defines — sample interval, both alphas, the
  parked sentinel, both transient thresholds, all ten `IMU_TRIM_*` — and every
  assert that involves only them.
- **Each `config.h` includes it at the top**, before any board setting (the
  user's call, and the better one: it makes the dependency visible where people
  look, and guarantees the shared file cannot come to lean on a per-board
  define). A one-line pointer sits in the IMU section where the blocks were.
- **The one cross-check stays in `config.h`**: GNSS epoch interval ≥
  `IMU_SAMPLE_INTERVAL_MS` compares a per-board value with a shared one.
- **What stays per board** is what really differs: `IMU_ENABLED`, address, pins,
  ranges, data rates, bandwidth, the nRF wake registers, and the axis map.
- **The transient thresholds are shared too** (decided): they depend on the
  mount, but per-board tuning doesn't exist yet and both are parked. The header
  says to move those two back into each `config.h` if re-tuning ever needs
  different values per board.
- **Three comment copies became one**, keeping both sides' content: the nRF
  measurement detail, and the ESP32's caveat that the accel tuning was never
  re-measured on the MPU-6050. One stale number fixed while merging: the gyro
  alpha's `~3.6Hz` corner was left over from an earlier alpha of 0.2 — at
  0.09 and 100 Hz it is ~1.5 Hz, the same as the accel's.
- **Side effect, intended:** `IMU_TRIM_REQUIRE_FIX 0` (the bench setting) left
  in one tree now fails `check_common.sh`, so it cannot quietly ship in one
  variant. The header says so beside the define.

**Verified.**

- **All 16 values unchanged on every board**, read out of each variant's
  preprocessed `config.h` before and after by a small host program (plus five
  per-board controls — `IMU_ENABLED`, address, two axis signs,
  `GNSS_NAV_RATE_HZ`). This covers `IMU_TRIM_MAX_TILT_DEG`, which the IMU
  harness doesn't reach.
- **IMU harness 15/15 byte-identical**; the `live` profile now edits
  `g_imu_tuning.h`, and its guard was made positive — it checks the replacement
  landed, rather than that the parked text is gone, which would have passed
  silently after the move.
- **Both new protections shown to fire**, in a scratch copy: flipping
  `IMU_TRIM_REQUIRE_FIX` in one tree fails `check_common.sh`; `IMU_ACCEL_ALPHA
  0.0f` fails the moved assert.
- `config.h` + `g_imu_tuning.h` compile clean with `-Wall -Wextra` on
  `xtensa-esp32-elf-g++` and `arm-none-eabi-g++`; `check_common.sh` clean;
  encoder harness 23952/23952.

**Also:** `src/README.md` file map (the `config.h` line also dropped its stale
"offsets"); both variant READMEs point to the new file, and the nRF one's
smoothing row now uses the real names (it still had the pre-`IMU_` names);
`tools/nRF52840/README.md`'s mirror list; `check_common.sh`'s `config.h`
exclusion reason.

**Follow-on, same session: ODR ≥ sample rate asserted (nRF).** The ODR comments
said "≥ the 100Hz poll rate" but nothing enforced it, and after the move the two
halves sit in different files. Both nRF `config.h`s now assert
`IMU_{ACCEL,GYRO}_ODR_HZ * IMU_SAMPLE_INTERVAL_MS >= 1000` (104 × 10 = 1040
passes; 52 Hz shown rejected). Polling faster than the ODR feeds the filters
duplicate samples and silently moves their corner frequency. Not on the ESP32:
its `config.h` sets no ODR. The Adafruit library's `begin()` sets a sample-rate
divisor of 0, and with the low-pass filter on (`IMU_FILTER_BANDWIDTH_HZ` 21 Hz)
that is 1 kHz — 10× the poll rate, with no setting a user could lower it by.

**Needs:** all three built and flashed. Nothing should change on the device.

### ROB-6 part 2 — as implemented (no extraction; plus two extras)

**The plan changed, and got smaller.** The deferral needed three things:
extract `buildSample()` into a new file pair in all three trees, replace its
`batteryGetStatus()` call with a parameter, and decide where `ImuProtocolUnits`
lives. Step A had already answered the third (`g_imu_sensor.h`, Arduino-free),
and the IMU harness had shown the better route: compile the REAL shared file on
the host against fakes. So nothing was extracted. **`src/` is untouched**; the
test compiles the shipping `g_telemetry.cpp` as-is.

**A correction to the earlier record.** ROB-6 called the battery parameter
"independently right", by analogy with the IMU parameter. The analogy fails: the
IMU is a parameter because the latch must happen exactly once per epoch, before
the copy - an ordering the compiler can enforce. A battery getter has no
ordering constraint. Dropped.

**What was added (test only).**

- `test/telemetry/telemetry_harness.cpp` - four modes, one process each so the
  module's statics start fresh:
  - **vectors:** every golden vector through `telemetrySendIfReady()`. The real
    `UBX_NAV_PVT_data_t` is filled FIELD BY NAME over a `0xA5` background, so
    reading the wrong member - or one never meant to be read, like `headVeh` or
    the neighbouring bits of `flags` - changes the frame. The IMU latch and the
    battery come from fakes carrying the vector's values; the frame the real
    code emits through the real encoder is compared byte for byte.
  - **invariants:** no frames while disconnected, but the IMU latched once per
    epoch anyway (the documented reason: a drain only while connected would
    dump a stale peak into the first packet after reconnecting); exactly one
    frame per epoch while connected; nothing at all without an epoch.
  - **rates (extra 1, NEW-3):** 50 s of a receiver running 100 ppm fast, epochs
    picked up 0-2 ms late and phase-aligned to the window boundaries. One lost
    epoch, one dropped BLE frame, a 3 s stall. Asserts: steady windows 20.0 ±
    0.1 (1 of 46 reads 19.9/20.1), the lost epoch's window alone 19.0, the drop
    shows as BLE 19.0 with GNSS 20.0, BLE equals GNSS everywhere else, stall
    windows 0.0, and the window after the stall not stretched. It also asserts
    the scenario is ADVERSARIAL: the old clock-window method would have read
    19/21 in it (it would have: 2 steady windows read 19 and 2 read 21,
    where the real code read 20.0 in all four). Golden output too.
  - **stats (extra 2):** the 1 Hz line through the REAL `g_log.h`, across nine
    windows: normal, sentinel accuracies (`-`), impossible coordinates (`-`),
    refused tilt (`❌` with the angle), IMU down (dashes), drop lines only when
    counts move, GNSS not responding, and the widest realistic line (runtime
    at the 32-bit `millis()` ceiling, every field at its bound). Asserts no
    write was ever clipped, plus content checks so a regenerated golden cannot
    quietly bless a regression. All positions synthetic.
- `test/telemetry/fakes/` - `Arduino.h` (the IMU harness's, plus `Serial`, plus
  the nRF core's `SERIAL_BUFFER_SIZE`, which the nRF `g_gnss.h` asserts on) and
  a one-include stand-in for the SparkFun header that pulls in the library's
  real `u-blox_structs.h`.
- `test/run_telemetry_harness.sh` - all three variants, each with its own real
  `config.h` and headers; `-Wall -Wextra -Werror`; `--save` for the goldens,
  refused when an assertion fails. Its own runner because `run_harness.sh`
  passes its arguments through as vector files.
- `test/gc1.h` - the GC1 loader, MOVED out of `harness.cpp` unchanged so both
  harnesses parse vectors with one piece of code. Encoder harness still
  23952/23952 after the move.

**Proven able to fail.** Eight breakages of the real `g_telemetry.cpp`, each in
a scratch copy, each caught: `hAcc`/`vAcc` swapped; `gnssFixOK` read from the
neighbouring `diffSoln` bit; `headVeh` for `headMot`; battery charging dropped;
the IMU latch moved inside the connected test; the rate divided by the clock
window (the NEW-3 bug); the opening epoch counted; the `tA`/`hA` bound removed.

**Results:** all modes pass on all three variants - 23952 vectors each through
the real path. Encoder harness 23952/23952, IMU harness 15/15,
`check_common.sh` clean.

**Two things it measured.**

- **The widest stats line is 214 bytes** on the nRF builds (199 on the ESP32),
  inside the 216 that LAT-3's arithmetic in `g_telemetry.cpp` claims; that
  figure is a conservative bound (it allows 10 digits for the runtime, which
  the 32-bit `millis()` caps at 7), so it stands.
- **A stale comment in `g_telemetry.cpp`.** The drop-lines block still says the
  stats line "measures 239 bytes normally and 257 at the no-fix sentinels - so
  it silently clips today (see LAT-3)". That was true before LAT-3; the harness
  now shows the opposite. Not fixed in this change (`src/` untouched); fixed
  straight after as a comment-only change in all three trees, citing the
  measured 214 and the 216 bound.

**Docs:** `architecture-verification.md` (diagram node, what-each-layer-catches,
blind spots, running it), the root README's `test/` map (which had also never
listed the IMU harness), `architecture-modules.md`, `harness.cpp`'s header.

**Needs:** nothing on hardware - no firmware changed.

### IMU-3 option 2 — as implemented (a checked read, and three things the plan missed)

**The problem was bigger than batch 1 recorded.** Re-reading the library for
this: every `getEvent()` is THREE unchecked I2C transactions - the 14-byte data
burst (a failure leaves an uninitialised buffer: garbage) plus a read of each
range register. A failed range read returns all ones, which the library decodes
as ±16 g / ±2000 °/s; at our ±4 g / ±500 °/s that scales the sample **4×** - at
rest, Z reads 4 g, indistinguishable from a real impact. Batch 1 knew only the
garbage case.

**What changed - `g_imu_mpu6050.cpp` only.**

- **Bring-up stays with the library** (`begin()`, the three setters). **The read
  is ours**: one burst of the 14 data registers from 0x3B, big-endian pairs,
  temperature skipped. A short or failed transfer returns false, and the shared
  pipeline does what it does for the nRF: holds the last good sample, marks the
  IMU down after 10 in a row. On the ESP32 down lasts until reboot.
- **Scaling from `config.h` at compile time** (the library's own divisors: 8192
  counts/g, 65.5 counts/(°/s)), with `static_assert`s that the configured enum
  tokens are ones the table knows. The library's per-sample range read-back -
  the 4× hazard - is gone.
- **The unit round trip is gone.** The library computes g and °/s, converts to
  m/s² and rad/s, and step B converted back; the driver now reads g and °/s
  directly and the two conversion constants are deleted.
- **One transaction per sample instead of three** (estimated ~0.3 ms saved per
  sample at 100 Hz; not measured).

**Three refinements from stress-testing the plan, all implemented:**

1. **The configuration is verified once at bring-up.** Scaling from the config
   is only right if the chip took it, and the setters are `void` and unchecked;
   `begin()` leaves ±2 g / 500 °/s / 260 Hz. Unverified, a lost accel-range write
   would read every acceleration at **twice** its true size all session (the
   chip at ±2 g gives 16384 counts/g; we divide by 8192). A lost filter write
   removes the anti-alias filter. Now all three are read back; a mismatch fails
   bring-up with its own log line, and a failed read-back (all ones) fails the
   comparison too. (While describing this I first said "half"; the harness's
   misconfigured run shows 1996 mG at rest - twice.)
2. **The check is `requestFrom()`'s count, not `endTransmission()`.** In ESP32
   core 3.3.11, `endTransmission(false)` sends nothing - it marks a repeated
   start and returns 0 unconditionally; the transfer happens in `requestFrom()`.
   Checking `endTransmission()` would have been `getEvent()`'s trap one layer
   down. The harness fake mirrors that behaviour, so a driver checking the wrong
   call fails.
3. **All three accel axes exactly zero is a failed read.** That is the chip's
   power-on state - asleep, every data register zero - which is what a module
   whose supply dropped out and came back returns, with the reads succeeding. A
   working accelerometer always sees gravity, so an exact zero on all three is
   not a measurement. It now ends in IMU-down rather than a sensor reading 0 g.

**Considered and rejected:** a shorter I2C timeout than the core's 50 ms (the
512-byte GNSS ring covers ~256 ms, the loop drains between reads, and the IMU
goes down after 10); a data-ready check (1 kHz sensor, 100 Hz poll - a repeat
cannot happen).

**Verified, in the step-B style of halves so each effect is attributable.**

- **Half 1 - new read, failure injection ignored:** all 15 IMU goldens
  **byte-identical**, including the ESP32's. Dropping the round trip changed
  nothing. (It really is the new path: the fake no longer has `getEvent()`, so
  the old driver cannot build against it.)
- **Half 2 - injection honoured:** the normal scenario's existing injected
  failures (1 read at 50 s, 3 at 55 s), which the MPU-6050 fake could not honour
  before, now reach the ESP32. Only the epochs after them change - 18 of 2200
  (parked) and 16 (live), fading out within 0.6 s as the filters forget the held
  sample (largest field difference 9 units parked, 79 centi-°/s live, where the
  transient blend is on). No trim line, no read count, nothing on the nRF.
- **New fault scenarios**, goldens checked by hand: `dies` (every variant - 15
  failed reads, down 100 ms later, one log line, reads stop, zeros); on the
  ESP32 `misconfigured` (refused at bring-up with its own line) and
  `sensor-reset` (zeros take it down). nRF fault goldens changed by additions
  only.
- **Harness fakes:** `Wire` now models the MPU-6050's data registers - the
  scripted motion quantised at the range the chip is ACTUALLY in, so an
  unverified misconfiguration shows as the real 2× error - and the ESP32 core's
  `endTransmission(false)`. The library fake's setters can be made not to land;
  `getEvent()` was removed from it deliberately.
- **Proven able to fail - five breakages of the new driver, each caught:**
  checking `endTransmission()` instead of the count; removing the bring-up
  read-back; little-endian decoding; removing the all-zero check; gyro scaled
  with the accel divisor. (A first attempt at the read-back mutation came back
  not caught - it was the mutation: `false && A || B || C` still checked B and C.)
- Driver clean under `-Wall -Wextra` on the host; `check_common.sh` clean;
  encoder harness 23952/23952; telemetry harness all passing.

**Not verifiable on the host:** the real ESP32 `Wire` and Adafruit headers
(overloads were checked against core 3.3.11's `Wire.h` by hand). The build is
the check.

**Needs:** ESP32 built and flashed. Expect the same values within a unit and
trim still locking. Optional bench test: touch the MPU-6050's SDA to GND with a
jumper (safe - I2C is open-drain) - within ~100 ms the stats line should read
`mG: -|c°/s: -|Trim: -` and stay that way until reboot. If the ESP32's Core
Debug Level is raised above None, expect a burst of `i2cWriteReadNonStop` errors
from the core too, ending when the IMU goes down.

**Hardware (2026-09-11): ESP32 built, flashed and working as expected** - the
first build against the real core 3.3.11 `Wire` and Adafruit headers, which the
host could not check. The SDA-to-GND bench test was not reported and remains
optional.

### LSM6DS3 filter setting corrected — `IMU_ACCEL_BANDWIDTH_HZ` -> `IMU_ACCEL_LPF1_ODR_DIV`

**Settled against ST's own register drivers**, not inference:

- **LSM6DS3** (what the Seeed library targets): `CTRL1_XL[1:0]` is `bw_xl`, an
  analog anti-alias filter - 400/200/100/50 Hz.
- **LSM6DS3TR-C** (what is fitted): those bits are split - bit 0 `bw0_xl`
  (analog chain) and bit 1 `lpf1_bw_sel` (digital LPF1, ODR/2 or ODR/4).
- The driver's own documentation settles the rest: the analog bit is *"only for
  accelerometer ODR >= 1.67 kHz"* - **inert at our 104 Hz** - and LPF1's
  selector applies when *"LPF2 is not used"*, which is our case.

So `IMU_ACCEL_BANDWIDTH_HZ 50` wrote 0x03, which on this part means LPF1 =
ODR/4 = **26 Hz**, not a 50 Hz anti-alias filter. At our data rate the four
legal values collapsed to two behaviours: 400/200 -> ODR/2, 100/50 -> ODR/4.
The name was wrong, the value was wrong, and the setting was twice as coarse as
the config claimed.

**The replacement** (both nRF `config.h`): `IMU_ACCEL_LPF1_ODR_DIV` (2 or 4),
with `IMU_ACCEL_LPF1_CUTOFF_HZ` derived from it and the ODR. A divider rather
than Hz because that is what the register holds, and because Hz cannot express
every case - ODR 13 would need 6.5. The shipped value stays 4 (26 Hz), so
**nothing changes about the filtering**; the tuning was measured on the part as
it actually behaves.

**Three checks replace the one that validated a fiction:**

1. The divider is 2 or 4 - the only values the part has.
2. **The cutoff must not alias against the read rate:**
   `IMU_ACCEL_LPF1_CUTOFF_HZ * 2 <= 1000 / IMU_SAMPLE_INTERVAL_MS`. The chip
   updates at 104 Hz and `imuPoll()` reads at 100 Hz; the sensor's filter is the
   only thing protecting that resample, and nothing downstream can undo a fold -
   the EMA attenuates folded energy but cannot tell it from signal, and the
   transient detector sees the raw sample first. Passes at divider 4 (26 Hz),
   **fails at 2 (52 Hz)**, and fails for any ODR raised without polling faster.
   Kept as a hard error deliberately (Chris, 2026-09-11): it forbids only
   combinations that genuinely alias.
3. `IMU_ACCEL_ODR_HZ < 1667`, the rate above which the analog bit stops being
   irrelevant and would have to be chosen deliberately.

**Driver:** stops passing the library's `accelBandWidth` (it encodes the wrong
part) and writes `CTRL1_XL` itself after `begin()` - rate, range and LPF1 in one
explicit write, with `bw0_xl` left at the part's default. The boot read-back now
compares that register WHOLE rather than masking the filter bits out, which is
strictly stronger than the version added with the nRF sibling. On the wire this
changes one inert bit (0x03 -> 0x02).

**Verified.** IMU harness 15/15 with the predicted golden changes and nothing
else: the expected value moves 0x48 -> 0x4A, `bdu-lost` now reports only its
CTRL3_C failure (CTRL1_XL matches), and a new **`lpf1-lost`** scenario - the
driver's CTRL1_XL write is the only one that does not land - is refused at
bring-up. Five breakages caught: the driver's write removed; the LPF1 bit
dropped from the expected value; the divider set to 2 (aliasing), to 3 (not a
real value), and the ODR raised to 1660 (analog bit no longer inert) - the last
three as compile errors. Driver clean under `-Wall -Wextra` on ARM against the
real Seeed header; GNSS 3/3, telemetry 12/12, encoder 23952/23952,
`check_common.sh` clean.

**Also corrected:** `g_imu_tuning.h`'s trim note, which did its sensor-noise
arithmetic at "50 Hz bandwidth" - it is 26 Hz, which only strengthens the
conclusion; both nRF README config rows; and the tools README, which now says
`imu_calibration` still uses the library's bandwidth setting and therefore
filters slightly differently from the firmware.

**Needs:** both nRF variants built and flashed. Nothing should change - same
filtering, same values, one inert register bit different.

### g_gnss.cpp split — as implemented (step 2)

The IMU pattern, applied to the GNSS: a driver identical in every tree plus a
thin per-core port. Held to "the receiver sees the same calls in the same
order" by the harness built for it in step 1.

**What moved where.**

- **`g_gnss.cpp` + `g_gnss.h` are now all-variant and CHECKED** (the common set
  is 20 files). Everything about the RECEIVER lives there: baud sweep,
  configuration sequence, PVT callback, epoch cache.
- **`g_gnss_port.h`** (all-variant, checked) is the seam - two functions:
  `gnssPortBegin(baud)` returns a `Stream *` (nullptr if it could not open),
  `gnssPortEnd()` releases it.
- **`g_gnss_port_esp32.cpp`** (one tree) owns `HardwareSerial(2)`, the pins from
  config.h, and the 512-byte ring. LAT-5's "size it ONCE, before the first
  begin()" is now enforced by a flag INSIDE the port rather than by the order
  of statements in the sweep - the driver calls the port repeatedly, so the
  rule had to move to where it can be kept.
- **`g_gnss_port_nrf52.cpp`** (nRF-shared, checked) owns `Serial1`, and carries
  the hard real-time prose and the `SERIAL_BUFFER_SIZE < 100` static_assert that
  used to sit in `g_gnss.h`. Each UART's constraint now lives with that UART;
  the shared header states only what is common (overflow is silent, and how it
  surfaces).
- **`gnssEnd()` is all-variant.** It was nRF-only, which was the last difference
  in the header. The ESP32 never calls it; it exists so the interface does not
  fork, and what it buys a given board is documented in that board's port file.
- **Drift converged on the way through:** the explicit `VAL_LAYER_RAM_BBR`
  layer on all seven config calls (it was the ESP32's implicit default -
  verified identical), log wording ("automatic PVT", "º"), function order, and
  ROB-1's failure message, which now says something true on every board
  ("everything else keeps running" rather than naming battery protection).

**Verified - the point of doing step 1 first.**

- **Both nRF goldens are BYTE-IDENTICAL after the split.** Same ports opened,
  same bauds tried in the same order, same configuration calls with the same
  arguments, same epoch plumbing.
- **The ESP32 golden differs by exactly three lines**, the newly-shared
  `gnssEnd()` scenario that previously could not compile there. Its sweep,
  ring sizing and configuration sequence are unchanged.
- `check_common.sh` clean (20 all-variant, 10 nRF-shared); IMU harness 15/15;
  telemetry 12/12; encoder 23952/23952.
- The driver and both ports compile clean under `-Wall -Wextra` on
  `arm-none-eabi-g++ -std=gnu++11` and `xtensa-esp32-elf-g++`.
- Test-side: `Stream` moved into the base fake Arduino so every harness shares
  one definition; the runner now compiles the shared driver plus that variant's
  port file.

**Needs:** all three built and flashed. Nothing should change. The check worth
doing once on hardware is a boot with the receiver NOT at `GNSS_BAUD` - power
the receiver from a build configured for a different rate, or flash after
changing `GNSS_BAUD` - so the sweep, the switch and the flash save run for real.
Everything else about this path is now covered on the host.

### GNSS harness — as implemented (step 1 of the g_gnss.cpp split)

Built BEFORE the split, deliberately: `g_gnss.cpp` had NO host coverage at all
(the telemetry harness fakes the GNSS module out), so a structural refactor of
the riskiest path in the firmware would have had only "it still works on the
bench" behind it - the thing ARC-7's evidence standard exists to prevent. The
bring-up is where a mistake bites hardest: the sweep ends in `setSerialRate()` +
`saveConfigSelective()`, which writes the baud into the receiver's flash.

**What it is** (`test/gnss/`, `test/run_gnss_harness.sh`): each variant's real
`g_gnss.cpp` compiled with its real `config.h` against a fake receiver and a
fake SparkFun library. The receiver answers only when the port is open at ITS
baud, which is what makes the sweep a real test rather than a walk through a
list. The **golden is the ordered log of every port and library call with its
arguments** - not the serial text: converging the two trees' log wording is part
of the split, and locking it here would report that convergence as a
regression.

**Scenarios:** `at-target` (found first try), `at-9600` (found late, switched,
port cycled, verified, I/O-port subsection saved), `absent` (all seven rates
fail: returns false, stays down, no halt - ROB-1's path, never before tested),
`verify-fails` (answers, takes the new baud, then does not answer - the
"something went deeply wrong" branch), `config-rejects` (a rejected key does
not stop bring-up), `epochs` (callback delivery, consume-once, latest surviving
the consume, and `gnssEnd()` on the nRF).

**Invariants asserted, not just recorded:** the sweep tries `GNSS_BAUD` first;
every failed attempt closes its port before the next opens (checked by walking
the log); and LAT-5's rule that the RX ring is sized ONCE, before the first
`begin()` - the ESP32 driver silently ignores it afterwards, which was prose
until now.

**Proven able to fail - five breakages of the real file, each caught:**
consume-once not clearing its flag; the sweep trying 9600 before the configured
baud; one NMEA key dropped; a full config save instead of the I/O subsection; a
failed attempt leaving its port open.

**Next:** the split itself - a shared core plus `g_gnss_port.h` (open the UART
at a baud, hand back a `Stream *`; release it), an ESP32 port file carrying the
ring sizing, an nRF one, and `gnssEnd()` made all-variant so `g_gnss.cpp` AND
`g_gnss.h` join the checked set. The harness's goldens are what will hold it to
"the receiver sees the same calls in the same order"; the only expected golden
change is the nRF-only `-DHARNESS_HAS_GNSS_END` flag going away.

### Whole-number rate display — as implemented

Agreed 2026-09-10 when NEW-3 landed, done 2026-09-11. The stats line's `BLE:`
and `GNSS:` fields and the OLED's rate field print `%.0f` instead of `%.1f`.

**Why the tenth was noise.** The epoch-to-epoch measurement's real resolution is
whole epochs per second at an integer nav rate, so the tenth only ever carried
the loop's pickup jitter - the 19.9 / 20.1 readings NEW-3's own note predicted
for about 1 window in 25. Chosen over clamping a band around the nominal rate,
which would print a number that is not the measurement.

**The value keeps its precision.** `telemetryGnssRateHz()` / `telemetryBleRateHz()`
still return the float, which is what the telemetry harness's rates mode asserts
on (steady windows 20.0 +/- 0.1, a lost epoch 19.0, a dropped frame's BLE 19.0).
Only the two displays round. A lost epoch still reads 19; what is now invisible
is the matched 19.6 / 20.4 pair a long loop stall used to show - the cost
recorded when this was deferred, accepted.

**Also:** the drop-lines note's measured figure updated - the widest stats line
is now **210 bytes** (was 214), against the same 216 bound and 255 ceiling.

**Verified.** Telemetry harness 12/12 with new stats goldens whose diff is
exactly the rate fields (`BLE: 20.0Hz|GNSS: 20.0Hz` -> `BLE: 20Hz|GNSS: 20Hz`)
and the new longest-line figure; IMU harness 15/15; encoder 23952/23952;
`check_common.sh` clean. The OLED's one-character change is not host-compilable
(u8g2), so the build is its check.

**Needs:** all three built and flashed. Expect `GNSS: 20Hz` where it used to
read `20.0Hz`, and no more 19.9 / 20.1 flicker.

### IMU failed-read counter — as implemented

**The gap it closes.** Since IMU-3 and its nRF sibling both drivers report
failed reads, and the pipeline holds the last good sample through them -
silently by design, because logging each one at 100 Hz would be its own latency
problem. It only speaks after ten failures IN A ROW, as "IMU stopped
responding". The middle case had no voice at all: an intermittent bus can fail
thousands of reads without ever failing ten consecutively, quietly degrading the
data toward held samples until the part finally dies.

**What changed** (all-variant: `g_imu.cpp`, `g_imu.h`, `g_telemetry.cpp`, ~15
lines):

- `totalFailedReads`, counted beside the existing consecutive counter and
  exposed as `imuFailedReads()`.
- `g_telemetry` prints, only when the count has moved:
  `⚠️  IMU: 3 failed read(s) this window (12 total)`.
- **Its own line, not a stats-line field**, exactly as the BLE drop lines
  beside it: the stats line is budgeted to a hard 255-byte ceiling (216 at every
  bound, 214 measured), and a field that is zero almost always would spend that
  margin permanently. Reported in both branches, since a flaky IMU bus does not
  depend on the receiver.
- **Quiet by construction when the IMU dies:** reads stop once it is down, so
  the counter stops, and the sequence is a few failure lines, then the one
  "stopped responding" line, then silence.

**Verified.**

- **The IMU harness carries the stronger check:** every run's summary line now
  prints the pipeline's own counter (`R up=0 reads=111 failed=10`), so it is
  tested in the real code rather than only in the formatting. The values land
  exactly where predicted: 4 in the normal runs (the injected 1 and 3), 10
  wherever the IMU dies (it stops reading at ten, though 15 were queued), 0 in
  every other scenario. Goldens changed by that one field and nothing else.
- **The telemetry harness covers the reporting:** three new stats windows - 7
  failures, then 2 more, then none - assert the line appears, reports the
  window's delta with the running total, and stays away when the count has not
  moved. Purely additive to the goldens.
- **Proven able to fail - four breakages, each caught:** the increment removed
  (IMU harness); the line printed unconditionally (telemetry); the total
  reported as the delta (telemetry); the increment moved so it also counted
  while down (IMU harness). The split is the point: the IMU harness catches
  counting bugs, the telemetry harness reporting bugs.
- `check_common.sh` clean; encoder harness 23952/23952.

**Needs:** all three built and flashed. Nothing new should appear unless a bus
is genuinely flaky - which is the point. On the nRF (onboard part, short traces)
expect silence; the ESP32's hand-wired module is where it would speak first.

**Hardware (2026-09-11): all three built, flashed and working as expected** -
no failed-read lines on any variant, which is the expected reading on healthy
buses rather than an absence of evidence about the counter (the harnesses cover
that).

### nRF sibling of IMU-3 — as implemented (plus a config bug it turned up)

`g_imu_lsm6ds3.cpp` and both nRF `config.h`s. The ESP32's fix, done for the
LSM6DS3, where the gaps are the same in a different form.

**What changed.**

- **Checked reads of our own**, replacing the library's `readRegisterRegion()`,
  which checks the register-address write but ignores `requestFrom()`'s count -
  a short or failed data phase left the tail of `raw[12]` as uninitialised
  stack and returned success. Same transaction shape as the library (address
  write with a stop, then the read), which is proven on this bus; only the
  checks are new. Both calls report honestly on this core: `endTransmission()`
  returns the TWIM error, `requestFrom()` returns `RXD.AMOUNT`.
- **A configuration read-back at bring-up.** Nothing checked anything before:
  the library's `begin()` returns only its WHO_AM_I result, every settings
  write inside it is unchecked, `writeRegister()` is unchecked, and
  `calcAccel()`/`calcGyro()` scale from the library's own copy of the settings.
  The failure that matters is specific to this part - CTRL1_XL and CTRL2_G
  reset to 0x00, POWERED DOWN - so a lost write leaves a sensor off while reads
  keep succeeding and returning zeros. Now CTRL1_XL, CTRL2_G and CTRL3_C are
  read back and compared: rate and range bits (bandwidth masked off), and BDU +
  IF_INC as bits. A failed read leaves 0 and fails the comparison too, so it
  errs toward "not up". One log line names both the values and what was wanted.
- **Bandwidth bits deliberately unchecked.** A lost write shows up in the rate
  bits anyway, and their meaning differs between the LSM6DS3 the library targets
  and the TR-C fitted here - see the deferred note below.
- **A config bug found while writing the tables.** `config.h` allowed accel and
  gyro ODRs of 1666/3332/6664 - the datasheet's spellings - but the library
  switches on 1660/3330/6660 and sends anything else to `default:`, **silently
  104 Hz**; it maps no gyro rate above 1660 at all. Both asserts now list what
  the library actually maps, and the driver's own `odrCode()` static_assert
  keeps the two lists in step. Latent only: the shipped value is 104.

**Verified.**

- **Harness, every change predicted:** ESP32 all five profiles byte-identical;
  nRF parked/live/notfitted/wrongaddr byte-identical (the fake bus emits the
  same bytes the fake library did, and injected failures are counted the same);
  nRF faults gains `misconfigured` and `bdu-lost`.
- **Fakes reworked so the driver is tested, not the library:** a fake I2C bus
  now serves both parts' registers - MPU-6050 big-endian at 0x68, LSM6DS3
  little-endian at 0x6A plus its control registers - and the fake library's
  `readRegisterRegion()` was REMOVED, so a driver going back to the unchecked
  call fails to build. Injected failures are now SHORT READS, the exact case
  the library mishandles. The fake's rate/range tables are written out
  independently of the driver's, so the two must agree.
- **New scenarios:** `misconfigured` (now both parts: on the nRF both sensors
  stay powered down; on the ESP32 it stays at +/-2 g) and `bdu-lost` (the
  driver's own BDU write is the only one that does not land - added after a
  first mutation run showed nothing could catch the CTRL3_C check being
  removed). The `bdu-lost` log incidentally shows the mask working: CTRL1_XL
  reads 0x4B, bandwidth bits included, and passes.
- **Proven able to fail - four of five breakages caught:** byte-count check
  dropped; read-back dropped; gyro register compared against the accel's
  expected bits; CTRL3_C check dropped. The fifth - dropping the
  address-write check - is NOT caught, and that is the honest result: it is
  redundant with the count check, on hardware as in the fake (an unacknowledged
  address means the read returns nothing). Kept anyway: it skips a doomed
  second transaction and mirrors the library.
- Driver compiles clean under `-Wall -Wextra` on `arm-none-eabi-g++
  -std=gnu++11` against the REAL Seeed library header, which is what validates
  the register names and the encoding tables; `check_common.sh` clean.

**Needs:** both nRF variants built and flashed. Nothing should change: trim
still locks, values unchanged. A wrong `IMU_*_ODR_HZ` is now a compile error
rather than a silent 104 Hz.

**Hardware (2026-09-11): both nRF variants built, flashed and working as
expected** - the read-back passed against the real part, so the chip does take
its configuration, and the checked burst read carries the live stream.

### LIGHT_SLEEP removed (replacing the planned collapse)

**The decision.** Describing the collapse against the code turned up a wrong
premise and a stale argument. The wrong premise, from this record's own note:
LIGHT_SLEEP was not dominated by BLE advertising - the nRF core runs `loop();
yield();` forever and `yield()` is only `taskYIELD()`, so the CPU never idled in
any state (a few mA, unmeasured), and the OLED panel stayed lit (10-20 mA). The
stale argument: the escalation's runtime `gnssBegin()` was no longer dangerous
once ROB-1 made it return a bool. And a trace of the state turned up two more
IMU gaps (the gyro was never powered down; the accelerometer was probably never
in its low-power mode).

Weighed plainly: LIGHT_SLEEP's GNSS backup mode is where nearly all of its
saving came from (the ~30 mA GNSS rail); the escalation saved ~0.14 mAh and the
IMU motion wake saved nothing, while being the most intricate and most
bug-prone code in the firmware. Removing the whole state costs a forgotten
device roughly 4 h of RUNNING draw - about a fifth to a quarter of a 900 mAh
cell, by the README's own runtime estimate - bounded by the low-voltage cutoff.
Chosen (2026-09-11): **remove LIGHT_SLEEP entirely**, cut to DEEP_SLEEP after
**4 h**, and key "in use" on **subscription**, not connection.

**What changed.**

- **State machine (both nRF trees):** four states - RUNNING, CHARGE_ONLY,
  BATTERY_WAIT, DEEP_SLEEP. RUNNING's idle rule: no subscribed client, on
  battery, for `STATE_IDLE_TIMEOUT_MIN` (now 240) -> DEEP_SLEEP through the
  existing `enterDeepSleepFrom()` (BLE stopped, UART released, OLED blanked,
  System OFF). Gone: `enterLightSleep()` / `exitLightSleep()`, the escalation,
  `gnssInBackup`, the heartbeat, the whole `case STATE_LIGHT_SLEEP`.
- **"In use" means subscribed** - the SEC-1 note, now trivially correct: with no
  wake condition left, the idle timer is the only place it has to change. New
  `bleIsSubscribed()` (nRF `g_ble`): connected AND `bleuart.notifyEnabled()`,
  the check `bleEmitFrame()` already makes. A stranger's bare connection or a
  forgotten nRF Connect session no longer holds the unit at full power until
  the cutoff.
- **The idle clock stands still on USB power** - a decision taken during
  implementation, flagged here to be reversible: the same reasoning as the
  low-voltage cutoff's `!powerUsbPresent()`. Without it a bench unit
  (`STATE_CHARGE_ONLY_ON_USB 0`) or one on car power would switch itself off
  after 4 h with no cell to protect.
- **Deleted with it:** `gnssSleep()` / `gnssWake()` (the Serial1-release TX
  pulse and its failure path); `powerGnssRailOff()`; the LSM6DS3 wake detector
  (`imuArmWake` / `imuWakeTriggered` / `imuDisarmWake`, the `sensorStarted`
  guard, the TAP_CFG1/MD1_CFG clearing, and `g_imu_lsm6ds3.h`, which declared
  nothing else - `check_common.sh`'s nRF list is 11 files); `imuSensorRestarted()`
  from the all-variant seam (its only caller was the wake exit), so "IMU down"
  now lasts until reboot on every build; `configureNormalMode()` folded into
  `imuSensorBegin()`; the LED sleep pulse, the OLED sleep screen; config
  `STATE_LIGHT_SLEEP_*`, `IMU_WAKE_*`, `IMU_INT1_PIN`, `GNSS_WAKE_PULSE_MS`,
  `LED_LIGHT_SLEEP_*`, `LOG_LIGHT_SLEEP_INTERVAL_MS` and their asserts.
- **Made moot:** the planned CPU-idle fix, the gyro and low-power-mode findings,
  panel blanking and `displayWake()`, "mark GNSS down on a failed wake", and the
  pending motion-wake hardware test. The `imu_wake` and `gnss_pmreq` bench
  sketches were then deleted, with the OLED `oled_layout` mockup (a
  pre-`g_display` layout study that still drew the sleep screen); all three
  remain in git history.
- **Comments** that described LIGHT_SLEEP across ~15 files (GNSS staleness,
  BBR layer, IMU down/trim lifetime, display states, g_state.h's dependency
  list) rewritten; READMEs (root privacy section, both nRF variants, tools) and
  `multiprotocol-design.md` updated.

**Verified.**

- **The idle rule, on the real `g_state.cpp`** (both trees), with a throwaway
  fake-clock driver: never subscribed -> DEEP_SLEEP at 240 min; subscribed until
  100 -> 340; a one-minute subscription at 200 -> 441; USB until 300 -> 540; USB
  throughout -> never; subscribed throughout -> never. (A first run failed one
  case because statics carried between scenarios in one process - a test
  artefact, since real DEEP_SLEEP is a System OFF; each case then ran in its
  own process.)
- **IMU harness, every change predicted:** ESP32 unchanged except its fault
  output losing the two removed scenarios' "not applicable" lines; nRF
  parked/live identical up to t = 80 s (where the old scenario slept), then 159
  more epoch lines (8 s x 20, less the old "woke" line) and ~800 more reads,
  trim lock unchanged; nRF no-IMU runs lost their wake-API line; the nRF
  `dies-then-recovers` and `restart-fails` scenarios removed with the path they
  tested. New goldens saved.
- `g_state.cpp` and `g_led.cpp` (both trees) syntax-clean under `-Wall -Wextra`
  on the host; telemetry harness 12/12; encoder 23952/23952;
  `check_common.sh` clean (17 all-variant, 11 nRF-shared).

**Needs:** all three built and flashed - the nRF variants for the state
machine, the ESP32 because shared IMU files changed (it loses only
`imuSensorRestarted()`, which nothing on the ESP32 ever called, and comments;
its IMU goldens are byte-identical). To see the
cutoff on the bench without waiting 4 h: temporarily set
`STATE_IDLE_TIMEOUT_MIN 1`, run on battery with no app subscribed, and expect
`RUNNING -> DEEP_SLEEP (idle: no subscribed client, no USB).` after a minute;
with an app subscribed it must not fire, and with a bare nRF Connect connection
(no subscribe) it must.

**Hardware (2026-09-11): all three variants built, flashed and working as
expected** - the first build of the four-state machine, `bleIsSubscribed()`
and the deleted wake paths against the real cores. The one-minute idle-cutoff
bench test was not reported and remains open.

### API-4 — as implemented (with the UUIDs the review's version missed)

**The gap.** RaceBox declares `TRANSPORT_NORDIC_UART`. On nRF that transport IS
Bluefruit's `BLEUart`, which builds a fixed GATT with its own hard-coded UUIDs
(`6E400001` service, `…02` Rx write + write-without-response, `…03` Tx
notify-only) and never reads the descriptor's channel table; on ESP32 the table
IS the GATT. The two families present the same GATT only while the table
matches `BLEUart` exactly - protected until now by prose ("both must stay in
step"), and a drift would be invisible on the nRF, which never looks.

**Found while picking up the review's loose ends:** the "not followed" section
declined ARC-3 on the strength of API-4's `static_assert` - which had never been
written.

**What changed** (`g_protocol.h`, `g_proto_racebox.cpp`, a comment in
`g_proto_racebox.h`; all-variant):

- **`nordicUartShapeOk(descriptor)` in `g_protocol.h`, with the transport it
  describes:** 128-bit Nordic UART service UUID, exactly two channels, index 0
  the Tx UUID with props EXACTLY notify, index 1 the Rx UUID with props EXACTLY
  write | write-without-response - the properties `BLEUart` sets. Exact rather
  than "includes", because an extra read or notify would be a characteristic the
  ESP32 serves and the nRF does not. UUID comparison is case-insensitive
  (`BLEUart` holds bytes). C++11 single-return `constexpr`, since the nRF core
  builds `-std=gnu++11`; `g_protocol.h` stays dependency-free.
- **Wider than the review's version:** it proposed channel count and props; the
  UUIDs are the likelier edit and the actual reason `BLEUart` can serve this
  protocol at all, so they are checked too.
- **It checks the real descriptor, not a copy:** `RACEBOX_PROTOCOL` became
  `constexpr` (the header's `extern const` declaration keeps it external -
  confirmed global `R` in the object file on both GCC toolchains; the telemetry
  harness links it from another translation unit on clang), and the assert is
  `transport != TRANSPORT_NORDIC_UART || nordicUartShapeOk(...)`, so it follows
  whatever transport the descriptor declares.
- It fires on the host too: the encoder harness compiles `g_proto_racebox.cpp`.

**Verified.** Six breakages in a scratch copy, each a compile error with the
new message: channels swapped, one Tx UUID digit changed, service UUID digit
changed, notify added to Rx, read added to Tx, a third channel. Two controls
compile: UUIDs in lower case, and the transport switched to
`TRANSPORT_GATT_CHANNELS` with a table that would otherwise fail. Clean with
`-Wall -Wextra` on `arm-none-eabi-g++ -std=gnu++11` and `xtensa-esp32-elf-g++`
(gnu++11/17/2b); encoder harness 23952/23952; telemetry harness 12/12;
`check_common.sh` clean.

**Needs:** a rebuild of all three (no behaviour change - the table was already
right); can ride with the LIGHT_SLEEP-removal flash.

**Hardware (2026-09-11): built into all three with the LIGHT_SLEEP removal,
flashed, working as expected** - the assert passed on the real ESP32 and nRF
cores, and the `constexpr` descriptor behaves as before.

### Deferred: collapse the LIGHT_SLEEP phases — SUPERSEDED: LIGHT_SLEEP was removed entirely (see "LIGHT_SLEEP removed" above)

Today sleep is three-tiered: 30 min idle → LIGHT_SLEEP (GNSS to backup,
IMU wake armed), 180 min → escalate to a GNSS EN-cut, 360 min → DEEP_SLEEP.

**Proposal (2026-09-10): drop the middle escalation** and go LIGHT_SLEEP →
DEEP_SLEEP after ~2 hours.

The escalation buys very little. GNSS in RXM-PMREQ backup draws tens of µA plus
the regulator's quiescent — call it 45 µA, over the 180→360 minute window about
**0.14 mAh**, a rounding error against a cell in the hundreds. LIGHT_SLEEP is
dominated by BLE advertising, not GNSS. The config comment justifies the
escalation as *harmless* ("no reacquisition-speed cost to cutting harder"),
never as worthwhile.

What it costs is real: a state variable (`gnssInBackup`), a branch in the wake
path, an escalation block — and it is **the only runtime path to
`gnssBegin()`**, which is what made ROB-1 dangerous at runtime rather than only
at boot. Removing the phase removes that path structurally.

The 6 h → 2 h part is a separate, behavioural question and not a power one
(~1–2 mAh). It is about how long a forgotten device should stay wakeable: past
DEEP_SLEEP, shake and BLE no longer work and a switch cycle is needed. An event
day with a long lunch break or weather hold argues against 1 h; 2 h gives 2.5 h
total from last activity once the 30-minute idle timer is counted.

Kept separate from ROB-1 deliberately: that is a safety fix to a failure path,
this is a deliberate behaviour change to a well-documented state machine with a
transition table and per-tier prose that would need rewriting.

**Added while doing ROB-3:** if this work revisits what LIGHT_SLEEP actually
powers down, **the OLED panel is the biggest single load in that state** — an
SSD1306 lit draws on the order of 10-20mA, dwarfing most of what else is running
there. Blanking it would be a deliberate reversal of the current design, which
chose to keep a status screen visible while dozing. It would also make
`LIGHT_SLEEP -> RUNNING` a genuine display-wake path and bring back the
`displayWake()` that ROB-3 deleted — written, that time, against a caller that
exists.

**Added while doing SEC-1: "in use" should mean subscribed, not merely
connected.** Today any connected central — even one that never subscribes —
holds `bleDisconnectedSinceMs` at zero, so the nRF never idles into LIGHT_SLEEP
while it is connected. A stranger's idle connection, or a forgotten
nRF Connect session (exactly the batch 2 test setup), keeps the unit fully awake
until the low-voltage cutoff. The fix is to key the idle timer on subscription.
But it **must** change together with LIGHT_SLEEP's wake condition, which is
`bleIsConnected()`: change only the timer and the unit sleeps, the
still-connected central wakes it immediately, and it cycles. That coupling is
why it belongs with this state-machine work rather than as a standalone
change. It helps only the passive or accidental squatter — a deliberate one
subscribes — which is the realistic case.

### Deferred: split `g_gnss.cpp` into a shared core and a platform layer (follow-on to the IMU unification) — DONE (see "g_gnss.cpp split" above)

`g_gnss.cpp` differs between the ESP32 and nRF trees, but mostly by drift, not
design. Diffed code-only (2026-09-11), the genuine platform differences are:

1. **Serial-port API** — ESP32 `HardwareSerial(2)` with pins assigned per
   `begin(baud, SERIAL_8N1, RX, TX)`; nRF `Uart &gnssSerial = Serial1`, fixed
   pins, `begin(baud)`.
2. **RX buffer sizing** — ESP32 only (`setRxBufferSize`, LAT-5); the nRF core
   has no equivalent.
3. **Sleep hooks** — nRF only: `gnssEnd()` (release the UART before a rail cut,
   so TX cannot back-power the receiver), `gnssSleep()` (RXM-PMREQ backup mode,
   UART-RX wake), `gnssWake()` (TX-pin pulse and re-sync). These exist because
   of the *battery* design, not the MCU — an ESP32 on a battery would need them
   too.

Everything else is shared logic sitting in files the checker cannot compare, and
it has already drifted:

- The nRF passes `VAL_LAYER_RAM_BBR` explicitly to every config call and the
  ESP32 does not — **behaviourally identical**: it is the SparkFun v3 default
  for all seven calls (verified in `u-blox_GNSS.h`).
- Log wording ("auto PVT" vs "automatic PVT", "deg" vs "º", ROB-1's
  battery-protection suffix that only fits the nRF) and function order.

The receiver configuration itself — baud sweep, constellations, dynamic model,
NMEA shutdown, nav rate, PVT callback, consume/latest — is the same in both.
Same shape as the IMU pipeline, milder: a shared core plus a thin platform
layer. Deferred until after the IMU unification, because the GNSS path has its
own hardware verification to protect and the IMU split will have proven the
pattern first.

Correction to a remark made while planning the IMU split: `g_gnss.cpp` was cited
alongside `g_ble.cpp` as "same name, differs per board family". `g_ble.cpp`
genuinely differs by MCU (the BLE stack is the chip's); `g_gnss.cpp` differs
mostly by drift. The EN rail is not in `g_gnss.cpp` at all — it lives in
`g_power.cpp`.

### Deferred here from batch 1: IMU-3 option 2 — a real ESP32 failed-read guard — DONE (see "IMU-3 option 2 — as implemented" above)

Replace `myIMU.getEvent()` in the ESP32 `readImuRaw()` with a burst read through
`Adafruit_BusIO_Register` whose result is checked, falling back to `lastGood`
exactly as the nRF path does. Requires this file to own the raw→physical
scaling, obtainable from the public `getAccelerometerRange()` / `getGyroRange()`.

Grouped here because it touches the same read path as the unification below,
and because it deserves its own build and capture rather than riding along with
a comment change. See the IMU-3 entry in batch 1 for why the review's simpler
fix is a no-op.

### Deferred: nRF sibling of IMU-3 option 2 — checked burst read and settings read-back (found 2026-09-11) — DONE (see "nRF sibling of IMU-3" above)

Found while asking whether the nRF IMU shares the ESP32's gaps. Two do, in a
different form; `g_imu_lsm6ds3.cpp` only, same harness treatment as the ESP32.

- **Failed reads are half-detected.** Seeed `LSM6DS3Core::readRegisterRegion()`
  checks the register-address write (`endTransmission()`, which the nRF core
  reports honestly) but ignores `requestFrom()`'s count and copies whatever
  arrived, returning success. A failed or short data phase leaves the tail of
  the driver's `raw[12]` as uninitialised stack - garbage reported as a good
  sample. Fix: do the 12-byte burst on `Wire1` directly and require
  `requestFrom() == 12`; the nRF core returns the true count (`RXD.AMOUNT`).
  Keep the library's `calcAccel()` / `calcGyro()`.
- **Configuration is never verified, and it runs repeatedly.** Seeed `begin()`
  returns only the WHO_AM_I result; every configuration write is unchecked, and
  `calcAccel()` / `calcGyro()` scale from the library's software settings, not
  the chip. The LSM6DS3's reset value for CTRL1_XL and CTRL2_G is 0x00 -
  POWERED DOWN - so a failed write leaves that sensor off while reads keep
  succeeding. (This ran at boot and on every LIGHT_SLEEP exit when found; since
  LIGHT_SLEEP's removal the configuration - now folded into `imuSensorBegin()`
  - runs once at boot, the same exposure as the ESP32's.) The driver's own BDU
  write (CTRL3_C) is unchecked too. Fix: read back CTRL1_XL, CTRL2_G and
  CTRL3_C after configuring; fail if a sensor is powered down or its full-scale
  bits disagree with the config. `imuSensorBegin()` returning false already
  marks the IMU down.
- **Not proposed: the all-zero check.** The part is onboard and powered from an
  MCU pin on the same supply, so the ESP32's loose-power-wire reset is not a
  realistic failure here.

### Deferred: does IMU_ACCEL_BANDWIDTH_HZ mean what the comment says on the TR-C? — ANSWERED: no (see "LSM6DS3 filter setting corrected" above)

The Seeed library targets the original LSM6DS3, where CTRL1_XL bits 1:0 are
BW_XL (400/200/100/50 Hz) - which is what `IMU_ACCEL_BANDWIDTH_HZ 50` and its
"anti-alias filter" comment assume. On the LSM6DS3TR-C actually fitted, those
two bits appear to be LPF1_BW_SEL and BW0_XL instead, so the setting would not
select a 50 Hz filter at all. The library knows about the -C variant elsewhere
(it branches on WHO_AM_I for other registers) but not here.

Needs the TR-C datasheet to confirm before anything is changed. Behaviour is
whatever it has always been - the filter tuning was measured on this hardware -
so the risk is a config comment that misdescribes the part, not a defect. If
confirmed: correct the comment, and decide whether the intended filtering is
reachable through the library at all. The read-back added with the nRF sibling
deliberately masks these bits, so it is unaffected either way.

### Deferred: nRF I2C bus hang with no watchdog (found 2026-09-11)

The nRF core's `TwoWire` (Seeeduino nrf52 1.1.13, `Wire_nRF52.cpp`) waits on the
TWIM peripheral in unbounded loops - `while(!EVENTS_LASTRX && !EVENTS_ERROR);`,
`while(!EVENTS_STOPPED);` - and nothing in the firmware arms a watchdog. If a
bus locks up in a way the TWIM does not flag as an error, the device freezes
until power is removed: no GNSS, no BLE, and **no low-voltage cutoff** (the
property ROB-1 protected), so a frozen unit on battery drains the cell past it.

Whether a held-low line really hangs these loops depends on the TWIM's error
detection and cannot be settled from source. Exposure differs by bus: the IMU's
`Wire1` is internal with short traces and a part that does not clock-stretch
(low); the OLED variant's display on `Wire` is external and hand-wired (much
higher). The ESP32 is not affected - its I2C driver times out (50 ms).

**Left unmeasured, deliberately (2026-09-11).** The bench test - shorting the
display's SCL, then SDA, to GND and watching whether serial and the BLE stream
stop - is not going to be run. So nothing is built: a watchdog is real machinery
(a feed in `loop()`, boot's long blocking waits - the 300 ms IMU power-up, the
GNSS baud sweep - kept under its timeout, System OFF interaction, and it cannot
be stopped once started) and it would be built against a hazard nobody has
observed, to convert a freeze nobody has seen into a reset. That is the kind of
guard this project strips, not adds.

What is recorded instead is the shape of the thing, so it is recognisable if it
ever happens: **total silence** - no stats line, no BLE, LED frozen - with
recovery only by a switch cycle or unplug. The most likely trigger is a
disturbed display connection on the OLED build, since that bus leaves the board.
Patching the vendor core's `Wire` with timeouts is rejected regardless: lost on
every core update.

Re-open this if a unit is ever found frozen, or if a watchdog is wanted for
unrelated reasons (it would cover any hang, not just I2C) - at which point the
bench test is the first step, since it also tells you whether the TWIM flags the
fault on its own.

### Deferred: IMU failed-read counter on the stats line (from IMU-3 option 2) — DONE (see "IMU failed-read counter" above)

The pipeline holds the last good sample through isolated failed reads, silently,
so an intermittent connection is invisible until it finally takes the IMU down.
A line like the BLE drop lines - printed only when the count moves - would show
it. Shared `g_imu.cpp` / `g_imu.h` / `g_telemetry.cpp`, both families; the
telemetry harness's stats mode covers the output.

### Deferred: ESP32 IMU recovery after going down (from IMU-3 option 2) — DECLINED 2026-09-11

**Declined (2026-09-11).** Not built: the fault is very unlikely, the
remediation is a power cycle, and a real wiring fault would not be fixed by
retrying anyway - a probe loop would just poll a disconnected part forever.
Same judgement as the nRF watchdog: no machinery for a hazard nobody has
observed. The failed-read counter already makes the run-up to it visible, and
"IMU down" is reported on the stats line rather than hidden. Re-open only if a
unit is actually seen dropping its IMU mid-session and coming back.

What follows is what was found while scoping it, kept because it constrains any
future attempt.

"IMU down" lasts until reboot on every build (the nRF's LIGHT_SLEEP-exit
restart went with LIGHT_SLEEP on 2026-09-11). The ESP32's hand-wired external
module is the likeliest part to drop out and come back. Two library facts constrain the design: Adafruit
`begin()` blocks >= 300 ms (two `delay(100)` in `reset()`, one in `_init()`),
longer than the ~256 ms the GNSS ring covers; and `reset()` loops
`while (device_reset.read() == 1)`, which a failed read (all ones) makes
infinite. So recovery needs a fast WHO_AM_I probe every few seconds and a
re-initialisation that avoids that loop, not a repeat of `begin()`.

### Deferred to phase H: ESP32 flash headroom (API-6)

The review measured the ESP32 at **1,178,939 of 1,310,720 bytes (89%)** - almost
all Bluedroid - with phase H due to add a GATT-channels builder and a second
encoder. Not a defect; the constraint most likely to stop phase H. Not
re-measured since (the IDE's build output shows it; no arduino-cli here).

Two ways out, cheapest first:

1. **Partition scheme, no code change.** 1,310,720 is the ESP32 Dev Module's
   *default* app partition, which reserves space for OTA and a 1.5 MB SPIFFS.
   The firmware uses neither - no filesystem, no NVS, no OTA (checked
   2026-09-11) - so the stock **"Huge APP (3MB No OTA/1MB SPIFFS)"** scheme
   gives it 3,145,728 bytes, 2.4x the room. Costs: a board-menu setting every
   builder must choose (belongs in the ESP32 README's build steps), and giving
   up OTA the firmware does not have anyway. The ESP32 `imu_calibration` bench
   sketch, which logs to LittleFS, is a separate sketch with its own setting.
2. **NimBLE-Arduino**, the review's suggestion - typically about half
   Bluedroid's size, but a port of `g_ble.cpp`, and a behaviour risk on the
   one file the descriptor split was built to isolate.

Decide at phase H start, after measuring what the second protocol adds. Try (1)
first; (2) only if 3 MB is somehow not enough.

### Deferred: whole-number rate display (from NEW-3) — DONE (see "Whole-number rate display" above)

Show the GNSS and BLE rates as whole numbers (`%.0f`) on the stats line and the
OLED panel, instead of LAT-3's one decimal. The epoch-to-epoch measurement's
real resolution is whole epochs per second at integer nav rates, so the tenth
only ever carries pickup jitter (19.9 / 20.1) or a long loop stall (a matched
19.6 / 20.4 pair). Chosen over clamping a band around the nominal rate, which
would make the displayed number untrue, need `GNSS_NAV_RATE_HZ`-relative bounds,
and have to clamp BLE identically to keep the drop comparison honest.

Cost to know about: it hides the stall pair too. Agreed 2026-09-10 as the right
direction, deferred.

### Deferred here from batch 4: ROB-6 part 2 — harness coverage of `buildSample()` — DONE (see "ROB-6 part 2 — as implemented" above)

Extract `buildSample()` into its own translation unit so the host harness can
test the `UBX_NAV_PVT_data_t` -> `TelemetrySample` copy, closing the last
untested gap between a captured vector and the wire.

Already established (see ROB-6): `u-blox_structs.h` compiles standalone on the
host, so the test uses the REAL struct rather than a stub. Remaining work is the
extraction, replacing the hidden `batteryGetStatus()` call with a parameter, and
deciding where `ImuProtocolUnits` lives.

Grouped here because that last question is the same one the `g_imu.cpp`
unification below and ARC-8 have to answer — doing it earlier means moving the
struct twice.

### Deferred here from batch 1: unify the `g_imu.cpp` decimation — DONE as step A (see above)

The two `imuLatchForEpoch()` bodies differ *only* in their unit-scale factors —
g vs m/s², deg/s vs rad/s. Express those as per-variant constants, following
the `IMU_GRAVITY_NATIVE` precedent the trim work already set, and the function
bodies become textually identical. The decimation could then live in the
checked shared set, removing a blind spot permanently.

Deliberately not folded into IMU-1: it is a structural refactor with its own
blast radius, and bundling it into a timing fix would make both harder to
verify or revert. It belongs beside ARC-8, which is the same migration argument
applied to the trim constants.

---

## Where the review was not followed

Recorded so the divergences are deliberate rather than forgotten.

**ARC-3 — declined.** The note argues `TransportKind` is platform knowledge
living in protocol data, and that `g_ble` could infer it from channel shape
instead. Inferring is more magic, not less: a protocol that coincidentally
matched the Nordic UART shape would silently get the wrong builder. Keeping the
explicit declaration and adding **API-4**'s `static_assert` gets the safety the
note actually wants, at build time. *(That check was only implemented on
2026-09-11 - see "API-4 — as implemented" in batch 5.)*

**ARC-11 — right about the defect, incomplete about the cause.** *(Now done —
see the batch 5 entry: the real reason is written into `src/README.md`.)* The READMEs do
state the shared-library rationale imprecisely. But the reviewer read fresh with
no project history, and the real barrier is not distribution friction: Arduino
compiles library sources without the sketch on the include path, so any
*config-coupled* module needs impl-header plus per-sketch-shim contortions. A
library was built, reviewed and rejected on those grounds. The fix is to state
the actual reason — not to substitute a weaker one that makes the route look
more viable than it is.

**ARC-9 — declined (2026-09-11), and the reason is stronger than "not needed".**
The note calls `imuPoll()`'s anchored cadence (`last += INTERVAL`, resync only
if more than one interval behind) the good idiom and every `last = now` timer
a drift bug, and suggests a `Cadence` helper to make anchoring the default.
Checked site by site, anchoring matters only where the LONG-RUN RATE is the
requirement, and there is exactly one such timer - `imuPoll()`, whose 100 Hz
is what the filter alphas in `g_imu_tuning.h` are tuned for. It is already
anchored. Everywhere else the requirement is a MINIMUM SPACING or a one-shot
delay, where `= now` is correct and anchoring would be wrong:

- **OLED slice pushes** (`lastSliceMs`): the interval exists to stop pushes
  landing back to back while the GNSS UART is receiving. An anchored timer
  "catches up" after a stall with exactly the burst it is there to prevent.
- **OLED render** (`lastRenderMs`): also waits for the post-epoch clear air, so
  its real phase is set by the epoch; `now` is the time it actually ran.
- **Battery sampler** (run starts and the 2.5 ms paced reads): the paced
  spacing is a minimum by design, and a run start drifting a few ms later
  changes nothing - the cutoff debounce is time-based.
- **Switch-sense poll**: a cache refresh; drift is irrelevant.
- **Stats window**: since NEW-3 the rates are measured epoch to epoch, so the
  window's length only sets the report cadence.
- **Debounce anchors, BLE settle and re-advertise delays**: one-shot delays, not
  cadences. The "light-sleep heartbeat" the note also listed is gone with
  LIGHT_SLEEP.

A helper that made anchoring the default would make it the default in the
places it is wrong. The one-off is a one-off because there is one timer that
needs it.

**ARC-6 — acknowledged, not acted on.** `TelemetrySample` does widen with each
protocol. At two protocols the ceiling question is premature; worth recording
where the line is before there are five encoders, not restructuring now.
