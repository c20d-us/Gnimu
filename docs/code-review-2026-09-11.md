# Gnimu Second-Pass Review

**Code review · branch `gnimu_multiproto` · 11 Sep 2026**

A second full pass over all three trees after the remediation and refactor work,
read the same way as the first: consistent loop timing, measurement accuracy, and
a seam that can carry a second protocol. Same security scope — BLE attack
surface, untrusted serial input, memory safety.

| | First pass (9 Sep) | This pass |
|---|---|---|
| Variants compiled | 3 / 3 clean | 3 / 3 clean |
| Sketch warnings | 5 | **0** |
| Host harnesses | 1 | **4** |
| Encoder vectors | 23,028 | 23,952 |
| Checked shared files | 24 | **30** |
| `static_assert`s per tree | — | 57 / 82 / 87 |
| ESP32 flash | 89% | 90% |
| New findings | 27 + 11 notes | **6 + 1 note** |

---

## Where this stands

Every finding and architecture note from the first review has a disposition, and
the ones marked fixed are genuinely fixed — I checked the code rather than the
record. Four things are worth saying before the new findings.

**The verification story is now the strongest part of the project.** Four
harnesses compile real firmware modules against fake Arduino and vendor headers:
the encoder against 23,952 golden vectors, the GNSS configuration sequence per
variant, the IMU pipeline across fourteen scenarios including fault injection and
a wrong-address part, and the telemetry path end to end — which closes `ROB-6`'s
`buildSample()` gap. `check_common.sh` grew a coverage sweep that fails on any
unaccounted shared file, and a concurrency tripwire that greps for ISRs, tasks
and locks. That last one is `ARC-1`'s invariant turned from prose into a check
that cannot rot, which is exactly the pattern `ARC-10` asked for.

**The driver seams are what actually killed `IMU-3`.** Splitting
`g_imu_sensor.h` / `g_gnss_port.h` out did not just fix the divergence — it
removed the category. `g_imu.cpp` and `g_gnss.cpp` are now byte-identical across
all three trees and checked, so the "design decision landed in one tree only"
failure has nowhere left to happen in those modules.

**Three of my findings were wrong or incomplete, and the remediation caught it.**
Recorded below, because the corrections matter more than the findings did.

**Six new findings, none severe.** Two are latent — correct today, wrong at
phase F. The rest are polish. Nothing here blocks phase F.

---

## Corrections to the first review

### LAT-3 — I understated it; the real behaviour is a stack over-read

I wrote that the over-long stats line "fails safely (`vsnprintf` is bounded, so
it clips rather than overflows)". That is wrong, and the remediation found the
right answer. `Print::printf` on the nRF cores is:

```c
char buf[256];
int len = vsnprintf(buf, 256, format, ap);
this->write(buf, len);
```

`vsnprintf` returns the length it *would* have written. Past 255 bytes,
`write()` is handed a length longer than the buffer and transmits adjacent stack
memory to the console. That is an over-read and an information leak, not
truncation. The fix now in place — render into an owned buffer with `snprintf`
and `LOG_PRINT` the result, plus a per-field width budget totalling 216 against
the 255 ceiling — addresses the real failure rather than the one I described.

### LAT-2 — my suggested fix was unsafe, and the reasoning against it is better than mine

I suggested raising the nRF UART ring with `-DSERIAL_BUFFER_SIZE=256` via
`boards.local.txt`. Declined, correctly: the macro sizes `RingBuffer::_aucBuffer`,
a **class member**, so it changes `sizeof(Uart)`. A define reaching the sketch's
translation units but not the prebuilt core archive leaves the two disagreeing
about object layout — memory corruption, not a build error. Buying margin on a
constraint the code already meets is not worth that. The `static_assert
(SERIAL_BUFFER_SIZE < 100)` that went in instead is the right answer: it permits
a bigger ring, it just refuses to let one arrive unnoticed and leave four files'
worth of comments quietly wrong.

### Three defects I missed

Found during remediation, all real:

- **The Seeed library's `readRegisterRegion()` checks the address write but not
  the byte count.** A short or failed data phase leaves the tail of the caller's
  buffer as uninitialised stack **and reports success** — uninitialised memory
  straight into the EMA and the trim's stillness gate. The checked `readRegs()`
  that replaced it is a genuine memory-safety fix, in the module I had called
  clean.
- **ESP32 `onConnect()` published `deviceConnected` before writing
  `connectTimeMs`.** A loop on the other core could see "connected" beside the
  previous connection's timestamp, skipping the settle window entirely. I flagged
  the `lastLoggedMtu` race and missed the one that mattered.
- **`toProtocolInt16()` truncated rather than rounded**, opening a two-unit dead
  zone around zero and pulling every value half a unit toward it — a systematic
  bias at the one place floats become protocol integers.

---

## New findings

### R2-1 — Medium — The OLED cannot say the GNSS is dead

`src/Gnimu-nRF52840-OLED/g_display.cpp` — never calls `gnssIsUp()`

Batch 4 gave the serial report an explicit branch for a receiver that never
answered, with the rationale written out: without it "every GNSS field below is a
sentinel or a zero — `SV: 0, Fix: 0, tAcc: 4294967295, Lat: 0.0000000` — which
reads like a device searching for a fix rather than one that has no GNSS at all."

That reasoning applies harder to the OLED, and was not applied there.
`drawRunningBody()` reads `gnssLatestPvt()`, gets `nullptr`, and renders
`0 SV` / `No Fix` / `pDOP --` / `0Hz` / `hAcc --`. A dead or unwired receiver is
pixel-identical to a cold start under a metal roof. This is the variant whose
stated purpose is "seeing fix quality, rate, and battery state without a
receiver", and on it the panel is the *only* readout — the serial line that now
tells the truth is the one nobody has attached.

The asymmetry is inside one function's neighbourhood: the same file already
threads `imuIsUp()` into `drawStatusBar()` for exactly this reason.

> **Fix.** One branch in `drawRunningBody()`, shaped like the IMU's. The status
> bar can keep drawing — battery and BLE state are still true with no receiver.

### R2-2 — Medium (latent) — The inbound write ring is never drained on disconnect

`g_ble.cpp` (both stacks) — `disconnectCallback()` / `ServerCallbacks::onDisconnect()`

The disconnect handlers clear `deviceConnected` and nothing else, and
`bleUpdate()` calls `rxDispatchOne()` unconditionally — before its own
`if (!deviceConnected)` reset of `notSubscribed`. So a write queued by a client
that has since disconnected is still dispatched to `onWrite`, and if a second
client connects in that window the command is attributed to *its* session.

Inert today: `raceboxOnWrite()` is empty. It stops being inert at phase F.
RaceChrono's CAN filter is per-session configuration state — a deny-all followed
by per-PID allows — which is precisely the thing that must not carry across a
client change. It is also the second symptom of `ARC-5` (no protocol lifecycle on
the descriptor); a `onDisconnect` hook would give the protocol somewhere to reset
and would make this a two-line change rather than a transport-level special case.

> **Fix.** Reset `rxHead`/`rxTail` loop-side when the link drops — the same place
> `notSubscribed` is already cleared, not in the callback. Count what was
> discarded, so it is visible rather than silent.

### R2-3 — Low (latent) — The transient window widens across a GNSS outage and discharges into the first recovered packet

`g_imu.cpp` — `imuLatchForEpoch()` is the only caller of `ImuAxis::read()`

`IMU-1`'s fix moved the decimation onto the epoch, which is right. The
consequence is that the transient window's *drain* moved with it. `imuPoll()`
keeps feeding `ImuAxis::update()` at 100 Hz for as long as `imuIsUp()`, and
`maxDeviation_` is reset only inside `read()`. If epochs stop while `gnssIsUp()`
stays true — a receiver brownout, a UART noise storm, an antenna fault — the loop
keeps sampling with nothing draining, and the first epoch after recovery latches
the peak deviation of the entire gap.

`g_imu.h` anticipates half of this: "the cached value freezes and the window
widens until epochs resume, a state in which the device is not producing
telemetry anyway." True while it lasts. What is not covered is the recovery edge,
where one packet carries a transient that may be minutes old.

Harmless as shipped, because both thresholds sit at `IMU_TRANSIENT_PARKED` and
`read()` never blends. It goes live the day they are un-parked — which the
tuning header explicitly expects, naming the transient thresholds as "the
likeliest" per-board value.

> **Fix.** Cheapest honest option: a line beside `IMU_TRANSIENT_PARKED` saying
> un-parking requires a stall-drain. Better: reset the six axes when the display's
> `epochsFlowing` condition would go false, or drain on a fallback timer when no
> epoch has arrived for one `DISPLAY_EPOCH_STALE_MS`-sized window.

### R2-4 — Low — An unsubscribed client emits a drop warning every second, forever

`g_telemetry.cpp` — the `bleDroppedFrames()` delta line

`bleEmitFrame()` counts a dropped frame per refusal, and the *explanatory* line is
correctly latched to once per episode by `notSubscribed`. The drop-delta line in
the stats report is not: it fires whenever the total moves. So an nRF Connect
session left attached produces `⚠️  BLE dropped 20 frame(s) this window` once a
second for as long as it sits there.

The count is accurate and the volume is bounded at one line per second. But this
is the one case where drops are *expected* rather than a fault, it is the exact
scenario `NEW-1` in the remediation record was written about, and it buries the
signal the counter exists to carry — a real congestion drop looks the same.

> **Fix.** Skip the delta line while `notSubscribed` is latched, or count
> pre-subscription refusals in their own counter and report them separately.

### R2-5 — Low — The harnesses run without sanitizers

`test/run_*.sh` — `g++ -std=c++14 -Wall -Wextra -Werror -O1`

Four harnesses now compile real firmware modules on a host, which is the
expensive part and it is done. `-fsanitize=address,undefined` on top is nearly
free and puts a runtime check under exactly the code this review keeps finding
edge cases in: the `int16_t` cast, the ring indices, the `snprintf` width budget,
`writeLittleEndian`'s offsets into an 80-byte payload.

Concretely: UBSan flags a NaN-to-integer conversion directly, so it would have
reported `IMU-4` as a failing test rather than leaving it to be reasoned about.

> **Fix.** Add the flags to the four runner scripts. Any platform difference
> (`-fsanitize` needs the same compiler for link) is contained in one line each.

### R2-6 — Low — ESP32 flash is at 90%, and the refactor is why

`arduino-cli compile` — 1,182,767 of 1,310,720 bytes (was 1,178,939)

Correctly deferred as `API-6` to phase H, so this is direction of travel rather
than a finding to act on. The refactor added ~4 KB net despite removing code,
and 128 KB of headroom now has to cover a GATT-channels builder plus a second
encoder. Worth measuring the partition scheme before phase G rather than
discovering it during phase H.

---

## Residual risk, not a finding

**No watchdog, and the loop carries more safety weight than it did.** Declined
during remediation with reasons recorded, and I am not re-litigating it. But two
changes this round both increased the dependency: `ROB-1` made `gnssBegin()`
non-halting so the loop keeps turning, and the `LIGHT_SLEEP` removal deleted the
alternative path. The low-voltage cutoff is now the only thing between a hung
loop and a damaged cell, and it lives in `loop()`. The decision stands; it is
worth revisiting only if a hang is ever observed in the field.

---

## Architecture, second pass

### What the refactor actually bought

The variant-duplication problem largely dissolved. Thirty files are now checked
byte-identical (twenty across all three trees, up from thirteen), and the two that
matter most — `g_imu.cpp` and `g_gnss.cpp` — joined by being *split* rather than
by being disciplined. That is the difference between a linter and a design: a
driver seam removes the category of bug that `IMU-3` was an instance of, where
`check_common.sh` could only ever have caught instances it was told to look for.

`ARC-1` is in better shape than I asked for. The invariant is documented, the
audit found a real race I had missed, and the tripwire in `check_common.sh`
enforces it mechanically. Where the model genuinely had to widen — the inbound
ring, the first data crossing the callback boundary wider than one byte — it
widened to `std::atomic` release/acquire rather than to `volatile`, with the
cross-core reasoning written down. I read the SPSC ring closely: producer owns
tail, consumer owns head, the full-check acquires on head and the publish releases
on tail, mirrored on the consumer side. It is correct.

`ARC-10` was taken as a standing pattern, which is what I hoped for. 57–87
`static_assert`s per tree, four harnesses, an inverted coverage sweep, a
concurrency grep. Materially fewer invariants now live only in prose, which is
the specific risk that review note was about.

### On the two notes that were declined

**`ARC-3` (`TransportKind` is platform knowledge in protocol data) — declined in
favour of `API-4`'s check. I am satisfied.** My concern was not really the field's
location; it was that phases G and H would produce two GATTs with nothing proving
they agree. `nordicUartShapeOk()` proves it at compile time, against the actual
UUIDs `BLEUart` serves. That addresses the risk. Where the field lives is now a
style question.

**`ARC-9` (the anchored cadence idiom is a one-off) — declined, and the argument
is fair.** My note was about consistency; the response was about which timers
actually need anchoring, which is the better question. A stats window that closes
on the clock and measures epoch-to-epoch does not want a drifting anchor, and
`NEW-3` in the remediation record shows that was worked out against a simulation
rather than asserted.

### The one structural recommendation

**`g_ble.cpp` is now the odd module out, and phase G is the moment to fix it.**

Every other subsystem got the same treatment this round: a tiny Arduino-free
seam header, one shared implementation in the checked set, one small per-platform
`.cpp`, and a host harness over the shared half. `g_imu_sensor.h` and
`g_gnss_port.h` are the same shape as each other, deliberately.

`g_ble.cpp` has none of it. It is the largest module in the project (424 lines on
nRF, 527 on ESP32), it is duplicated in *concept* but not in code, and it is the
only major module with no host test. The parts that are genuinely platform-specific
are small — stand up a service, add a characteristic, notify, read the MTU, read a
CCCD. Everything around them is identical in intent and diverging in detail: two
copies of the ring, two copies of the subscription latch, two copies of the
drop accounting, two copies of the descriptor walk.

Phase G and phase H are each scheduled to write a GATT-channels builder. Written
against the current structure that is two implementations of the same thing, in
two files no script compares, with the divergence invisible until someone connects
a real app to both boards. Written against a `g_ble_port.h` it is one builder and
one set of drop accounting, with the shared half under a harness the way the IMU
and GNSS now are.

This is the highest-leverage change available before phase F, and it is the same
move that has already paid off twice this round.

### Still open from the first review

- **`ARC-5`** (no protocol lifecycle on the descriptor) — recorded, revisit at
  phase F. `R2-2` is a second symptom, which strengthens the case: an
  `onDisconnect` hook would resolve both.
- **`ARC-6`** (`TelemetrySample` is a widening union) — acknowledged, not acted
  on. Correct at two protocols.
- **`API-6`** (ESP32 flash) — deferred to phase H. See `R2-6`.

---

## What I ran

- **arduino-cli compile ×3** — all variants, `--warnings all`, esp32 3.3.11 and
  Seeeduino nrf52 1.1.13. All succeed. **Zero sketch warnings**, down from five.
- **Four host harnesses** — `run_harness.sh` (23,952 vectors), `run_gnss_harness.sh`
  (3 variants), `run_imu_harness.sh` (14 scenarios), `run_telemetry_harness.sh`
  (12 checks). All pass.
- **check_common.sh** — 20 all-variant + 10 nRF-shared files identical; coverage
  sweep and concurrency tripwire both green.
- **Library and core source** — `Print::printf` (to confirm the `LAT-3`
  correction), `RingBuffer.h`, `main.cpp`'s `LOOP_STACK_SZ` (4 KB for the loop
  task, against ~1 KB peak in the stats renderer — comfortable), Bluefruit and
  esp32 `notify()` paths.
- **Memory-safety sweep** — no `strcpy`/`strcat`/`sprintf`/`alloca`. Two
  `memcpy` sites, both bounded (`sizeof` on the PVT copy, an explicit clamp to
  `TELEMETRY_MAX_WRITE_LEN` before the ring copy). All `snprintf` bounded. Heap
  allocation confined to ESP32 setup. No TODO/FIXME anywhere.
- **Not run** — nothing was flashed. Anything about real BLE clients, real MTU
  negotiation or real GNSS behaviour is reasoning from source.

---

Read against `docs/multiprotocol-design.md` phases A–E (done) and F–H (pending),
and `docs/code-review-remediation.md`. Findings here use `R2-*` IDs so they
cannot be confused with the remediation record's own `NEW-*` series, which is
referenced by name where cited. `R2-*` IDs are stable and safe to cite. No
source file was modified.
