# IMU Trim — runtime levelling and gyro de-biasing

Design record for `g_imu_trim`, a shared module that learns the correction for a
slightly off-level mount and the gyro's zero point at runtime, replacing the
per-board `imu_calibration` → `config.h` workflow.

Status: **design agreed, not yet implemented.** Branch `active_calibration`.
Started 2026-09-08.

---

## 1. Problem

Two separate errors currently ride on the IMU output, and only one of them is
handled.

**Chip zero-bias** is handled: `imu_calibration` measures it on a level bench and
the six `IMU_*_OFFSET_*` values get pasted into each variant's `config.h`. It
works, but it is the only per-chip data in the entire configuration, which means
every board needs its own calibration run and its own firmware image before it
produces correct data. For a GPL project that other people are meant to be able
to build, that is the one step in the build that cannot be skipped and is easy to
get wrong.

**Mounting tilt is not handled at all.** The device must be mounted flat and
level, because nothing corrects for it being otherwise. That constrains where it
can go in a car.

The spread across the three boards on hand shows why the first problem has no
shortcut — there is no "typical" value to hardcode:

| board | accel offsets | gyro offsets |
|---|---|---|
| nRF52840 | −0.001, +0.011, +0.031 g | +0.49, −1.56, +0.37 °/s |
| nRF52840-OLED | −0.004, −0.014, +0.057 g | +1.68, **−4.02**, +0.12 °/s |
| ESP32 (MPU-6050) | +0.367, +0.055, **−0.676** m/s² | −0.080, +0.003, −0.009 rad/s |

The MPU-6050's accel bias is several times the LSM6DS3's — worth remembering when
sizing anything that has to hold across both families.

## 2. What RaceBox does

Worth recording, because it sets the bar and it is lower than expected.

RaceBox does **not** auto-level. The Mini manual gives qualitative mounting
guidance only — "near the center of the vehicle, preferably completely level",
"logo towards the sky and the charging port towards the back" — with **no stated
angular tolerance**, and no warning, flag, or refusal at any angle.

Tilt is handled by a manual step in the app:

> "Use the Accelerometer menu to calibrate it so it compensates for mounting
> angles." … "Perform this procedure **every time you mount the device**." …
> "This affects only the recorded g-forces in Lap Timer and Drag Meter modes.
> Timing and positioning accuracy are not affected by calibration."

— <https://www.racebox.pro/info/racebox-user-manual/settings>,
<https://www.racebox.pro/info/racebox-mini-user-manual>

That is functionally our `imu_calibration` sketch with a nicer UI: a single-pose
capture, redone by hand at every remount. It fails silently in the obvious way —
remount, forget the step, and the whole session's g-data is tilted with no
indication.

**Two consequences for this design.** Automatic re-estimation is strictly better
UX than the device we emulate, and it additionally tracks mount sag, which a
single at-mount capture structurally cannot. And since the reference device has
no mounting guard of any kind, a guard here is us going beyond RaceBox rather
than matching it — no app expects a RaceBox-protocol device to stop emitting.

*Unresolved:* it is ambiguous from the docs whether RaceBox's correction lives on
the device or in the app. "Recorded g-forces in Lap Timer and Drag Meter modes"
is mode-scoped language that leans app-side, but the Mini S has onboard logging.
If it is app-side, a real RaceBox emits **uncorrected** data over BLE — which
would explain why AutoXDL grew its own de-biasing. Either way it does not change
this design.

## 3. Scope

**In scope:** slightly off-level mounts, corrected properly up to **15°**.

**Not in scope:** deliberately steep mounting (windshield, A-pillar). An earlier
draft targeted 30–40°; that was overshoot. Beyond ~15° the user has most likely
made a mistake rather than a choice, and the right response is to tell them, not
to silently compensate.

**Not in scope: yaw.** Gravity resolves roll and pitch and is permanently blind
to rotation about the vertical axis. A device yawed 15° mixes 26 % of lateral g
into longitudinal, and nothing in this module can detect it. The discrete
`IMU_AXIS_*_SRC`/`_SIGN` remap still handles the coarse 90° part; the fine yaw
angle needs a GNSS-based solve (straight-line accel/brake events give the
vehicle's forward axis directly) and is deferred to a later change.

## 4. The key distinction: tilt is a rotation, not a bias

Chip bias and mounting tilt look alike at rest and are not the same thing.

A tilt of θ leaks gravity onto the horizontal axes (`sin θ`), **and** scales real
accelerations by `cos θ`, **and** cross-couples them between axes. A constant
subtraction fixes only the first of those three.

| tilt | static gravity leak | gain error on a real 1 g event | leak of that event onto Z |
|---|---|---|---|
| 2° | 35 mg | 0.06 % | 35 mg |
| 5° | 87 mg | 0.4 % | 87 mg |
| 10° | 174 mg | 1.5 % | 174 mg |
| **15°** | **259 mg** | **3.4 %** | **259 mg** |
| 30° | 500 mg | 13.4 % | 500 mg |

At the 15° scope limit a subtraction leaves 3.4 % amplitude error — on a 1.1 g
corner, ~37 mg, the same order as the chip bias and paddock slope already
tolerated. So subtraction is *defensible* here in a way it was not at 30°.

**We implement the rotation anyway.** It costs a 3×3 matvec — 9 multiplies at
100 Hz on a Cortex-M4F with an FPU, and on a 240 MHz ESP32 — and about 25 lines
of Rodrigues written once in a shared file. What that buys is that 15° becomes a
pure policy number rather than an accuracy cliff, and a 12° mount is exact
instead of 2 % low.

The honest counter-argument: rotation math is where sign errors live, and this
codebase has been bitten there before (the inverted axis figure in the protocol
doc; `imu_tiltmap`'s warning that a mirrored map was once masked by the display
layer). That is why the module is a pure function with a host test — see §6.

**Corollary:** once levelling exists, `IMU_ACCEL_OFFSET_*` is redundant. The
estimator measures the total static reading and cannot separate chip bias from
tilt — and does not need to, because both produce the identical error and one
correction removes both.

## 5. Decisions and rejected alternatives

### 5.1 Update only when stationary; freeze while moving — not continuous adaptation

A slow continuous de-bias is what you build when you cannot reliably detect
stationarity. Gnimu has a GNSS speed channel, so it can. Continuous adaptation
during a run can only hurt: gated, it does nothing; ungated, it eats signal — a
sustained 4-second corner is exactly what a still-creeping estimator absorbs.

Frozen means frozen, not slowed. This also guarantees the correction is a
constant for the whole run, so it cannot contaminate the separate
alpha/threshold smoothing work.

### 5.2 Measure the orientation ONCE per power cycle, then lock

**Reversed on 2026-09-08, after bench data.** The original decision was to keep
refining at every stop, justified by tracking **mount sag**. Two findings killed
it.

*The stated rationale did not survive arithmetic.* The slow blend was defended as
averaging ground slope across separate stops — slopes vary in direction, mount
tilt does not. But at `accelBlend` 0.05 per 1 s block the time constant is ~20 s,
and an ordinary 30 s stop is 78 % converged, a 60 s stop 95 %. The blend
therefore converged **within a single stop**: it tracked the most recent ground
rather than averaging across stops, and got the worst of both — too fast to
average slope, too slow to respond to a remount (~90 s to visibly settle, which
is what surfaced it on the bench).

*And the failure mode is worst exactly where it hurts.* Consider an autocross
session: the grid is flat and yields a good trim, then you move to a staging lane
on an entry road that slopes down toward the course. Whatever the adaptation
rate, the last measurement before the run that matters is taken on the least
representative ground in the venue. A step detector adopts the slope instantly; a
slow blend adopts it gradually. Both end up polluted.

There is no way out of that by tuning, because **a resting accelerometer cannot
separate mount tilt from ground slope** — one measurement, two unknowns. Every
re-measurement is therefore a fresh chance to trade a good calibration for a
worse one, with no way to tell which you got.

So: learn once, from a deliberately long stop, and refuse to move afterwards.

The trade is that a mount which genuinely shifts mid-session goes uncorrected
until a power cycle. That is acceptable — a power cycle re-seeds instantly, and
it is the same thing RaceBox requires by hand, minus the step you can forget.
Sag-tracking is forfeited; it was speculative over a single event and was the
only surviving justification for continuous refinement.

**The gyro is NOT locked** and keeps refining at every qualifying stop. The whole
problem above is an accelerometer problem: ground attitude does not appear on a
rate gyro at all, so there is no bad-ground failure mode to protect against, and
gyro bias drifts with temperature in a way mount tilt does not.

*Consequence for `IMU_TRIM_QUALIFY_MS`:* an intermediate draft raised this to
60 s on the theory that duration is what selects a paddock over a staging lane.
That reasoning was superseded almost immediately — **locking after the first
qualifying window already does the selection, by ordering rather than by
duration.** You power on parked, so the paddock stop is first and the lock is set
before the car ever reaches staging.

Settled at **30 s**, where the length serves only as a backstop for the case
where the ordering fails: the device is switched on as the car leaves the
paddock, making the first qualifying stop a staging lane or a red light on a
cambered road. 30 s clears a rolling pause or a stop sign; no duration fixes that
case properly, and the real protection is the instruction to power on where the
car is parked.

*User-facing statement:* "Once the device is powered on, settled, and stationary
for 30 seconds, it will calibrate itself to mount orientation and retain that
calibration until the device is powered off." That this compresses to one
sentence with no caveats is itself part of the argument for it.

### 5.3 Gate the gyro on variance, not magnitude

A gate of `|gyro| < 2 °/s` on the **raw** reading is circular — the OLED board's
Y axis sits at −4.02 °/s at rest, so a bias larger than the threshold holds the
gate shut against the very calibration that would fix it.

Standard deviation over a block is bias-immune by construction: the **mean** is
what we are measuring, the **spread** is what says whether we are actually
rotating. It still catches the case that matters — creeping through a grid queue
in a slow steady turn is several °/s of genuine rotation that must not be
absorbed.

### 5.4 No persistence; zero at boot — rejected: flash-stored seed

A stored correction only earns its keep when powering on already in motion. But
the scenario that makes the stored value *accurate* — a device left mounted,
untouched between sessions — is also the one where you power on parked and get a
stationary window immediately. The cases where it would be accurate and the cases
where it would be needed barely overlap.

Also asymmetric in failure: starting from zero is *honestly* wrong and the
converged flag says so. A stale seed from last week's mounting is confidently
wrong and looks valid.

Rejecting persistence deletes the flash write path (and wear budgeting, and
torn-write handling on the unclean power-off a slide-switch device will see),
stored-format versioning, and a "has the mount changed since last boot?"
heuristic — which would have been the hard part, since a mount change and a slope
change look identical.

### 5.5 Require a valid GNSS fix — rejected: fix-less fallback

An earlier draft allowed trimming before first fix, to use the cold-start window.
Rejected: that window is not scarce. Power on, mount, walk away, return, connect,
drive to grid is minutes of stationary time *after* first fix.

Requiring the fix closes the one real hole in the gate. Constant-velocity cruise
on smooth pavement reads ~1 g magnitude with near-zero gyro variance — genuinely
indistinguishable from parked without a speed reference.

`IMU_TRIM_REQUIRE_FIX` stays as a flag for one specific reason: **bench testing
indoors never gets a fix.** Default 1, flip to 0 at the desk.

**3D fix required.** Briefly relaxed to accept a 2D fix after the first bench
run, where requiring 3D was seen to suspend learning for seconds at a time under
marginal sky (3–4 SV, fix flapping between 2D and 3D). Locking the orientation
(§5.2) removed that argument in the same session: there is no ongoing
accelerometer learning left to protect, and the one window that matters happens
in the paddock under clear sky where a 3D fix is a given. Strict costs nothing
there and buys a stronger validity guarantee. `gnssFixOK` is required either way.

**One gate serves both halves**, so this also pauses gyro refinement under a poor
fix. Accepted deliberately — the gyro only needs to land occasionally to track
thermal drift, and a second gate is not worth the extra constant or the longer
explanation. This design's value has been that it explains in a sentence.

### 5.6 Engine vibration: gate thresholds measured, not guessed

Raised as a concern after the first in-car run — an idling engine shakes the
device, so a trim captured with the engine running might be polluted. Settled
with a raw stationary capture in a 2018 M2 (2026-09-08, `IMU_*_ALPHA` set to
1.0 so the logged values are bit-exact raw samples).

**Only one gate was affected, and it was blocking rather than corrupting.**

| | cold idle | warm idle | driving | old limit |
|---|---|---|---|---|
| gyro sd, worst axis (°/s) | **0.62** | 0.37 | 4.2 | 0.50 |
| \|a\| worst deviation | 2.44 % | 1.71 % | 46.9 % | 3 % |
| GNSS speed | all pass | all pass | none pass | 0.5 m/s |

The elevated axis is rY — pitch, consistent with a longitudinally-mounted six
rocking on its mounts. Cold idle exceeded the gyro ceiling, so **the trim would
never have converged if the logger was switched on after starting the car**,
which is the natural order for most people. Cold-vs-warm is the high cold-idle
rpm, and it is sustained across the whole window, not a transient from the
driver settling in.

**Allowing an idling engine is safe, and this was verified rather than
assumed.** Simulating the real capture on that data — 1 s block means, then the
5-block average — gives:

| | cold idle (sd 0.62) | warm idle (sd 0.37) |
|---|---|---|
| 5-block capture repeatability | **0.019°** | **0.019°** |

Identical, despite 68 % more vibration, because a block mean removes a
zero-mean signal. Gyro means across the two conditions agree to 0.026 °/s.
Set against the **3.2° of ground-slope difference measured between two ordinary
parking spots 4.5 km apart** in the same session, vibration is ~150× down and
is simply not a term in the error budget.

So the thresholds were raised to admit an idling engine: `IMU_TRIM_GYRO_VAR_MAX`
0.5 → 1.0 °/s (1.6× clear of cold idle, still 4.2× under driving), and
`IMU_TRIM_ACCEL_MAG_TOL` 3 % → 4 % (cold idle peaked at 2.44 %, only 1.2× margin,
measured over a fifth of the samples the gate really sees). *That second constant
was retired the next day — see §5.6a; the tight magnitude test it belonged to was
the wrong shape, not merely the wrong value.*

*An earlier proposal to low-pass the gate inputs — separating vibration from
vehicle motion by frequency rather than amplitude — was dropped. It was the
right answer to the problem the filtered data appeared to show; once the data
turned out to be raw, the margin was 24 % rather than an order of magnitude and
a threshold change covers it. Keep it in mind if a rougher engine ever needs it.*

**Caveat: n = 1 vehicle**, and a reasonably smooth six. A four-cylinder or a
diesel could sit well above these figures. `IMU_TRIM_GYRO_VAR_MAX` is the first
constant to look at if trim will not converge in a rougher car.

### 5.6a Rotation is not enough: the magnitude residual

**Found on the ESP32's first run, 2026-09-09** — it never converged, and the
cause was a defect this design carried from the start.

`readImuRaw()` was reading `X=39 Y=0 Z=924.5` mg at rest, so `|a| = 925 mg`, and
the magnitude gate (then ±4 %) rejected **every sample**. No window could ever
qualify. The numbers traced exactly to the offsets §10 step 4 deleted: the
MPU-6050 carries **−69 mg on Z**, and its hand-measured `IMU_ACCEL_OFFSET_Z_MPS2`
had been covering that. Removing it exposed the bias with nothing left to
correct it.

Two separate defects, and both had to be fixed:

**1. The magnitude gate had the same circularity §5.3 fixed for the gyro.** It
tested whether `|a|` *equals* 1 g — but `|a|` at rest is contaminated by the
chip's own zero-g bias, which is exactly what the trim exists to remove. A tight
band therefore holds the gate shut against the very measurement that would fix
it. We identified this trap for the gyro and left it in place for the
accelerometer.

Replaced with the same two-part shape the gyro uses: a **wide plausibility band**
on the mean (`IMU_TRIM_ACCEL_SANITY_TOL`, 25 % — only catching a misconfigured
`IMU_GRAVITY_NATIVE`, a dead axis, a failed read) plus a tight test on the
**spread** (`IMU_TRIM_ACCEL_VAR_MAX`, 0.04 g — idle measures 6–10 mg per axis,
driving 70–118 mg). The mean is what we are measuring; the spread is what says
whether we are moving.

**2. A rotation preserves length, so it can never fix a short reading.** Even
with the gate open, `|a|` would have stayed 925 mg forever and `gZ` would read
925 instead of 1000. The deleted `IMU_*_OFFSET_*` defines corrected the whole
*vector*; replacing them with a pure rotation silently dropped the magnitude
half. So the lock now also stores `accelZBias_ = |a_rest| − g` and subtracts it
along the corrected vertical, making a resting device read exactly `(0, 0, 1 g)`.

Verified against the ESP32's actual logged readings: converges, reports 2.42°
of tilt — which is the 39 mg X bias correctly absorbed as apparent tilt,
`asin(39/925)` — and corrects to `(0, 0, 1000)` milli-g with the gyro at zero.

*Note the asymmetry this leaves:* the horizontal part of a chip bias is absorbed
as apparent tilt, the vertical part as the residual. Conceptually untidy, but it
produces the correct resting vector either way and the error left on real
accelerations is second order at the angles in scope. It is the same
bias-vs-tilt ambiguity §4 already accepted.

*This also vindicates keeping `imu_calibration` (§11).* A −69 mg part is exactly
what its QC role exists to surface, and it was the only thing that could have
predicted this before the hardware did.

### 5.7 Do not gate on BLE connection state — rejected

Considered, to avoid the correction shifting mid-session. Rejected: many users
connect once at the start of the day and stay connected, so trim would silently
never run for them. A feature that does nothing depending on an unrelated usage
pattern is worse than one with a small known wobble.

Connection is also the wrong proxy — the RaceBox protocol has no notion of
"recording", so connected ≠ recording. If mid-session drift proves objectionable
on bench data, the targeted fix is a **reduced blend rate after convergence**
(fast to settle, slow to drift), not a connection gate.

### 5.8 The estimator sees uncorrected data

`imuTrimUpdate()` and `imuTrimApply()` are separate calls, and update runs first,
on the uncorrected vehicle-frame vector.

Not just tidiness: feeding corrected samples back in would make it a closed loop,
and the reported tilt would decay toward zero as it converged. **The guard needs
absolute tilt**, so the estimator must see the uncorrected value.

### 5.9 The module does not include `config.h`

`ImuAxis` takes `alpha` and `transientThreshold` as constructor arguments rather
than reading config directly. Same precedent applies here and matters more: it is
what lets the rotation math be exercised against known inputs on a host, which is
the mitigation for §4's sign-error risk.

The module needs only `math.h` — no `Arduino.h`, no `millis()` (it counts samples
against `sampleIntervalMs`), no GNSS dependency (speed is passed in).

## 6. Algorithm

**Gate** — all three, every sample:

- `speedValid && speed < speedMaxMps` (or `!requireFix && !speedValid`)
- `| |a|/g − 1 | < accelSanityTol` — a wide plausibility band only, not an
  equality test; see §5.6a for why a tight one is circular. Note `|a|` is
  rotation-invariant, so it is immune to mounting orientation by construction
- per-axis accel **standard deviation** < `accelVarMax`
- per-axis gyro **standard deviation** < `gyroVarMax`

**Qualify** — the gate must hold continuously for `qualifyMs`. Any failing
sample slams the window shut and resets the counter.

**Accumulate** — while open, in `blockMs` blocks. Each block emits one mean
gravity vector and one mean gyro triple.

**Accel** — once per power cycle only. Normalize each block's gravity vector and
accumulate `lockBlocks` of them; average, rebuild the rotation, and **lock**.
Subsequent blocks skip the accelerometer entirely.

Note the averaging is on the **gravity vector, not the rotation matrix**:
averaging matrices element-wise does not yield a rotation (the result is not
orthonormal), and doing it properly would mean carrying quaternions and a slerp.
Averaging where gravity appears to point is both simpler and the more natural
space.

`lockBlocks` is not a noise requirement — the gate already rejects any block
containing a disturbance, and one block is far more than enough for precision.
It is margin against a sub-threshold disturbance (someone leaning on the car)
biasing a measurement that is never revisited.

**Out-of-range mounts are refused, not clamped** (revised during implementation;
the first draft said clamp). If the measured tilt exceeds `maxTiltDeg` the
rotation is left untouched and `converged` stays false — a clamped, partly
corrected signal would be harder to reason about downstream than an honest "not
converged". Two consequences: `tiltDegrees` is recorded *before* the range test
so an out-of-range mount still reports its angle to the eventual guard, and the
**gyro half of the block is still accepted**, since gyro bias is independent of
mounting tilt and a steeply mounted device still yields a valid gyro zero.

**Gyro** — take the block mean outright, no blending, **every block for the life
of the session**. At rest the true rate is exactly zero, so there is nothing to
average out and every block is a clean direct measurement. Accepted even when
the accelerometer half is refused for excess tilt, since gyro bias is
independent of mounting orientation.

**Motion** — both frozen.

Noise is not what sets these durations. The LSM6DS3TR-C runs about 90 µg/√Hz; at
50 Hz bandwidth that is ~0.6 mg RMS per sample, and 50 samples of averaging puts
it under 0.1 mg. **Half a second already gives an estimate 100× finer than
anything that matters.** Everything beyond that buys confidence that we are
really stopped, not precision — which is why the qualification window, not the
averaging window, is the number that matters.

**Trap:** Z must read +1 g at rest, not zero. Naively driving all three axes to
zero destroys the gravity reference and the gZ channel.

## 7. API

```c
struct ImuTrimConfig {
  float    gravityNative;     // 1.0f (g) or 9.80665f (m/s^2)
  float    sampleIntervalMs;
  uint32_t qualifyMs;
  uint32_t blockMs;
  uint32_t lockBlocks;        // blocks averaged before the lock
  float    speedMaxMps;
  float    accelSanityTol;    // plausibility band, fraction of gravity
  float    accelVarMax;       // per-axis accel std-dev, native units
  float    gyroVarMax;        // native units (dps or rad/s)
  float    maxTiltDeg;
  bool     requireFix;
};

void  imuTrimBegin(const ImuTrimConfig &cfg);
void  imuTrimUpdate(const float accel[3], const float gyro[3],
                    float speedMps, bool speedValid);
void  imuTrimApply(float accel[3], float gyro[3]);
float imuTrimTiltDegrees();
bool  imuTrimConverged();
```

Call order inside each variant's `readImuRaw()`:

```
read raw → remapAxes() → imuTrimUpdate(...) → imuTrimApply(...) → return
```

Placement after `remapAxes()` is deliberate: the module then sees a clean
vehicle-frame vector regardless of how each variant produced it, and the tilt it
reports is in vehicle terms, so both the display readout and the eventual guard
are meaningful.

`imuTrimTiltDegrees()` and `imuTrimConverged()` exist from day one even though
nothing consumes them yet — adding them later means changing the API in three
byte-identical copies instead of one, and both are wanted on the serial line and
the OLED during bring-up regardless.

## 8. Configuration

Per variant, inside the existing `IMU_*` block, with `static_assert`s in the
file's established style:

```
IMU_GRAVITY_NATIVE          1.0f      // 9.80665f on ESP32
IMU_TRIM_QUALIFY_MS         30000
IMU_TRIM_BLOCK_MS           1000
IMU_TRIM_LOCK_BLOCKS        5
IMU_TRIM_SPEED_MAX_MPS      0.5f
IMU_TRIM_ACCEL_SANITY_TOL   0.25f
IMU_TRIM_ACCEL_VAR_MAX      0.04f     // g; 0.392f m/s^2 on ESP32
IMU_TRIM_GYRO_VAR_MAX       1.0f      // dps; ~0.01745f rad/s on ESP32
IMU_TRIM_MAX_TILT_DEG       15.0f
IMU_TRIM_REQUIRE_FIX        1
```

Only two values differ between the nRF and ESP32 families — the unit systems are
intrinsically different (g / °/s vs m/s² / rad/s) and always have been:

| | nRF52840, nRF52840-OLED | ESP32 |
|---|---|---|
| `IMU_GRAVITY_NATIVE` | `1.0f` (g) | `9.80665f` (m/s²) |
| `IMU_TRIM_GYRO_VAR_MAX` | `1.0f` (°/s) | `0.017453f` (rad/s) |

`IMU_TRIM_ACCEL_VAR_MAX` is a third (0.04 g vs 0.392 m/s², the same value in each
family's units). Everything else is identical everywhere.
`IMU_TRIM_ACCEL_SANITY_TOL` is expressed as a fraction of gravity specifically so
it does not need a per-variant value.

**Removed:** all six `IMU_ACCEL_OFFSET_*` / `IMU_GYRO_OFFSET_*` per variant, and
the subtractions in `readImuRaw()`. This makes the firmware image identical
across boards — the offsets were the only per-chip data in the configuration.

## 9. Mounting guard (deferred)

**Partially landed on the OLED variant (2026-09-09)** as the status-bar trim
indicator — the first consumer of these values, and it went in exactly as
predicted, with no change to `g_imu_trim` and nothing propagating to the other
two trees:

```c
if (imuTrimConverged())                                  -> check
else if (imuTrimTiltDegrees() > IMU_TRIM_MAX_TILT_DEG)   -> X
else                                                      -> nothing
```

Two simplifications versus the table below. The **three tiers were folded to
two** — anything the trim refuses gets the X, with no separate warn band. And
**blank covers two distinct cases**: no stationary window has closed yet
(`tiltDegrees()` reads 0 until the first block, so a badly mounted device shows
blank for ~31 s before the X appears) and a correctable mount that has not
converged. Neither was worth distinguishing on an 8x8 glyph; the check is the
thing being waited for.

Gated to RUNNING, since `imuPoll()` is gated there in the `.ino` and the panel's
own rule is that each screen shows only what its state can know.

**The 1 Hz serial line carries the same three states** (`✅` / `❌` / `⏳`) on all
three variants, using the identical threshold. That matters more than the panel
does: `g_telemetry.cpp` is in the all-variant shared set, so the base nRF52840
and the ESP32 — which have no display — get the refusal signal too, and for them
this line is the only trim indicator there is.

The fuller tiering below is still unimplemented, and the other two variants have
no indicator at all — they have no panel, and the LED is already carrying state
colour.

Two cautions for whoever plugs it in:

**It measures tilt + slope, not tilt.** A correctly mounted device calibrated on
a ramp, a steep pit-in road, or a trailer will read high.

**The failure modes are not symmetric.** A device that silently refuses to emit
at an event is far worse than one emitting slightly degraded data with a warning.
So tier it, and put the refusal threshold well above the correction limit:

| tilt | behaviour |
|---|---|
| < 15° | correct silently |
| 15–35° | correct and emit, but flag hard — OLED readout, LED pattern, serial warning |
| > 35° | unambiguously wrong; refuse, or emit a hard fault indication |

## 10. Implementation plan

1. ~~**Add the module, wired to nothing.**~~ **Done.** Header and implementation;
   host test skipped by decision (§12).
2. ~~**Wire into `Gnimu-nRF52840` only.**~~ **Done and hardware-verified** — see
   the bench results in §12.
3. ~~**Propagate to the other two trees.**~~ **Done.** `g_imu_trim.*` added to
   `COMMON_FILES`; `check_common.sh` passes on all 9 all-variant and 11
   nRF-shared files.

   `g_imu_trim.*` and `g_telemetry.cpp` copied byte-identically, and `g_imu.cpp`
   likewise from the base tree to the OLED tree (they share it). The **ESP32's
   `g_imu.cpp` was adapted by hand**, since it legitimately diverges: a different
   driver (Adafruit MPU6050 via `getEvent()`), no failed-read early return, and
   `imuTrimBegin()` placed ahead of the seed `readImuRaw()` rather than ahead of
   a `configureNormalMode()` it does not have. The `trimSpeedMps()` helper
   transferred verbatim — it depends only on `g_gnss.h` and `config.h`, which
   both families share.
4. ~~**Delete the offsets** across all three.~~ **Done.** All six
   `IMU_*_OFFSET_*` removed from each `config.h`, and the subtractions removed
   from each `readImuRaw()`. **The firmware image is now identical across every
   board** — those six values were the last per-chip data in the configuration.

   Also swept the references they left behind: the `imu_calibration` sketch
   headers in all three tool trees now describe the diagnostic role (§11) and
   state plainly that there is nowhere left to paste their output, and the
   variant READMEs' config tables document `IMU_TRIM_*` in place of the removed
   offset rows.
5. ~~**README and docs.**~~ **Done.** New "A note about mounting and
   self-calibration" section in the root `README.md` — user-facing behaviour, the
   practical notes (power on parked, expect 70–95 s, watch `Trim:`, ~15° limit,
   ground slope is the accuracy floor), and the RaceBox comparison from §2. Both
   tools READMEs and the OLED variant README carry a note that
   `imu_calibration` no longer feeds `config.h` and why it is kept anyway;
   "paste-ready block" reworded to "aggregate block" throughout the sketches,
   since there is nowhere left to paste. `docs/` added to the repo-layout tree.

Offsets come out at step 4, not earlier, so the existing correction stays in
place until trim is proven on hardware.

## 11. `imu_calibration` keeps a job

Its output stops feeding `config.h`, but three roles survive — all of which this
change *creates* rather than removes:

1. **QC screening.** Auto-correction silently swallows a bad part. A chip well
   outside the sane band (accel < ~0.1 g, gyro < ~5 °/s) is defective or
   mechanically stressed; after this change the only symptom would be quietly
   lost dynamic range.
2. **Ground truth for validating the trim.** The sketch measures chip bias
   *alone* — level bench, gravity subtracted. The trim measures bias + tilt and
   cannot decompose it. Sketch on a level bench gives the known bias; mount on a
   known wedge and the trim's estimate should land on (wedge angle + that bias).
   Without this there is no independent reference, and no way to tell a working
   estimator from one converging to something plausible and wrong.
3. **Sizing the guard clamp.** Knowing how much of the error budget is normally
   chip bias says where the §9 thresholds belong — and the MPU-6050's intrinsic
   bias eats a real slice of it before any tilt is added.

Its thermal soak, stability gate, and multi-session flash aggregation make it a
far better measurement than 1.5 seconds in a staging lane will ever be — of a
quantity that no longer needs to be that good. It was over-engineered for its old
job and is about right for its new one.

## 12. Verification

- **Host test: skipped by decision (2026-09-08).** No test target was added to
  the repo. The sign convention was nonetheless verified once during step 2 by a
  throwaway harness in a scratchpad, compiled against the real `config.h`: a
  device pitched 10 degrees reports `tilt = 10.00` and corrects to exactly
  `(0, 0, 1 g)` — the tilt removed rather than doubled — and the gate correctly
  refuses both a moving device and a missing fix. That check is **not committed
  and will not re-run**, so any future change to the rotation math has nothing
  standing behind it but bench verification.
  Two things partly compensate: the derivation is worked by hand in the comment
  above `rebuildRotation()`, including the expanded matrix and a check that
  `R * u = z` for a pitched device, and the vector helpers are written in
  general form rather than folded into specialised expressions so the whole
  derivation is auditable by eye. The module has no Arduino dependency, so
  `g++ -Wall -Wextra -c g_imu_trim.cpp` remains available as a compile check and
  passes clean.
- **Bench, on hardware.** Tilt converges to a known wedge angle; converged flag
  behaves; gyro bias drops to ~0 at rest. Needs `IMU_TRIM_REQUIRE_FIX 0`.
- **In-car.** The gate does not fire while driving; the correction is stable
  across a session; logged tilt is plausible.

## 13. What this does not touch

`IMU_ACCEL_TRANSIENT_THRESHOLD_G` stays at its holding value of 1.5 g. Trim
shifts DC; the transient detector keys on `|raw − smoothedValue_|`, where both
terms shift equally. The two workstreams are independent, and the `alpha = 1.0`
vibration-floor diagnostic still works unchanged — provided the correction is
frozen while moving rather than adapting, which is §5.1.
