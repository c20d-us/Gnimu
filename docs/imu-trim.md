# IMU trim: runtime levelling and gyro de-biasing

`core/ImuTrim.{h,cpp}` learns two corrections while the vehicle is confirmed
stationary, and holds them frozen while it moves. This page explains why each
correction is handled the way it is; the header keeps a short summary.

## Accelerometer: rotation plus a vertical residual, measured once

The correction is the rotation that carries the measured gravity vector back
onto vehicle-up, correcting a slightly off-level mount, **plus** a residual
along the corrected vertical so that the resting magnitude comes out at exactly
1 g.

Both parts are needed. A rotation preserves length, so it can straighten a
tilted reading but can never fix one that is short. A rotation alone would leave
a chip with a real zero-g bias reading, say, 0.925 g forever.

Together they also remove the chip's own accelerometer zero-bias: at rest it is
indistinguishable from mounting tilt, and one correction handles both.

It is measured **once per power cycle and then locked**. A resting
accelerometer cannot separate mount tilt from the slope of the ground under the
vehicle, so every re-measurement is a chance to swap a good calibration for a
worse one. The worst-placed stop of a session, such as a sloped staging lane, is
also the last one before the run that matters.

You can ask for a new measurement (`T` on the console, or the [BLE command](device-commands.md)). That drops the lock and qualifies a fresh one at the next stop, and until it locks the output is uncorrected. The request is a deliberate act, so the risk above is yours to judge: send it where the ground is flat.

## Gyroscope: per-axis zero offset, refined at every stop

At rest the true rotation rate is exactly zero in every orientation, so whatever
the gyro reads is bias.

It is refined at **every** qualifying stop, not locked. The slope problem above
is purely an accelerometer problem (ground attitude doesn't appear on a rate gyro
at all), so there's no reason to freeze it, and gyro bias drifts with temperature
over a session in a way tilt does not.

## Design constraints

- **Unit-agnostic and pure.** No Arduino and no config include: every tunable
  arrives through `ImuTrimConfig`, and accel, gyro and speed are passed in by the
  caller. It builds and is tested on a host (`tests/test_imutrim.cpp`).
- **Uncorrected input.** `update()` must see samples before `apply()`. Feeding the
  estimator its own output would close the loop and make the reported tilt decay
  toward zero as the correction converged, which is wrong for a mounting guard
  that needs the absolute tilt.
- **The reported tilt includes ground slope.** A correctly mounted device
  calibrated on a ramp reads high.
