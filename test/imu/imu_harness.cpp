// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// ============================================================================
// IMU pipeline harness - runs the REAL g_imu / ImuAxis / g_imu_trim code, and
// each variant's REAL config.h, against fake sensor libraries driven by one
// deterministic script. Everything the pipeline emits goes to stdout; logs go
// to stderr and are not compared.
//
// It exists to hold the IMU pipeline to "no behaviour change" across
// refactors - first the g_imu.cpp unification - the way test/harness.cpp holds
// the RaceBox encoder. See test/run_imu_harness.sh.
//
// The fakes only need to be CONSISTENT, not faithful: the "before" and "after"
// builds run against the same ones, so any difference in output is the code.
// ============================================================================

#include "fake_control.h"
#include "g_imu.h"
#include "g_imu_trim.h"
#include <stdio.h>
#include <string.h>

// Deterministic PRNG - the same stream on every run and every machine.
static uint32_t rng = 12345u;
static float urand() { // [0, 1)
  rng = rng * 1664525u + 1013904223u;
  return (float)(rng >> 8) * (1.0f / 16777216.0f);
}
static float noise(float amp) { return (urand() * 2.0f - 1.0f) * amp; }

// Scenario phases, in milliseconds of fake time since boot.
struct Phase {
  unsigned long untilMs;
  bool pvt;        // receiver producing epochs at all
  uint8_t fix;     // fixType
  float speedMps;  // ground speed
  bool moving;     // motion profile
  bool itowFrozen; // receiver alive but epoch counter stalled (staleness)
};
static const Phase kPhases[] = {
    {2000, false, 0, 0.0f, false, false},  // no PVT yet
    {45000, true, 3, 0.0f, false, false},  // parked, 3D fix: trim qualifies
    {70000, true, 3, 8.0f, true, false},   // driving
    {75000, true, 3, 8.0f, true, true},    // iTOW stalls: speed goes stale
    {80000, true, 2, 8.0f, true, false},   // 2D fix: trim gate closed
    {110000, true, 3, 8.0f, true, false},  // driving again
};

static const Phase &phaseAt(unsigned long t) {
  for (const Phase &p : kPhases)
    if (t < p.untilMs)
      return p;
  return kPhases[sizeof(kPhases) / sizeof(kPhases[0]) - 1];
}

// Physical motion for this instant: a tilted, slightly biased sensor at rest,
// plus cornering and yaw when moving, plus deliberate spikes - one beyond the
// protocol's +/-327.67 deg/s so toProtocolInt16()'s clamp is exercised.
static void setMotion(unsigned long t, bool moving) {
  float a[3] = {0.052f, -0.031f, 0.998f};
  float g[3] = {0.31f, -0.22f, 0.12f};
  const float an = moving ? 0.02f : 0.002f, gn = moving ? 0.6f : 0.05f;
  if (moving) {
    const float ph = (float)(t % 4000) / 4000.0f * 6.2831853f;
    a[1] += 0.35f * sinf(ph);
    a[0] += 0.15f * cosf(ph * 0.5f);
    g[2] += 22.0f * sinf(ph);
  }
  for (int i = 0; i < 3; i++) {
    a[i] += noise(an);
    g[i] += noise(gn);
  }
  if (t == 52000) a[0] += 3.0f;   // 3 g impact
  if (t == 53000) g[2] += 410.0f; // yaw spike past the protocol ceiling
  if (t == 58000) a[1] -= 2.5f;
  fakeSetMotion(a, g);
}

static int runNormal() {
  imuBegin();

  uint32_t iTOW = 100000u;
  unsigned long lockedAt = 0;

  for (unsigned long t = fakeNowMs(); t < 110000; t = fakeNowMs()) {
    fakeAdvanceMs(1);
    t = fakeNowMs();
    const Phase &p = phaseAt(t);
    setMotion(t, p.moving);

    // The receiver's epoch: every 50 ms, unless stalled.
    if (t % 50 == 0 && !p.itowFrozen)
      iTOW += 50u;
    fakeSetPvt(p.pvt, p.fix, p.fix >= 2, (int32_t)(p.speedMps * 1000.0f),
               iTOW);

    // Injected failed reads. Both drivers report these since IMU-3 option 2
    // (before it, the MPU-6050 path could not fail - getEvent() always said
    // yes). All below the down-threshold: the pipeline holds the last good
    // sample through them.
    if (t == 50000) fakeFailNextReads(1);
    if (t == 55000) fakeFailNextReads(3);

    // A 25 ms loop stall: exercises imuPoll()'s resync.
    const bool stalled = (t >= 60000 && t < 60025);
    if (!stalled)
      imuPoll();

    if (t % 50 == 0) {
      const ImuProtocolUnits u = imuLatchForEpoch();
      printf("E %lu %d %d %d %d %d %d\n", t, u.gX, u.gY, u.gZ, u.rX, u.rY,
             u.rZ);
    }
    if (t % 1000 == 0)
      printf("T %lu %.9g %d\n", t, imuTrimTiltDegrees(),
             (int)imuTrimConverged());
    if (!lockedAt && imuTrimConverged())
      lockedAt = t;
  }
  printf("R reads=%d failed=%u trim_locked_at=%lu\n", fakeReadCount(),
         (unsigned int)imuFailedReads(), lockedAt);
  return 0;
}

// ----------------------------------------------------------------------------
// Fault scenarios: the "IMU down" state the unification introduced. No
// baseline exists for these - they document the new behaviour, and their
// golden output was checked by hand before it was saved. Logs are part of the
// behaviour here ("log once, never per sample"), so the runner keeps stderr.
// ----------------------------------------------------------------------------

// Run the loop for `ms`, polling and latching like the firmware; print an
// epoch line every `every` ms and whenever imuIsUp() changes.
static void runFor(unsigned long ms, unsigned long every) {
  static int lastUp = -1;
  const unsigned long end = fakeNowMs() + ms;
  while (fakeNowMs() < end) {
    fakeAdvanceMs(1);
    const unsigned long t = fakeNowMs();
    setMotion(t, false);
    fakeSetPvt(true, 3, true, 0, (uint32_t)(t / 50 * 50));
    imuPoll();
    if (t % 50 == 0) {
      const ImuProtocolUnits u = imuLatchForEpoch();
      if (t % every == 0)
        printf("E %lu %d %d %d %d %d %d\n", t, u.gX, u.gY, u.gZ, u.rX, u.rY,
               u.rZ);
    }
    if ((int)imuIsUp() != lastUp) {
      lastUp = (int)imuIsUp();
      printf("U %lu up=%d reads=%d\n", t, lastUp, fakeReadCount());
    }
  }
  fflush(stdout);
}

// The sensor never answers: no halt, zeros throughout, no reads attempted.
static int runBootMissing() {
  printf("S boot-missing\n");
  fflush(stdout);
  fakeSetBeginResult(false);
  imuBegin();
  runFor(1000, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}

// Healthy, then a run of failed reads longer than the down-threshold: down,
// zeros, one log line - and down for good. No build has a restart path (the
// nRF's was the LIGHT_SLEEP exit, removed 2026-09-11), so this is what a
// failed IMU does until reboot.
static int runDies() {
  printf("S dies\n");
  fflush(stdout);
  imuBegin();
  runFor(1000, 500);
  fakeFailNextReads(15);
  runFor(1000, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}

// The chip answers but its configuration writes do not land. Must be refused
// at bring-up, on either part, because neither library checks them and both
// drivers scale from config.h:
//   MPU-6050 - stays at the library's +/-2 g, so every acceleration would read
//              at TWICE its size for the whole session.
//   LSM6DS3  - CTRL1_XL/CTRL2_G stay 0x00, which is POWERED DOWN, so every
//              sample would read zero while the reads themselves succeed.
static int runMisconfigured() {
  printf("S misconfigured\n");
  fflush(stdout);
  fakeSetConfigTakes(false);
  imuBegin();
  runFor(500, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}

#ifdef HARNESS_LSM6DS3
// The part takes its ranges and rates, but the driver's own BDU write is lost.
// Nothing in the library or the write reports it, and the consequence is
// subtle: without Block Data Update a 16-bit read can straddle a sample
// (torn), and without auto-increment the burst read would re-read one register
// rather than walking the six. Must be refused at bring-up.
static int runBduLost() {
  printf("S bdu-lost\n");
  fflush(stdout);
  fakeSetBduWriteLands(false);
  imuBegin();
  runFor(500, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}

// The part takes everything except the driver's own CTRL1_XL write, so it
// keeps the library's LSM6DS3-shaped filter bits instead of the LPF1 setting
// this firmware asks for. Must be refused: the filter is what protects the
// 104Hz -> 100Hz resample (see IMU_ACCEL_LPF1_ODR_DIV in config.h).
static int runLpf1Lost() {
  printf("S lpf1-lost\n");
  fflush(stdout);
  fakeSetLpf1WriteLands(false);
  imuBegin();
  runFor(500, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}
#endif

#ifdef HARNESS_MPU6050
// The module's supply drops out and comes back: the chip is in its power-on
// state, asleep, every data register zero - and reads of it SUCCEED. Must end
// in the IMU-down state, not a sensor apparently reading 0 g.
static int runSensorReset() {
  printf("S sensor-reset\n");
  fflush(stdout);
  imuBegin();
  runFor(1000, 500);
  fakeSetSensorResetState(true);
  runFor(1000, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}
#endif

// IMU_ENABLED 0: the driver is a stub. Must say "not fitted" (not "not
// found"), never read, and report zeros. Built WITHOUT the fake sensor headers, which proves a no-IMU build
// needs no sensor library.
// Also used for "wrong-address": IMU_I2C_ADDRESS pointed where nothing
// answers, the on-hardware way to rehearse a missing IMU. That must take the
// "not found" path instead - same state, different message - which proves the
// configured address really reaches the driver's begin().
static int runNoImu(const char *name) {
  printf("S %s\n", name);
  fflush(stdout);
  imuBegin();
  runFor(1000, 250);
  printf("R up=%d reads=%d failed=%u\n", (int)imuIsUp(), fakeReadCount(),
         (unsigned int)imuFailedReads());
  return 0;
}

int main(int argc, char **argv) {
  // Line-buffered even into a file, so epoch lines (stdout) and log lines
  // (stderr, unbuffered) interleave in the order they actually happened.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  const char *which = argc > 1 ? argv[1] : "normal";
  if (!strcmp(which, "normal")) return runNormal();
  if (!strcmp(which, "boot-missing")) return runBootMissing();
  if (!strcmp(which, "dies")) return runDies();
  if (!strcmp(which, "not-fitted") || !strcmp(which, "wrong-address"))
    return runNoImu(which);
  if (!strcmp(which, "misconfigured")) return runMisconfigured();
#ifdef HARNESS_LSM6DS3
  if (!strcmp(which, "bdu-lost")) return runBduLost();
  if (!strcmp(which, "lpf1-lost")) return runLpf1Lost();
#endif
#ifdef HARNESS_MPU6050
  if (!strcmp(which, "sensor-reset")) return runSensorReset();
#endif
  printf("S %s: not applicable to this sensor\n", which);
  return 0;
}
