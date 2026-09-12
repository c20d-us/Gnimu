#!/usr/bin/env bash
# Gnimu - telemetry harness runner.
#
# Compiles each variant's REAL g_telemetry.cpp - unmodified - with its real
# config.h and module headers, the real RaceBox encoder, and the real g_log.h,
# against the fakes in test/telemetry. See test/telemetry/telemetry_harness.cpp
# for what each mode covers.
#
#   ./test/run_telemetry_harness.sh          run, and compare against goldens
#   ./test/run_telemetry_harness.sh --save   (re)write the golden output
#
# Per variant:
#   vectors     every golden vector through telemetrySendIfReady() (pass/fail)
#   invariants  frames per epoch, and the IMU latch per epoch (pass/fail)
#   rates       NEW-3's epoch-to-epoch rate measurement (golden + assertions)
#   stats       the 1 Hz serial stats line (golden + assertions)
#
# Its own runner rather than a mode of run_harness.sh, which passes its
# arguments through as vector files and so has no room for --save.
#
# Needs the SparkFun u-blox GNSS v3 library for u-blox_structs.h, which
# compiles standalone (see ROB-6). Override the path with SPARKFUN_UBLOX_SRC.
# The captured vectors (test/vectors.gc1) are gitignored; without them the
# vectors mode runs the committed synthetic set alone.
#
# Floating point is compiled with -ffp-contract=off so the output is a function
# of the source alone. Goldens are host output: if they mismatch on a different
# compiler, regenerate them from a known-good commit with --save.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" # the vector loader reads test/*.gc1 relative to the repo root
SPARKFUN="${SPARKFUN_UBLOX_SRC:-$HOME/Documents/Arduino/libraries/SparkFun_u-blox_GNSS_v3/src}"
GOLD="$ROOT/test/telemetry/golden"
mkdir -p "$GOLD"
SAVE=0
[ "${1:-}" = "--save" ] && SAVE=1
status=0

for V in Gnimu-ESP32 Gnimu-nRF52840 Gnimu-nRF52840-OLED; do
  B="$(mktemp -d)"
  BOARD=""
  case "$V" in *nRF*) BOARD="-DARDUINO_Seeed_XIAO_nRF52840_Sense" ;; esac
  # Include order matters: the fakes first, so the SparkFun stand-in shadows
  # the real library header; then test/ for gc1.h; then the variant itself.
  if ! c++ -std=gnu++11 -Wall -Wextra -Werror -ffp-contract=off $BOARD \
       -I"$ROOT/test/telemetry/fakes" -I"$ROOT/test" -I"$ROOT/src/$V" -I"$SPARKFUN" \
       "$ROOT/test/telemetry/telemetry_harness.cpp" \
       "$ROOT/src/$V/g_telemetry.cpp" "$ROOT/src/$V/g_proto_racebox.cpp" \
       "$ROOT/src/$V/g_ubx_helpers.cpp" -o "$B/harness" 2>"$B/build.txt"; then
    echo "❌ $V: build failed"; sed 's/^/     /' "$B/build.txt" | head -30
    status=1; rm -rf "$B"; continue
  fi

  for MODE in vectors invariants; do
    if "$B/harness" "$MODE" >"$B/out.txt" 2>&1; then
      echo "✅ $V $MODE  $(tail -1 "$B/out.txt" | sed 's/^✅ //')"
    else
      echo "❌ $V $MODE  failed:"; sed 's/^/     /' "$B/out.txt" | tail -25
      status=1
    fi
  done

  for MODE in rates stats; do
    rc=0
    "$B/harness" "$MODE" >"$B/out.txt" 2>&1 || rc=$?
    G="$GOLD/$V-$MODE.txt"
    if [ $rc -ne 0 ]; then
      # An assertion failed: never save over a golden with a failing run.
      echo "❌ $V $MODE  assertion failed:"; grep '^❌' "$B/out.txt" | sed 's/^/     /'
      status=1
    elif [ $SAVE = 1 ]; then
      cp "$B/out.txt" "$G"
      echo "saved   $V $MODE  ($(wc -l <"$G" | tr -d ' ') lines)"
    elif cmp -s "$B/out.txt" "$G"; then
      echo "✅ $V $MODE  assertions pass, identical to golden"
    else
      echo "❌ $V $MODE  assertions pass, but DIFFERS from golden:"
      # `|| true`: diff exits 1 on a difference, which under pipefail + set -e
      # would abort the whole run at the first mismatch and hide the rest.
      diff "$G" "$B/out.txt" | head -12 | sed 's/^/     /' || true
      status=1
    fi
  done
  rm -rf "$B"
done
exit $status
