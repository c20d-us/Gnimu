#!/usr/bin/env bash
# Gnimu - GNSS harness runner.
#
# Compiles each variant's REAL g_gnss.cpp - unmodified - with its real config.h,
# against a fake u-blox receiver (test/gnss). See gnss_harness.cpp for what each
# scenario covers and why the goldens hold call sequences rather than log text.
#
#   ./test/run_gnss_harness.sh          run, and compare against goldens
#   ./test/run_gnss_harness.sh --save   (re)write the golden output
#
# Scenarios: at-target, at-9600, absent, verify-fails, config-rejects, epochs.
#
# Built BEFORE the shared-core split, which it then held to "the receiver sees
# the same calls in the same order". Since the split it compiles the shared
# driver plus that variant's port file (g_gnss_port_<mcu>.cpp).
#
# Needs the SparkFun u-blox GNSS v3 library for u-blox_structs.h, which
# compiles standalone (ROB-6). Override with SPARKFUN_UBLOX_SRC.
set -euo pipefail
# Sanitizers (R2-5): AddressSanitizer and UndefinedBehaviorSanitizer, made fatal.
# They catch a memory error even when it lands in unused stack and changes no
# output - which a golden cannot see - and they change nothing else: every
# golden is identical with them on. Blank this on a machine that cannot link
# them (on Linux, LeakSanitizer may also want ASAN_OPTIONS=detect_leaks=0).
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"

# Print why a harness process failed. For a sanitizer abort the useful part is
# the head of the report - the error, the first stack frames, the SUMMARY -
# not its tail, which is the shadow-memory legend. Anything else: the tail.
showFailure() {
  if grep -qE 'ERROR: |runtime error: ' "$1"; then
    grep -E 'ERROR: |SUMMARY: |runtime error: |^ +#[0-3] ' "$1" | head -8 | sed 's/^/     /'
  else
    tail -10 "$1" | sed 's/^/     /'
  fi
}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
SPARKFUN="${SPARKFUN_UBLOX_SRC:-$HOME/Documents/Arduino/libraries/SparkFun_u-blox_GNSS_v3/src}"
GOLD="$ROOT/test/gnss/golden"
mkdir -p "$GOLD"
SAVE=0
[ "${1:-}" = "--save" ] && SAVE=1
status=0

for V in Gnimu-ESP32 Gnimu-nRF52840 Gnimu-nRF52840-OLED; do
  B="$(mktemp -d)"
  BOARD=""
  PORT="$ROOT/src/$V/g_gnss_port_esp32.cpp"
  case "$V" in
    *nRF*) BOARD="-DARDUINO_Seeed_XIAO_nRF52840_Sense"
           PORT="$ROOT/src/$V/g_gnss_port_nrf52.cpp" ;;
  esac
  # Fakes first, so the stand-ins for Arduino and the SparkFun library shadow
  # the real ones; then test/gnss for the harness's own headers.
  if ! c++ -std=gnu++11 -Wall -Wextra -Werror $SAN $BOARD \
       -I"$ROOT/test/gnss/fakes" -I"$ROOT/src/$V" -I"$SPARKFUN" \
       "$ROOT/test/gnss/gnss_harness.cpp" "$ROOT/src/$V/g_gnss.cpp" "$PORT" \
       "$ROOT/src/$V/g_log.cpp" \
       -o "$B/harness" 2>"$B/build.txt"; then
    echo "❌ $V: build failed"; sed 's/^/     /' "$B/build.txt" | head -30
    status=1; rm -rf "$B"; continue
  fi

  : >"$B/out.txt"
  : >"$B/err.txt"
  rc=0
  for S in at-target at-9600 absent verify-fails config-rejects epochs silent; do
    # stderr kept apart: the golden is stdout only, but a sanitizer report
    # lands on stderr and must be shown when a scenario fails.
    "$B/harness" "$S" >>"$B/out.txt" 2>>"$B/err.txt" || rc=$?
  done
  G="$GOLD/$V.txt"
  if [ $rc -ne 0 ]; then
    echo "❌ $V: a scenario failed (exit $rc):"
    grep '^❌' "$B/out.txt" | sed 's/^/     /' || true
    showFailure "$B/err.txt"
    status=1
  elif [ $SAVE = 1 ]; then
    cp "$B/out.txt" "$G"
    echo "saved   $V  ($(wc -l <"$G" | tr -d ' ') lines)"
  elif cmp -s "$B/out.txt" "$G"; then
    echo "✅ $V  all scenarios pass, identical to golden"
  else
    echo "❌ $V  assertions pass, but DIFFERS from golden:"
    diff "$G" "$B/out.txt" | head -20 | sed 's/^/     /' || true
    status=1
  fi
  rm -rf "$B"
done
exit $status
