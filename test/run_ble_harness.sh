#!/bin/bash
# Gnimu - GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Build and run the BLE harness: the REAL shared g_ble.cpp against a scripted
# fake port and a two-notify-channel test protocol (test/ble).
#
#   ./test/run_ble_harness.sh          compare against test/ble/golden
#   ./test/run_ble_harness.sh --save   (re)write the goldens
#
# The driver is copied into a scratch directory next to the harness's own
# g_protocol_active.h, so its #include "g_protocol_active.h" resolves to the
# test protocol rather than the variant's real one.
#
# Scenarios: begin, begin-fails, emit, multichannel, session, inbound.
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
GOLD="$ROOT/test/ble/golden"
mkdir -p "$GOLD"
SAVE=0
[ "${1:-}" = "--save" ] && SAVE=1
status=0

for V in Gnimu-ESP32 Gnimu-nRF52840 Gnimu-nRF52840-OLED; do
  B="$(mktemp -d)"
  for F in g_ble.cpp g_ble.h g_ble_port.h g_protocol.h g_log.cpp g_log.h config.h g_imu_tuning.h; do
    cp "$ROOT/src/$V/$F" "$B/"
  done
  cp "$ROOT/test/ble/g_protocol_active.h" "$B/"
  BOARD=""
  case "$V" in *nRF*) BOARD="-DARDUINO_Seeed_XIAO_nRF52840_Sense" ;; esac
  if ! c++ -std=gnu++11 -Wall -Wextra -Werror $SAN $BOARD \
       -I"$B" -I"$ROOT/test/telemetry/fakes" \
       "$B/g_ble.cpp" "$B/g_log.cpp" "$ROOT/test/ble/ble_harness.cpp" \
       -o "$B/harness" 2>"$B/build.txt"; then
    echo "❌ $V: build failed"; sed 's/^/     /' "$B/build.txt" | head -30
    status=1; rm -rf "$B"; continue
  fi

  : >"$B/out.txt"
  : >"$B/err.txt"
  rc=0
  for S in begin begin-fails emit multichannel session inbound; do
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
