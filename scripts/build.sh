#!/usr/bin/env bash
# Usage: scripts/build.sh [esp32|nrf52840]   (no argument: both boards)
#
# Builds the firmware with the real Arduino toolchain and fails on any compiler
# warning in the project's own sources (code/Gnimu). Warnings from the board
# cores and vendor libraries are shown but don't fail: they aren't ours to fix,
# and Arduino has no sketch-only -Werror (compiler.cpp.extra_flags reaches the
# core and every library too).
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ $# -eq 0 ]]; then
  "$0" esp32
  "$0" nrf52840
  exit
fi

case "$1" in
  esp32)    fqbn="esp32:esp32:esp32" ;;
  nrf52840) fqbn="Seeeduino:nrf52:xiaonRF52840Sense" ;;
  *) echo "Usage: $0 [esp32|nrf52840]" >&2; exit 2 ;;
esac

# $ARDUINO_CLI if set, else arduino-cli on the PATH, else the copy bundled with
# the Arduino IDE on macOS.
cli="${ARDUINO_CLI:-}"
if [[ -z "$cli" ]]; then
  if command -v arduino-cli >/dev/null 2>&1; then
    cli="arduino-cli"
  else
    cli="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
  fi
fi
if ! command -v "$cli" >/dev/null 2>&1; then
  echo "arduino-cli not found; install it or set ARDUINO_CLI" >&2
  exit 2
fi

revision="$(git rev-parse --short=12 HEAD 2>/dev/null || printf 'unknown')"
version="$(git describe --tags --always --dirty 2>/dev/null || printf '0.0.0-dev')"
if [[ -n "$(git status --porcelain --untracked-files=normal 2>/dev/null)" ]]; then
  dirty=1
else
  dirty=0
fi
build_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

extra_flags="-DGNIMU_VERSION=\"${version}\" -DGNIMU_REVISION=\"${revision}\""
extra_flags+=" -DGNIMU_DIRTY=\"${dirty}\" -DGNIMU_BUILD_UTC=\"${build_utc}\""

sketch="$PWD/code/Gnimu"
log="$(mktemp)"
trap 'rm -f "$log"' EXIT

# --clean: every source is recompiled, so every warning is printed. An
# incremental build skips unchanged files, and their warnings with them.
"$cli" compile \
  --clean \
  --config-file arduino-cli.yaml \
  --fqbn "$fqbn" \
  --warnings all \
  --output-dir "build/$1" \
  --build-property "compiler.cpp.extra_flags=${extra_flags}" \
  "$sketch" 2>&1 | tee "$log"

# Our warnings, found by the sketch's own path, which arduino-cli reports for
# every project source (the .ino included). grep finding none is the pass.
if ours="$(grep -F "$sketch/" "$log" | grep -F ': warning:')"; then
  echo >&2
  echo "$1: compiler warnings in project sources:" >&2
  echo "$ours" >&2
  exit 1
fi
echo "$1: no warnings in project sources"
