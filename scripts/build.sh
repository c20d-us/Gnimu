#!/usr/bin/env bash
# Gnimu - GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/build.sh [esp32|nrf52840] [--hardware <name>]
#                         [--protocol <name>]
#        scripts/build.sh --matrix
#
# No board builds both. --hardware and --protocol pick profile files from
# code/Gnimu/src/variants (hardware profiles are per board) without editing a
# tracked file, so a tagged commit builds clean, `dirty=0`, in every
# combination. Left out, the build uses the default profiles. A build with
# options goes to its own directory, such as build/nrf52840-classic.
# --matrix builds every board with every hardware and protocol profile.
#
# Builds the firmware with the real Arduino toolchain and fails on any compiler
# warning in the project's own sources (code/Gnimu). Warnings from the board
# cores and vendor libraries are shown but don't fail: they aren't ours to fix,
# and Arduino has no sketch-only -Werror (compiler.cpp.extra_flags reaches the
# core and every library too).
set -euo pipefail
cd "$(dirname "$0")/.."

usage() {
  echo "Usage: $0 [esp32|nrf52840] [--hardware <name>] [--protocol <name>]" >&2
  echo "       $0 --matrix" >&2
  exit 2
}

if [[ "${1:-}" == --matrix ]]; then
  [[ $# -eq 1 ]] || usage
  set --
  source scripts/variant.sh
  for board in esp32 nrf52840; do
    for hw in $(variant_names "$variants_dir/$board/hardware"); do
      for proto in $(variant_names "$variants_dir/protocol"); do
        "$0" "$board" --hardware "$hw" --protocol "$proto"
      done
    done
  done
  exit
fi

source scripts/variant.sh
[[ ${#positional[@]} -le 1 ]] || usage

if [[ ${#positional[@]} -eq 0 ]]; then
  if [[ -n "$hw_name" ]]; then
    echo "Hardware profiles are per board; name the board" >&2
    exit 2
  fi
  if [[ -n "$proto_name" ]]; then
    "$0" esp32 --protocol "$proto_name"
    "$0" nrf52840 --protocol "$proto_name"
  else
    "$0" esp32
    "$0" nrf52840
  fi
  exit
fi
board="${positional[0]}"

case "$board" in
  esp32)    fqbn="esp32:esp32:esp32" ;;
  nrf52840) fqbn="Seeeduino:nrf52:xiaonRF52840Sense" ;;
  *) usage ;;
esac
resolve_variant "$board"
out="$board$variant"

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
extra_flags+="${variant_flags}"

sketch="$PWD/code/Gnimu"
log="$(mktemp)"
trap 'rm -f "$log"' EXIT

# --clean: every source is recompiled, so every warning is printed. An
# incremental build skips unchanged files, and their warnings with them.
# --build-path: its own build folder, not the per-sketch one the Arduino IDE
# also uses. --clean wipes the build folder, and a shared one would break an
# IDE build running at the same time (and the IDE could break this one).
"$cli" compile \
  --clean \
  --build-path "build/work/$out" \
  --config-file arduino-cli.yaml \
  --fqbn "$fqbn" \
  --warnings all \
  --output-dir "build/$out" \
  --build-property "compiler.cpp.extra_flags=${extra_flags}" \
  "$sketch" 2>&1 | tee "$log"

# Our warnings, found by the sketch's own path, which arduino-cli reports for
# every project source (the .ino included). grep finding none is the pass.
if ours="$(grep -F "$sketch/" "$log" | grep -F ': warning:')"; then
  echo >&2
  echo "$out: compiler warnings in project sources:" >&2
  echo "$ours" >&2
  exit 1
fi
echo "$out: no warnings in project sources"
