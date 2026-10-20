#!/usr/bin/env bash
# Gnimu - GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/flash.sh <esp32|nrf52840> [port] [--build]
#                         [--hardware <name>] [--protocol <name>]
#
# Uploads the firmware scripts/build.sh last built for that board and
# combination, from build/<board> (or build/<board>-classic, ...). Pass the
# same --hardware and --protocol the build used. It doesn't build unless asked
# to with --build, which runs build.sh with the same options first and stops,
# flashing nothing, if the build fails. Either way, what's flashed is exactly
# what was built, stamp and all.
#
# Without a port, uses the one connected board that identifies itself as this
# board type. The XIAO does; an ESP32 dev board's USB-serial chip doesn't, so
# the ESP32 needs its port named.
set -euo pipefail
cd "$(dirname "$0")/.."

usage() {
  echo "Usage: $0 <esp32|nrf52840> [port] [--build] [--hardware <name>] [--protocol <name>]" >&2
  exit 2
}
build=0
rest=()
for arg in "$@"; do
  if [[ "$arg" == --build ]]; then build=1; else rest+=("$arg"); fi
done
set -- ${rest[@]+"${rest[@]}"}
source scripts/variant.sh
[[ ${#positional[@]} -ge 1 && ${#positional[@]} -le 2 ]] || usage
board="${positional[0]}"

case "$board" in
  esp32)    fqbn="esp32:esp32:esp32";                image="Gnimu.ino.bin" ;;
  nrf52840) fqbn="Seeeduino:nrf52:xiaonRF52840Sense"; image="Gnimu.ino.zip" ;;
  *) usage ;;
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

resolve_variant "$board"

if [[ $build -eq 1 ]]; then
  build_args=("$board")
  if [[ -n "$hw_name" ]]; then build_args+=(--hardware "$hw_name"); fi
  if [[ -n "$proto_name" ]]; then build_args+=(--protocol "$proto_name"); fi
  scripts/build.sh "${build_args[@]}" || {
    echo "$board$variant: build failed; nothing flashed" >&2
    exit 1
  }
fi

dir="build/$board$variant"
if [[ ! -f "$dir/$image" ]]; then
  echo "$dir/$image not found; run scripts/build.sh with the same options first" >&2
  exit 1
fi

port="${positional[1]:-}"
if [[ -z "$port" ]]; then
  boards="$("$cli" board list --config-file arduino-cli.yaml)"
  # The port is the first column of each row naming this board type.
  matches="$(grep -F "$fqbn" <<<"$boards" | awk '{print $1}' || true)"
  count="$(grep -c . <<<"$matches" || true)"
  if [[ "$count" -ne 1 ]]; then
    echo "Found $count connected boards identifying as $fqbn; name the port:" >&2
    echo "  $0 $board <port>" >&2
    echo >&2
    echo "$boards" >&2
    exit 1
  fi
  port="$matches"
fi

echo "Flashing $dir/$image to $port"
"$cli" upload \
  --config-file arduino-cli.yaml \
  --fqbn "$fqbn" \
  --port "$port" \
  --input-dir "$dir"
echo "$board$variant: flashed"
