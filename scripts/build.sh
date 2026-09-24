#!/usr/bin/env bash
# Usage: scripts/build.sh <esp32|nrf52840>
set -euo pipefail
cd "$(dirname "$0")/.."

case "${1:-}" in
  esp32)    fqbn="esp32:esp32:esp32" ;;
  nrf52840) fqbn="Seeeduino:nrf52:xiaonRF52840Sense" ;;
  *) echo "Usage: $0 <esp32|nrf52840>" >&2; exit 2 ;;
esac

arduino-cli compile \
  --config-file arduino-cli.yaml \
  --fqbn "$fqbn" \
  --warnings all \
  --output-dir "build/$1" \
  code/Gnimu
