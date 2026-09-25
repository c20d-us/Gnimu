#!/usr/bin/env bash
# Usage: scripts/build.sh <esp32|nrf52840>
set -euo pipefail
cd "$(dirname "$0")/.."

case "${1:-}" in
  esp32)    fqbn="esp32:esp32:esp32" ;;
  nrf52840) fqbn="Seeeduino:nrf52:xiaonRF52840Sense" ;;
  *) echo "Usage: $0 <esp32|nrf52840>" >&2; exit 2 ;;
esac

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

arduino-cli compile \
  --config-file arduino-cli.yaml \
  --fqbn "$fqbn" \
  --warnings all \
  --output-dir "build/$1" \
  --build-property "compiler.cpp.extra_flags=${extra_flags}" \
  code/Gnimu
