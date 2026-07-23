#!/bin/bash
# Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
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

# ============================================================================
# check_common.sh - verify the cross-variant common modules are byte-identical.
#
# The ESP32 and nRF52840 sketches deliberately carry duplicate copies of the
# modules below (a shared-library approach was evaluated and rejected as too
# convoluted for the Arduino build model). The duplication contract is: a
# change to one copy MUST be applied to the other. This script enforces that
# contract - it exits 0 when every pair is byte-identical and 1 otherwise.
#
# Run it from anywhere:  ./tools/check_common.sh
# ============================================================================

set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ESP32_DIR="$REPO_ROOT/src/ESP32/Gnimu"
NRF_DIR="$REPO_ROOT/src/nRF52840/Gnimu"

# The designated common set. Add a file here if it becomes shared; remove it
# if it is deliberately allowed to diverge.
COMMON_FILES=(
  ImuAxis.h
  ImuAxis.cpp
  g_log.h
  g_telemetry.h
  g_telemetry.cpp
  g_ubx_helpers.h
  g_ubx_helpers.cpp
)

status=0
for f in "${COMMON_FILES[@]}"; do
  esp32="$ESP32_DIR/$f"
  nrf="$NRF_DIR/$f"
  if [ ! -f "$esp32" ] || [ ! -f "$nrf" ]; then
    echo "❌ MISSING:   $f (esp32: $([ -f "$esp32" ] && echo present || echo absent), nrf52840: $([ -f "$nrf" ] && echo present || echo absent))"
    status=1
  elif cmp -s "$esp32" "$nrf"; then
    echo "✅ identical: $f"
  else
    echo "❌ DIFFERS:   $f"
    status=1
  fi
done

echo
if [ "$status" -eq 0 ]; then
  echo "All ${#COMMON_FILES[@]} common files are byte-identical across variants."
else
  echo "COMMON-FILE DRIFT DETECTED. The common modules must stay byte-identical:"
  echo "review the differences (e.g. diff src/ESP32/Gnimu/<file> src/nRF52840/Gnimu/<file>),"
  echo "pick the intended version, and copy it over the other before committing."
fi

exit $status
