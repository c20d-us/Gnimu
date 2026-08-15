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
# Every variant sketch deliberately carries duplicate copies of the modules
# below (a shared-library approach was evaluated and rejected as too convoluted
# for the Arduino build model). The duplication contract is: a change to one
# copy MUST be applied to all the others. This script enforces that contract -
# it exits 0 when every file is byte-identical across every variant and 1
# otherwise.
#
# There are TWO check groups, because the sharing is not uniform:
#
#   ALL-VARIANT  - modules every sketch carries, including the ESP32 tree.
#   NRF-ONLY     - modules the two nRF52840 trees share but the ESP32 tree has
#                  no counterpart for (different MCU, sensor, and power model:
#                  no battery gauge, no state machine, no power gate). These
#                  were drifting unchecked until 2026-08-14.
#
# Add a variant to the relevant *_VARIANTS list when a new sketch folder is
# created, or its copies go unchecked and drift silently.
#
# Run it from anywhere:  ./src/tools/check_common.sh
# ============================================================================

set -u

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# --- Group 1: every sketch folder holding a copy of the common set. The first
# entry is the reference the others are compared against - which one it is does
# not matter, since the contract is that all are identical.
VARIANTS=(
  Gnimu-ESP32
  Gnimu-nRF52840
  Gnimu-nRF52840-OLED
)

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

# --- Group 2: the two nRF52840 trees only.
NRF_VARIANTS=(
  Gnimu-nRF52840
  Gnimu-nRF52840-OLED
)

# Shared between the nRF trees. DELIBERATELY EXCLUDED, each for a real reason -
# do not "fix" these by adding them here:
#   g_imu.cpp   - OLED tree uses the generalized IMU_AXIS_*_SRC/_SIGN remap,
#                 the base tree still uses IMU_SWAP_XY/IMU_SIGN_*.
#   g_led.cpp   - OLED tree yields the LED to the panel via displayIsPresent().
#   g_state.cpp - OLED tree calls displaySleep() before the MCU halts.
#   g_power.h   - switch-sense pin differs (A4 base / A1 OLED; A4 IS SDA there).
NRF_COMMON_FILES=(
  g_battery.h
  g_battery.cpp
  g_ble.h
  g_ble.cpp
  g_gnss.h
  g_gnss.cpp
  g_power.cpp
  g_state.h
  g_led.h
)

status=0

# Compare one file set across one variant set. Args: <label> <ref> <file-count>
# then the files, then the variants - passed as two flattened lists because
# bash cannot pass arrays directly.
check_group() {
  local label="$1" ref="$2" nfiles="$3"
  shift 3
  local files=("${@:1:$nfiles}")
  local variants=("${@:$nfiles+1}")
  local f v missing differs

  for f in "${files[@]}"; do
    # Report every variant missing the file, not just the first, so one run
    # surfaces the whole picture.
    missing=""
    for v in "${variants[@]}"; do
      [ -f "$REPO_ROOT/src/$v/$f" ] || missing="$missing $v"
    done
    if [ -n "$missing" ]; then
      echo "❌ MISSING:   $f (absent in:$missing)"
      status=1
      continue
    fi

    # Same for mismatches - name every variant that differs from the reference.
    differs=""
    for v in "${variants[@]}"; do
      [ "$v" = "$ref" ] && continue
      cmp -s "$REPO_ROOT/src/$ref/$f" "$REPO_ROOT/src/$v/$f" || differs="$differs $v"
    done
    if [ -n "$differs" ]; then
      echo "❌ DIFFERS:   $f (vs $ref:$differs)"
      status=1
    else
      echo "✅ identical: $f"
    fi
  done
}

echo "--- All variants (${#VARIANTS[@]}) ---"
check_group "all" "${VARIANTS[0]}" "${#COMMON_FILES[@]}" \
  "${COMMON_FILES[@]}" "${VARIANTS[@]}"

echo
echo "--- nRF52840 trees only (${#NRF_VARIANTS[@]}) ---"
check_group "nrf" "${NRF_VARIANTS[0]}" "${#NRF_COMMON_FILES[@]}" \
  "${NRF_COMMON_FILES[@]}" "${NRF_VARIANTS[@]}"

echo
if [ "$status" -eq 0 ]; then
  echo "All ${#COMMON_FILES[@]} common files are byte-identical across all ${#VARIANTS[@]} variants,"
  echo "and all ${#NRF_COMMON_FILES[@]} nRF-shared files across both nRF52840 variants."
else
  echo "COMMON-FILE DRIFT DETECTED. The common modules must stay byte-identical:"
  echo "review the differences (e.g. diff src/<variant-a>/<file> src/<variant-b>/<file>),"
  echo "pick the intended version, and copy it over the other before committing."
fi

exit $status
