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
# Add a variant to VARIANTS below when a new sketch folder is created, or its
# copies go unchecked and drift silently.
#
# Run it from anywhere:  ./src/tools/check_common.sh
# ============================================================================

set -u

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# Every sketch folder holding a copy of the common set. The first entry is the
# reference the others are compared against - which one it is does not matter,
# since the contract is that all are identical.
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

status=0
ref="${VARIANTS[0]}"

for f in "${COMMON_FILES[@]}"; do
  # Report every variant missing the file, not just the first, so one run
  # surfaces the whole picture.
  missing=""
  for v in "${VARIANTS[@]}"; do
    [ -f "$REPO_ROOT/src/$v/$f" ] || missing="$missing $v"
  done
  if [ -n "$missing" ]; then
    echo "❌ MISSING:   $f (absent in:$missing)"
    status=1
    continue
  fi

  # Same for mismatches - name every variant that differs from the reference.
  differs=""
  for v in "${VARIANTS[@]}"; do
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

echo
if [ "$status" -eq 0 ]; then
  echo "All ${#COMMON_FILES[@]} common files are byte-identical across all ${#VARIANTS[@]} variants."
else
  echo "COMMON-FILE DRIFT DETECTED. The common modules must stay byte-identical:"
  echo "review the differences (e.g. diff src/$ref/<file> src/<other-variant>/<file>),"
  echo "pick the intended version, and copy it over the other before committing."
fi

exit $status
