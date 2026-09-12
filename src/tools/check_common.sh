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
# created. Forgetting VARIANTS now fails the variant sweep below; forgetting
# NRF_VARIANTS is still on you, since the script cannot tell an nRF build apart.
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
  g_protocol.h
  g_protocol_active.h
  g_proto_racebox.h
  g_proto_racebox.cpp
  g_imu_trim.h
  g_imu_trim.cpp
  g_log.h
  g_telemetry.h
  g_telemetry.cpp
  g_ubx_helpers.h
  g_ubx_helpers.cpp
  g_imu.h
  g_imu.cpp
  g_imu_sensor.h
  g_imu_tuning.h
  g_gnss.h
  g_gnss.cpp
  g_gnss_port.h
)

# --- Group 2: the two nRF52840 trees only.
NRF_VARIANTS=(
  Gnimu-nRF52840
  Gnimu-nRF52840-OLED
)

# Shared between the nRF trees. DELIBERATELY EXCLUDED, each for a real reason -
# do not "fix" these by adding them here:
#   g_led.cpp   - OLED tree yields the LED to the panel via displayIsPresent().
#   g_state.cpp - OLED tree calls displaySleep() before the MCU halts.
#
# g_power.h left this list on 2026-09-10 (ROB-6). It was excluded over two
# comment lines naming the switch-sense pin; both now say
# POWER_SWITCH_SENSE_PIN, so the file is identical and checked. The reason the
# pin differs - A4 is SDA on the OLED board, so the sense had to move to A1 -
# moved to that tree's config.h, beside the #define, where someone changing the
# pin will actually see it. It should never have been recorded only here.
#
# g_imu.cpp joined this list on 2026-08-14, when the base tree was migrated to
# the generalized IMU_AXIS_*_SRC/_SIGN remap the OLED tree already used. It was
# the last axis-scheme divergence in the codebase.
#
# On 2026-09-11 it LEFT this list for the all-variant one, with g_imu.h: the IMU
# was split into a pipeline identical on every board (g_imu.cpp) and one driver
# per sensor PART behind g_imu_sensor.h. The LSM6DS3 driver is what the two
# nRF trees share (its header, which declared only the motion-wake API, went
# with LIGHT_SLEEP on the same day); the ESP32's g_imu_mpu6050.cpp exists
# in one tree only, so it needs no list. Behaviour across the split is held by
# test/run_imu_harness.sh.
NRF_COMMON_FILES=(
  g_battery.h
  g_battery.cpp
  g_ble.h
  g_ble.cpp
  g_gnss_port_nrf52.cpp
  g_imu_lsm6ds3.cpp
  g_power.cpp
  g_power.h
  g_state.h
  g_led.h
)


# --- Group 3: files that live in two or more trees and are DELIBERATELY not
# checked. This list exists so the coverage sweep below can tell "known to
# diverge" from "nobody remembered to list it".
#
# Every entry needs a reason, above, in the group it belongs to. Adding a file
# here to silence the sweep, without a reason, defeats the point of the sweep.
EXCLUDED_FILES=(
  config.h    # per-variant by definition: pins, sensor settings, feature
              # flags. Its IMU tuning moved to g_imu_tuning.h (ARC-8).
  g_led.cpp   # see the NRF_COMMON_FILES note above.
  g_state.cpp # see the NRF_COMMON_FILES note above.
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


# --- Coverage sweep: the allowlists above cannot catch what nobody added to
# them. Invert the question - enumerate every .h/.cpp that exists in two or
# more trees, subtract everything accounted for, and fail on the remainder.
#
# This is the check that does not go stale. config.h was the proof: it sits in
# all three trees and appeared on no list, purely because it is so obviously
# per-variant that nobody thought to write it down. The exclusions you reasoned
# about are not the risk; the ones too obvious to mention are.
echo
echo "--- Coverage sweep (files in 2+ trees, accounted for?) ---"
accounted=" ${COMMON_FILES[*]} ${NRF_COMMON_FILES[*]} "
for e in "${EXCLUDED_FILES[@]}"; do accounted="$accounted$e "; done

unlisted=""
for f in $(ls "$REPO_ROOT"/src/*/*.h "$REPO_ROOT"/src/*/*.cpp 2>/dev/null |
             xargs -n1 basename | sort -u); do
  n=0
  for v in "${VARIANTS[@]}"; do
    [ -f "$REPO_ROOT/src/$v/$f" ] && n=$((n + 1))
  done
  [ "$n" -ge 2 ] || continue
  case "$accounted" in
  *" $f "*) ;;
  *) unlisted="$unlisted $f" ;;
  esac
done

if [ -n "$unlisted" ]; then
  for f in $unlisted; do
    echo "❌ UNLISTED:  $f (in 2+ trees, on no list)"
  done
  status=1
else
  echo "✅ every file shared by 2+ trees is on a list"
fi


# --- Variant sweep: a sketch folder missing from VARIANTS is invisible to every
# check above - its copies are never compared at all. Find sketch folders by the
# Arduino IDE's own rule (a folder holding a .ino of the same name) rather than
# by a name prefix, so tools/ is skipped naturally and a variant named outside
# the Gnimu-* convention cannot slip past.
#
# Only VARIANTS is enforced. Whether a new folder also belongs in NRF_VARIANTS
# depends on its MCU, which is a judgement this script cannot make.
echo
echo "--- Variant sweep (sketch folders, all in VARIANTS?) ---"
listed_variants=" ${VARIANTS[*]} "
unlisted_variants=""
for d in "$REPO_ROOT"/src/*/; do
  name="$(basename "$d")"
  [ -f "$d$name.ino" ] || continue
  case "$listed_variants" in
  *" $name "*) ;;
  *) unlisted_variants="$unlisted_variants $name" ;;
  esac
done

if [ -n "$unlisted_variants" ]; then
  for v in $unlisted_variants; do
    echo "❌ UNLISTED VARIANT: $v (a sketch folder not in VARIANTS - never checked)"
  done
  status=1
else
  echo "✅ every sketch folder is in VARIANTS"
fi


# --- Concurrency tripwire: the firmware is cooperative-polled, and the only
# code of ours that runs off loop() is BLE stack callbacks (see "Concurrency" in
# docs/architecture-runtime.md). This turns the model's strongest claim - no
# ISRs, no tasks, no locks of our own - into a checked fact rather than prose.
#
# Scans the firmware trees in VARIANTS only. tools/ is exempt on purpose: the
# model describes the shipping firmware, and a bench diagnostic is free to use
# an interrupt if it needs one. (None do today - that is a fact about now, not a
# constraint on them.)
# Whole-line // comments are ignored so the rule can be discussed in the code;
# a trailing comment naming one of these would trip it, which is loud and easy
# to fix. If you are adding one of these for real, the model is changing -
# update the doc first, then this list.
echo
echo "--- Concurrency tripwire (no ISRs, tasks or locks of our own) ---"
forbidden='attachInterrupt|xTaskCreate|xSemaphore|xQueue|portENTER_CRITICAL|taskENTER_CRITICAL|noInterrupts|__disable_irq|std::thread|std::mutex'
tripped=""
for v in "${VARIANTS[@]}"; do
  for f in "$REPO_ROOT/src/$v"/*.cpp "$REPO_ROOT/src/$v"/*.h "$REPO_ROOT/src/$v"/*.ino; do
    [ -f "$f" ] || continue
    hits=$(grep -nE "$forbidden" "$f" | grep -vE '^[0-9]+:[[:space:]]*//')
    [ -n "$hits" ] && tripped="$tripped
$v/$(basename "$f"): $hits"
  done
done

if [ -n "$tripped" ]; then
  echo "❌ CONCURRENCY: primitives outside the cooperative model:$tripped"
  status=1
else
  echo "✅ no interrupt, task, queue, semaphore or critical-section calls"
fi

if [ "$status" -eq 0 ]; then
  echo "All ${#COMMON_FILES[@]} common files are byte-identical across all ${#VARIANTS[@]} variants,"
  echo "and all ${#NRF_COMMON_FILES[@]} nRF-shared files across both nRF52840 variants."
else
  echo "COMMON-FILE DRIFT DETECTED. The common modules must stay byte-identical:"
  echo "review the differences (e.g. diff src/<variant-a>/<file> src/<variant-b>/<file>),"
  echo "pick the intended version, and copy it over the other before committing."
  echo
  echo "An UNLISTED file is a different problem: it is shared but on no list, so"
  echo "it has been drifting unchecked. Put it in COMMON_FILES / NRF_COMMON_FILES"
  echo "if the copies should match, or in EXCLUDED_FILES WITH A REASON if not."
  echo
  echo "An UNLISTED VARIANT is a sketch folder none of this script compares. Add it"
  echo "to VARIANTS - and to NRF_VARIANTS too if it is an nRF52840 build."
  echo
  echo "A CONCURRENCY hit means code outside the cooperative-polled model. If it is"
  echo "deliberate, the model is changing: update the Concurrency section of"
  echo "docs/architecture-runtime.md first, then the tripwire's list."
fi

exit $status
