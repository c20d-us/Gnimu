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

# ============================================================================
# Build and run the host harness against the golden vectors.
#
# Compiles the REAL firmware encoder - g_proto_racebox.cpp and g_ubx_helpers.cpp
# straight out of a sketch folder - and checks its output against the vectors
# produced in phase A.
#
# Run from anywhere:  ./test/run_harness.sh
# Extra arguments are passed through as vector files, overriding the defaults.
# ============================================================================

set -eu

# Sanitizers (R2-5): AddressSanitizer and UndefinedBehaviorSanitizer, made fatal.
# They catch a memory error even when it lands in unused stack and changes no
# output - which a golden cannot see - and they change nothing else: every
# golden is identical with them on. Blank this on a machine that cannot link
# them (on Linux, LeakSanitizer may also want ASAN_OPTIONS=detect_leaks=0).
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# WHICH variant's copy is compiled does not matter: check_common.sh guarantees
# the protocol and UBX-helper modules are byte-identical across all three, and
# the harness would fail loudly if that stopped being true. nRF52840 is the
# arbitrary reference, matching check_common.sh's own choice.
SRC="src/Gnimu-nRF52840"
OUT="${TMPDIR:-/tmp}/gnimu_harness"

echo "compiling encoder from $SRC ..."
g++ -std=c++14 -Wall -Wextra -Werror -O1 $SAN \
    -I "$SRC" \
    -o "$OUT" \
    test/harness.cpp \
    "$SRC/g_proto_racebox.cpp" \
    "$SRC/g_ubx_helpers.cpp"

"$OUT" "$@"
