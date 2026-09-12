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

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# WHICH variant's copy is compiled does not matter: check_common.sh guarantees
# the protocol and UBX-helper modules are byte-identical across all three, and
# the harness would fail loudly if that stopped being true. nRF52840 is the
# arbitrary reference, matching check_common.sh's own choice.
SRC="src/Gnimu-nRF52840"
OUT="${TMPDIR:-/tmp}/gnimu_harness"

echo "compiling encoder from $SRC ..."
g++ -std=c++14 -Wall -Wextra -Werror -O1 \
    -I "$SRC" \
    -o "$OUT" \
    test/harness.cpp \
    "$SRC/g_proto_racebox.cpp" \
    "$SRC/g_ubx_helpers.cpp"

"$OUT" "$@"
