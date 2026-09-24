#!/usr/bin/env bash
# Builds and runs every host test: tests/test_*.cpp against Gnimu/src/core.
set -euo pipefail
cd "$(dirname "$0")"
out="${TMPDIR:-/tmp}/gnimu-tests"
mkdir -p "$out"
for t in test_*.cpp; do
  g++ -std=c++17 -Wall -Wextra -Werror -O1 "$t" -o "$out/${t%.cpp}"
  "$out/${t%.cpp}"
done
