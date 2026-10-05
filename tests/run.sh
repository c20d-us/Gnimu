#!/usr/bin/env bash
# Builds and runs every host test: tests/test_*.cpp against Gnimu/src/core.
# Each test runs twice: a plain build (closest to the firmware's codegen) and
# one under AddressSanitizer + UndefinedBehaviorSanitizer, where the first
# finding fails the run (e.g. a signed-overflow negation a plain build passes).
set -euo pipefail
cd "$(dirname "$0")"
out="${TMPDIR:-/tmp}/gnimu-tests"
mkdir -p "$out"
core="../code/Gnimu/src/core"
flags=(-std=c++17 -Wall -Wextra -Werror -O1)
sanitize=(-g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
for t in test_*.cpp; do
  name="${t%.cpp}"
  # core/*.cpp (non-header-only pieces, e.g. ImuTrim) linked into every test;
  # harmless when unused.
  g++ "${flags[@]}" "$t" "$core"/*.cpp -o "$out/$name"
  "$out/$name"
  g++ "${flags[@]}" "${sanitize[@]}" "$t" "$core"/*.cpp -o "$out/$name-san"
  "$out/$name-san" > /dev/null
done
echo "all suites passed (plain and sanitized)"
