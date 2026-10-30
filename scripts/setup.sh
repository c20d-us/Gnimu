#!/usr/bin/env bash
# Gnimu - GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/setup.sh
#
# Prepares a machine to build with arduino-cli: checks the tools the build
# scripts rely on, then installs the board packages and libraries at the
# versions the firmware is built and tested against. Safe to run again; what is
# already installed at the right version is left alone.
set -euo pipefail
cd "$(dirname "$0")/.."

esp32_core="esp32:esp32@3.3.12"
seeed_core="Seeeduino:nrf52@1.1.13"
libs=("SparkFun u-blox GNSS v3@3.1.15" "Seeed Arduino LSM6DS3@2.0.7")

problems=0
problem() {
  echo "  - $1" >&2
  problems=1
}

echo "Checking tools"
for tool in git g++ python3; do
  command -v "$tool" >/dev/null 2>&1 || problem "$tool not found; on macOS run: xcode-select --install"
done
command -v arduino-cli >/dev/null 2>&1 || problem "arduino-cli not found; on macOS run: brew install arduino-cli"

if [[ "$(uname -s)" == Darwin && "$(uname -m)" == arm64 ]]; then
  /usr/bin/arch -x86_64 /usr/bin/true >/dev/null 2>&1 ||
    problem "Rosetta not installed; run: softwareupdate --install-rosetta --agree-to-license"
fi

# The Seeed nRF52 core's platform.txt runs bare `python`, which a clean macOS
# lacks, and the file is overwritten on every core update, so the fix belongs on
# the PATH rather than in the core.
command -v python >/dev/null 2>&1 ||
  problem "python not found, which the Seeed nRF52 core needs. With Homebrew's python installed, add to your zsh startup file (such as ~/.zshrc): export PATH=\"\$(brew --prefix python)/libexec/bin:\$PATH\""

if [[ $problems -ne 0 ]]; then
  echo "Fix the above and run $0 again" >&2
  exit 1
fi

echo "Installing board packages"
arduino-cli core update-index --config-file arduino-cli.yaml
arduino-cli core install "$esp32_core" --config-file arduino-cli.yaml
arduino-cli core install "$seeed_core" --config-file arduino-cli.yaml

echo "Installing libraries"
for lib in "${libs[@]}"; do
  arduino-cli lib install "$lib"
done

echo "Setup complete; build with scripts/build.sh"
