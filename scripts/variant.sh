# Gnimu - GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Sourced by build.sh and flash.sh, which must define usage() first.
#
# Reads the options both scripts share, --hardware <name> and
# --protocol <name>, from "$@" and sets:
#   positional   the arguments that aren't options, in order
#   hw_name      the --hardware value, lowercased, or "" if not given
#   proto_name   the --protocol value, lowercased, or "" if not given
#
# resolve_variant <board>, called once the board is known, checks the names
# against the profile files in code/Gnimu/src/variants and sets:
#   variant_flags  the compiler flags selecting those files (empty for the
#                  defaults, which Config.h and BoardConfig.h supply)
#   variant        the suffix that keeps each combination's build output
#                  apart: "" for the defaults, else "-classic", "-classic-
#                  racechrono", ...
# The names are whatever files exist there, so adding a variant is adding a
# file, not editing a script.

variants_dir=code/Gnimu/src/variants
hw_name=""
proto_name=""
positional=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --hardware|--protocol)
      [[ $# -ge 2 ]] || usage
      value="$(printf '%s' "$2" | tr '[:upper:]' '[:lower:]')"
      # Names become file names and compiler flags: keep them plain.
      [[ "$value" =~ ^[a-z0-9][a-z0-9_-]*$ ]] || { echo "Bad name '$2'" >&2; exit 2; }
      if [[ "$1" == --hardware ]]; then hw_name="$value"; else proto_name="$value"; fi
      shift 2 ;;
    -*) usage ;;
    *) positional+=("$1"); shift ;;
  esac
done

# The names of the .h files in a folder, space separated.
variant_names() {
  local f names=""
  for f in "$1"/*.h; do
    f="${f##*/}"
    names+="${names:+ }${f%.h}"
  done
  printf '%s' "$names"
}

resolve_variant() {
  variant_flags=""
  variant=""
  if [[ -n "$hw_name" ]]; then
    if [[ ! -f "$variants_dir/$1/hardware/$hw_name.h" ]]; then
      echo "Unknown hardware '$hw_name' for $1; available: $(variant_names "$variants_dir/$1/hardware")" >&2
      exit 2
    fi
    variant_flags+=" -DGNIMU_HARDWARE_FILE=\"$1/hardware/$hw_name.h\""
    variant+="-$hw_name"
  fi
  if [[ -n "$proto_name" ]]; then
    if [[ ! -f "$variants_dir/protocol/$proto_name.h" ]]; then
      echo "Unknown protocol '$proto_name'; available: $(variant_names "$variants_dir/protocol")" >&2
      exit 2
    fi
    variant_flags+=" -DGNIMU_PROTOCOL_FILE=\"src/variants/protocol/$proto_name.h\""
    variant+="-$proto_name"
  fi
}
