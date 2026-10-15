#!/usr/bin/env bash
# run_input.sh — simulate the power board's 12 V input protection (fuse, TVS, reverse-polarity
# diode) against the car's disturbances (ISO 16750-2, ISO 7637-2) in KiCad 10's ngspice.
# Writes hardware/sim/input/results.md and input_results.png; fails if any check fails.
#     hardware/sim/run_input.sh [--reuse]
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 hardware/sim/input/input.py "$@"
