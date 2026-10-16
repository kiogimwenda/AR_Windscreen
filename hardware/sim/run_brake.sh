#!/usr/bin/env bash
# run_brake.sh — simulate the power board's brake-light input (PC817 optocoupler) through the
# cable to the pod's input pin, in the ngspice that ships with KiCad 10.
# Writes hardware/sim/brake/results.md and brake_results.png; fails if any check fails.
#     hardware/sim/run_brake.sh [--reuse]
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 hardware/sim/brake/brake.py "$@"
