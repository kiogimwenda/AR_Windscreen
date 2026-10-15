#!/usr/bin/env bash
# run_buck.sh — simulate the power board's 5 V converter (TPS54360) with TI's transient model and
# the board's component values, in the ngspice that ships with KiCad 10 (no install needed).
# Downloads TI's model on first use into hardware/sim/models/ (not committed: TI's copyright).
# Writes hardware/sim/buck/results.md and buck_results.png; fails if any check fails.
#     hardware/sim/run_buck.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 hardware/sim/buck/buck.py
