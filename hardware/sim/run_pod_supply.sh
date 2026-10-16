#!/usr/bin/env bash
# run_pod_supply.sh — simulate the pod's supply (cable +5 V and USB diode-OR, 3.3 V LDO)
# in the ngspice that ships with KiCad 10.
# Writes hardware/sim/pod_supply/results.md and pod_supply_results.png; fails if any check fails.
#     hardware/sim/run_pod_supply.sh [--reuse]
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 hardware/sim/pod_supply/pod_supply.py "$@"
