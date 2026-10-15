#!/usr/bin/env bash
# run_relay.sh — simulate the power board's relay and magnet drivers (AO3400A + SS14 flyback,
# driven from the pod through the cable) in the ngspice that ships with KiCad 10.
# Writes hardware/sim/relay/results.md and relay_results.png; fails if any check fails.
#     hardware/sim/run_relay.sh [--reuse]
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 hardware/sim/relay/relay.py "$@"
