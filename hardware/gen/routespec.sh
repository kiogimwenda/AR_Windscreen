#!/usr/bin/env bash
# routespec.sh — check the routed boards against the specification points KiCad's DRC does not
# know (routespec.py). Run after route.sh:  hardware/gen/routespec.sh [pod_board] [power_board]
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
boards=("$@")
[[ ${#boards[@]} -gt 0 ]] || boards=(pod_board power_board)
fail=0
for b in "${boards[@]}"; do
    echo "$b:"
    set +e
    "$KICAD_DIR/bin/python.exe" "$(wslpath -w "$HW/gen/routespec.py")" "$(wslpath -w "$HW/$b/$b.kicad_pcb")" "$b" \
        2>&1 | grep -v "image handler"
    [[ ${PIPESTATUS[0]} == 0 ]] || fail=1
    set -e
done
exit $fail
