#!/usr/bin/env bash
# check.sh — regenerate both schematics and verify them. Run from anywhere:
#     hardware/gen/check.sh
# 1. generate.py writes hardware/{pod,power}_board/*.kicad_sch from the board descriptions;
# 2. KiCad's own ERC must report 0 violations;
# 3. the netlists are exported and check_nets.py verifies them against firmware Config.h and the
#    cable pinout in BUILD_GUIDE 4.8.3, plus the fail-safe rules and the footprints;
# 4. a PDF of each schematic and a CSV bill of materials are exported for review;
# 5. a board drawn as wired sheets (<board>_drawing.py) is also generated label-only in a scratch
#    directory, and compare_nets.py requires the two netlists to be identical;
# 6. the assembly-day documents (docs/assembly, assembly.py) must be up to date with the boards.
# Uses KiCad 10 for Windows from WSL (kicad-cli.exe); set KICAD_DIR for another install.
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
CLI="$KICAD_DIR/bin/kicad-cli.exe"
[[ -x "$CLI" ]] || CLI="$(command -v kicad-cli)"
win() { if [[ "$CLI" == *.exe ]]; then wslpath -w "$1"; else echo "$1"; fi; }

python3 "$HW/gen/generate.py" --symbols "$KICAD_DIR/share/kicad/symbols"
for b in pod_board power_board; do
    d="$HW/$b"
    "$CLI" sch erc --exit-code-violations -o "$(win "$d")/erc.rpt" "$(win "$d/$b.kicad_sch")" \
        | grep -i "violation" || { echo "ERC FAILED for $b: see $d/erc.rpt"; exit 1; }
    "$CLI" sch export netlist -o "$(win "$d")/$b.net" "$(win "$d/$b.kicad_sch")" >/dev/null
    "$CLI" sch export pdf -o "$(win "$d")/$b.pdf" "$(win "$d/$b.kicad_sch")" >/dev/null
    "$CLI" sch export bom -o "$(win "$d")/${b}_bom.csv" \
        --fields "Reference,Value,Footprint,\${QUANTITY},Description" --group-by "Value,Footprint" \
        "$(win "$d/$b.kicad_sch")" >/dev/null
done
REF="$(mktemp -d)"
trap 'rm -rf "$REF"' EXIT
for b in pod_board power_board; do
    python3 "$HW/gen/generate.py" --symbols "$KICAD_DIR/share/kicad/symbols" --flat --out "$REF" "$b" >/dev/null
    "$CLI" sch export netlist -o "$(win "$REF")/$b.net" "$(win "$REF/$b/$b.kicad_sch")" >/dev/null
    python3 "$HW/gen/compare_nets.py" "$REF/$b.net" "$HW/$b/$b.net"
done
KICAD_FOOTPRINTS="$KICAD_DIR/share/kicad/footprints" \
    python3 "$HW/gen/check_nets.py" "$HW/pod_board/pod_board.net" "$HW/power_board/power_board.net"
python3 "$HW/gen/assembly.py" --check
