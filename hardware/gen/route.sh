#!/usr/bin/env bash
# route.sh — route the placed boards (route.py: rule areas, ground fan-out, Freerouting, ground
# pours, stitching and mending), then KiCad's DRC with the schematic-parity check. Run from anywhere:
#     hardware/gen/route.sh [pod_board] [power_board]
# Each attempt starts from a fresh placement (layout.sh --force). Freerouting's result varies from
# run to run (it routes in parallel), so a board that does not pass is routed again, up to
# ROUTE_ATTEMPTS times (default 5); the first that passes is kept. It passes only with 0 DRC
# errors, 0 unconnected items and exact schematic parity. Needs Freerouting and Java 25 (see
# routecheck.sh) and KiCad 10 for Windows.
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
JAVA="${JAVA:-/mnt/c/Program Files/Java/jdk-25.0.2/bin/java.exe}"
JAR="${FREEROUTING_JAR:-$HOME/.local/share/freerouting/freerouting-2.5.0.jar}"
WORK="${ROUTE_DIR:-$HW/../build/route}"
ATTEMPTS="${ROUTE_ATTEMPTS:-5}"
win() { wslpath -w "$1"; }
# route.py runs in KiCad's Windows Python: WSL passes it only the variables WSLENV names.
export WSLENV="${WSLENV:+$WSLENV:}ROUTE_CONVERTER_FIRST"
mkdir -p "$WORK"
boards=("$@")
[[ ${#boards[@]} -gt 0 ]] || boards=(pod_board power_board)
fail=0
for b in "${boards[@]}"; do
    ok=0
    for ((i = 1; i <= ATTEMPTS; i++)); do
        echo "$b: attempt $i of $ATTEMPTS"
        "$HW/gen/layout.sh" --force "$b" >/dev/null || true
        "$KICAD_DIR/bin/python.exe" "$(win "$HW/gen/route.py")" "$(win "$HW/$b/$b.kicad_pcb")" \
            "$(win "$WORK")" "$(win "$JAVA")" "$(win "$JAR")" "$b" 2>&1 | grep -v "image handler" || true
        rpt="$HW/$b/drc.json"
        "$KICAD_DIR/bin/kicad-cli.exe" pcb drc --schematic-parity --severity-all --format json \
            -o "$(win "$rpt")" "$(win "$HW/$b/$b.kicad_pcb")" >/dev/null || true
        if python3 - "$rpt" "$b" <<'PY'; then ok=1; break; fi
import json, sys, collections
d = json.load(open(sys.argv[1]))
vs = d.get("violations", [])
errors = [v for v in vs if v.get("severity") == "error"]
warnings = collections.Counter(v["type"] for v in vs if v.get("severity") != "error")
left = d.get("unconnected_items", [])
parity = d.get("schematic_parity", [])
print(f"{sys.argv[2]}: {len(left)} unconnected; {len(errors)} DRC errors "
      f"{dict(collections.Counter(v['type'] for v in errors)) or ''}; warnings {dict(warnings) or 'none'}; "
      f"parity issues {len(parity)}")
for v in errors[:25]:
    print("   ", v["type"], "|", ", ".join(i.get("description", "") for i in v.get("items", []))[:180])
for u in left[:25]:
    print("    unconnected:", " | ".join(i["description"][:70] for i in u["items"]))
sys.exit(1 if errors or left or parity else 0)
PY
    done
    "$KICAD_DIR/bin/kicad-cli.exe" pcb render --side top --width 1600 --height 1100 --quality basic \
        -o "$(win "$HW/$b")\\routed.png" "$(win "$HW/$b/$b.kicad_pcb")" >/dev/null 2>&1 || true
    [[ $ok == 1 ]] || { echo "$b: FAILED after $ATTEMPTS attempts"; fail=1; }
done
exit $fail
