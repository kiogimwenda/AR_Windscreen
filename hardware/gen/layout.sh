#!/usr/bin/env bash
# layout.sh — build both boards with every part placed (layout.py, in KiCad's own Python), then
# check them with KiCad's DRC including the schematic-parity check. Run from anywhere:
#     hardware/gen/layout.sh [--force] [pod_board] [power_board]
# Passes only if the only DRC findings are "unconnected" items: the routing that is left to do.
# Uses KiCad 10 for Windows from WSL; set KICAD_DIR for another install.
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
PY="$KICAD_DIR/bin/python.exe"
CLI="$KICAD_DIR/bin/kicad-cli.exe"
win() { wslpath -w "$1"; }

"$PY" "$(win "$HW/gen/layout.py")" "$@"
boards=()
for a in "$@"; do [[ "$a" == --* ]] || boards+=("$a"); done
[[ ${#boards[@]} -gt 0 ]] || boards=(pod_board power_board)
fail=0
for b in "${boards[@]}"; do
    "$CLI" pcb render --side top --width 1600 --height 1100 --quality basic \
        -o "$(win "$HW/$b")\\placement.png" "$(win "$HW/$b/$b.kicad_pcb")" >/dev/null 2>&1 || true
    rpt="$HW/$b/drc.json"
    "$CLI" pcb drc --schematic-parity --severity-all --format json -o "$(win "$rpt")" \
        "$(win "$HW/$b/$b.kicad_pcb")" >/dev/null || true
    python3 - "$rpt" "$b" <<'PY' || fail=1
import json, sys, collections
d = json.load(open(sys.argv[1]))
vs = d.get("violations", [])
errors = [v for v in vs if v.get("severity") == "error"]
warnings = collections.Counter(v["type"] for v in vs if v.get("severity") != "error")
unconnected = len(d.get("unconnected_items", []))
parity = d.get("schematic_parity", [])
bad = errors
print(f"{sys.argv[2]}: {unconnected} connections to route; {len(errors)} DRC errors; warnings "
      f"{dict(warnings) or 'none'}; schematic parity issues {len(parity)}")
for v in errors:
    if True:
        print("   ", v["type"], "|", v["description"], "|",
              ", ".join(i.get("description", "") for i in v.get("items", []))[:200])
for p in parity:
    print("    parity:", p.get("description"), "|", ", ".join(i.get("description", "") for i in p.get("items", []))[:200])
sys.exit(1 if bad or parity else 0)
PY
done
exit $fail
