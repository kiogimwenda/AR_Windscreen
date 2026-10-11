#!/usr/bin/env bash
# routecheck.sh — can this placement be routed? Autoroutes a THROWAWAY copy of each board with
# Freerouting (routecheck.py) and runs KiCad's DRC on the copy. Passes only with every connection
# routed and no DRC error. The designs themselves are never touched. Run from anywhere:
#     hardware/gen/routecheck.sh [pod_board] [power_board]
# Needs Freerouting (FREEROUTING_JAR, default ~/.local/share/freerouting/freerouting-2.5.0.jar)
# and a Java it supports (JAVA, default the Windows JDK 25), and KiCad 10 for Windows.
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
JAVA="${JAVA:-/mnt/c/Program Files/Java/jdk-25.0.2/bin/java.exe}"
JAR="${FREEROUTING_JAR:-$HOME/.local/share/freerouting/freerouting-2.5.0.jar}"
WORK="${ROUTECHECK_DIR:-$HW/../build/routecheck}"
win() { wslpath -w "$1"; }
mkdir -p "$WORK"
boards=("$@")
[[ ${#boards[@]} -gt 0 ]] || boards=(pod_board power_board)
fail=0
for b in "${boards[@]}"; do
    "$KICAD_DIR/bin/python.exe" "$(win "$HW/gen/routecheck.py")" "$(win "$HW/$b/$b.kicad_pcb")" \
        "$(win "$WORK")" "$(win "$JAVA")" "$(win "$JAR")" 2>&1 | grep -v "image handler" || true
    cp "$HW/$b/$b.kicad_pro" "$WORK/$b.kicad_pro"  # the copy is checked with the board's own rules
    "$KICAD_DIR/bin/kicad-cli.exe" pcb drc --severity-error --format json -o "$(win "$WORK/$b.drc.json")" \
        "$(win "$WORK/$b.kicad_pcb")" >/dev/null || true
    python3 - "$WORK/$b.drc.json" "$b" <<'PY' || fail=1
import json, sys, collections
d = json.load(open(sys.argv[1]))
# Judged as a test of the PLACEMENT, not of Freerouting:
#  - fatal: a signal connection left unrouted, or any DRC error but the next one;
#  - not fatal: copper too near the board edge (Freerouting ignores KiCad's 0.5 mm edge rule; a
#    person routing keeps clear by eye), and a pad on a PLANE net (GND, +3V3) left without its
#    via down to the plane: Freerouting does not fan out to planes; it is one via per pad by hand.
PLANES = {"GND", "+3V3"}
def net(item):
    t = item["description"]
    return t[t.find("[") + 1:t.find("]")].split("/")[-1] if "[" in t else ""
#  - not fatal either: a "starved thermal" (a plane pad whose patch of plane the autorouter's own
#    tracks cut off: it routes freely on the ground layer, which a person keeps for ground).
SOFT = {"copper_edge_clearance", "starved_thermal"}
errors = collections.Counter(v["type"] for v in d.get("violations", []) if v["type"] not in SOFT)
starved = sum(1 for v in d.get("violations", []) if v["type"] == "starved_thermal")
edge = sum(1 for v in d.get("violations", []) if v["type"] == "copper_edge_clearance")
left = d.get("unconnected_items", [])
signal = [u for u in left if not all(net(i) in PLANES for i in u["items"])]
print(f"{sys.argv[2]}: autorouted; signal connections left {len(signal)}; plane vias to add "
      f"{len(left) - len(signal)}; DRC errors {dict(errors) or 0}; edge-hugging {edge}; "
      f"plane islands {starved}")
for u in signal:
    print("    unrouted:", " | ".join(i["description"][:70] for i in u["items"]))
sys.exit(1 if signal or errors else 0)
PY
done
exit $fail
