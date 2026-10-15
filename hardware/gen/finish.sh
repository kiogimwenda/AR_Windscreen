#!/usr/bin/env bash
# finish.sh — after route.sh: the board identification (finish.py), KiCad's DRC with the
# schematic-parity check again, then the fabrication outputs into hardware/<board>/fab/:
#   gerbers/  every copper, mask, paste, silkscreen layer and the outline (Protel extensions)
#   drill     Excellon, plated and non-plated holes in separate files, with a drill map
#   <board>-pos.csv  component positions (both sides, mm) for machine assembly
#   <board>-bom.csv  grouped bill of materials from the schematic
#   <board>-jlc-bom.csv, <board>-jlc-cpl.csv  the fab-assembly BOM and placement (sourcing.py)
#   <board>-gerbers.zip  what the fab asks for: gerbers + drill files
#   review/*.png      each layer rendered, to look at before paying
#     hardware/gen/finish.sh [pod_board] [power_board]
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
KICAD_DIR="${KICAD_DIR:-/mnt/c/Program Files/KiCad/10.0}"
CLI="$KICAD_DIR/bin/kicad-cli.exe"
win() { wslpath -w "$1"; }
boards=("$@")
[[ ${#boards[@]} -gt 0 ]] || boards=(pod_board power_board)
fail=0
for b in "${boards[@]}"; do
    pcb="$HW/$b/$b.kicad_pcb"
    fab="$HW/$b/fab"
    rm -rf "$fab"; mkdir -p "$fab/gerbers" "$fab/review"
    "$KICAD_DIR/bin/python.exe" "$(win "$HW/gen/finish.py")" "$(win "$pcb")" "$b" 2>&1 | grep -v "image handler"
    rpt="$HW/$b/drc.json"
    "$CLI" pcb drc --schematic-parity --severity-all --format json -o "$(win "$rpt")" "$(win "$pcb")" >/dev/null || true
    if grep -q '"type": "via_dangling"' "$rpt"; then  # router leftovers: remove, check again
        "$KICAD_DIR/bin/python.exe" "$(win "$HW/gen/finish.py")" "$(win "$pcb")" "$b" "$(win "$rpt")" 2>&1 \
            | grep -v "image handler" | grep "dangling"
        "$CLI" pcb drc --schematic-parity --severity-all --format json -o "$(win "$rpt")" "$(win "$pcb")" >/dev/null || true
    fi
    if python3 - "$rpt" "$b" <<'PY'
import json, sys, collections
d = json.load(open(sys.argv[1]))
errors = [v for v in d["violations"] if v["severity"] == "error"]
warn = collections.Counter(v["type"] for v in d["violations"] if v["severity"] != "error")
print(f"{sys.argv[2]}: {len(d['unconnected_items'])} unconnected; {len(errors)} DRC errors; "
      f"parity issues {len(d['schematic_parity'])}; warnings {dict(warn) or 'none'}")
sys.exit(1 if errors or d["unconnected_items"] or d["schematic_parity"] else 0)
PY
    then :; else
        echo "$b: NOT PASSING: no fabrication files written"; fail=1; rm -rf "$fab"; continue
    fi
    if [[ $b == pod_board ]]; then
        layers="F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    else
        layers="F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    fi
    "$CLI" pcb export gerbers --layers "$layers" --subtract-soldermask --exclude-value --use-drill-file-origin \
        -o "$(win "$fab/gerbers")\\" "$(win "$pcb")" >/dev/null
    "$CLI" pcb export drill --format excellon --drill-origin plot --excellon-separate-th --generate-map --map-format pdf \
        -o "$(win "$fab/gerbers")\\" "$(win "$pcb")" >/dev/null
    "$CLI" pcb export pos --format csv --units mm --side both --use-drill-file-origin -o "$(win "$fab/$b-pos.csv")" "$(win "$pcb")" >/dev/null
    "$CLI" sch export bom --fields 'Reference,Value,Footprint,${QUANTITY}' --labels 'Refs,Value,Footprint,Qty' \
        --group-by 'Value,Footprint' -o "$(win "$fab/$b-bom.csv")" "$(win "$HW/$b/$b.kicad_sch")" >/dev/null
    # the fab-assembly files: every part matched to its exact part number (sourcing.py)
    python3 "$HW/gen/sourcing.py" "$b" "$fab" || fail=1
    (cd "$fab/gerbers" && rm -f ../"$b-gerbers.zip" && zip -q -j ../"$b-gerbers.zip" *.g* *.drl 2>/dev/null)
    # one picture per layer (from the same board data the gerbers are plotted from)
    IFS=, read -ra L <<<"$layers"
    for l in "${L[@]}"; do
        [[ $l == Edge.Cuts ]] && continue
        n=${l//./_}
        "$CLI" pcb export pdf --layers "$l,Edge.Cuts" --mode-single --black-and-white \
            -o "$(win "$fab/review/$n.pdf")" "$(win "$pcb")" >/dev/null 2>&1 || true
        if [[ -f "$fab/review/$n.pdf" ]]; then
            pdftoppm -png -r 300 -singlefile "$fab/review/$n.pdf" "$fab/review/$n" && rm -f "$fab/review/$n.pdf"
            python3 - "$fab/review/$n.png" <<'PY'
import sys
from PIL import Image, ImageOps
im = Image.open(sys.argv[1]).convert("L")
box = ImageOps.invert(im).getbbox()  # the board: everything not white
if box:
    m = 30
    im.crop((max(0, box[0] - m), max(0, box[1] - m), box[2] + m, box[3] + m)).save(sys.argv[1])
PY
        fi
    done
    echo "$b: $(ls "$fab/gerbers" | wc -l) fabrication files; $fab/$b-gerbers.zip; review images in $fab/review"
done
exit $fail
