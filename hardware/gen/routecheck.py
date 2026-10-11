"""Routability check for a placement (KiCad's own Python): export the board to Specctra DSN,
autoroute it with Freerouting, import the result into a THROWAWAY copy, and save it, for
routecheck.sh to run KiCad's DRC on. The autorouted copy is never the design: it only proves the
placement leaves room for every connection. The real routing is done by hand (LAYOUT_GUIDE.md).

    python.exe routecheck.py <board.kicad_pcb> <workdir> <java.exe> <freerouting.jar> [passes]
"""

import subprocess
import sys
from pathlib import Path

import pcbnew

pcb_path, work, java, jar = sys.argv[1:5]
passes = sys.argv[5] if len(sys.argv) > 5 else "40"
work = Path(work)
work.mkdir(parents=True, exist_ok=True)
name = Path(pcb_path).stem
board = pcbnew.LoadBoard(pcb_path)
dsn, ses = work / f"{name}.dsn", work / f"{name}.ses"
if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
    sys.exit("DSN export failed")
r = subprocess.run([java, "-jar", jar, "-de", str(dsn), "-do", str(ses), "-mp", passes,
                    "--gui.enabled=false"], capture_output=True, text=True, timeout=3600)
if not ses.exists():
    sys.exit("Freerouting produced no session:\n" + r.stdout[-2000:] + r.stderr[-2000:])
if not pcbnew.ImportSpecctraSES(board, str(ses)):
    sys.exit("SES import failed")
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
out = work / f"{name}.kicad_pcb"
pcbnew.SaveBoard(str(out), board)
print(f"autorouted copy: {out} ({len(board.GetTracks())} track segments and vias)")
