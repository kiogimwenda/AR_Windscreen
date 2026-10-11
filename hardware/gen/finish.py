"""Finishing touches on a ROUTED board (KiCad's own Python), without re-routing it. Run through
finish.sh.

    python.exe finish.py <board.kicad_pcb> <pod_board|power_board> [drc.json]

Adds the board's identification on the BOTTOM silkscreen (no parts there, so it never crowds a
label): project, board, revision, date, designer. Re-running replaces it. Then refills the zones.
"""

import sys

import pcbnew

pcb_path, name = sys.argv[1:3]
board = pcbnew.LoadBoard(pcb_path)
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import outlines  # noqa: E402
ORIGIN = (50.0, 50.0)  # outlines.py draws the board here
TITLE = {"pod_board": "AR WINDSCREEN - POD BOARD", "power_board": "AR WINDSCREEN - POWER BOARD"}[name]
LINES = [TITLE, "REV 1  2026-10", "I. K. MWENDA  (FINAL YEAR PROJECT)"]
# centre of the text block, mm from the board's top-left corner: a clear area of the bottom side
AT = {"pod_board": (35.0, 22.0), "power_board": (38.0, 30.0)}[name]

# The drill/place origin at the board's bottom-left corner: the fabrication files (gerbers, drill,
# component positions) are given from there, as fabs and assembly services expect.
H = outlines.BOARDS[name]["h"]
board.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I_MM(ORIGIN[0], ORIGIN[1] + H))

# Only this script writes text on the bottom silkscreen: clear it all, so a re-run replaces it.
# (Delete, not Remove: removing a drawing through Python left the board's lists unusable.)
for d in [d for d in board.GetDrawings() if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.B_SilkS]:
    board.Delete(d)

y = AT[1] - (len(LINES) - 1) * 1.0
for i, line in enumerate(LINES):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(line)
    t.SetLayer(pcbnew.B_SilkS)
    size = 1.2 if i == 0 else 1.0
    t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
    t.SetTextThickness(pcbnew.FromMM(0.18 if i == 0 else 0.15))
    t.SetMirrored(True)  # read from the bottom
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_CENTER)
    t.SetPosition(pcbnew.VECTOR2I_MM(ORIGIN[0] + AT[0], ORIGIN[1] + y))
    board.Add(t)
    y += 2.0

# Track stubs that lead nowhere (an end touching no pad, via or other track of the net): removed.
def loose_ends():
    ends = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK":
            for p in (t.GetStart(), t.GetEnd()):
                k = (p.x, p.y, t.GetLayer())
                ends[k] = ends.get(k, 0) + 1
    out = []
    for t in board.GetTracks():
        if t.GetClass() != "PCB_TRACK" or t.IsLocked():
            continue
        for p in (t.GetStart(), t.GetEnd()):
            if ends[(p.x, p.y, t.GetLayer())] > 1:
                continue
            on_pad = any(pad.GetNetCode() == t.GetNetCode() and pad.IsOnLayer(t.GetLayer()) and pad.HitTest(p)
                         for fp in board.GetFootprints() for pad in fp.Pads())
            on_via = any(v.GetClass() == "PCB_VIA" and v.GetNetCode() == t.GetNetCode() and v.HitTest(p)
                         for v in board.GetTracks())
            on_track = any(o is not t and o.GetClass() == "PCB_TRACK" and o.GetNetCode() == t.GetNetCode()
                           and o.GetLayer() == t.GetLayer() and o.HitTest(p) for o in board.GetTracks())
            if not (on_pad or on_via or on_track):
                out.append(t)
                break
    return out


# A via KiCad's own DRC calls dangling (joined on one side only: a router leftover) is removed too: finish.sh passes its report back here (argument 3).
removed_from_drc = 0
if len(sys.argv) > 3:
    import json
    report = json.load(open(sys.argv[3]))
    spots = []
    for v in report.get("violations", []):
        # vias only: a "dangling" track sliver can still be the piece that joins two others (removing
        # a 0.01 mm one left a connection open); it stays, a warning
        if v["type"] == "via_dangling":
            for it in v["items"]:
                spots.append((it["description"].split()[0], pcbnew.VECTOR2I_MM(it["pos"]["x"], it["pos"]["y"])))
    for kind, pos in spots:
        for item in list(board.GetTracks()):
            if item.IsLocked():
                continue
            if kind == "Via" and item.GetClass() == "PCB_VIA" and item.HitTest(pos):
                board.Delete(item)
                removed_from_drc += 1
                break
            if kind == "Track" and item.GetClass() == "PCB_TRACK" and item.HitTest(pos):
                board.Delete(item)
                removed_from_drc += 1
                break

pruned = 0
while True:
    loose = loose_ends()
    if not loose:
        break
    for t in loose:
        board.Delete(t)
        pruned += 1
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(pcb_path, board)
print(f"{name}: dangling items removed: {removed_from_drc} (from the DRC) + {pruned} stubs")
print(f"{name}: identification on the bottom silkscreen: {' / '.join(LINES)}")
