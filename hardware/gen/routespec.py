"""Checks a routed board against the points of LAYOUT_GUIDE.md's specification that KiCad's DRC
does not know about (KiCad's own Python). Run through routespec.sh after route.sh.

    python.exe routespec.py <board.kicad_pcb> <pod_board|power_board>

Prints one line per check, PASS or FAIL with what broke it; exits 1 on any FAIL.
"""

import sys
from pathlib import Path

import pcbnew

pcb_path, name = sys.argv[1:3]
board = pcbnew.LoadBoard(pcb_path)
POD = name == "pod_board"
TM = pcbnew.ToMM
sys.path.insert(0, str(Path(__file__).resolve().parent))
import pcb_rules  # noqa: E402

results = []


def check(label, bad):
    results.append((label, bad))
    print(("PASS " if not bad else "FAIL ") + label + ("" if not bad else f": {bad[:6]}"))


def short(n):
    return n.split("/")[-1]


tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]


def in_courtyard(ref, layers, allowed):
    fp = board.FindFootprintByReference(ref)
    poly = fp.GetCourtyard(pcbnew.F_CrtYd)
    bad = []
    for t in tracks:
        if t.GetLayer() in layers and short(t.GetNetname()) not in allowed:
            if poly.Collide(t.GetEffectiveShape(t.GetLayer())):
                bad.append(f"{short(t.GetNetname())} on {t.GetLayerName()}")
    return bad


if POD:
    check("no track on In1 (the solid GND plane)", [short(t.GetNetname()) for t in tracks if t.GetLayer() == pcbnew.In1_Cu])
    check("no track on In2 (the +3V3 plane)", [short(t.GetNetname()) for t in tracks if t.GetLayer() == pcbnew.In2_Cu])
    check("nothing but the crystal's own nets in Y1's area on B.Cu",
          in_courtyard("Y1", (pcbnew.B_Cu,), {"OSC_IN", "OSC_OUT", "GND"}))
    # on F.Cu: under the body, i.e. between its two pads (its own tracks reach the pads from outside)
    y1 = board.FindFootprintByReference("Y1")
    a_, b_ = sorted((p.GetBoundingBox() for p in y1.Pads()), key=lambda bb: (bb.GetLeft(), bb.GetTop()))
    cy = y1.GetCourtyard(pcbnew.F_CrtYd).BBox()
    if a_.GetBottom() <= b_.GetTop():
        body = pcbnew.BOX2I(pcbnew.VECTOR2I(cy.GetLeft(), a_.GetBottom()),
                            pcbnew.VECTOR2I(cy.GetWidth(), b_.GetTop() - a_.GetBottom()))
    else:
        body = pcbnew.BOX2I(pcbnew.VECTOR2I(a_.GetRight(), cy.GetTop()),
                            pcbnew.VECTOR2I(b_.GetLeft() - a_.GetRight(), cy.GetHeight()))
    poly = pcbnew.SHAPE_POLY_SET()
    poly.NewOutline()
    for x, y in ((body.GetLeft(), body.GetTop()), (body.GetRight(), body.GetTop()),
                 (body.GetRight(), body.GetBottom()), (body.GetLeft(), body.GetBottom())):
        poly.Append(x, y)
    check("nothing under the crystal's body on F.Cu (between its pads)",
          [short(t.GetNetname()) for t in tracks if t.GetLayer() == pcbnew.F_Cu
           and poly.Collide(t.GetEffectiveShape(pcbnew.F_Cu))])
    usb = [t for t in tracks if short(t.GetNetname()) in ("USB_DP", "USB_DM")]
    usb_vias = [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and short(v.GetNetname()) in ("USB_DP", "USB_DM")]
    # the guide asks for no vias "if possible": full-speed USB (12 Mbit/s) does not need it; noted
    print(f"     note: USB D+/D- length {sum(TM(t.GetLength()) for t in usb):.1f} mm, {len(usb_vias)} vias")
else:
    rail = [z for z in board.Zones() if not z.GetIsRuleArea() and short(z.GetNetname()) == "ACT_12V"]
    check("10 A actuator rail is a solid-connected F.Cu zone",
          [] if rail and rail[0].GetPadConnection() == pcbnew.ZONE_CONNECTION_FULL else ["missing"])
    fps = [board.FindFootprintByReference(r) for r in ("U1", "C21", "C22", "D13", "L1")]
    box = fps[0].GetCourtyard(pcbnew.F_CrtYd).BBox()
    for fp in fps[1:]:
        box.Merge(fp.GetCourtyard(pcbnew.F_CrtYd).BBox())
    loop = pcbnew.SHAPE_POLY_SET()
    loop.NewOutline()
    for x, y in ((box.GetLeft(), box.GetTop()), (box.GetRight(), box.GetTop()),
                 (box.GetRight(), box.GetBottom()), (box.GetLeft(), box.GetBottom())):
        loop.Append(x, y)
    check("no B.Cu track under the converter's switching loop",
          [short(t.GetNetname()) for t in tracks if t.GetLayer() == pcbnew.B_Cu
           and loop.Collide(t.GetEffectiveShape(pcbnew.B_Cu))])
    gnd = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetLayer() == pcbnew.B_Cu and short(z.GetNetname()) == "GND"]
    check("bottom layer is a GND zone", [] if gnd else ["missing"])

# No via in (or touching) a surface-mount pad: an open via draws the solder away from the joint.
in_pad = []
smd = [(fp.GetReference(), p, p.GetEffectiveShape(pcbnew.F_Cu), p.GetBoundingBox())
       for fp in board.GetFootprints() for p in fp.Pads()
       if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and p.IsOnLayer(pcbnew.F_Cu)]
for v in board.GetTracks():
    if v.GetClass() != "PCB_VIA":
        continue
    vb, vs = v.GetBoundingBox(), v.GetEffectiveShape(pcbnew.F_Cu)
    in_pad += [f"{ref}.{p.GetNumber()}" for ref, p, ps, pb in smd if vb.Intersects(pb) and ps.Collide(vs, 0)]
check("no via in or touching a surface-mount pad", in_pad)

# The terminal and fuse-holder pins: every other net's copper at least TERMINAL_CLEARANCE away.
near_terminal = []
for fp in board.GetFootprints():
    if not any(k in fp.GetFPIDAsString() for k in pcb_rules.TERMINAL_FOOTPRINTS):
        continue
    for p in fp.Pads():
        ps, zone = p.GetEffectiveShape(pcbnew.F_Cu), p.GetBoundingBox()
        zone.Inflate(pcbnew.FromMM(1.0))
        for t in board.GetTracks():
            if t.GetNetCode() == p.GetNetCode() or not zone.Intersects(t.GetBoundingBox()):
                continue
            for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
                if t.IsOnLayer(layer) and p.GetEffectiveShape(layer).Collide(
                        t.GetEffectiveShape(layer), pcbnew.FromMM(pcb_rules.TERMINAL_CLEARANCE) - 1):
                    near_terminal.append(f"{fp.GetReference()}.{p.GetNumber()}~{short(t.GetNetname())}")
                    break
if any(any(k in fp.GetFPIDAsString() for k in pcb_rules.TERMINAL_FOOTPRINTS) for fp in board.GetFootprints()):
    check(f"other nets at least {pcb_rules.TERMINAL_CLEARANCE} mm from terminal and fuse-holder pins",
          sorted(set(near_terminal)))

# Track widths. The requirement: every track at least as wide as IPC-2221 asks for its class's
# current (the supply and rail classes) and never under the fab minimum. The class width is the
# design's nominal width, a little above that; tracks below it are counted (neck-downs at pins,
# the router's squeezes, the deliberate 0.25-0.5 mm ties and stubs placed before routing).
import fnmatch  # noqa: E402

spec = pcb_rules.NET_CLASSES[name]
fab_min = pcb_rules.FAB_RULES[name]["min_track_width"]


def klass(netname):
    for cname, width, _c, _v, amps, pats in spec["classes"]:
        if any(fnmatch.fnmatch(short(netname), p) for p in pats):
            return cname, width, max(fab_min, pcb_rules.ipc2221_width_mm(amps, spec["copper_oz"]))
    return "Default", spec["default"]["track"], fab_min


under, below_nominal = [], 0
for t in tracks:
    cname, nominal, need = klass(t.GetNetname())
    w = TM(t.GetWidth())
    if w + 1e-6 < need:
        under.append((short(t.GetNetname()), round(w, 3), round(need, 3)))
    elif w + 1e-6 < nominal:
        below_nominal += 1
check("every track at least the IPC-2221 width for its class's current, and the fab minimum", under)
print(f"     tracks narrower than their class's nominal width (but above the requirement): {below_nominal}")
sys.exit(1 if any(bad for _, bad in results) else 0)
