"""Routes a placed board (layout.py's output) to LAYOUT_GUIDE.md's specification, with KiCad's own
Python and Freerouting. Run through route.sh.

    python.exe route.py <board.kicad_pcb> <workdir> <java.exe> <freerouting.jar> <pod_board|power_board>

In order:
  1. Rule areas that encode the guide's constraints, so the router cannot break them:
     - a 0.6 mm no-track/no-via border inside the outline (KiCad's 0.5 mm copper-to-edge rule,
       which Freerouting does not know);
     - pod: no tracks on In1 (the GND plane) and nothing on B.Cu under the crystal; power board:
       no other copper in the actuator rail's zone, and no tracks on B.Cu under the 5 V
       converter's switching loop (the ground under it stays whole);
  2. before routing, the plane connections (locked, so the router keeps and avoids them): every
     SMD pad on GND (and on the pod, +3V3) gets its OWN via beside it with a short stub (the pod
     MCU's: straight out from the pin's tip), and the pod MCU's +3V3 pins a short track to their
     decoupling capacitor;
  3. Freerouting routes the signals at each net class's width and clearance; it is told to
     ignore GND (net class "Ground"); on the pod, In1 and In2 are declared power layers, so the
     +3V3 pins it still has to join go to the In2 plane by vias;
  4. mending what Freerouting gets wrong: the pre-placed ground copper it returns altered is
     replaced; a neck-down below the minimum width is widened or re-laid (GND ones dropped);
  5. ground pours on the outer layers (pod: F.Cu and B.Cu; power: F.Cu, the B.Cu one already
     exists), a grid of GND stitching vias wherever one fits, a via for each piece of pour that
     has none, a short track joining each group of plane copper still cut off to the rest, and
     a last fill that removes the scraps that touch nothing.
The board is saved in place; route.sh then runs KiCad's DRC with the schematic-parity check.
"""

import math
import os
import subprocess
import sys
from pathlib import Path

import pcbnew

pcb_path, work, java, jar, board_name = sys.argv[1:6]
work = Path(work)
work.mkdir(parents=True, exist_ok=True)
board = pcbnew.LoadBoard(pcb_path)
POD = board_name == "pod_board"
FM, TM = pcbnew.FromMM, pcbnew.ToMM
ORIGIN = (50.0, 50.0)  # outlines.py draws the board here
sys.path.insert(0, str(Path(__file__).resolve().parent))
import outlines  # noqa: E402
import pcb_rules  # noqa: E402

W, H = outlines.BOARDS[board_name]["w"], outlines.BOARDS[board_name]["h"]


def P(x, y):
    return pcbnew.VECTOR2I(FM(ORIGIN[0] + x), FM(ORIGIN[1] + y))


def net(name):
    for n, ni in board.GetNetsByName().items():
        if str(n).split("/")[-1] == name:
            return ni
    raise KeyError(name)


def rule_area(points, layers, tracks=True, vias=True, name=""):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetZoneName(name)
    ls = pcbnew.LSET()
    for layer in layers:
        ls.AddLayer(layer)
    z.SetLayerSet(ls)
    z.SetDoNotAllowTracks(tracks)
    z.SetDoNotAllowVias(vias)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowZoneFills(False)
    o = z.Outline()
    o.NewOutline()
    for x, y in points:
        o.Append(P(x, y))
    board.Add(z)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


copper = [pcbnew.F_Cu, pcbnew.B_Cu] + ([pcbnew.In1_Cu, pcbnew.In2_Cu] if POD else [])

# --- 1. Rule areas -------------------------------------------------------------------------------
E = 0.6
for r in (rect(0, 0, W, E), rect(0, H - E, W, H), rect(0, 0, E, H), rect(W - E, 0, W, H)):
    rule_area(r, [pcbnew.F_Cu, pcbnew.B_Cu], name="edge clearance")


def courtyard(ref, grow=0.0):
    fp = board.FindFootprintByReference(ref)
    bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
    return (TM(bb.GetLeft()) - ORIGIN[0] - grow, TM(bb.GetTop()) - ORIGIN[1] - grow,
            TM(bb.GetRight()) - ORIGIN[0] + grow, TM(bb.GetBottom()) - ORIGIN[1] + grow)


if POD:
    rule_area(rect(*courtyard("Y1")), [pcbnew.B_Cu], vias=True, name="nothing under the crystal")
    # and on the top layer, between its two pads (under its body): its own tracks reach the pads
    # from outside
    y1 = board.FindFootprintByReference("Y1")
    pb = [p.GetBoundingBox() for p in y1.Pads()]
    cx0, cy0, cx1, cy1 = courtyard("Y1")
    edges = lambda b: (TM(b.GetLeft()) - ORIGIN[0], TM(b.GetTop()) - ORIGIN[1], TM(b.GetRight()) - ORIGIN[0], TM(b.GetBottom()) - ORIGIN[1])
    (ax0, ay0, ax1, ay1), (bx0, by0, bx1, by1) = sorted((edges(b) for b in pb), key=lambda e: (e[0], e[1]))
    if ay1 <= by0:  # pads one above the other
        between = (cx0, ay1 + 0.2, cx1, by0 - 0.2)
    else:  # side by side
        between = (ax1 + 0.2, cy0, bx0 - 0.2, cy1)
    rule_area(rect(*between), [pcbnew.F_Cu], vias=True, name="nothing under the crystal's body")
    # In1 is the GND plane under the top layer's signals: no tracks at all, vias only (without this
    # Freerouting routes through it). In2 is the +3V3 plane: declared a power layer for the router
    # (below), so it carries no signals either; the signals use F.Cu and B.Cu.
    rule_area(rect(0, 0, W, H), [pcbnew.In1_Cu], vias=False, name="GND plane: no tracks")
else:
    # The actuator-rail zone: Freerouting does not see zone copper, so no other track may enter it.
    act = next(z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname().endswith("ACT_12V"))
    pts = [(TM(act.Outline().CVertex(i).x) - ORIGIN[0], TM(act.Outline().CVertex(i).y) - ORIGIN[1])
           for i in range(act.Outline().TotalVertices())]
    rule_area(pts, [pcbnew.F_Cu], name="actuator rail: no other copper")
    xs, ys = [], []
    for ref in ("U1", "C21", "C22", "D13", "L1"):  # the switching loop only
        x0, y0, x1, y1 = courtyard(ref)
        xs += [x0, x1]
        ys += [y0, y1]
    # tracks only: GND vias stay allowed, so the loop's ground pads reach the plane underneath
    rule_area(rect(min(xs), min(ys), max(xs), max(ys)), [pcbnew.B_Cu], vias=False,
              name="solid ground under the converter")

# --- 2. Plane fan-out ----------------------------------------------------------------------------
ds = board.GetDesignSettings()
VIA_D, VIA_DRILL = (0.6, 0.3) if POD else (0.8, 0.4)
CLEAR = 0.2 if POD else 0.25
PLANE_NETS = {"GND", "+3V3"} if POD else {"GND"}


def obstacles():
    items = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            items.append(pad)
    for t in board.GetTracks():
        items.append(t)
    return items


def is_terminal_pad(it):
    return it.GetClass() == "PAD" and any(k in it.GetParentFootprint().GetFPIDAsString()
                                          for k in pcb_rules.TERMINAL_FOOTPRINTS)


def clearance_to(it):
    """The room other nets' copper keeps from this item: more round terminal and fuse pins."""
    return max(CLEAR, pcb_rules.TERMINAL_CLEARANCE) if is_terminal_pad(it) else CLEAR


def free_spot(pt, netcode, items):
    """True if a via at pt clears every other-net pad/track/via (with the terminals' wider
    clearance), touches no surface-mount pad at all (not even its own net's: an open via in a pad
    draws the solder away from the joint), and keeps off the board edge."""
    x, y = TM(pt.x) - ORIGIN[0], TM(pt.y) - ORIGIN[1]
    r = VIA_D / 2
    if x - r < E + 0.1 or y - r < E + 0.1 or x + r > W - E - 0.1 or y + r > H - E - 0.1:
        return False
    shape = pcbnew.SHAPE_CIRCLE(pt, FM(r))
    for it in items:
        if it.GetNetCode() == netcode:
            # same net may touch, but two drilled holes may not meet (KiCad's hole-to-hole rule),
            # and no via goes in a surface-mount pad
            drilled = it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole())
            if drilled and it.GetEffectiveShape(pcbnew.F_Cu).Collide(shape, FM(0.25)):
                return False
            if it.GetClass() == "PAD" and it.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and \
                    it.GetEffectiveShape(pcbnew.F_Cu).Collide(shape, FM(0.1)):
                return False
            continue
        for layer in copper:
            if not it.IsOnLayer(layer):
                continue
            if it.GetEffectiveShape(layer).Collide(shape, FM(clearance_to(it))):
                return False
    return True


fanout_made = []  # (pad centre, via position, stub width, net): to put back after the SES import


def fan_out(nets):
    """A via of its own beside every SMD pad on `nets`, with a short stub to it. On the pod's MCU
    (0.5 mm pitch) the via goes straight out from the pin's tip, in line with it, a little beyond
    where its neighbours' tracks fan out, so it is in nobody's way."""
    global items
    fanned = 0
    missed = []
    items = obstacles()
    for fp in board.GetFootprints():
        centre = fp.GetPosition()
        mcu = POD and fp.GetReference() == "U1"
        for pad in fp.Pads():
            n = pad.GetNetname().split("/")[-1]
            if n not in nets or pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            if mcu and pad.GetNumber() in tied:
                continue  # joined to its decoupling capacitor, whose pad has the via
            pc = pad.GetPosition()
            size = max(TM(pad.GetSize(pcbnew.F_Cu).x), TM(pad.GetSize(pcbnew.F_Cu).y)) / 2
            if mcu:
                dx, dy = pc.x - centre.x, pc.y - centre.y
                base = (0.0 if dx > 0 else math.pi) if abs(dx) > abs(dy) else (math.pi / 2 if dy > 0 else -math.pi / 2)
                dists, angles, stub_w = (size + VIA_D / 2 + 0.6, size + VIA_D / 2 + 1.1, size + VIA_D / 2 + 1.6), (0,), 0.25
            else:
                base = math.atan2(pc.y - centre.y, pc.x - centre.x) if pc != centre else 0.0
                dists = (size + VIA_D / 2 + CLEAR, size + VIA_D / 2 + CLEAR + 0.3, size + VIA_D + 0.6)
                angles, stub_w = (0, 45, -45, 90, -90, 135, -135, 180), (0.3 if POD else 0.5)
            placed = False
            for dist in dists:
                for dang in angles:
                    a = base + math.radians(dang)
                    pt = pcbnew.VECTOR2I(int(pc.x + FM(dist) * math.cos(a)), int(pc.y + FM(dist) * math.sin(a)))
                    if not free_spot(pt, pad.GetNetCode(), items):
                        continue
                    stub = pcbnew.PCB_TRACK(board)
                    stub.SetStart(pc)
                    stub.SetEnd(pt)
                    stub.SetWidth(FM(stub_w))
                    stub.SetLayer(pcbnew.F_Cu)
                    stub.SetNet(pad.GetNet())
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(pt)
                    via.SetWidth(FM(VIA_D))
                    via.SetDrill(FM(VIA_DRILL))
                    via.SetNet(pad.GetNet())
                    # the stub must not cross another net's pad on its way
                    seg = pcbnew.SHAPE_SEGMENT(pc, pt, stub.GetWidth())
                    if any(it.GetNetCode() != pad.GetNetCode() and it.IsOnLayer(pcbnew.F_Cu)
                           and it.GetEffectiveShape(pcbnew.F_Cu).Collide(seg, FM(CLEAR)) for it in items):
                        continue
                    stub.SetLocked(True)  # exported as protected: the router keeps it and avoids it
                    via.SetLocked(True)
                    board.Add(stub)
                    board.Add(via)
                    fanout_made.append((pc, pt, stub.GetWidth(), pad.GetNet()))
                    items += [stub, via]
                    placed = placed or True
                    break
                if placed:
                    break
            if placed:
                fanned += 1
            else:
                missed.append(f"{fp.GetReference()}.{pad.GetNumber()}")
    print(f"fan-out: {fanned} plane vias; no room beside {missed or 'none'}")
    return missed


pcbnew.ZONE_FILLER(board).Fill(board.Zones())
def too_near(track):
    shape = track.GetEffectiveShape(track.GetLayer())
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetCode() != track.GetNetCode() and pad.IsOnLayer(track.GetLayer()) and \
                    pad.GetEffectiveShape(track.GetLayer()).Collide(shape, FM(clearance_to(pad))):
                return True
    for o in board.GetTracks():
        if o.GetNetCode() != track.GetNetCode() and o.IsOnLayer(track.GetLayer()) and \
                o.GetEffectiveShape(track.GetLayer()).Collide(shape, FM(CLEAR)):
            return True
    return False


def try_link(a, b, layer, the_net, widths=(0.5, 0.3)):
    """A track from a to b clear of every other net: straight, with one square bend, or a dogleg
    (first straight out from a along an axis, then across to b: how a fine-pitch pin is left)."""
    paths = [[a, b], [a, pcbnew.VECTOR2I(a.x, b.y), b], [a, pcbnew.VECTOR2I(b.x, a.y), b]]
    for length in (0.9, 1.4, 2.0):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            paths.append([a, pcbnew.VECTOR2I(a.x + dx * FM(length), a.y + dy * FM(length)), b])
    for w in widths:
        for path in paths:
            segs = []
            for p0, p1 in zip(path, path[1:]):
                if p0 == p1:
                    continue
                s = pcbnew.PCB_TRACK(board)
                s.SetStart(p0)
                s.SetEnd(p1)
                s.SetWidth(FM(w))
                s.SetLayer(layer)
                s.SetNet(the_net)
                segs.append(s)
            if any(too_near(s) or any(r.GetIsRuleArea() and r.GetDoNotAllowTracks() and r.IsOnLayer(layer)
                                      and r.Outline().Collide(s.GetEffectiveShape(layer)) for r in board.Zones())
                   for s in segs):
                continue
            for s in segs:
                board.Add(s)
            return True
    return False


gnd_net = net("GND")
tied = set()  # the MCU's +3V3 pins joined to a capacitor before routing
prerouted = []  # tracks placed before routing: put back after the SES import if it drops them


def tie_mcu(net_name, pins=None):
    """Pod MCU pins on a plane net (all of them, or just `pins`) each to the nearest pad of that net
    on another part (its decoupling capacitor: within 8 mm) by a short top-layer track, straight,
    with one bend or a dogleg, clear of every other net. The capacitor's pad has the via."""
    the_net = net(net_name)
    mcu = board.FindFootprintByReference("U1")
    caps = [p for fp in board.GetFootprints() if fp.GetReference() != "U1"
            for p in fp.Pads() if p.GetNetCode() == the_net.GetNetCode() and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
    joined = []
    for pad in mcu.Pads():
        if pad.GetNetCode() != the_net.GetNetCode() or (pins is not None and pad.GetNumber() not in pins):
            continue
        a = pad.GetPosition()
        near = sorted(caps, key=lambda c: (c.GetPosition().x - a.x) ** 2 + (c.GetPosition().y - a.y) ** 2)
        for c in near[:4]:
            b = c.GetPosition()
            if math.hypot(TM(b.x - a.x), TM(b.y - a.y)) > 8.0:
                break
            before = {t.m_Uuid.AsString() for t in board.GetTracks()}
            if try_link(a, b, pcbnew.F_Cu, the_net, widths=(0.3, 0.25)):
                tied.add(pad.GetNumber())
                joined.append(pad.GetNumber())
                for t in board.GetTracks():
                    if t.m_Uuid.AsString() not in before:
                        t.SetLocked(True)
                prerouted.extend((t.GetStart(), t.GetEnd(), t.GetWidth(), t.GetLayer(), t.GetNet())
                                 for t in board.GetTracks() if t.m_Uuid.AsString() not in before)
                break
    print(f"MCU {net_name} pins joined to their capacitors: {sorted(joined, key=int)}")


# The ground vias go in FIRST and the router is told to leave GND alone (its net class "Ground" is
# ignored): every ground pad reaches the ground plane by its own via, and the router never draws
# ground tracks (which it necked down below the minimum width at small pins). The pod's +3V3 pads
# get theirs after the signals (+3V3 is routed: the MCU's supply pins join their capacitors).
# The power board's 5 V converter: its short signal nets laid first, pad to pad as in TI's layout
# (SLVSBB4G section 10), locked: the bootstrap and the small-signal nets (EN, RT, FB,
# compensation) direct to their pins.
# Left to the autorouter these were sometimes not routed at all, or the long way round.
# (Only the thin signal nets: laying the wide supply links and the switch node first took the
# room the router needed for +5V. Those are routed at their 1.0 mm class width.)
CONVERTER = [("BOOT", [("U1", "1"), ("C23", "1")], 0.5),
             ("BUCK_EN", [("U1", "3"), ("R42", "1"), ("R41", "2")], 0.3),
             ("BUCK_RT", [("U1", "4"), ("R43", "1")], 0.3),
             ("BUCK_FB", [("U1", "5"), ("R45", "1"), ("R44", "2")], 0.3),
             ("BUCK_COMP", [("U1", "6"), ("R46", "1"), ("C27", "1")], 0.3),
             ("BUCK_COMP_RC", [("R46", "2"), ("C26", "1")], 0.3)]


def preroute(chains):
    laid, failed = 0, []
    for netname, pins, w in chains:
        the_net = net(netname)
        pads = [next(p for p in board.FindFootprintByReference(r).Pads() if p.GetNumber() == n) for r, n in pins]
        for (r0, n0), (r1, n1), p0, p1 in zip(pins, pins[1:], pads, pads[1:]):
            before = {x.m_Uuid.AsString() for x in board.GetTracks()}
            if try_link(p0.GetPosition(), p1.GetPosition(), pcbnew.F_Cu, the_net, widths=(w,)):
                new = [x for x in board.GetTracks() if x.m_Uuid.AsString() not in before]
                for x in new:
                    x.SetLocked(True)
                prerouted.extend((x.GetStart(), x.GetEnd(), x.GetWidth(), x.GetLayer(), x.GetNet()) for x in new)
                laid += 1
            else:
                failed.append(f"{netname} {r0}.{n0}-{r1}.{n1}")
    print(f"converter links laid before routing: {laid}; left to the router: {failed or 'none'}")


# Off by default: with any of these locked wires present, Freerouting left the +5V net in pieces
# all over the board in every attempt (13 of 13); without them +5V routed every time. Kept for
# trying again with a later router: ROUTE_CONVERTER_FIRST=1.
if not POD and os.environ.get("ROUTE_CONVERTER_FIRST", "0") == "1":
    preroute(CONVERTER)
if POD:
    tie_mcu("+3V3")
missed = fan_out({"GND", "+3V3"} if POD else {"GND"})
if POD:
    # an MCU ground pin with no room for its own via (a neighbour's tie or via in the way): to its
    # decoupling capacitor's ground pad instead
    tie_mcu("GND", [m.split(".")[1] for m in missed if m.startswith("U1.")])

# --- 3. Freerouting -------------------------------------------------------------------------------
# The pod's +3V3 fill on In2 stays: it is exported as the +3V3 plane, and Freerouting joins the
# +3V3 pins to it with vias. (Lifting it to let signals use In2 left the plane in pieces that had
# to be stitched together afterwards, and did not route better.)
lifted = []
name = Path(pcb_path).stem
dsn, ses = work / f"{name}.dsn", work / f"{name}.ses"
if not pcbnew.ExportSpecctraDSN(board, str(dsn)):
    sys.exit("DSN export failed")
# KiCad exports every rule area as a Specctra "keepout", which forbids vias as well as wires. The
# large rule areas here forbid TRACKS only (In1 is a plane that vias must still pass through; the
# converter's ground keeps its GND vias): rewrite those as "wire_keepout". The thin edge strips
# stay full keepouts (no via near the edge either). Without this the router could not change layer
# at all on the pod (a via always passes In1) and left most of the board on F.Cu.
import re as _re
text = dsn.read_text()


def _soften(m):
    nums = [float(v) for v in m.group(2).split()]
    xs, ys = nums[1::2], nums[2::2]  # first number is the aperture width
    small = min(max(xs) - min(xs), max(ys) - min(ys)) < 1000  # um: an edge strip
    return m.group(0) if small else m.group(0).replace("(keepout", "(wire_keepout", 1)


text = _re.sub(r'\(keepout "" \(polygon (\S+) ([-0-9. ]+)', _soften, text)
if POD:
    # In1, the GND plane: declare it a POWER layer (no signal tracks; vias pass through) and drop
    # its rule area's keepout. Freerouting treats even a wire_keepout as closed to vias.
    text = text.replace("(layer In1.Cu\n      (type signal)", "(layer In1.Cu\n      (type power)")
    text = _re.sub(r'\(wire_keepout "" \(polygon In1\.Cu [^)]*\)\s*\)', "", text)
    text = text.replace("(layer In2.Cu\n      (type signal)", "(layer In2.Cu\n      (type power)")
# The terminal and fuse-holder pins' wider clearance: KiCad does not export a pad's own clearance
# to the router, so their pads are given to it as copies grown by the difference; it then keeps its
# usual clearance from a pad that is, to it, that much larger. (The real pads keep their size; the
# DRC checks the clearance on them.)
GROW = int(round((pcb_rules.TERMINAL_CLEARANCE - CLEAR) * 1000))  # um
grown = {}


def _grow_padstack(name):
    m = _re.search(r'\(padstack %s\n(.*?)\n    \)' % _re.escape(name), text, _re.S)
    shapes = []
    for layer in ("F.Cu", "B.Cu"):
        c = _re.search(r'\(circle %s ([\d.]+)' % layer, m.group(1))
        if c:
            shapes.append(f"      (shape (circle {layer} {float(c.group(1)) + 2 * GROW:g}))")
            continue
        poly = _re.search(r'\((?:polygon|rect) %s ([-\d. \n]+)\)' % layer, m.group(1))
        nums = [float(v) for v in poly.group(1).split()]
        if "(polygon" in m.group(1):
            nums = nums[1:]  # aperture width first
        xs, ys = nums[0::2], nums[1::2]
        shapes.append(f"      (shape (rect {layer} {min(xs) - GROW:g} {min(ys) - GROW:g} "
                      f"{max(xs) + GROW:g} {max(ys) + GROW:g}))")
    return f"    (padstack {name}_keep\n" + "\n".join(shapes) + "\n      (attach off)\n    )\n"


def _keep_pins(m):
    body = m.group(0)
    if not any(k in m.group(2) for k in pcb_rules.TERMINAL_FOOTPRINTS):
        return body

    def pin(pm):
        grown.setdefault(pm.group(1), None)
        return f"(pin {pm.group(1)}_keep "
    return _re.sub(r'\(pin (\S+) ', pin, body)


if GROW > 0:
    # image names are quoted only when they need it
    text = _re.sub(r'\(image ("?)([^"\s]+)\1(.*?)\n    \)', _keep_pins, text, flags=_re.S)
    extra = "".join(_grow_padstack(n) for n in grown)
    text = text.replace("    (padstack ", extra + "    (padstack ", 1)
    print(f"terminal pins given to the router with {pcb_rules.TERMINAL_CLEARANCE} mm clearance: "
          f"{len(grown)} pad shapes")
dsn.write_text(text)
# Automatic neck-down stays on (narrowing at small pins): without it the 1.0 mm supply tracks
# could not reach the small pins at all (17 left unrouted); its too-narrow pieces are mended below.
args = [java, "-jar", jar, "-de", str(dsn), "-do", str(ses),
        "--router.autorouter.max_passes=100", "--gui.enabled=false"]
args.append("--router.autorouter.ignore_net_classes=Ground")
r = subprocess.run(args, capture_output=True, text=True, timeout=7200)
(work / f"{name}.freerouting.log").write_text(r.stdout + r.stderr)
if not ses.exists():
    sys.exit("Freerouting produced no session:\n" + r.stdout[-2000:] + r.stderr[-2000:])
if not pcbnew.ImportSpecctraSES(board, str(ses)):
    sys.exit("SES import failed")
# The router ignores GND, but the session it returns still carries the ground copper placed
# before routing, some of it altered (stubs shortened off their pads): remove all of it and put
# back exactly what was placed. The pod's +3V3 pieces were protected (locked): any the session
# dropped are put back.
have_tracks = [(t.GetStart(), t.GetEnd()) for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
have_vias = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]


def near(p, q):
    return abs(p.x - q.x) < FM(0.05) and abs(p.y - q.y) < FM(0.05)


def has_track(p0, p1):
    return any((near(p0, a) and near(p1, b)) or (near(p0, b) and near(p1, a)) for a, b in have_tracks)


for t in list(board.GetTracks()):
    if t.GetNetCode() == gnd_net.GetNetCode():
        board.Remove(t)


def add_track(p0, p1, w, layer, n):
    c = pcbnew.PCB_TRACK(board)
    c.SetStart(p0)
    c.SetEnd(p1)
    c.SetWidth(w)
    c.SetLayer(layer)
    c.SetNet(n)
    c.SetLocked(True)
    board.Add(c)


for p0, p1, w, layer, n in prerouted:
    if n.GetNetCode() == gnd_net.GetNetCode() or not has_track(p0, p1):
        add_track(p0, p1, w, layer, n)
for pc, pt, w, n in fanout_made:
    ground = n.GetNetCode() == gnd_net.GetNetCode()
    if ground or not has_track(pc, pt):
        add_track(pc, pt, w, pcbnew.F_Cu, n)
    if ground or not any(near(pt, v) for v in have_vias):
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pt)
        via.SetWidth(FM(VIA_D))
        via.SetDrill(FM(VIA_DRILL))
        via.SetNet(n)
        via.SetLocked(True)
        board.Add(via)
# Freerouting narrows ("necks down") a track where it meets a small pin, sometimes below the
# board's minimum width (0.15-0.18 mm seen): bring those back to the minimum, the DRC then
# checks that the clearances still hold.
# Where the wider track would then come too near another net, it is removed instead (with any
# stub left dangling); only GND tracks are let go like this, the ground pour joins their pads.
# First every track the router left narrower than its net class (its neck-downs at small pins, and
# sometimes a whole run) goes back to the class width, or the widest step towards it that clears.
import fnmatch  # noqa: E402

CLASSES = pcb_rules.NET_CLASSES[board_name]["classes"]
DEFAULT_W = pcb_rules.NET_CLASSES[board_name]["default"]["track"]


def class_width(netname):
    """The net class width of a net, by the same name patterns the project's net classes use."""
    short = netname.split("/")[-1]
    for _name, width, _clear, _via, _amps, pats in CLASSES:
        if any(fnmatch.fnmatch(short, p) for p in pats):
            return width
    return DEFAULT_W


restored = 0
for t in board.GetTracks():
    if t.GetClass() != "PCB_TRACK" or t.IsLocked():
        continue
    want = FM(class_width(t.GetNetname()))
    have = t.GetWidth()
    if have >= want:
        continue
    for w in (want, (want + have) // 2, (want + 3 * have) // 4):
        if w <= have:
            continue
        t.SetWidth(w)
        if not too_near(t):
            restored += 1
            break
        t.SetWidth(have)
print(f"tracks brought back towards their class width: {restored}")
min_w = ds.m_TrackMinWidth
narrowed, dropped, relaid = 0, [], 0



for t in list(board.GetTracks()):
    if t.GetClass() == "PCB_TRACK" and t.GetWidth() < min_w:
        t.SetWidth(min_w)
        narrowed += 1
        if too_near(t):
            if t.GetNetname().split("/")[-1] != "GND":
                # re-lay that short piece between the same two points at the minimum width, by a
                # path clear of every other net; if there is none, the DRC will say
                p0, p1, layer, n = t.GetStart(), t.GetEnd(), t.GetLayer(), t.GetNet()
                board.Remove(t)
                if try_link(p0, p1, layer, n, widths=(TM(min_w),)):
                    relaid += 1
                else:
                    t.SetWidth(min_w)
                    board.Add(t)
                    print(f"neck-down on {n.GetNetname()} could not be widened or re-laid (the DRC will say)")
                continue
            dropped.append(t)
            board.Remove(t)


def prune_dangling():
    """Remove GND track ends that lead nowhere (left by the removals above), repeatedly."""
    removed = 0
    while True:
        ends = {}
        for t in board.GetTracks():
            if t.GetClass() == "PCB_TRACK":
                for p in (t.GetStart(), t.GetEnd()):
                    ends[(p.x, p.y, t.GetLayer())] = ends.get((p.x, p.y, t.GetLayer()), 0) + 1
        loose = []
        for t in board.GetTracks():
            if t.GetClass() != "PCB_TRACK" or t.GetNetname().split("/")[-1] != "GND":
                continue
            for p in (t.GetStart(), t.GetEnd()):
                joined = ends[(p.x, p.y, t.GetLayer())] > 1 or any(
                    pad.GetNetCode() == t.GetNetCode() and pad.IsOnLayer(t.GetLayer()) and pad.HitTest(p)
                    for fp in board.GetFootprints() for pad in fp.Pads()) or any(
                    v.GetClass() == "PCB_VIA" and v.GetNetCode() == t.GetNetCode() and v.HitTest(p)
                    for v in board.GetTracks())
                if not joined:
                    loose.append(t)
                    break
        if not loose:
            return removed
        for t in loose:
            board.Remove(t)
            removed += 1


pruned = prune_dangling() if dropped else 0
print(f"neck-downs widened to the {TM(min_w):g} mm minimum: {narrowed} (re-laid by another path: {relaid}); GND ones removed instead "
      f"(too near another net once widened): {len(dropped)}, with {pruned} dangling stub segments")
for z in lifted:
    board.Add(z)
# Plane fan-out AFTER the signals: vias placed first sat exactly where the signals had to pass.
pcbnew.ZONE_FILLER(board).Fill(board.Zones())

# --- 4. Ground pours and stitching ---------------------------------------------------------------
gnd = net("GND")


def pour(layer, prio=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(gnd)
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(FM(0.2 if POD else 0.3))
    z.SetMinThickness(FM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefSpokeWidth(FM(0.3))
    z.SetThermalReliefGap(FM(0.25))
    z.SetZoneName("GND pour")
    o = z.Outline()
    o.NewOutline()
    for x, y in rect(0.3, 0.3, W - 0.3, H - 0.3):
        o.Append(P(x, y))
    board.Add(z)


pour(pcbnew.F_Cu)
if POD:
    pour(pcbnew.B_Cu)

# Stitching: a GND via every 4 mm wherever it fits (ties the outer pours to the plane)
items = obstacles()
stitched = 0
step = 4.0
y = 3.0
while y < H - 2:
    x = 3.0
    while x < W - 2:
        pt = P(x, y)
        if free_spot(pt, gnd.GetNetCode(), items) and all(
                not z.GetIsRuleArea() or not z.GetDoNotAllowVias() or not z.HitTestFilledArea(pcbnew.F_Cu, pt)
                for z in board.Zones()):
            inside_rule = any(z.GetIsRuleArea() and z.GetDoNotAllowVias() and z.Outline().Contains(pt)
                              for z in board.Zones())
            if not inside_rule:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pt)
                v.SetWidth(FM(VIA_D))
                v.SetDrill(FM(VIA_DRILL))
                v.SetNet(gnd)
                board.Add(v)
                items.append(v)
                stitched += 1
        x += step
    y += step
print(f"stitching: {stitched} GND vias")

# A piece of ground pour cut off by tracks joins nothing on its own: give each such piece a GND via
# inside it (to the other layer's ground), where one fits; a scrap too small for a via (under
# 4 mm^2) is removed instead of being left floating.
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
gnd_zones = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname().split("/")[-1] == "GND"]
for z in gnd_zones:
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_AREA)
    z.SetMinIslandArea(FM(1) * FM(4))
items = obstacles()
anchors = [it for it in items if it.GetNetCode() == gnd.GetNetCode() and
           (it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole()))]
rescued = 0
for z in gnd_zones:
    layer = z.GetLayer()
    polys = z.GetFilledPolysList(layer)
    for i in range(polys.OutlineCount()):
        piece = pcbnew.SHAPE_POLY_SET()
        piece.AddOutline(polys.Outline(i))
        if any(piece.Contains(a.GetPosition()) for a in anchors):
            continue
        # also joined if a GND SMD pad on this layer sits in it with its own via elsewhere: the
        # DRC decides; here only pieces with no drilled GND item get a via
        bb = polys.Outline(i).BBox()
        done = False
        yy = bb.GetTop()
        while yy < bb.GetBottom() and not done:
            xx = bb.GetLeft()
            while xx < bb.GetRight() and not done:
                pt = pcbnew.VECTOR2I(xx, yy)
                if piece.Contains(pt) and free_spot(pt, gnd.GetNetCode(), items) and not any(
                        r.GetIsRuleArea() and r.GetDoNotAllowVias() and r.Outline().Contains(pt)
                        for r in board.Zones()):
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pt)
                    v.SetWidth(FM(VIA_D))
                    v.SetDrill(FM(VIA_DRILL))
                    v.SetNet(gnd)
                    board.Add(v)
                    items.append(v)
                    anchors.append(v)
                    rescued += 1
                    done = True
                xx += FM(0.25)
            yy += FM(0.25)
print(f"ground-pour pieces given their own via: {rescued}")
pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def clusters(the_net):
    """The net's copper in groups that touch (KiCad's own connectivity, zones filled)."""
    board.BuildConnectivity()
    conn = board.GetConnectivity()
    code = the_net.GetNetCode()
    items = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetCode() == code]
    items += [t for t in board.GetTracks() if t.GetNetCode() == code]
    items += [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == code]
    key = {it.m_Uuid.AsString(): it for it in items}
    seen, groups = set(), []
    for it in items:
        k = it.m_Uuid.AsString()
        if k in seen:
            continue
        group, todo = [], [it]
        seen.add(k)
        while todo:
            cur = todo.pop()
            group.append(cur)
            for o in conn.GetConnectedItems(cur):
                ok = o.m_Uuid.AsString()
                if ok in key and ok not in seen:
                    seen.add(ok)
                    todo.append(key[ok])
        groups.append(group)
    return sorted(groups, key=len, reverse=True)


def anchors(group):
    pts = []
    for it in group:
        if it.GetClass() == "ZONE":
            continue  # joined through its pads and vias
        if it.GetClass() == "PCB_TRACK":
            pts += [(it.GetStart(), it.GetLayer()), (it.GetEnd(), it.GetLayer())]
        elif it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole()):
            pts += [(it.GetPosition(), pcbnew.F_Cu), (it.GetPosition(), pcbnew.B_Cu)]
        else:
            pts.append((it.GetPosition(), pcbnew.F_Cu if it.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu))
    return pts



def mend(the_net, widths=(0.5, 0.3)):
    """Join every stray group of the net's copper (a pin or a piece of pour boxed in by other nets'
    tracks) to the main group: a short track, straight or with one bend, on the layer both ends
    share, between the nearest pair of points that it can join clear of every other net.
    Returns (stray groups, the ones it could not join)."""
    groups = clusters(the_net)
    main = anchors(groups[0])
    left = []
    for g in groups[1:]:
        pairs = sorted(((a, b, la) for a, la in anchors(g) for b, lb in main if la == lb),
                       key=lambda q: (q[0].x - q[1].x) ** 2 + (q[0].y - q[1].y) ** 2)
        if not any(try_link(a, b, la, the_net, widths) for a, b, la in pairs[:40]):
            p = g[0].GetPosition()
            left.append((TM(p.x) - ORIGIN[0], TM(p.y) - ORIGIN[1]))
    return len(groups) - 1, left


# Any signal or supply connection the router left out, the same way: a short link at the net's
# class width, or the narrowest IPC-2221 allows for its current (never under the fab minimum).
FAB_MIN = pcb_rules.FAB_RULES[board_name]["min_track_width"]
plane_names = {"GND", "+3V3"} if POD else {"GND"}
signal_left = []
for n in sorted({p.GetNetname() for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetCode() > 0}):
    short_name = n.split("/")[-1]
    if short_name in plane_names or short_name.startswith("unconnected-"):
        continue
    ni = board.FindNet(n)
    if len(clusters(ni)) < 2:
        continue
    amps = next((c[4] for c in CLASSES if any(fnmatch.fnmatch(short_name, p) for p in c[5])), 0.0)
    need = max(FAB_MIN, pcb_rules.ipc2221_width_mm(amps, pcb_rules.NET_CLASSES[board_name]["copper_oz"]) if amps else 0)
    for attempt in range(2):
        k, left = mend(ni, widths=(class_width(n), round(need + 0.005, 2)))
        if not left:
            break
    print(f"{short_name}: {k} pieces the router left unjoined; could not join: {left or 'none'}")
    signal_left += left

for plane_net in ([gnd, net("+3V3")] if POD else [gnd]):
    for attempt in range(3):  # each new track can split a pour or join several pieces: refill, repeat
        n, left = mend(plane_net)
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        print(f"{plane_net.GetNetname()} links missing: {n}; could not join: "
              f"{[(round(x, 1), round(y, 1)) for x, y in left]}")
        if not n or len(left) == n:
            break
# Last fill: every piece of ground pour now joined is kept; scraps that touch nothing are removed.
for z in gnd_zones:
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(pcb_path, board)
tracks = sum(1 for t in board.GetTracks() if t.GetClass() == "PCB_TRACK")
vias = sum(1 for t in board.GetTracks() if t.GetClass() == "PCB_VIA")
print(f"routed and saved: {tracks} track segments, {vias} vias")
