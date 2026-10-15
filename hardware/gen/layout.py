"""Builds each board's .kicad_pcb with every part PLACED, ready to route: run with KiCad's own Python
(it needs pcbnew). From WSL:

    hardware/gen/layout.sh [--force] [pod_board] [power_board]

For each board it:
  1. writes the net classes and fabrication rules (pcb_rules.py) into <board>.kicad_pro;
  2. starts from the outline, hole and keep-zone drawing of outlines.py;
  3. loads every footprint from KiCad's library, assigns each pad its net (from the same board
     description the schematic is generated from), and places it (placement.py);
  4. links each footprint to its schematic symbol (the symbol's id path, read from the .kicad_sch
     files), so KiCad's "Update PCB from Schematic" and the schematic-parity check work;
  5. adds the copper zones (pod: GND on In1, +3V3 on In2; power board: GND on the bottom);
  6. saves, and refuses to overwrite a board that already has tracks, unless --force.
Then layout.sh runs KiCad's DRC with the schematic-parity check, and fails on anything but
"unconnected" items (the routing is yours).
"""

import importlib
import json
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
HW = HERE.parent
sys.path.insert(0, str(HERE))
import outlines  # noqa: E402
import pcb_rules  # noqa: E402
import placement  # noqa: E402
from kisch import parse  # noqa: E402

FP_ROOT = Path(r"C:\Program Files\KiCad\10.0\share\kicad\footprints")
ORIGIN = (50.0, 50.0)  # outlines.py draws the board here on the sheet


def mm(x, y):
    return pcbnew.VECTOR2I_MM(ORIGIN[0] + x, ORIGIN[1] + y)


def symbol_paths(board):
    """reference -> footprint path "/<sheet uuid>/<symbol uuid>" from the schematic files."""
    out = {}
    for f in (HW / board).glob("*.kicad_sch"):
        tree = parse(f.read_text(encoding="utf-8"))
        for node in tree[1:]:
            if not (isinstance(node, list) and node and node[0] == "symbol"):
                continue
            uid = next(x[1] for x in node if isinstance(x, list) and x and x[0] == "uuid")
            for inst in (x for x in node if isinstance(x, list) and x and x[0] == "instances"):
                for proj in inst[1:]:
                    for path in proj[2:]:
                        if not (isinstance(path, list) and path[0] == "path"):
                            continue
                        ref = next(x[1] for x in path if isinstance(x, list) and x[0] == "reference")
                        ref = ref.strip('"')
                        sheet = path[1].strip('"').split("/")[1:]  # drop the root
                        out[ref] = "/" + "/".join([s for s in sheet if s] + [uid.strip('"')])
    return out


def netlist(board):
    """KiCad's own netlist (exported by check.sh): {(ref, pin): net name}, {ref: description}.
    Its net names carry the sheet path (/CAN_TX, /CAN transceiver for OBD-II/OBD_CANH) and
    unconnected pins have their own nets; the PCB must use exactly these names."""
    tree = parse((HW / board / f"{board}.net").read_text(encoding="utf-8"))
    pins, desc = {}, {}

    def kids(node, key):
        return [x for x in node[1:] if isinstance(x, list) and x and x[0] == key]

    def val(node, key):
        k = kids(node, key)
        return k[0][1].strip('"') if k and len(k[0]) > 1 else ""

    for comps in kids(tree, "components"):
        for c in kids(comps, "comp"):
            desc[val(c, "ref")] = val(c, "description")
    for nets in kids(tree, "nets"):
        for n in kids(nets, "net"):
            name = val(n, "name")
            for node in kids(n, "node"):
                pins[(val(node, "ref"), val(node, "pin"))] = name
    return pins, desc


def write_project(board):
    pro = HW / board / f"{board}.kicad_pro"
    tmpl = json.loads((HW / "pod_board" / "pod_board.kicad_pro").read_text())
    d = json.loads(pro.read_text())
    for key in ("board", "net_settings"):
        if key not in d:
            d[key] = json.loads(json.dumps(tmpl[key]))
    spec = pcb_rules.NET_CLASSES[board]
    rules = d["board"]["design_settings"]["rules"]
    rules.update(pcb_rules.FAB_RULES[board])
    d["board"]["design_settings"]["track_widths"] = [0.0] + pcb_rules.TRACK_CHOICES[board]
    d["board"]["design_settings"]["via_dimensions"] = [
        {"diameter": 0.0, "drill": 0.0},
        {"diameter": spec["default"]["via"][0], "drill": spec["default"]["via"][1]},
        {"diameter": 1.0, "drill": 0.5}]
    ns = d["net_settings"]
    default = next(c for c in ns["classes"] if c["name"] == "Default")
    default.update(track_width=spec["default"]["track"], clearance=spec["default"]["clearance"],
                   via_diameter=spec["default"]["via"][0], via_drill=spec["default"]["via"][1])
    classes = [default]
    patterns = []
    for i, (name, track, clear, via, _amps, pats) in enumerate(spec["classes"]):
        c = json.loads(json.dumps(default))
        c.update(name=name, track_width=track, clearance=clear, via_diameter=via[0],
                 via_drill=via[1], priority=i)
        if name == "USB":
            c.update(diff_pair_width=0.2, diff_pair_gap=0.2)
        classes.append(c)
        # KiCad's net names carry the sheet path (/ACT_12V): match the name at the end.
        patterns += [{"netclass": name, "pattern": "*" + p} for p in pats]
    ns["classes"] = classes
    ns["netclass_patterns"] = patterns
    pro.write_text(json.dumps(d, indent=2) + "\n")


def zone(board, net, layer, w, h, priority=0, inset=0.3, clearance=0.2):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    # 0.2 mm clearance and 0.3 mm spokes: with 0.3 / 0.25 the closely spaced through-hole pins
    # (the DB-25's 2.77 mm rows) had no room for spokes and never joined the plane ("starved
    # thermal", found by the routability check).
    z.SetLocalClearance(pcbnew.FromMM(clearance))  # at least every net class's clearance
    z.SetMinThickness(pcbnew.FromMM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.3))
    z.SetThermalReliefGap(pcbnew.FromMM(0.25))
    o = z.Outline()
    o.NewOutline()
    for x, y in ((inset, inset), (w - inset, inset), (w - inset, h - inset), (inset, h - inset)):
        o.Append(mm(x, y))
    board.Add(z)
    return z


def actuator_rail(pcb, net, plan):
    """The 10 A actuator rail as copper, not a track: a top-layer zone joining pin 1 of J5 (from
    the kill relay), J6 (to the BTS7960) and J7 (the magnet), which sit one above the other on the
    right edge. A 4.4 mm spine inboard of the pins (pin 2 of each is another net, so the spine
    passes beside them), with a finger out to each pin 1; solid connection, no thermal spokes,
    for the current. (A router does wide tracks badly; LAYOUT_GUIDE.md F3.)"""
    xs = [plan[r][0] for r in ("J5", "J6", "J7")]
    ys = [plan[r][1] for r in ("J5", "J6", "J7")]
    pin_x, top = xs[0], min(ys) - 2.5
    bottom = plan["D14"][1] + 1.5  # down over the magnet's flyback diode D14 (cathode on the rail)
    spine = (pin_x - 6.6, pin_x - 2.2)  # 4.4 mm wide, clear of the pins' 2.6 mm pads
    z = pcbnew.ZONE(pcb)
    z.SetLayer(pcbnew.F_Cu)
    z.SetNet(net)
    z.SetAssignedPriority(1)
    z.SetLocalClearance(pcbnew.FromMM(0.3))
    z.SetMinThickness(pcbnew.FromMM(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    o = z.Outline()
    o.NewOutline()
    pts = [(spine[0], top), (spine[1], top)]
    for y in ys:  # a finger out to each pin 1 (3.0 mm tall, across the pad)
        pts += [(spine[1], y - 1.5), (pin_x + 1.6, y - 1.5), (pin_x + 1.6, y + 1.5), (spine[1], y + 1.5)]
    pts += [(spine[1], bottom), (spine[0], bottom)]
    for x, y in pts:
        o.Append(mm(x, y))
    pcb.Add(z)


def silk_text(pcb, text, pos, size, angle=0, justify=0):
    """A line of top silkscreen text at pos (board units); justify -1 left, 0 centre, 1 right."""
    t = pcbnew.PCB_TEXT(pcb)
    t.SetText(text)
    t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
    t.SetTextThickness(pcbnew.FromMM(0.15 if size >= 1.0 else 0.13))
    t.SetTextAngleDegrees(angle)
    t.SetHorizJustify({-1: pcbnew.GR_TEXT_H_ALIGN_LEFT, 0: pcbnew.GR_TEXT_H_ALIGN_CENTER,
                       1: pcbnew.GR_TEXT_H_ALIGN_RIGHT}[justify])
    t.SetPosition(pos)
    pcb.Add(t)
    bb = t.GetBoundingBox()
    return max(bb.GetWidth(), bb.GetHeight())  # its length along the text


def labels(pcb, board):
    """The silkscreen names of every terminal, port, header and button (placement.LABELS): pin
    names in line with each pin just outside the part's courtyard, the title beyond them."""
    V = pcbnew.VECTOR2I
    gap, pin_h = pcbnew.FromMM(0.3), 0.8
    for ref, lab in placement.LABELS.get(board, {}).items():
        fp = pcb.FindFootprintByReference(ref)
        fp.BuildCourtyardCaches()
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        side, names = lab["side"], lab["pins"]
        pads = []
        for i in range(len(names)):
            pads.append(next(p for p in fp.Pads() if p.GetNumber() == str(i + 1)).GetPosition())
        reach = 0  # how far the pin names stand out from the courtyard
        if side in "NS" and len(pads) > 1:
            pitch = min(abs(a.x - b.x) for a, b in zip(pads, pads[1:]))
        for name, p in zip(names, pads):
            if side in "EW":
                x = bb.GetRight() + gap if side == "E" else bb.GetLeft() - gap
                reach = max(reach, silk_text(pcb, name, V(x, p.y), pin_h, 0, -1 if side == "E" else 1))
                continue
            edge = bb.GetTop() - gap if side == "N" else bb.GetBottom() + gap
            wide = len(name) * pcbnew.FromMM(0.8 * pin_h) > pitch - pcbnew.FromMM(0.4)
            if wide:  # upright, reading upwards, standing out from the courtyard
                reach = max(reach, silk_text(pcb, name, V(p.x, edge), pin_h, 90,
                                             -1 if side == "N" else 1))
            else:
                half = pcbnew.FromMM(pin_h / 2)
                y = edge - half if side == "N" else edge + half
                reach = max(reach, pcbnew.FromMM(pin_h))
                silk_text(pcb, name, V(p.x, y), pin_h, 0, 0)
        title = f"{ref} {lab['title']}" if lab["ref"] else lab["title"]
        size = lab["size"]
        if lab["at"]:
            x, y, rot = lab["at"]
            silk_text(pcb, title, mm(x, y), size, rot)
        else:
            if pads:
                cx, cy = (pads[0].x + pads[-1].x) // 2, (pads[0].y + pads[-1].y) // 2
            else:
                c = bb.GetCenter()
                cx, cy = c.x, c.y
            off = gap + (reach + gap if names else 0) + pcbnew.FromMM(size / 2)
            pos = {"E": V(bb.GetRight() + off, cy), "W": V(bb.GetLeft() - off, cy),
                   "N": V(cx, bb.GetTop() - off), "S": V(cx, bb.GetBottom() + off)}[side]
            dx, dy = lab["shift"]
            pos = V(pos.x + pcbnew.FromMM(dx), pos.y + pcbnew.FromMM(dy))
            silk_text(pcb, title, pos, size, 90 if side in "EW" else 0)
        if lab["ref"]:
            fp.Reference().SetVisible(False)  # the title carries the reference


def tidy_refs(pcb, board_w, board_h):
    """Move each visible reference designator to the first free spot around its part (above,
    below, right, left; then the same turned 90 degrees), free meaning clear of every pad, every
    silkscreen line and every text already placed, and inside the board. A reference with no free
    spot is kept off the silkscreen (it stays on the fab layer, for assembly): returned."""
    margin = pcbnew.FromMM(0.15)
    hidden = []

    def grown(bb):
        b = pcbnew.BOX2I(bb.GetOrigin(), bb.GetSize())
        b.Inflate(margin)
        return b

    pads, silk, texts = [], [], []
    for fp in pcb.GetFootprints():
        pads += [grown(p.GetBoundingBox()) for p in fp.Pads()]
        silk += [(fp.GetReference(), g.GetBoundingBox()) for g in fp.GraphicalItems()
                 if g.GetLayer() == pcbnew.F_SilkS and not isinstance(g, pcbnew.PCB_TEXT)]
    for d in pcb.GetDrawings():
        if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.F_SilkS:
            texts.append(grown(d.GetBoundingBox()))
    inside = pcbnew.BOX2I(mm(0.3, 0.3), pcbnew.VECTOR2I_MM(board_w - 0.6, board_h - 0.6))
    fps = sorted((fp for fp in pcb.GetFootprints() if fp.Reference().IsVisible()),
                 key=lambda f: f.GetReference())
    for fp in fps:
        ref = fp.Reference()
        fp.BuildCourtyardCaches()
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False, False)
        c = bb.GetCenter()
        own = fp.GetReference()
        best = None  # (score, angle, position): fewest things overlapped, a pad counting most
        for gap in (pcbnew.FromMM(0.25), pcbnew.FromMM(1.0)):
            for angle in (0, 90):
                ref.SetTextAngleDegrees(angle)
                ref.SetPosition(c)
                tb = ref.GetBoundingBox()
                hw, hh = tb.GetWidth() // 2, tb.GetHeight() // 2
                spots = []
                for k in (0, 1, -1):  # centred on the side, then slid along it
                    spots += [pcbnew.VECTOR2I(c.x + k * hw, bb.GetTop() - gap - hh),
                              pcbnew.VECTOR2I(c.x + k * hw, bb.GetBottom() + gap + hh),
                              pcbnew.VECTOR2I(bb.GetRight() + gap + hw, c.y + k * hh),
                              pcbnew.VECTOR2I(bb.GetLeft() - gap - hw, c.y + k * hh)]
                for pos in spots:
                    ref.SetPosition(pos)
                    box = ref.GetBoundingBox()
                    if not inside.Contains(box):
                        continue
                    score = (10 * sum(box.Intersects(p) for p in pads)
                             + sum(box.Intersects(x) for x in texts)
                             + sum(box.Intersects(s) for r, s in silk if r != own))
                    if best is None or score < best[0]:
                        best = (score, angle, pos)
            if best and best[0] == 0:
                break
        if best and best[0] == 0:
            ref.SetTextAngleDegrees(best[1])
            ref.SetPosition(best[2])
            texts.append(grown(ref.GetBoundingBox()))
        else:  # nowhere clear: printed over something it would be unreadable; fab layer only
            fp.AutoPositionFields()
            ref.SetVisible(False)
            hidden.append(own)
    return hidden


def build(board, force):
    path = HW / board / f"{board}.kicad_pcb"
    if path.exists() and not force:
        existing = pcbnew.LoadBoard(str(path))
        if len(existing.GetTracks()) > 0:
            sys.exit(f"{path}: has {len(existing.GetTracks())} tracks: a layout in progress "
                     "(--force to replace it)")
    write_project(board)
    b = outlines.BOARDS[board]
    path.write_text(outlines.pcb(board, b), encoding="utf-8")
    pcb = pcbnew.LoadBoard(str(path))
    pcb.SetCopperLayerCount(b["copper"])

    m = importlib.import_module(board)
    parts = {p["ref"]: p for _, ps, _ in m.groups for p in ps}
    plan = placement.PLANS[board]
    links = symbol_paths(board)
    problems = []
    missing = sorted(set(parts) - set(plan))
    extra = sorted(set(plan) - set(parts))
    if missing or extra:
        problems.append(f"placement: unplaced {missing}, unknown {extra}")

    pinnet, desc = netlist(board)
    # The board description and KiCad's netlist must agree pin by pin (the name at the path's end).
    for ref, p in parts.items():
        for num, net in p["nets"].items():
            got = pinnet.get((ref, num))
            if got is None or got.split("/")[-1] != net:
                problems.append(f"{ref} pin {num}: description says {net}, netlist {got}")
    nets = {}
    for n in sorted(set(pinnet.values())):
        ni = pcbnew.NETINFO_ITEM(pcb, n)
        pcb.Add(ni)
        nets[n] = ni
    byname = {n.split("/")[-1]: ni for n, ni in nets.items()}  # for the zones

    for ref, p in sorted(parts.items()):
        lib, name = p["fp"].split(":")
        fp = pcbnew.FootprintLoad(str(FP_ROOT / f"{lib}.pretty"), name)
        if fp is None:
            problems.append(f"{ref}: footprint {p['fp']} not found")
            continue
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(p["value"])
        if desc.get(ref):
            fp.SetField("Description", desc[ref])
        if ref in links:
            fp.SetPath(pcbnew.KIID_PATH(links[ref]))
        else:
            problems.append(f"{ref}: no schematic symbol found to link to")
        pads = {pad.GetNumber() for pad in fp.Pads()}
        for (r, num) in pinnet:
            if r == ref and num not in pads:
                problems.append(f"{ref}: the schematic's pin {num} has no pad in {name}")
        for pad in fp.Pads():
            n = pinnet.get((ref, pad.GetNumber()))
            if n is not None:
                pad.SetNet(nets[n])
        if ref in plan:
            x, y, rot, _why = plan[ref]
            fp.SetPosition(mm(x, y))
            fp.SetOrientationDegrees(rot)
            if ref.startswith(("J", "H")):
                fp.SetLocked(True)  # edge connectors and holes are fixed by the enclosure
        if ref.startswith("H"):
            fp.SetExcludedFromBOM(False)  # as the schematic's mounting-hole symbols
        if any(k in p["fp"] for k in pcb_rules.TERMINAL_FOOTPRINTS):
            for pad in fp.Pads():  # wider clearance round the terminal and fuse-holder pins
                pad.SetLocalClearance(pcbnew.FromMM(pcb_rules.TERMINAL_CLEARANCE))
        # Reference labels: 0.8 mm (the fab's minimum readable height) and placed clear of the
        # part, so dense rows stay legible.
        ref_text = fp.Reference()
        ref_text.SetTextSize(pcbnew.VECTOR2I_MM(0.8, 0.8))
        ref_text.SetTextThickness(pcbnew.FromMM(0.12))
        pcb.Add(fp)
        fp.AutoPositionFields()

    labels(pcb, board)
    for ref in placement.HIDE_REFS.get(board, []):
        pcb.FindFootprintByReference(ref).Reference().SetVisible(False)
    w, h = b["w"], b["h"]
    hidden = tidy_refs(pcb, w, h)
    if hidden:
        print(f"{board}: references on the fab layer only (no clear spot): {' '.join(hidden)}")
    if board == "pod_board":
        zone(pcb, byname["GND"], pcbnew.In1_Cu, w, h)
        zone(pcb, byname["+3V3"], pcbnew.In2_Cu, w, h)
    else:
        zone(pcb, byname["GND"], pcbnew.B_Cu, w, h, clearance=0.3)
        actuator_rail(pcb, byname["ACT_12V"], plan)
    pcbnew.ZONE_FILLER(pcb).Fill(pcb.Zones())
    pcbnew.SaveBoard(str(path), pcb)
    write_table(board, parts, plan)
    for p in problems:
        print(f"PROBLEM {board}: {p}")
    print(f"wrote {path}: {len(parts)} parts placed, {len(nets)} nets")
    return not problems


def write_table(board, parts, plan):
    """hardware/<board>/placement.md: every part, where it went and why (for the layout guide)."""
    b = outlines.BOARDS[board]
    rows = [f"<!-- Generated by hardware/gen/layout.py from placement.py: do not edit. -->\n\n"
            f"# {board}: placement ({b['w']:g} x {b['h']:g} mm, {b['copper']} layers)\n\n"
            "x, y: mm from the board's top-left corner (y down); rotation: degrees, "
            "counter-clockwise; the position is the footprint's origin (pin 1 for connectors and "
            "headers, the centre for most SMD parts). See hardware/LAYOUT_GUIDE.md.\n\n"
            "| Ref | Value | x | y | Rot | Why it is there |\n|---|---|---|---|---|---|\n"]

    def key(r):
        head = r.rstrip("0123456789")
        return (head, int(r[len(head):] or 0))

    for ref in sorted(plan, key=key):
        x, y, rot, why = plan[ref]
        rows.append(f"| {ref} | {parts[ref]['value']} | {x:.2f} | {y:.2f} | {rot} | {why} |\n")
    (HW / board / "placement.md").write_text("".join(rows), encoding="utf-8")


def main():
    args = sys.argv[1:]
    force = "--force" in args
    boards = [a for a in args if not a.startswith("--")] or ["pod_board", "power_board"]
    pcb_rules.check_widths()
    ok = all([build(bd, force) for bd in boards])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
