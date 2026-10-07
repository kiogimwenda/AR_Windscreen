"""kisheet: hierarchical KiCad 10 schematics with drawn wires, from the same board descriptions.

kisch.py connects every pin by net label. That is exact, but hard to read: parts that touch in the
circuit can sit far apart on the page, and the reader matches names by eye. This module draws a
board the way a person would:
  - one sheet per function, and a root sheet that is the block diagram: sheet symbols wired
    together, every wire named;
  - parts placed by hand in the board's drawing file (position, rotation, mirror), signal flowing
    left to right, with notes explaining each block;
  - wires routed between the pins of each net by a small maze router on KiCad's 1.27 mm grid;
  - power symbols (GND, +5V, ...) at power pins, or one symbol on a drawn rail; hierarchical
    labels where a net leaves its sheet; a local label naming every other net, so the netlist keeps
    the description's names.
The router never lets two nets touch. A wire may cross another only straight over a plain run, at
right angles, never at a bend, pin or junction: KiCad does not connect a plain crossing. Then
generate.py compares the exported netlist with the label-only drawing's, connection by
connection, so a drawing mistake cannot change the circuit.

Coordinates in the drawing files are in units of 2.54 mm (U); KiCad's own grid is 1.27 mm.
"""

import heapq
import math
import uuid as uuidlib
from pathlib import Path

from kisch import POWER_NETS, dump, find, fnt, q, uid, unq

G = 1.27   # routing grid, mm
U = 2.54   # drawing unit, mm
PAPER = {"A4": (297.0, 210.0), "A3": (420.0, 297.0), "A2": (594.0, 420.0)}
DIRS = {"R": (1, 0), "L": (-1, 0), "D": (0, 1), "U": (0, -1)}
OPP = {"R": "L", "L": "R", "U": "D", "D": "U"}
COMPANY = "AR Windscreen FYP - Kiogora Ian Mwenda"


def cl(v_mm):
    return int(round(v_mm / G))


def mm(i):
    return round(i * G, 3)


def step(cell, d, k=1):
    return (cell[0] + DIRS[d][0] * k, cell[1] + DIRS[d][1] * k)


def vdir(dx, dy):
    for k, v in DIRS.items():
        if v == (int(round(dx)), int(round(dy))):
            return k
    raise ValueError((dx, dy))


# --- Symbol geometry ---------------------------------------------------------------------------
def body_box(lib, lib_id, unit=1):
    """Bounding box of a symbol's graphics in library coordinates (y up), or None."""
    xs, ys = [], []
    for sub in find(lib.symbol(lib_id), "symbol"):
        tail = unq(sub[1]).rsplit("_", 2)
        try:
            u, style = int(tail[-2]), int(tail[-1])
        except (ValueError, IndexError):
            continue
        if u not in (0, unit) or style not in (0, 1):
            continue
        for g in sub[2:]:
            if not isinstance(g, list):
                continue
            pts = []
            if g[0] == "rectangle":
                pts = [find(g, "start")[0], find(g, "end")[0]]
            elif g[0] in ("polyline", "bezier"):
                pts = find(find(g, "pts")[0], "xy")
            elif g[0] == "arc":
                pts = [find(g, k)[0] for k in ("start", "mid", "end")]
            elif g[0] == "circle":
                cx, cy = (float(v) for v in find(g, "center")[0][1:3])
                r = float(find(g, "radius")[0][1])
                xs += [cx - r, cx + r]
                ys += [cy - r, cy + r]
            for p in pts:
                xs.append(float(p[1]))
                ys.append(float(p[2]))
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def xform(px, py, rot, mirror):
    """Library offset (y up) -> sheet offset (y down), for rotation rot (CCW) and mirror about Y."""
    if mirror:
        px = -px
    a = math.radians(rot)
    rx = px * math.cos(a) - py * math.sin(a)
    ry = px * math.sin(a) + py * math.cos(a)
    return round(rx, 4), round(-ry, 4)


class Placed:
    """A library symbol placed on a sheet: its pins' cells and outward directions, its body box."""
    is_sheet = False

    def __init__(self, lib, part, x, y, rot, mirror, fields):
        self.part, self.x, self.y, self.rot, self.mirror, self.fields = part, x, y, rot, mirror, fields
        self.unit = part.get("unit", 1)
        self.pins = {}
        for num, (px, py, ang, length, name, etype) in lib.pins(part["sym"], self.unit).items():
            ox, oy = xform(px, py, rot, mirror)
            bx, by = xform(math.cos(math.radians(ang)), math.sin(math.radians(ang)), rot, mirror)
            self.pins[num] = {"cell": (cl(x + ox), cl(y + oy)), "out": OPP[vdir(bx, by)],
                              "len": length, "etype": etype}
            if abs((x + ox) / G - round((x + ox) / G)) > 0.01 or abs((y + oy) / G - round((y + oy) / G)) > 0.01:
                raise ValueError(f"{part['ref']} pin {num} is off the 1.27 mm grid")
        pts = []
        bb = body_box(lib, part["sym"], self.unit)
        if bb:
            for px, py in ((bb[0], bb[1]), (bb[2], bb[3]), (bb[0], bb[3]), (bb[2], bb[1])):
                ox, oy = xform(px, py, rot, mirror)
                pts.append((x + ox, y + oy))
        for p in self.pins.values():
            dx, dy = DIRS[OPP[p["out"]]]
            pts.append((mm(p["cell"][0]) + dx * p["len"], mm(p["cell"][1]) + dy * p["len"]))
        if not pts:
            pts = [(x - 2, y - 2), (x + 2, y + 2)]
        self.box = (min(p[0] for p in pts), min(p[1] for p in pts),
                    max(p[0] for p in pts), max(p[1] for p in pts))

    @property
    def nets(self):
        return self.part.get("nets", {})


class SheetBox:
    """A sheet symbol on the root sheet; its pins are the sub-sheet's hierarchical labels."""
    is_sheet = True

    def __init__(self, sheet, x, y, w, pins, page, min_h=0):
        # pins: one entry per row down the box: a list of (net, side, shape), empty for a gap
        self.sheet, self.page = sheet, page
        h = max((len(pins) + 2) * U, min_h * U)
        self.x, self.y, self.w, self.h = x, y, w, h
        self.box = (x, y, x + w, y + h)
        self.pins = {}
        self.shapes = {}
        for row, entry in enumerate(pins, start=1):
            for net, side, shape in entry:  # a row: pins on the left and/or right edge
                px = x if side == "L" else x + w
                self.pins[net] = {"cell": (cl(px), cl(y + (row + 0.5) * U)), "out": side, "len": 0,
                                  "etype": "passive"}
                self.shapes[net] = shape
        self.part = {"ref": sheet.name, "nets": {n: n for n in self.pins}}

    @property
    def nets(self):
        return self.part["nets"]


# --- A sheet as described by the drawing file ---------------------------------------------------
class Sheet:
    def __init__(self, lib, parts, name, title, paper="A4", file=None):
        self.lib, self.parts = lib, parts
        self.name, self.title, self.paper = name, title, paper
        self.file = file or name.lower().replace(" ", "_").replace(".", "") + ".kicad_sch"
        self.uuid = str(uuidlib.uuid4())       # the sheet symbol's uuid, in instance paths
        self.file_uuid = str(uuidlib.uuid4())
        self.placed, self.texts, self.flags, self.rails = [], [], [], []
        self.port_side, self.port_row, self.shapes = {}, {}, {}
        self.default_side = "L"
        self.columns = {}   # net -> x [U]: where its vertical run should be (a fan-in to a connector)
        self.splits = {}    # net -> [[(ref, pin), ...], ...]: pieces drawn apart, joined by name

    def add(self, ref, x, y, rot=0, mirror=False, fields=None):
        """Place a part by reference at (x, y) in units of 2.54 mm."""
        p = Placed(self.lib, self.parts[ref], x * U, y * U, rot, mirror, fields)
        self.placed.append(p)
        return p

    def rail(self, net, x, y, refs=None):
        """One power symbol at (x, y) [U], wired to this net's pins on the given parts (default:
        all on the sheet) instead of giving each pin its own symbol."""
        self.rails.append({"net": net, "cell": (cl(x * U), cl(y * U)), "refs": refs})

    def port(self, net, side=None, row=None, shape=None):
        if side:
            self.port_side[net] = side
        if row is not None:
            self.port_row[net] = row
        if shape:
            self.shapes[net] = shape

    def text(self, x, y, s, size=1.6):
        self.texts.append((x * U, y * U, s, size))

    def split(self, net, *pieces):
        """Draw this net as separate pieces (each a list of (ref, pin); the rest is one more piece),
        each named by a label: for pins a part already joins inside itself, such as an ESD
        array's pass-through pins, where an outside wire would only loop around the part."""
        self.splits[net] = [list(pc) for pc in pieces]

    def flag(self, net, x, y):
        self.flags.append((net, x * U, y * U))

    def nets(self):
        out = {}
        for p in self.placed:
            for num, net in p.nets.items():
                out.setdefault(net, []).append((p, num))
        return out


# --- Router ----------------------------------------------------------------------------------------
class Grid:
    def __init__(self, w, h):
        self.w, self.h = cl(w), cl(h)
        self.hard = set()
        self.soft = {}
        self.owner = {}       # cell -> net
        self.links = {}       # cell -> {direction: net}
        self.pinpts = {}      # cell -> net (pin, port, rail and power-symbol connection points)
        self.crossed = set()

    def block_rect(self, x0, y0, x1, y1):
        for i in range(cl(x0), cl(x1) + 1):
            for j in range(cl(y0), cl(y1) + 1):
                self.hard.add((i, j))

    def soft_rect(self, x0, y0, x1, y1, cost):
        for i in range(cl(x0), cl(x1) + 1):
            for j in range(cl(y0), cl(y1) + 1):
                self.soft[(i, j)] = max(self.soft.get((i, j), 0), cost)

    def own_dirs(self, cell, net):
        return {d for d, n in self.links.get(cell, {}).items() if n == net}

    def plain_run(self, cell):
        """True if exactly one net passes straight through (crossable)."""
        ds = self.links.get(cell, {})
        return (cell not in self.pinpts and cell not in self.crossed and len(ds) == 2
                and set(ds) in ({"L", "R"}, {"U", "D"}))

    def link(self, a, b, net):
        d = vdir(b[0] - a[0], b[1] - a[1])
        for cell, dd in ((a, d), (b, OPP[d])):
            self.links.setdefault(cell, {})[dd] = net
            o = self.owner.get(cell)
            if o is not None and o != net:
                self.crossed.add(cell)
            elif o is None:
                self.owner[cell] = net

    def enterable(self, net, cell, d):
        if not (1 <= cell[0] < self.w - 1 and 1 <= cell[1] < self.h - 1):
            return False
        if cell in self.hard or cell in self.crossed or cell in self.pinpts:
            return False
        o = self.owner.get(cell)
        if o is None:
            return True
        if o == net:
            return False  # own cells are targets
        return self.plain_run(cell) and d not in self.links[cell] and OPP[d] not in self.links[cell]

    def route(self, net, start, start_dir, targets, bend=5.0, cross=30.0, near=1.5, column=None):
        """A* from a terminal leaving in start_dir to a target cell ({cell: allowed arrival
        directions, or None for any})."""
        tl = list(targets)

        def h(cell):
            return min(abs(cell[0] - a) + abs(cell[1] - b) for a, b in tl)

        first = step(start, start_dir)
        if first in targets:
            ok = targets[first]
            return [start, first] if ok is None or start_dir in ok else None
        if not self.enterable(net, first, start_dir):
            return None
        best = {(first, start_dir): 1.0}
        prev = {(first, start_dir): None}
        heap = [(1 + h(first), 1.0, first, start_dir)]
        while heap:
            f, g, cell, d = heapq.heappop(heap)
            if best.get((cell, d), 1e18) < g:
                continue
            crossing = self.owner.get(cell) not in (None, net)
            for nd in DIRS:
                if nd == OPP[d] or (crossing and nd != d):
                    continue
                nxt = step(cell, nd)
                if nxt in targets:
                    ok = targets[nxt]
                    if ok is not None and nd not in ok:
                        continue
                    path, key = [nxt, cell], (cell, d)
                    while prev[key] is not None:
                        key = prev[key]
                        path.append(key[0])
                    path.append(start)
                    return list(reversed(path))
                if not self.enterable(net, nxt, nd):
                    continue
                ng = g + 1 + (bend if nd != d else 0) + self.soft.get(nxt, 0)
                if column is not None and nd in ("U", "D") and nxt[0] != column:
                    ng += 3  # vertical runs belong in this net's column
                if self.owner.get(nxt) not in (None, net):
                    ng += cross
                for dd in DIRS.values():  # keep a little away from other nets' wires
                    o = self.owner.get((nxt[0] + dd[0], nxt[1] + dd[1]))
                    if o is not None and o != net:
                        ng += near
                        break
                if ng < best.get((nxt, nd), 1e18):
                    best[(nxt, nd)] = ng
                    prev[(nxt, nd)] = (cell, d)
                    heapq.heappush(heap, (ng + h(nxt), ng, nxt, nd))
        return None


# --- Writer ------------------------------------------------------------------------------------------
class Writer:
    def __init__(self, sheet, project, path, counters):
        self.s, self.project, self.path, self.n = sheet, project, path, counters
        self.items, self.used = [], {}
        w, h = PAPER[sheet.paper]
        self.w, self.h = w, h
        self.grid = Grid(w, h)
        self.unrouted = []
        self.net_cells = {}
        self.breaks = set()

    def lib_sym(self, lib_id):
        if lib_id not in self.used:
            self.used[lib_id] = self.s.lib.symbol(lib_id)

    def wire(self, a, b):
        self.items.append(["wire", ["pts", ["xy", str(mm(a[0])), str(mm(a[1]))],
                                    ["xy", str(mm(b[0])), str(mm(b[1]))]],
                           ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid()]])

    # -- symbols
    def symbol(self, p):
        if p.is_sheet:
            return self.sheet_symbol(p)
        part = p.part
        self.lib_sym(part["sym"])
        x0, y0, x1, y1 = p.box
        outs = {pin["out"] for pin in p.pins.values()}
        if p.fields:
            mode = p.fields
        elif len(p.pins) == 2 and outs == {"U", "D"}:
            mode = "right"
        elif len(p.pins) == 2 and outs == {"L", "R"}:
            mode = "split"
        else:
            mode = "above"
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        spots = {
            "right": ((x1 + 0.8, cy - 0.4, "left bottom"), (x1 + 0.8, cy + 0.4, "left top")),
            "left": ((x0 - 0.8, cy - 0.4, "right bottom"), (x0 - 0.8, cy + 0.4, "right top")),
            "above": ((cx, y0 - 2.4, "bottom"), (cx, y0 - 0.6, "bottom")),
            "below": ((cx, y1 + 0.6, "top"), (cx, y1 + 2.4, "top")),
            "split": ((cx, y0 - 0.6, "bottom"), (cx, y1 + 0.6, "top")),
            "aboveleft": ((cx - 1.6, y0 - 2.4, "right bottom"), (cx - 1.6, y0 - 0.6, "right bottom")),
            "belowright": ((x1, y1 + 0.6, "right top"), (x1, y1 + 2.4, "right top")),
        }[mode]
        sym = ["symbol", ["lib_id", q(part["sym"])], ["at", str(round(p.x, 3)), str(round(p.y, 3)), str(p.rot)]]
        if p.mirror:
            sym.append(["mirror", "y"])
        sym += [["unit", str(p.unit)], ["exclude_from_sim", "no"],
                ["in_bom", "no" if part.get("nobom") else "yes"], ["on_board", "yes"], ["dnp", "no"],
                ["uuid", uid()]]
        props = [("Reference", part["ref"], False), ("Value", part["value"], False),
                 ("Footprint", part.get("fp", ""), True), ("Datasheet", part.get("ds", ""), True),
                 ("Description", part.get("desc", ""), True)]
        props += [(k, v, True) for k, v in part.get("fields", {}).items()]
        # Fields are written centred, at absolute positions: KiCad turns a field's angle and
        # justification with its symbol, so centred text with the angle undone stays upright.
        fang = "90" if p.rot % 180 == 90 else "0"
        for k, v, hide in props:
            if k in ("Reference", "Value") and not hide:
                px, py, just = spots[0 if k == "Reference" else 1]
                wd = 0.8 * 1.27 * len(v)
                cx_ = px + wd / 2 if "left" in just else px - wd / 2 if "right" in just else px
                cy_ = py - 0.65 if "bottom" in just else py + 0.65 if "top" in just else py
                sym.append(["property", q(k), q(v), ["at", str(round(cx_, 3)), str(round(cy_, 3)), fang],
                            *fnt(1.27)])
                self.grid.block_rect(cx_ - wd / 2, cy_ - 0.65, cx_ + wd / 2, cy_ + 0.65)
            else:
                sym.append(["property", q(k), q(v), ["at", str(round(p.x, 3)), str(round(p.y, 3)), "0"],
                            *fnt(1.27, None, True)])
        for num in p.pins:
            sym.append(["pin", q(num), ["uuid", uid()]])
        sym.append(["instances", ["project", q(self.project),
                                  ["path", q(self.path), ["reference", q(part["ref"])],
                                   ["unit", str(p.unit)]]]])
        self.items.append(sym)

    def sheet_symbol(self, b):
        node = ["sheet", ["at", str(round(b.x, 3)), str(round(b.y, 3))],
                ["size", str(round(b.w, 3)), str(round(b.h, 3))],
                ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"],
                ["fields_autoplaced", "yes"], ["stroke", ["width", "0.1524"], ["type", "solid"]],
                ["fill", ["color", "0", "0", "0", "0.0000"]], ["uuid", q(b.sheet.uuid)],
                ["property", q("Sheetname"), q(b.sheet.title), ["at", str(round(b.x, 3)), str(round(b.y - 0.7, 3)), "0"],
                 ["effects", ["font", ["size", "1.27", "1.27"], ["thickness", "0.254"], ["bold", "yes"]],
                  ["justify", "left", "bottom"]]],
                ["property", q("Sheetfile"), q(b.sheet.file),
                 ["at", str(round(b.x, 3)), str(round(b.y + b.h + 0.6, 3)), "0"], *fnt(1.27, "left top")]]
        for net, pin in b.pins.items():
            ang, just = (180, "left") if pin["out"] == "L" else (0, "right")
            node.append(["pin", q(net), b.shapes[net],
                         ["at", str(mm(pin["cell"][0])), str(mm(pin["cell"][1])), str(ang)],
                         ["uuid", uid()], *fnt(1.27, just)])
        node.append(["instances", ["project", q(self.project),
                                   ["path", q(self.path), ["page", q(str(b.page))]]]])
        self.items.append(node)
        # the pin names inside the box
        self.grid.block_rect(b.x, b.y - 2.5, b.x + b.w, b.y + b.h + 2.5)

    def power_symbol(self, net, cell, rot):
        lib_id = POWER_NETS[net]
        self.lib_sym(lib_id)
        self.n["pwr"] += 1
        self.items.append(power_sym(lib_id, f"#PWR{self.n['pwr']:03d}", net, mm(cell[0]), mm(cell[1]),
                                    rot, self.project, self.path))

    def power_pin(self, net, cell, out):
        """A power pin's own symbol: upright, on a stub (with a dog-leg for pins facing sideways)."""
        gr = self.grid
        up = net != "GND"
        vert = "U" if up else "D"
        if out in ("U", "D"):
            end = step(cell, out, 2)
            legs = [step(cell, out, 1), end]
            rot = 0 if (out == vert) else 180
            body = [step(end, out, k) for k in (1, 2, 3)]
            body += [(b[0] + s, b[1]) for b in body for s in (-1, 1)]
            plan = [(cell, end)]
            if out != vert:  # symbol upside down: GND on a pin facing up, rare
                pass
        else:
            elbow = step(cell, out, 2)
            end = step(elbow, vert, 2)
            legs = [step(cell, out, 1), elbow, step(elbow, vert, 1), end]
            rot = 0
            body = [step(end, vert, k) for k in (1, 2, 3)]
            body += [(b[0] + s, b[1]) for b in body for s in (-1, 1)]
            plan = [(cell, elbow), (elbow, end)]
        if any(c in gr.hard or c in gr.owner or c in gr.pinpts or c in self.lanes for c in legs + body):
            # no room for the dog-leg: a short stub and the symbol turned to face out
            end = step(cell, out, 2)
            legs = [step(cell, out, 1), end]
            rot = ({"D": 0, "U": 180, "R": 90, "L": 270} if not up else
                   {"U": 0, "D": 180, "R": 270, "L": 90})[out]
            body = [step(end, out, k) for k in (1, 2, 3)]
            plan = [(cell, end)]
        for a, b in plan:
            self.wire(a, b)
        for c in legs + body:
            gr.hard.add(c)
        self.power_symbol(net, end, rot)

    # -- build
    def build(self):
        s, gr = self.s, self.grid
        w, h = self.w, self.h
        gr.block_rect(w - 112, h - 36, w, h)  # title block
        gr.block_rect(0, 0, w, 6)
        gr.block_rect(0, h - 6, w, h)
        gr.block_rect(0, 0, 6, h)
        gr.block_rect(w - 6, 0, w, h)
        for p in s.placed:
            x0, y0, x1, y1 = p.box
            gr.block_rect(x0 + 0.05, y0 + 0.05, x1 - 0.05, y1 - 0.05)
            gr.soft_rect(x0 - G, y0 - G, x1 + G, y1 + G, 1.5)
            for pin in p.pins.values():
                dx, dy = DIRS[OPP[pin["out"]]]
                for k in range(1, int(round(pin["len"] / G)) + 1):
                    gr.hard.add((pin["cell"][0] + dx * k, pin["cell"][1] + dy * k))
        for x, y, t, size in s.texts:
            lines = t.split("\n")
            wd = max(len(l) for l in lines) * size * 0.62
            gr.block_rect(x - 0.5, y - size - 0.5, x + wd, y + size * 1.55 * (len(lines) - 1) + 0.8)
            self.items.append(["text", '"' + t.replace('"', '\\"').replace("\n", "\\n") + '"',
                               ["exclude_from_sim", "no"],
                               ["at", str(round(x, 3)), str(round(y, 3)), "0"],
                               *fnt(size, "left bottom"), ["uuid", uid()]])
        for p in s.placed:
            self.symbol(p)
        # whatever the text, every pin can get out: its first two cells stay free
        for p in s.placed:
            for pin in p.pins.values():
                for k in (1, 2):
                    gr.hard.discard(step(pin["cell"], pin["out"], k))

        nets = s.nets()
        railed = {}
        for r in s.rails:
            for p, num in nets.get(r["net"], []):
                if r["refs"] is None or p.part["ref"] in r["refs"]:
                    railed[(id(p), num)] = r["net"]
        for p in s.placed:
            pins = p.nets
            for num, pin in p.pins.items():
                if num in pins:
                    gr.pinpts[pin["cell"]] = pins[num]
                    gr.owner[pin["cell"]] = pins[num]
                elif not p.is_sheet:
                    self.items.append(["no_connect", ["at", str(mm(pin["cell"][0])), str(mm(pin["cell"][1]))],
                                       ["uuid", uid()]])
                    gr.hard.add(pin["cell"])
                    gr.hard.update(step(pin["cell"], pin["out"], k) for k in (1, 2))
        # rail symbols
        rail_terms = {}
        for r in s.rails:
            cell, net = r["cell"], r["net"]
            up = net != "GND"
            self.power_symbol(net, cell, 0)
            gr.pinpts[cell] = net
            gr.owner[cell] = net
            for k in (1, 2, 3, 4):
                for sd in (-1, 0, 1):
                    gr.hard.add((cell[0] + sd, cell[1] + (-k if up else k)))
            rail_terms.setdefault(net, []).append((cell, "D" if up else "U"))
        flag_terms = {}
        for net, x, y in s.flags:
            self.n["flg"] += 1
            self.lib_sym("power:PWR_FLAG")
            self.items.append(pwr_flag(f"#FLG{self.n['flg']:03d}", x, y, self.project, self.path))
            a, b = (cl(x), cl(y)), (cl(x), cl(y) + 2)
            self.wire(a, b)
            if net in POWER_NETS:
                self.power_symbol(net, b, 0)
                gr.block_rect(x - 2.5, y - 4, x + 2.5, y + 5)
            else:  # a flag on a signal net: wired to the net like a pin
                gr.block_rect(x - 2.5, y - 4, x + 2.5, y + 1.3)
                gr.pinpts[b] = net
                gr.owner[b] = net
                flag_terms.setdefault(net, []).append((b, "D"))
        # every signal pin's way out (4 cells) is kept clear of power symbols
        self.lanes = set()
        for p in s.placed:
            for num, net in p.nets.items():
                if net not in POWER_NETS or (id(p), num) in railed:
                    pin = p.pins[num]
                    self.lanes.update(step(pin["cell"], pin["out"], k) for k in range(1, 5))
        # power pins that are not on a rail (stacked pins share one stub and symbol)
        done = set()
        for p in s.placed:
            for num, net in p.nets.items():
                if net in POWER_NETS and (id(p), num) not in railed and p.pins[num]["cell"] not in done:
                    done.add(p.pins[num]["cell"])
                    self.power_pin(net, p.pins[num]["cell"], p.pins[num]["out"])
        # ports
        port_terms = {}
        rows_used = {"L": set(), "R": set()}
        for net in sorted(getattr(s, "ports", {})):
            side = s.port_side.get(net, s.default_side)
            if net in s.port_row:
                row = cl(s.port_row[net] * U)
            else:
                ys = [p.pins[num]["cell"][1] for p, num in nets.get(net, [])]
                row = min(ys) if ys else cl(h / 2)  # level with the topmost pin of the net
            while row in rows_used[side] or row + 1 in rows_used[side] or row - 1 in rows_used[side]:
                row += 2
            rows_used[side].add(row)
            x = getattr(s, "port_x", {}).get(side, 12 * U if side == "L" else w - 12 * U)
            cell = (cl(x), row)
            out = "R" if side == "L" else "L"
            shape = s.shapes.get(net, "passive")
            ang, just = (180, "right") if side == "L" else (0, "left")
            self.items.append(["hierarchical_label", q(net), ["shape", shape],
                               ["at", str(mm(cell[0])), str(mm(cell[1])), str(ang)],
                               *fnt(1.27, just), ["uuid", uid()]])
            gr.pinpts[cell] = net
            gr.owner[cell] = net
            for k in range(1, int(len(net) * 0.85) + 4):
                for j in (-1, 0, 1):
                    gr.hard.add((cell[0] - k if side == "L" else cell[0] + k, cell[1] + j))
            port_terms[net] = (cell, out)

        # route, shortest nets first
        routed = [n for n in nets if n not in POWER_NETS] + [r["net"] for r in s.rails] + list(flag_terms)
        routed = list(dict.fromkeys(routed))

        def pieces_of(net):
            """[(terminals, has_port)] for each separately drawn piece of a net."""
            allt = terms_of(net)
            if net not in s.splits:
                return [(allt, net in port_terms)]
            cell_of = {(p.part["ref"], num): p.pins[num]["cell"] for p, num in nets.get(net, [])}
            out, used = [], set()
            for pc in s.splits[net]:
                cells = {cell_of[rp] for rp in pc}
                used |= cells
                out.append(([t for t in allt if t[0] in cells], False))
            rest = [t for t in allt if t[0] not in used]
            port_cell = port_terms[net][0] if net in port_terms else None
            out.append((rest, port_cell in [c for c, _ in rest]))
            return out

        def terms_of(net):
            t = []
            if net in port_terms:
                t.append(port_terms[net])
            t += rail_terms.get(net, []) + flag_terms.get(net, [])
            for p, num in nets.get(net, []):
                if net in POWER_NETS and (id(p), num) not in railed:
                    continue
                pin = (p.pins[num]["cell"], p.pins[num]["out"])
                if pin[0] not in [c for c, _ in t]:  # stacked pins: one terminal
                    t.append(pin)
            return t

        def spread(net):
            cs = [t[0] for t in terms_of(net)]
            if not cs:
                return 0
            return (max(a for a, _ in cs) - min(a for a, _ in cs)) + (max(b for _, b in cs) - min(b for _, b in cs))

        to_name = []
        for net in sorted(routed, key=lambda n: (spread(n), n)):
            for terms, has_port in pieces_of(net):
                cells = self.route_net(net, terms)
                if net not in POWER_NETS and not has_port and cells:
                    to_name.append((net, cells))
        # name every piece that no port names
        for net, cells in to_name:
            self.name_net(net, cells)
        for net, cells in self.net_cells.items():
            self.emit(net, cells)
        return self

    def route_net(self, net, terms):
        gr = self.grid
        if len(terms) < 2:
            if terms and net not in POWER_NETS:
                self.unrouted.append((net, "only one terminal on this sheet"))
            return set()
        first = terms[0]
        cells = {first[0]}
        targets = {first[0]: {OPP[first[1]]}}
        rest = list(terms[1:])
        while rest:
            rest.sort(key=lambda t: min(abs(t[0][0] - a) + abs(t[0][1] - b) for a, b in targets))
            cell, out = rest.pop(0)
            col = self.s.columns.get(net)
            path = gr.route(net, cell, out, targets, column=None if col is None else cl(col * U))
            if path is None:
                self.unrouted.append((net, f"pin at ({mm(cell[0])}, {mm(cell[1])}) mm"))
                continue
            for a, b in zip(path, path[1:]):
                gr.link(a, b, net)
            cells.update(path)
            targets = {c: None for c in cells
                       if c not in gr.pinpts and c not in gr.crossed and gr.owner.get(c) == net}
            # a terminal not yet wired (only the first, until something reaches it)
            for c, o in [(first[0], first[1])]:
                if not gr.own_dirs(c, net):
                    targets[c] = {OPP[o]}
        self.net_cells.setdefault(net, set()).update(cells)
        return cells

    def name_net(self, net, cells):
        """A local label where its text has room: on a horizontal run (text above it), else on a
        vertical run (text beside it). With no room the net keeps KiCad's automatic name; the
        drawing stays readable and the connection is the same."""
        gr = self.grid
        need = int(len(net) * 0.85) + 2

        def free(c):
            return c not in gr.hard and c not in gr.owner and c not in gr.pinpts

        best = None
        for c in cells:
            if c in gr.crossed or c in gr.pinpts and not gr.own_dirs(c, net) & {"R"}:
                continue
            if "R" not in gr.own_dirs(c, net) or "L" in gr.own_dirs(c, net) and step(c, "L") in cells:
                continue  # start of a run going right
            n, e = 0, c
            while "R" in gr.own_dirs(e, net) and step(e, "R") in cells:
                e = step(e, "R")
                n += 1
            if n < 3:
                continue
            # start just past a pin, bend or junction, so the text clears the wire leaving it
            at = step(c, "R") if (c in gr.pinpts or gr.own_dirs(c, net) - {"L", "R"}) else c
            if at in gr.pinpts or at in gr.crossed:
                continue
            if all(free((at[0] + k, at[1] - 1)) for k in range(0, need + 1)):
                if best is None or n > best[0]:
                    best = (n, at, "0")
        if best is None:  # text running left from the end of a horizontal run
            for c in cells:
                if c in gr.crossed or "L" not in gr.own_dirs(c, net) or step(c, "L") not in cells:
                    continue
                at = step(c, "L") if c in gr.pinpts else c
                if at in gr.crossed or "L" not in gr.own_dirs(at, net):
                    continue
                if all(free((at[0] - k, at[1] - 1)) for k in range(0, need + 1)):
                    best = (0, at, "180")
                    break
        if best is None:
            for c in cells:
                if c in gr.crossed or "D" not in gr.own_dirs(c, net) or "U" in gr.own_dirs(c, net) and step(c, "U") in cells:
                    continue
                n, e = 0, c
                while "D" in gr.own_dirs(e, net) and step(e, "D") in cells:
                    e = step(e, "D")
                    n += 1
                if n < need:
                    continue
                at = step(e, "U") if e in gr.pinpts else e  # bottom of the run; text reads upwards
                if at in gr.crossed:
                    continue
                if all(free((at[0] - j, at[1] - k)) for k in range(0, need + 1) for j in (1, 2)):
                    if best is None or n > best[0]:
                        best = (n, at, "90")
        if best is None:
            return
        _, at, ang = best
        self.breaks.add(at)
        just = "right bottom" if ang == "180" else "left bottom"
        self.items.append(["label", q(net), ["at", str(mm(at[0])), str(mm(at[1])), "0" if ang == "180" else ang],
                           *fnt(1.27, just), ["uuid", uid()]])
        for k in range(0, need + 1):
            gr.hard.add({"0": (at[0] + k, at[1] - 1), "180": (at[0] - k, at[1] - 1),
                         "90": (at[0] - 1, at[1] - k)}[ang])

    def emit(self, net, cells):
        gr = self.grid
        edges = set()
        for a in cells:
            for d in gr.own_dirs(a, net):
                b = step(a, d)
                if b in cells:
                    edges.add(frozenset((a, b)))

        def deg(c):
            return sum(1 for d in gr.own_dirs(c, net) if step(c, d) in cells)

        def is_break(c):
            ds = {d for d in gr.own_dirs(c, net) if step(c, d) in cells}
            return (c in gr.pinpts or c in self.breaks or len(ds) != 2
                    or ds not in ({"L", "R"}, {"U", "D"}))

        used = set()
        for e in list(edges):
            if e in used:
                continue
            a, b = tuple(e)
            d = vdir(b[0] - a[0], b[1] - a[1])
            s0 = a
            while not is_break(s0) and frozenset((s0, step(s0, OPP[d]))) in edges:
                s0 = step(s0, OPP[d])
            e0 = s0
            while frozenset((e0, step(e0, d))) in edges:
                used.add(frozenset((e0, step(e0, d))))
                e0 = step(e0, d)
                if is_break(e0):
                    break
            self.wire(s0, e0)
        for c in cells:
            if deg(c) + (1 if c in gr.pinpts else 0) >= 3:
                self.items.append(["junction", ["at", str(mm(c[0])), str(mm(c[1]))], ["diameter", "0"],
                                   ["color", "0", "0", "0", "0"], ["uuid", uid()]])


def power_sym(lib_id, ref, net, x, y, rot, project, path):
    # the rail's name beyond the arrow's tip, upright (KiCad turns fields with the symbol)
    dx, dy = {0: (0, -1), 180: (0, 1), 90: (-1, 0), 270: (1, 0)}[rot % 360]
    reach = 3.6 + (0.45 * len(net) if dx else 0)
    vx, vy = round(x + dx * reach, 3), round(y + dy * reach, 3)
    show = net != "GND"
    return ["symbol", ["lib_id", q(lib_id)], ["at", str(x), str(y), str(rot)],
            ["unit", "1"], ["exclude_from_sim", "no"], ["in_bom", "yes"],
            ["on_board", "yes"], ["dnp", "no"], ["uuid", uid()],
            ["property", q("Reference"), q(ref), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["property", q("Value"), q(net), ["at", str(vx), str(vy), "90" if rot % 180 == 90 else "0"],
             *fnt(1.27, None, not show)],
            ["property", q("Footprint"), q(""), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["property", q("Datasheet"), q(""), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["pin", q("1"), ["uuid", uid()]],
            ["instances", ["project", q(project), ["path", q(path), ["reference", q(ref)],
                                                   ["unit", "1"]]]]]


def pwr_flag(ref, x, y, project, path):
    return ["symbol", ["lib_id", q("power:PWR_FLAG")], ["at", str(x), str(y), "0"],
            ["unit", "1"], ["exclude_from_sim", "no"], ["in_bom", "yes"],
            ["on_board", "yes"], ["dnp", "no"], ["uuid", uid()],
            ["property", q("Reference"), q(ref), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["property", q("Value"), q("PWR_FLAG"), ["at", str(x), str(round(y - 3, 2)), "0"], *fnt(1.0)],
            ["property", q("Footprint"), q(""), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["property", q("Datasheet"), q(""), ["at", str(x), str(y), "0"], *fnt(1.27, None, True)],
            ["pin", q("1"), ["uuid", uid()]],
            ["instances", ["project", q(project), ["path", q(path), ["reference", q(ref)],
                                                   ["unit", "1"]]]]]


# --- The project -------------------------------------------------------------------------------------
def sch_file(uuid, paper, title, page_title, date, comment, used, items, root=False):
    tree = ["kicad_sch", ["version", "20250610"], ["generator", q("eeschema")],
            ["generator_version", q("10.0")], ["uuid", q(uuid)], ["paper", q(paper)],
            ["title_block", ["title", q(page_title)], ["date", q(date)], ["rev", q("1")],
             ["company", q(COMPANY)], ["comment", "1", q(title)], ["comment", "2", q(comment)]],
            ["lib_symbols"] + list(used.values())] + items
    if root:
        tree.append(["sheet_instances", ["path", q("/"), ["page", q("1")]]])
    tree.append(["embedded_fonts", "no"])
    return dump(tree) + "\n"


def build_project(lib, project, title, sheets, root, outdir, date, comment):
    """sheets: [Sheet]; root: a Sheet-like description of the block diagram, with .boxes
    [(sheet, x, y, w, [(net, side)])], .texts, .paper. Writes <outdir>/<project>.kicad_sch and
    one file per sheet. Returns the unrouted list (empty when everything was drawn)."""
    root_uuid = str(uuidlib.uuid4())
    counters = {"pwr": 0, "flg": 0}
    problems = []
    # which nets leave which sheet
    where = {}
    for s in sheets:
        for net in s.nets():
            if net not in POWER_NETS:
                where.setdefault(net, set()).add(s.name)
    for s in sheets:
        s.ports = {n for n in s.nets() if n not in POWER_NETS and len(where[n]) > 1}
    seen = {}
    for s in sheets:
        for p in s.placed:
            if p.part["ref"] in seen:
                problems.append(("placement", f"{p.part['ref']} placed on {seen[p.part['ref']]} and {s.name}"))
            seen[p.part["ref"]] = s.name
    for page, s in enumerate(sheets, start=2):
        wr = Writer(s, project, f"/{root_uuid}/{s.uuid}", counters).build()
        problems += [(s.name, f"{n}: {why}") for n, why in wr.unrouted]
        (outdir / s.file).write_text(sch_file(s.file_uuid, s.paper, title, s.title, date, comment,
                                              wr.used, wr.items), encoding="utf-8")
        s.page = page
    # root: the block diagram
    rs = Sheet(lib, {}, "root", title, root.paper)
    rs.texts = [(x * U, y * U, t, size) for x, y, t, size in root.texts]
    for box in root.boxes:
        s, x, y, wdt, pins = box[:5]
        # a row is None (gap), one (net, side), or a list of them
        rows = [[] if r is None else [r] if isinstance(r, tuple) else list(r) for r in pins]
        named = {n for r in rows for n, _ in r}
        missing, extra = s.ports - named, named - s.ports
        if missing or extra:
            problems.append(("root", f"{s.name}: sheet pins missing {sorted(missing)} extra {sorted(extra)}"))
        rs.placed.append(SheetBox(s, x * U, y * U, wdt * U,
                                  [[(n, side, s.shapes.get(n, "passive")) for n, side in r] for r in rows],
                                  s.page, box[5] if len(box) > 5 else 0))
    wr = Writer(rs, project, f"/{root_uuid}", counters).build()
    problems += [("root", f"{n}: {why}") for n, why in wr.unrouted]
    (outdir / f"{project}.kicad_sch").write_text(
        sch_file(root_uuid, root.paper, title, title, date, comment, wr.used, wr.items, root=True),
        encoding="utf-8")
    return problems
