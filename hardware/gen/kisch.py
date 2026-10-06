"""kisch — a small KiCad 10 schematic writer.

The boards are described in Python (hardware/gen/pod_board.py, power_board.py): each part names its
KiCad library symbol, footprint and value, and maps PIN NUMBERS to net names. This module turns
that into a .kicad_sch that KiCad opens normally:
  - the library symbols are copied from KiCad's own libraries (so pins and graphics are exact);
  - parts are placed on a grid, in the groups the description gives;
  - every connected pin gets a short wire stub and a net label at its end, every power pin a power
    symbol, every unused pin a no-connect flag.
Connections are therefore made by NAME, the way large schematics are drawn: the schematic reads
as one block per function, and the netlist is what kicad-cli exports and check_nets.py verifies.

Why generate rather than draw: the pin assignments already live in firmware/.../Config.h and the
cable pinout in BUILD_GUIDE 4.8.3. Generating the schematic from one description, then checking
the exported netlist against both, makes "the schematic disagrees with the firmware" a failed check
instead of a board respin. Once generated, the files are ordinary KiCad files: open them, tidy the
drawing, lay out the PCB.
"""

import re
import uuid as uuidlib
from pathlib import Path

GRID = 2.54
POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V", "+12V": "power:+12V",
              "VBUS": "power:VBUS"}


# --- S-expression reading ----------------------------------------------------------------------
def tokenize(s):
    return re.findall(r'"(?:\\.|[^"\\])*"|\(|\)|[^\s()]+', s)


def parse(s):
    toks = tokenize(s)
    stack = [[]]
    for t in toks:
        if t == "(":
            stack.append([])
        elif t == ")":
            x = stack.pop()
            stack[-1].append(x)
        else:
            stack[-1].append(t)
    return stack[0][0]


def dump(x, ind=0):
    if not isinstance(x, list):
        return x
    if all(not isinstance(e, list) for e in x):
        return "(" + " ".join(x) + ")"
    pad = "\t" * (ind + 1)
    head = []
    rest = []
    for e in x:
        (rest if isinstance(e, list) or rest else head).append(e)
    out = "(" + " ".join(head)
    for e in rest:
        out += "\n" + pad + dump(e, ind + 1)
    return out + "\n" + "\t" * ind + ")"


def q(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def unq(s):
    return s[1:-1].replace('\\"', '"') if s.startswith('"') else s


def find(node, key):
    return [e for e in node if isinstance(e, list) and e and e[0] == key]


# --- Library symbols ---------------------------------------------------------------------------
class Library:
    def __init__(self, sym_dir):
        self.dir = Path(sym_dir)
        self.cache = {}

    def _lib(self, name):
        if name not in self.cache:
            tree = parse((self.dir / f"{name}.kicad_sym").read_text(encoding="utf-8"))
            self.cache[name] = {unq(s[1]): s for s in find(tree, "symbol")}
        return self.cache[name]

    def symbol(self, lib_id):
        """The symbol, flattened if it 'extends' another, renamed to the lib_id."""
        libname, name = lib_id.split(":", 1)
        lib = self._lib(libname)
        sym = [list(e) if isinstance(e, list) else e for e in lib[name]]
        ext = find(sym, "extends")
        if ext:
            parent = self.symbol(f"{libname}:{unq(ext[0][1])}")
            props = {unq(p[1]): p for p in find(sym, "property")}
            body = [e for e in parent[2:] if not (isinstance(e, list) and e[0] in ("property",))]
            pprops = [p for p in find(parent, "property") if unq(p[1]) not in props]
            # sub-units are named after the symbol: rename the parent's to this one
            pname = unq(parent[1]).split(":", 1)[-1]
            def rename(e):
                if isinstance(e, list) and e and e[0] == "symbol" and unq(e[1]).startswith(pname + "_"):
                    return ["symbol", q(name + unq(e[1])[len(pname):])] + e[2:]
                return e
            other = [e for e in sym[2:] if isinstance(e, list) and e[0] not in ("extends", "property")]
            sym = ["symbol", q(name)] + other + list(props.values()) + pprops + [rename(e) for e in body]
        out = ["symbol", q(lib_id)] + sym[2:]
        return out

    def pins(self, lib_id, unit=1):
        """{number: (x, y, angle, length, name, etype)} in symbol coordinates (y up)."""
        sym = self.symbol(lib_id)
        res = {}
        for sub in find(sym, "symbol"):
            m = re.search(r"_(\d+)_(\d+)$", unq(sub[1]))
            u = int(m.group(1)) if m else 0
            if u not in (0, unit):
                continue
            for p in find(sub, "pin"):
                at = find(p, "at")[0]
                length = float(find(p, "length")[0][1])
                name = unq(find(p, "name")[0][1])
                num = unq(find(p, "number")[0][1])
                res[num] = (float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0,
                            length, name, p[1])
        return res


# --- Schematic ---------------------------------------------------------------------------------
def uid():
    return q(str(uuidlib.uuid4()))


def fnt(size=1.27, justify=None, hide=False):
    e = ["effects", ["font", ["size", str(size), str(size)]]]
    if justify:
        e.append(["justify"] + justify.split())
    out = [e]
    if hide:
        out.insert(0, ["hide", "yes"])
    return out


class Schematic:
    def __init__(self, lib, title, project, rev="1", date="", comment=""):
        self.lib = lib
        self.title = title
        self.project = project
        self.root = str(uuidlib.uuid4())
        self.items = []
        self.used = {}
        self.pwr = 0
        self.rev, self.date, self.comment = rev, date, comment

    def _snap(self, v):
        return round(round(v / GRID) * GRID, 2)

    def text(self, x, y, s, size=2.0):
        self.items.append(["text", q(s), ["exclude_from_sim", "no"], ["at", str(x), str(y), "0"],
                           *fnt(size, "left bottom"), ["uuid", uid()]])

    def place(self, part, x, y):
        """Place a part; connect its pins by name. Returns the part's bounding height (mm)."""
        lib_id = part["sym"]
        if lib_id not in self.used:
            self.used[lib_id] = self.lib.symbol(lib_id)
        x, y = self._snap(x), self._snap(y)
        pins = self.lib.pins(lib_id, part.get("unit", 1))
        props = [
            ("Reference", part["ref"], False), ("Value", part["value"], False),
            ("Footprint", part.get("fp", ""), True), ("Datasheet", part.get("ds", ""), True),
            ("Description", part.get("desc", ""), True),
        ] + [(k, v, True) for k, v in part.get("fields", {}).items()]
        ys = [p[1] for p in pins.values()] or [0]
        xs = [p[0] for p in pins.values()] or [0]
        top, bot = max(ys), min(ys)
        narrow = max(abs(v) for v in xs) < 1  # two-pin vertical parts: text beside, not above
        sym = ["symbol", ["lib_id", q(lib_id)], ["at", str(x), str(y), "0"],
               ["unit", str(part.get("unit", 1))], ["exclude_from_sim", "no"],
               ["in_bom", "no" if part.get("nobom") else "yes"], ["on_board", "yes"], ["dnp", "no"],
               ["uuid", uid()]]
        for i, (k, v, hide) in enumerate(props):
            if k in ("Reference", "Value") and narrow:
                px, py, just = x + 2.54, y + (-1.27 if k == "Reference" else 1.27), "left"
            elif k in ("Reference", "Value"):
                px, py, just = x, y - top - 3.81 - (1.9 if k == "Value" else 0), None
            else:
                px, py, just = x, y, None
            sym.append(["property", q(k), q(v), ["at", str(round(px, 2)), str(round(py, 2)), "0"],
                        *fnt(1.27, just, hide)])
        for num in pins:
            sym.append(["pin", q(num), ["uuid", uid()]])
        sym.append(["instances", ["project", q(self.project),
                                  ["path", q("/" + self.root), ["reference", q(part["ref"])],
                                   ["unit", str(part.get("unit", 1))]]]])
        self.items.append(sym)

        nets = part.get("nets", {})
        unknown = [n for n in nets if n not in pins]
        if unknown:
            raise KeyError(f"{part['ref']} ({lib_id}): no pin(s) {unknown}; has {sorted(pins)}")
        for num, (px, py, ang, length, name, etype) in pins.items():
            cx, cy = round(x + px, 2), round(y - py, 2)  # connection point, schematic coords (y down)
            # pin angle: direction from connection point towards the body; the stub goes away from it
            dx, dy = {0: (-1, 0), 90: (0, 1), 180: (1, 0), 270: (0, -1)}[int(ang) % 360]
            net = nets.get(num)
            if net is None:
                if etype in ("power_in",) and part.get("strict", True):
                    raise ValueError(f"{part['ref']} pin {num} ({name}) is a power pin with no net")
                self.items.append(["no_connect", ["at", str(cx), str(cy)], ["uuid", uid()]])
                continue
            ex, ey = round(cx + dx * 2.54, 2), round(cy + dy * 2.54, 2)
            self.items.append(["wire", ["pts", ["xy", str(cx), str(cy)], ["xy", str(ex), str(ey)]],
                               ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid()]])
            if net in POWER_NETS:
                self._power(net, ex, ey, dx, dy)
            else:
                ang_l = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[(dx, dy)]
                just = "right bottom" if ang_l == 180 else "left bottom"
                if ang_l == 270:
                    just = "right bottom"
                self.items.append(["label", q(net), ["at", str(ex), str(ey), str(ang_l)],
                                   *fnt(1.27, just), ["uuid", uid()]])
        return top - bot

    def _power(self, net, x, y, dx, dy):
        lib_id = POWER_NETS[net]
        if lib_id not in self.used:
            self.used[lib_id] = self.lib.symbol(lib_id)
        self.pwr += 1
        ref = f"#PWR{self.pwr:03d}"
        # GND symbols point down, supply symbols up; rotate so the symbol sits beyond the stub.
        rot = 0
        if net == "GND":
            rot = {(0, 1): 0, (0, -1): 180, (1, 0): 90, (-1, 0): 270}[(dx, dy)]
        else:
            rot = {(0, -1): 0, (0, 1): 180, (1, 0): 270, (-1, 0): 90}[(dx, dy)]
        self.items.append(["symbol", ["lib_id", q(lib_id)], ["at", str(x), str(y), str(rot)],
                           ["unit", "1"], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                           ["on_board", "yes"], ["dnp", "no"], ["uuid", uid()],
                           ["property", q("Reference"), q(ref), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["property", q("Value"), q(net), ["at", str(x), str(y), "0"],
                            *fnt(1.0, None, True)],
                           ["property", q("Footprint"), q(""), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["property", q("Datasheet"), q(""), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["pin", q("1"), ["uuid", uid()]],
                           ["instances", ["project", q(self.project),
                                          ["path", q("/" + self.root), ["reference", q(ref)],
                                           ["unit", "1"]]]]])

    def power_flag(self, net, x, y):
        """PWR_FLAG on a net driven from off the board (a connector or a regulator ERC can't see)."""
        lib_id = "power:PWR_FLAG"
        if lib_id not in self.used:
            self.used[lib_id] = self.lib.symbol(lib_id)
        self.pwr += 1
        ref = f"#FLG{self.pwr:03d}"
        x, y = self._snap(x), self._snap(y)
        self.items.append(["symbol", ["lib_id", q(lib_id)], ["at", str(x), str(y), "0"],
                           ["unit", "1"], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                           ["on_board", "yes"], ["dnp", "no"], ["uuid", uid()],
                           ["property", q("Reference"), q(ref), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["property", q("Value"), q("PWR_FLAG"), ["at", str(x), str(y - 3), "0"],
                            *fnt(1.0)],
                           ["property", q("Footprint"), q(""), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["property", q("Datasheet"), q(""), ["at", str(x), str(y), "0"],
                            *fnt(1.27, None, True)],
                           ["pin", q("1"), ["uuid", uid()]],
                           ["instances", ["project", q(self.project),
                                          ["path", q("/" + self.root), ["reference", q(ref)],
                                           ["unit", "1"]]]]])
        # its pin (at 0,0) gets a short stub down to a label or power symbol
        self.items.append(["wire", ["pts", ["xy", str(x), str(y)], ["xy", str(x), str(round(y + 2.54, 2))]],
                           ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid()]])
        if net in POWER_NETS:
            self._power(net, x, round(y + 2.54, 2), 0, 1)
        else:
            self.items.append(["label", q(net), ["at", str(x), str(round(y + 2.54, 2)), "270"],
                               *fnt(1.27, "right bottom"), ["uuid", uid()]])

    def write(self, path, paper="A2"):
        tree = ["kicad_sch", ["version", "20250610"], ["generator", q("eeschema")],
                ["generator_version", q("10.0")], ["uuid", q(self.root)], ["paper", q(paper)],
                ["title_block", ["title", q(self.title)], ["date", q(self.date)], ["rev", q(self.rev)],
                 ["company", q("AR Windscreen FYP - Kiogora Ian Mwenda")],
                 ["comment", "1", q(self.comment)]],
                ["lib_symbols"] + list(self.used.values())] + self.items + [
                ["sheet_instances", ["path", q("/"), ["page", q("1")]]],
                ["embedded_fonts", "no"]]
        Path(path).write_text(dump(tree) + "\n", encoding="utf-8")


# --- Layout ------------------------------------------------------------------------------------
def footprint_of(lib, part):
    """(width, height) in mm a placed part needs, with room for its labels."""
    pins = lib.pins(part["sym"], part.get("unit", 1))
    if not pins:
        return 12.7, 10.16
    xs = [p[0] for p in pins.values()]
    ys = [p[1] for p in pins.values()]
    labelled = max((len(n) for n in part.get("nets", {}).values()), default=0)
    lab = 2.54 + 1.0 * labelled + 1.5  # stub + label text (~1 mm per character at 1.27 mm)
    left = any(p[0] < -0.5 for p in pins.values())
    right = any(p[0] > 0.5 for p in pins.values())
    w = (max(xs) - min(xs)) + (lab if left else 0) + (lab if right else 0) + 12
    vert = all(abs(p[0]) < 0.5 for p in pins.values())
    if vert:
        w = 22
    h = (max(ys) - min(ys)) + 10
    if vert:
        h += 2 * lab
    return max(w, 15.24), max(h, 12.7)


def layout(sch, groups, x0=20.0, y0=25.0, col_w=None, page_h=400.0, gap=12.0):
    """Place groups top-to-bottom in columns. groups: [(title, [part, ...], ncols)]."""
    x, y = x0, y0
    col_right = x0
    for title, parts, ncols in groups:
        sizes = [footprint_of(sch.lib, p) for p in parts]
        cell_w = max(s[0] for s in sizes)
        rows = [sizes[i:i + ncols] for i in range(0, len(sizes), ncols)]
        g_h = 8 + sum(max(s[1] for s in r) for r in rows)
        g_w = cell_w * min(ncols, len(parts))
        if y + g_h > page_h and y > y0:
            x, y = col_right + gap, y0
        sch.text(x, y, title, 2.5)
        yy = y + 8
        for ri in range(len(rows)):
            row_parts = parts[ri * ncols:(ri + 1) * ncols]
            rh = max(s[1] for s in rows[ri])
            for ci, p in enumerate(row_parts):
                pw, ph = sizes[ri * ncols + ci]
                cx = x + ci * cell_w + cell_w / 2
                pins = sch.lib.pins(p["sym"], p.get("unit", 1))
                top = max((v[1] for v in pins.values()), default=0)
                vert = pins and all(abs(v[0]) < 0.5 for v in pins.values())
                lab = max((len(n) for n in p.get("nets", {}).values()), default=0) * 1.0 + 4.0
                cy = yy + (lab if vert else 4) + top
                sch.place(p, cx, cy)
            yy += rh
        col_right = max(col_right, x + g_w)
        y = yy + gap
