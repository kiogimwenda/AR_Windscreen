#!/usr/bin/env python3
"""Board outlines and mounting holes: the one shared constraint between PCB layout and enclosures.

    python3 hardware/gen/outlines.py           # writes the starter .kicad_pcb files (only if
                                               # absent) and hardware/enclosures/common/boards.scad
    python3 hardware/gen/outlines.py --force   # also overwrites existing .kicad_pcb files

Each board's size, corner radius, mounting holes and connector "keep" positions are defined once,
here. The starter PCB carries:
  - the outline on Edge.Cuts (rounded corners) and the copper layer count;
  - each mounting hole's position as a circle and label on User.Drawings: place H1-H4 (the
    schematic's mounting-hole footprints) exactly on them;
  - where each edge connector goes, as a labelled box on User.Comments.
The enclosures (hardware/enclosures/*.scad) include boards.scad, so a change here moves the box's
standoffs and cut-outs with the board. Lay out the board INSIDE these constraints; if one has to
change, change it here and regenerate both.
"""

import argparse
import math
import uuid
from pathlib import Path

HW = Path(__file__).resolve().parents[1]

# Coordinates: mm, origin at the board's top-left corner, x right, y down (KiCad's convention).
# A "keep" is (label, x, y, w, h): where an edge component must sit, and the enclosure's cut-out.
BOARDS = {
    "pod_board": {
        "w": 80.0, "h": 55.0, "r": 2.0, "copper": 4, "thickness": 1.6,
        "holes": [(4.0, 4.0), (76.0, 4.0), (4.0, 51.0), (76.0, 51.0)],  # M3, 3.2 mm
        "keeps": [
            ("DB-25 (J7): flange flush with the bottom edge, centred", 13.5, 41.0, 53.0, 14.0),
            ("USB-C (J1): flush with the right edge", 70.5, 22.0, 9.5, 9.0),
            ("Status LEDs D3-D6: top edge, facing the driver", 30.0, 0.5, 20.0, 3.0),
            ("SWD (J5) and ELM327 (J6) headers: left edge", 0.5, 12.0, 6.0, 25.0),
        ],
        "notes": "4 layers: signal / GND / +3V3 / signal. Ground plane unbroken under GNSS and IMU.",
    },
    "power_board": {
        "w": 110.0, "h": 80.0, "r": 2.0, "copper": 2, "thickness": 1.6,
        "holes": [(4.0, 4.0), (106.0, 4.0), (4.0, 76.0), (106.0, 76.0)],
        "keeps": [
            ("DB-25 (J1): flange flush with the top edge", 28.5, 0.0, 53.0, 14.0),
            ("Fuse holders F1, F3: top edge, left", 2.0, 14.0, 24.0, 22.0),
            ("Terminal blocks J2-J11: bottom edge (all car-side wiring)", 9.0, 66.0, 92.0, 14.0),
            ("BTS7960 / ACS712 module headers J13, J14: right edge", 96.0, 20.0, 14.0, 40.0),
        ],
        "notes": "2 layers, order 2 oz (70 um) copper. TPS54360 loop compact (datasheet layout).",
    },
}


def u():
    return f'"{uuid.uuid4()}"'


def line(x1, y1, x2, y2, layer, width=0.1):
    return (f'\t(gr_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(stroke (width {width}) (type default)) (layer "{layer}") (uuid {u()}))')


def arc(cx, cy, r, a0, layer, width=0.1):
    """90 degree arc around (cx, cy) from angle a0 (deg, y down) clockwise on screen."""
    pts = [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))
           for a in (a0, a0 + 45, a0 + 90)]
    (sx, sy), (mx, my), (ex, ey) = pts
    return (f'\t(gr_arc (start {sx:.3f} {sy:.3f}) (mid {mx:.3f} {my:.3f}) (end {ex:.3f} {ey:.3f}) '
            f'(stroke (width {width}) (type default)) (layer "{layer}") (uuid {u()}))')


def text(s, x, y, layer, size=1.0):
    s = s.replace('"', "'")
    return (f'\t(gr_text "{s}" (at {x:.3f} {y:.3f} 0) (layer "{layer}") (uuid {u()}) '
            f'(effects (font (size {size} {size}) (thickness 0.15)) (justify left)))')


def circle(cx, cy, r, layer, width=0.1):
    return (f'\t(gr_circle (center {cx:.3f} {cy:.3f}) (end {cx + r:.3f} {cy:.3f}) '
            f'(stroke (width {width}) (type default)) (fill no) (layer "{layer}") (uuid {u()}))')


def pcb(name, b):
    ox, oy = 50.0, 50.0  # place the board away from the sheet corner
    w, h, r = b["w"], b["h"], b["r"]
    copper = ['\t\t(0 "F.Cu" signal)']
    if b["copper"] == 4:
        copper += ['\t\t(4 "In1.Cu" signal)', '\t\t(6 "In2.Cu" signal)']
    copper += ['\t\t(2 "B.Cu" signal)']
    user = ['(9 "F.Adhes" user "F.Adhesive")', '(11 "B.Adhes" user "B.Adhesive")',
            '(13 "F.Paste" user)', '(15 "B.Paste" user)', '(5 "F.SilkS" user "F.Silkscreen")',
            '(7 "B.SilkS" user "B.Silkscreen")', '(1 "F.Mask" user)', '(3 "B.Mask" user)',
            '(17 "Dwgs.User" user "User.Drawings")', '(19 "Cmts.User" user "User.Comments")',
            '(25 "Edge.Cuts" user)', '(27 "Margin" user)', '(31 "F.CrtYd" user "F.Courtyard")',
            '(29 "B.CrtYd" user "B.Courtyard")', '(35 "F.Fab" user)', '(33 "B.Fab" user)']
    g = []
    E = "Edge.Cuts"
    g += [line(ox + r, oy, ox + w - r, oy, E), line(ox + w, oy + r, ox + w, oy + h - r, E),
          line(ox + w - r, oy + h, ox + r, oy + h, E), line(ox, oy + h - r, ox, oy + r, E)]
    g += [arc(ox + w - r, oy + r, r, 270, E), arc(ox + w - r, oy + h - r, r, 0, E),
          arc(ox + r, oy + h - r, r, 90, E), arc(ox + r, oy + r, r, 180, E)]
    for i, (hx, hy) in enumerate(b["holes"]):
        g += [circle(ox + hx, oy + hy, 1.6, "Dwgs.User"), circle(ox + hx, oy + hy, 3.5, "Dwgs.User"),
              text(f"H{i + 1}: M3 (3.2 mm), 7 mm keep-out", ox + hx + 4.0, oy + hy, "Dwgs.User", 0.8)]
    for label, kx, ky, kw, kh in b["keeps"]:
        X, Y = ox + kx, oy + ky
        g += [line(X, Y, X + kw, Y, "Cmts.User"), line(X + kw, Y, X + kw, Y + kh, "Cmts.User"),
              line(X + kw, Y + kh, X, Y + kh, "Cmts.User"), line(X, Y + kh, X, Y, "Cmts.User"),
              text(label, X + 0.5, Y + kh / 2, "Cmts.User", 0.7)]
    g.append(text(f"{name}: {w:g} x {h:g} mm, {b['copper']} layers. {b['notes']} "
                  "Outline and holes from hardware/gen/outlines.py (shared with the enclosures).",
                  ox, oy - 4.0, "Cmts.User", 1.0))
    return "\n".join([
        "(kicad_pcb", '\t(version 20250513)', '\t(generator "pcbnew")', '\t(generator_version "10.0")',
        f'\t(general (thickness {b["thickness"]}) (legacy_teardrops no))', '\t(paper "A4")',
        f'\t(title_block (title "{name}") (date "2026-10-06") (rev "1") '
        '(company "AR Windscreen FYP - Kiogora Ian Mwenda"))',
        "\t(layers", *copper, *["\t\t" + x for x in user], "\t)",
        "\t(setup (pad_to_mask_clearance 0) (allow_soldermask_bridges_in_footprints no))",
        '\t(net 0 "")', *g, ")", ""])


def scad():
    out = ["// Generated by hardware/gen/outlines.py: do not edit; change the board there.",
           "// Board origin: top-left corner, x right, y DOWN (KiCad). Units: mm.", ""]
    for name, b in BOARDS.items():
        out.append(f"{name} = [{b['w']}, {b['h']}, {b['thickness']}];  // w, h, thickness")
        out.append(f"{name}_r = {b['r']};")
        out.append(f"{name}_holes = [" + ", ".join(f"[{x}, {y}]" for x, y in b["holes"]) + "];")
        out.append(f"{name}_keeps = [" + ", ".join(f"[{x}, {y}, {w}, {h}]" for _, x, y, w, h in
                                                  b["keeps"]) + "];  // x, y, w, h")
        for i, (label, *_) in enumerate(b["keeps"]):
            out.append(f"// {name}_keeps[{i}]: {label}")
        out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true", help="overwrite existing .kicad_pcb files")
    a = ap.parse_args()
    for name, b in BOARDS.items():
        p = HW / name / f"{name}.kicad_pcb"
        if p.exists() and not a.force:
            print(f"kept {p} (exists; --force to overwrite a layout in progress)")
        else:
            p.write_text(pcb(name, b), encoding="utf-8")
            print(f"wrote {p}")
    sp = HW / "enclosures" / "common" / "boards.scad"
    sp.parent.mkdir(parents=True, exist_ok=True)
    sp.write_text(scad(), encoding="utf-8")
    print(f"wrote {sp}")


if __name__ == "__main__":
    main()
