#!/usr/bin/env python3
"""1:1 drilling templates for the flat aluminium parts, from the OpenSCAD models' own hole lists.

    python3 hardware/enclosures/make_templates.py <model.scad> <out.pdf>

Runs OpenSCAD with part="holes" and draws every plate or panel the model echoes on its own A4
page at true size (landscape when it is too wide for portrait): outline, a crosshair at every hole
centre (centre-punch there), each hole's diameter and purpose, and a 100 mm scale bar. Echo lines:
  OUTLINE,plate,size,r        a square aluminium plate, origin at its centre, y towards the left
                              of the car (drawn up), with a FRONT arrow
  PANEL,name,w,h,description  a box wall or floor, origin at its bottom-left corner, y up
  HOLE,name,x,y,d,label       SLOT,name,x,y,w,h,label       DSUB,name,x,y,label (DB-25 cut-out)
 Print at 100 % ("actual size", never "fit to page") and measure the bar before
drilling. Prints to PDF with headless Edge (Windows) from WSL.
"""

import collections
import subprocess
import sys
from pathlib import Path

OPENSCAD = "/mnt/c/Program Files/OpenSCAD/openscad.com"
EDGE = "/mnt/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe"


def win(p):
    return subprocess.run(["wslpath", "-w", str(p)], capture_output=True, text=True).stdout.strip()


def holes(scad):
    out = scad.with_suffix(".holes.echo")
    subprocess.run([OPENSCAD, "-D", 'part="holes"', "-o", win(out), win(scad)],
                   capture_output=True, check=True)
    plates = collections.OrderedDict()
    for line in out.read_text().splitlines():
        if not line.startswith('ECHO: "'):
            continue
        f = line[7:-1].split(",")
        kind, plate = f[0], f[1]
        p = plates.setdefault(plate, {"holes": [], "slots": [], "dsubs": [], "w": 0, "h": 0, "r": 0,
                                      "centre": True, "desc": ""})
        if kind == "HOLE":
            p["holes"].append((float(f[2]), float(f[3]), float(f[4]), ",".join(f[5:])))
        elif kind == "SLOT":
            p["slots"].append((float(f[2]), float(f[3]), float(f[4]), float(f[5]), ",".join(f[6:])))
        elif kind == "DSUB":
            p["dsubs"].append((float(f[2]), float(f[3]), ",".join(f[4:])))
        elif kind == "OUTLINE":
            p["w"] = p["h"] = float(f[2])
            p["r"] = float(f[3])
        elif kind == "PANEL":
            p["w"], p["h"], p["centre"], p["desc"] = float(f[2]), float(f[3]), False, ",".join(f[4:])
    out.unlink()
    return plates


# DB-25 (shell size B) panel cut-out, the usual D-sub panel drawing: a trapezoid 39.0 mm wide at
# the top and 11.0 mm high with sides at 10 degrees, and two 3.1 mm screw holes 47.04 mm apart.
DSUB_W, DSUB_H, DSUB_PITCH = 39.0, 11.0, 47.04


def page(name, p):
    w, h, r = p["w"], p["h"], p["r"]
    m = 8 if p["centre"] else 3  # margin inside the SVG, mm (room for the FRONT arrow)
    W, H = w + 2 * m, h + 2 * m
    landscape = W > 190

    def at(x, y):  # model (x, y) -> SVG, y up
        return (m + w / 2 + x, m + h / 2 - y) if p["centre"] else (m + x, m + h - y)

    parts = [f'<rect x="{m}" y="{m}" width="{w}" height="{h}" rx="{r}" fill="none" stroke="#000" stroke-width="0.3"/>']
    legend = []

    def cross(cx, cy):
        parts.append(f'<path d="M{cx - 4} {cy}H{cx + 4}M{cx} {cy - 4}V{cy + 4}" stroke="#c00" stroke-width="0.15"/>')

    for i, (x, y, d, label) in enumerate(p["holes"]):
        cx, cy = at(x, y)
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="{d / 2}" fill="none" stroke="#000" stroke-width="0.2"/>')
        cross(cx, cy)
        parts.append(f'<text x="{cx + d / 2 + 1}" y="{cy - 2}" font-size="2.6">{i + 1}</text>')
        legend.append(f"<tr><td>{i + 1}</td><td>&Oslash; {d:g} mm</td><td>{label}</td><td>({x:.1f}, {y:.1f})</td></tr>")
    for (x, y, sw, sh, label) in p["slots"]:
        cx, cy = at(x, y)
        parts.append(f'<rect x="{cx - sw / 2}" y="{cy - sh / 2}" width="{sw}" height="{sh}" rx="{sw / 2}" '
                     'fill="none" stroke="#000" stroke-width="0.2"/>')
        cross(cx, cy)
        legend.append(f"<tr><td>slot</td><td>{sw:g} &times; {sh:g} mm</td><td>{label}</td><td>({x:.1f}, {y:.1f})</td></tr>")
    for (x, y, label) in p["dsubs"]:
        cx, cy = at(x, y)
        b = DSUB_W / 2 - DSUB_H * 0.17633  # half-width at the bottom (tan 10 deg)
        t, u = cy - DSUB_H / 2, cy + DSUB_H / 2
        parts.append(f'<path d="M{cx - DSUB_W / 2} {t}H{cx + DSUB_W / 2}L{cx + b} {u}H{cx - b}Z" '
                     'fill="none" stroke="#000" stroke-width="0.2"/>')
        cross(cx, cy)
        for sx in (-1, 1):
            parts.append(f'<circle cx="{cx + sx * DSUB_PITCH / 2}" cy="{cy}" r="1.55" fill="none" stroke="#000" stroke-width="0.2"/>')
            cross(cx + sx * DSUB_PITCH / 2, cy)
        legend.append(f"<tr><td>D</td><td>{DSUB_W:g} &times; {DSUB_H:g} mm D, 2 &times; &Oslash; 3.1 on {DSUB_PITCH:g}</td>"
                      f"<td>{label}. Drill the corners, cut, file to the line.</td><td>({x:.1f}, {y:.1f})</td></tr>")
    if p["centre"]:
        parts.append(f'<path d="M{m + w / 2 - 15} {m - 4}H{m + w / 2 + 15}l-3 -2m3 2l-3 2" stroke="#000" fill="none" stroke-width="0.3"/>')
        parts.append(f'<text x="{m + w / 2 + 18}" y="{m - 3}" font-size="3.2">FRONT (car forward)</text>')
        desc = (f"3 mm aluminium, {w:g} &times; {h:g} mm, corners R{r:g}. Coordinates from the plate's centre.")
    else:
        desc = (f"{p['desc'][:1].upper() + p['desc'][1:]}, {w:g} &times; {h:g} mm. Coordinates from the bottom-left "
                "corner. Measure the box first; if it differs, change box_in in the model and regenerate. "
                "Tape the outline to the panel's edges.")
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" '
           f'viewBox="0 0 {W} {H}" font-family="Arial">{"".join(parts)}</svg>')
    bar = ('<svg xmlns="http://www.w3.org/2000/svg" width="110mm" height="9mm" viewBox="0 0 110 9">'
           '<path d="M5 2V7M105 2V7M5 4.5H105" stroke="#000" stroke-width="0.3"/>'
           '<text x="40" y="9" font-size="3">100 mm: measure before drilling</text></svg>')
    return (f'<section class="{"wide" if landscape else ""}"><header><h1>{name.replace("_", " ")}: 1:1 drilling '
            f'template</h1>{bar}</header><p>{desc} Print at 100 % (actual size). Centre-punch each red cross, '
            f'pilot-drill 2.5 mm, then drill to size (a step drill for the large holes). Deburr.</p>'
            f'<div>{svg}</div><table><tr><th>#</th><th>Size</th><th>For</th><th>(x, y) mm</th></tr>'
            f'{"".join(legend)}</table></section>')


def main(scad, pdf):
    scad, pdf = Path(scad).resolve(), Path(pdf).resolve()
    plates = holes(scad)
    html = ('<!doctype html><html><head><meta charset="utf-8"><style>'
            '@page{size:A4;margin:10mm} @page wide{size:A4 landscape;margin:6mm} section.wide{page:wide}'
            'body{font-family:Arial;font-size:9pt;margin:0}'
            'section{page-break-after:always} h1{font-size:13pt;margin:0}'
            'header{display:flex;justify-content:space-between;align-items:center} p{margin:1mm 0}'
            'table{border-collapse:collapse;margin-top:2mm}td,th{border:0.2mm solid #888;padding:0.3mm 2mm}'
            '</style></head><body>' + "".join(page(n, p) for n, p in plates.items()) + '</body></html>')
    tmp = pdf.with_suffix(".html")
    tmp.write_text(html)
    subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    f"--print-to-pdf={win(pdf)}", "file:///" + win(tmp).replace("\\", "/")],
                   capture_output=True, timeout=120)
    tmp.unlink()
    print(f"wrote {pdf} ({', '.join(plates)})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
