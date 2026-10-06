#!/usr/bin/env python3
"""Builds the self-contained proposal HTML from src/parts/*.html (concatenated in name order).

    python3 docs/report/proposal/src/make_figures.py   # diagrams (only after editing them)
    python3 docs/report/proposal/src/build.py          # -> docs/report/proposal/<name>.html

Markers in the source, resolved here:
    {{fig:file.svg|Caption|4.5}}     inline SVG figure, numbered; optional max height in inches
    {{img:file.jpg|Caption|70}}      embedded image at 70 % width, numbered as a figure
    {{tab:key|Caption}}              a numbered table caption (place directly above the table)
    {{ref:file.svg}} {{ref:key}}     "Figure n" / "Table n" for a figure or table
    {{cite:a,b}}                     IEEE citation numbers, in order of first citation
    {{refs}}                         the reference list
The fonts (Liberation Serif, SIL Open Font Licence) are embedded, so the page looks the same on
any machine and prints to PDF faithfully.
"""

import base64
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FIGS = ROOT / "figures"
FONT_DIR = Path("/usr/share/fonts/truetype/liberation")
OUT = ROOT / "Kiogora_Ian_Mwenda_FYP_Project_Proposal.html"

# Shorter editions built from the same parts (python3 build.py --edition intro):
EDITIONS = {
    "full": (None, OUT),
    # Cover to the literature review, with the references those chapters cite: for a short
    # presentation of the proposal's first half.
    "intro": (("00", "10", "20", "45", "99"),
              ROOT / "Kiogora_Ian_Mwenda_FYP_Proposal_Intro_and_Literature_Review.html"),
}

from references import REFERENCES  # noqa: E402  (key -> IEEE entry, HTML allowed)


def b64(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


def fonts_css():
    faces = [("Regular", "normal", "normal"), ("Bold", "bold", "normal"),
             ("Italic", "normal", "italic"), ("BoldItalic", "bold", "italic")]
    out = []
    for name, weight, style in faces:
        data = b64(FONT_DIR / f"LiberationSerif-{name}.ttf")
        out.append(f"@font-face{{font-family:'Liberation Serif';font-weight:{weight};font-style:{style};"
                   f"src:url(data:font/ttf;base64,{data}) format('truetype');}}")
    return "\n".join(out)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--edition", choices=sorted(EDITIONS), default="full")
    edition, out = EDITIONS[ap.parse_args().edition]
    parts = [p for p in sorted((HERE / "parts").glob("*.html"))
             if edition is None or p.name[:2] in edition]
    src = "\n".join(p.read_text() for p in parts)
    # Table-of-contents lines whose target is not in this edition are dropped.
    ids = set(re.findall(r'id="([^"]+)"', src))
    src = re.sub(r'<li class="l\d"><a href="#([^"]+)">.*?</a></li>\n?',
                 lambda m: m.group(0) if m.group(1) in ids else "", src)

    # Number figures and tables in document order.
    fig_no, tab_no = {}, {}
    for m in re.finditer(r"\{\{(fig|img|tab):([^|}]+)", src):
        kind, key = m.group(1), m.group(2).strip()
        if kind == "tab":
            tab_no.setdefault(key, len(tab_no) + 1)
        else:
            fig_no.setdefault(key, len(fig_no) + 1)

    def fig(m):
        key, cap = m.group(1).strip(), m.group(2).strip()
        style = f' style="max-height:{m.group(3).strip()}in"' if m.group(3) else ""
        svg = (FIGS / key).read_text()
        svg = re.sub(r"<svg ", f'<svg class="diagram" role="img"{style} ', svg, count=1)
        return (f'<figure id="{key}">{svg}<figcaption>Figure {fig_no[key]}: {cap}</figcaption></figure>')

    def img(m):
        key, cap, width = m.group(1).strip(), m.group(2).strip(), m.group(3).strip()
        mime = "image/png" if key.endswith(".png") else "image/jpeg"
        return (f'<figure id="{key}"><img src="data:{mime};base64,{b64(FIGS / key)}" '
                f'style="width:{width}%" alt=""><figcaption>Figure {fig_no[key]}: {cap}</figcaption></figure>')

    def tab(m):
        key, cap = m.group(1).strip(), m.group(2).strip()
        return f'<p class="tabcap" id="{key}">Table {tab_no[key]}: {cap}</p>'

    def ref(m):
        key = m.group(1).strip()
        if key in fig_no:
            return f'<a href="#{key}">Figure {fig_no[key]}</a>'
        if key in tab_no:
            return f'<a href="#{key}">Table {tab_no[key]}</a>'
        raise KeyError(f"unknown figure/table reference {key}")

    src = re.sub(r"\{\{fig:([^|}]+)\|([^|}]+)(?:\|([^}]+))?\}\}", fig, src)
    src = re.sub(r"\{\{img:([^|}]+)\|([^|}]+)\|([^}]+)\}\}", img, src)
    src = re.sub(r"\{\{tab:([^|}]+)\|([^}]+)\}\}", tab, src)
    src = re.sub(r"\{\{ref:([^}]+)\}\}", ref, src)

    # Citations, numbered in order of first appearance (IEEE).
    order = []

    def cite(m):
        keys = [k.strip() for k in m.group(1).split(",")]
        nums = []
        for k in keys:
            if k not in REFERENCES:
                raise KeyError(f"unknown reference {k}")
            if k not in order:
                order.append(k)
            nums.append(order.index(k) + 1)
        return "".join(f'<a class="cite" href="#ref-{n}">[{n}]</a>' for n in nums)

    src = re.sub(r"\{\{cite:([^}]+)\}\}", cite, src)
    unused = [k for k in REFERENCES if k not in order]
    if unused and edition is None:
        print("note: references never cited:", ", ".join(unused))
    items = "\n".join(f'<li id="ref-{i + 1}"><span class="refno">[{i + 1}]</span> {REFERENCES[k]}</li>'
                      for i, k in enumerate(order))
    src = src.replace("{{refs}}", f'<ol class="refs">{items}</ol>')

    src = src.replace("/*{{fonts}}*/", fonts_css())
    src = src.replace("{{logo}}", f"data:image/png;base64,{b64(FIGS / 'jkuat_logo.png')}")
    left = re.findall(r"\{\{[^}]*\}\}", src)
    if left:
        raise ValueError(f"unresolved markers: {left[:5]}")
    out.write_text(src)
    print(f"wrote {out.relative_to(ROOT.parents[2])}: {out.stat().st_size / 1e6:.1f} MB, "
          f"{len(fig_no)} figures, {len(tab_no)} tables, {len(order)} references")


if __name__ == "__main__":
    main()
