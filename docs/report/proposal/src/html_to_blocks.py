#!/usr/bin/env python3
"""Turns a built proposal edition (HTML) into a simple block list (JSON) for make_docx.js, so the
Word version is generated from exactly the same text as the HTML/PDF, citations already numbered.

    python3 html_to_blocks.py <edition.html> <out.json>

Blocks: cover, h1/h2/h3 (+id), p (runs), list (ordered?, items of runs), table (rows of cells of
runs, header row flagged), tabcap, toc, refs (items of runs), pagebreak. A run is
{"t": text, "b": bold, "i": italic, "u": underline, "sub": ..., "sup": ...}.
"""

import json
import re
import sys
from html.parser import HTMLParser

INLINE = {"b": "b", "strong": "b", "i": "i", "em": "i", "sub": "sub", "sup": "sup"}


class P(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.blocks = []
        self.fmt = []          # stack of inline formats
        self.runs = None       # runs of the paragraph-like element being read
        self.skip = 0          # inside <style>, <head> etc.
        self.section_cls = ""
        self.list_stack = []   # [{"ordered": bool, "items": [...], "refs": bool}]
        self.table = None
        self.row = None
        self.cell = None
        self.in_toc = False
        self.cover = None
        self.p_cls = ""

    # --- helpers -------------------------------------------------------------------------------
    def start_runs(self):
        self.runs = []

    def add_text(self, text):
        if self.runs is None:
            return
        text = re.sub(r"\s+", " ", text)
        if not text:
            return
        f = set(self.fmt)
        run = {"t": text, "b": "b" in f, "i": "i" in f, "sub": "sub" in f, "sup": "sup" in f}
        if self.runs and all(self.runs[-1][k] == run[k] for k in ("b", "i", "sub", "sup")):
            self.runs[-1]["t"] += text
        else:
            self.runs.append(run)

    def finish_runs(self):
        runs = self.runs or []
        self.runs = None
        if runs:
            runs[0]["t"] = runs[0]["t"].lstrip()
            runs[-1]["t"] = runs[-1]["t"].rstrip()
        return [r for r in runs if r["t"]]

    # --- parser --------------------------------------------------------------------------------
    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        cls = a.get("class", "") or ""
        if tag in ("style", "script", "head", "title"):
            self.skip += 1
            return
        if self.skip:
            return
        if tag == "section":
            self.section_cls = cls
            if "cover" in cls:
                self.cover = {"lines": [], "student": []}
            elif a.get("id") in (None, ""):
                pass
            if "sheet" in cls and self.blocks and "cover" not in cls and a.get("id"):
                self.blocks.append({"type": "pagebreak"})
            elif "front" in cls and self.blocks:
                self.blocks.append({"type": "pagebreak"})
            return
        if tag == "ul" and "toc" in cls:
            self.in_toc = True
            return
        if self.in_toc:
            return
        if tag in ("h1", "h2", "h3"):
            self.start_runs()
            self.h = (tag, a.get("id"))
        elif tag == "p":
            self.p_cls = cls
            self.start_runs()
        elif tag in ("ol", "ul"):
            self.list_stack.append({"ordered": tag == "ol", "items": [], "refs": "refs" in cls})
        elif tag == "li":
            self.start_runs()
        elif tag == "table":
            self.table = {"rows": [], "cls": cls}
        elif tag == "tr":
            self.row = []
        elif tag in ("td", "th"):
            self.start_runs()
            self.cell_head = tag == "th"
        elif tag in INLINE:
            self.fmt.append(INLINE[tag])
        elif tag == "span" and "refno" in cls:
            self.fmt.append("refno")
        elif tag == "br":
            self.add_text(" ")

    def handle_endtag(self, tag):
        if tag in ("style", "script", "head", "title"):
            self.skip -= 1
            return
        if self.skip:
            return
        if tag == "ul" and self.in_toc:
            self.in_toc = False
            self.blocks.append({"type": "toc"})
            return
        if self.in_toc:
            return
        if tag == "section" and self.cover is not None and "cover" in self.section_cls:
            self.blocks.append({"type": "cover", **self.cover})
            self.cover = None
            self.section_cls = ""
            return
        if tag in ("h1", "h2", "h3"):
            runs = self.finish_runs()
            self.blocks.append({"type": tag, "id": self.h[1], "text": "".join(r["t"] for r in runs)})
        elif tag == "p":
            runs = self.finish_runs()
            if self.cover is not None:
                self.cover["lines"].append({"text": "".join(r["t"] for r in runs),
                                            "u": "u" in self.p_cls.split()})
            elif "tabcap" in self.p_cls:
                self.blocks.append({"type": "tabcap", "text": "".join(r["t"] for r in runs)})
            elif runs:
                self.blocks.append({"type": "p", "runs": runs})
            self.p_cls = ""
        elif tag == "li":
            runs = self.finish_runs()
            self.list_stack[-1]["items"].append(runs)
        elif tag in ("ol", "ul"):
            lst = self.list_stack.pop()
            if lst["refs"]:
                self.blocks.append({"type": "refs", "items": lst["items"]})
            else:
                self.blocks.append({"type": "list", "ordered": lst["ordered"], "items": lst["items"]})
        elif tag in ("td", "th"):
            runs = self.finish_runs()
            self.row.append({"runs": runs, "head": self.cell_head})
        elif tag == "tr":
            if self.cover is not None:
                self.cover["student"].append(["".join(r["t"] for r in c["runs"]) for c in self.row])
            else:
                self.table["rows"].append(self.row)
            self.row = None
        elif tag == "table":
            if self.cover is None:
                self.blocks.append({"type": "table", "rows": self.table["rows"],
                                    "booktabs": "booktabs" in self.table["cls"],
                                    "lined": "lined" in self.table["cls"],
                                    "wide": "wide" in self.table["cls"]})
            self.table = None
        elif tag in INLINE and self.fmt:
            self.fmt.pop()
        elif tag == "span" and self.fmt and self.fmt[-1] == "refno":
            self.fmt.pop()
            self.add_text("\t")

    def handle_data(self, data):
        if self.skip or self.in_toc:
            return
        if "refno" in self.fmt:
            if self.runs is not None:
                self.runs.append({"t": data.strip(), "b": False, "i": False, "sub": False, "sup": False})
            return
        self.add_text(data)


def main():
    src, out = sys.argv[1], sys.argv[2]
    p = P()
    p.feed(open(src, encoding="utf-8").read())
    json.dump(p.blocks, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    kinds = {}
    for b in p.blocks:
        kinds[b["type"]] = kinds.get(b["type"], 0) + 1
    print(f"{len(p.blocks)} blocks: {kinds}")


if __name__ == "__main__":
    main()
