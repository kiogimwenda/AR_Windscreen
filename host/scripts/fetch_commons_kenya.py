#!/usr/bin/env python3
"""Download openly licensed photos of Kenyan roads and road signs from Wikimedia Commons.

    python fetch_commons_kenya.py --out ../data/footage/kenya --max 150

Only files whose licence (read from Commons' own metadata) is CC0, public domain, CC BY or
CC BY-SA are kept; anything else (e.g. non-commercial, unknown) is skipped. Author, licence and
file page are written to commons_manifest.csv for attribution. Images are fetched as 1280 px wide (a standard Commons thumbnail size; non-standard sizes get HTTP 429)
thumbnails, which keeps the download small. Diagrams (SVG drawings of signs) are skipped: the
point is real street imagery.
"""

import argparse
import csv
import json
import re
import time
import urllib.parse
import urllib.request
from pathlib import Path

UA = "AR_Windscreen-FYP/0.1 (student project)"
API = "https://commons.wikimedia.org/w/api.php"
CATEGORIES = [
    "Category:Road signs in Kenya",
    "Category:Warning road signs in Kenya",
    "Category:Street signs in Kenya",
    "Category:Road sign gantries in Kenya",
    "Category:Historic road signs in Kenya",
    "Category:Thika Superhighway",
    "Category:Mombasa Road",
    "Category:Waiyaki Way",
    "Category:Uhuru Highway",
    "Category:Roads in Nairobi",
    "Category:Streets in Nairobi",
    "Category:Roads in Kenya",
]
OK_LICENCE = re.compile(r"^(cc0|pd|public domain|cc by(-sa)? [0-9.]+)", re.I)


def api(params: dict) -> dict:
    params = {**params, "format": "json"}
    url = API + "?" + urllib.parse.urlencode(params)
    for i in range(5):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            if e.code == 429 or e.code >= 500:
                time.sleep(15 * 2 ** i)
                continue
            raise
    raise RuntimeError("API keeps failing")


def members(cat: str) -> list[str]:
    out, cont = [], {}
    while True:
        d = api({"action": "query", "list": "categorymembers", "cmtitle": cat,
                 "cmtype": "file", "cmlimit": 500, **cont})
        out += [m["title"] for m in d.get("query", {}).get("categorymembers", [])]
        if "continue" not in d:
            return out
        cont = d["continue"]
        time.sleep(1)


def strip_html(s: str) -> str:
    return re.sub(r"<[^>]+>", "", s or "").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max", type=int, default=150)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    titles = []
    for c in CATEGORIES:
        try:
            m = members(c)
        except Exception as e:  # noqa: BLE001
            print(f"{c}: {e}")
            continue
        print(f"{c}: {len(m)} files")
        titles += [t for t in m if t not in titles and re.search(r"\.(jpe?g|png)$", t, re.I)]
        time.sleep(1)
    manifest = args.out / "commons_manifest.csv"
    n = 0
    with open(manifest, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "commons_title", "author", "licence", "page_url"])
        for i in range(0, len(titles), 20):
            if n >= args.max:
                break
            d = api({"action": "query", "titles": "|".join(titles[i:i + 20]), "prop": "imageinfo",
                     "iiprop": "url|extmetadata", "iiurlwidth": 1280})
            for p in d["query"]["pages"].values():
                if n >= args.max:
                    break
                ii = (p.get("imageinfo") or [{}])[0]
                meta = ii.get("extmetadata", {})
                lic = strip_html(meta.get("LicenseShortName", {}).get("value", ""))
                if not OK_LICENCE.match(lic):
                    print(f"  skip ({lic or 'no licence'}): {p['title']}")
                    continue
                author = strip_html(meta.get("Artist", {}).get("value", ""))
                src = ii.get("thumburl") or ii.get("url")
                fn = "commons_" + re.sub(r"[^A-Za-z0-9._-]+", "_", p["title"][5:])[:100]
                fn = re.sub(r"\.(png|jpeg)$", ".jpg", fn, flags=re.I)
                if not fn.lower().endswith(".jpg"):
                    fn += ".jpg"
                try:
                    req = urllib.request.Request(src, headers={"User-Agent": UA})
                    with urllib.request.urlopen(req, timeout=120) as r:
                        (args.out / fn).write_bytes(r.read())
                except Exception as e:  # noqa: BLE001
                    print(f"  {fn}: {e}")
                    time.sleep(10)
                    continue
                w.writerow([fn, p["title"], author, lic, ii.get("descriptionurl", "")])
                f.flush()
                n += 1
                time.sleep(1.5)
    print(f"{n} files, manifest {manifest}")


if __name__ == "__main__":
    main()
