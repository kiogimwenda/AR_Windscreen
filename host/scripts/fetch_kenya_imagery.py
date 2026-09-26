#!/usr/bin/env python3
"""Download openly licensed Kenyan street-level images for evaluating the sign detector.

Source: KartaView (formerly OpenStreetCam), CC BY-SA 4.0. Only public, already-blurred ("proc")
images are fetched. Every image's author and sequence are written to a manifest CSV, because
CC BY-SA requires attribution (data/README.md summarises it).

    python fetch_kenya_imagery.py --out ../data/footage/kenya --target 500

Politeness: one request at a time, a pause between requests, back-off on HTTP 429/5xx, and a
User-Agent that carries no personal data.
"""

import argparse
import csv
import json
import random
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

UA = "AR_Windscreen-FYP/0.1 (student project)"
API1 = "https://api.openstreetcam.org/1.0/list/"
API2 = "https://api.openstreetcam.org/2.0/photo/"

# (name, north-west lat,lng, south-east lat,lng). Nairobi carries most weight: it has most of the
# coverage, and Thika Road, Mombasa Road, Waiyaki Way and Uhuru Highway all lie inside it.
REGIONS = [
    ("nairobi", (-1.15, 36.65), (-1.40, 37.00)),
    ("thika", (-0.98, 36.95), (-1.15, 37.12)),
    ("a104_limuru_naivasha_nakuru", (-0.20, 36.00), (-1.15, 36.70)),
    ("mombasa", (-3.95, 39.58), (-4.10, 39.75)),
    ("eldoret", (0.58, 35.20), (0.45, 35.35)),
    ("kisumu", (-0.05, 34.70), (-0.15, 34.82)),
]


def http(url: str, data: dict | None = None, tries: int = 5) -> bytes:
    body = urllib.parse.urlencode(data).encode() if data else None
    for i in range(tries):
        req = urllib.request.Request(url, data=body, headers={"User-Agent": UA})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code in (429, 500, 502, 503, 504):
                wait = 10 * 2 ** i
                print(f"  HTTP {e.code}, backing off {wait}s")
                time.sleep(wait)
                continue
            raise
        except (urllib.error.URLError, TimeoutError) as e:
            print(f"  {e}, retrying")
            time.sleep(5 * (i + 1))
    raise RuntimeError(f"giving up on {url}")


def list_sequences(nw, se, tile=0.05):
    """The v1 list endpoint returns only a handful of sequences per query, however large the box,
    so the region is covered by small tiles and the results are merged."""
    seqs = {}
    lat = nw[0]
    while lat > se[0]:
        lng = nw[1]
        while lng < se[1]:
            try:
                d = json.loads(http(API1, {"bbTopLeft": f"{lat},{lng}",
                                           "bbBottomRight": f"{lat - tile},{lng + tile}",
                                           "ipp": 100, "page": 1}))
            except Exception as e:  # noqa: BLE001
                print(f"  tile {lat},{lng}: {e}")
                d = {}
            for s in d.get("currentPageItems") or []:
                if s.get("country_code") == "KE":
                    seqs[s["id"]] = s
            time.sleep(0.7)
            lng += tile
        lat -= tile
    return list(seqs.values())


def sequence_photos(seq_id: str):
    photos, page = [], 1
    while True:
        d = json.loads(http(f"{API2}?sequenceId={seq_id}&itemsPerPage=150&page={page}&join=user"))
        r = d.get("result") or {}
        photos += r.get("data") or []
        time.sleep(0.5)
        if not r.get("hasMoreData") or page >= 20:
            return photos
        page += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--target", type=int, default=500)
    ap.add_argument("--per-seq", type=int, default=10, help="max images kept per sequence")
    ap.add_argument("--stride", type=int, default=12, help="keep every Nth photo of a sequence")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--regions", nargs="*", help="only these region names (default: all)")
    args = ap.parse_args()
    random.seed(args.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    manifest = args.out / "manifest.csv"
    done = set()
    if manifest.exists():
        with open(manifest) as f:
            done = {row["file"] for row in csv.DictReader(f)}

    # Region quota: half for Nairobi, the rest shared.
    quota = {"nairobi": args.target // 2}
    rest = [r[0] for r in REGIONS if r[0] != "nairobi"]
    for name in rest:
        quota[name] = (args.target - quota["nairobi"]) // len(rest)

    new = not manifest.exists()
    with open(manifest, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["file", "region", "photo_id", "sequence_id", "sequence_index", "author",
                        "user_id", "shot_date", "lat", "lng", "source_url", "licence"])
        for name, nw, se in REGIONS:
            if args.regions and name not in args.regions:
                continue
            seqs = list_sequences(nw, se)
            random.shuffle(seqs)
            print(f"{name}: {len(seqs)} sequences, quota {quota[name]}")
            got = sum(1 for x in done if x.startswith(name + "_"))
            for s in seqs:
                if got >= quota[name]:
                    break
                try:
                    photos = sequence_photos(s["id"])
                except Exception as e:  # noqa: BLE001 - one bad sequence must not stop the run
                    print(f"  seq {s['id']}: {e}")
                    continue
                photos.sort(key=lambda p: int(p["sequenceIndex"]))
                picked = photos[args.stride // 2::args.stride][:args.per_seq]
                for p in picked:
                    if got >= quota[name]:
                        break
                    fn = f"{name}_{p['sequenceId']}_{int(p['sequenceIndex']):05d}.jpg"
                    if fn in done:
                        continue
                    url = p.get("imageProcUrl") or p["fileurlProc"]
                    try:
                        img = http(url, tries=2)  # some storage hosts return 500 for hours
                    except Exception as e:  # noqa: BLE001
                        print(f"  {fn}: {e}")
                        continue
                    if len(img) < 20000:  # placeholder / broken image
                        continue
                    (args.out / fn).write_bytes(img)
                    user = p.get("user") or {}
                    w.writerow([fn, name, p["id"], p["sequenceId"], p["sequenceIndex"],
                                user.get("username", ""), user.get("id", ""), p.get("shotDate", ""),
                                p["lat"], p["lng"],
                                f"https://kartaview.org/details/{p['sequenceId']}/{p['sequenceIndex']}",
                                "CC BY-SA 4.0"])
                    f.flush()
                    done.add(fn)
                    got += 1
                    time.sleep(0.5)
            print(f"{name}: {got} images")


if __name__ == "__main__":
    main()
