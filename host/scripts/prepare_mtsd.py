#!/usr/bin/env python3
"""Convert the Mapillary Traffic Sign Dataset (MTSD, fully annotated part) to YOLO format.

    python prepare_mtsd.py --mtsd ../data/datasets/mtsd --out ../data/datasets/mtsd_yolo
    python prepare_mtsd.py --mtsd ../data/datasets/mtsd --labels-only   # print the label list only
    python prepare_mtsd.py --mtsd ../data/datasets/mtsd --out ../data/datasets/mtsd_yolo_v3 \
        --us-speed-variants scripts/mtsd_uncertain_unit_speed_variants.json --mask-ignored

MTSD ships one JSON file per image. Each object has a `label` such as
"regulatory--maximum-speed-limit-50--g1": category, sign name, design variant (g1, g2, ...).
This script maps each label onto the 29 target classes below, merging the design variants, and
writes the mapping to scripts/mtsd_class_map.json so it can be read and checked.

What is dropped, and why:
  - information--*, complementary--* and other-sign: not trained. Their boxes are removed, so those
    signs count as background.
  - boxes flagged `ambiguous` (the annotator could not tell which sign it is): a wrong label is
    worse than a missing one. Occluded boxes are KEPT: partly hidden signs are normal on the road.
  - boxes flagged `dummy` (sign-like things that are not real traffic signs, e.g. printed on a
    vehicle): removed, so they count as background, which is what they are for the AR display.
  - panoramas (`ispano`): equirectangular 360-degree images whose boxes can wrap around the image
    edge. The production camera is a forward-facing pinhole camera, so they are skipped.

Images are re-saved once with the long side capped at 2048 px (boxes scaled to match), so that
data loading at imgsz=1280 does not have to decode 4000+ px JPEGs every epoch.
"""

import argparse
import json
import random
import re
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2

CLASSES = [
    "stop", "give_way", "no_entry", "no_left_turn", "no_right_turn", "no_u_turn", "no_overtaking",
    "speed_limit_10", "speed_limit_20", "speed_limit_30", "speed_limit_40", "speed_limit_50",
    "speed_limit_60", "speed_limit_70", "speed_limit_80", "speed_limit_90", "speed_limit_100",
    "speed_limit_110", "speed_limit_120",
    "end_of_restriction", "pedestrian_crossing", "children_school", "road_hump", "roundabout",
    "traffic_signals_ahead", "keep_left_or_right", "no_parking_or_stopping",
    "other_warning", "other_regulatory",
]
ID = {n: i for i, n in enumerate(CLASSES)}
MAX_SIDE = 2048

# Exact sign names (label without its --gN variant suffix) -> target class. Checked against the
# real label list (401 labels in MTSD v2) and against crops of every mapped variant. A few names
# here do not occur in MTSD v2 (e.g. end-of-no-overtaking); they are harmless and kept so that the
# rule reads as intended. Deliberately NOT merged, and so left in other_regulatory/other_warning:
# mandatory turn/straight-only signs (a different meaning from keep left/right),
# no-overtaking-by-heavy-goods-vehicles, give-way-to-oncoming-traffic, end-of-no-parking and the
# other "end of <lane type>" signs, stop-ahead, playground and uneven-road.
EXACT = {
    "regulatory--stop": "stop",
    "regulatory--yield": "give_way",
    "regulatory--no-entry": "no_entry",
    "regulatory--no-left-turn": "no_left_turn",
    "regulatory--no-right-turn": "no_right_turn",
    "regulatory--no-u-turn": "no_u_turn",
    "regulatory--no-overtaking": "no_overtaking",
    "regulatory--keep-left": "keep_left_or_right",
    "regulatory--keep-right": "keep_left_or_right",
    "regulatory--pass-on-either-side": "keep_left_or_right",
    "warning--pass-left-or-right": "keep_left_or_right",  # same meaning, US-style warning design
    "regulatory--roundabout": "roundabout",
    "warning--roundabout": "roundabout",
    "warning--traffic-signals": "traffic_signals_ahead",
    "warning--pedestrians-crossing": "pedestrian_crossing",
    "regulatory--pedestrians-crossing": "pedestrian_crossing",
    "warning--children": "children_school",
    "warning--school-zone": "children_school",
    "regulatory--school-zone": "children_school",
    "warning--road-bump": "road_hump",
    "warning--speed-bump": "road_hump",
    "warning--speed-bumps": "road_hump",
    "warning--hump": "road_hump",
    "regulatory--no-parking": "no_parking_or_stopping",
    "regulatory--no-stopping": "no_parking_or_stopping",
    "regulatory--no-standing": "no_parking_or_stopping",
    "regulatory--no-parking-or-no-stopping": "no_parking_or_stopping",
    "regulatory--end-of-maximum-speed-limit": "end_of_restriction",
    "regulatory--end-of-no-overtaking": "end_of_restriction",
    "regulatory--end-of-prohibition": "end_of_restriction",
    "regulatory--end-of-speed-limit-zone": "end_of_restriction",
}
SPEED_RE = re.compile(r"^regulatory--maximum-speed-limit-(?:led-)?(\d+)$")
END_SPEED_RE = re.compile(r"^regulatory--end-of-maximum-speed-limit-(?:led-)?\d+$")


def stem(label: str) -> str:
    return re.sub(r"--g\d+$", "", label)


def map_label(label: str, us_speed_variants: set[str]) -> int | None:
    """Target class id, or None when the box is dropped."""
    s = stem(label)
    cat = s.split("--")[0]
    if cat not in ("regulatory", "warning"):
        return None  # information, complementary, other-sign: not trained
    if s in EXACT:
        return ID[EXACT[s]]
    m = SPEED_RE.match(s)
    if m:
        v = int(m.group(1))
        name = f"speed_limit_{v}"
        # A US "SPEED LIMIT 40" is 40 mph, not 40 km/h. Design variants whose unit is mph or
        # cannot be told from crops are DROPPED (background), not merged into a km/h class.
        if label in us_speed_variants:
            return None
        return ID[name] if name in ID else ID["other_regulatory"]
    if END_SPEED_RE.match(s):
        return ID["end_of_restriction"]
    return ID["other_warning"] if cat == "warning" else ID["other_regulatory"]


def load_json(p: Path) -> dict:
    with open(p) as f:
        return json.load(f)


# Grey used to paint out ignored boxes: ultralytics' letterbox fill, i.e. a value the model already
# treats as "no image content" rather than as background scenery.
MASK_GREY = (114, 114, 114)


def convert_one(job):
    key, split, ann_path, img_path, out, class_map, mask_labels, mask_ignored = job
    done_img = out / "images" / split / f"{key}.jpg"
    done_lbl = out / "labels" / split / f"{key}.txt"
    ann = load_json(ann_path)
    if ann.get("ispano"):
        return key, "pano", []
    if not img_path.exists():
        return key, "noimg", []
    W, H = ann["width"], ann["height"]
    lines, kept = [], []
    masked, kept_px = [], []  # rectangles to grey out; kept boxes, to restore over the grey
    for o in ann.get("objects", []):
        props = o.get("properties", {})
        # With --mask-ignored, a sign we cannot label (ambiguous, or a speed limit of uncertain
        # unit) is painted out instead of merely unlabelled. Unlabelled but still visible, it
        # would be learned as BACKGROUND, teaching "small, unclear sign = not a sign"
        # (docs/experiments/2026-09-26-sign-detector.md, gap 2). Dummies stay background on
        # purpose: they are sign-like things that are not signs.
        if mask_ignored and (props.get("ambiguous") or o["label"] in mask_labels):
            b = o["bbox"]
            if "cross_boundary" not in b:
                masked.append((b["xmin"], b["ymin"], b["xmax"], b["ymax"]))
            continue
        if props.get("ambiguous") or props.get("dummy"):
            continue
        cid = class_map.get(o["label"])
        if cid is None:
            continue
        b = o["bbox"]
        if "cross_boundary" in b:
            continue
        x0, y0 = max(0.0, b["xmin"]), max(0.0, b["ymin"])
        x1, y1 = min(float(W), b["xmax"]), min(float(H), b["ymax"])
        if x1 - x0 < 1 or y1 - y0 < 1:
            continue
        lines.append(f"{cid} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} "
                     f"{(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}")
        kept.append(cid)
        kept_px.append((x0, y0, x1, y1))
    if done_img.exists():
        # Image already re-saved by an earlier run on partial data: only the labels are
        # rewritten (cheap), so a mapping change always reaches every label file.
        done_lbl.write_text("\n".join(lines) + ("\n" if lines else ""))
        return key, "ok", kept
    img = cv2.imread(str(img_path))
    if img is None:
        return key, "badimg", []
    h, w = img.shape[:2]
    if (w, h) != (W, H):
        # Normalised YOLO coordinates are resolution independent, but a mismatch means the JSON
        # does not describe this file. Report it rather than guessing.
        return key, f"size-mismatch {w}x{h} vs {W}x{H}", []
    if masked:
        painted = img.copy()
        for (x0, y0, x1, y1) in masked:
            # 20% margin: annotation boxes are tight, and a sign's edge left visible is still a
            # partial sign (10% visibly left slivers on small distant signs).
            mx, my = 0.2 * (x1 - x0), 0.2 * (y1 - y0)
            cv2.rectangle(painted, (int(max(0, x0 - mx)), int(max(0, y0 - my))),
                          (int(min(w, x1 + mx)), int(min(h, y1 + my))), MASK_GREY, -1)
        for (x0, y0, x1, y1) in kept_px:  # never erase a sign we ARE training on
            ix0, iy0, ix1, iy1 = int(x0), int(y0), int(x1 + 1), int(y1 + 1)
            painted[iy0:iy1, ix0:ix1] = img[iy0:iy1, ix0:ix1]
        img = painted
    s = MAX_SIDE / max(w, h)
    if s < 1:
        img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(done_img), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    # An image with no kept boxes still gets an (empty) label file: a background image. It is
    # written last, so an interrupted run never leaves a label without its image.
    done_lbl.write_text("\n".join(lines) + ("\n" if lines else ""))
    return key, "ok", kept


def find_root(mtsd: Path) -> Path:
    for p in [mtsd, *mtsd.glob("*")]:
        if (p / "annotations").is_dir() and (p / "splits").is_dir():
            return p
    sys.exit(f"no annotations/ + splits/ under {mtsd}")


def find_images(mtsd: Path) -> dict[str, Path]:
    return {p.stem: p for p in mtsd.rglob("*.jpg") if "mtsd_yolo" not in str(p)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mtsd", type=Path, required=True)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--labels-only", action="store_true")
    ap.add_argument("--us-speed-variants", type=Path,
                    help="JSON list of full MTSD labels (with --gN) that are US mph signs")
    ap.add_argument("--map-out", type=Path, default=Path(__file__).with_name("mtsd_class_map.json"))
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--mask-ignored", action="store_true",
                    help="paint ambiguous and uncertain-unit signs grey instead of leaving them "
                         "visible but unlabelled (use a fresh --out: existing images are reused)")
    args = ap.parse_args()

    root = find_root(args.mtsd)
    anns = {p.stem: p for p in (root / "annotations").glob("*.json")}
    print(f"{len(anns)} annotation files under {root}")

    # Real label list, with instance counts, straight from the data.
    label_counts, flags = Counter(), Counter()
    with ProcessPoolExecutor(args.workers) as ex:
        for ann in ex.map(load_json, anns.values(), chunksize=256):
            for o in ann.get("objects", []):
                label_counts[o["label"]] += 1
                for k, v in o.get("properties", {}).items():
                    if v:
                        flags[k] += 1
    print(f"{len(label_counts)} distinct labels, {sum(label_counts.values())} boxes; flags: "
          f"{dict(flags)}")
    if args.labels_only:
        for lab, n in sorted(label_counts.items()):
            print(f"{n:7d}  {lab}")
        return

    us = set(json.loads(args.us_speed_variants.read_text())) if args.us_speed_variants else set()
    class_map = {lab: map_label(lab, us) for lab in sorted(label_counts)}
    args.map_out.write_text(json.dumps({
        "classes": CLASSES,
        "dropped_categories": ["information--*", "complementary--*", "other-sign"],
        "dropped_mph_or_unclear_speed_variants": sorted(us),
        "map": {k: v for k, v in class_map.items()},
    }, indent=1) + "\n")
    print(f"wrote {args.map_out}")

    images = find_images(args.mtsd)
    print(f"{len(images)} images found on disk")
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        (args.out / sub).mkdir(parents=True, exist_ok=True)

    jobs = []
    for split in ("train", "val"):
        keys = (root / "splits" / f"{split}.txt").read_text().split()
        present = [k for k in keys if k in images and k in anns]
        print(f"{split}: {len(keys)} listed, {len(present)} with image + annotation on disk")
        jobs += [(k, split, anns[k], images[k], args.out, class_map, us, args.mask_ignored)
                 for k in present]

    counts = {"train": Counter(), "val": Counter()}
    status = Counter()
    with ProcessPoolExecutor(args.workers) as ex:
        for i, (key, st, kept) in enumerate(ex.map(convert_one, jobs, chunksize=16)):
            split = jobs[i][1]
            status[st.split()[0]] += 1
            if st != "ok" and not st.startswith(("pano", "noimg")):
                print(f"  {key}: {st}")
            counts[split].update(kept)
            if i % 2000 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    print(f"status: {dict(status)}")

    print(f"\n{'id':>3} {'class':24} {'train':>7} {'val':>6}")
    for i, n in enumerate(CLASSES):
        flag = "  <-- few" if counts["train"][i] < 100 else ""
        print(f"{i:3d} {n:24} {counts['train'][i]:7d} {counts['val'][i]:6d}{flag}")
    (args.out / "instance_counts.json").write_text(json.dumps(
        {s: {CLASSES[i]: counts[s][i] for i in range(len(CLASSES))} for s in counts}, indent=1))

    (args.out / "mtsd.yaml").write_text(
        f"# MTSD (fully annotated) mapped to {len(CLASSES)} sign classes by "
        f"host/scripts/prepare_mtsd.py\n"
        f"path: {args.out.resolve()}\ntrain: images/train\nval: images/val\n"
        f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASSES)))
    print(f"wrote {args.out / 'mtsd.yaml'}")


if __name__ == "__main__":
    main()
