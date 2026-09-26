#!/usr/bin/env python3
"""Run the sign detector on a folder of images or on a video, save annotated copies and a CSV.

    python eval_sign_detector.py --weights models/training/signs_v2/weights/best.pt \
        --images ../data/footage/kenya --out ../data/footage/kenya_annotated
    python eval_sign_detector.py --weights ... --video ../data/footage/krakow_0-120s.webm \
        --every 30 --out ../data/footage/krakow_signs

There is no ground truth for this imagery, so the output is for visual audit: every detection at
or above --conf is drawn with its class and confidence, and listed in detections.csv. The
threshold is deliberately low (0.10) so that the audit can see where correct detections stop.
The model input is 1280 px on the long side, as in training.
"""

import argparse
import csv
from pathlib import Path

import cv2


def draw(img, boxes, names):
    for (x0, y0, x1, y1), c, p in boxes:
        col = (0, 255, 0) if p >= 0.5 else (0, 200, 255) if p >= 0.25 else (0, 0, 255)
        t = max(2, img.shape[1] // 800)
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), col, t)
        label = f"{names[c]} {p:.2f}"
        fs = max(0.6, img.shape[1] / 2000)
        y = max(int(y0) - 6, 20)
        cv2.putText(img, label, (int(x0), y), 0, fs, (0, 0, 0), t + 3)
        cv2.putText(img, label, (int(x0), y), 0, fs, col, t)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--images", type=Path)
    ap.add_argument("--video", type=Path)
    ap.add_argument("--every", type=int, default=30)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--imgsz", type=int, default=1280)
    args = ap.parse_args()
    from ultralytics import YOLO

    model = YOLO(args.weights)
    names = model.names
    args.out.mkdir(parents=True, exist_ok=True)

    def frames():
        if args.images:
            for p in sorted(args.images.iterdir()):
                if p.suffix.lower() in (".jpg", ".jpeg", ".png"):
                    im = cv2.imread(str(p))
                    if im is not None:
                        yield p.stem, im
        else:
            cap = cv2.VideoCapture(str(args.video))
            i = 0
            while True:
                ok, im = cap.read()
                if not ok:
                    break
                if i % args.every == 0:
                    yield f"frame_{i:05d}", im
                i += 1

    with open(args.out / "detections.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["image", "class_id", "class", "conf", "x0", "y0", "x1", "y1"])
        n_img = n_det = 0
        for stem, im in frames():
            r = model.predict(im, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
            boxes = [(b.xyxy[0].tolist(), int(b.cls), float(b.conf)) for b in r.boxes]
            for (x0, y0, x1, y1), c, p in boxes:
                w.writerow([stem, c, names[c], f"{p:.3f}", int(x0), int(y0), int(x1), int(y1)])
            cv2.imwrite(str(args.out / f"{stem}.jpg"), draw(im, boxes, names),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
            n_img += 1
            n_det += len(boxes)
    print(f"{n_img} images, {n_det} detections at conf >= {args.conf} -> {args.out}")


if __name__ == "__main__":
    main()
