#!/usr/bin/env python3
"""Fine-tune YOLOv8s (COCO-pretrained) as a traffic-sign detector on MTSD.

    python train_sign_detector.py --data ../data/datasets/mtsd_yolo/mtsd.yaml --end 05:30
    python train_sign_detector.py --resume      # continue a crashed run from its last.pt
    python train_sign_detector.py --data ... --weights <best.pt> --name signs_v2 \
        --end none --epochs 60 --patience 15    # train until validation stops improving

This is the fourth model beside YOLOv8m, UFLDv2 and MiDaS. It exists because COCO has only one
sign class ("stop sign"), while the AR display must react differently to a stop sign, a no-left-turn
sign and a 50 km/h limit.

Settings that matter:
  imgsz=1280   signs are small, and the production input is 1280x736.
  fliplr=0.0   MANDATORY. A mirrored no-left-turn sign IS a no-right-turn sign, and keep-left
               becomes keep-right: horizontal-flip augmentation would teach the wrong meaning
               with the right-looking label. flipud stays at its default of 0.
  time=        training stops at a wall-clock deadline (--end, local time); ultralytics then
               rescales its learning-rate schedule to the number of epochs that fit.
               --end none disables it.
  patience=    early stopping: training ends once validation "fitness" (ultralytics weights it
               0.1*mAP50 + 0.9*mAP50-95) has not improved for this many consecutive epochs,
               i.e. "until it can no longer improve". --epochs is then only a safety cap, and it
               also sets the length of the learning-rate decay.
"""

import argparse
import datetime as dt
import sys
from pathlib import Path

HOST = Path(__file__).resolve().parents[1]
PROJECT = HOST / "models" / "training"
NAME = "signs_v1"


def hours_until(hhmm: str) -> float:
    now = dt.datetime.now()
    h, m = map(int, hhmm.split(":"))
    end = now.replace(hour=h, minute=m, second=0, microsecond=0)
    if end <= now:
        end += dt.timedelta(days=1)
    return (end - now).total_seconds() / 3600


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path)
    ap.add_argument("--weights", type=Path, default=HOST / "models" / "weights" / "yolov8s.pt")
    ap.add_argument("--end", default="05:30",
                    help="local wall-clock time training must end by, or 'none' for no limit")
    ap.add_argument("--name", default=NAME, help="run name under host/models/training/")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--patience", type=int, default=100)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--resume", action="store_true")
    args = ap.parse_args()

    from ultralytics import YOLO

    if args.resume:
        last = PROJECT / args.name / "weights" / "last.pt"
        if not last.exists():
            sys.exit(f"nothing to resume: {last} does not exist")
        YOLO(str(last)).train(resume=True)
        return

    hours = None if args.end.lower() == "none" else hours_until(args.end)
    if hours is None:
        print(f"no time limit: up to {args.epochs} epochs, early stop after "
              f"{args.patience} epochs without improvement")
    else:
        print(f"training for at most {hours:.2f} h (until {args.end})")
    model = YOLO(str(args.weights))
    model.train(
        data=str(args.data),
        imgsz=args.imgsz,
        epochs=args.epochs,  # cap; with a `time` limit, that ends training first
        time=hours,
        patience=args.patience,
        batch=args.batch,
        workers=args.workers,
        amp=True,
        fliplr=0.0,          # see the docstring: flipping changes a sign's meaning
        flipud=0.0,
        project=str(PROJECT),
        name=args.name,
        exist_ok=False,
        pretrained=True,
        plots=True,
        seed=0,
    )


if __name__ == "__main__":
    main()
