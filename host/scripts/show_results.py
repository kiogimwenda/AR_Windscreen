#!/usr/bin/env python3
"""Print an ultralytics results.csv as a readable table.

    python3 host/scripts/show_results.py host/models/training/signs_v2/results.csv
    python3 host/scripts/show_results.py <results.csv> --markdown     # for docs and reports

Columns: epoch; training losses (box, cls, dfl); validation precision, recall, mAP50, mAP50-95;
validation losses; learning rate. Marks:
  *  best epoch so far, by ultralytics' fitness (0.1*mAP50 + 0.9*mAP50-95), the value that
     best.pt and early stopping use
  R  first epoch after a crash-resume (ultralytics' `time` counter restarts there, so less time
     is added than a real epoch takes)
  M  first epoch with mosaic augmentation switched off (the run's last 10 epochs, close_mosaic=10)
"""

import csv
import sys


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    markdown = "--markdown" in sys.argv
    if len(args) != 1:
        sys.exit(__doc__)
    rows = [{k.strip(): v.strip() for k, v in r.items()} for r in csv.DictReader(open(args[0]))]
    if not rows:
        sys.exit("no epochs yet")

    header = ["Ep", "box", "cls", "dfl", "P", "R", "mAP50", "mAP50-95", "v.box", "v.cls", "lr",
              "note"]
    table = []
    best_fit, prev_time = -1.0, -1.0
    steps = sorted(float(b["time"]) - float(a["time"]) for a, b in zip(rows, rows[1:]))
    typical = steps[len(steps) // 2] if steps else 0.0  # median epoch duration
    total = None
    for r in rows:
        ep = int(float(r["epoch"]))
        m50, m5095 = float(r["metrics/mAP50(B)"]), float(r["metrics/mAP50-95(B)"])
        fit = 0.1 * m50 + 0.9 * m5095
        notes = []
        if fit > best_fit:
            best_fit = fit
            notes.append("*")
        # ultralytics restarts its elapsed-time counter on resume. A resumed epoch therefore shows
        # less time added than a real epoch takes. "Time went backwards" alone misses a resume that
        # follows another one closely (the counter can be low but still above the previous row).
        t = float(r["time"])
        if prev_time >= 0 and (t - prev_time) < 0.5 * typical:
            notes.append("R")
        prev_time = t
        table.append([str(ep), f'{float(r["train/box_loss"]):.3f}', f'{float(r["train/cls_loss"]):.3f}',
                      f'{float(r["train/dfl_loss"]):.3f}', f'{float(r["metrics/precision(B)"]):.3f}',
                      f'{float(r["metrics/recall(B)"]):.3f}', f"{m50:.4f}", f"{m5095:.4f}",
                      f'{float(r["val/box_loss"]):.3f}', f'{float(r["val/cls_loss"]):.3f}',
                      f'{float(r["lr/pg0"]):.5f}', notes])

    # The mosaic-off epoch is known from the run's configured length (args.yaml next to the csv).
    try:
        import pathlib
        cfg = pathlib.Path(args[0]).with_name("args.yaml").read_text()
        epochs = int(next(l.split(":")[1] for l in cfg.splitlines() if l.startswith("epochs:")))
        close = int(next(l.split(":")[1] for l in cfg.splitlines() if l.startswith("close_mosaic:")))
        total = epochs
        for row in table:
            if int(row[0]) == epochs - close + 1:
                row[-1].append("M")
    except (OSError, StopIteration, ValueError):
        pass
    for row in table:
        row[-1] = " ".join(row[-1])

    if markdown:
        print("| " + " | ".join(header) + " |")
        print("|" + "|".join("---" for _ in header) + "|")
        for row in table:
            print("| " + " | ".join(row) + " |")
    else:
        widths = [max(len(header[i]), *(len(r[i]) for r in table)) for i in range(len(header))]
        print("  ".join(h.rjust(w) for h, w in zip(header, widths)))
        for row in table:
            print("  ".join(c.rjust(w) for c, w in zip(row, widths)))
    print(f"\n{len(table)} epochs{f' of {total}' if total else ''}; best fitness {best_fit:.4f}; "
          "* best so far, R resumed after a crash, M mosaic off")


if __name__ == "__main__":
    main()
