#!/usr/bin/env python3
"""Follow a training log and show YOLO's progress bar as ONE line that updates in place.

    python3 host/scripts/watch_training.py host/models/training/signs_v2_pipeline.log

The training writes each progress update as "\\r\\033[K<line>": a carriage return plus clear-line,
with no newline. `tail -f` passes that through, which redraws in place only when the line fits
the terminal. The progress line is ~130 characters wide. In a narrower window it wraps, and the
carriage return then only returns to the start of the LAST wrapped row, so every update leaves
the rows above it behind.

This watcher compacts each progress update (drops the bar graphic and padding) and cuts it to
the terminal's current width before redrawing, so it always stays on one row and still shows the
percentage, batch count and time remaining. Text that ends in a real newline (epoch validation
tables, messages) is printed in full, as normal. Ctrl+C stops watching; the training is
unaffected.
"""

import os
import re
import shutil
import sys
import time

CLEAR = re.compile(r"\x1b\[K")
BAR = re.compile(r"\s*[━─╸╺]+\s*")  # the progress-bar graphic
SPACES = re.compile(r" {2,}")


def compact(line: str) -> str:
    """'  1/60   7.19G   0.852 ... 1280: 24% ━━━╸─── 1083/4470 1.6it/s 10:09<36:09'
    -> '1/60 7.19G 0.852 ... 1280: 24% 1083/4470 1.6it/s 10:09<36:09'. The bar and padding go, so
    the percentage, batch count and time remaining survive in a narrow terminal."""
    return SPACES.sub(" ", BAR.sub(" ", line)).strip()


# Training progress lines have 11 fields once compacted:
#   epoch  gpu_mem  box_loss  cls_loss  dfl_loss  instances  size:  pct  batch  rate  elapsed<left
# They are printed in fixed-width columns under a matching header, so the numbers line up.
COLS = [("Epoch", 6), ("GPU", 6), ("box", 7), ("cls", 7), ("dfl", 7), ("Inst", 5), ("Size", 5),
        ("Done", 5), ("Batch", 10), ("Speed", 8), ("Elapsed<Left", 12)]
HEADER = " ".join(f"{name:>{w}}" for name, w in COLS)


def columns(line: str) -> str | None:
    """A training progress line in fixed-width columns, or None if it isn't one (for example a
    validation progress line, which is shown compacted instead)."""
    f = compact(line).split(" ")
    if len(f) != len(COLS) or "/" not in f[0]:
        return None
    f[6] = f[6].rstrip(":")
    return " ".join(f"{v:>{w}}" for v, (_, w) in zip(f, COLS))


def width() -> int:
    return max(20, shutil.get_terminal_size((120, 24)).columns - 1)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        f.seek(max(0, os.path.getsize(path) - 4000))  # start near the end, like tail
        f.readline() if f.tell() else None  # drop a partial first line
        pending = ""
        sys.stdout.write(HEADER + "\n")
        need_header = False
        while True:
            chunk = f.read()
            if not chunk:
                time.sleep(0.2)
                continue
            pending += chunk
            # Split into segments at \r or \n, keeping which separator ended each one.
            parts = re.split(r"(\r|\n)", pending)
            pending = parts.pop()  # the last piece may still be incomplete
            for text, sep in zip(parts[0::2], parts[1::2]):
                text = CLEAR.sub("", text)
                if "Epoch" in text and "GPU_mem" in text:
                    need_header = True  # the log's own header: replaced by ours, below
                    continue
                if sep == "\r":
                    # A progress update: redraw in place, cut to the terminal width.
                    row = columns(text)
                    if row is not None and need_header:
                        sys.stdout.write("\r\x1b[K" + HEADER + "\n")
                        need_header = False
                    shown = row if row is not None else compact(text)
                    sys.stdout.write("\r\x1b[K" + shown[: width()])
                else:
                    # A finished line: overwrite the progress line, then keep it. Anything
                    # printed in full (e.g. an epoch's validation table) scrolls the header away,
                    # so it is shown again before the next progress row.
                    if text.strip():
                        sys.stdout.write("\r\x1b[K" + text + "\n")
                        need_header = True
            sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
