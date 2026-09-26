#!/usr/bin/env python3
"""Rewrite a CSV with its columns padded into alignment, so it reads as a table in any editor.

    python3 host/scripts/align_csv.py host/models/training/signs_v2/results.csv

The output is still a valid CSV (commas kept, values right-aligned with spaces). Tools that trim
whitespace (pandas with skipinitialspace=True, host/scripts/show_results.py) read it unchanged.
The original is kept beside it as <name>.raw.csv, byte for byte. Running it again is harmless:
existing padding is stripped before re-aligning, and an existing .raw.csv is never overwritten.

Do NOT run it on a results.csv that ultralytics is still writing. The trainer reads the file back
with polars every epoch (to embed it in checkpoints), where padded values can parse as text.
"""

import csv
import shutil
import sys
from pathlib import Path


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    path = Path(sys.argv[1])
    raw = path.with_suffix(".raw.csv")
    if not raw.exists():
        shutil.copy2(path, raw)  # keep the machine-format original

    with open(path, newline="") as f:
        rows = [[cell.strip() for cell in row] for row in csv.reader(f) if row]
    widths = [max(len(r[i]) for r in rows if i < len(r)) for i in range(len(rows[0]))]
    with open(path, "w", newline="") as f:
        for r in rows:
            f.write(", ".join(cell.rjust(w) for cell, w in zip(r, widths)) + "\n")
    print(f"aligned {len(rows) - 1} rows x {len(widths)} columns -> {path} (original: {raw})")


if __name__ == "__main__":
    main()
