#!/usr/bin/env python3
"""Extract the complete entries of a ZIP file that is still downloading.

    python extract_growing_zip.py <file.zip> <dest_dir> [--follow]

The MTSD image zips store their JPEGs uncompressed (method 0) with sizes in every local file
header, so each entry can be read front to back without the central directory at the end of the
file. This lets conversion start on the part already downloaded instead of waiting hours for the
whole file on a slow link. Entries already on disk with the right size are skipped, so the script
can be re-run (or left following the file with --follow) until the download finishes.
A final `unzip -t` / MD5 check of the finished file is still the proof of integrity.
"""

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

LOCAL = 0x04034B50


def scan(zpath: Path, dest: Path, start: int) -> tuple[int, int, bool]:
    """Extract every complete entry from byte offset `start`. Returns (next offset, n, finished)."""
    n = 0
    with open(zpath, "rb") as f:
        size = zpath.stat().st_size
        pos = start
        while pos + 30 <= size:
            f.seek(pos)
            hdr = f.read(30)
            sig, _, flag, method, _, _, crc, csize, usize, nlen, elen = struct.unpack(
                "<IHHHHHIIIHH", hdr)
            if sig != LOCAL:
                return pos, n, True  # central directory reached: all entries seen
            if flag & 0x08:
                sys.exit("entry uses a data descriptor; sizes unknown, cannot stream-extract")
            end = pos + 30 + nlen + elen + csize
            if end > size:
                break  # entry not fully downloaded yet
            name = f.read(nlen).decode()
            f.seek(elen, 1)
            data = f.read(csize)
            if not name.endswith("/"):
                out = dest / name
                if not (out.exists() and out.stat().st_size == usize):
                    if method == 8:
                        data = zlib.decompress(data, -15)
                    elif method != 0:
                        sys.exit(f"{name}: unsupported compression method {method}")
                    if zlib.crc32(data) != crc:
                        sys.exit(f"{name}: CRC mismatch")
                    out.parent.mkdir(parents=True, exist_ok=True)
                    tmp = out.with_suffix(out.suffix + ".part")
                    tmp.write_bytes(data)
                    tmp.rename(out)
                    n += 1
            pos = end
    return pos, n, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("zip", type=Path)
    ap.add_argument("dest", type=Path)
    ap.add_argument("--follow", action="store_true", help="keep extracting as the file grows")
    ap.add_argument("--done-flag", type=Path, help="with --follow: stop once this file exists "
                                                    "and everything has been extracted")
    args = ap.parse_args()
    pos, total = 0, 0
    while True:
        pos, n, finished = scan(args.zip, args.dest, pos)
        total += n
        print(f"{time.strftime('%H:%M:%S')} {args.zip.name}: offset {pos / 1e9:.2f} GB, "
              f"{total} files extracted", flush=True)
        if finished or not args.follow:
            break
        if args.done_flag and args.done_flag.exists() and pos >= args.zip.stat().st_size:
            break
        time.sleep(60)


if __name__ == "__main__":
    main()
