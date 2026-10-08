#!/usr/bin/env python3
"""A demonstration video of the system as it stands: the whole host on a recorded drive, before any
hardware. Run from the repository root after building:

    python3 tools/bench_rig/make_demo_video.py [--recording data/recordings/kitti_0005]
                                               [--obstacle 30,0,11.0] [--out data/demo]

It runs build/host/replay_inspect --ar-video (every frame in lock-step through inference, fusion,
tracking and the decision, and the DRIVER'S view drawn by the real renderer, offscreen), then lays
out each frame:

    +-------------------------------------------------------------+
    |  the driver's view (ArRenderer + WindowedSink)              |
    +------------------------------+------------------------------+
    |  what the sensors see        |  the decision, this frame    |
    |  (LiDAR points, detections,  |  (rule, target, time to      |
    |   tracks: replay_inspect)    |   collision, BRAKE request)  |
    +------------------------------+------------------------------+
    |  timeline: the test obstacle's true gap; brake requests; now |
    +-------------------------------------------------------------+

Writes <out>/demo.mp4 (and the two source videos). Nothing here is staged: every overlay, number
and brake request is the system's own output for that frame.
"""

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

W = 1280  # output width
FONT = cv2.FONT_HERSHEY_SIMPLEX


def frames(path):
    cap = cv2.VideoCapture(str(path))
    while True:
        ok, img = cap.read()
        if not ok:
            return
        yield img


def text(img, s, org, scale=0.6, col=(235, 235, 235), th=1):
    cv2.putText(img, s, org, FONT, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, col, th, cv2.LINE_AA)


def panel(row, size, appear_s):
    """The decision for this frame, as the arbiter made it."""
    w, h = size
    img = np.full((h, w, 3), 24, np.uint8)
    brake = row["request"] == "BRAKE"
    t = (int(row["t_ms"]) - 1000) / 1000
    text(img, f"t = {t:4.1f} s     speed {float(row['ego_mps']) * 3.6:4.1f} km/h", (16, 32), 0.65)
    rules = {"1": "rule 1: forward collision", "2": "rule 2: hard braking -> hazards",
             "3": "rule 3: warning only", "4": "rule 4: nothing to do"}
    text(img, "Decision: " + rules.get(row["rule"], row["rule"]), (16, 68), 0.65)
    if row["target"] != "-1" and float(row["ttc_s"]) > 0:
        text(img, f"target track #{row['target']}: gap {float(row['gap_m']):.1f} m, closing "
                  f"{float(row['closing_mps']):.1f} m/s", (16, 100), 0.55)
        text(img, f"time to collision {float(row['ttc_s']):.2f} s  (brake below 1.8 s)", (16, 128), 0.55)
    if brake:
        cv2.rectangle(img, (16, 142), (w - 16, 186), (0, 0, 200), cv2.FILLED)
        text(img, f"BRAKE REQUEST  intensity {row['intensity']}", (30, 174), 0.8, (255, 255, 255), 2)
    elif float(row["true_gap_m"]) > 0:
        text(img, "test obstacle ahead (simulated, both sensors)", (16, 172), 0.55, (0, 200, 255))
    if float(row["true_gap_m"]) > 0:
        text(img, f"truth: gap {float(row['true_gap_m']):.1f} m, time to collision "
                  f"{float(row['true_ttc_s']):.2f} s", (16, 208), 0.5, (170, 170, 170))
    return img


def timeline(rows, i, size, appear_s):
    w, h = size
    img = np.full((h, w, 3), 16, np.uint8)
    n = len(rows)
    t0, t1 = int(rows[0]["t_ms"]), int(rows[-1]["t_ms"])
    x = lambda r: int(10 + (int(r["t_ms"]) - t0) / (t1 - t0) * (w - 20))
    maxgap = 30.0
    pts = [(x(r), int(h - 14 - min(float(r["true_gap_m"]), maxgap) / maxgap * (h - 34)))
           for r in rows if float(r["true_gap_m"]) > 0]
    if len(pts) > 1:
        cv2.polylines(img, [np.array(pts)], False, (0, 200, 255), 2, cv2.LINE_AA)
    for r in rows:
        if r["request"] == "BRAKE":
            cv2.line(img, (x(r), h - 12), (x(r), h - 4), (0, 0, 255), 3)
    cv2.line(img, (x(rows[i]), 4), (x(rows[i]), h - 2), (255, 255, 255), 1)
    text(img, "obstacle's true gap (orange), brake requests (red)", (14, 18), 0.45, (190, 190, 190))
    return img


class H264Writer:
    """Frames piped to ffmpeg's libx264 (pip's OpenCV has no H.264 encoder): an MP4 that plays in
    any browser or phone."""

    def __init__(self, path, w, h, fps):
        self.p = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
             "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             "-crf", "20", "-movflags", "+faststart", str(path)], stdin=subprocess.PIPE)

    def write(self, img):
        self.p.stdin.write(np.ascontiguousarray(img).tobytes())

    def release(self):
        self.p.stdin.close()
        self.p.wait()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--recording", default="data/recordings/kitti_0005")
    ap.add_argument("--obstacle", default="30,0,11.0", help="gap_m,speed_mps,appear_s ('' = none)")
    ap.add_argument("--build", default="build")
    ap.add_argument("--out", default="data/demo")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cmd = [f"{a.build}/host/replay_inspect", a.recording, "--ar-video", "--out", str(out)]
    if a.obstacle:
        cmd += ["--inject-obstacle", a.obstacle]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rows = list(csv.DictReader(open(out / "decisions.csv")))
    appear = float(a.obstacle.split(",")[2]) if a.obstacle else -1

    writer = None
    for i, (ar, ins) in enumerate(zip(frames(out / "ar.mp4"), frames(out / "inspect.mp4"))):
        if i >= len(rows):
            break
        top = cv2.resize(ar, (W, int(ar.shape[0] * W / ar.shape[1])))
        text(top, "Driver's view (AR renderer)", (12, top.shape[0] - 12), 0.55)
        half = W // 2
        sens = cv2.resize(ins, (half, int(ins.shape[0] * half / ins.shape[1])))
        text(sens, "What the sensors see: LiDAR + camera + tracks", (8, 18), 0.45)
        dec = panel(rows[i], (W - half, sens.shape[0]), appear)
        mid = np.hstack([sens, dec])
        line = timeline(rows, i, (W, 70), appear)
        frame = np.vstack([top, mid, line])
        if writer is None:
            writer = H264Writer(out / "demo.mp4", frame.shape[1], frame.shape[0], 10)
            title = frame.copy() // 4
            for k, s in enumerate([
                    "AR Windscreen: the system as it stands (software only, no hardware yet)",
                    "Recorded drive: KITTI raw 2011_09_26 drive 0005, Karlsruhe (Geiger et al.,",
                    "IJRR 2013, CC BY-NC-SA 3.0). Camera and LiDAR replayed through the whole host.",
                    f"At {appear:.0f} s a stopped test obstacle (grey box) is injected into BOTH sensors"
                    if appear > 0 else "No test obstacle: the drive as recorded.",
                    "Every overlay, number and brake request is the system's own output."]):
                text(title, s, (40, 120 + 44 * k), 0.75 if k == 0 else 0.62)
            for _ in range(40):
                writer.write(title)
        writer.write(frame)
        if rows[i]["request"] == "BRAKE" and (i == 0 or rows[i - 1]["request"] != "BRAKE"):
            for _ in range(15):  # hold the first brake frame for 1.5 s
                writer.write(frame)
    writer.release()
    print(f"wrote {out / 'demo.mp4'}")


if __name__ == "__main__":
    sys.exit(main())
