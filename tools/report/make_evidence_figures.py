#!/usr/bin/env python3
"""Figures for the report's evidence pack (docs/report/evidence/). Run from the repository root,
after tools/bench_rig/make_demo_video.py has written data/demo/{obstacle,clean}:

    python3 tools/report/make_evidence_figures.py [--jitter-seeds 16]

Every figure is made from the system's own output on the replayed drive; nothing is drawn by hand.
KITTI-derived images are CC BY-NC-SA 3.0 (Geiger et al., IJRR 2013): credit them in the caption and
keep the report's use non-commercial.

  fig_ar_views.png        the driver's view, three moments of the obstacle run
  fig_demo_frame.png      one full demonstration frame (driver's view, sensors, decision, timeline)
  fig_brake_timeline.png  the obstacle run: true gap and time to collision, the threshold, brakes
  fig_jitter.png          (--jitter-seeds N) braking onsets and false brakes under timing jitter
  fig_calibration.png     synthetic calibration inputs: a rendered checkerboard view
"""

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT = Path("docs/report/evidence/figures")
DEMO = Path("data/demo")


def frame_at(video, n):
    cap = cv2.VideoCapture(str(video))
    img = None
    for _ in range(n):
        ok, img = cap.read()
        if not ok:
            return None
    return img


def rows(path):
    return list(csv.DictReader(open(path)))


def t_of(r):
    return (int(r["t_ms"]) - 1000) / 1000


def fig_ar_views(dec):
    first_brake = next(int(r["frame"]) for r in dec if r["request"] == "BRAKE")
    picks = [(60, "Normal driving: road users that are hazards glow; a sign banner"),
             (first_brake - 8, "Obstacle appeared, not yet a brake target: no barrier"),
             (first_brake + 4, "Braking: barrier at the obstacle with the measured gap")]
    fig, axes = plt.subplots(len(picks), 1, figsize=(10, 9.2))
    for ax, (n, cap) in zip(axes, picks):
        img = frame_at(DEMO / "obstacle" / "ar.mp4", n)
        ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        r = dec[n - 1]
        ax.set_title(f"t = {t_of(r):.1f} s: {cap}", fontsize=10)
        ax.axis("off")
    fig.text(0.5, 0.005, "KITTI raw 2011_09_26 drive 0005 (Geiger et al., IJRR 2013, CC BY-NC-SA "
             "3.0); grey box: simulated obstacle", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(OUT / "fig_ar_views.png", dpi=150)
    plt.close(fig)


def fig_demo_frame(dec):
    n = next(int(r["frame"]) for r in dec if r["request"] == "BRAKE") + 2
    cap = cv2.VideoCapture(str(DEMO / "obstacle" / "demo.mp4"))
    # demo.mp4 starts with a 40-frame title and holds the first brake frame for 15 frames.
    img = None
    for _ in range(40 + n + 15):
        ok, img = cap.read()
    cv2.imwrite(str(OUT / "fig_demo_frame.png"), img)


def fig_brake_timeline(dec):
    t = np.array([t_of(r) for r in dec])
    gap = np.array([float(r["true_gap_m"]) for r in dec])
    ttc = np.array([float(r["true_ttc_s"]) for r in dec])
    brake = np.array([r["request"] == "BRAKE" for r in dec])
    present = gap > 0
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    ax[0].plot(t[present], gap[present], color="tab:orange", label="true gap to the obstacle (m)")
    ax[0].set_ylabel("gap (m)")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[1].plot(t[present & (ttc > 0)], ttc[present & (ttc > 0)], color="tab:blue",
               label="true time to collision (s)")
    ax[1].axhline(1.8, color="grey", ls="--", lw=1, label="brake threshold 1.8 s")
    ax[1].axhline(1.0, color="grey", ls=":", lw=1, label="harness minimum at onset 1.0 s")
    for a in ax:
        for x in t[brake]:
            a.axvspan(x - 0.05, x + 0.05, color="red", alpha=0.25, lw=0)
    ax[1].set_ylim(0, 4)
    ax[1].set_xlabel("replay time (s); red: BRAKE requested")
    ax[1].set_ylabel("TTC (s)")
    ax[1].legend(loc="upper right", fontsize=8)
    onset = next(r for r in dec if r["request"] == "BRAKE")
    ax[0].set_title(f"Obstacle run: braking starts at true TTC {float(onset['true_ttc_s']):.2f} s "
                    f"(t = {t_of(onset):.1f} s)", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "fig_brake_timeline.png", dpi=150)
    plt.close(fig)


def fig_jitter(seeds, rec, build):
    """Runs replay_inspect --jitter for seeds 1..N: the drive as recorded and the obstacle at 11 s."""
    tmp = Path("data/demo/jitter")
    tmp.mkdir(parents=True, exist_ok=True)
    clean_brakes, onsets = [], []
    for s in range(1, seeds + 1):
        for obstacle, out in ((None, tmp / f"c{s}"), ("30,0,11.0", tmp / f"o{s}")):
            if not (out / "decisions.csv").exists():
                cmd = [f"{build}/host/replay_inspect", rec, "--no-video", "--jitter", str(s), "--out", str(out)]
                if obstacle:
                    cmd += ["--inject-obstacle", obstacle]
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            d = rows(out / "decisions.csv")
            if obstacle:
                f = next((r for r in d if r["request"] == "BRAKE"), None)
                onsets.append(float(f["true_ttc_s"]) if f else 0.0)
            else:
                clean_brakes.append(sum(r["request"] == "BRAKE" for r in d))
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    ax[0].bar(range(1, seeds + 1), clean_brakes, color="tab:red")
    ax[0].set_ylim(0, max(1, max(clean_brakes)) + 0.5)
    ax[0].set_title(f"Drive as recorded: brake requests per seed\n(total {sum(clean_brakes)} over "
                    f"{seeds} seeds)", fontsize=9)
    ax[0].set_xlabel("timing-jitter seed")
    ax[1].bar(range(1, seeds + 1), onsets, color="tab:blue")
    ax[1].axhline(1.0, color="grey", ls=":", lw=1)
    ax[1].axhline(1.8, color="grey", ls="--", lw=1)
    ax[1].set_title("Obstacle at 11 s: true TTC when braking starts", fontsize=9)
    ax[1].set_xlabel("timing-jitter seed")
    ax[1].set_ylabel("TTC (s)")
    fig.tight_layout()
    fig.savefig(OUT / "fig_jitter.png", dpi=150)
    plt.close(fig)
    return clean_brakes, onsets


def fig_calibration():
    sys.path.insert(0, "host/test/scripts")
    import synthetic as syn
    views = syn.calibration_views((9, 6), 0.025, n=4)
    imgs = [syn.render_board(syn.SIZE, syn.K_TRUE, syn.D_TRUE, R, t, (9, 6), 0.025) for R, t in views]
    fig, ax = plt.subplots(2, 2, figsize=(9, 5.4))
    for a, img in zip(ax.ravel(), imgs):
        a.imshow(img, cmap="gray")
        a.axis("off")
    fig.suptitle("Synthetic calibration views through a known lens (k1 = -0.28): the tests' "
                 "ground truth", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "fig_calibration.png", dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--jitter-seeds", type=int, default=0)
    ap.add_argument("--recording", default="data/recordings/kitti_0005")
    ap.add_argument("--build", default="build")
    a = ap.parse_args()
    if not (DEMO / "obstacle" / "decisions.csv").exists():
        sys.exit("run tools/bench_rig/make_demo_video.py --out data/demo/obstacle first")
    OUT.mkdir(parents=True, exist_ok=True)
    dec = rows(DEMO / "obstacle" / "decisions.csv")
    fig_ar_views(dec)
    fig_demo_frame(dec)
    fig_brake_timeline(dec)
    fig_calibration()
    if a.jitter_seeds:
        clean, onsets = fig_jitter(a.jitter_seeds, a.recording, a.build)
        print(f"jitter: clean brakes {clean}; onsets {[round(x, 2) for x in onsets]}")
    print(f"figures in {OUT}")


if __name__ == "__main__":
    main()
