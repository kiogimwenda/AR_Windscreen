#!/usr/bin/env python3
"""Offline pipeline replay harness (docs/BUILD_GUIDE.md Part 13.2): the whole host, on a recorded
drive, with the simulated hub, checked end to end. Run from the repository root after building:

    python3 tools/bench_rig/replay_recorded_frames.py [--recording data/recordings/kitti_0005]
                                                      [--build build] [--skip-realtime]
                                                      [--jitter-seeds N]

A recording is a directory in the format of host/include/ar_drive_assist/system/Recording.h
(tools/bench_rig/kitti_to_recording.py makes one from KITTI). Two parts, each PASS/FAIL:

1. Deterministic (build/host/replay_inspect: every frame in lock-step, same input same output)
   - the drive as recorded: NO brake decision at all;
   - with a test obstacle stopped on the road ahead (lidar/InjectedObstacle.h: seen by the LiDAR
     and painted into the camera), appearing early and late in the drive:
       braking starts while the true time to collision is still >= 1.0 s,
       once started it does not lapse for longer than a brake command's hold (300 ms, so the hub
       never releases) while the true time to collision is above 0.5 s (closer, the obstacle
       drops into the LiDAR's near blind zone, and braking can no longer change the outcome),
       the measured gap is within 0.3 m of the truth on average,
       no brake while no obstacle is present.
   - with timing variations (replay_inspect --jitter, seeds 1..N): the recorded drive still makes
     NO brake decision, and no brake while no obstacle is present; the obstacle onsets are printed.
2. Real time (the threaded host, build/host/ar_drive_assist --replay, against hub_sim --replay,
   which runs the firmware's own SafetyCore): with the obstacle, the EventLog shows inference and
   fusion running, a BRAKE ActuationRequest, an AckStatus with the brake applied, no FAULT, the
   final zeroed command and a clean shutdown, and hub_sim reports a brake applied; without it, no
   BRAKE request.

Exit status 0 only if everything passes.
"""

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

results = []


def check(ok, what):
    results.append((ok, what))
    print(("PASS " if ok else "FAIL ") + what, flush=True)


def inspect(build, rec, out, obstacle=None, jitter=None):
    cmd = [f"{build}/host/replay_inspect", rec, "--no-video", "--out", out]
    if obstacle:
        cmd += ["--inject-obstacle", obstacle]
    if jitter is not None:
        cmd += ["--jitter", str(jitter)]
    subprocess.run(cmd, check=True, capture_output=True)
    return list(csv.DictReader(open(f"{out}/decisions.csv")))


def deterministic(build, rec, tmp):
    rows = inspect(build, rec, f"{tmp}/clean")
    brakes = [r["frame"] for r in rows if r["request"] == "BRAKE"]
    check(not brakes, f"recorded drive: no brake decision ({len(rows)} frames; brakes at {brakes})")

    duration = (int(rows[-1]["t_ms"]) - int(rows[0]["t_ms"])) / 1000
    deterministic.appear = (0.15 * duration, 0.7 * duration)
    for appear in deterministic.appear:
        rows = inspect(build, rec, f"{tmp}/obstacle", f"30,0,{appear:.1f}")
        name = f"obstacle appearing at {appear:.1f} s"
        first = next((r for r in rows if r["request"] == "BRAKE"), None)
        ahead = [r for r in rows if float(r["true_gap_m"]) > 0]
        if not first:
            check(False, f"{name}: braking starts")
            continue
        ttc = float(first["true_ttc_s"])
        check(ttc >= 1.0, f"{name}: braking starts with the true time to collision at {ttc:.2f} s (>= 1.0)")
        started = int(first["frame"])
        # Lapses: runs of non-brake frames longer than the 300 ms brake hold, before the end.
        lapses, run = [], []
        for r in ahead:
            if int(r["frame"]) <= started or float(r["true_ttc_s"]) < 0.5:
                continue
            if r["request"] == "BRAKE":
                if run and int(run[-1]["t_ms"]) - int(run[0]["t_ms"]) + 100 > 300:
                    lapses.append(f"{run[0]['frame']}-{run[-1]['frame']}")
                run = []
            else:
                run.append(r)
        if run and int(run[-1]["t_ms"]) - int(run[0]["t_ms"]) + 100 > 300:
            lapses.append(f"{run[0]['frame']}-{run[-1]['frame']}")
        check(not lapses, f"{name}: braking holds until the true time to collision is 0.5 s "
                          f"(lapses longer than the 300 ms hold: {lapses})")
        errors = [float(r["gap_m"]) - float(r["true_gap_m"]) for r in rows
                  if r["request"] == "BRAKE" and float(r["true_gap_m"]) > 0]
        mean = sum(errors) / len(errors) if errors else 0
        check(abs(mean) <= 0.3, f"{name}: measured gap within 0.3 m of the truth (mean error {mean:+.2f} m, "
                                f"worst {max(errors, key=abs) if errors else 0:+.2f} m)")
        phantom = [r["frame"] for r in rows if r["request"] == "BRAKE" and float(r["true_gap_m"]) <= 0]
        check(not phantom, f"{name}: no brake while no obstacle is present ({phantom})")


def jittered(build, rec, tmp, seeds, appear):
    """The drive with real-time timing variations (replay_inspect --jitter): the recorded drive must
    still never brake. With the obstacle, the onsets are reported, not checked: on this drive the
    obstacles stand in a bend and beside a real cyclist, where the margin depends on the scene as
    much as on the timing (progress log, 2026-10-07)."""
    brakes = {}
    for seed in seeds:
        rows = inspect(build, rec, f"{tmp}/jitter", jitter=seed)
        b = [r["frame"] for r in rows if r["request"] == "BRAKE"]
        if b:
            brakes[seed] = b
    check(not brakes, f"recorded drive, timing jitter seeds {seeds[0]}-{seeds[-1]}: no brake decision "
                      f"({brakes})")
    for a in appear:
        onsets = []
        for seed in seeds:
            rows = inspect(build, rec, f"{tmp}/jitter", f"30,0,{a:.1f}", seed)
            first = next((r for r in rows if r["request"] == "BRAKE"), None)
            onsets.append(round(float(first["true_ttc_s"]), 2) if first else None)
            phantom = [r["frame"] for r in rows if r["request"] == "BRAKE" and float(r["true_gap_m"]) <= 0]
            if phantom:
                check(False, f"obstacle at {a:.1f} s, jitter seed {seed}: no brake while no obstacle "
                             f"is present ({phantom})")
        print(f"INFO obstacle at {a:.1f} s, timing jitter: braking starts at true TTC {onsets}", flush=True)


def realtime(build, rec, tmp, obstacle, label):
    log = f"{tmp}/{label}.log"
    hub = subprocess.Popen([f"{build}/host/hub_sim", "--replay", f"{rec}/hub.csv"],
                           stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, text=True)
    device = None
    t0 = time.time()
    while device is None and time.time() - t0 < 10:
        m = re.search(r"/dev/pts/\d+", hub.stdout.readline())
        if m:
            device = m.group(0)
    if not device:
        hub.kill()
        check(False, f"{label}: hub_sim started")
        return
    cmd = [f"{build}/host/ar_drive_assist", "host/config", log, "--serial", device, "--replay", rec,
           "--no-display"]
    if obstacle:
        cmd += ["--inject-obstacle", obstacle]
    host = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Run until the LiDAR recording has ended (or a generous limit), then stop as Ctrl+C would.
    deadline = time.time() + 120
    while time.time() < deadline and host.poll() is None:
        if os.path.exists(log) and "end of LiDAR recording" in Path(log).read_text():
            break
        time.sleep(0.5)
    time.sleep(1.0)
    host.send_signal(signal.SIGINT)
    host.wait(timeout=30)
    hub.send_signal(signal.SIGINT)
    hub_out, _ = hub.communicate(timeout=10)
    text = Path(log).read_text()

    peak = re.search(r"peak brake intensity applied (\d+)", hub_out)
    peak = int(peak.group(1)) if peak else -1
    faults = [l for l in text.splitlines() if " FAULT " in l]
    brakes = len(re.findall(r"ACTUATION_REQUEST type=BRAKE", text))
    applied = re.findall(r"ACK_STATUS .*applied=(\d+)", text)
    check(host.returncode == 0, f"{label}: the host exits cleanly (status {host.returncode})")
    check("inference:" in text and "fusion:" in text, f"{label}: inference and fusion ran")
    check(not faults, f"{label}: no FAULT in the EventLog {faults[:2]}")
    check("final zeroed ActuationCommand sent" in text and "shutdown complete" in text,
          f"{label}: ordered shutdown with the final zeroed command")
    if obstacle:
        check(brakes > 0, f"{label}: BRAKE requests logged ({brakes})")
        check(any(int(a) > 0 for a in applied), f"{label}: the hub acknowledged a brake applied")
        check(peak > 0, f"{label}: hub_sim (the firmware's SafetyCore) applied the brake (peak {peak})")
    else:
        check(brakes == 0 and peak == 0, f"{label}: no brake requested or applied ({brakes}, peak {peak})")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--recording", default="data/recordings/kitti_0005")
    ap.add_argument("--build", default="build")
    ap.add_argument("--skip-realtime", action="store_true")
    ap.add_argument("--jitter-seeds", type=int, default=10,
                    help="timing-jitter replays of the recorded drive (0 = none)")
    a = ap.parse_args()
    if not Path(a.recording, "recording.yaml").exists():
        sys.exit(f"{a.recording} is not a recording (see tools/bench_rig/kitti_to_recording.py)")
    with tempfile.TemporaryDirectory() as tmp:
        print("--- deterministic (replay_inspect) ---", flush=True)
        deterministic(a.build, a.recording, tmp)
        if a.jitter_seeds:
            print("--- timing jitter (replay_inspect --jitter) ---", flush=True)
            jittered(a.build, a.recording, tmp, list(range(1, a.jitter_seeds + 1)),
                     deterministic.appear)
        if not a.skip_realtime:
            print("--- real time (threaded host + hub_sim) ---", flush=True)
            rows = list(csv.DictReader(open(Path(a.recording, "camera.csv"))))
            late = 0.7 * (int(rows[-1]["t_us"]) - int(rows[0]["t_us"])) / 1e6
            realtime(a.build, a.recording, tmp, f"30,0,{late:.1f}", "with obstacle")
            realtime(a.build, a.recording, tmp, None, "recorded drive")
    failed = [w for ok, w in results if not ok]
    print(f"\n{'ALL CHECKS PASS' if not failed else f'{len(failed)} FAILED'} ({len(results)} checks)")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
