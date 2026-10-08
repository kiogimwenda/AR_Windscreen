#!/usr/bin/env python3
"""LiDAR-to-camera extrinsic calibration (docs/BUILD_GUIDE.md Part 12.2).

From a recording (`ar_drive_assist --record DIR`, system/Recorder.h) in which a checkerboard is
held STILL, for a second or more each, in several poses in front of the car, solve the rigid
transform from the LiDAR frame into the camera frame and write host/config/lidar_camera_extrinsics.yaml.

    python3 host/scripts/run_lidar_camera_calibration.py calib_recording/ \\
        --checkerboard 9x6 --square-size-mm 80 --board-size-mm 900x700 \\
        --output host/config/lidar_camera_extrinsics.yaml

Method: board planes, not picked corners. A Mid-360 scans in a non-repeating pattern, so no single
LiDAR point lands on a corner; but over a held second its points cover the board's surface.
  1. Camera: the board's corners in each frame (the recording's frames and camera model), its pose
     by solvePnP; still periods are where the corners move less than `--still-px` for at least
     `--still-s`. One board plane per still period (the median pose).
  2. LiDAR: every scan within a still period, accumulated. The points on the board are picked with
     the current estimate of the transform (at first the recording's own, from the hand-measured
     mount): within the board's outline grown by `--margin-m`, and within `--margin-m` of its plane;
     the picking is repeated, tighter, after each solve.
  3. Solve: the rotation and translation that put every picked LiDAR point on its pose's camera
     plane (point-to-plane least squares, robust to stray points). A plane fixes three of the six
     numbers; boards tilted different ways fix the rest. The script refuses a set of poses that
     leaves any direction unconstrained ("tilt the board more ways"), and reports the residual
     per pose: a pose far worse than the rest (the board moved, or was held off its outline) is
     dropped and the solve repeated.

Holding the board: a rigid board (foam board, not card), the checkerboard printed on it with a
plain margin, 3-8 m from the car, in 6-10 poses: left, right, near, far, and tilted forward, back and
to each side by 20-40 deg. Nothing else within `--margin-m` of the board (stand behind it, arms
below its edge). --board-size-mm is the whole board, margin included.

The camera-to-vehicle transform (camera_extrinsics.yaml) is measured by hand (Part 12.2) and is
not changed here; the LiDAR's place on the car follows from the two.
"""

import argparse
import csv
import datetime
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_camera_calibration import board_points, find_corners, parse_pattern  # noqa: E402


def parse_size(text):
    try:
        w, h = (float(v) for v in text.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError("--board-size-mm is WIDTHxHEIGHT, e.g. 900x700")
    return w / 1000, h / 1000


def matrix16(node, key, path):
    v = node.get(key)
    if not v or len(v) != 16:
        sys.exit(f"{path}: {key} needs 16 numbers")
    return np.array(v, float).reshape(4, 4)


def load_recording(d):
    with open(os.path.join(d, "camera_intrinsics.yaml")) as f:
        ci = yaml.safe_load(f)
    K = np.array(ci["camera_matrix"], float).reshape(3, 3)
    D = np.array(ci["distortion_coefficients"], float)
    with open(os.path.join(d, "extrinsics.yaml")) as f:
        ex = yaml.safe_load(f)
    T_cl = matrix16(ex, "camera_from_lidar", "extrinsics.yaml")
    T_cv = matrix16(ex, "camera_from_vehicle", "extrinsics.yaml")
    frame_t = [int(r["t_us"]) for r in csv.DictReader(open(os.path.join(d, "camera.csv")))]
    scans = [(int(r["scan"]), int(r["t_us"])) for r in csv.DictReader(open(os.path.join(d, "lidar.csv")))]
    return K, D, T_cl, T_cv, frame_t, scans


def read_scan(d, index):
    pts = np.fromfile(os.path.join(d, "lidar", f"{index:06d}.bin"), np.float32).reshape(-1, 4)
    return pts[:, :3].astype(float)


def board_poses(d, K, D, frame_t, pattern, square_m, stride, log):
    """[(t_us, R, t, corners)] for every `stride`-th frame where the board is found."""
    cap = cv2.VideoCapture(os.path.join(d, "camera.avi"))
    obj = board_points(pattern, square_m)
    out = []
    i = 0
    while True:
        ok, img = cap.read()
        if not ok or i >= len(frame_t):
            break
        if i % stride == 0:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            c = find_corners(gray, pattern)
            if c is not None:
                ok, rvec, tvec = cv2.solvePnP(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE)
                if ok:
                    rvec, tvec = cv2.solvePnPRefineLM(obj, c, K, D, rvec, tvec)
                    out.append((frame_t[i], cv2.Rodrigues(rvec)[0], tvec.reshape(3), c.reshape(-1, 2)))
        i += 1
    log(f"camera: board found in {len(out)} of {len(range(0, i, stride))} frames examined")
    return out


def still_periods(poses, still_px, still_s):
    """Runs of consecutive detections whose corners stay within still_px of the run's first."""
    periods, run = [], []
    for p in poses:
        if run and (np.abs(p[3] - run[0][3]).max() > still_px or p[0] - run[-1][0] > 0.5e6):
            if run[-1][0] - run[0][0] >= still_s * 1e6:
                periods.append(run)
            run = []
        run.append(p)
    if run and run[-1][0] - run[0][0] >= still_s * 1e6:
        periods.append(run)
    return periods


def plane_of(period, square_m, pattern):
    """The board's plane (unit normal n, offset d: n.x = d, camera frame), its centre and axes."""
    rv = np.median([cv2.Rodrigues(p[1])[0].reshape(3) for p in period], axis=0)
    t = np.median([p[2] for p in period], axis=0)
    R = cv2.Rodrigues(rv)[0]
    n = R[:, 2]
    cols, rows = pattern
    centre = t + R @ np.array([(cols - 1) * square_m / 2, (rows - 1) * square_m / 2, 0])
    return {"n": n, "d": float(n @ t), "centre": centre, "R": R,
            "t0": period[0][0], "t1": period[-1][0], "frames": len(period)}


def pick(points_l, T, plane, board_wh, margin):
    """LiDAR points (LiDAR frame) on the board: inside its outline + margin, near its plane."""
    pc = points_l @ T[:3, :3].T + T[:3, 3]
    rel = pc - plane["centre"]
    local = rel @ plane["R"]  # board axes: x along columns, y along rows, z the normal
    w, h = board_wh
    keep = (np.abs(local[:, 0]) <= w / 2 + margin) & (np.abs(local[:, 1]) <= h / 2 + margin) & \
           (np.abs(local[:, 2]) <= margin)
    return points_l[keep]


def residuals(x, sets):
    R = Rotation.from_rotvec(x[:3]).as_matrix()
    return np.concatenate([(p @ R.T + x[3:]) @ pl["n"] - pl["d"] for pl, p in sets])


def solve(sets, T0):
    x0 = np.concatenate([Rotation.from_matrix(T0[:3, :3]).as_rotvec(), T0[:3, 3]])
    r = least_squares(residuals, x0, args=(sets,), loss="huber", f_scale=0.02)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(r.x[:3]).as_matrix()
    T[:3, 3] = r.x[3:]
    # How well the poses constrain the six numbers: the Jacobian's singular values (per point).
    s = np.linalg.svd(r.jac, compute_uv=False) / np.sqrt(max(len(r.fun), 1))
    return T, s


def calibrate(d, pattern, square_m, board_wh, margin=0.3, still_px=1.5, still_s=1.0, stride=1,
              min_poses=4, min_points=60, log=print, T_init=None):
    K, D, T_cl0, T_cv, frame_t, scans = load_recording(d)
    T0 = T_cl0 if T_init is None else T_init
    poses = board_poses(d, K, D, frame_t, pattern, square_m, stride, log)
    planes = [plane_of(p, square_m, pattern) for p in still_periods(poses, still_px, still_s)]
    log(f"{len(planes)} still board poses")
    clouds = []
    for pl in planes:
        idx = [i for i, t in scans if pl["t0"] <= t <= pl["t1"]]
        clouds.append(np.concatenate([read_scan(d, i) for i in idx]) if idx else np.zeros((0, 3)))
    T, dropped = T0.copy(), set()
    for m in (margin, margin / 2, max(margin / 4, 0.05), max(margin / 4, 0.05)):
        sets = []
        for k, (pl, cl) in enumerate(zip(planes, clouds)):
            if k in dropped:
                continue
            p = pick(cl, T, pl, board_wh, m)
            if len(p) >= min_points:
                sets.append((pl, p))
        if len(sets) < min_poses:
            raise ValueError(f"only {len(sets)} board poses with {min_points}+ LiDAR points on the "
                             f"board (need {min_poses}): hold it still longer, nearer, or check that "
                             f"the recording's starting extrinsics are within {margin} m")
        T, sv = solve(sets, T)
        # Per pose, the RMS distance of its points from its plane; drop one clearly out of line.
        per = [float(np.sqrt(np.mean(((p @ T[:3, :3].T + T[:3, 3]) @ pl["n"] - pl["d"]) ** 2)))
               for pl, p in sets]
        worst = int(np.argmax(per))
        if per[worst] > max(0.03, 3 * float(np.median(per))) and len(sets) > min_poses:
            k = planes.index(sets[worst][0])
            log(f"  dropping pose {k}: {100 * per[worst]:.1f} cm from its plane")
            dropped.add(k)
    if sv.min() < 0.05:
        raise ValueError("the board poses do not constrain every direction (smallest singular value "
                         f"{sv.min():.3f}): tilt the board more ways (forward, back, both sides) and "
                         "place it at different heights and sides")
    return {"T": T, "T0": T0, "T_cv": T_cv, "per_pose_rms": per, "points": [len(p) for _, p in sets],
            "poses": len(sets), "dropped": sorted(dropped), "rms": float(np.sqrt(np.mean(
                np.concatenate([((p @ T[:3, :3].T + T[:3, 3]) @ pl["n"] - pl["d"]) ** 2
                                for pl, p in sets])))), "singular": sv}


def write_yaml(path, r):
    T = r["T"]
    R = ", ".join(f"{v:.9f}" for v in T[:3, :3].reshape(-1))
    t = ", ".join(f"{v:.5f}" for v in T[:3, 3])
    when = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    with open(path, "w") as f:
        f.write(f"""# Rigid transform from the LiDAR frame into the camera frame — see docs/BUILD_GUIDE.md Part 12.2.
# Produced by host/scripts/run_lidar_camera_calibration.py from board planes; do not hand-edit.
# {when}: {r['poses']} board poses, {sum(r['points'])} LiDAR points on the board.
# Consumed by SceneReconstruction (Part 8.3) and RoadSurfaceProjector (Part 11.4), and the
# baseline ExtrinsicMonitor (Part 12.2.1) verifies at every start.
rotation_matrix: [{R}]            # 3x3, row-major
translation_m: [{t}]
plane_rms_m: {r['rms']:.4f}       # LiDAR board points from the camera's board planes
""")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording")
    ap.add_argument("--checkerboard", type=parse_pattern, default=(9, 6))
    ap.add_argument("--square-size-mm", type=float, required=True)
    ap.add_argument("--board-size-mm", type=parse_size, required=True,
                    help="the whole board, margin included, WIDTHxHEIGHT (along columns x rows)")
    ap.add_argument("--output", default="host/config/lidar_camera_extrinsics.yaml")
    ap.add_argument("--margin-m", type=float, default=0.3,
                    help="how far the starting extrinsics may be off (first picking margin)")
    ap.add_argument("--still-px", type=float, default=1.5)
    ap.add_argument("--still-s", type=float, default=1.0)
    ap.add_argument("--stride", type=int, default=1, help="examine every Nth frame")
    ap.add_argument("--max-rms-m", type=float, default=0.03)
    a = ap.parse_args(argv)
    try:
        r = calibrate(a.recording, a.checkerboard, a.square_size_mm / 1000, a.board_size_mm,
                      a.margin_m, a.still_px, a.still_s, a.stride)
    except ValueError as e:
        print(f"NOT WRITTEN: {e}")
        return 1
    T, T0 = r["T"], r["T0"]
    dR = np.degrees(np.linalg.norm(Rotation.from_matrix(T[:3, :3] @ T0[:3, :3].T).as_rotvec()))
    dt = np.linalg.norm(T[:3, 3] - T0[:3, 3])
    print(f"\n{r['poses']} poses used ({len(r['dropped'])} dropped), LiDAR points per pose {r['points']}")
    print("distance from the board planes per pose (cm): " +
          " ".join(f"{100 * v:.1f}" for v in r["per_pose_rms"]))
    print(f"RMS {100 * r['rms']:.2f} cm; change from the starting extrinsics: {dR:.2f} deg, "
          f"{100 * dt:.1f} cm")
    T_vl = np.linalg.inv(r["T_cv"]) @ T
    print("LiDAR in the vehicle frame (with camera_extrinsics.yaml): position "
          f"{np.round(T_vl[:3, 3], 3).tolist()} m, roll/pitch/yaw "
          f"{np.round(Rotation.from_matrix(T_vl[:3, :3]).as_euler('xyz', degrees=True), 1).tolist()} deg"
          " (compare with the mount's drawing and an inclinometer: Part 12.2.2)")
    if r["rms"] > a.max_rms_m:
        print(f"NOT WRITTEN: RMS {100 * r['rms']:.1f} cm is above {100 * a.max_rms_m:.0f} cm")
        return 1
    write_yaml(a.output, r)
    print(f"written: {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
