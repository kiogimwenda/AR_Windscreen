"""host/scripts/run_lidar_camera_calibration.py on a synthetic recording with a known LiDAR-camera
transform (synthetic.py renders the camera; the LiDAR sees the same board, a wall, the ground and
the person holding it). Run: python3 -m pytest host/test/scripts"""

import csv
import os
import sys

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation, Slerp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts"))

import run_lidar_camera_calibration as lc  # noqa: E402
import synthetic as syn  # noqa: E402

PATTERN, SQUARE = (7, 5), 0.10
BOARD_WH = (1.0, 0.8)  # the whole board, margin included (8 x 6 squares + 0.1 m)
SIZE = (1280, 720)  # the 2K camera's view, at a size that renders quickly
K = np.array([[1000.0, 0, 641.0], [0, 1000.0, 359.0], [0, 0, 1]])
D0 = np.zeros(5)


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


# Camera axes: x right, y down, z forward. LiDAR (vehicle-like): x forward, y left, z up; 0.6 m
# above and 0.4 m behind the camera, pitched 15 deg down (the roof mount, Part 4.8.6).
AXES = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)  # camera-from-LiDAR axis swap
T_TRUE = T_of(AXES @ Rotation.from_euler("y", 15, degrees=True).as_matrix(),
              np.array([0.02, 0.6, -0.4]))


def perturbed(T, deg, cm, seed=0):
    rng = np.random.default_rng(seed)
    axis = rng.normal(size=3)
    dR = Rotation.from_rotvec(axis / np.linalg.norm(axis) * np.radians(deg)).as_matrix()
    dt = rng.normal(size=3)
    return T_of(dR @ T[:3, :3], T[:3, 3] + dt / np.linalg.norm(dt) * cm / 100)


POSES = [  # board centre in the camera frame (m), tilt about x and y (deg)
    ((-1.2, 0.3, 5.0), 25, -30), ((1.3, 0.2, 5.5), -20, 30), ((0.0, 0.0, 4.0), 0, 0),
    ((-0.6, 0.5, 7.0), 35, 20), ((0.8, -0.2, 6.0), -30, -25), ((0.2, 0.4, 3.5), 15, 40),
    ((-1.5, 0.1, 6.5), -10, 35)]


def lidar_scan(rng, board, T_cl, people=True):
    """Points in the LiDAR frame: the board (sparse, as a Mid-360's 100 ms), the person behind it,
    the ground and a wall."""
    R, t = board
    pts = []
    n = 120
    u = rng.uniform(-BOARD_WH[0] / 2, BOARD_WH[0] / 2, n)
    v = rng.uniform(-BOARD_WH[1] / 2, BOARD_WH[1] / 2, n)
    cols, rows = PATTERN
    centre = t + R @ np.array([(cols - 1) * SQUARE / 2, (rows - 1) * SQUARE / 2, 0])
    pc = centre + np.outer(u, R[:, 0]) + np.outer(v, R[:, 1])
    pc += rng.normal(0, 0.02, pc.shape) * (pc / np.linalg.norm(pc, axis=1)[:, None])  # range noise
    pts.append(pc)
    if people:  # the holder, 0.5 m behind the board
        pts.append(centre + R[:, 2] * 0.5 + rng.normal(0, 0.15, (60, 3)) * [1, 3, 1])
    T_lc = np.linalg.inv(T_cl)
    out = [p @ T_lc[:3, :3].T + T_lc[:3, 3] for p in pts]
    ground = np.column_stack([rng.uniform(2, 15, 300), rng.uniform(-6, 6, 300),
                              np.full(300, -1.8) + rng.normal(0, 0.02, 300)])
    wall = np.column_stack([np.full(200, 12.0), rng.uniform(-6, 6, 200), rng.uniform(-1.8, 2, 200)])
    return np.concatenate(out + [ground, wall])


def make_recording(d, T_cl, start_guess, fps=10, hold_s=1.6, move_frames=4, seed=0):
    rng = np.random.default_rng(seed)
    os.makedirs(os.path.join(d, "lidar"))
    boards = [syn.board_pose(PATTERN, SQUARE, c, rx, ry) for c, rx, ry in POSES]
    seq = []  # board pose for each frame (R, t, still)
    for k, b in enumerate(boards):
        seq += [(b, True)] * int(hold_s * fps)
        if k + 1 < len(boards):  # moving to the next: interpolated, not still
            nb = boards[k + 1]
            for j in range(1, move_frames + 1):
                a = j / (move_frames + 1)
                R = Slerp([0, 1], Rotation.from_matrix([b[0], nb[0]]))(a).as_matrix()
                seq.append(((R, (1 - a) * b[1] + a * nb[1]), False))
    vw = cv2.VideoWriter(os.path.join(d, "camera.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, SIZE)
    with open(os.path.join(d, "camera.csv"), "w") as fc, open(os.path.join(d, "lidar.csv"), "w") as fl:
        fc.write("frame,t_us\n")
        fl.write("scan,t_us\n")
        cache = {}
        for i, (b, _) in enumerate(seq):
            key = (b[1].tobytes(), b[0].tobytes())
            if key not in cache:  # a held pose is rendered once; each frame gets its own noise
                cache[key] = syn.render_board(SIZE, K, D0, b[0], b[1], PATTERN, SQUARE,
                                              noise=0).astype(float)
            img = np.clip(cache[key] + rng.normal(0, 2.0, cache[key].shape), 0, 255).astype(np.uint8)
            vw.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            t_us = i * 100000
            fc.write(f"{i},{t_us}\n")
            p = lidar_scan(rng, b, T_cl)
            np.column_stack([p, np.full(len(p), 50.0)]).astype(np.float32).tofile(
                os.path.join(d, "lidar", f"{i:06d}.bin"))
            fl.write(f"{i},{t_us + 50000}\n")
    vw.release()
    with open(os.path.join(d, "camera_intrinsics.yaml"), "w") as f:
        f.write(f"image_width: {SIZE[0]}\nimage_height: {SIZE[1]}\n"
                f"camera_matrix: [{', '.join(str(v) for v in K.reshape(-1))}]\n"
                "distortion_coefficients: [0, 0, 0, 0, 0]\nreprojection_error_px: 0.0\n")
    T_cv = T_of(AXES, np.array([0.0, 1.3, -1.5]))  # camera 1.3 m up, 1.5 m ahead of the axle
    T_vl = np.linalg.inv(T_cv) @ start_guess
    with open(os.path.join(d, "extrinsics.yaml"), "w") as f:
        for key, T in (("camera_from_lidar", start_guess), ("vehicle_from_lidar", T_vl),
                       ("camera_from_vehicle", T_cv)):
            f.write(f"{key}: [{', '.join(f'{v:.9f}' for v in T.reshape(-1))}]\n")
    with open(os.path.join(d, "recording.yaml"), "w") as f:
        f.write(f"name: synthetic\nwidth: {SIZE[0]}\nheight: {SIZE[1]}\n")


def angle_deg(A, B):
    return np.degrees(np.linalg.norm(Rotation.from_matrix(A[:3, :3] @ B[:3, :3].T).as_rotvec()))


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("rec") / "calib")
    make_recording(d, T_TRUE, perturbed(T_TRUE, 3.0, 10.0))
    return d


def test_recovers_the_known_transform(recording, tmp_path):
    out = tmp_path / "lidar_camera_extrinsics.yaml"
    rc = lc.main([recording, "--checkerboard", "7x5", "--square-size-mm", "100",
                  "--board-size-mm", "1000x800", "--output", str(out)])
    assert rc == 0
    y = yaml.safe_load(open(out))
    T = T_of(np.array(y["rotation_matrix"]).reshape(3, 3), np.array(y["translation_m"]))
    # Started 3 deg and 10 cm off. What a plane-based board calibration achieves is limited by the
    # camera's own estimate of each board's plane (a few tenths of a degree at 4-7 m): about 0.4 deg
    # here, within the +-1 deg ExtrinsicMonitor (Part 12.2.1) refines from the driving scene. The
    # rotation and translation errors are coupled (a turn seen at 5 m looks like a shift), so the
    # test is what fusion uses: where a LiDAR point lands in the image, from 5 to 40 m.
    rot, trans = angle_deg(T, T_TRUE), np.linalg.norm(T[:3, 3] - T_TRUE[:3, 3])
    worst = 0.0
    T_lc = np.linalg.inv(T_TRUE)
    for r_m in (5, 10, 20, 40):
        pc = np.array([[x, y, r_m] for x in np.linspace(-0.5, 0.5, 5) * r_m
                       for y in (-0.2 * r_m, 0, 0.1 * r_m)])
        pl = pc @ T_lc[:3, :3].T + T_lc[:3, 3]
        got = pl @ T[:3, :3].T + T[:3, 3]
        worst = max(worst, np.linalg.norm(K[0, 0] * (got[:, :2] / got[:, 2:] - pc[:, :2] / pc[:, 2:]),
                                          axis=1).max())
    print(f"error: {rot:.2f} deg, {100 * trans:.1f} cm; in the image {worst:.1f} px (f {K[0, 0]:.0f} px)")
    assert rot < 0.5
    assert worst < 8.0
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)  # the host checks it is a rotation


def test_uses_only_still_poses(recording):
    r = lc.calibrate(recording, PATTERN, SQUARE, BOARD_WH, log=lambda *_: None)
    assert r["poses"] == len(POSES)


def test_boards_all_facing_one_way_are_refused(tmp_path):
    """Parallel planes leave the translation along the board unconstrained: refused."""
    global POSES
    saved = POSES
    POSES = [((x, y, 5.0), 0, 0) for x, y in ((-1, 0), (1, 0), (0, 0.4), (0.5, -0.3), (-0.5, 0.3))]
    try:
        d = str(tmp_path / "flat")
        make_recording(d, T_TRUE, perturbed(T_TRUE, 2.0, 5.0, seed=1), seed=1)
        with pytest.raises(ValueError, match="tilt the board"):
            lc.calibrate(d, PATTERN, SQUARE, BOARD_WH, log=lambda *_: None)
    finally:
        POSES = saved
