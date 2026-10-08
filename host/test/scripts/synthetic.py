"""Synthetic calibration scenes with a known answer, for the calibration tools' tests.

render_board(): a checkerboard at a known pose, seen through a known pinhole + distortion camera,
rendered exactly: each pixel's ray is undistorted (cv2.undistortPointsIter), intersected with the
board's plane, and the board's pattern sampled there, 2x2 supersampled so edges are antialiased as a
real sensor's are. Inner corner (0, 0) is the board frame's origin; x along the columns, y along the
rows, z out of the board's back (OpenCV's convention for calibrateCamera's object points).
"""

import math

import cv2
import numpy as np


def rotation(rx_deg, ry_deg, rz_deg):
    r = [math.radians(v) for v in (rx_deg, ry_deg, rz_deg)]
    Rx = np.array([[1, 0, 0], [0, math.cos(r[0]), -math.sin(r[0])], [0, math.sin(r[0]), math.cos(r[0])]])
    Ry = np.array([[math.cos(r[1]), 0, math.sin(r[1])], [0, 1, 0], [-math.sin(r[1]), 0, math.cos(r[1])]])
    Rz = np.array([[math.cos(r[2]), -math.sin(r[2]), 0], [math.sin(r[2]), math.cos(r[2]), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def board_pose(pattern, square_m, centre_cam, rx_deg, ry_deg, rz_deg=0.0):
    """Camera-from-board (R, t) with the board's centre at `centre_cam` (camera frame, metres)."""
    cols, rows = pattern
    R = rotation(rx_deg, ry_deg, rz_deg)
    middle = np.array([(cols - 1) * square_m / 2, (rows - 1) * square_m / 2, 0.0])
    return R, np.asarray(centre_cam, float) - R @ middle


def render_board(size, K, D, R, t, pattern, square_m, background=110, noise=2.0, rng=None):
    w, h = size
    offsets = [(-0.25, -0.25), (0.25, -0.25), (-0.25, 0.25), (0.25, 0.25)]
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    acc = np.zeros((h, w))
    n = R[:, 2]
    cols, rows = pattern
    for du, dv in offsets:
        px = np.stack([us + du, vs + dv], -1).reshape(-1, 1, 2)
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-9)
        xy = cv2.undistortPointsIter(px, K, D, None, None, crit).reshape(-1, 2)
        ray = np.column_stack([xy, np.ones(len(xy))])
        denom = ray @ n
        s = (n @ t) / np.where(np.abs(denom) < 1e-12, 1e-12, denom)
        pc = ray * s[:, None]
        pb = (pc - t) @ R  # R^T (pc - t), row-wise
        X, Y = pb[:, 0], pb[:, 1]
        i = np.floor(X / square_m).astype(int)
        j = np.floor(Y / square_m).astype(int)
        square = (i + j) % 2 == 0
        val = np.where(square, 20.0, 235.0)
        # The printed squares span -1..cols columns of squares; a white margin of one square more.
        on_squares = (i >= -1) & (i <= cols - 1) & (j >= -1) & (j <= rows - 1)
        on_card = (X >= -2 * square_m) & (X <= (cols + 1) * square_m) & \
                  (Y >= -2 * square_m) & (Y <= (rows + 1) * square_m)
        val = np.where(on_squares, val, np.where(on_card, 235.0, float(background)))
        val = np.where(s > 0, val, float(background))
        acc += val.reshape(h, w)
    img = acc / len(offsets)
    rng = rng or np.random.default_rng(0)
    img = img + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


# A wide-angle 1280x720 lens with clear barrel distortion: the kind the calibration must recover.
SIZE = (1280, 720)
K_TRUE = np.array([[900.0, 0, 652.0], [0, 905.0, 355.0], [0, 0, 1]])
D_TRUE = np.array([-0.28, 0.09, 0.0008, -0.0005, -0.012])


def calibration_views(pattern=(9, 6), square_m=0.025, n=18, seed=1):
    """`n` board poses spread over the image and tilted up to ~40 deg, as Part 12.1 asks."""
    rng = np.random.default_rng(seed)
    views = []
    # The first views go into the image's corners and edge middles (where the distortion is
    # largest and must be measured, not extrapolated), the rest anywhere.
    aims = [(0.14, 0.17), (0.86, 0.17), (0.14, 0.83), (0.86, 0.83),
            (0.5, 0.15), (0.5, 0.85), (0.12, 0.5), (0.88, 0.5)]
    cols, rows = pattern
    # The printed card's outline (squares plus a one-square margin): all of it must be in frame,
    # as a detector needs the whole board.
    card = np.array([[x, y, 0] for x in (-2 * square_m, (cols + 1) * square_m)
                     for y in (-2 * square_m, (rows + 1) * square_m)])
    for k in range(n):
        z = rng.uniform(0.35, 0.6)
        a = aims[k] if k < len(aims) else (rng.uniform(0.2, 0.8), rng.uniform(0.2, 0.8))
        rx, ry, rz = rng.uniform(-35, 35), rng.uniform(-35, 35), rng.uniform(-15, 15)
        # Pull the aim towards the centre until the whole card is in frame, 8 px clear.
        for f in np.linspace(1.0, 0.0, 21):
            u = (0.5 + f * (a[0] - 0.5)) * SIZE[0]
            v = (0.5 + f * (a[1] - 0.5)) * SIZE[1]
            x = (u - K_TRUE[0, 2]) / K_TRUE[0, 0] * z
            y = (v - K_TRUE[1, 2]) / K_TRUE[1, 1] * z
            R, t = board_pose(pattern, square_m, (x, y, z), rx, ry, rz)
            p, _ = cv2.projectPoints(card, cv2.Rodrigues(R)[0], t, K_TRUE, D_TRUE)
            p = p.reshape(-1, 2)
            if (p.min() >= 8 and (p[:, 0] <= SIZE[0] - 8).all() and (p[:, 1] <= SIZE[1] - 8).all()):
                break
        views.append((R, t))
    return views
