"""host/scripts/run_camera_calibration.py on rendered checkerboards with a known lens
(synthetic.py). Run: python3 -m pytest host/test/scripts"""

import os
import sys

import cv2
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts"))

import run_camera_calibration as cal  # noqa: E402
import synthetic as syn  # noqa: E402

PATTERN, SQUARE = (9, 6), 0.025


@pytest.fixture(scope="module")
def views_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("views")
    for k, (R, t) in enumerate(syn.calibration_views(PATTERN, SQUARE, n=18)):
        img = syn.render_board(syn.SIZE, syn.K_TRUE, syn.D_TRUE, R, t, PATTERN, SQUARE,
                               rng=np.random.default_rng(k))
        cv2.imwrite(str(d / f"view_{k:03d}.png"), img)
    return d


def test_recovers_the_known_lens(views_dir, tmp_path):
    out = tmp_path / "intrinsics.yaml"
    rc = cal.main(["--images", str(views_dir), "--checkerboard", "9x6", "--square-size-mm", "25",
                   "--output", str(out), "--camera-config", ""])
    assert rc == 0
    y = yaml.safe_load(open(out))
    K = np.array(y["camera_matrix"]).reshape(3, 3)
    D = np.array(y["distortion_coefficients"])
    assert (y["image_width"], y["image_height"]) == syn.SIZE
    assert y["reprojection_error_px"] < 0.3
    # Focal lengths and principal point within 0.5 % / 3 px.
    assert abs(K[0, 0] / syn.K_TRUE[0, 0] - 1) < 0.005
    assert abs(K[1, 1] / syn.K_TRUE[1, 1] - 1) < 0.005
    assert abs(K[0, 2] - syn.K_TRUE[0, 2]) < 3 and abs(K[1, 2] - syn.K_TRUE[1, 2]) < 3
    # The coefficients trade off against each other; what matters is the mapping a LiDAR point
    # goes through. Two allowances, both real: (1) a principal point off by a pixel is nearly the
    # same as the camera turned by a twentieth of a degree, which the LiDAR-camera extrinsics
    # (Part 12.2, solved with these intrinsics) absorb, so the models are compared up to the best
    # small rotation; (2) only where the views' corners reached: a whole board cannot be seen in
    # the image's extreme corners, so the model is extrapolated there (printed, not asserted).
    views = []
    for f in sorted(views_dir.iterdir()):
        c = cal.find_corners(cv2.imread(str(f), cv2.IMREAD_GRAYSCALE), PATTERN)
        views.append(c.reshape(-1, 2))
    hull = cv2.convexHull(np.concatenate(views).astype(np.float32))
    g = np.stack(np.meshgrid(np.arange(0, syn.SIZE[0], 8.0), np.arange(0, syn.SIZE[1], 8.0)),
                 -1).reshape(-1, 2)
    inside = np.array([cv2.pointPolygonTest(hull, (float(x), float(y)), False) >= 0 for x, y in g])
    assert inside.mean() > 0.45, "the synthetic views must cover most of the image"
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-9)

    def rays(K_, D_):
        xy = cv2.undistortPointsIter(g.reshape(-1, 1, 2), K_, D_, None, None, crit).reshape(-1, 2)
        v = np.column_stack([xy, np.ones(len(xy))])
        return v / np.linalg.norm(v, axis=1)[:, None]

    true, got = rays(syn.K_TRUE, syn.D_TRUE), rays(K, D)
    U, _, Vt = np.linalg.svd(true[inside].T @ got[inside])
    R = (U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt).T  # best rotation true -> got
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    proj, _ = cv2.projectPoints(true @ R.T, np.zeros(3), np.zeros(3), K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - g, axis=1)
    print(f"rotation {angle:.3f} deg; covered region max {err[inside].max():.2f} px, "
          f"whole image max {err.max():.2f} px")
    assert angle < 0.2
    assert err[inside].max() < 0.5


def test_the_host_reads_what_it_writes(views_dir, tmp_path):
    """The file has the template's keys and shapes (CameraConfig.cpp checks 9 and 5 numbers)."""
    out = tmp_path / "intrinsics.yaml"
    assert cal.main(["--images", str(views_dir), "--output", str(out), "--camera-config", ""]) == 0
    y = yaml.safe_load(open(out))
    template = yaml.safe_load(open(os.path.join(HERE, "../../config/camera_intrinsics.yaml")))
    assert set(y) == set(template)
    assert len(y["camera_matrix"]) == 9 and len(y["distortion_coefficients"]) == 5


def test_refuses_a_resolution_other_than_the_running_one(views_dir, tmp_path):
    cfg = tmp_path / "camera.yaml"
    cfg.write_text("width: 1920\nheight: 1080\nfps: 30\n")
    with pytest.raises(SystemExit) as e:
        cal.main(["--images", str(views_dir), "--output", str(tmp_path / "x.yaml"),
                  "--camera-config", str(cfg)])
    assert "1920x1080" in str(e.value)


def test_frontal_views_only_are_not_written(tmp_path):
    """Views all facing the camera cannot separate focal length from distance: refused."""
    d = tmp_path / "frontal"
    d.mkdir()
    rng = np.random.default_rng(3)
    for k in range(14):
        z = rng.uniform(0.4, 0.6)
        u, v = rng.uniform(0.2, 0.8) * syn.SIZE[0], rng.uniform(0.25, 0.75) * syn.SIZE[1]
        R, t = syn.board_pose(PATTERN, SQUARE, ((u - 652) / 900 * z, (v - 355) / 905 * z, z),
                              rng.uniform(-5, 5), rng.uniform(-5, 5))
        cv2.imwrite(str(d / f"v{k:02d}.png"),
                    syn.render_board(syn.SIZE, syn.K_TRUE, syn.D_TRUE, R, t, PATTERN, SQUARE))
    out = tmp_path / "x.yaml"
    assert cal.main(["--images", str(d), "--output", str(out), "--camera-config", ""]) == 1
    assert not out.exists()


def test_a_bad_view_is_dropped():
    """A view whose corners are wrong (a bent board, a mis-detection) is dropped, and the solve
    repeated without it."""
    views = []
    for k, (R, t) in enumerate(syn.calibration_views(PATTERN, SQUARE, n=14, seed=5)):
        rvec, _ = cv2.Rodrigues(R)
        p, _ = cv2.projectPoints(cal.board_points(PATTERN, SQUARE), rvec, t, syn.K_TRUE, syn.D_TRUE)
        p = p.astype(np.float32) + np.random.default_rng(k).normal(0, 0.1, p.shape).astype(np.float32)
        views.append((f"v{k}", p))
    bent = views[4][1].copy()
    bent[: PATTERN[0]] += 6.0  # one row of corners 6 px off
    views[4] = ("bent", bent)
    r = cal.calibrate(views, PATTERN, SQUARE, syn.SIZE, log=lambda *_: None)
    assert [n for n, _ in r["dropped"]] == ["bent"]
    assert r["rms"] < 0.2
