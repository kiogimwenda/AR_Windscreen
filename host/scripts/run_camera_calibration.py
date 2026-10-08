#!/usr/bin/env python3
"""Camera intrinsic calibration (docs/BUILD_GUIDE.md Part 12.1).

Finds a checkerboard in 20-30 photographs of it, solves the pinhole model with OpenCV's
5-coefficient distortion (k1 k2 p1 p2 k3), and writes host/config/camera_intrinsics.yaml, the file
CameraPipeline (Part 6) and SceneReconstruction (Part 8.3) read.

    # from photographs already taken (any image format OpenCV reads):
    python3 host/scripts/run_camera_calibration.py --images calib/ \\
        --checkerboard 9x6 --square-size-mm 25 --output host/config/camera_intrinsics.yaml

    # or take them from the camera, at the resolution in host/config/camera.yaml:
    python3 host/scripts/run_camera_calibration.py --capture /dev/video0 --images calib/ ...

--checkerboard counts INNER corners (where four squares meet), columns x rows: a board of 10 x 7
squares is 9x6. Print it flat on stiff card and measure a square with calipers: the size sets the
scale of nothing downstream (intrinsics are in pixels) but a wrong one makes the reported board
poses wrong. Capture at the resolution the system runs at (camera.yaml): intrinsics do not carry
across resolutions, and the script refuses a mismatch.

What makes a good set (the script checks the first two and prints the third):
  - coverage: corners over the whole image, edges and corners included, where distortion is largest;
  - tilt: the board tilted up to ~45 deg about both axes in many views, not only facing the camera
    (a set of frontal views cannot separate focal length from distance);
  - the per-view error: a view far above the others (motion blur, a bent board) is dropped and the
    solve repeated; the result's RMS error should be below ~0.5 px.
"""

import argparse
import datetime
import glob
import math
import os
import sys

import cv2
import numpy as np

IMAGE_TYPES = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")


def parse_pattern(text):
    try:
        cols, rows = (int(v) for v in text.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError("--checkerboard is COLSxROWS of inner corners, e.g. 9x6")
    if cols < 3 or rows < 3:
        raise argparse.ArgumentTypeError("--checkerboard needs at least 3x3 inner corners")
    return cols, rows


def board_points(pattern, square_m):
    cols, rows = pattern
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return pts


def find_corners(gray, pattern):
    """Sub-pixel inner corners, or None. The sector-based detector is more accurate and more
    tolerant of blur and lighting than the classic one; the classic one is the fallback."""
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE |
                                              cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:
        return corners.astype(np.float32)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH |
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)


def coverage(corner_sets, size, grid=(8, 6)):
    """Fraction of a grid of image cells containing at least one corner."""
    w, h = size
    hit = np.zeros(grid[::-1], bool)
    for c in corner_sets:
        p = c.reshape(-1, 2)
        gx = np.clip((p[:, 0] / w * grid[0]).astype(int), 0, grid[0] - 1)
        gy = np.clip((p[:, 1] / h * grid[1]).astype(int), 0, grid[1] - 1)
        hit[gy, gx] = True
    return hit.mean()


def view_errors(obj, img, K, D, rvecs, tvecs):
    errs = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, 1)))))
    return errs


def tilt_deg(rvec):
    """Angle between the board's normal and the camera's optical axis."""
    R, _ = cv2.Rodrigues(rvec)
    return math.degrees(math.acos(min(1.0, abs(R[2, 2]))))


def calibrate(views, pattern, square_m, size, max_view_error_px=1.0, log=print):
    """views: [(name, corners)]. Returns a dict with K, D, rms, the views used and dropped."""
    obj = [board_points(pattern, square_m) for _ in views]
    img = [c for _, c in views]
    names = [n for n, _ in views]
    dropped = []
    while True:
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None)
        errs = view_errors(obj, img, K, D, rvecs, tvecs)
        # Drop the single worst view if it is clearly out of line, and solve again.
        worst = int(np.argmax(errs))
        limit = max(max_view_error_px, 3.0 * float(np.median(errs)))
        if errs[worst] <= limit or len(img) <= 10:
            break
        log(f"  dropping {names[worst]}: {errs[worst]:.2f} px (limit {limit:.2f} px)")
        dropped.append((names[worst], errs[worst]))
        for lst in (obj, img, names):
            del lst[worst]
    return {"K": K, "D": D.reshape(-1)[:5], "rms": float(rms), "names": names, "errors": errs,
            "dropped": dropped, "tilts": [tilt_deg(r) for r in rvecs],
            "coverage": coverage(img, size)}


def write_yaml(path, result, size, pattern, square_mm):
    K, D = result["K"], result["D"]
    k = ", ".join(f"{v:.6f}" for v in K.reshape(-1))
    d = ", ".join(f"{v:.8f}" for v in D)
    when = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    with open(path, "w") as f:
        f.write(f"""# Produced by host/scripts/run_camera_calibration.py — see docs/BUILD_GUIDE.md Part 12.1.
# Do not hand-edit the matrix; regenerate by re-running the calibration, and recalibrate whenever
# the capture resolution changes.
# {when}: {len(result['names'])} views of a {pattern[0]}x{pattern[1]} board ({square_mm:g} mm squares),
# {len(result['dropped'])} dropped; image coverage {100 * result['coverage']:.0f} %, tilt up to
# {max(result['tilts']):.0f} deg.
image_width: {size[0]}
image_height: {size[1]}
camera_matrix: [{k}]
distortion_coefficients: [{d}]
reprojection_error_px: {result['rms']:.4f}
""")


def load_images(folder):
    files = sorted(f for t in IMAGE_TYPES for f in glob.glob(os.path.join(folder, t)))
    if not files:
        sys.exit(f"no images in {folder}")
    return files


def camera_size(config):
    import yaml  # PyYAML: only for reading camera.yaml
    with open(config) as f:
        c = yaml.safe_load(f)
    return int(c["width"]), int(c["height"]), c


def capture(source, folder, count, pattern, size, fps, preview):
    """Saves a view whenever the board is found, sharp, and placed differently from every saved
    view (moved by a tenth of the image, or a different size or tilt), until `count`."""
    os.makedirs(folder, exist_ok=True)
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source, cv2.CAP_V4L2
                           if source.startswith("/dev/") or source.isdigit() else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        sys.exit(f"cannot open {source}")
    saved = []  # (centre, span, aspect) of saved views
    n = len(load_images(folder)) if any(glob.glob(os.path.join(folder, t)) for t in IMAGE_TYPES) else 0
    print(f"capturing to {folder}: move the board slowly; {count - n} more views wanted (q to stop)")
    while n < count:
        ok, frame = cap.read()
        if not ok:
            sys.exit(f"{source}: no frame")
        if (frame.shape[1], frame.shape[0]) != size:
            sys.exit(f"{source} delivers {frame.shape[1]}x{frame.shape[0]}, not {size[0]}x{size[1]} "
                     "(camera.yaml): calibrate at the running resolution")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, None, fx=0.5, fy=0.5)
        c = find_corners(small, pattern)
        status = "no board"
        if c is not None:
            c = c * 2
            p = c.reshape(-1, 2)
            centre = p.mean(0) / size
            span = (p.max(0) - p.min(0)) / size
            sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
            new = all(np.linalg.norm(centre - s[0]) > 0.1 or abs(span[0] - s[1][0]) > 0.08 or
                      abs(span[0] / max(span[1], 1e-6) - s[2]) > 0.15 for s in saved)
            if sharp < 50:
                status = "blurred"
            elif not new:
                status = "already have this view"
            else:
                n += 1
                name = os.path.join(folder, f"view_{n:03d}.png")
                cv2.imwrite(name, frame)
                saved.append((centre, span, span[0] / max(span[1], 1e-6)))
                status = f"saved {name}"
                print(f"  {status}")
        if preview:
            show = cv2.resize(frame, None, fx=0.5, fy=0.5)
            if c is not None:
                cv2.drawChessboardCorners(show, pattern, c / 2, True)
            cv2.putText(show, f"{n}/{count} {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2)
            cv2.imshow("calibration", show)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cap.release()
    if preview:
        cv2.destroyAllWindows()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--images", required=True, help="folder of views (read, or written by --capture)")
    ap.add_argument("--checkerboard", type=parse_pattern, default=(9, 6),
                    help="inner corners, COLSxROWS (default 9x6)")
    ap.add_argument("--square-size-mm", type=float, default=25.0)
    ap.add_argument("--output", default="host/config/camera_intrinsics.yaml")
    ap.add_argument("--camera-config", default="host/config/camera.yaml",
                    help="the running resolution is taken from here and enforced")
    ap.add_argument("--capture", metavar="SOURCE", help="take the views from this camera first")
    ap.add_argument("--count", type=int, default=25, help="views to capture")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--min-views", type=int, default=12)
    ap.add_argument("--min-coverage", type=float, default=0.6)
    ap.add_argument("--max-rms-px", type=float, default=1.0)
    a = ap.parse_args(argv)

    size = None
    fps = 30
    if a.camera_config and os.path.exists(a.camera_config):
        w, h, cfg = camera_size(a.camera_config)
        size, fps = (w, h), int(cfg.get("fps", 30))
    if a.capture:
        if not size:
            sys.exit("--capture needs --camera-config for the resolution")
        capture(a.capture, a.images, a.count, a.checkerboard, size, fps, not a.no_preview)

    views = []
    for f in load_images(a.images):
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if im is None:
            print(f"  {f}: not readable, skipped")
            continue
        s = (im.shape[1], im.shape[0])
        if size is None:
            size = s
        if s != size:
            sys.exit(f"{f} is {s[0]}x{s[1]}, but the camera runs at {size[0]}x{size[1]}: "
                     "intrinsics do not carry across resolutions")
        c = find_corners(im, a.checkerboard)
        print(f"  {os.path.basename(f)}: {'board found' if c is not None else 'NO BOARD, skipped'}")
        if c is not None:
            views.append((os.path.basename(f), c))
    if len(views) < a.min_views:
        sys.exit(f"only {len(views)} views with the board (need {a.min_views}): take more")

    r = calibrate(views, a.checkerboard, a.square_size_mm / 1000, size)
    K, D = r["K"], r["D"]
    print(f"\n{len(r['names'])} views used, {len(r['dropped'])} dropped")
    print(f"fx {K[0, 0]:.2f}  fy {K[1, 1]:.2f}  cx {K[0, 2]:.2f}  cy {K[1, 2]:.2f} px")
    print("k1 k2 p1 p2 k3 = " + " ".join(f"{v:+.5f}" for v in D))
    print(f"RMS reprojection error {r['rms']:.3f} px (worst view {max(r['errors']):.3f} px)")
    print(f"image coverage {100 * r['coverage']:.0f} %; tilt {min(r['tilts']):.0f}-{max(r['tilts']):.0f} deg")
    hfov = 2 * math.degrees(math.atan(size[0] / (2 * K[0, 0])))
    print(f"horizontal field of view {hfov:.1f} deg (compare with the lens's datasheet)")

    problems = []
    if r["coverage"] < a.min_coverage:
        problems.append(f"the corners cover only {100 * r['coverage']:.0f} % of the image "
                        f"(need {100 * a.min_coverage:.0f} %): add views near the edges and corners")
    if max(r["tilts"]) < 20:
        problems.append("no view is tilted more than 20 deg: add tilted views")
    if r["rms"] > a.max_rms_px:
        problems.append(f"RMS error {r['rms']:.2f} px is above {a.max_rms_px} px: blur, a bent "
                        "board, or a wrong --checkerboard")
    if problems:
        print("\nNOT WRITTEN:\n  " + "\n  ".join(problems))
        return 1
    write_yaml(a.output, r, size, a.checkerboard, a.square_size_mm)
    print(f"\nwritten: {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
