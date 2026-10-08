#!/usr/bin/env python3
"""Converts a KITTI raw "synced" drive into a host recording (host/include/ar_drive_assist/system/
Recording.h), so the whole pipeline can be replayed on real, synchronised camera + LiDAR + GPS/IMU
data before the project's own sensors exist (Part 13.2).

    python3 tools/bench_rig/kitti_to_recording.py data/datasets/kitti/2011_09_26 \
        2011_09_26_drive_0005_sync data/recordings/kitti_0005

KITTI raw data: Geiger, Lenz, Stiller, Urtasun, "Vision meets Robotics: The KITTI Dataset",
IJRR 2013. Licence CC BY-NC-SA 3.0 (non-commercial; credit the authors in anything shown).
https://www.cvlibs.net/datasets/kitti/raw_data.php

What maps to what:
  camera   image_02 (left colour camera, RECTIFIED: no lens distortion), encoded to MJPG like a UVC
           camera's stream; P_rect_02 gives the intrinsics
  LiDAR    velodyne_points (Velodyne HDL-64E, 10 Hz, x forward / y left / z up): copied as is.
           A Velodyne, not the project's Livox Mid-360: denser rings, 360 deg, no tilt. Good for
           exercising the pipeline, not for judging the Mid-360's coverage
  hub      oxts (OXTS RT3003 GPS/IMU, 10 Hz) -> the hub's SensorReport fields: position, forward
           speed (vf) as both GNSS and OBD speed, acceleration (af, al, au, gravity included,
           in g), rates (wf, wl, wu, in deg/s), compass heading from yaw
  extrinsics  camera_from_lidar = rectified cam 2 from calib_velo_to_cam + R_rect_00 + P_rect_02.
           vehicle_from_lidar: the Velodyne 1.73 m above the ground, ~1.9 m ahead of the rear
           axle (the KITTI setup drawing; an estimate that shifts every reported gap by the same
           constant)
"""

import math
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

VELO_HEIGHT_M = 1.73
VELO_AHEAD_OF_REAR_AXLE_M = 1.9


def read_calib(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        try:
            out[k.strip()] = np.array([float(x) for x in v.split()])
        except ValueError:
            pass
    return out


def times_us(path):
    stamps = [datetime.strptime(l.strip()[:26], "%Y-%m-%d %H:%M:%S.%f")
              for l in Path(path).read_text().splitlines() if l.strip()]
    t0 = stamps[0]
    return [int(round((t - t0).total_seconds() * 1e6)) for t in stamps]


def mat4(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def flat(m):
    return "[" + ", ".join(f"{x:.9g}" for x in m.reshape(-1)) + "]"


def main(day_dir, drive, out_dir):
    day, out = Path(day_dir), Path(out_dir)
    src = day / drive
    out.mkdir(parents=True, exist_ok=True)
    (out / "lidar").mkdir(exist_ok=True)

    # --- calibration ---------------------------------------------------------------------------
    cc = read_calib(day / "calib_cam_to_cam.txt")
    vc = read_calib(day / "calib_velo_to_cam.txt")
    P = cc["P_rect_02"].reshape(3, 4)
    fx, fy, cx, cy = P[0, 0], P[1, 1], P[0, 2], P[1, 2]
    tz = P[2, 3]
    tx, ty = (P[0, 3] - cx * tz) / fx, (P[1, 3] - cy * tz) / fy
    w, h = (int(x) for x in cc["S_rect_02"])
    cam0_from_velo = mat4(vc["R"].reshape(3, 3), vc["T"])
    rect = mat4(cc["R_rect_00"].reshape(3, 3), np.zeros(3))
    cam_from_lidar = mat4(np.eye(3), [tx, ty, tz]) @ rect @ cam0_from_velo
    U, _, Vt = np.linalg.svd(cam_from_lidar[:3, :3])  # re-orthonormalise the printed values
    cam_from_lidar[:3, :3] = U @ Vt
    vehicle_from_lidar = mat4(np.eye(3), [VELO_AHEAD_OF_REAR_AXLE_M, 0, VELO_HEIGHT_M])
    cam_from_vehicle = cam_from_lidar @ np.linalg.inv(vehicle_from_lidar)

    (out / "camera_intrinsics.yaml").write_text(
        f"# KITTI {drive}, image_02 rectified (P_rect_02): no lens distortion.\n"
        f"image_width: {w}\nimage_height: {h}\n"
        f"camera_matrix: [{fx:.6f}, 0, {cx:.6f}, 0, {fy:.6f}, {cy:.6f}, 0, 0, 1]\n"
        f"distortion_coefficients: [0, 0, 0, 0, 0]\nreprojection_error_px: 0.0\n")
    (out / "extrinsics.yaml").write_text(
        "# 4x4 row-major. Vehicle frame: x forward, y left, z up, origin on the ground under\n"
        "# the rear axle. vehicle_from_lidar is an estimate from the KITTI setup drawing.\n"
        f"camera_from_lidar: {flat(cam_from_lidar)}\n"
        f"vehicle_from_lidar: {flat(vehicle_from_lidar)}\n"
        f"camera_from_vehicle: {flat(cam_from_vehicle)}\n")

    # --- camera -------------------------------------------------------------------------------
    cam_t = times_us(src / "image_02" / "timestamps.txt")
    with open(out / "camera.csv", "w") as f:
        f.write("frame,t_us\n")
        for i, t in enumerate(cam_t):
            f.write(f"{i},{t}\n")
    fps = (len(cam_t) - 1) / ((cam_t[-1] - cam_t[0]) * 1e-6)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{fps:.4f}", "-i",
                    str(src / "image_02" / "data" / "%010d.png"), "-c:v", "mjpeg", "-q:v", "2",
                    str(out / "camera.avi")], check=True)

    # --- LiDAR --------------------------------------------------------------------------------
    lid_t = times_us(src / "velodyne_points" / "timestamps.txt")
    bins = sorted((src / "velodyne_points" / "data").glob("*.bin"))
    with open(out / "lidar.csv", "w") as f:
        f.write("scan,t_us\n")
        for i, (b, t) in enumerate(zip(bins, lid_t)):
            dst = out / "lidar" / f"{i:06d}.bin"
            if dst.exists():
                dst.unlink()
            try:
                os.link(b, dst)  # same file system: no copy
            except OSError:
                shutil.copy(b, dst)
            f.write(f"{i},{t}\n")

    # --- hub reports from the OXTS ------------------------------------------------------------
    ox_t = times_us(src / "oxts" / "timestamps.txt")
    g = 9.80665
    with open(out / "hub.csv", "w") as f:
        f.write("t_us,lat,lon,speed_kph,gps_fix_valid,accel_x_g,accel_y_g,accel_z_g,"
                "gyro_x_dps,gyro_y_dps,gyro_z_dps,heading_deg,obd_speed_kph\n")
        for i, t in enumerate(ox_t):
            v = [float(x) for x in (src / "oxts" / "data" / f"{i:010d}.txt").read_text().split()]
            lat, lon, yaw = v[0], v[1], v[5]
            vf, af, al, au, wf, wl, wu = v[8], v[14], v[15], v[16], v[20], v[21], v[22]
            compass = (90.0 - math.degrees(yaw)) % 360.0  # yaw: CCW from east -> CW from north
            kph = vf * 3.6
            f.write(f"{t},{lat:.9f},{lon:.9f},{kph:.4f},1,{af / g:.5f},{al / g:.5f},{au / g:.5f},"
                    f"{math.degrees(wf):.4f},{math.degrees(wl):.4f},{math.degrees(wu):.4f},"
                    f"{compass:.3f},{kph:.4f}\n")

    def q(text):  # a YAML double-quoted string: these values contain ": "
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'

    duration = (cam_t[-1] - cam_t[0]) * 1e-6
    (out / "recording.yaml").write_text(
        f"name: {q('kitti_' + drive.replace('_sync', ''))}\n"
        f"source: {q(f'KITTI raw data, {drive} (https://www.cvlibs.net/datasets/kitti/raw_data.php)')}\n"
        f"licence: {q('CC BY-NC-SA 3.0. Geiger, Lenz, Stiller, Urtasun, Vision meets Robotics: The KITTI Dataset, IJRR 2013')}\n"
        f"width: {w}\nheight: {h}\n"
        f"notes: {q(f'{len(cam_t)} frames, {len(bins)} scans, {duration:.1f} s. Velodyne HDL-64E, not the project Livox Mid-360.')}\n")
    print(f"wrote {out}: {len(cam_t)} frames at {fps:.2f} fps, {len(bins)} scans, "
          f"{len(ox_t)} hub reports, camera {w}x{h}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
