#!/usr/bin/env python3
"""TODO(Part 12.1): Camera intrinsic calibration.

Wraps cv2.calibrateCamera() over 20-30 checkerboard captures and writes focal length, principal
point and distortion coefficients to config/camera_intrinsics.yaml. Capture at the SAME
camera/resolution/lens setting the final system runs at — recalibrate if the resolution changes.

Filled in during Phase 6.
"""
