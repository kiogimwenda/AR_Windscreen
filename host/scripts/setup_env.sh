#!/usr/bin/env bash
# TODO(Part 2.3-2.8): One-shot Debian/WSL2 dependency installer.
#
# Installs the base apt packages, PCL, OSRM tools and Livox SDK2. Does NOT install CUDA/TensorRT
# (Part 2.4 — use the versions already validated on this machine, never mix major versions) and
# does NOT build OpenCV (Part 2.5 — that is a long source build with CUDA_ARCH_BIN=12.0 for
# Blackwell, run deliberately, not as a side effect of a setup script).
#
# Filled in during Phase 0.
set -euo pipefail
echo "TODO: see docs/BUILD_GUIDE.md Part 2" >&2
exit 1
