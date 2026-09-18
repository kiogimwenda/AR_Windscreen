#!/usr/bin/env bash
# TODO(Part 7.3): Build TensorRT engines from the exported ONNX models with trtexec --fp16.
#
# trtexec builds for whatever GPU it is run on, so the resulting .engine files are valid ONLY on
# this machine — they are gitignored and must be regenerated on any new machine. No
# CUDA_ARCH_BIN-style flag applies here (that is an OpenCV build concern, Part 2.5).
#
# Filled in during Phase 5.
set -euo pipefail
echo "TODO: see docs/BUILD_GUIDE.md Part 7.3" >&2
exit 1
