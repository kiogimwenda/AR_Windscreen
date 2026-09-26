#!/usr/bin/env bash
# Build TensorRT engines from the exported ONNX models — see docs/BUILD_GUIDE.md Part 7.3.
#
# Run from host/:   scripts/build_tensorrt_engines.sh
#
# ---------------------------------------------------------------------------------------------
# What "building an engine" means
#
# An ONNX file says WHAT to compute. A TensorRT engine says HOW, on one specific GPU. During the
# build, trtexec:
#   - fuses layers (e.g. Conv + BatchNorm + ReLU become a single kernel launch);
#   - with --fp16, lets layers run in half precision. Tensor cores are much faster in FP16, and
#     TensorRT keeps a layer in FP32 where FP16 would lose too much accuracy;
#   - times several candidate CUDA kernels ("tactics") for every layer on THIS GPU and keeps the
#     fastest.
# The last step is why an engine is only valid on the GPU model (and TensorRT version) it was
# built with. Engines are gitignored and rebuilt per machine (Part 7.3).
#
# Each build is followed by trtexec's own benchmark of the engine, with its median GPU latency,
# so the Part 7.1 FPS targets can be checked in isolation before any application code exists.
# Build logs, including those numbers, are kept in models/engines/*.log.
#
# Tensor names and shapes must match scripts/export_onnx.py. The shapes differ from Part 7.3's
# for the lane and depth models (see docs/decisions.md, Phase 5).
# ---------------------------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p models/engines

build() {
    local name=$1 input=$2 shape=$3
    local onnx=models/onnx/${name}.onnx engine=models/engines/${name}.engine
    [[ -f $onnx ]] || { echo "missing $onnx — run scripts/export_onnx.py first" >&2; exit 1; }
    echo "== ${name}: building ${engine} (FP16, ${input}:${shape})"
    # No --shapes flag (Part 7.3 has one): every export here is fully static, and TensorRT rejects
    # --shapes for a static model. The expected shape is checked after the build instead, so a
    # re-export at a different size cannot slip through unnoticed.
    trtexec --onnx="$onnx" --saveEngine="$engine" --fp16 >"models/engines/${name}.log" 2>&1 \
        || { echo "trtexec failed for ${name}; see models/engines/${name}.log" >&2; exit 1; }
    local want=${shape//x/x}
    grep -qi "input binding for ${input} with dimensions ${want}" "models/engines/${name}.log" \
        || { echo "${name}: engine input is not ${input}:${want}; see its log" >&2; exit 1; }
    grep -E "Throughput:|GPU Compute Time:" "models/engines/${name}.log" | sed 's/^/   /'
}

build yolov8m images 1x3x736x1280
build ufld    input  1x3x320x1600
build midas   input  1x3x256x448

# Optional fourth model: the traffic-sign detector (YOLOv8s fine-tuned on MTSD by
# scripts/train_sign_detector.py, exported with `export_onnx.py --model signs`). It is built only
# if its ONNX file exists, so the three builds above behave exactly as before without it.
if [[ -f models/onnx/signs.onnx ]]; then
    build signs images 1x3x736x1280
fi

# Object detection with masks (YOLOv8m-seg; Part 7.1 amended 2026-09-25). Built if exported, beside
# the box-only yolov8m engine, until MlInferenceEngine has switched over to it.
if [[ -f models/onnx/yolov8m_seg.onnx ]]; then
    build yolov8m_seg images 1x3x736x1280
fi
