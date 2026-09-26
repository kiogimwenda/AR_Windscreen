#!/usr/bin/env python3
"""Export a pretrained model to ONNX — see docs/BUILD_GUIDE.md Part 7.2, Part 7.3.

Run once per model. Pretrained weights only: Part 7.2 is explicit that the detector, the lane model
and the depth model are NOT trained from scratch for this project.

    python export_onnx.py --model yolov8m --weights models/weights/yolov8m.pt      --out models/onnx/yolov8m.onnx
    python export_onnx.py --model ufld    --weights models/weights/culane_res18.pth --out models/onnx/ufld.onnx
    python export_onnx.py --model midas   --out models/onnx/midas.onnx
    python export_onnx.py --model signs   --weights models/training/signs_v2/weights/best.pt --out models/onnx/signs.onnx
    python export_onnx.py --model yolov8m-seg --weights models/weights/yolov8m-seg.pt --out models/onnx/yolov8m_seg.onnx

Defaults: yolov8m, yolov8m-seg and signs 1280x736, midas 448x256 (width x height). --width/--height override
them for yolov8m, signs and midas. UFLD has no override: its fully-connected head is sized for
1600x320.

------------------------------------------------------------------------------------------------
What ONNX is, and why this step exists

PyTorch models are Python programs. TensorRT cannot run Python. ONNX is a file format that
records a model as a static graph of standard operators (Conv, MatMul, Resize, ...) together with
their weights. torch.onnx.export produces it by running the model once on an example input and
recording every operation that input passes through. That is why each export below takes a dummy
input of the exact shape the engine will use, and why every input shape here is FIXED. A fixed
shape lets TensorRT choose the fastest kernels for that one shape when it builds the engine
(Part 7.3).

Every export is verified before the script reports success. The same random input goes through
PyTorch and through ONNX Runtime, and the outputs must agree within a small tolerance. An export
that "succeeds" but silently changed the model's behaviour is a failure. That can happen: an
operator can be traced differently from how it runs in Python.

------------------------------------------------------------------------------------------------
Model choices (and where they differ from Part 7.1; see docs/decisions.md, Phase 5)

  yolov8m  COCO-pretrained YOLOv8m at 1280x736 (16:9, height rounded up to a multiple of 32).
           Output (1, 84, N): N candidate boxes (19320 at this size), each 4 box values + 80
           COCO class scores. Why 1280 and not 640 or the camera's native 2560 (the
           arithmetic is in docs/decisions.md, Phase 5): a 1.7 m pedestrian is ~1.39*W/d px tall
           at distance d, so 1280 keeps one >=12 px out to ~148 m, enough for ~100 km/h stopping
           plus tracking history. Close objects stay <=~900 px, inside the scale range YOLOv8 saw in
           training (640 input, up to 1.5x scale augmentation). Wider inputs make near objects
           larger than anything it learned, and near objects are what braking depends on. The five
           classes in Part 7.1 (vehicle, pedestrian, cyclist, sign, obstacle) are produced from
           COCO classes in C++ post-processing.
  ufld     Ultra-Fast-Lane-Detection-v2, CULane ResNet-18, 320x1600. Part 7.1's 800x288 is UFLD
           v1's input size; v2 ships no such model. CULane (urban, night, crowded, faded lines) is
           chosen over TuSimple (clear US highways) as the closer match to Nairobi roads.
  yolov8m-seg  COCO-pretrained YOLOv8m *segmentation* at 1280x736 (Part 7.1, amended 2026-09-25:
           hazard overlays follow object outlines). Two outputs:
             output0 (1, 4 + 80 + 32, N): per candidate, the box, 80 class scores and 32 mask
                     coefficients;
             output1 (1, 32, H/4, W/4): 32 "prototype" masks shared by the whole image.
           An object's mask = sigmoid(its 32 coefficients . the 32 prototypes), cropped to its box
           (the YOLACT approach). Decoding is in host/src/inference/Postprocess.cpp.
  signs    The traffic-sign detector (YOLOv8s fine-tuned on MTSD by scripts/train_sign_detector.py;
           the one model here that IS trained for this project). Same raw-Detect-head export as
           yolov8m, so the output is (1, 4 + 29, 19320) at 1280x736: 4 box values + 29 sign-class
           scores (class list: scripts/mtsd_class_map.json).
  midas    MiDaS v2.1 Small at 448x256 (16:9). MiDaS's own small-model transform limits the LONG
           side to 256 (256x128 for 16:9), so this is ~1.75x its standard scale: a moderate increase
           for sharper depth edges. Pushed much further past training scale, MiDaS is known to lose
           global consistency. Part 7.1's "v3.1 Small at 384x384" matches no published
           checkpoint, and v3.1's own small SwinV2 model does not load under any timm version that
           runs on Python 3.13. Output is RELATIVE inverse depth, not metres. Metric near-field
           depth comes from the LiDAR (Part 8).

Security: loading model code or a pickled checkpoint runs code from its author. The UFLD
checkpoint is loaded with torch.load(weights_only=True), which refuses to run embedded code.
YOLOv8m is loaded by the ultralytics package, which unpickles it. MiDaS comes from torch.hub,
which runs the MiDaS repository's code.
"""

import argparse
import os
import runpy
import sys
import types
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

OPSET = 17


def export(model: torch.nn.Module, dummy: torch.Tensor, out: Path, input_name: str,
           output_names: list[str]) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    with torch.no_grad():
        # dynamo=False selects the TorchScript-based exporter: the one TensorRT's ONNX parser has
        # been validated against longest, and the one whose output matches the pinned opset.
        torch.onnx.export(model, dummy, str(out), input_names=[input_name],
                          output_names=output_names, opset_version=OPSET, dynamo=False)


def verify(model: torch.nn.Module, out: Path, shape: tuple, input_name: str,
           rtol: float = 1e-3, atol: float = 1e-4) -> None:
    """Same random input through PyTorch and ONNX Runtime (CPU); every output must agree."""
    x = torch.rand(*shape)
    with torch.no_grad():
        ref = model(x)
    ref = [ref] if isinstance(ref, torch.Tensor) else list(ref)
    sess = ort.InferenceSession(str(out), providers=["CPUExecutionProvider"])
    got = sess.run(None, {input_name: x.numpy()})
    for i, (r, g) in enumerate(zip(ref, got)):
        r = r.numpy()
        worst = float(np.max(np.abs(r - g)))
        print(f"  output[{i}] {tuple(g.shape)}  max |torch - onnx| = {worst:.2e}")
        if not np.allclose(r, g, rtol=rtol, atol=atol):
            sys.exit(f"VERIFY FAILED: output[{i}] differs beyond rtol={rtol}, atol={atol}")
    print(f"  verified against PyTorch: {out}")


# --- YOLOv8m --------------------------------------------------------------------------------------
def export_yolov8m(weights: Path, out: Path, height: int, width: int) -> None:
    from ultralytics import YOLO

    yolo = YOLO(str(weights))
    model = yolo.model.float().eval()
    # ultralytics' own export() would also do this, but it adds post-processing options and writes
    # beside the weights. Exporting the raw network keeps the output format documented above.
    for m in model.modules():
        if hasattr(m, "export"):
            m.export = True  # Detect head: return the raw (1, 84, 8400) tensor, not a tuple
        if hasattr(m, "format"):
            m.format = "onnx"
    export(model, torch.zeros(1, 3, height, width), out, "images", ["output0"])
    verify(model, out, (1, 3, height, width), "images")


def export_yolov8m_seg(weights: Path, out: Path, height: int, width: int) -> None:
    """Same raw-head export as export_yolov8m. The Segment head in export mode returns two tensors
    (predictions + prototypes), so there are two named outputs, and both are verified."""
    from ultralytics import YOLO

    model = YOLO(str(weights)).model.float().eval()
    for m in model.modules():
        if hasattr(m, "export"):
            m.export = True
        if hasattr(m, "format"):
            m.format = "onnx"
    export(model, torch.zeros(1, 3, height, width), out, "images", ["output0", "output1"])
    verify(model, out, (1, 3, height, width), "images")


# --- UFLDv2 ---------------------------------------------------------------------------------------
class UfldOutputs(torch.nn.Module):
    """parsingNet returns a dict; ONNX outputs must be a fixed, named sequence."""

    def __init__(self, net: torch.nn.Module):
        super().__init__()
        self.net = net

    def forward(self, x):
        p = self.net(x)
        return p["loc_row"], p["loc_col"], p["exist_row"], p["exist_col"]


def export_ufld(weights: Path, out: Path, repo: Path) -> None:
    # UFLD's model file imports utils.common only for initialize_weights(), a random-weight
    # initialiser. utils.common in turn imports NVIDIA DALI, tensorboard and other training-only
    # packages. A stand-in module with a no-op initialize_weights is registered in its place.
    # That is safe because every weight is then overwritten by the checkpoint, and strict=True
    # below proves that no parameter is left at its initial value.
    stub = types.ModuleType("utils.common")
    stub.initialize_weights = lambda *models: None
    sys.modules["utils.common"] = stub
    sys.path.insert(0, str(repo))
    from model.model_culane import parsingNet  # noqa: E402

    # The config file is plain `name = value` assignments. runpy reads it without UFLD's Config
    # class, which needs the `addict` package.
    cfg = types.SimpleNamespace(**runpy.run_path(str(repo / "configs" / "culane_res18.py")))
    net = parsingNet(pretrained=False,  # ImageNet init is irrelevant: every weight is loaded below
                     backbone=cfg.backbone, num_grid_row=cfg.num_cell_row, num_cls_row=cfg.num_row,
                     num_grid_col=cfg.num_cell_col, num_cls_col=cfg.num_col,
                     num_lane_on_row=cfg.num_lanes, num_lane_on_col=cfg.num_lanes,
                     use_aux=False, input_height=cfg.train_height, input_width=cfg.train_width,
                     fc_norm=cfg.fc_norm)

    ckpt = torch.load(str(weights), map_location="cpu", weights_only=True)["model"]
    state = {k[7:] if k.startswith("module.") else k: v for k, v in ckpt.items()}
    # strict=True: UFLD's demo uses strict=False, which silently leaves any mismatched layer at
    # its random initialisation and still "works". Here a mismatch is a hard error.
    net.load_state_dict(state, strict=True)
    model = UfldOutputs(net).eval()

    shape = (1, 3, cfg.train_height, cfg.train_width)
    export(model, torch.zeros(*shape), out, "input",
           ["loc_row", "loc_col", "exist_row", "exist_col"])
    verify(model, out, shape, "input")
    print(f"  crop_ratio={cfg.crop_ratio}: resize frames to {cfg.train_width}x"
          f"{round(cfg.train_height / cfg.crop_ratio)} and keep the bottom {cfg.train_height} rows")


# --- MiDaS ----------------------------------------------------------------------------------------
def export_midas(out: Path, height: int, width: int) -> None:
    model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True).eval()
    export(model, torch.zeros(1, 3, height, width), out, "input", ["depth"])
    # MiDaS' output spans a large range of relative values, so compare relatively.
    verify(model, out, (1, 3, height, width), "input", rtol=1e-3, atol=1e-2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", required=True, choices=["yolov8m", "yolov8m-seg", "ufld", "midas", "signs"])
    ap.add_argument("--weights", type=Path, help="checkpoint (yolov8m, yolov8m-seg, ufld, signs)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--width", type=int, help="input width (yolov8m, signs, midas)")
    ap.add_argument("--height", type=int, help="input height (yolov8m, signs, midas)")
    ap.add_argument("--ufld-repo", type=Path,
                    default=Path(os.path.expanduser("~/src/Ultra-Fast-Lane-Detection-v2")))
    args = ap.parse_args()

    defaults = {"yolov8m": (1280, 736), "yolov8m-seg": (1280, 736), "signs": (1280, 736),
                "midas": (448, 256)}
    if args.model == "ufld" and (args.width or args.height):
        sys.exit("ufld's input size is fixed at 1600x320 by its fully-connected head")
    if args.model in defaults:
        w, h = args.width or defaults[args.model][0], args.height or defaults[args.model][1]
        if w % 32 or h % 32:
            sys.exit(f"{w}x{h}: both sides must be multiples of 32 (the network's largest stride)")

    print(f"exporting {args.model} -> {args.out} (opset {OPSET}, torch {torch.__version__})")
    if args.model in ("yolov8m", "signs"):
        # The sign detector is a YOLOv8 Detect model too: identical export and verification.
        export_yolov8m(args.weights, args.out, h, w)
    elif args.model == "yolov8m-seg":
        export_yolov8m_seg(args.weights, args.out, h, w)
    elif args.model == "ufld":
        export_ufld(args.weights, args.out, args.ufld_repo)
    else:
        export_midas(args.out, h, w)


if __name__ == "__main__":
    main()
