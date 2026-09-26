# Road-sign detector: training and evaluation report

**Dates:** 2026-09-24 → 2026-09-26. **Model:** YOLOv8s fine-tuned on the Mapillary Traffic Sign
Dataset (MTSD), 29 classes. **Final weights:** `host/models/training/signs_v2/weights/best.pt`
(epoch 53). **Engine:** `host/models/engines/signs.engine` (FP16, 1280×736).

This replaces the report the overnight agent was to write
(`2026-09-25-sign-detector-overnight.md`, never written; the agent was stopped twice by usage
limits). The per-task narrative is in `docs/progress-log.md` (2026-09-24 to 26), and the
decisions are in `docs/decisions.md` ("Phase 5 extension").

---

## 1. Why this model exists

The COCO-trained YOLOv8m detector knows only two sign classes, "stop sign" and "traffic light".
Ian observed on test footage that it misses most road signs. The AR overlay design
(`docs/architecture/ar-overlay-design.md`) needs to know what each sign *means*: stop barrier,
no-turn barriers, the speed limit against actual speed. So a dedicated fourth model detects
29 sign classes, including one class per speed-limit value.

## 2. Data

- **Source:** MTSD v2, fully annotated part, licence CC BY-NC-SA (non-commercial research).
  Obtained through Ian's Mapillary account. His signed download links were kept out of the
  repository.
- **Downloaded:** annotations, train.0/1/2 and val, 41,909 images, verified against the zip
  listings. **Not downloaded:** test (no public labels) and the partially annotated set
  (machine-labelled).
- **Conversion** (`host/scripts/prepare_mtsd.py`, map in `host/scripts/mtsd_class_map.json`):
  - 401 MTSD labels are mapped onto 29 classes, merging design variants;
  - information--*, complementary--* and other-sign are dropped;
  - boxes flagged ambiguous or dummy, and speed-limit variants of uncertain unit (mph vs km/h,
    `mtsd_uncertain_unit_speed_variants.json`), are dropped;
  - panoramas are skipped;
  - images are capped at 2,048 px on the long side.
  - **Result:** 35,758 training and 5,210 validation images. Per-class counts are in
    `data/datasets/mtsd_yolo/instance_counts.json`.
- **Smallest classes (train):** speed_limit_10 79, speed_limit_120 94, speed_limit_110 95,
  speed_limit_20 195, speed_limit_90 201. **Largest:** other_regulatory 11,105, other_warning
  9,953.

## 3. Training

| Run | Data | Start | Epochs | Stop reason | Best mAP50 / mAP50-95 |
|---|---|---|---|---|---|
| signs_v1 | train.0 only (12,411 images) | COCO `yolov8s.pt` | 22 | 4.94 h time budget | 0.533 / 0.418 |
| **signs_v2** | all (35,758 images) | signs_v1 `best.pt` | 60 (cap) | cap reached; converged | **0.716 / 0.572** (epoch 53) |

**Settings (both runs):**
- YOLOv8s, imgsz 1280, batch 8, AMP, SGD (ultralytics auto).
- **fliplr = 0**: a mirrored no-left-turn sign *is* a no-right-turn sign.
- signs_v2: `epochs=60, patience=15`. Early stopping never triggered, because fitness kept
  inching up to epoch 53.
- Full per-epoch table: `python3 host/scripts/show_results.py host/models/training/signs_v2/results.csv`.

**Curve summary:**
- *Epochs 2–6:* a dip, from learning-rate warm-up disturbing the warm-started weights.
- *To ~epoch 30:* steady gains.
- *From ~epoch 47:* a plateau, with mAP50-95 within 0.5713–0.5727.
- *Epochs 51–60 (mosaic off):* no final gain.
- *Losses:* training losses kept falling while validation losses stayed flat: converged, with
  the onset of overfitting.

**Incidents:** five GPU faults (`CUDA error: unknown error`) at epochs 21, 23, 30, 50 and 51.
- The first followed a Windows "power source change" (charger) one minute earlier.
- The later ones coincided with other GPU users (browser video, the NVIDIA overlay) on an
  8 GB card at ~95% memory.
- All were recovered automatically by `host/models/training/resume_signs_v2.sh` (resume from
  `last.pt`, retry limit raised to 7), with at most one partial epoch lost each time.

## 4. Validation results (MTSD val, `best.pt`)

Overall: **P 0.816, R 0.613, mAP50 0.716, mAP50-95 0.572** (5,210 images, 6,330 signs).

| Class | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| stop | 0.880 | 0.764 | 0.869 | 0.687 |
| give_way | 0.867 | 0.719 | 0.811 | 0.581 |
| no_entry | 0.833 | 0.597 | 0.711 | 0.486 |
| no_left_turn | 0.846 | 0.564 | 0.633 | 0.485 |
| no_right_turn | 0.927 | 0.691 | 0.778 | 0.596 |
| no_u_turn | 0.839 | 0.711 | 0.788 | 0.625 |
| no_overtaking | 0.756 | 0.807 | 0.866 | 0.708 |
| speed_limit_10 | 0.889 | 0.333 | 0.431 | 0.335 |
| speed_limit_20 | 0.846 | 0.392 | 0.448 | 0.383 |
| speed_limit_30 | 0.763 | 0.650 | 0.701 | 0.589 |
| speed_limit_40 | 0.835 | 0.631 | 0.750 | 0.616 |
| speed_limit_50 | 0.877 | 0.588 | 0.756 | 0.632 |
| speed_limit_60 | 0.779 | 0.361 | 0.547 | 0.417 |
| speed_limit_70 | 0.884 | 0.634 | 0.728 | 0.635 |
| speed_limit_80 | 0.828 | 0.462 | 0.605 | 0.496 |
| speed_limit_90 | 0.972 | 0.741 | 0.841 | 0.678 |
| speed_limit_100 | 0.819 | 0.514 | 0.677 | 0.505 |
| speed_limit_110 | 0.646 | 0.500 | 0.614 | 0.494 |
| speed_limit_120 | 0.667 | 0.429 | 0.530 | 0.479 |
| end_of_restriction | 0.621 | 0.709 | 0.658 | 0.527 |
| pedestrian_crossing | 0.853 | 0.671 | 0.818 | 0.689 |
| children_school | 0.742 | 0.642 | 0.764 | 0.648 |
| road_hump | 0.881 | 0.744 | 0.827 | 0.685 |
| roundabout | 0.771 | 0.573 | 0.736 | 0.594 |
| traffic_signals_ahead | 0.866 | 0.574 | 0.763 | 0.635 |
| keep_left_or_right | 0.882 | 0.706 | 0.797 | 0.578 |
| no_parking_or_stopping | 0.816 | 0.745 | 0.827 | 0.649 |
| other_warning | 0.715 | 0.750 | 0.794 | 0.644 |
| other_regulatory | 0.768 | 0.565 | 0.687 | 0.519 |

**Confusion matrix** (`host/models/training/signs_v2/confusion_matrix_normalized.png`):
- **Speed-limit errors are mainly misses, not misreads.** 40–55% of true 10/20/60/80/110/120
  signs are predicted as background. Value confusion is small (~5–15%: 120→110, 10→70/30).
- **False positives concentrate in the catch-all classes** (other_warning, other_regulatory).

## 5. Speed (TensorRT FP16, RTX 5060 Laptop, trtexec, each model alone)

| Model | Input | Median GPU time |
|---|---|---|
| YOLOv8m (objects) | 1280×736 | 5.38 ms |
| **Sign detector** | 1280×736 | **2.48 ms** |
| UFLDv2 (lanes) | 1600×320 | 2.17 ms |
| MiDaS v2.1 S (depth) | 448×256 | 1.29 ms |

Total ≈ **11.3 ms** of a 33 ms frame (30 FPS), measured one model at a time. Concurrent timing
with pre-processing (as in `inference_viewer`) comes when the sign model joins
`MlInferenceEngine`. The ONNX export was verified against PyTorch through ONNX Runtime (max
|Δ| 9.8e-4).

## 6. Evaluation on real Kenyan imagery

**Data:** 300 openly licensed images in `data/footage/kenya/`:
- 138 KartaView dashcam frames, Nairobi;
- 61 KartaView frames, A104 Limuru–Naivasha–Nakuru;
- 101 Wikimedia Commons photos.

Sources, authors and licences (CC BY-SA 4.0 / CC BY / CC0) are in `data/README.md` and the
manifests. Annotated outputs are in `data/footage/kenya_annotated/` (every detection ≥ 0.10, with
`detections.csv`). There is no ground truth, so this is a **visual audit**.

**Result:** 57 detections in 47 of the 300 images. **All 57 were audited** from contact sheets
cropped around each detection:

| Confidence | Detections | Correct | Wrong | Cannot verify |
|---|---|---|---|---|
| ≥ 0.5 | 27 | 24 | 1 | 2 |
| 0.25–0.5 | 10 | 6 | 1 | 3 |
| < 0.25 | 20 | 2 | 7 | 11 |

- **≥ 0.5 is reliable: 24 of 25 verifiable detections are correct.** Stop signs (including
  blurred and partly hidden ones), speed limit 30, road-hump warnings (one with a "BUMP AHEAD"
  plate), pedestrian crossing, keep-left, a 3.5 t weight limit, blue pedestrian and cycle signs,
  and a steep-hill warning. This supports the overlay design's ≥ 0.5 confirmation floor.
- **The one high-confidence error is a speed misread:** a 30 sign read as **50** (0.57). A second
  30 was read as 40 at 0.14. For the speed display, a wrong limit is worse than a missing one;
  see §7.
- **Below 0.25 it is mostly noise:** shop logos, an advert icon, a man's face in an old photo,
  the back of a sign.
- **Kenyan variant at low confidence:** a red **"NO ENTRY" sign with text** was found correctly,
  but only at 0.18. MTSD rarely shows that design.
- **Lowest correct confidences seen:** 0.18 (the text NO ENTRY) and 0.21 (a pedestrian-crossing
  warning).

**Recall on Kenyan roads could not be measured.** A random sample of 12 of the 248 images
without detections was inspected at full frame. 11 contain no traffic sign at all (dirt roads,
a game-park track with zebras, open highway, shop streets); one is uncertain (a small board at
dusk). The collected set is dominated by sign-free frames, which limits the evaluation, not the
model. **A Kenyan test set with signs, from the project's own drives, is needed** before any
claim about Kenyan recall.

**Comparison clip:** 240 frames (every 30th) of the Kraków test footage gave 447 detections
(274 ≥ 0.5), mostly other_regulatory, give_way, no_entry and no_parking. It was not audited
frame by frame. Outputs are in `data/footage/krakow_signs/`.

## 7. Gaps and improvement plan

Recorded in full in `docs/progress-log.md` ("Sign detector: known gaps and the improvement plan",
2026-09-26). In short:

1. **Missed speed limits.** 40–55% of rare-value speed signs go undetected on MTSD val.
2. **Dropped-but-still-visible signs became negatives.** Our conversion deleted ambiguous and
   uncertain-unit boxes without masking them, teaching "small, unclear sign = background".
   **Fix: mask them out of the images, then retrain.** This is the first next step.
3. **False positives in the catch-all classes,** likely the unlabelled information signs.
4. **Recall (0.61) is the limit, not precision (0.82).** It is a data problem.
5. **no_left_turn < no_right_turn,** with no flip augmentation. **Fix: label-aware flipping**
   (swap left/right labels; never flip text or digit signs).
6. **No Kenyan training data or test set.** **Fix: record drives, active learning** (label the
   frames the model is least sure of), fine-tune.
7. **Speed values.** **Fix: two stages.** Detect "speed-limit sign", then read the value from a
   crop of the native-resolution 2K frame. This also targets the 30→50 misread seen in Kenya.
8. **Measure per-sign recall on video** (the tracker sees a sign many times on approach), plus
   recall by sign size and day vs night.

**Safety context:** signs never actuate (overlay design rule 1). OSM speed/stop/turn priors fill
missed signs (rule 7). Multi-frame confirmation filters one-frame false positives (rule 2).
Because a *wrong speed value* is the most harmful display error seen, the speed badge should
require agreement across frames on the **value**, not only on "a speed sign is here". This is now
a rule in `docs/architecture/ar-overlay-design.md` §2 (speed limits).

## 8. Attribution

- MTSD: Ertler et al., "The Mapillary Traffic Sign Dataset for Detection and Classification on
  a Global Scale", ECCV 2020. CC BY-NC-SA.
- Kraków footage: "City Driving 4K – Kraków Poland 2024", Relaxing Roads 4K, CC BY 3.0.
- Kenyan imagery: KartaView contributors (CC BY-SA 4.0) and the Wikimedia Commons authors listed
  in `data/footage/kenya/*manifest.csv`.
