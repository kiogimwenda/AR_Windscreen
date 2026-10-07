# Enclosures and mounts

OpenSCAD models (2021.01 or later) for the hub's mechanical parts: `docs/BUILD_GUIDE.md` Part 4.8.6
is the design. Every model takes `part=` to pick what to render, and echoes its hole lists so the
1:1 drilling templates come from the same numbers as the 3D model.

| Directory | What | Status (2026-10-06) |
|---|---|---|
| `common/boards.scad` | Both PCBs' outlines, mounting holes and edge-connector positions. **Generated** by `hardware/gen/outlines.py`, which also writes the starter `.kicad_pcb` outlines, so board and box cannot drift apart. | Done |
| `lidar_mount/` | Livox Mid-360 roof mount: aluminium base plate on pot magnets, printed 15° wedge, aluminium top plate (the LiDAR's heatsink) | Designed; check against the LiDAR before drilling |
| `power_box/` | Printed chassis plate for a bought ABS junction box, and drilling templates for the box | Designed; measure the box and modules you buy first |
| (pod, E-stop housing, gesture puck) | Need the camera, the breakouts and the test car | Not started |

```bash
O="/mnt/c/Program Files/OpenSCAD/openscad.com"            # from WSL; plain `openscad` on Linux
W=$(wslpath -w $PWD/hardware/enclosures/lidar_mount)
"$O" -D 'part="wedge"' -o "$W\\wedge.stl" "$W\\lidar_mount.scad"
python3 hardware/enclosures/make_templates.py hardware/enclosures/lidar_mount/lidar_mount.scad \
        hardware/enclosures/lidar_mount/drilling_templates.pdf
python3 hardware/gen/outlines.py                           # after changing a board's size or holes
```

**Drilling templates** (`drilling_templates.pdf`): print at **100 % / actual size**, never "fit to
page", and measure the 100 mm bar before drilling. Each red cross is a hole centre: centre-punch,
pilot-drill 2.5 mm, drill to size, deburr. The scale was checked by rasterising at 10 px/mm (the
246 mm power-box wall measures 246.2 mm, the extra being the line width).

**Material:** print in **ASA or PETG**, never PLA: a car's cabin passes 70 °C in the sun and PLA
softens at about 55 °C. 4 perimeters, 40 % infill for the wedge and the chassis. Brass heat-set
inserts go in with a soldering iron at about 220 °C (PETG) / 250 °C (ASA), pressed square.

## LiDAR roof mount (`lidar_mount/`)

Stack, bottom to top: four rubber-coated pot magnets → **base plate** (3 mm aluminium, 150 × 150) →
**wedge** (printed, 15°, 120 × 120, 12 mm thick at the front) → **top plate** (3 mm aluminium,
130 × 130) → Livox Mid-360, connector facing the rear. Files: `assembly.png`, `side.png` (check the
tilt: the front is lower), `wedge.stl`, `top_plate_2d.dxf/svg`, `base_plate_2d.dxf/svg` (for a
laser or waterjet shop), `drilling_templates.pdf` (to drill by hand).

The top plate is the LiDAR's heatsink: Livox asks for at least 3 mm of metal and 10,000 mm² of
exposed area; 130² − 65² = 12,675 mm². Keep 10 mm clear around the sensor.

| Part | Qty | Note |
|---|---|---|
| M3 × 7 socket-head screw, stainless | 4 | LiDAR to the top plate, from below. The LiDAR's holes are **5 mm deep: 3 mm plate + at most 5 mm thread**. A longer screw bottoms out and can crack the housing. |
| Dowel pin 3 × 5 mm | 2 | Press into the top plate. **Before drilling the dowel holes, hold the plate to the LiDAR and check the template's positions** (round hole 16 mm and slot 23 mm from centre, across the connector axis; manual v1.2). Skip the dowels if they disagree: the four screws alone hold it. |
| M4 brass heat-set insert, 8.1 mm long (5.6 mm hole) | 8 | 4 in the wedge's sloped face, 4 in its base |
| M4 × 10 screw | 8 | Plates to the wedge; thread-locker on all of them |
| Pot magnet, rubber-coated, D43 with M6 stud | 4 | ~ 9 kg pull each on paint; the rubber protects the paint. **Or** roof-bar clamps through the same holes if the car has bars. |
| Tether (steel cable with eyes, ~0.5 m) | 1 | From the base plate's 6.5 mm hole to a door-frame anchor (closed in the door seal), so a slipping mount cannot leave the roof |

**For the calibration (Part 12.2.2):** the optical centre is **47 mm above the LiDAR's base**, so
50 mm above the top plate's bottom face. Measure the top plate's height and the forward offset
from the camera on the car, and use the actual tilt (inclinometer app on the top plate), not 15°.

## Power box (`power_box/`)

A **bought ABS junction box, about 250 × 150 × 100 mm outside (240 × 140 × 95 inside), IP65
class**, with a **printed chassis plate** (210 × 130 × 4 mm) that carries everything. Why not a
printed box: the contents need about 240 × 140 mm of floor, beyond most printer beds (220 × 220),
and a bought box is stronger and seals better. The guide's earlier 150 × 100 × 60 estimate cannot
hold six 28 mm automotive relays next to the 110 × 80 board.

Layout (`top.png`, `assembly.png`): the power board against the **front wall**, its DB-25 through
it; the BTS7960 (heatsink up) and ACS712 to the left, beside the board's module headers; the six
relays in a row along the back, the kill relay (red) nearest the battery and actuator glands; eight M16
glands in the **rear wall**; vent slots in the **left wall** level with the heatsink. The board is
placed turned 180° from KiCad's view (its top edge to the front), which the model handles.

**Measure before printing or drilling** (each is marked `VERIFY` in `power_box.scad`): the box's
inside size and floor bosses (the chassis must sit flat), the BTS7960 and ACS712 hole positions,
the relay sockets' tab positions, the heatsink height, and the DB-25's shell against the cut-out.
Change the numbers and re-render; the templates follow.

| Part | Qty | Note |
|---|---|---|
| ABS junction box, ~250 × 150 × 100, with lid gasket | 1 | Electrical shops |
| Cable gland M16 × 1.5 (4–8 mm cable), with locknut | 8 | 16.3 mm holes; seal every lead, strain-relieve it |
| M3 brass heat-set insert (4.0 mm hole) and M3 × 6 screws | 10 | Board (4), BTS7960 (4), ACS712 (2) |
| M4 brass heat-set insert and M4 × 10 screws | 10 | Chassis to the box floor from below (4), relay socket tabs (6) |
| DB-25 jackscrews (4-40 UNC) | 2 | Clamp the connector to the front wall |
| Mounting bracket (steel angle) and M5 bolts | 1 | Under the dash on a solid structure, **never the steering column**, clear of the pedals and knees |

Assembly: drill the box from the templates; print the chassis; fit the inserts; mark the floor
holes through the chassis (or use the floor template) and drill them; fit modules, relays and
board; wire; close.

**Heat:** the BTS7960 at the actuator's few amps dissipates little, but the box is sealed and
under the dash: the vent slots are the minimum. Measure the heatsink temperature during the
Phase 13 bench run; add a small fan only if it passes about 60 °C.
