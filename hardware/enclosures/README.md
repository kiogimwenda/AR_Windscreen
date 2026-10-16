# Enclosures and mounts

OpenSCAD models (2021.01 or later) for the hub's mechanical parts: `docs/BUILD_GUIDE.md` Part 4.8.6
is the design. Every model takes `part=` to pick what to render, and echoes its hole lists so the
1:1 drilling templates come from the same numbers as the 3D model.

| Directory | What | Status (2026-10-16) |
|---|---|---|
| `common/boards.scad` | Both PCBs' outlines, mounting holes and edge-connector positions. **Generated** by `hardware/gen/outlines.py`, which also writes the starter `.kicad_pcb` outlines, so board and box cannot drift apart. | Done |
| `lidar_mount/` | Livox Mid-360 roof mount: aluminium base plate on pot magnets, printed 15° wedge, aluminium top plate (the LiDAR's heatsink) | Designed; check against the LiDAR before drilling |
| `power_box/` | Printed chassis plate for a bought ABS junction box, and drilling templates for the box | Designed; measure the box and modules you buy first |
| `estop/` | E-stop housing: printed yellow box for the 22 mm mushroom E-stop, base plate to a dash panel, M16 gland | Designed 2026-10-16; measure the E-stop you buy first |
| `pod/` | Windscreen pod: printed box for the pod board, camera, IMU and GNSS on one rigid carrier wall; black lens hood; cradle with a ±8° pitch lock on a rail and an aluminium plate bonded to the glass | Designed 2026-10-16 around a placeholder camera; measure the camera, GNSS board and glass first |
| `gesture_puck/` | Gesture puck: printed black housing for the APDS-9960 module on the column shroud, open sensor window, cable exit, VHB base | Designed 2026-10-16 around the common GY-9960 module; measure yours first |

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
class**, with a **printed chassis plate** (210 × 112 × 4 mm) that carries everything. Why not a
printed box: the contents need about 240 × 140 mm of floor, beyond most printer beds (220 × 220),
and a bought box is stronger and seals better. The guide's earlier 150 × 100 × 60 estimate cannot
hold six 28 mm automotive relays next to the board. (The board was compacted to 90 × 66 mm (95 × 66 since 2026-10-09: room for its labels) on
2026-10-08 and the plate to 210 × 112, the relay row moved forward behind it; the box stays: the
row of six relays, ~193 mm, sets its length.)

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

## E-stop housing (`estop/`)

A printed box for the 22 mm mushroom emergency stop (BOM 3.4), beside the steering column where
the driver reaches it without looking (BUILD_GUIDE Part 0 property 2). **Print it in yellow ASA**:
ISO 13850 asks for a red actuator on a yellow background. Files: `assembly.png`, `body.stl`,
`base.stl`, `drilling_template.pdf` (the two M4 holes in the dash panel), `estop.scad`.

- **Body:** 56 × 56 × 59 mm (89 mm with the mushroom). The button mounts through its 3 mm front
  face in the standard **22.3 mm hole with the 3.2 mm anti-rotation notch**; its two contact
  blocks hang inside (48 mm of depth). "STOP" is sunk into the face below the head.
- **Base:** the back plate, four M3 screws into heat-set inserts in the body's corners (open it to
  wire), and two **M4 screws into a solid dash panel** (never the steering column or a trim panel
  that flexes). Countersunk both ways so it sits flat.
- **Cable:** a 4-core lead through an **M16 gland** in the side, to the power box: one contact
  pair in series with the 40 A kill relay's coil (power board J8), the second to the sense line
  (J9). The guide's wiring: `docs/assembly/power_box_wiring.md`.

**Measure before printing** (each `VERIFY` in `estop.scad`): the panel hole and notch (some
buttons have no notch), the mushroom's size, and the depth and width of the bezel with both
contact blocks fitted.

| Part | Qty | Note |
|---|---|---|
| E-stop, 22 mm mushroom, twist-release, NC + second contact block | 1 | BOM 3.4 |
| M3 brass heat-set insert, 5.7 mm long (4.0 mm hole) | 4 | in the body's corner bosses |
| M3 × 8 countersunk screw | 4 | base to body |
| M4 × 12 countersunk screw (or self-tapping for plastic panels) | 2 | base to the dash panel |
| Cable gland M16 × 1.5 (4–8 mm cable), with locknut | 1 | |

## Windscreen pod (`pod/`)

The box behind the rear-view mirror (BUILD_GUIDE 4.8.1, 4.8.6): the pod board, the camera, the
IMU (on the board) and the GNSS module. Files: `pod.scad`, `assembly.png`, `side.png` (from the
right, glass at 27°), `rear.png` (the driver's view), five STLs, `drilling_template.pdf` (the glass
plate's two holes).

- **Box:** 79 wide × 52 deep × 75 high mm, plus the 30 mm hood. Three printed parts, joined at
  y = 26 (rear cover | front shell) and y = 49 (front shell | front plate):
  - `rear_cover.stl` (light ASA): the driver's side. Four status-LED windows with their names
    (PWR, LINK, ARM, FAULT), open slots for the USB-C, the gesture cable and the DB-25 (so the
    cover slides on over the plugs), RESET/BOOT pin-holes in the left side (press with a 2 mm rod),
    top vents. Held by **two M3 × 30 screws at the top** (through tubes above the board) and **two
    lap tabs at the bottom**, each with an M3 × 6 from below: the board fills the cover wall to
    wall, so no screw can pass it lower down.
  - `front_shell.stl` (light ASA): the rigid core. Its **carrier wall** takes the pod board on four
    5 mm standoffs behind it and the camera on four 4 mm standoffs in front of it: camera and IMU
    share one rigid plate, so the camera-IMU extrinsics stay put (Part 12.2.2). The GNSS shelf
    above the board, the pivot and lock inserts in the side walls, bottom vents (the chimney's
    inlet; the top vents are its outlet) and the camera's USB lead out through the floor.
  - `front_plate.stl` (**black** ASA, matte): the lens window and the hood, splayed 4° wider than
    the lens's field (90° × 55°) so it stays out of the image. Its rim stops short of the glass:
    close the gap with a strip of black foam so the dash cannot reflect into the lens.
- **Mount:** `glass_plate` (3 mm aluminium, 50 × 60, cut by hand from the template or bought cut),
  bonded with 3M VHB in the glass's uncoated area behind the mirror; `rail.stl`, a dovetail screwed
  under it; `cradle.stl`, which slides onto the rail and is located and held by one M4 thumbscrew
  into the rail's insert (so the pod comes off and returns to the same place). The cradle's two
  ears carry the pod's **pitch pivot** and an **arc slot for the lock screw (±8°)**: pitch the
  camera until the horizon sits about a third from the top of the image, lock both sides, mark the
  position with paint, then recalibrate (Part 12.2.2).
- **Print:** rear cover on its back face, front shell on its front face (the carrier wall
  vertical), front plate on its face (the hood upwards: no supports), cradle on the saddle's top,
  rail on its wide face. 4 perimeters, 40 % infill, ASA (the glass zone passes 70 °C).

**Check after the first drive:** the image's bottom rows look steeply down, past the hood's lower
edge, at the dash. If the hood or the dash shows in the frame, pitch the pod up a step within the
±8° (or shorten `hood_len`) before calibrating.

**Measure before printing** (each `VERIFY` in `pod.scad`): the windscreen's angle from horizontal
at the mirror (`glass_angle`; the cradle and hood follow it), the camera board (38 × 38 with
M2 holes 34 apart is the common ELP size, but check yours), the lens barrel's diameter and length,
and the GNSS board with its stacked antenna (and whether it has holes: if not, strap it to the
shelf). The board's own outline and holes come from `../common/boards.scad`, already right. Before
bonding, check the plate's position is **outside the driver's swept view and inside the wipers'
sweep** (the camera needs clean glass), and that the car's mirror sensor area is clear.

| Part | Qty | Note |
|---|---|---|
| Aluminium plate 50 × 60 × 3 | 1 | two holes drilled 2.5 and tapped M3 (template) |
| 3M VHB tape (5952 or 4910), 50 × 60 | 1 | clean the glass with isopropanol, press 30 s, wait 24 h before loading |
| M3 brass heat-set insert (4.0 mm hole) | 14 | front shell: board 4, front plate 4, rear cover 2 + tabs 2, GNSS 2 |
| M2 brass heat-set insert (3.2 mm hole) | 4 | front shell: the camera's standoffs |
| M4 brass heat-set insert (5.6 mm hole) | 5 | front shell: pivot 2, lock 2; rail: thumbscrew 1 |
| M3 × 6 screw | 6 | board to standoffs (4); rear cover's bottom tabs, from below (2) |
| M3 × 5 screw | 2 | rail to plate: **never longer**, or it reaches the glass through the 3 mm plate |
| M3 × 30 socket-head screw | 2 | rear cover, top |
| M3 × 8 pan-head screw | 2 | GNSS board |
| M3 × 8 countersunk screw | 4 | front plate |
| M2 × 5 screw | 4 | camera board |
| M4 × 12 screw + 2 washers | 2 | pivot (one each side) |
| M4 thumb screw × 12 + washer | 2 | pitch lock (one each side) |
| M4 thumb screw × 12 | 1 | cradle to rail: **never longer**, or it reaches the plate |
| Black foam strip, 3 × 10 mm, self-adhesive | 0.3 m | the hood's rim against the glass |

## Gesture puck (`gesture_puck/`)

The APDS-9960 gesture sensor's housing (BOM 2.5), on the steering-column shroud with the sensor
facing up at the driver's hand (BUILD_GUIDE 4.8.1). Files: `gesture_puck.scad`, `assembly.png`,
`shell.stl`, `base.stl`. **Print in black ASA** (matte: light bouncing inside reaches the sensor).

- **Size:** 34 × 21 × 10.5 mm. The module is clamped between four posts on the base and four pins
  from the shell's ceiling, its sensor 0.3 mm under an **open 8 × 6 mm window** flared 45°
  outwards. Leave the window open: a clear cover reflects the sensor's own IR LED back into it
  and blinds the gesture engine (if a cover is ever needed, use a thin IR-transparent one and test).
- **Cable:** five wires soldered straight to the module's pads (no header fitted: it would not
  fit under the ceiling), out through the 4.5 mm exit at the pad end, a cable tie round the lead
  inside as strain relief, then ~1.5 m along the shroud and A-pillar to the pod's J4 (JST-GH
  5-pin: 3V3, GND, SCL, SDA, INT; the module's **VL pad powers its IR LED and must get 3.3 V**:
  close the module's PS jumper, or bridge VL to VCC; without it gestures are never seen).
- **Base:** two M2 × 8 self-tapping countersunk screws up into the shell's end bosses; the base
  is bonded to the shroud with VHB. The arrow on top points to the car's front: the firmware's
  left/right/up/down assume that orientation.

**Measure before printing** (each `VERIFY` in `gesture_puck.scad`): the board's size, the sensor's
position along it, the pad row, and that the clamp points (`post_xy`) land on bare board.

| Part | Qty | Note |
|---|---|---|
| APDS-9960 module | 1 | BOM 2.5 |
| M2 × 8 self-tapping countersunk screw | 2 | base to shell |
| 3M VHB tape, 30 × 20 | 1 | base to the shroud; clean with isopropanol |
| 5-core lead ~1.5 m, JST-GH 5-pin housing (GHR-05V-S) + crimps | 1 | to the pod's J4 (BOM §10) |
| Cable tie, 2.5 mm | 1 | strain relief inside |
