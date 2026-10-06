# Hardware

The custom hardware of the two-box hub: `docs/BUILD_GUIDE.md` Part 4.8 is the design, Part 14
Phase 12B the plan.

| Directory | Contents | Status (2026-10-06) |
|---|---|---|
| `gen/` | The board descriptions and the tools that turn them into KiCad files and check them | Done |
| `pod_board/` | KiCad 10 project: STM32F405RGT6, 8 MHz crystal, USB-C, diode-OR'd 5 V to 3.3 V, IMU / GNSS / gesture / SWD / ELM327 headers, status LEDs, DB-25 with series resistors and input filters. 4-layer. | Schematic done, ERC clean, cross-checked; **PCB layout to do** |
| `power_board/` | KiCad 10 project: 12 V fuse, TVS and reverse-polarity diode; TPS54360 5 V (TI reference design); 3.3 V; 74HCT244 buffer; BTS7960 and ACS712 module headers; magnet and relay MOSFETs with flyback diodes; fail-safe pull-downs; SN65HVD230 CAN; presence MOSFET; PC817 brake-light input; terminals for the kill relay, E-stop, relays, actuator, LiDAR and OBD lead; DB-25. 2-layer, 2 oz. | Schematic done, ERC clean, cross-checked; **PCB layout to do** |
| `enclosures/` | Printable pod, power box, gesture puck; LiDAR wedge-plate drawing | Not started |

Each board folder also has `<board>.pdf` (the schematic, for reading without KiCad) and
`<board>_bom.csv` (parts grouped by value and footprint).

## How the schematics are made and checked

```bash
hardware/gen/check.sh
```
1. `gen/generate.py` writes both `.kicad_sch` files from `gen/pod_board.py` and
   `gen/power_board.py`. Each part names its KiCad symbol, footprint and value and maps **pin
   numbers to net names**; the generator copies the real library symbols in, places the parts in
   labelled groups, and connects pins by net label.
2. KiCad's ERC must report **0 violations**.
3. `gen/check_nets.py` reads the exported netlists and verifies, against the sources of truth:
   - every pin in `firmware/.../Config.h` is on the right STM32 pin of the pod board;
   - every DB-25 pin on both boards matches the table in BUILD_GUIDE 4.8.3 (parsed from the guide);
   - pod: each cable line passes through a series resistor; inputs read safe when unplugged;
   - power board: every pod-driven line is pulled to its safe state (Part 4.8.4's rule);
   - no misspelt single-connection nets; every footprint exists in KiCad's library.

   It was tested by breaking the design on purpose (a relay moved to the wrong MCU pin, a
   pull-down removed, two cable pins swapped); each was caught.

**Change the design in `gen/*.py`, never only in KiCad's editor**, then run `check.sh`. Edits made
only in the `.kicad_sch` are overwritten by the next generation. (Once layout starts, the
schematic can be frozen and edited in KiCad directly; then stop regenerating and keep running the
netlist check: export the netlist from KiCad and run `check_nets.py` on it.)

## Choices made during schematic capture (see also decisions.md)

- **Reverse polarity: a B560C Schottky, not a P-MOSFET.** SOT-23 P-MOSFETs are rated −30 V, too
  close to the TVS's ~39 V clamp. The diode drops ~0.6 V (~1 W at 2 A) and needs no gate circuit.
- **5 V converter: TI's TPS54360 reference design, unchanged** (datasheet SLVSBB4G, Figure 34):
  600 kHz, 8.2 µH, B560C, 2 × 47 µF, compensation 13.0 k / 6800 p / 39 p, start 8 V / stop 6.25 V.
  Copying a validated design is the right call for a first board; it is rated well beyond our
  ~1 A load.
- **Relays and magnet: logic-level MOSFETs driven straight from the cable lines** (AO3400A turns
  fully on at 2.5 V); the 74HCT244 buffers only the BTS7960 module's three 5 V inputs.
- **Back-powering:** each pod output has a 1 k series resistor (330 Ω on CAN_TX for its 500 kbit/s
  edges), so a pod running on USB cannot push more than ~3 mA per line into an unpowered box.
- **Module headers are placeholders for breakouts** (BNO085, NEO-M8N): the pin order on `J2` and
  `J3` of the pod board must be matched to the actual breakouts bought **before layout**.

## What remains for Phase 12B (Ian's work)

Layout is where the remaining engineering judgement is, and the part examiners will ask about.
In KiCad: open each `.kicad_pro`, *Tools → Update PCB from Schematic*, draw the board outline,
place, route, run DRC.

**Pod board (4-layer: signal / ground / power / signal).**
- Decoupling capacitors (C3–C9) at their VDD pins, each with its own via to the ground plane.
  VCAP capacitors (C10, C11) right at pins 31 and 47.
- Crystal (Y1, C12, C13) within ~5 mm of PH0/PH1, nothing routed underneath, guard with ground.
- USB D+/D− as a matched pair (~90 Ω differential), short, the ESD chip (U4) right at the socket.
- The GNSS breakout at the board edge that faces the glass, away from the MCU and the USB; keep
  the ground plane unbroken under it.
- The IMU breakout on standoffs at a fixed, known orientation (record it: Part 12.2.2 calibrates
  it, but a sensible starting orientation makes the calibration easy to check).
- DB-25 at one end, the series resistors and input filters right behind it.
- Status LEDs on the edge that faces the driver.

**Power board (2-layer, 2 oz copper).**
- The TPS54360's power loop is the critical path: input capacitors (C21, C22), the IC, the catch
  diode (D13) and the inductor (L1) as close together as possible, the SW node small (it is the
  board's noise source), the feedback divider (R44, R45) away from L1 and the SW node. Follow the
  datasheet's layout example (section 10) closely.
- Wide traces or pours for the 12 V input, the actuator rail (ACT_12V) and the relay coils' +12 V;
  the actuator's heavy current itself runs in wire on the BTS7960 module, not on this board.
- Fuse holders and terminal blocks along the edges, grouped by what they connect to (12 V in,
  actuator side, car-wiring side).
- The SN65HVD230 and its ESD diode (D20) right at the OBD terminal block.
- The 10 k pull-downs close to the DB-25, so the lines are defined as soon as they enter the board.

**Then:** DRC clean, a 1:1 paper print to check every footprint against the real part, Gerbers,
order (JLCPCB / PCBWay), assemble, and bring up with Part 4.7 before the bench gate is re-run on
this hardware (three consecutive passes of all six items).
