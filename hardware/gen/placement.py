"""Where every part goes on each board: the placement plan layout.py applies, and the layout guide
(hardware/LAYOUT_GUIDE.md) explains.

Coordinates: mm from the board's top-left corner, x right, y DOWN (KiCad's convention); rotation in
degrees, counter-clockwise as KiCad shows it. (x, y) is the footprint's own origin (for most SMD
parts their centre; for connectors and headers, pin 1). All parts are on the top side.

Each entry: ref: (x, y, rotation, why). "why" is the placement reason, quoted in the guide.
layout.py checks the result with KiCad's own DRC (courtyards must not overlap, nothing off the
board) and links every footprint to its schematic symbol.
"""

# --- Pod board, 70 x 53 mm, 4 layers (compacted 2026-10-08 from 80 x 55) --------------------------
# The MCU at (28, 16): every MCU-side part is placed relative to its pins (LQFP-64, 0.5 mm pitch:
# pins 1-16 down the left side, 17-32 along the bottom, 33-48 up the right, 49-64 along the top).
POD_W, POD_H = 70.0, 53.0
MCU = (28.0, 16.0)

# DB-25 on the bottom edge, opening down, centred: its pin 1 (the footprint's origin) sits 7.78 mm
# in from the edge (the mounting holes are 4.94 mm from the edge, 2.84 mm behind pin 1's row) and
# 16.62 mm right of the connector's centre.
POD_DB25 = (POD_W / 2 + 16.62, POD_H - 7.78)


def pod_db25_x(pin):
    """x of a DB-25 pin on the pod (row 1: pins 1-13; row 2: pins 14-25, offset half a pitch)."""
    x0 = POD_DB25[0]
    return x0 - 2.77 * (pin - 1) if pin <= 13 else x0 - 1.39 - 2.77 * (pin - 14)


# The cable-conditioning parts stand in columns above their own DB-25 pin, in four rows:
# Row D stands 5 mm below the MCU: at 3.4 mm (a 51 mm board) the ~20 cable signals could not all
# get out from under the MCU on two signal layers (the routability check, routecheck.sh).
ROW_A, ROW_B, ROW_C, ROW_D = 41.0, 37.3, 33.6, 29.9  # row 1's series parts, row 2's, then extras

POD = {
    # Edge connectors and fixed parts
    "J7": (*POD_DB25, 0, "DB-25 on the bottom edge, flange flush, centred (the enclosure's cut-out)"),
    "J1": (65.8, 13.5, 90, "USB-C opening on the right edge (enclosure cut-out)"),
    "J4": (66.0, 25.5, 90, "Gesture puck cable (JST-GH) on the right edge, below the USB"),
    "J5": (3.5, 9.4, 0, "SWD header on the left edge: reachable with the board mounted"),
    "J6": (12.0, 27.0, 0, "ELM327 fallback header: an internal lead to the module inside the pod, so "
          "inboard, leaving the left edge to the programming controls"),
    "J2": (42.0, 4.5, 270, "IMU breakout header on the top edge: the breakout stands above the "
          "top-middle area on standoffs"),
    "J8": (46.0, 4.5, 0, "IMU reset pin beside J2: an optional jumper wire to the breakout's RST"),
    "J3": (60.5, 4.5, 270, "GNSS cable header at the top edge: the antenna lead goes up to the pod's roof face"),
    "H1": (4.0, 4.0, 0, "mounting hole (outlines.py)"), "H2": (66.0, 4.0, 0, "mounting hole"),
    "H3": (4.0, 38.5, 0, "mounting hole, above the DB-25 band"),
    "H4": (66.0, 38.5, 0, "mounting hole, above the DB-25 band"),
    # MCU and its supply pins
    "U1": (*MCU, 0, "STM32F405 in the middle: every subsystem fans out to it"),
    "C3": (26.2, 7.7, 0, "100 nF at VDD pin 64 / VSS 63 (top-left pins)"),
    "C4": (36.6, 12.5, 90, "100 nF at VDD pin 48 (right side)"),
    "C11": (36.6, 16.3, 90, "2.2 uF at VCAP2, pin 47: right at the pin"),
    "C7": (39.7, 16.5, 90, "4.7 uF bulk beside the MCU"),
    "C5": (23.9, 24.6, 90, "100 nF at VDD pin 19 / VSS 18 (bottom-left pins)"),
    "C10": (32.0, 25.6, 90, "2.2 uF at VCAP1, pin 31: right at the pin"),
    "C6": (34.3, 25.6, 90, "100 nF at VDD pin 32"),
    "C8": (15.6, 23.6, 90, "1 uF on VDDA, pin 13"),
    "C9": (19.0, 23.3, 0, "100 nF on VDDA, pin 13: the smaller capacitor nearer the pin"),
    # Clock, reset, boot
    "Y1": (17.6, 14.3, 90, "8 MHz crystal, upright, within 5 mm of OSC_IN/OUT (pins 5, 6); nothing routed under it"),
    "C12": (13.0, 18.55, 90, "crystal load capacitor beside Y1 pin 1"),
    "C13": (13.0, 10.05, 90, "crystal load capacitor beside Y1 pin 2"),
    # RESET and BOOT: side-push switches, plungers flush with the left edge, under the SWD header:
    # every programming control on one edge, pressed through pin-holes in the enclosure wall
    # (recessed, so not pressed by accident). Rotation 270 points the plunger out (-x).
    "SW1": (2.1, 24.2, 270, "RESET button on the left edge, below the SWD header"),
    "C14": (6.6, 22.0, 90, "NRST capacitor beside the RESET button"),
    "SW2": (2.1, 30.0, 270, "BOOT button (hold at reset for USB DFU) on the left edge, below RESET"),
    "R1": (9.5, 13.5, 90, "BOOT0 pull-down"),
    # Status LEDs on the top edge (they face the driver) with their resistors below
    "D3": (10.0, 2.1, 0, "POWER LED (green), top edge"), "R30": (10.0, 5.2, 0, "POWER LED resistor"),
    "D4": (14.2, 2.1, 0, "LINK LED (blue), top edge"), "R31": (14.2, 5.2, 0, "LINK LED resistor"),
    "D5": (18.4, 2.1, 0, "ARMED LED (amber), top edge"), "R32": (18.4, 5.2, 0, "ARMED LED resistor"),
    "D6": (22.6, 2.1, 0, "FAULT LED (red), top edge"), "R33": (22.6, 5.2, 0, "FAULT LED resistor"),
    # Power: USB VBUS and cable 5 V diode-OR'd, then the 3.3 V LDO (right of the MCU, near the USB)
    "D2": (49.5, 10.2, 0, "cable +5 V into the pod supply (diode-OR)"),
    "D1": (49.5, 14.5, 0, "USB VBUS into the pod supply (diode-OR)"),
    "C1": (49.5, 18.6, 0, "LDO input capacitor"),
    "U3": (49.5, 22.4, 0, "3.3 V LDO between the supply diodes and the MCU"),
    "C2": (54.5, 22.4, 90, "LDO output capacitor"),
    # USB
    "U4": (57.5, 13.5, 0, "USB ESD protection right at the socket's data pins"),
    "R2": (57.8, 9.3, 0, "USB-C CC1 5.1 k, at the socket"),
    "R3": (57.8, 17.8, 0, "USB-C CC2 5.1 k, at the socket"),
    "R4": (58.0, 21.3, 0, "USB shield bleed resistor"),
    "C15": (58.0, 23.8, 0, "USB shield capacitor"),
    # Sensor pull-ups beside their connectors
    "R5": (41.5, 8.5, 0, "IMU I2C pull-up (SCL), below J2"),
    "R6": (41.5, 11.0, 0, "IMU I2C pull-up (SDA), below J2"),
    "R7": (59.0, 27.2, 0, "gesture I2C pull-up (SCL), beside J4"),
    "R8": (59.0, 29.7, 0, "gesture I2C pull-up (SDA), beside J4"),
}

# Cable conditioning: each part above its own DB-25 pin (pod_board.py OUTPUTS / INPUTS).
_POD_CABLE = {
    # row A: row-1 pins' series parts
    "R40": (5, ROW_A), "R41": (7, ROW_A), "R42": (8, ROW_A), "R43": (9, ROW_A),
    "R64": (10, ROW_A), "R52": (12, ROW_A), "R55": (13, ROW_A),
    # row B: row-2 pins' series parts
    "R44": (14, ROW_B), "R45": (15, ROW_B), "R46": (16, ROW_B), "R47": (17, ROW_B),
    "R48": (18, ROW_B), "R58": (19, ROW_B), "R49": (20, ROW_B), "R61": (21, ROW_B),
    "R50": (22, ROW_B), "R63": (23, ROW_B), "R51": (24, ROW_B),
    # rows C and D: the inputs' pull-ups and filter capacitors, the current sense's 100 k
    "R53": (12, ROW_C), "R56": (13, ROW_C), "R59": (19, ROW_C), "R62": (21, ROW_C),
    "R66": (10, ROW_C),
    "C54": (12, ROW_D), "C57": (13, ROW_D), "C60": (19, ROW_D), "C65": (10, ROW_D),
}
for _ref, (_pin, _row) in _POD_CABLE.items():
    POD[_ref] = (round(pod_db25_x(_pin), 2), _row, 90,
                 f"cable line conditioning, in line with DB-25 pin {_pin}")

# --- Power board, 95 x 66 mm, 2 layers, 2 oz (compacted 2026-10-08 from 110 x 80; 5 mm wider on
# 2026-10-09 for the right-edge terminals' and module headers' silkscreen names) -----------------
# Holes: the top two in the corners; the bottom two moved UP beside the bottom terminal row, so
# the terminals can run from corner to corner.
PWR_W, PWR_H = 95.0, 66.0
# DB-25 on the top edge, opening up (rotated 180 degrees): pin 1 at the left end. Where it was
# when the board was 90 mm wide (centred then): the 5 mm added on the right is for labels.
PWR_DB25 = (45.0 - 16.62, 7.78)


def pwr_db25_x(pin):
    x0 = PWR_DB25[0]
    return x0 + 2.77 * (pin - 1) if pin <= 13 else x0 + 1.39 + 2.77 * (pin - 14)


# Terminal blocks: the wire openings face OUT over the board edge (checked in the 3D view), so a
# wire goes straight in through the box wall without crossing the board; the screws are on top.
# The footprint's openings face its +y side, so: bottom edge rotation 0, left edge 270, right edge
# 90. The footprint origin is pin 1; pins run left to right (bottom edge) and top to bottom (left
# edge), but bottom to top on the right edge. BOTTOM_T etc. is the pin row's distance in.
# The bottom row sits 0.55 mm further out (its opening face flush with the edge), clear of the
# bottom holes' courtyards.
BOTTOM_T, LEFT_T, RIGHT_T = PWR_H - 5.2, 5.75, PWR_W - 5.75
T_PITCH = 5.08


def right_t(top_y, n):
    """(x, y) of pin 1 for an n-pin terminal on the right edge whose top pin is at top_y."""
    return RIGHT_T, round(top_y + (n - 1) * T_PITCH, 2)

PWR = {
    "J1": (*PWR_DB25, 180, "DB-25 on the top edge, flange flush (the box's front-wall cut-out)"),
    "H1": (4.0, 4.0, 0, "mounting hole"), "H2": (91.0, 4.0, 0, "mounting hole"),
    "H3": (4.0, 51.5, 0, "mounting hole, above the bottom terminal row"),
    "H4": (91.0, 51.5, 0, "mounting hole, above the bottom terminal row"),
    # Input side, left: fuses at the top-left, the input and LiDAR terminals on the left edge
    "F1": (4.5, 10.0, 0, "3 A input fuse, top-left (reachable when the lid is off)"),
    "F3": (4.5, 18.0, 0, "3 A LiDAR fuse, below F1"),
    "J2": (LEFT_T, 27.92, 270, "12 V ACC input terminal on the left edge, below its fuse"),
    "J3": (LEFT_T, 39.42, 270, "LiDAR 12 V terminal on the left edge, below J2"),
    "D12": (16.4, 40.4, 0, "12 V present LED near the input"), "R40": (16.4, 43.2, 0, "its resistor"),
    "D10": (21.5, 15.5, 90, "TVS straight after the fuse: clamps transients before anything else"),
    "D11": (21.5, 26.5, 90, "reverse-polarity Schottky after the TVS"),
    "C20": (20.4, 38.0, 0, "47 uF bulk on +12 V before the converter"),
    # 5 V converter (TPS54360): the switching loop C21/C22 - U1 - D13 - L1 as tight as possible
    "U1": (36.0, 27.0, 270, "TPS54360 at the heart of the converter block, turned so its SW pin (8) is the corner nearest both D13 and L1: the switch node is a few mm, on the top layer"),
    "C21": (36.6, 20.5, 90, "input capacitor right above VIN (pin 2): the hot loop"),
    "C22": (40.0, 20.5, 90, "second input capacitor, beside the first"),
    "C23": (40.4, 27.0, 270, "bootstrap capacitor beside U1, between BOOT (pin 1) and SW (pin 8)"),
    "D13": (37.5, 35.0, 180, "catch diode right below the SW pin, cathode to it: the hot loop"),
    "L1": (48.5, 25.0, 0, "inductor on the SW side; keep the SW copper small"),
    "C24": (57.5, 22.0, 90, "output capacitor at the inductor's output"),
    "C25": (57.5, 28.0, 90, "second output capacitor"),
    # EN and RT leave U1's top edge to the left, then run down a 1.5 mm channel beside U1 to
    # these three, which stand left of U1 below the level of its top pins
    "R41": (30.0, 25.1, 0, "UVLO divider (top) left of U1, its EN pad on the channel"),
    "R42": (30.0, 27.2, 180, "UVLO divider (bottom), its EN pad on the channel"),
    "R43": (30.0, 29.3, 180, "RT resistor (600 kHz), its RT pad on the channel"),
    "R45": (29.8, 31.6, 180, "feedback divider (bottom) beside FB, pin 5, away from L1 and the SW node"),
    "R44": (29.8, 34.1, 0, "feedback divider (top): its FB pad right below R45's (one straight link)"),
    "R46": (29.8, 36.6, 180, "compensation resistor, its COMP pad facing pin 6"),
    "C26": (29.8, 41.6, 0, "compensation capacitor, below R46"),
    "C27": (29.8, 39.1, 180, "compensation high-frequency capacitor"),
    # The pod's supply, through the polyfuse, near DB-25 pins 1 and 2
    "F2": (34.0, 12.0, 0, "500 mA polyfuse for the pod's supply, near DB-25 pins 1-2"),
    "C30": (30.0, 19.8, 0, "5V_POD capacitor after the polyfuse"),
    # Presence detector near DB-25 pin 13
    "Q8": (62.0, 13.5, 0, "BOX_PRESENT MOSFET near DB-25 pin 13"),
    "R61": (66.5, 12.5, 0, "presence gate divider (top)"), "R62": (66.5, 15.0, 0, "presence gate divider (bottom)"),
    # Brake actuator side (right): terminals on the right edge, module headers inboard
    "J5": (*right_t(11.0, 2), 90, "actuator 12 V from the kill relay: right edge, the 10 A path starts here"),
    "J6": (*right_t(22.5, 2), 90, "to the BTS7960's B+/B-: next to J5, so the 10 A path is ~12 mm"),
    "J7": (*right_t(34.0, 2), 90, "cable magnet, on the same actuator rail"),
    "J13": (75.0, 11.5, 0, "BTS7960 logic header, beside its module (the box puts it next to this edge)"),
    "J14": (75.0, 35.5, 0, "ACS712 module header"),
    "U3": (67.0, 24.0, 0, "74HCT244 buffer between the DB-25 lines and the BTS7960 header"),
    "C31": (67.0, 32.5, 0, "74HCT244 decoupling"),
    "Q2": (63.0, 37.0, 0, "cable-magnet MOSFET"),
    "D14": (82.0, 45.5, 180, "magnet flyback diode right below the magnet terminal J7: its cathode on the actuator-rail zone, its anode facing inboard (clear of the rail) for MAG_NEG"),
    "R47": (70.0, 35.0, 0, "current-sense divider (top), by the ACS712 header"),
    "R48": (70.0, 37.5, 0, "current-sense divider (bottom)"),
    "J12": (58.0, 32.5, 0, "bench HX711 header (bench only), beside the converter's output"),
    # Car-wiring side (bottom edge, corner to corner): kill coil, E-stop, brake light, OBD, relays
    "J8": (3.6, BOTTOM_T, 0, "kill relay coil feed (via the E-stop)"),
    "J9": (15.12, BOTTOM_T, 0, "E-stop sense contact"),
    "J10": (26.72, BOTTOM_T, 0, "brake-light switch input"),
    "J11": (38.34, BOTTOM_T, 0, "OBD-II lead (CAN-H, CAN-L, GND)"),
    "J4": (55.0, BOTTOM_T, 0, "relay coils: common +12 V and five returns"),
    # Brake-light input above J10
    "U5": (24.0, 46.5, 0, "PC817 optocoupler above the brake-light terminal"),
    "R63": (17.2, 50.5, 0, "opto LED resistor"),
    "D21": (17.2, 47.0, 0, "opto LED reverse protection"),
    # CAN above the OBD terminal: the transceiver beside the lead (short stub)
    "U4": (44.0, 46.0, 0, "SN65HVD230 right above the OBD terminal: the bus stub is the lead only"),
    "D20": (51.0, 50.3, 0, "CAN ESD diode between the transceiver and the terminal"),
    "C32": (41.5, 50.5, 0, "transceiver decoupling, beside its VCC pin 3 (the right side stays clear for the CAN_STBY line to pin 8)"),
    "U2": (37.0, 47.0, 0, "3.3 V LDO for the transceiver"),
    "C28": (37.0, 43.2, 0, "LDO input capacitor"), "C29": (37.0, 50.8, 0, "LDO output capacitor"),
}

# Fail-safe pull-downs (and the CAN pull-ups) right behind the DB-25, each in line with its pin.
_PWR_PULLS = {"R50": 5, "R51": 7, "R52": 8, "R53": 9, "R54": 14, "R55": 15, "R56": 16, "R57": 17,
              "R58": 18, "R59": 20, "R60": 22}
for _ref, _pin in _PWR_PULLS.items():
    PWR[_ref] = (round(pwr_db25_x(_pin), 2), 12.5 if _pin <= 13 else 16.2, 90,
                 f"fail-safe resistor in line with DB-25 pin {_pin}, where the line enters the board")

# Relay drivers: each MOSFET and flyback diode one terminal pitch left of its J4 terminal (J4 pin 1,
# the coils' common +12 V, needs no driver), so the right end stays free for the magnet's D14.
for _i, _name in enumerate(["LEFT", "RIGHT", "HAZARD", "HORN", "BEAM"]):
    _x = round(55.0 + 5.08 * _i, 2)
    PWR[f"Q{3 + _i}"] = (_x, 52.0, 0, f"relay {_name} MOSFET, beside its J4 terminal")
    PWR[f"D{15 + _i}"] = (_x, 46.0, 90, f"relay {_name} flyback diode, beside its MOSFET")

PLANS = {"pod_board": POD, "power_board": PWR}


# --- Silkscreen labels -------------------------------------------------------------------------
# Every terminal, port, header and button is named on the top silkscreen, with its pins. Each entry:
# ref: dict(title, pins, side, at). The title includes the reference (it replaces the bare one);
# pins (by pad number) are named in line with each pin; both go beside the part's courtyard on
# `side` ("N", "S", "E" or "W": the inboard side for edge parts), unless `at` = (x, y, rotation)
# places the title. Header pins carry the MODULE's pin names, so a breakout plugs in by matching
# names (GNSS TX goes to the MCU's GPS_RX). Text: titles 1.0 mm, pin names 0.8 mm.


def L(title, pins=None, side="E", at=None, ref=True, size=1.0, shift=(0.0, 0.0)):
    """shift: (dx, dy) mm nudge of the title from its automatic place (clear of a hole, say)."""
    return dict(title=title, pins=pins or [], side=side, at=at, ref=ref, size=size, shift=shift)


LABELS = {
    "pod_board": {
        "J1": L("USB", side="W"),
        "J4": L("GESTURE", side="W"),
        "J5": L("SWD", ["3V3", "DIO", "CLK", "GND", "RST"], at=(4.2, 6.6, 0)),
        "J6": L("ELM327", ["VCC", "GND", "RXD", "TXD"], at=(12.5, 37.0, 0)),
        "J2": L("IMU", ["VIN", "3Vo", "GND", "SCL", "SDA", "INT"], side="S", at=(35.65, 1.6, 0)),
        "J8": L("RST", at=(46.0, 7.0, 0), size=0.8),
        "J3": L("GNSS", ["VCC", "GND", "TX", "RX", "PPS"], side="S", at=(55.42, 1.6, 0)),
        "J7": L("TO POWER BOX", at=(64.8, 46.6, 90), size=0.8),
        "SW1": L("RESET", at=(7.4, 24.2, 0), size=0.9), "SW2": L("BOOT", at=(7.1, 30.0, 0), size=0.9),
        # The status LEDs: named under their resistors. The row is too tight for references as
        # well: D3-D6 and R30-R33 are on the assembly (fab) layer only (HIDE_REFS).
        "D3": L("PWR", at=(10.0, 7.0, 0), ref=False, size=0.8),
        "D4": L("LINK", at=(14.2, 7.0, 0), ref=False, size=0.8),
        "D5": L("ARMED", at=(18.4, 7.0, 0), ref=False, size=0.8),
        "D6": L("FAULT", at=(22.6, 7.0, 0), ref=False, size=0.8),
    },
    "power_board": {
        "J1": L("TO POD", at=(79.5, 5.2, 0)),
        "F1": L("ACC 3A", side="N"), "F3": L("LIDAR 3A", side="S"),
        "J2": L("12V ACC IN", ["+", "-"]),
        "J3": L("LIDAR 12V", ["+", "-"]),
        "J5": L("ACT IN", ["+", "-"], side="W", shift=(0.25, 0)),
        "J6": L("BTS7960", ["+", "-"], side="W", shift=(0.25, 0)),
        "J7": L("MAGNET", ["+", "-"], side="W", shift=(0.25, 0)),
        "J8": L("KILL COIL", ["+12V", "GND"], side="N", size=0.9, shift=(3.3, 0.05)),
        "J9": L("E-STOP", ["SENSE", "GND"], side="N", size=0.9, shift=(0.8, 0)),
        "J10": L("BRAKE LIGHT", ["IN", "GND"], side="N", size=0.9),
        "J11": L("OBD-II", ["CANH", "CANL", "GND"], side="N"),
        "J4": L("RELAYS", ["+12V", "LEFT", "RIGHT", "HAZ", "HORN", "BEAM"], side="N", at=(64.0, 41.9, 0)),
        "J13": L("BTS7960", ["RPWM", "LPWM", "R_EN", "L_EN", "R_IS", "L_IS", "VCC", "GND"], at=(76.8, 8.8, 0)),
        "J14": L("ACS712", ["VCC", "OUT", "GND"], at=(75.9, 32.4, 0)),
        "J12": L("HX711", ["VCC", "DT", "SCK", "GND"], side="W"),
    },
}

# References kept off the silkscreen where there is no room (they stay on the fab layer).
HIDE_REFS = {"pod_board": ["D3", "D4", "D5", "D6", "R30", "R31", "R32", "R33"], "power_board": []}
