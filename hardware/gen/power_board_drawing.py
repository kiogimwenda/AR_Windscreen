"""How the power board's schematic is drawn: which sheet each part is on, where, which way round.

The circuit itself (parts, values, footprints, and which pin goes to which net) is power_board.py;
this file only arranges it, so a mistake here cannot change the circuit: generate.py checks that
every part is placed exactly once and that the drawn netlist equals the described one.

Coordinates are in units of 2.54 mm; rot is degrees counter-clockwise; mirror=True flips a symbol
left to right. Pin positions relative to a symbol at rot=0 (x right, y down, in units):
  R, C, fuse, inductor: pin 1 (0, -1.5) top, pin 2 (0, +1.5) bottom. rot=90: pin 1 on the left.
  diodes, LED, TVS: K (-1.5, 0), A (+1.5, 0). rot=180: anode left; rot=270: cathode on top.
  AO3400A / 2N7002: G (-2, 0), D (1, -2), S (1, +2).
  connectors: pins at x = -2 (mirror: +2), pin 1 at the top, 1 unit apart.
Conventions: signals flow left to right; pod-side ports on the left of each function sheet (the
pod is on the left of the block diagram); car-side connectors on the right.
"""

from kisheet import Sheet

# Direction of each cable signal, for the port arrows.
TO_BOX = ["BRAKE_RPWM", "BRAKE_LPWM", "BRAKE_EN", "MAGNET_EN", "RELAY_LEFT", "RELAY_RIGHT",
          "RELAY_HAZARD", "RELAY_HORN", "RELAY_BEAM", "CAN_TX", "CAN_STBY", "HX711_SCK"]
TO_POD = ["BRAKE_CURRENT", "KILL_SENSE", "BOX_PRESENT", "BRAKE_LIGHT", "CAN_RX", "HX711_DOUT"]


class Root:
    paper = "A3"

    def __init__(self):
        self.boxes, self.texts = [], []


def ports(s, nets, side="L", rows=None, pod=False):
    for i, n in enumerate(nets):
        if n == "5V_POD":
            shape = "input" if pod else "output"
        else:
            shape = ("output" if n in TO_BOX else "input") if pod else ("input" if n in TO_BOX else "output")
        s.port(n, side, rows[i] if rows else None, shape)


def draw(lib, parts):
    sheets = []

    def sheet(name, title, paper="A4"):
        s = Sheet(lib, parts, name, title, paper)
        sheets.append(s)
        return s

    # --- 1. 12 V input -----------------------------------------------------------------------------
    s = sheet("input", "12 V input and protection")
    s.add("J2", 14, 30, mirror=True, fields="left")
    s.add("F1", 26, 30, rot=90, fields="split")
    s.add("D10", 36, 35, rot=270)
    s.add("D11", 46, 30, rot=180, fields="split")
    s.add("C20", 58, 33)
    s.add("R40", 68, 33)
    s.add("D12", 68, 40, rot=90)
    s.rail("+12V", 76, 28)
    s.flag("+12V", 88, 24)
    s.flag("GND", 94, 24)
    for i in range(4):
        s.add(f"H{i + 1}", 16 + 8 * i, 64, fields="below")
    s.text(8, 10, "12 V INPUT (the logic feed): switched ACC from the car's fuse box, so the board is off\n"
                  "with the ignition. F1 3 A fuse. D10 bidirectional TVS clamps load-dump and jump-start\n"
                  "spikes to about 39 V. D11 Schottky (60 V, 5 A) blocks a reversed lead. D12: +12V present.", 1.5)
    s.text(14, 59, "Mounting holes: M3, no electrical connection", 1.4)

    # --- 2. 5 V converter ---------------------------------------------------------------------------
    s = sheet("buck", "5 V converter (TPS54360, TI reference design)")
    s.add("U1", 52, 40, fields="aboveleft")
    s.rail("+12V", 17, 36, refs=["U1", "C21", "C22", "R41"])
    s.add("C21", 20, 41)
    s.add("C22", 27, 41)
    s.add("R41", 35, 41)
    s.add("R42", 35, 47)
    s.add("R43", 42, 40, rot=270, fields="split")
    s.add("C23", 59, 38.5)
    s.add("D13", 64, 42.5, rot=270)
    s.add("L1", 69, 40, rot=90, fields="split")
    s.rail("+5V", 88, 35, refs=["L1", "C24", "C25", "R44"])
    s.add("C24", 74, 42.5)
    s.add("C25", 78, 42.5)
    s.add("R44", 83, 42.5)
    s.add("R45", 83, 48.5)
    s.add("C27", 59, 53)
    s.add("R46", 65, 51.5, rot=90, fields="split")
    s.add("C26", 69, 53)
    s.flag("+5V", 96, 31)
    s.text(8, 10, "5 V: TI's TPS54360 5 V / 3.5 A reference design (datasheet SLVSBB4G, Figure 34), unchanged.\n"
                  "Rated to 60 V input, so whatever the TVS lets through a load dump cannot destroy it.\n"
                  "R41/R42 start it at 8 V and stop it at 6.25 V. R43 sets 600 kHz. R44/R45 set 5.0 V.\n"
                  "C23 is the bootstrap for the high-side switch; D13 carries the current while it is off.\n"
                  "R46, C26, C27 compensate the control loop. Lay out C21/C22, U1, D13 and L1 as one tight loop.", 1.5)

    # --- 3. 3.3 V, pod supply, LiDAR feed --------------------------------------------------------
    s = sheet("rails", "3.3 V, the pod's 5 V, the LiDAR feed")
    s.add("U2", 40, 30, fields="above")
    s.rail("+5V", 24, 25, refs=["U2", "C28"])
    s.add("C28", 30, 32)
    s.rail("+3V3", 54, 25, refs=["U2", "C29"])
    s.add("C29", 48, 32)
    s.add("F2", 36, 48, rot=90, fields="split")
    s.add("C30", 44, 51)
    ports(s, ["5V_POD"], side="R", rows=[48])
    s.add("F3", 36, 64, rot=90, fields="split")
    s.add("J3", 52, 64, fields="right")
    s.text(8, 10, "3.3 V (AP2112K) for the CAN transceiver only; the pod makes its own 3.3 V.\n"
                  "5V_POD: the pod's supply up the cable, through F2, a 500 mA resettable fuse,\n"
                  "so a short in the cable cannot take down this board's 5 V.\n"
                  "LiDAR: 12 V through its own 3 A fuse F3 to the M12 socket.", 1.5)

    # --- 4. Brake actuator --------------------------------------------------------------------------
    s = sheet("actuator", "Brake actuator: buffer, BTS7960, current sense, cable magnet", "A3")
    ports(s, ["BRAKE_RPWM", "BRAKE_LPWM", "BRAKE_EN", "MAGNET_EN", "BRAKE_CURRENT"],
          rows=[16, 25, 34, 75, 92])
    s.add("R50", 24, 19)
    s.add("R51", 24, 28)
    s.add("R52", 24, 37)
    s.add("U3", 64, 42, fields="aboveleft")
    s.rail("GND", 54, 53, refs=["U3"])
    s.add("C31", 74, 29)
    s.add("J13", 88, 40, fields="right")
    s.add("J5", 22, 61, mirror=True, fields="left")
    s.add("J6", 88, 60, fields="right")
    s.add("D14", 72, 68, rot=270)
    s.add("J7", 88, 72, fields="right")
    s.add("Q2", 60, 77, fields="right")
    s.add("R53", 48, 78)
    s.add("J14", 88, 92, fields="right")
    s.add("R47", 72, 92, rot=270, fields="split")
    s.add("R48", 60, 95)
    s.text(8, 9, "BRAKE ACTUATOR. Every line from the pod has a 10 k pull-down here (R50-R53): with the pod in reset,\n"
                 "unpowered or unplugged, the drive, enable and magnet lines read LOW: released, cable dropped free.\n"
                 "U3 (74HCT244, TTL thresholds) turns the pod's 3.3 V logic into the 5 V the BTS7960 module expects.", 1.5)
    s.text(100, 56, "Actuator power comes ONLY through the 40 A kill relay's\n"
                    "contacts (J5), whose coil is fed through the E-stop.\n"
                    "Q2 holds the cable magnet; D14 absorbs its turn-off spike.", 1.5)
    s.text(100, 88, "ACS712-20A module: 2.5 V at 0 A, 100 mV/A.\n"
                    "R47/R48 scale it by 2/3 so nothing above\n"
                    "3.3 V travels up the cable to the pod's ADC.", 1.5)

    # --- 5. Signal relays ------------------------------------------------------------------------------
    s = sheet("relays", "Signal relays: indicators, hazards, horn, high beam", "A3")
    names = ["LEFT", "RIGHT", "HAZARD", "HORN", "BEAM"]
    for i, n in enumerate(names):
        yb = 22 + 15 * i
        ports(s, [f"RELAY_{n}"], rows=[yb])
        s.add(f"R{54 + i}", 26, yb + 3)
        s.add(f"Q{3 + i}", 50, yb, fields="right")
        s.add(f"D{15 + i}", 60, yb - 5, rot=270)
    s.add("J4", 100, 52, fields="right")
    s.text(8, 9, "SIGNAL RELAYS: five 12 V automotive relays in sockets, off the board; their contacts tap the car's\n"
                 "switch wiring (Part 15.2). Each coil is switched low-side by a MOSFET whose gate has a 10 k pull-down\n"
                 "(R54-R58): no drive from the pod, no relay. D15-D19 absorb each coil's turn-off spike.", 1.5)

    # --- 6. CAN --------------------------------------------------------------------------------------------
    s = sheet("can", "CAN transceiver for OBD-II")
    ports(s, ["CAN_TX", "CAN_RX", "CAN_STBY"], rows=[39, 40, 50])
    s.add("R59", 30, 35)
    s.add("R60", 30, 46)
    s.add("U4", 50, 40, fields="aboveleft")
    s.add("C32", 42, 30)
    s.add("D20", 62, 47, fields="right")
    s.add("J11", 80, 41, fields="right")
    s.text(8, 10, "CAN (ISO 15765-4) to the car's OBD-II port, on this board so the unterminated branch off the\n"
                  "car's bus is only the ~0.5 m OBD lead. R59 holds TX recessive and R60 holds Rs high (standby,\n"
                  "cannot transmit) whenever the pod is not driving them. No termination: the car has its two.", 1.5)

    # --- 7. Status inputs and bench ---------------------------------------------------------------------
    s = sheet("status", "Presence, brake light, E-stop sense, kill relay, bench", "A3")
    ports(s, ["BOX_PRESENT", "BRAKE_LIGHT", "KILL_SENSE", "HX711_DOUT", "HX711_SCK"],
          rows=[22, 40, 58, 77, 78])
    s.add("Q8", 40, 24, mirror=True, fields="left")
    s.add("R61", 48, 21)
    s.add("R62", 48, 29)
    s.add("J10", 104, 40, fields="right")
    s.add("R63", 92, 40, rot=270, fields="split")
    s.add("U5", 74, 41, mirror=True, fields="above")
    s.add("D21", 84, 44, rot=270)
    s.add("J9", 104, 58, fields="right")
    s.add("J8", 104, 66, fields="right")
    s.add("J12", 104, 77, fields="right")
    s.text(8, 9, "Inputs that report to the pod. Car side on the right, pod side (the ports) on the left.", 1.5)
    s.text(56, 16, "BOX_PRESENT: Q8 pulls it low only while this board's 5 V is up,\n"
                   "so the pod sees 'present' only when the box is connected AND powered.", 1.4)
    s.text(56, 34, "Brake light: the opto-coupler U5 isolates the car's 12 V brake-light\n"
                   "circuit; D21 protects its LED from reverse voltage.", 1.4)
    s.text(56, 53, "E-stop second contact (NC): released pulls KILL_SENSE low.", 1.4)
    s.text(56, 63, "J8: +12V -> E-stop NC contact -> 40 A kill relay coil -> GND (wired off the board).", 1.4)
    s.text(56, 73, "J12: HX711 load-cell amplifier, bench rig only.", 1.4)

    # --- 8. Pod interface ---------------------------------------------------------------------------------
    s = sheet("cable", "Inter-box cable: DB-25 to the pod")
    s.add("J1", 26, 46, mirror=True, fields="above")
    nets = ["5V_POD"] + TO_BOX + TO_POD
    ports(s, nets, side="R", pod=True)
    s.port_x = {"R": 70 * 2.54}
    s.text(8, 10, "INTER-BOX CABLE (BUILD_GUIDE 4.8.3). Shield bonded to GND at this end only.\n"
                  "Every line driven by the pod has its fail-safe pull-down on the sheet that uses it.", 1.5)

    # --- Root: the block diagram -----------------------------------------------------------------------
    by = {x.name: x for x in sheets}
    root = Root()
    act = ["BRAKE_RPWM", "BRAKE_LPWM", "BRAKE_EN", "MAGNET_EN", "BRAKE_CURRENT"]
    rel = ["RELAY_LEFT", "RELAY_RIGHT", "RELAY_HAZARD", "RELAY_HORN", "RELAY_BEAM"]
    can = ["CAN_TX", "CAN_RX", "CAN_STBY"]
    sta = ["BOX_PRESENT", "BRAKE_LIGHT", "KILL_SENSE", "HX711_DOUT", "HX711_SCK"]
    gap = [None] * 5
    pod_pins = (["5V_POD"] + [None] * 3 + act + gap + rel + gap + can + gap + sta)
    root.boxes.append((by["cable"], 20, 22, 26, [(n, "R") if n else None for n in pod_pins]))
    # each box sits level with its pins on the pod box, so every wire is straight
    first = {n: i for i, n in enumerate(pod_pins) if n}
    root.boxes.append((by["rails"], 76, 22 + first["5V_POD"], 34, [("5V_POD", "L")]))
    for name, pins in (("actuator", act), ("relays", rel), ("can", can), ("status", sta)):
        root.boxes.append((by[name], 76, 22 + first[pins[0]], 34, [(n, "L") for n in pins]))
    root.boxes.append((by["input"], 76, 10, 16, [], 4))
    root.boxes.append((by["buck"], 96, 10, 16, [], 4))
    root.texts.append((20, 8, "UNDER-DASH POWER BOARD: block diagram. Each box is a sheet (double-click it in KiCad).\n"
                              "Power rails +12V, +5V, +3V3 and GND join the sheets by name (power symbols);\n"
                              "every other connection between sheets is a wire here.", 1.8))
    root.texts.append((118, 30, "12 V input -> 5 V converter -> 3.3 V and the pod's 5 V.\n"
                                "The pod drives the actuator, relays and CAN;\n"
                                "the status sheet reports back to it.", 1.5))
    return sheets, root
