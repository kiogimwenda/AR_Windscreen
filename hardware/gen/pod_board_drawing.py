"""How the pod board's schematic is drawn: which sheet each part is on, where, which way round.

The circuit itself is pod_board.py; this file only arranges it (see power_board_drawing.py for the
coordinate conventions). generate.py checks every part is placed once, and check.sh that the drawn
netlist equals the label-only one.

Conventions: the MCU sheet has the STM32 in the middle with each pin running straight out to a
port; the sheets around it are drawn with the MCU side on the left and the outside world (USB,
breakout headers, the cable) on the right, except USB, whose connector is on the left.
"""

from pod_board import INPUTS, OUTPUTS
from power_board_drawing import Root
from kisheet import Sheet


def find(parts, sym, nets):
    """The reference of the part with this symbol joining exactly these nets."""
    hits = [r for r, p in parts.items() if p["sym"] == sym and set(p["nets"].values()) == set(nets)]
    assert len(hits) == 1, (sym, nets, hits)
    return hits[0]


def draw(lib, parts):
    sheets = []

    def sheet(name, title, paper="A4"):
        s = Sheet(lib, parts, name, title, paper)
        sheets.append(s)
        return s

    R = lambda a, b: find(parts, "Device:R", [a, b])  # noqa: E731
    C = lambda a, b: find(parts, "Device:C", [a, b])  # noqa: E731

    # --- 1. Power -----------------------------------------------------------------------------------
    s = sheet("power", "Pod supply: USB or cable 5 V, 3.3 V regulator")
    s.rail("VBUS", 14, 27, refs=["D1"])
    s.add("D1", 24, 30, rot=180, fields="split")
    s.add("D2", 24, 38, rot=180, fields="split")
    s.port("X_5V_CABLE", "L", 38, "input")
    s.add("C1", 34, 34)
    s.add("U3", 46, 31, fields="above")
    s.rail("+3V3", 64, 27, refs=["U3", "C2", "R30"])
    s.add("C2", 54, 34)
    s.add("R30", 62, 34)
    s.add("D3", 62, 42, rot=90)
    s.flag("VIN_POD", 30, 24)
    s.flag("X_5V_CABLE", 18, 33)
    s.flag("VBUS", 80, 22)
    s.flag("GND", 86, 22)
    for i in range(4):
        s.add(f"H{i + 1}", 16 + 8 * i, 64, fields="below")
    s.text(8, 10, "POD SUPPLY: two 5 V sources, diode-OR'd: the laptop's USB (D1) and the power box through the cable\n"
                  "(D2). Either one runs the pod, so it can be programmed and tested on USB alone. U3 makes 3.3 V\n"
                  "for the MCU and sensors. D3: 3.3 V present.", 1.5)
    s.text(14, 59, "Mounting holes: M3, no electrical connection", 1.4)

    # --- 2. USB ----------------------------------------------------------------------------------------
    s = sheet("usb", "USB-C to the laptop")
    s.add("J1", 20, 40, fields="above")
    s.add("R3", 31, 37, rot=90, fields="split")
    s.add("R2", 38, 36, rot=90, fields="split")
    s.add("U4", 52, 39, fields="belowright")
    # U4 joins its pins 1-6 and 3-4 inside: connector side and port side drawn apart, no wire around it
    s.split("USB_DM", [("J1", "A7"), ("J1", "B7"), ("U4", "1")])
    s.split("USB_DP", [("J1", "A6"), ("J1", "B6"), ("U4", "3")])
    for n, row in (("USB_DM", 39), ("USB_DP", 40)):
        s.port(n, "R", row, "bidirectional")
    s.add("R4", 12, 55)
    s.add("C15", 7, 55)
    s.text(8, 10, "USB-C: the Part 3 link to the laptop, and the pod's power when the cable is out.\n"
                  "R2/R3 (5.1 k on CC) tell the laptop this is a device that wants 5 V. U4 clamps ESD on D+/D-\n"
                  "and VBUS. The shell goes to ground through R4/C15, so it drains static without a ground loop.", 1.5)

    # --- 3. MCU -------------------------------------------------------------------------------------
    s = sheet("mcu", "MCU: STM32F405RGT6", "A3")
    u1 = s.add("U1", 82, 58, fields="belowright")
    for num, net in u1.nets.items():  # each pin's port on its own side of the sheet
        if u1.pins[num]["out"] == "R":
            s.port(net, "R")
    s.rail("+3V3", 88, 35, refs=["U1"])
    s.port("NRST", "L", 22)
    s.add("C14", 30, 25)
    s.add("SW1", 36, 25, rot=270, fields="right")
    s.add("SW2", 48, 30, rot=90, fields="right")
    s.add("R1", 52, 35)
    s.add("Y1", 64, 37, rot=180, fields="split")
    s.add("C12", 68, 41)
    s.add("C13", 60, 40)
    # lines coming down to the left pins: the higher the pin, the nearer the chip its column
    s.columns.update({"NRST": 73, "BOOT0": 72, "OSC_IN": 71, "OSC_OUT": 70})
    s.add("C10", 66, 72)
    s.add("C11", 70, 72)
    for i, ref in enumerate(["C3", "C4", "C5", "C6", "C7", "C8", "C9"]):
        s.add(ref, 104 + 5 * i, 22)
    s.port_x = {"R": 128 * 2.54}
    s.text(8, 9, "MCU. Every pin assignment equals firmware Config.h (checked by check_nets.py). Each pin runs\n"
                 "straight out to the sheet that uses it. Left: reset, boot, the 8 MHz crystal, the core regulator's\n"
                 "capacitors (VCAP), and port C. Right: ports A and B.", 1.5)
    s.text(100, 14, "Decoupling: C3-C6 100 nF, one beside each VDD pin;\n"
                    "C7 4.7 uF bulk; C8 1 uF + C9 100 nF at VDDA.", 1.4)
    s.text(8, 16, "SW1 RESET; SW2 BOOT: hold at reset for the USB DFU bootloader.\n"
                  "R1 holds BOOT0 low: run from flash.", 1.4)

    # --- 4. Sensors and local connectors --------------------------------------------------------------
    s = sheet("sensors", "Sensors and local connectors", "A3")
    # IMU: SCL at the port's row; SDA, INT, RST come up from lower ports
    s.add("J2", 100, 22, fields="right")
    s.port("IMU_SCL", "L", 22)
    s.port("IMU_SDA", "L", 30)
    s.port("IMU_INT", "L", 32)
    s.port("IMU_RST", "L", 33)
    s.add("R5", 60, 19.5)
    s.add("R6", 66, 27.5)
    s.add("J3", 100, 42, fields="right")
    s.add("J4", 100, 58, fields="right")
    s.port("GEST_SCL", "L", 58)
    s.port("GEST_SDA", "L", 66)
    s.port("GEST_INT", "L", 68)
    s.add("R7", 60, 55.5)
    s.add("R8", 66, 63.5)
    s.add("J5", 100, 78, fields="right")
    s.add("J6", 100, 92, fields="right")
    s.text(8, 9, "SENSORS AND LOCAL CONNECTORS. Breakouts on standoffs; MATCH EACH HEADER'S PIN ORDER TO THE\n"
                 "BREAKOUT YOU BUY before layout. Names are the MCU's view (GPS_TX goes to the module's RX).", 1.5)
    s.text(112, 20, "BNO085 IMU (I2C1, 4.7 k pull-ups R5/R6).\nMount rigidly: it measures the car's motion.", 1.4)
    s.text(112, 40, "NEO-M8N GNSS (USART2, PPS for timing).", 1.4)
    s.text(112, 56, "Gesture puck: APDS-9960 on ~1.5 m of cable\n(I2C2 at 100 kHz; 2.2 k pull-ups for the cable).", 1.4)
    s.text(112, 77, "SWD: ST-Link programming and debugging.", 1.4)
    s.text(112, 91, "ELM327 fallback (USART3, HC-05 bridge):\nbuild env hub_elm327 only (Part 4.8.5).", 1.4)

    # --- 5. Status LEDs ------------------------------------------------------------------------------------
    s = sheet("leds", "Status LEDs (driver-facing edge)")
    for i, net in enumerate(["LED_LINK", "LED_ARMED", "LED_FAULT"]):
        y = 28 + 10 * i
        s.port(net, "L", y)
        s.add(R(net, net + "_A"), 40, y, rot=90, fields="split")
        s.add(f"D{4 + i}", 48, y + 3, rot=90)
    s.text(8, 10, "STATUS LEDs on the board's top edge, facing the driver: LINK (blue) the laptop is talking,\n"
                  "ARMED (amber) the hub accepts brake commands, FAULT (red) a latched fault (Part 4.5).", 1.5)

    # --- 6. Cable interface ----------------------------------------------------------------------------
    s = sheet("cable", "Inter-box cable: DB-25, series resistors, input filters", "A3")
    j7 = s.add("J7", 112, 57, fields="above")
    s.port("X_5V_CABLE", "L", 16, "output")
    rows = {}
    y = 19
    order = ["RELAY_LEFT", "RELAY_RIGHT", "RELAY_HAZARD", "RELAY_HORN", "BRAKE_RPWM", "RELAY_BEAM",
             "BRAKE_LIGHT", "BRAKE_LPWM", "CAN_TX", "BRAKE_EN", "CAN_RX", "MAGNET_EN", "CAN_STBY",
             "BRAKE_CURRENT", "HX711_DOUT", "HX711_SCK", "KILL_SENSE", "BOX_PRESENT"]  # DB-25 order
    for sig in order:
        above = sig in INPUTS and INPUTS[sig][2]       # pull-up above the line
        below = (sig in INPUTS and INPUTS[sig][3]) or sig == "BRAKE_CURRENT"
        y += 5.5 if above else 0
        rows[sig] = y
        x_sig = "X_" + sig
        if sig in OUTPUTS:
            s.port(sig, "L", y, "input")
            s.add(R(sig, x_sig), 54, y, rot=90, fields="split")
        else:
            s.port(sig, "L", y, "output")
            s.add(R(x_sig, sig), 54, y, rot=270, fields="split")
            if above:
                s.add(R("+3V3", x_sig), 64, y - 2.5)
            if below and sig != "BRAKE_CURRENT":
                s.add(C(sig, "GND"), 44, y + 2.5)
        if sig == "BRAKE_CURRENT":
            s.add(C(sig, "GND"), 44, y + 2.5)
            s.add(R(x_sig, "GND"), 64, y + 2.5)
        y += (7 if below else 3)
    s.port_x = {"L": 12 * 2.54}
    # Fan-in to the DB-25 without crossings: lines above a pin turn down, the topmost nearest the
    # connector; lines below turn up, the bottommost nearest.
    jrow = {}
    for num, net in j7.nets.items():
        jrow.setdefault(net, j7.pins[num]["cell"][1] * 1.27 / 2.54)
    xs = [("X_5V_CABLE", 16)] + [("X_" + n, rows[n]) for n in order]
    above = [(n, r) for n, r in xs if r < jrow[n]]
    below = [(n, r) for n, r in xs if r > jrow[n]]
    for i, (n, _) in enumerate(above):
        s.columns[n] = 106 - i
    for i, (n, _) in enumerate(reversed(below)):
        s.columns[n] = 106 - i
    s.text(8, 9, "INTER-BOX CABLE (BUILD_GUIDE 4.8.3). MCU side on the left, the DB-25 on the right.\n"
                 "Outputs: 1 k in series (330 R on CAN_TX), so a pod on USB cannot push more than ~3 mA into an\n"
                 "unpowered power box. Inputs: a 10 k pull-up on the cable side gives the SAFE reading with the\n"
                 "cable out (kill engaged, box absent, brake light off, CAN recessive); 1 k + 10 nF filter PWM noise.\n"
                 "BRAKE_CURRENT: 1 k + 100 nF (1.6 kHz) removes the 20 kHz ripple; 100 k reads 0 V with the cable out.", 1.5)

    # --- Root: the block diagram -----------------------------------------------------------------------
    by = {x.name: x for x in sheets}
    root = Root()
    usb = ["USB_DM", "USB_DP"]
    sen = ["IMU_SCL", "IMU_SDA", "IMU_INT", "IMU_RST", "GPS_RX", "GPS_TX", "GPS_PPS", "GEST_SCL",
           "GEST_SDA", "GEST_INT", "SWDIO", "SWCLK", "NRST", "OBD_TX", "OBD_RX"]
    led = ["LED_LINK", "LED_ARMED", "LED_FAULT"]
    cab = order
    left = usb + [None] * 6 + sen + [None] * 6 + led
    right = [None] * 2 + cab
    my = 24
    root.boxes.append((by["mcu"], 70, my, 30, zip_sides(left, right)))
    first = {n: i for i, n in enumerate(left) if n}
    root.boxes.append((by["usb"], 22, my + first["USB_DM"], 24, [(n, "R") for n in usb]))
    root.boxes.append((by["sensors"], 22, my + first["IMU_SCL"], 24, [(n, "R") for n in sen]))
    root.boxes.append((by["leds"], 22, my + first["LED_LINK"], 24, [(n, "R") for n in led]))
    firstr = {n: i for i, n in enumerate(right) if n}
    root.boxes.append((by["cable"], 124, my + firstr[cab[0]] - 2, 30,
                       [("X_5V_CABLE", "L"), None] + [(n, "L") for n in cab]))
    root.boxes.append((by["power"], 124, my - 10, 30, [("X_5V_CABLE", "L")]))
    root.texts.append((20, 8, "WINDSCREEN POD BOARD: block diagram. Each box is a sheet (double-click it in KiCad).\n"
                              "Power rails +3V3, VBUS and GND join the sheets by name (power symbols);\n"
                              "every other connection between sheets is a wire here.", 1.8))
    return sheets, root


def zip_sides(left, right):
    """Pins for a box with two independent columns: KiCad sheet pins can sit anywhere on an edge,
    so a row may have a pin on the left, the right, both or neither."""
    out = []
    for i in range(max(len(left), len(right))):
        row = []
        if i < len(left) and left[i]:
            row.append((left[i], "L"))
        if i < len(right) and right[i]:
            row.append((right[i], "R"))
        out.append(row)
    return out
