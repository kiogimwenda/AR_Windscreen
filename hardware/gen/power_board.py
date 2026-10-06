"""The under-dash power board (BUILD_GUIDE Part 4.8.4). 2-layer, 2 oz copper, about 100 x 80 mm.

The rule that makes the two-box split safe: every line the pod drives enters this board through a
10 k pull-down (or, for the CAN transceiver's TX and standby inputs, a pull-up to their silent
state). MCU in reset, unpowered, or the cable unplugged: brake drive, magnet and relays are off and
the transceiver cannot drive the car's bus, in hardware. check_nets.py verifies every one.

Off-board in the power box, wired to terminals here: the 40 A kill relay and its socket, the
E-stop, the five signal relays in sockets, the BTS7960 module's power terminals, the ACS712
module's current path, the actuator and cable magnet, and the OBD-II lead.

The 5 V converter is TI's TPS54360 5 V / 3.5 A reference design (datasheet SLVSBB4G, Figure 34),
unchanged: 8.5-60 V in, 600 kHz, start at 8 V / stop at 6.25 V.
"""

from parts import (C, C0805, C1206, C1210, CONN, D, DSUB25, HOLES, LED, NMOS, R, SMA, SMC,
                   SOD123, header)

groups = []

# --- 12 V input and protection ----------------------------------------------------------------
inp = [
    CONN("J2", "12 V ACC in", ["ACC_IN", "GND"], kind="screw",
         desc="switched ACC (ignition) feed; its own fuse at the car's fuse box too"),
    {"sym": "Device:Fuse", "ref": "F1", "value": "3A", "fp": "Fuse:Fuseholder_Blade_Mini_Keystone_3568",
     "nets": {"1": "ACC_IN", "2": "ACC_FUSED"}},
    {"sym": "Device:D_TVS", "ref": "D10", "value": "SMCJ24CA", "fp": SMC,
     "desc": "load-dump / jump-start transient clamp (bidirectional: also survives reversed leads)",
     "nets": {"1": "ACC_FUSED", "2": "GND"}},
    D("D11", "B560C", "ACC_FUSED", "+12V", fp=SMC, sym="Device:D_Schottky",
      desc="reverse-polarity protection: 60 V, 5 A; ~0.6 V drop, ~1 W at 2 A"),
    {"sym": "Device:C_Polarized", "ref": "C20", "value": "47u 63V",
     "fp": "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm", "desc": "bulk: damps the harness inductance",
     "nets": {"1": "+12V", "2": "GND"}},
    R("R40", "4.7k", "+12V", "LED12_A"), LED("D12", "green", "LED12_A", "GND"),
]
groups.append(("12 V input: fuse, TVS, reverse-polarity diode", inp, 4))

# --- TPS54360 5 V converter (TI reference design) --------------------------------------------
buck = [
    {"sym": "Regulator_Switching:TPS54360DDA", "ref": "U1", "value": "TPS54360DDA",
     "fp": "Package_SO:TI_SO-PowerPAD-8_ThermalVias", "desc": "60 V, 3.5 A buck",
     "nets": {"1": "BOOT", "2": "+12V", "3": "BUCK_EN", "4": "BUCK_RT", "5": "BUCK_FB",
              "6": "BUCK_COMP", "7": "GND", "8": "BUCK_SW", "9": "GND"}},
    C("C21", "2.2u 100V", "+12V", "GND", C1210), C("C22", "2.2u 100V", "+12V", "GND", C1210),
    R("R41", "523k", "+12V", "BUCK_EN", desc="UVLO: start 8 V"),
    R("R42", "84.5k", "BUCK_EN", "GND", desc="UVLO: stop 6.25 V"),
    R("R43", "162k", "BUCK_RT", "GND", desc="600 kHz"),
    C("C23", "100n", "BOOT", "BUCK_SW"),
    D("D13", "B560C", "GND", "BUCK_SW", fp=SMC, sym="Device:D_Schottky", desc="catch diode"),
    {"sym": "Device:L", "ref": "L1", "value": "8.2u", "fp": "Inductor_SMD:L_Bourns_SRR1260",
     "desc": "SRR1260-8R2Y or similar, Isat > 4.5 A", "nets": {"1": "BUCK_SW", "2": "+5V"}},
    C("C24", "47u", "+5V", "GND", C1210), C("C25", "47u", "+5V", "GND", C1210),
    R("R44", "53.6k", "+5V", "BUCK_FB"), R("R45", "10.2k", "BUCK_FB", "GND"),
    R("R46", "13.0k", "BUCK_COMP", "BUCK_COMP_RC"), C("C26", "6800p", "BUCK_COMP_RC", "GND"),
    C("C27", "39p", "BUCK_COMP", "GND"),
]
groups.append(("5 V: TPS54360 (TI reference design, 600 kHz)", buck, 5))

# --- 3.3 V and the pod's supply ----------------------------------------------------------------
rails = [
    {"sym": "Regulator_Linear:AP2112K-3.3", "ref": "U2", "value": "AP2112K-3.3",
     "fp": "Package_TO_SOT_SMD:SOT-23-5", "desc": "3.3 V for the CAN transceiver",
     "nets": {"1": "+5V", "2": "GND", "3": "+5V", "5": "+3V3"}},
    C("C28", "10u", "+5V", "GND", C1206), C("C29", "10u", "+3V3", "GND", C1206),
    {"sym": "Device:Polyfuse", "ref": "F2", "value": "500mA", "fp": "Fuse:Fuse_1206_3216Metric",
     "desc": "the pod's supply up the cable", "nets": {"1": "+5V", "2": "5V_POD"}},
    C("C30", "10u", "5V_POD", "GND", C1206),
    {"sym": "Device:Fuse", "ref": "F3", "value": "3A", "fp": "Fuse:Fuseholder_Blade_Mini_Keystone_3568",
     "desc": "LiDAR feed", "nets": {"1": "+12V", "2": "LIDAR_12V"}},
    CONN("J3", "LiDAR 12 V out", ["LIDAR_12V", "GND"], kind="screw"),
]
groups.append(("3.3 V, pod supply, LiDAR feed", rails, 4))

# --- Brake actuator: buffer, BTS7960, ACS712, magnet ------------------------------------------
act = [
    {"sym": "74xx:74HCT244", "ref": "U3", "value": "74HCT244",
     "fp": "Package_SO:SOIC-20W_7.5x12.8mm_P1.27mm",
     "desc": "TTL-threshold buffer: 3.3 V logic in, 5 V out to the BTS7960 module",
     "nets": {"20": "+5V", "10": "GND", "1": "GND", "19": "GND",
              "2": "BRAKE_RPWM", "18": "RPWM_5V", "4": "BRAKE_LPWM", "16": "LPWM_5V",
              "6": "BRAKE_EN", "14": "EN_5V",
              "8": "GND", "17": "GND", "15": "GND", "13": "GND", "11": "GND"}},
    C("C31", "100n", "+5V", "GND"),
    CONN("J13", "BTS7960 module", ["RPWM_5V", "LPWM_5V", "EN_5V", "EN_5V", None, None, "+5V", "GND"],
         desc="module header: RPWM LPWM R_EN L_EN R_IS L_IS VCC GND (R_EN and L_EN tied)"),
    CONN("J6", "BTS7960 B+ / B-", ["ACT_12V", "GND"], kind="screw",
         desc="the module's motor supply, from the kill relay"),
    CONN("J5", "Actuator 12 V from kill relay", ["ACT_12V", "GND"], kind="screw",
         desc="the 40 A kill relay's contacts (BOM 3.5); its coil via the E-stop (J8)"),
    CONN("J14", "ACS712 module", ["+5V", "ACS_OUT", "GND"]),
    R("R47", "10k", "ACS_OUT", "BRAKE_CURRENT", desc="x2/3 divider: nothing above 3.3 V on the cable"),
    R("R48", "20k", "BRAKE_CURRENT", "GND"),
    NMOS("Q2", "AO3400A", "MAGNET_EN", "GND", "MAG_NEG"),
    D("D14", "SS14", "MAG_NEG", "ACT_12V", fp=SMA, sym="Device:D_Schottky", desc="flyback"),
    CONN("J7", "Cable magnet", ["ACT_12V", "MAG_NEG"], kind="screw",
         desc="12 V holding magnet; powered only through the kill relay"),
]
groups.append(("Brake actuator: buffer, BTS7960, current sense, cable magnet", act, 4))

# --- Signal relays ----------------------------------------------------------------------------
relays = [CONN("J4", "Relay coils", ["+12V", "COIL_LEFT", "COIL_RIGHT", "COIL_HAZARD",
                                     "COIL_HORN", "COIL_BEAM"], kind="screw",
               desc="five 12 V automotive relays in sockets; coil common to +12V")]
for i, name in enumerate(["LEFT", "RIGHT", "HAZARD", "HORN", "BEAM"]):
    relays += [NMOS(f"Q{3 + i}", "AO3400A", f"RELAY_{name}", "GND", f"COIL_{name}"),
               D(f"D{15 + i}", "SS14", f"COIL_{name}", "+12V", fp=SMA, sym="Device:D_Schottky")]
groups.append(("Signal relays (active-high, low-side MOSFETs)", relays, 4))

# --- Fail-safe pull-downs on every pod-driven line --------------------------------------------
PULLDOWN = ["BRAKE_RPWM", "BRAKE_LPWM", "BRAKE_EN", "MAGNET_EN", "RELAY_LEFT", "RELAY_RIGHT",
            "RELAY_HAZARD", "RELAY_HORN", "RELAY_BEAM"]
pulls = [R(f"R{50 + i}", "10k", sig, "GND", desc="fail-safe: off when not driven")
         for i, sig in enumerate(PULLDOWN)]
pulls += [R("R59", "10k", "+3V3", "CAN_TX", desc="recessive when not driven"),
          R("R60", "10k", "+3V3", "CAN_STBY", desc="standby (cannot transmit) when not driven")]
groups.append(("Fail-safe pull-downs (Part 4.8.4)", pulls, 6))

# --- CAN, presence, brake light, E-stop -------------------------------------------------------
misc = [
    {"sym": "Interface_CAN_LIN:SN65HVD230", "ref": "U4", "value": "SN65HVD230",
     "fp": "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
     "desc": "beside the OBD lead: the stub on the car's bus is the lead only. No termination.",
     "nets": {"1": "CAN_TX", "2": "GND", "3": "+3V3", "4": "CAN_RX", "6": "OBD_CANL",
              "7": "OBD_CANH", "8": "CAN_STBY"}},
    C("C32", "100n", "+3V3", "GND"),
    {"sym": "Power_Protection:NUP2105L", "ref": "D20", "value": "NUP2105L",
     "fp": "Package_TO_SOT_SMD:SOT-23", "desc": "CAN bus ESD",
     "nets": {"1": "OBD_CANH", "2": "OBD_CANL", "3": "GND"}},
    CONN("J11", "OBD-II lead", ["OBD_CANH", "OBD_CANL", "GND"], kind="screw",
         desc="J1962 pin 6 CAN_H, pin 14 CAN_L, pins 4/5 ground; ~0.5 m"),
    NMOS("Q8", "2N7002", "BOXP_G", "GND", "BOX_PRESENT"),
    R("R61", "10k", "+5V", "BOXP_G", desc="present = connected AND powered"),
    R("R62", "100k", "BOXP_G", "GND"),
    CONN("J10", "Brake-light switch", ["BL_IN", "GND"], kind="screw"),
    R("R63", "2.2k", "BL_IN", "BL_A", desc="~5 mA LED current at 12 V"),
    {"sym": "Isolator:PC817", "ref": "U5", "value": "PC817", "fp": "Package_DIP:DIP-4_W7.62mm",
     "nets": {"1": "BL_A", "2": "GND", "3": "GND", "4": "BRAKE_LIGHT"}},
    D("D21", "1N4148W", "GND", "BL_A", desc="reverse protection for the opto LED"),
    CONN("J9", "E-stop sense contact", ["KILL_SENSE", "GND"], kind="screw",
         desc="E-stop second contact block, NC: closed (released) pulls KILL_SENSE low"),
    CONN("J8", "Kill relay coil feed", ["+12V", "GND"], kind="screw",
         desc="+12V -> E-stop NC contact -> 40 A relay coil -> GND (off-board wiring)"),
    CONN("J12", "Bench: HX711", ["+5V", "HX711_DOUT", "HX711_SCK", "GND"],
         desc="load-cell amplifier on the bench rig only"),
]
groups.append(("CAN transceiver, presence, brake light, E-stop, bench", misc, 4))

# --- Cable ------------------------------------------------------------------------------------
cable = {"1": "5V_POD", "2": "5V_POD", "3": "GND", "4": "GND", "6": "GND", "11": "GND",
         "25": "GND", "SH": "GND", "5": "BRAKE_RPWM", "7": "BRAKE_LPWM", "8": "BRAKE_EN",
         "9": "MAGNET_EN", "10": "BRAKE_CURRENT", "12": "KILL_SENSE", "13": "BOX_PRESENT",
         "14": "RELAY_LEFT", "15": "RELAY_RIGHT", "16": "RELAY_HAZARD", "17": "RELAY_HORN",
         "18": "RELAY_BEAM", "19": "BRAKE_LIGHT", "20": "CAN_TX", "21": "CAN_RX", "22": "CAN_STBY",
         "23": "HX711_DOUT", "24": "HX711_SCK"}
groups.append(("Inter-box cable (DB-25), BUILD_GUIDE 4.8.3", [
    {"sym": "Connector:DB25_Socket_MountingHoles", "ref": "J1", "value": "DB-25 to pod", "fp": DSUB25,
     "desc": "shield (shell) bonded to ground at this end only", "nets": cable}], 1))
groups.append(("Mounting", HOLES("H", 4), 4))

PROJECT = "power_board"
TITLE = "Under-dash power board (power, actuator, relays, CAN)"
POWER_FLAGS = ["GND", "+12V", "+5V"]
