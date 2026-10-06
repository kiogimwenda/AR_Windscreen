"""Part factories shared by the two boards: a part is a dict that kisch.Schematic.place() takes.

Footprints are KiCad 10 standard-library names, chosen to be hand-solderable: 0805 passives,
SOT-23, SOIC, LQFP-64, through-hole connectors and terminal blocks. Change a footprint here and
both boards follow.
"""

R0805 = "Resistor_SMD:R_0805_2012Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
C1210 = "Capacitor_SMD:C_1210_3225Metric"
LED0805 = "LED_SMD:LED_0805_2012Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SMA = "Diode_SMD:D_SMA"
SMC = "Diode_SMD:D_SMC"
SOD123 = "Diode_SMD:D_SOD-123"
HOLE = "MountingHole:MountingHole_3.2mm_M3_Pad_Via"
DSUB25 = ("Connector_Dsub:DSUB-25_Socket_Horizontal_P2.77x2.84mm_EdgePinOffset4.94mm_Housed_"
          "MountingHolesOffset4.94mm")


def header(n):
    return f"Connector_PinHeader_2.54mm:PinHeader_1x{n:02d}_P2.54mm_Vertical"


def terminal(n):
    return f"TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-{n}-5.08_1x{n:02d}_P5.08mm_Horizontal"


def R(ref, value, a, b, fp=R0805, desc=""):
    return {"sym": "Device:R", "ref": ref, "value": value, "fp": fp, "desc": desc,
            "nets": {"1": a, "2": b}}


def C(ref, value, a, b, fp=C0805, desc=""):
    return {"sym": "Device:C", "ref": ref, "value": value, "fp": fp, "desc": desc,
            "nets": {"1": a, "2": b}}


def D(ref, value, anode, cathode, fp=SOD123, sym="Device:D", desc=""):
    return {"sym": sym, "ref": ref, "value": value, "fp": fp, "desc": desc,
            "nets": {"1": cathode, "2": anode}}


def LED(ref, value, anode, cathode):
    return {"sym": "Device:LED", "ref": ref, "value": value, "fp": LED0805,
            "nets": {"1": cathode, "2": anode}}


def NMOS(ref, value, g, s, d, fp=SOT23):
    return {"sym": f"Transistor_FET:{value}", "ref": ref, "value": value, "fp": fp,
            "nets": {"1": g, "2": s, "3": d}}


def CONN(ref, value, nets, fp=None, desc="", kind="generic"):
    n = len(nets)
    sym = f"Connector_Generic:Conn_01x{n:02d}" if kind == "generic" else f"Connector:Screw_Terminal_01x{n:02d}"
    fp = fp or (header(n) if kind == "generic" else terminal(n))
    return {"sym": sym, "ref": ref, "value": value, "fp": fp, "desc": desc,
            "nets": {str(i + 1): net for i, net in enumerate(nets) if net is not None}}


def HOLES(prefix, n, net=None):
    return [{"sym": "Mechanical:MountingHole", "ref": f"{prefix}{i + 1}", "value": "M3",
             "fp": "MountingHole:MountingHole_3.2mm_M3", "nets": {}} for i in range(n)]
