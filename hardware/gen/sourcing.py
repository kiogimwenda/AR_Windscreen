"""Which exact part each BOM line is (manufacturer and part number), for ordering the boards
assembled by the fab (JLCPCB; PCBWay takes the same files), and the fab-format files built from
the boards' own BOM and position files. Run by finish.sh:

    python3 sourcing.py <board> <fab dir>

writes <board>-jlc-bom.csv (Comment, Designator, Footprint, Manufacturer, MPN, LCSC Part #) and
<board>-jlc-cpl.csv (Designator, Mid X, Mid Y, Layer, Rotation), and fails if any part on the
board has no entry here: a fab-assembled board cannot have a line nobody can match.

An entry: (value or None for any, footprint name substring, manufacturer, part number, LCSC code
or "", note). The first entry that matches a BOM line is used. Part numbers: the buy list's
(docs/bill-of-materials.md section 10); passives from the makers the fab stocks cheaply. Where
the exact part number is not certain the note says what to match (the fab's BOM tool proposes
parts; check each one it picks against the note).
"""

import csv
import sys
from pathlib import Path

R = "UNI-ROYAL(Uniroyal Elec)"
SOURCING = [
    # --- resistors, 0805, 1 % (UNI-ROYAL 0805W8F<code>T5E) ------------------------------------
    *[(v, "R_0805", R, f"0805W8F{code}T5E", "", "0805 1 % 1/8 W") for v, code in [
        ("100", "1000"), ("330", "3300"), ("1k", "1001"), ("2.2k", "2201"), ("4.7k", "4701"),
        ("5.1k", "5101"), ("10k", "1002"), ("10.2k", "1022"), ("13.0k", "1302"), ("20k", "2002"),
        ("53.6k", "5362"), ("84.5k", "8452"), ("100k", "1003"), ("162k", "1623"), ("523k", "5233"),
        ("1M", "1004")]],
    # --- capacitors ------------------------------------------------------------------------------
    ("15p", "C_0805", "Samsung Electro-Mechanics", "CL21C150JBANNNC", "", "15 pF 50 V C0G"),
    ("39p", "C_0805", "Samsung Electro-Mechanics", "CL21C390JBANNNC", "", "39 pF 50 V C0G"),
    ("4.7n", "C_0805", "Samsung Electro-Mechanics", "CL21B472KBANNNC", "", "4.7 nF 50 V X7R"),
    ("6800p", "C_0805", "Samsung Electro-Mechanics", "CL21B682KBANNNC", "", "6.8 nF 50 V X7R"),
    ("10n", "C_0805", "Samsung Electro-Mechanics", "CL21B103KBANNNC", "", "10 nF 50 V X7R"),
    ("100n", "C_0805", "Samsung Electro-Mechanics", "CL21B104KBCNNNC", "", "100 nF 50 V X7R"),
    ("1u", "C_0805", "Samsung Electro-Mechanics", "CL21B105KBFNNNE", "", "1 uF 50 V X7R"),
    ("2.2u", "C_0805", "Samsung Electro-Mechanics", "CL21B225KOFNNNE", "", "2.2 uF 16 V X7R (MCU VCAP)"),
    ("4.7u", "C_1206", "Samsung Electro-Mechanics", "CL31A475KAHNNNE", "", "4.7 uF 25 V X5R"),
    ("10u", "C_1206", "Samsung Electro-Mechanics", "CL31A106KAHNNNE", "", "10 uF 25 V X5R"),
    ("2.2u 100V", "C_1210", "Murata", "GRM32ER72A225KA35L", "", "2.2 uF 100 V X7R: converter input"),
    ("47u", "C_1210", "Murata", "GRM32ER61C476KE15L", "", "47 uF 16 V X5R: converter output"),
    ("47u 63V", "CP_Radial", "Panasonic", "EEU-FR1J470", "", "47 uF 63 V aluminium, 8 mm, 3.5 mm pitch, 105 C"),
    # --- diodes, LEDs ------------------------------------------------------------------------------
    ("SS14", "D_SMA", "MDD(Microdiode Electronics)", "SS14", "", "1 A 40 V Schottky, SMA"),
    ("B560C", "D_SMC", "Diodes Incorporated", "B560C-13-F", "", "5 A 60 V Schottky, SMC"),
    ("SMCJ33CA", "D_SMC", "Littelfuse", "SMCJ33CA", "", "TVS 33 V bidirectional, SMC (NOT the 24 V part)"),
    ("NUP2105L", "SOT-23", "onsemi", "NUP2105LT1G", "", "CAN bus ESD"),
    ("1N4148W", "SOD-123", "Diodes Incorporated", "1N4148W-7-F", "", ""),
    ("green", "LED_0805", "Kingbright", "APT2012SGC", "", "green 0805"),
    ("blue", "LED_0805", "Kingbright", "APT2012QBC/D", "",
     "blue 0805: at 3.3 V through 1 k it barely lights; a <= 2.8 V blue or any other colour is better"),
    ("amber", "LED_0805", "Kingbright", "APT2012SYCK", "", "yellow/amber 0805"),
    ("red", "LED_0805", "Kingbright", "APT2012SURCK", "", "red 0805"),
    # --- transistors, ICs ------------------------------------------------------------------------
    ("AO3400A", "SOT-23", "Alpha & Omega Semicon", "AO3400A", "C20917", "30 V N-MOSFET"),
    ("2N7002", "SOT-23", "onsemi", "2N7002LT1G", "", ""),
    ("STM32F405RGT6", "LQFP-64", "STMicroelectronics", "STM32F405RGT6", "", ""),
    ("AP2112K-3.3", "SOT-23-5", "Diodes Incorporated", "AP2112K-3.3TRG1", "", "3.3 V 600 mA LDO"),
    ("USBLC6-2SC6", "SOT-23-6", "STMicroelectronics", "USBLC6-2SC6", "", "USB ESD"),
    ("TPS54360DDA", "SO-PowerPAD-8", "Texas Instruments", "TPS54360DDAR", "", "60 V 3.5 A buck: thermal pad soldered"),
    ("74HCT244", "SOIC-20W", "Texas Instruments", "SN74HCT244DWR", "", "HCT (not HC), SOIC-20 wide"),
    ("SN65HVD230", "SOIC-8", "Texas Instruments", "SN65HVD230DR", "", "CAN transceiver"),
    ("PC817", "DIP-4", "Sharp", "PC817X", "", "any PC817 in DIP-4 (or Everlight EL817): through-hole"),
    ("8MHz", "HC49-SD", "", "8.000 MHz HC-49/US SMD", "",
     "crystal, HC-49/US SMD, load capacitance 12 pF (to suit C12/C13 = 15 pF), <= 50 ppm"),
    ("8.2u", "SRR1260", "Bourns", "SRR1260-8R2Y", "", "8.2 uH, Isat > 4.5 A"),
    ("500mA", "Fuse_1206", "Bourns", "MF-NSMF050-2", "", "500 mA polyfuse, 1206"),
    # --- connectors, switches, fuse holders ---------------------------------------------------------
    (None, "USB_C_Receptacle_GCT_USB4105", "GCT", "USB4105-GF-A", "", "USB-C 16-pin"),
    (None, "SW_SPST_EVQP7C", "Panasonic", "EVQ-P7C01P", "", "side-push tactile switch"),
    (None, "JST_GH_SM05B", "JST", "SM05B-GHS-TB(LF)(SN)", "", "JST GH 5-pin, right angle"),
    (None, "DSUB-25_Socket", "", "DB-25 female right-angle PCB", "",
     "DB-25 FEMALE, right angle, through-hole, 2.77 x 2.84 mm pitch, 4.94 mm front-face-to-pin offset, "
     "board locks (e.g. TE 2311766-1, Norcomp 178-025): check the 4.94 mm on its drawing"),
    (None, "MKDS-1,5-2-5.08", "Phoenix Contact", "1715721", "", "MKDS 1,5/2-5,08 terminal block"),
    (None, "MKDS-1,5-3-5.08", "Phoenix Contact", "1715734", "", "MKDS 1,5/3-5,08 terminal block"),
    (None, "MKDS-1,5-6-5.08", "Phoenix Contact", "1715763", "", "MKDS 1,5/6-5,08 terminal block"),
    (None, "Fuseholder_Blade_Mini_Keystone_3568", "Keystone", "3568", "", "mini blade fuse holder (fuses separate)"),
    ("IMU (BNO085 breakout)", "PinHeader_1x06", "", "1x6 female socket 2.54 mm", "",
     "FEMALE socket (the IMU breakout plugs in), e.g. Sullins PPTC061LFBN-RC"),
    *[(None, f"PinHeader_1x{n:02d}", "", f"1x{n} male pin header 2.54 mm", "", "male pin header, straight")
      for n in (1, 3, 4, 5, 8)],
    (None, "MountingHole", None, None, None, "not a part"),
]


def lookup(value, footprint):
    for v, fp, maker, mpn, lcsc, note in SOURCING:
        if (v is None or v == value) and fp in footprint:
            return maker, mpn, lcsc, note
    return None


def expand(refs):
    """KiCad's "C3-C6,C9" -> "C3,C4,C5,C6,C9": the fab wants every designator."""
    import re
    out = []
    for part in refs.split(","):
        m = re.fullmatch(r"([A-Z]+)(\d+)-([A-Z]+)(\d+)", part.strip())
        if m and m.group(1) == m.group(3):
            out += [f"{m.group(1)}{n}" for n in range(int(m.group(2)), int(m.group(4)) + 1)]
        else:
            out.append(part.strip())
    return ",".join(out)


def main():
    board, fab = sys.argv[1], Path(sys.argv[2])
    rows, missing = [], []
    for r in csv.DictReader(open(fab / f"{board}-bom.csv", encoding="utf-8")):
        found = lookup(r["Value"], r["Footprint"])
        if found is None:
            missing.append(f'{r["Refs"]} {r["Value"]} {r["Footprint"]}')
            continue
        maker, mpn, lcsc, note = found
        if mpn is None:  # mounting holes
            continue
        rows.append([f"{r['Value']}" + (f" ({note})" if note else ""), expand(r["Refs"]),
                     r["Footprint"].split(":")[-1], maker, mpn, lcsc])
    if missing:
        sys.exit(f"{board}: no sourcing entry (hardware/gen/sourcing.py) for: " + "; ".join(missing))
    with open(fab / f"{board}-jlc-bom.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "Manufacturer", "MPN", "LCSC Part #"])
        w.writerows(rows)
    # placement: KiCad's position file (from the board's bottom-left corner) in the fab's columns
    with open(fab / f"{board}-jlc-cpl.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in csv.DictReader(open(fab / f"{board}-pos.csv", encoding="utf-8")):
            if r["Ref"].startswith("H"):
                continue
            w.writerow([r["Ref"], f'{float(r["PosX"]):.3f}mm', f'{float(r["PosY"]):.3f}mm',
                        "Top" if r["Side"] == "top" else "Bottom", f'{float(r["Rot"]):g}'])
    print(f"{board}: fab-assembly BOM ({len(rows)} lines) and placement file written")


if __name__ == "__main__":
    main()
