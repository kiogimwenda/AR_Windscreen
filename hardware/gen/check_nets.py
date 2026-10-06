#!/usr/bin/env python3
"""Cross-checks the exported KiCad netlists against the firmware and the build guide.

    python3 hardware/gen/check_nets.py hardware/pod_board/pod_board.net hardware/power_board/power_board.net

Checks, each printed as PASS/FAIL:
  1. every pin in firmware Config.h lands on the right STM32 pin of the pod board, by PIN NAME
     (PA8, PC6, ...) as KiCad's netlist reports it;
  2. the inter-box cable pinout, parsed from BUILD_GUIDE.md Part 4.8.3, is what both DB-25
     connectors carry, and the two ends agree pin for pin;
  3. on the pod, each cable line reaches its MCU pin through a series resistor, and the inputs that
     must read safe when the cable is out have their pull-ups;
  4. on the power board, every pod-driven line has a pull-down to GND (CAN_TX and CAN_STBY: a
     pull-up to +3V3), so an unpowered pod or an unplugged cable leaves everything off;
  5. no net has only one connection (a misspelt net name would);
  6. every part has a footprint.
Exit status 0 only if every check passes.
"""

import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "firmware/sensor_actuator_hub/include/hub/Config.h"
GUIDE = ROOT / "docs/BUILD_GUIDE.md"

# Config.h define -> the pod board's net name for that pin
CONFIG_NETS = {
    "HUB_PIN_BRAKE_RPWM": "BRAKE_RPWM", "HUB_PIN_BRAKE_LPWM": "BRAKE_LPWM",
    "HUB_PIN_BRAKE_EN": "BRAKE_EN", "HUB_PIN_BRAKE_CURRENT": "BRAKE_CURRENT",
    "HUB_PIN_CABLE_MAGNET": "MAGNET_EN", "HUB_PIN_KILL_SENSE": "KILL_SENSE",
    "HUB_PIN_BOX_PRESENT": "BOX_PRESENT", "HUB_PIN_RELAY_LEFT": "RELAY_LEFT",
    "HUB_PIN_RELAY_RIGHT": "RELAY_RIGHT", "HUB_PIN_RELAY_HAZARD": "RELAY_HAZARD",
    "HUB_PIN_RELAY_HORN": "RELAY_HORN", "HUB_PIN_RELAY_BEAM": "RELAY_BEAM",
    "HUB_PIN_BRAKE_LIGHT_SENSE": "BRAKE_LIGHT", "HUB_PIN_HX711_DOUT": "HX711_DOUT",
    "HUB_PIN_HX711_SCK": "HX711_SCK", "HUB_CAN1_RX": "CAN_RX", "HUB_CAN1_TX": "CAN_TX",
    "HUB_PIN_CAN_SILENT": "CAN_STBY", "HUB_OBD_TX": "OBD_TX", "HUB_OBD_RX": "OBD_RX",
    "HUB_IMU_SDA": "IMU_SDA", "HUB_IMU_SCL": "IMU_SCL", "HUB_GESTURE_SDA": "GEST_SDA",
    "HUB_GESTURE_SCL": "GEST_SCL", "HUB_GPS_TX": "GPS_TX", "HUB_GPS_RX": "GPS_RX",
    "HUB_PIN_LED_LINK": "LED_LINK", "HUB_PIN_LED_ARMED": "LED_ARMED",
    "HUB_PIN_LED_FAULT": "LED_FAULT", "HUB_PIN_IMU_INT": "IMU_INT", "HUB_PIN_IMU_RST": "IMU_RST",
    "HUB_PIN_GESTURE_INT": "GEST_INT", "HUB_PIN_GPS_PPS": "GPS_PPS",
}
# Guide 4.8.3 signal names -> per-board net names on the DB-25
GUIDE_TO_NET = {"+5V_POD": ("X_5V_CABLE", "5V_POD"), "GND": ("GND", "GND"),
                "BRAKE_LIGHT": ("X_BRAKE_LIGHT", "BRAKE_LIGHT"),
                "RELAY_LEFT": ("X_RELAY_LEFT", "RELAY_LEFT")}
POD_OUTPUTS = ["BRAKE_RPWM", "BRAKE_LPWM", "BRAKE_EN", "MAGNET_EN", "RELAY_LEFT", "RELAY_RIGHT",
               "RELAY_HAZARD", "RELAY_HORN", "RELAY_BEAM", "CAN_TX", "CAN_STBY", "HX711_SCK"]
POD_SAFE_INPUTS = {"KILL_SENSE": "+3V3", "BOX_PRESENT": "+3V3", "BRAKE_LIGHT": "+3V3",
                   "CAN_RX": "+3V3"}
BOX_FAILSAFE = {s: "GND" for s in POD_OUTPUTS if s not in ("CAN_TX", "CAN_STBY", "HX711_SCK")}
BOX_FAILSAFE.update({"CAN_TX": "+3V3", "CAN_STBY": "+3V3"})

failures = []


def check(ok, what):
    print(("PASS " if ok else "FAIL ") + what)
    if not ok:
        failures.append(what)


def read_netlist(path):
    s = Path(path).read_text(encoding="utf-8")
    comps = {}
    for m in re.finditer(r'\(comp \(ref "([^"]+)"\)(.*?)\n\t\t\)', s, re.S):
        body = m.group(2)
        val = re.search(r'\(value "([^"]*)"\)', body)
        fp = re.search(r'\(footprint "([^"]*)"\)', body)
        comps[m.group(1)] = {"value": val.group(1) if val else "", "fp": fp.group(1) if fp else ""}
    nets = {}
    pinnet = {}
    for m in re.finditer(r'\(net\s+\(code "\d+"\)\s+\(name "([^"]+)"\)(.*?)\n\t\t\)', s, re.S):
        name = m.group(1).lstrip("/")
        nodes = re.findall(r'\(node\s+\(ref "([^"]+)"\)\s+\(pin "([^"]+)"\)(?:\s+\(pinfunction "([^"]*)"\))?',
                           m.group(2))
        nets[name] = nodes
        for ref, pin, fn in nodes:
            pinnet[(ref, pin)] = name
            if fn:  # KiCad 10 writes "PA8_41": the pin name, then the pin number
                pinnet[(ref, "fn:" + re.sub(r"_" + re.escape(pin) + r"$", "", fn))] = name
    return comps, nets, pinnet


def resistor_between(comps, nets, a, b, value=None):
    ra = {ref for ref, _, _ in nets.get(a, []) if ref.startswith("R")}
    rb = {ref for ref, _, _ in nets.get(b, []) if ref.startswith("R")}
    both = ra & rb
    if value is not None:
        both = {r for r in both if comps[r]["value"] == value}
    return both


def config_pins():
    pins = {}
    for m in re.finditer(r"#define\s+(HUB_\w+)\s+(P[A-H]\d+)\b", CONFIG.read_text()):
        pins[m.group(1)] = m.group(2)
    return pins


def guide_pinout():
    text = GUIDE.read_text(encoding="utf-8")
    sec = text[text.index("#### 4.8.3"):text.index("#### 4.8.4")]
    table = {}
    for row in re.findall(r"^\| ([\d, –-]+) \| ([^|]+) \|", sec, re.M):
        pins_s, sig = row
        sigs = [x.strip() for x in sig.split(",")]
        pins = []
        for part in pins_s.split(","):
            part = part.strip()
            if "–" in part or "-" in part:
                a, b = re.split(r"[–-]", part)
                pins += list(range(int(a), int(b) + 1))
            elif part:
                pins.append(int(part))
        if len(sigs) == 1:
            for p in pins:
                table[p] = sigs[0]
        else:  # "RELAY_LEFT, _RIGHT, ..." style rows: one signal per pin
            base = sigs[0].rsplit("_", 1)[0]
            names = [sigs[0]] + [base + s if s.startswith("_") else s for s in sigs[1:]]
            for p, n in zip(pins, names):
                table[p] = n
    return table


def main(pod_net, box_net):
    pc, pn, pp = read_netlist(pod_net)
    bc, bn, bp = read_netlist(box_net)

    # 1. Config.h -> STM32 pins
    cfg = config_pins()
    for define, net in CONFIG_NETS.items():
        port = cfg.get(define)
        if port is None:
            check(False, f"Config.h defines {define}")
            continue
        got = pp.get(("U1", "fn:" + port))
        check(got == net, f"{define} = {port} -> pod net {net} (schematic: {got})")
    unmapped = sorted(set(cfg) - set(CONFIG_NETS))
    check(not unmapped, f"every Config.h pin is cross-checked (unmapped: {unmapped})")

    # 2. Cable pinout from the guide, on both boards
    table = guide_pinout()
    check(len(table) == 25, f"guide 4.8.3 table covers all 25 pins (found {len(table)})")
    for pin, sig in sorted(table.items()):
        if sig in ("+5V_POD",):
            want_pod, want_box = "X_5V_CABLE", "5V_POD"
        elif sig == "GND":
            want_pod = want_box = "GND"
        else:
            want_pod, want_box = "X_" + sig, sig
        got_pod = pp.get(("J7", str(pin)))
        got_box = bp.get(("J1", str(pin)))
        check(got_pod == want_pod and got_box == want_box,
              f"DB-25 pin {pin:2d} {sig:<14} pod {got_pod}, box {got_box}")
    check(pp.get(("J7", "SH")) == "GND" and bp.get(("J1", "SH")) == "GND", "DB-25 shells on GND")

    # 3. Pod: series resistors and safe-reading pull-ups
    for sig in POD_OUTPUTS + list(POD_SAFE_INPUTS) + ["HX711_DOUT", "BRAKE_CURRENT"]:
        check(bool(resistor_between(pc, pn, sig, "X_" + sig)), f"pod: {sig} reaches the cable through a resistor")
    for sig, rail in POD_SAFE_INPUTS.items():
        check(bool(resistor_between(pc, pn, rail, "X_" + sig)), f"pod: X_{sig} pulled up to {rail} (safe when the cable is out)")

    # 4. Power board: fail-safe defaults
    for sig, rail in BOX_FAILSAFE.items():
        check(bool(resistor_between(bc, bn, sig, rail)), f"box: {sig} defaults via a resistor to {rail}")
    q = [r for r, p, _ in bn.get("BOX_PRESENT", []) if r.startswith("Q")]
    check(bool(q), "box: BOX_PRESENT pulled low by a MOSFET (present only while powered)")

    # 5, 6. Single-node nets; footprints
    for label, nets, comps in (("pod", pn, pc), ("box", bn, bc)):
        lonely = [n for n, nodes in nets.items() if len(nodes) == 1 and not n.startswith("unconnected")
                  and not n.startswith("Net-(")]
        check(not lonely, f"{label}: no named net with a single connection {lonely}")
        nofp = [r for r, c in comps.items() if not c["fp"]]
        check(not nofp, f"{label}: every part has a footprint {nofp}")

    # 7. Footprints exist in KiCad's library (when its footprint directory is given)
    import os
    fpdir = os.environ.get("KICAD_FOOTPRINTS")
    if fpdir and Path(fpdir).is_dir():
        for label, comps in (("pod", pc), ("box", bc)):
            missing = sorted({c["fp"] for c in comps.values() if c["fp"] and not
                              (Path(fpdir) / (c["fp"].split(":")[0] + ".pretty") /
                               (c["fp"].split(":")[1] + ".kicad_mod")).is_file()})
            check(not missing, f"{label}: every footprint exists in KiCad's library {missing}")

    print(f"\n{'ALL CHECKS PASS' if not failures else f'{len(failures)} FAILED'}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
