#!/usr/bin/env python3
"""Generates the assembly-day documents (docs/assembly/) from the same board descriptions the
schematics are generated from (power_board.py, pod_board.py), so a pin, a part or a value can never
differ between the schematic and the procedure that tests it.

    python3 hardware/gen/assembly.py            # writes docs/assembly/*.md
    python3 hardware/gen/assembly.py --check    # fails if docs/assembly is out of date

Every expected reading is computed from the parts' values here (rail voltages from the feedback
divider, resistances seen at each DB-25 pin, the current sense's idle level), or quoted from a
datasheet that is named. The off-board wiring (OFFBOARD, below) is the one thing written by hand:
the script refuses to run if a connector on either board has no entry, or an entry names a
connector that no longer exists.
"""

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pod_board as pod  # noqa: E402
import power_board as pwr  # noqa: E402

OUT = HERE.parent.parent / "docs" / "assembly"


def parts_of(board):
    return {p["ref"]: p for _, ps, _ in board.groups for p in ps}


PWR, POD = parts_of(pwr), parts_of(pod)


def ohms(v):
    v = v.strip().lower().replace("ohm", "")
    mult = 1.0
    if v.endswith("k"):
        mult, v = 1e3, v[:-1]
    elif v.endswith("m"):
        mult, v = 1e6, v[:-1]
    return float(v) * mult


def fmt_ohms(x):
    return f"{x / 1e6:.3g} MΩ" if x >= 1e6 else f"{x / 1e3:.3g} kΩ" if x >= 1e3 else f"{x:.3g} Ω"


def resistors_between(parts, a, b):
    return [p for p in parts.values() if p["sym"] == "Device:R" and set(p["nets"].values()) == {a, b}]


# --- Computed expectations --------------------------------------------------------------------
TPS54360_VREF = 0.8  # V, TPS54360 datasheet SLVSBB4G, 7.3.4 ("0.8 V internal voltage reference")
ACS712_ZERO_FRACTION = 0.5  # ACS712 datasheet: zero-current output = VCC x 0.5
r_top, r_bot = ohms(PWR["R44"]["value"]), ohms(PWR["R45"]["value"])
V5 = TPS54360_VREF * (1 + r_top / r_bot)
r47, r48 = ohms(PWR["R47"]["value"]), ohms(PWR["R48"]["value"])
DIV = r48 / (r47 + r48)
ACS_IDLE_BOX = 5.0 * ACS712_ZERO_FRACTION * DIV  # at DB-25 pin 10, power board alone
r_thev = r47 * r48 / (r47 + r48)
r_pod_load = ohms(next(p for p in POD.values() if p["sym"] == "Device:R" and
                       set(p["nets"].values()) == {"X_BRAKE_CURRENT", "GND"})["value"])
ACS_IDLE_PAIR = ACS_IDLE_BOX * r_pod_load / (r_pod_load + r_thev)  # with the pod's 100 k load

# --- The cable, from both boards --------------------------------------------------------------
pwr_pins = PWR["J1"]["nets"]
pod_pins = POD["J7"]["nets"]


def signal_name(net):
    return net[2:] if net.startswith("X_") else net


def cable_rows():
    rows = []
    for pin in [str(i) for i in range(1, 26)] + ["SH"]:
        a, b = pwr_pins.get(pin), pod_pins.get(pin)
        sa, sb = a, signal_name(b) if b else b
        if sb == "5V_CABLE":
            sb = "5V_POD"
        if sa != sb:
            raise SystemExit(f"DB-25 pin {pin}: power board {a!r} vs pod board {b!r}")
        rows.append((pin, a, b))
    return rows


def pod_side(net):
    """What the pod board puts on a cable net: its series part, pull-up, MCU pin."""
    sig = signal_name(net)
    notes = []
    if sig in pod.OUTPUTS:
        _, mcu, r = pod.OUTPUTS[sig]
        notes.append(f"driven by MCU pin {mcu} through {r}")
    elif sig in pod.INPUTS:
        _, mcu, pull, cap = pod.INPUTS[sig]
        notes.append(f"read on MCU pin {mcu}" + ("; 10 k pull-up to 3.3 V" if pull else "")
                     + ("; 1 k + 10 nF filter" if cap else ""))
    elif sig == "BRAKE_CURRENT":
        notes.append("read on MCU pin 14 (PA0) through 1 k + 100 nF; 100 k to GND")
    elif sig == "5V_CABLE":
        notes.append("into VIN_POD through D2 (diode-OR with USB's VBUS through D1)")
    elif sig == "GND":
        notes.append("ground")
    return "; ".join(notes)


def box_side(net):
    pd = resistors_between(PWR, net, "GND")
    pu = resistors_between(PWR, net, "+3V3")
    if net in pwr.PULLDOWN:
        return f"10 k pull-down ({pd[0]['ref']}): off when not driven"
    if pu:
        return f"10 k pull-up to 3.3 V ({pu[0]['ref']}): {pu[0]['desc']}"
    return {
        "5V_POD": "+5 V through polyfuse F2 (500 mA)",
        "GND": "ground",
        "BRAKE_CURRENT": f"ACS712 output x{DIV:.3f} (R47/R48)",
        "KILL_SENSE": "E-stop second contact (J9), NC to ground",
        "BOX_PRESENT": "Q8 (2N7002) to ground while the box's 5 V is up",
        "BRAKE_LIGHT": "PC817 (U5) output: low = pedal pressed",
        "CAN_RX": "SN65HVD230 (U4) R output",
        "HX711_DOUT": "bench header J12 only",
        "HX711_SCK": "bench header J12 only",
    }.get(net, "")


# --- Off-board wiring: written by hand, checked against both boards ---------------------------
OFFBOARD = {
    "power_board": {
        "J1": "the inter-box DB-25 cable to the pod (shield bonded here only)",
        "J2": "switched ACC (ignition) feed from the car's fuse box (its own fuse there too); "
              "ground to the chassis ground stud",
        "J3": "the LiDAR's 12 V via its M12 power cable (fused here by F3, 3 A)",
        "J4": "pin 1 to the five relays' coil commons (+12 V); pins 2-6 to the coil returns of "
              "LEFT, RIGHT, HAZARD, HORN, BEAM",
        "J5": "the 40 A kill relay's switched contact (ACT_12V) and ground: battery -> 10 A fuse "
              "-> kill relay contacts -> here",
        "J6": "the BTS7960 module's B+ and B- (its motor supply)",
        "J7": "the cable magnet (12 V holding magnet), + to ACT_12V, - to MAG_NEG",
        "J8": "+12 V -> E-stop contact block 1 (NC) -> 40 A kill relay coil -> ground",
        "J9": "E-stop contact block 2 (NC): KILL_SENSE and ground",
        "J10": "the brake-light switch's lamp side (12 V when the pedal is pressed) and ground",
        "J11": "the OBD-II lead: J1962 pin 6 CAN_H, pin 14 CAN_L, pins 4/5 ground (~0.5 m)",
        "J12": "bench only: the HX711 load-cell amplifier (VCC, DOUT, SCK, GND)",
        "J13": "the BTS7960 module's logic header (RPWM LPWM R_EN L_EN R_IS L_IS VCC GND)",
        "J14": "the ACS712-20A module (VCC, OUT, GND); its current path in series with the actuator",
    },
    "pod_board": {
        "J1": "USB-C to the laptop",
        "J2": "the IMU breakout (BNO085), on standoffs",
        "J3": "the GNSS breakout (NEO-M8N); antenna on the pod's roof face",
        "J4": "the gesture puck (APDS-9960) on its ~1.5 m JST-GH cable",
        "J5": "ST-Link (SWD) for flashing and debugging",
        "J6": "ELM327 fallback bridge (HC-05), hub_elm327 build only",
        "J7": "the inter-box DB-25 cable to the power box",
    },
}


def check_offboard():
    for name, parts in (("power_board", PWR), ("pod_board", POD)):
        conns = {r for r, p in parts.items() if p["sym"].startswith(("Connector", "Connector_Generic"))}
        conns |= {r for r, p in parts.items() if "Screw_Terminal" in p["sym"]}
        known = set(OFFBOARD[name])
        if conns - known:
            raise SystemExit(f"{name}: no off-board wiring entry for {sorted(conns - known)}")
        if known - conns:
            raise SystemExit(f"{name}: wiring entries for connectors that no longer exist: "
                             f"{sorted(known - conns)}")


# --- Documents ----------------------------------------------------------------------------------
HEADER = ("<!-- Generated by hardware/gen/assembly.py from hardware/gen/{power,pod}_board.py. "
          "Do not edit: change the board description or the generator, then run it. -->\n\n")


def doc_cable():
    rows = cable_rows()
    out = [HEADER, "# The inter-box cable (DB-25): build sheet\n\n",
           "BUILD_GUIDE Part 4.8.3. **Male DB-25 at both ends** (both boards carry sockets), "
           "**shielded, all 25 conductors**, about 2 m. Solder-cup or IDC. The cable's **braid** is "
           "bonded to ground **at the power box end only**: soldered to the metal hood there (the "
           "hood meets power board J1's grounded shell); at the pod end, cut the braid back and "
           "insulate it, so it touches nothing (that hood still meets the pod's shell, which is "
           "fine: the braid is what must not form a loop).\n\n",
           "Wire it pin to pin: pin N at one end to pin N at the other (a straight cable; no "
           "crossovers). Write each conductor's colour in the table as you go: it is the record a "
           "later fault-finder needs.\n\n",
           "| Pin | Signal | Power board (box) | Pod board | Colour | Continuity | Not shorted to "
           "neighbours |\n|---|---|---|---|---|---|---|\n"]
    for pin, net, podnet in rows:
        if pin == "SH":
            out.append("| hood | braid | soldered to the hood: GND via J1's shell | **cut back, "
                       "insulated** | — | ☐ | — |\n")
            continue
        out.append(f"| {pin} | {net} | {box_side(net)} | {pod_side(podnet)} | | ☐ | ☐ |\n")
    out.append("""
## Testing the finished cable (before it touches either board)

1. **Continuity, every pin:** meter on continuity between pin N at one end and pin N at the other,
   for all 25. Every one must beep. (Cheap "printer" cables often wire only some pins; Part 4.7.)
2. **No shorts:** with one end free, each pin against its neighbours in both rows and against the
   shell: open. Pins 3, 4, 6, 11 and 25 are all GND at the boards, but **in the bare cable they are
   separate conductors**: they must not beep to each other until a board joins them.
3. **Braid:** the hood at the power-box end beeps to the braid; the hood at the pod end does not
   beep to the braid (nor to any pin).
4. Tick the table, sign and date it, and photograph both ends before closing the hoods.

## Once both boards are up (power box powered, pod on USB)

| Pin | Check | Expect |
|---|---|---|
| 1, 2 | DC volts to GND at the pod's J7 | the box's 5 V less F2's drop: about 5.0 V |
| 13 | the pod's `SensorReport.ignitionOn` | 1 (BOX_PRESENT pulled low by Q8) |
| 12 | E-stop released / pressed | `killSwitchEngaged` 0 / 1 |
| 19 | brake pedal pressed (or 12 V on J10) | `obdBrakePedalActive` follows it |
""" + f"""| 10 | volts to GND at the pod, actuator idle | about {ACS_IDLE_PAIR:.2f} V (the ACS712's 2.5 V zero x{DIV:.3f}, loaded by the pod's 100 k): the firmware's zero-current reading |
""")
    return "".join(out)


def doc_power_board():
    pulls = "".join(
        f"| {pin} | {net} | 10 kΩ to GND ({resistors_between(PWR, net, 'GND')[0]['ref']}) |\n"
        for pin, net, _ in cable_rows() if net in pwr.PULLDOWN)
    pulls += "".join(
        f"| {pin} | {net} | 10 kΩ to +3V3 ({resistors_between(PWR, net, '+3V3')[0]['ref']}) |\n"
        for pin, net, _ in cable_rows() if resistors_between(PWR, net, "+3V3"))
    # The descriptions read "UVLO: start 8 V" and "UVLO: stop 6.25 V".
    uvlo_start = " ".join(PWR["R41"]["desc"].split(":")[1].split()[1:])
    uvlo_stop = " ".join(PWR["R42"]["desc"].split(":")[1].split()[1:])
    return HEADER + f"""# Power board: bring-up

BUILD_GUIDE Parts 4.7 and 4.8.4. **Nothing connected but the bench supply** until step 6. A bench
supply with a current limit, a multimeter, and (once, step 5) a 3.3 V source through 1 kΩ.

## 1. Before any power: look, then measure

- Under magnification: every IC's pin 1 where the silkscreen says (U1 TPS54360, U2 AP2112K,
  U3 74HCT244, U4 SN65HVD230, U5 PC817), the diodes' bands (D10 is bidirectional; D11, D13-D19, D21
  band to the cathode), no bridges on U1's PowerPAD or U3's pins.
- Resistance to GND, supply off (a short here is found now, not with smoke):

| Net | Measure at | Expect |
|---|---|---|
| +12V | D11 cathode / C20 + | well above 1 kΩ (the UVLO divider R41+R42 is {fmt_ohms(ohms(PWR['R41']['value']) + ohms(PWR['R42']['value']))}; C20 charges from the meter at first) |
| +5V | C24 + | well above 1 kΩ (feedback divider R44+R45: {fmt_ohms(r_top + r_bot)}) |
| +3V3 | C29 + | well above 1 kΩ |
| 5V_POD | DB-25 pin 1 | the same as +5V (F2 is a polyfuse, a few ohms when cold) |

- The fail-safe resistors, at the DB-25 (J1) pins, supply off:

| Pin | Net | Expect |
|---|---|---|
{pulls}
## 2. First power: current-limited

1. Bench supply to **J2** (ACC_IN, GND): 12.0 V, current limit **100 mA**.
2. Switch on. The supply must not sit in current limit. Idle current with nothing connected: tens
   of milliamps at most (D12's LED ~2 mA, the buck's own quiescent current and the 5 V dividers).
   If it limits: off, and find the short (step 1 again, then U1's area).
3. **D12** (green) lights: 12 V is through F1 and D11 (D10, the TVS, only clamps to ground).

## 3. Rails

| Rail | Measure at | Expect |
|---|---|---|
| +12V | C20 + | the supply less D11's drop: about 0.3-0.5 V below it at this current |
| +5V | C24 + | **{V5:.2f} V** (0.8 V reference x (1 + {PWR['R44']['value']}/{PWR['R45']['value']})), within 2 % |
| +3V3 | C29 + | 3.30 V (AP2112K-3.3) |
| 5V_POD | DB-25 pin 1 | as +5V |
| LIDAR_12V | J3 pin 1 | as +12V (through F3) |

**Under-voltage lockout** (R41/R42): lower the supply slowly: +5V collapses near **{uvlo_stop}**; raise
it: +5V returns near **{uvlo_start}**. A car's cranking dip must not brown the board out half-way.

## 4. The fail-safe state, with the pod NOT connected

| Check | Measure | Expect |
|---|---|---|
| BTS7960 inputs off | J13 pins 1-3 (RPWM_5V, LPWM_5V, EN_5V) to GND | 0 V |
| Relays off | Q3-Q7 drains (J4 pins 2-6) | 12 V (coils not pulled down) |
| Magnet off | Q2 drain (J7 pin 2, MAG_NEG) | no voltage across J7 |
| CAN silent | U4 pin 8 (Rs) and pin 1 (D) | 3.3 V: standby, recessive |
| Present | DB-25 pin 13, with the 3.3 V source through 10 kΩ on it; supply on / off | near 0 V (Q8 on) / 3.3 V |
| Current sense idle | DB-25 pin 10 to GND, ACS712 module fitted | **{ACS_IDLE_BOX:.2f} V** (2.5 V zero-current output x{DIV:.3f}) |

## 5. Each pod-driven line, by hand

With the 3.3 V source through 1 kΩ, touch each DB-25 pin in turn (the pull-down holds it low
otherwise):

| Pin | Line | Expect while high |
|---|---|---|
| 14-18 | RELAY_LEFT .. RELAY_BEAM | that relay clicks (fit the relays in their sockets first); the others stay off |
| 9 | MAGNET_EN | 12 V across J7 (an LED + resistor stands in for the magnet) |
| 8 | BRAKE_EN | J13 pin 3 rises to ~5 V |
| 5 | BRAKE_RPWM | J13 pin 1 rises to ~5 V |
| 7 | BRAKE_LPWM | J13 pin 2 rises to ~5 V (**never** used in the car: the drive is one-directional) |
| 22 | CAN_STBY pulled LOW instead (through 1 kΩ to GND) | U4 pin 8 goes low: the transceiver is enabled |

Release each: it returns to off by itself. That is the property the whole split rests on.

## 6. The off-board parts, one at a time

Wire each per `power_box_wiring.md`, and check:
- **Kill chain:** with J8 wired through the E-stop to the 40 A relay coil, the relay pulls in at
  power-up and drops the instant the E-stop is pressed (ACT_12V at J5 goes to 0 V); KILL_SENSE (DB-25
  pin 12) is 0 Ω to GND released and open pressed.
- **Brake light:** 12 V on J10 pulls DB-25 pin 19 low; removing it releases it.
- **BTS7960 and ACS712:** only on the bench rig, with the actuator, through Part 13.3 (`bench_rig.md`).

Then the pod: `pod_board_bringup.md`, and the cable sheet's "once both boards are up" table.
"""


def doc_pod_board():
    pulls = "".join(f"| {pin} | {signal_name(n)} | 3.3 V (10 k pull-up): " +
                    {"KILL_SENSE": "reads as E-stop **engaged**",
                     "BOX_PRESENT": "reads as box **absent** (`ignitionOn` 0)",
                     "BRAKE_LIGHT": "reads as pedal **not pressed**",
                     "CAN_RX": "recessive: the bus looks idle"}.get(signal_name(n), "") + " |\n"
                    for pin, _, n in cable_rows()
                    if n and signal_name(n) in pod.INPUTS and pod.INPUTS[signal_name(n)][2])
    return HEADER + f"""# Pod board: bring-up

BUILD_GUIDE Parts 4.7 and 4.8.2. The pod is powered by USB (VBUS through D1) or by the cable's
5 V (through D2); bring it up on **USB through a USB power meter** first, cable unplugged.

## 1. Before power

- Under magnification: U1 (STM32F405, LQFP-64) pin 1 and no bridges; U3 (AP2112K) and U4 (USBLC6)
  orientation; D1/D2 bands; the crystal Y1 and its load capacitors C12/C13.
- Resistance to GND: VIN_POD (C1 +) and +3V3 (C2 +) well above 1 kΩ.

## 2. First power: USB through a power meter

1. USB-C to a USB power meter, then to a charger (not yet the laptop). Expect tens of milliamps;
   over 200 mA means a fault: unplug.
2. **D3** (green, power) lights. +3V3 at C2 +: **3.30 V**. VIN_POD at C1 +: USB's 5 V less D1's
   drop, about 4.7 V.

## 3. Flash

- **ST-Link on J5** (SWD: 3V3, SWDIO, SWCLK, GND, NRST):
  `cd firmware/sensor_actuator_hub && pio run -t upload` (the vehicle build), or
  `pio run -e bench -t upload` for the bench rig (Part 13.3; never in the car).
- **Without an ST-Link, USB DFU:** hold **BOOT** (SW2), press and release **RESET** (SW1), release
  BOOT: the MCU's ROM bootloader enumerates as `0483:df11`. Then
  `dfu-util -a 0 -s 0x08000000:leave -D .pio/build/stm32f4_hub/firmware.bin`.
- Reset. The firmware enumerates as an **ST virtual COM port, `0483:5740`**, which also proves the
  8 MHz crystal runs (Part 4.7: on the internal oscillator USB may not enumerate at all). Attach it
  to WSL2 (`attach_usb_devices.ps1`); it appears as `/dev/ttyACM0`.

## 4. Part 4.7's checks, cable still unplugged

- `SensorReport` at **50 Hz** (`log_actuator_bench.py status`, or the host).
- The cable-side inputs read their fail-safe values (the pod's own pull-ups):

| Pin | Line | Unplugged |
|---|---|---|
{pulls}
- Hence `killSwitchEngaged = 1`, `ignitionOn = 0`, AckStatus fault **5** (box absent), and nothing
  can be armed.
- The three status LEDs: LINK (blue, D4), ARMED (amber, D5), FAULT (red, D6) as the firmware drives
  them.
- Sensors one at a time, fitted to J2 (IMU), J3 (GNSS), J4 (gesture): each appears in
  `SensorReport` (Part 4.7). **Match each breakout's pin order to the header before fitting**
  (J2/J3 were drawn before the exact breakouts were chosen).

## 5. With the power box

Cable (tested per `db25_cable.md`) between pod J7 and power board J1, the box on its bench supply:
the cable sheet's "once both boards are up" table, then Part 4.7's remaining checks (a relay from a
hand-crafted `ActuationCommand`; release within 200 ms of pulling USB) and the bench rig.
"""


def doc_wiring():
    rows = "".join(f"| {r} | {PWR[r]['value']} | {', '.join(f'{k} {n}' for k, n in PWR[r]['nets'].items())} "
                   f"| {OFFBOARD['power_board'][r]} |\n"
                   for r in sorted(OFFBOARD["power_board"], key=lambda s: int(s[1:])))
    return HEADER + """# Power box: wiring

BUILD_GUIDE Part 4.8.4 and 4.8.6. Everything inside the box that is not on the power board, and
every wire that leaves it. Wire sizes: the actuator path (battery -> 10 A fuse -> kill relay ->
J5/J6 -> BTS7960 -> ACS712 -> actuator) in 2.5 mm²; the 12 V logic feed and relay coils in
0.75 mm²; signals in the DB-25 cable.

```mermaid
flowchart LR
  BAT[Battery +12 V] --> F10[10 A fuse] --> KR[40 A kill relay contacts]
  KR -->|ACT_12V| J5[J5] & J6[J6: BTS7960 B+/B-] & J7[J7: cable magnet]
  ACC[Car ACC fuse] --> J2[J2: 12 V ACC in] --> PB[[Power board]]
  PB -->|+12 V| J8[J8] --> ES1[E-stop NC block 1] --> KC[Kill relay coil] --> G1[GND]
  ES2[E-stop NC block 2] --- J9[J9: KILL_SENSE]
  J13[J13 logic] --> BTS[BTS7960 module] -->|M+| ACS[ACS712 current path] --> ACT[Brake actuator]
  BTS -->|M-| ACT
  J14[J14] --- ACS
  J4[J4: relay coils] --> RL[5 relays] -->|contacts| CAR[Car switch wiring, Part 15.2]
  BL[Brake-light switch] --> J10[J10]
  OBD[OBD-II J1962] --> J11[J11]
  J3[J3: F3 3 A] --> LID[LiDAR M12]
  J1[J1: DB-25] === POD[Windscreen pod]
```

| Ref | Board label | Pin: net | Wired to |
|---|---|---|---|
""" + rows + """
Checks, in order, before the box is closed:
1. With no battery feed: the kill relay coil circuit (J8 -> E-stop -> coil -> GND) by continuity,
   E-stop released: closed; pressed: open.
2. J9's pair: E-stop released: closed; pressed: open (the **second** contact block, independent of
   the first).
3. The actuator path: ACT_12V to GND is not a short (the reading climbs as the BTS7960 module's
   capacitors charge from the meter: that is expected; a steady few ohms is not).
4. Bring-up per `power_board_bringup.md` step 6, then the bench rig.
"""


def doc_bench():
    hx = PWR["J12"]["nets"]
    return HEADER + f"""# Bench rig: setup

BUILD_GUIDE Part 13.3, the mandatory gate before the vehicle. The actuator and a pedal fixture on a
bench, **never in a car**. The run itself is `tools/bench_rig/log_actuator_bench.py checklist`
(it records everything under `bench_runs/`).

1. **Supply:** a 12 V bench supply able to deliver the actuator's stall current (see its
   datasheet; at least 10 A), feeding the power box's battery input (through the 10 A fuse and the
   kill relay, exactly as in the car) and J2 (logic).
2. **Pedal fixture** with the load cell between the actuator's cable and the pedal; the HX711
   amplifier on power board **J12** ({', '.join(f'pin {k}: {v}' for k, v in hx.items())}), which
   travels up the cable on pins 23/24.
3. **Pod** on USB to the laptop, running the bench build: `pio run -e bench -t upload`.
4. **E-stop** wired and within reach of the person at the pedal.
5. **Oscilloscope** (once, recommended) on J13 pin 3 (EN_5V): the independent timing of item 5.
6. Before the first sweep: `log_actuator_bench.py selftest`, then `status` shows the load cell
   valid and the hub armed only when expected.
7. Run `log_actuator_bench.py checklist --attempt N` three times on three separate occasions (Part
   13.4 Stage A). **Item 3, the push-back, is the most important check in the project.**
"""


DOCS = {
    "db25_cable.md": doc_cable,
    "power_board_bringup.md": doc_power_board,
    "pod_board_bringup.md": doc_pod_board,
    "power_box_wiring.md": doc_wiring,
    "bench_rig.md": doc_bench,
}

README = HEADER + """# Assembly day

Generated from the board descriptions, so a pin or a value here always matches the schematics. In
order:

1. [`db25_cable.md`](db25_cable.md): build and test the inter-box cable, alone.
2. [`power_board_bringup.md`](power_board_bringup.md): the power board, current-limited, then its
   fail-safe state, then each line by hand.
3. [`power_box_wiring.md`](power_box_wiring.md): the box's off-board wiring and its checks.
4. [`pod_board_bringup.md`](pod_board_bringup.md): the pod on USB, flashing, Part 4.7's checks,
   then with the box.
5. [`bench_rig.md`](bench_rig.md): Part 13.3's gate, before any vehicle.

Regenerate after any board change: `python3 hardware/gen/assembly.py` (`--check` in review).
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="fail if docs/assembly is out of date")
    a = ap.parse_args()
    check_offboard()
    docs = {name: f() for name, f in DOCS.items()}
    docs["README.md"] = README
    stale = [n for n, text in docs.items() if not (OUT / n).exists() or (OUT / n).read_text() != text]
    if a.check:
        if stale:
            raise SystemExit(f"docs/assembly out of date: {stale} (run hardware/gen/assembly.py)")
        print("docs/assembly up to date")
        return
    OUT.mkdir(parents=True, exist_ok=True)
    for n, text in docs.items():
        (OUT / n).write_text(text)
    print(f"wrote {len(docs)} files to {OUT} (+5 V {V5:.3f} V; current sense idle "
          f"{ACS_IDLE_BOX:.3f} V alone, {ACS_IDLE_PAIR:.3f} V with the pod)")


if __name__ == "__main__":
    main()
