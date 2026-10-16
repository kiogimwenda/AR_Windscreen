"""Simulates the pod's supply (BUILD_GUIDE 4.8.2): the power box's +5 V through polyfuse F2 and
the DB-25 cable (pins 1-2 out, five ground pins back) into D2, and the laptop's USB VBUS into D1;
the two Schottkys diode-OR onto VIN_POD (C1, 1 uF), which feeds the AP2112K-3.3 (U3) and the
HC-05 header; +3V3 (C2, 4.7 uF, plus the decoupling) feeds the MCU, the IMU, the GNSS, the gesture
puck and the LEDs. Run through hardware/sim/run_pod_supply.sh.

Cases, each judged against a stated limit:
  cable_only    the worst cable path (F2 at its 1.0 Ohm maximum, 3 m of 26 AWG) at the budget
                load: +3V3 in regulation (>= 3.2 V), the LDO's headroom, F2 within its 500 mA
                hold current
  usb_only      the laptop alone at USB's lowest allowed 4.40 V, through 1 m of USB cable
  both          both present: +3V3, and which source carries the load
  cable_out     the cable unplugged while running on both: +3V3 never below 3.0 V (the GNSS
                module's lowest supply; the MCU runs from 1.8 V, the IMU from 2.4 V)
  usb_plugin    the laptop plugged in to an unpowered pod: the inrush current and its charge,
                against USB 2.0's limit (no more than 10 uF and 50 uC at attach)
  *_10u         the inrush, cable-out and cable-only cases with the 10 uF C1 and C2 first fitted,
                for comparison (2026-10-16: their inrush failed, so they became 1 uF and 4.7 uF)
  backfeed      cable powered, laptop asleep (its VBUS off, 100 kOhm on it): current back into
                the laptop through D1's leakage, reported
Loads, at their datasheets' maxima (the budget): MCU 90 mA, GNSS 67 mA (acquisition), IMU
15 mA, gesture 1 mA plus its IR LED's 100 mA pulses (10 us every 100 us while sensing), LEDs
8 mA, pull-ups 3 mA; HC-05 40 mA on VIN_POD (the ELM327 build only, included).

Models (stated in the results): no SPICE model is published for the AP2112K; it is a behavioural
regulator from the datasheet (3.3 V, dropout 250 mV at 600 mA, 0.8 A current limit, a 50 A/V
error amplifier into its output capacitor). SS14 as in the relay simulation, with its reverse
leakage. Polyfuse as a fixed resistance (it trips only above 1 A).
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlib import ROOT, read_raw, results_table, run_netlists, window  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = ROOT / "build" / "sim" / "pod_supply"
VECTORS = ["v(vin)", "v(v3)", "v(vbus)", "v(x5)", "vcab#branch", "vusb#branch", "@bldo[i]"]

NETLIST = """* AR Windscreen pod board: supply, case {name}
* --- the power box: +5 V, F2, the DB-25 cable (2 wires out, 5 back), D2
VBOX BOX5 0 {vbox}
RF2 BOX5 F2O {rf2}
VCAB F2O CABA 0
RCAB5 CABA X5A {rc5}
LCAB X5A X5 0.6u
{cable_switch}
D2 X5P VIN SS14
* --- the laptop: VBUS through the USB cable, D1
VUSBSRC USBS 0 {vusb}
VUSB USBS USBA 0
RUSB USBA VBUSL {ru}
LUSB VBUSL VBUS 1u
{usb_load}
D1 VBUS VIN SS14
* --- VIN_POD: C1, the HC-05, the LDO
C1 VIN 0 {c1} IC={ic}
IHC05 VIN 0 {ihc}
BLDO VIN V3 I = min(0.8, max(0, min(50*(3.3 - V(V3)), (V(VIN) - V(V3))/0.42)))
RLDOQ VIN 0 60k
* --- +3V3: C2 + decoupling, the loads
C2 V3 0 {c2} IC={ic}
CDEC V3 0 2.5u IC={ic}
BLOAD V3 0 I = {iload} * min(1, max(V(V3), 0) / 3.0)
IGEST V3 0 PULSE(0 0.1 0 1u 1u 10u 100u)
.model SS14 D(IS=2u N=1.1 RS=0.05 CJO=110p M=0.45 VJ=0.35 BV=40 IBV=0.5m)
.tran {step} {t_end} 0 {step} UIC
.end
"""
I3V3 = 0.090 + 0.067 + 0.015 + 0.001 + 0.008 + 0.003     # the budget, without the gesture LED
# 26 AWG: 0.134 Ohm/m; 3 m; two wires out, five back, plus the contacts. The ground return's drop
# is added to the 5 V wire's resistance (one loop resistance: the same voltage at D2).
CABLE = dict(rc5=f"{(0.134 * 3 / 2 + 0.05) + (0.134 * 3 / 5 + 0.03):.3f}")
BASE = dict(c1="1u", c2="4.7u", vbox="5.0", rf2="1.0", vusb="0", ru="0.25", ihc="0.04", iload=f"{I3V3:.3f}", ic="3.3",
            usb_load="", t_end="5m", step="0.5u", **CABLE)
CONNECTED = "RCONN X5 X5G 1u\nVGND X5G X5P 0"


CASES = {
    "cable_only": dict(BASE, vusb="0", cable_switch=CONNECTED, usb_load="RUSBOFF VBUS 0 100k"),
    "usb_only": dict(BASE, vbox="0", rf2="1e6", vusb="4.40", ru="0.25",
                     cable_switch=CONNECTED),
    "both": dict(BASE, vusb="5.0", cable_switch=CONNECTED),
    "cable_out": dict(BASE, vusb="4.75", t_end="6m",
                      cable_switch="SCAB X5 X5P CTL 0 SWM\nVCTL CTL 0 PWL(0 1 2m 1 2.001m 0)\n"
                                   ".model SWM SW(RON=0.01 ROFF=1e7 VT=0.5 VH=0.1)"),
    "usb_plugin": dict(BASE, vbox="0", rf2="1e6", ic="0", t_end="3m", step="0.1u",
                       vusb="PWL(0 0 0.1m 0 0.1001m 5.0)", cable_switch=CONNECTED),
    "backfeed": dict(BASE, vusb="0", ru="1e6", usb_load="RUSBOFF VBUS 0 100k", cable_switch=CONNECTED),
}
# The values first fitted, for comparison (the AP2112K asks for at least 1 uF on each side)
OLD = dict(c1="10u", c2="10u")
CASES["usb_plugin_10u"] = dict(CASES["usb_plugin"], **OLD)
CASES["cable_out_10u"] = dict(CASES["cable_out"], **OLD)
CASES["cable_only_10u"] = dict(CASES["cable_only"], **OLD)
COMPARISON = {"usb_plugin_10u"}  # reported, not judged: it is no longer on the board


def main():
    if "--reuse" in sys.argv:
        data = {name: read_raw(WORK / f"{name}.raw") for name in CASES}
    else:
        nets = {name: NETLIST.format(name=name, **c) for name, c in CASES.items()}
        data = run_netlists(WORK, nets, VECTORS)
    rows, ok = [], True

    def judge(case, what, value, limit, passed):
        nonlocal ok
        ok &= bool(passed)
        rows.append((case, what, value, limit, "PASS" if passed else "FAIL"))

    def info(case, what, value):
        rows.append((case, what, value, "reported", ""))

    def steady(name):
        return window(data[name], 3e-3, 5e-3)

    for name in ("cable_only", "usb_only", "both", "cable_only_10u"):
        d = steady(name)
        v3min = float(np.min(d["v(v3)"]))
        judge(name, "+3V3, lowest (with the gesture LED's pulses)", f"{v3min:.3f} V", ">= 3.2 V", v3min >= 3.2)
        head = float(np.min(d["v(vin)"] - d["v(v3)"]))
        info(name, "VIN_POD, lowest / LDO headroom", f"{float(np.min(d['v(vin)'])):.2f} V / {head * 1000:.0f} mV")
        icab = float(np.mean(d["i(vcab)"]))
        info(name, "from the cable / from USB (mean)",
             f"{icab * 1000:.0f} mA / {float(np.mean(d['i(vusb)'])) * 1000:.0f} mA")
        if name == "cable_only":
            judge(name, "F2 current (mean)", f"{icab * 1000:.0f} mA", "<= 500 mA (hold)", icab <= 0.5)
            ldo_p = float(np.mean((d["v(vin)"] - d["v(v3)"]) * d["i(@bldo[i])"]))
            info(name, "LDO dissipation", f"{ldo_p * 1000:.0f} mW (SOT-23-5: ~0.4 W at 70 C ambient)")
    for name in ("cable_out", "cable_out_10u"):
        d = window(data[name], 1.9e-3, 6e-3)
        v3min = float(np.min(d["v(v3)"]))
        judge(name, "+3V3, lowest after the cable is pulled", f"{v3min:.3f} V", ">= 3.0 V", v3min >= 3.0)
    for name in ("usb_plugin", "usb_plugin_10u"):
        d = data[name]
        i = np.abs(d["i(vusb)"])
        t = d["time"]
        m = (t >= 0.1e-3) & (t <= 0.2e-3)  # the first 100 us after attach
        i_run = float(np.mean(i[t >= t[-1] - 0.5e-3]))  # the running load, not inrush
        q = float(np.trapezoid(np.clip(i[m] - i_run, 0, None), t[m]))
        info(name, "inrush peak", f"{float(np.max(i)):.2f} A")
        if name in COMPARISON:
            info(name, "inrush charge (above the running load), first 100 us", f"{q * 1e6:.0f} uC (limit 50 uC)")
        else:
            judge(name, "inrush charge (above the running load), first 100 us", f"{q * 1e6:.0f} uC", "<= 50 uC (USB 2.0)", q <= 50e-6)
        up = np.where(d["v(v3)"] >= 3.2)[0]
        info(name, "+3V3 up (to 3.2 V) after attach", f"{(t[up[0]] - 0.1e-3) * 1e6:.0f} us" if len(up) else "never")
    d = steady("backfeed")
    ib = float(np.mean(np.abs(d["i(vusb)"])))
    info("backfeed", "current into the sleeping laptop's VBUS (25 C model)", f"{ib * 1e6:.1f} uA, VBUS at {float(np.mean(d['v(vbus)'])):.2f} V")

    plots(data)
    notes = ["Models: no SPICE model is published for the AP2112K; it is behavioural, from the datasheet "
             "(see `pod_supply.py`): it shows dropout and current limiting, not the regulator's own fast "
             "transient response, which the datasheet gives as tens of millivolts for a 100 mA step. SS14 as in "
             "the relay simulation; its leakage at 25 C (it rises about tenfold by 75 C).", "",
             "USB inrush: USB 2.0 allows a device to present at most 10 uF at attach (the 50 uC figure). With the "
             "10 uF C1 and C2 first fitted (the `_10u` cases) the pod presented about 22 uF and drew 88 uC; C1 "
             "and C2 became 1 uF and 4.7 uF on 2026-10-16 (value changes, same footprints; the AP2112K needs "
             "1 uF on each side)."]
    (HERE / "results.md").write_text(results_table(
        "Pod supply simulation results", "Plots: `pod_supply_results.png`.", rows, ok, notes))
    print("\n".join(f"{r[4] or 'info'} {r[0]}: {r[1]} = {r[2]} ({r[3]})" for r in rows))
    print("ALL PASS" if ok else "SOME FAIL")
    sys.exit(0 if ok else 1)


def plots(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(9, 8))
    d = data["cable_out"]
    t = d["time"] * 1e3
    ax[0].plot(t, d["v(vin)"], label="VIN_POD")
    ax[0].plot(t, d["v(v3)"], label="+3V3")
    ax[0].plot(t, d["v(x5)"], label="cable +5 V (the plug side of the connector)")
    ax[0].axvline(2.0, color="grey", ls="--", lw=0.8)
    ax[0].set(title="Running on both; the cable pulled at 2 ms (USB at 4.75 V)", xlabel="ms", ylabel="V")
    ax[0].legend()
    d = data["usb_plugin"]
    t = d["time"] * 1e3
    ax[1].plot(t, np.abs(d["i(vusb)"]), label="USB current, A")
    ax[1].plot(t, d["v(v3)"], label="+3V3, V")
    ax[1].set(title="Laptop plugged in at 0.1 ms (cable unpowered)", xlabel="ms")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(HERE / "pod_supply_results.png", dpi=110)


if __name__ == "__main__":
    main()
