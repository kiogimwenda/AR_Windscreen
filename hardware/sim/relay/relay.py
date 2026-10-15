"""Simulates the power board's relay and magnet drivers (BUILD_GUIDE 4.8.4): the pod's MCU pin
through its 1 k series resistor, the DB-25 cable, the power board's 10 k pull-down, an AO3400A
low-side MOSFET and the coil with its SS14 flyback diode. Run through hardware/sim/run_relay.sh.

Cases, each judged against a stated limit:
  relay_9 / relay_12 / relay_14.4   a signal relay switched on for 30 ms at 9, 12, 14.4 V:
      gate high level >= 2.5 V (the AO3400A's on-resistance is specified from 2.5 V; its
      threshold is at most 1.45 V); MOSFET on-voltage <= 0.1 V; drain peak at turn-off <= 30 V
      (its rating) ; diode peak current <= 1 A (SS14's rating); release time reported
  unplugged   the pod unpowered or the cable out: the pull-down holds the gate low, coil < 1 mA
  magnet_12 / magnet_14.4   the cable magnet (Q2, D14) on the actuator rail: the same checks,
      and how long it takes to let go (its current down to 10 %), reported

Models (stated in the results): no vendor SPICE model is published for the AO3400A or SS14, so
both are built from their datasheet figures: AO3400A threshold 1.05 V, about 30 mOhm at a 3 V
gate, Ciss about 630 pF, body diode; SS14 0.42 V at 1 A, 40 V. The coils are typical values for
parts not yet bought: automotive relay 85 Ohm / 0.35 H; holding magnet 48 Ohm (0.25 A) / 0.15 H.
The cable: 0.2 Ohm, 150 pF. The MCU pin: 3.3 V through 40 Ohm.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlib import ROOT, read_raw, results_table, run_netlists, window  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = ROOT / "build" / "sim" / "relay"
T_ON, T_OFF, T_END = 1e-3, 31e-3, 61e-3
VECTORS = ["v(gate)", "v(drain)", "v(supply)", "lcoil#branch", "@d1[id]", "v(mcu)"]

NETLIST = """* AR Windscreen power board: {what}, case {name}
VSUP SUPPLY 0 {vsup}
{mcu}
RSER MCUO CAB 1k
RCAB CAB GATE 0.2
CCAB GATE 0 150p
RPD GATE 0 10k
XQ DRAIN GATE 0 AO3400A
D1 DRAIN SUPPLY SS14
LCOIL SUPPLY LX {l}
RCOIL LX DRAIN {r}
.subckt AO3400A D G S
M1 D G S S NMOD L=1u W=1u
CGS G S 550p
CGD G D 80p
DB S D DBODY
.model NMOD NMOS(LEVEL=1 VTO=1.05 KP=17 LAMBDA=0.01)
.model DBODY D(IS=1e-12 N=1.2 RS=0.02 BV=32 IBV=1m)
.ends AO3400A
.model SS14 D(IS=2u N=1.1 RS=0.05 CJO=110p M=0.45 VJ=0.35 BV=40 IBV=0.5m)
.tran 0.5u {t_end} 0 0.5u
.end
"""
MCU_ON = (f"VMCU MCU 0 PWL(0 0 {T_ON} 0 {T_ON + 10e-9} 3.3 {T_OFF} 3.3 {T_OFF + 10e-9} 0)\n"
          "RMCU MCU MCUO 40")
MCU_UNPLUGGED = "RMCU MCUO 0 1G\nVMCU MCU 0 0"  # nothing drives the line: only the pull-down

RELAY = dict(what="signal relay driver (Q3-Q7)", r="85", l="0.35")
MAGNET = dict(what="cable magnet driver (Q2)", r="48", l="0.15")
CASES = {
    "relay_9": (RELAY, "9", MCU_ON), "relay_12": (RELAY, "12", MCU_ON),
    "relay_14.4": (RELAY, "14.4", MCU_ON), "unplugged": (RELAY, "14.4", MCU_UNPLUGGED),
    "magnet_12": (MAGNET, "12", MCU_ON), "magnet_14.4": (MAGNET, "14.4", MCU_ON),
}


def main():
    if "--reuse" in sys.argv:
        data = {name: read_raw(WORK / f"{name}.raw") for name in CASES}
    else:
        nets = {name: NETLIST.format(name=name, vsup=v, mcu=mcu, t_end=T_END, **coil)
                for name, (coil, v, mcu) in CASES.items()}
        data = run_netlists(WORK, nets, VECTORS)
    rows, ok = [], True

    def judge(case, what, value, limit, passed):
        nonlocal ok
        ok &= bool(passed)
        rows.append((case, what, value, limit, "PASS" if passed else "FAIL"))

    def info(case, what, value):
        rows.append((case, what, value, "reported", ""))

    for name in [n for n in CASES if n != "unplugged"]:
        d = data[name]
        on = window(d, T_OFF - 5e-3, T_OFF - 1e-3)  # steady, before switching off
        i_on = float(np.mean(on["i(lcoil)"]))
        gate = float(np.mean(on["v(gate)"]))
        vds = float(np.mean(on["v(drain)"]))
        vsup = float(np.mean(on["v(supply)"]))
        judge(name, "gate high level", f"{gate:.2f} V", ">= 2.5 V", gate >= 2.5)
        judge(name, "MOSFET on-voltage", f"{vds * 1000:.1f} mV", "<= 100 mV", vds <= 0.1)
        info(name, "coil current / voltage", f"{i_on * 1000:.0f} mA / {vsup - vds:.2f} V")
        off = window(d, T_OFF, T_END)
        peak = float(np.max(off["v(drain)"]))
        judge(name, "drain peak at turn-off", f"{peak:.1f} V", "<= 30 V", peak <= 30)
        idiode = float(np.max(d["i(@d1[id])"]))
        judge(name, "diode peak current", f"{idiode * 1000:.0f} mA", "<= 1 A", idiode <= 1.0)
        t = off["time"]
        below = np.where(off["i(lcoil)"] < 0.1 * i_on)[0]
        release = (t[below[0]] - T_OFF) * 1e3 if len(below) else float("nan")
        info(name, "current down to 10 % after switch-off", f"{release:.1f} ms")
        gw = window(d, T_ON, T_ON + 200e-6)
        rise = np.where(gw["v(gate)"] >= 0.9 * gate)[0]
        info(name, "gate rise (to 90 %)", f"{(gw['time'][rise[0]] - T_ON) * 1e6:.1f} us" if len(rise) else "-")
    d = data["unplugged"]
    leak = float(np.max(np.abs(d["i(lcoil)"])))
    gmax = float(np.max(d["v(gate)"]))
    judge("unplugged", "coil current", f"{leak * 1e6:.2f} uA", "< 1 mA", leak < 1e-3)
    judge("unplugged", "gate", f"{gmax:.3f} V", "< 0.5 V", gmax < 0.5)

    plots(data)
    notes = ["Models: no vendor SPICE model is published for the AO3400A or SS14; both are built from "
             "their datasheet figures (see `relay.py`). The coils are typical values for parts not yet "
             "bought: relay 85 Ohm / 0.35 H, magnet 48 Ohm / 0.15 H. Measure the real ones and rerun.", "",
             "Release time: with a plain flyback diode the coil current decays with the coil's own time "
             "constant (L/R), so a relay or the magnet lets go some milliseconds after switch-off. A relay "
             "drops out well before its current reaches 10 %; the 10 % figure is the slow, cautious bound."]
    (HERE / "results.md").write_text(results_table(
        "Relay and magnet driver simulation results", "Plots: `relay_results.png`.", rows, ok, notes))
    print("\n".join(f"{r[4] or 'info'} {r[0]}: {r[1]} = {r[2]} ({r[3]})" for r in rows))
    print("ALL PASS" if ok else "SOME FAIL")
    sys.exit(0 if ok else 1)


def plots(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(9, 10))
    d = data["relay_12"]
    t = d["time"] * 1e3
    ax[0].plot(t, d["v(mcu)"], label="MCU pin")
    ax[0].plot(t, d["v(gate)"], label="MOSFET gate (after 1 k, cable, 10 k)")
    ax[0].set(title="Relay at 12 V: the drive", xlabel="ms", ylabel="V")
    ax[0].legend()
    ax[1].plot(t, d["v(drain)"], label="drain (coil low side)")
    a1 = ax[1].twinx()
    a1.plot(t, d["i(lcoil)"] * 1000, color="tab:red", label="coil current")
    a1.set_ylabel("coil current, mA")
    ax[1].set(title="Relay at 12 V: on for 30 ms, then off (clamped by the SS14)", xlabel="ms", ylabel="V")
    for name, colour in (("relay_14.4", "tab:blue"), ("magnet_14.4", "tab:orange")):
        d = window(data[name], T_OFF - 2e-3, T_END)
        i0 = float(np.mean(window(data[name], T_OFF - 3e-3, T_OFF - 1e-3)["i(lcoil)"]))
        ax[2].plot((d["time"] - T_OFF) * 1e3, d["i(lcoil)"] / i0 * 100, color=colour, label=name)
    ax[2].axhline(10, color="grey", ls="--", lw=0.8)
    ax[2].set(title="Switch-off: coil current (percent of on-current); dashed: 10 %",
              xlabel="ms after switch-off", ylabel="%")
    ax[2].legend()
    fig.tight_layout()
    fig.savefig(HERE / "relay_results.png", dpi=110)


if __name__ == "__main__":
    main()
