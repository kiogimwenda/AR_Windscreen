"""Simulates the brake-light input (BUILD_GUIDE 4.8.4): the car's brake-lamp feed on the power
board's J10 -> R63 (4.7 k) -> the PC817 optocoupler's LED (D21, 1N4148W, across it against
reversal); its phototransistor pulls BRAKE_LIGHT low through the DB-25 cable to the pod, where a
10 k pull-up to 3.3 V and the 1 k + 10 nF filter feed the MCU's PB5 (low = pedal pressed). Run
through hardware/sim/run_brake.sh.

Cases, each judged against a stated limit:
  sweep_ctr25 / sweep_ctr600   the lamp feed ramped 0 -> 16 V slowly, with the weakest
      optocoupler (CTR 25 %: the PC817's 50 % minimum, halved for heat, ageing and low LED
      current) and the strongest (600 %, its maximum): pressed must read LOW (<= 0.99 V,
      0.3 x VDD) from 9 V with the weakest part; released must read HIGH (>= 2.31 V, 0.7 x VDD)
      up to 0.5 V on the line (the offset between the lamp's ground at the back of the car and
      the power box's) with the strongest; the line voltage where each starts to read pressed is
      reported; the
      LED current within 50 mA; R63's dissipation within 0.125 W (0805) at 16 V
  reverse        -14.4 V on J10 (wired the wrong way round): the LED's reverse voltage within its
                 6 V rating, D21's current within 300 mA
  jump_24 / dump_35   a 24 V jump-start (60 s) and the suppressed load dump (35 V, 400 ms): R63's
                 power, against its short-time overload rating (2.5 x rated voltage for 5 s:
                 6.25 x 0.125 W = 0.78 W) for the dump, reported for the jump-start
  timing         13.5 V for 10 ms with a typical part (CTR 100 %): how soon the pin reads
                 pressed and released (reported: the brake light is a 100 ms-scale signal)
  *_2k2          the weakest-part sweep and the jump-start with the 2.2 k first fitted, for comparison
                 (2026-10-16: it ran at 237 mW in the jump-start, so R63 became 4.7 k)

Models (stated in the results): no SPICE model is published for the PC817; it is built from the
datasheet: the LED (1.2 V at 20 mA), the phototransistor as an NPN whose base is fed the LED
current times CTR/BF, collector-base capacitance 12 pF. 1N4148W: the usual 1N4148 model. The
cable: 0.4 Ohm, 150 pF.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlib import ROOT, read_raw, results_table, run_netlists, window  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = ROOT / "build" / "sim" / "brake"
VECTORS = ["v(src)", "v(bla)", "v(coll)", "v(pin)", "vsense#branch", "@d21[id]", "@r63[i]"]
BF = 500
VIL, VIH = 0.99, 2.31

NETLIST = """* AR Windscreen power board: brake-light input, case {name}
VBL SRC 0 {src}
RSRC SRC BLIN 0.1
R63 BLIN BLA {r63}
D21 0 BLA D1N4148
DLED BLA LEDK DPC817
VSENSE LEDK 0 0
FPH 0 PB VSENSE {k}
QPH COLL PB 0 NPH
RBE PB 0 10Meg
RCAB COLL CAB 0.4
CCAB CAB 0 150p
VDD VDD 0 3.3
RPU VDD CAB 10k
RF CAB PIN 1k
CF PIN 0 10n
.model DPC817 D(IS=1.7e-12 N=2 RS=1 BV=6 IBV=10u CJO=30p)
.model D1N4148 D(IS=2.52n RS=0.568 N=1.752 BV=100 IBV=100u CJO=4p TT=11.5n)
.model NPH NPN(BF={bf} IS=1e-14 VAF=100 CJC=12p CJE=8p)
.tran {step} {t_end} 0 {step}
.end
"""
RAMP = dict(src="PWL(0 0 80m 16)", t_end="80m", step="20u")
CASES = {
    "sweep_ctr25": dict(RAMP, ctr=25, r63="4.7k"),
    "sweep_ctr600": dict(RAMP, ctr=600, r63="4.7k"),
    "sweep_ctr25_2k2": dict(RAMP, ctr=25, r63="2.2k"),
    "reverse": dict(src="-14.4", t_end="1m", step="1u", ctr=100, r63="4.7k"),
    "jump_24": dict(src="24", t_end="1m", step="1u", ctr=100, r63="4.7k"),
    "jump_24_2k2": dict(src="24", t_end="1m", step="1u", ctr=100, r63="2.2k"),
    "dump_35": dict(src="35", t_end="1m", step="1u", ctr=100, r63="4.7k"),
    "timing": dict(src="PWL(0 0 1m 0 1.01m 13.5 11m 13.5 11.01m 0)", t_end="15m", step="1u",
                   ctr=100, r63="4.7k"),
}


def at(d, key, vsrc):
    """The value of d[key] where the ramped source passes vsrc."""
    return float(np.interp(vsrc, d["v(src)"], d[key]))


def first_cross(d, key, level, below):
    idx = np.where(d[key] <= level if below else d[key] >= level)[0]
    return idx[0] if len(idx) else None


def main():
    if "--reuse" in sys.argv:
        data = {name: read_raw(WORK / f"{name}.raw") for name in CASES}
    else:
        nets = {name: NETLIST.format(name=name, src=c["src"], t_end=c["t_end"], step=c["step"],
                                     r63=c["r63"], k=c["ctr"] / 100 / BF, bf=BF)
                for name, c in CASES.items()}
        data = run_netlists(WORK, nets, VECTORS)
    rows, ok = [], True

    def judge(case, what, value, limit, passed):
        nonlocal ok
        ok &= bool(passed)
        rows.append((case, what, value, limit, "PASS" if passed else "FAIL"))

    def info(case, what, value):
        rows.append((case, what, value, "reported", ""))

    def r_ohms(name):
        return 2200.0 if CASES[name]["r63"] == "2.2k" else 4700.0

    for name in ("sweep_ctr25", "sweep_ctr25_2k2"):
        d = data[name]
        pin9 = at(d, "v(pin)", 9.0)
        judge(name, "pin at 9 V (pressed)", f"{pin9:.2f} V", f"<= {VIL} V", pin9 <= VIL)
        i = first_cross(d, "v(pin)", VIL, True)
        info(name, "reads pressed from", f"{d['v(src)'][i]:.2f} V" if i is not None else "never")
        info(name, "LED current at 9 / 12 / 16 V",
             " / ".join(f"{at(d, 'i(vsense)', v) * 1000:.1f}" for v in (9, 12, 16)) + " mA")
        p16 = at(d, "i(@r63[i])", 16.0) ** 2 * r_ohms(name)
        judge(name, "R63 dissipation at 16 V", f"{p16 * 1000:.0f} mW", "<= 125 mW", p16 <= 0.125)
    d = data["sweep_ctr25"]
    iled = float(np.max(d["i(vsense)"]))
    judge("sweep_ctr25", "LED current, maximum (16 V)", f"{iled * 1000:.1f} mA", "<= 50 mA", iled <= 0.05)
    d = data["sweep_ctr600"]
    pin1 = at(d, "v(pin)", 0.5)
    judge("sweep_ctr600", "pin at 0.5 V on the line (released)", f"{pin1:.2f} V", f">= {VIH} V", pin1 >= VIH)
    for lvl, below, what in ((VIH, True, "stops reading released (pin < 2.31 V) from"),
                             (VIL, True, "reads pressed from")):
        i = first_cross(d, "v(pin)", lvl, below)
        info("sweep_ctr600", what, f"{d['v(src)'][i]:.2f} V" if i is not None else "never")

    d = data["reverse"]
    vrev = float(np.min(d["v(bla)"]))
    judge("reverse", "LED reverse voltage", f"{-vrev:.2f} V", "<= 6 V", -vrev <= 6)
    idr = float(np.max(np.abs(d["i(@d21[id])"])))
    judge("reverse", "D21 current", f"{idr * 1000:.1f} mA", "<= 300 mA", idr <= 0.3)
    pin = float(np.mean(d["v(pin)"]))
    judge("reverse", "pin (reads released)", f"{pin:.2f} V", f">= {VIH} V", pin >= VIH)

    for name in ("jump_24", "jump_24_2k2", "dump_35"):
        d = data[name]
        p = float(np.mean(d["i(@r63[i])"][-50:])) ** 2 * r_ohms(name)
        if name == "dump_35":
            judge(name, "R63 dissipation (400 ms)", f"{p * 1000:.0f} mW", "<= 780 mW (short-time overload)", p <= 0.78)
        elif name == "jump_24":
            judge(name, "R63 dissipation (60 s)", f"{p * 1000:.0f} mW", "<= 125 mW (0805)", p <= 0.125)
        else:
            info(name, "R63 dissipation (60 s)", f"{p * 1000:.0f} mW (0805 rated 125 mW)")
        info(name, "LED current", f"{float(np.mean(d['i(vsense)'][-50:])) * 1000:.1f} mA")

    d = data["timing"]
    on = window(d, 1e-3, 11e-3)
    i = first_cross(on, "v(pin)", VIL, True)
    info("timing", "line on -> pin reads pressed", f"{(on['time'][i] - 1e-3) * 1e6:.0f} us" if i is not None else "never")
    off = window(d, 11e-3, 15e-3)
    i = first_cross(off, "v(pin)", VIH, False)
    info("timing", "line off -> pin reads released", f"{(off['time'][i] - 11e-3) * 1e6:.0f} us" if i is not None else "never")

    plots(data)
    notes = ["Models: no SPICE model is published for the PC817; it is built from the datasheet "
             "(see `brake.py`). CTR 25 % is the 50 % minimum halved for heat, ageing and the low LED "
             "current; 600 % is the datasheet's maximum (an unranked PC817 may be anywhere between).", "",
             "MCU thresholds: 0.3 x VDD and 0.7 x VDD, the CMOS levels the STM32F405's datasheet guarantees.", "",
             "**Findings.** (1) The input's threshold is the LED's forward voltage: a strong PC817 reads "
             "pressed from about 1 V on the line, a weak one from about 6 V. A lamp-outage check that "
             "puts a volt or more on a dark brake-lamp line would read as the pedal pressed (the safe "
             "direction: the system yields to the driver). A resistor across the LED (1 k on a later "
             "revision: nothing conducts below about 3 V) would make the threshold definite. "
             "(2) R63 was 2.2 k: 100 mW at 16 V, within its 125 mW, but 237 mW during a 60 s, 24 V "
             "jump-start (the `_2k2` cases). Changed to 4.7 k on 2026-10-16 (a value change, same "
             "footprint): the weakest part still reads pressed from 6 V."]
    (HERE / "results.md").write_text(results_table(
        "Brake-light input simulation results", "Plots: `brake_results.png`.", rows, ok, notes))
    print("\n".join(f"{r[4] or 'info'} {r[0]}: {r[1]} = {r[2]} ({r[3]})" for r in rows))
    print("ALL PASS" if ok else "SOME FAIL")
    sys.exit(0 if ok else 1)


def plots(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(9, 8))
    for name, colour in (("sweep_ctr25", "tab:red"), ("sweep_ctr600", "tab:blue"),
                         ("sweep_ctr25_2k2", "tab:orange")):
        d = data[name]
        ax[0].plot(d["v(src)"], d["v(pin)"], color=colour, label=name)
    for lvl in (VIL, VIH):
        ax[0].axhline(lvl, color="grey", ls="--", lw=0.8)
    ax[0].set(title="MCU pin against the brake-lamp line (dashed: 0.3 and 0.7 x VDD)",
              xlabel="brake-lamp line, V", ylabel="pin, V")
    ax[0].legend()
    d = data["timing"]
    t = d["time"] * 1e3
    ax[1].plot(t, d["v(src)"] / 13.5 * 3.3, label="lamp line (scaled to 3.3 V)")
    ax[1].plot(t, d["v(coll)"], label="phototransistor collector (cable)")
    ax[1].plot(t, d["v(pin)"], label="MCU pin (after 1 k + 10 nF)")
    ax[1].set(title="Pedal pressed for 10 ms (CTR 100 %)", xlabel="ms", ylabel="V")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(HERE / "brake_results.png", dpi=110)


if __name__ == "__main__":
    main()
