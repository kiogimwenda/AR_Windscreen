"""Simulates the power board's 12 V input protection (BUILD_GUIDE 4.8.4): J2 -> 3 A fuse F1 ->
bidirectional TVS D10 (SMCJ33CA) -> reverse-polarity Schottky D11 (B560C) -> the +12V rail with
C20 (47 uF 63 V), feeding the 5 V converter, the LiDAR and the relay coils. Run through
hardware/sim/run_input.sh.

The car's disturbances, from ISO 16750-2 (supply) and ISO 7637-2 (transients), for a 12 V system:
  normal_12 / normal_14.4  running: rail voltage, D11's dissipation, fuse current
  reverse                  battery leads reversed (-14.4 V for 60 s): the rail stays near 0 V
  jump_24                  jump-start from a 24 V source (60 s): the TVS stays out of the way
  ld_<tvs>                 load dump with the car's central suppression (ISO 16750-2 test B):
                           14 V -> 35 V, 0.5 Ohm source, 10 ms rise, 400 ms
  p1_<tvs> / p2a_<tvs>     ISO 7637-2 pulse 1 (-150 V, 10 Ohm, 2 ms) and pulse 2a (+112 V,
                           2 Ohm, 50 us) on 13.5 V
for the TVS fitted (SMCJ33CA) and the one first chosen (SMCJ24CA), against stated limits:
  the rail never above 60 V (the TPS54360's rating; C20 63 V) nor below -1 V; the TVS's
  energy within its pulse rating (1500 W for 10/1000 us, derated for longer pulses as the
  datasheet's curve, roughly with the square root of the duration); the fuse not overloaded in
  normal running.
Models (stated in the results): TVS from the datasheets (SMCJ24CA breakdown 26.7-29.5 V,
38.9 V at 38.6 A; SMCJ33CA 36.7-40.6 V, 53.3 V at 28.1 A) as back-to-back diodes; B560C a generic
5 A / 60 V Schottky (as in the converter simulation); the converter and the LiDAR as constant
power loads (6 W and 8 W), the five relay coils on (17 Ohm); the harness 1 uH and 50 mOhm.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simlib import ROOT, read_raw, results_table, run_netlists, window  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = ROOT / "build" / "sim" / "input"
VECTORS = ["v(src)", "v(fused)", "v(rail)", "vfuse#branch", "@d.xtvs.da[id]", "v(xtvs.mid)"]

TVS = {  # name: (breakdown of each half (V), series resistance (Ohm)), from the datasheet's figures
    "SMCJ24CA": (27.4, 0.27),
    "SMCJ33CA": (37.9, 0.50),
}
NETLIST = """* AR Windscreen power board: 12 V input protection, case {name}
VSRC SRCI 0 {src}
RSRC SRCI SRCR {ri}
LHARN SRCR SRC 1u
RHARN SRC J2 50m
VFUSE J2 J2M 0
RFUSE J2M FUSED 30m
XTVS FUSED 0 TVS_{tvs}
D11 FUSED RAIL B560C
XC20 RAIL 0 CPOL
XC21 RAIL 0 CER
BBUCK RAIL 0 I = V(RAIL) > 6 ? 6 / V(RAIL) : V(RAIL) / 6
BLIDAR RAIL 0 I = V(RAIL) > 9 ? 8 / V(RAIL) : V(RAIL) / 10
RRELAYS RAIL 0 17
.subckt CPOL a b
C1 a 1 47u
R1 1 b 0.3
.ends
.subckt CER a b
C1 a 1 4.4u
R1 1 b 5m
.ends
.subckt TVS_SMCJ24CA a b
DA a mid DZ24
DB b mid DZ24
.model DZ24 D(IS=1n N=1.5 RS={rs24} BV={bv24} IBV=1m)
.ends
.subckt TVS_SMCJ33CA a b
DA a mid DZ33
DB b mid DZ33
.model DZ33 D(IS=1n N=1.5 RS={rs33} BV={bv33} IBV=1m)
.ends
.model B560C D(IS=5u N=1.05 RS=15m CJO=500p M=0.45 VJ=0.35 BV=60 IBV=0.5m TT=5n)
.tran {step} {tstop} 0 {step}
.end
"""
# Source waveforms: (PWL source, source resistance, stop time, time step)
SOURCES = {
    "normal_12": ("PWL(0 0 1m 12)", "10m", "20m", "10u"),
    "normal_14.4": ("PWL(0 0 1m 14.4)", "10m", "20m", "10u"),
    "reverse": ("PWL(0 0 1m -14.4)", "10m", "20m", "10u"),
    "jump_24": ("PWL(0 0 1m 24)", "10m", "20m", "10u"),
    # ISO 16750-2 test B, suppressed load dump: 14 V, rises to 35 V in 10 ms, back by 400 ms
    "ld": ("PWL(0 0 1m 14 20m 14 30m 35 420m 14 500m 14)", "0.5", "500m", "20u"),
    # ISO 7637-2 pulse 1: -150 V, 10 Ohm, 1 us rise, 2 ms decay, on 13.5 V
    "p1": ("PWL(0 0 1m 13.5 5m 13.5 5.001m -150 7m 13.5 15m 13.5)", "10", "15m", "0.2u"),
    # ISO 7637-2 pulse 2a: +112 V, 2 Ohm, 1 us rise, 50 us, on 13.5 V
    "p2a": ("PWL(0 0 1m 13.5 5m 13.5 5.001m 125.5 5.05m 13.5 10m 13.5)", "2", "10m", "0.1u"),
}


def cases():
    out = {}
    for name in ("normal_12", "normal_14.4", "reverse", "jump_24"):
        out[name] = (name, "SMCJ33CA")
    for kind in ("ld", "p1", "p2a"):
        for tvs in TVS:
            out[f"{kind}_{tvs}"] = (kind, tvs)
    return out


def tvs_limit_w(duration_s):
    """The TVS's allowed peak power for a pulse of this length: 1500 W at 1 ms (10/1000 us),
    falling roughly with the square root of the duration (the datasheet's derating curve)."""
    return 1500.0 * (1e-3 / max(duration_s, 1e-6)) ** 0.5


def main():
    cs = cases()
    if "--reuse" in sys.argv:
        data = {name: read_raw(WORK / f"{name}.raw") for name in cs}
    else:
        params = dict(bv24=TVS["SMCJ24CA"][0], rs24=TVS["SMCJ24CA"][1],
                      bv33=TVS["SMCJ33CA"][0], rs33=TVS["SMCJ33CA"][1])
        nets = {}
        for name, (src, tvs) in cs.items():
            wave, ri, tstop, step = SOURCES[src]
            nets[name] = NETLIST.format(name=name, src=wave, ri=ri, tvs=tvs, tstop=tstop, step=step, **params)
        data = run_netlists(WORK, nets, VECTORS)
    rows, ok = [], True

    def judge(case, what, value, limit, passed):
        nonlocal ok
        ok &= bool(passed)
        rows.append((case, what, value, limit, "PASS" if passed else "FAIL"))

    def info(case, what, value):
        rows.append((case, what, value, "reported", ""))

    for name, (src, tvs) in cs.items():
        d = data[name]
        t = d["time"]
        rail, fused = d["v(rail)"], d["v(fused)"]
        i_fuse = d["i(vfuse)"]
        p_tvs = np.abs(d["i(@d.xtvs.da[id])"] * fused)
        judge(name, "rail never above 60 V", f"max {np.max(rail):.1f} V", "<= 60 V", np.max(rail) <= 60)
        judge(name, "rail never below -1 V", f"min {np.min(rail):.2f} V", ">= -1 V", np.min(rail) >= -1)
        if name.startswith("normal") or name == "jump_24":
            s = window(d, t[-1] - 5e-3, t[-1])
            i = float(np.mean(s["i(vfuse)"]))
            vd = float(np.mean(s["v(fused)"] - s["v(rail)"]))
            info(name, "rail / fuse current", f"{np.mean(s['v(rail)']):.2f} V / {i:.2f} A")
            info(name, "D11 drop / dissipation", f"{vd:.2f} V / {vd * i:.2f} W")
            judge(name, "fuse current within its 3 A", f"{i:.2f} A", "<= 2.4 A (80 %)", i <= 2.4)
        if name == "reverse":
            s = window(d, t[-1] - 5e-3, t[-1])
            info(name, "current drawn from the reversed battery", f"{abs(np.mean(s['i(vfuse)'])) * 1000:.1f} mA")
        # TVS: energy and peak power against its derated rating
        e = float(np.trapezoid(p_tvs, t))
        conducting = p_tvs > 0.05 * max(np.max(p_tvs), 1e-9)
        dur = float(np.sum(np.diff(t)[conducting[1:]])) if conducting.any() else 0.0
        if np.max(p_tvs) > 1:
            limit = tvs_limit_w(dur)
            judge(name, f"TVS ({tvs}) peak power vs its rating for {dur * 1e3:.1f} ms",
                  f"{np.max(p_tvs):.0f} W", f"<= {limit:.0f} W", np.max(p_tvs) <= limit)
            info(name, "TVS energy", f"{e:.2f} J")
        else:
            info(name, f"TVS ({tvs})", "not conducting")
        if not name.startswith("normal") and name != "jump_24":
            i2t = float(np.trapezoid(i_fuse ** 2, t))
            info(name, "fuse I^2t (a 3 A blade fuse melts at a few A^2s)", f"{i2t:.2f} A^2s")

    plots(data)
    notes = ["Models from datasheet figures (no vendor SPICE models): see `input.py`. The converter and "
             "the LiDAR are constant-power loads (6 W, 8 W), all five relay coils on.", "",
             "Suppressed load dump (ISO 16750-2 test B): the car's own suppression holds the supply at "
             "35 V for up to 400 ms. A TVS that starts conducting below 35 V tries to hold the whole car's "
             "supply down through its fuse: far beyond its rating. One that starts above 35 V lets the "
             "35 V through to the 60 V-rated converter and clips only the fast spikes."]
    (HERE / "results.md").write_text(results_table(
        "12 V input protection simulation results", "Plots: `input_results.png`.", rows, ok, notes))
    print("\n".join(f"{r[4] or 'info'} {r[0]}: {r[1]} = {r[2]} ({r[3]})" for r in rows))
    print("ALL PASS" if ok else "SOME FAIL")
    sys.exit(0 if ok else 1)


def plots(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(9, 10))
    for tvs, colour in (("SMCJ24CA", "tab:red"), ("SMCJ33CA", "tab:green")):
        d = data[f"ld_{tvs}"]
        ax[0].plot(d["time"] * 1e3, d["v(rail)"], color=colour, label=f"rail, {tvs}")
    ax[0].plot(data["ld_SMCJ24CA"]["time"] * 1e3, data["ld_SMCJ24CA"]["v(src)"], color="grey", ls="--",
               label="car supply (35 V suppressed load dump)")
    ax[0].axhline(60, color="black", lw=0.8)
    ax[0].set(title="Suppressed load dump (ISO 16750-2 B): 35 V for 400 ms", xlabel="ms", ylabel="V")
    ax[0].legend(fontsize=8)
    for tvs, colour in (("SMCJ24CA", "tab:red"), ("SMCJ33CA", "tab:green")):
        d = window(data[f"p2a_{tvs}"], 4.99e-3, 5.3e-3)
        ax[1].plot((d["time"] - 5e-3) * 1e6, d["v(fused)"], color=colour, label=f"after the fuse, {tvs}")
        ax[1].plot((d["time"] - 5e-3) * 1e6, d["v(rail)"], color=colour, ls=":", label=f"rail, {tvs}")
    ax[1].axhline(60, color="black", lw=0.8)
    ax[1].set(title="ISO 7637-2 pulse 2a: +112 V, 2 Ohm, 50 us", xlabel="us", ylabel="V")
    ax[1].legend(fontsize=8)
    d = data["reverse"]
    ax[2].plot(d["time"] * 1e3, d["v(src)"], color="grey", ls="--", label="battery (reversed)")
    ax[2].plot(d["time"] * 1e3, d["v(rail)"], label="rail")
    ax[2].set(title="Reversed battery: D11 blocks", xlabel="ms", ylabel="V")
    ax[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "input_results.png", dpi=110)


if __name__ == "__main__":
    main()
