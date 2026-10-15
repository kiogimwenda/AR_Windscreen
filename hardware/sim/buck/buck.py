"""Simulates the power board's 5 V converter (TPS54360, BUILD_GUIDE 4.8.4) with TI's own transient
model and the board's exact component values (hardware/gen/power_board.py), in the ngspice that
ships with KiCad. Run through hardware/sim/run_buck.sh.

Cases, each judged against a stated limit:
  startup   12 V applied (100 us ramp), 0.5 A load: reaches 4.75 V, overshoot <= 3 %
  step      12 V, load 0.2 -> 1.5 -> 0.2 A in 1 us: deviation <= 5 % (250 mV), back within 1 %
            in <= 300 us, and settles without ringing (a stability check: an unstable or poorly
            compensated loop rings or oscillates here)
  vin_*     9, 12, 14.4 and 30 V input at 1 A: output 5.0 V +-2 %, ripple <= 50 mV peak-to-peak,
            switching 600 kHz +-10 %; efficiency reported
Modelling assumptions (stated in the results): the 47 uF output ceramics at 25 uF each (DC-bias
derating at 5 V, the cautious case), 3 mOhm ESR; the input ceramics 2.2 uF; L1 8.2 uH with
17 mOhm DCR (SRR1260-8R2Y); D13 a generic 5 A / 60 V Schottky model (no vendor model in SPICE
form); the supply 20 mOhm. The 12 V input's own protection (fuse, TVS, reverse diode) is not
part of these cases.
"""

import io
import os
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
ROOT = SIM.parent.parent
MODEL_DIR = SIM / "models"
MODEL = MODEL_DIR / "TPS54360_TRANS.LIB"
MODEL_URL = "https://www.ti.com/lit/zip/SLVMCT7"  # TI: "TPS54360 Unencrypted PSpice Transient Model"
WORK = ROOT / "build" / "sim" / "buck"
KICAD_PY = os.environ.get("KICAD_PY", "/mnt/c/Program Files/KiCad/10.0/bin/python.exe")
VECTORS = ["v(vout)", "v(sw)", "v(vsrc)", "vsup#branch", "v(comp)", "@iload[current]"]

NETLIST = """* AR Windscreen power board: 5 V converter (TPS54360), case {name}
.include "{model}"
VSUP VSRC 0 {vin}
RSRC VSRC VIN 20m
XC21 VIN 0 CESR PARAMS: C=2.2u ESR=5m X=1
XC22 VIN 0 CESR PARAMS: C=2.2u ESR=5m X=1
R41 VIN EN 523k
R42 EN 0 84.5k
R43 RT 0 162k
XU1 BOOT COMP EN 0 SW RT VIN FB 0 TPS54360_TRANS PARAMS: STEADY_STATE={ss}
C23 BOOT SW 100n
D13 0 SW B560C
XL1 SW VOUT LDCR PARAMS: L=8.2u DCR=17m IC={il0}
XC24 VOUT 0 CESR PARAMS: C=25u ESR=3m X=1 IC={v0}
XC25 VOUT 0 CESR PARAMS: C=25u ESR=3m X=1 IC={v0}
R44 VOUT FB 53.6k
R45 FB 0 10.2k
R46 COMP RC 13.0k
C26 RC 0 6800p
C27 COMP 0 39p
ILOAD VOUT 0 {iload}
RMIN VOUT 0 1k
.model B560C D(IS=5u N=1.05 RS=15m CJO=500p M=0.45 VJ=0.35 BV=60 IBV=0.5m TT=5n)
.tran 10n {tstop} 0 20n {uic}
.end
"""

# name: (supply, load current, steady-state start?, stop time)
CASES = {
    "startup": ("PWL(0 0 100u 12)", "PWL(0 0 300u 0 400u 0.5)", 0, "4m"),
    "step": ("12", "PWL(0 0.2 600u 0.2 601u 1.5 1100u 1.5 1101u 0.2)", 1, "1.6m"),
    "vin_9": ("9", "1.0", 1, "0.8m"),
    "vin_12": ("12", "1.0", 1, "0.8m"),
    "vin_14.4": ("14.4", "1.0", 1, "0.8m"),
    "vin_30": ("30", "1.0", 1, "0.8m"),
}


def win(p):
    return subprocess.run(["wslpath", "-w", str(p)], capture_output=True, text=True).stdout.strip()


def fetch_model():
    if MODEL.exists():
        return
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request(MODEL_URL, headers={"User-Agent": "Mozilla/5.0"})
    data = urllib.request.urlopen(req, timeout=60).read()
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        name = next(n for n in z.namelist() if n.upper().endswith(".LIB"))
        MODEL.write_bytes(z.read(name).replace(b"\r\n", b"\n"))
    print(f"TI model downloaded: {MODEL} (not committed: TI's copyright)")


def read_raw(path):
    """ngspice ASCII raw file -> {vector name: numpy array}."""
    text = Path(path).read_text(errors="replace").splitlines()
    names, i = [], 0
    while not text[i].startswith("Variables:"):
        if text[i].startswith("No. Points:"):
            npts = int(text[i].split(":")[1])
        i += 1
    i += 1
    while not text[i].startswith("Values:"):
        parts = text[i].split()
        names.append(parts[1].lower())
        i += 1
    vals = []
    for line in text[i + 1:]:
        line = line.strip()
        if line:
            vals.append(float(line.split()[-1]))
    n = len(names)
    data = np.array(vals[: (len(vals) // n) * n]).reshape(-1, n)
    return {name: data[:, k] for k, name in enumerate(names)}


def run_all():
    WORK.mkdir(parents=True, exist_ok=True)
    procs = {}
    for name, (vin, iload, ss, tstop) in CASES.items():
        v0, il0 = ("5", "0.5") if ss else ("0", "0")
        cir = WORK / f"{name}.cir"
        cir.write_text(NETLIST.format(name=name, model=win(MODEL).replace("\\", "/"), vin=vin, ss=ss,
                                      il0=il0, v0=v0, iload=iload, tstop=tstop,
                                      uic="UIC" if ss else ""))
        procs[name] = subprocess.Popen(
            [KICAD_PY, win(SIM / "ngspice_run.py"), win(cir), win(WORK / f"{name}.raw"),
             win(WORK / f"{name}.log")] + VECTORS, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for name, p in procs.items():
        out = p.communicate()[0]
        if p.returncode:
            for q in procs.values():  # do not leave the other cases running
                if q.poll() is None:
                    q.kill()
            sys.exit(f"{name}: ngspice failed\n{out}\nsee {WORK / (name + '.log')}")
    return {name: read_raw(WORK / f"{name}.raw") for name in CASES}


def window(d, t0, t1):
    m = (d["time"] >= t0) & (d["time"] <= t1)
    return {k: v[m] for k, v in d.items()}


def steady(d, t0, t1, vin):
    w = window(d, t0, t1)
    vout, sw = w["v(vout)"], w["v(sw)"]
    t = w["time"]
    rising = np.where((sw[:-1] < vin / 2) & (sw[1:] >= vin / 2))[0]
    fsw = (len(rising) - 1) / (t[rising[-1]] - t[rising[0]]) if len(rising) > 2 else 0.0
    pin = np.trapezoid(w["v(vsrc)"] * -w["i(vsup)"], t)
    pout = np.trapezoid(vout * (w["i(@iload[current])"] + vout / 1000), t)
    return {"vout": float(np.mean(vout)), "ripple": float(np.ptp(vout)), "fsw": float(fsw),
            "eff": float(pout / pin) if pin > 0 else 0.0}


def main():
    fetch_model()
    if "--reuse" in sys.argv:  # analyse the last run's results again, without simulating
        data = {name: read_raw(WORK / f"{name}.raw") for name in CASES}
    else:
        data = run_all()
    rows, ok = [], True

    def judge(case, what, value, limit, passed):
        nonlocal ok
        ok &= passed
        rows.append((case, what, value, limit, "PASS" if passed else "FAIL"))

    # start-up
    d = data["startup"]
    t, v = d["time"], d["v(vout)"]
    reached = np.where(v >= 4.75)[0]
    judge("startup", "reaches 4.75 V", f"{t[reached[0]] * 1e3:.2f} ms" if len(reached) else "never",
          "yes", bool(len(reached)))
    final = float(np.mean(v[t > float(t[-1]) - 0.3e-3]))
    over = (float(np.max(v)) - final) / final * 100
    judge("startup", "overshoot", f"{over:.1f} %", "<= 3 %", over <= 3)

    # load step
    d = data["step"]
    t, v = d["time"], d["v(vout)"]
    pre = float(np.mean(v[(t > 0.4e-3) & (t < 0.6e-3)]))
    for label, t_step, t_end, sign in (("0.2 -> 1.5 A", 0.6e-3, 1.1e-3, -1), ("1.5 -> 0.2 A", 1.1e-3, 1.6e-3, 1)):
        w = (t >= t_step) & (t < t_end)
        dev = float(np.max(np.abs(v[w] - pre)))
        settle_ok = np.abs(v[w] - pre) <= 0.01 * pre
        tw = t[w]
        last_bad = tw[~settle_ok][-1] if (~settle_ok).any() else t_step
        settle_us = (last_bad - t_step) * 1e6
        # ringing: how often the error changes sign by more than 0.3 % after the first peak
        err = v[w] - pre
        big = err[np.abs(err) > 0.003 * pre]
        crossings = int(np.sum(np.diff(np.sign(big)) != 0))
        judge("step", f"{label}: deviation", f"{dev * 1000:.0f} mV ({dev / pre * 100:.1f} %)", "<= 5 %",
              dev <= 0.05 * pre)
        judge("step", f"{label}: back within 1 %", f"{settle_us:.0f} us", "<= 300 us", settle_us <= 300)
        judge("step", f"{label}: ringing (sign changes > 0.3 %)", str(crossings), "<= 2", crossings <= 2)

    # input voltages
    for name in [n for n in CASES if n.startswith("vin_")]:
        vin = float(name.split("_")[1])
        s = steady(data[name], 0.5e-3, 0.8e-3, vin)
        judge(name, "output", f"{s['vout']:.3f} V", "4.90-5.10 V", 4.90 <= s["vout"] <= 5.10)
        judge(name, "ripple", f"{s['ripple'] * 1000:.1f} mV", "<= 50 mV", s["ripple"] <= 0.050)
        judge(name, "switching", f"{s['fsw'] / 1e3:.0f} kHz", "540-660 kHz", 540e3 <= s["fsw"] <= 660e3)
        rows.append((name, "efficiency at 1 A", f"{s['eff'] * 100:.0f} %", "reported", ""))

    plots(data)
    write_results(rows, ok)
    print("\n".join(f"{'' if r[4] == 'PASS' else r[4] or 'info'} {r[0]}: {r[1]} = {r[2]} ({r[3]})" for r in rows))
    print("ALL PASS" if ok else "SOME FAIL")
    sys.exit(0 if ok else 1)


def plots(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(9, 10))
    d = data["startup"]
    ax[0].plot(d["time"] * 1e3, d["v(vout)"], label="Vout")
    ax[0].plot(d["time"] * 1e3, d["v(vsrc)"], label="Vin", alpha=0.6)
    ax[0].set(title="Start-up: 12 V applied, 0.5 A load", xlabel="ms", ylabel="V")
    ax[0].legend()
    d = window(data["step"], 0.3e-3, 1.6e-3)  # the first 0.3 ms settle from the preset start
    ax[1].plot(d["time"] * 1e3, d["v(vout)"], label="Vout")
    a2 = ax[1].twinx()
    a2.plot(d["time"] * 1e3, d["i(@iload[current])"], color="tab:red", alpha=0.5, label="load")
    a2.set_ylabel("load, A")
    ax[1].set(title="Load step 0.2 -> 1.5 -> 0.2 A at 12 V (stability)", xlabel="ms", ylabel="Vout, V")
    d = window(data["vin_12"], 0.75e-3, 0.76e-3)
    ax[2].plot(d["time"] * 1e6, d["v(sw)"], label="SW node")
    a3 = ax[2].twinx()
    a3.plot(d["time"] * 1e6, d["v(vout)"] * 1000, color="tab:green", label="Vout")
    a3.set_ylabel("Vout, mV")
    ax[2].set(title="Steady state at 12 V, 1 A: switch node and output ripple", xlabel="us", ylabel="SW, V")
    fig.tight_layout()
    fig.savefig(HERE / "buck_results.png", dpi=110)


def write_results(rows, ok):
    lines = ["<!-- Generated by hardware/sim/run_buck.sh: do not edit. -->\n",
             "# 5 V converter (TPS54360) simulation results\n",
             f"Overall: **{'ALL PASS' if ok else 'SOME FAIL'}**. Plots: `buck_results.png`. "
             "Model: TI's TPS54360 transient model (Final 1.10) with the power board's component values.\n",
             "| Case | Check | Result | Limit | |", "|---|---|---|---|---|"]
    lines += [f"| {c} | {w} | {v} | {lim} | {res} |" for c, w, v, lim, res in rows]
    lines += ["", "Assumptions: output ceramics 2 x 25 uF (47 uF derated at 5 V), 3 mOhm ESR; input "
              "2 x 2.2 uF; L1 8.2 uH, 17 mOhm; D13 a generic 5 A / 60 V Schottky; supply 20 mOhm. "
              "See the docstring of `buck.py`.", "",
              "What this does and does not show: it checks the converter's DESIGN (the values: output "
              "voltage, switching frequency, compensation, start-up) with TI's model. It does not "
              "include the board's layout (track inductance, switch-node ringing, noise), the real "
              "diode's forward voltage (the efficiency is somewhat optimistic for that reason), or "
              "temperature. The load-step and steady cases start from a preset state and settle for "
              "their first 0.3 ms; the measurements skip it."]
    (HERE / "results.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
