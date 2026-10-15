# Circuit simulations

SPICE simulations of the boards' circuits, run in the ngspice that ships with KiCad 10 (through
KiCad's own Python, so nothing has to be installed), with the boards' exact component values and
the manufacturers' models.

| Script | Circuit | Results |
|---|---|---|
| `run_buck.sh` | power board 5 V converter (TPS54360) with TI's transient model: start-up, load step (stability), 9–30 V input | `buck/results.md`, `buck/buck_results.png` |

- Vendor models are downloaded on first use into `models/` and **not committed** (their makers'
  copyright).
- `ngspice_run.py` runs one netlist (PSpice compatibility on, XSPICE code models loaded).
- Each case is judged against stated limits; a script fails if any check fails.
- `buck.py --reuse` analyses the last run again without simulating (the run takes from minutes to
  ~45 min: one case, 12 V, is slow for the solver).

These check the circuit **design** (values, loop stability, start-up), not the layout: track
inductance, switch-node ringing and noise need the real board (bring-up, `docs/assembly/`).
