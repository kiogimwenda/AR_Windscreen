# Circuit simulations

SPICE simulations of the boards' circuits, run in the ngspice that ships with KiCad 10 (through
KiCad's own Python, so nothing has to be installed), with the boards' exact component values and
the manufacturers' models.

| Script | Circuit | Results |
|---|---|---|
| `run_buck.sh` | power board 5 V converter (TPS54360) with TI's transient model: start-up, load step (stability), 9–30 V input | `buck/results.md`, `buck/buck_results.png` |
| `run_input.sh` | power board 12 V input protection (3 A fuse, SMCJ33CA TVS, B560C reverse diode, C20) against ISO 16750-2 / ISO 7637-2 disturbances: normal, reversed battery, jump-start, suppressed load dump, pulses 1 and 2a; the SMCJ24CA first chosen compared | `input/results.md`, `input/input_results.png` |
| `run_relay.sh` | power board relay and magnet drivers (AO3400A + SS14 flyback), driven from the pod through the cable: switching at 9–14.4 V, turn-off spike, release time, pod unplugged | `relay/results.md`, `relay/relay_results.png` |

- Vendor models are downloaded on first use into `models/` and **not committed** (their makers'
  copyright).
- `ngspice_run.py` runs one netlist (PSpice compatibility on, XSPICE code models loaded);
  `simlib.py` holds what the scripts share (running cases in parallel, reading results).
- Where a maker publishes no SPICE model (AO3400A, SS14, the TVS diodes), the model is built from the
  datasheet's figures and says so; coils of parts not yet bought use typical values.
- Each case is judged against stated limits; a script fails if any check fails.
- `--reuse` (every script) analyses the last run again without simulating (the converter run takes up to
  ~45 min: one case, 12 V, is slow for the solver; the relay run takes seconds).

These check the circuit **design** (values, loop stability, start-up), not the layout: track
inductance, switch-node ringing and noise need the real board (bring-up, `docs/assembly/`).
