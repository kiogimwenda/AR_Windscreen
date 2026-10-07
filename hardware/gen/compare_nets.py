#!/usr/bin/env python3
"""Proves a redrawn schematic is the same circuit: compares two exported KiCad netlists.

    python3 hardware/gen/compare_nets.py <reference.net> <drawn.net>

The reference is the label-only drawing (kisch.py), the other the hand-arranged, wired one
(kisheet.py). Same components with the same values and footprints, and every net joining exactly
the same pins under the same name (a sheet path prefix like "/actuator/" is ignored). Exit status 0
only if they are identical.
"""

import sys

from check_nets import read_netlist


def load(path):
    comps, nets, _ = read_netlist(path)
    return comps, {frozenset((r, p) for r, p, _ in nodes): name.split("/")[-1]
                   for name, nodes in nets.items()}


def main(ref, drawn):
    ca, a = load(ref)
    cb, b = load(drawn)
    problems = []
    for r in sorted(set(ca) ^ set(cb)):
        problems.append(f"component {r} only in {'reference' if r in ca else 'drawing'}")
    for r in sorted(set(ca) & set(cb)):
        if ca[r] != cb[r]:
            problems.append(f"component {r}: {ca[r]} vs {cb[r]}")
    for pins, name in a.items():
        if pins not in b:
            problems.append(f"net {name} joins different pins: {sorted(pins)}")
        elif b[pins] != name:
            problems.append(f"net {name} is named {b[pins]} in the drawing")
    for pins, name in b.items():
        if pins not in a:
            problems.append(f"drawing has a net {name} not in the reference: {sorted(pins)}")
    for p in problems:
        print("FAIL " + p)
    print(f"{'PASS' if not problems else 'FAIL'} drawn schematic = described circuit: "
          f"{len(cb)} components, {len(b)} nets, {len(problems)} differences")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
