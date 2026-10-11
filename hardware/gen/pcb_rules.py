"""PCB design rules and net classes for both boards: what layout.py writes into each .kicad_pro, and
what hardware/LAYOUT_GUIDE.md explains. Pure Python (no KiCad needed).

Fabrication: rules within the standard (cheapest) process of the common budget fabs (JLCPCB,
PCBWay) with margin, so a first board is never rejected or built at the limit: 0.2 mm track and
gap on 1 oz copper (the process allows 0.127 mm), 0.25 mm on 2 oz (thicker copper etches wider),
0.3 mm minimum drill, 0.6 mm vias, 0.5 mm copper-to-edge.

Track widths for current: IPC-2221's external-layer chart, I = k * dT^0.44 * A^0.725 (k = 0.048,
A the cross-section in square mils, dT the temperature rise in C), for a 10 C rise, rounded up.
"""

OZ_MIL = {1: 1.378, 2: 2.756}  # copper thickness, mils


def ipc2221_width_mm(amps, oz, rise_c=10.0, external=True):
    k = 0.048 if external else 0.024
    area = (amps / (k * rise_c ** 0.44)) ** (1 / 0.725)  # square mils
    return area / OZ_MIL[oz] * 0.0254


# Net classes: (name, track mm, clearance mm, via (dia, drill), current A it carries, patterns)
NET_CLASSES = {
    "pod_board": {
        "copper_oz": 1,
        "default": dict(track=0.2, clearance=0.2, via=(0.6, 0.3)),
        "classes": [
            ("Power", 0.4, 0.2, (0.6, 0.3), 0.6, ["VIN_POD", "VBUS", "X_5V_CABLE"]),
            # +3V3 is the In2 plane: each pad reaches it by its own via, the MCU's supply pins by
            # a short track to their decoupling capacitor (placed before routing; the router
            # joins anything left to the plane with a via of its own). 0.3 mm, not the Power
            # class's 0.4: these ties leave 0.5 mm-pitch pins; 0.15 mm carries the 0.6 A.
            ("Plane3V3", 0.3, 0.2, (0.6, 0.3), 0.6, ["+3V3"]),
            ("USB", 0.2, 0.2, (0.6, 0.3), 0.1, ["USB_DP", "USB_DM"]),
            # Ground is the whole In1 layer: every pad reaches it by its own via (a through-hole
            # pin directly); the autorouter leaves it alone. 0.25 mm: the stub from an MCU pin
            # (0.5 mm pitch) to its via must pass between the neighbouring pins.
            ("Ground", 0.25, 0.2, (0.6, 0.3), 0.5, ["GND"]),
        ],
    },
    "power_board": {
        "copper_oz": 2,
        "default": dict(track=0.3, clearance=0.25, via=(0.8, 0.4)),
        "classes": [
            # The actuator rail: battery -> 10 A fuse -> kill relay -> J5, then on the board to the
            # BTS7960 (J6) and the magnet (J7). Sized for the fuse, not the typical draw.
            ("Actuator", 4.0, 0.3, (1.0, 0.5), 10.0, ["ACT_12V"]),
            ("Supply", 1.0, 0.25, (0.8, 0.4), 3.0,
             ["ACC_IN", "ACC_FUSED", "+12V", "LIDAR_12V", "+5V", "BUCK_SW", "MAG_NEG"]),
            # the pod's 5 V after its 500 mA polyfuse (F2): it can never carry more
            ("Pod5V", 0.5, 0.25, (0.8, 0.4), 0.5, ["5V_POD"]),
            ("Coil", 0.5, 0.25, (0.8, 0.4), 0.3, ["COIL_*"]),
            # Ground is the whole bottom layer and a top pour: every pad reaches it directly (a
            # through-hole pin) or by its own via beside it; the autorouter leaves it alone.
            ("Ground", 0.5, 0.25, (0.8, 0.4), 1.0, ["GND"]),
        ],
    },
}

FAB_RULES = {
    "pod_board": dict(min_clearance=0.15, min_track_width=0.15, min_via_diameter=0.5,
                      min_through_hole_diameter=0.3, min_copper_edge_clearance=0.5,
                      min_hole_clearance=0.25, min_hole_to_hole=0.25, min_via_annular_width=0.1,
                      min_silk_clearance=0.0, min_text_height=0.8,
                      # as on the power board: a through-hole pin joined to its plane on one layer
                      # by a single spoke is joined (the default asks for 2 on every layer)
                      min_resolved_spokes=1),
    "power_board": dict(min_clearance=0.2, min_track_width=0.2, min_via_diameter=0.6,
                        min_through_hole_diameter=0.3, min_copper_edge_clearance=0.5,
                        min_hole_clearance=0.25, min_hole_to_hole=0.25, min_via_annular_width=0.13,
                        min_silk_clearance=0.0, min_text_height=0.8,
                        # A through-hole GND pin is joined to the ground on both layers; where a
                        # bottom track cuts one of its bottom spokes, the top pour still holds it.
                        # KiCad's default asks for 2 spokes on each layer.
                        min_resolved_spokes=1),
}

TRACK_CHOICES = {"pod_board": [0.2, 0.25, 0.3, 0.4, 0.6, 1.0],
                 "power_board": [0.3, 0.5, 0.8, 1.0, 1.5, 2.5, 4.0]}


def check_widths():
    """Every class at least as wide as IPC-2221 asks for its current (raises otherwise)."""
    rows = []
    for board, spec in NET_CLASSES.items():
        for name, track, _, _, amps, _ in spec["classes"]:
            need = ipc2221_width_mm(amps, spec["copper_oz"])
            if track + 1e-9 < need:
                raise ValueError(f"{board} {name}: {track} mm < {need:.2f} mm for {amps} A")
            rows.append((board, name, amps, spec["copper_oz"], round(need, 2), track))
    return rows


if __name__ == "__main__":
    for r in check_widths():
        print("%-12s %-9s %5.1f A  %d oz  IPC-2221 needs %.2f mm  -> %.2f mm" % r)
