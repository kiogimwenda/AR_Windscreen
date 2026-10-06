#!/usr/bin/env python3
"""Draws the proposal's diagrams as SVG, in the style of the FYP concept paper's figures:
black line work, rounded boxes, a bold title over a regular subtitle, layered containers,
flowcharts with decision diamonds. Text is measured with the real Liberation Serif metrics
(the typeface of the document), so every box fits its words.

    python3 docs/report/proposal/src/make_figures.py      # writes docs/report/proposal/figures/*.svg
"""

import math
from pathlib import Path

from PIL import ImageFont

OUT = Path(__file__).resolve().parent.parent / "figures"
FONT_DIR = Path("/usr/share/fonts/truetype/liberation")
_fonts = {}


def font(size, bold=False, italic=False):
    key = (size, bold, italic)
    if key not in _fonts:
        name = "LiberationSerif-" + ("BoldItalic" if bold and italic else "Bold" if bold else "Italic" if italic else "Regular")
        _fonts[key] = ImageFont.truetype(str(FONT_DIR / f"{name}.ttf"), size)
    return _fonts[key]


def text_width(s, size, bold=False, italic=False):
    return font(size, bold, italic).getlength(s)


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Svg:
    def __init__(self, w, h):
        self.w, self.h, self.parts = w, h, []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, size=12, bold=False, italic=False, anchor="middle"):
        w = ' font-weight="bold"' if bold else ""
        st = ' font-style="italic"' if italic else ""
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}"{w}{st} text-anchor="{anchor}">{esc(s)}</text>')

    def lines(self, cx, cy, rows, size=12, gap=1.25, bold_first=False):
        """Centred lines of text, vertically centred on cy. rows: list of str."""
        n = len(rows)
        top = cy - (n - 1) * size * gap / 2 + size * 0.35
        for i, r in enumerate(rows):
            self.text(cx, top + i * size * gap, r, size, bold=bold_first and i == 0)

    def box(self, x, y, w, h, title=None, sub=None, rows=None, size=12, stroke=1.4, rx=6, fill="none", dashed=False):
        dash = ' stroke-dasharray="5 3"' if dashed else ""
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}" stroke="#000" stroke-width="{stroke}"{dash}/>')
        cx, cy = x + w / 2, y + h / 2
        if rows:
            self.lines(cx, cy, rows, size)
        elif title and sub:
            subs = sub if isinstance(sub, list) else [sub]
            self.text(cx, cy - 4 - (len(subs) - 1) * 6, title, size + 1, bold=True)
            for i, s in enumerate(subs):
                self.text(cx, cy + 12 + i * 13 - (len(subs) - 1) * 6, s, size - 1)
        elif title:
            self.text(cx, cy + size * 0.35, title, size + 1, bold=True)

    def container(self, x, y, w, h, label, size=13):
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="7" fill="none" stroke="#000" stroke-width="2"/>')
        self.text(x + 14, y + 22, label, size, bold=True, anchor="start")

    def diamond(self, cx, cy, w, h, rows, size=12):
        pts = f"{cx},{cy - h / 2} {cx + w / 2},{cy} {cx},{cy + h / 2} {cx - w / 2},{cy}"
        self.add(f'<polygon points="{pts}" fill="none" stroke="#000" stroke-width="1.4"/>')
        self.lines(cx, cy, rows, size)

    def arrow(self, pts, label=None, lx=None, ly=None, anchor="start", dashed=False, head=True, both=False):
        d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        dash = ' stroke-dasharray="5 3"' if dashed else ""
        mk = ' marker-end="url(#ah)"' if head else ""
        mks = ' marker-start="url(#ahs)"' if both else ""
        self.add(f'<path d="{d}" fill="none" stroke="#000" stroke-width="1.3"{dash}{mk}{mks}/>')
        if label:
            if lx is None:
                (x0, y0), (x1, y1) = pts[0], pts[1]
                lx, ly = (x0 + x1) / 2 + 7, (y0 + y1) / 2 + 4
            self.text(lx, ly, label, 11, italic=True, anchor=anchor)

    def save(self, name):
        defs = ('<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
                'orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#000"/></marker>'
                '<marker id="ahs" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="8" markerHeight="8" '
                'orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#000"/></marker>'
                '<pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
                '<line x1="0" y1="0" x2="0" y2="6" stroke="#000" stroke-width="1"/></pattern></defs>')
        body = "\n".join(self.parts)
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" '
               f'font-family="Liberation Serif, Times New Roman, serif" fill="#000">{defs}\n{body}\n</svg>\n')
        (OUT / name).write_text(svg)
        print("wrote", name)


# ------------------------------------------------------------------------------------------------
def fig_architecture():
    s = Svg(700, 760)
    W, X = 660, 20
    # Layer 1: sensors
    s.container(X, 10, W, 120, "Vehicle sensors")
    items = [("Camera", "2K, 30 fps, USB"), ("LiDAR", "Livox Mid-360"), ("GNSS", "u-blox NEO-M8N"),
             ("IMU", "BNO085"), ("OBD-II", "vehicle CAN bus"), ("Gesture", "APDS-9960")]
    bw, gap = 98, 8.4
    for i, (t, sub) in enumerate(items):
        s.box(X + 14 + i * (bw + gap), 42, bw, 72, t, sub, size=12)
    # Layer 2: hub
    s.container(X, 200, W, 120, "Sensor and actuator hub: windscreen pod (STM32F405, FreeRTOS) + under-dash power box")
    hub = [("Sensor task", "50 Hz SensorReport"), ("Comms task", "framed, CRC-16"),
           ("Actuation task", "10 ms, clamps"), ("Watchdog task", "200 ms release")]
    bw, gap = 148, 8.7
    for i, (t, sub) in enumerate(hub):
        s.box(X + 14 + i * (bw + gap), 232, bw, 72, t, sub)
    # arrows sensors -> hub (GNSS, IMU, OBD, gesture) and camera/LiDAR -> host (bypass on the left)
    for i in (2, 3, 4, 5):
        cx = X + 14 + i * (98 + 8.4) + 49
        s.arrow([(cx, 114), (cx, 200)])
    s.text(X + 350, 160, "UART / I2C", 11, italic=True, anchor="start")
    # Layer 3: host
    s.container(X, 400, W, 200, "Host laptop  (RTX 5060, WSL2 Debian, C++17, CUDA/TensorRT)")
    host = [("Perception", ["TensorRT: YOLOv8m-seg,", "signs, UFLDv2, MiDaS"]),
            ("Fusion and tracking", ["EKF pose, mask-LiDAR", "fusion, IMM tracker"]),
            ("Navigation", ["OSRM routing, map", "matching, road projector"]),
            ("Decision", ["hazard classifier,", "decision arbiter"]),
            ("AR renderer", ["OpenGL overlays", "on the GPU"]),
            ("System", ["message bus, event log,", "system manager"])]
    bw, bh = 200, 70
    for i, (t, sub) in enumerate(host):
        r, c = divmod(i, 3)
        s.box(X + 16 + c * (bw + 14), 432 + r * (bh + 14), bw, bh, t, sub)
    # camera / LiDAR bypass to host
    for i, lab in ((0, "USB video"), (1, "Ethernet")):
        cx = X + 14 + i * (98 + 8.4) + 49
        s.arrow([(cx, 114), (cx, 400)])
        s.text(cx + 6, 380 - i * 0, lab, 11, italic=True, anchor="start")
    s.arrow([(X + W / 2 + 60, 320), (X + W / 2 + 60, 400)], both=True)
    s.text(X + W / 2 + 70, 352, "USB serial: SensorReport 50 Hz up;", 11, italic=True, anchor="start")
    s.text(X + W / 2 + 70, 366, "heartbeat 10 Hz, commands down", 11, italic=True, anchor="start")
    # Layer 4: outputs
    s.container(X, 660, W, 90, "Outputs")
    outs = [("AR display", "laptop screen"), ("Signal relays", "indicators, hazards, horn"),
            ("Brake actuator", "force-limited, pull-only"), ("Evidence log", "every request")]
    bw, gap = 148, 8.7
    for i, (t, sub) in enumerate(outs):
        s.box(X + 14 + i * (bw + gap), 690, bw, 50, t, sub, size=11)
    s.arrow([(X + 120, 600), (X + 120, 660)])
    s.arrow([(X + 560, 320), (X + 560, 340), (X + 672, 340), (X + 672, 630), (X + 360, 630), (X + 360, 690)])
    s.arrow([(X + 360, 630), (X + 200, 630), (X + 200, 690)])
    s.text(X + 580, 334, "PWM, relay drive", 11, italic=True, anchor="start")
    s.save("fig_architecture.svg")


def fig_hub_hardware():
    s = Svg(720, 560)
    # Windscreen pod
    s.container(10, 8, 700, 132, "Windscreen pod (behind the mirror): pod board, 4-layer")
    pod = [("STM32F405", ["FreeRTOS, 8 MHz", "crystal, USB to laptop"]),
           ("Camera", ["on the carrier plate,", "USB to laptop"]),
           ("BNO085 IMU", ["on the same carrier", "as the camera"]),
           ("NEO-M8N GNSS", ["antenna at the glass;", "status LEDs"])]
    bw = 160
    for i, (t, sub) in enumerate(pod):
        s.box(24 + i * (bw + 10), 40, bw, 84, t, sub, size=11)
    # cable
    s.arrow([(360, 140), (360, 200)], both=True)
    s.text(370, 158, "DB-25 cable, 2 m. Down: brake PWM and enable,", 11, italic=True, anchor="start")
    s.text(370, 172, "magnet, relays, CAN logic, +5 V. Up: current,", 11, italic=True, anchor="start")
    s.text(370, 186, "kill sense, box present, brake light.", 11, italic=True, anchor="start")
    s.text(350, 165, "5 V from the box OR USB", 11, italic=True, anchor="end")
    s.text(350, 179, "powers the pod", 11, italic=True, anchor="end")
    # Power box
    s.container(10, 200, 700, 250, "Power box (under the dash): power board, 2-layer, 2 oz copper")
    rows = [[("Input pull-downs", ["every line OFF when", "the pod is not driving it"]),
             ("74HCT244 buffer", ["3.3 V logic to", "5 V module inputs"]),
             ("12 V input", ["fuses, TVS, reverse", "protection, 60 V buck"])],
            [("BTS7960 + ACS712", ["H-bridge 20 kHz;", "current sense"]),
             ("Magnet MOSFET", ["cable held only", "while braking"]),
             ("40 A kill relay", ["coil through the", "E-stop's NC contact"])],
            [("5 signal relays", ["MOSFET-driven,", "active-high"]),
             ("SN65HVD230", ["CAN transceiver,", "standby by default"]),
             ("Presence MOSFET", ["box connected AND", "powered = present"])]]
    bw = 216
    for r, row in enumerate(rows):
        for c, (t, sub) in enumerate(row):
            s.box(24 + c * (bw + 10), 230 + r * 72, bw, 62, t, sub, size=11)
    # external items
    ext = [("Actuator", "pull-only cable"), ("Cable magnet", "holds the cable"),
           ("E-stop", "driver's reach"), ("Car lights, horn", "existing wiring"),
           ("OBD-II port", "~0.5 m lead")]
    bw = 128
    for i, (t, sub) in enumerate(ext):
        s.box(24 + i * (bw + 10), 488, bw, 56, t, sub, size=11)
    xs = [24 + i * 138 + 64 for i in range(5)]
    s.arrow([(xs[0], 450), (xs[0], 488)])
    s.arrow([(xs[1], 450), (xs[1], 488)])
    s.arrow([(xs[2], 488), (xs[2], 450)])
    s.arrow([(xs[3], 450), (xs[3], 488)])
    s.arrow([(xs[4], 450), (xs[4], 488)], both=True)
    s.save("fig_hub_hardware.svg")


def fig_installation():
    s = Svg(760, 430)
    gy = 360  # ground line
    k = 1 / 0.012  # px per metre
    s.add(f'<line x1="10" y1="{gy}" x2="750" y2="{gy}" stroke="#000" stroke-width="1.5"/>')
    body = [(40, 335), (40, 287), (92, 277), (150, 240), (270, 239), (332, 284), (404, 290),
            (416, 302), (416, 335), (378, 335), (372, 312), (345, 300), (318, 312), (312, 335),
            (133, 335), (127, 312), (100, 300), (73, 312), (67, 335)]
    s.add('<polygon points="' + " ".join(f"{x},{y}" for x, y in body) +
          '" fill="none" stroke="#000" stroke-width="1.6"/>')
    for cx in (100, 345):
        s.add(f'<circle cx="{cx}" cy="{gy - 30}" r="28" fill="none" stroke="#000" stroke-width="1.6"/>')
        s.add(f'<circle cx="{cx}" cy="{gy - 30}" r="9" fill="none" stroke="#000" stroke-width="1"/>')
    # LiDAR on the roof front edge, on a wedge
    lx, ly = 262, 231
    s.add(f'<rect x="{lx - 9}" y="{ly - 8}" width="18" height="10" fill="#000"/>')
    # rays: tilted lower edge -22 deg, level lower edge -7 deg
    import math
    d = (1.55 / math.tan(math.radians(22))) * k
    s.arrow([(lx, ly), (lx + d, gy)], head=False)
    s.add(f'<circle cx="{lx + d:.1f}" cy="{gy}" r="3" fill="#000"/>')
    x_end = 745
    y_end = ly + (x_end - lx) * math.tan(math.radians(7))
    s.arrow([(lx, ly), (x_end, y_end)], head=False, dashed=True)
    s.text(lx + d - 6, gy + 18, "tilted 15°: road visible from about 4–5 m", 11, italic=True, anchor="end")
    s.text(560, y_end - 34, "level mount: lowest beam (−7°)", 11, italic=True, anchor="start")
    s.text(560, y_end - 20, "reaches the road only at 12.6 m", 11, italic=True, anchor="start")
    # callouts: labels in two columns, leaders from the label's edge
    def call(x, y, tx, ty, lines, left=False):
        w = max(text_width(lines[0], 11.5, bold=True), *(text_width(t, 11.5) for t in lines[1:]))
        lx0 = tx if not left else tx
        edge = tx - 4 if not left else tx + w + 4
        s.add(f'<line x1="{x}" y1="{y}" x2="{edge}" y2="{ty}" stroke="#000" stroke-width="0.8"/>')
        s.add(f'<circle cx="{x}" cy="{y}" r="2.5" fill="#000"/>')
        for i, t in enumerate(lines):
            s.text(lx0, ty + 4 + i * 14, t, 11.5, bold=(i == 0), anchor="start")
    call(lx, ly - 8, 330, 30, ["Livox Mid-360", "roof front edge, 15° forward wedge,", "3 mm aluminium plate"])
    call(268, 250, 12, 40, ["Windscreen pod", "behind the mirror: camera,", "IMU, GNSS, STM32F405"], left=True)
    call(300, 283, 12, 105, ["Laptop (AR display)", "on a cabin mount"], left=True)
    call(318, 318, 12, 160, ["Power box", "under the dash, driver's side"], left=True)
    call(300, 300, 520, 95, ["E-stop and gesture puck", "within the driver's reach"])
    call(326, 324, 520, 150, ["OBD-II port and brake pedal", "actuator on a pull-only cable"])
    s.text(20, 410, "Not to scale except the LiDAR geometry (1.55 m mounting height; bonnet limits the near field).",
           11, italic=True, anchor="start")
    s.save("fig_installation.svg")


def fig_software_dataflow():
    s = Svg(720, 560)
    def b(x, y, t, sub, w=150, h=56):
        s.box(x, y, w, h, t, sub, size=11)
    b(20, 20, "Camera pipeline", "frameBus (2K)")
    b(20, 110, "LiDAR processor", "SceneCloud, ground")
    b(20, 200, "Vehicle interface", "hub link, 50 Hz")
    b(20, 290, "Navigation engine", "OSRM route")
    b(210, 20, "ML inference", "4 TensorRT engines")
    b(210, 110, "Scene reconstruction", "mask-LiDAR fusion")
    b(210, 200, "Sensor fusion", "EKF pose, 100 Hz")
    b(210, 290, "Map matcher", "trace -> road, 10 Hz")
    b(400, 110, "Object tracker", "IMM, Hungarian")
    b(400, 200, "Motion predictor", "paths, CPA, risk")
    b(400, 290, "Road surface projector", "lane-locked line")
    b(570, 110, "Hazard classifier", "flags, risk r", w=130)
    b(570, 200, "Decision arbiter", "4 rules", w=130)
    b(570, 290, "AR renderer", "OverlayScene", w=130)
    b(400, 400, "Display sink", "GPU, WSLg window")
    b(570, 400, "Event log", "fsync, evidence", w=130)
    b(20, 400, "System manager", "threads, health")
    A = s.arrow
    A([(170, 48), (210, 48)])
    A([(285, 76), (285, 110)])
    A([(170, 138), (210, 138)])
    A([(360, 138), (400, 138)])
    A([(170, 228), (210, 228)])
    A([(360, 228), (400, 228)])
    A([(475, 166), (475, 200)])
    A([(550, 138), (570, 138)])
    A([(635, 166), (635, 200)])
    A([(550, 228), (570, 228)])
    A([(170, 318), (210, 318)])
    A([(360, 318), (400, 318)])
    A([(550, 318), (570, 318)])
    A([(285, 256), (285, 290)])
    A([(635, 346), (635, 370), (475, 370), (475, 400)])
    A([(700, 228), (712, 228), (712, 428), (700, 428)])
    A([(570, 214), (560, 214), (560, 190), (95, 190), (95, 200)])
    s.text(300, 183, "ActuationRequest (the only producer)", 11, italic=True, anchor="start")
    A([(360, 48), (660, 48), (660, 110)])
    s.text(430, 42, "DetectionFrame (masks, lanes, signs)", 11, italic=True, anchor="start")
    s.text(20, 490, "Each arrow is a lock-free single-producer/single-consumer ring buffer carrying a FlatBuffers", 11, italic=True, anchor="start")
    s.text(20, 506, "message; consumers take the newest item, so a slow stage never makes another fall behind.", 11, italic=True, anchor="start")
    s.save("fig_software_dataflow.svg")


def fig_methodology():
    s = Svg(720, 560)
    cx, bw = 200, 290
    steps = [
        ("Requirements and safety rules", "hard actuation rule, scope, metrics"),
        ("Architecture and interfaces", "protocol, message bus, DisplaySink"),
        ("Phase n: design", "concept first, then interface"),
        ("Implement", "C++17 host / FreeRTOS firmware"),
        ("Test", "unit tests, sanitizers, fuzzing"),
    ]
    y, ys = 14, []
    for t, sub in steps:
        s.box(cx - bw / 2, y, bw, 52, t, sub)
        ys.append(y)
        y += 76
    for i in range(len(ys) - 1):
        s.arrow([(cx, ys[i] + 52), (cx, ys[i + 1])])
    dy = ys[-1] + 52 + 62
    s.diamond(cx, dy, 250, 100, ["Exit criteria", "demonstrably met?"])
    s.arrow([(cx, ys[-1] + 52), (cx, dy - 50)])
    s.arrow([(cx - 125, dy), (cx - 150, dy), (cx - 150, ys[2] + 26), (cx - bw / 2, ys[2] + 26)])
    s.text(cx - 132, dy - 8, "No", 11, italic=True, anchor="end")
    s.text(cx - 155, (dy + ys[2]) / 2 + 20, "revise", 11, italic=True, anchor="end")
    # right column: hardware sequence
    rx = 540
    later = [("Bench-top integration", "hub + host, no vehicle"),
             ("Actuator bench gate", "5 checks, 3 consecutive passes"),
             ("Vehicle installation", "calibration: camera, LiDAR, ground"),
             ("Staged validation", "bench, stationary, field, campus road"),
             ("Demonstration and report", "evidence from the event log")]
    y2 = 14
    prev = None
    for t, sub in later:
        s.box(rx - bw / 2, y2, bw, 52, t, sub)
        if prev is not None:
            s.arrow([(rx, prev), (rx, y2)])
        prev = y2 + 52
        y2 += 76
    s.arrow([(cx + 125, dy), (355, dy), (355, 40), (rx - bw / 2, 40)])
    s.text(349, dy - 34, "Yes: all software", 11, italic=True, anchor="end")
    s.text(349, dy - 20, "phases complete", 11, italic=True, anchor="end")
    s.box(rx - bw / 2 + 10, 425, bw - 10, 120, None, dashed=True)
    for i, r in enumerate(["Every task is logged: a progress log (what,",
                           "why, how verified) and a decision record for",
                           "any choice made under ambiguity. Hardware",
                           "steps follow a stop list; physical work is",
                           "done by the student."]):
        s.text(rx - bw / 2 + 22, 451 + i * 20, r, 12, anchor="start")
    s.save("fig_methodology.svg")


def fig_frame_pipeline():
    s = Svg(700, 700)
    cx, bw = 230, 320
    steps = [("Capture frame", "2560 x 1440 BGR, timestamped"),
             ("Pre-process on the GPU", "letterbox, normalise (CUDA)"),
             ("Four TensorRT engines", "objects+masks, signs, lanes, depth"),
             ("Decode", "boxes, masks, lane points, depth"),
             ("Project LiDAR into the image", "motion-compensated, depth-tested"),
             ("Assign points by eroded mask", "nearest dominant depth cluster"),
             ("Update IMM tracks", "world frame, gated assignment"),
             ("Predict and assess risk", "CPA, TTC, hazard flags")]
    y, ys = 20, []
    for t, sub in steps:
        s.box(cx - bw / 2, y, bw, 56, t, sub)
        ys.append(y)
        y += 80
    for i in range(len(ys) - 1):
        s.arrow([(cx, ys[i] + 56), (cx, ys[i + 1])])
    s.box(430, ys[6], 250, 56, "Sensor fusion (EKF)", "GNSS, IMU, OBD speed: ego pose")
    s.arrow([(430, ys[6] + 28), (cx + bw / 2, ys[6] + 28)])
    s.box(430, ys[4], 250, 56, "LiDAR point cloud", "10 Hz, ground plane fitted")
    s.arrow([(430, ys[4] + 28), (cx + bw / 2, ys[4] + 28)])
    s.box(430, ys[7], 250, 56, "To renderer and arbiter", "hazards, overlays, requests")
    s.arrow([(cx + bw / 2, ys[7] + 28), (430, ys[7] + 28)])
    s.text(20, 680, "Measured: the four models together take a median 17.8 ms per frame (p95 19.9 ms) against a 33 ms budget.", 11, italic=True, anchor="start")
    s.save("fig_frame_pipeline.svg")


def fig_arbiter():
    s = Svg(720, 640)
    cx = 240
    s.box(cx - 150, 10, 300, 50, "Every cycle", "tracks, ego state, hub state")
    d1 = 135
    s.diamond(cx, d1, 330, 130, ["Confirmed, LiDAR-ranged object", "in the ego path with", "TTC < 1.8 s, pedal not pressed,", "and the system ARMED?"], size=11)
    s.arrow([(cx, 60), (cx, d1 - 65)])
    s.box(480, d1 - 35, 220, 70, "Rule 1: BRAKE", ["intensity clamped (<= 90);", "logged with its context"])
    s.arrow([(cx + 165, d1), (480, d1)])
    s.text(cx + 175, d1 - 8, "Yes", 11, italic=True, anchor="start")
    d2 = 290
    s.diamond(cx, d2, 300, 100, ["Ego deceleration", "above 0.4 g?"])
    s.arrow([(cx, d1 + 65), (cx, d2 - 50)])
    s.text(cx + 8, d1 + 80, "No", 11, italic=True, anchor="start")
    s.box(480, d2 - 30, 220, 60, "Rule 2: HAZARDS", "hazard lights on")
    s.arrow([(cx + 150, d2), (480, d2)])
    s.text(cx + 160, d2 - 8, "Yes", 11, italic=True, anchor="start")
    d3 = 430
    s.diamond(cx, d3, 320, 110, ["HIGH or CRITICAL hazard", "(reckless driving, crossing,", "vulnerable user) in the path?"], size=11)
    s.arrow([(cx, d2 + 50), (cx, d3 - 55)])
    s.text(cx + 8, d2 + 66, "No", 11, italic=True, anchor="start")
    s.box(480, d3 - 35, 220, 70, "Rule 3: WARNING", ["overlay + tone only;", "never actuates"])
    s.arrow([(cx + 160, d3), (480, d3)])
    s.text(cx + 170, d3 - 8, "Yes", 11, italic=True, anchor="start")
    s.box(cx - 150, 545, 300, 56, "Rule 4: no request", "a zeroed command (releases)")
    s.arrow([(cx, d3 + 55), (cx, 545)])
    s.text(cx + 8, d3 + 74, "No", 11, italic=True, anchor="start")
    s.text(20, 628, "Predictions, collision probabilities, signs and map data never reach rule 1; they drive warnings only.", 11, italic=True, anchor="start")
    s.save("fig_arbiter.svg")


def fig_hub_states():
    s = Svg(720, 470)
    s.box(40, 170, 170, 80, "DISARMED", ["released, relays off,", "command forgotten"])
    s.box(300, 30, 170, 80, "ARMED", ["commands may act", "(clamped, <= 1.5 s)"])
    s.box(300, 330, 170, 80, "FAULT LATCHED", ["overcurrent: only a", "power cycle clears it"])
    s.box(520, 170, 180, 80, "Hardware release", ["watchdog task, no mutex;", "IWDG reset at 500 ms"])
    s.arrow([(210, 190), (300, 80)])
    s.text(150, 118, "fresh HEARTBEAT while", 11, italic=True, anchor="start")
    s.text(150, 132, "all conditions hold", 11, italic=True, anchor="start")
    s.arrow([(300, 95), (215, 210)])
    s.text(262, 172, "link lost > 200 ms, report", 11, italic=True, anchor="start")
    s.text(262, 186, "loop stalled, E-stop, frame errors,", 11, italic=True, anchor="start")
    s.text(262, 200, "power box unplugged", 11, italic=True, anchor="start")
    s.arrow([(385, 110), (385, 330)])
    s.text(392, 250, "current > limit for 30 ms", 11, italic=True, anchor="start")
    s.arrow([(300, 370), (125, 370), (125, 250)])
    s.arrow([(470, 70), (610, 70), (610, 170)], dashed=True)
    s.text(490, 62, "independent check every 20 ms", 11, italic=True, anchor="start")
    s.text(40, 450, "At boot the hub is DISARMED with the actuator released before any task runs.", 11, italic=True, anchor="start")
    s.save("fig_hub_states.svg")


def fig_navigation():
    s = Svg(720, 520)
    s.container(20, 10, 680, 110, "Road status: three layers (the map is an expectation; the sensors are the truth)")
    lay = [("1  Base map", ["OpenStreetMap, 100 km", "radius, refreshed nightly"]),
           ("2  Live network", ["traffic and incidents;", "deferred (licence terms)"]),
           ("3  Own sensors", ["closures, potholes;", "override the map"])]
    for i, (t, sub) in enumerate(lay):
        s.box(34 + i * 222, 40, 206, 66, t, sub, size=11, dashed=(i == 1))
    s.box(40, 170, 180, 60, "OSRM (MLD)", "route + geometry")
    s.box(270, 170, 180, 60, "Map matcher", "HMM over a GNSS trace")
    s.box(500, 170, 180, 60, "EKF pose", "local tangent plane")
    s.box(160, 290, 400, 70, "Road surface projector", ["route in the vehicle frame; lanes correct heading and", "lateral offset; heights from the LiDAR ground patch"])
    s.box(160, 410, 400, 60, "AR route band", "measured surface solid; far field fainter")
    s.arrow([(130, 120), (130, 170)])
    s.arrow([(220, 200), (270, 200)])
    s.arrow([(500, 200), (450, 200)])
    s.arrow([(360, 230), (360, 290)])
    s.text(368, 262, "progress along the route", 11, italic=True, anchor="start")
    s.arrow([(130, 230), (130, 325), (160, 325)])
    s.arrow([(590, 230), (590, 325), (560, 325)])
    s.arrow([(360, 360), (360, 410)])
    s.text(20, 500, "Measured on a recorded Nairobi drive (2,773 GNSS fixes): median snap distance 3.1 m; 98.7 % of fixes within 20 m.", 11, italic=True, anchor="start")
    s.save("fig_navigation.svg")


def fig_gantt():
    months = ["Aug", "Sep", "Oct", "Nov", "Dec", "Jan", "Feb", "Mar", "Apr"]
    tasks = [  # (label, start month index + fraction, end, done)
        ("Concept paper and presentation", 0.6, 1.0, True),
        ("Architecture, protocol, host foundations", 1.0, 1.6, True),
        ("ML perception and sign detector", 1.4, 2.0, True),
        ("Fusion, tracking, motion prediction (software)", 1.7, 2.0, True),
        ("Navigation and road projector (software)", 1.8, 2.0, True),
        ("Decision logic and AR renderer (software)", 1.9, 2.0, True),
        ("Hub firmware (code) and bench tooling", 1.95, 2.05, True),
        ("Two-box hub design (pod, power box)", 1.97, 2.05, True),
        ("Project proposal and proposal defence", 1.9, 3.6, False),
        ("Procurement and import of hardware", 2.0, 4.0, False),
        ("Actuator design approval and fabrication", 2.2, 4.3, False),
        ("Hub bring-up and camera pipeline (Ph. 2, 4)", 3.3, 4.6, False),
        ("LiDAR and calibration (Ph. 6)", 4.0, 5.3, False),
        ("Actuator bench gate (Ph. 12)", 4.3, 5.6, False),
        ("Custom PCBs and enclosures (Ph. 12B)", 4.4, 6.0, False),
        ("Bench-top end-to-end integration (Ph. 13)", 5.2, 6.2, False),
        ("Vehicle installation and calibration (Ph. 14)", 6.0, 6.8, False),
        ("Staged validation A-D (Ph. 15)", 6.5, 7.6, False),
        ("Final report, demonstration, defence (Ph. 16)", 7.3, 8.9, False),
    ]
    lw, cw, rh, top = 300, 44, 24, 50
    s = Svg(lw + cw * len(months) + 30, top + rh * len(tasks) + 90)
    for i, m in enumerate(months):
        x = lw + i * cw
        s.add(f'<rect x="{x}" y="{top - 26}" width="{cw}" height="{rh * len(tasks) + 26}" fill="none" stroke="#999" stroke-width="0.6"/>')
        s.text(x + cw / 2, top - 9, m, 12, bold=True)
    s.text(lw + cw * 2.5, top - 34, "2026", 12, bold=True)
    s.text(lw + cw * 7, top - 34, "2027", 12, bold=True)
    for r, (label, a, b, done) in enumerate(tasks):
        y = top + r * rh
        s.text(lw - 8, y + 16, label, 11.5, anchor="end")
        fill = "#333" if done else "url(#hatch)"
        s.add(f'<rect x="{lw + a * cw:.1f}" y="{y + 5}" width="{max(3, (b - a) * cw):.1f}" height="{rh - 10}" fill="{fill}" stroke="#000" stroke-width="1"/>')
    # progress-report milestones
    yb = top + rh * len(tasks) + 18
    s.text(lw - 8, yb + 4, "Progress reports / reviews", 11.5, anchor="end")
    for m in (3.3, 4.6, 5.8, 6.8, 7.8):
        x = lw + m * cw
        s.add(f'<polygon points="{x},{yb - 6} {x + 6},{yb} {x},{yb + 6} {x - 6},{yb}" fill="#000"/>')
    ly = yb + 32
    s.add(f'<rect x="{lw}" y="{ly - 10}" width="22" height="12" fill="#333" stroke="#000"/>')
    s.text(lw + 28, ly, "done (as of 29/09/2026)", 11.5, anchor="start")
    s.add(f'<rect x="{lw + 190}" y="{ly - 10}" width="22" height="12" fill="url(#hatch)" stroke="#000"/>')
    s.text(lw + 218, ly, "planned", 11.5, anchor="start")
    s.save("fig_gantt.svg")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    fig_architecture()
    fig_hub_hardware()
    fig_software_dataflow()
    fig_methodology()
    fig_frame_pipeline()
    fig_arbiter()
    fig_hub_states()
    fig_navigation()
    fig_gantt()
    fig_installation()
