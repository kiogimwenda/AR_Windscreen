#!/usr/bin/env python3
"""Actuator bench test (docs/BUILD_GUIDE.md Part 13.3): the Phase 12 gate, run and recorded.

The rig: actuator and pedal fixture on a bench (never in the car), hub on a bench 12 V supply, the
hub running the BENCH firmware (`cd firmware/sensor_actuator_hub && pio run -e bench -t upload`),
connected over USB. Run with the system Python (the one PlatformIO uses, which has pyserial;
the `python3` first on your PATH may be a virtual environment without it):

    /usr/bin/python3 tools/bench_rig/log_actuator_bench.py selftest
    /usr/bin/python3 tools/bench_rig/log_actuator_bench.py checklist --attempt 1
    /usr/bin/python3 tools/bench_rig/log_actuator_bench.py status

`checklist` walks the six items of Part 13.3, in order, and records everything under
bench_runs/<date>-attempt<N>/:
  1. Rig check: the actuator and pedal fixture are on the BENCH, not in a vehicle (you confirm).
  2. Sweep: a range of brakeRequest intensities; per step the applied intensity the hub reports
     (the evidence that its ceiling holds whatever is asked), peak current, peak force (load cell)
     and response time.
  3. PUSH-BACK: the actuator held at its maximum configured intensity while a PERSON pushes the
     pedal back by foot. THE most important check in the project. The tool records the force and
     current during the push; only YOUR answer, "the pedal moved", can pass it.
  4. Kill switch: the actuator applied, then the E-stop pressed. Measured: time from the hub
     sensing the E-stop to zero drive and zero current; and you confirm the actuator went limp.
  5. USB unplug: the actuator applied, then the USB cable physically pulled. Nothing can be seen
     from the laptop while it is out, so the HUB measures the time from the last host frame to the
     release and to zero current, and reports it when you plug back in. Pass: both within 200 ms.
     An oscilloscope on the BTS7960 enable line is the independent cross-check (recommended once).
     The pod stays powered through the inter-box cable while the USB cable is out (Part 4.8.4).
  6. Inter-box cable unplug (two-box hub, added 2026-09-30): the actuator applied, then the cable
     between the windscreen pod and the power box pulled at the power box. The power board's
     pull-downs release everything in hardware at once; the pod stays up on USB power, so the hub
     keeps talking and must report fault 5 (POWER_BOX) and applied 0, must NOT latch a false
     overcurrent from the floating current line, and must re-arm only after the cable is back.
     You confirm the actuator went limp. Recommended once: an oscilloscope on the power board's
     BTS7960 enable pin, triggered by the unplug.
`status` reports whether three CONSECUTIVE attempts passed all six items: Phase 12's exit
criterion (and Phase 12B's, repeated on the PCB hardware). Only then is
brake_actuator_max_intensity final (Part 13.3 item 7).

Safety of the tool itself: the hub clamps every request (SafetyCore and the driver, max 90) and
releases on its own if this script stops sending heartbeats. The script also sends a zeroed
"no request" command on exit, on Ctrl+C and on any error.
"""

import argparse
import csv
import datetime
import json
import os
import struct
import sys
import threading
import time
from pathlib import Path

# --- Protocol (Part 3; mirrors firmware/sensor_actuator_hub/include/hub/Protocol.h) -----------
START = 0xAA
MAX_PAYLOAD = 64
SENSOR_REPORT, ACTUATION_COMMAND, ACK_STATUS, HEARTBEAT, BENCH_TELEMETRY = 0x01, 0x02, 0x03, 0x04, 0x05
FMT = {
    SENSOR_REPORT: "<IddfB6fffHBBBB",   # 63 bytes
    ACTUATION_COMMAND: "<BBBHI",        # 9
    ACK_STATUS: "<IBBH",                # 8
    BENCH_TELEMETRY: "<IfBBBBfBBHH",    # 22
}
SIZES = {SENSOR_REPORT: 63, ACTUATION_COMMAND: 9, ACK_STATUS: 8, BENCH_TELEMETRY: 22}
CEILING = 90            # BrakeActuatorDriver::kMaxSafeIntensity
MAX_DURATION_MS = 1500  # hub_config::kMaxBrakeDurationMs
FAULTS = {0: "none", 1: "overcurrent (latched)", 2: "kill switch", 3: "link timeout", 4: "frame errors",
          5: "power box disconnected"}
FAULT_POWER_BOX = 5
ITEM_COUNT = 6


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, as Crc16.h."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type]) + struct.pack("<H", len(payload)) + payload
    return bytes([START]) + body + struct.pack("<H", crc16(body))


class Parser:
    """Byte-stream frame parser with the firmware's resynchronisation (FrameCodec.h)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        frames, errors = [], 0
        while True:
            i = self.buf.find(START)
            if i < 0:
                self.buf.clear()
                break
            del self.buf[:i]
            if len(self.buf) < 4:
                break
            length = self.buf[2] | (self.buf[3] << 8)
            if length > MAX_PAYLOAD:
                errors += 1
                del self.buf[:1]
                continue
            total = 4 + length + 2
            if len(self.buf) < total:
                break
            body, rx = bytes(self.buf[1 : 4 + length]), self.buf[4 + length] | (self.buf[5 + length] << 8)
            if rx != crc16(body):
                errors += 1
                del self.buf[:1]  # re-scan from the next byte
                continue
            frames.append((self.buf[1], bytes(self.buf[4 : 4 + length])))
            del self.buf[:total]
        return frames, errors


def selftest() -> bool:
    ok = True

    def check(cond, what):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + what)
        ok &= bool(cond)

    check(crc16(b"123456789") == 0x29B1, "CRC-16/CCITT-FALSE check value 0x29B1")
    check(encode(HEARTBEAT) == bytes([0xAA, 0x04, 0x00, 0x00, 0x5C, 0x10]), "heartbeat frame bytes (firmware test vector)")
    for t, n in SIZES.items():
        check(struct.calcsize(FMT[t]) == n, f"message 0x{t:02X} is {n} bytes")
    cmd = struct.pack(FMT[ACTUATION_COMMAND], 0, 0, 77, 900, 12345)
    frame = encode(ACTUATION_COMMAND, cmd)
    frames, _ = Parser().feed(b"\x13\x37" + frame)
    check(frames == [(ACTUATION_COMMAND, cmd)], "round trip through leading garbage")
    bad = bytearray(frame)
    bad[6] ^= 0x01
    frames, errs = Parser().feed(bytes(bad) + encode(HEARTBEAT))
    check(errs >= 1 and frames == [(HEARTBEAT, b"")], "corrupted frame rejected, next frame recovered")
    return ok


# --- Link to the hub -----------------------------------------------------------------------------


class Hub:
    """Keeps the link alive (heartbeat 20 Hz, zeroed command 10 Hz when idle) and records frames."""

    def __init__(self, port: str, run_dir: Path):
        import serial  # pyserial; PlatformIO's Python has it

        self.serial_mod = serial
        self.port_name = port
        self.ser = serial.Serial(port, 115200, timeout=0.01)
        self.parser = Parser()
        self.lock = threading.Lock()
        self.telemetry = []  # (host_s, dict)
        self.acks = []
        self.reports = []
        self.frame_errors = 0
        self.host_ts = 0
        self.stop = threading.Event()
        self.connected = threading.Event()
        self.connected.set()
        self.run_dir = run_dir
        self.tel_csv = open(run_dir / "telemetry.csv", "w", newline="")
        self.tel_w = csv.writer(self.tel_csv)
        self.tel_w.writerow(["host_s", "hub_ms", "current_a", "applied", "magnet", "armed", "fault",
                             "load_n", "load_valid", "kill", "loss_release_ms", "loss_zero_ms"])
        self.ack_csv = open(run_dir / "acks.csv", "w", newline="")
        self.ack_w = csv.writer(self.ack_csv)
        self.ack_w.writerow(["host_s", "hub_ms", "accepted", "fault", "applied"])
        self.threads = [threading.Thread(target=self._reader, daemon=True),
                        threading.Thread(target=self._writer, daemon=True)]
        for t in self.threads:
            t.start()

    def _write(self, data: bytes):
        try:
            self.ser.write(data)
        except Exception:
            self.connected.clear()

    def _reader(self):
        while not self.stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception:
                self.connected.clear()
                time.sleep(0.05)
                continue
            if not data:
                continue
            now = time.time()
            frames, errs = self.parser.feed(data)
            with self.lock:
                self.frame_errors += errs
                for t, p in frames:
                    if t == BENCH_TELEMETRY and len(p) == 22:
                        v = struct.unpack(FMT[t], p)
                        d = dict(hub_ms=v[0], current=v[1], applied=v[2], magnet=v[3], armed=v[4], fault=v[5],
                                 load=v[6], load_valid=v[7], kill=v[8], loss_release=v[9], loss_zero=v[10])
                        self.telemetry.append((now, d))
                        self.tel_w.writerow([f"{now:.4f}", *v])
                    elif t == ACK_STATUS and len(p) == 8:
                        v = struct.unpack(FMT[t], p)
                        self.acks.append((now, dict(hub_ms=v[0], accepted=v[1], fault=v[2], applied=v[3])))
                        self.ack_w.writerow([f"{now:.4f}", *v])
                    elif t == SENSOR_REPORT and len(p) == 63:
                        self.reports.append(now)

    def _writer(self):
        n = 0
        while not self.stop.is_set():
            self._write(encode(HEARTBEAT))
            if n % 2 == 0 and self.idle:
                self.command(0, 0)
            n += 1
            time.sleep(0.05)

    idle = True

    def command(self, intensity: int, duration_ms: int):
        self.host_ts += 1  # strictly increasing: the hub rejects stale or replayed commands
        payload = struct.pack(FMT[ACTUATION_COMMAND], 0, 0, intensity, duration_ms, self.host_ts)
        self._write(encode(ACTUATION_COMMAND, payload))

    def hold(self, intensity: int, seconds: float):
        """Keep a brake request applied for `seconds` (renewed within the hub's 1.5 s limit)."""
        self.idle = False
        end = time.time() + seconds
        while time.time() < end:
            self.command(intensity, MAX_DURATION_MS)
            time.sleep(min(0.5, max(0.0, end - time.time())))
        self.release()

    def release(self):
        self.idle = True
        for _ in range(3):
            self.command(0, 0)
            time.sleep(0.02)

    def latest(self):
        with self.lock:
            return self.telemetry[-1][1] if self.telemetry else None

    def since(self, t0: float):
        with self.lock:
            return [d for t, d in self.telemetry if t >= t0]

    def wait_armed(self, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            d = self.latest()
            if d and d["armed"]:
                return True
            time.sleep(0.05)
        return False

    def reconnect(self, timeout=20.0):
        """After the USB cable was unplugged: reopen the port once it re-enumerates."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                self.ser.close()
            except Exception:
                pass
            try:
                self.ser = self.serial_mod.Serial(self.port_name, 115200, timeout=0.01)
                self.connected.set()
                return True
            except Exception:
                time.sleep(0.5)
        return False

    def close(self):
        try:
            self.release()
        finally:
            self.stop.set()
            time.sleep(0.1)
            self.ser.close()
            self.tel_csv.close()
            self.ack_csv.close()


# --- The six items ----------------------------------------------------------------------------


def ask(question: str) -> bool:
    while True:
        a = input(f"{question} [y/n] ").strip().lower()
        if a in ("y", "yes"):
            return True
        if a in ("n", "no"):
            return False


def item1_rig() -> dict:
    print("\n=== Item 1: rig ===")
    on_bench = ask("Is the actuator mounted with the pedal fixture on the BENCH rig, NOT in a vehicle?")
    ok = on_bench and ask("Is the kill-switch relay in the actuator supply, and the E-stop within reach?")
    return {"pass": ok, "on_bench": on_bench}


def item2_sweep(hub: Hub, steps, hold_s: float, rest_s: float) -> dict:
    print("\n=== Item 2: intensity sweep ===")
    rows, ceiling_ok = [], True
    base = [d["current"] for d in hub.since(time.time() - 1.0)] or [0.0]
    baseline = sorted(base)[len(base) // 2]
    for i in steps:
        hub.wait_armed()
        t0 = time.time()
        hub.hold(i, hold_s)
        seg = hub.since(t0)
        applied = max((d["applied"] for d in seg), default=0)
        expected = min(i, CEILING)
        peak_a = max((d["current"] for d in seg), default=0.0)
        peak_n = max((d["load"] for d in seg if d["load_valid"]), default=float("nan"))
        onset = next((d["hub_ms"] for d in seg if d["current"] > baseline + 0.3), None)
        first = seg[0]["hub_ms"] if seg else None
        resp = (onset - first) if (onset is not None and first is not None) else None
        ok = applied == expected and applied <= CEILING
        ceiling_ok &= ok
        rows.append(dict(request=i, applied=applied, expected=expected, peak_current_a=round(peak_a, 2),
                         peak_force_n=None if peak_n != peak_n else round(peak_n, 1), response_ms=resp, ok=ok))
        print(f"  request {i:3d} -> applied {applied:3d} (expected {expected:3d}) peak {peak_a:5.2f} A "
              f"force {rows[-1]['peak_force_n']} N response {resp} ms {'OK' if ok else 'MISMATCH'}")
        time.sleep(rest_s)
    with open(hub.run_dir / "sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return {"pass": ceiling_ok, "steps": rows}


def item3_pushback(hub: Hub, seconds: float) -> dict:
    print("\n=== Item 3: PUSH-BACK (the most important check) ===")
    print(f"The actuator will be held at its MAXIMUM configured intensity ({CEILING}) for {seconds:.0f} s.")
    input("Stand ready at the pedal. Press Enter to engage, then PUSH the pedal back by foot... ")
    t0 = time.time()
    hub.hold(255, seconds)  # asks for 255: the hub applies its ceiling
    seg = hub.since(t0)
    peak_n = max((d["load"] for d in seg if d["load_valid"]), default=float("nan"))
    peak_a = max((d["current"] for d in seg), default=0.0)
    applied = max((d["applied"] for d in seg), default=0)
    print(f"  applied {applied}, peak force {peak_n:.1f} N, peak current {peak_a:.2f} A")
    moved = ask("Could you MOVE THE PEDAL by foot against the actuator, clearly and without strain?")
    return {"pass": moved and applied == CEILING, "pedal_moved": moved, "applied": applied,
            "peak_force_n": None if peak_n != peak_n else round(peak_n, 1), "peak_current_a": round(peak_a, 2)}


def item4_kill(hub: Hub) -> dict:
    print("\n=== Item 4: kill switch mid-actuation ===")
    input("Press Enter to apply the actuator, then PRESS THE E-STOP while it is applied... ")
    hub.idle = False
    t0 = time.time()
    engaged_at = released_at = zero_at = None
    while time.time() - t0 < 15 and zero_at is None:
        if engaged_at is None:
            hub.command(CEILING, MAX_DURATION_MS)
        for d in hub.since(t0):
            if engaged_at is None and d["kill"]:
                engaged_at = d["hub_ms"]
            if engaged_at is not None and released_at is None and d["applied"] == 0:
                released_at = d["hub_ms"]
            if engaged_at is not None and zero_at is None and d["current"] < 0.2:
                zero_at = d["hub_ms"]
        time.sleep(0.2)
    hub.release()
    rel = None if engaged_at is None or released_at is None else released_at - engaged_at
    zero = None if engaged_at is None or zero_at is None else zero_at - engaged_at
    print(f"  E-stop sensed: {engaged_at is not None}; drive off after {rel} ms; current zero after {zero} ms")
    limp = ask("Did the actuator go limp / the cable drop free the moment you pressed the E-stop?")
    input("Twist-release the E-stop, then press Enter...")
    return {"pass": limp and engaged_at is not None and rel is not None and rel <= 200, "estop_sensed": engaged_at is not None,
            "drive_off_ms": rel, "current_zero_ms": zero, "operator_confirms_limp": limp}


def item5_unplug(hub: Hub) -> dict:
    print("\n=== Item 5: USB unplug (watchdog release) ===")
    print("The pod must stay powered through the INTER-BOX CABLE during this test (power box on the")
    print("bench supply), so the hub keeps running and measures its own release.")
    input("Press Enter to apply the actuator, then PULL THE USB CABLE while it is applied... ")
    hub.idle = False
    t0 = time.time()
    while hub.connected.is_set() and time.time() - t0 < 20:
        hub.command(CEILING, MAX_DURATION_MS)
        time.sleep(0.2)
    released = ask("Cable out. Did the actuator release by itself (no command was sent)?")
    input("Plug the USB cable back in and press Enter... ")
    hub.idle = True
    if not hub.reconnect():
        print("  could not reopen the port")
        return {"pass": False, "reason": "no reconnect", "operator_confirms_release": released}
    time.sleep(1.0)
    d = hub.latest()
    rel = d["loss_release"] if d else 0xFFFF
    zero = d["loss_zero"] if d else 0xFFFF
    print(f"  hub-measured: release {rel} ms, current zero {zero} ms after the last host frame")
    ok = released and rel != 0xFFFF and rel <= 200 and zero != 0xFFFF and zero <= 200
    return {"pass": ok, "release_ms": rel, "current_zero_ms": zero, "operator_confirms_release": released}


def item6_interbox(hub: Hub) -> dict:
    print("\n=== Item 6: inter-box cable unplug (power box disconnected) ===")
    print("The pod stays powered from USB; only the cable to the power box is pulled.")
    input("Press Enter to apply the actuator, then PULL THE INTER-BOX CABLE at the power box... ")
    hub.idle = False
    t0 = time.time()
    absent_at = released_at = None
    while time.time() - t0 < 15 and released_at is None:
        if absent_at is None:
            hub.command(CEILING, MAX_DURATION_MS)
        for d in hub.since(t0):
            if absent_at is None and d["fault"] == FAULT_POWER_BOX:
                absent_at = d["hub_ms"]
            if absent_at is not None and released_at is None and d["applied"] == 0:
                released_at = d["hub_ms"]
        time.sleep(0.2)
    hub.release()
    rel = None if absent_at is None or released_at is None else released_at - absent_at
    print(f"  fault 5 reported: {absent_at is not None}; firmware drive off {rel} ms after it")
    limp = ask("Did the actuator go limp / the cable drop free the moment the cable came out?")
    input("Plug the inter-box cable back in and press Enter... ")
    rearmed = hub.wait_armed(5.0)
    d = hub.latest()
    fault_after = d["fault"] if d else None
    print(f"  after reconnect: armed={rearmed}, fault={FAULTS.get(fault_after, fault_after)}")
    ok = (limp and absent_at is not None and rel is not None and rel <= 200 and rearmed
          and fault_after == 0)
    return {"pass": ok, "power_box_fault_reported": absent_at is not None, "drive_off_ms": rel,
            "operator_confirms_limp": limp, "rearmed_after_reconnect": rearmed,
            "fault_after_reconnect": fault_after}


def checklist(args) -> int:
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = Path(args.out) / f"{stamp}-attempt{args.attempt}"
    run_dir.mkdir(parents=True, exist_ok=True)
    result = {"attempt": args.attempt, "started": stamp, "ceiling": CEILING, "items": {}}
    result["items"]["1_rig"] = item1_rig()
    if not result["items"]["1_rig"]["pass"]:
        print("Item 1 failed: stop. Nothing is commanded until the rig is on the bench.")
    else:
        hub = Hub(args.port, run_dir)
        try:
            if not hub.wait_armed(5.0):
                print("The hub did not arm (is it the BENCH firmware? E-stop released? heartbeats reaching it?)")
                d = hub.latest()
                result["items"]["arming"] = {"pass": False, "fault": FAULTS.get(d["fault"], d["fault"]) if d else "no telemetry"}
            else:
                steps = [int(s) for s in args.steps.split(",")]
                result["items"]["2_sweep"] = item2_sweep(hub, steps, args.hold, args.rest)
                result["items"]["3_pushback"] = item3_pushback(hub, args.push_seconds)
                result["items"]["4_kill_switch"] = item4_kill(hub)
                result["items"]["5_unplug"] = item5_unplug(hub)
                if not hub.wait_armed(5.0):
                    print("The hub did not re-arm after item 5; item 6 cannot start.")
                    result["items"]["6_interbox"] = {"pass": False, "reason": "not re-armed after item 5"}
                else:
                    result["items"]["6_interbox"] = item6_interbox(hub)
        except KeyboardInterrupt:
            print("\nInterrupted: releasing.")
            result["interrupted"] = True
        finally:
            hub.close()
    result["all_pass"] = all(v.get("pass") for v in result["items"].values()) and len(result["items"]) == ITEM_COUNT
    (run_dir / "checklist.json").write_text(json.dumps(result, indent=2))
    print(f"\nAttempt {args.attempt}: {'ALL SIX PASS' if result['all_pass'] else 'NOT PASSED'}  ->  {run_dir}")
    return 0 if result["all_pass"] else 1


def status(args) -> int:
    runs = sorted(Path(args.out).glob("*/checklist.json"))
    streak = best = 0
    for r in runs:
        res = json.loads(r.read_text())
        mark = "PASS" if res.get("all_pass") else "fail"
        print(f"{r.parent.name}: {mark}  " + "  ".join(f"{k}={'ok' if v.get('pass') else 'X'}" for k, v in res["items"].items()))
        streak = streak + 1 if res.get("all_pass") else 0
        best = max(best, streak)
    print(f"\nconsecutive full passes (latest streak): {streak}; Phase 12 gate {'MET' if streak >= 3 else 'not met'} (needs 3)")
    return 0 if streak >= 3 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("selftest", help="check the protocol code; no hardware")
    c = sub.add_parser("checklist", help="run Part 13.3's six items")
    c.add_argument("--attempt", type=int, required=True)
    c.add_argument("--port", default="/dev/ttyACM0")
    c.add_argument("--steps", default="0,15,30,45,60,75,90,120,200,255")
    c.add_argument("--hold", type=float, default=1.0, help="seconds per sweep step")
    c.add_argument("--rest", type=float, default=1.5)
    c.add_argument("--push-seconds", type=float, default=8.0)
    c.add_argument("--out", default="bench_runs")
    s = sub.add_parser("status", help="have three consecutive attempts passed?")
    s.add_argument("--out", default="bench_runs")
    args = ap.parse_args()
    if args.cmd == "selftest":
        return 0 if selftest() else 1
    if args.cmd == "checklist":
        return checklist(args)
    return status(args)


if __name__ == "__main__":
    sys.exit(main())
