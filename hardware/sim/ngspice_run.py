"""Runs one SPICE netlist with the ngspice that ships with KiCad (its ngspice.dll), from KiCad's
own Python, so nothing has to be installed. Run through the simulation scripts, e.g. run_buck.sh.

    python.exe ngspice_run.py <netlist.cir> <out.raw> <log.txt> <vector> [<vector> ...]

PSpice compatibility mode is on (vendor models, TI's included, are written for PSpice). Writes
the listed vectors to <out.raw> (ASCII raw file) and everything ngspice prints to <log.txt>.
"""

import ctypes
import sys

netlist, raw, log = sys.argv[1:4]
vectors = sys.argv[4:]
dll = ctypes.CDLL(r"C:\Program Files\KiCad\10.0\bin\ngspice.dll")
lines = []
SendChar = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
SendStat = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
ControlledExit = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_bool,
                                  ctypes.c_int, ctypes.c_void_p)


def _out(s, _id, _p):
    lines.append(s.decode(errors="replace"))
    return 0


callbacks = (SendChar(_out), SendStat(lambda s, i, p: 0), ControlledExit(lambda *a: 0))
dll.ngSpice_Init(callbacks[0], callbacks[1], callbacks[2], None, None, None, None)
# What ngspice's own start-up file (spinit, not found by the shared library) would do: load the
# XSPICE code models; PSpice switches and some behavioural parts are translated into them.
# (ngspice splits arguments at spaces and keeps quotes, so the path is given in Windows' short,
# space-free form.)
_buf = ctypes.create_unicode_buffer(512)
ctypes.windll.kernel32.GetShortPathNameW(r"C:\Program Files\KiCad\10.0\lib\ngspice", _buf, 512)
CM = _buf.value.replace("\\", "/") + "/"
setup = [f"codemodel {CM}{m}.cm" for m in ("analog", "digital", "spice2poly", "table", "xtradev", "xtraevt")]
for cmd in setup + ["set ngbehavior=ps", "set filetype=ascii", "source " + netlist.replace("\\", "/"),
                   "save " + " ".join(vectors),  # only these are kept: less memory, smaller files
                   "run", "write " + raw.replace("\\", "/") + " " + " ".join(vectors)]:
    dll.ngSpice_Command(cmd.encode())
open(log, "w", encoding="utf-8").write("\n".join(lines) + "\n")
errors = [ln for ln in lines if "error" in ln.lower() and "stderr" in ln.lower()]
print(f"{netlist}: {len(lines)} lines of output, {len(errors)} errors")
sys.exit(1 if errors else 0)
