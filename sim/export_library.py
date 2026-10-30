"""Export the margin cycle library for the viewer's MPC flight mode.

Reads sim/out/mpc_library_margin.json (python -m sim.mpc --build-margin-library) and writes
window.LIBRARY to sim/viewer/library.js (and, when sim/out/mpc_library_motor.json exists, the motor
library to sim/viewer/library_motor.js as window.LIBRARY_MOTOR, see build_motor):

  {"actuators": {...pb2v actuator constants...},
   "aircraft": {name: [{u10, chi_deg, margin_set, period, t, z, V, gamma, psi, cl, mu}, ...]}}

Only converged cycles are written. The cycle is in the wind frame (wind toward +x, angles
counter-clockwise from +x, z is the height above the surface). The x and y nodes are left out
(the planner does not use them in library mode). Values are rounded to 4 significant digits,
the node times to 6, the two angles gamma and psi to 5.
"""
import argparse
import json
import sys
from pathlib import Path

from . import record
from .aircraft import BASELINE, Actuators
from .mpc import LIB_MARGIN_PATH, LIB_MOTOR_PATH

_HERE = Path(__file__).resolve().parent
MODEL_FILES = [_HERE / "export_library.py", Path(LIB_MARGIN_PATH), _HERE / "mpc.py",
               _HERE / "aircraft.py", record.VALUES_PATH]
SIG = {"t": 6, "gamma": 5, "psi": 5}


def sig(a, n):
    return [float(f"{v:.{n}g}") for v in a]


def actuator_block():
    a = Actuators(BASELINE, rate_model="pb2v")
    return {"rate_model": "pb2v", "tau_s": a.tau, "cl_rate_per_s": a.cl_rate,
            "k_pb2v": a.k_pb2v, "delay_s": a.delay}


def build(path=LIB_MARGIN_PATH):
    with open(path) as fh:
        entries = json.load(fh)["entries"]
    out = {}
    for e in entries:
        if not e.get("success"):
            continue
        d = {"u10": e["u10"], "chi_deg": e["chi_deg"], "margin_set": e["margin_set"],
             "period": float(f"{e['period']:.6g}")}
        for k in ("t", "z", "V", "gamma", "psi", "cl", "mu"):
            d[k] = sig(e[k], SIG.get(k, 4))
        out.setdefault(e["aircraft"], []).append(d)
    return {"actuators": actuator_block(), "aircraft": out}


def build_motor(path=LIB_MOTOR_PATH):
    """window.LIBRARY_MOTOR: as build(), from the motor library (soaring and motor-assisted cycles of
    the kept aircraft). Each cycle also has kind ("soaring" or "motor"), power_avg_w and thrust (N per
    node, 4 significant digits). The old window.LIBRARY is left as it is."""
    with open(path) as fh:
        entries = json.load(fh)["entries"]
    out = {}
    for e in entries:
        if not e.get("success"):
            continue
        d = {"u10": e["u10"], "chi_deg": e["chi_deg"], "margin_set": e["margin_set"],
             "kind": e["kind"], "power_avg_w": float(f"{e.get('power_avg_w', 0.0):.4g}"),
             "period": float(f"{e['period']:.6g}")}
        for k in ("t", "z", "V", "gamma", "psi", "cl", "mu", "thrust"):
            d[k] = sig(e[k], SIG.get(k, 4))
        out.setdefault(e["aircraft"], []).append(d)
    return {"actuators": actuator_block(), "aircraft": out}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(_HERE / "viewer" / "library.js"))
    ap.add_argument("--motor-out", default=str(_HERE / "viewer" / "library_motor.js"))
    ap.add_argument("--only-motor", action="store_true", help="write only library_motor.js")
    a = ap.parse_args(argv)
    if not a.only_motor:
        data = build()
        with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(record.header("python -m sim.export_library", MODEL_FILES) + "\n")
            fh.write("window.LIBRARY = " + json.dumps(data, separators=(",", ":")) + ";\n")
        n = sum(len(v) for v in data["aircraft"].values())
        print("wrote %d cycles, %d bytes" % (n, Path(a.out).stat().st_size))
    if Path(LIB_MOTOR_PATH).exists():
        data = build_motor()
        with open(a.motor_out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(record.header("python -m sim.export_library --only-motor",
                                   MODEL_FILES + [Path(LIB_MOTOR_PATH)]) + "\n")
            fh.write("window.LIBRARY_MOTOR = " + json.dumps(data, separators=(",", ":")) + ";\n")
        n = sum(len(v) for v in data["aircraft"].values())
        print("wrote %d motor-library cycles, %d bytes" % (n, Path(a.motor_out).stat().st_size))


if __name__ == "__main__":
    sys.exit(main())
