"""Export the parameters of the aircraft of the MPC library to sim/viewer/aircraft.js.

The viewer needs the point-mass parameters of every aircraft in window.LIBRARY. They came with the
reference cycles of refs.js, which is gone with the tracker; this file carries them alone.
"""
import argparse
import json
import sys
from pathlib import Path

from . import record
from .aircraft_models import by_name
from .export_library import build

_HERE = Path(__file__).resolve().parent
MODEL_FILES = [_HERE / "export_aircraft.py", _HERE / "export_library.py", _HERE / "aircraft_models.py",
               _HERE / "aircraft.py", record.VALUES_PATH]


def params(ac):
    return {"mass": ac.mass, "span": ac.span, "wing_area": ac.wing_area, "cd0": ac.cd0,
            "oswald": ac.oswald, "cl_max": ac.cl_max, "cl_min": ac.cl_min, "mu_max": ac.mu_max,
            "n_max": ac.n_max, "rho": ac.rho, "k_induced": float(f"{ac.k_induced:.6g}"),
            "v_stall": float(f"{ac.v_stall:.6g}")}


def build_params():
    return {name: params(by_name(name)) for name in build()["aircraft"]}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(_HERE / "viewer" / "aircraft.js"))
    a = ap.parse_args(argv)
    data = build_params()
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header("python -m sim.export_aircraft", MODEL_FILES) + "\n")
        fh.write("window.AIRCRAFT = " + json.dumps(data, separators=(",", ":")) + ";\n")
    print("wrote %d aircraft, %d bytes" % (len(data), Path(a.out).stat().st_size))


if __name__ == "__main__":
    sys.exit(main())
