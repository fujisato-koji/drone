"""Export periodic dynamic-soaring reference cycles for the viewer's flight mode.

For every candidate except the bird, every u10 and both cycles, solve with sim.dsopt (max_speed
mode, because a fixed u10 needs it; default Charnock log profile, dsopt defaults) and write
window.REFS to sim/viewer/refs.js.

  travel: chi = +90 deg (wind along +x, so the cycle drifts across the wind), as the autonomy
          reference does.
  loiter: chi = 180 deg (maximum ground speed straight upwind, i.e. the least downwind drift
          over a full turn of heading).
"""
import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from . import record
from .aircraft_models import candidates
from .dsopt import Profile, solve
from .seawind import Wind

_HERE = Path(__file__).resolve().parent
MODEL_FILES = [_HERE / "export_refs.py", _HERE / "dsopt.py", _HERE / "aircraft.py",
               _HERE / "aircraft_models.py", _HERE / "seawind.py", record.VALUES_PATH]
CHI = {"travel": np.pi / 2, "loiter": np.pi}
KEYS = ("t", "x", "y", "z", "V", "gamma", "psi", "cl", "mu")


def sig4(a):
    return [float(f"{v:.4g}") for v in np.asarray(a, float)]


def solve_one(job):
    name, u10, cycle = job
    from .aircraft_models import by_name
    ac = by_name(name)
    t0 = time.time()
    try:
        prof = Profile.from_wind(Wind(float(u10), 270.0))
        info, x, col = solve(ac, prof, mode="max_speed", cycle=cycle, n=40, u10=float(u10),
                             chi=CHI[cycle], return_x=True)
    except Exception as e:  # a failed solve is a skipped entry
        return job, None, time.time() - t0, str(e)
    if not info["success"]:
        return job, None, time.time() - t0, "no convergence"
    S, C, T, _ = col.unpack(x)
    n = len(S)
    d = {"t": sig4(np.linspace(0.0, T, n)), "x": sig4(S[:, 0]), "y": sig4(S[:, 1]),
         "z": sig4(S[:, 2]), "V": sig4(S[:, 3]), "gamma": sig4(S[:, 4]), "psi": sig4(S[:, 5]),
         "cl": sig4(C[:, 0]), "mu": sig4(C[:, 1]), "period": float(f"{T:.4g}"), "success": True}
    return job, d, time.time() - t0, ""


def params(ac):
    return {"mass": ac.mass, "span": ac.span, "wing_area": ac.wing_area, "cd0": ac.cd0,
            "oswald": ac.oswald, "cl_max": ac.cl_max, "cl_min": ac.cl_min, "mu_max": ac.mu_max,
            "n_max": ac.n_max, "rho": ac.rho, "k_induced": float(f"{ac.k_induced:.6g}"),
            "v_stall": float(f"{ac.v_stall:.6g}")}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=1, help="u10 step in m/s (8 to 20)")
    ap.add_argument("--out", default=str(_HERE / "viewer" / "refs.js"))
    ap.add_argument("--workers", type=int, default=0)
    a = ap.parse_args(argv)
    # one BLAS thread per worker: with eight workers the default thread pools oversubscribe the
    # cores and slow the solves by an order of magnitude (children inherit the environment)
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[var] = "1"
    cands = [ac for ac in candidates() if ac.name != "bird-wandering"]
    u10s = list(range(8, 21, a.step))
    jobs = [(ac.name, u, c) for ac in cands for u in u10s for c in ("travel", "loiter")]
    t0 = time.time()
    with Pool(a.workers or None) as pool:
        results = []
        for r in pool.imap(solve_one, jobs, chunksize=1):
            results.append(r)
            print("%d/%d %s u10=%d %s %s (%.0f s)" % (len(results), len(jobs), r[0][0], r[0][1], r[0][2],
                  "ok" if r[1] else "skip", r[2]), file=sys.stderr, flush=True)
    data = {ac.name: {"params": params(ac), "travel": {}, "loiter": {}} for ac in cands}
    ok = {}
    for (name, u, c), d, rt, why in results:
        ok[(name, u, c)] = (d is not None, rt)
        if d is not None:
            data[name][c][str(u)] = d
    command = "python -m sim.export_refs" + ("" if a.step == 1 else " --step %d" % a.step)
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header(command, MODEL_FILES) + "\n")
        fh.write("window.REFS = " + json.dumps(data, separators=(",", ":")) + ";\n")
    print("candidate".ljust(18) + "cycle   " + " ".join("%3d" % u for u in u10s) + "  (+ converged, . skipped)")
    for ac in cands:
        for c in ("travel", "loiter"):
            print(ac.name.ljust(18) + c.ljust(8) + " ".join("  " + ("+" if ok[(ac.name, u, c)][0] else ".")
                                                          for u in u10s))
    n_ok = sum(v[0] for v in ok.values())
    print("converged %d of %d; solve time sum %.0f s; wall %.0f s; file %d bytes"
          % (n_ok, len(ok), sum(v[1] for v in ok.values()), time.time() - t0,
             Path(a.out).stat().st_size))


if __name__ == "__main__":
    sys.exit(main())
