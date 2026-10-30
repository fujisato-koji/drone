"""Speed polar of dynamic soaring: progress speed against the course relative to the wind.

Convention (checked in sim.dsopt.derivs: the wind U is added to the x velocity): the wind blows
toward +x and chi is the course angle ccw from +x, so chi = 0 is straight downwind and
chi = 180 deg is straight upwind. Each case is dsopt.solve(mode="max_speed", cycle="travel")
with the margin library's settings (sim.mpc.MARGIN_SETS, coare35 log profile, midpoints, pb2v
rate limits, n_min 0, geometric tip) and replay_check on.

Command: python -m sim.polar   (writes sim/out/ds_polar.json and prints markdown tables).
"""
from __future__ import annotations

import os

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_k] = "1"

import argparse
import json
import sys
import time

import numpy as np

AIRCRAFT = ("albatross-3m", "baseline-2m", "sealedhull-2p5m")
WINDS = (9.0, 11.0, 13.0, 16.0, 20.0)
COURSES = tuple(float(c) for c in range(0, 181, 15))
N_NODES = 40
OUT_PATH = os.path.join(os.path.dirname(__file__), "out", "ds_polar.json")
KEYS = ("h_min", "n_frac", "mu_frac", "v_factor")
MS_STANDARD = (2.0, 0.8, 0.9, 1.2)   # = sim.mpc.MARGIN_SETS[0]


def _margin_sets():
    from sim.mpc import MARGIN_SETS
    return MARGIN_SETS


def _entry(info, label, ms, chi_deg, runs):
    """Case record from a dsopt info dict (progress = mean speed along the course)."""
    prog = float(info["speed_along_chi"])
    return {"chi_deg": chi_deg, "converged": True, "margin_set": label,
            "margins": dict(zip(KEYS, ms)), "progress": prog,
            "ground_speed": float(info["ground_speed"]), "period": float(info["period"]),
            "n_max": float(info["n_max"]),
            "z_min": float(info["z_min"]), "runs": runs}


def solve_case(name, u10, chi_deg, ms, x0=None, n=N_NODES, seeds=(0, 1, 2)):
    """One dsopt solve with margin set ms = (h_min, n_frac, mu_frac, v_factor); returns
    (info, x), x None when the solve (with replay check) did not succeed. With x0 the seeds are
    not used."""
    from sim.aircraft_models import by_name
    from sim.dsopt import profile_for, solve
    h_min, n_frac, mu_frac, v_factor = ms
    info, x, _ = solve(by_name(name), profile_for(u10, "log", roughness="coare35"),
                       mode="max_speed", cycle="travel", n=n, u10=u10, chi=np.radians(chi_deg),
                       tip_model="geometric", midpoints=True, rate_limits=True,
                       rate_model="pb2v", n_min=0.0, h_min=h_min, n_frac=n_frac,
                       mu_frac=mu_frac, v_factor=v_factor, replay_check=True,
                       seeds=seeds, x0=x0, return_x=True, stop_after=1)
    return info, (np.asarray(x) if info["success"] else None)


def _run_set(name, u10, label, ms, courses, solved, n):
    """Sweep one margin set over the courses not yet in solved ({chi: [info, x, label, ms, runs]}),
    adding what converges. Procedure: probe cold (library seeds) at 90 deg; walk outward from 90
    to 180 and from 90 to 0 with a warm start from the previous course and, if that fails, one cold
    solve, stopping a direction at the first course where both fail; then a return sweep
    0 -> 180 and 180 -> 0 with warm starts only, over every course not yet solved by this set,
    keeping the better speed. Courses already solved by an earlier (standard) set are not
    re-solved, but their cycles serve as warm starts."""
    local = {}
    xs = {c: v[1] for c, v in solved.items()}   # latest cycle per course, any set

    def attempt(c, warm):
        info, x = solve_case(name, u10, c, ms, x0=warm, n=n)
        if x is not None:
            if c not in local:
                local[c] = [info, x, label, ms, 0]
            elif info["speed_along_chi"] > local[c][0]["speed_along_chi"]:
                local[c][:4] = [info, x, label, ms]
            local[c][4] += 1
            xs[c] = local[c][1]
        return x

    todo = [c for c in courses if c not in solved]
    if not todo:
        return local
    c0 = 90.0
    if c0 in solved:
        xs[c0] = solved[c0][1]
    elif attempt(c0, None) is None:   # cold probe failed: this set does not soar at this wind
        return local
    for direction in (1, -1):
        prev = xs[c0]
        for c in sorted((c for c in courses if (c - c0) * direction > 0), key=lambda c: abs(c - c0)):
            if c in solved:
                prev = xs[c]
                continue
            x = attempt(c, prev)
            if x is None:
                x = attempt(c, None)
            if x is None:
                break
            prev = x
    for order in (sorted(todo), sorted(todo, reverse=True)):
        prev = None
        for c in order:
            if c in solved:
                prev = xs[c]
                continue
            if prev is not None:
                x = attempt(c, prev)
                if x is not None:
                    prev = x
                    continue
            if c in xs:
                prev = xs[c]
    return local


def sweep_row(name, u10, courses=COURSES, n=N_NODES):
    """All courses for one aircraft and wind: the standard margins, then the lighter ones for
    the courses the standard margins did not solve (see _run_set)."""
    solved = {}
    for label, ms in _margin_sets():
        solved.update(_run_set(name, u10, label, ms, courses, solved, n))
    rows = []
    for c in courses:
        if c in solved:
            info, _, label, ms, runs = solved[c]
            rows.append(_entry(info, label, ms, c, runs))
        else:
            rows.append({"chi_deg": c, "converged": False})
    return {"aircraft": name, "u10": u10, "cases": rows}


def _job(args):
    name, u10 = args
    t0 = time.time()
    row = sweep_row(name, u10)
    row["runtime_s"] = time.time() - t0
    print(f"done {name} {u10:g} m/s in {row['runtime_s']:.0f} s", file=sys.stderr, flush=True)
    return row


def tables(rows):
    lines = []
    for name in AIRCRAFT:
        rs = sorted((r for r in rows if r["aircraft"] == name), key=lambda r: r["u10"])
        if not rs:
            continue
        lines += [f"### {name}: progress speed (m/s) by course (deg; 0 downwind, 180 upwind)", "",
                  "| u10 (m/s) | " + " | ".join(f"{c:g}" for c in COURSES) + " |",
                  "|---" * (len(COURSES) + 1) + "|"]
        for r in rs:
            cells = [f"{c['progress']:.1f}" + ("*" if c["margin_set"] == "light" else "")
                     if c.get("converged") else "no cycle" for c in r["cases"]]
            lines.append(f"| {r['u10']:g} | " + " | ".join(cells) + " |")
        lines += ["", "(* lighter margins)"]
        lines.append("")
    return "\n".join(lines)


def main(argv=None):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--drop", default="", help="comma list of name:u10 rows to skip")
    ap.add_argument("--out", default=OUT_PATH)
    a = ap.parse_args(argv)
    drop = {(s.split(":")[0], float(s.split(":")[1])) for s in a.drop.split(",") if s}
    jobs = [(n, u) for u in WINDS for n in AIRCRAFT if (n, u) not in drop]
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(_job, jobs, chunksize=1))
    here = os.path.dirname(__file__)
    meta = record.meta("python -m sim.polar" + (f" --drop {a.drop}" if a.drop else ""),
                       [__file__, os.path.join(here, "dsopt.py"), os.path.join(here, "mpc.py")])
    meta.update({"runtime_s": time.time() - t0, "n_nodes": N_NODES,
                 "dropped": sorted(f"{n}:{u:g}" for n, u in drop),
                 "frame": "wind toward +x, chi ccw from +x: 0 downwind, 180 deg straight upwind",
                 "dsopt": "solve(mode=max_speed, cycle=travel, midpoints, rate_limits, pb2v, "
                          "n_min 0, tip_model geometric, replay_check, coare35 log profile, "
                          "seeds 0-2 stop_after 1, plus warm starts from the neighbouring "
                          "course in both sweep directions)",
                 "margin_sets": {k: dict(zip(KEYS, v)) for k, v in _margin_sets()},
                 "margin_order": "standard first; light only for courses standard did not solve"})
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="\n") as fh:
        json.dump({"meta": meta, "rows": rows}, fh, indent=1)
        fh.write("\n")
    print(tables(rows))
    print(f"runtime {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
