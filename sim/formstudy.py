"""Batch form study: run the dynamic soaring cycle search over the candidate aircraft."""
import json
import os
import time

import numpy as np

from sim.aircraft_models import by_name, candidates

OUT_PATH = os.path.join("sim", "out", "formstudy.json")


def static_numbers(ac):
    v_ms, sink = ac.min_sink
    return {"aspect_ratio": ac.aspect_ratio, "wing_loading": ac.wing_loading,
            "best_ld": ac.best_ld, "v_stall": ac.v_stall, "v_best_ld": ac.v_best_ld,
            "min_sink": sink}


DSOPT_PATH = os.path.join("sim", "out", "dsopt.json")
U10_REF = 8.0  # Charnock z0 for the min_wind profile is taken at this u10
U10_CROSS = 12.0


def _profiles(u10):
    from sim.dsopt import Profile
    from sim.seawind import Wind
    return {"log": Profile.from_wind(Wind(u10, from_deg=270.0)),
            "knee": Profile.from_wind(Wind(u10, from_deg=270.0, profile="swell_knee",
                                           knee_m=7.5))}


def _echo(info):
    if info.get("skipped"):
        print("%s %s %s: %s" % (info["name"], info["profile"], info["case"], info["message"]),
              flush=True)
        return
    print("%s %s %s: ok=%s near=%s u10=%.2f T=%.2f vg=%.2f (%.0f s)" % (
        info["name"], info["profile"], info["case"], info["success"], info["near_converged"],
        info["u10"], info["period"], info["ground_speed"], info["total_s"]), flush=True)


def run(names=None, n=40, h_min=1.0, seeds=(0, 1, 2, 3), stop_after=2, skip=("bird-wandering",),
        path=DSOPT_PATH):
    """Direct-collocation yardstick (sim.dsopt) for every candidate and both wind profiles."""
    from sim.dsopt import solve
    if names is None:
        acs = [a for a in candidates() if a.name not in skip]
    else:
        acs = [by_name(nm) for nm in names]
    prof_min, prof_x = _profiles(U10_REF), _profiles(U10_CROSS)
    runs = []
    t_start = time.time()
    for ac in acs:
        for pname in ("log", "knee"):
            for label, kw in (("min_wind_travel", dict(mode="min_wind", cycle="travel")),
                              ("min_wind_loiter", dict(mode="min_wind", cycle="loiter")),
                              ("max_speed_cross", dict(mode="max_speed", cycle="travel",
                                                       u10=U10_CROSS, chi=np.pi / 2))):
                prof = (prof_x if kw["mode"] == "max_speed" else prof_min)[pname]
                t0 = time.time()
                u_trav = next((r["u10"] for r in runs if (r["name"], r["profile"], r["case"])
                               == (ac.name, pname, "min_wind_travel") and r["success"]), None)
                if kw["mode"] == "max_speed" and u_trav is not None and u_trav > U10_CROSS:
                    # no periodic travelling cycle exists below u10_min: skip the solve
                    info = {"success": False, "near_converged": False, "skipped": True,
                            "message": "skipped: u10_min(travel) = %.2f m/s > %g m/s, no periodic "
                                       "cycle" % (u_trav, U10_CROSS),
                            "u10": U10_CROSS, "period": None, "ground_speed": None,
                            "speed_along_chi": None}
                else:
                    info = solve(ac, prof, n=n, h_min=h_min, seeds=seeds,
                                 stop_after=stop_after, **kw)
                info.update({"name": ac.name, "profile": pname, "case": label,
                             "total_s": time.time() - t0})
                runs.append(info)
                _echo(info)
    result = {"candidates": {ac.name: static_numbers(ac) for ac in acs}, "runs": runs,
              "total_runtime_s": time.time() - t_start,
              "settings": {"n_nodes": n, "h_min": h_min, "h_tip_min": 0.3, "knee_m": 7.5,
                           "u10_ref_for_z0": U10_REF, "u10_cross": U10_CROSS, "seeds": list(seeds),
                           "stop_after": stop_after, "collocation": "Hermite-Simpson",
                           "solver": "SLSQP"}}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    print(dsopt_table(result))
    return result


def dsopt_table(result):
    def cell(name, pname, case, key, fmt="%.1f"):
        r = next((r for r in result["runs"] if (r["name"], r["profile"], r["case"]) ==
                  (name, pname, case)), None)
        if r is None or r.get("skipped") or r[key] is None:
            return "-" if r is None else "none"
        return (fmt % r[key]) + ("" if r["success"] else "~" if r["near_converged"] else "*")

    head = ["candidate", "u10 travel log", "u10 travel knee", "u10 loiter log", "u10 loiter knee",
            "cross log (m/s)", "cross knee (m/s)", "T travel log (s)"]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for name in result["candidates"]:
        row = [name, cell(name, "log", "min_wind_travel", "u10"),
               cell(name, "knee", "min_wind_travel", "u10"),
               cell(name, "log", "min_wind_loiter", "u10"),
               cell(name, "knee", "min_wind_loiter", "u10"),
               cell(name, "log", "max_speed_cross", "speed_along_chi"),
               cell(name, "knee", "max_speed_cross", "speed_along_chi"),
               cell(name, "log", "min_wind_travel", "period")]
        lines.append("| " + " | ".join(row) + " |")
    lines.append("(~ = stopped at the iteration limit with violation < 1e-3; * = not converged)")
    return "\n".join(lines)


def table(result):
    u10s = sorted({r["u10"] for r in result["runs"]})
    head = (["name", "AR", "W/S", "L/D", "v_stall"] + ["gain@%g" % u for u in u10s]
            + ["vg@%g" % u for u in u10s])
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for name, s in result["candidates"].items():
        row = [name, "%.1f" % s["aspect_ratio"], "%.1f" % s["wing_loading"],
               "%.1f" % s["best_ld"], "%.1f" % s["v_stall"]]
        gains, vgs = [], []
        for u in u10s:
            r = next((r for r in result["runs"] if r["name"] == name and r["u10"] == u), None)
            gains.append("%.1f" % r["gain_per_cycle"] if r else "-")
            vgs.append("%.1f (%.1f)" % (r["ground_speed"], r["ground_speed_along_wind"])
                       if r else "-")
        lines.append("| " + " | ".join(row + gains + vgs) + " |")
    return "\n".join(lines)


if __name__ == "__main__":
    run()
