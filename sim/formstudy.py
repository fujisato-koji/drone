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
        path=DSOPT_PATH, tip_model="abs", z0_mode="fixed8", roughness="charnock", z0=None):
    """Direct-collocation yardstick (sim.dsopt) for every candidate and both wind profiles.

    tip_model, z0_mode and roughness are the K-007 switches of sim.dsopt (defaults = today).
    """
    from sim.dsopt import profile_for, solve, solve_min_wind
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
                elif kw["mode"] == "min_wind" and (z0_mode != "fixed8"
                                                   or roughness != "charnock"):
                    info = solve_min_wind(ac, z0_mode, roughness, z0, u10_ref=U10_REF, kind=pname,
                                          cycle=kw["cycle"], n=n, h_min=h_min, seeds=seeds,
                                          stop_after=stop_after, tip_model=tip_model)
                else:
                    if roughness != "charnock":
                        prof = profile_for(kw.get("u10", U10_REF), pname, roughness, z0)
                    info = solve(ac, prof, n=n, h_min=h_min, seeds=seeds,
                                 stop_after=stop_after, tip_model=tip_model, **kw)
                info.update({"name": ac.name, "profile": pname, "case": label,
                             "total_s": time.time() - t0})
                runs.append(info)
                _echo(info)
    result = {"candidates": {ac.name: static_numbers(ac) for ac in acs}, "runs": runs,
              "total_runtime_s": time.time() - t_start,
              "settings": {"n_nodes": n, "h_min": h_min, "h_tip_min": 0.3, "knee_m": 7.5,
                           "u10_ref_for_z0": U10_REF, "u10_cross": U10_CROSS, "seeds": list(seeds),
                           "stop_after": stop_after, "tip_model": tip_model,
                           "z0_mode": z0_mode, "roughness": roughness, "z0": z0,
                           "collocation": "Hermite-Simpson",
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


WAVE_PATH = os.path.join("sim", "out", "dsopt_wave.json")
WAVE_AIRCRAFT = ("baseline-2m", "albatross-3m")
WAVE_HS = (1.0, 2.0, 3.0)
WAVE_TP = (6.0, 8.0, 11.0)
WAVE_DIRS = (("along", 0.0), ("across", 90.0))
FLAT_VARIANTS = (  # label, tip_model, z0_mode, roughness, z0
    ("today (abs tip, z0 at u10=8)", "abs", "fixed8", "charnock", None),
    ("geometric tip, z0 at u10=8", "geometric", "fixed8", "charnock", None),
    ("abs tip, self-consistent z0", "abs", "self_consistent", "charnock", None),
    ("geometric tip, self-consistent z0 (corrected)", "geometric", "self_consistent",
     "charnock", None),
    ("geometric tip, z0 = 0.03 m", "geometric", "fixed8", "fixed_z0", 0.03))


def _row(info, **extra):
    keys = ("success", "near_converged", "violation", "u10", "period", "n_max", "z0",
            "outer_iterations", "outer_converged", "scaling_error_first_step", "z_min",
            "mu_max_deg", "ground_speed", "runtime_s", "m", "phi0", "theta_at_h_min_deg",
            "eta_over_a_at_h_min", "h_min_position", "u10_at_floor", "shelter_r_max", "shelter_s_c", "h_node_min", "h_mid_min", "h_min_achieved", "h_range", "message")
    r = {k: info.get(k) for k in keys}
    r.update(extra)
    return r


def _better(a, b):
    """True if info a beats b: converged first, then lower u10, else lower violation."""
    if b is None:
        return True
    if a["success"] != b["success"]:
        return a["success"]
    if a["success"]:
        return a["u10"] < b["u10"]
    return a["violation"] < b["violation"]


def wave_flat_task(job):
    """Flat-sea references of one aircraft: all K-007 variants (travel cycle, h_min 1 m)."""
    from sim.dsopt import solve_min_wind
    name, n, seeds = job
    ac = by_name(name)
    out = []
    for label, tip, zmode, rough, z0 in FLAT_VARIANTS:
        t0 = time.time()
        info = solve_min_wind(ac, zmode, rough, z0, cycle="travel", n=n, h_min=1.0, seeds=seeds,
                              stop_after=2, tip_model=tip)
        out.append(_row(info, aircraft=name, variant=label, tip_model=tip, z0_mode=zmode,
                        roughness=rough, total_s=time.time() - t0))
    return out


def wave_case_task(job):
    """One (aircraft, Tp, direction): Hs 1 to 3 m, each also started from the previous Hs."""
    from sim.dsopt import solve_min_wind
    name, tp, dlabel, to_deg, n, seeds, hs_list = job
    ac = by_name(name)
    lam = 9.81 * tp * tp / (2.0 * np.pi)
    rows, prev = [], None
    for hs in hs_list:
        t0 = time.time()
        wave = {"a": hs / 2.0, "wavelength": lam, "to_deg": to_deg}
        kw = dict(cycle="travel", n=n, h_min=1.0, tip_model="geometric",
                  z0_mode="self_consistent", wave=wave, return_x=True)
        cands = []
        if prev is not None:
            cands.append(solve_min_wind(ac, x0=prev, seeds=seeds[:2], stop_after=1, **kw))
        cands.append(solve_min_wind(ac, seeds=seeds, stop_after=3, **kw))
        best = None
        for c in cands:
            if _better(c[0], best[0] if best else None):
                best = c
        info = best[0]
        prev = best[1] if info["success"] else prev
        rows.append(_row(info, aircraft=name, hs=hs, tp=tp, direction=dlabel, to_deg=to_deg,
                         a=hs / 2.0, wavelength=lam, total_s=time.time() - t0))
        print("%s Tp=%g %s Hs=%g: ok=%s u10=%.2f T=%.2f m=%s %s (%.0f s)" % (
            name, tp, dlabel, hs, info["success"], info["u10"], info["period"], info["m"],
            info["h_min_position"], time.time() - t0), flush=True)
    return rows


def wave_table(result):
    lines = []
    for name in result["settings"]["aircraft"]:
        lines += ["", "### " + name, "",
                  "| flat-sea variant | u10_min (m/s) | T (s) | z0 (m) | converged |",
                  "|---|---|---|---|---|"]
        for r in result["flat_reference"]:
            if r["aircraft"] == name:
                lines.append("| %s | %.2f | %.2f | %.2e | %s |" % (
                    r["variant"], r["u10"], r["period"], r["z0"], "yes" if r["success"] else "no"))
        head = ["Hs", "Tp", "dir", "u10_min", "T", "m", "theta_hmin (deg)", "cos theta",
                "low turn at", "h_min", "n_max", "conv"]
        lines += ["", "| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
        for r in result["wave_cases"]:
            if r["aircraft"] != name:
                continue
            lines.append("| %g | %g | %s | %.2f | %.2f | %s | %.0f | %.2f | %s | %.2f | %.2f | %s |"
                         % (r["hs"], r["tp"], r["direction"], r["u10"], r["period"], r["m"],
                            r["theta_at_h_min_deg"], r["eta_over_a_at_h_min"],
                            r["h_min_position"], r["h_min_achieved"], r["n_max"],
                            ("yes" if r["success"] else ("near" if r["near_converged"] else "NO"))
                            + (" (u10 at floor)" if r.get("u10_at_floor") else "")))
    return "\n".join(lines)


def _solve_wave(ac, wave, n, seeds, stop_after, starts):
    """Best of the warm starts in `starts` (list of x) and, if none converged, cold seeds."""
    from sim.dsopt import solve_min_wind
    kw = dict(cycle="travel", n=n, h_min=1.0, tip_model="geometric", z0_mode="self_consistent",
              wave=wave, return_x=True)
    best = None
    for x0 in starts:
        c = solve_min_wind(ac, x0=x0, seeds=seeds[:2], stop_after=1, **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    if best is None or not best[0]["success"]:
        c = solve_min_wind(ac, seeds=seeds, stop_after=stop_after, **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    return best


def wave_chain_task(job):
    """One (aircraft, Tp, direction), midpoint constraints: Hs 1 to 3 with decay (1, 1), then the
    same cases with the measured decay, each warm-started from the (1, 1) solution."""
    from sim.record import value
    name, tp, dlabel, to_deg, n, seeds, hs_list = job
    ac = by_name(name)
    lam = 9.81 * tp * tp / (2.0 * np.pi)
    measured = (value("lit.wave_induced_decay_u"), value("lit.wave_induced_decay_w"))
    out = {"11": [], "measured": []}
    xs = {}
    for label, decay in (("11", (1.0, 1.0)), ("measured", measured)):
        prev = None
        for hs in hs_list:
            t0 = time.time()
            wave = {"a": hs / 2.0, "wavelength": lam, "to_deg": to_deg, "decay": decay}
            starts = [x for x in (xs.get(("11", hs)) if label == "measured" else None, prev)
                      if x is not None]
            info, x, _ = _solve_wave(ac, wave, n, seeds, 2, starts)
            if info["success"]:
                prev = x
                xs[(label, hs)] = x
            out[label].append(_row(info, aircraft=name, hs=hs, tp=tp, direction=dlabel,
                                   to_deg=to_deg, a=hs / 2.0, wavelength=lam, decay=list(decay),
                                   total_s=time.time() - t0))
            print("%s Tp=%g %s Hs=%g decay=%s: ok=%s u10=%.2f T=%.2f m=%s %s hmin=%.2f (%.0f s)" % (
                name, tp, dlabel, hs, label, info["success"], info["u10"], info["period"],
                info["m"], info["h_min_position"], info["h_min_achieved"], time.time() - t0),
                flush=True)
    return out


def wave_steep_task(job):
    """Steep single wave, a k = 0.3, along the wind, measured decay: shelter off, then on."""
    from sim.dsopt import solve_min_wind
    from sim.record import value
    name, lam, n, seeds = job
    ac = by_name(name)
    k = 2.0 * np.pi / lam
    a = 0.3 / k
    measured = (value("lit.wave_induced_decay_u"), value("lit.wave_induced_decay_w"))
    rows, x_off = [], []
    for shelter in (False, True):
        t0 = time.time()
        wave = {"a": a, "wavelength": lam, "to_deg": 0.0, "decay": measured, "shelter": shelter}
        info, x, _ = _solve_wave(ac, wave, n, seeds, 2, x_off)
        if not shelter and info["success"]:
            x_off = [x]
        if shelter:  # also a cold search: the sheltered optimum may be a different cycle
            c = solve_min_wind(ac, seeds=seeds, stop_after=2, cycle="travel", n=n, h_min=1.0,
                               tip_model="geometric", z0_mode="self_consistent", wave=wave,
                               return_x=True)
            if _better(c[0], info):
                info = c[0]
        rows.append(_row(info, aircraft=name, wavelength=lam, a=a, steepness=0.3,
                         shelter=shelter, decay=list(measured), total_s=time.time() - t0))
        print("%s lam=%g shelter=%s: ok=%s u10=%.2f T=%.2f theta=%.0f %s hmin(node/mid)=%.2f/%.2f "
              "(%.0f s)" % (name, lam, shelter, info["success"], info["u10"], info["period"],
                            info["theta_at_h_min_deg"], info["h_min_position"],
                            info["h_node_min"], info["h_mid_min"], time.time() - t0), flush=True)
    return rows


def _table_rows(rows, flag=" (u10 at floor)"):
    head = ["Hs", "Tp", "dir", "u10_min", "T", "m", "theta_hmin (deg)", "cos theta", "low turn at",
            "h_min node/mid", "n_max", "conv"]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for r in rows:
        lines.append("| %g | %g | %s | %.2f | %.2f | %s | %.0f | %.2f | %s | %.2f/%.2f | %.2f | %s |"
                     % (r["hs"], r["tp"], r["direction"], r["u10"], r["period"], r["m"],
                        r["theta_at_h_min_deg"], r["eta_over_a_at_h_min"], r["h_min_position"],
                        r["h_node_min"], r["h_mid_min"], r["n_max"],
                        ("yes" if r["success"] else ("near" if r["near_converged"] else "NO"))
                        + (flag if r.get("u10_at_floor") else "")))
    return "\n".join(lines)


def steep_table(rows):
    head = ["aircraft", "lambda", "a", "shelter", "u10_min", "T", "m", "theta_hmin (deg)",
            "low turn at", "h_min node/mid", "n_max", "s_c", "r_max", "conv"]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for r in rows:
        lines.append("| %s | %g | %.2f | %s | %.2f | %.2f | %s | %.0f | %s | %.2f/%.2f | %.2f | %s | %s | %s |"
                     % (r["aircraft"], r["wavelength"], r["a"], "on" if r["shelter"] else "off",
                        r["u10"], r["period"], r["m"], r["theta_at_h_min_deg"],
                        r["h_min_position"], r["h_node_min"], r["h_mid_min"], r["n_max"],
                        "-" if r["shelter_s_c"] is None else "%.3f" % r["shelter_s_c"],
                        "-" if r["shelter_r_max"] is None else "%.3f" % r["shelter_r_max"],
                        ("yes" if r["success"] else ("near" if r["near_converged"] else "NO"))
                        + (" (floor)" if r.get("u10_at_floor") else "")))
    return "\n".join(lines)


def wave_study2(path=WAVE_PATH, n=30, seeds=(0, 1, 2, 3, 4, 5), workers=7, names=WAVE_AIRCRAFT,
                hs_list=WAVE_HS, tps=WAVE_TP, dirs=WAVE_DIRS, lams=(20.0, 40.0)):
    """Second pass: midpoint clearance, measured decay, steep-wave shelter test. Appends to path."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    jobs_steep = [(nm, lam, n, seeds) for nm in names for lam in lams]
    jobs_chain = [(nm, tp, dl, td, n, seeds, hs_list) for nm in names for tp in tps
                  for dl, td in dirs]
    with Pool(workers) as pool:
        steep = pool.map_async(wave_steep_task, jobs_steep, chunksize=1)
        chains = pool.map(wave_chain_task, jobs_chain, chunksize=1)
        steep = steep.get()
    with open(path, encoding="utf-8") as fh:
        result = json.load(fh)
    if "wave_cases" in result:  # keep the first table under its own key
        result["pass1_no_midpoint"] = {"meta": result.pop("meta"),
                                       "settings": result.pop("settings"),
                                       "wave_cases": result.pop("wave_cases"),
                                       "runtime_s": result.pop("runtime_s")}
    here = os.path.dirname(os.path.abspath(__file__))
    files = [os.path.join(here, f) for f in ("dsopt.py", "formstudy.py", "aircraft.py",
                                             "aircraft_models.py", "seawind.py")]
    files.append(record.VALUES_PATH)
    result["meta"] = record.meta("python -m sim.dsopt --wave-study2", files)
    result["settings_pass2"] = {
        "n_nodes": n, "seeds": list(seeds), "midpoint_constraints": True, "tip_model": "geometric",
        "z0_mode": "self_consistent", "u10_floor": 0.1,
        "decay_measured": [record.value("lit.wave_induced_decay_u"),
                           record.value("lit.wave_induced_decay_w")],
        "shelter": {
                    "F": record.value("env.shelter_factor"), "s_c": "sim.seawind.separation_steepness",
                    "D": record.value("env.shelter_decay_kz"),
                    "clip": "difference of two softplus, width 0.05 in (sigma - s_c)/s_c",
                    "gate": "sigmoid(urel / 0.3 m/s)"},
        "steep": {"ak": 0.3, "wavelengths": list(lams), "to_deg": 0.0}}
    result["pass2_midpoint_decay11"] = [r for c in chains for r in c["11"]]
    result["pass2_midpoint_decay_measured"] = [r for c in chains for r in c["measured"]]
    result["steep_wave"] = [r for s_ in steep for r in s_]
    result["runtime_pass2_s"] = time.time() - t_start
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    for nm in names:
        for key, title in (("pass2_midpoint_decay11", "midpoint constraints, decay (1, 1)"),
                           ("pass2_midpoint_decay_measured", "midpoint constraints, measured decay")):
            print("\n### %s: %s\n" % (nm, title))
            print(_table_rows([r for r in result[key] if r["aircraft"] == nm]))
    print("\n### steep waves (a k = 0.3, along the wind, measured decay)\n")
    print(steep_table(result["steep_wave"]))
    print("runtime pass 2: %.0f s" % result["runtime_pass2_s"])
    return result


def wave_study(path=WAVE_PATH, n=30, seeds=(0, 1, 2, 3, 4, 5, 6, 7), workers=4, names=WAVE_AIRCRAFT,
               hs_list=WAVE_HS, tps=WAVE_TP, dirs=WAVE_DIRS):
    """Flat-sea references (K-007 corrections) and wave-synchronized travel cycles over one wave."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    jobs_flat = [(nm, n, seeds[:3]) for nm in names]
    jobs_wave = [(nm, tp, dl, td, n, seeds, hs_list) for nm in names for tp in tps
                 for dl, td in dirs]
    with Pool(workers) as pool:
        waves = pool.map_async(wave_case_task, jobs_wave, chunksize=1)
        flats = pool.map(wave_flat_task, jobs_flat, chunksize=1)
        waves = waves.get()
    here = os.path.dirname(os.path.abspath(__file__))
    files = [os.path.join(here, f) for f in ("dsopt.py", "formstudy.py", "aircraft.py",
                                             "aircraft_models.py", "seawind.py")]
    files.append(record.VALUES_PATH)
    result = {"meta": record.meta("python -m sim.dsopt --wave-study", files),
              "settings": {"aircraft": list(names), "n_nodes": n, "seeds": list(seeds),
                           "hs": list(hs_list), "tp": list(tps),
                           "directions_math_deg": {d: a for d, a in dirs},
                           "tip_model": "geometric", "z0_mode": "self_consistent",
                           "h_min": 1.0, "h_tip_min": 0.3, "cycle": "travel",
                           "frame": "wind toward +x; wave to_deg counter-clockwise from +x",
                           "theta_convention": "eta = a cos(theta); 0 = crest, +-180 = trough; "
                                               "theta in (0, 180) is the forward face"},
              "flat_reference": [r for f in flats for r in f],
              "wave_cases": [r for w in waves for r in w],
              "runtime_s": time.time() - t_start}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    print(wave_table(result))
    print("runtime %.0f s" % result["runtime_s"])
    return result


if __name__ == "__main__":
    run()
