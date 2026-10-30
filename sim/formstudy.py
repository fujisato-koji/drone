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


CONSTRAINT_PATH = os.path.join("sim", "out", "dsopt_constraints.json")
STUDY_AIRCRAFT = ("baseline-2m", "albatross-3m")
# study A: label, solve keywords (all on the corrected settings: geometric tip, self-consistent z0)
STUDY_A = (
    ("(i) as today", dict()),
    ("(ii) midpoints", dict(midpoints=True)),
    ("(iii) mid + rates 150 deg/s, 2/s", dict(midpoints=True, rate_limits=True,
                                              bank_rate_max_deg_s=150.0, cl_rate_max_per_s=2.0)),
    ("(iv) mid + rates 100 deg/s, 2/s", dict(midpoints=True, rate_limits=True,
                                             bank_rate_max_deg_s=100.0, cl_rate_max_per_s=2.0)),
    ("(iv) mid + rates 60 deg/s, 2/s", dict(midpoints=True, rate_limits=True,
                                            bank_rate_max_deg_s=60.0, cl_rate_max_per_s=2.0)),
    ("(iv) mid + rates 30 deg/s, 2/s", dict(midpoints=True, rate_limits=True,
                                            bank_rate_max_deg_s=30.0, cl_rate_max_per_s=2.0)),
    ("(v) mid + rates 150, 2/s + n_min 0", dict(midpoints=True, rate_limits=True, n_min=0.0,
                                                bank_rate_max_deg_s=150.0, cl_rate_max_per_s=2.0)),
)
STUDY_B_MARGINS = ((1.0, 1.0, 1.0, 1.1), (2.0, 0.8, 0.9, 1.2), (3.0, 0.7, 0.85, 1.3))
STATS = ("success", "near_converged", "violation", "u10", "period", "z0", "outer_converged",
         "n_max_node", "n_min_node", "bank_rate_max_deg_s", "cl_rate_max_per_s",
         "control_seam_jump", "mu_max_deg", "replay_tip_min", "replay_h_min", "replay_closure",
         "speed_along_chi", "ground_speed", "m", "runtime_s")


def _stats(info, **extra):
    r = {k: info.get(k) for k in STATS}
    r.update(extra)
    return r


def constraint_task_a(job):
    """One aircraft through the study-A variants, each also warm-started from the previous."""
    from sim.dsopt import solve_min_wind
    name, n, seeds = job
    ac = by_name(name)
    rows, prev = [], None
    for label, kw in STUDY_A:
        t0 = time.time()
        base = dict(cycle="travel", n=n, h_min=1.0, tip_model="geometric",
                    z0_mode="self_consistent", return_x=True, **kw)
        cands = []
        if prev is not None:
            cands.append(solve_min_wind(ac, x0=prev, seeds=seeds[:2], stop_after=1, **base))
        cands.append(solve_min_wind(ac, seeds=seeds, stop_after=3, **base))
        best = None
        for c in cands:
            if _better(c[0], best[0] if best else None):
                best = c
        info = best[0]
        if info["success"] and label.startswith(("(ii)", "(iii)")):
            prev = best[1]
        rows.append(_stats(info, aircraft=name, variant=label, total_s=time.time() - t0))
        print("A %s %s: ok=%s u10=%.3f T=%.3f bank=%.0f cl_rate=%.2f (%.0f s)" % (
            name, label, info["success"], info["u10"], info["period"],
            info["bank_rate_max_deg_s"], info["cl_rate_max_per_s"], time.time() - t0), flush=True)
    return rows


def constraint_task_b(job):
    """Control-library margins: max speed toward chi at fixed u10."""
    from sim.dsopt import profile_for, solve
    name, n, seeds, u10, chi_deg = job
    ac = by_name(name)
    prof = profile_for(u10)
    rows = []
    for h_min, n_frac, mu_frac, v_factor in STUDY_B_MARGINS:
        t0 = time.time()
        info = solve(ac, prof, "max_speed", "travel", n, u10=u10, chi=np.radians(chi_deg),
                     h_min=h_min, v_factor=v_factor, tip_model="geometric", midpoints=True,
                     rate_limits=True, bank_rate_max_deg_s=150.0, cl_rate_max_per_s=2.0,
                     n_min=0.0, n_frac=n_frac, mu_frac=mu_frac, seeds=seeds, stop_after=3)
        rows.append(_stats(info, aircraft=name, h_min=h_min, n_frac=n_frac, mu_frac=mu_frac,
                           v_factor=v_factor, u10_fixed=u10, chi_deg=chi_deg,
                           total_s=time.time() - t0))
        print("B %s margins=%s: ok=%s v_chi=%.2f viol=%.1e (%.0f s)" % (
            name, (h_min, n_frac, mu_frac, v_factor), info["success"],
            info["speed_along_chi"], info["violation"], time.time() - t0), flush=True)
    return rows


def _record_files():
    from sim import record
    here = os.path.dirname(os.path.abspath(__file__))
    files = [os.path.join(here, f) for f in ("dsopt.py", "formstudy.py", "aircraft.py",
                                             "aircraft_models.py", "seawind.py")]
    files.append(record.VALUES_PATH)
    return files


def _md(head, rows):
    return "\n".join(["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
                     + ["| " + " | ".join(r) + " |" for r in rows])


def table_a(rows):
    lines = []
    for nm in dict.fromkeys(r["aircraft"] for r in rows):
        lines += ["", "### study A: " + nm, ""]
        body = []
        for r in (r for r in rows if r["aircraft"] == nm):
            body.append([r["variant"], "%.2f" % r["u10"], "%.2f" % r["period"],
                         "%.0f" % r["bank_rate_max_deg_s"], "%.2f" % r["cl_rate_max_per_s"],
                         "%.2f" % r["n_max_node"], "%.3f" % r["replay_tip_min"],
                         "yes" if r["success"] else "NO"])
        lines.append(_md(["variant", "u10_min", "T (s)", "max bank rate (deg/s)",
                          "max CL rate (1/s)", "max n", "min tip, dense (m)", "conv"], body))
    return "\n".join(lines)


def table_b(rows):
    body = []
    for r in rows:
        body.append([r["aircraft"], "%g, %g, %g, %g" % (r["h_min"], r["n_frac"], r["mu_frac"],
                                                        r["v_factor"]),
                     "%.2f" % r["speed_along_chi"], "%.2f" % r["period"] if r["period"] else "-",
                     "yes" if r["success"] else "NO (viol %.1e)" % r["violation"]])
    return _md(["aircraft", "(h_min, n_frac, mu_frac, v_factor)", "speed along chi (m/s)",
                "T (s)", "conv"], body)


def constraint_study(path=CONSTRAINT_PATH, n=30, seeds=(0, 1, 2, 3, 4, 5), workers=4,
                     names=STUDY_AIRCRAFT, u10_b=13.0, chi_deg=20.0):
    """Flat sea: study A (minimum wind under the new constraints) and B (control-library margins)."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    with Pool(workers) as pool:
        a = pool.map_async(constraint_task_a, [(nm, n, seeds) for nm in names], chunksize=1)
        b = pool.map(constraint_task_b, [(nm, n, seeds + (6, 7), u10_b, chi_deg) for nm in names],
                     chunksize=1)
        a = a.get()
    result = {"meta": record.meta("python -m sim.dsopt --constraint-study", _record_files()),
              "settings": {"n_nodes": n, "seeds": list(seeds), "tip_model": "geometric",
                           "z0_mode": "self_consistent", "roughness": "charnock", "h_min": 1.0,
                           "study_b": {"u10": u10_b, "chi_deg": chi_deg,
                                       "chi_frame": "optimizer frame, wind toward +x",
                                       "margins": [list(m) for m in STUDY_B_MARGINS]},
                           "replay": "RK4 from each node state, 20 steps per interval, controls "
                                     "linear; tip = z - (b/2) sin|mu| cos(gamma)"},
              "study_a": [r for x in a for r in x], "study_b": [r for x in b for r in x],
              "runtime_s": time.time() - t_start}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    print(table_a(result["study_a"]))
    print("\n### study B\n")
    print(table_b(result["study_b"]))
    print("runtime %.0f s" % result["runtime_s"])
    return result


def _solve_wave3(ac, wave, n, seeds, starts, **extra):
    """Best of the warm starts (list of x) and, if none converged, the cold seeds."""
    from sim.dsopt import solve_min_wind
    kw = dict(cycle="travel", n=n, h_min=1.0, tip_model="geometric", z0_mode="self_consistent",
              wave=wave, return_x=True, midpoints=True, rate_limits=True, n_min=0.0,
              bank_rate_max_deg_s=150.0, cl_rate_max_per_s=2.0, m_search=(-1, 0, 1), **extra)
    best = None
    for x0 in starts:
        c = solve_min_wind(ac, x0=x0, seeds=seeds[:2], stop_after=1, **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    if best is None or not best[0]["success"]:
        c = solve_min_wind(ac, seeds=seeds, stop_after=1, **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    return best


def wave_chain3_task(job):
    """One (aircraft, Tp, w_scale): across chain, then along chain started from the rotated across
    solutions, then the across chain again started from the rotated along solutions."""
    from sim.dsopt import rotate_start
    from sim.record import value
    name, tp, w_scale, n, seeds, hs_list, rotate = job
    ac = by_name(name)
    lam = 9.81 * tp * tp / (2.0 * np.pi)
    decay = (value("lit.wave_induced_decay_u"), value("lit.wave_induced_decay_w"))
    dirs = {"across": 90.0, "along": 0.0}
    res = {d: {} for d in dirs}   # direction -> hs -> (info, x, col)
    rows = []

    def sweep(dlabel, other):
        prev = None
        for hs in hs_list:
            t0 = time.time()
            wave = {"a": hs / 2.0, "wavelength": lam, "to_deg": dirs[dlabel], "decay": decay,
                    "w_scale": w_scale, "u_scale": 1.0}
            starts = []
            if prev is not None:
                starts.append(prev)
            if rotate and other is not None and hs in res[other] and res[other][hs][0]["success"]:
                _, xo, colo = res[other][hs]
                starts.append(rotate_start(colo, xo, dirs[dlabel] - dirs[other]))
            c = _solve_wave3(ac, wave, n, seeds, starts)
            old = res[dlabel].get(hs)
            if old is None or _better(c[0], old[0]):
                res[dlabel][hs] = c
            c = res[dlabel][hs]
            if c[0]["success"]:
                prev = c[1]
            print("%s Tp=%g w=%g %s Hs=%g: ok=%s u10=%.2f T=%.2f m=%s %s (%.0f s)" % (
                name, tp, w_scale, dlabel, hs, c[0]["success"], c[0]["u10"], c[0]["period"],
                c[0]["m"], c[0]["h_min_position"], time.time() - t0), flush=True)

    sweep("across", None)
    sweep("along", "across")
    if rotate:
        sweep("across", "along")
    for dlabel in dirs:
        for hs in hs_list:
            info = res[dlabel][hs][0]
            rows.append(_row(info, aircraft=name, hs=hs, tp=tp, direction=dlabel,
                             to_deg=dirs[dlabel], a=hs / 2.0, wavelength=lam, w_scale=w_scale,
                             decay=list(decay), **{k: info.get(k) for k in (
                                 "bank_rate_max_deg_s", "cl_rate_max_per_s", "replay_tip_min",
                                 "replay_h_min", "n_min_node", "n_max_node")}))
    return rows


def wave_study3(path=CONSTRAINT_PATH, n=30, seeds=(0, 1, 2), workers=6, names=WAVE_AIRCRAFT,
                hs_list=WAVE_HS, tps=WAVE_TP, w_scales=(1.0, 0.6), rotate=True):
    """Third wave pass (Q-059): the 36-case grid of wave_study2 with midpoints, rate limits 150 deg/s
    and 2/s, n >= 0, the theta pin, m_search (-1, 0, 1), rotated starts and the measured decay, for
    each w_scale. Added to path under "wave_pass3"."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    jobs = [(nm, tp, w, n, seeds, hs_list, rotate) for w in w_scales for nm in names for tp in tps]
    with Pool(workers) as pool:
        out = pool.map(wave_chain3_task, jobs, chunksize=1)
    rows = [r for o in out for r in o]
    result = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            result = json.load(fh)
    meta = record.meta("python -m sim.dsopt --wave-study3", _record_files())
    result["meta_wave_pass3"] = meta
    if "meta" not in result:
        result["meta"] = meta
    result["wave_pass3_settings"] = {
        "n_nodes": n, "seeds": list(seeds), "midpoints": True, "rate_limits_deg_s_and_per_s":
        [150.0, 2.0], "n_min": 0.0, "phase_pin": "theta", "m_search": [-1, 0, 1],
        "rotate_starts": rotate, "decay": "measured (lit.wave_induced_decay_u, _w)",
        "w_scales": list(w_scales), "u_scale": 1.0, "tip_model": "geometric",
        "z0_mode": "self_consistent", "u10_floor": 0.1}
    result["wave_pass3"] = rows
    result["wave_pass3_runtime_s"] = time.time() - t_start
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    print(wave3_tables(rows, w_scales))
    print("runtime pass 3: %.0f s" % result["wave_pass3_runtime_s"])
    return result


def wave3_tables(rows, w_scales):
    lines = []
    for w in w_scales:
        sel = [r for r in rows if r["w_scale"] == w]
        floor = [r for r in sel if r.get("u10_at_floor") and r["success"]]
        bad = [r for r in sel if not r["success"]]
        lines += ["", "### w_scale %g: %d of %d cases at the u10 floor, %d not converged" % (
            w, len(floor), len(sel), len(bad)), ""]
        rest = [r for r in sel if not (r.get("u10_at_floor") and r["success"])]
        if rest:
            lines.append(_table_rows(rest))
    return "\n".join(lines)


STUDY_A2 = tuple(("k %.2f, CL rate %g/s" % (k, r), dict(
    midpoints=True, rate_limits=True, n_min=0.0, rate_model="pb2v", roll_pb2v=k,
    cl_rate_max_per_s=r)) for k, r in ((0.07, 1.0), (0.08, 1.0), (0.10, 1.0), (0.08, 0.5)))
STATS2 = STATS + ("bank_rate_peak_deg_s", "bank_rate_limit_at_peak_deg_s",
                  "bank_rate_over_limit_max", "pb2v_used_max")


def constraint_task_a2(job):
    from sim.dsopt import solve_min_wind
    name, n, seeds = job
    ac = by_name(name)
    rows, prev = [], None
    for label, kw in STUDY_A2:
        t0 = time.time()
        base = dict(cycle="travel", n=n, h_min=1.0, tip_model="geometric",
                    z0_mode="self_consistent", return_x=True, **kw)
        cands = []
        if prev is not None:
            cands.append(solve_min_wind(ac, x0=prev, seeds=seeds[:2], stop_after=1, **base))
        cands.append(solve_min_wind(ac, seeds=seeds, stop_after=3, **base))
        best = None
        for c in cands:
            if _better(c[0], best[0] if best else None):
                best = c
        info = best[0]
        rows.append({**{k: info.get(k) for k in STATS2}, "aircraft": name, "variant": label,
                     "k": kw["roll_pb2v"], "cl_rate_limit": kw["cl_rate_max_per_s"],
                     "total_s": time.time() - t0})
        print("A2 %s %s: ok=%s u10=%.3f T=%.3f peak %.0f of %.0f deg/s, cl_rate=%.2f (%.0f s)" % (
            name, label, info["success"], info["u10"], info["period"],
            info["bank_rate_peak_deg_s"], info["bank_rate_limit_at_peak_deg_s"],
            info["cl_rate_max_per_s"], time.time() - t0), flush=True)
    return rows


def constraint_task_b2(job):
    from sim.dsopt import profile_for, solve
    name, n, seeds, u10, chi_deg = job
    ac = by_name(name)
    prof = profile_for(u10)
    rows = []
    for h_min, n_frac, mu_frac, v_factor in STUDY_B_MARGINS:
        t0 = time.time()
        info = solve(ac, prof, "max_speed", "travel", n, u10=u10, chi=np.radians(chi_deg),
                     h_min=h_min, v_factor=v_factor, tip_model="geometric", midpoints=True,
                     rate_limits=True, rate_model="pb2v", roll_pb2v=0.08, cl_rate_max_per_s=1.0,
                     n_min=0.0, n_frac=n_frac, mu_frac=mu_frac, seeds=seeds, stop_after=3)
        rows.append({**{k: info.get(k) for k in STATS2}, "aircraft": name, "h_min": h_min,
                     "n_frac": n_frac, "mu_frac": mu_frac, "v_factor": v_factor,
                     "u10_fixed": u10, "chi_deg": chi_deg, "total_s": time.time() - t0})
        print("B2 %s margins=%s: ok=%s v_chi=%.2f viol=%.1e (%.0f s)" % (
            name, (h_min, n_frac, mu_frac, v_factor), info["success"],
            info["speed_along_chi"], info["violation"], time.time() - t0), flush=True)
    return rows


def table_a2(rows):
    lines = []
    for nm in dict.fromkeys(r["aircraft"] for r in rows):
        lines += ["", "### study A2: " + nm, ""]
        body = [[r["variant"], "%.2f" % r["u10"], "%.2f" % r["period"],
                 "%.0f of %.0f" % (r["bank_rate_peak_deg_s"], r["bank_rate_limit_at_peak_deg_s"]),
                 "%.3f" % r["pb2v_used_max"], "%.2f" % r["cl_rate_max_per_s"],
                 "yes" if r["success"] else "NO"] for r in rows if r["aircraft"] == nm]
        lines.append(_md(["variant", "u10_min", "T (s)", "bank rate peak of limit there (deg/s)",
                          "max p b / 2V", "max CL rate (1/s)", "conv"], body))
    return "\n".join(lines)


def constraint_study2(path=CONSTRAINT_PATH, n=30, seeds=(0, 1, 2, 3, 4, 5), workers=4,
                      names=STUDY_AIRCRAFT, u10_b=13.0, chi_deg=20.0):
    """Studies A2 (minimum wind, roll rate p b / 2V <= k) and B2 (margins), added to path."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    with Pool(workers) as pool:
        a = pool.map_async(constraint_task_a2, [(nm, n, seeds) for nm in names], chunksize=1)
        b = pool.map(constraint_task_b2, [(nm, n, seeds + (6, 7), u10_b, chi_deg) for nm in names],
                     chunksize=1)
        a = a.get()
    with open(path, encoding="utf-8") as fh:
        result = json.load(fh)
    result["meta_study2"] = record.meta("python -m sim.dsopt --constraint-study2", _record_files())
    result["settings_study2"] = {
        "rate_model": "pb2v: |dmu| <= (2 k V / b) dt, V the slower node of the interval",
        "n_min": 0.0, "midpoints": True, "n_nodes": n, "seeds": list(seeds),
        "b2": {"k": 0.08, "cl_rate": 1.0, "u10": u10_b, "chi_deg": chi_deg}}
    result["study_a2"] = [r for x in a for r in x]
    result["study_b2"] = [r for x in b for r in x]
    result["runtime_study2_s"] = time.time() - t_start
    with open(path, "w", newline="\n") as fh:
        json.dump(result, fh, indent=1)
        fh.write("\n")
    print(table_a2(result["study_a2"]))
    print("\n### study B2 (pb2v k 0.08, CL rate 1.0)\n")
    print(table_b(result["study_b2"]))
    print("runtime %.0f s" % result["runtime_study2_s"])
    return result


CLEARANCE_PATH = os.path.join("sim", "out", "dsopt_clearance.json")


def clearance_study(path=CLEARANCE_PATH, floors=((1.5, 0.3), (1.0, 0.3), (1.0, 0.1), (0.5, 0.1), (0.3, 0.05),
                                                 (1.5, None), (0.5, None), (0.1, None))):
    """Minimum wind against the height floor and the wingtip clearance, COARE 3.5 roughness.

    The Sachs albatross model (P45, no speed floor) and albatross-3m (speed floor 1.1 Vs),
    min_wind travel, geometric wingtip, self-consistent roughness, n = 40, heights and the tip
    held at the midpoints as well (K-011 found the tip dipping between nodes). A tip clearance of
    None switches the wingtip constraint off (h_tip_min -50 m); the Sachs model is also run with
    the literature's z0 of 0.03 m for those cases, for comparison with P45.
    """
    from sim.dsopt import sachs_albatross, solve_min_wind
    from sim import record
    rows = []
    for name, vf in (("sachs-bird", 0.0), ("albatross-3m", 1.1)):
        ac = sachs_albatross()[0] if name == "sachs-bird" else by_name(name)
        for h_min, tip in floors:
            roughs = [("coare35", None)]
            if name == "sachs-bird" and tip is None:
                roughs.append(("fixed_z0", 0.03))
            for rough, z0 in roughs:
                t0 = time.time()
                info = solve_min_wind(ac, "self_consistent" if z0 is None else "fixed8", rough, z0,
                                      u10_ref=10.0, kind="log", cycle="travel", n=40, h_min=h_min,
                                      h_tip_min=-50.0 if tip is None else tip, seeds=(1, 0, 2, 3),
                                      stop_after=2, tip_model="geometric", v_factor=vf, midpoints=True)
                rows.append({"aircraft": name, "roughness": rough, "z0": z0, "h_min": h_min,
                             "h_tip_min": tip, "u10": info["u10"], "success": bool(info["success"]),
                             "period": info["period"], "z_min": info["z_min"],
                             "runtime_s": time.time() - t0})
                print(rows[-1], flush=True)
    paths = ["sim/formstudy.py", "sim/dsopt.py", "sim/aircraft.py", "sim/aircraft_models.py",
             "sim/seawind.py", "record/values.yaml"]
    out = {"meta": record.meta("python -c \"from sim import formstudy; formstudy.clearance_study()\"", paths),
           "rows": rows}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    return out


WINDDIFF_PATH = os.path.join("sim", "out", "dsopt_wind_difference.json")


def wind_difference_study(path=WINDDIFF_PATH):
    """Wind difference between the lowest and the highest point of the minimum-wind travel cycle.

    The Sachs albatross model (with the wingtip held 0.3 m clear, without it at a 0.1 m floor, and
    with the literature's z0 of 0.03 m), albatross-3m and baseline-2m, COARE 3.5 unless stated,
    geometric wingtip, heights held at the midpoints, n = 40.
    """
    from sim.dsopt import sachs_albatross, solve_min_wind
    from sim.seawind import Wind
    from sim import record
    cases = [("sachs-bird", "coare35", None, 1.5, 0.3, 0.0), ("sachs-bird", "coare35", None, 0.1, None, 0.0),
             ("sachs-bird", "fixed_z0", 0.03, 1.5, 0.3, 0.0), ("albatross-3m", "coare35", None, 1.0, 0.3, 1.1),
             ("baseline-2m", "coare35", None, 1.0, 0.3, 1.1)]
    rows = []
    for name, rough, z0f, h_min, tip, vf in cases:
        ac = sachs_albatross()[0] if name == "sachs-bird" else by_name(name)
        info = solve_min_wind(ac, "self_consistent" if z0f is None else "fixed8", rough, z0f, u10_ref=10.0,
                              kind="log", cycle="travel", n=40, h_min=h_min,
                              h_tip_min=-50.0 if tip is None else tip, seeds=(1, 0, 2, 3), stop_after=2,
                              tip_model="geometric", v_factor=vf, midpoints=True)
        u = info["u10"]
        z0 = z0f if z0f is not None else Wind(u, 270.0, roughness="coare35").z0
        speed = lambda z: u * np.log(z / z0) / np.log(10.0 / z0)
        lo, hi = speed(info["z_min"]), speed(info["z_max"])
        rows.append({"aircraft": name, "roughness": rough, "z0": z0, "h_min": h_min, "h_tip_min": tip,
                     "lift_to_drag": ac.best_ld, "u10": u, "success": bool(info["success"]),
                     "z_low": info["z_min"], "z_high": info["z_max"], "wind_low": lo, "wind_high": hi,
                     "wind_difference": hi - lo, "period": info["period"]})
        print(rows[-1], flush=True)
    paths = ["sim/formstudy.py", "sim/dsopt.py", "sim/aircraft.py", "sim/aircraft_models.py",
             "sim/seawind.py", "record/values.yaml"]
    out = {"meta": record.meta("python -c \"from sim import formstudy; formstudy.wind_difference_study()\"", paths),
           "rows": rows}
    with open(path, "w", newline=chr(10)) as fh:
        json.dump(out, fh, indent=1)
        fh.write(chr(10))
    return out


GROUND_MOTOR_PATH = os.path.join("sim", "out", "dsopt_ground_motor.json")
GM_CLEARANCE_FLOORS = ((1.5, 0.3), (1.0, 0.3), (1.0, 0.1), (0.5, 0.1), (0.3, 0.05),
                       (1.5, None), (0.5, None), (0.1, None))
# wind_difference_study cases: (name, roughness, z0, h_min, tip, v_factor)
GM_WINDDIFF_CASES = (("sachs-bird", "coare35", None, 1.5, 0.3, 0.0),
                     ("sachs-bird", "coare35", None, 0.1, None, 0.0),
                     ("sachs-bird", "fixed_z0", 0.03, 1.5, 0.3, 0.0),
                     ("albatross-3m", "coare35", None, 1.0, 0.3, 1.1),
                     ("baseline-2m", "coare35", None, 1.0, 0.3, 1.1))
GM_MOTOR_SETTINGS = dict(h_min=1.0, h_tip_min=0.3, v_factor=1.1, tip_model="geometric",
                         midpoints=True, rate_limits=True, rate_model="pb2v", roll_pb2v=0.08,
                         cl_rate_max_per_s=1.0, n_min=0.0)


def _gm_min_wind(name, rough, z0f, h_min, tip, vf, ge, n=40):
    from sim.dsopt import sachs_albatross, solve_min_wind
    ac = sachs_albatross()[0] if name == "sachs-bird" else by_name(name)
    info = solve_min_wind(ac, "self_consistent" if z0f is None else "fixed8", rough, z0f,
                          u10_ref=10.0, kind="log", cycle="travel", n=n, h_min=h_min,
                          h_tip_min=-50.0 if tip is None else tip, seeds=(1, 0, 2, 3),
                          stop_after=2, tip_model="geometric", v_factor=vf, midpoints=True,
                          ground_effect=ge)
    return ac, info


def ground_clearance_task(job):
    """One clearance_study case (the same call) with ground_effect on or off."""
    name, rough, z0f, h_min, tip, ge = job
    t0 = time.time()
    _, info = _gm_min_wind(name, rough, z0f, h_min, tip, 1.1 if name == "albatross-3m" else 0.0, ge)
    return {"study": "clearance", "aircraft": name, "roughness": rough, "z0": z0f, "h_min": h_min,
            "h_tip_min": tip, "ground_effect": ge, "u10": info["u10"],
            "success": bool(info["success"]), "period": info["period"], "z_min": info["z_min"],
            "runtime_s": time.time() - t0}


def ground_winddiff_task(job):
    """One wind_difference_study case with ground_effect on or off."""
    from sim.seawind import Wind
    case, ge = job
    name, rough, z0f, h_min, tip, vf = case
    t0 = time.time()
    ac, info = _gm_min_wind(name, rough, z0f, h_min, tip, vf, ge)
    u = info["u10"]
    z0 = z0f if z0f is not None else Wind(u, 270.0, roughness="coare35").z0
    speed = lambda z: u * np.log(z / z0) / np.log(10.0 / z0)
    lo, hi = speed(info["z_min"]), speed(info["z_max"])
    return {"study": "wind_difference", "aircraft": name, "roughness": rough, "z0": z0,
            "h_min": h_min, "h_tip_min": tip, "ground_effect": ge, "u10": u,
            "success": bool(info["success"]), "z_low": info["z_min"], "z_high": info["z_max"],
            "wind_low": lo, "wind_high": hi, "wind_difference": hi - lo, "period": info["period"],
            "runtime_s": time.time() - t0}


def motor_task(job):
    """Average electric power against u10 for one (aircraft, cycle, ground effect).

    u10 = 0, 3, 6, 9, the soaring minimum of the same settings (min_wind, same ground_effect)
    (the table row at u_min shows whether the power is 0 there). Travel: chi = 20 deg (the library's), mean speed along chi at least
    0.5 v_best_ld. Compared with patterns.flap_glide_cost at the same wind (same efficiencies, track 160 deg from
    the wind source, i.e. 20 deg from straight downwind; loiter has no track, same power)."""
    from sim.dsopt import profile_for, solve, solve_min_wind
    from sim.patterns import flap_glide_cost
    name, cycle, ge, n, seeds, maxiter = job
    ac = by_name(name)
    kw = GM_MOTOR_SETTINGS
    # the soaring minimum of the airframe that carries the motor: folded propeller drag included
    umin = solve_min_wind(ac, "self_consistent", "coare35", None, u10_ref=10.0, cycle="travel",
                          n=n, seeds=(0, 1, 2), stop_after=2, ground_effect=ge, folded_prop=True,
                          **kw)
    u_min = float(umin["u10"])
    vreq = 0.5 * ac.v_best_ld if cycle == "travel" else None
    rows = []
    for u10 in (0.0, 3.0, 6.0, 9.0, round(u_min, 2)):
        t1 = time.time()
        prof = profile_for(max(u10, 1.0), roughness="coare35")
        info = solve(ac, prof, "min_power", cycle, n, u10=u10, chi=np.radians(20.0),
                     seeds=seeds, stop_after=1, motor=True, ground_effect=ge, v_travel_min=vreq,
                     maxiter=maxiter, **kw)
        from sim.record import value
        fg = flap_glide_cost(ac, u10, 160.0, prop_eff=value("prop.prop_eff", 0.60),
                             motor_eff=value("prop.motor_eff", 0.75))
        vchi = info["speed_along_chi"]
        rows.append({"study": "motor", "aircraft": name, "cycle": cycle, "ground_effect": ge,
                     "u10": u10, "u10_min_soaring": u_min, "success": bool(info["success"]),
                     "violation": info["violation"], "period": info["period"],
                     "power_avg_w": info["power_avg_w"], "power_peak_w": info["power_peak_w"],
                     "thrust_on_share": info["thrust_on_share"], "wh_per_hour": info["wh_per_hour"],
                     "speed_along_chi": vchi,
                     "wh_per_km": (info["power_avg_w"] / (3.6 * vchi)
                                   if cycle == "travel" and vchi > 0.05 else None),
                     "v_travel_min": vreq, "flap_glide_power_w": fg["power_w"],
                     "flap_glide_wh_per_km": fg["wh_per_km"], "iterations": info["iterations"],
                     "runtime_s": time.time() - t1})
        print("motor %s %s ge=%s u10=%.2f ok=%s P=%.2f W (%.0f s)" % (
            name, cycle, ge, u10, info["success"], info["power_avg_w"], time.time() - t1),
            flush=True)
    return rows


def _gm_fmt(v, f="%.2f"):
    return "n/a" if v is None else f % v


def ground_motor_tables(rows):
    lines = ["", "### A. u10_min with and without ground effect (m/s)"]
    a = [r for r in rows if r["study"] in ("clearance", "wind_difference")]
    for st in ("clearance", "wind_difference"):
        body = []
        keys = dict.fromkeys((r["aircraft"], r["roughness"], r["h_min"], r["h_tip_min"])
                             for r in a if r["study"] == st)
        for k in keys:
            pair = {r["ground_effect"]: r for r in a if r["study"] == st and
                    (r["aircraft"], r["roughness"], r["h_min"], r["h_tip_min"]) == k}
            off, on = pair.get(False), pair.get(True)
            if not off or not on:
                continue
            extra = ([_gm_fmt(off["wind_difference"]), _gm_fmt(on["wind_difference"])]
                     if st == "wind_difference" else [])
            body.append([k[0], k[1], str(k[2]), str(k[3]), "%.3f" % off["u10"], "%.3f" % on["u10"],
                         "%+.3f" % (on["u10"] - off["u10"])] + extra +
                        ["yes" if off["success"] and on["success"] else "NO"])
        head = ["aircraft", "rough", "h_min", "tip", "off", "on", "diff"]
        if st == "wind_difference":
            head += ["dW off", "dW on"]
        lines += ["", "#### " + st, "", _md(head + ["conv"], body)]
    m = [r for r in rows if r["study"] == "motor"]
    lines += ["", "### B. average electric power (W) against u10"]
    for nm in dict.fromkeys(r["aircraft"] for r in m):
        for cyc in ("travel", "loiter"):
            sel = [r for r in m if r["aircraft"] == nm and r["cycle"] == cyc]
            if not sel:
                continue
            body = []
            for r in sorted(sel, key=lambda r: (r["ground_effect"], r["u10"])):
                body.append(["on" if r["ground_effect"] else "off", "%.2f" % r["u10"],
                             "%.2f" % r["power_avg_w"], "%.0f" % r["power_peak_w"],
                             "%.2f" % r["thrust_on_share"], _gm_fmt(r["wh_per_km"]),
                             "%.1f" % r["flap_glide_power_w"], _gm_fmt(r["flap_glide_wh_per_km"]),
                             "yes" if r["success"] else "no (viol %.0e)" % r["violation"]])
            umins = "/".join("%.2f" % next(r["u10_min_soaring"] for r in sel
                                           if r["ground_effect"] == g) for g in (False, True))
            lines += ["", "#### %s, %s (u10_min soaring off/on: %s)" % (nm, cyc, umins), "",
                      _md(["ground", "u10", "P avg", "P peak", "T>0", "Wh/km", "flap-glide W",
                           "flap-glide Wh/km", "conv"], body)]
    return "\n".join(lines)


def ground_motor_study(path=GROUND_MOTOR_PATH, workers=4, n_motor=24, seeds=(0, 1),
                       maxiter=400):
    """Study A (ground effect on u10_min, clearance and wind difference cases) and study B (motor).

    Writes path (rows, settings, meta) and prints markdown tables."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    bjobs = [(nm, cyc, ge, n_motor, seeds, maxiter) for nm in ("baseline-2m", "albatross-3m")
             for cyc in ("travel", "loiter") for ge in (False, True)]
    cjobs = []
    for name in ("sachs-bird", "albatross-3m"):
        for h_min, tip in GM_CLEARANCE_FLOORS:
            roughs = [("coare35", None)]
            if name == "sachs-bird" and tip is None:
                roughs.append(("fixed_z0", 0.03))
            for rough, z0 in roughs:
                for ge in (False, True):
                    cjobs.append((name, rough, z0, h_min, tip, ge))
    wjobs = [(c, ge) for c in GM_WINDDIFF_CASES for ge in (False, True)]
    with Pool(workers) as pool:
        b = pool.map_async(motor_task, bjobs, chunksize=1)
        c = pool.map_async(ground_clearance_task, cjobs, chunksize=1)
        w = pool.map_async(ground_winddiff_task, wjobs, chunksize=1)
        rows = [r for x in b.get() for r in x] + c.get() + w.get()
    paths = ["sim/formstudy.py", "sim/dsopt.py", "sim/aircraft.py", "sim/aircraft_models.py",
             "sim/seawind.py", "sim/patterns.py", "sim/groundeffect.py", "record/values.yaml"]
    out = {"meta": record.meta("python -c \"from sim import formstudy; "
                               "formstudy.ground_motor_study()\"", paths),
           "settings": {"ground_effect": "model image (sim.groundeffect): rolled elliptic lifting line, image "
                                         "method, Trefftz plane drag, tabulated K(hc/b, bank)",
                        "motor": {**GM_MOTOR_SETTINGS, "n_nodes": n_motor, "seeds": list(seeds),
                                  "maxiter": maxiter, "chi_deg": 20.0,
                                  "travel_speed_min": "0.5 v_best_ld along chi",
                                  "thrust_max": "0.5 m g", "prop_eff": 0.60, "motor_eff": 0.75,
                                  "folded_drag_cda_m2": 3.0e-4,
                                  "roughness": "coare35 at u10 (>= 1 m/s)"}},
           "rows": rows, "runtime_s": time.time() - t_start}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    print(ground_motor_tables(rows))
    print("runtime %.0f s" % out["runtime_s"])
    return out


WAVE4_PATH = os.path.join("sim", "out", "dsopt_wave4.json")
WAVE4_MARGINS = {"margins": (2.0, 0.8, 0.9, 1.2), "none": (1.0, 1.0, 1.0, 1.1)}
WAVE4_PRESETS = ("p63", "p64_low")
WAVE4_TP = (8.0, 11.0)
WAVE4_HS = (1.0, 3.0)
WAVE4_KEYS = ("success", "converged", "violation", "u10", "period", "n_max", "z0", "z_min",
              "mu_max_deg", "ground_speed", "runtime_s", "m", "phi0", "theta_at_h_min_deg",
              "eta_over_a_at_h_min", "h_min_position", "u10_at_floor", "h_node_min", "h_mid_min",
              "h_min_achieved", "n_nodes", "n_nodes_initial", "refined", "refine_steps",
              "replay_pass_initial", "n_nodes_first", "replay_pass_first", "replay_pass", "rc_closure_h", "rc_closure_V",
              "rc_closure_V_rel", "rc_closure_gamma", "rc_closure_psi", "rc_h_min", "rc_tip_min",
              "rc_n_max", "rc_n_min", "bank_rate_over_limit_max", "cl_rate_max_per_s", "message")


def _solve_wave4(ac, wave, preset, margins, n, seeds, starts):
    """Best of the warm starts (x vectors of any mesh) and, if none succeeded, the cold seeds.
    wave None: the flat-sea reference with the same settings. A warm start also sets the u10
    at which the roughness iteration begins; tol_u10 is 0.05 m/s."""
    from sim.dsopt import solve_min_wind
    h_min, n_frac, mu_frac, v_factor = margins
    kw = dict(cycle="travel", n=n, h_min=h_min, n_frac=n_frac, mu_frac=mu_frac,
              v_factor=v_factor, tip_model="geometric", z0_mode="self_consistent",
              roughness="coare35", wave=wave, wave_preset=preset, return_x=True,
              midpoints=True, rate_limits=True, rate_model="pb2v", n_min=0.0,
              phase_pin="theta", m_search=(-1, 0, 1), replay_check=True, refine=True,
              tol_u10=0.05, maxiter=150)
    best = None
    for x0 in starts:
        c = solve_min_wind(ac, x0=x0, seeds=seeds, stop_after=1, cold_fallback=False,
                           u10_ref=max(float(x0[-2]) * 10.0, 0.5), **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    if best is None or not best[0]["success"]:
        c = solve_min_wind(ac, seeds=seeds, stop_after=1, **kw)
        if _better(c[0], best[0] if best else None):
            best = c
    return best


def wave4_flat_task(job):
    name, mlabel, n, seeds = job
    t0 = time.time()
    c = _solve_wave4(by_name(name), None, "model", WAVE4_MARGINS[mlabel], n, seeds, [])
    info = c[0]
    print("flat %s %s: ok=%s u10=%.2f T=%.2f n=%s (%.0f s)" % (
        name, mlabel, info["success"], info["u10"], info["period"], info["n_nodes"],
        time.time() - t0), flush=True)
    return [dict({k: info.get(k) for k in WAVE4_KEYS}, aircraft=name, margin_set=mlabel,
                 kind="flat", margins=list(WAVE4_MARGINS[mlabel]))], {}


def wave4_chain_task(job):
    """One (aircraft, Tp, margin set, preset): across chain over Hs, then along chain started from
    the rotated across solutions (the third sweep of wave study 3 is left out); `earlier` holds
    the solutions of another preset for the same cases, used as further starts. Cases not
    started before the deadline (epoch seconds) are returned as not run. Returns (rows, xs)."""
    from sim.dsopt import rotate_start
    name, tp, mlabel, preset, n, seeds, hs_list, deadline, earlier = job
    ac = by_name(name)
    margins = WAVE4_MARGINS[mlabel]
    lam = 9.81 * tp * tp / (2.0 * np.pi)
    dirs = {"across": 90.0, "along": 0.0}
    res = {d: {} for d in dirs}

    def sweep(dlabel, other):
        prev = None
        for hs in hs_list:
            if time.time() > deadline:
                return
            t0 = time.time()
            wave = {"a": hs / 2.0, "wavelength": lam, "to_deg": dirs[dlabel]}
            starts = []
            if (dlabel, hs) in earlier:
                starts.append(earlier[(dlabel, hs)])
            if prev is not None:
                starts.append(prev)
            if other is not None and hs in res[other] and res[other][hs][0]["success"]:
                _, xo, colo = res[other][hs]
                starts.append(rotate_start(colo, xo, dirs[dlabel] - dirs[other]))
            c = _solve_wave4(ac, wave, preset, margins, n, seeds, starts)
            res[dlabel][hs] = c
            if c[0]["success"]:
                prev = c[1]
            print("%s Tp=%g %s %s %s Hs=%g: ok=%s u10=%.2f T=%.2f m=%s n=%s %s (%.0f s)" % (
                name, tp, mlabel, preset, dlabel, hs, c[0]["success"], c[0]["u10"],
                c[0]["period"], c[0]["m"], c[0]["n_nodes"], c[0]["h_min_position"],
                time.time() - t0), flush=True)

    sweep("across", None)
    sweep("along", "across")
    rows, xs = [], {}
    for dlabel in dirs:
        for hs in hs_list:
            base = dict(aircraft=name, hs=hs, tp=tp, direction=dlabel, to_deg=dirs[dlabel],
                        a=hs / 2.0, wavelength=lam, preset=preset, margin_set=mlabel,
                        margins=list(margins), kind="wave")
            if hs not in res[dlabel]:
                rows.append(dict(base, success=False, not_run=True))
                continue
            info = res[dlabel][hs][0]
            rows.append(dict({k: info.get(k) for k in WAVE4_KEYS}, **base))
            if info["success"]:
                xs[(dlabel, hs)] = res[dlabel][hs][1]
    return rows, xs


def wave4_tables(rows):
    flat = {(r["aircraft"], r["margin_set"]): r for r in rows if r["kind"] == "flat"}
    waves = [r for r in rows if r["kind"] == "wave"]
    out = []
    for key, r in sorted(flat.items()):
        out.append("flat %s %s: %s u10 %.2f T %.2f n %s" % (
            key[0], key[1], "ok" if r["success"] else "FAILED", r["u10"], r["period"],
            r["n_nodes"]))
    head = ["aircraft", "margins", "preset", "Tp", "dir", "Hs", "u10 (flat)", "T", "m",
            "low turn on", "n max", "n nodes (final/first)", "first mesh closed"]
    body = []
    for r in sorted(waves, key=lambda r: (r["aircraft"], r["margin_set"], r["preset"], r["tp"],
                                          r["direction"], r["hs"])):
        if r.get("not_run"):
            body.append([r["aircraft"], r["margin_set"], r["preset"], "%g" % r["tp"], r["direction"],
                         "%g" % r["hs"], "not run", "", "", "", "", ""])
            continue
        f = flat.get((r["aircraft"], r["margin_set"]))
        fu = "%.2f" % f["u10"] if f and f["success"] else "-"
        u = ("%.2f" % r["u10"]) + ("*" if r.get("u10_at_floor") and r["success"] else "")
        if not r["success"]:
            u = "fail"
        body.append([r["aircraft"], r["margin_set"], r["preset"], "%g" % r["tp"], r["direction"],
                     "%g" % r["hs"], "%s (%s)" % (u, fu), "%.1f" % (r["period"] or 0),
                     str(r["m"]), str(r["h_min_position"]),
                     "%.2f" % (r["rc_n_max"] or 0),
                     "%s/%s" % (r["n_nodes"], r.get("n_nodes_first")),
                     {True: "yes", False: "no"}.get(r.get("replay_pass_first"), "-")])
    out.append(_md(head, body))
    out.append("* = u10 at the floor (waves alone carry the cycle)")
    return "\n".join(out)


def wave_study4(path=WAVE4_PATH, n=30, seeds=(0,), workers=4, names=WAVE_AIRCRAFT,
                hs_list=WAVE4_HS, tps=WAVE4_TP, presets=WAVE4_PRESETS,
                margin_sets=("margins", "none"), budget_s=40 * 60, with_flat=True):
    """Fourth wave pass (Q-058, Q-059, K-011): calibrated wave wind (P63 and the P64 low bound),
    COARE 3.5 roughness, replay check and mesh refinement on, with and without the control
    library's margins, next to the flat-sea value with the same settings. Writes path after every
    finished job. The presets run as phases (the first preset for all (aircraft, Tp, margins),
    then the next, started also from the first one's solutions); no case is started once the
    budget (seconds) is used, and such cases are written as not run."""
    from multiprocessing import Pool
    from sim import record
    t_start = time.time()
    settings = {"n_nodes_start": n, "seeds": list(seeds), "roughness": "coare35",
                "z0_mode": "self_consistent", "tol_u10": 0.05, "tip_model": "geometric",
                "midpoints": True, "rate_model": "pb2v", "roll_pb2v": 0.08,
                "cl_rate_max_per_s": 1.0, "n_min": 0.0, "phase_pin": "theta",
                "m_search": [-1, 0, 1], "replay_check": True, "refine": True,
                "dt_max_s": 0.25, "n_cap": 160, "refine_tries": 2, "refine_maxiter": 100,
                "maxiter": 150,
                "u10_floor": 0.1, "sweeps": "across over Hs, along over Hs from rotated across",
                "margin_sets": {k: dict(zip(("h_min", "n_frac", "mu_frac", "v_factor"),
                                            WAVE4_MARGINS[k])) for k in margin_sets},
                "presets": list(presets), "hs": list(hs_list), "tp": list(tps),
                "aircraft": list(names), "budget_s": budget_s}
    meta = record.meta("python -m sim.dsopt --wave-study4", _record_files())
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rows = []

    def dump(done):
        result = {"meta": meta, "settings": settings, "complete": done, "rows": rows,
                  "runtime_s": time.time() - t_start}
        with open(path, "w", newline=chr(10)) as fh:
            json.dump(result, fh, indent=1)
            fh.write(chr(10))
        return result

    deadline = t_start + budget_s
    with Pool(workers) as pool:
        if with_flat:
            asyncs = [pool.apply_async(wave4_flat_task, ((nm, ml, n, tuple(seeds) + (3,)),))
                      for nm in names for ml in margin_sets]
            for a in asyncs:
                rows += a.get()[0]
            dump(False)
        prev = {}   # (aircraft, tp, margin set) -> solutions of the previous preset
        for preset in presets:
            keys = [(nm, tp, ml) for nm in names for tp in tps for ml in margin_sets]
            asyncs = [(k, pool.apply_async(wave4_chain_task, ((k[0], k[1], k[2], preset, n, seeds,
                                                               hs_list, deadline,
                                                               prev.get(k, {})),)))
                      for k in keys]
            new = {}
            for k, a in asyncs:
                r, xs = a.get()
                rows += r
                new[k] = xs
                dump(False)
            prev = new
    result = dump(True)
    print(wave4_tables(rows))
    print("runtime %.0f s" % result["runtime_s"])
    return result


# ---- skim study (D-018, docs/02-form.md#wingtip-skim)
SKIM_PATH = os.path.join("sim", "out", "skim_study.json")
SKIM_FLOORS = (2.0, 1.0, 0.5, 0.3)            # body floor h_min (m)
# wingtip clearances after ds.clearance_min_m (R-12's design point, read from the record): the
# control library's 0.3 m (dsopt's default), 0.1 m, and 0 (the skid touches)
SKIM_TIPS_EXTRA = (0.3, 0.1, 0.0)
SKIM_MARGINS = dict(n_frac=0.8, mu_frac=0.9, v_factor=1.2)   # wingopt.MARGINS without h_min
# (label, inner deg, outer deg, break eta); see dsopt.WingShape
SKIM_SHAPES = (("dihedral 5 deg", 5.0, 5.0, 1.0), ("dihedral 10 deg", 10.0, 10.0, 1.0),
               ("outer panel 10 deg from 0.5", 0.0, 10.0, 0.5),
               ("outer panel 20 deg from 0.5", 0.0, 20.0, 0.5))
SKIM_LEVER_TIPS = (0.3, 0.0)
SKIM_SEEDS = (0, 1, 2, 3, 4, 5)
SKIM_STOP_AFTER = 3
SKIM_AIRCRAFT = ("ar20-sd7037-limit", "ar20-sd7037-none", "albatross-3m")


def skim_aircraft(name):
    """(make_ac(re), wingopt Design or None, Reynolds number to start from) for a skim-study name:
    albatross-3m (fixed polar), the wing pass's best design (3.0 m, AR 20, SD7037, ballast to
    ds.mass_max_kg as in wingopt.ballast_grid) and the same wing without ballast."""
    if name == "albatross-3m":
        ac = by_name(name)
        return (lambda re: ac), None, None
    from sim import wingopt as w
    from sim.record import value
    if name == "ar20-sd7037-limit":
        bal = w.ballast_to_mass(3.0, 20.0, w.RHO_A["base"], value("ds.mass_max_kg") - 1e-3)
    elif name == "ar20-sd7037-none":
        bal = 0.0
    else:
        raise ValueError(name)
    d = w.Design(3.0, 20.0, airfoil="SD7037", ballast=bal)
    _, _, chord, vs = w.basics(d)
    return (lambda re: w.build_aircraft(d, re)), d, w.re_of(1.3 * vs, chord)


def skim_tip_stats(col, x):
    """Where the lowest wing point is in a flat-sea cycle: its height, the bank, airspeed, the
    horizontal speed over the water (centre's ground velocity; the rotation of the tip about the
    centre is neglected), the curvature of the tip height at its minimum (parabola through the
    nodes around it) and the time the tip spends within 0.1 m of its minimum (linear between
    nodes)."""
    S, C, T, u = col.unpack(x)
    N = col.N
    from sim.dsopt import _tip_heights
    tip = _tip_heights(col, S[:, 2], C[:, 1], S[:, 4])
    i0 = int(np.argmin(tip))
    dt = T / (N - 1)
    V, g, p, z = S[i0, 3], S[i0, 4], S[i0, 5], S[i0, 2]
    U = u * float(col.prof.f(z))
    vx, vy = V * np.cos(g) * np.cos(p) + U, V * np.cos(g) * np.sin(p)
    per = np.concatenate([tip[:-1], tip[:-1], tip[:-1]])   # node N-1 is node 0 of the next cycle
    j = i0 % (N - 1) + (N - 1)
    tt = np.arange(-2, 3) * dt
    a2 = float(np.polyfit(tt, per[j - 2:j + 3], 2)[0])
    fine = np.interp(np.linspace(0.0, N - 1, 40 * (N - 1) + 1), np.arange(N), tip)
    return {"tip_min_m": float(tip[i0]), "tip_t_s": float(i0 * dt), "bank_at_tip_deg": float(np.degrees(abs(C[i0, 1]))),
            "z_at_tip_m": float(z), "V_at_tip_ms": float(V), "water_speed_at_tip_ms": float(np.hypot(vx, vy)),
            "tip_curvature_ms2": 2.0 * a2, "time_within_0p1_s": float(np.mean(fine < tip[i0] + 0.1) * T),
            "bank_max_deg": float(np.degrees(np.abs(C[:, 1]).max())),
            "tip_active": bool(tip[i0] - col.h_tip < 0.02)}


def _skim_solve(make_ac, d, re, h_min, tip, shape, warm, seeds=SKIM_SEEDS):
    """Minimum wind of one cell: cold starts (seeds, the first SKIM_STOP_AFTER that converge) and every warm
    start, the lowest converged u10 kept; then, for a wingopt design, one more Reynolds pass warm
    from it when the polar scale at the cycle's mean airspeed differs by more than wingopt.SCALE_TOL
    (as wingopt.solve_a). Returns (row, x)."""
    from sim.dsopt import WingShape, solve_min_wind
    from sim import wingopt as w
    t0 = time.time()
    kw = dict(z0_mode="self_consistent", roughness="coare35", u10_ref=10.0, kind="log",
              cycle="travel", n=40, h_min=h_min, h_tip_min=tip, tip_model="geometric",
              midpoints=True, rate_limits=True, rate_model="pb2v", n_min=0.0, replay_check=True,
              return_x=True, wing_shape=None if shape is None else WingShape(*shape), **SKIM_MARGINS)

    def best_of(ac, starts):
        cands = []
        if starts is None:
            if seeds:
                cands.append(("cold", solve_min_wind(ac, seeds=seeds, stop_after=SKIM_STOP_AFTER, **kw)))
            starts = warm
        for label, xw in starts:
            cands.append((label, solve_min_wind(ac, x0=xw, seeds=seeds, cold_fallback=False, **kw)))
        ok = [c for c in cands if c[1][0]["success"]]
        tried = {c[0]: (float(c[1][0]["u10"]) if c[1][0]["success"] else None) for c in cands}
        return (min(ok, key=lambda c: c[1][0]["u10"]) if ok else None), tried

    best, tried = best_of(make_ac(re), None)
    re_cycle, passes = None, 1
    if best is not None and d is not None:
        nexp = w.AIRFOILS[d.airfoil][3]
        chord = w.basics(d)[2]
        re_cycle = w.re_of(w.mean_airspeed(best[1][2], best[1][1]), chord)
        if abs(w._scale(re_cycle, nexp) / w._scale(re, nexp) - 1.0) > w.SCALE_TOL:
            b2, t2 = best_of(make_ac(re_cycle), [("re pass", best[1][1])])
            passes = 2
            if b2 is not None:
                best, re = b2, re_cycle
                re_cycle = w.re_of(w.mean_airspeed(best[1][2], best[1][1]), chord)
            tried.update({"re pass": t2["re pass"]})
    row = {"h_min": h_min, "h_tip_min": tip, "shape": None if shape is None else WingShape(*shape).describe(),
           "success": best is not None, "tried": tried, "re_used": re, "re_cycle": re_cycle,
           "re_passes": passes, "runtime_s": time.time() - t0}
    if best is None:
        return row, None
    label, (info, x, col) = best
    row.update({"start": label, "u10": float(info["u10"]), "period": float(info["period"]),
                "z_min": float(info["z_min"]), "z_max": float(info["z_max"]), "n_max": float(info["n_max"]),
                "rc_tip_min": info.get("rc_tip_min"), "rc_h_min": info.get("rc_h_min"),
                "v_stall": float(col.ac.v_stall), "mass": float(col.ac.mass), "span": float(col.ac.span)})
    row.update(skim_tip_stats(col, x))
    row["x"] = [float("%.9g" % v) for v in np.asarray(x)]   # scaled decision vector (dsopt.Collocation.pack)
    return row, np.asarray(x)


def skim_task(job):
    """One aircraft: the floor and clearance grid from the tightest cell to the loosest (each cell
    warm-started from its tighter neighbours, whose solutions are feasible in it); then the cells
    that failed once more, from the loosest back, warm from their looser neighbours and with six
    more seeds; then the geometry levers at the lowest floor that solved, for SKIM_LEVER_TIPS."""
    from sim.record import value
    name, progress = job
    make_ac, d, re = skim_aircraft(name)
    tips = (float(value("ds.clearance_min_m")),) + SKIM_TIPS_EXTRA
    rows, sols = [], {}

    def note(row):
        row["aircraft"] = name
        rows.append(row)
        print("skim %s floor %.1f tip %.2f %s: ok=%s u10=%s (%.0f s)" % (
            name, row["h_min"], row["h_tip_min"], row.get("lever") or "flat", row["success"],
            "%.2f" % row["u10"] if row["success"] else "-", row["runtime_s"]), flush=True)
        if progress:
            with open(progress, "a") as fh:
                fh.write(json.dumps(row) + "\n")

    for ti, c in enumerate(tips):
        first_le = next((f for f in SKIM_FLOORS if f <= c), None)
        for fi, h in enumerate(SKIM_FLOORS):
            if h <= c and h != first_le:   # the tip constraint implies the floor: same problem
                same = dict(sols[(ti, SKIM_FLOORS.index(first_le))][0], h_min=h, same_as_floor=first_le,
                            runtime_s=0.0)
                sols[(ti, fi)] = (same, sols[(ti, SKIM_FLOORS.index(first_le))][1])
                note(same)
                continue
            warm = []
            for key, lab in (((ti - 1, fi), "tighter tip"), ((ti, fi - 1), "higher floor")):
                if key in sols and sols[key][1] is not None:
                    warm.append((lab, sols[key][1]))
            row, x = _skim_solve(make_ac, d, re, h, c, None, warm)
            if row["success"] and row.get("re_cycle"):
                re = row["re_used"]
            sols[(ti, fi)] = (row, x)
            note(row)
    for ti in reversed(range(len(tips))):
        c = tips[ti]
        first_le = next((f for f in SKIM_FLOORS if f <= c), None)
        for fi in reversed(range(len(SKIM_FLOORS))):
            row0 = sols[(ti, fi)][0]
            if row0["success"] or row0.get("same_as_floor") is not None:
                continue
            warm = [(lab, sols[key][1]) for key, lab in (((ti + 1, fi), "looser tip"), ((ti, fi + 1), "lower floor"))
                    if key in sols and sols[key][1] is not None]
            row, x = _skim_solve(make_ac, d, row0.get("re_used") or re, SKIM_FLOORS[fi], c, None, warm,
                                 seeds=tuple(s_ + 6 for s_ in SKIM_SEEDS))
            row["backfill"] = True
            row["runtime_s"] += row0["runtime_s"]
            rows.remove(row0)
            sols[(ti, fi)] = (row, x)
            note(row)
            if SKIM_FLOORS[fi] == first_le:   # the copies of this cell
                for fj, h in enumerate(SKIM_FLOORS):
                    old = sols[(ti, fj)][0]
                    if old.get("same_as_floor") == first_le:
                        rows.remove(old)
                        sols[(ti, fj)] = (dict(row, h_min=h, same_as_floor=first_le, runtime_s=0.0), x)
                        note(sols[(ti, fj)][0])
    for c in SKIM_LEVER_TIPS:
        ti = tips.index(c)
        solved = [fi for fi in range(len(SKIM_FLOORS)) if sols[(ti, fi)][0]["success"]]
        if not solved:
            continue
        fi = max(solved)
        h = SKIM_FLOORS[fi]
        base = sols[(ti, fi)]
        prev = None
        for label, gi, go, eta in SKIM_SHAPES:
            warm = [("flat wing", base[1])] + ([("previous lever", prev)] if prev is not None else [])
            row, x = _skim_solve(make_ac, d, base[0].get("re_used") or re, h, c, (gi, go, eta), warm)
            row["lever"] = label
            row["flat_u10"] = base[0].get("u10")
            note(row)
            prev = x if x is not None else prev
    return rows


def skim_study(path=SKIM_PATH, workers=2, names=SKIM_AIRCRAFT, progress=None):
    """Minimum wind against the body floor and the wingtip clearance, with the wingtip constraint
    kept (geometric tip, heights and the tip held at the midpoints), on a flat sea with COARE 3.5
    (self-consistent z0) and the control library's other margins (load factor 0.8 of the limit,
    0.9 of the bank limit, 1.2 times the stall speed, the pb2v roll and lift-coefficient rates),
    for albatross-3m and the wing pass's AR 20 SD7037 design with and without ballast; then
    dihedral and a bent-up outer panel (dsopt.WingShape) at the lowest floor that solved.
    Writes path (sim/out/skim_study.json)."""
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    from sim.record import value
    t0 = time.time()
    jobs = [(nm, progress) for nm in names]
    rows = []
    with ProcessPoolExecutor(workers) as ex:
        for r in ex.map(skim_task, jobs, chunksize=1):
            rows += r
    paths = ["sim/formstudy.py", "sim/dsopt.py", "sim/wingopt.py", "sim/aircraft.py",
             "sim/aircraft_models.py", "sim/seawind.py", "record/values.yaml"]
    out = {"meta": record.meta("python -c \"from sim import formstudy; formstudy.skim_study()\"", paths),
           "settings": {"floors_m": list(SKIM_FLOORS),
                        "tips_m": [float(value("ds.clearance_min_m"))] + list(SKIM_TIPS_EXTRA),
                        "margins": SKIM_MARGINS, "seeds": list(SKIM_SEEDS), "stop_after": SKIM_STOP_AFTER,
                        "shapes": [list(s) for s in SKIM_SHAPES], "lever_tips_m": list(SKIM_LEVER_TIPS),
                        "roughness": "coare35 self-consistent", "n": 40, "replay_check": True},
           "rows": rows, "runtime_s": time.time() - t0}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    return out


# ---- support values of the skim configurations (docs/02-form.md#wingtip-skim)
SKIM_UPWIND_PATH = os.path.join("sim", "out", "skim_upwind.json")
SKIM_UPWIND_AIRCRAFT = ("ar20-sd7037-limit", "albatross-3m")
# (label, body floor, tip clearance, WingShape arguments)
SKIM_UPWIND_CONFIGS = (("standard", 2.0, 0.3, None), ("skid", 1.0, 0.0, None),
                       ("skid, dihedral 10 deg", 1.0, 0.0, (10.0, 10.0, 1.0)))
SKIM_UPWIND_U10 = (13.0, 16.0)
SKIM_UPWIND_CHI = (90.0, 135.0, 180.0)   # deg from downwind


def _skim_re(name, path=SKIM_PATH):
    """Reynolds number to start a wing design from: the skim study's standard cell (floor 2 m,
    clearance 0.3 m) if its output exists, else 1.3 times the stall speed."""
    mk, d, re = skim_aircraft(name)
    try:
        with open(path) as fh:
            for r in json.load(fh)["rows"]:
                if (r["aircraft"] == name and not r.get("lever") and r["success"] and r["h_min"] == 2.0
                        and r["h_tip_min"] == 0.3 and r.get("re_used")):
                    return r["re_used"]
    except (OSError, ValueError, KeyError):
        pass
    return re


def _skim_course(ac, prof, u10, chi, h_min, tip, shape, starts, seeds=SKIM_SEEDS):
    """Support value h(chi): dsopt max_speed toward chi at u10 with the skim study's margins, from
    cold starts (the first SKIM_STOP_AFTER of seeds that converge) and every warm start; the
    largest converged progress is kept. Returns (label, (info, x, col)) or None, and the tries."""
    from sim.dsopt import WingShape, solve
    kw = dict(mode="max_speed", cycle="travel", n=40, u10=u10, chi=np.radians(chi), h_min=h_min,
              h_tip_min=tip, tip_model="geometric", midpoints=True, rate_limits=True,
              rate_model="pb2v", n_min=0.0, replay_check=True, return_x=True,
              wing_shape=None if shape is None else WingShape(*shape), **SKIM_MARGINS)
    cands = []
    if seeds:
        cands.append(("cold", solve(ac, prof, seeds=seeds, stop_after=SKIM_STOP_AFTER, **kw)))
    for label, x0 in starts:
        cands.append((label, solve(ac, prof, x0=x0, **kw)))
    ok = [c for c in cands if c[1][0]["success"]]
    tried = {c[0]: (float(c[1][0]["speed_along_chi"]) if c[1][0]["success"] else None) for c in cands}
    return (max(ok, key=lambda c: c[1][0]["speed_along_chi"]) if ok else None), tried


def skim_upwind_task(job):
    """One aircraft at one u10: the three configurations in turn (each course warm-started from the
    previous course and from the same course of the previous configuration, plus cold starts),
    then for a wing design one more Reynolds pass warm from each course when the polar scale at
    the fastest course's mean airspeed differs by more than wingopt.SCALE_TOL (as wingopt.solve_b)."""
    from sim.dsopt import profile_for
    from sim import wingopt as w
    name, u10 = job
    mk, d, _ = skim_aircraft(name)
    re0 = _skim_re(name) if d is not None else None
    prof = profile_for(u10, "log", roughness="coare35")
    rows, prev_cfg = [], {}
    for label, h_min, tip, shape in SKIM_UPWIND_CONFIGS:
        t0 = time.time()
        re, sols, tries = re0, {}, {}
        prev_x = None
        for chi in SKIM_UPWIND_CHI:
            starts = [(lab, x) for lab, x in (("previous course", prev_x), ("previous configuration", prev_cfg.get(chi)))
                      if x is not None]
            best, tried = _skim_course(mk(re), prof, u10, chi, h_min, tip, shape, starts)
            tries[chi] = tried
            if best is not None:
                sols[chi] = best
                prev_x = np.asarray(best[1][1])
        for chi in SKIM_UPWIND_CHI:   # a course that failed: again, warm from every solved course, six more seeds
            if chi in sols:
                continue
            starts = [("course %g" % c, np.asarray(s[1][1])) for c, s in sols.items()]
            best, tried = _skim_course(mk(re), prof, u10, chi, h_min, tip, shape, starts,
                                       seeds=tuple(s_ + 6 for s_ in SKIM_SEEDS))
            tries[chi].update({"backfill " + k: v for k, v in tried.items()})
            if best is not None:
                sols[chi] = best
        passes = 1
        re_of_chi = {chi: re for chi in sols}
        if d is not None and sols:
            nexp = w.AIRFOILS[d.airfoil][3]
            chord = w.basics(d)[2]
            fast = max(sols, key=lambda c: sols[c][1][0]["speed_along_chi"])
            re_new = w.re_of(w.mean_airspeed(sols[fast][1][2], sols[fast][1][1]), chord)
            if abs(w._scale(re_new, nexp) / w._scale(re, nexp) - 1.0) > w.SCALE_TOL:
                passes = 2
                for chi in list(sols):
                    b2, t2 = _skim_course(mk(re_new), prof, u10, chi, h_min, tip, shape,
                                          [("re pass", np.asarray(sols[chi][1][1]))], seeds=())
                    tries[chi]["re pass"] = t2["re pass"]
                    if b2 is not None:   # else the first pass stands, at its own Reynolds number
                        sols[chi], re_of_chi[chi] = b2, re_new
        for chi in SKIM_UPWIND_CHI:
            row = {"aircraft": name, "u10": u10, "config": label, "h_min": h_min, "h_tip_min": tip,
                   "shape": None if shape is None else list(shape), "chi_deg": chi,
                   "success": chi in sols, "tried": {k: v for k, v in tries.get(chi, {}).items()},
                   "re_used": re_of_chi.get(chi, re), "re_passes": passes}
            if chi in sols:
                start, (info, x, col) = sols[chi]
                row.update({"start": start, "h_ms": float(info["speed_along_chi"]),
                            "ground_speed_ms": float(info["ground_speed"]), "period": float(info["period"]),
                            "z_min": float(info["z_min"]), "z_max": float(info["z_max"]),
                            "rc_tip_min": info.get("rc_tip_min"), "x": [float("%.9g" % v) for v in np.asarray(x)]})
                row.update(skim_tip_stats(col, x))
            rows.append(row)
            print("upwind %s u10 %g %s chi %g: ok=%s h=%s" % (name, u10, label, chi, row["success"],
                                                           "%.2f" % row["h_ms"] if row["success"] else "-"), flush=True)
        prev_cfg = {chi: np.asarray(s[1][1]) for chi, s in sols.items()}
        print("upwind %s u10 %g %s done (%.0f s)" % (name, u10, label, time.time() - t0), flush=True)
    return rows


def skim_upwind_study(path=SKIM_UPWIND_PATH, workers=2):
    """Support values h(90), h(135) and h(180) at u10 13 and 16 m/s for the best wing and the
    albatross-like glider in the standard configuration (floor 2 m, clearance 0.3 m), with the
    skid (floor 1 m, clearance 0) and with the skid and 10 deg of dihedral. Writes path."""
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    t0 = time.time()
    jobs = [(nm, u) for nm in SKIM_UPWIND_AIRCRAFT for u in SKIM_UPWIND_U10]
    rows = []
    with ProcessPoolExecutor(workers) as ex:
        for r in ex.map(skim_upwind_task, jobs, chunksize=1):
            rows += r
    paths = ["sim/formstudy.py", "sim/dsopt.py", "sim/wingopt.py", "sim/aircraft.py",
             "sim/aircraft_models.py", "sim/seawind.py", "record/values.yaml"]
    out = {"meta": record.meta("python -c \"from sim import formstudy; formstudy.skim_upwind_study()\"", paths),
           "settings": {"configs": [list(c) for c in SKIM_UPWIND_CONFIGS], "u10": list(SKIM_UPWIND_U10),
                        "chi_deg": list(SKIM_UPWIND_CHI), "margins": SKIM_MARGINS, "seeds": list(SKIM_SEEDS),
                        "stop_after": SKIM_STOP_AFTER, "frame": "chi from downwind: 90 across, 180 straight upwind",
                        "roughness": "coare35 at u10", "n": 40, "replay_check": True},
           "rows": rows, "runtime_s": time.time() - t0}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    return out


if __name__ == "__main__":
    run()
