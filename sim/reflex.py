"""The reflex redesign (Q-068, D-019): constructed dives and contacts, and the reflex in the closed loop.

python -m sim.mpc --reflex-exp      key reflex_exp of sim/out/mpc.json: the constructed cases of reflex_cases, each
                                    flown without a reflex, with the first reflex (sim.mpc.Reflex, K-015) and with the
                                    redesigned one (sim.mpc.Reflex2).

A constructed case is a stale plan: the commands of a plan that leads into the water are held from a given state, as
when the planner is wrong about the sea or late, and only the reflex can save the aircraft. The true dynamics fly
through the actuators (rate model pb2v, the load-factor limit), and the reflex supervises at every step with the
prediction of that same plan from the present state (in the world it is given) and, for the redesign, the sensors at
the low wingtip on the true sea (sim.mpc.Reflex2). When the reflex lets go, the stale plan resumes; the case runs for
REFLEX_TRIAL_S.

- "dive": a flat sea, the plan pushes the nose down (a lift coefficient of DIVE_CL_FRAC of the best-glide one) with
  the bank held, from heights, flight-path angles, airspeeds and banks of DIVE_GRID.
- "contact": a banked low turn over a regular swell (Hs 3 m, Tp 10 s) flying into the crests, the bank and the lift
  coefficient of a level turn held, the low wingtip a little above the water (CONTACT_GRID), so that a crest rises
  under it; the reflex's prediction sees either the true swell ("truth") or a flat sea ("flat", the planner did not
  see the swell), and the tip sensors always see the true sea.

The case ends at a contact of the centre, or of the wingtip deeper than the skid's depth (skid.blade_depth_m, D-018);
the deepest immersion of the tip is kept, since a touch within the skid's depth is survivable in principle (the forces
of the skid are not in the point mass, docs/02-form.md#wingtip-skim).
"""
from __future__ import annotations

import numpy as np

from sim.aircraft import G, derivatives, n_limited_cl, rk4_step

REFLEX_TRIAL_S = 4.0
DIVE_CL_FRAC = 0.2
DIVE_GRID = {"h_m": (4.0, 8.0), "gamma_deg": (-10.0, -20.0, -30.0), "v_over_stall": (1.3, 1.8), "bank_deg": (0.0, 35.0)}
CONTACT_GRID = {"bank_deg": (35.0, 55.0), "tip_gap_m": (0.3, 0.6), "v_over_stall": (1.3, 1.8), "theta0": (1.2, 0.6)}
REFLEX_AIRCRAFT = ("albatross-3m", "baseline-2m")
REFLEX_VERSIONS = ("none", "v1", "v2")
REFLEX_U10 = 9.0


def reflex_trial(ac, env, world, s0, plan, version, t_end=REFLEX_TRIAL_S, dt=0.01, tip_depth_m=None, seed=1):
    """Fly a stale plan (dict knots_t, cl, mu, thrust: held from now on, its times counted from t = 0) from state s0
    on env with the reflex version ("none", "v1", "v2"); world: the reflex's prediction world. Returns the outcome."""
    from sim.aircraft import Actuators
    from sim import mpc
    from sim.dsopt import folded_dcd, prop_efficiency, thrust_max
    from sim.record import value
    tip_depth_m = float(value("skid.blade_depth_m", 0.04)) if tip_depth_m is None else tip_depth_m
    t_max = thrust_max(ac)
    fdcd = folded_dcd(ac)
    eta_p = prop_efficiency()
    act = Actuators(ac, rate_model="pb2v")
    act.reset()
    kt, clk, muk, thk = (np.asarray(plan[k], float) for k in ("knots_t", "cl", "mu", "thrust"))
    rf = None
    if version == "v1":
        rf = mpc.Reflex(ac, t_max, fdcd, True, "state")
    elif version == "v2":
        rf = mpc.Reflex2(ac, t_max, fdcd, True, "state", seed=seed)
    s = np.asarray(s0, float).copy()
    c0, m0 = float(np.interp(0.0, kt, clk)), float(np.interp(0.0, kt, muk))
    act.step(c0, m0, dt, s[3], thrust=float(np.interp(0.0, kt, thk)))      # the actuators hold the plan's command
    n_now = 1.0
    out = {"version": version, "ended": "complete", "firings": 0, "reflex_s": 0.0, "causes": {}, "first_fire_s": None}
    h_min, tip_min, depth, v_min, g_max, loop_s, energy, wet_s = np.inf, np.inf, 0.0, np.inf, -np.inf, 0.0, 0.0, 0.0
    on_prev = False
    t = 0.0
    for i in range(int(round(t_end / dt)) + 1):
        t = i * dt
        eta = float(env.surface(s[0], s[1], t)[0])
        h = s[2] - eta
        mu_a = act.mu
        tip = mpc.wingtip_height(ac, s[2], eta, s[4], mu_a)
        h_min, tip_min, v_min, g_max = min(h_min, h), min(tip_min, tip), min(v_min, s[3]), max(g_max, s[4])
        if tip < 0.0:
            depth, wet_s = max(depth, -tip), wet_s + dt
        if np.cos(s[4]) <= 0.0:
            loop_s += dt
        if h < 0.0:
            out["ended"] = "contact"
            break
        if tip < -tip_depth_m:
            out["ended"] = "tip contact"
            break
        if i * dt >= t_end - 1e-9:
            break
        cl, mu = float(np.interp(t, kt, clk)), float(np.interp(t, kt, muk))
        T = float(np.clip(np.interp(t, kt, thk), 0.0, t_max))
        hit = None
        if version == "v2":
            rf.sense(t, tip, mu_a, s[3])
            hit = rf.check(world, t, s, kt, clk, muk, thk, t)
        elif version == "v1":
            hit = rf.predict(world, t, s, kt, clk, muk, thk, t)
        if hit is not None:
            if not on_prev:
                out["firings"] += 1
                out["causes"][hit[0]] = out["causes"].get(hit[0], 0) + 1
                if out["first_fire_s"] is None:
                    out["first_fire_s"] = float(t)
            if version == "v2":
                cl, mu, T = rf.override(s, hit[0], n_now)
            else:
                cl, mu, _ = rf.override(s)
                T = t_max
            out["reflex_s"] += dt
        on_prev = hit is not None
        cl = n_limited_cl(ac, s[3], cl)
        cl_a, mu_a = act.step(cl, mu, dt, s[3], thrust=T)
        T_a = act.T
        cl_f = n_limited_cl(ac, s[3], cl_a)
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        n_now = q * cl_f / (ac.mass * G)
        energy += T_a * s[3] * dt / eta_p
        fd = float(mpc.fold_effective(ac, fdcd, T_a, T))
        s = rk4_step(lambda tt, ss: derivatives(tt, ss, ac, env, cl_a, mu_a, True, thrust=T_a, fold_dcd=fd)[0], t, s, dt)
        if not np.all(np.isfinite(s)):
            out["ended"] = "non-finite"
            break
    out.update({"t_s": float(t), "h_min_m": float(h_min), "tip_min_m": float(tip_min), "tip_depth_max_m": float(depth),
                "wet_s": float(wet_s), "v_min_over_stall": float(v_min / ac.v_stall), "gamma_max_deg": float(np.degrees(g_max)),
                "loop_s": float(loop_s), "energy_j": float(energy), "h_end_m": float(h)})
    return out


def dive_case(ac, h, gamma_deg, vf, bank_deg):
    g = np.radians(gamma_deg)
    s0 = np.array([0.0, 0.0, h, vf * ac.v_stall, g, 0.0])
    kt = np.linspace(0.0, 10.0, 41)
    plan = {"knots_t": kt, "cl": np.full(41, DIVE_CL_FRAC * ac.cl_best_ld), "mu": np.full(41, np.radians(bank_deg)),
            "thrust": np.zeros(41)}
    return s0, plan


def contact_case(ac, env, bank_deg, gap, vf, theta0):
    """A level banked turn whose low wingtip is gap above the swell at phase theta0 (rad, on the forward face of a crest)
    flying against the swell's travel, so that the crest rises under it; the bank and the lift coefficient of a level
    turn held."""
    from sim import mpc
    k, om, dx, dy, ph, a = mpc.world_swell(env)
    mu = np.radians(bank_deg)
    V = vf * ac.v_stall
    # the aircraft at the origin flies against the swell's travel (heading opposite to (dx, dy)); place the phase by
    # shifting the start along the travel
    x0 = (theta0 - ph) / k * dx
    y0 = (theta0 - ph) / k * dy
    eta = float(env.surface(x0, y0, 0.0)[0])
    z = eta + gap + 0.5 * ac.span * np.sin(mu)
    psi = float(np.arctan2(-dy, -dx))
    s0 = np.array([x0, y0, z, V, 0.0, psi])
    q = 0.5 * ac.rho * V * V * ac.wing_area
    cl = float(np.clip(ac.mass * G / (np.cos(mu) * q), ac.cl_min, ac.cl_max))
    kt = np.linspace(0.0, 10.0, 41)
    plan = {"knots_t": kt, "cl": np.full(41, cl), "mu": np.full(41, mu), "thrust": np.zeros(41)}
    return s0, plan


def reflex_cases():
    """(family, aircraft, world of the prediction, parameters) of the constructed cases."""
    out = []
    for name in REFLEX_AIRCRAFT:
        for h in DIVE_GRID["h_m"]:
            for gd in DIVE_GRID["gamma_deg"]:
                for vf in DIVE_GRID["v_over_stall"]:
                    for b in DIVE_GRID["bank_deg"]:
                        out.append(("dive", name, "truth", (h, gd, vf, b)))
        for w in ("truth", "flat"):
            for b in CONTACT_GRID["bank_deg"]:
                for gap in CONTACT_GRID["tip_gap_m"]:
                    for vf in CONTACT_GRID["v_over_stall"]:
                        for th in CONTACT_GRID["theta0"]:
                            out.append(("contact", name, w, (b, gap, vf, th)))
    return out


def run_reflex_case(case):
    from sim.aircraft_models import by_name
    from sim import mpc
    from sim.seawind import Environment, Wind
    from sim.wavelib import swell_env
    fam, name, wname, par = case
    ac = by_name(name)
    if fam == "dive":
        env = Environment.flat(Wind(REFLEX_U10, 270.0, roughness="coare35"))
        s0, plan = dive_case(ac, *par)
        world = env
    else:
        env = swell_env(REFLEX_U10, 3.0, 10.0, "along", "p63")
        s0, plan = contact_case(ac, env, *par)
        world = env if wname == "truth" else mpc._flat_world(env, REFLEX_U10)
    res = {v: reflex_trial(ac, env, world, s0, plan, v) for v in REFLEX_VERSIONS}
    return {"family": fam, "aircraft": name, "world": wname, "params": list(par), "results": res}


def reflex_summary(rows):
    """Counts per family and version: contacts, firings, rescues (a contact without a reflex avoided with it), false
    firings (a firing in a case that does not end in contact without a reflex), loops and the deepest immersion."""
    out = {}
    for fam in ("dive", "contact"):
        for w in ("truth", "flat"):
            rs = [r for r in rows if r["family"] == fam and r["world"] == w]
            if not rs:
                continue
            base = {id(r): r["results"]["none"]["ended"] != "complete" for r in rs}
            for v in REFLEX_VERSIONS:
                res = [r["results"][v] for r in rs]
                k = "%s_%s_%s" % (fam, w, v)
                out[k] = {"cases": len(rs), "contacts": sum(x["ended"] != "complete" for x in res),
                          "fired": sum(x["firings"] > 0 for x in res),
                          "rescues": sum(base[id(r)] and r["results"][v]["ended"] == "complete" for r in rs),
                          "false_firings": sum((not base[id(r)]) and r["results"][v]["firings"] > 0 for r in rs),
                          "base_contacts": sum(base.values()),
                          "loops": sum(x["loop_s"] > 0.0 for x in res),
                          "gamma_max_deg": float(max(x["gamma_max_deg"] for x in res)),
                          "v_min_over_stall": float(min(x["v_min_over_stall"] for x in res)),
                          "tip_depth_max_m": float(max(x["tip_depth_max_m"] for x in res)),
                          "touched": sum(x["tip_depth_max_m"] > 0.0 for x in res)}
    return out


def reflex_exp_main(a):
    import json
    import os
    import time
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    here = os.path.dirname(__file__)
    t0 = time.time()
    cases = reflex_cases()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_reflex_case, cases, chunksize=4))
    summ = reflex_summary(rows)
    for k, v in summ.items():
        print(k, v)
    meta = record.meta("python -m sim.mpc --reflex-exp --workers %d" % a.workers,
                       [os.path.join(here, "mpc.py"), __file__, os.path.join(here, "skid.py")])
    meta.update({"runtime_s": time.time() - t0, "trial_s": REFLEX_TRIAL_S, "dive_grid": DIVE_GRID,
                 "contact_grid": CONTACT_GRID, "dive_cl_frac": DIVE_CL_FRAC, "u10": REFLEX_U10,
                 "how": "a stale plan held from a constructed state (see the module notes of sim/reflex.py); flat COARE "
                        "3.5 sea for the dives, a regular swell (Hs 3 m, Tp 10 s, along a 9 m/s wind, P63) for the "
                        "contacts; the end at a contact of the centre or the wingtip deeper than skid.blade_depth_m"})
    cur = json.load(open(a.out)) if os.path.exists(a.out) else {}
    cur["reflex_exp"] = {"meta": meta, "summary": summ, "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(cur, fh, indent=1)
        fh.write(chr(10))
