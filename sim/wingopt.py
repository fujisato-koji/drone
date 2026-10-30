"""Wing sizing study for the dynamic-soaring drone: span, aspect ratio, taper and airfoil.

Design model: a sourced-form drag polar (airfoil fits to UIUC low-speed data with a Reynolds
number scaling, Nita and Scholz span efficiency, parasitic drag area) and a mass model (record
component masses plus an areal wing mass, a spar cap and a joiner, solved consistently with the
weight). Objective A: the soaring minimum wind (sim.dsopt min_wind, travel, flat sea, COARE 3.5,
planner margins). Objective B: the upwind VMG at u10 13 m/s.

Command: python -m sim.wingopt   (writes sim/out/wingopt.json and prints the tables).
"""
from __future__ import annotations

import os

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_k] = "1"

import argparse
import json
import sys
import time
from dataclasses import dataclass, replace

import numpy as np

from sim.aircraft import G, Aircraft
from sim.record import value
from sim.seawind import RHO_AIR

OUT_PATH = os.path.join(os.path.dirname(__file__), "out", "wingopt.json")
MU_AIR = 1.789e-5            # Pa s, ISA sea level
NU_AIR = MU_AIR / RHO_AIR    # m2/s
RE_REF = 2.0e5
RE_MIN = 8.0e4               # infeasible below this at 1.2 Vs (outside the data)
CL_MAX_KNOCKDOWN = 0.1       # cl_max (3D) = airfoil cl_max - 0.1
FUSELAGE_D = 0.06            # m
CDA_BASE, CDA_VARIANT = 10e-4, 20e-4   # m2 (10 and 20 cm2)
RHO_A = {"base": 1.5, "light": 0.95, "heavy": 2.2}   # kg/m2
SPAR = {"rho": 1400.0, "sigma": 300e6, "k": 0.85, "tc": 0.09}
JOINER_PER_M, JOINER_ABOVE = 0.05, 2.0   # kg per metre of span above 2 m (assumed)
COMPONENT_KEYS = ("battery.mass_kg", "avionics.mass_kg", "camera.mass_kg", "pod.mass_kg",
                  "boom.mass_kg", "tail.mass_kg")
MARGINS = dict(h_min=2.0, n_frac=0.8, mu_frac=0.9, v_factor=1.2)   # = sim.mpc.MARGIN_SETS[0]
U10_B = 13.0
COURSES_B = (105.0, 120.0, 135.0)   # deg from downwind; chi = 180 is straight upwind
N_NODES = 40
SCALE_TOL = 0.05

# (a0, a1, a2, n, a8, cl_max at Re 200k)
AIRFOILS = {
    "SD7037": (0.01061, -0.00677, 0.01036, 0.59, 0.00135, 1.26),
    "AG35-r": (0.00828, 0.00150, 0.00400, 0.59, 0.00641, 1.14),
    "MH32": (0.01162, -0.00785, 0.00901, 0.61, 0.00564, 1.18),
    "SD7062": (0.01373, -0.00121, 0.00249, 0.72, 0.00072, 1.63),
}


@dataclass(frozen=True)
class Design:
    span: float
    ar: float
    taper: float = 0.45
    airfoil: str = "SD7037"
    rho_a: str = "base"
    cda: float = CDA_BASE
    name: str = ""
    waive_re: bool = False   # candidates only: skip the Re 80k floor (reported, not hidden)

    @property
    def key(self):
        return (self.name or f"b{self.span:g}-ar{self.ar:g}") + \
            f"|{self.airfoil}|t{self.taper:g}|{self.rho_a}|cda{self.cda * 1e4:g}" +             ("|waive" if self.waive_re else "")


def airfoil_cd(name, cl, re):
    a0, a1, a2, n, a8, _ = AIRFOILS[name]
    return (a0 + a1 * cl + a2 * cl * cl) * (re / RE_REF) ** (-n) + a8 * cl ** 8


def span_efficiency(ar, taper, span):
    """Nita and Scholz 2012: e = k_eF / (1 + f(lambda) AR)."""
    lam = taper
    f = 0.0524 * lam ** 4 - 0.15 * lam ** 3 + 0.1659 * lam ** 2 - 0.0706 * lam + 0.0119
    return (1.0 - 2.0 * (FUSELAGE_D / span) ** 2) / (1.0 + f * ar)


def other_mass():
    """Everything except the wing: the sum of the record's component masses (kg)."""
    return float(sum(value(k) for k in COMPONENT_KEYS))


def mass_model(span, ar, rho_a_kgm2, m_other=None, n_ult=None):
    """Consistent mass: m_wing = rho_A S + m_spar(W) + m_joiner with W = (m_other + m_wing) g.

    m_spar = 2 rho n_ult W b AR / (9 pi sigma k t/c) is linear in W, so the fixed point is closed
    form. Returns a dict (kg)."""
    m_other = other_mass() if m_other is None else m_other
    n_ult = value("ds.n_limit", 4.0) * value("ds.safety_factor", 1.5) if n_ult is None else n_ult
    area = span ** 2 / ar
    c = 2.0 * SPAR["rho"] * n_ult * G * span * ar / (9.0 * np.pi * SPAR["sigma"] * SPAR["k"] * SPAR["tc"])
    joiner = JOINER_PER_M * max(span - JOINER_ABOVE, 0.0)
    skin = rho_a_kgm2 * area
    m_wing = (skin + joiner + c * m_other) / (1.0 - c)
    mass = m_other + m_wing
    return {"mass": float(mass), "m_wing": float(m_wing), "m_skin": float(skin),
            "m_spar": float(c * mass), "m_joiner": float(joiner), "m_other": float(m_other),
            "area": float(area), "n_ult": float(n_ult)}


def stall_speed(mass, area, cl_max, rho=RHO_AIR):
    return float(np.sqrt(2.0 * mass * G / (rho * area * cl_max)))


def basics(d: Design):
    mm = mass_model(d.span, d.ar, RHO_A[d.rho_a])
    cl_max = AIRFOILS[d.airfoil][5] - CL_MAX_KNOCKDOWN
    chord = mm["area"] / d.span
    vs = stall_speed(mm["mass"], mm["area"], cl_max)
    return mm, cl_max, chord, vs


def re_of(v, chord):
    return v * chord / NU_AIR


def feasibility(d: Design):
    """(ok, reason) from the span and mass limits and the Reynolds floor at 1.2 Vs."""
    mm, _, chord, vs = basics(d)
    smax, mmax = value("ds.span_max_m", None), value("ds.mass_max_kg", None)
    if smax is not None and d.span > smax + 1e-9:
        return False, f"span {d.span:g} > ds.span_max_m {smax:g}"
    if mmax is not None and mm["mass"] > mmax + 1e-9:
        return False, f"mass {mm['mass']:.2f} > ds.mass_max_kg {mmax:g}"
    re12 = re_of(1.2 * vs, chord)
    if re12 < RE_MIN and not d.waive_re:
        return False, f"Re {re12:.0f} at 1.2 Vs below {RE_MIN:.0f}"
    return True, ""


def build_aircraft(d: Design, re_ref):
    """Aircraft with the airfoil polar scaled to Reynolds number re_ref (mean chord)."""
    mm, cl_max, chord, vs = basics(d)
    a0, a1, a2, n, a8, _ = AIRFOILS[d.airfoil]
    s = (re_ref / RE_REF) ** (-n)
    area = mm["area"]
    e = span_efficiency(d.ar, d.taper, d.span)
    return Aircraft(d.key, mass=mm["mass"], span=d.span, wing_area=area, cd0=a0 * s + d.cda / area,
                    oswald=e, cl_max=cl_max, n_max=value("ds.n_limit", 4.0),
                    cdp1=a1 * s, cdp2=a2 * s, cdp8=a8)


def mean_airspeed(col, x):
    S, _, T, _ = col.unpack(x)
    return float(np.trapezoid(S[:, 3], dx=1.0) / (len(S) - 1))   # nodes are equally spaced in time


def _scale(re, n):
    return (re / RE_REF) ** (-n)


def solve_a(d: Design, n=N_NODES, seeds=(0, 1, 2)):
    """Minimum soaring wind with the Reynolds iteration. Returns a result dict."""
    from sim.dsopt import solve_min_wind
    mm, cl_max, chord, vs = basics(d)
    nexp = AIRFOILS[d.airfoil][3]
    kw = dict(z0_mode="self_consistent", roughness="coare35", cycle="travel", n=n,
              tip_model="geometric", midpoints=True, rate_limits=True, rate_model="pb2v",
              n_min=0.0, replay_check=True, seeds=seeds, stop_after=1, return_x=True, **MARGINS)
    re = re_of(1.3 * vs, chord)
    hist, x0 = [], None
    for _ in range(2):
        ac = build_aircraft(d, re)
        info, x, col = solve_min_wind(ac, x0=x0, **kw)
        if not info["success"]:
            hist.append({"re": re, "success": False})
            return {"success": False, "history": hist, "mass": mm["mass"]}
        v = mean_airspeed(col, x)
        re_new = re_of(v, chord)
        hist.append({"re": re, "u10": info["u10"], "v_mean": v, "re_cycle": re_new})
        if abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL:
            break
        re, x0 = re_new, x
    ac = build_aircraft(d, re)
    pr = ac
    return {"success": True, "u10": float(info["u10"]), "re_used": re, "re_cycle": hist[-1]["re_cycle"],
            "v_mean": hist[-1]["v_mean"], "period": float(info["period"]), "n_max": float(info["n_max"]),
            "history": hist, "mass": mm["mass"], "m_wing": mm["m_wing"], "m_spar": mm["m_spar"],
            "area": mm["area"], "wing_loading": mm["mass"] * G / mm["area"], "oswald": ac.oswald,
            "ld_max": pr.best_ld, "cl_ld": pr.cl_best_ld, "v_stall": ac.v_stall,
            "cl_max": ac.cl_max, "chord": chord, "replay_pass": info.get("replay_pass"),
            "cd0_eff": ac.cd0, "cdp1": ac.cdp1, "cdp2": ac.cdp2, "cdp8": ac.cdp8,
            "outer_converged": info.get("outer_converged")}


def solve_b(d: Design, re_start, u10=U10_B, n=N_NODES, seeds=(0, 1, 2)):
    """Upwind VMG at u10 13 m/s: best of the three courses, with one Reynolds iteration."""
    from sim.dsopt import profile_for, solve
    nexp = AIRFOILS[d.airfoil][3]
    chord = basics(d)[2]
    prof = profile_for(u10, "log", roughness="coare35")

    def course(ac, chi, x0):
        info, x, col = solve(ac, prof, mode="max_speed", cycle="travel", n=n, u10=u10,
                             chi=np.radians(chi), tip_model="geometric", midpoints=True,
                             rate_limits=True, rate_model="pb2v", n_min=0.0, replay_check=True,
                             seeds=seeds, x0=x0, return_x=True, stop_after=1, **MARGINS)
        return info, (np.asarray(x) if info["success"] else None), col

    def sweep(ac, warm):
        out, xs = {}, dict(warm)
        for chi in (120.0, 105.0, 135.0):
            info, x, col = course(ac, chi, xs.get(chi) if xs.get(chi) is not None else xs.get(120.0))
            if x is None and xs:
                info, x, col = course(ac, chi, None)
            if x is not None:
                xs[chi] = x
                out[chi] = (info, x, col)
        return out, xs

    re, warm, res = re_start, {}, {}
    for _ in range(2):
        ac = build_aircraft(d, re)
        res, warm = sweep(ac, warm)
        if not res:
            return {"success": False, "re_used": re}
        best = max(res, key=lambda c: res[c][0]["speed_along_chi"] * -np.cos(np.radians(c)))
        v = mean_airspeed(res[best][2], res[best][1])
        re_new = re_of(v, chord)
        if abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL:
            break
        re = re_new
    cases = {f"{c:g}": {"progress": float(i["speed_along_chi"]),
                        "vmg_up": float(i["speed_along_chi"] * -np.cos(np.radians(c)))}
             for c, (i, _, _) in res.items()}
    vmg = max(v["vmg_up"] for v in cases.values())
    return {"success": True, "vmg_up": float(vmg), "best_course": best, "cases": cases,
            "re_used": re, "v_mean": v}


def evaluate(job):
    """Worker: job = (kind, Design, extra). kind "A" or "B"."""
    kind, d, extra = job
    t0 = time.time()
    ok, why = feasibility(d)
    if not ok:
        return {"kind": kind, "key": d.key, "design": d.__dict__, "success": False, "reason": why}
    try:
        r = solve_a(d) if kind == "A" else solve_b(d, *extra)
    except Exception as exc:   # one bad solve must not stop the study
        r = {"success": False, "reason": f"exception {type(exc).__name__}: {exc}"}
    r.update({"kind": kind, "key": d.key, "extra": extra if kind == "B" else None, "design": d.__dict__, "runtime_s": time.time() - t0})
    print(f"done {kind} {d.key} u10={r.get('u10', r.get('vmg_up'))} {r['runtime_s']:.0f} s",
          file=sys.stderr, flush=True)
    return r


# ---- current candidates, rebuilt with the new polar and mass model
CANDIDATES = {"albatross-3m": (3.0, 0.36), "baseline-2m": (2.0, 0.25), "sealedhull-2p5m": (2.5, 0.34)}


def candidate_designs():
    return [Design(b, b * b / s, name=n, waive_re=True) for n, (b, s) in CANDIDATES.items()]


def grid(spans, ars, tapers=(0.45,), airfoils=tuple(AIRFOILS)):
    return [Design(b, ar, t, af) for b in spans for ar in ars for t in tapers for af in airfoils]


def run(workers, spans, ars, top, out):
    from concurrent.futures import ProcessPoolExecutor
    t0 = time.time()
    smax = value("ds.span_max_m", None)
    designs = grid(spans, ars)
    study = []
    with ProcessPoolExecutor(workers) as ex:
        ra = list(ex.map(evaluate, [("A", d, None) for d in designs + candidate_designs()], chunksize=1))
        okA = sorted((r for r in ra if r["success"] and r["design"]["name"] == ""),
                     key=lambda r: r["u10"])
        topd = [Design(**r["design"]) for r in okA[:top]]
        cand = [Design(**r["design"]) for r in ra if r["design"]["name"] and r["success"]]
        re0 = {r["key"]: r["re_used"] for r in ra if r["success"]}
        jobs = []
        for d in topd:   # B at 13 m/s and the sensitivities
            jobs.append(("B", d, (re0[d.key], U10_B)))
        for d in cand:   # a candidate is run at 13 m/s only if it soars there
            if next(r for r in ra if r["key"] == d.key)["u10"] < U10_B:
                jobs.append(("B", d, (re0[d.key], U10_B)))
        for d in topd:
            for kind, dd in (("taper0.3", replace(d, taper=0.3)), ("taper0.6", replace(d, taper=0.6)),
                             ("rho_a0.95", replace(d, rho_a="light")), ("rho_a2.2", replace(d, rho_a="heavy")),
                             ("cda20", replace(d, cda=CDA_VARIANT))):
                jobs.append(("A", dd, kind))
        rb = list(ex.map(evaluate, [(k, d, e if k == "B" else None) for k, d, e in jobs], chunksize=1))
    for r, (k, d, e) in zip(rb, jobs):
        if k == "A":
            r["variant"] = e
            r["base_key"] = replace(d, taper=0.45, rho_a="base", cda=CDA_BASE).key
    here = os.path.dirname(__file__)
    from sim import record
    meta = record.meta("python -m sim.wingopt", [__file__, os.path.join(here, "dsopt.py"),
                                                 os.path.join(here, "aircraft.py")])
    meta.update({"runtime_s": time.time() - t0, "spans": list(spans), "ars": list(ars),
                 "span_max_m": smax, "mass_max_kg": value("ds.mass_max_kg", None),
                 "n_limit": value("ds.n_limit", 4.0), "safety_factor": value("ds.safety_factor", 1.5),
                 "component_keys": list(COMPONENT_KEYS), "m_other_kg": other_mass(),
                 "margins": MARGINS, "n_nodes": N_NODES, "u10_B": U10_B, "courses_B": list(COURSES_B),
                 "frame": "chi ccw from the wind direction: 0 downwind, 180 straight upwind",
                 "dsopt": "min_wind travel, self_consistent COARE 3.5, geometric tip 0.3, midpoints, "
                          "pb2v, n_min 0, replay_check, seeds 0-2 stop_after 1"})
    os.makedirs(os.path.dirname(out), exist_ok=True)
    payload = {"meta": meta, "A": ra, "B_and_variants": rb}
    with open(out, "w", newline="\n") as fh:
        json.dump(payload, fh, indent=1)
        fh.write("\n")
    return payload


def supplement(path, u10s=(16.0,), workers=4):
    """Add to an existing result: B at the extra winds for the top designs and the candidates.

    Needed because no design soars at 13 m/s with the standard margins (min wind above 13). Written
    under the key "B_extra"; the main arrays are untouched."""
    from concurrent.futures import ProcessPoolExecutor
    with open(path) as fh:
        p = json.load(fh)
    t0 = time.time()
    with ProcessPoolExecutor(workers) as ex:
        if not any(r["design"]["name"] and r["success"] for r in p["A"]):   # candidates, Re floor waived
            new = list(ex.map(evaluate, [("A", d, None) for d in candidate_designs()], chunksize=1))
            p["A"] = [r for r in p["A"] if not r["design"]["name"]] + new
        top = [r for r in p["A"] if r["success"] and not r["design"]["name"]]
        top = sorted(top, key=lambda r: r["u10"])[:10]
        cand = [r for r in p["A"] if r["success"] and r["design"]["name"]]
        jobs = [("B", Design(**r["design"]), (r["re_used"], u)) for u in u10s for r in top + cand]
        p["B_extra"] = list(ex.map(evaluate, jobs, chunksize=1))
    p["meta"]["supplement_runtime_s"] = time.time() - t0
    p["meta"]["supplement_u10"] = list(u10s)
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")
    return p


def tables(p):
    ra = p["A"]
    rows = sorted((r for r in ra if r["success"] and not r["design"]["name"]), key=lambda r: r["u10"])
    bmap = {r["key"]: r for r in p["B_and_variants"] if r["kind"] == "B"}
    L = ["| rank | span | AR | airfoil | mass kg | W/S N/m2 | L/D max | Re used | u10 min | VMG up |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(rows, 1):
        d = r["design"]
        b = bmap.get(r["key"])
        vm = f"{b['vmg_up']:.2f}" if b and b["success"] else ("-" if not b else "fail")
        L.append(f"| {i} | {d['span']:g} | {d['ar']:g} | {d['airfoil']} | {r['mass']:.2f} | "
                 f"{r['wing_loading']:.1f} | {r['ld_max']:.1f} | {r['re_used'] / 1e3:.0f}k | "
                 f"{r['u10']:.2f} | {vm} |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--spans", default="2.0,2.5,3.0,3.5")
    ap.add_argument("--ars", default="12,16,20,25,30")
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--out", default=OUT_PATH)
    ap.add_argument("--supplement", default="", help="comma list of extra B winds; adds to --out and exits")
    a = ap.parse_args(argv)
    if a.supplement:
        supplement(a.out, tuple(float(u) for u in a.supplement.split(",")), a.workers)
        return
    spans = tuple(float(s) for s in a.spans.split(","))
    ars = tuple(float(s) for s in a.ars.split(","))
    p = run(a.workers, spans, ars, a.top, a.out)
    print(tables(p))
    print(f"runtime {p['meta']['runtime_s']:.0f} s")


if __name__ == "__main__":
    main()
