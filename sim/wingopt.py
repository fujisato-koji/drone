"""Wing sizing study for the dynamic-soaring drone: span, aspect ratio, taper, airfoil and wing loading.

Design model: a sourced-form drag polar (airfoil fits to UIUC low-speed data with a Reynolds
number scaling, Nita and Scholz span efficiency with their taper offset, a body drag area built up
from pod, boom, tails and linkages) and a mass model (record component masses plus ballast plus an
areal wing mass, a spar cap and a joiner, solved consistently with the weight).

Objectives for each design and ballast (second pass, Q-067):
  A  the soaring minimum wind (sim.dsopt min_wind, travel, flat sea, COARE 3.5, planner margins);
  B  the support value h(180) of the achievable mean ground velocities at u10 16 m/s (the speed
     made good straight upwind; negative = the least downwind drift; first pass at courses 105 to
     150 deg gave "progress" values that are not speeds made good, see solve_b);
  C  the average electric power of a motor-assisted travel cycle at 6 and 9 m/s (dsopt min_power
     with the motor, the control library's margins);
  D  the energy per kilometre of that cycle at 9 m/s.

Command: python -m sim.wingopt   (writes sim/out/wingopt2.json and prints the tables).
"""
from __future__ import annotations

import os

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_k] = "1"

import argparse
import json
import sys
import time
from dataclasses import dataclass

import numpy as np

from sim.aircraft import G, Aircraft
from sim.record import value
from sim.seawind import RHO_AIR

OUT_PATH = os.path.join(os.path.dirname(__file__), "out", "wingopt2.json")
FIRST_PASS_PATH = os.path.join(os.path.dirname(__file__), "out", "wingopt.json")
MU_AIR = 1.789e-5            # Pa s, ISA sea level
NU_AIR = MU_AIR / RHO_AIR    # m2/s
RE_REF = 2.0e5
RE_MIN = 8.0e4               # infeasible below this at 1.2 Vs (outside the data)
CL_MAX_KNOCKDOWN = 0.1       # cl_max (3D) = airfoil cl_max - 0.1
FUSELAGE_D = 0.06            # m
CDA_BASE, CDA_VARIANT = 10e-4, 20e-4   # m2, the first pass's fixed body drag areas (10 and 20 cm2)
RHO_A = {"base": 1.5, "light": 0.95, "heavy": 2.2}   # kg/m2
SPAR = {"rho": 1400.0, "sigma": 300e6, "k": 0.85, "tc": 0.09}
JOINER_PER_M, JOINER_ABOVE = 0.05, 2.0   # kg per metre of span above 2 m (assumed)
COMPONENT_KEYS = ("battery.mass_kg", "avionics.mass_kg", "camera.mass_kg", "pod.mass_kg",
                  "boom.mass_kg", "tail.mass_kg")
MARGINS = dict(h_min=2.0, n_frac=0.8, mu_frac=0.9, v_factor=1.2)   # = sim.mpc.MARGIN_SETS[0]
MARGINS_LIGHT = dict(h_min=1.5, n_frac=0.9, mu_frac=0.95, v_factor=1.15)   # = MARGIN_SETS[1]
MARGIN_SETS = (("standard", MARGINS), ("light", MARGINS_LIGHT))
MOTOR_SEEDS = ((0, 1),)   # sim.mpc.MOTOR_SEEDS is (0, 1, 2) then (3, 4, 5); cut for run time
MOTOR_MAXITER = 200                    # sim.mpc.MOTOR_MAXITER is 400; cut for run time (a failed solve costs most)
SEEDS_A = (0, 1, 2, 3, 4, 5)
U10_B = 16.0
COURSES_B = (120.0, 105.0, 135.0, 150.0)   # deg from downwind; chi = 180 is straight upwind
U10_C = (6.0, 9.0)
CHI_C = 20.0                 # deg: the motor cycle's course (near downwind, as the ground-motor study; crosswind often fails to converge; the spec fixes none)
N_NODES = 40
SCALE_TOL = 0.05
BALLAST_GRID = (0.0, 0.5)   # kg, plus the mass limit (the run of the record; --ballast 0,0.5,1.0 adds 1.0)

# Body drag build-up (P145, S7, S8; the choices the spec leaves are marked "assumed")
POD_L, POD_D = 1.0, 0.07           # m
BOOM_D = 0.012                     # m
TAIL_ARM_PER_SPAN = 0.33           # tail arm (wing to tail quarter chords) / span (assumed)
VH, VV = 0.4, 0.02                 # tail volume coefficients (horizontal on mean chord, vertical on span)
AR_H, AR_V = 6.0, 1.5              # tail aspect ratios (assumed)
TAIL_AIRFOIL = "SD7037"
GAPS_FACTOR = 1.10                 # gaps, hinges and linkages

# Nita and Scholz 2012 (P156) eq. 37: delta_lambda = -0.357 + 0.45 exp(0.0375 phi_25), phi_25 in deg
NS_DLAMBDA_UNSWEPT = -0.357 + 0.45   # = 0.093

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
    cda: float | None = None   # m2; None: the body drag build-up (body_drag), a number: fixed area
    ballast: float = 0.0       # kg added at the fuselage (counted in the spar's load)
    name: str = ""
    waive_re: bool = False     # candidates only: skip the Re 80k floor (reported, not hidden)

    @property
    def key(self):
        cda = "bu" if self.cda is None else f"{self.cda * 1e4:g}"
        return (self.name or f"b{self.span:g}-ar{self.ar:g}") + \
            f"|{self.airfoil}|t{self.taper:g}|{self.rho_a}|cda{cda}|bal{self.ballast:.3f}" + \
            ("|waive" if self.waive_re else "")


def airfoil_cd(name, cl, re):
    a0, a1, a2, n, a8, _ = AIRFOILS[name]
    return (a0 + a1 * cl + a2 * cl * cl) * (re / RE_REF) ** (-n) + a8 * cl ** 8


def span_efficiency(ar, taper, span):
    """Nita and Scholz 2012 eq. 36-38 and 40: e = k_eF / (1 + f(lambda - d_lambda) AR), with the
    unswept offset d_lambda = -0.357 + 0.45 = 0.093 (eq. 37 at zero sweep) and k_eF the fuselage
    factor. Their further factors k_eD0 and k_eM (category statistics, Mach) are not applied."""
    lam = taper - NS_DLAMBDA_UNSWEPT
    f = 0.0524 * lam ** 4 - 0.15 * lam ** 3 + 0.1659 * lam ** 2 - 0.0706 * lam + 0.0119
    return (1.0 - 2.0 * (FUSELAGE_D / span) ** 2) / (1.0 + f * ar)


def other_mass():
    """Everything except the wing: the sum of the record's component masses (kg)."""
    return float(sum(value(k) for k in COMPONENT_KEYS))


def mass_model(span, ar, rho_a_kgm2, m_other=None, n_ult=None):
    """Consistent mass: m_wing = rho_A S + m_spar(W) + m_joiner with W = (m_other + m_wing) g.

    m_spar = 2 rho n_ult W b AR / (9 pi sigma k t/c) is linear in W, so the fixed point is closed
    form. m_other includes any ballast. Returns a dict (kg)."""
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


def ballast_to_mass(span, ar, rho_a_kgm2, mass_target):
    """Ballast (kg) that brings the consistent mass to mass_target (mass is linear in m_other)."""
    m0 = mass_model(span, ar, rho_a_kgm2)
    c = m0["m_spar"] / m0["mass"]
    m_other = mass_target * (1.0 - c) - m0["m_skin"] - m0["m_joiner"]
    return float(m_other - other_mass())


def ballast_grid(span, ar, rho_a="base", grid=None, margin=1e-3):
    """Ballast values: the grid entries below the mass limit, then the limit itself (dropped if
    within 0.1 kg of the last grid entry)."""
    grid = BALLAST_GRID if grid is None else grid
    mmax = value("ds.mass_max_kg", None)
    m0 = mass_model(span, ar, RHO_A[rho_a])["mass"]
    out = [b for b in grid if mmax is None or m0 + b < mmax - 0.1]
    if mmax is not None and m0 < mmax - 0.05:
        top = ballast_to_mass(span, ar, RHO_A[rho_a], mmax - margin)
        if not out or top > out[-1] + 0.1:
            out.append(top)
    return out


def stall_speed(mass, area, cl_max, rho=RHO_AIR):
    return float(np.sqrt(2.0 * mass * G / (rho * area * cl_max)))


def basics(d: Design):
    mm = mass_model(d.span, d.ar, RHO_A[d.rho_a], m_other=other_mass() + d.ballast)
    cl_max = AIRFOILS[d.airfoil][5] - CL_MAX_KNOCKDOWN
    chord = mm["area"] / d.span
    vs = stall_speed(mm["mass"], mm["area"], cl_max)
    return mm, cl_max, chord, vs


def re_of(v, chord):
    return v * chord / NU_AIR


def _cf(re):
    """Turbulent flat-plate skin friction (Prandtl and Schlichting), incompressible."""
    return 0.455 / np.log10(re) ** 2.58


def _form_factor(fr):
    """Hoerner body form factor 1 + 1.5/FR^1.5 + 7/FR^3 (FR = length / diameter)."""
    return 1.0 + 1.5 / fr ** 1.5 + 7.0 / fr ** 3


def body_drag(span, area, v, tail_airfoil=TAIL_AIRFOIL):
    """Parasite drag area (m2) of everything but the wing, at airspeed v (m/s), with the parts.

    Pod: turbulent flat plate at the pod's Reynolds number, Hoerner form factor, wetted area of a
    body of revolution pi D L (1 - 2/FR)^(2/3) (1 + 1/FR^2) (Torenbeek's fuselage wetted
    area; the record's S7 gives the skin friction and the Hoerner form factor, not this formula, and the
    textbook is not in the record). Boom: the same on a cylinder.
    Tails: planform from the volume coefficients on the mean chord (horizontal, VH) and the span
    (vertical, VV) at the arm TAIL_ARM_PER_SPAN * span, profile drag at cl 0 of the tail airfoil's
    fit at the tail's Reynolds number (extrapolated below the data); induced drag of the tails is
    not counted. Sum plus GAPS_FACTOR for gaps and linkages."""
    chord = area / span
    fr = POD_L / POD_D
    swet = np.pi * POD_D * POD_L * (1.0 - 2.0 / fr) ** (2.0 / 3.0) * (1.0 + 1.0 / fr ** 2)
    pod = _cf(re_of(v, POD_L)) * _form_factor(fr) * swet
    arm = TAIL_ARM_PER_SPAN * span
    boom_len = max(arm - 0.5 * POD_L, 0.1)    # from the pod's tail end to the tail
    boom = _cf(re_of(v, boom_len)) * _form_factor(boom_len / BOOM_D) * np.pi * BOOM_D * boom_len
    sh, sv = VH * area * chord / arm, VV * area * span / arm
    ch, cv = np.sqrt(sh / AR_H), np.sqrt(sv / AR_V)
    tail = airfoil_cd(tail_airfoil, 0.0, re_of(v, ch)) * sh + airfoil_cd(tail_airfoil, 0.0, re_of(v, cv)) * sv
    total = GAPS_FACTOR * (pod + boom + tail)
    return {"total": float(total), "pod": float(pod), "boom": float(boom), "tail": float(tail),
            "gaps": float((GAPS_FACTOR - 1.0) * (pod + boom + tail)), "boom_len": float(boom_len),
            "s_h": float(sh), "s_v": float(sv), "re_tail_h": float(re_of(v, ch)),
            "re_pod": float(re_of(v, POD_L))}


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
    """Aircraft with the airfoil polar scaled to Reynolds number re_ref (mean chord); the body
    drag area is evaluated at the airspeed that Reynolds number stands for."""
    mm, cl_max, chord, vs = basics(d)
    a0, a1, a2, n, a8, _ = AIRFOILS[d.airfoil]
    s = (re_ref / RE_REF) ** (-n)
    area = mm["area"]
    e = span_efficiency(d.ar, d.taper, d.span)
    cda = d.cda if d.cda is not None else body_drag(d.span, area, re_ref * NU_AIR / chord)["total"]
    return Aircraft(d.key, mass=mm["mass"], span=d.span, wing_area=area, cd0=a0 * s + cda / area,
                    oswald=e, cl_max=cl_max, n_max=value("ds.n_limit", 4.0),
                    cdp1=a1 * s, cdp2=a2 * s, cdp8=a8)


def glide_point(d: Design):
    """L/D at the design's own best-glide speed (Reynolds number iterated to that speed)."""
    chord = basics(d)[2]
    re = re_of(1.3 * basics(d)[3], chord)
    for _ in range(6):
        ac = build_aircraft(d, re)
        re = re_of(ac.v_best_ld, chord)
    ac = build_aircraft(d, re)
    return {"ld_glide": float(ac.best_ld), "v_glide": float(ac.v_best_ld), "re_glide": float(re)}


def mean_airspeed(col, x):
    S, _, T, _ = col.unpack(x)
    return float(np.trapezoid(S[:, 3], dx=1.0) / (len(S) - 1))   # nodes are equally spaced in time


def _scale(re, n):
    return (re / RE_REF) ** (-n)


def _common_info(d, mm, ac):
    return {"mass": mm["mass"], "m_wing": mm["m_wing"], "m_spar": mm["m_spar"], "area": mm["area"],
            "wing_loading": mm["mass"] * G / mm["area"], "oswald": ac.oswald, "ld_max": ac.best_ld,
            "cl_ld": ac.cl_best_ld, "v_stall": ac.v_stall, "cl_max": ac.cl_max,
            "chord": mm["area"] / d.span, "cd0_eff": ac.cd0, "cdp1": ac.cdp1, "cdp2": ac.cdp2,
            "cdp8": ac.cdp8}


def solve_a(d: Design, n=N_NODES, seeds=SEEDS_A):
    """Minimum soaring wind with the Reynolds iteration (two passes at most).

    All seeds are run in the first pass (the best converged one with a passing replay is kept);
    the second pass is warm-started from it. The result is the last pass that converged and the
    aircraft of that same pass (the first-pass version reported the aircraft of a Reynolds number
    that had not been solved when the second pass ended unconverged, and discarded a good
    first pass when the second failed)."""
    from sim.dsopt import solve_min_wind
    mm, cl_max, chord, vs = basics(d)
    nexp = AIRFOILS[d.airfoil][3]
    kw = dict(z0_mode="self_consistent", roughness="coare35", cycle="travel", n=n,
              tip_model="geometric", midpoints=True, rate_limits=True, rate_model="pb2v",
              n_min=0.0, replay_check=True, seeds=seeds, stop_after=len(seeds), return_x=True,
              **MARGINS)
    re = re_of(1.3 * vs, chord)
    hist, x0, best = [], None, None
    for _ in range(2):
        ac = build_aircraft(d, re)
        info, x, col = solve_min_wind(ac, x0=x0, **kw)
        if not info["success"]:
            hist.append({"re": re, "success": False})
            break
        v = mean_airspeed(col, x)
        re_new = re_of(v, chord)
        hist.append({"re": re, "u10": info["u10"], "v_mean": v, "re_cycle": re_new})
        best = (info, ac, re, v, re_new)
        if abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL:
            break
        re, x0 = re_new, x
    if best is None:
        return {"success": False, "history": hist, "mass": mm["mass"]}
    info, ac, re, v, re_new = best
    out = {"success": True, "u10": float(info["u10"]), "re_used": re, "re_cycle": re_new, "v_mean": v,
           "period": float(info["period"]), "n_max": float(info["n_max"]), "history": hist,
           "replay_pass": info.get("replay_pass"), "outer_converged": info.get("outer_converged"),
           "re_converged": abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL,
           "later_pass_failed": hist[-1].get("success") is False}
    out.update(_common_info(d, mm, ac))
    return out


def solve_b(d: Design, re_start, u10=U10_B, n=N_NODES, seeds=SEEDS_A, courses=COURSES_B):
    """Support values h(chi) of the achievable mean ground velocities at u10 (16 m/s), one max_speed
    solve per course (the first converged start of seeds 0-5), with one Reynolds iteration.

    dsopt max_speed maximises only the component of the mean ground velocity along chi and leaves
    the cross-track velocity free, so "progress" at chi is h(chi), not a speed along a track, and
    h(chi) cos(angle off the wind) is not a speed made good upwind (docs/02-form.md#navigation).
    The speed made good straight upwind is h(180) (negative: the least downwind drift)."""
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
        for chi in courses:
            info, x, col = course(ac, chi, xs.get(chi) if xs.get(chi) is not None else xs.get(courses[0]))
            if x is None and xs:
                info, x, col = course(ac, chi, None)
            if x is not None:
                xs[chi] = x
                out[chi] = (info, x, col)
        return out, xs

    re, warm, last = re_start, {}, None
    for _ in range(2):
        res, warm_new = sweep(build_aircraft(d, re), warm)
        if not res:
            break
        best = max(res, key=lambda c: res[c][0]["speed_along_chi"])
        v = mean_airspeed(res[best][2], res[best][1])
        re_new = re_of(v, chord)
        last = (res, best, re, v)
        warm = warm_new
        if abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL:
            break
        re = re_new
    if last is None:
        return {"success": False, "re_used": re}
    res, best, re, v = last
    cases = {f"{c:g}": {"h": float(i["speed_along_chi"])} for c, (i, _, _) in res.items()}
    return {"success": True, "h180": cases.get("180", {}).get("h"), "cases": cases,
            "re_used": re, "v_mean": v}


def steady_guess(ac, prof, u10, chi_deg, mg, n=N_NODES, period=8.0, z_extra=0.5):
    """Steady level flight as a starting point of the motor solve (a periodic cycle of the model).

    Constant height h_min + z_extra, wings level, flight-path angle 0, constant airspeed, heading
    along the course chi, thrust equal to the drag (fixed point, since the folded-propeller drag
    depends on the thrust). The airspeed is the speed of minimum electric power, but at least the
    speed floor v_factor * v_stall and at least what the mean speed along chi must be
    (v_travel_min = 0.5 v_best_ld, V + U(z) cos chi >= it). Returns (x, info) where info says
    whether the thrust is within thrust_max (prop.thrust_max_per_weight m g; the record has no
    power limit)."""
    from sim.dsopt import Collocation, _drag, prop_efficiency, thrust_max
    chi = np.radians(chi_deg)
    vtm = 0.5 * ac.v_best_ld
    col = Collocation(ac, prof, "min_power", "travel", n, u10=u10, chi=chi, h_tip_min=0.3,
                      tip_model="geometric", midpoints=True, rate_limits=True, rate_model="pb2v",
                      n_min=0.0, motor=True, v_travel_min=vtm, **mg)
    z = mg["h_min"] + z_extra
    U = u10 * float(prof.f(z))
    vmin = max(mg["v_factor"] * ac.v_stall, 1.0)
    W = ac.mass * G

    def level(V):
        q = 0.5 * ac.rho * V * V * ac.wing_area
        cl = W / q
        th = 0.0
        for _ in range(40):
            th = float(_drag(ac, q, cl, 0.0, z, False, th))
        return cl, th

    Vs = np.linspace(vmin, 30.0, 600)
    P = np.array([level(V)[1] * V for V in Vs])
    V_mp = float(Vs[int(np.argmin(P))])
    V_prog = vtm - U * np.cos(chi)          # V + U cos chi >= vtm
    V = max(V_mp, vmin, V_prog)
    cl, th = level(V)
    psi = chi
    t = np.linspace(0.0, period, n)
    dx = V * np.cos(psi) + U
    dy = V * np.sin(psi)
    S = np.stack([dx * t, dy * t, np.full(n, z), np.full(n, V), np.zeros(n), np.full(n, psi)], axis=1)
    C = np.stack([np.full(n, cl), np.zeros(n)], axis=1)
    x = col.pack(S, C, period, u10, 0.0, np.full(n, th))
    info = {"V": V, "V_min_power": V_mp, "V_floor": vmin, "V_progress": float(V_prog), "z": z, "cl": float(cl),
            "thrust_n": th, "thrust_max_n": float(thrust_max(ac)), "thrust_ok": bool(th <= thrust_max(ac)),
            "power_w": float(th * V / prop_efficiency()), "cl_ok": bool(ac.cl_min <= cl <= ac.cl_max),
            "progress": float(V + U * np.cos(chi)), "v_travel_min": float(vtm)}
    return x, info, col


def solve_c(d: Design, u10, re_start, chi_deg=CHI_C, n=N_NODES, seed_sets=None, maxiter=None, margin_labels=None, steady=True):
    """Motor-assisted travel cycle at u10: dsopt min_power with the motor (as the control library
    builds it: mean speed along the course at least 0.5 v_best_ld, standard margins then light,
    starts 0-1), with one Reynolds iteration. Returns the average electric power, the
    speed along the course, the energy per kilometre and the margin set that solved."""
    from sim.dsopt import motor_stats, profile_for, solve
    seed_sets = MOTOR_SEEDS if seed_sets is None else seed_sets
    maxiter = MOTOR_MAXITER if maxiter is None else maxiter
    nexp = AIRFOILS[d.airfoil][3]
    chord = basics(d)[2]
    prof = profile_for(max(u10, 1.0), "log", roughness="coare35")

    diag = {}

    def cycle(re_in, label=None, x0=None):
        """Returns (info, x, col, label, re_used). The steady start is built at its own Reynolds
        number (iterated to the level-flight airspeed) and solved with that aircraft; the other
        starts use the polar at re_in."""
        sets = [m for m in MARGIN_SETS if (label is None or m[0] == label) and (margin_labels is None or m[0] in margin_labels)]
        for lab, mg in sets:
            starts = [("warm", x0, re_in)] if x0 is not None else []
            if x0 is None and steady:
                re_s = re_in
                for _ in range(8):
                    ac_s = build_aircraft(d, re_s)
                    xs, sinfo, _ = steady_guess(ac_s, prof, u10, chi_deg, mg, n)
                    re_n = re_of(sinfo["V"], chord)
                    done = abs(re_n / re_s - 1.0) < 1e-3
                    re_s = re_n
                    if done:
                        break
                ac_s = build_aircraft(d, re_s)
                xs, sinfo, _ = steady_guess(ac_s, prof, u10, chi_deg, mg, n)
                sinfo["re"] = re_s
                diag[lab] = sinfo
                starts.append(("steady", xs, re_s))
            starts += [] if x0 is not None else [("seeds", s_, re_in) for s_ in seed_sets]
            for kind, st, re_k in starts:
                ac = build_aircraft(d, re_k)
                kw2 = dict(x0=st) if kind != "seeds" else dict(seeds=st, x0=None)
                info, x, col = solve(ac, prof, mode="min_power", cycle="travel", n=n, u10=u10,
                                     chi=np.radians(chi_deg), tip_model="geometric", midpoints=True,
                                     rate_limits=True, rate_model="pb2v", n_min=0.0, motor=True,
                                     v_travel_min=0.5 * ac.v_best_ld,
                                     maxiter=maxiter, return_x=True, stop_after=1, **kw2, **mg)
                if info["success"]:
                    diag.setdefault("start", kind)
                    return info, np.asarray(x), col, lab, re_k
        return None

    re, x0, label, last = re_start, None, None, None
    for _ in range(2):
        r = cycle(re, label, x0)
        if r is None and x0 is not None:
            r = cycle(re, label, None)
        if r is None:
            break
        info, x, col, label, re = r
        v = mean_airspeed(col, x)
        re_new = re_of(v, chord)
        last = (info, x, col, label, re, v)
        x0 = x
        if abs(_scale(re_new, nexp) / _scale(re, nexp) - 1.0) <= SCALE_TOL:
            break
        re = re_new
    if last is None:
        return {"success": False, "u10": u10, "chi_deg": chi_deg, "re_used": re, "steady": diag}
    info, x, col, label, re, v = last
    ms = motor_stats(col, x)
    prog = float(info["speed_along_chi"])
    return {"success": True, "u10": u10, "power_w": ms["power_avg_w"], "power_peak_w": ms["power_peak_w"],
            "thrust_on_share": ms["thrust_on_share"], "progress": prog,
            "wh_per_km": float(ms["power_avg_w"] / (3.6 * prog)) if prog > 0.1 else None,
            "chi_deg": chi_deg, "margin_set": label, "steady": diag, "re_used": re, "v_mean": v, "period": float(info["period"])}


def evaluate(job):
    """Worker: job = (kind, Design, extra). kind "A", "B" (extra = (re_start, u10)) or "C" (extra =
    (re_start, u10))."""
    kind, d, extra = job
    t0 = time.time()
    ok, why = feasibility(d)
    if not ok:
        return {"kind": kind, "key": d.key, "design": d.__dict__, "success": False, "reason": why}
    try:
        r = solve_a(d) if kind == "A" else (solve_b(d, *extra) if kind == "B" else solve_c(d, extra[1], extra[0]))
    except Exception as exc:   # one bad solve must not stop the study
        r = {"success": False, "reason": f"exception {type(exc).__name__}: {exc}"}
    r.update({"kind": kind, "key": d.key, "extra": extra if kind != "A" else None, "design": d.__dict__,
              "runtime_s": time.time() - t0})
    return r


def evaluate_point(d: Design):
    """Worker: all objectives of one design and ballast. A first; B and C start from A's Reynolds
    number (or 1.3 Vs when A fails, since B and C are still informative)."""
    t0 = time.time()
    ok, why = feasibility(d)
    base = {"key": d.key, "design": d.__dict__}
    if not ok:
        return dict(base, success=False, reason=why)
    mm, _, chord, vs = basics(d)
    out = dict(base, mass=mm["mass"], wing_loading=mm["mass"] * G / mm["area"], area=mm["area"], success=True)
    out.update(glide_point(d))
    bd = body_drag(d.span, mm["area"], out["v_glide"])
    out["body_drag"] = bd
    try:
        a = solve_a(d)
        out["A"] = a
        re0 = a["re_used"] if a["success"] else re_of(1.3 * vs, chord)
        out["B"] = solve_b(d, re0)
        c = {}
        for u in U10_C:
            c[f"{u:g}"] = solve_c(d, u, re0)
        out["C"] = c
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    out["runtime_s"] = time.time() - t0
    print(f"done {d.key} A={out.get('A', {}).get('u10')} {out['runtime_s']:.0f} s", file=sys.stderr, flush=True)
    return out


def parse_sets(text):
    """"3.0:12,16;2.5:16" -> ((3.0, (12.0, 16.0)), (2.5, (16.0,)))"""
    out = []
    for part in text.split(";"):
        b, ars = part.split(":")
        out.append((float(b), tuple(float(a) for a in ars.split(","))))
    return tuple(out)


def grid(sets, tapers=(0.45,), airfoils=("SD7037", "AG35-r"), ballasts=None):
    """Design points with the ballast grid (per design, the grid entries below the mass limit and
    the limit). sets: ((span, (ar, ...)), ...)."""
    out = []
    for b, ars in sets:
        for ar in ars:
            for t in tapers:
                for af in airfoils:
                    for bal in ballast_grid(b, ar, grid=ballasts):
                        out.append(Design(b, ar, t, af, ballast=round(bal, 4)))
    return out


def run(workers, sets, out, resume=True, limit=None, ballasts=None):
    from concurrent.futures import ProcessPoolExecutor, as_completed
    t0 = time.time()
    designs = grid(sets, ballasts=ballasts)
    if limit:
        designs = designs[:limit]
    part = out + ".part"
    done = {}
    if resume and os.path.exists(part):
        with open(part) as fh:
            for line in fh:
                r = json.loads(line)
                done[r["key"]] = r
    todo = [d for d in designs if d.key not in done]
    todo.sort(key=lambda d: -d.span)   # long spans first (the slowest)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with ProcessPoolExecutor(workers) as ex, open(part, "a", newline="\n") as fh:
        futs = [ex.submit(evaluate_point, d) for d in todo]
        for f in as_completed(futs):
            r = f.result()
            done[r["key"]] = r
            fh.write(json.dumps(r) + "\n")
            fh.flush()
    results = [done[d.key] for d in designs]
    from sim import record
    here = os.path.dirname(__file__)
    meta = record.meta("python -m sim.wingopt", [__file__, os.path.join(here, "dsopt.py"),
                                                 os.path.join(here, "aircraft.py")])
    meta.update({"runtime_s": time.time() - t0, "sets": [[b, list(a)] for b, a in sets],
                 "span_max_m": value("ds.span_max_m", None), "mass_max_kg": value("ds.mass_max_kg", None),
                 "n_limit": value("ds.n_limit", 4.0), "safety_factor": value("ds.safety_factor", 1.5),
                 "component_keys": list(COMPONENT_KEYS), "m_other_kg": other_mass(),
                 "margins": MARGINS, "margins_light": MARGINS_LIGHT, "n_nodes": N_NODES,
                 "u10_B": U10_B, "courses_B": list(COURSES_B), "u10_C": list(U10_C), "chi_C_deg": CHI_C,
                 "seeds_A": list(SEEDS_A), "motor_seeds": [list(x) for x in MOTOR_SEEDS], "motor_maxiter": MOTOR_MAXITER, "ballast_grid_kg": list(BALLAST_GRID if ballasts is None else ballasts),
                 "frame": "chi ccw from the wind direction: 0 downwind, 180 straight upwind",
                 "body": {"pod_l": POD_L, "pod_d": POD_D, "boom_d": BOOM_D, "tail_arm_per_span": TAIL_ARM_PER_SPAN,
                          "vh": VH, "vv": VV, "ar_h": AR_H, "ar_v": AR_V, "gaps": GAPS_FACTOR},
                 "taper_offset": NS_DLAMBDA_UNSWEPT,
                 "dsopt": "min_wind travel, self_consistent COARE 3.5, geometric tip 0.3, midpoints, pb2v, "
                          "n_min 0, replay_check, seeds 0-5 all run; B max_speed seeds 0-5 stop_after 1; "
                          "C min_power motor, library margins and seeds"})
    payload = {"meta": meta, "points": results}
    with open(out, "w", newline="\n") as fh:
        json.dump(payload, fh, indent=1)
        fh.write("\n")
    return payload


MOTOR_SEEDS_FULL = ((0, 1, 2), (3, 4, 5))   # = sim.mpc.MOTOR_SEEDS
MOTOR_MAXITER_FULL = 400                    # = sim.mpc.MOTOR_MAXITER
FOLLOW_U10 = (6.0, 9.0, 13.0)
FOLLOW_CHI = (20.0, 180.0)


def _step(p, command):
    """Stamp a post-run step in the result's meta block (the main stamp predates these steps)."""
    from sim import record
    here = os.path.dirname(__file__)
    p["meta"].setdefault("steps", []).append(
        record.meta(command, [__file__, os.path.join(here, "dsopt.py"), os.path.join(here, "aircraft.py")]))


def stamp_step(path, command):
    """python -c "from sim import wingopt; wingopt.stamp_step(path, command)": stamp a step whose
    result was written by an earlier version of this file (the stamp is of the file as it is now)."""
    with open(path) as fh:
        p = json.load(fh)
    _step(p, command)
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")


def evaluate_follow(job):
    """Worker: for one top design, h(180) at u10_B and the motor minimum power at 6, 9 and 13 m/s at
    chi 20 and 180 with the full library settings. Failures are kept as failures."""
    d, re0 = job
    t0 = time.time()
    out = {"key": d.key, "design": d.__dict__, "re0": re0}
    try:
        out["B180"] = solve_b(d, re0, courses=(180.0,))
        out["C"] = {f"{u:g}@{chi:g}": solve_c(d, u, re0, chi_deg=chi, seed_sets=MOTOR_SEEDS_FULL, maxiter=MOTOR_MAXITER_FULL)
                    for chi in FOLLOW_CHI for u in FOLLOW_U10}
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    out["runtime_s"] = time.time() - t0
    print(f"follow {d.key} {out['runtime_s']:.0f} s", file=sys.stderr, flush=True)
    return out


def followup(path, top=4, workers=4):
    """Add key "followup" to the result file: the top designs by the soaring minimum wind."""
    from concurrent.futures import ProcessPoolExecutor
    with open(path) as fh:
        p = json.load(fh)
    pts = [r for r in p["points"] if r.get("success") and r.get("A", {}).get("success")]
    pts.sort(key=lambda r: r["A"]["u10"])
    jobs = [(Design(**r["design"]), r["A"]["re_used"]) for r in pts[:top]]
    t0 = time.time()
    with ProcessPoolExecutor(workers) as ex:
        p["followup"] = list(ex.map(evaluate_follow, jobs, chunksize=1))
    _step(p, "python -m sim.wingopt --followup")
    p["meta"]["followup"] = {"top_by_A": top, "u10": list(FOLLOW_U10), "chi_deg": list(FOLLOW_CHI),
                             "motor_seeds": [list(x) for x in MOTOR_SEEDS_FULL],
                             "motor_maxiter": MOTOR_MAXITER_FULL, "runtime_s": time.time() - t0}
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")
    return p


def evaluate_c9(job):
    """Worker: the motor power at 9 m/s, chi 20, with the full library settings (job's third entry:
    the margin sets allowed, None = standard then light)."""
    d, re0 = job[:2]
    labels = job[2] if len(job) > 2 else None
    t0 = time.time()
    try:
        c = solve_c(d, 9.0, re0, chi_deg=20.0, seed_sets=MOTOR_SEEDS_FULL, maxiter=MOTOR_MAXITER_FULL,
                    margin_labels=labels)
    except Exception as exc:
        c = {"success": False, "reason": f"{type(exc).__name__}: {exc}"}
    return {"key": d.key, "C9_full": c, "runtime_s": time.time() - t0}


def c9_all(path, workers=4):
    """Key "c9_full" of the result file: the full-settings motor power at 9 m/s, chi 20, for every
    design whose soaring minimum solved (the main run's cut settings failed on most)."""
    from concurrent.futures import ProcessPoolExecutor
    with open(path) as fh:
        p = json.load(fh)
    pts = [r for r in p["points"] if r.get("A", {}).get("success")]
    jobs = [(Design(**r["design"]), r["A"]["re_used"]) for r in pts]
    with ProcessPoolExecutor(workers) as ex:
        p["c9_full"] = list(ex.map(evaluate_c9, jobs, chunksize=1))
    _step(p, "python -m sim.wingopt --c9-all")
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")
    return p


def c9_standard(path, workers=4):
    """Key "c9_standard": the 9 m/s, chi 20 motor power at the standard margins only (full
    settings) for the designs whose c9_full solve used the light margins; failures are kept."""
    from concurrent.futures import ProcessPoolExecutor
    with open(path) as fh:
        p = json.load(fh)
    light = {r["key"] for r in p["c9_full"] if r["C9_full"].get("margin_set") == "light"}
    jobs = [(Design(**r["design"]), r["A"]["re_used"], ("standard",)) for r in p["points"]
            if r["key"] in light]
    with ProcessPoolExecutor(workers) as ex:
        p["c9_standard"] = list(ex.map(evaluate_c9, jobs, chunksize=1))
    _step(p, "python -m sim.wingopt --c9-standard")
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")
    return p


def evaluate_c9s(job):
    """Worker: 9 m/s motor power at the standard margins, full settings, started from the steady
    level-flight guess (then the seeds), at chi 20 and 180."""
    d, re0 = job
    t0 = time.time()
    out = {"key": d.key}
    for chi in (20.0, 180.0):
        try:
            out[f"chi{chi:g}"] = solve_c(d, 9.0, re0, chi_deg=chi, seed_sets=MOTOR_SEEDS_FULL,
                                         maxiter=MOTOR_MAXITER_FULL, margin_labels=("standard",))
        except Exception as exc:
            out[f"chi{chi:g}"] = {"success": False, "reason": f"{type(exc).__name__}: {exc}"}
    out["runtime_s"] = time.time() - t0
    return out


def c9_steady(path, workers=4):
    """Key "c9_steady": every design with a soaring minimum, 9 m/s at chi 20 and 180."""
    from concurrent.futures import ProcessPoolExecutor
    with open(path) as fh:
        p = json.load(fh)
    jobs = [(Design(**r["design"]), r["A"]["re_used"]) for r in p["points"] if r.get("A", {}).get("success")]
    with ProcessPoolExecutor(workers) as ex:
        p["c9_steady"] = list(ex.map(evaluate_c9s, jobs, chunksize=1))
    _step(p, "python -m sim.wingopt --c9-steady")
    with open(path, "w", newline="\n") as fh:
        json.dump(p, fh, indent=1)
        fh.write("\n")
    return p


def c9_best(entry, chi="chi20"):
    """The optimizer's 9 m/s motor cycle at the standard margins for one c9_steady entry: power,
    energy per kilometre, progress and which start solved it (steady, seeds or warm); None if it
    failed."""
    c = entry.get(chi, {})
    if not c.get("success"):
        return None
    prog = c["progress"]
    return {"source": c.get("steady", {}).get("start", "?"), "power_w": float(c["power_w"]),
            "progress": float(prog), "wh_per_km": float(c["power_w"] / (3.6 * prog)) if prog > 0.1 else None}


def _row(r, c9=None):
    a, b = r.get("A", {}), r.get("B", {})
    d = r["design"]
    return {"d": d, "mass": r["mass"], "ws": r["wing_loading"], "ld": r["ld_glide"],
            "u10": a["u10"] if a.get("success") else None,
            "h180": b.get("h180") if b.get("success") else None,
            "p9": c9["power_w"] if c9 else None, "e9": c9["wh_per_km"] if c9 else None,
            "cda": r["body_drag"]["total"], "src": c9["source"] if c9 else None}


def rows_of(p):
    """Design rows with the best standard-margin 9 m/s, chi 20 motor cycle (c9_steady)."""
    c9 = {e["key"]: c9_best(e) for e in p.get("c9_steady", [])}
    return [_row(r, c9.get(r["key"])) for r in p["points"] if r.get("success") and r.get("A", {}).get("success")]


def pareto(p, keys=("u10", "p9")):
    """Non-dominated rows (smaller is better in both keys) of the rows with both defined."""
    rows = [r for r in rows_of(p) if all(r[k] is not None for k in keys)]
    front = [r for r in rows if not any(all(o[k] <= r[k] for k in keys) and any(o[k] < r[k] for k in keys)
                                        for o in rows)]
    return sorted(front, key=lambda r: r[keys[0]])


def _fmt(x, f):
    return "-" if x is None else f.format(x)


def tables(p):
    rows = sorted(rows_of(p), key=lambda r: r["u10"])
    L = ["| span | AR | airfoil | ballast | mass kg | W/S | L/D glide | cda cm2 | u10 min | P9 W | Wh/km | source |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        d = r["d"]
        L.append(f"| {d['span']:g} | {d['ar']:g} | {d['airfoil']} | {d['ballast']:.2f} | {r['mass']:.2f} | {r['ws']:.1f} | "
                 f"{r['ld']:.1f} | {r['cda'] * 1e4:.1f} | {_fmt(r['u10'], '{:.2f}')} | {_fmt(r['p9'], '{:.1f}')} | "
                 f"{_fmt(r['e9'], '{:.2f}')} | {r['src'] or '-'} |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--sets", default="3.0:12,16,20,25", help="span:AR,AR;span:AR (the cut grid)")
    ap.add_argument("--ballast", default="", help="comma list of ballast grid kg (default 0,0.5,1.0; the mass limit is always added)")
    ap.add_argument("--out", default=OUT_PATH)
    ap.add_argument("--limit", type=int, default=0, help="only the first N design points (timing)")
    ap.add_argument("--fresh", action="store_true", help="ignore the .part file")
    ap.add_argument("--c9-steady", action="store_true", help="9 m/s motor power from the steady-flight start, chi 20 and 180, standard margins, all designs; exits")
    ap.add_argument("--c9-standard", action="store_true", help="re-solve the light-margin 9 m/s rows at the standard margins and exit")
    ap.add_argument("--c9-all", action="store_true", help="add the full-settings 9 m/s motor power for all designs to --out and exit")
    ap.add_argument("--followup", type=int, default=0, help="add the follow-up for the top N designs to --out and exit")
    a = ap.parse_args(argv)
    if a.c9_steady:
        c9_steady(a.out, a.workers)
        return
    if a.c9_standard:
        c9_standard(a.out, a.workers)
        return
    if a.c9_all:
        c9_all(a.out, a.workers)
        return
    if a.followup:
        followup(a.out, a.followup, a.workers)
        return
    p = run(a.workers, parse_sets(a.sets), a.out, resume=not a.fresh, limit=a.limit or None,
            ballasts=tuple(float(b) for b in a.ballast.split(",")) if a.ballast else None)
    print(tables(p))
    print(f"runtime {p['meta']['runtime_s']:.0f} s")


if __name__ == "__main__":
    main()
