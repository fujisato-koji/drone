"""Dynamic soaring cycle search with a parameterised periodic controller."""
import time

import numpy as np
from scipy.optimize import differential_evolution

from sim.aircraft import BASELINE, G, export_traj, simulate

try:
    from sim.seawind import Environment, Wind
except ImportError:
    from sim._seawind_stub import Environment, Wind

H_MIN_REQ = 1.0
H0 = 3.0


def make_ctrl(p, psi_wind):
    T, mu0, d_mu, cl0, cl1, d_cl, _ = p

    def ctrl(t, s, env, ac):
        phi = 2 * np.pi * (t % T) / T
        return cl0 + cl1 * np.sin(phi + d_cl), mu0 * np.sin(phi + d_mu)

    return ctrl


def evaluate(ac, env, p, h0, V0, n_cycles=2, dt=0.02):
    """Simulate n_cycles of the periodic controller; gain is per last cycle, in J/kg."""
    T = p[0]
    psi_wind = np.arctan2(env.wind.ey, env.wind.ex)
    s0 = [0.0, 0.0, h0 + float(env.surface(0.0, 0.0, 0.0)[0]), V0, 0.0, psi_wind + p[6]]
    r = simulate(ac, env, make_ctrl(p, psi_wind), s0, 0.0, n_cycles * T, dt=dt)
    t = r["t"]
    i1 = len(t) - 1
    i0 = min(int(np.searchsorted(t, (n_cycles - 1) * T - 1e-9)), i1)
    gv = np.array([np.diff(r[k][i0:i1 + 1][[0, -1]])[0] for k in ("x", "y", "z")])
    span = max(t[i1] - t[i0], 1e-9)
    return {
        "gain": G * (r["e_air"][i1] - r["e_air"][i0]),
        "min_h": float(r["h"].min()),
        "max_n": float(r["n"].max()),
        "min_V": float(r["V"].min()),
        "mean_ground_velocity": gv / span,
        "dV": float(r["V"][i1] - r["V"][i0]),
        "dz": float(r["z"][i1] - r["z"][i0]),
        "completed": float(t[i1] / (n_cycles * T)),
        "result": r,
    }


def penalties(ac, e):
    return (50 * max(0.0, H_MIN_REQ - e["min_h"]) ** 2
            + 50 * max(0.0, e["max_n"] - ac.n_max) ** 2
            + 50 * max(0.0, 1.1 * ac.v_stall - e["min_V"]) ** 2
            + 5 * G * (abs(e["dV"]) + abs(e["dz"]))  # periodicity, scaled by g to J/kg
            + 200 * (1.0 - e["completed"]))  # ended early (hit the sea)


def objective(x, ac, env, n_cycles=2, dt=0.05):
    """Negative of (gain - penalties) for the minimiser. x = p (7 values) + (V0,)."""
    e = evaluate(ac, env, x[:7], H0, x[7], n_cycles, dt)
    return -(e["gain"] - penalties(ac, e))


def find_cycle(ac, u10, from_deg=270, maxiter=40, popsize=12, dt=0.05, **kw):
    env = Environment.flat(Wind(u10, from_deg))
    vs = ac.v_stall
    bounds = [(3, 15), (0.2, ac.mu_max), (-np.pi, np.pi), (0.3, ac.cl_max), (0, 0.6),
              (-np.pi, np.pi), (-np.pi, np.pi), (1.2 * vs, 3 * vs)]
    res = differential_evolution(objective, bounds, args=(ac, env, 2, dt), maxiter=maxiter,
                                 popsize=popsize, seed=1, workers=1, polish=False,
                                 updating="immediate", **kw)
    p = tuple(res.x[:7])
    e = evaluate(ac, env, p, H0, res.x[7], 2, 0.02)  # re-evaluate at the default dt
    return {"p": p, "V0": float(res.x[7]), "gain": e["gain"], "min_h": e["min_h"],
            "max_n": e["max_n"], "min_V": e["min_V"],
            "mean_ground_velocity": e["mean_ground_velocity"],
            "feasible": bool(e["min_h"] >= H_MIN_REQ - 0.1 and e["max_n"] <= ac.n_max + 0.1
                             and e["min_V"] >= 1.1 * vs - 0.1 and e["completed"] >= 1.0),
            "result": e["result"], "env": env}


def min_u10_sustained(ac, lo=4, hi=20, tol=0.5, **kw):
    """Bisection on u10: smallest wind where a feasible cycle with gain >= 0 exists."""
    ok = lambda u: (lambda c: c["feasible"] and c["gain"] >= 0)(find_cycle(ac, u, **kw))
    if not ok(hi):
        return None
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if ok(mid):
            hi = mid
        else:
            lo = mid
    return hi


def _summary(c):
    p = c["p"]
    print("gain %.2f J/kg  T %.2f s  mu0 %.2f  min_h %.2f m  max_n %.2f  min_V %.2f  "
          "mean vg (%.2f, %.2f, %.2f)  feasible %s" %
          ((c["gain"], p[0], p[1], c["min_h"], c["max_n"], c["min_V"])
           + tuple(c["mean_ground_velocity"]) + (c["feasible"],)))


if __name__ == "__main__":
    t0 = time.time()
    c = find_cycle(BASELINE, 10)
    print("find_cycle u10=10: %.0f s" % (time.time() - t0))
    print("p =", [round(v, 3) for v in c["p"]], "V0 = %.2f" % c["V0"])
    _summary(c)
    export_traj(c["result"], "sim/viewer/traj.js")
    t0 = time.time()
    print("min_u10_sustained:", min_u10_sustained(BASELINE), "(%.0f s)" % (time.time() - t0))
