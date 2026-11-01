"""Dynamic soaring cycle search: sine (time-phase) and Rayleigh (heading-triggered) controllers."""
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from sim import record
from sim.aircraft import BASELINE, G, simulate
from sim.aircraft import export_traj as _write_traj
from sim.seawind import Environment, Wind, WaveTrain

H_MIN_REQ = record.value("ds.clearance_min_m", 1.0)
H0 = 3.0
N_CYCLES = 3
K_GAMMA = 2.0  # per radian, flight path angle hold in phases A and C
T_CAP = 60.0  # Rayleigh runs stop at this time if three cycles are not completed


def export_traj(result, path, name="TRAJ", command="python -m sim.dsearch"):
    """Write the trajectory JS file (sim.aircraft writer) with a tdr-run header as its first line."""
    _write_traj(result, path, name)
    here = Path(__file__).resolve().parent
    files = [here / "aircraft.py", here / "dsearch.py", here / "seawind.py"]
    with open(path, encoding="utf-8", newline="") as fh:
        body = fh.read()
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header(command, files) + "\n" + body)


def _psi_wind(env):
    return float(np.arctan2(env.wind.ey, env.wind.ex))


def _start_z(env, h0):
    return h0 + float(env.surface(0.0, 0.0, 0.0)[0])


# ---------------------------------------------------------------- sine controller

def make_ctrl(p, psi_wind=0.0):
    T, mu0, d_mu, cl0, cl1, d_cl, _ = p

    def ctrl(t, s, env, ac):
        phi = 2 * np.pi * (t % T) / T
        return cl0 + cl1 * np.sin(phi + d_cl), mu0 * np.sin(phi + d_mu)

    return ctrl


def _window(r, i0, i1):
    span = max(r["t"][i1] - r["t"][i0], 1e-9)
    return np.array([r[k][i1] - r[k][i0] for k in ("x", "y", "z")]) / span


def _stats(ac, r):
    return {"min_h": float(r["h"].min()), "max_n": float(r["n"].max()),
            "min_V": float(r["V"].min())}


def evaluate(ac, env, p, h0, V0, n_cycles=N_CYCLES, dt=0.02):
    """Sine controller: simulate n_cycles*T; gain (J/kg) is over the last cycle."""
    T = p[0]
    s0 = [0.0, 0.0, _start_z(env, h0), V0, 0.0, _psi_wind(env) + p[6]]
    r = simulate(ac, env, make_ctrl(p), s0, 0.0, n_cycles * T, dt=dt)
    t = r["t"]
    i1 = len(t) - 1
    i0 = min(int(np.searchsorted(t, (n_cycles - 1) * T - 1e-9)), i1)
    e = {"gain": G * (r["e_air"][i1] - r["e_air"][i0]),
         "mean_ground_velocity": _window(r, i0, i1),
         "completed": min(1.0, i1 / max(int(round(n_cycles * T / dt)), 1)), "result": r}
    e.update(_stats(ac, r))
    return e


def penalties(ac, e):
    return (50 * max(0.0, H_MIN_REQ - e["min_h"]) ** 2
            + 50 * max(0.0, e["max_n"] - ac.n_max) ** 2
            + 50 * max(0.0, 1.1 * ac.v_stall - e["min_V"]) ** 2
            + 200 * max(0.0, 1.0 - e["completed"]))  # ended early (hit the sea / cycles not finished)


def objective(x, ac, env, n_cycles=N_CYCLES, dt=0.05):
    """Sine search: minimiser objective for x = p (7 values) + (V0,)."""
    e = evaluate(ac, env, x[:7], H0, x[7], n_cycles, dt)
    return -(e["gain"] - penalties(ac, e))


# ---------------------------------------------------------------- Rayleigh controller

class RayleighCycleController:
    """Heading-triggered four-phase cycle (A upwind climb, B turn down, C descent, D turn up).

    p = (mu_turn, gamma_climb, cl_turn, gamma_desc, h_top, h_bot, turn_sign); turn_sign is
    thresholded at 0. In A and C the controller holds a flight path angle of +gamma_climb /
    -gamma_desc with cl = cl_level + K_GAMMA*(gamma_target - gamma). Phases advance in commit(), called once per accepted step.
    """

    def __init__(self, p, psi_wind):
        (self.mu_turn, self.gamma_climb, self.cl_turn, self.gamma_desc,
         self.h_top, self.h_bot, ts) = p
        self.turn_sign = 1.0 if ts >= 0 else -1.0
        self.psi_up = psi_wind + np.pi
        self.psi_down = psi_wind
        self.phase = "A"
        self.changes = []  # (t, new phase)
        self.cycle_starts = []  # times of entering A (first entry at t0)

    @staticmethod
    def _within(psi, target, deg=15.0):
        return np.cos(psi - target) >= np.cos(np.radians(deg))

    def commit(self, t, s, env, ac):
        if not self.cycle_starts:
            self.cycle_starts.append(t)
        h = s[2] - float(env.surface(s[0], s[1], t)[0])
        V, psi = s[3], s[5]
        new = self.phase
        if self.phase == "A":
            if h >= self.h_top or V <= 1.15 * ac.v_stall:
                new = "B"
        elif self.phase == "B":
            if self._within(psi, self.psi_down):
                new = "C"
        elif self.phase == "C":
            if h <= self.h_bot:
                new = "D"
        elif self.phase == "D":
            if self._within(psi, self.psi_up):
                new = "A"
        if new != self.phase:
            self.phase = new
            self.changes.append((t, new))
            if new == "A":
                self.cycle_starts.append(t)

    def __call__(self, t, s, env, ac):
        ph = self.phase
        if ph in ("A", "C"):
            g, V = s[4], s[3]
            target = self.gamma_climb if ph == "A" else -self.gamma_desc
            cl_level = 2 * ac.mass * G * np.cos(g) / (ac.rho * V * V * ac.wing_area)
            cl = cl_level + K_GAMMA * (target - g)
            return min(max(cl, ac.cl_min), ac.cl_max), 0.0
        if ph == "B":
            return self.cl_turn, self.turn_sign * self.mu_turn
        return self.cl_turn, -self.turn_sign * self.mu_turn

    @property
    def cycles(self):
        return len(self.cycle_starts) - 1


def evaluate_controller(ac, env, ctrl, h0, V0, t_end, dt=0.02, n_cycles=None, psi0=None):
    """Run a stateful cycle controller (with .cycle_starts / .cycles) and return the
    per-cycle gains (J/kg) in out["gains"], with cycle periods, stats, the mean ground
    velocity of the last complete cycle, and the simulate() result. Stops after n_cycles
    cycles if given. Default start heading is upwind."""
    if psi0 is None:
        psi0 = _psi_wind(env) + np.pi
    s0 = [0.0, 0.0, _start_z(env, h0), V0, 0.0, psi0]

    def stop(t, s):
        if s[2] - float(env.surface(s[0], s[1], t)[0]) < 0.0:
            return True
        return n_cycles is not None and ctrl.cycles >= n_cycles

    r = simulate(ac, env, ctrl, s0, 0.0, t_end, dt=dt, stop=stop)
    idx = [min(int(round(tc / dt)), len(r["t"]) - 1) for tc in ctrl.cycle_starts]
    pairs = list(zip(idx[:-1], idx[1:]))
    out = {"gains": [G * (r["e_air"][b] - r["e_air"][a]) for a, b in pairs],
           "T_cycles": [float(r["t"][b] - r["t"][a]) for a, b in pairs],
           "mean_ground_velocity": _window(r, *pairs[-1]) if pairs else np.zeros(3),
           "result": r}
    out.update(_stats(ac, r))
    return out


def evaluate_rayleigh(ac, env, x, dt=0.05):
    p, V0 = x[:7], x[7]
    ctrl = RayleighCycleController(p, _psi_wind(env))
    e = evaluate_controller(ac, env, ctrl, p[5] + 1.0, V0, T_CAP, dt, n_cycles=N_CYCLES)
    e["ctrl"] = ctrl
    e["gain"] = e["gains"][-1] if len(e["gains"]) >= N_CYCLES else 0.0
    e["completed"] = len(e["gains"]) / N_CYCLES
    return e


def objective_rayleigh(x, ac, env, dt=0.05):
    e = evaluate_rayleigh(ac, env, x, dt)
    return -(e["gain"] - penalties(ac, e))


# ---------------------------------------------------------------- search

def find_cycle(ac, u10, from_deg=270, controller="rayleigh", maxiter=40, popsize=12,
               seed=1, dt=0.05, **kw):
    env = Environment.flat(Wind(u10, from_deg))
    vs = ac.v_stall
    if controller == "sine":
        bounds = [(3, 15), (0.2, ac.mu_max), (-np.pi, np.pi), (0.3, ac.cl_max), (0, 0.6),
                  (-np.pi, np.pi), (-np.pi, np.pi), (1.2 * vs, 3 * vs)]
        fun, args = objective, (ac, env, N_CYCLES, dt)
    else:
        bounds = [(0.5, ac.mu_max), (0.1, 0.7), (0.5, ac.cl_max), (0.1, 0.7),
                  (4, 25), (1.0, 4), (-1, 1), (1.3 * vs, 2.2 * vs)]
        fun, args = objective_rayleigh, (ac, env, dt)
    res = differential_evolution(fun, bounds, args=args, maxiter=maxiter, popsize=popsize,
                                 seed=seed, workers=1, polish=False, updating="immediate", **kw)
    x = res.x
    if controller == "sine":
        e = evaluate(ac, env, x[:7], H0, x[7], N_CYCLES, 0.02)
        gains, T_last = [e["gain"]], float(x[0])
    else:
        e = evaluate_rayleigh(ac, env, x, 0.02)
        gains = e["gains"]
        T_last = e["T_cycles"][-1] if e["T_cycles"] else float("nan")
    return {"controller": controller, "p": tuple(float(v) for v in x[:7]), "V0": float(x[7]),
            "gain": e["gain"], "gains": gains, "T_last": T_last, "min_h": e["min_h"],
            "max_n": e["max_n"], "min_V": e["min_V"],
            "mean_ground_velocity": e["mean_ground_velocity"],
            "feasible": bool(e["min_h"] >= H_MIN_REQ - 0.1 and e["max_n"] <= ac.n_max + 0.1
                             and e["min_V"] >= 1.1 * vs - 0.1 and e["completed"] >= 1.0),
            "result": e["result"], "env": env}


def full_env_trajectory(ac, c, u10, from_deg=270, dt=0.02):
    """Re-fly a Rayleigh find_cycle result in the waved environment."""
    wind = Wind(u10, from_deg)
    env = Environment(wind, [WaveTrain.fully_developed(wind)])
    ctrl = RayleighCycleController(c["p"], _psi_wind(env))
    return evaluate_controller(ac, env, ctrl, c["p"][5] + 1.0, c["V0"], T_CAP, dt,
                               n_cycles=N_CYCLES)


def min_u10_sustained(ac, lo=4, hi=20, tol=0.5, **kw):
    """Bisection on u10: smallest wind where a feasible cycle with gain >= 0 exists."""
    def ok(u):
        c = find_cycle(ac, u, **kw)
        return c["feasible"] and c["gain"] >= 0
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
    print("gains", [round(g, 2) for g in c["gains"]], "J/kg  T_last %.2f s  min_h %.2f  "
          "max_n %.2f  min_V %.2f  mean vg (%.2f, %.2f, %.2f)  feasible %s" %
          ((c["T_last"], c["min_h"], c["max_n"], c["min_V"])
           + tuple(c["mean_ground_velocity"]) + (c["feasible"],)))
    print("p =", [round(v, 3) for v in c["p"]], "V0 = %.2f" % c["V0"])


if __name__ == "__main__":
    t0 = time.time()
    c = find_cycle(BASELINE, 10, maxiter=25, popsize=10)
    print("rayleigh u10=10: %.0f s" % (time.time() - t0), flush=True)
    _summary(c)
    e = full_env_trajectory(BASELINE, c, 10)
    print("waved env gains", [round(g, 2) for g in e["gains"]], "min_h %.2f" % e["min_h"])
    export_traj(e["result"], "sim/out/dsearch_traj.js")   # no longer read by the viewer
    for u in (6, 8, 12):
        t0 = time.time()
        cu = find_cycle(BASELINE, u, maxiter=15, popsize=8)
        print("u10=%d (%.0f s)" % (u, time.time() - t0), flush=True)
        _summary(cu)
