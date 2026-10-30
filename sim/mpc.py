"""Receding-horizon flight decisions for the dynamic soaring glider (MPPI on a point mass).

Choices the spec left open (simplest thing taken):
- Controls: n_ctrl knots spread evenly over [0, horizon_s] (knot 0 at "now"), piecewise linear.
  Perturbations are independent Gaussian per knot, clipped to the aircraft limits.
- MPPI weights exp(-(S - S_min) / (lam * scale)), scale = std of the costs of the rollouts that
  did not end in contact or stall (std of all if fewer than 2 did). So lam = 1 is in units of the
  spread of the costs, not of J/kg. Sample 0 is the unperturbed (shifted) plan.
- Contact for the planner is the clearance measure below zero (centre height by default, as for the true stop; "wingtip" optional);
  the closed loop stops on the TRUE centre height above the true surface below 0.
- Stall departure in a rollout (V < 0.5 v_stall or non-finite) ends the rollout like a contact.
- Cost of a contact: 1e4 + 1e4 (remaining time / horizon), on top of the cost accumulated to then.
- The terminal energy term is the specific air-relative energy g z + V^2/2 (J/kg).
- The "flat" world model puts the surface at z = 0 (mean sea level) and uses the GNSS height.
- Start: 10 m above the true surface at (0, 0), heading toward the target bearing, V = 2 v_stall.
- target_bearing_deg is a COMPASS bearing (0 = N, clockwise), like sim.seawind.
- Sensor noise on z (0.1 m, white) is added in fly_mpc before the estimator sees the height.
- Until the estimator has a fit (at least 40 samples spanning 4 s) the "estimated" model plans
  in the flat world.

Estimates in the loop (state_source / wind_source = "estimate"; choices the spec left open):
- A SensorSuite (default "standard") and the Estimator run at the integration step on the true
  state; the actual bank (after the actuators) feeds the sensor truth.
- The planner state is x, y from the estimate, z = h_surface_est + (planner world surface), V = est
  airspeed, gamma = asin of the vertical component of est.v_air / |v_air| (the estimator's climb
  angle from the GNSS climb rate over the airspeed), psi = est.heading.
- The wind estimate used by the planner is ready when est.valid, est.wind_valid and a profile fit
  exists (u10_est > 0). It is then held at its last ready value if the fit is lost. The planner
  world is Environment.flat(Wind(u10_est, from_deg_est)) (Charnock z0 of the Wind class, not the
  fitted z0); wind_source "estimate" needs world_model "flat".
- Until the sources used are ready the aircraft flies a glide at best-L/D cl with a +-20 deg
  bank that changes sign every 10 s (mu = +20 deg first).
- The start state, and the u10_est argument, are the same as with the truth (the argument is the
  nominal u10 for the start cycle only when wind_source is "estimate").
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

import numpy as np

from sim.aircraft import (_DT, _H, _OFFS, G, clip_controls, derivatives, ground_effect_phi,
                          n_limited_cl, rk4_step)
from sim.seawind import Environment, Wind

# ---------------------------------------------------------------------------------------------
# vectorized dynamics (a copy of sim.aircraft.derivatives over a batch of states)
# ---------------------------------------------------------------------------------------------


def wind_derivs_vec(world, x, y, z, t):
    """(W (S,3), grad (S,3,3) with grad[s,i,j] = dW_i/dx_j, dW/dt (S,3)); same differences as
    sim.aircraft.wind_derivs, one wind_at call for the batch."""
    z0 = getattr(getattr(world, "wind", None), "z0", 0.0)
    low = z - _H <= z0
    zlo = np.where(low, z, z - _H)
    px = np.stack([x, x + _OFFS[0, 0], x + _OFFS[1, 0], x, x, x, x, x, x])
    py = np.stack([y, y, y, y + _OFFS[2, 1], y + _OFFS[3, 1], y, y, y, y])
    pz = np.stack([z, z, z, z, z, z + _H, zlo, z, z])
    pt = np.full(px.shape, float(t))
    pt[7] += _DT
    pt[8] -= _DT
    w = np.asarray(world.wind_at(px, py, pz, pt), dtype=float)   # (9, S, 3)
    grad = np.empty((x.size, 3, 3))
    for j in range(2):
        grad[:, :, j] = (w[1 + 2 * j] - w[2 + 2 * j]) / (2 * _H)
    grad[:, :, 2] = (w[5] - w[6]) / np.where(low, _H, 2 * _H)[:, None]
    return w[0], grad, (w[7] - w[8]) / (2 * _DT)


def derivatives_vec(t, s, ac, world, cl, mu, n_limit=False, thrust=0.0, ground_effect=False):
    """s: (6, S) states; cl, mu: (S,). Returns (6, S) derivatives. With n_limit cl is first
    reduced to the load-factor limit at each state's airspeed (as sim.aircraft.n_limited_cl).
    thrust (N, scalar or (S,)) along the air-relative velocity; ground_effect multiplies the
    induced drag by sim.aircraft.ground_effect_phi at the height above world.surface."""
    x, y, z, V, g, p = s
    if n_limit:
        cl = np.minimum(cl, ac.n_max * ac.mass * G / (0.5 * ac.rho * V * V * ac.wing_area))
    cg_, sg_, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    eV = np.stack([cg_ * cp, cg_ * sp, sg_], axis=1)
    eg = np.stack([-sg_ * cp, -sg_ * sp, cg_], axis=1)
    ep = np.stack([-sp, cp, np.zeros_like(p)], axis=1)
    W, grad, dwdt = wind_derivs_vec(world, x, y, z, t)
    vg = V[:, None] * eV + W
    aW = dwdt + np.einsum("sij,sj->si", grad, vg)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    if ground_effect:
        eta, ex, ey = world.surface(x, y, np.full(np.shape(x), float(t)))
        D = q * (ac.cd0 + ac.k_induced * ground_effect_phi(z - eta, ac.span, mu, p, ex, ey)
                 * cl * cl)
    else:
        D = q * (ac.cd0 + ac.k_induced * cl * cl)
    m = ac.mass
    dV = -D / m - G * sg_ - np.sum(aW * eV, axis=1)
    if np.any(thrust):
        dV = dV + thrust / m
    dg = (L * np.cos(mu) / m - G * cg_ - np.sum(aW * eg, axis=1)) / V
    cgf = np.maximum(np.abs(cg_), 0.05)
    dp = (L * np.sin(mu) / m - np.sum(aW * ep, axis=1)) / (V * cgf)
    return np.stack([vg[:, 0], vg[:, 1], vg[:, 2], dV, dg, dp])


# ---------------------------------------------------------------------------------------------
# planner
# ---------------------------------------------------------------------------------------------


@dataclass
class Plan:
    """Result of one planning call."""
    knots_t: np.ndarray       # (n_ctrl,) seconds from now
    cl_knots: np.ndarray
    mu_knots: np.ndarray
    t: np.ndarray             # times of the first replan_s of controls (from now), step dt
    cl: np.ndarray
    mu: np.ndarray
    path: np.ndarray          # (N+1, 6) predicted states of the plan, step dt
    times: np.ndarray         # (N+1,) seconds from now
    cost: float
    ess: float                # effective sample size of the last iteration
    n_dead: int               # rollouts of the last iteration that ended in contact or stall

    def controls(self, tau):
        """(cl, mu) at tau seconds from the plan time (piecewise linear, held after the end)."""
        return (float(np.interp(tau, self.knots_t, self.cl_knots)),
                float(np.interp(tau, self.knots_t, self.mu_knots)))


class MPPIPlanner:
    def __init__(self, aircraft, horizon_s=6.0, dt=0.05, n_samples=256, n_ctrl=12, lam=1.0,
                 sigma_cl=0.15, sigma_mu=0.25, replan_s=0.5, seed=1, n_iter=2, n_iter_cold=4,
                 w_e=1.0, w_p=0.5, h_clear=1.0, clearance="height", w_clear=200.0,
                 w_n=50.0, w_stall=300.0, w_smooth=2.0, c_contact=1e4, smooth_modes=0,
                 smooth_gain=3.0, w_d=10.0, n_limit=False):
        self.ac = aircraft
        self.horizon_s, self.dt, self.n_samples, self.n_ctrl = horizon_s, dt, n_samples, n_ctrl
        self.lam, self.sigma_cl, self.sigma_mu, self.replan_s = lam, sigma_cl, sigma_mu, replan_s
        self.n_iter, self.n_iter_cold = n_iter, n_iter_cold
        self.w_e, self.w_p, self.h_clear, self.clearance = w_e, w_p, h_clear, clearance
        self.w_clear, self.w_n, self.w_stall, self.w_smooth = w_clear, w_n, w_stall, w_smooth
        self.c_contact = c_contact
        self.smooth_modes, self.smooth_gain = smooth_modes, smooth_gain
        self.w_d = w_d
        self.act = None             # an sim.aircraft.Actuators: rollouts fly through its model
        self.act_state = None       # (actual cl, actual mu, last cl command, last mu command)
        self.n_limit = n_limit      # rollouts fly the load-limited cl (the cost still sees the commanded n)
        self.rng = np.random.default_rng(seed)
        self.nsteps = int(round(horizon_s / dt))
        self.knots_t = np.linspace(0.0, horizon_s, n_ctrl)
        tau = np.arange(2 * self.nsteps + 1) * dt / 2.0       # half-step control grid
        idx = np.clip(np.searchsorted(self.knots_t, tau, side="right") - 1, 0, n_ctrl - 2)
        self._idx = idx
        self._frac = (tau - self.knots_t[idx]) / (self.knots_t[idx + 1] - self.knots_t[idx])
        self.U = None                                         # (2, n_ctrl) nominal plan
        self.guide = None
        self.eps_hook = None        # test hook: callable(iteration) -> (n_samples, 2, n_ctrl) perturbations
        # smooth perturbation basis: cos(pi m tau / H), m = 1..smooth_modes (zero mean excursions
        # lasting from one horizon down to 2 H / smooth_modes), so a coherent turn can be sampled
        m = np.arange(1, smooth_modes + 1)[:, None]
        self._basis = np.cos(np.pi * m * self.knots_t[None, :] / horizon_s)

    # -- control plan bookkeeping ------------------------------------------------------------
    def _initial_plan(self):
        ac = self.ac
        cl0 = float(np.clip(1.0 * ac.cl_best_ld, ac.cl_min, ac.cl_max))
        return np.stack([np.full(self.n_ctrl, cl0), np.zeros(self.n_ctrl)])

    def _shift(self, U):
        tau = np.minimum(self.knots_t + self.replan_s, self.horizon_s)
        return np.stack([np.interp(tau, self.knots_t, U[0]), np.interp(tau, self.knots_t, U[1])])

    def _candidates(self):
        """Constant-control plans always added to the samples: best L/D and a firm pull-up, each
        wings level and banked either way. They keep a recovery in the set when the shifted plan
        is already on its way into the surface."""
        ac = self.ac
        out = []
        for cl in (ac.cl_best_ld, 0.8 * ac.cl_max):
            for mu in (0.0, -0.6, 0.6):
                out.append(np.stack([np.full(self.n_ctrl, cl), np.full(self.n_ctrl, mu)]))
        return np.array(out)

    def _clip(self, U):
        ac = self.ac
        U[..., 0, :] = np.clip(U[..., 0, :], ac.cl_min, ac.cl_max)
        U[..., 1, :] = np.clip(U[..., 1, :], -ac.mu_max, ac.mu_max)
        return U

    def reset(self):
        self.U = None

    # -- rollout -------------------------------------------------------------------------------
    def clearance_height(self, world, x, y, z, g, mu, t):
        eta = np.asarray(world.surface(x, y, np.full(np.shape(x), float(t)))[0])
        h = z - eta
        if self.clearance == "wingtip":
            h = h - 0.5 * self.ac.span * np.sin(np.abs(mu)) * np.cos(g)
        return h

    def rollout(self, world, t0, s0, Us, target_dir):
        """Cost of each control plan in Us (S, 2, n_ctrl). Returns (costs (S,), paths (N+1,6,S),
        dead (S,) bool)."""
        ac, dt, N = self.ac, self.dt, self.nsteps
        S = Us.shape[0]
        i0, fr = self._idx, self._frac
        cl_all = Us[:, 0, i0] * (1 - fr) + Us[:, 0, i0 + 1] * fr      # (S, 2N+1)
        mu_all = Us[:, 1, i0] * (1 - fr) + Us[:, 1, i0 + 1] * fr
        cl_all = np.clip(cl_all, ac.cl_min, ac.cl_max)
        mu_all = np.clip(mu_all, -ac.mu_max, ac.mu_max)
        act = self.act
        if act is not None:
            # delayed commands on the half-step grid (before the delay: the last command sent)
            m = int(round(act.delay / (dt / 2.0)))
            st = self.act_state
            if st is None:
                st = (cl_all[0, 0], mu_all[0, 0], cl_all[0, 0], mu_all[0, 0])
            if m > 0:
                cl_all = np.concatenate([np.full((S, m), st[2]), cl_all[:, :-m]], axis=1)
                mu_all = np.concatenate([np.full((S, m), st[3]), mu_all[:, :-m]], axis=1)
            x_cl, x_mu = np.full(S, st[0]), np.full(S, st[1])
            hd = dt / 2.0
            a_lag = 1.0 - np.exp(-hd / act.tau) if act.tau > 0 else 1.0

            def _act(x, cmd, lim):
                return x + np.clip(a_lag * (cmd - x), -lim, lim)
        s = np.repeat(np.asarray(s0, float)[:, None], S, axis=1)
        e0 = G * s0[2] + 0.5 * s0[3] ** 2
        cost = np.zeros(S)
        dead = np.zeros(S, bool)
        t_dead = np.full(S, self.horizon_s)
        path = np.empty((N + 1, 6, S))
        path[0] = s
        vmin = 1.1 * ac.v_stall
        for i in range(N):
            t = t0 + i * dt
            c1, c2, c3 = cl_all[:, 2 * i], cl_all[:, 2 * i + 1], cl_all[:, 2 * i + 2]
            m1, m2, m3 = mu_all[:, 2 * i], mu_all[:, 2 * i + 1], mu_all[:, 2 * i + 2]
            c3_cmd = c3
            if act is not None:
                lim_mu = act.mu_rate_at(s[3]) * hd
                lim_cl = act.cl_rate * hd
                c1, m1 = x_cl, x_mu
                x_cl = _act(x_cl, c2, lim_cl); x_mu = _act(x_mu, m2, lim_mu)
                c2, m2 = x_cl, x_mu
                x_cl = _act(x_cl, c3, lim_cl); x_mu = _act(x_mu, m3, lim_mu)
                c3, m3 = x_cl, x_mu
            k1 = derivatives_vec(t, s, ac, world, c1, m1, self.n_limit)
            k2 = derivatives_vec(t + dt / 2, s + dt / 2 * k1, ac, world, c2, m2, self.n_limit)
            k3 = derivatives_vec(t + dt / 2, s + dt / 2 * k2, ac, world, c2, m2, self.n_limit)
            k4 = derivatives_vec(t + dt, s + dt * k3, ac, world, c3, m3, self.n_limit)
            sn = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            sn = np.where(dead[None, :], s, sn)       # frozen once dead
            bad = ~np.all(np.isfinite(sn), axis=0) | (sn[3] < 0.5 * ac.v_stall)
            sn = np.where(bad[None, :], s, sn)
            s = sn
            x, y, z, V, g, p = s
            hc = self.clearance_height(world, x, y, z, g, m3, t + dt)
            live = ~dead & ~bad
            q = 0.5 * ac.rho * V * V * ac.wing_area
            n = q * c3_cmd / (ac.mass * G)
            pen = (self.w_clear * (np.maximum(0.0, self.h_clear - hc) / self.h_clear) ** 2
                   + self.w_n * np.maximum(0.0, n - ac.n_max) ** 2
                   + self.w_stall * (np.maximum(0.0, vmin - V) / ac.v_stall) ** 2)
            cost += np.where(live, dt * pen, 0.0)
            hit = live & (hc < 0.0)
            newly = hit | (bad & ~dead)
            t_dead = np.where(newly, (i + 1) * dt, t_dead)
            dead |= newly
            path[i + 1] = s
        # terminal terms
        e1 = G * s[2] + 0.5 * s[3] ** 2
        if self.guide is None:
            prog = (s[0] - s0[0]) * target_dir[0] + (s[1] - s0[1]) * target_dir[1]
            cost += -self.w_e * (e1 - e0) - self.w_p * prog
        else:
            cost += self.guide.terminal_cost(world, t0 + N * dt, s, self)
        # smoothness of the knot differences
        d = np.diff(Us, axis=2)
        cost += (self.guide is None) * self.w_smooth * (np.sum((d[:, 0] / self.sigma_cl) ** 2, axis=1)
                                 + np.sum((d[:, 1] / self.sigma_mu) ** 2, axis=1)) / self.n_ctrl
        cost += np.where(dead, self.c_contact * (1.0 + (self.horizon_s - t_dead) / self.horizon_s),
                         0.0)
        return cost, path, dead

    def update(self, U, Us, cost, dead):
        """The MPPI weighted update of the nominal U from the samples Us (S, 2, n_ctrl) with their
        costs and dead flags. Returns (new U, effective sample size, number dead)."""
        ok = ~dead
        scale = float(np.std(cost[ok])) if ok.sum() >= 2 else float(np.std(cost))
        scale = max(scale, 1e-6)
        w = np.exp(-(cost - cost.min()) / (self.lam * scale))
        w /= w.sum()
        ess = float(1.0 / np.sum(w ** 2))
        return self._clip(U + np.tensordot(w, Us - U[None], axes=(0, 0))), ess, int(dead.sum())

    # -- one planning call ---------------------------------------------------------------------
    def plan(self, world, t, state, target_dir, guide=None):
        """Plan from state at time t in `world`. Returns a Plan. target_dir: unit (ux, uy).

        With a guide (CycleGuide) the centre of the search is the time-shifted library cycle
        (sample 0), the shifted previous plan is sample 1, and the terminal cost is the distance
        from the library cycle at the horizon end."""
        self.guide = guide
        cold = self.U is None
        prev = None if cold else self._shift(self.U)
        if guide is not None:
            U = guide.nominal(self.knots_t)
            n_it = self.n_iter
        else:
            U = self._initial_plan() if cold else prev
            n_it = self.n_iter_cold if cold else self.n_iter
        s0 = np.asarray(state, float)
        sig = np.array([[self.sigma_cl], [self.sigma_mu]])
        ess = 1.0
        n_dead = 0
        for _ in range(n_it):
            if self.eps_hook is not None:
                eps = np.array(self.eps_hook(_), float)
            else:
                eps = self.rng.standard_normal((self.n_samples, 2, self.n_ctrl)) * sig
                if self.smooth_modes:
                    z = self.rng.standard_normal((self.n_samples, 2, self.smooth_modes))
                    eps += (self.smooth_gain * sig[None, :, :] / np.sqrt(self.smooth_modes)
                            * (z @ self._basis))
            eps[0] = 0.0
            Us = self._clip(U[None] + eps)
            cand = self._candidates()
            Us[1:1 + len(cand)] = cand
            if guide is not None and prev is not None:
                Us[1 + len(cand)] = self._clip(prev.copy())
            cost, _, dead = self.rollout(world, t, s0, Us, target_dir)
            U, ess, n_dead = self.update(U, Us, cost, dead)
        self.U = U
        cost, path, dead = self.rollout(world, t, s0, U[None], target_dir)
        nfirst = int(round(self.replan_s / self.dt))
        times = np.arange(self.nsteps + 1) * self.dt
        cl_k, mu_k = U[0].copy(), U[1].copy()
        return Plan(self.knots_t.copy(), cl_k, mu_k, times[:nfirst + 1],
                    np.interp(times[:nfirst + 1], self.knots_t, cl_k),
                    np.interp(times[:nfirst + 1], self.knots_t, mu_k),
                    path[:, :, 0].copy(), times, float(cost[0]), ess, n_dead)


# ---------------------------------------------------------------------------------------------
# cycle library (sim.dsopt periodic travel cycles) and the guide built from it
# ---------------------------------------------------------------------------------------------

LIB_PATH = os.path.join(os.path.dirname(__file__), "out", "mpc_library.json")
LIB_CHI_DEG = (0.0, 20.0, 45.0, 90.0)
LIB_U10 = (11.0, 13.0, 16.0)
LIB_FALLBACK_U10 = (11.5, 12.0, 12.5)
LIB_MARGIN_PATH = os.path.join(os.path.dirname(__file__), "out", "mpc_library_margin.json")
MARGIN_U10 = (11.0, 13.0, 16.0, 20.0)
MARGIN_SETS = (("standard", (2.0, 0.8, 0.9, 1.2)), ("light", (1.5, 0.9, 0.95, 1.15)))


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _solve_entry(job):
    """One library cycle: max speed toward chi at fixed u10, geometric wingtip, wave mode with
    a = 0 (no wave wind; the wave mode adds the h >= h_min and tip constraints at the
    Hermite-Simpson midpoints; with a = 0 dsopt skips the wave phase condition)."""
    from sim.aircraft_models import by_name
    from sim.dsopt import profile_for, solve
    name, u10, chi_deg, n = job
    ac = by_name(name)
    tried = []
    for u in (u10,) + (LIB_FALLBACK_U10 if u10 <= 11.0 else ()):
        t0 = time.time()
        info, x, col = solve(ac, profile_for(u), mode="max_speed", cycle="travel", n=n, u10=u,
                             chi=np.radians(chi_deg), tip_model="geometric",
                             wave={"a": 0.0, "wavelength": 100.0, "to_deg": 0.0,
                                   "midpoints": True},
                             return_x=True, stop_after=1)
        tried.append({"u10": u, "success": bool(info["success"]),
                      "violation": float(info["violation"]), "runtime_s": time.time() - t0})
        if info["success"]:
            S, C, T, _ = col.unpack(x)
            t = np.linspace(0.0, T, n)
            r = lambda a: [float(f"{v:.6g}") for v in np.asarray(a)]
            return {"aircraft": name, "u10": float(u), "u10_requested": float(u10),
                    "chi_deg": float(chi_deg), "success": True, "period": float(T),
                    "speed_along_chi": float(info["speed_along_chi"]),
                    "z_min": float(info["z_min"]), "z_min_mid": float(info["z_min_mid"]),
                    "n_max": float(info["n_max"]), "tried": tried,
                    "t": r(t), "x": r(S[:, 0]), "y": r(S[:, 1]), "z": r(S[:, 2]),
                    "V": r(S[:, 3]), "gamma": r(S[:, 4]), "psi": r(S[:, 5]),
                    "cl": r(C[:, 0]), "mu": r(C[:, 1])}
    return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
            "chi_deg": float(chi_deg), "success": False, "tried": tried}


def build_library(names=("baseline-2m", "albatross-3m"), u10s=LIB_U10, chis=LIB_CHI_DEG, n=40,
                  workers=8, path=LIB_PATH):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    jobs = [(a, u, c, n) for a in names for u in u10s for c in chis]
    t0 = time.time()
    with ProcessPoolExecutor(workers) as ex:
        entries = list(ex.map(_solve_entry, jobs))
    meta = record.meta("python -m sim.mpc --build-library",
                       [__file__, os.path.join(os.path.dirname(__file__), "dsopt.py")])
    meta.update({"runtime_s": time.time() - t0, "n_nodes": n,
                 "dsopt": "solve(mode=max_speed, cycle=travel, tip_model=geometric, "
                          "wave={a: 0, midpoints: True}, profile_for(u10) Charnock z0, "
                          "h_min 1.0, h_tip_min 0.3)",
                 "frame": "wind toward +x, angles ccw from +x; z is the height above the surface"})
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump({"meta": meta, "entries": entries}, fh, indent=1)
        fh.write("\n")
    return entries


def _solve_margin_entry(job):
    """One margin-library cycle: dsopt flat mode, max speed toward chi at fixed u10, coare35
    roughness, geometric wingtip, Hermite-Simpson midpoints, pb2v rate limits, n >= 0, with the
    standard margins and, if that fails, the lighter ones. The entry holds the margins used and
    the dense-replay figures of dsopt.control_stats."""
    from sim.aircraft_models import by_name
    from sim.dsopt import control_stats, profile_for, solve
    name, u10, chi_deg, n = job
    ac = by_name(name)
    prof = profile_for(u10, "log", roughness="coare35")
    tried = []
    for label, (h_min, n_frac, mu_frac, v_factor) in MARGIN_SETS:
        t0 = time.time()
        info, x, col = solve(ac, prof, mode="max_speed", cycle="travel", n=n, u10=u10,
                             chi=np.radians(chi_deg), tip_model="geometric", midpoints=True,
                             rate_limits=True, rate_model="pb2v", n_min=0.0, h_min=h_min,
                             n_frac=n_frac, mu_frac=mu_frac, v_factor=v_factor,
                             return_x=True, stop_after=1)
        tried.append({"margins": label, "success": bool(info["success"]),
                      "violation": float(info["violation"]), "runtime_s": time.time() - t0})
        if info["success"]:
            S, C, T, _ = col.unpack(x)
            t = np.linspace(0.0, T, n)
            r = lambda a: [float(f"{v:.6g}") for v in np.asarray(a)]
            cs = control_stats(col, x)
            return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
                    "chi_deg": float(chi_deg), "success": True, "period": float(T),
                    "margin_set": label,
                    "margins": {"h_min": h_min, "n_frac": n_frac, "mu_frac": mu_frac,
                                "v_factor": v_factor, "n_min": 0.0, "h_tip_min": col.h_tip,
                                "k_roll": col.k_roll, "cl_rate_max_per_s": col.r_max},
                    "speed_along_chi": float(info["speed_along_chi"]),
                    "z_min": float(info["z_min"]), "z_min_mid": float(info["z_min_mid"]),
                    "n_max": float(info["n_max"]), "replay": cs, "tried": tried,
                    "t": r(t), "x": r(S[:, 0]), "y": r(S[:, 1]), "z": r(S[:, 2]),
                    "V": r(S[:, 3]), "gamma": r(S[:, 4]), "psi": r(S[:, 5]),
                    "cl": r(C[:, 0]), "mu": r(C[:, 1])}
    return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
            "chi_deg": float(chi_deg), "success": False, "missing": True, "tried": tried}


def build_margin_library(names=("baseline-2m", "albatross-3m"), u10s=MARGIN_U10,
                         chis=LIB_CHI_DEG, n=40, workers=6, path=LIB_MARGIN_PATH):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    jobs = [(a, u, c, n) for a in names for u in u10s for c in chis]
    t0 = time.time()
    with ProcessPoolExecutor(workers) as ex:
        entries = list(ex.map(_solve_margin_entry, jobs, chunksize=1))
    meta = record.meta("python -m sim.mpc --build-margin-library",
                       [__file__, os.path.join(os.path.dirname(__file__), "dsopt.py")])
    meta.update({"runtime_s": time.time() - t0, "n_nodes": n,
                 "dsopt": "solve(mode=max_speed, cycle=travel, flat mode, tip_model=geometric, "
                          "midpoints=True, rate_limits=True, rate_model=pb2v, n_min=0, "
                          "profile_for(u10, log, roughness=coare35), default seeds and tolerance)",
                 "margin_sets": {k: dict(zip(("h_min", "n_frac", "mu_frac", "v_factor"), v))
                                 for k, v in MARGIN_SETS},
                 "margin_order": "standard first; light only where standard did not converge",
                 "frame": "wind toward +x, angles ccw from +x; z is the height above the surface"})
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump({"meta": meta, "entries": entries}, fh, indent=1)
        fh.write("\n")
    return entries


class Cycle:
    """One periodic library cycle in the wind frame (wind toward +x), optionally mirrored."""

    def __init__(self, d, mirror=False):
        a = lambda k: np.asarray(d[k], float)
        self.d, self.mirror = d, mirror
        sg = -1.0 if mirror else 1.0
        self.T = float(d["period"])
        self.t = a("t")
        self.x, self.y = a("x"), sg * a("y")
        self.z, self.V, self.g = a("z"), a("V"), a("gamma")
        self.psi = np.unwrap(sg * a("psi"))
        self.cl, self.mu = a("cl"), sg * a("mu")
        self.disp = np.array([self.x[-1] - self.x[0], self.y[-1] - self.y[0]])
        self.v_mean = float(np.mean(self.V))
        self.i_low = int(np.argmin(self.z))
        self.tau_low = float(self.t[self.i_low])

    def _p(self, tau, arr):
        return np.interp(np.mod(tau, self.T), self.t, arr)

    def state(self, tau):
        """(h, V, gamma, psi relative to the wind) of the cycle at phase tau."""
        return (self._p(tau, self.z), self._p(tau, self.V), self._p(tau, self.g),
                float(wrap(self._p(tau, self.psi))))

    def controls(self, tau):
        return self._p(tau, self.cl), self._p(tau, self.mu)

    def pos(self, tau):
        k = np.floor(np.asarray(tau) / self.T)
        return np.stack([self._p(tau, self.x) + k * self.disp[0],
                         self._p(tau, self.y) + k * self.disp[1]], axis=-1)

    def energy(self, tau):
        h, V, _, _ = self.state(tau)
        return G * h + 0.5 * V * V


class CycleLibrary:
    def __init__(self, entries):
        self.entries = [e for e in entries if e.get("success")]

    @classmethod
    def load(cls, path=LIB_PATH):
        with open(path) as fh:
            return cls(json.load(fh)["entries"])

    @classmethod
    def from_keyword(cls, library_path=None):
        """"margin" picks LIB_MARGIN_PATH, None the default file, else a file path."""
        if library_path is None:
            return cls.load()
        return cls.load(LIB_MARGIN_PATH if library_path == "margin" else library_path)

    def pick(self, name, u10_est, chi_rad):
        """Cycle for the aircraft nearest to u10_est and |chi| (mirrored for negative chi)."""
        es = [e for e in self.entries if e["aircraft"] == name]
        if not es:
            raise KeyError(name)
        u = min({e["u10"] for e in es}, key=lambda v: abs(v - u10_est))
        es = [e for e in es if e["u10"] == u]
        chi = abs(np.degrees(chi_rad))
        e = min(es, key=lambda e: abs(e["chi_deg"] - chi))
        return Cycle(e, mirror=chi_rad < 0.0)


def dominant_wave(world):
    """(a, k, dx, dy, phase, omega) of the largest component (longest wavelength on ties) of a
    world that is an Environment (or has .c), else None."""
    c = getattr(world, "c", None)
    if c is None or len(c["a"]) == 0:
        return None
    i = int(np.lexsort((c["k"], -np.round(c["a"], 3)))[0])
    return (float(c["a"][i]), float(c["k"][i]), float(c["dx"][i]), float(c["dy"][i]),
            float(c["phase"][i]), float(c["omega"][i]))


class CycleGuide:
    """Library cycle matched to the current state and shifted in phase by the predicted wave.

    The match uses the air-relative state: height above the (estimated) surface, V, gamma and the
    heading relative to the wind. Nominal controls are the cycle's controls from the shifted
    phase on. The terminal cost is the weighted squared distance of the rollout end from the cycle
    at the end phase, plus the energy deficit (g h + V^2/2) against the cycle's own change over
    the horizon."""

    SIG = (3.0, None, 0.25, 0.6)     # h (m), V (None: 0.25 mean V), gamma, heading (rad)

    def __init__(self, cycle, world, t, s, psi_w, horizon, wave_aware=True, w_d=10.0,
                 d_max=0.15, theta_star=np.pi / 3, tau_hint=None, hint_window=0.2):
        self.cycle, self.psi_w, self.H, self.w_d = cycle, psi_w, horizon, w_d
        self.sig = (self.SIG[0], self.SIG[1] or 0.25 * cycle.v_mean, self.SIG[2], self.SIG[3])
        x, y, z, V, g, psi = (float(v) for v in s)
        eta = float(world.surface(x, y, t)[0])
        self.h0 = z - eta
        feat = np.array([self.h0, V, g, wrap(psi - psi_w)])
        T = cycle.T
        taus = np.linspace(0.0, T, 80, endpoint=False)
        st = np.array([cycle.state(tau) for tau in taus])
        dist = (((st[:, 0] - feat[0]) / self.sig[0]) ** 2 + ((st[:, 1] - feat[1]) / self.sig[1]) ** 2
                + ((st[:, 2] - feat[2]) / self.sig[2]) ** 2
                + (wrap(st[:, 3] - feat[3]) / self.sig[3]) ** 2)
        if tau_hint is not None:
            # continuity: only phases within +-hint_window T of the predicted one compete
            far = np.abs(wrap((taus - tau_hint) * 2 * np.pi / T)) > hint_window * 2 * np.pi
            dist = np.where(far, np.inf, dist)
        self.tau0 = float(taus[int(np.argmin(dist))])
        self.match_dist = float(dist.min())
        self.delta = 0.0
        self.theta_low = None
        self._theta0 = None
        wave = dominant_wave(world) if wave_aware else None
        if wave is not None:
            a, k, dx, dy, ph, om = wave
            best = None
            R = np.array([[np.cos(psi_w), -np.sin(psi_w)], [np.sin(psi_w), np.cos(psi_w)]])
            for d in np.linspace(-d_max * T, d_max * T, 13):
                ts = self.tau0 + d
                dt_low = (cycle.tau_low - ts) % T
                p = np.array([x, y]) + R @ (cycle.pos(ts + dt_low) - cycle.pos(ts))
                th = k * (dx * p[0] + dy * p[1]) - om * (t + dt_low) + ph
                err = abs(float(wrap(th - theta_star)))
                if abs(d) < 1e-12:
                    self._theta0 = float(wrap(th))
                if best is None or err < best[0]:
                    best = (err, d, th)
            self.delta, self.theta_low = float(best[1]), float(wrap(best[2]))
        self.tau_s = self.tau0 + self.delta
        te = self.tau_s + horizon
        self.target = cycle.state(te)
        self.dE_c = float(cycle.energy(te) - cycle.energy(self.tau_s))
        self.E0 = G * self.h0 + 0.5 * V * V

    def nominal(self, knots_t):
        cl, mu = self.cycle.controls(self.tau_s + np.asarray(knots_t))
        return np.stack([cl, mu])

    def terminal_cost(self, world, t_end, s, planner):
        x, y, z, V, g, psi = s
        h = z - np.asarray(world.surface(x, y, np.full(np.shape(x), float(t_end)))[0])
        ht, Vt, gt, pt = self.target
        d2 = (((h - ht) / self.sig[0]) ** 2 + ((V - Vt) / self.sig[1]) ** 2
              + ((g - gt) / self.sig[2]) ** 2 + (wrap(psi - self.psi_w - pt) / self.sig[3]) ** 2)
        deficit = np.maximum(0.0, self.dE_c - (G * h + 0.5 * V * V - self.E0))
        return self.w_d * d2 + planner.w_e * deficit


# ---------------------------------------------------------------------------------------------
# closed loop on the true dynamics
# ---------------------------------------------------------------------------------------------


class _Limited:
    """A world restricted to its n strongest components by a*k (a copy of the Environment)."""

    def __init__(self, env, n):
        self.wind = env.wind
        k = env.c["a"] * env.c["k"]
        keep = np.sort(np.argsort(-k)[:n])
        e = Environment(env.wind, [], seed=env.seed, decay=env.decay)
        e.c = {key: v[keep] for key, v in env.c.items()}
        e.urel = env.urel[keep]
        self.env = e
        self.c = e.c
        self.wind_at = e.wind_at
        self.surface = e.surface


def _flat_world(env_true, u10_est):
    return Environment.flat(Wind(u10_est, env_true.wind.from_deg))


def _flat_world_uf(u10, from_deg):
    return Environment.flat(Wind(u10, from_deg))


def wingtip_height(ac, z, eta, g, mu):
    return z - eta - 0.5 * ac.span * np.sin(np.abs(mu)) * np.cos(g)


def traj_metrics(ac, env, t, st, mu, n, bearing_u):
    """Metrics of a recorded trajectory on the true environment (shared with the old tracker)."""
    t, st = np.asarray(t), np.asarray(st)
    eta = np.asarray(env.surface(st[:, 0], st[:, 1], t)[0])
    h = st[:, 2] - eta
    hw = wingtip_height(ac, st[:, 2], eta, st[:, 4], np.asarray(mu))
    e = G * st[:, 2] + 0.5 * st[:, 3] ** 2
    dur = float(t[-1] - t[0])
    out = {"duration_s": dur,
           "energy_per_min": float((e[-1] - e[0]) / dur * 60.0) if dur > 0 else 0.0,
           "min_clear_surface": float(h.min()), "min_clear_wingtip": float(hw.min()),
           "max_n": float(np.max(n)),
           "v_toward_target": float(((st[-1, :2] - st[0, :2]) @ bearing_u) / dur) if dur > 0 else 0.0}
    return out


def fly_mpc(aircraft, env_true, world_model, sensors, t_end, target_bearing_deg, u10_est,
            seed=1, planner_kw=None, dt=0.01, max_components=None, estimator=None, keep=False,
            library=None, wave_aware=True, state_source="truth", wind_source="truth",
            suite="standard", profile_fit="two_param", actuators=None, n_limit=False,
            log_every_s=5.0, stop_on_tip=False, library_path=None):
    """Closed loop: true dynamics (rk4, dt), planner every replan_s in the chosen world model.

    world_model: "truth" | "flat" | "estimated". sensors: list of SurfaceSensor (used only for
    "estimated"; also fed to the estimator). max_components: restrict the "truth" world to its
    strongest components (speed). Returns a metrics dict (and the trajectory if keep).

    state_source / wind_source: "truth" (today) or "estimate". With an estimate a SensorSuite
    (suite: "minimal" | "standard" | "full") and an Estimator(profile_fit) run in the loop; the
    planner then starts from the estimated state, and the library cycle and the flat world use the
    estimated u10 and wind direction (see the module docstring). actuators: an
    sim.aircraft.Actuators or None (fly the actual controls); n_limit: load-factor limit in the
    flight model. The metrics then also hold max_n_cmd, max_n_flown, n_limit_bind_frac,
    actuator_bind and, with estimates, the time series "est_log".
    """
    from sim.seasense import LocalWaveEstimator
    ac = aircraft
    if library is None and library_path is not None:
        library = CycleLibrary.from_keyword(library_path)   # a library file picked by keyword
    kw = dict(seed=seed)
    kw.update(planner_kw or {})
    pl = MPPIPlanner(ac, **kw)
    pl.n_limit = bool(n_limit)
    pl.act = actuators
    rng = np.random.default_rng(seed + 1000)
    est = (estimator or LocalWaveEstimator()) if world_model == "estimated" else None
    for sn in sensors or []:
        sn.reset()
    brg = np.radians(target_bearing_deg)
    u = np.array([np.sin(brg), np.cos(brg)])                  # (east, north)
    psi0 = float(np.arctan2(u[1], u[0]))
    s = np.array([0.0, 0.0, 10.0 + float(env_true.surface(0.0, 0.0, 0.0)[0]),
                  2.0 * ac.v_stall, 0.0, psi0])
    psi_w = float(np.arctan2(env_true.wind.ey, env_true.wind.ex))
    chi = float(wrap(np.arctan2(u[1], u[0]) - psi_w))
    if library is not None:
        # start on the library cycle at its highest node (as the old tracker does)
        c0 = library.pick(ac.name, u10_est, chi)
        k0 = int(np.argmax(c0.z))
        s = np.array([0.0, 0.0, float(env_true.surface(0.0, 0.0, 0.0)[0]) + c0.z[k0], c0.V[k0],
                      c0.g[k0], psi_w + c0.psi[k0]])
    use_est = state_source == "estimate" or wind_source == "estimate"
    if state_source not in ("truth", "estimate") or wind_source not in ("truth", "estimate"):
        raise ValueError("state_source and wind_source must be 'truth' or 'estimate'")
    if wind_source == "estimate" and world_model not in ("flat", "estimated"):
        raise ValueError("wind_source='estimate' needs world_model 'flat' or 'estimated'")
    if use_est:
        from sim.estimator import Estimator
        from sim.sensors import SensorSuite, truth_from_state
        ssuite = getattr(SensorSuite, suite)(seed)
        sest = Estimator(ssuite.names, profile_fit=profile_fit, sample_dt=True)
    if actuators is not None:
        actuators.reset()
    tau_prev, n_cyc, deltas, mdist = None, 0, [], []
    if world_model == "truth":
        w_truth = env_true if max_components is None else _Limited(env_true, max_components)
    w_flat = _flat_world(env_true, u10_est)
    from_deg = env_true.wind.from_deg

    T, ST, MU, CL, NN = [], [], [], [], []
    NF, NBND, NCMD = [], [], []
    est_log = []
    e_cur, ready_wind, u10_cur, from_cur, psi_w_cur = None, False, u10_est, from_deg, psi_w
    prev_truth = None
    z0_true = float(env_true.wind.z0)
    n_prelude = 0
    cpu, fit_cpu, fit_q, fit_n, pred_err, ess, ndead = [], [], [], [], [], [], []
    plan, t_plan = None, 0.0
    t, ended = 0.0, "complete"
    mu = 0.0
    cl = ac.cl_best_ld
    nsteps = int(round(t_end / dt))
    steps_per_plan = int(round(pl.replan_s / dt))
    t_ready = None
    mu_a, cl_a = 0.0, ac.cl_best_ld
    for i in range(nsteps + 1):
        t = i * dt
        eta_t = float(env_true.surface(s[0], s[1], t)[0])
        if s[2] - eta_t < 0.0:
            ended = "contact"
            break
        if stop_on_tip and wingtip_height(ac, s[2], eta_t, s[4], mu_a) < 0.0:
            ended = "tip contact"
            break
        if not np.all(np.isfinite(s)):
            ended = "non-finite"
            break
        ready = True
        if use_est:
            truth = truth_from_state(s, env_true, t, mu_a, prev_truth)
            e_cur = sest.update(ssuite.measure(truth, t), t)
            if (wind_source == "estimate" and e_cur.valid and e_cur.wind_valid
                    and e_cur.u10_est > 0.0 and np.isfinite(e_cur.wind_dir_est_deg)):
                ready_wind = True
                u10_cur, from_cur = float(e_cur.u10_est), float(e_cur.wind_dir_est_deg)
                psi_w_cur = float(np.pi / 2 - np.radians(from_cur + 180.0))
            ready = ((state_source == "truth" or bool(e_cur.valid))
                     and (wind_source == "truth" or ready_wind))
            if ready and t_ready is None:
                t_ready = t
            if i % int(round(log_every_s / dt)) == 0:
                w_t = np.asarray(env_true.wind_at(s[0], s[1], s[2], t), float).reshape(3)
                w_e = np.asarray(e_cur.wind, float)
                est_log.append({
                    "t": t, "ready": bool(ready),
                    "wind_speed_err": float(np.hypot(*w_e[:2]) - np.hypot(*w_t[:2])),
                    "wind_dir_err_deg": float(np.degrees(wrap(np.arctan2(w_e[1], w_e[0])
                                                               - np.arctan2(w_t[1], w_t[0])))),
                    "profile_dir_err_deg": float(np.degrees(wrap(np.radians(
                        e_cur.wind_dir_est_deg - env_true.wind.from_deg)))),
                    "u10_est": float(e_cur.u10_est), "u10_true": float(env_true.wind.u10),
                    "z0_est": float(e_cur.z0_est), "z0_true": z0_true,
                    "h_err": float(e_cur.h_surface - (s[2] - eta_t)),
                    "speed_err": float(e_cur.airspeed - s[3])})
        if ready and (plan is None or i % steps_per_plan == 0):
            c0 = time.process_time()
            if world_model == "truth":
                world = w_truth
            elif world_model == "flat":
                world = (_flat_world_uf(u10_cur, from_cur) if wind_source == "estimate"
                         else w_flat)
            else:
                f0 = time.process_time()
                fit = est.fit(t)
                fit_cpu.append(time.process_time() - f0)
                est_w = wind_source == "estimate"
                if fit is None:
                    world = _flat_world_uf(u10_cur, from_cur) if est_w else w_flat
                else:
                    world = fit.environment(u10_cur if est_w else u10_est,
                                            from_cur if est_w else from_deg, env_true.decay)
                    fit_q.append(fit.quality)
                    fit_n.append(fit.n_samples)
            if state_source == "estimate":
                px_, py_ = float(e_cur.pos[0]), float(e_cur.pos[1])
                va = np.asarray(e_cur.v_air, float)
                gam_e = float(np.arcsin(np.clip(va[2] / max(float(np.linalg.norm(va)), 1e-6),
                                                -1.0, 1.0)))
                s_plan = np.array([px_, py_, float(e_cur.h_surface)
                                   + float(world.surface(px_, py_, t)[0]),
                                   float(e_cur.airspeed), gam_e, float(e_cur.heading)])
            else:
                s_plan = s
            psi_p = psi_w_cur if wind_source == "estimate" else psi_w
            chi_p = float(wrap(np.arctan2(u[1], u[0]) - psi_p))
            u10_p = u10_cur if wind_source == "estimate" else u10_est
            guide = None
            if library is not None:
                cyc = library.pick(ac.name, u10_p, chi_p)
                guide = CycleGuide(cyc, world, t, s_plan, psi_p, pl.horizon_s,
                                  wave_aware=wave_aware, w_d=pl.w_d,
                                  tau_hint=None if tau_prev is None else tau_prev + pl.replan_s)
                if tau_prev is not None and guide.tau0 < tau_prev - 0.5 * cyc.T:
                    n_cyc += 1
                tau_prev = guide.tau0
                deltas.append(guide.delta)
                mdist.append(guide.match_dist)
            if actuators is not None and actuators.cl is not None:
                pl.act_state = (actuators.cl, actuators.mu) + actuators.cmd_last
            plan = pl.plan(world, t, s_plan, u, guide=guide)
            t_plan = t
            cpu.append(time.process_time() - c0)
            ess.append(plan.ess)
            ndead.append(plan.n_dead)
            if world_model == "estimated" and fit is not None:
                px, py, ptt = plan.path[::10, 0], plan.path[::10, 1], t + plan.times[::10]
                pe = np.asarray(world.surface(px, py, ptt)[0]) - np.asarray(
                    env_true.surface(px, py, ptt)[0])
                pred_err.append(float(np.sqrt(np.mean(pe ** 2))))
        if ready:
            cl, mu = clip_controls(ac, *plan.controls(t - t_plan))
        else:
            n_prelude += 1
            cl, mu = ac.cl_best_ld, np.radians(20.0) * (1.0 if int(t // 10.0) % 2 == 0 else -1.0)
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        if n_limit:
            cl = n_limited_cl(ac, s[3], cl)       # commanded cl is held to the limit
        cl_a, mu_a = (cl, mu) if actuators is None else actuators.step(cl, mu, dt, s[3])
        cl_f = n_limited_cl(ac, s[3], cl_a) if n_limit else cl_a
        T.append(t); ST.append(s.copy()); MU.append(mu_a); CL.append(cl_f)
        NN.append(q * cl_f / (ac.mass * G))
        NCMD.append(q * cl / (ac.mass * G))
        NBND.append(float(cl_f < cl_a - 1e-12))
        if world_model == "estimated":
            zk = float(s[2]) + 0.1 * rng.standard_normal()
            for sn in sensors:
                got = sn.measure(s, mu_a, env_true, t, rng)
                if got:
                    est.add(got, zk)
        if i == nsteps:
            break
        if use_est:
            prev_truth = (s.copy(), t, mu_a)
        s = rk4_step(lambda tt, ss: derivatives(tt, ss, ac, env_true, cl_a, mu_a, n_limit)[0],
                     t, s, dt)
    if ended == "complete" and not np.all(np.isfinite(s)):
        ended = "non-finite"
    out = {"ended": ended, "survival_s": float(t if ended != "complete" else t_end)}
    if len(T) > 1:
        out.update(traj_metrics(ac, env_true, T, ST, MU, NN, u))
    out["cpu_mean_s"] = float(np.mean(cpu)) if cpu else 0.0
    out["cpu_max_s"] = float(np.max(cpu)) if cpu else 0.0
    out["n_replans"] = len(cpu)
    if library is not None:
        out["cycles"] = n_cyc
        out["phase_shift_mean_abs_s"] = float(np.mean(np.abs(deltas))) if deltas else 0.0
        out["match_dist_mean"] = float(np.mean(mdist)) if mdist else None
    out["ess_mean"] = float(np.mean(ess)) if ess else 0.0
    out["dead_frac_mean"] = float(np.mean(ndead) / pl.n_samples) if ndead else 0.0
    if world_model == "estimated":
        out["fit_cpu_mean_s"] = float(np.mean(fit_cpu)) if fit_cpu else 0.0
        out["fit_quality_mean"] = float(np.mean(fit_q)) if fit_q else None
        out["fit_quality_median"] = float(np.median(fit_q)) if fit_q else None
        out["fit_samples_mean"] = float(np.mean(fit_n)) if fit_n else 0.0
        out["fit_available_frac"] = float(len(fit_q) / max(len(cpu), 1))
        out["eta_pred_rms"] = float(np.sqrt(np.mean(np.square(pred_err)))) if pred_err else None
    if len(T) > 1:
        Ta, Sa = np.asarray(T), np.asarray(ST)
        j = int(np.searchsorted(Ta, Ta[-1] - 60.0))
        Ea = G * Sa[:, 2] + 0.5 * Sa[:, 3] ** 2
        out["energy_per_min_last60"] = float((Ea[-1] - Ea[j]) / (Ta[-1] - Ta[j]) * 60.0)
        out["energy_trend_window_s"] = float(Ta[-1] - Ta[j])
    out["max_n_cmd"] = float(np.max(NCMD)) if NCMD else None
    out["max_n_flown"] = float(np.max(NN)) if NN else None
    out["n_limit_bind_frac"] = float(np.mean(NBND)) if NBND else 0.0
    if actuators is not None:
        out["actuator_bind"] = actuators.bind_fractions()
    if use_est:
        out["t_ready_s"] = t_ready
        out["prelude_s"] = n_prelude * dt
        out["est_log"] = est_log
        post = [r for r in est_log if r["ready"]]
        if post:
            out["est_summary"] = {
                "wind_speed_err_rms": float(np.sqrt(np.mean([r["wind_speed_err"] ** 2 for r in post]))),
                "wind_dir_err_rms_deg": float(np.sqrt(np.mean([r["wind_dir_err_deg"] ** 2 for r in post]))),
                "profile_dir_err_rms_deg": float(np.sqrt(np.mean([r["profile_dir_err_deg"] ** 2 for r in post]))),
                "u10_est_last": post[-1]["u10_est"], "z0_est_last": post[-1]["z0_est"]}
    if keep:
        out["traj"] = {"t": np.array(T), "state": np.array(ST), "mu": np.array(MU),
                       "cl": np.array(CL)}
    return out


# ---------------------------------------------------------------------------------------------
# experiment
# ---------------------------------------------------------------------------------------------

MODELS = ("flat", "truth", "est-radar", "est-lidar", "est-radar+fwd")


def _make_env(u10):
    from sim.autonomy import make_env
    return make_env("waves", u10)


def _sensors(model):
    from sim.seasense import ForwardScanner, LidarAltimeter, RadarAltimeter
    return {"est-radar": [RadarAltimeter()], "est-lidar": [LidarAltimeter()],
            "est-radar+fwd": [RadarAltimeter(), ForwardScanner()]}.get(model, [])


def run_job(job):
    """job = (aircraft name, u10, model, t_end, planner_kw, max_components)."""
    from sim.aircraft_models import by_name
    name, u10, model, t_end, pkw, maxc = job
    ac = by_name(name)
    env = _make_env(u10)
    bearing = env.wind.to_deg - 20.0          # 20 degrees off downwind (math ccw = compass minus)
    t0 = time.time()
    if model == "tracker":
        r = run_tracker(ac, env, u10, t_end, bearing)
    else:
        wm = "estimated" if model.startswith("est-") else model
        r = fly_mpc(ac, env, wm, _sensors(model), t_end, bearing, u10, seed=1,
                    planner_kw=pkw, max_components=maxc if wm == "truth" else None)
    r["wall_s"] = time.time() - t0
    return {"aircraft": name, "u10": u10, "model": model, **r}


def run_tracker(ac, env, u10, t_end, bearing_compass):
    """The old method (sim.autonomy.ReferenceTracker) with perfect knowledge, same metrics."""
    from sim.autonomy import (PerfectEstimator, ReferenceTracker, fly, initial_state_ref,
                              reference_from_dsopt)
    ref = reference_from_dsopt(ac, u10, "log")
    psi_w = np.arctan2(env.wind.ey, env.wind.ex)
    target = 3000.0 * np.array([np.cos(psi_w + np.radians(20)), np.sin(psi_w + np.radians(20))])
    ctrl = ReferenceTracker(ac, ref)
    r = fly(ac, env, None, PerfectEstimator(), ctrl, initial_state_ref(env, ref), t_end,
            dt=0.02, seed=1, target=target)
    u = target / np.linalg.norm(target)
    out = traj_metrics(ac, env, r["t"], r["state"], r["mu"], r["n"], u)
    out["ended"] = r["ended"]
    out["survival_s"] = float(r["t"][-1]) if r["ended"] != "complete" else t_end
    return out


LIB_PKW = {"n_samples": 96, "dt": 0.1, "n_iter": 1, "n_ctrl": 24, "lam": 0.2,
           "sigma_cl": 0.08, "sigma_mu": 0.12, "clearance": "wingtip", "h_clear": 0.3}
LIB_MODELS = ("lib-flatenv", "lib-flat", "lib-truth", "lib-truth-nophase", "lib-est-radar+fwd",
              "tracker")


def run_lib_job(job):
    """job = (aircraft, u10, model, t_end, max_components). Library planner rows and the tracker."""
    from sim.aircraft_models import by_name
    from sim.autonomy import make_env
    name, u10, model, t_end, maxc = job
    ac = by_name(name)
    t0 = time.time()
    if model == "tracker":
        env = make_env("waves", u10)
        r = run_tracker(ac, env, u10, t_end, env.wind.to_deg - 20.0)
    else:
        env = make_env("flat" if model == "lib-flatenv" else "waves", u10)
        base = model[4:]
        wm = {"flatenv": "flat", "flat": "flat", "truth": "truth", "truth-nophase": "truth",
              "est-radar+fwd": "estimated"}[base]
        r = fly_mpc(ac, env, wm, _sensors("est-radar+fwd") if wm == "estimated" else [], t_end,
                    env.wind.to_deg - 20.0, u10, seed=1, planner_kw=dict(LIB_PKW),
                    max_components=maxc if wm == "truth" else None,
                    library=CycleLibrary.load(), wave_aware=(base != "truth-nophase"))
    r["wall_s"] = time.time() - t0
    return {"aircraft": name, "u10": u10, "model": model, **r}


def lib_table(rows):
    lines = ["| aircraft | u10 | model | ended | survival (s) | cycles | dE/min (J/kg) | min clear "
             "surf / tip (m) | max n | v toward (m/s) | cpu mean / max (s) |",
             "|" + "---|" * 11]
    for r in rows:
        lines.append("| %s | %g | %s | %s | %.0f | %s | %s | %s / %s | %s | %s | %.2f / %.2f |" % (
            r["aircraft"], r["u10"], r["model"], r["ended"], r["survival_s"],
            r.get("cycles", "n/a"), _fmt(r.get("energy_per_min"), "%.0f"),
            _fmt(r.get("min_clear_surface"), "%.2f"), _fmt(r.get("min_clear_wingtip"), "%.2f"),
            _fmt(r.get("max_n"), "%.2f"), _fmt(r.get("v_toward_target"), "%.1f"),
            r.get("cpu_mean_s", 0.0), r.get("cpu_max_s", 0.0)))
    return chr(10).join(lines)


def lib_main(a):
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    jobs = [(n, float(u), m, a.t_end, a.max_components) for n in a.aircraft.split(",")
            for u in a.u10.split(",") for m in a.models.split(",")]
    jobs.sort(key=lambda j: j[2] not in ("lib-truth", "lib-truth-nophase"))
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_lib_job, jobs))
    order = {m: i for i, m in enumerate(LIB_MODELS)}
    rows.sort(key=lambda r: (r["aircraft"], r["u10"], order[r["model"]]))
    print(lib_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    old = d.get("library_exp", {}).get("results", [])
    keep = [r for r in old if (r["aircraft"], r["u10"], r["model"])
            not in {(x["aircraft"], x["u10"], x["model"]) for x in rows}]
    rows = sorted(keep + rows, key=lambda r: (r["aircraft"], r["u10"], order[r["model"]]))
    d["library_exp"] = {
        "meta": {**record.meta("python -m sim.mpc --lib-exp --u10 " + a.u10 + " --aircraft "
                               + a.aircraft, [__file__, LIB_PATH]),
                 "planner_kw": LIB_PKW, "runtime_s": runtime, "t_end": a.t_end,
                 "max_components": a.max_components, "seed": 1,
                 "guide": "state match sigma (h 3 m, V 0.25 mean, gamma 0.25, heading 0.6 rad), "
                          "w_d 10, w_e 1, wave-aware phase shift +-0.15 T toward theta 60 deg"},
        "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))


EST_CASES = (
    # (label, sea, state/wind source, actuators + n_limit, suite)
    ("a1 flat sea, truth", "flat", "truth", False, "standard"),
    ("a2 flat sea, truth + act + nlim", "flat", "truth", True, "standard"),
    ("a3 flat sea, est standard + act + nlim", "flat", "estimate", True, "standard"),
    ("a4 flat sea, est full + act + nlim", "flat", "estimate", True, "full"),
    ("b1 sea, flat world, truth + act + nlim", "waves", "truth", True, "standard"),
    ("b2 sea, flat world, est standard + act + nlim", "waves", "estimate", True, "standard"),
)


def _no_nan(v):
    """NaN to None, recursively (the JSON file must stay standard)."""
    if isinstance(v, dict):
        return {k: _no_nan(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_no_nan(x) for x in v]
    return None if isinstance(v, float) and v != v else v


def run_est_job(job):
    """job = (aircraft, u10, case index, t_end). The planner world is flat (the u10 of the case
    or the estimate); the true sea is flat or make_env("waves")."""
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim.autonomy import make_env
    name, u10, k, t_end = job[:4]
    tip = len(job) > 4 and bool(job[4])
    label, sea, src, act, suite = EST_CASES[k]
    ac = by_name(name)
    env = make_env(sea, u10)
    t0 = time.time()
    r = fly_mpc(ac, env, "flat", [], t_end, env.wind.to_deg - 20.0, u10, seed=1,
                planner_kw=dict(LIB_PKW), library=CycleLibrary.load(), state_source=src,
                wind_source=src, suite=suite, profile_fit="two_param",
                actuators=Actuators(ac) if act else None, n_limit=act, stop_on_tip=tip)
    r["wall_s"] = time.time() - t0
    r = _no_nan(r)
    return {"aircraft": name, "u10": u10, "case": label, "sea": sea, "sources": src,
            "actuators_and_nlimit": act, "stop_on_tip": tip, "suite": suite if src == "estimate" else None, **r}


def est_table(rows):
    lines = ["| aircraft | case | ended | survival (s) | cycles | min clear surf / tip (m) | "
             "max n cmd / flown | n-limit binds | mu / cl rate binds | wind err rms (m/s, deg) | "
             "u10_est last | z0_est last |", "|" + "---|" * 12]
    for r in rows:
        ab = r.get("actuator_bind") or {}
        es = r.get("est_summary") or {}
        lines.append("| %s | %s | %s | %.0f | %s | %s / %s | %s / %s | %s | %s / %s | %s, %s | %s | %s |" % (
            r["aircraft"], r["case"], r["ended"], r["survival_s"], r.get("cycles", "n/a"),
            _fmt(r.get("min_clear_surface"), "%.2f"), _fmt(r.get("min_clear_wingtip"), "%.2f"),
            _fmt(r.get("max_n_cmd"), "%.2f"), _fmt(r.get("max_n_flown"), "%.2f"),
            _fmt(r.get("n_limit_bind_frac"), "%.3f"), _fmt(ab.get("mu_rate"), "%.3f"),
            _fmt(ab.get("cl_rate"), "%.3f"), _fmt(es.get("wind_speed_err_rms"), "%.2f"),
            _fmt(es.get("wind_dir_err_rms_deg"), "%.1f"), _fmt(es.get("u10_est_last"), "%.2f"),
            _fmt(es.get("z0_est_last"), "%.1e")))
    return chr(10).join(lines)


def est_main(a, exp2=False):
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    ks = (1, 4, 5) if exp2 else range(len(EST_CASES))
    jobs = [(n, float(u), k, a.t_end, exp2) for n in a.aircraft.split(",")
            for u in a.u10.split(",") for k in ks]
    jobs.sort(key=lambda j: EST_CASES[j[2]][1] != "waves")           # slow ones first
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_est_job, jobs))
    labels = [c[0] for c in EST_CASES]
    rows.sort(key=lambda r: (r["aircraft"], r["u10"], labels.index(r["case"])))
    print(est_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    d["estimates_exp2" if exp2 else "estimates_exp"] = {
        "meta": {**record.meta("python -m sim.mpc --est-exp%s --u10 " % ("2" if exp2 else "") + a.u10 + " --aircraft "
                               + a.aircraft, [__file__, LIB_PATH,
                                              os.path.join(os.path.dirname(__file__), "aircraft.py"),
                                              os.path.join(os.path.dirname(__file__), "estimator.py")]),
                 "planner_kw": LIB_PKW, "runtime_s": runtime, "t_end": a.t_end, "seed": 1,
                 "profile_fit": "two_param", "estimator": "sample_dt=True",
                 "sea": "make_env('waves', u10): the sea of the library experiment; the planner "
                        "world is flat in every case",
                 "actuators": "tau, rate limits from record ctrl.* (defaults 0.1 s, 150 deg/s, 2 /s)",
                 "est_log_every_s": 5.0, "stop_on_tip": exp2,
                 "n_limit": "planner rollouts fly the limited cl, commanded cl held to the limit"
                            if exp2 else "flight model only"},
        "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))


MARGIN_CASES = (
    # (label, sea, planner world, state/wind source)
    ("a flat sea, truth", "flat", "flat", "truth"),
    ("b flat sea, estimated", "flat", "flat", "estimate"),
    ("c sea, flat world, truth", "waves", "flat", "truth"),
    ("d sea, flat world, estimated", "waves", "flat", "estimate"),
    ("e sea, wave-fit world, estimated", "waves", "estimated", "estimate"),
)


def run_margin_job(job):
    """job = (aircraft, u10, case index, library keyword, t_end). Actuators pb2v, n_limit,
    stop_on_tip, the sea of the earlier rows (make_env), one seed."""
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim.autonomy import make_env
    name, u10, k, lib, t_end = job
    label, sea, wm, src = MARGIN_CASES[k]
    ac = by_name(name)
    env = make_env(sea, u10)
    t0 = time.time()
    r = fly_mpc(ac, env, wm, _sensors("est-radar+fwd") if wm == "estimated" else [], t_end,
                env.wind.to_deg - 20.0, u10, seed=1, planner_kw=dict(LIB_PKW),
                library=CycleLibrary.from_keyword(lib), state_source=src, wind_source=src,
                suite="standard", profile_fit="two_param",
                actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                wave_aware=False)
    r["wall_s"] = time.time() - t0
    r = _no_nan(r)
    r.pop("est_log", None)
    return {"aircraft": name, "u10": u10, "case": label, "library": lib or "old", "sea": sea,
            "planner_world": wm, "sources": src, **r}


def margin_table(rows):
    lines = ["| aircraft | u10 | case | lib | end (s) | cycles | min surf / tip (m) | "
             "n cmd / flown | binds n / mu / cl | v toward (m/s) | dE/min last 60 s |",
             "|" + "---|" * 11]
    for r in rows:
        ab = r.get("actuator_bind") or {}
        lines.append("| %s | %g | %s | %s | %s %.0f | %s | %s / %s | %s / %s | %s / %s / %s | %s | %s |" % (
            r["aircraft"], r["u10"], r["case"], r["library"], r["ended"], r["survival_s"],
            r.get("cycles", "n/a"), _fmt(r.get("min_clear_surface"), "%.2f"),
            _fmt(r.get("min_clear_wingtip"), "%.2f"), _fmt(r.get("max_n_cmd"), "%.2f"),
            _fmt(r.get("max_n_flown"), "%.2f"), _fmt(r.get("n_limit_bind_frac"), "%.3f"),
            _fmt(ab.get("mu_rate"), "%.3f"), _fmt(ab.get("cl_rate"), "%.3f"),
            _fmt(r.get("v_toward_target"), "%.1f"), _fmt(r.get("energy_per_min_last60"), "%.0f")))
    return chr(10).join(lines)


def margin_main(a):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    u10s = [13.0, 16.0] if a.u10 == "10,13,16" else [float(v) for v in a.u10.split(",")]
    jobs = [(n, u, k, "margin", a.t_end) for n in a.aircraft.split(",") for u in u10s
            for k in range(len(MARGIN_CASES))]
    jobs += [(n, u, k, None, a.t_end) for n in a.aircraft.split(",") for u in u10s for k in (0, 2)]
    jobs.sort(key=lambda j: -(MARGIN_CASES[j[2]][2] == "estimated") * 2
              - (MARGIN_CASES[j[2]][1] == "waves"))
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_margin_job, jobs, chunksize=1))
    labels = [c[0] for c in MARGIN_CASES]
    rows.sort(key=lambda r: (r["library"] == "old", r["aircraft"], r["u10"],
                             labels.index(r["case"])))
    print(margin_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    d["margin_exp"] = {
        "meta": {**record.meta("python -m sim.mpc --margin-exp --t-end %g" % a.t_end,
                               [__file__, LIB_PATH, LIB_MARGIN_PATH,
                                os.path.join(os.path.dirname(__file__), "aircraft.py"),
                                os.path.join(os.path.dirname(__file__), "estimator.py")]),
                 "planner_kw": LIB_PKW, "runtime_s": runtime, "t_end": a.t_end, "seed": 1,
                 "profile_fit": "two_param", "suite": "standard", "wave_aware": False,
                 "sea": "make_env('waves', u10) for c, d, e; make_env('flat', u10) for a, b",
                 "actuators": "rate_model pb2v (record ctrl.*)",
                 "n_limit": True, "stop_on_tip": True,
                 "rows_old_library": "cases a and c only, library sim/out/mpc_library.json",
                 "case_e": "world_model estimated (local wave fit, radar + forward scanner), "
                           "wind from the estimator",
                 "energy_trend": "(E_end - E_start)/(t_end - t_start) * 60 over the last 60 s "
                                 "flown, E = g z + V^2/2"},
        "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(_no_nan(d), fh, indent=1)
        fh.write(chr(10))


def _fmt(v, f):
    return "n/a" if v is None else f % v


def table(rows):
    lines = ["| aircraft | u10 | model | ended | survival (s) | dE/min (J/kg) | min clear surf / tip (m)"
             " | max n | v toward (m/s) | cpu mean / max (s) |", "|" + "---|" * 10]
    for r in rows:
        lines.append("| %s | %g | %s | %s | %.0f | %s | %s / %s | %s | %s | %.2f / %.2f |" % (
            r["aircraft"], r["u10"], r["model"], r["ended"], r["survival_s"],
            _fmt(r.get("energy_per_min"), "%.0f"), _fmt(r.get("min_clear_surface"), "%.2f"),
            _fmt(r.get("min_clear_wingtip"), "%.2f"), _fmt(r.get("max_n"), "%.2f"),
            _fmt(r.get("v_toward_target"), "%.1f"), r.get("cpu_mean_s", 0.0),
            r.get("cpu_max_s", 0.0)))
    return "\n".join(lines)


def main():
    import argparse
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    ap = argparse.ArgumentParser()
    ap.add_argument("--t-end", type=float, default=180.0)
    ap.add_argument("--aircraft", default="baseline-2m,albatross-3m")
    ap.add_argument("--u10", default="10,13,16")
    ap.add_argument("--models", default=",".join(MODELS))
    ap.add_argument("--workers", type=int, default=max(1, min(8, os.cpu_count() or 1)))
    ap.add_argument("--n-samples", type=int, default=128)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--max-components", type=int, default=48)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "out", "mpc.json"))
    ap.add_argument("--build-library", action="store_true")
    ap.add_argument("--lib-exp", action="store_true")
    ap.add_argument("--est-exp", action="store_true")
    ap.add_argument("--est-exp2", action="store_true")
    ap.add_argument("--build-margin-library", action="store_true")
    ap.add_argument("--margin-exp", action="store_true")
    a = ap.parse_args()
    if a.build_margin_library:
        build_margin_library(workers=a.workers)
        return
    if a.margin_exp:
        margin_main(a)
        return
    if a.build_library:
        build_library(workers=a.workers)
        return
    if a.est_exp or a.est_exp2:
        est_main(a, exp2=a.est_exp2)
        return
    if a.lib_exp:
        if a.models == ",".join(MODELS):
            a.models = ",".join(LIB_MODELS)
        lib_main(a)
        return
    pkw = {"n_samples": a.n_samples, "dt": a.dt, "n_iter": 1}
    jobs = []
    for name in a.aircraft.split(","):
        for u10 in (float(v) for v in a.u10.split(",")):
            for m in list(a.models.split(",")) + ["tracker"]:
                jobs.append((name, u10, m, a.t_end, pkw, a.max_components))
    jobs.sort(key=lambda j: j[2] != "truth")            # slow ones first
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_job, jobs))
    order = {m: i for i, m in enumerate(list(MODELS) + ["tracker"])}
    rows.sort(key=lambda r: (r["aircraft"], r["u10"], order[r["model"]]))
    print(table(rows))
    print("\nestimator fit quality (residual rms / sample std; eta prediction rms over the plan, m)")
    for r in rows:
        if r["model"].startswith("est-"):
            print("%s u10 %g %s: q mean %s median %s, samples %.0f, fit available %.2f, "
                  "eta pred rms %s, fit cpu %.3f s" % (
                      r["aircraft"], r["u10"], r["model"], _fmt(r["fit_quality_mean"], "%.3f"),
                      _fmt(r["fit_quality_median"], "%.3f"), r["fit_samples_mean"],
                      r["fit_available_frac"], _fmt(r["eta_pred_rms"], "%.2f"),
                      r["fit_cpu_mean_s"]))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    cmd = "python -m sim.mpc"
    meta = record.meta(cmd, [__file__, os.path.join(os.path.dirname(__file__), "seasense.py")])
    meta.update({"args": vars(a), "planner_kw": pkw, "seed": 1, "runtime_s": runtime,
                 "env": "make_env('waves', u10): fully developed, seed 1, Charnock log wind, "
                        "wave-coherent perturbation on",
                 "target": "20 degrees off downwind, t_end %g s" % a.t_end})
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="\n") as fh:
        json.dump({"meta": meta, "results": rows}, fh, indent=1)
        fh.write("\n")


if __name__ == "__main__":
    main()
