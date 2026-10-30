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

Motor as a third control (MPPIPlanner motor=True, fly_mpc motor=True; choices the spec left open):
- Each knot carries the thrust T in [0, T_max] (T_max = prop.thrust_max_per_weight m g); the nominal
  thrust of a library cycle is 0. The perturbation of T is Gaussian with sigma
  ctrl.thrust_sigma_frac (0.15) T_max, independent per knot, clipped at 0 and T_max (so about half
  of the perturbed knots are 0 and the rest a half-normal burst).
- Rollouts and the closed loop fly with the thrust and with the folded-propeller drag
  cd0 + Delta CD0 (1 - T / (T + 0.01 m g)). The motor follows the command through a first-order lag
  prop.thrust_tau_s (0.1 s) in sim.aircraft.Actuators (no delay, no rate limit); without actuators
  the commanded thrust is applied as it is.
- The cost adds w_energy times the electric energy E = integral of T V / (eta_p eta_m) dt over the
  horizon (per step T at the step midpoint, V at the step start, counted while the rollout is
  alive), and keeps every other term. w_energy is in cost units per joule. The three named levels
  (W_ENERGY_LEVELS, energy_weight) are multiples of the break-even weight w_e eta / m, the weight at
  which a joule of electricity costs as much as the specific energy it can buy is worth.
- A burst is a run of the actual thrust above 1 % of the weight (the half point of the drag switch).

Motor library, sustained thrust sampling, state fold, reflex (choices the spec left open):
- LIB_MOTOR_PATH (build_motor_library) holds soaring cycles (max_speed, as the margin library, kind
  "soaring", zero thrust) and, for every (wind, chi) of the grid MOTOR_U10 that has no soaring cycle,
  a motor cycle of dsopt mode min_power (kind "motor", thrust per node, average electric power, speed
  along chi at least 0.5 v_best_ld) for the kept aircraft. Entries that leave their margins on a dense
  replay are dropped (filter_motor_library). Cycle.thrust / CycleGuide.nominal_thrust give the thrust
  schedule, phase-matched like the controls; the planner's nominal thrust is that schedule and the
  perturbation is around it.
- thrust_sampling "sustained" (default "basic" = the earlier draws): half of the samples hold one
  half-normal thrust level (scale HOLD_SIGMA_FRAC T_max) over a random run of knots, and 6 candidates
  with constant thrust (THRUST_CAND_LEVELS of the weight, with the nominal controls and wings level)
  join the constant-control candidates.
- fold_switch "state" (default "smooth"): in the rollouts and (fly_mpc) in the true dynamics the
  propeller is folded (full folded drag) when the commanded thrust is below FOLD_CMD_FRAC of the weight
  and not folded otherwise, instead of the smooth function of the thrust (see fold_effective).
- Reflex (reflex=True in fly_mpc): see the Reflex class.

Navigation in the closed loop (fly_mpc target / navigator / nav_hull; choices the spec left open):
- The tactical layer is sim.navigator.HullNavigator on the velocity hull of the library entries (the
  mean ground velocity of each cycle, not its course speed: see the notes in sim.navigator). It is called
  every value("nav.update_s", 5.0) s with the true position and wind and names the library cycle to fly
  (entry and side), not a course; fly_mpc builds the Cycle from it. A change of cycle drops the phase
  hint. The Polar based Navigator is still accepted (navigator=True with nav_polar) and picks the cycle
  by course.
- Arrival: the target within NAV_ARRIVAL_M (100 m), tested at every integration step.
- nav_exp (run_nav_job): flat COARE 3.5 sea, truth state and wind, motor on at the middle weight, the
  settings of the motor library experiment; the prediction is sail_hull on the same hull.
- Library: the upwind-side courses (extend_motor_library) and straight upwind motor cycles
  (extend_motor_upwind); calm entries are turned onto their course (align_calm_entries).

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
import math
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


FOLD_CMD_FRAC = 0.005       # fold_switch "state": a commanded thrust below this share of the weight = motor off


def fold_effective(ac, dcd, t_act, t_cmd):
    """Folded-propeller drag coefficient to hand to the smooth-switch formulas so that they give the
    state switch: dcd (propeller folded) where the commanded thrust is below FOLD_CMD_FRAC of the
    weight, 0 (running) elsewhere, whatever the actual thrust t_act (lagging the command) is."""
    t_act = np.asarray(t_act, float)
    f = 1.0 - t_act / (t_act + 0.01 * ac.mass * G)
    return np.where(np.asarray(t_cmd) < FOLD_CMD_FRAC * ac.mass * G, dcd / f, 0.0)


def derivatives_vec(t, s, ac, world, cl, mu, n_limit=False, thrust=0.0, ground_effect=False,
                    fold_dcd=None):
    """s: (6, S) states; cl, mu: (S,). Returns (6, S) derivatives. With n_limit cl is first
    reduced to the load-factor limit at each state's airspeed (as sim.aircraft.n_limited_cl).
    thrust (N, scalar or (S,)) along the air-relative velocity; ground_effect multiplies the
    induced drag by sim.aircraft.ground_effect_phi at the height above world.surface.
    fold_dcd (None: off): the folded-propeller drag of sim.aircraft.derivatives."""
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
    cd0 = ac.cd0
    if fold_dcd is not None:
        cd0 = cd0 + fold_dcd * (1.0 - thrust / (thrust + 0.01 * ac.mass * G))
    if ground_effect:
        eta, ex, ey = world.surface(x, y, np.full(np.shape(x), float(t)))
        D = q * (cd0 + ac.k_induced * ground_effect_phi(z - eta, ac.span, mu, p, ex, ey)
                 * cl * cl)
    else:
        D = q * (cd0 + ac.k_induced * cl * cl)
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

P_HOLD = 0.5               # thrust_sampling "sustained": share of the samples with a held thrust perturbation
HOLD_SIGMA_FRAC = 0.3      # scale of that perturbation, in T_max (half-normal level)
THRUST_CAND_LEVELS = (0.05, 0.10, 0.20)      # constant-thrust candidates, shares of the weight


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
    thrust_knots: np.ndarray = None   # (n_ctrl,) commanded thrust (N) with the motor, else None

    def controls(self, tau):
        """(cl, mu) at tau seconds from the plan time (piecewise linear, held after the end)."""
        return (float(np.interp(tau, self.knots_t, self.cl_knots)),
                float(np.interp(tau, self.knots_t, self.mu_knots)))

    def thrust(self, tau):
        """Commanded thrust (N) at tau seconds from the plan time (0 without the motor)."""
        if self.thrust_knots is None:
            return 0.0
        return float(np.interp(tau, self.knots_t, self.thrust_knots))


class MPPIPlanner:
    def __init__(self, aircraft, horizon_s=6.0, dt=0.05, n_samples=256, n_ctrl=12, lam=1.0,
                 sigma_cl=0.15, sigma_mu=0.25, replan_s=0.5, seed=1, n_iter=2, n_iter_cold=4,
                 w_e=1.0, w_p=0.5, h_clear=1.0, clearance="height", w_clear=200.0,
                 w_n=50.0, w_stall=300.0, w_smooth=2.0, c_contact=1e4, smooth_modes=0,
                 smooth_gain=3.0, w_d=10.0, n_limit=False, motor=False, w_energy=None,
                 thrust_sampling="basic", fold_switch="smooth", thrust_update="clipped"):
        self.ac = aircraft
        if thrust_update not in ("clipped", "raw"):
            raise ValueError(thrust_update)
        self.thrust_update = thrust_update
        self.horizon_s, self.dt, self.n_samples, self.n_ctrl = horizon_s, dt, n_samples, n_ctrl
        self.lam, self.sigma_cl, self.sigma_mu, self.replan_s = lam, sigma_cl, sigma_mu, replan_s
        self.n_iter, self.n_iter_cold = n_iter, n_iter_cold
        self.w_e, self.w_p, self.h_clear, self.clearance = w_e, w_p, h_clear, clearance
        self.w_clear, self.w_n, self.w_stall, self.w_smooth = w_clear, w_n, w_stall, w_smooth
        self.c_contact = c_contact
        self.smooth_modes, self.smooth_gain = smooth_modes, smooth_gain
        self.w_d = w_d
        self.motor = bool(motor)
        if thrust_sampling not in ("basic", "sustained") or fold_switch not in ("smooth", "state"):
            raise ValueError((thrust_sampling, fold_switch))
        self.thrust_sampling, self.fold_switch = thrust_sampling, fold_switch
        self.nu = 3 if self.motor else 2         # controls per knot: cl, mu (and the thrust T)
        if self.motor:
            from sim.dsopt import folded_dcd, prop_efficiency, thrust_max
            from sim.record import value
            self.t_max = thrust_max(aircraft)
            self.eta = prop_efficiency()
            self.w_energy = float(energy_weight(aircraft, "normal") if w_energy is None else w_energy)
            self.fold_dcd = folded_dcd(aircraft)
            self.sigma_t = float(value("ctrl.thrust_sigma_frac", 0.15)) * self.t_max
            self.sigma_hold = HOLD_SIGMA_FRAC * self.t_max
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
        return np.stack([np.full(self.n_ctrl, cl0)] + [np.zeros(self.n_ctrl)] * (self.nu - 1))

    def _shift(self, U):
        tau = np.minimum(self.knots_t + self.replan_s, self.horizon_s)
        return np.stack([np.interp(tau, self.knots_t, U[k]) for k in range(self.nu)])

    def _candidates(self):
        """Constant-control plans always added to the samples: best L/D and a firm pull-up, each
        wings level and banked either way. They keep a recovery in the set when the shifted plan
        is already on its way into the surface."""
        ac = self.ac
        out = []
        for cl in (ac.cl_best_ld, 0.8 * ac.cl_max):
            for mu in (0.0, -0.6, 0.6):
                u = [np.full(self.n_ctrl, cl), np.full(self.n_ctrl, mu)]
                if self.motor:
                    u.append(np.zeros(self.n_ctrl))
                out.append(np.stack(u))
        return np.array(out)

    def _thrust_candidates(self, U):
        """Constant-thrust candidates (thrust_sampling "sustained"): each of THRUST_CAND_LEVELS of the
        weight, held over the whole horizon, once with the controls of the nominal U (the cycle) and
        once wings level at the best L/D."""
        ac = self.ac
        out = []
        for lv in THRUST_CAND_LEVELS:
            th = np.full(self.n_ctrl, min(lv * ac.mass * G, self.t_max))
            out.append(np.stack([U[0], U[1], th]))
            out.append(np.stack([np.full(self.n_ctrl, ac.cl_best_ld), np.zeros(self.n_ctrl), th]))
        return np.array(out)

    def _thrust_noise(self, S):
        """Thrust perturbations (S, 1, n_ctrl): independent Gaussian per knot; with thrust_sampling
        "sustained" a share P_HOLD of the samples instead hold one half-normal level (scale
        HOLD_SIGMA_FRAC T_max) over a random run of knots."""
        n = self.n_ctrl
        eT = self.sigma_t * self.rng.standard_normal((S, 1, n))
        if self.thrust_sampling == "sustained":
            hold = self.rng.random(S) < P_HOLD
            lvl = np.abs(self.rng.standard_normal(S)) * self.sigma_hold
            a, b = self.rng.integers(0, n, S), self.rng.integers(0, n, S)
            lo, hi = np.minimum(a, b), np.maximum(a, b)
            k = np.arange(n)[None, :]
            win = (k >= lo[:, None]) & (k <= hi[:, None])
            eT = np.where(hold[:, None, None], (lvl[:, None] * win)[:, None, :], eT)
        return eT

    def _clip(self, U):
        ac = self.ac
        U[..., 0, :] = np.clip(U[..., 0, :], ac.cl_min, ac.cl_max)
        U[..., 1, :] = np.clip(U[..., 1, :], -ac.mu_max, ac.mu_max)
        if self.motor:
            U[..., 2, :] = np.clip(U[..., 2, :], 0.0, self.t_max)
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
        """Cost of each control plan in Us (S, nu, n_ctrl), nu 2 (cl, mu) or 3 with the motor
        (thrust T, N). Returns (costs (S,), paths (N+1,6,S), dead (S,) bool)."""
        ac, dt, N = self.ac, self.dt, self.nsteps
        S = Us.shape[0]
        i0, fr = self._idx, self._frac
        cl_all = Us[:, 0, i0] * (1 - fr) + Us[:, 0, i0 + 1] * fr      # (S, 2N+1)
        mu_all = Us[:, 1, i0] * (1 - fr) + Us[:, 1, i0 + 1] * fr
        cl_all = np.clip(cl_all, ac.cl_min, ac.cl_max)
        mu_all = np.clip(mu_all, -ac.mu_max, ac.mu_max)
        motor = self.motor
        if motor:
            T_all = np.clip(Us[:, 2, i0] * (1 - fr) + Us[:, 2, i0 + 1] * fr, 0.0, self.t_max)
            fd = self.fold_dcd
            state_fold = self.fold_switch == "state"
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

            if motor:
                x_T = np.full(S, st[4] if len(st) > 4 else 0.0)
                aT_lag = 1.0 - np.exp(-hd / act.thrust_tau) if act.thrust_tau > 0 else 1.0
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
            if motor:
                T1, T2, T3 = T_all[:, 2 * i], T_all[:, 2 * i + 1], T_all[:, 2 * i + 2]
                Tc1, Tc2, Tc3 = T1, T2, T3          # the commands (before the motor lag)
            if act is not None:
                lim_mu = act.mu_rate_at(s[3]) * hd
                lim_cl = act.cl_rate * hd
                c1, m1 = x_cl, x_mu
                x_cl = _act(x_cl, c2, lim_cl); x_mu = _act(x_mu, m2, lim_mu)
                c2, m2 = x_cl, x_mu
                x_cl = _act(x_cl, c3, lim_cl); x_mu = _act(x_mu, m3, lim_mu)
                c3, m3 = x_cl, x_mu
                if motor:
                    T1 = x_T
                    x_T = x_T + aT_lag * (T2 - x_T)
                    T2 = x_T
                    x_T = x_T + aT_lag * (T3 - x_T)
                    T3 = x_T
            if motor:
                V0 = s[3]
                if state_fold:
                    fd1, fd2, fd3 = (fold_effective(ac, self.fold_dcd, T1, Tc1),
                                     fold_effective(ac, self.fold_dcd, T2, Tc2),
                                     fold_effective(ac, self.fold_dcd, T3, Tc3))
                else:
                    fd1 = fd2 = fd3 = fd
                k1 = derivatives_vec(t, s, ac, world, c1, m1, self.n_limit, thrust=T1, fold_dcd=fd1)
                k2 = derivatives_vec(t + dt / 2, s + dt / 2 * k1, ac, world, c2, m2, self.n_limit,
                                     thrust=T2, fold_dcd=fd2)
                k3 = derivatives_vec(t + dt / 2, s + dt / 2 * k2, ac, world, c2, m2, self.n_limit,
                                     thrust=T2, fold_dcd=fd2)
                k4 = derivatives_vec(t + dt, s + dt * k3, ac, world, c3, m3, self.n_limit,
                                     thrust=T3, fold_dcd=fd3)
            else:
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
            if motor:
                cost += np.where(live, dt * self.w_energy * T2 * V0 / self.eta, 0.0)
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
        """The MPPI weighted update of the nominal U from the samples Us (S, nu, n_ctrl) with their
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
            if self.motor:    # the thrust schedule of the cycle (zero for a soaring cycle)
                U = np.concatenate([U, guide.nominal_thrust(self.knots_t)[None]])
                U[2] = np.clip(U[2], 0.0, self.t_max)
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
                if self.motor:   # thrust: see _thrust_noise; _clip makes U_T + eps non-negative
                    eps = np.concatenate([eps, self._thrust_noise(self.n_samples)], axis=1)
            eps[0] = 0.0
            Us = self._clip(U[None] + eps)
            Uraw = (U[None] + eps) if (self.motor and self.thrust_update == "raw") else None
            cand = self._candidates()
            if self.motor and self.thrust_sampling == "sustained":
                cand = np.concatenate([cand, self._thrust_candidates(U)])
            Us[1:1 + len(cand)] = cand
            if guide is not None and prev is not None:
                Us[1 + len(cand)] = self._clip(prev.copy())
            cost, _, dead = self.rollout(world, t, s0, Us, target_dir)
            if Uraw is not None:
                # thrust_update "raw": the thrust row of the update uses the perturbed values before the
                # clip at zero. The clipped values are all >= 0, so their weighted mean is above the
                # nominal even when no sample is better (a thrust that is never asked for).
                Uu = Us.copy()
                keep = np.ones(Us.shape[0], bool)
                keep[1:1 + len(cand)] = False
                if guide is not None and prev is not None:
                    keep[1 + len(cand)] = False
                Uu[keep, 2, :] = Uraw[keep, 2, :]
                U, ess, n_dead = self.update(U, Uu, cost, dead)
            else:
                U, ess, n_dead = self.update(U, Us, cost, dead)
        self.U = U
        cost, path, dead = self.rollout(world, t, s0, U[None], target_dir)
        nfirst = int(round(self.replan_s / self.dt))
        times = np.arange(self.nsteps + 1) * self.dt
        cl_k, mu_k = U[0].copy(), U[1].copy()
        return Plan(self.knots_t.copy(), cl_k, mu_k, times[:nfirst + 1],
                    np.interp(times[:nfirst + 1], self.knots_t, cl_k),
                    np.interp(times[:nfirst + 1], self.knots_t, mu_k),
                    path[:, :, 0].copy(), times, float(cost[0]), ess, n_dead,
                    U[2].copy() if self.motor else None)


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


LIB_MOTOR_PATH = os.path.join(os.path.dirname(__file__), "out", "mpc_library_motor.json")
KEPT_AIRCRAFT = ("albatross-3m", "baseline-2m", "sealedhull-2p5m", "easyglider-1p8m", "demo-1p5m")
MOTOR_U10 = (0.0, 3.0, 6.0, 9.0, 11.0, 13.0, 16.0, 20.0, 24.0)
MOTOR_SEEDS = ((0, 1, 2), (3, 4, 5))      # tried in this order for a motor cycle
MOTOR_MAXITER = 400


def _solve_motor_entry(job):
    try:
        return _solve_motor_entry_raw(job)
    except Exception as exc:
        name, u10, chi_deg, n = job
        return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
                "chi_deg": float(chi_deg), "success": False, "missing": True, "kind": "motor",
                "error": repr(exc)}


def _solve_motor_entry_raw(job):
    """One motor-assisted library cycle: dsopt mode min_power with the motor (thrust a third
    control per node, mean speed along chi at least 0.5 v_best_ld as in the ground-motor study),
    flat mode, COARE 3.5 roughness at max(u10, 1), geometric wingtip, midpoints, pb2v rates, n >=
    0, no ground effect, the margin sets of the margin library (standard first, then light). The
    entry has the node thrust (N), the average electric power and the replay figures of
    dsopt.control_stats and dsopt.motor_stats."""
    from sim.aircraft_models import by_name
    from sim.dsopt import control_stats, motor_stats, profile_for, solve
    name, u10, chi_deg, n = job[:4]
    h_over = job[4] if len(job) > 4 else None      # optional: a larger minimum height than the margin set's
    ac = by_name(name)
    prof = profile_for(max(u10, 1.0), "log", roughness="coare35")
    tried = []
    for label, (h_min, n_frac, mu_frac, v_factor) in MARGIN_SETS:
        h_min = h_min if h_over is None else max(h_min, h_over)
        for seeds in MOTOR_SEEDS:
            t0 = time.time()
            info, x, col = solve(ac, prof, mode="min_power", cycle="travel", n=n, u10=u10,
                                 chi=np.radians(chi_deg), tip_model="geometric", midpoints=True,
                                 rate_limits=True, rate_model="pb2v", n_min=0.0, h_min=h_min,
                                 n_frac=n_frac, mu_frac=mu_frac, v_factor=v_factor, motor=True,
                                 v_travel_min=0.5 * ac.v_best_ld, seeds=seeds,
                                 maxiter=MOTOR_MAXITER, return_x=True, stop_after=1)
            tried.append({"margins": label, "seeds": list(seeds), "success": bool(info["success"]),
                          "violation": float(info["violation"]), "runtime_s": time.time() - t0})
            if info["success"]:
                S, C, T, _ = col.unpack(x)
                t = np.linspace(0.0, T, n)
                r = lambda a: [float(f"{v:.6g}") for v in np.asarray(a)]
                ms = motor_stats(col, x)
                return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
                        "chi_deg": float(chi_deg), "success": True, "kind": "motor",
                        "period": float(T), "margin_set": label,
                        "margins": {"h_min": h_min, "n_frac": n_frac, "mu_frac": mu_frac,
                                    "v_factor": v_factor, "n_min": 0.0, "h_tip_min": col.h_tip,
                                    "k_roll": col.k_roll, "cl_rate_max_per_s": col.r_max},
                        "speed_along_chi": float(info["speed_along_chi"]),
                        "z_min": float(info["z_min"]), "z_min_mid": float(info["z_min_mid"]),
                        "n_max": float(info["n_max"]), "power_avg_w": ms["power_avg_w"],
                        "power_avg_dense_w": ms["power_avg_dense_w"],
                        "power_peak_w": ms["power_peak_w"],
                        "thrust_on_share": ms["thrust_on_share"],
                        "thrust_max_n": ms["thrust_max_n"],
                        "replay": control_stats(col, x), "tried": tried,
                        "t": r(t), "x": r(S[:, 0]), "y": r(S[:, 1]), "z": r(S[:, 2]),
                        "V": r(S[:, 3]), "gamma": r(S[:, 4]), "psi": r(S[:, 5]),
                        "cl": r(C[:, 0]), "mu": r(C[:, 1]), "thrust": r(col.thrust(x))}
    return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
            "chi_deg": float(chi_deg), "success": False, "missing": True, "kind": "motor",
            "tried": tried}


def _solve_soaring_entry(job):
    """A soaring cycle of the motor library: the margin-library solve (mode max_speed), with the
    kind and a zero thrust schedule added."""
    name, u10, chi_deg, n = job
    if u10 < 1.0:      # no log profile at a calm: no soaring cycle
        return {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
                "chi_deg": float(chi_deg), "success": False, "missing": True, "kind": "soaring",
                "tried": []}
    try:
        e = _solve_margin_entry(job)
    except Exception as exc:       # a failed solve is a missing entry, not a lost build
        e = {"aircraft": name, "u10": float(u10), "u10_requested": float(u10),
             "chi_deg": float(chi_deg), "success": False, "missing": True, "error": repr(exc)}
    e["kind"] = "soaring"
    if e["success"]:
        e["thrust"] = [0.0] * len(e["t"])
        e["power_avg_w"] = 0.0
    return e


def build_motor_library(names=KEPT_AIRCRAFT, u10s=MOTOR_U10, chis=LIB_CHI_DEG, n=40, workers=6,
                        path=LIB_MOTOR_PATH):
    """The margin library extended with motor-assisted cycles and to the kept aircraft.

    Soaring cycles (max_speed, travel) first: chi = 20 deg at every wind of the grid; at the winds
    where that converged the other chi values; winds where chi = 20 failed are below the soaring
    minimum of the aircraft and are not tried for the other chi. Then a motor cycle (min_power,
    travel, motor) for every (aircraft, wind, chi) of the grid that has no soaring cycle, so every
    wind of the grid has all the chi values of one kind. Winds with no cycle of either kind stay
    missing (the list is in the meta block)."""
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    t0 = time.time()
    chi20 = min(chis, key=lambda c: abs(c - 20.0))
    with ProcessPoolExecutor(workers) as ex:
        first = list(ex.map(_solve_soaring_entry, [(a, u, chi20, n) for a in names for u in u10s],
                            chunksize=1))
        ok20 = {(e["aircraft"], e["u10"]) for e in first if e["success"]}
        rest = [(a, u, c, n) for a in names for u in u10s if (a, u) in ok20 for c in chis
                if c != chi20]
        soaring = first + list(ex.map(_solve_soaring_entry, rest, chunksize=1))
        have = {(e["aircraft"], e["u10"], e["chi_deg"]) for e in soaring if e["success"]}
        mjobs = [(a, u, c, n) for a in names for u in u10s for c in chis if (a, u, c) not in have]
        mjobs.sort(key=lambda j: (j[1], j[0]))
        motor = list(ex.map(_solve_motor_entry, mjobs, chunksize=1))
    entries = [e for e in soaring if e["success"]] + motor
    entries.sort(key=lambda e: (e["aircraft"], e["u10"], e["chi_deg"]))
    meta = record.meta("python -m sim.mpc --build-motor-library",
                       [__file__, os.path.join(os.path.dirname(__file__), "dsopt.py")])
    meta.update({"runtime_s": time.time() - t0, "n_nodes": n, "u10_grid": list(u10s),
                 "chi_deg": list(chis),
                 "dsopt_soaring": "as the margin library: solve(mode=max_speed, cycle=travel, flat "
                                  "mode, tip_model=geometric, midpoints=True, rate_limits=True, "
                                  "rate_model=pb2v, n_min=0, profile_for(u10, log, "
                                  "roughness=coare35)), chi 20 deg first, the other chi only where "
                                  "that converged",
                 "dsopt_motor": "solve(mode=min_power, cycle=travel, motor=True, v_travel_min = "
                                "0.5 v_best_ld, same flat mode settings, profile_for(max(u10, 1)), "
                                "ground effect off, seeds 0-2 then 3-5, maxiter %d) for every "
                                "(aircraft, wind, chi) without a soaring cycle" % MOTOR_MAXITER,
                 "margin_sets": {k: dict(zip(("h_min", "n_frac", "mu_frac", "v_factor"), v))
                                 for k, v in MARGIN_SETS},
                 "margin_order": "standard first; light only where standard did not converge",
                 "kept_aircraft": list(names),
                 "frame": "wind toward +x, angles ccw from +x; z is the height above the surface; "
                          "thrust in N per node (0 for soaring cycles)",
                 "missing": [{"aircraft": e["aircraft"], "u10": e["u10"], "chi_deg": e["chi_deg"]}
                             for e in entries if not e["success"]],
                 "counts": {a: {"soaring": sum(1 for e in entries if e["aircraft"] == a
                                               and e["success"] and e["kind"] == "soaring"),
                                "motor": sum(1 for e in entries if e["aircraft"] == a
                                             and e["success"] and e["kind"] == "motor")}
                            for a in names}})
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as fh:
        json.dump({"meta": meta, "entries": entries}, fh, indent=1)
        fh.write("\n")
    filter_motor_library(path)
    return entries


# ---- upwind-side courses and the calm-entry course fix ------------------------------------------
NAV_CHI_DEG = (120.0, 135.0, 150.0)       # upwind-side courses added for the navigator (0 downwind, 180 upwind)
NAV_U10 = (9.0, 11.0, 13.0, 16.0, 20.0, 24.0)    # winds of the extension (3 and 6 m/s motor cycles cut, see meta)
SOARING_MIN_PROGRESS = 1.0                # m/s along the course; a soaring cycle that makes less hands over to a motor cycle
CALM_COURSE_TOL_DEG = 5.0


def track_angle_deg(e):
    """Direction (deg ccw from +x, the library frame) of the mean ground track of an entry: its net
    displacement over one period."""
    return float(np.degrees(np.arctan2(e["y"][-1] - e["y"][0], e["x"][-1] - e["x"][0])))


def course_error_deg(e):
    """Signed difference (deg, in (-180, 180]) of the entry's mean ground track from its course chi."""
    return float((track_angle_deg(e) - e["chi_deg"] + 180.0) % 360.0 - 180.0)


def calm_course_errors(entries, tol=CALM_COURSE_TOL_DEG):
    """[(aircraft, chi, error deg)] of the successful calm (u10 = 0) entries whose mean track is more
    than tol deg off their course. Empty after the build's fix."""
    return [(e["aircraft"], e["chi_deg"], course_error_deg(e)) for e in entries
            if e.get("success") and e["u10"] == 0.0 and abs(course_error_deg(e)) > tol]


def rotate_calm_entry(e, chi_deg):
    """Copy of the calm entry e flown along the course chi_deg. With no wind the dynamics are
    invariant under a rotation about the vertical, so the cycle turned by (chi - mean track) is the
    same cycle (same power, margins, replay) with its mean track exactly along chi; the optimizer
    only bounded the speed along chi from below, so its track could lie anywhere (the cause of a calm
    entry 49 deg off its course). speed_along_chi becomes the whole ground speed."""
    rot = np.radians(chi_deg - track_angle_deg(e))
    c, s = np.cos(rot), np.sin(rot)
    x, y = np.asarray(e["x"], float), np.asarray(e["y"], float)
    r = dict(e)
    r["x"] = [float(f"{v:.6g}") for v in c * x - s * y]
    r["y"] = [float(f"{v:.6g}") for v in s * x + c * y]
    r["psi"] = [float(f"{v:.6g}") for v in np.asarray(e["psi"], float) + rot]
    r["chi_deg"] = float(chi_deg)
    r["speed_along_chi"] = float(np.hypot(r["x"][-1] - r["x"][0], r["y"][-1] - r["y"][0]) / e["period"])
    r["calm_rotated_deg"] = float(np.degrees(rot))
    return r


def align_calm_entries(entries, chis):
    """Entries with every calm entry turned onto its course; calm (aircraft, chi) of chis that have
    no entry (new courses, or a failed solve) are filled with a turned copy of the aircraft's
    lowest-power calm cycle (marked calm_derived_from_chi)."""
    out, have = [], {}
    for e in entries:
        if e["u10"] == 0.0 and e.get("success"):
            e = rotate_calm_entry(e, e["chi_deg"])
            have.setdefault(e["aircraft"], {})[e["chi_deg"]] = e
        out.append(e)
    ids = {(e["aircraft"], e["u10"], e["chi_deg"]) for e in out if e.get("success")}
    for a, d in have.items():
        src = min(d.values(), key=lambda e: e["power_avg_w"])
        for chi in chis:
            if (a, 0.0, chi) in ids:
                continue
            new = rotate_calm_entry(src, chi)
            new["calm_derived_from_chi"] = src["chi_deg"]
            new["tried"] = []
            out = [x for x in out if (x["aircraft"], x["u10"], x["chi_deg"]) != (a, 0.0, chi)] + [new]
    return out


def extend_motor_library(chis=NAV_CHI_DEG, u10s=NAV_U10, workers=6, path=LIB_MOTOR_PATH, n=40):
    """Add the upwind-side courses chis to the motor library file (the existing entries are kept).

    At every (aircraft, wind) that already has soaring cycles: a soaring cycle per course (margin
    library solve); where that does not converge or makes less than SOARING_MIN_PROGRESS along the
    course, and at the winds without soaring cycles, a motor cycle (min_power) as in the build.
    Calm entries are not solved: they are turned copies (align_calm_entries), which also puts the
    existing calm entries onto their courses. Dense-replay filter at the end."""
    from concurrent.futures import ProcessPoolExecutor

    t0 = time.time()
    with open(path) as fh:
        d = json.load(fh)
    names = d["meta"]["kept_aircraft"]
    soar_winds = {(e["aircraft"], e["u10"]) for e in d["entries"]
                  if e["success"] and e["kind"] == "soaring"}
    chis = tuple(float(c) for c in chis)
    keep = [e for e in d["entries"] if e["chi_deg"] not in chis or e["u10"] == 0.0]
    sjobs = [(a, u, c, n) for a in names for u in u10s if (a, u) in soar_winds for c in chis]
    with ProcessPoolExecutor(workers) as ex:
        soaring = list(ex.map(_solve_soaring_entry, sorted(sjobs, key=lambda j: j[1]), chunksize=1))
        ok = {(e["aircraft"], e["u10"], e["chi_deg"]) for e in soaring
              if e["success"] and e["speed_along_chi"] >= SOARING_MIN_PROGRESS}
        mjobs = [(a, u, c, n) for a in names for u in u10s for c in chis if (a, u, c) not in ok]
        mjobs.sort(key=lambda j: (j[1] > 11.0, j[1] > 13.0, j[1]))
        motor = list(ex.map(_solve_motor_entry, mjobs, chunksize=1))
    new = [e for e in soaring if (e["aircraft"], e["u10"], e["chi_deg"]) in ok] + motor
    entries = align_calm_entries(keep + new, tuple(d["meta"]["chi_deg"]) + chis)
    entries.sort(key=lambda e: (e["aircraft"], e["u10"], e["chi_deg"]))
    d["entries"] = entries
    m = d["meta"]
    m["chi_deg"] = sorted(set(m["chi_deg"]) | set(chis))
    m["extension"] = {
        "tdr_run": "python -m sim.mpc --extend-motor-library", "chi_deg": list(chis),
        "u10_solved": list(u10s), "runtime_s": time.time() - t0, "workers": workers,
        "soaring_jobs": len(sjobs), "motor_jobs": len(mjobs),
        "soaring_min_progress_m_s": SOARING_MIN_PROGRESS,
        "cut": "motor cycles at 3 and 6 m/s were not solved for the new courses (the slowest solves, "
               "about 170 to 300 s each) and no new course was solved at the calm; the planner then "
               "takes the new course from the nearest wind that has it (CycleLibrary.pick)",
        "calm": "u10 = 0 entries are not solved: each is the cycle turned about the vertical onto its "
                "course (rotate_calm_entry); calm entries missing at the old courses and all the new "
                "courses are turned copies of the aircraft's lowest-power calm cycle",
        "soaring_to_motor": "a soaring cycle that did not converge or made less than "
                            "soaring_min_progress_m_s along its course is replaced by a motor cycle"}
    with open(path, "w", newline="\n") as fh:
        json.dump(d, fh, indent=1)
        fh.write("\n")
    filter_motor_library(path)
    return entries


NAV_UPWIND_H_MIN = 10.0     # m; the steady upwind motor cycle flies this high: at the 2 m margin the planner cannot hold it
NAV_UPWIND_MOTOR = tuple((a, u) for a in KEPT_AIRCRAFT for u in (3.0, 6.0, 9.0, 11.0, 13.0, 16.0, 20.0))


def extend_motor_upwind(pairs=NAV_UPWIND_MOTOR, chi=180.0, workers=6, path=LIB_MOTOR_PATH, n=40):
    """Add motor cycles straight upwind (chi = 180 deg) for the (aircraft, wind) pairs: the soaring
    cycles never have an upwind velocity component, so without these the velocity hull has no way
    upwind at the soaring winds. The same motor solve as the build (min_power, speed along chi at
    least 0.5 v_best_ld; with chi = 180 that is a mean ground speed straight into the wind) with the minimum
    height raised to NAV_UPWIND_H_MIN: the cycle is a steady straight flight at that height, which the
    receding-horizon planner holds at 10 m (about 90 % of its ground speed) but not at the 2 m margin
    (it climbs away, 15 to 20 % of it). The calm
    entries are filled for chi by align_calm_entries. Dense-replay filter at the end."""
    from concurrent.futures import ProcessPoolExecutor

    t0 = time.time()
    with open(path) as fh:
        d = json.load(fh)
    jobs = [(a, u, float(chi), n, NAV_UPWIND_H_MIN) for a, u in pairs]
    with ProcessPoolExecutor(workers) as ex:
        new = list(ex.map(_solve_motor_entry, jobs, chunksize=1))
    ids = {(e["aircraft"], e["u10"], e["chi_deg"]) for e in new}
    keep = [e for e in d["entries"] if (e["aircraft"], e["u10"], e["chi_deg"]) not in ids]
    entries = align_calm_entries(keep + new, tuple(d["meta"]["chi_deg"]) + (float(chi),))
    entries.sort(key=lambda e: (e["aircraft"], e["u10"], e["chi_deg"]))
    d["entries"] = entries
    d["meta"]["chi_deg"] = sorted(set(d["meta"]["chi_deg"]) | {float(chi)})
    d["meta"]["upwind_motor"] = {
        "tdr_run": "python -m sim.mpc --extend-motor-upwind", "chi_deg": float(chi),
        "pairs": [list(p) for p in pairs], "runtime_s": time.time() - t0, "workers": workers,
        "h_min_m": NAV_UPWIND_H_MIN,
        "why": "soaring cycles never fly upwind (every mean ground velocity has a downwind "
               "component), so the velocity hull needs motor cycles that do",
        "missing": [[e["aircraft"], e["u10"]] for e in new if not e["success"]]}
    with open(path, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))
    filter_motor_library(path)
    return entries


def entry_replay(e, ac, sub=20):
    """Dense replay of a library entry (RK4 from each node with the piecewise-linear controls and
    thrust, dsopt dynamics): (lowest height, lowest geometric wingtip height, node states (N, 6),
    node controls (N, 2), node time step)."""
    from sim.dsopt import dyn, profile_for
    a = lambda k: np.asarray(e[k], float)
    S = np.stack([a("x"), a("y"), a("z"), a("V"), a("gamma"), a("psi")], axis=1)
    C = np.stack([a("cl"), a("mu")], axis=1)
    Th = a("thrust") if "thrust" in e else np.zeros(len(S))
    prof = profile_for(max(e["u10"], 1.0), "log", roughness="coare35")
    dt = e["period"] / (len(S) - 1)
    f = lambda s, c, th: dyn(s, c, ac, prof, e["u10"], False, th)
    s = S[:-1].copy()
    hmin, tipmin = np.inf, np.inf
    for j in range(sub + 1):
        fr = j / sub
        c = C[:-1] + (C[1:] - C[:-1]) * fr
        hmin = min(hmin, s[:, 2].min())
        tipmin = min(tipmin, (s[:, 2] - 0.5 * ac.span * np.sin(np.abs(c[:, 1])) * np.cos(s[:, 4])).min())
        if j == sub:
            break
        h = dt / sub
        cm = C[:-1] + (C[1:] - C[:-1]) * (fr + 0.5 / sub)
        cn = C[:-1] + (C[1:] - C[:-1]) * (fr + 1.0 / sub)
        t0, tm, tn = (Th[:-1] + (Th[1:] - Th[:-1]) * q for q in (fr, fr + 0.5 / sub, fr + 1.0 / sub))
        k1 = f(s, c, t0)
        k2 = f(s + 0.5 * h * k1, cm, tm)
        k3 = f(s + 0.5 * h * k2, cm, tm)
        k4 = f(s + h * k3, cn, tn)
        s = s + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return hmin, tipmin, S, C, dt


def replay_failures(e, ac):
    """Reasons (list of str, empty if none) why the dense replay of entry e leaves its margins: lowest
    height and wingtip 0.05 m under h_min and h_tip_min, load factor 1 % over n_frac n_max or 0.02
    under n_min, bank 1 % over mu_frac mu_max, airspeed 1 % under v_factor v_stall, rates 2 % over."""
    m = e["margins"]
    hmin, tipmin, S, C, dt = entry_replay(e, ac)
    n = 0.5 * ac.rho * ac.wing_area / (ac.mass * G) * S[:, 3] ** 2 * C[:, 0]
    Vm = np.minimum(S[1:, 3], S[:-1, 3])
    lim = 2.0 * m["k_roll"] * Vm / ac.span
    bad = []
    for ok, why in ((hmin >= m["h_min"] - 0.05, "h %.3f < %g" % (hmin, m["h_min"])),
                    (tipmin >= m["h_tip_min"] - 0.05, "tip %.3f" % tipmin),
                    (n.max() <= m["n_frac"] * ac.n_max * 1.01, "n max %.2f" % n.max()),
                    (n.min() >= m["n_min"] - 0.02, "n min %.2f" % n.min()),
                    (np.abs(C[:, 1]).max() <= m["mu_frac"] * ac.mu_max * 1.01, "bank"),
                    (S[:, 3].min() >= m["v_factor"] * ac.v_stall * 0.99, "airspeed"),
                    ((np.abs(np.diff(C[:, 1])) / dt / lim).max() <= 1.02, "bank rate"),
                    (np.abs(np.diff(C[:, 0])).max() / dt <= m["cl_rate_max_per_s"] * 1.02, "cl rate")):
        if not ok:
            bad.append(why)
    return bad


def filter_motor_library(path=LIB_MOTOR_PATH):
    """Dense-replay every entry of the motor library file and mark those that leave their margins as
    missing (success False, replay_failed with the reasons); rewrites the file. Returns the list."""
    from sim.aircraft_models import by_name
    with open(path) as fh:
        d = json.load(fh)
    dropped = []
    for e in d["entries"]:
        if not e["success"]:
            continue
        bad = replay_failures(e, by_name(e["aircraft"]))
        if bad:
            for k in ("t", "x", "y", "z", "V", "gamma", "psi", "cl", "mu", "thrust"):
                e.pop(k, None)
            e.update({"success": False, "missing": True, "replay_failed": bad})
            dropped.append((e["aircraft"], e["u10"], e["chi_deg"], bad))
    d["meta"]["replay_filter"] = ("every entry replayed densely (mpc.replay_failures) against its own "
                                 "margins; failures marked missing: %d" % len(dropped))
    d["meta"]["missing"] = [{"aircraft": e["aircraft"], "u10": e["u10"], "chi_deg": e["chi_deg"]}
                            for e in d["entries"] if not e["success"]]
    d["meta"]["counts"] = {a: {k: sum(1 for e in d["entries"] if e["aircraft"] == a and e["success"]
                                      and e["kind"] == k) for k in ("soaring", "motor")}
                           for a in d["meta"]["kept_aircraft"]}
    with open(path, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))
    return dropped


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
        # node thrust (N) of a motor-assisted cycle; zeros for a soaring one or an old library entry
        self.thrust = a("thrust") if "thrust" in d else np.zeros_like(self.t)
        self.has_thrust = bool(np.any(self.thrust > 0.0))
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

    def thrust_at(self, tau):
        """Nominal thrust (N) at phase tau (zero for a soaring cycle)."""
        return self._p(tau, self.thrust)

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
        """"margin" picks LIB_MARGIN_PATH, "motor" LIB_MOTOR_PATH, None the default file, else a
        file path."""
        if library_path is None:
            return cls.load()
        return cls.load({"margin": LIB_MARGIN_PATH, "motor": LIB_MOTOR_PATH}.get(library_path,
                                                                                    library_path))

    def pick(self, name, u10_est, chi_rad, chi_tol=20.0):
        """Cycle for the aircraft nearest to u10_est and |chi| (mirrored for negative chi).

        The wind is the nearest one that has a course within chi_tol deg of the one asked (the one
        with the smallest gap when none has), so a wind without the upwind-side courses hands over
        to the next wind that has them."""
        es = [e for e in self.entries if e["aircraft"] == name]
        if not es:
            raise KeyError(name)
        chi = abs(np.degrees(chi_rad))
        winds = sorted({e["u10"] for e in es}, key=lambda v: abs(v - u10_est))
        gap = {u: min(abs(e["chi_deg"] - chi) for e in es if e["u10"] == u) for u in winds}
        u = next((u for u in winds if gap[u] <= chi_tol),
                 min(winds, key=lambda u: (gap[u], abs(u - u10_est))))
        e = min((e for e in es if e["u10"] == u), key=lambda e: abs(e["chi_deg"] - chi))
        return Cycle(e, mirror=chi_rad < 0.0)


def polar_from_library(lib, name):
    """Polar (sim.navigator.Polar) of one aircraft from the library entries: the speed along the
    course of each entry (speed_along_chi, soaring or motor-assisted) at its wind and course; NaN where
    the library has no entry or the speed is at most sim.navigator.MIN_PROGRESS. Winds with fewer than
    two courses are left out; the course axis is the courses the library has."""
    from sim.navigator import MIN_PROGRESS, Polar
    es = [e for e in lib.entries if e["aircraft"] == name]
    chis = sorted({e["chi_deg"] for e in es})
    winds = sorted({e["u10"] for e in es if sum(1 for x in es if x["u10"] == e["u10"]) >= 2})
    tab = np.full((len(winds), len(chis)), np.nan)
    for e in es:
        if e["u10"] in winds and e["speed_along_chi"] > MIN_PROGRESS:
            tab[winds.index(e["u10"]), chis.index(e["chi_deg"])] = e["speed_along_chi"]
    return Polar(winds, chis, tab)


def nav_polar(lib, name, u10):
    """(polar, source) for the closed-loop navigator: the optimizer's speed polar
    (sim/out/ds_polar.json) where it has a progress speed at this wind for the aircraft, else the
    polar of the library entries (polar_from_library)."""
    from sim.navigator import polar_from_ds
    ds = polar_from_ds(name)
    if ds is not None and np.any(np.isfinite(ds.speed(np.arange(0.0, 181.0, 5.0), u10))):
        return ds, "ds_polar"
    return polar_from_library(lib, name), "library"


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
                 d_max=0.15, theta_star=np.pi / 3, tau_hint=None, hint_window=0.2, sig=None):
        self.cycle, self.psi_w, self.H, self.w_d = cycle, psi_w, horizon, w_d
        sg = self.SIG if sig is None else sig      # sig: (h m, V m/s or None, gamma, heading rad) of the match
        self.sig = (sg[0], sg[1] or 0.25 * cycle.v_mean, sg[2], sg[3])
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

    def nominal_thrust(self, knots_t):
        """Thrust (N) of the cycle from the matched phase on, phase-matched like the controls."""
        return self.cycle.thrust_at(self.tau_s + np.asarray(knots_t))

    def terminal_cost(self, world, t_end, s, planner):
        x, y, z, V, g, psi = s
        h = z - np.asarray(world.surface(x, y, np.full(np.shape(x), float(t_end)))[0])
        ht, Vt, gt, pt = self.target
        d2 = (((h - ht) / self.sig[0]) ** 2 + ((V - Vt) / self.sig[1]) ** 2
              + ((g - gt) / self.sig[2]) ** 2 + (wrap(psi - self.psi_w - pt) / self.sig[3]) ** 2)
        deficit = np.maximum(0.0, self.dE_c - (G * h + 0.5 * V * V - self.E0))
        return self.w_d * d2 + planner.w_e * deficit


# ---------------------------------------------------------------------------------------------
# reflex layer (a supervisor over the planner's commands, every control step)
# ---------------------------------------------------------------------------------------------

REFLEX_DT = 0.1        # step of the reflex prediction (midpoint rule); the supervisor itself runs every control step


class Reflex:
    """Predicts the next ctrl.reflex_horizon_s (1.5 s) of flight with the controls the planner has
    commanded and reports the first time the centre or a wingtip is predicted below
    ctrl.reflex_clearance_m (0.3 m) above the surface of the planner's world, or the airspeed below
    ctrl.reflex_v_frac (1.05) times the stall speed.

    Choices the spec left open (simplest thing taken):
    - "the current commanded controls" are the plan's own schedule from now on (cl, mu and thrust
      interpolated in the plan's knots, held after its end), not the present values held constant: a
      held pull-up of the cycle's low point would predict a stall at every cycle.
    - The prediction flies the point mass without the actuator lags, from the state given, with the
      midpoint rule (REFLEX_DT 0.1 s), with the load-factor limit and the folded-propeller drag of
      the planner.
    - The state tested includes the present one (step 0). Centre, then tip, then stall win a tie.
    - The override, while the prediction fails: full thrust (t_max), the lift coefficient at its
      margin limit (the lower of cl_max / v_factor^2 and the load factor n_frac n_max at the present
      airspeed, with the standard margin set), bank 0 (the actuators move to them at their rates).
      Control returns to the planner at the first step the prediction of the plan clears.
    """

    def __init__(self, aircraft, t_max, fold_dcd, n_limit=True, fold_switch="smooth"):
        from sim.record import value
        self.ac, self.t_max, self.fold_dcd, self.n_limit = aircraft, float(t_max), fold_dcd, n_limit
        self.fold_switch = fold_switch
        self.horizon_s = float(value("ctrl.reflex_horizon_s", 1.5))
        self.clearance_m = float(value("ctrl.reflex_clearance_m", 0.3))
        self.v_frac = float(value("ctrl.reflex_v_frac", 1.05))
        self.dt = REFLEX_DT
        self.n = int(round(self.horizon_s / self.dt))
        _, self.n_frac, _, self.v_factor = MARGIN_SETS[0][1]

    def predict(self, world, t, s, knots_t, cl_k, mu_k, th_k, tau):
        """First violation of the prediction from state s at time t, the plan's knots (arrays) being
        read from tau seconds after the plan time: (cause, seconds ahead, height of the centre, tip
        height, airspeed) with cause "centre", "tip" or "stall", or None when it clears."""
        ac, dt, n = self.ac, self.dt, self.n
        taus = tau + np.arange(2 * n + 1) * dt / 2.0
        cl = np.clip(np.interp(taus, knots_t, cl_k), ac.cl_min, ac.cl_max)
        mu = np.clip(np.interp(taus, knots_t, mu_k), -ac.mu_max, ac.mu_max)
        T = (np.clip(np.interp(taus, knots_t, th_k), 0.0, self.t_max) if th_k is not None
             else np.zeros_like(taus))
        st = np.asarray(s, float)[:, None]
        v_lim = self.v_frac * ac.v_stall
        kw = dict(n_limit=self.n_limit)
        fd = [self.fold_dcd if self.fold_switch == "smooth"
              else float(fold_effective(ac, self.fold_dcd, T[j], T[j])) for j in range(2 * n + 1)]
        for i in range(n + 1):
            tt = t + i * dt
            x, y, z, V, g, _ = st[:, 0]
            h = z - float(world.surface(x, y, tt)[0])
            tip = h - 0.5 * ac.span * np.sin(abs(mu[2 * i])) * np.cos(g)
            cause = ("centre" if h < self.clearance_m else "tip" if tip < self.clearance_m
                     else "stall" if V < v_lim else None)
            if cause is not None:
                return (cause, i * dt, float(h), float(tip), float(V))
            if i == n:
                break
            k1 = derivatives_vec(tt, st, ac, world, cl[2 * i:2 * i + 1], mu[2 * i:2 * i + 1],
                                 thrust=T[2 * i], fold_dcd=fd[2 * i], **kw)
            km = derivatives_vec(tt + dt / 2, st + dt / 2 * k1, ac, world,
                                 cl[2 * i + 1:2 * i + 2], mu[2 * i + 1:2 * i + 2],
                                 thrust=T[2 * i + 1], fold_dcd=fd[2 * i + 1], **kw)
            st = st + dt * km
            if not np.all(np.isfinite(st)):
                return ("stall", (i + 1) * dt, float("nan"), float("nan"), float("nan"))
        return None

    def override(self, s):
        """(cl, mu, thrust) of the reflex at state s."""
        ac = self.ac
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        cl = min(ac.cl_max / self.v_factor ** 2, self.n_frac * ac.n_max * ac.mass * G / q)
        return float(np.clip(cl, ac.cl_min, ac.cl_max)), 0.0, self.t_max


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
    # the roughness model of the true wind (Charnock for make_env, so as before; COARE 3.5 where the
    # experiment gives the true wind that)
    return Environment.flat(Wind(u10_est, env_true.wind.from_deg,
                                 roughness=getattr(env_true.wind, "roughness", "charnock")))


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
            log_every_s=5.0, stop_on_tip=False, library_path=None, motor=False, w_energy=None, reflex=False,
            true_fold_switch=None, target=None, navigator=None, nav_polar=None, nav_kw=None, nav_hull=None, guide_sig=None):
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

    motor (default False; nothing changes then): the thrust is a third control of the planner with
    the energy weight w_energy (None: energy_weight(ac, "normal")); the true dynamics carry the thrust
    and the folded-propeller drag. The metrics then also hold out["motor"]: electric energy and
    average power, the share of time with thrust, the number and length of bursts and where in the
    cycle they start (see burst_stats).

    target / navigator (default None; nothing changes then; needs a library): with target = (x, y)
    (metres, x east, y north, from the start at the origin) and navigator=True (or a
    sim.navigator.Navigator) the tactical layer chooses the course. Every nav.update_s (5 s) the
    Navigator gets the position, the wind (truth for now: u10_est and the true wind direction) and
    the target and returns the course; the planner then flies the library cycle of that course
    relative to the wind (CycleLibrary.pick, mirrored for the port side). target_bearing_deg is
    ignored. nav_polar: the sim.navigator.Polar, or nav_hull: the sim.navigator.HullSource of the
    aircraft (the hull navigator, which names the cycle to fly itself; one of them is needed when
    navigator is True); nav_kw: Navigator
    keywords (default arrival_radius NAV_ARRIVAL_M). guide_sig: the match tolerances of CycleGuide
    (h, V, gamma, heading); the navigator needs a tight heading (NAV_GUIDE_SIG), because the cycle's mean
    cross-track velocity is set by its mean heading, which the loose default does not hold. A change of cycle (course or side) drops the
    phase hint, so CycleGuide matches the phase of the new cycle to the state. The run ends "arrived"
    when the target is within the arrival radius. The metrics then also hold out["nav"]: the
    track (every NAV_TRACK_S), the commands, the tacks, the mode shares and the arrival time.

    reflex (default False; nothing changes then; needs motor=True): the Reflex supervisor runs at
    every control step on the true state, in the world of the latest plan, and overrides the
    commands with full thrust and a pull-up while the plan is predicted to leave 0.3 m of clearance
    or 1.05 stall speeds within 1.5 s. The metrics then also hold out["reflex"]: the number of
    reflexes, their causes, durations and electric energy (also counted in out["motor"]).
    """
    from sim.seasense import LocalWaveEstimator
    if reflex and not motor:
        raise ValueError("reflex needs motor=True (it commands the thrust)")
    ac = aircraft
    if library is None and library_path is not None:
        library = CycleLibrary.from_keyword(library_path)   # a library file picked by keyword
    kw = dict(seed=seed)
    kw.update(planner_kw or {})
    if motor:
        kw["motor"] = True
        if w_energy is not None:
            kw["w_energy"] = w_energy
    pl = MPPIPlanner(ac, **kw)
    pl.n_limit = bool(n_limit)
    pl.act = actuators
    rng = np.random.default_rng(seed + 1000)
    est = (estimator or LocalWaveEstimator()) if world_model == "estimated" else None
    for sn in sensors or []:
        sn.reset()
    nav, nav_log, nav_track, nav_next, nav_cmd = None, [], [], 0.0, None
    if navigator:
        from sim.navigator import Navigator
        from sim.record import value as _value
        if target is None or library is None:
            raise ValueError("navigator needs a target and a library")
        nav = navigator if hasattr(navigator, "step") else None
        if nav is None and nav_hull is not None:
            from sim.navigator import HullNavigator
            nav = HullNavigator(nav_hull, **dict({"arrival_radius": NAV_ARRIVAL_M}, **(nav_kw or {})))
        if nav is None:
            if nav_polar is None:
                raise ValueError("navigator=True needs nav_polar or nav_hull")
            nav = Navigator(nav_polar, **dict({"arrival_radius": NAV_ARRIVAL_M}, **(nav_kw or {})))
        nav.reset()
        nav_update_s = float(_value("nav.update_s", 5.0))
        target = np.asarray(target, float)
        nav_cmd = nav.step(0.0, (0.0, 0.0), u10_est, env_true.wind.from_deg, target)
        target_bearing_deg = nav_cmd.bearing
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
        pt0 = getattr(nav_cmd, "point", None)
        if pt0 is not None and pt0.steady is not None:
            c0 = Cycle(steady_entry(pt0), mirror=False)
        elif pt0 is not None and pt0.entry is not None:      # the hull navigator's first cycle
            c0 = Cycle(pt0.entry, mirror=pt0.side < 0)
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
    TH, VA, HH, PHA, PHL = [], [], [], [], []       # motor logs per step
    ph_ref = None                                   # (tau_s, period, low-point fraction, plan time)
    T_a = 0.0
    fold_state = (pl.fold_switch if true_fold_switch is None else true_fold_switch) == "state" if motor else False
    rf = Reflex(ac, pl.t_max, pl.fold_dcd, n_limit, pl.fold_switch) if reflex else None
    rf_events, rf_cur, world = [], None, None
    cyc_prev = None
    for i in range(nsteps + 1):
        t = i * dt
        eta_t = float(env_true.surface(s[0], s[1], t)[0])
        if s[2] - eta_t < 0.0:
            ended = "contact"
            break
        if nav is not None and math.hypot(target[0] - s[0], target[1] - s[1]) <= nav.arrival_radius:
            ended = "arrived"
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
            u10_p = u10_cur if wind_source == "estimate" else u10_est
            if nav is not None and t >= nav_next - 1e-9:
                from_n = from_cur if wind_source == "estimate" else from_deg
                nav_cmd = nav.step(t, (s[0], s[1]), u10_p, from_n, target,
                                   None if not nav_log else nav_log[-1]["bearing"])
                b_ = np.radians(nav_cmd.bearing)
                u = np.array([np.sin(b_), np.cos(b_)])
                nav_next = t + (NAV_FAST_UPDATE_S if nav_cmd.mode == "direct" else nav_update_s)
                nav_log.append({"t": t, "x": float(s[0]), "y": float(s[1]),
                                "bearing": float(nav_cmd.bearing), "chi": float(nav_cmd.chi),
                                "side": nav_cmd.side, "mode": nav_cmd.mode,
                                "vmg": float(nav_cmd.vmg), "tacked": bool(nav_cmd.tacked),
                                "distance": float(nav_cmd.distance),
                                "kind": getattr(nav_cmd, "kind", ""),
                                "per_m": float(getattr(nav_cmd, "per_m", 0.0)),
                                "cross": float(getattr(nav_cmd, "cross", 0.0)),
                                "cycle": (None if getattr(nav_cmd, "point", None) is None
                                          else nav_cmd.point.key)})
            chi_p = float(wrap(np.arctan2(u[1], u[0]) - psi_p))
            guide = None
            if library is not None:
                pt_ = getattr(nav_cmd, "point", None) if nav is not None else None
                if pt_ is not None and pt_.steady is not None:    # a steady motor leg: its own reference
                    cyc = Cycle(steady_entry(pt_), mirror=False)
                elif pt_ is not None and pt_.entry is not None:   # the hull navigator names the cycle
                    cyc = Cycle(pt_.entry, mirror=pt_.side < 0)
                else:
                    cyc = library.pick(ac.name, u10_p, chi_p)
                if cyc_prev is not None and (cyc.d is not cyc_prev.d or cyc.mirror != cyc_prev.mirror):
                    tau_prev = None          # another cycle: its phase is matched afresh
                cyc_prev = cyc
                guide = CycleGuide(cyc, world, t, s_plan, psi_p, pl.horizon_s,
                                  wave_aware=wave_aware, w_d=pl.w_d, sig=guide_sig,
                                  tau_hint=None if tau_prev is None else tau_prev + pl.replan_s)
                if tau_prev is not None and guide.tau0 < tau_prev - 0.5 * cyc.T:
                    n_cyc += 1
                tau_prev = guide.tau0
                ph_ref = (guide.tau_s, cyc.T, cyc.tau_low / cyc.T, t)
                deltas.append(guide.delta)
                mdist.append(guide.match_dist)
            if actuators is not None and actuators.cl is not None:
                pl.act_state = (actuators.cl, actuators.mu) + actuators.cmd_last
                if motor:
                    pl.act_state = pl.act_state + (actuators.T,)
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
        rx_on = False
        if reflex and ready:
            hit = rf.predict(world, t, s, plan.knots_t, plan.cl_knots, plan.mu_knots,
                             plan.thrust_knots, t - t_plan)
            if hit is not None:
                rx_on = True
                cl, mu, _ = rf.override(s)
                if rf_cur is None:
                    rf_cur = {"t_s": t, "cause": hit[0], "ahead_s": hit[1], "h_m": hit[2],
                              "tip_m": hit[3], "v_ms": hit[4], "duration_s": 0.0, "energy_j": 0.0}
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        if n_limit:
            cl = n_limited_cl(ac, s[3], cl)       # commanded cl is held to the limit
        if motor:
            T_cmd = (pl.t_max if rx_on else
                     float(np.clip(plan.thrust(t - t_plan), 0.0, pl.t_max)) if ready else 0.0)
            cl_a, mu_a = ((cl, mu) if actuators is None
                          else actuators.step(cl, mu, dt, s[3], thrust=T_cmd))
            T_a = T_cmd if actuators is None else actuators.T
            TH.append(T_a); VA.append(s[3]); HH.append(s[2] - eta_t)
            if rx_on:
                rf_cur["duration_s"] += dt
                rf_cur["energy_j"] += float(T_a * s[3] * dt / pl.eta)
            elif rf_cur is not None:
                rf_events.append(rf_cur); rf_cur = None
            if ph_ref is None:
                PHA.append(np.nan); PHL.append(np.nan)
            else:
                ph = ((ph_ref[0] + t - ph_ref[3]) % ph_ref[1]) / ph_ref[1]
                PHA.append(ph); PHL.append((ph - ph_ref[2]) % 1.0)
        else:
            cl_a, mu_a = (cl, mu) if actuators is None else actuators.step(cl, mu, dt, s[3])
        cl_f = n_limited_cl(ac, s[3], cl_a) if n_limit else cl_a
        T.append(t); ST.append(s.copy()); MU.append(mu_a); CL.append(cl_f)
        if nav is not None and i % int(round(NAV_TRACK_S / dt)) == 0:
            nav_track.append([float(t), float(s[0]), float(s[1])])
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
        if motor:
            fdc = float(fold_effective(ac, pl.fold_dcd, T_a, T_cmd)) if fold_state else pl.fold_dcd
            s = rk4_step(lambda tt, ss: derivatives(tt, ss, ac, env_true, cl_a, mu_a, n_limit,
                                                    thrust=T_a, fold_dcd=fdc)[0], t, s, dt)
        else:
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
    if motor:
        out["motor"] = burst_stats(ac, pl, dt, TH, VA, HH, PHA, PHL)
    if reflex:
        if rf_cur is not None:
            rf_events.append(rf_cur)
        ej = float(sum(e["energy_j"] for e in rf_events))
        out["reflex"] = {
            "n": len(rf_events), "total_s": float(sum(e["duration_s"] for e in rf_events)),
            "energy_j": ej, "avg_power_w": ej / max(len(T) * dt, dt),
            "causes": {c: sum(1 for e in rf_events if e["cause"] == c)
                       for c in ("centre", "tip", "stall")},
            "horizon_s": rf.horizon_s, "clearance_m": rf.clearance_m, "v_frac": rf.v_frac,
            "events": rf_events[:60]}
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
    if nav is not None:
        nav_track.append([float(t), float(s[0]), float(s[1])])
        out["nav"] = nav_summary(nav, nav_log, nav_track, target, ended == "arrived", float(t), nav_update_s)
    if keep:
        out["traj"] = {"t": np.array(T), "state": np.array(ST), "mu": np.array(MU),
                       "cl": np.array(CL)}
        if motor:
            out["traj"]["thrust"] = np.array(TH)
    return out


NAV_ARRIVAL_M = 200.0      # arrival radius of the closed-loop navigator (the Navigator default is 50 m)
STEADY_PERIOD_S = 8.0     # period of the synthetic cycle of a steady leg (nothing in it depends on the phase)


def steady_entry(pt):
    """Library-entry dict of a steady motor leg (HullPoint.steady, set by sim.navigator.HullNavigator): level
    flight at the leg's height, airspeed, lift coefficient, thrust and heading (the angle in the wind frame is
    the entry's psi), wings level. The same fields as a library cycle, so Cycle and CycleGuide use it as
    the planner's reference unchanged. Cached on the point."""
    st = pt.steady
    if "_entry" not in st:
        n, P = 9, STEADY_PERIOD_S
        t = [P * i / (n - 1) for i in range(n)]
        psi = float(np.arctan2(st["heading"][1], st["heading"][0]))
        st["_entry"] = {"aircraft": "steady", "u10": 0.0, "chi_deg": round(abs(float(np.degrees(psi))), 1),
                        "margin_set": "steady", "success": True, "kind": "motor",
                        "period": P, "t": t, "x": [pt.vx * v for v in t], "y": [pt.vy * v for v in t],
                        "z": [st["height"]] * n, "V": [st["V"]] * n, "gamma": [0.0] * n, "psi": [psi] * n,
                        "cl": [st["cl"]] * n, "mu": [0.0] * n, "thrust": [st["thrust"]] * n,
                        "power_avg_w": pt.power}
    return st["_entry"]


NAV_FAST_UPDATE_S = 1.0     # the navigator is called this often in the final approach (mode direct)
NAV_TRACK_S = 5.0          # spacing of the logged track points


def nav_summary(nav, log, track, target, arrived, t_end, update_s):
    """Log of a navigated run: track [t, x, y] every NAV_TRACK_S, the commands (every navigator
    call), the tack times, the time share of each navigator mode, and the arrival."""
    d0 = float(np.hypot(*np.asarray(target, float))) if len(track) else 0.0
    xe, ye = track[-1][1], track[-1][2]
    d1 = float(np.hypot(target[0] - xe, target[1] - ye))
    modes = {}
    for k, c in enumerate(log):
        t1 = log[k + 1]["t"] if k + 1 < len(log) else t_end
        modes[c["mode"]] = modes.get(c["mode"], 0.0) + max(0.0, t1 - c["t"])
    tacks = [c["t"] for c in log if c["tacked"]]
    kinds = {}
    for k, c in enumerate(log):
        t1 = log[k + 1]["t"] if k + 1 < len(log) else t_end
        kinds[c.get("kind", "")] = kinds.get(c.get("kind", ""), 0.0) + max(0.0, t1 - c["t"])
    return {"target": [float(target[0]), float(target[1])], "arrived": bool(arrived),
            "arrival_time_s": t_end if arrived else None, "end_distance_m": d1,
            "start_distance_m": d0, "mean_vmg_ms": (d0 - d1) / t_end if t_end > 0 else 0.0,
            "n_tacks": len(tacks), "tack_times_s": tacks, "mode_time_s": modes, "kind_time_s": kinds,
            "update_s": float(update_s),
            "navigator": {"tack_cost_s": nav.tack_cost_s, "hysteresis": nav.hysteresis,
                          "min_leg_s": nav.min_leg_s, "arrival_radius_m": nav.arrival_radius,
                          "amort_s": getattr(nav, "amort_s", None)},
            "track": track, "commands": [[c["t"], c["bearing"], c["chi"], c["side"], c["mode"],
                                          c["vmg"], c["tacked"], c.get("kind", ""), c.get("cycle")]
                                         for c in log]}


BURST_ON_FRAC = 0.01       # thrust above this share of the weight counts as running


def burst_stats(ac, pl, dt, TH, VA, HH, PHA, PHL, max_listed=100):
    """Electric energy, average power and the bursts of a motor run. The arrays hold one value per
    integration step (actual thrust N, airspeed, true height above the surface, phase of the
    library cycle as a fraction of its period and relative to its low point, NaN without a
    library). Energy is the sum of T V dt / (eta_p eta_m); a burst is a run of steps with
    T > BURST_ON_FRAC m g."""
    T, V, H = np.asarray(TH, float), np.asarray(VA, float), np.asarray(HH, float)
    PH, PL = np.asarray(PHA, float), np.asarray(PHL, float)
    dur = len(T) * dt
    e_j = float(np.sum(T * V) * dt / pl.eta)
    on = T > BURST_ON_FRAC * ac.mass * G
    edges = np.diff(np.concatenate([[0], on.astype(int), [0]]))
    st, en = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    ln = (en - st) * dt
    out = {"w_energy": pl.w_energy, "t_max_n": pl.t_max, "eta_total": pl.eta, "duration_s": dur,
           "energy_j": e_j, "energy_wh": e_j / 3600.0,
           "avg_power_w": e_j / dur if dur > 0 else 0.0,
           "wh_per_hour": e_j / 3600.0 / (dur / 3600.0) if dur > 0 else 0.0,
           "thrust_on_share": float(on.mean()) if len(on) else 0.0,
           "mean_thrust_frac_weight": float(T.mean() / (ac.mass * G)) if len(T) else 0.0,
           "peak_thrust_frac_weight": float(T.max() / (ac.mass * G)) if len(T) else 0.0,
           "n_bursts": int(len(st)), "burst_threshold_n": BURST_ON_FRAC * ac.mass * G,
           "h_median_all_m": float(np.median(H)) if len(H) else None}
    if len(st):
        out["burst_len_median_s"] = float(np.median(ln))
        out["burst_len_mean_s"] = float(np.mean(ln))
        out["burst_len_max_s"] = float(np.max(ln))
        out["burst_peak_thrust_frac"] = float(np.mean([T[a:b].max() for a, b in zip(st, en)])
                                              / pl.t_max)
        out["burst_h_start_median_m"] = float(np.median(H[st]))
        out["burst_h_start_q25_q75_m"] = [float(np.quantile(H[st], 0.25)),
                                          float(np.quantile(H[st], 0.75))]
        ph, pr = PH[st], PL[st]
        if np.all(np.isfinite(ph)):
            out["burst_phase_hist8"] = np.histogram(ph, bins=8, range=(0.0, 1.0))[0].tolist()
            for key, v in (("burst_phase", ph), ("burst_phase_rel_low", pr)):
                z = np.mean(np.exp(2j * np.pi * v))
                out[key + "_circ_mean"] = float((np.angle(z) / (2 * np.pi)) % 1.0)
                out[key + "_concentration"] = float(abs(z))
        out["bursts"] = [{"t_s": float(a * dt), "len_s": float((b - a) * dt), "h_m": float(H[a]),
                          "phase": None if not np.isfinite(PH[a]) else float(PH[a]),
                          "phase_rel_low": None if not np.isfinite(PL[a]) else float(PL[a]),
                          "peak_n": float(T[a:b].max())}
                         for a, b in list(zip(st, en))[:max_listed]]
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
# w_energy (cost per joule of electric energy) of the motor option as three named levels, in
# multiples of the break-even weight w_e eta_p eta_m / m (the cost of one electric joule equals the
# worth of the mechanical specific energy it buys). Chosen from a pilot on a flat sea below the
# soaring minimum: 0.03 gives near continuous thrust, 2 about what a steady flap-glide needs or less,
# 4 sits just before the planner stops using the motor and the flight ends in contact.
W_ENERGY_LEVELS = {"free": 0.03, "normal": 2.0, "costly": 4.0}


def energy_weight(ac, level, w_e=1.0):
    """w_energy (cost per electric joule) of a named level for this aircraft."""
    from sim.dsopt import prop_efficiency
    return W_ENERGY_LEVELS[level] * w_e * prop_efficiency() / ac.mass
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


BURST_LEVELS = ("free", "normal", "costly")
BURST_CASES_A = (("baseline-2m", 11.0), ("baseline-2m", 9.0), ("baseline-2m", 6.0),
                 ("albatross-3m", 9.0), ("albatross-3m", 6.0))
BURST_MAX_COMPONENTS = 48        # wave components of the planner world "truth" (as the browser)
FLAP_GLIDE_TRACK_DEG = 160.0     # as sim.formstudy: track 160 deg from the wind (power does not depend on it)


def burst_jobs(part, t_end):
    """job = (aircraft, u10, sea, planner world, level or None, t_end). part: "a", "b" or "ab"."""
    jobs = []
    if "a" in part:
        jobs += [(n, u, "flat", "flat", lv, t_end) for n, u in BURST_CASES_A
                 for lv in BURST_LEVELS + (None,)]
    if "b" in part:
        jobs += [(n, u, "waves", wm, lv, t_end) for n in ("baseline-2m", "albatross-3m")
                 for u in (13.0, 16.0) for wm in ("flat", "truth") for lv in BURST_LEVELS + (None,)]
    return jobs


def run_burst_job(job):
    """One closed-loop run of the burst experiment: margin library, actuators pb2v, n_limit,
    stop_on_tip, truth state and wind, seed 1, the sea of the margin experiment (make_env). level
    None: motor off (as the margin experiment)."""
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim.autonomy import make_env
    name, u10, sea, wm, level, t_end = job
    ac = by_name(name)
    env = make_env(sea, u10)
    t0 = time.time()
    r = fly_mpc(ac, env, wm, [], t_end, env.wind.to_deg - 20.0, u10, seed=1,
                planner_kw=dict(LIB_PKW), library=CycleLibrary.from_keyword("margin"),
                state_source="truth", wind_source="truth",
                max_components=BURST_MAX_COMPONENTS if wm == "truth" else None,
                actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                wave_aware=False, motor=level is not None,
                w_energy=None if level is None else energy_weight(ac, level))
    r["wall_s"] = time.time() - t0
    r = _no_nan(r)
    m = r.get("motor")
    if m is not None:
        m["bursts"] = m.get("bursts", [])[:60]
    return {"aircraft": name, "u10": u10, "sea": sea, "planner_world": wm,
            "level": level or "off", "source": "this run", **r}


def _margin_off_row(name, u10):
    """The motor-off row of the margin experiment (sea, flat planner world, truth state) or None."""
    path = os.path.join(os.path.dirname(__file__), "out", "mpc.json")
    for r in json.load(open(path)).get("margin_exp", {}).get("results", []):
        if (r["aircraft"], r["u10"], r["library"], r["case"][0]) == (name, u10, "margin", "c"):
            keep = ("ended", "survival_s", "energy_per_min_last60", "min_clear_surface",
                    "min_clear_wingtip", "wall_s")
            return {"aircraft": name, "u10": u10, "sea": "waves", "planner_world": "flat",
                    "level": "off", "source": "margin_exp case c", **{k: r.get(k) for k in keep}}
    return None


def burst_table(rows):
    lines = ["| aircraft | u10 | sea / world | level | end (s) | avg power (W) | Wh per hour | "
             "thrust on | bursts | median burst (s) | mean / peak thrust (% weight) | "
             "burst start h (m), phase | dE/min last 60 s |", "|" + "---|" * 13]
    for r in rows:
        m = r.get("motor") or {}
        lines.append("| %s | %g | %s / %s | %s | %s %.0f | %s | %s | %s | %s | %s | %s | %s, %s | %s |" % (
            r["aircraft"], r["u10"], r["sea"], r["planner_world"], r["level"], r["ended"],
            r["survival_s"], _fmt(m.get("avg_power_w"), "%.1f"), _fmt(m.get("wh_per_hour"), "%.1f"),
            _fmt(m.get("thrust_on_share"), "%.2f"), m.get("n_bursts", "n/a"),
            _fmt(m.get("burst_len_median_s"), "%.1f"),
            "n/a" if not m else "%.1f / %.1f" % (100 * m["mean_thrust_frac_weight"],
                                                 100 * m["peak_thrust_frac_weight"]),
            _fmt(m.get("burst_h_start_median_m"), "%.1f"),
            _fmt(m.get("burst_phase_circ_mean"), "%.2f"), _fmt(r.get("energy_per_min_last60"), "%.0f")))
    return chr(10).join(lines)


def burst_main(a):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    from sim.aircraft_models import by_name
    from sim.patterns import flap_glide_cost
    from sim.record import value
    jobs = burst_jobs(a.burst_part, a.t_end)
    ref_rows = []
    keep_jobs = []
    for j in jobs:      # motor-off rows of the margin experiment are taken where they exist
        off = _margin_off_row(j[0], j[1]) if j[4] is None and j[2] == "waves" and j[3] == "flat" else None
        if off is not None:
            ref_rows.append(off)
        else:
            keep_jobs.append(j)
    keep_jobs.sort(key=lambda j: -(j[3] == "truth") * 2 - (j[2] == "waves"))      # slow ones first
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_burst_job, keep_jobs, chunksize=1))
    rows += ref_rows
    levels = ("off",) + BURST_LEVELS
    order = lambda r: (r["sea"] == "waves", r["aircraft"], r["u10"], r["planner_world"],
                       levels.index(r["level"]))
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    old = d.get("burst_exp", {}).get("results", [])
    key = lambda r: (r["aircraft"], r["u10"], r["sea"], r["planner_world"], r["level"])
    new_keys = {key(r) for r in rows}
    rows = sorted([r for r in old if key(r) not in new_keys] + rows, key=order)
    print(burst_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    eff = (value("prop.prop_eff", 0.60), value("prop.motor_eff", 0.75))
    fg = {}
    for n in ("baseline-2m", "albatross-3m"):
        f = flap_glide_cost(by_name(n), 9.0, FLAP_GLIDE_TRACK_DEG, prop_eff=eff[0], motor_eff=eff[1])
        fg[n] = {"power_w": f["power_w"], "airspeed": f["airspeed"], "sink": f["sink"]}
    d["burst_exp"] = {
        "meta": {**record.meta("python -m sim.mpc --burst-exp --burst-part %s --t-end %g"
                               % (a.burst_part, a.t_end),
                               [__file__, LIB_MARGIN_PATH,
                                os.path.join(os.path.dirname(__file__), "aircraft.py"),
                                os.path.join(os.path.dirname(__file__), "dsopt.py")]),
                 "planner_kw": LIB_PKW, "runtime_s_last_call": runtime, "t_end": a.t_end, "seed": 1,
                 "state_source": "truth", "wind_source": "truth", "wave_aware": False,
                 "actuators": "rate_model pb2v (record ctrl.*), thrust lag prop.thrust_tau_s",
                 "n_limit": True, "stop_on_tip": True, "library": "margin",
                 "sea": "make_env('flat', u10) for (a); make_env('waves', u10), the sea of the "
                        "margin experiment, for (b)",
                 "planner_world_truth": "world_model truth with the %d strongest wave components"
                                        % BURST_MAX_COMPONENTS,
                 "motor": {"t_max": "prop.thrust_max_per_weight m g", "eta": "prop.prop_eff * prop.motor_eff",
                           "folded_drag": "prop.folded_drag_cda_m2 / S with the smooth switch, in "
                                          "the planner and in the true dynamics",
                           "thrust_sigma_frac": value("ctrl.thrust_sigma_frac", 0.15),
                           "thrust_tau_s": value("prop.thrust_tau_s", 0.1),
                           "perturbation": "Gaussian per knot, clipped to [0, T_max]",
                           "burst": "actual thrust above %g of the weight" % BURST_ON_FRAC},
                 "w_energy_levels": {
                     "definition": "w_energy = k w_e eta / m (cost per electric joule), k = "
                                   + ", ".join("%s %g" % kv for kv in W_ENERGY_LEVELS.items()),
                     "values": {n: {lv: energy_weight(by_name(n), lv) for lv in BURST_LEVELS}
                                for n in ("baseline-2m", "albatross-3m")},
                     "how_chosen": "pilot on the flat sea below the soaring minimum (baseline u10 "
                                   "6, 9, 11, albatross u10 9, 150 s): k 0.03 gives thrust on "
                                   "all the time (about 15 to 28 W); k near 2 gives the planner's "
                                   "working range; above k 4 to 5 the planner stops using the motor "
                                   "before the horizon shows the contact and the flight ends"},
                 "flap_glide": {"note": "steady flap-glide at best L/D with the same efficiencies "
                                        "(patterns.flap_glide_cost), wind independent",
                                "aircraft": fg},
                 "motor_off_rows": "case c of margin_exp where it exists (sea, flat planner world); "
                                   "the other motor-off rows are run here without the folded-propeller "
                                   "drag (motor off means no motor in the aircraft)",
                 "energy_trend": "(E_end - E_start)/(t_end - t_start) * 60 over the last 60 s "
                                 "flown, E = g z + V^2/2 (J/kg per min)"},
        "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(_no_nan(d), fh, indent=1)
        fh.write(chr(10))


MOTOR_EXP_PKW = dict(LIB_PKW, thrust_sampling="sustained", fold_switch="state")
MOTOR_EXP_EXTRA_AIRCRAFT = ("sealedhull-2p5m", "easyglider-1p8m")


def _make_env_coare(sea, u10):
    """make_env with the COARE 3.5 roughness of the library and the optimizer for the true wind
    (make_env uses the fixed Charnock constant): flat sea, or the fully developed sea of seed 1."""
    from sim.seawind import WaveTrain
    wind = Wind(u10, 270.0, roughness="coare35")
    if sea == "flat":
        return Environment.flat(wind)
    return Environment(wind, [WaveTrain.fully_developed(wind)], seed=1)


def motor_exp_cases(lib=None):
    """(aircraft, u10, sea, planner world) of the experiment: the burst experiment's cases (flat sea
    below the soaring minimum, then the waves at 13 and 16 with the flat and the truth world) and,
    for sealedhull-2p5m and easyglider-1p8m, the two highest winds of the library grid (3 m/s or
    more) that have motor cycles and lie below the lowest wind with a soaring cycle."""
    lib = lib or CycleLibrary.load(LIB_MOTOR_PATH)
    cases = [(n, u, "flat", "flat") for n, u in BURST_CASES_A]
    cases += [(n, u, "waves", wm) for n in ("baseline-2m", "albatross-3m") for u in (13.0, 16.0)
              for wm in ("flat", "truth")]
    for n in MOTOR_EXP_EXTRA_AIRCRAFT:
        es = [e for e in lib.entries if e["aircraft"] == n]
        soar = [e["u10"] for e in es if e["kind"] == "soaring"]
        lo = min(soar) if soar else float("inf")
        ws = sorted({e["u10"] for e in es if e["kind"] == "motor" and e["u10"] < lo and e["u10"] >= 3.0})
        cases += [(n, u, "flat", "flat") for u in ws[-2:]]
    return cases


def run_motor_job(job):
    """One run of the motor-library experiment: motor library, middle energy weight, sustained thrust
    sampling and constant-thrust candidates, folded drag switched on the commanded motor state,
    COARE 3.5 true wind, actuators pb2v, n_limit, stop_on_tip, truth state and wind, seed 1."""
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    name, u10, sea, wm, reflex, t_end = job
    ac = by_name(name)
    env = _make_env_coare(sea, u10)
    t0 = time.time()
    r = fly_mpc(ac, env, wm, [], t_end, env.wind.to_deg - 20.0, u10, seed=1,
                planner_kw=dict(MOTOR_EXP_PKW), library=CycleLibrary.from_keyword("motor"),
                state_source="truth", wind_source="truth",
                max_components=BURST_MAX_COMPONENTS if wm == "truth" else None,
                actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                wave_aware=False, motor=True, w_energy=energy_weight(ac, "normal"), reflex=reflex)
    r["wall_s"] = time.time() - t0
    r = _no_nan(r)
    r["motor"]["bursts"] = r["motor"].get("bursts", [])[:60]
    log = r.pop("reflex", None)
    return {"aircraft": name, "u10": u10, "sea": sea, "planner_world": wm,
            "reflex": "on" if reflex else "off", "reflex_log": log, **r}


def _optimizer_row(lib, name, u10):
    """The library entry (chi 20 deg) the planner flies at this wind: min-power motor cycle or
    soaring cycle, with the figures of the optimizer."""
    es = [e for e in lib.entries if e["aircraft"] == name and abs(e["chi_deg"] - 20.0) < 1e-9]
    if not es:
        return None
    u = min({e["u10"] for e in es}, key=lambda v: abs(v - u10))
    e = [e for e in es if e["u10"] == u][0]
    return {"u10": e["u10"], "kind": e["kind"], "margin_set": e["margin_set"],
            "power_avg_w": e.get("power_avg_w", 0.0), "power_avg_dense_w": e.get("power_avg_dense_w"),
            "thrust_on_share": e.get("thrust_on_share", 0.0),
            "speed_along_chi": e["speed_along_chi"], "period": e["period"]}


def _burst_row(name, u10, sea, wm):
    """The middle-weight row of burst_exp for the same case, or None (the flat-sea cases of
    burst_exp are the same ones; its true wind was Charnock)."""
    path = os.path.join(os.path.dirname(__file__), "out", "mpc.json")
    for r in json.load(open(path)).get("burst_exp", {}).get("results", []):
        if (r["aircraft"], r["u10"], r["sea"], r["planner_world"], r["level"]) == (
                name, u10, sea, wm, "normal"):
            m = r.get("motor") or {}
            return {"ended": r["ended"], "survival_s": r["survival_s"],
                    "avg_power_w": m.get("avg_power_w"), "thrust_on_share": m.get("thrust_on_share"),
                    "n_bursts": m.get("n_bursts"), "v_toward_target": r.get("v_toward_target")}
    return None


def motor_exp_table(rows):
    lines = ["| aircraft | u10 | sea / world | reflex | end (s) | avg power (W) | thrust on | bursts | "
             "reflexes (n, J) | v toward (m/s), Wh/km | burst_exp normal: end, W | optimizer: W, speed (m/s), Wh/km |",
             "|" + "---|" * 12]
    for r in rows:
        m, rf, b, o = r.get("motor") or {}, r.get("reflex_log") or {}, r.get("burst_exp"), r.get("optimizer")
        lines.append("| %s | %g | %s / %s | %s | %s %.0f | %s | %s | %s | %s | %s | %s | %s |" % (
            r["aircraft"], r["u10"], r["sea"], r["planner_world"], r["reflex"], r["ended"],
            r["survival_s"], _fmt(m.get("avg_power_w"), "%.1f"), _fmt(m.get("thrust_on_share"), "%.2f"),
            m.get("n_bursts", "n/a"),
            "n/a" if not rf else "%d, %.0f" % (rf["n"], rf["energy_j"]),
            "%s, %s" % (_fmt(r.get("v_toward_target"), "%.1f"), _fmt(r.get("wh_per_km"), "%.2f")),
            "n/a" if not b else "%s %.0f, %s" % (b["ended"], b["survival_s"], _fmt(b["avg_power_w"], "%.1f")),
            "n/a" if not o else "%s, %s, %s" % (_fmt(o["power_avg_w"], "%.1f"), _fmt(o["speed_along_chi"], "%.1f"),
                            _fmt(o.get("wh_per_km"), "%.2f"))))
    return chr(10).join(lines)


def motor_exp_main(a):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    from sim.record import value
    lib = CycleLibrary.load(LIB_MOTOR_PATH)
    cases = motor_exp_cases(lib)
    jobs = [(n, u, sea, wm, rx, a.t_end) for n, u, sea, wm in cases for rx in (False, True)]
    jobs.sort(key=lambda j: -(j[3] == "truth") * 2 - (j[2] == "waves"))      # slow ones first
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_motor_job, jobs, chunksize=1))
    rows.sort(key=lambda r: (r["sea"] == "waves", r["aircraft"], r["u10"], r["planner_world"],
                             r["reflex"]))
    for r in rows:
        r["burst_exp"] = _burst_row(r["aircraft"], r["u10"], r["sea"], r["planner_world"])
        r["optimizer"] = _optimizer_row(lib, r["aircraft"], r["u10"])
        v = r.get("v_toward_target")
        r["wh_per_km"] = (r["motor"]["avg_power_w"] / (3.6 * v)) if v and v > 0.05 else None
        o = r["optimizer"]
        if o is not None:
            o["wh_per_km"] = o["power_avg_w"] / (3.6 * o["speed_along_chi"]) if o["speed_along_chi"] > 0.05 else None
    print(motor_exp_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    d["motor_library_exp"] = {
        "meta": {**record.meta("python -m sim.mpc --motor-lib-exp --t-end %g --workers %d"
                               % (a.t_end, a.workers),
                               [__file__, LIB_MOTOR_PATH,
                                os.path.join(os.path.dirname(__file__), "aircraft.py"),
                                os.path.join(os.path.dirname(__file__), "dsopt.py")]),
                 "planner_kw": MOTOR_EXP_PKW, "runtime_s": runtime, "t_end": a.t_end, "seed": 1,
                 "workers": a.workers, "library": "motor (sim/out/mpc_library_motor.json)",
                 "energy_level": "normal (k = %g)" % W_ENERGY_LEVELS["normal"],
                 "true_wind": "Wind(u10, 270, roughness='coare35') (flat sea) and the fully "
                              "developed sea of seed 1 on that wind (waves): COARE 3.5 as the library "
                              "and the optimizer; burst_exp used make_env with the Charnock constant. "
                              "The planner's flat world takes the roughness of the true wind.",
                 "thrust_sampling": "sustained: half of the samples hold one half-normal level "
                                    "(scale %g T_max) over a random run of knots, the rest "
                                    "independent Gaussian (ctrl.thrust_sigma_frac); candidates with "
                                    "constant thrust %s of the weight, with the nominal controls and "
                                    "wings level at best L/D" % (HOLD_SIGMA_FRAC, list(THRUST_CAND_LEVELS)),
                 "fold_switch": "state: in the planner's rollouts and the true dynamics the "
                                "propeller is folded (full folded drag) when the commanded thrust is "
                                "below %g of the weight and not folded (zero) otherwise, instead of "
                                "the smooth switch; the optimizer keeps the smooth one" % FOLD_CMD_FRAC,
                 "reflex": {"horizon_s": value("ctrl.reflex_horizon_s", 1.5),
                            "clearance_m": value("ctrl.reflex_clearance_m", 0.3),
                            "v_frac": value("ctrl.reflex_v_frac", 1.05), "rollout_dt_s": REFLEX_DT,
                            "prediction": "the plan's own controls, flat or truth world of the planner"},
                 "state_source": "truth", "wind_source": "truth", "wave_aware": False,
                 "actuators": "rate_model pb2v, thrust lag prop.thrust_tau_s", "n_limit": True,
                 "stop_on_tip": True, "planner_world_truth": "%d strongest wave components"
                 % BURST_MAX_COMPONENTS,
                 "comparison": "burst_exp: the normal-level row of the same aircraft, wind, sea and "
                               "world (Charnock true wind, margin library, no sustained sampling); "
                               "optimizer: the library entry at chi 20 deg of the nearest library "
                               "wind (same margins, no ground effect, smooth fold, speed along chi "
                               "from the optimizer; power in W is the average electric power)"},
        "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(_no_nan(d), fh, indent=1)
        fh.write(chr(10))


NAV_CASES = (       # (aircraft, u10, direction of the target from the start)
    ("albatross-3m", 13.0, "upwind"), ("albatross-3m", 13.0, "across"),
    ("albatross-3m", 16.0, "upwind"), ("albatross-3m", 16.0, "across"), ("albatross-3m", 16.0, "downwind"),
    ("baseline-2m", 16.0, "upwind"), ("baseline-2m", 16.0, "across"),
    ("albatross-3m", 9.0, "upwind"))
NAV_PKW = dict(MOTOR_EXP_PKW, thrust_update="raw")      # raw: the unbiased thrust update (see MPPIPlanner)
STEADY_HEIGHT_REF = 10.0                  # sim.navigator.STEADY_HEIGHT_M
NAV_GUIDE_SIG = (3.0, None, 0.25, 0.1)     # heading tolerance 0.1 rad instead of 0.6 (see fly_mpc guide_sig)


def nav_target(direction, dist):
    """Target (x east, y north) of the experiment: the wind blows from 270 deg (toward +x), so upwind
    is -x, downwind +x; across the wind is +y."""
    return {"upwind": (-dist, 0.0), "downwind": (dist, 0.0)}.get(direction, (0.0, dist))


def run_nav_job(job):
    """One navigated flight of nav_exp: flat COARE 3.5 sea, truth state and wind, motor on at the
    middle energy weight, the settings of the motor library experiment, library "motor", the hull
    navigator. job = (aircraft, u10, direction, distance m, t_end s or None). Returns the closed-loop
    result plus the kinematic prediction (sail_hull) on the same hull."""
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim.navigator import HullNavigator, HullSource, sail_hull
    name, u10, direction, dist, t_end = job
    ac = by_name(name)
    env = _make_env_coare("flat", u10)
    lib = CycleLibrary.from_keyword("motor")
    src = HullSource.from_entries(lib.entries, name)
    hull = src.at(u10)
    tgt = nav_target(direction, dist)
    from sim.navigator import SteadyModel
    steady = SteadyModel.from_aircraft(ac)
    pred = sail_hull(src, lambda t: (u10, env.wind.from_deg), (0.0, 0.0), tgt,
                     nav=HullNavigator(src, arrival_radius=NAV_ARRIVAL_M, steady=steady), t_max=6 * 3600.0)
    first = HullNavigator(src, arrival_radius=NAV_ARRIVAL_M, steady=steady).step(
        0.0, (0.0, 0.0), u10, env.wind.from_deg, tgt)
    if t_end is None:
        t_end = min(2.0 * pred["time"] + 600.0, 7200.0) if pred["arrived"] else 3600.0
    t0 = time.time()
    r = fly_mpc(ac, env, "flat", [], t_end, 0.0, u10, seed=1, planner_kw=dict(NAV_PKW),
                library=lib, state_source="truth", wind_source="truth",
                actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                wave_aware=False, motor=True, w_energy=energy_weight(ac, "normal"),
                target=tgt, navigator=True, nav_hull=src, guide_sig=NAV_GUIDE_SIG,
                nav_kw={"steady": steady})
    r["wall_s"] = time.time() - t0
    r = _no_nan(r)
    r["motor"]["bursts"] = r["motor"].get("bursts", [])[:20]
    r["predicted"] = {"arrived": pred["arrived"], "time_s": pred["time"], "n_legs": pred["n_legs"],
                      "mean_vmg_ms": pred["mean_vmg"], "energy_wh": pred["energy_j"] / 3600.0,
                      "plan_vmg_ms": first.vmg, "plan_kind": first.kind,
                      "plan_energy_per_m_j": first.per_m, "hull_wind_u10": hull.u10,
                      "track": pred["track"][::10].tolist()}
    return {"aircraft": name, "u10": u10, "direction": direction, "distance_m": dist,
            "hull_points": len(hull.points), **r}


def nav_exp_table(rows):
    lines = ["| aircraft | u10 | target | result | time (s) | legs | VMG (m/s) | hull plan: kind, VMG, "
             "kinematic time (s), legs, VMG | energy (Wh) | Wh/km made good | thrust on |",
             "|" + "---|" * 11]
    for r in rows:
        nv, m, p = r.get("nav") or {}, r.get("motor") or {}, r["predicted"]
        km = (nv.get("start_distance_m", 0) - nv.get("end_distance_m", 0)) / 1000.0
        lines.append("| %s | %g | %s %g km | %s | %s | %s | %s | %s, %s, %s, %s, %s | %s | %s | %s |" % (
            r["aircraft"], r["u10"], r["direction"], r["distance_m"] / 1000.0,
            r["ended"], _fmt(nv.get("arrival_time_s"), "%.0f"), nv.get("n_tacks", "n/a"),
            _fmt(nv.get("mean_vmg_ms"), "%.2f"), p["plan_kind"] or "none", _fmt(p["plan_vmg_ms"], "%.2f"),
            _fmt(p["time_s"], "%.0f"), p["n_legs"], _fmt(p["mean_vmg_ms"], "%.2f"),
            _fmt(m.get("energy_wh"), "%.1f"),
            _fmt(m["energy_wh"] / km if km > 0.05 and m.get("energy_wh") is not None else None, "%.2f"),
            _fmt(m.get("thrust_on_share"), "%.2f")))
    return chr(10).join(lines)


def nav_exp_main(a):
    from concurrent.futures import ProcessPoolExecutor

    from sim import record
    from sim.record import value
    dist = a.nav_dist
    jobs = [(n, u, d, dist, None) for n, u, d in NAV_CASES]
    if a.nav_only:
        keys = set(a.nav_only.split(";"))
        jobs = [j for j in jobs if "%s,%g,%s" % (j[0], j[1], j[2]) in keys]
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_nav_job, jobs, chunksize=1))
    print(nav_exp_table(rows))
    runtime = time.time() - t0
    print("runtime %.0f s" % runtime)
    d = json.load(open(a.out)) if os.path.exists(a.out) else {}
    d[a.nav_key] = {
        "meta": {**record.meta("python -m sim.mpc --nav-exp --nav-dist %g --workers %d"
                               % (dist, a.workers),
                               [__file__, LIB_MOTOR_PATH, os.path.join(os.path.dirname(__file__), "navigator.py"),
                                os.path.join(os.path.dirname(__file__), "out", "ds_polar.json")]),
                 "planner_kw": NAV_PKW, "runtime_s": runtime, "distance_m": dist, "seed": 1,
                 "workers": a.workers, "library": "motor (sim/out/mpc_library_motor.json)",
                 "energy_level": "normal (k = %g)" % W_ENERGY_LEVELS["normal"],
                 "sea": "flat, Wind(u10, 270, roughness='coare35'); wind from the west, toward +x",
                 "targets": "upwind = (-D, 0), across = (0, D) from the start at the origin",
                 "state_source": "truth", "wind_source": "truth", "reflex": False,
                 "actuators": "rate_model pb2v", "n_limit": True, "stop_on_tip": True,
                 "navigator": "sim.navigator.HullNavigator on the velocity hull of the library entries "
                              "(HullSource, nearest library wind), arrival radius %g m, called every %g s "
                              "(nav.update_s)" % (NAV_ARRIVAL_M, value("nav.update_s", 5.0)),
                 "steady_leg": "where no soaring pair makes progress the motor leg is a steady level flight at "
                               "%g m aimed by the wind triangle at the airspeed of least electric energy per "
                               "metre (sim.navigator.SteadyModel), the planner's reference; the hull's 180 deg "
                               "motor cycles stay as points" % STEADY_HEIGHT_REF,
                 "planner": "MOTOR_EXP_PKW plus thrust_update raw (the unbiased thrust update), guide heading "
                            "tolerance %g rad (NAV_GUIDE_SIG)" % NAV_GUIDE_SIG[3],
                 "prediction": "sim.navigator.sail_hull on the same hull and navigator, tack pause 30 s",
                 "t_end": "2 x the predicted arrival time + 600 s, at most 7200 s",
                 "mean_vmg": "(start distance - end distance) / time, in m/s"},
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
    ap.add_argument("--build-motor-library", action="store_true")
    ap.add_argument("--extend-motor-library", action="store_true")
    ap.add_argument("--extend-motor-upwind", action="store_true")
    ap.add_argument("--nav-exp", action="store_true")
    ap.add_argument("--nav-dist", type=float, default=10000.0)
    ap.add_argument("--nav-key", default="nav_exp", help="key of sim/out/mpc.json the nav experiment writes")
    ap.add_argument("--nav-only", default="", help="cases as 'aircraft,u10,direction' joined by ;")
    ap.add_argument("--margin-exp", action="store_true")
    ap.add_argument("--burst-exp", action="store_true")
    ap.add_argument("--motor-lib-exp", action="store_true")
    ap.add_argument("--burst-part", default="ab", help="a, b or ab")
    a = ap.parse_args()
    if a.build_margin_library:
        build_margin_library(workers=a.workers)
        return
    if a.build_motor_library:
        build_motor_library(workers=a.workers)
        return
    if a.extend_motor_upwind:
        extend_motor_upwind(workers=a.workers)
        return
    if a.nav_exp:
        nav_exp_main(a)
        return
    if a.extend_motor_library:
        extend_motor_library(workers=a.workers)
        return
    if a.margin_exp:
        margin_main(a)
        return
    if a.burst_exp:
        burst_main(a)
        return
    if a.motor_lib_exp:
        motor_exp_main(a)
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
