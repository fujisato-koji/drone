"""Point-mass glider flight model in a wind field."""
import json
from collections import deque
from dataclasses import dataclass

import numpy as np

from sim.record import value
from sim.seawind import RHO_AIR

G = 9.81


@dataclass
class Aircraft:
    name: str
    mass: float
    span: float
    wing_area: float
    cd0: float
    oswald: float
    cl_max: float
    cl_min: float = -0.3
    mu_max: float = 1.2
    n_max: float = 4.0
    rho: float = RHO_AIR

    @property
    def aspect_ratio(self):
        return self.span ** 2 / self.wing_area

    @property
    def wing_loading(self):
        return self.mass * G / self.wing_area

    @property
    def k_induced(self):
        return 1.0 / (np.pi * self.oswald * self.aspect_ratio)

    @property
    def cl_best_ld(self):
        return float(np.sqrt(self.cd0 / self.k_induced))

    @property
    def best_ld(self):
        return float(1.0 / (2.0 * np.sqrt(self.cd0 * self.k_induced)))

    def v_level(self, cl):
        return float(np.sqrt(2 * self.mass * G / (self.rho * self.wing_area * cl)))

    @property
    def v_stall(self):
        return self.v_level(self.cl_max)

    @property
    def v_best_ld(self):
        return self.v_level(self.cl_best_ld)

    @property
    def min_sink(self):
        """(speed, sink rate) at cl = sqrt(3 cd0 / k)."""
        cl = np.sqrt(3 * self.cd0 / self.k_induced)
        v = self.v_level(cl)
        cd = self.cd0 + self.k_induced * cl ** 2
        return v, float(v * cd / cl)


BASELINE = Aircraft("baseline-2m", mass=1.5, span=2.0, wing_area=0.25,
                    cd0=0.015, oswald=0.85, cl_max=1.2)

_H = 0.05
_DT = 0.01
_OFFS = np.array([[_H, 0, 0], [-_H, 0, 0], [0, _H, 0], [0, -_H, 0],
                  [0, 0, _H], [0, 0, -_H]])


def wind_derivs(env, x, y, z, t):
    """Return (W, grad W[i, j] = dW_i/dx_j, dW/dt) by central differences, one wind_at call.

    If z - h would fall to the roughness length z0 or below, the z derivative uses the
    one-sided pair (z + h, z) instead.
    """
    z0 = getattr(getattr(env, "wind", None), "z0", 0.0)
    low = z - _H <= z0
    zlo = z if low else z - _H
    px = np.concatenate([[x], x + _OFFS[:4, 0], [x, x], [x, x]])
    py = np.concatenate([[y], y + _OFFS[:4, 1], [y, y], [y, y]])
    pz = np.concatenate([[z], [z] * 4, [z + _H, zlo], [z, z]])
    pt = np.concatenate([[t] * 7, [t + _DT, t - _DT]])
    w = np.asarray(env.wind_at(px, py, pz, pt), dtype=float)
    grad = np.empty((3, 3))
    for j in range(2):
        grad[:, j] = (w[1 + 2 * j] - w[2 + 2 * j]) / (2 * _H)
    grad[:, 2] = (w[5] - w[6]) / ((_H if low else 2 * _H))
    dwdt = (w[7] - w[8]) / (2 * _DT)
    return w[0], grad, dwdt


def _unit_vectors(g, p):
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    return (np.array([cg * cp, cg * sp, sg]),
            np.array([-sg * cp, -sg * sp, cg]),
            np.array([-sp, cp, 0.0]))


def clip_controls(ac, cl, mu):
    return (min(max(cl, ac.cl_min), ac.cl_max), min(max(mu, -ac.mu_max), ac.mu_max))


def n_limited_cl(ac, V, cl):
    """cl reduced so that n = 0.5 rho V^2 S cl / (m g) does not exceed ac.n_max at airspeed V.

    Only the positive side is limited (the negative load factor is bounded by cl_min)."""
    q = 0.5 * ac.rho * V * V * ac.wing_area
    if not q > 0.0:
        return cl
    return min(cl, ac.n_max * ac.mass * G / q)


class Actuators:
    """Control surfaces between the commanded and the actual (cl, mu).

    Each step: the command is clipped to the aircraft limits, delayed by a FIFO of `delay_s`,
    the actual value moves toward it by a first-order lag (time constant tau, exact for a held
    command), and the move is limited to rate * dt. The first call starts at the command (no
    initial transient). Counts how often a rate limit reduced the move (n_mu_rate, n_cl_rate
    out of n_steps).

    rate_model "constant" (default, the old behaviour): bank rate limit ctrl.bank_rate_max_deg_s
    (150 deg/s), cl rate ctrl.const_cl_rate_max_per_s (2 /s), lag ctrl.const_actuator_tau_s (0.1 s),
    no delay.
    rate_model "pb2v": bank rate p_max = k 2 V / b with k = ctrl.roll_pb2v (0.08) and V the current
    airspeed (step needs V), cl rate ctrl.cl_rate_max_per_s (1 /s), lag ctrl.actuator_tau_s
    (0.05 s), delay ctrl.actuator_delay_s (0.05 s). Explicit arguments override the record.
    """

    def __init__(self, ac, tau=None, mu_rate_deg_s=None, cl_rate=None, rate_model="constant",
                 delay_s=None, k_pb2v=None):
        if rate_model not in ("constant", "pb2v"):
            raise ValueError("rate_model must be 'constant' or 'pb2v'")
        pb = rate_model == "pb2v"
        self.ac, self.rate_model = ac, rate_model
        self.tau = float((value("ctrl.actuator_tau_s", 0.05) if pb else value("ctrl.const_actuator_tau_s", 0.1))
                         if tau is None else tau)
        self.cl_rate = float((value("ctrl.cl_rate_max_per_s", 1.0) if pb
                              else value("ctrl.const_cl_rate_max_per_s", 2.0))
                             if cl_rate is None else cl_rate)
        self.mu_rate = np.radians(float(value("ctrl.bank_rate_max_deg_s", 150.0)
                                        if mu_rate_deg_s is None else mu_rate_deg_s))
        self.k_pb2v = float(value("ctrl.roll_pb2v", 0.08) if k_pb2v is None else k_pb2v)
        self.delay = float((value("ctrl.actuator_delay_s", 0.05) if pb else 0.0)
                           if delay_s is None else delay_s)
        self.reset()

    def reset(self):
        self.cl = None
        self.mu = None
        self.cmd_last = None
        self._q = None
        self.n_steps = 0
        self.n_mu_rate = 0
        self.n_cl_rate = 0

    def mu_rate_at(self, V):
        """Bank rate limit (rad/s) at airspeed V (array ok)."""
        if self.rate_model == "pb2v":
            return self.k_pb2v * 2.0 * np.asarray(V, dtype=float) / self.ac.span
        return self.mu_rate

    def step(self, cl_cmd, mu_cmd, dt, V=None):
        cl_cmd, mu_cmd = clip_controls(self.ac, cl_cmd, mu_cmd)
        self.n_steps += 1
        self.cmd_last = (float(cl_cmd), float(mu_cmd))
        if self._q is None:
            nd = int(round(self.delay / dt))
            self._q = deque([self.cmd_last] * nd)
        self._q.append(self.cmd_last)
        cl_cmd, mu_cmd = self._q.popleft()
        if self.cl is None:
            self.cl, self.mu = float(cl_cmd), float(mu_cmd)
            return self.cl, self.mu
        if self.rate_model == "pb2v" and V is None:
            raise ValueError("rate_model 'pb2v' needs the airspeed V")
        a = 1.0 - np.exp(-dt / self.tau) if self.tau > 0 else 1.0
        d_mu, d_cl = a * (mu_cmd - self.mu), a * (cl_cmd - self.cl)
        lim_mu, lim_cl = float(self.mu_rate_at(V)) * dt, self.cl_rate * dt
        if abs(d_mu) > lim_mu:
            d_mu = float(np.copysign(lim_mu, d_mu))
            self.n_mu_rate += 1
        if abs(d_cl) > lim_cl:
            d_cl = float(np.copysign(lim_cl, d_cl))
            self.n_cl_rate += 1
        self.mu += d_mu
        self.cl += d_cl
        return self.cl, self.mu

    def bind_fractions(self):
        n = max(self.n_steps, 1)
        return {"mu_rate": self.n_mu_rate / n, "cl_rate": self.n_cl_rate / n}


def ground_effect_factor(h_over_b, bank=0.0):
    """Induced drag in ground effect over free air, K(hc/b, bank): model "image" of
    sim.groundeffect (rolled elliptic lifting line over a plane, image method, tabulated).
    hc/b: centre height over the surface along the surface normal, in spans; bank (rad) relative
    to the surface. Complex-step safe. Kept in one function so the model can change."""
    from sim.groundeffect import induced_drag_ratio
    return induced_drag_ratio(h_over_b, bank)


def ground_effect_phi(h, span, mu, psi=0.0, ex=0.0, ey=0.0):
    """K for a centre height h above the local surface (vertical, m), bank mu (positive: left wing
    down, lift toward the left of the heading psi) and surface slope (ex, ey) = grad eta.

    Normal height h / sqrt(1 + ex^2 + ey^2); bank relative to the surface mu + atan(s_L) with s_L
    the surface slope along the left wing direction (-sin psi, cos psi). Flat surface: h, mu.
    The flight path angle is not used."""
    cn = 1.0 / np.sqrt(1.0 + ex * ex + ey * ey)
    s_left = -ex * np.sin(psi) + ey * np.cos(psi)
    return ground_effect_factor(h * cn / span, mu + np.arctan(s_left))


def derivatives(t, s, ac, env, cl, mu, n_limit=False, thrust=0.0, ground_effect=False):
    """Derivatives of the state. With n_limit, cl is first reduced to the load-factor limit at
    the current airspeed (n_limited_cl).

    thrust (N, default 0): along the air-relative velocity, adds thrust / m to dV/dt.
    ground_effect (default off): the induced drag is multiplied by ground_effect_phi at the
    height above env.surface (lift unchanged)."""
    x, y, z, V, g, p = s
    if n_limit:
        cl = n_limited_cl(ac, V, cl)
    eV, eg, ep = _unit_vectors(g, p)
    W, grad, dwdt = wind_derivs(env, x, y, z, t)
    vg = V * eV + W
    aW = dwdt + grad @ vg
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    if ground_effect:
        eta, ex, ey = env.surface(x, y, t)
        K = float(ground_effect_phi(z - float(eta), ac.span, mu, p, float(ex), float(ey)))
        D = q * (ac.cd0 + ac.k_induced * K * cl * cl)
    else:
        D = q * (ac.cd0 + ac.k_induced * cl * cl)
    m = ac.mass
    dV = -D / m - G * np.sin(g) - aW @ eV
    if thrust:
        dV = dV + thrust / m
    dg = (L * np.cos(mu) / m - G * np.cos(g) - aW @ eg) / V
    cg = max(abs(np.cos(g)), 0.05)  # floor near vertical flight
    dp = (L * np.sin(mu) / m - aW @ ep) / (V * cg)
    return np.array([vg[0], vg[1], vg[2], dV, dg, dp]), vg


def rk4_step(f, t, s, dt):
    k1 = f(t, s)
    k2 = f(t + dt / 2, s + dt / 2 * k1)
    k3 = f(t + dt / 2, s + dt / 2 * k2)
    k4 = f(t + dt, s + dt * k3)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def dumb_glide_ctrl(t, s, env, ac):
    return ac.cl_best_ld, 0.0


def simulate(ac, env, ctrl, s0, t0, t_end, dt=0.02, stop=None, n_limit=False):
    """Integrate with RK4 (controls evaluated at each stage). Returns dict of arrays."""
    if stop is None:
        def stop(t, s):
            return s[2] - float(env.surface(s[0], s[1], t)[0]) < 0.0

    def f(t, s):
        cl, mu = clip_controls(ac, *ctrl(t, s, env, ac))
        return derivatives(t, s, ac, env, cl, mu, n_limit)[0]

    rec = {k: [] for k in ("t", "x", "y", "z", "V", "gamma", "psi", "cl", "mu", "n",
                           "h", "e_air", "e_ground")}

    def record(t, s):
        cl, mu = clip_controls(ac, *ctrl(t, s, env, ac))
        _, vg = derivatives(t, s, ac, env, cl, mu, n_limit)
        if n_limit:
            cl = n_limited_cl(ac, s[3], cl)
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        vals = (t, s[0], s[1], s[2], s[3], s[4], s[5], cl, mu,
                q * cl / (ac.mass * G),
                s[2] - float(env.surface(s[0], s[1], t)[0]),
                s[2] + s[3] ** 2 / (2 * G),
                s[2] + float(vg @ vg) / (2 * G))
        for k, v in zip(rec, vals):
            rec[k].append(float(v))

    commit = getattr(ctrl, "commit", None)  # optional hook for stateful controllers
    s = np.asarray(s0, dtype=float)
    t = t0
    if commit:
        commit(t, s, env, ac)
    record(t, s)
    nsteps = int(round((t_end - t0) / dt))
    for i in range(nsteps):
        s = rk4_step(f, t, s, dt)
        t = t0 + (i + 1) * dt
        if commit:
            commit(t, s, env, ac)
        record(t, s)
        if stop(t, s) or not np.all(np.isfinite(s)):
            break
    return {k: np.array(v) for k, v in rec.items()}


def export_traj(result, path, name="TRAJ"):
    n = len(result["t"])
    idx = np.unique(np.linspace(0, n - 1, min(n, 3000)).astype(int))
    t = result["t"][idx]
    dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.0

    def fmt(a):
        return [float(f"{v:.4g}") for v in a[idx]]

    d = {"dt": float(f"{dt:.4g}"), "x": fmt(result["x"]), "y": fmt(result["y"]),
         "z": fmt(result["z"]), "psi": fmt(result["psi"]), "mu": fmt(result["mu"])}
    with open(path, "w", newline="\n") as fh:
        fh.write("window.%s = %s;\n" % (name, json.dumps(d, separators=(",", ":"))))
