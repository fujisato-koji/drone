"""Point-mass glider flight model in a wind field."""
import json
from dataclasses import dataclass

import numpy as np

try:
    from sim.seawind import RHO_AIR
except ImportError:  # seawind not written yet: fall back to the stand-in
    from sim._seawind_stub import RHO_AIR

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
    """Return (W, grad W[i, j] = dW_i/dx_j, dW/dt) by central differences, one wind_at call."""
    px = np.concatenate([[x], x + _OFFS[:, 0], [x, x]])
    py = np.concatenate([[y], y + _OFFS[:, 1], [y, y]])
    pz = np.concatenate([[z], z + _OFFS[:, 2], [z, z]])
    pt = np.concatenate([[t] * 7, [t + _DT, t - _DT]])
    w = np.asarray(env.wind_at(px, py, pz, pt), dtype=float)
    grad = np.empty((3, 3))
    for j in range(3):
        grad[:, j] = (w[1 + 2 * j] - w[2 + 2 * j]) / (2 * _H)
    dwdt = (w[7] - w[8]) / (2 * _DT)
    return w[0], grad, dwdt


def _unit_vectors(g, p):
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    return (np.array([cg * cp, cg * sp, sg]),
            np.array([-sg * cp, -sg * sp, cg]),
            np.array([-sp, cp, 0.0]))


def clip_controls(ac, cl, mu):
    return (min(max(cl, ac.cl_min), ac.cl_max), min(max(mu, -ac.mu_max), ac.mu_max))


def derivatives(t, s, ac, env, cl, mu):
    x, y, z, V, g, p = s
    eV, eg, ep = _unit_vectors(g, p)
    W, grad, dwdt = wind_derivs(env, x, y, z, t)
    vg = V * eV + W
    aW = dwdt + grad @ vg
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    D = q * (ac.cd0 + ac.k_induced * cl * cl)
    m = ac.mass
    dV = -D / m - G * np.sin(g) - aW @ eV
    dg = (L * np.cos(mu) / m - G * np.cos(g) - aW @ eg) / V
    dp = (L * np.sin(mu) / m - aW @ ep) / (V * np.cos(g))
    return np.array([vg[0], vg[1], vg[2], dV, dg, dp]), vg


def rk4_step(f, t, s, dt):
    k1 = f(t, s)
    k2 = f(t + dt / 2, s + dt / 2 * k1)
    k3 = f(t + dt / 2, s + dt / 2 * k2)
    k4 = f(t + dt, s + dt * k3)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def dumb_glide_ctrl(t, s, env, ac):
    return ac.cl_best_ld, 0.0


def simulate(ac, env, ctrl, s0, t0, t_end, dt=0.02, stop=None):
    """Integrate with RK4 (controls evaluated at each stage). Returns dict of arrays."""
    if stop is None:
        def stop(t, s):
            return s[2] - float(env.surface(s[0], s[1], t)[0]) < 0.0

    def f(t, s):
        cl, mu = clip_controls(ac, *ctrl(t, s, env, ac))
        return derivatives(t, s, ac, env, cl, mu)[0]

    rec = {k: [] for k in ("t", "x", "y", "z", "V", "gamma", "psi", "cl", "mu", "n",
                           "h", "e_air", "e_ground")}

    def record(t, s):
        cl, mu = clip_controls(ac, *ctrl(t, s, env, ac))
        _, vg = derivatives(t, s, ac, env, cl, mu)
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        vals = (t, s[0], s[1], s[2], s[3], s[4], s[5], cl, mu,
                q * cl / (ac.mass * G),
                s[2] - float(env.surface(s[0], s[1], t)[0]),
                s[2] + s[3] ** 2 / (2 * G),
                s[2] + float(vg @ vg) / (2 * G))
        for k, v in zip(rec, vals):
            rec[k].append(float(v))

    s = np.asarray(s0, dtype=float)
    t = t0
    record(t, s)
    nsteps = int(round((t_end - t0) / dt))
    for i in range(nsteps):
        s = rk4_step(f, t, s, dt)
        t = t0 + (i + 1) * dt
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
