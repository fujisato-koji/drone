"""Sea-surface and wind model for the form study of a dynamic soaring drone.

Coordinates: x east, y north, z up, metres, seconds. Compass degrees run
clockwise from north (0 = N, 90 = E); the unit vector toward compass angle d
is (sin d, cos d) in (east, north). A wind direction is where the wind blows
FROM; a wave direction is where the waves travel TO.

Model assumptions:
- Mean wind: neutral logarithmic profile over the local surface, with
  Charnock roughness (smooth-flow term included) solved by fixed-point
  iteration from the 10 m wind speed.
- Wave-coherent wind perturbation: linear potential flow over each wave
  component, decaying as exp(-k * zloc), with zloc the height above the
  local (moving) surface.
- Waves: linear superposition of JONSWAP (or Pierson-Moskowitz when
  gamma = 1) components with cos^2 directional spreading, deep water.
- No turbulence yet (see Environment.gust for the hook).

Constants (record keys in comments): see the module-level names below.
"""

from __future__ import annotations

import datetime
import json
from dataclasses import dataclass

import numpy as np

G = 9.81
KARMAN = 0.40  # env.karman
CHARNOCK = 0.0144  # env.charnock
NU_AIR = 1.46e-5  # env.air_kinematic_viscosity
JONSWAP_GAMMA = 3.3  # env.jonswap_gamma
PM_HS = 0.21  # env.pm_hs_coefficient
PM_PEAK = 0.877  # env.pm_peak_coefficient
RHO_AIR = 1.225


def compass_to_vector(deg):
    """Unit vector (east, north) toward compass angle deg."""
    r = np.radians(deg)
    return float(np.sin(r)), float(np.cos(r))


@dataclass
class Wind:
    """Neutral log wind profile. u10 in m/s (> 0), from_deg = direction wind blows from."""

    u10: float
    from_deg: float = 270.0
    charnock: float = CHARNOCK

    def __post_init__(self):
        if not self.u10 > 0:
            raise ValueError("u10 must be > 0")
        self.to_deg = (self.from_deg + 180.0) % 360.0
        self.ex, self.ey = compass_to_vector(self.to_deg)
        ustar = 0.03 * self.u10 + 1e-3
        z0 = 0.0
        for _ in range(50):
            z0 = self.charnock * ustar**2 / G + 0.11 * NU_AIR / ustar
            ustar = KARMAN * self.u10 / np.log(10.0 / z0)
        self.ustar = float(ustar)
        self.z0 = float(z0)

    def speed(self, z):
        """Mean wind speed at height z above the local surface (0 for z <= z0)."""
        z = np.asarray(z, dtype=float)
        ok = z > self.z0
        zz = np.where(ok, z, 1.0)
        return np.where(ok, (self.ustar / KARMAN) * np.log(zz / self.z0), 0.0)

    @property
    def u19_5(self):
        return float(self.speed(19.5))

    def vector(self, z):
        """Mean wind vector, shape (..., 3)."""
        s = self.speed(z)
        return s[..., None] * np.array([self.ex, self.ey, 0.0])


def jonswap(omega, hs, tp, gamma=JONSWAP_GAMMA):
    """JONSWAP spectrum S(omega); gamma = 1 gives Pierson-Moskowitz."""
    omega = np.asarray(omega, dtype=float)
    wp = 2.0 * np.pi / tp
    sigma = np.where(omega <= wp, 0.07, 0.09)
    r = np.exp(-((omega - wp) ** 2) / (2.0 * sigma**2 * wp**2))
    a_gamma = 1.0 - 0.287 * np.log(gamma)
    return (a_gamma * (5.0 / 16.0) * hs**2 * wp**4 * omega**-5
            * np.exp(-1.25 * (wp / omega) ** 4) * gamma**r)


@dataclass
class WaveTrain:
    hs: float
    tp: float
    to_deg: float
    gamma: float = JONSWAP_GAMMA
    n_omega: int = 12
    n_theta: int = 8
    name: str = "wind sea"

    @classmethod
    def fully_developed(cls, wind: Wind, **kw):
        u = wind.u19_5
        return cls(hs=PM_HS * u**2 / G, tp=2.0 * np.pi * u / (PM_PEAK * G),
                   to_deg=wind.to_deg, gamma=1.0, **kw)

    def components(self, rng):
        wp = 2.0 * np.pi / self.tp
        om = np.linspace(0.5 * wp, 4.0 * wp, 2000)
        s = jonswap(om, self.hs, self.tp, self.gamma)
        # cumulative energy (trapezoid) at grid points
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * np.diff(om))])
        m0 = cum[-1]
        idx = np.minimum((cum / m0 * self.n_omega).astype(int), self.n_omega - 1)
        w_om = np.bincount(idx, weights=s * om, minlength=self.n_omega)
        w_s = np.bincount(idx, weights=s, minlength=self.n_omega)
        omega_b = w_om / w_s
        th = (self.to_deg - 90.0
              + (np.arange(self.n_theta) + 0.5) * 180.0 / self.n_theta)
        d = np.cos(np.radians(th - self.to_deg)) ** 2
        d = d / d.sum()
        om2, th2 = np.meshgrid(omega_b, th, indexing="ij")
        d2 = np.broadcast_to(d, (self.n_omega, self.n_theta))
        a = np.sqrt(2.0 * (m0 / self.n_omega) * d2).ravel()
        omega = om2.ravel()
        vec = np.array([compass_to_vector(t) for t in th2.ravel()])
        phase = rng.uniform(0.0, 2.0 * np.pi, a.size)
        return {"a": a, "k": omega**2 / G, "omega": omega,
                "dx": vec[:, 0], "dy": vec[:, 1], "phase": phase}


def _r6(arr):
    return [float(f"{v:.6g}") for v in np.asarray(arr, dtype=float).ravel()]


class Environment:
    """Wind plus wave field.

    urel per component = wind.speed(zref) * (ex*dx + ey*dy) - c, with
    c = omega/k and zref = max(1/k, 2*z0). Assumption: zref is the e-folding
    height of the component's perturbation.
    """

    def __init__(self, wind: Wind, trains=(), seed=1):
        self.wind = wind
        self.trains = list(trains)
        self.seed = seed
        rng = np.random.default_rng(seed)
        keys = ("a", "k", "omega", "dx", "dy", "phase")
        parts = [t.components(rng) for t in self.trains]
        if parts:
            self.c = {key: np.concatenate([p[key] for p in parts]) for key in keys}
        else:
            self.c = {key: np.zeros(0) for key in keys}
        self._train_slices = []
        i = 0
        for p in parts:
            self._train_slices.append(slice(i, i + p["a"].size))
            i += p["a"].size
        k = self.c["k"]
        if k.size:
            zref = np.maximum(1.0 / k, 2.0 * wind.z0)
            along = wind.ex * self.c["dx"] + wind.ey * self.c["dy"]
            self.urel = wind.speed(zref) * along - self.c["omega"] / k
        else:
            self.urel = np.zeros(0)

    @classmethod
    def flat(cls, wind):
        return cls(wind, [])

    @property
    def n_components(self):
        return int(self.c["a"].size)

    _CHUNK = 16

    def _chunks(self):
        n = self.n_components
        for i in range(0, n, self._CHUNK):
            yield slice(i, min(i + self._CHUNK, n))

    def _theta(self, sl, x, y, t):
        c = self.c
        return (c["k"][sl] * (c["dx"][sl] * x[..., None] + c["dy"][sl] * y[..., None])
                - c["omega"][sl] * t[..., None] + c["phase"][sl])

    def surface(self, x, y, t):
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        eta = np.zeros(x.shape)
        ex = np.zeros(x.shape)
        ey = np.zeros(x.shape)
        c = self.c
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            a = c["a"][sl]
            eta += (a * np.cos(th)).sum(-1)
            s = a * c["k"][sl] * np.sin(th)
            ex -= (s * c["dx"][sl]).sum(-1)
            ey -= (s * c["dy"][sl]).sum(-1)
        return eta, ex, ey

    def orbital(self, x, y, t):
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        u = np.zeros(x.shape)
        v = np.zeros(x.shape)
        w = np.zeros(x.shape)
        c = self.c
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            ao = c["a"][sl] * c["omega"][sl]
            cs = ao * np.cos(th)
            u += (cs * c["dx"][sl]).sum(-1)
            v += (cs * c["dy"][sl]).sum(-1)
            w += (ao * np.sin(th)).sum(-1)
        return u, v, w

    def gust(self, x, y, z, t):
        """Hook for turbulence; returns zeros (shape (..., 3)) for now."""
        shape = np.broadcast(*(np.asarray(v, float) for v in (x, y, z, t))).shape
        return np.zeros(shape + (3,))

    def wind_at(self, x, y, z, t):
        x, y, z, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, z, t)))
        w0 = self.wind
        eta, _, _ = self.surface(x, y, t)
        zloc = z - eta
        above = zloc > w0.z0
        zl = np.where(above, zloc, w0.z0 * 2.0)
        out = w0.vector(zl)
        c = self.c
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            amp = (self.urel[sl] * c["a"][sl] * c["k"][sl]
                   * np.exp(-c["k"][sl] * zl[..., None]))
            up = amp * np.cos(th)
            out[..., 0] += (up * c["dx"][sl]).sum(-1)
            out[..., 1] += (up * c["dy"][sl]).sum(-1)
            out[..., 2] += (-amp * np.sin(th)).sum(-1)
        out = out + self.gust(x, y, z, t)
        if not above.all():
            u, v, w = self.orbital(x, y, t)
            orb = np.stack([u, v, w], axis=-1)
            out = np.where(above[..., None], out, orb)
        return out

    def profile(self, z, x=0.0, y=0.0, t=0.0):
        return self.wind_at(x, y, np.asarray(z, float), t)

    def to_dict(self):
        w = self.wind
        c = self.c
        trains = []
        for tr, sl in zip(self.trains, self._train_slices):
            trains.append({
                "name": tr.name, "hs": float(f"{tr.hs:.6g}"), "tp": float(f"{tr.tp:.6g}"),
                "to_deg": float(f"{tr.to_deg:.6g}"), "gamma": tr.gamma,
                "n_omega": tr.n_omega, "n_theta": tr.n_theta,
                "hs_discrete": float(f"{4.0 * np.sqrt(np.sum(c['a'][sl] ** 2 / 2.0)):.6g}"),
            })
        comps = {key: _r6(c[key]) for key in ("a", "k", "omega", "dx", "dy", "phase")}
        comps["urel"] = _r6(self.urel)
        return {
            "meta": {"generated": datetime.date.today().isoformat(),
                     "seed": self.seed, "n_components": self.n_components},
            "constants": {"g": G, "karman": KARMAN, "charnock": w.charnock,
                          "nu_air": NU_AIR, "rho_air": RHO_AIR},
            "wind": {"u10": w.u10, "from_deg": w.from_deg, "to_deg": w.to_deg,
                     "ex": float(f"{w.ex:.6g}"), "ey": float(f"{w.ey:.6g}"),
                     "ustar": float(f"{w.ustar:.6g}"), "z0": float(f"{w.z0:.6g}")},
            "trains": trains,
            "components": comps,
            "model": {"zref": "max(1/k, 2*z0)", "noslip_below_z0": True,
                      "perturbation": "linear potential flow over each component, "
                                      "decaying as exp(-k*zloc)"},
        }

    def to_js(self, path, name="FIELD"):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(f"window.{name} = {json.dumps(self.to_dict())};\n")
