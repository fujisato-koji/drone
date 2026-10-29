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
- Turbulence is optional (Environment(turbulence="iso")): frozen random Fourier
  field advected with the 10 m mean wind; default is none (gust returns zeros).

Constants (record keys in comments): see the module-level names below.

Changes:
- 2026-10-29 (check K-005): duration_limited uses the CEM 2002 friction-velocity
  forms with their own fully developed limits (was SPM-style forms in U10).
- 2026-10-29 (check K-005): fully_developed takes u19.5 from the log profile with
  the same u10, also under the swell_knee profile.
- 2026-10-29 (check K-005): turbulence lateral intensity sigma_v = 0.8 sigma_u
  (was equal to sigma_u); the 1-D spectrum caveat is documented.
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
DECAY_MEASURED = (0.77, 0.62)  # Grare et al. 2018 (record P63): (a_u, a_w) in exp(-a k zloc)
KNEE_DEFAULT_M = 7.5  # Smedman et al. 2008, record P65, 5 to 10 m
TURB_N_MODES = 64
TURB_L = 50.0  # von Karman length scale, m
TURB_K_RANGE = (2.0 * np.pi / 500.0, 2.0 * np.pi / 2.0)  # rad/m
TURB_W_RATIO = 0.5  # sigma_w / sigma_u (IEC 61400-1 Kaimal ratio; record P72, not read; to be verified, Q-008)
TURB_V_RATIO = 0.8  # sigma_v / sigma_u (IEC 61400-1 Kaimal ratio; record P72, not read; to be verified, Q-008)
CEM_HM0_FULL = 211.5  # g Hm0 / u*^2 at full development (CEM 2002, record P71, not read; Q-008)
CEM_TP_FULL = 239.8  # g Tp / u* at full development (CEM 2002, record P71, not read; Q-008)


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
    roughness: str = "charnock"
    hs: float | None = None
    tp: float | None = None
    sea: tuple | None = None
    profile: str = "log"  # "log" or "swell_knee"
    knee_m: float | None = None  # swell_knee: height above which speed is constant

    def __post_init__(self):
        """Roughness options (textbook forms, to be verified; Q-008).

        With Lp = g Tp^2 / (2 pi), cp = g Tp / (2 pi), and the smooth-flow
        term 0.11 nu / u* added in every option:
          "charnock" (default): z0 = charnock u*^2 / g
          "wave_age" (Taylor and Yelland 2001 steepness form):
                                z0 = 1200 Hs (Hs / Lp)^4.5
          "drennan"  (Drennan et al. 2003 wave-age form):
                                z0 = 3.35 Hs (u* / cp)^3.4
        The wave options need hs and tp (or sea=(hs, tp)) of the wind sea.
        """
        if not self.u10 > 0:
            raise ValueError("u10 must be > 0")
        if self.roughness not in ("charnock", "wave_age", "drennan"):
            raise ValueError("unknown roughness " + str(self.roughness))
        if self.profile not in ("log", "swell_knee"):
            raise ValueError("unknown profile " + str(self.profile))
        if self.profile == "swell_knee" and self.knee_m is None:
            self.knee_m = KNEE_DEFAULT_M
        if self.profile == "swell_knee" and not self.knee_m > 0:
            raise ValueError("knee_m must be > 0")
        # Height at which the speed equals u10 (the knee itself when it is below 10 m).
        zref = 10.0
        if self.profile == "swell_knee":
            zref = min(10.0, float(self.knee_m))
        if self.sea is not None:
            self.hs, self.tp = self.sea
        if self.roughness != "charnock" and not (
                self.hs is not None and self.tp is not None
                and self.hs > 0 and self.tp > 0):
            raise ValueError("wave roughness needs hs > 0 and tp > 0")
        self.to_deg = (self.from_deg + 180.0) % 360.0
        self.ex, self.ey = compass_to_vector(self.to_deg)
        ustar = 0.03 * self.u10 + 1e-3
        z0 = 0.0
        if self.roughness != "charnock":
            lp = G * self.tp**2 / (2.0 * np.pi)
            cp = G * self.tp / (2.0 * np.pi)
        for _ in range(200 if self.roughness != "charnock" else 50):
            if self.roughness == "charnock":
                z0 = self.charnock * ustar**2 / G + 0.11 * NU_AIR / ustar
            elif self.roughness == "wave_age":
                z0 = 1200.0 * self.hs * (self.hs / lp) ** 4.5 + 0.11 * NU_AIR / ustar
            else:
                z0 = 3.35 * self.hs * (ustar / cp) ** 3.4 + 0.11 * NU_AIR / ustar
            new = KARMAN * self.u10 / np.log(zref / z0)
            ustar = new if self.roughness == "charnock" else 0.5 * (ustar + new)
        self.ustar = float(ustar)
        self.z0 = float(z0)

    def speed(self, z):
        """Mean wind speed at height z above the local surface (0 for z <= z0)."""
        z = np.asarray(z, dtype=float)
        ok = z > self.z0
        zz = np.where(ok, z, 1.0)
        if self.profile == "swell_knee":
            zz = np.minimum(zz, self.knee_m)
        return np.where(ok, (self.ustar / KARMAN) * np.log(zz / self.z0), 0.0)

    @property
    def u19_5(self):
        return float(self.speed(19.5))

    def vector(self, z):
        """Mean wind vector, shape (..., 3)."""
        s = self.speed(z)
        return s[..., None] * np.array([self.ex, self.ey, 0.0])


def _log_wind(wind: Wind) -> Wind:
    """Log-profile Wind with the same u10, from_deg and charnock as `wind`."""
    if wind.profile == "log" and wind.roughness == "charnock":
        return wind
    return Wind(wind.u10, wind.from_deg, charnock=wind.charnock)


def cem_limits(u10: float):
    """(u*, Hm0 limit, Tp limit) of the CEM 2002 forms for a 10 m wind (record P71, Q-008)."""
    cd = 0.001 * (1.1 + 0.035 * u10)
    ust = float(np.sqrt(cd) * u10)
    return ust, CEM_HM0_FULL * ust**2 / G, CEM_TP_FULL * ust / G


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
    hs_full: float = 0.0  # fully developed Hs for the same wind (0 = unknown)

    @property
    def growth_fraction(self):
        """hs / fully developed hs (1.0 when the reference is unknown)."""
        return self.hs / self.hs_full if self.hs_full > 0 else 1.0

    @classmethod
    def fully_developed(cls, wind: Wind, **kw):
        """Pierson-Moskowitz fully developed sea for wind.u10.

        u19.5 is taken from the log profile with the same u10, from_deg and
        charnock, whatever the profile of `wind` (the swell_knee profile would
        otherwise make u19.5 equal to u10 and the sea about 11% too low).
        """
        u = _log_wind(wind).u19_5
        hs = PM_HS * u**2 / G
        kw.setdefault("hs_full", hs)
        return cls(hs=hs, tp=2.0 * np.pi * u / (PM_PEAK * G),
                   to_deg=wind.to_deg, gamma=1.0, **kw)

    @classmethod
    def duration_limited(cls, wind: Wind, hours: float, **kw):
        """JONSWAP wind sea grown for `hours` under wind.u10, capped at full development.

        Coastal Engineering Manual (USACE 2002, EM 1110-2-1100, Part II
        chapter 2, Change 1 2003) deep-water forms in friction velocity
        (record P71, equations checked against the printed pages II-2-44 and
        II-2-45, check K-006). With U = U10 in m/s:
          u*^2 = Cd U^2,  Cd = 0.001 (1.1 + 0.035 U)
          X* = g X / u*^2
          g Hm0 / u*^2 = 4.13e-2 X*^(1/2)
          g Tp / u*    = 0.651 X*^(1/3)
          duration to equivalent fetch (II-2-38):  X* = 5.23e-3 (g t / u*)^(3/2)
          fully developed limits (II-2-37):  g Hm0 / u*^2 = 211.5,  g Tp / u* = 239.8
        CEM warns that duration-limited growth is rarely met in nature and
        recommends numerical wave models where safety matters (p. II-2-43, 44).
        Hs (= Hm0) and Tp are each capped at these limits (not at the
        Pierson-Moskowitz values of `fully_developed`). Gamma is the JONSWAP
        default while not both are capped, and 1.0 once both are capped.
        hs_full defaults to the CEM Hs limit.
        """
        if hours <= 0:
            raise ValueError("hours must be > 0")
        ust, hs_cap, tp_cap = cem_limits(wind.u10)
        xstar = 5.23e-3 * (G * hours * 3600.0 / ust) ** 1.5
        hs = 4.13e-2 * np.sqrt(xstar) * ust**2 / G
        tp = 0.651 * xstar ** (1.0 / 3.0) * ust / G
        capped = hs >= hs_cap and tp >= tp_cap
        hs = min(hs, hs_cap)
        tp = min(tp, tp_cap)
        kw.setdefault("hs_full", hs_cap)
        return cls(hs=float(hs), tp=float(tp), to_deg=wind.to_deg,
                   gamma=1.0 if capped else JONSWAP_GAMMA, **kw)

    @staticmethod
    def equivalent_hours(u10: float, hs: float):
        """Duration (h) at which duration_limited(u10, .) reaches hs (inverse of the Hs law).

        Returns inf when hs is at or above the CEM limit for u10.
        """
        ust, hs_cap, _ = cem_limits(u10)
        if hs <= 0:
            return 0.0
        if hs >= hs_cap:
            return float("inf")
        xstar = (hs * G / (4.13e-2 * ust**2)) ** 2
        return float((xstar / 5.23e-3) ** (2.0 / 3.0) * ust / G / 3600.0)

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

    stokes=True adds the second-order self-interaction term of each component
    to surface() only. orbital() and wind_at() keep the linear terms.
    Cross-component second-order terms are omitted.
    """

    def __init__(self, wind: Wind, trains=(), seed=1, stokes=False,
                 decay=(1.0, 1.0), turbulence=None):
        self.wind = wind
        self.stokes = bool(stokes)
        self.decay = (float(decay[0]), float(decay[1]))
        if turbulence not in (None, "iso"):
            raise ValueError("unknown turbulence " + str(turbulence))
        self.turbulence = turbulence
        self._turb = self._make_turbulence(seed) if turbulence == "iso" else None
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

    @staticmethod
    def _make_turbulence(seed):
        """Modes of the frozen field. Own generator, so wave phases are unchanged.

        Longitudinal (along mean wind), lateral and vertical fluctuations are
        independent sums over the same wavenumbers and amplitudes, with
        independent phases; unit variance each (vertical scaled later).
        Amplitude per mode ~ sqrt(E(k) k) (log-uniform sampling), von Karman
        E(k) ~ (kL)^4 / (1 + (kL)^2)^(17/6). Not divergence-free (simplest choice).
        The 3-D von Karman energy spectrum is applied to horizontal wavenumbers,
        so the 1-D longitudinal spectrum is not exactly von Karman; this is
        accepted for the form study. Intensities: sigma_v = 0.8 sigma_u and
        sigma_w = 0.5 sigma_u (IEC 61400-1 Kaimal ratios; record P72, not read;
        to be verified, Q-008).
        """
        rng = np.random.default_rng(np.random.SeedSequence([int(seed), 12345]))
        n = TURB_N_MODES
        lo, hi = np.log(TURB_K_RANGE[0]), np.log(TURB_K_RANGE[1])
        k = np.exp(rng.uniform(lo, hi, n))
        ang = rng.uniform(0.0, 2.0 * np.pi, n)
        kl = k * TURB_L
        amp2 = kl**4 / (1.0 + kl**2) ** (17.0 / 6.0) * k
        amp = np.sqrt(2.0 * amp2 / amp2.sum())  # sum(amp^2 / 2) = 1
        return {"kx": k * np.sin(ang), "ky": k * np.cos(ang), "amp": amp,
                "phase": rng.uniform(0.0, 2.0 * np.pi, (3, n))}

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
            if self.stokes:
                # eta2 = 0.5 a^2 k cos(2 theta); slope term -a^2 k^2 d sin(2 theta)
                k = c["k"][sl]
                eta += (0.5 * a**2 * k * np.cos(2.0 * th)).sum(-1)
                s2 = a**2 * k**2 * np.sin(2.0 * th)
                ex -= (s2 * c["dx"][sl]).sum(-1)
                ey -= (s2 * c["dy"][sl]).sum(-1)
        return eta, ex, ey

    def breaking(self, x, y, t, limit=0.3):
        """True where the surface slope magnitude exceeds `limit`.

        The slope threshold is an assumed steepness criterion. A real criterion
        would use the crest speed against the particle speed, or a*k > 0.44;
        to be sourced.
        """
        _, sx, sy = self.surface(x, y, t)
        return np.sqrt(sx**2 + sy**2) > limit

    def whitecap_fraction(self, t, n=4000, seed=0):
        """Fraction of n random points over a 1 km square that are breaking at t."""
        rng = np.random.default_rng(seed)
        x = rng.uniform(0.0, 1000.0, n)
        y = rng.uniform(0.0, 1000.0, n)
        return float(np.mean(self.breaking(x, y, np.full(n, float(t)))))

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
        """Turbulent velocity (east, north, up), shape (..., 3); zeros unless turbulence="iso"."""
        x, y, z, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, z, t)))
        out = np.zeros(x.shape + (3,))
        if self._turb is None or out.size == 0:
            return out
        w0 = self.wind
        m = self._turb
        xs = x - w0.u10 * w0.ex * t
        ys = y - w0.u10 * w0.ey * t
        zc = np.maximum(z, 2.0 * w0.z0)
        iu = 0.06 * (1.0 + 0.043 * w0.u10) * (zc / 10.0) ** -0.22
        sig = iu * w0.speed(zc)
        fx, fy = xs.ravel(), ys.ravel()
        lon = np.empty(fx.size)
        lat = np.empty(fx.size)
        ver = np.empty(fx.size)
        for i in range(0, fx.size, 4096):
            sl = slice(i, i + 4096)
            ph = m["kx"] * fx[sl, None] + m["ky"] * fy[sl, None]
            lon[sl] = (m["amp"] * np.cos(ph + m["phase"][0])).sum(-1)
            lat[sl] = (m["amp"] * np.cos(ph + m["phase"][1])).sum(-1)
            ver[sl] = (m["amp"] * np.cos(ph + m["phase"][2])).sum(-1)
        lon = lon.reshape(x.shape) * sig
        lat = lat.reshape(x.shape) * sig * TURB_V_RATIO
        ver = ver.reshape(x.shape) * sig * TURB_W_RATIO
        # longitudinal axis = mean wind direction (ex, ey); lateral = +90 deg
        out[..., 0] = lon * w0.ex - lat * w0.ey
        out[..., 1] = lon * w0.ey + lat * w0.ex
        out[..., 2] = ver
        return out

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
            base = self.urel[sl] * c["a"][sl] * c["k"][sl]
            kz = c["k"][sl] * zl[..., None]
            amp = base * np.exp(-self.decay[0] * kz)
            ampw = base * np.exp(-self.decay[1] * kz)
            up = amp * np.cos(th)
            out[..., 0] += (up * c["dx"][sl]).sum(-1)
            out[..., 1] += (up * c["dy"][sl]).sum(-1)
            out[..., 2] += (-ampw * np.sin(th)).sum(-1)
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
                                      "decaying as exp(-k*zloc)",
                      "decay": list(self.decay), "profile": w.profile,
                      "knee_m": w.knee_m, "turbulence": self.turbulence},
        }

    def to_js(self, path, name="FIELD"):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(f"window.{name} = {json.dumps(self.to_dict())};\n")
