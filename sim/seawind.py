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

- Optional (opt-in, Environment keywords; defaults reproduce the plain model):
  profile_mode="auto" picks the mean profile from the wave age (log to swell
  knee, Hanley et al. 2010); shelter=True adds a crude airflow-separation
  stand-in behind steep crests (see the Environment docstring).

Changes:
- 2026-10-29: Environment(profile_mode="auto") and Environment(shelter=True), both opt-in.
- 2026-10-29 (check K-005): duration_limited uses the CEM 2002 friction-velocity
  forms with their own fully developed limits (was SPM-style forms in U10).
- 2026-10-29 (check K-005): fully_developed takes u19.5 from the log profile with
  the same u10, also under the swell_knee profile.
- 2026-10-29 (check K-005): turbulence lateral intensity sigma_v = 0.8 sigma_u
  (was equal to sigma_u); the 1-D spectrum caveat is documented.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from sim.record import commit, stamp, value

# Keyed constants are read from the record (record/values.yaml); the default is the number used so far.
G = 9.81
KARMAN = value("env.karman", 0.40)
CHARNOCK = value("env.charnock", 0.0144)
# COARE 3.5 Charnock parameter (Edson et al. 2013, JPO 43, 1589, eq. 11 and 13): alpha = clip(m U10N + b, 0, a_max).
# The printed slope 0.017 is a typo; 0.0017 matches their Fig. 6 and the NOAA code.
COARE35_SLOPE = value("env.coare35_charnock_slope", 0.0017)  # per m/s
COARE35_INTERCEPT = value("env.coare35_charnock_intercept", -0.005)
COARE35_MAX = value("env.coare35_charnock_max", 0.028)
NU_AIR = value("env.air_kinematic_viscosity", 1.46e-5)
JONSWAP_GAMMA = value("env.jonswap_gamma", 3.3)
PM_HS = value("env.pm_hs_coefficient", 0.21)
PM_PEAK = value("env.pm_peak_coefficient", 0.877)
RHO_AIR = 1.225
DECAY_MEASURED = (value("lit.wave_induced_decay_u", 0.77),
                  value("lit.wave_induced_decay_w", 0.62))  # Grare et al. 2018 (record P63): (a_u, a_w) in exp(-a k zloc)
KNEE_DEFAULT_M = value("lit.swell_knee_height_m", 7.5)  # Smedman et al. 2008, record P65, 5 to 10 m
# Hanley, Belcher and Sullivan 2010 inverse wave age U10/cp: wind-driven above the first, wave-driven below the second.
# Record source to be added by the main session; thresholds to be verified (Q-008).
A_WIND_DRIVEN = value("env.inverse_wave_age_wind_driven", 0.83)
A_WAVE_DRIVEN = value("env.inverse_wave_age_wave_driven", 0.15)
# Sheltering stand-in: assumed constants, to be verified (Q-008, Q-011).
# Onset slope from the 50% separation contour of Buckley, Veron and Yousefi 2020 (JFM 905 A31, eq. 4.8).
SEP_GAMMA_FLOOR = value("env.separation_gamma_floor", 0.1)
SEP_GAMMA_SPAN = value("env.separation_gamma_span", 0.3)
SEP_USTAR_C_REF = value("env.separation_ustar_c_ref", 0.3)
SEP_EXPONENT = value("env.separation_exponent", 2.5)
SHELTER_RAMP_RATIO = value("env.shelter_ramp_ratio", 1.5)  # r = 0.5 at the 50% contour; assumed (Q-008)
SHELTER_FACTOR = value("env.shelter_factor", 1.0)
SHELTER_DECAY_KZ = value("env.shelter_decay_kz", 0.5)  # reduction decays as exp(-k zloc / this); assumed
TURB_N_MODES = 64
TURB_L = value("env.turb_length_m", 50.0)  # von Karman length scale, m
TURB_K_RANGE = (2.0 * np.pi / 500.0, 2.0 * np.pi / 2.0)  # rad/m
TURB_W_RATIO = value("env.turb_w_ratio", 0.5)  # sigma_w / sigma_u (IEC 61400-1 Kaimal ratio; record P72, not read; to be verified, Q-008)
TURB_V_RATIO = value("env.turb_v_ratio", 0.8)  # sigma_v / sigma_u (IEC 61400-1 Kaimal ratio; record P72, not read; to be verified, Q-008)
CEM_HM0_FULL = value("env.cem_full_hm0_limit", 211.5)  # g Hm0 / u*^2 at full development (CEM 2002, record P71, not read; Q-008)
CEM_TP_FULL = value("env.cem_full_tp_limit", 239.8)  # g Tp / u* at full development (CEM 2002, record P71; Q-008)
CEM_HM0_COEFF = value("env.cem_hm0_coeff", 4.13e-2)  # g Hm0 / u*^2 = coeff X*^(1/2)
CEM_TP_COEFF = value("env.cem_tp_coeff", 0.651)  # g Tp / u* = coeff X*^(1/3)
CEM_DURATION_COEFF = value("env.cem_duration_coeff", 5.23e-3)  # X* = coeff (g t / u*)^(3/2)
CD_LAW_A = value("env.cd_law_a", 1.1)  # Cd = 0.001 (a + b U10)
CD_LAW_B = value("env.cd_law_b", 0.035)


def separation_steepness(ustar, c):
    """Onset slope s_c = gamma50 / R of the sheltering ramp (numpy arrays accepted).

    gamma50 = g0 + span / (1 + ((ustar/c)/ref)^p), the 50% separation contour of
    Buckley, Veron and Yousefi 2020 (JFM 905 A31, eq. 4.8); R is
    env.shelter_ramp_ratio, so that r = 0.5 exactly at gamma50.
    """
    q = np.asarray(ustar, dtype=float) / np.asarray(c, dtype=float)
    g50 = SEP_GAMMA_FLOOR + SEP_GAMMA_SPAN / (1.0 + (q / SEP_USTAR_C_REF) ** SEP_EXPONENT)
    return g50 / SHELTER_RAMP_RATIO


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
          "coare35" (Edson et al. 2013): z0 = alpha u*^2 / g with
                                alpha = clip(m U10 + b, 0, a_max), U10 as U10N
          "wave_age" (Taylor and Yelland 2001 steepness form):
                                z0 = 1200 Hs (Hs / Lp)^4.5
          "drennan"  (Drennan et al. 2003 wave-age form):
                                z0 = 3.35 Hs (u* / cp)^3.4
        The wave options need hs and tp (or sea=(hs, tp)) of the wind sea.
        """
        if not self.u10 > 0:
            raise ValueError("u10 must be > 0")
        if self.roughness not in ("charnock", "coare35", "wave_age", "drennan"):
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
        wind_only = self.roughness in ("charnock", "coare35")
        if not wind_only and not (
                self.hs is not None and self.tp is not None
                and self.hs > 0 and self.tp > 0):
            raise ValueError("wave roughness needs hs > 0 and tp > 0")
        self.to_deg = (self.from_deg + 180.0) % 360.0
        self.ex, self.ey = compass_to_vector(self.to_deg)
        ustar = 0.03 * self.u10 + 1e-3
        z0 = 0.0
        alpha = self.charnock
        if self.roughness == "coare35":
            alpha = float(np.clip(COARE35_SLOPE * self.u10 + COARE35_INTERCEPT, 0.0, COARE35_MAX))
        self.alpha = alpha
        if not wind_only:
            lp = G * self.tp**2 / (2.0 * np.pi)
            cp = G * self.tp / (2.0 * np.pi)
        for _ in range(50 if wind_only else 200):
            if wind_only:
                z0 = alpha * ustar**2 / G + 0.11 * NU_AIR / ustar
            elif self.roughness == "wave_age":
                z0 = 1200.0 * self.hs * (self.hs / lp) ** 4.5 + 0.11 * NU_AIR / ustar
            else:
                z0 = 3.35 * self.hs * (ustar / cp) ** 3.4 + 0.11 * NU_AIR / ustar
            new = KARMAN * self.u10 / np.log(zref / z0)
            ustar = new if wind_only else 0.5 * (ustar + new)
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
    if wind.profile == "log" and wind.roughness in ("charnock", "coare35"):
        return wind
    return Wind(wind.u10, wind.from_deg, charnock=wind.charnock)


def cem_limits(u10: float):
    """(u*, Hm0 limit, Tp limit) of the CEM 2002 forms for a 10 m wind (record P71, Q-008)."""
    cd = 0.001 * (CD_LAW_A + CD_LAW_B * u10)
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
        xstar = CEM_DURATION_COEFF * (G * hours * 3600.0 / ust) ** 1.5
        hs = CEM_HM0_COEFF * np.sqrt(xstar) * ust**2 / G
        tp = CEM_TP_COEFF * xstar ** (1.0 / 3.0) * ust / G
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
        xstar = (hs * G / (CEM_HM0_COEFF * ust**2)) ** 2
        return float((xstar / CEM_DURATION_COEFF) ** (2.0 / 3.0) * ust / G / 3600.0)

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

    profile_mode="fixed" (default) uses wind.speed as the mean profile.
    profile_mode="auto" mixes the log and the swell_knee profiles by the wave
    age of the most energetic train (largest sum of a^2), U = (1 - f) U_log +
    f U_knee, both with U(10) = u10. With cp = g Tp / 2 pi of that train and
    A = U10 cos(theta) / cp (theta: wind to train direction), f is a smoothstep from A = 0.83 (f = 0, wind-driven) down to
    A = 0.15 (f = 1, wave-driven), the thresholds of Hanley et al. 2010 (record
    source to be added). No trains: f = 0. The turbulence intensity (gust)
    keeps using wind.speed.

    shelter=True (default off) is a crude stand-in for airflow separation
    behind steep crests, motivated by Buckley et al. 2025 (record P64) and the
    classical sheltering idea; the constants are assumed, to be verified
    (Q-008, Q-011). The trigger is the local slope of the summed surface, so
    it acts on a spectral sea whose single components are all gentle. Let
    w_hat be the horizontal unit vector the wind blows towards, and count
    only the components the wind outruns (urel > 0). Their linear slope along
    the wind is s = sum grad(eta_i) . w_hat at (x, y, t) (Stokes term
    ignored), the lee steepness is sigma = max(0, -s), and
    r = clip((sigma - s_c) / s_c, 0, 1), with s_c = separation_steepness(
    ustar, c_s) (ustar of the mean wind, c_s = sqrt(g / k_s)); shelter_slope,
    when given, overrides s_c. The along-wind horizontal wind (mean plus
    wave perturbation) is multiplied by 1 - F r exp(-k_s zloc / D), with F =
    shelter_factor (default env.shelter_factor = 1.0), D = env.shelter_decay_kz
    (0.5) and k_s = sum(a k^2) / sum(a k) over the counted components. The
    wave-coherent vertical perturbation of every component is multiplied by
    (1 - r). The turbulence (gust) is not sheltered.
    """

    def __init__(self, wind: Wind, trains=(), seed=1, stokes=False,
                 decay=(1.0, 1.0), turbulence=None, profile_mode="fixed",
                 shelter=False, shelter_slope=None, shelter_factor=None):
        self.wind = wind
        if profile_mode not in ("fixed", "auto"):
            raise ValueError("unknown profile_mode " + str(profile_mode))
        self.profile_mode = profile_mode
        self.shelter = bool(shelter)
        self.shelter_slope = None if shelter_slope is None else float(shelter_slope)  # None: computed
        self.shelter_factor = float(SHELTER_FACTOR if shelter_factor is None else shelter_factor)
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
        self._wave_age = self._compute_wave_age()
        if profile_mode == "auto":
            self._w_log = wind if wind.profile == "log" else _log_wind(wind)
            self._w_knee = wind if wind.profile == "swell_knee" else Wind(
                wind.u10, wind.from_deg, charnock=wind.charnock, profile="swell_knee")
        k = self.c["k"]
        if k.size:
            zref = np.maximum(1.0 / k, 2.0 * wind.z0)
            along = wind.ex * self.c["dx"] + wind.ey * self.c["dy"]
            self.urel = self._mean_speed(zref) * along - self.c["omega"] / k
        else:
            self.urel = np.zeros(0)
        # sheltering: components the wind outruns, their slope weights and the slope wavenumber
        self._sh_n = 0
        if self.shelter and k.size:
            cnt = self.urel > 0.0
            self._sh_cnt = cnt
            self._sh_n = int(cnt.sum())
            if self._sh_n:
                ak = self.c["a"][cnt] * k[cnt]
                self._sh_ks = float(np.sum(ak * k[cnt]) / np.sum(ak))
                self._sh_sc = (self.shelter_slope if self.shelter_slope is not None
                               else float(separation_steepness(
                                   self.wind.ustar, np.sqrt(G / self._sh_ks))))
                # slope along the wind of component i: -a k p sin(theta), p = d . w_hat
                self._sh_ak = np.zeros(k.size)
                self._sh_ak[cnt] = (ak * (self.wind.ex * self.c["dx"][cnt]
                                          + self.wind.ey * self.c["dy"][cnt]))

    def shelter_r(self, x, y, t):
        """Separation strength r in [0, 1] at (x, y, t); zeros unless shelter=True."""
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        r = np.zeros(x.shape)
        if not (self.shelter and self._sh_n):
            return r
        s = np.zeros(x.shape)
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            s -= (self._sh_ak[sl] * np.sin(th)).sum(-1)
        sig = np.maximum(0.0, -s)
        return np.clip((sig - self._sh_sc) / self._sh_sc, 0.0, 1.0)

    def _compute_wave_age(self):
        """(cp, A, f) of the most energetic train; (inf, 0, 0) without trains.

        A = U10 cos(theta) / cp (Hanley et al. 2010), theta the angle between the
        wind and the train's travel direction. f = 0 when A <= 0 (counterswell:
        no knee; its enhanced drag is not modelled).
        """
        best, e_best = None, -1.0
        for tr, sl in zip(self.trains, self._train_slices):
            e = float(np.sum(self.c["a"][sl] ** 2))
            if e > e_best:
                best, e_best = tr, e
        if best is None:
            return (float("inf"), 0.0, 0.0)
        cp = G * best.tp / (2.0 * np.pi)
        dx, dy = compass_to_vector(best.to_deg)
        a = self.wind.u10 * (self.wind.ex * dx + self.wind.ey * dy) / cp
        if a <= 0.0:
            return (float(cp), float(a), 0.0)  # counterswell or crossing at 90 deg: no knee
        t = float(np.clip((A_WIND_DRIVEN - a) / (A_WIND_DRIVEN - A_WAVE_DRIVEN), 0.0, 1.0))
        return (float(cp), float(a), t * t * (3.0 - 2.0 * t))

    def wave_age(self):
        """(cp, A, f): phase speed of the most energetic train, inverse wave age U10 cos(theta)/cp, wave-driven fraction."""
        return self._wave_age

    def _mean_speed(self, z):
        if self.profile_mode == "fixed":
            return self.wind.speed(z)
        f = self._wave_age[2]
        return (1.0 - f) * self._w_log.speed(z) + f * self._w_knee.speed(z)

    def _mean_vector(self, z):
        if self.profile_mode == "fixed":
            return self.wind.vector(z)
        s = self._mean_speed(z)
        return s[..., None] * np.array([self.wind.ex, self.wind.ey, 0.0])

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
        out = self._mean_vector(zl)
        c = self.c
        sh = self.shelter and self._sh_n > 0
        rr = self.shelter_r(x, y, t) if sh else None
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            base = self.urel[sl] * c["a"][sl] * c["k"][sl]
            kz = c["k"][sl] * zl[..., None]
            amp = base * np.exp(-self.decay[0] * kz)
            ampw = base * np.exp(-self.decay[1] * kz)
            up = amp * np.cos(th)
            out[..., 0] += (up * c["dx"][sl]).sum(-1)
            out[..., 1] += (up * c["dy"][sl]).sum(-1)
            out[..., 2] += (-ampw * np.sin(th)).sum(-1) * (1.0 if rr is None else 1.0 - rr)
        if sh:
            fac = 1.0 - self.shelter_factor * rr * np.exp(-self._sh_ks * zl / SHELTER_DECAY_KZ)
            al = out[..., 0] * w0.ex + out[..., 1] * w0.ey
            out[..., 0] += (fac - 1.0) * al * w0.ex
            out[..., 1] += (fac - 1.0) * al * w0.ey
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
        wa = self._wave_age
        cp_json = float(f"{wa[0]:.6g}") if np.isfinite(wa[0]) else None
        c_id, s_id = commit(), stamp([Path(__file__)])
        return {
            # no wall-clock date: the viewer shows "generated" as the commit and stamp
            "meta": {"generated": f"commit {c_id} stamp {s_id}", "commit": c_id, "stamp": s_id,
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
                      "knee_m": w.knee_m, "turbulence": self.turbulence,
                      "profile_mode": self.profile_mode,
                      "wave_age": {"cp": cp_json, "A": float(f"{wa[1]:.6g}"),
                                   "f": float(f"{wa[2]:.6g}")},
                      "shelter": {"on": self.shelter, "steepness": (self._sh_sc if self.shelter and self._sh_n else self.shelter_slope),
                                  "factor": self.shelter_factor,
                                  "decay_kz": SHELTER_DECAY_KZ}},
        }

    def to_js(self, path, name="FIELD", header=None):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            if header:
                f.write(header + "\n")
            f.write(f"window.{name} = {json.dumps(self.to_dict())};\n")


def gust_factor(u10=10.0, from_deg=270.0, z=10.0, duration=600.0, dt=0.1, window=3.0, seed=1):
    """Gust factor of the turbulence model: max running mean over `window` s / mean wind speed.

    Wind speed along the mean wind direction at a fixed point (x = y = 0, height z),
    sampled every dt over `duration` s, in the frozen turbulence field of the given seed.
    """
    w = Wind(u10, from_deg)
    env = Environment(w, [], seed=seed, turbulence="iso")
    tt = np.arange(0.0, duration, dt)
    s = env.wind_at(np.zeros_like(tt), np.zeros_like(tt), np.full_like(tt, z), tt)
    spd = s[:, 0] * w.ex + s[:, 1] * w.ey
    n = int(round(window / dt))
    run = np.convolve(spd, np.ones(n) / n, mode="valid")
    return float(run.max() / spd.mean())


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--gust-factor", action="store_true", help="print the 3 s gust factor")
    if ap.parse_args().gust_factor:
        print(f"3 s gust factor {gust_factor():.4f}")
