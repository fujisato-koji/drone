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
- 2026-10-31 (Q-060), opt-in: Environment(wave_amp=(u, w)) multiplies the wave-coherent wind by
  these amplitudes (after any wave_wind preset), so that a sea can carry the presets of the cycle
  optimizer (sim.dsopt.wave_preset_params: horizontal 1.0, vertical lit.wave_induced_amp_w or 0.6).
- 2026-10-30 (Q-064, Q-061), all opt-in with defaults that reproduce the earlier numbers:
  Environment(surface_model="stokes2", breaking="stats", turbulence_profile="height",
  swell_profile="jet", critical_floor=..., wave_wind="p63"), Wind(stability_L=...,
  profile="swell_jet"), obukhov_length(), Environment.crest_stats(), breaking_mask(),
  whitecap_callaghan(), forristall_crest().
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
# Near-surface options (Q-064, Q-061).
BREAK_PB_COEFF = 22.0  # Banner et al. 2000 as given by Stringari et al. 2021: Pb = 22 (eps_p - 0.055)^2.01
BREAK_PB_THRESHOLD = 0.055
BREAK_PB_EXPONENT = 2.01
BREAK_STOKES_STEEPNESS = 0.443  # a k of the limiting Stokes wave (the corner)
BREAK_BAND = (0.7, 1.3)  # dominant band, fractions of the peak frequency
BREAK_CREST_LENGTH = value("env.breaking_crest_length_frac", 0.25)  # crest region length / dominant wavelength; no source
BREAK_LIFETIME = value("env.breaking_lifetime_s", 2.0)  # s; no source, a parameter
BREAK_LIFT_MAX_FRAC = value("env.breaking_lift_max_frac", 0.3)  # cap of the lift / local amplitude (slope / kp); assumed, no source
BREAK_LATERAL_FRAC = 1.0  # lateral length of one breaking event / dominant wavelength; assumed, no source
BACKFLOW_FACTOR = value("env.breaking_lee_deficit_factor", 1.2)  # along-wind deficit in the lee of a breaking crest (backflow 0.2, P92, P93)
WAVE_WIND_FLOOR = value("env.wave_wind_critical_floor", 0.1)  # floor of |1 - U/c| (P63, Grare et al. 2013)
WAVE_AMP_PRESET = (value("lit.wave_induced_amp_u", 0.95), value("lit.wave_induced_amp_w", 1.07))  # P63 A_u, A_w
TURB_LEN_W_PER_Z = value("env.turb_lw_per_height", 0.58)  # L_w = L_v = this * height (marine spectral peaks)
TURB_LEN_U_PER_Z = value("env.turb_lu_per_height", 2.4)  # L_u = this * height
SWELL_JET_SHEAR = value("env.swell_jet_shear_frac", 0.25)  # (U(H) - U(2.5 m)) / U(H) under swell
SWELL_JET_HEIGHT = value("env.swell_jet_height_m", 8.0)  # H, m
SWELL_JET_MAX_RATIO = value("env.swell_jet_max_ratio", 1.1)  # wind maximum / wind above, cp/U8 > 4
SWELL_JET_LOW_M = 2.5  # height at which the shear fraction is defined (P65 Fig. 3b)
Z0T = value("env.z0t_m", 1e-4)  # scalar (temperature) roughness length for obukhov_length, m


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


def psi_m(zeta):
    """Momentum stability function of Large and Yeager 2004 (record P95, eq. 8).

    Stable (zeta >= 0): -5 zeta. Unstable: Paulson form
    2 ln((1+X)/2) + ln((1+X^2)/2) - 2 atan X + pi/2, X = (1 - 16 zeta)^(1/4).
    """
    z = np.asarray(zeta, dtype=float)
    x = (1.0 - 16.0 * np.minimum(z, 0.0)) ** 0.25
    unst = (2.0 * np.log((1.0 + x) / 2.0) + np.log((1.0 + x * x) / 2.0)
            - 2.0 * np.arctan(x) + np.pi / 2.0)
    return np.where(z >= 0.0, -5.0 * z, unst)


def psi_h(zeta):
    """Heat stability function of P95 eq. 8: stable -5 zeta, unstable 2 ln((1+X^2)/2)."""
    z = np.asarray(zeta, dtype=float)
    x = (1.0 - 16.0 * np.minimum(z, 0.0)) ** 0.25
    return np.where(z >= 0.0, -5.0 * z, 2.0 * np.log((1.0 + x * x) / 2.0))


def obukhov_length(u10, dtheta_v, theta_v=290.0, z_u=10.0, z_t=10.0, z0t=None, n_iter=80):
    """Obukhov length L (m) from the 10 m wind and the air-sea virtual temperature difference.

    dtheta_v = virtual potential temperature of the air at z_t minus that at the
    sea surface, K (positive: warm air over cold sea, stable, L > 0). Bulk scheme
    (record P95, Large and Yeager 2004; reading assumed, Q-061):
      neutral drag 1000 CdN = 2.70/U + 0.142 + U/13.09 (eq. 6a) gives z0 = z_u exp(-k / sqrt(CdN));
      u*  = k U / (ln(z_u/z0) - psi_m(z_u/L))
      th* = k dtheta_v / (ln(z_t/z0t) - psi_h(z_t/L)),  z0t = env.z0t_m
      L   = u*^2 theta_v / (k g th*)
    iterated from neutral with damping. Returns inf for dtheta_v == 0.
    """
    if dtheta_v == 0.0:
        return float("inf")
    z0t = Z0T if z0t is None else z0t
    cdn = 1e-3 * (2.7 / u10 + 0.142 + u10 / 13.09)
    z0 = z_u * np.exp(-KARMAN / np.sqrt(cdn))
    inv_l = 0.0
    for _ in range(n_iter):
        ustar = KARMAN * u10 / (np.log(z_u / z0) - float(psi_m(z_u * inv_l)))
        thstar = KARMAN * dtheta_v / (np.log(z_t / z0t) - float(psi_h(z_t * inv_l)))
        new = KARMAN * G * thstar / (ustar**2 * theta_v)
        inv_l = 0.5 * inv_l + 0.5 * float(new)
    return float(1.0 / inv_l)


@dataclass
class Wind:
    """Mean wind profile. u10 in m/s (> 0), from_deg = direction wind blows from.

    profile="log" (default): log profile, neutral unless stability_L is given.
    stability_L (Obukhov length in m, None = neutral): Monin-Obukhov profile
    U = (u*/k)[ln(z/z0) - psi_m(z/L) + psi_m(z0/L)] with the stability functions
    of Large and Yeager 2004 (record P95, eq. 8), u* and z0 solved so that U(10) = u10;
    L is taken as given, not recomputed from u*.

    profile="swell_knee": log below the knee, constant above.

    profile="swell_jet": a shape for the wind under swell. Below the jet height H
    (env.swell_jet_height_m, 8 m) a log law over its own roughness length z0j,
    chosen so that (U(H) - U(2.5 m)) / U(H) = env.swell_jet_shear_frac (0.25);
    above H the wind is constant or, with a swell phase speed cp given, falls
    to 1/ratio of its value at H by 2H, with ratio = 1 + (env.swell_jet_max_ratio
    - 1) clip((cp/u10 - 1.7) / (4 - 1.7), 0, 1) (a wind maximum of up to 1.1 times
    the wind above for cp/U8 > 4). Scaled so that U(10) = u10. CAUTION: the
    calibration rests on a few Baltic cases (Smedman et al. 2008, record P65, Fig. 3b;
    Smedman et al. 2009, record P100), not the open ocean; the shear fraction was
    read for cp/U8 near 1.7 and is used for every wave age, and the linear ramp of
    the maximum is a model choice. ustar and z0 stay the neutral log values for
    u10 (a shape, not a stress-consistent profile). It does not combine with stability_L.
    """

    u10: float
    from_deg: float = 270.0
    charnock: float = CHARNOCK
    roughness: str = "charnock"
    hs: float | None = None
    tp: float | None = None
    sea: tuple | None = None
    profile: str = "log"  # "log" or "swell_knee"
    knee_m: float | None = None  # swell_knee: height above which speed is constant
    stability_L: float | None = None  # Obukhov length, m; None = neutral (log and swell_knee only)
    cp: float | None = None  # swell_jet: phase speed of the swell, m/s (None: no wind maximum)
    jet_height_m: float | None = None  # swell_jet: H, default env.swell_jet_height_m
    jet_shear_frac: float | None = None  # swell_jet: (U(H)-U(2.5))/U(H), default env.swell_jet_shear_frac
    jet_max_ratio: float | None = None  # swell_jet: wind maximum / wind above, default env.swell_jet_max_ratio

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
        if self.profile not in ("log", "swell_knee", "swell_jet"):
            raise ValueError("unknown profile " + str(self.profile))
        if self.stability_L is not None:
            if self.profile == "swell_jet":
                raise ValueError("stability_L does not combine with swell_jet")
            if not self.stability_L != 0.0:
                raise ValueError("stability_L must be nonzero (None = neutral)")
        if self.profile == "swell_jet":
            self._init_jet()
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
            if self.stability_L is None:
                new = KARMAN * self.u10 / np.log(zref / z0)
            else:
                new = KARMAN * self.u10 / (np.log(zref / z0) - float(psi_m(zref / self.stability_L))
                                           + float(psi_m(z0 / self.stability_L)))
            ustar = new if wind_only else 0.5 * (ustar + new)
        self.ustar = float(ustar)
        self.z0 = float(z0)

    def _init_jet(self):
        """Constants of the swell_jet shape: z0j, wind-maximum ratio, scale to U(10) = u10."""
        h = SWELL_JET_HEIGHT if self.jet_height_m is None else float(self.jet_height_m)
        sf = SWELL_JET_SHEAR if self.jet_shear_frac is None else float(self.jet_shear_frac)
        rmax = SWELL_JET_MAX_RATIO if self.jet_max_ratio is None else float(self.jet_max_ratio)
        if not h > SWELL_JET_LOW_M:
            raise ValueError("jet_height_m must exceed 2.5 m")
        if not 0.0 < sf < 1.0:
            raise ValueError("jet_shear_frac must be in (0, 1)")
        if not rmax >= 1.0:
            raise ValueError("jet_max_ratio must be >= 1")
        q = 1.0 - sf  # U(2.5)/U(H) for a log law over z0j
        self._jet_h = h
        self._jet_sf = sf
        self._jet_ln_z0j = (np.log(SWELL_JET_LOW_M) - q * np.log(h)) / (1.0 - q)
        ramp = 0.0 if self.cp is None else float(np.clip((self.cp / self.u10 - 1.7) / (4.0 - 1.7), 0.0, 1.0))
        self.jet_ratio = 1.0 + (rmax - 1.0) * ramp
        self._jet_scale = 1.0
        self._jet_scale = self.u10 / float(self._jet_shape(10.0))

    def _jet_shape(self, z):
        """swell_jet speed: log below H, falling to 1/ratio of U(H) between H and 2H, times the scale."""
        z = np.maximum(np.asarray(z, dtype=float), 1e-9)
        lz0 = self._jet_ln_z0j
        below = np.maximum(np.log(np.minimum(z, self._jet_h)) - lz0, 0.0) / (np.log(self._jet_h) - lz0)
        frac = np.clip(np.log(np.maximum(z, self._jet_h) / self._jet_h) / np.log(2.0), 0.0, 1.0)
        return below * (1.0 - (1.0 - 1.0 / self.jet_ratio) * frac) * self._jet_scale

    def speed(self, z):
        """Mean wind speed at height z above the local surface (0 for z <= z0)."""
        z = np.asarray(z, dtype=float)
        ok = z > self.z0
        zz = np.where(ok, z, 1.0)
        if self.profile == "swell_jet":
            return np.where(ok, self._jet_shape(zz), 0.0)
        if self.profile == "swell_knee":
            zz = np.minimum(zz, self.knee_m)
        if self.stability_L is None:
            return np.where(ok, (self.ustar / KARMAN) * np.log(zz / self.z0), 0.0)
        ln = np.log(zz / self.z0) - psi_m(zz / self.stability_L) + float(psi_m(self.z0 / self.stability_L))
        return np.where(ok, (self.ustar / KARMAN) * ln, 0.0)

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
    return Wind(wind.u10, wind.from_deg, charnock=wind.charnock, stability_L=wind.stability_L)


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


def turbulence_length_scales(zloc):
    """(L_u, L_v, L_w): von Karman length scales (m) of the longitudinal, lateral and vertical turbulence at height zloc.

    The one place for the near-surface laws (marine spectral peaks): L_w = L_v =
    min(env.turb_length_m, env.turb_lw_per_height * z), L_u = min(env.turb_length_m,
    env.turb_lu_per_height * z), all floored at 1 / k_max of the field (0.32 m, the
    smallest scale the modes resolve).
    """
    z = np.asarray(zloc, dtype=float)
    floor = 1.0 / TURB_K_RANGE[1]
    l_w = np.maximum(np.minimum(TURB_L, TURB_LEN_W_PER_Z * z), floor)
    l_u = np.maximum(np.minimum(TURB_L, TURB_LEN_U_PER_Z * z), floor)
    return l_u, l_w, l_w


def _turb_amp(k, length):
    """Mode amplitudes, shape (n_points, n_modes), unit variance, von Karman shape for each length (n_points,)."""
    kl = k[None, :] * np.asarray(length, dtype=float)[:, None]
    w = kl**4 / (1.0 + kl**2) ** (17.0 / 6.0) * k[None, :]
    return np.sqrt(2.0 * w / w.sum(axis=1, keepdims=True))


def rayleigh_crest_stats(sigma, n):
    """Crest-height statistics of a Rayleigh law P(C > h) = exp(-h^2 / (2 sigma^2)).

    Returns h_1/3 and h_1/10 (means of the highest third and tenth) and the
    expected maximum of n independent crests, sigma (sqrt(2 ln n) + 0.5772 / sqrt(2 ln n)).
    """
    from scipy.special import erfc

    def mean_above(q):
        h = sigma * np.sqrt(-2.0 * np.log(q))
        return float(h + sigma * np.sqrt(np.pi / 2.0) * np.exp(h * h / (2 * sigma**2))
                     * erfc(h / (np.sqrt(2.0) * sigma)))

    r = np.sqrt(2.0 * np.log(max(n, 2)))
    return {"h13": mean_above(1.0 / 3.0), "h110": mean_above(0.1),
            "hmax": float(sigma * (r + 0.5772 / r))}


def whitecap_callaghan(u10):
    """Whitecap cover (fraction of the surface) of Callaghan et al. 2008.

    W% = 3.18e-3 (U10 - 3.70)^3 for 3.70 < U10 <= 10.18; 4.82e-4 (U10 + 1.98)^3 above; zero below 3.70 m/s.
    """
    u = np.asarray(u10, dtype=float)
    w = np.where(u <= 3.70, 0.0, np.where(u <= 10.18, 3.18e-3 * (u - 3.70) ** 3, 4.82e-4 * (u + 1.98) ** 3))
    return w / 100.0


def forristall_crest(hs, t1, p, ur=0.0):
    """Crest height (m) exceeded with probability p, Forristall 3-D crest distribution.

    P(eta > eta_c) = exp(-(eta_c / (alpha Hs))^beta), alpha = 0.3536 + 0.2568 S1 + 0.0800 Ur,
    beta = 2 - 1.7912 S1 - 0.5302 Ur + 0.284 Ur^2, S1 = 2 pi Hs / (g T1^2), Ur = Hs / (k1^2 d^3)
    (deep water: Ur = 0); T1 is the mean period m0 / m1.
    """
    s1 = 2.0 * np.pi * hs / (G * t1**2)
    alpha = 0.3536 + 0.2568 * s1 + 0.0800 * ur
    beta = 2.0 - 1.7912 * s1 - 0.5302 * ur + 0.284 * ur**2
    return float(alpha * hs * (-np.log(p)) ** (1.0 / beta))


def _hash_uniform(seed, *keys):
    """Uniform [0, 1) numbers from integer keys (arrays of one shape), by a splitmix64 mix; no state."""
    with np.errstate(over="ignore"):
        h = np.full(np.shape(keys[0]), np.uint64((int(seed) * 0x9E3779B97F4A7C15 + 12345) % (1 << 64)), dtype=np.uint64)
        for k in keys:
            h = h ^ np.asarray(k).astype(np.int64).view(np.uint64)
            h = h + np.uint64(0x9E3779B97F4A7C15)
            h = (h ^ (h >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
            h = (h ^ (h >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
            h = h ^ (h >> np.uint64(31))
    return (h >> np.uint64(11)).astype(np.float64) * (1.0 / 9007199254740992.0)


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
                 shelter=False, shelter_slope=None, shelter_factor=None,
                 surface_model="linear", interaction_n=64, breaking="none",
                 turbulence_profile="fixed", swell_profile="knee",
                 critical_floor=None, wave_wind=None, wave_amp=None):
        self.wind = wind
        if wave_wind not in (None, "p63"):
            raise ValueError("unknown wave_wind " + str(wave_wind))
        if critical_floor is None or critical_floor is False:
            self.critical_floor = None
        elif critical_floor is True:
            self.critical_floor = float(WAVE_WIND_FLOOR)
        else:
            self.critical_floor = float(critical_floor)
            if self.critical_floor < 0.0:
                raise ValueError("critical_floor must be >= 0")
        self.wave_wind = wave_wind
        if surface_model not in ("linear", "stokes2"):
            raise ValueError("unknown surface_model " + str(surface_model))
        if breaking not in ("none", "stats"):
            raise ValueError("unknown breaking " + str(breaking))
        if turbulence_profile not in ("fixed", "height"):
            raise ValueError("unknown turbulence_profile " + str(turbulence_profile))
        if swell_profile not in ("knee", "jet"):
            raise ValueError("unknown swell_profile " + str(swell_profile))
        if swell_profile == "jet" and wind.stability_L is not None:
            raise ValueError("stability_L does not combine with swell_profile='jet'")
        self.surface_model = surface_model
        self.interaction_n = int(interaction_n)
        self.breaking_mode = breaking  # (the method breaking() is the older slope test)
        self.breaking_length_frac = BREAK_CREST_LENGTH
        self.breaking_lifetime_s = BREAK_LIFETIME
        self.turbulence_profile = turbulence_profile
        self.swell_profile = swell_profile
        if profile_mode not in ("fixed", "auto"):
            raise ValueError("unknown profile_mode " + str(profile_mode))
        self.profile_mode = profile_mode
        self.shelter = bool(shelter)
        self.shelter_slope = None if shelter_slope is None else float(shelter_slope)  # None: computed
        self.shelter_factor = float(SHELTER_FACTOR if shelter_factor is None else shelter_factor)
        self.stokes = bool(stokes)
        self.decay = (float(decay[0]), float(decay[1]))
        self._amp = (1.0, 1.0)
        if wave_wind == "p63":
            self._amp = (float(WAVE_AMP_PRESET[0]), float(WAVE_AMP_PRESET[1]))
            self.decay = (float(DECAY_MEASURED[0]), float(DECAY_MEASURED[1]))
        self.wave_amp = None
        if wave_amp is not None:   # explicit amplitudes (u, w) of the wave-coherent wind
            self.wave_amp = (float(wave_amp[0]), float(wave_amp[1]))
            self._amp = self.wave_amp
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
        self._brk = self._build_breaking() if breaking == "stats" else None
        if profile_mode == "auto":
            self._w_log = wind if wind.profile == "log" else _log_wind(wind)
            if swell_profile == "jet":
                cp = self._wave_age[0]
                self._w_swell = Wind(wind.u10, wind.from_deg, charnock=wind.charnock, profile="swell_jet",
                                     cp=float(cp) if np.isfinite(cp) else None)
            else:
                self._w_swell = wind if wind.profile == "swell_knee" else Wind(
                    wind.u10, wind.from_deg, charnock=wind.charnock, profile="swell_knee",
                    stability_L=wind.stability_L)
        k = self.c["k"]
        if k.size:
            zref = np.maximum(1.0 / k, 2.0 * wind.z0)
            along = wind.ex * self.c["dx"] + wind.ey * self.c["dy"]
            self.urel = self._mean_speed(zref) * along - self.c["omega"] / k
            self._along = along
            self._cph = self.c["omega"] / k
        else:
            self.urel = np.zeros(0)
            self._along = np.zeros(0)
            self._cph = np.zeros(0)
        self._self2 = self.stokes or surface_model == "stokes2"
        self._pairs = self._build_pairs() if surface_model == "stokes2" else None
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
        return (1.0 - f) * self._w_log.speed(z) + f * self._w_swell.speed(z)

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
        return {"kx": k * np.sin(ang), "ky": k * np.cos(ang), "amp": amp, "k": k,
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

    def _build_pairs(self):
        """Sum-frequency pairs among the interaction_n strongest components (deep water, Hasselmann 1962 form).

        For components i, j with unit directions at cosine c, K = |k_i + k_j|, w+ = w_i + w_j:
          E_ij = 0.5 a_i a_j (Q K - w+ P) / (g K - w+^2),
          P = c (w_i k_j + w_j k_i) + w_i k_i + w_j k_j,   Q = w_i^2 + w_j^2 + w_i w_j (1 - c),
        and the elevation term is E_ij cos(theta_i + theta_j) (both orders included). For
        collinear waves this reduces to 0.5 a_i a_j (k_i + k_j); the self term (i = j,
        half of E_ii) is Stokes' 0.5 a^2 k cos 2 theta. Derived from the second-order
        kinematic and dynamic conditions; the difference-frequency terms are not included.
        """
        c = self.c
        n = c["a"].size
        m = min(self.interaction_n, n)
        if m < 2:
            return None
        idx = np.sort(np.argsort(-c["a"], kind="stable")[:m])
        ii, jj = np.triu_indices(m, 1)
        i, j = idx[ii], idx[jj]
        k, om, a = c["k"], c["omega"], c["a"]
        cs = c["dx"][i] * c["dx"][j] + c["dy"][i] * c["dy"][j]
        kx = k[i] * c["dx"][i] + k[j] * c["dx"][j]
        ky = k[i] * c["dy"][i] + k[j] * c["dy"][j]
        kk = np.sqrt(kx**2 + ky**2)
        wp = om[i] + om[j]
        pp = cs * (om[i] * k[j] + om[j] * k[i]) + om[i] * k[i] + om[j] * k[j]
        qq = om[i] ** 2 + om[j] ** 2 + om[i] * om[j] * (1.0 - cs)
        e = 0.5 * a[i] * a[j] * (qq * kk - wp * pp) / (G * kk - wp**2)
        return {"idx": idx, "ii": ii, "jj": jj, "e": e, "kx": kx, "ky": ky}

    def _pair_terms(self, x, y, t):
        """Sum-frequency elevation and its slopes (d eta / dx, d eta / dy); zeros without pairs."""
        d_eta = np.zeros(x.shape)
        d_ex = np.zeros(x.shape)
        d_ey = np.zeros(x.shape)
        pr = self._pairs
        if pr is None:
            return d_eta, d_ex, d_ey
        c, idx = self.c, pr["idx"]
        k, dx, dy, om, ph = c["k"][idx], c["dx"][idx], c["dy"][idx], c["omega"][idx], c["phase"][idx]
        fx, fy, ft = x.ravel(), y.ravel(), t.ravel()
        np_pair = pr["e"].size
        step = max(1, 2_000_000 // np_pair)
        o_eta, o_ex, o_ey = d_eta.ravel(), d_ex.ravel(), d_ey.ravel()
        for s0 in range(0, fx.size, step):
            sl = slice(s0, s0 + step)
            th = k * (dx * fx[sl, None] + dy * fy[sl, None]) - om * ft[sl, None] + ph
            big = th[:, pr["ii"]] + th[:, pr["jj"]]
            ce = pr["e"] * np.cos(big)
            se = pr["e"] * np.sin(big)
            o_eta[sl] = ce.sum(-1)
            o_ex[sl] = -(se * pr["kx"]).sum(-1)
            o_ey[sl] = -(se * pr["ky"]).sum(-1)
        return o_eta.reshape(x.shape), o_ex.reshape(x.shape), o_ey.reshape(x.shape)

    def surface(self, x, y, t):
        """(eta, d eta/dx, d eta/dy) of the surface used for contact and for the height above the local surface.

        surface_model="linear": the linear sum (plus the self term with stokes=True).
        surface_model="stokes2": the linear sum plus the second-order self term of
        every component plus the sum-frequency interaction of the interaction_n
        strongest components (see _build_pairs), in eta and in both slopes.
        breaking="stats": breaking crests of the dominant group are lifted towards a Stokes
        corner, in eta only (see _build_breaking). The lift is (0.443 - s_max) / kp, capped at
        env.breaking_lift_max_frac (0.3, an assumption without a source) times the local
        amplitude s_max / kp, with s_max the largest slope in the crest region (7 samples across it),
        times cos^2 centred on the crest, so it peaks at the crest top and never adds metres to a long wave.
        """
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        eta, ex, ey = self._surface_sum(x, y, t)
        if self._brk is not None and self._brk["p_slot"] > 0.0:
            ev = self._breaking_events(x, y, t)
            on = ev["active"] & (np.abs(ev["sc"]) < 0.5 * self._brk["lc"])
            if on.any():
                b = self._brk
                kp = b["kp"]
                # largest slope in the crest region, from samples around the crest (one value per crest)
                xc = x[on] + ev["sc"][on] * b["dx"]
                yc = y[on] + ev["sc"][on] * b["dy"]
                tc = t[on]
                smax = np.zeros(xc.shape)
                for u in np.linspace(-0.5 * b["lc"], 0.5 * b["lc"], 7):
                    _, sxu, syu = self._surface_sum(xc + u * b["dx"], yc + u * b["dy"], tc)
                    smax = np.maximum(smax, np.hypot(sxu, syu))
                extra = np.minimum(np.maximum(0.0, BREAK_STOKES_STEEPNESS - smax) / kp,
                                   BREAK_LIFT_MAX_FRAC * smax / kp)
                lift = np.zeros(x.shape)
                lift[on] = extra * np.cos(np.pi * ev["sc"][on] / b["lc"]) ** 2
                eta = eta + lift
        return eta, ex, ey

    def _surface_sum(self, x, y, t):
        eta, ex, ey = self._surface_base(x, y, t)
        if self._pairs is not None:
            d_eta, d_ex, d_ey = self._pair_terms(x, y, t)
            eta, ex, ey = eta + d_eta, ex + d_ex, ey + d_ey
        return eta, ex, ey

    def _build_breaking(self):
        """Constants of the breaking model (breaking="stats"); None without waves.

        Probability that a dominant crest breaks (Banner et al. 2000, as given by Stringari et
        al. 2021): Pb = 22 (eps_p - 0.055)^2.01 for eps_p > 0.055, else 0, eps_p = Hp kp / 2,
        Hp = 4 sqrt(m0 of the components of the most energetic train between 0.7 and 1.3 of
        its peak frequency), kp = wp^2 / g. Events are drawn per crest of that group (the
        "dominant group"), per lateral cell (one dominant wavelength) and per time slot of the
        breaking lifetime (env.breaking_lifetime_s, a parameter), each with probability
        Pb tau / Tp, so that a crest passing a cell breaks with probability Pb. A breaking crest
        is marked over env.breaking_crest_length_frac (0.25, a parameter) of the dominant
        wavelength centred on the crest; the lee region used for shelter extends a quarter
        wavelength downwind of it. Crests are found from the dominant group alone (slope and
        curvature along the train direction, one Newton step), labelled by the phase of the
        peak wave at the crest, so draws are stateless in (x, y, t).

        The marked fraction covers only breaking of dominant waves (Banner et al.); it comes out
        one to two orders of magnitude below the whitecap cover of Callaghan et al. 2008
        (whitecap_callaghan), which includes shorter waves and the foam they leave.
        """
        if not self.trains or self.c["a"].size == 0:
            return None
        best, e_best = 0, -1.0
        for i, sl in enumerate(self._train_slices):
            e = float(np.sum(self.c["a"][sl] ** 2))
            if e > e_best:
                best, e_best = i, e
        tr, sl = self.trains[best], self._train_slices[best]
        wp = 2.0 * np.pi / tr.tp
        om = self.c["omega"][sl]
        sel = (om >= BREAK_BAND[0] * wp) & (om <= BREAK_BAND[1] * wp)
        idx = np.arange(sl.start, sl.stop)[sel]
        if idx.size == 0:
            return None
        hp = 4.0 * np.sqrt(np.sum(self.c["a"][idx] ** 2 / 2.0))
        kp = wp**2 / G
        eps = hp * kp / 2.0
        pb = (min(1.0, BREAK_PB_COEFF * (eps - BREAK_PB_THRESHOLD) ** BREAK_PB_EXPONENT)
              if eps > BREAK_PB_THRESHOLD else 0.0)
        dx, dy = compass_to_vector(tr.to_deg)
        lam = 2.0 * np.pi / kp
        return {"idx": idx, "dx": dx, "dy": dy, "kp": kp, "wp": wp, "lam": lam, "tp": tr.tp,
                "hp": float(hp), "eps_p": float(eps), "pb": float(pb),
                "tau": self.breaking_lifetime_s, "lc": self.breaking_length_frac * lam,
                "p_slot": float(min(1.0, pb * self.breaking_lifetime_s / tr.tp))}

    def breaking_stats(self):
        """{"eps_p", "pb", "hp", "kp", "p_slot", ...} of the breaking model, or None without it."""
        return None if self._brk is None else {k: v for k, v in self._brk.items() if k != "idx"}

    def _breaking_events(self, x, y, t):
        """Breaking events at (x, y, t): {"active", "sc"}.

        sc is the signed distance (m) from the point to the nearest crest of the dominant group
        along the train direction (positive: the crest is ahead). active is True where such a
        crest (curvature < 0, elevation of the group above zero) is in a breaking event.
        """
        b = self._brk
        c = self.c
        i = b["idx"]
        k, om, a, ph = c["k"][i], c["omega"][i], c["a"][i], c["phase"][i]
        pr = c["dx"][i] * b["dx"] + c["dy"][i] * b["dy"]
        fx, fy, ft = x.ravel(), y.ravel(), t.ravel()
        g1 = np.empty(fx.size)
        g2 = np.empty(fx.size)
        eg = np.empty(fx.size)
        for s0 in range(0, fx.size, 65536):
            sl = slice(s0, s0 + 65536)
            th = k * (c["dx"][i] * fx[sl, None] + c["dy"][i] * fy[sl, None]) - om * ft[sl, None] + ph
            g1[sl] = -(a * k * pr * np.sin(th)).sum(-1)
            g2[sl] = -(a * k**2 * pr**2 * np.cos(th)).sum(-1)
            eg[sl] = (a * np.cos(th)).sum(-1)
        ok = g2 < 0.0
        sc = np.where(ok, g1 / np.where(ok, -g2, 1.0), 1e12)
        crest_h = eg + np.where(ok, g1**2 / (2.0 * np.where(ok, -g2, 1.0)), 0.0)
        xc = fx + np.where(ok, sc, 0.0) * b["dx"]
        yc = fy + np.where(ok, sc, 0.0) * b["dy"]
        n = np.floor((b["kp"] * (b["dx"] * xc + b["dy"] * yc) - b["wp"] * ft) / (2.0 * np.pi) + 0.5)
        j = np.floor((-b["dy"] * xc + b["dx"] * yc) / (BREAK_LATERAL_FRAC * b["lam"]))
        m = np.floor(ft / b["tau"])
        u = _hash_uniform(self.seed, n, j, m)
        active = ok & (crest_h > 0.0) & (u < b["p_slot"])
        return {"active": active.reshape(x.shape), "sc": sc.reshape(x.shape)}

    def breaking_mask(self, x, y, t):
        """True where (x, y, t) lies in the crest region of a breaking crest (all False unless breaking="stats")."""
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        if self._brk is None or self._brk["p_slot"] <= 0.0 or x.size == 0:
            return np.zeros(x.shape, dtype=bool)
        ev = self._breaking_events(x, y, t)
        return ev["active"] & (np.abs(ev["sc"]) < 0.5 * self._brk["lc"])

    def _breaking_lee(self, x, y, t):
        """1.0 in the lee of a breaking crest (downwind side, within a quarter dominant wavelength), else 0.0."""
        if self._brk is None or self._brk["p_slot"] <= 0.0:
            return np.zeros(x.shape)
        b = self._brk
        ev = self._breaking_events(x, y, t)
        w0 = self.wind
        # the point lies downwind of the crest when -sc (train direction) points along the wind
        down = -ev["sc"] * (b["dx"] * w0.ex + b["dy"] * w0.ey) > 0.0
        near = np.abs(ev["sc"]) < 0.25 * b["lam"]
        return np.where(ev["active"] & down & near, 1.0, 0.0)

    def crest_stats(self, region, duration, spacing=25.0, dt=0.5, t0=0.0, block=40):
        """Crest-height distribution over a grid of points and a time span.

        region = (x0, x1, y0, y1) in m; points every `spacing` m (both ends included);
        eta from surface() every dt s over `duration` s from t0. A crest is the maximum of
        eta between a zero up-crossing and the next one at the same point. Returns
        {"heights": all crest heights (sorted, m), "n", "sigma" (std of eta), "mean",
        "h13", "h110", "hmax" (mean of highest third, highest tenth, maximum),
        "rayleigh": the same three for a Rayleigh law with that sigma and n crests,
        "n_points", "n_times"}. Crests at nearby points are not independent, so the
        maximum and the Rayleigh maximum are only roughly comparable.
        """
        x0, x1, y0, y1 = region
        xs = np.linspace(x0, x1, int(round((x1 - x0) / spacing)) + 1)
        ys = np.linspace(y0, y1, int(round((y1 - y0) / spacing)) + 1)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        px, py = gx.ravel(), gy.ravel()
        tt = t0 + np.arange(0.0, duration, dt)
        series = np.empty((tt.size, px.size))
        for i in range(0, tt.size, block):
            tb = tt[i:i + block]
            series[i:i + tb.size] = self.surface(px[None, :], py[None, :], tb[:, None])[0]
        heights = []
        for j in range(px.size):
            e = series[:, j]
            up = np.flatnonzero((e[:-1] < 0.0) & (e[1:] >= 0.0)) + 1
            if up.size >= 2:
                heights.append(np.maximum.reduceat(e, up)[:-1])
        h = np.sort(np.concatenate(heights)) if heights else np.zeros(0)
        sigma = float(series.std())
        out = {"heights": h, "n": int(h.size), "sigma": sigma, "mean": float(series.mean()),
               "n_points": int(px.size), "n_times": int(tt.size)}
        if h.size:
            top3 = h[-max(1, int(np.ceil(h.size / 3))):]
            top10 = h[-max(1, int(np.ceil(h.size / 10))):]
            out.update({"h13": float(top3.mean()), "h110": float(top10.mean()), "hmax": float(h[-1]),
                        "rayleigh": rayleigh_crest_stats(sigma, int(h.size))})
        return out

    def _surface_base(self, x, y, t):
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
            if self._self2:
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

    def gust(self, x, y, z, t, zloc=None):
        """Turbulent velocity (east, north, up), shape (..., 3); zeros unless turbulence="iso".

        turbulence_profile="height": the von Karman length scales follow the height above the
        local surface, turbulence_length_scales(zloc) (zloc defaults to z - eta). The frozen
        field keeps its wavenumbers and phases; only the mode amplitudes change with the
        height, by the von Karman shape for the local length, renormalised to unit variance,
        so the variance and the phases are unchanged and the turbulence energy moves to
        smaller scales near the water. The intensity law is unchanged.
        """
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
        height = self.turbulence_profile == "height"
        if height:
            if zloc is None:
                zloc = z - self.surface(x, y, t)[0]
            l_u, l_v, l_w = (v.ravel() for v in turbulence_length_scales(np.maximum(zloc, 2.0 * w0.z0)))
        lon = np.empty(fx.size)
        lat = np.empty(fx.size)
        ver = np.empty(fx.size)
        for i in range(0, fx.size, 4096):
            sl = slice(i, i + 4096)
            ph = m["kx"] * fx[sl, None] + m["ky"] * fy[sl, None]
            if height:
                lon[sl] = (_turb_amp(m["k"], l_u[sl]) * np.cos(ph + m["phase"][0])).sum(-1)
                lat[sl] = (_turb_amp(m["k"], l_v[sl]) * np.cos(ph + m["phase"][1])).sum(-1)
                ver[sl] = (_turb_amp(m["k"], l_w[sl]) * np.cos(ph + m["phase"][2])).sum(-1)
            else:
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
        rb = None
        if self.shelter and self._brk is not None:
            rb = self._breaking_lee(x, y, t)
            rr = rb if rr is None else np.maximum(rr, rb)
        if self.critical_floor is not None and c["a"].size:
            ubar = self._mean_speed(zl)
        for sl in self._chunks():
            th = self._theta(sl, x, y, t)
            if self.critical_floor is not None:
                # u~ = A a k (U(zeta) - c) exp(-a k zeta), with |1 - U/c| >= f, sign kept
                cph = self._cph[sl]
                dd = 1.0 - ubar[..., None] * self._along[sl] / cph
                dd = np.where(dd >= 0.0, 1.0, -1.0) * np.maximum(np.abs(dd), self.critical_floor)
                base = -cph * dd * c["a"][sl] * c["k"][sl]
            else:
                base = self.urel[sl] * c["a"][sl] * c["k"][sl]
            kz = c["k"][sl] * zl[..., None]
            amp = base * np.exp(-self.decay[0] * kz)
            ampw = base * np.exp(-self.decay[1] * kz)
            if self._amp != (1.0, 1.0):
                amp = amp * self._amp[0]
                ampw = ampw * self._amp[1]
            up = amp * np.cos(th)
            out[..., 0] += (up * c["dx"][sl]).sum(-1)
            out[..., 1] += (up * c["dy"][sl]).sum(-1)
            out[..., 2] += (-ampw * np.sin(th)).sum(-1) * (1.0 if rr is None else 1.0 - rr)
        if sh or (rb is not None and rb.any()):
            ks = self._sh_ks if sh else self._brk["kp"]
            f_eff = self.shelter_factor if rb is None else np.where(rb > 0.0, BACKFLOW_FACTOR, self.shelter_factor)
            fac = 1.0 - f_eff * rr * np.exp(-ks * zl / SHELTER_DECAY_KZ)
            al = out[..., 0] * w0.ex + out[..., 1] * w0.ey
            out[..., 0] += (fac - 1.0) * al * w0.ex
            out[..., 1] += (fac - 1.0) * al * w0.ey
        out = out + self.gust(x, y, z, t, zloc=zl if self.turbulence_profile == "height" else None)
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
        d = {
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
        opts = self._options()
        if opts:
            d["model"]["options"] = opts
        return d

    def _options(self):
        """The near-surface options that are on (empty dict with the defaults, so exported fields do not change)."""
        w = self.wind
        o = {}
        if self.surface_model != "linear":
            o["surface_model"] = self.surface_model
            o["interaction_n"] = self.interaction_n
        if self.breaking_mode != "none":
            o["breaking"] = {"mode": self.breaking_mode, "length_frac": self.breaking_length_frac,
                             "lifetime_s": self.breaking_lifetime_s, "lift_max_frac": BREAK_LIFT_MAX_FRAC, "stats": self.breaking_stats()}
        if self.critical_floor is not None:
            o["critical_floor"] = self.critical_floor
        if self.wave_wind is not None:
            o["wave_wind"] = {"preset": self.wave_wind, "amp": list(self._amp), "decay": list(self.decay)}
        if self.wave_amp is not None:
            o["wave_amp"] = {"amp": list(self._amp), "decay": list(self.decay)}
        if self.turbulence_profile != "fixed":
            o["turbulence_profile"] = self.turbulence_profile
        if self.swell_profile != "knee":
            o["swell_profile"] = self.swell_profile
        if w.stability_L is not None:
            o["stability_L"] = w.stability_L
        if w.profile == "swell_jet":
            o["swell_jet"] = {"height_m": w._jet_h, "shear_frac": w._jet_sf, "max_ratio": w.jet_ratio}
        return o

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
