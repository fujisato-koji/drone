"""State and wind estimator working on SensorSuite measurements only.

Choices the spec left open (kept simple):
- Inputs are the dicts returned by SensorSuite.measure; "rtk" is treated like "gnss".
- ustar_est is 0.0 and shear_quality 0.0 while no profile fit is available.
- Profile samples are stored at most every 0.1 s (10 Hz) and the fit is
  recomputed at most every 0.5 s.
- With a probe: heading of the air velocity = mag heading + beta, climb angle
  = climb angle estimate - alpha (small-angle, sign convention assumed).
- If an altimeter exists but has not delivered a reading for 1 s (out of
  range), h_surface falls back to z (flat sea).
- Estimate.valid also requires that h_surface is trustworthy (altimeter
  present, or z > 5 m).

Wind without a magnetometer (airspeed-magnitude fit, needs no heading):
- A buffer holds the last 30 s of GNSS fixes as (horizontal ground velocity,
  horizontal airspeed Va = airspeed cos(gamma_est)). When the heading spread of
  the ground track in the buffer exceeds 90 degrees, the fit
  min over W (2-vector) and s of sum (|v_g,i - W| - s Va_i)^2 is solved by
  Gauss-Newton (start: previous W, s = 1; first fit starts from the mean
  ground velocity). W is the horizontal wind; s is the pitot scale error
  (pitot_scale_est), used to correct the airspeed. The wind is not valid
  (Estimate.wind_valid False) before the spread is reached; the last W is held.
  Until then the heading falls back to the ground course.
- Without a mag the heading is the direction of v_g - W once wind_valid.
- With a mag the earlier method (wind = v_g - v_air, low-passed) is kept and the
  fit runs too, only to expose pitot_scale_est.
- The vertical wind is not observed and is set to 0: the only vertical
  information is the GNSS climb rate, which mixes the vertical wind with the
  aircraft's own climb (the pitch angle is not measured by these sensors),
  and a horizontal circle manoeuvre gives no leverage on it.
- The wind profile fit without a mag uses per-sample v_g - s Va along (v_g - W).

Changes:
- 2026-10-29 (check K-005): without a mag the ground course no longer serves as
  the heading (it hid the crosswind, 9 m/s error in a northward glide); the
  airspeed-magnitude wind fit replaces it. Added Estimate.wind_valid and
  Estimate.pitot_scale_est.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

KARMAN = 0.4
Z0_FIT = 2e-4
FIT_WINDOW_S = 30.0
FIT_SPREAD_RAD = np.radians(90.0)


@dataclass
class Estimate:
    t: float
    pos: np.ndarray
    v_ground: np.ndarray
    v_air: np.ndarray
    airspeed: float
    heading: float
    wind: np.ndarray
    h_surface: float
    z: float
    ustar_est: float
    wind_dir_est_deg: float
    shear_quality: float
    valid: bool
    wind_valid: bool = False
    pitot_scale_est: float = 1.0


def _lp(old, new, dt, tau):
    if old is None:
        return new
    a = 1.0 - np.exp(-dt / tau) if tau > 0 else 1.0
    return old + a * (new - old)


class Estimator:
    def __init__(self, sensor_names, tau_z=20.0, tau_air=0.2, tau_wind=3.0, tau_h=0.1,
                 buffer_s=60.0, z0=Z0_FIT):
        self.names = set(sensor_names)
        self.gnss_key = "rtk" if "rtk" in self.names else "gnss"
        self.has_altimeter = "altimeter" in self.names
        self.tau_z, self.tau_air, self.tau_wind, self.tau_h = tau_z, tau_air, tau_wind, tau_h
        self.buffer_s, self.z0 = buffer_s, z0
        self._t = None
        self._fix_pos = None
        self._fix_t = None
        self._v_ground = np.zeros(3)
        self._have_gnss = False
        self._have_pitot = False
        self._airspeed = None
        self._heading = 0.0
        self._gamma = 0.0
        self._alpha = 0.0
        self._beta = 0.0
        self._baro = None
        self._zoff = None  # low-passed (gnss z - baro)
        self._wind = None
        self._h_alt = None
        self._h_alt_t = -1e9
        self._buf = deque()  # (t, h, wx, wy)
        self._last_buf_t = -1e9
        self._last_fit_t = -1e9
        self._fit = (0.0, 0.0)  # ustar, quality
        self._dir = 0.0
        self._fitbuf = deque()  # (t, vgx, vgy, va_h), one per GNSS fix
        self._W = np.zeros(2)
        self._W_known = False
        self._s = 1.0
        self._wind_valid_nomag = False

    @staticmethod
    def _heading_spread(vg):
        """Range (rad) of the ground-track heading about its circular mean."""
        a = np.arctan2(vg[:, 1], vg[:, 0])
        m = np.arctan2(np.sin(a).mean(), np.cos(a).mean())
        d = (a - m + np.pi) % (2.0 * np.pi) - np.pi
        return float(d.max() - d.min())

    def _fit_airspeed_wind(self):
        """Gauss-Newton on sum (|v_g - W| - s Va)^2; returns True when updated."""
        b = np.array(self._fitbuf)
        vg, va = b[:, 1:3], b[:, 3]
        if self._heading_spread(vg) <= FIT_SPREAD_RAD:
            return False
        W = self._W.copy() if self._W_known else vg.mean(axis=0)
        s = 1.0
        for _ in range(20):
            d = vg - W
            n = np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-6)
            r = n - s * va
            J = np.column_stack([-d[:, 0] / n, -d[:, 1] / n, -va])
            try:
                step = np.linalg.solve(J.T @ J + 1e-9 * np.eye(3), J.T @ r)
            except np.linalg.LinAlgError:
                return False
            W = W - step[:2]
            s = s - step[2]
            if np.abs(step).max() < 1e-6:
                break
        if not (np.all(np.isfinite(W)) and 0.5 < s < 2.0):
            return False
        self._W, self._s, self._W_known = W, float(s), True
        return True

    def update(self, meas, t):
        dt = 0.0 if self._t is None else max(t - self._t, 0.0)
        self._t = t

        # Position and ground velocity.
        g = meas.get(self.gnss_key)
        if g is not None:
            dtg = 0.0 if self._fix_t is None else max(t - self._fix_t, 0.0)
            self._fix_pos = np.asarray(g["pos"], dtype=float).copy()
            self._v_ground = np.asarray(g["vel"], dtype=float).copy()
            self._fix_t = t
            self._have_gnss = True
            if "baro" in meas or self._baro is not None:
                b = meas["baro"]["altitude"] if "baro" in meas else self._baro
                self._zoff = _lp(self._zoff, self._fix_pos[2] - b, dtg, self.tau_z)
        if "baro" in meas:
            self._baro = float(meas["baro"]["altitude"])
        if self._fix_pos is not None:
            pos = self._fix_pos + self._v_ground * (t - self._fix_t)
        else:
            pos = np.zeros(3)
        if self._baro is not None and self._zoff is not None:
            pos[2] = self._baro + self._zoff
        elif self._baro is not None and self._fix_pos is None:
            pos[2] = self._baro
        z = float(pos[2])

        # Heading.
        vg = self._v_ground
        if "mag" in meas:
            self._heading = float(meas["mag"]["heading"])
        elif "mag" not in self.names and self._wind_valid_nomag:
            d = vg[:2] - self._W
            if np.hypot(d[0], d[1]) > 1.0:
                self._heading = float(np.arctan2(d[1], d[0]))
        elif "mag" not in self.names and np.hypot(vg[0], vg[1]) > 3.0:
            self._heading = float(np.arctan2(vg[1], vg[0]))  # fallback until the fit is valid

        # Air velocity.
        if "pitot" in meas:
            self._have_pitot = True
            self._airspeed = _lp(self._airspeed, float(meas["pitot"]["airspeed"]), dt, self.tau_air)
        if "probe" in meas:
            self._alpha = float(meas["probe"]["alpha"])
            self._beta = float(meas["probe"]["beta"])
        V = 0.0 if self._airspeed is None else float(self._airspeed)
        if V > 3.0:
            self._gamma = float(np.arcsin(np.clip(vg[2] / V, -1.0, 1.0)))
        psi_v = self._heading + self._beta
        gam_v = self._gamma - self._alpha
        v_air = V * np.array([np.cos(gam_v) * np.cos(psi_v),
                              np.cos(gam_v) * np.sin(psi_v), np.sin(gam_v)])

        # Wind.
        has_mag = "mag" in self.names
        if g is not None and self._have_pitot and V > 3.0:
            self._fitbuf.append((t, vg[0], vg[1], V * float(np.cos(self._gamma))))
        while self._fitbuf and self._fitbuf[0][0] < t - FIT_WINDOW_S:
            self._fitbuf.popleft()
        if g is not None and len(self._fitbuf) >= 10:
            if self._fit_airspeed_wind() and not has_mag:
                self._wind_valid_nomag = True
        if has_mag:
            wind_raw = vg - v_air
            if self._have_gnss and self._have_pitot:
                self._wind = _lp(self._wind, wind_raw, dt, self.tau_wind)
            wind = np.zeros(3) if self._wind is None else self._wind.copy()
            wind_valid = self._wind is not None
        else:
            # Horizontal wind from the fit (held while not valid); vertical not observed.
            wind = np.array([self._W[0], self._W[1], 0.0])
            wind_valid = self._wind_valid_nomag
            if wind_valid:
                V = V * self._s
                dv = vg[:2] - self._W
                nd = max(float(np.hypot(dv[0], dv[1])), 1e-6)
                va_h = V * float(np.cos(self._gamma))
                wind_raw = vg.copy()
                wind_raw[:2] = vg[:2] - dv / nd * va_h
                wind_raw[2] = 0.0
                v_air = vg - wind
            else:
                wind_raw = np.zeros(3)
        self._pitot_scale = self._s if self._W_known else 1.0

        # Height above the surface.
        if "altimeter" in meas:
            self._h_alt = _lp(self._h_alt, float(meas["altimeter"]["h_surface"]), dt, self.tau_h)
            self._h_alt_t = t
        if self.has_altimeter and self._h_alt is not None and t - self._h_alt_t <= 1.0:
            h = float(self._h_alt)
        else:
            h = z
        h_ok = self.has_altimeter or z > 5.0

        valid = bool(self._have_gnss and self._have_pitot and h_ok)

        # Wind profile buffer and fit.
        if valid and (has_mag or wind_valid) and t - self._last_buf_t >= 0.1:
            self._buf.append((t, h, wind_raw[0], wind_raw[1]))
            self._last_buf_t = t
        while self._buf and self._buf[0][0] < t - self.buffer_s:
            self._buf.popleft()
        if len(self._buf) >= 20 and t - self._last_fit_t >= 0.5:
            self._last_fit_t = t
            self._fit_profile()
        ustar, quality = self._fit if len(self._buf) >= 20 else (0.0, 0.0)

        return Estimate(t=t, pos=pos, v_ground=vg.copy(), v_air=v_air, airspeed=V,
                        heading=self._heading, wind=wind, h_surface=h, z=z,
                        ustar_est=float(ustar), wind_dir_est_deg=float(self._dir),
                        shear_quality=float(quality), valid=valid,
                        wind_valid=bool(wind_valid),
                        pitot_scale_est=float(self._pitot_scale))

    def _fit_profile(self):
        b = np.array(self._buf)
        h = b[:, 1]
        mw = b[:, 2:4].mean(axis=0)
        sp = np.hypot(mw[0], mw[1])
        if sp < 1e-6:
            self._fit = (0.0, 0.0)
            return
        e = mw / sp
        self._dir = float((np.degrees(np.arctan2(mw[0], mw[1])) + 180.0) % 360.0)
        ok = h > self.z0 * 10.0
        if ok.sum() < 20:
            self._fit = (0.0, 0.0)
            return
        h, u = h[ok], b[ok, 2] * e[0] + b[ok, 3] * e[1]
        span = float(h.max() - h.min())
        if span < 3.0:
            self._fit = (0.0, 0.0)
            return
        L = np.log(h / self.z0)
        a = float(L @ u / (L @ L))  # least squares through the origin: U = a * L
        rms = float(np.sqrt(np.mean((u - a * L) ** 2)))
        q = (span / 10.0) * (1.0 - rms / max(1.0, float(u.mean())))
        self._fit = (KARMAN * a, float(np.clip(q, 0.0, 1.0)))


def run_estimator(suite, estimator, states, env, times, mu=None):
    from sim.sensors import truth_from_state
    out = []
    prev = None
    for i, (s, t) in enumerate(zip(states, times)):
        m = None if mu is None else mu[i]
        truth = truth_from_state(s, env, t, m, prev)
        out.append(estimator.update(suite.measure(truth, t), t))
        prev = (s, t, m)
    return out
