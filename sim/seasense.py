"""What the aircraft can see of the sea: surface sensors and a local wave estimator.

Conventions (assumed where the spec was silent):
- A sensor measure() returns a list of (x, y, t, h) with (x, y) the point where the beam meets
  the instantaneous surface and h the measured HEIGHT of the aircraft above the surface at that
  point, i.e. the measured range projected on the vertical (the attitude is taken as known from
  the IMU). The wave estimator then uses eta = z - h at (x, y, t).
- The beam is body-down: up = e_gamma cos(mu) + e_psi sin(mu) with the point-mass unit vectors of
  sim.aircraft (pitch = flight path angle gamma; the angle of attack is ignored). Tilt from the
  vertical: cos(tilt) = cos(gamma) cos(mu).
- Rates: a sensor fires when t >= its next time, then the next time advances by 1/rate_hz, so the
  mean rate is exact only if the caller steps finer than the period (at dt 0.01 and 80 Hz the
  mean is about 80 Hz, with jitter).
- Wave directions of the estimator (theta) are math angles from +x (east) toward +y (north) of
  the direction the wave travels TO, not compass angles. Components handed to sim.seawind carry
  the unit vector (cos theta, sin theta) as (dx, dy), so no conversion is needed.
- Sensor mass and power: radar altimeter M11 (110 g, 2 W); lidar mass 19 g and power 0.5 W are
  assumed; the forward scanner values are all assumed, to be sourced.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sim.seawind import Environment, G, Wind, WaveTrain


def up_vector(gamma, psi, mu):
    """Unit vector of the body up axis (lift direction) in the world frame."""
    cg, sg, cp, sp = np.cos(gamma), np.sin(gamma), np.cos(psi), np.sin(psi)
    eg = np.array([-sg * cp, -sg * sp, cg])
    ep = np.array([-sp, cp, 0.0])
    return eg * np.cos(mu) + ep * np.sin(mu)


class SurfaceSensor:
    """Base class: rate_hz, noise (std of the range, m), per-sensor firing clock."""

    name = "surface"
    mass_g = 0.0
    power_w = 0.0

    def __init__(self, rate_hz, noise):
        self.rate_hz = float(rate_hz)
        self.noise = float(noise)
        self._next = 0.0

    def reset(self):
        self._next = 0.0

    def due(self, t):
        if t + 1e-9 < self._next:
            return False
        self._next = max(self._next + 1.0 / self.rate_hz, t + 0.5 / self.rate_hz)
        return True

    def measure(self, state, mu, env, t, rng):
        raise NotImplementedError

    @staticmethod
    def beam(state, mu, env, t):
        """(tilt from vertical in rad, range r to the surface, surface point x, y, up_z).

        r is None when the beam does not point down. The intersection is found by fixed-point
        iteration on r = (z - eta(p(r))) / up_z.
        """
        x, y, z, _, g, p = (float(v) for v in state)
        up = up_vector(g, p, mu)
        uz = float(up[2])
        if uz <= 1e-3:
            return np.pi / 2, None, None, None, uz
        tilt = float(np.arccos(min(uz, 1.0)))
        r = (z - float(env.surface(x, y, t)[0])) / uz
        for _ in range(10):
            xp, yp = x - r * up[0], y - r * up[1]
            rn = (z - float(env.surface(xp, yp, t)[0])) / uz
            if abs(rn - r) < 1e-6:
                r = rn
                break
            r = rn
        xp, yp = x - r * up[0], y - r * up[1]
        return tilt, r, xp, yp, uz


class RadarAltimeter(SurfaceSensor):
    """24 GHz-class radar altimeter, 43 x 30 degree cone (record M11; mass and power from M11).

    Approximation: the return is the beam-axis point when the tilt of the axis from vertical is
    under 15 degrees plus half the cone. The half cone taken is the narrower one (15 degrees), so
    the tilt limit is 30 degrees. The nearest point inside the cone is not searched.
    """

    name = "radar"
    mass_g = 110.0
    power_w = 2.0

    def __init__(self, rate_hz=80.0, noise=0.05, rmin=0.5, rmax=50.0,
                 nadir_deg=15.0, half_cone_deg=15.0):
        super().__init__(rate_hz, noise)
        self.rmin, self.rmax = rmin, rmax
        self.tilt_max = np.radians(nadir_deg + half_cone_deg)

    def measure(self, state, mu, env, t, rng):
        if not self.due(t):
            return []
        tilt, r, xp, yp, uz = self.beam(state, mu, env, t)
        if r is None or tilt >= self.tilt_max or not (self.rmin <= r <= self.rmax):
            return []
        rm = r + self.noise * rng.standard_normal()
        return [(xp, yp, t, rm * uz)]


class LidarAltimeter(SurfaceSensor):
    """Narrow-beam lidar altimeter (record P85: about half the returns lost over water at 2 to 3
    m/s wind, none when tilted). Mass and power assumed."""

    name = "lidar"
    mass_g = 19.0
    power_w = 0.5

    def __init__(self, rate_hz=20.0, noise=0.03, rmin=0.2, rmax=100.0, tilt_max_deg=10.0,
                 dropout=0.5, wind_threshold=2.0):
        super().__init__(rate_hz, noise)
        self.rmin, self.rmax = rmin, rmax
        self.tilt_max = np.radians(tilt_max_deg)
        self.dropout, self.wind_threshold = dropout, wind_threshold

    def measure(self, state, mu, env, t, rng):
        if not self.due(t):
            return []
        tilt, r, xp, yp, uz = self.beam(state, mu, env, t)
        if r is None or tilt >= self.tilt_max or not (self.rmin <= r <= self.rmax):
            return []
        if env.wind.u10 > self.wind_threshold and rng.random() < self.dropout:
            return []
        rm = r + self.noise * rng.standard_normal()
        return [(xp, yp, t, rm * uz)]


class ForwardScanner(SurfaceSensor):
    """Samples the surface at n points along the heading, 10 to 40 m ahead (all values assumed,
    to be sourced). Direction: the air-relative heading psi (the velocity direction without the
    wind). h at each point is the vertical height above the surface there. Each point drops with
    probability 0.3; noise is on h."""

    name = "forward"
    mass_g = 50.0
    power_w = 1.5

    def __init__(self, rate_hz=10.0, noise=0.1, n_points=8, d_min=10.0, d_max=40.0, dropout=0.3):
        super().__init__(rate_hz, noise)
        self.d = np.linspace(d_min, d_max, n_points)
        self.dropout = dropout

    def measure(self, state, mu, env, t, rng):
        if not self.due(t):
            return []
        x, y, z, _, _, p = (float(v) for v in state)
        xs, ys = x + self.d * np.cos(p), y + self.d * np.sin(p)
        eta = np.asarray(env.surface(xs, ys, np.full(xs.shape, t))[0])
        keep = rng.random(xs.size) >= self.dropout
        hm = z - eta + self.noise * rng.standard_normal(xs.size)
        return [(float(xs[i]), float(ys[i]), t, float(hm[i])) for i in range(xs.size) if keep[i]]


# ---------------------------------------------------------------------------------------------
# wave estimator
# ---------------------------------------------------------------------------------------------

def _phase_arg(k, th, x, y, t):
    """k (cos th x + sin th y) - omega t, with omega = sqrt(g k); broadcasting."""
    return k * (np.cos(th) * x + np.sin(th) * y) - np.sqrt(G * k) * t


class _FittedTrain(WaveTrain):
    """WaveTrain whose components() returns the fitted components (as test_seawind does)."""

    def __init__(self, comps):
        a = comps["a"]
        hs = 4.0 * float(np.sqrt(np.sum(a ** 2) / 2.0)) if a.size else 0.0
        super().__init__(hs=max(hs, 1e-6), tp=10.0, to_deg=0.0, name="fitted")
        self._comps = comps

    def components(self, rng):
        return {k: v.copy() for k, v in self._comps.items()}


class WaveFit:
    """Fitted plane waves. params: arrays k, th, a, phi (length K)."""

    def __init__(self, k, th, a, phi, quality=float("nan"), n_samples=0, t_ref=0.0):
        self.k, self.th, self.a, self.phi = (np.atleast_1d(np.asarray(v, float))
                                              for v in (k, th, a, phi))
        self.quality = float(quality)   # residual rms / sample std
        self.n_samples = int(n_samples)
        self.t_ref = float(t_ref)

    @property
    def wavelength(self):
        return 2.0 * np.pi / self.k

    def components(self):
        k = self.k
        return {"a": self.a.copy(), "k": k.copy(), "omega": np.sqrt(G * k),
                "dx": np.cos(self.th), "dy": np.sin(self.th), "phase": self.phi.copy()}

    def surface(self, x, y, t):
        """(eta, slope_x, slope_y), same signs as Environment.surface."""
        x, y, t = np.broadcast_arrays(*(np.asarray(v, float) for v in (x, y, t)))
        eta = np.zeros(x.shape)
        ex = np.zeros(x.shape)
        ey = np.zeros(x.shape)
        for k, th, a, ph in zip(self.k, self.th, self.a, self.phi):
            arg = _phase_arg(k, th, x, y, t) + ph
            eta += a * np.cos(arg)
            s = a * k * np.sin(arg)
            ex -= s * np.cos(th)
            ey -= s * np.sin(th)
        return eta, ex, ey

    def environment(self, u10, from_deg, decay=(1.0, 1.0)):
        """Environment with the fitted components, log wind u10 blowing from from_deg (compass)."""
        return Environment(Wind(u10, from_deg), [_FittedTrain(self.components())],
                           decay=decay)


class LocalWaveEstimator:
    """Plane-wave fit of the surface samples seen in the last `window_s` seconds.

    add() stores (x, y, t, eta_measured) with eta_measured = z - h. fit() runs K steps of
    matching pursuit over a grid (wavelength 10 to 250 m, 40 log-spaced values; direction 0 to
    350 deg in 10 degree steps; best a and phi by linear least squares per grid point) and then
    Levenberg-Marquardt (Gauss-Newton with damping) on all (k, theta, a, phi). At most
    `max_samples` samples are used (uniform thinning of the window).
    """

    def __init__(self, k_waves=4, window_s=30.0, n_lambda=40, lam_range=(10.0, 250.0),
                 dtheta_deg=10.0, refine_iters=8, max_samples=400, min_samples=40,
                 min_span_s=4.0):
        self.K = int(k_waves)
        self.window_s = window_s
        lam = np.geomspace(lam_range[0], lam_range[1], n_lambda)
        th = np.radians(np.arange(0.0, 360.0, dtheta_deg))
        kk, tt = np.meshgrid(2.0 * np.pi / lam, th, indexing="ij")
        self.gk, self.gth = kk.ravel(), tt.ravel()
        self.refine_iters = refine_iters
        self.max_samples = max_samples
        self.min_samples = min_samples
        self.min_span_s = min_span_s
        self._buf = []
        self.fit_result = None

    def reset(self):
        self._buf = []
        self.fit_result = None

    def add(self, samples, z):
        """samples: list of (x, y, t, h) from a SurfaceSensor; z: aircraft height as known."""
        for x, y, t, h in samples:
            self._buf.append((x, y, t, z - h))

    def _window(self, t_now):
        buf = self._buf
        lo = t_now - self.window_s
        i = 0
        while i < len(buf) and buf[i][2] < lo:
            i += 1
        if i:
            del buf[:i]
        return np.asarray(buf, dtype=float) if buf else np.zeros((0, 4))

    def fit(self, t_now):
        """Fit the window ending at t_now; returns a WaveFit or None (too few samples)."""
        d = self._window(t_now)
        if len(d) < self.min_samples or d[-1, 2] - d[0, 2] < self.min_span_s:
            self.fit_result = None
            return None
        if len(d) > self.max_samples:
            d = d[np.linspace(0, len(d) - 1, self.max_samples).astype(int)]
        x, y, t, eta = d.T
        std = float(np.std(eta))
        # origin shift for conditioning: phases are referred to (0, 0, 0) of the world frame, so
        # no shift is applied; k x stays below 1e3 rad over the window.
        arg = _phase_arg(self.gk[:, None], self.gth[:, None], x, y, t)
        C, S = np.cos(arg), np.sin(arg)
        cc, ss, cs = (C * C).sum(1), (S * S).sum(1), (C * S).sum(1)
        det = np.maximum(cc * ss - cs * cs, 1e-12)
        r = eta.copy()
        k_l, th_l, a_l, ph_l = [], [], [], []
        for _ in range(self.K):
            rc, rs = C @ r, S @ r
            bc = (ss * rc - cs * rs) / det     # r ~ bc cos(arg) + bs sin(arg)
            bs = (cc * rs - cs * rc) / det
            gain = bc * rc + bs * rs
            i = int(np.argmax(gain))
            if gain[i] <= 1e-9 * max(float(r @ r), 1e-12):
                break
            # bc cos + bs sin = a cos(arg + phi): a cos phi = bc, -a sin phi = bs
            k_l.append(self.gk[i]); th_l.append(self.gth[i])
            a_l.append(float(np.hypot(bc[i], bs[i]))); ph_l.append(float(np.arctan2(-bs[i], bc[i])))
            r = r - bc[i] * C[i] - bs[i] * S[i]
        if not k_l:
            self.fit_result = None
            return None
        p = self._refine(x, y, t, eta, np.array(k_l), np.array(th_l), np.array(a_l),
                         np.array(ph_l))
        k, th, a, ph = p
        pred = self._model(x, y, t, k, th, a, ph)
        rms = float(np.sqrt(np.mean((eta - pred) ** 2)))
        q = rms / std if std > 0 else float("nan")
        # keep amplitudes positive and wrapped phases
        neg = a < 0
        a, ph = np.abs(a), np.where(neg, ph + np.pi, ph)
        ph = (ph + np.pi) % (2 * np.pi) - np.pi
        th = th % (2 * np.pi)
        self.fit_result = WaveFit(k, th, a, ph, quality=q, n_samples=len(d), t_ref=t_now)
        return self.fit_result

    @staticmethod
    def _model(x, y, t, k, th, a, ph):
        out = np.zeros(x.shape)
        for i in range(len(k)):
            out += a[i] * np.cos(_phase_arg(k[i], th[i], x, y, t) + ph[i])
        return out

    def _refine(self, x, y, t, eta, k, th, a, ph):
        K = len(k)
        lo_k, hi_k = 2.0 * np.pi / 400.0, 2.0 * np.pi / 6.0

        def resid(p):
            kk, tt, aa, pp = p.reshape(4, K)
            return eta - self._model(x, y, t, kk, tt, aa, pp)

        p = np.concatenate([k, th, a, ph])
        res = resid(p)
        cost = float(res @ res)
        mu = 1e-3
        for _ in range(self.refine_iters):
            kk, tt, aa, pp = p.reshape(4, K)
            Jm = np.empty((len(x), 4 * K))   # d model / d params
            for i in range(K):
                om = np.sqrt(G * kk[i])
                arg = _phase_arg(kk[i], tt[i], x, y, t) + pp[i]
                sn, cs = np.sin(arg), np.cos(arg)
                dk = np.cos(tt[i]) * x + np.sin(tt[i]) * y - om / (2.0 * kk[i]) * t
                dth = kk[i] * (-np.sin(tt[i]) * x + np.cos(tt[i]) * y)
                Jm[:, i] = -aa[i] * sn * dk
                Jm[:, K + i] = -aa[i] * sn * dth
                Jm[:, 2 * K + i] = cs
                Jm[:, 3 * K + i] = -aa[i] * sn
            A = Jm.T @ Jm
            g = Jm.T @ res
            improved = False
            for _ in range(8):
                step = np.linalg.solve(A + mu * np.diag(np.diag(A) + 1e-9), g)
                pn = p + step
                pn[:K] = np.clip(pn[:K], lo_k, hi_k)
                rn = resid(pn)
                cn = float(rn @ rn)
                if cn < cost:
                    p, res, cost = pn, rn, cn
                    mu = max(mu / 3.0, 1e-9)
                    improved = True
                    break
                mu *= 4.0
            if not improved:
                break
        k, th, a, ph = p.reshape(4, K)
        return k, th, a, ph
