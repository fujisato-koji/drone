"""Estimating the dominant swell from the aircraft's own sensors (Q-064, stage 2 of the flight over swell).

What it holds (docs/02-form.md#sea-estimation):

- Sensor models with the values of the record (keys sense.*): downward range sensors at the fuselage (a laser
  altimeter, M14, with the over-water return losses of P85 and P135, or a radar altimeter, M11) and at each wingtip
  (a wide-beam radar, M16 and P136), an optional forward-looking radar (all values assumed), and the navigation
  height and position error of a vertical and horizontal Kalman filter on a MEMS IMU (M21), GNSS height and velocity
  with or without RTK (M20) and optionally a barometer (M22).
- SwellEKF: an extended Kalman filter of one long-crested deep-water component (amplitude, phase at a reference
  point near the aircraft, frequency, direction) and a mean level that absorbs the slow part of the aircraft's own
  height error, started by a grid search and a Gauss-Newton fit over the first samples.
- The open-loop error experiment (python -m sim.swellest --errors): the estimate against the truth along kinematic
  paths over the oracle swell, an irregular long-crested swell and swell with a wind sea; key "errors" of
  sim/out/swellest.json.
- The closed loop (python -m sim.swellest --loop): the wave loop experiment's sea with the planner flying the
  estimated sea (fly_mpc's "estimated" world model, which calls the sensors every step and the estimator at every
  plan); key "loop".

Conventions:
- World frame x east, y north, z up; the wind of the experiments blows from 270 deg (toward +x). The phase of a
  component is theta = k (cos(beta) x + sin(beta) y) - omega t + phase, beta a math angle of the direction the wave
  travels to, as sim.seawind (dx, dy) = (cos beta, sin beta); the crest is theta = 0, the forward face theta in
  (0, pi).
- The navigation frame is the true frame plus the navigation error (dx, dy, dz)(t) of NavError. The estimator works
  in it. fly_mpc hands the planner the true state, so the estimated sea is moved into the true frame at each plan by
  the navigation error at that time (ShiftedWorld); what the planner then sees wrong is what an aircraft planning
  in its own navigation frame would see wrong.
"""
from __future__ import annotations

import math
import os as _os

import numpy as np

from sim.aircraft import G
from sim.record import value

HERE = _os.path.dirname(__file__)
OUT_PATH = _os.path.join(HERE, "out", "swellest.json")


def _v(key, default):
    return float(value(key, default))


# ---------------------------------------------------------------------------------------------------------------
# sensors
# ---------------------------------------------------------------------------------------------------------------

def mean_square_slope(u10):
    """Sea surface mean square slope of P135: 0.0146 sqrt(U) below 7 m/s, Cox and Munk 0.003 + 0.00512 U above."""
    u = max(float(u10), 0.1)
    if u < 7.0:
        return _v("sense.mss_low_coef", 0.0146) * math.sqrt(u)
    return _v("sense.mss_cm_a", 0.003) + _v("sense.mss_cm_b", 0.00512) * u


class RangeSensor:
    """A downward range sensor (or the forward radar). kind: "lidar" (narrow beam on the body-down axis, return
    probability p_nadir exp(-tan^2(tilt) / s^2), P85 and P135), "radar" (the nearest surface point inside a cone of
    half angle half_beam, P137; the estimator assumes the ray of the cone nearest the vertical) or "forward" (points
    ahead along the heading, all assumed). mount: "nadir", "tip_r", "tip_l" or "nadir" for the forward radar."""

    def __init__(self, name, kind, mount, rate_hz, noise_m, latency_s, rmin, rmax, half_beam_deg=0.0,
                 p_nadir=1.0, dropout=0.0, mass_g=0.0, power_w=0.0, n_points=8, d_min=10.0, d_max=40.0):
        self.name, self.kind, self.mount = name, kind, mount
        self.rate_hz, self.noise, self.latency = float(rate_hz), float(noise_m), float(latency_s)
        self.rmin, self.rmax = float(rmin), float(rmax)
        self.half_beam = math.radians(half_beam_deg)
        self.p_nadir, self.dropout = float(p_nadir), float(dropout)
        self.mass_g, self.power_w = float(mass_g), float(power_w)
        self.d = np.linspace(d_min, d_max, int(n_points))


def lidar_nadir():
    """Laser altimeter at the fuselage, LW20/C-like (M14), over-water returns of P85 and P135."""
    return RangeSensor("lidar", "lidar", "nadir", _v("sense.lidar_rate_hz", 50.0), _v("sense.lidar_noise_m", 0.05),
                       _v("sense.lidar_latency_s", 0.065), 0.2, 100.0, p_nadir=_v("sense.lidar_nadir_return", 0.5),
                       mass_g=_v("sense.lidar_mass_g", 19.5), power_w=_v("sense.lidar_power_w", 0.43))


def radar_nadir():
    """Radar altimeter at the fuselage, US-D1 (M11)."""
    return RangeSensor("radar", "radar", "nadir", _v("sense.radar_rate_hz", 82.0), _v("sense.radar_noise_m", 0.04),
                       _v("sense.radar_latency_s", 0.05), 0.5, 50.0, half_beam_deg=_v("sense.radar_half_beam_deg", 15.0),
                       mass_g=_v("sense.radar_mass_g", 110.0), power_w=_v("sense.radar_power_w", 2.0))


def radar_tip(side):
    """Wide-beam radar at a wingtip (the tip sensor of D-019 with the reach of M16 and the over-water noise of P136)."""
    return RangeSensor("tip_" + side, "radar", "tip_" + side, _v("sense.tip_radar_rate_hz", 50.0),
                       _v("sense.tip_radar_noise_m", 0.014), _v("sense.tip_radar_latency_s", 0.02), 0.05,
                       _v("sense.tip_radar_rmax_m", 20.0), half_beam_deg=_v("sense.tip_radar_half_beam_deg", 26.5),
                       mass_g=_v("sense.tip_radar_mass_g", 10.0), power_w=_v("sense.tip_radar_power_w", 0.13))


def radar_forward():
    """Forward-looking radar: points 10 to 40 m ahead along the heading (all values assumed, Q-064)."""
    return RangeSensor("forward", "forward", "nadir", _v("sense.fwd_rate_hz", 10.0), _v("sense.fwd_noise_m", 0.1),
                       _v("sense.fwd_latency_s", 0.1), 1.0, 60.0, dropout=_v("sense.fwd_dropout", 0.3),
                       mass_g=_v("sense.fwd_mass_g", 50.0), power_w=_v("sense.fwd_power_w", 1.5))


def nav_params(kind):
    """Parameters of the navigation filter's sensors for kind "standalone", "baro" (standalone plus barometer) or
    "rtk" (RTK GNSS plus barometer)."""
    p = {"rate": _v("sense.gnss_rate_hz", 10.0),
         "gz_sigma": _v("sense.gnss_vert_sigma_m", 2.5), "gz_tau": _v("sense.gnss_bias_tau_s", 300.0),
         "gz_white": _v("sense.gnss_height_white_m", 0.3),
         "gh_sigma": _v("sense.gnss_cep_m", 1.5) / 1.1774, "gh_white": _v("sense.gnss_horiz_white_m", 0.2),
         "vz_sigma": _v("sense.gnss_vz_sigma_ms", 0.1), "vh_sigma": _v("sense.gnss_vh_sigma_ms", 0.074),
         "v_tau": _v("sense.gnss_vel_tau_s", 1.0),
         "acc_nd": _v("sense.imu_accel_noise_ug", 70.0) * 1e-6 * G,
         "acc_bias": _v("sense.imu_vbias_sigma_ms2", 0.05), "acc_tau": _v("sense.imu_vbias_tau_s", 20.0),
         "baro": kind in ("baro", "rtk"),
         "baro_noise": _v("sense.baro_precision_m", 0.05), "baro_drift": _v("sense.baro_relative_m", 0.5),
         "baro_tau": _v("sense.baro_drift_tau_s", 1800.0), "baro_static": _v("sense.baro_static_sigma_m", 0.1),
         "baro_static_tau": _v("sense.baro_static_tau_s", 5.0), "baro_rate": 25.0,
         "att_sigma": math.radians(_v("sense.att_sigma_deg", 0.5)), "att_tau": _v("sense.att_tau_s", 10.0)}
    if kind == "rtk":
        p.update(gz_sigma=_v("sense.rtk_vert_sigma_m", 0.02), gz_tau=_v("sense.rtk_bias_tau_s", 60.0), gz_white=0.01,
                 gh_sigma=_v("sense.rtk_horiz_sigma_m", 0.015), gh_white=0.005, gz_tau_h=60.0)
    elif kind not in ("standalone", "baro"):
        raise ValueError("unknown navigation kind " + str(kind))
    return p


def _gm(rng, n, dt, sigma, tau):
    """A first-order Gauss-Markov series (stationary start)."""
    out = np.empty(n)
    if sigma <= 0.0:
        out[:] = 0.0
        return out
    phi = math.exp(-dt / tau)
    q = sigma * math.sqrt(1.0 - phi * phi)
    w = rng.standard_normal(n)
    x = sigma * w[0]
    for i in range(n):
        x = phi * x + q * w[i] if i else x
        out[i] = x
    return out


def _channel_error(rng, n, dt, p, axis):
    """Error of the estimate of one position axis by a 3-state Kalman filter (position, velocity, accelerometer bias)
    on the IMU, GNSS position and velocity and (vertical axis, if p["baro"]) the barometer. The error of a linear
    filter does not depend on the true motion, so the truth is taken at rest and the filter's position estimate is
    its error. axis "z" or "h" (horizontal). The correlated GNSS and barometer errors are taken by the filter as
    white noise of the same low-frequency density at their sample rate."""
    vert = axis == "z"
    g_sig, g_tau = (p["gz_sigma"], p["gz_tau"]) if vert else (p["gh_sigma"], p.get("gz_tau_h", p["gz_tau"]))
    g_white = p["gz_white"] if vert else p["gh_white"]
    v_sig = p["vz_sigma"] if vert else p["vh_sigma"]
    bias_g = _gm(rng, n, dt, g_sig, g_tau)
    v_err = _gm(rng, n, dt, v_sig, p["v_tau"])
    acc_b = _gm(rng, n, dt, p["acc_bias"], p["acc_tau"])
    acc_n = p["acc_nd"] / math.sqrt(dt) * rng.standard_normal(n)
    g_wn = g_white * rng.standard_normal(n)
    use_baro = vert and p["baro"]
    if use_baro:
        baro = (_gm(rng, n, dt, p["baro_drift"], p["baro_tau"]) + _gm(rng, n, dt, p["baro_static"],
                p["baro_static_tau"]) + p["baro_noise"] * rng.standard_normal(n))
    phi = math.exp(-dt / p["acc_tau"])
    q1, q2 = p["acc_nd"] ** 2 * dt ** 3 / 4.0, p["acc_nd"] ** 2 * dt
    q3 = p["acc_bias"] ** 2 * (1.0 - phi * phi)
    dtg = 1.0 / p["rate"]
    Rz = g_sig ** 2 + g_white ** 2
    Rv = v_sig ** 2 * (1.0 + 2.0 * p["v_tau"] / dtg)
    Rb = 0.0
    if use_baro:
        dtb = 1.0 / p["baro_rate"]
        Rb = (p["baro_noise"] ** 2 + p["baro_static"] ** 2 * (1.0 + 2.0 * p["baro_static_tau"] / dtb)
              + p["baro_drift"] ** 2 * 0.05)
    kg = max(1, int(round(dtg / dt)))
    kb = max(1, int(round(1.0 / (p["baro_rate"] * dt)))) if use_baro else 0
    # state z, v, b and the symmetric covariance (pzz, pzv, pzb, pvv, pvb, pbb), plain floats for speed
    z = v = b = 0.0
    pzz, pzv, pzb, pvv, pvb, pbb = Rz, 0.0, 0.0, Rv, 0.0, p["acc_bias"] ** 2
    h = 0.5 * dt * dt
    out = np.empty(n)
    for i in range(n):
        a = acc_b[i] + acc_n[i] - b               # true acceleration zero; the filter removes its bias estimate
        z, v, b = z + v * dt + h * a, v + dt * a, phi * b
        # P = F P F' + Q with F = [[1, dt, -h], [0, 1, -dt], [0, 0, phi]]
        nzz = pzz + 2 * dt * pzv - 2 * h * pzb + dt * dt * pvv - 2 * dt * h * pvb + h * h * pbb + q1
        nzv = pzv - dt * pzb + dt * pvv - dt * dt * pvb - h * pvb + h * dt * pbb
        nzb = phi * (pzb + dt * pvb - h * pbb)
        nvv = pvv - 2 * dt * pvb + dt * dt * pbb + q2
        nvb = phi * (pvb - dt * pbb)
        nbb = phi * phi * pbb + q3
        pzz, pzv, pzb, pvv, pvb, pbb = nzz, nzv, nzb, nvv, nvb, nbb
        if i % kg == 0:
            # position: H = [1, 0, 0]
            s = pzz + Rz
            kz, kv, kb_ = pzz / s, pzv / s, pzb / s
            r = bias_g[i] + g_wn[i] - z
            z, v, b = z + kz * r, v + kv * r, b + kb_ * r
            pzz, pzv, pzb, pvv, pvb, pbb = (pzz - kz * pzz, pzv - kz * pzv, pzb - kz * pzb, pvv - kv * pzv,
                                            pvb - kv * pzb, pbb - kb_ * pzb)
            # velocity: H = [0, 1, 0]
            s = pvv + Rv
            kz, kv, kb_ = pzv / s, pvv / s, pvb / s
            r = v_err[i] - v
            z, v, b = z + kz * r, v + kv * r, b + kb_ * r
            pzz, pzv, pzb, pvv, pvb, pbb = (pzz - kz * pzv, pzv - kz * pvv, pzb - kz * pvb, pvv - kv * pvv,
                                            pvb - kv * pvb, pbb - kb_ * pvb)
        if use_baro and i % kb == 0:
            s = pzz + Rb
            kz, kv, kb_ = pzz / s, pzv / s, pzb / s
            r = baro[i] - z
            z, v, b = z + kz * r, v + kv * r, b + kb_ * r
            pzz, pzv, pzb, pvv, pvb, pbb = (pzz - kz * pzz, pzv - kz * pzv, pzb - kz * pzb, pvv - kv * pzv,
                                            pvb - kv * pzb, pbb - kb_ * pzb)
        out[i] = z
    return out


class NavError:
    """The aircraft's navigation error over a run: (dx, dy, dz)(t) of the position filters and the bank error
    dmu(t) of the attitude, sampled at dt from t = 0 to t_end; nav kind as nav_params."""

    def __init__(self, kind, t_end, seed=0, dt=0.02):
        self.kind, self.dt = kind, float(dt)
        p = nav_params(kind)
        rng = np.random.default_rng(int(seed) + 7919)
        n = int(math.ceil((t_end + 5.0) / dt)) + 1
        self.t = np.arange(n) * dt
        self.dz = _channel_error(rng, n, dt, p, "z")
        self.dx = _channel_error(rng, n, dt, p, "h")
        self.dy = _channel_error(rng, n, dt, p, "h")
        self.dmu = _gm(rng, n, dt, p["att_sigma"], p["att_tau"])
        # what the swell estimator assumes of it: the drift rate of the slow part (variance growth of the
        # increments over 10 s) and the variance of the fast part (increments over 0.5 s)
        k10, k05 = int(round(10.0 / dt)), int(round(0.5 / dt))
        self.q_drift = float(np.var(self.dz[k10:] - self.dz[:-k10]) / 10.0) if n > k10 + 10 else 1e-4
        self.r_fast = float(np.var(self.dz[k05:] - self.dz[:-k05])) if n > k05 + 10 else 1e-4

    def at(self, t):
        """(dx, dy, dz, dmu) at time t (linear interpolation)."""
        f = min(max(float(t) / self.dt, 0.0), len(self.t) - 1.001)
        i = int(f)
        w = f - i
        return tuple(float((1.0 - w) * a[i] + w * a[i + 1]) for a in (self.dx, self.dy, self.dz, self.dmu))

    def at_many(self, t):
        t = np.asarray(t, float)
        return tuple(np.interp(t, self.t, a) for a in (self.dx, self.dy, self.dz, self.dmu))


SENSOR_SETS = {
    # name: (range sensors, navigation kind)
    "lidar": (("lidar",), "standalone"),
    "radar": (("radar",), "standalone"),
    "radar_tips": (("radar", "tip_r", "tip_l"), "standalone"),
    "radar_tips_baro": (("radar", "tip_r", "tip_l"), "baro"),
    "radar_tips_rtk": (("radar", "tip_r", "tip_l"), "rtk"),
    "lidar_rtk": (("lidar",), "rtk"),
    "radar_tips_fwd_baro": (("radar", "tip_r", "tip_l", "forward"), "baro"),
}
SET_ORDER = tuple(SENSOR_SETS)


def make_sensor(name):
    return {"lidar": lidar_nadir, "radar": radar_nadir, "forward": radar_forward,
            "tip_r": lambda: radar_tip("r"), "tip_l": lambda: radar_tip("l")}[name]()


def nav_mass_power(kind):
    """(mass g, power W) of the navigation sensors of a kind (GNSS receiver and antenna, IMU, barometer, and for
    RTK the correction link)."""
    m = _v("sense.gnss_mass_g", 40.0) + _v("sense.imu_mass_g", 5.0)
    p = _v("sense.gnss_power_w", 0.26) + _v("sense.imu_power_w", 0.002)
    if kind in ("baro", "rtk"):
        m += _v("sense.baro_mass_g", 1.0)
        p += _v("sense.baro_power_w", 0.001)
    if kind == "rtk":
        m += _v("sense.rtk_link_mass_g", 20.0)
        p += _v("sense.rtk_link_power_w", 0.5)
    return m, p


def set_mass_power(set_name):
    rs, nav = SENSOR_SETS[set_name]
    m, p = nav_mass_power(nav)
    for r in rs:
        s = make_sensor(r)
        m += s.mass_g
        p += s.power_w
    return m, p


# ---- geometry of the beams -------------------------------------------------------------------------------------

def body_axes(gamma, psi, mu):
    """(forward, up, right) unit vectors (n, 3) of the point-mass attitude (angle of attack ignored, as
    sim.seasense): forward along the air-relative velocity, up the lift direction, right = forward x up."""
    gamma, psi, mu = (np.atleast_1d(np.asarray(v, float)) for v in (gamma, psi, mu))
    cg, sg, cp, sp = np.cos(gamma), np.sin(gamma), np.cos(psi), np.sin(psi)
    fw = np.stack([cg * cp, cg * sp, sg], axis=-1)
    eg = np.stack([-sg * cp, -sg * sp, cg], axis=-1)
    ep = np.stack([-sp, cp, np.zeros_like(sp)], axis=-1)
    up = eg * np.cos(mu)[:, None] + ep * np.sin(mu)[:, None]
    right = np.cross(fw, up)
    return fw, up, right


def _cone_rays(down, fw, half):
    """Ray directions (n, m, 3) of a cone of half angle half around `down`: the axis, six rays on the edge, and the
    ray of the cone nearest the vertical (the vertical itself when inside the cone). Also returns that ray's index
    (the one the estimator assumes) and its angle from the vertical."""
    n = down.shape[0]
    e1 = fw - (fw * down).sum(-1, keepdims=True) * down
    e1 /= np.maximum(np.linalg.norm(e1, axis=-1, keepdims=True), 1e-9)
    e2 = np.cross(down, e1)
    rays = [down]
    for az in np.radians(np.arange(0.0, 360.0, 60.0)):
        rays.append(down * math.cos(half) + (e1 * math.cos(az) + e2 * math.sin(az)) * math.sin(half))
    vert = np.array([0.0, 0.0, -1.0])
    tilt = np.arccos(np.clip(-down[:, 2], -1.0, 1.0))
    # the ray nearest the vertical: rotate down towards the vertical by min(tilt, half)
    ang = np.minimum(tilt, half)
    perp = vert[None, :] - (-down[:, 2])[:, None] * down          # component of the vertical normal to down
    pn = np.linalg.norm(perp, axis=-1, keepdims=True)
    perp = np.where(pn > 1e-9, perp / np.maximum(pn, 1e-12), e1)
    near = down * np.cos(ang)[:, None] + perp * np.sin(ang)[:, None]
    rays.append(near)
    R = np.stack(rays, axis=1)
    return R, R.shape[1] - 1, tilt - ang


def ray_ranges(env, P, D, t, iters=6):
    """Range along each ray from origins P (n, 3) in directions D (n, m, 3) to the instantaneous surface of env at
    times t (n,), by fixed-point iteration on r = (z - eta(p(r))) / (-d_z). nan where the ray does not point down."""
    uz = -D[..., 2]
    ok = uz > 1e-3
    uzs = np.where(ok, uz, 1.0)
    T = np.broadcast_to(np.asarray(t, float)[:, None], uz.shape)
    X0 = np.broadcast_to(P[:, None, 0], uz.shape)
    Y0 = np.broadcast_to(P[:, None, 1], uz.shape)
    Z0 = np.broadcast_to(P[:, None, 2], uz.shape)
    r = (Z0 - np.asarray(env.surface(X0, Y0, T)[0])) / uzs
    for _ in range(iters):
        X = X0 + r * D[..., 0]
        Y = Y0 + r * D[..., 1]
        r = (Z0 - np.asarray(env.surface(X, Y, T)[0])) / uzs
    return np.where(ok, r, np.nan)


def measure(sensor, env, S, MU, t, nav, rng, u10, span):
    """Samples of one sensor at the states S (n, 6) = [x, y, z, V, gamma, psi], banks MU (n,) and times t (n,), as
    an array (k, 6) of (x, y, t, eta in the navigation frame, noise variance, time available). The navigation frame
    position of each sample is the point the estimator assumes, plus the navigation error; the height is the
    navigation height of the antenna minus the vertical height the sensor reports."""
    S = np.atleast_2d(np.asarray(S, float))
    t = np.atleast_1d(np.asarray(t, float))
    MU = np.atleast_1d(np.asarray(MU, float))
    n = len(t)
    fw, up, right = body_axes(S[:, 4], S[:, 5], MU)
    dxn, dyn, dzn, dmu = nav.at_many(t)
    P = S[:, :3].copy()
    Pn = P + np.stack([dxn, dyn, dzn], axis=-1)                  # the antenna as the navigation filter puts it
    if sensor.mount in ("tip_r", "tip_l"):
        sg = 1.0 if sensor.mount == "tip_r" else -1.0
        P = P + sg * 0.5 * span * right
        _, _, right_n = body_axes(S[:, 4], S[:, 5], MU + dmu)
        Pn = Pn + sg * 0.5 * span * right_n
    if sensor.kind == "forward":
        d = sensor.d
        hx, hy = np.cos(S[:, 5]), np.sin(S[:, 5])
        X = P[:, 0:1] + d[None, :] * hx[:, None]
        Y = P[:, 1:2] + d[None, :] * hy[:, None]
        T = np.broadcast_to(t[:, None], X.shape)
        eta = np.asarray(env.surface(X, Y, T)[0])
        h = P[:, 2:3] - eta + sensor.noise * rng.standard_normal(X.shape)
        keep = rng.random(X.shape) >= sensor.dropout
        Xn = Pn[:, 0:1] + d[None, :] * hx[:, None]
        Yn = Pn[:, 1:2] + d[None, :] * hy[:, None]
        e = Pn[:, 2:3] - h
        nk = int(keep.sum())
        out = np.stack([Xn[keep], Yn[keep], T[keep], e[keep], np.full(nk, sensor.noise ** 2),
                        T[keep] + sensor.latency, np.zeros(nk)], axis=-1)
        return out
    down = -up
    if sensor.kind == "lidar":
        D = down[:, None, :]
        r = ray_ranges(env, P, D, t)[:, 0]
        tilt = np.arccos(np.clip(-down[:, 2], -1.0, 1.0))
        p = sensor.p_nadir * np.exp(-np.tan(np.minimum(tilt, 1.5)) ** 2 / mean_square_slope(u10))
        ok = np.isfinite(r) & (r >= sensor.rmin) & (r <= sensor.rmax) & (rng.random(n) < p)
        rm = r + sensor.noise * rng.standard_normal(n)
        # the estimator's beam: body-down from the navigation attitude
        _, up_n, _ = body_axes(S[:, 4], S[:, 5], MU + dmu)
        dn = -up_n
        hv = rm * (-dn[:, 2])
        X, Y = Pn[:, 0] + rm * dn[:, 0], Pn[:, 1] + rm * dn[:, 1]
    else:
        R, i_near, tilt_off = _cone_rays(down, fw, sensor.half_beam)
        rr = ray_ranges(env, P, R, t)
        rr = np.where(np.isfinite(rr), rr, np.inf)
        r = rr.min(axis=1)
        ok = np.isfinite(r) & (r >= sensor.rmin) & (r <= sensor.rmax)
        r = np.where(ok, r, 0.0)
        rm = r + sensor.noise * rng.standard_normal(n)
        _, up_n, _ = body_axes(S[:, 4], S[:, 5], MU + dmu)
        Rn, i_n, _ = _cone_rays(-up_n, fw, sensor.half_beam)
        dn = Rn[:, i_n, :]
        hv = rm * (-dn[:, 2])
        X, Y = Pn[:, 0] + rm * dn[:, 0], Pn[:, 1] + rm * dn[:, 1]
    e = Pn[:, 2] - hv
    k = ok
    lever = np.zeros(n)
    if sensor.mount in ("tip_r", "tip_l"):
        # d(tip height)/d(bank) of the tip the estimator assumes: the error of the bank estimate enters through it
        lever = (1.0 if sensor.mount == "tip_r" else -1.0) * 0.5 * span * np.cos(MU) * np.cos(S[:, 4])
    nk = int(k.sum())
    return np.stack([X[k], Y[k], t[k], e[k], np.full(nk, sensor.noise ** 2), t[k] + sensor.latency, lever[k]],
                    axis=-1)


class SensorRig:
    """The range sensors of a set and the navigation error, as one sensor of fly_mpc(world_model="estimated"):
    measure(state, mu, env, t, rng) returns the samples due at t (an array (k, 6), see measure), with the rig's own
    random generator."""

    name = "rig"

    def __init__(self, set_name, nav, u10, span, seed=0):
        rs, _ = SENSOR_SETS[set_name]
        self.set_name = set_name
        self.sensors = [make_sensor(r) for r in rs]
        self.nav, self.u10, self.span = nav, float(u10), float(span)
        self.rng = np.random.default_rng(int(seed) + 104729)
        self._next = [0.0] * len(self.sensors)

    def reset(self):
        self._next = [0.0] * len(self.sensors)

    def measure(self, state, mu, env, t, rng=None):
        got = []
        for j, sn in enumerate(self.sensors):
            if t + 1e-9 < self._next[j]:
                continue
            self._next[j] = max(self._next[j] + 1.0 / sn.rate_hz, t + 0.5 / sn.rate_hz)
            m = measure(sn, env, np.asarray(state, float)[None, :], np.array([mu]), np.array([t]), self.nav,
                        self.rng, self.u10, self.span)
            if len(m):
                got.append(np.column_stack([m, np.full(len(m), j)]))
        return list(np.vstack(got)) if got else []


# ---------------------------------------------------------------------------------------------------------------
# the swell estimator
# ---------------------------------------------------------------------------------------------------------------

def wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


EKF_DEFAULTS = dict(
    acquire_after_s=20.0,      # first grid search this long after the first sample
    acquire_every_s=10.0,      # and again this often while the estimate is not ready ...
    reacquire_n=12,            # ... at most this many times
    acquire_max_s=120.0,       # samples of at most this window enter the grid search
    grid_tp=(5.0, 16.0, 0.1),  # s; periods of the grid search
    grid_beta_deg=1.0,         # deg; directions of the grid search
    grid_samples=600,          # samples of the grid search (thinned)
    keep_per_sensor=10,        # samples of each sensor per update (thinned)
    q_a=0.02,                  # amplitude random walk, a fraction of the amplitude per sqrt(s)
    q_psi=0.02,                # rad per sqrt(s)
    q_om=2e-5,                 # rad/s per sqrt(s)
    q_beta=math.radians(0.02),  # rad per sqrt(s)
    ready_psi=0.15,            # rad; one-sigma phase below this (and the rest below) and the estimate is used
    ready_a=0.08,              # one-sigma amplitude, a fraction of the amplitude
    ready_tp=0.3,              # s; one-sigma period
    ready_beta=math.radians(8.0),
    gate=4.0,                  # innovations beyond this many sigmas are dropped
    second_order=True,         # the model includes the second-order self term of the oracle surface
)


class SwellEKF:
    """Extended Kalman filter of one long-crested deep-water swell component and a mean level.

    State [a, psi, omega, beta, b]: amplitude, phase at the reference point (x_r, y_r, t_r), angular frequency,
    direction of travel (math angle), mean level in the navigation frame (absorbing the slow navigation height
    error). The reference moves to the newest sample at each update, so the phase stays local. Model of a sample
    at (x, y, t): eta = b + a cos(theta) + 0.5 a^2 k cos(2 theta), theta = psi + k (cos(beta) dx + sin(beta) dy)
    - omega dt, k = omega^2 / g. Started by a grid search over period and direction with a linear fit of the
    amplitude, phase, level and level trend at each grid point, then Gauss-Newton on all parameters.

    As fly_mpc's estimator: add(samples, z) buffers the rig's samples (z is ignored: the samples carry the
    navigation height); fit(t) processes the samples available by t and returns a SwellFit once the estimate is
    ready (its one-sigma errors below the ready limits), else None."""

    def __init__(self, nav, preset="p63", q_drift=None, r_fast=None, **kw):
        self.o = dict(EKF_DEFAULTS, **kw)
        self.nav, self.preset = nav, preset
        self.q_b = float(nav.q_drift if q_drift is None else q_drift)
        self.r_fast = float(nav.r_fast if r_fast is None else r_fast)
        npar = nav_params(nav.kind)
        self.att_sigma, self.att_tau = npar["att_sigma"], npar["att_tau"]
        self.reset()

    def reset(self):
        self._buf = []            # arrays (k, 8): x, y, t, eta, var, t_avail, lever, sensor
        self._pending = np.zeros((0, 8))
        self._hist = np.zeros((0, 8))
        self.x = None
        self.P = None
        self.ref = None
        self.ready = False
        self.t_first = None
        self.t_last = 0.0
        self.t_acq_next = None
        self.t_init = None
        self.t_ready = None
        self.log = []
        self.n_used = 0
        self.n_gated = 0
        self.n_reacq = 0
        self.n_replaced = 0
        self.fit_chi = None

    # ---- fly_mpc interface --------------------------------------------------------------------------------------
    def add(self, samples, z=None):
        if len(samples):
            a = np.asarray(samples, float)
            if a.shape[1] == 7:
                a = np.column_stack([a, np.zeros(len(a))])
            self._buf.append(a)

    def fit(self, t):
        self.step(t)
        return SwellFit(self, t) if self.ready else None

    # ---- the filter ----------------------------------------------------------------------------------------------
    def _take(self, t):
        if self._buf:
            self._pending = np.vstack([self._pending] + self._buf)
            self._buf = []
        av = self._pending[:, 5] <= t + 1e-9
        new = self._pending[av]
        self._pending = self._pending[~av]
        if len(new):
            new = new[np.argsort(new[:, 2], kind="stable")]
        return new

    def step(self, t):
        """Process every sample available at t. Before the first estimate, and while the estimate is not ready (at
        most reacquire_n times), a grid search over the last acquire_max_s of samples runs every acquire_every_s;
        its result replaces the filter's state when it fits that window better."""
        self.t_last = float(t)
        new = self._take(t)
        o = self.o
        if len(new):
            self._hist = np.vstack([self._hist, new])
            if self.t_first is None:
                self.t_first = float(new[0, 2])
                self.t_acq_next = self.t_first + o["acquire_after_s"]
            self._hist = self._hist[self._hist[:, 2] >= t - o["acquire_max_s"]]
        if self.x is not None and len(new):
            self._update(self._thin(new))
        due = self.t_acq_next is not None and t >= self.t_acq_next and len(self._hist) >= 50
        if due and (self.x is None or (not self.ready and self.n_reacq < o["reacquire_n"])):
            cand = self._acquire(self._hist)
            self.t_acq_next = t + o["acquire_every_s"]
            if cand is not None:
                if self.x is None:
                    self._set(cand)
                else:
                    self.n_reacq += 1
                    if cand[3] < self._chi(self._hist):
                        self._set(cand)
                        self.n_replaced += 1
        if self.x is not None:
            self._check_ready(t)
            self.log.append(self.snapshot(t))

    def _set(self, cand):
        x, P, ref, chi = cand
        self.x, self.P, self.ref = x, P, ref
        if self.t_init is None:
            self.t_init = float(self.t_last)
        self.fit_chi = chi

    def _chi(self, d):
        """Weighted mean square residual of the present state over the samples d (the state carried back to them
        without process noise)."""
        if len(d) > self.o["grid_samples"]:
            d = d[np.linspace(0, len(d) - 1, self.o["grid_samples"]).astype(int)]
        pred, _ = self._model(self.x, d, self.ref)
        w = 1.0 / (d[:, 4] + self.r_fast)
        return float(np.sum(w * (d[:, 3] - pred) ** 2) / max(len(d) - 6, 1))

    def _thin(self, d):
        k = self.o["keep_per_sensor"]
        out = []
        for s in np.unique(d[:, 7]):
            ds = d[d[:, 7] == s]
            if len(ds) > k:
                ds = ds[np.linspace(0, len(ds) - 1, k).astype(int)]
            out.append(ds)
        d = np.vstack(out)
        return d[np.argsort(d[:, 2], kind="stable")]

    def _model(self, p, d, ref):
        """Predicted eta (n,) and the Jacobian (n, len(p)) of the samples d for p = [a, psi, om, beta, b, dmu(, b1)]:
        dmu is the bank error of the navigation filter, which moves the tips' heights by lever times dmu."""
        a, psi, om, beta, b, dmu = p[:6]
        k = om * om / G
        cb, sb = math.cos(beta), math.sin(beta)
        xi, ze, tau = d[:, 0] - ref[0], d[:, 1] - ref[1], d[:, 2] - ref[2]
        along = cb * xi + sb * ze
        th = psi + k * along - om * tau
        c, s = np.cos(th), np.sin(th)
        so = 1.0 if self.o["second_order"] else 0.0
        c2, s2 = np.cos(2.0 * th), np.sin(2.0 * th)
        eta = b + a * c + so * 0.5 * a * a * k * c2 + dmu * d[:, 6]
        dth = -a * s - so * a * a * k * s2           # d eta / d theta
        J = np.empty((len(d), len(p)))
        J[:, 0] = c + so * a * k * c2
        J[:, 1] = dth
        J[:, 2] = dth * (2.0 * om / G * along - tau) + so * 0.5 * a * a * c2 * 2.0 * om / G
        J[:, 3] = dth * k * (-sb * xi + cb * ze)
        J[:, 4] = 1.0
        J[:, 5] = d[:, 6]
        if len(p) > 6:
            eta = eta + p[6] * tau
            J[:, 6] = tau
        return eta, J

    def _acquire(self, d):
        """Grid search over period and direction, then Gauss-Newton on [a, psi, om, beta, b, dmu, b1] with the
        prior of the bank error; returns (state, covariance, reference, chi^2 per degree of freedom) or None."""
        o = self.o
        if len(d) > o["grid_samples"]:
            dg = d[np.linspace(0, len(d) - 1, o["grid_samples"]).astype(int)]
        else:
            dg = d
        ref = (float(d[-1, 0]), float(d[-1, 1]), float(d[-1, 2]))
        w = 1.0 / (dg[:, 4] + self.r_fast)
        xi, ze, tau, eta = dg[:, 0] - ref[0], dg[:, 1] - ref[1], dg[:, 2] - ref[2], dg[:, 3]
        # the level, its trend and the bank error are common to all grid points: project them out once
        cols = [np.ones_like(tau), tau]
        has_lever = bool(np.any(dg[:, 6] != 0.0))
        if has_lever:
            cols.append(dg[:, 6])
        A0 = np.stack(cols, axis=1)
        W = w[:, None]
        G0 = A0.T @ (W * A0)
        if has_lever:
            G0[2, 2] += 1.0 / self.att_sigma ** 2
        M0 = np.linalg.solve(G0, (A0 * W).T)          # (m, n)

        def proj(v):           # remove the weighted fit of the common columns from the columns of v (n, m)
            return v - A0 @ (M0 @ v)
        e0 = proj(eta[:, None])[:, 0]

        def scan(tps, betas):
            """Best (gain, (a, psi, om, beta)) per period of the grid, as a list."""
            out = []
            for tp in tps:
                om = 2.0 * np.pi / tp
                k = om * om / G
                th = (k * (np.cos(betas)[None, :] * xi[:, None] + np.sin(betas)[None, :] * ze[:, None])
                      - om * tau[:, None])
                C, S = proj(np.cos(th)), proj(np.sin(th))
                cc = (W * C * C).sum(0)
                ss = (W * S * S).sum(0)
                cs = (W * C * S).sum(0)
                rc = (W * C * e0[:, None]).sum(0)
                rs = (W * S * e0[:, None]).sum(0)
                det = np.maximum(cc * ss - cs * cs, 1e-12)
                bc = (ss * rc - cs * rs) / det
                bs = (cc * rs - cs * rc) / det
                gain = bc * rc + bs * rs
                j = int(np.argmax(gain))
                out.append((float(gain[j]), (float(np.hypot(bc[j], bs[j])), float(np.arctan2(-bs[j], bc[j])), om,
                                             float(betas[j]))))
            return out
        # coarse grid, then a fine grid around the three best coarse periods
        lo, hi, step = o["grid_tp"]
        coarse = scan(np.arange(lo, hi + 1e-9, 2.5 * step),
                      np.radians(np.arange(0.0, 360.0, 2.0 * o["grid_beta_deg"])))
        coarse.sort(key=lambda c: -c[0])
        best = (-np.inf, None)
        for g, (a_, p_, om_, b_) in coarse[:3]:
            tp_ = 2.0 * np.pi / om_
            fine = scan(np.arange(tp_ - 2.5 * step, tp_ + 2.5 * step + 1e-9, 0.5 * step),
                        b_ + np.radians(np.arange(-3.0, 3.01, 0.5) * o["grid_beta_deg"]))
            for c in fine:
                if c[0] > best[0]:
                    best = c
        if best[1] is None:
            return None
        a, psi, om, beta = best[1]
        lv = M0 @ (eta - a * np.cos(psi + om * om / G * (np.cos(beta) * xi + np.sin(beta) * ze) - om * tau))
        p = np.array([a, psi, om, beta, lv[0], lv[2] if has_lever else 0.0, lv[1]])
        # Gauss-Newton on all samples of the window, with the prior of the bank error
        wa = 1.0 / (d[:, 4] + self.r_fast)
        prior = np.zeros(7)
        prior[5] = 1.0 / self.att_sigma ** 2
        for _ in range(10):
            pred, J = self._model(p, d, ref)
            r = d[:, 3] - pred
            A = J.T @ (wa[:, None] * J) + np.diag(prior)
            g = J.T @ (wa * r) - prior * p
            try:
                step = np.linalg.solve(A + 1e-9 * np.diag(np.diag(A) + 1e-12), g)
            except np.linalg.LinAlgError:
                return None
            p = p + step
            if np.max(np.abs(step[:4] / np.array([max(abs(p[0]), 0.1), 1.0, 0.05, 0.05]))) < 1e-4:
                break
        pred, J = self._model(p, d, ref)
        r = d[:, 3] - pred
        chi = float(np.sum(wa * r * r) / max(len(d) - 7, 1))
        A = J.T @ (wa[:, None] * J) + np.diag(prior)
        try:
            Pfull = np.linalg.inv(A) * max(chi, 1.0)
        except np.linalg.LinAlgError:
            return None
        if p[0] < 0:
            p[0], p[1] = -p[0], p[1] + np.pi
        p[1] = float(wrap(p[1]))
        p[3] = float(p[3] % (2.0 * np.pi))
        # the trend leaves the state (its slow part is the level's random walk)
        x = p[:6].copy()
        Pk = Pfull[:6, :6].copy()
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(Pk)) or np.any(np.diag(Pk) <= 0):
            return None
        if not (2.0 * np.pi / 20.0 < x[2] < 2.0 * np.pi / 3.0):
            return None
        return x, Pk, ref, chi

    def _propagate(self, ref_new):
        a, psi, om, beta, b, dmu = self.x
        k = om * om / G
        dxr, dyr, dt = ref_new[0] - self.ref[0], ref_new[1] - self.ref[1], ref_new[2] - self.ref[2]
        along = math.cos(beta) * dxr + math.sin(beta) * dyr
        dts = max(dt, 0.0)
        phi = math.exp(-dts / self.att_tau)
        F = np.eye(6)
        F[1, 2] = 2.0 * om / G * along - dt
        F[1, 3] = k * (-math.sin(beta) * dxr + math.cos(beta) * dyr)
        F[5, 5] = phi
        self.x = self.x.copy()
        self.x[1] = float(wrap(psi + k * along - om * dt))
        self.x[5] = dmu * phi
        o = self.o
        Q = np.diag([(o["q_a"] * max(a, 0.2)) ** 2 * dts, o["q_psi"] ** 2 * dts, o["q_om"] ** 2 * dts,
                     o["q_beta"] ** 2 * dts, self.q_b * dts, self.att_sigma ** 2 * (1.0 - phi * phi)])
        self.P = F @ self.P @ F.T + Q
        self.ref = tuple(ref_new)

    def _update(self, d):
        ref_new = (float(d[-1, 0]), float(d[-1, 1]), float(d[-1, 2]))
        self._propagate(ref_new)
        R = d[:, 4] + self.r_fast
        x0, P0 = self.x.copy(), self.P.copy()
        x = x0.copy()
        keep = np.ones(len(d), bool)
        for it in range(2):                      # iterated EKF
            pred, J = self._model(x, d, self.ref)
            if it == 0:
                S = np.einsum("ij,jk,ik->i", J, P0, J) + R
                inno = d[:, 3] - pred
                keep = np.abs(inno) <= self.o["gate"] * np.sqrt(S)
                if not keep.any():
                    self.n_gated += len(d)
                    return
            Jk, Rk = J[keep], R[keep]
            r = d[keep, 3] - pred[keep] - Jk @ (x0 - x)
            S = Jk @ P0 @ Jk.T + np.diag(Rk)
            K = np.linalg.solve(S, Jk @ P0).T
            x = x0 + K @ r
        self.x = x
        self.P = (np.eye(6) - K @ Jk) @ P0
        self.P = 0.5 * (self.P + self.P.T)
        if self.x[0] < 0.0:
            self.x[0], self.x[1] = -self.x[0], self.x[1] + np.pi
        self.x[1] = float(wrap(self.x[1]))
        self.x[3] = float(self.x[3] % (2.0 * np.pi))
        self.n_used += int(keep.sum())
        self.n_gated += int((~keep).sum())

    def sigmas(self):
        if self.P is None:
            return None
        s = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        return {"a": float(s[0]), "psi": float(s[1]), "om": float(s[2]), "beta": float(s[3]), "b": float(s[4]),
                "dmu": float(s[5])}

    def _check_ready(self, t):
        s = self.sigmas()
        a, om = self.x[0], self.x[2]
        o = self.o
        sig_tp = 2.0 * np.pi / om ** 2 * s["om"]
        ok = (s["psi"] < o["ready_psi"] and s["a"] < o["ready_a"] * max(a, 0.1) and sig_tp < o["ready_tp"]
              and s["beta"] < o["ready_beta"] and a > 0.05)
        if ok and not self.ready:
            self.t_ready = float(t)
        self.ready = bool(ok) or (self.ready and s["psi"] < 2.0 * o["ready_psi"])

    def snapshot(self, t):
        s = self.sigmas()
        return {"t": float(t), "x": [float(v) for v in self.x], "ref": [float(v) for v in self.ref],
                "sig": s, "ready": bool(self.ready)}

    # ---- the estimate --------------------------------------------------------------------------------------------
    def phase_at(self, x, y, t, state=None):
        """Estimated linear phase theta at (x, y, t) in the navigation frame."""
        sx = self.snapshot(self.t_last) if state is None else state
        a, psi, om, beta, b = sx["x"][:5]
        xr, yr, tr = sx["ref"]
        k = om * om / G
        return psi + k * (math.cos(beta) * (np.asarray(x) - xr) + math.sin(beta) * (np.asarray(y) - yr)) \
            - om * (np.asarray(t) - tr)


def phase_origin(state):
    """(a, k, omega, beta, phase at the origin, mean level) of an estimator snapshot: theta = k (cos(beta) x +
    sin(beta) y) - omega t + phase."""
    a, psi, om, beta, b = state["x"][:5]
    xr, yr, tr = state["ref"]
    k = om * om / G
    ph = psi - k * (math.cos(beta) * xr + math.sin(beta) * yr) + om * tr
    return a, k, om, beta, float(wrap(ph)), b


class ShiftedWorld:
    """An Environment raised by dz: surface(x, y, t) = env.surface + dz and wind_at(x, y, z, t) = env.wind_at(x, y,
    z - dz, t), the rest delegated (the planner uses surface, wind_at, wind and c)."""

    def __init__(self, env, dz):
        self.env, self.dz = env, float(dz)
        self.wind, self.c = env.wind, env.c

    def surface(self, x, y, t):
        eta, ex, ey = self.env.surface(x, y, t)
        return np.asarray(eta) + self.dz, ex, ey

    def wind_at(self, x, y, z, t):
        return self.env.wind_at(x, y, np.asarray(z, float) - self.dz, t)

    def __getattr__(self, name):
        return getattr(self.env, name)


def estimated_world(state, u10, from_deg, preset, nav_shift):
    """The planner's world from an estimator snapshot: the oracle sea's form (one regular component, COARE 3.5 log
    wind, the wave wind of the preset, second-order surface) with the estimated swell, moved from the navigation
    frame into the true frame by nav_shift = (dx, dy, dz) at the time of the plan."""
    from sim.seawind import Environment, Wind
    from sim.wavelib import CALM_U10, RegularSwell, preset_amp_decay
    a, k, om, beta, ph, b = phase_origin(state)
    dx, dy, dz = nav_shift
    ph = ph + k * (math.cos(beta) * dx + math.sin(beta) * dy)     # theta at the true x is theta_nav at x + dx
    to_deg = (90.0 - math.degrees(beta)) % 360.0
    wind = Wind(max(float(u10), CALM_U10), from_deg, roughness="coare35")
    amp, decay = preset_amp_decay(preset)
    env = Environment(wind, [RegularSwell(2.0 * a, 2.0 * np.pi / om, to_deg, float(wrap(ph)))], seed=1, decay=decay,
                      wave_amp=amp, surface_model="stokes2")
    return ShiftedWorld(env, b - dz)


class SwellFit:
    """What fit(t) hands fly_mpc: the estimate at t; environment() builds the planner's world."""

    def __init__(self, est, t):
        self.est, self.t = est, float(t)
        self.state = est.snapshot(t)
        s = self.state["sig"]
        self.quality = float(s["psi"])
        self.n_samples = int(est.n_used)

    def environment(self, u10, from_deg, decay=None):
        dx, dy, dz, _ = self.est.nav.at(self.t)
        return estimated_world(self.state, u10, from_deg, self.est.preset, (dx, dy, dz))


# ---------------------------------------------------------------------------------------------------------------
# truth of the swell for the error measures
# ---------------------------------------------------------------------------------------------------------------

def swell_components(env, names=("swell",)):
    """Linear components (a, k, omega, dx, dy, phase) of the swell trains of env (all trains if none is named
    swell)."""
    sl = [s for tr, s in zip(env.trains, env._train_slices) if getattr(tr, "name", "") in names]
    if not sl:
        sl = list(env._train_slices)
    idx = np.concatenate([np.arange(s.start, s.stop) for s in sl])
    return {k: env.c[k][idx] for k in ("a", "k", "omega", "dx", "dy", "phase")}


def analytic(comps, x, y, t):
    """Local amplitude and linear phase of the swell (the analytic signal of its components) at (x, y, t)."""
    x, y, t = (np.atleast_1d(np.asarray(v, float)) for v in (x, y, t))
    th = (comps["k"][None, :] * (comps["dx"][None, :] * x[:, None] + comps["dy"][None, :] * y[:, None])
          - comps["omega"][None, :] * t[:, None] + comps["phase"][None, :])
    z = (comps["a"][None, :] * np.exp(1j * th)).sum(1)
    return np.abs(z), np.angle(z)


def swell_peak(comps):
    """(period, direction beta) of the swell's energy-weighted peak component."""
    i = int(np.argmax(comps["a"]))
    return 2.0 * np.pi / comps["omega"][i], math.atan2(comps["dy"][i], comps["dx"][i])


# ---------------------------------------------------------------------------------------------------------------
# the open-loop error experiment: seas and kinematic paths
# ---------------------------------------------------------------------------------------------------------------

ERR_U10 = 6.0              # m/s; the wind of the error experiment (the closed loop's)
ERR_PRESET = "p63"
ERR_HS = (3.0, 2.0)
ERR_TP = (8.0, 10.0, 12.0)
ERR_DIRS = ("along", "across", "against")
ERR_T_END = 300.0
ERR_SEEDS = (0, 1, 2)
ERR_CHECKS = (30.0, 60.0, 120.0, 300.0)   # s since the start; errors over the 10 s before each
PATH_DT = 0.01
SURVEY_START_S = 10.0      # straight along the wind before the survey turn
SURVEY_TURN_S = 36.0       # one full turn at SURVEY_RATE
SURVEY_RATE = math.radians(10.0)
RIDE_PHASE = 0.9           # rad; the ride's place on the forward face (the closed loop's 0.7 to 1.2)
RIDE_H = 10.0              # m above the local surface (the closed loop's median about 9 to 11)
RIDE_VC = 7.0              # m/s along the crest (the closed loop's slow ride)
RIDE_SETTLE_S = 20.0       # s to move from the survey's end onto the ride's phase
RIDE_BLEND_S = 6.0         # s to blend the survey's velocity into the ride's
RIDE_WANDER = (0.03, 20.0)  # rad, s: the ride's phase wander (the closed loop's spread about its median)
LEG_V = 12.16              # m/s airspeed of the steady leg (sim.wavelib.flat_start_entry for the albatross-like glider)
SPAN = 3.0                 # m; the albatross-like glider
MIXED_HOUR = 54.0


def error_sea(kind, hs, tp, direction, seed):
    """(environment, swell names) of a sea of the error experiment. kind "regular": the oracle sea (swell_env, a
    random phase per seed); "irregular": a long-crested JONSWAP swell of gamma 7 (the swell of sim.sea_history),
    16 frequencies, random phases, linear surface; "mixed": the sea history's sea, swell and wind sea (see
    mixed_history), linear surface. The wave wind is the preset's in all."""
    from sim.seawind import Environment, WaveTrain, Wind
    from sim.wavelib import SWELL_DIRS, preset_amp_decay, swell_env
    rng = np.random.default_rng(1000 + int(seed))
    amp, decay = preset_amp_decay(ERR_PRESET)
    if kind == "regular":
        return swell_env(ERR_U10, hs, tp, direction, ERR_PRESET, phase=float(rng.uniform(-np.pi, np.pi))), ("swell",)
    if kind == "irregular":
        wind = Wind(ERR_U10, 270.0, roughness="coare35")
        to = (wind.to_deg - SWELL_DIRS[direction]) % 360.0
        tr = WaveTrain(hs=hs, tp=tp, to_deg=to, gamma=7.0, n_omega=16, n_theta=1, name="swell")
        return Environment(wind, [tr], seed=1000 + int(seed), decay=decay, wave_amp=amp), ("swell",)
    if kind == "mixed":
        h = mixed_history(direction).environment(MIXED_HOUR, seed=1000 + int(seed))
        return Environment(h.wind, h.trains, seed=1000 + int(seed), decay=decay, wave_amp=amp), ("swell",)
    raise ValueError(kind)


def mixed_history(direction):
    """The sea history of the mixed sea: 11 m/s from 270 deg for 48 h (fully developed, CEM limits), then 6 m/s
    from 270 deg ("along": the swell runs with the new wind) or from 180 deg ("across"), taken at MIXED_HOUR."""
    from sim.sea_history import SeaHistory, WindHistory
    d2 = 270.0 if direction == "along" else 180.0
    return SeaHistory(WindHistory([(0.0, 11.0, 270.0), (48.0, 11.0, 270.0), (48.01, 6.0, d2), (80.0, 6.0, d2)]))


def path_states(path, env, comps, t_end, seed, dt=PATH_DT, u_wind=ERR_U10):
    """Kinematic flight over the sea: (T, S (n, 6) = [x, y, z, V, gamma, psi], MU). path "survey_ride": straight
    along the wind for SURVEY_START_S, one level turn at SURVEY_RATE, then a crest ride (moving with the swell's
    peak component at RIDE_VC along its crest, reaching RIDE_PHASE on the forward face within RIDE_SETTLE_S, with a
    phase wander); "survey_leg": the same survey, then straight on; "leg": straight along the wind throughout;
    "ride": the ride from the start. The height is RIDE_H above the local surface; the air-relative heading, the
    climb angle and the bank follow from the ground track and the wind (u_wind toward +x)."""
    rng = np.random.default_rng(int(seed) + 31)
    n = int(round(t_end / dt)) + 1
    T = np.arange(n) * dt
    U = np.array([u_wind, 0.0])
    tp, beta = swell_peak(comps)
    om = 2.0 * np.pi / tp
    k = om * om / G
    c = om / k
    dvec = np.array([math.cos(beta), math.sin(beta)])
    perp = np.array([math.sin(beta), -math.cos(beta)])        # to the right of the travel, as the closed loop's ride
    # a smooth phase wander: three sines of random phase, periods 23, 41 and 67 s, total std RIDE_WANDER[0]
    amp_w = RIDE_WANDER[0] * math.sqrt(2.0 / 3.0)
    ph_w = rng.uniform(0.0, 2.0 * np.pi, 3)
    per_w = np.array([23.0, 41.0, 67.0])
    wander_rate = (amp_w * 2.0 * np.pi / per_w[None, :]
                   * np.cos(2.0 * np.pi * T[:, None] / per_w[None, :] + ph_w[None, :])).sum(1)
    if path == "ride":
        i_s = 0
    elif path == "leg":
        i_s = n
    else:
        i_s = min(int(round((SURVEY_START_S + SURVEY_TURN_S) / dt)), n)
    psi_air = np.zeros(n)
    turning = (T >= SURVEY_START_S) & (T < SURVEY_START_S + SURVEY_TURN_S) if path.startswith("survey") else \
        np.zeros(n, bool)
    psi_air[1:] = np.cumsum(np.where(turning[1:], SURVEY_RATE * dt, 0.0))
    vel = LEG_V * np.column_stack([np.cos(psi_air), np.sin(psi_air)]) + U[None, :]
    pos = np.zeros((n, 2))
    pos[1:] = np.cumsum(vel[1:] * dt, axis=0)
    if path in ("survey_ride", "ride"):
        if path == "ride":
            th_start = float(analytic(comps, 0.0, 0.0, 0.0)[1][0])
            pos[0] = float(wrap(RIDE_PHASE - th_start)) / k * dvec
            i0, dth = 0, 0.0
        else:
            i0 = i_s - 1
            th0 = float(analytic(comps, pos[i0, 0], pos[i0, 1], T[i0])[1][0])
            dth = float(wrap(RIDE_PHASE - th0))
        tt = T[i0 + 1:] - T[i0]
        # the settle: a raised-cosine move of the phase by dth over RIDE_SETTLE_S, and the velocity blended from the
        # survey's over RIDE_BLEND_S (a smooth start for the attitude the sensors see)
        sv = np.clip(tt / RIDE_SETTLE_S, 0.0, 1.0)
        th_rate = dth * np.pi / (2.0 * RIDE_SETTLE_S) * np.sin(np.pi * sv) * (tt < RIDE_SETTLE_S)
        th_rate = th_rate + wander_rate[i0 + 1:]
        v = (c + th_rate / k)[:, None] * dvec[None, :] + RIDE_VC * perp[None, :]
        if path == "survey_ride":
            wb = np.clip(tt / RIDE_BLEND_S, 0.0, 1.0)
            wb = (wb * wb * (3.0 - 2.0 * wb))[:, None]
            v = (1.0 - wb) * vel[i0][None, :] + wb * v
        pos[i0 + 1:] = pos[i0] + np.cumsum(v * dt, axis=0)
    vg = np.gradient(pos, dt, axis=0)
    va = vg - U[None, :]
    psi = np.unwrap(np.arctan2(va[:, 1], va[:, 0]))
    V = np.linalg.norm(va, axis=1)
    eta = np.asarray(env.surface(pos[:, 0], pos[:, 1], T)[0])
    z = eta + RIDE_H
    vz = np.gradient(z, dt)
    gamma = np.arcsin(np.clip(vz / np.maximum(V, 1.0), -0.5, 0.5))
    kern = np.ones(int(round(1.0 / dt))) / round(1.0 / dt)
    mu = np.arctan(V * np.convolve(np.gradient(psi, dt), kern, mode="same") / G)
    S = np.column_stack([pos[:, 0], pos[:, 1], z, V, gamma, psi])
    return T, S, mu


ORACLE_PATH = _os.path.join(HERE, "cache", "swellest_oracle_hs%g.npz")


def oracle_path(hs):
    """The trajectory of the oracle closed loop (written by --oracle-paths)."""
    d = np.load(ORACLE_PATH % hs)
    return d["t"], d["state"], d["mu"]


_NAV_CACHE = {}


def nav_error(kind, t_end, seed):
    key = (kind, round(float(t_end), 3), int(seed))
    if key not in _NAV_CACHE:
        _NAV_CACHE[key] = NavError(kind, t_end, seed)
    return _NAV_CACHE[key]


def sample_path(set_name, env, T, S, MU, nav, seed, u10=ERR_U10, span=SPAN):
    """All samples of a sensor set along a path, sorted by the time they become available: (k, 7)."""
    rs, _ = SENSOR_SETS[set_name]
    rng = np.random.default_rng(int(seed) + 4241)
    out = []
    psi_u = np.unwrap(S[:, 5])
    for j, name in enumerate(rs):
        sn = make_sensor(name)
        tk = np.arange(0.0, T[-1], 1.0 / sn.rate_hz)
        Sk = np.column_stack([np.interp(tk, T, S[:, c]) for c in range(6)])
        Sk[:, 5] = np.interp(tk, T, psi_u)
        mk = np.interp(tk, T, MU)
        m = []
        for lo in range(0, len(tk), 4000):          # chunks keep the ray arrays small
            sl = slice(lo, lo + 4000)
            mm = measure(sn, env, Sk[sl], mk[sl], tk[sl], nav, rng, u10, span)
            if len(mm):
                m.append(mm)
        if m:
            m = np.vstack(m)
            out.append(np.column_stack([m, np.full(len(m), j)]))
    d = np.vstack(out) if out else np.zeros((0, 8))
    return d[np.argsort(d[:, 5], kind="stable")]


def run_estimator(est, d, t_end, step_s=0.5):
    """Feed the samples d to the estimator in steps of step_s (as fly_mpc's plans)."""
    j = 0
    for t in np.arange(step_s, t_end + 1e-9, step_s):
        k = int(np.searchsorted(d[:, 5], t, side="right"))
        if k > j:
            est.add(d[j:k])
            j = k
        est.step(float(t))
    return est


def evaluate(est, env, comps, T, S, nav, every_s=1.0, ahead_s=3.0, u10=ERR_U10, preset=ERR_PRESET):
    """Errors of the estimate along the path every every_s once the estimator has a state: phase (estimated minus
    true linear phase of the swell at the aircraft, rad), height (estimated amplitude over the local amplitude of
    the swell, minus 1), period (s, against the swell's peak), direction (deg), the rms error of the surface the
    planner would see over the next ahead_s of the path (m) and of the vertical wave wind at the aircraft (m/s,
    estimated minus true), the ready flag and the true local amplitude."""
    logs = est.log
    if not logs:
        return []
    lt = np.array([g["t"] for g in logs])
    tp_true, beta_true = swell_peak(comps)
    dtp = T[1] - T[0]
    na = int(round(ahead_s / dtp))
    stride = max(1, int(round(0.25 / dtp)))
    rows = []
    for t in np.arange(every_s, T[-1] - ahead_s, every_s):
        j = int(np.searchsorted(lt, t + 1e-9, side="right")) - 1
        if j < 0:
            continue
        g = logs[j]
        i = int(round(t / dtp))
        x, y, z = S[i, 0], S[i, 1], S[i, 2]
        dx, dy, dz, _ = nav.at(t)
        a_t, th_t = analytic(comps, x, y, t)
        th_e = est.phase_at(x + dx, y + dy, t, state=g)
        a, psi, om, beta, b = g["x"][:5]
        ia = slice(i, i + na + 1, stride)
        w = estimated_world(g, u10, 270.0, preset, (dx, dy, dz))
        e_ahead = (np.asarray(w.surface(S[ia, 0], S[ia, 1], T[ia])[0])
                   - np.asarray(env.surface(S[ia, 0], S[ia, 1], T[ia])[0]))
        ww = (float(np.asarray(w.wind_at(x, y, z, t)).reshape(3)[2])
              - float(np.asarray(env.wind_at(x, y, z, t)).reshape(3)[2]))
        rows.append((t, float(wrap(th_e - th_t[0])), a / max(a_t[0], 1e-6) - 1.0, 2.0 * np.pi / om - tp_true,
                     math.degrees(float(wrap(beta - beta_true))), float(np.sqrt(np.mean(e_ahead ** 2))), ww,
                     float(g["ready"]), float(a_t[0])))
    return rows


ERR_FIELDS = ("t", "phase", "height", "period", "direction", "ahead_rms", "wave_w", "ready", "a_true")


def error_job(job):
    """One open-loop run: job = (sea kind, hs, tp, direction, path, set, seed, t_end)."""
    import time
    kind, hs, tp, direction, path, set_name, seed, t_end = job
    t0 = time.time()
    env, names = error_sea(kind, hs, tp, direction, seed)
    comps = swell_components(env, names)
    if path == "oracle":
        T, S, MU = oracle_path(hs)
        m = T <= t_end + 1e-9
        T, S, MU = T[m], S[m], MU[m]
    else:
        T, S, MU = path_states(path, env, comps, t_end, seed)
    nav = nav_error(SENSOR_SETS[set_name][1], t_end, seed)
    d = sample_path(set_name, env, T, S, MU, nav, seed)
    est = run_estimator(SwellEKF(nav, ERR_PRESET), d, t_end)
    rows = evaluate(est, env, comps, T, S, nav)
    return {"sea": kind, "hs": hs, "tp": tp, "direction": direction, "path": path, "set": set_name, "seed": seed,
            "t_end": t_end, "t_init": est.t_init, "t_ready": est.t_ready, "n_samples": int(len(d)),
            "n_used": est.n_used, "n_gated": est.n_gated, "n_reacq": est.n_reacq, "n_replaced": est.n_replaced,
            "rows": [[round(float(v), 5) for v in r] for r in rows], "wall_s": time.time() - t0}


# ---------------------------------------------------------------------------------------------------------------
# the closed loop: the wave loop experiment's sea, the planner flying the estimated sea
# ---------------------------------------------------------------------------------------------------------------

LOOP_CASES = (("albatross-3m", 3.0, "along", 6.0, "p63"), ("albatross-3m", 2.0, "along", 6.0, "p63"))
LOOP_T_END = 600.0
SURVEY_LEG_S = 6.0         # each leg of the survey circle in the closed loop
SURVEY_STEP_DEG = 60.0     # course step between the legs (one circle in 6 legs)
SURVEY_GIVE_UP_S = 240.0   # without a ready estimate by then, fly the start's flat leg on


class SensingPilot:
    """The wave library's pilot behind the swell estimate: until the estimator is ready, the start's steady leg for
    SURVEY_START_S, then a survey circle of steady legs (SURVEY_STEP_DEG apart, SURVEY_LEG_S each, repeated until
    SURVEY_GIVE_UP_S); once ready, the wave library (sim.wavelib.WavePilot) on the estimated sea. The guide's
    settings are the inner pilot's."""

    def __init__(self, pilot, est, ac, u10, start_entry, start_course_deg, survey=True):
        from sim.mpc import Cycle
        from sim.wavelib import flat_start_entry
        self.pilot, self.est = pilot, est
        self.start = Cycle(start_entry)
        self.legs = []
        if survey:
            for i in range(1, int(round(360.0 / SURVEY_STEP_DEG)) + 1):
                c = (start_course_deg + i * SURVEY_STEP_DEG) % 360.0
                self.legs.append(Cycle(flat_start_entry(ac, u10, c)))
        self.used_wave = False

    def __getattr__(self, name):
        if name in ("pilot", "est", "start", "legs", "used_wave"):
            raise AttributeError(name)
        return getattr(self.pilot, name)

    @property
    def last(self):
        return self.pilot.last

    def cycle(self, name, world, u10, chi, psi_w):
        t = self.est.t_last
        if self.est.ready and (self.used_wave or t < SURVEY_GIVE_UP_S):
            self.used_wave = True
            return self.pilot.cycle(name, world, u10, chi, psi_w)
        if not self.legs or t < SURVEY_START_S or t >= SURVEY_GIVE_UP_S:
            return self.start
        i = int((t - SURVEY_START_S) // SURVEY_LEG_S) % len(self.legs)
        return self.legs[i]


def loop_job(job):
    """One closed-loop run over swell with the estimated sea: job = (aircraft, hs, direction, u10, preset, t_end, set,
    seed[, survey]). As sim.wavelib.run_wave_job (the oracle runs: truth state and wind, motor at the middle energy
    weight, actuators pb2v, load-factor limit, stop on a wingtip contact, no reflex, start on the flat leg along the
    swell's travel, the wave library's pilot), but fly_mpc's world model "estimated": the rig's sensors every step,
    the estimator at every plan, the planner and the pilot in the estimated sea (flat until it is ready)."""
    import time
    from sim import mpc
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim import wavelib as wl
    name, hs, d, u10, preset, t_end, set_name, seed = job[:8]
    survey = job[8] if len(job) > 8 else True
    ac = by_name(name)
    env = wl.swell_env(u10, hs, wl.WAVE_EXP_TP, d, preset)
    lib = mpc.CycleLibrary.from_keyword("motor")
    pilot = wl.WavePilot.load(preset, policy="energy", flat_entries=lib.entries)
    bearing = (env.wind.to_deg - wl.SWELL_DIRS[d]) % 360.0
    start = wl.flat_start_entry(ac, u10, wl.SWELL_DIRS[d])
    nav = NavError(SENSOR_SETS[set_name][1], t_end, seed)
    rig = SensorRig(set_name, nav, u10, ac.span, seed)
    est = SwellEKF(nav, preset)
    sp = SensingPilot(pilot, est, ac, u10, start, wl.SWELL_DIRS[d], survey=survey)
    t0 = time.time()
    r = mpc.fly_mpc(ac, env, "estimated", [rig], t_end, bearing, max(u10, wl.CALM_U10), seed=1,
                    planner_kw=wl.wave_pkw(), estimator=est, library=lib, state_source="truth", wind_source="truth",
                    actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True, wave_aware=False,
                    motor=True, w_energy=mpc.energy_weight(ac, "normal"), keep=True, wave_library=sp,
                    start_entry=start)
    tr = r.pop("traj")
    T, ST, MU = np.asarray(tr["t"]), np.asarray(tr["state"]), np.asarray(tr["mu"])
    marg = mpc.flight_margins(ac, env, T, ST, MU, r)
    marg["counts"] = bool(r["ended"] == "complete" and marg["valid"])
    sw = r.get("wave_switches") or []
    t_in = sw[0][0] if sw else None
    th = np.asarray(tr["thrust"], float)
    P = th * ST[:, 3] / r["motor"]["eta_total"]
    after = None
    if t_in is not None and T[-1] > t_in + 1.0:
        k = T >= t_in
        kk, om, dx, dy, ph, _ = mpc.world_swell(env)
        theta = np.mod(kk * (dx * ST[k, 0] + dy * ST[k, 1]) - om * T[k] + ph, 2.0 * np.pi)
        after = {"t_entry_s": float(t_in), "duration_s": float(T[-1] - t_in), "avg_power_w": float(P[k].mean()),
                 "thrust_on_share": float(np.mean(th[k] > mpc.BURST_ON_FRAC * ac.mass * G)),
                 "height_median_m": float(np.median(ST[k, 2] - np.asarray(env.surface(ST[k, 0], ST[k, 1], T[k])[0]))),
                 "forward_face_share": float(np.mean(theta < np.pi)), "phase_median": float(np.median(theta))}
    comps = swell_components(env)
    rows = evaluate(est, env, comps, T, ST, nav, u10=u10, preset=preset)
    e = pilot.last
    entry = None if e is None else {"course_rel_deg": e["course_rel_deg"], "crest": bool(e.get("crest")),
                                    "kind": e["kind"], "power_avg_w": e["power_avg_w"],
                                    "h": e["ride"]["h"] if e.get("ride") else None}
    mot = r["motor"]
    mot["bursts"] = mot.get("bursts", [])[:10]
    out = {"aircraft": name, "hs": hs, "tp": wl.WAVE_EXP_TP, "direction": d, "u10": u10, "preset": preset,
           "t_end": t_end, "set": set_name, "seed": seed, "survey": bool(survey), "ended": r["ended"],
           "survival_s": r["survival_s"], "wall_s": time.time() - t0, "margins": marg, "motor": mot,
           "after_entry": after, "entry": entry, "wave_switches": sw, "flat": wl.flat_reference(lib.entries, name, u10),
           "cpu_mean_s": r["cpu_mean_s"], "max_n_flown": r["max_n_flown"],
           "min_clear_wingtip": r.get("min_clear_wingtip"), "eta_pred_rms": r.get("eta_pred_rms"),
           "fit_available_frac": r.get("fit_available_frac"), "fit_cpu_mean_s": r.get("fit_cpu_mean_s"),
           "t_init": est.t_init, "t_ready": est.t_ready, "n_used": est.n_used, "n_gated": est.n_gated,
           "n_reacq": est.n_reacq, "n_replaced": est.n_replaced,
           "rows": [[round(float(v), 5) for v in row] for row in rows]}
    return mpc._no_nan(out)


def oracle_paths_main(workers=2):
    """Rerun the oracle closed loop of LOOP_CASES with the trajectory kept, for the error experiment's oracle path
    (sim/out/swellest_oracle_hs<H>.npz)."""
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(workers) as ex:
        for res in ex.map(_oracle_path_job, LOOP_CASES):
            print(res, flush=True)


def _oracle_path_job(case):
    from sim import mpc
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim import wavelib as wl
    name, hs, d, u10, preset = case
    ac = by_name(name)
    env = wl.swell_env(u10, hs, wl.WAVE_EXP_TP, d, preset)
    lib = mpc.CycleLibrary.from_keyword("motor")
    pilot = wl.WavePilot.load(preset, policy="energy", flat_entries=lib.entries)
    bearing = (env.wind.to_deg - wl.SWELL_DIRS[d]) % 360.0
    start = wl.flat_start_entry(ac, u10, wl.SWELL_DIRS[d])
    r = mpc.fly_mpc(ac, env, "truth", [], LOOP_T_END, bearing, max(u10, wl.CALM_U10), seed=1,
                    planner_kw=wl.wave_pkw(), library=lib, state_source="truth", wind_source="truth",
                    actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True, wave_aware=False,
                    motor=True, w_energy=mpc.energy_weight(ac, "normal"), keep=True, wave_library=pilot,
                    start_entry=start)
    tr = r.pop("traj")
    np.savez_compressed(ORACLE_PATH % hs, t=tr["t"], state=tr["state"], mu=tr["mu"])
    return hs, r["ended"], r["survival_s"], r["motor"]["avg_power_w"]
