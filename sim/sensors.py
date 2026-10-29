"""Sensor models for the point-mass dynamic soaring glider.

State vector s = [x, y, z, V, gamma, psi]: x east, y north, z up, V the airspeed,
gamma the air-relative climb angle, psi the air-relative heading from +x toward +y.

Choices the spec left open (kept simple):
- accel_body is gravity-free zeros (the specific force of a point mass in
  steady flight is not modelled); the IMU accel channel is therefore pure
  noise plus bias.
- gyro_body is the finite difference of the Euler angles (euler rates), not
  true body rates, and is zero when no previous state is passed.
- A sensor output outside its `range` is dropped (not clipped), for every sensor.
- Multi-channel sensors use one flat noise/bias vector: gnss 6 (pos 3, vel 3),
  imu 6 (accel 3, gyro 3), probe 2 (alpha, beta); a float applies to all channels.
- The first call of measure() fires every sensor once, then each at 1/rate_hz.
- The output of measure() is {sensor_name: {key: value}}; the RTK variant keeps
  the name "rtk" (same keys as "gnss").
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

# Number of output channels per sensor kind, and the output keys.
_CHANNELS = {"gnss": 6, "rtk": 6, "pitot": 1, "imu": 6, "baro": 1,
             "altimeter": 1, "mag": 1, "probe": 2}

# Assumed, to be sourced: (power W, mass g) per sensor.
_POWER_MASS = {"gnss": (0.3, 20.0), "rtk": (1.0, 40.0), "pitot": (0.05, 10.0),
               "imu": (0.1, 5.0), "baro": (0.01, 1.0), "altimeter": (0.5, 25.0),
               "mag": (0.01, 2.0), "probe": (0.2, 30.0)}


def truth_from_state(s, env, t, mu=None, prev=None):
    """True quantities for state s at time t.

    mu: bank angle (roll for the Euler angles), 0 if None.
    prev: optional (s_prev, t_prev[, mu_prev]) used for the gyro finite difference.
    """
    s = np.asarray(s, dtype=float)
    x, y, z, V, gam, psi = s
    pos = np.array([x, y, z])
    e_v = np.array([np.cos(gam) * np.cos(psi), np.cos(gam) * np.sin(psi), np.sin(gam)])
    v_air = V * e_v
    wind = np.asarray(env.wind_at(x, y, z, t), dtype=float).reshape(3)
    eta, sx, sy = env.surface(x, y, t)
    eta, sx, sy = float(eta), float(sx), float(sy)
    euler = np.array([0.0 if mu is None else float(mu), gam, psi])
    gyro = np.zeros(3)
    if prev is not None:
        sp, tp = np.asarray(prev[0], dtype=float), float(prev[1])
        mup = float(prev[2]) if len(prev) > 2 and prev[2] is not None else 0.0
        if t > tp:
            ep = np.array([mup if mu is not None else 0.0, sp[4], sp[5]])
            d = euler - ep
            d[2] = (d[2] + np.pi) % (2 * np.pi) - np.pi  # wrap yaw difference
            gyro = d / (t - tp)
    return {
        "t": float(t), "pos": pos, "v_air": v_air, "wind": wind,
        "v_ground": v_air + wind, "eta": eta, "slope_x": sx, "slope_y": sy,
        "h_surface": float(z - eta), "airspeed": float(V), "alpha": 0.0, "beta": 0.0,
        "heading": float(psi), "euler": euler,
        "accel_body": np.zeros(3), "gyro_body": gyro,
    }


@dataclass
class SensorSpec:
    name: str
    rate_hz: float
    noise: object = 0.0          # float or per-channel std vector
    bias: object = 0.0           # float or per-channel initial bias vector
    bias_walk: object = 0.0      # std per sqrt(s), float or vector
    range: Optional[tuple] = None  # (min, max) of the measured value, else dropped
    delay_s: float = 0.0


def default_spec(name):
    if name == "gnss":
        return SensorSpec("gnss", 5.0, noise=[1.5, 1.5, 3.0, 0.1, 0.1, 0.1])
    if name == "rtk":
        return SensorSpec("rtk", 10.0, noise=[0.02, 0.02, 0.04, 0.1, 0.1, 0.1])
    if name == "pitot":
        return SensorSpec("pitot", 50.0, noise=0.5, bias=0.3, bias_walk=0.01, range=(3.0, 60.0))
    if name == "imu":
        return SensorSpec("imu", 200.0, noise=[0.05] * 3 + [0.002] * 3,
                          bias=[0.0] * 3 + [0.005] * 3,
                          bias_walk=[0.0] * 3 + [1e-4] * 3)
    if name == "baro":
        return SensorSpec("baro", 20.0, noise=0.5, bias_walk=0.02)
    if name == "altimeter":
        return SensorSpec("altimeter", 20.0, noise=0.1, range=(0.2, 50.0))
    if name == "mag":
        return SensorSpec("mag", 20.0, noise=0.035)
    if name == "probe":
        return SensorSpec("probe", 50.0, noise=0.0175)
    raise KeyError(name)


def _ideal(name, truth):
    """Noise-free output vector for sensor `name`."""
    if name in ("gnss", "rtk"):
        return np.concatenate([truth["pos"], truth["v_ground"]])
    if name == "pitot":
        return np.array([truth["airspeed"]])
    if name == "imu":
        return np.concatenate([truth["accel_body"], truth["gyro_body"]])
    if name == "baro":
        return np.array([truth["pos"][2]])
    if name == "altimeter":
        return np.array([truth["h_surface"]])
    if name == "mag":
        return np.array([truth["heading"]])
    if name == "probe":
        return np.array([truth["alpha"], truth["beta"]])
    raise KeyError(name)


def _pack(name, v):
    if name in ("gnss", "rtk"):
        return {"pos": v[:3].copy(), "vel": v[3:].copy()}
    if name == "pitot":
        return {"airspeed": float(v[0])}
    if name == "imu":
        return {"accel_body": v[:3].copy(), "gyro_body": v[3:].copy()}
    if name == "baro":
        return {"altitude": float(v[0])}
    if name == "altimeter":
        return {"h_surface": float(v[0])}
    if name == "mag":
        return {"heading": float(v[0])}
    if name == "probe":
        return {"alpha": float(v[0]), "beta": float(v[1])}
    raise KeyError(name)


class SensorSuite:
    def __init__(self, specs, seed=0):
        if isinstance(specs, dict):
            specs = list(specs.values())
        self.specs = {sp.name: sp for sp in specs}
        self.rng = np.random.default_rng(seed)
        self._next = {n: None for n in self.specs}
        self._last = {n: None for n in self.specs}
        self._bias = {n: self._vec(sp.bias, n) for n, sp in self.specs.items()}
        self._pending = {n: deque() for n in self.specs}

    @property
    def names(self):
        return list(self.specs)

    @staticmethod
    def _n(name):
        return _CHANNELS[name]

    def _vec(self, val, name):
        return np.array(np.broadcast_to(np.asarray(val, dtype=float), (self._n(name),)))

    def measure(self, truth, t):
        out = {}
        for name, sp in self.specs.items():
            nxt = self._next[name]
            if nxt is not None and t < nxt - 1e-9:
                self._release(name, t, out)
                continue
            period = 1.0 / sp.rate_hz
            last = self._last[name]
            dt = period if last is None else max(t - last, 0.0)
            n = nxt + period if nxt is not None else t + period
            self._next[name] = n if n > t - 1e-9 else t + period
            self._last[name] = t
            walk = self._vec(sp.bias_walk, name)
            self._bias[name] = self._bias[name] + walk * np.sqrt(dt) * self.rng.standard_normal(self._n(name))
            noise = self._vec(sp.noise, name)
            v = _ideal(name, truth) + self._bias[name] + noise * self.rng.standard_normal(self._n(name))
            if sp.range is not None and not (sp.range[0] <= v[0] <= sp.range[1]):
                self._release(name, t, out)
                continue
            self._pending[name].append((t + sp.delay_s, v))
            self._release(name, t, out)
        return out

    def _release(self, name, t, out):
        q = self._pending[name]
        got = None
        while q and q[0][0] <= t + 1e-9:
            got = q.popleft()[1]
        if got is not None:
            out[name] = _pack(name, got)

    def power_w(self):
        return float(sum(_POWER_MASS[n][0] for n in self.specs))

    def mass_g(self):
        return float(sum(_POWER_MASS[n][1] for n in self.specs))

    @classmethod
    def minimal(cls, seed=0):
        return cls([default_spec(n) for n in ("gnss", "pitot", "imu", "baro")], seed)

    @classmethod
    def standard(cls, seed=0):
        return cls([default_spec(n) for n in ("gnss", "pitot", "imu", "baro", "altimeter", "mag")], seed)

    @classmethod
    def full(cls, seed=0):
        return cls([default_spec(n) for n in ("rtk", "pitot", "imu", "baro", "altimeter", "mag", "probe")], seed)
